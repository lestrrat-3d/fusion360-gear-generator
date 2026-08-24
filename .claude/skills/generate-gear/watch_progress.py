#!/usr/bin/env python3
"""Watch a progress-heartbeat log and block until it reaches a verdict.

`/generate-gear` step 4 runs its drafting subagent as a background task. The
subagent is expected to append heartbeat lines of the form
`<unix-epoch-seconds> <milestone>` to a log file, where milestone is one of
`start`, `read:<path>`, `draft:written`, `check:<n>:pass`, `check:<n>:fail`,
or `done`. This script watches that log so the orchestrator does not have to
poll it by hand: call it once after launching the subagent, and if it exits
with status RUNNING, call the identical command again — one foreground call
per wait, repeated until the exit code is not RUNNING.

Exit codes:
  0  DONE         a `done` heartbeat is present; the subagent finished.
  1  NO_START     no heartbeat at all, and the launch window has passed.
  2  STALLED      the run started, then the log stopped growing too long.
  3  MAX_RUNTIME  still alive and heartbeating, but past the absolute ceiling.
  4  RUNNING      healthy, still working; this invocation's budget elapsed.
                  Call the watcher again, unchanged.
  5  ANOMALY      the log vanished/became unreadable mid-watch, or a stale
                  pre-existing log was detected at first read.
  64 USAGE        bad arguments.
"""
import argparse
import collections
import os
import re
import sys
import time


EXIT_DONE = 0
EXIT_NO_START = 1
EXIT_STALLED = 2
EXIT_MAX_RUNTIME = 3
EXIT_RUNNING = 4
EXIT_ANOMALY = 5
EXIT_USAGE = 64

STATUS_NAMES = {EXIT_DONE: 'DONE', EXIT_NO_START: 'NO_START', EXIT_STALLED: 'STALLED',
                EXIT_MAX_RUNTIME: 'MAX_RUNTIME', EXIT_RUNNING: 'RUNNING',
                EXIT_ANOMALY: 'ANOMALY'}

DEFAULT_LAUNCH_WINDOW = 300      # SKILL.md "~5 minutes"
DEFAULT_STALL_WINDOW = 600       # SKILL.md ">10 minutes"
DEFAULT_MAX_RUNTIME = 2700       # 45 min ceiling; healthy round is 10-15 min
DEFAULT_BUDGET = 540             # 9 min, inside the Bash tool's 10-min cap
DEFAULT_POLL_INTERVAL = 5
DEFAULT_TAIL = 15
STALE_SLACK = 60
SKEW_SLACK = 120

Heartbeat = collections.namedtuple('Heartbeat', 'epoch milestone kind raw parsed')
LogState = collections.namedtuple('LogState', 'exists size mtime lines')
Summary = collections.namedtuple(
    'Summary', 'heartbeats unparsed counts first_epoch last_epoch last_raw saw_done start_count')
Windows = collections.namedtuple(
    'Windows', 'launch stall max_runtime budget poll tail allow_stale')

_CHECK_RE = re.compile(r'^check:\d+:(pass|fail)$', re.IGNORECASE)
_READ_RE = re.compile(r'^read:', re.IGNORECASE)


class RealClock(object):
    """Wall clock. Every time read and every wait in this module goes through a clock
    object so the stall logic can be tested without real sleeping."""
    def now(self):
        return time.time()

    def sleep(self, seconds):
        time.sleep(seconds)


def parse_epoch(token):
    """Strip quotes/brackets; 9-11 digits -> seconds; 12-14 digits -> ms/1000; else None."""
    token = token.strip()
    if token.startswith('[') and token.endswith(']'):
        token = token[1:-1]
    token = token.strip('[]')
    if not token.isdigit():
        return None
    length = len(token)
    if 9 <= length <= 11:
        return int(token)
    if 12 <= length <= 14:
        return int(token) // 1000
    return None


def classify(milestone):
    """Classify a milestone field into one of: start, read, draft:written,
    check:pass, check:fail, done, other. Case-insensitive, by prefix."""
    lower = milestone.lower()
    if lower == 'start':
        return 'start'
    if lower.startswith('read:'):
        return 'read'
    if lower.startswith('draft:'):
        return 'draft:written'
    m = _CHECK_RE.match(lower)
    if m:
        return 'check:pass' if m.group(1) == 'pass' else 'check:fail'
    if lower == 'done':
        return 'done'
    return 'other'


def _looks_like_milestone(token):
    """True if a bare (timestamp-less) token is a recognised milestone."""
    lower = token.lower()
    if lower in ('start', 'done', 'draft:written'):
        return True
    if lower.startswith('read:'):
        return True
    if _CHECK_RE.match(lower):
        return True
    return False


def parse_line(raw):
    """Parse one raw log line into a Heartbeat, or None if it is blank."""
    line = raw.rstrip('\r\n')
    line = line.strip()
    if not line:
        return None
    if len(line) >= 2 and line[0] == line[-1] and line[0] in ('"', "'"):
        line = line[1:-1].strip()

    parts = line.split(None, 1)
    if len(parts) == 2:
        ts_token, milestone = parts
        epoch = parse_epoch(ts_token)
        kind = classify(milestone)
        parsed = epoch is not None or kind != 'other'
        return Heartbeat(epoch=epoch, milestone=milestone, kind=kind, raw=raw, parsed=parsed)

    # single field
    token = parts[0] if parts else ''
    if _looks_like_milestone(token):
        kind = classify(token)
        return Heartbeat(epoch=None, milestone=token, kind=kind, raw=raw, parsed=True)

    return Heartbeat(epoch=None, milestone=token, kind='other', raw=raw, parsed=False)


def summarize(lines):
    """Fold parse_line over the lines into a Summary. Pure."""
    heartbeats = 0
    unparsed = 0
    counts = collections.Counter()
    first_epoch = None
    last_epoch = None
    last_raw = None
    saw_done = False
    start_count = 0

    for raw in lines:
        hb = parse_line(raw)
        if hb is None:
            continue
        last_raw = raw.rstrip('\r\n').strip()
        if not hb.parsed:
            unparsed += 1
            continue
        heartbeats += 1
        counts[hb.kind] += 1
        if hb.kind == 'start':
            start_count += 1
        if hb.kind == 'done':
            saw_done = True
        if hb.epoch is not None:
            if first_epoch is None:
                first_epoch = hb.epoch
            last_epoch = hb.epoch

    return Summary(heartbeats=heartbeats, unparsed=unparsed, counts=counts,
                    first_epoch=first_epoch, last_epoch=last_epoch, last_raw=last_raw,
                    saw_done=saw_done, start_count=start_count)


def read_log(path):
    """Stat + read the log. Never raises; missing/unreadable yields exists=False."""
    try:
        st = os.stat(path)
    except OSError:
        return LogState(exists=False, size=0, mtime=None, lines=[])
    try:
        with open(path, 'r', errors='replace') as f:
            text = f.read()
    except OSError:
        return LogState(exists=False, size=0, mtime=None, lines=[])
    lines = text.splitlines()
    return LogState(exists=True, size=st.st_size, mtime=st.st_mtime, lines=lines)


def anchor_path(log_path):
    return log_path + '.watch'


def load_anchor(log_path, clock):
    """Read the sidecar's single integer; on missing/garbage write now() and
    return created=True."""
    path = anchor_path(log_path)
    try:
        with open(path, 'r') as f:
            content = f.read().strip()
        epoch = int(content)
        return epoch, False
    except (OSError, ValueError):
        epoch = int(clock.now())
        with open(path, 'w') as f:
            f.write('%d\n' % epoch)
        return epoch, True


def reset_anchor(log_path):
    try:
        os.remove(anchor_path(log_path))
    except OSError:
        pass


def is_stale_log(state, summary, reference):
    """A leftover log from a previous run predates this watch. Pure."""
    if not state.exists or not state.lines:
        return False
    if summary.first_epoch is not None:
        return (reference - summary.first_epoch) > STALE_SLACK
    if state.mtime is not None:
        return (reference - state.mtime) > STALE_SLACK
    return False


def evaluate(now, reference, last_activity, state, summary, windows, ever_existed):
    """Decide the status. Pure, no I/O, no clock. Precedence per design §4.4."""
    if ever_existed and not state.exists:
        return EXIT_ANOMALY, 'the progress log existed and is now missing or unreadable'

    if summary.saw_done:
        return EXIT_DONE, "a 'done' heartbeat is present"

    launch_evidence = summary.start_count > 0 or summary.heartbeats > 0
    if not launch_evidence and (now - reference) > windows.launch:
        return EXIT_NO_START, 'no heartbeat within the launch window'

    if launch_evidence and (now - last_activity) > windows.stall:
        return EXIT_STALLED, 'no log growth within the stall window'

    if (now - reference) > windows.max_runtime:
        return EXIT_MAX_RUNTIME, 'past the max-runtime ceiling'

    return EXIT_RUNNING, 'healthy'


def notes_for(state, summary):
    """Advisory note: lines. Pure."""
    notes = []
    launch_evidence = summary.start_count > 0 or summary.heartbeats > 0
    if launch_evidence and summary.start_count == 0:
        notes.append("note: no explicit 'start' line; treating the first heartbeat as the launch.")
    if summary.start_count > 1:
        notes.append("note: %d 'start' lines — the log may hold more than one run." % summary.start_count)
    if summary.last_epoch is not None and state.mtime is not None:
        skew = abs(summary.last_epoch - state.mtime)
        if skew > SKEW_SLACK:
            notes.append(
                "note: last line's timestamp is %ds from the file's mtime — "
                "timestamps in this log may be unreliable." % int(skew))
    if summary.unparsed > 0:
        notes.append('note: %d unparsed line(s) present.' % summary.unparsed)
    return notes


_NEXT_LINES = {
    EXIT_NO_START: 'next: the launch never began (e.g. an unanswered approval gate) — '
                   'stop and surface it to the user.',
    EXIT_STALLED: 'next: the agent stalled — stop the run, relaunch once, and only then '
                  'involve the user.',
    EXIT_MAX_RUNTIME: 'next: past the ceiling with the run still alive — stop it and surface '
                       'the elapsed time to the user.',
    EXIT_ANOMALY: 'next: the progress log is not in the state this watch assumed — stop and '
                  'inspect before relaunching.',
    EXIT_RUNNING: 'next: still healthy — run this same command again.',
}


def _format_hms(seconds):
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return '%02d:%02d:%02d' % (h, m, s)


def render(out, code, reason, log_path, reference, now, last_activity, state, summary, windows):
    """Produce the report block. Pure apart from writing to out."""
    status = STATUS_NAMES.get(code, '?')
    out.write('watch_progress: %s (exit %d)\n' % (status, code))
    out.write('log:        %s\n' % log_path)
    out.write('elapsed:    %s  (reference %d, now %d)\n' % (
        _format_hms(now - reference), int(reference), int(now)))

    if summary.last_raw is not None:
        if summary.last_epoch is not None:
            ago = max(0, now - summary.last_epoch)
        elif state.mtime is not None:
            ago = max(0, now - state.mtime)
        else:
            ago = max(0, now - last_activity)
        out.write('last line:  %s  (%ds ago)\n' % (summary.last_raw, int(ago)))
    else:
        out.write('last line:  (none)\n')

    out.write('lines:      %d heartbeats, %d unparsed\n' % (summary.heartbeats, summary.unparsed))

    if summary.counts:
        parts = ['%s=%d' % (kind, count) for kind, count in sorted(summary.counts.items())]
        out.write('milestones: %s\n' % ' '.join(parts))
    else:
        out.write('milestones: (none)\n')

    for note in notes_for(state, summary):
        out.write(note + '\n')

    if code != EXIT_DONE and code in _NEXT_LINES:
        out.write(_NEXT_LINES[code] + '\n')

    tail = state.lines[-windows.tail:] if state.lines else []
    out.write('--- tail (last %d of %d lines) ---\n' % (len(tail), len(state.lines)))
    for line in tail:
        out.write(line + '\n')


def watch(log_path, windows, clock, out, since=None, reset=False):
    """The wait loop. Blocks (via clock.sleep) until a terminal status or the
    budget elapses."""
    if reset:
        reset_anchor(log_path)
    if since is not None:
        reference, created = since, False
    else:
        reference, created = load_anchor(log_path, clock)

    deadline = clock.now() + windows.budget
    ever_existed = False
    last_activity = None
    prev_signature = None
    first_read = True

    while True:
        state = read_log(log_path)
        if state.exists:
            ever_existed = True
            signature = (state.size, state.mtime)
            if last_activity is None:
                last_activity = max(reference, state.mtime)
            elif signature != prev_signature:
                last_activity = clock.now()
            prev_signature = signature
        else:
            if last_activity is None:
                last_activity = reference

        summary = summarize(state.lines)

        if (first_read and created and not windows.allow_stale
                and is_stale_log(state, summary, reference)):
            now = clock.now()
            render(out, EXIT_ANOMALY, 'stale log predates this watch', log_path, reference, now,
                   last_activity, state, summary, windows)
            return EXIT_ANOMALY
        first_read = False

        now = clock.now()
        code, reason = evaluate(now, reference, last_activity, state, summary, windows, ever_existed)
        if code != EXIT_RUNNING:
            render(out, code, reason, log_path, reference, now, last_activity, state, summary, windows)
            return code

        if now >= deadline:
            render(out, EXIT_RUNNING, reason, log_path, reference, now, last_activity, state,
                   summary, windows)
            return EXIT_RUNNING

        clock.sleep(min(windows.poll, max(0, deadline - now)))


class _ArgumentParser(argparse.ArgumentParser):
    """Bad CLI usage exits EXIT_USAGE (sysexits.h EX_USAGE), not argparse's default 2,
    so it can never be mistaken for one of this script's own status codes."""
    def error(self, message):
        self.print_usage(sys.stderr)
        sys.stderr.write('%s: error: %s\n' % (self.prog, message))
        raise SystemExit(EXIT_USAGE)


def build_parser():
    parser = _ArgumentParser(
        prog='watch_progress.py',
        description='Watch a progress-heartbeat log and block until it reaches a verdict.')
    parser.add_argument('log', metavar='LOG', help='Path to the progress log to watch.')
    parser.add_argument('--launch-window', type=int, default=DEFAULT_LAUNCH_WINDOW)
    parser.add_argument('--stall-window', type=int, default=DEFAULT_STALL_WINDOW)
    parser.add_argument('--max-runtime', type=int, default=DEFAULT_MAX_RUNTIME)
    parser.add_argument('--budget', type=int, default=DEFAULT_BUDGET)
    parser.add_argument('--poll-interval', type=int, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument('--tail', type=int, default=DEFAULT_TAIL)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--since', type=int, default=None)
    parser.add_argument('--reset', action='store_true')
    parser.add_argument('--allow-stale', action='store_true')
    return parser


def main(argv, clock=None, out=None):
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as e:
        return e.code if e.code is not None else EXIT_USAGE

    if clock is None:
        clock = RealClock()
    if out is None:
        out = sys.stdout

    for name, value in (('--launch-window', args.launch_window), ('--stall-window', args.stall_window),
                         ('--max-runtime', args.max_runtime), ('--budget', args.budget),
                         ('--poll-interval', args.poll_interval), ('--tail', args.tail)):
        if value < 0:
            sys.stderr.write('watch_progress: %s must not be negative\n' % name)
            return EXIT_USAGE

    budget = 0 if args.once else args.budget

    windows = Windows(launch=args.launch_window, stall=args.stall_window,
                       max_runtime=args.max_runtime, budget=budget,
                       poll=args.poll_interval, tail=args.tail, allow_stale=args.allow_stale)

    return watch(args.log, windows, clock, out, since=args.since, reset=args.reset)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
