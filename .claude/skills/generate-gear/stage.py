#!/usr/bin/env python3
"""Stage drafted gear artifacts into the working tree.

`compile-gear` and `emit-gear` used to move drafted files by hand with shell
commands. That left three hazards: a partial copy on a shell-loop failure, a
stale `.go` file from an earlier draft round still compiled into
`proof/<gear>/` and silently credited to the current draft, and an untracked
first-time proof that fails `check_compile`'s tracked-or-committed gate for
reasons that have nothing to do with the draft. This script replaces the
shell moves with one mechanical, atomic, idempotent placement per target.

Four subcommands, one per artifact this pipeline stages plus one that
places a whole stage's artifacts together:

  stage.py <gear> proof  [--run] [--no-index] [--force] [--dry-run]
      Copies every `*.go` file from `.tmp/<gear>-proof/` into `proof/<gear>/`,
      deleting any `.go` file already there that the draft no longer
      produces. Indexes the result (`git add -A -- proof/<gear>`) unless
      `--no-index` is given, so a first-time proof passes the
      tracked-or-committed check. `--run` then runs `bash proof/run.sh`.

  stage.py <gear> steps  [--force] [--dry-run]
      Copies `.tmp/<gear>.steps.md` to `spec/<gear>/steps.md`.

  stage.py <gear> module [--force] [--dry-run]
      Copies `.tmp/<gear>.generated.py` to `lib/geargen/<gear>.py`.

  stage.py <gear> compile [--no-index] [--force] [--dry-run]
      Places the compile stage's two artifacts in one call: `steps` then
      `proof`, exactly as the two single-target commands would, refusing
      both if it would refuse either. There is no `--run`; the compile
      gate runner (`run_compile_gates.py`) runs the proof.

`--root DIR` (default `.`) is the repo root; every path above is relative to
it. `--force` overwrites a destination file that is neither this script's
own prior output nor clean against HEAD. `--dry-run` validates and reports
what would happen, writing nothing.

The source is always copied, never deleted, so every subcommand is
idempotent: running it again over the same draft reports every file
unchanged.

Usage:
    python3 stage.py [--root DIR] <gear> {proof,steps,module,compile} [options]

Run from the repo root.

Exit 0 = everything asked for was done (and `--run`, if given, passed).
Exit 1 = placement succeeded, but `bash proof/run.sh` failed (only
         `proof --run` can return this).
Exit 2 = refused; nothing was written, or everything written was rolled
         back.
"""
import argparse
import collections
import hashlib
import json
import os
import re
import subprocess
import sys

GEAR_NAME = re.compile(r'[a-z][a-z0-9_]*\Z')


class Refusal(Exception):
    """Raised to refuse a placement. The message is printed to stderr; always exit 2."""


Plan = collections.namedtuple(
    'Plan', 'target gear source destination sources_are_dir')
Actions = collections.namedtuple('Actions', 'write unchanged prune extra')
PlannedTarget = collections.namedtuple(
    'PlannedTarget', 'plan sources existing actions')

COMPILE_TARGETS = ('steps', 'proof')


def _joined(root, *parts):
    return os.path.normpath(os.path.join(root, *parts))


def gear_paths(root, gear, target):
    """Build the source and destination paths for one target.

    Pure path arithmetic: validates the gear name and raises `Refusal`
    otherwise, but performs no I/O.
    """
    if not GEAR_NAME.match(gear):
        raise Refusal(
            "%r is not a valid gear name (expected lowercase letters, digits or "
            "underscore, starting with a letter)" % gear)
    if target == 'proof':
        source = _joined(root, '.tmp', '%s-proof' % gear)
        destination = _joined(root, 'proof', gear)
        sources_are_dir = True
    elif target == 'steps':
        source = _joined(root, '.tmp', '%s.steps.md' % gear)
        destination = _joined(root, 'spec', gear, 'steps.md')
        sources_are_dir = False
    elif target == 'module':
        source = _joined(root, '.tmp', '%s.generated.py' % gear)
        destination = _joined(root, 'lib', 'geargen', '%s.py' % gear)
        sources_are_dir = False
    else:
        raise Refusal("unknown target %r" % target)
    return Plan(target, gear, source, destination, sources_are_dir)


def read_sources(plan):
    """Phase 1 read of the draft. Returns an ordered dict of name to bytes.

    Applies the source refusals (missing source, empty source, a zero-byte
    `.go` file) and the draft-directory refusal (a non-`.go` regular file or
    a subdirectory in the draft directory).
    """
    sources = collections.OrderedDict()
    if plan.sources_are_dir:
        if not os.path.isdir(plan.source):
            raise Refusal(
                "%s does not exist, so there is nothing to place" % plan.source)
        entries = sorted(os.listdir(plan.source))
        directories = [e for e in entries
                       if os.path.isdir(os.path.join(plan.source, e))]
        if directories:
            full = os.path.join(plan.source, directories[0])
            raise Refusal(
                "%s is a subdirectory of the draft, and only .go files belong there"
                % full)
        go_files = [e for e in entries if e.endswith('.go')]
        others = [e for e in entries if not e.endswith('.go')]
        if others:
            full = os.path.join(plan.source, others[0])
            raise Refusal(
                "%s is not a .go file, and only .go files belong in the draft "
                "directory" % full)
        if not go_files:
            raise Refusal(
                "%s contains no .go file, so there is nothing to place" % plan.source)
        for name in go_files:
            full = os.path.join(plan.source, name)
            with open(full, 'rb') as fh:
                data = fh.read()
            if not data:
                raise Refusal("%s is zero bytes" % full)
            sources[name] = data
    else:
        if not os.path.isfile(plan.source):
            raise Refusal(
                "%s does not exist, so there is nothing to place" % plan.source)
        with open(plan.source, 'rb') as fh:
            data = fh.read()
        if not data:
            raise Refusal("%s is zero bytes" % plan.source)
        sources[os.path.basename(plan.destination)] = data
    return sources


def existing_destination(plan):
    """Read what is in the destination now, restricted to `*.go` for `proof`.

    Returns name to bytes. Raises `Refusal` if a destination path is a
    directory or a symlink where a file is expected, or if the destination's
    parent directory does not exist (a missing `proof/<gear>/` is the one
    exception: it is created when absent, so it is not checked here).
    """
    existing = collections.OrderedDict()
    if plan.sources_are_dir:
        if os.path.islink(plan.destination):
            raise Refusal("%s exists but is a symlink" % plan.destination)
        if os.path.exists(plan.destination) and not os.path.isdir(plan.destination):
            raise Refusal("%s exists but is not a directory" % plan.destination)
        if os.path.isdir(plan.destination):
            for name in sorted(os.listdir(plan.destination)):
                if not name.endswith('.go'):
                    continue
                full = os.path.join(plan.destination, name)
                if os.path.islink(full):
                    raise Refusal("%s exists but is a symlink" % full)
                if os.path.isdir(full):
                    raise Refusal("%s exists but is a directory" % full)
                with open(full, 'rb') as fh:
                    existing[name] = fh.read()
        return existing

    parent = os.path.dirname(plan.destination)
    if not os.path.isdir(parent):
        raise Refusal(
            "%s does not exist, which likely means %r is the wrong gear name"
            % (parent, plan.gear))
    if os.path.exists(plan.destination):
        if os.path.islink(plan.destination):
            raise Refusal("%s exists but is a symlink" % plan.destination)
        if os.path.isdir(plan.destination):
            raise Refusal("%s exists but is a directory" % plan.destination)
        with open(plan.destination, 'rb') as fh:
            existing[os.path.basename(plan.destination)] = fh.read()
    return existing


def destination_extras(plan):
    """Non-`.go` regular files already sitting in a `proof` destination.

    Reported in the summary, never touched: the destination directory may be
    legitimately extended by a human (testdata, a README), so only the
    extension the pipeline actually generates is ever pruned.
    """
    if not plan.sources_are_dir or not os.path.isdir(plan.destination):
        return []
    extras = []
    for name in sorted(os.listdir(plan.destination)):
        if name.endswith('.go'):
            continue
        full = os.path.join(plan.destination, name)
        if os.path.isfile(full):
            extras.append(name)
    return extras


def receipt_path(root, gear, target):
    return _joined(root, '.tmp', 'stage', '%s.%s.json' % (gear, target))


def load_receipt(root, gear, target):
    """The map of relative name to SHA-256 this script wrote last time it succeeded.

    A missing or unreadable receipt is `{}`, never an error.
    """
    path = receipt_path(root, gear, target)
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save_receipt(root, gear, target, sources):
    """Rewrite the receipt to record the bytes this run placed."""
    path = receipt_path(root, gear, target)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    receipt = {name: hashlib.sha256(data).hexdigest() for name, data in sources.items()}
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(receipt, fh, sort_keys=True, indent=2)
        fh.write('\n')


def git_is_clean(root, relpath):
    """Whether `<relpath>` is clean against HEAD, or `None` if that cannot be known.

    `None` means the root is not a git work tree (or git is unavailable), so
    the dirty-destination check degrades to "safe" rather than to a refusal.
    """
    try:
        result = subprocess.run(
            ['git', 'status', '--porcelain', '--', relpath],
            cwd=root, capture_output=True, text=True)
    except FileNotFoundError:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip() == ''


def classify_destination(plan, existing, receipt, force, root):
    """Raise `Refusal` on the first existing file that is neither ours nor clean.

    A file is safe to replace when its SHA-256 matches the receipt from this
    script's last successful run, or when it is clean against HEAD. `root` is
    needed here (beyond what `plan` carries) to run `git status` from the
    right working tree; see the deviations noted in the PR description.
    """
    if force:
        return
    warned = False
    for name in sorted(existing):
        data = existing[name]
        digest = hashlib.sha256(data).hexdigest()
        if receipt.get(name) == digest:
            continue
        full = (os.path.join(plan.destination, name)
                if plan.sources_are_dir else plan.destination)
        relpath = os.path.relpath(full, root)
        clean = git_is_clean(root, relpath)
        if clean is None:
            if not warned:
                print("stage: %s is not a git work tree; skipping the clean-tree check"
                      % root, file=sys.stderr)
                warned = True
            continue
        if clean:
            continue
        raise Refusal(
            "%s is neither this script's own output nor clean against HEAD; "
            "commit it, revert it, or pass --force" % full)


def plan_actions(sources, existing):
    """Diff sources against what already exists. Pure, no I/O.

    `prune` only ever contains names present in `existing` but absent from
    `sources`; the caller only builds a nonempty `existing` for `proof`, so
    this is naturally scoped to that target.
    """
    write = []
    unchanged = []
    for name, data in sources.items():
        if existing.get(name) == data:
            unchanged.append(name)
        else:
            write.append(name)
    prune = [name for name in existing if name not in sources]
    return Actions(sorted(write), sorted(unchanged), sorted(prune), [])


def plan_target(root, gear, target, force):
    """Everything one target does before `apply_actions`. Raises `Refusal`.

    Pulling phase 1 out of `main` is what lets a composite target plan every
    child before any of them writes, so a refusal anywhere leaves the whole
    working tree untouched.
    """
    plan = gear_paths(root, gear, target)
    sources = read_sources(plan)
    existing = existing_destination(plan)
    extras = destination_extras(plan)
    receipt = load_receipt(root, gear, plan.target)
    classify_destination(plan, existing, receipt, force, root)
    actions = plan_actions(sources, existing)._replace(extra=extras)
    return PlannedTarget(plan, sources, existing, actions)


def _temp_path(path):
    directory = os.path.dirname(path) or '.'
    base = os.path.basename(path)
    n = 0
    while True:
        candidate = os.path.join(directory, '.%s.stage-tmp.%d.%d' % (base, os.getpid(), n))
        if not os.path.exists(candidate):
            return candidate
        n += 1


def _atomic_write(path, data):
    tmp = _temp_path(path)
    with open(tmp, 'wb') as fh:
        fh.write(data)
    try:
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _restore(full, previous):
    """Best-effort restore of one file to its pre-run state during rollback."""
    if previous is None:
        try:
            os.unlink(full)
        except OSError:
            pass
    else:
        try:
            _atomic_write(full, previous)
        except OSError:
            pass


def apply_actions(plan, actions, sources, existing):
    """Phase 2: write, then prune, restoring everything touched on any failure.

    Each write goes to a temp name in the destination directory and is moved
    into place with `os.replace`, so no half-written file is ever visible.
    If any write or unlink raises, every file this call already touched is
    restored from the in-memory `existing` snapshot before the exception is
    re-raised.
    """
    if plan.sources_are_dir:
        os.makedirs(plan.destination, exist_ok=True)

    touched = []
    try:
        for name in actions.write:
            full = (os.path.join(plan.destination, name)
                    if plan.sources_are_dir else plan.destination)
            previous = existing.get(name)
            _atomic_write(full, sources[name])
            touched.append((full, previous))
        for name in actions.prune:
            full = (os.path.join(plan.destination, name)
                    if plan.sources_are_dir else plan.destination)
            previous = existing.get(name)
            os.unlink(full)
            touched.append((full, previous))
    except Exception:
        for full, previous in reversed(touched):
            _restore(full, previous)
        raise


def rollback_target(planned):
    """Undo a target that `apply_actions` already completed.

    `apply_actions` unwinds its own target when it fails part way through, so
    this exists only for a composite run: when a later child refuses to
    place, an earlier child that already succeeded has to go back to the
    bytes it replaced (or be removed again, when there were none).
    """
    plan = planned.plan
    for name in list(planned.actions.write) + list(planned.actions.prune):
        full = (os.path.join(plan.destination, name)
                if plan.sources_are_dir else plan.destination)
        _restore(full, planned.existing.get(name))


def index_paths(root, plan):
    """`git add -A -- proof/<gear>`. Returns False (with a printed note) if git fails."""
    relpath = os.path.relpath(plan.destination, root)
    try:
        result = subprocess.run(
            ['git', 'add', '-A', '--', relpath], cwd=root,
            capture_output=True, text=True)
    except FileNotFoundError:
        print("stage: git is not available; %s was not indexed" % relpath, file=sys.stderr)
        return False
    if result.returncode != 0:
        print("stage: git add failed for %s; not indexed (%s)"
              % (relpath, result.stderr.strip()), file=sys.stderr)
        return False
    print("stage: indexed %s" % relpath)
    return True


def run_proof(root):
    """Run `bash proof/run.sh`, passing its output through, and return its exit code."""
    print("stage: running bash proof/run.sh")
    result = subprocess.run(
        ['bash', 'proof/run.sh'], cwd=root, capture_output=True, text=True)
    if result.stdout:
        sys.stdout.write(result.stdout)
    if result.stderr:
        sys.stderr.write(result.stderr)
    return result.returncode


def report(plan, actions, dry_run, sources):
    """Print the action lines and the summary line, in sorted path order."""
    prefix = 'would ' if dry_run else ''
    write_verb = 'write' if dry_run else 'wrote'
    prune_verb = 'prune' if dry_run else 'pruned'

    def full_path(name):
        return (os.path.join(plan.destination, name)
                if plan.sources_are_dir else plan.destination)

    for name in actions.write:
        print("stage: %s%s %s (%d bytes)"
              % (prefix, write_verb, full_path(name), len(sources[name])))
    for name in actions.unchanged:
        print("stage: unchanged %s" % full_path(name))
    for name in actions.prune:
        print("stage: %s%s %s (the draft no longer produces it)"
              % (prefix, prune_verb, full_path(name)))
    for name in actions.extra:
        print("stage: %s left alone (not produced by the draft)" % full_path(name))

    if plan.target == 'proof':
        print("stage: proof OK (%d written, %d unchanged, %d pruned)"
              % (len(actions.write), len(actions.unchanged), len(actions.prune)))
    elif actions.write:
        print("stage: %s OK (%d written)" % (plan.target, len(actions.write)))
    else:
        print("stage: %s OK (%d unchanged)" % (plan.target, len(actions.unchanged)))


def build_parser():
    parser = argparse.ArgumentParser(
        prog='stage.py',
        description='Stage a drafted gear artifact (proof, steps or module), or a whole '
                     'stage of them, into the working tree.')
    parser.add_argument('--root', default='.', help='repo root (default .)')
    parser.add_argument('gear', help='gear name, e.g. spurgear')
    sub = parser.add_subparsers(dest='target', required=True)

    proof = sub.add_parser('proof', help='place proof/<gear>/ from .tmp/<gear>-proof/')
    proof.add_argument('--run', action='store_true',
                        help='run bash proof/run.sh after a successful placement')
    proof.add_argument('--no-index', action='store_true',
                        help='skip `git add -A -- proof/<gear>`')
    proof.add_argument('--force', action='store_true',
                        help='overwrite a destination file that is not clean or ours')
    proof.add_argument('--dry-run', action='store_true',
                        help='validate and report; write nothing')

    steps = sub.add_parser('steps', help='place spec/<gear>/steps.md from .tmp/<gear>.steps.md')
    steps.add_argument('--force', action='store_true',
                        help='overwrite a destination file that is not clean or ours')
    steps.add_argument('--dry-run', action='store_true',
                        help='validate and report; write nothing')

    module = sub.add_parser('module',
                             help='place lib/geargen/<gear>.py from .tmp/<gear>.generated.py')
    module.add_argument('--force', action='store_true',
                         help='overwrite a destination file that is not clean or ours')
    module.add_argument('--dry-run', action='store_true',
                         help='validate and report; write nothing')

    compile_ = sub.add_parser(
        'compile',
        help='place the compile stage: steps then proof, refusing both if either refuses')
    compile_.add_argument('--no-index', action='store_true',
                           help='skip `git add -A -- proof/<gear>`')
    compile_.add_argument('--force', action='store_true',
                           help='overwrite a destination file that is not clean or ours')
    compile_.add_argument('--dry-run', action='store_true',
                           help='validate and report; write nothing')

    return parser


def run_compile(root, args):
    """The `compile` target: place `steps` and `proof` together, or neither.

    Every child is planned before any of them writes, so a refusal in either
    one leaves the working tree exactly as it was. Once writing starts, a
    failure in the second child rolls the first one back as well, and the
    receipts and the proof indexing are the same ones the two single-target
    commands write, so a later single-target run still recognises its own
    output.
    """
    try:
        planned = [plan_target(root, args.gear, target, args.force)
                   for target in COMPILE_TARGETS]
    except Refusal as exc:
        print("stage: %s" % exc, file=sys.stderr)
        return 2

    if args.dry_run:
        for item in planned:
            report(item.plan, item.actions, True, item.sources)
        return 0

    applied = []
    for item in planned:
        try:
            apply_actions(item.plan, item.actions, item.sources, item.existing)
        except Exception as exc:
            for done in reversed(applied):
                rollback_target(done)
            print("stage: compile: %s placement failed and the whole compile placement "
                  "was rolled back (%s)" % (item.plan.target, exc), file=sys.stderr)
            return 2
        applied.append(item)

    for item in planned:
        save_receipt(root, args.gear, item.plan.target, item.sources)

    for item in planned:
        if item.plan.target == 'proof' and not getattr(args, 'no_index', False):
            index_paths(root, item.plan)

    for item in planned:
        report(item.plan, item.actions, False, item.sources)
    print("stage: compile OK (steps + proof)")
    return 0


def main(argv):
    parser = build_parser()
    args = parser.parse_args(argv[1:])
    root = args.root

    if args.target == 'compile':
        return run_compile(root, args)

    try:
        plan, sources, existing, actions = plan_target(
            root, args.gear, args.target, args.force)
    except Refusal as exc:
        print("stage: %s" % exc, file=sys.stderr)
        return 2

    if args.dry_run:
        report(plan, actions, True, sources)
        return 0

    try:
        apply_actions(plan, actions, sources, existing)
    except Exception as exc:
        print("stage: %s: placement failed and was rolled back (%s)" % (plan.target, exc),
              file=sys.stderr)
        return 2

    save_receipt(root, args.gear, plan.target, sources)

    if plan.target == 'proof' and not getattr(args, 'no_index', False):
        index_paths(root, plan)

    report(plan, actions, False, sources)

    if getattr(args, 'run', False):
        code = run_proof(root)
        if code != 0:
            print("stage: proof run FAILED (exit %d)" % code, file=sys.stderr)
            return 1
        print("stage: proof run OK")

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
