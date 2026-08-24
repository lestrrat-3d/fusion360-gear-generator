#!/usr/bin/env python3
"""Tests for watch_progress.py's parsing, stall-decision, and loop behaviour.

No test sleeps for real: the loop tests inject FakeClock, and RealClock.sleep
is additionally patched out and asserted never called, so a regression that
reintroduces a direct time.sleep fails the suite instead of slowing it down.
"""
import importlib.util, os, tempfile, time, unittest
from pathlib import Path
from unittest import mock
import io

MODULE_PATH = Path(__file__).with_name('watch_progress.py')
SPEC = importlib.util.spec_from_file_location('watch_progress', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeClock(object):
    """Virtual clock. sleep() advances time instead of spending it."""
    def __init__(self, start=None):
        self.t = float(start if start is not None else time.time())
        self.slept = []
    def now(self):
        return self.t
    def sleep(self, seconds):
        self.slept.append(seconds)
        self.t += seconds


def windows(**kw):
    base = dict(launch=MODULE.DEFAULT_LAUNCH_WINDOW, stall=MODULE.DEFAULT_STALL_WINDOW,
                max_runtime=MODULE.DEFAULT_MAX_RUNTIME, budget=MODULE.DEFAULT_BUDGET,
                poll=MODULE.DEFAULT_POLL_INTERVAL, tail=MODULE.DEFAULT_TAIL, allow_stale=False)
    base.update(kw)
    return MODULE.Windows(**base)


class ParseLineTests(unittest.TestCase):
    def test_basic_start(self):
        hb = MODULE.parse_line('1755993212 start')
        self.assertEqual(hb.epoch, 1755993212)
        self.assertEqual(hb.milestone, 'start')
        self.assertEqual(hb.kind, 'start')
        self.assertTrue(hb.parsed)

    def test_read_milestone_kept_whole(self):
        hb = MODULE.parse_line('1755993212 read:spec/spurgear/instructions.md')
        self.assertEqual(hb.milestone, 'read:spec/spurgear/instructions.md')
        self.assertEqual(hb.kind, 'read')

    def test_check_fail(self):
        hb = MODULE.parse_line('1755993212 check:2:fail')
        self.assertEqual(hb.kind, 'check:fail')

    def test_tab_and_space_runs(self):
        hb1 = MODULE.parse_line('1755993212\tdraft:written')
        hb2 = MODULE.parse_line('1755993212   done')
        self.assertEqual(hb1.kind, 'draft:written')
        self.assertEqual(hb2.kind, 'done')

    def test_whitespace_and_crlf_stripped(self):
        hb = MODULE.parse_line('  1755993212 start  \r\n')
        self.assertEqual(hb.epoch, 1755993212)
        self.assertEqual(hb.kind, 'start')

    def test_blank_lines_return_none(self):
        self.assertIsNone(MODULE.parse_line(''))
        self.assertIsNone(MODULE.parse_line('   '))

    def test_millisecond_epoch(self):
        hb = MODULE.parse_line('1755993212000 done')
        self.assertEqual(hb.epoch, 1755993212)

    def test_quoted_line(self):
        hb = MODULE.parse_line('"1755993212 draft:written"')
        self.assertEqual(hb.kind, 'draft:written')
        self.assertTrue(hb.parsed)

    def test_bad_timestamp_keeps_milestone(self):
        hb = MODULE.parse_line('2026-08-24T10:00:00 start')
        self.assertIsNone(hb.epoch)
        self.assertEqual(hb.kind, 'start')
        self.assertTrue(hb.parsed)

    def test_bare_done(self):
        hb = MODULE.parse_line('done')
        self.assertIsNone(hb.epoch)
        self.assertEqual(hb.kind, 'done')
        self.assertTrue(hb.parsed)

    def test_unrecognized_line(self):
        hb = MODULE.parse_line('hello world')
        self.assertEqual(hb.kind, 'other')
        self.assertFalse(hb.parsed)

    def test_bracketed_timestamp(self):
        hb = MODULE.parse_line('[1755993212] start')
        self.assertEqual(hb.epoch, 1755993212)


class SummarizeTests(unittest.TestCase):
    def test_full_log_counts(self):
        base = 1755990000
        lines = ['%d start' % base]
        lines += ['%d read:foo%d' % (base + i + 1, i) for i in range(6)]
        lines += ['%d draft:written' % (base + 7)]
        lines += ['%d check:1:pass' % (base + 8), '%d check:2:pass' % (base + 9)]
        lines += ['%d check:3:fail' % (base + 10)]
        lines += ['%d done' % (base + 11)]
        summary = MODULE.summarize(lines)
        self.assertEqual(summary.counts['start'], 1)
        self.assertEqual(summary.counts['read'], 6)
        self.assertEqual(summary.counts['draft:written'], 1)
        self.assertEqual(summary.counts['check:pass'], 2)
        self.assertEqual(summary.counts['check:fail'], 1)
        self.assertEqual(summary.counts['done'], 1)
        self.assertTrue(summary.saw_done)
        self.assertEqual(summary.first_epoch, base)

    def test_done_in_middle_still_seen(self):
        base = 1755990000
        summary = MODULE.summarize(
            ['%d start' % base, '%d done' % (base + 1), '%d check:1:pass' % (base + 2)])
        self.assertTrue(summary.saw_done)

    def test_two_start_lines(self):
        base = 1755990000
        summary = MODULE.summarize(
            ['%d start' % base, '%d read:x' % (base + 5), '%d start' % (base + 15)])
        self.assertEqual(summary.start_count, 2)
        self.assertEqual(summary.first_epoch, base)

    def test_unparsed_excluded_from_heartbeats(self):
        summary = MODULE.summarize(['1 start', 'garbage line here', 'also not valid !!', '2 done'])
        self.assertEqual(summary.unparsed, 2)
        self.assertEqual(summary.heartbeats, 2)

    def test_no_start_three_reads(self):
        summary = MODULE.summarize(['1 read:a', '2 read:b', '3 read:c'])
        self.assertEqual(summary.counts['start'], 0)
        self.assertEqual(summary.heartbeats, 3)


class EvaluateTests(unittest.TestCase):
    def _state(self, exists=True, lines=None):
        lines = lines or []
        return MODULE.LogState(exists=exists, size=len(''.join(lines)), mtime=0.0, lines=lines)

    def test_evaluate_precedence_table(self):
        # (case #, now, reference, last_activity, lines, ever_existed, windows_kw, expected)
        cases = [
            (18, 60, 0, 0, [], True, {'launch': 300}, MODULE.EXIT_RUNNING),
            (19, 301, 0, 0, [], True, {'launch': 300}, MODULE.EXIT_NO_START),
            (20, 300, 0, 0, [], True, {'launch': 300}, MODULE.EXIT_RUNNING),
            (21, 400, 0, 390, ['0 read:x'], True, {'launch': 300, 'stall': 600}, MODULE.EXIT_RUNNING),
            (22, 601, 0, 0, ['0 start'], True, {'stall': 600}, MODULE.EXIT_STALLED),
            (23, 600, 0, 0, ['0 start'], True, {'stall': 600}, MODULE.EXIT_RUNNING),
            (24, 10000, 0, 5000, ['0 start', '1 done'], True,
             {'stall': 600, 'max_runtime': 2700}, MODULE.EXIT_DONE),
            (25, 2701, 0, 2696, ['0 start'], True, {'max_runtime': 2700, 'stall': 600}, MODULE.EXIT_MAX_RUNTIME),
            (26, 3000, 0, 0, [], True, {'launch': 300, 'max_runtime': 2700}, MODULE.EXIT_NO_START),
            (27, 60, 0, 0, [], True, {}, MODULE.EXIT_ANOMALY),
            (28, 60, 0, 0, [], False, {'launch': 300}, MODULE.EXIT_RUNNING),
            (29, 10, 0, 5, ['0 start', '5 check:1:fail'], True, {}, MODULE.EXIT_RUNNING),
        ]
        for case_num, now, reference, last_activity, lines, ever_existed, kw, expected in cases:
            with self.subTest(case=case_num):
                exists = not (case_num in (27, 28))
                st = self._state(exists=exists, lines=lines)
                summary = MODULE.summarize(lines)
                ws = windows(**kw)
                code, _ = MODULE.evaluate(now, reference, last_activity, st, summary, ws, ever_existed)
                self.assertEqual(code, expected)


class StaleLogTests(unittest.TestCase):
    # anchor/reference epoch used by these cases
    ANCHOR = 1755990000

    def test_30_stale_by_epoch(self):
        lines = ['%d start' % (self.ANCHOR - 3600)]
        state = MODULE.LogState(exists=True, size=10, mtime=self.ANCHOR - 3600.0, lines=lines)
        summary = MODULE.summarize(lines)
        self.assertTrue(MODULE.is_stale_log(state, summary, self.ANCHOR))

    def test_32_stale_by_mtime_when_unparsable(self):
        lines = ['not-a-timestamp start']
        state = MODULE.LogState(exists=True, size=10, mtime=self.ANCHOR - 3600.0, lines=lines)
        summary = MODULE.summarize(lines)
        self.assertTrue(MODULE.is_stale_log(state, summary, self.ANCHOR))

    def test_33_not_stale_within_slack(self):
        lines = ['%d start' % (self.ANCHOR - 5)]
        state = MODULE.LogState(exists=True, size=10, mtime=self.ANCHOR - 5.0, lines=lines)
        summary = MODULE.summarize(lines)
        self.assertFalse(MODULE.is_stale_log(state, summary, self.ANCHOR))


class WatchLoopTests(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self.dir = self._tmpdir.name
        self.log_path = os.path.join(self.dir, 'sample.progress.log')
        patcher = mock.patch.object(MODULE.time, 'sleep')
        self.real_sleep = patcher.start()
        self.addCleanup(patcher.stop)

    def _write(self, clock, lines):
        with open(self.log_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.utime(self.log_path, (clock.t, clock.t))

    def _write_with_mtime(self, lines, mtime):
        with open(self.log_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.utime(self.log_path, (mtime, mtime))

    def test_30_pre_existing_stale_log_is_anomaly(self):
        anchor_epoch = 1755990000
        clock = FakeClock(start=anchor_epoch)
        stale_epoch = anchor_epoch - 3600
        self._write_with_mtime(['%d start' % stale_epoch, '%d done' % stale_epoch], stale_epoch)
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(), clock, out)
        self.assertEqual(code, MODULE.EXIT_ANOMALY)
        self.assertIn('next: the progress log is not in the state this watch assumed', out.getvalue())
        self.assertFalse(self.real_sleep.called)

    def test_31_allow_stale_skips_anomaly(self):
        anchor_epoch = 1755990000
        clock = FakeClock(start=anchor_epoch)
        stale_epoch = anchor_epoch - 3600
        self._write_with_mtime(['%d start' % stale_epoch, '%d done' % stale_epoch], stale_epoch)
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(allow_stale=True), clock, out)
        self.assertEqual(code, MODULE.EXIT_DONE)
        self.assertFalse(self.real_sleep.called)

    def test_34_done_before_third_poll(self):
        clock = FakeClock(start=1000)
        state = {'polls': 0}
        real_sleep_orig = MODULE.time.sleep

        def side_effecting_sleep(seconds):
            state['polls'] += 1
            clock.t += seconds
            if state['polls'] >= 2:
                self._write(clock, ['%d start' % clock.t, '%d done' % clock.t])
        clock.sleep = side_effecting_sleep
        self._write(clock, ['%d start' % clock.t])

        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(poll=1, budget=60), clock, out)
        self.assertEqual(code, MODULE.EXIT_DONE)
        self.assertFalse(self.real_sleep.called)

    def test_35_log_never_appears(self):
        clock = FakeClock(start=1000)
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(launch=10, poll=1, budget=60), clock, out)
        self.assertEqual(code, MODULE.EXIT_NO_START)
        self.assertGreaterEqual(clock.t, 1000 + 11)
        self.assertLess(clock.t, 1000 + 15)
        self.assertFalse(self.real_sleep.called)

    def test_36_one_start_then_nothing(self):
        clock = FakeClock(start=1000)
        self._write(clock, ['%d start' % clock.t])
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(stall=20, poll=1, budget=120), clock, out)
        self.assertEqual(code, MODULE.EXIT_STALLED)
        self.assertFalse(self.real_sleep.called)

    def test_37_growth_resets_stall(self):
        clock = FakeClock(start=1000)
        self._write(clock, ['%d start' % clock.t])
        counter = {'n': 0}

        def growing_sleep(seconds):
            clock.t += seconds
            counter['n'] += 1
            self._write(clock, ['%d start' % 1000, '%d read:%d' % (clock.t, counter['n'])])
        clock.sleep = growing_sleep

        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(budget=30, poll=5, stall=20), clock, out)
        self.assertEqual(code, MODULE.EXIT_RUNNING)
        self.assertGreaterEqual(clock.t, 1030)
        self.assertFalse(self.real_sleep.called)

    def test_38_max_runtime_while_alive(self):
        clock = FakeClock(start=1000)
        self._write(clock, ['%d start' % clock.t])
        counter = {'n': 0}

        def growing_sleep(seconds):
            clock.t += seconds
            counter['n'] += 1
            self._write(clock, ['%d start' % 1000, '%d read:%d' % (clock.t, counter['n'])])
        clock.sleep = growing_sleep

        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(max_runtime=40, budget=300, poll=5, stall=600),
                             clock, out)
        self.assertEqual(code, MODULE.EXIT_MAX_RUNTIME)
        self.assertFalse(self.real_sleep.called)

    def test_39_anchor_reused_across_calls(self):
        clock = FakeClock(start=1000)
        out1 = io.StringIO()
        code1 = MODULE.watch(self.log_path, windows(budget=0), clock, out1)
        self.assertEqual(code1, MODULE.EXIT_RUNNING)

        clock.t += 400
        out2 = io.StringIO()
        code2 = MODULE.watch(self.log_path, windows(launch=300, budget=0), clock, out2)
        self.assertEqual(code2, MODULE.EXIT_NO_START)
        self.assertFalse(self.real_sleep.called)

    def test_40_reset_restarts_reference(self):
        clock = FakeClock(start=1000)
        MODULE.watch(self.log_path, windows(budget=0), clock, io.StringIO())
        clock.t += 400
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(launch=300, budget=0), clock, out, reset=True)
        self.assertEqual(code, MODULE.EXIT_RUNNING)
        self.assertFalse(self.real_sleep.called)

    def test_41_poll_clamped_to_budget(self):
        clock = FakeClock(start=1000)
        # never write the file, so we hit budget expiry (RUNNING) rather than NO_START
        out = io.StringIO()
        code = MODULE.watch(self.log_path, windows(launch=100, poll=100, budget=7), clock, out)
        self.assertEqual(code, MODULE.EXIT_RUNNING)
        self.assertEqual(len(clock.slept), 1)
        self.assertLessEqual(clock.slept[0], 7)
        self.assertFalse(self.real_sleep.called)


class RenderAndMainTests(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self.dir = self._tmpdir.name
        self.log_path = os.path.join(self.dir, 'sample.progress.log')

    def _write(self, lines, mtime=None):
        with open(self.log_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        if mtime is not None:
            os.utime(self.log_path, (mtime, mtime))

    def test_42_render_done(self):
        lines = ['1000 start', '1010 done']
        state = MODULE.LogState(exists=True, size=20, mtime=1010.0, lines=lines)
        summary = MODULE.summarize(lines)
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_DONE, 'done', self.log_path, 1000, 1010, 1000,
                       state, summary, windows())
        text = out.getvalue()
        self.assertTrue(text.startswith('watch_progress: DONE (exit 0)\n'))
        self.assertIn('elapsed:', text)
        self.assertIn('milestones:', text)
        self.assertIn('1010 done', text)

    def test_43_tail_limit(self):
        base = 1755990000
        lines = ['%d read:%d' % (base + i, i) for i in range(20)]
        state = MODULE.LogState(exists=True, size=100, mtime=base + 19.0, lines=lines)
        summary = MODULE.summarize(lines)
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_RUNNING, 'healthy', self.log_path, base, base + 19, base + 19,
                       state, summary, windows(tail=3))
        text = out.getvalue()
        self.assertIn('last 3 of 20 lines', text)
        header = '--- tail (last 3 of 20 lines) ---\n'
        tail_section = text.split(header)[1]
        tail_lines = tail_section.rstrip('\n').split('\n')
        self.assertEqual(len(tail_lines), 3)
        self.assertEqual(tail_lines, lines[-3:])

    def test_44_no_start_render(self):
        state = MODULE.LogState(exists=False, size=0, mtime=None, lines=[])
        summary = MODULE.summarize([])
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_NO_START, 'no heartbeat', self.log_path, 0, 400, 0,
                       state, summary, windows())
        self.assertIn('next: the launch never began', out.getvalue())

    def test_45_running_render(self):
        state = MODULE.LogState(exists=False, size=0, mtime=None, lines=[])
        summary = MODULE.summarize([])
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_RUNNING, 'healthy', self.log_path, 0, 60, 0,
                       state, summary, windows())
        self.assertIn('next: still healthy — run this same command again.', out.getvalue())

    def test_46_missing_start_note(self):
        lines = ['1000 read:x']
        state = MODULE.LogState(exists=True, size=10, mtime=1000.0, lines=lines)
        summary = MODULE.summarize(lines)
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_RUNNING, 'healthy', self.log_path, 990, 1005, 1000,
                       state, summary, windows())
        self.assertIn("note: no explicit 'start' line", out.getvalue())

    def test_47_skew_note_does_not_change_exit_code(self):
        base = 1755990000
        lines = ['%d start' % base]
        state = MODULE.LogState(exists=True, size=10, mtime=base + 3600.0, lines=lines)
        summary = MODULE.summarize(lines)
        out = io.StringIO()
        MODULE.render(out, MODULE.EXIT_RUNNING, 'healthy', self.log_path, base, base + 5, base,
                       state, summary, windows())
        self.assertIn('note:', out.getvalue())
        self.assertIn('timestamps in this log may be unreliable', out.getvalue())

    def test_48_main_no_log_argument(self):
        out = io.StringIO()
        err = io.StringIO()
        with mock.patch.object(MODULE.sys, 'stderr', err):
            code = MODULE.main([], clock=FakeClock(), out=out)
        self.assertEqual(code, MODULE.EXIT_USAGE)
        self.assertTrue(err.getvalue())

    def test_49_main_negative_window(self):
        err = io.StringIO()
        with mock.patch.object(MODULE.sys, 'stderr', err):
            code = MODULE.main(['--stall-window', '-5', self.log_path], clock=FakeClock(),
                                out=io.StringIO())
        self.assertEqual(code, MODULE.EXIT_USAGE)

    def test_50_once_on_healthy_growing_log(self):
        self._write(['1000 start', '1005 read:x'], mtime=1005.0)
        clock = FakeClock(start=1010)
        out = io.StringIO()
        with mock.patch.object(MODULE.time, 'sleep') as real_sleep:
            code = MODULE.main([self.log_path, '--once'], clock=clock, out=out)
        self.assertEqual(code, MODULE.EXIT_RUNNING)
        self.assertFalse(real_sleep.called)

    def test_51_once_on_done_log(self):
        self._write(['1000 start', '1005 done'], mtime=1005.0)
        clock = FakeClock(start=1010)
        out = io.StringIO()
        code = MODULE.main([self.log_path, '--once'], clock=clock, out=out)
        self.assertEqual(code, MODULE.EXIT_DONE)


if __name__ == '__main__':
    unittest.main()
