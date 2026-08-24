#!/usr/bin/env python3
"""Tests for run_sketch_bench.py.

No test runs a real sketch bench or needs the Go toolchain: every case writes a tiny
executable `run.sh` into a throwaway `<tmp>/spec/<gear>/sketch/` and calls `main([...])`
in-process, asserting on the exit code and the captured output.
"""
import contextlib
import importlib.util
import io
import stat
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name('run_sketch_bench.py')
SPEC = importlib.util.spec_from_file_location('run_sketch_bench', MODULE_PATH)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)

GEAR = 'spurgear'


def make_bench(root, body, gear=GEAR):
    """Write an executable spec/<gear>/sketch/run.sh with `body` as its script text."""
    bench_dir = Path(root) / 'spec' / gear / 'sketch'
    bench_dir.mkdir(parents=True, exist_ok=True)
    script = bench_dir / 'run.sh'
    script.write_text('#!/usr/bin/env bash\n' + body)
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    return script


def run(root, *argv):
    """Call main() with --root <root>, returning (exit_code, captured stdout)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = RUNNER.main(['--root', str(root)] + list(argv))
    return code, buf.getvalue()


class BenchRunnerTest(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.root = Path(self._tmpdir.name)
        self.addCleanup(self._tmpdir.cleanup)

    def test_missing_bench_is_a_setup_error(self):
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 2)
        self.assertIn('spec/spurgear/sketch/run.sh', out)
        self.assertIn('[PB-SKETCH-FIRST]', out)
        self.assertTrue(out.strip().splitlines()[-1].startswith('[SETUP]'))

    def test_pass_verdict_exits_zero(self):
        make_bench(self.root,
                   'echo "case 1 OK"\n'
                   'echo "ALL PASS — the spur Gear Profile constraint scheme fully '
                   'constrains across sizes."\n'
                   'exit 0\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 0)
        self.assertEqual(out.strip().splitlines()[-1], '[GATE] sketch bench: PASS')

    def test_fail_verdict_exits_one(self):
        make_bench(self.root,
                   'echo "FAIL — case N=17 is under-constrained (DOF 2)"\n'
                   'exit 1\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 1)
        last = out.strip().splitlines()[-1]
        self.assertTrue(last.startswith('[GATE] sketch bench: FAIL'))
        self.assertIn('spec/playbook defect', last)

    def test_engine_missing_is_not_a_gate_failure(self):
        make_bench(self.root,
                   'echo "sketch repo not found at: /nowhere/sketch" >&2\n'
                   'echo "set SKETCH_DIR=/path/to/lestrrat-3d/sketch and re-run" >&2\n'
                   'exit 2\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 2)
        last = out.strip().splitlines()[-1]
        self.assertTrue(last.startswith('[SETUP]'))
        self.assertIn('exited 2', last)
        self.assertNotIn('[GATE]', out)

    def test_build_failure_is_a_setup_error(self):
        make_bench(self.root,
                   'echo "main.go:41:2: undefined: sketch.NewAngel" >&2\n'
                   'exit 1\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 2)
        last = out.strip().splitlines()[-1]
        self.assertTrue(last.startswith('[SETUP]'))
        self.assertIn("no 'FAIL' verdict line", last)

    def test_zero_exit_without_verdict_is_a_contract_violation(self):
        make_bench(self.root, 'echo "case 1 solved, DOF 0"\nexit 0\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 2)
        last = out.strip().splitlines()[-1]
        self.assertIn("no 'ALL PASS' verdict line", last)
        self.assertIn('[PB-SKETCH-FIRST]', last)

    def test_timeout_is_a_setup_error(self):
        make_bench(self.root, 'sleep 60\n')
        code, out = run(self.root, GEAR, '--timeout', '1')
        self.assertEqual(code, 2)
        self.assertIn('bench timed out after', out.strip().splitlines()[-1])

    def test_bench_output_is_passed_through(self):
        make_bench(self.root,
                   'echo "using sketch engine at: /somewhere/sketch"\n'
                   'echo "MARKER-advisory ProfilesValid=true probeAmbiguous=true"\n'
                   'echo "ALL PASS — everything holds"\n'
                   'exit 0\n')
        code, out = run(self.root, GEAR)
        self.assertEqual(code, 0)
        self.assertIn('using sketch engine at: /somewhere/sketch', out)
        self.assertIn('MARKER-advisory ProfilesValid=true probeAmbiguous=true', out)


class SentinelTest(unittest.TestCase):
    def test_real_spur_pass_line_matches(self):
        output = ('using sketch engine at: /home/u/sketch\n'
                  'ALL PASS — the spur Gear Profile constraint scheme fully constrains '
                  'across sizes.\nCleared to generate Fusion add-in code.\n')
        self.assertEqual(RUNNER.verdict_sentinel(output), 'pass')

    def test_indented_fail_line_matches(self):
        self.assertEqual(RUNNER.verdict_sentinel('  FAIL — module 2 N=17\n'), 'fail')

    def test_no_verdict_line(self):
        self.assertIsNone(RUNNER.verdict_sentinel('solved 6 cases\nDOF 0\n'))

    def test_last_verdict_line_wins(self):
        self.assertEqual(
            RUNNER.verdict_sentinel('ALL PASS — round 1\nFAIL — round 2\n'), 'fail')

    def test_verdict_must_start_the_line(self):
        self.assertIsNone(RUNNER.verdict_sentinel('the run did not ALL PASS\n'))


if __name__ == '__main__':
    unittest.main()
