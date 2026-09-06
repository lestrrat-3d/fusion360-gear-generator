#!/usr/bin/env python3
"""Tests for run_compile_gates.py.

No test touches the real proof, the Go toolchain, or the API database. Every test builds a
throwaway repo in a tempfile.TemporaryDirectory(), writes stub gate scripts and a stub
`proof/run.sh`, and points the runner at them by patching RUNNER.scripts_dir with
unittest.mock.patch.object.
"""
import contextlib
import importlib.util
import io
import json
import os
import stat
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

CHECKER_PATH = Path(__file__).with_name('run_compile_gates.py')
MODULE_SPEC = importlib.util.spec_from_file_location('run_compile_gates', CHECKER_PATH)
RUNNER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(RUNNER)

GEAR = 'spurgear'

STUB_HEADLINE = {
    'compile': 'compile check: OK (12 steps, 4 proof functions, 3 spec files stamped)',
    'playbook': ('playbook-extract check: spurgear: 38 anchors cited (25 defined in the '
                 'playbook), 27 blocks written'),
    'step_calls': 'step-call check: OK (0 named calls present, no stubs, no shared-point misuse)',
}

PROOF_OK = 'ok  \tgithub.com/lestrrat-3d/fusion360-gear-generator/proof/spurgear\t0.4s'


def _write_script(path, lines):
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    st = os.stat(path)
    os.chmod(path, st.st_mode | stat.S_IEXEC)
    return path


def make_stub(directory, name, exit_code=0, stdout='', stderr='', sleep=0.0, argv_marker=None,
              cwd_marker=None):
    """Write a Python gate script that prints `stdout`, prints `stderr` to stderr, optionally
    sleeps, and exits `exit_code`. `argv_marker` records sys.argv[1:] as JSON, `cwd_marker`
    records os.getcwd(), so a test can assert on what the runner passed and where it ran."""
    lines = ['#!/usr/bin/env python3', 'import sys']
    if argv_marker or cwd_marker:
        lines.append('import json, os')
    if argv_marker:
        lines.append('with open(%r, "w") as fh:' % argv_marker)
        lines.append('    json.dump(sys.argv[1:], fh)')
    if cwd_marker:
        lines.append('with open(%r, "w") as fh:' % cwd_marker)
        lines.append('    fh.write(os.getcwd())')
    if sleep:
        lines.append('import time; time.sleep(%r)' % sleep)
    for line in stdout.splitlines():
        lines.append('print(%r)' % line)
    for line in stderr.splitlines():
        lines.append('print(%r, file=sys.stderr)' % line)
    lines.append('sys.exit(%r)' % exit_code)
    return _write_script(os.path.join(directory, name), lines)


def make_stubs(scripts, overrides=None):
    """Default-passing stubs for both Python checkers, with per-key overrides (dict of stage
    key -> kwargs for make_stub)."""
    overrides = overrides or {}
    for key, name in RUNNER.STAGE_SCRIPTS.items():
        kwargs = dict(exit_code=0, stdout=STUB_HEADLINE[key])
        kwargs.update(overrides.get(key, {}))
        make_stub(scripts, name, **kwargs)
    return scripts


def make_proof_runner(root, exit_code=0, stdout=PROOF_OK, stderr='', sleep=0.0, marker=None,
                      argv_marker=None):
    """A stub `proof/run.sh` standing in for the Go proof."""
    lines = ['#!/usr/bin/env bash']
    if marker:
        lines.append('echo "$PWD" > %s' % marker)
    if argv_marker:
        lines.append('printf "%s\\n" "$@" > %s' % ("%s", argv_marker))
    if sleep:
        lines.append('sleep %s' % sleep)
    for line in stdout.splitlines():
        lines.append('echo %s' % json.dumps(line))
    for line in stderr.splitlines():
        lines.append('echo %s >&2' % json.dumps(line))
    lines.append('exit %d' % exit_code)
    return _write_script(os.path.join(root, 'proof', 'run.sh'), lines)


def make_repo(tmp, gear=GEAR, *, steps='# steps\n', module=None, proof_go='package spurgear\n'):
    """Create spec/<gear>/steps.md, proof/<gear>/proof.go, and, when `module` is not None,
    lib/geargen/<gear>.py. Returns the root path."""
    root = tmp
    os.makedirs(os.path.join(root, 'spec', gear), exist_ok=True)
    os.makedirs(os.path.join(root, 'proof', gear), exist_ok=True)
    os.makedirs(os.path.join(root, 'lib', 'geargen'), exist_ok=True)
    if steps is not None:
        with open(os.path.join(root, 'spec', gear, 'steps.md'), 'w') as fh:
            fh.write(steps)
    if proof_go is not None:
        with open(os.path.join(root, 'proof', gear, 'proof.go'), 'w') as fh:
            fh.write(proof_go)
    if module is not None:
        with open(os.path.join(root, 'lib', 'geargen', '%s.py' % gear), 'w') as fh:
            fh.write(module)
    return root


def run(root, scripts, *argv):
    """Patch scripts_dir -> scripts, capture stdout, call RUNNER.main, and return
    (exit_code, text, parsed_json)."""
    buf = io.StringIO()
    with mock.patch.object(RUNNER, 'scripts_dir', return_value=scripts):
        with contextlib.redirect_stdout(buf):
            exit_code = RUNNER.main(['--root', root] + list(argv))
    text = buf.getvalue()
    parsed = None
    for line in text.splitlines():
        if line.startswith(RUNNER.JSON_MARKER):
            parsed = json.loads(line[len(RUNNER.JSON_MARKER):])
    return exit_code, text, parsed


def git(root, *args):
    return subprocess.run(['git', '-C', root] + list(args), check=True,
                          capture_output=True, text=True)


def init_fixture_git(root):
    git(root, 'init', '--quiet')
    git(root, 'config', 'user.email', 'fixture@example.invalid')
    git(root, 'config', 'user.name', 'fixture')
    git(root, 'add', '.')
    git(root, 'commit', '--quiet', '-m', 'base')
    return git(root, 'rev-parse', 'HEAD').stdout.strip()


class BaseRunnerTest(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self.tmp = self._tmpdir.name
        self.scripts = os.path.join(self.tmp, 'scripts')
        os.makedirs(self.scripts, exist_ok=True)

    def by_key(self, parsed):
        return {s['key']: s for s in parsed['stages']}


class HappyPathTests(BaseRunnerTest):
    def test_all_stages_pass_with_module_present(self):
        root = make_repo(self.tmp, module='x = 1\n')
        marker = os.path.join(self.tmp, 'step-calls-argv.json')
        make_stubs(self.scripts, overrides={'step_calls': dict(argv_marker=marker)})
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['verdict'], 'pass')
        self.assertEqual(parsed['effective_proof_scope'], 'full')
        self.assertTrue(parsed['proof_is_complete'])
        self.assertEqual([s['key'] for s in parsed['stages']], list(RUNNER.STAGE_ORDER))
        for stage in parsed['stages']:
            self.assertEqual(stage['status'], 'pass', msg=stage)
        with open(marker) as fh:
            argv = json.load(fh)
        self.assertEqual(argv, [os.path.join('spec', GEAR, 'steps.md'),
                                os.path.join('lib', 'geargen', '%s.py' % GEAR)])

    def test_every_stage_runs_from_the_repo_root(self):
        root = make_repo(self.tmp, module='x = 1\n')
        proof_cwd = os.path.join(self.tmp, 'proof-cwd.txt')
        compile_cwd = os.path.join(self.tmp, 'compile-cwd.txt')
        make_stubs(self.scripts, overrides={'compile': dict(cwd_marker=compile_cwd)})
        make_proof_runner(root, marker=proof_cwd)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        for path in (proof_cwd, compile_cwd):
            with open(path) as fh:
                self.assertEqual(os.path.realpath(fh.read().strip()), os.path.realpath(root))

    def test_compile_stage_is_given_the_gear_name(self):
        root = make_repo(self.tmp)
        marker = os.path.join(self.tmp, 'compile-argv.json')
        make_stubs(self.scripts, overrides={'compile': dict(argv_marker=marker)})
        make_proof_runner(root)
        run(root, self.scripts, GEAR)
        with open(marker) as fh:
            self.assertEqual(json.load(fh), [GEAR])

    def test_default_proof_invocation_keeps_complete_wrapper_scope(self):
        root = make_repo(self.tmp)
        marker = os.path.join(self.tmp, 'proof-argv.txt')
        make_stubs(self.scripts)
        make_proof_runner(root, argv_marker=marker)
        exit_code, _text, _parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        with open(marker) as fh:
            self.assertEqual(fh.read().strip(), '')


class ModuleAbsentTests(BaseRunnerTest):
    def test_step_calls_skips_with_the_stated_reason(self):
        root = make_repo(self.tmp)  # no lib/geargen/<gear>.py
        marker = os.path.join(self.tmp, 'never.marker')
        make_stubs(self.scripts, overrides={'step_calls': dict(argv_marker=marker)})
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['verdict'], 'pass')
        stage = self.by_key(parsed)['step_calls']
        self.assertEqual(stage['status'], 'skip')
        self.assertEqual(stage['skip_reason'], RUNNER.MODULE_ABSENT_REASON % GEAR)
        self.assertFalse(os.path.exists(marker))

    def test_missing_step_calls_script_is_not_a_setup_error_without_a_module(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        os.remove(os.path.join(self.scripts, 'check_step_calls.py'))
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)


class ProofFailureTests(BaseRunnerTest):
    def test_red_proof_is_a_draft_fault(self):
        root = make_repo(self.tmp, module='x = 1\n')
        make_stubs(self.scripts)
        make_proof_runner(root, exit_code=1, stdout='--- FAIL: TestGearProfileSketch\nFAIL')
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 1)
        self.assertEqual(parsed['verdict'], 'fail')
        stage = self.by_key(parsed)['proof']
        self.assertEqual(stage['status'], 'fail')
        self.assertEqual(stage['fault'], RUNNER.FAULT_PROOF)
        self.assertIn('DRAFT FAULT', text)
        # a failure does not stop the later stages by default
        self.assertEqual(self.by_key(parsed)['compile']['status'], 'pass')

    def test_proof_setup_exit_is_an_error(self):
        root = make_repo(self.tmp, module='x = 1\n')
        make_stubs(self.scripts)
        make_proof_runner(root, exit_code=2, stdout='',
                          stderr='Go module not found at: /nowhere/sketch')
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['verdict'], 'error')
        stage = self.by_key(parsed)['proof']
        self.assertEqual(stage['status'], 'error')
        self.assertEqual(stage['fault'], RUNNER.FAULT_SETUP)
        self.assertIn('Go module not found', text)


class CompileClassificationTests(BaseRunnerTest):
    def _run_compile_failure(self, problem):
        root = make_repo(self.tmp)
        make_stubs(self.scripts, overrides={
            'compile': dict(exit_code=1,
                            stdout='compile check: BLOCKING (1)\n  %s' % problem),
        })
        make_proof_runner(root)
        return run(root, self.scripts, GEAR)

    def test_unknown_api_call_needs_judgment(self):
        exit_code, text, parsed = self._run_compile_failure(
            "the step list names 'fooBar(', which the Fusion API database does not have")
        self.assertEqual(exit_code, 1)
        stage = self.by_key(parsed)['compile']
        self.assertEqual(stage['status'], 'fail')
        self.assertIn('NEEDS JUDGMENT', stage['fault'])
        self.assertIn('fooBar', stage['fault'])
        self.assertIn('NEEDS JUDGMENT', text)

    def test_provenance_drift_names_a_mid_run_edit(self):
        exit_code, text, parsed = self._run_compile_failure(
            'spec/spurgear/instructions.md has changed since the step list was compiled '
            '(abc123456789 now, def123456789 then)')
        self.assertEqual(exit_code, 1)
        stage = self.by_key(parsed)['compile']
        self.assertEqual(stage['fault'], RUNNER.FAULT_DRIFT)
        self.assertIn('edited the spec mid-run', text)

    def test_citation_problem_is_a_plain_draft_fault(self):
        exit_code, text, parsed = self._run_compile_failure(
            'S3 cites spec/spurgear/nope.md, which does not exist')
        self.assertEqual(exit_code, 1)
        self.assertEqual(self.by_key(parsed)['compile']['fault'], RUNNER.FAULT_DRAFT)

    def test_step_calls_failure_needs_classification(self):
        root = make_repo(self.tmp, module='x = 1\n')
        make_stubs(self.scripts, overrides={
            'step_calls': dict(exit_code=1,
                               stdout="step-call check: BLOCKING (1)\n"
                                      "  never calls 'addByCenterRadius('"),
        })
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 1)
        self.assertEqual(self.by_key(parsed)['step_calls']['fault'], RUNNER.FAULT_STEP_CALLS)
        self.assertIn('NEEDS CLASSIFICATION', text)


class PlaybookStageTests(BaseRunnerTest):
    """The stage that refuses a step list citing no playbook rule."""

    def _run_playbook_failure(self, stderr):
        root = make_repo(self.tmp)
        make_stubs(self.scripts, overrides={
            'playbook': dict(exit_code=1, stdout='', stderr=stderr),
        })
        make_proof_runner(root)
        return run(root, self.scripts, GEAR)

    def test_stage_gets_the_gear_name_and_the_anchor_floor(self):
        root = make_repo(self.tmp)
        marker = os.path.join(self.tmp, 'playbook-argv.json')
        make_stubs(self.scripts, overrides={'playbook': dict(argv_marker=marker)})
        make_proof_runner(root)
        exit_code, _text, _parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        with open(marker) as fh:
            self.assertEqual(json.load(fh),
                             [GEAR, '--min-anchors', str(RUNNER.MIN_PLAYBOOK_ANCHORS)])

    def test_stage_runs_even_without_a_module(self):
        root = make_repo(self.tmp)  # no lib/geargen/<gear>.py
        make_stubs(self.scripts)
        make_proof_runner(root)
        _exit_code, _text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(self.by_key(parsed)['playbook']['status'], 'pass')

    def test_uncited_step_list_is_a_draft_fault(self):
        exit_code, text, parsed = self._run_playbook_failure(
            'extract_playbook: the step list cites 0 playbook anchor(s); at least 1 is required.')
        self.assertEqual(exit_code, 1)
        stage = self.by_key(parsed)['playbook']
        self.assertEqual(stage['status'], 'fail')
        self.assertEqual(stage['fault'], RUNNER.FAULT_PLAYBOOK_UNCITED)
        self.assertIn('cites no playbook rule', text)

    def test_undefined_anchor_is_the_other_fault(self):
        exit_code, _text, parsed = self._run_playbook_failure(
            'extract_playbook: cited but defined nowhere in the playbook: PB-MADE-UP')
        self.assertEqual(exit_code, 1)
        self.assertEqual(self.by_key(parsed)['playbook']['fault'],
                         RUNNER.FAULT_PLAYBOOK_UNDEFINED)

    def test_unreadable_playbook_is_a_setup_error_not_a_draft_fault(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts, overrides={
            'playbook': dict(exit_code=2,
                             stderr='extract_playbook: cannot read playbook PLAYBOOK.md'),
        })
        make_proof_runner(root)
        exit_code, _text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        stage = self.by_key(parsed)['playbook']
        self.assertEqual(stage['status'], 'error')
        self.assertEqual(stage['fault'], RUNNER.FAULT_SETUP)


class SelectionTests(BaseRunnerTest):
    def test_only_compile_runs_one_stage(self):
        root = make_repo(self.tmp, module='x = 1\n')
        compile_marker = os.path.join(self.tmp, 'compile-argv.json')
        step_marker = os.path.join(self.tmp, 'never.marker')
        proof_marker = os.path.join(self.tmp, 'proof-ran.txt')
        make_stubs(self.scripts, overrides={
            'compile': dict(argv_marker=compile_marker),
            'step_calls': dict(argv_marker=step_marker),
        })
        make_proof_runner(root, marker=proof_marker)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--only', 'compile')
        self.assertEqual(exit_code, 0)
        by_key = self.by_key(parsed)
        self.assertEqual(by_key['compile']['status'], 'pass')
        self.assertTrue(os.path.exists(compile_marker))
        for key in ('proof', 'step_calls'):
            self.assertEqual(by_key[key]['status'], 'skip')
            self.assertEqual(by_key[key]['skip_reason'], 'not selected')
        self.assertFalse(os.path.exists(step_marker))
        self.assertFalse(os.path.exists(proof_marker))
        self.assertEqual(parsed['effective_proof_scope'], 'omitted')
        self.assertFalse(parsed['proof_is_complete'])

    def test_only_compile_does_not_need_the_proof_runner(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        # no proof/run.sh written at all
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--only', 'compile')
        self.assertEqual(exit_code, 0)

    def test_unknown_only_key_is_a_usage_error(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--only', 'nope')
        self.assertEqual(exit_code, 2)
        self.assertIn('nope', text)
        self.assertEqual(parsed['stages'], [])

    def test_fail_fast_stops_scheduling(self):
        root = make_repo(self.tmp, module='x = 1\n')
        compile_marker = os.path.join(self.tmp, 'never-compile.marker')
        step_marker = os.path.join(self.tmp, 'never-steps.marker')
        make_stubs(self.scripts, overrides={
            'compile': dict(argv_marker=compile_marker),
            'step_calls': dict(argv_marker=step_marker),
        })
        make_proof_runner(root, exit_code=1, stdout='FAIL')
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--fail-fast')
        self.assertEqual(exit_code, 1)
        by_key = self.by_key(parsed)
        for key in ('compile', 'step_calls'):
            self.assertEqual(by_key[key]['status'], 'skip')
            self.assertIn('--fail-fast', by_key[key]['skip_reason'])
        self.assertFalse(os.path.exists(compile_marker))
        self.assertFalse(os.path.exists(step_marker))


class OutputTests(BaseRunnerTest):
    def test_text_report_ends_with_one_json_line(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        make_proof_runner(root)
        json_out = os.path.join(self.tmp, 'verdict.json')
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--json-out', json_out)
        last_line = [ln for ln in text.splitlines() if ln.strip()][-1]
        self.assertTrue(last_line.startswith(RUNNER.JSON_MARKER))
        self.assertEqual(parsed['schema'], 1)
        self.assertEqual(parsed['gear'], GEAR)
        self.assertEqual(os.path.realpath(parsed['root']), os.path.realpath(root))
        with open(json_out) as fh:
            self.assertEqual(json.load(fh), parsed)

    def test_format_json_prints_only_json(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--format', 'json')
        lines = [ln for ln in text.splitlines() if ln.strip()]
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith(RUNNER.JSON_MARKER))
        self.assertEqual(parsed['verdict'], 'pass')
        for stage in parsed['stages']:
            for field in ('key', 'title', 'status', 'exit_code', 'duration_s', 'command',
                          'stdout', 'stderr', 'skip_reason', 'fault'):
                self.assertIn(field, stage)

    def test_advisory_lines_survive_a_passing_run(self):
        root = make_repo(self.tmp)
        advisory = ('coverage: spec/spurgear/instructions.md — 40/60 lines claimed by a step, '
                    '20 unclaimed')
        make_stubs(self.scripts, overrides={
            'compile': dict(stdout='%s\n%s' % (advisory, STUB_HEADLINE['compile'])),
        })
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        self.assertIn(advisory, text)
        self.assertIn('compile check: OK', text)


class SetupErrorTests(BaseRunnerTest):
    def test_missing_step_list_stops_before_any_stage(self):
        root = make_repo(self.tmp, steps=None)
        marker = os.path.join(self.tmp, 'never.marker')
        make_stubs(self.scripts, overrides={'compile': dict(argv_marker=marker)})
        proof_marker = os.path.join(self.tmp, 'proof-ran.txt')
        make_proof_runner(root, marker=proof_marker)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['verdict'], 'error')
        self.assertIn(os.path.join('spec', GEAR, 'steps.md'), text)
        self.assertFalse(os.path.exists(marker))
        self.assertFalse(os.path.exists(proof_marker))

    def test_missing_proof_directory_says_the_draft_was_never_placed(self):
        root = make_repo(self.tmp, proof_go=None)
        os.rmdir(os.path.join(root, 'proof', GEAR))
        make_stubs(self.scripts)
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertIn('never placed', text)

    def test_bad_gear_name(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, 'Spur Gear')
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['stages'], [])

    def test_missing_compile_script(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts)
        os.remove(os.path.join(self.scripts, 'check_compile.py'))
        make_proof_runner(root)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertIn('check_compile.py', text)

    def test_timeout_is_an_error(self):
        root = make_repo(self.tmp)
        make_stubs(self.scripts, overrides={'compile': dict(sleep=5)})
        make_proof_runner(root)
        start = time.monotonic()
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--timeout', '0.2')
        elapsed = time.monotonic() - start
        self.assertEqual(exit_code, 2)
        stage = self.by_key(parsed)['compile']
        self.assertEqual(stage['status'], 'error')
        self.assertEqual(stage['fault'], RUNNER.FAULT_SETUP)
        self.assertIn('timed out', stage['stderr'])
        self.assertLess(elapsed, 4)


class IterationTests(BaseRunnerTest):
    def setUp(self):
        super().setUp()
        make_repo(self.tmp, module='x = 1\n')
        with open(os.path.join(self.tmp, 'PLAYBOOK.md'), 'w') as fh:
            fh.write('# fixture playbook\n')
        with open(os.path.join(self.tmp, '.gitignore'), 'w') as fh:
            fh.write('scratch/\n')
        self.proof_args = os.path.join(self.tmp, 'proof-args.txt')
        make_stubs(self.scripts)
        make_proof_runner(self.tmp, argv_marker=self.proof_args)
        self.base = init_fixture_git(self.tmp)

    def test_gear_only_change_selects_one_package_and_marks_iteration(self):
        with open(os.path.join(self.tmp, 'spec', GEAR, 'instructions.md'), 'w') as fh:
            fh.write('changed gear input\n')
        exit_code, text, parsed = run(self.tmp, self.scripts, GEAR,
                                     '--iteration-base', self.base)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['iteration_mode'], True)
        self.assertEqual(parsed['iteration_base'], self.base)
        self.assertEqual(parsed['effective_proof_scope'], 'selected')
        self.assertEqual(parsed['selected_packages'], ['./%s' % GEAR])
        self.assertFalse(parsed['proof_is_complete'])
        self.assertIn('proof scope: selected', text)
        with open(self.proof_args) as fh:
            self.assertEqual(fh.read().splitlines(), ['--package', './%s' % GEAR])

    def test_shared_change_expands_full_scope(self):
        with open(os.path.join(self.tmp, 'PLAYBOOK.md'), 'a') as fh:
            fh.write('shared change\n')
        exit_code, text, parsed = run(self.tmp, self.scripts, GEAR,
                                     '--iteration-base', self.base)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['effective_proof_scope'], 'expanded-full')
        self.assertEqual(parsed['selected_packages'], [])
        self.assertEqual(parsed['full_suite_reason'], 'shared_or_other_path_changed')
        self.assertIn('full-suite expansion reason:', text)
        with open(self.proof_args) as fh:
            self.assertEqual(fh.read().strip(), '')

    def test_committed_change_after_base_expands_scope(self):
        with open(os.path.join(self.tmp, 'shared.txt'), 'w') as fh:
            fh.write('shared\n')
        git(self.tmp, 'add', 'shared.txt')
        git(self.tmp, 'commit', '--quiet', '-m', 'shared')
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['effective_proof_scope'], 'expanded-full')

    def test_staged_unstaged_renamed_deleted_and_untracked_shared_paths_expand(self):
        cases = ('staged', 'unstaged', 'renamed', 'deleted', 'untracked')
        for case in cases:
            with self.subTest(case=case):
                root = tempfile.mkdtemp(dir=self.tmp)
                make_repo(root, module='x = 1\n')
                with open(os.path.join(root, 'PLAYBOOK.md'), 'w') as fh:
                    fh.write('playbook\n')
                marker = os.path.join(root, 'proof-args.txt')
                scripts = os.path.join(root, 'scripts')
                os.makedirs(scripts)
                make_stubs(scripts)
                make_proof_runner(root, argv_marker=marker)
                with open(os.path.join(root, 'shared.txt'), 'w') as fh:
                    fh.write('shared\n')
                base = init_fixture_git(root)
                if case == 'staged':
                    with open(os.path.join(root, 'shared.txt'), 'a') as fh:
                        fh.write('staged\n')
                    git(root, 'add', 'shared.txt')
                elif case == 'unstaged':
                    with open(os.path.join(root, 'shared.txt'), 'a') as fh:
                        fh.write('unstaged\n')
                elif case == 'renamed':
                    git(root, 'mv', 'shared.txt', 'other-shared.txt')
                elif case == 'deleted':
                    os.remove(os.path.join(root, 'shared.txt'))
                else:
                    with open(os.path.join(root, 'shared-untracked.txt'), 'w') as fh:
                        fh.write('untracked\n')
                exit_code, _text, parsed = run(root, scripts, GEAR,
                                               '--iteration-base', base)
                self.assertEqual(exit_code, 0)
                self.assertEqual(parsed['effective_proof_scope'], 'expanded-full')

    def test_ignored_scratch_does_not_expand_a_gear_only_change(self):
        with open(os.path.join(self.tmp, 'spec', GEAR, 'instructions.md'), 'w') as fh:
            fh.write('changed gear input\n')
        os.makedirs(os.path.join(self.tmp, 'scratch'))
        with open(os.path.join(self.tmp, 'scratch', 'ignored.txt'), 'w') as fh:
            fh.write('ignored\n')
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['effective_proof_scope'], 'selected')

    def test_untracked_proof_go_source_is_relevant(self):
        os.remove(os.path.join(self.tmp, 'proof', GEAR, 'proof.go'))
        with open(os.path.join(self.tmp, 'proof', GEAR, 'new.go'), 'w') as fh:
            fh.write('package spurgear\n')
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['effective_proof_scope'], 'selected')

    def test_invalid_base_and_only_fail_before_execution(self):
        marker = os.path.join(self.tmp, 'ran.marker')
        make_proof_runner(self.tmp, marker=marker)
        for args in (('--iteration-base', 'not-a-commit'),
                     ('--iteration-base', self.base, '--only', 'compile')):
            with self.subTest(args=args):
                exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR, *args)
                self.assertEqual(exit_code, 2)
                self.assertEqual(parsed['stages'], [])
                self.assertFalse(os.path.exists(marker))

    def test_content_and_setup_failures_omit_proof_with_distinct_reasons(self):
        for code, reason in ((1, 'compile_content_failure'), (2, 'compile_setup_failure')):
            with self.subTest(code=code):
                marker = os.path.join(self.tmp, 'proof-ran-%s.marker' % code)
                make_proof_runner(self.tmp, marker=marker)
                make_stubs(self.scripts, overrides={
                    'compile': dict(exit_code=code, stdout='compile failed')})
                exit_code, text, parsed = run(self.tmp, self.scripts, GEAR,
                                             '--iteration-base', self.base)
                self.assertEqual(exit_code, code)
                self.assertEqual(parsed['effective_proof_scope'], 'omitted')
                self.assertEqual(parsed['planned_proof_scope'], 'expanded-full')
                self.assertEqual(parsed['proof_omission_reasons'], [reason])
                self.assertIn(reason, parsed['proof_omission_reason'])
                self.assertIn(reason, text)
                self.assertFalse(os.path.exists(marker))

    def test_selected_and_expanded_proof_failures_propagate(self):
        make_proof_runner(self.tmp, exit_code=1, stdout='proof failed')
        with open(os.path.join(self.tmp, 'spec', GEAR, 'instructions.md'), 'w') as fh:
            fh.write('changed gear input\n')
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base)
        self.assertEqual(exit_code, 1)
        self.assertEqual(parsed['stages'][-1]['status'], 'fail')
        make_proof_runner(self.tmp, exit_code=2, stderr='engine mismatch')
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base)
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['stages'][-1]['status'], 'error')

    def test_fail_fast_step_call_failure_omits_proof(self):
        marker = os.path.join(self.tmp, 'proof-ran.marker')
        make_proof_runner(self.tmp, marker=marker)
        make_stubs(self.scripts, overrides={
            'step_calls': dict(exit_code=1, stdout='step-call failed')})
        exit_code, _text, parsed = run(self.tmp, self.scripts, GEAR,
                                       '--iteration-base', self.base, '--fail-fast')
        self.assertEqual(exit_code, 1)
        by_key = {stage['key']: stage for stage in parsed['stages']}
        self.assertEqual(by_key['step_calls']['status'], 'fail')
        self.assertEqual(by_key['proof']['status'], 'skip')
        self.assertEqual(parsed['proof_omission_reasons'], ['step_calls_content_failure'])
        self.assertEqual(parsed['effective_proof_scope'], 'omitted')
        self.assertFalse(os.path.exists(marker))


class CompileWorkflowInstructionTests(unittest.TestCase):
    def test_retry_instructions_require_final_complete_runner(self):
        root = CHECKER_PATH.parents[3]
        skill = (root / '.claude' / 'skills' / 'compile-gear' / 'SKILL.md').read_text()
        self.assertIn('--iteration-base', skill)
        self.assertIn('> .tmp/<gear>.compile-gates.txt', skill)
        self.assertNotIn('.tmp/<gear>.compile-iteration.txt', skill)
        self.assertIn('ordinary complete', skill)
        self.assertIn('require a pass from the ordinary complete', skill)
        self.assertIn('Reuse the latest report', skill)


class HandoffBindingTests(unittest.TestCase):
    def test_checker_payload_requires_exact_machine_types(self):
        payload = {'ok': True, 'named_calls': 1, 'missing': [], 'stubs': [],
                   'shared_point': [], 'parse_error': None}
        self.assertIsNone(RUNNER._validate_checker_payload(payload))
        payload['missing'] = [{'name': 'op', 'has_receiver': 1, 'textual_match': False}]
        self.assertIn('invalid types', RUNNER._validate_checker_payload(payload))

    def test_missing_shapes_keep_new_existing_and_unknown_origins(self):
        missing = [{'name': 'old', 'has_receiver': True, 'textual_match': False},
                   {'name': 'new', 'has_receiver': True, 'textual_match': False}]
        self.assertEqual([item['origin'] for item in RUNNER._review_requirements(
            missing, b'Call `sketch.old()`.')], ['already_required', 'new_since_base'])
        self.assertEqual({item['origin'] for item in RUNNER._review_requirements(missing, None)},
                         {'baseline_unknown'})

    def test_review_binding_rejects_missing_duplicate_and_stale_decisions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'spec').mkdir()
            (root / 'lib' / 'geargen').mkdir(parents=True)
            (root / 'spec' / 'x' ).mkdir()
            (root / 'spec' / 'x' / 'steps.md').write_text('Call `newOp()`\n', encoding='utf-8')
            (root / 'lib' / 'geargen' / 'x.py').write_text('newOp()\n', encoding='utf-8')
            paths = RUNNER.Paths(str(root), 'x', 'spec/x/steps.md', 'proof/x', 'lib/geargen/x.py')
            requirements = [{'name': 'newOp', 'has_receiver': False, 'textual_match': False,
                             'origin': 'new_since_base'}]
            evidence = {'path': 'spec/x/steps.md', 'line': 1,
                        'sha256': RUNNER.sha256_bytes((root / 'spec/x/steps.md').read_bytes())}
            record = {'name': 'newOp', 'has_receiver': False, 'decision': 'emit_required',
                      'evidence': [evidence], 'reason': 'The source requires this call.'}
            binding = {'comparison_base': 'a' * 40,
                       'steps_sha256': RUNNER.sha256_bytes((root / 'spec/x/steps.md').read_bytes()),
                       'module_sha256': RUNNER.sha256_bytes((root / 'lib/geargen/x.py').read_bytes()),
                       'requirements_sha256': RUNNER.sha256_bytes(RUNNER._canonical(requirements)),
                       'review_inputs_sha256': RUNNER._review_input_hash(str(root), 'spec/x/steps.md', [record])}
            review = {'schema': 1, 'gear': 'x', 'binding': binding, 'requirements': [record]}
            review_path = root / 'review.json'
            review_path.write_text(json.dumps(review), encoding='utf-8')
            records, error = RUNNER._validate_review(str(review_path), 'x', 'a' * 40,
                                                     paths, requirements)
            self.assertIsNone(error)
            self.assertEqual(records[0]['decision'], 'emit_required')
            review['requirements'][0]['reason'] = ''
            review_path.write_text(json.dumps(review), encoding='utf-8')
            _records, error = RUNNER._validate_review(str(review_path), 'x', 'a' * 40,
                                                      paths, requirements)
            self.assertIn('malformed', error)

    def test_compile_policy_records_handoff_and_requires_complete_proof(self):
        args = type('Args', (), {'only': None})()
        metadata = {'iteration_mode': False, 'proof_is_complete': True,
                    'effective_proof_scope': 'full'}
        results = [type('Stage', (), {'key': 'proof', 'status': 'pass', 'skip_reason': None})(),
                   type('Stage', (), {'key': 'step_calls', 'status': 'pass', 'skip_reason': None})()]
        args._handoff_status = 'emit_required'
        timing = importlib.util.spec_from_file_location('timing_handoff_test',
            CHECKER_PATH.with_name('pipeline_timing.py'))
        module = importlib.util.module_from_spec(timing)
        timing.loader.exec_module(module)
        self.assertEqual(module.compile_policy_for_run(args, results, metadata)['handoff_status'],
                         'emit_required')


if __name__ == '__main__':
    unittest.main()
