#!/usr/bin/env python3
"""Tests for run_gates.py.

No test touches a real gear, a real pyright, or the real API database. Every test builds a
throwaway repo in a tempfile.TemporaryDirectory() and points the runner at stub gate scripts
by patching RUNNER.scripts_dir with unittest.mock.patch.object.
"""
import contextlib
import importlib.util
import io
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

CHECKER_PATH = Path(__file__).with_name('run_gates.py')
MODULE_SPEC = importlib.util.spec_from_file_location('run_gates', CHECKER_PATH)
RUNNER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(RUNNER)

GEAR = 'spurgear'

STUB_NAMES = {
    'input_read': 'check_input_read.py',
    'contract': 'check_contract.py',
    'step_calls': 'check_step_calls.py',
    'anchors': 'check_anchors.py',
    'api_calls': 'check_api_calls.py',
    'pyright': 'pyright_check.py',
    'novel_types': 'check_novel_types.py',
}

STUB_HEADLINE = {
    'input_read': 'input-read check: OK (0 inputs)',
    'contract': 'contract check: OK',
    'step_calls': 'step-call check: OK (0 named calls present, no stubs, no shared-point misuse)',
    'anchors': 'anchor check: OK (0 anchors defined, 0 cited, 0 files)',
    'api_calls': 'api-call check: OK',
    'pyright': 'pyright_check spurgear.generated.py: 0 blocking, 0 review, 0 ignored',
    'novel_types': 'novel-type check: 0 complaint(s) no shipped gear produces -- triage each',
}


def make_stub(directory, name, exit_code=0, stdout='', stderr='', sleep=0.0,
              argv_marker=None):
    """Write a gate script that prints `stdout`, prints `stderr` to stderr, optionally sleeps,
    and exits `exit_code`. If `argv_marker` is given, writes sys.argv (minus argv[0]) as JSON
    to that path, so a test can assert on the arguments the runner passed."""
    path = os.path.join(directory, name)
    lines = ['#!/usr/bin/env python3', 'import sys']
    if argv_marker:
        lines.append('import json')
        lines.append('with open(%r, "w") as fh:' % argv_marker)
        lines.append('    json.dump(sys.argv[1:], fh)')
    if sleep:
        lines.append('import time; time.sleep(%r)' % sleep)
    if stdout:
        for line in stdout.splitlines():
            lines.append('print(%r)' % line)
    if stderr:
        for line in stderr.splitlines():
            lines.append('print(%r, file=sys.stderr)' % line)
    lines.append('sys.exit(%r)' % exit_code)
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    st = os.stat(path)
    os.chmod(path, st.st_mode | stat.S_IEXEC)
    return path


def make_all_stubs(scripts, overrides=None):
    """Write default-passing stubs for every gate script name, applying per-key overrides
    (dict of key -> kwargs for make_stub). Returns the scripts dir."""
    overrides = overrides or {}
    for key, name in STUB_NAMES.items():
        kwargs = dict(exit_code=0, stdout=STUB_HEADLINE[key])
        kwargs.update(overrides.get(key, {}))
        make_stub(scripts, name, **kwargs)
    return scripts


def make_repo(tmp, gear=GEAR, *, candidate='x = 1\n', steps='# steps\n', contract=None):
    """Create <tmp>/.tmp/<gear>.generated.py and, when the argument is not None,
    spec/<gear>/steps.md and spec/<gear>/contract.json. Return the root path."""
    root = tmp
    os.makedirs(os.path.join(root, '.tmp'), exist_ok=True)
    os.makedirs(os.path.join(root, 'spec', gear), exist_ok=True)
    cand_path = os.path.join(root, '.tmp', '%s.generated.py' % gear)
    with open(cand_path, 'w') as fh:
        fh.write(candidate)
    if steps is not None:
        with open(os.path.join(root, 'spec', gear, 'steps.md'), 'w') as fh:
            fh.write(steps)
    if contract is not None:
        with open(os.path.join(root, 'spec', gear, 'contract.json'), 'w') as fh:
            fh.write(contract)
    return root


def run(root, scripts, *argv):
    """patch scripts_dir -> scripts, capture stdout with contextlib.redirect_stdout, call
    RUNNER.main([...]), return (exit_code, text, parsed_json)."""
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


class BaseGateTest(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self.tmp = self._tmpdir.name
        self.scripts = os.path.join(self.tmp, 'scripts')
        os.makedirs(self.scripts, exist_ok=True)


class HappyPathTests(BaseGateTest):
    def test_all_gates_pass(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['verdict'], 'pass')
        gate_keys = [g['key'] for g in parsed['gates']]
        self.assertEqual(len(parsed['gates']), 8)
        for g in parsed['gates']:
            self.assertEqual(g['status'], 'pass', msg=g)
        self.assertEqual(parsed['classification'], [])
        # one line per gate in the human report
        for key in gate_keys:
            self.assertIn(key, text)

    def test_json_line_is_parseable(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        json_out = os.path.join(self.tmp, 'v.json')
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--json-out', json_out)
        last_line = [ln for ln in text.splitlines() if ln.strip()][-1]
        self.assertTrue(last_line.startswith(RUNNER.JSON_MARKER))
        self.assertEqual(parsed['schema'], 1)
        with open(json_out) as fh:
            written = json.load(fh)
        self.assertEqual(written, parsed)

    def test_format_json_prints_only_json(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--format', 'json')
        lines = [ln for ln in text.splitlines() if ln.strip()]
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith(RUNNER.JSON_MARKER))


class ParseBarrierTests(BaseGateTest):
    def test_syntax_error_skips_candidate_gates(self):
        root = make_repo(self.tmp, candidate='def (:\n', contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 1)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['parse']['status'], 'fail')
        for key in ('input_read', 'contract', 'step_calls', 'api_calls', 'pyright',
                    'novel_types'):
            self.assertEqual(by_key[key]['status'], 'skip')
            self.assertIn('does not parse', by_key[key]['skip_reason'])
        self.assertEqual(by_key['anchors']['status'], 'pass')
        emit_rows = [c for c in parsed['classification'] if c['gate'] == 'parse']
        self.assertEqual(len(emit_rows), 1)
        self.assertEqual(emit_rows[0]['fault'], 'emit')
        self.assertTrue(emit_rows[0]['certain'])

    def test_parse_failure_message_carries_line(self):
        root = make_repo(self.tmp, candidate='x = 1\ny = 2\ndef (:\n', contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertIn('L3', by_key['parse']['headline'])


class ContinueTests(BaseGateTest):
    def test_failure_does_not_stop_later_gates(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'input_read': dict(exit_code=1, stdout='input-read check: BLOCKING (1)'),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 1)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['input_read']['status'], 'fail')
        self.assertEqual(by_key['api_calls']['status'], 'pass')
        self.assertIsNotNone(by_key['api_calls']['duration_s'])
        self.assertEqual(by_key['pyright']['status'], 'pass')
        self.assertIsNotNone(by_key['pyright']['duration_s'])

    def test_fail_fast_stops_scheduling(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'input_read': dict(exit_code=1, stdout='input-read check: BLOCKING (1)'),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--fail-fast')
        self.assertEqual(exit_code, 1)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['api_calls']['status'], 'skip')
        self.assertIn('--fail-fast', by_key['api_calls']['skip_reason'])
        self.assertEqual(by_key['pyright']['status'], 'skip')
        self.assertIn('--fail-fast', by_key['pyright']['skip_reason'])


class ContractTests(BaseGateTest):
    def test_missing_manifest_skips_and_passes(self):
        root = make_repo(self.tmp)  # no contract
        marker = os.path.join(self.tmp, 'contract-invoked.marker')
        make_all_stubs(self.scripts, overrides={
            'contract': dict(exit_code=0, stdout='contract check: OK', argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['contract']['status'], 'skip')
        self.assertIn('spec/%s/contract.json' % GEAR, by_key['contract']['skip_reason'])
        self.assertIn('--require-contract', by_key['contract']['skip_reason'])
        self.assertFalse(os.path.exists(marker))

    def test_require_contract_is_setup_error(self):
        root = make_repo(self.tmp)  # no contract
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--require-contract')
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['verdict'], 'setup_error')

    def test_manifest_present_runs_gate(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        by_key = {g['key']: g for g in parsed['gates']}
        cmd = by_key['contract']['command']
        manifest_idx = next(i for i, a in enumerate(cmd) if a.endswith('contract.json'))
        candidate_idx = next(i for i, a in enumerate(cmd) if a.endswith('.generated.py'))
        self.assertLess(manifest_idx, candidate_idx)


class SetupErrorTests(BaseGateTest):
    def test_gate_exit_two_is_setup_error(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'pyright': dict(exit_code=2, stdout='ERROR: pyright did not run'),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['pyright']['status'], 'error')
        self.assertEqual(by_key['pyright']['fault'], 'setup')
        self.assertEqual(by_key['novel_types']['status'], 'pass')

    def test_missing_candidate(self):
        root = make_repo(self.tmp, contract='{}')
        os.remove(os.path.join(root, '.tmp', '%s.generated.py' % GEAR))
        marker = os.path.join(self.tmp, 'never.marker')
        make_all_stubs(self.scripts, overrides={
            'input_read': dict(argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertFalse(os.path.exists(marker))

    def test_missing_steps_md(self):
        root = make_repo(self.tmp, contract='{}')
        os.remove(os.path.join(root, 'spec', GEAR, 'steps.md'))
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertIn('spec/%s/steps.md' % GEAR, text)

    def test_missing_gate_script(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        os.remove(os.path.join(self.scripts, 'check_anchors.py'))
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertIn('check_anchors.py', text)

    def test_timeout_is_error(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'pyright': dict(sleep=5),
        })
        start = __import__('time').monotonic()
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--timeout', '1')
        elapsed = __import__('time').monotonic() - start
        self.assertEqual(exit_code, 2)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['pyright']['status'], 'error')
        self.assertLess(elapsed, 3)


class SkipMissingStepsTests(BaseGateTest):
    """--skip-missing-steps: the generate stage runs from the prose spec, so a gear with no
    compiled spec/<gear>/steps.md skips the step_calls gate instead of erroring out."""

    def test_missing_steps_skips_step_calls(self):
        root = make_repo(self.tmp, steps=None, contract='{}')
        marker = os.path.join(self.tmp, 'step-calls-invoked.marker')
        make_all_stubs(self.scripts, overrides={
            'step_calls': dict(argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--skip-missing-steps')
        self.assertEqual(exit_code, 0)
        self.assertEqual(parsed['verdict'], 'pass')
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['step_calls']['status'], 'skip')
        self.assertIn('spec/%s/steps.md' % GEAR, by_key['step_calls']['skip_reason'])
        self.assertIn('--skip-missing-steps', by_key['step_calls']['skip_reason'])
        self.assertFalse(os.path.exists(marker))
        for key in ('parse', 'input_read', 'contract', 'anchors', 'api_calls', 'pyright',
                    'novel_types'):
            self.assertEqual(by_key[key]['status'], 'pass', msg=by_key[key])

    def test_missing_steps_without_flag_still_errors(self):
        root = make_repo(self.tmp, steps=None, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 2)
        self.assertEqual(parsed['verdict'], 'setup_error')
        self.assertIn('spec/%s/steps.md' % GEAR, text)

    def test_flag_with_steps_present_runs_gate(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--skip-missing-steps')
        self.assertEqual(exit_code, 0)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['step_calls']['status'], 'pass')

    def test_api_fault_classification_survives_missing_step_list(self):
        root = make_repo(self.tmp, steps=None, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'api_calls': dict(
                exit_code=1,
                stdout="api-call check: BLOCKING (1)\n"
                       "  x.py:1 calls 'setExtentDefinition(' -- no such name, ..."),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--skip-missing-steps')
        self.assertEqual(exit_code, 1)
        row = next(c for c in parsed['classification'] if c['gate'] == 'api_calls')
        self.assertEqual(row['fault'], 'emit')
        self.assertIn('could not read the step list', row['why'])


class ClassificationTests(BaseGateTest):
    def test_deterministic_mapping(self):
        cases = [
            ('parse', 'emit'), ('pyright', 'emit'), ('input_read', 'emit'),
            ('step_calls', 'emit'), ('anchors', 'compile'), ('contract', 'judgment'),
        ]
        for failing_key, expected_fault in cases:
            with self.subTest(failing_key=failing_key):
                tmp = tempfile.TemporaryDirectory()
                self.addCleanup(tmp.cleanup)
                scripts = os.path.join(tmp.name, 'scripts')
                os.makedirs(scripts, exist_ok=True)
                candidate = 'def (:\n' if failing_key == 'parse' else 'x = 1\n'
                root = make_repo(tmp.name, candidate=candidate, contract='{}')
                overrides = {}
                if failing_key != 'parse':
                    overrides[failing_key] = dict(exit_code=1, stdout='%s: BLOCKING (1)'
                                                   % failing_key)
                make_all_stubs(scripts, overrides=overrides)
                exit_code, text, parsed = run(root, scripts, GEAR)
                rows = [c for c in parsed['classification'] if c['gate'] == failing_key]
                self.assertEqual(len(rows), 1, msg=parsed['classification'])
                self.assertEqual(rows[0]['fault'], expected_fault)
                if failing_key == 'contract':
                    self.assertFalse(rows[0]['certain'])
                else:
                    self.assertTrue(rows[0]['certain'])

    def test_api_fault_is_emit_when_step_list_is_silent(self):
        root = make_repo(self.tmp, contract='{}', steps='# steps\nno calls named here.\n')
        make_all_stubs(self.scripts, overrides={
            'api_calls': dict(
                exit_code=1,
                stdout="api-call check: BLOCKING (1)\n"
                       "  x.py:1 calls 'setExtentDefinition(' -- no such name in the "
                       "Fusion API database, ..."),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        row = next(c for c in parsed['classification'] if c['gate'] == 'api_calls')
        self.assertEqual(row['fault'], 'emit')
        self.assertTrue(row['certain'])

    def test_api_fault_needs_judgment_when_step_list_names_it(self):
        root = make_repo(self.tmp, contract='{}',
                          steps='# steps\nCall `setExtentDefinition(` here.\n')
        make_all_stubs(self.scripts, overrides={
            'api_calls': dict(
                exit_code=1,
                stdout="api-call check: BLOCKING (1)\n"
                       "  x.py:1 calls 'setExtentDefinition(' -- no such name in the "
                       "Fusion API database, ..."),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        row = next(c for c in parsed['classification'] if c['gate'] == 'api_calls')
        self.assertEqual(row['fault'], 'judgment')
        self.assertFalse(row['certain'])
        self.assertIn('setExtentDefinition', row['why'])

    def test_unreadable_step_list_falls_back_to_emit(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'api_calls': dict(
                exit_code=1,
                stdout="api-call check: BLOCKING (1)\n"
                       "  x.py:1 calls 'setExtentDefinition(' -- no such name, ..."),
        })
        with mock.patch.object(RUNNER, 'steps_named_calls', return_value=None):
            exit_code, text, parsed = run(root, self.scripts, GEAR)
        row = next(c for c in parsed['classification'] if c['gate'] == 'api_calls')
        self.assertEqual(row['fault'], 'emit')
        self.assertIn('could not read the step list', row['why'])


class AdvisoryTests(BaseGateTest):
    def test_findings_do_not_fail_the_run(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts, overrides={
            'novel_types': dict(
                exit_code=0,
                stdout='novel-type check: 2 complaint(s) no shipped gear produces -- triage each'),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        self.assertEqual(exit_code, 0)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['novel_types']['status'], 'note')
        self.assertEqual(parsed['counts']['advisory_findings'], 2)
        self.assertTrue(any(c['gate'] == 'novel_types' and c['fault'] == 'judgment'
                             for c in parsed['classification']))

    def test_gate_novel_types_makes_it_blocking(self):
        root = make_repo(self.tmp, contract='{}')
        marker = os.path.join(self.tmp, 'novel-argv.json')
        make_all_stubs(self.scripts, overrides={
            'novel_types': dict(exit_code=1, stdout='novel-type check: 1 complaint(s) ...',
                                 argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--gate-novel-types')
        self.assertEqual(exit_code, 1)
        with open(marker) as fh:
            argv = json.load(fh)
        self.assertIn('--gate', argv)

    def test_no_advisory_skips_it(self):
        root = make_repo(self.tmp, contract='{}')
        marker = os.path.join(self.tmp, 'never.marker')
        make_all_stubs(self.scripts, overrides={
            'novel_types': dict(argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--no-advisory')
        self.assertFalse(os.path.exists(marker))
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['novel_types']['status'], 'skip')


class SelectionTests(BaseGateTest):
    def test_only_runs_named_gates(self):
        root = make_repo(self.tmp, contract='{}')
        marker = os.path.join(self.tmp, 'step-calls.marker')
        make_all_stubs(self.scripts, overrides={
            'step_calls': dict(argv_marker=marker),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--only', 'step_calls')
        self.assertTrue(os.path.exists(marker))
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['step_calls']['status'], 'pass')
        for key in GATE_TITLES_MINUS('step_calls'):
            self.assertEqual(by_key[key]['status'], 'skip')
            self.assertIn('not selected', by_key[key]['skip_reason'])

    def test_only_rejects_unknown_key(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--only', 'nope')
        self.assertEqual(exit_code, 2)


def GATE_TITLES_MINUS(*exclude):
    return [k for k in RUNNER.GATE_ORDER if k not in exclude]


class CommandShapeTests(BaseGateTest):
    def test_commands_are_root_relative(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        for g in parsed['gates']:
            for arg in g['command'][1:]:
                self.assertFalse(arg.startswith('/'), msg=(g['key'], arg))

    def test_cwd_is_repo_root(self):
        root = make_repo(self.tmp, contract='{}')
        cwd_marker = os.path.join(self.tmp, 'cwd.txt')
        stub_path = os.path.join(self.scripts, 'check_anchors.py')
        make_all_stubs(self.scripts)
        with open(stub_path, 'w') as fh:
            fh.write(
                '#!/usr/bin/env python3\n'
                'import os, sys\n'
                'with open(%r, "w") as fh:\n'
                '    fh.write(os.getcwd())\n'
                'print(%r)\n'
                'sys.exit(0)\n' % (cwd_marker, STUB_HEADLINE['anchors']))
        st = os.stat(stub_path)
        os.chmod(stub_path, st.st_mode | stat.S_IEXEC)
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        with open(cwd_marker) as fh:
            recorded = fh.read()
        self.assertEqual(os.path.realpath(recorded), os.path.realpath(root))


class OutputTests(BaseGateTest):
    def test_failing_gate_body_is_in_text_report(self):
        root = make_repo(self.tmp, contract='{}')
        detail = ('step-call check: BLOCKING (3)\n'
                   '  detail one\n'
                   '  detail two\n'
                   '  detail three')
        make_all_stubs(self.scripts, overrides={
            'step_calls': dict(exit_code=1, stdout=detail),
        })
        exit_code, text, parsed = run(root, self.scripts, GEAR)
        for line in ('detail one', 'detail two', 'detail three'):
            self.assertIn(line, text)
        by_key = {g['key']: g for g in parsed['gates']}
        self.assertEqual(by_key['step_calls']['stdout'].rstrip('\n'), detail)

    def test_json_out_writes_file(self):
        root = make_repo(self.tmp, contract='{}')
        make_all_stubs(self.scripts)
        json_out = os.path.join(self.tmp, 'out.json')
        exit_code, text, parsed = run(root, self.scripts, GEAR, '--json-out', json_out)
        with open(json_out) as fh:
            written = json.load(fh)
        self.assertEqual(written, parsed)


if __name__ == '__main__':
    unittest.main()
