"""Focused checks for compact retry feedback from structured gate reports."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import retry_report


def full_report(skill, payload):
    marker = retry_report.MARKERS[skill]
    runner = retry_report.RUNNERS[skill]
    return '{} {}\nverdict: FAIL\n{}{}\n'.format(
        runner, payload['gear'], marker, json.dumps(payload))


class RetryReportTests(unittest.TestCase):
    def test_compile_keeps_advisories_failure_and_classification(self):
        payload = {
            'schema': 1, 'gear': 'bevelgear', 'verdict': 'fail', 'exit_code': 1,
            'stages': [
                {'key': 'compile', 'status': 'pass', 'headline': 'compile check: OK',
                 'stdout': 'coverage: 2 unclaimed\nunverified: project\ncompile check: OK\n'},
                {'key': 'playbook', 'status': 'pass', 'headline': 'playbook check: OK',
                 'stdout': 'playbook check: OK\n'},
                {'key': 'step_calls', 'status': 'fail', 'headline': 'step-call check: BLOCKING',
                 'stdout': 'missing deleteMe\n', 'stderr': '', 'fault': 'DRAFT FAULT'},
                {'key': 'proof', 'status': 'pass', 'headline': 'proof: OK',
                 'stdout': 'large successful proof transcript\n'},
            ],
            'handoff': {'missing_call_names': ['deleteMe']},
            'iteration_mode': True, 'iteration_base': 'a1b2c3',
            'effective_proof_scope': 'selected', 'proof_is_complete': False,
        }
        view = retry_report.compact(full_report('compile-gear', payload),
                                    'compile-gear', 'bevelgear')
        self.assertIn('PASS proof: proof: OK', view)
        self.assertIn('coverage: 2 unclaimed', view)
        self.assertIn('unverified: project', view)
        self.assertIn('missing deleteMe', view)
        self.assertIn('step_calls: DRAFT FAULT', view)
        self.assertIn('"missing_call_names": ["deleteMe"]', view)
        self.assertIn('iteration base: a1b2c3', view)
        self.assertIn('proof is complete: False', view)
        self.assertEqual(view.count('compile check: OK'), 1)
        self.assertNotIn('successful proof transcript', view)
        self.assertNotIn('COMPILE_GATES_JSON:', view)

    def test_emit_keeps_advisory_and_one_classification(self):
        payload = {
            'schema': 1, 'gear': 'spurgear', 'verdict': 'fail', 'exit_code': 1,
            'gates': [
                {'key': 'parse', 'status': 'pass', 'headline': 'parse: OK',
                 'stdout': 'parse: OK\n'},
                {'key': 'api_calls', 'status': 'fail', 'headline': 'api calls: BLOCKING',
                 'stdout': 'bad call\n', 'fault': 'judgment'},
                {'key': 'novel_types', 'status': 'note', 'advisory': True,
                 'headline': 'novel type found', 'stdout': 'review type 1\n',
                 'fault': 'judgment'},
            ],
            'classification': [
                {'gate': 'api_calls', 'fault': 'judgment', 'why': 'check the step'},
                {'gate': 'novel_types', 'fault': 'judgment', 'why': 'triage finding'},
            ],
        }
        view = retry_report.compact(full_report('emit-gear', payload), 'emit-gear', 'spurgear')
        self.assertIn('FAIL api_calls', view)
        self.assertIn('NOTE novel_types', view)
        self.assertIn('review type 1', view)
        self.assertIn('api_calls: judgment: check the step', view)
        self.assertEqual(view.count('first-pass fault classification:'), 1)
        self.assertNotIn('GATES_JSON:', view)

    def test_runner_report_must_have_a_matching_failed_verdict(self):
        payload = {'schema': 1, 'gear': 'spurgear', 'verdict': 'fail', 'exit_code': 1,
                   'gates': [{'key': 'parse', 'status': 'fail', 'headline': 'parse failed'}]}
        report = full_report('emit-gear', payload)
        with self.assertRaisesRegex(retry_report.RetryReportError, 'wrong schema or gear'):
            retry_report.compact(report, 'emit-gear', 'helicalgear')
        payload['verdict'], payload['exit_code'] = 'pass', 0
        with self.assertRaisesRegex(retry_report.RetryReportError, 'failed draft'):
            retry_report.compact(full_report('emit-gear', payload), 'emit-gear', 'spurgear')
        with self.assertRaisesRegex(retry_report.RetryReportError, 'structured verdict'):
            retry_report.compact('run_gates: spurgear\nverdict: FAIL\n',
                                 'emit-gear', 'spurgear')
        with self.assertRaisesRegex(retry_report.RetryReportError, 'structured verdict'):
            retry_report.compact(report, 'compile-gear', 'spurgear')

    def test_handoff_failure_survives_when_stages_pass(self):
        payload = {
            'schema': 1, 'gear': 'spurgear', 'verdict': 'fail', 'exit_code': 1,
            'stages': [{'key': 'compile', 'status': 'pass', 'headline': 'compile check: OK',
                        'stdout': 'compile check: OK\n'}],
            'handoff': {'status': 'review_required', 'review_error': 'missing source decision'},
        }
        view = retry_report.compact(full_report('compile-gear', payload),
                                    'compile-gear', 'spurgear')
        self.assertIn('PASS compile', view)
        self.assertIn('missing source decision', view)

    def test_cli_reads_full_report_and_prints_only_view(self):
        payload = {'schema': 1, 'gear': 'spurgear', 'verdict': 'fail', 'exit_code': 1,
                   'gates': [{'key': 'parse', 'status': 'fail', 'headline': 'parse failed',
                              'stdout': 'SyntaxError\n'}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'gates.txt'
            path.write_text(full_report('emit-gear', payload), encoding='utf-8')
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = retry_report.main(['emit-gear', 'spurgear', str(path)])
        self.assertEqual(code, 0)
        self.assertIn('SyntaxError', output.getvalue())
        self.assertNotIn('GATES_JSON:', output.getvalue())


if __name__ == '__main__':
    unittest.main()
