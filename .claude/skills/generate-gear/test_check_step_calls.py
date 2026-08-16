#!/usr/bin/env python3
"""Regression tests for the step-call coverage gate."""
import contextlib
import importlib.util
import io
import os
import sys
import tempfile
import unittest
import warnings
from pathlib import Path
from unittest import mock


CHECKER_PATH = Path(__file__).with_name('check_step_calls.py')
MODULE_SPEC = importlib.util.spec_from_file_location('check_step_calls', CHECKER_PATH)
CHECKER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(CHECKER)

API_CHECKER_PATH = Path(__file__).with_name('check_api_calls.py')
API_MODULE_SPEC = importlib.util.spec_from_file_location('check_api_calls', API_CHECKER_PATH)
API_CHECKER = importlib.util.module_from_spec(API_MODULE_SPEC)
API_MODULE_SPEC.loader.exec_module(API_CHECKER)

COMPILE_CHECKER_PATH = Path(__file__).with_name('check_compile.py')
COMPILE_MODULE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_CHECKER_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_MODULE_SPEC)
COMPILE_MODULE_SPEC.loader.exec_module(COMPILE_CHECKER)


class CheckStepCallsTest(unittest.TestCase):
    def run_checker(self, steps, candidate):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            steps_path = root / 'steps.md'
            candidate_path = root / 'candidate.py'
            steps_path.write_text(steps)
            candidate_path.write_text(candidate)
            output = io.StringIO()
            with warnings.catch_warnings(), contextlib.redirect_stdout(output):
                warnings.simplefilter('ignore', ResourceWarning)
                result = CHECKER.main(['check_step_calls.py', str(steps_path), str(candidate_path)])
            return result, output.getvalue()

    def test_forbidden_call_is_not_required(self):
        steps = 'Call `safeCall()`. Do not read the direction via `getTangent(0)`.'

        self.assertEqual(CHECKER.named_calls(steps), {'safeCall'})
        result, output = self.run_checker(steps, 'safeCall()\n')

        self.assertEqual(result, 0, output)
        self.assertNotIn('getTangent', output)

    def test_comment_only_call_still_fails_coverage(self):
        steps = 'Call `safeCall()`.'

        result, output = self.run_checker(steps, '# safeCall()\n')

        self.assertEqual(result, 1)
        self.assertIn('textual match exists, but it is not an executable call', output)

    def test_dotted_call_does_not_match_local_helper(self):
        steps = 'Call `sketch.addByTwoPoints(start, end)`.'
        candidate = "def addByTwoPoints(start, end):\n    pass\n\naddByTwoPoints(None, None)\n"

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1)
        self.assertIn("receiver.addByTwoPoints('", output)

    def test_dotted_call_requires_an_attribute_call(self):
        steps = 'Call `sketch.addByTwoPoints(start, end)`.'

        result, output = self.run_checker(steps, 'sketch.addByTwoPoints(None, None)\n')

        self.assertEqual(result, 0, output)


class CheckApiCallsTest(unittest.TestCase):
    def run_checker(self, candidate, framework):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate_path = root / 'candidate.py'
            framework_path = root / 'framework'
            framework_path.mkdir()
            (framework_path / 'helpers.py').write_text(framework)
            candidate_path.write_text(candidate)
            output = io.StringIO()
            with mock.patch.object(
                    API_CHECKER.fusion_api, 'lookup_many',
                    side_effect=lambda names: {name: [] for name in names}), \
                    mock.patch.object(API_CHECKER.fusion_api, 'similar', return_value=[]), \
                    mock.patch.object(API_CHECKER.fusion_api, 'query_script',
                                      return_value='/hermetic/fusion-query-api.py'), \
                    contextlib.redirect_stdout(output):
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', ResourceWarning)
                    with mock.patch.object(sys, 'argv', [
                            'check_api_calls.py', str(candidate_path), '--framework', str(framework_path)]):
                        result = API_CHECKER.main()
            return result, output.getvalue()

    def test_framework_function_is_not_a_target_method(self):
        result, output = self.run_checker(
            'class Candidate:\n    def run(self):\n        self.to_cm(10)\n',
            'def to_cm(value):\n    return value\n')

        self.assertEqual(result, 1)
        self.assertIn("calls 'to_cm('", output)

    def test_target_defined_method_is_allowed(self):
        result, output = self.run_checker(
            'class Candidate:\n'
            '    def helper(self):\n'
            '        return self.helper()\n',
            'def unrelated(value):\n    return value\n')

        self.assertEqual(result, 0, output)


class CheckCompileTest(unittest.TestCase):
    def run_checker(self, provenance, from_line='**From:** `instructions.md:1`'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'spec' / 'gear').mkdir(parents=True)
            (root / 'proof' / 'gear').mkdir(parents=True)
            (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)
            for path in (
                    root / 'spec' / 'gear' / 'instructions.md',
                    root / 'spec' / 'gear' / 'fusion.md',
                    root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md'):
                path.write_text('source\n')
            (root / 'proof' / 'gear' / 'proof.go').write_text('func stepOne() {}\n')
            table = '\n'.join((
                '| Source | Blob hash |',
                '|---|---|',
                provenance,
            ))
            steps = (
                '# Steps\n\n'
                '## S1 `[GO]` One — `stepOne`\n\n'
                'Build the thing.\n\n'
                '%s\n\n'
                '## Provenance\n\n'
                '%s\n' % (from_line, table))
            (root / 'spec' / 'gear' / 'steps.md').write_text(steps)
            output = io.StringIO()
            prior = os.getcwd()
            try:
                os.chdir(root)
                with mock.patch.object(COMPILE_CHECKER.fusion_api, 'lookup_many', return_value={}), \
                        contextlib.redirect_stdout(output):
                    result = COMPILE_CHECKER.main(['check_compile.py', 'gear'])
            finally:
                os.chdir(prior)
            return result, output.getvalue()

    def complete_provenance(self):
        return (
            '| `spec/gear/instructions.md` | `aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa` |\n'
            '| `spec/gear/fusion.md` | `aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa` |\n'
            '| `.claude/skills/generate-gear/PLAYBOOK.md` | `aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa` |'
        )

    def test_all_declared_compile_inputs_are_required(self):
        provenance = self.complete_provenance().splitlines()[:-1]

        result, output = self.run_checker('\n'.join(provenance))

        self.assertEqual(result, 1)
        self.assertIn('provenance omits required source', output)

    def test_each_step_requires_a_from_citation(self):
        result, output = self.run_checker(self.complete_provenance(), from_line='')

        self.assertEqual(result, 1)
        self.assertIn('has no nonempty **From:** citation', output)


if __name__ == '__main__':
    unittest.main()
