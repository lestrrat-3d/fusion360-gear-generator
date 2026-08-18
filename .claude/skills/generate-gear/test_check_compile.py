#!/usr/bin/env python3
"""Regression tests for the compile-stage gate."""
import contextlib
import importlib.util
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


COMPILE_CHECKER_PATH = Path(__file__).with_name('check_compile.py')
COMPILE_MODULE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_CHECKER_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_MODULE_SPEC)
COMPILE_MODULE_SPEC.loader.exec_module(COMPILE_CHECKER)


class CheckCompileTest(unittest.TestCase):
    SOURCE_PATHS = (
        'spec/gear/instructions.md',
        'spec/gear/fusion.md',
        '.claude/skills/generate-gear/PLAYBOOK.md',
    )

    def provenance_rows(self, root, paths=None):
        paths = paths or self.SOURCE_PATHS
        return '\n'.join(
            '| `%s` | `%s` |' % (path, COMPILE_CHECKER.blob_hash(str(root / path)))
            for path in paths)

    def run_checker(self, provenance=None, from_line='**From:** `spec/gear/instructions.md` L1',
                    proof_body=None, proof_filename='proof_test.go', step_body=None,
                    include_fusion=True, auxiliary=False, mutate_auxiliary=False,
                    api_lookup=None, unverified_findings=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'spec' / 'gear').mkdir(parents=True)
            (root / 'proof' / 'gear').mkdir(parents=True)
            (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)
            instruction_text = 'source\nline two\nline three\n'
            if auxiliary:
                instruction_text = 'source\nSee `trace.md` for details.\nline three\n'
                (root / 'spec' / 'gear' / 'trace.md').write_text('trace\n')
            (root / 'spec' / 'gear' / 'instructions.md').write_text(instruction_text)
            if include_fusion:
                (root / 'spec' / 'gear' / 'fusion.md').write_text(
                    'source\nline two\nline three\n')
            (root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md').write_text(
                'source\nline two\nline three\n')
            if provenance is None:
                paths = self.SOURCE_PATHS if include_fusion else (
                    self.SOURCE_PATHS[0], self.SOURCE_PATHS[2])
                if auxiliary:
                    paths = (*paths[:-1], 'spec/gear/trace.md', paths[-1])
                provenance = self.provenance_rows(root, paths)
            if proof_body is None:
                proof_body = (
                    'func TestOne(t *testing.T) {\n'
                    '    proofkit.Run(t, cases(\n'
                    '        gear{name: "one"},\n'
                    '    ), stepOne)\n'
                    '}\n\n'
                    'func stepOne() {}\n')
            (root / 'proof' / 'gear' / proof_filename).write_text(proof_body)
            table = '\n'.join((
                '| Source | Blob hash |',
                '|---|---|',
                provenance,
            ))
            if step_body is None:
                step_body = (
                    '## S1 `[GO]` One — `stepOne`\n\n'
                    'Build the thing.\n\n'
                    '%s\n\n' % from_line)
            steps = (
                '# Steps\n\n'
                '%s'
                '## Provenance\n\n'
                '%s\n' % (step_body, table))
            (root / 'spec' / 'gear' / 'steps.md').write_text(steps)
            if mutate_auxiliary:
                (root / 'spec' / 'gear' / 'trace.md').write_text('changed\n')
            output = io.StringIO()
            prior = os.getcwd()
            try:
                os.chdir(root)
                lookup_patch = mock.patch.object(
                    COMPILE_CHECKER.fusion_api, 'lookup_many',
                    return_value={} if api_lookup is None else api_lookup)
                findings_patch = mock.patch.object(
                    COMPILE_CHECKER.fusion_api, 'unverified_findings',
                    return_value=[] if unverified_findings is None else unverified_findings)
                with lookup_patch, findings_patch, contextlib.redirect_stdout(output):
                    result = COMPILE_CHECKER.main(['check_compile.py', 'gear'])
            finally:
                os.chdir(prior)
            return result, output.getvalue()

    def solid_proof(self, run_call):
        """A proof whose 2D run is in order and whose solid run is run_call."""
        return (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func TestSolid(t *testing.T) {\n'
            '    %s\n'
            '}\n\n'
            'func stepOne() {}\n' % run_call)

    def test_step_named_build_argument_is_accepted(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(t, solidCases, stepOne, assertSolid)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_misnamed_build_argument_is_blocking(self):
        proof_body = self.solid_proof(
            'proofkit3d.Run(t, solidCases, buildSolid, assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go registers buildSolid as a proof run\'s build argument, '
            'but that argument must be a step<Title> function so a step can claim it', output)

    def test_misnamed_build_argument_in_run_solid_is_blocking(self):
        proof_body = self.solid_proof(
            'proofkit3d.RunSolid(t, solidCases, buildSolid, assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('registers buildSolid as a proof run\'s build argument', output)

    def test_misnamed_build_argument_in_run_with_gate_is_blocking(self):
        proof_body = self.solid_proof(
            'proofkit3d.RunWithGate(t, solidCases, buildSolid, proofkit3d.RequireSolid, '
            'assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('registers buildSolid as a proof run\'s build argument', output)

    def test_misnamed_2d_build_argument_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func TestTwo(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "two"}), buildProfile)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('registers buildProfile as a proof run\'s build argument', output)

    def test_one_misnamed_build_is_reported_once_per_file(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(t, solidCases, buildSolid, assertSolid)\n'
            '}\n\n'
            'func TestBore(t *testing.T) {\n'
            '    proofkit3d.RunSolid(t, boreCases, buildSolid, assertSolid)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertEqual(output.count('registers buildSolid'), 1)

    def test_unreachable_misnamed_build_argument_is_not_counted(self):
        proof_body = self.solid_proof(
            'if false {\n'
            '        proofkit3d.RunSolid(t, solidCases, buildSolid, assertSolid)\n'
            '    }')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertNotIn('buildSolid', output)

    def test_early_returned_misnamed_build_argument_is_not_counted(self):
        proof_body = self.solid_proof(
            'if t != nil { return }\n'
            '    proofkit3d.Run(t, solidCases, buildSolid, assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertNotIn('buildSolid', output)

    def test_assertion_and_gate_arguments_are_not_step_functions(self):
        src = (
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.RunWithGate(t, solidCases, stepSolid, proofkit3d.RequireSolid, '
            'assertSolid)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, {'stepSolid'})
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [])

    def test_build_argument_expression_is_labelled_on_one_line(self):
        src = (
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(t, solidCases, func(t *testing.T) []*decad.Body {\n'
            '        return nil\n'
            '    }, assertSolid)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(unreadable, [])
        self.assertEqual(len(misnamed), 1)
        self.assertNotIn('\n', misnamed[0])
        self.assertTrue(misnamed[0].startswith('func(t *testing.T) []*decad.Body {'))

    # A run whose argument list the parse cannot read to the build slot is reported, because
    # such a run can compile: Go lets one multi-value call supply a whole argument list, so
    # proofkit3d.Run(runArgs(t)) vets clean and still parses to a single argument here.
    # Skipping it silently would let its build argument escape the step<Title> check.

    def test_multi_value_forwarded_run_is_blocking(self):
        proof_body = self.solid_proof('proofkit3d.Run(runArgs(t))')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go runs proofkit3d.Run(runArgs(t)), whose arguments could '
            'not be read, so its build argument cannot be checked', output)

    def test_multi_value_forwarded_2d_run_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func TestTwo(t *testing.T) {\n'
            '    proofkit.Run(runArgs(t))\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs proofkit.Run(runArgs(t)), whose arguments could not be read', output)

    def test_unclosed_run_call_is_reported_not_dropped(self):
        """The argument list that never closes is reported too.

        No Go source reaches this through the checker: a Test body is only read when its
        delimiters nest, which already gives every open paren inside it a close. The branch
        stays as the parse's own guard, and this test holds it to reporting rather than
        skipping by making the paren lookup fail.
        """
        src = (
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(t, solidCases, stepSolid, assertSolid)\n'
            '}\n')
        real_match = COMPILE_CHECKER.matching_delimiter

        def no_close_paren(text, start):
            return None if text[start] == '(' else real_match(text, start)

        with mock.patch.object(COMPILE_CHECKER, 'matching_delimiter', no_close_paren):
            registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(len(unreadable), 1)
        self.assertTrue(unreadable[0].startswith('proofkit3d.Run(t, solidCases, stepSolid'))

    def test_unreachable_unreadable_run_is_not_counted(self):
        proof_body = self.solid_proof(
            'if false {\n'
            '        proofkit3d.Run(runArgs(t))\n'
            '    }')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertNotIn('could not be read', output)

    def test_short_argument_list_is_reported_not_registered(self):
        src = (
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(runArgs(t))\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, ['proofkit3d.Run(runArgs(t))'])


if __name__ == '__main__':
    unittest.main()
