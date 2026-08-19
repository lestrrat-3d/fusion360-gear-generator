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
                proof_body = self.registration('TestOne', 'proofkit.Run', 'stepOne')
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

    def registration(self, test, call, build, cases='profileCases', extra=''):
        """The canonical three-line registration, plus the step definition it names.

        Every argument is a knob a test turns to step one way off the shape, so a fixture that
        only differs in the build argument reads as exactly that difference.
        """
        return (
            'func %s(t *testing.T) {\n'
            '\t%s(t, %s, %s%s)\n'
            '}\n\n'
            'func stepOne() {}\n' % (test, call, cases, build, extra))

    def body(self, statements, test='TestTwo'):
        """A proof whose first registration is canonical and whose second Test is `statements`."""
        return (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func %s(t *testing.T) {\n'
            '%s\n'
            '}\n\n'
            'func stepOne() {}\n' % (test, statements))

    # The canonical shape, in each of the four run forms.

    def test_canonical_2d_registration_is_accepted(self):
        result, output = self.run_checker()

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_canonical_3d_registrations_are_accepted(self):
        for call, extra in (('proofkit3d.Run', ', assertOne'),
                            ('proofkit3d.RunSolid', ', assertOne'),
                            ('proofkit3d.RunWithGate', ', proofkit3d.RequireSolid, assertOne')):
            with self.subTest(call=call):
                proof_body = self.registration('TestOne', call, 'stepOne', extra=extra)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_gofmt_spacing_is_not_required(self):
        """The shape is matched by line, not by byte, so spacing inside the call is free."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit . Run( t ,  profileCases ,  stepOne )\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)

    # The build argument must be the step of the Test's own title.

    def test_misnamed_build_argument_is_blocking(self):
        proof_body = self.body('\tproofkit3d.Run(t, solidCases, buildSolid, assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:6 registers buildSolid as a proof run\'s build argument, '
            'but that argument must be a step<Title> function so a step can claim it', output)

    def test_crossed_registration_is_blocking(self):
        """A Test that builds with some other step is the failure name-matching exists to catch."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n\n'
            'func stepTwo() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:2 registers stepTwo inside TestOne, but a step is '
            'registered by the Test of its own title, so this build belongs in TestTwo', output)

    def test_step_the_matching_test_does_not_build_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n\n'
            'func stepTwo() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof function stepTwo is not claimed by any step', output)

    def test_claimed_step_with_no_registration_is_blocking(self):
        proof_body = 'func stepOne() {}\n'

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    # Everything off the shape is refused rather than read. These are the shapes that cost the
    # brace-matching reader its rounds; here each one is one line of expected output.

    def test_run_in_a_condition_is_refused(self):
        proof_body = self.body(
            '\tif solid {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t}')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:7 runs a proof outside the shape this gate reads',
                      output)

    def test_run_in_a_loop_is_refused(self):
        proof_body = self.body(
            '\tfor _, c := range []struct{ n int }{{1}} {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t}')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_run_in_a_closure_is_refused(self):
        proof_body = self.body(
            '\tt.Run("one", func(t *testing.T) {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t})')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_forwarded_argument_list_is_refused(self):
        proof_body = self.body('\tproofkit3d.Run(runArgs(t))')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads, '
                      'so its build argument cannot be checked', output)

    def test_case_table_built_in_place_is_refused(self):
        """The table is a named variable, so the run stays one line the gate can match."""
        proof_body = self.body(
            '\tproofkit.Run(t, cases(\n'
            '\t\tgear{name: "two"},\n'
            '\t), stepTwo)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_extra_statement_in_the_test_body_is_refused(self):
        proof_body = self.body(
            '\tt.Parallel()\n'
            '\tproofkit.Run(t, profileCases, stepTwo)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_run_outside_any_test_is_refused(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func register(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads',
                      output)

    def test_test_name_go_would_not_run_is_refused(self):
        """`go test` runs Test followed by a non-lower rune, and this gate never has to know that.

        It looks for the title it expects instead of classifying the name it finds, so a name Go
        rejects simply fails to be a registration and its run is reported.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func Testé(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads',
                      output)

    # Comments and literals are blanked first, so a quoted registration is not a real one.

    def test_registration_inside_a_comment_does_not_count(self):
        proof_body = (
            '/*\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n'
            '*/\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    def test_registration_inside_a_raw_string_does_not_count(self):
        proof_body = (
            'var sample = `\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n'
            '`\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    # Two rules Go owns that no reader of source text can infer, refused rather than copied.

    def test_build_constraint_is_blocking(self):
        proof_body = (
            '//go:build ignore\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:1 carries a build constraint, so whether Go ever compiles '
            'these registrations is decided outside the file', output)

    def test_registration_outside_a_test_file_is_blocking(self):
        result, output = self.run_checker(proof_filename='proof.go')

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof.go:2 registers stepOne, but `go test` only runs tests in a '
            '_test.go file, so nothing here ever builds it', output)

    def test_each_off_shape_run_is_reported_at_its_own_line(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func TestTwo(t *testing.T) {\n'
            '\tproofkit3d.Run(runArgs(t))\n'
            '}\n\n'
            'func TestThree(t *testing.T) {\n'
            '\tproofkit3d.RunSolid(runArgs(t))\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof', output)
        self.assertIn('proof/gear/proof_test.go:10 runs a proof', output)


class CommittedStepListProofPathsTest(unittest.TestCase):
    """The proof-path gate must have something to check on the real step lists.

    `proof_paths` reads only the text above `## Provenance`, so a recompile that writes the
    summary sentence below that heading, or that names bare file names instead of
    `proof/<gear>/...`, leaves the gate scanning an empty slice and passing vacuously. The
    synthetic-string tests elsewhere cannot see that, because they supply the sentence the
    committed file is supposed to carry.
    """

    ROOT = Path(__file__).parents[3]

    def step_lists(self):
        paths = sorted((self.ROOT / 'spec').glob('*/steps.md'))
        self.assertTrue(paths, 'no compiled step list found under spec/')
        return paths

    def test_every_committed_step_list_names_its_proof_files(self):
        for path in self.step_lists():
            with self.subTest(steps=str(path.relative_to(self.ROOT))):
                named = COMPILE_CHECKER.proof_paths(path.read_text(encoding='utf-8'))

                self.assertTrue(
                    named,
                    '%s names no proof path above ## Provenance, so the gate checks nothing'
                    % path.relative_to(self.ROOT))
                gear = path.parent.name
                for proof_path in named:
                    self.assertTrue(
                        proof_path.startswith('proof/%s/' % gear),
                        '%s points at %s, not the committed proof/%s/'
                        % (path.relative_to(self.ROOT), proof_path, gear))

    def test_every_named_proof_path_exists_and_is_committed(self):
        prior = os.getcwd()
        try:
            os.chdir(self.ROOT)
            for path in self.step_lists():
                for proof_path in COMPILE_CHECKER.proof_paths(path.read_text(encoding='utf-8')):
                    with self.subTest(proof=proof_path):
                        self.assertTrue(
                            os.path.isfile(proof_path),
                            '%s does not exist' % proof_path)
                        self.assertTrue(
                            COMPILE_CHECKER.proof_path_is_tracked_or_committed(proof_path),
                            '%s is not tracked or committed' % proof_path)
        finally:
            os.chdir(prior)


if __name__ == '__main__':
    unittest.main()
