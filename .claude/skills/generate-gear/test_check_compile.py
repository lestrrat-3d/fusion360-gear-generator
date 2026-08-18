#!/usr/bin/env python3
"""Regression tests for the compile-stage gate."""
import collections
import contextlib
import importlib.util
import io
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock


COMPILE_CHECKER_PATH = Path(__file__).with_name('check_compile.py')
COMPILE_MODULE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_CHECKER_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_MODULE_SPEC)
COMPILE_MODULE_SPEC.loader.exec_module(COMPILE_CHECKER)


UNREADABLE_ARGUMENT = COMPILE_CHECKER.UNREADABLE_ARGUMENT
UNREADABLE_GUARD = COMPILE_CHECKER.UNREADABLE_GUARD
UNREADABLE_OUTSIDE_TEST = COMPILE_CHECKER.UNREADABLE_OUTSIDE_TEST
LOCAL_STEP_PREFIX = 'this Test body binds stepOne'


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
        self.assertEqual(unreadable, [])

    def looping_proof(self, run_call):
        """A proof whose only run is run_call, reached through a loop and a subtest closure."""
        return (
            'func TestOne(t *testing.T) {\n'
            '    for _, name := range names {\n'
            '        t.Run(name, func(t *testing.T) {\n'
            '            %s\n'
            '        })\n'
            '    }\n'
            '}\n\n'
            'func stepOne() {}\n' % run_call)

    def test_run_inside_a_loop_is_registered(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    for _, c := range cases() {\n'
            '        proofkit.Run(t, []proofkit.Case{c}, stepOne)\n'
            '    }\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_run_inside_a_subtest_closure_is_registered(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    t.Run("one", func(t *testing.T) {\n'
            '        proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '    })\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_run_inside_a_loop_and_a_closure_is_registered(self):
        proof_body = self.looping_proof(
            'proofkit.Run(t, cases(gear{name: "one"}), stepOne)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_misnamed_build_inside_a_loop_is_blocking(self):
        proof_body = self.looping_proof(
            'proofkit.Run(t, cases(gear{name: "one"}), buildProfile)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('registers buildProfile as a proof run\'s build argument', output)

    def test_table_registered_build_argument_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    steps := []struct {\n'
            '        name  string\n'
            '        build proofkit.Build\n'
            '    }{\n'
            '        {name: "one", build: stepOne},\n'
            '    }\n'
            '    for _, step := range steps {\n'
            '        step := step\n'
            '        t.Run(step.name, func(t *testing.T) {\n'
            '            proofkit.Run(t, cases(gear{name: "one"}), step.build)\n'
            '        })\n'
            '    }\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go has a proof run the gate cannot read as a registration: '
            'step.build; write the run\'s arguments out one by one, with the build argument a '
            'literal step<Title> identifier so a step can claim the run', output)

    def test_table_registered_step_is_not_called_unregistered(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    for _, step := range []proofkit.Build{stepOne} {\n'
            '        proofkit.Run(t, cases(gear{name: "one"}), step)\n'
            '    }\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertNotIn('no Go Test registers it', output)
        self.assertNotIn('is defined but is not registered', output)

    def test_unreachable_unreadable_build_argument_is_not_counted(self):
        proof_body = self.solid_proof(
            'if false {\n'
            '        proofkit3d.RunSolid(t, solidCases, builds[0].build, assertSolid)\n'
            '    }')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertNotIn('builds[0].build', output)

    def test_table_build_argument_is_unreadable_not_misnamed(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for _, step := range steps {\n'
            '        proofkit.Run(t, spurCases(), step.build)\n'
            '    }\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [('step.build', UNREADABLE_ARGUMENT)])

    def test_loop_variable_build_argument_is_unreadable_not_misnamed(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for _, build := range []proofkit.Build{stepOne} {\n'
            '        proofkit.Run(t, spurCases(), build)\n'
            '    }\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [('build', UNREADABLE_ARGUMENT)])

    def test_comma_declared_local_build_argument_is_unreadable(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    var ignored, stepOne proofkit.Build = buildA, buildB\n'
            '    _ = ignored\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(len(unreadable), 1)
        self.assertEqual(unreadable[0][0], 'stepOne')
        self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_grouped_var_local_build_argument_is_unreadable(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    var (\n'
            '        stepOne proofkit.Build = buildA\n'
            '    )\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(len(unreadable), 1)
        self.assertEqual(unreadable[0][0], 'stepOne')
        self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_grouped_var_initializer_does_not_hide_a_real_registration(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    var (\n'
            '        alias proofkit.Build = stepOne\n'
            '    )\n'
            '    _ = alias\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, {'stepOne'})
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [])

    def test_wrapped_var_initializer_does_not_hide_a_real_registration(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    var (\n'
            '        builds = []proofkit.Build{\n'
            '            stepOne,\n'
            '            stepTwo,\n'
            '        }\n'
            '    )\n'
            '    _ = builds\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '    proofkit.Run(t, spurCases(), stepTwo)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, {'stepOne', 'stepTwo'})
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [])

    # `local` stands in for the declared name, so one template drives both directions: the
    # name a proof must not use, and any other name.
    LOCAL_DECLARATIONS = (
        'var local proofkit.Build = buildA',
        'local := buildA',
    )

    LOCAL_SHAPES = {
        'later in the same block': (
            'func TestOne(t *testing.T) {\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '    %s\n'
            '    _ = local\n'
            '}\n'),
        'earlier sibling block': (
            'func TestOne(t *testing.T) {\n'
            '    if true {\n'
            '        %s\n'
            '        _ = local\n'
            '    }\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n'),
        'sibling blocks': (
            'func TestOne(t *testing.T) {\n'
            '    if true {\n'
            '        %s\n'
            '        _ = local\n'
            '    }\n'
            '    if true {\n'
            '        proofkit.Run(t, spurCases(), stepOne)\n'
            '    }\n'
            '}\n'),
    }

    def local_shapes(self, name):
        for shape, template in self.LOCAL_SHAPES.items():
            for declaration in self.LOCAL_DECLARATIONS:
                yield shape, declaration, (template % declaration).replace('local', name)

    # Go scopes a local from the end of its own spec to the end of the block containing it, so
    # a real scope analysis would leave every run below registered. The chokepoint makes no
    # scope decision, so it reports them unreadable instead. This is the precision option A
    # gives up, written down as a test so a later reader meets it as a decision rather than as
    # a surprise: the cost is a false failure, which is loud, and a proof pays it only by
    # naming a local step<Title>.
    def test_local_outside_the_run_is_given_up_as_unreadable(self):
        for shape, declaration, src in self.local_shapes('stepOne'):
            with self.subTest(shape=shape, declaration=declaration):
                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, set())
                self.assertEqual(misnamed, [])
                self.assertEqual(len(unreadable), 1)
                self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    # The same shapes with the local under any other name still count the literal
    # registration, which is what keeps the chokepoint from being a blanket refusal.
    def test_local_under_another_name_leaves_the_run_registered(self):
        for shape, declaration, src in self.local_shapes('localBuild'):
            with self.subTest(shape=shape, declaration=declaration):
                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, {'stepOne'})
                self.assertEqual(misnamed, [])
                self.assertEqual(unreadable, [])

    def test_local_in_an_enclosing_block_still_makes_the_run_unreadable(self):
        for declaration in self.LOCAL_DECLARATIONS:
            with self.subTest(declaration=declaration):
                src = ((
                    'func TestOne(t *testing.T) {\n'
                    '    if true {\n'
                    '        %s\n'
                    '        proofkit.Run(t, spurCases(), stepOne)\n'
                    '    }\n'
                    '}\n') % declaration).replace('local', 'stepOne')

                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, set())
                self.assertEqual(misnamed, [])
                self.assertEqual(len(unreadable), 1)
                self.assertEqual(unreadable[0][0], 'stepOne')
                self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_loop_header_binding_covers_the_loop_body(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for _, stepOne := range []proofkit.Build{buildA} {\n'
            '        proofkit.Run(t, spurCases(), stepOne)\n'
            '    }\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(len(unreadable), 1)
        self.assertEqual(unreadable[0][0], 'stepOne')
        self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_loop_header_binding_reaches_past_its_loop(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for _, stepOne := range []proofkit.Build{buildA} {\n'
            '        _ = stepOne\n'
            '    }\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(len(unreadable), 1)
        self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_loop_header_binding_under_another_name_leaves_the_run_registered(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for _, localBuild := range []proofkit.Build{buildA} {\n'
            '        _ = localBuild\n'
            '    }\n'
            '    proofkit.Run(t, spurCases(), stepOne)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, {'stepOne'})
        self.assertEqual(misnamed, [])
        self.assertEqual(unreadable, [])

    # A loop header can write a brace group of its own before the block it opens, and gofmt
    # leaves a space in front of that group's brace exactly as it does in front of the block's.
    # Every body below is gofmt's own output, so the shapes are what a drafter would really
    # write, and each one has to end with the loop variable holding the build: a run reported
    # as registering a step it only names through the loop variable is a false pass, and the
    # geometry that step claims would go unproven with the gate green.
    HEADER_BLOCK_SHAPES = {
        'multi-line anonymous struct': (
            '\tfor _, stepOne := range []struct {\n'
            '\t\tbuild proofkit.Build\n'
            '\t}{{buildA}} {\n'),
        'function literal in the header': (
            '\tfor _, stepOne := range func() []proofkit.Build {\n'
            '\t\treturn []proofkit.Build{buildA}\n'
            '\t}() {\n'),
        'build slice literal': (
            '\tfor _, stepOne := range []proofkit.Build{buildA} {\n'),
    }

    def test_loop_header_shapes_leave_the_run_unreadable(self):
        for shape, header in self.HEADER_BLOCK_SHAPES.items():
            with self.subTest(shape=shape):
                src = (
                    'func TestOne(t *testing.T) {\n'
                    + header
                    + '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
                    '\t}\n'
                    '}\n')

                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, set())
                self.assertEqual(misnamed, [])
                self.assertEqual(len(unreadable), 1)
                self.assertEqual(unreadable[0][0], 'stepOne')
                self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    def test_loop_header_shapes_reach_past_their_loop(self):
        for shape, header in self.HEADER_BLOCK_SHAPES.items():
            with self.subTest(shape=shape):
                src = (
                    'func TestOne(t *testing.T) {\n'
                    + header
                    + '\t\t_ = stepOne\n'
                    '\t}\n'
                    '\tproofkit.Run(t, spurCases(), stepOne)\n'
                    '}\n')

                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, set())
                self.assertEqual(misnamed, [])
                self.assertEqual(len(unreadable), 1)
                self.assertTrue(unreadable[0][1].startswith(LOCAL_STEP_PREFIX))

    # The same three headers with an ordinary loop variable still count the registration, so
    # the header shapes that defeated three rounds of the old scope decision now cost nothing.
    def test_loop_header_shapes_under_another_name_stay_registered(self):
        for shape, header in self.HEADER_BLOCK_SHAPES.items():
            with self.subTest(shape=shape):
                src = (
                    'func TestOne(t *testing.T) {\n'
                    + header.replace('stepOne', 'localBuild')
                    + '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
                    '\t}\n'
                    '}\n')

                registered, misnamed, unreadable = (
                    COMPILE_CHECKER.registered_step_functions(src))

                self.assertEqual(registered, {'stepOne'})
                self.assertEqual(misnamed, [])
                self.assertEqual(unreadable, [])

    def test_grouped_var_local_build_argument_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    var (\n'
            '        stepOne proofkit.Build = buildProfile\n'
            '    )\n'
            '    proofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go has a proof run the gate cannot read as a registration: '
            'stepOne; ' + LOCAL_STEP_PREFIX, output)

    def test_loop_whose_body_returns_does_not_hide_a_later_run(self):
        src = (
            'func TestOne(t *testing.T) {\n'
            '    for range cases() {\n'
            '        return\n'
            '    }\n'
            '    proofkit.Run(t, cases(), buildProfile)\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, ['buildProfile'])
        self.assertEqual(unreadable, [])

    # A run whose argument list the parse cannot read to the build slot is reported, because
    # such a run can compile: Go lets one multi-value call supply a whole argument list, so
    # proofkit3d.Run(runArgs(t)) vets clean and still parses to a single argument here.
    # Skipping it silently would let its build argument escape the step<Title> check.

    def test_multi_value_forwarded_run_is_blocking(self):
        proof_body = self.solid_proof('proofkit3d.Run(runArgs(t))')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go has a proof run the gate cannot read as a registration: '
            'proofkit3d.Run(runArgs(t)); write the run\'s arguments out one by one, with the '
            'build argument a literal step<Title> identifier so a step can claim the run',
            output)

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
        self.assertIn(
            'has a proof run the gate cannot read as a registration: '
            'proofkit.Run(runArgs(t))', output)

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
        self.assertTrue(unreadable[0][0].startswith('proofkit3d.Run(t, solidCases, stepSolid'))
        self.assertEqual(unreadable[0][1], UNREADABLE_ARGUMENT)

    def test_unreachable_unreadable_run_is_not_counted(self):
        proof_body = self.solid_proof(
            'if false {\n'
            '        proofkit3d.Run(runArgs(t))\n'
            '    }')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertNotIn('cannot read as a registration', output)

    # A run the walk cannot decide about used to be dropped before its build argument was read,
    # which left the step it registers reported as registered nowhere. That is a phantom: the
    # proof does register the step, and the drafter sent to fix it finds nothing wrong. Both
    # tests below hold the gate to naming the run instead.

    def test_run_under_an_unreadable_condition_is_loud_not_dropped(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tif enabled() {\n'
            '\t\tproofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '\t}\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('has a proof run the gate cannot read as a registration', output)
        self.assertIn(UNREADABLE_GUARD, output)
        self.assertNotIn('no Go Test registers it', output)
        self.assertNotIn('is defined but is not registered', output)

    def test_run_under_a_wrapped_unreadable_condition_is_loud_not_dropped(self):
        """The same guard, written over three lines instead of one.

        Reading only the line before the brace left this header as `)`, which named no
        condition, so the guard passed as an enclosure and the step counted as registered.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tif enabled(\n'
            '\t\t"one",\n'
            '\t) {\n'
            '\t\tproofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '\t}\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('has a proof run the gate cannot read as a registration', output)
        self.assertIn(UNREADABLE_GUARD, output)
        self.assertNotIn('no Go Test registers it', output)
        self.assertNotIn('is defined but is not registered', output)

    def test_run_in_the_else_arm_of_an_unreadable_if_is_loud_not_dropped(self):
        """An else arm is as unreadable as the if it closes.

        `} else` names no condition of its own, so it used to read as an enclosure and the run
        inside it counted, though it executes only when a condition the gate cannot read fails.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tif enabled() {\n'
            '\t} else {\n'
            '\t\tproofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '\t}\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('has a proof run the gate cannot read as a registration', output)
        self.assertIn(UNREADABLE_GUARD, output)
        self.assertNotIn('no Go Test registers it', output)
        self.assertNotIn('is defined but is not registered', output)

    def test_run_in_a_helper_is_loud_not_dropped(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tregister(t)\n'
            '}\n\n'
            'func register(t *testing.T) {\n'
            '\tproofkit.Run(t, cases(gear{name: "one"}), stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(UNREADABLE_OUTSIDE_TEST, output)
        self.assertNotIn('no Go Test registers it', output)
        self.assertNotIn('is defined but is not registered', output)

    def test_short_argument_list_is_reported_not_registered(self):
        src = (
            'func TestSolid(t *testing.T) {\n'
            '    proofkit3d.Run(runArgs(t))\n'
            '}\n')

        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)

        self.assertEqual(registered, set())
        self.assertEqual(misnamed, [])
        self.assertEqual(
            unreadable, [('proofkit3d.Run(runArgs(t))', UNREADABLE_ARGUMENT)])


# ---------------------------------------------------------------------------------------------
# The root-cause map, transcribed as a table and then covered cell by cell.
#
# The map crossed the Go binding construct (its rows) with what the construct writes before its
# block and where the run sits (its columns): C1 no brace group in the header, C2 a brace group
# mid-header, C3 a brace group ending a clause so a `;` follows its `}`, C4 the run in a sibling
# clause or after the construct. H is a shape the gate already read correctly, N a shape the
# grammar does not allow, and a G marks one of the seven gaps the map found.
#
# Every cell that is not N is covered below in both directions: the construct binding a
# `step<Title>` name, where the run must now be unreadable, and the same construct binding an
# ordinary name, where the literal registration must still be counted. The second direction is
# what keeps the chokepoint from being a blanket refusal, so it is not optional.
MAP_TABLE = {
    ':= single-line left-hand side': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    ':= multi-line left-hand side': {'C1': 'G5', 'C2': 'G5', 'C3': 'N', 'C4': 'H'},
    'var x = ...': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'var ( ... ) group': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'for ... := range header': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'three-clause for, init binds': {'C1': 'H', 'C2': 'H', 'C3': 'G1', 'C4': 'H'},
    'if with init, run in the then-branch': {'C1': 'G6', 'C2': 'G6', 'C3': 'G6', 'C4': 'N'},
    'if with init, run in the else-branch': {'C1': 'H', 'C2': 'H', 'C3': 'G1', 'C4': 'H'},
    'expression switch with init': {'C1': 'H', 'C2': 'H', 'C3': 'G1', 'C4': 'H'},
    'type-switch guard': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'switch case clause body binding': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'G3'},
    'select clause case v := <-ch:': {'C1': 'G2', 'C2': 'G2', 'C3': 'N', 'C4': 'G3'},
    'plain block': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'func-literal body': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'func-literal parameters and named results': {'C1': 'G4', 'C2': 'G4', 'C3': 'N', 'C4': 'H'},
    'labelled statement, gofmt form': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'labelled statement on one line': {'C1': 'N', 'C2': 'N', 'C3': 'N', 'C4': 'N'},
    'const / type declaration': {'C1': 'N', 'C2': 'N', 'C3': 'N', 'C4': 'N'},
    'range-over-func / range-over-int': {'C1': 'H', 'C2': 'H', 'C3': 'N', 'C4': 'H'},
    'run in a helper, not in a Test body': {'C1': 'G7', 'C2': 'G7', 'C3': 'G7', 'C4': 'G7'},
    'run under a condition the walk cannot read': {
        'C1': 'G6', 'C2': 'G6', 'C3': 'G6', 'C4': 'G6'},
}

REGISTERED = 'registered'
LOCAL = 'unreadable: the body binds a step name'
GUARD = 'unreadable: the guard cannot be read'
OUTSIDE = 'unreadable: the run is outside every Test'

Cell = collections.namedtuple('Cell', 'row columns source bound free')

# LOCAL_NAME is the identifier the construct binds. Substituting `stepOne` for it gives the
# false-pass direction, and any other identifier gives the direction that must still count.
LOCAL_NAME = 'localName'

MAP_CELLS = (
    Cell(':= single-line left-hand side', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tlocalName := buildA\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell(':= single-line left-hand side', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tlocalName := []proofkit.Build{buildA}[0]\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell(':= single-line left-hand side', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\tlocalName := buildA\n'
        '\t_ = localName\n'
        '}\n'), LOCAL, REGISTERED),
    Cell(':= multi-line left-hand side', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tlocalName,\n'
        '\t\tother := buildA, buildB\n'
        '\t_, _ = localName, other\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell(':= multi-line left-hand side', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tlocalName,\n'
        '\t\tother := []proofkit.Build{buildA}[0], buildB\n'
        '\t_, _ = localName, other\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell(':= multi-line left-hand side', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\tlocalName,\n'
        '\t\tother := buildA, buildB\n'
        '\t_, _ = localName, other\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var x = ...', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tvar localName = buildA\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var x = ...', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tvar localName = []proofkit.Build{buildA}[0]\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var x = ...', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\tvar localName = buildA\n'
        '\t_ = localName\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var ( ... ) group', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tvar (\n'
        '\t\tlocalName proofkit.Build = buildA\n'
        '\t)\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var ( ... ) group', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tvar (\n'
        '\t\tlocalName = []proofkit.Build{buildA}[0]\n'
        '\t)\n'
        '\t_ = localName\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('var ( ... ) group', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\tvar (\n'
        '\t\tlocalName proofkit.Build = buildA\n'
        '\t)\n'
        '\t_ = localName\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('for ... := range header', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor _, localName := range []proofkit.Build{buildA} {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('for ... := range header', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor _, localName := range []struct {\n'
        '\t\tbuild proofkit.Build\n'
        '\t}{{buildA}} {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('for ... := range header', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor _, localName := range []proofkit.Build{buildA} {\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('three-clause for, init binds', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := buildA; localName != nil; localName = nil {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('three-clause for, init binds', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := []proofkit.Build{buildA}[0]; localName != nil; localName = nil {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('three-clause for, init binds', ('C3',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName, list := buildA, []proofkit.Build{buildA}; len(list) > 0; list = nil {\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('three-clause for, init binds', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := buildA; localName != nil; localName = nil {\n'
        '\t\tbreak\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('if with init, run in the then-branch', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName := buildA; localName != nil {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), GUARD, GUARD),
    Cell('if with init, run in the then-branch', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName := []proofkit.Build{buildA}[0]; localName != nil {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), GUARD, GUARD),
    Cell('if with init, run in the then-branch', ('C3',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName, list := buildA, []proofkit.Build{buildA}; len(list) > 0 {\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), GUARD, GUARD),
    # These three guard on a literal, so that the else arm they put the run in stays readable.
    # An else arm inherits the readability of the if it closes (see the guard axis below), so a
    # condition the walk cannot read would answer for these cells first and they would stop
    # testing the binding they exist to test.
    Cell('if with init, run in the else-branch', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName := buildA; false {\n'
        '\t\t_ = localName\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('if with init, run in the else-branch', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName := []proofkit.Build{buildA}[0]; false {\n'
        '\t\t_ = localName\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('if with init, run in the else-branch', ('C3',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName, list := buildA, []proofkit.Build{buildA}; false {\n'
        '\t\t_, _ = localName, list\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('if with init, run in the else-branch', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tif localName := buildA; localName == nil {\n'
        '\t\t_ = localName\n'
        '\t} else {\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('expression switch with init', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := buildA; localName {\n'
        '\tcase buildA:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('expression switch with init', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := []proofkit.Build{buildA}[0]; localName {\n'
        '\tcase buildA:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('expression switch with init', ('C3',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName, list := buildA, []proofkit.Build{buildA}; len(list) {\n'
        '\tcase 1:\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('expression switch with init', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := buildA; localName {\n'
        '\tcase buildA:\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('type-switch guard', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := any(buildA).(type) {\n'
        '\tcase proofkit.Build:\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('type-switch guard', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := any([]proofkit.Build{buildA}[0]).(type) {\n'
        '\tcase proofkit.Build:\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('type-switch guard', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch localName := any(buildA).(type) {\n'
        '\tcase proofkit.Build:\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('switch case clause body binding', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch {\n'
        '\tcase true:\n'
        '\t\tlocalName := buildA\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('switch case clause body binding', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch {\n'
        '\tcase true:\n'
        '\t\tlocalName := []proofkit.Build{buildA}[0]\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('switch case clause body binding', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tswitch {\n'
        '\tcase true:\n'
        '\t\tlocalName := buildA\n'
        '\t\t_ = localName\n'
        '\tdefault:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('select clause case v := <-ch:', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tch := make(chan proofkit.Build)\n'
        '\tselect {\n'
        '\tcase localName := <-ch:\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('select clause case v := <-ch:', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tch := make(chan proofkit.Build, len([]proofkit.Build{buildA}))\n'
        '\tselect {\n'
        '\tcase localName := <-ch:\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('select clause case v := <-ch:', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tch := make(chan proofkit.Build)\n'
        '\tselect {\n'
        '\tcase localName := <-ch:\n'
        '\t\t_ = localName\n'
        '\tdefault:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('plain block', ('C1', 'C2'), (
        'func TestOne(t *testing.T) {\n'
        '\t{\n'
        '\t\tlocalName := []proofkit.Build{buildA}[0]\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('plain block', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\t{\n'
        '\t\tlocalName := buildA\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('func-literal body', ('C1', 'C2'), (
        'func TestOne(t *testing.T) {\n'
        '\tt.Run("one", func(t *testing.T) {\n'
        '\t\tlocalName := []proofkit.Build{buildA}[0]\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t})\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('func-literal body', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tt.Run("one", func(t *testing.T) {\n'
        '\t\tlocalName := buildA\n'
        '\t\t_ = localName\n'
        '\t})\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('func-literal parameters and named results', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tcall := func(localName proofkit.Build) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '\tcall(buildA)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('func-literal parameters and named results', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tcall := func() (localName proofkit.Build) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\treturn []proofkit.Build{buildA}[0]\n'
        '\t}\n'
        '\t_ = call\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('func-literal parameters and named results', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tcall := func(localName proofkit.Build) {\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tcall(buildA)\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('labelled statement, gofmt form', ('C1', 'C2'), (
        'func TestOne(t *testing.T) {\n'
        'Loop:\n'
        '\tfor _, localName := range []proofkit.Build{buildA} {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\tbreak Loop\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('labelled statement, gofmt form', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        'Loop:\n'
        '\tfor _, localName := range []proofkit.Build{buildA} {\n'
        '\t\t_ = localName\n'
        '\t\tbreak Loop\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('range-over-func / range-over-int', ('C1',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := range 3 {\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('range-over-func / range-over-int', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := range slices.Values([]proofkit.Build{buildA}) {\n'
        '\t\t_ = localName\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('range-over-func / range-over-int', ('C4',), (
        'func TestOne(t *testing.T) {\n'
        '\tfor localName := range 3 {\n'
        '\t\t_ = localName\n'
        '\t}\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), LOCAL, REGISTERED),
    Cell('run under a condition the walk cannot read', ('C1', 'C2', 'C3', 'C4'), (
        'func TestOne(t *testing.T) {\n'
        '\tif enabled() {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), GUARD, GUARD),
    Cell('run in a helper, not in a Test body', ('C1', 'C2', 'C3', 'C4'), (
        'func TestOne(t *testing.T) {\n'
        '\tregister(t)\n'
        '}\n'
        '\n'
        'func register(t *testing.T) {\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'
        '}\n'), OUTSIDE, OUTSIDE),
)


class MapCellTest(unittest.TestCase):
    """Every cell of the root-cause map, in both directions.

    A cell fails in one of two ways, and the two are not symmetric. A false pass — a local
    counted as a registered step — is silent, and it lets the geometry that step claims go
    unproven behind a green gate. A false failure is loud and costs a drafter a rename. The
    chokepoint converts the first into the second everywhere, so each cell is asserted twice:
    once with the construct binding the step name, where the run must now be unreadable, and
    once with it binding an ordinary name, where the registration must still be counted.
    """

    def source(self, cell, name):
        return cell.source.replace(LOCAL_NAME, name)

    def outcome(self, src):
        """Which of the four results the gate gives this source."""
        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(src)
        if registered == {'stepOne'} and not misnamed and not unreadable:
            return REGISTERED
        self.assertEqual(registered, set(), src)
        self.assertEqual(misnamed, [], src)
        self.assertEqual(len(unreadable), 1, src)
        reason = unreadable[0][1]
        if reason.startswith(LOCAL_STEP_PREFIX):
            return LOCAL
        if reason == UNREADABLE_GUARD:
            return GUARD
        if reason == UNREADABLE_OUTSIDE_TEST:
            return OUTSIDE
        return reason

    def test_every_cell_binding_the_step_name(self):
        for cell in MAP_CELLS:
            with self.subTest(row=cell.row, columns=cell.columns):
                self.assertEqual(
                    self.outcome(self.source(cell, 'stepOne')), cell.bound)

    def test_every_cell_binding_an_ordinary_name(self):
        for cell in MAP_CELLS:
            with self.subTest(row=cell.row, columns=cell.columns):
                self.assertEqual(
                    self.outcome(self.source(cell, 'localBuild')), cell.free)

    def test_every_cell_of_the_map_is_covered(self):
        """The table above and the cells below are reconciled, not eyeballed.

        The map is the enumeration of the space, so a cell it lists and no case exercises is a
        hole, and a case naming a cell the map calls impossible is a case testing nothing.
        """
        listed = {(row, column)
                  for row, columns in MAP_TABLE.items()
                  for column, mark in columns.items() if mark != 'N'}
        covered = {(cell.row, column) for cell in MAP_CELLS for column in cell.columns}

        self.assertEqual(covered, listed)

    def test_every_cell_source_is_what_gofmt_writes(self):
        """gofmt itself, not a guess at it.

        A probe gofmt would reformat is a probe of a shape no drafter would commit, and three
        review rounds turned on exactly which brace a real formatter puts where.
        """
        if shutil.which('gofmt') is None:
            self.skipTest('gofmt is not on PATH')
        for cell in MAP_CELLS:
            for name in ('stepOne', 'localBuild'):
                with self.subTest(row=cell.row, columns=cell.columns, name=name):
                    src = 'package proof_test\n\n' + self.source(cell, name)

                    formatted = subprocess.run(
                        ['gofmt'], input=src, capture_output=True, text=True, check=True)

                    self.assertEqual(formatted.stdout, src)


# ---------------------------------------------------------------------------------------------
# The guard axis: given a brace the walk crosses on its way to a run, is it a guard or an
# enclosure, and if a guard, can its condition be read?
#
# This is not a column of the map above. That map decided which NAMES a body binds; this decides
# whether the run executes at all. Go's grammar closes the axis — only the if / else if / else
# family skips its own block on a condition — so the shapes can be listed once, and HEADER_SHAPES
# is that list. Every shape gets one case, and the list and the cases are reconciled below rather
# than eyeballed.
#
# Both directions are asserted, because they fail differently. Calling a guard an enclosure is a
# false pass: the step counts as registered while its run may never execute, and the geometry it
# claims goes unproven behind a green gate. Calling an enclosure a guard is a false failure: loud,
# and it costs a drafter a rewrite of a proof that was already correct. The enclosure shapes used
# to come out right by accident, an unread header falling through to reachable; they are asserted
# here so that the keyword now deciding them is what keeps them right.
HEADER_SHAPES = (
    'if, single-line',
    'if, wrapped condition',
    'if with an init statement, single-line',
    'if with an init statement, wrapped',
    'else, closing an unreadable if',
    'else, closing a known-false if',
    'else, closing a known-true if',
    'else if, single-line',
    'else if, wrapped condition',
    'known literal, true',
    'known literal, false',
    'known literals compounded, single-line',
    'known literals compounded, wrapped',
    'for, three-clause',
    'for, range',
    'for, bare',
    'for, wrapped range expression',
    'switch, expression, and its case body',
    'switch, type, and its case body',
    'switch with an init statement, and its case body',
    'select, and its case body',
    'plain block',
    'func literal, immediately invoked',
    'func literal, a t.Run closure',
    'func literal, started with go',
    'func literal, deferred',
    'labelled statement',
    'composite literal at line start',
    'struct literal at line start',
    'map literal at line start',
    'if alone on its line, which gofmt rewrites',
)

REACHABLE = REGISTERED
DEAD = 'dead: the walk knows the run never executes'

# rewritten marks a shape gofmt does not leave alone. Such a shape cannot reach a committed
# proof, so its case pins the rewrite rather than the reading.
Shape = collections.namedtuple('Shape', 'name verdict source rewritten', defaults=(False,))

HEADER_SHAPE_CASES = (
    # The if family. A condition that is not a known literal makes the run unreadable, and the
    # shape it is written in must not change that: these are the same guard four ways.
    Shape('if, single-line', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif enabled(t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('if, wrapped condition', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif enabled(\n'
        '\t\tt,\n'
        '\t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('if with an init statement, single-line', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif on := enabled(t); on {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('if with an init statement, wrapped', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif on := enabled(\n'
        '\t\tt,\n'
        '\t); on {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    # The else family. A bare else names no condition of its own, so it takes the readability of
    # the chain it closes: unreadable behind an unreadable if, live behind a dead one, dead
    # behind a live one.
    Shape('else, closing an unreadable if', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif enabled(t) {\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('else, closing a known-false if', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tif false {\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('else, closing a known-true if', DEAD, (
        'func TestOne(t *testing.T) {\n'
        '\tif true {\n'
        '\t} else {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('else if, single-line', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif false {\n'
        '\t} else if enabled(t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('else if, wrapped condition', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif false {\n'
        '\t} else if enabled(\n'
        '\t\tt,\n'
        '\t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    # The literal conditions the walk does read. A compound of them is decidable and is still
    # refused, because deciding it is evaluating Go; the wrapped form must give the same answer
    # as the single-line one, which is the whole point of reading the header to its start.
    Shape('known literal, true', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tif true {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('known literal, false', DEAD, (
        'func TestOne(t *testing.T) {\n'
        '\tif false {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('known literals compounded, single-line', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif false || false {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('known literals compounded, wrapped', GUARD, (
        'func TestOne(t *testing.T) {\n'
        '\tif false ||\n'
        '\t\tfalse {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    # Every enclosure below. None of them can skip its block on a condition, so a run inside one
    # runs when the Test does, and the keyword at the head of the header is what says so.
    Shape('for, three-clause', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tfor i := 0; i < 2; i++ {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('for, range', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tfor range spurCases() {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('for, bare', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tfor {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\tbreak\n'
        '\t}\n'
        '}\n')),
    Shape('for, wrapped range expression', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tfor range namedCases(\n'
        '\t\tt,\n'
        '\t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('switch, expression, and its case body', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tswitch len(spurCases()) {\n'
        '\tcase 0:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('switch, type, and its case body', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tswitch any(t).(type) {\n'
        '\tcase *testing.T:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('switch with an init statement, and its case body', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tswitch n := len(spurCases()); n {\n'
        '\tcase 0:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('select, and its case body', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tselect {\n'
        '\tcase <-done:\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('plain block', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\t{\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n')),
    Shape('func literal, immediately invoked', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tfunc() {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}()\n'
        '}\n')),
    Shape('func literal, a t.Run closure', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tt.Run("one", func(t *testing.T) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t})\n'
        '}\n')),
    Shape('func literal, started with go', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tgo func() {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}()\n'
        '}\n')),
    Shape('func literal, deferred', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tdefer func() {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}()\n'
        '}\n')),
    Shape('labelled statement', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        'Loop:\n'
        '\tfor {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\tbreak Loop\n'
        '\t}\n'
        '}\n')),
    Shape('composite literal at line start', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\truns := []func(){\n'
        '\t\tfunc() {\n'
        '\t\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\t},\n'
        '\t}\n'
        '\t_ = runs\n'
        '}\n')),
    Shape('struct literal at line start', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tspec := runSpec{\n'
        '\t\trun: func() {\n'
        '\t\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\t},\n'
        '\t}\n'
        '\t_ = spec\n'
        '}\n')),
    Shape('map literal at line start', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tbyName := map[string]func(){\n'
        '\t\t"one": func() {\n'
        '\t\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\t},\n'
        '\t}\n'
        '\t_ = byName\n'
        '}\n')),
    # The one shape read wrongly, kept in the list rather than hidden. An `if` alone on its line
    # is legal Go, the scan stops at the line break, and the guard reads as an enclosure. gofmt
    # rewrites it to one line, where it reads as the guard it is, so no committed proof holds it.
    Shape('if alone on its line, which gofmt rewrites', REACHABLE, (
        'func TestOne(t *testing.T) {\n'
        '\tif\n'
        '\tenabled(t) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t}\n'
        '}\n'), True),
)


class HeaderShapeTest(unittest.TestCase):
    """Every header shape the Go grammar allows in front of a brace, one case each."""

    def outcome(self, src):
        """Which of the three verdicts the gate gives this source."""
        scrubbed = COMPILE_CHECKER.strip_go_comments_and_literals(src)
        registered, misnamed, unreadable = COMPILE_CHECKER.registered_step_functions(scrubbed)
        self.assertEqual(misnamed, [], src)
        if registered == {'stepOne'} and not unreadable:
            return REACHABLE
        self.assertEqual(registered, set(), src)
        if not unreadable:
            return DEAD
        self.assertEqual(len(unreadable), 1, src)
        reason = unreadable[0][1]
        return GUARD if reason == UNREADABLE_GUARD else reason

    def test_every_shape_reads_as_the_table_says(self):
        for shape in HEADER_SHAPE_CASES:
            with self.subTest(shape=shape.name):
                self.assertEqual(self.outcome(shape.source), shape.verdict)

    def test_every_shape_of_the_grammar_is_covered(self):
        """The list above and the cases below it are reconciled, not eyeballed.

        Three review rounds each repaired one header shape and left another open, because the
        shapes were found one at a time rather than listed. The list is the enumeration now, so
        a shape it names and no case exercises is a hole.
        """
        self.assertEqual(
            [shape.name for shape in HEADER_SHAPE_CASES], list(HEADER_SHAPES))

    def test_every_shape_source_is_what_gofmt_writes(self):
        """gofmt itself, not a guess at it.

        A probe gofmt would reformat is a probe of a shape no drafter can commit. The one shape
        that is reformatted says so in the table, and is asserted to be reformatted, so the gap
        it documents cannot quietly become a shape the gate must read.
        """
        if shutil.which('gofmt') is None:
            self.skipTest('gofmt is not on PATH')
        for shape in HEADER_SHAPE_CASES:
            with self.subTest(shape=shape.name):
                src = 'package proof_test\n\n' + shape.source

                formatted = subprocess.run(
                    ['gofmt'], input=src, capture_output=True, text=True, check=True)

                if shape.rewritten:
                    self.assertNotEqual(formatted.stdout, src)
                else:
                    self.assertEqual(formatted.stdout, src)


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
