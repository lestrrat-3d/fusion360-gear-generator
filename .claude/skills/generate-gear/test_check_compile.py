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
UNREADABLE_CLOSURE = COMPILE_CHECKER.UNREADABLE_CLOSURE
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
    # The literal is called rather than discarded, because a run inside a literal stored under
    # a name that is never called is unreadable on the reached-literal axis, and this cell is
    # about which NAME a named result binds. Both axes are asserted, each in its own table.
    Cell('func-literal parameters and named results', ('C2',), (
        'func TestOne(t *testing.T) {\n'
        '\tcall := func() (localName proofkit.Build) {\n'
        '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
        '\t\treturn []proofkit.Build{buildA}[0]\n'
        '\t}\n'
        '\t_ = call()\n'
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


# ---------------------------------------------------------------------------------------------
# The signature matrix: which identifiers a func SIGNATURE binds.
#
# A parameter list holds names and types together, and only the names are bindings. The types
# must stay out, because every name collected here reaches the chokepoint through
# `go_bound_step_names`, which keeps anything matching `step<Title>`. A proof is free to declare
# a package-level `type stepType`, `stepCase` or `stepResult` and write it in a parameter's
# type; reading that type as a binding makes every run in the body unreadable, which suppresses
# the registration verdict for the whole proof directory and tells the drafter to rename a local
# that was never written.
#
# The eleven shapes below are the audited matrix, and `required` is its third column verbatim.
# `incidental` names what the surrounding statements bind — the `call` of a `call := func...` —
# so the two together are the whole binding set of the body and the assertion is exact rather
# than a containment check.
#
# Both directions are asserted, because they fail differently. Reading a type as a binding is a
# false failure: loud, and it costs a drafter a rename with nothing to rename. Losing a real
# parameter name is a false pass: silent, and it lets a run that builds with a local count as
# the registration of the package-level step of the same name, leaving that step's geometry
# unproven behind a green gate. So each shape is written twice, once with `stepType` standing in
# the type position, where the run must still register, and once with `stepOne` standing in the
# shape's binding position, where the run must become unreadable.
#
# Three shapes bind nothing at all in either direction — a func-typed struct field, an interface
# method and `func()` — so their bound direction wraps the same shape in a literal that does
# bind, which is the nearest genuine binding the shape admits.
Signature = collections.namedtuple('Signature', 'shape required incidental body bound')

RUN = '\t\tproofkit.Run(t, spurCases(), stepOne)\n'
BOUND_RUN = '\t\tproofkit.Run(t, spurCases(), stepTwo)\n'

SIGNATURE_SHAPES = (
    Signature(
        'func(a, b stepType)', frozenset({'a', 'b'}), frozenset({'call'}),
        '\tcall := func(a, b stepType) {\n' + RUN + '\t}\n\tcall(0, 0)\n',
        '\tcall := func(a, stepOne stepType) {\n' + BOUND_RUN + '\t}\n\tcall(0, 0)\n'),
    Signature(
        'func(a ...stepType)', frozenset({'a'}), frozenset({'call'}),
        '\tcall := func(a ...stepType) {\n' + RUN + '\t}\n\tcall()\n',
        '\tcall := func(stepOne ...stepType) {\n' + BOUND_RUN + '\t}\n\tcall()\n'),
    Signature(
        'func() (res stepType, err error)', frozenset({'res', 'err'}), frozenset({'call'}),
        '\tcall := func() (res stepType, err error) {\n' + RUN
        + '\t\treturn 0, nil\n\t}\n\t_, _ = call()\n',
        '\tcall := func() (stepOne stepType, err error) {\n' + BOUND_RUN
        + '\t\treturn 0, nil\n\t}\n\t_, _ = call()\n'),
    # The unnamed result list and the named one hold groups of a single identifier alike, so
    # this shape and the one above it are what the list-wide rule separates. Writing the names
    # in is what turns this list into that one, and it is the bound direction of both.
    Signature(
        'func() (stepType, error)', frozenset(), frozenset({'call'}),
        '\tcall := func() (stepType, error) {\n' + RUN
        + '\t\treturn 0, nil\n\t}\n\t_, _ = call()\n',
        '\tcall := func() (stepOne stepType, err error) {\n' + BOUND_RUN
        + '\t\treturn 0, nil\n\t}\n\t_, _ = call()\n'),
    Signature(
        'func(stepType)', frozenset(), frozenset({'call'}),
        '\tcall := func(stepType) {\n' + RUN + '\t}\n\tcall(0)\n',
        '\tcall := func(stepOne stepType) {\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        'func(cb func(arg stepType))', frozenset({'cb'}), frozenset({'call'}),
        '\tcall := func(cb func(arg stepType)) {\n' + RUN + '\t}\n\tcall(nil)\n',
        '\tcall := func(stepOne func(arg stepType)) {\n' + BOUND_RUN + '\t}\n\tcall(nil)\n'),
    Signature(
        'struct field run func(arg stepType)', frozenset(), frozenset(),
        '\ttype config struct{ run func(arg stepType) }\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\ttype config struct{ run func(arg stepType) }\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        'interface method do(arg stepType)', frozenset(), frozenset(),
        '\ttype doer interface{ do(arg stepType) }\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\ttype doer interface{ do(arg stepType) }\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        'func(p pair[stepType])', frozenset({'p'}), frozenset({'call'}),
        '\tcall := func(p pair[stepType]) {\n' + RUN + '\t}\n\tcall(pair[stepType]{})\n',
        '\tcall := func(stepOne pair[stepType]) {\n' + BOUND_RUN
        + '\t}\n\tcall(pair[stepType]{})\n'),
    Signature(
        'func()', frozenset(), frozenset({'call'}),
        '\tcall := func() {\n' + RUN + '\t}\n\tcall()\n',
        '\tcall := func(stepOne stepType) {\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        't.Run("x", func(inner *testing.T) {})', frozenset({'inner'}), frozenset(),
        '\tt.Run("x", func(inner *testing.T) {\n'
        '\t\tproofkit.Run(inner, spurCases(), stepOne)\n\t})\n',
        '\tt.Run("x", func(stepOne *testing.T) {\n'
        '\t\tproofkit.Run(stepOne, spurCases(), stepTwo)\n\t})\n'),
)

# The scan restriction, anchored the same way. A func TYPE writes a parameter list that no body
# can read, so nothing in it is a binding, and the first two rows are the audited fixtures for
# exactly that. The third is the same in a `var` declaration. The fourth is the other side of
# the restriction: a literal whose single result is written without parentheses is still a
# literal, and dropping it would lose a real parameter name, which is the silent failure.
#
# The fifth and sixth rows are the audited fixtures for the two positions where a following
# brace hides a func type: the RESULT type of an enclosing literal, where the brace standing
# after the type opens that literal's body, and the element type of a composite literal, where
# it opens the literal's value. A local test at the parameter list sees a brace in both and
# cannot say whose it is, so the scan over the whole body decides it. TYPE_TEXT_SHAPES below
# takes the element-type position through every bracketed context it is written in.
#
# The last row is the other side of the pointer prefix that scan walks over, and it is here
# rather than there because it is not type text at all.
SCAN_ANCHORS = (
    Signature(
        'type handler func(arg stepType)', frozenset(), frozenset(),
        '\ttype handler func(arg stepType)\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\ttype handler func(arg stepType)\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        'call := func(arg stepType)', frozenset({'arg'}), frozenset({'call'}),
        '\tcall := func(arg stepType) {\n' + RUN + '\t}\n\tcall(0)\n',
        '\tcall := func(stepOne stepType) {\n' + BOUND_RUN + '\t}\n\tcall(0)\n'),
    Signature(
        'var local func(arg stepType)', frozenset(), frozenset({'local'}),
        '\tvar local func(arg stepType)\n\t_ = local\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tvar stepOne func(arg stepType)\n\t_ = stepOne\n'
        '\tproofkit.Run(t, spurCases(), stepTwo)\n'),
    # A row holding its run inside the literal calls that literal rather than discarding it,
    # because a run inside a literal stored under a name that is never called is unreadable on
    # the reached-literal axis, which ReachedLiteralTest owns. These rows are about which names
    # a SIGNATURE binds, and the call keeps them asking only that.
    Signature(
        'func(arg stepType) error', frozenset({'arg'}), frozenset({'call'}),
        '\tcall := func(arg stepType) error {\n' + RUN
        + '\t\treturn nil\n\t}\n\t_ = call(0)\n',
        '\tcall := func(stepOne stepType) error {\n' + BOUND_RUN
        + '\t\treturn nil\n\t}\n\t_ = call(0)\n'),
    Signature(
        'func() func(arg stepType), a result type', frozenset(), frozenset({'call'}),
        '\tcall := func() func(stepOne stepType) {\n'
        '\t\treturn nil\n\t}\n\t_ = call\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) func(arg stepType) {\n' + BOUND_RUN
        + '\t\treturn nil\n\t}\n\t_ = call(0)\n'),
    Signature(
        '[]func(arg stepType){}, an element type', frozenset(), frozenset({'table'}),
        '\ttable := []func(stepOne stepType){}\n\t_ = table\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\ttable := []func(arg stepType){}\n\t\t_ = table\n' + BOUND_RUN
        + '\t}\n\tcall(0)\n'),
    # The half of the pointer prefix that must NOT move. `[]*func(arg stepType){}` is a func
    # TYPE and this is a product whose right operand is an invoked literal, and the two are
    # told apart by the space gofmt writes around a binary operator and never inside a pointer
    # type. Reading this one as type text would drop a real parameter name, which is the
    # direction that can leave a run counted when it should not be.
    Signature(
        'm[0] * func(arg stepType) int { ... }(0), a product', frozenset({'arg'}),
        frozenset({'m'}),
        '\tm := []int{1}\n'
        '\t_ = m[0] * func(arg stepType) int { return 1 }(0)\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tm := []int{1}\n'
        '\t_ = m[0] * func(stepOne stepType) int { return 1 }(0)\n'
        '\tproofkit.Run(t, spurCases(), stepTwo)\n'),
)

# The chokepoint working as designed, and a fixture so the next reader meets it as a decision.
#
# The inner literal below is a real func literal and `stepOne` is a real binding of it, in a
# scope the Test body cannot reach. This gate tracks no scope by construction — that is what
# ended the four rounds of header-shape repairs recorded above the chokepoint — so it reads the
# name as bound and reports the run unreadable. The verdict is a false failure, which is the
# price this design pays deliberately, and it is the correct verdict for this gate.
#
# It sits one keyword apart from the result-type row above: there the same parameter list stands
# in a TYPE, which binds nothing anywhere in the program, and the run must register. Separating
# those two is the whole of that fix, so this row is what pins the half that must NOT move.
ScopeBlind = collections.namedtuple('ScopeBlind', 'shape binds body')

SCOPE_BLIND_SHAPES = (
    ScopeBlind(
        'a literal returned by a literal', frozenset({'call', 'stepOne'}),
        '\tcall := func() func(arg stepType) {\n'
        '\t\treturn func(stepOne stepType) {}\n\t}\n\t_ = call\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n'),
)

# A func TYPE written inside larger type text, in every context that announces itself with a
# bracket, and one that announces itself with nothing.
#
# The element-type row in SCAN_ANCHORS covers one of these — a bracket standing directly in
# front of the keyword. It is not the shape. The shape is a `func` keyword standing in TYPE
# text, and the bracket is one of the ways it is announced: an element type can carry `*`,
# `chan` or `<-chan` in front of its own keyword, and a bracketed element type can be a func
# type whose RESULT is another func type, where the inner keyword has a `)` in front of it and
# no bracket anywhere near.
#
# Seven of the twelve rows below were read as func literals: the parameter list of the type was
# collected as a binding, `stepOne` came out bound in a Test body that binds nothing, and the
# run built with the package-level `stepOne` was reported unreadable. The verdict was a false
# failure, never a false pass, because a spuriously bound name can only send a run down the
# chokepoint. The other five rows already read correctly and are here so a later change cannot
# move them: four through the bracket test, and `chan func(...)` at the top of a var
# declaration only because no brace follows it, which is an accident of that shape rather than
# a decision anything makes.
#
# Every row is written twice, like the tables above: with the type's parameter named `stepOne`,
# where the run must register because a type binds nothing, and with a real literal binding
# `stepOne` around the same type, where the run must become unreadable.


def type_text_row(type_text):
    """A row whose two directions write `type_text` with its parameter named two ways.

    The type direction writes it as the type of a composite literal, which is where a func
    type standing in type text is followed by a brace and reads locally like a literal.
    """
    return Signature(
        type_text % 'arg', frozenset(), frozenset({'f'}),
        '\tf := %s\n\t_ = f\n' % (type_text % 'stepOne')
        + '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\tf := %s\n\t\t_ = f\n' % (type_text % 'arg')
        + BOUND_RUN + '\t}\n\tcall(0)\n')


TYPE_TEXT_SHAPES = tuple(type_text_row(type_text) for type_text in (
    '[]func(%s stepType){}',
    'map[string]func(%s stepType){}',
    '[2]func(%s stepType){}',
    '[][]func(%s stepType){}',
    '[]func() func(%s stepType){}',
    'map[string]func() func(%s stepType){}',
    '[2]func() func(%s stepType){}',
    '[][]func() func(%s stepType){}',
    '[]func(a stepType) func(%s stepType){}',
    'map[string]chan func(%s stepType){}',
    '[]*func(%s stepType){}',
)) + (
    Signature(
        'chan func(arg stepType), declared with var', frozenset(), frozenset({'ch'}),
        '\tvar ch chan func(stepOne stepType)\n\t_ = ch\n'
        '\tproofkit.Run(t, spurCases(), stepOne)\n',
        '\tcall := func(stepOne stepType) {\n'
        '\t\tvar ch chan func(arg stepType)\n\t\t_ = ch\n' + BOUND_RUN
        + '\t}\n\tcall(0)\n'),
)

# The Go package the shapes are compiled in. `stepType` is the package-level type whose name the
# gate used to read as a binding, `stepOne` and `stepTwo` are the step functions a run can
# register, and the proof kit is stubbed down to the shapes the runs need, so the fixtures vet
# without the real engines.
FIXTURE_MODULE = 'module fixture\n\ngo 1.24\n'

FIXTURE_PROOFKIT = (
    'package proofkit\n\n'
    'import "testing"\n\n'
    'type Case int\n\n'
    'type Build func(int)\n\n'
    'func Run(t *testing.T, cases []Case, build Build) {\n'
    '\t_, _, _ = t, cases, build\n'
    '}\n')

FIXTURE_SUPPORT = (
    'package proof\n\n'
    'import "fixture/proofkit"\n\n'
    'type stepType int\n\n'
    'type pair[T any] struct{}\n\n'
    'func spurCases() []proofkit.Case { return nil }\n\n'
    'func stepOne(int) {}\n\n'
    'func stepTwo(int) {}\n')

FIXTURE_TEST_HEADER = (
    'package proof\n\n'
    'import (\n'
    '\t"testing"\n\n'
    '\t"fixture/proofkit"\n'
    ')\n')


class GoBodyFixtureTest(unittest.TestCase):
    """What reads the fixture bodies above, shared by the tables that hold them.

    The gate reads each body, and gofmt and `go vet` read it too, so a shape that only looks
    like Go cannot pass as evidence here.
    """

    def source(self, body):
        return 'func TestOne(t *testing.T) {\n' + body + '}\n'

    def bound_names(self, body):
        src = self.source(body)
        spans = list(COMPILE_CHECKER.go_func_body_spans(
            src, COMPILE_CHECKER.GO_TEST_FUNCTION_NAME))
        self.assertEqual(len(spans), 1, src)
        _, start, end = spans[0]
        return COMPILE_CHECKER.go_bound_names(src[start:end])

    def outcome(self, body):
        registered, misnamed, unreadable = (
            COMPILE_CHECKER.registered_step_functions(self.source(body)))
        if registered == {'stepOne'} and not misnamed and not unreadable:
            return REGISTERED
        self.assertEqual(registered, set(), body)
        self.assertEqual(misnamed, [], body)
        self.assertEqual(len(unreadable), 1, body)
        reason = unreadable[0][1]
        return LOCAL if reason.startswith(LOCAL_STEP_PREFIX) else reason

    def assert_gofmt_writes_every_source(self, labelled_bodies):
        """gofmt itself, not a guess at it.

        A shape gofmt would reformat is a shape no drafter would commit, and a table of those
        proves nothing about the proofs this gate reads.
        """
        if shutil.which('gofmt') is None:
            self.skipTest('gofmt is not on PATH')
        for label, body in labelled_bodies:
            with self.subTest(shape=label):
                src = 'package proof\n\n' + self.source(body)

                formatted = subprocess.run(
                    ['gofmt'], input=src, capture_output=True, text=True, check=True)

                self.assertEqual(formatted.stdout, src)

    def assert_every_source_compiles_and_vets(self, labelled_bodies):
        """One `go vet` over every body given, in one package the compiler type-checks.

        A shape that does not compile is not a shape the gate can ever meet, so pinning the
        gate's reading of one would pin a reading of nothing. Vetting them together is also how
        a matrix's two directions stay honest: the bound direction has to keep the same shape
        and change only the name, and a rename that does not type-check fails here.
        """
        vetted = self.vet_sources(labelled_bodies)

        self.assertEqual(vetted.returncode, 0, vetted.stderr)

    def vet_sources(self, labelled_bodies):
        """Run `go vet` over the bodies given and return the finished process.

        A table that pins a name Go itself refuses needs the refusal, not just a zero exit, so
        the run is handed back rather than asserted on here.
        """
        if shutil.which('go') is None:
            self.skipTest('go is not on PATH')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'go.mod').write_text(FIXTURE_MODULE)
            (root / 'proofkit').mkdir()
            (root / 'proofkit' / 'proofkit.go').write_text(FIXTURE_PROOFKIT)
            (root / 'proof').mkdir()
            (root / 'proof' / 'support.go').write_text(FIXTURE_SUPPORT)
            tests = [FIXTURE_TEST_HEADER]
            for index, (_, body) in enumerate(labelled_bodies):
                tests.append(self.source(body).replace(
                    'func TestOne(', 'func TestShape%02d(' % index, 1))
            (root / 'proof' / 'shapes_test.go').write_text('\n'.join(tests))
            environment = dict(os.environ, GOWORK='off', GOTOOLCHAIN='local', GOFLAGS='-mod=mod')

            return subprocess.run(
                ['go', 'vet', './...'], cwd=directory, env=environment,
                capture_output=True, text=True)


class ScrubbedBodyFixtureTest(GoBodyFixtureTest):
    """A fixture table read the way the gate reads a proof: after the literals are scrubbed.

    A literal left standing in the raw fixture text ends its own statement, so a table that
    skipped the scrubber would pass with a scrubber defect in place and say nothing about the
    copy the gate actually reads.
    """

    def bound_names(self, body):
        return super().bound_names(COMPILE_CHECKER.strip_go_comments_and_literals(body))

    def outcome(self, body):
        return super().outcome(COMPILE_CHECKER.strip_go_comments_and_literals(body))


class SignatureShapeTest(GoBodyFixtureTest):
    """The signature matrix, in both directions, against real Go."""

    SHAPES = SIGNATURE_SHAPES + SCAN_ANCHORS

    def labelled_bodies(self):
        return [('%s, %s direction' % (shape.shape, direction), body)
                for shape in self.SHAPES
                for direction, body in (('type', shape.body), ('binding', shape.bound))]

    def test_every_shape_binds_exactly_the_required_names(self):
        for shape in self.SHAPES:
            with self.subTest(shape=shape.shape):
                self.assertEqual(
                    self.bound_names(shape.body), set(shape.required | shape.incidental))

    def test_no_shape_makes_the_body_unreadable(self):
        for shape in self.SHAPES:
            with self.subTest(shape=shape.shape):
                self.assertEqual(self.outcome(shape.body), REGISTERED)

    def test_a_step_named_binding_in_the_same_shape_still_does(self):
        for shape in self.SHAPES:
            with self.subTest(shape=shape.shape):
                self.assertEqual(self.outcome(shape.bound), LOCAL)

    def test_every_source_is_what_gofmt_writes(self):
        self.assert_gofmt_writes_every_source(self.labelled_bodies())

    def test_every_source_compiles_and_vets(self):
        self.assert_every_source_compiles_and_vets(self.labelled_bodies())


class TypeTextShapeTest(SignatureShapeTest):
    """The twelve-variant type-text table, read the same way the signature matrix is.

    Every row's type direction has to bind nothing and register, and every row's bound
    direction has to bind `stepOne` and stop the run, so the two halves of the rule are pinned
    against the same Go the signature matrix is pinned against.
    """

    SHAPES = TYPE_TEXT_SHAPES


class ScopeBlindShapeTest(GoBodyFixtureTest):
    """The false failures the chokepoint is designed to take, pinned as such.

    A shape here binds a `step<Title>` name in a scope the run cannot see, and the gate reports
    the run unreadable anyway. That is the design and not a defect, so it is asserted rather
    than left for a later reader to read as one and 'fix'.
    """

    SHAPES = SCOPE_BLIND_SHAPES

    def labelled_bodies(self):
        return [(shape.shape, shape.body) for shape in self.SHAPES]

    def test_every_shape_binds_exactly_the_names_it_really_binds(self):
        for shape in self.SHAPES:
            with self.subTest(shape=shape.shape):
                self.assertEqual(self.bound_names(shape.body), set(shape.binds))

    def test_every_shape_stays_unreadable(self):
        for shape in self.SHAPES:
            with self.subTest(shape=shape.shape):
                self.assertEqual(self.outcome(shape.body), LOCAL)

    def test_every_source_is_what_gofmt_writes(self):
        self.assert_gofmt_writes_every_source(self.labelled_bodies())

    def test_every_source_compiles_and_vets(self):
        self.assert_every_source_compiles_and_vets(self.labelled_bodies())


# ---------------------------------------------------------------------------------------------
# A var block's spec boundaries, and what an initializer written as a literal does to them.
#
# Go ends a var spec at a line break whose last token can end a statement, and a literal is
# such a token. The gate reads the block after the scrubber has blanked every literal, so the
# scrubber has to leave that token standing. Blanking a literal's delimiters along with its
# text left `label = "one"` reading as a line that continues into the next one: the block was
# then read as ONE spec, only the first spec's names came out bound, and a `step<Title>`
# declared under a string-initialised spec was invisible to the chokepoint. The run built with
# that local while the gate reported the package-level step of the same name as registered,
# which is the silent false pass the chokepoint exists to convert into a loud one.
#
# Each initializer is written twice, because the two directions fail differently: with the
# second spec binding `stepOne`, where the run must become unreadable, and with it binding an
# ordinary name, where the run must still register.
VarSpecInitializer = collections.namedtuple('VarSpecInitializer', 'kind literal')

VAR_SPEC_INITIALIZERS = (
    VarSpecInitializer('an int', '1'),
    VarSpecInitializer('a string', '"one"'),
    VarSpecInitializer('a rune', "'x'"),
    VarSpecInitializer('a raw string', '`one`'),
)


class VarSpecInitializerTest(ScrubbedBodyFixtureTest):
    """Every initializer kind splits a var block the same way, in both directions."""

    def body(self, literal, name):
        """A var block whose first spec holds the literal and whose second binds `name`.

        gofmt aligns a group's `=` signs, so the first name is padded to the second's width.
        That padding is asserted rather than assumed: `test_every_source_is_what_gofmt_writes`
        runs gofmt over each body.
        """
        return ('\tvar (\n'
                '\t\t%s = %s\n'
                '\t\t%s = stepTwo\n'
                '\t)\n'
                '\t_, _ = label, %s\n'
                '\tproofkit.Run(t, spurCases(), stepOne)\n'
                % ('label'.ljust(len(name)), literal, name, name))

    def labelled_bodies(self):
        return [('%s, %s direction' % (initializer.kind, direction), self.body(
                    initializer.literal, name))
                for initializer in VAR_SPEC_INITIALIZERS
                for direction, name in (('bound', 'stepOne'), ('free', 'localBuild'))]

    def test_every_initializer_binds_both_spec_names(self):
        """The direct probe: a spec the scrubber ended wrongly loses the names after it."""
        for initializer in VAR_SPEC_INITIALIZERS:
            for name in ('stepOne', 'localBuild'):
                with self.subTest(initializer=initializer.kind, name=name):
                    self.assertEqual(
                        self.bound_names(self.body(initializer.literal, name)),
                        {'label', name})

    def test_every_initializer_leaves_the_step_named_local_caught(self):
        for initializer in VAR_SPEC_INITIALIZERS:
            with self.subTest(initializer=initializer.kind):
                self.assertEqual(
                    self.outcome(self.body(initializer.literal, 'stepOne')), LOCAL)

    def test_every_initializer_leaves_an_ordinary_local_registered(self):
        for initializer in VAR_SPEC_INITIALIZERS:
            with self.subTest(initializer=initializer.kind):
                self.assertEqual(
                    self.outcome(self.body(initializer.literal, 'localBuild')), REGISTERED)

    def test_every_source_is_what_gofmt_writes(self):
        self.assert_gofmt_writes_every_source(self.labelled_bodies())

    def test_every_source_compiles_and_vets(self):
        self.assert_every_source_compiles_and_vets(self.labelled_bodies())


# ---------------------------------------------------------------------------------------------
# The reached-literal axis: does the func literal holding a run ever run?
#
# A func literal encloses what is written in it, so the guard axis reads a run inside one as
# running when the statement around the literal runs. For a literal stored under a name and
# never called, that statement runs and the run does not, and counting the step registered is a
# silent false pass. The POSITION the literal is written in separates the two, and both halves
# are pinned here, because the cheap rule — report every func-literal enclosure unreadable —
# takes the `t.Run` subtest with it, which is the false failure this module was rewritten to
# stop producing.
Position = collections.namedtuple('Position', 'position verdict body')

RUN_IN_A_LITERAL = '\t\tproofkit.Run(t, spurCases(), stepOne)\n'

LITERAL_POSITIONS = (
    Position('stored under a name and never called', UNREADABLE_CLOSURE,
             '\tunused := func() {\n' + RUN_IN_A_LITERAL + '\t}\n\t_ = unused\n'),
    Position('declared with var and never called', UNREADABLE_CLOSURE,
             '\tvar unused = func() {\n' + RUN_IN_A_LITERAL + '\t}\n\t_ = unused\n'),
    Position('stored under a name and called', REGISTERED,
             '\tcall := func() {\n' + RUN_IN_A_LITERAL + '\t}\n\tcall()\n'),
    Position('handed to t.Run', REGISTERED,
             '\tt.Run("one", func(t *testing.T) {\n' + RUN_IN_A_LITERAL + '\t})\n'),
    Position('invoked where it stands', REGISTERED,
             '\tfunc() {\n' + RUN_IN_A_LITERAL + '\t}()\n'),
    Position('deferred', REGISTERED,
             '\tdefer func() {\n' + RUN_IN_A_LITERAL + '\t}()\n'),
    Position('started with go', REGISTERED,
             '\tgo func() {\n' + RUN_IN_A_LITERAL + '\t}()\n'),
)


class ReachedLiteralTest(ScrubbedBodyFixtureTest):
    """Every position a func literal holding a run can stand in, and what the gate says."""

    def labelled_bodies(self):
        return [(position.position, position.body) for position in LITERAL_POSITIONS]

    def test_every_position_reads_as_the_table_says(self):
        for position in LITERAL_POSITIONS:
            with self.subTest(position=position.position):
                self.assertEqual(self.outcome(position.body), position.verdict)

    def test_every_source_is_what_gofmt_writes(self):
        self.assert_gofmt_writes_every_source(self.labelled_bodies())

    def test_every_source_compiles_and_vets(self):
        self.assert_every_source_compiles_and_vets(self.labelled_bodies())


# ---------------------------------------------------------------------------------------------
# Which function names `go test` runs.
#
# Go's rule is `Test` followed by a rune that is not a lower-case letter, `Test` alone
# included, and `go vet` enforces the same rule on a function taking `*testing.T`. Reading only
# `Test[A-Z]\w*` left a run in a `Test_Profile` or `Test1` body reported as 'not inside a Go
# Test function', which names the wrong thing to fix, since the Test was there and running.
#
# The rule is Unicode, and `[^a-z]` was the ASCII reading of it. `Testé`, `Testα`, `Testñ` and
# `Testı` are not tests: `go vet` refuses each one with the same malformed-name error it gives
# `Testing`, and `go test` runs none of them. Reading them as tests counted the runs in them as
# registered, which is a false pass, so the four stand in the rejected table below with the
# refusal asserted for each.
#
# `TestÉ` is the other side of the same rule and registers, because an upper-case letter is not
# a lower-case one whatever alphabet it comes from. `Testʰ` is the row that separates Go's rule
# from Python's `str.islower()`: U+02B0 is Other_Lowercase but category Lm, so Go runs it and
# Python's own predicate would have called it lower and dropped it.
#
# The rejected rows are not vetted with the accepted ones: `go vet` refuses them, and that
# refusal is asserted below rather than worked around, because it is the evidence that the gate
# and Go draw the same line.
TestName = collections.namedtuple('TestName', 'name verdict')

TEST_FUNCTION_NAMES = (
    TestName('Test', REGISTERED),
    TestName('TestOne', REGISTERED),
    TestName('Test_Foo', REGISTERED),
    TestName('Test1', REGISTERED),
    TestName('TestÉ', REGISTERED),
    TestName('Testʰ', REGISTERED),
)

REJECTED_TEST_FUNCTION_NAMES = (
    TestName('Testing', UNREADABLE_OUTSIDE_TEST),
    TestName('Testé', UNREADABLE_OUTSIDE_TEST),
    TestName('Testα', UNREADABLE_OUTSIDE_TEST),
    TestName('Testñ', UNREADABLE_OUTSIDE_TEST),
    TestName('Testı', UNREADABLE_OUTSIDE_TEST),
)


class TestFunctionNameTest(GoBodyFixtureTest):
    """Every name shape, against the gate and against `go vet`."""

    def source(self, body):
        """The fixture is the whole function here, because the name is what is under test."""
        return body

    def function(self, name):
        return ('func %s(t *testing.T) {\n'
                '\tproofkit.Run(t, spurCases(), stepOne)\n'
                '}\n' % name)

    def labelled_bodies(self):
        return [(name.name, self.function(name.name)) for name in TEST_FUNCTION_NAMES]

    def test_every_accepted_name_registers_its_run(self):
        for name in TEST_FUNCTION_NAMES:
            with self.subTest(name=name.name):
                self.assertEqual(self.outcome(self.function(name.name)), name.verdict)

    def test_every_rejected_name_leaves_the_run_outside_every_test(self):
        for rejected in REJECTED_TEST_FUNCTION_NAMES:
            with self.subTest(name=rejected.name):
                self.assertEqual(self.outcome(self.function(rejected.name)), rejected.verdict)

    def test_go_vet_rejects_every_name_the_gate_rejects(self):
        """Go itself, not this table, is what says these names are no tests.

        One vet run per name, because a run over all of them together proves only that one of
        them is malformed.
        """
        for rejected in REJECTED_TEST_FUNCTION_NAMES:
            with self.subTest(name=rejected.name):
                vetted = self.vet_sources([(rejected.name, self.function(rejected.name))])

                self.assertNotEqual(vetted.returncode, 0)
                self.assertIn('malformed name', vetted.stderr)

    def test_every_source_is_what_gofmt_writes(self):
        self.assert_gofmt_writes_every_source(
            self.labelled_bodies()
            + [(rejected.name, self.function(rejected.name))
               for rejected in REJECTED_TEST_FUNCTION_NAMES])

    def test_every_accepted_source_compiles_and_vets(self):
        self.assert_every_source_compiles_and_vets(self.labelled_bodies())


# ---------------------------------------------------------------------------------------------
# A build constraint on a proof file.
#
# `go test` never compiles a file its build constraint excludes, so a run written in one never
# executes and registers nothing. The gate reads every `*_test.go` in the proof directory, and
# the scrubber blanks the constraint along with every other comment before anything looks at
# it, so a run parked behind `//go:build never` used to come back registered. That is a false
# pass with no second line of defence: the proof runs green because Go skips the file, and the
# gate says the step it holds is proven.
#
# Only the two never-build markers are honoured, which is the whole realistic case. The table
# below pins that boundary in both directions, including the constraints that are read as if
# the file compiled, so the gate's own documentation of what it honours is mechanical rather
# than a claim.
BuildConstraint = collections.namedtuple('BuildConstraint', 'shape header built')

BUILD_CONSTRAINTS = (
    BuildConstraint('no constraint', '', True),
    BuildConstraint('//go:build never', '//go:build never\n\n', False),
    BuildConstraint('//go:build ignore', '//go:build ignore\n\n', False),
    BuildConstraint('//go:build linux', '//go:build linux\n\n', True),
    BuildConstraint('//go:build !windows', '//go:build !windows\n\n', True),
    BuildConstraint('//go:build never && linux', '//go:build never && linux\n\n', True),
)

# The legacy build line, which is not in the table because gofmt does not leave it standing: it
# writes the `//go:build` form above it, and that form is honoured. The rewrite is asserted
# below, so "the legacy spelling is not read" is a statement about a file no drafter can commit
# rather than a hole in the rule.
LEGACY_BUILD_LINE = '// +build never\n\n'

CONSTRAINED_RUN = (
    'func TestExcluded(t *testing.T) {\n'
    '\tproofkit.Run(t, spurCases(), stepTwo)\n'
    '}\n')

UNCONSTRAINED_RUN = (
    'func TestIncluded(t *testing.T) {\n'
    '\tproofkit.Run(t, spurCases(), stepOne)\n'
    '}\n')


def constrained_file(header, body):
    """A whole `_test.go` file: the constraint header, the package clause, then the body."""
    return header + FIXTURE_TEST_HEADER + '\n' + body


class BuildConstraintTest(unittest.TestCase):
    """What a build constraint on a proof file does to the registrations read out of it."""

    def registrations(self, files):
        """`proof_registrations` over a directory holding exactly the files given."""
        with tempfile.TemporaryDirectory() as directory:
            for name, text in files:
                (Path(directory) / name).write_text(text, encoding='utf-8')
            found, misnamed, unreadable = COMPILE_CHECKER.proof_registrations(directory)
            return found, misnamed, [(label, reason) for _, label, reason in unreadable]

    def test_a_never_built_file_registers_nothing(self):
        files = [('excluded_test.go', constrained_file('//go:build never\n\n', CONSTRAINED_RUN))]

        self.assertEqual(self.registrations(files), (set(), [], []))

    def test_an_unconstrained_file_still_registers(self):
        files = [('proof_test.go', constrained_file('', UNCONSTRAINED_RUN))]

        self.assertEqual(self.registrations(files), ({'stepOne'}, [], []))

    def test_only_the_compiled_file_of_the_two_registers(self):
        """Both in one directory, which is how the shape reaches a real proof."""
        files = [
            ('excluded_test.go', constrained_file('//go:build never\n\n', CONSTRAINED_RUN)),
            ('proof_test.go', constrained_file('', UNCONSTRAINED_RUN)),
        ]

        self.assertEqual(self.registrations(files), ({'stepOne'}, [], []))

    def test_every_constraint_is_honoured_or_read_as_the_table_says(self):
        for constraint in BUILD_CONSTRAINTS:
            with self.subTest(constraint=constraint.shape):
                files = [('proof_test.go',
                          constrained_file(constraint.header, UNCONSTRAINED_RUN))]

                registered, _, _ = self.registrations(files)

                self.assertEqual(registered, {'stepOne'} if constraint.built else set())

    def test_a_constraint_below_the_package_clause_is_not_a_constraint(self):
        """Go reads `//go:build` only above the package clause, and so does this."""
        files = [('proof_test.go', constrained_file('', UNCONSTRAINED_RUN)
                  + '\n// //go:build never\nfunc helper() {}\n')]

        registered, _, _ = self.registrations(files)

        self.assertEqual(registered, {'stepOne'})

    def test_every_source_is_what_gofmt_writes(self):
        if shutil.which('gofmt') is None:
            self.skipTest('gofmt is not on PATH')
        for constraint in BUILD_CONSTRAINTS:
            with self.subTest(constraint=constraint.shape):
                text = constrained_file(constraint.header, UNCONSTRAINED_RUN)

                formatted = subprocess.run(
                    ['gofmt'], input=text, capture_output=True, text=True, check=True)

                self.assertEqual(formatted.stdout, text)

    def test_gofmt_rewrites_the_legacy_build_line_into_the_honoured_one(self):
        if shutil.which('gofmt') is None:
            self.skipTest('gofmt is not on PATH')
        text = constrained_file(LEGACY_BUILD_LINE, UNCONSTRAINED_RUN)

        formatted = subprocess.run(
            ['gofmt'], input=text, capture_output=True, text=True, check=True)

        self.assertTrue(formatted.stdout.startswith('//go:build never\n// +build never\n'),
                        formatted.stdout)
        rewritten = [('proof_test.go', formatted.stdout)]
        self.assertEqual(self.registrations(rewritten), (set(), [], []))

    def test_go_test_runs_only_the_unconstrained_file(self):
        """Go itself, not this table, is what says the excluded file never runs."""
        if shutil.which('go') is None:
            self.skipTest('go is not on PATH')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'go.mod').write_text(FIXTURE_MODULE)
            (root / 'proofkit').mkdir()
            (root / 'proofkit' / 'proofkit.go').write_text(FIXTURE_PROOFKIT)
            (root / 'proof').mkdir()
            (root / 'proof' / 'support.go').write_text(FIXTURE_SUPPORT)
            (root / 'proof' / 'excluded_test.go').write_text(
                constrained_file('//go:build never\n\n', CONSTRAINED_RUN))
            (root / 'proof' / 'proof_test.go').write_text(
                constrained_file('', UNCONSTRAINED_RUN))
            environment = dict(os.environ, GOWORK='off', GOTOOLCHAIN='local', GOFLAGS='-mod=mod')

            vetted = subprocess.run(
                ['go', 'vet', './...'], cwd=directory, env=environment,
                capture_output=True, text=True)
            listed = subprocess.run(
                ['go', 'test', '-list', '.*', './proof'], cwd=directory, env=environment,
                capture_output=True, text=True)

            self.assertEqual(vetted.returncode, 0, vetted.stderr)
            self.assertEqual(listed.returncode, 0, listed.stderr)
            self.assertIn('TestIncluded', listed.stdout)
            self.assertNotIn('TestExcluded', listed.stdout)


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
