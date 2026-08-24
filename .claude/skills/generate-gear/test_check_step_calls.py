#!/usr/bin/env python3
"""Regression tests for the step-call coverage gate."""
import contextlib
import importlib.util
import io
import json
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
    def run_checker_argv(self, steps, candidate, flags=(), flags_first=False):
        """Run the checker with optional output-mode flags, capturing stdout and stderr."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            steps_path = root / 'steps.md'
            candidate_path = root / 'candidate.py'
            steps_path.write_text(steps)
            candidate_path.write_text(candidate)
            positionals = [str(steps_path), str(candidate_path)]
            flags = list(flags)
            tail = flags + positionals if flags_first else positionals + flags
            output = io.StringIO()
            errors = io.StringIO()
            with warnings.catch_warnings(), contextlib.redirect_stdout(output), \
                    contextlib.redirect_stderr(errors):
                warnings.simplefilter('ignore', ResourceWarning)
                result = CHECKER.main(['check_step_calls.py'] + tail)
            return result, output.getvalue(), errors.getvalue()

    def run_checker(self, steps, candidate):
        result, output, _ = self.run_checker_argv(steps, candidate)
        return result, output

    def test_forbidden_call_is_not_required(self):
        steps = 'Call `safeCall()`. Do not read the direction via `getTangent(0)`.'

        self.assertEqual(CHECKER.named_calls(steps), {'safeCall'})
        result, output = self.run_checker(steps, 'safeCall()\n')

        self.assertEqual(result, 0, output)
        self.assertNotIn('getTangent', output)

    def test_forbidden_call_stays_exempt_when_the_sentence_wraps(self):
        # The prohibition and the call it forbids land on different Markdown
        # lines. Where a paragraph wraps says nothing about where its sentences
        # end, so the wrap must not turn the example into a requirement.
        steps = (
            'Call `safeCall()`. Never read the direction\n'
            'with `getTangent(0)`: parameter 0 may sit outside the range.')

        self.assertEqual(CHECKER.named_calls(steps), {'safeCall'})
        result, output = self.run_checker(steps, 'safeCall()\n')

        self.assertEqual(result, 0, output)
        self.assertNotIn('getTangent', output)

    def test_required_call_on_the_line_after_a_finished_prohibition(self):
        # The prohibition ended with its own sentence, and the next block is a
        # bullet, so the call in it is still required.
        steps = (
            'Never read the direction with `getTangent(0)`\n'
            '- Call `safeCall()`\n')

        self.assertEqual(CHECKER.named_calls(steps), {'safeCall'})
        result, output = self.run_checker(steps, 'pass\n')

        self.assertEqual(result, 1, output)
        self.assertIn('safeCall', output)
        self.assertNotIn('getTangent', output)

    def test_not_a_substitute_call_is_not_required(self):
        steps = (
            'Use the angular pin. A plain `GeometricConstraints.addHorizontal(line)` '
            'is not a substitute at angle 0.')

        self.assertEqual(
            CHECKER.named_calls(steps),
            set())
        result, output = self.run_checker(steps, 'pass\n')

        self.assertEqual(result, 0, output)
        self.assertNotIn('addHorizontal', output)

    def test_comment_only_call_still_fails_coverage(self):
        steps = 'Call `safeCall()`.'

        result, output = self.run_checker(steps, '# safeCall()\n')

        self.assertEqual(result, 1)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)

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

    def test_short_dotted_calls_are_required(self):
        steps = 'Call `sketch.add()` and `sketch.set()`.'

        self.assertEqual(
            CHECKER.named_call_shapes(steps),
            {('add', 'sketch'), ('set', 'sketch')})
        result, output = self.run_checker(steps, 'sketch.add(None)\n')

        self.assertEqual(result, 1)
        self.assertIn("receiver.set('", output)

    def test_unreachable_branch_does_not_cover_named_call(self):
        steps = 'Call `sketch.requiredFusionCall()`.'
        candidate = 'if False:\n    sketch.requiredFusionCall()\n'

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)

    def test_unreachable_helper_does_not_cover_named_call(self):
        steps = 'Call `sketch.requiredFusionCall()`.'
        candidate = (
            'def unused_helper():\n'
            '    sketch.requiredFusionCall()\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)

    def test_reachability_stays_with_the_called_class_method(self):
        steps = 'Call `sketch.project(entity)`.'
        candidate = (
            'class A:\n'
            '    def generate(self):\n'
            '        return A.helper(self)\n'
            '    def helper(self):\n'
            '        return None\n'
            '\n'
            'class B:\n'
            '    def helper(self):\n'
            '        return sketch.project(entity)\n'
            '\n'
            'def generate():\n'
            '    return A.generate(None)\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)


    def test_call_through_a_local_alias_covers_the_named_method(self):
        # The generated spur module binds `pn = self.parameterName` and builds every
        # derived-parameter expression through `pn`, so the alias is where the call
        # actually happens.
        steps = 'Call `self.parameterName(name)` and `parameterName(name)`.'
        candidate = (
            'class Gear:\n'
            '    def parameterName(self, name):\n'
            '        return name\n'
            '    def generate(self):\n'
            '        pn = self.parameterName\n'
            '        return pn("module")\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 0, output)

    def test_annotated_alias_also_covers_the_named_method(self):
        steps = 'Call `self.parameterName(name)`.'
        candidate = (
            'class Gear:\n'
            '    def parameterName(self, name):\n'
            '        return name\n'
            '    def generate(self):\n'
            '        pn: object = self.parameterName\n'
            '        return pn("module")\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 0, output)

    def test_alias_of_another_method_does_not_cover_the_named_method(self):
        steps = 'Call `self.parameterName(name)`.'
        candidate = (
            'class Gear:\n'
            '    def parameterName(self, name):\n'
            '        return name\n'
            '    def otherName(self, name):\n'
            '        return name\n'
            '    def generate(self):\n'
            '        pn = self.otherName\n'
            '        return pn("module")\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1, output)
        self.assertIn('parameterName', output)

    def test_alias_that_is_never_called_does_not_cover_the_named_method(self):
        steps = 'Call `self.parameterName(name)`.'
        candidate = (
            'class Gear:\n'
            '    def parameterName(self, name):\n'
            '        return name\n'
            '    def generate(self):\n'
            '        pn = self.parameterName\n'
            '        return pn\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1, output)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)

    def test_alias_rebound_to_something_else_stops_covering_the_named_method(self):
        steps = 'Call `self.parameterName(name)`.'
        candidate = (
            'class Gear:\n'
            '    def parameterName(self, name):\n'
            '        return name\n'
            '    def generate(self):\n'
            '        pn = self.parameterName\n'
            '        pn = str\n'
            '        return pn("module")\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 1, output)
        self.assertIn('textual match exists, but it is not a reachable executable call', output)

    def test_alias_of_a_module_function_covers_the_named_function(self):
        steps = 'Call `requiredHelper(value)`.'
        candidate = (
            'def requiredHelper(value):\n'
            '    return value\n'
            '\n'
            'def generate():\n'
            '    helper = requiredHelper\n'
            '    return helper(1)\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 0, output)

    def test_alias_call_reaches_the_body_of_the_aliased_method(self):
        steps = 'Call `sketch.requiredFusionCall()`.'
        candidate = (
            'class Gear:\n'
            '    def helper(self):\n'
            '        return sketch.requiredFusionCall()\n'
            '    def generate(self):\n'
            '        run = self.helper\n'
            '        return run()\n')

        result, output = self.run_checker(steps, candidate)

        self.assertEqual(result, 0, output)


    # The two machine-readable modes `/compile-gear` step 5 consumes.

    def test_names_prints_bare_names_of_missing_calls(self):
        steps = 'Call `sketch.addByTwoPoints(start, end)` and `helperCall(value)`.'

        result, output, errors = self.run_checker_argv(steps, 'pass\n', flags=['--names'])

        self.assertEqual(result, 1, output)
        self.assertEqual(output, 'addByTwoPoints\nhelperCall\n')
        self.assertEqual(errors, '')

    def test_names_is_empty_but_still_blocking_for_a_stub_marker(self):
        steps = 'Call `safeCall()`.'

        result, output, _ = self.run_checker_argv(
            steps, 'safeCall()\n# TODO fill in\n', flags=['--names'])

        self.assertEqual(result, 1)
        self.assertEqual(output, '')

    def test_names_is_empty_on_a_green_pair(self):
        steps = 'Call `safeCall()`.'

        result, output, _ = self.run_checker_argv(steps, 'safeCall()\n', flags=['--names'])

        self.assertEqual(result, 0)
        self.assertEqual(output, '')

    def test_names_deduplicates_the_bare_and_dotted_shapes_of_one_name(self):
        steps = 'Call `foo()` and `obj.foo()`.'

        result, output, _ = self.run_checker_argv(steps, 'pass\n', flags=['--names'])

        self.assertEqual(result, 1)
        self.assertEqual(output, 'foo\n')

    def test_names_sends_a_parse_error_to_stderr_and_still_lists_names(self):
        steps = 'Call `safeCall()`.'

        result, output, errors = self.run_checker_argv(
            steps, 'def broken(:\n', flags=['--names'])

        self.assertEqual(result, 1)
        self.assertEqual(output, 'safeCall\n')
        self.assertIn('check_step_calls: candidate is not valid Python:', errors)

    def test_json_green_case(self):
        steps = 'Call `safeCall()`.'

        result, output, _ = self.run_checker_argv(steps, 'safeCall()\n', flags=['--json'])
        report = json.loads(output)

        self.assertEqual(result, 0)
        self.assertTrue(report['ok'])
        self.assertEqual(report['named_calls'], 1)
        self.assertEqual(report['missing'], [])
        self.assertEqual(report['stubs'], [])
        self.assertEqual(report['shared_point'], [])
        self.assertIsNone(report['parse_error'])

    def test_json_reports_every_problem_category(self):
        steps = 'Call `sketch.missingCall(entity)`.'
        candidate = (
            '# missingCall(entity)\n'
            'circles.addByCenterRadius(center.geometry, radius)\n'
            '# TODO wire the rest\n')

        result, output, _ = self.run_checker_argv(steps, candidate, flags=['--json'])
        report = json.loads(output)

        self.assertEqual(result, 1)
        self.assertFalse(report['ok'])
        self.assertEqual(report['named_calls'], 1)
        self.assertEqual(
            report['missing'],
            [{'name': 'missingCall', 'has_receiver': True, 'textual_match': True}])
        self.assertEqual(
            report['stubs'],
            [{'line': 3, 'marker': 'TODO', 'text': '# TODO wire the rest'}])
        self.assertEqual(
            report['shared_point'], [{'line': 2, 'argument': 'center.geometry'}])
        self.assertIsNone(report['parse_error'])

    def test_json_reports_a_syntax_error_and_counts_every_call_missing(self):
        steps = 'Call `safeCall()` and `sketch.otherCall(entity)`.'

        result, output, _ = self.run_checker_argv(steps, 'def broken(:\n', flags=['--json'])
        report = json.loads(output)

        self.assertEqual(result, 1)
        self.assertFalse(report['ok'])
        self.assertIsInstance(report['parse_error'], str)
        self.assertTrue(report['parse_error'])
        self.assertEqual(report['named_calls'], 2)
        self.assertEqual(
            sorted(record['name'] for record in report['missing']),
            ['otherCall', 'safeCall'])

    def test_both_output_modes_at_once_is_a_usage_error(self):
        result, output, errors = self.run_checker_argv(
            'Call `safeCall()`.', 'safeCall()\n', flags=['--names', '--json'])

        self.assertEqual(result, 2)
        self.assertEqual(output, '')
        self.assertIn('usage: check_step_calls.py', errors)

    def test_unknown_flag_is_a_usage_error(self):
        result, output, errors = self.run_checker_argv(
            'Call `safeCall()`.', 'safeCall()\n', flags=['--bogus'])

        self.assertEqual(result, 2)
        self.assertEqual(output, '')
        self.assertIn('usage: check_step_calls.py', errors)

    def test_a_flag_is_position_independent(self):
        steps = 'Call `helperCall(value)`.'

        trailing = self.run_checker_argv(steps, 'pass\n', flags=['--names'])
        leading = self.run_checker_argv(steps, 'pass\n', flags=['--names'], flags_first=True)

        self.assertEqual(trailing, (1, 'helperCall\n', ''))
        self.assertEqual(leading, trailing)


class CheckApiCallsTest(unittest.TestCase):
    def run_checker(self, candidate, framework, framework_name='helpers.py'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate_path = root / 'candidate.py'
            framework_path = root / 'framework'
            framework_path.mkdir()
            (framework_path / framework_name).write_text(framework)
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

    def test_target_helper_does_not_allow_unrelated_receiver(self):
        result, output = self.run_checker(
            'class Candidate:\n'
            '    def totallyBogus(self, entity):\n'
            '        return entity\n'
            '\n'
            'def build(sketch, entity):\n'
            '    return sketch.totallyBogus(entity)\n',
            '')

        self.assertEqual(result, 1)
        self.assertIn("calls 'totallyBogus('", output)

    def test_target_method_on_typed_local_instance_is_allowed(self):
        result, output = self.run_checker(
            'class Candidate:\n'
            '    def helper(self):\n'
            '        return None\n'
            '\n'
            'def build():\n'
            '    candidate = Candidate()\n'
            '    return candidate.helper()\n',
            '')

        self.assertEqual(result, 0, output)

    def test_project_on_unrelated_receiver_is_blocking(self):
        result, output = self.run_checker(
            'def build(other, entity):\n'
            '    return other.project(entity)\n',
            '')

        self.assertEqual(result, 1)
        self.assertIn("calls 'project('", output)

    def test_fillet_method_on_unrelated_receiver_is_blocking(self):
        result, output = self.run_checker(
            'def build(other, edges, radius):\n'
            '    return other.addConstantRadiusEdgeSet(edges, radius, False)\n',
            '')

        self.assertEqual(result, 1)
        self.assertIn("calls 'addConstantRadiusEdgeSet('", output)

    def test_framework_method_on_unrelated_target_receiver_is_blocking(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            framework_path = root / 'framework'
            framework_path.mkdir()
            (framework_path / 'base.py').write_text(
                'class Generator:\n'
                '    def getParameterValue(self, name):\n'
                '        return name\n')
            (framework_path / 'spurgear.py').write_text(
                'class SpurGearInvoluteToothDesignGenerator:\n'
                '    def getParameterValue(self, name):\n'
                '        return name\n')
            candidate_path = root / 'candidate.py'
            candidate_path.write_text(
                'class Candidate:\n'
                '    def run(self):\n'
                "        return self.getParameterValue('x')\n")
            output = io.StringIO()
            with mock.patch.object(
                    API_CHECKER.fusion_api, 'lookup_many',
                    side_effect=lambda names: {name: [] for name in names}), \
                    mock.patch.object(API_CHECKER.fusion_api, 'similar', return_value=[]), \
                    mock.patch.object(API_CHECKER.fusion_api, 'query_script',
                                      return_value='/hermetic/fusion-query-api.py'), \
                    contextlib.redirect_stdout(output):
                with mock.patch.object(sys, 'argv', [
                        'check_api_calls.py', str(candidate_path),
                        '--framework', str(framework_path)]):
                    result = API_CHECKER.main()

        self.assertEqual(result, 1)
        self.assertIn("calls 'getParameterValue('", output.getvalue())

    def test_method_on_verified_framework_object_is_allowed(self):
        result, output = self.run_checker(
            'class Candidate:\n'
            '    def run(self):\n'
            '        parent = Generator()\n'
            "        return parent.getParameterValue('x')\n",
            'class Generator:\n'
            '    def getParameterValue(self, name):\n'
            '        return name\n',
            framework_name='base.py')

        self.assertEqual(result, 0, output)

    def test_method_on_constructor_verified_field_is_allowed(self):
        result, output = self.run_checker(
            'class Candidate:\n'
            '    def __init__(self, parent):\n'
            '        self.parent = parent\n'
            '    def run(self):\n'
            "        return self.parent.getParameterValue('x')\n"
            '\n'
            'def build():\n'
            '    return Candidate(Generator()).run()\n',
            'class Generator:\n'
            '    def getParameterValue(self, name):\n'
            '        return name\n',
            framework_name='base.py')

        self.assertEqual(result, 0, output)


class SpurDimensionContractTest(unittest.TestCase):
    def test_rib_and_midpoint_chain_dimensions_use_axis_orientations(self):
        source = (Path(__file__).parents[3] / 'lib' / 'geargen' / 'spurgear.py').read_text()
        ribs = source.split('        # 9.8 Ribs [SPUR-F-RIBS]', 1)[1].split(
            '        # 9.9 Close the tooth at the root', 1)[0]

        self.assertNotIn('AlignedDimensionOrientation', ribs)
        self.assertIn('VerticalDimensionOrientation', ribs)
        self.assertIn('HorizontalDimensionOrientation', ribs)
        self.assertIn('acrossVertical', ribs)

    def test_spine_uses_pinned_reference_for_zero_angle(self):
        source = (Path(__file__).parents[3] / 'lib' / 'geargen' / 'spurgear.py').read_text()
        spine = source.split('        # 9.7 Spine and +X reference', 1)[1].split(
            '        # 9.8 Ribs', 1)[0]

        self.assertIn('refEnd = sketch.sketchPoints.add', spine)
        self.assertIn('HorizontalDimensionOrientation', spine)
        self.assertIn('VerticalDimensionOrientation', spine)
        self.assertIn('addAngularDimension(\n            refLine, spineLine', spine)
        self.assertNotIn('addHorizontal', spine)


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
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
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
                    '<!-- proof-run: proofkit.Run(profileCases, stepOne) -->\n\n'
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

    def test_all_declared_compile_inputs_are_required(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for path in self.SOURCE_PATHS:
                (root / path).parent.mkdir(parents=True, exist_ok=True)
                (root / path).write_text('source\nline two\nline three\n')
            provenance = self.provenance_rows(root, self.SOURCE_PATHS[:-1])
        result, output = self.run_checker(provenance)

        self.assertEqual(result, 1)
        self.assertIn('provenance omits required source', output)

    def test_existing_auxiliary_spec_documents_are_provenance_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            instructions = root / 'spec' / 'gear' / 'instructions.md'
            auxiliary = root / 'spec' / 'gear' / 'trace.md'
            playbook = root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md'
            instructions.parent.mkdir(parents=True)
            playbook.parent.mkdir(parents=True)
            instructions.write_text('See `trace.md` for the derivation.\n')
            auxiliary.write_text('trace\n')
            playbook.write_text('playbook\n')
            prior = os.getcwd()
            try:
                os.chdir(root)
                inputs = COMPILE_CHECKER.provenance_inputs('gear')
            finally:
                os.chdir(prior)

        self.assertEqual(
            inputs,
            {
                'spec/gear/instructions.md',
                'spec/gear/trace.md',
                '.claude/skills/generate-gear/PLAYBOOK.md',
            })

    def test_absent_fusion_sidecar_is_optional(self):
        result, output = self.run_checker(include_fusion=False)

        self.assertEqual(result, 0, output)
        self.assertNotIn('spec/gear/fusion.md', output)

    def test_referenced_auxiliary_drift_is_blocking(self):
        result, output = self.run_checker(auxiliary=True, mutate_auxiliary=True)

        self.assertEqual(result, 1)
        self.assertIn('spec/gear/trace.md has changed since the step list was compiled', output)

    def test_each_step_requires_a_from_citation(self):
        result, output = self.run_checker(from_line='')

        self.assertEqual(result, 1)
        self.assertIn('has no nonempty **From:** citation', output)

    def test_prose_only_from_block_is_not_a_citation(self):
        result, output = self.run_checker(from_line='**From:** the gear instructions prose')

        self.assertEqual(result, 1)
        self.assertIn('has no parseable **From:** file-and-line citation', output)

    def test_citation_line_zero_is_rejected(self):
        result, output = self.run_checker(from_line='**From:** `spec/gear/instructions.md` L0')

        self.assertEqual(result, 1)
        self.assertIn('line numbers start at 1', output)

    def test_citation_reversed_range_is_rejected(self):
        result, output = self.run_checker(from_line='**From:** `spec/gear/instructions.md` L3–2')

        self.assertEqual(result, 1)
        self.assertIn('first line is after the last line', output)

    def test_citation_out_of_range_is_rejected(self):
        result, output = self.run_checker(from_line='**From:** `spec/gear/instructions.md` L1–4')

        self.assertEqual(result, 1)
        self.assertIn('but that file has 3 lines', output)

    def test_current_format_citation_is_valid(self):
        result, output = self.run_checker(
            from_line='**From:** `spec/gear/instructions.md` L1-2,\n'
            'L3; `spec/gear/fusion.md` L1–2')

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_registered_proofkit_step_is_accepted(self):
        result, output = self.run_checker()

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_registration_in_non_test_go_file_is_rejected(self):
        result, output = self.run_checker(proof_filename='proof.go')

        self.assertEqual(result, 1)
        self.assertIn('registers stepOne, but `go test` only runs tests in a _test.go file',
                      output)

    def test_claimed_but_unregistered_proof_function_fails(self):
        """A step whose Test does not exist fails, and a quoted registration does not save it."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'var sample = "proofkit.Run(t, profileCases, stepUnused)"\n\n'
            '// proofkit.Run(t, profileCases, stepUnused)\n\n'
            'func stepOne() {}\n'
            'func stepUnused() {}\n')
        step_body = (
            '## S1 `[GO]` One — `stepOne`\n\n'
            'Build the first thing.\n\n'
            '**From:** `spec/gear/instructions.md` L1\n\n'
            '## S2 `[GO]` Unused — `stepUnused`\n\n'
            'Build the second thing.\n\n'
            '**From:** `spec/gear/instructions.md` L1\n\n')

        result, output = self.run_checker(proof_body=proof_body, step_body=step_body)

        self.assertEqual(result, 1)
        self.assertIn('S2 names proof function stepUnused, which TestUnused does not build with',
                      output)

    def test_early_returned_proof_registration_is_rejected(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tif t != nil {\n'
            '\t\treturn\n'
            '\t}\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:5 runs a proof outside the shape this gate reads',
                      output)
        self.assertIn('S1 names proof function stepOne, which TestOne does not build with', output)

    def test_false_branch_proof_registration_is_rejected(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tif false {\n'
            '\t\tproofkit.Run(t, profileCases, stepOne)\n'
            '\t}\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:3 runs a proof outside the shape this gate reads',
                      output)
        self.assertIn('S1 names proof function stepOne, which TestOne does not build with', output)

    def test_short_dotted_calls_are_compile_candidates(self):
        steps = 'Call `sketch.add()`, `sketch.set()`, and `safeCall()`.'

        self.assertEqual(
            COMPILE_CHECKER.named_calls(steps),
            {'add', 'set', 'safeCall'})

    def test_call_parser_preserves_bare_and_dotted_receivers(self):
        self.assertEqual(
            COMPILE_CHECKER.call_shapes(
                'sketch.project(entity), sketch.sketchTexts.createInput2(text, height), unknown()'),
            {
                ('project', 'sketch'),
                ('createInput2', 'sketch.sketchTexts'),
                ('unknown', None),
            })

    def test_compile_watchlist_matches_only_the_declared_receiver(self):
        valid = (
            '`sketch.project(entity)` `Sketch.project(entity)` '
            '`sketch.sketchTexts.createInput2(text, height)`')
        invalid = '`SketchPoint.project(entity)` `chamferFeatures.createInput2(text, height)`'

        self.assertEqual(
            COMPILE_CHECKER.watched_calls(valid, 'steps.md'),
            {'project': 'steps.md:1', 'createInput2': 'steps.md:1'})
        self.assertEqual(
            COMPILE_CHECKER.watched_calls(invalid, 'steps.md'), {})

    def test_compile_watchlist_reads_a_qualified_sketch_receiver(self):
        # A step list carries no types, so a receiver is judged by the class its own name
        # denotes. `other.sketch` denotes a Sketch however it was reached. The type-aware
        # checkers still gate this shape; only the step list treats it as the watched call.
        self.assertEqual(
            COMPILE_CHECKER.watched_calls(
                'Call `other.sketch.project(entity)`.', 'steps.md'),
            {'project': 'steps.md:1'})

    def test_compile_watchlist_accepts_verified_receiver_shape(self):
        self.assertEqual(
            COMPILE_CHECKER.watched_calls(
                'Call `self.sketch.project(entity)`.', 'steps.md'),
            {'project': 'steps.md:1'})

    def test_compile_queries_and_rejects_wrong_watchlist_receiver(self):
        result, output = self.run_checker(
            step_body=(
                '## S1 `[PROSE]` Invalid call — `stepOne`\n\n'
                'Call `SketchPoint.project(entity)`.\n\n'
                '**From:** `spec/gear/instructions.md` L1\n\n'),
            api_lookup={'project': []})

        self.assertEqual(result, 1)
        self.assertIn("names 'project(', which the Fusion API database does not have", output)

    def test_compile_allows_legitimate_unwatched_namesake_from_api(self):
        result, output = self.run_checker(
            step_body=(
                # The step is `[GO]` and carries its annotation because the default proof body
                # registers `stepOne`, and a registration no step annotates is BLOCKING. The
                # subject here is the API-reality check, so the fixture has to be clean elsewhere.
                '## S1 `[GO]` Chamfer call — `stepOne`\n\n'
                'Call `chamferFeatures.createInput2(edges)`.\n\n'
                '<!-- proof-run: proofkit.Run(profileCases, stepOne) -->\n\n'
                '**From:** `spec/gear/instructions.md` L1\n\n'),
            api_lookup={
                'createInput2': [('adsk.fusion.ChamferFeatures.createInput2', 'method')]})

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_compile_rejects_same_name_from_wrong_watchlist_owner(self):
        result, output = self.run_checker(
            step_body=(
                '## S1 `[PROSE]` Invalid receiver — `stepOne`\n\n'
                'Call `SketchPoint.createInput2(edges)`.\n\n'
                '**From:** `spec/gear/instructions.md` L1\n\n'),
            api_lookup={
                'createInput2': [('adsk.fusion.ChamferFeatures.createInput2', 'method')]})

        self.assertEqual(result, 1)
        self.assertIn("on receiver 'SketchPoint'", output)
        self.assertIn('database declares it on ChamferFeatures', output)

    def test_compiled_proof_summary_requires_a_tracked_path(self):
        result, output = self.run_checker(
            step_body=(
                '## S1 `[GO]` One — `stepOne`\n\n'
                'The proof in `.tmp/step_test.go` exercises this step.\n\n'
                '**From:** `spec/gear/instructions.md` L1\n\n'))

        self.assertEqual(result, 1)
        self.assertIn('proof path .tmp/step_test.go does not exist', output)

    def test_committed_proof_summary_path_is_accepted(self):
        root = Path(__file__).parents[3]
        prior = os.getcwd()
        try:
            os.chdir(root)
            self.assertEqual(
                COMPILE_CHECKER.proof_paths(
                    '`[GO]` marks the proof in `proof/spurgear/solids_test.go`.'),
                ['proof/spurgear/solids_test.go'])
            self.assertTrue(
                COMPILE_CHECKER.proof_path_is_tracked_or_committed(
                    'proof/spurgear/solids_test.go'))
        finally:
            os.chdir(prior)


class WorkflowGateWiringTest(unittest.TestCase):
    def test_workflow_runs_all_generated_candidate_gates(self):
        root = Path(__file__).parents[3]
        workflow = (root / '.github' / 'workflows' / '3d-proof.yml').read_text()

        for gate in (
                'check_compile.py spurgear',
                'pyright_check.py .tmp/spurgear.generated.py',
                'check_novel_types.py .tmp/spurgear.generated.py',
                'check_contract.py spec/spurgear/contract.json',
                'check_input_read.py .tmp/spurgear.generated.py',
                'check_anchors.py',
                'check_step_calls.py spec/spurgear/steps.md .tmp/spurgear.generated.py',
                'check_api_calls.py .tmp/spurgear.generated.py'):
            self.assertIn(gate, workflow)

        # The checker regression tests run as one discovery pass, so adding a
        # test file needs no workflow edit and cannot be forgotten.
        self.assertIn(
            "unittest discover -s .claude/skills/generate-gear -p 'test_*.py'", workflow)

        self.assertIn('FUSION_QUERY_API', workflow)
        self.assertIn('FUSION_API_STUBS', workflow)
        self.assertIn('cp lib/geargen/spurgear.py .tmp/spurgear.generated.py', workflow)

        lookup_step = workflow.split('name: Check out the Fusion API lookup database', 1)[1]
        lookup_step = lookup_step.split('name:', 1)[0]
        self.assertRegex(lookup_step, r'(?m)^\s+ref: [0-9a-f]{40}$')


if __name__ == '__main__':
    unittest.main()
