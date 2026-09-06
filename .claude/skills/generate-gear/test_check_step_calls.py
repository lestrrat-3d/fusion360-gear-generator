#!/usr/bin/env python3
"""Regression tests for the step-call coverage gate."""
import contextlib
import importlib.util
import io
import json
import os
import re
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
    """Guards on the shipped tooth drawer.

    These read lib/geargen/spurgear.py, which is build output, so they anchor on code
    the recipe requires rather than on comment numbering or local names — both of which
    legitimately change when the gear is recompiled and re-emitted.
    """

    @staticmethod
    def drawer_source():
        # The whole tooth-generator class, not one method: the emit is free to split
        # drawTooth into helpers, and it has.
        source = (Path(__file__).parents[3] / 'lib' / 'geargen' / 'spurgear.py').read_text()
        start = source.index('class SpurGearInvoluteToothDesignGenerator')
        end = source.find('\nclass ', start + 1)
        return source[start:end if end != -1 else len(source)]

    def test_rib_and_midpoint_chain_dimensions_use_axis_orientations(self):
        ribs = self.drawer_source()

        # [SPUR-F-RIBS]: the rib takes the axis across the spine and the chain takes the
        # axis along it, so both axis orientations appear and the aligned one never does.
        self.assertNotIn('AlignedDimensionOrientation', ribs)
        self.assertIn('VerticalDimensionOrientation', ribs)
        self.assertIn('HorizontalDimensionOrientation', ribs)
        self.assertRegex(ribs, r'addMidPoint\(|MidPoint')

    def test_spine_uses_pinned_reference_for_zero_angle(self):
        spine = self.drawer_source()

        # [SPUR-F-SPINE]: the far reference endpoint is pinned by two axis dimensions,
        # and the signed angular dimension is what forbids the mirrored answer. A plain
        # addHorizontal carries no direction and must not stand in for it.
        self.assertIn('HorizontalDimensionOrientation', spine)
        self.assertIn('VerticalDimensionOrientation', spine)
        self.assertIn('addAngularDimension(', spine)
        self.assertNotIn('addHorizontal(', spine)

    def test_toothtop_arc_centre_is_tied_to_the_local_origin(self):
        drawer = self.drawer_source()

        # [SPUR-F-TOOTHTOP-ARC]: addByCenterStartEnd COPIES the centre point it is
        # handed, so the arc needs an explicit coincident or its centre is free. Without
        # it the centre strands behind when the sketch is dragged onto the anchor, which
        # collapsed a bevel pinion's tooth-top arc to 0.5743 mm against a 22.5 mm tip
        # radius in Fusion on 2026-09-02.
        self.assertIn('addByCenterStartEnd(', drawer)
        self.assertRegex(drawer, r'addCoincident\([^)]*centerSketchPoint')


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
    """CI must gate every compiled gear with the whole battery.

    The workflow used to spell each gate out as its own command line for spurgear alone, and
    this test pinned those literal strings so a gate could not be quietly dropped. The battery
    now runs through run_gates.py under a per-gear matrix, so the same guard is expressed in
    two halves: the workflow has to invoke run_gates.py for each gear, and run_gates.py has to
    schedule every gate script the old list named. Neither half alone is enough — a workflow
    that calls the runner proves nothing if the runner stopped running a gate.
    """

    ROOT = Path(__file__).parents[3]

    def setUp(self):
        self.workflow = (self.ROOT / '.github' / 'workflows' / '3d-proof.yml').read_text()
        self.matrix_gears = re.findall(r'(?m)^\s*- gear: (\S+)$', self.workflow)

    def test_runner_still_schedules_every_candidate_gate(self):
        runner_path = Path(__file__).with_name('run_gates.py')
        spec = importlib.util.spec_from_file_location('run_gates', runner_path)
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)

        for script in ('check_input_read.py', 'check_contract.py', 'check_step_calls.py',
                       'check_anchors.py', 'check_api_calls.py', 'pyright_check.py',
                       'check_novel_types.py'):
            self.assertIn(script, runner.GATE_SCRIPTS.values())
        # parse has no script of its own; it is run in-process, so assert its key instead.
        self.assertIn('parse', runner.GATE_ORDER)
        self.assertEqual(set(runner.GATE_ORDER), set(runner.GATE_TITLES))

    def test_workflow_gates_every_matrix_gear(self):
        self.assertTrue(self.matrix_gears, 'the gates matrix names no gear')

        self.assertIn('check_compile.py ${{ matrix.gear }}', self.workflow)
        self.assertIn('run_gates.py ${{ matrix.gear }}', self.workflow)
        self.assertIn('--gate-novel-types', self.workflow)
        self.assertIn(
            'cp lib/geargen/${{ matrix.gear }}.py .tmp/${{ matrix.gear }}.generated.py',
            self.workflow)

    def test_every_compiled_gear_is_in_the_matrix(self):
        """A gear with a step list is a gear CI can gate, so it must be listed.

        This is the check that keeps coverage from silently lagging the repo: compiling a new
        gear produces spec/<gear>/steps.md, and without this the gear would sit ungated until
        somebody noticed. helicalgear sat ungated for exactly that reason.
        """
        compiled = sorted(path.parent.name
                          for path in (self.ROOT / 'spec').glob('*/steps.md'))
        self.assertEqual(sorted(self.matrix_gears), compiled)

    def test_gear_with_a_contract_manifest_requires_it(self):
        """--require-contract turns a deleted manifest into a failure, not a silent skip."""
        for gear in self.matrix_gears:
            has_manifest = (self.ROOT / 'spec' / gear / 'contract.json').exists()
            leg = self.workflow.split('- gear: %s\n' % gear, 1)[1].split('- gear:', 1)[0]
            self.assertEqual(
                "require_contract: '--require-contract'" in leg, has_manifest,
                '%s: require_contract must be set exactly when spec/%s/contract.json exists'
                % (gear, gear))

    def test_workflow_keeps_its_shared_wiring(self):
        # The checker regression tests run as one discovery pass, so adding a
        # test file needs no workflow edit and cannot be forgotten.
        self.assertIn(
            "unittest discover -s .claude/skills/generate-gear -p 'test_*.py'", self.workflow)

        self.assertIn('FUSION_QUERY_API', self.workflow)
        self.assertIn('FUSION_API_STUBS', self.workflow)

        # The Fusion API database is still pinned in the workflow itself, because
        # nothing outside CI checks it out.
        self.assertRegex(self.workflow, r'(?m)^\s+FUSION_API_COMMIT: [0-9a-f]{40}$')

    def test_engine_revisions_are_pinned_only_in_go_mod(self):
        """proof/go.mod is the one place the engine revisions are written.

        They used to be written in the workflow as well, which meant a local
        `proof/run.sh` ran against whatever engine checkout happened to sit beside
        the repo. A proof could then pass locally against a newer engine and fail
        CI against the pin, which happened twice. go.mod already pins both in the
        pseudo-version Go records for an untagged module, so a second copy could
        only drift from it.
        """
        gomod = (self.ROOT / 'proof' / 'go.mod').read_text()
        for module in ('sketch', 'decad'):
            self.assertRegex(
                gomod, r'(?m)^\s*github\.com/lestrrat-3d/%s v\S*-[0-9a-f]{12}$' % module,
                '%s must be pinned by a pseudo-version in proof/go.mod' % module)

        # And nowhere else: no SHA for either engine may appear in the workflow.
        for commit in ('SKETCH_COMMIT', 'DECAD_COMMIT'):
            self.assertNotRegex(
                self.workflow, r'(?m)^\s+%s: [0-9a-f]{12,40}$' % commit,
                '%s belongs only in proof/go.mod' % commit)

        # CI has to read go.mod before it checks an engine out, or the refs it
        # resolves are empty.
        read_at = self.workflow.find('proof/go.mod pins')
        sketch_at = self.workflow.find('repository: lestrrat-3d/sketch')
        self.assertNotEqual(read_at, -1, 'the workflow must read the pins out of proof/go.mod')
        self.assertNotEqual(sketch_at, -1)
        self.assertLess(read_at, sketch_at,
                        'the workflow must read the pins before checking out an engine')


class WorkflowProofCacheTest(unittest.TestCase):
    """The proof job's cache must be able to go stale and then refresh.

    A cache entry is never rewritten: actions/cache skips its save whenever the restore hit the
    primary key. A key that stays the same while its contents go stale is therefore not a cold
    cache, it is a permanently wrong one — the affected package re-executes on every run and
    nothing is ever saved. setup-go's built-in cache keys on the dependency files alone and
    passes no restore-keys, so it has exactly that hole for a proof source edit, and it cannot
    be closed by widening its key without also losing the fallback.
    """

    ROOT = Path(__file__).parents[3]

    def setUp(self):
        workflow = (self.ROOT / '.github' / 'workflows' / '3d-proof.yml').read_text()
        # Comments discuss the keys by name; a comment caches nothing.
        self.lines = [line for line in workflow.splitlines()
                      if not line.lstrip().startswith('#')]
        self.commands = '\n'.join(self.lines)

    def cache_step(self, marker):
        """The key line and the restore-key lines of the cache step whose key holds marker."""
        for index, line in enumerate(self.lines):
            if 'key:' not in line or marker not in line:
                continue
            restores = []
            for follow in self.lines[index + 1:]:
                if follow.strip() == 'restore-keys: |':
                    continue
                if not follow.startswith('            proof-'):
                    break
                restores.append(follow.strip())
            return line, restores
        return None, []

    def test_setup_go_does_no_caching_of_its_own(self):
        """setup-go's key cannot carry the proof source and keep a fallback, so it owns neither.

        cache-dependency-path decides its whole key, and its restore is a bare primary-key
        lookup. Listing the sources there would turn every source edit into a total miss that
        re-downloads the module cache; leaving them out is the staleness this class exists for.
        """
        self.assertNotIn('cache-dependency-path', self.commands,
                         'a job still lets setup-go derive its own cache key')

    def test_the_build_cache_key_carries_the_proof_source(self):
        key, _ = self.cache_step('proof-build-')
        self.assertIsNotNone(key, 'the proof job declares no build and test cache')
        self.assertIn("hashFiles('gears/proof/**/*.go')", key,
                      'the build cache key ignores the proof source, so it can never refresh')

    def test_the_build_cache_falls_back_to_its_older_key_shapes(self):
        """Without the fallbacks, adding the source component costs a full rebuild every edit."""
        module_hash = "${{ hashFiles('gears/proof/go.sum', 'gears/proof/go.mod') }}"
        _, restores = self.cache_step('proof-build-')
        self.assertEqual(2, len(restores),
                         'the build cache declares %d fallbacks, want 2' % len(restores))
        self.assertTrue(restores[0].endswith(module_hash + '-'),
                        'the build cache has no same-module-graph fallback')
        self.assertTrue(restores[1].endswith('${{ runner.os }}-'),
                        'the build cache drops the pre-source key shape')
        self.assertTrue(restores[0].startswith(restores[1]),
                        'the build cache fallbacks are not longest-prefix-first')

    def test_the_module_cache_is_kept_separate_from_the_proof_source(self):
        """The dependency files decide the module cache's content, and nothing else does.

        Rolling it on a source edit would re-download every module for a change that cannot
        affect one.
        """
        key, restores = self.cache_step('proof-mod-')
        self.assertIsNotNone(key, 'the proof job declares no module cache')
        self.assertNotIn("gears/proof/**/*.go", key)
        self.assertEqual([], restores, 'the module cache needs no fallback: its key is exact')
        self.assertIn('path: ~/go/pkg/mod', self.commands)
        self.assertIn('path: ~/.cache/go-build', self.commands)


if __name__ == '__main__':
    unittest.main()
