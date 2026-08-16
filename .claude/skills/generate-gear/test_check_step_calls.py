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

    def test_short_dotted_calls_are_required(self):
        steps = 'Call `sketch.add()` and `sketch.set()`.'

        self.assertEqual(
            CHECKER.named_call_shapes(steps),
            {('add', True), ('set', True)})
        result, output = self.run_checker(steps, 'sketch.add(None)\n')

        self.assertEqual(result, 1)
        self.assertIn("receiver.set('", output)


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
    def test_rib_and_midpoint_chain_dimensions_are_signed(self):
        source = (Path(__file__).parents[3] / 'lib' / 'geargen' / 'spurgear.py').read_text()
        ribs = source.split('        # 8. Ribs ([SPUR-F-RIBS])', 1)[1].split(
            '        # 9. Close the tooth at the root', 1)[0]

        self.assertNotIn('AlignedDimensionOrientation', ribs)
        self.assertIn('VerticalDimensionOrientation', ribs)
        self.assertIn('HorizontalDimensionOrientation', ribs)
        self.assertIn('ribOrientation', ribs)
        self.assertIn('chainOrientation', ribs)

    def test_spine_uses_pinned_reference_for_zero_angle(self):
        source = (Path(__file__).parents[3] / 'lib' / 'geargen' / 'spurgear.py').read_text()
        spine = source.split('        # 7. Spine + +X reference', 1)[1].split(
            '        # 8. Ribs', 1)[0]

        self.assertIn('referenceEnd = sketch.sketchPoints.add', spine)
        self.assertIn('HorizontalDimensionOrientation', spine)
        self.assertIn('VerticalDimensionOrientation', spine)
        self.assertIn('addAngularDimension(\n            reference, spine', spine)
        self.assertNotIn('addHorizontal(spine)', spine)


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

    def run_checker(self, provenance=None, from_line='**From:** `spec/gear/instructions.md` L1'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'spec' / 'gear').mkdir(parents=True)
            (root / 'proof' / 'gear').mkdir(parents=True)
            (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)
            for path in (
                    root / 'spec' / 'gear' / 'instructions.md',
                    root / 'spec' / 'gear' / 'fusion.md',
                    root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md'):
                path.write_text('source\nline two\nline three\n')
            if provenance is None:
                provenance = self.provenance_rows(root)
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

    def test_short_dotted_calls_are_compile_candidates(self):
        steps = 'Call `sketch.add()`, `sketch.set()`, and `safeCall()`.'

        self.assertEqual(
            COMPILE_CHECKER.named_calls(steps),
            {'add', 'set', 'safeCall'})


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

        self.assertIn('FUSION_QUERY_API', workflow)
        self.assertIn('FUSION_API_STUBS', workflow)
        self.assertIn('cp lib/geargen/spurgear.py .tmp/spurgear.generated.py', workflow)

        lookup_step = workflow.split('name: Check out the Fusion API lookup database', 1)[1]
        lookup_step = lookup_step.split('name:', 1)[0]
        self.assertRegex(lookup_step, r'(?m)^\s+ref: [0-9a-f]{40}$')


if __name__ == '__main__':
    unittest.main()
