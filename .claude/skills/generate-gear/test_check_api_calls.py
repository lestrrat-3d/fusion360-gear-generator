#!/usr/bin/env python3
"""Focused regressions for receiver-owned Fusion API call validation."""
import contextlib
import importlib.util
import io
import sys
import tempfile
import textwrap
import unittest
import warnings
from pathlib import Path
from unittest import mock


CHECKER_PATH = Path(__file__).with_name('check_api_calls.py')
MODULE_SPEC = importlib.util.spec_from_file_location('check_api_calls', CHECKER_PATH)
CHECKER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(CHECKER)


def api_member(returns=None, kind='method', lookup=None):
    return {
        'lookup': lookup or 'adsk.fusion.Mock',
        'name': 'member',
        'kind': kind,
        'declared_on': lookup or 'adsk.fusion.Mock',
        'returns': returns,
    }


class CheckApiReceiverOwnershipTest(unittest.TestCase):
    def run_checker(self, candidate, api_names=(), members=None, framework_files=None):
        framework_files = framework_files or {'base.py': ''}
        members = members or {}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate_path = root / 'candidate.py'
            framework_path = root / 'framework'
            framework_path.mkdir()
            candidate_path.write_text(textwrap.dedent(candidate))
            for name, content in framework_files.items():
                (framework_path / name).write_text(textwrap.dedent(content))

            def lookup_many(names):
                return {
                    name: [('adsk.fusion.Mock.%s' % name, 'method')] if name in api_names else []
                    for name in names
                }

            def member_info(cls, name):
                return members.get((CHECKER.normalize_api_type(cls), name))

            output = io.StringIO()
            with mock.patch.object(CHECKER.fusion_api, 'lookup_many',
                                   side_effect=lookup_many), \
                    mock.patch.object(CHECKER.fusion_api, 'member_info',
                                      side_effect=member_info), \
                    mock.patch.object(CHECKER.fusion_api, 'similar', return_value=[]), \
                    mock.patch.object(CHECKER.fusion_api, 'query_script',
                                      return_value='/hermetic/fusion-query-api.py'), \
                    mock.patch.object(CHECKER.fusion_api, 'unverified_findings',
                                      return_value=['reported unverified call']), \
                    contextlib.redirect_stdout(output):
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', ResourceWarning)
                    with mock.patch.object(sys, 'argv', [
                            'check_api_calls.py', str(candidate_path),
                            '--framework', str(framework_path)]):
                        result = CHECKER.main()
            return result, output.getvalue()

    def test_fusion_method_on_wrong_inferred_receiver_is_blocking(self):
        result, output = self.run_checker(
            """
            def build(sketch, center):
                return sketch.sketchCircles.addByCenterRadius(center, 1)
            """,
            api_names={'addByCenterRadius'},
            members={
                ('Sketch', 'sketchCurves'): api_member('SketchCurves', kind='property'),
                ('SketchCurves', 'sketchCircles'): api_member('SketchCircles', kind='property'),
                ('SketchCircles', 'addByCenterRadius'): api_member('SketchCircle'),
            })

        self.assertEqual(result, 1)
        self.assertIn("calls 'addByCenterRadius('", output)
        self.assertIn('receiver ownership is required', output)

    def test_unrelated_local_helper_does_not_mask_invalid_fusion_call(self):
        result, output = self.run_checker(
            """
            class Helper:
                def addByCenterRadius(self, center, radius):
                    return None

            def build(sketch, center):
                helper = Helper()
                helper.addByCenterRadius(center, 1)
                return sketch.sketchCircles.addByCenterRadius(center, 1)
            """,
            api_names={'addByCenterRadius'},
            members={
                ('Sketch', 'sketchCurves'): api_member('SketchCurves', kind='property'),
                ('SketchCurves', 'sketchCircles'): api_member('SketchCircles', kind='property'),
                ('SketchCircles', 'addByCenterRadius'): api_member('SketchCircle'),
            })

        self.assertEqual(result, 1)
        self.assertEqual(output.count("calls 'addByCenterRadius('"), 1)

    def test_self_method_found_only_on_imported_gear_class_is_blocking(self):
        result, output = self.run_checker(
            """
            from .spurgear import SpurGearInvoluteToothDesignGenerator

            class Candidate:
                def run(self):
                    return self.getParameterValue('x')
            """,
            framework_files={
                'base.py': '',
                'spurgear.py': (
                    "class SpurGearInvoluteToothDesignGenerator:\n"
                    "    def getParameterValue(self, name):\n"
                    "        return name\n"
                ),
            })

        self.assertEqual(result, 1)
        self.assertIn("calls 'getParameterValue('", output)

    def test_valid_fusion_chain_and_exact_unverified_receiver_are_allowed(self):
        result, output = self.run_checker(
            """
            def build(sketch: adsk.fusion.Sketch, center):
                circles = sketch.sketchCurves.sketchCircles
                circle = circles.addByCenterRadius(center, 1)
                projected = sketch.project(center)
                return projected.item(0), circle
            """,
            api_names={'addByCenterRadius', 'item'},
            members={
                ('Sketch', 'sketchCurves'): api_member('SketchCurves', kind='property'),
                ('SketchCurves', 'sketchCircles'): api_member('SketchCircles', kind='property'),
                ('SketchCircles', 'addByCenterRadius'): api_member('SketchCircle'),
                ('ObjectCollection', 'item'): api_member('Base'),
            })

        self.assertEqual(result, 0, output)
        self.assertIn('api-call check: OK', output)
        self.assertIn('UNVERIFIED call', output)

    def test_identifier_ending_in_sketch_does_not_bind_a_receiver(self):
        result, output = self.run_checker(
            """
            def build(not_a_sketch, center):
                return not_a_sketch.sketchCurves.sketchCircles.addByCenterRadius(center, 1)
            """,
            api_names={'addByCenterRadius'},
            members={
                ('Sketch', 'sketchCurves'): api_member('SketchCurves', kind='property'),
                ('SketchCurves', 'sketchCircles'): api_member('SketchCircles', kind='property'),
                ('SketchCircles', 'addByCenterRadius'): api_member('SketchCircle'),
            })

        self.assertEqual(result, 1)
        self.assertIn("calls 'addByCenterRadius('", output)

    def test_reassignment_removes_prior_fusion_receiver_type(self):
        result, output = self.run_checker(
            """
            def build(sketch: adsk.fusion.Sketch, bad, center):
                sketch = bad
                return sketch.sketchCurves.sketchCircles.addByCenterRadius(center, 1)
            """,
            api_names={'addByCenterRadius'},
            members={
                ('Sketch', 'sketchCurves'): api_member('SketchCurves', kind='property'),
                ('SketchCurves', 'sketchCircles'): api_member('SketchCircles', kind='property'),
                ('SketchCircles', 'addByCenterRadius'): api_member('SketchCircle'),
            })

        self.assertEqual(result, 1)
        self.assertIn('receiver ownership is required', output)

    def test_local_class_named_sketch_does_not_authorize_unverified_call(self):
        result, output = self.run_checker(
            """
            class Sketch:
                pass

            def build(sketch: Sketch, entity):
                return sketch.project(entity)
            """,
            api_names={'project'})

        self.assertEqual(result, 1)
        self.assertIn('receiver ownership is required', output)

    def test_local_sketch_method_does_not_authorize_fusion_sketch_receiver(self):
        result, output = self.run_checker(
            """
            class Sketch:
                def notAnApi(self):
                    return None

            def build(sketch: adsk.fusion.Sketch):
                return sketch.notAnApi()
            """,
            api_names={'notAnApi'})

        self.assertEqual(result, 1)
        self.assertIn("calls 'notAnApi('", output)
        self.assertIn('receiver ownership is required', output)

    def test_untyped_watchlist_aliases_require_verified_bindings(self):
        candidates = (
            ('sketch', 'project(entity)'),
            ('toolsSketch', 'project(entity)'),
            ('Sketch', 'project(entity)'),
            ('sketchTexts', 'createInput2(text, 1)'),
            ('SketchTexts', 'createInput2(text, 1)'),
            ('filletInput', 'addConstantRadiusEdgeSet(edges, value, False)'),
            ('FilletFeatureInput', 'addConstantRadiusEdgeSet(edges, value, False)'),
        )
        for receiver, call in candidates:
            with self.subTest(receiver=receiver):
                result, output = self.run_checker(
                    "def build(%s, entity, text, edges, value):\n"
                    "    return %s.%s\n" % (receiver, receiver, call),
                    api_names={'project', 'createInput2', 'addConstantRadiusEdgeSet'})

                self.assertEqual(result, 1)
                self.assertIn('receiver ownership is required', output)

    def test_unverified_namesake_without_verified_receiver_binding_is_blocking(self):
        result, output = self.run_checker(
            """
            def build(Sketch, entity):
                return Sketch.project(entity)
            """,
            api_names={'project'})

        self.assertEqual(result, 1)
        self.assertIn("calls 'project('", output)
        self.assertIn('receiver ownership is required', output)

    def test_prefixed_unverified_receiver_is_blocking(self):
        result, output = self.run_checker(
            """
            def build(other, entity):
                return other.sketch.project(entity)
            """,
            api_names={'project'})

        self.assertEqual(result, 1)
        self.assertIn("calls 'project('", output)
        self.assertIn('receiver ownership is required', output)

    def test_verified_self_fusion_field_allows_unverified_call(self):
        result, output = self.run_checker(
            """
            class Candidate:
                def __init__(self, sketch: adsk.fusion.Sketch):
                    self.sketch = sketch

                def run(self, entity):
                    return self.sketch.project(entity)
            """,
            api_names={'project'})

        self.assertEqual(result, 0, output)
        self.assertIn('api-call check: OK', output)
        self.assertIn('UNVERIFIED call', output)


if __name__ == '__main__':
    unittest.main()
