#!/usr/bin/env python3
"""Focused regressions for receiver-owned Fusion API call validation."""
import contextlib
import importlib.util
import io
import subprocess
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
    def run_checker(self, candidate, api_names=(), members=None, framework_files=None,
                    member_failure=None):
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
                if member_failure is not None:
                    raise member_failure
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

    def test_unavailable_during_receiver_inference_is_a_setup_error(self):
        result, _output = self.run_checker(
            """
            def build(sketch: adsk.fusion.Sketch, center):
                return sketch.sketchCircles.addByCenterRadius(center, 1)
            """,
            member_failure=CHECKER.fusion_api.Unavailable('session startup failed'))

        self.assertEqual(result, 2)

    def test_help_does_not_require_api_script_discovery(self):
        with mock.patch.object(
                CHECKER.fusion_api, 'query_script',
                side_effect=CHECKER.fusion_api.Unavailable('missing query tool')), \
                mock.patch.object(sys, 'argv', ['check_api_calls.py', '--help']), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                CHECKER.main()

        self.assertEqual(raised.exception.code, 0)

    def test_query_free_check_does_not_require_api_script_discovery(self):
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / 'candidate.py'
            candidate.write_text('def build():\n    return 1\n')
            with mock.patch.object(
                    CHECKER.fusion_api, 'query_script',
                    side_effect=CHECKER.fusion_api.Unavailable('missing query tool')), \
                    mock.patch.object(sys, 'argv', ['check_api_calls.py', str(candidate)]), \
                    contextlib.redirect_stdout(io.StringIO()):
                result = CHECKER.main()

        self.assertEqual(result, 0)

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

    # `HerringboneGearGenerator` reaches `Generator.getComponent` only through two gear modules
    # that are neither the target nor shared framework. Before those modules' base classes and
    # return annotations were read, the walk stopped at the first of them and every call chained
    # off `self.getComponent()` was reported as having an unknown receiver.
    DERIVED_GEAR_FRAMEWORK = {
        'base.py': (
            "class Generator:\n"
            "    def getComponent(self) -> adsk.fusion.Component:\n"
            "        return self.component\n"
        ),
        'spurgear.py': (
            "from .base import Generator\n"
            "\n"
            "class SpurGearGenerator(Generator):\n"
            "    def buildTooth(self, ctx):\n"
            "        pass\n"
        ),
        'helicalgear.py': (
            "from .spurgear import SpurGearGenerator\n"
            "\n"
            "class HelicalGearGenerator(SpurGearGenerator):\n"
            "    def loftTooth(self, ctx):\n"
            "        pass\n"
        ),
    }

    DERIVED_GEAR_MEMBERS = {
        ('Component', 'features'): api_member('Features', kind='property'),
        ('Features', 'mirrorFeatures'): api_member('MirrorFeatures', kind='property'),
        ('MirrorFeatures', 'createInput'): api_member('MirrorFeatureInput'),
    }

    def test_receiver_resolves_through_two_imported_gear_modules(self):
        result, output = self.run_checker(
            """
            from .helicalgear import HelicalGearGenerator

            class HerringboneGearGenerator(HelicalGearGenerator):
                def buildTooth(self, ctx):
                    return self.getComponent().features.mirrorFeatures.createInput(
                        ctx.entities, ctx.helixPlane)
            """,
            api_names={'createInput'},
            members=self.DERIVED_GEAR_MEMBERS,
            framework_files=self.DERIVED_GEAR_FRAMEWORK)

        self.assertEqual(result, 0, output)
        self.assertNotIn('receiver ownership is required', output)

    def test_a_broken_middle_module_still_reports_the_unknown_receiver(self):
        # The negative control for the test above: with `helicalgear` naming a base nothing
        # defines, the chain to `Generator` really is broken and the finding must come back.
        framework = dict(self.DERIVED_GEAR_FRAMEWORK)
        framework['helicalgear.py'] = (
            "class HelicalGearGenerator(NoSuchBase):\n"
            "    def loftTooth(self, ctx):\n"
            "        pass\n"
        )
        result, output = self.run_checker(
            """
            from .helicalgear import HelicalGearGenerator

            class HerringboneGearGenerator(HelicalGearGenerator):
                def buildTooth(self, ctx):
                    return self.getComponent().features.mirrorFeatures.createInput(
                        ctx.entities, ctx.helixPlane)
            """,
            api_names={'createInput'},
            members=self.DERIVED_GEAR_MEMBERS,
            framework_files=framework)

        self.assertEqual(result, 1)
        self.assertIn('receiver ownership is required', output)

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



class RefutedCallTest(unittest.TestCase):
    """A call the database declares and Fusion does not have must block.

    adsk.core.Base.cast is the first of these. The database reports it as a staticmethod and the
    intellisense stub declares it, so pyright, the API-call check and the novel-type check all
    passed a bevel gear that raised AttributeError on the very first line of its constructor.
    Only running the add-in found it, which is why the finding lives in a list rather than in
    someone's memory.
    """

    def test_base_cast_is_refuted(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'fusion_api', str(Path(__file__).with_name('fusion_api.py')))
        fusion_api = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fusion_api)

        rows = [row for row in fusion_api.REFUTED_CALLS
                if row[0] == 'cast' and row[1] == 'adsk.core.Base']
        self.assertEqual(len(rows), 1, 'adsk.core.Base.cast must stay refuted')
        self.assertIn('AttributeError', rows[0][2])
        for row in fusion_api.REFUTED_CALLS:
            with self.subTest(call='%s.%s' % (row[1], row[0])):
                self.assertTrue(row[2], 'every refuted call states its evidence')

    def test_checker_blocks_a_refuted_call(self):
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / 'candidate.py'
            candidate.write_text(
                'import adsk.core\n'
                'class Gear:\n'
                '    def __init__(self):\n'
                '        self.plane = adsk.core.Base.cast(None)\n')
            result = subprocess.run(
                [sys.executable, str(Path(__file__).with_name('check_api_calls.py')),
                 str(candidate)],
                capture_output=True, text=True)

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('the database declares it but Fusion does NOT have it', result.stdout)


if __name__ == '__main__':
    unittest.main()
