#!/usr/bin/env python3
"""Regression tests for the generated-candidate novel-type baseline."""
import importlib.util
import tempfile
import unittest
from pathlib import Path


CHECKER = Path(__file__).with_name('check_novel_types.py')
SPEC = importlib.util.spec_from_file_location('check_novel_types', CHECKER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ReferenceGearTests(unittest.TestCase):
    def test_copied_candidate_excludes_its_source_from_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / 'reference'
            reference.mkdir()
            source = reference / 'spurgear.py'
            source.write_text('source gear\n')
            (reference / 'othergear.py').write_text('other gear\n')
            candidate = root / 'spurgear.generated.py'
            candidate.write_text(source.read_text())

            self.assertEqual(
                MODULE.reference_gears(str(reference), str(candidate)),
                [str(reference / 'othergear.py')],
            )

    def test_in_place_candidate_excludes_itself(self):
        with tempfile.TemporaryDirectory() as directory:
            reference = Path(directory)
            candidate = reference / 'spurgear.py'
            candidate.write_text('source gear\n')
            other = reference / 'othergear.py'
            other.write_text('other gear\n')

            self.assertEqual(
                MODULE.reference_gears(str(reference), str(candidate)),
                [str(other)],
            )


class DiagnosticFilterTests(unittest.TestCase):
    def test_unverified_fusion_member_is_non_gating(self):
        for member, class_name in (
                ('project', 'Sketch'),
                ('addConstantRadiusEdgeSet', 'FilletFeatureInput')):
            diagnostic = {
                'rule': 'reportAttributeAccessIssue',
                'message': 'Cannot access attribute "%s" for class "adsk.fusion.%s"' %
                           (member, class_name),
            }

            with self.subTest(member=member):
                self.assertTrue(MODULE.is_unverified_api_diagnostic(diagnostic))

    def test_other_member_on_valid_class_remains_gating(self):
        diagnostic = {
            'rule': 'reportAttributeAccessIssue',
            'message': 'Cannot access attribute "missing" for class "Sketch"',
        }

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic))

    def test_local_class_namesake_remains_gating(self):
        diagnostic = {
            'rule': 'reportAttributeAccessIssue',
            'message': 'Cannot access attribute "project" for class "Sketch"',
        }

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic))

    def test_receiver_name_exemption_still_rejects_local_class_namesake(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.py'
            path.write_text(
                'class Sketch:\n'
                '    pass\n'
                '\n'
                'def build(sketch: Sketch, entity):\n'
                '    return sketch.project(entity)\n')
            diagnostic = {
                'rule': 'reportAttributeAccessIssue',
                'message': 'Cannot access attribute "project" for class "Sketch"',
                'range': {'start': {'line': 3}},
            }

            verified = MODULE.verified_fusion_classes(str(path))

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic, verified))

    def test_receiver_name_exemption_matches_sanctioned_call(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.py'
            path.write_text(
                'def build(toolsSketch, entity):\n'
                '    return toolsSketch.project(entity)\n')
            diagnostic = {
                'rule': 'reportAttributeAccessIssue',
                'message': 'Cannot access attribute "project" for class "Sketch"',
                'range': {'start': {'line': 1}},
            }

            verified = MODULE.verified_fusion_classes(str(path))

        self.assertTrue(MODULE.is_unverified_api_diagnostic(diagnostic, verified))

    def test_prefixed_receiver_name_does_not_match_sanctioned_call(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.py'
            path.write_text(
                'def build(other, entity):\n'
                '    return other.sketch.project(entity)\n')
            diagnostic = {
                'rule': 'reportAttributeAccessIssue',
                'message': 'Cannot access attribute "project" for class "Sketch"',
                'range': {'start': {'line': 1}},
            }

            verified = MODULE.verified_fusion_classes(str(path))

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic, verified))

    def test_unverified_member_on_wrong_class_remains_gating(self):
        diagnostic = {
            'rule': 'reportAttributeAccessIssue',
            'message': 'Cannot access attribute "project" for class "SketchPoint"',
        }

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic))

    def test_optional_member_access_remains_gating(self):
        diagnostic = {
            'rule': 'reportOptionalMemberAccess',
            'message': '"x" is not a known attribute of "None"',
        }

        self.assertFalse(MODULE.is_unverified_api_diagnostic(diagnostic))


if __name__ == '__main__':
    unittest.main()
