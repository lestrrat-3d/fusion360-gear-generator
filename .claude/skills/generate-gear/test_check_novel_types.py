#!/usr/bin/env python3
"""Regression tests for the generated-candidate novel-type baseline."""
import importlib.util
import json
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

    def test_qualified_receiver_name_matches_sanctioned_call(self):
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

        # `other.sketch` denotes a Sketch, and pyright named that same class, so the pair
        # agrees. The receiver still has to be typed for the api-call check to exempt it.
        self.assertTrue(MODULE.is_unverified_api_diagnostic(diagnostic, verified))

    def test_verified_self_fusion_field_matches_sanctioned_call(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.py'
            path.write_text(
                'class Candidate:\n'
                '    def __init__(self, sketch: adsk.fusion.Sketch):\n'
                '        self.sketch = sketch\n'
                '\n'
                '    def build(self, entity):\n'
                '        return self.sketch.project(entity)\n')
            diagnostic = {
                'rule': 'reportAttributeAccessIssue',
                'message': 'Cannot access attribute "project" for class "Sketch"',
                'range': {'start': {'line': 5}},
            }

            verified = MODULE.verified_fusion_classes(str(path))

        self.assertTrue(MODULE.is_unverified_api_diagnostic(diagnostic, verified))

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


class AcceptedNoiseTests(unittest.TestCase):
    def test_argument_entry_matches_wrong_argument_signature(self):
        diagnostic = {
            'rule': 'reportArgumentType',
            'message': 'Argument of type "int" cannot be assigned to parameter "orientation" '
                       'of type "DimensionOrientations" in function "addDistanceDimension"',
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'
            path.write_text(
                '[{"rule": "reportArgumentType", "param": "orientation",'
                ' "want": "DimensionOrientations", "func": "addDistanceDimension",'
                ' "why": "stub types the enum members as int"}]')

            accepted = MODULE.accepted_signatures(str(path))

        self.assertIn(MODULE.signature(diagnostic), accepted)

    def test_message_entry_matches_plain_signature(self):
        diagnostic = {
            'rule': 'reportOptionalMemberAccess',
            'message': '"x" is not a known attribute of "None"',
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'
            path.write_text(
                '[{"rule": "reportOptionalMemberAccess",'
                ' "message": "\\"x\\" is not a known attribute of \\"None\\"",'
                ' "why": "example"}]')

            accepted = MODULE.accepted_signatures(str(path))

        self.assertIn(MODULE.signature(diagnostic), accepted)

    def test_entry_without_why_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'
            path.write_text('[{"rule": "reportArgumentType", "param": "orientation",'
                            ' "want": "DimensionOrientations", "func": "addDistanceDimension"}]')

            with self.assertRaises(ValueError):
                MODULE.accepted_signatures(str(path))

    def test_missing_file_accepts_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(
                MODULE.accepted_signatures(str(Path(directory) / 'absent.json')), set())

    def test_checked_in_record_parses_and_names_only_triaged_noise(self):
        accepted = MODULE.accepted_signatures()
        self.assertIn(
            ('reportArgumentType', 'orientation', 'DimensionOrientations',
             'addDistanceDimension'),
            accepted)


class RecordAcceptedTests(unittest.TestCase):
    ARGUMENT_DIAGNOSTIC = {
        'rule': 'reportArgumentType',
        'message': 'Argument of type "int" cannot be assigned to parameter "orientation" '
                   'of type "DimensionOrientations" in function "addDistanceDimension"',
        'range': {'start': {'line': 41}},
    }
    PLAIN_DIAGNOSTIC = {
        'rule': 'reportOptionalMemberAccess',
        'message': '"x" is not a known attribute of "None"\n  second line of detail',
        'range': {'start': {'line': 7}},
    }

    def test_wrong_argument_entry_round_trips_through_accepted_signatures(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'

            status, entry = MODULE.record_accepted(
                str(path), self.ARGUMENT_DIAGNOSTIC, 'the stubs type the members as int')

            self.assertEqual(status, 'added')
            self.assertEqual(list(entry), ['rule', 'param', 'want', 'func', 'why'])
            self.assertEqual(entry['param'], 'orientation')
            self.assertIn(MODULE.signature(self.ARGUMENT_DIAGNOSTIC),
                          MODULE.accepted_signatures(str(path)))

    def test_plain_entry_records_first_message_line_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'

            _, entry = MODULE.record_accepted(str(path), self.PLAIN_DIAGNOSTIC, 'stub gap')

            self.assertEqual(list(entry), ['rule', 'message', 'why'])
            self.assertEqual(entry['message'], '"x" is not a known attribute of "None"')
            self.assertIn(MODULE.signature(self.PLAIN_DIAGNOSTIC),
                          MODULE.accepted_signatures(str(path)))

    def test_missing_file_is_created_with_one_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'absent.json'

            MODULE.record_accepted(str(path), self.PLAIN_DIAGNOSTIC, 'stub gap')

            self.assertEqual(len(json.loads(path.read_text(encoding='utf-8'))), 1)

    def test_re_recording_updates_the_why_in_place(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'
            MODULE.record_accepted(str(path), self.ARGUMENT_DIAGNOSTIC, 'first reason')
            first_status, _ = MODULE.record_accepted(
                str(path), self.PLAIN_DIAGNOSTIC, 'other finding')

            status, entry = MODULE.record_accepted(
                str(path), self.ARGUMENT_DIAGNOSTIC, 'second reason')

            entries = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(first_status, 'added')
            self.assertEqual(status, 'updated')
            self.assertEqual(len(entries), 2)
            self.assertEqual(entries[0]['why'], 'second reason')
            self.assertEqual(entry['why'], 'second reason')
            self.assertEqual(entries[1]['message'], '"x" is not a known attribute of "None"')

    def test_blank_why_is_refused_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'
            MODULE.record_accepted(str(path), self.PLAIN_DIAGNOSTIC, 'stub gap')
            before = path.read_text(encoding='utf-8')

            with self.assertRaises(ValueError):
                MODULE.record_accepted(str(path), self.ARGUMENT_DIAGNOSTIC, '   ')

            self.assertEqual(path.read_text(encoding='utf-8'), before)

    def test_written_file_keeps_the_checked_in_formatting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'accepted.json'

            MODULE.record_accepted(str(path), self.PLAIN_DIAGNOSTIC, 'stub gap')

            self.assertEqual(
                path.read_text(encoding='utf-8'),
                '[\n'
                '  {\n'
                '    "rule": "reportOptionalMemberAccess",\n'
                '    "message": "\\"x\\" is not a known attribute of \\"None\\"",\n'
                '    "why": "stub gap"\n'
                '  }\n'
                ']\n')


class NovelOrderTests(unittest.TestCase):
    def test_findings_on_one_line_order_by_rule_then_message(self):
        first = {'rule': 'reportArgumentType', 'message': 'b complaint',
                 'range': {'start': {'line': 4}}}
        second = {'rule': 'reportArgumentType', 'message': 'c complaint',
                  'range': {'start': {'line': 4}}}
        third = {'rule': 'reportOptionalMemberAccess', 'message': 'a complaint',
                 'range': {'start': {'line': 4}}}

        self.assertEqual(
            sorted([third, second, first], key=MODULE.novel_sort_key),
            [first, second, third])


if __name__ == '__main__':
    unittest.main()
