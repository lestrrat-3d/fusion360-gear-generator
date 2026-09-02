#!/usr/bin/env python3
"""Regression tests for the contract check's source-drift guards.

The guards exist for the one class of regression the rest of the contract check
cannot see: a constraint recipe reverting to an alternative that also solves.
Nothing is renamed when that happens, so the constant, class and method checks
all stay green. These tests hold the guards to catching it.
"""
import importlib.util
import json
import re
import tempfile
import unittest
from pathlib import Path


CHECKER = Path(__file__).with_name('check_contract.py')
SPEC = importlib.util.spec_from_file_location('check_contract', CHECKER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

REPO = Path(__file__).resolve().parents[3]

MODULE_SOURCE = '''\
class ToothGenerator:
    def _drawFlankToRoot(self, flankStart):
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flankStart)
        horizontal = sketch.sketchDimensions.addDistanceDimension(
            origin, line.startSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        horizontal.parameter.value = dx
        vertical = sketch.sketchDimensions.addDistanceDimension(
            origin, line.startSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)
        vertical.parameter.value = dy

    def drawTooth(self):
        sketch.geometricConstraints.addCoincident(toothTop, self.tipCircle)
'''


def guard(**overrides):
    base = {
        'file': 'gear.py',
        'in_function': '_drawFlankToRoot',
        'why': 'invented guard for this test',
        'required': ['DimensionOrientations\\.HorizontalDimensionOrientation'],
        'banned': ['addCoincident\\('],
    }
    base.update(overrides)
    return base


class ScopedGuardTests(unittest.TestCase):
    """`in_function` narrows a pattern to the one def that owns the recipe."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.source = self.root / 'gear.py'
        self.source.write_text(MODULE_SOURCE)

    def check(self, *guards):
        return MODULE.guard_problems(list(guards), str(self.source), None, str(self.root))

    def test_intact_recipe_reports_nothing(self):
        self.assertEqual(self.check(guard()), [])

    def test_banned_call_outside_the_scope_is_not_a_violation(self):
        # drawTooth legitimately adds a coincidence; only the flank-to-root
        # helper may not. A module-wide ban would flag the wrong def.
        self.assertEqual(self.check(guard(banned=['addCoincident\\('])), [])

    def test_rejected_coincidence_inside_the_scope_is_a_violation(self):
        self.source.write_text(MODULE_SOURCE.replace(
            '        horizontal.parameter.value = dx',
            '        sketch.geometricConstraints.addCoincident(rootEndPoint, self.rootCircle)'))

        problems = self.check(guard())

        self.assertEqual(len(problems), 1)
        self.assertIn('rejected pattern', problems[0])
        self.assertIn('_drawFlankToRoot', problems[0])
        self.assertIn('invented guard for this test', problems[0])

    def test_dropped_signed_dimension_is_a_violation(self):
        self.source.write_text(MODULE_SOURCE.replace(
            'DimensionOrientations.HorizontalDimensionOrientation', 'AlignedDimension'))

        problems = self.check(guard())

        self.assertEqual(len(problems), 1)
        self.assertIn('required pattern', problems[0])

    def test_renamed_function_is_a_violation_rather_than_a_silent_pass(self):
        self.source.write_text(MODULE_SOURCE.replace('_drawFlankToRoot', '_drawStub'))

        problems = self.check(guard())

        self.assertEqual(len(problems), 1)
        self.assertIn('no definition of _drawFlankToRoot', problems[0])

    def test_ambiguous_function_name_is_a_violation(self):
        self.source.write_text(MODULE_SOURCE + '\n\ndef _drawFlankToRoot():\n    pass\n')

        problems = self.check(guard())

        self.assertEqual(len(problems), 1)
        self.assertIn('cannot isolate', problems[0])

    def test_missing_file_is_a_violation(self):
        problems = self.check(guard(file='nowhere.py', in_function=None))

        self.assertEqual(len(problems), 1)
        self.assertIn('file not found', problems[0])

    def test_unscoped_guard_reads_the_whole_file(self):
        problems = self.check(guard(in_function=None, required=['def drawTooth'], banned=[]))

        self.assertEqual(problems, [])


class CandidateRoutingTests(unittest.TestCase):
    """A guard on the manifest's own module reads the candidate under test."""

    def test_module_guard_reads_the_candidate_not_the_repo_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'lib').mkdir()
            (root / 'lib' / 'gear.py').write_text(MODULE_SOURCE)
            candidate = root / 'candidate.py'
            candidate.write_text(MODULE_SOURCE.replace(
                '        horizontal.parameter.value = dx',
                '        sketch.geometricConstraints.addCoincident(rootEndPoint, self.rootCircle)'))

            problems = MODULE.guard_problems(
                [guard(file='lib/gear.py')], str(candidate), 'lib/gear.py', str(root))

            self.assertEqual(len(problems), 1)
            self.assertIn('candidate.py', problems[0])
            self.assertIn('rejected pattern', problems[0])


class ShippedManifestTests(unittest.TestCase):
    """The guards the repo actually ships, against the sources they guard."""

    manifest_path = REPO / 'spec' / 'spurgear' / 'contract.json'
    generated = REPO / 'lib' / 'geargen' / 'spurgear.py'

    def setUp(self):
        if not self.manifest_path.is_file() or not self.generated.is_file():
            self.skipTest('run from the gear repository')
        self.guards = json.loads(self.manifest_path.read_text())['source_guards']

    def test_shipped_sources_satisfy_every_guard(self):
        problems = MODULE.guard_problems(
            self.guards, str(self.generated), 'lib/geargen/spurgear.py', str(REPO))

        self.assertEqual(problems, [])

    def test_every_guard_names_the_reason_it_exists(self):
        for entry in self.guards:
            with self.subTest(file=entry['file'], scope=entry.get('in_function')):
                self.assertTrue(entry.get('why'))
                self.assertTrue(entry.get('required') or entry.get('banned'))

    def test_reverted_root_end_recipe_fails_the_shipped_guard(self):
        # The mutation is located by the guard's own required pattern rather than by a
        # literal from one emit. The module is build output and its local names change
        # between regenerations; the recipe the guard protects does not.
        text = self.generated.read_text()
        match = re.search(r'^.*\.parameter\.value\s*=\s*abs\(.*$', text, re.M)
        self.assertIsNotNone(
            match, 'no signed axis-dimension assignment found to mutate')
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / 'spurgear.generated.py'
            original = match.group(0)
            indent = original[:len(original) - len(original.lstrip())]
            candidate.write_text(text.replace(
                original,
                indent + 'sketch.geometricConstraints.addCoincident('
                'rootEnd, self.rootCircle)'))

            problems = MODULE.guard_problems(
                self.guards, str(candidate), 'lib/geargen/spurgear.py', str(REPO))

        self.assertTrue(any('rejected pattern' in p for p in problems), problems)
        self.assertTrue(any('SPUR-F-FLANK-ROOT' in p for p in problems), problems)

    def test_bore_anchor_grounded_on_sketch_origin_fails_the_shipped_guard(self):
        # Located by the guard's own required patterns, for the reason above: the
        # grounding coincident is what matters, not what the emit called its second
        # argument.
        text = self.generated.read_text()
        start = text.index('def buildBore')
        end = text.find('\n    def ', start + 1)
        bore = text[start:end if end != -1 else len(text)]
        match = re.search(r'^.*\.anchorPoint,.*$', bore, re.M)
        self.assertIsNotNone(
            match, 'no anchor-grounding coincident found in buildBore to mutate')
        grounded = match.group(0)
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / 'spurgear.generated.py'
            candidate.write_text(text.replace(
                grounded,
                grounded.split('.anchorPoint,')[0] + '.originPoint, boreSketch.originPoint)'))

            problems = MODULE.guard_problems(
                self.guards, str(candidate), 'lib/geargen/spurgear.py', str(REPO))

        self.assertTrue(any('rejected pattern' in p for p in problems), problems)
        self.assertTrue(any('PB-CIRCLE-CENTER' in p for p in problems), problems)


if __name__ == '__main__':
    unittest.main()
