#!/usr/bin/env python3
"""Regression tests for how the unverified-call watchlist reads a receiver."""
import importlib.util
import unittest
from pathlib import Path


API_PATH = Path(__file__).with_name('fusion_api.py')
API_SPEC = importlib.util.spec_from_file_location('fusion_api', API_PATH)
FUSION_API = importlib.util.module_from_spec(API_SPEC)
API_SPEC.loader.exec_module(FUSION_API)

COMPILE_PATH = Path(__file__).with_name('check_compile.py')
COMPILE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_SPEC)
COMPILE_SPEC.loader.exec_module(COMPILE_CHECKER)


def entry(member):
    """The watchlist entry for one member name."""
    for row in FUSION_API.UNVERIFIED_CALLS:
        if row[0] == member:
            return row
    raise AssertionError('no watchlist entry for %s' % member)


class DenotesClassTest(unittest.TestCase):
    def test_whole_name_denotes_the_class(self):
        for name in ('sketch', 'Sketch', 'SKETCH'):
            with self.subTest(name=name):
                self.assertTrue(FUSION_API.denotes_class('Sketch', name))

    def test_trailing_word_denotes_the_class(self):
        for name in ('gearSketch', 'boreSketch', 'toolsSketch', 'bore_sketch'):
            with self.subTest(name=name):
                self.assertTrue(FUSION_API.denotes_class('Sketch', name))

    def test_run_on_lowercase_namesake_does_not_denote_the_class(self):
        for name in ('mysketch', 'unsketch'):
            with self.subTest(name=name):
                self.assertFalse(FUSION_API.denotes_class('Sketch', name))

    def test_leading_word_does_not_denote_the_class(self):
        self.assertFalse(FUSION_API.denotes_class('Sketch', 'sketchTexts'))

    def test_shorter_name_does_not_denote_the_class(self):
        self.assertFalse(FUSION_API.denotes_class('SketchTexts', 'Texts'))

    def test_missing_name_does_not_denote_the_class(self):
        self.assertFalse(FUSION_API.denotes_class('Sketch', None))


class SketchEntryTest(unittest.TestCase):
    """A step list is compiled without reading an implementation, so it names its own sketches."""

    def matches(self, receiver):
        return FUSION_API.receiver_matches(entry('project')[2], receiver)

    def test_any_sketch_variable_matches(self):
        for receiver in ('sketch', 'toolsSketch', 'gearSketch', 'boreSketch', 'self.sketch',
                         'ctx.gearSketch', 'Sketch'):
            with self.subTest(receiver=receiver):
                self.assertTrue(self.matches(receiver))

    def test_unrelated_receiver_does_not_match(self):
        for receiver in ('component', 'self.rootComponent', 'sketchTexts'):
            with self.subTest(receiver=receiver):
                self.assertFalse(self.matches(receiver))

    def test_bare_call_does_not_match(self):
        self.assertFalse(self.matches(None))


class SketchTextsEntryTest(unittest.TestCase):
    def matches(self, receiver):
        return FUSION_API.receiver_matches(entry('createInput2')[2], receiver)

    def test_qualified_sketch_texts_receiver_matches(self):
        for receiver in ('sketchTexts', 'sketch.sketchTexts', 'gearSketch.sketchTexts',
                         'self.sketch.sketchTexts', 'SketchTexts'):
            with self.subTest(receiver=receiver):
                self.assertTrue(self.matches(receiver))

    def test_real_namesake_on_another_class_is_not_exempted(self):
        for receiver in ('chamferFeatures', 'component.features.chamferFeatures',
                         'ChamferFeatures', 'moveFeatures', 'MoveFeatures'):
            with self.subTest(receiver=receiver):
                self.assertFalse(self.matches(receiver))


class FilletEntryTest(unittest.TestCase):
    def matches(self, receiver):
        return FUSION_API.receiver_matches(entry('addConstantRadiusEdgeSet')[2], receiver)

    def test_fillet_input_receiver_matches(self):
        for receiver in ('filletInput', 'FilletFeatureInput', 'self.filletInput'):
            with self.subTest(receiver=receiver):
                self.assertTrue(self.matches(receiver))

    def test_edge_set_inputs_receiver_does_not_match(self):
        for receiver in ('FilletEdgeSetInputs', 'edgeSetInputs'):
            with self.subTest(receiver=receiver):
                self.assertFalse(self.matches(receiver))


class WatchlistShapeTest(unittest.TestCase):
    def test_every_entry_names_its_own_class(self):
        """Entries carry class names. A local variable lifted from one gear would fail here."""
        for member, cls, words, _ in FUSION_API.UNVERIFIED_CALLS:
            with self.subTest(member=member):
                self.assertIn(cls.rsplit('.', 1)[-1], words)

    def test_unverified_class_reports_the_matching_entry(self):
        self.assertEqual(FUSION_API.unverified_class('project', 'boreSketch'),
                         'adsk.fusion.Sketch')
        self.assertEqual(FUSION_API.unverified_class('createInput2', 'gearSketch.sketchTexts'),
                         'adsk.fusion.SketchTexts')

    def test_unverified_class_reports_nothing_for_an_uncovered_call(self):
        self.assertIsNone(FUSION_API.unverified_class('createInput2', 'chamferFeatures'))
        self.assertIsNone(FUSION_API.unverified_class('project', 'component'))


class IsWatchedCallTest(unittest.TestCase):
    """The compile gate reads the watchlist through the same rule."""

    def test_every_sketch_spelling_is_watched(self):
        for receiver in ('toolsSketch', 'gearSketch', 'boreSketch'):
            with self.subTest(receiver=receiver):
                self.assertTrue(COMPILE_CHECKER.is_watched_call('project', receiver))

    def test_sketch_texts_receiver_is_watched(self):
        self.assertTrue(
            COMPILE_CHECKER.is_watched_call('createInput2', 'gearSketch.sketchTexts'))

    def test_chamfer_receiver_is_not_watched(self):
        self.assertFalse(
            COMPILE_CHECKER.is_watched_call('createInput2', 'component.features.chamferFeatures'))

    def test_fillet_input_receiver_is_watched(self):
        self.assertTrue(
            COMPILE_CHECKER.is_watched_call('addConstantRadiusEdgeSet', 'filletInput'))


class StepListTest(unittest.TestCase):
    """The shapes a compiled step list actually produces, straight through the parser."""

    SPANS = (
        '`toolsSketch.project([self.anchorPoint], True)`',
        '`gearSketch.project([ctx.anchorPoint], True)`',
        '`boreSketch.project([ctx.anchorPoint], True)`',
        '`gearSketch.sketchTexts.createInput2(text, size)`',
        '`component.features.chamferFeatures.createInput2()`',
        '`filletInput.addConstantRadiusEdgeSet(edges, radius)`',
    )

    def verdicts(self):
        src = '\n'.join(self.SPANS)
        return {
            (name, receiver): COMPILE_CHECKER.is_watched_call(name, receiver)
            for name, receiver in COMPILE_CHECKER.named_call_shapes(src)
        }

    def test_watchlist_covers_every_sketch_call_and_no_namesake(self):
        self.assertEqual(self.verdicts(), {
            ('project', 'toolsSketch'): True,
            ('project', 'gearSketch'): True,
            ('project', 'boreSketch'): True,
            ('createInput2', 'gearSketch.sketchTexts'): True,
            ('createInput2', 'component.features.chamferFeatures'): False,
            ('addConstantRadiusEdgeSet', 'filletInput'): True,
        })

    def test_watched_calls_reports_every_line_a_sketch_call_sits_on(self):
        src = '\n'.join(self.SPANS)
        seen = COMPILE_CHECKER.watched_calls(src, 'steps.md')
        self.assertEqual(seen['project'], 'steps.md:1,2,3')
        self.assertEqual(seen['createInput2'], 'steps.md:4')
        self.assertEqual(seen['addConstantRadiusEdgeSet'], 'steps.md:6')


if __name__ == '__main__':
    unittest.main()
