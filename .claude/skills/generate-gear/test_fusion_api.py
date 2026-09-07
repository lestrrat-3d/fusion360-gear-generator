#!/usr/bin/env python3
"""Regression tests for how the unverified-call watchlist reads a receiver."""
import importlib.util
import unittest
from pathlib import Path
from unittest import mock


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


class DescribeCallTest(unittest.TestCase):
    KEYS = {
        'schema', 'owner', 'name', 'status', 'scope', 'disposition', 'declared_on',
        'returns', 'evidence', 'stale_watchlist',
    }

    def describe_typed(self, owner, name, info=None, members=None):
        with mock.patch.object(FUSION_API, 'member_info', return_value=info), \
                mock.patch.object(FUSION_API, 'class_members', return_value=members or {}):
            return FUSION_API.describe_call(owner, name)

    def test_member_info_keeps_qualified_owner_from_query_output(self):
        response = (
            'adsk.core.CommandInputs.addSelectionInput  [method]\n'
            'signature: (self, id: str, name: str, commandPrompt: str) '
            '-> SelectionCommandInput\n')
        with mock.patch.object(FUSION_API, '_run', return_value=response):
            info = FUSION_API.member_info('CommandInputs', 'addSelectionInput')

        self.assertEqual(info['lookup'], 'adsk.core.CommandInputs')
        self.assertEqual(info['declared_on'], 'adsk.core.CommandInputs')
        self.assertEqual(info['returns'], 'SelectionCommandInput')

    def test_member_info_keeps_qualified_inherited_owner_from_query_output(self):
        response = (
            'adsk.fusion.ChildTools.addWidget  [method inherited]\n'
            'inherited from: adsk.fusion.BaseTools\n'
            'signature: (self) -> adsk.fusion.Widget\n')
        with mock.patch.object(FUSION_API, '_run', return_value=response):
            info = FUSION_API.member_info('adsk.fusion.ChildTools', 'addWidget')

        self.assertEqual(info['lookup'], 'adsk.fusion.ChildTools')
        self.assertEqual(info['declared_on'], 'adsk.fusion.BaseTools')
        self.assertEqual(info['returns'], 'adsk.fusion.Widget')

    def test_documented_member(self):
        result = self.describe_typed(
            'adsk.fusion.WidgetTools', 'addWidget',
            {'declared_on': 'adsk.fusion.WidgetTools', 'returns': 'adsk.fusion.Widget'})

        self.assertEqual(set(result), self.KEYS)
        self.assertEqual((result['status'], result['scope'], result['disposition']),
                         ('documented', 'receiver', 'allow'))
        self.assertEqual(result['declared_on'], 'adsk.fusion.WidgetTools')
        self.assertEqual(result['returns'], 'adsk.fusion.Widget')

    def test_inherited_member(self):
        result = self.describe_typed(
            'adsk.fusion.WidgetTools', 'addWidget', members={
                'addWidget': 'adsk.fusion.BaseWidgetTools',
            })

        self.assertEqual(result['status'], 'documented')
        self.assertEqual(result['declared_on'], 'adsk.fusion.BaseWidgetTools')

    def test_repeated_lookup_reuses_session_after_seeded_member_miss(self):
        owner = 'adsk.fusion.WidgetTools'
        with FUSION_API.query_session(), \
                mock.patch.object(FUSION_API, 'member_info') as member_info, \
                mock.patch.object(
                    FUSION_API, 'class_members',
                    return_value={'addWidget': 'adsk.fusion.BaseWidgetTools'}) as class_members:
            FUSION_API.cache_member_info(owner, 'addWidget', None)
            first = FUSION_API.describe_call(owner, 'addWidget')
            second = FUSION_API.describe_call(owner, 'addWidget')

        member_info.assert_not_called()
        class_members.assert_called_once_with(owner)
        self.assertEqual(first, second)
        self.assertEqual(first['declared_on'], 'adsk.fusion.BaseWidgetTools')

    def test_watchlist_entries(self):
        for name, owner, _, _ in FUSION_API.UNVERIFIED_CALLS:
            with self.subTest(owner=owner, name=name):
                result = self.describe_typed(owner, name)
                self.assertEqual((result['status'], result['disposition']),
                                 ('unverified', 'advisory'))

    def test_wrong_owner_not_exempt(self):
        result = self.describe_typed('adsk.fusion.Component', 'project')
        self.assertEqual((result['status'], result['disposition']), ('not_found', 'block'))

    def test_refuted_precedence(self):
        with mock.patch.object(FUSION_API, 'member_info') as member_info:
            result = FUSION_API.describe_call('adsk.core.Base', 'cast')

        member_info.assert_not_called()
        self.assertEqual((result['status'], result['disposition']), ('refuted', 'block'))

    def test_database_unavailable(self):
        with mock.patch.object(
                FUSION_API, 'member_info', side_effect=FUSION_API.Unavailable('offline')):
            result = FUSION_API.describe_call('adsk.fusion.WidgetTools', 'addWidget')

        self.assertEqual((result['status'], result['disposition']),
                         ('unavailable', 'setup_error'))
        self.assertEqual(result['evidence'], ['offline'])

    def test_stale_watchlist(self):
        result = self.describe_typed(
            'adsk.fusion.Sketch', 'project',
            {'declared_on': 'adsk.fusion.Sketch', 'returns': 'adsk.core.ObjectCollection'})
        self.assertEqual(result['status'], 'documented')
        self.assertTrue(result['stale_watchlist'])
        self.assertTrue(any('UNVERIFIED_CALLS' in line for line in result['evidence']))

    def test_name_only_is_not_typed(self):
        with mock.patch.object(
                FUSION_API, 'lookup', return_value=[('adsk.fusion.Sketch.project', 'method')]):
            result = FUSION_API.describe_call(None, 'project')

        self.assertEqual((result['status'], result['scope'], result['disposition']),
                         ('documented', 'name_only', 'allow'))
        self.assertFalse(result['stale_watchlist'])
        self.assertIsNone(result['declared_on'])

    def test_name_only_does_not_apply_refuted_pair(self):
        with mock.patch.object(
                FUSION_API, 'lookup', return_value=[('adsk.core.Base.cast', 'staticmethod')]):
            result = FUSION_API.describe_call(None, 'cast')

        self.assertEqual((result['status'], result['scope'], result['disposition']),
                         ('documented', 'name_only', 'allow'))

    def test_invalid_inputs(self):
        for owner, name in (('Sketch', 'project'), ('adsk.fusion.Sketch.extra', 'project'),
                            ('adsk.fusion.Sketch', 'not-a-member')):
            with self.subTest(owner=owner, name=name), self.assertRaises(ValueError):
                FUSION_API.describe_call(owner, name)


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
