#!/usr/bin/env python3
"""Unions and turned copies are batched into one Fusion feature each, not one per tooth.

Thirty teeth joined one at a time are thirty combines, and thirty turned copies are sixty features. The runtime
records them and makes one circular pattern and one combine. This runs the runtime against a recording stand-in for
Fusion and counts the features.
"""
import importlib
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class FakeBRep:
    count = 0

    def __init__(self, x, y, z):
        FakeBRep.count += 1
        self.entityToken = f'body{FakeBRep.count}'
        self.name = ''
        self.physicalProperties = SimpleNamespace(centerOfMass=SimpleNamespace(x=x, y=y, z=z))


class FakeFeatures:
    """Records each feature added, and answers the way Fusion does for the calls the batching makes."""

    def __init__(self, log, teeth):
        self.log, self.teeth = log, teeth
        self.combineFeatures = SimpleNamespace(createInput=self._combine_input, add=self._add('combine'))
        self.copyPasteBodies = SimpleNamespace(add=self._copy)
        self.moveFeatures = SimpleNamespace(createInput2=lambda entities: SimpleNamespace(
            defineAsFreeMove=lambda matrix: None), add=self._add('move'))
        self.circularPatternFeatures = SimpleNamespace(createInput=self._pattern_input, add=self._pattern)
        self.source = None

    def _add(self, kind):
        def add(feature_input):
            self.log.append((kind, getattr(feature_input, 'tools', None)))
            return SimpleNamespace(name='')
        return add

    def _combine_input(self, base, tools):
        return SimpleNamespace(base=base, tools=[tools.items[i] for i in range(len(tools.items))])

    def _copy(self, entities):
        self.log.append(('copy', None))
        return SimpleNamespace(bodies=SimpleNamespace(count=1, item=lambda i: FakeBRep(1.0, 0.0, 0.0)), name='')

    def _pattern_input(self, entities, axis):
        self.source = entities.items[0]
        return SimpleNamespace(quantity=None, totalAngle=None, isSymmetric=None)

    def _pattern(self, feature_input):
        count = int(feature_input.quantity)
        self.log.append(('pattern', count))
        bodies = [self.source]
        for k in range(1, count):
            a = 2 * math.pi * k / count
            bodies.append(FakeBRep(1.0, math.cos(a) * 3.0, math.sin(a) * 3.0))
        return SimpleNamespace(bodies=bodies, name='')


class FakeCollection:
    def __init__(self):
        self.items = []

    def add(self, item):
        self.items.append(item)


class BatchingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = {n: m for n, m in sys.modules.items() if n == 'adsk' or n.startswith(('adsk.', 'addin'))}
        adsk = types.ModuleType('adsk')
        adsk.__path__ = []
        for name in ('core', 'fusion'):
            stub = mock.MagicMock(name=f'adsk.{name}')
            sys.modules[f'adsk.{name}'] = stub
            setattr(adsk, name, stub)
        sys.modules['adsk'] = adsk
        spec = importlib.util.spec_from_file_location(
            'addin', ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
        sys.modules['addin'] = importlib.util.module_from_spec(spec)
        cls.fx = importlib.import_module('addin.lib.geargen.decadfusion')

    @classmethod
    def tearDownClass(cls):
        for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
            del sys.modules[name]
        sys.modules.update(cls.saved)

    def setUp(self):
        fx = self.fx
        self.log = []
        self.runtime = SimpleNamespace(
            component=SimpleNamespace(features=FakeFeatures(self.log, 12)),
            to_local=lambda v: v, to_world=lambda v: v, to_world_dir=lambda v: v, scale=1.0)
        self.patches = [
            mock.patch.object(fx.adsk.core.ObjectCollection, 'create', side_effect=FakeCollection),
            mock.patch.object(fx, '_axis_through', return_value='axis'),
            mock.patch.object(fx.solids, 'name_feature'),
            mock.patch.object(fx, 'logged', lambda f: f, create=True),
            mock.patch.object(fx, 'value_input', lambda v, s=1.0: v),
        ]
        for patcher in self.patches:
            patcher.start()
        fx.adsk.core.ValueInput.createByReal.side_effect = lambda v: v
        fx.adsk.core.ValueInput.createByString.side_effect = lambda v: v
        fx.adsk.fusion.FeatureOperations.JoinFeatureOperation = 'join'

    def tearDown(self):
        for patcher in self.patches:
            patcher.stop()

    def body(self, x, y, z):
        return self.fx.Body(self.runtime, FakeBRep(x, y, z))

    def test_the_copies_of_a_tooth_at_equal_angles_are_one_pattern_and_the_join_is_one_combine(self):
        fx, teeth = self.fx, 12
        blank, tooth = self.body(0, 0, 0), self.body(1.0, 3.0, 0.0)
        axis, centre = fx.r3_new_vec(1, 0, 0), fx.r3_vec()
        pieces = [tooth]
        for i in range(1, teeth):
            angle = fx.units_radians(2 * math.pi * i / teeth)
            pieces.append(fx.decad_copy(tooth, fx.decad_with_copy_rotation(centre, axis, angle), name='Tooth Copy'))
        joined = blank
        # The gallery's balanced join: pairs, then the pairs, then the blank.
        level = pieces
        while len(level) > 1:
            level = [fx.decad_union(level[i], level[i + 1], name='Pair') if i + 1 < len(level) else level[i]
                     for i in range(0, len(level), 2)]
        joined = fx.decad_union(joined, level[0], name='Teeth Join')
        self.assertEqual([], self.log, 'nothing is made before the geometry is needed')
        joined.body
        self.assertEqual([('pattern', teeth)], [e for e in self.log if e[0] == 'pattern'])
        self.assertEqual(1, sum(1 for e in self.log if e[0] == 'combine'))
        self.assertEqual(0, sum(1 for e in self.log if e[0] in ('copy', 'move')))
        # Every copy is a distinct body of the pattern.
        tokens = {c.body.entityToken for c in pieces[1:]}
        self.assertEqual(teeth - 1, len(tokens))

    def test_copies_at_uneven_angles_are_made_one_by_one(self):
        fx = self.fx
        tooth = self.body(1.0, 3.0, 0.0)
        axis, centre = fx.r3_new_vec(1, 0, 0), fx.r3_vec()
        copies = [fx.decad_copy(tooth, fx.decad_with_copy_rotation(centre, axis, fx.units_radians(a)), name='Copy')
                  for a in (0.5, 1.0, 2.5)]
        fx._realize_copies(copies)
        self.assertEqual([], self.log)
        copies[0].body
        self.assertEqual(['copy', 'move'], [e[0] for e in self.log])


if __name__ == '__main__':
    unittest.main()
