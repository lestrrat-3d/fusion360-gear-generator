#!/usr/bin/env python3
"""The Fusion runtime's own bookkeeping, run against recording stand-ins for Fusion.

A fake cannot show that a Fusion call is right. These tests pin what the runtime decides by itself: which body is which,
which container a body belongs to, the order pending copies and unions are made in, and the errors it raises for what it
cannot represent.
"""
import importlib
import math
import unittest
from types import SimpleNamespace
from unittest import mock

from fakefusion import Entity, FakeBRep, FusionTestCase, Recorder


class RuntimeTestCase(FusionTestCase):
    def setUp(self):
        fx = self.fx
        self.log = []
        self.features = Recorder(self.log)
        self.component = SimpleNamespace(name='Gear', features=self.features, bRepBodies=[])
        self.runtime = fx.Runtime(self.component, 'plane', SimpleNamespace(geometry=fx.Vec(0, 0, 0)), None)
        self.patches = [
            mock.patch.object(fx, '_axis_through', return_value='axis'),
            mock.patch.object(fx.solids, 'name_feature'),
        ]
        for patcher in self.patches:
            patcher.start()

    def tearDown(self):
        for patcher in self.patches:
            patcher.stop()

    def body(self, x=0.0, y=0.0, z=0.0, container=None):
        body = self.fx.Body(self.runtime, FakeBRep(x, y, z))
        if container is not None:
            self.runtime.move_body(body, container)
        return body


class EntityIdentityTest(RuntimeTestCase):
    """Bodies and sketch curves are matched with ==, never by entityToken, whose string can change over time."""

    def test_the_same_body_keeps_its_wrapper_when_its_token_changes(self):
        fbody = FakeBRep()
        self.component.bRepBodies = [fbody]
        first = self.runtime.bodies()
        self.component.bRepBodies = [fbody.again()]
        self.assertIs(first[0], self.runtime.bodies()[0])

    def test_a_pattern_does_not_return_its_source_as_a_copy(self):
        tooth = self.body(1.0, 3.0, 0.0)
        copies = self.fx.decad_pattern_copies(
            tooth, self.fx.decad_circular_pattern(Center=self.fx.r3_vec(), Axis=self.fx.r3_new_vec(1, 0, 0), Count=4),
            name='Pattern')
        self.assertEqual(3, len(copies))
        self.assertFalse(any(copy.body == tooth.body for copy in copies))

    def test_a_mirror_returns_the_new_body_and_not_its_source(self):
        fx = self.fx
        source = self.body(1.0, 0.0, 0.0)
        with mock.patch.object(fx.Runtime, 'create_plane_from_frame', return_value=SimpleNamespace(entity='plane')):
            mirrored = fx.decad_mirrored_copy(source, fx.decad_mirror_frame(fx.r3_new_frame(
                fx.Vec(0, 0, 0), fx.Vec(1, 0, 0), fx.Vec(0, 1, 0))), name='Mirror', body='Mirrored')
        self.assertFalse(mirrored.body == source.body)

    def test_a_profile_entity_equals_the_line_drawn_for_it(self):
        fx = self.fx
        entity = Entity()
        curve = SimpleNamespace(sketchEntity=entity.again())
        profile = fx.Profile(SimpleNamespace(profileLoops=[SimpleNamespace(isOuter=True, profileCurves=[curve])]))
        line, other = fx.Line(None, None), fx.Line(None, None)
        line.fusion, other.fusion = entity, Entity()
        self.assertTrue(profile.Entities[0] == line)
        self.assertFalse(profile.Entities[0] == other)

    def test_the_display_settle_skips_the_sketches_that_existed_before(self):
        misc = importlib.import_module('addin.lib.geargen.misc')
        old, new = Entity(), Entity()
        for sketch in (old, new):
            sketch.isComputeDeferred = False
            sketch.isFullyConstrained = True
        design = SimpleNamespace(allComponents=[SimpleNamespace(sketches=[old])])
        known = misc.existing_sketches(design)
        design.allComponents[0].sketches = [old.again(), new]
        self.assertEqual(1, misc.settle_sketch_display(design, known))


if __name__ == '__main__':
    unittest.main()
