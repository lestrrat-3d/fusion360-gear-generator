#!/usr/bin/env python3
"""The runtime reads the anchor the way the bevel pair needs.

A fake Fusion cannot show that a Fusion call is right. It does catch the two defects the first Fusion loads of the bevel
pair found in the runtime's own code: a sketch on a plane that has no anchor of its own lost the runtime's anchor, and
a construction point, which has `geometry` and no `worldGeometry`, was read like a sketch point.
"""
import importlib
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class BevelRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = {n: m for n, m in sys.modules.items() if n == 'adsk' or n.startswith(('adsk.', 'addin'))}
        adsk = types.ModuleType('adsk')
        adsk.__path__ = []
        core = mock.MagicMock(name='adsk.core')
        core.Point3D.create = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        for name, stub in (('core', core), ('fusion', mock.MagicMock(name='adsk.fusion'))):
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

    def vector(self, x, y, z):
        return SimpleNamespace(x=x, y=y, z=z)

    def runtime(self, anchor):
        return self.fx.Runtime(SimpleNamespace(name='component'), 'plane', anchor, None)

    def fusion_sketch(self):
        return SimpleNamespace(
            xDirection=self.vector(1, 0, 0), yDirection=self.vector(0, 1, 0),
            sketchPoints=SimpleNamespace(add=lambda point: SimpleNamespace(origin=True)))

    def test_a_construction_point_is_a_position_in_the_design(self):
        anchor = SimpleNamespace(geometry=self.vector(1.0, 2.0, 3.0))
        runtime = self.runtime(anchor)
        runtime.build_frame(self.fusion_sketch(), self.fx.Vec(0, 0, 1), self.fx.Vec(1, 0, 0))
        origin = runtime.placement.origin()
        self.assertEqual((1.0, 2.0, 3.0), (origin.x, origin.y, origin.z))

    def test_a_sketch_point_is_read_by_its_position_in_the_design(self):
        anchor = SimpleNamespace(worldGeometry=self.vector(4.0, 5.0, 6.0), geometry=self.vector(0.0, 0.0, 0.0))
        runtime = self.runtime(anchor)
        runtime.build_frame(self.fusion_sketch())
        self.assertEqual(4.0, runtime.placement.origin().x)

    def test_a_plane_with_no_anchor_of_its_own_keeps_the_runtimes(self):
        anchor = SimpleNamespace(geometry=self.vector(0.0, 0.0, 0.0))
        runtime = self.runtime(anchor)
        runtime.build_frame(self.fusion_sketch(), self.fx.Vec(0, 0, 1), self.fx.Vec(1, 0, 0))
        plane = runtime.xy()
        self.assertIsNotNone(plane.frame)
        self.assertIsNone(plane.anchor)
        sketch = self.fx.Sketch(runtime, self.fusion_sketch(), plane=plane)
        self.assertIs(anchor, sketch._anchorPoint)

    def test_a_plane_across_the_base_plane_brings_its_own_anchor(self):
        anchor = SimpleNamespace(geometry=self.vector(0.0, 0.0, 0.0))
        runtime = self.runtime(anchor)
        runtime.build_frame(self.fusion_sketch())
        own = object()
        frame = self.fx.r3_new_frame(self.fx.Vec(0, 0, 0), self.fx.Vec(0, 1, 0), self.fx.Vec(0, 0, 1))
        plane = self.fx.Plane(runtime, 'entity', frame=frame, anchor=own)
        sketch = self.fx.Sketch(runtime, self.fusion_sketch(), plane=plane)
        self.assertIs(own, sketch._anchorPoint)


if __name__ == '__main__':
    unittest.main()
