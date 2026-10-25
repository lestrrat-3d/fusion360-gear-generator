#!/usr/bin/env python3
"""The Fusion runtime that draws the gallery's tooth section behaves as the Fusion probes showed.

A fake sketch stands in for Fusion. At tooth angles 0 and 90 degrees the two flanks are mirror images, and in the
probes Fusion refused one distance dimension that repeated an earlier one from the same origin point (the first
point of the second flank) and accepted the rest. The reason is not understood, so the fake refuses the first
repeated value it sees and nothing else. This cannot show that a Fusion call is right; it shows that the runtime
draws every curve, dimensions every point once per coordinate it owns, and aligns two points where Fusion refuses.
"""
import collections
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
H, V = 'horizontal', 'vertical'


class FakePoint:
    def __init__(self, x, y):
        self.geometry = SimpleNamespace(x=x, y=y)


class FakePointList:
    def __init__(self, points):
        self.points = points
        self.count = len(points)

    def item(self, i):
        return self.points[i]


class FakeSketch:
    def __init__(self):
        self.name = 'Fake Sketch'
        self.isComputeDeferred = False
        self.constraints = []
        self.dimensions = []
        self.lines = []
        self.arcs = []
        self.splines = []
        self.origin = FakePoint(0.0, 0.0)
        self.refused = False
        sketch = self
        point_of = lambda e: e if isinstance(e, FakePoint) else FakePoint(e.x, e.y)

        class Splines:
            def add(self, collection):
                fits = FakePointList([FakePoint(p.x, p.y) for p in collection.items])
                sketch.splines.append(fits)
                return SimpleNamespace(fitPoints=fits)

        class Lines:
            def addByTwoPoints(self, a, b):
                line = SimpleNamespace(startSketchPoint=point_of(a), endSketchPoint=point_of(b))
                sketch.lines.append(line)
                return line

        class Arcs:
            def addByCenterStartEnd(self, center, start, end):
                arc = SimpleNamespace(centerSketchPoint=FakePoint(0.0, 0.0), center=center, start=start, end=end)
                sketch.arcs.append(arc)
                return arc

        class Constraints:
            def addCoincident(self, a, b):
                sketch.constraints.append(('coincident', a, b))

            def addVerticalPoints(self, a, b):
                sketch.constraints.append(('vertical', a, b))

            def addHorizontalPoints(self, a, b):
                sketch.constraints.append(('horizontal', a, b))

        class Dimensions:
            def addDistanceDimension(self, origin, point, orientation, text):
                g = point.geometry
                value = g.x if orientation == H else g.y
                key = (orientation, round(value, 6))
                if not sketch.refused and any(d[0] == key for d in sketch.dimensions):
                    sketch.refused = True
                    raise RuntimeError('3 : VCS_SKETCH_OVER_CONSTRAINTS')
                sketch.dimensions.append((key, point))

        self.sketchCurves = SimpleNamespace(
            sketchFittedSplines=Splines(), sketchLines=Lines(), sketchArcs=Arcs())
        self.geometricConstraints = Constraints()
        self.sketchDimensions = Dimensions()


class ToothSketchBuilderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = {n: m for n, m in sys.modules.items() if n == 'adsk' or n.startswith(('adsk.', 'addin'))}
        adsk = types.ModuleType('adsk')
        adsk.__path__ = []
        core, fusion = mock.MagicMock(name='adsk.core'), mock.MagicMock(name='adsk.fusion')
        core.Point3D.create = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        core.ObjectCollection.create = lambda: SimpleNamespace(
            items=[], add=lambda self_item=None: None)
        fusion.DimensionOrientations.HorizontalDimensionOrientation = H
        fusion.DimensionOrientations.VerticalDimensionOrientation = V
        for name, stub in (('core', core), ('fusion', fusion)):
            sys.modules[f'adsk.{name}'] = stub
            setattr(adsk, name, stub)
        sys.modules['adsk'] = adsk

        def make_collection():
            items = []
            return SimpleNamespace(items=items, add=items.append)
        core.ObjectCollection.create = make_collection

        spec = importlib.util.spec_from_file_location(
            'addin', ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
        package = importlib.util.module_from_spec(spec)
        sys.modules['addin'] = package
        cls.spur = importlib.import_module('addin.lib.geargen.spurgear')
        cls.fx = importlib.import_module('addin.lib.geargen.decadfusion')

    @classmethod
    def tearDownClass(cls):
        for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
            del sys.modules[name]
        sys.modules.update(cls.saved)

    def draw(self, teeth, angle, samples=15):
        spur = self.spur
        module = 0.2
        pitch = module * teeth / 2
        base = pitch * math.cos(math.radians(20))
        root, tip = pitch - 1.25 * module, pitch + module
        sketch = FakeSketch()
        circle = object()
        builder = self.fx.Sketch(None, sketch, sketch.origin, circle, root)
        params = collections.namedtuple('Params', ['Teeth', 'Samples'])(teeth, samples)
        spur._draw_tooth_section(builder, params, spur._Dimensions(pitch, base, root, tip, 0.0), angle)
        builder.finish()
        return sketch, circle

    def test_draws_two_splines_two_lines_and_one_tip_arc(self):
        sketch, circle = self.draw(17, 0.0)
        self.assertEqual(2, len(sketch.splines))
        self.assertEqual(2, len(sketch.lines))
        self.assertEqual(1, len(sketch.arcs), 'the root arc is the root circle')
        self.assertIn(('coincident', sketch.arcs[0].centerSketchPoint, sketch.origin), sketch.constraints)

    def test_root_points_sit_on_the_root_circle(self):
        sketch, circle = self.draw(17, 0.0)
        on_circle = [c for c in sketch.constraints if c[0] == 'coincident' and c[2] is circle]
        self.assertEqual(2, len(on_circle))

    def test_every_point_is_dimensioned_or_aligned(self):
        for angle in (0.0, math.radians(14.5), math.radians(90.0), -0.4):
            with self.subTest(angle=angle):
                sketch, _ = self.draw(17, angle)
                aligned = [c for c in sketch.constraints if c[0] in ('vertical', 'horizontal')]
                # 2n flank points and two root points; the last left flank point and both root
                # points get one dimension, the others two, less one per alignment.
                n = 15
                self.assertEqual(2 * (2 * n - 1) + 3 - len(aligned), len(sketch.dimensions))

    def test_alignment_replaces_a_refused_dimension_only_at_right_angles(self):
        for angle, aligned in ((0.0, 1), (math.radians(90.0), 1), (math.radians(14.5), 0), (1e-4, 0)):
            with self.subTest(angle=angle):
                sketch, _ = self.draw(17, angle)
                got = [c for c in sketch.constraints if c[0] in ('vertical', 'horizontal')]
                self.assertEqual(aligned, len(got))

    def test_a_refusal_with_no_partner_is_raised(self):
        spur = self.spur
        sketch = FakeSketch()
        builder = self.fx.Sketch(None, sketch, sketch.origin, object(), 1.0)
        first = self.fx.PendingPoint(1.0, 2.0, False)
        first.point = FakePoint(1.0, 2.0)
        sketch.sketchDimensions.addDistanceDimension(sketch.origin, first.point, H, None)
        second = self.fx.PendingPoint(5.0, 2.0, False)
        second.point = FakePoint(1.0, 2.0)  # the fake refuses it, but no earlier point shares its x
        with self.assertRaises(RuntimeError):
            builder._dimension(second, H)


if __name__ == '__main__':
    unittest.main()
