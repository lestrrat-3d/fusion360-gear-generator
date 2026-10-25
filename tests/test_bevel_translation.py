#!/usr/bin/env python3
"""The translated bevel pair runs against a fake Fusion in the order the gallery builds it.

The add-ins run only inside Fusion, so this cannot show that a Fusion call is right. It runs the whole translated
BuildPair for a straight pair, with the runtime's pure parts real (the parameter table, the containers, the vectors,
the number helpers) and the parts that touch Fusion faked, and requires the gallery's sketches, features and
containers in the gallery's order. A call the generated code makes that the runtime does not define fails here, and
so does a change that reorders the build.
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


class FakeLine:
    def __init__(self, construction=False):
        self.construction = construction

    def set_construction(self, flag):
        self.construction = flag


class FakeProfile:
    Valid = True

    def __init__(self, lines=(), outer=0):
        self.Entities = list(lines)
        self.Outer = [object()] * outer
        self.Holes = []


class FakeSketch:
    def __init__(self, log, name):
        self.log, self.name = log, name
        self.lines = []
        self.points = 0
        self.container = None

    def origin(self):
        return SimpleNamespace(origin=True)

    def dimension_point(self, x, y):
        self.points += 1
        return SimpleNamespace(x=x, y=y)

    dimension_arc_end = dimension_point

    def create_line(self, start, end):
        line = FakeLine()
        self.lines.append(line)
        return line

    def create_arc(self, center, start, end):
        pass

    def create_fit_spline(self, *points):
        assert all(p is not None for p in points), self.name + ': a spline point is missing'

    def create_circle(self, center, radius):
        return SimpleNamespace(Center=center)

    def add_constraint(self, constraint):
        pass

    def solve(self):
        self.log.append(('sketch', self.name, self.points))

    def profiles(self):
        if self.name == 'Gear Profiles':
            drawn = [line for line in self.lines if not line.construction]
            half = len(drawn) // 2
            return [FakeProfile(drawn[:half]), FakeProfile(drawn[half:])]
        if self.name == 'Tooth Profile':
            return [FakeProfile(outer=6)]
        return [FakeProfile()]


class FakeBody:
    def __init__(self, name):
        self.name = name
        self.container = None

    def bounds(self):
        return SimpleNamespace(Min=SimpleNamespace(X=-10.0, Y=-10.0), Max=SimpleNamespace(X=10.0, Y=10.0))


class FakeRuntime:
    """The gallery's Document and World, with the real parameter table and containers."""

    def __init__(self, fx):
        self.fx, self.log = fx, []
        self._table = fx.ParamTable()
        self._root = fx.Container(self, 'Root')

    def params(self):
        return self._table

    def param(self, name):
        return self._table.get_value(name)

    def root(self):
        return self._root

    def move_body(self, body, container):
        if body.container is not None:
            body.container.bodies.remove(body)
        container.bodies.append(body)
        body.container = container

    def container_of(self, body):
        return body.container if body.container is not None else self._root

    def xy(self):
        return 'xy'

    def create_sketch(self, plane, name):
        return FakeSketch(self.log, name)

    def create_plane_from_frame(self, frame, name):
        self.log.append(('plane', name))
        return SimpleNamespace(container=None)


class BevelTranslationTest(unittest.TestCase):
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
        cls.bevel = importlib.import_module('addin.lib.geargen.bevelgear')
        cls.fx = importlib.import_module('addin.lib.geargen.decadfusion')

    @classmethod
    def tearDownClass(cls):
        for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
            del sys.modules[name]
        sys.modules.update(cls.saved)

    def run_pair(self, **changes):
        bevel, fx = self.bevel, self.fx
        runtime = FakeRuntime(fx)
        log = runtime.log
        counter = {}

        def feature(kind, make=None):
            def run(*args, name=None, body=None, **kwargs):
                log.append((kind, name))
                return make(name, body, *args) if make else args[0]
            return run

        def body_of(name, body, *args):
            counter[name] = counter.get(name, 0) + 1
            return FakeBody(body or name)

        def pattern_copies(tooth, pattern, name):
            log.append(('pattern', name, pattern.Count))
            return [FakeBody('Tooth Copy') for _ in range(pattern.Count - 1)]

        namespace = SimpleNamespace(**{n: getattr(fx, n) for n in (
            'param_new', 'shaft_snap', 'field_ref', 'math_round', 'math_ilogb', 'r3_new_vec', 'r3_vec', 'r3_new_frame', 'r3_rotation',
            'units_radians', 'units_millimeters', 'UNITS_MILLIMETER', 'UNITS_RADIAN', 'UNITS_ONE', 'DECAD_ALONG',
            'decad_distance', 'decad_circular_pattern', 'decad_with_matching_faces', 'decad_full_revolution',
            'decad_sketch_line', 'decad_point2')})
        namespace.sketch_new_world = lambda: runtime
        namespace.sketch_new_diameter = lambda circle, d: SimpleNamespace()
        namespace.sketch_new_radius = lambda circle, r: SimpleNamespace()
        namespace.sketchcheck_verify_fully_constrained = lambda s: None
        namespace.decad_revolve = feature('revolve', body_of)
        namespace.decad_loft_from_point = feature('loft', lambda name, body, *a: FakeBody(body))
        namespace.decad_extrude = feature('extrude', lambda name, body, *a: FakeBody(body))
        namespace.decad_cut = feature('cut')
        namespace.decad_intersect = feature('intersect')
        namespace.decad_union = feature('union')
        namespace.decad_transform = feature('transform')
        namespace.decad_pattern_copies = pattern_copies
        namespace.select_embedded_tooth_profile = lambda profiles, root: profiles[0]
        namespace.placement_place = lambda body, frame: body
        sketch_parameters = dict(
            Module=1.0, Teeth=31, MateTeeth=31, ShaftAngle=1.5707963267948966, FaceWidth=0.0,
            PressureAngle=0.3490658503988659, Samples=15, EnableBore=True, BoreDiameter=0.0, MateBoreDiameter=0.0,
            SpiralAngle=0.0, SpiralHand='right', CutterRadius=0.0, ToothSpacing=0.0, ToeExtension=0.0,
            ToeRadius=0.0, MateToeRadius=0.0, BaseHeight=0.0, MateBaseHeight=0.0, Placement=None)
        sketch_parameters.update(changes)
        saved = bevel._fx, bevel._spur
        bevel._fx = namespace
        bevel._spur = SimpleNamespace(involute_flanks=self.involute)
        try:
            pair = bevel.build_pair(runtime, bevel._Params(**sketch_parameters))
        finally:
            bevel._fx, bevel._spur = saved
        return runtime, log, pair

    @staticmethod
    def involute(pitch, base, tip, teeth, samples, angle):
        import math
        left = [(base + (tip - base) * i / (samples - 1), 0.1 * i) for i in range(samples)]
        right = [(x, -y) for x, y in left]
        return left, right

    def test_a_straight_pair_builds_in_the_gallerys_order(self):
        runtime, log, pair = self.run_pair()
        sketches = [entry[1] for entry in log if entry[0] == 'sketch']
        # The shared sketch comes first, and once; then the pinion's parts, then the driving gear's.
        self.assertEqual('Gear Profiles', sketches[0])
        self.assertEqual(1, sketches.count('Gear Profiles'))
        self.assertEqual(2, sum(1 for entry in log if entry[0] == 'revolve' and entry[1].startswith('Revolve ')
                                and entry[1] in ('Revolve driving', 'Revolve pinion')))
        self.assertEqual(2, sketches.count('Bore Profile'))
        self.assertEqual(2, sketches.count('Tooth Profile'))
        self.assertEqual(2, [e[0] for e in log].count('pattern'))
        self.assertEqual([31, 31], [e[2] for e in log if e[0] == 'pattern'])
        # Each member trims its tooth, joins the teeth and cuts its bore.
        kinds = [e[0] for e in log if e[0] in ('loft', 'cut', 'intersect', 'union', 'pattern', 'extrude')]
        member = ['loft', 'cut', 'intersect', 'pattern', 'union', 'extrude', 'cut']
        # cut, intersect trims run after the cones are revolved, so keep only the order of the member's own steps.
        self.assertEqual(member, kinds[:len(member)])

    def test_the_pair_sits_in_containers_named_as_the_gallery_names_them(self):
        runtime, log, pair = self.run_pair()
        pair_container = runtime.root().children()[0]
        self.assertEqual('Pair', pair_container.name())
        self.assertEqual(['Driving Gear', 'Pinion Gear'], [c.name() for c in pair_container.children()])
        self.assertIs(pair.Driving.container, pair_container.children()[0])
        self.assertIs(pair.Pinion.container, pair_container.children()[1])

    def test_a_second_pair_takes_the_next_container_name(self):
        runtime, log, pair = self.run_pair()
        pair2 = self.bevel._new_pair_containers(runtime)
        self.assertEqual('Pair 2', pair2.pair.name())
        self.assertEqual('Driving Gear', pair2.driving.name())

    def test_an_odd_pinion_turns_by_one_tooth(self):
        runtime, log, pair = self.run_pair(Teeth=16, MateTeeth=15)
        turns = [entry[1] for entry in log if entry[0] == 'transform']
        self.assertEqual(['Pinion Tooth Phase Turn', 'Driving Gear Phase Turn'], turns)
        runtime, log, pair = self.run_pair(Teeth=16, MateTeeth=12)
        self.assertEqual(['Driving Gear Phase Turn'], [e[1] for e in log if e[0] == 'transform'])

    def test_no_bore_skips_the_bore_features(self):
        runtime, log, pair = self.run_pair(EnableBore=False)
        self.assertNotIn('Bore Profile', [e[1] for e in log if e[0] == 'sketch'])

    def test_the_spiral_pair_refuses_until_it_is_translated(self):
        with self.assertRaisesRegex(NotImplementedError, 'Mean Spiral Angle'):
            self.run_pair(SpiralAngle=0.6)

    def test_the_gallerys_validity_checks_raise_their_messages(self):
        with self.assertRaisesRegex(RuntimeError, 'shaft angle must lie between 30 and 150 degrees'):
            self.run_pair(ShaftAngle=0.1)
        with self.assertRaisesRegex(RuntimeError, 'teeth, mate teeth, and samples must be at least 3'):
            self.run_pair(Teeth=2)


if __name__ == '__main__':
    unittest.main()
