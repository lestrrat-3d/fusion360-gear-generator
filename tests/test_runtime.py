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


class FakeOccurrence:
    def __init__(self, parent_log):
        self.component = SimpleNamespace(name='', occurrences=FakeOccurrences(parent_log))
        self.log = parent_log

    def createForAssemblyContext(self, context):
        return ('proxy', self, context)


class FakeOccurrences:
    def __init__(self, log):
        self.log = log

    def addNewComponent(self, matrix):
        occurrence = FakeOccurrence(self.log)
        self.log.append(occurrence)
        return occurrence


class ContainerTest(RuntimeTestCase):
    """finish_containers makes a component for each container that holds a body and moves the bodies in."""

    def test_each_container_with_a_body_becomes_a_component_and_its_bodies_move_in(self):
        made = []
        self.component.occurrences = FakeOccurrences(made)
        root = self.runtime.root()
        pair = root.new_child('Pair')
        driving, pinion = pair.new_child('Driving Gear'), pair.new_child('Pinion Gear')
        root.new_child('Empty')
        gear = self.body(1.0, container=driving)
        before = gear.body
        self.runtime.finish_containers()
        self.assertEqual(['Pair', 'Driving Gear'], [o.component.name for o in made])
        self.assertIs(made[1], driving.context[1], 'a nested component is reached through its parent occurrence')
        self.assertIsNot(before, gear.body)
        self.assertEqual(driving.context, gear.body.movedTo)
        self.assertIsNone(pinion.context)

    def test_a_refused_move_raises(self):
        self.component.occurrences = FakeOccurrences([])
        container = self.runtime.root().new_child('Gear')
        gear = self.body(container=container)
        gear.body.moveToComponent = lambda target: None
        with self.assertRaises(self.fx.FusionRefused):
            self.runtime.finish_containers()


class LazyBookkeepingTest(RuntimeTestCase):
    """Pending unions and copies give every body the shape the eager sequence of features would."""

    def turn(self, angle=0.5):
        return self.fx.r3_rotation(self.fx.r3_new_vec(1, 0, 0), angle)

    def test_a_union_a_later_union_absorbs_leaves_its_container(self):
        fx = self.fx
        container = fx.Container(self.runtime, 'Gear')
        blank, tooth = self.body(container=container), self.body(1.0, container=container)
        inner = fx.decad_union(blank, tooth, name='Inner')
        outer = fx.decad_union(self.body(2.0), inner, name='Outer')
        self.assertEqual([], container.bodies)
        outer.body
        for held in container.bodies:
            held.body  # finish_containers reads every body it moves

    def test_an_absorbed_union_cannot_be_joined_again(self):
        fx = self.fx
        first = fx.decad_union(self.body(), self.body(1.0), name='U')
        second = fx.decad_union(self.body(2.0), first, name='V')
        with self.assertRaises(fx.ConsumedBody):
            fx.decad_union(second, first, name='W')
        with self.assertRaises(fx.ConsumedBody):
            fx.decad_union(first, self.body(3.0), name='X')

    def test_a_copy_is_made_before_its_source_moves(self):
        fx = self.fx
        source = self.body(1.0)
        copy = fx.decad_copy(source, name='Copy')
        fx.decad_transform(source, self.turn(), name='Move')
        self.assertEqual(['copy', 'move'], self.features.kinds())
        copy.body
        self.assertEqual(['copy', 'move'], self.features.kinds())

    def test_a_pending_union_is_joined_before_its_base_moves(self):
        fx = self.fx
        base = self.body()
        fx.decad_union(base, self.body(1.0), name='Join')
        fx.decad_transform(base, self.turn(), name='Move')
        self.assertEqual(['combine', 'move'], self.features.kinds())

    def test_a_copy_of_a_tool_is_made_before_the_join_consumes_the_tool(self):
        fx = self.fx
        tool = self.body(1.0)
        copy = fx.decad_copy(tool, name='Copy')
        joined = fx.decad_union(self.body(), tool, name='Join')
        joined.body
        self.assertEqual(['copy', 'combine'], self.features.kinds())
        self.assertIsNotNone(copy._body)

    def test_a_copy_asked_for_after_a_union_copies_the_joined_body(self):
        fx = self.fx
        base = self.body()
        fx.decad_union(base, self.body(1.0), name='Join')
        copy = fx.decad_copy(base, name='Copy')
        copy.body
        self.assertEqual(['combine', 'copy'], self.features.kinds())

    def test_every_operation_on_a_body_makes_its_pending_copies_first(self):
        fx = self.fx
        operations = {
            'cut base': lambda b: fx.decad_cut(b, self.body(9.0), name='Cut'),
            'cut tool': lambda b: fx.decad_cut(self.body(9.0), b, name='Cut'),
            'intersect': lambda b: fx.decad_intersect(b, self.body(9.0), name='Intersect'),
            'remove': lambda b: fx.decad_remove(b, name='Remove'),
            'centroid': lambda b: b.centroid(),
            'faces': lambda b: b.faces(),
            'solid': lambda b: b.is_solid(),
        }
        for what, operation in operations.items():
            with self.subTest(operation=what):
                self.log.clear()
                source = self.body(1.0)
                fx.decad_copy(source, name='Copy')
                operation(source)
                self.assertEqual('copy', self.features.kinds()[0])


class PatternCopiesTest(RuntimeTestCase):
    """Copies at equal angles are made by one circular pattern and matched to the instances by angle."""

    def copies(self, source, centre, count):
        fx = self.fx
        return [fx.decad_copy(source, fx.decad_with_copy_rotation(centre, fx.r3_new_vec(1, 0, 0),
                                                                   2 * math.pi * k / count), name='Copy')
                for k in range(1, count)]

    def test_copies_turned_about_another_centre_get_their_own_instances(self):
        fx = self.fx
        centre = fx.Vec(0.0, 5.0, 0.0)
        self.features.pattern_axis = ((0.0, 5.0, 0.0), (1.0, 0.0, 0.0))
        source = self.body(1.0, 8.0, 0.0)
        copies = self.copies(source, centre, 4)
        fx._realize_copies(copies)
        self.assertEqual(['pattern'], self.features.kinds())
        for k, copy in enumerate(copies, start=1):
            c = copy.body.physicalProperties.centerOfMass
            a = 2 * math.pi * k / 4
            want = (1.0, 5.0 + 3.0 * math.cos(a), 3.0 * math.sin(a))
            self.assertAlmostEqual(0.0, math.dist(want, (c.x, c.y, c.z)), places=9)

    def test_a_pattern_with_too_few_instances_is_a_named_error(self):
        fx = self.fx
        source = self.body(1.0, 3.0, 0.0)
        copies = self.copies(source, fx.r3_vec(), 4)
        real = fx._circular_pattern
        with mock.patch.object(fx, '_circular_pattern', lambda *a: real(*a)[:1]):
            with self.assertRaisesRegex(fx.FusionRefused, 'made 1 copies; want 3'):
                fx._realize_copies(copies)

    def test_a_copy_made_on_its_own_turns_about_its_centre(self):
        fx = self.fx
        source = self.body(1.0, 8.0, 0.0)
        copy = fx.decad_copy(source, fx.decad_with_copy_rotation(fx.Vec(0.0, 5.0, 0.0), fx.r3_new_vec(1, 0, 0), 0.5),
                             name='Copy')
        matrix = fx.adsk.core.Matrix3D.create.return_value
        matrix.reset_mock()
        copy.body
        angle, axis, origin = matrix.setToRotation.call_args.args
        self.assertEqual(0.5, angle)
        self.assertEqual((0.0, 5.0, 0.0), (origin.x, origin.y, origin.z))


class ContainerPlacementTest(RuntimeTestCase):
    """A result takes the container of the body or sketch it was made from, as extrude, revolve and copy do."""

    def test_pattern_copies_join_the_container_of_their_tooth(self):
        fx = self.fx
        container = self.runtime.root().new_child('Gear')
        tooth = self.body(1.0, 3.0, 0.0, container=container)
        copies = fx.decad_pattern_copies(
            tooth, fx.decad_circular_pattern(Center=fx.r3_vec(), Axis=fx.r3_new_vec(1, 0, 0), Count=3), name='Pattern')
        self.assertEqual([tooth] + copies, container.bodies)
        self.assertTrue(all(copy.container is container for copy in copies))

    def test_a_mirrored_copy_joins_the_container_of_its_source(self):
        fx = self.fx
        container = self.runtime.root().new_child('Gear')
        source = self.body(1.0, container=container)
        with mock.patch.object(fx.Runtime, 'create_plane_from_frame', return_value=SimpleNamespace(entity='plane')):
            mirrored = fx.decad_mirrored_copy(source, fx.decad_mirror_frame(fx.r3_new_frame(
                fx.Vec(0, 0, 0), fx.Vec(1, 0, 0), fx.Vec(0, 1, 0))), name='Mirror', body='Mirrored')
        self.assertIs(container, mirrored.container)
        self.assertIn(mirrored, container.bodies)

    def test_a_loft_joins_the_container_of_its_lower_sketch(self):
        fx = self.fx
        container = self.runtime.root().new_child('Gear')
        lower = SimpleNamespace(container=container)
        loft = fx.decad_loft(self.runtime, lower, SimpleNamespace(fprofile='a'), SimpleNamespace(container=None),
                             SimpleNamespace(fprofile='b'), name='Loft', body='Tooth')
        self.assertIs(container, loft.container)

    def test_a_loft_through_faces_joins_the_container_of_its_first_section(self):
        fx = self.fx
        container = self.runtime.root().new_child('Gear')
        slab = self.body(1.0, container=container)
        slab.body.faces = ['cap']
        faces = slab.faces()
        loft = fx.decad_loft_faces(self.runtime, *faces, name='Loft', body='Tooth')
        self.assertIs(container, loft.container)


class AxisTest(RuntimeTestCase):
    """A pattern axis with no coaxial cylinder is a construction line in the base plane, or a named error."""

    def setUp(self):
        super().setUp()
        self.patches[0].stop()  # the real _axis_through
        self.patches = self.patches[1:]
        self.component.bRepBodies = []

    def axis(self, center, direction):
        fx = self.fx
        return fx._axis_through(self.runtime, fx.decad_circular_pattern(Center=center, Axis=direction, Count=4), 'Axis')

    def test_an_axis_along_the_gear_normal_with_no_cylinder_is_a_named_error(self):
        fx = self.fx
        with self.assertRaises(fx.Untranslated):
            self.axis(fx.Vec(0, 0, 0), fx.Vec(0, 0, 1))

    def test_an_axis_that_leaves_the_base_plane_is_not_replaced_by_its_projection(self):
        fx = self.fx
        with self.assertRaises(fx.Untranslated):
            self.axis(fx.Vec(0, 0, 0), fx.Vec(1, 0, 1))
        with self.assertRaises(fx.Untranslated):
            self.axis(fx.Vec(0, 0, 2), fx.Vec(1, 0, 0))

    def test_an_axis_in_the_base_plane_is_a_construction_line(self):
        fx = self.fx
        with mock.patch.object(fx.Runtime, 'create_sketch') as create:
            line = self.axis(fx.Vec(0, 0, 0), fx.Vec(1, 0, 0))
        self.assertEqual(1, create.call_count)
        self.assertIsNotNone(line)


if __name__ == '__main__':
    unittest.main()
