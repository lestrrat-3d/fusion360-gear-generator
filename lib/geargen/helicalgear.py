"""Helical gear generator.

Helical is a subclass-only module: it inherits spur's whole build — the Tools sketch
and anchor chain, the Gear Profile sketch, the circular pattern, the root fillets, the
bore and the cleanup — and changes three things. It adds a Helix Angle input, draws a
second "Twisted Gear Profile" sketch on a construction plane offset from the target
plane by the full Thickness, and lofts the bottom profile to that twisted top profile
instead of extruding it.
"""

import math

import adsk.core
import adsk.fusion

from .base import GenerationContext, get_value
from .spurgear import (PARAM_MODULE, PARAM_THICKNESS, PARAM_TOOTH_NUMBER,
                       SpurGearCommandInputsConfigurator,
                       SpurGearGenerationContext,
                       SpurGearGenerator,
                       SpurGearInvoluteToothDesignGenerator)
from .utilities import find_profile_by_curve_counts

PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        # Spur's configure() adds Parent Component last, so Helix Angle necessarily
        # lands after it in the dialog. That is the shipped ordering.
        super().configure(cmd)

        # The default is passed in Fusion internal units, which for an angle is
        # radians, even though the field displays degrees.
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE,
            'Helix Angle',
            'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        # The offset plane the twisted top profile is drawn on, and the plane
        # herringbone later mirrors across.
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The top loft section.
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    def newContext(self) -> HelicalGearGenerationContext:
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        # .expression, never .value, so the units show through in the name.
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression,
            toothNumber.expression,
            thickness.expression,
            helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        # The dialog is in degrees; the parameter is registered in radians. This hook
        # runs before the derived parameters, which reference HelixAngle.
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad',
                          'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        # Spliced in as the last factor of the FilletRadius expression, which makes the
        # root fillet read correctly on the transverse plane of a tilted tooth.
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        # Its own hook, not inlined: herringbone re-points it at half the thickness so
        # its mirror plane lands mid-body. The value is a numeric snapshot of Thickness
        # as it stood at generation time, not a live parameter reference.
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: GenerationContext):
        # Draws the bottom Gear Profile and runs the spur tooth generator at angle 0.
        super().buildSketches(ctx)

        component: adsk.fusion.Component = self.getComponent()
        constructionPlaneInput = component.constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        ctx.helixPlane = component.constructionPlanes.add(constructionPlaneInput)

        # createSketchObject() returns a hidden sketch, and nothing ever shows this one:
        # reading its profiles for the loft works while it stays hidden.
        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=ctx.helixPlane)
        ctx.twistedGearProfileSketch = loftSketch

        # The twist is delivered as draw()'s angle argument and nothing else: the spur
        # tooth generator rotates the tooth in its own point math and confirms the
        # rotation with the spine's angular dimension. The angle is a raw .value, which
        # is radians. The anchor keeps this sketch on the same projection chain as every
        # other sketch, and draw() does the anchoring itself.
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint,
            angle=self.getParameter(PARAM_HELIX_ANGLE).value)

    def buildTooth(self, ctx: GenerationContext):
        # Replaces spur's tooth extrude. chamferTooth is the last action, because
        # buildMainGearBody relies on this method's boundary and does not chamfer
        # separately.
        self.loftTooth(ctx)
        self.chamferTooth(ctx)

    def loftTooth(self, ctx: GenerationContext):
        # Both searches use the fixed non-embedded six-curve key: 2 NURBS flanks,
        # 2 arcs, 2 flank-to-root lines. There is no embedded branch, so an embedded
        # low-tooth-count gear finds no profile here.
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)

        component: adsk.fusion.Component = self.getComponent()
        lofts = component.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Bottom section first, top second: loftSections.add order is the loft order,
        # and reversing it turns a right-hand helix into a left-hand one.
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)

        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'

    def chamferWantEdges(self) -> int:
        return 4
