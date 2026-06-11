import math
import adsk.core, adsk.fusion
from .base import GenerationContext, get_value
from .utilities import find_profile_by_curve_counts
from .spurgear import (
    PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
    SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
    SpurGearGenerator, SpurGearInvoluteToothDesignGenerator,
)

PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'

class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        super().configure(cmd)
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE,
            'Helix Angle',
            'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)),
        )

class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)

class HelicalGearGenerator(SpurGearGenerator):
    def newContext(self):
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    # Offset of the construction plane used for the twisted top profile,
    # relative to the base plane. Herringbone halves this so the mirror plane
    # sits in the middle of the gear body.
    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def chamferWantEdges(self):
        return 4

    def buildSketches(self, ctx: GenerationContext):
        super().buildSketches(ctx)

        # The bottom profile has already been drawn by the base class.
        # Now draw a twisted profile on a plane offset from the base plane
        # so the tooth can be built with a loft.
        constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
        ctx.helixPlane = plane

        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE).value
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=helixAngle)
        ctx.twistedGearProfileSketch = loftSketch

    def buildTooth(self, ctx: GenerationContext):
        self.loftTooth(ctx)
        self.chamferTooth(ctx)

    def loftTooth(self, ctx: GenerationContext):
        bottomSketch = ctx.gearProfileSketch
        topSketch = ctx.twistedGearProfileSketch

        lofts = self.getComponent().features.loftFeatures

        bottomToothProfile = find_profile_by_curve_counts(
            bottomSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            topSketch, nurbs=2, arcs=2, lines=2)

        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
