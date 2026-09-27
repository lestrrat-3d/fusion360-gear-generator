import math
import adsk.core, adsk.fusion
from .base import get_value
from .spurgear import (
    PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
    SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
    SpurGearGenerator, SpurGearInvoluteToothDesignGenerator,
)
from .utilities import find_profile_by_curve_counts


PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'

class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        super().configure(cmd)
        inputs: adsk.core.CommandInputs = cmd.commandInputs
        inputs.addValueInput(INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
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
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs):
        self.addParameter(
            PARAM_HELIX_ANGLE, get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad'),
            'rad', 'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    def helicalPlaneOffset(self):
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        super().buildSketches(ctx)
        constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
        ctx.helixPlane = plane
        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        toothGenerator = SpurGearInvoluteToothDesignGenerator(loftSketch, self)
        toothGenerator.draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        ctx.twistedGearProfileSketch = loftSketch

    def buildTooth(self, ctx: SpurGearGenerationContext):
        self.loftTooth(ctx)

    def loftTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        lofts = self.getComponent().features.loftFeatures
        bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
