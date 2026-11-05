import math
import adsk.core
import adsk.fusion
from .base import get_value
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
    def configure(cls, cmd: adsk.core.Command):
        super().configure(cmd)
        inputs: adsk.core.CommandInputs = cmd.commandInputs
        inputs.addValueInput(INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    def newContext(self) -> SpurGearGenerationContext:
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helix = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression,
            thickness.expression, helix.expression)

    def addExtraPrimaryParameters(self, inputs):
        self.addParameter(
            PARAM_HELIX_ANGLE, get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad'),
            'rad', 'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        return 'cos({})'.format(self.parameterName(PARAM_HELIX_ANGLE))

    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        super().buildSketches(ctx)

        planes: adsk.fusion.ConstructionPlanes = self.getComponent().constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
        offset = self.helicalPlaneOffset()
        planeInput.setByOffset(self.plane, offset)
        ctx.helixPlane = planes.add(planeInput)

        plane: adsk.fusion.ConstructionPlane = ctx.helixPlane
        loftSketch: adsk.fusion.Sketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        helixAngle: float = self.getParameter(PARAM_HELIX_ANGLE).value
        toothGenerator = SpurGearInvoluteToothDesignGenerator(loftSketch, self)
        toothGenerator.draw(ctx.anchorPoint, angle=helixAngle)
        ctx.twistedGearProfileSketch = loftSketch

    def buildTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        self.loftTooth(ctx)

    def loftTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)

        lofts: adsk.fusion.LoftFeatures = self.getComponent().features.loftFeatures
        loftInput: adsk.fusion.LoftFeatureInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult: adsk.fusion.LoftFeature = lofts.add(loftInput)
        toothBody: adsk.fusion.BRepBody = loftResult.bodies.item(0)
        ctx.toothBody = toothBody
        toothBody.name = 'Tooth Body'
