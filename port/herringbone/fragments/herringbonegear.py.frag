import adsk.core
import adsk.fusion
from .base import GenerationContext
from .spurgear import PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS
from .helicalgear import (
    PARAM_HELIX_ANGLE,
    HelicalGearCommandConfigurator,
    HelicalGearGenerationContext,
    HelicalGearGenerator,
)


class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator):
    pass


class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    pass


class HerringboneGearGenerator(HelicalGearGenerator):
    def newContext(self) -> HerringboneGearGenerationContext:
        return HerringboneGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HerringboneGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    def helicalPlaneOffset(self):
        thickness = self.getParameter(PARAM_THICKNESS).value
        return adsk.core.ValueInput.createByReal(thickness / 2)

    def buildTooth(self, ctx: GenerationContext):
        assert isinstance(ctx, HerringboneGearGenerationContext)
        self.loftTooth(ctx)

        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
        mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
        mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'

        entities = adsk.core.ObjectCollection.create()
        entities.add(mirrorResult.bodies.item(0))
        combineInput = self.getComponent().features.combineFeatures.createInput(
            self.getComponent().bRepBodies.itemByName('Tooth Body'),
            entities,
        )
        self.getComponent().features.combineFeatures.add(combineInput)
