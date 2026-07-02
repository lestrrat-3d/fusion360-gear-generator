import adsk.core, adsk.fusion
from .base import GenerationContext
from .spurgear import PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS
from .helicalgear import (
    PARAM_HELIX_ANGLE,
    HelicalGearCommandConfigurator,
    HelicalGearGenerationContext,
    HelicalGearGenerator,
)


class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator):
    # Herringbone's dialog is exactly helical's (Helix Angle + inherited spur
    # inputs); no new input is added.
    pass


class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    # Same context fields as helical; ctx.helixPlane is now the mid-body mirror
    # plane (a consequence of the half-thickness helicalPlaneOffset override).
    pass


class HerringboneGearGenerator(HelicalGearGenerator):
    def newContext(self):
        return HerringboneGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HerringboneGear'

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    # Place the twisted-profile plane at mid-body (half thickness) instead of
    # the far face, so ctx.helixPlane becomes the mirror plane and the loft
    # spans the bottom half while the mirror completes the top half. This is a
    # raw computed offset, not a parameter reference.
    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        thickness = self.getParameter(PARAM_THICKNESS).value
        return adsk.core.ValueInput.createByReal(thickness / 2)

    def buildTooth(self, ctx: GenerationContext):
        self.loftTooth(ctx)

        # Mirror the lofted half across the mid-body helix plane to form the other half.
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
        mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
        mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'

        # Combine the mirrored half into the original so the pattern/combine steps in the
        # inherited spur pipeline operate on a single body.
        entities = adsk.core.ObjectCollection.create()
        entities.add(mirrorResult.bodies.item(0))
        combineInput = self.getComponent().features.combineFeatures.createInput(
            self.getComponent().bRepBodies.itemByName('Tooth Body'),
            entities,
        )
        self.getComponent().features.combineFeatures.add(combineInput)

        self.chamferTooth(ctx)
