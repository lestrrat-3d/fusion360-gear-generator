from .helicalgear import *

class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator): pass

class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    def __init__(self):
        super().__init__()

class HerringboneGearGenerator(HelicalGearGenerator):
    def newContext(self):
        return HerringboneGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HerringboneGear'

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter('Thickness')
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    # The mirror plane sits in the middle of the body, so the loft only covers
    # half the thickness.
    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        thickness = self.getParameter('Thickness').value
        return adsk.core.ValueInput.createByReal(thickness / 2)

    def buildTooth(self, ctx: GenerationContext):
        self.loftTooth(ctx)

        # Mirror the lofted tooth across the helix plane to form the other half.
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
        mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
        mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'

        # Combine the mirrored half into the original tooth body so the pattern
        # and combine steps in the base class operate on a single body.
        entities = adsk.core.ObjectCollection.create()
        entities.add(mirrorResult.bodies.item(0))
        combineInput = self.getComponent().features.combineFeatures.createInput(
            self.getComponent().bRepBodies.itemByName('Tooth Body'),
            entities,
        )
        self.getComponent().features.combineFeatures.add(combineInput)

        self.chamferTooth(ctx)
