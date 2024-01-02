from .helicalgear import *

class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    def __init__(self):
        super().__init__()

class HerringboneGearGenerator(HelicalGearGenerator):
    def __init__(self, component: adsk.fusion.Component):
        super().__init__(component)

    def newContext(self):
        return HerringboneGearGenerationContext()
    
    def generateName(self, spec):
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(spec.module, spec.toothNumber, spec.thickness, math.degrees(spec.helixAngle))

    def helicalPlaneOffset(self, spec: HelicalGearSpecification):
        return super().helicalPlaneOffset(spec) / 2

    def buildTooth(self, ctx: GenerationContext, spec: SpurGearSpecification):
        self.loftTooth(ctx, spec)

        # mirror the single tooth
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        input = self.component.features.mirrorFeatures.createInput(entities, ctx.helixPlane)
        mirrorResult = self.component.features.mirrorFeatures.add(input)
        mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'

        entities = adsk.core.ObjectCollection.create()
        entities.add(mirrorResult.bodies.item(0))
        combineInput = self.component.features.combineFeatures.createInput(
            self.component.bRepBodies.itemByName('Tooth Body'),
            entities,
        )
        self.component.features.combineFeatures.add(combineInput)
        if spec.chamferTooth > 0:
            self.chamferTooth(ctx, spec)

class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator): pass