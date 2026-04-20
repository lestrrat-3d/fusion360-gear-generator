from .spurgear import *
from .misc import *

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
        thickness = self.getParameter('Thickness')
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        super().processInputs(inputs)
        (helixAngle, ok) = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        if not ok:
            raise Exception('Invalid helix angle value')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')

    # Offset of the construction plane used for the twisted top profile,
    # relative to the base plane. Herringbone halves this so the mirror plane
    # sits in the middle of the gear body.
    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        return self.getParameterAsValueInput('Thickness')

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

        def findProfile(profiles):
            for profile in profiles:
                for loop in profile.profileLoops:
                    if loop.profileCurves.count == 6:
                        return profile
            return None

        bottomToothProfile = findProfile(bottomSketch.profiles)
        topToothProfile = findProfile(topSketch.profiles)
        if not bottomToothProfile or not topToothProfile:
            raise Exception('could not find tooth profile for loft')

        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
