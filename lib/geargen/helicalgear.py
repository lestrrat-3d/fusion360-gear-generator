import math
import adsk.core, adsk.fusion
from .spurgear import (
    PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
    SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
    SpurGearGenerator, SpurGearInvoluteToothDesignGenerator,
)
from .base import GenerationContext, get_value
from .utilities import find_profile_by_curve_counts


# --- Helical-specific parameter name / dialog input id (verbatim surface) ---
PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        # Spur adds every base input first (Parent Component last); the Helix
        # Angle value input is appended after, so it necessarily lands LAST in
        # the dialog, after Parent Component ([SPUR-SUBCLASS-INPUT] consequence).
        super().configure(cmd)
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        # The offset plane the twisted top profile is drawn on (also the mirror
        # plane herringbone reflects across).
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The second, "Twisted Gear Profile" sketch: the top loft section.
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    def prefixBase(self):
        return 'HelicalGear'

    def newContext(self):
        return HelicalGearGenerationContext()

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression,
            thickness.expression, helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs):
        # Register HelixAngle from the degree dialog input, stored in radians.
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad',
                          'Helix angle for the helical gear')

    def filletHelixFactorExpression(self):
        # Multiplies the root-fillet radius by cos(HelixAngle) so it reads
        # correctly on the tilted tooth's transverse plane (spur base: '1').
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    def chamferWantEdges(self):
        # [HELI-F-CHAMFER-COUNT] Reproduced verbatim (spur base returns 6).
        return 4

    def helicalPlaneOffset(self):
        # Distinct overridable hook (herringbone re-points it to half-thickness);
        # helical returns the full Thickness.
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: GenerationContext):
        # Draw the bottom Gear Profile + spur tooth at angle 0 (spur steps 3-5).
        super().buildSketches(ctx)

        # [HELI-F-TWIST-PLANE] Offset construction plane for the twisted top
        # profile, on the gear's own component.
        constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
        ctx.helixPlane = plane

        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        # The twist is delivered as the draw() angle: the spur tooth generator
        # rotates the whole tooth by angle ([SPUR-F-ROTATE-CONFIRM]).
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        ctx.twistedGearProfileSketch = loftSketch

    def buildTooth(self, ctx: GenerationContext):
        # Loft (not extrude); must end by calling chamferTooth (spur's boundary).
        self.loftTooth(ctx)
        self.chamferTooth(ctx)

    def loftTooth(self, ctx: GenerationContext):
        # [HELI-F-LOFT] Loft the bottom tooth loop to the top twisted tooth loop.
        # Non-embedded only: both sections use a fixed nurbs=2, arcs=2, lines=2.
        lofts = self.getComponent().features.loftFeatures
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Add the bottom section first, then the top.
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
