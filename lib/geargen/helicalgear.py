"""Helical gear generator, emitted from the compiled step list spec/helicalgear/steps.md
(H-01..H-11).

Helical is a thin specialization of the spur gear: it subclasses the spur family and reuses the
whole spur build pipeline. Only the deltas live here — the Helix Angle dialog input, the HelixAngle
user parameter, the cos(HelixAngle) factor on the root-fillet radius, the offset construction plane
carrying the twisted top profile, and the loft that replaces spur's tooth extrude. Everything else
(the Tools sketch, the bottom Gear Profile, the body extrude, the circular pattern, the root
fillets, the optional bore, the completed-gear chamfer and the cleanup) is inherited (H-11).

The geometry of the twisted section and of the loft is transliterated from proof/helicalgear/
(sketches_test.go, solids_test.go) for the steps the step list tags [GO]; nothing here is
re-derived.

Units: Fusion's internal units are centimetres for length and radians for angle. A dialog's unit
string controls display and expression parsing only, so the Helix Angle default is written in
internal radians ([PB-DIALOG-DEFAULT-UNITS]).
"""

import math

import adsk.core, adsk.fusion

from .base import get_value
from .spurgear import (
    PARAM_MODULE,
    PARAM_THICKNESS,
    PARAM_TOOTH_NUMBER,
    SpurGearCommandInputsConfigurator,
    SpurGearGenerationContext,
    SpurGearGenerator,
    SpurGearInvoluteToothDesignGenerator,
)
from .utilities import find_profile_by_curve_counts

# H-01: the two exported constants. herringbonegear.py imports PARAM_HELIX_ANGLE by name, so both
# the identifiers and their exact string values are contract surface.
PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    """H-02: spur's dialog plus the Helix Angle input."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        # Every spur input is added first. Spur's configure() puts Parent Component last, so the
        # Helix Angle input below necessarily lands after it. That is the current behavior and it
        # is reproduced exactly ([SPUR-SUBCLASS-INPUT]).
        super().configure(cmd)

        # Displayed in degrees, defaulted in internal radians ([PB-DIALOG-DEFAULT-UNITS]).
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    """H-03: the spur context plus the two entities the helical steps thread through.

    Every spur field is inherited unchanged. Subclasses (herringbone) read both fields below by
    name, so neither may be renamed."""

    def __init__(self):
        super().__init__()
        # The offset construction plane the twisted top profile is drawn on, and the plane
        # herringbone later mirrors across (H-08).
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The second, "Twisted Gear Profile" sketch, which is the top loft section (H-09).
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    """The helical build pipeline: spur's, with the hooks below re-pointed."""

    # --- H-04: identity hooks the spur base calls -----------------------------------------

    def newContext(self) -> HelicalGearGenerationContext:
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        # Rendered from each parameter's .expression, not its .value, so units show through.
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    # --- H-05: the HelixAngle user parameter ----------------------------------------------

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        """[SPUR-EXTRA-PARAMS]: processInputs calls this between the input-sourced parameters and
        the derived ones, which is what lets H-06 name HelixAngle inside a live expression.

        The input was declared with addValueInput, so it is read with get_value, which already
        returns a ValueInput ready for addParameter ([PB-INPUT-READ], [PB-GET-VALUE-CONTRACT]).
        The dialog is in degrees; the parameter is registered in radians.

        The value is signed and the sign is the hand of the helix: a negative value is a left-hand
        helix, the dialog accepts it, and nothing rescales it. No range is enforced."""
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(
            PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')

    # --- H-06: the root fillet on the transverse plane of a tilted tooth ------------------

    def filletHelixFactorExpression(self) -> str:
        """The last factor of the live FilletRadius expression, where the spur base returns '1'.

        An expression string, not a number. registerDerivedParameters splices it into
        (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>; createFillets never calls this
        hook, it reads only the resulting FilletRadius parameter's numeric value."""
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    # --- H-07: where the twisted profile is drawn -----------------------------------------

    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        """The distance from the base plane to the twisted profile's plane. Helical returns the
        full thickness; herringbone re-points this hook to half of it, which is why it stays its
        own method rather than being inlined into buildSketches.

        getParameterAsValueInput wraps the Thickness parameter's value at generation time, so this
        is a numeric snapshot, not a live reference: editing Thickness afterwards does not move the
        plane, the gear is regenerated instead ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT])."""
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    # --- H-08 and H-09: the offset plane and the Twisted Gear Profile sketch ---------------

    def buildSketches(self, ctx: SpurGearGenerationContext):
        """The bottom Gear Profile (inherited), then the offset plane and the twisted top section.

        The parameter annotation matches the inherited signature and the assertion does the
        narrowing; every read and write of ctx.helixPlane / ctx.twistedGearProfileSketch happens
        after it."""
        assert isinstance(ctx, HelicalGearGenerationContext)

        # Draws the bottom Gear Profile and runs the spur tooth generator at angle 0.
        super().buildSketches(ctx)

        # H-08: the offset construction plane, on the gear's own component. The offset argument is
        # a ValueInput, never a bare number ([PB-CONSTRUCTION-PLANES]).
        component: adsk.fusion.Component = self.getComponent()
        constructionPlaneInput = component.constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = component.constructionPlanes.add(constructionPlaneInput)
        # The helix plane is left visible after generation, deliberately: spur's cleanup switches
        # off only the entities spur itself created and helical adds no cleanup, so this plane
        # stays lit, in Generate-Sketches-Only mode too ([HELI-F-TWIST-PLANE]).
        ctx.helixPlane = plane

        # H-09: the twisted top section. The tooth is drawn already rotated by the helix angle,
        # delivered as the tooth generator's own draw() angle in raw radians: the generator
        # rotates the tooth in its point math and then confirms that rotation with the spine's
        # angular dimension ([SPUR-F-ROTATE-CONFIRM], [SPUR-F-SPINE]). Drawing it flat and
        # rotating the sketch geometry afterwards would leave the spine dimension measuring the
        # unrotated angle, and would let Fusion pick the half-turn-off branch that sends the loft
        # through the gear centre. The anchoring happens inside draw() itself.
        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        # This sketch stays hidden its whole life: createSketchObject returns a hidden sketch and
        # nothing ever shows it, not here and not spur's cleanup. Reading profiles off it for the
        # loft works anyway — a declared delta from [PB-HIDE-AFTER-USE] ([HELI-F-TWIST-PLANE]).
        ctx.twistedGearProfileSketch = loftSketch

    # --- H-10: the tooth, lofted rather than extruded -------------------------------------

    def buildTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        self.loftTooth(ctx)

    def loftTooth(self, ctx: SpurGearGenerationContext):
        """Loft the bottom tooth loop to the twisted top one into a new body.

        Non-embedded only: both sections pass a fixed nurbs=2, arcs=2, lines=2, the six-curve
        tooth loop. This implementation does not read ctx.toothProfileIsEmbedded and has no
        embedded branch, so a low-module, high-tooth-count gear whose flank starts inside the root
        circle draws no flank-to-root lines and the profile search finds nothing. That is a
        documented limitation ([HELI-F-LOFT])."""
        assert isinstance(ctx, HelicalGearGenerationContext)

        component: adsk.fusion.Component = self.getComponent()
        lofts = component.features.loftFeatures

        # Profiles are found by the curve-type counts of their loop, never by index; the framework
        # helper raises when nothing matches ([PB-PROFILE-MATCH]).
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)

        # [PB-LOFT]: create the input with the operation, add each section in order, then add the
        # input. The bottom section goes first. Adding them in the other order also lofts a valid
        # solid and does not flip the hand — the same twist, with the same sign, comes back off
        # either body — but it rebuilds the ruled walls from the other section and changes the
        # volume with them, so the order is pinned here rather than left to the implementation.
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)

        toothBody = loftResult.bodies.item(0)
        toothBody.name = 'Tooth Body'
        ctx.toothBody = toothBody
