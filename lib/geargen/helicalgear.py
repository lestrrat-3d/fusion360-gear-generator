"""Helical gear generator, emitted from spec/helicalgear/steps.md.

Helical is a thin specialization of the spur gear: it subclasses spur's three
classes, inherits the whole spur build pipeline, and changes three things — one
extra dialog input (Helix Angle), a second twisted profile sketch on an offset
construction plane, and a loft in place of spur's tooth extrude (steps H2, H7,
H8, H9).

Everything else — processInputs, prepareTools, buildMainGearBody, buildBody,
patternTeeth, createFillets, buildBore, chamferTeeth, cleanup and the whole of
SpurGearInvoluteToothDesignGenerator — is spur's code, inherited unchanged and
never re-implemented here (step H10).

The [GO]-tagged steps H8 and H9 are transliterated from proof/helicalgear/, not
re-derived. Lengths in Fusion's internal units are cm and angles are radians.
"""

import math

import adsk.core, adsk.fusion

from .base import GenerationContext, get_value
from .spurgear import (
    PARAM_MODULE,
    PARAM_TOOTH_NUMBER,
    PARAM_THICKNESS,
    SpurGearCommandInputsConfigurator,
    SpurGearGenerationContext,
    SpurGearGenerator,
    SpurGearInvoluteToothDesignGenerator,
)
from .utilities import find_profile_by_curve_counts

# Step H1. Public API: herringbonegear.py imports both of these by name
# ([SPUR-EXPORTED-CONSTANTS]).
PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    """Step H2: spur's dialog plus the Helix Angle value input."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        # The configurator extension seam [SPUR-SUBCLASS-INPUT]: spur's inputs
        # first, then this one appended. Spur adds Parent Component last, so
        # Helix Angle lands after it in the dialog — the current behaviour,
        # reproduced exactly.
        super().configure(cmd)
        # 'deg' is the display unit; the default is passed in Fusion's internal
        # unit for an angle, which is radians ([PB-DIALOG-DEFAULT-UNITS]). A
        # bare 14.5 would ship a 14.5-radian default in a field labelled
        # degrees.
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    """Step H3: every spur context field, inherited unchanged, plus two."""

    def __init__(self):
        super().__init__()
        # The offset construction plane the twisted top profile is drawn on,
        # and the plane herringbone later mirrors across (step H7).
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The second sketch, the top loft section (step H8).
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    """Spur's generator with the helical deltas: steps H4 to H9."""

    def newContext(self):
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        # The parameters' .expression strings, not .value, so units show
        # through (step H4).
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        """Step H5: register HelixAngle on spur's [SPUR-EXTRA-PARAMS] hook,
        which processInputs calls between the input-sourced parameters and the
        derived ones — so HelixAngle exists before FilletRadius refers to it."""
        # The input was declared with addValueInput, so it is read with
        # get_value ([PB-INPUT-READ]); what comes back is a ValueInput fit to
        # pass straight to addParameter, and it raises on an invalid expression
        # ([PB-GET-VALUE-CONTRACT]). The dialog is in degrees, the parameter is
        # registered in radians, and the sign is the hand of the helix.
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad',
                          'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        """Step H6: the last factor of spur's live FilletRadius expression,
        where the spur base returns '1'. An expression string, spliced in by
        registerDerivedParameters; createFillets reads only the resulting
        parameter's numeric value ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT])."""
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        """Step H7: the full Thickness, as a ValueInput.

        Its own overridable hook on purpose: herringbone re-points this one
        method at half the thickness so its mirror plane lands mid-body.
        getParameterAsValueInput returns ValueInput.createByReal(param.value),
        a numeric snapshot of Thickness at generation time rather than a live
        parameter reference ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT]).
        """
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: GenerationContext):
        """Steps H7 and H8: spur's bottom Gear Profile, then the offset helix
        plane and the twisted top section drawn on it ([HELI-F-TWIST-PLANE])."""
        # Spur draws the bottom Gear Profile and runs the tooth generator at
        # angle 0.
        super().buildSketches(ctx)

        # Step H7: one construction plane, one timeline entry. The offset is a
        # ValueInput, never a bare number, which is a runtime TypeError
        # ([PB-CONSTRUCTION-PLANES]).
        component: adsk.fusion.Component = self.getComponent()
        constructionPlaneInput = component.constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        ctx.helixPlane = component.constructionPlanes.add(constructionPlaneInput)
        # The plane is left lit. Spur's cleanup switches the light bulb off only
        # on the entities spur itself created ([SPUR-F-CLEANUP]) and helical
        # adds no cleanup of its own, so this is a declared, deliberate delta
        # from [PB-HIDE-AFTER-USE] — in SketchOnly mode too.

        # Step H8: the top loft section. createSketchObject leaves it hidden and
        # nothing shows it; reading sketch.profiles for the loft works on it
        # anyway, which is this gear's declared exception to [PB-HIDE-AFTER-USE].
        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=ctx.helixPlane)
        # The twist is delivered as the tooth generator's own draw() angle, read
        # as a raw .value in radians: the generator draws the whole tooth already
        # rotated in its point math and then confirms that rotation with the
        # spine's angular dimension ([SPUR-F-ROTATE-CONFIRM], [SPUR-F-SPINE]).
        # Drawing the tooth flat and rotating the sketch geometry afterwards
        # leaves the spine dimension measuring the unrotated angle and lets
        # Fusion settle the tooth about 180 degrees away.
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        # That single call is the whole of this sketch: the tooth-top arc
        # ([SPUR-F-TOOTHTOP-ARC]), the rib chain ([SPUR-F-RIBS]), the
        # flank-to-root stubs ([SPUR-F-FLANK-ROOT]), the shared adjacencies that
        # close both loops ([SPUR-F-SHARED-ADJACENCY]) and the anchoring
        # ([SPUR-F-ANCHOR-CHAIN], [SPUR-F-LOCAL-ORIGIN]) are all the generator's
        # own construction. Helical adds no constraint of its own here.
        ctx.twistedGearProfileSketch = loftSketch

    def buildTooth(self, ctx: GenerationContext):
        """Step H9: helical lofts where spur extrudes. No extrude, no chamfer."""
        self.loftTooth(ctx)

    def loftTooth(self, ctx: GenerationContext):
        """Loft the bottom tooth loop to the twisted top one ([HELI-F-LOFT],
        [PB-LOFT]): createInput(operation), one loftSections.add per section in
        loft order, then add(input)."""
        component: adsk.fusion.Component = self.getComponent()
        lofts = component.features.loftFeatures
        # Each section is found by the count and type of the curves on its loop,
        # never by index, and the helper raises rather than falling back to a
        # wrong profile ([PB-PROFILE-MATCH]). Both searches pass the fixed
        # six-curve non-embedded tooth: this implementation never reads
        # ctx.toothProfileIsEmbedded and has no embedded branch, so an embedded
        # low-tooth-count gear, whose tooth loop carries no lines at all, finds
        # no profile. That is the current behaviour and a documented limitation.
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Bottom section first, top second. The order is not cosmetic: the ruled
        # walls are built outward from the section the loft starts at, so
        # swapping the two builds a different solid.
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
