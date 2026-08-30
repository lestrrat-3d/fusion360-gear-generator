"""Helical gear generator, emitted from spec/helicalgear/steps.md.

Helical is a subclass module: it inherits spur's whole build pipeline and adds
three timeline entries of its own — the helix construction plane (step 8), the
Twisted Gear Profile sketch (step 9) and the loft (step 10). Everything else is
spur's and is inherited unchanged (step 12).

The three added steps are transliterated from proof/helicalgear/
(tooth_test.go, sketches_test.go, solids_test.go), not re-derived. Lengths in
Fusion's internal units are cm; the Helix Angle parameter is in radians.
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

# Public API (step 1): herringbone imports both by name, and the contract
# manifest pins their exact string values.
PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    """Adds the Helix Angle dialog input (step 2)."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        # Spur's inputs first; its configure has already added Parent
        # Component last, so Helix Angle lands after it. That is the current
        # behaviour and it is reproduced deliberately.
        super().configure(cmd)

        # The default is in Fusion's internal unit for an angle
        # ([PB-DIALOG-DEFAULT-UNITS]): a bare createByReal(14.5) would ship a
        # 14.5-radian default that the dialog still renders as degrees.
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    """Spur's build context plus the two fields helical adds (step 3). Both
    names are read from outside this module, so they are public API."""

    def __init__(self):
        super().__init__()
        # The offset construction plane the twisted profile is drawn on, and
        # the plane herringbone later mirrors across.
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The second sketch, the loft's top section.
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    """Spur's generator with the helix angle threaded through it (steps 4-11).
    The call graph and override boundaries are spur's and do not move."""

    def newContext(self):
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        # Spur's rule with HelixAngle appended, reading each parameter's
        # .expression string, not .value, so units show through.
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        """Step 5: register HelixAngle. The inherited processInputs calls this
        between the input-sourced parameters and the derived ones, which is
        what lets step 6's FilletRadius expression reference HelixAngle.

        The dialog is in degrees and the parameter is in radians; get_value
        returns a ValueInput ready to hand straight to addParameter
        ([PB-INPUT-READ], [PB-GET-VALUE-CONTRACT])."""
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad',
                          'Helix angle for the helical gear')

    def filletHelixFactorExpression(self) -> str:
        """Step 6: the spur base returns '1'. This string is spliced in once,
        as the last factor of the live FilletRadius expression at
        parameter-registration time, so the root fillet reads correctly on the
        tilted tooth's transverse plane. The inherited createFillets never
        calls this hook — it reads only the resulting parameter's value."""
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    def helicalPlaneOffset(self) -> adsk.core.ValueInput:
        """Step 7: the full Thickness for helical. A method of its own because
        it is the seam herringbone re-points at half the thickness so its
        mirror plane lands mid-body.

        What it returns is a numeric snapshot, not a live parameter reference
        ([PB-NUMERIC-SNAPSHOT]): getParameterAsValueInput wraps param.value in
        ValueInput.createByReal, so the plane is placed at the Thickness that
        held at generation time and editing the parameter afterwards does not
        move it."""
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    def buildSketches(self, ctx: GenerationContext):
        """Steps 8-9: spur's bottom Gear Profile, then the helix plane and the
        twisted profile drawn on it."""
        # Draws the bottom Gear Profile and runs the spur tooth generator at
        # angle 0.
        super().buildSketches(ctx)

        # Step 8: the offset plane, on the gear's own component. The offset
        # argument is a ValueInput, never a bare number
        # ([PB-CONSTRUCTION-PLANES]).
        component: adsk.fusion.Component = self.getComponent()
        constructionPlaneInput = component.constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = component.constructionPlanes.add(constructionPlaneInput)
        ctx.helixPlane = plane
        # The plane is left visible when the build finishes: spur's cleanup
        # hides only the entities spur created, and helical adds no cleanup of
        # its own. That is deliberate — do not add cleanup for it (step 12).

        # Step 9: the second sketch, created hidden and never shown, in either
        # mode including SketchOnly. Declared delta from [PB-HIDE-AFTER-USE]:
        # profile finding for the loft works on it hidden.
        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        # The twist is delivered as the tooth generator's own draw() angle
        # ([SPUR-F-ROTATE-CONFIRM], [SPUR-F-SPINE]): the generator rotates the
        # whole tooth by it in the Python point math and then confirms the
        # rotation with the spine's angular dimension, so the sketch says which
        # way it is turned. Drawing the tooth flat and rotating the geometry
        # afterwards leaves the spine dimension measuring the unrotated angle,
        # and Fusion is then free to settle the tooth on the far solver branch.
        # The angle is the raw radian .value and it is signed — a negative
        # value is a left-hand helix. Nothing rescales it: the twist between
        # the two loft sections is the Helix Angle itself, so Thickness does
        # not enter.
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        ctx.twistedGearProfileSketch = loftSketch

    def loftTooth(self, ctx: GenerationContext):
        """Step 10: loft the bottom tooth loop to the twisted one, bottom
        section added before the top ([HELI-F-LOFT], [PB-LOFT])."""
        component: adsk.fusion.Component = self.getComponent()
        lofts: adsk.fusion.LoftFeatures = component.features.loftFeatures
        # Both searches pass a fixed lines=2, the non-embedded six-curve tooth,
        # and neither reads ctx.toothProfileIsEmbedded. An embedded
        # low-tooth-count gear's loop has four curves, so the search finds
        # nothing and the build fails there ([PB-PROFILE-MATCH]). That is the
        # current implementation's documented limitation, reproduced as it
        # stands — there is no embedded branch.
        bottomToothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile = find_profile_by_curve_counts(
            ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'

    def buildTooth(self, ctx: GenerationContext):
        """Step 11: loft, then chamfer, and nothing else — helical does not
        extrude. Ending with the chamfer is spur's boundary: the inherited
        buildMainGearBody does not chamfer separately, so omitting the call
        would silently drop the chamfer.

        chamferTooth is inherited unchanged and selects the tooth's front face
        by an edge count and a coplanarity test against the sketch plane.
        Helical does NOT override chamferWantEdges: the inherited 6 is correct,
        because helical's lofted tooth is built from the non-embedded six-curve
        profile and its cap face carries the same six curves spur's extruded
        tooth does ([HELI-F-CHAMFER-COUNT]). An embedded gear still raises
        here, exactly as an embedded spur gear does, and the whole new
        component is rolled back by the entry point; the raise stays a raise."""
        self.loftTooth(ctx)
        self.chamferTooth(ctx)
