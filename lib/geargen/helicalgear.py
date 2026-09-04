"""Helical gear generator, emitted from the compiled step list spec/helicalgear/steps.md
(steps 1-12).

Helical is a subclass-only module: it inherits spur's whole build pipeline and changes three
things — one dialog input, one extra pair of timeline entries inside buildSketches (an offset
construction plane and a second, twisted sketch), and one replacement of spur's tooth extrude
with a loft. The geometry for the steps tagged [GO] (8, 9, 10) is transliterated from
proof/helicalgear/ (sketches_test.go, solids_test.go); nothing here is re-derived.

Units: Fusion's internal units are centimetres for length and radians for angle. A dialog's unit
string controls display and expression parsing only, so every ValueInput.createByReal default is
written in internal units ([PB-DIALOG-DEFAULT-UNITS]).
"""

import math

import adsk.core, adsk.fusion

from .base import get_value
from .utilities import find_profile_by_curve_counts
from .spurgear import (PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
                       SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
                       SpurGearGenerator, SpurGearInvoluteToothDesignGenerator)

# Dialog input id and registered user-parameter name for the Helix Angle input (step 1).
# helicalgear.py's own contract surface: herringbonegear.py imports both by name.
PARAM_HELIX_ANGLE = 'HelixAngle'
INPUT_ID_HELIX_ANGLE = 'helixAngle'


class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    """Step 2: spur's ten dialog inputs plus one. Helix Angle lands after Parent Component
    because super().configure(cmd) already added Parent Component last ([SPUR-SUBCLASS-INPUT])."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        super().configure(cmd)
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))


class HelicalGearGenerationContext(SpurGearGenerationContext):
    """Step 3: spur's context fields plus the two helical adds."""

    def __init__(self):
        super().__init__()
        # The offset ConstructionPlane the twisted top profile is drawn on. Also the plane
        # herringbone mirrors across, so the field name is public API.
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The second, 'Twisted Gear Profile' sketch: the loft's top section.
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)


class HelicalGearGenerator(SpurGearGenerator):
    """The spur build pipeline with exactly nine method boundaries changed (step 11):
    newContext, prefixBase, generateName, addExtraPrimaryParameters,
    filletHelixFactorExpression, helicalPlaneOffset, buildSketches, buildTooth, loftTooth.
    Everything else — processInputs, prepareTools, buildMainGearBody, buildBody, patternTeeth,
    createFillets, buildBore, chamferTeeth, cleanup, and the entire
    SpurGearInvoluteToothDesignGenerator — is inherited unchanged and not re-implemented here."""

    # --- step 4: generator identity --------------------------------------------------------
    # Spur's base pins prefixBase -> str, generateName -> str, filletHelixFactorExpression ->
    # str and newContext -> SpurGearGenerationContext on these five overridable methods
    # precisely so a subclass may annotate its own; a subclass return may repeat or narrow the
    # parent's, never widen it. newContext below narrows to HelicalGearGenerationContext, which
    # a type checker accepts because a narrowed RETURN is a legal override — unlike a ctx
    # PARAMETER, which steps 8 and 10 forbid narrowing.

    def newContext(self) -> HelicalGearGenerationContext:
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    # --- step 5: register the HelixAngle user parameter --------------------------------------

    def addExtraPrimaryParameters(self, inputs):
        helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
        self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')

    # --- step 6: the root-fillet transverse correction ---------------------------------------

    def filletHelixFactorExpression(self) -> str:
        return f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'

    # --- step 7: the helix plane's offset hook ------------------------------------------------

    def helicalPlaneOffset(self):
        return self.getParameterAsValueInput(PARAM_THICKNESS)

    # --- steps 8 and 9: the helix construction plane and the Twisted Gear Profile sketch -----

    def buildSketches(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        super().buildSketches(ctx)

        constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
        constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
        plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
        ctx.helixPlane = plane

        loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
        SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
            ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
        ctx.twistedGearProfileSketch = loftSketch

    # --- step 10: loft the tooth ---------------------------------------------------------------

    def buildTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        self.loftTooth(ctx)

    def loftTooth(self, ctx: SpurGearGenerationContext):
        assert isinstance(ctx, HelicalGearGenerationContext)
        lofts = self.getComponent().features.loftFeatures
        bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
        topToothProfile    = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(bottomToothProfile)
        loftInput.loftSections.add(topToothProfile)
        loftResult = lofts.add(loftInput)
        ctx.toothBody = loftResult.bodies.item(0)
        ctx.toothBody.name = 'Tooth Body'
