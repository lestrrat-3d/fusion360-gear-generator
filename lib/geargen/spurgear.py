"""Spur gear generator, emitted from the compiled step list spec/spurgear/steps.md (S1-S16).

The geometry is transliterated from proof/spurgear/ (sketches_test.go, solids_test.go) for the
steps the step list tags [GO]; nothing here is re-derived.

Units: Fusion's internal units are centimetres for length and radians for angle. A dialog's unit
string controls display and expression parsing only, so every ValueInput.createByReal default is
written in internal units ([PB-DIALOG-DEFAULT-UNITS]).
"""

import math

import adsk.core, adsk.fusion

from .base import Generator, GenerationContext, get_boolean, get_selection, get_value
from .misc import get_design, to_cm
from .utilities import find_profile_by_curve_counts, get_normal

# Dialog input ids (S1). helicalgear.py and herringbonegear.py import these by name, so the
# identifiers and their string values are contract surface ([SPUR-EXPORTED-CONSTANTS]).
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'
INPUT_ID_MODULE = 'module'
INPUT_ID_TOOTH_NUMBER = 'toothNumber'
INPUT_ID_PRESSURE_ANGLE = 'pressureAngle'
INPUT_ID_BORE_DIAMETER = 'boreDiameter'
INPUT_ID_THICKNESS = 'thickness'
INPUT_ID_CHAMFER_TOOTH = 'chamferTooth'
INPUT_ID_SKETCH_ONLY = 'sketchOnly'

# Registered user-parameter names (S2). The prefix base below turns each into SpurGear<N>_<name>.
PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
PARAM_PRESSURE_ANGLE = 'PressureAngle'
PARAM_BORE_DIAMETER = 'BoreDiameter'
PARAM_THICKNESS = 'Thickness'
PARAM_CHAMFER_TOOTH = 'ChamferTooth'
PARAM_SKETCH_ONLY = 'SketchOnly'
PARAM_PITCH_DIAMETER = 'PitchCircleDiameter'
PARAM_PITCH_RADIUS = 'PitchCircleRadius'
PARAM_BASE_DIAMETER = 'BaseCircleDiameter'
PARAM_BASE_RADIUS = 'BaseCircleRadius'
PARAM_ROOT_DIAMETER = 'RootCircleDiameter'
PARAM_ROOT_RADIUS = 'RootCircleRadius'
PARAM_TIP_DIAMETER = 'TipCircleDiameter'
PARAM_TIP_RADIUS = 'TipCircleRadius'
PARAM_INVOLUTE_STEPS = 'InvoluteSteps'
PARAM_TOOTH_SPACE_ANGLE = 'ToothSpaceAngleAtRoot'
PARAM_TOOTH_SPACE_ARC = 'ToothSpaceArcAtRoot'
PARAM_FILLET_CLEARANCE = 'FilletClearance'
PARAM_FILLET_RADIUS = 'FilletRadius'

# The label of the sketch-only check box (S1 item 9), quoted from the step list.
SKETCH_ONLY_LABEL = 'Generate sketches, but do not build body'

# Tolerance, in internal centimetres, for matching a face or edge radius against a parameter.
RADIUS_TOLERANCE = 0.001


def _rotate(x, y, angle):
    """Turn (x, y) counter-clockwise by angle radians."""
    sa, ca = math.sin(angle), math.cos(angle)
    return x * ca - y * sa, x * sa + y * ca


class SpurGearCommandInputsConfigurator:
    """S1: the command dialog. `configure` is called by the shared command wiring in
    commands/_gear_command.py; this module declares it and never calls it.

    The add order below is the display order, not the processInputs read order. Target Plane is
    first because Fusion auto-focuses the first selection input ([PB-AUTOFOCUS-FIRST]), and
    Parent Component is last so a subclass appending its own input after super().configure(cmd)
    lands below it ([SPUR-SUBCLASS-INPUT])."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane. Filters and limits are declared per input ([PB-SELECTION-DECL]) and
        # each filter is the named enum member, never a string ([PB-SELECTION-FILTER-ENUM]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the gear on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point.
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point the gear is centered on')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Module: unitless, so the unit string is empty and not 'mm'.
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 4. Tooth Number.
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))

        # 5. Pressure Angle: displayed in degrees, defaulted in internal radians.
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))

        # 6. Bore Diameter is a string input so it accepts an expression.
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

        # 7. Thickness: displayed in millimetres, defaulted in internal centimetres.
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))

        # 8. Tooth chamfer distance.
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 9. The sketch-only switch. The trailing True makes it a check box, not a button.
        inputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, SKETCH_ONLY_LABEL, True)

        # 10. Parent Component, last, pre-selected on the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the component to build the gear in')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    """The build context the spur pipeline threads through its steps."""

    def __init__(self):
        super().__init__()
        # The normalized target plane (S3).
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        # The anchor projected into the Tools sketch; every later sketch re-projects this one
        # point ([SPUR-F-ANCHOR-CHAIN]).
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        # The to-entity target both extrudes end on (S5).
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        # The far end-cap face the bore cut runs to (S9, S14).
        self.extrusionExtent = adsk.fusion.BRepFace.cast(None)
        self.toothProfileIsEmbedded = False
        # The two sketches cleanup hides alongside the Gear Profile sketch (S4, S13, S16). The
        # step list keeps every entity a later step consumes on the context, and cleanup takes
        # only the context, so these live here too.
        self.toolsSketch = adsk.fusion.Sketch.cast(None)
        self.boreProfileSketch = adsk.fusion.Sketch.cast(None)


class SpurGearInvoluteToothDesignGenerator:
    """S6 and S13: the drawing of one involute tooth, and of the bore circle, in one sketch.

    The generator is instantiated on the sketch it draws into. helicalgear.py instantiates it on
    its own loft sketch and calls draw() with a non-zero angle, so everything here is written for
    an arbitrary requested angle."""

    def __init__(self, sketch: adsk.fusion.Sketch, parent):
        self.sketch = sketch
        self.parent = parent

        # The local origin ([SPUR-F-LOCAL-ORIGIN]): a fresh, movable SketchPoint at the sketch
        # origin, never sketch.originPoint, which is immutable and cannot be made coincident with
        # a projection ([PB-CIRCLE-CENTER]). Every piece of geometry below is drawn relative to
        # it, and the anchoring at the end of draw() slides the whole drawing onto the user's
        # anchor point.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

        # Filled in by drawCircles / drawTooth / drawBore.
        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
        self.baseCircle = adsk.fusion.SketchCircle.cast(None)
        self.pitchCircle = adsk.fusion.SketchCircle.cast(None)
        self.toothAngleDimension = adsk.fusion.SketchAngularDimension.cast(None)
        self.projectedAnchorPoint = adsk.fusion.SketchPoint.cast(None)

    # --- parameter access -----------------------------------------------------------------

    def getParameter(self, name: str):
        return self.parent.getParameter(name)

    def getParameterValue(self, name: str) -> float:
        return self.getParameter(name).value

    # --- involute math --------------------------------------------------------------------

    def calculateInvolutePoint(self, baseCircleRadius: float, intersectionRadius: float):
        """The point on the involute of baseCircleRadius where the unrolled string reaches
        intersectionRadius, or None inside the base circle, where the curve does not exist.

        The curve parameter is tan(alpha), NOT the involute function tan(alpha) - alpha."""
        if intersectionRadius < baseCircleRadius:
            return None
        alpha = math.acos(baseCircleRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseCircleRadius * (math.cos(t) + t * math.sin(t))
        y = baseCircleRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    # --- the Gear Profile sketch (S6) -----------------------------------------------------

    def draw(self, anchorPoint, angle: float = 0):
        """Draw the circles, the tooth, and the anchoring that ties the drawing to the user's
        anchor point; then, for a rotated tooth only, confirm the rotation."""
        self.drawCircles()
        self.drawTooth(angle)

        # The anchoring ([SPUR-F-ANCHOR-CHAIN]). It happens here rather than in buildSketches
        # because helical and herringbone call draw() directly on their own loft sketch and rely
        # on this one call to anchor it. A projection is associative but not fixed
        # ([PB-PROJECT-NOT-FIXED]), so this coincidence is what carries the sketch to full
        # constraint ([PB-FULL-CONSTRAINT]).
        projected = self.sketch.project(anchorPoint)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projected.item(0))
        self.projectedAnchorPoint = projected.item(0)

        # [SPUR-F-ROTATE-CONFIRM]: the value-set is the very last action of draw, after the whole
        # constraint network exists, and it carries the sign a mirrored scheme would lose.
        if angle != 0:
            self.toothAngleDimension.parameter.value = angle

    def drawCircles(self):
        """The four gear circles, all sharing the local origin SketchPoint."""
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles

        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)

        # The text height, and the size the labels report.
        size = tipRadius - rootRadius

        # The local-origin SketchPoint object is passed in, so all four circles SHARE it
        # ([PB-SHARE-XOR-COINCIDENT] via [SPUR-F-SHARED-ADJACENCY]) and none of them gets a
        # coincident to it. Only the root circle is solid: the tip circle is construction
        # geometry, which is why the body extrude sees a disc and not an annulus.
        self.rootCircle = circles.addByCenterRadius(self.anchorPoint, rootRadius)
        self.tipCircle = circles.addByCenterRadius(self.anchorPoint, tipRadius)
        self.baseCircle = circles.addByCenterRadius(self.anchorPoint, baseRadius)
        self.pitchCircle = circles.addByCenterRadius(self.anchorPoint, pitchRadius)
        self.tipCircle.isConstruction = True
        self.baseCircle.isConstruction = True
        self.pitchCircle.isConstruction = True

        for name, circle, radius in (
                ('Root Circle', self.rootCircle, rootRadius),
                ('Tip Circle', self.tipCircle, tipRadius),
                ('Base Circle', self.baseCircle, baseRadius),
                ('Pitch Circle', self.pitchCircle, pitchRadius)):
            # A driving diameter dimension: driving is the default and the trailing driven flag
            # is never passed ([PB-DRIVING-DIM]). The text point sits on the circle, because a
            # diameter dimension rejects a text point at the centre ([PB-RADIAL-DIM]).
            textPoint = adsk.core.Point3D.create(
                radius * math.cos(math.pi / 4), radius * math.sin(math.pi / 4), 0)
            sketch.sketchDimensions.addDiameterDimension(circle, textPoint)

            # Label the circle along its own path ([PB-SKETCH-TEXT]'s fixed three-call shape).
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(text, size)
            textInput.setAsAlongPath(
                circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)

    def drawTooth(self, angle: float = 0):
        """One involute tooth, drawn at its final angular position."""
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        leftFlank, rightFlank = self._drawFlanks(
            baseRadius, tipRadius, pitchRadius, toothNumber, steps, angle)
        toothTopPoint = self._drawToothTop(leftFlank, rightFlank, tipRadius, angle)
        spine = self._drawSpine(toothTopPoint, tipRadius, angle)
        self._drawRibs(leftFlank, rightFlank, spine, angle)
        self._drawFlankToRoot(leftFlank, rightFlank)

    def _drawFlanks(self, baseRadius, tipRadius, pitchRadius, toothNumber, steps, angle):
        """The two involute flanks, as fitted splines through endpoint-inclusive samples.

        Three things happen to the samples, in this order. They are mirrored across +X, because
        the standard parametric involute spirals the wrong way for a left flank; they are rotated
        so the pitch crossing lands at +pi/(2N), which centres the tooth on +X; and only then is
        the requested angle applied to both flanks together."""
        sketch = self.sketch

        mirrored = []
        for i in range(steps):
            radius = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            point = self.calculateInvolutePoint(baseRadius, radius)
            if point is None:
                continue
            mirrored.append((point.x, -point.y))

        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None:
            raise Exception(
                'Spur gear: the pitch circle lies inside the base circle, so the involute has '
                'no pitch crossing to centre the tooth on')
        rotateAngle = math.pi / (2 * toothNumber) - math.atan2(-pitchPoint.y, pitchPoint.x)

        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for x, y in mirrored:
            lx, ly = _rotate(x, y, rotateAngle)
            rx, ry = lx, -ly
            lx, ly = _rotate(lx, ly, angle)
            rx, ry = _rotate(rx, ry, angle)
            leftPoints.add(adsk.core.Point3D.create(lx, ly, 0))
            rightPoints.add(adsk.core.Point3D.create(rx, ry, 0))

        leftFlank = sketch.sketchCurves.sketchFittedSplines.add(leftPoints)
        rightFlank = sketch.sketchCurves.sketchFittedSplines.add(rightPoints)
        return leftFlank, rightFlank

    def _drawToothTop(self, leftFlank, rightFlank, tipRadius, angle):
        """The tooth-top point and the arc that caps the tooth ([SPUR-F-TOOTHTOP-ARC])."""
        sketch = self.sketch

        toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(
            tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0))
        sketch.geometricConstraints.addCoincident(toothTopPoint, self.tipCircle)

        # addByCenterStartEnd shares the start and end points but COPIES the centre, so this is
        # the one place [PB-SHARE-XOR-COINCIDENT] calls for a coincident on a point that was
        # passed in: without it the arc's centre is a free point that stays behind when the
        # drawing is dragged onto the anchor. No diameter dimension — the coincident centre and
        # the two shared ends already determine the arc.
        toothTopArc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            self.anchorPoint, rightFlank.endSketchPoint, leftFlank.endSketchPoint)
        sketch.geometricConstraints.addCoincident(toothTopArc.centerSketchPoint, self.anchorPoint)
        return toothTopPoint

    def _drawSpine(self, toothTopPoint, tipRadius, angle):
        """The spine, the +X reference and the angular pin ([SPUR-F-SPINE]).

        The reference and its dimension are built for every angle, zero included. A horizontal
        constraint on the spine is deliberately not used: it fixes the line's direction but not
        which way it points, and the tooth can come out a half turn around."""
        sketch = self.sketch

        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True

        # The reference endpoint is seeded on the +X side and both dimensions carry non-negative
        # magnitudes: a linear dimension's value is a magnitude and its direction is captured
        # from the geometry at creation ([PB-DIM-VALUE-SEMANTICS]).
        referenceEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        horizontal = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, referenceEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius / 2, -tipRadius / 8, 0))
        horizontal.parameter.value = tipRadius
        vertical = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, referenceEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius * 1.1, -tipRadius / 8, 0))
        vertical.parameter.value = 0

        referenceLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            self.anchorPoint, referenceEnd)
        referenceLine.isConstruction = True

        # An angular dimension measures the wedge its text point lies in ([PB-ANGULAR-DIM]), so
        # the text goes on the bisector at a small radius. The argument order is reference then
        # spine, which is what carries the sign of the requested rotation.
        textRadius = tipRadius / 4
        self.toothAngleDimension = sketch.sketchDimensions.addAngularDimension(
            referenceLine, spine,
            adsk.core.Point3D.create(
                textRadius * math.cos(angle / 2), textRadius * math.sin(angle / 2), 0))
        return spine

    def _drawRibs(self, leftFlank: adsk.fusion.SketchFittedSpline,
                  rightFlank: adsk.fusion.SketchFittedSpline,
                  spine: adsk.fusion.SketchLine, angle):
        """One rib per fit-point index, endpoints included ([SPUR-F-RIBS]).

        The rib takes the axis ACROSS the spine and the midpoint chain the one ALONG it; past the
        |cos| >= |sin| test the two swap, which is what keeps a tooth at a quarter turn
        solvable."""
        sketch = self.sketch
        count = leftFlank.fitPoints.count
        ribAcrossVertical = abs(math.cos(angle)) >= abs(math.sin(angle))
        previous = self.anchorPoint

        for i in range(count):
            leftFit = leftFlank.fitPoints.item(i)
            rightFit = rightFlank.fitPoints.item(i)
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True

            # One axis dimension across the spine, created at the seeded positions and left at
            # the measured magnitude, so the direction that forbids a mirrored tooth comes from
            # the seed ([PB-DIM-VALUE-SEMANTICS]).
            leftGeometry = leftFit.geometry
            rightGeometry = rightFit.geometry
            ribTextPoint = adsk.core.Point3D.create(
                (leftGeometry.x + rightGeometry.x) / 2,
                (leftGeometry.y + rightGeometry.y) / 2, 0)
            if ribAcrossVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                    ribTextPoint)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                    ribTextPoint)

            # The midpoint is seeded at the foot of the left fit point ON the spine: the solver
            # is seed-sensitive and a point that must end up on a line belongs near that line
            # ([PB-SEED-NEAR]).
            along = leftGeometry.x * math.cos(angle) + leftGeometry.y * math.sin(angle)
            midX, midY = along * math.cos(angle), along * math.sin(angle)
            midPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(midX, midY, 0))
            sketch.geometricConstraints.addCoincident(midPoint, spine)
            sketch.geometricConstraints.addMidPoint(midPoint, rib)
            if i != count - 1:
                # The last rib carries no perpendicular: the tooth-top arc already holds its two
                # ends at equal radius either side of the spine, and adding a constraint an
                # existing one already drives over-constrains the sketch ([PB-NO-OVERCONSTRAIN]).
                sketch.geometricConstraints.addPerpendicular(spine, rib)

            # Chain the midpoints along the spine direction, the first from the local origin.
            previousGeometry = previous.geometry
            chainTextPoint = adsk.core.Point3D.create(
                (previousGeometry.x + midX) / 2, (previousGeometry.y + midY) / 2, 0)
            if ribAcrossVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    previous, midPoint,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                    chainTextPoint)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    previous, midPoint,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                    chainTextPoint)
            previous = midPoint

    def _drawFlankToRoot(self, leftFlank: adsk.fusion.SketchFittedSpline,
                         rightFlank: adsk.fusion.SketchFittedSpline):
        """The flank-to-root lines ([SPUR-F-FLANK-ROOT]).

        The root endpoint is pinned by exactly two axis dimensions from the local origin.
        Constraining it onto the root circle, or the local origin onto the stub line, also
        reaches zero degrees of freedom but leaves the far root-circle intersection equally
        valid, and the stub then runs across the gear."""
        sketch = self.sketch
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)

        firstPoint = leftFlank.fitPoints.item(0).geometry
        firstRadius = self.anchorPoint.geometry.distanceTo(firstPoint)

        # Compared raw, with no tolerance: exact equality counts as NOT embedded and draws a
        # zero-length stub.
        embedded = firstRadius < rootRadius
        if not embedded:
            scale = rootRadius / firstRadius
            for flank in (leftFlank, rightFlank):
                start = flank.startSketchPoint.geometry
                endX, endY = start.x * scale, start.y * scale

                # Seed the root end at its exact computed position BEFORE the dimensions exist,
                # then set each to the ABSOLUTE magnitude: a negative value flips the point to
                # the other side of the origin ([PB-DIM-VALUE-SEMANTICS]).
                rootEndPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(endX, endY, 0))
                sketch.sketchCurves.sketchLines.addByTwoPoints(
                    rootEndPoint, flank.startSketchPoint)
                horizontal = sketch.sketchDimensions.addDistanceDimension(
                    self.anchorPoint, rootEndPoint,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                    adsk.core.Point3D.create(endX / 2, endY / 2, 0))
                horizontal.parameter.value = abs(endX)
                vertical = sketch.sketchDimensions.addDistanceDimension(
                    self.anchorPoint, rootEndPoint,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                    adsk.core.Point3D.create(endX / 2, endY / 2, 0))
                vertical.parameter.value = abs(endY)

        self.parent._lastToothEmbedded = embedded

    # --- the Bore Profile sketch (S13) ----------------------------------------------------

    def drawBore(self, anchorPoint, boreDiameter: float):
        """Project the anchor into this sketch and draw the dimensioned bore circle on it."""
        sketch = self.sketch
        projected = sketch.project(anchorPoint)
        projectedAnchor = projected.item(0)
        self.projectedAnchorPoint = projectedAnchor

        boreCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, boreDiameter / 2)
        sketch.sketchDimensions.addDiameterDimension(
            boreCircle, adsk.core.Point3D.create(
                projectedAnchor.geometry.x + boreDiameter / 2,
                projectedAnchor.geometry.y, 0))
        return boreCircle


class SpurGearGenerator(Generator):
    """The spur build pipeline. helicalgear.py and herringbonegear.py subclass it and override
    at these method boundaries, so the boundaries and names do not move."""

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        # The construction plane S3 built when the user picked a planar face; cleanup switches
        # its light bulb off, and None means the user's own plane was used unchanged.
        self.normalizedPlane = adsk.fusion.ConstructionPlane.cast(None)
        # The output slot the tooth generator writes during draw() ([SPUR-F-FLANK-ROOT]).
        self._lastToothEmbedded = False

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    # --- S2: inputs and parameters --------------------------------------------------------

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        """[SPUR-EXTRA-PARAMS]: the hook a subclass registers its own primary parameters in,
        called between the input-sourced parameters and the derived ones. A no-op on the spur
        base; the call site is what a subclass hooks."""
        pass

    def filletHelixFactorExpression(self) -> str:
        """The last factor of the live FilletRadius expression. The spur base has no helix."""
        return '1'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        """Read the dialog and register the parameters.

        This runs before anything touches the design: creating the gear occurrence shifts
        Fusion's active component, and a selection input holding an entity from another component
        can drop it ([PB-SELECTION-STASH]), so the three selections are read and stashed first."""
        parentEntity = get_selection(inputs, INPUT_ID_PARENT)[0]
        occurrence = adsk.fusion.Occurrence.cast(parentEntity)
        if occurrence is not None:
            parentEntity = occurrence.component
        self.parentComponent = parentEntity
        self.plane = get_selection(inputs, INPUT_ID_PLANE)[0]
        self.anchorPoint = get_selection(inputs, INPUT_ID_ANCHOR_POINT)[0]

        # Each input is read with the helper that matches the type it was declared with
        # ([PB-INPUT-READ]); get_value returns a ValueInput ready for addParameter and raises on
        # a bad expression, so nothing is wrapped or ok-flagged here ([PB-GET-VALUE-CONTRACT]).
        self.addParameter(PARAM_MODULE, get_value(inputs, INPUT_ID_MODULE, ''), '',
                          'Module of the gear')
        self.addParameter(PARAM_TOOTH_NUMBER, get_value(inputs, INPUT_ID_TOOTH_NUMBER, ''), '',
                          'Number of teeth')
        self.addParameter(PARAM_PRESSURE_ANGLE,
                          get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad'), 'rad',
                          'Pressure angle')
        self.addParameter(PARAM_BORE_DIAMETER,
                          get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm'), 'mm',
                          'Bore diameter')
        self.addParameter(PARAM_THICKNESS, get_value(inputs, INPUT_ID_THICKNESS, 'mm'), 'mm',
                          'Thickness of the gear')
        self.addParameter(PARAM_CHAMFER_TOOTH,
                          get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm'), 'mm',
                          'Chamfer distance applied to the teeth')
        # A check box has no expression, so it is read with get_boolean and registered as a real
        # 1 or 0 that getParameterAsBoolean reads back.
        sketchOnly = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(PARAM_SKETCH_ONLY,
                          adsk.core.ValueInput.createByReal(1 if sketchOnly else 0), '',
                          'Generate sketches only')

        self.addExtraPrimaryParameters(inputs)
        self.registerDerivedParameters()

    def registerDerivedParameters(self):
        """The derived parameters, as live expression strings referencing the ones above.

        They stay in the parameter table for reference only: every dimension and feature input
        later in the build takes the current numeric value of its parameter, never a live
        expression ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT])."""
        name = self.parameterName

        self.addParameter(
            PARAM_PITCH_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} * {}'.format(name(PARAM_MODULE), name(PARAM_TOOTH_NUMBER))),
            'mm', 'Pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(name(PARAM_PITCH_DIAMETER))),
            'mm', 'Pitch circle radius')
        self.addParameter(
            PARAM_BASE_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} * cos({})'.format(name(PARAM_PITCH_DIAMETER), name(PARAM_PRESSURE_ANGLE))),
            'mm', 'Base circle diameter')
        self.addParameter(
            PARAM_BASE_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(name(PARAM_BASE_DIAMETER))),
            'mm', 'Base circle radius')
        self.addParameter(
            PARAM_ROOT_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} - 2.5 * {}'.format(name(PARAM_PITCH_DIAMETER), name(PARAM_MODULE))),
            'mm', 'Root circle diameter')
        self.addParameter(
            PARAM_ROOT_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(name(PARAM_ROOT_DIAMETER))),
            'mm', 'Root circle radius')
        self.addParameter(
            PARAM_TIP_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} + 2 * {}'.format(name(PARAM_PITCH_DIAMETER), name(PARAM_MODULE))),
            'mm', 'Tip circle diameter')
        self.addParameter(
            PARAM_TIP_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(name(PARAM_TIP_DIAMETER))),
            'mm', 'Tip circle radius')
        self.addParameter(
            PARAM_INVOLUTE_STEPS, adsk.core.ValueInput.createByString('15'), '',
            'Number of points sampled along each involute flank')

        # The one parameter computed in Python: Fusion's expression engine will not subtract a
        # radian-valued PressureAngle from the unitless output of tan. It is registered UNITLESS,
        # not 'rad', so ToothSpaceArcAtRoot below reads as a length.
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngle = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE, adsk.core.ValueInput.createByReal(toothSpaceAngle), '',
            'Angular width of the tooth space at the root circle')

        self.addParameter(
            PARAM_TOOTH_SPACE_ARC,
            adsk.core.ValueInput.createByString(
                '{} * {}'.format(name(PARAM_ROOT_RADIUS), name(PARAM_TOOTH_SPACE_ANGLE))),
            'mm', 'Arc length of the tooth space at the root circle')
        self.addParameter(
            PARAM_FILLET_CLEARANCE, adsk.core.ValueInput.createByString('0.9'), '',
            'Clearance factor applied to the root fillet radius')
        self.addParameter(
            PARAM_FILLET_RADIUS,
            adsk.core.ValueInput.createByString(
                '({} / 2) * {} * {}'.format(
                    name(PARAM_TOOTH_SPACE_ARC), name(PARAM_FILLET_CLEARANCE),
                    self.filletHelixFactorExpression())),
            'mm', 'Radius of the root fillets')

    # --- orchestration --------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component: adsk.fusion.Component = self.getComponent()
        component.name = self.generateName()

        self._normalizeTargetPlane(component)

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def _normalizeTargetPlane(self, component: adsk.fusion.Component):
        """S3: the user may have picked a planar face, so replace it with a coplanar
        construction plane. The offset argument is a ValueInput, never a bare number
        ([PB-CONSTRUCTION-PLANES])."""
        if adsk.fusion.ConstructionPlane.cast(self.plane) is not None:
            return
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
        self.normalizedPlane = component.constructionPlanes.add(planeInput)
        self.plane = self.normalizedPlane

    def prepareTools(self, ctx: SpurGearGenerationContext):
        """S4 and S5: the Tools sketch that owns the anchor projection, and the Extrusion End
        Plane both extrudes end on."""
        component: adsk.fusion.Component = self.getComponent()

        toolsSketch = self.createSketchObject('Tools', self.plane)
        # It stays visible through the bore, which re-projects from it; projection has failed on
        # an invisible sketch in this repository's history ([PB-HIDE-AFTER-USE]).
        toolsSketch.isVisible = True
        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)
        ctx.toolsSketch = toolsSketch

        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        extrusionEndPlane = component.constructionPlanes.add(planeInput)
        extrusionEndPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = extrusionEndPlane

    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        """S7: the sketches, then — unless the user asked for sketches only — the two extrudes,
        the pattern and the fillets."""
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)
        self.createFillets(ctx)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        """S6: one sketch, one timeline entry, and the whole involute construction inside it."""
        gearProfileSketch = self.createSketchObject('Gear Profile', self.plane)
        gearProfileSketch.isVisible = True
        ctx.gearProfileSketch = gearProfileSketch

        SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self).draw(
            ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # --- S8: the tooth --------------------------------------------------------------------

    def buildTooth(self, ctx: SpurGearGenerationContext):
        component: adsk.fusion.Component = self.getComponent()

        # Profiles are found by the curve-type counts of their loop, never by index
        # ([PB-PROFILE-MATCH]); the helper raises when nothing matches.
        profile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2)

        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

    # --- S9: the body, the Gear Center axis and the far end cap ---------------------------

    def buildBody(self, ctx: SpurGearGenerationContext):
        component: adsk.fusion.Component = self.getComponent()

        # Exactly two arcs: the two pieces the tooth cut the root circle into.
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)

        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude body'

        gearBody = extrude.bodies.item(0)
        gearBody.name = 'Gear Body'
        ctx.gearBody = gearBody

        # Both references are found by surface type and then disambiguated against the sketch
        # plane, never by enumeration order ([PB-FACE-BY-MIDPOINT] is the cross-gear form).
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        cylindricalFace = None
        extrusionExtent = None
        faces = extrude.bodies.item(0).faces
        for i in range(faces.count):
            face = faces.item(i)
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                cylindricalFace = face
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if sketchPlane.isParallelToPlane(face.geometry) and \
                        not sketchPlane.isCoPlanarTo(face.geometry):
                    extrusionExtent = face

        if cylindricalFace is None:
            raise Exception(
                'Spur gear: the Gear Body has no cylindrical face to build the Gear Center '
                'axis from')
        if extrusionExtent is None:
            raise Exception(
                'Spur gear: the Gear Body has no end-cap face parallel to but not coplanar '
                'with the Gear Profile sketch plane')

        axisInput = component.constructionAxes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        centerAxis = component.constructionAxes.add(axisInput)
        centerAxis.name = 'Gear Center'
        centerAxis.isLightBulbOn = False
        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent

    # --- S10 and S11: pattern the tooth, then join the copies into the Gear Body -----------

    def patternTeeth(self, ctx: SpurGearGenerationContext):
        component: adsk.fusion.Component = self.getComponent()
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value

        bodyCollection = adsk.core.ObjectCollection.create()
        bodyCollection.add(ctx.toothBody)
        patternInput = component.features.circularPatternFeatures.createInput(
            bodyCollection, ctx.centerAxis)
        # All three are pinned explicitly, never left to Fusion's defaults
        # ([PB-CIRCULAR-PATTERN]).
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = component.features.circularPatternFeatures.add(patternInput)

        # The pattern's own bodies collection already holds the seed alongside the copies, so the
        # seed is not added twice; a BRepBodies is not an ObjectCollection, so its items are
        # copied into a fresh one ([PB-PATTERN-BODIES]).
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))

        combineInput = component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(combineInput)

    # --- S12: the root fillets ------------------------------------------------------------

    def createFillets(self, ctx: SpurGearGenerationContext):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        component: adsk.fusion.Component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)

        # Every root cylinder patch, not just the first: after the pattern and the combine the
        # root cylinder is usually one patch per valley. On each, only the AXIAL straight edges
        # are structural root corners; the circular edges wrapping the end caps are rims.
        edgeCollection = adsk.core.ObjectCollection.create()
        faces = ctx.gearBody.faces
        for i in range(faces.count):
            face = faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) > RADIUS_TOLERANCE:
                continue
            for j in range(face.edges.count):
                edge = face.edges.item(j)
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01:
                    edgeCollection.add(edge)

        # A search that legitimately finds nothing is guarded where it is produced
        # ([PB-EMPTY-RESULT]): an empty edge set must not reach add().
        if edgeCollection.count == 0:
            return

        # The edge set goes on the fillet input ITSELF — the fillet side of the asymmetry in
        # [PB-FILLET-CHAMFER]. The trailing False keeps Fusion from pulling in tangent-adjacent
        # edges and rounding more than the root corner.
        filletInput = component.features.filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edgeCollection, adsk.core.ValueInput.createByReal(filletRadius), False)
        component.features.filletFeatures.add(filletInput)

    # --- S13 and S14: the bore sketch and the bore cut ------------------------------------

    def buildBore(self, ctx: SpurGearGenerationContext):
        """Runs unconditionally from generate, so it returns early twice: in SketchOnly mode,
        where buildMainGearBody short-circuited and ctx.gearBody and ctx.extrusionExtent were
        never set, and when Bore Diameter is not positive. The user may set both."""
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        component: adsk.fusion.Component = self.getComponent()

        boreSketch = self.createSketchObject('Bore Profile', self.plane)
        boreSketch.isVisible = True
        ctx.boreProfileSketch = boreSketch

        toothGenerator = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        toothGenerator.drawBore(ctx.anchorPoint, boreDiameter)
        # The tooth generator's constructor always adds its local origin, so this sketch carries
        # one stray unused point. It is grounded on the same projection the bore circle sits on,
        # never on the sketch's own origin, which pins it to the plane rather than to the gear
        # and has been observed to fail the solver ([PB-CIRCLE-CENTER], [SPUR-F-LOCAL-ORIGIN]).
        boreSketch.geometricConstraints.addCoincident(
            toothGenerator.anchorPoint, toothGenerator.projectedAnchorPoint)

        if boreSketch.profiles.count != 1:
            raise Exception(
                'Spur gear: the Bore Profile sketch closes {} regions, expected exactly the '
                'bore circle'.format(boreSketch.profiles.count))
        boreProfile = boreSketch.profiles.item(0)

        # The extent is the far end-cap face, so the bore pierces the gear whatever the
        # thickness, and participantBodies restricts the cut to the gear ([PB-THROUGH-CUT]).
        extrudeInput = component.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        component.features.extrudeFeatures.add(extrudeInput)

    # --- S15: the chamfer -----------------------------------------------------------------

    def chamferTeeth(self, ctx: SpurGearGenerationContext):
        """Runs from generate after buildBore, so it sees the patterned, joined, filleted gear
        and an optional bore."""
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance <= 0:
            return

        component: adsk.fusion.Component = self.getComponent()
        boreRadius = self.getParameter(PARAM_BORE_DIAMETER).value / 2
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry

        # Every edge of every end-cap face, once: the tooth flanks, the tooth tops and the
        # root-radius arcs. Only a circular edge at the bore radius is excluded, so a bore never
        # receives a chamfer.
        edgeCollection = adsk.core.ObjectCollection.create()
        seenEdges = []
        endCapCount = 0
        faces = ctx.gearBody.faces
        for i in range(faces.count):
            face = faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            if not sketchPlane.isParallelToPlane(face.geometry):
                continue
            endCapCount += 1
            for j in range(face.edges.count):
                edge = face.edges.item(j)
                if edge.tempId in seenEdges:
                    continue
                if boreRadius > 0 and \
                        edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType and \
                        abs(edge.geometry.radius - boreRadius) < RADIUS_TOLERANCE:
                    continue
                seenEdges.append(edge.tempId)
                edgeCollection.add(edge)

        if endCapCount == 0:
            raise Exception(
                'Spur gear: no end-cap face parallel to the Gear Profile sketch plane to chamfer')
        if edgeCollection.count == 0:
            raise Exception('Spur gear: no chamfer edge left on the end-cap faces')

        # The edge set goes on the input's chamferEdgeSets collection — the chamfer side of the
        # asymmetry in [PB-FILLET-CHAMFER], not the fillet's.
        chamferInput = component.features.chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edgeCollection, adsk.core.ValueInput.createByReal(chamferDistance), False)
        component.features.chamferFeatures.add(chamferInput)

    # --- S16: cleanup ---------------------------------------------------------------------

    def cleanup(self, ctx: SpurGearGenerationContext):
        """The very last action of generate, in both modes. It must not move up into
        buildMainGearBody: buildBore re-projects the anchor out of the Tools sketch, and
        projection fails once that sketch is hidden.

        Construction geometry is hidden with isLightBulbOn and sketches with isVisible; the two
        are never crossed ([PB-HIDE-AFTER-USE], [SPUR-F-CLEANUP])."""
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False

        # Sketch-only mode leaves the sketches visible, which is the point of that mode.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        for sketch in (ctx.toolsSketch, ctx.gearProfileSketch, ctx.boreProfileSketch):
            if sketch is not None:
                sketch.isVisible = False
