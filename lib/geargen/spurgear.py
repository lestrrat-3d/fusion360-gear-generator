"""Spur gear generator, emitted from spec/spurgear/steps.md.

The geometry and constraint scheme are proven in proof/spurgear/
(geometry_test.go, sketches_test.go, solids_test.go); the [GO]-tagged steps
are transliterated from that proof, not re-derived. Lengths in Fusion's
internal units are cm.
"""

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts

# Dialog input ids (step 1). Public API: dependents import these by name.
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

# Parameter names (step 1). Public API: dependents import these by name.
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


def _rotate(x, y, a):
    """Turn (x, y) counter-clockwise by a radians."""
    ca, sa = math.cos(a), math.sin(a)
    return x * ca - y * sa, x * sa + y * ca


class SpurGearCommandInputsConfigurator:
    """Adds the dialog inputs (step 2). The shared command entry invokes
    configure when the dialog opens; subclass configurators reach it through
    super().configure(cmd) and append their extras after Parent Component
    [SPUR-SUBCLASS-INPUT]."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # Target Plane and Anchor Point come first so plane selection owns
        # the dialog's initial focus ([PB-AUTOFOCUS-FIRST]). Filters are the
        # enum attributes of SelectionCommandInput ([PB-SELECTION-FILTER-ENUM]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane or planar face the gear is built on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Select the point the gear is centred on')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '',
            adsk.core.ValueInput.createByReal(17))
        # createByReal defaults are in internal units whatever the display
        # unit says ([PB-DIALOG-DEFAULT-UNITS]): radians here, cm below.
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))
        # A string input so it accepts expressions.
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(0))
        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body',
            True, '', False)

        # Parent Component is last; the spec pins the pre-selection outcome
        # (the root component starts selected). addSelection is the API for
        # adding to a selection input; Fusion documents it as invalid during
        # commandCreated, which is when configure runs — see the emit report.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the component the gear is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    """Data carrier for one spur gear build (step 1). The field names are
    public API; subclasses read and add to them."""

    def __init__(self):
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.extrusionExtent = adsk.fusion.BRepFace.cast(None)
        self.toothProfileIsEmbedded = False


class SpurGearInvoluteToothDesignGenerator:
    """Draws the involute tooth profile into a sketch (steps 5 and 9).

    The parent is the generator (or a VirtualSpurProxy when bevel borrows
    this drawer, [PB-PRECOMPUTED-MODE]); on the drawing paths only the keys
    the proxy serves are read: Module, ToothNumber, PressureAngle, the
    Pitch/Base/Root/Tip circle diameters and radii, and InvoluteSteps.
    """

    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        # Retained but never used by drawTooth: the live rotation always
        # comes from draw()'s runtime argument. Reading self.toothAngle
        # instead draws a flat tooth and kills the helical loft's twist.
        self.toothAngle = angle
        # [SPUR-F-LOCAL-ORIGIN] the movable local origin: a fresh SketchPoint
        # at (0, 0, 0), never the sketch's own origin point.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self.spineDimension = adsk.fusion.SketchAngularDimension.cast(None)
        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
        self.baseCircle = adsk.fusion.SketchCircle.cast(None)
        self.pitchCircle = adsk.fusion.SketchCircle.cast(None)

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name):
        return self.parent.getParameter(name).value

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        """The involute point at the radius where the unrolled string reaches
        intersectionRadius, or None inside the base circle. The curve
        parameter is tan(alpha), NOT inv(alpha) = tan(alpha) - alpha; using
        the involute function mis-parameterises the flank (step 5)."""
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    def draw(self, anchorPoint, angle=0):
        """Step 9: circles, tooth, anchoring, and — very last, only when
        angle != 0 — the confirming angular dimension [SPUR-F-ROTATE-CONFIRM].
        The anchoring lives here, not in buildSketches: helical calls draw
        directly on its twisted sketch and relies on this one call."""
        self.drawCircles()
        self.drawTooth(angle)
        # [SPUR-F-ANCHOR-CHAIN] everything above hangs off the local origin;
        # this one coincidence to the re-projected Tools anchor slides the
        # whole tooth onto the user's anchor.
        projectedAnchor = adsk.fusion.SketchPoint.cast(
            self.sketch.project(anchorPoint).item(0))
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)
        if angle != 0:
            # Set only after the entire constraint network exists.
            self.spineDimension.parameter.value = angle

    def drawCircles(self):
        """The four circles, all centred by sharing the one local-origin
        SketchPoint ([PB-SHARE-XOR-COINCIDENT], [SPUR-F-SHARED-ADJACENCY]):
        root solid, tip/base/pitch construction, each with a driving diameter
        dimension and an along-path label."""
        sketch = self.sketch
        origin = self.anchorPoint
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        size = tipRadius - rootRadius

        circles = sketch.sketchCurves.sketchCircles
        self.rootCircle = circles.addByCenterRadius(origin, rootRadius)
        self.tipCircle = circles.addByCenterRadius(origin, tipRadius)
        self.tipCircle.isConstruction = True
        self.baseCircle = circles.addByCenterRadius(origin, baseRadius)
        self.baseCircle.isConstruction = True
        self.pitchCircle = circles.addByCenterRadius(origin, pitchRadius)
        self.pitchCircle.isConstruction = True

        for name, circle, radius in (
                ('Root Circle', self.rootCircle, rootRadius),
                ('Tip Circle', self.tipCircle, tipRadius),
                ('Base Circle', self.baseCircle, baseRadius),
                ('Pitch Circle', self.pitchCircle, pitchRadius)):
            # Driving is the default; never pass isDriven ([PB-DRIVING-DIM]).
            # The text point is off-centre, near the curve ([PB-RADIAL-DIM]).
            sketch.sketchDimensions.addDiameterDimension(
                circle,
                adsk.core.Point3D.create(radius * 0.7071, radius * 0.7071, 0))
            # Along-path label ([PB-SKETCH-TEXT]).
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True,
                adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)

    def drawTooth(self, angle=0):
        """One involute tooth, drawn already at its final rotation (step 9).
        Rotates by the angle argument, never self.toothAngle."""
        sketch = self.sketch
        origin = self.anchorPoint
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        # 9.1 Sample the flank endpoint-inclusively, first sample exactly at
        # the base radius, last exactly at the tip radius; the start is NOT
        # clamped to the root circle — the embedded case is detected from
        # where the flank start lands. Samples inside the base circle drop.
        mirrored = []
        for i in range(steps):
            r = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            sample = self.calculateInvolutePoint(baseRadius, r)
            if sample is None:
                continue
            # 9.2 Mirror across +X (negate y) before rotating: the raw
            # involute's angular position grows with radius, which as a left
            # flank makes a tooth wider at the tip.
            mirrored.append((sample.x, -sample.y))

        # 9.3 Centre the tooth on the analytic pitch crossing of the MIRRORED
        # flank (atan2(-py, px)), computed rather than interpolated so the
        # tooth lands right at any sample count.
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None:
            raise ValueError(
                'drawTooth: no involute point at the pitch radius — pitch '
                'radius {} is inside base radius {}'.format(pitchRadius, baseRadius))
        rotateAngle = math.pi / (2 * toothNumber) - math.atan2(-pitchPoint.y, pitchPoint.x)

        # 9.4 Rotate the mirrored samples (left flank), mirror across X for
        # the right flank, then rotate BOTH flanks by the requested angle.
        leftPoints = []
        rightPoints = []
        for x, y in mirrored:
            lx, ly = _rotate(x, y, rotateAngle)
            rx, ry = lx, -ly
            leftPoints.append(_rotate(lx, ly, angle))
            rightPoints.append(_rotate(rx, ry, angle))

        # 9.5 Each flank is a fitted spline through its point collection.
        leftCollection = adsk.core.ObjectCollection.create()
        for x, y in leftPoints:
            leftCollection.add(adsk.core.Point3D.create(x, y, 0))
        leftSpline = sketch.sketchCurves.sketchFittedSplines.add(leftCollection)
        rightCollection = adsk.core.ObjectCollection.create()
        for x, y in rightPoints:
            rightCollection.add(adsk.core.Point3D.create(x, y, 0))
        rightSpline = sketch.sketchCurves.sketchFittedSplines.add(rightCollection)

        last = len(leftPoints) - 1
        ca, sa = math.cos(angle), math.sin(angle)

        # 9.6 Tooth-top arc [SPUR-F-TOOTHTOP-ARC]: the shared origin as
        # centre, the flank splines' end SketchPoints passed directly, and no
        # diameter dimension (a free centre plus a diameter leaves the inward
        # bulge available; the shared centre also makes the last rib's
        # perpendicular redundant).
        toothTopPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipRadius * ca, tipRadius * sa, 0))
        sketch.geometricConstraints.addCoincident(toothTopPoint, self.tipCircle)
        sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            origin, rightSpline.fitPoints.item(last), leftSpline.fitPoints.item(last))

        # 9.7 Spine and +X reference [SPUR-F-SPINE], built for EVERY angle, 0
        # included. The far reference endpoint is pinned by two axis
        # dimensions, not by coincidence onto the tip circle, whose extreme-x
        # touch is numerically unstable. The signed angular dimension is what
        # forbids the mirrored answer; a plain horizontal has no direction.
        spineLine = sketch.sketchCurves.sketchLines.addByTwoPoints(origin, toothTopPoint)
        spineLine.isConstruction = True
        refEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        sketch.sketchDimensions.addDistanceDimension(
            origin, refEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius * 0.5, -tipRadius * 0.1, 0))
        sketch.sketchDimensions.addDistanceDimension(
            origin, refEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius * 1.1, -tipRadius * 0.05, 0))
        refLine = sketch.sketchCurves.sketchLines.addByTwoPoints(origin, refEnd)
        refLine.isConstruction = True
        # Text on the bisector of the intended angle so Fusion selects it,
        # not its supplement ([PB-ANGULAR-DIM]).
        half = angle / 2
        self.spineDimension = sketch.sketchDimensions.addAngularDimension(
            refLine, spineLine,
            adsk.core.Point3D.create(0.6 * tipRadius * math.cos(half),
                                     0.6 * tipRadius * math.sin(half), 0))

        # 9.8 Ribs [SPUR-F-RIBS], one per fit-point index, first and last
        # included, in exactly this per-rib order — any other over-constrains.
        acrossVertical = abs(ca) >= abs(sa)
        prevPoint = origin
        prevT = 0.0
        for i in range(len(leftPoints)):
            leftFit = leftSpline.fitPoints.item(i)
            rightFit = rightSpline.fitPoints.item(i)
            # 1. The rib, sharing the fit points; construction.
            ribLine = sketch.sketchCurves.sketchLines.addByTwoPoints(leftFit, rightFit)
            ribLine.isConstruction = True
            lx, ly = leftPoints[i]
            rx, ry = rightPoints[i]
            # 2. An AXIS dimension across the rib, created with the points
            # already seeded on their sides; an aligned dimension lets the
            # flanks swap and mirror the tooth ([PB-DIM-VALUE-SEMANTICS]).
            ribText = adsk.core.Point3D.create(
                (lx + rx) / 2 - 0.05 * tipRadius * sa,
                (ly + ry) / 2 + 0.05 * tipRadius * ca, 0)
            if acrossVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                    ribText)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                    ribText)
            # 3. A fresh midpoint seeded ON the spine at the foot of the left
            # fit point.
            t = lx * ca + ly * sa
            midPoint = sketch.sketchPoints.add(
                adsk.core.Point3D.create(t * ca, t * sa, 0))
            # 4-6. Coincident first, then midpoint, then perpendicular —
            # skipped for the last rib, which the tooth-top arc already
            # implies; keeping it throws VCS_SKETCH_OVER_CONSTRAINTS.
            sketch.geometricConstraints.addCoincident(midPoint, spineLine)
            sketch.geometricConstraints.addMidPoint(midPoint, ribLine)
            if i != last:
                sketch.geometricConstraints.addPerpendicular(spineLine, ribLine)
            # Chain the midpoints down the spine, the first from the local
            # origin; without the origin-to-first dimension the chain slides
            # along the spine as a unit.
            chainText = adsk.core.Point3D.create(
                ((prevT + t) / 2) * ca + 0.08 * tipRadius * sa,
                ((prevT + t) / 2) * sa - 0.08 * tipRadius * ca, 0)
            if acrossVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    prevPoint, midPoint,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                    chainText)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    prevPoint, midPoint,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                    chainText)
            prevPoint = midPoint
            prevT = t

        # 9.9 Close the tooth at the root [SPUR-F-FLANK-ROOT]: strict
        # less-than, raw values, no tolerance; exact equality counts as
        # non-embedded and draws a zero-length stub. The tooth generator has
        # no ctx, so the embedded flag goes to the parent.
        firstX, firstY = leftPoints[0]
        firstRadius = math.hypot(firstX, firstY)
        embedded = firstRadius < rootRadius
        self.parent._lastToothEmbedded = embedded
        if not embedded:
            self._drawFlankToRoot(leftSpline.fitPoints.item(0), firstX, firstY,
                                  rootRadius, firstRadius, tipRadius)
            self._drawFlankToRoot(rightSpline.fitPoints.item(0),
                                  rightPoints[0][0], rightPoints[0][1],
                                  rootRadius, firstRadius, tipRadius)

    def _drawFlankToRoot(self, flankStartPoint, startX, startY, rootRadius,
                         firstRadius, tipRadius):
        """[SPUR-F-FLANK-ROOT] one radial stub, sharing the spline's start
        point, its root end seeded at its exact computed position on the root
        circle and pinned by exactly two axis dimensions from the local
        origin. The dimension values are only the unsigned abs magnitudes: a
        negative parameter.value flips the point to the origin's other side
        ([PB-DIM-VALUE-SEMANTICS]). Root-end-on-circle plus origin-on-line is
        also satisfied on the far side of the gear, and the stub becomes a
        chord across it."""
        sketch = self.sketch
        rootEndX = rootRadius * startX / firstRadius
        rootEndY = rootRadius * startY / firstRadius
        rootEnd = sketch.sketchPoints.add(
            adsk.core.Point3D.create(rootEndX, rootEndY, 0))
        sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flankStartPoint)
        dimH = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(rootEndX + 0.05 * tipRadius,
                                     rootEndY - 0.05 * tipRadius, 0))
        dimH.parameter.value = abs(rootEndX)
        dimV = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(rootEndX - 0.05 * tipRadius,
                                     rootEndY + 0.05 * tipRadius, 0))
        dimV.parameter.value = abs(rootEndY)

    def drawBore(self, anchorPoint, diameter):
        """Step 18: re-project the anchor into this sketch, draw the solid
        bore circle centred on the projection by sharing it, and give it a
        driving diameter dimension. Returns the projected SketchPoint so the
        caller can ground this generator's stray local origin on it."""
        sketch = self.sketch
        projectedAnchor = adsk.fusion.SketchPoint.cast(
            sketch.project(anchorPoint).item(0))
        radius = diameter / 2.0
        center = projectedAnchor.geometry
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, radius)
        sketch.sketchDimensions.addDiameterDimension(
            circle,
            adsk.core.Point3D.create(center.x + radius * 0.7071,
                                     center.y + radius * 0.7071, 0))
        return projectedAnchor


class SpurGearGenerator(Generator):
    """The orchestrator (steps 3-4). Method boundaries are public API:
    helical/herringbone override at them and call super() at specific
    points."""

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self._lastToothEmbedded = False
        self.toolsSketch = None
        self.boreSketch = None
        self.plane = None
        self.anchorPoint = None
        self._normalizedPlane = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self):
        return SpurGearGenerationContext()

    def chamferWantEdges(self) -> int:
        """Front-face edge count chamferTooth selects on; helical overrides
        only this count."""
        return 6

    def filletHelixFactorExpression(self) -> str:
        """Hook consumed only in registerDerivedParameters' FilletRadius
        splice; createFillets reads the resulting parameter's numeric value,
        never this hook."""
        return '1'

    def generateName(self) -> str:
        # The parameters' .expression strings, not .value, so units show
        # through.
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        """Step 3: selections first — anything that creates the child
        occurrence (the first addParameter does) can drop selection inputs
        holding entities in another component."""
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception(
                'processInputs: expected exactly 1 Parent Component selection, '
                'got {}'.format(len(parents)))
        occurrence = adsk.fusion.Occurrence.cast(parents[0])
        if occurrence:
            self.parentComponent = occurrence.component
        else:
            self.parentComponent = adsk.fusion.Component.cast(parents[0])

        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception(
                'processInputs: expected exactly 1 Target Plane selection, '
                'got {}'.format(len(planes)))
        self.plane = planes[0]

        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception(
                'processInputs: expected exactly 1 Anchor Point selection, '
                'got {}'.format(len(anchors)))
        self.anchorPoint = anchors[0]

        # Each remaining input read with the helper matching its declared
        # type ([PB-INPUT-READ]); get_value returns a ready ValueInput and
        # raises on a bad expression ([PB-GET-VALUE-CONTRACT]). Never
        # get_value on the checkbox: BoolValueCommandInput has no expression.
        module = get_value(inputs, INPUT_ID_MODULE, '')
        toothNumber = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        pressureAngle = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        boreDiameter = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        thickness = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        chamferTooth = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        sketchOnly = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)

        # Module is unitless — NOT 'mm' — so generateName renders M=1 and the
        # derived mm expressions accept it.
        self.addParameter(PARAM_MODULE, module, '', 'module of the gear')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'number of teeth')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad', 'pressure angle')
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm', 'bore diameter')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm', 'gear thickness')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm', 'tooth chamfer distance')
        # The framework reads booleans back with getParameterAsBoolean.
        self.addParameter(PARAM_SKETCH_ONLY,
                          adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
                          '', 'generate sketches only (1/0)')

        self.addExtraPrimaryParameters(inputs)
        self.registerDerivedParameters()

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        """[SPUR-EXTRA-PARAMS] a no-op on the spur base: subclasses register
        their own primary parameters here, before any derived expression
        references them."""

    def registerDerivedParameters(self):
        """Step 3.5: derived parameters as live expressions, in dependency
        order, mm unless noted."""
        pn = self.parameterName
        self.addParameter(
            PARAM_PITCH_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} * {}'.format(pn(PARAM_MODULE), pn(PARAM_TOOTH_NUMBER))),
            'mm', 'pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(pn(PARAM_PITCH_DIAMETER))),
            'mm', 'pitch circle radius')
        self.addParameter(
            PARAM_BASE_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} * cos({})'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_PRESSURE_ANGLE))),
            'mm', 'base circle diameter')
        self.addParameter(
            PARAM_BASE_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(pn(PARAM_BASE_DIAMETER))),
            'mm', 'base circle radius')
        self.addParameter(
            PARAM_ROOT_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} - 2.5 * {}'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_MODULE))),
            'mm', 'root circle diameter (dedendum 1.25 module)')
        self.addParameter(
            PARAM_ROOT_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(pn(PARAM_ROOT_DIAMETER))),
            'mm', 'root circle radius')
        self.addParameter(
            PARAM_TIP_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} + 2 * {}'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_MODULE))),
            'mm', 'tip circle diameter (addendum 1 module)')
        self.addParameter(
            PARAM_TIP_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(pn(PARAM_TIP_DIAMETER))),
            'mm', 'tip circle radius')
        self.addParameter(
            PARAM_INVOLUTE_STEPS,
            adsk.core.ValueInput.createByString('15'),
            '', 'involute flank sample count')
        # Pre-computed in Python: Fusion's expression engine refuses to mix
        # unitless tan() output with a radian value. Registered unitless, not
        # 'rad' — the next parameter multiplies it by a length, and a 'rad'
        # factor makes that product mm*rad, which Fusion rejects with
        # RuntimeError: Invalid expression.
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        spaceAngle = (math.pi / toothNumberValue
                      - 2 * (math.tan(pressureAngleValue) - pressureAngleValue))
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE,
            adsk.core.ValueInput.createByReal(spaceAngle),
            '', 'tooth space angle at the root (radian magnitude, unitless)')
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC,
            adsk.core.ValueInput.createByString(
                '{} * {}'.format(pn(PARAM_ROOT_RADIUS), pn(PARAM_TOOTH_SPACE_ANGLE))),
            'mm', 'tooth space arc length at the root')
        self.addParameter(
            PARAM_FILLET_CLEARANCE,
            adsk.core.ValueInput.createByString('0.9'),
            '', 'root fillet clearance factor')
        self.addParameter(
            PARAM_FILLET_RADIUS,
            adsk.core.ValueInput.createByString(
                '({} / 2) * {} * {}'.format(pn(PARAM_TOOTH_SPACE_ARC),
                                            pn(PARAM_FILLET_CLEARANCE),
                                            self.filletHelixFactorExpression())),
            'mm', 'root fillet radius')

    def generate(self, inputs: adsk.core.CommandInputs):
        futil.log('SpurGearGenerator: reading inputs and registering parameters')
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()
        self._normalizeTargetPlane()
        ctx = self.newContext()
        ctx.plane = adsk.fusion.ConstructionPlane.cast(self.plane)
        futil.log('SpurGearGenerator: preparing tools')
        self.prepareTools(ctx)
        futil.log('SpurGearGenerator: building the main gear body')
        self.buildMainGearBody(ctx)
        futil.log('SpurGearGenerator: building the bore')
        self.buildBore(ctx)
        # Unconditionally the very last action, after buildBore: buildBore
        # re-projects ctx.anchorPoint from the Tools sketch, and projection
        # fails once that sketch is hidden.
        self.cleanup(ctx)

    def _normalizeTargetPlane(self):
        """Step 6: if the user picked a planar face, replace self.plane with
        a coplanar construction plane so profile detection never sees the
        picked face's native profile. self.plane stays readable by
        subclasses."""
        if adsk.fusion.ConstructionPlane.cast(self.plane):
            return
        constructionPlanes = self.getComponent().constructionPlanes
        planeInput = constructionPlanes.createInput()
        # The offset is a ValueInput, never a bare number, which is a runtime
        # TypeError ([PB-CONSTRUCTION-PLANES]).
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
        self._normalizedPlane = constructionPlanes.add(planeInput)
        self.plane = self._normalizedPlane

    def prepareTools(self, ctx: SpurGearGenerationContext):
        """Steps 7-8: the Tools sketch and the Extrusion End Plane."""
        # The Tools sketch draws nothing else; its projection is the
        # canonical handle of the anchor chain [SPUR-F-ANCHOR-CHAIN]. It
        # stays visible until cleanup — hiding it earlier breaks the bore's
        # re-projection.
        sketch = self.createSketchObject('Tools', plane=self.plane)
        sketch.isVisible = True
        ctx.anchorPoint = adsk.fusion.SketchPoint.cast(
            sketch.project(self.anchorPoint).item(0))
        self.toolsSketch = sketch

        # The Extrusion End Plane's only purpose is to be the to-entity
        # target of the tooth and body extrudes, so both end on the same
        # face. It stays visible while the extrudes run; step 20 hides it
        # with isLightBulbOn.
        constructionPlanes = self.getComponent().constructionPlanes
        planeInput = constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, self.getParameterAsValueInput(PARAM_THICKNESS))
        endPlane = constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 10: sketch-only short circuit. buildBore and cleanup still
            # run from generate with their own guards.
            futil.log('SpurGearGenerator: sketch-only mode, no body is built')
            ctx.gearProfileSketch.isVisible = True
            return
        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        """Step 9: the Gear Profile sketch, drawn by the tooth generator."""
        sketch = self.createSketchObject('Gear Profile', plane=self.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch
        toothGenerator = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGenerator.draw(ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    def buildTooth(self, ctx: SpurGearGenerationContext):
        """Step 11: extrude the tooth section to the Extrusion End Plane."""
        futil.log('SpurGearGenerator: extruding the tooth')
        sketch = ctx.gearProfileSketch
        # The tooth section by its curve counts; the helper raises when
        # nothing matches ([PB-PROFILE-MATCH]).
        profile = find_profile_by_curve_counts(
            sketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2)
        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extentDefinition = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)
        self.chamferTooth(ctx)

    def chamferTooth(self, ctx: SpurGearGenerationContext):
        """Step 12: chamfer the tooth's front-face edges, except the root
        arc. (Known, accepted: an embedded profile's front face has 4 edges
        while chamferWantEdges() stays 6, so chamfering an embedded spur
        tooth raises — users disable chamfer there.)"""
        chamferValue = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferValue == 0:
            return
        futil.log('SpurGearGenerator: chamfering the tooth')
        wantEdges = self.chamferWantEdges()
        sketchPlane = adsk.fusion.ConstructionPlane.cast(
            ctx.gearProfileSketch.referencePlane).geometry
        frontFace = None
        for face in ctx.toothBody.faces:
            # A single conjunction predicate: the edge count AND coplanarity
            # with the sketch plane; never a partial match.
            if face.edges.count != wantEdges:
                continue
            facePlane = adsk.core.Plane.cast(face.geometry)
            if facePlane is None:
                continue
            if sketchPlane.isCoPlanarTo(facePlane):
                frontFace = face
                break
        if frontFace is None:
            raise Exception(
                'chamferTooth: no face of the tooth body has {} edges and is '
                'coplanar with the sketch plane (faces: {})'.format(
                    wantEdges, ctx.toothBody.faces.count))
        # Every front-face edge except the root arc, identified by radius,
        # not size — chamfering the root arc eats the neighbouring tooth.
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        edges = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            geometry = edge.geometry
            if geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                arc = adsk.core.Arc3D.cast(geometry)
                if abs(arc.radius - rootRadius) < 0.001:
                    continue
            edges.add(edge)
        chamferFeatures = self.getComponent().features.chamferFeatures
        chamferInput = chamferFeatures.createInput2()
        # The chamfer-side shape of [PB-FILLET-CHAMFER]: the edge set goes on
        # the input's chamferEdgeSets collection.
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferValue), False)
        chamferFeatures.add(chamferInput)

    def buildBody(self, ctx: SpurGearGenerationContext):
        """Steps 13-14: extrude the gear body disc, then capture the Gear
        Center axis and the extrusion-extent face from it."""
        futil.log('SpurGearGenerator: extruding the gear body')
        sketch = ctx.gearProfileSketch
        # The solid disc inside the root circle: its boundary is exactly the
        # 2 arcs the tooth's contact points split the root circle into. Not
        # an annulus; the tip circle is construction geometry and bounds
        # nothing.
        profile = find_profile_by_curve_counts(sketch, arcs=2)
        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extentDefinition = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude body'
        body = extrude.bodies.item(0)
        body.name = 'Gear Body'
        ctx.gearBody = body

        # Step 14: classify the new body's faces; raise if either capture is
        # missing — never let a failed search surface three calls later
        # ([PB-EMPTY-RESULT]).
        sketchPlane = adsk.fusion.ConstructionPlane.cast(
            ctx.gearProfileSketch.referencePlane).geometry
        centerAxis = None
        extrusionExtent = None
        for face in extrude.bodies.item(0).faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if centerAxis is None:
                    constructionAxes = self.getComponent().constructionAxes
                    axisInput = constructionAxes.createInput()
                    axisInput.setByCircularFace(face)
                    axis = constructionAxes.add(axisInput)
                    axis.name = 'Gear Center'
                    axis.isLightBulbOn = False
                    centerAxis = axis
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                facePlane = adsk.core.Plane.cast(face.geometry)
                # The far cap is the only face parallel to but not coplanar
                # with the sketch plane.
                if (sketchPlane.isParallelToPlane(facePlane)
                        and not sketchPlane.isCoPlanarTo(facePlane)):
                    extrusionExtent = face
        if centerAxis is None:
            raise Exception(
                'buildBody: no cylindrical face on the gear body to build '
                'the Gear Center axis from')
        if extrusionExtent is None:
            raise Exception(
                'buildBody: no planar face parallel to but not coplanar with '
                'the sketch plane on the gear body')
        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent

    def patternTeeth(self, ctx: SpurGearGenerationContext):
        """Steps 15-16: circular-pattern the tooth body about the Gear
        Center axis, then combine-join the pattern's bodies into the gear
        body; ends by calling createFillets."""
        futil.log('SpurGearGenerator: patterning the teeth')
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(ctx.toothBody)
        circularPatternFeatures = self.getComponent().features.circularPatternFeatures
        patternInput = circularPatternFeatures.createInput(bodies, ctx.centerAxis)
        # Pin all three inputs explicitly ([PB-CIRCULAR-PATTERN]).
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = circularPatternFeatures.add(patternInput)

        # Step 16: one Combine-Join. pattern.bodies already includes the
        # original tooth body — never re-add it — and createInput rejects a
        # raw BRepBodies, so copy into a fresh collection
        # ([PB-PATTERN-BODIES]).
        futil.log('SpurGearGenerator: combining the teeth into the body')
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))
        combineFeatures = self.getComponent().features.combineFeatures
        combineInput = combineFeatures.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineFeatures.add(combineInput)

        self.createFillets(ctx)

    def createFillets(self, ctx: SpurGearGenerationContext):
        """Step 17: round the corner where each root valley floor meets a
        tooth flank — the structurally loaded corner, not the cosmetic
        front/back rims."""
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return
        futil.log('SpurGearGenerator: filleting the root corners')
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)
        edges = adsk.core.ObjectCollection.create()
        # Every cylindrical face at the root radius: the pattern-and-combine
        # usually splits the root cylinder into one patch per valley.
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            cylinder = adsk.core.Cylinder.cast(face.geometry)
            if abs(cylinder.radius - rootRadius) > 0.001:
                continue
            # Keep only the AXIAL straight edges. Directions come from the
            # geometry endpoints — parameter 0 of an edge evaluator need not
            # lie in the edge's range and Fusion raises on it. The circular
            # end-cap rim edges fail the direction test. Use exactly this
            # tolerance: a tighter one drops slightly-off tessellated edges
            # and leaves fillets missing.
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                line = adsk.core.Line3D.cast(edge.geometry)
                direction = line.startPoint.vectorTo(line.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01:
                    edges.add(edge)
        if edges.count == 0:
            # An empty edge set must not reach filletFeatures.add.
            return
        filletFeatures = self.getComponent().features.filletFeatures
        filletInput = filletFeatures.createInput()
        # The edge set goes on the input ITSELF — there is no edgeSetInputs
        # member; that is the chamfer-side shape ([PB-FILLET-CHAMFER]).
        # isTangentChain=False so Fusion cannot chain past the corner.
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        filletFeatures.add(filletInput)

    def buildBore(self, ctx: SpurGearGenerationContext):
        """Steps 18-19: the Bore Profile sketch and the bore cut."""
        # In sketch-only mode ctx.gearBody and ctx.extrusionExtent were never
        # set — do not rely on the bore being 0.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        futil.log('SpurGearGenerator: cutting the bore')
        boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch
        toothGenerator = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        projectedAnchor = toothGenerator.drawBore(ctx.anchorPoint, boreDiameter)
        # The constructor's stray local-origin SketchPoint at (0,0,0) is
        # faithful behaviour — keep it, and ground it on that same
        # projection. Grounding it on the sketch's own origin pins it to the
        # plane rather than the gear, and [PB-CIRCLE-CENTER] records a solver
        # failure from constraining to the sketch origin. Ungrounded, the
        # point keeps two free DOF and the sketch never fully constrains.
        boreSketch.geometricConstraints.addCoincident(
            toothGenerator.anchorPoint, projectedAnchor)

        # Step 19: extrude-cut to ctx.extrusionExtent — the far end-cap face
        # captured in step 14 — so the bore pierces the whole body regardless
        # of Thickness.
        profile = boreSketch.profiles.item(0)
        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extentDefinition = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(
            extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudeFeatures.add(extrudeInput)

    def cleanup(self, ctx: SpurGearGenerationContext):
        """Step 20 [SPUR-F-CLEANUP]: the last action of generate, both modes,
        never guarded at the call site. isVisible hides sketches;
        isLightBulbOn hides construction geometry — isVisible has no effect
        on it ([PB-HIDE-AFTER-USE]). Each entity is guarded individually: the
        Gear Center axis and Bore Profile sketch do not exist in sketch-only
        mode, and the bore sketch does not exist when no bore was cut."""
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self._normalizedPlane is not None:
            self._normalizedPlane.isLightBulbOn = False
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Sketch-only mode leaves Tools and Gear Profile visible — that
            # is the mode's whole point.
            return
        if self.toolsSketch is not None:
            self.toolsSketch.isVisible = False
        if ctx.gearProfileSketch is not None:
            ctx.gearProfileSketch.isVisible = False
        if self.boreSketch is not None:
            self.boreSketch.isVisible = False
