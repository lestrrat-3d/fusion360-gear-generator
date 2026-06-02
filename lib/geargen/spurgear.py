import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import *
from .base import *
from .utilities import *

# ---------------------------------------------------------------------------
# Dialog input ids and registered user-parameter names. These literal strings
# are part of the reproduced surface (they appear in the dialog and in the
# post-generation user-parameter table; saved designs reference them), so they
# are kept verbatim per the spec.
# ---------------------------------------------------------------------------

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

PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
PARAM_PRESSURE_ANGLE = 'PressureAngle'
PARAM_BORE_DIAMETER = 'BoreDiameter'
PARAM_THICKNESS = 'Thickness'
PARAM_CHAMFER_TOOTH = 'ChamferTooth'
PARAM_SKETCH_ONLY = 'SketchOnly'

PARAM_PITCH_CIRCLE_DIAMETER = 'PitchCircleDiameter'
PARAM_PITCH_CIRCLE_RADIUS = 'PitchCircleRadius'
PARAM_BASE_CIRCLE_DIAMETER = 'BaseCircleDiameter'
PARAM_BASE_CIRCLE_RADIUS = 'BaseCircleRadius'
PARAM_ROOT_CIRCLE_DIAMETER = 'RootCircleDiameter'
PARAM_ROOT_CIRCLE_RADIUS = 'RootCircleRadius'
PARAM_TIP_CIRCLE_DIAMETER = 'TipCircleDiameter'
PARAM_TIP_CIRCLE_RADIUS = 'TipCircleRadius'
PARAM_INVOLUTE_STEPS = 'InvoluteSteps'
PARAM_TOOTH_SPACE_ANGLE_AT_ROOT = 'ToothSpaceAngleAtRoot'
PARAM_TOOTH_SPACE_ARC_AT_ROOT = 'ToothSpaceArcAtRoot'
PARAM_FILLET_CLEARANCE = 'FilletClearance'
PARAM_FILLET_RADIUS = 'FilletRadius'


class SpurGearCommandInputsConfigurator:
    """Adds the Spur Gear dialog inputs in the order the spec lists them."""

    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # --- Selection inputs ---
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the gear front face is sketched on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the gear center is aligned with')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1)

        # --- Numeric / boolean inputs ---
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '',
            adsk.core.ValueInput.createByReal(17))
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByString('20 deg'))
        # Bore Diameter is a string value input so it accepts expressions.
        inputs.addStringValueInput(
            INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body',
            True, '', False)

        # Parent Component last: the default (root) is correct for most uses.
        componentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component that will contain the new Spur Gear')
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        componentInput.setSelectionLimits(1)
        componentInput.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    """Plain data carrier passed between generation steps. These field names
    are public API: helical/herringbone subclasses read them by name."""

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
        # Set in step 1 when a normalized plane is created, so cleanup can hide
        # it. Stays None when the user's selection was already a plane.
        self.normalizedPlane = adsk.fusion.ConstructionPlane.cast(None)


class SpurGearInvoluteToothDesignGenerator:
    """Draws the four gear circles and a single involute tooth into a sketch,
    then anchors the whole drawing onto the user's anchor point."""

    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle

        # The movable local origin: a fresh SketchPoint at (0, 0, 0). Every
        # piece of geometry is drawn relative to this, and at the very end it is
        # made coincident with the projected anchor so the tooth slides onto it.
        # Must NOT be sketch.originPoint (immutable, can't be coincided).
        self.anchorPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))

    # ----- parameter helpers -----
    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name):
        return self.parent.getParameter(name).value

    # ----- circles -----
    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions

        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)

        # Label height/size: difference between tip and root radii (cm).
        size = tipRadius - rootRadius

        def makeCircle(name, radius, construction):
            # Share the local origin directly as the center (do NOT pass
            # .geometry and add a center coincident - that over-constrains).
            circle = circles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = construction
            # Driving diameter dimension (no isDriven argument).
            dims.addDiameterDimension(
                circle,
                adsk.core.Point3D.create(radius, radius, 0))
            self._labelCircle(circle, name, radius, size)
            return circle

        # Root Circle is solid; the others are construction.
        self.rootCircle = makeCircle('Root Circle', rootRadius, False)
        self.tipCircle = makeCircle('Tip Circle', tipRadius, True)
        self.baseCircle = makeCircle('Base Circle', baseRadius, True)
        self.pitchCircle = makeCircle('Pitch Circle', pitchRadius, True)

    def _labelCircle(self, circle, name, radius, size):
        text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
        textInput = self.sketch.sketchTexts.createInput2(text, size)
        textInput.setAsAlongPath(
            circle, True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        self.sketch.sketchTexts.add(textInput)

    # ----- involute math -----
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        # The involute point at the radius where the unrolled string reaches
        # intersectionRadius. Returns None when intersectionRadius sits inside
        # the base circle (no valid involute - the parameter is non-positive).
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        # Standard parametric involute at parameter t = tan(alpha):
        #   x = rb*(cos t + t sin t), y = rb*(sin t - t cos t)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    # ----- tooth -----
    def drawTooth(self, angle=0):
        sketch = self.sketch

        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        involuteSteps = int(round(self.getParameterValue(PARAM_INVOLUTE_STEPS)))

        # --- 1. sample involute points from base to tip in equal radial steps;
        # drop any whose involute parameter would be non-positive (inside base).
        rawPoints = []
        for i in range(involuteSteps):
            if involuteSteps > 1:
                r = baseRadius + (tipRadius - baseRadius) * i / (involuteSteps - 1)
            else:
                r = baseRadius
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                continue
            rawPoints.append(p)

        # --- 2. mirror across +X (negate y) so the spiral matches a left
        # flank's shape; the rotation below lifts it back up into +Y.
        mirrored = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in rawPoints]

        # --- 3. rotation so the mirrored involute's pitch-circle crossing lands
        # at +pi/(2*ToothNumber) above +X. Use the analytic pitch crossing.
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        # The pitch-crossing angle of the mirrored curve (negate y component).
        mirroredPitchAngle = -math.atan2(pitchPoint.y, pitchPoint.x)
        targetAngle = math.pi / (2 * toothNumber)
        rotate_angle = targetAngle - mirroredPitchAngle

        # --- 4. rotate the mirrored points -> left flank; mirror across X for
        # the right flank.
        def rotate(points, a):
            ca = math.cos(a)
            sa = math.sin(a)
            out = []
            for p in points:
                out.append(adsk.core.Point3D.create(
                    p.x * ca - p.y * sa,
                    p.x * sa + p.y * ca, 0))
            return out

        leftPoints = rotate(mirrored, rotate_angle)
        rightPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in leftPoints]

        # Fold the requested rotation `angle` into the drawing: rotate the whole
        # (symmetric, +X-centered) tooth by `angle` so the seed tooth is drawn
        # directly at its final angular position. angle==0 and angle!=0 then
        # share the same centering baseline and differ by exactly `angle`, so a
        # helical loft between an angle=0 profile and an angle=helixAngle profile
        # twists by exactly the helix angle. Drawing the tooth already at `angle`
        # also means the spine angular dimension below merely *confirms* the
        # position instead of forcing a rotation the solver could resolve to the
        # wrong (~180-off) branch. For spur (angle==0) this is a no-op.
        if angle != 0:
            leftPoints = rotate(leftPoints, angle)
            rightPoints = rotate(rightPoints, angle)

        # --- 5. draw the two flanks as fitted splines through the collections.
        leftCollection = adsk.core.ObjectCollection.create()
        for p in leftPoints:
            leftCollection.add(p)
        rightCollection = adsk.core.ObjectCollection.create()
        for p in rightPoints:
            rightCollection.add(p)

        splines = sketch.sketchCurves.sketchFittedSplines
        leftSpline = splines.add(leftCollection)
        rightSpline = splines.add(rightCollection)

        # --- 6. tooth-top arc (minimal, exactly-constrained set).
        self._drawToothTopArc(leftSpline, rightSpline, tipRadius, angle)

        # --- 7. spine (construction): shares the local origin and tooth top.
        self._drawSpine(angle, tipRadius)

        # --- 8. ribs between matching left/right flank fit points.
        self._drawRibs(leftSpline, rightSpline, angle)

        # --- 9. flank-to-root lines (common case) or embedded profile.
        self._drawFlankToRoot(leftSpline, rightSpline, rootRadius)

        # The helical rotation is driven, not constructed: apply the tilt as the
        # very last action, after the whole constraint network exists.
        if angle != 0 and self.spineAngularDimension is not None:
            self.spineAngularDimension.parameter.value = angle

    def _drawToothTopArc(self, leftSpline, rightSpline, tipRadius, angle=0):
        sketch = self.sketch
        constraints = sketch.geometricConstraints

        leftEnd = leftSpline.endSketchPoint
        rightEnd = rightSpline.endSketchPoint

        # Materialize a tooth-top point at the tip, rotated by `angle` to match
        # the rotated flanks, and coincident to the tip circle. This is the ONLY
        # coincidence here - do not constrain the arc's centerSketchPoint to
        # anything.
        toothTop = sketch.sketchPoints.add(
            adsk.core.Point3D.create(
                tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0))
        constraints.addCoincident(toothTop, self.tipCircle)
        self.toothTopPoint = toothTop

        # Arc through the two shared flank end SketchPoints (passed directly so
        # the arc shares them) and the tooth-top point's geometry.
        arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            rightEnd, toothTop.geometry, leftEnd)
        self.toothTopArc = arc

        # A single driving diameter dimension equal to the tip circle diameter.
        sketch.sketchDimensions.addDiameterDimension(
            arc,
            adsk.core.Point3D.create(0, tipRadius, 0))

    def _drawSpine(self, angle, tipRadius):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        lines = sketch.sketchCurves.sketchLines

        # Spine shares both existing points directly: local origin (start) and
        # the step-6 tooth-top point (far end). No separate start-coincident,
        # no end-on-arc constraint.
        spine = lines.addByTwoPoints(self.anchorPoint, self.toothTopPoint)
        spine.isConstruction = True
        self.spine = spine
        self.spineAngularDimension = None

        if angle == 0:
            # Spur: the spine's only additional constraint is horizontal.
            constraints.addHorizontal(spine)
        else:
            # Horizontal reference that ALWAYS points +X (fixed +tipRadius), so
            # the spine->horizontal angle equals `angle` for any angle (the
            # tooth-top point itself may sit on the -X side once angle > 90 deg,
            # so deriving the reference from it would flip the sign).
            refEnd = adsk.core.Point3D.create(tipRadius, 0, 0)
            horizontal = lines.addByTwoPoints(self.anchorPoint, refEnd)
            horizontal.isConstruction = True
            constraints.addHorizontal(horizontal)
            # Angular dimension spine -> horizontal, in that argument order, so
            # the sign of the tilt (helix hand) is fixed. Place the text on the
            # bisector of the intended angle so Fusion selects `angle` (not its
            # supplement). The spine is already drawn at `angle`, so setting the
            # value later confirms the position rather than rotating into it.
            dim = sketch.sketchDimensions.addAngularDimension(
                spine, horizontal,
                adsk.core.Point3D.create(
                    tipRadius / 2 * math.cos(angle / 2),
                    tipRadius / 2 * math.sin(angle / 2), 0))
            self.spineAngularDimension = dim

    def _drawRibs(self, leftSpline, rightSpline, angle=0):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines
        ca = math.cos(angle)
        sa = math.sin(angle)

        leftFits = leftSpline.fitPoints
        rightFits = rightSpline.fitPoints
        count = min(leftFits.count, rightFits.count)

        # The chain starts with previous = local origin so the first rib's
        # midpoint is dimensioned from the origin (locks the residual DOF).
        previousMidpoint = self.anchorPoint
        for i in range(count):
            leftFit = leftFits.item(i)
            rightFit = rightFits.item(i)

            # 1. rib line shares the two fit points directly; construction.
            rib = lines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True

            # 2. dimension the rib's aligned length to its current value.
            dims.addDistanceDimension(
                rib.startSketchPoint, rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (leftFit.geometry.x + rightFit.geometry.x) / 2,
                    (leftFit.geometry.y + rightFit.geometry.y) / 2, 0))

            # 3. fresh midpoint, created ALREADY on the spine. The spine is the
            # line at `angle` through the origin (y = 0 only when angle == 0),
            # so seed the midpoint at the foot of the left fit point on that
            # line - it must start ON the spine (an off-spine seed
            # over-constrains). For angle == 0 this reduces to (fitX, 0).
            t = leftFit.geometry.x * ca + leftFit.geometry.y * sa
            midpoint = sketch.sketchPoints.add(
                adsk.core.Point3D.create(t * ca, t * sa, 0))
            # 4. pin onto the spine first.
            constraints.addCoincident(midpoint, self.spine)
            # 5. then make it the rib's midpoint.
            constraints.addMidPoint(midpoint, rib)
            # 6. then make the rib perpendicular to the spine.
            constraints.addPerpendicular(self.spine, rib)

            # Chain distance from the previous midpoint (origin for the first).
            dims.addDistanceDimension(
                previousMidpoint, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (previousMidpoint.geometry.x + midpoint.geometry.x) / 2,
                    -0.1, 0))
            previousMidpoint = midpoint

    def _drawFlankToRoot(self, leftSpline, rightSpline, rootRadius):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        lines = sketch.sketchCurves.sketchLines

        leftStart = leftSpline.startSketchPoint
        # The flank's first point lies on the base circle. If it sits inside the
        # root circle, no flank-to-root lines are drawn ("embedded" profile).
        startRadius = math.hypot(leftStart.geometry.x, leftStart.geometry.y)
        if startRadius <= rootRadius:
            self.parent._lastToothEmbedded = True
            return
        self.parent._lastToothEmbedded = False

        rightStart = rightSpline.startSketchPoint

        for flankStart in (leftStart, rightStart):
            # Root-end seed along the origin -> flank-start ray, on the root
            # circle. The line shares the flank's start point directly.
            ang = math.atan2(flankStart.geometry.y, flankStart.geometry.x)
            rootEnd = adsk.core.Point3D.create(
                rootRadius * math.cos(ang), rootRadius * math.sin(ang), 0)
            line = lines.addByTwoPoints(rootEnd, flankStart)
            # (a) the line's root-end point coincident to the root circle.
            constraints.addCoincident(line.startSketchPoint, self.rootCircle)
            # (b) the local origin coincident to the line itself (point-on-line,
            # line treated as infinite) - pins the line to a radial direction.
            constraints.addCoincident(self.anchorPoint, line)

    def drawBore(self, anchorPoint):
        # Construction-less bore circle of the registered diameter at the
        # projected anchor point.
        sketch = self.sketch
        boreDiameter = self.getParameterValue(PARAM_BORE_DIAMETER)
        radius = boreDiameter / 2
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            anchorPoint, radius)
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0))
        return circle

    def draw(self, anchorPoint, angle=0):
        # Order: circles, tooth, then the step-5 anchoring as the final action
        # of draw() itself (helical/herringbone rely on this single call).
        self.drawCircles()
        self.drawTooth(angle)

        # --- step 5: project the Tools anchor into this sketch (chaining the
        # sketches) and constrain it coincident with the local origin, sliding
        # the whole tooth onto the user's anchor.
        projected = self.sketch.project(anchorPoint)
        projectedAnchor = projected.item(0)
        self.sketch.geometricConstraints.addCoincident(
            projectedAnchor, self.anchorPoint)


class SpurGearGenerator(Generator):
    """Orchestrates building one spur gear: parameters, sketches, body, bore."""

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self._lastToothEmbedded = False
        self.toolsSketch = adsk.fusion.Sketch.cast(None)
        self.boreSketch = adsk.fusion.Sketch.cast(None)

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    # ----- input processing -----
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Pull every selection out of `inputs` BEFORE anything creates the
        #    occurrence (the context shift can drop selections otherwise).
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PARENT}' missing")
        parentEntity = parents[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        elif parentEntity.objectType == adsk.fusion.Component.classType():
            self.parentComponent = parentEntity
        else:
            raise Exception(
                f'Invalid parent component type: {parentEntity.objectType}')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PLANE}' missing")
        self.plane = planes[0]

        (anchors, _) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception(
                f"Required selection '{INPUT_ID_ANCHOR_POINT}' missing")
        self.anchorPoint = anchors[0]

        # 2-3. Register input-sourced parameters from the still-live inputs. The
        #      first addParameter() transitively creates the occurrence (so the
        #      selections above are already safely stashed on self).
        (module, _) = get_value(inputs, INPUT_ID_MODULE, '')
        # Module comes straight from get_value (a ValueInput for a numeric).
        self.addParameter(PARAM_MODULE, module, '', 'Module')

        (toothNumber, _) = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Tooth Number')

        (pressureAngle, _) = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad',
                          'Pressure Angle')

        # Bore Diameter is an addStringValueInput field: get_value may hand back
        # a raw evaluated float with ok=False. Wrap THAT value (not 0) so a
        # typed bore diameter actually takes effect.
        (boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        if not ok:
            boreDiameter = adsk.core.ValueInput.createByReal(
                boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm',
                          'Bore Diameter')

        (thickness, _) = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm', 'Thickness')

        (chamferTooth, _) = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm',
                          'Chamfer applied to the tooth edges')

        # SketchOnly persisted as a real-valued parameter (1 = true, 0 = false)
        # because the framework reads numeric parameters as booleans.
        (sketchOnly, _) = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only')

        # 4. hook for subclasses to inject their own primary parameters before
        #    the derived parameters reference them.
        self.addExtraPrimaryParameters(inputs)

        # 5-6. Register derived parameters (live expressions, plus the two
        #      Python-computed ones).
        self.registerDerivedParameters()

    def addExtraPrimaryParameters(self, inputs):
        # Base spur gear adds nothing; subclasses inject helix params here.
        pass

    def registerDerivedParameters(self):
        def expr(name):
            return self.parameterName(name)

        byString = adsk.core.ValueInput.createByString
        byReal = adsk.core.ValueInput.createByReal

        self.addParameter(
            PARAM_PITCH_CIRCLE_DIAMETER,
            byString(f'{expr(PARAM_MODULE)} * {expr(PARAM_TOOTH_NUMBER)}'),
            'mm', 'Pitch Circle Diameter')
        self.addParameter(
            PARAM_PITCH_CIRCLE_RADIUS,
            byString(f'{expr(PARAM_PITCH_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Pitch Circle Radius')

        self.addParameter(
            PARAM_BASE_CIRCLE_DIAMETER,
            byString(f'{expr(PARAM_PITCH_CIRCLE_DIAMETER)} * '
                     f'cos({expr(PARAM_PRESSURE_ANGLE)})'),
            'mm', 'Base Circle Diameter')
        self.addParameter(
            PARAM_BASE_CIRCLE_RADIUS,
            byString(f'{expr(PARAM_BASE_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Base Circle Radius')

        self.addParameter(
            PARAM_ROOT_CIRCLE_DIAMETER,
            byString(f'{expr(PARAM_PITCH_CIRCLE_DIAMETER)} - '
                     f'2.5 * {expr(PARAM_MODULE)}'),
            'mm', 'Root Circle Diameter')
        self.addParameter(
            PARAM_ROOT_CIRCLE_RADIUS,
            byString(f'{expr(PARAM_ROOT_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Root Circle Radius')

        self.addParameter(
            PARAM_TIP_CIRCLE_DIAMETER,
            byString(f'{expr(PARAM_PITCH_CIRCLE_DIAMETER)} + '
                     f'2 * {expr(PARAM_MODULE)}'),
            'mm', 'Tip Circle Diameter')
        self.addParameter(
            PARAM_TIP_CIRCLE_RADIUS,
            byString(f'{expr(PARAM_TIP_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Tip Circle Radius')

        self.addParameter(
            PARAM_INVOLUTE_STEPS, byReal(15), '', 'Involute Steps')

        # ToothSpaceAngleAtRoot: Fusion's expression engine refuses to mix the
        # unitless tan() output with the radian Pressure Angle in a
        # subtraction, so compute it in Python. Register UNITLESS ('') even
        # though it holds a radian value - the next parameter multiplies it by a
        # length, which Fusion accepts as a length only if this factor is
        # unitless (a radian magnitude is dimensionless anyway).
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        toothSpaceAngleAtRoot = (
            math.pi / toothNumberValue
            - 2 * (math.tan(pressureAngleValue) - pressureAngleValue))
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            byReal(toothSpaceAngleAtRoot), '',
            'Tooth Space Angle At Root')

        # ToothSpaceArcAtRoot: live expression in mm. Reads as a pure length
        # only because the angle factor above is unitless.
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC_AT_ROOT,
            byString(f'{expr(PARAM_ROOT_CIRCLE_RADIUS)} * '
                     f'{expr(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)}'),
            'mm', 'Tooth Space Arc At Root')

        self.addParameter(
            PARAM_FILLET_CLEARANCE, byReal(0.9), '', 'Fillet Clearance')

        # Fillet Radius: the trailing factor is the helix hook (cos(Helix
        # Angle) in subclasses); always 1 for a spur gear.
        self.addParameter(
            PARAM_FILLET_RADIUS,
            byString(f'({expr(PARAM_TOOTH_SPACE_ARC_AT_ROOT)} / 2) * '
                     f'{expr(PARAM_FILLET_CLEARANCE)} * '
                     f'{self.filletHelixFactorExpression()}'),
            'mm', 'Fillet Radius')

    def filletHelixFactorExpression(self) -> str:
        # Base spur gear: factor is 1. Subclasses return cos(Helix Angle).
        return '1'

    # ----- name -----
    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        # Use the parameters' .expression strings so units show through.
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    # ----- orchestration -----
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        ctx = self.newContext()

        # Step 1: normalize the target plane to a ConstructionPlane. If the user
        # picked a planar face, build a coplanar construction plane so the
        # downstream profile detection isn't confused by the selected face.
        if self.plane.objectType == adsk.fusion.ConstructionPlane.classType():
            ctx.normalizedPlane = None
        else:
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(
                self.plane, adsk.core.ValueInput.createByReal(0))
            normalized = component.constructionPlanes.add(planeInput)
            self.plane = normalized
            ctx.normalizedPlane = normalized
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.cleanup(ctx)

    # ----- step 1-2: tools sketch + extrusion end plane -----
    def prepareTools(self, ctx: SpurGearGenerationContext):
        component = self.getComponent()

        # Step 2: Tools sketch on the target plane. Project the anchor point and
        # keep the result as ctx.anchorPoint (the canonical handle every later
        # sketch re-projects from). Leave it visible while later sketches
        # project from it.
        toolsSketch = self.createSketchObject('Tools', ctx.plane)
        toolsSketch.isVisible = True
        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)
        self.toolsSketch = toolsSketch

        # Offset construction plane 'Extrusion End Plane' at Thickness from the
        # target plane. The to-entity target for the tooth and body extrudes.
        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(
            ctx.plane, adsk.core.ValueInput.createByReal(thickness))
        endPlane = component.constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    # ----- step 3-6: sketches + sketch-only short-circuit -----
    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 6: show the Gear Profile sketch and stop (no body ops). The
            # end-of-build cleanup still runs and hides the planes/axes.
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        # Step 3-5: Gear Profile sketch + the tooth generator. buildSketches
        # owns creating this sketch and invoking the tooth generator (helical
        # overrides it, calls super(), then draws a second twisted profile).
        sketch = self.createSketchObject('Gear Profile', ctx.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch

        generator = SpurGearInvoluteToothDesignGenerator(sketch, self)
        generator.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # ----- step 7-8: extrude tooth + chamfer -----
    def buildTooth(self, ctx: SpurGearGenerationContext):
        component = self.getComponent()

        profile = self._findToothProfile(ctx)
        if profile is None:
            raise Exception('Could not find the tooth cross-section profile')

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

        # buildTooth owns the tooth body AND must call chamferTooth last.
        self.chamferTooth(ctx)

    def _findToothProfile(self, ctx: SpurGearGenerationContext):
        # The tooth loop: 2 NURBS flanks, 2 arcs (tooth top + root arc), and -
        # unless embedded - 2 short flank-to-root line segments.
        expectedSplines = 2
        expectedArcs = 2
        expectedLines = 0 if ctx.toothProfileIsEmbedded else 2
        expectedTotal = expectedSplines + expectedArcs + expectedLines

        for profile in ctx.gearProfileSketch.profiles:
            for loop in profile.profileLoops:
                curves = loop.profileCurves
                if curves.count != expectedTotal:
                    continue
                splines = arcs = lines = 0
                for j in range(curves.count):
                    ctype = curves.item(j).geometry.curveType
                    if ctype == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        splines += 1
                    elif ctype == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ctype == adsk.core.Curve3DTypes.Line3DCurveType:
                        lines += 1
                if (splines == expectedSplines and arcs == expectedArcs
                        and lines == expectedLines):
                    return profile
        return None

    def chamferTooth(self, ctx: SpurGearGenerationContext):
        chamferValue = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferValue <= 0:
            return

        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value

        frontFace = self._findToothFrontFace(ctx)
        if frontFace is None:
            raise Exception('Could not find the tooth front face to chamfer')

        # Build the chamfer edge set: every front-face edge EXCEPT the root arc.
        # Identify the root arc by radius (Arc3DCurveType whose geometry.radius
        # equals Root Circle Radius, tolerance 0.001 cm) - the exact, robust
        # match (more robust than "smallest-radius arc") that step 11 reuses.
        edges = adsk.core.ObjectCollection.create()
        for i in range(frontFace.edges.count):
            edge = frontFace.edges.item(i)
            geometry = edge.geometry
            if geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(geometry.radius - rootRadius) < 0.001:
                    continue
            edges.add(edge)

        chamfers = component.features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferValue), False)
        chamfers.add(chamferInput)

    def chamferWantEdges(self) -> int:
        # Expected front-face edge count for the chamfer: 6 for a spur tooth
        # (2 flanks, tooth-top arc, root arc, 2 flank-to-root lines). Subclasses
        # override (a lofted embedded tooth has 4).
        return 6

    def _findToothFrontFace(self, ctx: SpurGearGenerationContext):
        wantEdges = self.chamferWantEdges()
        for i in range(ctx.toothBody.faces.count):
            face = ctx.toothBody.faces.item(i)
            if face.edges.count == wantEdges:
                return face
        return None

    # ----- step 9: extrude body -----
    def buildBody(self, ctx: SpurGearGenerationContext):
        component = self.getComponent()

        profile = self._findBodyProfile(ctx)
        if profile is None:
            raise Exception('Could not find the annular body profile')

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'

        body = extrude.bodies.item(0)
        body.name = 'Gear Body'
        ctx.gearBody = body

        # Capture the Gear Center axis and the far end-cap face by classifying
        # the new body's faces by surfaceType.
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        centerAxis = None
        extrusionExtent = None
        for i in range(body.faces.count):
            face = body.faces.item(i)
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if centerAxis is None:
                    axisInput = component.constructionAxes.createInput()
                    axisInput.setByCircularFace(face)
                    centerAxis = component.constructionAxes.add(axisInput)
                    centerAxis.name = 'Gear Center'
                    centerAxis.isLightBulbOn = False
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                # The far cap is parallel to but not coplanar with the sketch
                # plane (the near cap IS coplanar, so isCoPlanarTo rules it out).
                if (sketchPlane.isParallelToPlane(face.geometry)
                        and not sketchPlane.isCoPlanarTo(face.geometry)):
                    extrusionExtent = face

        if centerAxis is None:
            raise Exception('Could not build the Gear Center axis')
        if extrusionExtent is None:
            raise Exception('Could not find the far end-cap face')

        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent

    def _findBodyProfile(self, ctx: SpurGearGenerationContext):
        # The annular body loop: exactly 2 arcs (root + tip circles).
        for profile in ctx.gearProfileSketch.profiles:
            for loop in profile.profileLoops:
                curves = loop.profileCurves
                if curves.count != 2:
                    continue
                allArcs = True
                for j in range(curves.count):
                    if (curves.item(j).geometry.curveType
                            != adsk.core.Curve3DTypes.Arc3DCurveType):
                        allArcs = False
                        break
                if allArcs:
                    return profile
        return None

    # ----- step 10-11: circular pattern + combine + fillets -----
    def patternTeeth(self, ctx: SpurGearGenerationContext):
        component = self.getComponent()

        toothNumber = int(round(self.getParameter(PARAM_TOOTH_NUMBER).value))

        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(ctx.toothBody)

        patterns = component.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(
            float(toothNumber))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # pattern.bodies already includes the seed body plus the N-1 copies.
        # Feed the whole collection to combine - do NOT re-add ctx.toothBody.
        combineTools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            combineTools.add(pattern.bodies.item(i))

        combines = component.features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # patternTeeth owns the pattern+combine and calls createFillets last.
        self.createFillets(ctx)

    def createFillets(self, ctx: SpurGearGenerationContext):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value

        # Gear main axis = target plane normal.
        axisDir = get_normal(ctx.plane)
        axisDir.normalize()

        edges = adsk.core.ObjectCollection.create()
        body = ctx.gearBody
        for i in range(body.faces.count):
            face = body.faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            # Collect EVERY cylindrical face whose radius equals Root radius
            # (the pattern/combine usually splits the root into one patch per
            # valley), not just the first found.
            if abs(face.geometry.radius - rootRadius) >= 0.001:
                continue
            for j in range(face.edges.count):
                edge = face.edges.item(j)
                geometry = edge.geometry
                # Keep only axial straight edges (tangent parallel to the main
                # axis); drop the circular end-cap rims.
                if geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                # A line edge has a constant tangent = its direction. Read it
                # from the line geometry's endpoints rather than
                # evaluator.getTangent(0): parameter 0 is not guaranteed to lie
                # inside the edge's parameter range and raises
                # "invalid argument parameter".
                direction = geometry.startPoint.vectorTo(geometry.endPoint)
                if direction.length == 0:
                    continue
                direction.normalize()
                if abs(abs(direction.dotProduct(axisDir)) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            return

        fillets = component.features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        fillets.add(filletInput)

    # ----- step 12: bore -----
    def buildBore(self, ctx: SpurGearGenerationContext):
        # Returns early in sketch-only mode or when no bore is requested.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        component = self.getComponent()

        # Separate Bore Profile sketch on the target plane.
        sketch = self.createSketchObject('Bore Profile', ctx.plane)
        sketch.isVisible = True
        self.boreSketch = sketch

        # Re-project ctx.anchorPoint from the (still visible) Tools sketch, so
        # the bore chains back to the user's anchor. (This is why cleanup runs
        # after buildBore - projection fails once the Tools sketch is hidden.)
        projected = sketch.project(ctx.anchorPoint)
        projectedAnchor = projected.item(0)

        radius = boreDiameter / 2
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, radius)
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0))

        boreProfile = None
        for profile in sketch.profiles:
            boreProfile = profile
            break
        if boreProfile is None:
            raise Exception('Could not find the bore profile')

        # Extrude-cut from the target plane to the far end-cap face, affecting
        # only the gear body. The to-face extent guarantees a through bore
        # regardless of Thickness.
        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

        # buildBore hides its own Bore Profile sketch.
        sketch.isVisible = False

    # ----- end-of-build cleanup -----
    def cleanup(self, ctx: SpurGearGenerationContext):
        # ALWAYS hide the construction planes/axes via isLightBulbOn = False -
        # this runs in BOTH modes, so sketch-only mode leaves no stray plane
        # floating. Guard each entity individually (Gear Center axis and the
        # normalized Target Plane don't always exist).
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if ctx.normalizedPlane is not None:
            ctx.normalizedPlane.isLightBulbOn = False

        # Hide the sketches via isVisible = False ONLY when NOT SketchOnly. In
        # SketchOnly mode the gear sketches stay visible for inspection.
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
