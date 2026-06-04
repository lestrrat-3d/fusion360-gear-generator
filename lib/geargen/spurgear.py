import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids
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

# ---------------------------------------------------------------------------
# User-parameter names
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to sketch the gear on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point (selection)
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point to align the gear center with')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 4. Tooth Number
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))

        # 5. Pressure Angle
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))

        # 6. Bore Diameter (string value input so it accepts expressions)
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

        # 7. Thickness
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))

        # 8. Apply chamfer to teeth
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 9. Generate sketches, but do not build body
        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True,
            '', False)

        # 10. Parent Component (selection) -- last
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component for the new gear')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)


# ---------------------------------------------------------------------------
# Generation context
# ---------------------------------------------------------------------------
class SpurGearGenerationContext(GenerationContext):
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


# ---------------------------------------------------------------------------
# Tooth profile generator
# ---------------------------------------------------------------------------
class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        # The movable local origin -- a fresh SketchPoint at (0,0,0).
        # NOT sketch.originPoint, which is immutable.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

        # Handles to circles, filled in by drawCircles().
        self.rootCircle = None
        self.tipCircle = None
        self.baseCircle = None
        self.pitchCircle = None

    # -- parameter accessors (part of the reproduced surface) ----------------
    def getParameter(self, name: str) -> adsk.fusion.UserParameter:
        return self.parent.getParameter(name)

    def getParameterValue(self, name: str):
        return self.parent.getParameter(name).value

    # -- involute math -------------------------------------------------------
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    # -- top-level draw ------------------------------------------------------
    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)

        # Step 5: anchor the sketch onto the user's anchor.
        projected = self.sketch.project(anchorPoint)
        projectedPoint = projected.item(0)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedPoint)

    def drawCircles(self):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        circles = sketch.sketchCurves.sketchCircles

        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)

        rootDiameter = self.getParameterValue(PARAM_ROOT_CIRCLE_DIAMETER)
        tipDiameter = self.getParameterValue(PARAM_TIP_CIRCLE_DIAMETER)
        baseDiameter = self.getParameterValue(PARAM_BASE_CIRCLE_DIAMETER)
        pitchDiameter = self.getParameterValue(PARAM_PITCH_CIRCLE_DIAMETER)

        size = tipRadius - rootRadius

        def makeCircle(name, radius, diameter, isConstruction):
            # Share the local-origin point directly as the center.
            circle = circles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = isConstruction
            dimensions.addDiameterDimension(
                circle,
                adsk.core.Point3D.create(radius, radius, 0))
            # along-path label
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True,
                adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            return circle

        # 1. Root Circle (solid)
        self.rootCircle = makeCircle('Root Circle', rootRadius, rootDiameter, False)
        # 2. Tip Circle (construction)
        self.tipCircle = makeCircle('Tip Circle', tipRadius, tipDiameter, True)
        # 3. Base Circle (construction)
        self.baseCircle = makeCircle('Base Circle', baseRadius, baseDiameter, True)
        # 4. Pitch Circle (construction)
        self.pitchCircle = makeCircle('Pitch Circle', pitchRadius, pitchDiameter, True)

    def drawTooth(self, angle):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        splines = sketch.sketchCurves.sketchFittedSplines
        lines = sketch.sketchCurves.sketchLines

        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)
        tipDiameter = self.getParameterValue(PARAM_TIP_CIRCLE_DIAMETER)
        toothNumber = int(self.getParameterValue(PARAM_TOOTH_NUMBER))
        involuteSteps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        # ---- 1. Sample the involute flank from base radius outward to tip ----
        rawPoints = []
        for i in range(0, involuteSteps):
            if involuteSteps > 1:
                r = baseRadius + (tipRadius - baseRadius) * (i / (involuteSteps - 1))
            else:
                r = baseRadius
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                continue
            rawPoints.append(p)

        # ---- 2. Mirror across +X (negate y) so the spiral matches a left flank ----
        mirrored = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in rawPoints]

        # ---- 3. analytic rotate_angle from pitch-circle crossing ----
        pitchCross = self.calculateInvolutePoint(baseRadius, pitchRadius)
        # mirrored pitch crossing angle (negate y as above)
        pitchAngle = math.atan2(-pitchCross.y, pitchCross.x)
        targetHalfTooth = math.pi / (2 * toothNumber)
        rotate_angle = targetHalfTooth - pitchAngle

        def rotate(points, theta):
            c = math.cos(theta)
            s = math.sin(theta)
            return [adsk.core.Point3D.create(
                p.x * c - p.y * s, p.x * s + p.y * c, 0) for p in points]

        # ---- 4. rotate mirrored -> left flank, mirror -> right flank, then apply `angle` ----
        leftPoints = rotate(mirrored, rotate_angle)
        rightPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in leftPoints]

        if angle != 0:
            leftPoints = rotate(leftPoints, angle)
            rightPoints = rotate(rightPoints, angle)

        # ---- 5. draw the flanks as fitted splines ----
        def toCollection(points):
            coll = adsk.core.ObjectCollection.create()
            for p in points:
                coll.add(p)
            return coll

        leftSpline = splines.add(toCollection(leftPoints))
        rightSpline = splines.add(toCollection(rightPoints))

        # ---- 6. tooth-top arc ----
        toothTopGeom = adsk.core.Point3D.create(
            tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0)
        toothTopPoint = sketch.sketchPoints.add(toothTopGeom)
        constraints.addCoincident(toothTopPoint, self.tipCircle)

        leftEnd = leftSpline.endSketchPoint
        rightEnd = rightSpline.endSketchPoint
        arcs = sketch.sketchCurves.sketchArcs
        topArc = arcs.addByThreePoints(rightEnd, toothTopPoint.geometry, leftEnd)
        dimensions.addDiameterDimension(
            topArc,
            adsk.core.Point3D.create(toothTopGeom.x, toothTopGeom.y + tipRadius * 0.1, 0))

        # ---- 7. spine ----
        spine = lines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True

        spineAngularDimension = None
        if angle == 0:
            constraints.addHorizontal(spine)
        else:
            # Horizontal reference construction line that always points +X.
            horizontal = lines.addByTwoPoints(
                self.anchorPoint,
                adsk.core.Point3D.create(tipRadius, 0, 0))
            horizontal.isConstruction = True
            constraints.addHorizontal(horizontal)
            # Pin the far endpoint so the reference line is fully determined.
            # Its endpoint at (TipCircleRadius, 0) already lies on the tip circle;
            # with Horizontal + shared origin, coincidence to the tip circle fixes
            # it exactly at the +X intersection, removing the otherwise-free length DOF.
            constraints.addCoincident(horizontal.endSketchPoint, self.tipCircle)

            # angular dimension from spine to horizontal reference, text on bisector.
            R = tipRadius * 0.5
            textPoint = adsk.core.Point3D.create(
                R * math.cos(angle / 2), R * math.sin(angle / 2), 0)
            spineAngularDimension = dimensions.addAngularDimension(spine, horizontal, textPoint)

        # ---- 8. ribs ----
        leftFit = leftSpline.fitPoints
        rightFit = rightSpline.fitPoints
        ribCount = leftFit.count

        previousMidpoint = self.anchorPoint  # chain starts at the local origin
        c = math.cos(angle)
        s = math.sin(angle)
        for i in range(0, ribCount):
            leftPt = leftFit.item(i)
            rightPt = rightFit.item(i)

            # 1. rib line sharing the two fit points
            rib = lines.addByTwoPoints(leftPt, rightPt)
            rib.isConstruction = True

            # 2. dimension the rib's aligned length to current measured value
            ribLine = rib
            ribStartGeom = ribLine.startSketchPoint.geometry
            ribEndGeom = ribLine.endSketchPoint.geometry
            ribLengthText = adsk.core.Point3D.create(
                (ribStartGeom.x + ribEndGeom.x) / 2,
                (ribStartGeom.y + ribEndGeom.y) / 2,
                0)
            dimensions.addDistanceDimension(
                ribLine.startSketchPoint, ribLine.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                ribLengthText)

            # 3. midpoint seeded on the spine (foot of left fit point on that line)
            fitX = leftPt.geometry.x
            fitY = leftPt.geometry.y
            tparam = fitX * c + fitY * s
            midSeed = adsk.core.Point3D.create(tparam * c, tparam * s, 0)
            midpoint = sketch.sketchPoints.add(midSeed)

            # 4. coincident midpoint onto spine FIRST
            constraints.addCoincident(midpoint, spine)
            # 5. then make it the rib's midpoint
            constraints.addMidPoint(midpoint, rib)
            # 6. then make the rib perpendicular to the spine
            constraints.addPerpendicular(spine, rib)

            # chain distance: midpoint to previous midpoint (origin for the first)
            prevGeom = previousMidpoint.geometry
            chainText = adsk.core.Point3D.create(
                (midSeed.x + prevGeom.x) / 2,
                (midSeed.y + prevGeom.y) / 2,
                0)
            dimensions.addDistanceDimension(
                previousMidpoint, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                chainText)
            previousMidpoint = midpoint

        # ---- 9. flank-to-root lines (common case) ----
        flankStartLeft = leftSpline.startSketchPoint
        flankStartRight = rightSpline.startSketchPoint
        flankStartRadius = math.hypot(
            flankStartLeft.geometry.x, flankStartLeft.geometry.y)

        if flankStartRadius > rootRadius:
            # flank starts outside the root circle -> draw flank-to-root lines.
            embedded = False
            for flankStart in (flankStartLeft, flankStartRight):
                fg = flankStart.geometry
                # root end seeded along the origin->flank-start ray at root radius
                scale = rootRadius / math.hypot(fg.x, fg.y)
                rootEndGeom = adsk.core.Point3D.create(fg.x * scale, fg.y * scale, 0)
                line = lines.addByTwoPoints(rootEndGeom, flankStart)
                # (a) root end on the root circle
                constraints.addCoincident(line.startSketchPoint, self.rootCircle)
                # (b) local origin on the line (point-on-line) -> radial direction
                constraints.addCoincident(self.anchorPoint, line)
        else:
            embedded = True

        # Record embedded-ness on the parent generator (the tooth gen has no ctx).
        self.parent._lastToothEmbedded = embedded

        # ---- helical rotation confirm (last action) ----
        if angle != 0:
            spineAngularDimension.parameter.value = angle

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dimensions = sketch.sketchDimensions

        projected = sketch.project(anchorPoint)
        center = projected.item(0)

        radius = diameter / 2
        circle = circles.addByCenterRadius(center, radius)
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0))
        return circle


# ---------------------------------------------------------------------------
# Generator (orchestrator)
# ---------------------------------------------------------------------------
class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self._lastToothEmbedded = False
        self.toolsSketch = None
        self.boreSketch = None
        self.normalizedPlane = None
        self.plane = None
        self.anchorPoint = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self):
        return SpurGearGenerationContext()

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    # ----- hooks subclasses override -----
    def chamferWantEdges(self):
        return 6

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        pass

    # ----- input processing -----
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Read selections FIRST (before any occurrence creation).
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one parent component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parent.component
        elif parent.objectType == adsk.fusion.Component.classType():
            self.parentComponent = parent
        else:
            raise Exception('Selected parent is not a component or occurrence')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        self.plane = planes[0]

        (anchors, _) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception('Exactly one anchor point must be selected')
        self.anchorPoint = anchors[0]

        # 3. Numeric/boolean inputs -> register as user parameters.
        (module, _) = get_value(inputs, INPUT_ID_MODULE, '')
        self.addParameter(PARAM_MODULE, module, '', 'Module of the gear')

        (toothNumber, _) = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Number of teeth')

        (pressureAngle, _) = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad', 'Pressure angle')

        (boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        if not ok:
            boreDiameter = adsk.core.ValueInput.createByReal(
                boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm', 'Bore diameter')

        (thickness, _) = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm', 'Axial thickness of the gear')

        (chamferTooth, _) = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm', 'Chamfer distance on the teeth')

        (sketchOnly, _) = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only (1=true, 0=false)')

        # 4. subclass primary parameters before derived references.
        self.addExtraPrimaryParameters(inputs)

        # 5. derived parameters as live Fusion expressions.
        def p(name):
            return self.parameterName(name)

        self.addParameter(
            PARAM_PITCH_CIRCLE_DIAMETER,
            adsk.core.ValueInput.createByString(f'{p(PARAM_MODULE)} * {p(PARAM_TOOTH_NUMBER)}'),
            'mm', 'Pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(f'{p(PARAM_PITCH_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Pitch circle radius')

        self.addParameter(
            PARAM_BASE_CIRCLE_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{p(PARAM_PITCH_CIRCLE_DIAMETER)} * cos({p(PARAM_PRESSURE_ANGLE)})'),
            'mm', 'Base circle diameter')
        self.addParameter(
            PARAM_BASE_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(f'{p(PARAM_BASE_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Base circle radius')

        self.addParameter(
            PARAM_ROOT_CIRCLE_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{p(PARAM_PITCH_CIRCLE_DIAMETER)} - 2.5 * {p(PARAM_MODULE)}'),
            'mm', 'Root circle diameter')
        self.addParameter(
            PARAM_ROOT_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(f'{p(PARAM_ROOT_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Root circle radius')

        self.addParameter(
            PARAM_TIP_CIRCLE_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{p(PARAM_PITCH_CIRCLE_DIAMETER)} + 2 * {p(PARAM_MODULE)}'),
            'mm', 'Tip circle diameter')
        self.addParameter(
            PARAM_TIP_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(f'{p(PARAM_TIP_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Tip circle radius')

        self.addParameter(
            PARAM_INVOLUTE_STEPS,
            adsk.core.ValueInput.createByReal(15),
            '', 'Number of involute sample steps')

        # 6. Tooth Space Angle At Root -- pre-computed in Python, registered UNITLESS.
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngleAtRoot = (
            math.pi / toothNumberValue
            - 2 * (math.tan(pressureAngleValue) - pressureAngleValue))
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            adsk.core.ValueInput.createByReal(toothSpaceAngleAtRoot),
            '', 'Tooth space angle at root (radians, registered unitless)')

        # Tooth Space Arc At Root -- live expression in mm.
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC_AT_ROOT,
            adsk.core.ValueInput.createByString(
                f'{p(PARAM_ROOT_CIRCLE_RADIUS)} * {p(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)}'),
            'mm', 'Tooth space arc length at root')

        self.addParameter(
            PARAM_FILLET_CLEARANCE,
            adsk.core.ValueInput.createByReal(0.9),
            '', 'Fillet clearance fraction')

        self.addParameter(
            PARAM_FILLET_RADIUS,
            adsk.core.ValueInput.createByString(
                f'({p(PARAM_TOOTH_SPACE_ARC_AT_ROOT)} / 2) * {p(PARAM_FILLET_CLEARANCE)} '
                f'* {self.filletHelixFactorExpression()}'),
            'mm', 'Root fillet radius')

    # ----- orchestration -----
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalize self.plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = self.getComponent().constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = self.getComponent().constructionPlanes.add(planeInput)
            self.normalizedPlane = self.plane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.cleanup(ctx)

    # ----- steps 1-2: tools sketch + extrusion end plane -----
    def prepareTools(self, ctx: SpurGearGenerationContext):
        toolsSketch = self.createSketchObject('Tools', plane=self.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch

        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)

        # Extrusion End Plane offset by Thickness.
        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = self.getComponent().constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        endPlane = self.getComponent().constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    # ----- main body build -----
    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    # step 3-5
    def buildSketches(self, ctx: SpurGearGenerationContext):
        sketch = self.createSketchObject('Gear Profile', plane=self.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch

        SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # step 7-8
    def buildTooth(self, ctx: SpurGearGenerationContext):
        sketch = ctx.gearProfileSketch
        embedded = ctx.toothProfileIsEmbedded
        expectedCount = 4 if embedded else 6

        profile = None
        for prof in sketch.profiles:
            for loop in prof.profileLoops:
                if loop.profileCurves.count == expectedCount:
                    profile = prof
                    break
            if profile is not None:
                break
        if profile is None:
            raise Exception(
                f'Could not find tooth profile (expected {expectedCount} curves)')

        extrudes = self.getComponent().features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

        self.chamferTooth(ctx)

    # step 8
    def chamferTooth(self, ctx: SpurGearGenerationContext):
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        wantEdges = self.chamferWantEdges()

        # Find the front face by edge count.
        frontFace = None
        for face in ctx.toothBody.faces:
            if face.edges.count == wantEdges:
                frontFace = face
                break
        if frontFace is None:
            raise Exception(
                f'Could not find tooth front face with {wantEdges} edges for chamfer')

        edges = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            curve = edge.geometry
            if curve.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(curve.radius - rootRadius) < 0.001:
                    continue  # skip the root arc
            edges.add(edge)

        chamfers = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferDistance), False)
        chamfers.add(chamferInput)

    # step 9
    def buildBody(self, ctx: SpurGearGenerationContext):
        sketch = ctx.gearProfileSketch

        profile = None
        for prof in sketch.profiles:
            for loop in prof.profileLoops:
                if loop.profileCurves.count != 2:
                    continue
                allArcs = True
                for curve in loop.profileCurves:
                    if curve.geometry.curveType != adsk.core.Curve3DTypes.Arc3DCurveType:
                        allArcs = False
                        break
                if allArcs:
                    profile = prof
                    break
            if profile is not None:
                break
        if profile is None:
            raise Exception('Could not find annular gear body profile (2 arcs)')

        extrudes = self.getComponent().features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extInput)
        extrude.name = 'Extrude body'
        body = extrude.bodies.item(0)
        body.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry

        centerAxis = None
        extrusionExtent = None
        for face in body.faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and centerAxis is None:
                axisInput = self.getComponent().constructionAxes.createInput()
                axisInput.setByCircularFace(face)
                centerAxis = self.getComponent().constructionAxes.add(axisInput)
                centerAxis.name = 'Gear Center'
                centerAxis.isLightBulbOn = False
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                faceGeom = face.geometry
                if (sketchPlane.isParallelToPlane(faceGeom)
                        and not sketchPlane.isCoPlanarTo(faceGeom)):
                    extrusionExtent = face

        if centerAxis is None:
            raise Exception('Could not find cylindrical face for Gear Center axis')
        if extrusionExtent is None:
            raise Exception('Could not find far end-cap face for bore extent')

        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent
        ctx.gearBody = body

    # step 10-11
    def patternTeeth(self, ctx: SpurGearGenerationContext):
        toothNumber = int(self.getParameter(PARAM_TOOTH_NUMBER).value)

        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(ctx.toothBody)

        patterns = self.getComponent().features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # CircularPatternFeature.bodies already includes the seed + copies.
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)

        combines = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        self.createFillets(ctx)

    # step 11
    def createFillets(self, ctx: SpurGearGenerationContext):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        axisNormal = get_normal(self.plane)
        axisNormal.normalize()

        edges = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) >= 0.001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                startP = edge.geometry.startPoint
                endP = edge.geometry.endPoint
                direction = startP.vectorTo(endP)
                direction.normalize()
                dot = direction.dotProduct(axisNormal)
                if abs(abs(dot) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            return

        fillets = self.getComponent().features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        fillets.add(filletInput)

    # step 12
    def buildBore(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return

        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch

        toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        toothGen.drawBore(ctx.anchorPoint, boreDiameter)

        profile = boreSketch.profiles.item(0)

        extrudes = self.getComponent().features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extInput.participantBodies = [ctx.gearBody]
        extrudes.add(extInput)

    # final cleanup
    def cleanup(self, ctx: SpurGearGenerationContext):
        # Construction planes/axes: always hidden, in both modes.
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False

        # Sketches: hidden only on the full-build path.
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
