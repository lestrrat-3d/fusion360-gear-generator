import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal

import adsk.core, adsk.fusion


# --- Dialog input ids ---------------------------------------------------------
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

# --- User-parameter names -----------------------------------------------------
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

INVOLUTE_STEPS = 15
FILLET_CLEARANCE = 0.9


class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to sketch the gear on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point to center the gear on')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1)

        # 3. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        # 4. Tooth Number
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '',
            adsk.core.ValueInput.createByReal(17))

        # 5. Pressure Angle
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))

        # 6. Bore Diameter (string value input so expressions are accepted)
        inputs.addStringValueInput(
            INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

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
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True, '', False)

        # 10. Parent Component (last; defaults to root component)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1)
        parentInput.addSelection(get_design().rootComponent)


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


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        # The movable local origin: a fresh SketchPoint at (0,0,0) that the
        # whole tooth construction is built relative to. NOT sketch.originPoint.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

    # --- parameter accessors (reproduced surface) ----------------------------
    def getParameter(self, name: str):
        return self.parent.getParameter(name)

    def getParameterValue(self, name: str):
        return self.parent.getParameter(name).value

    # --- involute math -------------------------------------------------------
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    # --- drawing -------------------------------------------------------------
    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        # Step 5: anchor this sketch onto the user's anchor as the final action.
        projected = self.sketch.project(anchorPoint)
        projectedAnchor = projected.item(0)
        self.sketch.geometricConstraints.addCoincident(projectedAnchor, self.anchorPoint)

    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions

        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)

        size = tipRadius - rootRadius

        def label(circle, name, radius):
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(text, size)
            textInput.setAsAlongPath(
                circle, True,
                adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)

        def dimensionPoint(radius):
            return adsk.core.Point3D.create(radius, radius, 0)

        # 1. Root Circle (solid)
        rootCircle = circles.addByCenterRadius(self.anchorPoint, rootRadius)
        rootCircle.isConstruction = False
        dims.addDiameterDimension(rootCircle, dimensionPoint(rootRadius))
        label(rootCircle, 'Root Circle', rootRadius)
        self.rootCircle = rootCircle

        # 2. Tip Circle (construction)
        tipCircle = circles.addByCenterRadius(self.anchorPoint, tipRadius)
        tipCircle.isConstruction = True
        dims.addDiameterDimension(tipCircle, dimensionPoint(tipRadius))
        label(tipCircle, 'Tip Circle', tipRadius)

        # 3. Base Circle (construction)
        baseCircle = circles.addByCenterRadius(self.anchorPoint, baseRadius)
        baseCircle.isConstruction = True
        dims.addDiameterDimension(baseCircle, dimensionPoint(baseRadius))
        label(baseCircle, 'Base Circle', baseRadius)

        # 4. Pitch Circle (construction)
        pitchCircle = circles.addByCenterRadius(self.anchorPoint, pitchRadius)
        pitchCircle.isConstruction = True
        dims.addDiameterDimension(pitchCircle, dimensionPoint(pitchRadius))
        label(pitchCircle, 'Pitch Circle', pitchRadius)

        self.tipCircle = tipCircle

    def drawTooth(self, angle):
        sketch = self.sketch
        lines = sketch.sketchCurves.sketchLines
        arcs = sketch.sketchCurves.sketchArcs
        splines = sketch.sketchCurves.sketchFittedSplines
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        toothNumber = int(self.getParameterValue(PARAM_TOOTH_NUMBER))
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        def rotate(p, a):
            ca = math.cos(a)
            sa = math.sin(a)
            return adsk.core.Point3D.create(
                p.x * ca - p.y * sa,
                p.x * sa + p.y * ca,
                0)

        # 1. Sample the involute flank, starting exactly on the base circle and
        #    walking outward to the tip circle in equal radial steps.
        startRadius = baseRadius
        endRadius = tipRadius
        if steps > 1:
            radialStep = (endRadius - startRadius) / (steps - 1)
        else:
            radialStep = 0
        rawPoints = []
        for i in range(steps):
            r = startRadius + radialStep * i
            pt = self.calculateInvolutePoint(baseRadius, r)
            if pt is None:
                continue
            rawPoints.append(pt)

        # 2. Mirror across +X (negate y) so the spiral matches a left flank.
        mirrored = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in rawPoints]

        # 3. Determine rotate_angle analytically from the pitch-circle crossing.
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        # apply the same mirror to the analytic crossing
        pitchCrossAngle = math.atan2(-pitchPoint.y, pitchPoint.x)
        targetAngle = math.pi / (2 * toothNumber)
        rotate_angle = targetAngle - pitchCrossAngle

        # 4. Rotate the mirrored samples by rotate_angle to form the left flank,
        #    then mirror across X for the right flank. Then apply `angle`.
        leftPoints = [rotate(p, rotate_angle) for p in mirrored]
        rightPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in leftPoints]

        if angle != 0:
            leftPoints = [rotate(p, angle) for p in leftPoints]
            rightPoints = [rotate(p, angle) for p in rightPoints]

        # 5. Draw the two flanks as fitted splines.
        leftCollection = adsk.core.ObjectCollection.create()
        for p in leftPoints:
            leftCollection.add(p)
        leftSpline = splines.add(leftCollection)

        rightCollection = adsk.core.ObjectCollection.create()
        for p in rightPoints:
            rightCollection.add(p)
        rightSpline = splines.add(rightCollection)

        # 6. Tooth-top arc.
        toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(
            tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0))
        constraints.addCoincident(toothTopPoint, self.tipCircle)

        topArc = arcs.addByThreePoints(
            rightSpline.endSketchPoint,
            toothTopPoint.geometry,
            leftSpline.endSketchPoint)
        dims.addDiameterDimension(
            topArc,
            adsk.core.Point3D.create(tipRadius, tipRadius, 0))

        # 7. Spine: construction line from local origin to tooth-top point.
        spine = lines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True

        spineAngularDimension = None
        if angle == 0:
            constraints.addHorizontal(spine)
        else:
            # horizontal reference always pointing +X
            horizontal = lines.addByTwoPoints(
                self.anchorPoint,
                adsk.core.Point3D.create(tipRadius, 0, 0))
            horizontal.isConstruction = True
            constraints.addHorizontal(horizontal)
            smallR = tipRadius * 0.5
            textPoint = adsk.core.Point3D.create(
                smallR * math.cos(angle / 2),
                smallR * math.sin(angle / 2),
                0)
            spineAngularDimension = dims.addAngularDimension(spine, horizontal, textPoint)

        # 8. Ribs between matching flank fit-points.
        leftFitPoints = leftSpline.fitPoints
        rightFitPoints = rightSpline.fitPoints
        previousMidpoint = self.anchorPoint
        n = min(leftFitPoints.count, rightFitPoints.count)
        for i in range(n):
            leftFit = leftFitPoints.item(i)
            rightFit = rightFitPoints.item(i)

            # 1. rib line sharing the two fit points
            rib = lines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True

            # 2. aligned length dimension at its current measured value
            dims.addDistanceDimension(
                rib.startSketchPoint, rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (leftFit.geometry.x + rightFit.geometry.x) / 2,
                    (leftFit.geometry.y + rightFit.geometry.y) / 2,
                    0))

            # 3. midpoint, seeded on the spine (foot of the left fit point)
            fitX = leftFit.geometry.x
            fitY = leftFit.geometry.y
            t = fitX * math.cos(angle) + fitY * math.sin(angle)
            midpoint = sketch.sketchPoints.add(adsk.core.Point3D.create(
                t * math.cos(angle), t * math.sin(angle), 0))

            # 4-6. coincident on spine, then midpoint, then perpendicular
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            constraints.addPerpendicular(spine, rib)

            # chain distance from previous midpoint (origin for the first rib)
            dims.addDistanceDimension(
                previousMidpoint, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (previousMidpoint.geometry.x + midpoint.geometry.x) / 2,
                    (previousMidpoint.geometry.y + midpoint.geometry.y) / 2,
                    0))
            previousMidpoint = midpoint

        # 9. Flank-to-root lines (common case) or embedded profile.
        flankStartRadius = math.hypot(
            leftSpline.fitPoints.item(0).geometry.x,
            leftSpline.fitPoints.item(0).geometry.y)
        embedded = flankStartRadius <= rootRadius

        if not embedded:
            leftStart = leftSpline.startSketchPoint
            rightStart = rightSpline.startSketchPoint

            leftRootGeom = adsk.core.Point3D.create(
                rootRadius * leftStart.geometry.x / flankStartRadius,
                rootRadius * leftStart.geometry.y / flankStartRadius,
                0)
            leftRootLine = lines.addByTwoPoints(leftRootGeom, leftStart)
            constraints.addCoincident(leftRootLine.startSketchPoint, self.rootCircle)
            constraints.addCoincident(self.anchorPoint, leftRootLine)

            rightRootGeom = adsk.core.Point3D.create(
                rootRadius * rightStart.geometry.x / flankStartRadius,
                rootRadius * rightStart.geometry.y / flankStartRadius,
                0)
            rightRootLine = lines.addByTwoPoints(rightRootGeom, rightStart)
            constraints.addCoincident(rightRootLine.startSketchPoint, self.rootCircle)
            constraints.addCoincident(self.anchorPoint, rightRootLine)

        # Record on the parent generator (the tooth generator has no ctx).
        self.parent._lastToothEmbedded = embedded

        # Confirm the rotation (helical/herringbone); no-op for spur (angle==0).
        if angle != 0 and spineAngularDimension is not None:
            spineAngularDimension.parameter.value = angle

    def drawBore(self, diameter):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        circle = circles.addByCenterRadius(self.anchorPoint, diameter / 2)
        circle.isConstruction = False
        return circle


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        self.toolsSketch = None
        self.boreSketch = None
        self.normalizedPlane = None
        self._lastToothEmbedded = False

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

    # --- hooks subclasses override -------------------------------------------
    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        pass

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def chamferWantEdges(self):
        return 6

    # --- input processing ----------------------------------------------------
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Read selection inputs FIRST (before any occurrence creation).
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one parent component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parent.component
        elif parent.objectType == adsk.fusion.Component.classType():
            self.parentComponent = parent
        else:
            raise Exception('Selected parent is not a component')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        self.plane = planes[0]

        (anchors, _) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception('Exactly one anchor point must be selected')
        self.anchorPoint = anchors[0]

        # 3. Register numeric/bool input parameters (creates the occurrence).
        (module, ok) = get_value(inputs, INPUT_ID_MODULE, '')
        self.addParameter(PARAM_MODULE, module, '', 'Module of the gear')

        (toothNumber, ok) = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Number of teeth')

        (pressureAngle, ok) = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad', 'Pressure angle')

        (boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        if not ok:
            boreDiameter = adsk.core.ValueInput.createByReal(
                boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm', 'Bore diameter')

        (thickness, ok) = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm', 'Thickness of the gear body')

        (chamferTooth, ok) = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm', 'Tooth chamfer distance')

        (sketchOnly, ok) = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only (1=true, 0=false)')

        # 4. subclass extra primary parameters (before derived parameters).
        self.addExtraPrimaryParameters(inputs)

        # 5. Derived parameters as live Fusion expression strings.
        def expr(s):
            return adsk.core.ValueInput.createByString(s)

        pn = self.parameterName

        self.addParameter(
            PARAM_PITCH_CIRCLE_DIAMETER,
            expr(f'{pn(PARAM_MODULE)} * {pn(PARAM_TOOTH_NUMBER)}'),
            'mm', 'Pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_CIRCLE_RADIUS,
            expr(f'{pn(PARAM_PITCH_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Pitch circle radius')
        self.addParameter(
            PARAM_BASE_CIRCLE_DIAMETER,
            expr(f'{pn(PARAM_PITCH_CIRCLE_DIAMETER)} * cos({pn(PARAM_PRESSURE_ANGLE)})'),
            'mm', 'Base circle diameter')
        self.addParameter(
            PARAM_BASE_CIRCLE_RADIUS,
            expr(f'{pn(PARAM_BASE_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Base circle radius')
        self.addParameter(
            PARAM_ROOT_CIRCLE_DIAMETER,
            expr(f'{pn(PARAM_PITCH_CIRCLE_DIAMETER)} - 2.5 * {pn(PARAM_MODULE)}'),
            'mm', 'Root circle diameter')
        self.addParameter(
            PARAM_ROOT_CIRCLE_RADIUS,
            expr(f'{pn(PARAM_ROOT_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Root circle radius')
        self.addParameter(
            PARAM_TIP_CIRCLE_DIAMETER,
            expr(f'{pn(PARAM_PITCH_CIRCLE_DIAMETER)} + 2 * {pn(PARAM_MODULE)}'),
            'mm', 'Tip circle diameter')
        self.addParameter(
            PARAM_TIP_CIRCLE_RADIUS,
            expr(f'{pn(PARAM_TIP_CIRCLE_DIAMETER)} / 2'),
            'mm', 'Tip circle radius')

        self.addParameter(
            PARAM_INVOLUTE_STEPS,
            adsk.core.ValueInput.createByReal(INVOLUTE_STEPS),
            '', 'Number of involute flank samples')

        # ToothSpaceAngleAtRoot: pre-computed in Python, registered UNITLESS.
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        toothSpaceAngleAtRoot = (
            math.pi / toothNumberValue
            - 2 * (math.tan(pressureAngleValue) - pressureAngleValue))
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            adsk.core.ValueInput.createByReal(toothSpaceAngleAtRoot),
            '', 'Tooth space angle at the root circle (radians, unitless)')

        # ToothSpaceArcAtRoot: live expression (length).
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC_AT_ROOT,
            expr(f'{pn(PARAM_ROOT_CIRCLE_RADIUS)} * {pn(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)}'),
            'mm', 'Tooth space arc length at the root circle')

        self.addParameter(
            PARAM_FILLET_CLEARANCE,
            adsk.core.ValueInput.createByReal(FILLET_CLEARANCE),
            '', 'Fraction of the half-valley arc used for the fillet radius')

        self.addParameter(
            PARAM_FILLET_RADIUS,
            expr(f'({pn(PARAM_TOOTH_SPACE_ARC_AT_ROOT)} / 2) * {pn(PARAM_FILLET_CLEARANCE)} * {self.filletHelixFactorExpression()}'),
            'mm', 'Root fillet radius')

    # --- orchestration -------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalize plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.normalizedPlane = component.constructionPlanes.add(planeInput)
            self.plane = self.normalizedPlane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx):
        component = self.getComponent()

        # Step 2: Tools sketch.
        toolsSketch = self.createSketchObject('Tools', plane=self.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch
        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)

        # Extrusion End Plane offset by Thickness.
        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        endPlane = component.constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    def buildMainGearBody(self, ctx):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    def buildSketches(self, ctx):
        sketch = self.createSketchObject('Gear Profile', plane=self.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch

        SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    def buildTooth(self, ctx):
        self.extrudeTooth(ctx)
        self.chamferTooth(ctx)

    def extrudeTooth(self, ctx):
        sketch = ctx.gearProfileSketch
        expected = 4 if ctx.toothProfileIsEmbedded else 6

        toothProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == expected:
                    toothProfile = profile
                    break
            if toothProfile is not None:
                break
        if toothProfile is None:
            raise Exception('Could not find tooth profile to extrude')

        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeResult = extrudes.add(extrudeInput)
        extrudeResult.name = 'Extrude tooth'
        ctx.toothBody = extrudeResult.bodies.item(0)

    def chamferTooth(self, ctx):
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        wantEdges = self.chamferWantEdges()

        frontFace = None
        for face in ctx.toothBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            if face.edges.count == wantEdges:
                frontFace = face
                break
        if frontFace is None:
            raise Exception('Could not find tooth front face to chamfer')

        edges = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            geom = edge.geometry
            if geom.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(geom.radius - rootRadius) < 0.001:
                    continue
            edges.add(edge)

        chamfers = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferDistance), False)
        chamfers.add(chamferInput)

    def buildBody(self, ctx):
        sketch = ctx.gearProfileSketch

        bodyProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count != 2:
                    continue
                allArcs = True
                for curve in loop.profileCurves:
                    if curve.geometry.curveType != adsk.core.Curve3DTypes.Arc3DCurveType:
                        allArcs = False
                        break
                if allArcs:
                    bodyProfile = profile
                    break
            if bodyProfile is not None:
                break
        if bodyProfile is None:
            raise Exception('Could not find annular gear-body profile')

        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            bodyProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeResult = extrudes.add(extrudeInput)
        extrudeResult.name = 'Extrude body'

        gearBody = extrudeResult.bodies.item(0)
        gearBody.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry

        for face in gearBody.faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if ctx.centerAxis is None:
                    axisInput = self.getComponent().constructionAxes.createInput()
                    axisInput.setByCircularFace(face)
                    axis = self.getComponent().constructionAxes.add(axisInput)
                    axis.name = 'Gear Center'
                    axis.isLightBulbOn = False
                    ctx.centerAxis = axis
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if (sketchPlane.isParallelToPlane(face.geometry)
                        and not sketchPlane.isCoPlanarTo(face.geometry)):
                    ctx.extrusionExtent = face

        if ctx.centerAxis is None:
            raise Exception('Could not build Gear Center axis')
        if ctx.extrusionExtent is None:
            raise Exception('Could not find far end-cap face')

        ctx.gearBody = gearBody

    def patternTeeth(self, ctx):
        toothNumber = int(self.getParameter(PARAM_TOOTH_NUMBER).value)

        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(ctx.toothBody)

        patterns = self.getComponent().features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(toothNumber))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # CircularPatternFeature.bodies includes the seed + N-1 copies.
        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))

        combines = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        self.createFillets(ctx)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        axisNormal = get_normal(self.plane)

        def dot(a, b):
            return a.x * b.x + a.y * b.y + a.z * b.z

        edges = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) >= 0.001:
                continue
            for edge in face.edges:
                geom = edge.geometry
                if geom.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = geom.startPoint.vectorTo(geom.endPoint)
                direction.normalize()
                if abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            return

        fillets = self.getComponent().features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        fillets.add(filletInput)

    def buildBore(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        sketch = self.createSketchObject('Bore Profile', plane=self.plane)
        sketch.isVisible = True
        self.boreSketch = sketch

        projected = sketch.project(ctx.anchorPoint)
        anchor = projected.item(0)

        # Draw the bore circle centered on the projected anchor directly.
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(anchor, boreDiameter / 2)
        circle.isConstruction = False
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter / 2, boreDiameter / 2, 0))

        boreProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 1:
                    boreProfile = profile
                    break
            if boreProfile is not None:
                break
        if boreProfile is None:
            raise Exception('Could not find bore profile')

        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudeResult = extrudes.add(extrudeInput)
        extrudeResult.name = 'Bore cut'

    def cleanup(self, ctx):
        sketchOnly = self.getParameterAsBoolean(PARAM_SKETCH_ONLY)

        # Construction planes/axes always hidden, in both modes.
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False

        # Sketch hiding only on the full-build path.
        if not sketchOnly:
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
