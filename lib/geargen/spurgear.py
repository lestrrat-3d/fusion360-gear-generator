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
# Registered user-parameter names
# ---------------------------------------------------------------------------
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
PARAM_TOOTH_SPACE_ANGLE_AT_ROOT = 'ToothSpaceAngleAtRoot'
PARAM_TOOTH_SPACE_ARC_AT_ROOT = 'ToothSpaceArcAtRoot'
PARAM_FILLET_CLEARANCE = 'FilletClearance'
PARAM_FILLET_RADIUS = 'FilletRadius'


# ---------------------------------------------------------------------------
# 1. Command inputs configurator
# ---------------------------------------------------------------------------
class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # Dialog display order is fixed (see spec): plane, anchorPoint, module,
        # toothNumber, pressureAngle, boreDiameter, thickness, chamferTooth,
        # sketchOnly, parentComponent (last). Do NOT reorder by input type.

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
            INPUT_ID_THICKNESS, 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))

        # 8. Apply chamfer to teeth
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 9. Generate sketches, but do not build body
        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True,
            '', False)

        # 10. Parent Component (last; default pre-selected to root component)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1)
        parentInput.addSelection(get_design().rootComponent)


# ---------------------------------------------------------------------------
# 2. Generation context
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
# 3. Involute tooth design generator
# ---------------------------------------------------------------------------
class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        # Retained only as an incidental field; the live rotation always comes
        # from draw()'s runtime argument. Do NOT use self.toothAngle in drawTooth.
        self.toothAngle = angle
        # The movable local origin: a fresh SketchPoint at (0, 0, 0).
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

    # --- parameter accessors (reproduced surface) -------------------------
    def getParameter(self, name: str):
        return self.parent.getParameter(name)

    def getParameterValue(self, name: str):
        return self.parent.getParameter(name).value

    # --- exact involute math (pinned) -------------------------------------
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    # --- entry point ------------------------------------------------------
    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)

        # Step 5: anchor the sketch. Project the Tools-sketch anchor into this
        # sketch and make it coincident with the local origin.
        projected = self.sketch.project(anchorPoint)
        projectedPoint = projected.item(0)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedPoint)

    # --- circles ----------------------------------------------------------
    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions

        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)

        size = tipRadius - rootRadius

        def addCircle(name, radius, isConstruction):
            circle = circles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = isConstruction
            # off-centre text point for the diameter dimension
            textPoint = adsk.core.Point3D.create(radius, radius, 0)
            dims.addDiameterDimension(circle, textPoint)
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True,
                adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            return circle

        addCircle('Root Circle', rootRadius, False)
        addCircle('Tip Circle', tipRadius, True)
        addCircle('Base Circle', baseRadius, True)
        addCircle('Pitch Circle', pitchRadius, True)

    # --- tooth ------------------------------------------------------------
    def drawTooth(self, angle=0):
        sketch = self.sketch
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        involuteSteps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        # 1. Sample the involute flank from the base circle out to the tip
        #    circle in equal radial steps. First sample radius is EXACTLY the
        #    base radius (do not clamp to max(base, root)).
        samplePoints = []
        for i in range(0, involuteSteps):
            r = baseRadius + (tipRadius - baseRadius) * (i / (involuteSteps - 1))
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                continue
            samplePoints.append(p)

        # 2. Mirror across +X (negate y) so the spiral matches a left flank.
        mirrored = [(p.x, -p.y) for p in samplePoints]

        # 3. Analytic pitch-circle crossing angle of the (un-mirrored) involute.
        pitchCrossing = self.calculateInvolutePoint(baseRadius, pitchRadius)
        # After mirroring (negate y), the crossing angle is negated too.
        pitchAngle = -math.atan2(pitchCrossing.y, pitchCrossing.x)
        # Rotate so that the (mirrored) pitch crossing lands at +pi/(2*N).
        rotate_angle = (math.pi / (2 * toothNumber)) - pitchAngle

        def rotate(points, theta):
            c = math.cos(theta)
            s = math.sin(theta)
            return [(x * c - y * s, x * s + y * c) for (x, y) in points]

        # 4. Rotate the mirrored points -> left flank. Then mirror across X -> right.
        leftFlank = rotate(mirrored, rotate_angle)
        rightFlank = [(x, -y) for (x, y) in leftFlank]

        # Then apply the requested angle to the whole +X-centered tooth.
        leftFlank = rotate(leftFlank, angle)
        rightFlank = rotate(rightFlank, angle)

        # 5. Draw the two flanks as fitted splines through the point collections.
        def toCollection(points):
            coll = adsk.core.ObjectCollection.create()
            for (x, y) in points:
                coll.add(adsk.core.Point3D.create(x, y, 0))
            return coll

        splines = sketch.sketchCurves.sketchFittedSplines
        leftSpline = splines.add(toCollection(leftFlank))
        rightSpline = splines.add(toCollection(rightFlank))

        # 6. Tooth-top arc (minimal constraint set, [SPUR-F-TOOTHTOP-ARC]).
        toothTopPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(
                tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0))
        tipCircle = self._findCircleByRadius(tipRadius)
        sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)

        arcs = sketch.sketchCurves.sketchArcs
        toothTopArc = arcs.addByThreePoints(
            rightSpline.endSketchPoint,
            toothTopPoint.geometry,
            leftSpline.endSketchPoint)
        sketch.sketchDimensions.addDiameterDimension(
            toothTopArc,
            adsk.core.Point3D.create(tipRadius * math.cos(angle),
                                     tipRadius * math.sin(angle) + tipRadius * 0.1, 0))

        # 7. Spine + horizontal reference + angular pin ([SPUR-F-SPINE]).
        lines = sketch.sketchCurves.sketchLines
        spine = lines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True
        if angle == 0:
            sketch.geometricConstraints.addHorizontal(spine)
        else:
            horizontal = lines.addByTwoPoints(
                self.anchorPoint,
                adsk.core.Point3D.create(tipRadius, 0, 0))
            horizontal.isConstruction = True
            sketch.geometricConstraints.addHorizontal(horizontal)
            sketch.geometricConstraints.addCoincident(
                horizontal.endSketchPoint, tipCircle)
            spineAngularDimension = sketch.sketchDimensions.addAngularDimension(
                spine, horizontal,
                adsk.core.Point3D.create(
                    tipRadius * 0.5 * math.cos(angle / 2),
                    tipRadius * 0.5 * math.sin(angle / 2), 0))

        # 8. Ribs ([SPUR-F-RIBS]).
        leftFitPoints = leftSpline.fitPoints
        rightFitPoints = rightSpline.fitPoints
        previousMid = self.anchorPoint
        for i in range(0, leftFitPoints.count):
            leftFit = leftFitPoints.item(i)
            rightFit = rightFitPoints.item(i)

            rib = lines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True
            # 2. Dimension the rib's aligned length to its current measured value.
            ribLen = rib.startSketchPoint.geometry.distanceTo(rib.endSketchPoint.geometry)
            sketch.sketchDimensions.addDistanceDimension(
                rib.startSketchPoint, rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(0, 0, 0))

            # 3. Midpoint seeded on the spine: foot of left fit point on the
            #    line at `angle` through the local origin.
            fitX = leftFit.geometry.x
            fitY = leftFit.geometry.y
            t = fitX * math.cos(angle) + fitY * math.sin(angle)
            midpoint = sketch.sketchPoints.add(
                adsk.core.Point3D.create(t * math.cos(angle), t * math.sin(angle), 0))

            # 4. coincident midpoint on spine first
            sketch.geometricConstraints.addCoincident(midpoint, spine)
            # 5. make it the rib's midpoint
            sketch.geometricConstraints.addMidPoint(midpoint, rib)
            # 6. make the rib perpendicular to the spine
            sketch.geometricConstraints.addPerpendicular(spine, rib)

            # distance from previous midpoint (origin for the first rib)
            midDist = previousMid.geometry.distanceTo(midpoint.geometry)
            sketch.sketchDimensions.addDistanceDimension(
                previousMid, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(0, 0, 0))
            previousMid = midpoint

        # 9. Close the tooth at the root ([SPUR-F-FLANK-ROOT]).
        flankStartRadius = leftSpline.startSketchPoint.geometry.distanceTo(
            adsk.core.Point3D.create(0, 0, 0))
        embedded = flankStartRadius < rootRadius
        if not embedded:
            self._drawFlankToRoot(leftSpline.startSketchPoint, rootRadius, angle)
            self._drawFlankToRoot(rightSpline.startSketchPoint, rootRadius, angle)

        # Record the embedded flag on the parent generator.
        self.parent._lastToothEmbedded = embedded

    def _drawFlankToRoot(self, flankStartPoint, rootRadius, angle):
        sketch = self.sketch
        lines = sketch.sketchCurves.sketchLines
        rootCircle = self._findCircleByRadius(rootRadius)

        # The root end is seeded radially inward from the flank start.
        startGeom = flankStartPoint.geometry
        ang = math.atan2(startGeom.y, startGeom.x)
        rootEndGeometry = adsk.core.Point3D.create(
            rootRadius * math.cos(ang), rootRadius * math.sin(ang), 0)

        line = lines.addByTwoPoints(rootEndGeometry, flankStartPoint)
        # (a) root-end coincident to the root circle
        sketch.geometricConstraints.addCoincident(line.startSketchPoint, rootCircle)
        # (b) local origin coincident to the line (point-on-line, infinite)
        sketch.geometricConstraints.addCoincident(self.anchorPoint, line)

    def _findCircleByRadius(self, radius):
        circles = self.sketch.sketchCurves.sketchCircles
        for i in range(0, circles.count):
            c = circles.item(i)
            if abs(c.radius - radius) < 0.0001:
                return c
        return circles.item(0)

    # --- bore -------------------------------------------------------------
    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projected = sketch.project(anchorPoint)
        projectedPoint = projected.item(0)
        circles = sketch.sketchCurves.sketchCircles
        circle = circles.addByCenterRadius(projectedPoint, diameter / 2)
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(diameter / 2, diameter / 2, 0))
        return circle


# ---------------------------------------------------------------------------
# 4. The orchestrator
# ---------------------------------------------------------------------------
class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False
        self.normalizedPlane = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self):
        return SpurGearGenerationContext()

    # --- subclass hooks ---------------------------------------------------
    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        pass

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def chamferWantEdges(self):
        return 6

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    # --- input processing -------------------------------------------------
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Pull every selection input out FIRST (before occurrence creation),
        #    to dodge the context shift. (Generation Order / [PB-SELECTION-STASH].)
        (parentList, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentList) != 1:
            raise Exception('Exactly one Parent Component must be selected')
        parentEntity = parentList[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        else:
            self.parentComponent = parentEntity

        (planeList, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeList) != 1:
            raise Exception('Exactly one Target Plane must be selected')
        self.plane = planeList[0]

        (anchorList, _) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchorList) != 1:
            raise Exception('Exactly one Anchor Point must be selected')
        self.anchorPoint = anchorList[0]

        # 2/3. Register input-sourced numeric/boolean parameters. The first
        #      addParameter call triggers occurrence creation.
        (module, ok) = get_value(inputs, INPUT_ID_MODULE, '')
        self.addParameter(PARAM_MODULE, module, '', 'Module of the gear')

        (toothNumber, ok) = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Number of teeth')

        (pressureAngle, ok) = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad', 'Pressure angle')

        (boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        if not ok:
            # get_value returned the evaluated value as a raw number (internal cm).
            boreDiameter = adsk.core.ValueInput.createByReal(
                boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm', 'Bore diameter')

        (thickness, ok) = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm', 'Thickness of the gear')

        (chamferTooth, ok) = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm', 'Chamfer applied to teeth')

        (sketchOnly, ok) = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only (1=true, 0=false)')

        # 4. subclass primary parameters before derived parameters reference them.
        self.addExtraPrimaryParameters(inputs)

        # 5. Register derived parameters as live Fusion expression strings.
        self.addParameter(
            PARAM_PITCH_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_MODULE)} * {self.parameterName(PARAM_TOOTH_NUMBER)}'),
            'mm', 'Pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_RADIUS,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_PITCH_DIAMETER)} / 2'),
            'mm', 'Pitch circle radius')
        self.addParameter(
            PARAM_BASE_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_PITCH_DIAMETER)} * cos({self.parameterName(PARAM_PRESSURE_ANGLE)})'),
            'mm', 'Base circle diameter')
        self.addParameter(
            PARAM_BASE_RADIUS,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_BASE_DIAMETER)} / 2'),
            'mm', 'Base circle radius')
        self.addParameter(
            PARAM_ROOT_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_PITCH_DIAMETER)} - 2.5 * {self.parameterName(PARAM_MODULE)}'),
            'mm', 'Root circle diameter')
        self.addParameter(
            PARAM_ROOT_RADIUS,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_ROOT_DIAMETER)} / 2'),
            'mm', 'Root circle radius')
        self.addParameter(
            PARAM_TIP_DIAMETER,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_PITCH_DIAMETER)} + 2 * {self.parameterName(PARAM_MODULE)}'),
            'mm', 'Tip circle diameter')
        self.addParameter(
            PARAM_TIP_RADIUS,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_TIP_DIAMETER)} / 2'),
            'mm', 'Tip circle radius')
        self.addParameter(
            PARAM_INVOLUTE_STEPS,
            adsk.core.ValueInput.createByReal(15),
            '', 'Number of involute samples')

        # Tooth Space Angle At Root: pre-evaluated in Python (Fusion's expr
        # engine refuses to mix unitless tan() with the radian PressureAngle).
        # Register UNITLESS (''), not 'rad' -- it is multiplied by a length below.
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = (math.pi / toothNumberValue) - 2 * (
            math.tan(pressureAngleValue) - pressureAngleValue)
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            adsk.core.ValueInput.createByReal(toothSpaceAngle),
            '', 'Tooth space angle at the root circle (radian magnitude, unitless)')

        self.addParameter(
            PARAM_TOOTH_SPACE_ARC_AT_ROOT,
            adsk.core.ValueInput.createByString(
                f'{self.parameterName(PARAM_ROOT_RADIUS)} * {self.parameterName(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)}'),
            'mm', 'Tooth space arc length at the root circle')

        self.addParameter(
            PARAM_FILLET_CLEARANCE,
            adsk.core.ValueInput.createByReal(0.9),
            '', 'Fraction of half-valley arc used for the fillet radius')

        self.addParameter(
            PARAM_FILLET_RADIUS,
            adsk.core.ValueInput.createByString(
                f'({self.parameterName(PARAM_TOOTH_SPACE_ARC_AT_ROOT)} / 2) * '
                f'{self.parameterName(PARAM_FILLET_CLEARANCE)} * {self.filletHelixFactorExpression()}'),
            'mm', 'Fillet radius at the tooth root')

    # --- top-level orchestration -----------------------------------------
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

    # --- steps 1-2 --------------------------------------------------------
    def prepareTools(self, ctx: SpurGearGenerationContext):
        # Tools sketch + ctx.anchorPoint
        toolsSketch = self.createSketchObject('Tools', plane=self.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch
        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)

        # Extrusion End Plane at distance Thickness from the target plane.
        planeInput = self.getComponent().constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, self.getParameterAsValueInput(PARAM_THICKNESS))
        endPlane = self.getComponent().constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 6: show the Gear Profile sketch and stop.
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    # --- step 3-5 ---------------------------------------------------------
    def buildSketches(self, ctx: SpurGearGenerationContext):
        gearProfileSketch = self.createSketchObject('Gear Profile', plane=self.plane)
        gearProfileSketch.isVisible = True
        ctx.gearProfileSketch = gearProfileSketch

        generator = SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self)
        generator.draw(ctx.anchorPoint, angle=0)

        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # --- step 7-8 ---------------------------------------------------------
    def buildTooth(self, ctx: SpurGearGenerationContext):
        profile = self._findToothProfile(ctx)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

        self.chamferTooth(ctx)

    def _findToothProfile(self, ctx: SpurGearGenerationContext):
        if ctx.toothProfileIsEmbedded:
            expectedCount = 4
        else:
            expectedCount = 6
        sketch = ctx.gearProfileSketch
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count != expectedCount:
                    continue
                nurbs = 0
                arcs = 0
                lineCount = 0
                for k in range(0, loop.profileCurves.count):
                    curveType = loop.profileCurves.item(k).geometry.curveType
                    if curveType == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif curveType == adsk.core.Curve3DTypes.Line3DCurveType:
                        lineCount += 1
                expectedLines = 0 if ctx.toothProfileIsEmbedded else 2
                if nurbs == 2 and arcs == 2 and lineCount == expectedLines:
                    return profile
        raise Exception('Could not find the tooth cross-section profile')

    def chamferTooth(self, ctx: SpurGearGenerationContext):
        chamferValue = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferValue <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value

        frontFace = self._findToothFrontFace(ctx)
        edges = adsk.core.ObjectCollection.create()
        for i in range(0, frontFace.edges.count):
            edge = frontFace.edges.item(i)
            geom = edge.geometry
            if geom.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(geom.radius - rootRadius) < 0.001:
                    continue
            edges.add(edge)

        chamfers = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferValue), False)
        chamfers.add(chamferInput)

    def _findToothFrontFace(self, ctx: SpurGearGenerationContext):
        wantEdges = self.chamferWantEdges()
        for i in range(0, ctx.toothBody.faces.count):
            face = ctx.toothBody.faces.item(i)
            if face.edges.count == wantEdges:
                return face
        raise Exception(
            'Could not find the tooth front face (expected {} edges)'.format(wantEdges))

    # --- step 9 -----------------------------------------------------------
    def buildBody(self, ctx: SpurGearGenerationContext):
        bodyProfile = self._findBodyProfile(ctx)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            bodyProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'
        gearBody = extrude.bodies.item(0)
        gearBody.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry

        centerAxis = None
        extrusionExtent = None
        for i in range(0, gearBody.faces.count):
            face = gearBody.faces.item(i)
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and centerAxis is None:
                axes = self.getComponent().constructionAxes
                axisInput = axes.createInput()
                axisInput.setByCircularFace(face)
                centerAxis = axes.add(axisInput)
                centerAxis.name = 'Gear Center'
                centerAxis.isLightBulbOn = False
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                faceGeom = face.geometry
                if sketchPlane.isParallelToPlane(faceGeom) and not sketchPlane.isCoPlanarTo(faceGeom):
                    extrusionExtent = face

        if centerAxis is None:
            raise Exception('Could not build the Gear Center axis (no cylindrical face found)')
        if extrusionExtent is None:
            raise Exception('Could not find the far end-cap face for the bore extent')

        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent
        ctx.gearBody = gearBody

    def _findBodyProfile(self, ctx: SpurGearGenerationContext):
        sketch = ctx.gearProfileSketch
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count != 2:
                    continue
                allArcs = True
                for k in range(0, loop.profileCurves.count):
                    if loop.profileCurves.item(k).geometry.curveType != adsk.core.Curve3DTypes.Arc3DCurveType:
                        allArcs = False
                        break
                if allArcs:
                    return profile
        raise Exception('Could not find the annular gear body profile (2 arcs)')

    # --- step 10-11 -------------------------------------------------------
    def patternTeeth(self, ctx: SpurGearGenerationContext):
        toothNumber = int(self.getParameter(PARAM_TOOTH_NUMBER).value)

        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(ctx.toothBody)

        patterns = self.getComponent().features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternResult = patterns.add(patternInput)

        # CircularPatternFeature.bodies already includes ALL N tooth bodies
        # (original + N-1 copies). Feed the whole collection -- do not re-add.
        combineTools = adsk.core.ObjectCollection.create()
        for i in range(0, patternResult.bodies.count):
            combineTools.add(patternResult.bodies.item(i))

        combines = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        self.createFillets(ctx)

    def createFillets(self, ctx: SpurGearGenerationContext):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)

        edges = adsk.core.ObjectCollection.create()
        for i in range(0, ctx.gearBody.faces.count):
            face = ctx.gearBody.faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) >= 0.001:
                continue
            for j in range(0, face.edges.count):
                edge = face.edges.item(j)
                geom = edge.geometry
                if geom.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = geom.startPoint.vectorTo(geom.endPoint)
                direction.normalize()
                dot = direction.dotProduct(axisNormal)
                if abs(abs(dot) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            return

        fillets = self.getComponent().features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.edgeSetInputs.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        fillets.add(filletInput)

    # --- step 12 ----------------------------------------------------------
    def buildBore(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch

        projected = boreSketch.project(ctx.anchorPoint)
        projectedPoint = projected.item(0)
        circles = boreSketch.sketchCurves.sketchCircles
        circle = circles.addByCenterRadius(projectedPoint, boreDiameter / 2)
        boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter / 2, boreDiameter / 2, 0))

        profile = boreSketch.profiles.item(0)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    # --- cleanup ----------------------------------------------------------
    def cleanup(self, ctx: SpurGearGenerationContext):
        sketchOnly = self.getParameterAsBoolean(PARAM_SKETCH_ONLY)

        # Construction-plane/axis hiding ALWAYS runs (both modes).
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False

        # Sketch hiding runs only on the full-build path.
        if not sketchOnly:
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
