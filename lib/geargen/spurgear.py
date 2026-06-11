import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import find_profile_by_curve_counts

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
# User-parameter names (the <prefix>_<name> table)
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

INVOLUTE_STEPS = 15
FILLET_CLEARANCE = 0.9


class SpurGearCommandInputsConfigurator:
    """Adds the dialog inputs, in the fixed display order, to the command."""

    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs
        design = get_design()

        # 1. Target Plane
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane on which the gear front face is sketched')
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the gear center is aligned with')
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

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

        # 6. Bore Diameter (string input so it accepts expressions)
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
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body',
            True, '', False)

        # 10. Parent Component (last; defaults to the root component)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the new gear occurrence is nested under')
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(0, 1)
        parentInput.addSelection(design.rootComponent)


class SpurGearGenerationContext(GenerationContext):
    """Data carrier passed between generation steps; subclasses read these
    field names, so they are public API."""

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
    """Draws the involute tooth profile (four circles + a single tooth) into a
    sketch. Borrowed by the helical/herringbone subclasses and (via a proxy) by
    the bevel gear."""

    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        # Retained as an incidental field only. drawTooth rotates by the live
        # argument flowing in from draw(), NOT by this stored value.
        self.toothAngle = angle
        # A fresh movable local origin at (0,0,0) — NOT sketch.originPoint.
        self.anchorPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))
        # References to the four circles drawn by drawCircles().
        self.rootCircle = None
        self.tipCircle = None
        self.baseCircle = None
        self.pitchCircle = None

    # -- parameter accessors (reproduced surface) ---------------------------
    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name):
        return self.parent.getParameter(name).value

    # -----------------------------------------------------------------------
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        """Point on the involute of baseRadius where the unrolled string
        reaches intersectionRadius. Returns None when the sample sits inside
        the base circle."""
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)          # curve parameter is tan(alpha), NOT inv(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)

        # -- step 5: anchor the sketch (final action of draw()) -------------
        projected = self.sketch.project(anchorPoint)
        projectedPoint = projected.item(0)
        self.sketch.geometricConstraints.addCoincident(
            projectedPoint, self.anchorPoint)

    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions

        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)

        size = tipRadius - rootRadius

        def make(name, radius, isConstruction):
            # Share the local origin directly as the center (no separate
            # coincident) so all four circles share that one point.
            circle = circles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = isConstruction
            # Driving diameter dimension; text point off-centre.
            dims.addDiameterDimension(
                circle, adsk.core.Point3D.create(radius, 0, 0))
            # Along-path label.
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True,
                adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            return circle

        self.rootCircle = make('Root Circle', rootRadius, False)
        self.tipCircle = make('Tip Circle', tipRadius, True)
        self.baseCircle = make('Base Circle', baseRadius, True)
        self.pitchCircle = make('Pitch Circle', pitchRadius, True)

    def drawTooth(self, angle=0):
        sketch = self.sketch
        baseRadius = self.getParameterValue(PARAM_BASE_CIRCLE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_CIRCLE_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_CIRCLE_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        # -- step 4.1: sample the involute flank -----------------------------
        # First sample radius is exactly baseRadius (no clamping to root).
        samples = []
        for i in range(steps):
            r = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                continue
            samples.append(p)

        # -- step 4.2: mirror across +X so the spiral matches a left flank ---
        mirrored = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in samples]

        # -- step 4.3: analytic pitch-circle crossing angle ------------------
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        # Mirror it too (we measure on the mirrored curve).
        pitchAngle = math.atan2(-pitchPoint.y, pitchPoint.x)
        # Rotate so that pitch crossing lands at +pi/(2*toothNumber).
        rotate_angle = (math.pi / (2.0 * toothNumber)) - pitchAngle

        def rotate(points, a):
            ca = math.cos(a)
            sa = math.sin(a)
            return [adsk.core.Point3D.create(
                p.x * ca - p.y * sa, p.x * sa + p.y * ca, 0) for p in points]

        # -- step 4.4: rotate to form the left flank, mirror for the right ---
        leftPoints = rotate(mirrored, rotate_angle)
        rightPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in leftPoints]

        # Apply the requested `angle` to the whole +X-centered tooth.
        leftPoints = rotate(leftPoints, angle)
        rightPoints = rotate(rightPoints, angle)

        # Tooth-top point at the tip circle, rotated by `angle`.
        toothTopCoord = adsk.core.Point3D.create(
            tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0)

        # -- step 4.5: draw the flanks as fitted splines ---------------------
        fittedSplines = sketch.sketchCurves.sketchFittedSplines

        leftCollection = adsk.core.ObjectCollection.create()
        for p in leftPoints:
            leftCollection.add(p)
        leftSpline = fittedSplines.add(leftCollection)

        rightCollection = adsk.core.ObjectCollection.create()
        for p in rightPoints:
            rightCollection.add(p)
        rightSpline = fittedSplines.add(rightCollection)

        # -- step 4.6: tooth-top arc [SPUR-F-TOOTHTOP-ARC] -------------------
        toothTopPoint = sketch.sketchPoints.add(toothTopCoord)
        sketch.geometricConstraints.addCoincident(toothTopPoint, self.tipCircle)
        arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            rightSpline.endSketchPoint,
            toothTopPoint.geometry,
            leftSpline.endSketchPoint)
        sketch.sketchDimensions.addDiameterDimension(
            arc, adsk.core.Point3D.create(toothTopCoord.x, toothTopCoord.y, 0))

        # -- step 4.7: spine [SPUR-F-SPINE] ----------------------------------
        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            self.anchorPoint, toothTopPoint)
        spine.isConstruction = True
        spineAngularDimension = None
        if angle == 0:
            sketch.geometricConstraints.addHorizontal(spine)
        else:
            horizontal = sketch.sketchCurves.sketchLines.addByTwoPoints(
                self.anchorPoint,
                adsk.core.Point3D.create(tipRadius, 0, 0))
            horizontal.isConstruction = True
            sketch.geometricConstraints.addHorizontal(horizontal)
            # Pin the far endpoint so the reference line's length is fixed.
            sketch.geometricConstraints.addCoincident(
                horizontal.endSketchPoint, self.tipCircle)
            # Angular dimension from spine to horizontal; text on the bisector.
            bisR = tipRadius * 0.5
            spineAngularDimension = sketch.sketchDimensions.addAngularDimension(
                spine, horizontal,
                adsk.core.Point3D.create(
                    bisR * math.cos(angle / 2.0),
                    bisR * math.sin(angle / 2.0), 0))

        # -- step 4.8: ribs [SPUR-F-RIBS] ------------------------------------
        leftFit = leftSpline.fitPoints
        rightFit = rightSpline.fitPoints
        previousMidpoint = self.anchorPoint
        ribCount = min(leftFit.count, rightFit.count)
        for i in range(ribCount):
            lp = leftFit.item(i)
            rp = rightFit.item(i)
            # 1. rib shares the two fit points; construction.
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(lp, rp)
            rib.isConstruction = True
            # 2. dimension the rib's aligned length to its measured value.
            ribLen = lp.geometry.distanceTo(rp.geometry)
            ribMidText = adsk.core.Point3D.create(
                (lp.geometry.x + rp.geometry.x) / 2.0,
                (lp.geometry.y + rp.geometry.y) / 2.0, 0)
            ribDim = sketch.sketchDimensions.addDistanceDimension(
                rib.startSketchPoint, rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                ribMidText)
            ribDim.parameter.value = ribLen
            # 3. midpoint seeded on the spine (foot of the left fit point).
            fitX = lp.geometry.x
            fitY = lp.geometry.y
            t = fitX * math.cos(angle) + fitY * math.sin(angle)
            midpoint = sketch.sketchPoints.add(adsk.core.Point3D.create(
                t * math.cos(angle), t * math.sin(angle), 0))
            # 4. pin the midpoint onto the spine first.
            sketch.geometricConstraints.addCoincident(midpoint, spine)
            # 5. make it the rib's midpoint.
            sketch.geometricConstraints.addMidPoint(midpoint, rib)
            # 6. make the rib perpendicular to the spine.
            sketch.geometricConstraints.addPerpendicular(spine, rib)
            # chain: dimension distance from previous midpoint to this one.
            chainText = adsk.core.Point3D.create(
                midpoint.geometry.x, midpoint.geometry.y, 0)
            chainDim = sketch.sketchDimensions.addDistanceDimension(
                previousMidpoint, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                chainText)
            chainDim.parameter.value = previousMidpoint.geometry.distanceTo(
                midpoint.geometry)
            previousMidpoint = midpoint

        # -- step 4.9: close the tooth at the root [SPUR-F-FLANK-ROOT] -------
        leftStart = leftSpline.startSketchPoint
        rightStart = rightSpline.startSketchPoint
        flankStartRadius = math.sqrt(
            leftStart.geometry.x ** 2 + leftStart.geometry.y ** 2)

        if flankStartRadius > rootRadius:
            # Flank starts outside the root circle: draw radial flank-to-root
            # lines on each side. 6-curve loop.
            self.parent._lastToothEmbedded = False
            for flankStart in (leftStart, rightStart):
                gx = flankStart.geometry.x
                gy = flankStart.geometry.y
                norm = math.sqrt(gx * gx + gy * gy)
                rootEndSeed = adsk.core.Point3D.create(
                    gx / norm * rootRadius, gy / norm * rootRadius, 0)
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                    rootEndSeed, flankStart)
                # (a) root-end coincident to the root circle.
                sketch.geometricConstraints.addCoincident(
                    line.startSketchPoint, self.rootCircle)
                # (b) local origin coincident to the line (radial direction).
                sketch.geometricConstraints.addCoincident(
                    self.anchorPoint, line)
        else:
            # Embedded profile: no flank-to-root lines. 4-curve loop.
            self.parent._lastToothEmbedded = True

        # -- step 7 (continued): confirm the rotation [SPUR-F-ROTATE-CONFIRM]
        if angle != 0 and spineAngularDimension is not None:
            spineAngularDimension.parameter.value = angle

    def drawBore(self, anchorPoint, diameter):
        """Draws the bore circle of `diameter` (cm) centered on the projected
        anchor, with a driving diameter dimension. Returns the circle."""
        sketch = self.sketch
        projected = sketch.project(anchorPoint)
        center = projected.item(0)
        radius = diameter / 2.0
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            center, radius)
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(
                center.geometry.x + radius, center.geometry.y, 0))
        return circle


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False
        self._normalizedPlane = None

    # -- subclass hooks ------------------------------------------------------
    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def chamferWantEdges(self):
        return 6

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        # Spur base has no extra primary parameters; subclasses inject theirs.
        pass

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    # -- input reading -------------------------------------------------------
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Pull every selection out FIRST (before occurrence creation).
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one Parent Component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent

        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one Target Plane must be selected')
        self.plane = planes[0]

        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception('Exactly one Anchor Point must be selected')
        self.anchorPoint = anchors[0]

        # 2/3. Register the input-sourced numeric/bool parameters. The first
        # addParameter call creates the occurrence (context shift), but the
        # selections are already stashed on self.
        self.addParameter(
            PARAM_MODULE, get_value(inputs, INPUT_ID_MODULE, ''),
            '', 'Module of the gear')
        self.addParameter(
            PARAM_TOOTH_NUMBER, get_value(inputs, INPUT_ID_TOOTH_NUMBER, ''),
            '', 'Number of teeth')
        self.addParameter(
            PARAM_PRESSURE_ANGLE, get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad'),
            'rad', 'Pressure angle')
        self.addParameter(
            PARAM_BORE_DIAMETER, get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm'),
            'mm', 'Bore diameter (0 = no bore)')
        self.addParameter(
            PARAM_THICKNESS, get_value(inputs, INPUT_ID_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the gear body')
        self.addParameter(
            PARAM_CHAMFER_TOOTH, get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm'),
            'mm', 'Chamfer distance on the tooth front edges (0 = none)')

        sketchOnly = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only (1 = true, 0 = false)')

        # 4. Subclass primary parameters, before derived ones reference them.
        self.addExtraPrimaryParameters(inputs)

        # 5. Derived parameters as live Fusion expression strings.
        def addDerived(name, expr, units, comment):
            self.addParameter(
                name, adsk.core.ValueInput.createByString(expr), units, comment)

        nMod = self.parameterName(PARAM_MODULE)
        nTooth = self.parameterName(PARAM_TOOTH_NUMBER)
        nPress = self.parameterName(PARAM_PRESSURE_ANGLE)

        addDerived(PARAM_PITCH_CIRCLE_DIAMETER,
                   f'{nMod} * {nTooth}', 'mm', 'Pitch circle diameter')
        nPitchD = self.parameterName(PARAM_PITCH_CIRCLE_DIAMETER)
        addDerived(PARAM_PITCH_CIRCLE_RADIUS,
                   f'{nPitchD} / 2', 'mm', 'Pitch circle radius')
        addDerived(PARAM_BASE_CIRCLE_DIAMETER,
                   f'{nPitchD} * cos({nPress})', 'mm', 'Base circle diameter')
        nBaseD = self.parameterName(PARAM_BASE_CIRCLE_DIAMETER)
        addDerived(PARAM_BASE_CIRCLE_RADIUS,
                   f'{nBaseD} / 2', 'mm', 'Base circle radius')
        addDerived(PARAM_ROOT_CIRCLE_DIAMETER,
                   f'{nPitchD} - 2.5 * {nMod}', 'mm', 'Root circle diameter')
        nRootD = self.parameterName(PARAM_ROOT_CIRCLE_DIAMETER)
        addDerived(PARAM_ROOT_CIRCLE_RADIUS,
                   f'{nRootD} / 2', 'mm', 'Root circle radius')
        addDerived(PARAM_TIP_CIRCLE_DIAMETER,
                   f'{nPitchD} + 2 * {nMod}', 'mm', 'Tip circle diameter')
        nTipD = self.parameterName(PARAM_TIP_CIRCLE_DIAMETER)
        addDerived(PARAM_TIP_CIRCLE_RADIUS,
                   f'{nTipD} / 2', 'mm', 'Tip circle radius')

        addDerived(PARAM_INVOLUTE_STEPS,
                   str(INVOLUTE_STEPS), '', 'Number of involute samples')

        # Tooth Space Angle At Root: pre-evaluated in Python, registered
        # UNITLESS (radian magnitude treated as dimensionless).
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = (math.pi / toothNumberValue) - 2.0 * (
            math.tan(pressureAngleValue) - pressureAngleValue)
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            adsk.core.ValueInput.createByReal(toothSpaceAngle),
            '', 'Angular width of the valley at the root circle (radians)')

        nRootR = self.parameterName(PARAM_ROOT_CIRCLE_RADIUS)
        nSpaceAngle = self.parameterName(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)
        addDerived(PARAM_TOOTH_SPACE_ARC_AT_ROOT,
                   f'{nRootR} * {nSpaceAngle}', 'mm',
                   'Arc length of the valley along the root circle')

        addDerived(PARAM_FILLET_CLEARANCE,
                   str(FILLET_CLEARANCE), '',
                   'Fraction of the half-valley arc used for the fillet radius')

        nSpaceArc = self.parameterName(PARAM_TOOTH_SPACE_ARC_AT_ROOT)
        nFilletClear = self.parameterName(PARAM_FILLET_CLEARANCE)
        addDerived(PARAM_FILLET_RADIUS,
                   f'({nSpaceArc} / 2) * {nFilletClear} * '
                   f'{self.filletHelixFactorExpression()}',
                   'mm', 'Root fillet radius')

    # -- orchestration -------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalize the plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(
                self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self._normalizedPlane = self.plane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.cleanup(ctx)

    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 6: show the Gear Profile sketch and stop.
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    # -- step 1-2 ------------------------------------------------------------
    def prepareTools(self, ctx: SpurGearGenerationContext):
        # Step 2: Tools sketch + projected anchor.
        toolsSketch = self.createSketchObject('Tools', plane=self.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch

        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)

        # Offset construction plane used as the to-entity for the extrudes.
        thicknessValue = self.getParameter(PARAM_THICKNESS).value
        planeInput = self.getComponent().constructionPlanes.createInput()
        planeInput.setByOffset(
            self.plane, adsk.core.ValueInput.createByReal(thicknessValue))
        endPlane = self.getComponent().constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    # -- step 3-5 ------------------------------------------------------------
    def buildSketches(self, ctx: SpurGearGenerationContext):
        gearProfileSketch = self.createSketchObject(
            'Gear Profile', plane=self.plane)
        gearProfileSketch.isVisible = True
        ctx.gearProfileSketch = gearProfileSketch

        generator = SpurGearInvoluteToothDesignGenerator(
            gearProfileSketch, self)
        generator.draw(ctx.anchorPoint, angle=0)

        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # -- step 7-8 ------------------------------------------------------------
    def buildTooth(self, ctx: SpurGearGenerationContext):
        sketch = ctx.gearProfileSketch
        lines = 0 if ctx.toothProfileIsEmbedded else 2
        toothProfile = find_profile_by_curve_counts(
            sketch, nurbs=2, arcs=2, lines=lines)

        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            toothProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

        self.chamferTooth(ctx)

    def chamferTooth(self, ctx: SpurGearGenerationContext):
        chamferValue = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferValue <= 0:
            return

        wantEdges = self.chamferWantEdges()
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value

        frontFace = None
        for face in ctx.toothBody.faces:
            if face.edges.count != wantEdges:
                continue
            if sketchPlane.isCoPlanarTo(face.geometry):
                frontFace = face
                break
        if frontFace is None:
            raise Exception('Chamfer: tooth front face not found '
                            f'(want {wantEdges} edges, coplanar with sketch)')

        edges = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            curve = edge.geometry
            if (curve.curveType == adsk.core.Curve3DTypes.Arc3DCurveType and
                    abs(curve.radius - rootRadius) < 0.001):
                continue  # skip the root arc
            edges.add(edge)

        chamfers = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferValue), False)
        chamfers.add(chamferInput)

    # -- step 9 --------------------------------------------------------------
    def buildBody(self, ctx: SpurGearGenerationContext):
        sketch = ctx.gearProfileSketch
        bodyProfile = find_profile_by_curve_counts(sketch, arcs=2)

        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            bodyProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'

        body = extrude.bodies.item(0)
        body.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        centerAxis = None
        extrusionExtent = None
        for face in body.faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if centerAxis is None:
                    axisInput = self.getComponent().constructionAxes.createInput()
                    axisInput.setByCircularFace(face)
                    centerAxis = self.getComponent().constructionAxes.add(
                        axisInput)
                    centerAxis.name = 'Gear Center'
                    centerAxis.isLightBulbOn = False
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if (sketchPlane.isParallelToPlane(face.geometry) and
                        not sketchPlane.isCoPlanarTo(face.geometry)):
                    extrusionExtent = face

        if centerAxis is None:
            raise Exception('Gear body: no cylindrical face for Gear Center axis')
        if extrusionExtent is None:
            raise Exception('Gear body: far end-cap face not found')

        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent
        ctx.gearBody = body

    # -- step 10-11 ----------------------------------------------------------
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

        # pattern.bodies already includes all N tooth bodies (seed + copies).
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))

        combines = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        self.createFillets(ctx)

    def createFillets(self, ctx: SpurGearGenerationContext):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        axisNormal = self.plane.geometry.normal
        axisNormal.normalize()

        edges = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) >= 0.001:
                continue
            for edge in face.edges:
                curve = edge.geometry
                if curve.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = curve.startPoint.vectorTo(curve.endPoint)
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

    # -- step 12 -------------------------------------------------------------
    def buildBore(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch

        generator = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        circle = generator.drawBore(ctx.anchorPoint, boreDiameter)

        boreProfile = boreSketch.profiles.item(0)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    # -- cleanup (last action of generate) -----------------------------------
    def cleanup(self, ctx: SpurGearGenerationContext):
        sketchOnly = self.getParameterAsBoolean(PARAM_SKETCH_ONLY)

        # Construction plane/axis hiding ALWAYS runs (both modes).
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self._normalizedPlane is not None:
            self._normalizedPlane.isLightBulbOn = False

        # Sketch hiding runs ONLY on the full-build path.
        if not sketchOnly:
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
