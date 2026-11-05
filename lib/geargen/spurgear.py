import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts


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

class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs: adsk.core.CommandInputs = cmd.commandInputs
        selection0: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the gear on')
        selection0.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        selection0.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        selection0.setSelectionLimits(1, 1)
        selection1: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point the gear is centered on')
        selection1.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        selection1.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        selection1.setSelectionLimits(1, 1)
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))
        inputs.addValueInput(INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')
        inputs.addValueInput(INPUT_ID_THICKNESS, 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))
        inputs.addValueInput(INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))
        inputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True, '', False)
        selection9: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_PARENT, 'Parent Component', 'Select the component to build the gear in')
        selection9.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        selection9.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        selection9.setSelectionLimits(1, 1)
        selection9.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    def __init__(self):
        super().__init__()
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
        # The movable local origin (see Sketch Discipline); everything is drawn about it
        # and it is dragged onto the projected anchor at the end of draw().
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
        self.angularDimension = adsk.fusion.SketchAngularDimension.cast(None)

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        ratio = baseRadius / intersectionRadius
        if ratio > 1.0:
            ratio = 1.0
        alpha = math.acos(ratio)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return (x, y)

    def _mid(self, a: adsk.fusion.SketchPoint, b: adsk.fusion.SketchPoint, dx: float, dy: float) -> adsk.core.Point3D:
        ga: adsk.core.Point3D = a.geometry
        gb: adsk.core.Point3D = b.geometry
        return adsk.core.Point3D.create((ga.x + gb.x) / 2 + dx, (ga.y + gb.y) / 2 + dy, 0)

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        # Step-5 anchoring: re-project the Tools anchor into this sketch and drag the
        # whole local-origin network onto it.
        projectedAnchor: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(
            self.sketch.project(anchorPoint).item(0))
        constraints: adsk.fusion.GeometricConstraints = self.sketch.geometricConstraints
        constraints.addCoincident(self.anchorPoint, projectedAnchor)
        # [SPUR-F-ROTATE-CONFIRM] very last action, after the entire constraint network exists.
        if angle != 0:
            self.angularDimension.parameter.value = angle

    def drawCircles(self):
        sketch = self.sketch
        circles: adsk.fusion.SketchCircles = sketch.sketchCurves.sketchCircles
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        size = tipRadius - rootRadius
        specs = [
            ('Root Circle', rootRadius, False),
            ('Tip Circle', tipRadius, True),
            ('Base Circle', baseRadius, True),
            ('Pitch Circle', pitchRadius, True),
        ]
        for name, radius, construction in specs:
            circle: adsk.fusion.SketchCircle = circles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = construction
            offCenterTextPoint = adsk.core.Point3D.create(radius * 0.7, radius * 0.7, 0)
            dims.addDiameterDimension(circle, offCenterTextPoint)
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput: adsk.fusion.SketchTextInput = sketch.sketchTexts.createInput2(text, size)
            alignment: adsk.core.HorizontalAlignments = adsk.core.HorizontalAlignments.CenterHorizontalAlignment
            textInput.setAsAlongPath(circle, True, alignment, 0)
            sketch.sketchTexts.add(textInput)
            if name == 'Root Circle':
                self.rootCircle = circle
            elif name == 'Tip Circle':
                self.tipCircle = circle

    def _flankPoints(self, angle: float):
        """Return (left, right) lists of (x, y) flank samples about the local origin,
        already rotated by angle."""
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(round(self.getParameterValue(PARAM_INVOLUTE_STEPS)))
        if steps < 2:
            raise Exception('Spur gear: Involute Steps must be at least 2, got {}'.format(steps))
        samples = []
        for i in range(0, steps):
            r = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                continue
            # Mirror across +X so the spiral matches a left flank.
            samples.append((p[0], -p[1]))
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None:
            raise Exception('Spur gear: pitch radius is below the base radius')
        px, py = pitchPoint
        rotateAngle = math.pi / (2 * toothNumber) - math.atan2(-py, px)
        cr = math.cos(rotateAngle)
        sr = math.sin(rotateAngle)
        left0 = [(x * cr - y * sr, x * sr + y * cr) for (x, y) in samples]
        right0 = [(x, -y) for (x, y) in left0]
        ca = math.cos(angle)
        sa = math.sin(angle)
        left = [(x * ca - y * sa, x * sa + y * ca) for (x, y) in left0]
        right = [(x * ca - y * sa, x * sa + y * ca) for (x, y) in right0]
        return left, right

    def drawTooth(self, angle=0):
        # NOTE: rotates by the angle argument supplied by draw(), never by self.toothAngle.
        sketch = self.sketch
        lines: adsk.fusion.SketchLines = sketch.sketchCurves.sketchLines
        arcs: adsk.fusion.SketchArcs = sketch.sketchCurves.sketchArcs
        splines: adsk.fusion.SketchFittedSplines = sketch.sketchCurves.sketchFittedSplines
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        step = tipRadius * 0.1

        left, right = self._flankPoints(angle)
        count = len(left)

        # Flank splines through the seeded points.
        leftFit: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        rightFit: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        for i in range(0, count):
            leftFit.add(adsk.core.Point3D.create(left[i][0], left[i][1], 0))
            rightFit.add(adsk.core.Point3D.create(right[i][0], right[i][1], 0))
        leftSpline: adsk.fusion.SketchFittedSpline = splines.add(leftFit)
        rightSpline: adsk.fusion.SketchFittedSpline = splines.add(rightFit)

        # Tooth-top point, then the arc centred on the local origin [SPUR-F-TOOTHTOP-ARC].
        toothTopPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipRadius * math.cos(angle), tipRadius * math.sin(angle), 0))
        constraints.addCoincident(toothTopPoint, self.tipCircle)
        rightFlankEndPoint: adsk.fusion.SketchPoint = rightSpline.endSketchPoint
        leftFlankEndPoint: adsk.fusion.SketchPoint = leftSpline.endSketchPoint
        arc: adsk.fusion.SketchArc = arcs.addByCenterStartEnd(
            self.anchorPoint, rightFlankEndPoint, leftFlankEndPoint)
        constraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)

        # Spine, +X reference and the angular pin [SPUR-F-SPINE].
        spine: adsk.fusion.SketchLine = lines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True
        referenceEnd: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipRadius, 0, 0))
        dims.addDistanceDimension(
            self.anchorPoint, referenceEnd, horizontal,
            self._mid(self.anchorPoint, referenceEnd, 0, -step))
        dims.addDistanceDimension(
            self.anchorPoint, referenceEnd, vertical,
            self._mid(self.anchorPoint, referenceEnd, step, 0))
        reference: adsk.fusion.SketchLine = lines.addByTwoPoints(self.anchorPoint, referenceEnd)
        reference.isConstruction = True
        bisectorR = tipRadius * 0.5
        bisectorTextPoint = adsk.core.Point3D.create(
            bisectorR * math.cos(angle / 2), bisectorR * math.sin(angle / 2), 0)
        self.angularDimension = dims.addAngularDimension(reference, spine, bisectorTextPoint)

        # Ribs [SPUR-F-RIBS]: one per fit-point index, endpoints included.
        acrossVertical = abs(math.cos(angle)) >= abs(math.sin(angle))
        acrossOrientation = vertical if acrossVertical else horizontal
        alongOrientation = horizontal if acrossVertical else vertical
        previousMidpoint: adsk.fusion.SketchPoint = self.anchorPoint
        for i in range(0, count):
            leftFitPoint: adsk.fusion.SketchPoint = leftSpline.fitPoints.item(i)
            rightFitPoint: adsk.fusion.SketchPoint = rightSpline.fitPoints.item(i)
            rib: adsk.fusion.SketchLine = lines.addByTwoPoints(leftFitPoint, rightFitPoint)
            rib.isConstruction = True
            acrossText = self._mid(leftFitPoint, rightFitPoint, step, step)
            dims.addDistanceDimension(leftFitPoint, rightFitPoint, acrossOrientation, acrossText)
            fitX = left[i][0]
            fitY = left[i][1]
            along = fitX * math.cos(angle) + fitY * math.sin(angle)
            midpoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
                adsk.core.Point3D.create(along * math.cos(angle), along * math.sin(angle), 0))
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            if i != count - 1:
                constraints.addPerpendicular(spine, rib)
            alongText = self._mid(previousMidpoint, midpoint, -step, -step)
            dims.addDistanceDimension(previousMidpoint, midpoint, alongOrientation, alongText)
            previousMidpoint = midpoint

        # Flank-to-root lines, or none when the profile is embedded [SPUR-F-FLANK-ROOT].
        firstRadius = math.hypot(left[0][0], left[0][1])
        embedded = firstRadius < rootRadius
        if not embedded:
            for flankIndex in range(0, 2):
                flankStartFitPoint: adsk.fusion.SketchPoint = (
                    leftSpline.fitPoints.item(0) if flankIndex == 0 else rightSpline.fitPoints.item(0))
                flank = left[0] if flankIndex == 0 else right[0]
                self._drawFlankToRoot(flankStartFitPoint, flank[0], flank[1], rootRadius / baseRadius)
        self.parent._lastToothEmbedded = embedded

    def _drawFlankToRoot(self, flankStartFitPoint: adsk.fusion.SketchPoint, flankX: float, flankY: float,
                         scale: float):
        sketch = self.sketch
        lines: adsk.fusion.SketchLines = sketch.sketchCurves.sketchLines
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        dx = flankX * scale
        dy = flankY * scale
        # Seed the root end at its exact position first, so each dimension captures its side.
        rootEndGeometry = adsk.core.Point3D.create(dx, dy, 0)
        rootLine: adsk.fusion.SketchLine = lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)
        rootEnd: adsk.fusion.SketchPoint = rootLine.startSketchPoint
        step = math.hypot(flankX, flankY) * 0.1
        horizontalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            self.anchorPoint, rootEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            self._mid(self.anchorPoint, rootEnd, 0, -step))
        verticalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            self.anchorPoint, rootEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            self._mid(self.anchorPoint, rootEnd, step, 0))
        # Magnitudes only: a negative value would flip the point to the other side.
        horizontalDim.parameter.value = abs(dx)
        verticalDim.parameter.value = abs(dy)

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projectedAnchor: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(
            sketch.project(anchorPoint).item(0))
        circles: adsk.fusion.SketchCircles = sketch.sketchCurves.sketchCircles
        circle: adsk.fusion.SketchCircle = circles.addByCenterRadius(projectedAnchor, diameter / 2)
        center: adsk.core.Point3D = projectedAnchor.geometry
        offCenterTextPoint = adsk.core.Point3D.create(
            center.x + diameter * 0.35, center.y + diameter * 0.35, 0)
        sketch.sketchDimensions.addDiameterDimension(circle, offCenterTextPoint)
        return circle


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.createdPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.toolsSketch = adsk.fusion.Sketch.cast(None)
        self.boreSketch = adsk.fusion.Sketch.cast(None)
        self._lastToothEmbedded = False

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def addExtraPrimaryParameters(self, inputs: adsk.core.CommandInputs):
        pass

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def generateName(self) -> str:
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            self.getParameter(PARAM_MODULE).expression,
            self.getParameter(PARAM_TOOTH_NUMBER).expression,
            self.getParameter(PARAM_THICKNESS).expression)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Read the selections first: creating the occurrence shifts the context and drops them.
        parents = get_selection(inputs, INPUT_ID_PARENT)
        planes = get_selection(inputs, INPUT_ID_PLANE)
        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(parents) < 1 or len(planes) < 1 or len(anchors) < 1:
            raise Exception('Spur gear: Parent Component, Target Plane and Anchor Point must all be selected')
        parent = parents[0]
        if isinstance(parent, adsk.fusion.Occurrence):
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent
        self.plane = planes[0]
        self.anchorPoint = anchors[0]
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
            'mm', 'Bore diameter')
        self.addParameter(
            PARAM_THICKNESS, get_value(inputs, INPUT_ID_THICKNESS, 'mm'),
            'mm', 'Thickness of the gear')
        self.addParameter(
            PARAM_CHAMFER_TOOTH, get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm'),
            'mm', 'Chamfer distance applied to the teeth')
        self.addParameter(
            PARAM_SKETCH_ONLY, adsk.core.ValueInput.createByReal(1 if get_boolean(inputs, INPUT_ID_SKETCH_ONLY) else 0),
            '', 'Generate sketches only')
        self.addExtraPrimaryParameters(inputs)
        self.registerDerivedParameters()

    def registerDerivedParameters(self):
        expression = '{} * {}'.format(
            self.parameterName(PARAM_MODULE),
            self.parameterName(PARAM_TOOTH_NUMBER),
        )
        self.addParameter(
            PARAM_PITCH_DIAMETER, adsk.core.ValueInput.createByString(expression),
            'mm', 'Pitch circle diameter')
        expression = '{} / 2'.format(
            self.parameterName(PARAM_PITCH_DIAMETER),
        )
        self.addParameter(
            PARAM_PITCH_RADIUS, adsk.core.ValueInput.createByString(expression),
            'mm', 'Pitch circle radius')
        expression = '{} * cos({})'.format(
            self.parameterName(PARAM_PITCH_DIAMETER),
            self.parameterName(PARAM_PRESSURE_ANGLE),
        )
        self.addParameter(
            PARAM_BASE_DIAMETER, adsk.core.ValueInput.createByString(expression),
            'mm', 'Base circle diameter')
        expression = '{} / 2'.format(
            self.parameterName(PARAM_BASE_DIAMETER),
        )
        self.addParameter(
            PARAM_BASE_RADIUS, adsk.core.ValueInput.createByString(expression),
            'mm', 'Base circle radius')
        expression = '{} - 2.5 * {}'.format(
            self.parameterName(PARAM_PITCH_DIAMETER),
            self.parameterName(PARAM_MODULE),
        )
        self.addParameter(
            PARAM_ROOT_DIAMETER, adsk.core.ValueInput.createByString(expression),
            'mm', 'Root circle diameter')
        expression = '{} / 2'.format(
            self.parameterName(PARAM_ROOT_DIAMETER),
        )
        self.addParameter(
            PARAM_ROOT_RADIUS, adsk.core.ValueInput.createByString(expression),
            'mm', 'Root circle radius')
        expression = '{} + 2 * {}'.format(
            self.parameterName(PARAM_PITCH_DIAMETER),
            self.parameterName(PARAM_MODULE),
        )
        self.addParameter(
            PARAM_TIP_DIAMETER, adsk.core.ValueInput.createByString(expression),
            'mm', 'Tip circle diameter')
        expression = '{} / 2'.format(
            self.parameterName(PARAM_TIP_DIAMETER),
        )
        self.addParameter(
            PARAM_TIP_RADIUS, adsk.core.ValueInput.createByString(expression),
            'mm', 'Tip circle radius')
        expression = '15'
        self.addParameter(
            PARAM_INVOLUTE_STEPS, adsk.core.ValueInput.createByString(expression),
            '', 'Number of points sampled along each involute flank')
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngle = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE, adsk.core.ValueInput.createByReal(toothSpaceAngle),
            '', 'Angular width of the tooth space at the root circle')
        expression = '{} * {}'.format(
            self.parameterName(PARAM_ROOT_RADIUS),
            self.parameterName(PARAM_TOOTH_SPACE_ANGLE),
        )
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC, adsk.core.ValueInput.createByString(expression),
            'mm', 'Arc length of the tooth space at the root circle')
        expression = '0.9'
        self.addParameter(
            PARAM_FILLET_CLEARANCE, adsk.core.ValueInput.createByString(expression),
            '', 'Clearance factor applied to the root fillet radius')
        expression = '({} / 2) * {} * {}'.format(
            self.parameterName(PARAM_TOOTH_SPACE_ARC),
            self.parameterName(PARAM_FILLET_CLEARANCE),
            self.filletHelixFactorExpression(),
        )
        self.addParameter(
            PARAM_FILLET_RADIUS, adsk.core.ValueInput.createByString(expression),
            'mm', 'Radius of the root fillets')

    def _normalizePlane(self):
        selected = self.plane
        if selected.objectType == adsk.fusion.ConstructionPlane.classType():
            return
        selectedPlane = selected
        planes: adsk.fusion.ConstructionPlanes = self.getComponent().constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
        offset = adsk.core.ValueInput.createByReal(0)
        planeInput.setByOffset(selectedPlane, offset)
        created: adsk.fusion.ConstructionPlane = planes.add(planeInput)
        created.name = 'Target Plane'
        self.createdPlane = created
        self.plane = created

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()
        self._normalizePlane()
        ctx = self.newContext()
        ctx.plane = self.plane
        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx: SpurGearGenerationContext):
        toolsSketch: adsk.fusion.Sketch = self.createSketchObject('Tools', ctx.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch
        ctx.anchorPoint = adsk.fusion.SketchPoint.cast(toolsSketch.project(self.anchorPoint).item(0))
        if not toolsSketch.isFullyConstrained:
            raise Exception('Sketch "{}" is not fully constrained'.format(toolsSketch.name))

        thickness = self.getParameter(PARAM_THICKNESS).value
        planes: adsk.fusion.ConstructionPlanes = self.getComponent().constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
        offset = adsk.core.ValueInput.createByReal(thickness)
        planeInput.setByOffset(ctx.plane, offset)
        endPlane: adsk.fusion.ConstructionPlane = planes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    def buildMainGearBody(self, ctx: SpurGearGenerationContext):
        self.buildSketches(ctx)
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return
        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)
        self.createFillets(ctx)

    def buildSketches(self, ctx: SpurGearGenerationContext):
        sketch: adsk.fusion.Sketch = self.createSketchObject('Gear Profile', ctx.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGen.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded
        # Labelled sketches never reliably read fully constrained; log, do not raise.
        futil.log('Gear Profile isFullyConstrained={}'.format(sketch.isFullyConstrained))

    def _extrudeToEndPlane(self, ctx: SpurGearGenerationContext, profile: adsk.fusion.Profile
                           ) -> adsk.fusion.ExtrudeFeature:
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = extrudes.createInput(profile, operation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        return extrudes.add(extrudeInput)

    def buildTooth(self, ctx: SpurGearGenerationContext):
        lines = 0 if ctx.toothProfileIsEmbedded else 2
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=lines)
        extrude = self._extrudeToEndPlane(ctx, profile)
        extrude.name = 'Extrude tooth'
        if extrude.bodies.count < 1:
            raise Exception('Spur gear: tooth extrude produced no body')
        ctx.toothBody = adsk.fusion.BRepBody.cast(extrude.bodies.item(0))
        ctx.toothBody.name = 'Tooth'

    def buildBody(self, ctx: SpurGearGenerationContext):
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=0, arcs=2, lines=0)
        extrude = self._extrudeToEndPlane(ctx, profile)
        extrude.name = 'Extrude body'
        if extrude.bodies.count < 1:
            raise Exception('Spur gear: body extrude produced no body')
        body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(extrude.bodies.item(0))
        body.name = 'Gear Body'
        ctx.gearBody = body

        sketchPlane: adsk.core.Plane = ctx.plane.geometry
        cylindricalFace = adsk.fusion.BRepFace.cast(None)
        farFace = adsk.fusion.BRepFace.cast(None)
        for i in range(0, body.faces.count):
            face: adsk.fusion.BRepFace = body.faces.item(i)
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if cylindricalFace is None:
                    cylindricalFace = face
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                facePlane: adsk.core.Plane = adsk.core.Plane.cast(face.geometry)
                if sketchPlane.isParallelToPlane(facePlane) and not sketchPlane.isCoPlanarTo(facePlane):
                    if farFace is None:
                        farFace = face
        if cylindricalFace is None:
            raise Exception('Spur gear: gear body has no cylindrical face for the Gear Center axis')
        if farFace is None:
            raise Exception('Spur gear: gear body has no far end-cap face')

        axes: adsk.fusion.ConstructionAxes = self.getComponent().constructionAxes
        axisInput: adsk.fusion.ConstructionAxisInput = axes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        axis: adsk.fusion.ConstructionAxis = axes.add(axisInput)
        axis.name = 'Gear Center'
        axis.isLightBulbOn = False
        ctx.centerAxis = axis
        ctx.extrusionExtent = farFace

    def patternTeeth(self, ctx: SpurGearGenerationContext):
        component = self.getComponent()
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        entities: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        patterns: adsk.fusion.CircularPatternFeatures = component.features.circularPatternFeatures
        patternInput: adsk.fusion.CircularPatternFeatureInput = patterns.createInput(entities, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString("360 deg")
        patternInput.isSymmetric = False
        pattern: adsk.fusion.CircularPatternFeature = patterns.add(patternInput)
        if pattern.bodies.count < 1:
            raise Exception('Spur gear: tooth pattern produced {} bodies, expected at least 1'.format(
                pattern.bodies.count))

        # pattern.bodies already includes the seed tooth.
        tools: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        for i in range(0, pattern.bodies.count):
            patternBody: adsk.fusion.BRepBody = pattern.bodies.item(i)
            tools.add(patternBody)
        combines: adsk.fusion.CombineFeatures = component.features.combineFeatures
        combineInput: adsk.fusion.CombineFeatureInput = combines.createInput(ctx.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

    def createFillets(self, ctx: SpurGearGenerationContext):
        radius = self.getParameter(PARAM_FILLET_RADIUS).value
        if radius <= 0:
            return
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(ctx.plane)
        if axisNormal is None:
            raise Exception('Spur gear: cannot determine the target plane normal')
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = set()
        body = ctx.gearBody
        for i in range(0, body.faces.count):
            face: adsk.fusion.BRepFace = body.faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            cylinder: adsk.core.Cylinder = adsk.core.Cylinder.cast(face.geometry)
            if abs(cylinder.radius - rootRadius) > 0.0001:
                continue
            for j in range(0, face.edges.count):
                edge: adsk.fusion.BRepEdge = face.edges.item(j)
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                line: adsk.core.Line3D = adsk.core.Line3D.cast(edge.geometry)
                direction: adsk.core.Vector3D = line.startPoint.vectorTo(line.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) >= 0.01:
                    continue
                if edge.tempId in seen:
                    continue
                seen.add(edge.tempId)
                edges.add(edge)
        if edges.count == 0:
            return
        fillets: adsk.fusion.FilletFeatures = self.getComponent().features.filletFeatures
        filletInput: adsk.fusion.FilletFeatureInput = fillets.createInput()
        radiusValue = adsk.core.ValueInput.createByReal(radius)
        filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)
        fillets.add(filletInput)

    def buildBore(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        boreSketch: adsk.fusion.Sketch = self.createSketchObject('Bore Profile', ctx.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch
        toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        projection: adsk.core.ObjectCollection = adsk.core.ObjectCollection.cast(
            boreSketch.project(ctx.anchorPoint))
        projectedAnchor: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projection.item(0))
        toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        constraints: adsk.fusion.GeometricConstraints = boreSketch.geometricConstraints
        constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)
        if not boreSketch.isFullyConstrained:
            raise Exception('Sketch "{}" is not fully constrained'.format(boreSketch.name))
        if boreSketch.profiles.count != 1:
            raise Exception('Spur gear: Bore Profile has {} profiles, expected 1'.format(
                boreSketch.profiles.count))
        profile: adsk.fusion.Profile = boreSketch.profiles.item(0)

        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    def chamferTeeth(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance <= 0:
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        sketchPlane: adsk.core.Plane = ctx.plane.geometry
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = set()
        capFaces = 0
        body = ctx.gearBody
        for i in range(0, body.faces.count):
            face: adsk.fusion.BRepFace = body.faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            facePlane: adsk.core.Plane = adsk.core.Plane.cast(face.geometry)
            if not sketchPlane.isParallelToPlane(facePlane):
                continue
            capFaces += 1
            for j in range(0, face.edges.count):
                edge: adsk.fusion.BRepEdge = face.edges.item(j)
                if edge.tempId in seen:
                    continue
                if boreDiameter > 0 and edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                    circle: adsk.core.Circle3D = adsk.core.Circle3D.cast(edge.geometry)
                    if abs(circle.radius - boreDiameter / 2) <= 0.001:
                        continue
                seen.add(edge.tempId)
                edges.add(edge)
        if capFaces == 0:
            raise Exception('Spur gear: chamfer found no end-cap face')
        if edges.count == 0:
            raise Exception('Spur gear: chamfer found no edge on the end-cap faces')
        chamfers: adsk.fusion.ChamferFeatures = self.getComponent().features.chamferFeatures
        chamferInput: adsk.fusion.ChamferFeatureInput = chamfers.createInput2()
        distance = adsk.core.ValueInput.createByReal(chamferDistance)
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)
        chamfers.add(chamferInput)

    def cleanup(self, ctx: SpurGearGenerationContext):
        # Construction planes/axes are hidden with isLightBulbOn, sketches with isVisible.
        if self.createdPlane is not None and self.createdPlane.isValid:
            self.createdPlane.isLightBulbOn = False
        if ctx.extrusionEndPlane is not None and ctx.extrusionEndPlane.isValid:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None and ctx.centerAxis.isValid:
            ctx.centerAxis.isLightBulbOn = False
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        if self.toolsSketch is not None and self.toolsSketch.isValid:
            self.toolsSketch.isVisible = False
        if ctx.gearProfileSketch is not None and ctx.gearProfileSketch.isValid:
            ctx.gearProfileSketch.isVisible = False
        if self.boreSketch is not None and self.boreSketch.isValid:
            self.boreSketch.isVisible = False
