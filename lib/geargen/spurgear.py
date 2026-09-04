import math
import adsk.core, adsk.fusion
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts
from .misc import to_cm, get_design


# -----------------------------------------------------------------------------------------
# Dialog input ids and registered user-parameter names. [SPUR-EXPORTED-CONSTANTS]: renaming
# any of these is a breaking change — helicalgear.py and herringbonegear.py import
# PARAM_MODULE, PARAM_TOOTH_NUMBER and PARAM_THICKNESS from this module by name.
# -----------------------------------------------------------------------------------------
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


# -----------------------------------------------------------------------------------------
# S1 — dialog inputs.
# -----------------------------------------------------------------------------------------
class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane — first, so Fusion's auto-focus-first-selection-input
        #    behaviour opens the dialog on it ([PB-AUTOFOCUS-FIRST]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the gear on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point the gear is centered on')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Module
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 4. Tooth Number
        inputs.addValueInput(INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))

        # 5. Pressure Angle
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))

        # 6. Bore Diameter — a string input so it accepts expressions.
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

        # 7. Thickness
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))

        # 8. Apply chamfer to teeth
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))

        # 9. Generate sketches, but do not build body — a checkbox, default false.
        inputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True)

        # 10. Parent Component — last, so a subclass's own extra input still lands
        #     below it ([SPUR-SUBCLASS-INPUT]).
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the component to build the gear in')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)


# -----------------------------------------------------------------------------------------
# S3 — the generation context.
# -----------------------------------------------------------------------------------------
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


# -----------------------------------------------------------------------------------------
# S7 — the involute tooth drawer. Borrowed unmodified by bevelgear.py via a
# spurproxy.VirtualSpurProxy ([PB-PRECOMPUTED-MODE]): every parameter read below stays
# inside the key set that proxy serves.
# -----------------------------------------------------------------------------------------
class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        # [SPUR-F-LOCAL-ORIGIN]: a fresh SketchPoint, NOT sketch.originPoint — every
        # piece of tooth geometry hangs off this one, and S7's anchoring drags it (and
        # everything sharing it) onto the user's anchor as a unit.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)  # the curve parameter is tan(alpha), NOT inv(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return (x, y)

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)

        # Anchor the sketch, inside draw() itself — helical/herringbone call draw()
        # directly and rely on this one call to anchor their twisted profile too.
        projectedAnchor = self.sketch.project(anchorPoint).item(0)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)

        # Confirm the rotation as the very last action, after the whole constraint
        # network exists, and only when angle != 0 ([SPUR-F-ROTATE-CONFIRM]).
        if angle != 0:
            self._spineAngularDimension.parameter.value = angle

    def drawCircles(self):
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        size = tipRadius - rootRadius

        self._rootCircle = self._addLabeledCircle('Root Circle', rootRadius, size, False)
        self._tipCircle = self._addLabeledCircle('Tip Circle', tipRadius, size, True)
        self._baseCircle = self._addLabeledCircle('Base Circle', baseRadius, size, True)
        self._pitchCircle = self._addLabeledCircle('Pitch Circle', pitchRadius, size, True)

    def _addLabeledCircle(self, name, radius, size, construction):
        sketch = self.sketch
        localOrigin = self.anchorPoint

        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)
        circle.isConstruction = construction
        textPoint = adsk.core.Point3D.create(radius, 0, 0)
        sketch.sketchDimensions.addDiameterDimension(circle, textPoint)

        label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
        textInput = sketch.sketchTexts.createInput2(label, size)
        textInput.setAsAlongPath(
            circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(textInput)
        return circle

    def drawTooth(self, angle):
        sketch = self.sketch
        localOrigin = self.anchorPoint

        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)

        # 1. Sample the flank from the base circle out to the tip circle.
        samples = []
        for i in range(steps):
            r = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            point = self.calculateInvolutePoint(baseRadius, r)
            if point is None:
                continue
            samples.append(point)

        # 2. Mirror across +X.
        mirrored = [(x, -y) for (x, y) in samples]

        # 3. Rotate so the tooth is symmetric about +X — the pitch crossing is
        #    computed analytically, not interpolated between samples.
        (px, py) = self.calculateInvolutePoint(baseRadius, pitchRadius)
        rotateAngle = math.pi / (2 * toothNumber) - math.atan2(-py, px)

        def rotate(point, ang):
            x, y = point
            c = math.cos(ang)
            s = math.sin(ang)
            return (x * c - y * s, x * s + y * c)

        # 4. Rotate the mirrored samples by rotateAngle for the left flank, mirror
        #    that across the X axis for the right flank, then rotate BOTH flanks by
        #    the angle argument.
        leftLocal = [rotate(point, rotateAngle) for point in mirrored]
        rightLocal = [(x, -y) for (x, y) in leftLocal]
        left = [rotate(point, angle) for point in leftLocal]
        right = [rotate(point, angle) for point in rightLocal]

        # 5. Draw each flank as a SketchFittedSpline through its point collection.
        def toPointCollection(points):
            collection = adsk.core.ObjectCollection.create()
            for (x, y) in points:
                collection.add(adsk.core.Point3D.create(x, y, 0))
            return collection

        leftSpline = sketch.sketchCurves.sketchFittedSplines.add(toPointCollection(left))
        rightSpline = sketch.sketchCurves.sketchFittedSplines.add(toPointCollection(right))
        rightFlankEndPoint = rightSpline.endSketchPoint
        leftFlankEndPoint = leftSpline.endSketchPoint

        # ---- Tooth-top arc [SPUR-F-TOOTHTOP-ARC] ----
        toothTopX = tipRadius * math.cos(angle)
        toothTopY = tipRadius * math.sin(angle)
        toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(toothTopX, toothTopY, 0))
        sketch.geometricConstraints.addCoincident(toothTopPoint, self._tipCircle)

        arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            localOrigin, rightFlankEndPoint, leftFlankEndPoint)
        # addByCenterStartEnd shares the start/end points but COPIES the centre —
        # tie it back explicitly ([PB-SHARE-XOR-COINCIDENT]).
        sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, localOrigin)

        # ---- Spine, +X reference and angular pin [SPUR-F-SPINE] ----
        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)
        spine.isConstruction = True

        referenceEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        hDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, referenceEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius / 2, tipRadius * 0.1, 0))
        hDim.parameter.value = tipRadius
        vDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, referenceEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(tipRadius * 0.9, tipRadius * 0.1, 0))
        vDim.parameter.value = 0

        reference = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, referenceEnd)
        reference.isConstruction = True

        textR = baseRadius * 0.5
        angleTextPoint = adsk.core.Point3D.create(
            textR * math.cos(angle / 2), textR * math.sin(angle / 2), 0)
        self._spineAngularDimension = sketch.sketchDimensions.addAngularDimension(
            reference, spine, angleTextPoint)

        # ---- Ribs [SPUR-F-RIBS] ----
        n = len(left)
        cosA = math.cos(angle)
        sinA = math.sin(angle)
        acrossIsVertical = abs(cosA) >= abs(sinA)

        previousMidpoint = localOrigin
        previousX, previousY = 0.0, 0.0
        for i in range(n):
            leftFit = leftSpline.fitPoints.item(i)
            rightFit = rightSpline.fitPoints.item(i)
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True

            ribTextPoint = adsk.core.Point3D.create(
                (left[i][0] + right[i][0]) / 2 + 0.05, (left[i][1] + right[i][1]) / 2, 0)
            if acrossIsVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, ribTextPoint)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    leftFit, rightFit,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, ribTextPoint)

            fitX, fitY = left[i]
            t = fitX * cosA + fitY * sinA
            midX, midY = t * cosA, t * sinA
            midPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(midX, midY, 0))

            sketch.geometricConstraints.addCoincident(midPoint, spine)
            sketch.geometricConstraints.addMidPoint(midPoint, rib)
            if i != n - 1:
                sketch.geometricConstraints.addPerpendicular(spine, rib)

            chainTextPoint = adsk.core.Point3D.create(
                (previousX + midX) / 2, (previousY + midY) / 2 + 0.05, 0)
            if acrossIsVertical:
                sketch.sketchDimensions.addDistanceDimension(
                    previousMidpoint, midPoint,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, chainTextPoint)
            else:
                sketch.sketchDimensions.addDistanceDimension(
                    previousMidpoint, midPoint,
                    adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, chainTextPoint)

            previousMidpoint = midPoint
            previousX, previousY = midX, midY

        # ---- Flank-to-root lines and the embedded test [SPUR-F-FLANK-ROOT] ----
        leftFirstX, leftFirstY = left[0]
        firstRadius = math.hypot(leftFirstX, leftFirstY)
        embedded = firstRadius < rootRadius  # strict '<' — exact equality is non-embedded
        self.parent._lastToothEmbedded = embedded

        if not embedded:
            def footXY(seed):
                x, y = seed
                magnitude = math.hypot(x, y)
                return rootRadius * x / magnitude, rootRadius * y / magnitude

            leftRootX, leftRootY = footXY(left[0])
            self._drawFlankToRoot(leftRootX, leftRootY, leftSpline.fitPoints.item(0))
            rightRootX, rightRootY = footXY(right[0])
            self._drawFlankToRoot(rightRootX, rightRootY, rightSpline.fitPoints.item(0))

    def _drawFlankToRoot(self, rootEndX, rootEndY, flankStartPoint):
        sketch = self.sketch
        localOrigin = self.anchorPoint

        rootEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(rootEndX, rootEndY, 0))
        sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flankStartPoint)

        hDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, rootEnd,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(rootEndX / 2, rootEndY + 0.05, 0))
        hDim.parameter.value = abs(rootEndX)

        vDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, rootEnd,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(rootEndX + 0.05, rootEndY / 2, 0))
        vDim.parameter.value = abs(rootEndY)

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        self.projectedAnchor = projectedAnchor

        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, diameter / 2)
        centre = projectedAnchor.geometry
        textPoint = adsk.core.Point3D.create(centre.x + diameter / 2, centre.y, 0)
        sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
        return circle


# -----------------------------------------------------------------------------------------
# The orchestrator.
# -----------------------------------------------------------------------------------------
class SpurGearGenerator(Generator):
    def __init__(self, design):
        super().__init__(design)
        self._lastToothEmbedded = False
        self.toolsSketch = None
        self.boreSketch = None
        self._createdTargetPlane = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def addExtraPrimaryParameters(self, inputs):
        # [SPUR-EXTRA-PARAMS] — a no-op hook on the spur base; a subclass registers
        # its own primary parameter here, between the input-sourced parameters and
        # the derived ones.
        pass

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    # -- S2 --------------------------------------------------------------------------
    def processInputs(self, inputs):
        # Order is load-bearing ([PB-SELECTION-STASH]): resolve every selection
        # before anything creates the gear occurrence.
        parentEntities = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentEntities) != 1:
            raise Exception(
                'Parent Component: expected exactly one selection, got {}'.format(len(parentEntities)))
        parentEntity = parentEntities[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        elif parentEntity.objectType == adsk.fusion.Component.classType():
            self.parentComponent = parentEntity
        else:
            raise Exception(
                'Parent Component: unexpected selection type {}'.format(parentEntity.objectType))

        planeEntities = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeEntities) != 1:
            raise Exception(
                'Target Plane: expected exactly one selection, got {}'.format(len(planeEntities)))
        self.plane = planeEntities[0]

        anchorEntities = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchorEntities) != 1:
            raise Exception(
                'Anchor Point: expected exactly one selection, got {}'.format(len(anchorEntities)))
        self.anchorPoint = anchorEntities[0]

        # Input-sourced parameters ([PB-INPUT-READ], [PB-GET-VALUE-CONTRACT]). Comments
        # are the exact strings S2's table gives, shown to the user in the parameter
        # table's Comment column.
        self.addParameter(
            PARAM_MODULE, get_value(inputs, INPUT_ID_MODULE, ''), '', comment='Module of the gear')
        self.addParameter(
            PARAM_TOOTH_NUMBER, get_value(inputs, INPUT_ID_TOOTH_NUMBER, ''), '',
            comment='Number of teeth')
        self.addParameter(
            PARAM_PRESSURE_ANGLE, get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad'), 'rad',
            comment='Pressure angle')
        self.addParameter(
            PARAM_BORE_DIAMETER, get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm'), 'mm',
            comment='Bore diameter')
        self.addParameter(
            PARAM_THICKNESS, get_value(inputs, INPUT_ID_THICKNESS, 'mm'), 'mm',
            comment='Thickness of the gear')
        self.addParameter(
            PARAM_CHAMFER_TOOTH, get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm'), 'mm',
            comment='Chamfer distance applied to the teeth')

        sketchOnly = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY, adsk.core.ValueInput.createByReal(1 if sketchOnly else 0), '',
            comment='Generate sketches only')

        self.addExtraPrimaryParameters(inputs)

        self.registerDerivedParameters()

    def registerDerivedParameters(self):
        p = self.parameterName

        self.addParameter(
            PARAM_PITCH_DIAMETER,
            adsk.core.ValueInput.createByString('{} * {}'.format(p(PARAM_MODULE), p(PARAM_TOOTH_NUMBER))),
            'mm', comment='Pitch circle diameter')
        self.addParameter(
            PARAM_PITCH_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(p(PARAM_PITCH_DIAMETER))),
            'mm', comment='Pitch circle radius')
        self.addParameter(
            PARAM_BASE_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} * cos({})'.format(p(PARAM_PITCH_DIAMETER), p(PARAM_PRESSURE_ANGLE))),
            'mm', comment='Base circle diameter')
        self.addParameter(
            PARAM_BASE_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(p(PARAM_BASE_DIAMETER))),
            'mm', comment='Base circle radius')
        self.addParameter(
            PARAM_ROOT_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} - 2.5 * {}'.format(p(PARAM_PITCH_DIAMETER), p(PARAM_MODULE))),
            'mm', comment='Root circle diameter')
        self.addParameter(
            PARAM_ROOT_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(p(PARAM_ROOT_DIAMETER))),
            'mm', comment='Root circle radius')
        self.addParameter(
            PARAM_TIP_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} + 2 * {}'.format(p(PARAM_PITCH_DIAMETER), p(PARAM_MODULE))),
            'mm', comment='Tip circle diameter')
        self.addParameter(
            PARAM_TIP_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(p(PARAM_TIP_DIAMETER))),
            'mm', comment='Tip circle radius')
        self.addParameter(
            PARAM_INVOLUTE_STEPS, adsk.core.ValueInput.createByString('15'), '',
            comment='Number of points sampled along each involute flank')

        # Pre-computed in Python: Fusion's expression engine refuses to subtract a
        # radian-valued Pressure Angle from the unitless output of tan().
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngle = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE, adsk.core.ValueInput.createByReal(toothSpaceAngle), '',
            comment='Angular width of the tooth space at the root circle')

        self.addParameter(
            PARAM_TOOTH_SPACE_ARC,
            adsk.core.ValueInput.createByString(
                '{} * {}'.format(p(PARAM_ROOT_RADIUS), p(PARAM_TOOTH_SPACE_ANGLE))),
            'mm', comment='Arc length of the tooth space at the root circle')

        self.addParameter(
            PARAM_FILLET_CLEARANCE, adsk.core.ValueInput.createByString('0.9'), '',
            comment='Clearance factor applied to the root fillet radius')

        factor = self.filletHelixFactorExpression()
        self.addParameter(
            PARAM_FILLET_RADIUS,
            adsk.core.ValueInput.createByString(
                '({} / 2) * {} * {}'.format(p(PARAM_TOOTH_SPACE_ARC), p(PARAM_FILLET_CLEARANCE), factor)),
            'mm', comment='Radius of the root fillets')

    # -- S3/S4 -------------------------------------------------------------------------
    def generate(self, inputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # S4 — normalize the Target Plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = self.getComponent().constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = self.getComponent().constructionPlanes.add(planeInput)
            self._createdTargetPlane = self.plane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)

        self.buildMainGearBody(ctx)

        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    # -- S5/S6 -------------------------------------------------------------------------
    def prepareTools(self, ctx):
        self.toolsSketch = self.createSketchObject('Tools', plane=self.plane)
        self.toolsSketch.isVisible = True
        ctx.anchorPoint = self.toolsSketch.project(self.anchorPoint).item(0)

        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = self.getComponent().constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        ctx.extrusionEndPlane = self.getComponent().constructionPlanes.add(planeInput)
        ctx.extrusionEndPlane.name = 'Extrusion End Plane'

    # -- S7/S8 -------------------------------------------------------------------------
    def buildMainGearBody(self, ctx):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)
        self.createFillets(ctx)

    def buildSketches(self, ctx):
        ctx.gearProfileSketch = self.createSketchObject('Gear Profile', plane=self.plane)
        ctx.gearProfileSketch.isVisible = True

        toothGen = SpurGearInvoluteToothDesignGenerator(ctx.gearProfileSketch, self)
        toothGen.draw(ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    # -- S9 --------------------------------------------------------------------------
    def buildTooth(self, ctx):
        toothProfile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2)

        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude tooth'

        ctx.toothBody = extrude.bodies.item(0)

    # -- S10 -------------------------------------------------------------------------
    def buildBody(self, ctx):
        bodyProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)

        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            bodyProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude body'

        body = extrude.bodies.item(0)
        body.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        cylindricalFace = None
        extentFace = None
        for face in body.faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if cylindricalFace is None:
                    cylindricalFace = face
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry):
                    extentFace = face

        if cylindricalFace is None:
            raise Exception(
                'buildBody: {}: no cylindrical face found to build the Gear Center axis from'.format(
                    self.getComponent().name))
        if extentFace is None:
            raise Exception(
                'buildBody: {}: no far end-cap face found parallel to but not coplanar with the '
                'sketch plane'.format(self.getComponent().name))

        axisInput = self.getComponent().constructionAxes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        ctx.centerAxis = self.getComponent().constructionAxes.add(axisInput)
        ctx.centerAxis.name = 'Gear Center'
        ctx.centerAxis.isLightBulbOn = False

        ctx.extrusionExtent = extentFace
        ctx.gearBody = body

    # -- S11/S12 -----------------------------------------------------------------------
    def patternTeeth(self, ctx):
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(ctx.toothBody)
        circularPatternFeatures = self.getComponent().features.circularPatternFeatures
        patternInput = circularPatternFeatures.createInput(bodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = circularPatternFeatures.add(patternInput)

        # [PB-PATTERN-BODIES]: pattern.bodies already holds the seed plus copies —
        # copy it into a fresh ObjectCollection, combineFeatures rejects BRepBodies.
        tools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(i))

        combineFeatures = self.getComponent().features.combineFeatures
        combineInput = combineFeatures.createInput(ctx.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineFeatures.add(combineInput)

    # -- S13 -------------------------------------------------------------------------
    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)

        edges = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            geometry = face.geometry
            if geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(geometry.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                curve = edge.geometry
                if curve.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = curve.startPoint.vectorTo(curve.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            # [PB-EMPTY-RESULT]: an empty edge set must never reach filletFeatures.add.
            return

        filletFeatures = self.getComponent().features.filletFeatures
        filletInput = filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        filletFeatures.add(filletInput)

    # -- S14/S15 -----------------------------------------------------------------------
    def buildBore(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        self.boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)
        self.boreSketch.isVisible = True

        toothGen = SpurGearInvoluteToothDesignGenerator(self.boreSketch, self)
        toothGen.drawBore(ctx.anchorPoint, boreDiameter)

        # [SPUR-F-LOCAL-ORIGIN] / [PB-CIRCLE-CENTER]: ground the stray local-origin
        # point on the same projection drawBore already made — never on the
        # sketch's own origin point.
        self.boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, toothGen.projectedAnchor)

        boreProfile = self.boreSketch.profiles.item(0)
        extrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudeFeatures.add(extrudeInput)

    # -- S16 -------------------------------------------------------------------------
    def chamferTeeth(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferTooth = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferTooth <= 0:
            return

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value

        edges = adsk.core.ObjectCollection.create()
        seenEdgeIds = []
        foundEndCap = False
        for face in ctx.gearBody.faces:
            geometry = face.geometry
            if geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            if not sketchPlane.isParallelToPlane(geometry):
                continue
            foundEndCap = True
            for edge in face.edges:
                if edge.tempId in seenEdgeIds:
                    continue
                seenEdgeIds.append(edge.tempId)

                curve = edge.geometry
                if (curve.curveType == adsk.core.Curve3DTypes.Circle3DCurveType and boreDiameter > 0
                        and abs(curve.radius - boreDiameter / 2) < 0.001):
                    continue
                edges.add(edge)

        if not foundEndCap:
            raise Exception(
                'chamferTeeth: {}: no end-cap face found parallel to the sketch plane'.format(
                    self.getComponent().name))
        if edges.count == 0:
            raise Exception(
                'chamferTeeth: {}: no chamfer edges found on the end-cap faces'.format(
                    self.getComponent().name))

        chamferFeatures = self.getComponent().features.chamferFeatures
        chamferInput = chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferTooth), False)
        chamferFeatures.add(chamferInput)

    # -- S17 -------------------------------------------------------------------------
    def cleanup(self, ctx):
        # Construction planes/axes: always hidden, in both modes ([PB-HIDE-AFTER-USE]).
        if self._createdTargetPlane is not None:
            self._createdTargetPlane.isLightBulbOn = False
        ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False

        # Sketches: hidden only on the full-build path — Generate-Sketches-Only
        # leaves the Tools and Gear Profile sketches visible for inspection.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return

        self.toolsSketch.isVisible = False
        ctx.gearProfileSketch.isVisible = False
        if self.boreSketch is not None:
            self.boreSketch.isVisible = False
