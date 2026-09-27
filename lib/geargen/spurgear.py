import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .misc import to_cm, get_design
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
    def configure(cls, cmd):
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


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False
        self._normalizedPlane = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def addExtraPrimaryParameters(self, inputs):
        pass

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def processInputs(self, inputs):
        parent = get_selection(inputs, INPUT_ID_PARENT)[0]
        self.plane = get_selection(inputs, INPUT_ID_PLANE)[0]
        self.anchorPoint = get_selection(inputs, INPUT_ID_ANCHOR_POINT)[0]
        if isinstance(parent, adsk.fusion.Occurrence):
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent
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

    def generate(self, inputs):
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()
        if not isinstance(self.plane, adsk.fusion.ConstructionPlane):
            selectedPlane = self.plane
            planes = component.constructionPlanes
            planeInput = planes.createInput()
            zero = adsk.core.ValueInput.createByReal(0)
            planeInput.setByOffset(selectedPlane, zero)
            self.plane = planes.add(planeInput)
            self._normalizedPlane = self.plane
        ctx = self.newContext()
        ctx.plane = self.plane
        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx):
        sketch = self.createSketchObject('Tools', ctx.plane)
        sketch.isVisible = True
        self.toolsSketch = sketch
        ctx.anchorPoint = sketch.project(self.anchorPoint).item(0)
        if not sketch.isFullyConstrained:
            raise RuntimeError('Spur gear: Tools sketch is not fully constrained')
        thickness = self.getParameter(PARAM_THICKNESS).value
        planes = self.getComponent().constructionPlanes
        planeInput = planes.createInput()
        thicknessValue = adsk.core.ValueInput.createByReal(thickness)
        planeInput.setByOffset(ctx.plane, thicknessValue)
        ctx.extrusionEndPlane = planes.add(planeInput)
        ctx.extrusionEndPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane.isLightBulbOn = True

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
        sketch = self.createSketchObject('Gear Profile', ctx.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGen.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    def buildTooth(self, ctx):
        sketch = ctx.gearProfileSketch
        profile = find_profile_by_curve_counts(
            sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

    def buildBody(self, ctx):
        sketch = ctx.gearProfileSketch
        profile = find_profile_by_curve_counts(sketch, arcs=2)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'
        ctx.gearBody = extrude.bodies.item(0)
        ctx.gearBody.name = 'Gear Body'
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        cylindricalFace = None
        planarCount = 0
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                cylindricalFace = face
            elif face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                planarCount += 1
                if sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry):
                    ctx.extrusionExtent = face
        if cylindricalFace is None or ctx.extrusionExtent is None:
            raise RuntimeError(
                f'Spur gear: root disc has {ctx.gearBody.faces.count} faces and {planarCount} planar faces; '
                f'cylinder found={cylindricalFace is not None}, far cap found={ctx.extrusionExtent is not None}')
        axes = self.getComponent().constructionAxes
        axisInput = axes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        ctx.centerAxis = axes.add(axisInput)
        ctx.centerAxis.name = 'Gear Center'
        ctx.centerAxis.isLightBulbOn = False

    def patternTeeth(self, ctx):
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        patterns = self.getComponent().features.circularPatternFeatures
        patternInput = patterns.createInput(entities, ctx.centerAxis)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            body = pattern.bodies.item(i)
            toolBodies.add(body)
        if toolBodies.count == 0:
            raise RuntimeError('Spur gear: circular pattern returned 0 bodies for the tooth join')
        combines = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combines.add(combineInput)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(ctx.plane)
        edges = adsk.core.ObjectCollection.create()
        seen = set()
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            cylinder = adsk.core.Cylinder.cast(face.geometry)
            if abs(cylinder.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                line = adsk.core.Line3D.cast(edge.geometry)
                direction = line.startPoint.vectorTo(line.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01 and edge.tempId not in seen:
                    edges.add(edge)
                    seen.add(edge.tempId)
        if edges.count == 0:
            return
        fillets = self.getComponent().features.filletFeatures
        filletInput = fillets.createInput()
        radiusValue = adsk.core.ValueInput.createByReal(filletRadius)
        filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)
        fillets.add(filletInput)

    def buildBore(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        sketch = self.createSketchObject('Bore Profile', ctx.plane)
        sketch.isVisible = True
        self.boreSketch = sketch
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        circle = toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        projectedAnchor = circle.centerSketchPoint
        constraints = sketch.geometricConstraints
        constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)
        if not sketch.isFullyConstrained:
            raise RuntimeError('Spur gear: Bore Profile sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise RuntimeError(f'Spur gear: Bore Profile has {sketch.profiles.count} regions, expected 1')
        profile = sketch.profiles.item(0)
        extrudes = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    def chamferTeeth(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferTooth = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferTooth <= 0:
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        edges = adsk.core.ObjectCollection.create()
        seen = set()
        capCount = 0
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            if not sketchPlane.isParallelToPlane(face.geometry):
                continue
            capCount += 1
            for edge in face.edges:
                if edge.tempId in seen:
                    continue
                seen.add(edge.tempId)
                if boreDiameter > 0 and edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                    circle = adsk.core.Circle3D.cast(edge.geometry)
                    if abs(circle.radius - boreDiameter / 2) <= 0.001:
                        continue
                edges.add(edge)
        if capCount == 0 or edges.count == 0:
            raise RuntimeError(
                f'Spur gear: completed chamfer found {capCount} end-cap faces and {edges.count} outer edges '
                f'among {ctx.gearBody.faces.count} faces; bore diameter={boreDiameter} cm')
        chamfers = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        distanceValue = adsk.core.ValueInput.createByReal(chamferTooth)
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distanceValue, False)
        chamfers.add(chamferInput)

    def cleanup(self, ctx):
        for entity in (ctx.extrusionEndPlane, ctx.centerAxis, self._normalizedPlane):
            if entity is not None:
                entity.isLightBulbOn = False
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            for sketch in (self.toolsSketch, ctx.gearProfileSketch, self.boreSketch):
                if sketch is not None:
                    sketch.isVisible = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        point = adsk.core.Point3D.create(0, 0, 0)
        self.anchorPoint = sketch.sketchPoints.add(point)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.spineAngularDimension = adsk.fusion.SketchAngularDimension.cast(None)

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return x, y

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        constraints = sketch.geometricConstraints
        localOrigin = self.anchorPoint
        constraints.addCoincident(localOrigin, projectedAnchor)
        futil.log(f'{sketch.name}: isFullyConstrained={sketch.isFullyConstrained}; circle labels carry text DOF')
        if angle != 0:
            self.spineAngularDimension.parameter.value = angle

    def drawCircles(self):
        sketch = self.sketch
        localOrigin = self.anchorPoint
        circles = sketch.sketchCurves.sketchCircles
        dimensions = sketch.sketchDimensions
        size = self.getParameterValue(PARAM_TIP_RADIUS) - self.getParameterValue(PARAM_ROOT_RADIUS)
        for name, parameter, construction in (
            ('Root Circle', PARAM_ROOT_RADIUS, False),
            ('Tip Circle', PARAM_TIP_RADIUS, True),
            ('Base Circle', PARAM_BASE_RADIUS, True),
            ('Pitch Circle', PARAM_PITCH_RADIUS, True),
        ):
            radius = self.getParameterValue(parameter)
            circle = circles.addByCenterRadius(localOrigin, radius)
            circle.isConstruction = construction
            textPoint = adsk.core.Point3D.create(radius, 0, 0)
            dimension = dimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2 * radius
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(text, size)
            textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            if parameter == PARAM_ROOT_RADIUS:
                self.rootCircle = circle
            elif parameter == PARAM_TIP_RADIUS:
                self.tipCircle = circle

    def drawTooth(self, angle=0):
        sketch = self.sketch
        localOrigin = self.anchorPoint
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None or steps < 2:
            raise RuntimeError(f'Spur tooth: invalid pitch crossing or sample count {steps}')
        px, py = pitchPoint
        rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-py, px)
        cr, sr = math.cos(rotate_angle), math.sin(rotate_angle)
        ca, sa = math.cos(angle), math.sin(angle)
        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for i in range(steps):
            radius = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            sample = self.calculateInvolutePoint(baseRadius, radius)
            if sample is None:
                continue
            x, y = sample
            y = -y
            lx, ly = x * cr - y * sr, x * sr + y * cr
            point = adsk.core.Point3D.create(lx * ca - ly * sa, lx * sa + ly * ca, 0)
            leftPoints.add(point)
            point = adsk.core.Point3D.create(lx * ca + ly * sa, lx * sa - ly * ca, 0)
            rightPoints.add(point)
        if leftPoints.count < 2:
            raise RuntimeError(f'Spur tooth: only {leftPoints.count} valid involute samples')
        splines = sketch.sketchCurves.sketchFittedSplines
        leftSpline = splines.add(leftPoints)
        rightSpline = splines.add(rightPoints)
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines
        arcs = sketch.sketchCurves.sketchArcs
        toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius * ca, tipRadius * sa, 0))
        tipCircle = self.tipCircle
        constraints.addCoincident(toothTopPoint, tipCircle)
        rightFlankEndPoint = rightSpline.endSketchPoint
        leftFlankEndPoint = leftSpline.endSketchPoint
        arc = arcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)
        constraints.addCoincident(arc.centerSketchPoint, localOrigin)
        spine = lines.addByTwoPoints(localOrigin, toothTopPoint)
        spine.isConstruction = True
        referenceEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        textPoint = adsk.core.Point3D.create(tipRadius, tipRadius / 4, 0)
        orientation = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        dimension = dimensions.addDistanceDimension(localOrigin, referenceEnd, orientation, textPoint)
        dimension.parameter.value = tipRadius
        orientation = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        dimension = dimensions.addDistanceDimension(localOrigin, referenceEnd, orientation, textPoint)
        dimension.parameter.value = 0
        reference = lines.addByTwoPoints(localOrigin, referenceEnd)
        reference.isConstruction = True
        textPoint = adsk.core.Point3D.create(tipRadius * math.cos(angle / 2) / 2,
                                           tipRadius * math.sin(angle / 2) / 2, 0)
        self.spineAngularDimension = dimensions.addAngularDimension(reference, spine, textPoint)
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        ribOrientation, chainOrientation = (vertical, horizontal) if abs(ca) >= abs(sa) else (horizontal, vertical)
        previous = localOrigin
        for i in range(leftSpline.fitPoints.count):
            left = leftSpline.fitPoints.item(i)
            right = rightSpline.fitPoints.item(i)
            rib = lines.addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])
            rib.isConstruction = True
            textPoint = left.geometry
            dimensions.addDistanceDimension(left, right, ribOrientation, textPoint)
            fitX, fitY = left.geometry.x, left.geometry.y
            t = fitX * ca + fitY * sa
            midpoint = sketch.sketchPoints.add(adsk.core.Point3D.create(t * ca, t * sa, 0))
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            if i != leftSpline.fitPoints.count - 1:
                constraints.addPerpendicular(spine, rib)
            dimensions.addDistanceDimension(previous, midpoint, chainOrientation, textPoint)
            previous = midpoint
        first = leftSpline.startSketchPoint.geometry
        firstRadius = math.hypot(first.x - localOrigin.geometry.x, first.y - localOrigin.geometry.y)
        self.parent._lastToothEmbedded = firstRadius < rootRadius
        if not self.parent._lastToothEmbedded:
            self._drawFlankToRoot(leftSpline.startSketchPoint, rootRadius)
            self._drawFlankToRoot(rightSpline.startSketchPoint, rootRadius)

    def _drawFlankToRoot(self, flankStartFitPoint, rootRadius):
        localOrigin = self.anchorPoint
        origin = localOrigin.geometry
        start = flankStartFitPoint.geometry
        dx, dy = start.x - origin.x, start.y - origin.y
        radius = math.hypot(dx, dy)
        dx, dy = dx * rootRadius / radius, dy * rootRadius / radius
        rootEndGeometry = adsk.core.Point3D.create(origin.x + dx, origin.y + dy, 0)
        lines = self.sketch.sketchCurves.sketchLines
        line = lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)
        rootEnd = line.startSketchPoint
        dimensions = self.sketch.sketchDimensions
        textPoint = rootEndGeometry
        dimension = dimensions.addDistanceDimension(
            localOrigin, rootEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dimension.parameter.value = abs(dx)
        dimension = dimensions.addDistanceDimension(
            localOrigin, rootEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)
        dimension.parameter.value = abs(dy)

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        boreDiameter = diameter
        circles = sketch.sketchCurves.sketchCircles
        circle = circles.addByCenterRadius(projectedAnchor, boreDiameter / 2)
        circle.isConstruction = False
        center = projectedAnchor.geometry
        textPoint = adsk.core.Point3D.create(center.x + boreDiameter / 2, center.y, center.z)
        dimensions = sketch.sketchDimensions
        dimension = dimensions.addDiameterDimension(circle, textPoint)
        dimension.parameter.value = boreDiameter
        return circle
