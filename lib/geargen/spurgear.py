import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .base import GenerationContext, Generator, get_boolean, get_selection, get_value
from .misc import get_design, to_cm
from .utilities import find_profile_by_curve_counts, get_normal


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
    def __init__(self, design):
        super().__init__(design)
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False
        self._normalizedPlane = None
        self.plane = None
        self.anchorPoint = None

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
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise ValueError(f'Spur gear requires one parent component; got {len(parents)}')
        parent = parents[0]
        if isinstance(parent, adsk.fusion.Occurrence):
            parent = parent.component
        if not isinstance(parent, adsk.fusion.Component):
            raise TypeError('Spur gear parent must be a component or occurrence')
        self.parentComponent = parent
        planes = get_selection(inputs, INPUT_ID_PLANE)
        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(planes) != 1 or len(anchors) != 1:
            raise ValueError(f'Spur gear requires one plane and anchor; got {len(planes)} and {len(anchors)}')
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

    def generate(self, inputs):
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()
        ctx = self.newContext()
        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx):
        component = self.getComponent()
        if not isinstance(self.plane, adsk.fusion.ConstructionPlane):
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self._normalizedPlane = self.plane
        ctx.plane = self.plane
        toolsSketch = self.createSketchObject('Tools', self.plane)
        self.toolsSketch = toolsSketch
        toolsSketch.isVisible = True
        ctx.anchorPoint = toolsSketch.project(self.anchorPoint).item(0)
        thickness = self.getParameter(PARAM_THICKNESS).value
        endPlaneInput = component.constructionPlanes.createInput()
        endPlaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        ctx.extrusionEndPlane = component.constructionPlanes.add(endPlaneInput)
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
        sketch = self.createSketchObject('Gear Profile', self.plane)
        ctx.gearProfileSketch = sketch
        sketch.isVisible = True
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGen.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded
        futil.log(f'Gear Profile isFullyConstrained: {sketch.isFullyConstrained}')

    def buildTooth(self, ctx):
        component = self.getComponent()
        profile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)
        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        if extrude.bodies.count != 1:
            raise RuntimeError(f'Extrude tooth produced {extrude.bodies.count} bodies; expected one')
        ctx.toothBody = extrude.bodies.item(0)

    def buildBody(self, ctx):
        component = self.getComponent()
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)
        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude body'
        if extrude.bodies.count != 1:
            raise RuntimeError(f'Extrude body produced {extrude.bodies.count} bodies; expected one')
        ctx.gearBody = extrude.bodies.item(0)
        ctx.gearBody.name = 'Gear Body'
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        cylindricalFace = None
        for face in ctx.gearBody.faces:
            geometry = face.geometry
            if geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                cylindricalFace = face
            elif geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if sketchPlane.isParallelToPlane(geometry) and not sketchPlane.isCoPlanarTo(geometry):
                    ctx.extrusionExtent = face
        if cylindricalFace is None or ctx.extrusionExtent is None:
            raise RuntimeError(
                f'Gear Body has {ctx.gearBody.faces.count} faces; '
                f'cylinder found={cylindricalFace is not None}, far cap found={ctx.extrusionExtent is not None}')
        axisInput = component.constructionAxes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        ctx.centerAxis = component.constructionAxes.add(axisInput)
        ctx.centerAxis.name = 'Gear Center'
        ctx.centerAxis.isLightBulbOn = False

    def patternTeeth(self, ctx):
        component = self.getComponent()
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(ctx.toothBody)
        patternInput = component.features.circularPatternFeatures.createInput(bodies, ctx.centerAxis)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = component.features.circularPatternFeatures.add(patternInput)
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))
        if toolBodies.count == 0:
            raise RuntimeError(f'Spur tooth pattern produced zero bodies for quantity {toothNumber}')
        combineInput = component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(combineInput)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = {}
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(face.geometry.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01 and edge.tempId not in seen:
                    seen[edge.tempId] = True
                    edges.add(edge)
        if edges.count == 0:
            return
        component = self.getComponent()
        filletInput = component.features.filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        component.features.filletFeatures.add(filletInput)

    def buildBore(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        boreSketch = self.createSketchObject('Bore Profile', self.plane)
        self.boreSketch = boreSketch
        boreSketch.isVisible = True
        toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, toothGen.projectedAnchor)
        if not boreSketch.isFullyConstrained:
            raise RuntimeError('Bore Profile is not fully constrained after anchoring')
        if boreSketch.profiles.count != 1:
            raise RuntimeError(f'Bore Profile has {boreSketch.profiles.count} profiles; expected one')
        boreProfile = boreSketch.profiles.item(0)
        component = self.getComponent()
        cutInput = component.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        cutExtent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        cutInput.setOneSideExtent(cutExtent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        cutInput.participantBodies = [ctx.gearBody]
        component.features.extrudeFeatures.add(cutInput)

    def chamferTeeth(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance == 0:
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = {}
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
                seen[edge.tempId] = True
                geometry = edge.geometry
                if (boreDiameter > 0 and geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType
                        and abs(geometry.radius - boreDiameter / 2) <= 0.001):
                    continue
                edges.add(edge)
        if capCount == 0 or edges.count == 0:
            raise RuntimeError(f'Spur chamfer found {capCount} end caps and {edges.count} eligible edges')
        component = self.getComponent()
        chamferInput = component.features.chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferDistance), False)
        component.features.chamferFeatures.add(chamferInput)

    def cleanup(self, ctx):
        for entity in (ctx.extrusionEndPlane, ctx.centerAxis, self._normalizedPlane):
            if entity is not None:
                entity.isLightBulbOn = False
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            for sketch in (self.toolsSketch, ctx.gearProfileSketch, self.boreSketch):
                if sketch is not None:
                    sketch.isVisible = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self.projectedAnchor = adsk.fusion.SketchPoint.cast(None)
        self.spineAngularDimension = None
        self.circles = {}

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        return (baseRadius * (math.cos(t) + t * math.sin(t)),
                baseRadius * (math.sin(t) - t * math.cos(t)))

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)
        futil.log(f'{sketch.name} isFullyConstrained: {sketch.isFullyConstrained}')
        if angle != 0:
            self.spineAngularDimension.parameter.value = angle

    def drawCircles(self):
        sketch = self.sketch
        size = self.getParameterValue(PARAM_TIP_RADIUS) - self.getParameterValue(PARAM_ROOT_RADIUS)
        for name, key, construction in (
                ('Root Circle', PARAM_ROOT_RADIUS, False),
                ('Tip Circle', PARAM_TIP_RADIUS, True),
                ('Base Circle', PARAM_BASE_RADIUS, True),
                ('Pitch Circle', PARAM_PITCH_RADIUS, True)):
            radius = self.getParameterValue(key)
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = construction
            self.circles[key] = circle
            textPoint = adsk.core.Point3D.create(radius, radius, 0)
            dimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2 * radius
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(text, size)
            textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)

    def drawTooth(self, angle=0):
        sketch = self.sketch
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise ValueError(f'InvoluteSteps must be at least two; got {steps}')
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None:
            raise ValueError(f'Pitch radius {pitchRadius} is below base radius {baseRadius}')
        px, py = pitchPoint
        rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-py, px)
        c, s = math.cos(angle), math.sin(angle)
        rc, rs = math.cos(rotate_angle), math.sin(rotate_angle)
        left = []
        right = []
        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for i in range(steps):
            r = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            point = self.calculateInvolutePoint(baseRadius, r)
            if point is None:
                continue
            x, y = point[0], -point[1]
            lx, ly = x * rc - y * rs, x * rs + y * rc
            left.append((lx * c - ly * s, lx * s + ly * c))
            right.append((lx * c + ly * s, lx * s - ly * c))
            leftPoints.add(adsk.core.Point3D.create(*left[-1], 0))
            rightPoints.add(adsk.core.Point3D.create(*right[-1], 0))
        if len(left) < 2:
            raise RuntimeError(f'Involute sampling produced {len(left)} points; expected at least two')
        leftSpline = sketch.sketchCurves.sketchFittedSplines.add(leftPoints)
        rightSpline = sketch.sketchCurves.sketchFittedSplines.add(rightPoints)
        toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius * c, tipRadius * s, 0))
        sketch.geometricConstraints.addCoincident(toothTopPoint, self.circles[PARAM_TIP_RADIUS])
        arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            self.anchorPoint, rightSpline.fitPoints.item(len(right) - 1), leftSpline.fitPoints.item(len(left) - 1))
        sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)
        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True
        referenceEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        textPoint = adsk.core.Point3D.create(tipRadius, tipRadius / 2, 0)
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        dim = sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, referenceEnd, horizontal, textPoint)
        dim.parameter.value = tipRadius
        dim = sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, referenceEnd, vertical, textPoint)
        dim.parameter.value = 0
        referenceLine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, referenceEnd)
        referenceLine.isConstruction = True
        radius = tipRadius / 4
        angleTextPoint = adsk.core.Point3D.create(radius * math.cos(angle / 2), radius * math.sin(angle / 2), 0)
        self.spineAngularDimension = sketch.sketchDimensions.addAngularDimension(referenceLine, spine, angleTextPoint)
        across = vertical if abs(c) >= abs(s) else horizontal
        along = horizontal if abs(c) >= abs(s) else vertical
        previous = self.anchorPoint
        for i, (fitX, fitY) in enumerate(left):
            leftFit = leftSpline.fitPoints.item(i)
            rightFit = rightSpline.fitPoints.item(i)
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftFit, rightFit)
            rib.isConstruction = True
            ribText = adsk.core.Point3D.create(fitX, fitY, 0)
            sketch.sketchDimensions.addDistanceDimension(leftFit, rightFit, across, ribText)
            t = fitX * c + fitY * s
            midpoint = sketch.sketchPoints.add(adsk.core.Point3D.create(t * c, t * s, 0))
            sketch.geometricConstraints.addCoincident(midpoint, spine)
            sketch.geometricConstraints.addMidPoint(midpoint, rib)
            if i != len(left) - 1:
                sketch.geometricConstraints.addPerpendicular(spine, rib)
            sketch.sketchDimensions.addDistanceDimension(previous, midpoint, along, ribText)
            previous = midpoint
        firstRadius = math.hypot(*left[0])
        embedded = firstRadius < rootRadius
        self.parent._lastToothEmbedded = embedded
        if not embedded:
            self._drawFlankToRoot(leftSpline.fitPoints.item(0), left[0], rootRadius)
            self._drawFlankToRoot(rightSpline.fitPoints.item(0), right[0], rootRadius)

    def _drawFlankToRoot(self, flankStartFitPoint, seed, rootRadius):
        sketch = self.sketch
        theta = math.atan2(seed[1], seed[0])
        dx, dy = rootRadius * math.cos(theta), rootRadius * math.sin(theta)
        rootEndPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(dx, dy, 0))
        sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndPoint, flankStartFitPoint)
        textPoint = adsk.core.Point3D.create(dx, dy, 0)
        horizontal = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEndPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        horizontal.parameter.value = abs(dx)
        vertical = sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEndPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)
        vertical.parameter.value = abs(dy)

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        self.projectedAnchor = projectedAnchor
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, diameter / 2)
        center = projectedAnchor.geometry
        textPoint = adsk.core.Point3D.create(center.x + diameter / 2, center.y, center.z)
        dimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
        dimension.parameter.value = diameter
        return circle
