import math
from typing import cast
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
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self._createdTargetPlane = False
        self._toolsSketch = adsk.fusion.Sketch.cast(None)
        self._boreSketch = adsk.fusion.Sketch.cast(None)
        self._lastToothEmbedded = False

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

    def processInputs(self, inputs: adsk.core.CommandInputs):
        parent = get_selection(inputs, INPUT_ID_PARENT)[0]
        self.plane = get_selection(inputs, INPUT_ID_PLANE)[0]
        self.anchorPoint = get_selection(inputs, INPUT_ID_ANCHOR_POINT)[0]
        if isinstance(parent, adsk.fusion.Occurrence):
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent
        self.getOccurrence()
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

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        component: adsk.fusion.Component = self.getComponent()
        component.name = self.generateName()
        self._createdTargetPlane = False
        if not isinstance(self.plane, adsk.fusion.ConstructionPlane):
            selectedPlane = self.plane
            planes: adsk.fusion.ConstructionPlanes = component.constructionPlanes
            planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
            offset = adsk.core.ValueInput.createByReal(0)
            planeInput.setByOffset(selectedPlane, offset)
            self.plane = planes.add(planeInput)
            self.plane.isLightBulbOn = True
            self._createdTargetPlane = True
        ctx = self.newContext()
        ctx.plane = self.plane
        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx: SpurGearGenerationContext):
        futil.log('Creating Tools sketch')
        toolsSketch: adsk.fusion.Sketch = self.createSketchObject('Tools', ctx.plane)
        toolsSketch.isVisible = True
        self._toolsSketch = toolsSketch
        ctx.anchorPoint = adsk.fusion.SketchPoint.cast(toolsSketch.project(self.anchorPoint).item(0))
        if ctx.anchorPoint is None:
            raise RuntimeError('Tools: anchor projection produced no SketchPoint')
        if not toolsSketch.isFullyConstrained:
            raise RuntimeError(f'{toolsSketch.name}: sketch is not fully constrained')
        thickness = self.getParameter(PARAM_THICKNESS).value
        planes: adsk.fusion.ConstructionPlanes = self.getComponent().constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
        offset = adsk.core.ValueInput.createByReal(thickness)
        planeInput.setByOffset(ctx.plane, offset)
        ctx.extrusionEndPlane = planes.add(planeInput)
        ctx.extrusionEndPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane.isLightBulbOn = True

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
        futil.log(f'{sketch.name}: isFullyConstrained={sketch.isFullyConstrained}')

    def buildTooth(self, ctx: SpurGearGenerationContext):
        profile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2)
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = extrudes.createInput(profile, operation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude: adsk.fusion.ExtrudeFeature = extrudes.add(extrudeInput)
        if extrude.bodies.count != 1:
            raise RuntimeError(f'Extrude tooth: expected one body, got {extrude.bodies.count}')
        ctx.toothBody = extrude.bodies.item(0)

    def buildBody(self, ctx: SpurGearGenerationContext):
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = extrudes.createInput(profile, operation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude: adsk.fusion.ExtrudeFeature = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'
        if extrude.bodies.count != 1:
            raise RuntimeError(f'Extrude body: expected one body, got {extrude.bodies.count}')
        body: adsk.fusion.BRepBody = extrude.bodies.item(0)
        body.name = 'Gear Body'
        ctx.gearBody = body
        sketchPlane: adsk.core.Plane = ctx.gearProfileSketch.referencePlane.geometry
        axes: adsk.fusion.ConstructionAxes = self.getComponent().constructionAxes
        for face in body.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if ctx.centerAxis is None:
                    cylindricalFace: adsk.fusion.BRepFace = face
                    axisInput: adsk.fusion.ConstructionAxisInput = axes.createInput()
                    axisInput.setByCircularFace(cylindricalFace)
                    ctx.centerAxis = axes.add(axisInput)
                    ctx.centerAxis.name = 'Gear Center'
                    ctx.centerAxis.isLightBulbOn = False
            elif face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                facePlane = cast(adsk.core.Plane, face.geometry)
                if sketchPlane.isParallelToPlane(facePlane) and not sketchPlane.isCoPlanarTo(facePlane):
                    ctx.extrusionExtent = face
        if ctx.centerAxis is None:
            raise RuntimeError('Extrude body: no cylindrical face for Gear Center')
        if ctx.extrusionExtent is None:
            raise RuntimeError('Extrude body: no far planar end cap')

    def patternTeeth(self, ctx: SpurGearGenerationContext):
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        patterns: adsk.fusion.CircularPatternFeatures = self.getComponent().features.circularPatternFeatures
        patternInput: adsk.fusion.CircularPatternFeatureInput = patterns.createInput(entities, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern: adsk.fusion.CircularPatternFeature = patterns.add(patternInput)
        tools = adsk.core.ObjectCollection.create()
        for patternBody in pattern.bodies:
            tools.add(patternBody)
        if tools.count == 0:
            raise RuntimeError('Pattern teeth: no bodies to combine')
        combines: adsk.fusion.CombineFeatures = self.getComponent().features.combineFeatures
        combineInput: adsk.fusion.CombineFeatureInput = combines.createInput(ctx.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

    def createFillets(self, ctx: SpurGearGenerationContext):
        radius = self.getParameter(PARAM_FILLET_RADIUS).value
        if radius <= 0:
            return
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal: adsk.core.Vector3D = get_normal(ctx.plane)
        edges = adsk.core.ObjectCollection.create()
        seen = set()
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            cylinder = cast(adsk.core.Cylinder, face.geometry)
            if abs(cylinder.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction: adsk.core.Vector3D = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01 and edge.tempId not in seen:
                    edges.add(edge)
                    seen.add(edge.tempId)
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
        self._boreSketch = boreSketch
        toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        circle = toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        projectedAnchor: adsk.fusion.SketchPoint = toothGen._boreProjectedAnchor
        constraints: adsk.fusion.GeometricConstraints = boreSketch.geometricConstraints
        constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)
        if not boreSketch.isFullyConstrained:
            raise RuntimeError(f'{boreSketch.name}: sketch is not fully constrained')
        if boreSketch.profiles.count != 1:
            raise RuntimeError(f'Bore Profile: expected one profile, got {boreSketch.profiles.count}')
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
        if chamferDistance == 0:
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        sketchPlane: adsk.core.Plane = ctx.gearProfileSketch.referencePlane.geometry
        edges = adsk.core.ObjectCollection.create()
        seen = set()
        endCaps = 0
        for face in ctx.gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            facePlane = cast(adsk.core.Plane, face.geometry)
            if not sketchPlane.isParallelToPlane(facePlane):
                continue
            endCaps += 1
            for edge in face.edges:
                if edge.tempId in seen:
                    continue
                seen.add(edge.tempId)
                if boreDiameter > 0 and edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                    circle = cast(adsk.core.Circle3D, edge.geometry)
                    if abs(circle.radius - boreDiameter / 2) <= 0.001:
                        continue
                edges.add(edge)
        if endCaps == 0 or edges.count == 0:
            raise RuntimeError(f'Chamfer teeth: found {endCaps} end caps and {edges.count} chamfer edges')
        chamfers: adsk.fusion.ChamferFeatures = self.getComponent().features.chamferFeatures
        chamferInput: adsk.fusion.ChamferFeatureInput = chamfers.createInput2()
        distance = adsk.core.ValueInput.createByReal(chamferDistance)
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)
        chamfers.add(chamferInput)

    def cleanup(self, ctx: SpurGearGenerationContext):
        if self._createdTargetPlane and ctx.plane is not None:
            ctx.plane.isLightBulbOn = False
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            if self._toolsSketch is not None:
                self._toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self._boreSketch is not None:
                self._boreSketch.isVisible = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.sketch: adsk.fusion.Sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        self.anchorPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self._tipCircle = adsk.fusion.SketchCircle.cast(None)
        self._angleDimension = adsk.fusion.SketchAngularDimension.cast(None)
        self._boreProjectedAnchor = adsk.fusion.SketchPoint.cast(None)

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
        return adsk.core.Point3D.create(x, y, 0)

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        sketch: adsk.fusion.Sketch = self.sketch
        projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(anchorPoint).item(0))
        if projectedAnchor is None:
            raise RuntimeError(f'{sketch.name}: anchor projection produced no SketchPoint')
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        constraints.addCoincident(self.anchorPoint, projectedAnchor)
        if angle != 0:
            if self._angleDimension is None:
                raise RuntimeError(f'{sketch.name}: missing confirming angular dimension')
            self._angleDimension.parameter.value = angle

    def drawCircles(self):
        sketch: adsk.fusion.Sketch = self.sketch
        circles: adsk.fusion.SketchCircles = sketch.sketchCurves.sketchCircles
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        localOrigin = self.anchorPoint
        size = self.getParameterValue(PARAM_TIP_RADIUS) - self.getParameterValue(PARAM_ROOT_RADIUS)
        for name, parameter, construction in (
                ('Root Circle', PARAM_ROOT_RADIUS, False),
                ('Tip Circle', PARAM_TIP_RADIUS, True),
                ('Base Circle', PARAM_BASE_RADIUS, True),
                ('Pitch Circle', PARAM_PITCH_RADIUS, True)):
            radius = self.getParameterValue(parameter)
            circle: adsk.fusion.SketchCircle = circles.addByCenterRadius(localOrigin, radius)
            circle.isConstruction = construction
            offCenterTextPoint = adsk.core.Point3D.create(radius, radius, 0)
            dimension: adsk.fusion.SketchDiameterDimension = dims.addDiameterDimension(circle, offCenterTextPoint)
            dimension.parameter.value = 2 * radius
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput: adsk.fusion.SketchTextInput = sketch.sketchTexts.createInput2(text, size)
            alignment = cast(adsk.core.HorizontalAlignments, adsk.core.HorizontalAlignments.CenterHorizontalAlignment)
            textInput.setAsAlongPath(circle, True, alignment, 0)
            sketch.sketchTexts.add(textInput)
            if parameter == PARAM_TIP_RADIUS:
                self._tipCircle = circle

    def drawTooth(self, angle=0):
        sketch: adsk.fusion.Sketch = self.sketch
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise RuntimeError('Gear Profile: InvoluteSteps must be at least two')
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        if pitchPoint is None:
            raise RuntimeError('Gear Profile: pitch radius is below base radius')
        rotateAngle = math.pi / (2 * toothNumber) - math.atan2(-pitchPoint.y, pitchPoint.x)
        ca, sa = math.cos(angle), math.sin(angle)
        cr, sr = math.cos(rotateAngle), math.sin(rotateAngle)
        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for i in range(steps):
            radius = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            point = self.calculateInvolutePoint(baseRadius, radius)
            if point is None:
                continue
            x = point.x * cr + point.y * sr
            y = point.x * sr - point.y * cr
            leftPoints.add(adsk.core.Point3D.create(x * ca - y * sa, x * sa + y * ca, 0))
            rightPoints.add(adsk.core.Point3D.create(x * ca + y * sa, x * sa - y * ca, 0))
        if leftPoints.count < 2:
            raise RuntimeError('Gear Profile: fewer than two involute samples')
        splines: adsk.fusion.SketchFittedSplines = sketch.sketchCurves.sketchFittedSplines
        leftSpline: adsk.fusion.SketchFittedSpline = splines.add(leftPoints)
        rightSpline: adsk.fusion.SketchFittedSpline = splines.add(rightPoints)
        leftFits: adsk.fusion.SketchPointList = leftSpline.fitPoints
        rightFits: adsk.fusion.SketchPointList = rightSpline.fitPoints
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        lines: adsk.fusion.SketchLines = sketch.sketchCurves.sketchLines
        arcs: adsk.fusion.SketchArcs = sketch.sketchCurves.sketchArcs
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        localOrigin: adsk.fusion.SketchPoint = self.anchorPoint
        toothTopPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipRadius * ca, tipRadius * sa, 0))
        tipCircle = self._tipCircle
        if tipCircle is None:
            raise RuntimeError('Gear Profile: missing Tip Circle')
        constraints.addCoincident(toothTopPoint, tipCircle)
        rightFlankEndPoint = rightFits.item(rightFits.count - 1)
        leftFlankEndPoint = leftFits.item(leftFits.count - 1)
        arc: adsk.fusion.SketchArc = arcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)
        constraints.addCoincident(arc.centerSketchPoint, localOrigin)
        spine: adsk.fusion.SketchLine = lines.addByTwoPoints(localOrigin, toothTopPoint)
        spine.isConstruction = True
        referenceEnd: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        textPoint = adsk.core.Point3D.create(tipRadius, tipRadius / 2, 0)
        horizontalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            localOrigin, referenceEnd, horizontal, textPoint)
        horizontalDim.parameter.value = tipRadius
        verticalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            localOrigin, referenceEnd, vertical, textPoint)
        verticalDim.parameter.value = 0
        reference: adsk.fusion.SketchLine = lines.addByTwoPoints(localOrigin, referenceEnd)
        reference.isConstruction = True
        bisectorTextPoint = adsk.core.Point3D.create(
            tipRadius / 2 * math.cos(angle / 2), tipRadius / 2 * math.sin(angle / 2), 0)
        self._angleDimension = dims.addAngularDimension(reference, spine, bisectorTextPoint)
        acrossOrientation = vertical if abs(ca) >= abs(sa) else horizontal
        alongOrientation = horizontal if abs(ca) >= abs(sa) else vertical
        previousMidpoint: adsk.fusion.SketchPoint = localOrigin
        for i in range(leftFits.count):
            leftFitPoint: adsk.fusion.SketchPoint = leftFits.item(i)
            rightFitPoint: adsk.fusion.SketchPoint = rightFits.item(i)
            rib: adsk.fusion.SketchLine = lines.addByTwoPoints(leftFitPoint, rightFitPoint)
            rib.isConstruction = True
            fit = leftFitPoint.geometry
            textPoint = adsk.core.Point3D.create(fit.x, fit.y, 0)
            dims.addDistanceDimension(leftFitPoint, rightFitPoint, acrossOrientation, textPoint)
            t = fit.x * ca + fit.y * sa
            midpoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(t * ca, t * sa, 0))
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            if i != leftFits.count - 1:
                constraints.addPerpendicular(spine, rib)
            dims.addDistanceDimension(previousMidpoint, midpoint, alongOrientation, textPoint)
            previousMidpoint = midpoint
        firstLeft: adsk.fusion.SketchPoint = leftFits.item(0)
        firstRight: adsk.fusion.SketchPoint = rightFits.item(0)
        firstRadius = math.hypot(
            firstLeft.geometry.x - localOrigin.geometry.x,
            firstLeft.geometry.y - localOrigin.geometry.y)
        self.parent._lastToothEmbedded = firstRadius < rootRadius
        if not self.parent._lastToothEmbedded:
            self._drawFlankToRoot(firstLeft, rootRadius)
            self._drawFlankToRoot(firstRight, rootRadius)

    def _drawFlankToRoot(self, flankStartFitPoint: adsk.fusion.SketchPoint, rootRadius):
        localOrigin: adsk.fusion.SketchPoint = self.anchorPoint
        dx = flankStartFitPoint.geometry.x - localOrigin.geometry.x
        dy = flankStartFitPoint.geometry.y - localOrigin.geometry.y
        scale = rootRadius / math.hypot(dx, dy)
        dx, dy = dx * scale, dy * scale
        rootEndGeometry = adsk.core.Point3D.create(localOrigin.geometry.x + dx, localOrigin.geometry.y + dy, 0)
        lines: adsk.fusion.SketchLines = self.sketch.sketchCurves.sketchLines
        line: adsk.fusion.SketchLine = lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)
        rootEnd = line.startSketchPoint
        dims: adsk.fusion.SketchDimensions = self.sketch.sketchDimensions
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        textPoint = rootEndGeometry
        horizontalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            localOrigin, rootEnd, horizontal, textPoint)
        horizontalDim.parameter.value = abs(dx)
        verticalDim: adsk.fusion.SketchLinearDimension = dims.addDistanceDimension(
            localOrigin, rootEnd, vertical, textPoint)
        verticalDim.parameter.value = abs(dy)

    def drawBore(self, anchorPoint, diameter):
        sketch: adsk.fusion.Sketch = self.sketch
        projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(anchorPoint).item(0))
        if projectedAnchor is None:
            raise RuntimeError(f'{sketch.name}: anchor projection produced no SketchPoint')
        self._boreProjectedAnchor = projectedAnchor
        circles: adsk.fusion.SketchCircles = sketch.sketchCurves.sketchCircles
        circle: adsk.fusion.SketchCircle = circles.addByCenterRadius(projectedAnchor, diameter / 2)
        circle.isConstruction = False
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        offCenterTextPoint = adsk.core.Point3D.create(
            projectedAnchor.geometry.x + diameter / 2,
            projectedAnchor.geometry.y + diameter / 2, 0)
        dimension: adsk.fusion.SketchDiameterDimension = dims.addDiameterDimension(circle, offCenterTextPoint)
        dimension.parameter.value = diameter
        return circle
