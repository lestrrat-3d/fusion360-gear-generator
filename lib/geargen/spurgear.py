import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .base import Generator, GenerationContext, get_boolean, get_selection, get_value
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
        self.plane = None
        self.anchorPoint = None
        self.normalizedPlane = None
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False

    def prefixBase(self) -> str:
        return 'SpurGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def filletHelixFactorExpression(self) -> str:
        return '1'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def processInputs(self, inputs):
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise ValueError(f'Spur Gear: expected one Parent Component, got {len(parents)}')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            parent = parent.component
        if parent.objectType != adsk.fusion.Component.classType():
            raise TypeError('Spur Gear: Parent Component must be a component or occurrence')
        self.parentComponent = parent

        planes = get_selection(inputs, INPUT_ID_PLANE)
        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(planes) != 1 or len(anchors) != 1:
            raise ValueError(
                f'Spur Gear: expected one Target Plane and one Anchor Point, '
                f'got {len(planes)} and {len(anchors)}')
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

    def addExtraPrimaryParameters(self, inputs):
        pass

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
        if self.plane is None:
            raise ValueError('Spur Gear: Target Plane is missing')
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self.normalizedPlane = self.plane
        ctx.plane = self.plane

        toolsSketch = self.createSketchObject('Tools', self.plane)
        toolsSketch.isVisible = True
        ctx.anchorPoint = toolsSketch.project(self.anchorPoint).item(0)
        self.toolsSketch = toolsSketch

        thickness = self.getParameter(PARAM_THICKNESS).value
        endPlaneInput = component.constructionPlanes.createInput()
        endPlaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
        ctx.extrusionEndPlane = component.constructionPlanes.add(endPlaneInput)
        ctx.extrusionEndPlane.name = 'Extrusion End Plane'

    def buildSketches(self, ctx):
        sketch = self.createSketchObject('Gear Profile', self.plane)
        ctx.gearProfileSketch = sketch
        sketch.isVisible = True
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGen.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded
        futil.log(f'Gear Profile fully constrained: {sketch.isFullyConstrained}')

    def buildMainGearBody(self, ctx):
        self.buildSketches(ctx)
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            ctx.gearProfileSketch.isVisible = True
            return
        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)
        self.createFillets(ctx)

    def buildTooth(self, ctx):
        component = self.getComponent()
        profile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2)
        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(extrudeInput)
        extrude.name = 'Extrude tooth'
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
        ctx.gearBody = extrude.bodies.item(0)
        ctx.gearBody.name = 'Gear Body'

        cylindricalFace = None
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        for face in ctx.gearBody.faces:
            surface = face.geometry
            if surface.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                cylindricalFace = face
            elif surface.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if sketchPlane.isParallelToPlane(surface) and not sketchPlane.isCoPlanarTo(surface):
                    ctx.extrusionExtent = face
        if cylindricalFace is None or ctx.extrusionExtent is None:
            raise RuntimeError(
                'Spur Gear: Extrude body did not provide a root cylinder and far planar cap '
                f'(cylinder={cylindricalFace is not None}, far cap={ctx.extrusionExtent is not None})')
        axisInput = component.constructionAxes.createInput()
        axisInput.setByCircularFace(cylindricalFace)
        ctx.centerAxis = component.constructionAxes.add(axisInput)
        ctx.centerAxis.name = 'Gear Center'
        ctx.centerAxis.isLightBulbOn = False

    def patternTeeth(self, ctx):
        component = self.getComponent()
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(ctx.toothBody)
        patternInput = component.features.circularPatternFeatures.createInput(bodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = component.features.circularPatternFeatures.add(patternInput)
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))
        if toolBodies.count == 0:
            raise RuntimeError('Spur Gear: circular pattern returned zero tooth bodies')
        combineInput = component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(combineInput)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return
        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(self.plane)
        edges = adsk.core.ObjectCollection.create()
        seen = {}
        for face in ctx.gearBody.faces:
            surface = face.geometry
            if surface.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(surface.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                direction = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)
                direction.normalize()
                if abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01:
                    if edge.tempId not in seen:
                        seen[edge.tempId] = True
                        edges.add(edge)
        if edges.count == 0:
            return
        filletInput = component.features.filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        component.features.filletFeatures.add(filletInput)

    def buildBore(self, ctx):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        component = self.getComponent()
        boreSketch = self.createSketchObject('Bore Profile', self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch
        toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        projectedAnchor = toothGen.projectedAnchor
        if projectedAnchor is None:
            raise RuntimeError('Spur Gear: Bore Profile anchor projection failed')
        boreSketch.geometricConstraints.addCoincident(
            toothGen.anchorPoint, projectedAnchor)
        if not boreSketch.isFullyConstrained:
            raise RuntimeError('Spur Gear: Bore Profile is not fully constrained')
        if boreSketch.profiles.count != 1:
            raise RuntimeError(
                f'Spur Gear: Bore Profile has {boreSketch.profiles.count} profiles, expected one')
        boreProfile = boreSketch.profiles.item(0)
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
        component = self.getComponent()
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        boreRadius = self.getParameter(PARAM_BORE_DIAMETER).value / 2
        edges = adsk.core.ObjectCollection.create()
        seen = {}
        planarFaces = 0
        for face in ctx.gearBody.faces:
            surface = face.geometry
            if surface.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            if not sketchPlane.isParallelToPlane(surface):
                continue
            planarFaces += 1
            for edge in face.edges:
                curve = edge.geometry
                if curve.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                    if boreRadius > 0 and abs(curve.radius - boreRadius) <= 0.001:
                        continue
                if edge.tempId not in seen:
                    seen[edge.tempId] = True
                    edges.add(edge)
        if planarFaces == 0 or edges.count == 0:
            raise RuntimeError(
                f'Spur Gear: chamfer found {planarFaces} end-cap faces and {edges.count} edges')
        chamferInput = component.features.chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferDistance), False)
        component.features.chamferFeatures.add(chamferInput)

    def cleanup(self, ctx):
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False
        if not self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self.projectedAnchor = None

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    @staticmethod
    def calculateInvolutePoint(baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return x, y

    @staticmethod
    def _rotate(x, y, angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return x * c - y * s, x * s + y * c

    @staticmethod
    def _point(x, y):
        return adsk.core.Point3D.create(x, y, 0)

    def _axis_dimension(self, first, second, orientation, x, y):
        dimension = self.sketch.sketchDimensions.addDistanceDimension(
            first, second, orientation, self._point(x, y))
        return dimension

    def drawCircles(self):
        sketch = self.sketch
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        size = tipRadius - rootRadius
        circles = {}
        for name, radius, construction in (
            ('Root Circle', rootRadius, False),
            ('Tip Circle', tipRadius, True),
            ('Base Circle', baseRadius, True),
            ('Pitch Circle', pitchRadius, True),
        ):
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = construction
            dimension = sketch.sketchDimensions.addDiameterDimension(circle, self._point(radius, 0))
            dimension.parameter.value = 2 * radius
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(text, size)
            textInput.setAsAlongPath(
                circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            circles[name] = circle
        return circles

    def drawTooth(self, angle=0, circles=None):
        if circles is None:
            circles = self.drawCircles()
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise ValueError('Spur Gear: InvoluteSteps must be at least two')
        px, py = self.calculateInvolutePoint(baseRadius, pitchRadius)
        rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-py, px)
        left = []
        right = []
        for i in range(steps):
            radius = baseRadius + (tipRadius - baseRadius) * i / (steps - 1)
            sample = self.calculateInvolutePoint(baseRadius, radius)
            if sample is None:
                continue
            x, y = sample
            lx, ly = self._rotate(x, -y, rotate_angle)
            rx, ry = lx, -ly
            left.append(self._rotate(lx, ly, angle))
            right.append(self._rotate(rx, ry, angle))
        if len(left) < 2:
            raise RuntimeError(f'Spur Gear: involute produced {len(left)} fit points')

        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for x, y in left:
            leftPoints.add(self._point(x, y))
        for x, y in right:
            rightPoints.add(self._point(x, y))
        leftSpline: adsk.fusion.SketchFittedSpline = (
            sketch.sketchCurves.sketchFittedSplines.add(leftPoints))
        rightSpline: adsk.fusion.SketchFittedSpline = (
            sketch.sketchCurves.sketchFittedSplines.add(rightPoints))
        tipCircle = circles['Tip Circle']
        toothTopPoint = sketch.sketchPoints.add(
            self._point(tipRadius * math.cos(angle), tipRadius * math.sin(angle)))
        constraints.addCoincident(toothTopPoint, tipCircle)
        arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            self.anchorPoint, rightSpline.endSketchPoint, leftSpline.endSketchPoint)
        constraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)

        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True
        referenceEnd = sketch.sketchPoints.add(self._point(tipRadius, 0))
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        self._axis_dimension(self.anchorPoint, referenceEnd, horizontal, tipRadius / 2, 0)
        self._axis_dimension(self.anchorPoint, referenceEnd, vertical, tipRadius, tipRadius / 4)
        referenceLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            self.anchorPoint, referenceEnd)
        referenceLine.isConstruction = True
        spineAngularDimension = dimensions.addAngularDimension(
            referenceLine, spine,
            self._point(tipRadius * math.cos(angle / 2), tipRadius * math.sin(angle / 2)))

        acrossIsVertical = abs(math.cos(angle)) >= abs(math.sin(angle))
        previous = self.anchorPoint
        previousX = previousY = 0
        for i in range(len(left)):
            leftPoint = leftSpline.fitPoints.item(i)
            rightPoint = rightSpline.fitPoints.item(i)
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftPoint, rightPoint)
            rib.isConstruction = True
            if acrossIsVertical:
                axis = vertical
                magnitude = abs(right[i][1] - left[i][1])
            else:
                axis = horizontal
                magnitude = abs(right[i][0] - left[i][0])
            across = self._axis_dimension(
                leftPoint, rightPoint, axis,
                (left[i][0] + right[i][0]) / 2,
                (left[i][1] + right[i][1]) / 2)
            across.parameter.value = magnitude
            fitX, fitY = left[i]
            t = fitX * math.cos(angle) + fitY * math.sin(angle)
            midX, midY = t * math.cos(angle), t * math.sin(angle)
            midpoint = sketch.sketchPoints.add(self._point(midX, midY))
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            if i != len(left) - 1:
                constraints.addPerpendicular(spine, rib)
            alongAxis = horizontal if acrossIsVertical else vertical
            alongMagnitude = abs(midX - previousX) if acrossIsVertical else abs(midY - previousY)
            along = self._axis_dimension(
                previous, midpoint, alongAxis,
                (midX + previousX) / 2, (midY + previousY) / 2)
            along.parameter.value = alongMagnitude
            previous, previousX, previousY = midpoint, midX, midY

        firstRadius = math.hypot(left[0][0], left[0][1])
        embedded = firstRadius < rootRadius
        self.parent._lastToothEmbedded = embedded
        if not embedded:
            for splineCurve, point in ((leftSpline, left[0]), (rightSpline, right[0])):
                self._drawFlankToRoot(splineCurve, point, rootRadius)
        return spineAngularDimension

    def _drawFlankToRoot(self, splineCurve, point, rootRadius):
        theta = math.atan2(point[1], point[0])
        rx = rootRadius * math.cos(theta)
        ry = rootRadius * math.sin(theta)
        rootEndPoint = self.sketch.sketchPoints.add(self._point(rx, ry))
        self.sketch.sketchCurves.sketchLines.addByTwoPoints(
            rootEndPoint, splineCurve.startSketchPoint)
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        xdim = self.sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEndPoint, horizontal, self._point(rx, ry))
        ydim = self.sketch.sketchDimensions.addDistanceDimension(
            self.anchorPoint, rootEndPoint, vertical, self._point(rx, ry))
        xdim.parameter.value = abs(rx)
        ydim.parameter.value = abs(ry)

    def draw(self, anchorPoint, angle=0):
        circles = self.drawCircles()
        spineAngularDimension = self.drawTooth(angle, circles)
        self.projectedAnchor = self.sketch.project(anchorPoint).item(0)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, self.projectedAnchor)
        if angle != 0:
            spineAngularDimension.parameter.value = angle

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projectedAnchor = sketch.project(anchorPoint).item(0)
        self.projectedAnchor = projectedAnchor
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, diameter / 2)
        center = projectedAnchor.geometry
        dimension = sketch.sketchDimensions.addDiameterDimension(
            circle, self._point(center.x + diameter / 2, center.y))
        dimension.parameter.value = diameter
        return circle
