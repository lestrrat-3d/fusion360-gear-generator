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
        self.selectedAnchor = None
        self.toolsSketch = None
        self.boreSketch = None
        self._lastToothEmbedded = False
        self._createdPlane = False

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
            self.getParameter(PARAM_THICKNESS).expression,
        )

    def processInputs(self, inputs: adsk.core.CommandInputs):
        parentSelections = get_selection(inputs, INPUT_ID_PARENT)
        planeSelections = get_selection(inputs, INPUT_ID_PLANE)
        anchorSelections = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(parentSelections) != 1 or len(planeSelections) != 1 or len(anchorSelections) != 1:
            raise Exception('Spur gear requires one parent, one target plane, and one anchor point')
        selectedParent = parentSelections[0]
        if isinstance(selectedParent, adsk.fusion.Occurrence):
            self.parentComponent = selectedParent.component
        else:
            self.parentComponent = selectedParent
        self.plane = planeSelections[0]
        self.selectedAnchor = anchorSelections[0]
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
        component = self.getComponent()
        component.name = self.generateName()
        if not isinstance(self.plane, adsk.fusion.ConstructionPlane):
            planes: adsk.fusion.ConstructionPlanes = component.constructionPlanes
            planeInput = planes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = planes.add(planeInput)
            self._createdPlane = True
        ctx = self.newContext()
        ctx.plane = self.plane
        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.chamferTeeth(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx: SpurGearGenerationContext):
        self.toolsSketch = self.createSketchObject('Tools', ctx.plane)
        self.toolsSketch.isVisible = True
        ctx.anchorPoint = self.toolsSketch.project(self.selectedAnchor).item(0)
        planes: adsk.fusion.ConstructionPlanes = self.getComponent().constructionPlanes
        endInput = planes.createInput()
        thicknessCm = self.getParameter(PARAM_THICKNESS).value
        endInput.setByOffset(ctx.plane, adsk.core.ValueInput.createByReal(thicknessCm))
        ctx.extrusionEndPlane = planes.add(endInput)
        ctx.extrusionEndPlane.name = 'Extrusion End Plane'

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
        sketch = self.createSketchObject('Gear Profile', ctx.plane)
        sketch.isVisible = True
        ctx.gearProfileSketch = sketch
        toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        toothGen.draw(ctx.anchorPoint)
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded
        futil.log('Gear Profile isFullyConstrained: {}'.format(sketch.isFullyConstrained))

    def buildTooth(self, ctx: SpurGearGenerationContext):
        profile = find_profile_by_curve_counts(
            ctx.gearProfileSketch, nurbs=2, arcs=2,
            lines=0 if ctx.toothProfileIsEmbedded else 2,
        )
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        feature = extrudes.add(extrudeInput)
        feature.name = 'Extrude tooth'
        if feature.bodies.count == 0:
            raise Exception('Spur gear tooth extrusion produced no body')
        ctx.toothBody = feature.bodies.item(0)

    def buildBody(self, ctx: SpurGearGenerationContext):
        profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        feature = extrudes.add(extrudeInput)
        feature.name = 'Extrude body'
        if feature.bodies.count == 0:
            raise Exception('Spur gear root extrusion produced no body')
        ctx.gearBody = feature.bodies.item(0)
        ctx.gearBody.name = 'Gear Body'
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        axes: adsk.fusion.ConstructionAxes = self.getComponent().constructionAxes
        for face in ctx.gearBody.faces:
            if isinstance(face.geometry, adsk.core.Cylinder) and ctx.centerAxis is None:
                axisInput = axes.createInput()
                axisInput.setByCircularFace(face)
                ctx.centerAxis = axes.add(axisInput)
                ctx.centerAxis.name = 'Gear Center'
                ctx.centerAxis.isLightBulbOn = False
            if isinstance(face.geometry, adsk.core.Plane):
                if sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry):
                    ctx.extrusionExtent = face
        if ctx.centerAxis is None or ctx.extrusionExtent is None:
            raise Exception('Spur gear root body lacks cylindrical face or far planar cap')

    def patternTeeth(self, ctx: SpurGearGenerationContext):
        seeds: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seeds.add(ctx.toothBody)
        patterns: adsk.fusion.CircularPatternFeatures = self.getComponent().features.circularPatternFeatures
        patternInput = patterns.createInput(seeds, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(self.getParameter(PARAM_TOOTH_NUMBER).value)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)
        copiedBodies: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            copiedBodies.add(pattern.bodies.item(i))
        if copiedBodies.count == 0:
            raise Exception('Spur gear tooth pattern produced no bodies')
        combines: adsk.fusion.CombineFeatures = self.getComponent().features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, copiedBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

    def createFillets(self, ctx: SpurGearGenerationContext):
        radius = self.getParameter(PARAM_FILLET_RADIUS).value
        if radius <= 0:
            return
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        normal = get_normal(ctx.plane)
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = {}
        for face in ctx.gearBody.faces:
            if not isinstance(face.geometry, adsk.core.Cylinder):
                continue
            if abs(face.geometry.radius - rootRadius) > 0.0001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                start = edge.startVertex.geometry
                end = edge.endVertex.geometry
                direction: adsk.core.Vector3D = adsk.core.Vector3D.create(
                    end.x - start.x, end.y - start.y, end.z - start.z,
                )
                magnitude = math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)
                if magnitude == 0:
                    continue
                axial = abs((direction.x * normal.x + direction.y * normal.y + direction.z * normal.z) / magnitude)
                if abs(axial - 1) < 0.01 and edge.tempId not in seen:
                    seen[edge.tempId] = True
                    edges.add(edge)
        if edges.count == 0:
            return
        fillets: adsk.fusion.FilletFeatures = self.getComponent().features.filletFeatures
        filletInput: adsk.fusion.FilletFeatureInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(radius), False,
        )
        fillets.add(filletInput)

    def buildBore(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return
        self.boreSketch = self.createSketchObject('Bore Profile', ctx.plane)
        self.boreSketch.isVisible = True
        toothGen = SpurGearInvoluteToothDesignGenerator(self.boreSketch, self)
        projectedAnchor = toothGen.drawBore(ctx.anchorPoint, boreDiameter)
        self.boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)
        if not self.boreSketch.isFullyConstrained:
            raise Exception('Spur gear Bore Profile sketch is not fully constrained')
        if self.boreSketch.profiles.count != 1:
            raise Exception('Spur gear Bore Profile expected one profile, found {}'.format(
                self.boreSketch.profiles.count,
            ))
        profile = self.boreSketch.profiles.item(0)
        extrudes: adsk.fusion.ExtrudeFeatures = self.getComponent().features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    def chamferTeeth(self, ctx: SpurGearGenerationContext):
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        chamferCm = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferCm == 0:
            return
        plane = ctx.gearProfileSketch.referencePlane.geometry
        boreRadius = self.getParameter(PARAM_BORE_DIAMETER).value / 2
        edges: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        seen = {}
        endCaps = 0
        for face in ctx.gearBody.faces:
            if not isinstance(face.geometry, adsk.core.Plane):
                continue
            if not plane.isParallelToPlane(face.geometry):
                continue
            endCaps += 1
            for edge in face.edges:
                if edge.tempId in seen:
                    continue
                if boreRadius > 0 and edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                    if abs(edge.geometry.radius - boreRadius) < 0.001:
                        continue
                seen[edge.tempId] = True
                edges.add(edge)
        if endCaps == 0 or edges.count == 0:
            raise Exception('Spur gear chamfer has no end-cap face or usable edge')
        chamfers: adsk.fusion.ChamferFeatures = self.getComponent().features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferCm), False,
        )
        chamfers.add(chamferInput)

    def cleanup(self, ctx: SpurGearGenerationContext):
        if self._createdPlane and self.plane is not None:
            self.plane.isLightBulbOn = False
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        for sketch in (self.toolsSketch, ctx.gearProfileSketch, self.boreSketch):
            if sketch is not None:
                sketch.isVisible = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch: adsk.fusion.Sketch, parent: SpurGearGenerator, angle: float = 0):
        self.sketch = sketch
        self.parent = parent
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        self.toothAngle = angle
        self.circles = {}

    def getParameter(self, name: str):
        return self.parent.getParameter(name)

    def getParameterValue(self, name: str) -> float:
        return float(self.getParameter(name).value)

    def calculateInvolutePoint(self, base: float, radius: float):
        if radius < base:
            return None
        alpha = math.acos(base / radius)
        t = math.tan(alpha)
        x = base * (math.cos(t) + t * math.sin(t))
        y = base * (math.sin(t) - t * math.cos(t))
        return x, y

    def draw(self, anchorPoint: adsk.fusion.SketchPoint, angle: float = 0):
        self.drawCircles()
        angularDimension = self.drawTooth(angle)
        projectedAnchor = self.sketch.project(anchorPoint).item(0)
        self.sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)
        if angle != 0:
            angularDimension.parameter.value = angle

    def drawCircles(self):
        radiusByName = (
            ('Root Circle', PARAM_ROOT_RADIUS, PARAM_ROOT_DIAMETER),
            ('Tip Circle', PARAM_TIP_RADIUS, PARAM_TIP_DIAMETER),
            ('Base Circle', PARAM_BASE_RADIUS, PARAM_BASE_DIAMETER),
            ('Pitch Circle', PARAM_PITCH_RADIUS, PARAM_PITCH_DIAMETER),
        )
        size = self.getParameterValue(PARAM_TIP_RADIUS) - self.getParameterValue(PARAM_ROOT_RADIUS)
        for index, (name, radiusName, diameterName) in enumerate(radiusByName):
            radius = self.getParameterValue(radiusName)
            diameter = self.getParameterValue(diameterName)
            circle = self.sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radius)
            circle.isConstruction = index != 0
            self.circles[name] = circle
            textPoint = adsk.core.Point3D.create(radius / math.sqrt(2), radius / math.sqrt(2), 0)
            diameterDimension = self.sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameterDimension.parameter.value = diameter
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = self.sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0,
            )
            self.sketch.sketchTexts.add(textInput)

    def _dimensionDistance(self, first, second, orientation, textPoint, magnitude):
        dimension = self.sketch.sketchDimensions.addDistanceDimension(
            first, second, orientation, textPoint,
        )
        dimension.parameter.value = abs(magnitude)
        return dimension

    def _drawFlankToRoot(self, origin, flankStart, flankPoint):
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        length = math.hypot(flankPoint[0], flankPoint[1])
        rootX = flankPoint[0] * rootRadius / length
        rootY = flankPoint[1] * rootRadius / length
        rootEnd = self.sketch.sketchPoints.add(adsk.core.Point3D.create(rootX, rootY, 0))
        self.sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flankStart)
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        horizontalDimension = self.sketch.sketchDimensions.addDistanceDimension(
            origin, rootEnd, horizontal, adsk.core.Point3D.create(rootX / 2, rootY, 0),
        )
        horizontalDimension.parameter.value = abs(rootX)
        verticalDimension = self.sketch.sketchDimensions.addDistanceDimension(
            origin, rootEnd, vertical, adsk.core.Point3D.create(rootX, rootY / 2, 0),
        )
        verticalDimension.parameter.value = abs(rootY)

    def drawTooth(self, angle: float = 0):
        base = self.getParameterValue(PARAM_BASE_RADIUS)
        tip = self.getParameterValue(PARAM_TIP_RADIUS)
        pitch = self.getParameterValue(PARAM_PITCH_RADIUS)
        root = self.getParameterValue(PARAM_ROOT_RADIUS)
        toothCount = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise Exception('Spur gear involute requires at least two sample points')
        pitchPoint = self.calculateInvolutePoint(base, pitch)
        if pitchPoint is None:
            raise Exception('Spur gear pitch circle lies inside its base circle')
        crossingRotation = math.pi / (2 * toothCount) - math.atan2(-pitchPoint[1], pitchPoint[0])

        def rotate(x, y, theta):
            return (x * math.cos(theta) - y * math.sin(theta),
                    x * math.sin(theta) + y * math.cos(theta))

        leftCoords = []
        rightCoords = []
        for i in range(steps):
            radius = base + (tip - base) * i / (steps - 1)
            point = self.calculateInvolutePoint(base, radius)
            if point is None:
                raise Exception('Spur gear involute sample lies inside its base circle')
            left = rotate(point[0], -point[1], crossingRotation)
            right = (left[0], -left[1])
            leftCoords.append(rotate(left[0], left[1], angle))
            rightCoords.append(rotate(right[0], right[1], angle))

        leftFit: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        rightFit: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        for x, y in leftCoords:
            leftFit.add(self.sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, 0)))
        for x, y in rightCoords:
            rightFit.add(self.sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, 0)))
        leftFlank = self.sketch.sketchCurves.sketchFittedSplines.add(leftFit)
        rightFlank = self.sketch.sketchCurves.sketchFittedSplines.add(rightFit)

        topX, topY = tip * math.cos(angle), tip * math.sin(angle)
        toothTop = self.sketch.sketchPoints.add(adsk.core.Point3D.create(topX, topY, 0))
        self.sketch.geometricConstraints.addCoincident(toothTop, self.circles['Tip Circle'])
        arc = self.sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            self.anchorPoint, rightFlank.endSketchPoint, leftFlank.endSketchPoint,
        )
        self.sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)

        spine = self.sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTop)
        spine.isConstruction = True
        refEnd = self.sketch.sketchPoints.add(adsk.core.Point3D.create(tip, 0, 0))
        horizontal = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        vertical = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        self._dimensionDistance(
            self.anchorPoint, refEnd, horizontal, adsk.core.Point3D.create(tip / 2, 0, 0), tip,
        )
        self._dimensionDistance(
            self.anchorPoint, refEnd, vertical, adsk.core.Point3D.create(tip, 0, 0), 0,
        )
        reference = self.sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, refEnd)
        reference.isConstruction = True
        angularDimension = self.sketch.sketchDimensions.addAngularDimension(
            reference, spine,
            adsk.core.Point3D.create(tip * math.cos(angle / 2), tip * math.sin(angle / 2), 0),
        )

        previous = self.anchorPoint
        previousProjection = 0.0
        acrossVertical = abs(math.cos(angle)) >= abs(math.sin(angle))
        for i in range(steps):
            leftPoint = leftFlank.fitPoints.item(i)
            rightPoint = rightFlank.fitPoints.item(i)
            rib = self.sketch.sketchCurves.sketchLines.addByTwoPoints(leftPoint, rightPoint)
            rib.isConstruction = True
            if acrossVertical:
                self._dimensionDistance(
                    leftPoint, rightPoint, vertical,
                    adsk.core.Point3D.create(leftCoords[i][0], rightCoords[i][1], 0),
                    rightCoords[i][1] - leftCoords[i][1],
                )
            else:
                self._dimensionDistance(
                    leftPoint, rightPoint, horizontal,
                    adsk.core.Point3D.create(rightCoords[i][0], leftCoords[i][1], 0),
                    rightCoords[i][0] - leftCoords[i][0],
                )
            projection = leftCoords[i][0] * math.cos(angle) + leftCoords[i][1] * math.sin(angle)
            midpoint = self.sketch.sketchPoints.add(adsk.core.Point3D.create(
                projection * math.cos(angle), projection * math.sin(angle), 0,
            ))
            self.sketch.geometricConstraints.addCoincident(midpoint, spine)
            self.sketch.geometricConstraints.addMidPoint(midpoint, rib)
            if i != steps - 1:
                self.sketch.geometricConstraints.addPerpendicular(spine, rib)
            if acrossVertical:
                along = horizontal
                magnitude = (projection - previousProjection) * math.cos(angle)
            else:
                along = vertical
                magnitude = (projection - previousProjection) * math.sin(angle)
            self._dimensionDistance(
                previous, midpoint, along,
                adsk.core.Point3D.create(midpoint.geometry.x, midpoint.geometry.y, 0), magnitude,
            )
            previous = midpoint
            previousProjection = projection

        embedded = base < root
        self.parent._lastToothEmbedded = embedded
        if not embedded:
            self._drawFlankToRoot(self.anchorPoint, leftFlank.startSketchPoint, leftCoords[0])
            self._drawFlankToRoot(self.anchorPoint, rightFlank.startSketchPoint, rightCoords[0])
        return angularDimension

    def drawBore(self, anchorPoint: adsk.fusion.SketchPoint, boreDiameterCm: float):
        projectedAnchor = self.sketch.project(anchorPoint).item(0)
        circle = self.sketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, boreDiameterCm / 2,
        )
        diameterDimension = self.sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameterCm / 2, 0, 0),
        )
        diameterDimension.parameter.value = boreDiameterCm
        return projectedAnchor
