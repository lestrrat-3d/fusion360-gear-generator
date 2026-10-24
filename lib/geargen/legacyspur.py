# Transitional module: the spur gear's stored Fusion construction, kept only for the add-ins that have not been
# translated from their gallery models yet. HelicalGearGenerator and HerringboneGearGenerator build on
# StoredSpurGearGenerator, and the bevel add-in draws its tooth sketch with SpurGearInvoluteToothDesignGenerator.
# The spur itself no longer uses any of it: its build is translated from the gallery (spurgear.py). Delete this
# module when the last of those add-ins is translated. docs/gallery-translation-goal.md owns the plan.

import collections
import math
from typing import cast
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil
from .utilities import get_normal, find_profile_by_curve_counts
from . import solids
from . import decadfusion as _fx
from . import spurgear as _spur
from .spurgear import *  # noqa: F401,F403 -- the PARAM_ and INPUT_ID_ constants and the spur classes
from .spurgear import SpurGearGenerationContext, SpurGearGenerator

_ToothParams = collections.namedtuple('ToothParams', ['Teeth', 'Samples'])


class StoredSpurGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.extrusionExtent = adsk.fusion.BRepFace.cast(None)
        self.toothProfileIsEmbedded = False


class StoredSpurGearGenerator(SpurGearGenerator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self._toolsSketch = adsk.fusion.Sketch.cast(None)
        self._boreSketch = adsk.fusion.Sketch.cast(None)
        self._lastToothEmbedded = False

    def newContext(self) -> StoredSpurGenerationContext:
        return StoredSpurGenerationContext()

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
            self.plane.name = 'Target Plane'
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
        solids.name_feature(extrude, 'Extrude tooth')
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
        solids.name_feature(pattern, 'Tooth Pattern')
        tools = adsk.core.ObjectCollection.create()
        for patternBody in pattern.bodies:
            tools.add(patternBody)
        if tools.count == 0:
            raise RuntimeError('Pattern teeth: no bodies to combine')
        combines: adsk.fusion.CombineFeatures = self.getComponent().features.combineFeatures
        combineInput: adsk.fusion.CombineFeatureInput = combines.createInput(ctx.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        teethJoin = combines.add(combineInput)
        solids.name_feature(teethJoin, 'Teeth Join')

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
        rootFillet = fillets.add(filletInput)
        solids.name_feature(rootFillet, 'Root Fillet')

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
        boreCut = extrudes.add(extrudeInput)
        solids.name_feature(boreCut, 'Bore Cut')

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
        toothChamfer = chamfers.add(chamferInput)
        solids.name_feature(toothChamfer, 'Tooth Chamfer')

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
        self._rootCircle = adsk.fusion.SketchCircle.cast(None)
        self._angleDimension = adsk.fusion.SketchAngularDimension.cast(None)
        self._boreProjectedAnchor = adsk.fusion.SketchPoint.cast(None)

    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name) -> float:
        return self.getParameter(name).value

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        sketch: adsk.fusion.Sketch = self.sketch
        projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(anchorPoint).item(0))
        if projectedAnchor is None:
            raise RuntimeError(f'{sketch.name}: anchor projection produced no SketchPoint')
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        constraints.addCoincident(self.anchorPoint, projectedAnchor)
        if angle != 0 and self._angleDimension is not None:
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
            if parameter == PARAM_ROOT_RADIUS:
                self._rootCircle = circle

    def drawTooth(self, angle=0):
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise RuntimeError('Gear Profile: InvoluteSteps must be at least two')
        if pitchRadius < baseRadius:
            raise RuntimeError('Gear Profile: pitch radius is below base radius')
        self.parent._lastToothEmbedded = baseRadius < rootRadius
        if self.parent._lastToothEmbedded:
            self.drawEmbeddedTooth(angle)
            return
        builder = _fx.Sketch(None, self.sketch, self.anchorPoint, self._rootCircle, rootRadius)
        _spur._draw_tooth_section(
            builder, _ToothParams(toothNumber, steps),
            _spur._Dimensions(pitchRadius, baseRadius, rootRadius, tipRadius, 0.0), angle)
        builder.finish()

    def drawEmbeddedTooth(self, angle=0):
        sketch: adsk.fusion.Sketch = self.sketch
        baseRadius = self.getParameterValue(PARAM_BASE_RADIUS)
        tipRadius = self.getParameterValue(PARAM_TIP_RADIUS)
        rootRadius = self.getParameterValue(PARAM_ROOT_RADIUS)
        pitchRadius = self.getParameterValue(PARAM_PITCH_RADIUS)
        toothNumber = self.getParameterValue(PARAM_TOOTH_NUMBER)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))
        if steps < 2:
            raise RuntimeError('Gear Profile: InvoluteSteps must be at least two')
        if pitchRadius < baseRadius:
            raise RuntimeError('Gear Profile: pitch radius is below base radius')
        ca, sa = math.cos(angle), math.sin(angle)
        flankLeft, flankRight = _spur._sample_flanks(
            pitchRadius, baseRadius, baseRadius, tipRadius, toothNumber, steps, angle)
        leftPoints = adsk.core.ObjectCollection.create()
        rightPoints = adsk.core.ObjectCollection.create()
        for p in flankLeft:
            leftPoints.add(adsk.core.Point3D.create(p.x, p.y, 0))
        for p in flankRight:
            rightPoints.add(adsk.core.Point3D.create(p.x, p.y, 0))
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
