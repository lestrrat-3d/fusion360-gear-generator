# Transitional module: the spur gear's stored tooth sketch, kept only for the bevel add-in, which draws its tooth
# sketch with SpurGearInvoluteToothDesignGenerator until its construction is translated from the gallery. The spur,
# helical and herringbone add-ins no longer use any of it: their builds are translated from the gallery. Delete this
# module when the bevel add-in is translated. docs/gallery-translation-goal.md owns the plan.

import collections
import math
from typing import cast
import adsk.core
import adsk.fusion
from . import decadfusion as _fx
from . import spurgear as _spur
from .spurgear import (
    PARAM_BASE_RADIUS,
    PARAM_INVOLUTE_STEPS,
    PARAM_PITCH_RADIUS,
    PARAM_ROOT_RADIUS,
    PARAM_TIP_RADIUS,
    PARAM_TOOTH_NUMBER,
)

_ToothParams = collections.namedtuple('ToothParams', ['Teeth', 'Samples'])


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
