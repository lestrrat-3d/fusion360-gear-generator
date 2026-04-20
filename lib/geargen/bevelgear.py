import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .base import *
from .misc import *
from .utilities import *
from .spurgear import SpurGearInvoluteToothDesignGenerator

# This module follows lib/geargen/bevelgear.md literally. It stops at step 2
# (Gear Profiles rectangle). Body/tooth construction is intentionally absent
# until the doc specifies more steps.

INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER_POINT = 'centerPoint'
INPUT_ID_MODULE = 'module'
INPUT_ID_DRIVING_TEETH = 'drivingTeeth'
INPUT_ID_PINION_TEETH = 'pinionTeeth'
INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'
INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'


class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        componentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component that will contain the new Bevel Gear assembly')
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        componentInput.setSelectionLimits(1)
        componentInput.addSelection(get_design().rootComponent)

        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        pointInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1)

        moduleInput = inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))
        moduleInput.isFullWidth = False
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(24))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(12))
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))


class _Val:
    __slots__ = ('value',)
    def __init__(self, v):
        self.value = v


class _VirtualSpurProxy:
    """Minimal stand-in for a Generator used by the SpurGearInvoluteToothDesign
    drawer in Section 3. It supplies the parameters the drawer reads (module,
    tooth number, pressure angle, pitch/base/root/tip circles, involute
    steps) as Python-computed values, so we don't have to register user
    parameters in the design just to draw two virtual spur tooth cross
    sections."""

    def __init__(self, module_mm: float, virtualTeeth: int,
                 pressureAngle_deg: float = 20.0):
        module_cm = to_cm(module_mm)
        pressureAngle_rad = math.radians(pressureAngle_deg)
        pitch_d = virtualTeeth * module_cm
        base_d = pitch_d * math.cos(pressureAngle_rad)
        root_d = pitch_d - 2.5 * module_cm
        tip_d = pitch_d + 2 * module_cm
        self._values = {
            'Module': module_cm,
            'ToothNumber': virtualTeeth,
            'PressureAngle': pressureAngle_rad,
            'PitchCircleDiameter': pitch_d,
            'PitchCircleRadius': pitch_d / 2,
            'BaseCircleDiameter': base_d,
            'BaseCircleRadius': base_d / 2,
            'RootCircleDiameter': root_d,
            'RootCircleRadius': root_d / 2,
            'TipCircleDiameter': tip_d,
            'TipCircleRadius': tip_d / 2,
            'InvoluteSteps': 15,
        }

    def getParameter(self, name):
        return _Val(self._values[name])


class BevelGearGenerator:
    """Creates the scaffolding for a bevel-gear pair exactly as described in
    lib/geargen/bevelgear.md. Produces a Bevel Gear component containing a
    Design sub-component with the Anchor Sketch, axial construction plane
    and Gear Profiles sketch."""

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth) = self._readInputs(inputs)

        # Pitch diameters: module * teeth (in millimeters). Convert to cm for
        # Fusion's internal distance units when we use them in sketch math.
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        # Bevel Gear component under the user-chosen parent.
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        bevelComponent = self.bevelOccurrence.component

        # Design component under the Bevel Gear component. Shared sketches
        # and construction geometry live here; the two actual gear bodies
        # will become siblings of this component in later steps.
        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        anchorLine = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)

        self._buildGearProfiles(
            designComponent, targetPlane, anchorLine, centerPoint,
            module, drivingPitchDiameter_cm, pinionPitchDiameter_cm)

    def deleteComponent(self):
        if self.bevelOccurrence is not None:
            try:
                self.bevelOccurrence.deleteMe()
            except Exception:
                futil.log('Failed to clean up Bevel Gear component')

    def _readInputs(self, inputs):
        um = self.design.unitsManager

        def evalNum(name, units):
            cinput = inputs.itemById(name)
            if cinput.classType == adsk.core.StringValueCommandInput.classType:
                expr = cinput.value
            else:
                expr = cinput.expression
            val = um.evaluateExpression(expr, units)
            if val is None:
                raise Exception(f"Invalid expression for '{name}'")
            return val

        (values, ok) = get_selection(inputs, INPUT_ID_PARENT)
        if not ok or len(values) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PARENT}' missing")
        entity = values[0]
        if entity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = entity.component
        elif entity.objectType == adsk.fusion.Component.classType():
            parentComponent = entity
        else:
            raise Exception(f'Invalid parent component type: {entity.objectType}')

        (values, ok) = get_selection(inputs, INPUT_ID_PLANE)
        if not ok or len(values) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PLANE}' missing")
        targetPlane = values[0]

        (values, ok) = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if not ok or len(values) != 1:
            raise Exception(f"Required selection '{INPUT_ID_CENTER_POINT}' missing")
        centerPoint = values[0]

        module = evalNum(INPUT_ID_MODULE, '')
        drivingTeeth = int(round(evalNum(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalNum(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Tooth counts must be at least 3')

        self._drivingBaseHeight_cm = evalNum(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalNum(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be positive numbers')

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth)

    def _buildAnchorSketch(self, component, targetPlane, centerPoint):
        """Step 1 of the doc: sketch on the target plane containing a single
        reference line centered on the projected center point."""
        sketch = component.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'

        projected = sketch.project(centerPoint)
        if projected.count == 0:
            raise Exception('Could not project center point onto target plane')
        projectedCenter = projected.item(0)

        # Reference line (length arbitrary — we pick 10 mm per the doc).
        # Placed initially so the projected center is its midpoint, then pinned
        # with geometric constraints so the solver can't move it elsewhere.
        length_cm = to_cm(10)
        cx = projectedCenter.geometry.x
        cy = projectedCenter.geometry.y
        p1 = adsk.core.Point3D.create(cx - length_cm / 2, cy, 0)
        p2 = adsk.core.Point3D.create(cx + length_cm / 2, cy, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)

        constraints = sketch.geometricConstraints
        # Projected center lies on the line ("intersection constraint" in the
        # doc — Fusion calls the point-on-curve variant Coincident).
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # Length dimension (10 mm). The doc says the exact value is not
        # critical, but the line must be dimensioned so it's fully constrained.
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint,
            anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cx, cy + to_cm(3), 0))

        return anchorLine

    def _buildGearProfiles(self, component, targetPlane, anchorLine,
                           centerPoint, module_mm, drivingPD_cm, pinionPD_cm):
        """Step 2 of the doc: axial construction plane, Gear Profiles sketch
        with the apex, the two-path construction rectangle, Apex 2, the Pitch
        Line, the two Dedendum lines and the two Root Axis lines."""

        # Axial plane via setByAngle: rotate the target plane 90° around the
        # anchor line. The result contains the anchor line and stands
        # perpendicular to the target plane.
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine,
            adsk.core.ValueInput.createByString('90 deg'),
            targetPlane)
        axialPlane = component.constructionPlanes.add(planeInput)
        axialPlane.name = 'Gear Axial Plane'

        sketch = component.sketches.add(axialPlane)
        sketch.name = 'Gear Profiles'

        projected = sketch.project(centerPoint)
        if projected.count == 0:
            raise Exception('Could not project center point into axial sketch')
        projectedCenter = projected.item(0)
        cx = projectedCenter.geometry.x
        cy = projectedCenter.geometry.y

        # Anchor the Apex to the target plane's normal (per the MD doc) rather
        # than to whichever side the sketch's local +Y happens to be on.
        # Compute the apex in world coords, then translate into sketch 2D.
        normal = get_normal(targetPlane)
        normal.normalize()
        center_world = self._pointWorldGeometry(centerPoint)
        apex_world = adsk.core.Point3D.create(
            center_world.x + drivingPD_cm * normal.x,
            center_world.y + drivingPD_cm * normal.y,
            center_world.z + drivingPD_cm * normal.z)
        apex_local = sketch.modelToSketchSpace(apex_world)

        # Sign of the "up" axis in the sketch (toward the target plane
        # normal). Every Y offset in the rectangle below is scaled by this so
        # the rectangle lies on the correct side of the anchor line regardless
        # of which way Fusion chose to orient the sketch's +Y direction.
        y_up_sign = 1.0 if apex_local.y >= cy else -1.0

        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        # Center -> Apex line. No length dimension: the apex Y stays free
        # initially; a later coincident constraint (point I with center)
        # pins it through the chained perpendicular/colinear constraints.
        centerToApex = sketch.sketchCurves.sketchLines.addByTwoPoints(
            projectedCenter.geometry, apex_local)
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addVertical(centerToApex)
        apex = centerToApex.endSketchPoint

        ay = apex_local.y

        # --- Path A (dimensioned): construction horizontal D_pd/2 (Pinion
        # Gear Shaft Axis), construction vertical P_pd/2 toward the anchor
        # line ---
        a1End = adsk.core.Point3D.create(
            apex_local.x + drivingPD_cm / 2, ay, 0)
        lineA1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex_local, a1End)
        lineA1.isConstruction = True
        constraints.addCoincident(lineA1.startSketchPoint, apex)
        constraints.addHorizontal(lineA1)
        dims.addDistanceDimension(
            lineA1.startSketchPoint,
            lineA1.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(
                (apex_local.x + a1End.x) / 2,
                ay + y_up_sign * to_cm(3), 0))

        a2End = adsk.core.Point3D.create(
            a1End.x, a1End.y - y_up_sign * pinionPD_cm / 2, 0)
        lineA2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            lineA1.endSketchPoint.geometry, a2End)
        lineA2.isConstruction = True
        constraints.addCoincident(lineA2.startSketchPoint, lineA1.endSketchPoint)
        constraints.addVertical(lineA2)
        dims.addDistanceDimension(
            lineA2.startSketchPoint,
            lineA2.endSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(
                a2End.x + to_cm(3),
                (lineA2.startSketchPoint.geometry.y + a2End.y) / 2, 0))

        # --- Path B (no length dimensions): construction vertical P_pd/2
        # (Driving Gear Shaft Axis), then construction horizontal D_pd/2.
        # Dimensions come from the coincident constraint closing the
        # rectangle below. ---
        b1End = adsk.core.Point3D.create(
            apex_local.x, ay - y_up_sign * pinionPD_cm / 2, 0)
        lineB1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex_local, b1End)
        lineB1.isConstruction = True
        constraints.addCoincident(lineB1.startSketchPoint, apex)
        constraints.addVertical(lineB1)

        b2End = adsk.core.Point3D.create(
            b1End.x + drivingPD_cm / 2, b1End.y, 0)
        lineB2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            lineB1.endSketchPoint.geometry, b2End)
        lineB2.isConstruction = True
        constraints.addCoincident(lineB2.startSketchPoint, lineB1.endSketchPoint)
        constraints.addHorizontal(lineB2)

        # Close the rectangle at Apex 2.
        constraints.addCoincident(lineA2.endSketchPoint, lineB2.endSketchPoint)
        apex2 = lineA2.endSketchPoint

        # Pitch Line: construction line from Apex to Apex 2, coincident at
        # both ends.
        pitchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, apex2.geometry)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # Dedendum lines: from Apex 2, perpendicular to the Pitch Line, length
        # 1.5 * module. "Toward the anchor line" is the perpendicular direction
        # whose sketch-Y component has sign opposite to y_up_sign. The unit
        # vector for that direction in the sketch's 2D frame is
        #   (-P_pd/2, -y_up_sign * D_pd/2) / |pitch vector|
        # and the "away from anchor line" direction is its negation.
        pitchVecLen_cm = math.sqrt(
            (drivingPD_cm / 2) ** 2 + (pinionPD_cm / 2) ** 2)
        dedendumLen_cm = to_cm(1.25 * module_mm)
        toward_ux = -(pinionPD_cm / 2) / pitchVecLen_cm
        toward_uy = (-y_up_sign * drivingPD_cm / 2) / pitchVecLen_cm

        a2x = apex2.geometry.x
        a2y = apex2.geometry.y

        drivingDedEnd = adsk.core.Point3D.create(
            a2x + toward_ux * dedendumLen_cm,
            a2y + toward_uy * dedendumLen_cm, 0)
        drivingDedendum = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex2.geometry, drivingDedEnd)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint,
            drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (a2x + drivingDedEnd.x) / 2 - to_cm(2),
                (a2y + drivingDedEnd.y) / 2 - to_cm(2), 0))

        pinionDedEnd = adsk.core.Point3D.create(
            a2x - toward_ux * dedendumLen_cm,
            a2y - toward_uy * dedendumLen_cm, 0)
        pinionDedendum = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex2.geometry, pinionDedEnd)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint,
            pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (a2x + pinionDedEnd.x) / 2 + to_cm(2),
                (a2y + pinionDedEnd.y) / 2 + to_cm(2), 0))

        # Root Axis lines: from Apex to each Dedendum endpoint. Fully fixed by
        # the coincident constraints at both ends.
        drivingRootAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, drivingDedendum.endSketchPoint.geometry)
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(
            drivingRootAxis.endSketchPoint, drivingDedendum.endSketchPoint)

        pinionRootAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, pinionDedendum.endSketchPoint.geometry)
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(
            pinionRootAxis.endSketchPoint, pinionDedendum.endSketchPoint)

        # Named endpoints per the doc: A = end of lineA1 (along Pinion Gear
        # Shaft Axis), B = end of lineB1 (along Driving Gear Shaft Axis),
        # C = end of Pinion Gear Dedendum, D = end of Driving Gear Dedendum.
        pointA = lineA1.endSketchPoint
        pointB = lineB1.endSketchPoint
        pointC = pinionDedendum.endSketchPoint
        pointD = drivingDedendum.endSketchPoint

        module_cm = to_cm(module_mm)

        # Point E: colinear extension from point A along the Pinion Gear
        # Shaft Axis, initial length = module (no dimensional constraint).
        # Final position is pinned by the C->E perpendicular constraint below
        # (which forces C.x == E.x in the sketch).
        eInit = adsk.core.Point3D.create(
            pointA.geometry.x + module_cm, pointA.geometry.y, 0)
        aToE = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointA.geometry, eInit)
        aToE.isConstruction = True
        constraints.addCoincident(aToE.startSketchPoint, pointA)
        constraints.addCollinear(aToE, lineA1)
        pointE = aToE.endSketchPoint

        cToE = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointC.geometry, pointE.geometry)
        cToE.isConstruction = True
        constraints.addCoincident(cToE.startSketchPoint, pointC)
        constraints.addCoincident(cToE.endSketchPoint, pointE)
        constraints.addPerpendicular(aToE, cToE)

        # Point F: colinear extension from point B along the Driving Gear
        # Shaft Axis, initial length = module. Extending in the same sense as
        # Apex->B (toward the anchor line), which is -y_up_sign direction.
        fInit = adsk.core.Point3D.create(
            pointB.geometry.x,
            pointB.geometry.y - y_up_sign * module_cm, 0)
        bToF = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointB.geometry, fInit)
        bToF.isConstruction = True
        constraints.addCoincident(bToF.startSketchPoint, pointB)
        constraints.addCollinear(bToF, lineB1)
        pointF = bToF.endSketchPoint

        dToF = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointD.geometry, pointF.geometry)
        dToF.isConstruction = True
        constraints.addCoincident(dToF.startSketchPoint, pointD)
        constraints.addCoincident(dToF.endSketchPoint, pointF)
        constraints.addPerpendicular(bToF, dToF)

        # Point G: colinear extension from E along the Pinion Gear Shaft Axis
        # (same horizontal line as A->E), initial length = module, no
        # dimensional constraint.
        gInit = adsk.core.Point3D.create(
            pointE.geometry.x + module_cm, pointE.geometry.y, 0)
        eToG = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointE.geometry, gInit)
        eToG.isConstruction = True
        constraints.addCoincident(eToG.startSketchPoint, pointE)
        constraints.addCollinear(eToG, aToE)
        pointG = eToG.endSketchPoint

        # Point H: construction line from C of initial length = module (no
        # dimensional constraint). Colinear with the Pinion Gear Dedendum
        # (Apex2->C) so H extends the dedendum line past C.
        pinionDedDx = pointC.geometry.x - apex2.geometry.x
        pinionDedDy = pointC.geometry.y - apex2.geometry.y
        pinionDedLen = math.sqrt(pinionDedDx ** 2 + pinionDedDy ** 2)
        hInit = adsk.core.Point3D.create(
            pointC.geometry.x + pinionDedDx / pinionDedLen * module_cm,
            pointC.geometry.y + pinionDedDy / pinionDedLen * module_cm, 0)
        cToH = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointC.geometry, hInit)
        constraints.addCoincident(cToH.startSketchPoint, pointC)
        constraints.addCollinear(cToH, pinionDedendum)
        pointH = cToH.endSketchPoint

        gToH = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointG.geometry, pointH.geometry)
        constraints.addCoincident(gToH.startSketchPoint, pointG)
        constraints.addCoincident(gToH.endSketchPoint, pointH)
        constraints.addPerpendicular(eToG, gToH)

        # Point I: colinear extension from F along the Driving Gear Shaft
        # Axis (same vertical line as B->F), initial length = module, no
        # dimensional constraint.
        iInit = adsk.core.Point3D.create(
            pointF.geometry.x,
            pointF.geometry.y - y_up_sign * module_cm, 0)
        fToI = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointF.geometry, iInit)
        fToI.isConstruction = True
        constraints.addCoincident(fToI.startSketchPoint, pointF)
        constraints.addCollinear(fToI, bToF)
        pointI = fToI.endSketchPoint

        # Point J: construction line from D of initial length = module, no
        # dimensional constraint. Colinear with the Driving Gear Dedendum
        # (Apex2->D) so J extends the dedendum line past D.
        drivingDedDx = pointD.geometry.x - apex2.geometry.x
        drivingDedDy = pointD.geometry.y - apex2.geometry.y
        drivingDedLen = math.sqrt(drivingDedDx ** 2 + drivingDedDy ** 2)
        jInit = adsk.core.Point3D.create(
            pointD.geometry.x + drivingDedDx / drivingDedLen * module_cm,
            pointD.geometry.y + drivingDedDy / drivingDedLen * module_cm, 0)
        dToJ = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointD.geometry, jInit)
        constraints.addCoincident(dToJ.startSketchPoint, pointD)
        constraints.addCollinear(dToJ, drivingDedendum)
        pointJ = dToJ.endSketchPoint

        iToJ = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointI.geometry, pointJ.geometry)
        constraints.addCoincident(iToJ.startSketchPoint, pointI)
        constraints.addCoincident(iToJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(fToI, iToJ)

        # Solid line from Apex to G, coincident at both ends.
        apexToG = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, pointG.geometry)
        constraints.addCoincident(apexToG.startSketchPoint, apex)
        constraints.addCoincident(apexToG.endSketchPoint, pointG)

        # Pin the rectangle vertically by making point I coincident with the
        # projected center. Point I already lies on the Driving Gear Shaft
        # Axis (x = cx) via the chain of colinear extensions, so this only
        # fixes its Y coordinate; that back-propagates through the
        # perpendicular/colinear constraints to pin apex.y.
        constraints.addCoincident(pointI, projectedCenter)

        # Resolve base heights, applying the doc's fallbacks when the user
        # leaves them unspecified (0):
        #   Driving: module * drivingTeeth / 8 == drivingPD / 8
        #   Pinion:  (resolved) Driving Base Height * pinionPD / drivingPD
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = drivingPD_cm / 8.0

        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = (
                drivingBaseHeight_cm * (pinionPD_cm / drivingPD_cm))

        # Offset dimension between lineB2 (Apex2->B) and iToJ (J->I), both
        # horizontal, equal to Driving Gear Base Height. Together with the
        # I-on-center coincident above, this pins apex.y.
        dims.addOffsetDimension(
            lineB2, iToJ,
            adsk.core.Point3D.create(
                apex.geometry.x - to_cm(5),
                (lineB2.startSketchPoint.geometry.y
                 + pointI.geometry.y) / 2, 0)
        ).parameter.value = drivingBaseHeight_cm

        # Offset dimension between lineA2 (Apex2->A) and gToH (G->H), both
        # vertical, equal to Pinion Gear Base Height.
        dims.addOffsetDimension(
            lineA2, gToH,
            adsk.core.Point3D.create(
                (lineA2.startSketchPoint.geometry.x
                 + pointG.geometry.x) / 2,
                apex.geometry.y + to_cm(5), 0)
        ).parameter.value = pinionBaseHeight_cm

        # Point K: construction line from G heading "away from Apex". Only
        # constraint is coincident at G; initial direction chosen as the
        # Apex->G unit vector so the construction points in a reasonable
        # direction when the solver runs.
        gk_dx = pointG.geometry.x - apex.geometry.x
        gk_dy = pointG.geometry.y - apex.geometry.y
        gk_len = math.sqrt(gk_dx ** 2 + gk_dy ** 2)
        if gk_len == 0:
            gk_ux, gk_uy = 1.0, 0.0
        else:
            gk_ux, gk_uy = gk_dx / gk_len, gk_dy / gk_len
        kInit = adsk.core.Point3D.create(
            pointG.geometry.x + gk_ux * module_cm,
            pointG.geometry.y + gk_uy * module_cm, 0)
        gToK = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointG.geometry, kInit)
        gToK.isConstruction = True
        constraints.addCoincident(gToK.startSketchPoint, pointG)
        pointK = gToK.endSketchPoint
        # Doc says G->K colinear with Apex->A (lineA1). Using point-on-line
        # coincident on the far endpoint K instead of addCollinear avoids the
        # over-constraint error Fusion reports when the chain lineA1 ->
        # aToE -> eToG already has pointG lying on lineA1's infinite line.
        constraints.addCoincident(pointK, lineA1)
        # Doc also says H->K colinear with Apex2->C (pinionDedendum). Point H
        # is already on that line's extension via cToH, so constraining K to
        # the same infinite line forces hToK (added below) to be colinear.
        constraints.addCoincident(pointK, pinionDedendum)

        # Construction line C->K, coincident at both ends. C and K both lie
        # on pinionDedendum's infinite line (C as its endpoint, K via the
        # addCoincident above), so this line runs along Apex2->C as the doc
        # requires.
        cToK = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointC.geometry, pointK.geometry)
        cToK.isConstruction = True
        constraints.addCoincident(cToK.startSketchPoint, pointC)
        constraints.addCoincident(cToK.endSketchPoint, pointK)

        # Point L: mirror of K on the driving side. Construction line from I
        # heading away from Apex.
        il_dx = pointI.geometry.x - apex.geometry.x
        il_dy = pointI.geometry.y - apex.geometry.y
        il_len = math.sqrt(il_dx ** 2 + il_dy ** 2)
        if il_len == 0:
            il_ux, il_uy = 0.0, -y_up_sign
        else:
            il_ux, il_uy = il_dx / il_len, il_dy / il_len
        lInit = adsk.core.Point3D.create(
            pointI.geometry.x + il_ux * module_cm,
            pointI.geometry.y + il_uy * module_cm, 0)
        iToL = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointI.geometry, lInit)
        iToL.isConstruction = True
        constraints.addCoincident(iToL.startSketchPoint, pointI)
        pointL = iToL.endSketchPoint
        # Doc says I->L colinear with Apex->B (lineB1). Same reasoning as for
        # G->K / lineA1 above: use point-on-line on L to avoid over-constraint.
        constraints.addCoincident(pointL, lineB1)
        # Doc also says J->L (the doc's I->L typo) colinear with Apex2->D
        # (drivingDedendum). Point J is on that line's extension via dToJ, so
        # constraining L here forces jToL (below) to be colinear.
        constraints.addCoincident(pointL, drivingDedendum)

        # Construction line D->L, coincident at both ends. Mirror of C->K.
        dToL = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointD.geometry, pointL.geometry)
        dToL.isConstruction = True
        constraints.addCoincident(dToL.startSketchPoint, pointD)
        constraints.addCoincident(dToL.endSketchPoint, pointL)

        # ----- Section 3: Gear Tooth Profiles -----
        # Virtual pitch radii per the doc: length(Apex2->K) for the pinion and
        # length(Apex2->L) for the driving gear. Computed from the known
        # rectangle geometry rather than measured from the sketch: for a pair
        # of bevel gears with horizontal (pinion) and vertical (driving) shaft
        # axes and 90 degree shaft angle, cos(pitchConeAngle_pinion) =
        # drivingPD / hypot and cos(pitchConeAngle_driving) = pinionPD / hypot,
        # where hypot = sqrt(drivingPD^2 + pinionPD^2).
        hypot_cm = math.sqrt(drivingPD_cm ** 2 + pinionPD_cm ** 2)
        pinionVirtualPitchRadius_cm = (
            pinionPD_cm * hypot_cm / (2 * drivingPD_cm))
        drivingVirtualPitchRadius_cm = (
            drivingPD_cm * hypot_cm / (2 * pinionPD_cm))
        pinionVirtualTeeth = int(math.ceil(
            2 * pinionVirtualPitchRadius_cm / module_cm))
        drivingVirtualTeeth = int(math.ceil(
            2 * drivingVirtualPitchRadius_cm / module_cm))

        self._buildVirtualSpurProfile(
            component=component,
            referencePlane=axialPlane,
            lineToInclude=cToK,
            anchorPoint=pointK,
            module_mm=module_mm,
            virtualTeeth=pinionVirtualTeeth,
            name='Pinion Tooth Profile')

        self._buildVirtualSpurProfile(
            component=component,
            referencePlane=axialPlane,
            lineToInclude=dToL,
            anchorPoint=pointL,
            module_mm=module_mm,
            virtualTeeth=drivingVirtualTeeth,
            name='Driving Tooth Profile')

    def _buildVirtualSpurProfile(self, component, referencePlane, lineToInclude,
                                  anchorPoint, module_mm, virtualTeeth, name):
        """Create a plane perpendicular to the Gear Profiles sketch plane
        that includes the given line, and draw a virtual spur gear tooth
        profile on it centered at anchorPoint."""
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByAngle(
            lineToInclude,
            adsk.core.ValueInput.createByString('90 deg'),
            referencePlane)
        toothPlane = component.constructionPlanes.add(planeInput)
        toothPlane.name = f'{name} Plane'

        sketch = component.sketches.add(toothPlane)
        sketch.name = name

        proxy = _VirtualSpurProxy(
            module_mm=module_mm,
            virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        drawer.draw(anchorPoint)

    def _pointWorldGeometry(self, point) -> adsk.core.Point3D:
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        if point.objectType == adsk.fusion.ConstructionPoint.classType():
            return point.geometry
        raise Exception(f'Unsupported center point type: {point.objectType}')
