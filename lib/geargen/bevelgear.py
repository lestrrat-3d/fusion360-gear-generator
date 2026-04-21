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
INPUT_ID_FACE_WIDTH = 'faceWidth'


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
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
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
            module, drivingPitchDiameter_cm, pinionPitchDiameter_cm,
            drivingTeeth=drivingTeeth, pinionTeeth=pinionTeeth,
            bevelComponent=bevelComponent,
            designOccurrence=designOccurrence)

        # Cleanup: hide all sketches, construction planes, and construction
        # axes across the Bevel Gear component tree so only the finished
        # gear bodies are visible.
        self._hideConstructionGeometry(bevelComponent)

    @staticmethod
    def _hideConstructionGeometry(rootComponent):
        visited = set()

        def walk(comp):
            if comp.entityToken in visited:
                return
            visited.add(comp.entityToken)
            for sk in comp.sketches:
                sk.isLightBulbOn = False
            for pl in comp.constructionPlanes:
                pl.isLightBulbOn = False
            for ax in comp.constructionAxes:
                ax.isLightBulbOn = False
            for occ in comp.occurrences:
                walk(occ.component)

        walk(rootComponent)

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
        self._faceWidth_cm = evalNum(INPUT_ID_FACE_WIDTH, 'mm')
        if (self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0
                or self._faceWidth_cm < 0):
            raise Exception('Base heights and face width must be positive numbers')

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
                           centerPoint, module_mm, drivingPD_cm, pinionPD_cm,
                           drivingTeeth=None, pinionTeeth=None,
                           bevelComponent=None, designOccurrence=None):
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
        centerToApex.isConstruction = True
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

        # Root Axis lines (construction): from Apex to each Dedendum endpoint.
        # Fully fixed by the coincident constraints at both ends.
        drivingRootAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, drivingDedendum.endSketchPoint.geometry)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(
            drivingRootAxis.endSketchPoint, drivingDedendum.endSketchPoint)

        pinionRootAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(
            apex.geometry, pinionDedendum.endSketchPoint.geometry)
        pinionRootAxis.isConstruction = True
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

        # Solid line from A to G, coincident at both ends.
        aToG = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointA.geometry, pointG.geometry)
        constraints.addCoincident(aToG.startSketchPoint, pointA)
        constraints.addCoincident(aToG.endSketchPoint, pointG)

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

        # Face Width resolution: user value if > 0, else Cone Distance / 6
        # where Cone Distance = hypot(drivingPD, pinionPD) per the doc's
        # Variables section.
        if self._faceWidth_cm > 0:
            faceWidth_cm = self._faceWidth_cm
        else:
            coneDistance_cm = math.sqrt(drivingPD_cm ** 2 + pinionPD_cm ** 2)
            faceWidth_cm = coneDistance_cm / 6.0

        # Doc line 115: a new line from pinionRootAxis (Apex->C) to lineA2
        # (A->Apex2), parallel to C->H. M on pinionRootAxis, N on lineA2.
        # The M-N line is offset from C->H by Face Width.
        # Initial placement: parallel to cToH direction through a midpoint.
        ch_dx = pointH.geometry.x - pointC.geometry.x
        ch_dy = pointH.geometry.y - pointC.geometry.y
        ch_len = math.sqrt(ch_dx ** 2 + ch_dy ** 2)
        ch_ux = ch_dx / ch_len
        ch_uy = ch_dy / ch_len
        m_init = adsk.core.Point3D.create(
            (apex.geometry.x + pointC.geometry.x) / 2,
            (apex.geometry.y + pointC.geometry.y) / 2, 0)
        # Slide n_init along ch direction from m_init enough to roughly reach
        # lineA2. The exact value is irrelevant for initial placement; the
        # constraints below settle it.
        approxLen = math.sqrt(
            (lineA2.startSketchPoint.geometry.x - m_init.x) ** 2
            + (lineA2.startSketchPoint.geometry.y - m_init.y) ** 2)
        n_init = adsk.core.Point3D.create(
            m_init.x + ch_ux * approxLen,
            m_init.y + ch_uy * approxLen, 0)
        mToN = sketch.sketchCurves.sketchLines.addByTwoPoints(m_init, n_init)
        pointM = mToN.startSketchPoint
        pointN = mToN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, lineA2)
        constraints.addParallel(mToN, cToH)

        dims.addOffsetDimension(
            cToH, mToN,
            adsk.core.Point3D.create(
                (m_init.x + pointC.geometry.x) / 2,
                (m_init.y + pointC.geometry.y) / 2, 0)
        ).parameter.value = faceWidth_cm

        # Doc line 113: lines from M to C and from N to A.
        mToC = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointM.geometry, pointC.geometry)
        constraints.addCoincident(mToC.startSketchPoint, pointM)
        constraints.addCoincident(mToC.endSketchPoint, pointC)

        nToA = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointN.geometry, pointA.geometry)
        constraints.addCoincident(nToA.startSketchPoint, pointN)
        constraints.addCoincident(nToA.endSketchPoint, pointA)

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

        # Doc line 123: mirror of the M/N construction for the driving side.
        # O on drivingRootAxis (Apex->D), P on lineB2 (B->Apex2), O-P parallel
        # to D->J. No dimensional constraint on this side per the doc.
        dj_dx = pointJ.geometry.x - pointD.geometry.x
        dj_dy = pointJ.geometry.y - pointD.geometry.y
        dj_len = math.sqrt(dj_dx ** 2 + dj_dy ** 2)
        dj_ux = dj_dx / dj_len
        dj_uy = dj_dy / dj_len
        o_init = adsk.core.Point3D.create(
            (apex.geometry.x + pointD.geometry.x) / 2,
            (apex.geometry.y + pointD.geometry.y) / 2, 0)
        approxLenD = math.sqrt(
            (lineB2.startSketchPoint.geometry.x - o_init.x) ** 2
            + (lineB2.startSketchPoint.geometry.y - o_init.y) ** 2)
        p_init = adsk.core.Point3D.create(
            o_init.x + dj_ux * approxLenD,
            o_init.y + dj_uy * approxLenD, 0)
        oToP = sketch.sketchCurves.sketchLines.addByTwoPoints(o_init, p_init)
        pointO = oToP.startSketchPoint
        pointP = oToP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, lineB2)
        constraints.addParallel(oToP, dToJ)

        dims.addOffsetDimension(
            dToJ, oToP,
            adsk.core.Point3D.create(
                (o_init.x + pointD.geometry.x) / 2,
                (o_init.y + pointD.geometry.y) / 2, 0)
        ).parameter.value = faceWidth_cm

        # Doc line 121: lines from O to D and from P to B (mirror of M-to-C
        # and N-to-A on the pinion side).
        oToD = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointO.geometry, pointD.geometry)
        constraints.addCoincident(oToD.startSketchPoint, pointO)
        constraints.addCoincident(oToD.endSketchPoint, pointD)

        pToB = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointP.geometry, pointB.geometry)
        constraints.addCoincident(pToB.startSketchPoint, pointP)
        constraints.addCoincident(pToB.endSketchPoint, pointB)

        # Doc line 121: additional line from B to I.
        bToI = sketch.sketchCurves.sketchLines.addByTwoPoints(
            pointB.geometry, pointI.geometry)
        constraints.addCoincident(bToI.startSketchPoint, pointB)
        constraints.addCoincident(bToI.endSketchPoint, pointI)

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
        pinionVirtualTeeth = int(math.floor(
            2 * pinionVirtualPitchRadius_cm / module_cm))
        drivingVirtualTeeth = int(math.floor(
            2 * drivingVirtualPitchRadius_cm / module_cm))

        pinionToothSketch = self._buildVirtualSpurProfile(
            component=component,
            referencePlane=axialPlane,
            lineToInclude=cToK,
            anchorPoint=pointK,
            module_mm=module_mm,
            virtualTeeth=pinionVirtualTeeth,
            name='Pinion Tooth Profile',
            rotationDegrees=180.0)

        drivingToothSketch = self._buildVirtualSpurProfile(
            component=component,
            referencePlane=axialPlane,
            lineToInclude=dToL,
            anchorPoint=pointL,
            module_mm=module_mm,
            virtualTeeth=drivingVirtualTeeth,
            name='Driving Tooth Profile',
            rotationDegrees=180.0)

        # ----- Create the Pinion Gear and Driving Gear bodies -----
        # The doc says "child of Parent Component" but the overall Component
        # Setup section describes a single outer component containing the
        # design + two gears, so we add the gear components under the Bevel
        # Gear parent that already owns the Design sub-component.
        if bevelComponent is not None:
            self._createGearBody(
                bevelComponent=bevelComponent,
                designComponent=component,
                designOccurrence=designOccurrence,
                axialPlane=axialPlane,
                name='Pinion Gear',
                profilePoints=[pointA, pointG, pointH, pointC, pointM, pointN],
                apexPoint=apex,
                toothSketch=pinionToothSketch,
                cutLines=[mToN, cToH],
                toothCount=pinionTeeth)
            self._createGearBody(
                bevelComponent=bevelComponent,
                designComponent=component,
                designOccurrence=designOccurrence,
                axialPlane=axialPlane,
                name='Driving Gear',
                profilePoints=[pointB, pointI, pointJ, pointD, pointO, pointP],
                apexPoint=apex,
                toothSketch=drivingToothSketch,
                cutLines=[oToP, dToJ],
                toothCount=drivingTeeth,
                meshingOffsetDegrees=(180.0 / drivingTeeth
                                      if drivingTeeth else None))

    def _createGearBody(self, bevelComponent, designComponent, axialPlane,
                         name, profilePoints, apexPoint=None, toothSketch=None,
                         cutLines=None, toothCount=None,
                         meshingOffsetDegrees=None,
                         designOccurrence=None):
        # Make sure we're operating in the Design occurrence's context for
        # the subsequent feature calls; addNewComponent above can shift the
        # active context, which then makes Fusion fail to resolve sketch
        # entities that live in Design (Path.create in particular needs an
        # active context whose component matches the entity owner).
        """Build a bevel gear sub-component. Despite the doc's
        "operations within this new component" wording, every step here runs
        in `designComponent` — Fusion's API rejects cross-sibling references
        (sketches.add, project) even when the target component is activated
        and the entities are wrapped in createForAssemblyContext proxies.
        After all features run in Design, the resulting bodies are moved
        into the gear sub-component."""
        gearOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOcc.component.name = name

        if designOccurrence is not None:
            try:
                designOccurrence.activate()
            except Exception:
                pass

        # Step 1: revolve the hexagonal profile in Design.
        profileSketch = designComponent.sketches.add(axialPlane)
        profileSketch.name = f'{name} Profile'

        projected = []
        for pt in profilePoints:
            projected.append(profileSketch.project(pt).item(0))

        lines = profileSketch.sketchCurves.sketchLines
        sketchLines = []
        for i in range(len(projected)):
            a = projected[i]
            b = projected[(i + 1) % len(projected)]
            sketchLines.append(lines.addByTwoPoints(a.geometry, b.geometry))

        expectedCurves = len(sketchLines)
        profile = None
        for p in profileSketch.profiles:
            for loop in p.profileLoops:
                if loop.profileCurves.count == expectedCurves:
                    profile = p
                    break
            if profile is not None:
                break
        if profile is None:
            raise Exception(f'{name}: could not find revolve profile')

        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            profile,
            sketchLines[0],
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False,
            adsk.core.ValueInput.createByReal(2 * math.pi))
        revolveResult = revolves.add(revolveInput)
        revolveResult.name = f'{name} Revolve'

        frustumBodies = []
        for i in range(revolveResult.bodies.count):
            b = revolveResult.bodies.item(i)
            b.name = f'{name} Body'
            frustumBodies.append(b)

        # Step 2: loft tooth from apex to tooth profile as a separate body.
        toothBodies = []
        if apexPoint is not None and toothSketch is not None:
            toothProfile = self._findSpurToothProfile(toothSketch)
            if toothProfile is not None:
                lofts = designComponent.features.loftFeatures
                loftInput = lofts.createInput(
                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                loftInput.loftSections.add(apexPoint)
                loftInput.loftSections.add(toothProfile)
                try:
                    loftResult = lofts.add(loftInput)
                    loftResult.name = f'{name} Tooth Loft'
                    for i in range(loftResult.bodies.count):
                        b = loftResult.bodies.item(i)
                        b.name = f'{name} Tooth Body'
                        toothBodies.append(b)
                except Exception as e:
                    futil.log(f'{name}: tooth loft failed ({e})')

        # Step 3: two sequential conical-surface cuts on the tooth body, then
        # drop the apex-side piece and the smaller of the two remaining
        # pieces. The conical surfaces needed for the cuts are already faces
        # of the frustum body (the cut edges M->N and C->H, or O->P and
        # D->J, are sides of the revolved hexagon), so we locate them by
        # heuristic instead of building separate surfaces -- building a
        # fresh revolved surface from a cross-component sketch line trips
        # Fusion's path-resolution checks.
        futil.log(
            f'{name}: pre-cut state: cutLines={len(cutLines) if cutLines else 0}, '
            f'toothBodies={len(toothBodies)}, frustumBodies={len(frustumBodies)}',
            force_console=True)
        if (cutLines and toothBodies and frustumBodies):
            toothBodies = self._applyConicalCut(
                designComponent, toothBodies, frustumBodies[0],
                cutLines, apexPoint, name)

        # Step 3.5: circular-pattern the surviving tooth piece around the
        # shaft axis. sketchLines[0] is the hexagon's first edge (A->G for
        # pinion, B->I for driving), colinear with the gear's shaft axis,
        # so it serves as the pattern axis.
        if toothBodies and toothCount and toothCount > 1 and sketchLines:
            patternAxis = sketchLines[0]
            patternBodies = adsk.core.ObjectCollection.create()
            for b in toothBodies:
                if b.isValid:
                    patternBodies.add(b)
            if patternBodies.count > 0:
                patterns = designComponent.features.circularPatternFeatures
                patternInput = patterns.createInput(patternBodies, patternAxis)
                patternInput.quantity = adsk.core.ValueInput.createByReal(
                    float(toothCount))
                patternInput.totalAngle = adsk.core.ValueInput.createByString(
                    '360 deg')
                patternInput.isSymmetric = False
                patternResult = patterns.add(patternInput)
                patternResult.name = f'{name} Tooth Pattern'
                # Collect the patterned bodies (they include the original
                # plus N-1 new copies).
                allTeeth = list(toothBodies)
                for i in range(patternResult.bodies.count):
                    pb = patternResult.bodies.item(i)
                    if pb.isValid and pb not in allTeeth:
                        allTeeth.append(pb)
                toothBodies = allTeeth
                futil.log(
                    f'{name}: pattern produced {len(toothBodies)} tooth '
                    f'body(ies) total (target count {toothCount})',
                    force_console=True)

        # Step 4: join all (patterned) tooth pieces into the gear body in a
        # single Combine-Join.
        if toothBodies and frustumBodies:
            target = frustumBodies[0]
            tools = adsk.core.ObjectCollection.create()
            for b in toothBodies:
                if b.isValid:
                    tools.add(b)
            if tools.count > 0:
                combines = designComponent.features.combineFeatures
                combineInput = combines.createInput(target, tools)
                combineInput.operation = (
                    adsk.fusion.FeatureOperations.JoinFeatureOperation)
                try:
                    combines.add(combineInput)
                except Exception as e:
                    futil.log(
                        f'{name}: could not join tooth into body ({e})')

        # Step 5 (driving gear only): rotate the finished body by half its
        # tooth pitch around the shaft axis so the teeth appear to mesh
        # with the pinion. Done BEFORE moveToComponent so we stay in the
        # Design component's context (where we can add construction axes
        # reliably) rather than the gear sub-component (where
        # `constructionAxes.add` fails with "Environment is not
        # supported"). Functionally identical to rotating afterwards.
        # MoveFeatures.defineAsRotate requires a linear edge or
        # construction axis -- a SketchLine is rejected as "Invalid
        # entity" -- so we build a construction axis from the hexagon's
        # B->I world geometry.
        if (meshingOffsetDegrees and sketchLines and frustumBodies):
            shaftLine = sketchLines[0]
            p1 = shaftLine.startSketchPoint.worldGeometry
            p2 = shaftLine.endSketchPoint.worldGeometry
            direction = adsk.core.Vector3D.create(
                p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
            direction.normalize()

            # Build a rotation matrix around the world-space shaft axis
            # and apply it via MoveFeatures.defineAsFreeMove. This avoids
            # needing a construction axis entity -- adding construction
            # axes in this context errors with "Environment is not
            # supported", and defineAsRotate requires a linear edge or
            # construction axis (SketchLine is rejected).
            transform = adsk.core.Matrix3D.create()
            transform.setToRotation(
                math.radians(meshingOffsetDegrees), direction, p1)

            bodyCol = adsk.core.ObjectCollection.create()
            for body in frustumBodies:
                if body.isValid:
                    bodyCol.add(body)
            if bodyCol.count > 0:
                moves = designComponent.features.moveFeatures
                moveInput = moves.createInput2(bodyCol)
                moveInput.defineAsFreeMove(transform)
                moves.add(moveInput)
                futil.log(
                    f'{name}: applied meshing rotation '
                    f'{meshingOffsetDegrees:.4f} deg around shaft axis',
                    force_console=True)

        # Step 6: move surviving bodies (the frustum, now containing the
        # joined tooth) to the gear sub-component.
        for body in frustumBodies:
            if not body.isValid:
                continue
            try:
                body.moveToComponent(gearOcc)
            except Exception as e:
                futil.log(
                    f'{name}: could not move body ({e}); stays in Design')

    def _applyConicalCut(self, component, bodies, frustumBody, cutLines,
                          apexPoint, name):
        """Apply each cut in `cutLines` sequentially to every piece in
        `bodies`, sourcing the cutting tool as an existing conical face of
        `frustumBody` (the cut edges are sides of the revolved hexagon).
        After all cuts, expect exactly three pieces total: drop the piece
        containing the apex, drop the smaller of the two remaining (by
        volume), and return the largest -- the actual tooth."""
        splits = component.features.splitBodyFeatures
        current = list(bodies)
        for cutIdx, cutLine in enumerate(cutLines):
            cutFace = self._findConeFaceForCutLine(
                frustumBody, cutLine, f'{name} cut#{cutIdx + 1}')
            if cutFace is None:
                raise Exception(
                    f'{name}: could not locate cone face for cut#{cutIdx + 1} '
                    f'on frustum body (searched {frustumBody.faces.count} '
                    f'face(s)) -- cutLine endpoints did not lie on any cone '
                    f'face within tolerance')
            futil.log(
                f'{name}: cut#{cutIdx + 1}: splitting {len(current)} body(ies)',
                force_console=True)
            nextPieces = []
            for body in current:
                if not body.isValid:
                    continue
                splitInput = splits.createInput(body, cutFace, True)
                try:
                    splitResult = splits.add(splitInput)
                except RuntimeError as e:
                    # SPLIT_TARGET_TOOL_NOT_INTERSECT: the cut tool doesn't
                    # cross this piece (expected when the second cut sits
                    # past the first cut's plane). Keep the piece intact.
                    msg = str(e)
                    if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or \
                       '交差' in msg:
                        futil.log(
                            f'{name}: cut#{cutIdx + 1}: tool does not '
                            f'intersect body "{body.name}", keeping intact',
                            force_console=True)
                        nextPieces.append(body)
                        continue
                    raise
                futil.log(
                    f'{name}: cut#{cutIdx + 1}: split produced '
                    f'{splitResult.bodies.count} piece(s)',
                    force_console=True)
                for i in range(splitResult.bodies.count):
                    nextPieces.append(splitResult.bodies.item(i))
            current = nextPieces

        if len(current) != 3:
            raise Exception(
                f'{name}: expected three pieces after both cuts, got '
                f'{len(current)}')

        apexWorld = None
        if apexPoint is not None:
            try:
                apexWorld = apexPoint.worldGeometry
            except Exception:
                apexWorld = None

        apexPieces = []
        nonApexPieces = []
        for idx, body in enumerate(current):
            if not body.isValid:
                continue
            containment = None
            if apexWorld is not None:
                containment = body.pointContainment(apexWorld)
            isApex = containment in (
                adsk.fusion.PointContainment.PointInsidePointContainment,
                adsk.fusion.PointContainment.PointOnPointContainment)
            futil.log(
                f'{name}: piece[{idx}] containment={containment} '
                f'-> {"APEX" if isApex else "non-apex"}',
                force_console=True)
            (apexPieces if isApex else nonApexPieces).append(body)

        if len(apexPieces) != 1:
            raise Exception(
                f'{name}: expected exactly one apex-containing piece, got '
                f'{len(apexPieces)}')
        if len(nonApexPieces) != 2:
            raise Exception(
                f'{name}: expected exactly two non-apex pieces, got '
                f'{len(nonApexPieces)}')

        nonApexPieces.sort(key=lambda b: b.physicalProperties.volume)
        smallerPiece = nonApexPieces[0]
        largerPiece = nonApexPieces[1]
        futil.log(
            f'{name}: piece volumes: smaller='
            f'{smallerPiece.physicalProperties.volume:.6f} larger='
            f'{largerPiece.physicalProperties.volume:.6f}',
            force_console=True)

        removes = component.features.removeFeatures
        removes.add(apexPieces[0])
        removes.add(smallerPiece)
        futil.log(
            f'{name}: removed apex piece + smaller piece via RemoveFeatures',
            force_console=True)

        return [largerPiece]

    def _findConeFaceForCutLine(self, body, cutLine, name=''):
        """Return the BRepFace on `body` whose underlying conical surface
        contains both endpoints of `cutLine` (in world coordinates), or
        None. When the frustum's hexagonal profile included `cutLine` as
        one of its edges, the revolve produced exactly such a face."""
        try:
            mWorld = cutLine.startSketchPoint.worldGeometry
            nWorld = cutLine.endSketchPoint.worldGeometry
        except Exception as e:
            futil.log(
                f'{name}: cone-face search: cutLine endpoints unreadable '
                f'({e})', force_console=True)
            return None

        tol = 1e-2  # cm (Fusion internal length unit) ~ 0.1 mm
        coneCount = 0
        best = None
        bestDist = None
        for face in body.faces:
            surf = face.geometry
            if surf is None:
                continue
            if surf.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            coneCount += 1
            dM = self._surfaceDistance(surf, mWorld)
            dN = self._surfaceDistance(surf, nWorld)
            futil.log(
                f'{name}: cone face #{coneCount} M_dist={dM} N_dist={dN}',
                force_console=True)
            if dM is None or dN is None:
                continue
            if dM < tol and dN < tol:
                score = dM + dN
                if bestDist is None or score < bestDist:
                    best = face
                    bestDist = score
        futil.log(
            f'{name}: cone-face search: {coneCount} cone(s) on body, '
            f'matched={"yes" if best else "no"}',
            force_console=True)
        return best

    @staticmethod
    def _surfaceDistance(surface, point):
        evaluator = surface.evaluator
        ok, param = evaluator.getParameterAtPoint(point)
        if not ok:
            return None
        ok2, projected = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        dx = projected.x - point.x
        dy = projected.y - point.y
        dz = projected.z - point.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _findSpurToothProfile(sketch):
        """Find the tooth cross-section profile in a spur-gear tooth sketch.
        Handles both non-embedded (6 curves: 2 splines + 2 arcs + 2 lines)
        and embedded (4 curves: 2 splines + 2 arcs) tooth shapes."""
        for expectCurves, expectLines in ((6, 2), (4, 0)):
            for profile in sketch.profiles:
                for loop in profile.profileLoops:
                    if loop.profileCurves.count != expectCurves:
                        continue
                    arcs = nurbs = lines = 0
                    invalid = False
                    for curve in loop.profileCurves:
                        ctyp = curve.geometry.curveType
                        if ctyp == adsk.core.Curve3DTypes.Arc3DCurveType:
                            arcs += 1
                        elif ctyp == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                            nurbs += 1
                        elif ctyp == adsk.core.Curve3DTypes.Line3DCurveType:
                            lines += 1
                        else:
                            invalid = True
                            break
                    if invalid:
                        continue
                    if nurbs == 2 and arcs == 2 and lines == expectLines:
                        return profile
        return None

    def _buildVirtualSpurProfile(self, component, referencePlane, lineToInclude,
                                  anchorPoint, module_mm, virtualTeeth, name,
                                  rotationDegrees=180.0):
        """Create a plane perpendicular to the Gear Profiles sketch plane
        that includes the given line, draw a virtual spur gear tooth profile
        on it centered at anchorPoint (rotated by rotationDegrees), and
        create a construction axis through anchorPoint perpendicular to the
        new plane."""
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
        # The doc asks for a post-draw rotation around the axis created below.
        # The drawer's built-in `angle` kwarg produces the same final
        # geometry (rotated tooth around the anchor) without needing a
        # separate Move feature, so we use it here.
        drawer.draw(anchorPoint, angle=math.radians(rotationDegrees))

        # Construction axis through anchorPoint perpendicular to the tooth
        # profile plane — kept as a parametric reference per the doc.
        #
        # Fusion's setByPerpendicularAtPoint requires a BRepFace (not a
        # ConstructionPlane), so we build the axis as the intersection of two
        # planes:
        #   - the Gear Profiles sketch plane (referencePlane)
        #   - a helper plane perpendicular to lineToInclude at its far end,
        #     i.e., at anchorPoint
        # That intersection is the line in referencePlane perpendicular to
        # lineToInclude through anchorPoint, which is exactly the tooth
        # plane's normal through anchorPoint.
        helperInput = component.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            lineToInclude,
            adsk.core.ValueInput.createByReal(1.0))
        helperPlane = component.constructionPlanes.add(helperInput)
        helperPlane.name = f'{name} Axis Helper'
        helperPlane.isLightBulbOn = False

        axisInput = component.constructionAxes.createInput()
        axisInput.setByTwoPlanes(referencePlane, helperPlane)
        toothAxis = component.constructionAxes.add(axisInput)
        toothAxis.name = f'{name} Axis'
        toothAxis.isLightBulbOn = False

        return sketch

    def _pointWorldGeometry(self, point) -> adsk.core.Point3D:
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        if point.objectType == adsk.fusion.ConstructionPoint.classType():
            return point.geometry
        raise Exception(f'Unsupported center point type: {point.objectType}')
