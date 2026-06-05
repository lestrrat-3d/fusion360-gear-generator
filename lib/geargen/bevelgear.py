import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the only module-level constants -- bevel registers no user
# parameters; every value is precomputed in Python in internal cm).
# ---------------------------------------------------------------------------
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER_POINT = 'centerPoint'
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_MODULE = 'module'
INPUT_ID_SHAFT_ANGLE = 'shaftAngle'
INPUT_ID_DRIVING_TEETH = 'drivingTeeth'
INPUT_ID_PINION_TEETH = 'pinionTeeth'
INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'
INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'
INPUT_ID_BORE_ENABLE = 'boreEnable'
INPUT_ID_DRIVING_BORE = 'drivingBore'
INPUT_ID_PINION_BORE = 'pinionBore'
INPUT_ID_FACE_WIDTH = 'faceWidth'

# Pressure angle is not a bevel dialog input; bevel hardcodes the standard 20 deg.
_PRESSURE_ANGLE_RAD = math.radians(20)
_INVOLUTE_STEPS = 15


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection -- FIRST so it wins Fusion's auto-focus).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point (selection).
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Select the point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component (selection -- pre-selected to the root component).
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the parent component for the new bevel gear pair')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module (unitless).
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle (deg).
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6. Driving Gear Teeth Number.
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth Number.
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 8. Driving Gear Base Height (mm).
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 9. Pinion Gear Base Height (mm).
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore (checkbox).
        inputs.addBoolValueInput(
            INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11. Driving Gear Bore Diameter (mm).
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 12. Pinion Gear Bore Diameter (mm).
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 13. Face Width (mm).
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))


# ---------------------------------------------------------------------------
# Virtual spur proxy -- a fake spur `parent` so the borrowed spur tooth
# generator runs without registering Fusion user parameters.
# ---------------------------------------------------------------------------
class _Val:
    """Tiny value-wrapper exposing `.value`, matching UserParameter's surface."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """Precomputes (in internal cm) exactly the keys the spur tooth drawer reads.

    The spur generator reads parameters via `parent.getParameter(name).value`.
    Module arrives in raw mm and the standard spur formulas are applied, then the
    resulting circle radii/diameters are `to_cm`'d (they must be in cm to match
    what the spur tooth generator expects)."""

    def __init__(self, module_mm, virtualTeeth):
        alpha = _PRESSURE_ANGLE_RAD
        pitch_mm = module_mm * virtualTeeth
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._values = {
            'Module': module_mm,
            'ToothNumber': virtualTeeth,
            'PressureAngle': alpha,
            'InvoluteSteps': _INVOLUTE_STEPS,
            'PitchCircleDiameter': to_cm(pitch_mm),
            'PitchCircleRadius': to_cm(pitch_mm / 2),
            'BaseCircleDiameter': to_cm(base_mm),
            'BaseCircleRadius': to_cm(base_mm / 2),
            'RootCircleDiameter': to_cm(root_mm),
            'RootCircleRadius': to_cm(root_mm / 2),
            'TipCircleDiameter': to_cm(tip_mm),
            'TipCircleRadius': to_cm(tip_mm / 2),
        }
        # The spur tooth generator records embedded-ness here.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator).
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

        # Stashed inputs (filled by _readInputs).
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0

        # Stashed anchor-sketch projected center point (set in _buildAnchorSketch).
        self._anchorCenterPoint = None

    # ===================================================================
    # input reading
    # ===================================================================
    def _readInputs(self, inputs: adsk.core.CommandInputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections.
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one parent component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parent.component
        elif parent.objectType == adsk.fusion.Component.classType():
            parentComponent = parent
        else:
            raise Exception('Selected parent is not a component or occurrence')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        targetPlane = planes[0]

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # Module: unit '' -> raw number meaning millimetres.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft angle: comes back in radians; convert to degrees for range check.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.3f})')

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Tooth numbers must be at least 3')

        # 'mm' inputs come back already in internal cm -- use as-is, do NOT to_cm again.
        self._drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        (self._boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')

        if (self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0
                or self._drivingBore_cm < 0 or self._pinionBore_cm < 0
                or self._faceWidth_cm < 0):
            raise Exception('Base heights, bore diameters and face width must be non-negative')

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ===================================================================
    # small helpers
    # ===================================================================
    def _assertFullyConstrained(self, sketch, name):
        if not sketch.isFullyConstrained:
            raise Exception(f'Sketch "{name}" is not fully constrained (free DOF remain)')

    def _pointWorldGeometry(self, point):
        """World Point3D for a SketchPoint (.worldGeometry) or ConstructionPoint (.geometry)."""
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _surfaceDistance(self, surface, worldPoint):
        """Distance from worldPoint to the infinite surface; None if unevaluable."""
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return worldPoint.distanceTo(projected)

    # ===================================================================
    # §1 anchor sketch
    # ===================================================================
    def _buildAnchorSketch(self, designComp, targetPlane, centerPoint):
        # Start the sketch DIRECTLY on the user-selected target plane (no re-derive/offset).
        sketch = designComp.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        # Project the user center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Anchor line through the projected center, seeded ~10mm long.
        cg = projectedCenter.geometry
        p0 = adsk.core.Point3D.create(cg.x - to_cm(5), cg.y, 0)
        p1 = adsk.core.Point3D.create(cg.x + to_cm(5), cg.y, 0)
        anchorLine = lines.addByTwoPoints(p0, p1)

        # BOTH: intersection (center on line) AND midpoint (center bisects line).
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length dimension (arbitrary reference value).
        lengthDim = dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + to_cm(2), 0))
        lengthDim.parameter.value = to_cm(10)

        # Pin direction in the sketch's OWN frame (works on any tilted plane).
        constraints.addHorizontal(anchorLine)

        # Stash the projected-center SketchPoint so §2 re-projects THIS point.
        self._anchorCenterPoint = projectedCenter

        self._assertFullyConstrained(sketch, 'Anchor Sketch')
        return anchorLine

    # ===================================================================
    # §2 gear profiles
    # ===================================================================
    def _buildGearProfiles(self, designComp, bevelComp, pinionOcc, drivingOcc,
                           targetPlane, anchorLine,
                           module, drivingTeeth, pinionTeeth, shaftAngle_deg,
                           drivingPitchDia_cm, pinionPitchDia_cm,
                           drivingBore_cm, pinionBore_cm):
        Sigma = math.radians(shaftAngle_deg)

        # Build the Gear Profiles plane: includes the Anchor Line, at 90 deg off the
        # ORIGINAL targetPlane (so it inherits the true target-plane orientation).
        planeInput = designComp.constructionPlanes.createInput()
        planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComp.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComp.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the ANCHOR SKETCH center point (stashed in §1), not the raw center.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        # Project the anchor line to read its 2-D direction.
        projectedAnchor = sketch.project(anchorLine).item(0)

        c = projectedCenter.geometry
        aStart = projectedAnchor.startSketchPoint.geometry
        aEnd = projectedAnchor.endSketchPoint.geometry
        dx = aEnd.x - aStart.x
        dy = aEnd.y - aStart.y
        dlen = math.hypot(dx, dy)
        d = (dx / dlen, dy / dlen)

        # In-plane perpendicular (two senses). Pick sign by the target normal.
        perp = (-d[1], d[0])

        # One-bit direction comparison: choose perp pointing toward the target normal.
        normal = get_normal(targetPlane)
        normal.normalize()
        # Map perp (sketch-local) to world via the sketch's two in-plane axes.
        xAxis = sketch.xDirection
        yAxis = sketch.yDirection
        perpWorld = adsk.core.Vector3D.create(
            perp[0] * xAxis.x + perp[1] * yAxis.x,
            perp[0] * xAxis.y + perp[1] * yAxis.y,
            perp[0] * xAxis.z + perp[1] * yAxis.z)
        if perpWorld.dotProduct(normal) < 0:
            perp = (-perp[0], -perp[1])

        DPD = drivingPitchDia_cm
        PPD = pinionPitchDia_cm

        # ---- center -> Apex construction line (perpendicular to anchor) ----
        apexSeed = adsk.core.Point3D.create(c.x + perp[0] * DPD, c.y + perp[1] * DPD, 0)
        centerToApex = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x, c.y, 0), apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projectedAnchor)
        # No length constraint -- Apex pinned later by "point I coincident with center".
        apexPoint = centerToApex.endSketchPoint
        apex = apexPoint.geometry

        # ---- closed-form cone geometry seeds for Apex->A / Apex->B ----
        # tan gamma_p = sin Sigma * PPD / (DPD + PPD cos Sigma)
        gamma_p = math.atan2(math.sin(Sigma) * PPD, DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        R = (PPD / 2) / math.sin(gamma_p)
        lenA = R * math.cos(gamma_p)   # |Apex->A|
        lenB = R * math.cos(gamma_g)   # |Apex->B|

        # Driving Gear Shaft Axis: from apex toward the anchor line (-perp), parallel
        # to centerToApex. End is point B. seed |Apex->B| = lenB along -perp.
        bSeed = adsk.core.Point3D.create(
            apex.x - perp[0] * lenB, apex.y - perp[1] * lenB, 0)
        drivingShaft = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex.x, apex.y, 0), bSeed)
        drivingShaft.isConstruction = True
        constraints.addCoincident(drivingShaft.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaft, centerToApex)
        pointB = drivingShaft.endSketchPoint

        # Pinion Gear Shaft Axis: driving-shaft direction rotated about apex by Sigma.
        # Form BOTH candidates (+Sigma and -Sigma) and keep the one with greater sketch-X.
        # Driving shaft direction (apex -> B) is -perp.
        ddir = (-perp[0], -perp[1])

        def rotate_about_apex(direction, theta, length):
            cs = math.cos(theta)
            sn = math.sin(theta)
            rx = direction[0] * cs - direction[1] * sn
            ry = direction[0] * sn + direction[1] * cs
            return adsk.core.Point3D.create(apex.x + rx * length, apex.y + ry * length, 0)

        candPlus = rotate_about_apex(ddir, Sigma, lenA)
        candMinus = rotate_about_apex(ddir, -Sigma, lenA)
        # Compare BOTH candidates' X; take the larger (NOT a rotate-then-conditional-flip).
        if candPlus.x >= candMinus.x:
            aSeed = candPlus
        else:
            aSeed = candMinus

        pinionShaft = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex.x, apex.y, 0), aSeed)
        pinionShaft.isConstruction = True
        constraints.addCoincident(pinionShaft.startSketchPoint, apexPoint)
        pointA = pinionShaft.endSketchPoint

        # Angular dimension Sigma between pinion shaft and driving shaft.
        # Place text inside the wedge containing Sigma (toward the aSeed/bSeed bisector).
        angText = adsk.core.Point3D.create(
            apex.x + (ddir[0] + (aSeed.x - apex.x) / lenA) * 0.25 * lenA,
            apex.y + (ddir[1] + (aSeed.y - apex.y) / lenA) * 0.25 * lenA, 0)
        dimensions.addAngularDimension(pinionShaft, drivingShaft, angText).parameter.value = Sigma

        # ---- A -> Apex2 perpendicular drop (length PPD/2), perpendicular to pinion shaft ----
        # seed toward where Apex2 lies (between the two shaft axes / toward anchor line).
        toMidAB = adsk.core.Vector3D.create(
            (aSeed.x + bSeed.x) / 2 - aSeed.x, (aSeed.y + bSeed.y) / 2 - aSeed.y, 0)
        if toMidAB.length > 0:
            toMidAB.normalize()
        nApex2A = adsk.core.Point3D.create(
            aSeed.x + toMidAB.x * (PPD / 2), aSeed.y + toMidAB.y * (PPD / 2), 0)
        dropA = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed.x, aSeed.y, 0), nApex2A)
        dropA.isConstruction = True
        constraints.addCoincident(dropA.startSketchPoint, pointA)
        constraints.addPerpendicular(dropA, pinionShaft)
        dimDropA = dimensions.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(nApex2A.x, nApex2A.y, 0))
        dimDropA.parameter.value = PPD / 2

        # ---- B -> Apex2 perpendicular drop (length DPD/2), perpendicular to driving shaft ----
        toMidBA = adsk.core.Vector3D.create(
            (aSeed.x + bSeed.x) / 2 - bSeed.x, (aSeed.y + bSeed.y) / 2 - bSeed.y, 0)
        if toMidBA.length > 0:
            toMidBA.normalize()
        nApex2B = adsk.core.Point3D.create(
            bSeed.x + toMidBA.x * (DPD / 2), bSeed.y + toMidBA.y * (DPD / 2), 0)
        dropB = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0), nApex2B)
        dropB.isConstruction = True
        constraints.addCoincident(dropB.startSketchPoint, pointB)
        constraints.addPerpendicular(dropB, drivingShaft)
        dimDropB = dimensions.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(nApex2B.x, nApex2B.y, 0))
        dimDropB.parameter.value = DPD / 2

        # ---- Apex2: coincide the two drop ends ----
        constraints.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # ---- Pitch Line: Apex -> Apex2 ----
        pitchLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex.x, apex.y, 0),
            adsk.core.Point3D.create(apex2Point.geometry.x, apex2Point.geometry.y, 0))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        ded_cm = to_cm(1.25 * module)

        # ---- Dedendum lines from Apex2, perpendicular to the Pitch Line ----
        # Pinion Gear Dedendum (away from anchor line) -> point C.
        # Driving Gear Dedendum (toward anchor line) -> point D.
        a2 = apex2Point.geometry
        # pitch-line direction (apex -> apex2)
        pdx = a2.x - apex.x
        pdy = a2.y - apex.y
        plen = math.hypot(pdx, pdy)
        pu = (pdx / plen, pdy / plen)
        pperp = (-pu[1], pu[0])  # perpendicular to pitch line

        # Decide which perpendicular sense points "away from anchor line" (toward +perp).
        if (pperp[0] * perp[0] + pperp[1] * perp[1]) >= 0:
            awayDir = pperp
        else:
            awayDir = (-pperp[0], -pperp[1])
        towardDir = (-awayDir[0], -awayDir[1])

        cSeed = adsk.core.Point3D.create(
            a2.x + awayDir[0] * ded_cm, a2.y + awayDir[1] * ded_cm, 0)
        pinionDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(a2.x, a2.y, 0), cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dimC = dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cSeed.x, cSeed.y, 0))
        dimC.parameter.value = ded_cm
        pointC = pinionDedendum.endSketchPoint

        dSeed = adsk.core.Point3D.create(
            a2.x + towardDir[0] * ded_cm, a2.y + towardDir[1] * ded_cm, 0)
        drivingDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(a2.x, a2.y, 0), dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dimD = dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(dSeed.x, dSeed.y, 0))
        dimD.parameter.value = ded_cm
        pointD = drivingDedendum.endSketchPoint

        # ---- Root Axes: Apex -> C (pinion), Apex -> D (driving) ----
        pinionRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex.x, apex.y, 0),
            adsk.core.Point3D.create(pointC.geometry.x, pointC.geometry.y, 0))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        drivingRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex.x, apex.y, 0),
            adsk.core.Point3D.create(pointD.geometry.x, pointD.geometry.y, 0))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        mod_cm = to_cm(module)

        # ---- point E: from A, collinear with Apex->A, length module (no dim) ----
        ag = pointA.geometry
        adir = (ag.x - apex.x, ag.y - apex.y)
        adlen = math.hypot(adir[0], adir[1])
        adir = (adir[0] / adlen, adir[1] / adlen)
        eSeed = adsk.core.Point3D.create(ag.x + adir[0] * mod_cm, ag.y + adir[1] * mod_cm, 0)
        lineAE = lines.addByTwoPoints(
            adsk.core.Point3D.create(ag.x, ag.y, 0), eSeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaft)
        pointE = lineAE.endSketchPoint

        # ---- line C->E, perpendicular to A->E ----
        lineCE = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointC.geometry.x, pointC.geometry.y, 0),
            adsk.core.Point3D.create(eSeed.x, eSeed.y, 0))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # ---- point F: from B, collinear with Apex->B, length module (no dim) ----
        bg = pointB.geometry
        bdir = (bg.x - apex.x, bg.y - apex.y)
        bdlen = math.hypot(bdir[0], bdir[1])
        bdir = (bdir[0] / bdlen, bdir[1] / bdlen)
        fSeed = adsk.core.Point3D.create(bg.x + bdir[0] * mod_cm, bg.y + bdir[1] * mod_cm, 0)
        lineBF = lines.addByTwoPoints(
            adsk.core.Point3D.create(bg.x, bg.y, 0), fSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaft)
        pointF = lineBF.endSketchPoint

        # ---- line D->F, perpendicular to B->F ----
        lineDF = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointD.geometry.x, pointD.geometry.y, 0),
            adsk.core.Point3D.create(fSeed.x, fSeed.y, 0))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # ---- point G: from E collinear to A->E, length module (no dim) ----
        eg = pointE.geometry
        gSeed = adsk.core.Point3D.create(eg.x + adir[0] * mod_cm, eg.y + adir[1] * mod_cm, 0)
        lineEG = lines.addByTwoPoints(
            adsk.core.Point3D.create(eg.x, eg.y, 0), gSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # ---- point H: from C, length module, collinear with Apex2->C ----
        cg = pointC.geometry
        cdir = (cg.x - a2.x, cg.y - a2.y)
        cdlen = math.hypot(cdir[0], cdir[1])
        cdir = (cdir[0] / cdlen, cdir[1] / cdlen)
        hSeed = adsk.core.Point3D.create(cg.x + cdir[0] * mod_cm, cg.y + cdir[1] * mod_cm, 0)
        lineCH = lines.addByTwoPoints(
            adsk.core.Point3D.create(cg.x, cg.y, 0), hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # ---- line G->H, perpendicular to E->G ----
        lineGH = lines.addByTwoPoints(
            adsk.core.Point3D.create(gSeed.x, gSeed.y, 0),
            adsk.core.Point3D.create(hSeed.x, hSeed.y, 0))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # ---- point I: from F collinear to B->F, length module (no dim) ----
        fg = pointF.geometry
        iSeed = adsk.core.Point3D.create(fg.x + bdir[0] * mod_cm, fg.y + bdir[1] * mod_cm, 0)
        lineFI = lines.addByTwoPoints(
            adsk.core.Point3D.create(fg.x, fg.y, 0), iSeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # ---- point J: from D, length module, collinear with Apex2->D ----
        dg = pointD.geometry
        ddir2 = (dg.x - a2.x, dg.y - a2.y)
        ddlen2 = math.hypot(ddir2[0], ddir2[1])
        ddir2 = (ddir2[0] / ddlen2, ddir2[1] / ddlen2)
        jSeed = adsk.core.Point3D.create(dg.x + ddir2[0] * mod_cm, dg.y + ddir2[1] * mod_cm, 0)
        lineDJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(dg.x, dg.y, 0), jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # ---- line I->J, perpendicular to F->I ----
        lineIJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(iSeed.x, iSeed.y, 0),
            adsk.core.Point3D.create(jSeed.x, jSeed.y, 0))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # ---- offset dimension: B->Apex2 drop (dropB) vs J->I (already parallel) ----
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8)
        offJI = dimensions.addOffsetDimension(
            dropB, lineIJ,
            adsk.core.Point3D.create(jSeed.x, jSeed.y, 0))
        offJI.parameter.value = drivingBaseHeight_cm

        # ---- offset dimension: A->Apex2 drop (dropA) vs G->H (already parallel) ----
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        offGH = dimensions.addOffsetDimension(
            dropA, lineGH,
            adsk.core.Point3D.create(hSeed.x, hSeed.y, 0))
        offGH.parameter.value = pinionBaseHeight_cm

        # ---- line A->G ----
        lineAG = lines.addByTwoPoints(
            adsk.core.Point3D.create(ag.x, ag.y, 0),
            adsk.core.Point3D.create(pointG.geometry.x, pointG.geometry.y, 0))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # ---- close the apex: point I coincident with center (pins the figure) ----
        constraints.addCoincident(pointI, projectedCenter)

        # ---- point K: from G along Apex->A (away from apex); pin with two coincidents ----
        kg = pointG.geometry
        kSeed = adsk.core.Point3D.create(kg.x + adir[0] * mod_cm, kg.y + adir[1] * mod_cm, 0)
        lineGK = lines.addByTwoPoints(
            adsk.core.Point3D.create(kg.x, kg.y, 0), kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        # Pin K: on line Apex->A and on pinion dedendum (Apex2->C extended).
        constraints.addCoincident(pointK, pinionShaft)
        constraints.addCoincident(pointK, pinionDedendum)
        # reference line C -> K
        lineCK = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointC.geometry.x, pointC.geometry.y, 0),
            adsk.core.Point3D.create(pointK.geometry.x, pointK.geometry.y, 0))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # ---- point L: from I along Apex->B; pin the same way ----
        ig = pointI.geometry
        lSeed = adsk.core.Point3D.create(ig.x + bdir[0] * mod_cm, ig.y + bdir[1] * mod_cm, 0)
        lineIL = lines.addByTwoPoints(
            adsk.core.Point3D.create(ig.x, ig.y, 0), lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaft)
        constraints.addCoincident(pointL, drivingDedendum)
        lineDL = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointD.geometry.x, pointD.geometry.y, 0),
            adsk.core.Point3D.create(pointL.geometry.x, pointL.geometry.y, 0))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # ==========================================================
        # Maximum Face Width from SOLVED geometry; cap / validate Face Width.
        # ==========================================================
        def perpDistance(pt, lineP, lineQ):
            px, py = pt.x, pt.y
            qx, qy = lineQ.x - lineP.x, lineQ.y - lineP.y
            qlen = math.hypot(qx, qy)
            qx, qy = qx / qlen, qy / qlen
            wx, wy = px - lineP.x, py - lineP.y
            # perpendicular component magnitude
            cross = wx * qy - wy * qx
            return abs(cross)

        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distPinion = perpDistance(gA, gC, gH)   # A to line C-H
        distDriving = perpDistance(gB, gD, gJ)  # B to line D-J
        maxFaceWidth_cm = 0.95 * min(distPinion, distDriving)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face Width exceeds the maximum permitted '
                    f'({to_mm(maxFaceWidth_cm):.4f} mm); reduce it or it would '
                    'self-intersect the revolve axis')
            faceWidth_cm = self._faceWidth_cm
        else:
            coneDistance_cm = to_cm(
                math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
            faceWidth_cm = min(coneDistance_cm / 6, maxFaceWidth_cm)

        # ==========================================================
        # Toe line M->N (pinion) and its mirror O->P (driving).
        # ==========================================================
        # --- M->N ---
        # Seed M near midpoint of Apex->C; N slid from M along C->H toward A->Apex2.
        mSeed = adsk.core.Point3D.create((apex.x + gC.x) / 2, (apex.y + gC.y) / 2, 0)
        chDir = (gH.x - gC.x, gH.y - gC.y)
        chLen = math.hypot(chDir[0], chDir[1])
        chDir = (chDir[0] / chLen, chDir[1] / chLen)
        mToA = math.hypot(gA.x - mSeed.x, gA.y - mSeed.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + chDir[0] * mToA, mSeed.y + chDir[1] * mToA, 0)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)   # M on Apex->C root axis
        constraints.addCoincident(pointN, dropA)            # N on A->Apex2 drop line
        constraints.addParallel(lineMN, lineCH)
        offMN = dimensions.addOffsetDimension(
            lineCH, lineMN,
            adsk.core.Point3D.create(nSeed.x, nSeed.y, 0))
        offMN.parameter.value = faceWidth_cm
        # connecting lines M->C and N->A
        lineMC = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointM.geometry.x, pointM.geometry.y, 0),
            adsk.core.Point3D.create(gC.x, gC.y, 0))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointN.geometry.x, pointN.geometry.y, 0),
            adsk.core.Point3D.create(gA.x, gA.y, 0))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- O->P (driving mirror) ---
        oSeed = adsk.core.Point3D.create((apex.x + gD.x) / 2, (apex.y + gD.y) / 2, 0)
        djDir = (gJ.x - gD.x, gJ.y - gD.y)
        djLen = math.hypot(djDir[0], djDir[1])
        djDir = (djDir[0] / djLen, djDir[1] / djLen)
        oToB = math.hypot(gB.x - oSeed.x, gB.y - oSeed.y)
        pSeed = adsk.core.Point3D.create(
            oSeed.x + djDir[0] * oToB, oSeed.y + djDir[1] * oToB, 0)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)  # O on Apex->D root axis
        constraints.addCoincident(pointP, dropB)            # P on B->Apex2 drop line
        constraints.addParallel(lineOP, lineDJ)
        offOP = dimensions.addOffsetDimension(
            lineDJ, lineOP,
            adsk.core.Point3D.create(pSeed.x, pSeed.y, 0))
        offOP.parameter.value = faceWidth_cm
        lineOD = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointO.geometry.x, pointO.geometry.y, 0),
            adsk.core.Point3D.create(gD.x, gD.y, 0))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(
            adsk.core.Point3D.create(pointP.geometry.x, pointP.geometry.y, 0),
            adsk.core.Point3D.create(gB.x, gB.y, 0))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)

        # line from B to I
        lineBI = lines.addByTwoPoints(
            adsk.core.Point3D.create(gB.x, gB.y, 0),
            adsk.core.Point3D.create(pointI.geometry.x, pointI.geometry.y, 0))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch, 'Gear Profiles')

        # gather the §2 anchors used downstream
        prof = {
            'sketch': sketch,
            'gearProfilesPlane': gearProfilesPlane,
            'apexPoint': apexPoint,
            'A': pointA, 'B': pointB, 'C': pointC, 'D': pointD,
            'G': pointG, 'H': pointH, 'I': pointI, 'J': pointJ,
            'K': pointK, 'L': pointL,
            'M': pointM, 'N': pointN, 'O': pointO, 'P': pointP,
            'lineCK': lineCK, 'lineDL': lineDL,
            'gamma_p': gamma_p, 'gamma_g': gamma_g,
        }

        # ==========================================================
        # §3 + per-gear body creation (pinion first, then driving).
        # ==========================================================
        # ---- pinion tooth profile ----
        (pinionToothSketch, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComp, prof, targetPlane, module,
            prof['gamma_p'], pinionPitchDia_cm, prof['C'], prof['K'], prof['lineCK'],
            'Pinion')
        # ---- driving tooth profile ----
        (drivingToothSketch, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComp, prof, targetPlane, module,
            prof['gamma_g'], drivingPitchDia_cm, prof['D'], prof['L'], prof['lineDL'],
            'Driving')

        # ---- create pinion gear body ----
        self._createGearBody(
            designComp, bevelComp, pinionOcc, prof, gearProfilesPlane,
            ['A', 'G', 'H', 'C', 'M', 'N'],
            pinionToothSketch, 'M', 'N', 'C', 'H',
            pinionTeeth, pinionBore_cm, pinionPitchDia_cm,
            'Pinion', meshRotateTeeth=None)

        # ---- create driving gear body (mesh-rotated) ----
        self._createGearBody(
            designComp, bevelComp, drivingOcc, prof, gearProfilesPlane,
            ['B', 'I', 'J', 'D', 'O', 'P'],
            drivingToothSketch, 'O', 'P', 'D', 'J',
            drivingTeeth, drivingBore_cm, drivingPitchDia_cm,
            'Driving', meshRotateTeeth=drivingTeeth)

    # ===================================================================
    # §3 virtual spur tooth profile for one gear
    # ===================================================================
    def _buildVirtualSpurProfile(self, designComp, prof, targetPlane, module,
                                 gamma, pitchDia_cm, rootPoint, centerPoint,
                                 rootToCenterLine, label):
        gearProfilesPlane = prof['gearProfilesPlane']

        # virtual (Tredgold) tooth number from closed form.
        virtualPitchRadius_cm = (pitchDia_cm / 2) / math.cos(gamma)
        # Module is mm; radius is cm -> convert radius to mm for the count.
        virtualTeeth = int(math.floor(2 * to_mm(virtualPitchRadius_cm) / module))
        if virtualTeeth < 3:
            virtualTeeth = 3

        # plane that includes line (root->center), perpendicular to Gear Profiles plane.
        planeInput = designComp.constructionPlanes.createInput()
        planeInput.setByAngle(
            rootToCenterLine, adsk.core.ValueInput.createByString('90 deg'),
            gearProfilesPlane)
        toothPlane = designComp.constructionPlanes.add(planeInput)
        toothPlane.name = f'{label} Tooth Plane'

        toothSketch = designComp.sketches.add(toothPlane)
        toothSketch.name = f'{label} Tooth'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        self._assertFullyConstrained(toothSketch, f'{label} Tooth')

        # construction axis through center, normal to the tooth plane, via setByTwoPlanes:
        #   Gear Profiles plane  X  helper plane (perp to root->center at its far end).
        helperInput = designComp.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(rootToCenterLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComp.constructionPlanes.add(helperInput)
        helperPlane.name = f'{label} Tooth Axis Helper'

        axisInput = designComp.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComp.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Tooth Axis'
        toothAxis.isLightBulbOn = False

        return (toothSketch, toothAxis)

    # ===================================================================
    # per-gear body creation
    # ===================================================================
    def _createGearBody(self, designComp, bevelComp, gearOcc, prof, gearProfilesPlane,
                        hexVerts, toothSketch, toeStartKey, toeEndKey,
                        heelStartKey, heelEndKey,
                        teethNumber, bore_cm, pitchDia_cm,
                        label, meshRotateTeeth):
        # ---- fresh profile sketch with the six recreated, FIXED vertices ----
        profileSketch = designComp.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{label} Profile'
        profileSketch.isVisible = True

        lines = profileSketch.sketchCurves.sketchLines

        # recreate each §2 vertex as a NEW point (modelToSketchSpace of worldGeometry).
        srcPoints = [prof[k] for k in hexVerts]
        verts = [profileSketch.sketchPoints.add(
            profileSketch.modelToSketchSpace(p.worldGeometry)) for p in srcPoints]

        # draw the closed hexagon SHARING those points.
        hexLines = []
        n = len(verts)
        for i in range(n):
            line = lines.addByTwoPoints(verts[i], verts[(i + 1) % n])
            line.isConstruction = False
            hexLines.append(line)

        # fix lines AND endpoints AFTER the lines exist.
        for line in hexLines:
            line.startSketchPoint.isFixed = True
            line.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch, f'{label} Profile')

        # The first edge (verts[0]->verts[1]) is the shaft axis for every body op.
        shaftEdge = hexLines[0]

        # single hexagon loop -> single profile.
        profile = profileSketch.profiles.item(0)

        # ---- revolve about the shaft edge ----
        revolves = designComp.features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{label} Gear Body'

        # ---- loft Apex sketch point -> tooth profile ----
        apexSketchPoint = prof['apexPoint']  # centerToApex.endSketchPoint
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = designComp.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{label} Gear Tooth Body'

        # ---- two interleaved conical cuts ----
        apexWorld = self._pointWorldGeometry(prof['apexPoint'])
        toeMidWorld = self._edgeMidWorld(prof, toeStartKey, toeEndKey)
        heelMidWorld = self._edgeMidWorld(prof, heelStartKey, heelEndKey)

        # cut #1: toe cut on the Tooth Body.
        toePieces = self._applyConicalCut(
            designComp, gearBody, [toothBody], toeMidWorld,
            f'{label} cut#1 toe')
        if len(toePieces) < 2:
            raise Exception(
                f'{label}: toe cut produced {len(toePieces)} piece(s); expected >=2 '
                '(apex tip + keeper)')

        # drop apex tip, keep largest non-apex piece (the keeper).
        keeper = self._selectKeeper(designComp, toePieces, apexWorld, f'{label} toe')

        # cut #2: heel cut on the keeper ALONE (may not intersect -> keep whole).
        try:
            heelPieces = self._applyConicalCut(
                designComp, gearBody, [keeper], heelMidWorld,
                f'{label} cut#2 heel')
        except RuntimeError as e:
            msg = str(e)
            if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                futil.log(f'{label} heel cut: tool did not intersect, kept intact',
                          force_console=True)
                heelPieces = [keeper]
            else:
                raise

        # select tooth (drop any apex piece, keep largest non-apex).
        toothPiece = self._selectKeeper(designComp, heelPieces, apexWorld, f'{label} heel')

        # ---- circular pattern the tooth about the shaft edge ----
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(toothPiece)
        patterns = designComp.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # ---- combine-join all patterned teeth into the gear body ----
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = designComp.features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # ---- bore ----
        if self._boreEnable:
            self._cutBore(designComp, gearBody, shaftEdge, bore_cm, pitchDia_cm)

        # ---- meshing rotation (driving only), in Design before moveToComponent ----
        if meshRotateTeeth is not None:
            startW = shaftEdge.startSketchPoint.worldGeometry
            endW = shaftEdge.endSketchPoint.worldGeometry
            axisVec = startW.vectorTo(endW)
            axisVec.normalize()
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(math.radians(180.0 / meshRotateTeeth), axisVec, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = designComp.features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # ---- relocate finished body into the gear component ----
        gearBody.moveToComponent(gearOcc)

        return gearBody

    # ===================================================================
    # body-build sub-helpers
    # ===================================================================
    def _edgeMidWorld(self, prof, startKey, endKey):
        s = self._pointWorldGeometry(prof[startKey])
        e = self._pointWorldGeometry(prof[endKey])
        return adsk.core.Point3D.create((s.x + e.x) / 2, (s.y + e.y) / 2, (s.z + e.z) / 2)

    def _findSpurToothProfile(self, toothSketch):
        """Match the tooth cross-section loop by curve-TYPE mix.

        Non-embedded: 2 NURBS flanks + 2 arcs (tip/root) + 2 lines.
        Embedded:     2 NURBS + 2 arcs + 0 lines."""
        best = None
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nNurbs = nArc = nLine = nOther = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nNurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        nArc += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        nLine += 1
                    else:
                        nOther += 1
                if nNurbs == 2 and nArc == 2 and nOther == 0 and nLine in (0, 2):
                    return profile
                # remember a near-miss for diagnostics
                best = (nNurbs, nArc, nLine, nOther)
        raise Exception(
            f'Could not find spur tooth profile (best loop type mix nNurbs/nArc/nLine/nOther={best})')

    def _findConeFaceForCutLine(self, frustumBody, edgeMidWorld):
        """Return frustum ConeSurfaceType faces ordered best-first by surface distance
        to the cut-edge midpoint (unevaluable None treated as +inf, tried last)."""
        candidates = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            dist = self._surfaceDistance(face.geometry, edgeMidWorld)
            candidates.append((face, dist))
        candidates.sort(key=lambda fd: (fd[1] is None, fd[1] if fd[1] is not None else 0.0))
        return candidates

    def _applyConicalCut(self, designComp, frustumBody, targetBodies, edgeMidWorld, label):
        """Try each frustum cone face best-first as the split tool; keep the first
        that actually splits a target into >1 piece. Returns the resulting pieces."""
        candidates = self._findConeFaceForCutLine(frustumBody, edgeMidWorld)
        diagnostics = []
        for (face, dist) in candidates:
            attemptedAll = True
            producedPieces = []
            for targetBody in targetBodies:
                splits = designComp.features.splitBodyFeatures
                splitInput = splits.createInput(targetBody, face, True)
                try:
                    splitFeature = splits.add(splitInput)
                except RuntimeError as e:
                    msg = str(e)
                    if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                        attemptedAll = False
                        break
                    raise
                for b in splitFeature.bodies:
                    producedPieces.append(b)
            if attemptedAll and len(producedPieces) > len(targetBodies):
                futil.log(
                    f'{label}: selected cone face dist={dist}; '
                    f'{len(targetBodies)} body(ies) -> {len(producedPieces)} pieces',
                    force_console=True)
                return producedPieces
            diagnostics.append((dist, len(producedPieces) if attemptedAll else 'no-intersect'))

        # No cone face split the target.
        raise Exception(
            f'{label}: no ConeSurfaceType face of the frustum split the target '
            f'(cone faces tried: {diagnostics})')

    def _selectKeeper(self, designComp, pieces, apexWorld, label):
        """Drop every Apex-containing piece, keep the single largest by volume,
        remove the rest."""
        nonApex = []
        apexPieces = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            if (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                    or containment == adsk.fusion.PointContainment.PointOnPointContainment):
                apexPieces.append(body)
            else:
                nonApex.append(body)

        if len(nonApex) == 0:
            raise Exception(
                f'{label}: no non-apex piece after cut '
                f'(total pieces={len(pieces)}, apex pieces={len(apexPieces)})')

        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]

        removes = designComp.features.removeFeatures
        for body in apexPieces + nonApex[1:]:
            removes.add(body)

        return keeper

    def _cutBore(self, designComp, gearBody, shaftEdge, bore_cm, pitchDia_cm):
        boreDiameter_cm = bore_cm if bore_cm > 0 else pitchDia_cm / 4
        if boreDiameter_cm <= 0:
            return

        # bore plane normal to the shaft at its start.
        planeInput = designComp.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComp.constructionPlanes.add(planeInput)
        borePlane.name = 'Bore Plane'

        boreSketch = designComp.sketches.add(borePlane)
        boreSketch.name = 'Bore Profile'
        boreSketch.isVisible = True

        circles = boreSketch.sketchCurves.sketchCircles
        dimensions = boreSketch.sketchDimensions
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2)
        # fix center (do NOT coincident to origin) + diameter dimension.
        circle.centerSketchPoint.isFixed = True
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter_cm / 2, boreDiameter_cm / 2, 0))

        self._assertFullyConstrained(boreSketch, 'Bore Profile')

        profile = boreSketch.profiles.item(0)
        extrudes = designComp.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(to_cm(1000)), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

        boreSketch.isVisible = False

    # ===================================================================
    # cleanup
    # ===================================================================
    def _hideConstructionGeometry(self, bevelComp):
        seen = set()

        def walk(component):
            for sketch in component.sketches:
                token = sketch.entityToken
                if token in seen:
                    continue
                seen.add(token)
                sketch.isVisible = False
            for plane in component.constructionPlanes:
                token = plane.entityToken
                if token in seen:
                    continue
                seen.add(token)
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                token = axis.entityToken
                if token in seen:
                    continue
                seen.add(token)
                axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComp)

    # ===================================================================
    # orchestration
    # ===================================================================
    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # pitch diameters & bores (Python, cm).
        drivingPitchDia_cm = to_cm(module * drivingTeeth)
        pinionPitchDia_cm = to_cm(module * pinionTeeth)
        drivingBore_cm = self._drivingBore_cm
        pinionBore_cm = self._pinionBore_cm

        # ---- build component tree: Bevel Gear -> Design / Pinion / Driving ----
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelComp = self.bevelOccurrence.component
        bevelComp.name = (
            f'Bevel Gear (M={module}, Driving={drivingTeeth}, Pinion={pinionTeeth}, '
            f'Shaft={shaftAngle_deg:.1f} deg)')

        designOcc = bevelComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        designComp = designOcc.component
        designComp.name = 'Design'

        pinionOcc = bevelComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        pinionOcc.component.name = 'Pinion Gear'

        drivingOcc = bevelComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        drivingOcc.component.name = 'Driving Gear'

        # ---- §1 anchor sketch ----
        anchorLine = self._buildAnchorSketch(designComp, targetPlane, centerPoint)

        # ---- §2 + §3 + per-gear body creation ----
        self._buildGearProfiles(
            designComp, bevelComp, pinionOcc, drivingOcc,
            targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDia_cm, pinionPitchDia_cm,
            drivingBore_cm, pinionBore_cm)

        # ---- cleanup ----
        self._hideConstructionGeometry(bevelComp)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None
