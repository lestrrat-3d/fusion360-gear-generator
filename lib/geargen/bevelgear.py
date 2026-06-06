import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the 14 reproduced surface strings, verbatim)
# ---------------------------------------------------------------------------
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER = 'centerPoint'
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
INPUT_ID_TOOTH_SPACING = 'toothSpacing'

# Hardcoded pressure angle for the virtual spur teeth (bevel is not a dialog input).
PRESSURE_ANGLE_RAD = math.radians(20)
INVOLUTE_STEPS = 15


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so it wins Fusion auto-focus.
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point (selection)
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Select the point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component (selection) -- root pre-selected.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the parent component for the new bevel gear pair')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6. Driving Gear Teeth
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 8. Driving Gear Base Height
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 9. Pinion Gear Base Height
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore (checkbox)
        inputs.addBoolValueInput(
            INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11. Driving Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 12. Pinion Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 13. Face Width
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 14. Tooth Spacing
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))


# ---------------------------------------------------------------------------
# Value-wrapper + virtual spur proxy (fake parent for the borrowed spur drawer)
# ---------------------------------------------------------------------------
class _Val:
    """Tiny wrapper exposing a `.value` attribute, mimicking a UserParameter."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """A fake spur `parent` so SpurGearInvoluteToothDesignGenerator can read the
    parameters it needs (via parent.getParameter(name).value) without registering
    any real Fusion user parameters.

    module_mm is the raw-mm Module value; virtualTeeth is the Tredgold/back-cone
    virtual tooth number. All served circle radii/diameters are in internal cm.
    """
    def __init__(self, module_mm, virtualTeeth):
        moduleCm = to_cm(module_mm)
        teeth = int(virtualTeeth)

        pitchDiameter = moduleCm * teeth
        pitchRadius = pitchDiameter / 2
        baseDiameter = pitchDiameter * math.cos(PRESSURE_ANGLE_RAD)
        baseRadius = baseDiameter / 2
        rootDiameter = pitchDiameter - 2.5 * moduleCm
        rootRadius = rootDiameter / 2
        tipDiameter = pitchDiameter + 2 * moduleCm
        tipRadius = tipDiameter / 2

        self._values = {
            'Module': moduleCm,
            'ToothNumber': teeth,
            'PressureAngle': PRESSURE_ANGLE_RAD,
            'PitchCircleDiameter': pitchDiameter,
            'PitchCircleRadius': pitchRadius,
            'BaseCircleDiameter': baseDiameter,
            'BaseCircleRadius': baseRadius,
            'RootCircleDiameter': rootDiameter,
            'RootCircleRadius': rootRadius,
            'TipCircleDiameter': tipDiameter,
            'TipCircleRadius': tipRadius,
            'InvoluteSteps': INVOLUTE_STEPS,
        }
        # The spur drawer records embedded-ness on the parent; absorb it here.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

        # Stashed inputs (set in _readInputs).
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0
        self._toothSpacing_cm = 0.0
        self._faceWidthSpecified = False

        # Stashed center sketch point from the Anchor Sketch (re-projected in §2).
        self._anchorCenterPoint = None

    # ----- error rollback -----
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # ----- full-constraint gate -----
    def _assertFullyConstrained(self, sketch, name):
        if not sketch.isFullyConstrained:
            raise Exception(
                f'Sketch "{name}" is not fully constrained (free DOF remain)')

    # ----- input reading + validation -----
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def rawValue(id, units):
            inp = inputs.itemById(id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # --- selections ---
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

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # --- Module: read with unit '' -> raw number meaning millimetres ---
        module = rawValue(INPUT_ID_MODULE, '')
        if module <= 0:
            raise Exception('Module must be a positive number')

        # --- Teeth ---
        drivingTeeth = int(round(rawValue(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(rawValue(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception('Driving gear must have at least 3 teeth')
        if pinionTeeth < 3:
            raise Exception('Pinion gear must have at least 3 teeth')

        # --- Shaft angle: comes back in radians; convert to degrees to range-check ---
        shaftAngle_rad = rawValue(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception('Shaft angle must be between 30 and 150 degrees')

        # --- 'mm' inputs: already in internal cm -- use as-is ---
        self._drivingBaseHeight_cm = rawValue(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = rawValue(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._drivingBore_cm = rawValue(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = rawValue(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = rawValue(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = rawValue(INPUT_ID_TOOTH_SPACING, 'mm')

        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable

        if self._drivingBaseHeight_cm < 0:
            raise Exception('Driving gear base height must be non-negative')
        if self._pinionBaseHeight_cm < 0:
            raise Exception('Pinion gear base height must be non-negative')
        if self._drivingBore_cm < 0:
            raise Exception('Driving gear bore diameter must be non-negative')
        if self._pinionBore_cm < 0:
            raise Exception('Pinion gear bore diameter must be non-negative')
        if self._faceWidth_cm < 0:
            raise Exception('Face width must be non-negative')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth spacing must be non-negative')

        # Face width "unspecified" means default (0).
        self._faceWidthSpecified = self._faceWidth_cm > 0

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ----- helpers -----
    def _pointWorldGeometry(self, entity):
        """World Point3D for a SketchPoint (worldGeometry) or ConstructionPoint (geometry)."""
        if entity.objectType == adsk.fusion.SketchPoint.classType():
            return entity.worldGeometry
        return entity.geometry

    def _perpDistancePointToLine(self, p, a, b):
        """Perpendicular distance (2-D x/y) from point p to the line through a, b."""
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
        px, py = p.x, p.y
        dx = bx - ax
        dy = by - ay
        denom = math.hypot(dx, dy)
        if denom == 0:
            return math.hypot(px - ax, py - ay)
        return abs(dx * (ay - py) - (ax - px) * dy) / denom

    def _surfaceDistance(self, surface, worldPoint):
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return projected.distanceTo(worldPoint)

    # ----- main entry -----
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Resolve pitch diameters (cm) and bores (cm).
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        if self._drivingBore_cm > 0:
            drivingBore_cm = self._drivingBore_cm
        else:
            drivingBore_cm = drivingPitchDiameter_cm / 4
        if self._pinionBore_cm > 0:
            pinionBore_cm = self._pinionBore_cm
        else:
            pinionBore_cm = pinionPitchDiameter_cm / 4

        # --- Component tree: Bevel Gear under parent, Design under Bevel Gear ---
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # --- §1 Anchor Sketch ---
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # --- §2 + §3 + per-gear body creation ---
        self._buildGearProfiles(
            designComponent, bevelComponent, targetPlane, centerPoint, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm,
            drivingBore_cm, pinionBore_cm)

        # --- Cleanup ---
        self._hideConstructionGeometry(bevelComponent)

    # ----- §1 Anchor Sketch -----
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user-specified center onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        # Create the anchor line through the projected center (~10mm reference line).
        cg = projectedCenter.geometry
        half = to_cm(5)  # ~10mm total length
        p0 = adsk.core.Point3D.create(cg.x - half, cg.y, 0)
        p1 = adsk.core.Point3D.create(cg.x + half, cg.y, 0)
        anchorLine = lines.addByTwoPoints(p0, p1)

        # Pin the center onto the line AND make it bisect.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length dimension (arbitrary reference value).
        midText = adsk.core.Point3D.create(cg.x, cg.y + half, 0)
        lengthDim = dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            midText)
        lengthDim.parameter.value = to_cm(10)

        # Pin the direction sketch-locally so the sketch ends fully constrained.
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch, 'Anchor Sketch')
        return anchorLine

    # ----- §2 + §3 + body creation -----
    def _buildGearProfiles(self, designComponent, bevelComponent, targetPlane,
                           centerPoint, anchorLine, module, drivingTeeth,
                           pinionTeeth, shaftAngle_deg,
                           drivingPitchDiameter_cm, pinionPitchDiameter_cm,
                           drivingBore_cm, pinionBore_cm):
        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm
        Sigma = math.radians(shaftAngle_deg)
        moduleCm = to_cm(module)
        dedendum_cm = to_cm(1.25 * module)
        moduleExt_cm = moduleCm  # module-length construction extensions

        # --- Gear Profiles plane: through anchor line, 90deg off the target plane ---
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the Anchor Sketch's center point (NOT the raw user point).
        projected = sketch.project(self._anchorCenterPoint)
        centerSP = projected.item(0)
        projectedAnchor = sketch.project(anchorLine)
        anchorLineProj = projectedAnchor.item(0)

        c = centerSP.geometry
        # Anchor line 2-D unit direction.
        ag = anchorLineProj.startSketchPoint.geometry
        bg = anchorLineProj.endSketchPoint.geometry
        dvec = adsk.core.Vector3D.create(bg.x - ag.x, bg.y - ag.y, 0)
        dlen = math.hypot(dvec.x, dvec.y)
        dx = dvec.x / dlen
        dy = dvec.y / dlen
        # In-plane perpendicular candidate.
        perpx, perpy = -dy, dx

        # Pick sign by the target-plane normal (one-bit direction), toward the normal.
        targetNormal = get_normal(targetPlane)
        targetNormal.normalize()
        # Map perp into world to compare its direction against the target normal.
        # The perp lies in the gear-profiles plane; compare against target normal
        # by converting two sketch points to world.
        perpWorldA = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x, c.y, 0))
        perpWorldB = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perpx, c.y + perpy, 0))
        perpWorld = adsk.core.Vector3D.create(
            perpWorldB.x - perpWorldA.x,
            perpWorldB.y - perpWorldA.y,
            perpWorldB.z - perpWorldA.z)
        if perpWorld.dotProduct(targetNormal) < 0:
            perpx, perpy = -perpx, -perpy

        # --- Apex: free end of a construction line from center, perp to anchor ---
        apexSeed = adsk.core.Point3D.create(c.x + perpx * DPD, c.y + perpy * DPD, 0)
        centerToApex = lines.addByTwoPoints(centerSP, apexSeed)
        centerToApex.isConstruction = True
        constraints.addPerpendicular(centerToApex, anchorLineProj)
        apexPoint = centerToApex.endSketchPoint

        # --- closed-form cone geometry seeds for Apex->A, Apex->B ---
        # tan(gamma_p) = sin Sigma * PPD / (DPD + PPD cos Sigma)
        gamma_p = math.atan2(math.sin(Sigma) * PPD, DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        coneDist_R = (PPD / 2) / math.sin(gamma_p)
        lenApexA = coneDist_R * math.cos(gamma_p)
        lenApexB = coneDist_R * math.cos(gamma_g)

        # --- Driving Gear Shaft Axis: from apex toward anchor line (-perp), -> B ---
        apexGeom = apexSeed  # seed coordinate for apex
        bSeed = adsk.core.Point3D.create(
            apexGeom.x - perpx * lenApexB, apexGeom.y - perpy * lenApexB, 0)
        drivingShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0), bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis: driving direction rotated about apex by Sigma ---
        # Driving shaft direction (from apex toward B) is -perp.
        ddx, ddy = -perpx, -perpy

        def rotate2d(vx, vy, theta):
            cc = math.cos(theta)
            ss = math.sin(theta)
            return (vx * cc - vy * ss, vx * ss + vy * cc)

        candPlus = rotate2d(ddx, ddy, Sigma)
        candMinus = rotate2d(ddx, ddy, -Sigma)
        aPlus = (apexGeom.x + candPlus[0] * lenApexA,
                 apexGeom.y + candPlus[1] * lenApexA)
        aMinus = (apexGeom.x + candMinus[0] * lenApexA,
                  apexGeom.y + candMinus[1] * lenApexA)
        # Keep the candidate whose endpoint has the GREATER X coordinate.
        if aPlus[0] >= aMinus[0]:
            aSeed = aPlus
            usedPlusSense = True
        else:
            aSeed = aMinus
            usedPlusSense = False

        pinionShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0))
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        pointA = pinionShaftAxis.endSketchPoint

        # Angular dimension Sigma between the two shaft axes, text inside the wedge.
        wedgeMidx, wedgeMidy = rotate2d(ddx, ddy, Sigma / 2.0
                                        if usedPlusSense else -Sigma / 2.0)
        wedgeR = lenApexA * 0.5
        angleText = adsk.core.Point3D.create(
            apexGeom.x + wedgeMidx * wedgeR,
            apexGeom.y + wedgeMidy * wedgeR, 0)
        angDim = dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, angleText)
        angDim.parameter.value = Sigma

        # --- From A, perpendicular drop to Apex2 (length PPD/2) ---
        # Direction toward Apex2 (between the two shafts, toward anchor line).
        # Seed: perpendicular to pinion shaft axis, toward center.
        pinDirx = aSeed[0] - apexGeom.x
        pinDiry = aSeed[1] - apexGeom.y
        plen = math.hypot(pinDirx, pinDiry)
        pinDirx /= plen
        pinDiry /= plen
        # Two perpendiculars to pinion shaft; pick the one pointing toward center c.
        perpAcand1 = (-pinDiry, pinDirx)
        perpAcand2 = (pinDiry, -pinDirx)
        toCenterAx = c.x - aSeed[0]
        toCenterAy = c.y - aSeed[1]
        if (perpAcand1[0] * toCenterAx + perpAcand1[1] * toCenterAy) >= \
           (perpAcand2[0] * toCenterAx + perpAcand2[1] * toCenterAy):
            perpA = perpAcand1
        else:
            perpA = perpAcand2
        apex2SeedA = (aSeed[0] + perpA[0] * (PPD / 2),
                      aSeed[1] + perpA[1] * (PPD / 2))
        aToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(apex2SeedA[0], apex2SeedA[1], 0))
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaftAxis)
        aDropDim = dimensions.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (aSeed[0] + apex2SeedA[0]) / 2,
                (aSeed[1] + apex2SeedA[1]) / 2, 0))
        aDropDim.parameter.value = PPD / 2

        # --- From B, perpendicular drop to Apex2 (length DPD/2) ---
        drvDirx = bSeed.x - apexGeom.x
        drvDiry = bSeed.y - apexGeom.y
        dlen2 = math.hypot(drvDirx, drvDiry)
        drvDirx /= dlen2
        drvDiry /= dlen2
        perpBcand1 = (-drvDiry, drvDirx)
        perpBcand2 = (drvDiry, -drvDirx)
        toCenterBx = c.x - bSeed.x
        toCenterBy = c.y - bSeed.y
        if (perpBcand1[0] * toCenterBx + perpBcand1[1] * toCenterBy) >= \
           (perpBcand2[0] * toCenterBx + perpBcand2[1] * toCenterBy):
            perpB = perpBcand1
        else:
            perpB = perpBcand2
        apex2SeedB = (bSeed.x + perpB[0] * (DPD / 2),
                      bSeed.y + perpB[1] * (DPD / 2))
        bToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0))
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaftAxis)
        bDropDim = dimensions.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (bSeed.x + apex2SeedB[0]) / 2,
                (bSeed.y + apex2SeedB[1]) / 2, 0))
        bDropDim.parameter.value = DPD / 2

        # --- Apex2: coincide the two drop endpoints ---
        constraints.addCoincident(
            aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint

        # --- Pitch Line: Apex -> Apex2 ---
        pitchLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # --- Dedendum lines from Apex2, perpendicular to Pitch Line, len module*1.25 ---
        # Pitch line direction (apex -> apex2).
        plx = apex2SeedB[0] - apexGeom.x
        ply = apex2SeedB[1] - apexGeom.y
        pllen = math.hypot(plx, ply)
        plx /= pllen
        ply /= pllen
        pitchPerp1 = (-ply, plx)
        pitchPerp2 = (ply, -plx)
        # Driving Gear Dedendum (point D): toward anchor line (toward center).
        toCenterApex2x = c.x - apex2SeedB[0]
        toCenterApex2y = c.y - apex2SeedB[1]
        if (pitchPerp1[0] * toCenterApex2x + pitchPerp1[1] * toCenterApex2y) >= \
           (pitchPerp2[0] * toCenterApex2x + pitchPerp2[1] * toCenterApex2y):
            dDir = pitchPerp1
            cDir = pitchPerp2
        else:
            dDir = pitchPerp2
            cDir = pitchPerp1

        dSeed = (apex2SeedB[0] + dDir[0] * dedendum_cm,
                 apex2SeedB[1] + dDir[1] * dedendum_cm)
        drivingDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        ddDim = dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2SeedB[0] + dSeed[0]) / 2,
                                     (apex2SeedB[1] + dSeed[1]) / 2, 0))
        ddDim.parameter.value = dedendum_cm
        pointD = drivingDedendum.endSketchPoint

        cSeed = (apex2SeedB[0] + cDir[0] * dedendum_cm,
                 apex2SeedB[1] + cDir[1] * dedendum_cm)
        pinionDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        cdDim = dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2SeedB[0] + cSeed[0]) / 2,
                                     (apex2SeedB[1] + cSeed[1]) / 2, 0))
        cdDim.parameter.value = dedendum_cm
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex->D (driving), Apex->C (pinion) ---
        drivingRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- E: from A collinear with Apex->A, length module (no dimension) ---
        # Direction along Apex->A.
        aax = aSeed[0] - apexGeom.x
        aay = aSeed[1] - apexGeom.y
        aalen = math.hypot(aax, aay)
        aax /= aalen
        aay /= aalen
        eSeed = (aSeed[0] + aax * moduleExt_cm, aSeed[1] + aay * moduleExt_cm)
        aToE = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0))
        aToE.isConstruction = True
        constraints.addCoincident(aToE.startSketchPoint, pointA)
        constraints.addCollinear(aToE, pinionShaftAxis)
        pointE = aToE.endSketchPoint

        # --- C->E line, perpendicular to A->E ---
        cToE = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0))
        cToE.isConstruction = True
        constraints.addCoincident(cToE.startSketchPoint, pointC)
        constraints.addCoincident(cToE.endSketchPoint, pointE)
        constraints.addPerpendicular(aToE, cToE)

        # --- F: from B collinear with Apex->B, length module ---
        bbx = bSeed.x - apexGeom.x
        bby = bSeed.y - apexGeom.y
        bblen = math.hypot(bbx, bby)
        bbx /= bblen
        bby /= bblen
        fSeed = (bSeed.x + bbx * moduleExt_cm, bSeed.y + bby * moduleExt_cm)
        bToF = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0))
        bToF.isConstruction = True
        constraints.addCoincident(bToF.startSketchPoint, pointB)
        constraints.addCollinear(bToF, drivingShaftAxis)
        pointF = bToF.endSketchPoint

        # --- D->F line, perpendicular to B->F ---
        dToF = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0))
        dToF.isConstruction = True
        constraints.addCoincident(dToF.startSketchPoint, pointD)
        constraints.addCoincident(dToF.endSketchPoint, pointF)
        constraints.addPerpendicular(bToF, dToF)

        # --- G: from E collinear to A->E, length module ---
        gSeed = (eSeed[0] + aax * moduleExt_cm, eSeed[1] + aay * moduleExt_cm)
        eToG = lines.addByTwoPoints(
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0),
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0))
        eToG.isConstruction = True
        constraints.addCoincident(eToG.startSketchPoint, pointE)
        constraints.addCollinear(eToG, aToE)
        pointG = eToG.endSketchPoint

        # --- H: from C, length module, collinear with Apex2->C ---
        # direction along Apex2->C = cDir
        hSeed = (cSeed[0] + cDir[0] * moduleExt_cm, cSeed[1] + cDir[1] * moduleExt_cm)
        cToH = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        cToH.isConstruction = True
        constraints.addCoincident(cToH.startSketchPoint, pointC)
        constraints.addCollinear(cToH, pinionDedendum)
        pointH = cToH.endSketchPoint

        # --- G->H line, perpendicular to E->G ---
        gToH = lines.addByTwoPoints(
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0),
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        gToH.isConstruction = True
        constraints.addCoincident(gToH.startSketchPoint, pointG)
        constraints.addCoincident(gToH.endSketchPoint, pointH)
        constraints.addPerpendicular(eToG, gToH)

        # --- I: from F collinear to B->F, length module ---
        iSeed = (fSeed[0] + bbx * moduleExt_cm, fSeed[1] + bby * moduleExt_cm)
        fToI = lines.addByTwoPoints(
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0),
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0))
        fToI.isConstruction = True
        constraints.addCoincident(fToI.startSketchPoint, pointF)
        constraints.addCollinear(fToI, bToF)
        pointI = fToI.endSketchPoint

        # --- J: from D, length module, collinear with Apex2->D ---
        jSeed = (dSeed[0] + dDir[0] * moduleExt_cm, dSeed[1] + dDir[1] * moduleExt_cm)
        dToJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        dToJ.isConstruction = True
        constraints.addCoincident(dToJ.startSketchPoint, pointD)
        constraints.addCollinear(dToJ, drivingDedendum)
        pointJ = dToJ.endSketchPoint

        # --- I->J line, perpendicular to F->I ---
        iToJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0),
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        iToJ.isConstruction = True
        constraints.addCoincident(iToJ.startSketchPoint, pointI)
        constraints.addCoincident(iToJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(fToI, iToJ)

        # --- Offset dimension: B->Apex2 drop vs J->I ; value = driving base height ---
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeightVal = self._drivingBaseHeight_cm
        else:
            drivingBaseHeightVal = to_cm(module * drivingTeeth / 8)
        jiOffsetDim = dimensions.addOffsetDimension(
            bToApex2, iToJ,
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        jiOffsetDim.parameter.value = drivingBaseHeightVal

        # --- Offset dimension: A->Apex2 drop vs G->H ; value = pinion base height ---
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeightVal = self._pinionBaseHeight_cm
        else:
            pinionBaseHeightVal = drivingBaseHeightVal * (pinionTeeth / drivingTeeth)
        ghOffsetDim = dimensions.addOffsetDimension(
            aToApex2, gToH,
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        ghOffsetDim.parameter.value = pinionBaseHeightVal

        # --- Line A->G ---
        aToG = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0))
        aToG.isConstruction = True
        constraints.addCoincident(aToG.startSketchPoint, pointA)
        constraints.addCoincident(aToG.endSketchPoint, pointG)

        # --- Constrain Point I with center point ---
        constraints.addCoincident(pointI, centerSP)

        # --- K: construction line from G away from Apex, along Apex->A, end K ---
        # direction away from apex along Apex->A = (aax, aay)
        kSeed = (gSeed[0] + aax * moduleExt_cm, gSeed[1] + aay * moduleExt_cm)
        gToK = lines.addByTwoPoints(
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0),
            adsk.core.Point3D.create(kSeed[0], kSeed[1], 0))
        gToK.isConstruction = True
        constraints.addCoincident(gToK.startSketchPoint, pointG)
        pointK = gToK.endSketchPoint
        # Pin K with two point-on-line coincidents (NOT addCollinear).
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)

        # Reference line C->K.
        cToK = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(kSeed[0], kSeed[1], 0))
        cToK.isConstruction = True
        constraints.addCoincident(cToK.startSketchPoint, pointC)
        constraints.addCoincident(cToK.endSketchPoint, pointK)

        # --- Tooth-center point K' (Tooth Spacing offset) ---
        # When spacing == 0: K' == K, reference line C->K. Build nothing extra.
        if self._toothSpacing_cm > 0:
            # Shift K outward along the dedendum direction, away from C.
            # dedendum direction outward = cDir (Apex2 -> C direction), continuing past K.
            kpSeed = (kSeed[0] + cDir[0] * self._toothSpacing_cm,
                      kSeed[1] + cDir[1] * self._toothSpacing_cm)
            kToKp = lines.addByTwoPoints(
                adsk.core.Point3D.create(kSeed[0], kSeed[1], 0),
                adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
            kToKp.isConstruction = True
            constraints.addCoincident(kToKp.startSketchPoint, pointK)
            pointKp = kToKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            kpDim = dimensions.addDistanceDimension(
                kToKp.startSketchPoint, kToKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create((kSeed[0] + kpSeed[0]) / 2,
                                         (kSeed[1] + kpSeed[1]) / 2, 0))
            kpDim.parameter.value = self._toothSpacing_cm
            # Tooth-center reference line C->K'.
            cToKp = lines.addByTwoPoints(
                adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
                adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
            cToKp.isConstruction = True
            constraints.addCoincident(cToKp.startSketchPoint, pointC)
            constraints.addCoincident(cToKp.endSketchPoint, pointKp)
            pinionCenterPoint = pointKp
            pinionCenterRefLine = cToKp
        else:
            pinionCenterPoint = pointK
            pinionCenterRefLine = cToK

        # --- Resolve Maximum Face Width from SOLVED geometry ---
        aGeo = pointA.geometry
        bGeo = pointB.geometry
        cGeo = pointC.geometry
        dGeo = pointD.geometry
        hGeo = pointH.geometry
        jGeo = pointJ.geometry
        distA = self._perpDistancePointToLine(aGeo, cGeo, hGeo)
        distB = self._perpDistancePointToLine(bGeo, dGeo, jGeo)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        if self._faceWidthSpecified:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face width {:.3f} mm exceeds maximum {:.3f} mm'.format(
                        to_mm(self._faceWidth_cm), to_mm(maxFaceWidth_cm)))
            faceWidth_cm = self._faceWidth_cm
        else:
            coneDistance_cm = to_cm(math.sqrt(
                (module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
            faceWidth_cm = min(coneDistance_cm / 6, maxFaceWidth_cm)

        # --- M->N (pinion toe line) ---
        # C->H direction.
        chx = hGeo.x - cGeo.x
        chy = hGeo.y - cGeo.y
        chlen = math.hypot(chx, chy)
        chx /= chlen
        chy /= chlen
        # Seed M at midpoint of Apex->C.
        apexSolved = apexPoint.geometry
        mSeed = ((apexSolved.x + cGeo.x) / 2, (apexSolved.y + cGeo.y) / 2)
        # Seed N by sliding from M-seed along C->H by ~distance M-seed to A.
        slideN = math.hypot(mSeed[0] - aGeo.x, mSeed[1] - aGeo.y)
        nSeed = (mSeed[0] + chx * slideN, mSeed[1] + chy * slideN)
        mToN = lines.addByTwoPoints(
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0),
            adsk.core.Point3D.create(nSeed[0], nSeed[1], 0))
        mToN.isConstruction = True
        pointM = mToN.startSketchPoint
        pointN = mToN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, aToApex2)
        constraints.addParallel(mToN, cToH)
        mnOffsetDim = dimensions.addOffsetDimension(
            cToH, mToN,
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0))
        mnOffsetDim.parameter.value = faceWidth_cm

        # M->C and N->A connecting lines.
        mToC = lines.addByTwoPoints(
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        mToC.isConstruction = True
        constraints.addCoincident(mToC.startSketchPoint, pointM)
        constraints.addCoincident(mToC.endSketchPoint, pointC)
        nToA = lines.addByTwoPoints(
            adsk.core.Point3D.create(nSeed[0], nSeed[1], 0),
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0))
        nToA.isConstruction = True
        constraints.addCoincident(nToA.startSketchPoint, pointN)
        constraints.addCoincident(nToA.endSketchPoint, pointA)

        # --- L: construction line from I away from Apex, along Apex->B, end L ---
        lSeed = (iSeed[0] + bbx * moduleExt_cm, iSeed[1] + bby * moduleExt_cm)
        iToL = lines.addByTwoPoints(
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0),
            adsk.core.Point3D.create(lSeed[0], lSeed[1], 0))
        iToL.isConstruction = True
        constraints.addCoincident(iToL.startSketchPoint, pointI)
        pointL = iToL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)

        dToL = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(lSeed[0], lSeed[1], 0))
        dToL.isConstruction = True
        constraints.addCoincident(dToL.startSketchPoint, pointD)
        constraints.addCoincident(dToL.endSketchPoint, pointL)

        # --- Tooth-center point L' (Tooth Spacing offset) ---
        if self._toothSpacing_cm > 0:
            lpSeed = (lSeed[0] + dDir[0] * self._toothSpacing_cm,
                      lSeed[1] + dDir[1] * self._toothSpacing_cm)
            lToLp = lines.addByTwoPoints(
                adsk.core.Point3D.create(lSeed[0], lSeed[1], 0),
                adsk.core.Point3D.create(lpSeed[0], lpSeed[1], 0))
            lToLp.isConstruction = True
            constraints.addCoincident(lToLp.startSketchPoint, pointL)
            pointLp = lToLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            lpDim = dimensions.addDistanceDimension(
                lToLp.startSketchPoint, lToLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create((lSeed[0] + lpSeed[0]) / 2,
                                         (lSeed[1] + lpSeed[1]) / 2, 0))
            lpDim.parameter.value = self._toothSpacing_cm
            dToLp = lines.addByTwoPoints(
                adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
                adsk.core.Point3D.create(lpSeed[0], lpSeed[1], 0))
            dToLp.isConstruction = True
            constraints.addCoincident(dToLp.startSketchPoint, pointD)
            constraints.addCoincident(dToLp.endSketchPoint, pointLp)
            drivingCenterPoint = pointLp
            drivingCenterRefLine = dToLp
        else:
            drivingCenterPoint = pointL
            drivingCenterRefLine = dToL

        # --- O->P (driving toe line, mirror of M->N) ---
        djx = jGeo.x - dGeo.x
        djy = jGeo.y - dGeo.y
        djlen = math.hypot(djx, djy)
        djx /= djlen
        djy /= djlen
        oSeed = ((apexSolved.x + dGeo.x) / 2, (apexSolved.y + dGeo.y) / 2)
        slideP = math.hypot(oSeed[0] - bGeo.x, oSeed[1] - bGeo.y)
        pSeed = (oSeed[0] + djx * slideP, oSeed[1] + djy * slideP)
        oToP = lines.addByTwoPoints(
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0),
            adsk.core.Point3D.create(pSeed[0], pSeed[1], 0))
        oToP.isConstruction = True
        pointO = oToP.startSketchPoint
        pointP = oToP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, bToApex2)
        constraints.addParallel(oToP, dToJ)
        opOffsetDim = dimensions.addOffsetDimension(
            dToJ, oToP,
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0))
        opOffsetDim.parameter.value = faceWidth_cm

        oToD = lines.addByTwoPoints(
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        oToD.isConstruction = True
        constraints.addCoincident(oToD.startSketchPoint, pointO)
        constraints.addCoincident(oToD.endSketchPoint, pointD)
        pToB = lines.addByTwoPoints(
            adsk.core.Point3D.create(pSeed[0], pSeed[1], 0),
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0))
        pToB.isConstruction = True
        constraints.addCoincident(pToB.startSketchPoint, pointP)
        constraints.addCoincident(pToB.endSketchPoint, pointB)

        # Line B->I.
        bToI = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0))
        bToI.isConstruction = True
        constraints.addCoincident(bToI.startSketchPoint, pointB)
        constraints.addCoincident(bToI.endSketchPoint, pointI)

        # Gate: the §2 sketch must be fully constrained.
        self._assertFullyConstrained(sketch, 'Gear Profiles')

        # --- §3: virtual spur tooth profiles ---
        # gamma_p, gamma_g already computed above.
        pinionVirtualPitchRadius_cm = (PPD / 2) / math.cos(gamma_p)
        pinionVirtualTeeth = int(math.floor(
            2 * pinionVirtualPitchRadius_cm / moduleCm))
        drivingVirtualPitchRadius_cm = (DPD / 2) / math.cos(gamma_g)
        drivingVirtualTeeth = int(math.floor(
            2 * drivingVirtualPitchRadius_cm / moduleCm))

        # apex sketch point used as the loft degenerate section.
        apexSketchPoint = centerToApex.endSketchPoint

        (pinionToothPlane, pinionToothSketch, pinionToothAxis) = \
            self._buildVirtualSpurProfile(
                designComponent, sketch, gearProfilesPlane,
                pinionCenterRefLine, pinionCenterPoint,
                module, pinionVirtualTeeth, 'Pinion Tooth')

        (drivingToothPlane, drivingToothSketch, drivingToothAxis) = \
            self._buildVirtualSpurProfile(
                designComponent, sketch, gearProfilesPlane,
                drivingCenterRefLine, drivingCenterPoint,
                module, drivingVirtualTeeth, 'Driving Tooth')

        # --- Pinion Gear body ---
        pinionGearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        pinionGearOccurrence.component.name = 'Pinion Gear'

        self._createGearBody(
            designComponent, gearProfilesPlane, sketch,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            apexSketchPoint, pinionToothSketch,
            (pointM, pointN), (pointC, pointH),
            pinionTeeth, pinionBore_cm,
            'Pinion', pinionGearOccurrence, meshRotation=0.0)

        # --- Driving Gear body ---
        drivingGearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        drivingGearOccurrence.component.name = 'Driving Gear'

        drivingMeshRotation = math.radians(180.0 / drivingTeeth)
        self._createGearBody(
            designComponent, gearProfilesPlane, sketch,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            apexSketchPoint, drivingToothSketch,
            (pointO, pointP), (pointD, pointJ),
            drivingTeeth, drivingBore_cm,
            'Driving', drivingGearOccurrence, meshRotation=drivingMeshRotation)

    # ----- §3 helper: virtual spur tooth profile -----
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesSketch,
                                 gearProfilesPlane, centerRefLine, centerPoint,
                                 module, virtualTeeth, label):
        # Plane that includes centerRefLine, perpendicular to the Gear Profiles plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerRefLine, adsk.core.ValueInput.createByString('90 deg'),
            gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = f'{label} Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{label} Profile'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        self._assertFullyConstrained(toothSketch, f'{label} Profile')

        # Construction axis through centerPoint normal to the tooth plane, via setByTwoPlanes:
        # the Gear Profiles plane intersected with a helper plane perpendicular to
        # centerRefLine at its far end (setByDistanceOnPath at 1.0).
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            centerRefLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = f'{label} Helper Plane'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Axis'

        return (toothPlane, toothSketch, toothAxis)

    # ----- per-gear body creation -----
    def _createGearBody(self, designComponent, gearProfilesPlane, gearProfilesSketch,
                        hexVertices, apexSketchPoint, toothSketch,
                        toeEdgePoints, heelEdgePoints,
                        teethNumber, bore_cm, gearLabel, targetOccurrence,
                        meshRotation):
        features = designComponent.features

        # --- Profile sketch with recreated fixed vertices ---
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        # Recreate the six vertices at their world-mapped positions.
        verts = []
        for src in hexVertices:
            local = profileSketch.modelToSketchSpace(src.worldGeometry)
            verts.append(profileSketch.sketchPoints.add(local))

        # Draw the closed hexagon sharing those points (A->G->H->C->M->N->A style).
        hexLines = []
        for i in range(len(verts)):
            nxt = (i + 1) % len(verts)
            hexLines.append(lines.addByTwoPoints(verts[i], verts[nxt]))

        # Fix endpoints AFTER the lines exist.
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch, f'{gearLabel} Profile')

        # The shaft axis is this profile sketch's FIRST edge.
        shaftAxisEdge = hexLines[0]

        # Single profile (one hexagon loop).
        profile = profileSketch.profiles.item(0)

        # --- Revolve around the shaft axis edge ---
        revolves = features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # --- Loft apex point -> tooth profile ---
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth Body'

        # --- Apex world point (for containment tests) ---
        apexWorld = apexSketchPoint.worldGeometry

        # --- Conical cuts: toe then heel, with keeper selection between ---
        toe_m = self._pointWorldGeometry(toeEdgePoints[0])
        toe_n = self._pointWorldGeometry(toeEdgePoints[1])
        toeMid = adsk.core.Point3D.create(
            (toe_m.x + toe_n.x) / 2, (toe_m.y + toe_n.y) / 2,
            (toe_m.z + toe_n.z) / 2)

        heel_c = self._pointWorldGeometry(heelEdgePoints[0])
        heel_h = self._pointWorldGeometry(heelEdgePoints[1])
        heelMid = adsk.core.Point3D.create(
            (heel_c.x + heel_h.x) / 2, (heel_c.y + heel_h.y) / 2,
            (heel_c.z + heel_h.z) / 2)

        # Cut 1: toe cut on the Tooth Body using a cone face of the Gear Body (frustum).
        toePieces = self._applyConicalCut(
            designComponent, gearBody, [toothBody], toeMid,
            f'{gearLabel} toe cut#1')

        # Drop the apex tip; keep the largest non-apex piece (the keeper).
        keeper = self._selectKeeper(designComponent, toePieces, apexWorld,
                                    f'{gearLabel} toe keeper')

        # Cut 2: heel cut on the keeper alone. May not intersect (try/except).
        try:
            heelPieces = self._applyConicalCut(
                designComponent, gearBody, [keeper], heelMid,
                f'{gearLabel} heel cut#2')
            tooth = self._selectKeeper(
                designComponent, heelPieces, apexWorld,
                f'{gearLabel} heel keeper')
        except RuntimeError as e:
            msg = str(e)
            if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                futil.log(
                    f'{gearLabel} heel cut: tool did not intersect, kept intact',
                    force_console=True)
                tooth = keeper
            else:
                raise

        # --- Circular-pattern the tooth around the shaft axis edge ---
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(tooth)
        patterns = features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # --- Combine-join the patterned teeth with the Gear Body ---
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ---
        if self._boreEnable and bore_cm > 0:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel)

        # --- Meshing rotation (driving gear only) ---
        if meshRotation != 0.0:
            startW = shaftAxisEdge.startSketchPoint.worldGeometry
            endW = shaftAxisEdge.endSketchPoint.worldGeometry
            axisVec = adsk.core.Vector3D.create(
                endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
            axisVec.normalize()
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(meshRotation, axisVec, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- Relocate finished body into the gear component ---
        gearBody.moveToComponent(targetOccurrence)

    # ----- find the spur tooth cross-section profile -----
    def _findSpurToothProfile(self, toothSketch):
        # Match by curve-type mix: 2 NURBS flanks + 2 arcs (tip/root) + 2 lines
        # (non-embedded), or 2 NURBS + 2 arcs + 0 lines (embedded).
        line_t = adsk.core.Curve3DTypes.Line3DCurveType
        arc_t = adsk.core.Curve3DTypes.Arc3DCurveType
        nurbs_t = adsk.core.Curve3DTypes.NurbsCurve3DCurveType
        for prof in toothSketch.profiles:
            for loop in prof.profileLoops:
                nurbs = arcs = others = lines = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == nurbs_t:
                        nurbs += 1
                    elif ct == arc_t:
                        arcs += 1
                    elif ct == line_t:
                        lines += 1
                    else:
                        others += 1
                if others != 0:
                    continue
                if nurbs == 2 and arcs == 2 and (lines == 2 or lines == 0):
                    return prof
        raise Exception('Could not find spur tooth cross-section profile')

    # ----- conical cut: split target bodies by best cone face of the frustum -----
    def _applyConicalCut(self, designComponent, frustumBody, targets,
                         edgeMidWorld, cutLabel):
        coneType = adsk.core.SurfaceTypes.ConeSurfaceType
        coneFaces = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType == coneType:
                coneFaces.append(face)

        # Order cone faces by surface distance to the edge midpoint (None -> +inf).
        def faceKey(face):
            d = self._surfaceDistance(face.geometry, edgeMidWorld)
            return d if d is not None else float('inf')

        coneFaces.sort(key=faceKey)

        diag = []
        for target in targets:
            for face in coneFaces:
                d = self._surfaceDistance(face.geometry, edgeMidWorld)
                try:
                    splits = designComponent.features.splitBodyFeatures
                    splitInput = splits.createInput(target, face, True)
                    split = splits.add(splitInput)
                    pieces = []
                    for b in split.bodies:
                        pieces.append(b)
                    if len(pieces) > 1:
                        futil.log(
                            f'{cutLabel}: split with face dist={d} -> '
                            f'{len(pieces)} pieces', force_console=True)
                        return pieces
                    # Did not split (one piece) -- continue trying other faces.
                    diag.append((d, len(pieces)))
                except RuntimeError as e:
                    msg = str(e)
                    if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                        diag.append((d, 'not-intersect'))
                        continue
                    raise

        raise Exception(
            f'{cutLabel}: no cone face split the target '
            f'(cone faces tried={len(coneFaces)}, results={diag})')

    # ----- keeper/tooth selection: drop apex pieces, keep largest by volume -----
    def _selectKeeper(self, designComponent, pieces, apexWorld, label):
        nonApex = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            if (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                    or containment ==
                    adsk.fusion.PointContainment.PointOnPointContainment):
                # apex-containing piece: remove it.
                designComponent.features.removeFeatures.add(body)
            else:
                nonApex.append(body)

        if len(nonApex) == 0:
            raise Exception(f'{label}: no non-apex piece found')

        # Keep the single largest by volume; remove the rest.
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for body in nonApex[1:]:
            designComponent.features.removeFeatures.add(body)
        return keeper

    # ----- bore cut -----
    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel):
        # Bore plane normal to the shaft at its start (distance 0 on the A->G edge).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{gearLabel} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore'
        boreSketch.isVisible = True
        circles = boreSketch.sketchCurves.sketchCircles
        dimensions = boreSketch.sketchDimensions

        radius = bore_cm / 2
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0)).parameter.value = bore_cm

        self._assertFullyConstrained(boreSketch, f'{gearLabel} Bore')

        profile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(to_cm(1000)), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # ----- cleanup: hide construction geometry -----
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                sketch.isLightBulbOn = False
                sketch.isVisible = False
            for plane in component.constructionPlanes:
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                axis.isLightBulbOn = False

            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
