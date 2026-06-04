import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator

import adsk.core, adsk.fusion


# --- Dialog input ids (the only module-level constants; no user parameters) ---
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

# Bevel hardcodes a 20-degree pressure angle (not a dialog input).
PRESSURE_ANGLE = math.radians(20)


# =============================================================================
# Command dialog
# =============================================================================
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first so it wins Fusion's auto-focus)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the driving gear bottom sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        # 2. Center Point
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Select the point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1)

        # 3. Parent Component (pre-selected to the root component)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

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

        # 10. Enable Bore
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


# =============================================================================
# Virtual spur proxy — a fake "parent" so the borrowed spur tooth generator can
# run without registering Fusion user parameters. It precomputes the keys the
# spur drawer reads (in internal cm) and serves them wrapped in _Val.
# =============================================================================
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        # Module arrives in mm. Standard spur formulas in mm, then to_cm for the
        # circle radii/diameters the spur tooth generator expects (cm).
        pitch_mm = module_mm * virtualTeeth
        base_mm = pitch_mm * math.cos(PRESSURE_ANGLE)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._params = {
            'Module': to_cm(module_mm),
            'ToothNumber': virtualTeeth,
            'PressureAngle': PRESSURE_ANGLE,
            'PitchCircleDiameter': to_cm(pitch_mm),
            'PitchCircleRadius': to_cm(pitch_mm / 2.0),
            'BaseCircleDiameter': to_cm(base_mm),
            'BaseCircleRadius': to_cm(base_mm / 2.0),
            'RootCircleDiameter': to_cm(root_mm),
            'RootCircleRadius': to_cm(root_mm / 2.0),
            'TipCircleDiameter': to_cm(tip_mm),
            'TipCircleRadius': to_cm(tip_mm / 2.0),
            'InvoluteSteps': 15,
        }

    def getParameter(self, name):
        return _Val(self._params[name])


# =============================================================================
# Generator
# =============================================================================
class BevelGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # --- error rollback (entry point calls on exception) ---------------------
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # Abstract base requires this signature; we override generate fully below.
    def generate(self, inputs: adsk.core.CommandInputs):
        return self._generate(inputs)

    # =========================================================================
    # input reading
    # =========================================================================
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Parent component
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one parent component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parent.component
        elif parent.objectType == adsk.fusion.Component.classType():
            parentComponent = parent
        else:
            raise Exception('Selected parent is not a component')

        # Target plane
        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        targetPlane = planes[0]

        # Center point
        (centers, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # Module — read with '' so it comes back as a raw number meaning mm.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Teeth
        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Both gears must have at least 3 teeth')

        # Shaft angle — evaluateExpression(..., 'deg') returns radians.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                'Shaft Angle must be between 30 and 150 degrees (got {:.2f})'.format(
                    shaftAngle_deg))

        # 'mm' inputs already come back internal (cm). Do NOT to_cm again.
        self._drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')

        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')

        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # =========================================================================
    # small geometry helpers
    # =========================================================================
    def _pointWorldGeometry(self, p):
        # SketchPoint via .worldGeometry; ConstructionPoint via .geometry.
        if p.objectType == adsk.fusion.SketchPoint.classType():
            return p.worldGeometry
        return p.geometry

    def _surfaceDistance(self, surface, worldPoint):
        # Distance from a (possibly infinite) surface to a world point, via the
        # evaluator. Returns None when the point can't be projected (e.g. near
        # a cone's apex singularity).
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, onSurface) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return onSurface.distanceTo(worldPoint)

    # =========================================================================
    # main orchestration
    # =========================================================================
    def _generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Pitch diameters (Module is mm -> to_cm), cm.
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        # Resolve bores (cm). 0 => auto = pitch diameter / 4.
        drivingBore_cm = self._drivingBore_cm if self._drivingBore_cm > 0 \
            else drivingPitchDiameter_cm / 4.0
        pinionBore_cm = self._pinionBore_cm if self._pinionBore_cm > 0 \
            else pinionPitchDiameter_cm / 4.0

        # --- Component tree: Bevel Gear -> Design (+ per-gear in _createGearBody)
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # §1 Anchor sketch
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear bodies
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm,
            drivingBore_cm, pinionBore_cm)

        # Cleanup
        self._hideConstructionGeometry(bevelComponent)

    # =========================================================================
    # full-constraint gate
    # =========================================================================
    def _assertFullyConstrained(self, sketch):
        if not sketch.isFullyConstrained:
            raise RuntimeError(
                f"sketch '{sketch.name}' is not fully constrained")

    # =========================================================================
    # §1 Anchor Sketch
    # =========================================================================
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Sketch directly on the user-selected target plane (NO normalization).
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user's center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        c = projectedCenter.geometry

        # Anchor line through the projected center (~10mm reference length).
        half = to_cm(5)
        anchorLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x - half, c.y, 0),
            adsk.core.Point3D.create(c.x + half, c.y, 0))

        # Both: coincident (center on line) AND midpoint (center bisects).
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length dimension (arbitrary reference).
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + half, 0)).parameter.value = to_cm(10)

        # Pin direction: sketch-local Horizontal (works on tilted planes).
        constraints.addHorizontal(anchorLine)

        # Stash the projected-center SketchPoint for §2 re-projection.
        self._anchorCenterPoint = projectedCenter

        self._assertFullyConstrained(sketch)
        return anchorLine

    # =========================================================================
    # §2 Gear Profiles + §3 tooth profiles + per-gear bodies
    # =========================================================================
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane,
                           anchorLine, module, drivingTeeth, pinionTeeth,
                           shaftAngle_deg, DPD, PPD, drivingBore_cm, pinionBore_cm):
        # DPD = driving pitch diameter (cm); PPD = pinion pitch diameter (cm).
        sigma = math.radians(shaftAngle_deg)

        def dot2(a, b):
            return a[0] * b[0] + a[1] * b[1]

        def normalize2(vx, vy):
            n = math.hypot(vx, vy)
            return (vx / n, vy / n)

        # --- Gear Profiles plane: setByAngle(anchorLine, '90 deg', targetPlane)
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the Anchor Sketch's stashed center point (NOT the raw point).
        projected = sketch.project(self._anchorCenterPoint)
        projectedCenter = projected.item(0)
        c = projectedCenter.geometry

        # 2-D direction of the projected anchor line.
        projAnchor = sketch.project(anchorLine)
        anchorLine2D = projAnchor.item(0)
        p1 = anchorLine2D.startSketchPoint.geometry
        p2 = anchorLine2D.endSketchPoint.geometry
        d = normalize2(p2.x - p1.x, p2.y - p1.y)

        # In-plane perpendicular; sign chosen by the target NORMAL (one bit).
        perp = (-d[1], d[0])
        nWorld = get_normal(targetPlane)
        P = projectedCenter.worldGeometry
        o2 = sketch.modelToSketchSpace(P)
        n2 = sketch.modelToSketchSpace(
            adsk.core.Point3D.create(P.x + nWorld.x, P.y + nWorld.y, P.z + nWorld.z))
        nDir2D = (n2.x - o2.x, n2.y - o2.y)
        if dot2(perp, nDir2D) <= 0:
            perp = (-perp[0], -perp[1])

        # Apex = c + perp * DPD (sketch-local). centerToApex construction line.
        apexSeed = adsk.core.Point3D.create(
            c.x + perp[0] * DPD, c.y + perp[1] * DPD, 0)
        centerToApex = lines.addByTwoPoints(c, apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, anchorLine2D)
        apex = centerToApex.endSketchPoint  # the Apex sketch point

        # closed-form cone seeds for Apex->A / Apex->B
        # tan gamma_p = sin Sigma * PPD / (DPD + PPD cos Sigma); gamma_g = Sigma - gamma_p
        gamma_p = math.atan2(math.sin(sigma) * PPD,
                             (DPD + PPD * math.cos(sigma)))
        gamma_g = sigma - gamma_p
        R = (PPD / 2.0) / math.sin(gamma_p)        # cone distance
        lenA = R * math.cos(gamma_p)               # |Apex->A|
        lenB = R * math.cos(gamma_g)               # |Apex->B|

        apexGeom = apexSeed  # local apex coords

        # Driving Gear Shaft Axis: from apex toward anchor line (-perp side),
        # parallel to centerToApex, ends at B.
        bSeed = adsk.core.Point3D.create(
            apexGeom.x - perp[0] * lenB, apexGeom.y - perp[1] * lenB, 0)
        drivingShaftAxis = lines.addByTwoPoints(apexGeom, bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Gear Shaft Axis: from apex, at Shaft Angle to driving shaft,
        # ends at A, drawn on the +X-ish side (toward anchor leading dir).
        # Seed A by rotating the -perp direction by gamma offset along the cone.
        # Use closed-form: pinion shaft direction = rotate driving dir by sigma
        # in the plane spanned by (-perp) and d.
        # driving dir (apex->B) ~ -perp ; rotate toward +d by sigma.
        ddir = (-perp[0], -perp[1])
        cosS = math.cos(sigma)
        sinS = math.sin(sigma)
        # rotate ddir toward the +d half-plane (pinion on the leading side)
        aDir = (ddir[0] * cosS + d[0] * sinS, ddir[1] * cosS + d[1] * sinS)
        aDir = normalize2(aDir[0], aDir[1])
        aSeed = adsk.core.Point3D.create(
            apexGeom.x + aDir[0] * lenA, apexGeom.y + aDir[1] * lenA, 0)
        pinionShaftAxis = lines.addByTwoPoints(apexGeom, aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        pointA = pinionShaftAxis.endSketchPoint
        # Angular dimension Sigma between the two shaft axes; text inside wedge.
        wedgeText = adsk.core.Point3D.create(
            apexGeom.x + (aDir[0] + ddir[0]) * (lenA * 0.25),
            apexGeom.y + (aDir[1] + ddir[1]) * (lenA * 0.25), 0)
        dims.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, wedgeText).parameter.value = sigma

        # From A: perpendicular drop of length PPD/2 toward Apex2 side.
        # Seed toward the interior (toward -aDir-ish / toward driving shaft).
        aPerpDir = (-aDir[1], aDir[0])
        # choose the sense pointing toward the driving shaft (interior)
        if dot2(aPerpDir, ddir) < 0:
            aPerpDir = (-aPerpDir[0], -aPerpDir[1])
        apex2SeedFromA = adsk.core.Point3D.create(
            aSeed.x + aPerpDir[0] * (PPD / 2.0),
            aSeed.y + aPerpDir[1] * (PPD / 2.0), 0)
        lineA_Apex2 = lines.addByTwoPoints(aSeed, apex2SeedFromA)
        lineA_Apex2.isConstruction = True
        constraints.addCoincident(lineA_Apex2.startSketchPoint, pointA)
        constraints.addPerpendicular(lineA_Apex2, pinionShaftAxis)
        dims.addDistanceDimension(
            lineA_Apex2.startSketchPoint, lineA_Apex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            apex2SeedFromA).parameter.value = PPD / 2.0

        # From B: perpendicular drop of length DPD/2 toward Apex2 side.
        bPerpDir = (-ddir[1], ddir[0])
        if dot2(bPerpDir, aDir) < 0:
            bPerpDir = (-bPerpDir[0], -bPerpDir[1])
        apex2SeedFromB = adsk.core.Point3D.create(
            bSeed.x + bPerpDir[0] * (DPD / 2.0),
            bSeed.y + bPerpDir[1] * (DPD / 2.0), 0)
        lineB_Apex2 = lines.addByTwoPoints(bSeed, apex2SeedFromB)
        lineB_Apex2.isConstruction = True
        constraints.addCoincident(lineB_Apex2.startSketchPoint, pointB)
        constraints.addPerpendicular(lineB_Apex2, drivingShaftAxis)
        dims.addDistanceDimension(
            lineB_Apex2.startSketchPoint, lineB_Apex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            apex2SeedFromB).parameter.value = DPD / 2.0

        # Apex2 = coincidence of the two perpendicular drop endpoints.
        constraints.addCoincident(
            lineA_Apex2.endSketchPoint, lineB_Apex2.endSketchPoint)
        apex2 = lineA_Apex2.endSketchPoint

        # Pitch Line: Apex -> Apex2.
        pitchLine = lines.addByTwoPoints(apexGeom, apex2.geometry)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # Dedendum lines from Apex2, length Module*1.25, perpendicular to Pitch.
        dedendum_cm = to_cm(1.25 * module)
        a2 = apex2.geometry
        pdir = normalize2(a2.x - apexGeom.x, a2.y - apexGeom.y)  # apex->apex2
        pperp = (-pdir[1], pdir[0])
        # Driving Gear Dedendum -> point D : drawn toward the anchor line.
        # Pinion Gear Dedendum -> point C : drawn away from the anchor line.
        # "toward anchor line" = -perp side; "away" = +perp side.
        if dot2(pperp, perp) > 0:
            cPerp = pperp                  # away from anchor (toward apex side)
            dPerp = (-pperp[0], -pperp[1])  # toward anchor line
        else:
            cPerp = (-pperp[0], -pperp[1])
            dPerp = pperp
        cSeed = adsk.core.Point3D.create(
            a2.x + cPerp[0] * dedendum_cm, a2.y + cPerp[1] * dedendum_cm, 0)
        pinionDedendum = lines.addByTwoPoints(a2, cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            cSeed).parameter.value = dedendum_cm
        pointC = pinionDedendum.endSketchPoint

        dSeed = adsk.core.Point3D.create(
            a2.x + dPerp[0] * dedendum_cm, a2.y + dPerp[1] * dedendum_cm, 0)
        drivingDedendum = lines.addByTwoPoints(a2, dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            dSeed).parameter.value = dedendum_cm
        pointD = drivingDedendum.endSketchPoint

        # Root Axes: Apex -> C (pinion), Apex -> D (driving).
        pinionRootAxis = lines.addByTwoPoints(apexGeom, pointC.geometry)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        drivingRootAxis = lines.addByTwoPoints(apexGeom, pointD.geometry)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        module_cm = to_cm(module)

        # Helper: collinear extension from a point along an existing line.
        def collinearExtension(srcLine, fromPoint, refDir):
            # refDir: unit (dx,dy) of the direction to extend (away from apex).
            fp = fromPoint.geometry
            endSeed = adsk.core.Point3D.create(
                fp.x + refDir[0] * module_cm, fp.y + refDir[1] * module_cm, 0)
            newLine = lines.addByTwoPoints(fp, endSeed)
            newLine.isConstruction = True
            constraints.addCoincident(newLine.startSketchPoint, fromPoint)
            constraints.addCollinear(newLine, srcLine)
            return newLine

        # A->E collinear with Apex->A (extending away from apex).
        aDirUnit = normalize2(aSeed.x - apexGeom.x, aSeed.y - apexGeom.y)
        lineAE = collinearExtension(pinionShaftAxis, pointA, aDirUnit)
        pointE = lineAE.endSketchPoint

        # C->E line, perpendicular to A->E.
        lineCE = lines.addByTwoPoints(pointC.geometry, pointE.geometry)
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # B->F collinear with Apex->B.
        bDirUnit = normalize2(bSeed.x - apexGeom.x, bSeed.y - apexGeom.y)
        lineBF = collinearExtension(drivingShaftAxis, pointB, bDirUnit)
        pointF = lineBF.endSketchPoint

        # D->F line, perpendicular to B->F.
        lineDF = lines.addByTwoPoints(pointD.geometry, pointF.geometry)
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # E->G collinear with A->E.
        lineEG = collinearExtension(lineAE, pointE, aDirUnit)
        pointG = lineEG.endSketchPoint

        # C->H : length module, collinear with Apex2->C (the pinion dedendum).
        cDirUnit = normalize2(cSeed.x - a2.x, cSeed.y - a2.y)
        hSeed = adsk.core.Point3D.create(
            pointC.geometry.x + cDirUnit[0] * module_cm,
            pointC.geometry.y + cDirUnit[1] * module_cm, 0)
        lineCH = lines.addByTwoPoints(pointC.geometry, hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # G->H line; E->G perpendicular to H->G.
        lineGH = lines.addByTwoPoints(pointG.geometry, pointH.geometry)
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # F->I collinear with B->F.
        lineFI = collinearExtension(lineBF, pointF, bDirUnit)
        pointI = lineFI.endSketchPoint

        # D->J : length module, collinear with Apex2->D (the driving dedendum).
        dDirUnit = normalize2(dSeed.x - a2.x, dSeed.y - a2.y)
        jSeed = adsk.core.Point3D.create(
            pointD.geometry.x + dDirUnit[0] * module_cm,
            pointD.geometry.y + dDirUnit[1] * module_cm, 0)
        lineDJ = lines.addByTwoPoints(pointD.geometry, jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # I->J line; F->I perpendicular to J->I.
        lineIJ = lines.addByTwoPoints(pointI.geometry, pointJ.geometry)
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # Offset dimension between B->Apex2 drop and J->I -> driving base height.
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8.0)
        textJI = adsk.core.Point3D.create(
            (pointI.geometry.x + pointJ.geometry.x) / 2.0,
            (pointI.geometry.y + pointJ.geometry.y) / 2.0, 0)
        dims.addOffsetDimension(
            lineB_Apex2, lineIJ, textJI).parameter.value = drivingBaseHeight_cm

        # Offset dimension between A->Apex2 drop and G->H -> pinion base height.
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        textGH = adsk.core.Point3D.create(
            (pointG.geometry.x + pointH.geometry.x) / 2.0,
            (pointG.geometry.y + pointH.geometry.y) / 2.0, 0)
        dims.addOffsetDimension(
            lineA_Apex2, lineGH, textGH).parameter.value = pinionBaseHeight_cm

        # Line A to G.
        lineAG = lines.addByTwoPoints(pointA.geometry, pointG.geometry)
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # Constrain Point I with center point (pins the figure scale via I=center).
        constraints.addCoincident(pointI, projectedCenter)

        # K: from G along Apex->A away from apex; pin with two point-on-line.
        gGeom = pointG.geometry
        kSeed = adsk.core.Point3D.create(
            gGeom.x + aDirUnit[0] * module_cm, gGeom.y + aDirUnit[1] * module_cm, 0)
        lineGK = lines.addByTwoPoints(gGeom, kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)   # K on Apex->A
        constraints.addCoincident(pointK, lineCH)            # K on Apex2->C extended (C->H)
        lineCK = lines.addByTwoPoints(pointC.geometry, pointK.geometry)
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # --- Maximum Face Width from SOLVED geometry, then resolve Face Width.
        def perpDistancePointToLine(pt, la, lb):
            # distance from pt to the infinite line through la,lb (sketch geom)
            vx = lb.x - la.x
            vy = lb.y - la.y
            wx = pt.x - la.x
            wy = pt.y - la.y
            cross = abs(vx * wy - vy * wx)
            return cross / math.hypot(vx, vy)

        aG = pointA.geometry
        bG = pointB.geometry
        cG = pointC.geometry
        dG = pointD.geometry
        hG = pointH.geometry
        jG = pointJ.geometry
        distPinion = perpDistancePointToLine(aG, cG, hG)  # A to C->H line
        distDriving = perpDistancePointToLine(bG, dG, jG)  # B to D->J line
        maxFaceWidth_cm = 0.95 * min(distPinion, distDriving)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face Width {:.3f} mm exceeds the maximum {:.3f} mm'.format(
                        to_mm(self._faceWidth_cm), to_mm(maxFaceWidth_cm)))
            faceWidth_cm = self._faceWidth_cm
        else:
            coneDistance_cm = to_cm(
                math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
            faceWidth_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)

        # --- Toe line M->N (pinion side) -------------------------------------
        # Seed M ~ midpoint of Apex->C ; seed N slid along C->H toward A->Apex2.
        chDir = normalize2(hG.x - cG.x, hG.y - cG.y)
        mSeed = adsk.core.Point3D.create(
            (apexGeom.x + cG.x) / 2.0, (apexGeom.y + cG.y) / 2.0, 0)
        distMtoA = math.hypot(aG.x - mSeed.x, aG.y - mSeed.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + chDir[0] * distMtoA, mSeed.y + chDir[1] * distMtoA, 0)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)   # M on Apex->C
        constraints.addCoincident(pointN, lineA_Apex2)       # N on A->Apex2 drop
        constraints.addParallel(lineMN, lineCH)
        dims.addOffsetDimension(
            lineCH, lineMN,
            adsk.core.Point3D.create(
                (mSeed.x + cG.x) / 2.0, (mSeed.y + cG.y) / 2.0, 0)
        ).parameter.value = faceWidth_cm
        # M->C and N->A connecting lines.
        lineMC = lines.addByTwoPoints(pointM.geometry, pointC.geometry)
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(pointN.geometry, pointA.geometry)
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # L: from I along Apex->B away from apex; pin same as K.
        iGeom = pointI.geometry
        lSeed = adsk.core.Point3D.create(
            iGeom.x + bDirUnit[0] * module_cm, iGeom.y + bDirUnit[1] * module_cm, 0)
        lineIL = lines.addByTwoPoints(iGeom, lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)  # L on Apex->B
        constraints.addCoincident(pointL, lineDJ)            # L on Apex2->D extended (D->J)
        lineDL = lines.addByTwoPoints(pointD.geometry, pointL.geometry)
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # --- Toe line O->P (driving side, mirror of M->N) --------------------
        djDir = normalize2(jG.x - dG.x, jG.y - dG.y)
        oSeed = adsk.core.Point3D.create(
            (apexGeom.x + dG.x) / 2.0, (apexGeom.y + dG.y) / 2.0, 0)
        distOtoB = math.hypot(bG.x - oSeed.x, bG.y - oSeed.y)
        pSeed = adsk.core.Point3D.create(
            oSeed.x + djDir[0] * distOtoB, oSeed.y + djDir[1] * distOtoB, 0)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)  # O on Apex->D
        constraints.addCoincident(pointP, lineB_Apex2)       # P on B->Apex2 drop
        constraints.addParallel(lineOP, lineDJ)
        dims.addOffsetDimension(
            lineDJ, lineOP,
            adsk.core.Point3D.create(
                (oSeed.x + dG.x) / 2.0, (oSeed.y + dG.y) / 2.0, 0)
        ).parameter.value = faceWidth_cm
        lineOD = lines.addByTwoPoints(pointO.geometry, pointD.geometry)
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(pointP.geometry, pointB.geometry)
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        # Line B to I.
        lineBI = lines.addByTwoPoints(pointB.geometry, pointI.geometry)
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch)

        # =====================================================================
        # §3 Gear Tooth Profiles
        # =====================================================================
        # Pinion: gamma_p ; driving: gamma_g already computed above.
        (pinionToothSketch, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineCK, pointK,
            module, PPD, gamma_p)
        (drivingToothSketch, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineDL, pointL,
            module, DPD, gamma_g)

        # =====================================================================
        # Per-gear bodies (pinion first, no mesh; driving second, mesh offset)
        # =====================================================================
        apexSketchPoint = centerToApex.endSketchPoint

        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            gearName='Pinion Gear',
            srcPoints=[pointA, pointG, pointH, pointC, pointM, pointN],
            toothSketch=pinionToothSketch,
            apexSketchPoint=apexSketchPoint,
            toeEdgePts=(pointM, pointN), heelEdgePts=(pointC, pointH),
            apexPoint=apex,
            teeth=pinionTeeth, bore_cm=pinionBore_cm,
            meshRotation=False, meshTeeth=pinionTeeth)

        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            gearName='Driving Gear',
            srcPoints=[pointB, pointI, pointJ, pointD, pointO, pointP],
            toothSketch=drivingToothSketch,
            apexSketchPoint=apexSketchPoint,
            toeEdgePts=(pointO, pointP), heelEdgePts=(pointD, pointJ),
            apexPoint=apex,
            teeth=drivingTeeth, bore_cm=drivingBore_cm,
            meshRotation=True, meshTeeth=drivingTeeth)

    # =========================================================================
    # §3 helper: virtual spur tooth profile on a plane through C->K / D->L
    # =========================================================================
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 centerLine, centerPoint, module, pitchDiameter_cm,
                                 gamma):
        # Virtual (Tredgold) pitch radius and tooth number (closed form).
        virtualPitchRadius_cm = (pitchDiameter_cm / 2.0) / math.cos(gamma)
        # Module is mm; radius is cm -> compare in mm.
        virtualTeeth = int(math.floor(2.0 * to_mm(virtualPitchRadius_cm) / module))

        # Plane that includes centerLine, perpendicular to the Gear Profiles plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerLine, adsk.core.ValueInput.createByString('90 deg'), gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = 'Tooth Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = 'Tooth Profile'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        # Construction axis through centerPoint normal to the tooth plane via
        # setByTwoPlanes: Gear Profiles plane intersected with a helper plane
        # built setByDistanceOnPath(centerLine, 1.0).
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            centerLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = 'Tooth Axis Helper Plane'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = 'Tooth Axis'

        return (toothSketch, toothAxis)

    # =========================================================================
    # find the borrowed spur tooth cross-section loop by curve-TYPE mix
    # =========================================================================
    def _findSpurToothProfile(self, toothSketch):
        # 2 NURBS flanks + 2 arcs (tip/root) + 2 lines (non-embedded), OR
        # 2 NURBS + 2 arcs + 0 lines (embedded).
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nNurbs = 0
                nArcs = 0
                nLines = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nNurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        nArcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        nLines += 1
                if nNurbs == 2 and nArcs == 2 and (nLines == 2 or nLines == 0):
                    return profile
        raise Exception('Could not find spur tooth cross-section profile')

    # =========================================================================
    # per-gear body creation
    # =========================================================================
    def _createGearBody(self, bevelComponent, designComponent, gearProfilesPlane,
                        gearName, srcPoints, toothSketch, apexSketchPoint,
                        toeEdgePts, heelEdgePts, apexPoint,
                        teeth, bore_cm, meshRotation, meshTeeth):
        features = designComponent.features

        # The gear's own component under Bevel Gear (bodies relocated here).
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = gearName

        # --- Profile sketch: recreate the six vertices as FIXED points -------
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearName} Profile'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        verts = [profileSketch.sketchPoints.add(
                    profileSketch.modelToSketchSpace(self._pointWorldGeometry(p)))
                 for p in srcPoints]
        hexLines = [lines.addByTwoPoints(verts[i], verts[(i + 1) % len(verts)])
                    for i in range(len(verts))]
        for edge in hexLines:
            edge.isFixed = True
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True
        self._assertFullyConstrained(profileSketch)

        # Single closed loop -> profiles.item(0); do NOT filter.
        revolveProfile = profileSketch.profiles.item(0)

        # The shaft axis = the FIRST hexagon edge (A->G / B->I).
        shaftEdge = hexLines[0]

        # --- Revolve about the shaft edge ------------------------------------
        revolves = features.revolveFeatures
        revolveInput = revolves.createInput(
            revolveProfile, shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = f'{gearName} Body'

        # --- Loft apex sketch point -> tooth profile -------------------------
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = f'{gearName} Tooth Body'

        # --- Two conical cuts (interleaved keeper selection) -----------------
        apexWorld = self._pointWorldGeometry(apexPoint)
        toothBody = self._applyConicalCut(
            designComponent, gearName, gearBody, toothBody,
            toeEdgePts, heelEdgePts, apexWorld)

        # --- Circular pattern about the shaft edge ---------------------------
        patternBodies = adsk.core.ObjectCollection.create()
        patternBodies.add(toothBody)
        patterns = features.circularPatternFeatures
        patternInput = patterns.createInput(patternBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # --- Combine-join all tooth pieces into the gear body ----------------
        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))
        combines = features.combineFeatures
        combineInput = combines.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ------------------------------------------------------------
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftEdge, bore_cm)

        # --- Driving gear meshing rotation (in Design, before move) ----------
        if meshRotation:
            startW = shaftEdge.startSketchPoint.worldGeometry
            endW = shaftEdge.endSketchPoint.worldGeometry
            axisVector = adsk.core.Vector3D.create(
                endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(
                math.radians(180.0 / meshTeeth), axisVector, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- Relocate the finished body into the gear's component ------------
        gearBody.moveToComponent(gearOccurrence)

    # =========================================================================
    # conical cut: toe cut (split tooth) -> keeper -> heel cut (keeper alone)
    # =========================================================================
    def _applyConicalCut(self, designComponent, gearName, gearBody, toothBody,
                         toeEdgePts, heelEdgePts, apexWorld):
        features = designComponent.features

        def edgeMid(pts):
            a = self._pointWorldGeometry(pts[0])
            b = self._pointWorldGeometry(pts[1])
            return adsk.core.Point3D.create(
                (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)

        def coneFacesByMidpoint(midWorld):
            faces = []
            for face in gearBody.faces:
                if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                    dist = self._surfaceDistance(face.geometry, midWorld)
                    faces.append((face, dist))
            # ascending distance; None last (treated as +inf).
            faces.sort(key=lambda fd: (fd[1] is None, fd[1] if fd[1] is not None else 0.0))
            return faces

        def trySplit(target, midWorld, catchNoIntersect):
            coneFaces = coneFacesByMidpoint(midWorld)
            for (face, dist) in coneFaces:
                splits = features.splitBodyFeatures
                splitInput = splits.createInput(target, face, True)
                try:
                    splitResult = splits.add(splitInput)
                except RuntimeError as e:
                    msg = str(e)
                    if catchNoIntersect and (
                            'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg):
                        continue
                    continue
                if splitResult.bodies.count > 1:
                    futil.log(
                        f'{gearName} cut: face dist={dist} produced '
                        f'{splitResult.bodies.count} pieces',
                        force_console=True)
                    return [splitResult.bodies.item(i)
                            for i in range(splitResult.bodies.count)]
            return None  # nothing split

        def diag(midWorld):
            return [(self._surfaceDistance(f.geometry, midWorld))
                    for f in gearBody.faces
                    if f.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType]

        def dropApexAndKeepLargest(pieces):
            # Remove every Apex-containing piece, keep the single largest of rest.
            survivors = []
            for body in pieces:
                pc = body.pointContainment(apexWorld)
                if pc in (adsk.fusion.PointContainment.PointInsidePointContainment,
                          adsk.fusion.PointContainment.PointOnPointContainment):
                    continue
                survivors.append(body)
            if len(survivors) == 0:
                raise Exception(
                    f'{gearName}: no non-apex piece remained after a cut')
            survivors.sort(
                key=lambda b: b.physicalProperties.volume, reverse=True)
            keeper = survivors[0]
            removeFeatures = features.removeFeatures
            for body in pieces:
                if body == keeper:
                    continue
                removeFeatures.add(body)
            return keeper

        # --- Cut 1: toe cut on the single Tooth Body (must split, >=2) -------
        toeMid = edgeMid(toeEdgePts)
        toePieces = trySplit(toothBody, toeMid, catchNoIntersect=False)
        if toePieces is None or len(toePieces) < 2:
            raise Exception(
                f'{gearName}: toe cut produced <2 pieces '
                f'(cone face dists to toe midpoint: {diag(toeMid)})')

        # --- Select keeper (drop apex tip) ----------------------------------
        keeper = dropApexAndKeepLargest(toePieces)

        # --- Cut 2: heel cut on the keeper ALONE (may not intersect) --------
        heelMid = edgeMid(heelEdgePts)
        heelPieces = trySplit(keeper, heelMid, catchNoIntersect=True)
        if heelPieces is None:
            # heel cone did not reach the keeper — keeper is already the tooth.
            futil.log(
                f'{gearName}: heel cut tool did not intersect, kept keeper intact',
                force_console=True)
            return keeper

        # heel cut split -> select the tooth again (same rule).
        tooth = dropApexAndKeepLargest(heelPieces)
        return tooth

    # =========================================================================
    # bore cut
    # =========================================================================
    def _cutBore(self, designComponent, gearBody, shaftEdge, bore_cm):
        features = designComponent.features

        # Bore plane normal to the shaft at its start (path distance 0).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{gearBody.name} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearBody.name} Bore'
        boreSketch.isVisible = True

        circles = boreSketch.sketchCurves.sketchCircles
        boreCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore_cm / 2.0)
        boreCircle.isConstruction = False
        # Fix the center (free even at origin) + diameter dimension.
        boreCircle.centerSketchPoint.isFixed = True
        boreSketch.sketchDimensions.addDiameterDimension(
            boreCircle, adsk.core.Point3D.create(bore_cm / 2.0, bore_cm / 2.0, 0)
        ).parameter.value = bore_cm
        self._assertFullyConstrained(boreSketch)

        boreProfile = boreSketch.profiles.item(0)
        extrudes = features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        largeExtent = max(1000.0, bore_cm * 100.0)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(largeExtent), False)
        extrudeInput.participantBodies = [gearBody]
        extrudeResult = extrudes.add(extrudeInput)
        extrudeResult.name = f'{gearBody.name} Bore Cut'

    # =========================================================================
    # Cleanup: hide sketches / construction planes / construction axes
    # =========================================================================
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                if sketch.entityToken in seen:
                    continue
                seen.add(sketch.entityToken)
                sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                if plane.entityToken in seen:
                    continue
                seen.add(plane.entityToken)
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                if axis.entityToken in seen:
                    continue
                seen.add(axis.entityToken)
                axis.isLightBulbOn = False

            for occurrence in component.occurrences:
                walk(occurrence.component)

        walk(bevelComponent)
