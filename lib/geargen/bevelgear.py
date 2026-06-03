import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator

import adsk.core, adsk.fusion


# --- Dialog input ids (the only module-level constants; no PARAM_* names) -----
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

PRESSURE_ANGLE_DEG = 20.0
INVOLUTE_STEPS = 15


class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first so Fusion auto-focuses it)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits against')
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

        # 3. Parent Component (pre-selected to root)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module (unitless)
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


# A tiny value-wrapper so the borrowed spur tooth generator's
# parent.getParameter(name).value interface is satisfied.
class _Val:
    def __init__(self, value):
        self.value = value


# A fake spur `parent` so SpurGearInvoluteToothDesignGenerator can run without
# registering Fusion user parameters. Precomputes (in internal cm) exactly the
# keys the spur drawer reads via parent.getParameter(name).value.
class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        alpha = math.radians(PRESSURE_ANGLE_DEG)
        # Standard spur formulas. Module is in mm; circle radii/diameters served
        # to the spur drawer are converted to cm (what spur expects).
        pitch_mm = virtualTeeth * module_mm
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._values = {
            'Module': to_cm(module_mm),
            'ToothNumber': virtualTeeth,
            'PressureAngle': alpha,
            'PitchCircleDiameter': to_cm(pitch_mm),
            'PitchCircleRadius': to_cm(pitch_mm / 2.0),
            'BaseCircleDiameter': to_cm(base_mm),
            'BaseCircleRadius': to_cm(base_mm / 2.0),
            'RootCircleDiameter': to_cm(root_mm),
            'RootCircleRadius': to_cm(root_mm / 2.0),
            'TipCircleDiameter': to_cm(tip_mm),
            'TipCircleRadius': to_cm(tip_mm / 2.0),
            'InvoluteSteps': INVOLUTE_STEPS,
        }
        # the spur tooth generator records embeddedness on its parent
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # ------------------------------------------------------------------ inputs
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections first.
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

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        targetPlane = planes[0]

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # Module: unitless -> raw number meaning millimetres.
        module = evalExpr(INPUT_ID_MODULE, '')
        if module <= 0:
            raise Exception('Module must be a positive number')

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Tooth counts must be >= 3')

        # Shaft Angle: evaluateExpression(..., 'deg') returns RADIANS internally.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                'Shaft Angle must be between 30 and 150 degrees (got {:.3f})'.format(
                    shaftAngle_deg))

        # 'mm' inputs come back already in internal units (cm) -- use as-is.
        self._drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')

        (self._boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)

        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')

        self._faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ----------------------------------------------------------- world helpers
    def _pointWorldGeometry(self, point):
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    # ----------------------------------------------------------------- driver
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Pitch diameters (Module is mm; convert to cm for geometry).
        drivingPitchDia_cm = to_cm(module * drivingTeeth)
        pinionPitchDia_cm = to_cm(module * pinionTeeth)
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # Resolve bore diameters (cm). 0 => auto (pitchDia / 4).
        drivingBore_cm = self._drivingBore_cm if self._drivingBore_cm > 0 else drivingPitchDia_cm / 4.0
        pinionBore_cm = self._pinionBore_cm if self._pinionBore_cm > 0 else pinionPitchDia_cm / 4.0

        Sigma = math.radians(shaftAngle_deg)

        # --- Build the component tree -----------------------------------------
        # Bevel Gear under the user's parent; Design under Bevel Gear.
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # NEVER activate any occurrence. Build everything on designComponent's
        # own collections (non-activated), exactly as the spur generator does.

        # §1 Anchor sketch.
        (anchorSketch, projectedCenter, anchorLine) = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear bodies.
        self._buildGearProfiles(
            designComponent, bevelComponent, targetPlane, centerPoint,
            anchorSketch, projectedCenter, anchorLine,
            module, drivingTeeth, pinionTeeth, Sigma,
            drivingPitchDia_cm, pinionPitchDia_cm, coneDistance_cm,
            drivingBore_cm, pinionBore_cm)

        # Cleanup.
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # ----------------------------------------------------------------- §1
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Sketch DIRECTLY on the user-selected target plane (ZERO setByOffset).
        anchorSketch = designComponent.sketches.add(targetPlane)
        anchorSketch.name = 'Anchor Sketch'
        anchorSketch.isVisible = True

        projected = anchorSketch.project(centerPoint)
        projectedCenter = projected.item(0)

        lines = anchorSketch.sketchCurves.sketchLines
        constraints = anchorSketch.geometricConstraints
        dims = anchorSketch.sketchDimensions

        c = projectedCenter.geometry
        half_cm = to_cm(5.0)  # 10mm reference line, center at midpoint
        anchorLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x - half_cm, c.y, 0),
            adsk.core.Point3D.create(c.x + half_cm, c.y, 0))
        # Center point divides the line in half via intersection + midpoint.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + to_cm(2.0), 0))

        return (anchorSketch, projectedCenter, anchorLine)

    # ----------------------------------------------------------------- §2 + §3
    def _buildGearProfiles(self, designComponent, bevelComponent, targetPlane,
                           centerPoint, anchorSketch, anchorProjectedCenter,
                           anchorLine, module, drivingTeeth, pinionTeeth, Sigma,
                           drivingPitchDia_cm, pinionPitchDia_cm, coneDistance_cm,
                           drivingBore_cm, pinionBore_cm):
        # Gear Profiles plane: through the anchor line, perpendicular to the
        # ORIGINAL targetPlane (setByAngle 90 deg, passing targetPlane).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        lines = sketch.sketchCurves.sketchLines
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        # Project the center point from the anchor sketch.
        projected = sketch.project(anchorProjectedCenter)
        projectedCenter = projected.item(0)

        # --- world->sketch apex ----------------------------------------------
        normal = get_normal(targetPlane)
        centerWorld = self._pointWorldGeometry(centerPoint)
        # apexWorld = projected center world point offset along normal by
        # Driving Gear Pitch Diameter (in cm).
        nx, ny, nz = normal.x, normal.y, normal.z
        nlen = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nlen == 0:
            nlen = 1.0
        ux, uy, uz = nx / nlen, ny / nlen, nz / nlen
        offset = drivingPitchDia_cm
        apexWorld = adsk.core.Point3D.create(
            centerWorld.x + ux * offset,
            centerWorld.y + uy * offset,
            centerWorld.z + uz * offset)
        # Determine y-up sign by mapping (centerWorld + normal) into sketch.
        apexLocal = sketch.modelToSketchSpace(apexWorld)
        centerLocal = projectedCenter.geometry
        normalProbeWorld = adsk.core.Point3D.create(
            centerWorld.x + ux, centerWorld.y + uy, centerWorld.z + uz)
        normalProbeLocal = sketch.modelToSketchSpace(normalProbeWorld)
        y_up_sign = 1.0 if (normalProbeLocal.y - centerLocal.y) >= 0 else -1.0

        # --- Apex: free endpoint of a vertical construction line --------------
        centerToApex = lines.addByTwoPoints(projectedCenter.geometry, apexLocal)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addVertical(centerToApex)
        apex = centerToApex.endSketchPoint
        # apex Y stays free (no fix/dimension); pinned later through chain.

        # --- seed along-shaft cone geometry ----------------------------------
        PPD = pinionPitchDia_cm
        DPD = drivingPitchDia_cm
        # tan gamma_p = sin Σ · PPD / (DPD + PPD·cos Σ)
        gamma_p = math.atan2(math.sin(Sigma) * PPD, DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        R_seed = (PPD / 2.0) / math.sin(gamma_p) if math.sin(gamma_p) != 0 else coneDistance_cm
        lenApexA = R_seed * math.cos(gamma_p)  # |Apex->A|
        lenApexB = R_seed * math.cos(gamma_g)  # |Apex->B|

        ax = apexLocal.x
        ay = apexLocal.y
        up = y_up_sign

        # --- Driving Gear Shaft Axis (Apex -> B), vertical, downward ----------
        bSeed = adsk.core.Point3D.create(ax, ay - up * lenApexB, 0)
        drivingShaft = lines.addByTwoPoints(apexLocal, bSeed)
        drivingShaft.isConstruction = True
        constraints.addCoincident(drivingShaft.startSketchPoint, apex)
        constraints.addVertical(drivingShaft)
        B = drivingShaft.endSketchPoint

        # --- Pinion Gear Shaft Axis (Apex -> A), tilted by Σ, +X half-plane ---
        # Seed A on +X side, below the apex by lenApexA along direction tilted Σ.
        aSeed = adsk.core.Point3D.create(
            ax + lenApexA * math.sin(Sigma),
            ay - up * lenApexA * math.cos(Sigma), 0)
        pinionShaft = lines.addByTwoPoints(apexLocal, aSeed)
        pinionShaft.isConstruction = True
        constraints.addCoincident(pinionShaft.startSketchPoint, apex)
        A = pinionShaft.endSketchPoint
        # Angular dimension Σ between the two shaft axes; text inside the wedge.
        wedgeText = adsk.core.Point3D.create(
            ax + 0.25 * lenApexA * math.sin(Sigma / 2.0),
            ay - up * 0.25 * lenApexA, 0)
        dims.addAngularDimension(pinionShaft, drivingShaft, wedgeText).parameter.value = Sigma

        # --- Perpendicular drop from A (length PPD/2) toward Apex2 ------------
        # Direction perpendicular to Apex->A, toward the wedge (anchor side).
        # Apex->A direction (seed):
        aDir = adsk.core.Vector3D.create(
            aSeed.x - ax, aSeed.y - ay, 0)
        aDir.normalize()
        # perpendicular: toward -X relative to A (between the two shafts).
        perpA = adsk.core.Vector3D.create(-aDir.y, aDir.x, 0)
        # ensure perpA points toward the driving shaft (negative X direction).
        if perpA.x > 0:
            perpA = adsk.core.Vector3D.create(aDir.y, -aDir.x, 0)
        apex2Seed = adsk.core.Point3D.create(
            aSeed.x + perpA.x * (PPD / 2.0),
            aSeed.y + perpA.y * (PPD / 2.0), 0)
        dropA = lines.addByTwoPoints(aSeed, apex2Seed)
        dropA.isConstruction = True
        constraints.addCoincident(dropA.startSketchPoint, A)
        constraints.addPerpendicular(dropA, pinionShaft)
        dims.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(apex2Seed.x, apex2Seed.y, 0)).parameter.value = PPD / 2.0

        # --- Perpendicular drop from B (length DPD/2) toward Apex2 -----------
        bDir = adsk.core.Vector3D.create(bSeed.x - ax, bSeed.y - ay, 0)
        bDir.normalize()
        perpB = adsk.core.Vector3D.create(-bDir.y, bDir.x, 0)
        if perpB.x < 0:
            perpB = adsk.core.Vector3D.create(bDir.y, -bDir.x, 0)
        apex2SeedB = adsk.core.Point3D.create(
            bSeed.x + perpB.x * (DPD / 2.0),
            bSeed.y + perpB.y * (DPD / 2.0), 0)
        dropB = lines.addByTwoPoints(bSeed, apex2SeedB)
        dropB.isConstruction = True
        constraints.addCoincident(dropB.startSketchPoint, B)
        constraints.addPerpendicular(dropB, drivingShaft)
        dims.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(apex2SeedB.x, apex2SeedB.y, 0)).parameter.value = DPD / 2.0

        # Coincide the two drop endpoints -> Apex 2.
        constraints.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2 = dropA.endSketchPoint

        # --- Pitch Line (Apex -> Apex2) ---------------------------------------
        pitchLine = lines.addByTwoPoints(apexLocal, apex2Seed)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # --- Dedendum lines from Apex2 (length module*1.25), perpendicular ----
        ded_cm = to_cm(1.25 * module)
        # Pitch line direction (seed).
        plDir = adsk.core.Vector3D.create(
            apex2Seed.x - ax, apex2Seed.y - ay, 0)
        plDir.normalize()
        perpPL = adsk.core.Vector3D.create(-plDir.y, plDir.x, 0)
        # Driving Gear Dedendum -> point D (toward anchor line / -X side).
        dDir = perpPL if perpPL.x <= 0 else adsk.core.Vector3D.create(-perpPL.x, -perpPL.y, 0)
        DSeed = adsk.core.Point3D.create(
            apex2Seed.x + dDir.x * ded_cm, apex2Seed.y + dDir.y * ded_cm, 0)
        drivingDedendum = lines.addByTwoPoints(apex2Seed, DSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(DSeed.x, DSeed.y, 0)).parameter.value = ded_cm
        D = drivingDedendum.endSketchPoint

        # Pinion Gear Dedendum -> point C (away from anchor line / +X side).
        cDir = adsk.core.Vector3D.create(-dDir.x, -dDir.y, 0)
        CSeed = adsk.core.Point3D.create(
            apex2Seed.x + cDir.x * ded_cm, apex2Seed.y + cDir.y * ded_cm, 0)
        pinionDedendum = lines.addByTwoPoints(apex2Seed, CSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(CSeed.x, CSeed.y, 0)).parameter.value = ded_cm
        C = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex->D (driving), Apex->C (pinion) -------------------
        drivingRootAxis = lines.addByTwoPoints(apexLocal, DSeed)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, D)

        pinionRootAxis = lines.addByTwoPoints(apexLocal, CSeed)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, C)

        mod_cm = to_cm(module)

        # --- A -> E : colinear with Apex->A, length module -------------------
        # Direction continuing away from apex past A.
        aeDir = adsk.core.Vector3D.create(
            aSeed.x - ax, aSeed.y - ay, 0)
        aeDir.normalize()
        ESeed = adsk.core.Point3D.create(
            aSeed.x + aeDir.x * mod_cm, aSeed.y + aeDir.y * mod_cm, 0)
        lineAE = lines.addByTwoPoints(aSeed, ESeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, A)
        constraints.addCollinear(lineAE, pinionShaft)
        E = lineAE.endSketchPoint

        # --- C -> E : perpendicular to A->E ----------------------------------
        lineCE = lines.addByTwoPoints(CSeed, ESeed)
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, C)
        constraints.addCoincident(lineCE.endSketchPoint, E)
        constraints.addPerpendicular(lineAE, lineCE)

        # --- B -> F : colinear with Apex->B, length module -------------------
        bfDir = adsk.core.Vector3D.create(
            bSeed.x - ax, bSeed.y - ay, 0)
        bfDir.normalize()
        FSeed = adsk.core.Point3D.create(
            bSeed.x + bfDir.x * mod_cm, bSeed.y + bfDir.y * mod_cm, 0)
        lineBF = lines.addByTwoPoints(bSeed, FSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, B)
        constraints.addCollinear(lineBF, drivingShaft)
        F = lineBF.endSketchPoint

        # --- D -> F : perpendicular to B->F ----------------------------------
        lineDF = lines.addByTwoPoints(DSeed, FSeed)
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, D)
        constraints.addCoincident(lineDF.endSketchPoint, F)
        constraints.addPerpendicular(lineBF, lineDF)

        # --- E -> G : colinear with A->E, length module ----------------------
        GSeed = adsk.core.Point3D.create(
            ESeed.x + aeDir.x * mod_cm, ESeed.y + aeDir.y * mod_cm, 0)
        lineEG = lines.addByTwoPoints(ESeed, GSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, E)
        constraints.addCollinear(lineEG, lineAE)
        G = lineEG.endSketchPoint

        # --- C -> H : length module, colinear with Apex2->C ------------------
        chDir = cDir  # Apex2->C direction
        HSeed = adsk.core.Point3D.create(
            CSeed.x + chDir.x * mod_cm, CSeed.y + chDir.y * mod_cm, 0)
        lineCH = lines.addByTwoPoints(CSeed, HSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, C)
        constraints.addCollinear(lineCH, pinionDedendum)
        H = lineCH.endSketchPoint

        # --- G -> H : connect; E->G perpendicular H->G -----------------------
        lineGH = lines.addByTwoPoints(GSeed, HSeed)
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, G)
        constraints.addCoincident(lineGH.endSketchPoint, H)
        constraints.addPerpendicular(lineEG, lineGH)

        # --- F -> I : colinear with B->F, length module ----------------------
        ISeed = adsk.core.Point3D.create(
            FSeed.x + bfDir.x * mod_cm, FSeed.y + bfDir.y * mod_cm, 0)
        lineFI = lines.addByTwoPoints(FSeed, ISeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, F)
        constraints.addCollinear(lineFI, lineBF)
        I = lineFI.endSketchPoint

        # --- D -> J : length module, colinear with Apex2->D ------------------
        djDir = dDir  # Apex2->D direction
        JSeed = adsk.core.Point3D.create(
            DSeed.x + djDir.x * mod_cm, DSeed.y + djDir.y * mod_cm, 0)
        lineDJ = lines.addByTwoPoints(DSeed, JSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, D)
        constraints.addCollinear(lineDJ, drivingDedendum)
        J = lineDJ.endSketchPoint

        # --- I -> J : connect; F->I perpendicular J->I -----------------------
        lineIJ = lines.addByTwoPoints(ISeed, JSeed)
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, I)
        constraints.addCoincident(lineIJ.endSketchPoint, J)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- Base-height offset dims use the PERPENDICULAR DROP lines --------
        # Driving base height: offset between dropB (DPD/2 drop) and J->I.
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8.0)
        dims.addOffsetDimension(
            dropB, lineIJ,
            adsk.core.Point3D.create(ISeed.x, ISeed.y, 0)).parameter.value = drivingBaseHeight_cm

        # Pinion base height: offset between dropA (PPD/2 drop) and G->H.
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / float(drivingTeeth))
        dims.addOffsetDimension(
            dropA, lineGH,
            adsk.core.Point3D.create(GSeed.x, GSeed.y, 0)).parameter.value = pinionBaseHeight_cm

        # --- A -> G line ------------------------------------------------------
        lineAG = lines.addByTwoPoints(aSeed, GSeed)
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, A)
        constraints.addCoincident(lineAG.endSketchPoint, G)

        # --- Constrain Point I with center point ------------------------------
        constraints.addCoincident(I, projectedCenter)

        # --- K : on Apex->A and on Pinion Dedendum (Apex2->C extended) -------
        kSeed = adsk.core.Point3D.create(
            GSeed.x + aeDir.x * mod_cm, GSeed.y + aeDir.y * mod_cm, 0)
        lineGK = lines.addByTwoPoints(GSeed, kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, G)
        K = lineGK.endSketchPoint
        constraints.addCoincident(K, pinionShaft)
        constraints.addCoincident(K, pinionDedendum)
        lineCK = lines.addByTwoPoints(CSeed, kSeed)
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, C)
        constraints.addCoincident(lineCK.endSketchPoint, K)

        # --- Maximum Face Width (now A,B,C,D,H,J exist) ----------------------
        def perpDistance(px, py, lax, lay, lbx, lby):
            # distance from point P to line through La,Lb.
            vx, vy = lbx - lax, lby - lay
            vlen = math.hypot(vx, vy)
            if vlen == 0:
                return 0.0
            return abs((px - lax) * (-vy) + (py - lay) * vx) / vlen

        # Pinion dedendum line through C and H; distance from A.
        distPinion = perpDistance(aSeed.x, aSeed.y, CSeed.x, CSeed.y, HSeed.x, HSeed.y)
        # Driving dedendum line through D and J; distance from B.
        distDriving = perpDistance(bSeed.x, bSeed.y, DSeed.x, DSeed.y, JSeed.x, JSeed.y)
        maxFaceWidth_cm = 0.95 * min(distPinion, distDriving)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face Width {:.4f} mm exceeds the maximum {:.4f} mm for this gear'.format(
                        to_mm(self._faceWidth_cm), to_mm(maxFaceWidth_cm)))
            faceWidth_cm = self._faceWidth_cm
        else:
            faceWidth_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)

        # --- M -> N : toe line on pinion side --------------------------------
        # Seed M near midpoint of Apex->C; seed N slid along C->H toward A->Apex2.
        mSeed = adsk.core.Point3D.create((ax + CSeed.x) / 2.0, (ay + CSeed.y) / 2.0, 0)
        chDir2 = adsk.core.Vector3D.create(HSeed.x - CSeed.x, HSeed.y - CSeed.y, 0)
        chDir2.normalize()
        distMtoA = math.hypot(aSeed.x - mSeed.x, aSeed.y - mSeed.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + chDir2.x * distMtoA, mSeed.y + chDir2.y * distMtoA, 0)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        M = lineMN.startSketchPoint
        N = lineMN.endSketchPoint
        constraints.addCoincident(M, pinionRootAxis)
        constraints.addCoincident(N, dropA)  # N on line A->Apex2
        constraints.addParallel(lineMN, lineCH)
        dims.addOffsetDimension(
            lineCH, lineMN,
            adsk.core.Point3D.create(mSeed.x, mSeed.y, 0)).parameter.value = faceWidth_cm
        lineMC = lines.addByTwoPoints(mSeed, CSeed)
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, M)
        constraints.addCoincident(lineMC.endSketchPoint, C)
        lineNA = lines.addByTwoPoints(nSeed, aSeed)
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, N)
        constraints.addCoincident(lineNA.endSketchPoint, A)

        # --- L : on Apex->B and on Driving Dedendum (Apex2->D extended) ------
        lSeed = adsk.core.Point3D.create(
            ISeed.x + bfDir.x * mod_cm, ISeed.y + bfDir.y * mod_cm, 0)
        lineIL = lines.addByTwoPoints(ISeed, lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, I)
        L = lineIL.endSketchPoint
        constraints.addCoincident(L, drivingShaft)
        constraints.addCoincident(L, drivingDedendum)
        lineDL = lines.addByTwoPoints(DSeed, lSeed)
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, D)
        constraints.addCoincident(lineDL.endSketchPoint, L)

        # --- O -> P : mirror of M->N on driving side -------------------------
        oSeed = adsk.core.Point3D.create((ax + DSeed.x) / 2.0, (ay + DSeed.y) / 2.0, 0)
        djDir2 = adsk.core.Vector3D.create(JSeed.x - DSeed.x, JSeed.y - DSeed.y, 0)
        djDir2.normalize()
        distOtoB = math.hypot(bSeed.x - oSeed.x, bSeed.y - oSeed.y)
        pSeed = adsk.core.Point3D.create(
            oSeed.x + djDir2.x * distOtoB, oSeed.y + djDir2.y * distOtoB, 0)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        O = lineOP.startSketchPoint
        P = lineOP.endSketchPoint
        constraints.addCoincident(O, drivingRootAxis)
        constraints.addCoincident(P, dropB)  # P on line B->Apex2
        constraints.addParallel(lineOP, lineDJ)
        dims.addOffsetDimension(
            lineDJ, lineOP,
            adsk.core.Point3D.create(oSeed.x, oSeed.y, 0)).parameter.value = faceWidth_cm
        lineOD = lines.addByTwoPoints(oSeed, DSeed)
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, O)
        constraints.addCoincident(lineOD.endSketchPoint, D)
        linePB = lines.addByTwoPoints(pSeed, bSeed)
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, P)
        constraints.addCoincident(linePB.endSketchPoint, B)
        lineBI = lines.addByTwoPoints(bSeed, ISeed)
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, B)
        constraints.addCoincident(lineBI.endSketchPoint, I)

        # ===================== §3: Gear Tooth Profiles =======================
        # Pinion virtual tooth profile.
        pinionVirtPitchR = (pinionPitchDia_cm / 2.0) / math.cos(gamma_p)
        pinionVirtTeeth = int(math.floor(2.0 * pinionVirtPitchR / mod_cm))
        (pinionToothSketch, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineCK, K,
            module, pinionVirtTeeth)

        # Driving virtual tooth profile.
        drivingVirtPitchR = (drivingPitchDia_cm / 2.0) / math.cos(gamma_g)
        drivingVirtTeeth = int(math.floor(2.0 * drivingVirtPitchR / mod_cm))
        (drivingToothSketch, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineDL, L,
            module, drivingVirtTeeth)

        # World apex for loft-section / piece-removal.
        apexW = apex.worldGeometry

        # ===================== Create the Pinion Gear ========================
        self._createGearBody(
            designComponent, bevelComponent, 'Pinion',
            gearProfilesPlane,
            [A, G, H, C, M, N], pinionToothSketch, apex, apexW,
            C, H, M, N, pinionTeeth, pinionBore_cm, pinionPitchDia_cm,
            meshRotate=False, drivingTeeth=drivingTeeth)

        # ===================== Create the Driving Gear =======================
        self._createGearBody(
            designComponent, bevelComponent, 'Driving',
            gearProfilesPlane,
            [B, I, J, D, O, P], drivingToothSketch, apex, apexW,
            D, J, O, P, drivingTeeth, drivingBore_cm, drivingPitchDia_cm,
            meshRotate=True, drivingTeeth=drivingTeeth)

    # ----------------------------------------------------------------- §3 tool
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 centerLine, centerPoint, module, virtualTeeth):
        # Plane that includes centerLine (C->K or D->L), perpendicular to the
        # Gear Profiles sketch plane (setByAngle 90 deg, no Path.create).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerLine, adsk.core.ValueInput.createByString('90 deg'), gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = 'Tooth Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = 'Tooth Profile'
        toothSketch.isVisible = True

        # Project the center point (K or L) into the tooth sketch as anchor.
        projected = toothSketch.project(centerPoint)
        anchor = projected.item(0)

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(anchor, angle=math.radians(180))   # 180 deg via draw() angle

        # Construction axis through the center point, normal to the tooth plane,
        # via setByTwoPlanes: Gear Profiles plane intersected with a helper plane
        # setByDistanceOnPath(centerLine, 1.0).
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

    # ------------------------------------------------- face / cut diagnostics
    def _surfaceDistance(self, face, worldPoint):
        evaluator = face.geometry.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return float('inf')
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return float('inf')
        return projected.distanceTo(worldPoint)

    def _findConeFaceForCutLine(self, body, worldA, worldB, gearName, cutLabel,
                                tol=1e-2):
        best = None
        bestDist = None
        diagnostics = []
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            dA = self._surfaceDistance(face, worldA)
            dB = self._surfaceDistance(face, worldB)
            diagnostics.append((dA, dB))
            if dA <= tol and dB <= tol:
                total = dA + dB
                if bestDist is None or total < bestDist:
                    bestDist = total
                    best = face
        if best is None:
            raise Exception(
                '{} {}: no ConeSurfaceType face contains both cut-line endpoints '
                'within tol {} cm. Cone faces (dA,dB): {}'.format(
                    gearName, cutLabel, tol,
                    ['({:.5f},{:.5f})'.format(d[0], d[1]) for d in diagnostics]))
        return (best, bestDist, len(diagnostics))

    def _applyConicalCut(self, designComponent, pieces, body, worldA, worldB,
                         gearName, cutLabel, history):
        (face, selDist, nFaces) = self._findConeFaceForCutLine(
            body, worldA, worldB, gearName, cutLabel)
        futil.log(
            '{} {}: bodies in={}, selected cone face dist={:.5f} '
            '(of {} cone faces)'.format(
                gearName, cutLabel, len(pieces), selDist, nFaces),
            force_console=True)

        resultPieces = []
        produced = 0
        for piece in pieces:
            try:
                splitInput = designComponent.features.splitBodyFeatures.createInput(
                    piece, face, True)
                splitResult = designComponent.features.splitBodyFeatures.add(splitInput)
                for i in range(splitResult.bodies.count):
                    resultPieces.append(splitResult.bodies.item(i))
                    produced += 1
            except RuntimeError as e:
                msg = str(e)
                if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                    futil.log(
                        '{} {}: tool did not intersect a piece, kept intact'.format(
                            gearName, cutLabel),
                        force_console=True)
                    resultPieces.append(piece)
                    produced += 1
                else:
                    raise
        futil.log(
            '{} {}: produced {} pieces'.format(gearName, cutLabel, produced),
            force_console=True)
        history.append(
            'cut {}: faces found={}, selected dist={:.5f}, produced={}'.format(
                cutLabel, nFaces, selDist, produced))
        return resultPieces

    def _findSpurToothProfile(self, sketch):
        # Match by curve-TYPE mix: 2 NURBS flanks + 2 arcs + 2 lines (non-embedded)
        # or 2 NURBS + 2 arcs + 0 lines (embedded).
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                nurbs = arcs = lineCount = others = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        lineCount += 1
                    else:
                        others += 1
                if others != 0:
                    continue
                if nurbs == 2 and arcs == 2 and (lineCount == 2 or lineCount == 0):
                    return profile
        raise Exception('Could not find spur tooth profile (NURBS/arc/line mix)')

    def _cutBore(self, designComponent, gearBody, profileEdge, boreDiameter_cm):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            profileEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = 'Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = 'Bore Profile'
        boreSketch.isVisible = True
        # Plane rooted at shaft start, origin on axis -> circle at origin.
        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2.0)
        circle.isConstruction = False

        boreProfile = None
        for profile in boreSketch.profiles:
            boreProfile = profile
            break
        if boreProfile is None:
            raise Exception('Could not find bore profile')

        extrudes = designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(max(1000.0, boreDiameter_cm * 100.0)),
            False)
        extrudeInput.participantBodies = [gearBody]
        extrudes.add(extrudeInput)

    # ------------------------------------------------------------- gear body
    def _createGearBody(self, designComponent, bevelComponent, gearName,
                        gearProfilesPlane, hexPoints, toothSketch, apex, apexWorld,
                        coneNearPoint1, coneNearPoint2, coneNearPoint3, coneNearPoint4,
                        teeth, boreDiameter_cm, pitchDia_cm,
                        meshRotate, drivingTeeth):
        # Per-gear target component, child of Bevel Gear.
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = '{} Gear'.format(gearName)
        gearComponent = gearOccurrence.component

        # Fresh profile sketch on the Gear Profiles plane.
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = '{} Profile'.format(gearName)
        profileSketch.isVisible = True

        # Project the six hexagon points.
        projectedPts = []
        for p in hexPoints:
            proj = profileSketch.project(p)
            projectedPts.append(proj.item(0))

        lines = profileSketch.sketchCurves.sketchLines
        # Draw the closed hexagon as six SketchLines; FIRST edge is the shaft axis.
        hexLines = []
        n = len(projectedPts)
        for i in range(n):
            a = projectedPts[i]
            b = projectedPts[(i + 1) % n]
            seg = lines.addByTwoPoints(a, b)
            hexLines.append(seg)
        shaftEdge = hexLines[0]  # A->G (pinion) / B->I (driving)

        # Find the hexagon profile loop.
        hexProfile = None
        for profile in profileSketch.profiles:
            hexProfile = profile
            break
        if hexProfile is None:
            raise Exception('{}: could not find profile loop'.format(gearName))

        # --- Revolve around the shaft edge ------------------------------------
        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            hexProfile, shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = '{} Gear Body'.format(gearName)

        # --- Loft Apex -> tooth profile --------------------------------------
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apex)
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = '{} Gear Tooth Body'.format(gearName)

        # --- Two sequential conical cuts -------------------------------------
        w1 = coneNearPoint3.worldGeometry  # M / O
        w2 = coneNearPoint4.worldGeometry  # N / P
        w3 = coneNearPoint1.worldGeometry  # C / D
        w4 = coneNearPoint2.worldGeometry  # H / J

        history = []
        pieces = [toothBody]
        pieces = self._applyConicalCut(
            designComponent, pieces, gearBody, w1, w2, gearName, '#1', history)
        pieces = self._applyConicalCut(
            designComponent, pieces, gearBody, w3, w4, gearName, '#2', history)

        if len(pieces) != 3:
            raise Exception(
                '{}: expected 3 pieces, got {} ({})'.format(
                    gearName, len(pieces), '; '.join(history)))

        # Remove the piece containing the Apex.
        remaining = []
        for piece in pieces:
            containment = piece.pointContainment(apexWorld)
            if containment in (adsk.fusion.PointContainment.PointInsidePointContainment,
                               adsk.fusion.PointContainment.PointOnPointContainment):
                designComponent.features.removeFeatures.add(piece)
            else:
                remaining.append(piece)

        if len(remaining) != 2:
            raise Exception(
                '{}: after apex removal expected 2 pieces, got {}'.format(
                    gearName, len(remaining)))

        # Remove the smaller of the remaining two.
        volA = remaining[0].physicalProperties.volume
        volB = remaining[1].physicalProperties.volume
        if volA >= volB:
            toothPiece = remaining[0]
            designComponent.features.removeFeatures.add(remaining[1])
        else:
            toothPiece = remaining[1]
            designComponent.features.removeFeatures.add(remaining[0])

        # --- Circular pattern around the shaft edge --------------------------
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(toothPiece)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))

        # --- Combine-Join with the gear body ---------------------------------
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore --------------------------------------------------------------
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftEdge, boreDiameter_cm)

        # --- Meshing rotation (driving only), in Design before moveToComponent
        if meshRotate:
            startW = shaftEdge.startSketchPoint.worldGeometry
            endW = shaftEdge.endSketchPoint.worldGeometry
            axisVector = startW.vectorTo(endW)
            axisVector.normalize()
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(
                math.radians(180.0 / drivingTeeth), axisVector, startW)
            bodyColl = adsk.core.ObjectCollection.create()
            bodyColl.add(gearBody)
            moveInput = designComponent.features.moveFeatures.createInput2(bodyColl)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # --- Move finished body into the per-gear component ------------------
        gearBody.moveToComponent(gearOccurrence)

        return gearBody

    # ------------------------------------------------------------- cleanup
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            for sketch in component.sketches:
                token = sketch.entityToken
                if token not in seen:
                    seen.add(token)
                    sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                token = plane.entityToken
                if token not in seen:
                    seen.add(token)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                token = axis.entityToken
                if token not in seen:
                    seen.add(token)
                    axis.isLightBulbOn = False
            for occurrence in component.occurrences:
                walk(occurrence.component)

        walk(bevelComponent)
