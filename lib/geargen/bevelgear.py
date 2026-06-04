import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator

import adsk.core, adsk.fusion


# --- Dialog input ids ---------------------------------------------------------
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

# Pressure angle is not a bevel dialog input; bevel hardcodes 20 deg.
_PRESSURE_ANGLE_DEG = 20.0
_INVOLUTE_STEPS = 15


# =============================================================================
# Command inputs configurator
# =============================================================================
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first so it wins Fusion's auto-focus)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        # 2. Center Point
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
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

        # 4. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6. Driving Gear Teeth Number
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth Number
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
# Virtual-spur proxy: a fake `parent` for the borrowed spur tooth generator
# =============================================================================
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """Stands in for a spur `parent` so SpurGearInvoluteToothDesignGenerator can
    read parameters via parent.getParameter(name).value without any real Fusion
    user parameters. Module arrives in mm; circle radii/diameters are served in
    internal cm (which is what the spur tooth generator expects)."""

    def __init__(self, module_mm, virtualTeeth):
        alpha = math.radians(_PRESSURE_ANGLE_DEG)
        pitch_mm = virtualTeeth * module_mm
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._params = {
            'Module': module_mm,
            'ToothNumber': float(virtualTeeth),
            'PressureAngle': alpha,
            'PitchCircleDiameter': to_cm(pitch_mm),
            'PitchCircleRadius': to_cm(pitch_mm / 2.0),
            'BaseCircleDiameter': to_cm(base_mm),
            'BaseCircleRadius': to_cm(base_mm / 2.0),
            'RootCircleDiameter': to_cm(root_mm),
            'RootCircleRadius': to_cm(root_mm / 2.0),
            'TipCircleDiameter': to_cm(tip_mm),
            'TipCircleRadius': to_cm(tip_mm / 2.0),
            'InvoluteSteps': float(_INVOLUTE_STEPS),
        }
        # The spur tooth generator stores this attribute on its parent.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._params[name])


# =============================================================================
# Generator
# =============================================================================
class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

        # stashed (non-primary) inputs, filled by _readInputs
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0

        # stashed §1 anchor-center SketchPoint (re-projected in §2)
        self._anchorCenterPoint = None

    # -------------------------------------------------------------------------
    # input reading & validation
    # -------------------------------------------------------------------------
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections first (immune to context shift, but read uniformly).
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

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # Module: unit '' -> raw number meaning millimetres.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft Angle: 'deg' returns radians internally -> back to degrees.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30.0 or shaftAngle_deg > 150.0:
            raise Exception(
                'Shaft Angle must be between 30 and 150 degrees (got {:.3f})'.format(
                    shaftAngle_deg))

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Teeth numbers must be at least 3')

        # 'mm' inputs come back already in internal cm; do NOT to_cm again.
        self._drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        if (self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0
                or self._drivingBore_cm < 0 or self._pinionBore_cm < 0
                or self._faceWidth_cm < 0):
            raise Exception('Heights / bore diameters / face width must be non-negative')

        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # -------------------------------------------------------------------------
    # small geometry helpers
    # -------------------------------------------------------------------------
    def _pointWorldGeometry(self, point):
        """World Point3D for a SketchPoint (.worldGeometry) or a
        ConstructionPoint (.geometry)."""
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _surfaceDistance(self, surface, worldPoint):
        """Distance from a point to an (infinite) surface via the evaluator's
        project-to-surface round-trip. Returns None when unevaluable (e.g. near
        a cone apex singularity)."""
        try:
            evaluator = surface.evaluator
            (ok, param) = evaluator.getParameterAtPoint(worldPoint)
            if not ok:
                return None
            (ok2, projected) = evaluator.getPointAtParameter(param)
            if not ok2:
                return None
            return projected.distanceTo(worldPoint)
        except:
            return None

    # -------------------------------------------------------------------------
    # generate
    # -------------------------------------------------------------------------
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # pitch diameters (Module is mm -> to_cm) in internal cm
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        # bore diameters (auto = pitchDiameter / 4) in cm
        drivingBore_cm = (self._drivingBore_cm if self._drivingBore_cm > 0
                          else drivingPitchDiameter_cm / 4.0)
        pinionBore_cm = (self._pinionBore_cm if self._pinionBore_cm > 0
                         else pinionPitchDiameter_cm / 4.0)

        # cone distance (cm)
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # --- component tree ---------------------------------------------------
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # --- §1 anchor sketch -------------------------------------------------
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # --- §2 + §3 + per-gear bodies ---------------------------------------
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorLine,
            module, shaftAngle_deg, drivingTeeth, pinionTeeth,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm, coneDistance_cm,
            drivingBore_cm, pinionBore_cm)

        # --- cleanup ----------------------------------------------------------
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # -------------------------------------------------------------------------
    # §1: Anchor Sketch
    # -------------------------------------------------------------------------
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Sketch directly on the user-selected target plane (no normalize).
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        # Project the user center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        lines = sketch.sketchCurves.sketchLines
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        c = projectedCenter.geometry
        halfLen = to_cm(5.0)  # ~10mm reference line
        p1 = adsk.core.Point3D.create(c.x - halfLen, c.y, 0)
        p2 = adsk.core.Point3D.create(c.x + halfLen, c.y, 0)
        anchorLine = lines.addByTwoPoints(p1, p2)

        # BOTH coincident (pin center onto the line) AND midpoint (center
        # bisects the line).
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + halfLen, 0)).parameter.value = to_cm(10.0)

        return anchorLine

    # -------------------------------------------------------------------------
    # §2 + §3 + bodies
    # -------------------------------------------------------------------------
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane,
                           anchorLine, module, shaftAngle_deg,
                           drivingTeeth, pinionTeeth,
                           drivingPitchDiameter_cm, pinionPitchDiameter_cm,
                           coneDistance_cm, drivingBore_cm, pinionBore_cm):
        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm
        sigma = math.radians(shaftAngle_deg)

        # cone half-angles (closed form) reused for seeds & §3 virtual radii
        gamma_p = math.atan2(math.sin(sigma) * PPD, DPD + PPD * math.cos(sigma))
        gamma_g = sigma - gamma_p
        coneR = (PPD / 2.0) / math.sin(gamma_p)  # cone distance (cm); == coneDistance_cm
        lenApexA = coneR * math.cos(gamma_p)     # |Apex->A|
        lenApexB = coneR * math.cos(gamma_g)     # |Apex->B|

        moduleLen = to_cm(module)
        dedendumLen = to_cm(1.25 * module)

        # --- Gear Profiles plane: includes anchor line, 90 deg off targetPlane
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

        # project the stashed §1 anchor-center SketchPoint (not the raw center)
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        # project the anchor line to get its 2-D direction
        projectedAnchor = sketch.project(anchorLine).item(0)

        c = projectedCenter.geometry
        a1 = projectedAnchor.startSketchPoint.geometry
        a2 = projectedAnchor.endSketchPoint.geometry
        dx = a2.x - a1.x
        dy = a2.y - a1.y
        dlen = math.hypot(dx, dy)
        d = (dx / dlen, dy / dlen)
        # in-plane perpendicular (one of two senses)
        perp = (-d[1], d[0])

        # Grow-side sign: choose perp pointing TOWARD the target normal (1-bit
        # direction only; the apex POSITION stays sketch-local).
        nWorld = get_normal(targetPlane)
        P = projectedCenter.worldGeometry
        o2 = sketch.modelToSketchSpace(P)
        Pn = adsk.core.Point3D.create(
            P.x + nWorld.x, P.y + nWorld.y, P.z + nWorld.z)
        n2 = sketch.modelToSketchSpace(Pn)
        nDir2D = (n2.x - o2.x, n2.y - o2.y)
        if (perp[0] * nDir2D[0] + perp[1] * nDir2D[1]) <= 0:
            perp = (-perp[0], -perp[1])

        # --- Apex: free end of a construction line from projected center -----
        apexSeed = adsk.core.Point3D.create(
            c.x + perp[0] * DPD, c.y + perp[1] * DPD, 0)
        centerToApex = lines.addByTwoPoints(c, apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projectedAnchor)
        apex = centerToApex.endSketchPoint

        def seedPoint(px, py):
            return adsk.core.Point3D.create(px, py, 0)

        # --- Driving Gear Shaft Axis (apex -> B), parallel to centerToApex ----
        bSeed = seedPoint(
            apexSeed.x - perp[0] * lenApexB, apexSeed.y - perp[1] * lenApexB)
        drivingShaftAxis = lines.addByTwoPoints(
            seedPoint(apexSeed.x, apexSeed.y), bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis (apex -> A), angular Sigma off driving ----
        # drawn on the side away from the anchor leading direction (+d half).
        # rotate -perp direction by gamma toward +d for a reasonable seed
        # A is at angle sigma from B about the apex. Seed using cone geometry.
        # direction of apex->B is -perp; rotate by sigma toward +d side.
        def rot2(vx, vy, ang):
            ca = math.cos(ang)
            sa = math.sin(ang)
            return (vx * ca - vy * sa, vx * sa + vy * ca)

        # Determine rotation sign so A lands on +d side (toward anchor leading).
        baseDir = (-perp[0], -perp[1])  # apex->B direction
        (rx, ry) = rot2(baseDir[0], baseDir[1], sigma)
        if (rx * d[0] + ry * d[1]) < 0:
            (rx, ry) = rot2(baseDir[0], baseDir[1], -sigma)
        aSeed = seedPoint(apexSeed.x + rx * lenApexA, apexSeed.y + ry * lenApexA)
        pinionShaftAxis = lines.addByTwoPoints(
            seedPoint(apexSeed.x, apexSeed.y), aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        pointA = pinionShaftAxis.endSketchPoint

        # angular dimension Sigma between the two shaft axes; text inside wedge
        wedgeX = apexSeed.x + (rx + baseDir[0]) * 0.25 * lenApexA
        wedgeY = apexSeed.y + (ry + baseDir[1]) * 0.25 * lenApexA
        dims.addAngularDimension(
            pinionShaftAxis, drivingShaftAxis,
            seedPoint(wedgeX, wedgeY)).parameter.value = sigma

        # --- A -> Apex2 perpendicular drop, length PPD/2 ----------------------
        # toward the side where Apex2 lies (between the shafts, +d direction)
        apex2Seed = seedPoint(
            (aSeed.x + bSeed.x) / 2.0 + d[0] * 0.01,
            (aSeed.y + bSeed.y) / 2.0 + d[1] * 0.01)
        dropA = lines.addByTwoPoints(seedPoint(aSeed.x, aSeed.y), apex2Seed)
        dropA.isConstruction = True
        constraints.addCoincident(dropA.startSketchPoint, pointA)
        constraints.addPerpendicular(dropA, pinionShaftAxis)
        dims.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            seedPoint(apex2Seed.x, apex2Seed.y)).parameter.value = PPD / 2.0

        # --- B -> Apex2 perpendicular drop, length DPD/2 ----------------------
        dropB = lines.addByTwoPoints(seedPoint(bSeed.x, bSeed.y),
                                     seedPoint(apex2Seed.x, apex2Seed.y))
        dropB.isConstruction = True
        constraints.addCoincident(dropB.startSketchPoint, pointB)
        constraints.addPerpendicular(dropB, drivingShaftAxis)
        dims.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            seedPoint(apex2Seed.x, apex2Seed.y)).parameter.value = DPD / 2.0

        # Close the two drops at Apex2.
        constraints.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2 = dropA.endSketchPoint

        # --- Pitch Line: Apex -> Apex2 ----------------------------------------
        pitchLine = lines.addByTwoPoints(
            seedPoint(apexSeed.x, apexSeed.y), seedPoint(apex2Seed.x, apex2Seed.y))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # --- Dedendum lines from Apex2, length 1.25*module, perp to Pitch -----
        # direction along pitch line (apex->apex2)
        pdx = apex2Seed.x - apexSeed.x
        pdy = apex2Seed.y - apexSeed.y
        pdl = math.hypot(pdx, pdy)
        pdir = (pdx / pdl, pdy / pdl)
        pperp = (-pdir[1], pdir[0])
        # Driving Gear Dedendum (toward anchor line) -> point D
        # Pinion Gear Dedendum (away from anchor line) -> point C
        # "toward anchor line" ~ -perp side (back toward center/anchor)
        if (pperp[0] * perp[0] + pperp[1] * perp[1]) > 0:
            towardAnchorDir = (-pperp[0], -pperp[1])
            awayDir = (pperp[0], pperp[1])
        else:
            towardAnchorDir = (pperp[0], pperp[1])
            awayDir = (-pperp[0], -pperp[1])

        dSeed = seedPoint(apex2Seed.x + towardAnchorDir[0] * dedendumLen,
                          apex2Seed.y + towardAnchorDir[1] * dedendumLen)
        drivingDedendum = lines.addByTwoPoints(
            seedPoint(apex2Seed.x, apex2Seed.y), dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            seedPoint(dSeed.x, dSeed.y)).parameter.value = dedendumLen
        pointD = drivingDedendum.endSketchPoint

        cSeed = seedPoint(apex2Seed.x + awayDir[0] * dedendumLen,
                          apex2Seed.y + awayDir[1] * dedendumLen)
        pinionDedendum = lines.addByTwoPoints(
            seedPoint(apex2Seed.x, apex2Seed.y), cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            seedPoint(cSeed.x, cSeed.y)).parameter.value = dedendumLen
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex -> D and Apex -> C -------------------------------
        drivingRootAxis = lines.addByTwoPoints(
            seedPoint(apexSeed.x, apexSeed.y), seedPoint(dSeed.x, dSeed.y))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(
            seedPoint(apexSeed.x, apexSeed.y), seedPoint(cSeed.x, cSeed.y))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- E: from A, colinear with Apex->A, length module ------------------
        # direction apex->A
        aDir = (rx, ry)
        eSeed = seedPoint(aSeed.x + aDir[0] * moduleLen, aSeed.y + aDir[1] * moduleLen)
        lineAE = lines.addByTwoPoints(seedPoint(aSeed.x, aSeed.y), eSeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        # --- C -> E, perpendicular to A->E ------------------------------------
        lineCE = lines.addByTwoPoints(seedPoint(cSeed.x, cSeed.y),
                                      seedPoint(eSeed.x, eSeed.y))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # --- F: from B, colinear with Apex->B, length module ------------------
        bDir = (baseDir[0], baseDir[1])  # apex->B direction
        fSeed = seedPoint(bSeed.x + bDir[0] * moduleLen, bSeed.y + bDir[1] * moduleLen)
        lineBF = lines.addByTwoPoints(seedPoint(bSeed.x, bSeed.y), fSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        # --- D -> F, perpendicular to B->F ------------------------------------
        lineDF = lines.addByTwoPoints(seedPoint(dSeed.x, dSeed.y),
                                      seedPoint(fSeed.x, fSeed.y))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # --- G: from E, colinear A->E, length module --------------------------
        gSeed = seedPoint(eSeed.x + aDir[0] * moduleLen, eSeed.y + aDir[1] * moduleLen)
        lineEG = lines.addByTwoPoints(seedPoint(eSeed.x, eSeed.y), gSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # --- H: from C, length module, colinear with Apex2->C -----------------
        # direction apex2->C is awayDir
        hSeed = seedPoint(cSeed.x + awayDir[0] * moduleLen,
                          cSeed.y + awayDir[1] * moduleLen)
        lineCH = lines.addByTwoPoints(seedPoint(cSeed.x, cSeed.y), hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # --- G -> H, perpendicular to E->G ------------------------------------
        lineGH = lines.addByTwoPoints(seedPoint(gSeed.x, gSeed.y),
                                      seedPoint(hSeed.x, hSeed.y))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # --- I: from F, colinear B->F, length module --------------------------
        iSeed = seedPoint(fSeed.x + bDir[0] * moduleLen, fSeed.y + bDir[1] * moduleLen)
        lineFI = lines.addByTwoPoints(seedPoint(fSeed.x, fSeed.y), iSeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # --- J: from D, length module, colinear with Apex2->D -----------------
        jSeed = seedPoint(dSeed.x + towardAnchorDir[0] * moduleLen,
                          dSeed.y + towardAnchorDir[1] * moduleLen)
        lineDJ = lines.addByTwoPoints(seedPoint(dSeed.x, dSeed.y), jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # --- I -> J, perpendicular to F->I ------------------------------------
        lineIJ = lines.addByTwoPoints(seedPoint(iSeed.x, iSeed.y),
                                      seedPoint(jSeed.x, jSeed.y))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- offset dim: B->Apex2 drop (dropB) and J->I (lineIJ) --------------
        drivingBaseHeight = (self._drivingBaseHeight_cm
                             if self._drivingBaseHeight_cm > 0
                             else to_cm(module * drivingTeeth / 8.0))
        dims.addOffsetDimension(
            dropB, lineIJ, seedPoint(jSeed.x, jSeed.y)).parameter.value = drivingBaseHeight

        # --- offset dim: A->Apex2 drop (dropA) and G->H (lineGH) --------------
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight = drivingBaseHeight * (float(pinionTeeth) / float(drivingTeeth))
        dims.addOffsetDimension(
            dropA, lineGH, seedPoint(hSeed.x, hSeed.y)).parameter.value = pinionBaseHeight

        # --- A -> G line ------------------------------------------------------
        lineAG = lines.addByTwoPoints(seedPoint(aSeed.x, aSeed.y),
                                      seedPoint(gSeed.x, gSeed.y))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # --- Constrain point I with center point ------------------------------
        constraints.addCoincident(pointI, projectedCenter)

        # --- K: from G, away from Apex along Apex->A, pinned by two coincidents
        kSeed = seedPoint(gSeed.x + aDir[0] * dedendumLen,
                          gSeed.y + aDir[1] * dedendumLen)
        lineGK = lines.addByTwoPoints(seedPoint(gSeed.x, gSeed.y), kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)
        lineCK = lines.addByTwoPoints(seedPoint(cSeed.x, cSeed.y),
                                      seedPoint(kSeed.x, kSeed.y))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # --- Maximum Face Width from SOLVED geometry --------------------------
        def perpDistance(pt, la, lb):
            # distance from pt to the (infinite) line through la, lb (2-D)
            vx = lb.x - la.x
            vy = lb.y - la.y
            vl = math.hypot(vx, vy)
            if vl == 0:
                return 0.0
            return abs((pt.x - la.x) * vy - (pt.y - la.y) * vx) / vl

        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distPinion = perpDistance(gA, gC, gH)   # A to line C->H
        distDriving = perpDistance(gB, gD, gJ)  # B to line D->J
        maxFaceWidth = 0.95 * min(distPinion, distDriving)

        # --- resolve Face Width ----------------------------------------------
        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth:
                raise Exception(
                    'Face Width {:.4f} cm exceeds the maximum {:.4f} cm '
                    '(bounded by the dedendum geometry)'.format(
                        self._faceWidth_cm, maxFaceWidth))
            faceWidth = self._faceWidth_cm
        else:
            faceWidth = min(coneDistance_cm / 6.0, maxFaceWidth)

        # --- M -> N (pinion toe) ---------------------------------------------
        # seed M near midpoint of Apex->C; seed N slid along C->H toward A->Apex2
        chx = gH.x - gC.x
        chy = gH.y - gC.y
        chl = math.hypot(chx, chy)
        chDir = (chx / chl, chy / chl)
        mSeed = seedPoint((apexSeed.x + gC.x) / 2.0, (apexSeed.y + gC.y) / 2.0)
        slideDist = math.hypot(mSeed.x - gA.x, mSeed.y - gA.y)
        nSeed = seedPoint(mSeed.x + chDir[0] * slideDist,
                          mSeed.y + chDir[1] * slideDist)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, dropA)
        constraints.addParallel(lineMN, lineCH)
        dims.addOffsetDimension(
            lineCH, lineMN, seedPoint(mSeed.x, mSeed.y)).parameter.value = faceWidth

        lineMC = lines.addByTwoPoints(seedPoint(mSeed.x, mSeed.y),
                                      seedPoint(gC.x, gC.y))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(seedPoint(nSeed.x, nSeed.y),
                                      seedPoint(gA.x, gA.y))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- L: from I, away from Apex along Apex->B, pinned by two coincidents
        lSeed = seedPoint(iSeed.x + bDir[0] * dedendumLen,
                          iSeed.y + bDir[1] * dedendumLen)
        lineIL = lines.addByTwoPoints(seedPoint(iSeed.x, iSeed.y), lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)
        lineDL = lines.addByTwoPoints(seedPoint(dSeed.x, dSeed.y),
                                      seedPoint(lSeed.x, lSeed.y))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # --- O -> P (driving toe), mirror of M->N -----------------------------
        djx = gJ.x - gD.x
        djy = gJ.y - gD.y
        djl = math.hypot(djx, djy)
        djDir = (djx / djl, djy / djl)
        oSeed = seedPoint((apexSeed.x + gD.x) / 2.0, (apexSeed.y + gD.y) / 2.0)
        slideDistP = math.hypot(oSeed.x - gB.x, oSeed.y - gB.y)
        pSeed = seedPoint(oSeed.x + djDir[0] * slideDistP,
                          oSeed.y + djDir[1] * slideDistP)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, dropB)
        constraints.addParallel(lineOP, lineDJ)
        dims.addOffsetDimension(
            lineDJ, lineOP, seedPoint(oSeed.x, oSeed.y)).parameter.value = faceWidth

        lineOD = lines.addByTwoPoints(seedPoint(oSeed.x, oSeed.y),
                                      seedPoint(gD.x, gD.y))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(seedPoint(pSeed.x, pSeed.y),
                                      seedPoint(gB.x, gB.y))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(seedPoint(gB.x, gB.y),
                                      seedPoint(iSeed.x, iSeed.y))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        # Hide construction sketch once consumed for measurements? -> keep until
        # §3 / bodies consume it. Done at cleanup.

        # =====================================================================
        # §3: Gear Tooth Profiles + per-gear bodies
        # =====================================================================
        # Pinion tooth (uses point K; γ_p)
        pinionVirtualPitchRadius = (to_cm(module * pinionTeeth) / 2.0) / math.cos(gamma_p)
        pinionVirtualTeeth = int(math.floor(
            2.0 * to_mm(pinionVirtualPitchRadius) / module))
        if pinionVirtualTeeth < 3:
            pinionVirtualTeeth = 3
        (pinionToothSketch, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineCK, pointK,
            module, pinionVirtualTeeth, 'Pinion Tooth')

        # Driving tooth (uses point L; γ_g)
        drivingVirtualPitchRadius = (to_cm(module * drivingTeeth) / 2.0) / math.cos(gamma_g)
        drivingVirtualTeeth = int(math.floor(
            2.0 * to_mm(drivingVirtualPitchRadius) / module))
        if drivingVirtualTeeth < 3:
            drivingVirtualTeeth = 3
        (drivingToothSketch, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, lineDL, pointL,
            module, drivingVirtualTeeth, 'Driving Tooth')

        apexWorld = apex.worldGeometry

        # --- Pinion gear body ------------------------------------------------
        pinionOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        pinionOccurrence.component.name = 'Pinion Gear'
        self._createGearBody(
            designComponent, pinionOccurrence, gearProfilesPlane,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            'Pinion Profile', apex, apexWorld,
            pinionToothSketch, pinionVirtualTeeth,
            pointM, pointN, pointC, pointH,  # cut#1 edge M->N, cut#2 edge C->H
            pinionTeeth, self._boreEnable, pinionBore_cm,
            isDriving=False, drivingTeeth=drivingTeeth)

        # --- Driving gear body -----------------------------------------------
        drivingOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        drivingOccurrence.component.name = 'Driving Gear'
        self._createGearBody(
            designComponent, drivingOccurrence, gearProfilesPlane,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            'Driving Profile', apex, apexWorld,
            drivingToothSketch, drivingVirtualTeeth,
            pointO, pointP, pointD, pointJ,  # cut#1 edge O->P, cut#2 edge D->J
            drivingTeeth, self._boreEnable, drivingBore_cm,
            isDriving=True, drivingTeeth=drivingTeeth)

    # -------------------------------------------------------------------------
    # §3 helper: virtual spur tooth profile + tooth-normal axis
    # -------------------------------------------------------------------------
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 centerLine, centerPoint, module, virtualTeeth,
                                 name):
        # Tooth plane: includes centerLine (C->K or D->L), perpendicular to
        # the Gear Profiles plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerLine, adsk.core.ValueInput.createByString('90 deg'),
            gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = name + ' Plane'

        sketch = designComponent.sketches.add(toothPlane)
        sketch.name = name
        sketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        # Construction axis through the center point, normal to the tooth plane,
        # via setByTwoPlanes( GearProfiles plane, helper-plane-at-K ).
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            centerLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = name + ' Helper Plane'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = name + ' Axis'

        return (sketch, toothAxis)

    # -------------------------------------------------------------------------
    # tooth-profile loop finder (by curve-TYPE mix)
    # -------------------------------------------------------------------------
    def _findSpurToothProfile(self, sketch):
        Curve = adsk.core.Curve3DTypes
        best = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                nNurbs = 0
                nArc = 0
                nLine = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == Curve.NurbsCurve3DCurveType:
                        nNurbs += 1
                    elif ct == Curve.Arc3DCurveType:
                        nArc += 1
                    elif ct == Curve.Line3DCurveType:
                        nLine += 1
                # 2 NURBS flanks + 2 arcs (tip/root) + (2 lines | 0 lines)
                if nNurbs == 2 and nArc == 2 and (nLine == 2 or nLine == 0):
                    return profile
        return best

    # -------------------------------------------------------------------------
    # cone-face finder for a cut line, by the edge MIDPOINT
    # -------------------------------------------------------------------------
    def _findConeFaceForCutLine(self, frustumBody, edgeMidWorld):
        Surface = adsk.core.SurfaceTypes
        coneFaces = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType == Surface.ConeSurfaceType:
                dist = self._surfaceDistance(face.geometry, edgeMidWorld)
                coneFaces.append((face, dist))

        def sortKey(item):
            d = item[1]
            return d if d is not None else float('inf')

        coneFaces.sort(key=sortKey)
        return coneFaces  # list of (face, dist) best-first

    # -------------------------------------------------------------------------
    # apply the two conical cuts sequentially over the current pieces
    # -------------------------------------------------------------------------
    def _applyConicalCut(self, designComponent, toothBody, frustumBody,
                         cut1MidWorld, cut2MidWorld, apexWorld, gearLabel):
        splits = designComponent.features.splitBodyFeatures
        removes = designComponent.features.removeFeatures

        history = {'cut1': {}, 'cut2': {}}

        def tryCut(pieces, midWorld, cutKey):
            coneFaces = self._findConeFaceForCutLine(frustumBody, midWorld)
            history[cutKey]['faceCount'] = len(coneFaces)
            history[cutKey]['dists'] = [d for (_f, d) in coneFaces]
            outPieces = []
            anySplit = False
            for piece in pieces:
                splitThis = False
                for (face, dist) in coneFaces:
                    try:
                        splitInput = splits.createInput(piece, face, True)
                        result = splits.add(splitInput)
                        if result.bodies.count > 1:
                            for i in range(result.bodies.count):
                                outPieces.append(result.bodies.item(i))
                            splitThis = True
                            anySplit = True
                            history[cutKey]['selectedDist'] = dist
                            break
                    except RuntimeError as e:
                        msg = str(e)
                        if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                            continue
                        # other cone face may still work; keep trying
                        continue
                if not splitThis:
                    outPieces.append(piece)
            return (outPieces, anySplit)

        # cut #1 (must split or it's a hard failure)
        (pieces, any1) = tryCut([toothBody], cut1MidWorld, 'cut1')
        history['cut1']['producedCount'] = len(pieces)
        futil.log(
            '{} cut#1: in=1 faces={} selectedDist={} out={}'.format(
                gearLabel, history['cut1'].get('faceCount'),
                history['cut1'].get('selectedDist'), len(pieces)),
            force_console=True)
        if not any1:
            raise Exception(
                '{} cut#1: no cone face split the Tooth Body '
                '(frustum cone faces={}, surfaceDists={})'.format(
                    gearLabel, history['cut1'].get('faceCount'),
                    history['cut1'].get('dists')))

        # cut #2 (may not intersect every piece)
        (pieces, any2) = tryCut(pieces, cut2MidWorld, 'cut2')
        history['cut2']['producedCount'] = len(pieces)
        futil.log(
            '{} cut#2: in={} faces={} selectedDist={} out={}'.format(
                gearLabel, history['cut1'].get('producedCount'),
                history['cut2'].get('faceCount'),
                history['cut2'].get('selectedDist'), len(pieces)),
            force_console=True)

        if len(pieces) < 2:
            raise Exception(
                '{}: expected >=2 pieces, got {} '
                '(cut#1: faces={}, selectedDist={}, produced={}; '
                'cut#2: faces={}, selectedDist={}, produced={})'.format(
                    gearLabel, len(pieces),
                    history['cut1'].get('faceCount'),
                    history['cut1'].get('selectedDist'),
                    history['cut1'].get('producedCount'),
                    history['cut2'].get('faceCount'),
                    history['cut2'].get('selectedDist'),
                    history['cut2'].get('producedCount')))

        # --- select the tooth robustly (3, 4, or more pieces) ----------------
        Containment = adsk.fusion.PointContainment
        nonApex = []
        apexPieces = []
        for piece in pieces:
            c = piece.pointContainment(apexWorld)
            if c == Containment.PointInsidePointContainment or \
               c == Containment.PointOnPointContainment:
                apexPieces.append(piece)
            else:
                nonApex.append(piece)

        if len(nonApex) == 0:
            raise Exception(
                '{}: no non-apex piece found among {} pieces'.format(
                    gearLabel, len(pieces)))

        # keep the largest non-apex piece (the tooth band)
        tooth = None
        toothVol = -1.0
        for piece in nonApex:
            vol = piece.physicalProperties.volume
            if vol > toothVol:
                toothVol = vol
                tooth = piece

        # remove every other piece (apex pieces + smaller non-apex stubs)
        for piece in pieces:
            if piece is tooth:
                continue
            try:
                removes.add(piece)
            except:
                pass

        return tooth

    # -------------------------------------------------------------------------
    # per-gear body creation
    # -------------------------------------------------------------------------
    def _createGearBody(self, designComponent, gearOccurrence, gearProfilesPlane,
                        hexPoints, profileName, apex, apexWorld,
                        toothSketch, virtualTeeth,
                        cut1A, cut1B, cut2A, cut2B,
                        teeth, boreEnable, bore_cm,
                        isDriving, drivingTeeth):
        # --- fresh profile sketch on the axial (Gear Profiles) plane ---------
        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = profileName
        sketch.isVisible = True
        lines = sketch.sketchCurves.sketchLines

        # project the six §2 points
        projected = []
        for pt in hexPoints:
            projected.append(sketch.project(pt).item(0))

        # draw the closed hexagon, sharing each projected SketchPoint
        hexEdges = []
        for i in range(6):
            startP = projected[i]
            endP = projected[(i + 1) % 6]
            edge = lines.addByTwoPoints(startP, endP)
            hexEdges.append(edge)

        # shaft axis = the first edge (A->G for pinion, B->I for driving)
        shaftEdge = hexEdges[0]

        # find the hexagon profile (single loop)
        bodyProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 6:
                    bodyProfile = profile
                    break
            if bodyProfile is not None:
                break
        if bodyProfile is None:
            # fall back to the first profile available
            if sketch.profiles.count > 0:
                bodyProfile = sketch.profiles.item(0)
        if bodyProfile is None:
            raise Exception('Could not find {} hexagon profile'.format(profileName))

        # --- revolve around shaft edge ---------------------------------------
        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            bodyProfile, shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = profileName + ' Body'

        # --- loft Apex -> tooth profile --------------------------------------
        toothProfile = self._findSpurToothProfile(toothSketch)
        if toothProfile is None:
            raise Exception('Could not find {} tooth profile'.format(profileName))

        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apex)
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = profileName + ' Tooth Body'

        # --- two conical cuts over the Tooth Body ----------------------------
        cut1Mid = adsk.core.Point3D.create(
            (self._pointWorldGeometry(cut1A).x + self._pointWorldGeometry(cut1B).x) / 2.0,
            (self._pointWorldGeometry(cut1A).y + self._pointWorldGeometry(cut1B).y) / 2.0,
            (self._pointWorldGeometry(cut1A).z + self._pointWorldGeometry(cut1B).z) / 2.0)
        cut2Mid = adsk.core.Point3D.create(
            (self._pointWorldGeometry(cut2A).x + self._pointWorldGeometry(cut2B).x) / 2.0,
            (self._pointWorldGeometry(cut2A).y + self._pointWorldGeometry(cut2B).y) / 2.0,
            (self._pointWorldGeometry(cut2A).z + self._pointWorldGeometry(cut2B).z) / 2.0)

        gearLabel = 'driving' if isDriving else 'pinion'
        tooth = self._applyConicalCut(
            designComponent, toothBody, gearBody, cut1Mid, cut2Mid, apexWorld,
            gearLabel)

        # --- circular pattern of the tooth around the shaft edge -------------
        toolBodies = adsk.core.ObjectCollection.create()
        toolBodies.add(tooth)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(toolBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # --- combine-join all tooth pieces into the gear body ----------------
        joinTools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            joinTools.add(patternResult.bodies.item(i))
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, joinTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- bore -------------------------------------------------------------
        if boreEnable:
            self._cutBore(designComponent, gearBody, shaftEdge, bore_cm, profileName)

        # --- meshing rotation (driving only) ---------------------------------
        if isDriving:
            startW = shaftEdge.startSketchPoint.worldGeometry
            endW = shaftEdge.endSketchPoint.worldGeometry
            axisVector = startW.vectorTo(endW)
            axisVector.normalize()
            angle = math.radians(180.0 / drivingTeeth)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(angle, axisVector, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = designComponent.features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- relocate finished body into the gear component ------------------
        gearBody.moveToComponent(gearOccurrence)

    # -------------------------------------------------------------------------
    # bore cut along the shaft axis
    # -------------------------------------------------------------------------
    def _cutBore(self, designComponent, gearBody, shaftEdge, bore_cm, profileName):
        # bore plane: normal to the shaft at its start
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = profileName + ' Bore Plane'

        sketch = designComponent.sketches.add(borePlane)
        sketch.name = profileName + ' Bore'
        sketch.isVisible = True

        # circle centered at sketch origin (the plane is rooted on the axis)
        circles = sketch.sketchCurves.sketchCircles
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore_cm / 2.0)
        circle.isConstruction = False

        boreProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 1:
                    boreProfile = profile
                    break
            if boreProfile is not None:
                break
        if boreProfile is None:
            raise Exception('Could not find {} bore profile'.format(profileName))

        extrudes = designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        largeExtent = max(1000.0, bore_cm * 100.0)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(largeExtent), False)
        extrudeInput.participantBodies = [gearBody]
        extrudes.add(extrudeInput)

    # -------------------------------------------------------------------------
    # cleanup
    # -------------------------------------------------------------------------
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
                try:
                    sketch.isLightBulbOn = False
                except:
                    pass
                try:
                    sketch.isVisible = False
                except:
                    pass

            for plane in component.constructionPlanes:
                if plane.entityToken in seen:
                    continue
                seen.add(plane.entityToken)
                try:
                    plane.isLightBulbOn = False
                except:
                    pass

            for axis in component.constructionAxes:
                if axis.entityToken in seen:
                    continue
                seen.add(axis.entityToken)
                try:
                    axis.isLightBulbOn = False
                except:
                    pass

            for occurrence in component.occurrences:
                walk(occurrence.component)

        walk(bevelComponent)
