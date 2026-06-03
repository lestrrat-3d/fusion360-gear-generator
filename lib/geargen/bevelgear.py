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

# Pressure angle is not a bevel dialog input; bevel hardcodes 20 degrees.
PRESSURE_ANGLE_DEG = 20.0
INVOLUTE_STEPS = 15


class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first, so it wins Fusion's auto-focus).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        # 2. Center Point.
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Select the point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1)

        # 3. Parent Component (pre-selected to the root component).
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module (unitless).
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle (deg).
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6. Driving Gear Teeth.
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth.
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

        # 10. Enable Bore (checkbox, default True).
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


class _Val:
    """A tiny value-wrapper so a precomputed number can be read as `.value`."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """A fake spur `parent` so the borrowed spur tooth generator can run without
    registering Fusion user parameters. Precomputes, in internal cm, exactly the
    keys the spur drawer reads (via parent.getParameter(name).value)."""

    def __init__(self, module_mm, virtualTeeth):
        module = module_mm  # raw mm
        pressureAngle = math.radians(PRESSURE_ANGLE_DEG)

        pitchDiameter_mm = virtualTeeth * module
        baseDiameter_mm = pitchDiameter_mm * math.cos(pressureAngle)
        rootDiameter_mm = pitchDiameter_mm - 2.5 * module
        tipDiameter_mm = pitchDiameter_mm + 2.0 * module

        # circle radii / diameters served to the spur generator must be in cm.
        self._values = {
            'Module': to_cm(module),
            'ToothNumber': virtualTeeth,
            'PressureAngle': pressureAngle,  # radians; unitless angle
            'PitchCircleDiameter': to_cm(pitchDiameter_mm),
            'PitchCircleRadius': to_cm(pitchDiameter_mm / 2),
            'BaseCircleDiameter': to_cm(baseDiameter_mm),
            'BaseCircleRadius': to_cm(baseDiameter_mm / 2),
            'RootCircleDiameter': to_cm(rootDiameter_mm),
            'RootCircleRadius': to_cm(rootDiameter_mm / 2),
            'TipCircleDiameter': to_cm(tipDiameter_mm),
            'TipCircleRadius': to_cm(tipDiameter_mm / 2),
            'InvoluteSteps': INVOLUTE_STEPS,
        }
        # The spur drawer records its embedded flag here.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

        # Stashed (set in _readInputs).
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0

    # ------------------------------------------------------------------ inputs
    def _readInputs(self, inputs):
        """Read+validate all inputs. Returns the 7-tuple
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth,
        pinionTeeth, shaftAngle_deg) and stashes the rest on self."""
        design = self.design
        um = design.unitsManager

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
            raise Exception('Selected parent is not a component')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        targetPlane = planes[0]

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # --- numeric / angle / bool, read by evaluating each expression ---
        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return um.evaluateExpression(inp.expression, units)

        # Module: unit '' -> raw number meaning mm.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft Angle: evaluateExpression(..., 'deg') returns RADIANS internally.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30.0 or shaftAngle_deg > 150.0:
            raise Exception('Shaft Angle must be between 30 and 150 degrees')

        # Teeth counts: unit '' -> raw numbers.
        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Tooth numbers must be at least 3')

        # 'mm' inputs come back already in internal cm. Do NOT to_cm again.
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

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ----------------------------------------------------------------- helpers
    def _pointWorldGeometry(self, point):
        """World Point3D for a SketchPoint (.worldGeometry) or a
        ConstructionPoint (.geometry)."""
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _surfaceDistance(self, surface, worldPoint):
        """Distance from worldPoint to the nearest point on a surface."""
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return float('inf')
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return float('inf')
        return projected.distanceTo(worldPoint)

    def _findConeFaceForCutLine(self, body, worldA, worldB, tolerance=1e-2):
        """Among the ConeSurfaceType faces of `body`, return the best-matching
        one whose surface contains both world endpoints within tolerance (the
        smallest total endpoint distance), not merely the first found."""
        best = None
        bestDistance = None
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            surface = face.geometry
            dA = self._surfaceDistance(surface, worldA)
            dB = self._surfaceDistance(surface, worldB)
            if dA <= tolerance and dB <= tolerance:
                total = dA + dB
                if bestDistance is None or total < bestDistance:
                    bestDistance = total
                    best = face
        return best

    def _findSpurToothProfile(self, sketch):
        """Match the tooth cross-section loop by curve-TYPE mix: 2 NURBS flanks
        + 2 arcs (tip/root) + 2 lines (non-embedded), or 2 NURBS + 2 arcs + 0
        lines (embedded)."""
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                nurbs = 0
                arcs = 0
                lines = 0
                other = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        lines += 1
                    else:
                        other += 1
                if other != 0:
                    continue
                if nurbs == 2 and arcs == 2 and lines in (0, 2):
                    return profile
        raise Exception('Could not find spur tooth profile loop')

    # --------------------------------------------------------------- top-level
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Resolve pitch diameters (Python, cm). Module is mm -> to_cm.
        drivingPD_cm = to_cm(module * drivingTeeth)
        pinionPD_cm = to_cm(module * pinionTeeth)

        # Resolve bore diameters (cm). 0 => auto = PD / 4.
        drivingBore_cm = self._drivingBore_cm
        if drivingBore_cm <= 0:
            drivingBore_cm = drivingPD_cm / 4.0
        pinionBore_cm = self._pinionBore_cm
        if pinionBore_cm <= 0:
            pinionBore_cm = pinionPD_cm / 4.0

        # Cone distance (cm).
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # --- build the component tree ---
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # Activate the Design occurrence so all features run there.
        designOccurrence.activate()

        # §1 Anchor Sketch.
        anchorData = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear body creation.
        self._buildGearProfiles(
            designComponent, bevelComponent, anchorData,
            module=module,
            drivingTeeth=drivingTeeth, pinionTeeth=pinionTeeth,
            shaftAngle_deg=shaftAngle_deg,
            drivingPD_cm=drivingPD_cm, pinionPD_cm=pinionPD_cm,
            coneDistance_cm=coneDistance_cm,
            drivingBore_cm=drivingBore_cm, pinionBore_cm=pinionBore_cm)

        # Cleanup.
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # ------------------------------------------------------------- §1 anchor
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Normalize the target plane into a ConstructionPlane if needed.
        plane = targetPlane
        if plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = designComponent.constructionPlanes.createInput()
            planeInput.setByOffset(plane, adsk.core.ValueInput.createByReal(0))
            plane = designComponent.constructionPlanes.add(planeInput)

        sketch = designComponent.sketches.add(plane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        # Anchor Line: a reference line through the projected center, with the
        # center at its midpoint. Length is arbitrary (10 mm).
        lines = sketch.sketchCurves.sketchLines
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        half = to_cm(5)  # 10 mm total
        cx = projectedCenter.geometry.x
        cy = projectedCenter.geometry.y
        p1 = adsk.core.Point3D.create(cx - half, cy, 0)
        p2 = adsk.core.Point3D.create(cx + half, cy, 0)
        anchorLine = lines.addByTwoPoints(p1, p2)
        constraints.addMidPoint(projectedCenter, anchorLine)
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cx, cy + half, 0))

        return {
            'sketch': sketch,
            'plane': plane,
            'targetPlane': targetPlane,
            'centerPoint': centerPoint,
            'projectedCenter': projectedCenter,
            'anchorLine': anchorLine,
        }

    # --------------------------------------------------------- §2/§3 profiles
    def _buildGearProfiles(self, designComponent, bevelComponent, anchorData,
                           module, drivingTeeth, pinionTeeth, shaftAngle_deg,
                           drivingPD_cm, pinionPD_cm, coneDistance_cm,
                           drivingBore_cm, pinionBore_cm):
        anchorSketch = anchorData['sketch']
        anchorLine = anchorData['anchorLine']
        targetPlane = anchorData['targetPlane']
        centerPoint = anchorData['centerPoint']

        # Gear Profiles plane: includes the anchor line, perpendicular to the
        # anchor sketch plane (setByAngle 90 deg, line passed DIRECTLY).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'),
            anchorData['plane'])
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        lines = sketch.sketchCurves.sketchLines
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        # Project the center point into this sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        # --- Apex (free line endpoint, placed numerically) ---
        centerWorld = self._pointWorldGeometry(centerPoint)
        normal = get_normal(targetPlane)
        normal.normalize()

        # apex_world = projected-center world offset along normal by drivingPD.
        # Use the projected center's world position as the on-plane base.
        baseWorld = projectedCenter.worldGeometry
        apexWorld = adsk.core.Point3D.create(
            baseWorld.x + normal.x * drivingPD_cm,
            baseWorld.y + normal.y * drivingPD_cm,
            baseWorld.z + normal.z * drivingPD_cm)
        apexLocal = sketch.modelToSketchSpace(apexWorld)

        # y_up_sign: compare sketch-space Y of (centerWorld + normal) vs center.
        normalTipWorld = adsk.core.Point3D.create(
            baseWorld.x + normal.x, baseWorld.y + normal.y, baseWorld.z + normal.z)
        normalTipLocal = sketch.modelToSketchSpace(normalTipWorld)
        centerLocal = projectedCenter.geometry
        y_up_sign = 1.0 if (normalTipLocal.y - centerLocal.y) >= 0 else -1.0

        # The construction line from projected center UP to the apex.
        centerToApex = lines.addByTwoPoints(projectedCenter.geometry, apexLocal)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addVertical(centerToApex)
        apex = centerToApex.endSketchPoint

        # Helper: create a line from raw coords whose start coincides with an
        # existing point (share OR coincident, never both -> create from coords).
        def lineFromPoint(startPointObj, endCoord, construction=True):
            startCoord = startPointObj.geometry
            line = lines.addByTwoPoints(startCoord, endCoord)
            line.isConstruction = construction
            constraints.addCoincident(line.startSketchPoint, startPointObj)
            return line

        # --- closed-form cone geometry for seeding A/B along-shaft lengths ---
        Sigma = math.radians(shaftAngle_deg)
        PPD = pinionPD_cm
        DPD = drivingPD_cm
        # tan gamma_p = sin Sigma * PPD / (DPD + PPD * cos Sigma)
        gamma_p = math.atan2(math.sin(Sigma) * PPD, DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        R = (PPD / 2.0) / math.sin(gamma_p) if math.sin(gamma_p) != 0 else coneDistance_cm
        lenApexA = R * math.cos(gamma_p)  # |Apex->A|
        lenApexB = R * math.cos(gamma_g)  # |Apex->B|

        apexCoord = apex.geometry  # current seeded apex coordinates

        # --- Driving Gear Shaft Axis: vertical, downward from apex, end = B ---
        bSeed = adsk.core.Point3D.create(
            apexCoord.x, apexCoord.y - y_up_sign * lenApexB, 0)
        drivingShaftAxis = lines.addByTwoPoints(apexCoord, bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addVertical(drivingShaftAxis)
        B = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis: end = A, at Shaft Angle from driving. ---
        # Drawn on the +X side. Direction makes angle Sigma with the
        # downward driving axis.
        downDir = adsk.core.Vector3D.create(0, -y_up_sign, 0)
        # rotate downDir toward +X by Sigma.
        aDirX = downDir.x * math.cos(-Sigma) - downDir.y * math.sin(-Sigma)
        aDirY = downDir.x * math.sin(-Sigma) + downDir.y * math.cos(-Sigma)
        # pick the sign putting A on the +X half-plane.
        if aDirX < 0:
            aDirX = downDir.x * math.cos(Sigma) - downDir.y * math.sin(Sigma)
            aDirY = downDir.x * math.sin(Sigma) + downDir.y * math.cos(Sigma)
        aSeed = adsk.core.Point3D.create(
            apexCoord.x + aDirX * lenApexA,
            apexCoord.y + aDirY * lenApexA, 0)
        pinionShaftAxis = lines.addByTwoPoints(apexCoord, aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        A = pinionShaftAxis.endSketchPoint
        # Angular dimension Sigma; text point inside the wedge between the axes.
        wedgeMid = adsk.core.Point3D.create(
            apexCoord.x + (aDirX + downDir.x) * 0.25 * lenApexA,
            apexCoord.y + (aDirY + downDir.y) * 0.25 * lenApexA, 0)
        dims.addAngularDimension(drivingShaftAxis, pinionShaftAxis, wedgeMid)

        # --- Apex2: perpendicular drops from A (PPD/2) and B (DPD/2) ---
        # Seed Apex2 near the geometric intersection.
        # perpendicular to pinion axis from A, toward between-axes side.
        aDir = adsk.core.Vector3D.create(aDirX, aDirY, 0)
        # perpendicular candidates: rotate aDir by +/-90; choose toward anchor.
        perpA1 = adsk.core.Vector3D.create(-aDir.y, aDir.x, 0)
        # midpoint between the two shaft directions (the between-axes side).
        towardMid = adsk.core.Vector3D.create(
            (aDir.x + downDir.x), (aDir.y + downDir.y), 0)
        if perpA1.x * towardMid.x + perpA1.y * towardMid.y < 0:
            perpA = adsk.core.Vector3D.create(aDir.y, -aDir.x, 0)
        else:
            perpA = perpA1
        apex2Seed = adsk.core.Point3D.create(
            aSeed.x + perpA.x * (PPD / 2.0),
            aSeed.y + perpA.y * (PPD / 2.0), 0)

        lineAtoApex2 = lines.addByTwoPoints(aSeed, apex2Seed)
        lineAtoApex2.isConstruction = True
        constraints.addCoincident(lineAtoApex2.startSketchPoint, A)
        constraints.addPerpendicular(lineAtoApex2, pinionShaftAxis)
        dimTextA = adsk.core.Point3D.create(
            (aSeed.x + apex2Seed.x) / 2, (aSeed.y + apex2Seed.y) / 2, 0)
        dims.addDistanceDimension(
            lineAtoApex2.startSketchPoint, lineAtoApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            dimTextA).parameter.value = PPD / 2.0

        lineBtoApex2 = lines.addByTwoPoints(bSeed, apex2Seed)
        lineBtoApex2.isConstruction = True
        constraints.addCoincident(lineBtoApex2.startSketchPoint, B)
        constraints.addPerpendicular(lineBtoApex2, drivingShaftAxis)
        dimTextB = adsk.core.Point3D.create(
            (bSeed.x + apex2Seed.x) / 2, (bSeed.y + apex2Seed.y) / 2, 0)
        dims.addDistanceDimension(
            lineBtoApex2.startSketchPoint, lineBtoApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            dimTextB).parameter.value = DPD / 2.0

        # Close at Apex2.
        constraints.addCoincident(
            lineAtoApex2.endSketchPoint, lineBtoApex2.endSketchPoint)
        apex2 = lineAtoApex2.endSketchPoint

        # --- Pitch Line: Apex -> Apex2 ---
        pitchLine = lines.addByTwoPoints(apexCoord, apex2Seed)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # --- Dedendum lines from Apex2, perpendicular to pitch line ---
        dedendum_cm = to_cm(1.25 * module)
        pitchDir = adsk.core.Vector3D.create(
            apex2Seed.x - apexCoord.x, apex2Seed.y - apexCoord.y, 0)
        pitchDir.normalize()
        perpPitch = adsk.core.Vector3D.create(-pitchDir.y, pitchDir.x, 0)
        # Driving dedendum (point D) toward the anchor line; pinion (C) away.
        anchorDir = adsk.core.Vector3D.create(0, -y_up_sign, 0)  # toward anchor line
        if perpPitch.x * anchorDir.x + perpPitch.y * anchorDir.y >= 0:
            towardAnchor = perpPitch
            awayAnchor = adsk.core.Vector3D.create(-perpPitch.x, -perpPitch.y, 0)
        else:
            towardAnchor = adsk.core.Vector3D.create(-perpPitch.x, -perpPitch.y, 0)
            awayAnchor = perpPitch

        dSeed = adsk.core.Point3D.create(
            apex2Seed.x + towardAnchor.x * dedendum_cm,
            apex2Seed.y + towardAnchor.y * dedendum_cm, 0)
        drivingDedendum = lines.addByTwoPoints(apex2Seed, dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2Seed.x + dSeed.x) / 2,
                                     (apex2Seed.y + dSeed.y) / 2, 0)).parameter.value = dedendum_cm
        D = drivingDedendum.endSketchPoint

        cSeed = adsk.core.Point3D.create(
            apex2Seed.x + awayAnchor.x * dedendum_cm,
            apex2Seed.y + awayAnchor.y * dedendum_cm, 0)
        pinionDedendum = lines.addByTwoPoints(apex2Seed, cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2Seed.x + cSeed.x) / 2,
                                     (apex2Seed.y + cSeed.y) / 2, 0)).parameter.value = dedendum_cm
        C = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex -> D (driving), Apex -> C (pinion) ---
        drivingRootAxis = lines.addByTwoPoints(apexCoord, dSeed)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, D)

        pinionRootAxis = lines.addByTwoPoints(apexCoord, cSeed)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, C)

        module_cm = to_cm(module)

        # --- E: from A, colinear with Apex->A, length module (no dim) ---
        aDirUnit = adsk.core.Vector3D.create(aDir.x, aDir.y, 0)
        aDirUnit.normalize()
        eSeed = adsk.core.Point3D.create(
            aSeed.x + aDirUnit.x * module_cm,
            aSeed.y + aDirUnit.y * module_cm, 0)
        lineAE = lines.addByTwoPoints(aSeed, eSeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, A)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        E = lineAE.endSketchPoint

        # C -> E, perpendicular to A->E.
        lineCE = lines.addByTwoPoints(cSeed, eSeed)
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, C)
        constraints.addCoincident(lineCE.endSketchPoint, E)
        constraints.addPerpendicular(lineAE, lineCE)

        # --- F: from B, colinear with Apex->B, length module (no dim) ---
        bDirUnit = adsk.core.Vector3D.create(0, -y_up_sign, 0)
        fSeed = adsk.core.Point3D.create(
            bSeed.x + bDirUnit.x * module_cm,
            bSeed.y + bDirUnit.y * module_cm, 0)
        lineBF = lines.addByTwoPoints(bSeed, fSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, B)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        F = lineBF.endSketchPoint

        # D -> F, perpendicular to B->F.
        lineDF = lines.addByTwoPoints(dSeed, fSeed)
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, D)
        constraints.addCoincident(lineDF.endSketchPoint, F)
        constraints.addPerpendicular(lineBF, lineDF)

        # --- G: from E, colinear with A->E, length module (no dim) ---
        gSeed = adsk.core.Point3D.create(
            eSeed.x + aDirUnit.x * module_cm,
            eSeed.y + aDirUnit.y * module_cm, 0)
        lineEG = lines.addByTwoPoints(eSeed, gSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, E)
        constraints.addCollinear(lineEG, lineAE)
        G = lineEG.endSketchPoint

        # --- H: from C, length module, colinear with Apex2->C ---
        cDirUnit = adsk.core.Vector3D.create(awayAnchor.x, awayAnchor.y, 0)
        cDirUnit.normalize()
        hSeed = adsk.core.Point3D.create(
            cSeed.x + cDirUnit.x * module_cm,
            cSeed.y + cDirUnit.y * module_cm, 0)
        lineCH = lines.addByTwoPoints(cSeed, hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, C)
        constraints.addCollinear(lineCH, pinionDedendum)
        H = lineCH.endSketchPoint

        # G -> H, perpendicular to E->G.
        lineGH = lines.addByTwoPoints(gSeed, hSeed)
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, G)
        constraints.addCoincident(lineGH.endSketchPoint, H)
        constraints.addPerpendicular(lineEG, lineGH)

        # --- I: from F, colinear with B->F, length module (no dim) ---
        iSeed = adsk.core.Point3D.create(
            fSeed.x + bDirUnit.x * module_cm,
            fSeed.y + bDirUnit.y * module_cm, 0)
        lineFI = lines.addByTwoPoints(fSeed, iSeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, F)
        constraints.addCollinear(lineFI, lineBF)
        I = lineFI.endSketchPoint

        # --- J: from D, length module, colinear with Apex2->D ---
        dDirUnit = adsk.core.Vector3D.create(towardAnchor.x, towardAnchor.y, 0)
        dDirUnit.normalize()
        jSeed = adsk.core.Point3D.create(
            dSeed.x + dDirUnit.x * module_cm,
            dSeed.y + dDirUnit.y * module_cm, 0)
        lineDJ = lines.addByTwoPoints(dSeed, jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, D)
        constraints.addCollinear(lineDJ, drivingDedendum)
        J = lineDJ.endSketchPoint

        # I -> J, perpendicular to F->I.
        lineIJ = lines.addByTwoPoints(iSeed, jSeed)
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, I)
        constraints.addCoincident(lineIJ.endSketchPoint, J)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- Base height dimensions ---
        # Apex2->B vs J->I : driving base height (or module*drivingTeeth/8).
        drivingBaseHeight_cm = self._drivingBaseHeight_cm
        if drivingBaseHeight_cm <= 0:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8.0)
        # lines must be parallel for addOffsetDimension; B->Apex2 || I->J here.
        offDimBI = dims.addOffsetDimension(
            lineBtoApex2, lineIJ,
            adsk.core.Point3D.create(jSeed.x, jSeed.y + to_cm(1), 0))
        offDimBI.parameter.value = drivingBaseHeight_cm

        # Apex2->A vs G->H : pinion base height (or driving*pinionT/drivingT).
        pinionBaseHeight_cm = self._pinionBaseHeight_cm
        if pinionBaseHeight_cm <= 0:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        offDimAG = dims.addOffsetDimension(
            lineAtoApex2, lineGH,
            adsk.core.Point3D.create(hSeed.x, hSeed.y + to_cm(1), 0))
        offDimAG.parameter.value = pinionBaseHeight_cm

        # --- A to G line ---
        lineAG = lines.addByTwoPoints(aSeed, gSeed)
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, A)
        constraints.addCoincident(lineAG.endSketchPoint, G)

        # Constrain point I with center point.
        constraints.addCoincident(I, projectedCenter)

        # --- K: from G along Apex->A (away from apex); two coincident pins ---
        kSeed = adsk.core.Point3D.create(
            gSeed.x + aDirUnit.x * module_cm,
            gSeed.y + aDirUnit.y * module_cm, 0)
        lineGK = lines.addByTwoPoints(gSeed, kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, G)
        K = lineGK.endSketchPoint
        # Pin K with two point-on-line coincidents (NOT addCollinear).
        constraints.addCoincident(K, pinionShaftAxis)   # on Apex->A
        constraints.addCoincident(K, pinionDedendum)    # on Apex2->C extended
        lineCK = lines.addByTwoPoints(cSeed, kSeed)
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, C)
        constraints.addCoincident(lineCK.endSketchPoint, K)

        # --- Maximum Face Width (now A,B,C,D,H,J exist) ---
        # Perp distance from A to line C->H (pinion dedendum line), and from B
        # to line D->J (driving dedendum line). Compute in current coords.
        def perpDistance(pCoord, lineStartCoord, lineEndCoord):
            dx = lineEndCoord.x - lineStartCoord.x
            dy = lineEndCoord.y - lineStartCoord.y
            length = math.hypot(dx, dy)
            if length == 0:
                return float('inf')
            return abs(dx * (lineStartCoord.y - pCoord.y)
                       - (lineStartCoord.x - pCoord.x) * dy) / length

        distA = perpDistance(A.geometry, C.geometry, H.geometry)
        distB = perpDistance(B.geometry, D.geometry, J.geometry)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        faceWidth_cm = self._faceWidth_cm
        if faceWidth_cm <= 0:
            faceWidth_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)
        elif faceWidth_cm > maxFaceWidth_cm:
            raise Exception(
                'Face Width exceeds the maximum of {:.4f} mm'.format(
                    to_mm(maxFaceWidth_cm)))

        # --- M->N toe line (pinion) ---
        # Seed M ~ midpoint of Apex->C; seed N by sliding from M-seed along the
        # C->H direction far enough to roughly reach line A->Apex2 (by the
        # M-seed -> A distance). NOT faceWidth from C/H.
        mSeed = adsk.core.Point3D.create(
            (apexCoord.x + cSeed.x) / 2.0,
            (apexCoord.y + cSeed.y) / 2.0, 0)
        chDir = adsk.core.Vector3D.create(
            hSeed.x - cSeed.x, hSeed.y - cSeed.y, 0)
        chDir.normalize()
        mToA = math.hypot(aSeed.x - mSeed.x, aSeed.y - mSeed.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + chDir.x * mToA,
            mSeed.y + chDir.y * mToA, 0)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        M = lineMN.startSketchPoint
        N = lineMN.endSketchPoint
        constraints.addCoincident(M, pinionRootAxis)     # M on Apex->C
        constraints.addCoincident(N, lineAtoApex2)        # N on A->Apex2
        constraints.addParallel(lineMN, lineCH)
        offDimMN = dims.addOffsetDimension(
            lineCH, lineMN,
            adsk.core.Point3D.create(
                (mSeed.x + nSeed.x) / 2.0,
                (mSeed.y + nSeed.y) / 2.0 + to_cm(0.5), 0))
        offDimMN.parameter.value = faceWidth_cm

        # M->C and N->A lines.
        lineMC = lines.addByTwoPoints(mSeed, cSeed)
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, M)
        constraints.addCoincident(lineMC.endSketchPoint, C)
        lineNA = lines.addByTwoPoints(nSeed, aSeed)
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, N)
        constraints.addCoincident(lineNA.endSketchPoint, A)

        # --- L: from I along Apex->B (away from apex); two coincident pins ---
        lSeed = adsk.core.Point3D.create(
            iSeed.x + bDirUnit.x * module_cm,
            iSeed.y + bDirUnit.y * module_cm, 0)
        lineIL = lines.addByTwoPoints(iSeed, lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, I)
        L = lineIL.endSketchPoint
        constraints.addCoincident(L, drivingShaftAxis)   # on Apex->B
        constraints.addCoincident(L, drivingDedendum)    # on Apex2->D extended
        lineDL = lines.addByTwoPoints(dSeed, lSeed)
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, D)
        constraints.addCoincident(lineDL.endSketchPoint, L)

        # --- O->P toe line (driving), mirror of M->N ---
        oSeed = adsk.core.Point3D.create(
            (apexCoord.x + dSeed.x) / 2.0,
            (apexCoord.y + dSeed.y) / 2.0, 0)
        djDir = adsk.core.Vector3D.create(
            jSeed.x - dSeed.x, jSeed.y - dSeed.y, 0)
        djDir.normalize()
        oToB = math.hypot(bSeed.x - oSeed.x, bSeed.y - oSeed.y)
        pSeed = adsk.core.Point3D.create(
            oSeed.x + djDir.x * oToB,
            oSeed.y + djDir.y * oToB, 0)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        O = lineOP.startSketchPoint
        P = lineOP.endSketchPoint
        constraints.addCoincident(O, drivingRootAxis)    # O on Apex->D
        constraints.addCoincident(P, lineBtoApex2)        # P on B->Apex2
        constraints.addParallel(lineOP, lineDJ)
        offDimOP = dims.addOffsetDimension(
            lineDJ, lineOP,
            adsk.core.Point3D.create(
                (oSeed.x + pSeed.x) / 2.0,
                (oSeed.y + pSeed.y) / 2.0 + to_cm(0.5), 0))
        offDimOP.parameter.value = faceWidth_cm

        # O->D and P->B lines, and B->I.
        lineOD = lines.addByTwoPoints(oSeed, dSeed)
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, O)
        constraints.addCoincident(lineOD.endSketchPoint, D)
        linePB = lines.addByTwoPoints(pSeed, bSeed)
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, P)
        constraints.addCoincident(linePB.endSketchPoint, B)
        lineBI = lines.addByTwoPoints(bSeed, iSeed)
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, B)
        constraints.addCoincident(lineBI.endSketchPoint, I)

        # Done with §2 sketch; consume later. Keep visible for projection.
        gearProfiles = {
            'sketch': sketch,
            'plane': gearProfilesPlane,
            'apex': apex, 'apex2': apex2,
            'A': A, 'B': B, 'C': C, 'D': D, 'E': E, 'F': F, 'G': G,
            'H': H, 'I': I, 'J': J, 'K': K, 'L': L,
            'M': M, 'N': N, 'O': O, 'P': P,
            'lineCK': lineCK, 'lineDL': lineDL,
            'gamma_p': gamma_p, 'gamma_g': gamma_g,
        }

        # §3 tooth profiles.
        pinionTooth = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, gearProfiles['lineCK'],
            gearProfiles['K'], module, pinionPD_cm, gamma_p)
        drivingTooth = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, gearProfiles['lineDL'],
            gearProfiles['L'], module, drivingPD_cm, gamma_g)

        # Hide §2 sketch now (after projections happen in _createGearBody we
        # re-project into fresh per-gear sketches, so this sketch is consumed
        # only as anchor source; keep it visible until bodies are built).

        # --- Pinion gear (built first; no mesh offset) ---
        self._createGearBody(
            designComponent, bevelComponent, gearProfiles, pinionTooth,
            ['A', 'G', 'H', 'C', 'M', 'N'], 'Pinion',
            pinionTeeth, pinionBore_cm, mesh=False)

        # --- Driving gear (built second; mesh offset 180/drivingTeeth) ---
        self._createGearBody(
            designComponent, bevelComponent, gearProfiles, drivingTooth,
            ['B', 'I', 'J', 'D', 'O', 'P'], 'Driving',
            drivingTeeth, drivingBore_cm, mesh=True)

        sketch.isVisible = False

    # --------------------------------------------------- §3 virtual spur tooth
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 lineToCenter, centerPoint, module,
                                 pitchDiameter_cm, gamma):
        # Virtual (back-cone / Tredgold) tooth number.
        virtualPitchRadius_cm = (pitchDiameter_cm / 2.0) / math.cos(gamma)
        virtualPitchRadius_mm = to_mm(virtualPitchRadius_cm)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / module))

        # Tooth plane: includes lineToCenter, perpendicular to Gear Profiles
        # plane (setByAngle 90 deg; line passed DIRECTLY).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            lineToCenter, adsk.core.ValueInput.createByString('90 deg'),
            gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = 'Tooth Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = 'Tooth Profile'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        # The 180-degree tooth rotation IS the draw() angle.
        drawer.draw(centerPoint, angle=math.radians(180))

        # Construction axis through the center point, normal to the tooth plane
        # via setByTwoPlanes: Gear Profiles plane intersected with a helper
        # plane setByDistanceOnPath(lineToCenter, 1.0).
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            lineToCenter, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = 'Tooth Axis Helper'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = 'Tooth Axis'

        return {
            'sketch': toothSketch,
            'plane': toothPlane,
            'axis': toothAxis,
            'virtualTeeth': virtualTeeth,
        }

    # -------------------------------------------------------- per-gear bodies
    def _createGearBody(self, designComponent, bevelComponent, gearProfiles,
                        toothData, hexagonKeys, label, teeth, bore_cm, mesh):
        # New component under Bevel Gear for this gear's bodies.
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = '{} Gear'.format(label)
        gearComponent = gearOccurrence.component

        # Fresh sketch on the axial (Gear Profiles) plane; project the 6 points
        # and draw the closed hexagon as six SketchLines. One profile per gear.
        profileSketch = designComponent.sketches.add(gearProfiles['plane'])
        profileSketch.name = '{} Profile'.format(label)
        profileSketch.isVisible = True

        projectedPoints = []
        for key in hexagonKeys:
            proj = profileSketch.project(gearProfiles[key])
            projectedPoints.append(proj.item(0))

        lines = profileSketch.sketchCurves.sketchLines
        constraints = profileSketch.geometricConstraints
        hexLines = []
        count = len(projectedPoints)
        for i in range(count):
            startPt = projectedPoints[i]
            endPt = projectedPoints[(i + 1) % count]
            line = lines.addByTwoPoints(startPt.geometry, endPt.geometry)
            constraints.addCoincident(line.startSketchPoint, startPt)
            constraints.addCoincident(line.endSketchPoint, endPt)
            hexLines.append(line)

        # Shaft axis = the profile sketch's first edge (A->G / B->I), NOT §2.
        shaftEdge = hexLines[0]

        # Find the hexagon profile loop.
        hexProfile = None
        for profile in profileSketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 6:
                    hexProfile = profile
                    break
            if hexProfile is not None:
                break
        if hexProfile is None:
            raise Exception('Could not find {} hexagon profile'.format(label))

        # --- Revolve around the shaft edge ---
        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            hexProfile, shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = '{} Gear Body'.format(label)

        # --- Loft: Apex point + tooth profile = Tooth Body ---
        toothProfile = self._findSpurToothProfile(toothData['sketch'])
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(gearProfiles['apex'])
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = '{} Gear Tooth Body'.format(label)

        # Apex world for piece identification.
        apexWorld = gearProfiles['apex'].worldGeometry

        # --- Two sequential conical cuts ---
        # Cut 1 face: from the toe edge (M->N / O->P).
        toeStart = hexagonKeys[4]  # M / O
        toeEnd = hexagonKeys[5]    # N / P
        toeStartWorld = gearProfiles[toeStart].worldGeometry
        toeEndWorld = gearProfiles[toeEnd].worldGeometry
        # Cut 2 face: from the heel dedendum edge (C->H / D->J).
        heelStart = hexagonKeys[3]  # C / D
        heelEnd = hexagonKeys[2]    # H / J
        heelStartWorld = gearProfiles[heelStart].worldGeometry
        heelEndWorld = gearProfiles[heelEnd].worldGeometry

        cone1 = self._findConeFaceForCutLine(
            gearBody, toeStartWorld, toeEndWorld)
        if cone1 is None:
            raise Exception('Could not find toe cone face for {}'.format(label))
        pieces = self._applyConicalCut(
            designComponent, [toothBody], cone1)

        cone2 = self._findConeFaceForCutLine(
            gearBody, heelStartWorld, heelEndWorld)
        if cone2 is None:
            raise Exception('Could not find heel cone face for {}'.format(label))
        pieces = self._applyConicalCut(designComponent, pieces, cone2)

        if len(pieces) != 3:
            raise Exception(
                'Expected exactly 3 pieces after cuts for {}, got {}'.format(
                    label, len(pieces)))

        # Remove the piece containing the Apex.
        remaining = []
        apexPiece = None
        for piece in pieces:
            containment = piece.pointContainment(apexWorld)
            if (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                    or containment == adsk.fusion.PointContainment.PointOnPointContainment):
                apexPiece = piece
            else:
                remaining.append(piece)
        if apexPiece is None:
            # Fallback: none flagged; treat the largest as apex piece.
            remaining = list(pieces)
            apexPiece = max(remaining, key=lambda b: b.physicalProperties.volume)
            remaining.remove(apexPiece)
        designComponent.features.removeFeatures.add(apexPiece)

        # Of the two remaining, remove the smaller one.
        if len(remaining) != 2:
            raise Exception('Unexpected piece count for {}'.format(label))
        smaller = min(remaining, key=lambda b: b.physicalProperties.volume)
        toothPiece = max(remaining, key=lambda b: b.physicalProperties.volume)
        designComponent.features.removeFeatures.add(smaller)

        # --- Circular-pattern the tooth piece around the shaft edge ---
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(toothPiece)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # --- Combine-Join (gearBody as target; patterned teeth as tools) ---
        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ---
        if self._boreEnable:
            self._cutBore(designComponent, shaftEdge, gearBody, bore_cm)

        # --- Meshing rotation (driving only), in Design, before move out ---
        if mesh:
            startWorld = shaftEdge.startSketchPoint.worldGeometry
            endWorld = shaftEdge.endSketchPoint.worldGeometry
            axisVector = startWorld.vectorTo(endWorld)
            axisVector.normalize()
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(
                math.radians(180.0 / teeth), axisVector, startWorld)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = designComponent.features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- Relocate the finished body into this gear's component ---
        gearBody.moveToComponent(gearOccurrence)

        # Hide this profile sketch.
        profileSketch.isVisible = False

        return gearBody

    def _applyConicalCut(self, designComponent, bodies, coneFace):
        """Split each body in `bodies` by coneFace; keep the resulting pieces.
        If a body doesn't intersect (SPLIT_TARGET_TOOL_NOT_INTERSECT / 交差),
        keep it intact."""
        splits = designComponent.features.splitBodyFeatures
        result = []
        for body in bodies:
            try:
                splitInput = splits.createInput(body, coneFace, True)
                splitResult = splits.add(splitInput)
                for i in range(splitResult.bodies.count):
                    result.append(splitResult.bodies.item(i))
            except RuntimeError as e:
                msg = str(e)
                if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                    result.append(body)
                else:
                    raise
        return result

    def _cutBore(self, designComponent, shaftEdge, gearBody, bore_cm):
        # Bore plane normal to the shaft at its start.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = 'Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = 'Bore Profile'
        boreSketch.isVisible = True

        # Circle centered at the sketch origin (the plane is rooted on the axis).
        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore_cm / 2.0)
        circle.isConstruction = False

        boreProfile = None
        for profile in boreSketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 1:
                    boreProfile = profile
                    break
            if boreProfile is not None:
                break
        if boreProfile is None:
            raise Exception('Could not find bore profile')

        extrudes = designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(max(1000.0, bore_cm * 100.0)), False)
        extrudeInput.participantBodies = [gearBody]
        extrudes.add(extrudeInput)

        boreSketch.isVisible = False

    # ------------------------------------------------------------- cleanup
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                try:
                    sketch.isVisible = False
                    sketch.isLightBulbOn = False
                except Exception:
                    pass
            for plane in component.constructionPlanes:
                try:
                    plane.isLightBulbOn = False
                except Exception:
                    pass
            for axis in component.constructionAxes:
                try:
                    axis.isLightBulbOn = False
                except Exception:
                    pass
            for occurrence in component.occurrences:
                walk(occurrence.component)

        walk(bevelComponent)
