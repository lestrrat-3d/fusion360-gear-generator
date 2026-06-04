import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator

import adsk.core, adsk.fusion


# --- Dialog input ids ---------------------------------------------------------
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

# Bevel hardcodes the pressure angle (not a dialog input).
PRESSURE_ANGLE_DEG = 20
INVOLUTE_STEPS = 15
DISTANCE_TOLERANCE = 1e-2


class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first so it wins Fusion's auto-focus)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        # 2. Center Point
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Select the point to center the driving bevel gear on')
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


class _Val:
    """Tiny value-wrapper so spur's parent.getParameter(name).value works."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """A fake spur `parent` so the borrowed spur tooth generator can run
    without registering any Fusion user parameters. It precomputes (in internal
    cm) exactly the keys the spur drawer reads via parent.getParameter(name).value."""

    def __init__(self, module_mm, virtualTeeth):
        alpha = math.radians(PRESSURE_ANGLE_DEG)
        # Standard spur formulas. Module here is the raw mm value; convert the
        # resulting length quantities to internal cm (what the spur drawer expects).
        pitch_mm = virtualTeeth * module_mm
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2 * module_mm

        self._values = {
            'Module': module_mm,
            'ToothNumber': virtualTeeth,
            'PressureAngle': alpha,
            'PitchCircleDiameter': to_cm(pitch_mm),
            'PitchCircleRadius': to_cm(pitch_mm / 2),
            'BaseCircleDiameter': to_cm(base_mm),
            'BaseCircleRadius': to_cm(base_mm / 2),
            'RootCircleDiameter': to_cm(root_mm),
            'RootCircleRadius': to_cm(root_mm / 2),
            'TipCircleDiameter': to_cm(tip_mm),
            'TipCircleRadius': to_cm(tip_mm / 2),
            'InvoluteSteps': INVOLUTE_STEPS,
        }
        # The spur drawTooth records embedded-ness on the parent.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


class BevelGearGenerator:
    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

        # Stashed input attributes (filled by _readInputs).
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0

    # ------------------------------------------------------------------ inputs
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

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

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Module: read unitless; raw number means MILLIMETRES.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft Angle: evaluateExpression(..,'deg') returns RADIANS internally.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.3f})')

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception(f'Driving Gear Teeth must be >= 3 (got {drivingTeeth})')
        if pinionTeeth < 3:
            raise Exception(f'Pinion Gear Teeth must be >= 3 (got {pinionTeeth})')

        # 'mm' inputs already come back in internal cm; use as-is.
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

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _pointWorldGeometry(p):
        """World Point3D for a SketchPoint (.worldGeometry) or ConstructionPoint (.geometry)."""
        if p.objectType == adsk.fusion.SketchPoint.classType():
            return p.worldGeometry
        return p.geometry

    @staticmethod
    def _surfaceDistance(surface, worldPoint):
        """Distance from a world point to an (infinite) surface, or None if unevaluable
        (e.g. near a cone's apex singularity)."""
        try:
            evaluator = surface.evaluator
            (ok, param) = evaluator.getParameterAtPoint(worldPoint)
            if not ok:
                return None
            (ok2, onSurface) = evaluator.getPointAtParameter(param)
            if not ok2:
                return None
            return onSurface.distanceTo(worldPoint)
        except Exception:
            return None

    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                axis.isLightBulbOn = False

            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)

    # ------------------------------------------------------------------ generate
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Pitch diameters & cone distance, in internal cm (Module is mm).
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # Bore diameters (auto-calc when 0). Only consulted when bore enabled.
        drivingBore_cm = (self._drivingBore_cm if self._drivingBore_cm > 0
                          else drivingPitchDiameter_cm / 4)
        pinionBore_cm = (self._pinionBore_cm if self._pinionBore_cm > 0
                         else pinionPitchDiameter_cm / 4)

        # --- Build the component tree --------------------------------------
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # --- §1 Anchor sketch ----------------------------------------------
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # --- §2 + §3 + per-gear bodies -------------------------------------
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm, coneDistance_cm,
            drivingBore_cm, pinionBore_cm)

        # --- Cleanup -------------------------------------------------------
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # ------------------------------------------------------------------ §1
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Start the Anchor Sketch directly on the user-selected target plane.
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user center onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        cg = projectedCenter.geometry
        anchorLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(cg.x - 0.5, cg.y, cg.z),
            adsk.core.Point3D.create(cg.x + 0.5, cg.y, cg.z))

        # Both the intersection (coincident) and midpoint constraints.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # Reference length (~10mm); value is arbitrary.
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + 0.5, cg.z)
        ).parameter.value = to_cm(10)

        # Pin the direction (sketch-local Horizontal) so the sketch is fully constrained.
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch)
        return anchorLine

    @staticmethod
    def _assertFullyConstrained(sketch):
        if not sketch.isFullyConstrained:
            raise RuntimeError(f"sketch '{sketch.name}' is not fully constrained")

    # ------------------------------------------------------------------ §2/§3 + bodies
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane,
                           anchorLine, module, drivingTeeth, pinionTeeth,
                           shaftAngle_deg, drivingPitchDiameter_cm,
                           pinionPitchDiameter_cm, coneDistance_cm,
                           drivingBore_cm, pinionBore_cm):
        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm
        sigma = math.radians(shaftAngle_deg)

        # --- Gear Profiles plane: perpendicular to target, through anchor line.
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

        # Project the stashed anchor-sketch center point.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        c = projectedCenter.geometry

        # 2-D direction of the projected anchor line.
        p1 = anchorLine.startSketchPoint.worldGeometry
        p2 = anchorLine.endSketchPoint.worldGeometry
        a1 = sketch.modelToSketchSpace(p1)
        a2 = sketch.modelToSketchSpace(p2)
        dx = a2.x - a1.x
        dy = a2.y - a1.y
        dlen = math.hypot(dx, dy)
        dx /= dlen
        dy /= dlen

        # In-plane perpendicular (one of two senses).
        perpx = -dy
        perpy = dx

        # Grow-side sign: pick by target normal as a one-bit direction.
        nWorld = get_normal(targetPlane)
        Pw = projectedCenter.worldGeometry
        o2d = sketch.modelToSketchSpace(Pw)
        n2d = sketch.modelToSketchSpace(adsk.core.Point3D.create(
            Pw.x + nWorld.x, Pw.y + nWorld.y, Pw.z + nWorld.z))
        nDir2Dx = n2d.x - o2d.x
        nDir2Dy = n2d.y - o2d.y
        if (perpx * nDir2Dx + perpy * nDir2Dy) <= 0:
            perpx = -perpx
            perpy = -perpy

        # --- Apex: free end of a construction line from the projected center.
        apexLocal = adsk.core.Point3D.create(
            c.x + perpx * DPD, c.y + perpy * DPD, c.z)
        centerToApex = lines.addByTwoPoints(c, apexLocal)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, anchorLineProjection(sketch, anchorLine))
        apex = centerToApex.endSketchPoint

        # Closed-form seeds for Apex->A (pinion) and Apex->B (driving).
        # tan g_p = sin S * PPD / (DPD + PPD*cos S); g_g = S - g_p
        gamma_p = math.atan2(math.sin(sigma) * PPD, DPD + PPD * math.cos(sigma))
        gamma_g = sigma - gamma_p
        R = (PPD / 2) / math.sin(gamma_p)
        lenApexA = R * math.cos(gamma_p)   # Apex->A (pinion shaft)
        lenApexB = R * math.cos(gamma_g)   # Apex->B (driving shaft)

        # --- Driving Gear Shaft Axis (Apex -> B), seeded toward -perp.
        bSeed = adsk.core.Point3D.create(
            apexLocal.x - perpx * lenApexB, apexLocal.y - perpy * lenApexB, c.z)
        drivingShaftAxis = lines.addByTwoPoints(apexLocal, bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis (Apex -> A). Drawn on +X side; angular dim = sigma.
        # Seed A by rotating -perp toward +X by (sigma) about apex.
        baseDirx = -perpx
        baseDiry = -perpy
        # rotate base dir by +sigma and by -sigma; pick the +X half-plane one.
        def rotated(angle):
            ca = math.cos(angle)
            sa = math.sin(angle)
            return (baseDirx * ca - baseDiry * sa, baseDirx * sa + baseDiry * ca)
        (ax1, ay1) = rotated(sigma)
        (ax2, ay2) = rotated(-sigma)
        # Prefer the sense whose x-component is more positive (the +X half-plane).
        if ax1 >= ax2:
            (adirx, adiry) = (ax1, ay1)
        else:
            (adirx, adiry) = (ax2, ay2)
        aSeed = adsk.core.Point3D.create(
            apexLocal.x + adirx * lenApexA, apexLocal.y + adiry * lenApexA, c.z)
        pinionShaftAxis = lines.addByTwoPoints(apexLocal, aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)

        # Angular dimension between the two shaft axes = Shaft Angle, text inside the wedge.
        wedgeTextx = apexLocal.x + (adirx - perpx) * 0.25 * lenApexA
        wedgeTexty = apexLocal.y + (adiry - perpy) * 0.25 * lenApexA
        dims.addAngularDimension(
            pinionShaftAxis, drivingShaftAxis,
            adsk.core.Point3D.create(wedgeTextx, wedgeTexty, c.z)
        ).parameter.value = sigma
        pointA = pinionShaftAxis.endSketchPoint

        # --- From A: perpendicular drop of length PPD/2 toward Apex2 (A->Apex2).
        apex2Seedx = (aSeed.x + bSeed.x) / 2
        apex2Seedy = (aSeed.y + bSeed.y) / 2
        aDropLine = lines.addByTwoPoints(
            aSeed, adsk.core.Point3D.create(apex2Seedx, apex2Seedy, c.z))
        aDropLine.isConstruction = True
        constraints.addCoincident(aDropLine.startSketchPoint, pointA)
        constraints.addPerpendicular(aDropLine, pinionShaftAxis)
        dims.addDistanceDimension(
            aDropLine.startSketchPoint, aDropLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(apex2Seedx, apex2Seedy + 0.1, c.z)
        ).parameter.value = PPD / 2

        # --- From B: perpendicular drop of length DPD/2 toward Apex2 (B->Apex2).
        bDropLine = lines.addByTwoPoints(
            bSeed, adsk.core.Point3D.create(apex2Seedx, apex2Seedy, c.z))
        bDropLine.isConstruction = True
        constraints.addCoincident(bDropLine.startSketchPoint, pointB)
        constraints.addPerpendicular(bDropLine, drivingShaftAxis)
        dims.addDistanceDimension(
            bDropLine.startSketchPoint, bDropLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(apex2Seedx, apex2Seedy - 0.1, c.z)
        ).parameter.value = DPD / 2

        # --- Apex2: coincident the far ends of the two drops.
        constraints.addCoincident(aDropLine.endSketchPoint, bDropLine.endSketchPoint)
        apex2 = aDropLine.endSketchPoint

        # --- Pitch Line: Apex to Apex2.
        pitchLine = lines.addByTwoPoints(apexLocal, apex2.geometry)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        ded = to_cm(1.25 * module)
        modLen = to_cm(module)

        # Helper for "constrained perpendicular construction line from an existing point".
        def perpLineFrom(startPoint, refLine, length, seedDir):
            sg = startPoint.geometry
            endSeed = adsk.core.Point3D.create(
                sg.x + seedDir[0] * length, sg.y + seedDir[1] * length, sg.z)
            ln = lines.addByTwoPoints(sg, endSeed)
            ln.isConstruction = True
            constraints.addCoincident(ln.startSketchPoint, startPoint)
            constraints.addPerpendicular(ln, refLine)
            return ln

        # Direction from apex2 toward anchor line (= -perp side) and away (= +perp).
        # Driving Gear Dedendum toward the anchor line -> point D; Pinion away -> point C.
        a2g = apex2.geometry
        towardAnchor = (-perpx, -perpy)
        awayAnchor = (perpx, perpy)

        drivingDedendum = perpLineFrom(apex2, pitchLine, ded, towardAnchor)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(a2g.x, a2g.y, a2g.z)
        ).parameter.value = ded
        pointD = drivingDedendum.endSketchPoint

        pinionDedendum = perpLineFrom(apex2, pitchLine, ded, awayAnchor)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(a2g.x, a2g.y, a2g.z)
        ).parameter.value = ded
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex->D (driving), Apex->C (pinion).
        drivingRootAxis = lines.addByTwoPoints(apexLocal, pointD.geometry)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(apexLocal, pointC.geometry)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # Helper: collinear extension of `alongLine` from `startPoint` for `length`.
        def collinearExtension(startPoint, alongLine, length, seedDir):
            sg = startPoint.geometry
            endSeed = adsk.core.Point3D.create(
                sg.x + seedDir[0] * length, sg.y + seedDir[1] * length, sg.z)
            ln = lines.addByTwoPoints(sg, endSeed)
            ln.isConstruction = True
            constraints.addCoincident(ln.startSketchPoint, startPoint)
            constraints.addCollinear(ln, alongLine)
            return ln

        # Direction along Apex->A away from Apex (A side outward).
        def dirAlong(fromPt, toPt):
            fg = fromPt.geometry
            tg = toPt.geometry
            vx = tg.x - fg.x
            vy = tg.y - fg.y
            vlen = math.hypot(vx, vy)
            return (vx / vlen, vy / vlen)

        # --- E: from A, collinear with Apex->A, length module, extending outward.
        dirAA = dirAlong(apex, pointA)
        lineAE = collinearExtension(pointA, pinionShaftAxis, modLen, dirAA)
        pointE = lineAE.endSketchPoint

        # --- C->E line, perpendicular to A->E.
        lineCE = lines.addByTwoPoints(pointC.geometry, pointE.geometry)
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # --- F: from B, collinear with Apex->B, length module, extending outward.
        dirAB = dirAlong(apex, pointB)
        lineBF = collinearExtension(pointB, drivingShaftAxis, modLen, dirAB)
        pointF = lineBF.endSketchPoint

        # --- D->F line, perpendicular to B->F.
        lineDF = lines.addByTwoPoints(pointD.geometry, pointF.geometry)
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # --- G: from E, collinear to A->E, length module.
        lineEG = collinearExtension(pointE, lineAE, modLen, dirAA)
        pointG = lineEG.endSketchPoint

        # --- H: from C, length module, collinear with Apex2->C (pinion dedendum).
        dirA2C = dirAlong(apex2, pointC)
        lineCH = collinearExtension(pointC, pinionDedendum, modLen, dirA2C)
        pointH = lineCH.endSketchPoint

        # --- G->H line, perpendicular against E->G.
        lineGH = lines.addByTwoPoints(pointG.geometry, pointH.geometry)
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # --- I: from F, collinear to B->F, length module.
        lineFI = collinearExtension(pointF, lineBF, modLen, dirAB)
        pointI = lineFI.endSketchPoint

        # --- J: from D, length module, collinear with Apex2->D (driving dedendum).
        dirA2D = dirAlong(apex2, pointD)
        lineDJ = collinearExtension(pointD, drivingDedendum, modLen, dirA2D)
        pointJ = lineDJ.endSketchPoint

        # --- I->J line, perpendicular against F->I.
        lineIJ = lines.addByTwoPoints(pointI.geometry, pointJ.geometry)
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- Offset dim: B->Apex2 drop to J->I = driving base height.
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8)
        midIJ = adsk.core.Point3D.create(
            (pointI.geometry.x + pointJ.geometry.x) / 2,
            (pointI.geometry.y + pointJ.geometry.y) / 2, c.z)
        dims.addOffsetDimension(bDropLine, lineIJ, midIJ).parameter.value = drivingBaseHeight_cm

        # --- Offset dim: A->Apex2 drop to G->H = pinion base height.
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        midGH = adsk.core.Point3D.create(
            (pointG.geometry.x + pointH.geometry.x) / 2,
            (pointG.geometry.y + pointH.geometry.y) / 2, c.z)
        dims.addOffsetDimension(aDropLine, lineGH, midGH).parameter.value = pinionBaseHeight_cm

        # --- A to G line.
        lineAG = lines.addByTwoPoints(pointA.geometry, pointG.geometry)
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # --- Constrain Point I with center point (pins the figure scale via the closing net).
        constraints.addCoincident(pointI, projectedCenter)

        # --- K: from G, construction line away from Apex along Apex->A; pin by two coincidents.
        gK = pointG.geometry
        kSeed = adsk.core.Point3D.create(
            gK.x + dirAA[0] * modLen, gK.y + dirAA[1] * modLen, gK.z)
        lineGK = lines.addByTwoPoints(gK, kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, lineCH)
        lineCK = lines.addByTwoPoints(pointC.geometry, pointK.geometry)
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # --- Maximum Face Width from SOLVED geometry of A, B, C, D, H, J.
        def perpDistPointToLine(pt, la, lb):
            ax_, ay_ = la.x, la.y
            bx_, by_ = lb.x, lb.y
            px_, py_ = pt.x, pt.y
            vx, vy = bx_ - ax_, by_ - ay_
            wx, wy = px_ - ax_, py_ - ay_
            cross = abs(vx * wy - vy * wx)
            vlen = math.hypot(vx, vy)
            return cross / vlen if vlen > 0 else 0.0

        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distA = perpDistPointToLine(gA, gC, gH)   # A to line C-H (pinion dedendum)
        distB = perpDistPointToLine(gB, gD, gJ)   # B to line D-J (driving dedendum)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        # --- Resolve Face Width with the cap.
        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width {to_mm(self._faceWidth_cm):.3f} mm exceeds the maximum '
                    f'{to_mm(maxFaceWidth_cm):.3f} mm for this gear pair')
            faceWidth_cm = self._faceWidth_cm
        else:
            faceWidth_cm = min(coneDistance_cm / 6, maxFaceWidth_cm)

        # --- Toe line M->N on the pinion side.
        # Seed M near the midpoint of Apex->C; seed N slid from M along C->H toward A->Apex2.
        midApexC = adsk.core.Point3D.create(
            (apexLocal.x + gC.x) / 2, (apexLocal.y + gC.y) / 2, c.z)
        dirCH = dirAlong(pointC, pointH)
        distMtoA = math.hypot(gA.x - midApexC.x, gA.y - midApexC.y)
        nSeed = adsk.core.Point3D.create(
            midApexC.x + dirCH[0] * distMtoA, midApexC.y + dirCH[1] * distMtoA, c.z)
        lineMN = lines.addByTwoPoints(midApexC, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)     # M on Apex->C
        constraints.addCoincident(pointN, aDropLine)          # N on A->Apex2 (PPD/2 drop)
        constraints.addParallel(lineMN, lineCH)
        midMN = adsk.core.Point3D.create(
            (pointM.geometry.x + pointN.geometry.x) / 2,
            (pointM.geometry.y + pointN.geometry.y) / 2, c.z)
        dims.addOffsetDimension(lineCH, lineMN, midMN).parameter.value = faceWidth_cm

        lineMC = lines.addByTwoPoints(pointM.geometry, pointC.geometry)
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(pointN.geometry, pointA.geometry)
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- L: from I, construction line away from Apex along Apex->B; pin by two coincidents.
        iL = pointI.geometry
        lSeed = adsk.core.Point3D.create(
            iL.x + dirAB[0] * modLen, iL.y + dirAB[1] * modLen, iL.z)
        lineIL = lines.addByTwoPoints(iL, lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, lineDJ)
        lineDL = lines.addByTwoPoints(pointD.geometry, pointL.geometry)
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # --- Toe line O->P on the driving side (mirror of M->N).
        midApexD = adsk.core.Point3D.create(
            (apexLocal.x + gD.x) / 2, (apexLocal.y + gD.y) / 2, c.z)
        dirDJ = dirAlong(pointD, pointJ)
        distOtoB = math.hypot(gB.x - midApexD.x, gB.y - midApexD.y)
        pSeed = adsk.core.Point3D.create(
            midApexD.x + dirDJ[0] * distOtoB, midApexD.y + dirDJ[1] * distOtoB, c.z)
        lineOP = lines.addByTwoPoints(midApexD, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)    # O on Apex->D
        constraints.addCoincident(pointP, bDropLine)          # P on B->Apex2 (DPD/2 drop)
        constraints.addParallel(lineOP, lineDJ)
        midOP = adsk.core.Point3D.create(
            (pointO.geometry.x + pointP.geometry.x) / 2,
            (pointO.geometry.y + pointP.geometry.y) / 2, c.z)
        dims.addOffsetDimension(lineDJ, lineOP, midOP).parameter.value = faceWidth_cm

        lineOD = lines.addByTwoPoints(pointO.geometry, pointD.geometry)
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(pointP.geometry, pointB.geometry)
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(pointB.geometry, pointI.geometry)
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch)

        sketch.isVisible = False

        # §2 Apex sketch point (used directly as the loft point-section).
        apexSketchPoint = centerToApex.endSketchPoint

        # --- §3: build virtual spur tooth profiles, pinion then driving.
        (pinionToothProfile, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, sketch,
            module, pinionTeeth, PPD, gamma_p, pointC, pointK, lineCK, 'Pinion')

        (drivingToothProfile, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, sketch,
            module, drivingTeeth, DPD, gamma_g, pointD, pointL, lineDL, 'Driving')

        # --- Create the Pinion Gear (no mesh offset).
        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            apexSketchPoint, pinionToothProfile,
            pinionTeeth, pinionBore_cm, apexLocal,
            toeEdge=(pointM, pointN), heelEdge=(pointC, pointH),
            meshOffsetRad=0.0, gearName='Pinion')

        # --- Create the Driving Gear (mesh offset 180/drivingTeeth).
        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            apexSketchPoint, drivingToothProfile,
            drivingTeeth, drivingBore_cm, apexLocal,
            toeEdge=(pointO, pointP), heelEdge=(pointD, pointJ),
            meshOffsetRad=math.radians(180.0 / drivingTeeth), gearName='Driving')

    # ------------------------------------------------------------------ §3
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 gearProfilesSketch, module, teeth, pitchDiameter_cm,
                                 gamma, dedendumPoint, toothCenterPoint, centerToKLine,
                                 gearLabel):
        # Virtual (Tredgold/back-cone) tooth number from the closed form.
        virtualPitchRadius_cm = (pitchDiameter_cm / 2) / math.cos(gamma)
        virtualTeeth = int(math.floor(2 * to_mm(virtualPitchRadius_cm) / module))

        # New plane including line C->K (or D->L), perpendicular to Gear Profiles plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerToKLine, adsk.core.ValueInput.createByString('90 deg'), gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = f'{gearLabel} Tooth Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth Profile'
        toothSketch.isVisible = True

        # Anchor point for the spur drawer = the tooth center point K (or L).
        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCenterPoint, angle=math.radians(180))

        # Gate: the tooth sketch must end fully constrained.
        self._assertFullyConstrained(toothSketch)

        # Revolve/loft profile: the tooth cross-section loop, by curve-type mix.
        toothProfile = self._findSpurToothProfile(toothSketch, proxy._lastToothEmbedded)

        # Construction axis through K (or L) normal to the tooth plane via setByTwoPlanes.
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(centerToKLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = f'{gearLabel} Tooth Axis Helper'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'

        toothSketch.isVisible = False
        return (toothProfile, toothAxis)

    def _findSpurToothProfile(self, toothSketch, embedded):
        # Match the tooth cross-section loop by curve-TYPE mix (not count alone):
        #   non-embedded: 2 NURBS flanks + 2 arcs (tip/root) + 2 lines
        #   embedded:     2 NURBS + 2 arcs + 0 lines
        wantLines = 0 if embedded else 2
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nurbs = arcs = lineCount = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        lineCount += 1
                if nurbs == 2 and arcs == 2 and lineCount == wantLines:
                    return profile
        raise Exception(
            f"Could not find spur tooth profile loop "
            f"(embedded={embedded}, wanted 2 NURBS + 2 arcs + {wantLines} lines)")

    # ------------------------------------------------------------------ body
    def _createGearBody(self, bevelComponent, designComponent, gearProfilesPlane,
                        srcVertices, apexSketchPoint, toothProfile,
                        teeth, bore_cm, apexLocal,
                        toeEdge, heelEdge, meshOffsetRad, gearName):
        # Create the gear component as a child of Bevel Gear.
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{gearName} Gear'

        # --- Fresh profile sketch on the axial (Gear Profiles) plane.
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearName} Gear Profile Sketch'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        # 1. Recreate each §2 vertex as a NEW SketchPoint at its exact location.
        verts = [profileSketch.sketchPoints.add(
                    profileSketch.modelToSketchSpace(self._pointWorldGeometry(p)))
                 for p in srcVertices]
        # 2. Draw the closed hexagon SHARING those points.
        hexLines = [lines.addByTwoPoints(verts[i], verts[(i + 1) % len(verts)])
                    for i in range(len(verts))]
        # 3. NOW fix every line AND its endpoints (after the topology is built).
        for edge in hexLines:
            edge.isFixed = True
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch)

        # Revolve profile: the single closed loop. profiles.count == 1.
        bodyProfile = profileSketch.profiles.item(0)

        # The shaft axis for every body op is this sketch's FIRST edge (A->G / B->I).
        shaftEdge = hexLines[0]

        # --- Revolve about the shaft edge.
        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            bodyProfile, shaftEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = f'{gearName} Gear Body'

        # --- Loft: Apex sketch point -> tooth profile.
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = f'{gearName} Gear Tooth Body'

        # --- Two conical cuts, interleaved keeper selection.
        apexWorld = self._pointWorldGeometry(apexSketchPoint)
        keeper = self._applyConicalCut(
            designComponent, gearBody, toothBody, toeEdge, heelEdge, apexWorld, gearName)

        # --- Circular-pattern the tooth around the shaft edge.
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(keeper)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # --- Combine-Join all patterned teeth into the gear body.
        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore (optional).
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftEdge, bore_cm, gearName)

        # --- Meshing rotation (driving only), before moveToComponent.
        if meshOffsetRad != 0.0:
            startW = shaftEdge.startSketchPoint.worldGeometry
            endW = shaftEdge.endSketchPoint.worldGeometry
            axisVector = adsk.core.Vector3D.create(
                endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(meshOffsetRad, axisVector, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = designComponent.features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- Relocate the finished body into the gear component.
        gearBody.moveToComponent(gearOccurrence)

    def _applyConicalCut(self, designComponent, gearBody, toothBody,
                         toeEdge, heelEdge, apexWorld, gearName):
        splits = designComponent.features.splitBodyFeatures
        removes = designComponent.features.removeFeatures

        def coneFacesByMidpoint(edgePts):
            (pa, pb) = edgePts
            aW = self._pointWorldGeometry(pa)
            bW = self._pointWorldGeometry(pb)
            midW = adsk.core.Point3D.create(
                (aW.x + bW.x) / 2, (aW.y + bW.y) / 2, (aW.z + bW.z) / 2)
            faces = []
            for face in gearBody.faces:
                if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                    dist = self._surfaceDistance(face.geometry, midW)
                    faces.append((face, dist, aW, bW))
            # Order by ascending distance (None last).
            faces.sort(key=lambda fd: (fd[1] is None, fd[1] if fd[1] is not None else 0.0))
            return faces

        def tryCut(targetBody, edgePts, cutLabel):
            candidates = coneFacesByMidpoint(edgePts)
            for (face, dist, _aw, _bw) in candidates:
                try:
                    splitInput = splits.createInput(targetBody, face, True)
                    result = splits.add(splitInput)
                    pieces = result.bodies.count
                    if pieces > 1:
                        futil.log(
                            f'{gearName} {cutLabel}: split into {pieces} pieces '
                            f'(selected cone dist={dist})', force_console=True)
                        return [result.bodies.item(i) for i in range(pieces)]
                    # Did not split into >1; this face is wrong, but the feature
                    # still "succeeded" producing 1 body — keep trying others.
                except RuntimeError as e:
                    msg = str(e)
                    if ('SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg) or ('交差' in msg):
                        continue
                    continue
            return None  # caller decides if this is fatal

        # --- Cut #1: toe edge cone on the Tooth Body. Must split (>=2).
        toePieces = tryCut(toothBody, toeEdge, 'toe cut')
        if toePieces is None or len(toePieces) < 2:
            distsInfo = []
            (aW, bW) = (self._pointWorldGeometry(toeEdge[0]),
                        self._pointWorldGeometry(toeEdge[1]))
            for face in gearBody.faces:
                if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                    dA = self._surfaceDistance(face.geometry, aW)
                    dB = self._surfaceDistance(face.geometry, bW)
                    distsInfo.append((dA, dB))
            raise Exception(
                f'{gearName}: toe cut produced '
                f'{0 if toePieces is None else len(toePieces)} pieces '
                f'(frustum cone faces (dA,dB)={distsInfo}); expected >=2 (apex tip + keeper)')

        # --- Keeper selection AFTER toe cut: drop apex tip, keep largest non-apex.
        keeper = self._selectKeeper(removes, toePieces, apexWorld, gearName, 'after toe cut')

        # --- Cut #2: heel edge cone on the keeper ALONE. May not intersect.
        heelPieces = tryCut(keeper, heelEdge, 'heel cut')
        if heelPieces is None:
            # Heel cone did not split the keeper (no overshoot) — keep it whole.
            futil.log(
                f'{gearName} heel cut: tool did not intersect, kept keeper intact',
                force_console=True)
            return keeper

        # Keeper selection after heel cut: keep largest non-apex.
        return self._selectKeeper(removes, heelPieces, apexWorld, gearName, 'after heel cut')

    def _selectKeeper(self, removes, pieces, apexWorld, gearName, phase):
        nonApex = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            isApex = (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                      or containment == adsk.fusion.PointContainment.PointOnPointContainment)
            if isApex:
                removes.add(body)
            else:
                nonApex.append(body)

        if len(nonApex) == 0:
            raise Exception(
                f'{gearName}: no non-apex piece remained ({phase}); '
                f'{len(pieces)} pieces in, all contained the apex')

        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for body in nonApex[1:]:
            removes.add(body)
        return keeper

    def _cutBore(self, designComponent, gearBody, shaftEdge, bore_cm, gearName):
        # Bore plane normal to the shaft at its start.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{gearName} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearName} Bore Sketch'
        boreSketch.isVisible = True

        circles = boreSketch.sketchCurves.sketchCircles
        boreCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore_cm / 2)
        boreCircle.isConstruction = False
        # Fully constrain: fix center + diameter dimension.
        boreCircle.centerSketchPoint.isFixed = True
        boreSketch.sketchDimensions.addDiameterDimension(
            boreCircle, adsk.core.Point3D.create(bore_cm / 2, bore_cm / 2, 0))

        self._assertFullyConstrained(boreSketch)

        boreProfile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(max(1000.0, bore_cm * 100)), False)
        extrudeInput.participantBodies = [gearBody]
        extrudes.add(extrudeInput)
        boreSketch.isVisible = False


def anchorLineProjection(sketch, anchorLine):
    """Project the anchor line into `sketch` so an in-sketch curve is available
    to constrain against (the §2 perpendicular reference)."""
    projected = sketch.project(anchorLine)
    return projected.item(0)
