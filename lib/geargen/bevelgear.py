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

# Bevel hardcodes a 20-degree pressure angle (not a dialog input).
PRESSURE_ANGLE_RAD = math.radians(20)
INVOLUTE_STEPS = 15


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

        # 4. Module (unitless)
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle (degrees; default 90)
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

        # 10. Enable Bore (checkbox, default True)
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
# Virtual spur proxy — lets the borrowed spur tooth generator run without
# registering real Fusion user parameters.
# =============================================================================
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        # module_mm is the raw mm value; circle radii/diameters served must be
        # in internal cm (what the spur tooth generator expects).
        module = module_mm
        teeth = virtualTeeth
        alpha = PRESSURE_ANGLE_RAD

        pitchDiameter_mm = teeth * module
        baseDiameter_mm = pitchDiameter_mm * math.cos(alpha)
        rootDiameter_mm = pitchDiameter_mm - 2.5 * module
        tipDiameter_mm = pitchDiameter_mm + 2 * module

        self._values = {
            'Module': _Val(module),
            'ToothNumber': _Val(teeth),
            'PressureAngle': _Val(alpha),
            'PitchCircleDiameter': _Val(to_cm(pitchDiameter_mm)),
            'PitchCircleRadius': _Val(to_cm(pitchDiameter_mm / 2)),
            'BaseCircleDiameter': _Val(to_cm(baseDiameter_mm)),
            'BaseCircleRadius': _Val(to_cm(baseDiameter_mm / 2)),
            'RootCircleDiameter': _Val(to_cm(rootDiameter_mm)),
            'RootCircleRadius': _Val(to_cm(rootDiameter_mm / 2)),
            'TipCircleDiameter': _Val(to_cm(tipDiameter_mm)),
            'TipCircleRadius': _Val(to_cm(tipDiameter_mm / 2)),
            'InvoluteSteps': _Val(INVOLUTE_STEPS),
        }
        # The spur generator records last-tooth-embedded state on its parent.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return self._values[name]


# =============================================================================
# Bevel gear generator (standalone — does NOT subclass base.Generator)
# =============================================================================
class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # --- small vector helpers (2-D, sketch-local) ----------------------------
    @staticmethod
    def _dot2(a, b):
        return a[0] * b[0] + a[1] * b[1]

    @staticmethod
    def _normalize2(v):
        n = math.hypot(v[0], v[1])
        if n == 0:
            return (0.0, 0.0)
        return (v[0] / n, v[1] / n)

    @staticmethod
    def _pointWorldGeometry(p):
        # World Point3D for either a SketchPoint (.worldGeometry) or a
        # ConstructionPoint (.geometry).
        otyp = p.objectType
        if otyp == adsk.fusion.SketchPoint.classType():
            return p.worldGeometry
        return p.geometry

    # --- input reading -------------------------------------------------------
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        # Selections first (no occurrence is created yet, but keep the order).
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

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Module: read unitless -> raw number meaning millimetres.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft angle: evaluateExpression(..., 'deg') returns RADIANS internally.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))

        # 'mm' inputs come back already in internal cm.
        drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')

        # --- validation ---
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.2f})')
        if drivingBaseHeight_cm < 0:
            raise Exception('Driving Gear Base Height must be non-negative')
        if pinionBaseHeight_cm < 0:
            raise Exception('Pinion Gear Base Height must be non-negative')
        if drivingBore_cm < 0:
            raise Exception('Driving Gear Bore Diameter must be non-negative')
        if pinionBore_cm < 0:
            raise Exception('Pinion Gear Bore Diameter must be non-negative')
        if faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')

        # Stash the rest on self.
        self._drivingBaseHeight_cm = drivingBaseHeight_cm
        self._pinionBaseHeight_cm = pinionBaseHeight_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBore_cm
        self._pinionBore_cm = pinionBore_cm
        self._faceWidth_cm = faceWidth_cm

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # --- full-constraint gate ------------------------------------------------
    @staticmethod
    def _assertFullyConstrained(sketch):
        if not sketch.isFullyConstrained:
            raise RuntimeError(f"sketch '{sketch.name}' is not fully constrained")

    # --- orchestration -------------------------------------------------------
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Pitch diameters (mm), cone distance (mm).
        drivingPitchDiameter_mm = module * drivingTeeth
        pinionPitchDiameter_mm = module * pinionTeeth
        coneDistance_mm = math.sqrt(
            (module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2)

        # cm versions used in geometry
        drivingPitchDiameter_cm = to_cm(drivingPitchDiameter_mm)
        pinionPitchDiameter_cm = to_cm(pinionPitchDiameter_mm)
        coneDistance_cm = to_cm(coneDistance_mm)

        # Resolve bore diameters (cm); 0 => auto = pitch diameter / 4.
        drivingBore_cm = self._drivingBore_cm
        if drivingBore_cm <= 0:
            drivingBore_cm = drivingPitchDiameter_cm / 4
        pinionBore_cm = self._pinionBore_cm
        if pinionBore_cm <= 0:
            pinionBore_cm = pinionPitchDiameter_cm / 4

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

        futil.log('BevelGear: built component tree', force_console=True)

        # §1 Anchor sketch
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear bodies
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm, coneDistance_cm,
            drivingPitchDiameter_mm, pinionPitchDiameter_mm,
            drivingBore_cm, pinionBore_cm)

        # Cleanup
        self._hideConstructionGeometry(bevelComponent)

        futil.log('BevelGear: generation complete', force_console=True)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # =========================================================================
    # §1 Anchor Sketch
    # =========================================================================
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user-specified center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        c = projectedCenter.geometry
        # ~10mm reference line through the center.
        halfLen = to_cm(5)
        anchorLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x - halfLen, c.y, 0),
            adsk.core.Point3D.create(c.x + halfLen, c.y, 0))

        # Both: center on the line AND center bisects the line.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length dimension (arbitrary reference value).
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + halfLen, 0)).parameter.value = to_cm(10)

        # Pin direction with a sketch-local Horizontal constraint.
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch)
        return anchorLine

    # =========================================================================
    # §2 + §3 + per-gear bodies
    # =========================================================================
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane,
                           anchorLine, module, drivingTeeth, pinionTeeth,
                           shaftAngle_deg, drivingPitchDiameter_cm,
                           pinionPitchDiameter_cm, coneDistance_cm,
                           drivingPitchDiameter_mm, pinionPitchDiameter_mm,
                           drivingBore_cm, pinionBore_cm):
        Sigma = math.radians(shaftAngle_deg)
        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm
        module_cm = to_cm(module)
        dedendum_cm = to_cm(1.25 * module)

        # --- Gear Profiles plane: through anchor line, perpendicular to target.
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

        def pt(x, y):
            return adsk.core.Point3D.create(x, y, 0)

        # Project the STASHED anchor-sketch center point.
        projected = sketch.project(self._anchorCenterPoint)
        projectedCenter = projected.item(0)
        c = projectedCenter.geometry

        # Projected anchor line 2-D direction.
        projAnchor = sketch.project(anchorLine)
        projAnchorLine = projAnchor.item(0)
        ap1 = projAnchorLine.startSketchPoint.geometry
        ap2 = projAnchorLine.endSketchPoint.geometry
        d = self._normalize2((ap2.x - ap1.x, ap2.y - ap1.y))
        perp = (-d[1], d[0])

        # Grow-side sign from the target normal (one-bit direction only).
        nWorld = get_normal(targetPlane)
        pWorld = projectedCenter.worldGeometry
        o2 = sketch.modelToSketchSpace(pWorld)
        n2 = sketch.modelToSketchSpace(adsk.core.Point3D.create(
            pWorld.x + nWorld.x, pWorld.y + nWorld.y, pWorld.z + nWorld.z))
        nDir2D = (n2.x - o2.x, n2.y - o2.y)
        if self._dot2(perp, nDir2D) <= 0:
            perp = (-perp[0], -perp[1])

        # --- Apex: free end of centerToApex (sketch-local c + perp*DPD).
        apexLocal = pt(c.x + perp[0] * DPD, c.y + perp[1] * DPD)
        centerToApex = lines.addByTwoPoints(pt(c.x, c.y), apexLocal)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projAnchorLine)
        apex = centerToApex.endSketchPoint
        apexSketchPoint = centerToApex.endSketchPoint

        # --- closed-form cone geometry to seed Apex->A / Apex->B lengths.
        # tan gamma_p = sin Sigma * PPD / (DPD + PPD * cos Sigma)
        gamma_p = math.atan2(math.sin(Sigma) * PPD,
                             DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        R = (PPD / 2) / math.sin(gamma_p)
        lenA = R * math.cos(gamma_p)   # |Apex->A| (pinion shaft)
        lenB = R * math.cos(gamma_g)   # |Apex->B| (driving shaft)

        # In-plane "toward anchor" unit (= -perp) and the along-shaft seeds.
        # Driving Shaft Axis: from apex back toward anchor line (-perp dir),
        # parallel to centerToApex. End = point B.
        bSeed = pt(apexLocal.x - perp[0] * lenB, apexLocal.y - perp[1] * lenB)
        drivingShaftAxis = lines.addByTwoPoints(pt(apexLocal.x, apexLocal.y), bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Shaft Axis: end = point A, on +X half-plane side away from
        # the anchor's leading direction; angular dim Sigma against driving.
        # Seed by rotating -perp by gamma toward +d (so A lies between shafts).
        # Direction from apex along pinion shaft: rotate (-perp) by -? Use the
        # closed-form: pinion shaft makes angle Sigma with driving shaft.
        # Seed A at apex + (rotate of -perp by +gamma combination); simplest
        # robust seed: place along (cos/sin) in the (-perp, +d) basis.
        # Driving shaft direction = -perp ; pinion shaft = rotate -perp by Sigma
        # toward +d side.
        u = (-perp[0], -perp[1])              # driving shaft direction
        # rotate u by Sigma in the plane, choosing the +d side
        # build orthonormal-ish basis using d as the "across" axis
        # pinion direction = cos(Sigma)*u + sin(Sigma)*dHat where dHat chosen on
        # the side the pinion shaft is drawn (+X half-plane / +d).
        dHat = d
        # ensure dHat points to the "+X half-plane" side (away from leading dir):
        pinDir = (math.cos(Sigma) * u[0] + math.sin(Sigma) * dHat[0],
                  math.cos(Sigma) * u[1] + math.sin(Sigma) * dHat[1])
        aSeed = pt(apexLocal.x + pinDir[0] * lenA, apexLocal.y + pinDir[1] * lenA)
        pinionShaftAxis = lines.addByTwoPoints(pt(apexLocal.x, apexLocal.y), aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        pointA = pinionShaftAxis.endSketchPoint

        # Angular dimension Sigma between the two shaft axes, text inside wedge.
        wedgeMid = (u[0] + pinDir[0], u[1] + pinDir[1])
        wedgeMid = self._normalize2(wedgeMid)
        wedgeText = pt(apexLocal.x + wedgeMid[0] * lenA * 0.4,
                       apexLocal.y + wedgeMid[1] * lenA * 0.4)
        dims.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, wedgeText).parameter.value = Sigma

        # --- From A: perpendicular drop of length PPD/2 toward Apex2 side.
        # Direction roughly toward driving shaft (between the two shafts).
        toApex2_p = self._normalize2((-pinDir[1], pinDir[0]))
        # ensure it points toward the anchor/between-shaft side (toward -perp):
        if self._dot2(toApex2_p, u) < 0:
            toApex2_p = (-toApex2_p[0], -toApex2_p[1])
        aDropSeed = pt(aSeed.x + toApex2_p[0] * (PPD / 2),
                       aSeed.y + toApex2_p[1] * (PPD / 2))
        lineA_Apex2 = lines.addByTwoPoints(pt(aSeed.x, aSeed.y), aDropSeed)
        lineA_Apex2.isConstruction = True
        constraints.addCoincident(lineA_Apex2.startSketchPoint, pointA)
        constraints.addPerpendicular(lineA_Apex2, pinionShaftAxis)
        dims.addDistanceDimension(
            lineA_Apex2.startSketchPoint, lineA_Apex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            aDropSeed).parameter.value = PPD / 2

        # --- From B: perpendicular drop of length DPD/2 toward Apex2 side.
        toApex2_g = self._normalize2((-u[1], u[0]))
        if self._dot2(toApex2_g, pinDir) < 0:
            toApex2_g = (-toApex2_g[0], -toApex2_g[1])
        bDropSeed = pt(bSeed.x + toApex2_g[0] * (DPD / 2),
                       bSeed.y + toApex2_g[1] * (DPD / 2))
        lineB_Apex2 = lines.addByTwoPoints(pt(bSeed.x, bSeed.y), bDropSeed)
        lineB_Apex2.isConstruction = True
        constraints.addCoincident(lineB_Apex2.startSketchPoint, pointB)
        constraints.addPerpendicular(lineB_Apex2, drivingShaftAxis)
        dims.addDistanceDimension(
            lineB_Apex2.startSketchPoint, lineB_Apex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            bDropSeed).parameter.value = DPD / 2

        # --- Apex2 = coincident of the two drop ends.
        constraints.addCoincident(
            lineA_Apex2.endSketchPoint, lineB_Apex2.endSketchPoint)
        apex2 = lineA_Apex2.endSketchPoint

        # --- Pitch Line Apex -> Apex2.
        pitchLine = lines.addByTwoPoints(apexLocal, apex2.geometry)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # --- Dedendum lines from Apex2, perpendicular to Pitch Line, len 1.25m.
        # Driving Gear Dedendum -> point D (toward anchor line, -perp-ish).
        ap2g = apex2.geometry
        # perpendicular to pitch line direction
        plDir = self._normalize2((ap2g.x - apexLocal.x, ap2g.y - apexLocal.y))
        ded = (-plDir[1], plDir[0])
        # D toward anchor line (toward -perp); C away.
        dSign = ded
        if self._dot2(dSign, perp) > 0:
            dSign = (-dSign[0], -dSign[1])
        dSeed = pt(ap2g.x + dSign[0] * dedendum_cm, ap2g.y + dSign[1] * dedendum_cm)
        drivingDedendum = lines.addByTwoPoints(pt(ap2g.x, ap2g.y), dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            dSeed).parameter.value = dedendum_cm
        pointD = drivingDedendum.endSketchPoint

        cSign = (-dSign[0], -dSign[1])
        cSeed = pt(ap2g.x + cSign[0] * dedendum_cm, ap2g.y + cSign[1] * dedendum_cm)
        pinionDedendum = lines.addByTwoPoints(pt(ap2g.x, ap2g.y), cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            cSeed).parameter.value = dedendum_cm
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axis lines: Apex -> D (driving), Apex -> C (pinion).
        drivingRootAxis = lines.addByTwoPoints(apexLocal, pointD.geometry)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(apexLocal, pointC.geometry)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- A -> E : colinear with Apex->A, length module (undimensioned).
        ag = pointA.geometry
        aDir = self._normalize2((ag.x - apexLocal.x, ag.y - apexLocal.y))
        eSeed = pt(ag.x + aDir[0] * module_cm, ag.y + aDir[1] * module_cm)
        lineAE = lines.addByTwoPoints(pt(ag.x, ag.y), eSeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        # --- C -> E line, perpendicular to A->E.
        cg = pointC.geometry
        lineCE = lines.addByTwoPoints(pt(cg.x, cg.y), eSeed)
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # --- B -> F : colinear with Apex->B, length module.
        bg = pointB.geometry
        bDir = self._normalize2((bg.x - apexLocal.x, bg.y - apexLocal.y))
        fSeed = pt(bg.x + bDir[0] * module_cm, bg.y + bDir[1] * module_cm)
        lineBF = lines.addByTwoPoints(pt(bg.x, bg.y), fSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        # --- D -> F line, perpendicular to B->F.
        dg = pointD.geometry
        lineDF = lines.addByTwoPoints(pt(dg.x, dg.y), fSeed)
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # --- E -> G : colinear with A->E, length module.
        eg = pointE.geometry
        gSeed = pt(eg.x + aDir[0] * module_cm, eg.y + aDir[1] * module_cm)
        lineEG = lines.addByTwoPoints(pt(eg.x, eg.y), gSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # --- C -> H : length module, colinear with Apex2->C (pinion dedendum).
        chDir = self._normalize2((cg.x - ap2g.x, cg.y - ap2g.y))
        hSeed = pt(cg.x + chDir[0] * module_cm, cg.y + chDir[1] * module_cm)
        lineCH = lines.addByTwoPoints(pt(cg.x, cg.y), hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # --- G -> H line, E->G ⊥ H->G.
        gg = pointG.geometry
        lineGH = lines.addByTwoPoints(gg, pointH.geometry)
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # --- F -> I : colinear with B->F, length module.
        fg = pointF.geometry
        iSeed = pt(fg.x + bDir[0] * module_cm, fg.y + bDir[1] * module_cm)
        lineFI = lines.addByTwoPoints(pt(fg.x, fg.y), iSeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # --- D -> J : length module, colinear with Apex2->D (driving dedendum).
        djDir = self._normalize2((dg.x - ap2g.x, dg.y - ap2g.y))
        jSeed = pt(dg.x + djDir[0] * module_cm, dg.y + djDir[1] * module_cm)
        lineDJ = lines.addByTwoPoints(pt(dg.x, dg.y), jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # --- I -> J line, F->I ⊥ J->I.
        ig = pointI.geometry
        lineIJ = lines.addByTwoPoints(ig, pointJ.geometry)
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- Offset dim between B->Apex2 drop and J->I = driving base height.
        if self._drivingBaseHeight_cm > 0:
            drivingBase = self._drivingBaseHeight_cm
        else:
            drivingBase = to_cm(module * drivingTeeth / 8)
        offTextJI = pt((ig.x + jSeed.x) / 2, (ig.y + jSeed.y) / 2)
        dims.addOffsetDimension(
            lineB_Apex2, lineIJ, offTextJI).parameter.value = drivingBase

        # --- Offset dim between A->Apex2 drop and G->H = pinion base height.
        if self._pinionBaseHeight_cm > 0:
            pinionBase = self._pinionBaseHeight_cm
        else:
            pinionBase = drivingBase * (pinionTeeth / drivingTeeth)
        offTextGH = pt((gg.x + hSeed.x) / 2, (gg.y + hSeed.y) / 2)
        dims.addOffsetDimension(
            lineA_Apex2, lineGH, offTextGH).parameter.value = pinionBase

        # --- A -> G line.
        lineAG = lines.addByTwoPoints(ag, gg)
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # --- Constrain Point I with center point (pins the lattice scale).
        constraints.addCoincident(pointI, projectedCenter)

        # --- K: from G along Apex->A (away from apex), pinned to two lines.
        gg2 = pointG.geometry
        kSeed = pt(gg2.x + aDir[0] * module_cm, gg2.y + aDir[1] * module_cm)
        lineGK = lines.addByTwoPoints(gg2, kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, lineCH)
        lineCK = lines.addByTwoPoints(pointC.geometry, pointK.geometry)
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # --- L: from I along Apex->B, pinned to two lines.
        ig2 = pointI.geometry
        lSeed = pt(ig2.x + bDir[0] * module_cm, ig2.y + bDir[1] * module_cm)
        lineIL = lines.addByTwoPoints(ig2, lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, lineDJ)
        lineDL = lines.addByTwoPoints(pointD.geometry, pointL.geometry)
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # --- Maximum Face Width (from SOLVED geometry) and Face Width resolve.
        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry

        def perpDistance(p, l0, l1):
            vx = l1.x - l0.x
            vy = l1.y - l0.y
            wx = p.x - l0.x
            wy = p.y - l0.y
            denom = math.hypot(vx, vy)
            if denom == 0:
                return float('inf')
            return abs(vx * wy - vy * wx) / denom

        distPinion = perpDistance(gA, gC, gH)   # A to line C->H
        distDriving = perpDistance(gB, gD, gJ)   # B to line D->J
        maxFaceWidth = 0.95 * min(distPinion, distDriving)

        if self._faceWidth_cm > 0:
            faceWidth = self._faceWidth_cm
            if faceWidth > maxFaceWidth:
                raise Exception(
                    'Face Width {:.4f} cm exceeds the maximum allowed {:.4f} cm; '
                    'reduce Face Width.'.format(faceWidth, maxFaceWidth))
        else:
            faceWidth = min(coneDistance_cm / 6, maxFaceWidth)

        # --- Toe line M -> N (pinion side).
        # Seed M near midpoint of Apex->C, N slid along C->H toward A->Apex2.
        apexC_mid = pt((apexLocal.x + gC.x) / 2, (apexLocal.y + gC.y) / 2)
        chDir2 = self._normalize2((gH.x - gC.x, gH.y - gC.y))
        distMtoA = math.hypot(gA.x - apexC_mid.x, gA.y - apexC_mid.y)
        mSeed = apexC_mid
        nSeed = pt(mSeed.x + chDir2[0] * distMtoA, mSeed.y + chDir2[1] * distMtoA)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, lineA_Apex2)
        constraints.addParallel(lineMN, lineCH)
        offTextMN = pt((mSeed.x + nSeed.x) / 2, (mSeed.y + nSeed.y) / 2)
        dims.addOffsetDimension(
            lineCH, lineMN, offTextMN).parameter.value = faceWidth

        lineMC = lines.addByTwoPoints(pointM.geometry, gC)
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(pointN.geometry, gA)
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- Toe line O -> P (driving side).
        apexD_mid = pt((apexLocal.x + gD.x) / 2, (apexLocal.y + gD.y) / 2)
        djDir2 = self._normalize2((gJ.x - gD.x, gJ.y - gD.y))
        distOtoB = math.hypot(gB.x - apexD_mid.x, gB.y - apexD_mid.y)
        oSeed = apexD_mid
        pSeed = pt(oSeed.x + djDir2[0] * distOtoB, oSeed.y + djDir2[1] * distOtoB)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, lineB_Apex2)
        constraints.addParallel(lineOP, lineDJ)
        offTextOP = pt((oSeed.x + pSeed.x) / 2, (oSeed.y + pSeed.y) / 2)
        dims.addOffsetDimension(
            lineDJ, lineOP, offTextOP).parameter.value = faceWidth

        lineOD = lines.addByTwoPoints(pointO.geometry, gD)
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(pointP.geometry, gB)
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(gB, pointI.geometry)
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch)

        # --- §3: virtual spur tooth profiles ---
        # Pinion: virtual pitch radius = (PPD/2)/cos(gamma_p)
        pinionVirtualPitchRadius = (PPD / 2) / math.cos(gamma_p)
        # virtual tooth number uses Module (mm) and radius; keep consistent:
        # radius is cm, module is mm -> use 2*r / module_cm for unit-consistency.
        pinionVirtualTeeth = int(math.floor(2 * pinionVirtualPitchRadius / module_cm))
        (pinionToothProfile, pinionToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, sketch, lineCK, pointK,
            module, pinionVirtualTeeth, 'Pinion')

        drivingVirtualPitchRadius = (DPD / 2) / math.cos(gamma_g)
        drivingVirtualTeeth = int(math.floor(2 * drivingVirtualPitchRadius / module_cm))
        (drivingToothProfile, drivingToothAxis) = self._buildVirtualSpurProfile(
            designComponent, gearProfilesPlane, sketch, lineDL, pointL,
            module, drivingVirtualTeeth, 'Driving')

        apexWorld = apexSketchPoint.worldGeometry

        # --- Pinion gear body (built first; no mesh offset) ---
        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            pinionToothProfile, apexSketchPoint, apexWorld,
            (pointM, pointN), (pointC, pointH),
            pinionTeeth, pinionBore_cm, 'Pinion', isDriving=False)

        # --- Driving gear body (built second; mesh offset 180/N) ---
        self._createGearBody(
            bevelComponent, designComponent, gearProfilesPlane,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            drivingToothProfile, apexSketchPoint, apexWorld,
            (pointO, pointP), (pointD, pointJ),
            drivingTeeth, drivingBore_cm, 'Driving', isDriving=True)

    # =========================================================================
    # §3 helper: virtual spur tooth profile + normal axis
    # =========================================================================
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesPlane,
                                 gearProfilesSketch, lineToK, pointK,
                                 module, virtualTeeth, label):
        # Tooth plane: include line C->K (resp D->L), perpendicular to profiles.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            lineToK, adsk.core.ValueInput.createByString('90 deg'), gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = f'{label} Tooth Plane'

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{label} Tooth Profile'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(pointK, angle=math.radians(180))

        # Construction axis through K normal to tooth plane via setByTwoPlanes:
        # gear profiles plane ∩ helper plane perpendicular to C->K at K.
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(lineToK, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = f'{label} Tooth Normal Helper'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Tooth Axis'

        return (toothSketch, toothAxis)

    # =========================================================================
    # Tooth profile loop finder (by curve-type mix)
    # =========================================================================
    @staticmethod
    def _findSpurToothProfile(sketch):
        line = adsk.core.Curve3DTypes.Line3DCurveType
        arc = adsk.core.Curve3DTypes.Arc3DCurveType
        nurbs = adsk.core.Curve3DTypes.NurbsCurve3DCurveType

        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                curves = loop.profileCurves
                nNurbs = 0
                nArc = 0
                nLine = 0
                other = 0
                for curve in curves:
                    t = curve.geometry.curveType
                    if t == nurbs:
                        nNurbs += 1
                    elif t == arc:
                        nArc += 1
                    elif t == line:
                        nLine += 1
                    else:
                        other += 1
                if other != 0:
                    continue
                # non-embedded: 2 nurbs + 2 arcs + 2 lines; embedded: + 0 lines.
                if nNurbs == 2 and nArc == 2 and (nLine == 2 or nLine == 0):
                    return profile
        raise Exception('Could not find spur tooth profile loop')

    # =========================================================================
    # Cone-face helpers
    # =========================================================================
    @staticmethod
    def _surfaceDistance(surface, worldPoint):
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

    def _orderedConeFaces(self, frustumBody, edgeMidWorld):
        coneFaces = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                dist = self._surfaceDistance(face.geometry, edgeMidWorld)
                coneFaces.append((face, dist))

        def sortKey(item):
            d = item[1]
            return float('inf') if d is None else d

        coneFaces.sort(key=sortKey)
        return coneFaces

    def _applyConicalCut(self, designComponent, pieces, frustumBody,
                         edgeMidWorld, label):
        # Try each cone face best-first as a split tool; collect resulting
        # pieces. Catches non-intersection per piece. Returns (newPieces, info).
        coneFaces = self._orderedConeFaces(frustumBody, edgeMidWorld)
        if len(coneFaces) == 0:
            raise Exception(
                '{}: no ConeSurfaceType faces found on frustum (distances: [])'.format(label))

        splitFeatures = designComponent.features.splitBodyFeatures
        resultPieces = []
        anySplit = False
        distInfo = [d for (_, d) in coneFaces]

        for piece in pieces:
            splitThisPiece = False
            for (face, dist) in coneFaces:
                try:
                    splitInput = splitFeatures.createInput(piece, face, True)
                    splitResult = splitFeatures.add(splitInput)
                    if splitResult.bodies.count > 1:
                        for i in range(splitResult.bodies.count):
                            resultPieces.append(splitResult.bodies.item(i))
                        splitThisPiece = True
                        anySplit = True
                        futil.log(
                            '{}: split piece into {} (selected dist={})'.format(
                                label, splitResult.bodies.count, dist),
                            force_console=True)
                        break
                except RuntimeError as e:
                    msg = str(e)
                    if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                        continue
                    # other cone faces may still work; keep trying
                    continue
            if not splitThisPiece:
                # keep this piece intact
                resultPieces.append(piece)
                futil.log(
                    '{}: tool did not intersect, kept piece intact'.format(label),
                    force_console=True)

        return (resultPieces, anySplit, distInfo)

    # =========================================================================
    # Per-gear body creation
    # =========================================================================
    def _createGearBody(self, bevelComponent, designComponent, gearProfilesPlane,
                        hexVertices, toothSketch, apexSketchPoint, apexWorld,
                        toeEdgePoints, heelEdgePoints, teeth, bore_cm,
                        label, isDriving):
        # Component for the finished bodies (child of Bevel Gear).
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{label} Gear'

        # --- per-gear Profile sketch (one hexagon loop) ---
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{label} Gear Profile Sketch'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        # 1. recreate each §2 vertex as a NEW SketchPoint (sketch-space).
        verts = [profileSketch.sketchPoints.add(
                    profileSketch.modelToSketchSpace(self._pointWorldGeometry(p)))
                 for p in hexVertices]
        # 2. draw the closed hexagon SHARING those points.
        hexLines = [lines.addByTwoPoints(verts[i], verts[(i + 1) % len(verts)])
                    for i in range(len(verts))]
        # 3. NOW fix every line AND its endpoints (after the topology exists).
        for edge in hexLines:
            edge.isFixed = True
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch)

        # The first edge (A->G / B->I) is the shaft axis for all body ops.
        shaftEdge = hexLines[0]

        # --- profile loop (the hexagon) ---
        bodyProfile = None
        for profile in profileSketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == len(hexVertices):
                    bodyProfile = profile
                    break
            if bodyProfile is not None:
                break
        if bodyProfile is None:
            raise Exception(f'{label}: could not find hexagon profile to revolve')

        # --- Revolve around the shaft (A->G / B->I) edge ---
        revolves = designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            bodyProfile, shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveResult = revolves.add(revolveInput)
        gearBody = revolveResult.bodies.item(0)
        gearBody.name = f'{label} Gear Body'

        # --- Loft: apex sketch point -> tooth profile ---
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftResult = lofts.add(loftInput)
        toothBody = loftResult.bodies.item(0)
        toothBody.name = f'{label} Gear Tooth Body'

        # --- Two conical cuts (cut tool from the frustum; target = tooth body) ---
        (toeM, toeN) = toeEdgePoints
        toeMid = self._midpointWorld(toeM, toeN)
        (cut1Pieces, cut1Split, cut1Dists) = self._applyConicalCut(
            designComponent, [toothBody], gearBody, toeMid, f'{label} cut#1 toe')
        if not cut1Split:
            raise Exception(
                '{} cut#1 toe: no cone face split the tooth body (cone face dists={})'.format(
                    label, cut1Dists))

        (heelC, heelH) = heelEdgePoints
        heelMid = self._midpointWorld(heelC, heelH)
        (cut2Pieces, cut2Split, cut2Dists) = self._applyConicalCut(
            designComponent, cut1Pieces, gearBody, heelMid, f'{label} cut#2 heel')

        pieces = cut2Pieces
        if len(pieces) < 2:
            raise Exception(
                '{}: expected >=2 pieces, got {} '
                '(cut#1: dists={}, split={}; cut#2: dists={}, split={})'.format(
                    label, len(pieces), cut1Dists, cut1Split, cut2Dists, cut2Split))

        # --- Select the tooth robustly ---
        toothPiece = self._selectToothPiece(designComponent, pieces, apexWorld, label)

        # --- Circular pattern around the shaft edge ---
        patternBodies = adsk.core.ObjectCollection.create()
        patternBodies.add(toothPiece)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(patternBodies, shaftEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)

        # --- Combine-join the patterned teeth into the gear body ---
        tools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            tools.add(patternResult.bodies.item(i))
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ---
        if self._boreEnable:
            self._cutBore(designComponent, shaftEdge, gearBody, bore_cm, label)

        # --- Meshing rotation (driving only), before moveToComponent ---
        if isDriving:
            self._applyMeshRotation(designComponent, shaftEdge, gearBody, teeth)

        # --- Relocate the finished body into the gear component ---
        gearBody.moveToComponent(gearOccurrence)

    @staticmethod
    def _midpointWorld(pointA, pointB):
        a = pointA.worldGeometry
        b = pointB.worldGeometry
        return adsk.core.Point3D.create(
            (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)

    def _selectToothPiece(self, designComponent, pieces, apexWorld, label):
        removeFeatures = designComponent.features.removeFeatures
        inside = adsk.fusion.PointContainment.PointInsidePointContainment
        onPoint = adsk.fusion.PointContainment.PointOnPointContainment

        nonApex = []
        apexPieces = []
        for piece in pieces:
            containment = piece.pointContainment(apexWorld)
            if containment == inside or containment == onPoint:
                apexPieces.append(piece)
            else:
                nonApex.append(piece)

        if len(nonApex) == 0:
            raise Exception(
                '{}: no non-apex piece remained after cuts (pieces={})'.format(
                    label, len(pieces)))

        # Keep the largest non-apex piece; remove the rest + apex pieces.
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        toothPiece = nonApex[0]

        for piece in apexPieces:
            removeFeatures.add(piece)
        for piece in nonApex[1:]:
            removeFeatures.add(piece)

        return toothPiece

    def _cutBore(self, designComponent, shaftEdge, gearBody, bore_cm, label):
        # Bore plane normal to shaft at its start.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{label} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{label} Bore Sketch'
        boreSketch.isVisible = True

        radius = bore_cm / 2
        circles = boreSketch.sketchCurves.sketchCircles
        boreCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)

        # Fully constrain: FIX the center (2 DOF) + diameter dimension (1 DOF).
        # Do NOT addCoincident(centerSketchPoint, originPoint) (VCS_SKETCH_SOLVING_FAILED).
        boreCircle.centerSketchPoint.isFixed = True
        boreSketch.sketchDimensions.addDiameterDimension(
            boreCircle,
            adsk.core.Point3D.create(radius, radius, 0)).parameter.value = bore_cm

        self._assertFullyConstrained(boreSketch)

        boreProfile = None
        for profile in boreSketch.profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 1:
                    boreProfile = profile
                    break
            if boreProfile is not None:
                break
        if boreProfile is None:
            raise Exception(f'{label}: could not find bore profile')

        extrudes = designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        largeDist = max(1000.0, bore_cm * 100)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(largeDist), False)
        extrudeInput.participantBodies = [gearBody]
        extrudeResult = extrudes.add(extrudeInput)
        extrudeResult.name = f'{label} Bore cut'

    def _applyMeshRotation(self, designComponent, shaftEdge, gearBody, teeth):
        startWorld = shaftEdge.startSketchPoint.worldGeometry
        endWorld = shaftEdge.endSketchPoint.worldGeometry
        axisVector = startWorld.vectorTo(endWorld)
        axisVector.normalize()

        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(math.radians(180 / teeth), axisVector, startWorld)

        bodyCollection = adsk.core.ObjectCollection.create()
        bodyCollection.add(gearBody)
        moves = designComponent.features.moveFeatures
        moveInput = moves.createInput2(bodyCollection)
        moveInput.defineAsFreeMove(matrix)
        moves.add(moveInput)

    # =========================================================================
    # Cleanup
    # =========================================================================
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                if sketch.entityToken not in seen:
                    seen.add(sketch.entityToken)
                    sketch.isLightBulbOn = False
                    sketch.isVisible = False
            for plane in component.constructionPlanes:
                if plane.entityToken not in seen:
                    seen.add(plane.entityToken)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                if axis.entityToken not in seen:
                    seen.add(axis.entityToken)
                    axis.isLightBulbOn = False

            for occurrence in component.occurrences:
                walk(occurrence.component)

        walk(bevelComponent)
