import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the only module-level name strings bevel reproduces)
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
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'

# Hand-of-spiral list-item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Tunable spiral constants
_CROWN_PER_RAD = 0.5
_PINION_MESH_PHASE_TEETH = 0

# Spur-tooth-generator parameter keys (must match spurgear's PARAM_* names)
_VS_MODULE = 'Module'
_VS_TOOTH_NUMBER = 'ToothNumber'
_VS_PRESSURE_ANGLE = 'PressureAngle'
_VS_PITCH_DIAMETER = 'PitchCircleDiameter'
_VS_PITCH_RADIUS = 'PitchCircleRadius'
_VS_BASE_DIAMETER = 'BaseCircleDiameter'
_VS_BASE_RADIUS = 'BaseCircleRadius'
_VS_ROOT_DIAMETER = 'RootCircleDiameter'
_VS_ROOT_RADIUS = 'RootCircleRadius'
_VS_TIP_DIAMETER = 'TipCircleDiameter'
_VS_TIP_RADIUS = 'TipCircleRadius'
_VS_INVOLUTE_STEPS = 'InvoluteSteps'


# ---------------------------------------------------------------------------
# Virtual-spur proxy: a fake `parent` so the borrowed spur tooth generator can
# run without registering Fusion user parameters.
# ---------------------------------------------------------------------------
class _Val:
    """Lightweight value wrapper exposing `.value` like a UserParameter."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """Precomputes (in internal cm) exactly the keys the spur drawer reads,
    each wrapped in `_Val`. Also carries `_lastToothEmbedded`, an OUTPUT the
    spur generator writes during draw() that bevel reads back."""

    def __init__(self, module_mm, virtualTeeth):
        # _lastToothEmbedded is written by the spur generator during draw();
        # declare it up front to absorb that write.
        self._lastToothEmbedded = False

        pressureAngle = math.radians(20)   # bevel hardcodes 20 deg

        # Standard spur formulas. Module arrives in mm; circle radii/diameters
        # served back must be in internal cm (matching what spur expects).
        pitch = module_mm * virtualTeeth                  # mm
        base = pitch * math.cos(pressureAngle)            # mm
        root = pitch - 2.5 * module_mm                    # mm
        tip = pitch + 2.0 * module_mm                     # mm

        self._params = {
            _VS_MODULE: _Val(module_mm),
            _VS_TOOTH_NUMBER: _Val(virtualTeeth),
            _VS_PRESSURE_ANGLE: _Val(pressureAngle),
            _VS_PITCH_DIAMETER: _Val(to_cm(pitch)),
            _VS_PITCH_RADIUS: _Val(to_cm(pitch / 2)),
            _VS_BASE_DIAMETER: _Val(to_cm(base)),
            _VS_BASE_RADIUS: _Val(to_cm(base / 2)),
            _VS_ROOT_DIAMETER: _Val(to_cm(root)),
            _VS_ROOT_RADIUS: _Val(to_cm(root / 2)),
            _VS_TIP_DIAMETER: _Val(to_cm(tip)),
            _VS_TIP_RADIUS: _Val(to_cm(tip / 2)),
            _VS_INVOLUTE_STEPS: _Val(15),
        }

    def getParameter(self, name):
        return self._params[name]


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so it wins Fusion's auto-focus
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

        # 3. Parent Component (selection) -- root pre-selected
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the parent component for the new bevel gear pair')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module (unitless)
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle (deg)
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

        # 14. Tooth Spacing
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 15. Mean Spiral Angle
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral (text-list dropdown)
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # Last step: set initial spiral-only visibility (default psi=35 -> shown)
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args):
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            value = get_design().unitsManager.evaluateExpression(
                spiralInput.expression, 'deg')   # internal radians
        except Exception:
            # half-typed expression mid-edit; leave both shown
            handInput.isVisible = True
            cutterInput.isVisible = True
            return
        show = value > 0
        handInput.isVisible = show
        cutterInput.isVisible = show


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # =======================================================================
    # Top-level orchestration
    # =======================================================================
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        Sigma = math.radians(shaftAngle_deg)

        # Pitch diameters & cone distance (Python, cm). Module is raw mm.
        drivingPitchDia = to_cm(module * drivingTeeth)
        pinionPitchDia = to_cm(module * pinionTeeth)
        coneDistance = to_cm(math.sqrt(
            (module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # Resolve bores (cm). 0 means auto-calculate (pitch dia / 4).
        drivingBore = self._drivingBore_cm if self._drivingBore_cm > 0 else drivingPitchDia / 4.0
        pinionBore = self._pinionBore_cm if self._pinionBore_cm > 0 else pinionPitchDia / 4.0

        # Pitch-cone half-angles (driving = gear, pinion).
        # tan gamma_p = sin S * PPD / (DPD + PPD * cos S)
        gamma_p = math.atan2(
            math.sin(Sigma) * pinionPitchDia,
            drivingPitchDia + pinionPitchDia * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        # ---- build component tree --------------------------------------
        bevelOcc = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOcc.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOcc
        bevelComponent = bevelOcc.component

        designOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        designComponent = designOcc.component
        self._designOccurrence = designOcc

        # ---- §1 Anchor sketch ------------------------------------------
        # Use the user-selected target plane directly (do NOT re-derive it).
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # ---- §2 + §3 + per-gear bodies ---------------------------------
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, Sigma,
            drivingPitchDia, pinionPitchDia, coneDistance,
            gamma_p, gamma_g, drivingBore, pinionBore)

        # ---- Cleanup ---------------------------------------------------
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # =======================================================================
    # Input reading / validation
    # =======================================================================
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def raw(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        (parentSel, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentSel) != 1:
            raise Exception('Select exactly one parent component')
        parentEntity = parentSel[0]
        if isinstance(parentEntity, adsk.fusion.Occurrence):
            parentComponent = parentEntity.component
        elif hasattr(parentEntity, 'component'):
            parentComponent = parentEntity.component
        else:
            parentComponent = parentEntity

        (planeSel, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeSel) != 1:
            raise Exception('Select exactly one target plane')
        targetPlane = planeSel[0]

        (centerSel, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centerSel) != 1:
            raise Exception('Select exactly one center point')
        centerPoint = centerSel[0]

        # Module: unitless -> raw number meaning mm
        module = raw(INPUT_ID_MODULE, '')

        # Shaft Angle: read back in radians; convert to degrees for range check
        shaftAngle_rad = raw(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.3f})')

        drivingTeeth = int(round(raw(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(raw(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Tooth counts must be at least 3')

        # 'mm' inputs come back already in internal cm; use as-is.
        self._drivingBaseHeight_cm = raw(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = raw(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        (self._boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = raw(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = raw(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = raw(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = raw(INPUT_ID_TOOTH_SPACING, 'mm')

        for label, val in (
                ('Driving Gear Base Height', self._drivingBaseHeight_cm),
                ('Pinion Gear Base Height', self._pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', self._drivingBore_cm),
                ('Pinion Gear Bore Diameter', self._pinionBore_cm),
                ('Face Width', self._faceWidth_cm),
                ('Tooth Spacing', self._toothSpacing_cm)):
            if val < 0:
                raise Exception(f'{label} must be non-negative (got {val})')

        # Spiral inputs
        spiralAngle_rad = raw(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be in [0, 60) degrees (got {spiralAngle_deg:.3f})')
        self._spiralAngle_rad = spiralAngle_rad

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        self._spiralHand = handItem.name if handItem is not None else _HAND_RIGHT

        cutterRadius_cm = raw(INPUT_ID_CUTTER_RADIUS, 'mm')
        if cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')
        self._cutterRadius_cm = cutterRadius_cm

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # =======================================================================
    # Small geometry helpers
    # =======================================================================
    def _pointWorldGeometry(self, point):
        """World Point3D for a SketchPoint (.worldGeometry) or ConstructionPoint (.geometry)."""
        if isinstance(point, adsk.fusion.SketchPoint):
            return point.worldGeometry
        return point.geometry

    def _combine(self, base, a, e1, b=0.0, e2=None):
        """World point = base + a*e1 (+ b*e2). Lengths in cm."""
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, angleDeg):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByAngle(line, adsk.core.ValueInput.createByString(f'{angleDeg} deg'), refPlane)
        return comp.constructionPlanes.add(planeInput)

    def _distDim(self, sketch, lineA, lineB, value, textPoint):
        dim = sketch.sketchDimensions.addOffsetDimension(lineA, lineB, textPoint)
        dim.parameter.value = value
        return dim

    def _assertFullyConstrained(self, sketch, label):
        if not sketch.isFullyConstrained:
            raise Exception(f'Sketch "{label}" is not fully constrained')

    @staticmethod
    def _unit2d(dx, dy):
        n = math.hypot(dx, dy)
        if n == 0:
            return (0.0, 0.0)
        return (dx / n, dy / n)

    @staticmethod
    def _circleIntersectNearest(R, Cx, Cy, rc, refx, refy):
        """Two-circle intersection: apex circle (origin, R) & cutter circle
        (centre (Cx,Cy), radius rc). Returns the solution nearest (refx,refy)."""
        d = math.hypot(Cx, Cy)
        if d == 0:
            raise Exception('Cutter circle centred at apex; no intersection')
        a = (R * R - rc * rc + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)
        ux, uy = Cx / d, Cy / d
        # base point along the centre line
        bx, by = a * ux, a * uy
        # perpendicular offsets
        px, py = -uy, ux
        s1 = (bx + h * px, by + h * py)
        s2 = (bx - h * px, by - h * py)
        d1 = math.hypot(s1[0] - refx, s1[1] - refy)
        d2 = math.hypot(s2[0] - refx, s2[1] - refy)
        return s1 if d1 <= d2 else s2

    @staticmethod
    def _lineIntersect2d(p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-12:
            return None
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
        return (px, py)

    @staticmethod
    def _pointLineDistance2d(p, a, b):
        """Perpendicular distance from point p to the line through a and b (2-D)."""
        dx = b.x - a.x
        dy = b.y - a.y
        n = math.hypot(dx, dy)
        if n == 0:
            return math.hypot(p.x - a.x, p.y - a.y)
        return abs((p.x - a.x) * dy - (p.y - a.y) * dx) / n

    # =======================================================================
    # §1 Anchor sketch
    # =======================================================================
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        cg = projectedCenter.geometry

        # A reference line through the projected center, ~10mm long.
        p0 = adsk.core.Point3D.create(cg.x - to_cm(5), cg.y, cg.z)
        p1 = adsk.core.Point3D.create(cg.x + to_cm(5), cg.y, cg.z)
        anchorLine = lines.addByTwoPoints(p0, p1)

        # BOTH coincident (center on line) AND midpoint (center bisects).
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length reference dimension.
        lengthDim = dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + to_cm(2), cg.z))
        lengthDim.parameter.value = to_cm(10)

        # Pin direction sketch-locally (survives a tilted host plane).
        constraints.addHorizontal(anchorLine)

        # Stash the projected-center SketchPoint for §2 re-projection.
        self._anchorCenterPoint = projectedCenter

        self._assertFullyConstrained(sketch, 'Anchor Sketch')
        return anchorLine

    # =======================================================================
    # §2 + §3 + per-gear bodies
    # =======================================================================
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane,
                           anchorLine, module, drivingTeeth, pinionTeeth, Sigma,
                           drivingPitchDia, pinionPitchDia, coneDistance,
                           gamma_p, gamma_g, drivingBore, pinionBore):

        moduleLen = to_cm(module)
        dedendumLen = to_cm(1.25 * module)
        DPD = drivingPitchDia          # cm
        PPD = pinionPitchDia           # cm

        # The Gear Profiles plane: through the anchor line, 90deg off targetPlane.
        gpPlane = self._planeByAngle(designComponent, anchorLine, targetPlane, 90)
        self._gpPlaneRef = gpPlane

        sketch = designComponent.sketches.add(gpPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        def P2(x, y):
            return adsk.core.Point3D.create(x, y, 0)

        # Project the stashed anchor-sketch center point.
        projected = sketch.project(self._anchorCenterPoint)
        projectedCenter = projected.item(0)
        c = projectedCenter.geometry

        # Projected anchor-line direction in this sketch's 2-D frame.
        projAnchor = sketch.project(anchorLine)
        anchorProj = projAnchor.item(0)
        ag = anchorProj.startSketchPoint.geometry
        bg = anchorProj.endSketchPoint.geometry
        dx, dy = self._unit2d(bg.x - ag.x, bg.y - ag.y)

        # In-plane perpendicular; sign chosen toward target-plane normal.
        perpx, perpy = -dy, dx
        targetNormal = get_normal(targetPlane)
        # map perp (sketch-local direction) to world to compare against normal
        originW = sketch.sketchToModelSpace(P2(0, 0))
        perpW = sketch.sketchToModelSpace(P2(perpx, perpy))
        pwx = perpW.x - originW.x
        pwy = perpW.y - originW.y
        pwz = perpW.z - originW.z
        if (pwx * targetNormal.x + pwy * targetNormal.y + pwz * targetNormal.z) < 0:
            perpx, perpy = -perpx, -perpy

        def perpPt(dist):
            return P2(c.x + perpx * dist, c.y + perpy * dist)

        # --- Apex via undimensioned construction line from projected center ---
        apexSeed = perpPt(DPD)
        centerToApex = lines.addByTwoPoints(P2(c.x, c.y), apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, anchorProj)
        apexPoint = centerToApex.endSketchPoint
        apexSketchPoint = apexPoint

        # Closed-form along-shaft seeds so the solver finds the right branch.
        tan_gp = math.sin(Sigma) * PPD / (DPD + PPD * math.cos(Sigma))
        gp = math.atan(tan_gp)
        gg = Sigma - gp
        R = (PPD / 2.0) / math.sin(gp) if math.sin(gp) != 0 else coneDistance
        lenA = R * math.cos(gp)
        lenB = R * math.cos(gg)

        ap = apexSeed

        # --- Driving Gear Shaft Axis (Apex -> B), parallel to centerToApex ---
        bSeed = P2(ap.x - perpx * lenB, ap.y - perpy * lenB)
        drivingShaft = lines.addByTwoPoints(P2(ap.x, ap.y), bSeed)
        drivingShaft.isConstruction = True
        constraints.addCoincident(drivingShaft.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaft, centerToApex)
        pointB = drivingShaft.endSketchPoint

        # --- Pinion Gear Shaft Axis (Apex -> A), angled by Sigma ---
        # Candidate A positions: rotate the driving-shaft direction (-perp)
        # about apex by +/-Sigma; keep the larger-X endpoint.
        baseDirx, baseDiry = -perpx, -perpy
        cosS, sinS = math.cos(Sigma), math.sin(Sigma)

        def rotated(sign):
            rx = baseDirx * cosS - baseDiry * (sign * sinS)
            ry = baseDirx * (sign * sinS) + baseDiry * cosS
            return P2(ap.x + rx * lenA, ap.y + ry * lenA)

        candPlus = rotated(+1)
        candMinus = rotated(-1)
        aSeed = candPlus if candPlus.x >= candMinus.x else candMinus

        pinionShaft = lines.addByTwoPoints(P2(ap.x, ap.y), aSeed)
        pinionShaft.isConstruction = True
        constraints.addCoincident(pinionShaft.startSketchPoint, apexPoint)
        pointA = pinionShaft.endSketchPoint

        # Angular dimension Sigma between the two shaft axes. Place text in the
        # wedge between them so we measure Sigma, not 180-Sigma.
        midDirx = (baseDirx + (aSeed.x - ap.x)) / 2.0
        midDiry = (baseDiry + (aSeed.y - ap.y)) / 2.0
        mlen = math.hypot(midDirx, midDiry) or 1.0
        textPt = P2(ap.x + (midDirx / mlen) * lenA * 0.5,
                    ap.y + (midDiry / mlen) * lenA * 0.5)
        angDim = dimensions.addAngularDimension(drivingShaft, pinionShaft, textPt)
        angDim.parameter.value = Sigma

        # --- A->Apex2: from A, perpendicular to pinion shaft, length PPD/2 ---
        psx, psy = self._unit2d(aSeed.x - ap.x, aSeed.y - ap.y)
        cand1 = (-psy, psx)
        toCenterx, toCentery = c.x - aSeed.x, c.y - aSeed.y
        if cand1[0] * toCenterx + cand1[1] * toCentery >= 0:
            dropx, dropy = cand1
        else:
            dropx, dropy = (psy, -psx)
        apex2Seed = P2(aSeed.x + dropx * (PPD / 2.0), aSeed.y + dropy * (PPD / 2.0))
        aToApex2 = lines.addByTwoPoints(P2(aSeed.x, aSeed.y), apex2Seed)
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaft)
        dimA = dimensions.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P2(apex2Seed.x, apex2Seed.y))
        dimA.parameter.value = PPD / 2.0

        # --- B->Apex2: from B, perpendicular to driving shaft, length DPD/2 ---
        dsx, dsy = self._unit2d(bSeed.x - ap.x, bSeed.y - ap.y)
        cand1 = (-dsy, dsx)
        toCenterx, toCentery = c.x - bSeed.x, c.y - bSeed.y
        if cand1[0] * toCenterx + cand1[1] * toCentery >= 0:
            bdropx, bdropy = cand1
        else:
            bdropx, bdropy = (dsy, -dsx)
        bApex2Seed = P2(bSeed.x + bdropx * (DPD / 2.0), bSeed.y + bdropy * (DPD / 2.0))
        bToApex2 = lines.addByTwoPoints(P2(bSeed.x, bSeed.y), bApex2Seed)
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaft)
        dimB = dimensions.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P2(bApex2Seed.x, bApex2Seed.y))
        dimB.parameter.value = DPD / 2.0

        # Coincide the two drops' ends -> Apex 2
        constraints.addCoincident(aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint

        # --- Pitch Line Apex -> Apex2 ---
        pl = apex2Point.geometry
        pitchLine = lines.addByTwoPoints(P2(ap.x, ap.y), P2(pl.x, pl.y))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # --- Dedendum lines from Apex2, +/- perpendicular to pitch line ---
        plx, ply = self._unit2d(pl.x - ap.x, pl.y - ap.y)
        ded1 = (-ply, plx)
        toAnchorx, toAnchory = c.x - pl.x, c.y - pl.y
        if ded1[0] * toAnchorx + ded1[1] * toAnchory >= 0:
            dDirx, dDiry = ded1            # Driving Dedendum -> toward anchor (point D)
        else:
            dDirx, dDiry = (ply, -plx)
        cDirx, cDiry = -dDirx, -dDiry      # Pinion Dedendum -> away from anchor (point C)

        dSeed = P2(pl.x + dDirx * dedendumLen, pl.y + dDiry * dedendumLen)
        drivingDedendum = lines.addByTwoPoints(P2(pl.x, pl.y), dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dimDed = dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P2(dSeed.x, dSeed.y))
        dimDed.parameter.value = dedendumLen
        pointD = drivingDedendum.endSketchPoint

        cSeed = P2(pl.x + cDirx * dedendumLen, pl.y + cDiry * dedendumLen)
        pinionDedendum = lines.addByTwoPoints(P2(pl.x, pl.y), cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dimDedC = dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P2(cSeed.x, cSeed.y))
        dimDedC.parameter.value = dedendumLen
        pointC = pinionDedendum.endSketchPoint

        # --- Root axes Apex->D and Apex->C ---
        dg = pointD.geometry
        cg2 = pointC.geometry
        drivingRootAxis = lines.addByTwoPoints(P2(ap.x, ap.y), P2(dg.x, dg.y))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(P2(ap.x, ap.y), P2(cg2.x, cg2.y))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- Module-length extensions (named lines; return the line, reuse) ---
        # A->E collinear with Apex->A (pinion shaft), length module (undimensioned)
        aGeom = pointA.geometry
        eSeed = P2(aGeom.x + psx * moduleLen, aGeom.y + psy * moduleLen)
        lineAE = lines.addByTwoPoints(P2(aGeom.x, aGeom.y), eSeed)
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaft)
        pointE = lineAE.endSketchPoint

        # C->E line; A->E perp C->E
        eGeom = pointE.geometry
        lineCE = lines.addByTwoPoints(P2(cg2.x, cg2.y), P2(eGeom.x, eGeom.y))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # B->F collinear with Apex->B (driving shaft), length module
        bGeom = pointB.geometry
        fSeed = P2(bGeom.x + dsx * moduleLen, bGeom.y + dsy * moduleLen)
        lineBF = lines.addByTwoPoints(P2(bGeom.x, bGeom.y), fSeed)
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaft)
        pointF = lineBF.endSketchPoint

        # D->F line; B->F perp D->F
        fGeom = pointF.geometry
        lineDF = lines.addByTwoPoints(P2(dg.x, dg.y), P2(fGeom.x, fGeom.y))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # E->G collinear with A->E, length module
        gSeed = P2(eGeom.x + psx * moduleLen, eGeom.y + psy * moduleLen)
        lineEG = lines.addByTwoPoints(P2(eGeom.x, eGeom.y), gSeed)
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # C->H length module, collinear with Apex2->C (pinion dedendum)
        hSeed = P2(cg2.x + cDirx * moduleLen, cg2.y + cDiry * moduleLen)
        lineCH = lines.addByTwoPoints(P2(cg2.x, cg2.y), hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # G->H line; E->G perp H->G
        gGeom = pointG.geometry
        hGeom = pointH.geometry
        lineGH = lines.addByTwoPoints(P2(gGeom.x, gGeom.y), P2(hGeom.x, hGeom.y))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # F->I collinear with B->F, length module
        iSeed = P2(fGeom.x + dsx * moduleLen, fGeom.y + dsy * moduleLen)
        lineFI = lines.addByTwoPoints(P2(fGeom.x, fGeom.y), iSeed)
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # D->J length module, collinear with Apex2->D (driving dedendum)
        jSeed = P2(dg.x + dDirx * moduleLen, dg.y + dDiry * moduleLen)
        lineDJ = lines.addByTwoPoints(P2(dg.x, dg.y), jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # I->J line; F->I perp J->I
        iGeom = pointI.geometry
        jGeom = pointJ.geometry
        lineIJ = lines.addByTwoPoints(P2(iGeom.x, iGeom.y), P2(jGeom.x, jGeom.y))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- Driving base height: offset between B->Apex2 drop and J->I ---
        drivingBaseH = self._drivingBaseHeight_cm
        if drivingBaseH <= 0:
            drivingBaseH = to_cm(module * drivingTeeth / 8.0)
        self._distDim(sketch, bToApex2, lineIJ, drivingBaseH, P2(jGeom.x, jGeom.y))

        # --- Pinion base height: offset between A->Apex2 drop and G->H ---
        pinionBaseH = self._pinionBaseHeight_cm
        if pinionBaseH <= 0:
            pinionBaseH = drivingBaseH * (pinionTeeth / float(drivingTeeth))
        self._distDim(sketch, aToApex2, lineGH, pinionBaseH, P2(hGeom.x, hGeom.y))

        # --- A->G line ---
        lineAG = lines.addByTwoPoints(P2(aGeom.x, aGeom.y), P2(gGeom.x, gGeom.y))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # --- Constrain Point I with center point ---
        constraints.addCoincident(pointI, projectedCenter)

        # --- Point K: on Apex->A and on pinion dedendum (Apex2->C extended) ---
        kSeed = self._lineIntersect2d(
            (ap.x, ap.y), (aGeom.x, aGeom.y),
            (pl.x, pl.y), (cg2.x, cg2.y))
        if kSeed is None:
            kSeed = (gGeom.x, gGeom.y)
        lineGK = lines.addByTwoPoints(P2(gGeom.x, gGeom.y), P2(kSeed[0], kSeed[1]))
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        constraints.addCoincident(lineGK.endSketchPoint, pinionShaft)
        constraints.addCoincident(lineGK.endSketchPoint, pinionDedendum)
        pointK = lineGK.endSketchPoint

        lineCK = lines.addByTwoPoints(P2(cg2.x, cg2.y), P2(kSeed[0], kSeed[1]))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # --- Tooth-center K' (Tooth Spacing offset) ---
        toothSpacing = self._toothSpacing_cm
        if toothSpacing <= 0:
            pointKp = pointK
            lineCKp = lineCK
        else:
            kg = pointK.geometry
            ckx, cky = self._unit2d(kg.x - cg2.x, kg.y - cg2.y)
            kpSeed = P2(kg.x + ckx * toothSpacing, kg.y + cky * toothSpacing)
            lineKKp = lines.addByTwoPoints(P2(kg.x, kg.y), kpSeed)
            lineKKp.isConstruction = True
            constraints.addCoincident(lineKKp.startSketchPoint, pointK)
            constraints.addCoincident(lineKKp.endSketchPoint, pinionDedendum)
            dimKKp = dimensions.addDistanceDimension(
                lineKKp.startSketchPoint, lineKKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                P2(kpSeed.x, kpSeed.y))
            dimKKp.parameter.value = toothSpacing
            pointKp = lineKKp.endSketchPoint
            kpg = pointKp.geometry
            lineCKp = lines.addByTwoPoints(P2(cg2.x, cg2.y), P2(kpg.x, kpg.y))
            lineCKp.isConstruction = True
            constraints.addCoincident(lineCKp.startSketchPoint, pointC)
            constraints.addCoincident(lineCKp.endSketchPoint, pointKp)

        # --- Maximum Face Width from SOLVED geometry of A,B,C,D,H,J ---
        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distA = self._pointLineDistance2d(gA, gC, gH)   # A to line C-H (pinion ded.)
        distB = self._pointLineDistance2d(gB, gD, gJ)   # B to line D-J (driving ded.)
        maxFaceWidth = 0.95 * min(distA, distB)

        # --- Resolve Face Width with the cap ---
        faceWidth = self._faceWidth_cm
        if faceWidth <= 0:
            faceWidth = min(coneDistance / 6.0, maxFaceWidth)
        else:
            if faceWidth > maxFaceWidth:
                raise Exception(
                    f'Face Width {to_mm(faceWidth):.3f}mm exceeds the maximum '
                    f'{to_mm(maxFaceWidth):.3f}mm for this gear geometry')

        # --- M->N (pinion toe) ---
        mSeed = P2((ap.x + gC.x) / 2.0, (ap.y + gC.y) / 2.0)
        chx, chy = self._unit2d(gH.x - gC.x, gH.y - gC.y)
        distMtoA = math.hypot(gA.x - mSeed.x, gA.y - mSeed.y)
        nSeed = P2(mSeed.x + chx * distMtoA, mSeed.y + chy * distMtoA)
        lineMN = lines.addByTwoPoints(P2(mSeed.x, mSeed.y), P2(nSeed.x, nSeed.y))
        lineMN.isConstruction = True
        constraints.addCoincident(lineMN.startSketchPoint, pinionRootAxis)
        constraints.addCoincident(lineMN.endSketchPoint, aToApex2)
        constraints.addParallel(lineMN, lineCH)
        self._distDim(sketch, lineCH, lineMN, faceWidth, P2(nSeed.x, nSeed.y))
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint

        mGeom = pointM.geometry
        nGeom = pointN.geometry
        lineMC = lines.addByTwoPoints(P2(mGeom.x, mGeom.y), P2(cg2.x, cg2.y))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(P2(nGeom.x, nGeom.y), P2(gA.x, gA.y))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- Point L: on Apex->B and on driving dedendum (Apex2->D extended) ---
        lSeed = self._lineIntersect2d(
            (ap.x, ap.y), (bGeom.x, bGeom.y),
            (pl.x, pl.y), (dg.x, dg.y))
        if lSeed is None:
            lSeed = (iGeom.x, iGeom.y)
        lineIL = lines.addByTwoPoints(P2(iGeom.x, iGeom.y), P2(lSeed[0], lSeed[1]))
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        constraints.addCoincident(lineIL.endSketchPoint, drivingShaft)
        constraints.addCoincident(lineIL.endSketchPoint, drivingDedendum)
        pointL = lineIL.endSketchPoint

        lineDL = lines.addByTwoPoints(P2(dg.x, dg.y), P2(lSeed[0], lSeed[1]))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # --- Tooth-center L' ---
        if toothSpacing <= 0:
            pointLp = pointL
            lineDLp = lineDL
        else:
            lg = pointL.geometry
            dlx, dly = self._unit2d(lg.x - dg.x, lg.y - dg.y)
            lpSeed = P2(lg.x + dlx * toothSpacing, lg.y + dly * toothSpacing)
            lineLLp = lines.addByTwoPoints(P2(lg.x, lg.y), lpSeed)
            lineLLp.isConstruction = True
            constraints.addCoincident(lineLLp.startSketchPoint, pointL)
            constraints.addCoincident(lineLLp.endSketchPoint, drivingDedendum)
            dimLLp = dimensions.addDistanceDimension(
                lineLLp.startSketchPoint, lineLLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                P2(lpSeed.x, lpSeed.y))
            dimLLp.parameter.value = toothSpacing
            pointLp = lineLLp.endSketchPoint
            lpg = pointLp.geometry
            lineDLp = lines.addByTwoPoints(P2(dg.x, dg.y), P2(lpg.x, lpg.y))
            lineDLp.isConstruction = True
            constraints.addCoincident(lineDLp.startSketchPoint, pointD)
            constraints.addCoincident(lineDLp.endSketchPoint, pointLp)

        # --- O->P (driving toe) ---
        oSeed = P2((ap.x + gD.x) / 2.0, (ap.y + gD.y) / 2.0)
        djx, djy = self._unit2d(gJ.x - gD.x, gJ.y - gD.y)
        distOtoB = math.hypot(gB.x - oSeed.x, gB.y - oSeed.y)
        pSeed = P2(oSeed.x + djx * distOtoB, oSeed.y + djy * distOtoB)
        lineOP = lines.addByTwoPoints(P2(oSeed.x, oSeed.y), P2(pSeed.x, pSeed.y))
        lineOP.isConstruction = True
        constraints.addCoincident(lineOP.startSketchPoint, drivingRootAxis)
        constraints.addCoincident(lineOP.endSketchPoint, bToApex2)
        constraints.addParallel(lineOP, lineDJ)
        self._distDim(sketch, lineDJ, lineOP, faceWidth, P2(pSeed.x, pSeed.y))
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint

        oGeom = pointO.geometry
        pGeom = pointP.geometry
        lineOD = lines.addByTwoPoints(P2(oGeom.x, oGeom.y), P2(dg.x, dg.y))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(P2(pGeom.x, pGeom.y), P2(gB.x, gB.y))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)

        # --- B->I line ---
        lineBI = lines.addByTwoPoints(P2(gB.x, gB.y), P2(iGeom.x, iGeom.y))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch, 'Gear Profiles')

        # ---- §3 build virtual spur tooth profiles (pinion then driving) ----
        (pinionToothSketch, pinionToothPlane, pinionEmbedded) = \
            self._buildVirtualSpurProfile(
                designComponent, gpPlane, module, pinionPitchDia, gamma_p,
                pointKp, lineCKp, 'Pinion')

        (drivingToothSketch, drivingToothPlane, drivingEmbedded) = \
            self._buildVirtualSpurProfile(
                designComponent, gpPlane, module, drivingPitchDia, gamma_g,
                pointLp, lineDLp, 'Driving')

        # ---- Per-gear bodies (pinion first, then driving) ----
        self._createGearBody(
            bevelComponent, designComponent, gpPlane, 'Pinion',
            [pointA, pointG, pointH, pointC, pointM, pointN],
            pinionToothSketch, pinionToothPlane, pinionEmbedded,
            apexSketchPoint, pinionTeeth, pinionBore,
            (pointM, pointN), (pointC, pointH), pointM, pointC,
            gamma_p, True, None)

        self._createGearBody(
            bevelComponent, designComponent, gpPlane, 'Driving',
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            drivingToothSketch, drivingToothPlane, drivingEmbedded,
            apexSketchPoint, drivingTeeth, drivingBore,
            (pointO, pointP), (pointD, pointJ), pointO, pointD,
            gamma_g, False, drivingTeeth)

        sketch.isVisible = False

    # =======================================================================
    # §3 virtual spur tooth profile
    # =======================================================================
    def _buildVirtualSpurProfile(self, designComponent, gpPlane, module,
                                 pitchDia, gamma, toothCenterPt, centerRefLine,
                                 gearLabel):
        # Virtual (Tredgold) tooth number.
        virtualPitchRadius = (pitchDia / 2.0) / math.cos(gamma)   # cm
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius / to_cm(module)))

        # Plane that includes C->K' (or D->L'), perpendicular to Gear Profiles.
        toothPlane = self._planeByAngle(designComponent, centerRefLine, gpPlane, 90)
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCenterPt, angle=math.radians(180))

        embedded = proxy._lastToothEmbedded

        self._assertFullyConstrained(toothSketch, f'{gearLabel} Tooth')

        # Construction axis through K'/L', normal to tooth plane, via setByTwoPlanes:
        # Gear Profiles plane ∩ (plane perpendicular to C->K' at its far end).
        helperPlane = self._planeBySetByDistanceOnPath(
            designComponent, centerRefLine, 1.0)
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gpPlane, helperPlane)
        designComponent.constructionAxes.add(axisInput)

        toothSketch.isVisible = False
        return (toothSketch, toothPlane, embedded)

    def _planeBySetByDistanceOnPath(self, comp, curve, fraction):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(curve, adsk.core.ValueInput.createByReal(fraction))
        return comp.constructionPlanes.add(planeInput)

    # =======================================================================
    # Profile-finding helper (tooth cross-section loop)
    # =======================================================================
    def _findSpurToothProfile(self, toothSketch, embedded):
        wantLines = 0 if embedded else 2
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nLines = nArcs = nNurbs = 0
                for pc in loop.profileCurves:
                    ct = pc.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        nLines += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        nArcs += 1
                    elif ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nNurbs += 1
                if nNurbs == 2 and nArcs == 2 and nLines == wantLines:
                    return profile
        raise Exception(
            f'Could not find tooth profile (expected 2 NURBS + 2 arcs + '
            f'{wantLines} lines, embedded={embedded})')

    # =======================================================================
    # Per-gear body creation
    # =======================================================================
    def _createGearBody(self, bevelComponent, designComponent, gpPlane,
                        gearLabel, hexVerts, toothSketch, toothPlane, embedded,
                        apexSketchPoint, teethNumber, boreDiameter,
                        toeEdge, heelEdge, toeConeWorld, heelConeWorld,
                        gamma, isPinion, drivingTeeth):

        # Target gear component (child of Bevel Gear)
        gearOcc = bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOcc.component.name = f'{gearLabel} Gear'

        # --- Profile sketch on the axial (Gear Profiles) plane ---
        profSketch = designComponent.sketches.add(gpPlane)
        profSketch.name = f'{gearLabel} Profile'
        profSketch.isVisible = True
        lines = profSketch.sketchCurves.sketchLines

        # Recreate the six vertices as fresh points (world-mapped), draw hexagon
        # sharing them, then fix lines/endpoints AFTER the lines exist.
        verts = [profSketch.sketchPoints.add(
            profSketch.modelToSketchSpace(v.worldGeometry)) for v in hexVerts]

        hexLines = []
        for i in range(len(verts)):
            a = verts[i]
            b = verts[(i + 1) % len(verts)]
            hexLines.append(lines.addByTwoPoints(a, b))

        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profSketch, f'{gearLabel} Profile')

        # Single hexagon loop -> single profile
        revolveProfile = profSketch.profiles.item(0)

        # Shaft axis = first edge (A->G / B->I)
        shaftAxisEdge = hexLines[0]

        # --- Revolve ---
        revolveInput = designComponent.features.revolveFeatures.createInput(
            revolveProfile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveFeature = designComponent.features.revolveFeatures.add(revolveInput)
        gearBody = revolveFeature.bodies.item(0)

        # --- Loft Apex -> tooth profile ---
        toothProfile = self._findSpurToothProfile(toothSketch, embedded)
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        toothBody = loftFeature.bodies.item(0)

        # --- toe/heel hand-off world points (per the §3a caller hand-off table) ---
        apexWorld = self._pointWorldGeometry(apexSketchPoint)
        toeMid = self._midWorld(toeEdge[0], toeEdge[1])
        heelMid = self._midWorld(heelEdge[0], heelEdge[1])
        toeConeW = self._pointWorldGeometry(toeConeWorld)
        heelConeW = self._pointWorldGeometry(heelConeWorld)

        # --- tooth-body step (straight = conical trims; spiral if psi>0) ---
        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            apexSketchPoint, toeMid, heelMid, toeConeW, heelConeW,
            toothPlane, gearLabel, teethNumber, gamma, isPinion)

        # --- Circular pattern around shaft axis ---
        toolBodies = adsk.core.ObjectCollection.create()
        toolBodies.add(toothBody)
        patternInput = designComponent.features.circularPatternFeatures.createInput(
            toolBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternFeature = designComponent.features.circularPatternFeatures.add(patternInput)

        # --- Combine-join patterned tooth bodies with gear body ---
        combineTools = adsk.core.ObjectCollection.create()
        for b in patternFeature.bodies:
            combineTools.add(b)
        combineInput = designComponent.features.combineFeatures.createInput(
            gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        designComponent.features.combineFeatures.add(combineInput)

        # --- Bore ---
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, boreDiameter, gearLabel)

        # --- Mesh rotation (driving by 180/N; pinion by _pinionMeshPhase) ---
        if not isPinion:
            meshAngle = math.radians(180.0) / float(drivingTeeth)
        else:
            meshAngle = self._pinionMeshPhase() * (2.0 * math.pi / float(teethNumber))
        if meshAngle != 0.0:
            self._rotateBody(designComponent, gearBody, shaftAxisEdge, meshAngle)

        # --- Relocate finished body into the gear component ---
        gearBody.moveToComponent(gearOcc)

    def _pinionMeshPhase(self):
        return _PINION_MESH_PHASE_TEETH

    def _midWorld(self, ptA, ptB):
        a = self._pointWorldGeometry(ptA)
        b = self._pointWorldGeometry(ptB)
        return adsk.core.Point3D.create(
            (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)

    def _rotateBody(self, comp, body, shaftAxisEdge, angle):
        sw = shaftAxisEdge.startSketchPoint.worldGeometry
        ew = shaftAxisEdge.endSketchPoint.worldGeometry
        axisVec = adsk.core.Vector3D.create(ew.x - sw.x, ew.y - sw.y, ew.z - sw.z)
        axisVec.normalize()
        self._rotateBodyAboutAxis(comp, body, axisVec, sw, angle)

    def _rotateBodyAboutAxis(self, comp, body, axisVec, originPoint, angle):
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angle, axisVec, originPoint)
        coll = adsk.core.ObjectCollection.create()
        coll.add(body)
        moveInput = comp.features.moveFeatures.createInput2(coll)
        moveInput.defineAsFreeMove(matrix)
        comp.features.moveFeatures.add(moveInput)

    # =======================================================================
    # Tooth-body hook
    # =======================================================================
    def _transformToothBody(self, designComponent, toothBody, gearBody,
                            shaftAxisEdge, apexWorld, apexSketchPoint, toeMid,
                            heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber, gamma,
                            isPinion):
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)

        return self._buildSpiralTooth(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane,
            gearLabel, gamma, isPinion)

    # =======================================================================
    # Conical-trim helpers (straight tooth, also flushes spiral ends)
    # =======================================================================
    def _surfaceDistance(self, surface, worldPoint):
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return worldPoint.distanceTo(projected)

    def _findConeFaceForCutLine(self, body, worldA, worldB):
        """The ConeSurfaceType face whose surface contains both world endpoints."""
        best = None
        bestScore = None
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            dA = self._surfaceDistance(face.geometry, worldA)
            dB = self._surfaceDistance(face.geometry, worldB)
            score = (0.0 if dA is None else dA) + (0.0 if dB is None else dB)
            if bestScore is None or score < bestScore:
                bestScore = score
                best = face
        return best

    def _coneFacesByDistance(self, body, edgeMidWorld):
        candidates = []
        for face in body.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                d = self._surfaceDistance(face.geometry, edgeMidWorld)
                candidates.append((face, d))
        # order best-first; unevaluable distance (None) tried last
        candidates.sort(key=lambda fd: (fd[1] is None, fd[1] if fd[1] is not None else 0.0))
        return candidates

    def _applyConicalCut(self, designComponent, targetBody, frustumBody,
                         edgeMidWorld, apexWorld, gearLabel, cutLabel):
        candidates = self._coneFacesByDistance(frustumBody, edgeMidWorld)
        history = []
        for (face, dist) in candidates:
            try:
                splitInput = designComponent.features.splitBodyFeatures.createInput(
                    targetBody, face, True)
                splitFeature = designComponent.features.splitBodyFeatures.add(splitInput)
                pieces = list(splitFeature.bodies)
                history.append((dist, len(pieces)))
                if len(pieces) > 1:
                    futil.log(
                        f'{gearLabel}: {cutLabel} cut split into {len(pieces)} '
                        f'pieces (face dist={dist})', force_console=True)
                    return self._keepLargestNonApex(
                        designComponent, pieces, apexWorld, gearLabel, cutLabel)
            except RuntimeError as e:
                history.append((dist, f'error:{e}'))
                continue
        raise Exception(
            f'{gearLabel}: {cutLabel} cut found no cone face that split the body '
            f'(candidates dist/result: {history})')

    def _keepLargestNonApex(self, designComponent, pieces, apexWorld, gearLabel, cutLabel):
        nonApex = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            isApex = containment in (
                adsk.fusion.PointContainment.PointInsidePointContainment,
                adsk.fusion.PointContainment.PointOnPointContainment)
            if isApex:
                continue
            nonApex.append(body)
        if len(nonApex) == 0:
            raise Exception(
                f'{gearLabel}: {cutLabel} keeper selection found no non-apex piece '
                f'(pieces in={len(pieces)})')
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for body in pieces:
            if body == keeper:
                continue
            designComponent.features.removeFeatures.add(body)
        return keeper

    def _cutConicalEnds(self, designComponent, toothBody, gearBody, toeMid,
                        heelMid, apexWorld, gearLabel):
        # Cut 1: toe cone (M->N / O->P), located by toe-edge midpoint.
        keeper = self._applyConicalCut(
            designComponent, toothBody, gearBody, toeMid, apexWorld,
            gearLabel, 'toe')

        # Cut 2: heel cone (C->H / D->J), on the keeper alone, by heel-edge midpoint.
        try:
            keeper = self._applyConicalCut(
                designComponent, keeper, gearBody, heelMid, apexWorld,
                gearLabel, 'heel')
        except RuntimeError as e:
            msg = str(e)
            if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                futil.log(f'{gearLabel}: heel cone did not intersect keeper; kept whole',
                          force_console=True)
            else:
                raise
        except Exception as e:
            if 'no cone face that split' in str(e):
                futil.log(f'{gearLabel}: heel cone did not split keeper; kept whole',
                          force_console=True)
            else:
                raise
        return keeper

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, boreDiameter, gearLabel):
        # Bore plane normal to shaft at its start.
        borePlane = self._planeBySetByDistanceOnPath(
            designComponent, shaftAxisEdge, 0.0)
        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore'
        boreSketch.isVisible = True

        circles = boreSketch.sketchCurves.sketchCircles
        dimensions = boreSketch.sketchDimensions
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter / 2.0)
        circle.centerSketchPoint.isFixed = True
        diamDim = dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter / 2.0, 0, 0))
        diamDim.parameter.value = boreDiameter

        self._assertFullyConstrained(boreSketch, f'{gearLabel} Bore')

        boreProfile = boreSketch.profiles.item(0)
        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(100.0), False)
        extrudeInput.participantBodies = [gearBody]
        designComponent.features.extrudeFeatures.add(extrudeInput)
        boreSketch.isVisible = False

    # =======================================================================
    # §3a Spiral tooth body (psi > 0)
    # =======================================================================
    def _buildSpiralTooth(self, designComponent, toothBody, gearBody,
                          shaftAxisEdge, apexWorld, toeMid, heelMid,
                          toeConeWorld, heelConeWorld, parentToothPlane,
                          gearLabel, gamma, isPinion):
        # --- A. Gate & frame ---
        sw = shaftAxisEdge.startSketchPoint.worldGeometry
        ew = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = self._vecNorm(ew.x - sw.x, ew.y - sw.y, ew.z - sw.z)
        apex = apexWorld

        # Fix swapped toe/heel: heel must be the OUTER end (farther from apex).
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = self._vecNorm(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y, heelConeWorld.z - apex.z)
        v = self._vecNormV(self._cross(axisDir, coneVec))
        # tpNormal (coneVec x v) is the tangent-plane normal in the derivation;
        # the analytic step-G twist needs no projection, so it is not used here.

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x +
                    (p.y - apex.y) * coneVec.y +
                    (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- B. Cutter-arc geometry (tangent-plane 2-D: x=coneVec, y=v) ---
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1.0 if self._spiralHand == _HAND_RIGHT else -1.0
        if isPinion:
            handSign = -handSign

        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = self._circleIntersectNearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)
        mean2d = (R_mean, 0.0)

        # --- C. 2-D trace sketch (genuine cutter arc) ---
        coneElementEnd = self._combine(apex, R_heel, coneVec)
        ceSketch = designComponent.sketches.add(self._gpPlaneRef)
        ceSketch.name = f'{gearLabel} Cone Element'
        ceSketch.isVisible = True
        ceLines = ceSketch.sketchCurves.sketchLines
        apexLocal = ceSketch.modelToSketchSpace(apex)
        endLocal = ceSketch.modelToSketchSpace(coneElementEnd)
        coneElementLine = ceLines.addByTwoPoints(apexLocal, endLocal)
        coneElementLine.isConstruction = True

        tangentPlane = self._planeByAngle(
            designComponent, coneElementLine, self._gpPlaneRef, 90)

        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        traceSketch.isVisible = True

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        tCircles = traceSketch.sketchCurves.sketchCircles
        tArcs = traceSketch.sketchCurves.sketchArcs
        tDims = traceSketch.sketchDimensions

        centerLocal = traceSketch.modelToSketchSpace(tanW(Cx, Cy))
        cutterCircle = tCircles.addByCenterRadius(centerLocal, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        diamDim = tDims.addDiameterDimension(
            cutterCircle, traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy)))
        diamDim.parameter.value = 2 * r_c

        toeLocal = traceSketch.modelToSketchSpace(tanW(toe2d[0], toe2d[1]))
        meanLocal = traceSketch.modelToSketchSpace(tanW(mean2d[0], mean2d[1]))
        heelLocal = traceSketch.modelToSketchSpace(tanW(heel2d[0], heel2d[1]))
        traceArc = tArcs.addByThreePoints(toeLocal, meanLocal, heelLocal)
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        rdim = tDims.addRadialDimension(
            traceArc, traceSketch.modelToSketchSpace(tanW(R_mean, Cy * 0.5)))
        rdim.parameter.value = r_c
        # trace sketch deliberately left with free DOF (exempt from gate).

        # --- E. Slice the straight tooth ---
        segments = self._sliceTooth(
            designComponent, toothBody, parentToothPlane, apex, span, gearLabel)

        # --- F. Order & drop scrap ---
        segments.sort(key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                f'{gearLabel}: spiral slice produced no working segments after '
                f'dropping scrap (span={span})')

        # --- G. Twist ---
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        def slabHeelFaceDist(body):
            best = None
            bestD = None
            for face in body.faces:
                d = distAlong(face.centroid)
                if bestD is None or d > bestD:
                    bestD = d
                    best = face
            return best, bestD

        segAngles = []
        for seg in segments:
            _hf, rHeelFace = slabHeelFaceDist(seg)
            ang = -handSign * total * (R_mean - rHeelFace) / span
            segAngles.append(ang)
            self._rotateBodyAboutAxis(designComponent, seg, axisDir, apex, ang)

        # --- H. Lengthwise crown (relief) ---
        if _CROWN_PER_RAD > 0:
            order_after_twist = sorted(
                range(len(segments)),
                key=lambda i: slabHeelFaceDist(segments[i])[1])
            outermostIdx = order_after_twist[-1]
            self._designOccurrence.activate()
            try:
                for i, seg in enumerate(segments):
                    if i == outermostIdx:
                        continue
                    self._crownScale(designComponent, seg, distAlong,
                                     1.0 - _CROWN_PER_RAD * abs(segAngles[i]))
            finally:
                self.design.activateRootComponent()

        # --- I. Loft -> curved tooth (re-sort by heel-face cone distance NOW) ---
        order = sorted(range(len(segments)),
                       key=lambda i: slabHeelFaceDist(segments[i])[1])
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        toeSeg = segments[order[0]]
        loftInput.loftSections.add(self._slabToeFace(toeSeg, distAlong))
        for i in order:
            hf, _d = slabHeelFaceDist(segments[i])
            loftInput.loftSections.add(hf)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        curvedTooth = loftFeature.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        # remove the segment scaffolding (loft has captured their faces)
        for seg in segments:
            try:
                designComponent.features.removeFeatures.add(seg)
            except Exception:
                pass

        # --- J. Flush trim ---
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apexWorld, gearLabel)

    def _slabToeFace(self, body, distAlong):
        best = None
        bestD = None
        for face in body.faces:
            d = distAlong(face.centroid)
            if bestD is None or d < bestD:
                bestD = d
                best = face
        return best

    def _sliceTooth(self, designComponent, toothBody, parentToothPlane, apex,
                    span, gearLabel):
        def attempt(sign):
            pieces = [toothBody]
            for k in range(8):
                offset = sign * (k + 1) * span / 6.0
                plane = self._offsetPlane(designComponent, parentToothPlane, offset)
                newPieces = []
                for piece in pieces:
                    try:
                        splitInput = designComponent.features.splitBodyFeatures.createInput(
                            piece, plane, True)
                        sf = designComponent.features.splitBodyFeatures.add(splitInput)
                        for b in sf.bodies:
                            newPieces.append(b)
                    except RuntimeError:
                        newPieces.append(piece)
                pieces = newPieces
            return pieces

        # Choose sign so the first plane moves toward the apex.
        normal = get_normal(parentToothPlane)
        origin = parentToothPlane.geometry.origin
        toApex = adsk.core.Vector3D.create(
            apex.x - origin.x, apex.y - origin.y, apex.z - origin.z)
        sign = 1.0 if (normal.x * toApex.x + normal.y * toApex.y +
                       normal.z * toApex.z) > 0 else -1.0

        pieces = attempt(sign)
        if len(pieces) <= 1:
            pieces = attempt(-sign)
            sign = -sign
        if len(pieces) <= 1:
            raise Exception(
                f'{gearLabel}: spiral slice failed to split the tooth '
                f'(pieces={len(pieces)}, span={span}, sign tried={sign})')
        return pieces

    def _offsetPlane(self, comp, refPlane, offset):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByOffset(refPlane, adsk.core.ValueInput.createByReal(offset))
        return comp.constructionPlanes.add(planeInput)

    def _crownScale(self, comp, body, distAlong, factor):
        # Scale about the centre of the body's heel face (a sketch point on it).
        best = None
        bestD = None
        for face in body.faces:
            d = distAlong(face.centroid)
            if bestD is None or d > bestD:
                bestD = d
                best = face
        heelFace = best
        scaleSketch = comp.sketches.add(heelFace)
        basePoint = scaleSketch.sketchPoints.add(
            scaleSketch.modelToSketchSpace(heelFace.centroid))
        coll = adsk.core.ObjectCollection.create()
        coll.add(body)
        scaleInput = comp.features.scaleFeatures.createInput(
            coll, basePoint, adsk.core.ValueInput.createByReal(factor))
        comp.features.scaleFeatures.add(scaleInput)

    # ------------------------------------------------------------------
    # small vector helpers
    @staticmethod
    def _vecNorm(x, y, z):
        v = adsk.core.Vector3D.create(x, y, z)
        v.normalize()
        return v

    @staticmethod
    def _vecNormV(v):
        v.normalize()
        return v

    @staticmethod
    def _cross(a, b):
        return adsk.core.Vector3D.create(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x)

    @staticmethod
    def _perpToAxis(vec, axisDir):
        """Component of vec perpendicular to axisDir."""
        dot = vec.x * axisDir.x + vec.y * axisDir.y + vec.z * axisDir.z
        return adsk.core.Vector3D.create(
            vec.x - dot * axisDir.x,
            vec.y - dot * axisDir.y,
            vec.z - dot * axisDir.z)

    def _bottomEdgeMid(self, ptA, ptB):
        return self._midWorld(ptA, ptB)

    # =======================================================================
    # Cleanup
    # =======================================================================
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            if component.entityToken in seen:
                return
            seen.add(component.entityToken)
            for sketch in component.sketches:
                if sketch.entityToken not in seen:
                    seen.add(sketch.entityToken)
                    sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                if plane.entityToken not in seen:
                    seen.add(plane.entityToken)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                if axis.entityToken not in seen:
                    seen.add(axis.entityToken)
                    axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
