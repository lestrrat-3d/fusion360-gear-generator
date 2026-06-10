import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ...
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the reproduced surface -- 17 inputs / 17 INPUT_ID_* consts)
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

# Hand-of-spiral dropdown item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Pressure angle is hard-coded for bevel (not a dialog input)
_PRESSURE_ANGLE_DEG = 20.0

# Spur drawer parameter-name keys the proxy must serve.
_SPUR_PARAM_MODULE = 'Module'
_SPUR_PARAM_TOOTH_NUMBER = 'ToothNumber'
_SPUR_PARAM_PRESSURE_ANGLE = 'PressureAngle'
_SPUR_PARAM_PITCH_DIAMETER = 'PitchCircleDiameter'
_SPUR_PARAM_PITCH_RADIUS = 'PitchCircleRadius'
_SPUR_PARAM_BASE_DIAMETER = 'BaseCircleDiameter'
_SPUR_PARAM_BASE_RADIUS = 'BaseCircleRadius'
_SPUR_PARAM_ROOT_DIAMETER = 'RootCircleDiameter'
_SPUR_PARAM_ROOT_RADIUS = 'RootCircleRadius'
_SPUR_PARAM_TIP_DIAMETER = 'TipCircleDiameter'
_SPUR_PARAM_TIP_RADIUS = 'TipCircleRadius'
_SPUR_PARAM_INVOLUTE_STEPS = 'InvoluteSteps'


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first, so it wins Fusion auto-focus
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
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component for the new gear')
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

        # Initial conditional visibility for the spiral-only inputs.
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
            # spiralAngle reads back in internal radians.
            value = spiralInput.value
        except Exception:
            # A half-typed expression can raise mid-edit: leave both shown.
            handInput.isVisible = True
            cutterInput.isVisible = True
            return
        show = (value > 0)
        handInput.isVisible = show
        cutterInput.isVisible = show


# ---------------------------------------------------------------------------
# Virtual spur proxy (so the borrowed spur tooth generator can run without
# registering Fusion user parameters)
# ---------------------------------------------------------------------------
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        # _lastToothEmbedded is an OUTPUT the spur generator writes back.
        self._lastToothEmbedded = False

        alpha = math.radians(_PRESSURE_ANGLE_DEG)
        # Standard spur formulas; lengths in mm, then to_cm for geometry use.
        pitch_mm = virtualTeeth * module_mm
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._params = {
            _SPUR_PARAM_MODULE: _Val(module_mm),
            _SPUR_PARAM_TOOTH_NUMBER: _Val(virtualTeeth),
            _SPUR_PARAM_PRESSURE_ANGLE: _Val(alpha),
            _SPUR_PARAM_PITCH_DIAMETER: _Val(to_cm(pitch_mm)),
            _SPUR_PARAM_PITCH_RADIUS: _Val(to_cm(pitch_mm / 2)),
            _SPUR_PARAM_BASE_DIAMETER: _Val(to_cm(base_mm)),
            _SPUR_PARAM_BASE_RADIUS: _Val(to_cm(base_mm / 2)),
            _SPUR_PARAM_ROOT_DIAMETER: _Val(to_cm(root_mm)),
            _SPUR_PARAM_ROOT_RADIUS: _Val(to_cm(root_mm / 2)),
            _SPUR_PARAM_TIP_DIAMETER: _Val(to_cm(tip_mm)),
            _SPUR_PARAM_TIP_RADIUS: _Val(to_cm(tip_mm / 2)),
            _SPUR_PARAM_INVOLUTE_STEPS: _Val(15),
        }

    def getParameter(self, name):
        return self._params[name]


# ---------------------------------------------------------------------------
# The bevel generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    # Tunable class constants (reproduced surface).
    _CROWN_PER_RAD = 0.5            # lengthwise crown relief per radian of twist (0 disables)
    _PINION_MESH_PHASE_TEETH = 0   # extra pinion mesh phase in tooth-fractions (0 default)

    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # ----------------------------------------------------------------- inputs
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        (parentList, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentList) != 1:
            raise Exception('Exactly one Parent Component must be selected')
        parentEntity = parentList[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentEntity.component
        else:
            parentComponent = parentEntity

        (planeList, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeList) != 1:
            raise Exception('Exactly one Target Plane must be selected')
        targetPlane = planeList[0]

        (centerList, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centerList) != 1:
            raise Exception('Exactly one Center Point must be selected')
        centerPoint = centerList[0]

        # Module: unit '' -> raw number meaning millimetres.
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft Angle: 'deg' reads back in radians.
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.3f})')

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')

        # 'mm' inputs read back already in internal cm.
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

        self._toothSpacing_cm = evalExpr(INPUT_ID_TOOTH_SPACING, 'mm')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        # Spiral inputs.
        spiralAngle_rad = evalExpr(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be in [0, 60) degrees (got {spiralAngle_deg:.3f})')
        self._spiralAngle_rad = spiralAngle_rad

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem = handInput.selectedItem
        self._hand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        self._cutterRadius_cm = evalExpr(INPUT_ID_CUTTER_RADIUS, 'mm')
        if self._cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ------------------------------------------------------------------ utils
    def _pointWorldGeometry(self, point):
        # SketchPoint -> worldGeometry; ConstructionPoint -> geometry.
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _combine(self, base, a, e1, b=0.0, e2=None):
        # world point = base + a*e1 (+ b*e2); lengths in cm.
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, angleDeg):
        planes = comp.constructionPlanes
        ci = planes.createInput()
        ci.setByAngle(line, adsk.core.ValueInput.createByString(f'{angleDeg} deg'), refPlane)
        return planes.add(ci)

    def _distDim(self, sketch, lineA, lineB, textPoint, value):
        dim = sketch.sketchDimensions.addOffsetDimension(lineA, lineB, textPoint)
        dim.parameter.value = value
        return dim

    def _bottomEdgeMid(self, p0, p1):
        # World midpoint of an edge given its two world endpoints.
        return adsk.core.Point3D.create(
            (p0.x + p1.x) / 2, (p0.y + p1.y) / 2, (p0.z + p1.z) / 2)

    def _perpToAxis(self, p, apex, axisDir):
        # Perpendicular distance from world point p to the shaft axis
        # (the line through apex along axisDir).
        wx = p.x - apex.x
        wy = p.y - apex.y
        wz = p.z - apex.z
        dot = wx * axisDir.x + wy * axisDir.y + wz * axisDir.z
        px = wx - dot * axisDir.x
        py = wy - dot * axisDir.y
        pz = wz - dot * axisDir.z
        return math.sqrt(px * px + py * py + pz * pz)

    def _findConeFaceForCutLine(self, gearBody, edgeMidWorld):
        # Order the frustum's ConeSurfaceType faces best-first by surface
        # distance to the cut edge's world midpoint (unevaluable -> +inf, last).
        coneFaces = self._findConeFaces(gearBody)
        return sorted(
            coneFaces,
            key=lambda f: self._surfaceDistance(f.geometry, edgeMidWorld))

    def _circleIntersectNearest(self, R, Cx, Cy, r_c, refX, refY):
        # Intersect apex circle radius R (centre origin) with cutter circle
        # (centre (Cx,Cy), radius r_c); keep the solution nearest (refX, refY).
        d = math.hypot(Cx, Cy)
        if d == 0:
            return (R, 0.0)
        # distance from origin to the radical line along the centre direction
        a = (R * R - r_c * r_c + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)
        ux = Cx / d
        uy = Cy / d
        # base point on the line between centres
        px = a * ux
        py = a * uy
        # two candidates (perpendicular offset)
        s1 = (px - h * uy, py + h * ux)
        s2 = (px + h * uy, py - h * ux)
        d1 = math.hypot(s1[0] - refX, s1[1] - refY)
        d2 = math.hypot(s2[0] - refX, s2[1] - refY)
        return s1 if d1 <= d2 else s2

    # ----------------------------------------------------------- orchestration
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Resolve pitch diameters & bores (Python, cm).
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)
        drivingBore_cm = self._drivingBore_cm if self._drivingBore_cm > 0 \
            else drivingPitchDiameter_cm / 4
        pinionBore_cm = self._pinionBore_cm if self._pinionBore_cm > 0 \
            else pinionPitchDiameter_cm / 4

        self._module_mm = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_deg = shaftAngle_deg
        self._drivingPitchDiameter_cm = drivingPitchDiameter_cm
        self._pinionPitchDiameter_cm = pinionPitchDiameter_cm
        self._drivingBoreResolved_cm = drivingBore_cm
        self._pinionBoreResolved_cm = pinionBore_cm

        # Cone distance (cm) and pitch cone half-angles.
        sigma = math.radians(shaftAngle_deg)
        PPD = pinionPitchDiameter_cm
        DPD = drivingPitchDiameter_cm
        self._coneDistance_cm = math.sqrt(
            to_cm(module * drivingTeeth) ** 2 + to_cm(module * pinionTeeth) ** 2)
        gamma_p = math.atan2(math.sin(sigma) * PPD, DPD + PPD * math.cos(sigma))
        gamma_g = sigma - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        # Build the component tree.
        matrix = adsk.core.Matrix3D.create()
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(matrix)
        bevelComponent = self.bevelOccurrence.component
        bevelComponent.name = self._generateName()

        designOccurrence = bevelComponent.occurrences.addNewComponent(matrix)
        designComponent = designOccurrence.component
        designComponent.name = 'Design'
        self._designOccurrence = designOccurrence
        self._designComponent = designComponent

        futil.log('bevel: building anchor sketch', force_console=True)
        anchorData = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        futil.log('bevel: building gear profiles', force_console=True)
        self._buildGearProfiles(
            bevelComponent, designComponent, targetPlane, anchorData)

        self._hideConstructionGeometry(bevelComponent)

    def _generateName(self):
        return f'Bevel Gear (m{self._module_mm} {self._drivingTeeth}x{self._pinionTeeth})'

    # ---------------------------------------------------------- §1 anchor sketch
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        c = projectedCenter.geometry
        # Anchor line through the projected center.
        p0 = adsk.core.Point3D.create(c.x - 0.5, c.y, 0)
        p1 = adsk.core.Point3D.create(c.x + 0.5, c.y, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)

        # Pin the center onto the line AND make it the midpoint.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)
        # ~10mm reference length.
        lenDim = dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + 0.3, 0))
        lenDim.parameter.value = to_cm(10)
        # Pin its direction sketch-locally.
        constraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor Sketch ended under-constrained')

        # Stash the projected-center sketch point for §2.
        self._anchorProjectedCenter = projectedCenter
        return {
            'sketch': sketch,
            'anchorLine': anchorLine,
            'projectedCenter': projectedCenter,
        }

    # ----------------------------------------------------------- §2 + §3 + bodies
    def _buildGearProfiles(self, bevelComponent, designComponent, targetPlane, anchorData):
        anchorLine = anchorData['anchorLine']
        projectedCenterSrc = anchorData['projectedCenter']

        # Gear Profiles plane: includes the Anchor Line, perpendicular to targetPlane.
        gpPlane = self._planeByAngle(
            designComponent, anchorLine, targetPlane, 90)
        self._gpPlaneRef = gpPlane

        sketch = designComponent.sketches.add(gpPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the anchor-sketch center point (NOT the raw user center).
        projCenter = sketch.project(projectedCenterSrc).item(0)
        projAnchor = sketch.project(anchorLine).item(0)

        c = projCenter.geometry
        a0 = projAnchor.startSketchPoint.geometry
        a1 = projAnchor.endSketchPoint.geometry
        dx = a1.x - a0.x
        dy = a1.y - a0.y
        dlen = math.hypot(dx, dy)
        d = (dx / dlen, dy / dlen)
        perp = (-d[1], d[0])

        # Choose perp sign by the target-plane normal (one bit, toward normal).
        normal = get_normal(targetPlane)
        # In-plane perp mapped to world to compare against the normal direction.
        perpWorld = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perp[0], c.y + perp[1], 0))
        cWorld = sketch.sketchToModelSpace(adsk.core.Point3D.create(c.x, c.y, 0))
        pwx = perpWorld.x - cWorld.x
        pwy = perpWorld.y - cWorld.y
        pwz = perpWorld.z - cWorld.z
        if (pwx * normal.x + pwy * normal.y + pwz * normal.z) < 0:
            perp = (-perp[0], -perp[1])

        DPD = self._drivingPitchDiameter_cm
        PPD = self._pinionPitchDiameter_cm
        module_cm = to_cm(self._module_mm)

        # ----- Apex: free end of a construction line from the projected center.
        apexSeed = adsk.core.Point3D.create(
            c.x + perp[0] * DPD, c.y + perp[1] * DPD, 0)
        centerToApex = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x, c.y, 0), apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projCenter)
        constraints.addPerpendicular(centerToApex, projAnchor)
        apexPoint = centerToApex.endSketchPoint

        # Closed-form cone geometry seeds for along-shaft lengths.
        sigma = math.radians(self._shaftAngle_deg)
        gamma_p = self._gamma_p
        gamma_g = self._gamma_g
        R = (PPD / 2) / math.sin(gamma_p) if math.sin(gamma_p) != 0 else self._coneDistance_cm
        lenA = R * math.cos(gamma_p)
        lenB = R * math.cos(gamma_g)

        ap = apexPoint.geometry
        # Driving Gear Shaft Axis: from apex toward the anchor line (-perp), end = B.
        bSeed = adsk.core.Point3D.create(
            ap.x - perp[0] * lenB, ap.y - perp[1] * lenB, 0)
        drivingShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap.x, ap.y, 0), bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Gear Shaft Axis: driving direction rotated about apex by ±Σ.
        def rotateAbout(px, py, ox, oy, ang):
            sx = px - ox
            sy = py - oy
            rx = sx * math.cos(ang) - sy * math.sin(ang)
            ry = sx * math.sin(ang) + sy * math.cos(ang)
            return (ox + rx, oy + ry)

        drivDirX = -perp[0]
        drivDirY = -perp[1]
        candPlus = rotateAbout(ap.x + drivDirX * lenA, ap.y + drivDirY * lenA,
                               ap.x, ap.y, sigma)
        candMinus = rotateAbout(ap.x + drivDirX * lenA, ap.y + drivDirY * lenA,
                                ap.x, ap.y, -sigma)
        # Keep the candidate whose endpoint has the greater X.
        aSeedTuple = candPlus if candPlus[0] >= candMinus[0] else candMinus
        aSeed = adsk.core.Point3D.create(aSeedTuple[0], aSeedTuple[1], 0)
        pinionShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap.x, ap.y, 0), aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        # Angular dimension Σ between the two shaft axes (place text in the Σ wedge).
        wedgeMidX = ap.x + (drivDirX + (aSeedTuple[0] - ap.x) / lenA) * 0.3 * lenA
        wedgeMidY = ap.y + (drivDirY + (aSeedTuple[1] - ap.y) / lenA) * 0.3 * lenA
        angDim = dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis,
            adsk.core.Point3D.create(wedgeMidX, wedgeMidY, 0))
        angDim.parameter.value = sigma
        pointA = pinionShaftAxis.endSketchPoint

        # ----- A -> Apex2 perpendicular drop (PPD/2), toward Apex2 side.
        pa = pointA.geometry
        # direction toward anchor line (-perp) is the Apex2 side
        aApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(pa.x, pa.y, 0),
            adsk.core.Point3D.create(pa.x - perp[0] * (PPD / 2), pa.y - perp[1] * (PPD / 2), 0))
        aApex2.isConstruction = True
        constraints.addCoincident(aApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aApex2, pinionShaftAxis)
        aApex2Dim = dimensions.addDistanceDimension(
            aApex2.startSketchPoint, aApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(pa.x - perp[0] * 0.3, pa.y - perp[1] * 0.3, 0))
        aApex2Dim.parameter.value = PPD / 2

        # ----- B -> Apex2 perpendicular drop (DPD/2).
        pb = pointB.geometry
        bApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(pb.x, pb.y, 0),
            adsk.core.Point3D.create(pb.x - perp[0] * (DPD / 2), pb.y - perp[1] * (DPD / 2), 0))
        bApex2.isConstruction = True
        constraints.addCoincident(bApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bApex2, drivingShaftAxis)
        bApex2Dim = dimensions.addDistanceDimension(
            bApex2.startSketchPoint, bApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(pb.x - perp[0] * 0.3, pb.y - perp[1] * 0.3, 0))
        bApex2Dim.parameter.value = DPD / 2

        # Coincide the two drop ends -> Apex2.
        constraints.addCoincident(aApex2.endSketchPoint, bApex2.endSketchPoint)
        apex2Point = aApex2.endSketchPoint

        # Pitch Line Apex -> Apex2.
        pitchLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap.x, ap.y, 0),
            adsk.core.Point3D.create(apex2Point.geometry.x, apex2Point.geometry.y, 0))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # Dedendum lines from Apex2, perpendicular to pitch line, length 1.25*module.
        ded = to_cm(1.25 * self._module_mm)
        a2 = apex2Point.geometry
        # pitch line direction
        pdx = a2.x - ap.x
        pdy = a2.y - ap.y
        pdl = math.hypot(pdx, pdy)
        pux = pdx / pdl
        puy = pdy / pdl
        # perpendicular to pitch line (two senses)
        ped = (-puy, pux)
        # Driving Gear Dedendum -> toward anchor line -> point D
        # choose the sense pointing toward -perp (anchor line side) for D
        if (ped[0] * (-perp[0]) + ped[1] * (-perp[1])) >= 0:
            dDir = ped
            cDir = (-ped[0], -ped[1])
        else:
            dDir = (-ped[0], -ped[1])
            cDir = ped
        drivingDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(a2.x, a2.y, 0),
            adsk.core.Point3D.create(a2.x + dDir[0] * ded, a2.y + dDir[1] * ded, 0))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dDim = dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(a2.x + dDir[0] * 0.2, a2.y + dDir[1] * 0.2, 0))
        dDim.parameter.value = ded
        pointD = drivingDedendum.endSketchPoint

        pinionDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(a2.x, a2.y, 0),
            adsk.core.Point3D.create(a2.x + cDir[0] * ded, a2.y + cDir[1] * ded, 0))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        cDim = dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(a2.x + cDir[0] * 0.2, a2.y + cDir[1] * 0.2, 0))
        cDim.parameter.value = ded
        pointC = pinionDedendum.endSketchPoint

        # Root Axis lines Apex->D and Apex->C.
        pcD = pointD.geometry
        drivingRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap.x, ap.y, 0),
            adsk.core.Point3D.create(pcD.x, pcD.y, 0))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pcC = pointC.geometry
        pinionRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap.x, ap.y, 0),
            adsk.core.Point3D.create(pcC.x, pcC.y, 0))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # ----- module-length extensions (helper returns the LINE, reused later).
        def collinearExtension(fromPoint, alongLine, awayFromPoint):
            # Build a module-length line from fromPoint, collinear with alongLine,
            # extending away from awayFromPoint.
            fp = fromPoint.geometry
            ax = awayFromPoint.geometry
            ddx = fp.x - ax.x
            ddy = fp.y - ax.y
            dl = math.hypot(ddx, ddy)
            ux = ddx / dl
            uy = ddy / dl
            seed = adsk.core.Point3D.create(
                fp.x + ux * module_cm, fp.y + uy * module_cm, 0)
            ln = lines.addByTwoPoints(
                adsk.core.Point3D.create(fp.x, fp.y, 0), seed)
            ln.isConstruction = True
            constraints.addCoincident(ln.startSketchPoint, fromPoint)
            constraints.addCollinear(ln, alongLine)
            return ln

        # A->E collinear with Apex->A (pinionShaftAxis), away from Apex.
        lineAE = collinearExtension(pointA, pinionShaftAxis, apexPoint)
        pointE = lineAE.endSketchPoint

        # C->E line; A->E and C->E perpendicular.
        pe = pointE.geometry
        lineCE = lines.addByTwoPoints(
            adsk.core.Point3D.create(pcC.x, pcC.y, 0),
            adsk.core.Point3D.create(pe.x, pe.y, 0))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # B->F collinear with Apex->B (drivingShaftAxis), away from Apex.
        lineBF = collinearExtension(pointB, drivingShaftAxis, apexPoint)
        pointF = lineBF.endSketchPoint

        pf = pointF.geometry
        lineDF = lines.addByTwoPoints(
            adsk.core.Point3D.create(pcD.x, pcD.y, 0),
            adsk.core.Point3D.create(pf.x, pf.y, 0))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # E->G collinear with A->E, module length.
        lineEG = collinearExtension(pointE, lineAE, pointA)
        pointG = lineEG.endSketchPoint

        # C->H, module length, collinear with Apex2->C (pinionDedendum).
        lineCH = collinearExtension(pointC, pinionDedendum, apex2Point)
        pointH = lineCH.endSketchPoint

        # G->H line; E->G and H->G perpendicular.
        pg = pointG.geometry
        ph = pointH.geometry
        lineGH = lines.addByTwoPoints(
            adsk.core.Point3D.create(pg.x, pg.y, 0),
            adsk.core.Point3D.create(ph.x, ph.y, 0))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # F->I collinear with B->F, module length.
        lineFI = collinearExtension(pointF, lineBF, pointB)
        pointI = lineFI.endSketchPoint

        # D->J, module length, collinear with Apex2->D (drivingDedendum).
        lineDJ = collinearExtension(pointD, drivingDedendum, apex2Point)
        pointJ = lineDJ.endSketchPoint

        pi = pointI.geometry
        pj = pointJ.geometry
        lineIJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(pi.x, pi.y, 0),
            adsk.core.Point3D.create(pj.x, pj.y, 0))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # Base-height offset dims (J->I parallel to B->Apex2 by construction).
        drivingBaseHeight = self._drivingBaseHeight_cm if self._drivingBaseHeight_cm > 0 \
            else to_cm(self._module_mm * self._drivingTeeth / 8)
        self._distDim(
            sketch, bApex2, lineIJ,
            adsk.core.Point3D.create(pi.x - perp[0] * 0.2, pi.y - perp[1] * 0.2, 0),
            drivingBaseHeight)

        pinionBaseHeight = self._pinionBaseHeight_cm if self._pinionBaseHeight_cm > 0 \
            else drivingBaseHeight * (self._pinionTeeth / self._drivingTeeth)
        self._distDim(
            sketch, aApex2, lineGH,
            adsk.core.Point3D.create(pg.x - perp[0] * 0.2, pg.y - perp[1] * 0.2, 0),
            pinionBaseHeight)

        # A->G line.
        lineAG = lines.addByTwoPoints(
            adsk.core.Point3D.create(pa.x, pa.y, 0),
            adsk.core.Point3D.create(pg.x, pg.y, 0))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # Constrain Point I with center point.
        constraints.addCoincident(pointI, projCenter)

        # ----- K (point-on-line intersection of Apex->A and Apex2->C extended).
        pgg = pointG.geometry
        kSeed = adsk.core.Point3D.create(
            pgg.x + (pgg.x - pa.x), pgg.y + (pgg.y - pa.y), 0)
        lineGK = lines.addByTwoPoints(
            adsk.core.Point3D.create(pgg.x, pgg.y, 0), kSeed)
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)
        # C->K reference line.
        pk = pointK.geometry
        lineCK = lines.addByTwoPoints(
            adsk.core.Point3D.create(pcC.x, pcC.y, 0),
            adsk.core.Point3D.create(pk.x, pk.y, 0))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # K' tooth-center (Tooth Spacing offset).
        (pointKp, lineCKp) = self._buildToothCenter(
            sketch, constraints, dimensions, lines,
            pointK, pointC, pinionDedendum)

        # ----- L (intersection of Apex->B and Apex2->D extended).
        pii = pointI.geometry
        lSeed = adsk.core.Point3D.create(
            pii.x + (pii.x - pb.x), pii.y + (pii.y - pb.y), 0)
        lineIL = lines.addByTwoPoints(
            adsk.core.Point3D.create(pii.x, pii.y, 0), lSeed)
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)
        pl = pointL.geometry
        lineDL = lines.addByTwoPoints(
            adsk.core.Point3D.create(pcD.x, pcD.y, 0),
            adsk.core.Point3D.create(pl.x, pl.y, 0))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        (pointLp, lineDLp) = self._buildToothCenter(
            sketch, constraints, dimensions, lines,
            pointL, pointD, drivingDedendum)

        # ----- Maximum Face Width from SOLVED geometry, then resolve Face Width.
        maxFaceWidth = self._maxFaceWidth(
            pointA, pointB, pointC, pointD, pointH, pointJ)
        if self._faceWidth_cm > 0:
            faceWidth = self._faceWidth_cm
            if faceWidth > maxFaceWidth:
                raise Exception(
                    f'Face Width {to_mm(faceWidth):.3f}mm exceeds the maximum '
                    f'{to_mm(maxFaceWidth):.3f}mm for this gear')
        else:
            faceWidth = min(self._coneDistance_cm / 6, maxFaceWidth)
        self._faceWidthResolved_cm = faceWidth

        # ----- M->N (pinion toe line).
        (pointM, pointN) = self._buildToeLine(
            sketch, constraints, dimensions, lines,
            pinionRootAxis, aApex2, lineCH, faceWidth,
            pointA, pointC, pointH, apexPoint)

        # ----- O->P (driving toe line).
        (pointO, pointP) = self._buildToeLine(
            sketch, constraints, dimensions, lines,
            drivingRootAxis, bApex2, lineDJ, faceWidth,
            pointB, pointD, pointJ, apexPoint)
        # B->I line.
        lineBI = lines.addByTwoPoints(
            adsk.core.Point3D.create(pb.x, pb.y, 0),
            adsk.core.Point3D.create(pii.x, pii.y, 0))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch ended under-constrained')

        sketch.isVisible = False

        # Collect the anchors that the per-gear helpers need.
        anchors = {
            'gpPlane': gpPlane,
            'gpSketch': sketch,
            'apexPoint': apexPoint,
            'apex2Point': apex2Point,
            'A': pointA, 'B': pointB, 'C': pointC, 'D': pointD,
            'G': pointG, 'H': pointH, 'I': pointI, 'J': pointJ,
            'K': pointK, 'L': pointL, 'Kp': pointKp, 'Lp': pointLp,
            'M': pointM, 'N': pointN, 'O': pointO, 'P': pointP,
            'lineCKp': lineCKp, 'lineDLp': lineDLp,
            'pinionRootAxis': pinionRootAxis,
            'drivingRootAxis': drivingRootAxis,
        }

        # §3 + body creation, pinion first then driving.
        pinionSpur = self._buildVirtualSpurProfile(
            bevelComponent, designComponent, anchors, 'Pinion',
            pointKp, anchors['lineCKp'], self._gamma_p, self._pinionPitchDiameter_cm,
            self._pinionTeeth)
        self._createGearBody(
            bevelComponent, designComponent, anchors, 'Pinion', pinionSpur,
            ('A', 'G', 'H', 'C', 'M', 'N'),
            ('M', 'N'), ('C', 'H'),
            self._pinionTeeth, self._pinionBoreResolved_cm, self._pinionPitchDiameter_cm,
            self._gamma_p)

        drivingSpur = self._buildVirtualSpurProfile(
            bevelComponent, designComponent, anchors, 'Driving',
            pointLp, anchors['lineDLp'], self._gamma_g, self._drivingPitchDiameter_cm,
            self._drivingTeeth)
        self._createGearBody(
            bevelComponent, designComponent, anchors, 'Driving', drivingSpur,
            ('B', 'I', 'J', 'D', 'O', 'P'),
            ('O', 'P'), ('D', 'J'),
            self._drivingTeeth, self._drivingBoreResolved_cm, self._drivingPitchDiameter_cm,
            self._gamma_g)

    # ------------------------------------------------------- §2 sub-builders
    def _buildToothCenter(self, sketch, constraints, dimensions, lines,
                          pointK, lowerCorner, dedendumLine):
        # K' shifted outward along dedendum, away from lowerCorner. 0 -> reuse.
        if self._toothSpacing_cm <= 0:
            lc = lowerCorner.geometry
            pk = pointK.geometry
            refLine = lines.addByTwoPoints(
                adsk.core.Point3D.create(lc.x, lc.y, 0),
                adsk.core.Point3D.create(pk.x, pk.y, 0))
            refLine.isConstruction = True
            constraints.addCoincident(refLine.startSketchPoint, lowerCorner)
            constraints.addCoincident(refLine.endSketchPoint, pointK)
            return (pointK, refLine)

        lc = lowerCorner.geometry
        pk = pointK.geometry
        ddx = pk.x - lc.x
        ddy = pk.y - lc.y
        dl = math.hypot(ddx, ddy)
        ux = ddx / dl
        uy = ddy / dl
        seed = adsk.core.Point3D.create(
            pk.x + ux * self._toothSpacing_cm, pk.y + uy * self._toothSpacing_cm, 0)
        spacingLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(pk.x, pk.y, 0), seed)
        spacingLine.isConstruction = True
        constraints.addCoincident(spacingLine.startSketchPoint, pointK)
        pointKp = spacingLine.endSketchPoint
        constraints.addCoincident(pointKp, dedendumLine)
        spacingDim = dimensions.addDistanceDimension(
            spacingLine.startSketchPoint, spacingLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(seed.x + 0.1, seed.y + 0.1, 0))
        spacingDim.parameter.value = self._toothSpacing_cm

        kp = pointKp.geometry
        refLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(lc.x, lc.y, 0),
            adsk.core.Point3D.create(kp.x, kp.y, 0))
        refLine.isConstruction = True
        constraints.addCoincident(refLine.startSketchPoint, lowerCorner)
        constraints.addCoincident(refLine.endSketchPoint, pointKp)
        return (pointKp, refLine)

    def _maxFaceWidth(self, A, B, C, D, H, J):
        # 0.95 * smaller of perpendicular distances:
        #  A -> line(C,H) ; B -> line(D,J), all from SOLVED geometry.
        def perpDist(p, l0, l1):
            px, py = p.geometry.x, p.geometry.y
            x0, y0 = l0.geometry.x, l0.geometry.y
            x1, y1 = l1.geometry.x, l1.geometry.y
            dx = x1 - x0
            dy = y1 - y0
            dl = math.hypot(dx, dy)
            if dl == 0:
                return 0.0
            return abs((px - x0) * dy - (py - y0) * dx) / dl

        distA = perpDist(A, C, H)
        distB = perpDist(B, D, J)
        return 0.95 * min(distA, distB)

    def _buildToeLine(self, sketch, constraints, dimensions, lines,
                     rootAxis, dropLine, heelLine, faceWidth,
                     dropEndPoint, coneCorner, heelCorner, apexPoint):
        # M near midpoint of Apex->coneCorner; N slid along cone->heel toward drop.
        cc = coneCorner.geometry
        ap = apexPoint.geometry
        mSeed = adsk.core.Point3D.create(
            (ap.x + cc.x) / 2, (ap.y + cc.y) / 2, 0)
        # C->H direction
        hc = heelCorner.geometry
        hdx = hc.x - cc.x
        hdy = hc.y - cc.y
        hdl = math.hypot(hdx, hdy)
        hux = hdx / hdl
        huy = hdy / hdl
        de = dropEndPoint.geometry
        slide = math.hypot(de.x - mSeed.x, de.y - mSeed.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + hux * slide, mSeed.y + huy * slide, 0)
        toeLine = lines.addByTwoPoints(mSeed, nSeed)
        toeLine.isConstruction = True
        pointM = toeLine.startSketchPoint
        pointN = toeLine.endSketchPoint
        constraints.addCoincident(pointM, rootAxis)
        constraints.addCoincident(pointN, dropLine)
        constraints.addParallel(toeLine, heelLine)
        textPt = adsk.core.Point3D.create(
            (mSeed.x + nSeed.x) / 2 + 0.1, (mSeed.y + nSeed.y) / 2 + 0.1, 0)
        offDim = dimensions.addOffsetDimension(heelLine, toeLine, textPt)
        offDim.parameter.value = faceWidth

        # M->coneCorner and N->dropEnd lines.
        m = pointM.geometry
        lineMC = lines.addByTwoPoints(
            adsk.core.Point3D.create(m.x, m.y, 0),
            adsk.core.Point3D.create(cc.x, cc.y, 0))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, coneCorner)
        n = pointN.geometry
        lineNA = lines.addByTwoPoints(
            adsk.core.Point3D.create(n.x, n.y, 0),
            adsk.core.Point3D.create(de.x, de.y, 0))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, dropEndPoint)
        return (pointM, pointN)

    # ------------------------------------------------ §3 virtual spur profile
    def _buildVirtualSpurProfile(self, bevelComponent, designComponent, anchors,
                                gearLabel, toothCenter, refLine, gamma, pitchDiameter_cm,
                                teeth):
        gpPlane = anchors['gpPlane']

        # 1. virtual tooth number (closed form).
        virtualPitchRadius_mm = (to_mm(pitchDiameter_cm) / 2) / math.cos(gamma)
        virtualTeeth = int(math.floor(2 * virtualPitchRadius_mm / self._module_mm))

        # 2. plane including refLine, perpendicular to Gear Profiles plane.
        toothPlane = self._planeByAngle(designComponent, refLine, gpPlane, 90)

        # 3. spur tooth profile at the tooth-center point.
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        toothSketch.isVisible = True
        proxy = _VirtualSpurProxy(module_mm=self._module_mm, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCenter, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded
        if not toothSketch.isFullyConstrained:
            futil.log(f'{gearLabel} Tooth sketch not fully constrained (expected for embedded)')

        # 4. construction axis through the tooth-center, normal to the tooth plane.
        helperPlanes = designComponent.constructionPlanes
        hpi = helperPlanes.createInput()
        hpi.setByDistanceOnPath(refLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = helperPlanes.add(hpi)
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gpPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)

        toothSketch.isVisible = False
        return {
            'toothSketch': toothSketch,
            'toothPlane': toothPlane,
            'toothAxis': toothAxis,
            'embedded': embedded,
            'virtualTeeth': virtualTeeth,
        }

    def _findSpurToothProfile(self, toothSketch, embedded):
        # tooth loop = 2 NURBS flanks + 2 arcs + wantLines lines.
        wantLines = 0 if embedded else 2
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nurbs = 0
                arcs = 0
                linesC = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        linesC += 1
                if nurbs == 2 and arcs == 2 and linesC == wantLines:
                    return profile
        raise Exception(
            f'Could not find spur tooth profile (embedded={embedded}, '
            f'wanted 2 NURBS + 2 arcs + {wantLines} lines)')

    # ------------------------------------------------------ body creation
    def _createGearBody(self, bevelComponent, designComponent, anchors, gearLabel,
                       spurData, hexVertexNames, toeEdgeNames, heelEdgeNames,
                       teeth, boreDiameter_cm, pitchDiameter_cm, gamma):
        gpPlane = anchors['gpPlane']
        apexPoint = anchors['apexPoint']
        apexWorld = self._pointWorldGeometry(apexPoint)

        # Output gear component (child of Bevel Gear).
        matrix = adsk.core.Matrix3D.create()
        gearOccurrence = bevelComponent.occurrences.addNewComponent(matrix)
        gearComponent = gearOccurrence.component
        gearComponent.name = f'{gearLabel} Gear'

        # Profile sketch (per gear) on the axial Gear Profiles plane.
        profileSketch = designComponent.sketches.add(gpPlane)
        profileSketch.name = f'{gearLabel} Profile'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        # Recreate the six §2 vertices as fixed points (recreate-share-fix).
        srcPoints = [anchors[name] for name in hexVertexNames]
        verts = []
        for src in srcPoints:
            local = profileSketch.modelToSketchSpace(self._pointWorldGeometry(src))
            verts.append(profileSketch.sketchPoints.add(local))

        hexLines = []
        for i in range(len(verts)):
            ln = lines.addByTwoPoints(verts[i], verts[(i + 1) % len(verts)])
            hexLines.append(ln)
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Profile sketch ended under-constrained')

        # The shaft-axis edge is the hexagon's FIRST edge.
        shaftAxisEdge = hexLines[0]

        # Revolve the single profile around the shaft-axis edge.
        revolveProfile = profileSketch.profiles.item(0)
        revolves = designComponent.features.revolveFeatures
        rInput = revolves.createInput(
            revolveProfile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        rInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveFeature = revolves.add(rInput)
        gearBody = revolveFeature.bodies.item(0)

        # Tooth loft: §2 Apex sketch point -> §3 tooth profile.
        toothProfile = self._findSpurToothProfile(
            spurData['toothSketch'], spurData['embedded'])
        lofts = designComponent.features.loftFeatures
        lInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        lInput.loftSections.add(apexPoint)
        lInput.loftSections.add(toothProfile)
        loftFeature = lofts.add(lInput)
        toothBody = loftFeature.bodies.item(0)

        # Build the toe/heel hand-off args per §3a Caller hand-off.
        toeStart = self._pointWorldGeometry(anchors[toeEdgeNames[0]])
        toeEnd = self._pointWorldGeometry(anchors[toeEdgeNames[1]])
        heelStart = self._pointWorldGeometry(anchors[heelEdgeNames[0]])
        heelEnd = self._pointWorldGeometry(anchors[heelEdgeNames[1]])
        toeMid = self._bottomEdgeMid(toeStart, toeEnd)
        heelMid = self._bottomEdgeMid(heelStart, heelEnd)
        toeConeWorld = toeStart       # M (pinion) / O (driving)
        heelConeWorld = heelStart     # C (pinion) / D (driving)

        # Tooth-body step (straight conical trim, or spiral build for psi > 0).
        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            apexPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
            spurData['toothPlane'], gearLabel, teeth)

        # Circular pattern around the shaft-axis edge.
        patterns = designComponent.features.circularPatternFeatures
        toothColl = adsk.core.ObjectCollection.create()
        toothColl.add(toothBody)
        pInput = patterns.createInput(toothColl, shaftAxisEdge)
        pInput.quantity = adsk.core.ValueInput.createByReal(teeth)
        pInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternFeature = patterns.add(pInput)

        # Combine-join all patterned teeth into the gear body.
        combines = designComponent.features.combineFeatures
        toolBodies = adsk.core.ObjectCollection.create()
        for b in patternFeature.bodies:
            toolBodies.add(b)
        cInput = combines.createInput(gearBody, toolBodies)
        cInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(cInput)

        # Bore.
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, boreDiameter_cm)

        # Meshing rotation (driving only) about the shaft-axis edge.
        if gearLabel == 'Driving':
            self._meshRotate(designComponent, gearBody, shaftAxisEdge,
                             math.radians(180.0 / teeth))
        else:
            phase = self._pinionMeshPhase()
            if phase != 0:
                self._meshRotate(designComponent, gearBody, shaftAxisEdge, phase)

        profileSketch.isVisible = False

        # Move the finished body into the per-gear component.
        gearBody.moveToComponent(gearOccurrence)
        return gearBody

    def _pinionMeshPhase(self):
        # Extra pinion mesh rotation (about its shaft axis). 0 by default.
        if self._PINION_MESH_PHASE_TEETH == 0:
            return 0
        return self._PINION_MESH_PHASE_TEETH * (2 * math.pi / self._pinionTeeth)

    def _meshRotate(self, designComponent, gearBody, shaftAxisEdge, angle):
        p0 = shaftAxisEdge.startSketchPoint.worldGeometry
        p1 = shaftAxisEdge.endSketchPoint.worldGeometry
        axisVec = adsk.core.Vector3D.create(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
        axisVec.normalize()
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angle, axisVec, p0)
        moves = designComponent.features.moveFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(gearBody)
        mInput = moves.createInput2(coll)
        mInput.defineAsFreeMove(matrix)
        moves.add(mInput)

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, boreDiameter_cm):
        planes = designComponent.constructionPlanes
        pi = planes.createInput()
        pi.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = planes.add(pi)
        sketch = designComponent.sketches.add(borePlane)
        sketch.name = 'Bore'
        sketch.isVisible = True
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2)
        circle.centerSketchPoint.isFixed = True
        sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter_cm / 2, 0, 0)
        ).parameter.value = boreDiameter_cm
        if not sketch.isFullyConstrained:
            raise Exception('Bore sketch ended under-constrained')

        boreProfile = sketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        eInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        largeDist = max(self._coneDistance_cm, 1.0) * 4
        eInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(largeDist), False)
        eInput.participantBodies = [gearBody]
        extrudes.add(eInput)
        sketch.isVisible = False

    # ----------------------------------------------------- tooth-body hook
    def _transformToothBody(self, designComponent, toothBody, gearBody, shaftAxisEdge,
                           apexWorld, apexSketchPoint, toeMid, heelMid, toeConeWorld,
                           heelConeWorld, parentToothPlane, gearLabel, teethNumber):
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
        return self._buildSpiralTooth(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane,
            gearLabel, teethNumber)

    # ----------------------------------------------------- straight conical trim
    def _cutConicalEnds(self, designComponent, toothBody, gearBody,
                       toeMid, heelMid, apexWorld, gearLabel):
        history = []
        # First the toe cut, then the heel cut on the keeper alone.
        keeper = self._applyConicalCut(
            designComponent, toothBody, gearBody, toeMid, apexWorld,
            gearLabel, 'toe', history, lenient=False)
        keeper = self._applyConicalCut(
            designComponent, keeper, gearBody, heelMid, apexWorld,
            gearLabel, 'heel', history, lenient=True)
        return keeper

    def _findConeFaces(self, gearBody):
        coneFaces = []
        for face in gearBody.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                coneFaces.append(face)
        return coneFaces

    def _surfaceDistance(self, surface, worldPoint):
        try:
            evaluator = surface.evaluator
            (ok, param) = evaluator.getParameterAtPoint(worldPoint)
            if not ok:
                return float('inf')
            (ok2, projected) = evaluator.getPointAtParameter(param)
            if not ok2:
                return float('inf')
            return worldPoint.distanceTo(projected)
        except Exception:
            return float('inf')

    def _pieceContainsApex(self, body, apexWorld):
        containment = body.pointContainment(apexWorld)
        return containment == adsk.fusion.PointContainment.PointInsidePointContainment or \
            containment == adsk.fusion.PointContainment.PointOnPointContainment

    def _applyConicalCut(self, designComponent, targetBody, gearBody, edgeMidWorld,
                        apexWorld, gearLabel, cutName, history, lenient):
        # Order cone faces best-first by surface distance to the edge midpoint.
        ranked = self._findConeFaceForCutLine(gearBody, edgeMidWorld)
        splits = designComponent.features.splitBodyFeatures

        pieces = None
        selectedDist = None
        faceErrors = []
        for face in ranked:
            try:
                sInput = splits.createInput(targetBody, face, True)
                splitFeature = splits.add(sInput)
                if splitFeature.bodies.count > 1:
                    pieces = [splitFeature.bodies.item(i)
                              for i in range(splitFeature.bodies.count)]
                    selectedDist = self._surfaceDistance(face.geometry, edgeMidWorld)
                    break
            except RuntimeError as e:
                faceErrors.append(str(e))
                continue

        if pieces is None:
            distList = [self._surfaceDistance(f.geometry, edgeMidWorld) for f in ranked]
            msg = (f'{gearLabel}: {cutName} cut found no cone face that splits the '
                   f'tooth body (faces found={len(ranked)}, dists={distList}, '
                   f'face errors={faceErrors}); history={history}')
            if lenient:
                # If every attempt was a non-intersection, keep the keeper whole.
                joined = ' '.join(faceErrors)
                if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in joined or '交差' in joined:
                    futil.log(
                        f'{gearLabel}: {cutName} cut tool did not intersect, kept intact',
                        force_console=True)
                    history.append(f'{cutName}: not intersecting, kept whole')
                    return targetBody
            raise RuntimeError(msg)

        futil.log(
            f'{gearLabel}: {cutName} cut 1 body in -> {len(pieces)} pieces '
            f'(selected dist={selectedDist})', force_console=True)
        history.append(f'{cutName}: {len(pieces)} pieces (dist={selectedDist})')

        # Keeper selection: drop apex pieces, keep the single largest by volume.
        nonApex = [p for p in pieces if not self._pieceContainsApex(p, apexWorld)]
        removes = designComponent.features.removeFeatures
        for p in pieces:
            if self._pieceContainsApex(p, apexWorld):
                removes.add(p)
        if len(nonApex) == 0:
            raise RuntimeError(
                f'{gearLabel}: {cutName} cut left no non-apex piece; history={history}')
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for p in nonApex[1:]:
            removes.add(p)
        return keeper

    # ----------------------------------------------------- spiral tooth body
    def _buildSpiralTooth(self, designComponent, toothBody, gearBody, shaftAxisEdge,
                         apexWorld, toeMid, heelMid, toeConeWorld, heelConeWorld,
                         parentToothPlane, gearLabel, teethNumber):
        apex = apexWorld

        # A. Gate & frame.
        s0 = shaftAxisEdge.startSketchPoint.worldGeometry
        s1 = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = adsk.core.Vector3D.create(s1.x - s0.x, s1.y - s0.y, s1.z - s0.z)
        axisDir.normalize()

        # Fix swapped toe/heel: heel must be the OUTER end.
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = adsk.core.Vector3D.create(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y, heelConeWorld.z - apex.z)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        # tangent-plane normal (unused directly but kept per spec frame)
        tpNormal = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            return (p.x - apex.x) * coneVec.x + (p.y - apex.y) * coneVec.y + \
                (p.z - apex.z) * coneVec.z

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        gamma = self._gamma_p if gearLabel == 'Pinion' else self._gamma_g

        # B. Cutter-arc geometry.
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1 if self._hand == _HAND_RIGHT else -1
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)
        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = self._circleIntersectNearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        # C. 2-D trace sketch (the cutter arc).
        coneElemSketch = designComponent.sketches.add(self._gearProfilesPlaneFor(designComponent))
        # cone-element line on the axial plane
        coneElemSketch.name = f'{gearLabel} Cone Element'
        coneElemSketch.isVisible = True
        heelWorld = self._combine(apex, R_heel, coneVec)
        ceLine = coneElemSketch.sketchCurves.sketchLines.addByTwoPoints(
            coneElemSketch.modelToSketchSpace(apex),
            coneElemSketch.modelToSketchSpace(heelWorld))
        ceLine.isConstruction = True

        tangentPlane = self._planeByAngle(
            designComponent, ceLine, coneElemSketch.referencePlane, 90)
        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        traceSketch.isVisible = True

        cutterCenterW = tanW(Cx, Cy)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            traceSketch.modelToSketchSpace(cutterCenterW), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle,
            traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy))).parameter.value = 2 * r_c

        toeW = tanW(toe2d[0], toe2d[1])
        meanW = tanW(R_mean, 0)
        heelW = tanW(heel2d[0], heel2d[1])
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            traceSketch.modelToSketchSpace(toeW),
            traceSketch.modelToSketchSpace(meanW),
            traceSketch.modelToSketchSpace(heelW))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        traceSketch.sketchDimensions.addRadialDimension(
            traceArc, traceSketch.modelToSketchSpace(meanW)).parameter.value = r_c

        # E. Slice the straight tooth.
        segments = self._sliceTooth(
            designComponent, toothBody, parentToothPlane, apex, coneVec, span,
            distAlong, gearLabel)

        # F. Order & drop scrap.
        segments.sort(
            key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise RuntimeError(
                f'{gearLabel}: slice left no working segments after dropping scrap')

        # G. Twist (the spiral).
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        def slabHeelFace(seg):
            best = None
            bestD = None
            for face in seg.faces:
                d = distAlong(face.centroid)
                if bestD is None or d > bestD:
                    bestD = d
                    best = face
            return best

        def R_heelFace(seg):
            return distAlong(slabHeelFace(seg).centroid)

        moves = designComponent.features.moveFeatures
        for seg in segments:
            ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apex)
            coll = adsk.core.ObjectCollection.create()
            coll.add(seg)
            mInput = moves.createInput2(coll)
            mInput.defineAsFreeMove(matrix)
            moves.add(mInput)

        # H. Lengthwise crown (relief).
        self._crownSegments(
            designComponent, segments, slabHeelFace, distAlong, R_heel, span,
            total, axisDir, apex)

        # I. Loft -> curved tooth.
        order = sorted(
            range(len(segments)),
            key=lambda i: distAlong(slabHeelFace(segments[i]).centroid))

        def slabToeFace(seg):
            best = None
            bestD = None
            for face in seg.faces:
                d = distAlong(face.centroid)
                if bestD is None or d < bestD:
                    bestD = d
                    best = face
            return best

        lofts = designComponent.features.loftFeatures
        lInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        lInput.loftSections.add(slabToeFace(segments[order[0]]))
        for i in order:
            lInput.loftSections.add(slabHeelFace(segments[i]))
        loftFeature = lofts.add(lInput)
        curvedTooth = loftFeature.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        # Remove the segment scaffolding.
        removes = designComponent.features.removeFeatures
        for seg in segments:
            removes.add(seg)

        # cleanup transient spiral sketches.
        coneElemSketch.isVisible = False
        traceSketch.isVisible = False

        # J. Flush trim.
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apex, gearLabel)

    def _gearProfilesPlaneFor(self, designComponent):
        # The Gear Profiles (axial) plane stashed on self during §2.
        return self._gpPlaneRef

    def _sliceTooth(self, designComponent, toothBody, parentToothPlane, apex,
                   coneVec, span, distAlong, gearLabel):
        planeNormal = get_normal(parentToothPlane)
        planeOrigin = parentToothPlane.geometry.origin

        def sliceWithSign(sign):
            planes = designComponent.constructionPlanes
            cutPlanes = []
            for k in range(8):
                offset = sign * (k + 1) * span / 6
                pi = planes.createInput()
                pi.setByOffset(
                    parentToothPlane, adsk.core.ValueInput.createByReal(offset))
                cutPlanes.append(planes.add(pi))
            pieces = [toothBody]
            splits = designComponent.features.splitBodyFeatures
            for plane in cutPlanes:
                newPieces = []
                for piece in pieces:
                    try:
                        sInput = splits.createInput(piece, plane, True)
                        sf = splits.add(sInput)
                        if sf.bodies.count > 1:
                            for j in range(sf.bodies.count):
                                newPieces.append(sf.bodies.item(j))
                        else:
                            newPieces.append(piece)
                    except RuntimeError:
                        newPieces.append(piece)
                pieces = newPieces
            return pieces

        # Choose sign so the offset moves toward the apex.
        toApex = adsk.core.Vector3D.create(
            apex.x - planeOrigin.x, apex.y - planeOrigin.y, apex.z - planeOrigin.z)
        dot = toApex.x * planeNormal.x + toApex.y * planeNormal.y + toApex.z * planeNormal.z
        sign = 1 if dot > 0 else -1

        pieces = sliceWithSign(sign)
        if len(pieces) <= 1:
            pieces = sliceWithSign(-sign)
        if len(pieces) <= 1:
            raise RuntimeError(
                f'{gearLabel}: slice did not split the tooth (pieces={len(pieces)}, '
                f'span={span}, sign tried={sign} and {-sign})')
        return pieces

    def _crownSegments(self, designComponent, segments, slabHeelFace, distAlong,
                      R_heel, span, total, axisDir, apex):
        if self._CROWN_PER_RAD == 0:
            return
        # Sort by heel-face distance; the outermost (heel) is held full.
        ordered = sorted(
            segments, key=lambda s: distAlong(slabHeelFace(s).centroid))
        heelSeg = ordered[-1]

        designOccurrence = self._designOccurrence
        designOccurrence.activate()
        try:
            scales = designComponent.features.scaleFeatures
            for seg in ordered:
                if seg is heelSeg:
                    continue
                heelFace = slabHeelFace(seg)
                R_heelFace = distAlong(heelFace.centroid)
                u = (R_heel - R_heelFace) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2) * u
                if factor >= 1.0 or factor <= 0:
                    continue
                basePoint = self._heelRootMidPoint(heelFace, apex, axisDir)
                coll = adsk.core.ObjectCollection.create()
                coll.add(seg)
                sInput = scales.createInput(
                    coll, basePoint, adsk.core.ValueInput.createByReal(factor))
                scales.add(sInput)
        finally:
            self.design.activateRootComponent()

    def _heelRootMidPoint(self, heelFace, apex, axisDir):
        # On the heel face, the two vertices nearest the shaft axis are the root
        # corners; place a sketch point at their midpoint.
        verts = [vtx.geometry for vtx in heelFace.vertices]
        verts.sort(key=lambda p: self._perpToAxis(p, apex, axisDir))
        root1 = verts[0]
        root2 = verts[1]
        mid = adsk.core.Point3D.create(
            (root1.x + root2.x) / 2, (root1.y + root2.y) / 2, (root1.z + root2.z) / 2)
        # Map into a sketch on the heel face.
        comp = self._designComponent
        sketch = comp.sketches.add(heelFace)
        sketch.isVisible = False
        return sketch.sketchPoints.add(sketch.modelToSketchSpace(mid))

    # ------------------------------------------------------------- cleanup
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            if component.entityToken in seen:
                return
            seen.add(component.entityToken)
            for sketch in component.sketches:
                sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None
