import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts, get_normal
from . import solids
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy


# ---------------------------------------------------------------------------
# S1: dialog input ids and dropdown item labels. Reproduced verbatim.
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
INPUT_ID_TOOTH_SPACING = 'toothSpacing'
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
INPUT_ID_TOE_EXTENSION = 'toeExtension'
INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'
INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'


# ---------------------------------------------------------------------------
# Plain 2-D helpers used throughout the S6 lattice. All of the S6 geometry is
# sketch-local, so it is cheaper and clearer to work in (x, y) float tuples
# and wrap only at the point of a Fusion call than to carry Vector3D/Point3D
# objects through every closed-form step.
# ---------------------------------------------------------------------------

def _pt2(xy):
    return adsk.core.Point3D.create(xy[0], xy[1], 0.0)


def _v2_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _v2_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _v2_scale(a, s):
    return (a[0] * s, a[1] * s)


def _v2_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _v2_len(a):
    return math.hypot(a[0], a[1])


def _v2_unit(a):
    n = _v2_len(a)
    return (a[0] / n, a[1] / n)


def _v2_perp(a):
    return (-a[1], a[0])


def _v2_rotate(v, theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def _v2_lerp(a, b, t):
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def _v2_mid(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _line_intersect2(p1, d1, p2, d2):
    """Intersection of the infinite lines p1 + t*d1 and p2 + s*d2."""
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    t = (dx * d2[1] - dy * d2[0]) / denom
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def _perp_dist_from_line2(p, linePt, lineDirUnit):
    v = _v2_sub(p, linePt)
    return abs(v[0] * lineDirUnit[1] - v[1] * lineDirUnit[0])


def _point_line_dist2(p, a, b):
    ab = _v2_sub(b, a)
    ap = _v2_sub(p, a)
    abLen = _v2_len(ab)
    cross = ab[0] * ap[1] - ab[1] * ap[0]
    return abs(cross) / abLen


def _text_point2(p0, p1, nudge=0.2):
    mid = _v2_mid(p0, p1)
    d = _v2_sub(p1, p0)
    length = _v2_len(d)
    if length < 1e-9:
        perp = (0.0, 1.0)
    else:
        perp = _v2_perp(_v2_scale(d, 1.0 / length))
    return _pt2(_v2_add(mid, _v2_scale(perp, nudge)))


def _geom2(sketchPoint):
    g = sketchPoint.geometry
    return (g.x, g.y)


def _midpoint3d(a, b):
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


class BevelGearCommandInputsConfigurator:
    # S1: dialog inputs, in the contract row order.
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
            adsk.core.ValueInput.createByReal(0))

        inputs.addValueInput(
            INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addValueInput(
            INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    # S1: spiral-only inputs (Hand of Spiral, Cutter Radius) show only when
    # Mean Spiral Angle is above zero.
    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return
        try:
            design: adsk.fusion.Design = get_design()
            value = design.unitsManager.evaluateExpression(spiral.expression, 'rad')
            visible = value > 0
        except Exception:
            visible = True
        hand.isVisible = visible
        cutter.isVisible = visible

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


class BevelGearGenerator:
    # S15: tuning constants read through self from inside the tooth-body build.
    _CROWN_PER_RAD = 0.5
    _PINION_MESH_PHASE_TEETH = 0.0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None
        self.bevelComponent = None
        self.designOccurrence = None
        self.designComponent = None
        self._gearProfilesPlane = None
        self._anchorCenterPoint = None

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)

        module_cm = to_cm(module_mm)
        shaftAngle_rad = math.radians(shaftAngle_deg)

        self._createComponentTree(parentComponent)

        anchorLine = self._buildAnchorSketch(targetPlane, centerPoint)
        self._buildGearProfilesPlane(targetPlane, anchorLine)

        (pinionCtx, drivingCtx, apexSketchPoint, coneDistance_cm) = self._buildGearProfilesSketch(
            module_cm, pinionTeeth, drivingTeeth, shaftAngle_rad, targetPlane, anchorLine)

        for ctx in (pinionCtx, drivingCtx):
            self._createGearBody(ctx, module_cm, module_mm, apexSketchPoint, coneDistance_cm)

        # S30: cleanup.
        solids.hide_construction_geometry(self.bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence is not None:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = None
        self.bevelComponent = None
        self.designOccurrence = None
        self.designComponent = None

    # ------------------------------------------------------------------
    # S2: read and validate every input
    # ------------------------------------------------------------------

    @staticmethod
    def _coneGeometry(module_cm, pinionTeeth, drivingTeeth, shaftAngle_rad):
        ppd = module_cm * pinionTeeth
        dpd = module_cm * drivingTeeth
        tan_gamma_p = (math.sin(shaftAngle_rad) * ppd) / (dpd + ppd * math.cos(shaftAngle_rad))
        gamma_p = math.atan(tan_gamma_p)
        gamma_g = shaftAngle_rad - gamma_p
        R = (ppd / 2.0) / math.sin(gamma_p)
        return ppd, dpd, gamma_p, gamma_g, R

    def _evalExpr(self, inputs: adsk.core.CommandInputs, inputId, units):
        valueInput: adsk.core.ValueCommandInput = inputs.itemById(inputId)
        return self.design.unitsManager.evaluateExpression(valueInput.expression, units)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        targetPlane = get_selection(inputs, INPUT_ID_PLANE)[0]
        centerPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]
        selectedParent = get_selection(inputs, INPUT_ID_PARENT)[0]
        if selectedParent.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = selectedParent.component
        else:
            parentComponent = selectedParent

        module_mm = self._evalExpr(inputs, INPUT_ID_MODULE, '')
        shaftAngle_rad_raw = self._evalExpr(inputs, INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad_raw)

        drivingTeeth = int(round(self._evalExpr(inputs, INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(self._evalExpr(inputs, INPUT_ID_PINION_TEETH, '')))

        drivingBaseHeight_in_cm = self._evalExpr(inputs, INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeight_in_cm = self._evalExpr(inputs, INPUT_ID_PINION_BASE_HEIGHT, 'mm')

        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_in_cm = self._evalExpr(inputs, INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore_in_cm = self._evalExpr(inputs, INPUT_ID_PINION_BORE, 'mm')

        faceWidth_in_cm = self._evalExpr(inputs, INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing_cm = self._evalExpr(inputs, INPUT_ID_TOOTH_SPACING, 'mm')

        spiralAngle_rad = self._evalExpr(inputs, INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        hand = handItem.name if handItem is not None else _HAND_RIGHT

        cutterRadius_cm = self._evalExpr(inputs, INPUT_ID_CUTTER_RADIUS, 'mm')
        toeExtension_pct = self._evalExpr(inputs, INPUT_ID_TOE_EXTENSION, '')
        drivingToeRadius_in_cm = self._evalExpr(inputs, INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        pinionToeRadius_in_cm = self._evalExpr(inputs, INPUT_ID_PINION_TOE_RADIUS, 'mm')

        # --- Range checks, in order ---

        if module_mm <= 0:
            raise ValueError(f'Module must be greater than 0 (got {module_mm})')
        if pinionTeeth < 3:
            raise ValueError(f'Pinion Gear Teeth must be at least 3 (got {pinionTeeth})')
        if drivingTeeth < 3:
            raise ValueError(f'Driving Gear Teeth must be at least 3 (got {drivingTeeth})')
        if drivingBaseHeight_in_cm < 0:
            raise ValueError('Driving Gear Base Height must be non-negative')
        if pinionBaseHeight_in_cm < 0:
            raise ValueError('Pinion Gear Base Height must be non-negative')
        if drivingBore_in_cm < 0:
            raise ValueError('Driving Gear Bore Diameter must be non-negative')
        if pinionBore_in_cm < 0:
            raise ValueError('Pinion Gear Bore Diameter must be non-negative')
        if faceWidth_in_cm < 0:
            raise ValueError('Face Width must be non-negative')
        if toothSpacing_cm < 0:
            raise ValueError('Tooth Spacing must be non-negative')
        if cutterRadius_cm < 0:
            raise ValueError('Cutter Radius must be non-negative')
        if drivingToeRadius_in_cm < 0:
            raise ValueError('Driving Gear Toe Radius must be non-negative')
        if pinionToeRadius_in_cm < 0:
            raise ValueError('Pinion Gear Toe Radius must be non-negative')
        if not (0 <= toeExtension_pct <= 100):
            raise ValueError(
                f'Toe Extension must be between 0 and 100 percent (got {toeExtension_pct})')
        if not (0 <= spiralAngle_deg < 60):
            raise ValueError(
                'Mean Spiral Angle must be at least 0 and below 60 degrees '
                f'(got {spiralAngle_deg})')

        module_cm = to_cm(module_mm)

        ppd_for_shaft = module_cm * pinionTeeth
        dpd_for_shaft = module_cm * drivingTeeth
        maxShaftAngle_deg = min(
            150.0,
            math.degrees(math.acos(
                -min(ppd_for_shaft, dpd_for_shaft) / max(ppd_for_shaft, dpd_for_shaft))))
        if not (30.0 <= shaftAngle_deg <= maxShaftAngle_deg):
            raise ValueError(
                f'Shaft Angle must be between 30 and {maxShaftAngle_deg:.4f} degrees '
                f'(got {shaftAngle_deg})')

        sigma = math.radians(shaftAngle_deg)
        _, _, gamma_p, gamma_g, _ = self._coneGeometry(module_cm, pinionTeeth, drivingTeeth, sigma)

        pinionFloor = 5.27 * math.cos(gamma_p)
        if pinionTeeth < pinionFloor:
            raise ValueError(
                f'Pinion Gear Teeth must be at least {pinionFloor:.4f} for this configuration '
                f'(got {pinionTeeth})')
        drivingFloor = 5.27 * math.cos(gamma_g)
        if drivingTeeth < drivingFloor:
            raise ValueError(
                f'Driving Gear Teeth must be at least {drivingFloor:.4f} for this configuration '
                f'(got {drivingTeeth})')

        # Base heights, driving first.
        drivingPitchRadius = dpd_for_shaft / 2.0
        drivingMin = 1.05 * 1.25 * module_cm * math.sin(gamma_g)
        drivingMax = 0.95 * (
            drivingPitchRadius - 1.25 * module_cm * math.cos(gamma_g)) * math.tan(gamma_g)
        if drivingBaseHeight_in_cm == 0:
            drivingResolved = module_cm * drivingTeeth / 8.0
            if drivingResolved < drivingMin:
                drivingResolved = drivingMin
            elif drivingResolved > drivingMax:
                drivingResolved = drivingMax
        else:
            if drivingBaseHeight_in_cm < drivingMin:
                raise ValueError(
                    'Driving Gear Base Height must be at least '
                    f'{to_mm(drivingMin):.4f} mm (got {to_mm(drivingBaseHeight_in_cm):.4f} mm)')
            if drivingBaseHeight_in_cm > drivingMax:
                raise ValueError(
                    'Driving Gear Base Height must be at most '
                    f'{to_mm(drivingMax):.4f} mm (got {to_mm(drivingBaseHeight_in_cm):.4f} mm)')
            drivingResolved = drivingBaseHeight_in_cm
        self._drivingBaseHeight_cm = drivingResolved

        pinionPitchRadius = ppd_for_shaft / 2.0
        pinionMin = 1.05 * 1.25 * module_cm * math.sin(gamma_p)
        pinionMax = 0.95 * (
            pinionPitchRadius - 1.25 * module_cm * math.cos(gamma_p)) * math.tan(gamma_p)
        if pinionBaseHeight_in_cm == 0:
            pinionResolved = drivingResolved * pinionTeeth / drivingTeeth
            if pinionResolved < pinionMin:
                pinionResolved = pinionMin
            elif pinionResolved > pinionMax:
                pinionResolved = pinionMax
        else:
            if pinionBaseHeight_in_cm < pinionMin:
                raise ValueError(
                    'Pinion Gear Base Height must be at least '
                    f'{to_mm(pinionMin):.4f} mm (got {to_mm(pinionBaseHeight_in_cm):.4f} mm)')
            if pinionBaseHeight_in_cm > pinionMax:
                raise ValueError(
                    'Pinion Gear Base Height must be at most '
                    f'{to_mm(pinionMax):.4f} mm (got {to_mm(pinionBaseHeight_in_cm):.4f} mm)')
            pinionResolved = pinionBaseHeight_in_cm
        self._pinionBaseHeight_cm = pinionResolved

        # The bore bound is NOT part of this pass; only non-negativity above.
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBore_in_cm
        self._pinionBore_cm = pinionBore_in_cm
        self._faceWidth_cm = faceWidth_in_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm
        self._toeExtension_pct = toeExtension_pct
        self._drivingToeRadius_cm = drivingToeRadius_in_cm
        self._pinionToeRadius_cm = pinionToeRadius_in_cm

        return (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # ------------------------------------------------------------------
    # S3: component tree
    # ------------------------------------------------------------------

    def _createComponentTree(self, parentComponent: adsk.fusion.Component):
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelComponent = self.bevelOccurrence.component
        self.bevelComponent.name = 'Bevel Gear'

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designComponent = self.designOccurrence.component
        self.designComponent.name = 'Design'

    # ------------------------------------------------------------------
    # S4: Anchor sketch
    # ------------------------------------------------------------------

    def _buildAnchorSketch(self, targetPlane, centerPoint):
        sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint)[0]
        cx = projectedCenter.geometry.x
        cy = projectedCenter.geometry.y

        start = adsk.core.Point3D.create(cx - 0.5, cy, 0.0)
        end = adsk.core.Point3D.create(cx + 0.5, cy, 0.0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        textPoint = adsk.core.Point3D.create(cx, cy + 0.3, 0.0)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)

        sketch.geometricConstraints.addHorizontal(anchorLine)

        self._anchorCenterPoint = projectedCenter

        if not sketch.isFullyConstrained:
            raise RuntimeError('Anchor sketch is not fully constrained')

        return anchorLine

    # ------------------------------------------------------------------
    # S5: Gear Profiles plane
    # ------------------------------------------------------------------

    def _buildGearProfilesPlane(self, targetPlane, anchorLine):
        planeInput = self.designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = self.designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane
        return plane

    # ------------------------------------------------------------------
    # S6: Gear Profiles sketch — the shared lattice for both gears.
    #
    # All twenty-two named points are first computed as a pure closed-form
    # model in the sketch's own 2-D frame (this doubles as both the seed
    # source and the end-of-step gate's expected-position table), then the
    # Fusion sketch entities are created against those seeds in the order
    # the step list gives, with one exception: the Face Width / Root Length
    # / Toe Radius / Bore-bound resolution (steps 26-27) is computed as soon
    # as A, B, Apex2, C, D, H and J exist and are read back from solved
    # geometry, because the pinion's shaft-axis edge (step 22) needs the
    # Root Length before it can be seeded, even though it is numbered ahead
    # of the resolution steps in the step list's own walk order.
    # ------------------------------------------------------------------

    def _buildGearProfilesSketch(self, module_cm, pinionTeeth, drivingTeeth, shaftAngle_rad,
                                  targetPlane, anchorLine):
        sketch = self.designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        gc = sketch.geometricConstraints
        sd = sketch.sketchDimensions
        AlignedDim = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        ppd_cm, dpd_cm, gamma_p, gamma_g, R_cm = self._coneGeometry(
            module_cm, pinionTeeth, drivingTeeth, shaftAngle_rad)

        # Step 1: project the Anchor sketch's own centre point and line.
        cPoint = sketch.project(self._anchorCenterPoint)[0]
        projAnchorLine = sketch.project(anchorLine)[0]
        c = (cPoint.geometry.x, cPoint.geometry.y)
        aStart = (projAnchorLine.startSketchPoint.geometry.x,
                  projAnchorLine.startSketchPoint.geometry.y)
        aEnd = (projAnchorLine.endSketchPoint.geometry.x,
                projAnchorLine.endSketchPoint.geometry.y)
        d = _v2_unit(_v2_sub(aEnd, aStart))
        perp = _v2_perp(d)

        # [BEVEL-F-GROW-SIDE]: pick perp's sign from the target-plane normal.
        xDir = sketch.xDirection
        yDir = sketch.yDirection

        perpWorld = xDir.copy()
        perpWorld.scaleBy(perp[0])
        perpWorldY = yDir.copy()
        perpWorldY.scaleBy(perp[1])
        perpWorld.add(perpWorldY)

        normal = get_normal(targetPlane)
        if perpWorld.dotProduct(normal) < 0:
            perp = (-perp[0], -perp[1])

        # Step 2: centre -> Apex.
        apex = _v2_add(c, _v2_scale(perp, R_cm * math.cos(gamma_g) + self._drivingBaseHeight_cm))
        centerToApex = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(c), _pt2(apex))
        centerToApex.isConstruction = True
        gc.addCoincident(centerToApex.startSketchPoint, cPoint)
        gc.addPerpendicular(centerToApex, projAnchorLine)
        apexPoint = centerToApex.endSketchPoint

        # Step 3: Driving Gear Shaft Axis (Apex -> B).
        drivingDir = (-perp[0], -perp[1])
        ptB = _v2_add(apex, _v2_scale(drivingDir, R_cm * math.cos(gamma_g)))
        drivingShaftAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex), _pt2(ptB))
        drivingShaftAxis.isConstruction = True
        gc.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        gc.addParallel(drivingShaftAxis, centerToApex)
        bPoint = drivingShaftAxis.endSketchPoint

        # Step 4: Pinion Gear Shaft Axis (Apex -> A), pick the +X candidate.
        cand1Dir = _v2_rotate(drivingDir, shaftAngle_rad)
        cand2Dir = _v2_rotate(drivingDir, -shaftAngle_rad)
        cand1 = _v2_add(apex, _v2_scale(cand1Dir, R_cm * math.cos(gamma_p)))
        cand2 = _v2_add(apex, _v2_scale(cand2Dir, R_cm * math.cos(gamma_p)))
        if cand1[0] > cand2[0]:
            pinionDir, ptA = cand1Dir, cand1
        else:
            pinionDir, ptA = cand2Dir, cand2
        pinionShaftAxis = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex), _pt2(ptA))
        pinionShaftAxis.isConstruction = True
        gc.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        aPoint = pinionShaftAxis.endSketchPoint

        # Step 5: the Shaft Angle dimension.
        angleTextXY = _v2_add(apex, _v2_scale(_v2_unit(_v2_add(pinionDir, drivingDir)), ppd_cm / 4.0))
        angDim = sd.addAngularDimension(drivingShaftAxis, pinionShaftAxis, _pt2(angleTextXY))
        angDim.parameter.value = shaftAngle_rad

        # Step 6: A -> Apex2 drop, toward B.
        aToB = _v2_unit(_v2_sub(ptB, ptA))
        candA1 = (-pinionDir[1], pinionDir[0])
        candA2 = (pinionDir[1], -pinionDir[0])
        dropADir = candA1 if _v2_dot(candA1, aToB) > 0 else candA2
        apex2FromA = _v2_add(ptA, _v2_scale(dropADir, ppd_cm / 2.0))
        aApex2Drop = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptA), _pt2(apex2FromA))
        aApex2Drop.isConstruction = True
        gc.addCoincident(aApex2Drop.startSketchPoint, aPoint)
        gc.addPerpendicular(aApex2Drop, pinionShaftAxis)
        dimA = sd.addDistanceDimension(
            aApex2Drop.startSketchPoint, aApex2Drop.endSketchPoint, AlignedDim,
            _text_point2(ptA, apex2FromA))
        dimA.parameter.value = ppd_cm / 2.0

        # Step 7: B -> Apex2 drop, toward A.
        bToA = _v2_unit(_v2_sub(ptA, ptB))
        candB1 = (-drivingDir[1], drivingDir[0])
        candB2 = (drivingDir[1], -drivingDir[0])
        dropBDir = candB1 if _v2_dot(candB1, bToA) > 0 else candB2
        apex2FromB = _v2_add(ptB, _v2_scale(dropBDir, dpd_cm / 2.0))
        bApex2Drop = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptB), _pt2(apex2FromB))
        bApex2Drop.isConstruction = True
        gc.addCoincident(bApex2Drop.startSketchPoint, bPoint)
        gc.addPerpendicular(bApex2Drop, drivingShaftAxis)
        dimB = sd.addDistanceDimension(
            bApex2Drop.startSketchPoint, bApex2Drop.endSketchPoint, AlignedDim,
            _text_point2(ptB, apex2FromB))
        dimB.parameter.value = dpd_cm / 2.0

        # Step 8: close them.
        gc.addCoincident(aApex2Drop.endSketchPoint, bApex2Drop.endSketchPoint)
        apex2Point = aApex2Drop.endSketchPoint
        apex2 = apex2FromA

        # Step 10: Pitch Line.
        pitchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex), _pt2(apex2))
        pitchLine.isConstruction = True
        gc.addCoincident(pitchLine.startSketchPoint, apexPoint)
        gc.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # Step 11: the two dedendum lines.
        pitchLineDir = _v2_unit(_v2_sub(apex2, apex))
        candDed1 = (-pitchLineDir[1], pitchLineDir[0])
        candDed2 = (pitchLineDir[1], -pitchLineDir[0])
        if _v2_dot(candDed1, pinionDir) > 0:
            pinionDedDir, drivingDedDir = candDed1, candDed2
        else:
            pinionDedDir, drivingDedDir = candDed2, candDed1

        ptC = _v2_add(apex2, _v2_scale(pinionDedDir, 1.25 * module_cm))
        pinionDedendum = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex2), _pt2(ptC))
        pinionDedendum.isConstruction = True
        gc.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        gc.addPerpendicular(pinionDedendum, pitchLine)
        dimC = sd.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint, AlignedDim,
            _text_point2(apex2, ptC))
        dimC.parameter.value = 1.25 * module_cm
        cPoint2 = pinionDedendum.endSketchPoint

        ptD = _v2_add(apex2, _v2_scale(drivingDedDir, 1.25 * module_cm))
        drivingDedendum = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex2), _pt2(ptD))
        drivingDedendum.isConstruction = True
        gc.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        gc.addPerpendicular(drivingDedendum, pitchLine)
        dimD = sd.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint, AlignedDim,
            _text_point2(apex2, ptD))
        dimD.parameter.value = 1.25 * module_cm
        dPoint = drivingDedendum.endSketchPoint

        # Step 12: the two Root Axes.
        rootAxisPinion = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex), _pt2(ptC))
        rootAxisPinion.isConstruction = True
        gc.addCoincident(rootAxisPinion.startSketchPoint, apexPoint)
        gc.addCoincident(rootAxisPinion.endSketchPoint, cPoint2)

        rootAxisDriving = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(apex), _pt2(ptD))
        rootAxisDriving.isConstruction = True
        gc.addCoincident(rootAxisDriving.startSketchPoint, apexPoint)
        gc.addCoincident(rootAxisDriving.endSketchPoint, dPoint)

        # Step 13: point E.
        ptE = _v2_add(ptA, _v2_scale(pinionDir, 1.25 * module_cm * math.sin(gamma_p)))
        lineAE = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptA), _pt2(ptE))
        lineAE.isConstruction = True
        gc.addCoincident(lineAE.startSketchPoint, aPoint)
        gc.addCollinear(lineAE, pinionShaftAxis)
        ePoint = lineAE.endSketchPoint

        # Step 14: line C -> E.
        lineCE = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptC), _pt2(ptE))
        lineCE.isConstruction = True
        gc.addCoincident(lineCE.startSketchPoint, cPoint2)
        gc.addCoincident(lineCE.endSketchPoint, ePoint)
        gc.addPerpendicular(lineAE, lineCE)

        # Step 15: point F, driving twin of E.
        ptF = _v2_add(ptB, _v2_scale(drivingDir, 1.25 * module_cm * math.sin(gamma_g)))
        lineBF = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptB), _pt2(ptF))
        lineBF.isConstruction = True
        gc.addCoincident(lineBF.startSketchPoint, bPoint)
        gc.addCollinear(lineBF, drivingShaftAxis)
        fPoint = lineBF.endSketchPoint

        lineDF = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptD), _pt2(ptF))
        lineDF.isConstruction = True
        gc.addCoincident(lineDF.startSketchPoint, dPoint)
        gc.addCoincident(lineDF.endSketchPoint, fPoint)
        gc.addPerpendicular(lineBF, lineDF)

        # Step 16: point G.
        ptG = _v2_add(ptA, _v2_scale(pinionDir, self._pinionBaseHeight_cm))
        lineEG = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptE), _pt2(ptG))
        lineEG.isConstruction = True
        gc.addCoincident(lineEG.startSketchPoint, ePoint)
        gc.addCollinear(lineEG, lineAE)
        gPoint = lineEG.endSketchPoint

        # Step 17: point H.
        ptH = _v2_add(apex2, _v2_scale(pinionDedDir, self._pinionBaseHeight_cm / math.sin(gamma_p)))
        lineCH = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptC), _pt2(ptH))
        lineCH.isConstruction = True
        gc.addCoincident(lineCH.startSketchPoint, cPoint2)
        gc.addCollinear(lineCH, pinionDedendum)
        hPoint = lineCH.endSketchPoint

        # Step 18: line G -> H.
        lineGH = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptG), _pt2(ptH))
        lineGH.isConstruction = True
        gc.addCoincident(lineGH.startSketchPoint, gPoint)
        gc.addCoincident(lineGH.endSketchPoint, hPoint)
        gc.addPerpendicular(lineEG, lineGH)

        # Step 19: points I and J, driving twins of G and H.
        ptI = _v2_add(ptB, _v2_scale(drivingDir, self._drivingBaseHeight_cm))
        lineFI = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptF), _pt2(ptI))
        lineFI.isConstruction = True
        gc.addCoincident(lineFI.startSketchPoint, fPoint)
        gc.addCollinear(lineFI, lineBF)
        iPoint = lineFI.endSketchPoint

        ptJ = _v2_add(apex2, _v2_scale(drivingDedDir, self._drivingBaseHeight_cm / math.sin(gamma_g)))
        lineDJ = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptD), _pt2(ptJ))
        lineDJ.isConstruction = True
        gc.addCoincident(lineDJ.startSketchPoint, dPoint)
        gc.addCollinear(lineDJ, drivingDedendum)
        jPoint = lineDJ.endSketchPoint

        lineIJ = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptI), _pt2(ptJ))
        lineIJ.isConstruction = True
        gc.addCoincident(lineIJ.startSketchPoint, iPoint)
        gc.addCoincident(lineIJ.endSketchPoint, jPoint)
        gc.addPerpendicular(lineFI, lineIJ)

        # Step 20: driving base-height offset.
        dimDrivingOffset = sd.addOffsetDimension(
            bApex2Drop, lineIJ, _text_point2(apex2FromB, ptJ))
        dimDrivingOffset.parameter.value = self._drivingBaseHeight_cm

        # Step 21: pinion base-height offset.
        dimPinionOffset = sd.addOffsetDimension(
            aApex2Drop, lineGH, _text_point2(apex2FromA, ptG))
        dimPinionOffset.parameter.value = self._pinionBaseHeight_cm

        # Step 23: close the last freedom (Apex's distance along perp).
        gc.addCoincident(iPoint, cPoint)

        # Read SOLVED geometry now that only the intended shape freedoms
        # remain ([PB-SOLVED-GEOMETRY]); the resolution below needs it.
        apex = _geom2(apexPoint)
        ptA = _geom2(aPoint)
        ptB = _geom2(bPoint)
        apex2 = _geom2(apex2Point)
        ptC = _geom2(cPoint2)
        ptD = _geom2(dPoint)
        ptH = _geom2(hPoint)
        ptJ = _geom2(jPoint)

        # Step 26: resolve the Maximum Face Width and apply it.
        distA_CH = _point_line_dist2(ptA, ptC, ptH)
        distB_DJ = _point_line_dist2(ptB, ptD, ptJ)
        maxFaceWidth_cm = 0.95 * min(distA_CH, distB_DJ)
        coneDistance_cm = math.hypot(ppd_cm, dpd_cm)
        if self._faceWidth_cm == 0:
            faceWidthResolved_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)
        else:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise ValueError(
                    f'Face Width must be at most {to_mm(maxFaceWidth_cm):.4f} mm '
                    f'(got {to_mm(self._faceWidth_cm):.4f} mm)')
            faceWidthResolved_cm = self._faceWidth_cm
        self._faceWidthResolved_cm = faceWidthResolved_cm

        # Step 27: resolve the Root Length and each gear's Maximum Bore Diameter.
        apexToDed_cm = math.sqrt(R_cm ** 2 + (1.25 * module_cm) ** 2)
        delta_f = math.atan(1.25 * module_cm / R_cm)
        gamma_root_p = gamma_p - delta_f
        gamma_root_g = gamma_g - delta_f
        rootLength0_cm = faceWidthResolved_cm * apexToDed_cm / R_cm

        def resolveToeRadius(pitchRadius, gamma, userValue, gearName):
            auto = pitchRadius - faceWidthResolved_cm / math.sin(gamma)
            ceiling = (pitchRadius - 1.25 * module_cm * math.cos(gamma)) * (
                1.0 - faceWidthResolved_cm / R_cm)
            if userValue == 0:
                return auto, ceiling
            if userValue >= ceiling:
                raise ValueError(
                    f'{gearName} Toe Radius must be strictly below {to_mm(ceiling):.4f} mm '
                    f'(got {to_mm(userValue):.4f} mm)')
            return userValue, ceiling

        pinionToeRadiusResolved_cm, pinionToeCeiling_cm = resolveToeRadius(
            ppd_cm / 2.0, gamma_p, self._pinionToeRadius_cm, 'Pinion Gear')
        drivingToeRadiusResolved_cm, drivingToeCeiling_cm = resolveToeRadius(
            dpd_cm / 2.0, gamma_g, self._drivingToeRadius_cm, 'Driving Gear')
        self._pinionToeRadiusResolved_cm = pinionToeRadiusResolved_cm
        self._drivingToeRadiusResolved_cm = drivingToeRadiusResolved_cm

        toeLimitPinion_cm = apexToDed_cm - pinionToeRadiusResolved_cm / math.sin(gamma_root_p)
        toeLimitDriving_cm = apexToDed_cm - drivingToeRadiusResolved_cm / math.sin(gamma_root_g)
        minToeLimit_cm = min(toeLimitPinion_cm, toeLimitDriving_cm)

        if self._toeExtension_pct > 0:
            if minToeLimit_cm <= rootLength0_cm:
                if toeLimitPinion_cm <= rootLength0_cm:
                    raise ValueError(
                        'Toe Extension cannot be above 0 for this pair: Pinion Gear Toe Radius '
                        f'must come below {to_mm(pinionToeCeiling_cm):.4f} mm')
                raise ValueError(
                    'Toe Extension cannot be above 0 for this pair: Driving Gear Toe Radius '
                    f'must come below {to_mm(drivingToeCeiling_cm):.4f} mm')
            rootLength_cm = rootLength0_cm + (self._toeExtension_pct / 100.0) * 0.99 * (
                minToeLimit_cm - rootLength0_cm)
        else:
            rootLength_cm = rootLength0_cm

        def resolveBore(pitchDiameter, pitchRadius, gamma, gamma_root, resolvedBaseHeight,
                         userValue, gearName):
            if not self._boreEnable:
                return 0.0
            r_heel = pitchRadius - resolvedBaseHeight / math.tan(gamma)
            r_toe = (apexToDed_cm - rootLength_cm) * math.sin(gamma_root)
            maxBore_cm = 2.0 * 0.95 * min(r_heel, r_toe)
            if userValue == 0:
                return min(pitchDiameter / 4.0, maxBore_cm)
            if userValue > maxBore_cm:
                raise ValueError(
                    f'{gearName} Bore Diameter must be at most {to_mm(maxBore_cm):.4f} mm '
                    f'(got {to_mm(userValue):.4f} mm)')
            return userValue

        pinionBoreResolved_cm = resolveBore(
            ppd_cm, ppd_cm / 2.0, gamma_p, gamma_root_p, self._pinionBaseHeight_cm,
            self._pinionBore_cm, 'Pinion Gear')
        drivingBoreResolved_cm = resolveBore(
            dpd_cm, dpd_cm / 2.0, gamma_g, gamma_root_g, self._drivingBaseHeight_cm,
            self._drivingBore_cm, 'Driving Gear')

        # Compute M, N (pinion toe line) and O, P (driving toe line) now,
        # since the pinion's shaft-axis edge (step 22) needs N's along-shaft
        # coordinate before it can be seeded.
        apexToC_len = _v2_len(_v2_sub(ptC, apex))
        fracM = 1.0 - rootLength_cm / apexToC_len
        ptM = _v2_lerp(apex, ptC, fracM)
        mPerpDist = _perp_dist_from_line2(ptM, apex, pinionDir)
        slideM = (mPerpDist - pinionToeRadiusResolved_cm) / math.cos(gamma_p)
        chDir = (math.sin(gamma_p) * pinionDir[0] - math.cos(gamma_p) * dropADir[0],
                  math.sin(gamma_p) * pinionDir[1] - math.cos(gamma_p) * dropADir[1])
        ptN = _v2_add(ptM, _v2_scale(chDir, slideM))

        apexToD_len = _v2_len(_v2_sub(ptD, apex))
        fracO = 1.0 - rootLength_cm / apexToD_len
        ptO = _v2_lerp(apex, ptD, fracO)
        oPerpDist = _perp_dist_from_line2(ptO, apex, drivingDir)
        slideO = (oPerpDist - drivingToeRadiusResolved_cm) / math.cos(gamma_g)
        djDir = (math.sin(gamma_g) * drivingDir[0] - math.cos(gamma_g) * dropBDir[0],
                  math.sin(gamma_g) * drivingDir[1] - math.cos(gamma_g) * dropBDir[1])
        ptP = _v2_add(ptO, _v2_scale(djDir, slideO))

        # Step 24: point K.
        ptK = _line_intersect2(apex, pinionDir, apex2, pinionDedDir)
        lineGK = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptG), _pt2(ptK))
        lineGK.isConstruction = True
        gc.addCoincident(lineGK.startSketchPoint, gPoint)
        kPoint = lineGK.endSketchPoint
        gc.addCoincident(kPoint, pinionShaftAxis)
        gc.addCoincident(kPoint, pinionDedendum)

        lineCK = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptC), _pt2(ptK))
        lineCK.isConstruction = True
        gc.addCoincident(lineCK.startSketchPoint, cPoint2)
        gc.addCoincident(lineCK.endSketchPoint, kPoint)

        # Step 25: tooth-centre point K'.
        pinionVirtualPitchRadius_cm = (ppd_cm / 2.0) / math.cos(gamma_p)
        if self._toothSpacing_cm > 0:
            ptKp = _v2_add(apex2, _v2_scale(
                pinionDedDir, pinionVirtualPitchRadius_cm + self._toothSpacing_cm))
            lineKKp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptK), _pt2(ptKp))
            lineKKp.isConstruction = True
            gc.addCoincident(lineKKp.startSketchPoint, kPoint)
            gc.addCoincident(lineKKp.endSketchPoint, pinionDedendum)
            dimKKp = sd.addDistanceDimension(
                lineKKp.startSketchPoint, lineKKp.endSketchPoint, AlignedDim,
                _text_point2(ptK, ptKp))
            dimKKp.parameter.value = self._toothSpacing_cm
            kpPoint = lineKKp.endSketchPoint

            lineCKp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptC), _pt2(ptKp))
            lineCKp.isConstruction = True
            gc.addCoincident(lineCKp.startSketchPoint, cPoint2)
            gc.addCoincident(lineCKp.endSketchPoint, kpPoint)
            pinionToothCenterPoint = kpPoint
            pinionToothCenterRefLine = lineCKp
        else:
            ptKp = ptK
            kpPoint = None
            pinionToothCenterPoint = kPoint
            pinionToothCenterRefLine = lineCK

        # Step 22: line A' -> G, the pinion's shaft-axis edge.
        alongN = _v2_dot(_v2_sub(ptN, apex), pinionDir)
        ptAp = _v2_add(apex, _v2_scale(pinionDir, alongN))
        lineApG = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptAp), _pt2(ptG))
        lineApG.isConstruction = True
        gc.addCoincident(lineApG.endSketchPoint, gPoint)
        apPoint = lineApG.startSketchPoint

        # Step 28: line M -> N, the pinion toe line.
        lineMN = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptM), _pt2(ptN))
        lineMN.isConstruction = True
        gc.addCoincident(lineMN.startSketchPoint, rootAxisPinion)
        mPoint = lineMN.startSketchPoint
        nPoint = lineMN.endSketchPoint
        gc.addParallel(lineMN, lineCH)
        dimMN = sd.addOffsetDimension(lineCH, lineMN, _text_point2(ptM, ptC, nudge=0.1))
        dimMN.parameter.value = rootLength_cm * R_cm / apexToC_len

        lineMC = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptM), _pt2(ptC))
        lineMC.isConstruction = True
        gc.addCoincident(lineMC.startSketchPoint, mPoint)
        gc.addCoincident(lineMC.endSketchPoint, cPoint2)

        # Step 29: the front face N -> A'.
        lineNAp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptN), _pt2(ptAp))
        lineNAp.isConstruction = True
        gc.addCoincident(lineNAp.startSketchPoint, nPoint)
        gc.addCoincident(lineNAp.endSketchPoint, apPoint)
        gc.addCoincident(apPoint, pinionShaftAxis)
        gc.addPerpendicular(lineNAp, pinionShaftAxis)
        dimNAp = sd.addDistanceDimension(
            lineNAp.startSketchPoint, lineNAp.endSketchPoint, AlignedDim,
            _text_point2(ptN, ptAp))
        dimNAp.parameter.value = pinionToeRadiusResolved_cm

        # Step 30: point L and L', the driving twins of K and K'.
        ptL = _line_intersect2(apex, drivingDir, apex2, drivingDedDir)
        lineIL = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptI), _pt2(ptL))
        lineIL.isConstruction = True
        gc.addCoincident(lineIL.startSketchPoint, iPoint)
        lPoint = lineIL.endSketchPoint
        gc.addCoincident(lPoint, drivingShaftAxis)
        gc.addCoincident(lPoint, drivingDedendum)

        lineDL = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptD), _pt2(ptL))
        lineDL.isConstruction = True
        gc.addCoincident(lineDL.startSketchPoint, dPoint)
        gc.addCoincident(lineDL.endSketchPoint, lPoint)

        drivingVirtualPitchRadius_cm = (dpd_cm / 2.0) / math.cos(gamma_g)
        if self._toothSpacing_cm > 0:
            ptLp = _v2_add(apex2, _v2_scale(
                drivingDedDir, drivingVirtualPitchRadius_cm + self._toothSpacing_cm))
            lineLLp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptL), _pt2(ptLp))
            lineLLp.isConstruction = True
            gc.addCoincident(lineLLp.startSketchPoint, lPoint)
            gc.addCoincident(lineLLp.endSketchPoint, drivingDedendum)
            dimLLp = sd.addDistanceDimension(
                lineLLp.startSketchPoint, lineLLp.endSketchPoint, AlignedDim,
                _text_point2(ptL, ptLp))
            dimLLp.parameter.value = self._toothSpacing_cm
            lpPoint = lineLLp.endSketchPoint

            lineDLp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptD), _pt2(ptLp))
            lineDLp.isConstruction = True
            gc.addCoincident(lineDLp.startSketchPoint, dPoint)
            gc.addCoincident(lineDLp.endSketchPoint, lpPoint)
            drivingToothCenterPoint = lpPoint
            drivingToothCenterRefLine = lineDLp
        else:
            ptLp = ptL
            lpPoint = None
            drivingToothCenterPoint = lPoint
            drivingToothCenterRefLine = lineDL

        # Step 31: line O -> P, the driving mirror of M -> N.
        lineOP = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptO), _pt2(ptP))
        lineOP.isConstruction = True
        gc.addCoincident(lineOP.startSketchPoint, rootAxisDriving)
        oPoint = lineOP.startSketchPoint
        pPoint = lineOP.endSketchPoint
        gc.addParallel(lineOP, lineDJ)
        dimOP = sd.addOffsetDimension(lineDJ, lineOP, _text_point2(ptO, ptD, nudge=0.1))
        dimOP.parameter.value = rootLength_cm * R_cm / apexToD_len

        lineOD = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptO), _pt2(ptD))
        lineOD.isConstruction = True
        gc.addCoincident(lineOD.startSketchPoint, oPoint)
        gc.addCoincident(lineOD.endSketchPoint, dPoint)

        # Step 32: the driving front face P -> B', then line B' -> I.
        alongP = _v2_dot(_v2_sub(ptP, apex), drivingDir)
        ptBp = _v2_add(apex, _v2_scale(drivingDir, alongP))
        linePBp = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptP), _pt2(ptBp))
        linePBp.isConstruction = True
        gc.addCoincident(linePBp.startSketchPoint, pPoint)
        bpPoint = linePBp.endSketchPoint
        gc.addCoincident(bpPoint, drivingShaftAxis)
        gc.addPerpendicular(linePBp, drivingShaftAxis)
        dimPBp = sd.addDistanceDimension(
            linePBp.startSketchPoint, linePBp.endSketchPoint, AlignedDim,
            _text_point2(ptP, ptBp))
        dimPBp.parameter.value = drivingToeRadiusResolved_cm

        lineBpI = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt2(ptBp), _pt2(ptI))
        lineBpI.isConstruction = True
        gc.addCoincident(lineBpI.startSketchPoint, bpPoint)
        gc.addCoincident(lineBpI.endSketchPoint, iPoint)

        # --- End-of-step gates ---

        if not sketch.isFullyConstrained:
            raise RuntimeError('Gear Profiles sketch is not fully constrained')

        expected = {
            'Apex': apex, 'B': ptB, 'A': ptA, 'Apex2': apex2, 'C': ptC, 'D': ptD,
            'E': ptE, 'F': ptF, 'G': ptG, 'H': ptH, 'I': ptI, 'J': ptJ, 'K': ptK,
            "K'": ptKp, 'M': ptM, 'N': ptN, "A'": ptAp, 'L': ptL, "L'": ptLp,
            'O': ptO, 'P': ptP, "B'": ptBp,
        }
        actual = {
            'Apex': apexPoint, 'B': bPoint, 'A': aPoint, 'Apex2': apex2Point, 'C': cPoint2,
            'D': dPoint, 'E': ePoint, 'F': fPoint, 'G': gPoint, 'H': hPoint, 'I': iPoint,
            'J': jPoint, 'K': kPoint, "K'": kpPoint, 'M': mPoint, 'N': nPoint, "A'": apPoint,
            'L': lPoint, "L'": lpPoint, 'O': oPoint, 'P': pPoint, "B'": bpPoint,
        }
        order = ['Apex', 'B', 'A', 'Apex2', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
                 "K'", 'M', 'N', "A'", 'L', "L'", 'O', 'P', "B'"]
        if self._toothSpacing_cm == 0:
            order = [name for name in order if name not in ("K'", "L'")]

        tol_cm = 1e-4
        for name in order:
            ex, ey = expected[name]
            ag = actual[name].geometry
            if math.hypot(ag.x - ex, ag.y - ey) > tol_cm:
                raise RuntimeError(
                    f'Gear Profiles sketch: point {name} moved from its seed by more than '
                    f'0.001 mm (expected ({ex:.6f}, {ey:.6f}), got ({ag.x:.6f}, {ag.y:.6f}))')

        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'gamma': gamma_p,
            'pitchDiameter_cm': ppd_cm,
            'toothCenterPoint': pinionToothCenterPoint,
            'toothCenterRefLine': pinionToothCenterRefLine,
            'hexVertices': [apPoint, gPoint, hPoint, cPoint2, mPoint, nPoint],
            'toeEdgePoints': (mPoint, nPoint),
            'heelEdgePoints': (cPoint2, hPoint),
            'boreDiameter_cm': pinionBoreResolved_cm,
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'gamma': gamma_g,
            'pitchDiameter_cm': dpd_cm,
            'toothCenterPoint': drivingToothCenterPoint,
            'toothCenterRefLine': drivingToothCenterRefLine,
            'hexVertices': [bpPoint, iPoint, jPoint, dPoint, oPoint, pPoint],
            'toeEdgePoints': (oPoint, pPoint),
            'heelEdgePoints': (dPoint, jPoint),
            'boreDiameter_cm': drivingBoreResolved_cm,
        }

        return pinionCtx, drivingCtx, apexPoint, coneDistance_cm

    # ------------------------------------------------------------------
    # S7-S29: run once per gear.
    # ------------------------------------------------------------------

    def _createGearBody(self, ctx, module_cm, module_mm, apexSketchPoint, coneDistance_cm):
        gearLabel = ctx['label']
        gamma = ctx['gamma']

        # S7: resolve the virtual-spur dimensions for this gear.
        pitchDia_mm = to_mm(ctx['pitchDiameter_cm'])
        virtualPitchRadius_mm = (pitchDia_mm / 2.0) / math.cos(gamma)
        virtualTeeth = 2.0 * virtualPitchRadius_mm / module_mm
        rootSink_mm = 0.05 * 2.25 * module_mm

        # S8: the tooth plane.
        toothPlane = solids.plane_by_angle(
            self.designComponent, ctx['toothCenterRefLine'], self._gearProfilesPlane, 90)
        toothPlane.name = f'{gearLabel} Plane'
        ctx['toothPlane'] = toothPlane

        # S9: the tooth sketch.
        toothSketch = self.designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        proxy = VirtualSpurProxy(
            module_mm=module_mm, virtualTeeth=virtualTeeth, rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))
        toothEmbedded = proxy._lastToothEmbedded
        ctx['toothEmbedded'] = toothEmbedded
        if not toothSketch.isFullyConstrained:
            futil.log(f'{gearLabel} Tooth sketch is not fully constrained '
                      '(expected: the circle labels hold a degree of freedom)')
        ctx['toothSketch'] = toothSketch

        # S10: the tooth axis.
        helperPlaneInput = self.designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(
            ctx['toothCenterRefLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane = self.designComponent.constructionPlanes.add(helperPlaneInput)

        axisInput = self.designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = self.designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'
        ctx['toothAxis'] = toothAxis

        # S11: the gear component.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{gearLabel} Gear'
        ctx['gearOccurrence'] = gearOccurrence

        # S12: the hexagon profile sketch.
        profileSketch = self.designComponent.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'
        srcPoints = ctx['hexVertices']
        newPts = [profileSketch.sketchPoints.add(profileSketch.modelToSketchSpace(src.worldGeometry))
                  for src in srcPoints]
        n = len(newPts)
        edges = []
        for i in range(n):
            edges.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(
                newPts[i], newPts[(i + 1) % n]))
        for e in edges:
            e.startSketchPoint.isFixed = True
            e.endSketchPoint.isFixed = True
        shaftAxisEdge = edges[0]
        ctx['profileSketch'] = profileSketch
        ctx['shaftAxisEdge'] = shaftAxisEdge
        if not profileSketch.isFullyConstrained:
            raise RuntimeError(f'{gearLabel} Profile sketch is not fully constrained')

        # S13: revolve the Gear Body.
        profile = profileSketch.profiles.item(0)
        revolveInput = self.designComponent.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = self.designComponent.features.revolveFeatures.add(revolveInput)
        gearBody = revolve.bodies.item(0)
        ctx['gearBody'] = gearBody

        # S14: loft the Apex sketch point to the tooth profile.
        loftInput = self.designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        wantLines = 0 if toothEmbedded else 2
        toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput.loftSections.add(toothProfile)
        toothBody = self.designComponent.features.loftFeatures.add(loftInput).bodies.item(0)

        # S15: the tooth-body hook.
        toeP0, toeP1 = ctx['toeEdgePoints']
        heelP0, heelP1 = ctx['heelEdgePoints']
        toeMid = _midpoint3d(toeP0.worldGeometry, toeP1.worldGeometry)
        heelMid = _midpoint3d(heelP0.worldGeometry, heelP1.worldGeometry)
        toeConeWorld = toeP0.worldGeometry
        heelConeWorld = heelP0.worldGeometry
        apexWorld = apexSketchPoint.worldGeometry

        toothBody = self._transformToothBody(
            self.designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, toothPlane,
            gearLabel, ctx['teeth'], gamma)

        # S24: circular-pattern the tooth.
        seedColl = adsk.core.ObjectCollection.create()
        seedColl.add(toothBody)
        patternInput = self.designComponent.features.circularPatternFeatures.createInput(
            seedColl, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = self.designComponent.features.circularPatternFeatures.add(patternInput)

        toolCollection = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolCollection.add(pattern.bodies.item(i))

        # S25: Combine-Join the teeth into the Gear Body.
        combineInput = self.designComponent.features.combineFeatures.createInput(
            gearBody, toolCollection)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        self.designComponent.features.combineFeatures.add(combineInput)

        # S26 / S27: the bore.
        if self._boreEnable:
            borePlaneInput = self.designComponent.constructionPlanes.createInput()
            borePlaneInput.setByDistanceOnPath(
                shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
            borePlane = self.designComponent.constructionPlanes.add(borePlaneInput)

            boreSketch = self.designComponent.sketches.add(borePlane)
            boreSketch.name = f'{gearLabel} Bore'
            boreRadius_cm = ctx['boreDiameter_cm'] / 2.0
            boreCircle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0.0, 0.0, 0.0), boreRadius_cm)
            boreCircle.centerSketchPoint.isFixed = True
            textPointBore = adsk.core.Point3D.create(boreRadius_cm, 0.0, 0.0)
            dimBore = boreSketch.sketchDimensions.addDiameterDimension(boreCircle, textPointBore)
            dimBore.parameter.value = ctx['boreDiameter_cm']
            if not boreSketch.isFullyConstrained:
                raise RuntimeError(f'{gearLabel} Bore sketch is not fully constrained')

            boreProfile = boreSketch.profiles.item(0)
            extrudeInput = self.designComponent.features.extrudeFeatures.createInput(
                boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
            extrudeInput.setSymmetricExtent(
                adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)
            extrudeInput.participantBodies = [gearBody]
            self.designComponent.features.extrudeFeatures.add(extrudeInput)

        # S28: meshing rotation.
        if gearLabel == 'Driving':
            angle = math.pi / ctx['teeth']
        else:
            angle = self._pinionMeshPhase(ctx['teeth'])
        solids.rotate_body_about_edge(self.designComponent, gearBody, shaftAxisEdge, angle)

        # S29: move the finished body into the gear component.
        gearBody.moveToComponent(gearOccurrence)

    def _pinionMeshPhase(self, pinionTeeth):
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # ------------------------------------------------------------------
    # S15 / S16-S23: the tooth-body hook and, for a spiral gear, its curved
    # tooth construction.
    # ------------------------------------------------------------------

    def _distAlong(self, apexWorld: adsk.core.Point3D, coneVec: adsk.core.Vector3D,
                   p: adsk.core.Point3D):
        return apexWorld.vectorTo(p).dotProduct(coneVec)

    def _perpDistToAxis(self, apexWorld: adsk.core.Point3D, axisDir: adsk.core.Vector3D,
                        pt: adsk.core.Point3D):
        rel = apexWorld.vectorTo(pt)
        along = rel.dotProduct(axisDir)
        alongVec = axisDir.copy()
        alongVec.scaleBy(along)
        perpVec = rel.copy()
        perpVec.subtract(alongVec)
        return perpVec.length

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                             toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                             shaftAxisEdge: adsk.fusion.SketchLine, apexWorld: adsk.core.Point3D,
                             apexSketchPoint: adsk.fusion.SketchPoint, toeMid: adsk.core.Point3D,
                             heelMid: adsk.core.Point3D, toeConeWorld: adsk.core.Point3D,
                             heelConeWorld: adsk.core.Point3D,
                             parentToothPlane: adsk.fusion.ConstructionPlane, gearLabel,
                             teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        # S16: the world frame.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        edgeStart = shaftAxisEdge.startSketchPoint.worldGeometry
        edgeEnd = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = edgeStart.vectorTo(edgeEnd)
        axisDir.normalize()

        coneVec = apexWorld.vectorTo(heelConeWorld)
        coneVec.normalize()

        v = axisDir.crossProduct(coneVec)
        v.normalize()

        R_toe = self._distAlong(apexWorld, coneVec, toeMid)
        R_heel = self._distAlong(apexWorld, coneVec, heelMid)
        R_mean = (R_toe + R_heel) / 2.0
        span = R_heel - R_toe

        coneElemEnd = apexWorld.copy()
        disp = coneVec.copy()
        disp.scaleBy(R_heel)
        coneElemEnd.translateBy(disp)

        coneElemSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneElemSketch.name = f'{gearLabel} Cone Element'
        coneElemLine = coneElemSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, coneElemEnd)
        coneElemLine.isConstruction = True

        tracePlane = solids.plane_by_angle(designComponent, coneElemLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # S17: the cutter arc.
        r_c = self._cutterRadius_cm if self._cutterRadius_cm != 0 else R_mean
        sign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            sign = -sign
        psi = self._spiralAngle_rad
        Cx = R_mean - r_c * math.sin(psi)
        Cy = sign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
        heel2d = solids.circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)

        def tanW(px, py):
            return solids.combine_point(apexWorld, px, coneVec, py, v)

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            tanW(Cx, Cy), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        dimCircle = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, tanW(Cx + r_c, Cy))
        dimCircle.parameter.value = 2 * r_c

        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            tanW(toe2d[0], toe2d[1]), tanW(R_mean, 0), tanW(heel2d[0], heel2d[1]))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        dimArc = traceSketch.sketchDimensions.addRadialDimension(traceArc, tanW(R_mean, 0))
        dimArc.parameter.value = r_c

        # S18: slice the straight tooth into 8 slabs.
        planeOrigin = parentToothPlane.geometry.origin
        planeNormal = get_normal(parentToothPlane)
        testVec = planeOrigin.vectorTo(apexWorld)
        sliceSign = 1.0 if testVec.dotProduct(planeNormal) > 0 else -1.0
        offsets = [sliceSign * (k + 1) * span / 6.0 for k in range(8)]
        pieces = solids.slice_body_by_offset_planes(
            designComponent, toothBody, parentToothPlane, offsets)
        if len(pieces) == 1:
            offsets = [-o for o in offsets]
            pieces = solids.slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)
            if len(pieces) == 1:
                raise RuntimeError(
                    f'{gearLabel}: tooth slice produced 1 piece (expected 9) - span={span}, '
                    f'sign tried={sliceSign} and {-sliceSign}')

        # S19: order the segments and drop the Apex scrap.
        piecesSorted = sorted(
            pieces,
            key=lambda b: self._distAlong(apexWorld, coneVec, b.physicalProperties.centerOfMass))
        scrap = piecesSorted[0]
        segments = piecesSorted[1:]
        designComponent.features.removeFeatures.add(scrap)
        if not segments:
            raise RuntimeError(f'{gearLabel}: no tooth segments remain after dropping the Apex scrap')

        # S20: twist each segment about the shaft axis.
        def heelFaceOf(body):
            best = None
            bestDist = None
            for face in body.faces:
                dist = self._distAlong(apexWorld, coneVec, face.centroid)
                if bestDist is None or dist > bestDist:
                    bestDist = dist
                    best = face
            return best, bestDist

        def toeFaceOf(body):
            best = None
            bestDist = None
            for face in body.faces:
                dist = self._distAlong(apexWorld, coneVec, face.centroid)
                if bestDist is None or dist < bestDist:
                    bestDist = dist
                    best = face
            return best, bestDist

        toeAngle = math.atan2(toe2d[1], toe2d[0])
        heelAngle = math.atan2(heel2d[1], heel2d[0])
        phi_crown = heelAngle - toeAngle
        total = abs(phi_crown) / math.sin(gamma)

        for seg in segments:
            _, distH = heelFaceOf(seg)
            ang = -sign * total * (R_mean - distH) / span
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir.copy(), apexWorld)
            coll = adsk.core.ObjectCollection.create()
            coll.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(coll)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # S21: lengthwise crown.
        segData = [(seg,) + heelFaceOf(seg) for seg in segments]
        segData.sort(key=lambda t: t[2])
        outermost = segData[-1][0]

        self.designOccurrence.activate()
        try:
            for seg, face, distH in segData[:-1]:
                u = (R_heel - distH) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise RuntimeError(
                        f'{gearLabel}: crown factor {factor} is not positive for segment '
                        f'u={u}')

                verts = list(face.vertices)
                verts.sort(key=lambda vtx: self._perpDistToAxis(apexWorld, axisDir, vtx.geometry))
                p0 = verts[0].geometry
                p1 = verts[1].geometry
                mid = adsk.core.Point3D.create(
                    (p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0, (p0.z + p1.z) / 2.0)

                baseSketch = self.designComponent.sketches.add(face)
                baseSketchPoint = baseSketch.sketchPoints.add(baseSketch.modelToSketchSpace(mid))

                scaleColl = adsk.core.ObjectCollection.create()
                scaleColl.add(seg)
                scaleInput = self.designComponent.features.scaleFeatures.createInput(
                    scaleColl, baseSketchPoint, adsk.core.ValueInput.createByReal(factor))
                self.designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # S22: loft the crowned segments into the spiral tooth.
        order = sorted(segments, key=lambda seg: heelFaceOf(seg)[1])
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toeFaceOf(order[0])[0])
        for seg in order:
            loftInput.loftSections.add(heelFaceOf(seg)[0])
        spiralToothFeature = designComponent.features.loftFeatures.add(loftInput)
        spiralTooth = spiralToothFeature.bodies.item(0)
        spiralTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in segments:
            designComponent.features.removeFeatures.add(seg)

        # S23: conical end cuts.
        return solids.cut_conical_ends(
            designComponent, spiralTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
