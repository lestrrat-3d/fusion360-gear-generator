# Bevel gear pair generator.
#
# Generated from spec/bevelgear/steps.md. This is a STANDALONE generator: it does not
# subclass base.Generator, carries no GenerationContext, and registers no Fusion user
# parameters -- every value is precomputed in Python and written into geometry numerically
# ([PB-PRECOMPUTED-MODE]).

import math
import typing
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_selection, get_boolean
from .utilities import find_profile_by_curve_counts
from .solids import (
    cut_conical_ends, slice_body_by_offset_planes, rotate_body_about_edge,
    plane_by_angle, combine_point, circle_intersect_nearest,
    hide_construction_geometry,
)
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy


# ---------------------------------------------------------------------------
# S01: dialog input ids, in dialog row order.
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

# S23: lengthwise-crown tuning constant. 0 disables the crown.
_CROWN_PER_RAD = 0.5

# S25 / S31: the pinion's extra mesh phase, in whole teeth. 0 by default, because the
# spiral twist is centred on R_mean so the mid-face section already meshes.
_PINION_MESH_PHASE_TEETH = 0


# ---------------------------------------------------------------------------
# Generic 2-D / 3-D vector helpers used to seed the S07 lattice. These are ours, not
# Fusion API calls -- they do the same closed-form math the spec states, in plain
# Python, so every seed lands at (or extremely near) its solved position
# ([PB-SEED-NEAR]).
# ---------------------------------------------------------------------------

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
    length = _v2_len(a)
    return (a[0] / length, a[1] / length)


def _v2_perp(a):
    return (-a[1], a[0])


def _v2_mid(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _v2_rotate(a, angleRad):
    c, s = math.cos(angleRad), math.sin(angleRad)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _line_intersect_2d(p1, d1, p2, d2):
    # Intersection of the infinite line through p1 in direction d1 with the infinite
    # line through p2 in direction d2.
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-12:
        raise Exception('bevelgear: two supposedly-crossing §2 lines came out parallel')
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def _perp_dist_point_to_line_2d(p, linePt, lineDir):
    vx, vy = p[0] - linePt[0], p[1] - linePt[1]
    return abs(vx * lineDir[1] - vy * lineDir[0])


def _perp_dist_point_to_segment_2d(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    return abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / length


def _sub_vec3(toPt, fromPt):
    return adsk.core.Vector3D.create(toPt.x - fromPt.x, toPt.y - fromPt.y, toPt.z - fromPt.z)


def _unit_vec3(fromPt, toPt):
    v: adsk.core.Vector3D = _sub_vec3(toPt, fromPt)
    v.normalize()
    return v


def _midpoint3(a, b):
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


# ---------------------------------------------------------------------------
# S01: the command dialog.
# ---------------------------------------------------------------------------

class BevelGearCommandInputsConfigurator:
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
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))
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
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
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

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        design: adsk.fusion.Design = get_design()
        try:
            value = design.unitsManager.evaluateExpression(spiralInput.expression, 'rad')
            visible = value > 0
        except Exception:
            visible = True
        handInput.isVisible = visible
        cutterInput.isVisible = visible

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


# ---------------------------------------------------------------------------
# The generator.
# ---------------------------------------------------------------------------

class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design

        self.bevelOccurrence: adsk.fusion.Occurrence = None
        self.bevelComponent: adsk.fusion.Component = None
        self.designOccurrence: adsk.fusion.Occurrence = None
        self.designComponent: adsk.fusion.Component = None

        self._anchorSketch: adsk.fusion.Sketch = None
        self._anchorCenterPoint: adsk.fusion.SketchPoint = None
        self._anchorLine: adsk.fusion.SketchLine = None

        self._gearProfilesPlane: adsk.fusion.ConstructionPlane = None
        self._gpSketch: adsk.fusion.Sketch = None
        self._apexSketchPoint: adsk.fusion.SketchPoint = None
        self._apex2d = None

        self._gamma_p = None
        self._gamma_g = None
        self._coneDistance_cm = None
        self._faceWidthResolved_cm = None

    # -----------------------------------------------------------------
    # S02: read and validate the inputs.
    # -----------------------------------------------------------------

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        design = self.design
        unitsManager = design.unitsManager

        parentSel = get_selection(inputs, INPUT_ID_PARENT)[0]
        if parentSel.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentSel.component
        else:
            parentComponent = parentSel

        targetPlane = get_selection(inputs, INPUT_ID_PLANE)[0]
        centerPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]

        module = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_MODULE).expression, '')

        shaftAngle_rad = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_SHAFT_ANGLE).expression, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)

        drivingTeeth = int(round(unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_TEETH).expression, '')))
        pinionTeeth = int(round(unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_TEETH).expression, '')))

        drivingBaseHeight_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_BASE_HEIGHT).expression, 'mm')
        pinionBaseHeight_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_BASE_HEIGHT).expression, 'mm')

        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_BORE).expression, 'mm')
        pinionBore_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_BORE).expression, 'mm')

        faceWidth_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_FACE_WIDTH).expression, 'mm')
        toothSpacing_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_TOOTH_SPACING).expression, 'mm')

        spiralAngle_rad = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_SPIRAL_ANGLE).expression, 'deg')

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem = handInput.selectedItem
        hand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        cutterRadius_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_CUTTER_RADIUS).expression, 'mm')

        toeExtension = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_TOE_EXTENSION).expression, '')
        drivingToeRadius_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_TOE_RADIUS).expression, 'mm')
        pinionToeRadius_cm = unitsManager.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_TOE_RADIUS).expression, 'mm')

        # --- 1. Basic range checks. ---
        if module <= 0:
            raise Exception(f'Module must be greater than 0 (got {module})')
        if drivingTeeth < 3:
            raise Exception(f'Driving Gear Teeth must be at least 3 (got {drivingTeeth})')
        if pinionTeeth < 3:
            raise Exception(f'Pinion Gear Teeth must be at least 3 (got {pinionTeeth})')
        for (label, value) in (
            ('Driving Gear Base Height', drivingBaseHeight_cm),
            ('Pinion Gear Base Height', pinionBaseHeight_cm),
            ('Driving Gear Bore Diameter', drivingBore_cm),
            ('Pinion Gear Bore Diameter', pinionBore_cm),
            ('Face Width', faceWidth_cm),
            ('Tooth Spacing', toothSpacing_cm),
            ('Cutter Radius', cutterRadius_cm),
            ('Driving Gear Toe Radius', drivingToeRadius_cm),
            ('Pinion Gear Toe Radius', pinionToeRadius_cm),
        ):
            if value < 0:
                raise Exception(f'{label} must be non-negative (got {to_mm(value)} mm)')
        if toeExtension < 0 or toeExtension > 100:
            raise Exception(f'Toe Extension must be between 0 and 100 (got {toeExtension})')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be at least 0 deg and below 60 deg '
                f'(got {spiralAngle_deg} deg)')

        # --- 2. Shaft Angle, against the Maximum Shaft Angle. ---
        DPD_cm = to_cm(module * drivingTeeth)
        PPD_cm = to_cm(module * pinionTeeth)
        maxShaftAngleFromCones_deg = math.degrees(
            math.acos(-min(DPD_cm, PPD_cm) / max(DPD_cm, PPD_cm)))
        maxShaftAngle_deg = min(maxShaftAngleFromCones_deg, 150.0)
        if maxShaftAngleFromCones_deg <= 150.0:
            shaftAngleOk = (shaftAngle_deg >= 30.0) and (shaftAngle_deg < maxShaftAngleFromCones_deg)
        else:
            shaftAngleOk = (shaftAngle_deg >= 30.0) and (shaftAngle_deg <= 150.0)
        if not shaftAngleOk:
            raise Exception(
                f'Shaft Angle must be at least 30 deg and below {maxShaftAngle_deg} deg '
                f'(got {shaftAngle_deg} deg)')

        tan_gamma_p = (math.sin(shaftAngle_rad) * PPD_cm) / (
            DPD_cm + PPD_cm * math.cos(shaftAngle_rad))
        gamma_p = math.atan(tan_gamma_p)
        gamma_g = shaftAngle_rad - gamma_p

        # --- 3. Minimum Teeth, per gear, against that gear's own gamma. ---
        minTeethFloor_p = 5.27 * math.cos(gamma_p)
        minTeethFloor_g = 5.27 * math.cos(gamma_g)
        if pinionTeeth < minTeethFloor_p:
            raise Exception(
                f'Pinion Gear Teeth must be at least {minTeethFloor_p} at this Shaft Angle '
                f'(got {pinionTeeth})')
        if drivingTeeth < minTeethFloor_g:
            raise Exception(
                f'Driving Gear Teeth must be at least {minTeethFloor_g} at this Shaft Angle '
                f'(got {drivingTeeth})')

        # --- 4. Base heights, per gear. ---
        module_cm = to_cm(module)

        def base_height_bounds(r_cm, gamma):
            minH = 1.05 * 1.25 * module_cm * math.sin(gamma)
            maxH = 0.95 * (r_cm - 1.25 * module_cm * math.cos(gamma)) * math.tan(gamma)
            return minH, maxH

        def resolve_base_height(userValue_cm, fallback_cm, minH, maxH, label):
            if userValue_cm == 0:
                v = fallback_cm
                if v < minH:
                    v = minH
                elif v > maxH:
                    v = maxH
                return v
            if userValue_cm < minH or userValue_cm > maxH:
                raise Exception(
                    f'{label} Base Height must be between {to_mm(minH)} mm and '
                    f'{to_mm(maxH)} mm (got {to_mm(userValue_cm)} mm)')
            return userValue_cm

        drivingMin, drivingMax = base_height_bounds(DPD_cm / 2.0, gamma_g)
        pinionMin, pinionMax = base_height_bounds(PPD_cm / 2.0, gamma_p)

        drivingFallback_cm = module_cm * drivingTeeth / 8.0
        drivingBaseHeightResolved_cm = resolve_base_height(
            drivingBaseHeight_cm, drivingFallback_cm, drivingMin, drivingMax, 'Driving Gear')

        pinionFallback_cm = drivingBaseHeightResolved_cm * (pinionTeeth / drivingTeeth)
        pinionBaseHeightResolved_cm = resolve_base_height(
            pinionBaseHeight_cm, pinionFallback_cm, pinionMin, pinionMax, 'Pinion Gear')

        # --- Bore diameters: 0 means auto (this gear's Pitch Diameter / 4). ---
        drivingBoreResolved_cm = drivingBore_cm if drivingBore_cm != 0 else DPD_cm / 4.0
        pinionBoreResolved_cm = pinionBore_cm if pinionBore_cm != 0 else PPD_cm / 4.0

        self._drivingBaseHeight_cm = drivingBaseHeightResolved_cm
        self._pinionBaseHeight_cm = pinionBaseHeightResolved_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBoreResolved_cm
        self._pinionBore_cm = pinionBoreResolved_cm
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm

        self._toeExtension = toeExtension
        self._drivingToeRadius_cm = drivingToeRadius_cm
        self._pinionToeRadius_cm = pinionToeRadius_cm

        self._module = module
        self._module_cm = module_cm
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_rad = shaftAngle_rad

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth,
                pinionTeeth, shaftAngle_deg)

    # -----------------------------------------------------------------
    # S03 / S04: the Bevel Gear and Design components.
    # -----------------------------------------------------------------

    def _createBevelAndDesignComponents(self, parentComponent: adsk.fusion.Component):
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

    # -----------------------------------------------------------------
    # S05: the Anchor sketch.
    # -----------------------------------------------------------------

    def _buildAnchorSketch(
            self,
            targetPlane: typing.Union[adsk.fusion.ConstructionPlane, adsk.fusion.BRepFace],
            centerPoint: typing.Union[adsk.fusion.ConstructionPoint, adsk.fusion.SketchPoint]):
        designComponent: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        # ⚠ Write the call as sketch.project(entity); project2 is not a substitute
        # (it takes a list and returns a list) -- see S05.
        projectedCenter = sketch.project(centerPoint).item(0)

        cx, cy = projectedCenter.geometry.x, projectedCenter.geometry.y
        start = adsk.core.Point3D.create(cx - 0.5, cy, 0)
        end = adsk.core.Point3D.create(cx + 0.5, cy, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)
        textPoint = adsk.core.Point3D.create(cx, cy + 0.3, 0)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)
        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorSketch = sketch
        self._anchorCenterPoint = projectedCenter
        self._anchorLine = anchorLine

    # -----------------------------------------------------------------
    # S06 + S07: the Gear Profiles plane and the §2 lattice.
    # -----------------------------------------------------------------

    def _buildGearProfiles(
            self,
            targetPlane: typing.Union[adsk.fusion.ConstructionPlane, adsk.fusion.BRepFace]):
        designComponent: adsk.fusion.Component = self.designComponent
        Aligned = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        module = self._module
        module_cm = self._module_cm
        drivingTeeth = self._drivingTeeth
        pinionTeeth = self._pinionTeeth
        sigma = self._shaftAngle_rad

        # --- S06: the Gear Profiles plane. ---
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            self._anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = gearProfilesPlane

        # --- S07: the sketch. ---
        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch
        geo = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        skLines = sketch.sketchCurves.sketchLines

        def raw(p) -> adsk.core.Point3D:
            return adsk.core.Point3D.create(p[0], p[1], 0)

        # Project the anchor geometry -- the anchor-sketch centre point (S05), not the
        # raw user-selected point, and the anchor line, both into THIS sketch.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchorLine = sketch.project(self._anchorLine).item(0)

        c = (projectedCenter.geometry.x, projectedCenter.geometry.y)
        startG = projectedAnchorLine.startSketchPoint.geometry
        endG = projectedAnchorLine.endSketchPoint.geometry
        d = _v2_unit((endG.x - startG.x, endG.y - startG.y))
        perp = _v2_perp(d)

        # The perpendicular's sign is the target-plane normal, the single permitted
        # world use in §2 ([BEVEL-F-GROW-SIDE]).
        normal = targetPlane.geometry.normal
        originWorld: adsk.core.Point3D = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(0, 0, 0))
        perpWorld: adsk.core.Point3D = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(perp[0], perp[1], 0))
        perpVecWorld: adsk.core.Vector3D = adsk.core.Vector3D.create(
            perpWorld.x - originWorld.x, perpWorld.y - originWorld.y, perpWorld.z - originWorld.z)
        if perpVecWorld.dotProduct(normal) < 0:
            perp = _v2_scale(perp, -1.0)

        # The closed form (see S07).
        DPD_cm = to_cm(module * drivingTeeth)
        PPD_cm = to_cm(module * pinionTeeth)
        tan_gamma_p = (math.sin(sigma) * PPD_cm) / (DPD_cm + PPD_cm * math.cos(sigma))
        gamma_p = math.atan(tan_gamma_p)
        gamma_g = sigma - gamma_p
        R_cm = (PPD_cm / 2.0) / math.sin(gamma_p)
        coneDistance_cm = math.hypot(DPD_cm, PPD_cm)
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._coneDistance_cm = coneDistance_cm

        lenApexA = R_cm * math.cos(gamma_p)

        # --- The centre->apex line. ---
        apexOffset = R_cm * math.cos(gamma_g) + self._drivingBaseHeight_cm
        apexSeed = _v2_add(c, _v2_scale(perp, apexOffset))
        centerToApex = skLines.addByTwoPoints(raw(c), raw(apexSeed))
        geo.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        geo.addPerpendicular(centerToApex, projectedAnchorLine)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint

        # --- The Driving Gear Shaft Axis, Apex->B. ---
        bSeed = _v2_add(c, _v2_scale(perp, self._drivingBaseHeight_cm))
        drivingShaftAxis = skLines.addByTwoPoints(raw(apexSeed), raw(bSeed))
        geo.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        geo.addParallel(drivingShaftAxis, centerToApex)
        bPoint = drivingShaftAxis.endSketchPoint
        drivingDir = _v2_unit(_v2_sub(bSeed, apexSeed))

        # --- The Pinion Gear Shaft Axis, Apex->A. ---
        candPlus = _v2_rotate(drivingDir, sigma)
        candMinus = _v2_rotate(drivingDir, -sigma)
        pinionDir = candPlus if candPlus[0] > candMinus[0] else candMinus
        aSeed = _v2_add(apexSeed, _v2_scale(pinionDir, lenApexA))
        pinionShaftAxis = skLines.addByTwoPoints(raw(apexSeed), raw(aSeed))
        geo.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        aPoint = pinionShaftAxis.endSketchPoint

        bisector = _v2_unit(_v2_add(pinionDir, drivingDir))
        angleTextPt = _v2_add(apexSeed, _v2_scale(bisector, PPD_cm / 4.0))
        angDim = dims.addAngularDimension(drivingShaftAxis, pinionShaftAxis, raw(angleTextPt))
        angDim.parameter.value = sigma

        # --- The two perpendicular drops to Apex 2. ---
        abDir = _v2_unit(_v2_sub(bSeed, aSeed))
        aCand1 = _v2_perp(pinionDir)
        aCand2 = _v2_scale(aCand1, -1.0)
        aApex2DropDir = aCand1 if _v2_dot(aCand1, abDir) > 0 else aCand2
        aApex2DropSeed = _v2_add(aSeed, _v2_scale(aApex2DropDir, PPD_cm / 2.0))
        aApex2Drop = skLines.addByTwoPoints(raw(aSeed), raw(aApex2DropSeed))
        geo.addCoincident(aApex2Drop.startSketchPoint, aPoint)
        geo.addPerpendicular(aApex2Drop, pinionShaftAxis)
        dimA = dims.addDistanceDimension(
            aApex2Drop.startSketchPoint, aApex2Drop.endSketchPoint, Aligned,
            raw(_v2_mid(aSeed, aApex2DropSeed)))
        dimA.parameter.value = PPD_cm / 2.0

        baDir = _v2_unit(_v2_sub(aSeed, bSeed))
        bCand1 = _v2_perp(drivingDir)
        bCand2 = _v2_scale(bCand1, -1.0)
        bApex2DropDir = bCand1 if _v2_dot(bCand1, baDir) > 0 else bCand2
        bApex2DropSeed = _v2_add(bSeed, _v2_scale(bApex2DropDir, DPD_cm / 2.0))
        bApex2Drop = skLines.addByTwoPoints(raw(bSeed), raw(bApex2DropSeed))
        geo.addCoincident(bApex2Drop.startSketchPoint, bPoint)
        geo.addPerpendicular(bApex2Drop, drivingShaftAxis)
        dimB = dims.addDistanceDimension(
            bApex2Drop.startSketchPoint, bApex2Drop.endSketchPoint, Aligned,
            raw(_v2_mid(bSeed, bApex2DropSeed)))
        dimB.parameter.value = DPD_cm / 2.0

        geo.addCoincident(aApex2Drop.endSketchPoint, bApex2Drop.endSketchPoint)
        apex2Point = aApex2Drop.endSketchPoint
        apex2SeedApprox = _v2_mid(aApex2DropSeed, bApex2DropSeed)

        # --- The Pitch Line. ---
        pitchLine = skLines.addByTwoPoints(raw(apexSeed), raw(apex2SeedApprox))
        geo.addCoincident(pitchLine.startSketchPoint, apexPoint)
        geo.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # --- The two dedendum lines. ---
        pitchDirSeed = _v2_unit(_v2_sub(apex2SeedApprox, apexSeed))
        pCand1 = _v2_perp(pitchDirSeed)
        pCand2 = _v2_scale(pCand1, -1.0)
        towardAnchorDir = pCand1 if _v2_dot(pCand1, perp) < _v2_dot(pCand2, perp) else pCand2
        awayFromAnchorDir = _v2_scale(towardAnchorDir, -1.0)
        drivingDedendumDir = towardAnchorDir
        pinionDedendumDir = awayFromAnchorDir

        dSeed = _v2_add(apex2SeedApprox, _v2_scale(drivingDedendumDir, 1.25 * module_cm))
        drivingDedendum = skLines.addByTwoPoints(raw(apex2SeedApprox), raw(dSeed))
        geo.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        geo.addPerpendicular(drivingDedendum, pitchLine)
        dimD = dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint, Aligned,
            raw(_v2_mid(apex2SeedApprox, dSeed)))
        dimD.parameter.value = 1.25 * module_cm
        dPoint = drivingDedendum.endSketchPoint

        cSeed = _v2_add(apex2SeedApprox, _v2_scale(pinionDedendumDir, 1.25 * module_cm))
        pinionDedendum = skLines.addByTwoPoints(raw(apex2SeedApprox), raw(cSeed))
        geo.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        geo.addPerpendicular(pinionDedendum, pitchLine)
        dimC = dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint, Aligned,
            raw(_v2_mid(apex2SeedApprox, cSeed)))
        dimC.parameter.value = 1.25 * module_cm
        cPoint = pinionDedendum.endSketchPoint

        # --- The two root axes. ---
        drivingRootAxis = skLines.addByTwoPoints(raw(apexSeed), raw(dSeed))
        geo.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        geo.addCoincident(drivingRootAxis.endSketchPoint, dPoint)

        pinionRootAxis = skLines.addByTwoPoints(raw(apexSeed), raw(cSeed))
        geo.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        geo.addCoincident(pinionRootAxis.endSketchPoint, cPoint)

        # --- The module-length extensions. ---
        eSeed = _v2_add(aSeed, _v2_scale(pinionDir, module_cm))
        lineAE = skLines.addByTwoPoints(raw(aSeed), raw(eSeed))
        geo.addCoincident(lineAE.startSketchPoint, aPoint)
        geo.addCollinear(lineAE, pinionShaftAxis)
        ePoint = lineAE.endSketchPoint

        lineCE = skLines.addByTwoPoints(raw(cSeed), raw(eSeed))
        geo.addCoincident(lineCE.startSketchPoint, cPoint)
        geo.addCoincident(lineCE.endSketchPoint, ePoint)
        geo.addPerpendicular(lineAE, lineCE)

        fSeed = _v2_add(bSeed, _v2_scale(drivingDir, module_cm))
        lineBF = skLines.addByTwoPoints(raw(bSeed), raw(fSeed))
        geo.addCoincident(lineBF.startSketchPoint, bPoint)
        geo.addCollinear(lineBF, drivingShaftAxis)
        fPoint = lineBF.endSketchPoint

        lineDF = skLines.addByTwoPoints(raw(dSeed), raw(fSeed))
        geo.addCoincident(lineDF.startSketchPoint, dPoint)
        geo.addCoincident(lineDF.endSketchPoint, fPoint)
        geo.addPerpendicular(lineBF, lineDF)

        # --- The base-height chains. ---
        gSeed = _v2_add(eSeed, _v2_scale(pinionDir, module_cm))
        lineEG = skLines.addByTwoPoints(raw(eSeed), raw(gSeed))
        geo.addCoincident(lineEG.startSketchPoint, ePoint)
        geo.addCollinear(lineEG, lineAE)
        gPoint = lineEG.endSketchPoint

        hSeed = _v2_add(cSeed, _v2_scale(pinionDedendumDir, module_cm))
        lineCH = skLines.addByTwoPoints(raw(cSeed), raw(hSeed))
        geo.addCoincident(lineCH.startSketchPoint, cPoint)
        geo.addCollinear(lineCH, pinionDedendum)
        hPoint = lineCH.endSketchPoint

        lineGH = skLines.addByTwoPoints(raw(gSeed), raw(hSeed))
        geo.addCoincident(lineGH.startSketchPoint, gPoint)
        geo.addCoincident(lineGH.endSketchPoint, hPoint)
        # Required in Fusion, omitted in the proof harness -- see S07.
        geo.addPerpendicular(lineEG, lineGH)

        iSeed = _v2_add(fSeed, _v2_scale(drivingDir, module_cm))
        lineFI = skLines.addByTwoPoints(raw(fSeed), raw(iSeed))
        geo.addCoincident(lineFI.startSketchPoint, fPoint)
        geo.addCollinear(lineFI, lineBF)
        iPoint = lineFI.endSketchPoint

        jSeed = _v2_add(dSeed, _v2_scale(drivingDedendumDir, module_cm))
        lineDJ = skLines.addByTwoPoints(raw(dSeed), raw(jSeed))
        geo.addCoincident(lineDJ.startSketchPoint, dPoint)
        geo.addCollinear(lineDJ, drivingDedendum)
        jPoint = lineDJ.endSketchPoint

        lineIJ = skLines.addByTwoPoints(raw(iSeed), raw(jSeed))
        geo.addCoincident(lineIJ.startSketchPoint, iPoint)
        geo.addCoincident(lineIJ.endSketchPoint, jPoint)
        geo.addPerpendicular(lineFI, lineIJ)

        # --- The two base-height offsets. ---
        offsetDimB = dims.addOffsetDimension(
            bApex2Drop, lineIJ, raw(_v2_mid(bApex2DropSeed, iSeed)))
        offsetDimB.parameter.value = self._drivingBaseHeight_cm

        offsetDimA = dims.addOffsetDimension(
            aApex2Drop, lineGH, raw(_v2_mid(aApex2DropSeed, gSeed)))
        offsetDimA.parameter.value = self._pinionBaseHeight_cm

        # --- Close the figure. ---
        geo.addCoincident(iPoint, projectedCenter)

        # --- The tooth-centre points K and L. ---
        kSeed = _line_intersect_2d(apexSeed, pinionDir, apex2SeedApprox, pinionDedendumDir)
        lineGK = skLines.addByTwoPoints(raw(gSeed), raw(kSeed))
        geo.addCoincident(lineGK.startSketchPoint, gPoint)
        kPoint = lineGK.endSketchPoint
        geo.addCoincident(kPoint, pinionShaftAxis)
        geo.addCoincident(kPoint, pinionDedendum)
        lineCK = skLines.addByTwoPoints(raw(cSeed), raw(kSeed))
        geo.addCoincident(lineCK.startSketchPoint, cPoint)
        geo.addCoincident(lineCK.endSketchPoint, kPoint)

        lSeed = _line_intersect_2d(apexSeed, drivingDir, apex2SeedApprox, drivingDedendumDir)
        lineIL = skLines.addByTwoPoints(raw(iSeed), raw(lSeed))
        geo.addCoincident(lineIL.startSketchPoint, iPoint)
        lPoint = lineIL.endSketchPoint
        geo.addCoincident(lPoint, drivingShaftAxis)
        geo.addCoincident(lPoint, drivingDedendum)
        lineDL = skLines.addByTwoPoints(raw(dSeed), raw(lSeed))
        geo.addCoincident(lineDL.startSketchPoint, dPoint)
        geo.addCoincident(lineDL.endSketchPoint, lPoint)

        # --- Tooth-centre point K' (the Tooth Spacing offset). ---
        toothSpacing_cm = self._toothSpacing_cm
        if toothSpacing_cm == 0:
            kPrimePoint = kPoint
            kPrimeSeed = kSeed
            lineCKPrime = lineCK
        else:
            kPrimeSeed = _v2_add(kSeed, _v2_scale(pinionDedendumDir, toothSpacing_cm))
            lineKKPrime = skLines.addByTwoPoints(raw(kSeed), raw(kPrimeSeed))
            geo.addCoincident(lineKKPrime.startSketchPoint, kPoint)
            kPrimePoint = lineKKPrime.endSketchPoint
            geo.addCoincident(kPrimePoint, pinionDedendum)
            dimKKPrime = dims.addDistanceDimension(
                lineKKPrime.startSketchPoint, lineKKPrime.endSketchPoint, Aligned,
                raw(_v2_mid(kSeed, kPrimeSeed)))
            dimKKPrime.parameter.value = toothSpacing_cm
            lineCKPrime = skLines.addByTwoPoints(raw(cSeed), raw(kPrimeSeed))
            geo.addCoincident(lineCKPrime.startSketchPoint, cPoint)
            geo.addCoincident(lineCKPrime.endSketchPoint, kPrimePoint)

        if toothSpacing_cm == 0:
            lPrimePoint = lPoint
            lPrimeSeed = lSeed
            lineDLPrime = lineDL
        else:
            lPrimeSeed = _v2_add(lSeed, _v2_scale(drivingDedendumDir, toothSpacing_cm))
            lineLLPrime = skLines.addByTwoPoints(raw(lSeed), raw(lPrimeSeed))
            geo.addCoincident(lineLLPrime.startSketchPoint, lPoint)
            lPrimePoint = lineLLPrime.endSketchPoint
            geo.addCoincident(lPrimePoint, drivingDedendum)
            dimLLPrime = dims.addDistanceDimension(
                lineLLPrime.startSketchPoint, lineLLPrime.endSketchPoint, Aligned,
                raw(_v2_mid(lSeed, lPrimeSeed)))
            dimLLPrime.parameter.value = toothSpacing_cm
            lineDLPrime = skLines.addByTwoPoints(raw(dSeed), raw(lPrimeSeed))
            geo.addCoincident(lineDLPrime.startSketchPoint, dPoint)
            geo.addCoincident(lineDLPrime.endSketchPoint, lPrimePoint)

        # --- Resolve the Maximum Face Width, from SOLVED geometry. ---
        aG, bG = aPoint.geometry, bPoint.geometry
        cG, dG = cPoint.geometry, dPoint.geometry
        hG, jG = hPoint.geometry, jPoint.geometry
        distA_CH = _perp_dist_point_to_segment_2d((aG.x, aG.y), (cG.x, cG.y), (hG.x, hG.y))
        distB_DJ = _perp_dist_point_to_segment_2d((bG.x, bG.y), (dG.x, dG.y), (jG.x, jG.y))
        maxFaceWidth_cm = 0.95 * min(distA_CH, distB_DJ)

        # --- Resolve Face Width. ---
        if self._faceWidth_cm != 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width exceeds the maximum of {to_mm(maxFaceWidth_cm)} mm '
                    f'(got {to_mm(self._faceWidth_cm)} mm)')
            faceWidthResolved_cm = self._faceWidth_cm
        else:
            faceWidthResolved_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)
        self._faceWidthResolved_cm = faceWidthResolved_cm

        # --- Resolve the Toe Radii and the Root Length. ---
        apexToDed_cm = math.sqrt(R_cm ** 2 + (1.25 * module_cm) ** 2)

        def resolve_toe_radius(r_cm, gamma, userValue_cm, label):
            ceiling = (r_cm - 1.25 * module_cm * math.cos(gamma)) * (
                1.0 - faceWidthResolved_cm / R_cm)
            if userValue_cm != 0:
                if userValue_cm >= ceiling:
                    raise Exception(
                        f'{label} Toe Radius must be strictly below the ceiling of '
                        f'{to_mm(ceiling)} mm (got {to_mm(userValue_cm)} mm)')
                toeRadius = userValue_cm
                defaulted = False
            else:
                toeRadius = r_cm - faceWidthResolved_cm / math.sin(gamma)
                defaulted = True
            gammaRoot = gamma - math.atan((1.25 * module_cm) / R_cm)
            toeLimit = apexToDed_cm - toeRadius / math.sin(gammaRoot)
            return toeRadius, ceiling, gammaRoot, toeLimit, defaulted

        (pinionToeRadiusResolved_cm, pinionToeCeiling_cm, gammaRoot_p, pinionToeLimit_cm,
         pinionToeDefaulted) = resolve_toe_radius(
            PPD_cm / 2.0, gamma_p, self._pinionToeRadius_cm, 'Pinion Gear')
        (drivingToeRadiusResolved_cm, drivingToeCeiling_cm, gammaRoot_g, drivingToeLimit_cm,
         drivingToeDefaulted) = resolve_toe_radius(
            DPD_cm / 2.0, gamma_g, self._drivingToeRadius_cm, 'Driving Gear')

        rootLength0_cm = faceWidthResolved_cm * apexToDed_cm / R_cm
        minToeLimit_cm = min(pinionToeLimit_cm, drivingToeLimit_cm)
        if self._toeExtension > 0 and minToeLimit_cm < rootLength0_cm:
            if pinionToeLimit_cm <= drivingToeLimit_cm:
                bindingLabel, bindingCeiling, bindingDefaulted = (
                    'Pinion Gear', pinionToeCeiling_cm, pinionToeDefaulted)
            else:
                bindingLabel, bindingCeiling, bindingDefaulted = (
                    'Driving Gear', drivingToeCeiling_cm, drivingToeDefaulted)
            if bindingDefaulted:
                raise Exception(
                    f'{bindingLabel}: Toe Extension above 0 needs a Toe Radius below the '
                    f'ceiling of {to_mm(bindingCeiling)} mm; Toe Extension 0 still resolves')

        rootLength_cm = rootLength0_cm + (self._toeExtension / 100.0) * 0.99 * (
            minToeLimit_cm - rootLength0_cm)
        self._rootLength_cm = rootLength_cm

        # --- The toe line M->N (pinion). ---
        cDir = _v2_unit(_v2_sub(cSeed, apexSeed))
        distApexM = apexToDed_cm - rootLength_cm
        mSeed = _v2_add(apexSeed, _v2_scale(cDir, distApexM))
        mDistFromPinionAxis = _perp_dist_point_to_line_2d(mSeed, apexSeed, pinionDir)
        slideM = (mDistFromPinionAxis - pinionToeRadiusResolved_cm) / math.cos(gamma_p)
        nSeed = _v2_add(mSeed, _v2_scale(pinionDedendumDir, slideM))

        lineMN = skLines.addByTwoPoints(raw(mSeed), raw(nSeed))
        mPoint = lineMN.startSketchPoint
        nPoint = lineMN.endSketchPoint
        geo.addCoincident(mPoint, pinionRootAxis)
        geo.addParallel(lineMN, lineCH)
        offsetDimMN = dims.addOffsetDimension(
            lineCH, lineMN, raw(_v2_mid(mSeed, cSeed)))
        offsetDimMN.parameter.value = rootLength_cm * R_cm / apexToDed_cm

        lineMC = skLines.addByTwoPoints(raw(mSeed), raw(cSeed))
        geo.addCoincident(lineMC.startSketchPoint, mPoint)
        geo.addCoincident(lineMC.endSketchPoint, cPoint)

        # --- The front face A'->N. ---
        axisDistN = _v2_dot(_v2_sub(nSeed, apexSeed), pinionDir)
        aPrimeSeed = _v2_add(apexSeed, _v2_scale(pinionDir, axisDistN))
        lineNAprime = skLines.addByTwoPoints(raw(nSeed), raw(aPrimeSeed))
        geo.addCoincident(lineNAprime.startSketchPoint, nPoint)
        aPrimePoint = lineNAprime.endSketchPoint
        geo.addCoincident(aPrimePoint, pinionShaftAxis)
        geo.addPerpendicular(lineNAprime, pinionShaftAxis)
        dimNAprime = dims.addDistanceDimension(
            lineNAprime.startSketchPoint, lineNAprime.endSketchPoint, Aligned,
            raw(_v2_mid(nSeed, aPrimeSeed)))
        dimNAprime.parameter.value = pinionToeRadiusResolved_cm

        # --- The shaft-axis edge's first vertex, A'->G. ---
        lineAprimeG = skLines.addByTwoPoints(raw(aPrimeSeed), raw(gSeed))
        geo.addCoincident(lineAprimeG.startSketchPoint, aPrimePoint)
        geo.addCoincident(lineAprimeG.endSketchPoint, gPoint)

        # --- The driving side: O->P, the mirror of M->N. ---
        dDir = _v2_unit(_v2_sub(dSeed, apexSeed))
        distApexO = apexToDed_cm - rootLength_cm
        oSeed = _v2_add(apexSeed, _v2_scale(dDir, distApexO))
        oDistFromDrivingAxis = _perp_dist_point_to_line_2d(oSeed, apexSeed, drivingDir)
        slideO = (oDistFromDrivingAxis - drivingToeRadiusResolved_cm) / math.cos(gamma_g)
        pSeed = _v2_add(oSeed, _v2_scale(drivingDedendumDir, slideO))

        lineOP = skLines.addByTwoPoints(raw(oSeed), raw(pSeed))
        oPoint = lineOP.startSketchPoint
        pPoint = lineOP.endSketchPoint
        geo.addCoincident(oPoint, drivingRootAxis)
        geo.addParallel(lineOP, lineDJ)
        offsetDimOP = dims.addOffsetDimension(
            lineDJ, lineOP, raw(_v2_mid(oSeed, dSeed)))
        offsetDimOP.parameter.value = rootLength_cm * R_cm / apexToDed_cm

        lineOD = skLines.addByTwoPoints(raw(oSeed), raw(dSeed))
        geo.addCoincident(lineOD.startSketchPoint, oPoint)
        geo.addCoincident(lineOD.endSketchPoint, dPoint)

        # --- The driving front face, B'->P. ---
        axisDistP = _v2_dot(_v2_sub(pSeed, apexSeed), drivingDir)
        bPrimeSeed = _v2_add(apexSeed, _v2_scale(drivingDir, axisDistP))
        linePBprime = skLines.addByTwoPoints(raw(pSeed), raw(bPrimeSeed))
        geo.addCoincident(linePBprime.startSketchPoint, pPoint)
        bPrimePoint = linePBprime.endSketchPoint
        geo.addCoincident(bPrimePoint, drivingShaftAxis)
        geo.addPerpendicular(linePBprime, drivingShaftAxis)
        dimPBprime = dims.addDistanceDimension(
            linePBprime.startSketchPoint, linePBprime.endSketchPoint, Aligned,
            raw(_v2_mid(pSeed, bPrimeSeed)))
        dimPBprime.parameter.value = drivingToeRadiusResolved_cm

        # --- The driving shaft-axis edge, B'->I. ---
        lineBprimeI = skLines.addByTwoPoints(raw(bPrimeSeed), raw(iSeed))
        geo.addCoincident(lineBprimeI.startSketchPoint, bPrimePoint)
        geo.addCoincident(lineBprimeI.endSketchPoint, iPoint)

        # --- Gate the sketch. ---
        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        apexSolved = apexPoint.geometry
        self._apex2d = (apexSolved.x, apexSolved.y)

        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'pitchDiameter_cm': PPD_cm,
            'gamma': gamma_p,
            'toothCentrePoint': kPrimePoint,
            'toothCentreLine': lineCKPrime,
            'hexVertices': [aPrimePoint, gPoint, hPoint, cPoint, mPoint, nPoint],
            'shaftEdgePointsSeed': (aPrimePoint, gPoint),
            'toeEdge': (mPoint, nPoint),
            'heelEdge': (cPoint, hPoint),
            'toeConePoint': mPoint,
            'heelConePoint': cPoint,
            'rootAxis': pinionRootAxis,
            'boreDiameter_cm': self._pinionBore_cm,
            'meshAngle': self._pinionMeshPhase(pinionTeeth),
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'pitchDiameter_cm': DPD_cm,
            'gamma': gamma_g,
            'toothCentrePoint': lPrimePoint,
            'toothCentreLine': lineDLPrime,
            'hexVertices': [bPrimePoint, iPoint, jPoint, dPoint, oPoint, pPoint],
            'shaftEdgePointsSeed': (bPrimePoint, iPoint),
            'toeEdge': (oPoint, pPoint),
            'heelEdge': (dPoint, jPoint),
            'toeConePoint': oPoint,
            'heelConePoint': dPoint,
            'rootAxis': drivingRootAxis,
            'boreDiameter_cm': self._drivingBore_cm,
            'meshAngle': math.pi / drivingTeeth,
        }
        return pinionCtx, drivingCtx

    # -----------------------------------------------------------------
    # S09-S12: the tooth plane, tooth sketch, helper plane and tooth axis.
    # -----------------------------------------------------------------

    def _buildVirtualSpurProfile(self, ctx):
        designComponent: adsk.fusion.Component = self.designComponent

        # S09: the {gearLabel} Plane.
        toothPlane = plane_by_angle(
            designComponent, ctx['toothCentreLine'], self._gearProfilesPlane, 90)
        toothPlane.name = f'{ctx["label"]} Plane'
        ctx['toothPlane'] = toothPlane

        # S10: the {gearLabel} Tooth sketch.
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{ctx["label"]} Tooth'

        pitchDia_cm = ctx['pitchDiameter_cm']
        gamma = ctx['gamma']
        virtualPitchRadius_mm = (pitchDia_cm * 10.0 / 2.0) / math.cos(gamma)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / self._module))

        proxy = VirtualSpurProxy(module_mm=self._module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(ctx['toothCentrePoint'], angle=math.radians(180))
        embedded = proxy._lastToothEmbedded

        if not toothSketch.isFullyConstrained:
            futil.log(
                f'{ctx["label"]} Tooth sketch not fully constrained '
                f'(exempt: labelled -- [PB-TEXT-HOLDS-DOF])', force_console=True)

        ctx['toothSketch'] = toothSketch
        ctx['embedded'] = embedded
        ctx['virtualTeeth'] = virtualTeeth

        # S11: the tooth-axis helper plane.
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            ctx['toothCentreLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = f'{ctx["label"]} Tooth Axis Helper'

        # S12: the {gearLabel} Tooth Axis.
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{ctx["label"]} Tooth Axis'
        ctx['toothAxis'] = toothAxis

    # -----------------------------------------------------------------
    # S13: the {gearLabel} Profile sketch.
    # -----------------------------------------------------------------

    def _buildProfileSketch(self, ctx) -> adsk.fusion.SketchLine:
        designComponent: adsk.fusion.Component = self.designComponent
        profileSketch = designComponent.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{ctx["label"]} Profile'

        verts = [
            profileSketch.sketchPoints.add(profileSketch.modelToSketchSpace(src.worldGeometry))
            for src in ctx['hexVertices']
        ]

        profLines = profileSketch.sketchCurves.sketchLines
        edges = []
        for i in range(6):
            p1 = verts[i]
            p2 = verts[(i + 1) % 6]
            edges.append(profLines.addByTwoPoints(p1, p2))

        for e in edges:
            e.startSketchPoint.isFixed = True
            e.endSketchPoint.isFixed = True

        shaftAxisEdge = edges[0]

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{profileSketch.name} sketch is not fully constrained')

        ctx['profileSketch'] = profileSketch
        ctx['shaftAxisEdge'] = shaftAxisEdge
        return shaftAxisEdge

    # -----------------------------------------------------------------
    # S17-S25: the spiral tooth-body transform, called once per gear from
    # _createGearBody on the freshly lofted uncut apex->heel tooth.
    # -----------------------------------------------------------------

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                             toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                             shaftAxisEdge: adsk.fusion.SketchLine, apexWorld: adsk.core.Point3D,
                             apexSketchPoint: adsk.fusion.SketchPoint, toeMid: adsk.core.Point3D,
                             heelMid: adsk.core.Point3D, toeConeWorld: adsk.core.Point3D,
                             heelConeWorld: adsk.core.Point3D,
                             parentToothPlane: adsk.fusion.ConstructionPlane, gearLabel: str,
                             teethNumber: int, gamma: float) -> adsk.fusion.BRepBody:
        if self._spiralAngle_rad <= 0:
            return cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        # --- A. Gate and frame. ---
        startW: adsk.core.Point3D = shaftAxisEdge.startSketchPoint.worldGeometry
        endW: adsk.core.Point3D = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir: adsk.core.Vector3D = _unit_vec3(startW, endW)

        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec: adsk.core.Vector3D = _unit_vec3(apexWorld, heelConeWorld)
        v: adsk.core.Vector3D = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal: adsk.core.Vector3D = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            diff: adsk.core.Vector3D = _sub_vec3(p, apexWorld)
            return diff.dotProduct(coneVec)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- S17: the {gear} Cone Element sketch. ---
        coneElemSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneElemSketch.name = f'{gearLabel} Cone Element'
        heelEndPt = adsk.core.Point3D.create(
            apexWorld.x + R_heel * coneVec.x,
            apexWorld.y + R_heel * coneVec.y,
            apexWorld.z + R_heel * coneVec.z)
        coneElementLine = coneElemSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, heelEndPt)

        # --- S18: the {gear} Trace Plane. ---
        tracePlane = plane_by_angle(designComponent, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # --- S19: the {gear} 2D Tooth Trace sketch. ---
        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        r_c = self._cutterRadius_cm if self._cutterRadius_cm != 0 else R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign

        Cx = R_mean - r_c * math.sin(self._spiralAngle_rad)
        Cy = handSign * r_c * math.cos(self._spiralAngle_rad)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        def tanW(px, py):
            return combine_point(apexWorld, px, coneVec, py, v)

        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            tanW(Cx, Cy), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        diamDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, tanW(Cx + r_c, Cy))
        diamDim.parameter.value = 2.0 * r_c

        toeW = tanW(toe2d[0], toe2d[1])
        meanW = tanW(R_mean, 0.0)
        heelW = tanW(heel2d[0], heel2d[1])
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(toeW, meanW, heelW)
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radiusDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, meanW)
        radiusDim.parameter.value = r_c

        # --- G (part 1): the twist magnitude. ---
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        # --- E. Slice the straight tooth. ---
        planeOrigin = parentToothPlane.geometry.origin
        planeNormal = parentToothPlane.geometry.normal
        apexVec: adsk.core.Vector3D = _sub_vec3(apexWorld, planeOrigin)
        sign = 1.0 if apexVec.dotProduct(planeNormal) > 0 else -1.0

        def build_offsets(s):
            return [s * (k + 1) * span / 6.0 for k in range(8)]

        offsets = build_offsets(sign)
        pieces = slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
        if len(pieces) <= 1:
            sign = -sign
            offsets = build_offsets(sign)
            pieces = slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)
            if len(pieces) <= 1:
                raise Exception(
                    f'{gearLabel}: slice produced {len(pieces)} piece(s) '
                    f'(span={span}, signs tried=+/-{abs(sign)}) -- cut planes missed the tooth')

        # --- F. Order and drop the apex scrap. ---
        sortedPieces = sorted(pieces, key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap = sortedPieces[0]
        segments = sortedPieces[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                f'{gearLabel}: no segments remain after dropping the apex scrap '
                f'(slice produced {len(pieces)} piece(s))')

        def heel_face(body):
            best, bestDist = None, None
            for face in body.faces:
                d = distAlong(face.centroid)
                if bestDist is None or d > bestDist:
                    bestDist, best = d, face
            return best, bestDist

        def toe_face(body):
            best, bestDist = None, None
            for face in body.faces:
                d = distAlong(face.centroid)
                if bestDist is None or d < bestDist:
                    bestDist, best = d, face
            return best, bestDist

        # --- G (part 2): the twist itself. ---
        axisVec = adsk.core.Vector3D.create(axisDir.x, axisDir.y, axisDir.z)
        for seg in segments:
            _, segHeelDist = heel_face(seg)
            ang = -handSign * total * (R_mean - segHeelDist) / span
            if ang == 0:
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisVec, apexWorld)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # --- H. Lengthwise crown, keyed on the post-twist heel-face distAlong. ---
        segHeelInfo = []
        for seg in segments:
            face, dist = heel_face(seg)
            segHeelInfo.append((seg, face, dist))
        segHeelInfo.sort(key=lambda t: t[2])
        heelSegment = segHeelInfo[-1][0]

        try:
            self.designOccurrence.activate()
            for (seg, face, dist) in segHeelInfo:
                if seg is heelSegment:
                    continue
                u = (R_heel - dist) / span
                factor = 1.0 - _CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor} is non-positive '
                        f'(u={u}, total={total})')

                def perp_axis_dist(pt):
                    vec: adsk.core.Vector3D = _sub_vec3(pt, apexWorld)
                    along = vec.dotProduct(axisDir)
                    px = vec.x - along * axisDir.x
                    py = vec.y - along * axisDir.y
                    pz = vec.z - along * axisDir.z
                    return math.sqrt(px * px + py * py + pz * pz)

                vertList = [(perp_axis_dist(vt.geometry), vt.geometry) for vt in face.vertices]
                vertList.sort(key=lambda t: t[0])
                rootA, rootB = vertList[0][1], vertList[1][1]
                baseWorld = _midpoint3(rootA, rootB)

                baseSketch = designComponent.sketches.add(face)
                basePoint = baseSketch.sketchPoints.add(
                    baseSketch.modelToSketchSpace(baseWorld))

                bodyColl = adsk.core.ObjectCollection.create()
                bodyColl.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    bodyColl, basePoint, adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # --- I. Loft -> curved tooth, re-sorted after twist AND crown. ---
        segHeelPost = []
        for seg in segments:
            face, dist = heel_face(seg)
            segHeelPost.append((seg, face, dist))
        segHeelPost.sort(key=lambda t: t[2])
        orderedSegs = [t[0] for t in segHeelPost]
        orderedHeelFaces = [t[1] for t in segHeelPost]

        toeFace, _ = toe_face(orderedSegs[0])

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toeFace)
        for face in orderedHeelFaces:
            loftInput.loftSections.add(face)
        curvedToothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
        curvedToothBody.name = f'{gearLabel} Spiral Tooth'

        for seg in orderedSegs:
            designComponent.features.removeFeatures.add(seg)

        # --- J. Flush trim and mesh phase (phase itself applied at S31). ---
        return cut_conical_ends(
            designComponent, curvedToothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # -----------------------------------------------------------------
    # S28-S30: the bore.
    # -----------------------------------------------------------------

    def _cutBore(self, ctx, shaftAxisEdge: adsk.fusion.SketchLine,
                 gearBody: adsk.fusion.BRepBody):
        if not self._boreEnable:
            return
        designComponent: adsk.fusion.Component = self.designComponent

        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{ctx["label"]} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{ctx["label"]} Bore'
        boreRadius_cm = ctx['boreDiameter_cm'] / 2.0
        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreRadius_cm)
        circle.centerSketchPoint.isFixed = True
        diamDim = boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreRadius_cm, 0, 0))
        diamDim.parameter.value = ctx['boreDiameter_cm']

        if not boreSketch.isFullyConstrained:
            raise Exception(f'{boreSketch.name} sketch is not fully constrained')

        boreProfile = boreSketch.profiles.item(0)
        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2.0 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [gearBody]
        designComponent.features.extrudeFeatures.add(extrudeInput)

    # -----------------------------------------------------------------
    # S31: the pinion's extra mesh phase.
    # -----------------------------------------------------------------

    def _pinionMeshPhase(self, pinionTeeth):
        return _PINION_MESH_PHASE_TEETH * 2.0 * math.pi / pinionTeeth

    # -----------------------------------------------------------------
    # S08 + S13-S32: build one gear's finished body.
    # -----------------------------------------------------------------

    def _createGearBody(self, ctx):
        # S08: create the {gearLabel} Gear component, a child of Bevel Gear.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{ctx["label"]} Gear'

        # S09-S12.
        self._buildVirtualSpurProfile(ctx)

        # S13.
        shaftAxisEdge: adsk.fusion.SketchLine = self._buildProfileSketch(ctx)
        profileSketch: adsk.fusion.Sketch = ctx['profileSketch']

        # S14: revolve the Gear Body.
        profile = profileSketch.profiles.item(0)
        revolveInput = self.designComponent.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        gearBody: adsk.fusion.BRepBody = self.designComponent.features.revolveFeatures.add(
            revolveInput).bodies.item(0)

        # S15: loft the Tooth Body.
        loftInput = self.designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        wantLines = 0 if ctx['embedded'] else 2
        toothProfile = find_profile_by_curve_counts(
            ctx['toothSketch'], nurbs=2, arcs=2, lines=wantLines)
        loftInput.loftSections.add(toothProfile)
        toothBody: adsk.fusion.BRepBody = self.designComponent.features.loftFeatures.add(
            loftInput).bodies.item(0)

        # S16 (straight) or S17-S25 (spiral): trim the tooth body.
        toeP1, toeP2 = ctx['toeEdge']
        heelP1, heelP2 = ctx['heelEdge']
        toeMid = _midpoint3(toeP1.worldGeometry, toeP2.worldGeometry)
        heelMid = _midpoint3(heelP1.worldGeometry, heelP2.worldGeometry)
        toeConeWorld = ctx['toeConePoint'].worldGeometry
        heelConeWorld = ctx['heelConePoint'].worldGeometry
        apexWorld = self._apexSketchPoint.worldGeometry

        toothKeeper = self._transformToothBody(
            self.designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            self._apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
            ctx['toothPlane'], ctx['label'], ctx['teeth'], ctx['gamma'])

        # S26: circular-pattern the tooth (serial -- the seed body is retired by the
        # pattern increment).
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(toothKeeper)
        patternInput = self.designComponent.features.circularPatternFeatures.createInput(
            bodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern: adsk.fusion.CircularPatternFeature = (
            self.designComponent.features.circularPatternFeatures.add(patternInput))

        # S27: combine-join.
        tools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(i))
        combineInput = self.designComponent.features.combineFeatures.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        self.designComponent.features.combineFeatures.add(combineInput)

        # S28-S30: the bore.
        self._cutBore(ctx, shaftAxisEdge, gearBody)

        # S31: the meshing rotation, still in Design, before the body is moved out.
        rotate_body_about_edge(
            self.designComponent, gearBody, shaftAxisEdge, ctx['meshAngle'])

        # S32: move the finished body into the gear component.
        gearBody.moveToComponent(gearOccurrence)

    # -----------------------------------------------------------------
    # S33: cleanup.
    # -----------------------------------------------------------------

    def _hideConstructionGeometry(self):
        hide_construction_geometry(self.bevelComponent)

    # -----------------------------------------------------------------
    # generate() / deleteComponent(): the entry points commands/_gear_command.py binds.
    # -----------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)

        self._createBevelAndDesignComponents(parentComponent)
        self._buildAnchorSketch(targetPlane, centerPoint)
        pinionCtx, drivingCtx = self._buildGearProfiles(targetPlane)

        # Pinion first, then driving -- profile and body interleaved per gear.
        self._createGearBody(pinionCtx)
        self._createGearBody(drivingCtx)

        self._hideConstructionGeometry()

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
