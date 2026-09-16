# Bevel gear generator.
#
# Generated from spec/bevelgear/steps.md (the compiled step list) and
# proof/bevelgear/ (the checked geometry). Every length below is in Fusion
# internal centimetres unless a name says otherwise; Module is the raw
# millimetre number the dialog returns, so every Module-derived length goes
# through to_cm() before it touches geometry (S02).

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator


# The 20 dialog input ids, in the dialog's display order (S01).
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
# Plane-local 2-D helpers. The §2 lattice is built entirely in the Gear
# Profiles sketch's own frame ([BEVEL-F-APEX-LOCAL]); no §2 POSITION is ever
# computed from a world round-trip.
# ---------------------------------------------------------------------------

def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _mul2(a, k):
    return (a[0] * k, a[1] * k)


def _neg2(a):
    return (-a[0], -a[1])


def _dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _len2(a):
    return math.hypot(a[0], a[1])


def _unit2(a):
    length = _len2(a)
    if length == 0:
        raise Exception('bevel gear: cannot take the direction of a zero-length vector')
    return (a[0] / length, a[1] / length)


def _rot2(a, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _left2(a):
    u = _unit2(a)
    return (-u[1], u[0])


def _dist_point_line2(p, base, direction):
    # Perpendicular distance from p to the infinite line through base along direction.
    return abs(_cross2(_sub2(p, base), _unit2(direction)))


def _p3(a):
    return adsk.core.Point3D.create(a[0], a[1], 0)


def _xy(point3d):
    return (point3d.x, point3d.y)


def _mid2(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _text_point(a, b, away=0.2):
    # A dimension text point beside the segment a->b: its midpoint pushed off
    # the segment so the annotation does not sit on the geometry.
    mid = _mid2(a, b)
    try:
        offset = _mul2(_left2(_sub2(b, a)), away)
    except Exception:
        offset = (away, away)
    return _p3(_add2(mid, offset))


def _world_mid(pointA, pointB):
    a = pointA.worldGeometry
    b = pointB.worldGeometry
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


def _vec(fromPoint, toPoint):
    return adsk.core.Vector3D.create(
        toPoint.x - fromPoint.x, toPoint.y - fromPoint.y, toPoint.z - fromPoint.z)


def _unit_vec(vector):
    out = adsk.core.Vector3D.create(vector.x, vector.y, vector.z)
    if not out.normalize():
        raise Exception('bevel gear: cannot normalize a zero-length direction')
    return out


class BevelGearCommandInputsConfigurator:
    """S01 — the command dialog. `configure` and `handle_input_changed` are bound
    by name from commands/bevelgear/entry.py, so both are part of the reproduced
    surface."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs: adsk.core.CommandInputs = cmd.commandInputs

        # Target Plane is added FIRST so it wins Fusion's auto-focus
        # ([PB-AUTOFOCUS-FIRST]).
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
        design: adsk.fusion.Design = get_design()
        parentInput.addSelection(design.rootComponent)

        # Every `mm`/`deg` default passed as createByReal is in Fusion INTERNAL
        # units ([PB-DIALOG-DEFAULT-UNITS]); the 90 deg Shaft Angle default goes
        # through createByString so the expression engine parses it.
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '',
                             adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
                             adsk.core.ValueInput.createByString('90 deg'))
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)
        inputs.addValueInput(INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
                             adsk.core.ValueInput.createByString('35 deg'))

        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
                             adsk.core.ValueInput.createByReal(0))
        inputs.addValueInput(INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # LAST, so the initial state is correct at the default psi = 35 deg.
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        # Recompute on EVERY input change, with no branch on which input changed.
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        # The spiral-only inputs show only when psi > 0. isVisible hides only the
        # dialog row — the input still exists and is read normally — so hiding is
        # purely cosmetic and cannot affect generation.
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return
        try:
            # The input's .expression, evaluated in internal RADIANS; never .value.
            design: adsk.fusion.Design = get_design()
            value = design.unitsManager.evaluateExpression(spiral.expression, 'rad')
        except Exception:
            # A half-typed expression can raise mid-edit; leave both shown.
            hand.isVisible = True
            cutter.isVisible = True
            return
        hand.isVisible = (value > 0)
        cutter.isVisible = (value > 0)


class BevelGearGenerator:
    """The bevel pair. Per-gear state travels in plain per-gear dicts and shared
    anchors on self; there is no GenerationContext and no base.Generator."""

    # S19: the lengthwise crown's tunable constant, 0 disables the crown.
    _CROWN_PER_RAD = 0.5
    # S25: the pinion's extra mesh phase, in whole teeth. 0 by default.
    _PINION_MESH_PHASE_TEETH = 0

    # [BEVEL-F-SEED-HELD] tolerance: 0.001 mm, i.e. 1e-4 cm in internal units.
    _SEED_TOLERANCE_CM = 1e-4

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.bevelComponent = adsk.fusion.Component.cast(None)
        self.designOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designComponent = adsk.fusion.Component.cast(None)

    # -- entry points ------------------------------------------------------

    def deleteComponent(self):
        # The error rollback the command entry point calls on an exception.
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.bevelComponent = adsk.fusion.Component.cast(None)
        self.designOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designComponent = adsk.fusion.Component.cast(None)

    def generate(self, inputs: adsk.core.CommandInputs):
        # S02 — read (and validate) EVERY input before anything creates an
        # occurrence.
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        module_cm = to_cm(module)
        ppd_cm = to_cm(module * pinionTeeth)
        dpd_cm = to_cm(module * drivingTeeth)
        shaftAngle_rad = math.radians(shaftAngle_deg)

        # Validation 3: the two pitch cone angles, then the Minimum Teeth floor.
        gamma_p = math.atan2(math.sin(shaftAngle_rad) * ppd_cm,
                             dpd_cm + ppd_cm * math.cos(shaftAngle_rad))
        gamma_g = shaftAngle_rad - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._coneDistance_cm = math.hypot(ppd_cm, dpd_cm)

        for (label, teeth, gamma) in (('Pinion', pinionTeeth, gamma_p),
                                      ('Driving', drivingTeeth, gamma_g)):
            floor = 5.27 * math.cos(gamma)
            if teeth < floor:
                raise Exception(
                    f'{label} Gear Teeth is {teeth}, below the Minimum Teeth floor of '
                    f'{floor:.2f} for a pitch cone angle of {math.degrees(gamma):.3f} deg. '
                    f'Below it the Minimum and Maximum Base Height cross and no base '
                    f'height can build this gear.')

        # Validation 4: per gear the Minimum and Maximum Base Height.
        drivingBaseHeight_cm = self._resolveBaseHeight(
            'Driving', self._drivingBaseHeight_cm, to_cm(module * drivingTeeth / 8.0),
            module_cm, dpd_cm / 2.0, gamma_g)
        pinionBaseHeight_cm = self._resolveBaseHeight(
            'Pinion', self._pinionBaseHeight_cm,
            drivingBaseHeight_cm * (pinionTeeth / drivingTeeth),
            module_cm, ppd_cm / 2.0, gamma_p)

        # S03 — the Bevel Gear and Design components.
        self._createComponents(parentComponent)

        # S04 — the Anchor Sketch.
        anchorLine = self._buildAnchorSketch(targetPlane, centerPoint)

        # S05 — the Gear Profiles Plane.
        self._buildGearProfilesPlane(anchorLine, targetPlane)

        # S06 — the §2 lattice, and the two per-gear contexts it ends with.
        pinionCtx, drivingCtx = self._buildGearProfiles(
            targetPlane, module, module_cm, ppd_cm, dpd_cm,
            pinionTeeth, drivingTeeth, shaftAngle_rad,
            gamma_p, gamma_g, pinionBaseHeight_cm, drivingBaseHeight_cm)

        # S07..S09 — once per gear, pinion first and driving second.
        for ctx in (pinionCtx, drivingCtx):
            self._buildToothPlane(ctx)
            self._buildVirtualSpurTooth(ctx, module)
            self._buildToothAxis(ctx)

        # S10..S26 — once per gear, pinion first and driving second, with the
        # profile and the body interleaved per gear.
        for ctx in (pinionCtx, drivingCtx):
            self._createGearComponent(ctx)
            self._createGearBody(ctx)
            self._moveBodyIntoGearComponent(ctx)

        # S27 — cleanup.
        solids.hide_construction_geometry(self.bevelComponent)

    # -- S02: read and validate -------------------------------------------

    def _evaluate(self, inputs: adsk.core.CommandInputs, inputId, units):
        # Every numeric and angle input is read by evaluating its expression.
        # The result is ALWAYS in Fusion internal units regardless of the unit
        # string ([PB-EVAL-EXPRESSION]).
        design: adsk.fusion.Design = get_design()
        return design.unitsManager.evaluateExpression(
            inputs.itemById(inputId).expression, units)

    def _readInputs(self, inputs: adsk.core.CommandInputs):

        def evaluate(inputId, units):
            return self._evaluate(inputs, inputId, units)

        parentSelection = get_selection(inputs, INPUT_ID_PARENT)
        planeSelection = get_selection(inputs, INPUT_ID_PLANE)
        centerSelection = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if not parentSelection or not planeSelection or not centerSelection:
            raise Exception(
                'Bevel gear: Target Plane, Center Point and Parent Component must '
                'each carry exactly one selection.')

        parentEntity = parentSelection[0]
        parentOccurrence = adsk.fusion.Occurrence.cast(parentEntity)
        parentComponent = parentOccurrence.component if parentOccurrence else parentEntity
        targetPlane = planeSelection[0]
        centerPoint = centerSelection[0]

        # Module is read with unit '' and therefore comes back as a raw number
        # meaning MILLIMETRES.
        module = evaluate(INPUT_ID_MODULE, '')
        # 'mm' and 'deg' expressions come back in Fusion internal units (cm,
        # radians) regardless of the unit string — use them as-is.
        shaftAngle_rad = evaluate(INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(evaluate(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evaluate(INPUT_ID_PINION_TEETH, '')))
        self._drivingBaseHeight_cm = evaluate(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evaluate(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        # A checkbox is read with get_boolean; get_value would reach for
        # .expression, which BoolValueCommandInput does not have ([PB-INPUT-READ]).
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = evaluate(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evaluate(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evaluate(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = evaluate(INPUT_ID_TOOTH_SPACING, 'mm')
        self._spiralAngle_rad = evaluate(INPUT_ID_SPIRAL_ANGLE, 'deg')
        selectedHand = inputs.itemById(INPUT_ID_HAND).selectedItem
        self._hand = selectedHand.name if selectedHand else _HAND_RIGHT
        self._cutterRadius_cm = evaluate(INPUT_ID_CUTTER_RADIUS, 'mm')
        # The raw unitless percentage the dialog returns, not a length.
        self._toeExtension_pct = evaluate(INPUT_ID_TOE_EXTENSION, '')
        self._drivingToeRadius_cm = evaluate(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        self._pinionToeRadius_cm = evaluate(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        shaftAngle_deg = math.degrees(shaftAngle_rad)
        spiralAngle_deg = math.degrees(self._spiralAngle_rad)

        # Validation 1.
        if module <= 0:
            raise Exception(f'Module must be greater than 0 (got {module}).')
        if drivingTeeth < 3:
            raise Exception(f'Driving Gear Teeth must be at least 3 (got {drivingTeeth}).')
        if pinionTeeth < 3:
            raise Exception(f'Pinion Gear Teeth must be at least 3 (got {pinionTeeth}).')
        for (label, value) in (
                ('Driving Gear Base Height', self._drivingBaseHeight_cm),
                ('Pinion Gear Base Height', self._pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', self._drivingBore_cm),
                ('Pinion Gear Bore Diameter', self._pinionBore_cm),
                ('Face Width', self._faceWidth_cm),
                ('Tooth Spacing', self._toothSpacing_cm),
                ('Cutter Radius', self._cutterRadius_cm)):
            if value < 0:
                raise Exception(f'{label} must not be negative (got {to_mm(value)} mm).')
        if self._toeExtension_pct < 0 or self._toeExtension_pct > 100:
            raise Exception(
                f'Toe Extension must be between 0 and 100 percent '
                f'(got {self._toeExtension_pct}).')
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be at least 0 deg and below 60 deg '
                f'(got {spiralAngle_deg:.3f} deg).')

        # Validation 2: the Shaft Angle against the Maximum Shaft Angle. Both
        # tooth counts are read and coerced first, because the limit depends on
        # them and the computed limit goes in the rejection message.
        ppd = module * pinionTeeth
        dpd = module * drivingTeeth
        singularity_deg = math.degrees(math.acos(-min(ppd, dpd) / max(ppd, dpd)))
        maximumShaftAngle_deg = min(singularity_deg, 150.0)
        if shaftAngle_deg < 30.0:
            raise Exception(
                f'Shaft Angle must be at least 30 deg (got {shaftAngle_deg:.3f} deg).')
        # The acos half is EXCLUSIVE — it is a hard cone-angle singularity — and
        # the 150 deg half is INCLUSIVE.
        if singularity_deg <= 150.0:
            if shaftAngle_deg >= singularity_deg:
                raise Exception(
                    f'Shaft Angle {shaftAngle_deg:.3f} deg must be strictly below the '
                    f'Maximum Shaft Angle of {maximumShaftAngle_deg:.3f} deg for '
                    f'{drivingTeeth}/{pinionTeeth} teeth: at that angle a pitch cone '
                    f'angle reaches 90 deg and the gear turns inside out.')
        elif shaftAngle_deg > 150.0:
            raise Exception(
                f'Shaft Angle {shaftAngle_deg:.3f} deg exceeds the Maximum Shaft Angle '
                f'of {maximumShaftAngle_deg:.3f} deg.')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    def _resolveBaseHeight(self, label, userValue_cm, fallback_cm,
                           module_cm, pitchRadius_cm, gamma):
        # The bounds are measured from Apex 2's plane. The minimum is where the
        # heel edge stops running back inward; the maximum is deliberately
        # conservative, sitting one dedendum projection below the true crossing
        # of r * tan(gamma).
        minimum = 1.05 * 1.25 * module_cm * math.sin(gamma)
        maximum = 0.95 * (pitchRadius_cm - 1.25 * module_cm * math.cos(gamma)) * math.tan(gamma)
        if userValue_cm > 0:
            if userValue_cm < minimum:
                raise Exception(
                    f'{label} Gear Base Height {to_mm(userValue_cm):.4f} mm is below the '
                    f'Minimum Base Height of {to_mm(minimum):.4f} mm.')
            if userValue_cm > maximum:
                raise Exception(
                    f'{label} Gear Base Height {to_mm(userValue_cm):.4f} mm is above the '
                    f'Maximum Base Height of {to_mm(maximum):.4f} mm.')
            return userValue_cm
        # A fallback below the minimum is raised, a fallback above it is capped.
        return min(max(fallback_cm, minimum), maximum)

    # -- S03: components ---------------------------------------------------

    def _createComponents(self, parentComponent: adsk.fusion.Component):
        # NEVER activate any occurrence ([PB-NEVER-ACTIVATE],
        # [BEVEL-F-NEVER-ACTIVATE]): the Anchor Sketch is created on the user's
        # EXTERNAL, root-owned target plane, and an activated occurrence resolves
        # that plane in its own local frame.
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

    # -- S04: the Anchor Sketch -------------------------------------------

    def _buildAnchorSketch(self, targetPlane, centerPoint):
        # Directly on the user-selected target plane, whether the selection is a
        # ConstructionPlane or a PlanarFace ([PB-USE-SELECTED-PLANE]).
        sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint).item(0)
        # §2 re-projects THIS anchor-sketch point, never the raw user selection.
        self._anchorCenterPoint = projectedCenter

        centre = _xy(projectedCenter.geometry)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(centre[0] - 0.5, centre[1], 0),
            adsk.core.Point3D.create(centre[0] + 0.5, centre[1], 0))
        anchorLine.isConstruction = False

        # BOTH the point-on-line "intersection" and the midpoint, not the
        # midpoint alone.
        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        # The dimension simply locks the seeded 10 mm; its value is arbitrary
        # because this is only a reference line, so .parameter.value is NOT set.
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(centre[0], centre[1] + 0.3, 0))

        # Sketch-local, per [PB-REFLINE-DIRECTION]: works on any tilted target
        # plane where a world-axis lock would mis-orient.
        sketch.geometricConstraints.addHorizontal(anchorLine)

        self._gateFullyConstrained(sketch)
        self._anchorLine = anchorLine
        return anchorLine

    def _gateFullyConstrained(self, sketch: adsk.fusion.Sketch):
        if not sketch.isFullyConstrained:
            raise Exception(
                f'Bevel gear: sketch "{sketch.name}" is not fully constrained. '
                f'A free degree of freedom here is a generation defect.')

    # -- S05: the Gear Profiles Plane -------------------------------------

    def _buildGearProfilesPlane(self, anchorLine, targetPlane):
        planeInput = self.designComponent.constructionPlanes.createInput()
        # Pass the SketchLine DIRECTLY; never wrap it in Path.create
        # ([PB-CONSTRUCTION-PLANES]). The reference is the ORIGINAL targetPlane.
        planeInput.setByAngle(anchorLine,
                              adsk.core.ValueInput.createByString('90 deg'),
                              targetPlane)
        plane = self.designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # -- S06: the §2 lattice ----------------------------------------------

    def _buildGearProfiles(self, targetPlane, module, module_cm, ppd_cm, dpd_cm,
                           pinionTeeth, drivingTeeth, shaftAngle_rad,
                           gamma_p, gamma_g, bh_p, bh_g):
        sketch = self.designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines
        aligned = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        def construction(a, b):
            # [BEVEL-F-COINCIDENT-STYLE]: both ends from raw Point3D coordinates,
            # never by sharing an existing SketchPoint into the creation call.
            sketchLines: adsk.fusion.SketchLines = lines
            line = sketchLines.addByTwoPoints(_p3(a), _p3(b))
            line.isConstruction = True
            return line

        def lengthDim(line, a, b, value):
            sketchDimensions: adsk.fusion.SketchDimensions = dimensions
            dim = sketchDimensions.addDistanceDimension(
                line.startSketchPoint, line.endSketchPoint, aligned, _text_point(a, b))
            dim.parameter.value = value
            return dim

        # Project the ANCHOR SKETCH's centre point (not the raw user selection)
        # so the chain stays inside the Design component.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchorLine = sketch.project(self._anchorLine).item(0)

        c = _xy(projectedCenter.geometry)
        anchorDir = _unit2(_sub2(_xy(projectedAnchorLine.endSketchPoint.geometry),
                                 _xy(projectedAnchorLine.startSketchPoint.geometry)))
        perp = (-anchorDir[1], anchorDir[0])
        # The one-bit grow-side decision, and the ONLY permitted world use in
        # this section ([BEVEL-F-GROW-SIDE]).
        if not self._pointsTowardNormal(sketch, c, perp, targetPlane):
            perp = _neg2(perp)

        # ---- the closed forms -------------------------------------------
        ded = 1.25 * module_cm
        r_p = ppd_cm / 2.0
        r_g = dpd_cm / 2.0
        R = r_p / math.sin(gamma_p)
        apexDed = math.hypot(R, ded)

        uB = _neg2(perp)
        apex2d = _add2(c, _mul2(perp, R * math.cos(gamma_g) + bh_g))
        b2d = _add2(apex2d, _mul2(uB, R * math.cos(gamma_g)))

        # The pinion shaft direction: the driving direction rotated about the
        # Apex by the Shaft Angle. Form BOTH candidates and keep the one whose
        # endpoint has the greater X.
        candPlus = _rot2(uB, shaftAngle_rad)
        candMinus = _rot2(uB, -shaftAngle_rad)
        aPlus = _add2(apex2d, _mul2(candPlus, R * math.cos(gamma_p)))
        aMinus = _add2(apex2d, _mul2(candMinus, R * math.cos(gamma_p)))
        if aPlus[0] >= aMinus[0]:
            uA, sense = candPlus, 1.0
        else:
            uA, sense = candMinus, -1.0
        a2d = _add2(apex2d, _mul2(uA, R * math.cos(gamma_p)))

        pitchDir = _rot2(uB, sense * gamma_g)
        apex2_2d = _add2(apex2d, _mul2(pitchDir, R))

        # Seed the two dedendum ends by dot product against the shaft axes: the
        # pinion direction is the unit perpendicular to the Pitch Line whose dot
        # with unit(Apex->A) is positive (that dot is sin(gamma_p)).
        uPcand = (-pitchDir[1], pitchDir[0])
        uP = uPcand if _dot2(uPcand, uA) > 0 else _neg2(uPcand)
        uG = _neg2(uP)

        c2d = _add2(apex2_2d, _mul2(uP, ded))
        d2d = _add2(apex2_2d, _mul2(uG, ded))
        e2d = _add2(a2d, _mul2(uA, ded * math.sin(gamma_p)))
        f2d = _add2(b2d, _mul2(uB, ded * math.sin(gamma_g)))
        g2d = _add2(a2d, _mul2(uA, bh_p))
        i2d = _add2(b2d, _mul2(uB, bh_g))
        h2d = _add2(apex2_2d, _mul2(uP, bh_p / math.sin(gamma_p)))
        j2d = _add2(apex2_2d, _mul2(uG, bh_g / math.sin(gamma_g)))
        k2d = _add2(apex2_2d, _mul2(uP, r_p / math.cos(gamma_p)))
        l2d = _add2(apex2_2d, _mul2(uG, r_g / math.cos(gamma_g)))
        kp2d = _add2(apex2_2d, _mul2(uP, r_p / math.cos(gamma_p) + self._toothSpacing_cm))
        lp2d = _add2(apex2_2d, _mul2(uG, r_g / math.cos(gamma_g) + self._toothSpacing_cm))

        # ---- centre -> Apex ---------------------------------------------
        centerToApex = construction(c, apex2d)
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projectedAnchorLine)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint
        self._apex2d = apex2d

        # ---- Apex -> B, the Driving Gear Shaft Axis ----------------------
        drivingShaftAxis = construction(apex2d, b2d)
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        # addParallel, NEVER addVertical: a world-vertical lock mis-orients the
        # figure on a tilted target plane.
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # ---- Apex -> A, the Pinion Gear Shaft Axis -----------------------
        pinionShaftAxis = construction(apex2d, a2d)
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        # The text point goes inside the Sigma wedge so the dimension measures
        # Sigma and not its supplement ([PB-ANGULAR-DIM]).
        bisector = _add2(uA, uB)
        if _len2(bisector) < 1e-12:
            bisector = uP
        angleText = _add2(apex2d, _mul2(_unit2(bisector), ppd_cm / 4.0))
        angleDim = dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, _p3(angleText))
        angleDim.parameter.value = shaftAngle_rad
        pointA = pinionShaftAxis.endSketchPoint

        # ---- the two perpendicular drops that close at Apex 2 ------------
        dropA = construction(a2d, apex2_2d)
        constraints.addCoincident(dropA.startSketchPoint, pointA)
        constraints.addPerpendicular(dropA, pinionShaftAxis)
        lengthDim(dropA, a2d, apex2_2d, r_p)

        dropB = construction(b2d, apex2_2d)
        constraints.addCoincident(dropB.startSketchPoint, pointB)
        constraints.addPerpendicular(dropB, drivingShaftAxis)
        lengthDim(dropB, b2d, apex2_2d, r_g)

        constraints.addCoincident(dropB.endSketchPoint, dropA.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # ---- the Pitch Line and the two dedendum lines -------------------
        pitchLine = construction(apex2d, apex2_2d)
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        pinionDedendum = construction(apex2_2d, c2d)
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        lengthDim(pinionDedendum, apex2_2d, c2d, ded)
        pointC = pinionDedendum.endSketchPoint

        drivingDedendum = construction(apex2_2d, d2d)
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        lengthDim(drivingDedendum, apex2_2d, d2d, ded)
        pointD = drivingDedendum.endSketchPoint

        # ---- the two Root Axes -------------------------------------------
        pinionRootAxis = construction(apex2d, c2d)
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        drivingRootAxis = construction(apex2d, d2d)
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        # ---- E and F, the feet of the dedendum perpendiculars ------------
        lineAE = construction(a2d, e2d)
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        lineCE = construction(c2d, e2d)
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineCE, lineAE)

        lineBF = construction(b2d, f2d)
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        lineDF = construction(d2d, f2d)
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineDF, lineBF)

        # ---- G and H, the pinion heel ------------------------------------
        lineEG = construction(e2d, g2d)
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        # Collinear names A->E, never the Apex->A shaft axis further up the
        # chain ([PB-COLLINEAR-CHAIN], [BEVEL-F-COLLINEAR-CHAIN]).
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        lineCH = construction(c2d, h2d)
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        lineGH = construction(g2d, h2d)
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        # Required in Fusion and deliberately absent from the proof: it is what
        # supplies addOffsetDimension's parallelism.
        constraints.addPerpendicular(lineEG, lineGH)

        # ---- I and J, the driving heel -----------------------------------
        lineFI = construction(f2d, i2d)
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        lineDJ = construction(d2d, j2d)
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        lineIJ = construction(i2d, j2d)
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # ---- the two base-height offsets ---------------------------------
        # Between the B->Apex2 DROP and I->J, already parallel by construction,
        # so no extra parallel constraint ([PB-OFFSET-DIM]).
        drivingOffset = dimensions.addOffsetDimension(
            dropB, lineIJ, _p3(_mid2(_mid2(b2d, apex2_2d), _mid2(i2d, j2d))))
        drivingOffset.parameter.value = bh_g

        pinionOffset = dimensions.addOffsetDimension(
            dropA, lineGH, _p3(_mid2(_mid2(a2d, apex2_2d), _mid2(g2d, h2d))))
        pinionOffset.parameter.value = bh_p

        # ---- close the figure --------------------------------------------
        constraints.addCoincident(pointI, projectedCenter)

        # ---- resolve the two bounds that could not resolve earlier --------
        # A, B, C, D, H and J all exist and are SOLVED by now, so the Maximum
        # Face Width — and with it the Root Length, the Toe Radii and each
        # gear's Maximum Bore Diameter — resolves here. It has to resolve BEFORE
        # the A'->G line below, because A' is seeded at N's along-shaft
        # coordinate and N needs the Root Length.
        # Read the SOLVED .geometry ([PB-SOLVED-GEOMETRY]), never the seeds.
        solvedA = _xy(pointA.geometry)
        solvedB = _xy(pointB.geometry)
        solvedC = _xy(pointC.geometry)
        solvedD = _xy(pointD.geometry)
        solvedH = _xy(pointH.geometry)
        solvedJ = _xy(pointJ.geometry)
        # Compute BOTH distances and take the minimum: either gear can be the
        # binding side, never the pinion by name.
        distA = _dist_point_line2(solvedA, solvedC, _sub2(solvedH, solvedC))
        distB = _dist_point_line2(solvedB, solvedD, _sub2(solvedJ, solvedD))
        maximumFaceWidth = 0.95 * min(distA, distB)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maximumFaceWidth:
                raise Exception(
                    f'Face Width {to_mm(self._faceWidth_cm):.4f} mm exceeds the Maximum '
                    f'Face Width of {to_mm(maximumFaceWidth):.4f} mm; beyond it the '
                    f'revolved profile crosses its own axis of revolution.')
            faceWidth = self._faceWidth_cm
        else:
            faceWidth = min(self._coneDistance_cm / 6.0, maximumFaceWidth)
        self._faceWidthResolved_cm = faceWidth

        rootLengthAtZero = faceWidth * apexDed / R
        gammaRoot_p = gamma_p - math.atan(ded / R)
        gammaRoot_g = gamma_g - math.atan(ded / R)

        toeRadii = {}
        toeLimits = {}
        toeCeilings = {}
        for (label, pitchRadius, gamma, gammaRoot, userToe) in (
                ('Pinion', r_p, gamma_p, gammaRoot_p, self._pinionToeRadius_cm),
                ('Driving', r_g, gamma_g, gammaRoot_g, self._drivingToeRadius_cm)):
            ceiling = (pitchRadius - ded * math.cos(gamma)) * (1 - faceWidth / R)
            toeCeilings[label] = ceiling
            if userToe > 0:
                if userToe >= ceiling:
                    raise Exception(
                        f'{label} Gear Toe Radius {to_mm(userToe):.4f} mm must be strictly '
                        f'below that gear\'s Toe Radius Ceiling of {to_mm(ceiling):.4f} mm.')
                toeRadius = userToe
            else:
                toeRadius = pitchRadius - faceWidth / math.sin(gamma)
            toeRadii[label] = toeRadius
            toeLimits[label] = apexDed - toeRadius / math.sin(gammaRoot)

        self._pinionToeRadiusResolved_cm = toeRadii['Pinion']
        self._drivingToeRadiusResolved_cm = toeRadii['Driving']

        # A defaulted Toe Radius can leave no room at all, which is a real
        # configuration rather than a defect: reject the Toe Extension rather
        # than silently substituting a smaller Toe Radius.
        if self._toeExtension_pct > 0:
            for label in ('Pinion', 'Driving'):
                if toeLimits[label] <= rootLengthAtZero:
                    raise Exception(
                        f'Toe Extension {self._toeExtension_pct} percent cannot be applied: '
                        f'the {label} gear\'s inner toe corner already sits at a larger '
                        f'radius than its outer one, so its Toe Limit '
                        f'({to_mm(toeLimits[label]):.4f} mm) falls behind the Toe '
                        f'Extension 0 root length ({to_mm(rootLengthAtZero):.4f} mm). '
                        f'Its Toe Radius must come below {to_mm(toeCeilings[label]):.4f} mm, '
                        f'or leave Toe Extension at 0.')

        # The pair shares one root length, so the SMALLER Toe Limit wins, and
        # Toe Extension 100 stops at 0.99 of the way to it so the toe face never
        # closes to nothing.
        limit = min(toeLimits['Pinion'], toeLimits['Driving'])
        rootLength = rootLengthAtZero + (self._toeExtension_pct / 100.0) * 0.99 * (
            limit - rootLengthAtZero)

        boreDiameters = {}
        if self._boreEnable:
            for (label, pitchRadius, pitchDiameter, gamma, gammaRoot, baseHeight, userBore) in (
                    ('Pinion', r_p, ppd_cm, gamma_p, gammaRoot_p, bh_p, self._pinionBore_cm),
                    ('Driving', r_g, dpd_cm, gamma_g, gammaRoot_g, bh_g, self._drivingBore_cm)):
                rHeel = pitchRadius - baseHeight / math.tan(gamma)
                rToe = (apexDed - rootLength) * math.sin(gammaRoot)
                maximumBore = 2 * 0.95 * min(rHeel, rToe)
                if userBore > 0:
                    if userBore > maximumBore:
                        raise Exception(
                            f'{label} Gear Bore Diameter {to_mm(userBore):.4f} mm exceeds '
                            f'the Maximum Bore Diameter of {to_mm(maximumBore):.4f} mm; a '
                            f'wider bore takes the whole end face off the body.')
                    boreDiameters[label] = userBore
                else:
                    boreDiameters[label] = min(pitchDiameter / 4.0, maximumBore)
        else:
            # Skipped entirely when Enable Bore is unchecked.
            boreDiameters['Pinion'] = 0.0
            boreDiameters['Driving'] = 0.0

        # The toe lattice's closed forms. Both ends of each toe line are seeded
        # at their solved positions, never merely somewhere plausible.
        m2d = _add2(apex2d, _mul2(_unit2(_sub2(c2d, apex2d)), apexDed - rootLength))
        n2d = _add2(m2d, _mul2(uP, (_dist_point_line2(m2d, apex2d, uA)
                                    - toeRadii['Pinion']) / math.cos(gamma_p)))
        aprime2d = _add2(apex2d, _mul2(uA, _dot2(_sub2(n2d, apex2d), uA)))
        o2d = _add2(apex2d, _mul2(_unit2(_sub2(d2d, apex2d)), apexDed - rootLength))
        p2d = _add2(o2d, _mul2(uG, (_dist_point_line2(o2d, apex2d, uB)
                                    - toeRadii['Driving']) / math.cos(gamma_g)))
        bprime2d = _add2(apex2d, _mul2(uB, _dot2(_sub2(p2d, apex2d), uB)))
        perpendicularRootLength = rootLength * R / apexDed

        # ---- A' and the hexagon's shaft-axis edge -------------------------
        # This line is what CREATES A'; the front face N->A' below is what PINS
        # it to the shaft axis. It is drawn here so the hexagon's edges are
        # created in the walk order A' -> G -> H -> C -> M -> N.
        lineApG = construction(aprime2d, g2d)
        constraints.addCoincident(lineApG.endSketchPoint, pointG)
        pointAprime = lineApG.startSketchPoint

        # ---- K and L ------------------------------------------------------
        lineGK = construction(g2d, k2d)
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        # Two point-on-line coincidents rather than addCollinear: by now G and C
        # are already fixed and a collinear here over-constrains.
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)
        lineCK = construction(c2d, k2d)
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        lineIL = construction(i2d, l2d)
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)
        lineDL = construction(d2d, l2d)
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # ---- the tooth centres K' and L' (Tooth Spacing) ------------------
        if self._toothSpacing_cm > 0:
            lineKKp = construction(k2d, kp2d)
            constraints.addCoincident(lineKKp.startSketchPoint, pointK)
            pointKp = lineKKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            lengthDim(lineKKp, k2d, kp2d, self._toothSpacing_cm)
            toothCenterRefLinePinion = construction(c2d, kp2d)
            constraints.addCoincident(toothCenterRefLinePinion.startSketchPoint, pointC)
            constraints.addCoincident(toothCenterRefLinePinion.endSketchPoint, pointKp)

            lineLLp = construction(l2d, lp2d)
            constraints.addCoincident(lineLLp.startSketchPoint, pointL)
            pointLp = lineLLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            lengthDim(lineLLp, l2d, lp2d, self._toothSpacing_cm)
            toothCenterRefLineDriving = construction(d2d, lp2d)
            constraints.addCoincident(toothCenterRefLineDriving.startSketchPoint, pointD)
            constraints.addCoincident(toothCenterRefLineDriving.endSketchPoint, pointLp)
        else:
            # K' is K and L' is L; nothing is built and the existing C->K and
            # D->L reference lines are reused ([BEVEL-F-LINE-ONCE]).
            pointKp, pointLp = pointK, pointL
            toothCenterRefLinePinion = lineCK
            toothCenterRefLineDriving = lineDL

        # ---- the toe lines and the two front faces ------------------------
        lineMN = construction(m2d, n2d)
        constraints.addCoincident(lineMN.startSketchPoint, pinionRootAxis)
        constraints.addParallel(lineMN, lineCH)
        toeOffsetPinion = dimensions.addOffsetDimension(
            lineCH, lineMN, _p3(_mid2(m2d, c2d)))
        toeOffsetPinion.parameter.value = perpendicularRootLength
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint

        lineMC = construction(m2d, c2d)
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)

        # N is NEVER pinned to the shaft axis; A' is a foot, not a corner.
        lineNAp = construction(n2d, aprime2d)
        constraints.addCoincident(lineNAp.startSketchPoint, pointN)
        constraints.addCoincident(lineNAp.endSketchPoint, pointAprime)
        constraints.addCoincident(pointAprime, pinionShaftAxis)
        constraints.addPerpendicular(lineNAp, pinionShaftAxis)
        lengthDim(lineNAp, n2d, aprime2d, toeRadii['Pinion'])

        lineOP = construction(o2d, p2d)
        constraints.addCoincident(lineOP.startSketchPoint, drivingRootAxis)
        constraints.addParallel(lineOP, lineDJ)
        toeOffsetDriving = dimensions.addOffsetDimension(
            lineDJ, lineOP, _p3(_mid2(o2d, d2d)))
        toeOffsetDriving.parameter.value = perpendicularRootLength
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint

        lineOD = construction(o2d, d2d)
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)

        linePBp = construction(p2d, bprime2d)
        constraints.addCoincident(linePBp.startSketchPoint, pointP)
        pointBprime = linePBp.endSketchPoint
        constraints.addCoincident(pointBprime, drivingShaftAxis)
        constraints.addPerpendicular(linePBp, drivingShaftAxis)
        lengthDim(linePBp, p2d, bprime2d, toeRadii['Driving'])

        lineBpI = construction(bprime2d, i2d)
        constraints.addCoincident(lineBpI.startSketchPoint, pointBprime)
        constraints.addCoincident(lineBpI.endSketchPoint, pointI)

        # ---- the two gates ------------------------------------------------
        self._gateFullyConstrained(sketch)

        seedEntries = [
            ('Apex', apexPoint, apex2d),
            ('B', pointB, b2d),
            ('A', pointA, a2d),
            ('Apex 2', apex2Point, apex2_2d),
            ('C', pointC, c2d),
            ('D', pointD, d2d),
            ('E', pointE, e2d),
            ('F', pointF, f2d),
            ('G', pointG, g2d),
            ('H', pointH, h2d),
            ('I', pointI, i2d),
            ('J', pointJ, j2d),
            ('K', pointK, k2d),
        ]
        if self._toothSpacing_cm > 0:
            seedEntries.append(("K'", pointKp, kp2d))
        seedEntries.extend([
            ('M', pointM, m2d),
            ('N', pointN, n2d),
            ("A'", pointAprime, aprime2d),
            ('L', pointL, l2d),
        ])
        if self._toothSpacing_cm > 0:
            seedEntries.append(("L'", pointLp, lp2d))
        seedEntries.extend([
            ('O', pointO, o2d),
            ('P', pointP, p2d),
            ("B'", pointBprime, bprime2d),
        ])
        self._gateSeedsHeld(seedEntries)

        # ---- the per-gear contexts ----------------------------------------
        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'gamma': gamma_p,
            'pitchDiameter_cm': ppd_cm,
            'toothCenterPoint': pointKp,
            'toothCenterRefLine': toothCenterRefLinePinion,
            'hexVertices': [pointAprime, pointG, pointH, pointC, pointM, pointN],
            'toeEdgePoints': [pointM, pointN],
            'heelEdgePoints': [pointC, pointH],
            'boreDiameter_cm': boreDiameters['Pinion'],
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'gamma': gamma_g,
            'pitchDiameter_cm': dpd_cm,
            'toothCenterPoint': pointLp,
            'toothCenterRefLine': toothCenterRefLineDriving,
            'hexVertices': [pointBprime, pointI, pointJ, pointD, pointO, pointP],
            'toeEdgePoints': [pointO, pointP],
            'heelEdgePoints': [pointD, pointJ],
            'boreDiameter_cm': boreDiameters['Driving'],
        }
        return pinionCtx, drivingCtx

    def _pointsTowardNormal(self, sketch: adsk.fusion.Sketch, origin2d,
                            direction2d, targetPlane):
        # The ONE permitted world use in §2: a single-bit comparison of the grow
        # direction against the target plane's normal. A BRepFace's geometry and
        # a ConstructionPlane's geometry are each a core.Plane carrying .normal.
        base = sketch.sketchToModelSpace(_p3(origin2d))
        tip = sketch.sketchToModelSpace(_p3(_add2(origin2d, direction2d)))
        along: adsk.core.Vector3D = _vec(base, tip)
        return along.dotProduct(targetPlane.geometry.normal) >= 0

    def _gateSeedsHeld(self, entries):
        # [BEVEL-F-SEED-HELD]: compare every named point's solved geometry with
        # the closed-form position §2 seeded it at, in creation order, and name
        # the FIRST point that moved.
        for (name, point, seed) in entries:
            solved = _xy(point.geometry)
            moved = _len2(_sub2(solved, seed))
            if moved > self._SEED_TOLERANCE_CM:
                raise Exception(
                    f'Bevel gear: §2 point {name} moved {to_mm(moved):.6f} mm off its '
                    f'seed: solved ({to_mm(solved[0]):.6f}, {to_mm(solved[1]):.6f}) mm, '
                    f'seeded ({to_mm(seed[0]):.6f}, {to_mm(seed[1]):.6f}) mm. The solve '
                    f'took a mirrored branch of the lattice.')

    # -- S07: the tooth plane ---------------------------------------------

    def _buildToothPlane(self, ctx):
        # Pass the sketch line directly; never wrap it in Path.create.
        plane = solids.plane_by_angle(
            self.designComponent, ctx['toothCenterRefLine'], self._gearProfilesPlane, 90)
        plane.name = f'{ctx["label"]} Plane'
        ctx['toothPlane'] = plane

    # -- S08: the virtual spur tooth --------------------------------------

    def _buildVirtualSpurTooth(self, ctx, module):
        sketch = self.designComponent.sketches.add(ctx['toothPlane'])
        sketch.name = f'{ctx["label"]} Tooth'

        # The *10 converts the stashed internal cm to the raw mm Module is in.
        virtualPitchRadius_mm = (to_mm(ctx['pitchDiameter_cm']) / 2.0) / math.cos(ctx['gamma'])
        # A REAL number, never rounded, floored or ceiled.
        virtualTeeth = 2 * virtualPitchRadius_mm / module
        rootSink_mm = 0.05 * 2.25 * module

        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth,
                                 rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        # Drawn ALREADY ROTATED 180 degrees, through draw()'s angle argument.
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))

        # The deterministic selector for the tooth loop's line count in S13.
        ctx['toothEmbedded'] = proxy._lastToothEmbedded

        # The one exemption to the full-constraint gate, and only because the
        # four circles are labelled ([PB-SKETCH-TEXT], [PB-TEXT-HOLDS-DOF]).
        if not sketch.isFullyConstrained:
            futil.log(f'{ctx["label"]} Tooth: sketch reports not fully constrained '
                      f'(labelled sketches hold a DOF; logged, never raised)')

        ctx['toothSketch'] = sketch

    # -- S09: the tooth axis ----------------------------------------------

    def _buildToothAxis(self, ctx):
        helperInput = self.designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(ctx['toothCenterRefLine'],
                                        adsk.core.ValueInput.createByReal(1.0))
        helperPlane = self.designComponent.constructionPlanes.add(helperInput)

        axisInput = self.designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        axis = self.designComponent.constructionAxes.add(axisInput)
        axis.name = f'{ctx["label"]} Tooth Axis'
        ctx['toothAxis'] = axis

    # -- S10: the per-gear component --------------------------------------

    def _createGearComponent(self, ctx):
        # A child of the Bevel Gear component, NOT of the user's Parent Component.
        occurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        occurrence.component.name = f'{ctx["label"]} Gear'
        ctx['gearOccurrence'] = occurrence

    # -- S11..S25: the body ------------------------------------------------

    def _createGearBody(self, ctx):
        component = self.designComponent

        # S11 — the per-gear Profile sketch.
        profileSketch = component.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{ctx["label"]} Profile'
        ctx['profileSketch'] = profileSketch

        # Recreate the six §2 vertices, draw the hexagon SHARING them, and only
        # THEN fix the endpoints ([PB-PROJECT-NOT-FIXED]).
        vertices = [profileSketch.sketchPoints.add(
            profileSketch.modelToSketchSpace(vertex.worldGeometry))
            for vertex in ctx['hexVertices']]
        hexLines = []
        for i in range(len(vertices)):
            hexLines.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(
                vertices[i], vertices[(i + 1) % len(vertices)]))
        for line in hexLines:
            line.startSketchPoint.isFixed = True
            line.endSketchPoint.isFixed = True

        shaftAxisEdge = hexLines[0]
        ctx['shaftAxisEdge'] = shaftAxisEdge
        self._gateFullyConstrained(profileSketch)

        # S12 — revolve the Gear Body.
        profile = profileSketch.profiles.item(0)
        revolveInput = component.features.revolveFeatures.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = component.features.revolveFeatures.add(revolveInput)
        gearBody = revolve.bodies.item(0)
        ctx['gearBody'] = gearBody

        # S13 — loft the uncut tooth body from the §2 Apex sketch point.
        wantLines = 0 if ctx['toothEmbedded'] else 2
        toothProfile = find_profile_by_curve_counts(
            ctx['toothSketch'], nurbs=2, arcs=2, lines=wantLines)
        loftInput = component.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = component.features.loftFeatures.add(loftInput)
        toothBody = loft.bodies.item(0)

        # S14 — the tooth-body hook, with the caller's obligations.
        toeEdge = ctx['toeEdgePoints']
        heelEdge = ctx['heelEdgePoints']
        toeMid = _world_mid(toeEdge[0], toeEdge[1])
        heelMid = _world_mid(heelEdge[0], heelEdge[1])
        toeConeWorld = toeEdge[0].worldGeometry
        heelConeWorld = heelEdge[0].worldGeometry
        apexWorld = self._apexSketchPoint.worldGeometry

        toothPiece = self._transformToothBody(
            component, toothBody, gearBody, shaftAxisEdge, apexWorld,
            self._apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
            ctx['toothPlane'], ctx['label'], ctx['teeth'], ctx['gamma'])

        # S22 — circular-pattern the tooth around the shaft-axis edge.
        seedBodies = adsk.core.ObjectCollection.create()
        seedBodies.add(toothPiece)
        patternInput = component.features.circularPatternFeatures.createInput(
            seedBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = component.features.circularPatternFeatures.add(patternInput)

        # The pattern returns the original plus the copies; copy them into an
        # ObjectCollection ([PB-PATTERN-BODIES]).
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))

        # S23 — Combine-Join the teeth onto the Gear Body.
        combineInput = component.features.combineFeatures.createInput(gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(combineInput)

        # S24 — the bore.
        if self._boreEnable:
            self._cutBore(ctx)

        # S25 — the meshing rotation, here, before the body is moved out.
        if ctx['label'] == 'Driving':
            angle = math.pi / ctx['teeth']
        else:
            angle = self._PINION_MESH_PHASE_TEETH * 2 * math.pi / ctx['teeth']
        # A zero angle is a no-op: the framework helper absorbs it, so no guard
        # belongs at this call site.
        solids.rotate_body_about_edge(component, gearBody, shaftAxisEdge, angle)

    def _cutBore(self, ctx):
        component = self.designComponent
        boreDiameter = ctx['boreDiameter_cm']
        boreRadius = boreDiameter / 2.0

        planeInput = component.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(ctx['shaftAxisEdge'],
                                       adsk.core.ValueInput.createByReal(0.0))
        borePlane = component.constructionPlanes.add(planeInput)

        sketch = component.sketches.add(borePlane)
        sketch.name = f'{ctx["label"]} Bore'
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreRadius)
        # isFixed on the centre, never addCoincident to the sketch origin
        # ([PB-CIRCLE-CENTER]).
        circle.centerSketchPoint.isFixed = True
        dimension = sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreRadius, 0, 0))
        dimension.parameter.value = boreDiameter

        extrudeInput = component.features.extrudeFeatures.createInput(
            sketch.profiles.item(0),
            adsk.fusion.FeatureOperations.CutFeatureOperation)
        # Cone Distance here is the DIAGONAL, and isFullLength=False makes it the
        # half-length per side ([PB-THROUGH-CUT]).
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [ctx['gearBody']]
        component.features.extrudeFeatures.add(extrudeInput)

        self._gateFullyConstrained(sketch)

    # -- S26: relocate the finished body ----------------------------------

    def _moveBodyIntoGearComponent(self, ctx):
        gearBody: adsk.fusion.BRepBody = ctx['gearBody']
        gearBody.moveToComponent(ctx['gearOccurrence'])

    # -- S14..S21: the tooth-body hook ------------------------------------

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                            toothBody: adsk.fusion.BRepBody,
                            gearBody: adsk.fusion.BRepBody,
                            shaftAxisEdge: adsk.fusion.SketchLine,
                            apexWorld: adsk.core.Point3D,
                            apexSketchPoint: adsk.fusion.SketchPoint,
                            toeMid: adsk.core.Point3D,
                            heelMid: adsk.core.Point3D,
                            toeConeWorld: adsk.core.Point3D,
                            heelConeWorld: adsk.core.Point3D,
                            parentToothPlane: adsk.fusion.ConstructionPlane,
                            gearLabel, teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            # Straight bevels are byte-for-byte the prior behaviour.
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)

        # ---- S15: the frame and its gate ---------------------------------
        startWorld = shaftAxisEdge.startSketchPoint.worldGeometry
        endWorld = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir: adsk.core.Vector3D = _unit_vec(_vec(startWorld, endWorld))

        # The heel MUST be the outer end so coneVec points outward and the span
        # is positive; a negative span silently inverts the whole spiral frame.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld
            futil.log(f'{gearLabel}: toe and heel arrived swapped; the spiral frame '
                      f'guard corrected them', force_console=True)

        coneVec: adsk.core.Vector3D = _unit_vec(_vec(apexWorld, heelConeWorld))
        v: adsk.core.Vector3D = _unit_vec(axisDir.crossProduct(coneVec))
        # tpNormal completes the frame and nothing consumes it.
        tpNormal: adsk.core.Vector3D = _unit_vec(coneVec.crossProduct(v))

        def distAlong(point):
            offset: adsk.core.Vector3D = _vec(apexWorld, point)
            return offset.dotProduct(coneVec)

        rToe = distAlong(toeMid)
        rHeel = distAlong(heelMid)
        rMean = (rToe + rHeel) / 2.0
        span = rHeel - rToe
        if span <= 0:
            raise RuntimeError(
                f'{gearLabel}: the spiral frame span is {to_mm(span):.6f} mm; the heel is '
                f'not the outer end, which inverts the whole frame.')

        # ---- the cutter-arc geometry --------------------------------------
        psi = self._spiralAngle_rad
        cutterRadius = self._cutterRadius_cm if self._cutterRadius_cm != 0 else rMean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        # The hand sign goes on the cos / Cy term, NEVER on the sin / Cx term.
        cx = rMean - cutterRadius * math.sin(psi)
        cy = handSign * cutterRadius * math.cos(psi)
        rLo = rToe - 0.06 * span
        rHi = rHeel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(rLo, cx, cy, cutterRadius, rMean, 0)
        heel2d = solids.circle_intersect_nearest(rHi, cx, cy, cutterRadius, rMean, 0)

        # ---- the cone element and the trace sketches ----------------------
        # Every point below is passed DIRECTLY into the sketch calls, consumed as
        # sketch-space input with no modelToSketchSpace conversion. This is safe
        # only because no downstream feature consumes either sketch or the Trace
        # Plane.
        coneSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, solids.combine_point(apexWorld, rHeel, coneVec))
        coneElementLine.isConstruction = True

        tracePlane = solids.plane_by_angle(
            designComponent, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        cutterCentre = solids.combine_point(apexWorld, cx, coneVec, cy, v)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            cutterCentre, cutterRadius)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        cutterDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, solids.combine_point(apexWorld, cx + cutterRadius, coneVec, cy, v))
        cutterDim.parameter.value = 2 * cutterRadius

        meanPoint = solids.combine_point(apexWorld, rMean, coneVec, 0.0, v)
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            solids.combine_point(apexWorld, toe2d[0], coneVec, toe2d[1], v),
            meanPoint,
            solids.combine_point(apexWorld, heel2d[0], coneVec, heel2d[1], v))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        # Text point off-centre and on the curve ([PB-RADIAL-DIM]).
        radialDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, meanPoint)
        radialDim.parameter.value = cutterRadius
        # These three — the Cone Element sketch, the Trace Plane and the trace
        # sketch — are exempt from the full-constraint gate; do NOT gate them.

        # ---- S16: slice the straight tooth into slabs ---------------------
        planeGeometry = parentToothPlane.geometry
        towardApex: adsk.core.Vector3D = _vec(planeGeometry.origin, apexWorld)
        sign = 1.0 if towardApex.dotProduct(planeGeometry.normal) > 0 else -1.0

        def sliceWith(signValue):
            offsets = [signValue * (k + 1) * span / 6.0 for k in range(8)]
            return solids.slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)

        pieces = sliceWith(sign)
        if len(pieces) == 1:
            futil.log(f'{gearLabel}: the slice left one piece; retrying with the '
                      f'opposite offset sign', force_console=True)
            sign = -sign
            pieces = sliceWith(sign)
        if len(pieces) == 1:
            raise RuntimeError(
                f'{gearLabel}: the slice produced {len(pieces)} piece(s) and never split '
                f'the tooth (span {to_mm(span):.4f} mm, both offset signs tried, last '
                f'sign {sign:+.0f}). The parent tooth plane sits outside the tooth span.')

        # ---- S17: order the segments and drop the apex scrap --------------
        def centroidDistAlong(body):
            return distAlong(body.physicalProperties.centerOfMass)

        segments = sorted(pieces, key=centroidDistAlong)
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise RuntimeError(
                f'{gearLabel}: dropping the apex scrap left no segments; the slice in the '
                f'previous step produced nothing the twist and the crown can work on.')

        # ---- S18: twist each segment about the shaft axis -----------------
        phiCrown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        # The roll ratio uses the PITCH cone angle from §2, never
        # acos(coneVec . axisDir), which is the root cone angle.
        total = abs(phiCrown) / math.sin(gamma)

        axisVector = adsk.core.Vector3D.create(axisDir.x, axisDir.y, axisDir.z)
        for segment in segments:
            heelFace = self._endFace(segment, distAlong, farthest=True)
            angle = -handSign * total * (rMean - distAlong(heelFace.centroid)) / span
            if angle == 0:
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(angle, axisVector, apexWorld)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(segment)
            moveInput = designComponent.features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # ---- S19: the lengthwise crown ------------------------------------
        postTwist = [(segment, distAlong(
            self._endFace(segment, distAlong, farthest=True).centroid))
            for segment in segments]
        outermost = max(range(len(postTwist)), key=lambda i: postTwist[i][1])

        # scaleFeatures is the ONE exception to never-activate: it needs the
        # Design occurrence as the active edit target.
        self.designOccurrence.activate()
        try:
            for (index, (segment, heelDistance)) in enumerate(postTwist):
                if index == outermost:
                    # The heel segment stays full so the heel cone trims it flush.
                    continue
                u = (rHeel - heelDistance) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise RuntimeError(
                        f'{gearLabel}: crown factor {factor:.6f} at u = {u:.6f} on segment '
                        f'{index} is not positive; a body is never scaled by a '
                        f'non-positive factor.')
                heelFace = self._endFace(segment, distAlong, farthest=True)
                basePoint = self._rootEdgeMidpoint(heelFace, apexWorld, axisDir)
                baseSketch = designComponent.sketches.add(heelFace)
                baseSketch.name = f'{gearLabel} Crown Base {index}'
                baseSketchPoint = baseSketch.sketchPoints.add(
                    baseSketch.modelToSketchSpace(basePoint))

                scaleBodies = adsk.core.ObjectCollection.create()
                scaleBodies.add(segment)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    scaleBodies, baseSketchPoint,
                    adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            # A Component has no activate method; the root is re-activated
            # through Design.
            self.design.activateRootComponent()

        # ---- S20: loft the curved tooth -----------------------------------
        # Re-sort HERE, after the twist and the crown.
        ordered = sorted(segments, key=lambda segment: distAlong(
            self._endFace(segment, distAlong, farthest=True).centroid))

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # First the toe-most segment's apex-side, toe-facing face, so the loft is
        # pushed past the toe cone and the toe trim bites.
        loftInput.loftSections.add(
            self._endFace(ordered[0], distAlong, farthest=False))
        for segment in ordered:
            loftInput.loftSections.add(
                self._endFace(segment, distAlong, farthest=True))
        spiralLoft = designComponent.features.loftFeatures.add(loftInput)
        curvedTooth = spiralLoft.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for segment in segments:
            designComponent.features.removeFeatures.add(segment)

        # ---- S21: the flush trim ------------------------------------------
        return solids.cut_conical_ends(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apexWorld, gearLabel)

    def _endFace(self, body: adsk.fusion.BRepBody, distAlong, farthest=True):
        # A slab's heel face is the face whose centroid has the GREATEST cone
        # distance, searched across ALL faces with NO surface-type filter; its
        # toe face is the least-centroid one.
        best = None
        bestDistance = None
        for face in body.faces:
            distance = distAlong(face.centroid)
            if bestDistance is None or (distance > bestDistance if farthest
                                        else distance < bestDistance):
                best, bestDistance = face, distance
        if best is None:
            raise RuntimeError(
                'Bevel gear: a sliced segment carries no faces, so its end face '
                'cannot be found.')
        return best

    def _rootEdgeMidpoint(self, face: adsk.fusion.BRepFace,
                          apexWorld: adsk.core.Point3D,
                          axisDir: adsk.core.Vector3D):
        # The two vertices with the smallest perpendicular distance to the shaft
        # axis are the root corners; the base point is their midpoint. Anchoring
        # on the heel face's centroid instead lifts the tooth off the gear base.
        corners = []
        for i in range(face.vertices.count):
            point = face.vertices.item(i).geometry
            offset: adsk.core.Vector3D = _vec(apexWorld, point)
            along = offset.dotProduct(axisDir)
            radial = adsk.core.Vector3D.create(
                offset.x - along * axisDir.x,
                offset.y - along * axisDir.y,
                offset.z - along * axisDir.z)
            corners.append((radial.length, point))
        if len(corners) < 2:
            raise RuntimeError(
                f'Bevel gear: the crown base face carries {len(corners)} vertices, so its '
                f'root edge cannot be located.')
        corners.sort(key=lambda entry: entry[0])
        first = corners[0][1]
        second = corners[1][1]
        return adsk.core.Point3D.create(
            (first.x + second.x) / 2.0,
            (first.y + second.y) / 2.0,
            (first.z + second.z) / 2.0)
