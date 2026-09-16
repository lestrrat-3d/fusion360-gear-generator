# Bevel gear generator.
#
# Written from the compiled step list `spec/bevelgear/steps.md` (S1-S32) and the
# checked geometry in `proof/bevelgear/`. Bevel is a STANDALONE generator: it does
# not subclass base.Generator, carries no GenerationContext and registers no Fusion
# user parameters — every value is precomputed in Python in internal cm and written
# into geometry numerically ([PB-PRECOMPUTED-MODE]).

import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator


# ---------------------------------------------------------------- dialog input ids
# S2: the 20 ids, in the dialog's display order. Bevel registers no user
# parameters, so there are no PARAM_* strings.

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

# The Hand of Spiral dropdown's two list-item strings.
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Gear-geometry constants the step list states as literals.
_DEDENDUM_MODULES = 1.25                 # the dedendum, in modules
_MIN_TEETH_FACTOR = 5.27                 # 2*(1.05*1.25/0.95 + 1.25), rounded UP
_MIN_TEETH_ABSOLUTE = 3                  # the blanket absolute floor
_FACE_WIDTH_MARGIN = 0.95                # the Maximum Face Width / Maximum Base Height margin
_BASE_HEIGHT_MARGIN = 1.05               # the Minimum Base Height margin
_SHAFT_ANGLE_FLOOR_DEG = 30.0            # the documented geometric floor
_SHAFT_ANGLE_CEILING_DEG = 150.0         # the practical cap on the Maximum Shaft Angle
_SPIRAL_ANGLE_CEILING_DEG = 60.0         # Mean Spiral Angle is in [0, 60)
_ROOT_SINK_FRACTION = 0.05 * 2.25        # the §3 root sink, in modules
_TOE_EXTENSION_REACH = 0.99              # Toe Extension 100 stops 0.99 of the way to the Toe Limit
_SPIRAL_SLICE_STEPS = 8                  # the fixed slice scheme's plane count
_SPIRAL_SLICE_DIVISOR = 6.0              # the span/6 step the cut planes move by
_SPIRAL_ARC_OVERSHOOT = 0.06             # the hair past the face the trace's ends are taken at


# ---------------------------------------------------------------- 2-D sketch math
# Every §2 position is computed in the Gear Profiles sketch's own local frame
# ([BEVEL-F-APEX-LOCAL]); these operate on plain (x, y) tuples.

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _scale(a, s):
    return (a[0] * s, a[1] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _length(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    n = _length(a)
    if n == 0:
        raise Exception('cannot normalize a zero-length direction in the §2 lattice')
    return (a[0] / n, a[1] / n)


def _perpendicular(a):
    # The in-plane perpendicular, rotated a quarter turn counter-clockwise.
    return (-a[1], a[0])


def _turn(a, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _midpoint2(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)


def _foot_on_line(p, origin, direction):
    # The foot of p's perpendicular onto the line through origin along direction.
    return _add(origin, _scale(direction, _dot(_sub(p, origin), direction)))


def _perpendicular_distance(p, origin, direction):
    return abs(_cross(direction, _sub(p, origin)))


def _p3(p):
    return adsk.core.Point3D.create(p[0], p[1], 0)


def _xy(point):
    return (point.x, point.y)


# ---------------------------------------------------------------- world helpers

def _world_midpoint(a: adsk.core.Point3D, b: adsk.core.Point3D) -> adsk.core.Point3D:
    return adsk.core.Point3D.create((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)


def _axis_distance(point: adsk.core.Point3D, origin: adsk.core.Point3D,
                   axisDir: adsk.core.Vector3D) -> float:
    # Perpendicular distance from point to the line through origin along axisDir.
    w = origin.vectorTo(point)
    along = w.dotProduct(axisDir)
    return math.sqrt(max(0.0, w.dotProduct(w) - along * along))


def _input_value(unitsManager: adsk.core.UnitsManager, inputs: adsk.core.CommandInputs,
                 name: str, units: str) -> float:
    # S4: read one dialog input by evaluating its EXPRESSION, which ALWAYS returns
    # Fusion internal units — cm for length and radians for angle — regardless of
    # the unit string ([PB-EVAL-EXPRESSION]). Never read via realValue.
    return unitsManager.evaluateExpression(inputs.itemById(name).expression, units)


def _cone_distance(point: adsk.core.Point3D, apex: adsk.core.Point3D,
                   coneVec: adsk.core.Vector3D) -> float:
    # A point's cone distance: how far it sits from the apex measured ALONG the
    # cone element, which is what every spiral station below is measured with.
    return apex.vectorTo(point).dotProduct(coneVec)


def _solve_rim(apex, apex2, axisEnd, axisDir, ded, dedDir, baseHeight):
    """Closed-form E, G, H and K for one gear's dedendum chain (S10).

    E is where the dedendum corner C (D) drops perpendicularly onto the shaft
    axis; G (I) is the point on that axis at the resolved base height's offset
    from the axis->Apex2 drop; H (J) carries the same signed offset on the
    dedendum line; K (L) is the shaft axis crossed with the dedendum line. These
    are seeds only — the constraint net is what places the drawn points."""
    ext = _foot_on_line(ded, apex, axisDir)

    dropDir = _unit(_sub(apex2, axisEnd))
    along = _dot(_sub(ded, axisEnd), axisDir)
    side = 1.0 if along >= 0 else -1.0
    o0 = _cross(dropDir, _sub(apex, axisEnd))
    o1 = _cross(dropDir, axisDir)
    want = side * abs(baseHeight)
    if o1 * side < 0:
        want = -want
    ext2 = _add(apex, _scale(axisDir, (want - o0) / o1))
    offset = _cross(dropDir, _sub(ext2, axisEnd))

    h0 = _cross(dropDir, _sub(apex2, axisEnd))
    h1 = _cross(dropDir, dedDir)
    heel = _add(apex2, _scale(dedDir, (offset - h0) / h1))

    k1 = _cross(dedDir, axisDir)
    k0 = _cross(dedDir, _sub(apex, apex2))
    centre = _add(apex, _scale(axisDir, -k0 / k1))
    return ext, ext2, heel, centre


# ---------------------------------------------------------------- the dialog

class BevelGearCommandInputsConfigurator:
    """S2 / S3: the 20 dialog inputs and the spiral-only visibility rule. Bound by
    name from commands/bevelgear/entry.py."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane comes first so it wins Fusion's auto-focus
        # ([PB-AUTOFOCUS-FIRST]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point, so the user flows from plane to point.
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component, pre-selected with the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4-9. The mm/deg defaults are Fusion INTERNAL units
        # ([PB-DIALOG-DEFAULT-UNITS]).
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

        # 10-14.
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)
        inputs.addValueInput(INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # 15-17. The spiral group.
        inputs.addValueInput(INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
                             adsk.core.ValueInput.createByString('35 deg'))
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)
        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # 18-20. The toe group.
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
                             adsk.core.ValueInput.createByReal(0))
        inputs.addValueInput(INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # The last step, so the initial state is correct for the default ψ = 35°.
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        # S3: recompute the spiral-only visibility on EVERY input change.
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        # S3: Hand of Spiral and Cutter Radius are shown only when ψ > 0. There is
        # no declarative show-if in the Fusion API, so this is `isVisible`, which
        # hides the dialog row only — the inputs are still read normally and the
        # ψ = 0 build ignores both anyway.
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return

        visible = True
        try:
            # The input's EXPRESSION evaluated to internal radians, never `.value`.
            unitsManager: adsk.core.UnitsManager = get_design().unitsManager
            value = unitsManager.evaluateExpression(spiral.expression, 'rad')
            visible = value > 0
        except Exception:
            # A half-typed expression can raise mid-edit; leave both inputs shown.
            visible = True

        hand.isVisible = visible
        cutter.isVisible = visible


# ---------------------------------------------------------------- the generator

class BevelGearGenerator:
    """S1: a standalone generator. `commands/_gear_command.py` calls generate(inputs)
    inside its execute handler and deleteComponent() on an exception."""

    # S22: the lengthwise crown's tunable class constant; 0 disables the crown.
    _CROWN_PER_RAD = 0.5
    # S18: the pinion's extra mesh rotation, in teeth. 0 by default, because the
    # spiral tooth's mid-face section is unrotated and already meshes.
    _PINION_MESH_PHASE_TEETH = 0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # ------------------------------------------------------------ orchestration

    def generate(self, inputs: adsk.core.CommandInputs):
        # S4: read every input first, before anything creates an occurrence.
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngleDeg) = self._readInputs(inputs)

        # S5: validate the ranges and resolve every closed-form bound.
        self._validate(module, drivingTeeth, pinionTeeth, shaftAngleDeg)

        # S7: the component tree.
        self._buildTree(parentComponent)

        # S8 / S9 / S10: the anchor, the axial plane and the §2 lattice.
        self._buildAnchorSketch(targetPlane, centerPoint)
        self._buildGearProfilesPlane(targetPlane)
        pinionCtx, drivingCtx = self._buildGearProfiles(
            targetPlane, module, drivingTeeth, pinionTeeth, math.radians(shaftAngleDeg))

        # S11-S31, pinion first and then the driving gear.
        for ctx in (pinionCtx, drivingCtx):
            self._buildGear(ctx, module)

        # S32: cleanup.
        solids.hide_construction_geometry(self.bevelComponent)

    def deleteComponent(self):
        # S7: the error rollback the entry point calls on an exception.
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = None

    # ------------------------------------------------------------ S4: the inputs

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        """Every numeric input is read by evaluating its EXPRESSION, which always
        returns Fusion internal units — cm for length and radians for angle —
        regardless of the unit string ([PB-EVAL-EXPRESSION]). Module is read with
        unit '' and so comes back as a raw number that means MILLIMETRES."""
        unitsManager: adsk.core.UnitsManager = get_design().unitsManager

        def value(name, units):
            return _input_value(unitsManager, inputs, name, units)

        planeSelection = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeSelection) == 0:
            raise Exception('Select the Target Plane the driving gear sits flush against')
        targetPlane = planeSelection[0]

        centerSelection = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centerSelection) == 0:
            raise Exception('Select the Center Point the driving bevel gear is centered on')
        centerPoint = centerSelection[0]

        parentSelection = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentSelection) == 0:
            parentComponent = get_design().rootComponent
        else:
            parentComponent = self._componentOf(parentSelection[0])

        # Module is raw mm; every length derived from it is to_cm-converted before
        # it touches geometry.
        module = value(INPUT_ID_MODULE, '')
        shaftAngle_rad = value(INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(value(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(value(INPUT_ID_PINION_TEETH, '')))

        # The mm inputs come back already in internal cm — never to_cm them again.
        self._drivingBaseHeight_cm = value(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = value(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = value(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = value(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = value(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = value(INPUT_ID_TOOTH_SPACING, 'mm')
        self._spiralAngle_rad = value(INPUT_ID_SPIRAL_ANGLE, 'deg')

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        self._hand = handItem.name if handItem is not None else _HAND_RIGHT

        self._cutterRadius_cm = value(INPUT_ID_CUTTER_RADIUS, 'mm')
        # A plain unitless percentage; no conversion.
        self._toeExtension = value(INPUT_ID_TOE_EXTENSION, '')
        self._drivingToeRadius_cm = value(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        self._pinionToeRadius_cm = value(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, math.degrees(shaftAngle_rad))

    def _componentOf(self, entity):
        if entity.objectType == adsk.fusion.Occurrence.classType():
            return entity.component
        return entity

    # ------------------------------------------------------------ S5: validation

    def _validate(self, module, drivingTeeth, pinionTeeth, shaftAngleDeg):
        if module <= 0:
            raise Exception('Module must be greater than 0')

        for label, teeth in (('Driving Gear', drivingTeeth), ('Pinion Gear', pinionTeeth)):
            if teeth < _MIN_TEETH_ABSOLUTE:
                raise Exception(
                    f'{label} Teeth Number must be at least {_MIN_TEETH_ABSOLUTE} '
                    f'(got {teeth})')

        for label, amount in (
                ('Driving Gear Base Height', self._drivingBaseHeight_cm),
                ('Pinion Gear Base Height', self._pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', self._drivingBore_cm),
                ('Pinion Gear Bore Diameter', self._pinionBore_cm),
                ('Face Width', self._faceWidth_cm),
                ('Tooth Spacing', self._toothSpacing_cm),
                ('Cutter Radius', self._cutterRadius_cm),
                ('Driving Gear Toe Radius', self._drivingToeRadius_cm),
                ('Pinion Gear Toe Radius', self._pinionToeRadius_cm)):
            if amount < 0:
                raise Exception(f'{label} must not be negative')

        spiralAngleDeg = math.degrees(self._spiralAngle_rad)
        if spiralAngleDeg < 0 or spiralAngleDeg >= _SPIRAL_ANGLE_CEILING_DEG:
            raise Exception(
                f'Mean Spiral Angle must be at least 0 deg and below '
                f'{_SPIRAL_ANGLE_CEILING_DEG:.0f} deg (got {spiralAngleDeg:.3f} deg)')

        if self._toeExtension < 0 or self._toeExtension > 100:
            raise Exception(
                f'Toe Extension (%) must be between 0 and 100 (got {self._toeExtension:.3f})')

        # The two pitch diameters and the Cone Distance, which is the DIAGONAL of
        # the two pitch diameters and never depends on the Shaft Angle.
        drivingPitchDia_cm = to_cm(module * drivingTeeth)
        pinionPitchDia_cm = to_cm(module * pinionTeeth)
        self._coneDistance_cm = math.hypot(drivingPitchDia_cm, pinionPitchDia_cm)

        # The Maximum Shaft Angle: a pitch cone angle reaching 90 deg turns that
        # gear's pitch cone inside out. The cone-angle half is EXCLUSIVE and the
        # 150 deg half INCLUSIVE.
        smaller = min(drivingPitchDia_cm, pinionPitchDia_cm)
        larger = max(drivingPitchDia_cm, pinionPitchDia_cm)
        coneLimitDeg = math.degrees(math.acos(-smaller / larger))
        maximumShaftAngleDeg = min(_SHAFT_ANGLE_CEILING_DEG, coneLimitDeg)
        if shaftAngleDeg < _SHAFT_ANGLE_FLOOR_DEG:
            raise Exception(
                f'Shaft Angle must be at least {_SHAFT_ANGLE_FLOOR_DEG:.0f} deg '
                f'(got {shaftAngleDeg:.3f} deg)')
        if coneLimitDeg <= _SHAFT_ANGLE_CEILING_DEG:
            if shaftAngleDeg >= coneLimitDeg:
                raise Exception(
                    f'Shaft Angle must be below the Maximum Shaft Angle '
                    f'{maximumShaftAngleDeg:.3f} deg for {drivingTeeth}/{pinionTeeth} teeth '
                    f'(got {shaftAngleDeg:.3f} deg); at or above it a pitch cone angle '
                    f'reaches 90 deg and the back-cone virtual radius is unbounded')
        elif shaftAngleDeg > _SHAFT_ANGLE_CEILING_DEG:
            raise Exception(
                f'Shaft Angle must not exceed the Maximum Shaft Angle '
                f'{maximumShaftAngleDeg:.3f} deg (got {shaftAngleDeg:.3f} deg)')

        # The two pitch cone angles and the Pitch Cone Distance R, from the closed
        # form. R is a different length from the Cone Distance above; the two
        # coincide as `Cone Distance = 2 * R` exactly at Shaft Angle 90 deg.
        sigma = math.radians(shaftAngleDeg)
        self._gamma_p = math.atan2(
            math.sin(sigma) * pinionPitchDia_cm,
            drivingPitchDia_cm + pinionPitchDia_cm * math.cos(sigma))
        self._gamma_g = sigma - self._gamma_p
        self._pitchConeDistance_cm = (pinionPitchDia_cm / 2) / math.sin(self._gamma_p)

        # 1. Minimum Teeth, per gear against its OWN pitch cone angle. Running this
        # before the base heights is exactly the statement that the base-height
        # window is non-empty.
        for label, teeth, gamma in (
                ('Driving Gear', drivingTeeth, self._gamma_g),
                ('Pinion Gear', pinionTeeth, self._gamma_p)):
            floor = _MIN_TEETH_FACTOR * math.cos(gamma)
            if teeth < floor:
                raise Exception(
                    f'{label} Teeth Number must be at least {floor:.2f} at a pitch cone '
                    f'angle of {math.degrees(gamma):.3f} deg (got {teeth}); below that the '
                    f'Minimum and Maximum Base Heights have crossed')

        # 2. The two base heights, both bounds per gear, in both directions.
        self._drivingBaseHeightResolved_cm = self._resolveBaseHeight(
            'Driving Gear', self._drivingBaseHeight_cm,
            to_cm(module * drivingTeeth / 8),
            module, drivingPitchDia_cm / 2, self._gamma_g)
        # The pinion's fallback is the RESOLVED driving base height scaled by the
        # tooth ratio, and then the pinion's OWN bounds are applied to that.
        self._pinionBaseHeightResolved_cm = self._resolveBaseHeight(
            'Pinion Gear', self._pinionBaseHeight_cm,
            self._drivingBaseHeightResolved_cm * pinionTeeth / drivingTeeth,
            module, pinionPitchDia_cm / 2, self._gamma_p)

        self._module = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._drivingPitchDia_cm = drivingPitchDia_cm
        self._pinionPitchDia_cm = pinionPitchDia_cm

        # Bore diameters: only consulted when Enable Bore is checked; 0 means auto.
        self._drivingBoreResolved_cm = (
            self._drivingBore_cm if self._drivingBore_cm > 0 else drivingPitchDia_cm / 4)
        self._pinionBoreResolved_cm = (
            self._pinionBore_cm if self._pinionBore_cm > 0 else pinionPitchDia_cm / 4)

    def _resolveBaseHeight(self, label, wanted_cm, fallback_cm, module, pitchRadius_cm, gamma):
        """Both bounds are closed-form: r, γ and Module are all known before §2
        draws anything. A fallback is raised to the minimum and capped to the
        maximum; a USER value outside either end is rejected naming the bound."""
        maximum = _FACE_WIDTH_MARGIN * (
            pitchRadius_cm - to_cm(_DEDENDUM_MODULES * module) * math.cos(gamma)) * math.tan(gamma)
        minimum = _BASE_HEIGHT_MARGIN * to_cm(_DEDENDUM_MODULES * module) * math.sin(gamma)

        if wanted_cm > 0:
            if wanted_cm < minimum:
                raise Exception(
                    f'{label} Base Height must be at least the Minimum Base Height '
                    f'{minimum * 10:.4f} mm (got {wanted_cm * 10:.4f} mm); below it the heel '
                    f'point lands behind the dedendum corner and the heel edge runs inward')
            if wanted_cm > maximum:
                raise Exception(
                    f'{label} Base Height must not exceed the Maximum Base Height '
                    f'{maximum * 10:.4f} mm (got {wanted_cm * 10:.4f} mm); above it the heel '
                    f'point reaches the shaft axis and the revolve fails with ASM_WIRE_X_AXIS')
            return wanted_cm

        return max(minimum, min(fallback_cm, maximum))

    # ------------------------------------------------------------ S7: the tree

    def _buildTree(self, parentComponent: adsk.fusion.Component):
        # Never call activate() on any occurrence ([PB-NEVER-ACTIVATE],
        # [BEVEL-F-NEVER-ACTIVATE]); the sole exception is the spiral crown's scale
        # feature in S22, which restores the root in a finally.
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

    # ------------------------------------------------------------ sketch helpers

    def _line(self, sketch: adsk.fusion.Sketch, startSeed, endSeed,
              pinStart=None, pinEnd=None) -> adsk.fusion.SketchLine:
        """A §2 construction line in the COINCIDENT style
        ([BEVEL-F-COINCIDENT-STYLE]): created from raw Point3D coordinates, with
        each end that connects to existing geometry pinned by exactly one
        addCoincident and never shared."""
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(_p3(startSeed), _p3(endSeed))
        line.isConstruction = True
        if pinStart is not None:
            sketch.geometricConstraints.addCoincident(line.startSketchPoint, pinStart)
        if pinEnd is not None:
            sketch.geometricConstraints.addCoincident(line.endSketchPoint, pinEnd)
        return line

    def _alignedDistance(self, sketch: adsk.fusion.Sketch, pointOne, pointTwo,
                         textPoint, value=None):
        """Every length dimension in §2 is an aligned distance: this figure has no
        axis-aligned line in it, so a horizontal or vertical orientation would
        dimension the line's projection onto a sketch axis instead of its length.
        The value assigned is the ABSOLUTE magnitude ([PB-DIM-VALUE-SEMANTICS])."""
        dimension = sketch.sketchDimensions.addDistanceDimension(
            pointOne, pointTwo,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(textPoint))
        if value is not None:
            dimension.parameter.value = abs(value)
        return dimension

    def _gateSketch(self, sketch: adsk.fusion.Sketch):
        # [PB-FULL-CONSTRAINT], [BEVEL-F-FULL-CONSTRAINT]: a free DOF is a
        # generation defect, not a warning.
        if not sketch.isFullyConstrained:
            raise Exception(
                f'Sketch "{sketch.name}" is not fully constrained; a free degree of '
                f'freedom lets the geometry shift between rebuilds')

    # ------------------------------------------------------------ S8: the anchor

    def _buildAnchorSketch(self, targetPlane, centerPoint):
        # Start directly on the user-selected target plane, whether it is a
        # ConstructionPlane or a PlanarFace ([PB-USE-SELECTED-PLANE]).
        sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projected = sketch.project(centerPoint).item(0)
        centre = _xy(projected.geometry)

        # Seed the two endpoints at exactly +/- 0.5 cm along the sketch-local X, so
        # the seeded length is 10 mm.
        start = (centre[0] - 0.5, centre[1])
        end = (centre[0] + 0.5, centre[1])
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(_p3(start), _p3(end))
        anchorLine.isConstruction = True

        constraints = sketch.geometricConstraints
        # Both the coincident and the midpoint: midpoint alone is not enough.
        constraints.addCoincident(projected, anchorLine)
        constraints.addMidPoint(projected, anchorLine)
        # An aligned distance dimension WITHOUT assigning .parameter.value: it
        # simply locks the length at the seeded 10 mm, and nothing downstream reads
        # the value.
        self._alignedDistance(
            sketch, anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            (centre[0], centre[1] + 0.2))
        # Sketch-local, per [PB-REFLINE-DIRECTION]: a world-axis lock would
        # mis-orient the figure on a tilted target plane.
        constraints.addHorizontal(anchorLine)

        self._gateSketch(sketch)

        self._anchorSketch = sketch
        self._anchorCenterPoint = projected
        self._anchorLine = anchorLine

    # ------------------------------------------------------------ S9: the plane

    def _buildGearProfilesPlane(self, targetPlane):
        planes = self.designComponent.constructionPlanes
        planeInput = planes.createInput()
        # Built off the ORIGINAL targetPlane as the reference
        # ([PB-USE-SELECTED-PLANE]); the SketchLine goes in directly and never
        # through Path.create ([PB-CONSTRUCTION-PLANES]).
        planeInput.setByAngle(self._anchorLine,
                              adsk.core.ValueInput.createByString('90 deg'),
                              targetPlane)
        plane = planes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # ------------------------------------------------------------ S10: the lattice

    def _buildGearProfiles(self, targetPlane, module, drivingTeeth, pinionTeeth, sigma):
        sketch = self.designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions

        R = self._pitchConeDistance_cm
        dpd = self._drivingPitchDia_cm
        ppd = self._pinionPitchDia_cm
        gammaP = self._gamma_p
        gammaG = self._gamma_g
        moduleLength = to_cm(module)
        dedendum = to_cm(_DEDENDUM_MODULES * module)

        # --- the projections ---------------------------------------------------
        # Project the ANCHOR SKETCH's centre SketchPoint and its line, never the
        # raw user selection: that keeps the chain inside the Design component.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchor = sketch.project(self._anchorLine).item(0)

        centre = _xy(projectedCenter.geometry)
        anchorDir = _unit(_sub(_xy(projectedAnchor.endSketchPoint.geometry),
                               _xy(projectedAnchor.startSketchPoint.geometry)))

        # --- the apex and the two shaft axes -----------------------------------
        perp = _perpendicular(anchorDir)
        # The sign of perp is chosen by the target-plane normal — the one-bit
        # direction comparison that is the only permitted world use in §2
        # ([BEVEL-F-GROW-SIDE]). The Gear Profiles plane stands perpendicular to the
        # target plane through the anchor line, so that normal lies in this sketch's
        # own plane along +/- perp, and the figure grows off the target plane.
        growFrom = sketch.sketchToModelSpace(_p3(centre))
        growTo = sketch.sketchToModelSpace(_p3(_add(centre, perp)))
        if growFrom.vectorTo(growTo).dotProduct(targetPlane.geometry.normal) < 0:
            perp = _scale(perp, -1)

        drivingBaseHeight = self._drivingBaseHeightResolved_cm
        pinionBaseHeight = self._pinionBaseHeightResolved_cm

        # Seeded at the distance the constraint net closes this line at, which is
        # R*cos(γ_g) above point I plus the resolved driving base height
        # ([PB-SEED-NEAR]) — NOT at the Driving Gear Pitch Diameter.
        apex = _add(centre, _scale(perp, R * math.cos(gammaG) + drivingBaseHeight))
        centerToApex = self._line(sketch, centre, apex, projectedCenter, None)
        constraints.addPerpendicular(centerToApex, projectedAnchor)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint

        # The Driving Gear Shaft Axis, back toward the anchor line. Its far end is
        # measured FROM THE APEX, and its length is not dimensioned
        # ([BEVEL-F-DRIVEN-DIMS]).
        drivingDir = _scale(perp, -1)
        pointBSeed = _add(apex, _scale(drivingDir, R * math.cos(gammaG)))
        drivingAxis = self._line(sketch, apex, pointBSeed, apexPoint, None)
        # addParallel and never addVertical, which would force the line to the
        # sketch's world-vertical and mis-orient the figure on a tilted plane.
        constraints.addParallel(drivingAxis, centerToApex)
        pointB = drivingAxis.endSketchPoint

        # The Pinion Gear Shaft Axis: the driving direction rotated about the apex
        # by the Shaft Angle. Form BOTH candidates and keep the one whose endpoint
        # has the greater X in this sketch.
        plusDir = _turn(drivingDir, sigma)
        minusDir = _turn(drivingDir, -sigma)
        plusA = _add(apex, _scale(plusDir, R * math.cos(gammaP)))
        minusA = _add(apex, _scale(minusDir, R * math.cos(gammaP)))
        if minusA[0] > plusA[0]:
            pinionDir, pointASeed = minusDir, minusA
        else:
            pinionDir, pointASeed = plusDir, plusA
        pinionAxis = self._line(sketch, apex, pointASeed, apexPoint, None)
        pointA = pinionAxis.endSketchPoint

        # The text point goes inside the Σ wedge so the dimension measures Σ and
        # not its supplement ([PB-ANGULAR-DIM]).
        wedgePoint = _add(apex, _scale(_unit(_add(pinionDir, drivingDir)), ppd / 4))
        angularDimension = dimensions.addAngularDimension(
            pinionAxis, drivingAxis, _p3(wedgePoint))
        angularDimension.parameter.value = sigma

        # --- the two drops and Apex 2 ------------------------------------------
        # The drop from A must point toward the OTHER shaft axis, toward B.
        dropDirA = _perpendicular(pinionDir)
        if _dot(dropDirA, _sub(pointBSeed, pointASeed)) < 0:
            dropDirA = _scale(dropDirA, -1)
        apex2 = _add(pointASeed, _scale(dropDirA, ppd / 2))
        dropA = self._line(sketch, pointASeed, apex2, pointA, None)
        constraints.addPerpendicular(dropA, pinionAxis)
        self._alignedDistance(sketch, dropA.startSketchPoint, dropA.endSketchPoint,
                              _midpoint2(pointASeed, apex2), ppd / 2)
        apex2Point = dropA.endSketchPoint

        # The twin from B, its sense picked by the dot with the B->A direction and
        # never against a "toward the anchor line" reference: the Driving Gear
        # Shaft Axis is itself parallel to that direction, which makes the test
        # degenerate and can seed Apex 2 on the mirror side.
        dropDirB = _perpendicular(drivingDir)
        if _dot(dropDirB, _sub(pointASeed, pointBSeed)) < 0:
            dropDirB = _scale(dropDirB, -1)
        dropB = self._line(sketch, pointBSeed, apex2, pointB, apex2Point)
        constraints.addPerpendicular(dropB, drivingAxis)
        self._alignedDistance(sketch, dropB.startSketchPoint, dropB.endSketchPoint,
                              _midpoint2(pointBSeed, apex2), dpd / 2)

        # --- the pitch line, the dedendum lines and the root axes ---------------
        pitchLine = self._line(sketch, apex, apex2, apexPoint, apex2Point)

        pitchDir = _unit(_sub(apex2, apex))
        toward = _perpendicular(pitchDir)
        if _dot(toward, _scale(perp, -1)) < 0:
            toward = _scale(toward, -1)
        drivingDedSeed = _add(apex2, _scale(toward, dedendum))
        pinionDedSeed = _sub(apex2, _scale(toward, dedendum))

        drivingDedLine = self._line(sketch, apex2, drivingDedSeed, apex2Point, None)
        constraints.addPerpendicular(drivingDedLine, pitchLine)
        self._alignedDistance(sketch, drivingDedLine.startSketchPoint,
                              drivingDedLine.endSketchPoint,
                              _midpoint2(apex2, drivingDedSeed), dedendum)
        pointD = drivingDedLine.endSketchPoint

        pinionDedLine = self._line(sketch, apex2, pinionDedSeed, apex2Point, None)
        constraints.addPerpendicular(pinionDedLine, pitchLine)
        self._alignedDistance(sketch, pinionDedLine.startSketchPoint,
                              pinionDedLine.endSketchPoint,
                              _midpoint2(apex2, pinionDedSeed), dedendum)
        pointC = pinionDedLine.endSketchPoint

        pinionRootAxis = self._line(sketch, apex, pinionDedSeed, apexPoint, pointC)
        drivingRootAxis = self._line(sketch, apex, drivingDedSeed, apexPoint, pointD)

        # --- the two dedendum chains and the base-height offsets ----------------
        pinion = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'pitchDia': ppd,
            'gamma': gammaP,
            'baseHeight': pinionBaseHeight,
            'toeRadiusInput': self._pinionToeRadius_cm,
            'bore': self._pinionBoreResolved_cm,
            'axisDir': _unit(_sub(pointASeed, apex)),
            'axisSeed': pointASeed,
            'axisLine': pinionAxis,
            'axisPoint': pointA,
            'dedSeed': pinionDedSeed,
            'dedDir': _unit(_sub(pinionDedSeed, apex2)),
            'dedLine': pinionDedLine,
            'dedPoint': pointC,
            'dropLine': dropA,
            'rootAxis': pinionRootAxis,
            # S30: the pinion's extra mesh phase, 0 by default.
            'meshAngle': self._pinionMeshPhase(pinionTeeth),
        }
        driving = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'pitchDia': dpd,
            'gamma': gammaG,
            'baseHeight': drivingBaseHeight,
            'toeRadiusInput': self._drivingToeRadius_cm,
            'bore': self._drivingBoreResolved_cm,
            'axisDir': _unit(_sub(pointBSeed, apex)),
            'axisSeed': pointBSeed,
            'axisLine': drivingAxis,
            'axisPoint': pointB,
            'dedSeed': drivingDedSeed,
            'dedDir': _unit(_sub(drivingDedSeed, apex2)),
            'dedLine': drivingDedLine,
            'dedPoint': pointD,
            'dropLine': dropB,
            'rootAxis': drivingRootAxis,
            # S30: half a tooth pitch, so a driving valley sits where the pinion
            # tooth crosses the axial plane.
            'meshAngle': math.radians(180.0 / drivingTeeth),
        }

        for side in (pinion, driving):
            self._buildRim(sketch, side, apex, apex2, moduleLength)

        # --- closing the figure -------------------------------------------------
        # I is on B->F, collinear with the driving shaft axis, which passes through
        # the projected centre by construction, so this pins the one translation
        # the figure still has.
        constraints.addCoincident(driving['ext2Point'], projectedCenter)

        # --- K, L and the two tooth centres -------------------------------------
        for side in (pinion, driving):
            self._buildToothCentre(sketch, side)

        # --- the Maximum Face Width, from SOLVED geometry -----------------------
        # A, B, C, D, H and J all exist and are solved, so read `.geometry` and
        # never the pre-solve seeds ([PB-SOLVED-GEOMETRY]).
        pinionReach = _perpendicular_distance(
            _xy(pointA.geometry), _xy(pointC.geometry),
            _unit(_sub(_xy(pinion['heelPoint'].geometry), _xy(pointC.geometry))))
        drivingReach = _perpendicular_distance(
            _xy(pointB.geometry), _xy(pointD.geometry),
            _unit(_sub(_xy(driving['heelPoint'].geometry), _xy(pointD.geometry))))
        maximumFaceWidth = _FACE_WIDTH_MARGIN * min(pinionReach, drivingReach)

        self._resolveFaceWidthAndToe(module, maximumFaceWidth, pinion, driving)

        # --- the two toe ends ---------------------------------------------------
        for side in (pinion, driving):
            self._buildToe(sketch, side, apex)

        # --- the two hexagon shaft-axis edges -----------------------------------
        # A'->G and B'->I. The spec's prose lists A'->G before A' exists; it cannot
        # be drawn there.
        for side in (pinion, driving):
            self._line(sketch, side['footSeed'], side['ext2Seed'],
                       side['footPoint'], side['ext2Point'])

        self._gateSketch(sketch)

        for side in (pinion, driving):
            side['hexagon'] = [side['footPoint'], side['ext2Point'], side['heelPoint'],
                               side['dedPoint'], side['toePoint'], side['innerPoint']]
            side['toeEdge'] = (side['toePoint'], side['innerPoint'])
            side['heelEdge'] = (side['dedPoint'], side['heelPoint'])
            side['toeCone'] = side['toePoint']
            side['heelCone'] = side['dedPoint']

        return pinion, driving

    def _buildRim(self, sketch: adsk.fusion.Sketch, side, apex, apex2, moduleLength):
        """One gear's dedendum chain: A->E, C->E, E->G, C->H, G->H and the
        base-height offset (B->F, D->F, F->I, D->J, I->J for the driving gear)."""
        constraints = sketch.geometricConstraints
        axisDir = side['axisDir']
        dedDir = side['dedDir']

        extSolved, ext2Solved, heelSolved, centreSolved = _solve_rim(
            apex, apex2, side['axisSeed'], axisDir, side['dedSeed'], dedDir,
            side['baseHeight'])

        # A->E (B->F): collinear with the Apex->A (Apex->B) shaft axis, extending
        # one Module as a SEED only, its length driven by the perpendicular below.
        extSeed = _add(side['axisSeed'], _scale(axisDir, moduleLength))
        extLine = self._line(sketch, side['axisSeed'], extSeed, side['axisPoint'], None)
        constraints.addCollinear(extLine, side['axisLine'])
        extPoint = extLine.endSketchPoint

        # C->E (D->F), perpendicular to that extension: this is what puts E (F) at
        # the dedendum corner's own station on the shaft axis.
        connector = self._line(sketch, side['dedSeed'], extSeed, side['dedPoint'], extPoint)
        constraints.addPerpendicular(extLine, connector)

        # E->G (F->I): collinear with line A->E (B->F), NEVER with the Apex->A
        # shaft axis further up the chain ([BEVEL-F-COLLINEAR-CHAIN],
        # [PB-COLLINEAR-CHAIN]) — naming the axis raises VCS_SKETCH_OVER_CONSTRAINTS.
        ext2Seed = _add(extSolved, _scale(axisDir, moduleLength))
        ext2Line = self._line(sketch, extSolved, ext2Seed, extPoint, None)
        constraints.addCollinear(ext2Line, extLine)
        ext2Point = ext2Line.endSketchPoint

        # C->H (D->J): collinear with the dedendum line Apex2->C (Apex2->D) that C
        # (D) is the endpoint of.
        heelSeed = _add(side['dedSeed'], _scale(dedDir, moduleLength))
        heelStub = self._line(sketch, side['dedSeed'], heelSeed, side['dedPoint'], None)
        constraints.addCollinear(heelStub, side['dedLine'])
        heelPoint = heelStub.endSketchPoint

        # G->H (I->J), the heel edge. The perpendicular is what makes H->G parallel
        # to the A->Apex2 drop, which the offset dimension below requires.
        heelLine = self._line(sketch, ext2Solved, heelSolved, ext2Point, heelPoint)
        constraints.addPerpendicular(ext2Line, heelLine)

        # The base-height offset: between the axis->Apex2 perpendicular DROP line —
        # not the shaft axis — and J->I, which is already parallel by construction,
        # so no extra addParallel ([PB-OFFSET-DIM]).
        offsetDimension = sketch.sketchDimensions.addOffsetDimension(
            side['dropLine'], heelLine, _p3(_midpoint2(ext2Solved, heelSolved)))
        offsetDimension.parameter.value = side['baseHeight']

        side['extLine'] = extLine
        side['extPoint'] = extPoint
        side['extSolved'] = extSolved
        side['ext2Line'] = ext2Line
        side['ext2Point'] = ext2Point
        side['ext2Seed'] = ext2Solved
        side['heelStub'] = heelStub
        side['heelPoint'] = heelPoint
        side['heelSeed'] = heelSolved
        side['heelLine'] = heelLine
        side['centreSeed'] = centreSolved

    def _buildToothCentre(self, sketch: adsk.fusion.Sketch, side):
        """K (L) and the Tooth Spacing offset K' (L'), plus the tooth-centre
        reference line C->K' (D->L') §3 draws its plane from."""
        constraints = sketch.geometricConstraints
        centreSeed = side['centreSeed']

        centreLine = self._line(sketch, side['ext2Seed'], centreSeed, side['ext2Point'], None)
        centrePoint = centreLine.endSketchPoint
        # By the time K is added, G and C are already fixed, so an addCollinear here
        # over-constrains the sketch; two point-on-line coincidents locate K exactly
        # ([BEVEL-F-COLLINEAR-CHAIN]).
        constraints.addCoincident(centrePoint, side['axisLine'])
        constraints.addCoincident(centrePoint, side['dedLine'])

        if self._toothSpacing_cm <= 0:
            # K' IS K: build nothing here and reuse the existing C->K reference
            # line. A zero-length dimensioned line is degenerate, and one segment
            # gets one line ([BEVEL-F-LINE-ONCE]).
            referenceLine = self._line(sketch, side['dedSeed'], centreSeed,
                                       side['dedPoint'], centrePoint)
            side['toothPoint'] = centrePoint
            side['referenceLine'] = referenceLine
            return

        # Shift K outward along the dedendum line, away from the lower corner C (D).
        toothSeed = _add(centreSeed, _scale(side['dedDir'], self._toothSpacing_cm))
        spacingLine = self._line(sketch, centreSeed, toothSeed, centrePoint, None)
        toothPoint = spacingLine.endSketchPoint
        constraints.addCoincident(toothPoint, side['dedLine'])
        self._alignedDistance(sketch, spacingLine.startSketchPoint,
                              spacingLine.endSketchPoint,
                              _midpoint2(centreSeed, toothSeed), self._toothSpacing_cm)
        referenceLine = self._line(sketch, side['dedSeed'], toothSeed,
                                   side['dedPoint'], toothPoint)
        side['toothPoint'] = toothPoint
        side['referenceLine'] = referenceLine

    def _resolveFaceWidthAndToe(self, module, maximumFaceWidth, pinion, driving):
        """S6 plus S10's cap: the Face Width's default and bound, the two Toe
        Radii, and the Root Length the toe lines are offset by."""
        R = self._pitchConeDistance_cm

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maximumFaceWidth:
                raise Exception(
                    f'Face Width must not exceed the Maximum Face Width '
                    f'{maximumFaceWidth * 10:.4f} mm (got {self._faceWidth_cm * 10:.4f} mm); '
                    f'above it the toe line pushes the inner toe corner across the shaft '
                    f'axis and the revolve fails with ASM_WIRE_X_AXIS')
            faceWidth = self._faceWidth_cm
        else:
            faceWidth = min(self._coneDistance_cm / 6, maximumFaceWidth)
        self._faceWidthResolved_cm = faceWidth

        # |Apex->Ded|, the root element's length from the apex to the dedendum
        # corner.
        apexToDed = math.hypot(R, to_cm(_DEDENDUM_MODULES * module))

        toeLimits = []
        for side in (pinion, driving):
            gamma = side['gamma']
            pitchRadius = side['pitchDia'] / 2
            # 0 means auto: this gear's inner toe corner radius at Toe Extension 0,
            # which is what makes Toe Extension 0 today's profile exactly.
            if side['toeRadiusInput'] > 0:
                ceiling = (pitchRadius - to_cm(_DEDENDUM_MODULES * module) * math.cos(gamma)) \
                    * (1 - faceWidth / R)
                if side['toeRadiusInput'] >= ceiling:
                    raise Exception(
                        f'{side["label"]} Gear Toe Radius must be strictly below the Toe '
                        f'Radius Ceiling {ceiling * 10:.4f} mm '
                        f'(got {side["toeRadiusInput"] * 10:.4f} mm)')
                toeRadius = side['toeRadiusInput']
            else:
                toeRadius = pitchRadius - faceWidth / math.sin(gamma)
            side['toeRadius'] = toeRadius

            rootConeAngle = gamma - math.atan(to_cm(_DEDENDUM_MODULES * module) / R)
            toeLimits.append(apexToDed - toeRadius / math.sin(rootConeAngle))

        rootLengthAtZero = faceWidth * apexToDed / R
        limit = min(toeLimits)

        if self._toeExtension > 0 and limit <= rootLengthAtZero:
            # A defaulted Toe Radius can leave no room at all on a gear with a
            # large pitch cone angle; that is a real configuration, and silently
            # substituting a smaller Toe Radius would change the toe end of a gear
            # whose inputs asked for no change.
            blocking = pinion if toeLimits[0] <= toeLimits[1] else driving
            ceiling = (blocking['pitchDia'] / 2
                       - to_cm(_DEDENDUM_MODULES * module) * math.cos(blocking['gamma'])) \
                * (1 - faceWidth / R)
            raise Exception(
                f'Toe Extension must be 0 for this pair: the {blocking["label"]} Gear\'s '
                f'Toe Limit {limit * 10:.4f} mm is at or below the Toe Extension 0 root '
                f'length {rootLengthAtZero * 10:.4f} mm. Give the {blocking["label"]} Gear '
                f'a Toe Radius strictly below {ceiling * 10:.4f} mm to extend the toe')

        # Toe Extension 100 stops at 0.99 of the way to the smaller Toe Limit: AT
        # the limit the toe face has zero length, the body carries no cone at its
        # toe end, and S22's conical end-cut has no ConeSurfaceType face to find.
        self._rootLength_cm = rootLengthAtZero + (self._toeExtension / 100) \
            * _TOE_EXTENSION_REACH * (limit - rootLengthAtZero)

    def _buildToe(self, sketch: adsk.fusion.Sketch, side, apexSeed):
        """One gear's toe end: M->N (O->P), its connector M->C (O->D) and the front
        face N->A' (P->B') that holds N off the shaft axis."""
        constraints = sketch.geometricConstraints
        rootLength = self._rootLength_cm
        R = self._pitchConeDistance_cm

        # Seed BOTH ends at their closed-form SOLVED positions ([PB-SEED-NEAR]);
        # a wrong seed here builds the wrong gear rather than failing to converge,
        # and the proof cannot catch it.
        apex = _xy(self._apexSketchPoint.geometry)
        ded = _xy(side['dedPoint'].geometry)
        heel = _xy(side['heelPoint'].geometry)
        axisEnd = _xy(side['axisPoint'].geometry)
        axisDir = _unit(_sub(axisEnd, apex))
        rootDir = _unit(_sub(ded, apex))
        dedDir = _unit(_sub(heel, ded))
        apexToDed = _length(_sub(ded, apex))

        # M on Apex->C at the fraction 1 - Root Length / |Apex->C| from the Apex.
        toeSeed = _add(apex, _scale(rootDir, apexToDed - rootLength))
        # N slid from that M seed along C->H by
        # (M's perpendicular distance from the shaft axis - Toe Radius) / cos γ.
        # Sliding by the Root Length, or by the distance from M to A, both put N
        # past the shaft axis and the revolve then aborts with ASM_WIRE_X_AXIS.
        slide = (_perpendicular_distance(toeSeed, apex, axisDir) - side['toeRadius']) \
            / math.cos(side['gamma'])
        innerSeed = _add(toeSeed, _scale(dedDir, slide))

        toeLine = self._line(sketch, toeSeed, innerSeed, None, None)
        toePoint = toeLine.startSketchPoint
        innerPoint = toeLine.endSketchPoint

        constraints.addCoincident(toePoint, side['rootAxis'])
        constraints.addParallel(toeLine, side['heelStub'])
        # An offset dimension controls a PERPENDICULAR distance, so the Root Length
        # is re-measured perpendicular to the pitch line. At Toe Extension 0 that
        # value is exactly the resolved Face Width.
        offsetDimension = sketch.sketchDimensions.addOffsetDimension(
            side['heelStub'], toeLine, _p3(_midpoint2(toeSeed, ded)))
        offsetDimension.parameter.value = rootLength * R / apexToDed

        self._line(sketch, toeSeed, ded, toePoint, side['dedPoint'])

        # The front face N->A'. N is NEVER pinned to the shaft axis — it rides the
        # Toe Radius, and only A' touches the axis, as a foot rather than a corner.
        footSeed = _foot_on_line(innerSeed, apex, axisDir)
        frontFace = self._line(sketch, innerSeed, footSeed, innerPoint, None)
        footPoint = frontFace.endSketchPoint
        constraints.addCoincident(footPoint, side['axisLine'])
        constraints.addPerpendicular(frontFace, side['axisLine'])
        self._alignedDistance(sketch, frontFace.startSketchPoint, frontFace.endSketchPoint,
                              _midpoint2(innerSeed, footSeed), side['toeRadius'])

        side['toePoint'] = toePoint
        side['innerPoint'] = innerPoint
        side['footPoint'] = footPoint
        side['footSeed'] = footSeed
        side['toeLine'] = toeLine

    # ------------------------------------------------------------ S11-S31

    def _buildGear(self, ctx, module):
        label = ctx['label']
        design = self.designComponent
        features = design.features

        # --- S11: the per-gear component ---------------------------------------
        # A child of the BEVEL GEAR component — the same component that owns
        # Design — and not the user's Parent Component.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{label} Gear'

        # --- S12: the virtual tooth geometry and the tooth plane ---------------
        gamma = ctx['gamma']
        # The stashed pitch diameters are internal cm while Module is raw mm, so
        # the *10 converts cm to mm; skipping it makes the count about ten times off.
        virtualPitchRadius_mm = (ctx['pitchDia'] * 10 / 2) / math.cos(gamma)
        # A REAL number, never rounded: z_v = 2*r_v/Module is what makes the drawn
        # pitch circle reach the back cone.
        virtualTeeth = 2 * virtualPitchRadius_mm / module
        rootSink_mm = _ROOT_SINK_FRACTION * module

        toothPlane = solids.plane_by_angle(
            design, ctx['referenceLine'], self._gearProfilesPlane, 90)
        toothPlane.name = f'{label} Plane'

        # --- S13: the borrowed virtual spur tooth ------------------------------
        toothSketch = design.sketches.add(toothPlane)
        toothSketch.name = f'{label} Tooth'
        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth,
                                 rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(ctx['toothPoint'], angle=math.radians(180))
        # The spur generator decides during draw() whether the tooth is embedded and
        # records it on the proxy; this is the deterministic selector for the tooth
        # loop's line count in S17.
        embedded = proxy._lastToothEmbedded
        # Never gate a tooth sketch: the spur drawer labels each circle with
        # along-path sketch text, and sketch text holds a DOF ([PB-TEXT-HOLDS-DOF]).
        if not toothSketch.isFullyConstrained:
            futil.log(f'{label} Tooth sketch reports not fully constrained; the four circle '
                      f'labels hold a DOF ([PB-TEXT-HOLDS-DOF]), so this is logged, never raised')

        # --- S14: the tooth axis ------------------------------------------------
        planes = design.constructionPlanes
        helperInput = planes.createInput()
        helperInput.setByDistanceOnPath(ctx['referenceLine'],
                                        adsk.core.ValueInput.createByReal(1.0))
        helperPlane = planes.add(helperInput)
        helperPlane.name = f'{label} Tooth Axis Helper'
        axisInput = design.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = design.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Tooth Axis'

        # --- S15: the frustum hexagon ------------------------------------------
        profileSketch = design.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{label} Profile'
        # Recreate the six §2 vertices, draw the hexagon SHARING them, and fix the
        # endpoints only AFTER the lines exist ([PB-PROJECT-NOT-FIXED]).
        vertices = [profileSketch.sketchPoints.add(
            profileSketch.modelToSketchSpace(source.worldGeometry))
            for source in ctx['hexagon']]
        hexagonLines = []
        for i in range(6):
            hexagonLines.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(
                vertices[i], vertices[(i + 1) % 6]))
        for line in hexagonLines:
            line.startSketchPoint.isFixed = True
            line.endSketchPoint.isFixed = True
        self._gateSketch(profileSketch)
        # The hexagon's FIRST edge is this gear's shaft axis for the revolve, the
        # pattern, the bore plane and the meshing rotation
        # ([PB-WORLDGEO-CONSTRAINED]).
        shaftAxisEdge = hexagonLines[0]

        # --- S16: revolve the Gear Body ----------------------------------------
        # This sketch holds exactly one hexagon loop ([PB-SINGLE-PROFILE]).
        revolveInput = features.revolveFeatures.createInput(
            profileSketch.profiles.item(0), shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = features.revolveFeatures.add(revolveInput)
        gearBody = revolve.bodies.item(0)

        # --- S17: loft the Tooth Body ------------------------------------------
        # The line count is DETERMINED by the embedded flag and never guessed.
        wantLines = 0 if embedded else 2
        toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The §2 Apex SKETCH point is the degenerate point-section; no construction
        # point, because the Design component is never active
        # ([PB-CONSTRUCTION-NEEDS-ACTIVE]).
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = features.loftFeatures.add(loftInput)
        toothBody = loft.bodies.item(0)

        # --- S18: the tooth-body hook ------------------------------------------
        toeStart, toeEnd = ctx['toeEdge']
        heelStart, heelEnd = ctx['heelEdge']
        toeMid = _world_midpoint(toeStart.worldGeometry, toeEnd.worldGeometry)
        heelMid = _world_midpoint(heelStart.worldGeometry, heelEnd.worldGeometry)
        apexWorld = self._apexSketchPoint.worldGeometry
        finishedTooth = self._transformToothBody(
            design, toothBody, gearBody, shaftAxisEdge, apexWorld, self._apexSketchPoint,
            toeMid, heelMid, ctx['toeCone'].worldGeometry, ctx['heelCone'].worldGeometry,
            toothPlane, label, ctx['teeth'], gamma)

        # --- S26: circular-pattern the tooth -----------------------------------
        patternBodies = adsk.core.ObjectCollection.create()
        patternBodies.add(finishedTooth)
        patternInput = features.circularPatternFeatures.createInput(patternBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = features.circularPatternFeatures.add(patternInput)

        # --- S27: Combine-Join the teeth into the Gear Body --------------------
        # pattern.bodies already includes the seed plus the copies, and it is a
        # BRepBodies the combine rejects ([PB-PATTERN-BODIES]).
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))
        combineInput = features.combineFeatures.createInput(gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        features.combineFeatures.add(combineInput)

        # --- S28 / S29: the bore -----------------------------------------------
        if self._boreEnable:
            boreInput = planes.createInput()
            boreInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
            borePlane = planes.add(boreInput)
            borePlane.name = f'{label} Bore Plane'

            boreSketch = design.sketches.add(borePlane)
            boreSketch.name = f'{label} Bore'
            boreRadius = ctx['bore'] / 2
            # The plane is rooted at the shaft start, so the sketch origin is on the
            # axis. addByCenterRadius does NOT reuse originPoint, so the centre is
            # fixed rather than coincident to it ([PB-CIRCLE-CENTER]).
            boreCircle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), boreRadius)
            boreCircle.centerSketchPoint.isFixed = True
            boreSketch.sketchDimensions.addDiameterDimension(
                boreCircle, adsk.core.Point3D.create(boreRadius, 0, 0))
            self._gateSketch(boreSketch)

            extrudeInput = features.extrudeFeatures.createInput(
                boreSketch.profiles.item(0),
                adsk.fusion.FeatureOperations.CutFeatureOperation)
            # isFullLength=False means the distance is the half-length per side, and
            # 2 * Cone Distance is generously past any face width ([PB-THROUGH-CUT]).
            extrudeInput.setSymmetricExtent(
                adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
            extrudeInput.participantBodies = [gearBody]
            features.extrudeFeatures.add(extrudeInput)

        # --- S30: the meshing rotation -----------------------------------------
        # Here in Design, before the body is moved out, because the rotation reads
        # the shaft edge's world geometry.
        # The driving gear turns half a tooth pitch; the pinion takes its own mesh
        # phase, which is 0 by default and which rotate_body_about_edge absorbs.
        solids.rotate_body_about_edge(design, gearBody, shaftAxisEdge, ctx['meshAngle'])

        # --- S31: move the finished body into its gear component ---------------
        gearBody.moveToComponent(gearOccurrence)

    def _pinionMeshPhase(self, pinionTeeth):
        # The pinion's extra mesh rotation in radians; 0 by default, and
        # rotate_body_about_edge absorbs a zero angle.
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # ------------------------------------------------------------ S18-S25: the tooth

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                            toothBody: adsk.fusion.BRepBody,
                            gearBody: adsk.fusion.BRepBody,
                            shaftAxisEdge: adsk.fusion.SketchLine,
                            apexWorld: adsk.core.Point3D,
                            apexSketchPoint: adsk.fusion.SketchPoint,
                            toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                            toeConeWorld: adsk.core.Point3D,
                            heelConeWorld: adsk.core.Point3D,
                            parentToothPlane: adsk.fusion.ConstructionPlane,
                            gearLabel, teethNumber, gamma):
        """The single tooth-body hook, called after lofting the uncut apex-to-heel
        tooth and before pattern, combine and bore."""
        if self._spiralAngle_rad <= 0:
            # S25: the straight tooth, byte for byte the prior behaviour.
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        features = designComponent.features

        # --- S19 A: the frame ---------------------------------------------------
        # Read the shaft-axis endpoints in WORLD space and measure every angle and
        # distance against world quantities ([PB-WORLD-FRAME]).
        startWorld = shaftAxisEdge.startSketchPoint.worldGeometry
        endWorld = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = startWorld.vectorTo(endWorld)
        axisDir.normalize()

        # The heel MUST be the outer end so coneVec points outward and span > 0; a
        # negative span silently inverts the entire spiral frame.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = apexWorld.vectorTo(heelConeWorld)
        coneVec.normalize()
        circumferential = axisDir.crossProduct(coneVec)
        circumferential.normalize()
        tangentPlaneNormal = coneVec.crossProduct(circumferential)
        tangentPlaneNormal.normalize()

        # A point's cone distance is (p - apexWorld) . coneVec, and everything
        # below is measured with it.
        rToe = _cone_distance(toeMid, apexWorld, coneVec)
        rHeel = _cone_distance(heelMid, apexWorld, coneVec)
        rMean = (rToe + rHeel) / 2
        span = rHeel - rToe
        futil.log(f'{gearLabel} spiral frame: R_toe={rToe:.6f} R_heel={rHeel:.6f} '
                  f'span={span:.6f} tangent-plane normal '
                  f'({tangentPlaneNormal.x:.4f}, {tangentPlaneNormal.y:.4f}, '
                  f'{tangentPlaneNormal.z:.4f})')

        # --- S19 B: the cutter-arc geometry ------------------------------------
        cutterRadius = self._cutterRadius_cm if self._cutterRadius_cm > 0 else rMean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            # The pair meshes with opposite hands.
            handSign = -handSign
        psi = self._spiralAngle_rad
        # The hand sign goes on the cos/Cy term and NEVER on the sin/Cx term:
        # opposite hand mirrors the cutter centre across the cone element.
        centreX = rMean - cutterRadius * math.sin(psi)
        centreY = handSign * cutterRadius * math.cos(psi)

        rLo = rToe - _SPIRAL_ARC_OVERSHOOT * span
        rHi = rHeel + _SPIRAL_ARC_OVERSHOOT * span
        toe2d = solids.circle_intersect_nearest(rLo, centreX, centreY, cutterRadius, rMean, 0)
        heel2d = solids.circle_intersect_nearest(rHi, centreX, centreY, cutterRadius, rMean, 0)

        # --- S19 C: the 2-D trace sketch ---------------------------------------
        # The world points below are passed DIRECTLY into the sketch calls, with no
        # modelToSketchSpace conversion: the trace sketch is construction and
        # reference only, no downstream feature consumes it, and the twist is
        # computed analytically in S21.
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

        cutterCentre = solids.combine_point(apexWorld, centreX, coneVec, centreY, circumferential)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            cutterCentre, cutterRadius)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        # A radial/diameter text point must be OFF-CENTRE and on or near the curve
        # ([PB-RADIAL-DIM]).
        traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle,
            solids.combine_point(apexWorld, centreX + cutterRadius, coneVec,
                                 centreY, circumferential))

        toePoint = solids.combine_point(apexWorld, toe2d[0], coneVec, toe2d[1], circumferential)
        meanPoint = solids.combine_point(apexWorld, rMean, coneVec, 0.0, circumferential)
        heelPoint = solids.combine_point(apexWorld, heel2d[0], coneVec, heel2d[1], circumferential)
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            toePoint, meanPoint, heelPoint)
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radialDimension = traceSketch.sketchDimensions.addRadialDimension(traceArc, meanPoint)
        radialDimension.parameter.value = cutterRadius
        # This sketch is deliberately left with free DOF and is exempt from the
        # full-constraint gate; do not gate it.

        # --- S20 E: slice the tooth into slabs ---------------------------------
        planeNormal = parentToothPlane.geometry.normal
        planeOrigin = parentToothPlane.geometry.origin
        apexSide = planeOrigin.vectorTo(apexWorld).dotProduct(planeNormal)
        sign = 1.0 if apexSide > 0 else -1.0
        step = span / _SPIRAL_SLICE_DIVISOR

        def offsetsFor(s):
            return [s * (k + 1) * step for k in range(_SPIRAL_SLICE_STEPS)]

        pieces = solids.slice_body_by_offset_planes(
            designComponent, toothBody, parentToothPlane, offsetsFor(sign))
        if len(pieces) == 1:
            # The offset sign was wrong or the parent plane sits outside the
            # tooth's span: retry the whole cut once with the opposite sign.
            sign = -sign
            pieces = solids.slice_body_by_offset_planes(
                designComponent, pieces[0], parentToothPlane, offsetsFor(sign))
        if len(pieces) == 1:
            raise Exception(
                f'{gearLabel}: the spiral slice left the tooth in 1 piece after trying both '
                f'offset signs (span={span:.6f} cm, step={step:.6f} cm, last sign={sign:+.0f}); '
                f'the parent tooth plane does not cross the tooth\'s span')

        # --- S20 F: order the pieces and drop the apex scrap -------------------
        pieces.sort(key=lambda body: _cone_distance(
            body.physicalProperties.centerOfMass, apexWorld, coneVec))
        scrap = pieces[0]
        # Re-slice the list FIRST, and only then remove the scrap.
        segments = pieces[1:]
        features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                f'{gearLabel}: the spiral slice left no working segments after the apex-side '
                f'scrap was dropped (pieces={len(pieces)}, span={span:.6f} cm); the twist and '
                f'the crown both assume at least one segment')

        # --- S21 G: twist the segments -----------------------------------------
        # The conjugate crown-gear generation law: the work gear's shaft rotation
        # relates to the developed crown-plane azimuth by the roll ratio 1/sin γ,
        # with γ this gear's PITCH cone angle and never acos(coneVec · axisDir),
        # which is the root cone angle.
        phiCrown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phiCrown) / math.sin(gamma)

        for segment in segments:
            _, heelFace = self._slabEndFaces(segment, apexWorld, coneVec, gearLabel)
            rHeelFace = _cone_distance(heelFace.centroid, apexWorld, coneVec)
            # Centred on R_mean so the mid-face section stays unrotated, and keyed
            # on the segment's HEEL-FACE cone distance, never its centroid.
            angle = -handSign * total * (rMean - rHeelFace) / span
            if angle == 0:
                # setToRotation(0, ...) builds the identity and Fusion refuses it.
                continue
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(segment)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(angle, axisDir, apexWorld)
            moveInput = features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            features.moveFeatures.add(moveInput)

        # --- S22 H: crown the segments lengthwise ------------------------------
        # Recomputed AFTER the twist has moved the slabs.
        heelDistances = []
        for segment in segments:
            _, heelFace = self._slabEndFaces(segment, apexWorld, coneVec, gearLabel)
            heelDistances.append(_cone_distance(heelFace.centroid, apexWorld, coneVec))
        outermost = heelDistances.index(max(heelDistances))

        # scaleFeatures is the ONE exception to never-activate: it needs the Design
        # occurrence as the active edit target.
        self.designOccurrence.activate()
        try:
            for index, segment in enumerate(segments):
                if index == outermost:
                    # The heel segment stays full: its heel face is the loft's heel
                    # end and the heel cone in S24 trims it flush with the base.
                    continue
                u = (rHeel - heelDistances[index]) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: the lengthwise crown computed a non-positive scale '
                        f'factor {factor:.6f} for the segment at heel-distance fraction '
                        f'u={u:.6f} (total twist {total:.6f} rad, _CROWN_PER_RAD '
                        f'{self._CROWN_PER_RAD})')
                _, heelFace = self._slabEndFaces(segment, apexWorld, coneVec, gearLabel)
                basePoint = self._heelRootBasePoint(
                    designComponent, heelFace, apexWorld, axisDir, gearLabel, index)
                bodies = adsk.core.ObjectCollection.create()
                bodies.add(segment)
                scaleInput = features.scaleFeatures.createInput(
                    bodies, basePoint, adsk.core.ValueInput.createByReal(factor))
                features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # --- S23 I: loft the spiral tooth --------------------------------------
        # Re-sort HERE, after the twist and the crown: for high-twist unequal-ratio
        # pairs the rotation reorders adjacent slabs, and lofting in the stale order
        # assembles the cross-sections out of sequence.
        endFaces = [self._slabEndFaces(segment, apexWorld, coneVec, gearLabel)
                    for segment in segments]
        order = sorted(range(len(segments)),
                       key=lambda i: _cone_distance(
                           endFaces[i][1].centroid, apexWorld, coneVec))

        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The toe-most segment's apex-side face first, so the loft reaches past the
        # toe cone and the toe trim bites.
        loftInput.loftSections.add(endFaces[order[0]][0])
        for index in order:
            loftInput.loftSections.add(endFaces[index][1])
        loft = features.loftFeatures.add(loftInput)
        curvedTooth = loft.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for segment in segments:
            features.removeFeatures.add(segment)

        # --- S24 J: the flush trim ---------------------------------------------
        return solids.cut_conical_ends(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    def _slabEndFaces(self, body: adsk.fusion.BRepBody, apexWorld: adsk.core.Point3D,
                      coneVec: adsk.core.Vector3D, gearLabel):
        """A slab's toe and heel faces: the faces whose centroids have the LEAST
        and GREATEST cone distance, searched across ALL of the slab's faces with NO
        surface-type filter — a type filter can pick the wrong face or miss the cut
        face, which makes the loft fail with ASM_NOT_ALL_SECTIONS_MEET."""
        toeFace = None
        heelFace = None
        least = None
        greatest = None
        for face in body.faces:
            distance = apexWorld.vectorTo(face.centroid).dotProduct(coneVec)
            if least is None or distance < least:
                least, toeFace = distance, face
            if greatest is None or distance > greatest:
                greatest, heelFace = distance, face
        if toeFace is None or heelFace is None:
            raise Exception(
                f'{gearLabel}: a spiral segment reported {body.faces.count} faces, so it has '
                f'no toe or heel end face to loft through')
        return toeFace, heelFace

    def _heelRootBasePoint(self, designComponent: adsk.fusion.Component,
                           heelFace: adsk.fusion.BRepFace, apexWorld: adsk.core.Point3D,
                           axisDir: adsk.core.Vector3D, gearLabel, index):
        """The crown's scale base: a sketch point at the MIDPOINT OF THE HEEL FACE'S
        ROOT EDGE, never the heel-face centroid. A uniform scale keeps every line
        through its base point invariant, so anchoring on the root keeps the root
        edge on the seating cone while the tip is relieved."""
        vertices = [heelFace.vertices.item(i).geometry
                    for i in range(heelFace.vertices.count)]
        if len(vertices) < 2:
            raise Exception(
                f'{gearLabel}: the heel face of spiral segment {index} carries '
                f'{len(vertices)} vertices, so its root edge cannot be found')
        # The two vertices nearest the shaft axis are the root corners; the tip
        # corners are the farthest from it.
        vertices.sort(key=lambda point: _axis_distance(point, apexWorld, axisDir))
        rootMidpoint = _world_midpoint(vertices[0], vertices[1])

        sketch = designComponent.sketches.add(heelFace)
        sketch.name = f'{gearLabel} Crown Base {index}'
        # The heel face is a planar cut, so that midpoint lies on it.
        return sketch.sketchPoints.add(sketch.modelToSketchSpace(rootMidpoint))
