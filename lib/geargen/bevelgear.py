# Bevel gear pair generator, emitted from spec/bevelgear/steps.md.
#
# Every value is precomputed in Python in Fusion-internal centimetres and written
# into geometry numerically; the module registers no Fusion user parameters
# ([PB-PRECOMPUTED-MODE]).

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy
from . import solids


# --------------------------------------------------------------------------
# S01: dialog input ids, in the table's display order.
# --------------------------------------------------------------------------

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


# --------------------------------------------------------------------------
# Local 2-D helpers for the §2 lattice. The Gear Profiles sketch is planar, so
# every lattice position is a sketch-local (x, y) pair in internal centimetres
# ([BEVEL-F-APEX-LOCAL]). None of these is a Fusion API call.
# --------------------------------------------------------------------------

def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _mul2(a, k):
    return (a[0] * k, a[1] * k)


def _dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _len2(a):
    return math.hypot(a[0], a[1])


def _unit2(a):
    n = _len2(a)
    return (a[0] / n, a[1] / n)


def _rot2(a, angle):
    s, c = math.sin(angle), math.cos(angle)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _leftperp2(a):
    return (-a[1], a[0])


def _dist_point_line2(p, a, direction):
    u = _unit2(direction)
    return abs(_cross2(_sub2(p, a), u))


def _pt2(p) -> adsk.core.Point3D:
    # A sketch-local 2-D position as the Point3D the sketch calls take.
    return adsk.core.Point3D.create(p[0], p[1], 0)


def _text2(a, b, nudge=0.25) -> adsk.core.Point3D:
    # A dimension text point beside the segment a->b, never on an endpoint.
    mid = _mul2(_add2(a, b), 0.5)
    d = _sub2(b, a)
    if _len2(d) == 0:
        return _pt2(_add2(mid, (nudge, nudge)))
    return _pt2(_add2(mid, _mul2(_unit2(_leftperp2(d)), nudge)))


# --------------------------------------------------------------------------
# Local world-frame helpers. Each takes its Fusion operands as typed
# parameters, so the arithmetic below reads only coordinates.
# --------------------------------------------------------------------------

def _vector_between(a: adsk.core.Point3D,
                    b: adsk.core.Point3D) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(b.x - a.x, b.y - a.y, b.z - a.z)


def _unit_between(a: adsk.core.Point3D,
                  b: adsk.core.Point3D) -> adsk.core.Vector3D:
    unit = adsk.core.Vector3D.create(b.x - a.x, b.y - a.y, b.z - a.z)
    unit.normalize()
    return unit


def _unit_cross(a: adsk.core.Vector3D,
                b: adsk.core.Vector3D) -> adsk.core.Vector3D:
    unit = a.crossProduct(b)
    unit.normalize()
    return unit


def _midpoint(a: adsk.core.Point3D,
              b: adsk.core.Point3D) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


def _along(origin: adsk.core.Point3D, point: adsk.core.Point3D,
           direction: adsk.core.Vector3D) -> float:
    # The component of origin->point along `direction`.
    return ((point.x - origin.x) * direction.x
            + (point.y - origin.y) * direction.y
            + (point.z - origin.z) * direction.z)


def _axis_distance(point: adsk.core.Point3D, origin: adsk.core.Point3D,
                   axisDir: adsk.core.Vector3D) -> float:
    # Perpendicular distance from `point` to the line through `origin` along
    # the unit vector `axisDir`.
    t = _along(origin, point, axisDir)
    dx = point.x - origin.x - t * axisDir.x
    dy = point.y - origin.y - t * axisDir.y
    dz = point.z - origin.z - t * axisDir.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _read_value(unitsManager: adsk.core.UnitsManager,
                inputs: adsk.core.CommandInputs, inputId, units) -> float:
    # Evaluating an input's expression ALWAYS returns Fusion internal units -
    # cm for length, radians for angle ([PB-EVAL-EXPRESSION]).
    return unitsManager.evaluateExpression(
        inputs.itemById(inputId).expression, units)


def _construction_line(lines: adsk.fusion.SketchLines, p0,
                       p1) -> adsk.fusion.SketchLine:
    # [BEVEL-F-COINCIDENT-STYLE]: every §2 line is built from raw Point3D
    # coordinates, so the caller can coincident each endpoint to the point it
    # meets. No existing SketchPoint is ever shared into the creation call.
    created = lines.addByTwoPoints(_pt2(p0), _pt2(p1))
    created.isConstruction = True
    return created


def _aligned_length(dims: adsk.fusion.SketchDimensions,
                    entity: adsk.fusion.SketchLine, value, a, b):
    # Every length dimension in the §2 sketch is aligned: the figure has no
    # axis-aligned line in it, so a horizontal or vertical orientation would
    # dimension a projection instead of a length.
    dim = dims.addDistanceDimension(
        entity.startSketchPoint, entity.endSketchPoint,
        adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
        _text2(a, b))
    dim.parameter.value = abs(value)
    return dim


# --------------------------------------------------------------------------
# S01 / S02: the command dialog.
# --------------------------------------------------------------------------

class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs
        design: adsk.fusion.Design = get_design()

        # 1. Target Plane. Added first so Fusion's auto-focus lands on it
        #    ([PB-AUTOFOCUS-FIRST]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point.
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component, with the root component pre-selected.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(design.rootComponent)

        # 4-9: the numeric fields. Every mm default goes in as internal cm and
        # every deg default as a parsed expression ([PB-DIALOG-DEFAULT-UNITS]).
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '',
                             adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
                             adsk.core.ValueInput.createByString('90 deg'))
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT,
                             'Driving Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT,
                             'Pinion Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore.
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True,
                                 '', True)

        # 11-15.
        inputs.addValueInput(INPUT_ID_DRIVING_BORE,
                             'Driving Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BORE,
                             'Pinion Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
                             adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17-20.
        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
                             adsk.core.ValueInput.createByReal(0))
        inputs.addValueInput(INPUT_ID_DRIVING_TOE_RADIUS,
                             'Driving Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_TOE_RADIUS,
                             'Pinion Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        # S02: the spiral-only rows start out matching the current angle.
        cls._updateSpiralInputVisibility(inputs)

    # S02. Hand of Spiral and Cutter Radius are relevant only to a curved
    # bevel, so they are hidden at psi = 0 and shown above it. There is no
    # declarative show-if in the Fusion API, hence commandInput.isVisible.
    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return

        # A half-typed expression can raise mid-edit; on failure leave both
        # rows shown ([PB-EVAL-EXPRESSION]).
        try:
            design: adsk.fusion.Design = get_design()
            unitsManager: adsk.core.UnitsManager = design.unitsManager
            value = unitsManager.evaluateExpression(spiral.expression, 'rad')
        except Exception:
            hand.isVisible = True
            cutter.isVisible = True
            return

        visible = (value > 0)
        hand.isVisible = visible
        cutter.isVisible = visible

    # Bound by commands/bevelgear/entry.py as the dialog's inputChanged
    # handler. Runs on every change: cheap, and no branch on which input moved.
    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


class BevelGearGenerator:
    # S20: the lengthwise crown's tunable relief per radian of twist. 0
    # disables the crown.
    _CROWN_PER_RAD = 0.5

    # S28: the pinion's extra mesh phase, in whole teeth.
    _PINION_MESH_PHASE_TEETH = 0

    def __init__(self, design: adsk.fusion.Design):
        self.design: adsk.fusion.Design = design
        self.bevelOccurrence: adsk.fusion.Occurrence = \
            adsk.fusion.Occurrence.cast(None)
        self.bevelComponent: adsk.fusion.Component = \
            adsk.fusion.Component.cast(None)
        self.designOccurrence: adsk.fusion.Occurrence = \
            adsk.fusion.Occurrence.cast(None)
        self.designComponent: adsk.fusion.Component = \
            adsk.fusion.Component.cast(None)
        # `_anchorCenterPoint`, `_apexSketchPoint` and `_gearProfilesPlane` are
        # bound as the build reaches them, and this constructor declares none
        # of the three. Each is passed as an argument or read for an
        # attribute, never called on, so none needs a declared type. The
        # projected anchor point arrives as the ObjectCollection item type,
        # which is the narrowest type this file can state for it, so it keeps
        # that type.

    # ----------------------------------------------------------------------
    # S04 rollback. There are no user parameters to clean up.
    # ----------------------------------------------------------------------
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()

    # ----------------------------------------------------------------------
    # S03: read and validate every input.
    # ----------------------------------------------------------------------
    def _readInputs(self, inputs: adsk.core.CommandInputs):
        unitsManager: adsk.core.UnitsManager = self.design.unitsManager

        def value(inputId, units):
            return _read_value(unitsManager, inputs, inputId, units)

        parentSelection = get_selection(inputs, INPUT_ID_PARENT)
        planeSelection = get_selection(inputs, INPUT_ID_PLANE)
        centerSelection = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if not parentSelection:
            raise Exception('Bevel Gear: no Parent Component selected')
        if not planeSelection:
            raise Exception('Bevel Gear: no Target Plane selected')
        if not centerSelection:
            raise Exception('Bevel Gear: no Center Point selected')

        parentEntity = parentSelection[0]
        parentComponent = getattr(parentEntity, 'component', parentEntity)
        targetPlane = planeSelection[0]
        centerPoint = centerSelection[0]

        # Module is read with unit '' so it comes back a raw number meaning
        # MILLIMETRES; every length derived from it needs to_cm. Every 'mm' /
        # 'deg' input comes back ALREADY internal - never to_cm one again.
        module = value(INPUT_ID_MODULE, '')
        shaftAngle_rad = value(INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(value(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(value(INPUT_ID_PINION_TEETH, '')))

        self._drivingBaseHeight_cm = value(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = value(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = value(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = value(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = value(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = value(INPUT_ID_TOOTH_SPACING, 'mm')
        self._spiralAngle_rad = value(INPUT_ID_SPIRAL_ANGLE, 'deg')

        selectedItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        self._hand = selectedItem.name if selectedItem else _HAND_RIGHT

        self._cutterRadius_cm = value(INPUT_ID_CUTTER_RADIUS, 'mm')
        self._toeExtension_pct = value(INPUT_ID_TOE_EXTENSION, '')
        self._drivingToeRadius_cm = value(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        self._pinionToeRadius_cm = value(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        # 1. The blanket range checks.
        if module <= 0:
            raise Exception(
                f'Module must be greater than 0, got {module}')
        if drivingTeeth < 3:
            raise Exception(
                f'Driving Gear Teeth Number must be at least 3, got {drivingTeeth}')
        if pinionTeeth < 3:
            raise Exception(
                f'Pinion Gear Teeth Number must be at least 3, got {pinionTeeth}')
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
                raise Exception(
                    f'{label} must not be negative, got {amount * 10} mm')
        if self._toeExtension_pct < 0 or self._toeExtension_pct > 100:
            raise Exception(
                f'Toe Extension must be within [0, 100], got {self._toeExtension_pct}')
        spiralAngle_deg = math.degrees(self._spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be within [0, 60) degrees, got '
                f'{spiralAngle_deg:.4f} deg')

        # 2. Shaft Angle, against the tooth-count-dependent maximum.
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        drivingPitchDia_mm = module * drivingTeeth
        pinionPitchDia_mm = module * pinionTeeth
        lo = min(drivingPitchDia_mm, pinionPitchDia_mm)
        hi = max(drivingPitchDia_mm, pinionPitchDia_mm)
        maxShaftAngle_deg = min(math.degrees(math.acos(-lo / hi)), 150.0)
        if shaftAngle_deg < 30:
            raise Exception(
                f'Shaft Angle must be at least 30 deg, got {shaftAngle_deg:.4f} deg')
        if (shaftAngle_deg > maxShaftAngle_deg
                or (shaftAngle_deg >= maxShaftAngle_deg and maxShaftAngle_deg < 150)):
            raise Exception(
                f'Shaft Angle {shaftAngle_deg:.4f} deg is at or above the Maximum '
                f'Shaft Angle {maxShaftAngle_deg:.4f} deg for a '
                f'{drivingTeeth}/{pinionTeeth} pair')

        # 3. Both pitch cone angles, from the closed form.
        gammaP = math.atan2(
            math.sin(shaftAngle_rad) * pinionPitchDia_mm,
            drivingPitchDia_mm + pinionPitchDia_mm * math.cos(shaftAngle_rad))
        gammaG = shaftAngle_rad - gammaP
        self._gamma_p = gammaP
        self._gamma_g = gammaG

        # 4. Minimum Teeth, per gear, with that gear's own gamma. This runs
        #    before the base-height resolution because it is exactly the
        #    statement that the base-height window is non-empty.
        pinionFloor = 5.27 * math.cos(gammaP)
        if pinionTeeth < pinionFloor:
            raise Exception(
                f'Pinion Gear Teeth Number {pinionTeeth} is below the computed '
                f'floor {pinionFloor:.3f} for its pitch cone angle '
                f'{math.degrees(gammaP):.3f} deg')
        drivingFloor = 5.27 * math.cos(gammaG)
        if drivingTeeth < drivingFloor:
            raise Exception(
                f'Driving Gear Teeth Number {drivingTeeth} is below the computed '
                f'floor {drivingFloor:.3f} for its pitch cone angle '
                f'{math.degrees(gammaG):.3f} deg')

        # 5. Base heights. The driving gear first, then the pinion scaled off
        #    the RESOLVED driving value and passed through the pinion's own
        #    bounds.
        drivingPitchRadius_cm = to_cm(drivingPitchDia_mm) / 2
        pinionPitchRadius_cm = to_cm(pinionPitchDia_mm) / 2
        self._drivingBaseHeightResolved_cm = self._resolveBaseHeight(
            'Driving Gear', self._drivingBaseHeight_cm,
            to_cm(module * drivingTeeth / 8),
            module, drivingPitchRadius_cm, gammaG)
        self._pinionBaseHeightResolved_cm = self._resolveBaseHeight(
            'Pinion Gear', self._pinionBaseHeight_cm,
            self._drivingBaseHeightResolved_cm * (pinionTeeth / drivingTeeth),
            module, pinionPitchRadius_cm, gammaP)

        # The bore bound is deliberately NOT part of this pass: its toe term
        # needs the Root Length, hence solved §2 geometry. It resolves in S07.

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    @staticmethod
    def _resolveBaseHeight(label, userValue_cm, fallback_cm, module,
                           pitchRadius_cm, gamma):
        dedendum_cm = to_cm(1.25 * module)
        minimum = 1.05 * dedendum_cm * math.sin(gamma)
        maximum = 0.95 * (pitchRadius_cm - dedendum_cm * math.cos(gamma)) * math.tan(gamma)
        if minimum > maximum:
            raise Exception(
                f'{label} base-height window is empty: minimum '
                f'{minimum * 10:.4f} mm is above maximum {maximum * 10:.4f} mm')
        if userValue_cm > 0:
            if userValue_cm < minimum:
                raise Exception(
                    f'{label} Base Height {userValue_cm * 10:.4f} mm is below the '
                    f'Minimum Base Height {minimum * 10:.4f} mm')
            if userValue_cm > maximum:
                raise Exception(
                    f'{label} Base Height {userValue_cm * 10:.4f} mm is above the '
                    f'Maximum Base Height {maximum * 10:.4f} mm')
            return userValue_cm
        # Raise a fallback that falls below the minimum, cap one above the
        # maximum.
        return max(minimum, min(fallback_cm, maximum))

    # ----------------------------------------------------------------------
    # Orchestration.
    # ----------------------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth,
         pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        self._module = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_rad = math.radians(shaftAngle_deg)

        self._buildComponentTree(parentComponent)
        anchorLine = self._createAnchorSketch(targetPlane, centerPoint)
        self._gearProfilesPlane = self._createGearProfilesPlane(
            anchorLine, targetPlane)
        pinionCtx, drivingCtx = self._createGearProfiles(
            anchorLine, targetPlane)

        for ctx in (pinionCtx, drivingCtx):
            self._createToothPlane(ctx)
            self._createToothSketch(ctx)
            self._createToothAxis(ctx)
            self._createGearComponent(ctx)
            self._createProfileSketch(ctx)
            self._createGearBody(ctx)
            self._cutBore(ctx)
            self._applyMeshRotation(ctx)
            self._moveBodyToGearComponent(ctx)

        self._cleanup()

    # ----------------------------------------------------------------------
    # S04: the component tree. Bevel does not subclass base.Generator and does
    # not use getOccurrence ([PB-OCCURRENCE-TREE]). occurrence.activate() is
    # NEVER called here ([PB-NEVER-ACTIVATE], [BEVEL-F-NEVER-ACTIVATE]); the
    # sole exception in this module is S20's crown scale.
    # ----------------------------------------------------------------------
    def _buildComponentTree(self, parentComponent: adsk.fusion.Component):
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelComponent = self.bevelOccurrence.component
        self.bevelComponent.name = 'Bevel Gear'

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designComponent = self.designOccurrence.component
        self.designComponent.name = 'Design'

    # ----------------------------------------------------------------------
    # S05: the Anchor sketch, directly on the user-selected target plane
    # ([PB-USE-SELECTED-PLANE]).
    # ----------------------------------------------------------------------
    def _createAnchorSketch(self, targetPlane,
                            centerPoint) -> adsk.fusion.SketchLine:
        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint).item(0)
        self._anchorCenterPoint = projectedCenter

        c = projectedCenter.geometry
        anchorLine: adsk.fusion.SketchLine = \
            sketch.sketchCurves.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create(c.x - 0.5, c.y, 0),
                adsk.core.Point3D.create(c.x + 0.5, c.y, 0))

        # Both: the coincident pins the center onto the line, the midpoint
        # makes the center bisect it. Never midpoint alone.
        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        # An aligned dimension locking the seeded 10 mm. The value is
        # arbitrary, so .parameter.value is not assigned.
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + 0.25, 0))

        # Sketch-local direction lock ([PB-REFLINE-DIRECTION]), so the sketch
        # ends with zero DOF on any tilted target plane.
        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception(
                'Bevel Gear: the "Anchor" sketch is not fully constrained')

        return anchorLine

    # ----------------------------------------------------------------------
    # S06: the Gear Profiles Plane, 90 deg off the ORIGINAL target plane
    # through the Anchor Line. The SketchLine goes in directly; never wrapped
    # in Path.create ([PB-CONSTRUCTION-PLANES]).
    # ----------------------------------------------------------------------
    def _createGearProfilesPlane(self, anchorLine,
                                 targetPlane) -> adsk.fusion.ConstructionPlane:
        planes: adsk.fusion.ConstructionPlanes = \
            self.designComponent.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'),
            targetPlane)
        plane: adsk.fusion.ConstructionPlane = planes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        return plane

    # ----------------------------------------------------------------------
    # S07: the Gear Profiles sketch, the whole §2 lattice.
    # ----------------------------------------------------------------------
    def _createGearProfiles(self, anchorLine, targetPlane):
        module = self._module
        Nd, Np = self._drivingTeeth, self._pinionTeeth
        sigma = self._shaftAngle_rad
        gammaP, gammaG = self._gamma_p, self._gamma_g

        DPD = to_cm(module * Nd)
        PPD = to_cm(module * Np)
        dedendum = to_cm(1.25 * module)
        rp, rg = PPD / 2.0, DPD / 2.0
        R = (PPD / 2.0) / math.sin(gammaP)
        coneDist = math.hypot(DPD, PPD)
        apexDed = math.hypot(R, dedendum)
        gammaRootP = gammaP - math.atan(dedendum / R)
        gammaRootG = gammaG - math.atan(dedendum / R)
        BHd = self._drivingBaseHeightResolved_cm
        BHp = self._pinionBaseHeightResolved_cm
        toothSpacing = self._toothSpacing_cm

        sketch: adsk.fusion.Sketch = \
            self.designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        lines: adsk.fusion.SketchLines = sketch.sketchCurves.sketchLines
        cons: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions

        # --- Frame --------------------------------------------------------
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchorLine = sketch.project(anchorLine).item(0)

        cg = projectedCenter.geometry
        c = (cg.x, cg.y)
        ag = projectedAnchorLine.geometry
        d = _unit2((ag.endPoint.x - ag.startPoint.x,
                    ag.endPoint.y - ag.startPoint.y))
        perp = _leftperp2(d)

        # [BEVEL-F-GROW-SIDE]: the grow side is chosen by the target-plane
        # normal, never by the sketch's local +Y. Only the sign of the
        # comparison is used - a one-bit read of a direction, not a position.
        normal = targetPlane.geometry.normal
        xDir, yDir = sketch.xDirection, sketch.yDirection
        worldPerp: adsk.core.Vector3D = adsk.core.Vector3D.create(
            xDir.x * perp[0] + yDir.x * perp[1],
            xDir.y * perp[0] + yDir.y * perp[1],
            xDir.z * perp[0] + yDir.z * perp[1])
        if worldPerp.dotProduct(normal) < 0:
            perp = _mul2(perp, -1.0)

        # --- Closed-form seeds, the whole figure before anything is drawn ---
        apexSeed = _add2(c, _mul2(perp, R * math.cos(gammaG) + BHd))
        drivingDir = _mul2(perp, -1.0)
        bSeed = _add2(apexSeed, _mul2(drivingDir, R * math.cos(gammaG)))

        # Form BOTH candidate A positions and keep the greater-X one; rotating
        # one fixed sense and flipping only on a negative X keeps the wrong
        # candidate whenever both come out positive.
        plusDir = _rot2(drivingDir, sigma)
        minusDir = _rot2(drivingDir, -sigma)
        aPlus = _add2(apexSeed, _mul2(plusDir, R * math.cos(gammaP)))
        aMinus = _add2(apexSeed, _mul2(minusDir, R * math.cos(gammaP)))
        if aMinus[0] > aPlus[0]:
            pinionDir, aSeed = minusDir, aMinus
        else:
            pinionDir, aSeed = plusDir, aPlus

        # The PPD/2 drop from A points at the OTHER shaft axis, so its sense is
        # picked by the dot product with A->B.
        dropDirP = _leftperp2(pinionDir)
        if _dot2(dropDirP, _sub2(bSeed, aSeed)) < 0:
            dropDirP = _mul2(dropDirP, -1.0)
        apex2Seed = _add2(aSeed, _mul2(dropDirP, rp))

        # The DPD/2 drop from B must aim at the same interior point. Its sense
        # is picked against B->A, never against the grow direction, which the
        # driving shaft axis is parallel to.
        dropDirG = _leftperp2(drivingDir)
        if _dot2(dropDirG, _sub2(aSeed, bSeed)) < 0:
            dropDirG = _mul2(dropDirG, -1.0)

        pitchDir = _unit2(_sub2(apex2Seed, apexSeed))

        # The dedendum directions, seeded by dot product against the shaft
        # axes: those dot products are exactly sin(gamma_p) and sin(gamma_g).
        dedDirP = _leftperp2(pitchDir)
        if _dot2(dedDirP, pinionDir) < 0:
            dedDirP = _mul2(dedDirP, -1.0)
        dedDirG = _mul2(dedDirP, -1.0)

        cSeed = _add2(apex2Seed, _mul2(dedDirP, dedendum))
        dSeed = _add2(apex2Seed, _mul2(dedDirG, dedendum))
        eSeed = _add2(aSeed, _mul2(pinionDir, dedendum * math.sin(gammaP)))
        fSeed = _add2(bSeed, _mul2(drivingDir, dedendum * math.sin(gammaG)))
        gSeed = _add2(aSeed, _mul2(pinionDir, BHp))
        hSeed = _add2(apex2Seed, _mul2(dedDirP, BHp / math.sin(gammaP)))
        iSeed = _add2(bSeed, _mul2(drivingDir, BHd))
        jSeed = _add2(apex2Seed, _mul2(dedDirG, BHd / math.sin(gammaG)))

        # The exact back-cone (Tredgold) radii. |Apex2 -> K| is r / cos(gamma),
        # never a radius rebuilt from a rounded tooth count.
        virtualRadiusP = rp / math.cos(gammaP)
        virtualRadiusG = rg / math.cos(gammaG)
        kSeed = _add2(apex2Seed, _mul2(dedDirP, virtualRadiusP))
        lSeed = _add2(apex2Seed, _mul2(dedDirG, virtualRadiusG))
        kpSeed = _add2(apex2Seed,
                       _mul2(dedDirP, virtualRadiusP + toothSpacing))
        lpSeed = _add2(apex2Seed,
                       _mul2(dedDirG, virtualRadiusG + toothSpacing))

        # --- The lattice, in creation order -------------------------------

        # 1. centerToApex.
        centerToApex = _construction_line(lines, c, apexSeed)
        cons.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        cons.addPerpendicular(centerToApex, projectedAnchorLine)
        pointApex = centerToApex.endSketchPoint

        # 2. Driving Gear Shaft Axis, Apex -> B.
        drivingAxis = _construction_line(lines, apexSeed, bSeed)
        cons.addCoincident(drivingAxis.startSketchPoint, pointApex)
        cons.addParallel(drivingAxis, centerToApex)
        pointB = drivingAxis.endSketchPoint

        # 3. Pinion Gear Shaft Axis, Apex -> A.
        pinionAxis = _construction_line(lines, apexSeed, aSeed)
        cons.addCoincident(pinionAxis.startSketchPoint, pointApex)
        pointA = pinionAxis.endSketchPoint

        # 4. The Shaft Angle, with its text point inside the sigma wedge so it
        #    measures sigma and not 180 - sigma ([PB-ANGULAR-DIM]).
        bisector = _unit2(_add2(pinionDir, drivingDir))
        angleDim = dims.addAngularDimension(
            pinionAxis, drivingAxis,
            _pt2(_add2(apexSeed, _mul2(bisector, PPD / 4.0))))
        angleDim.parameter.value = sigma

        # 5. A->Apex2, the PPD/2 drop.
        dropP = _construction_line(lines, aSeed, apex2Seed)
        cons.addCoincident(dropP.startSketchPoint, pointA)
        cons.addPerpendicular(dropP, pinionAxis)
        _aligned_length(dims, dropP, rp, aSeed, apex2Seed)

        # 6. B->Apex2, the DPD/2 drop.
        dropG = _construction_line(lines, bSeed, apex2Seed)
        cons.addCoincident(dropG.startSketchPoint, pointB)
        cons.addPerpendicular(dropG, drivingAxis)
        _aligned_length(dims, dropG, rg, bSeed, apex2Seed)

        # 7. Apex 2 closes the two drops.
        pointApex2 = dropP.endSketchPoint
        cons.addCoincident(dropG.endSketchPoint, pointApex2)

        # 8. The Pitch Line.
        pitchLine = _construction_line(lines, apexSeed, apex2Seed)
        cons.addCoincident(pitchLine.startSketchPoint, pointApex)
        cons.addCoincident(pitchLine.endSketchPoint, pointApex2)

        # 9. The two dedendum lines out of Apex 2.
        dedLineP = _construction_line(lines, apex2Seed, cSeed)
        cons.addCoincident(dedLineP.startSketchPoint, pointApex2)
        cons.addPerpendicular(dedLineP, pitchLine)
        _aligned_length(dims, dedLineP, dedendum, apex2Seed, cSeed)
        pointC = dedLineP.endSketchPoint

        dedLineG = _construction_line(lines, apex2Seed, dSeed)
        cons.addCoincident(dedLineG.startSketchPoint, pointApex2)
        cons.addPerpendicular(dedLineG, pitchLine)
        _aligned_length(dims, dedLineG, dedendum, apex2Seed, dSeed)
        pointD = dedLineG.endSketchPoint

        # 10. The two Root Axes. They stay inside §2.
        drivingRootAxis = _construction_line(lines, apexSeed, dSeed)
        cons.addCoincident(drivingRootAxis.startSketchPoint, pointApex)
        cons.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = _construction_line(lines, apexSeed, cSeed)
        cons.addCoincident(pinionRootAxis.startSketchPoint, pointApex)
        cons.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # 11. A->E, collinear with Apex->A.
        lineAE = _construction_line(lines, aSeed, eSeed)
        cons.addCoincident(lineAE.startSketchPoint, pointA)
        cons.addCollinear(lineAE, pinionAxis)
        pointE = lineAE.endSketchPoint

        # 12. C->E.
        lineCE = _construction_line(lines, cSeed, eSeed)
        cons.addCoincident(lineCE.startSketchPoint, pointC)
        cons.addCoincident(lineCE.endSketchPoint, pointE)
        cons.addPerpendicular(lineAE, lineCE)

        # 13. B->F, the driving twin of A->E.
        lineBF = _construction_line(lines, bSeed, fSeed)
        cons.addCoincident(lineBF.startSketchPoint, pointB)
        cons.addCollinear(lineBF, drivingAxis)
        pointF = lineBF.endSketchPoint

        # 14. D->F.
        lineDF = _construction_line(lines, dSeed, fSeed)
        cons.addCoincident(lineDF.startSketchPoint, pointD)
        cons.addCoincident(lineDF.endSketchPoint, pointF)
        cons.addPerpendicular(lineBF, lineDF)

        # 15. E->G, collinear with line A->E - never the Apex->A axis further
        #     up the chain ([BEVEL-F-COLLINEAR-CHAIN], [PB-COLLINEAR-CHAIN]).
        lineEG = _construction_line(lines, eSeed, gSeed)
        cons.addCoincident(lineEG.startSketchPoint, pointE)
        cons.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # 16. C->H, collinear with the dedendum line Apex2->C.
        lineCH = _construction_line(lines, cSeed, hSeed)
        cons.addCoincident(lineCH.startSketchPoint, pointC)
        cons.addCollinear(lineCH, dedLineP)
        pointH = lineCH.endSketchPoint

        # 17. G->H, with the perpendicular Fusion's offset dimension needs to
        #     have a parallel pair to measure across.
        lineGH = _construction_line(lines, gSeed, hSeed)
        cons.addCoincident(lineGH.startSketchPoint, pointG)
        cons.addCoincident(lineGH.endSketchPoint, pointH)
        cons.addPerpendicular(lineEG, lineGH)

        # 18. F->I, collinear with line B->F.
        lineFI = _construction_line(lines, fSeed, iSeed)
        cons.addCoincident(lineFI.startSketchPoint, pointF)
        cons.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # 19. D->J, collinear with the dedendum line Apex2->D.
        lineDJ = _construction_line(lines, dSeed, jSeed)
        cons.addCoincident(lineDJ.startSketchPoint, pointD)
        cons.addCollinear(lineDJ, dedLineG)
        pointJ = lineDJ.endSketchPoint

        # 20. I->J, the driving-side twin of 17.
        lineIJ = _construction_line(lines, iSeed, jSeed)
        cons.addCoincident(lineIJ.startSketchPoint, pointI)
        cons.addCoincident(lineIJ.endSketchPoint, pointJ)
        cons.addPerpendicular(lineFI, lineIJ)

        # 21. The driving base-height offset, between the DPD/2 drop and I->J.
        #     Already parallel by construction, so no addParallel
        #     ([PB-OFFSET-DIM]).
        drivingOffset = dims.addOffsetDimension(
            dropG, lineIJ,
            _pt2(_mul2(_add2(_mul2(_add2(bSeed, apex2Seed), 0.5),
                             _mul2(_add2(iSeed, jSeed), 0.5)), 0.5)))
        drivingOffset.parameter.value = BHd

        # 22. The pinion base-height offset, between the PPD/2 drop and G->H.
        pinionOffset = dims.addOffsetDimension(
            dropP, lineGH,
            _pt2(_mul2(_add2(_mul2(_add2(aSeed, apex2Seed), 0.5),
                             _mul2(_add2(gSeed, hSeed), 0.5)), 0.5)))
        pinionOffset.parameter.value = BHp

        # --- 27 / 28, resolved here because step 23's A' seed needs N -------
        # The Maximum Face Width comes off the SOLVED positions of A, B, C, D,
        # H and J ([PB-SOLVED-GEOMETRY]), all of which are located by now.
        solvedA = self._solved2(pointA)
        solvedB = self._solved2(pointB)
        solvedC = self._solved2(pointC)
        solvedD = self._solved2(pointD)
        solvedH = self._solved2(pointH)
        solvedJ = self._solved2(pointJ)
        maxFaceWidth = 0.95 * min(
            _dist_point_line2(solvedA, solvedC, _sub2(solvedH, solvedC)),
            _dist_point_line2(solvedB, solvedD, _sub2(solvedJ, solvedD)))

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth:
                raise Exception(
                    f'Face Width {self._faceWidth_cm * 10:.4f} mm exceeds the '
                    f'Maximum Face Width {maxFaceWidth * 10:.4f} mm')
            faceWidth = self._faceWidth_cm
        else:
            faceWidth = min(coneDist / 6.0, maxFaceWidth)

        rootLen0 = faceWidth * apexDed / R
        toeCeilingP = (rp - dedendum * math.cos(gammaP)) * (1 - faceWidth / R)
        toeCeilingG = (rg - dedendum * math.cos(gammaG)) * (1 - faceWidth / R)
        toeRadiusP = self._resolveToeRadius(
            'Pinion Gear', self._pinionToeRadius_cm,
            rp - faceWidth / math.sin(gammaP), toeCeilingP)
        toeRadiusG = self._resolveToeRadius(
            'Driving Gear', self._drivingToeRadius_cm,
            rg - faceWidth / math.sin(gammaG), toeCeilingG)

        toeLimitP = apexDed - toeRadiusP / math.sin(gammaRootP)
        toeLimitG = apexDed - toeRadiusG / math.sin(gammaRootG)
        toeLimit = min(toeLimitP, toeLimitG)
        if self._toeExtension_pct > 0 and toeLimit <= rootLen0:
            binding = 'Pinion Gear' if toeLimitP <= toeLimitG else 'Driving Gear'
            ceiling = toeCeilingP if toeLimitP <= toeLimitG else toeCeilingG
            raise Exception(
                f'Toe Extension {self._toeExtension_pct} rejected: the {binding} '
                f'Toe Limit {toeLimit * 10:.4f} mm is already at or below the Toe '
                f'Extension 0 root length {rootLen0 * 10:.4f} mm. Its Toe Radius '
                f'must come below the Toe Radius Ceiling {ceiling * 10:.4f} mm.')
        # Toe Extension 100 deliberately stops at 0.99 of the way to the
        # smaller Toe Limit, so the toe face keeps a cone for S22 to find.
        rootLen = rootLen0 + (self._toeExtension_pct / 100.0) * 0.99 * (
            toeLimit - rootLen0)

        boreP = 0.0
        boreG = 0.0
        if self._boreEnable:
            maxBoreP = self._maxBore(rp, BHp, gammaP, apexDed, rootLen, gammaRootP)
            maxBoreG = self._maxBore(rg, BHd, gammaG, apexDed, rootLen, gammaRootG)
            boreP = self._resolveBore(
                'Pinion Gear', self._pinionBore_cm, PPD / 4.0, maxBoreP)
            boreG = self._resolveBore(
                'Driving Gear', self._drivingBore_cm, DPD / 4.0, maxBoreG)

        # The toe seeds, now that the Root Length and both Toe Radii are known.
        apexToC = _sub2(cSeed, apexSeed)
        mSeed = _add2(apexSeed, _mul2(apexToC, 1 - rootLen / _len2(apexToC)))
        mRadius = _dist_point_line2(mSeed, apexSeed, pinionDir)
        nSeed = _add2(mSeed, _mul2(
            dedDirP, (mRadius - toeRadiusP) / math.cos(gammaP)))
        apSeed = _add2(apexSeed,
                       _mul2(pinionDir, _dot2(_sub2(nSeed, apexSeed), pinionDir)))

        apexToD = _sub2(dSeed, apexSeed)
        oSeed = _add2(apexSeed, _mul2(apexToD, 1 - rootLen / _len2(apexToD)))
        oRadius = _dist_point_line2(oSeed, apexSeed, drivingDir)
        pSeed = _add2(oSeed, _mul2(
            dedDirG, (oRadius - toeRadiusG) / math.cos(gammaG)))
        bpSeed = _add2(apexSeed,
                       _mul2(drivingDir, _dot2(_sub2(pSeed, apexSeed), drivingDir)))

        # 23. A'->G, the pinion hexagon's shaft-axis edge. This line is what
        #     creates A'; nothing above it does.
        lineApG = _construction_line(lines, apSeed, gSeed)
        cons.addCoincident(lineApG.endSketchPoint, pointG)
        pointAp = lineApG.startSketchPoint

        # 24. Hang the whole lattice off the anchor.
        cons.addCoincident(pointI, projectedCenter)

        # 25. G->K, with K pinned by two point-on-line coincidents rather than
        #     a collinear, which would over-constrain by now.
        lineGK = _construction_line(lines, gSeed, kSeed)
        cons.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        cons.addCoincident(pointK, pinionAxis)
        cons.addCoincident(pointK, dedLineP)

        lineCK = _construction_line(lines, cSeed, kSeed)
        cons.addCoincident(lineCK.startSketchPoint, pointC)
        cons.addCoincident(lineCK.endSketchPoint, pointK)

        # 26. The pinion tooth centre K'. At Tooth Spacing 0 it IS K and
        #     nothing is built ([BEVEL-F-LINE-ONCE]).
        if toothSpacing > 0:
            lineKKp = _construction_line(lines, kSeed, kpSeed)
            cons.addCoincident(lineKKp.startSketchPoint, pointK)
            pointKp = lineKKp.endSketchPoint
            cons.addCoincident(pointKp, dedLineP)
            _aligned_length(dims, lineKKp, toothSpacing, kSeed, kpSeed)

            lineCKp = _construction_line(lines, cSeed, kpSeed)
            cons.addCoincident(lineCKp.startSketchPoint, pointC)
            cons.addCoincident(lineCKp.endSketchPoint, pointKp)
            pinionToothCenter, pinionToothRefLine = pointKp, lineCKp
        else:
            pointKp = None
            pinionToothCenter, pinionToothRefLine = pointK, lineCK

        # 29. M->N, the pinion toe line.
        lineMN = _construction_line(lines, mSeed, nSeed)
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        cons.addCoincident(pointM, pinionRootAxis)
        cons.addParallel(lineMN, lineCH)
        pinionToeOffset = dims.addOffsetDimension(
            lineCH, lineMN, _pt2(_mul2(_add2(mSeed, cSeed), 0.5)))
        pinionToeOffset.parameter.value = rootLen * R / _len2(apexToC)

        lineMC = _construction_line(lines, mSeed, cSeed)
        cons.addCoincident(lineMC.startSketchPoint, pointM)
        cons.addCoincident(lineMC.endSketchPoint, pointC)

        # 30. The front face N->A'. N rides the resolved Toe Radius and is
        #     never pinned to the Apex->A shaft axis; only A' touches it.
        lineNAp = _construction_line(lines, nSeed, apSeed)
        cons.addCoincident(lineNAp.startSketchPoint, pointN)
        cons.addCoincident(lineNAp.endSketchPoint, pointAp)
        cons.addCoincident(pointAp, pinionAxis)
        cons.addPerpendicular(lineNAp, pinionAxis)
        _aligned_length(dims, lineNAp, toeRadiusP, nSeed, apSeed)

        # 31. I->L, the driving twin of G->K.
        lineIL = _construction_line(lines, iSeed, lSeed)
        cons.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        cons.addCoincident(pointL, drivingAxis)
        cons.addCoincident(pointL, dedLineG)

        lineDL = _construction_line(lines, dSeed, lSeed)
        cons.addCoincident(lineDL.startSketchPoint, pointD)
        cons.addCoincident(lineDL.endSketchPoint, pointL)

        # 32. The driving tooth centre L'.
        if toothSpacing > 0:
            lineLLp = _construction_line(lines, lSeed, lpSeed)
            cons.addCoincident(lineLLp.startSketchPoint, pointL)
            pointLp = lineLLp.endSketchPoint
            cons.addCoincident(pointLp, dedLineG)
            _aligned_length(dims, lineLLp, toothSpacing, lSeed, lpSeed)

            lineDLp = _construction_line(lines, dSeed, lpSeed)
            cons.addCoincident(lineDLp.startSketchPoint, pointD)
            cons.addCoincident(lineDLp.endSketchPoint, pointLp)
            drivingToothCenter, drivingToothRefLine = pointLp, lineDLp
        else:
            pointLp = None
            drivingToothCenter, drivingToothRefLine = pointL, lineDL

        # 33. O->P, the driving toe line.
        lineOP = _construction_line(lines, oSeed, pSeed)
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        cons.addCoincident(pointO, drivingRootAxis)
        cons.addParallel(lineOP, lineDJ)
        drivingToeOffset = dims.addOffsetDimension(
            lineDJ, lineOP, _pt2(_mul2(_add2(oSeed, dSeed), 0.5)))
        drivingToeOffset.parameter.value = rootLen * R / _len2(apexToD)

        lineOD = _construction_line(lines, oSeed, dSeed)
        cons.addCoincident(lineOD.startSketchPoint, pointO)
        cons.addCoincident(lineOD.endSketchPoint, pointD)

        # 34. The driving front face P->B', then B'->I.
        linePBp = _construction_line(lines, pSeed, bpSeed)
        pointBp = linePBp.endSketchPoint
        cons.addCoincident(linePBp.startSketchPoint, pointP)
        cons.addCoincident(pointBp, drivingAxis)
        cons.addPerpendicular(linePBp, drivingAxis)
        _aligned_length(dims, linePBp, toeRadiusG, pSeed, bpSeed)

        lineBpI = _construction_line(lines, bpSeed, iSeed)
        cons.addCoincident(lineBpI.startSketchPoint, pointBp)
        cons.addCoincident(lineBpI.endSketchPoint, pointI)

        # --- End of step ---------------------------------------------------
        if not sketch.isFullyConstrained:
            raise Exception(
                'Bevel Gear: the "Gear Profiles" sketch is not fully constrained')

        # [BEVEL-F-SEED-HELD]: compare every named point's solved position
        # against the closed form this step seeded it at, in the sketch's own
        # 2-D frame, and raise naming the FIRST point that moved. This is the
        # only measure that catches the mirrored figures; the revolve's
        # ASM_WIRE_X_AXIS is not a tripwire, because several flips build a
        # valid-looking gear on the wrong side.
        checks = [
            ('Apex', pointApex, apexSeed),
            ('B', pointB, bSeed),
            ('A', pointA, aSeed),
            ('Apex 2', pointApex2, apex2Seed),
            ('C', pointC, cSeed),
            ('D', pointD, dSeed),
            ('E', pointE, eSeed),
            ('F', pointF, fSeed),
            ('G', pointG, gSeed),
            ('H', pointH, hSeed),
            ('I', pointI, iSeed),
            ('J', pointJ, jSeed),
            ('K', pointK, kSeed),
        ]
        if pointKp is not None:
            checks.append(("K'", pointKp, kpSeed))
        checks.extend([
            ('M', pointM, mSeed),
            ('N', pointN, nSeed),
            ("A'", pointAp, apSeed),
            ('L', pointL, lSeed),
        ])
        if pointLp is not None:
            checks.append(("L'", pointLp, lpSeed))
        checks.extend([
            ('O', pointO, oSeed),
            ('P', pointP, pSeed),
            ("B'", pointBp, bpSeed),
        ])
        self._gateSeedsHeld(checks)

        self._apexSketchPoint = pointApex

        pinionCtx = {
            'label': 'Pinion',
            'teeth': Np,
            'gamma': gammaP,
            'pitchDiameter_cm': PPD,
            'toothCenterPoint': pinionToothCenter,
            'toothCenterRefLine': pinionToothRefLine,
            'hexVertices': [pointAp, pointG, pointH, pointC, pointM, pointN],
            'toeEdgePoints': (pointM, pointN),
            'heelEdgePoints': (pointC, pointH),
            'boreDiameter_cm': boreP,
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': Nd,
            'gamma': gammaG,
            'pitchDiameter_cm': DPD,
            'toothCenterPoint': drivingToothCenter,
            'toothCenterRefLine': drivingToothRefLine,
            'hexVertices': [pointBp, pointI, pointJ, pointD, pointO, pointP],
            'toeEdgePoints': (pointO, pointP),
            'heelEdgePoints': (pointD, pointJ),
            'boreDiameter_cm': boreG,
        }
        return pinionCtx, drivingCtx

    @staticmethod
    def _solved2(sketchPoint: adsk.fusion.SketchPoint):
        g = sketchPoint.geometry
        return (g.x, g.y)

    @staticmethod
    def _gateSeedsHeld(checks, tolerance_cm=1e-4):
        for (name, sketchPoint, seed) in checks:
            solved = sketchPoint.geometry
            moved = math.hypot(solved.x - seed[0], solved.y - seed[1])
            if moved > tolerance_cm:
                raise Exception(
                    f'Bevel Gear: §2 point {name} solved at '
                    f'({solved.x * 10:.5f}, {solved.y * 10:.5f}) mm but was seeded at '
                    f'({seed[0] * 10:.5f}, {seed[1] * 10:.5f}) mm, '
                    f'{moved * 10:.5f} mm away (tolerance '
                    f'{tolerance_cm * 10:.5f} mm). The figure solved to a mirror of '
                    f'the intended one.')

    @staticmethod
    def _resolveToeRadius(label, userValue_cm, auto_cm, ceiling_cm):
        if userValue_cm > 0:
            if userValue_cm >= ceiling_cm:
                raise Exception(
                    f'{label} Toe Radius {userValue_cm * 10:.4f} mm must be '
                    f'strictly below its Toe Radius Ceiling '
                    f'{ceiling_cm * 10:.4f} mm')
            return userValue_cm
        return auto_cm

    @staticmethod
    def _maxBore(pitchRadius_cm, baseHeight_cm, gamma, apexDed_cm, rootLen_cm,
                 gammaRoot):
        rHeel = pitchRadius_cm - baseHeight_cm / math.tan(gamma)
        rToe = (apexDed_cm - rootLen_cm) * math.sin(gammaRoot)
        return 2 * 0.95 * min(rHeel, rToe)

    @staticmethod
    def _resolveBore(label, userValue_cm, auto_cm, max_cm):
        if userValue_cm > 0:
            if userValue_cm > max_cm:
                raise Exception(
                    f'{label} Bore Diameter {userValue_cm * 10:.4f} mm exceeds '
                    f'the Maximum Bore Diameter {max_cm * 10:.4f} mm')
            return userValue_cm
        return min(auto_cm, max_cm)

    # ----------------------------------------------------------------------
    # S08: the tooth plane, through the tooth-centre reference line.
    # ----------------------------------------------------------------------
    def _createToothPlane(self, ctx):
        plane: adsk.fusion.ConstructionPlane = solids.plane_by_angle(
            self.designComponent, ctx['toothCenterRefLine'],
            self._gearProfilesPlane, 90)
        plane.name = f"{ctx['label']} Plane"
        ctx['toothPlane'] = plane

    # ----------------------------------------------------------------------
    # S09: the virtual spur tooth on the back-cone plane.
    # ----------------------------------------------------------------------
    def _createToothSketch(self, ctx):
        module = self._module
        gamma = ctx['gamma']

        # The stashed pitch diameters are internal cm and Module is raw mm, so
        # the cm -> mm conversion is pinned here. Skipping the x10 makes the
        # virtual tooth count about ten times off.
        virtualPitchRadius_mm = (ctx['pitchDiameter_cm'] * 10 / 2) / math.cos(gamma)
        # NEVER rounded, floored, ceiled or cast to an int.
        virtualTeeth = 2 * virtualPitchRadius_mm / module
        rootSink_mm = 0.05 * 2.25 * module

        sketch: adsk.fusion.Sketch = \
            self.designComponent.sketches.add(ctx['toothPlane'])
        sketch.name = f"{ctx['label']} Tooth"

        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth,
                                 rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        # The 180 degree tooth rotation IS the draw() angle - never a post-hoc
        # Move or sketch rotation.
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))

        # The spur generator decides during draw() whether the tooth is
        # embedded and records it on the proxy. It is the deterministic
        # selector for S14's line count, not optional bookkeeping.
        ctx['toothEmbedded'] = proxy._lastToothEmbedded
        ctx['toothSketch'] = sketch
        ctx['virtualTeeth'] = virtualTeeth

        # The tooth-profile sketches are exempt from the full-constraint gate
        # ([BEVEL-F-FULL-CONSTRAINT], [PB-TEXT-HOLDS-DOF]): log, never raise.
        if not sketch.isFullyConstrained:
            futil.log(
                f'{ctx["label"]} Tooth sketch is not fully constrained '
                f'(expected: the drawer\'s four circle labels hold a DOF)')

    # ----------------------------------------------------------------------
    # S10: the tooth axis, through the tooth centre normal to the tooth plane.
    # ----------------------------------------------------------------------
    def _createToothAxis(self, ctx):
        planes: adsk.fusion.ConstructionPlanes = \
            self.designComponent.constructionPlanes
        helperInput = planes.createInput()
        helperInput.setByDistanceOnPath(
            ctx['toothCenterRefLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane: adsk.fusion.ConstructionPlane = planes.add(helperInput)

        axes: adsk.fusion.ConstructionAxes = \
            self.designComponent.constructionAxes
        axisInput = axes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        axis: adsk.fusion.ConstructionAxis = axes.add(axisInput)
        axis.name = f"{ctx['label']} Tooth Axis"
        # No step reads this key back; Cleanup hides the axis by entity kind.
        ctx['toothAxis'] = axis

    # ----------------------------------------------------------------------
    # S11: the per-gear component, a child of Bevel Gear.
    # ----------------------------------------------------------------------
    def _createGearComponent(self, ctx):
        occurrence: adsk.fusion.Occurrence = \
            self.bevelComponent.occurrences.addNewComponent(
                adsk.core.Matrix3D.create())
        occurrence.component.name = f"{ctx['label']} Gear"
        ctx['gearOccurrence'] = occurrence

    # ----------------------------------------------------------------------
    # S12: the per-gear hexagon profile sketch, on fixed recreated vertices
    # ([PB-PROJECT-NOT-FIXED]).
    # ----------------------------------------------------------------------
    def _createProfileSketch(self, ctx):
        sketch: adsk.fusion.Sketch = \
            self.designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = f"{ctx['label']} Profile"

        verts = [sketch.sketchPoints.add(
            sketch.modelToSketchSpace(src.worldGeometry))
            for src in ctx['hexVertices']]

        edges = []
        for i in range(len(verts)):
            edges.append(sketch.sketchCurves.sketchLines.addByTwoPoints(
                verts[i], verts[(i + 1) % len(verts)]))

        # Fix the endpoints only once the lines exist: setting isFixed on a
        # bare point beforehand does not leave the sketch fully constrained.
        for edge in edges:
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(
                f'Bevel Gear: the "{ctx["label"]} Profile" sketch is not fully '
                f'constrained')

        ctx['profileSketch'] = sketch
        # The hexagon's FIRST edge is the gear's shaft axis for every body
        # operation - never the §2 Apex->A / Apex->B construction line.
        ctx['shaftAxisEdge'] = edges[0]

    # ----------------------------------------------------------------------
    # S13, S14 and S22 through S24: the gear body.
    # ----------------------------------------------------------------------
    def _createGearBody(self, ctx):
        component: adsk.fusion.Component = self.designComponent
        features: adsk.fusion.Features = component.features
        label = ctx['label']
        shaftAxisEdge: adsk.fusion.SketchLine = ctx['shaftAxisEdge']
        profileSketch: adsk.fusion.Sketch = ctx['profileSketch']
        toothSketch: adsk.fusion.Sketch = ctx['toothSketch']

        # S13. This sketch holds exactly one hexagon loop, so take its single
        # profile and do not filter ([PB-SINGLE-PROFILE]).
        profile = profileSketch.profiles.item(0)
        revolveInput = features.revolveFeatures.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = features.revolveFeatures.add(revolveInput)
        gearBody: adsk.fusion.BRepBody = revolve.bodies.item(0)
        ctx['gearBody'] = gearBody

        # S14. Loft the §2 Apex sketch point to this gear's tooth profile. The
        # line count is decided by the toothEmbedded flag, never guessed and
        # never accepted as "0 or 2".
        wantLines = 0 if ctx['toothEmbedded'] else 2
        toothProfile = find_profile_by_curve_counts(
            toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = features.loftFeatures.add(loftInput)
        toothBody: adsk.fusion.BRepBody = loft.bodies.item(0)

        # The hand-off the tooth-body hook takes. Mislabeling these silently
        # inverts the spiral.
        (toeStart, toeEnd) = ctx['toeEdgePoints']
        (heelCorner, heelOther) = ctx['heelEdgePoints']
        toeStartPoint: adsk.fusion.SketchPoint = toeStart
        toeEndPoint: adsk.fusion.SketchPoint = toeEnd
        heelCornerPoint: adsk.fusion.SketchPoint = heelCorner
        heelOtherPoint: adsk.fusion.SketchPoint = heelOther
        toeMid = _midpoint(toeStartPoint.worldGeometry, toeEndPoint.worldGeometry)
        heelMid = _midpoint(heelCornerPoint.worldGeometry,
                            heelOtherPoint.worldGeometry)
        # toeConeWorld is the toe edge's inner endpoint M / O, heelConeWorld is
        # the dedendum corner C / D - never H / J, which sit off the root cone
        # element.
        toeConeWorld = toeStartPoint.worldGeometry
        heelConeWorld = heelCornerPoint.worldGeometry
        apexWorld = self._apexSketchPoint.worldGeometry

        toothPiece = self._transformToothBody(
            component, toothBody, gearBody, shaftAxisEdge, apexWorld,
            self._apexSketchPoint, toeMid, heelMid, toeConeWorld,
            heelConeWorld, ctx['toothPlane'], label, ctx['teeth'],
            ctx['gamma'])

        # S23. Circular pattern around the shaft-axis edge, all three inputs
        # pinned explicitly ([PB-CIRCULAR-PATTERN]).
        seed = adsk.core.ObjectCollection.create()
        seed.add(toothPiece)
        patternInput = features.circularPatternFeatures.createInput(
            seed, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = features.circularPatternFeatures.add(patternInput)

        # pattern.bodies already holds the seed plus the copies, and it is a
        # BRepBodies the Combine rejects ([PB-PATTERN-BODIES]).
        tools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(i))

        # S24. One Combine-Join: the Gear Body is the target.
        combineInput = features.combineFeatures.createInput(gearBody, tools)
        combineInput.operation = \
            adsk.fusion.FeatureOperations.JoinFeatureOperation
        features.combineFeatures.add(combineInput)

    # ----------------------------------------------------------------------
    # S15 through S22: the tooth-body hook.
    # ----------------------------------------------------------------------
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
        # S17's gate: a straight bevel is byte-for-byte the prior behavior.
        if self._spiralAngle_rad <= 0:
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)

        features: adsk.fusion.Features = designComponent.features

        # --- S17 A. Gate and frame ----------------------------------------
        edgeStartW = shaftAxisEdge.startSketchPoint.worldGeometry
        edgeEndW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir: adsk.core.Vector3D = _unit_between(edgeStartW, edgeEndW)

        # The heel MUST be the OUTER end so coneVec points outward and span is
        # positive. A negative span silently inverts the entire spiral frame.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld
        coneVec: adsk.core.Vector3D = _unit_between(apexWorld, heelConeWorld)
        v: adsk.core.Vector3D = _unit_cross(axisDir, coneVec)
        # tpNormal completes the frame and nothing consumes it.
        tpNormal: adsk.core.Vector3D = _unit_cross(coneVec, v)

        def distAlong(point):
            return _along(apexWorld, point, coneVec)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- S15. The cone element sketch and the Trace Plane --------------
        # Both are exempt from the full-constraint gate, and both take their
        # raw model-space Point3Ds directly, with NO modelToSketchSpace
        # conversion: the chain ends at the inspection-only trace sketch. If a
        # later revision ever makes a feature consume either, this shortcut
        # stops being safe.
        coneSketch: adsk.fusion.Sketch = \
            designComponent.sketches.add(self._gearProfilesPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneEnd = solids.combine_point(apexWorld, R_heel, coneVec)
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, coneEnd)
        coneElementLine.isConstruction = True

        tracePlane: adsk.fusion.ConstructionPlane = solids.plane_by_angle(
            designComponent, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # --- S16. The genuine cutter arc ----------------------------------
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        # The hand sign goes on the cos / Cy term, NOT the sin / Cx term.
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
        heel2d = solids.circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)

        def tanW(px, py):
            return solids.combine_point(apexWorld, px, coneVec, py, v)

        traceSketch: adsk.fusion.Sketch = \
            designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            tanW(Cx, Cy), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        cutterDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, tanW(Cx + r_c, Cy))
        cutterDim.parameter.value = 2 * r_c

        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            tanW(toe2d[0], toe2d[1]), tanW(R_mean, 0),
            tanW(heel2d[0], heel2d[1]))
        traceArc.isConstruction = True
        # addByThreePoints COPIES the arc's centre rather than sharing it, so
        # this coincident is required, not redundant.
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        traceDim = traceSketch.sketchDimensions.addRadialDimension(
            traceArc, tanW(R_mean, 0))
        traceDim.parameter.value = r_c

        # --- S17 E. The slice ---------------------------------------------
        # Exactly 8 planes PARALLEL TO THE PARENT TOOTH PLANE, stepping toward
        # the apex in span/6 increments. The family is what the build requires:
        # the heel-most slab's heel face IS the parent plane.
        planeGeometry: adsk.core.Plane = parentToothPlane.geometry
        toApex: adsk.core.Vector3D = _vector_between(
            planeGeometry.origin, apexWorld)
        sign = 1.0 if toApex.dotProduct(planeGeometry.normal) > 0 else -1.0

        def slice_with(signValue):
            offsets = [signValue * (k + 1) * span / 6.0 for k in range(8)]
            return solids.slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)

        pieces = slice_with(sign)
        if len(pieces) <= 1:
            futil.log(
                f'{gearLabel}: slice produced {len(pieces)} piece(s) with sign '
                f'{sign:+.0f}; retrying with the opposite sign',
                force_console=True)
            sign = -sign
            pieces = slice_with(sign)
        if len(pieces) <= 1:
            raise Exception(
                f'{gearLabel}: the slice produced {len(pieces)} piece(s) with '
                f'both signs tried (last sign {sign:+.0f}, span {span * 10:.4f} mm, '
                f'R_toe {R_toe * 10:.4f} mm, R_heel {R_heel * 10:.4f} mm). The '
                f'parent tooth plane sits outside the tooth\'s span.')

        # --- S18. Order and drop the apex scrap ---------------------------
        segments = sorted(
            pieces,
            key=lambda body: distAlong(body.physicalProperties.centerOfMass))
        scrap = segments[0]
        # Re-slice the list FIRST, then delete.
        segments = segments[1:]
        features.removeFeatures.add(scrap)
        if not segments:
            raise Exception(
                f'{gearLabel}: no segments remain after dropping the apex '
                f'scrap - the slice in S17 failed (span {span * 10:.4f} mm)')

        def slabHeelFace(body):
            # The slab's heel face is the one whose centroid has the GREATEST
            # distAlong, searched across ALL faces with NO surface-type filter.
            best = None
            bestKey = None
            for face in body.faces:
                key = distAlong(face.centroid)
                if bestKey is None or key > bestKey:
                    best, bestKey = face, key
            if best is None:
                raise Exception(
                    f'{gearLabel}: a sliced segment carries no faces')
            return best

        def slabToeFace(body):
            best = None
            bestKey = None
            for face in body.faces:
                key = distAlong(face.centroid)
                if bestKey is None or key < bestKey:
                    best, bestKey = face, key
            if best is None:
                raise Exception(
                    f'{gearLabel}: a sliced segment carries no faces')
            return best

        # --- S19. Twist ---------------------------------------------------
        # The conjugate crown-gear generation law, computed analytically. gamma
        # is this gear's PITCH cone angle, not acos(coneVec . axisDir).
        phi_crown = (math.atan2(heel2d[1], heel2d[0])
                     - math.atan2(toe2d[1], toe2d[0]))
        total = abs(phi_crown) / math.sin(gamma)

        for segment in segments:
            R_heelFace = distAlong(slabHeelFace(segment).centroid)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            if ang == 0:
                # A zero angle is the identity, which Fusion refuses to move a
                # body by ([PB-MOVE-ROTATE]).
                continue
            matrix: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(segment)
            moveInput = features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            features.moveFeatures.add(moveInput)

        # --- S20. Crown ---------------------------------------------------
        # scaleFeatures is the ONE exception to never-activate: it needs the
        # Design occurrence as the active edit target.
        self.designOccurrence.activate()
        try:
            # Recomputed AFTER the twist has moved the slabs.
            keyed = [(distAlong(slabHeelFace(segment).centroid), segment)
                     for segment in segments]
            outermost = max(range(len(keyed)), key=lambda i: keyed[i][0])
            for index, (R_heelFace, segment) in enumerate(keyed):
                if index == outermost:
                    continue
                u = (R_heel - R_heelFace) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor:.6f} is not positive '
                        f'at u {u:.6f} (total twist {total:.6f} rad, '
                        f'_CROWN_PER_RAD {self._CROWN_PER_RAD})')

                heelFace: adsk.fusion.BRepFace = slabHeelFace(segment)
                # Anchor on the heel face's ROOT edge, not its centroid: a
                # uniform scale about a centroid lifts the tooth off the gear
                # base. The two vertices nearest the shaft axis are the root
                # corners.
                vertexPoints = [heelFace.vertices.item(i).geometry
                                for i in range(heelFace.vertices.count)]
                vertexPoints.sort(
                    key=lambda point: _axis_distance(point, apexWorld, axisDir))
                if len(vertexPoints) < 2:
                    raise Exception(
                        f'{gearLabel}: the heel face of a crowned segment has '
                        f'{len(vertexPoints)} vertices, needs at least 2 to find '
                        f'its root edge')
                rootMid = _midpoint(vertexPoints[0], vertexPoints[1])

                baseSketch: adsk.fusion.Sketch = \
                    designComponent.sketches.add(heelFace)
                basePoint = baseSketch.sketchPoints.add(
                    baseSketch.modelToSketchSpace(rootMid))

                targets = adsk.core.ObjectCollection.create()
                targets.add(segment)
                scaleInput = features.scaleFeatures.createInput(
                    targets, basePoint,
                    adsk.core.ValueInput.createByReal(factor))
                features.scaleFeatures.add(scaleInput)
        finally:
            # A Component has no .activate(); the root is re-activated through
            # Design.activateRootComponent().
            self.design.activateRootComponent()

        # --- S21. Loft the spiral tooth -----------------------------------
        # Re-sort HERE, after the twist and the crown: the twist can reorder
        # adjacent slabs for high-twist unequal-ratio pairs.
        order = sorted(
            range(len(segments)),
            key=lambda i: distAlong(slabHeelFace(segments[i]).centroid))

        spiralInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The toe-most segment's toe-facing face goes in first, to push the
        # loft past the toe cone so the toe trim bites.
        spiralInput.loftSections.add(slabToeFace(segments[order[0]]))
        for i in order:
            spiralInput.loftSections.add(slabHeelFace(segments[i]))
        spiralLoft = features.loftFeatures.add(spiralInput)
        spiralTooth: adsk.fusion.BRepBody = spiralLoft.bodies.item(0)
        spiralTooth.name = f'{gearLabel} Spiral Tooth'

        for segment in segments:
            features.removeFeatures.add(segment)

        # --- S22. The conical end cuts ------------------------------------
        return solids.cut_conical_ends(
            designComponent, spiralTooth, gearBody, toeMid, heelMid,
            apexWorld, gearLabel)

    # ----------------------------------------------------------------------
    # S25 through S27: the bore.
    # ----------------------------------------------------------------------
    def _cutBore(self, ctx):
        if not self._boreEnable:
            return

        component: adsk.fusion.Component = self.designComponent
        shaftAxisEdge: adsk.fusion.SketchLine = ctx['shaftAxisEdge']

        # S25. A plane normal to the shaft at its start, so its origin sits on
        # the axis. The in-sketch profile edge goes in directly.
        planes: adsk.fusion.ConstructionPlanes = component.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane: adsk.fusion.ConstructionPlane = planes.add(planeInput)

        # S26. The bore circle, centred on the sketch origin. Its centre is
        # FIXED rather than coincidented to sketch.originPoint
        # ([PB-CIRCLE-CENTER]).
        sketch: adsk.fusion.Sketch = component.sketches.add(borePlane)
        sketch.name = f"{ctx['label']} Bore"
        boreDiameter = ctx['boreDiameter_cm']
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter / 2.0)
        circle.centerSketchPoint.isFixed = True
        diameterDim = sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreDiameter / 2.0, 0, 0))
        diameterDim.parameter.value = boreDiameter

        if not sketch.isFullyConstrained:
            raise Exception(
                f'Bevel Gear: the "{ctx["label"]} Bore" sketch is not fully '
                f'constrained')

        # S27. A symmetric through-cut restricted to this Gear Body.
        coneDist = math.hypot(to_cm(self._module * self._drivingTeeth),
                              to_cm(self._module * self._pinionTeeth))
        features: adsk.fusion.Features = component.features
        extrudeInput = features.extrudeFeatures.createInput(
            sketch.profiles.item(0),
            adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * coneDist), False)
        extrudeInput.participantBodies = [ctx['gearBody']]
        features.extrudeFeatures.add(extrudeInput)

    # ----------------------------------------------------------------------
    # S28: the meshing rotation, in Design before the body is moved out.
    # ----------------------------------------------------------------------
    def _applyMeshRotation(self, ctx):
        if ctx['label'] == 'Driving':
            angle = math.radians(180.0 / self._drivingTeeth)
        else:
            angle = self._pinionMeshPhase(self._pinionTeeth)
        solids.rotate_body_about_edge(
            self.designComponent, ctx['gearBody'], ctx['shaftAxisEdge'], angle)

    @classmethod
    def _pinionMeshPhase(cls, pinionTeeth):
        # Radians. A zero angle is a no-op the helper absorbs, not a move.
        return cls._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # ----------------------------------------------------------------------
    # S29: relocate the finished body, which preserves world position and
    # needs no activation ([PB-NO-CROSS-SIBLING]).
    # ----------------------------------------------------------------------
    def _moveBodyToGearComponent(self, ctx):
        gearBody: adsk.fusion.BRepBody = ctx['gearBody']
        gearBody.moveToComponent(ctx['gearOccurrence'])

    # ----------------------------------------------------------------------
    # S30: hide the construction geometry. Bevel always builds solids, so
    # there is no sketch-only mode and no per-mode guard.
    # ----------------------------------------------------------------------
    def _cleanup(self):
        solids.hide_construction_geometry(self.bevelComponent)
