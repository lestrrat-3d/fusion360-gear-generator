# Bevel gear pair generator.
#
# Transcribed from the compiled step list spec/bevelgear/steps.md (S01-S28) and its
# proof under proof/bevelgear/. Bevel registers NO Fusion user parameters
# ([PB-PRECOMPUTED-MODE]): every value is precomputed in Python, in Fusion's internal
# centimetres, and written into geometry numerically ([PB-NUMERIC-SNAPSHOT]).

import math
from typing import Tuple, overload

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_selection, get_boolean
from .utilities import find_profile_by_curve_counts
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator


# -----------------------------------------------------------------------------------------
# S01 — dialog input ids, in the table's row order. Every string here is contract surface.
# -----------------------------------------------------------------------------------------
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

# S07's seed-held gate ([BEVEL-F-SEED-HELD]): 0.001 mm, in internal cm.
_SEED_TOLERANCE_CM = 1e-4


# -----------------------------------------------------------------------------------------
# Small geometry helpers. The §2 lattice lives in the Gear Profiles sketch's own 2-D frame
# ([BEVEL-F-APEX-LOCAL]); the body steps work in world space ([PB-WORLD-FRAME]).
# -----------------------------------------------------------------------------------------
def _xy(p):
    # [PB-POINT-HELPER]: §2 mixes raw seed tuples with solved `.geometry` points and feeds
    # both to the same helpers, so every one of them branches on the type.
    if isinstance(p, (tuple, list)):
        return float(p[0]), float(p[1])
    return float(p.x), float(p.y)


def _pt3(p):
    x, y = _xy(p)
    return adsk.core.Point3D.create(x, y, 0)


def _add2(p, q):
    px, py = _xy(p)
    qx, qy = _xy(q)
    return (px + qx, py + qy)


def _sub2(p, q):
    px, py = _xy(p)
    qx, qy = _xy(q)
    return (px - qx, py - qy)


def _scale2(p, k):
    px, py = _xy(p)
    return (px * k, py * k)


def _dot2(p, q):
    px, py = _xy(p)
    qx, qy = _xy(q)
    return px * qx + py * qy


def _norm2(p):
    px, py = _xy(p)
    return math.hypot(px, py)


@overload
def normalize(v: adsk.core.Vector3D) -> adsk.core.Vector3D: ...


@overload
def normalize(v: Tuple[float, float]) -> Tuple[float, float]: ...


def normalize(v):
    """The unit vector of v, in the same shape it arrives in.

    The step list writes `normalize(...)` for the §2 sketch directions and asks for the
    §3a world frame to be normalized too, so one helper carries both: a 2-D seed tuple
    comes back as a tuple and a world Vector3D comes back as a Vector3D. It branches on
    the type rather than assuming one ([PB-POINT-HELPER]). The two shapes are declared
    as overloads so a caller that reads .x / .y / .z off a world direction narrows to
    Vector3D rather than to the 2-D branch; neither declaration changes what runs.
    """
    if isinstance(v, (tuple, list)):
        length = _norm2(v)
        if length == 0:
            raise Exception('Bevel Gear: cannot normalize a zero-length direction')
        px, py = _xy(v)
        return (px / length, py / length)
    length = math.sqrt(_dot3(v, v))
    if length == 0:
        raise Exception('Bevel Gear: cannot normalize a zero-length world direction')
    return adsk.core.Vector3D.create(v.x / length, v.y / length, v.z / length)


def _perp2(p):
    px, py = _xy(p)
    return (-py, px)


def _rotate2(p, angle):
    px, py = _xy(p)
    ca, sa = math.cos(angle), math.sin(angle)
    return (px * ca - py * sa, px * sa + py * ca)


def _mid2(p, q):
    return _scale2(_add2(p, q), 0.5)


def _point_line_distance2(p, a, b):
    # Perpendicular distance from p to the infinite line through a and b.
    d = _sub2(b, a)
    n = _norm2(d)
    if n == 0:
        raise Exception('Bevel Gear: degenerate line in a distance measurement')
    v = _sub2(p, a)
    return abs(v[0] * d[1] - v[1] * d[0]) / n


def _vec3(p, q):
    # The world vector q -> p is _vec3(p, q): p minus q.
    return adsk.core.Vector3D.create(p.x - q.x, p.y - q.y, p.z - q.z)


def _dot3(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z


def _cross3(a, b):
    return adsk.core.Vector3D.create(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x)


def _mid3(p, q):
    return adsk.core.Point3D.create((p.x + q.x) / 2, (p.y + q.y) / 2, (p.z + q.z) / 2)


def _axis_distance3(p, origin, axisDir):
    # Perpendicular distance from the world point p to the line through origin along axisDir.
    d = _vec3(p, origin)
    t = _dot3(d, axisDir)
    r = adsk.core.Vector3D.create(
        d.x - t * axisDir.x, d.y - t * axisDir.y, d.z - t * axisDir.z)
    return math.sqrt(_dot3(r, r))


# -----------------------------------------------------------------------------------------
# S01 — the command dialog.
#
# `configure` and `handle_input_changed` are bound BY NAME from commands/bevelgear/entry.py
# and are never called by this module.
# -----------------------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane — FIRST, so Fusion's auto-focus lands on it
        #    ([PB-AUTOFOCUS-FIRST]: a later hasFocus is ignored).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component — the root component is pre-selected.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4-9, 11-15, 17-20. Numeric fields. Every createByReal default is in Fusion
        # INTERNAL units whatever the unit string says ([PB-DIALOG-DEFAULT-UNITS]), which
        # is why the mm lengths pass through to_cm; the deg fields use createByString so
        # the expression engine parses them.
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

        # 10. Enable Bore — a checkbox, default on.
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

        # 16. Hand of Spiral — a text-list dropdown, Right selected.
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

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        # Cheap and robust: no branch on which input changed.
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        # Hand of Spiral and Cutter Radius matter only for a curved bevel, so they are
        # hidden at psi = 0 and shown above it. Mean Spiral Angle is the controller and
        # stays visible. isVisible only hides the dialog row — the input still exists and
        # _readInputs reads it normally.
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return

        visible = True
        try:
            # The input's .expression, evaluated to internal radians — NOT its .value,
            # which a half-typed expression can leave stale.
            unitsManager: adsk.core.UnitsManager = get_design().unitsManager
            value = unitsManager.evaluateExpression(spiral.expression, 'rad')
            visible = value > 0
        except Exception:
            # A half-typed expression can raise mid-edit; leave both inputs SHOWN.
            visible = True

        hand.isVisible = visible
        cutter.isVisible = visible


# -----------------------------------------------------------------------------------------
# The generator. Bevel uses a STANDALONE generator: it does not subclass base.Generator,
# carries no GenerationContext, and uses none of getOccurrence / addParameter /
# parameterName / createSketchObject. From base.py it borrows only the two input readers.
# -----------------------------------------------------------------------------------------
class BevelGearGenerator:
    # S20: the lengthwise crown's tunable class constant. 0 disables the crown.
    _CROWN_PER_RAD = 0.5

    # S26: the pinion's extra mesh phase, in teeth. 0 by default — the mid-face section
    # is unrotated and already meshes.
    _PINION_MESH_PHASE_TEETH = 0.0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        # Seeded with the cast of the class the field actually holds, never
        # adsk.core.Base.cast, which Fusion does not have.
        self.bevelOccurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(None)

    # -------------------------------------------------------------------------------
    # Error rollback, called by the command entry point on an exception (S28). Bevel
    # registers no user parameters, so there are none to clean up.
    # -------------------------------------------------------------------------------
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)

    # -------------------------------------------------------------------------------
    # S02 — read every input up front, in ONE pass, before anything creates an
    # occurrence ([PB-SELECTION-STASH]), then validate in the stated order.
    # -------------------------------------------------------------------------------
    def _evaluate(self, inputs: adsk.core.CommandInputs, inputId, units):
        # [PB-EVAL-EXPRESSION]: always comes back in Fusion internal units — cm for
        # length, RADIANS for angle — whatever the unit string says.
        unitsManager: adsk.core.UnitsManager = self.design.unitsManager
        return unitsManager.evaluateExpression(
            inputs.itemById(inputId).expression, units)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        def evaluate(inputId, units):
            return self._evaluate(inputs, inputId, units)

        def one_selection(inputId, label):
            entities = get_selection(inputs, inputId)
            if len(entities) != 1:
                raise Exception(
                    '{}: expected exactly one selection, got {}'.format(label, len(entities)))
            return entities[0]

        parentEntity = one_selection(INPUT_ID_PARENT, 'Parent Component')
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentEntity = parentEntity.component
        elif parentEntity.objectType != adsk.fusion.Component.classType():
            raise Exception(
                'Parent Component: unexpected selection type {}'.format(parentEntity.objectType))
        parentComponent: adsk.fusion.Component = parentEntity

        targetPlane = one_selection(INPUT_ID_PLANE, 'Target Plane')
        centerPoint = one_selection(INPUT_ID_CENTER_POINT, 'Center Point')

        # Module is read with '' — a RAW number meaning MILLIMETRES. Every length derived
        # from it is to_cm-converted before it touches geometry.
        module = evaluate(INPUT_ID_MODULE, '')
        drivingTeeth = int(round(evaluate(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evaluate(INPUT_ID_PINION_TEETH, '')))
        shaftAngle_rad = evaluate(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)

        # The 'mm' and 'deg' inputs come back ALREADY internal — never to_cm them again.
        self._drivingBaseHeight_cm = evaluate(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evaluate(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = evaluate(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evaluate(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evaluate(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = evaluate(INPUT_ID_TOOTH_SPACING, 'mm')
        self._spiralAngle_rad = evaluate(INPUT_ID_SPIRAL_ANGLE, 'deg')

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        self._hand = handItem.name if handItem is not None else _HAND_RIGHT

        self._cutterRadius_cm = evaluate(INPUT_ID_CUTTER_RADIUS, 'mm')
        self._toeExtension_pct = evaluate(INPUT_ID_TOE_EXTENSION, '')
        self._drivingToeRadius_cm = evaluate(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        self._pinionToeRadius_cm = evaluate(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        # --- 1. Range checks -------------------------------------------------------
        if module <= 0:
            raise Exception('Module must be greater than 0, got {}'.format(module))
        for label, teeth in (('Driving Gear Teeth', drivingTeeth),
                             ('Pinion Gear Teeth', pinionTeeth)):
            if teeth < 3:
                raise Exception('{} must be at least 3, got {}'.format(label, teeth))

        for label, value in (
                ('Driving Gear Base Height', self._drivingBaseHeight_cm),
                ('Pinion Gear Base Height', self._pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', self._drivingBore_cm),
                ('Pinion Gear Bore Diameter', self._pinionBore_cm),
                ('Face Width', self._faceWidth_cm),
                ('Tooth Spacing', self._toothSpacing_cm),
                ('Toe Extension (%)', self._toeExtension_pct),
                ('Driving Gear Toe Radius', self._drivingToeRadius_cm),
                ('Pinion Gear Toe Radius', self._pinionToeRadius_cm),
                ('Cutter Radius', self._cutterRadius_cm)):
            if value < 0:
                raise Exception('{} must not be negative'.format(label))

        spiralAngle_deg = math.degrees(self._spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                'Mean Spiral Angle must be in [0, 60) degrees, got {:.4f}'.format(spiralAngle_deg))
        if self._toeExtension_pct < 0 or self._toeExtension_pct > 100:
            raise Exception(
                'Toe Extension must be in [0, 100], got {:.4f}'.format(self._toeExtension_pct))

        # --- 2. The Maximum Shaft Angle, which needs BOTH tooth counts --------------
        module_cm = to_cm(module)
        ppd = module_cm * pinionTeeth
        dpd = module_cm * drivingTeeth
        coneLimit_deg = math.degrees(math.acos(-min(ppd, dpd) / max(ppd, dpd)))
        maxShaftAngle_deg = min(coneLimit_deg, 150.0)

        if shaftAngle_deg < 30:
            raise Exception(
                'Shaft Angle must be at least 30 degrees, got {:.4f}'.format(shaftAngle_deg))
        if maxShaftAngle_deg < 150.0:
            # The cone-angle half is EXCLUSIVE: a pitch cone angle reaching 90 degrees
            # turns that gear's cone inside out and R * cos(gamma) changes sign.
            if shaftAngle_deg >= maxShaftAngle_deg:
                raise Exception(
                    'Shaft Angle {:.4f} degrees must stay below the Maximum Shaft Angle '
                    '{:.4f} degrees for a {}/{} pair'.format(
                        shaftAngle_deg, maxShaftAngle_deg, drivingTeeth, pinionTeeth))
        elif shaftAngle_deg > 150.0:
            # The 150 degree half is a practical ceiling and is inclusive.
            raise Exception(
                'Shaft Angle {:.4f} degrees must not exceed the Maximum Shaft Angle '
                '150.0000 degrees'.format(shaftAngle_deg))

        # --- 3. The two pitch cone angles ------------------------------------------
        sigma = math.radians(shaftAngle_deg)
        gamma_p = math.atan2(math.sin(sigma) * ppd, dpd + ppd * math.cos(sigma))
        gamma_g = sigma - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        # --- 4. Minimum Teeth, per gear with that gear's own gamma. Checked BEFORE the
        #        base heights, because it is exactly the statement that the base-height
        #        window is non-empty. 5.27 is 2*(1.05*1.25/0.95 + 1.25) rounded UP.
        for label, teeth, gamma in (('Driving Gear', drivingTeeth, gamma_g),
                                    ('Pinion Gear', pinionTeeth, gamma_p)):
            floor = 5.27 * math.cos(gamma)
            if teeth < floor:
                raise Exception(
                    '{}: {} teeth is below the computed Minimum Teeth floor {:.4f} for a '
                    'pitch cone angle of {:.4f} degrees'.format(
                        label, teeth, floor, math.degrees(gamma)))

        # --- 5. Base heights, per gear, both closed-form ----------------------------
        ded = 1.25 * module_cm
        R = (ppd / 2) / math.sin(gamma_p)

        def base_height_window(gamma, pitchRadius):
            minimum = 1.05 * ded * math.sin(gamma)
            maximum = 0.95 * (pitchRadius - ded * math.cos(gamma)) * math.tan(gamma)
            return minimum, maximum

        def resolve_base_height(label, user, fallback, gamma, pitchRadius):
            minimum, maximum = base_height_window(gamma, pitchRadius)
            if user == 0:
                # A fallback below the minimum is RAISED and one above the maximum CAPPED.
                return max(minimum, min(fallback, maximum))
            if user < minimum:
                raise Exception(
                    '{} Base Height {:.4f} mm is below the Minimum Base Height '
                    '{:.4f} mm'.format(label, to_mm(user), to_mm(minimum)))
            if user > maximum:
                raise Exception(
                    '{} Base Height {:.4f} mm is above the Maximum Base Height '
                    '{:.4f} mm'.format(label, to_mm(user), to_mm(maximum)))
            return user

        drivingHeight = resolve_base_height(
            'Driving Gear', self._drivingBaseHeight_cm,
            module_cm * drivingTeeth / 8, gamma_g, dpd / 2)
        # The pinion fallback is the RESOLVED driving height times the tooth ratio, and
        # then the PINION's own window applies, because the two gammas differ.
        pinionHeight = resolve_base_height(
            'Pinion Gear', self._pinionBaseHeight_cm,
            drivingHeight * pinionTeeth / drivingTeeth, gamma_p, ppd / 2)

        self._drivingBaseHeight_cm = drivingHeight
        self._pinionBaseHeight_cm = pinionHeight

        # The bore bound is NOT part of this pass: its toe term needs the Root Length,
        # hence the resolved Face Width, hence solved §2 geometry. It resolves in S07.
        self._module_mm = module
        self._module_cm = module_cm
        self._ppd_cm = ppd
        self._dpd_cm = dpd
        self._pitchCone_cm = R
        self._dedendum_cm = ded
        self._coneDistance_cm = math.hypot(ppd, dpd)
        self._sigma_rad = sigma

        futil.log(
            'Bevel Gear: module {} mm, teeth {}/{}, shaft angle {:.4f} deg, '
            'gamma_p {:.4f} deg, gamma_g {:.4f} deg'.format(
                module, drivingTeeth, pinionTeeth, shaftAngle_deg,
                math.degrees(gamma_p), math.degrees(gamma_g)))

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # -------------------------------------------------------------------------------
    # The module's entry point. commands/_gear_command.py calls it; nothing here does.
    # -------------------------------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        (parent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)
        parentComponent: adsk.fusion.Component = parent

        # S03 — the Bevel Gear component, a child of the user's Parent Component.
        # NEVER activate any occurrence ([PB-NEVER-ACTIVATE], [BEVEL-F-NEVER-ACTIVATE]):
        # the Anchor Sketch sits on the user's EXTERNAL, root-owned target plane, and an
        # activated occurrence resolves that plane in its own local frame, collapsing the
        # whole build onto world XY. The sole exception is the spiral crown's scale (S20).
        self.bevelOccurrence: adsk.fusion.Occurrence = (
            parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()))
        self.bevelComponent: adsk.fusion.Component = self.bevelOccurrence.component
        self.bevelComponent.name = 'Bevel Gear'

        # S04 — the Design component, a child of Bevel Gear. Every sketch, construction
        # plane, construction axis and feature runs here, because Fusion rejects
        # cross-sibling sketch and project references ([PB-NO-CROSS-SIBLING]); the
        # finished bodies are moved out at the end.
        self.designOccurrence: adsk.fusion.Occurrence = (
            self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()))
        self.designComponent: adsk.fusion.Component = self.designOccurrence.component
        self.designComponent.name = 'Design'

        self._buildAnchorSketch(targetPlane, centerPoint)
        self._buildGearProfilesPlane(targetPlane)
        self._buildGearProfiles(
            targetPlane, module, drivingTeeth, pinionTeeth, shaftAngle_deg)

        # S28 — cleanup.
        solids.hide_construction_geometry(self.bevelComponent)

    # -------------------------------------------------------------------------------
    # S05 — the Anchor sketch, DIRECTLY on the user-selected target plane.
    # -------------------------------------------------------------------------------
    def _buildAnchorSketch(self, targetPlane, centerPoint):
        # [PB-USE-SELECTED-PLANE]: never re-derive or offset the selected plane — a
        # coplanar plane built inside Design resolves in Design's own frame and silently
        # loses the selected plane's world orientation.
        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint).item(0)
        c = projectedCenter.geometry

        # Seed the two endpoints at exactly +/- 0.5 cm along the sketch-local X, so the
        # seeded length is 10 mm.
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(c.x - 0.5, c.y, 0),
            adsk.core.Point3D.create(c.x + 0.5, c.y, 0))
        anchorLine.isConstruction = True

        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        # BOTH are required: the coincident pins the centre onto the line and the
        # midpoint makes it bisect it. Midpoint alone is not enough.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # An ALIGNED dimension, left at the seeded 10 mm: its value is arbitrary because
        # this is only a reference line, so .parameter.value is deliberately NOT assigned.
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(c.x, c.y + 0.2, 0))

        # [PB-REFLINE-DIRECTION]: sketch-local, so it survives any tilted target plane.
        # A world-axis lock would mis-orient it.
        constraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception(
                'Anchor sketch is not fully constrained — a free DOF is a generation '
                'defect, not a warning')

        self._anchorSketch = sketch
        self._anchorLine = anchorLine
        # §2 re-projects THIS point rather than the raw user-selected centre.
        self._anchorCenterPoint = projectedCenter

    # -------------------------------------------------------------------------------
    # S06 — the Gear Profiles Plane, built off the ORIGINAL targetPlane.
    # -------------------------------------------------------------------------------
    def _buildGearProfilesPlane(self, targetPlane):
        planes: adsk.fusion.ConstructionPlanes = self.designComponent.constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
        # Pass the SketchLine DIRECTLY; never wrap it in Path.create, which raises an
        # InternalValidationError in a multi-component context ([PB-CONSTRUCTION-PLANES]).
        planeInput.setByAngle(
            self._anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = planes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # -------------------------------------------------------------------------------
    # The §2 drawing primitives. Every line in the Gear Profiles sketch is a
    # construction line and every length dimension in it is ALIGNED, so both live here
    # rather than being spelled out at each of the forty-odd sites.
    # -------------------------------------------------------------------------------
    def _constructionLine(self, sketch: adsk.fusion.Sketch, p,
                          q) -> adsk.fusion.SketchLine:
        lines: adsk.fusion.SketchLines = sketch.sketchCurves.sketchLines
        drawn = lines.addByTwoPoints(_pt3(p), _pt3(q))
        drawn.isConstruction = True
        return drawn

    def _alignedDimension(self, sketch: adsk.fusion.Sketch,
                          drawn: adsk.fusion.SketchLine, value, textPoint):
        # The §2 figure has no axis-aligned line in it — the shaft axes sit at the Shaft
        # Angle to each other and the whole lattice tilts with the target plane — so a
        # horizontal or vertical orientation would dimension the line's PROJECTION onto a
        # sketch axis instead of its length.
        dimensions: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        dim = dimensions.addDistanceDimension(
            drawn.startSketchPoint, drawn.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _pt3(textPoint))
        # [PB-DIM-VALUE-SEMANTICS]: only the absolute magnitude goes in; the side is
        # chosen by the seed. [PB-DRIVING-DIM]: never pass a trailing isDriven.
        dim.parameter.value = abs(value)
        return dim

    def _toothSpacingLine(self, sketch: adsk.fusion.Sketch,
                          constraints: adsk.fusion.GeometricConstraints,
                          startPoint, startSeed, dedLine, dedDir, apex2Seed,
                          virtualPitchRadius, toothSpacing, cornerPoint, cornerSeed):
        # S07 step 26. The seed uses the EXACT back-cone radius, never a radius rebuilt
        # from a rounded tooth count. The length dimension is UNSIGNED, so the
        # point-on-line pin plus the length admit the mirror one Tooth Spacing on the C
        # side — a flipped K' tightens the mesh by exactly the clearance the input asked
        # to add, and builds a gear that looks right.
        primeSeed = _add2(apex2Seed, _scale2(dedDir, virtualPitchRadius + toothSpacing))
        spacingLine = self._constructionLine(sketch, startSeed, primeSeed)
        constraints.addCoincident(spacingLine.startSketchPoint, startPoint)
        primePoint = spacingLine.endSketchPoint
        constraints.addCoincident(primePoint, dedLine)
        self._alignedDimension(
            sketch, spacingLine, toothSpacing, _mid2(startSeed, primeSeed))
        refLine = self._constructionLine(sketch, cornerSeed, primeSeed)
        constraints.addCoincident(refLine.startSketchPoint, cornerPoint)
        constraints.addCoincident(refLine.endSketchPoint, primePoint)
        return primePoint, primeSeed, refLine

    # -------------------------------------------------------------------------------
    # S07 — the Gear Profiles sketch: the whole §2 lattice, in ONE sketch.
    #
    # Every line here is a construction line; every length dimension is ALIGNED, because
    # this figure has no axis-aligned line in it. Every line is built in the COINCIDENT
    # style ([BEVEL-F-COINCIDENT-STYLE]): created from raw Point3D coordinates for BOTH
    # endpoints, with exactly one addCoincident per endpoint that already exists. Each
    # named line is created ONCE and reused ([BEVEL-F-LINE-ONCE]).
    # -------------------------------------------------------------------------------
    def _buildGearProfiles(self, targetPlane, module, drivingTeeth, pinionTeeth,
                           shaftAngle_deg):
        ppd = self._ppd_cm
        dpd = self._dpd_cm
        sigma = self._sigma_rad
        gp = self._gamma_p
        gg = self._gamma_g
        R = self._pitchCone_cm
        ded = self._dedendum_cm
        coneDistance = self._coneDistance_cm
        hp = self._pinionBaseHeight_cm
        hg = self._drivingBaseHeight_cm
        # |Apex->Ded|, the dedendum corner's distance from the apex.
        apexDed = math.hypot(R, ded)

        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(
            self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch
        gc: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions

        # --- The frame -------------------------------------------------------------
        # Project the Anchor sketch's stashed centre point, not the raw user-selected
        # centre, which is a cross-component reference and can resolve inconsistently.
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchorLine = sketch.project(self._anchorLine).item(0)

        c = _xy(projectedCenter.geometry)
        d = normalize(_sub2(projectedAnchorLine.endSketchPoint.geometry,
                         projectedAnchorLine.startSketchPoint.geometry))
        perp = _perp2(d)

        # [BEVEL-F-GROW-SIDE]: the perpendicular's sign points toward the TARGET-PLANE
        # NORMAL, for both selection kinds — a BRepFace's geometry and a
        # ConstructionPlane's geometry are each a core.Plane carrying .normal. A
        # sketch-local rule such as perp.y >= 0 is deterministic but not tied to a
        # physical side. The normal is read as a DIRECTION only, which is the single
        # permitted world use in this sketch ([BEVEL-F-APEX-LOCAL]).
        normal = targetPlane.geometry.normal
        originLocal = sketch.modelToSketchSpace(adsk.core.Point3D.create(0, 0, 0))
        normalLocal = sketch.modelToSketchSpace(
            adsk.core.Point3D.create(normal.x, normal.y, normal.z))
        if _dot2(perp, _sub2(normalLocal, originLocal)) < 0:
            perp = _scale2(perp, -1)

        # --- The seeds, all from the closed forms §2 states -------------------------
        apexSeed = _add2(c, _scale2(perp, R * math.cos(gg) + hg))
        drivingDir = _scale2(perp, -1)
        bSeed = _add2(apexSeed, _scale2(drivingDir, R * math.cos(gg)))

        # Form BOTH candidate point-A positions and keep the one with the GREATER X.
        # Rotating one fixed sense and flipping only on a negative X keeps the wrong one
        # whenever both candidates have a positive X.
        candidates = []
        for sense in (1.0, -1.0):
            direction = normalize(_rotate2(drivingDir, sense * sigma))
            candidates.append((direction, _add2(apexSeed, _scale2(direction, R * math.cos(gp)))))
        pinionDir, aSeed = max(candidates, key=lambda candidate: candidate[1][0])

        # The A->Apex2 drop points into the interior wedge, toward B; the B->Apex2 drop
        # toward A. Each sense is picked by the sign of its dot with that direction, never
        # against a "toward the anchor line" reference: the Driving Gear Shaft Axis is
        # itself parallel to the grow direction, so that test reads about zero.
        def drop_direction(axisDir, fromPoint, towardPoint):
            u = _perp2(axisDir)
            if _dot2(u, _sub2(towardPoint, fromPoint)) < 0:
                u = _scale2(u, -1)
            return u

        radialP = drop_direction(pinionDir, aSeed, bSeed)
        radialD = drop_direction(drivingDir, bSeed, aSeed)
        apex2Seed = _add2(aSeed, _scale2(radialP, ppd / 2))
        apex2SeedFromB = _add2(bSeed, _scale2(radialD, dpd / 2))

        pitchDir = normalize(_sub2(apex2Seed, apexSeed))
        uDed = _perp2(pitchDir)
        # The PINION dedendum direction is the u with u . unit(Apex->A) > 0 — that dot is
        # exactly sin(gamma_p), strictly positive for every admitted configuration — and
        # the DRIVING one is its negation.
        dedDirP = uDed if _dot2(uDed, pinionDir) > 0 else _scale2(uDed, -1)
        dedDirD = _scale2(dedDirP, -1)

        cSeed = _add2(apex2Seed, _scale2(dedDirP, ded))
        dSeed = _add2(apex2Seed, _scale2(dedDirD, ded))
        eSeed = _add2(aSeed, _scale2(pinionDir, ded * math.sin(gp)))
        fSeed = _add2(bSeed, _scale2(drivingDir, ded * math.sin(gg)))
        gSeed = _add2(aSeed, _scale2(pinionDir, hp))
        hSeed = _add2(apex2Seed, _scale2(dedDirP, hp / math.sin(gp)))
        iSeed = _add2(bSeed, _scale2(drivingDir, hg))
        jSeed = _add2(apex2Seed, _scale2(dedDirD, hg / math.sin(gg)))

        # The EXACT back-cone radii the tooth centres sit at, never a radius rebuilt from
        # a rounded tooth count.
        virtualPitchRadiusP = (ppd / 2) / math.cos(gp)
        virtualPitchRadiusD = (dpd / 2) / math.cos(gg)
        kSeed = _add2(apex2Seed, _scale2(dedDirP, virtualPitchRadiusP))
        lSeed = _add2(apex2Seed, _scale2(dedDirD, virtualPitchRadiusD))

        # --- 1. c->Apex ------------------------------------------------------------
        centerToApex = self._constructionLine(sketch, c, apexSeed)
        gc.addPerpendicular(centerToApex, projectedAnchorLine)
        gc.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        pointApex = centerToApex.endSketchPoint

        # --- 2. Apex->B, the Driving Gear Shaft Axis --------------------------------
        drivingShaftAxis = self._constructionLine(sketch, apexSeed, bSeed)
        # addParallel, NOT addVertical, which forces the sketch's world-vertical and
        # mis-orients the figure on a tilted target plane.
        gc.addParallel(drivingShaftAxis, centerToApex)
        gc.addCoincident(drivingShaftAxis.startSketchPoint, pointApex)
        pointB = drivingShaftAxis.endSketchPoint

        # --- 3. Apex->A, the Pinion Gear Shaft Axis ---------------------------------
        pinionShaftAxis = self._constructionLine(sketch, apexSeed, aSeed)
        gc.addCoincident(pinionShaftAxis.startSketchPoint, pointApex)
        # [PB-ANGULAR-DIM]: the text point sits INSIDE the sigma wedge, so the dimension
        # measures sigma and not 180 - sigma.
        wedgePoint = _add2(
            apexSeed, _scale2(normalize(_add2(pinionDir, drivingDir)), ppd / 4))
        shaftAngleDim = dims.addAngularDimension(
            pinionShaftAxis, drivingShaftAxis, _pt3(wedgePoint))
        shaftAngleDim.parameter.value = sigma
        pointA = pinionShaftAxis.endSketchPoint

        # --- 4. A->Apex2, the PPD/2 drop -------------------------------------------
        aDropLine = self._constructionLine(sketch, aSeed, apex2Seed)
        gc.addCoincident(aDropLine.startSketchPoint, pointA)
        gc.addPerpendicular(aDropLine, pinionShaftAxis)
        self._alignedDimension(sketch, aDropLine, ppd / 2, _mid2(aSeed, apex2Seed))

        # --- 5. B->Apex2, the DPD/2 drop -------------------------------------------
        bDropLine = self._constructionLine(sketch, bSeed, apex2SeedFromB)
        gc.addCoincident(bDropLine.startSketchPoint, pointB)
        gc.addPerpendicular(bDropLine, drivingShaftAxis)
        self._alignedDimension(sketch, bDropLine, dpd / 2, _mid2(bSeed, apex2SeedFromB))

        # --- 6. Close the two drops: that point is Apex 2 ---------------------------
        gc.addCoincident(aDropLine.endSketchPoint, bDropLine.endSketchPoint)
        pointApex2 = aDropLine.endSketchPoint

        # --- 7. Apex->Apex2, the Pitch Line ----------------------------------------
        pitchLine = self._constructionLine(sketch, apexSeed, apex2Seed)
        gc.addCoincident(pitchLine.startSketchPoint, pointApex)
        gc.addCoincident(pitchLine.endSketchPoint, pointApex2)

        # --- 8. Apex2->C and Apex2->D, the dedendum lines --------------------------
        # These two sites are among the 15 whose side is decided by the SEED alone
        # ([BEVEL-F-MIRROR-FIGURE]): the perpendicular fixes the direction and the
        # dimension the magnitude, and neither picks a side. Flip the pinion seed and C
        # solves exactly onto D. Never add a Fusion constraint to pin a side — every
        # constraint Fusion offers here is unsigned or undirected.
        pinionDedLine = self._constructionLine(sketch, apex2Seed, cSeed)
        gc.addCoincident(pinionDedLine.startSketchPoint, pointApex2)
        gc.addPerpendicular(pinionDedLine, pitchLine)
        self._alignedDimension(sketch, pinionDedLine, ded, _mid2(apex2Seed, cSeed))
        pointC = pinionDedLine.endSketchPoint

        drivingDedLine = self._constructionLine(sketch, apex2Seed, dSeed)
        gc.addCoincident(drivingDedLine.startSketchPoint, pointApex2)
        gc.addPerpendicular(drivingDedLine, pitchLine)
        self._alignedDimension(sketch, drivingDedLine, ded, _mid2(apex2Seed, dSeed))
        pointD = drivingDedLine.endSketchPoint

        # --- 9. Apex->C and Apex->D, the two Root Axes -----------------------------
        pinionRootAxis = self._constructionLine(sketch, apexSeed, cSeed)
        gc.addCoincident(pinionRootAxis.startSketchPoint, pointApex)
        gc.addCoincident(pinionRootAxis.endSketchPoint, pointC)
        drivingRootAxis = self._constructionLine(sketch, apexSeed, dSeed)
        gc.addCoincident(drivingRootAxis.startSketchPoint, pointApex)
        gc.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        # --- 10-13. The two extension chains, first leg ----------------------------
        aToE = self._constructionLine(sketch, aSeed, eSeed)
        gc.addCoincident(aToE.startSketchPoint, pointA)
        gc.addCollinear(aToE, pinionShaftAxis)
        pointE = aToE.endSketchPoint

        cToE = self._constructionLine(sketch, cSeed, eSeed)
        gc.addCoincident(cToE.startSketchPoint, pointC)
        gc.addCoincident(cToE.endSketchPoint, pointE)
        gc.addPerpendicular(cToE, aToE)

        bToF = self._constructionLine(sketch, bSeed, fSeed)
        gc.addCoincident(bToF.startSketchPoint, pointB)
        gc.addCollinear(bToF, drivingShaftAxis)
        pointF = bToF.endSketchPoint

        dToF = self._constructionLine(sketch, dSeed, fSeed)
        gc.addCoincident(dToF.startSketchPoint, pointD)
        gc.addCoincident(dToF.endSketchPoint, pointF)
        gc.addPerpendicular(dToF, bToF)

        # --- 14-19. The heel edges. Each collinear names the line the new line's start
        # actually sits ON ([BEVEL-F-COLLINEAR-CHAIN], [PB-COLLINEAR-CHAIN]): E->G names
        # A->E and NEVER the Apex->A shaft axis further up the same chain, which asserts
        # one point-on-line row twice and raises VCS_SKETCH_OVER_CONSTRAINTS.
        eToG = self._constructionLine(sketch, eSeed, gSeed)
        gc.addCoincident(eToG.startSketchPoint, pointE)
        gc.addCollinear(eToG, aToE)
        pointG = eToG.endSketchPoint

        cToH = self._constructionLine(sketch, cSeed, hSeed)
        gc.addCoincident(cToH.startSketchPoint, pointC)
        gc.addCollinear(cToH, pinionDedLine)
        pointH = cToH.endSketchPoint

        gToH = self._constructionLine(sketch, gSeed, hSeed)
        gc.addCoincident(gToH.startSketchPoint, pointG)
        gc.addCoincident(gToH.endSketchPoint, pointH)
        # REQUIRED in Fusion: addOffsetDimension needs its second entity already parallel
        # to the first, and E->G runs along the pinion shaft, so squaring H->G to it makes
        # H->G parallel to the A->Apex2 drop.
        gc.addPerpendicular(eToG, gToH)

        fToI = self._constructionLine(sketch, fSeed, iSeed)
        gc.addCoincident(fToI.startSketchPoint, pointF)
        gc.addCollinear(fToI, bToF)
        pointI = fToI.endSketchPoint

        dToJ = self._constructionLine(sketch, dSeed, jSeed)
        gc.addCoincident(dToJ.startSketchPoint, pointD)
        gc.addCollinear(dToJ, drivingDedLine)
        pointJ = dToJ.endSketchPoint

        iToJ = self._constructionLine(sketch, iSeed, jSeed)
        gc.addCoincident(iToJ.startSketchPoint, pointI)
        gc.addCoincident(iToJ.endSketchPoint, pointJ)
        gc.addPerpendicular(fToI, iToJ)

        # --- 20-21. The two base-height offsets. The first entity is the DROP, not the
        # shaft axis. J->I and H->G are already parallel to their drop by construction, so
        # add NO extra parallel constraint ([PB-OFFSET-DIM]). Both are UNSIGNED: the I/J
        # and G/H seeds are the only thing that picks the side.
        drivingOffsetDim = dims.addOffsetDimension(
            bDropLine, iToJ, _pt3(_mid2(_mid2(bSeed, apex2SeedFromB), _mid2(iSeed, jSeed))))
        drivingOffsetDim.parameter.value = hg
        pinionOffsetDim = dims.addOffsetDimension(
            aDropLine, gToH, _pt3(_mid2(_mid2(aSeed, apex2Seed), _mid2(gSeed, hSeed))))
        pinionOffsetDim.parameter.value = hp

        # --- 27. Resolve the Maximum Face Width and apply it ------------------------
        # A, B, C, D, H and J all exist and are SOLVED by now, so the bound is read off
        # their solved .geometry ([PB-SOLVED-GEOMETRY]) rather than off the pre-solve
        # seeds, which diverge substantially for asymmetric tooth counts and non-90 degree
        # shaft angles. Compute BOTH distances and take the minimum: the pinion is only
        # USUALLY the smaller, binding side.
        #
        # This resolution is stated as step 27 of §2 but is performed HERE, before the
        # A'->G line of step 22, because A' is seeded at the along-shaft coordinate of N,
        # which needs the Root Length, which needs this Face Width. Nothing it reads moves
        # between the two positions: every point it reads is already solved.
        pinionSpan = _point_line_distance2(
            pointA.geometry, pointC.geometry, pointH.geometry)
        drivingSpan = _point_line_distance2(
            pointB.geometry, pointD.geometry, pointJ.geometry)
        maxFaceWidth = 0.95 * min(pinionSpan, drivingSpan)

        faceWidth = self._faceWidth_cm
        if faceWidth == 0:
            faceWidth = min(coneDistance / 6, maxFaceWidth)
        elif faceWidth > maxFaceWidth:
            raise Exception(
                'Face Width {:.4f} mm exceeds the Maximum Face Width {:.4f} mm for this '
                'pair'.format(to_mm(faceWidth), to_mm(maxFaceWidth)))
        self._faceWidth_cm = faceWidth

        # --- The resolved toe values this step needs --------------------------------
        def resolve_toe_radius(label, user, pitchRadius, gamma):
            # The Toe Radius Ceiling is that gear's OUTER toe corner at Toe Extension 0.
            ceiling = (pitchRadius - ded * math.cos(gamma)) * (1 - faceWidth / R)
            if user > 0:
                if user >= ceiling:
                    raise Exception(
                        '{} Toe Radius {:.4f} mm must be strictly below its Toe Radius '
                        'Ceiling {:.4f} mm'.format(label, to_mm(user), to_mm(ceiling)))
                radius = user
            else:
                # 0 means auto: that gear's inner toe corner at Toe Extension 0, which is
                # what makes Toe Extension 0 reproduce today's profile exactly.
                radius = pitchRadius - faceWidth / math.sin(gamma)
            gammaRoot = gamma - math.atan(ded / R)
            limit = apexDed - radius / math.sin(gammaRoot)
            return radius, ceiling, limit, gammaRoot

        (pinionToeRadius, pinionToeCeiling, pinionToeLimit,
         pinionGammaRoot) = resolve_toe_radius(
            'Pinion Gear', self._pinionToeRadius_cm, ppd / 2, gp)
        (drivingToeRadius, drivingToeCeiling, drivingToeLimit,
         drivingGammaRoot) = resolve_toe_radius(
            'Driving Gear', self._drivingToeRadius_cm, dpd / 2, gg)

        # Root Length: the Face Width re-measured along the root element, plus the Toe
        # Extension's share of the window. Toe Extension 100 stops at 0.99 of the way to
        # the SMALLER of the two Toe Limits, never at the limit itself — at the limit the
        # toe face has zero length and the conical end-cut has no cone face to find.
        rootLength0 = faceWidth * apexDed / R
        rootLength = rootLength0
        if self._toeExtension_pct > 0:
            # A defaulted Toe Radius can leave no room at all, and that is a real
            # configuration: on a gear with a large pitch cone angle the inner toe corner
            # already sits at a LARGER radius than the outer one. Reject rather than
            # silently substitute a smaller Toe Radius.
            for label, limit, ceiling in (
                    ('Pinion Gear', pinionToeLimit, pinionToeCeiling),
                    ('Driving Gear', drivingToeLimit, drivingToeCeiling)):
                if limit <= rootLength0:
                    raise Exception(
                        'Toe Extension {:.4f}% leaves no room on the {}: its Toe Limit '
                        '{:.4f} mm is at or below the Toe Extension 0 root length '
                        '{:.4f} mm. Give that gear a Toe Radius below its Toe Radius '
                        'Ceiling {:.4f} mm, or leave Toe Extension at 0.'.format(
                            self._toeExtension_pct, label, to_mm(limit),
                            to_mm(rootLength0), to_mm(ceiling)))
            reach = min(pinionToeLimit, drivingToeLimit)
            rootLength = rootLength0 + (self._toeExtension_pct / 100) * 0.99 * (
                reach - rootLength0)
        self._rootLength_cm = rootLength

        # --- 28. Resolve each gear's Maximum Bore Diameter and apply it, here and
        # nowhere else. With the Face Width resolved the Root Length follows, so this is
        # the only step at which the whole bound can resolve. Skipped entirely when Enable
        # Bore is unchecked.
        def resolve_bore(label, user, pitchRadius, gamma, gammaRoot, baseHeight):
            if not self._boreEnable:
                return 0.0
            rHeel = pitchRadius - baseHeight / math.tan(gamma)
            rToe = (apexDed - rootLength) * math.sin(gammaRoot)
            maxBore = 2 * 0.95 * min(rHeel, rToe)
            if user == 0:
                return min(2 * pitchRadius / 4, maxBore)
            if user > maxBore:
                raise Exception(
                    '{} Bore Diameter {:.4f} mm exceeds its Maximum Bore Diameter '
                    '{:.4f} mm — a bore past the heel term takes the entire flat back '
                    'face and past the toe term the whole toe dish'.format(
                        label, to_mm(user), to_mm(maxBore)))
            return user

        pinionBore = resolve_bore(
            'Pinion Gear', self._pinionBore_cm, ppd / 2, gp, pinionGammaRoot, hp)
        drivingBore = resolve_bore(
            'Driving Gear', self._drivingBore_cm, dpd / 2, gg, drivingGammaRoot, hg)

        # --- The toe seeds, both gears ---------------------------------------------
        # Seed BOTH ends at their closed-form solved positions, not near them
        # ([PB-SEED-NEAR]). M rides the root axis one Root Length back from the dedendum
        # corner; N slides from THAT M seed along the C->H direction until it reaches this
        # gear's Toe Radius. A seed that merely lands somewhere plausible builds the WRONG
        # gear rather than failing to converge: the toe line meets the Toe Radius on BOTH
        # sides of the shaft axis and the solver takes whichever side the seed starts on.
        def toe_seeds(cornerSeed, axisDir, radialDir, dedDir, gamma, toeRadius):
            apexToCorner = _sub2(cornerSeed, apexSeed)
            mSeed = _add2(
                apexSeed, _scale2(apexToCorner, 1 - rootLength / _norm2(apexToCorner)))
            mRadius = _dot2(_sub2(mSeed, apexSeed), radialDir)
            nSeed = _add2(
                mSeed, _scale2(dedDir, (mRadius - toeRadius) / math.cos(gamma)))
            primeSeed = _add2(
                apexSeed, _scale2(axisDir, _dot2(_sub2(nSeed, apexSeed), axisDir)))
            return mSeed, nSeed, primeSeed

        mSeed, nSeed, aPrimeSeed = toe_seeds(
            cSeed, pinionDir, radialP, dedDirP, gp, pinionToeRadius)
        oSeed, pSeed, bPrimeSeed = toe_seeds(
            dSeed, drivingDir, radialD, dedDirD, gg, drivingToeRadius)

        # An offset dimension controls a PERPENDICULAR distance, so the Root Length is
        # carried in that form. At Toe Extension 0 this is exactly the resolved Face Width.
        rootLengthOffset = rootLength * R / apexDed

        # --- 22. A'->G, the pinion hexagon's shaft-axis edge ------------------------
        # This line is what CREATES A' — nothing above it does — so its start is seeded at
        # the foot of the perpendicular from N onto the pinion shaft axis. It is drawn
        # HERE rather than after the front face, so the hexagon's edges are created in the
        # walk order A' -> G -> H -> C -> M -> N.
        aPrimeToG = self._constructionLine(sketch, aPrimeSeed, gSeed)
        gc.addCoincident(aPrimeToG.endSketchPoint, pointG)
        pointAprime = aPrimeToG.startSketchPoint

        # --- 23. Constrain point I with the centre point ----------------------------
        # This is what closes the figure's one remaining freedom.
        gc.addCoincident(pointI, projectedCenter)

        # --- 24-25. The tooth centres K and L. By the time K is added G and C are already
        # fixed, so a collinear here over-constrains; two point-on-line coincidents locate
        # K exactly.
        gToK = self._constructionLine(sketch, gSeed, kSeed)
        gc.addCoincident(gToK.startSketchPoint, pointG)
        pointK = gToK.endSketchPoint
        gc.addCoincident(pointK, pinionShaftAxis)
        gc.addCoincident(pointK, pinionDedLine)
        cToK = self._constructionLine(sketch, cSeed, kSeed)
        gc.addCoincident(cToK.startSketchPoint, pointC)
        gc.addCoincident(cToK.endSketchPoint, pointK)

        iToL = self._constructionLine(sketch, iSeed, lSeed)
        gc.addCoincident(iToL.startSketchPoint, pointI)
        pointL = iToL.endSketchPoint
        gc.addCoincident(pointL, drivingShaftAxis)
        gc.addCoincident(pointL, drivingDedLine)
        dToL = self._constructionLine(sketch, dSeed, lSeed)
        gc.addCoincident(dToL.startSketchPoint, pointD)
        gc.addCoincident(dToL.endSketchPoint, pointL)

        # --- 26. The tooth-centre points K' and L' ---------------------------------
        toothSpacing = self._toothSpacing_cm
        kPrimeSeed = None
        lPrimeSeed = None
        if toothSpacing > 0:
            pointKprime, kPrimeSeed, pinionToothCenterLine = self._toothSpacingLine(
                sketch, gc, pointK, kSeed, pinionDedLine, dedDirP, apex2Seed,
                virtualPitchRadiusP, toothSpacing, pointC, cSeed)
            pointLprime, lPrimeSeed, drivingToothCenterLine = self._toothSpacingLine(
                sketch, gc, pointL, lSeed, drivingDedLine, dedDirD, apex2Seed,
                virtualPitchRadiusD, toothSpacing, pointD, dSeed)
        else:
            # At Tooth Spacing 0 — the default — build NOTHING here: K' is K, L' is L, and
            # the existing C->K and D->L reference lines are reused, because a zero-length
            # dimensioned line is degenerate and one segment gets one line.
            pointKprime, pinionToothCenterLine = pointK, cToK
            pointLprime, drivingToothCenterLine = pointL, dToL

        # --- 29-30. The pinion toe line M->N, the reference M->C and the front face
        # N->A'. All three constraints on the toe line are required. The offset is
        # UNSIGNED, so a correctly built frame still admits M->N one root length on the FAR
        # side of C->H, where the toe lands outside the heel and the revolved frustum is
        # degenerate: the M seed is the only thing that holds the toe's side.
        mToN = self._constructionLine(sketch, mSeed, nSeed)
        pointM = mToN.startSketchPoint
        pointN = mToN.endSketchPoint
        gc.addCoincident(pointM, pinionRootAxis)
        gc.addParallel(mToN, cToH)
        pinionRootDim = dims.addOffsetDimension(cToH, mToN, _pt3(_mid2(mSeed, cSeed)))
        pinionRootDim.parameter.value = rootLengthOffset
        mToC = self._constructionLine(sketch, mSeed, cSeed)
        gc.addCoincident(mToC.startSketchPoint, pointM)
        gc.addCoincident(mToC.endSketchPoint, pointC)

        # N is NOT pinned to line A->Apex2 and NOT pinned to the shaft axis: N rides the
        # Toe Radius, and putting N on the axis of revolution makes the later conical split
        # fail with ASM_API_FAILED for asymmetric tooth counts. A' is the only toe-end
        # point that touches the axis, and it is a FOOT, not a corner.
        nToAprime = self._constructionLine(sketch, nSeed, aPrimeSeed)
        gc.addCoincident(nToAprime.startSketchPoint, pointN)
        gc.addCoincident(nToAprime.endSketchPoint, pointAprime)
        gc.addCoincident(pointAprime, pinionShaftAxis)
        gc.addPerpendicular(nToAprime, pinionShaftAxis)
        self._alignedDimension(sketch, nToAprime, pinionToeRadius, _mid2(nSeed, aPrimeSeed))

        # --- 31. The driving mirrors, built exactly as the pinion's -----------------
        oToP = self._constructionLine(sketch, oSeed, pSeed)
        pointO = oToP.startSketchPoint
        pointP = oToP.endSketchPoint
        gc.addCoincident(pointO, drivingRootAxis)
        gc.addParallel(oToP, dToJ)
        drivingRootDim = dims.addOffsetDimension(dToJ, oToP, _pt3(_mid2(oSeed, dSeed)))
        drivingRootDim.parameter.value = rootLengthOffset
        oToD = self._constructionLine(sketch, oSeed, dSeed)
        gc.addCoincident(oToD.startSketchPoint, pointO)
        gc.addCoincident(oToD.endSketchPoint, pointD)

        pToBprime = self._constructionLine(sketch, pSeed, bPrimeSeed)
        gc.addCoincident(pToBprime.startSketchPoint, pointP)
        pointBprime = pToBprime.endSketchPoint
        gc.addCoincident(pointBprime, drivingShaftAxis)
        gc.addPerpendicular(pToBprime, drivingShaftAxis)
        self._alignedDimension(sketch, pToBprime, drivingToeRadius, _mid2(pSeed, bPrimeSeed))

        bPrimeToI = self._constructionLine(sketch, bPrimeSeed, iSeed)
        gc.addCoincident(bPrimeToI.startSketchPoint, pointBprime)
        gc.addCoincident(bPrimeToI.endSketchPoint, pointI)

        # --- The two gates that close this step ------------------------------------
        if not sketch.isFullyConstrained:
            raise Exception(
                'Gear Profiles sketch is not fully constrained — full constraint comes '
                'from the missing constraint, never from dimensioning a driven length')

        # [BEVEL-F-SEED-HELD]: compare every named point's solved .geometry against the
        # closed-form position this step seeded it at, in the sketch's own 2-D frame with
        # no world round trip, and raise naming the FIRST point that has moved. This gate
        # is the only measure that catches all 8192 figures the seed-decided sites admit;
        # never treat the revolve's ASM_WIRE_X_AXIS as the tripwire, because several of
        # the flips build a valid-looking gear on the wrong side and reach no error.
        seedChecks = [
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
        # The list is 22 points only when Tooth Spacing is above zero; at Tooth Spacing 0
        # K' and L' are not built at all, so they are dropped and 20 are compared —
        # comparing a K' that was never created is the one way this gate can raise on a
        # correct figure.
        if toothSpacing > 0:
            seedChecks.append(("K'", pointKprime, kPrimeSeed))
        seedChecks.extend([
            ('M', pointM, mSeed),
            ('N', pointN, nSeed),
            ("A'", pointAprime, aPrimeSeed),
            ('L', pointL, lSeed),
        ])
        if toothSpacing > 0:
            seedChecks.append(("L'", pointLprime, lPrimeSeed))
        seedChecks.extend([
            ('O', pointO, oSeed),
            ('P', pointP, pSeed),
            ("B'", pointBprime, bPrimeSeed),
        ])

        for name, point, seed in seedChecks:
            solved = _xy(point.geometry)
            if _norm2(_sub2(solved, seed)) > _SEED_TOLERANCE_CM:
                raise Exception(
                    'Gear Profiles: point {} solved to ({:.6f}, {:.6f}) cm but was seeded '
                    'at ({:.6f}, {:.6f}) cm, past the {:.4f} mm tolerance — the figure has '
                    'flipped at this site'.format(
                        name, solved[0], solved[1], seed[0], seed[1],
                        to_mm(_SEED_TOLERANCE_CM)))

        futil.log(
            'Bevel Gear: §2 lattice fully constrained, {} seeded points held'.format(
                len(seedChecks)))

        # The §2 Apex sketch point is the loft's degenerate point-section (S14).
        self._apexSketchPoint = pointApex

        # --- The per-gear context dictionaries. These key strings are reproduced surface:
        # never rename a key, split the dict, wrap it in a class, or carry one of these
        # values under a different shape. Both gears carry the same 18 keys and no others.
        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'gamma': gp,
            'pitchDiameter_cm': ppd,
            'toothCenterPoint': pointKprime,
            'toothCenterRefLine': pinionToothCenterLine,
            'hexVertices': [pointAprime, pointG, pointH, pointC, pointM, pointN],
            'toeEdgePoints': (pointM, pointN),
            'heelEdgePoints': (pointC, pointH),
            'boreDiameter_cm': pinionBore,
            'toothPlane': None,
            'toothSketch': None,
            'toothEmbedded': None,
            'toothAxis': None,
            'gearOccurrence': None,
            'profileSketch': None,
            'shaftAxisEdge': None,
            'gearBody': None,
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'gamma': gg,
            'pitchDiameter_cm': dpd,
            'toothCenterPoint': pointLprime,
            'toothCenterRefLine': drivingToothCenterLine,
            'hexVertices': [pointBprime, pointI, pointJ, pointD, pointO, pointP],
            'toeEdgePoints': (pointO, pointP),
            'heelEdgePoints': (pointD, pointJ),
            'boreDiameter_cm': drivingBore,
            'toothPlane': None,
            'toothSketch': None,
            'toothEmbedded': None,
            'toothAxis': None,
            'gearOccurrence': None,
            'profileSketch': None,
            'shaftAxisEdge': None,
            'gearBody': None,
        }

        # Pinion first, driving second, and profile and body INTERLEAVED per gear.
        for ctx in (pinionCtx, drivingCtx):
            self._buildVirtualSpurProfile(ctx)
            self._createGearBody(ctx)

    # -------------------------------------------------------------------------------
    # S08-S12 — this gear's tooth plane, tooth sketch, tooth axis, gear component and
    # Profile sketch. Once per gear, pinion first.
    # -------------------------------------------------------------------------------
    def _buildVirtualSpurProfile(self, ctx):
        label = ctx['label']
        component: adsk.fusion.Component = self.designComponent

        # --- S08. {gearLabel} Plane: through this gear's tooth-centre reference line,
        # perpendicular to the Gear Profiles sketch plane. The sketch line is passed
        # DIRECTLY; never through Path.create ([PB-CONSTRUCTION-PLANES]).
        toothPlane: adsk.fusion.ConstructionPlane = solids.plane_by_angle(
            component, ctx['toothCenterRefLine'], self._gearProfilesPlane, 90)
        toothPlane.name = '{} Plane'.format(label)
        ctx['toothPlane'] = toothPlane

        # --- S09. {gearLabel} Tooth -------------------------------------------------
        toothSketch: adsk.fusion.Sketch = component.sketches.add(toothPlane)
        toothSketch.name = '{} Tooth'.format(label)

        module = self._module_mm
        gamma = ctx['gamma']
        # Step 1 — the back-cone (Tredgold) figures, from the closed form, never by
        # measuring Apex2->K'. The * 10 converts the stashed internal-cm pitch diameter to
        # millimetres; skipping it makes the count about 10x wrong.
        virtualPitchRadius_mm = (ctx['pitchDiameter_cm'] * 10 / 2) / math.cos(gamma)
        # A REAL number, NEVER rounded — not floored, not ceiled, not cast to an int. The
        # Tredgold construction puts the equivalent spur gear's pitch radius exactly at
        # the back-cone distance, and z_v = z / cos(gamma) is real in every published form.
        virtualTeeth = 2 * virtualPitchRadius_mm / module
        # The root circle is drawn one root sink INSIDE the dedendum corner, so the
        # Combine-Join meets the gear body across the root rather than along one line.
        rootSink_mm = 0.05 * 2.25 * module

        # Step 3 — the framework proxy and the borrowed spur drawer. No local proxy or
        # value-wrapper class: the proxy's own defaults already match bevel (pressure
        # angle 20 degrees, 15 involute steps) and it serves each value in internal cm.
        proxy = VirtualSpurProxy(
            module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        # The 180 degree rotation is delivered through draw()'s angle argument, not a
        # post-hoc move or sketch rotation.
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))

        # An OUTPUT the spur generator writes during draw(), and the deterministic
        # selector for the tooth loop's line count (S14).
        ctx['toothEmbedded'] = proxy._lastToothEmbedded
        ctx['toothSketch'] = toothSketch

        # Do NOT hard-gate this sketch ([PB-LOGGING]). The tooth sketches are exempt from
        # the full-constraint gate, and ONLY because the drawer labels each of the four
        # circles with along-path sketch text, which holds a DOF ([PB-TEXT-HOLDS-DOF]).
        # That exemption covers the labels and nothing else. The reading is also not
        # stable between runs, which is why the instruction is to log either way.
        futil.log(
            '{} Tooth: virtual teeth {:.4f}, virtual pitch radius {:.4f} mm, root sink '
            '{:.4f} mm, embedded={}, fully constrained={}'.format(
                label, virtualTeeth, virtualPitchRadius_mm, rootSink_mm,
                ctx['toothEmbedded'], toothSketch.isFullyConstrained))

        # --- S10. {gearLabel} Tooth Axis --------------------------------------------
        # setByPerpendicularAtPoint would need a BRepFace that does not exist here, so the
        # axis is the intersection of two planes ([PB-CONSTRUCTION-AXES]).
        helperInput: adsk.fusion.ConstructionPlaneInput = (
            component.constructionPlanes.createInput())
        helperInput.setByDistanceOnPath(
            ctx['toothCenterRefLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane = component.constructionPlanes.add(helperInput)

        axisInput: adsk.fusion.ConstructionAxisInput = (
            component.constructionAxes.createInput())
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = component.constructionAxes.add(axisInput)
        toothAxis.name = '{} Tooth Axis'.format(label)
        # No step reads toothAxis back; Cleanup hides the axis by entity kind. The key is
        # carried so that a regen which stashes the axis is not read as having invented it.
        ctx['toothAxis'] = toothAxis

        # --- S11. {gearLabel} Gear component, a child of Bevel Gear — the same component
        # that owns Design, and NOT the user's Parent Component.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = '{} Gear'.format(label)
        ctx['gearOccurrence'] = gearOccurrence

        # --- S12. {gearLabel} Profile: a FRESH sketch on the axial Gear Profiles plane,
        # holding exactly this one hexagon loop.
        profileSketch: adsk.fusion.Sketch = component.sketches.add(
            self._gearProfilesPlane)
        profileSketch.name = '{} Profile'.format(label)

        # [PB-PROJECT-NOT-FIXED] recreate-share-fix, and the order is load-bearing:
        # recreate the six §2 vertices at their exact world-mapped positions, draw the
        # closed hexagon SHARING those points, and only THEN fix the lines' endpoints.
        # Fixing a bare point before it is consumed as a line endpoint does not leave the
        # sketch fully constrained. modelToSketchSpace is a point-transforming METHOD, not
        # a matrix ([PB-SPACE-METHODS]).
        verts = [profileSketch.sketchPoints.add(
            profileSketch.modelToSketchSpace(src.worldGeometry))
            for src in ctx['hexVertices']]
        hexLines = []
        for index in range(len(verts)):
            hexLines.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(
                verts[index], verts[(index + 1) % len(verts)]))
        for drawn in hexLines:
            drawn.startSketchPoint.isFixed = True
            drawn.endSketchPoint.isFixed = True

        ctx['profileSketch'] = profileSketch
        # The hexagon's FIRST edge is the gear's shaft axis for the revolve, the pattern,
        # the bore plane AND the meshing rotation. Fixed endpoints are what give it a
        # trustworthy worldGeometry ([PB-WORLDGEO-CONSTRAINED]).
        ctx['shaftAxisEdge'] = hexLines[0]

        if not profileSketch.isFullyConstrained:
            raise Exception(
                '{} Profile sketch is not fully constrained'.format(label))

    # -------------------------------------------------------------------------------
    # S13-S27 — this gear's solid body.
    # -------------------------------------------------------------------------------
    def _createGearBody(self, ctx):
        label = ctx['label']
        component: adsk.fusion.Component = self.designComponent
        features: adsk.fusion.Features = component.features
        shaftAxisEdge: adsk.fusion.SketchLine = ctx['shaftAxisEdge']

        # --- S13. Revolve the Gear Body --------------------------------------------
        # The Profile sketch holds exactly one hexagon loop, so take its single profile
        # and do not filter ([PB-SINGLE-PROFILE]): a curve-type filter has spuriously
        # rejected a valid all-line loop and made the revolve fail.
        profileSketch: adsk.fusion.Sketch = ctx['profileSketch']
        profile = profileSketch.profiles.item(0)
        revolveInput: adsk.fusion.RevolveFeatureInput = features.revolveFeatures.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = features.revolveFeatures.add(revolveInput)
        gearBody: adsk.fusion.BRepBody = revolve.bodies.item(0)
        gearBody.name = '{} Gear Body'.format(label)
        ctx['gearBody'] = gearBody

        # --- S14. Loft the Tooth Body ----------------------------------------------
        # The line count is DETERMINED by the embedded flag read back off the proxy in
        # S09, never guessed and never accepted either way: an unrelated loop can carry
        # the same 2 NURBS and 2 arcs with the other line count, and lofting it fails with
        # ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY.
        wantLines = 0 if ctx['toothEmbedded'] else 2
        toothProfile = find_profile_by_curve_counts(
            ctx['toothSketch'], nurbs=2, arcs=2, lines=wantLines)

        loftInput: adsk.fusion.LoftFeatureInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The §2 Apex SKETCH point is used directly as the degenerate point-section: a
        # ConstructionPoint needs an active component and Design is never activated, while
        # a SketchPoint works as a loft point-section ([PB-CONSTRUCTION-NEEDS-ACTIVE],
        # [PB-LOFT]). Sections are added in loft order.
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = features.loftFeatures.add(loftInput)
        toothBody: adsk.fusion.BRepBody = loft.bodies.item(0)
        toothBody.name = '{} Tooth Body'.format(label)

        # --- The caller hand-off into the tooth-body hook (S15, or S16-S21 when psi > 0).
        # The toe edge is M->N / O->P and the heel edge C->H / D->J — two DIFFERENT edges,
        # never the two endpoints of one. toeConeWorld / heelConeWorld are the dedendum
        # corners M/O and C/D, and heelConeWorld is NEVER H/J, which lie one Module beyond
        # C/D off the root cone element and skew coneVec.
        toeStart, toeEnd = ctx['toeEdgePoints']
        heelStart, heelEnd = ctx['heelEdgePoints']
        toeMid = _mid3(toeStart.worldGeometry, toeEnd.worldGeometry)
        heelMid = _mid3(heelStart.worldGeometry, heelEnd.worldGeometry)
        toeConeWorld = toeStart.worldGeometry
        heelConeWorld = heelStart.worldGeometry
        apexWorld = self._apexSketchPoint.worldGeometry

        toothPiece: adsk.fusion.BRepBody = self._transformToothBody(
            toothBody, gearBody, toeMid, heelMid, toeConeWorld, heelConeWorld,
            apexWorld, shaftAxisEdge, ctx['toothPlane'], ctx['gamma'],
            ctx['teeth'], label)

        # --- S22. Circular pattern around the SHAFT-AXIS EDGE, never the §2 line ----
        seedCollection = adsk.core.ObjectCollection.create()
        seedCollection.add(toothPiece)
        patternInput: adsk.fusion.CircularPatternFeatureInput = (
            features.circularPatternFeatures.createInput(
                seedCollection, shaftAxisEdge))
        # Pin all three explicitly rather than relying on Fusion's defaults
        # ([PB-CIRCULAR-PATTERN]).
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = features.circularPatternFeatures.add(patternInput)

        # The pattern's bodies collection already includes the seed plus the copies, so do
        # not re-add the seed; copy them into a fresh ObjectCollection, which is what the
        # Combine accepts ([PB-PATTERN-BODIES]).
        toolBodies = adsk.core.ObjectCollection.create()
        for index in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(index))

        # --- S23. A single Combine-Join, the Gear Body as target -------------------
        combineInput: adsk.fusion.CombineFeatureInput = (
            features.combineFeatures.createInput(gearBody, toolBodies))
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        features.combineFeatures.add(combineInput)

        # --- S24-S25. The bore, skipped entirely when Enable Bore is unchecked ------
        if self._boreEnable:
            self._buildBore(ctx)

        # --- S26. Meshing rotation, in Design and BEFORE the body is moved out ------
        if label == 'Driving':
            # Half a tooth pitch, so a driving valley sits where the pinion tooth crosses.
            solids.rotate_body_about_edge(
                component, gearBody, shaftAxisEdge,
                math.radians(180.0 / ctx['teeth']))
        else:
            # 0 by default: the mid-face section is unrotated and already meshes. A zero
            # angle is a no-op, not a move, and rotate_body_about_edge absorbs it.
            solids.rotate_body_about_edge(
                component, gearBody, shaftAxisEdge,
                self._pinionMeshPhase(ctx['teeth']))

        # --- S27. Move the finished body into its own gear component ---------------
        # moveToComponent preserves world position and needs no activation.
        gearBody.moveToComponent(ctx['gearOccurrence'])
        futil.log('{} Gear: body moved into its own component'.format(label))

    def _pinionMeshPhase(self, pinionTeeth):
        # The pinion's extra rotation about its own shaft axis, in radians.
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # -------------------------------------------------------------------------------
    # S24-S25 — the bore sketch and its through cut.
    # -------------------------------------------------------------------------------
    def _buildBore(self, ctx):
        label = ctx['label']
        component: adsk.fusion.Component = self.designComponent
        features: adsk.fusion.Features = component.features

        # The bore plane is normal to the shaft at its START. Pass the in-sketch edge
        # directly, not the §2 construction line, and never through Path.create.
        planeInput: adsk.fusion.ConstructionPlaneInput = (
            component.constructionPlanes.createInput())
        planeInput.setByDistanceOnPath(
            ctx['shaftAxisEdge'], adsk.core.ValueInput.createByReal(0.0))
        borePlane = component.constructionPlanes.add(planeInput)

        boreSketch: adsk.fusion.Sketch = component.sketches.add(borePlane)
        boreSketch.name = '{} Bore'.format(label)

        # The plane is rooted at the shaft start, so the sketch origin is on the axis.
        diameter = ctx['boreDiameter_cm']
        radius = diameter / 2
        circle: adsk.fusion.SketchCircle = (
            boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), radius))
        # [PB-CIRCLE-CENTER]: a circle's centre is free even when created at (0,0,0), and
        # addCoincident to the sketch origin has thrown VCS_SKETCH_SOLVING_FAILED on
        # exactly this kind of setByDistanceOnPath plane. isFixed is the reliable pin.
        circle.centerSketchPoint.isFixed = True
        # [PB-RADIAL-DIM]: the text point must be off-centre, on or near the curve.
        diameterDim = boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, 0, 0))
        diameterDim.parameter.value = diameter

        if not boreSketch.isFullyConstrained:
            raise Exception('{} Bore sketch is not fully constrained'.format(label))

        # The diameter is the value already resolved AND already bounded in S07; it is not
        # re-derived here, or the cap is lost and the bore deletes the body's back face.
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = features.extrudeFeatures.createInput(
            boreSketch.profiles.item(0),
            adsk.fusion.FeatureOperations.CutFeatureOperation)
        # [PB-THROUGH-CUT]: isFullLength=False means the distance is the half-length PER
        # SIDE, and 2 x Cone Distance is generously past any face width. No taper argument.
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [ctx['gearBody']]
        features.extrudeFeatures.add(extrudeInput)

    # -------------------------------------------------------------------------------
    # A slab's end faces. Defined precisely ([S19]): the heel face is the face whose
    # centroid has the GREATEST cone distance, searched across ALL of the slab's faces
    # with NO surface-type filter; its toe-side face is the least-centroid one. A type
    # filter can pick the wrong face or miss the cut face, which makes the loft fail with
    # ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY.
    # -------------------------------------------------------------------------------
    @staticmethod
    def _endFace(body: adsk.fusion.BRepBody, apexWorld, coneVec,
                 greatest) -> adsk.fusion.BRepFace:
        best = None
        bestKey = None
        for face in body.faces:
            key = _dot3(_vec3(face.centroid, apexWorld), coneVec)
            if bestKey is None or (key > bestKey if greatest else key < bestKey):
                best, bestKey = face, key
        if best is None:
            raise Exception(
                'Bevel Gear: a tooth slab reported no faces, so its {} face cannot be '
                'found'.format('heel' if greatest else 'toe'))
        return best

    # -------------------------------------------------------------------------------
    # S15-S21 — the tooth-body hook.
    # -------------------------------------------------------------------------------
    def _transformToothBody(self, toothBody: adsk.fusion.BRepBody,
                            gearBody: adsk.fusion.BRepBody,
                            toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                            toeConeWorld: adsk.core.Point3D,
                            heelConeWorld: adsk.core.Point3D,
                            apexWorld: adsk.core.Point3D,
                            shaftAxisEdge: adsk.fusion.SketchLine,
                            parentToothPlane: adsk.fusion.ConstructionPlane,
                            gamma, teethNumber, gearLabel):
        component: adsk.fusion.Component = self.designComponent
        features: adsk.fusion.Features = component.features

        # S15 — the psi = 0 path: a straight bevel is byte-for-byte the prior behaviour
        # and every spiral input is ignored. The cutting TOOLS are ConeSurfaceType faces
        # of the revolved Gear Body and the TARGET is the lofted Tooth Body, which has no
        # cone faces of its own. Do not re-implement the cut machinery.
        if self._spiralAngle_rad <= 0:
            return solids.cut_conical_ends(
                component, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        # --- S16 step A. The frame, every quantity read in WORLD space
        # ([PB-WORLD-FRAME]): mixing a sketch-local curve with a world axis is valid
        # Python that silently returns wrong numbers, and a wrong spiral-twist magnitude is
        # exactly the failure it produces.
        axisStart = shaftAxisEdge.startSketchPoint.worldGeometry
        axisEnd = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = normalize(_vec3(axisEnd, axisStart))

        # Before building coneVec, fix a swapped toe and heel. A negative span silently
        # inverts the entire spiral frame — cutter-arc direction, slice direction and
        # per-segment twist all flip — and the gear comes out wrong with no error.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld
            futil.log('{}: toe and heel arrived swapped; corrected'.format(gearLabel))

        coneVec = normalize(_vec3(heelConeWorld, apexWorld))
        v = normalize(_cross3(axisDir, coneVec))
        # tpNormal completes the frame and NOTHING consumes it: step D removed the
        # projection that once used it, so it is computed and left unread.
        tpNormal = normalize(_cross3(coneVec, v))

        def cone_distance(point):
            # A point's distance from the apex measured along the cone element.
            return _dot3(_vec3(point, apexWorld), coneVec)

        # --- S17 step B. The cutter arc, in the tangent-plane 2-D frame with the origin
        # at the apex, x = coneVec (so a point's x is its cone distance) and y = v.
        rToe = cone_distance(toeMid)
        rHeel = cone_distance(heelMid)
        rMean = 0.5 * (rToe + rHeel)
        span = rHeel - rToe

        cutterRadius = self._cutterRadius_cm if self._cutterRadius_cm != 0 else rMean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            # The pair meshes with opposite hands.
            handSign = -handSign

        # The hand sign goes on the cos / Cy term, NOT the sin / Cx term: opposite hands
        # mirror the cutter centre across the cone element y = 0. Putting handSign on Cx
        # mirrors about x = R_mean instead, a different curve that gives the two gears
        # unequal twist where equal teeth must come out as exact mirror images.
        cx = rMean - cutterRadius * math.sin(self._spiralAngle_rad)
        cy = handSign * cutterRadius * math.cos(self._spiralAngle_rad)

        # The endpoints are taken a hair PAST the face so the kept arc reaches cleanly
        # past the end trims.
        rLo = rToe - 0.06 * span
        rHi = rHeel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(rLo, cx, cy, cutterRadius, rMean, 0)
        heel2d = solids.circle_intersect_nearest(rHi, cx, cy, cutterRadius, rMean, 0)

        futil.log(
            '{} ({} teeth): spiral frame R_toe={:.4f} R_heel={:.4f} span={:.4f} cm, '
            'cutter radius {:.4f} cm, hand sign {:+.0f}, frame normal '
            '({:.4f}, {:.4f}, {:.4f}), toe cone at {:.4f} cm'.format(
                gearLabel, teethNumber, rToe, rHeel, span, cutterRadius, handSign,
                tpNormal.x, tpNormal.y, tpNormal.z, cone_distance(toeConeWorld)))

        # --- S16, second half. The {gear} Cone Element sketch and the {gear} Trace Plane.
        # The world Point3Ds these two sketches are built from are passed DIRECTLY into the
        # sketch calls, with NO modelToSketchSpace conversion. That is deliberate: the
        # trace sketch is construction and reference only and no downstream feature ever
        # consumes it, because the twist is computed analytically in S19. If a later
        # revision ever makes a feature consume the trace sketch or the Trace Plane, this
        # shortcut stops being safe and both sketches need modelToSketchSpace on every
        # point. Both sketches are exempt from the full-constraint gate.
        coneSketch: adsk.fusion.Sketch = component.sketches.add(self._gearProfilesPlane)
        coneSketch.name = '{} Cone Element'.format(gearLabel)
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, solids.combine_point(apexWorld, rHeel, coneVec))
        coneElementLine.isConstruction = True

        tracePlane = solids.plane_by_angle(
            component, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = '{} Trace Plane'.format(gearLabel)

        # --- S17. The {gear} 2D Tooth Trace sketch, holding the genuine cutter arc.
        # There is no 3-D projection: no projectToSurface, no root-cone-face search and no
        # 3-D trace sketch. For unequal-ratio pairs the old projection came back as
        # multiple disjoint fragments and the pinion came out grossly under-twisted.
        traceSketch: adsk.fusion.Sketch = component.sketches.add(tracePlane)
        traceSketch.name = '{} 2D Tooth Trace'.format(gearLabel)

        def tanW(px, py):
            return solids.combine_point(apexWorld, px, coneVec, py, v)

        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            tanW(cx, cy), cutterRadius)
        cutterCircle.isConstruction = True
        # [PB-CIRCLE-CENTER]: the centre is free even here, and a coincident to the sketch
        # origin has thrown VCS_SKETCH_SOLVING_FAILED on exactly this kind of plane.
        cutterCircle.centerSketchPoint.isFixed = True
        # [PB-RADIAL-DIM]: a text point at the curve's centre is rejected.
        cutterDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, tanW(cx + cutterRadius, cy))
        cutterDim.parameter.value = 2 * cutterRadius

        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            tanW(toe2d[0], toe2d[1]), tanW(rMean, 0), tanW(heel2d[0], heel2d[1]))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        # A radius dimension of r_c, so it is the genuine cutter circle and not a
        # look-alike spline.
        arcDim = traceSketch.sketchDimensions.addRadialDimension(
            traceArc, tanW(rMean, 0))
        arcDim.parameter.value = cutterRadius
        # This sketch is deliberately left with free DOF and must not be gated.

        # --- S18. Slice the tooth into slabs and drop the apex scrap ----------------
        # The slice planes are NOT perpendicular to the cone element: the helper offsets
        # the parent plane with setByOffset, the sign test below reads that plane's own
        # normal, and the tooth is lofted to the profile drawn in the parent plane, so the
        # heel-most slab's heel face IS the parent plane and a consistent family has to
        # contain it. A build that follows "perpendicular to the cone element" is wrong
        # and silent.
        planeGeometry = parentToothPlane.geometry
        planeNormal = planeGeometry.normal
        planeOrigin = planeGeometry.origin
        # The parent plane's normal points opposite ways for the two gears, so pick the
        # sign that moves the planes apex-ward.
        sign = 1.0 if _dot3(_vec3(apexWorld, planeOrigin), planeNormal) > 0 else -1.0

        def slice_once(body, signValue):
            # A fixed scheme of exactly eight planes, a count that is not
            # user-configurable.
            offsets = [signValue * (k + 1) * span / 6 for k in range(8)]
            return solids.slice_body_by_offset_planes(
                component, body, parentToothPlane, offsets)

        pieces = slice_once(toothBody, sign)
        if len(pieces) < 2:
            # The slice MUST actually split the tooth: retry the whole cut once with the
            # opposite sign ([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]).
            sign = -sign
            pieces = slice_once(pieces[0], sign)
        if len(pieces) < 2:
            raise Exception(
                '{}: the slice left the tooth in {} piece(s) after trying both offset '
                'signs (final sign {:+.0f}, span {:.6f} cm) — the parent plane sits '
                'outside the tooth\'s span'.format(
                    gearLabel, len(pieces), sign, span))

        # Order the pieces and drop the scrap. Re-slice the list FIRST and delete after,
        # so the list never holds a deleted body.
        pieces.sort(key=lambda body: cone_distance(
            body.physicalProperties.centerOfMass))
        scrap = pieces[0]
        segments = pieces[1:]
        features.removeFeatures.add(scrap)
        if not segments:
            raise Exception(
                '{}: dropping the apex scrap left no segments, so the slice '
                'failed'.format(gearLabel))

        # --- S19. Twist the slabs about the SHAFT AXIS, axisDir through apexWorld,
        # centred on R_mean so the mid-face section stays unrotated.
        #
        # The total toe->heel twist comes from the conjugate crown-gear generation law,
        # computed analytically, with no projection and no curve sampling. `gamma` here is
        # this gear's PITCH cone angle and NOT acos(coneVec . axisDir), which is the root
        # cone angle, smaller by the dedendum angle and worth a twist about 1.15x too
        # large. The two members of a meshing pair legitimately get DIFFERENT twists: same
        # cutter and same psi, but gamma differs, so 1 / sin(gamma) differs.
        phiCrown = (math.atan2(heel2d[1], heel2d[0])
                    - math.atan2(toe2d[1], toe2d[0]))
        total = abs(phiCrown) / math.sin(gamma)
        futil.log(
            '{}: phi_crown {:.6f} rad, total twist {:.6f} rad on a pitch cone angle of '
            '{:.4f} deg'.format(gearLabel, phiCrown, total, math.degrees(gamma)))

        axisVector = adsk.core.Vector3D.create(axisDir.x, axisDir.y, axisDir.z)
        for segment in segments:
            # Key the twist on the HEEL-FACE cone distance, not on the segment's centroid:
            # the loft samples each segment's heel face, so that face is what must land at
            # the right azimuth. Centroid-keying leaves the loft's mid-face section rotated
            # by half a segment and the mid-faces overlap.
            heelFace = self._endFace(segment, apexWorld, coneVec, True)
            heelDistance = cone_distance(heelFace.centroid)
            ang = -handSign * total * (rMean - heelDistance) / span
            if ang == 0:
                # A zero angle is a no-op, not a move ([PB-MOVE-ROTATE]): Fusion refuses
                # the identity transform with `RuntimeError: 3 : invalid transform`.
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisVector, apexWorld)
            bodyCollection = adsk.core.ObjectCollection.create()
            bodyCollection.add(segment)
            moveInput: adsk.fusion.MoveFeatureInput = (
                features.moveFeatures.createInput2(bodyCollection))
            moveInput.defineAsFreeMove(matrix)
            features.moveFeatures.add(moveInput)

        # --- S20. Lengthwise crown -------------------------------------------------
        # Each segment's heel-distance fraction is RECOMPUTED HERE, AFTER the twist has
        # moved the slabs; pre-twist values are not reused.
        crownEntries = []
        for segment in segments:
            heelFace = self._endFace(segment, apexWorld, coneVec, True)
            crownEntries.append((cone_distance(heelFace.centroid), segment))
        # "Outermost (heel) segment" is the one with the GREATEST post-twist heel-face
        # distance; it is held FULL, so its heel face stays the loft's heel end and the
        # heel cone in S21 trims it flush with the gear base.
        crownEntries.sort(key=lambda entry: entry[0])
        outermostSegment = crownEntries[-1][1]

        # scaleFeatures is the ONE exception to never-activate: it needs the Design
        # occurrence as the active edit target. The root is re-activated through
        # Design.activateRootComponent — NEVER design.rootComponent.activate(), because a
        # Component has no activate method and raises AttributeError.
        self.designOccurrence.activate()
        try:
            for heelDistance, segment in crownEntries:
                if segment is outermostSegment:
                    continue
                u = (rHeel - heelDistance) / span
                # Key the relief on the monotonic u, NEVER on |ang|, which is symmetric
                # about the mid-face and makes the heel-adjacent slab the most relieved
                # one, reversing the heel->toe taper.
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2) * u
                if factor <= 0:
                    raise Exception(
                        '{}: the crown factor came out {:.6f} at u = {:.6f}; a body is '
                        'never scaled by a non-positive factor'.format(
                            gearLabel, factor, u))

                # Anchor the scale on the heel face's ROOT edge, not its centroid, or the
                # crowned tooth lifts off the gear base: a uniform scale keeps every line
                # through the base point invariant, so a root anchor keeps the root edge on
                # the seating cone while the tip is relieved. The two vertices with the
                # SMALLEST perpendicular distance to the shaft axis are the root corners,
                # the tip corners being farthest from the axis.
                heelFace: adsk.fusion.BRepFace = self._endFace(
                    segment, apexWorld, coneVec, True)
                vertexPoints = [heelFace.vertices.item(i).geometry
                                for i in range(heelFace.vertices.count)]
                if len(vertexPoints) < 2:
                    raise Exception(
                        '{}: the heel face carries {} vertices, too few to find its root '
                        'edge'.format(gearLabel, len(vertexPoints)))
                vertexPoints.sort(
                    key=lambda point: _axis_distance3(point, apexWorld, axisDir))
                rootMid = _mid3(vertexPoints[0], vertexPoints[1])

                # The scale base must be a sketch point or a BRep vertex
                # ([PB-CONSTRUCTION-NEEDS-ACTIVE]). The heel face is a planar cut, so the
                # root-corner midpoint lies on it.
                baseSketch: adsk.fusion.Sketch = component.sketches.add(heelFace)
                baseSketch.name = '{} Crown Base'.format(gearLabel)
                basePoint = baseSketch.sketchPoints.add(
                    baseSketch.modelToSketchSpace(rootMid))

                inputEntities = adsk.core.ObjectCollection.create()
                inputEntities.add(segment)
                scaleInput = features.scaleFeatures.createInput(
                    inputEntities, basePoint,
                    adsk.core.ValueInput.createByReal(factor))
                features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # --- S21. Loft the spiral tooth and trim it flush ---------------------------
        # Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and
        # the crown. The twist rotates each slab about the shaft axis, and for high-twist
        # unequal-ratio pairs that rotation changes the slabs' along-cone order enough to
        # reorder adjacent slabs; lofting in the stale order assembles the cross-sections
        # out of sequence and the two gears interfere. For equal or low-twist pairs the two
        # orders coincide, which is why 31/31 looked fine while 31/17 distorted.
        ordered = []
        for segment in segments:
            heelFace = self._endFace(segment, apexWorld, coneVec, True)
            ordered.append((cone_distance(heelFace.centroid), segment))
        ordered.sort(key=lambda entry: entry[0])

        loftInput: adsk.fusion.LoftFeatureInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The toe-most segment is ordered[0], and its apex-side (toe-facing) face is added
        # FIRST, to push the loft past the toe cone so the toe trim bites.
        loftInput.loftSections.add(
            self._endFace(ordered[0][1], apexWorld, coneVec, False))
        for _, segment in ordered:
            loftInput.loftSections.add(
                self._endFace(segment, apexWorld, coneVec, True))
        loft = features.loftFeatures.add(loftInput)
        curvedTooth: adsk.fusion.BRepBody = loft.bodies.item(0)
        curvedTooth.name = '{} Spiral Tooth'.format(gearLabel)

        # The loft has captured their faces, so the scaffolding goes.
        for _, segment in ordered:
            features.removeFeatures.add(segment)

        # The same toe-then-heel two-cone trim the straight tooth takes in S15, so the
        # curved tooth's ends sit flush on the gear base.
        return solids.cut_conical_ends(
            component, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
