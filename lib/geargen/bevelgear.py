"""Bevel gear pair generator, emitted from the compiled step list spec/bevelgear/steps.md
(S01-S31).

The geometry is transliterated from proof/bevelgear/ (bevel_test.go, lattice_test.go,
gearprofiles_test.go, tooth_test.go, sketches_test.go, solids_test.go, bodies_test.go,
spiral_test.go) for the steps the step list tags [GO]; nothing here is re-derived.

Units. Fusion's internal units are centimetres for length and radians for angle, and
`evaluateExpression` always returns them whatever unit string it is handed
([PB-EVAL-EXPRESSION]). Module alone is read with the empty unit string, so it comes back as a
raw number that means MILLIMETRES; every length derived from it is `to_cm`-converted before it
touches geometry. Bevel registers no Fusion user parameters and writes every value numerically
([PB-PRECOMPUTED-MODE]).
"""

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .base import Generator, get_boolean, get_selection
from .misc import get_design, to_cm, to_mm
from .solids import (circle_intersect_nearest, combine_point, cut_conical_ends,
                     hide_construction_geometry, plane_by_angle, rotate_body_about_edge,
                     slice_body_by_offset_planes)
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy
from .utilities import find_profile_by_curve_counts, get_normal

# Dialog input ids (S01), in the dialog's own display order.
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

# The two Hand of Spiral list items (S01 row 16).
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# The fixed slice scheme of S18: eight cut planes, not user-configurable.
_SLICE_PLANES = 8

# How far past the face the trace arc's ends are taken, as a fraction of the span (S17).
_END_RELIEF = 0.06


# --- small 2-D helpers for the §2 lattice, in the sketch's own frame ----------------------

def _xy(p):
    """The (x, y) of a raw 2-D tuple/list OR of an object carrying .x/.y.

    S07 feeds the same helpers both seed tuples and solved `.geometry` points, so this must
    take either ([PB-POINT-HELPER])."""
    if isinstance(p, (tuple, list)):
        return (p[0], p[1])
    return (p.x, p.y)


def _p3(p):
    """A Point3D in the sketch plane from a 2-D tuple or an object with .x/.y."""
    x, y = _xy(p)
    return adsk.core.Point3D.create(x, y, 0)


def _vadd(a, b):
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return (ax + bx, ay + by)


def _vsub(a, b):
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return (ax - bx, ay - by)


def _vscale(a, k):
    ax, ay = _xy(a)
    return (ax * k, ay * k)


def _vdot(a, b):
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return ax * bx + ay * by


def _vcross(a, b):
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return ax * by - ay * bx


def _vlen(a):
    ax, ay = _xy(a)
    return math.hypot(ax, ay)


def _vunit(a):
    length = _vlen(a)
    if length == 0:
        return (0.0, 0.0)
    ax, ay = _xy(a)
    return (ax / length, ay / length)


def _vperp(a):
    """The left normal of a: the in-plane perpendicular, before its sign is chosen."""
    ax, ay = _xy(a)
    return (-ay, ax)


def _vrot(a, angle):
    ax, ay = _xy(a)
    sa, ca = math.sin(angle), math.cos(angle)
    return (ax * ca - ay * sa, ax * sa + ay * ca)


def _vtoward(direction, toward):
    """The unit perpendicular of `direction` that points to the same side as `toward`."""
    perp = _vperp(direction)
    if _vdot(perp, toward) < 0:
        perp = _vscale(perp, -1)
    return _vunit(perp)


def _line_intersect(p0, d0, p1, d1):
    """Where the line through p0 along d0 meets the line through p1 along d1."""
    den = _vcross(d0, d1)
    t = _vcross(_vsub(p1, p0), d1) / den
    return _vadd(p0, _vscale(d0, t))


def _toe_edge(corner, hat, apex, shaftPoint, apex2, face):
    """One gear's toe line: the dedendum line through `corner` offset toward the apex by
    `face`, its near end on the root axis and its far end on the drop from the shaft."""
    toward = _vunit(_vsub(apex, corner))
    normal = _vtoward(hat, toward)
    base = _vadd(corner, _vscale(normal, face))
    near = _line_intersect(base, hat, apex, _vunit(_vsub(corner, apex)))
    far = _line_intersect(base, hat, shaftPoint, _vunit(_vsub(apex2, shaftPoint)))
    return near, far


# --- small world-space helpers ------------------------------------------------------------

def _wvec(fromPoint: adsk.core.Point3D,
          toPoint: adsk.core.Point3D) -> adsk.core.Vector3D:
    return fromPoint.vectorTo(toPoint)


def _wunit(vector: adsk.core.Vector3D) -> adsk.core.Vector3D:
    out: adsk.core.Vector3D = vector.copy()
    out.normalize()
    return out


def _wmid(a: adsk.core.Point3D, b: adsk.core.Point3D) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


def _cone_distance(point: adsk.core.Point3D, apex: adsk.core.Point3D,
                   coneVec: adsk.core.Vector3D) -> float:
    """A world point's cone distance: its offset from the apex projected on the cone
    element ([PB-WORLD-FRAME] — every term of this is world geometry)."""
    offset: adsk.core.Vector3D = apex.vectorTo(point)
    return offset.dotProduct(coneVec)


def _axis_distance(point: adsk.core.Point3D, apex: adsk.core.Point3D,
                   axisDir: adsk.core.Vector3D) -> float:
    """A world point's perpendicular distance from the shaft axis through the apex."""
    offset: adsk.core.Vector3D = apex.vectorTo(point)
    along = offset.dotProduct(axisDir)
    return math.sqrt(max(offset.length * offset.length - along * along, 0.0))


class _GearSide:
    """Everything one gear of the pair reads out of the §2 lattice.

    `vertices` is the profile hexagon in draw order, A-G-H-C-M-N for the pinion and
    B-I-J-D-O-P for the driving gear (S11), so index 3 is the heel dedendum corner, index 2
    its far end, and indices 4 and 5 the toe edge."""

    def __init__(self, label, isPinion, teeth, pitchDiameter, gamma, boreDiameter,
                 toothCentreLine, toothCentrePoint, vertices):
        self.label = label
        self.isPinion = isPinion
        self.teeth = teeth
        self.pitchDiameter = pitchDiameter
        self.gamma = gamma
        self.boreDiameter = boreDiameter
        self.toothCentreLine = toothCentreLine
        self.toothCentrePoint = toothCentrePoint
        self.vertices = vertices
        # Filled in as the per-gear build runs.
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.toothSketch = adsk.fusion.Sketch.cast(None)
        self.toothEmbedded = False
        self.profileSketch = adsk.fusion.Sketch.cast(None)
        self.shaftEdge = adsk.fusion.SketchLine.cast(None)
        self.occurrence = adsk.fusion.Occurrence.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)

    @property
    def heelCorner(self):
        return self.vertices[3]

    @property
    def heelFar(self):
        return self.vertices[2]

    @property
    def toeNear(self):
        return self.vertices[4]

    @property
    def toeFar(self):
        return self.vertices[5]


class BevelGearCommandInputsConfigurator:
    """S01 and S02: the command dialog, and the conditional visibility of the two
    spiral-only inputs.

    `configure` and `handle_input_changed` are bound by name by the shared command wiring in
    commands/_gear_command.py; this module declares them and never calls them."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane, first so it wins Fusion's auto-focus ([PB-AUTOFOCUS-FIRST]). Each
        # filter is the named enum attribute, never a quoted literal
        # ([PB-SELECTION-FILTER-ENUM]), and the limits are declared per input.
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point.
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component, pre-selected on the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module: unitless, so the unit string is empty and the number means millimetres.
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle.
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6-7. The two tooth counts.
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 8-9. The two base heights: displayed in millimetres, defaulted in internal
        # centimetres ([PB-DIALOG-DEFAULT-UNITS]).
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore. The third argument makes it a check box, not a button.
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11-12. The two bore diameters.
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 13-14. Face Width and Tooth Spacing.
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 15. Mean Spiral Angle: always visible, because it is how the user reaches a
        # curved bevel.
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True, '')
        handInput.listItems.add(_HAND_LEFT, False, '')

        # 17. Cutter Radius.
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # The initial state of the two spiral-only inputs (S02).
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        """S02: hide Hand of Spiral and Cutter Radius while the Mean Spiral Angle is 0.

        The Fusion API has no declarative show-if, so this is the `isVisible` property. The
        spiral angle is read from its EXPRESSION, not from the input's `value`."""
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return

        # Leave both shown when the expression cannot be evaluated: hiding is cosmetic, and
        # the inputs are read normally either way.
        visible = True
        try:
            design: adsk.fusion.Design = get_design()
            spiral = design.unitsManager.evaluateExpression(
                spiralInput.expression, 'rad')
            visible = spiral > 0
        except Exception:
            visible = True

        handInput.isVisible = visible
        cutterInput.isVisible = visible

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        """S02: the dialog's inputChanged event, which recomputes the visibility on every
        change."""
        cls._updateSpiralInputVisibility(args.inputs)


class BevelGearGenerator(Generator):
    """The bevel gear pair: one §2 lattice, a borrowed spur tooth per gear, and a body per
    gear revolved, lofted, trimmed, patterned, joined and bored from that lattice."""

    # S21: the lengthwise crown's relief per radian of total twist. 0 disables the crown.
    _CROWN_PER_RAD = 0.5

    # S29: the pinion's extra mesh rotation, in tooth fractions. It is 0 because the spiral
    # build leaves the mid-face section unrotated, so the pinion already meshes where the
    # straight tooth did.
    _PINION_MESH_PHASE_TEETH = 0.0

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        # Seeded with the cast of a class the field really holds. `adsk.core.Base` has no
        # `cast` at runtime, even though the stub and the API database both declare one.
        # Target Plane accepts a construction plane or a planar face, Center Point a
        # construction point or a sketch point (S01); either seed narrows the field off None.
        self._targetPlane = adsk.fusion.ConstructionPlane.cast(None)
        self._centerPoint = adsk.fusion.SketchPoint.cast(None)
        self._module = 1.0
        self._drivingTeeth = 31
        self._pinionTeeth = 31
        self._shaftAngle_rad = math.radians(90)
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0
        self._toothSpacing_cm = 0.0
        self._spiralAngle_rad = 0.0
        self._hand = _HAND_RIGHT
        self._cutterRadius_cm = 0.0
        # Derived in _readInputs (S03).
        self._drivingPitchDiameter_cm = 0.0
        self._pinionPitchDiameter_cm = 0.0
        self._coneDistance_cm = 0.0
        self._gamma_p = 0.0
        self._gamma_g = 0.0
        self._drivingBoreDiameter_cm = 0.0
        self._pinionBoreDiameter_cm = 0.0
        # The occurrence tree and the shared sketches (S04-S07).
        self._bevelComponent = adsk.fusion.Component.cast(None)
        self._designOccurrence = adsk.fusion.Occurrence.cast(None)
        self._designComponent = adsk.fusion.Component.cast(None)
        self._anchorSketch = adsk.fusion.Sketch.cast(None)
        self._anchorCentre = adsk.fusion.SketchPoint.cast(None)
        self._anchorLine = adsk.fusion.SketchLine.cast(None)
        self._gearProfilesPlane = adsk.fusion.ConstructionPlane.cast(None)
        self._gearProfilesSketch = adsk.fusion.Sketch.cast(None)
        self._apexPoint = adsk.fusion.SketchPoint.cast(None)
        self._resolvedFaceWidth_cm = 0.0
        self._resolvedDrivingBase_cm = 0.0
        self._resolvedPinionBase_cm = 0.0

    def prefixBase(self) -> str:
        return 'BevelGear'

    def generateName(self) -> str:
        return 'Bevel Gear ({}:{})'.format(self._drivingTeeth, self._pinionTeeth)

    # --- S03: read and validate every input, and derive the cone geometry -----------------

    def generate(self, inputs: adsk.core.CommandInputs):
        """The build, in the order the step list gives it (S03 through S31)."""
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)
        futil.log('bevel: module={} teeth={}/{} shaft angle={} deg'.format(
            module, drivingTeeth, pinionTeeth, shaftAngle_deg))

        self.parentComponent = parentComponent
        self._targetPlane = targetPlane
        self._centerPoint = centerPoint

        self._buildComponents()
        self._buildAnchorSketch()
        self._buildGearProfilesPlane()
        sides = self._buildGearProfilesSketch()

        # Pinion first, which is the order S08 pins.
        for side in sides:
            self._buildGear(side)

        # S31: leave only the two finished gear bodies visible.
        hide_construction_geometry(self._bevelComponent)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        """S03: every input in one pass, before anything creates an occurrence.

        Each input is read with the helper matching the type it was declared with, and every
        numeric and angular field through `evaluateExpression`, which always returns Fusion
        internal units — centimetres and radians — whatever unit string is passed
        ([PB-EVAL-EXPRESSION])."""
        design: adsk.fusion.Design = get_design()

        def value(inputId, unit):
            return self._readValue(inputs, inputId, unit)

        parentSelection = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentSelection) == 0:
            parentComponent = design.rootComponent
        else:
            entity = parentSelection[0]
            if entity.objectType == adsk.fusion.Occurrence.classType():
                parentComponent = entity.component
            else:
                parentComponent = entity

        planeSelection = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeSelection) == 0:
            raise Exception('Select the Target Plane the driving gear sits flush against.')
        targetPlane = planeSelection[0]

        centerSelection = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centerSelection) == 0:
            raise Exception('Select the Center Point the driving bevel gear is centered on.')
        centerPoint = centerSelection[0]

        # Module comes back as a RAW MILLIMETRE number, because its unit string is empty.
        module = value(INPUT_ID_MODULE, '')
        shaftAngle_rad = value(INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(value(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(value(INPUT_ID_PINION_TEETH, '')))
        drivingBaseHeight_cm = value(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeight_cm = value(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = value(INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore_cm = value(INPUT_ID_PINION_BORE, 'mm')
        faceWidth_cm = value(INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing_cm = value(INPUT_ID_TOOTH_SPACING, 'mm')
        spiralAngle_rad = value(INPUT_ID_SPIRAL_ANGLE, 'deg')
        cutterRadius_cm = value(INPUT_ID_CUTTER_RADIUS, 'mm')

        handInput = adsk.core.DropDownCommandInput.cast(inputs.itemById(INPUT_ID_HAND))
        hand = _HAND_RIGHT
        if handInput is not None and handInput.selectedItem is not None:
            hand = handInput.selectedItem.name

        # Validation.
        if module <= 0:
            raise Exception('Module must be greater than 0.')
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3.')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3.')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                'Shaft Angle must be between 30 and 150 degrees (got {:.3f}).'.format(
                    shaftAngle_deg))
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                'Mean Spiral Angle must be at least 0 and under 60 degrees '
                '(got {:.3f}).'.format(spiralAngle_deg))
        for label, amount in (
                ('Driving Gear Base Height', drivingBaseHeight_cm),
                ('Pinion Gear Base Height', pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', drivingBore_cm),
                ('Pinion Gear Bore Diameter', pinionBore_cm),
                ('Face Width', faceWidth_cm),
                ('Tooth Spacing', toothSpacing_cm),
                ('Cutter Radius', cutterRadius_cm)):
            if amount < 0:
                raise Exception('{} must not be negative.'.format(label))

        self._module = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_rad = shaftAngle_rad
        self._drivingBaseHeight_cm = drivingBaseHeight_cm
        self._pinionBaseHeight_cm = pinionBaseHeight_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBore_cm
        self._pinionBore_cm = pinionBore_cm
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm

        # The derived cone geometry. Every length below is to_cm-converted, because it comes
        # from Module, which is millimetres.
        self._drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        self._pinionPitchDiameter_cm = to_cm(module * pinionTeeth)
        self._coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
        drivingPitch = self._drivingPitchDiameter_cm
        pinionPitch = self._pinionPitchDiameter_cm
        self._gamma_p = math.atan2(
            math.sin(shaftAngle_rad) * pinionPitch,
            drivingPitch + pinionPitch * math.cos(shaftAngle_rad))
        self._gamma_g = shaftAngle_rad - self._gamma_p

        self._drivingBoreDiameter_cm = (
            drivingBore_cm if drivingBore_cm > 0 else drivingPitch / 4.0)
        self._pinionBoreDiameter_cm = (
            pinionBore_cm if pinionBore_cm > 0 else pinionPitch / 4.0)

        # The two resolved base heights: the driving offset falls back to
        # module * driving teeth / 8, and the pinion offset to the RESOLVED driving offset
        # scaled by the tooth ratio.
        self._resolvedDrivingBase_cm = (
            drivingBaseHeight_cm if drivingBaseHeight_cm > 0
            else to_cm(module * drivingTeeth / 8.0))
        self._resolvedPinionBase_cm = (
            pinionBaseHeight_cm if pinionBaseHeight_cm > 0
            else self._resolvedDrivingBase_cm * (float(pinionTeeth) / float(drivingTeeth)))

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth,
                pinionTeeth, shaftAngle_deg)

    def _readValue(self, inputs: adsk.core.CommandInputs, inputId, unit) -> float:
        """One numeric or angular input, evaluated from its expression. The unit string
        controls parsing only: the result is always Fusion internal units — centimetres and
        radians ([PB-EVAL-EXPRESSION])."""
        design: adsk.fusion.Design = get_design()
        return design.unitsManager.evaluateExpression(
            inputs.itemById(inputId).expression, unit)

    # --- S04: the Bevel Gear and Design components ----------------------------------------

    def _buildComponents(self):
        """S04: the occurrence tree. Never activate any occurrence ([PB-NEVER-ACTIVATE]) —
        the Anchor sketch is authored on the user's external, root-owned target plane, and an
        activated occurrence resolves that plane in its own local frame."""
        bevelOccurrence = self.getOccurrence()
        self._bevelComponent = bevelOccurrence.component
        self._bevelComponent.name = 'Bevel Gear'

        designOccurrence = self._bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        self._designOccurrence = designOccurrence
        self._designComponent = designOccurrence.component

    # --- S05: the Anchor sketch -----------------------------------------------------------

    def _buildAnchorSketch(self):
        """S05: the sketch on the user's own target plane, its projected centre, and the
        reference line every §2 direction is taken relative to."""
        sketch = self._designComponent.sketches.add(self._targetPlane)
        sketch.name = 'Anchor'
        self._anchorSketch = sketch

        # The projection, not a re-derived copy ([PB-USE-SELECTED-PLANE]).
        centre = sketch.project(self._centerPoint).item(0)
        centreXY = _xy(centre.geometry)

        # Seeded at exactly plus and minus 0.5 cm along the sketch's local X, so the seeded
        # length is 10 mm.
        start = _p3(_vadd(centreXY, (-0.5, 0.0)))
        end = _p3(_vadd(centreXY, (0.5, 0.0)))
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)

        constraints = sketch.geometricConstraints
        constraints.addCoincident(centre, line)
        constraints.addMidPoint(centre, line)
        # The seeded 10 mm is locked without assigning the dimension's value: the length is
        # arbitrary, the line is only a reference.
        sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(_vadd(centreXY, (0.0, 0.2))))
        # Sketch-local, so it survives a tilted target plane ([PB-REFLINE-DIRECTION]).
        constraints.addHorizontal(line)

        self._anchorLine = line
        self._anchorCentre = centre
        self._gateSketch(sketch)

    # --- S06: the Gear Profiles plane -----------------------------------------------------

    def _buildGearProfilesPlane(self):
        """S06: the plane through the Anchor Line at 90 degrees to the ORIGINAL target
        plane. The sketch line goes straight into setByAngle, never wrapped in a path
        ([PB-CONSTRUCTION-PLANES])."""
        planeInput = self._designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(self._anchorLine,
                              adsk.core.ValueInput.createByString('90 deg'),
                              self._targetPlane)
        plane = self._designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # --- S07: the §2 lattice --------------------------------------------------------------

    def _buildGearProfilesSketch(self):
        """S07: one sketch holding the whole lattice, every line of it a construction line.

        Two construction rules run through this method. Each line is built from raw
        Point3D coordinates and every endpoint that meets existing geometry is pinned with
        exactly one addCoincident — never a shared point and a coincident both
        ([BEVEL-F-COINCIDENT-STYLE], [PB-SHARE-XOR-COINCIDENT]). And each named line is
        created once and reused ([BEVEL-F-LINE-ONCE])."""
        sketch = self._designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gearProfilesSketch = sketch
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions

        def solved(point):
            """The SOLVED position of a sketch point ([PB-SOLVED-GEOMETRY])."""
            return _xy(point.geometry)

        # The projection keeps the chain inside Design: §2 re-projects the ANCHOR SKETCH's
        # stashed centre point rather than the raw user selection.
        centre = sketch.project(self._anchorCentre).item(0)
        anchor = sketch.project(self._anchorLine).item(0)
        centreXY = _xy(centre.geometry)
        anchorDir = _vunit(_vsub(anchor.endSketchPoint.geometry,
                                 anchor.startSketchPoint.geometry))

        # The in-plane perpendicular, its SIGN chosen so it points toward the target plane's
        # normal rather than by the sketch's local +Y ([BEVEL-F-GROW-SIDE]). The whole figure
        # is then placed in the sketch's own 2-D coordinates ([BEVEL-F-APEX-LOCAL]).
        perp = _vunit(_vperp(anchorDir))
        # `targetPlane.geometry.normal` for both selection kinds, which is what the framework
        # reader does; a selection with no readable normal is rejected here rather than
        # silently giving the grow side a meaningless sign.
        normal = get_normal(self._targetPlane)
        if normal is None:
            raise Exception(
                'The selected Target Plane carries no readable normal, so the grow side of '
                'the Gear Profiles lattice cannot be chosen. Select a construction plane or '
                'a planar face.')
        here = sketch.sketchToModelSpace(_p3(centreXY))
        there = sketch.sketchToModelSpace(_p3(_vadd(centreXY, perp)))
        if here.vectorTo(there).dotProduct(normal) < 0:
            perp = _vscale(perp, -1)

        drivingPitch = self._drivingPitchDiameter_cm
        pinionPitch = self._pinionPitchDiameter_cm
        dedendum = to_cm(1.25 * self._module)
        moduleLength = to_cm(self._module)
        coneRadius = (pinionPitch / 2.0) / math.sin(self._gamma_p)
        alongA = coneRadius * math.cos(self._gamma_p)
        alongB = coneRadius * math.cos(self._gamma_g)

        # 1. Centre to Apex. No length constraint: the far end is the Apex.
        apexSeed = _vadd(centreXY, _vscale(perp, drivingPitch))
        centerToApex = self._constructionLine(sketch, centreXY, apexSeed)
        constraints.addCoincident(centerToApex.startSketchPoint, centre)
        constraints.addPerpendicular(centerToApex, anchor)
        apexPoint = centerToApex.endSketchPoint

        # 2. Driving Gear Shaft Axis, from the Apex back toward the anchor line. Parallel to
        # the centre-to-apex line, never addVertical, which would force the sketch's
        # world-vertical and be wrong on a tilted plane. The far end is B, undimensioned.
        drivingDir = _vscale(perp, -1)
        bSeed = _vadd(apexSeed, _vscale(drivingDir, alongB))
        drivingShaft = self._constructionLine(sketch, apexSeed, bSeed)
        constraints.addCoincident(drivingShaft.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaft, centerToApex)
        pointB = drivingShaft.endSketchPoint

        # 3. Pinion Gear Shaft Axis. BOTH candidate senses are formed and the one whose
        # endpoint has the greater sketch X is kept — never one fixed sense flipped on a
        # negative X.
        plusDir = _vrot(drivingDir, self._shaftAngle_rad)
        minusDir = _vrot(drivingDir, -self._shaftAngle_rad)
        plusEnd = _vadd(apexSeed, _vscale(plusDir, alongA))
        minusEnd = _vadd(apexSeed, _vscale(minusDir, alongA))
        pinionDir = plusDir
        aSeed = plusEnd
        if minusEnd[0] > plusEnd[0]:
            pinionDir = minusDir
            aSeed = minusEnd
        pinionShaft = self._constructionLine(sketch, apexSeed, aSeed)
        constraints.addCoincident(pinionShaft.startSketchPoint, apexPoint)
        pointA = pinionShaft.endSketchPoint
        # The text point sits inside the wedge, on the interior bisector a quarter of the
        # pinion pitch diameter out from the apex, so the dimension measures the shaft angle
        # and not its supplement ([PB-ANGULAR-DIM]).
        bisector = _vunit(_vadd(drivingDir, pinionDir))
        shaftAngleDimension = dimensions.addAngularDimension(
            drivingShaft, pinionShaft,
            _p3(_vadd(apexSeed, _vscale(bisector, pinionPitch / 4.0))))
        shaftAngleDimension.parameter.value = self._shaftAngle_rad

        # 4. The two drops to Apex 2. Each perpendicular's sense is picked by the sign of its
        # dot product with the direction toward the OTHER shaft's point — never against a
        # toward-the-anchor-line reference, which is degenerate on the driving side.
        dropADir = _vtoward(pinionDir, _vsub(bSeed, aSeed))
        apex2Seed = _vadd(aSeed, _vscale(dropADir, pinionPitch / 2.0))
        dropA = self._constructionLine(sketch, aSeed, apex2Seed)
        constraints.addCoincident(dropA.startSketchPoint, pointA)
        constraints.addPerpendicular(dropA, pinionShaft)
        dropADimension = dimensions.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(_vadd(aSeed, _vscale(dropADir, pinionPitch / 4.0))))
        dropADimension.parameter.value = pinionPitch / 2.0

        dropBDir = _vtoward(drivingDir, _vsub(aSeed, bSeed))
        apex2SeedB = _vadd(bSeed, _vscale(dropBDir, drivingPitch / 2.0))
        dropB = self._constructionLine(sketch, bSeed, apex2SeedB)
        constraints.addCoincident(dropB.startSketchPoint, pointB)
        constraints.addPerpendicular(dropB, drivingShaft)
        dropBDimension = dimensions.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(_vadd(bSeed, _vscale(dropBDir, drivingPitch / 4.0))))
        dropBDimension.parameter.value = drivingPitch / 2.0

        constraints.addCoincident(dropB.endSketchPoint, dropA.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # 6. The Pitch Line, Apex to Apex 2.
        pitchLine = self._constructionLine(sketch, apexSeed, apex2Seed)
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # 7. The two dedendum lines. The one drawn toward the anchor line is the driving
        # gear's, ending at D; the one away is the pinion's, ending at C.
        pitchDir = _vunit(_vsub(apex2Seed, apexSeed))
        dHat = _vtoward(pitchDir, drivingDir)
        cHat = _vscale(dHat, -1)

        dSeed = _vadd(apex2Seed, _vscale(dHat, dedendum))
        dedendumD = self._constructionLine(sketch, apex2Seed, dSeed)
        constraints.addCoincident(dedendumD.startSketchPoint, apex2Point)
        constraints.addPerpendicular(dedendumD, pitchLine)
        dedendumDDimension = dimensions.addDistanceDimension(
            dedendumD.startSketchPoint, dedendumD.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(_vadd(apex2Seed, _vscale(dHat, dedendum * 1.5))))
        dedendumDDimension.parameter.value = dedendum
        pointD = dedendumD.endSketchPoint

        cSeed = _vadd(apex2Seed, _vscale(cHat, dedendum))
        dedendumC = self._constructionLine(sketch, apex2Seed, cSeed)
        constraints.addCoincident(dedendumC.startSketchPoint, apex2Point)
        constraints.addPerpendicular(dedendumC, pitchLine)
        dedendumCDimension = dimensions.addDistanceDimension(
            dedendumC.startSketchPoint, dedendumC.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(_vadd(apex2Seed, _vscale(cHat, dedendum * 1.5))))
        dedendumCDimension.parameter.value = dedendum
        pointC = dedendumC.endSketchPoint

        # 8. The two root axes.
        rootC = self._constructionLine(sketch, apexSeed, cSeed)
        constraints.addCoincident(rootC.startSketchPoint, apexPoint)
        constraints.addCoincident(rootC.endSketchPoint, pointC)
        rootD = self._constructionLine(sketch, apexSeed, dSeed)
        constraints.addCoincident(rootD.startSketchPoint, apexPoint)
        constraints.addCoincident(rootD.endSketchPoint, pointD)

        # 9. The module-length extensions. Each is seeded one module long and NOT
        # dimensioned; the perpendiculars place their far ends.
        eSeed = _vadd(aSeed, _vscale(pinionDir, moduleLength))
        lineAE = self._constructionLine(sketch, aSeed, eSeed)
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaft)
        pointE = lineAE.endSketchPoint
        lineCE = self._constructionLine(sketch, cSeed, eSeed)
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        fSeed = _vadd(bSeed, _vscale(drivingDir, moduleLength))
        lineBF = self._constructionLine(sketch, bSeed, fSeed)
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaft)
        pointF = lineBF.endSketchPoint
        lineDF = self._constructionLine(sketch, dSeed, fSeed)
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        gSeed = _vadd(eSeed, _vscale(pinionDir, moduleLength))
        lineEG = self._constructionLine(sketch, eSeed, gSeed)
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint
        hSeed = _vadd(cSeed, _vscale(cHat, moduleLength))
        lineCH = self._constructionLine(sketch, cSeed, hSeed)
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, dedendumC)
        pointH = lineCH.endSketchPoint
        lineGH = self._constructionLine(sketch, gSeed, hSeed)
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        iSeed = _vadd(fSeed, _vscale(drivingDir, moduleLength))
        lineFI = self._constructionLine(sketch, fSeed, iSeed)
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint
        jSeed = _vadd(dSeed, _vscale(dHat, moduleLength))
        lineDJ = self._constructionLine(sketch, dSeed, jSeed)
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, dedendumD)
        pointJ = lineDJ.endSketchPoint
        lineIJ = self._constructionLine(sketch, iSeed, jSeed)
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # 10. The two base-height offsets. The pairs are already parallel by construction, so
        # no parallel constraint is added — a redundant one over-constrains ([PB-OFFSET-DIM]).
        drivingOffset = dimensions.addOffsetDimension(
            dropB, lineIJ, _p3(_vscale(_vadd(_vadd(bSeed, apex2Seed), _vadd(iSeed, jSeed)),
                                       0.25)))
        drivingOffset.parameter.value = self._resolvedDrivingBase_cm
        pinionOffset = dimensions.addOffsetDimension(
            dropA, lineGH, _p3(_vscale(_vadd(_vadd(aSeed, apex2Seed), _vadd(gSeed, hSeed)),
                                       0.25)))
        pinionOffset.parameter.value = self._resolvedPinionBase_cm

        # 11. A to G, and the closure that fixes the figure's position along the grow
        # direction.
        lineAG = self._constructionLine(sketch, aSeed, gSeed)
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)
        constraints.addCoincident(pointI, centre)

        # 12. The Maximum Face Width, read from the SOLVED positions
        # ([PB-SOLVED-GEOMETRY]), never from the seed coordinates.
        solvedA, solvedB = solved(pointA), solved(pointB)
        solvedC, solvedD = solved(pointC), solved(pointD)
        solvedH, solvedJ = solved(pointH), solved(pointJ)
        pinionReach = abs(_vcross(_vsub(solvedA, solvedC),
                                  _vunit(_vsub(solvedH, solvedC))))
        drivingReach = abs(_vcross(_vsub(solvedB, solvedD),
                                   _vunit(_vsub(solvedJ, solvedD))))
        maximumFaceWidth = 0.95 * min(pinionReach, drivingReach)
        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maximumFaceWidth:
                raise Exception(
                    'Face Width {:.4f} mm exceeds the Maximum Face Width {:.4f} mm this '
                    'geometry allows; the revolved profile would cross its own axis of '
                    'revolution.'.format(to_mm(self._faceWidth_cm),
                                         to_mm(maximumFaceWidth)))
            faceWidth = self._faceWidth_cm
        else:
            faceWidth = min(self._coneDistance_cm / 6.0, maximumFaceWidth)
        self._resolvedFaceWidth_cm = faceWidth
        futil.log('bevel: face width {:.4f} mm (maximum {:.4f} mm)'.format(
            to_mm(faceWidth), to_mm(maximumFaceWidth)))

        # The solved frame the last two items are placed in.
        solvedApex = solved(apexPoint)
        solvedApex2 = solved(apex2Point)
        solvedG, solvedI = solved(pointG), solved(pointI)
        solvedPinionDir = _vunit(_vsub(solvedA, solvedApex))
        solvedDrivingDir = _vunit(_vsub(solvedB, solvedApex))
        solvedCHat = _vunit(_vsub(solvedC, solvedApex2))
        solvedDHat = _vunit(_vsub(solvedD, solvedApex2))

        # 13. K, the tooth-centre point K prime, and the §3 reference line. The far end of
        # G-to-K is pinned with two point-on-line coincidents rather than a collinear, which
        # would over-constrain because G and C are already fixed.
        kSeed = _line_intersect(solvedApex, solvedPinionDir, solvedApex2, solvedCHat)
        lineGK = self._constructionLine(sketch, solvedG, kSeed)
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        constraints.addCoincident(lineGK.endSketchPoint, pinionShaft)
        constraints.addCoincident(lineGK.endSketchPoint, dedendumC)
        pointK = lineGK.endSketchPoint
        lineCK = self._constructionLine(sketch, solvedC, kSeed)
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)
        pinionToothCentreLine = lineCK
        pinionToothCentre = pointK
        if self._toothSpacing_cm > 0:
            kPrimeSeed = _vadd(kSeed, _vscale(solvedCHat, self._toothSpacing_cm))
            lineKKPrime = self._constructionLine(sketch, kSeed, kPrimeSeed)
            constraints.addCoincident(lineKKPrime.startSketchPoint, pointK)
            constraints.addCoincident(lineKKPrime.endSketchPoint, dedendumC)
            spacingDimension = dimensions.addDistanceDimension(
                lineKKPrime.startSketchPoint, lineKKPrime.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                _p3(_vadd(kSeed, _vscale(solvedCHat, self._toothSpacing_cm / 2.0))))
            spacingDimension.parameter.value = self._toothSpacing_cm
            pinionToothCentre = lineKKPrime.endSketchPoint
            lineCKPrime = self._constructionLine(sketch, solvedC, kPrimeSeed)
            constraints.addCoincident(lineCKPrime.startSketchPoint, pointC)
            constraints.addCoincident(lineCKPrime.endSketchPoint, pinionToothCentre)
            pinionToothCentreLine = lineCKPrime

        lSeed = _line_intersect(solvedApex, solvedDrivingDir, solvedApex2, solvedDHat)
        lineIL = self._constructionLine(sketch, solvedI, lSeed)
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        constraints.addCoincident(lineIL.endSketchPoint, drivingShaft)
        constraints.addCoincident(lineIL.endSketchPoint, dedendumD)
        pointL = lineIL.endSketchPoint
        lineDL = self._constructionLine(sketch, solvedD, lSeed)
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)
        drivingToothCentreLine = lineDL
        drivingToothCentre = pointL
        if self._toothSpacing_cm > 0:
            lPrimeSeed = _vadd(lSeed, _vscale(solvedDHat, self._toothSpacing_cm))
            lineLLPrime = self._constructionLine(sketch, lSeed, lPrimeSeed)
            constraints.addCoincident(lineLLPrime.startSketchPoint, pointL)
            constraints.addCoincident(lineLLPrime.endSketchPoint, dedendumD)
            drivingSpacing = dimensions.addDistanceDimension(
                lineLLPrime.startSketchPoint, lineLLPrime.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                _p3(_vadd(lSeed, _vscale(solvedDHat, self._toothSpacing_cm / 2.0))))
            drivingSpacing.parameter.value = self._toothSpacing_cm
            drivingToothCentre = lineLLPrime.endSketchPoint
            lineDLPrime = self._constructionLine(sketch, solvedD, lPrimeSeed)
            constraints.addCoincident(lineDLPrime.startSketchPoint, pointD)
            constraints.addCoincident(lineDLPrime.endSketchPoint, drivingToothCentre)
            drivingToothCentreLine = lineDLPrime

        # 14. The two toe lines, seeded near their solved positions ([PB-SEED-NEAR]) and then
        # given exactly four constraints each.
        mSeed, nSeed = _toe_edge(solvedC, solvedCHat, solvedApex, solvedA, solvedApex2,
                                 faceWidth)
        lineMN = self._constructionLine(sketch, mSeed, nSeed)
        constraints.addCoincident(lineMN.startSketchPoint, rootC)
        # N goes on the A-to-Apex-2 DROP, not the shaft axis, which would put it on the axis
        # of revolution.
        constraints.addCoincident(lineMN.endSketchPoint, dropA)
        constraints.addParallel(lineMN, lineCH)
        pinionToe = dimensions.addOffsetDimension(
            lineCH, lineMN,
            _p3(_vscale(_vadd(_vadd(solvedC, solvedH), _vadd(mSeed, nSeed)), 0.25)))
        pinionToe.parameter.value = faceWidth
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint

        oSeed, pSeed = _toe_edge(solvedD, solvedDHat, solvedApex, solvedB, solvedApex2,
                                 faceWidth)
        lineOP = self._constructionLine(sketch, oSeed, pSeed)
        constraints.addCoincident(lineOP.startSketchPoint, rootD)
        constraints.addCoincident(lineOP.endSketchPoint, dropB)
        constraints.addParallel(lineOP, lineDJ)
        drivingToe = dimensions.addOffsetDimension(
            lineDJ, lineOP,
            _p3(_vscale(_vadd(_vadd(solvedD, solvedJ), _vadd(oSeed, pSeed)), 0.25)))
        drivingToe.parameter.value = faceWidth
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint

        for (startPoint, endPoint) in ((pointM, pointC), (pointN, pointA),
                                       (pointO, pointD), (pointP, pointB),
                                       (pointB, pointI)):
            reference = self._constructionLine(sketch, solved(startPoint), solved(endPoint))
            constraints.addCoincident(reference.startSketchPoint, startPoint)
            constraints.addCoincident(reference.endSketchPoint, endPoint)

        self._apexPoint = apexPoint
        self._gateSketch(sketch)

        # Pinion first, which is the order S08 pins.
        pinion = _GearSide(
            'Pinion', True, self._pinionTeeth, self._pinionPitchDiameter_cm, self._gamma_p,
            self._pinionBoreDiameter_cm, pinionToothCentreLine, pinionToothCentre,
            [pointA, pointG, pointH, pointC, pointM, pointN])
        driving = _GearSide(
            'Driving', False, self._drivingTeeth, self._drivingPitchDiameter_cm,
            self._gamma_g, self._drivingBoreDiameter_cm, drivingToothCentreLine,
            drivingToothCentre, [pointB, pointI, pointJ, pointD, pointO, pointP])
        return (pinion, driving)

    def _constructionLine(self, sketch: adsk.fusion.Sketch, startXY,
                          endXY) -> adsk.fusion.SketchLine:
        """One §2 construction line, both endpoints created from raw coordinates so each end
        that meets existing geometry can carry exactly one coincident
        ([BEVEL-F-COINCIDENT-STYLE]). Curve collections live under `sketch.sketchCurves`
        ([PB-SKETCHCURVES])."""
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(_p3(startXY), _p3(endXY))
        line.isConstruction = True
        return line

    def _gateSketch(self, sketch):
        """The full-constraint gate, naming the sketch ([PB-FULL-CONSTRAINT],
        [BEVEL-F-FULL-CONSTRAINT])."""
        if not sketch.isFullyConstrained:
            raise Exception(
                'Sketch "{}" is not fully constrained.'.format(sketch.name))

    # --- the per-gear build ---------------------------------------------------------------

    def _buildGear(self, side: _GearSide):
        """S08 through S30 for one gear of the pair."""
        self._buildToothPlane(side)
        self._buildToothSketch(side)
        self._buildToothAxis(side)
        self._buildProfileSketch(side)
        self._revolveGearBody(side)
        toothBody = self._loftToothBody(side)
        toothBody = self._buildToothBody(side, toothBody)
        side.toothBody = toothBody
        pattern = self._patternTeeth(side, toothBody)
        body = self._combineTeeth(side, pattern)
        body = self._cutBore(side, body)
        self._meshRotate(side, body)
        # S30: relocating a body between components preserves world position and needs no
        # activation ([PB-NO-CROSS-SIBLING]).
        body.moveToComponent(side.occurrence)

    def _buildToothPlane(self, side: _GearSide):
        """S08: the plane through this gear's tooth-centre reference line, perpendicular to
        the Gear Profiles sketch plane."""
        plane = plane_by_angle(self._designComponent, side.toothCentreLine,
                               self._gearProfilesPlane, 90)
        plane.name = '{} Plane'.format(side.label)
        side.plane = plane

    def _virtualTeeth(self, side: _GearSide) -> int:
        """S09: the back-cone tooth number, from the closed form and never by measuring the
        lattice.

        The stashed pitch diameters are internal CENTIMETRES while Module is the raw
        MILLIMETRE number, so the factor of ten below is what keeps the count right."""
        virtualPitchRadius_mm = (side.pitchDiameter * 10 / 2) / math.cos(side.gamma)
        return int(math.floor(2 * virtualPitchRadius_mm / self._module))

    def _buildToothSketch(self, side: _GearSide):
        """S09: one virtual spur tooth, drawn by the borrowed spur generator."""
        sketch = self._designComponent.sketches.add(side.plane)
        sketch.name = '{} Tooth'.format(side.label)
        side.toothSketch = sketch

        virtualTeeth = self._virtualTeeth(side)
        futil.log('bevel: {} virtual tooth number {}'.format(side.label, virtualTeeth))
        proxy = VirtualSpurProxy(module_mm=self._module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        # The 180 degree rotation is delivered through the angle argument, which rotates the
        # whole tooth as it is drawn; it is not a later move or sketch rotation.
        drawer.draw(side.toothCentrePoint, angle=math.radians(180))

        # The drawer writes this slot during the draw; it is the deterministic selector for
        # the tooth loop's line count in S13.
        side.toothEmbedded = proxy._lastToothEmbedded

        # NOT gated: the drawer labels its four circles with along-path sketch text, and such
        # text carries its own unpinned position, so a tooth whose geometry is completely
        # determined still reads false ([BEVEL-F-FULL-CONSTRAINT]).
        if not sketch.isFullyConstrained:
            futil.log('bevel: sketch "{}" reports under-constrained (expected: the '
                      'drawer\'s along-path sketch text)'.format(sketch.name))

    def _buildToothAxis(self, side: _GearSide):
        """S10: the axis through the tooth-centre point, normal to the plane the tooth was
        drawn on ([PB-CONSTRUCTION-AXES]).

        It is the intersection of the Gear Profiles plane with a helper plane taken at the
        far end of the tooth-centre reference line. `setByPerpendicularAtPoint` is the obvious
        alternative and needs a BRepFace this step does not have."""
        planes = self._designComponent.constructionPlanes
        helperInput = planes.createInput()
        helperInput.setByDistanceOnPath(side.toothCentreLine,
                                        adsk.core.ValueInput.createByReal(1.0))
        helperPlane = planes.add(helperInput)
        helperPlane.name = '{} Tooth Normal Plane'.format(side.label)

        axisInput = self._designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        axis = self._designComponent.constructionAxes.add(axisInput)
        axis.name = '{} Tooth Axis'.format(side.label)

    def _buildProfileSketch(self, side: _GearSide):
        """S11: a FRESH sketch on the axial plane holding exactly one hexagon loop, built by
        the recreate-share-fix recipe ([PB-PROJECT-NOT-FIXED]) rather than by projecting the
        §2 points."""
        sketch = self._designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = '{} Profile'.format(side.label)
        side.profileSketch = sketch

        # modelToSketchSpace is a point-transforming METHOD, not a matrix
        # ([PB-SPACE-METHODS]).
        vertices = []
        for source in side.vertices:
            vertices.append(sketch.sketchPoints.add(
                sketch.modelToSketchSpace(source.worldGeometry)))

        lines = []
        for index in range(len(vertices)):
            lines.append(sketch.sketchCurves.sketchLines.addByTwoPoints(
                vertices[index], vertices[(index + 1) % len(vertices)]))

        # Order matters: fixing a bare point before a line consumes it does not leave the
        # sketch fully constrained.
        for line in lines:
            line.startSketchPoint.isFixed = True
            line.endSketchPoint.isFixed = True

        # The FIRST edge is the gear's shaft axis for the revolve, the pattern, the bore
        # plane and the meshing rotation, so it has to carry a trustworthy world position
        # ([PB-WORLDGEO-CONSTRAINED]).
        side.shaftEdge = lines[0]
        self._gateSketch(sketch)

    def _revolveGearBody(self, side: _GearSide):
        """S12: the gear's own component, then the frustum revolved from the hexagon."""
        occurrence = self._bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        occurrence.component.name = '{} Gear'.format(side.label)
        side.occurrence = occurrence

        # The sketch holds exactly one closed loop ([PB-SINGLE-PROFILE]).
        profileSketch: adsk.fusion.Sketch = side.profileSketch
        profile = profileSketch.profiles.item(0)
        revolves = self._designComponent.features.revolveFeatures
        revolveInput = revolves.createInput(
            profile, side.shaftEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revolveInput)
        side.gearBody = revolve.bodies.item(0)
        side.gearBody.name = '{} Gear Body'.format(side.label)

    def _loftToothBody(self, side: _GearSide) -> adsk.fusion.BRepBody:
        """S13: the §2 Apex sketch point lofted to this gear's §3 tooth profile
        ([PB-LOFT])."""
        # The line count is DETERMINED by the embedded flag read back in S09 — never
        # accepted as either.
        lineCount = 0 if side.toothEmbedded else 2
        profile = find_profile_by_curve_counts(side.toothSketch, 2, 2, lineCount)

        lofts = self._designComponent.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Section order is loft order. The apex is the §2 SKETCH point, used directly:
        # construction geometry needs an active component and Design is never activated
        # ([PB-CONSTRUCTION-NEEDS-ACTIVE]).
        loftInput.loftSections.add(self._apexPoint)
        loftInput.loftSections.add(profile)
        loft = lofts.add(loftInput)
        body = loft.bodies.item(0)
        body.name = '{} Tooth'.format(side.label)
        return body

    # --- S14: the tooth-body hook ---------------------------------------------------------

    def _sideWorldFrame(self, side: _GearSide):
        """The world frame S15 to S23 work in, plus the two edge midpoints S14 owes
        `cut_conical_ends`. Every term is world geometry ([PB-WORLD-FRAME])."""
        startWorld = side.shaftEdge.startSketchPoint.worldGeometry
        endWorld = side.shaftEdge.endSketchPoint.worldGeometry
        axisDir = _wunit(_wvec(startWorld, endWorld))

        apexWorld = self._apexPoint.worldGeometry
        toeMid = _wmid(side.toeNear.worldGeometry, side.toeFar.worldGeometry)
        heelMid = _wmid(side.heelCorner.worldGeometry, side.heelFar.worldGeometry)
        # The root cone element runs from the apex to the HEEL DEDENDUM CORNER — never to H
        # or J, which lie a module beyond it and skew the vector.
        toeCone = side.toeNear.worldGeometry
        heelCone = side.heelCorner.worldGeometry

        # Guard the ends before anything else: a negative span silently inverts the whole
        # spiral, flipping the cutter arc, the slice direction and every segment's twist,
        # with no error.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeCone, heelCone = heelCone, toeCone
        coneVec = _wunit(_wvec(apexWorld, heelCone))

        return (apexWorld, axisDir, coneVec, toeMid, heelMid)

    def _buildToothBody(self, side: _GearSide,
                        toothBody: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        """S14: the straight branch is the whole step; above a 0 spiral angle S15 to S23 run
        in its place and this trim runs again at the end of them.

        The cutting TOOLS are the cone faces of the GEAR BODY, the revolved frustum, and the
        TARGET being split is the TOOTH BODY, the loft."""
        (apexWorld, axisDir, coneVec, toeMid, heelMid) = self._sideWorldFrame(side)

        if self._spiralAngle_rad <= 0:
            return cut_conical_ends(self._designComponent, toothBody, side.gearBody,
                                    toeMid, heelMid, apexWorld, side.label)

        coneLine = self._buildConeElementSketch(side, apexWorld, coneVec, heelMid)
        tracePlane = self._buildTracePlane(side, coneLine)
        (toe2d, heel2d, handSign) = self._buildTraceSketch(
            side, tracePlane, apexWorld, axisDir, coneVec, toeMid, heelMid)
        segments = self._sliceTooth(side, toothBody, apexWorld, coneVec, toeMid, heelMid)
        segments = self._dropApexScrap(side, segments, apexWorld, coneVec)
        total = self._twistSegments(side, segments, apexWorld, axisDir, coneVec,
                                    toeMid, heelMid, toe2d, heel2d, handSign)
        self._crownSegments(side, segments, apexWorld, axisDir, coneVec, toeMid, heelMid,
                            total)
        curved = self._loftSpiralTooth(side, segments, apexWorld, coneVec)
        return cut_conical_ends(self._designComponent, curved, side.gearBody,
                                toeMid, heelMid, apexWorld, side.label)

    # --- S15 to S23: the curved-tooth branch ----------------------------------------------

    def _buildConeElementSketch(self, side: _GearSide, apexWorld: adsk.core.Point3D,
                                coneVec: adsk.core.Vector3D,
                                heelMid: adsk.core.Point3D) -> adsk.fusion.SketchLine:
        """S15: one construction line from the apex out along the root cone element.

        A transient auxiliary sketch: it is deliberately NOT gated on isFullyConstrained
        ([BEVEL-F-FULL-CONSTRAINT])."""
        heelDistance = _cone_distance(heelMid, apexWorld, coneVec)
        sketch = self._designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = '{} Cone Element'.format(side.label)
        far = combine_point(apexWorld, heelDistance, coneVec)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            sketch.modelToSketchSpace(apexWorld), sketch.modelToSketchSpace(far))
        line.isConstruction = True
        return line

    def _buildTracePlane(self, side: _GearSide,
                         coneLine: adsk.fusion.SketchLine) -> adsk.fusion.ConstructionPlane:
        """S16: the cone's tangent plane, the axial plane rotated 90 degrees about the
        cone-element line. Its 2-D frame puts the apex at the origin with x along the cone
        element, so a point's x IS its cone distance."""
        plane = plane_by_angle(self._designComponent, coneLine, self._gearProfilesPlane, 90)
        plane.name = '{} Trace Plane'.format(side.label)
        return plane

    def _buildTraceSketch(self, side: _GearSide,
                          tracePlane: adsk.fusion.ConstructionPlane,
                          apexWorld: adsk.core.Point3D, axisDir: adsk.core.Vector3D,
                          coneVec: adsk.core.Vector3D, toeMid: adsk.core.Point3D,
                          heelMid: adsk.core.Point3D):
        """S17: the genuine face-mill cutter circle and the arc of it the trace follows."""
        rToe = _cone_distance(toeMid, apexWorld, coneVec)
        rHeel = _cone_distance(heelMid, apexWorld, coneVec)
        rMean = (rToe + rHeel) / 2.0
        span = rHeel - rToe

        cutterRadius = self._cutterRadius_cm if self._cutterRadius_cm > 0 else rMean
        # The hand sign is negated for the pinion, so the pair meshes with opposite hands.
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if side.isPinion:
            handSign = -handSign

        psi = self._spiralAngle_rad
        # The hand sign belongs on the COSINE term: on the sine term it mirrors the centre
        # about x = R_mean instead of across the cone element.
        centreX = rMean - cutterRadius * math.sin(psi)
        centreY = handSign * cutterRadius * math.cos(psi)

        # The circumferential direction of the tangent plane's 2-D frame.
        circumferential = _wunit(axisDir.crossProduct(coneVec))

        # The arc's two ends, taken a hair past the face so the kept arc reaches cleanly past
        # the end trims.
        toe2d = circle_intersect_nearest(
            rToe - _END_RELIEF * span, centreX, centreY, cutterRadius, rMean, 0.0)
        heel2d = circle_intersect_nearest(
            rHeel + _END_RELIEF * span, centreX, centreY, cutterRadius, rMean, 0.0)

        sketch = self._designComponent.sketches.add(tracePlane)
        sketch.name = '{} 2D Tooth Trace'.format(side.label)

        centreWorld = combine_point(apexWorld, centreX, coneVec, centreY, circumferential)
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            centreWorld, cutterRadius)
        circle.isConstruction = True
        # A circle's centre is free even when created where it belongs, and coinciding it to
        # the sketch origin has been observed to fail the solve ([PB-CIRCLE-CENTER]).
        circle.centerSketchPoint.isFixed = True
        # Off-centre, on the circle: a text point AT the centre is rejected
        # ([PB-RADIAL-DIM]).
        diameterDimension = sketch.sketchDimensions.addDiameterDimension(
            circle,
            combine_point(apexWorld, centreX + cutterRadius, coneVec, centreY,
                          circumferential))
        diameterDimension.parameter.value = 2 * cutterRadius

        meanWorld = combine_point(apexWorld, rMean, coneVec, 0.0, circumferential)
        toeWorld = combine_point(apexWorld, toe2d[0], coneVec, toe2d[1], circumferential)
        heelWorld = combine_point(apexWorld, heel2d[0], coneVec, heel2d[1], circumferential)
        arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            toeWorld, meanWorld, heelWorld)
        # addByThreePoints gives the arc its own centre point, so this coincident is what
        # makes it the genuine cutter circle and not a look-alike spline.
        sketch.geometricConstraints.addCoincident(
            arc.centerSketchPoint, circle.centerSketchPoint)
        radialDimension = sketch.sketchDimensions.addRadialDimension(arc, meanWorld)
        radialDimension.parameter.value = cutterRadius

        # Deliberately left with free degrees of freedom, and NOT gated
        # ([BEVEL-F-FULL-CONSTRAINT]). No downstream feature consumes this sketch: the twist
        # is computed analytically in S20, and there is no 3-D projection anywhere in this
        # branch.
        return (toe2d, heel2d, handSign)

    def _sliceTooth(self, side: _GearSide, toothBody: adsk.fusion.BRepBody,
                    apexWorld: adsk.core.Point3D, coneVec: adsk.core.Vector3D,
                    toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D):
        """S18: split the uncut apex-to-heel tooth into cross-section slabs with a FIXED
        scheme of eight planes."""
        span = (_cone_distance(heelMid, apexWorld, coneVec)
                - _cone_distance(toeMid, apexWorld, coneVec))
        basePlane: adsk.fusion.ConstructionPlane = side.plane

        # Choose the offset sign per gear so it really moves apex-ward: the two gears'
        # normals point opposite ways.
        origin = basePlane.geometry.origin
        normal = basePlane.geometry.normal
        sign = 1.0 if origin.vectorTo(apexWorld).dotProduct(normal) > 0 else -1.0

        pieces = self._sliceOnce(toothBody, basePlane, sign, span)
        if len(pieces) == 1:
            futil.log('bevel: {} slice produced one piece; retrying with the opposite '
                      'sign'.format(side.label))
            sign = -sign
            pieces = self._sliceOnce(pieces[0], basePlane, sign, span)
        if len(pieces) == 1:
            raise Exception(
                'The {} tooth could not be sliced: it is still one piece after both offset '
                'signs. Face span {:.4f} mm, sign tried {}, planes {}. The parent tooth '
                'plane may sit outside the tooth\'s span.'.format(
                    side.label, to_mm(span), sign, _SLICE_PLANES))
        return pieces

    def _sliceOnce(self, body, basePlane, sign, span):
        offsets = [sign * (k + 1) * span / 6.0 for k in range(_SLICE_PLANES)]
        return slice_body_by_offset_planes(self._designComponent, body, basePlane, offsets)

    def _dropApexScrap(self, side: _GearSide, pieces, apexWorld: adsk.core.Point3D,
                       coneVec: adsk.core.Vector3D):
        """S19: sort the slabs by the cone distance of their centroid, then re-slice the list
        FIRST and delete the apex-most scrap afterwards ([PB-REMOVE-PIECES])."""
        ordered = sorted(
            pieces,
            key=lambda piece: _cone_distance(
                piece.physicalProperties.centerOfMass, apexWorld, coneVec))
        scrap = ordered[0]
        segments = ordered[1:]
        if len(segments) == 0:
            raise Exception(
                'The {} tooth left no segments after dropping the apex scrap; the slice in '
                'S18 produced {} piece(s).'.format(side.label, len(pieces)))
        # Timeline-visible, rather than a bare delete.
        self._designComponent.features.removeFeatures.add(scrap)
        return segments

    def _endFace(self, body: adsk.fusion.BRepBody, apexWorld: adsk.core.Point3D,
                 coneVec: adsk.core.Vector3D, farthest=True):
        """A slab's heel face is the face whose centroid has the GREATEST cone distance, and
        its toe face the least — searched across ALL faces with NO surface-type filter,
        because a slab is bounded by a mix of planar cut faces and ruled side faces."""
        bestIndex = -1
        bestDistance = 0.0
        for index in range(body.faces.count):
            face = body.faces.item(index)
            distance = _cone_distance(face.centroid, apexWorld, coneVec)
            if bestIndex < 0 or (distance > bestDistance if farthest
                                 else distance < bestDistance):
                bestIndex = index
                bestDistance = distance
        if bestIndex < 0:
            raise Exception(
                'A tooth slab came back with no faces, so its {} end cannot be '
                'found.'.format('heel' if farthest else 'toe'))
        return (body.faces.item(bestIndex), bestDistance)

    def _twistSegments(self, side: _GearSide, segments, apexWorld: adsk.core.Point3D,
                       axisDir: adsk.core.Vector3D, coneVec: adsk.core.Vector3D,
                       toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                       toe2d, heel2d, handSign) -> float:
        """S20: rotate each segment about the shaft axis through the apex, centred on the
        mean cone distance so the mid-face section stays UNROTATED."""
        rToe = _cone_distance(toeMid, apexWorld, coneVec)
        rHeel = _cone_distance(heelMid, apexWorld, coneVec)
        rMean = (rToe + rHeel) / 2.0
        span = rHeel - rToe

        # The conjugate crown-gear generation law, analytically, with no projection and no
        # curve sampling. gamma is this gear's PITCH cone angle — never the root cone angle,
        # which is a different, smaller angle and inflates the twist.
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(side.gamma)
        futil.log('bevel: {} total toe-to-heel twist {:.4f} deg'.format(
            side.label, math.degrees(total)))

        for segment in segments:
            (_, heelDistance) = self._endFace(segment, apexWorld, coneVec, True)
            angle = -handSign * total * (rMean - heelDistance) / span

            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(angle, axisDir, apexWorld)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(segment)
            moves = self._designComponent.features.moveFeatures
            moveInput = moves.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        return total

    def _crownSegments(self, side: _GearSide, segments, apexWorld: adsk.core.Point3D,
                       axisDir: adsk.core.Vector3D, coneVec: adsk.core.Vector3D,
                       toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D, total):
        """S21: scale each segment EXCEPT the outermost down by a monotonic factor, full at
        the heel and growing smoothly toward the toe."""
        if len(segments) == 0:
            return
        rToe = _cone_distance(toeMid, apexWorld, coneVec)
        rHeel = _cone_distance(heelMid, apexWorld, coneVec)
        span = rHeel - rToe

        # The heel face is recomputed HERE, after the twist has moved the slabs.
        keyed = []
        for segment in segments:
            (heelFace, heelDistance) = self._endFace(segment, apexWorld, coneVec, True)
            keyed.append((heelDistance, heelFace, segment))
        keyed.sort(key=lambda entry: entry[0])
        # The outermost segment's heel face is the loft's heel end and stays full, so the
        # heel cone trims it flush with the gear base.
        outermostIndex = len(keyed) - 1

        scales = self._designComponent.features.scaleFeatures
        # scaleFeatures is the one exception to never-activate
        # ([PB-CONSTRUCTION-NEEDS-ACTIVE]). Only an OCCURRENCE has activate; a component
        # does not.
        self._designOccurrence.activate()
        try:
            for index in range(len(keyed)):
                if index == outermostIndex:
                    continue
                (heelDistance, heelFace, segment) = keyed[index]
                u = (rHeel - heelDistance) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        'The {} tooth crowns segment at heel-distance fraction {:.4f} by a '
                        'factor of {:.4f}; a non-positive factor is never scaled '
                        'by.'.format(side.label, u, factor))

                basePoint = self._crownBasePoint(heelFace, apexWorld, axisDir)
                bodies = adsk.core.ObjectCollection.create()
                bodies.add(segment)
                scaleInput = scales.createInput(
                    bodies, basePoint, adsk.core.ValueInput.createByReal(factor))
                scales.add(scaleInput)
        finally:
            design: adsk.fusion.Design = self.design
            design.activateRootComponent()

    def _crownBasePoint(self, heelFace: adsk.fusion.BRepFace,
                        apexWorld: adsk.core.Point3D,
                        axisDir: adsk.core.Vector3D) -> adsk.fusion.SketchPoint:
        """S21: the scale base, on the heel face's ROOT edge rather than at its centroid.

        A uniform scale keeps every line through its base point where it is, so a base on the
        root keeps the root edge on the seating cone; a base at mid tooth height lifts the
        root and the tooth floats off the gear base. The base must be a sketch point, because
        construction points need an active component."""
        vertices = []
        for index in range(heelFace.vertices.count):
            vertex = heelFace.vertices.item(index)
            vertices.append(
                (_axis_distance(vertex.geometry, apexWorld, axisDir), vertex.geometry))
        vertices.sort(key=lambda entry: entry[0])
        # The two vertices nearest the shaft axis are the root corners; the tip corners are
        # farthest from it.
        first = vertices[0][1]
        second = vertices[1][1] if len(vertices) > 1 else vertices[0][1]
        midpoint = _wmid(first, second)

        sketch = self._designComponent.sketches.add(heelFace)
        sketch.name = 'Crown Base'
        # modelToSketchSpace is a point-transforming method ([PB-SPACE-METHODS]).
        return sketch.sketchPoints.add(sketch.modelToSketchSpace(midpoint))

    def _loftSpiralTooth(self, side: _GearSide, segments, apexWorld: adsk.core.Point3D,
                         coneVec: adsk.core.Vector3D) -> adsk.fusion.BRepBody:
        """S22: loft the curved tooth through the segments, in the order their heel faces sit
        along the cone element AFTER the twist and the crown ([PB-LOFT])."""
        keyed = []
        for segment in segments:
            (heelFace, heelDistance) = self._endFace(segment, apexWorld, coneVec, True)
            keyed.append((heelDistance, heelFace, segment))
        keyed.sort(key=lambda entry: entry[0])

        lofts = self._designComponent.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # The toe-most segment's apex-side face goes first, so the loft pushes past the toe
        # cone and the toe trim bites.
        (toeFace, _) = self._endFace(keyed[0][2], apexWorld, coneVec, False)
        loftInput.loftSections.add(toeFace)
        for (_, heelFace, _segment) in keyed:
            loftInput.loftSections.add(heelFace)
        loft = lofts.add(loftInput)
        body = loft.bodies.item(0)
        body.name = '{} Spiral Tooth'.format(side.label)

        # The loft has captured their faces, so the scaffolding goes.
        removes = self._designComponent.features.removeFeatures
        for (_, _heelFace, segment) in keyed:
            removes.add(segment)
        return body

    # --- S24 to S29: pattern, join, bore, mesh --------------------------------------------

    def _patternTeeth(self, side: _GearSide,
                      toothBody: adsk.fusion.BRepBody) -> adsk.fusion.CircularPatternFeature:
        """S24: circular-pattern the tooth around the SHAFT-AXIS EDGE — the same in-sketch
        profile edge the revolve used, never the §2 construction line."""
        entities = adsk.core.ObjectCollection.create()
        entities.add(toothBody)
        patterns = self._designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(entities, side.shaftEdge)
        # All three pinned explicitly ([PB-CIRCULAR-PATTERN]).
        patternInput.quantity = adsk.core.ValueInput.createByReal(side.teeth)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        return patterns.add(patternInput)

    def _combineTeeth(self, side: _GearSide,
                      pattern: adsk.fusion.CircularPatternFeature) -> adsk.fusion.BRepBody:
        """S25: one Combine-Join, the Gear Body as the target and every patterned tooth as a
        tool.

        The pattern's bodies already include the seed plus the copies, and they arrive as a
        BRepBodies the input rejects, so they are copied into an ObjectCollection
        ([PB-PATTERN-BODIES])."""
        tools = adsk.core.ObjectCollection.create()
        for index in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(index))

        combines = self._designComponent.features.combineFeatures
        combineInput = combines.createInput(side.gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combine = combines.add(combineInput)
        body = combine.bodies.item(0)
        body.name = '{} Gear'.format(side.label)
        return body

    def _cutBore(self, side: _GearSide,
                 body: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        """S26 to S28: the bore plane, the bore sketch and the symmetric through-cut. All
        three are skipped entirely when Enable Bore is unchecked."""
        if not self._boreEnable:
            return body

        # S26: normal to the shaft at its start, off the in-sketch profile edge. The edge
        # goes in directly, never wrapped in a path ([PB-CONSTRUCTION-PLANES]).
        planes = self._designComponent.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByDistanceOnPath(side.shaftEdge,
                                       adsk.core.ValueInput.createByReal(0.0))
        borePlane = planes.add(planeInput)
        borePlane.name = '{} Bore Plane'.format(side.label)

        # S27: the plane is rooted at the shaft's start, so the sketch origin is on the axis.
        sketch = self._designComponent.sketches.add(borePlane)
        sketch.name = '{} Bore'.format(side.label)
        diameter = side.boreDiameter
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), diameter / 2.0)
        # isFixed is the reliable pin; a coincident to the sketch origin has been observed to
        # fail the solve ([PB-CIRCLE-CENTER]).
        circle.centerSketchPoint.isFixed = True
        boreDimension = sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(diameter / 2.0, 0, 0))
        boreDimension.parameter.value = diameter
        self._gateSketch(sketch)

        # S28: a SYMMETRIC through-cut restricted to this gear's body, with no taper
        # argument ([PB-THROUGH-CUT]).
        extrudes = self._designComponent.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            sketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [body]
        extrude = extrudes.add(extrudeInput)
        return extrude.bodies.item(0)

    def _pinionMeshPhase(self, pinionTeeth) -> float:
        """S29: the pinion's extra mesh phase, in radians. It is 0 for every straight bevel,
        and 0 by default for a spiral one because the spiral build leaves the mid-face
        section unrotated."""
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    def _meshRotate(self, side: _GearSide, body: adsk.fusion.BRepBody):
        """S29: done HERE, in the Design component, before the body is moved out — a
        construction axis cannot be added in the moved-out gear component, so the rotation
        uses the profile edge's world geometry while the body is still in Design."""
        if side.isPinion:
            angle = self._pinionMeshPhase(side.teeth)
        else:
            # Half a tooth pitch, so a driving valley sits where the pinion tooth crosses.
            angle = math.pi / side.teeth
        rotate_body_about_edge(self._designComponent, body, side.shaftEdge, angle)
