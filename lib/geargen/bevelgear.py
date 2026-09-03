# Bevel gear pair generator, compiled from spec/bevelgear/steps.md.
#
# Precomputed mode ([PB-PRECOMPUTED-MODE]): every value is derived in Python, in internal
# centimetres, and written into the sketch/feature calls numerically. There are no live Fusion
# user parameters and no GenerationContext. This module does not subclass base.Generator.

import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .base import get_selection, get_boolean
from .misc import to_cm, to_mm, get_design
from .utilities import find_profile_by_curve_counts
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .solids import (
    cut_conical_ends,
    slice_body_by_offset_planes,
    rotate_body_about_edge,
    plane_by_angle,
    combine_point,
    circle_intersect_nearest,
    hide_construction_geometry,
)


# --- S02: the 17 dialog input ids, the two dropdown item names, and the spiral tunables -------

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

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# S25: the crown tunable, default 0.5 (0 disables the crown).
_CROWN_PER_RAD = 0.5
# S18: the pinion's extra mesh phase, in teeth. 0 because the mid-face section is unrotated and
# already meshes.
_PINION_MESH_PHASE_TEETH = 0


# --- private 2-D and 3-D vector helpers ---------------------------------------------------------
#
# These operate on plain (x, y) / (x, y, z) tuples, mirroring the closed-form lattice math the
# proof (proof/bevelgear/geometry_test.go, buildLattice/buildToe) checks. Every 2-D tuple here is
# expressed in the Gear Profiles sketch's own local frame -- the "c"/"perp" basis S06 seeds off,
# read at runtime rather than assumed -- so the arithmetic is valid regardless of which way Fusion
# actually orients that sketch.

def _v2add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _v2sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _v2scale(a, s):
    return (a[0] * s, a[1] * s)


def _v2dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _v2cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _v2len(a):
    return math.hypot(a[0], a[1])


def _v2unit(a):
    n = _v2len(a)
    return (a[0] / n, a[1] / n)


def _v2perp(a):
    return (-a[1], a[0])


def _v2rot(a, t):
    s, c = math.sin(t), math.cos(t)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _v2distToLine(p, origin, direction):
    return abs(_v2cross(direction, _v2sub(p, origin)))


def _v2intersect(p1, d1, p2, d2):
    den = _v2cross(d1, d2)
    return _v2add(p1, _v2scale(d1, _v2cross(_v2sub(p2, p1), d2) / den))


def _pt3(local2d):
    return adsk.core.Point3D.create(local2d[0], local2d[1], 0)


def _w3(point3d):
    return (point3d.x, point3d.y, point3d.z)


def _wpoint(world3d):
    return adsk.core.Point3D.create(world3d[0], world3d[1], world3d[2])


def _wvector(world3d):
    return adsk.core.Vector3D.create(world3d[0], world3d[1], world3d[2])


def _wsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _wadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _wscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _wdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _wcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _wlen(a):
    return math.sqrt(_wdot(a, a))


def _wunit(a):
    n = _wlen(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def _wmid(a, b):
    return _wscale(_wadd(a, b), 0.5)


def _distPointToAxis(p, axisPoint, axisDir):
    d = _wsub(p, axisPoint)
    along = _wdot(d, axisDir)
    perp = _wsub(d, _wscale(axisDir, along))
    return _wlen(perp)


# --- S06 closed-form lattice (mirrors proof/bevelgear/geometry_test.go buildLattice/buildToe) --

def _computeLattice(centre, perp, R_cm, gamma_p, gamma_g, sigma_rad,
                     ppd_cm, dpd_cm, module_cm,
                     drivingBaseHeight_cm, pinionBaseHeight_cm, toothSpacing_cm):
    apex = _v2add(centre, _v2scale(perp, R_cm * math.cos(gamma_g) + drivingBaseHeight_cm))
    drivingAxisPt = _v2add(centre, _v2scale(perp, drivingBaseHeight_cm))
    drivingAxisDir = _v2unit(_v2sub(drivingAxisPt, apex))

    reach = R_cm * math.cos(gamma_p)
    plus = _v2rot(drivingAxisDir, sigma_rad)
    minus = _v2rot(drivingAxisDir, -sigma_rad)
    pinionAxisDir = plus
    if _v2add(apex, _v2scale(minus, reach))[0] > _v2add(apex, _v2scale(plus, reach))[0]:
        pinionAxisDir = minus
    pinionAxisPt = _v2add(apex, _v2scale(pinionAxisDir, reach))

    drop = _v2perp(pinionAxisDir)
    if _v2dot(drop, _v2sub(drivingAxisPt, pinionAxisPt)) < 0:
        drop = _v2scale(drop, -1)
    apex2 = _v2add(pinionAxisPt, _v2scale(drop, ppd_cm / 2.0))

    pitchDir = _v2unit(_v2sub(apex2, apex))
    n = _v2perp(pitchDir)
    if (_v2distToLine(_v2add(apex2, n), apex, pinionAxisDir)
            > _v2distToLine(apex2, apex, pinionAxisDir)):
        n = _v2scale(n, -1)
    pinionDedDir = n
    drivingDedDir = _v2scale(n, -1)

    sides = {}
    for label, axisDir, axisPt, dedDir, baseHeight_cm, pitchDia_cm, gamma in (
        ('Pinion', pinionAxisDir, pinionAxisPt, pinionDedDir, pinionBaseHeight_cm, ppd_cm, gamma_p),
        ('Driving', drivingAxisDir, drivingAxisPt, drivingDedDir, drivingBaseHeight_cm, dpd_cm, gamma_g),
    ):
        ded = _v2add(apex2, _v2scale(dedDir, 1.25 * module_cm))
        h = _v2add(apex2, _v2scale(dedDir, baseHeight_cm / math.sin(gamma)))

        def foot(p, axisDir=axisDir):
            return _v2add(apex, _v2scale(axisDir, _v2dot(_v2sub(p, apex), axisDir)))

        e = foot(ded)
        g = foot(h)
        k = _v2add(apex2, _v2scale(dedDir, (pitchDia_cm / 2.0) / math.cos(gamma)))
        kPrime = _v2add(k, _v2scale(dedDir, toothSpacing_cm))
        sides[label] = dict(axisDir=axisDir, axis=axisPt, dedDir=dedDir, ded=ded,
                             e=e, g=g, h=h, k=k, kPrime=kPrime)

    return dict(apex=apex, apex2=apex2, sides=sides)


def _computeToe(apex, apex2, gLat, faceWidth_cm):
    dedDir = gLat['dedDir']
    ded = gLat['ded']
    axis = gLat['axis']

    toward = _v2perp(dedDir)
    if _v2dot(toward, _v2sub(apex, ded)) < 0:
        toward = _v2scale(toward, -1)
    base = _v2add(ded, _v2scale(toward, faceWidth_cm))
    rootDir = _v2unit(_v2sub(ded, apex))
    m = _v2intersect(base, dedDir, apex, rootDir)
    n = _v2intersect(base, dedDir, axis, _v2unit(_v2sub(apex2, axis)))
    return m, n


# --- S02: the dialog inputs ----------------------------------------------------------------

class BevelGearCommandInputsConfigurator:
    """Adds the 17 dialog inputs in their fixed row order and wires the spiral-only
    visibility toggle. Bound by name from commands/bevelgear/entry.py."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs
        design = get_design()

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
        parentInput.addSelection(design.rootComponent)

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
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)

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
        except Exception:
            return
        hand.isVisible = (value > 0)
        cutter.isVisible = (value > 0)


# --- the generator ----------------------------------------------------------------------------

class BevelGearGenerator:
    """Builds the Bevel Gear pair. Does not subclass base.Generator
    ([PB-PRECOMPUTED-MODE]) -- there is no live user parameter and no GenerationContext."""

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designComponent = adsk.fusion.Component.cast(None)
        self.bevelComponent = adsk.fusion.Component.cast(None)

    # --- S01: read and validate every dialog input --------------------------------------

    def _evalExpr(self, inputs: adsk.core.CommandInputs, inputId: str, unit: str):
        # [PB-EVAL-EXPRESSION]: always returns Fusion internal units.
        return self.design.unitsManager.evaluateExpression(
            inputs.itemById(inputId).expression, unit)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        targetPlane = get_selection(inputs, INPUT_ID_PLANE)[0]
        centerPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]
        parentEntity = get_selection(inputs, INPUT_ID_PARENT)[0]
        # The Parent Component selection filter admits an Occurrence or the RootComponent; a
        # generator always builds off a real Component.
        parentComponent = parentEntity
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentEntity.component

        module_mm = self._evalExpr(inputs, INPUT_ID_MODULE, '')
        shaftAngle_deg = math.degrees(self._evalExpr(inputs, INPUT_ID_SHAFT_ANGLE, 'deg'))
        drivingTeeth = int(round(self._evalExpr(inputs, INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(self._evalExpr(inputs, INPUT_ID_PINION_TEETH, '')))
        drivingBaseHeight_cm = self._evalExpr(inputs, INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeight_cm = self._evalExpr(inputs, INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = self._evalExpr(inputs, INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore_cm = self._evalExpr(inputs, INPUT_ID_PINION_BORE, 'mm')
        faceWidth_cm = self._evalExpr(inputs, INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing_cm = self._evalExpr(inputs, INPUT_ID_TOOTH_SPACING, 'mm')
        spiralAngle_rad = self._evalExpr(inputs, INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        cutterRadius_cm = self._evalExpr(inputs, INPUT_ID_CUTTER_RADIUS, 'mm')

        handInput = inputs.itemById(INPUT_ID_HAND)
        hand = handInput.selectedItem.name if handInput.selectedItem else _HAND_RIGHT

        # 1. ranges that need no derived geometry.
        if module_mm <= 0:
            raise Exception('Module must be greater than 0')
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')
        for label, value in (
            ('Driving Gear Base Height', drivingBaseHeight_cm),
            ('Pinion Gear Base Height', pinionBaseHeight_cm),
            ('Driving Gear Bore Diameter', drivingBore_cm),
            ('Pinion Gear Bore Diameter', pinionBore_cm),
            ('Face Width', faceWidth_cm),
            ('Tooth Spacing', toothSpacing_cm),
        ):
            if value < 0:
                raise Exception(f'{label} must be non-negative')
        if cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')
        if not (0 <= spiralAngle_deg < 60):
            raise Exception('Mean Spiral Angle must be at least 0 and below 60 deg')

        ppd_mm = module_mm * pinionTeeth
        dpd_mm = module_mm * drivingTeeth

        # 2. Maximum Shaft Angle: the cone-angle singularity, exclusive, capped at an
        # inclusive 150 deg; the floor is an inclusive 30 deg.
        coneLimitDeg = math.degrees(math.acos(-min(ppd_mm, dpd_mm) / max(ppd_mm, dpd_mm)))
        if coneLimitDeg <= 150:
            maxShaftAngleDeg = coneLimitDeg
            coneLimited = True
        else:
            maxShaftAngleDeg = 150.0
            coneLimited = False
        if shaftAngle_deg < 30:
            raise Exception('Shaft Angle must be at least 30 deg')
        if coneLimited and shaftAngle_deg >= maxShaftAngleDeg:
            raise Exception(f'Shaft Angle must be below {maxShaftAngleDeg:.2f} deg')
        if not coneLimited and shaftAngle_deg > maxShaftAngleDeg:
            raise Exception(f'Shaft Angle must be at most {maxShaftAngleDeg:.2f} deg')

        # 3. the two pitch cone angles, closed form.
        sigma_rad = math.radians(shaftAngle_deg)
        gamma_p = math.atan2(math.sin(sigma_rad) * ppd_mm, dpd_mm + ppd_mm * math.cos(sigma_rad))
        gamma_g = sigma_rad - gamma_p

        # 4. Minimum Teeth, on top of the blanket >= 3.
        pinionMinTeeth = 5.27 * math.cos(gamma_p)
        drivingMinTeeth = 5.27 * math.cos(gamma_g)
        if drivingTeeth < drivingMinTeeth:
            raise Exception(f'Driving Gear Teeth must be at least {drivingMinTeeth:.2f}')
        if pinionTeeth < pinionMinTeeth:
            raise Exception(f'Pinion Gear Teeth must be at least {pinionMinTeeth:.2f}')

        # 5. the base-height window per gear, driving first.
        def baseWindow(r_mm, gamma):
            maxBase = 0.95 * (r_mm - 1.25 * module_mm * math.cos(gamma)) * math.tan(gamma)
            minBase = 1.05 * 1.25 * module_mm * math.sin(gamma)
            return minBase, maxBase

        drivingMinBase, drivingMaxBase = baseWindow(dpd_mm / 2.0, gamma_g)
        drivingBaseHeight_mm_in = to_mm(drivingBaseHeight_cm)
        if drivingBaseHeight_mm_in == 0:
            fallback = module_mm * drivingTeeth / 8.0
            resolvedDrivingBase_mm = min(max(fallback, drivingMinBase), drivingMaxBase)
        else:
            if drivingBaseHeight_mm_in > drivingMaxBase:
                raise Exception(
                    f'Driving Gear Base Height must be at most {drivingMaxBase:.3f} mm')
            if drivingBaseHeight_mm_in < drivingMinBase:
                raise Exception(
                    f'Driving Gear Base Height must be at least {drivingMinBase:.3f} mm')
            resolvedDrivingBase_mm = drivingBaseHeight_mm_in

        pinionMinBase, pinionMaxBase = baseWindow(ppd_mm / 2.0, gamma_p)
        pinionBaseHeight_mm_in = to_mm(pinionBaseHeight_cm)
        if pinionBaseHeight_mm_in == 0:
            fallback = resolvedDrivingBase_mm * (pinionTeeth / drivingTeeth)
            resolvedPinionBase_mm = min(max(fallback, pinionMinBase), pinionMaxBase)
        else:
            if pinionBaseHeight_mm_in > pinionMaxBase:
                raise Exception(
                    f'Pinion Gear Base Height must be at most {pinionMaxBase:.3f} mm')
            if pinionBaseHeight_mm_in < pinionMinBase:
                raise Exception(
                    f'Pinion Gear Base Height must be at least {pinionMinBase:.3f} mm')
            resolvedPinionBase_mm = pinionBaseHeight_mm_in

        # 6. bore diameters: zero means auto, this gear's Pitch Diameter / 4.
        resolvedDrivingBore_mm = to_mm(drivingBore_cm)
        if resolvedDrivingBore_mm == 0:
            resolvedDrivingBore_mm = dpd_mm / 4.0
        resolvedPinionBore_mm = to_mm(pinionBore_cm)
        if resolvedPinionBore_mm == 0:
            resolvedPinionBore_mm = ppd_mm / 4.0

        self._drivingBaseHeight_cm = to_cm(resolvedDrivingBase_mm)
        self._pinionBaseHeight_cm = to_cm(resolvedPinionBase_mm)
        self._boreEnable = boreEnable
        self._drivingBore_cm = to_cm(resolvedDrivingBore_mm)
        self._pinionBore_cm = to_cm(resolvedPinionBore_mm)
        # The Face Width is not resolved here -- its cap needs the solved S06 geometry.
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm

        return (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth,
                pinionTeeth, shaftAngle_deg)

    # --- orchestration --------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth,
         pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        sigma_rad = math.radians(shaftAngle_deg)
        ppd_mm = module_mm * pinionTeeth
        dpd_mm = module_mm * drivingTeeth
        gamma_p = math.atan2(math.sin(sigma_rad) * ppd_mm, dpd_mm + ppd_mm * math.cos(sigma_rad))
        gamma_g = sigma_rad - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._coneDistance_cm = to_cm(math.hypot(dpd_mm, ppd_mm))
        R_cm = to_cm((ppd_mm / 2.0) / math.sin(gamma_p))

        module_cm = to_cm(module_mm)
        ppd_cm = to_cm(ppd_mm)
        dpd_cm = to_cm(dpd_mm)

        # S03: the occurrence tree.
        parentComponent: adsk.fusion.Component = parentComponent
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component
        designComponent = self.designComponent

        # S04
        anchorSketch, anchorLine, anchorCenterPoint = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)

        # S05
        gearProfilesPlane = self._buildGearProfilesPlane(designComponent, targetPlane, anchorLine)

        # S06
        lattice = self._buildGearProfilesSketch(
            designComponent, gearProfilesPlane, targetPlane, anchorLine, anchorCenterPoint,
            module_cm, ppd_cm, dpd_cm, R_cm, gamma_p, gamma_g, sigma_rad)

        gearSpecs = {
            'Pinion': dict(teeth=pinionTeeth, pitchDia_cm=ppd_cm, gamma=gamma_p,
                           boreDiameter_cm=self._pinionBore_cm),
            'Driving': dict(teeth=drivingTeeth, pitchDia_cm=dpd_cm, gamma=gamma_g,
                             boreDiameter_cm=self._drivingBore_cm),
        }

        # S07-S18 (and S19-S27 where spiral), pinion first.
        for gearLabel in ('Pinion', 'Driving'):
            spec = gearSpecs[gearLabel]
            self._buildGear(gearLabel, designComponent, gearProfilesPlane, lattice, module_mm,
                             spec['teeth'], spec['pitchDia_cm'], spec['gamma'],
                             spec['boreDiameter_cm'])

        # S28
        hide_construction_geometry(self.bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designComponent = adsk.fusion.Component.cast(None)
        self.bevelComponent = adsk.fusion.Component.cast(None)

    def _pinionMeshPhase(self, pinionTeeth):
        return _PINION_MESH_PHASE_TEETH * 2.0 * math.pi / pinionTeeth

    def _rawLine(self, sketch: adsk.fusion.Sketch, aLocal, bLocal) -> adsk.fusion.SketchLine:
        # [BEVEL-F-COINCIDENT-STYLE]: every S06 line is built from two fresh raw Point3D
        # coordinates, never a shared SketchPoint.
        # SketchLine (and SketchCurve/SketchEntity) declares no `name` member in the
        # compiled API reference -- only sketches, planes, bodies, components and
        # occurrences are named objects in this API -- so this helper does not try to
        # name the line; the comment above each call site records which S06 line it is.
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt3(aLocal), _pt3(bLocal))
        line.isConstruction = True
        return line

    # --- S04: Anchor sketch -----------------------------------------------------------------

    def _buildAnchorSketch(self, designComponent: adsk.fusion.Component, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint).item(0)
        centreGeom = projectedCenter.geometry

        start = adsk.core.Point3D.create(centreGeom.x - 0.5, centreGeom.y, 0)
        end = adsk.core.Point3D.create(centreGeom.x + 0.5, centreGeom.y, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)
        anchorLine.isConstruction = True

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        textPoint = adsk.core.Point3D.create(centreGeom.x, centreGeom.y + 0.3, 0)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)

        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        return sketch, anchorLine, projectedCenter

    # --- S05: Gear Profiles Plane ------------------------------------------------------------

    def _buildGearProfilesPlane(self, designComponent: adsk.fusion.Component, targetPlane,
                                anchorLine: adsk.fusion.SketchLine):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'),
                               targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'
        return gearProfilesPlane

    # --- S06: Gear Profiles sketch -- the two-gear lattice ------------------------------------

    def _buildGearProfilesSketch(self, designComponent: adsk.fusion.Component,
                                  gearProfilesPlane: adsk.fusion.ConstructionPlane, targetPlane,
                                  anchorLine: adsk.fusion.SketchLine,
                                  anchorCenterPoint: adsk.fusion.SketchPoint,
                                  module_cm, ppd_cm, dpd_cm,
                                  R_cm, gamma_p, gamma_g, sigma_rad):
        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'

        # Step 1: project the Anchor Sketch's centre point (never the raw user selection)
        # and the Anchor Line, so the direction and grow-side reads come off real geometry.
        projCentre = sketch.project(anchorCenterPoint).item(0)
        projAnchor = sketch.project(anchorLine).item(0)

        centreLocal = (projCentre.geometry.x, projCentre.geometry.y)
        aStartLocal = (projAnchor.startSketchPoint.geometry.x,
                        projAnchor.startSketchPoint.geometry.y)
        aEndLocal = (projAnchor.endSketchPoint.geometry.x,
                     projAnchor.endSketchPoint.geometry.y)
        d = _v2unit(_v2sub(aEndLocal, aStartLocal))
        perp0 = _v2perp(d)

        # [BEVEL-F-GROW-SIDE]: the sign of perp is the one bit read from the world -- grow
        # toward the target plane's normal.
        baseWorld = sketch.sketchToModelSpace(_pt3(centreLocal))
        testWorld = sketch.sketchToModelSpace(_pt3(_v2add(centreLocal, perp0)))
        moveVec = adsk.core.Vector3D.create(
            testWorld.x - baseWorld.x, testWorld.y - baseWorld.y, testWorld.z - baseWorld.z)
        normal = targetPlane.geometry.normal
        growSign = 1.0 if moveVec.dotProduct(normal) > 0 else -1.0
        perp = _v2scale(perp0, growSign)

        lat = _computeLattice(centreLocal, perp, R_cm, gamma_p, gamma_g, sigma_rad,
                               ppd_cm, dpd_cm, module_cm,
                               self._drivingBaseHeight_cm, self._pinionBaseHeight_cm,
                               self._toothSpacing_cm)
        apexLocal = lat['apex']
        apex2Local = lat['apex2']
        pinionLat = lat['sides']['Pinion']
        drivingLat = lat['sides']['Driving']

        gc = sketch.geometricConstraints
        sd = sketch.sketchDimensions
        AlignedDim = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        def rawLine(aLocal, bLocal):
            return self._rawLine(sketch, aLocal, bLocal)

        # Step 2: centre->apex, perpendicular to the projected Anchor Line. No length dim.
        centreToApex = rawLine(centreLocal, apexLocal)
        gc.addCoincident(centreToApex.startSketchPoint, projCentre)
        gc.addPerpendicular(centreToApex, projAnchor)
        apexPoint = centreToApex.endSketchPoint

        # Step 3: Driving Gear Shaft Axis, Apex->B. No length dim.
        drivingAxis = rawLine(apexLocal, drivingLat['axis'])
        gc.addCoincident(drivingAxis.startSketchPoint, apexPoint)
        gc.addParallel(drivingAxis, centreToApex)
        bPoint = drivingAxis.endSketchPoint

        # Step 4: Pinion Gear Shaft Axis, Apex->A. No length dim (Step 5 gives its angle).
        pinionAxis = rawLine(apexLocal, pinionLat['axis'])
        gc.addCoincident(pinionAxis.startSketchPoint, apexPoint)
        aPoint = pinionAxis.endSketchPoint

        # Step 5: the Shaft Angle, text point inside the Sigma wedge.
        bisector = _v2unit(_v2add(pinionLat['axisDir'], drivingLat['axisDir']))
        textPoint5 = _pt3(_v2add(apexLocal, _v2scale(bisector, ppd_cm / 4.0)))
        angDim = sd.addAngularDimension(drivingAxis, pinionAxis, textPoint5)
        angDim.parameter.value = sigma_rad

        # Step 6: the two perpendicular drops to Apex 2.
        dropA = rawLine(pinionLat['axis'], apex2Local)
        gc.addCoincident(dropA.startSketchPoint, aPoint)
        gc.addPerpendicular(pinionAxis, dropA)
        textDropA = _pt3(_v2add(pinionLat['axis'],
                                 _v2scale(_v2unit(_v2sub(apex2Local, pinionLat['axis'])),
                                          ppd_cm / 4.0)))
        distDimA = sd.addDistanceDimension(dropA.startSketchPoint, dropA.endSketchPoint,
                                            AlignedDim, textDropA)
        distDimA.parameter.value = ppd_cm / 2.0

        dropB = rawLine(drivingLat['axis'], apex2Local)
        gc.addCoincident(dropB.startSketchPoint, bPoint)
        gc.addPerpendicular(drivingAxis, dropB)
        textDropB = _pt3(_v2add(drivingLat['axis'],
                                 _v2scale(_v2unit(_v2sub(apex2Local, drivingLat['axis'])),
                                          dpd_cm / 4.0)))
        distDimB = sd.addDistanceDimension(dropB.startSketchPoint, dropB.endSketchPoint,
                                            AlignedDim, textDropB)
        distDimB.parameter.value = dpd_cm / 2.0

        gc.addCoincident(dropB.endSketchPoint, dropA.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # Step 7: Pitch Line, Apex->Apex2.
        pitchLine = rawLine(apexLocal, apex2Local)
        gc.addCoincident(pitchLine.startSketchPoint, apexPoint)
        gc.addCoincident(pitchLine.endSketchPoint, apex2Point)

        sides = {
            'Pinion': dict(axisEnd=aPoint, dropLine=dropA),
            'Driving': dict(axisEnd=bPoint, dropLine=dropB),
        }

        # Steps 8-15, once per gear.
        for label, gLat, axisLine in (
            ('Pinion', pinionLat, pinionAxis),
            ('Driving', drivingLat, drivingAxis),
        ):
            side = sides[label]
            axisEndPoint = side['axisEnd']
            dropLine = side['dropLine']

            # Step 8: the dedendum line, perpendicular to the Pitch Line.
            dedLine = rawLine(apex2Local, gLat['ded'])
            gc.addCoincident(dedLine.startSketchPoint, apex2Point)
            gc.addPerpendicular(pitchLine, dedLine)
            textDed = _pt3(_v2add(apex2Local, _v2scale(gLat['dedDir'], module_cm)))
            dedDim = sd.addDistanceDimension(dedLine.startSketchPoint, dedLine.endSketchPoint,
                                              AlignedDim, textDed)
            dedDim.parameter.value = 1.25 * module_cm
            dedEndPoint = dedLine.endSketchPoint

            # Step 9: the Root Axis, Apex->C / Apex->D.
            rootLine = rawLine(apexLocal, gLat['ded'])
            gc.addCoincident(rootLine.startSketchPoint, apexPoint)
            gc.addCoincident(rootLine.endSketchPoint, dedEndPoint)

            # Step 10: A->E collinear with Apex->A, one module (seeded, undimensioned);
            # C->E perpendicular to A->E.
            lineAE = rawLine(gLat['axis'], gLat['e'])
            gc.addCoincident(lineAE.startSketchPoint, axisEndPoint)
            gc.addCollinear(lineAE, axisLine)
            ePoint = lineAE.endSketchPoint

            lineCE = rawLine(gLat['ded'], gLat['e'])
            gc.addCoincident(lineCE.startSketchPoint, dedEndPoint)
            gc.addCoincident(lineCE.endSketchPoint, ePoint)
            gc.addPerpendicular(lineAE, lineCE)

            # Step 11: E->G collinear with line A->E (never the axis further up the chain,
            # [BEVEL-F-COLLINEAR-CHAIN]); C->H collinear with the dedendum line; G->H
            # perpendicular to E->G; the base-height offset dimension.
            lineEG = rawLine(gLat['e'], gLat['g'])
            gc.addCoincident(lineEG.startSketchPoint, ePoint)
            gc.addCollinear(lineEG, lineAE)
            gPoint = lineEG.endSketchPoint

            lineCH = rawLine(gLat['ded'], gLat['h'])
            gc.addCoincident(lineCH.startSketchPoint, dedEndPoint)
            gc.addCollinear(lineCH, dedLine)
            hPoint = lineCH.endSketchPoint

            lineGH = rawLine(gLat['g'], gLat['h'])
            gc.addCoincident(lineGH.startSketchPoint, gPoint)
            gc.addCoincident(lineGH.endSketchPoint, hPoint)
            gc.addPerpendicular(lineEG, lineGH)

            # Step 12: the resolved base height as the offset between the drop and G->H.
            # G->H and the drop are already parallel by construction -- no addParallel
            # ([PB-OFFSET-DIM]).
            textGH = _pt3(_v2scale(_v2add(gLat['g'], gLat['h']), 0.5))
            offGH = sd.addOffsetDimension(dropLine, lineGH, textGH)
            offGH.parameter.value = (self._pinionBaseHeight_cm if label == 'Pinion'
                                      else self._drivingBaseHeight_cm)

            # Step 13: the reference line A->G / B->I; the driving loop also closes the
            # figure by pinning I to the projected centre.
            lineAG = rawLine(gLat['axis'], gLat['g'])
            gc.addCoincident(lineAG.startSketchPoint, axisEndPoint)
            gc.addCoincident(lineAG.endSketchPoint, gPoint)
            if label == 'Driving':
                gc.addCoincident(gPoint, projCentre)

            # Step 14: K, pinned by two point-on-curve coincidents (never addCollinear --
            # G and C are already fixed, so a collinear over-constrains); the reference
            # line C->K.
            lineGK = rawLine(gLat['g'], gLat['k'])
            gc.addCoincident(lineGK.startSketchPoint, gPoint)
            gc.addCoincident(lineGK.endSketchPoint, axisLine)
            gc.addCoincident(lineGK.endSketchPoint, dedLine)
            kPoint = lineGK.endSketchPoint

            lineCK = rawLine(gLat['ded'], gLat['k'])
            gc.addCoincident(lineCK.startSketchPoint, dedEndPoint)
            gc.addCoincident(lineCK.endSketchPoint, kPoint)

            # Step 15: K' / L'. Zero Tooth Spacing reuses C->K; a positive spacing draws
            # K->K' and the tooth-centre reference line fresh.
            if self._toothSpacing_cm > 0:
                lineKKp = rawLine(gLat['k'], gLat['kPrime'])
                gc.addCoincident(lineKKp.startSketchPoint, kPoint)
                gc.addCoincident(lineKKp.endSketchPoint, dedLine)
                textKKp = _pt3(_v2add(gLat['k'],
                                       _v2scale(gLat['dedDir'], self._toothSpacing_cm / 2.0)))
                spacingDim = sd.addDistanceDimension(
                    lineKKp.startSketchPoint, lineKKp.endSketchPoint, AlignedDim, textKKp)
                spacingDim.parameter.value = self._toothSpacing_cm
                centreEndPoint = lineKKp.endSketchPoint

                centreLine = rawLine(gLat['ded'], gLat['kPrime'])
                gc.addCoincident(centreLine.startSketchPoint, dedEndPoint)
                gc.addCoincident(centreLine.endSketchPoint, centreEndPoint)
            else:
                centreEndPoint = kPoint
                centreLine = lineCK

            side.update(ded=dedEndPoint, h=hPoint, g=gPoint, dedLine=dedLine, rootLine=rootLine,
                        centreLine=centreLine, centrePoint=centreEndPoint,
                        dedDir=gLat['dedDir'], dedLocal=gLat['ded'], axisLocal=gLat['axis'])

        # Step 16: resolve the Maximum Face Width from the SOLVED geometry of A, B, C, D,
        # H, J ([PB-SOLVED-GEOMETRY]) -- never the seeds.
        aGeom = (aPoint.geometry.x, aPoint.geometry.y)
        bGeom = (bPoint.geometry.x, bPoint.geometry.y)
        cGeom = (sides['Pinion']['ded'].geometry.x, sides['Pinion']['ded'].geometry.y)
        dGeom = (sides['Driving']['ded'].geometry.x, sides['Driving']['ded'].geometry.y)
        hGeom = (sides['Pinion']['h'].geometry.x, sides['Pinion']['h'].geometry.y)
        jGeom = (sides['Driving']['h'].geometry.x, sides['Driving']['h'].geometry.y)

        pinDist = _v2distToLine(aGeom, cGeom, _v2unit(_v2sub(hGeom, cGeom)))
        drvDist = _v2distToLine(bGeom, dGeom, _v2unit(_v2sub(jGeom, dGeom)))
        maxFaceWidth_cm = 0.95 * min(pinDist, drvDist)

        if self._faceWidth_cm == 0:
            faceWidth_cm = min(self._coneDistance_cm / 6.0, maxFaceWidth_cm)
        else:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width must be at most {to_mm(maxFaceWidth_cm):.3f} mm')
            faceWidth_cm = self._faceWidth_cm
        self._faceWidthResolved_cm = faceWidth_cm

        # Step 17: M->N / O->P, the toe line, once per gear.
        names = {
            'Pinion': dict(m='M', n='N', a='A', c='C'),
            'Driving': dict(m='O', n='P', a='B', c='D'),
        }
        for label, gLat in (('Pinion', pinionLat), ('Driving', drivingLat)):
            side = sides[label]
            m, n = _computeToe(apexLocal, apex2Local, gLat, faceWidth_cm)
            nm = names[label]

            lineMN = rawLine(m, n)
            gc.addCoincident(lineMN.startSketchPoint, side['rootLine'])
            gc.addCoincident(lineMN.endSketchPoint, side['dropLine'])
            gc.addParallel(lineMN, side['dedLine'])
            textMN = _pt3(_v2scale(_v2add(m, gLat['ded']), 0.5))
            offMN = sd.addOffsetDimension(side['dedLine'], lineMN, textMN)
            offMN.parameter.value = faceWidth_cm
            mPoint = lineMN.startSketchPoint
            nPoint = lineMN.endSketchPoint

            lineMC = rawLine(m, gLat['ded'])
            gc.addCoincident(lineMC.startSketchPoint, mPoint)
            gc.addCoincident(lineMC.endSketchPoint, side['ded'])

            lineNA = rawLine(n, gLat['axis'])
            gc.addCoincident(lineNA.startSketchPoint, nPoint)
            gc.addCoincident(lineNA.endSketchPoint, side['axisEnd'])

            side['m'] = mPoint
            side['n'] = nPoint

        # Step 18: gate.
        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        return dict(sketch=sketch, apexPoint=apexPoint, sides=sides)

    # --- S07-S27: once per gear -------------------------------------------------------------

    def _buildGear(self, gearLabel, designComponent: adsk.fusion.Component,
                   gearProfilesPlane: adsk.fusion.ConstructionPlane, lattice, module_mm,
                   teeth, pitchDia_cm, gamma, boreDiameter_cm):
        side = lattice['sides'][gearLabel]
        centreLine = side['centreLine']
        centrePoint = side['centrePoint']
        apexSketchPoint = lattice['apexPoint']

        # S07: {gearLabel} Plane, through the tooth-centre reference line.
        toothPlane = plane_by_angle(designComponent, centreLine, gearProfilesPlane, 90)
        toothPlane.name = f'{gearLabel} Plane'

        # S08: {gearLabel} Tooth -- the virtual spur tooth.
        virtualPitchRadius_mm = (to_mm(pitchDia_cm) / 2.0) / math.cos(gamma)
        virtualTeeth = int(math.floor(2 * virtualPitchRadius_mm / module_mm))

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        proxy = VirtualSpurProxy(module_mm=module_mm, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centrePoint, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded
        if not toothSketch.isFullyConstrained:
            futil.log(f'{gearLabel} Tooth sketch is not fully constrained '
                      f'(expected: the along-path circle labels)')

        # S09: {gearLabel} Tooth Axis, through the tooth centre, normal to the plane the
        # tooth was drawn on.
        helperPlaneInput = designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(centreLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperPlaneInput)
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'

        # S10: the {gearLabel} Gear component, under Bevel Gear.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{gearLabel} Gear'

        # S11: {gearLabel} Profile -- the hexagon.
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'
        srcPoints = [side['axisEnd'], side['g'], side['h'], side['ded'], side['m'], side['n']]
        verts = [profileSketch.sketchPoints.add(profileSketch.modelToSketchSpace(p.worldGeometry))
                 for p in srcPoints]
        edges = []
        for i in range(6):
            line = profileSketch.sketchCurves.sketchLines.addByTwoPoints(
                verts[i], verts[(i + 1) % 6])
            edges.append(line)
        for edge in edges:
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True
        if not profileSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Profile sketch is not fully constrained')
        shaftAxisEdge = edges[0]

        # S12: revolve the hexagon into the Gear Body.
        hexProfile = profileSketch.profiles.item(0)
        revolveInput = designComponent.features.revolveFeatures.createInput(
            hexProfile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveFeature = designComponent.features.revolveFeatures.add(revolveInput)
        gearBody = revolveFeature.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # S13: loft the Apex point to the tooth profile.
        wantLines = 0 if embedded else 2
        toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        toothBody = loftFeature.bodies.item(0)

        # Caller obligations shared by S14/S19/S27.
        apexWorld = _w3(apexSketchPoint.worldGeometry)
        toeMidW = _wmid(_w3(side['m'].worldGeometry), _w3(side['n'].worldGeometry))
        heelMidW = _wmid(_w3(side['ded'].worldGeometry), _w3(side['h'].worldGeometry))
        toeConeW = _w3(side['m'].worldGeometry)
        heelConeW = _w3(side['ded'].worldGeometry)

        # S14 / S19-S27: the tooth-body hook.
        if self._spiralAngle_rad <= 0:
            trimmedTooth = cut_conical_ends(
                designComponent, toothBody, gearBody,
                _wpoint(toeMidW), _wpoint(heelMidW), _wpoint(apexWorld), gearLabel)
        else:
            baseHand = 1.0 if self._hand == _HAND_RIGHT else -1.0
            handSign = -baseHand if gearLabel == 'Pinion' else baseHand
            curvedTooth = self._buildCurvedTooth(
                gearLabel, designComponent, gearProfilesPlane, toothPlane,
                toothBody, toeMidW, heelMidW, toeConeW, heelConeW, apexWorld,
                shaftAxisEdge, gamma, handSign)
            # S27: flush-trim the curved tooth, the same caller obligations as S14.
            trimmedTooth = cut_conical_ends(
                designComponent, curvedTooth, gearBody,
                _wpoint(toeMidW), _wpoint(heelMidW), _wpoint(apexWorld), gearLabel)

        # S15: circular-pattern the tooth.
        patternBodies = adsk.core.ObjectCollection.create()
        patternBodies.add(trimmedTooth)
        patternInput = designComponent.features.circularPatternFeatures.createInput(
            patternBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(float(teeth))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternFeature = designComponent.features.circularPatternFeatures.add(patternInput)

        # S16: combine the teeth into the Gear Body.
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(patternFeature.bodies.count):
            toolBodies.add(patternFeature.bodies.item(i))
        combineInput = designComponent.features.combineFeatures.createInput(gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        designComponent.features.combineFeatures.add(combineInput)

        # S17: bore, only when Enable Bore is checked.
        if self._boreEnable:
            borePlaneInput = designComponent.constructionPlanes.createInput()
            borePlaneInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
            borePlane = designComponent.constructionPlanes.add(borePlaneInput)

            boreSketch = designComponent.sketches.add(borePlane)
            boreSketch.name = f'{gearLabel} Bore'
            circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2.0)
            circle.centerSketchPoint.isFixed = True
            diamDim = boreSketch.sketchDimensions.addDiameterDimension(
                circle, adsk.core.Point3D.create(boreDiameter_cm / 2.0, 0, 0))
            diamDim.parameter.value = boreDiameter_cm
            if not boreSketch.isFullyConstrained:
                raise Exception(f'{gearLabel} Bore sketch is not fully constrained')

            boreProfile = boreSketch.profiles.item(0)
            extrudeInput = designComponent.features.extrudeFeatures.createInput(
                boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
            extrudeInput.setSymmetricExtent(
                adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
            extrudeInput.participantBodies = [gearBody]
            designComponent.features.extrudeFeatures.add(extrudeInput)

        # S18: meshing rotation, in Design, before the body is moved out.
        if gearLabel == 'Driving':
            angle = math.pi / teeth
        else:
            angle = self._pinionMeshPhase(teeth)
        rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)
        gearBody.moveToComponent(gearOccurrence)

    # --- S19-S26: the spiral tooth-body hook -------------------------------------------------

    def _buildCurvedTooth(self, gearLabel, designComponent: adsk.fusion.Component,
                          gearProfilesPlane: adsk.fusion.ConstructionPlane,
                          parentToothPlane: adsk.fusion.ConstructionPlane,
                          toothBody: adsk.fusion.BRepBody, toeMidW, heelMidW, toeConeW, heelConeW,
                          apexWorld, shaftAxisEdge: adsk.fusion.SketchLine, gamma, handSign):
        # S19: the frame. Swap guard first -- the heel must be the outer end.
        axisStartW = _w3(shaftAxisEdge.startSketchPoint.worldGeometry)
        axisEndW = _w3(shaftAxisEdge.endSketchPoint.worldGeometry)
        axisDir = _wunit(_wsub(axisEndW, axisStartW))

        apexPt3: adsk.core.Point3D = _wpoint(apexWorld)
        if apexPt3.distanceTo(_wpoint(heelMidW)) < apexPt3.distanceTo(_wpoint(toeMidW)):
            toeMidW, heelMidW = heelMidW, toeMidW
            toeConeW, heelConeW = heelConeW, toeConeW

        coneVec = _wunit(_wsub(heelConeW, apexWorld))
        v = _wunit(_wcross(axisDir, coneVec))

        def distAlong(p):
            return _wdot(_wsub(p, apexWorld), coneVec)

        rToe = distAlong(toeMidW)
        rHeel = distAlong(heelMidW)
        rMean = 0.5 * (rToe + rHeel)
        span = rHeel - rToe

        # The Cone Element sketch: inspection only, and deliberately built from raw WORLD
        # coordinates with no modelToSketchSpace conversion ([BEVEL-F-...], see S21's note,
        # which also governs this sketch).
        coneElementSketch = designComponent.sketches.add(gearProfilesPlane)
        coneElementSketch.name = f'{gearLabel} Cone Element'
        coneEndW = _wadd(apexWorld, _wscale(coneVec, rHeel))
        coneElementLine = coneElementSketch.sketchCurves.sketchLines.addByTwoPoints(
            _wpoint(apexWorld), _wpoint(coneEndW))
        coneElementLine.isConstruction = True

        # S20: the Trace Plane.
        tracePlane = plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # S21: the 2D Tooth Trace -- the cutter arc. Inspection only, exempt from the
        # full-constraint gate.
        rc = self._cutterRadius_cm if self._cutterRadius_cm != 0 else rMean
        psi = self._spiralAngle_rad
        cx = rMean - rc * math.sin(psi)
        cy = handSign * rc * math.cos(psi)
        rLo = rToe - 0.06 * span
        rHi = rHeel + 0.06 * span
        toe2d = circle_intersect_nearest(rLo, cx, cy, rc, rMean, 0.0)
        heel2d = circle_intersect_nearest(rHi, cx, cy, rc, rMean, 0.0)

        def tanW(px, py):
            return combine_point(_wpoint(apexWorld), px, _wvector(coneVec), py, _wvector(v))

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        cutterCentre3 = tanW(cx, cy)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            cutterCentre3, rc)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        diamDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, tanW(cx + rc, cy))
        diamDim.parameter.value = 2 * rc

        toePt3 = tanW(toe2d[0], toe2d[1])
        meanPt3 = tanW(rMean, 0.0)
        heelPt3 = tanW(heel2d[0], heel2d[1])
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(toePt3, meanPt3, heelPt3)
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, meanPt3)
        radDim.parameter.value = rc

        # S22: slice the straight tooth into cross-section slabs.
        pieces = self._sliceToothIntoSlabs(
            designComponent, toothBody, parentToothPlane, apexWorld, span, gearLabel)

        # S23: order the segments and drop the apex scrap.
        segments = self._dropApexScrap(designComponent, pieces, apexWorld, coneVec, gearLabel)

        # S24: twist each segment about the shaft axis.
        phiCrown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phiCrown) / math.sin(gamma)
        self._twistSegments(designComponent, segments, apexWorld, axisDir, coneVec,
                            rMean, span, total, handSign)

        # S25: lengthwise crown.
        self._crownSegments(designComponent, segments, apexWorld, axisDir, coneVec,
                            rHeel, span, total, gearLabel)

        # S26: loft the crowned segments into the curved tooth.
        curvedTooth = self._loftCurvedTooth(designComponent, segments, apexWorld, coneVec,
                                            gearLabel)
        return curvedTooth

    def _faceExtremesByDistAlong(self, body: adsk.fusion.BRepBody, apexWorld, coneVec):
        least = (None, None)
        greatest = (None, None)
        for face in body.faces:
            c = _w3(face.centroid)
            val = _wdot(_wsub(c, apexWorld), coneVec)
            if greatest[1] is None or val > greatest[1]:
                greatest = (face, val)
            if least[1] is None or val < least[1]:
                least = (face, val)
        return least, greatest

    def _sliceToothIntoSlabs(self, designComponent: adsk.fusion.Component,
                             toothBody: adsk.fusion.BRepBody,
                             parentToothPlane: adsk.fusion.ConstructionPlane, apexWorld,
                             span, gearLabel):
        origin = _w3(parentToothPlane.geometry.origin)
        normal = _w3(parentToothPlane.geometry.normal)
        sign = 1.0 if _wdot(_wsub(apexWorld, origin), normal) > 0 else -1.0

        def offsetsFor(s):
            return [s * (k + 1) * span / 6.0 for k in range(8)]

        pieces = slice_body_by_offset_planes(
            designComponent, toothBody, parentToothPlane, offsetsFor(sign))
        if len(pieces) <= 1:
            sign = -sign
            pieces = slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsetsFor(sign))
        if len(pieces) <= 1:
            raise Exception(
                f'{gearLabel}: slice produced {len(pieces)} piece(s) after trying both signs '
                f'(span={span:.6f} cm); the parent plane may sit outside the tooth span')
        return pieces

    def _dropApexScrap(self, designComponent: adsk.fusion.Component, pieces, apexWorld, coneVec,
                       gearLabel):
        def distAlong(body):
            c = _w3(body.physicalProperties.centerOfMass)
            return _wdot(_wsub(c, apexWorld), coneVec)

        piecesSorted = sorted(pieces, key=distAlong)
        scrap = piecesSorted[0]
        segments = piecesSorted[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(f'{gearLabel}: no segments remain after dropping the apex scrap')
        return segments

    def _rotateBodyFreeMove(self, designComponent: adsk.fusion.Component,
                            body: adsk.fusion.BRepBody, angleRad,
                            axisVec3: adsk.core.Vector3D, originPt3: adsk.core.Point3D):
        # A zero-angle identity transform raises `invalid transform` in Fusion; absorb it as
        # a no-op, matching solids.rotate_body_about_edge's own guard.
        if angleRad == 0:
            return
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angleRad, axisVec3, originPt3)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = designComponent.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(matrix)
        designComponent.features.moveFeatures.add(moveInput)

    def _twistSegments(self, designComponent: adsk.fusion.Component, segments, apexWorld, axisDir,
                       coneVec, rMean, span, total, handSign):
        axisVec3 = _wvector(axisDir)
        apexPt3 = _wpoint(apexWorld)
        for seg in segments:
            _, (heelFace, heelVal) = self._faceExtremesByDistAlong(seg, apexWorld, coneVec)
            ang = -handSign * total * (rMean - heelVal) / span
            self._rotateBodyFreeMove(designComponent, seg, ang, axisVec3, apexPt3)

    def _crownSegments(self, designComponent: adsk.fusion.Component, segments, apexWorld, axisDir,
                       coneVec, rHeel, span, total, gearLabel):
        info = []
        for seg in segments:
            _, (heelFace, heelVal) = self._faceExtremesByDistAlong(seg, apexWorld, coneVec)
            info.append((seg, heelFace, heelVal))
        # Sort by post-twist heel-face cone distance, greatest (outermost/heel) first.
        info.sort(key=lambda t: t[2], reverse=True)

        for seg, heelFace, heelVal in info[1:]:
            u = (rHeel - heelVal) / span
            factor = 1.0 - _CROWN_PER_RAD * (abs(total) / 2.0) * u
            if factor <= 0:
                raise Exception(
                    f'{gearLabel}: crown factor {factor:.6f} at u={u:.6f} is not positive')

            ranked = []
            for vertex in heelFace.vertices:
                p = _w3(vertex.geometry)
                ranked.append((_distPointToAxis(p, apexWorld, axisDir), p))
            ranked.sort(key=lambda t: t[0])
            basePtWorld = _wmid(ranked[0][1], ranked[1][1])

            try:
                self.designOccurrence.activate()
                faceSketch = designComponent.sketches.add(heelFace)
                baseSketchPoint = faceSketch.sketchPoints.add(
                    faceSketch.modelToSketchSpace(_wpoint(basePtWorld)))
                scaleBodies = adsk.core.ObjectCollection.create()
                scaleBodies.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    scaleBodies, baseSketchPoint, adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
            finally:
                self.design.activateRootComponent()

    def _loftCurvedTooth(self, designComponent: adsk.fusion.Component, segments, apexWorld,
                         coneVec, gearLabel):
        # Re-sort by post-twist, post-crown heel-face cone distance -- toe to heel.
        order = sorted(segments,
                        key=lambda seg: self._faceExtremesByDistAlong(seg, apexWorld, coneVec)[1][1])

        (toeFace, _), _ = self._faceExtremesByDistAlong(order[0], apexWorld, coneVec)
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toeFace)
        for seg in order:
            _, (heelFace, _) = self._faceExtremesByDistAlong(seg, apexWorld, coneVec)
            loftInput.loftSections.add(heelFace)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        curvedTooth = loftFeature.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in order:
            designComponent.features.removeFeatures.add(seg)

        return curvedTooth
