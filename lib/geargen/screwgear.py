import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .misc import get_design
from .base import Generator
from .utilities import find_profile_by_curve_counts
from . import solids


INPUT_ID_PLANE = 'plane'
INPUT_ID_POINT = 'point'
INPUT_ID_PARENT = 'parent'
INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'
INPUT_ID_TOOTH_COUNT = 'toothCount'
INPUT_ID_TWIST_LEAD = 'twistLead'
INPUT_ID_RIBBON_THICKNESS = 'ribbonThickness'
INPUT_ID_TOOTH_PITCH = 'toothPitch'
INPUT_ID_TOOTH_HEIGHT = 'toothHeight'
INPUT_ID_CAGE_RADIUS = 'cageRadius'
INPUT_ID_CAGE_RISE = 'cageRise'
INPUT_ID_CLEARANCE = 'clearance'
INPUT_ID_COLLAR_HALF = 'collarHalf'
INPUT_ID_COLLAR_WALL = 'collarWall'
INPUT_ID_CROSS_ANGLE = 'crossAngle'
INPUT_ID_ENGAGEMENT = 'engagement'
INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'
INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'
INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'

CELL_TEETH = 4


# ---------------------------------------------------------------------------
# Private vector/geometry helpers. Every one of these is plain-Python math on
# (x, y, z) tuples; the only place a tuple becomes a Fusion object is where a
# method below hands it to a sketch/plane/body call.
# ---------------------------------------------------------------------------

def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vlen(a):
    return math.sqrt(_vdot(a, a))


def _vnorm(a):
    length = _vlen(a)
    return (a[0] / length, a[1] / length, a[2] / length)


def _point3d(p) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(p[0], p[1], p[2])


def _vector3d(v) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


def _from_point3d(p):
    return (p.x, p.y, p.z)


def _from_vector3d(v):
    return (v.x, v.y, v.z)


def _fold_pi(angle):
    two_pi = 2 * math.pi
    wrapped = (angle + math.pi) % two_pi
    if wrapped <= 0:
        wrapped += two_pi
    return wrapped - math.pi


def _clip_polygon(poly, alpha, beta, gamma):
    # Keep the part of a convex polygon where alpha*x + beta*y <= gamma.
    if not poly:
        return poly
    out = []
    count = len(poly)
    for i in range(count):
        p = poly[i]
        r = poly[(i + 1) % count]
        fp = gamma - (alpha * p[0] + beta * p[1])
        fr = gamma - (alpha * r[0] + beta * r[1])
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fr >= 0):
            k = fp / (fp - fr)
            out.append((p[0] + k * (r[0] - p[0]), p[1] + k * (r[1] - p[1])))
    return out


def _convex_hull(points2d):
    # Andrew's monotone chain, counter-clockwise.
    pts = sorted(set(points2d))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


class _Gear:
    __slots__ = ('origin', 'dir', 'u', 'v', 'phi', 'z0', 'label')

    def __init__(self, origin, dir, u, v, phi, z0, label):
        self.origin = origin
        self.dir = dir
        self.u = u
        self.v = v
        self.phi = phi
        self.z0 = z0
        self.label = label


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs = command.commandInputs

        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to")
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        pointInput = inputs.addSelectionInput(
            INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1, 1)

        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        ribbonGroup = inputs.addGroupCommandInput('ribbonGroup', 'Ribbon')
        ribbonGroup.isExpanded = True
        ribbonChildren = ribbonGroup.children
        ribbonChildren.addValueInput(
            INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm',
            adsk.core.ValueInput.createByReal(1.5))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_COUNT, 'Tooth Count', '',
            adsk.core.ValueInput.createByReal(68))
        ribbonChildren.addValueInput(
            INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm',
            adsk.core.ValueInput.createByReal(4.95))
        ribbonChildren.addValueInput(
            INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm',
            adsk.core.ValueInput.createByReal(0.375))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm',
            adsk.core.ValueInput.createByReal(0.2625))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm',
            adsk.core.ValueInput.createByReal(0.2625))

        frameGroup = inputs.addGroupCommandInput('frameGroup', 'Frame')
        frameGroup.isExpanded = True
        frameChildren = frameGroup.children
        frameChildren.addValueInput(
            INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm',
            adsk.core.ValueInput.createByReal(1.5))
        frameChildren.addValueInput(
            INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm',
            adsk.core.ValueInput.createByReal(1.875))
        frameChildren.addValueInput(
            INPUT_ID_CLEARANCE, 'Clearance', 'mm',
            adsk.core.ValueInput.createByReal(0.045))
        frameChildren.addValueInput(
            INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm',
            adsk.core.ValueInput.createByReal(0.3))
        frameChildren.addValueInput(
            INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm',
            adsk.core.ValueInput.createByReal(0.3))

        meshGroup = inputs.addGroupCommandInput(
            'meshGroup', 'Mesh (from the mesh search)')
        meshGroup.isExpanded = False
        meshChildren = meshGroup.children
        meshChildren.addValueInput(
            INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg',
            adsk.core.ValueInput.createByReal(80 * math.pi / 180))
        meshChildren.addValueInput(
            INPUT_ID_ENGAGEMENT, 'Engagement', 'mm',
            adsk.core.ValueInput.createByReal(0.075))
        meshChildren.addValueInput(
            INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg',
            adsk.core.ValueInput.createByReal(15 * math.pi / 180))
        meshChildren.addValueInput(
            INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg',
            adsk.core.ValueInput.createByReal(15 * math.pi / 180))
        meshChildren.addValueInput(
            INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm',
            adsk.core.ValueInput.createByReal(-0.131))


class ScrewGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'ScrewGear'

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        self.buildGear(0)
        self.buildGear(1)
        self.buildCage()
        self.relocateBodies()
        solids.hide_construction_geometry(self.designOcc.component)

    # -----------------------------------------------------------------
    # S03, S04, S05 - read and check the inputs, then the two searches.
    # -----------------------------------------------------------------

    def _read_input_value(self, inputs: adsk.core.CommandInputs, inputId: str, unit: str) -> float:
        valueInput = adsk.core.ValueCommandInput.cast(inputs.itemById(inputId))
        if valueInput is None:
            raise Exception(f'missing input "{inputId}"')
        design = get_design()
        unitsManager: adsk.core.UnitsManager = design.unitsManager
        return unitsManager.evaluateExpression(valueInput.expression, unit)

    def _read_input_mm(self, inputs: adsk.core.CommandInputs, inputId: str) -> float:
        # evaluateExpression always returns internal units (cm for a length)
        # regardless of the unit string passed; convert to mm for every
        # check and search ([PB-EVAL-EXPRESSION]).
        return self._read_input_value(inputs, inputId, 'mm') * 10.0

    def _read_input_deg(self, inputs: adsk.core.CommandInputs, inputId: str) -> float:
        return self._read_input_value(inputs, inputId, 'deg')

    def processInputs(self, inputs: adsk.core.CommandInputs):
        selections = {}
        for inputId in (INPUT_ID_PLANE, INPUT_ID_POINT, INPUT_ID_PARENT):
            selectionInput = adsk.core.SelectionCommandInput.cast(inputs.itemById(inputId))
            if selectionInput.selectionCount != 1:
                raise Exception(f'"{inputId}" must have exactly one selection')
            selections[inputId] = selectionInput.selection(0).entity

        self.plane = selections[INPUT_ID_PLANE]
        self.point = selections[INPUT_ID_POINT]

        parentEntity = selections[INPUT_ID_PARENT]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        elif parentEntity.objectType == adsk.fusion.Component.classType():
            self.parentComponent = parentEntity
        else:
            raise Exception('"parent" must be an occurrence or a root component')

        toothCountRaw = self._read_input_value(inputs, INPUT_ID_TOOTH_COUNT, '')

        W = self._read_input_mm(inputs, INPUT_ID_RIBBON_WIDTH)
        T = self._read_input_mm(inputs, INPUT_ID_RIBBON_THICKNESS)
        H = self._read_input_mm(inputs, INPUT_ID_TOOTH_HEIGHT)
        P = self._read_input_mm(inputs, INPUT_ID_TOOTH_PITCH)
        lead = self._read_input_mm(inputs, INPUT_ID_TWIST_LEAD)
        Sigma = self._read_input_deg(inputs, INPUT_ID_CROSS_ANGLE)
        Eng = self._read_input_mm(inputs, INPUT_ID_ENGAGEMENT)
        PhiA = self._read_input_deg(inputs, INPUT_ID_MOUNT_ANGLE_A)
        PhiB = self._read_input_deg(inputs, INPUT_ID_MOUNT_ANGLE_B)
        Z0B = self._read_input_mm(inputs, INPUT_ID_ASSEMBLY_PHASE)
        cageRadius = self._read_input_mm(inputs, INPUT_ID_CAGE_RADIUS)
        cageRise = self._read_input_mm(inputs, INPUT_ID_CAGE_RISE)
        clearance = self._read_input_mm(inputs, INPUT_ID_CLEARANCE)
        collarHalf = self._read_input_mm(inputs, INPUT_ID_COLLAR_HALF)
        collarWall = self._read_input_mm(inputs, INPUT_ID_COLLAR_WALL)

        for fieldName, value in (
            ('ribbonWidth', W), ('ribbonThickness', T), ('toothPitch', P),
            ('twistLead', lead), ('collarHalf', collarHalf),
            ('collarWall', collarWall), ('clearance', clearance),
        ):
            if not value > 0:
                raise Exception(f'"{fieldName}" must be greater than 0 mm, got {value} mm')

        if abs(toothCountRaw - round(toothCountRaw)) >= 1e-9:
            raise Exception(f'"toothCount" must be a whole number, got {toothCountRaw}')
        N = int(round(toothCountRaw))
        if N < 4:
            raise Exception(f'"toothCount" must be at least 4, got {N}')

        if not (H > 0 and H < W / 2):
            raise Exception(
                f'"toothHeight" must be greater than 0 and less than half the ribbon width '
                f'({W / 2} mm), got {H} mm')

        if not (Eng > 0 and Eng <= H):
            raise Exception(
                f'"engagement" must be greater than 0 and at most the tooth height ({H} mm), '
                f'got {Eng} mm')

        if not (0 < Sigma < math.pi):
            raise Exception(
                f'"crossAngle" must lie strictly between 0 and 180 degrees, '
                f'got {math.degrees(Sigma)} deg')

        if not (abs(Z0B) < P):
            raise Exception(
                f'"assemblyPhase" must lie strictly within plus or minus the tooth pitch '
                f'({P} mm), got {Z0B} mm')

        lengthCheck = N * P
        if not (cageRadius + collarHalf + 1 < lengthCheck / 2):
            raise Exception(
                f'"cageRadius" places a bore outside the ribbon: cageRadius + collarHalf + '
                f'1 mm ({cageRadius + collarHalf + 1} mm) must be less than half the ribbon '
                f'length ({lengthCheck / 2} mm)')

        Lambda = lead / (2 * math.pi)
        A = W - Eng
        L = N * P
        c = min(CELL_TEETH, N)
        stepsPerTooth = max(int(math.ceil((P / Lambda) / (2 * math.pi / 180))), 8)
        q = N // c
        r = N % c
        Ri = cageRadius - collarHalf
        Ro = cageRadius + collarHalf
        hw = W / 2 + clearance
        ht = T / 2 + clearance
        cc = math.hypot(hw, ht)

        if not (math.sqrt(cc * cc + 1) < Ri):
            sIn = math.sqrt(max(0.0, Ri * Ri - cc * cc)) - 1
            raise Exception(
                f'"cageRadius": the bore\'s corner with the cut\'s 1 mm margin reaches the '
                f'sleeve\'s inner radius (sIn = {sIn} mm)')

        sIn = math.sqrt(Ri * Ri - cc * cc) - 1
        sOut = Ro + 1
        axialWindow = 1.5 * math.sqrt(W * W - A * A) / math.sin(Sigma)
        twist = (sOut - sIn) / Lambda

        self.W, self.T, self.H, self.P, self.N = W, T, H, P, N
        self.lead, self.Sigma, self.Eng = lead, Sigma, Eng
        self.PhiA, self.PhiB, self.Z0B = PhiA, PhiB, Z0B
        self.cageRadius, self.cageRise = cageRadius, cageRise
        self.clearance, self.collarHalf, self.collarWall = clearance, collarHalf, collarWall
        self.Lambda, self.A, self.L = Lambda, A, L
        self.c, self.stepsPerTooth, self.q, self.r = c, stepsPerTooth, q, r
        self.Ri, self.Ro = Ri, Ro
        self.hw, self.ht, self.cc = hw, ht, cc
        self.sIn, self.sOut = sIn, sOut
        self.axialWindow = axialWindow
        self.twist = twist

        self._checkSleeve()
        self.windows = []
        self._searchWindows()

    def _place_gears(self, C, e, n, halfA):
        # The frame of S04/S09: gear A and gear B placed from C, e, n. halfA
        # is A/2 already converted into the caller's length unit (mm for the
        # dry searches, cm for the real build).
        k = _vcross(n, e)
        half = self.Sigma / 2
        dirA = _vadd(_vscale(e, math.cos(half)), _vscale(k, math.sin(half)))
        originA = _vsub(C, _vscale(n, halfA))
        uA = n
        vA = _vcross(dirA, uA)
        dirB = _vsub(_vscale(e, math.cos(half)), _vscale(k, math.sin(half)))
        originB = _vadd(C, _vscale(n, halfA))
        uB = _vscale(n, -1.0)
        vB = _vcross(dirB, uB)
        gearA = _Gear(origin=originA, dir=dirA, u=uA, v=vA, phi=self.PhiA, z0=0.0, label='Gear A')
        gearB = _Gear(origin=originB, dir=dirB, u=uB, v=vB, phi=self.PhiB, z0=self.Z0B, label='Gear B')
        return (gearA, gearB)

    def _gear_theta(self, gear, s):
        return s / self.Lambda + gear.phi

    def _gear_world_mm(self, gear, u, v, s):
        # Turned section coordinates (u, v) at station s, in mm, no
        # conversion to cm: used only by the dry S04/S05 searches.
        theta = self._gear_theta(gear, s)
        c, sn = math.cos(theta), math.sin(theta)
        x = u * c - v * sn
        y = u * sn + v * c
        axisPoint = _vadd(gear.origin, _vscale(gear.dir, s))
        return _vadd(_vadd(axisPoint, _vscale(gear.u, x)), _vscale(gear.v, y))

    def _gear_point(self, gear, u, v, s):
        # Turned section coordinates (u, v) at station s, in mm, converted
        # to a world Fusion point in cm.
        theta = self._gear_theta(gear, s)
        c, sn = math.cos(theta), math.sin(theta)
        x = u * c - v * sn
        y = u * sn + v * c
        axisPoint = _vadd(gear.origin, _vscale(gear.dir, s / 10.0))
        return _vadd(_vadd(axisPoint, _vscale(gear.u, x / 10.0)), _vscale(gear.v, y / 10.0))

    def _gear_raw_point(self, gear, s, alongU=0.0, alongV=0.0):
        # A point along the gear's own axis plus an UNROTATED offset along
        # u_g/v_g (mm), converted to a world Fusion point in cm.
        axisPoint = _vadd(gear.origin, _vscale(gear.dir, s / 10.0))
        return _vadd(_vadd(axisPoint, _vscale(gear.u, alongU / 10.0)), _vscale(gear.v, alongV / 10.0))

    def _bore_crossing(self, gear, sigma):
        return _vadd(gear.origin, _vscale(gear.dir, sigma * self.cageRadius))

    def _dry_frame(self):
        C0 = (0.0, 0.0, 0.0)
        e0 = (1.0, 0.0, 0.0)
        n0 = (0.0, 0.0, 1.0)
        k0 = _vcross(n0, e0)
        return C0, e0, n0, k0

    def _dry_bores(self, gears):
        return (
            (gears[0], -1.0, 'gear A -R'),
            (gears[0], 1.0, 'gear A +R'),
            (gears[1], -1.0, 'gear B -R'),
            (gears[1], 1.0, 'gear B +R'),
        )

    def _channel_outline(self, gear, sigma, C, e, k):
        hw, ht, Ri, Ro = self.hw, self.ht, self.Ri, self.Ro
        sIn, sOut = self.sIn, self.sOut
        points = []
        s = sigma * sIn
        while abs(s) <= sOut + 1e-9:
            for i in range(17):
                frac = i / 16.0
                for (u, v) in (
                    (hw, -ht + 2 * ht * frac),
                    (-hw, -ht + 2 * ht * frac),
                    (-hw + 2 * hw * frac, ht),
                    (-hw + 2 * hw * frac, -ht),
                ):
                    world = self._gear_world_mm(gear, u, v, s)
                    rel = _vsub(world, C)
                    radial = math.hypot(_vdot(rel, e), _vdot(rel, k))
                    if Ri - 0.5 <= radial <= Ro + 0.5:
                        points.append(world)
            s += sigma * 0.1
        return points

    def _channel_separation(self, outlineByBore, d, bore1, bore2, C, n):
        across = _vcross(n, d)

        def project(points):
            return [(_vdot(_vsub(p, C), across), _vdot(_vsub(p, C), n)) for p in points]

        proj1 = project(outlineByBore[bore1])
        proj2 = project(outlineByBore[bore2])
        hull1 = _convex_hull(proj1)
        hull2 = _convex_hull(proj2)

        best = -float('inf')
        for hull in (hull1, hull2):
            count = len(hull)
            if count < 2:
                continue
            for i in range(count):
                a, b = hull[i], hull[(i + 1) % count]
                mx, my = b[1] - a[1], a[0] - b[0]
                length = math.hypot(mx, my)
                if length == 0:
                    continue
                mx, my = mx / length, my / length

                def proj_m(pt):
                    return pt[0] * mx + pt[1] * my

                ps = [proj_m(p) for p in hull1]
                qs = [proj_m(p) for p in hull2]
                sep = max(min(qs) - max(ps), min(ps) - max(qs))
                best = max(best, sep)
        return best

    def _checkSleeve(self):
        # S04: the sleeve's four checks, and the wall between the bores.
        Ri, A, cc = self.Ri, self.A, self.cc
        W, T, clearance = self.W, self.T, self.clearance
        cageRise, collarWall = self.cageRise, self.collarWall
        axialWindow = self.axialWindow

        meshReach = math.sqrt(axialWindow ** 2 + (W / 2) ** 2 + (T / 2) ** 2) + clearance
        if not (meshReach <= Ri):
            raise Exception(
                f'"cageRadius": the mesh does not stay visible along the axis '
                f'({meshReach} mm against Ri = {Ri} mm)')

        minCageRise = A / 2 + cc + collarWall
        if not (cageRise >= minCageRise):
            raise Exception(
                f'"cageRise" must be at least {minCageRise} mm to keep collarWall at the end '
                f'faces, got {cageRise} mm')

        C0, e0, n0, k0 = self._dry_frame()
        gearsMm = self._place_gears(C0, e0, n0, A / 2)
        bores = self._dry_bores(gearsMm)
        outlines = [self._channel_outline(g, sigma, C0, e0, k0) for (g, sigma, _) in bores]

        gaps = (
            (k0, 1, 2),
            (_vscale(e0, -1.0), 2, 0),
            (_vscale(k0, -1.0), 0, 3),
            (e0, 3, 1),
        )
        worstSeparation = float('inf')
        worstPair = ('', '')
        for d, i, j in gaps:
            sep = self._channel_separation(outlines, d, i, j, C0, n0)
            if sep < worstSeparation:
                worstSeparation = sep
                worstPair = (bores[i][2], bores[j][2])

        if not (worstSeparation >= collarWall):
            raise Exception(
                f'"collarWall": the wall between {worstPair[0]} and {worstPair[1]} is only '
                f'{worstSeparation} mm, less than collarWall ({collarWall} mm)')

    # -----------------------------------------------------------------
    # S05: the window search. Every helper below works in the dry mm-only
    # frame of S04/S05, before any Fusion feature exists.
    # -----------------------------------------------------------------

    def _section_in_wall(self, gear, s, C, n):
        if abs(s) >= self.Ro:
            return ([], [])
        hw, ht = self.hw, self.ht
        theta = self._gear_theta(gear, s)
        c, sn = math.cos(theta), math.sin(theta)
        rect = []
        for (u, v) in ((-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht)):
            rect.append((u * c - v * sn, u * sn + v * c))

        originRel = _vsub(gear.origin, C)
        uDotN = _vdot(gear.u, n)
        originDotN = _vdot(originRel, n)
        xa = (-self.cageRise - originDotN) / uDotN
        xb = (self.cageRise - originDotN) / uDotN
        if xa > xb:
            xa, xb = xb, xa
        rect = _clip_polygon(rect, 1, 0, xb)
        rect = _clip_polygon(rect, -1, 0, -xa)

        near = math.sqrt(max(0.0, self.Ri ** 2 - s ** 2))
        far = math.sqrt(max(0.0, self.Ro ** 2 - s ** 2))
        pieces = []
        for side in (1, -1):
            piece = _clip_polygon(rect, 0, side, far)
            piece = _clip_polygon(piece, 0, -side, -near)
            pieces.append(piece)
        return tuple(pieces)

    def _wall_corners(self, gear, sigma, C, n, across):
        lo = -self.sOut if sigma < 0 else self.sIn
        hi = -self.sIn if sigma < 0 else self.sOut
        pts = []
        steps = max(0, int(round((hi - lo) / 0.001)))
        for i in range(steps + 1):
            s = lo + i * 0.001
            for piece in self._section_in_wall(gear, s, C, n):
                for (x, y) in piece:
                    world = _vadd(
                        _vadd(_vadd(gear.origin, _vscale(gear.u, x)), _vscale(gear.v, y)),
                        _vscale(gear.dir, s))
                    rel = _vsub(world, C)
                    t = _vdot(rel, across)
                    z = _vdot(rel, n)
                    pts.append((world, t, z))
        return pts

    def _channel_top(self, bores):
        hw, ht, Ri, Ro, A = self.hw, self.ht, self.Ri, self.Ro, self.A
        zLimit = 0.0
        for gear, sigma, _ in bores:
            steps = max(0, int(round((self.sOut - self.sIn) / 0.01)))
            for i in range(steps + 1):
                s = sigma * self.sIn + sigma * i * 0.01
                if abs(s) > Ro:
                    continue
                theta = self._gear_theta(gear, s)
                side = hw * abs(math.sin(theta)) + ht * abs(math.cos(theta))
                if math.hypot(s, side) < Ri:
                    continue
                up = hw * abs(math.cos(theta)) + ht * abs(math.sin(theta))
                reach = A / 2 + up
                if reach > zLimit:
                    zLimit = reach
        return zLimit

    def _bore_span(self, sigma):
        if sigma < 0:
            return (-self.sOut, -self.sIn)
        return (self.sIn, self.sOut)

    def _build_station_table(self, gear, sigma, C, n):
        lo, hi = self._bore_span(sigma)
        step, fine = 0.002, 0.0001

        def make(s):
            pieces = self._section_in_wall(gear, s, C, n)
            circles = []
            for piece in pieces:
                if not piece:
                    circles.append(None)
                    continue
                cx = sum(p[0] for p in piece) / len(piece)
                cy = sum(p[1] for p in piece) / len(piece)
                radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in piece)
                circles.append((cx, cy, radius))
            return (s, pieces, circles)

        def hasPieces(entry):
            return tuple(bool(p) for p in entry[1])

        table = []
        k = 0
        prev = None
        while True:
            s = lo + k * step
            if s > hi:
                break
            entry = make(s)
            if prev is not None and hasPieces(entry) != hasPieces(prev):
                prevS = prev[0]
                j = 1
                while prevS + j * fine < s:
                    table.append(make(prevS + j * fine))
                    j += 1
            table.append(entry)
            prev = entry
            k += 1
        return table

    def _wall_gap(self, table, gear, Q, reach, cc, span):
        lo, hi = span
        rel = _vsub(Q, gear.origin)
        x = _vdot(rel, gear.u)
        y = _vdot(rel, gear.v)
        sq = _vdot(rel, gear.dir)
        clamped = max(lo, min(hi, sq))
        axisDist = math.sqrt(x * x + y * y + (sq - clamped) ** 2)
        if axisDist - cc >= reach:
            return reach

        best = [reach * reach]

        def pointInPiece(piece, px, py):
            if len(piece) < 3:
                return False
            n = len(piece)
            for i in range(n):
                p, r = piece[i], piece[(i + 1) % n]
                if (r[0] - p[0]) * (py - p[1]) - (r[1] - p[1]) * (px - p[0]) < 0:
                    return False
            return True

        def edgeDist2(piece, px, py):
            best2 = float('inf')
            n = len(piece)
            for i in range(n):
                p, r = piece[i], piece[(i + 1) % n]
                ex, ey = r[0] - p[0], r[1] - p[1]
                l2 = ex * ex + ey * ey
                if l2 > 0:
                    kk = max(0.0, min(1.0, ((px - p[0]) * ex + (py - p[1]) * ey) / l2))
                else:
                    kk = 0.0
                dx, dy = px - (p[0] + kk * ex), py - (p[1] + kk * ey)
                best2 = min(best2, dx * dx + dy * dy)
            return best2

        def visit(entry):
            s, pieces, circles = entry
            ds = (sq - s) ** 2
            if ds >= best[0]:
                return False
            for piece, circle in zip(pieces, circles):
                if not piece or circle is None:
                    continue
                cx, cy, radius = circle
                o = math.hypot(x - cx, y - cy) - radius
                if o > 0 and ds + o * o >= best[0]:
                    continue
                if pointInPiece(piece, x, y):
                    d2 = 0.0
                else:
                    d2 = edgeDist2(piece, x, y)
                best[0] = min(best[0], ds + d2)
            return True

        ss = [entry[0] for entry in table]
        lo_i, hi_i = 0, len(ss)
        while lo_i < hi_i:
            mid = (lo_i + hi_i) // 2
            if ss[mid] < sq:
                lo_i = mid + 1
            else:
                hi_i = mid
        at = lo_i

        i = at
        while i < len(table) and visit(table[i]):
            i += 1
        i = at - 1
        while i >= 0 and visit(table[i]):
            i -= 1

        return math.sqrt(best[0])

    def _window_end(self, lean, lo, hi, bottom, top, fars, farTables, C, n, across, facing,
                     zLimit, Ri, Ro, dir_):
        need = self.collarWall + 0.1 / math.sqrt(2) + 0.005

        def chord(t):
            a0 = math.sqrt(max(0.0, Ri * Ri - t * t))
            a1 = math.sqrt(max(0.0, Ro * Ro - t * t))
            return a0, a1

        def near(z, a, t):
            pt = _vadd(_vadd(_vscale(across, t), _vscale(n, z)), _vscale(facing, a))
            Q = _vadd(C, pt)
            for (gear, sigma, _), table in zip(fars, farTables):
                if self._wall_gap(table, gear, Q, need, self.cc, self._bore_span(sigma)) < need:
                    return True
            return False

        def clear(te):
            t = dir_ * te
            zLow = max(lo - lean * t, bottom + lean * t)
            zHigh = min(hi - lean * t, top + lean * t)
            if zLow > zHigh:
                return False
            a0, a1 = chord(t)
            for z in (zLow, zHigh):
                a = a0
                while True:
                    if near(z, a, t):
                        return False
                    if a >= a1:
                        break
                    a = min(a + 0.1, a1)
            nn = max(1, int(math.ceil((zHigh - zLow) / 0.1)))
            for a in (a0, a1):
                for kk in range(1, nn):
                    z = zLow + (zHigh - zLow) * kk / nn
                    if near(z, a, t):
                        return False
                j = 0
                while True:
                    z = -zLimit + j * 0.1
                    if z > zLimit:
                        break
                    if zLow < z < zHigh and near(z, a, t):
                        return False
                    j += 1
            return True

        loE, hiE = 0.0, min(Ri, Ro / math.sqrt(2)) * (1 - 1e-9)
        for _ in range(24):
            mid = (loE + hiE) / 2
            if clear(mid):
                loE = mid
            else:
                hiE = mid
        return loE

    def _window_facings(self, e, k):
        if self.Sigma <= math.pi / 2:
            return [(k, '+k'), (_vscale(k, -1.0), '-k')]
        return [(e, '+e'), (_vscale(e, -1.0), '-e')]

    def _find_window(self, facing, facingName, C, n, bores, zLimit, stationCache):
        across = _vcross(n, facing)

        flanks, fars = [], []
        for bore in bores:
            gear, sigma, name = bore
            crossing = self._bore_crossing(gear, sigma)
            depth = _vdot(_vsub(crossing, C), facing)
            (flanks if depth > 0 else fars).append(bore)

        if len(flanks) != 2:
            raise Exception(
                f'window {facingName}: the window\'s side of the tube does not hold two bores')

        def crossingOf(bore):
            gear, sigma, _ = bore
            return self._bore_crossing(gear, sigma)

        low, high = flanks[0], flanks[1]
        if _vdot(_vsub(crossingOf(low), C), n) > _vdot(_vsub(crossingOf(high), C), n):
            low, high = high, low

        lean = 1.0
        if _vdot(_vsub(crossingOf(high), C), across) < _vdot(_vsub(crossingOf(low), C), across):
            lean = -1.0

        lowGear, lowSigma, lowName = low
        highGear, highSigma, highName = high

        lowCorners = self._wall_corners(lowGear, lowSigma, C, n, across)
        highCorners = self._wall_corners(highGear, highSigma, C, n, across)
        if not lowCorners or not highCorners:
            futil.log(f'No window facing {facingName}: no wall corners found')
            return None

        lowReach = max(z + lean * t for (_, t, z) in lowCorners)
        highReach = min(z + lean * t for (_, t, z) in highCorners)

        lo = lowReach + math.sqrt(2) * self.collarWall
        hi = highReach - math.sqrt(2) * self.collarWall

        top = min(2 * zLimit - hi, hi + math.sqrt(2) * self.Ri)
        bottom = max(-2 * zLimit - lo, lo - math.sqrt(2) * self.Ri)

        farTables = [stationCache[b[2]] for b in fars]

        right = self._window_end(
            lean, lo, hi, bottom, top, fars, farTables, C, n, across, facing, zLimit,
            self.Ri, self.Ro, 1.0)
        left = -self._window_end(
            lean, lo, hi, bottom, top, fars, farTables, C, n, across, facing, zLimit,
            self.Ri, self.Ro, -1.0)

        Ro2 = self.Ro
        poly = [(-2 * Ro2, -2 * Ro2), (2 * Ro2, -2 * Ro2), (2 * Ro2, 2 * Ro2), (-2 * Ro2, 2 * Ro2)]
        for (alpha, beta, gamma) in (
            (lean, 1.0, hi),
            (-lean, -1.0, -lo),
            (-lean, 1.0, top),
            (lean, -1.0, -bottom),
            (1.0, 0.0, right),
            (-1.0, 0.0, -left),
        ):
            poly = _clip_polygon(poly, alpha, beta, gamma)

        corners = []
        for p in poly:
            if corners and math.hypot(p[0] - corners[-1][0], p[1] - corners[-1][1]) < 0.001:
                continue
            corners.append(p)
        while len(corners) > 1:
            a, b = corners[-1], corners[0]
            if math.hypot(a[0] - b[0], a[1] - b[1]) >= 0.001:
                break
            corners.pop()

        reason = None
        if hi <= lo:
            reason = 'the flanking bores leave no band between them'
        elif right <= left or len(corners) < 3:
            reason = 'the far bores leave the band no length'
        else:
            area = 0.0
            for i in range(len(corners)):
                p, q = corners[i], corners[(i + 1) % len(corners)]
                area += p[0] * q[1] - q[0] * p[1]
            area /= 2.0
            if area <= 0:
                reason = 'the far bores leave the band no length'

        if reason is not None:
            futil.log(f'No window facing {facingName}: {reason}')
            return None

        return {'facing': facing, 'name': facingName, 'across': across, 'corners': corners}

    def _searchWindows(self):
        C0, e0, n0, k0 = self._dry_frame()
        gearsMm = self._place_gears(C0, e0, n0, self.A / 2)
        bores = self._dry_bores(gearsMm)

        zLimit = self._channel_top(bores)
        stationCache = {}
        for gear, sigma, name in bores:
            stationCache[name] = self._build_station_table(gear, sigma, C0, n0)

        for (facing, facingName) in self._window_facings(e0, k0):
            window = self._find_window(facing, facingName, C0, n0, bores, zLimit, stationCache)
            if window is not None:
                self.windows.append(window)

    # -----------------------------------------------------------------
    # S06: the component tree.
    # -----------------------------------------------------------------

    def buildComponentTree(self):
        topComponent = self.getComponent()
        topComponent.name = 'Screw Gearing'

        self.pathLines = [{}, {}]
        self.gearBodies = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]

        self.designOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.designOcc.component.name = 'Design'

        gearAOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearAOcc.component.name = 'Gear A'
        gearBOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearBOcc.component.name = 'Gear B'
        self.gearOccs = [gearAOcc, gearBOcc]

        self.cageOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.cageOcc.component.name = 'Cage'

    # -----------------------------------------------------------------
    # S07, S08, S09: the Anchor sketch, both Axis Planes, and the frame.
    # -----------------------------------------------------------------

    def buildAnchor(self):
        component = self.designOcc.component

        # S07: the Anchor sketch.
        sketch = component.sketches.add(self.plane)
        sketch.name = 'Anchor'

        projectedPoint = sketch.project(self.point).item(0)
        g0 = projectedPoint.geometry
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(g0.x - 0.5, g0.y, 0),
            adsk.core.Point3D.create(g0.x + 0.5, g0.y, 0))

        sketch.geometricConstraints.addCoincident(projectedPoint, line)
        sketch.geometricConstraints.addMidPoint(projectedPoint, line)
        sketch.geometricConstraints.addHorizontal(line)
        anchorText = adsk.core.Point3D.create(g0.x, g0.y + 0.3, 0)
        anchorDim = sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, anchorText)
        anchorDim.parameter.value = 1.0

        if not sketch.isFullyConstrained:
            raise Exception('"Anchor" sketch is not fully constrained')

        C = _from_point3d(projectedPoint.worldGeometry)
        startWorld = _from_point3d(line.startSketchPoint.worldGeometry)
        endWorld = _from_point3d(line.endSketchPoint.worldGeometry)
        e = _vnorm(_vsub(endWorld, startWorld))

        self.anchorLine = line

        # S08: Gear A Axis Plane, and the sign of n.
        planeInputA = component.constructionPlanes.createInput()
        planeInputA.setByOffset(
            self.plane, adsk.core.ValueInput.createByReal(-self.A / 2 / 10.0))
        axisPlaneA = component.constructionPlanes.add(planeInputA)
        axisPlaneA.name = 'Gear A Axis Plane'

        geomA = axisPlaneA.geometry
        originA = _from_point3d(geomA.origin)
        normalA = _from_vector3d(geomA.normal)
        if _vdot(_vsub(C, originA), normalA) > 0:
            n = normalA
        else:
            n = _vscale(normalA, -1.0)
        n = _vnorm(n)

        # S09: Gear B Axis Plane, and the frame.
        planeInputB = component.constructionPlanes.createInput()
        planeInputB.setByOffset(
            self.plane, adsk.core.ValueInput.createByReal(self.A / 2 / 10.0))
        axisPlaneB = component.constructionPlanes.add(planeInputB)
        axisPlaneB.name = 'Gear B Axis Plane'

        for (label, plane) in (('Gear A Axis Plane', axisPlaneA), ('Gear B Axis Plane', axisPlaneB)):
            geom = plane.geometry
            origin = _from_point3d(geom.origin)
            dist = abs(_vdot(_vsub(C, origin), n))
            if abs(dist - self.A / 20.0) > 1e-6:
                raise Exception(
                    f'"{label}" stands {dist} cm from C, expected {self.A / 20.0} cm')

        sideA = _vdot(_vsub(C, _from_point3d(axisPlaneA.geometry.origin)), n)
        sideB = _vdot(_vsub(C, _from_point3d(axisPlaneB.geometry.origin)), n)
        if not (sideA * sideB < 0):
            raise Exception(
                '"Gear A Axis Plane" and "Gear B Axis Plane" do not stand on opposite sides '
                'of C')

        k = _vcross(n, e)
        self.C, self.e, self.n, self.k = C, e, n, k
        self.axisPlanes = (axisPlaneA, axisPlaneB)
        self.gears = self._place_gears(C, e, n, self.A / 20.0)

    # -----------------------------------------------------------------
    # S10 to S18: one gear, built twice (index 0 then index 1).
    # -----------------------------------------------------------------

    def buildGear(self, index):
        self.buildSweepPaths(index)
        cellBody = self.buildToothCell(index)
        self.repeatCellByDoubling(index, cellBody)

    def buildSweepPaths(self, index):
        # S10: a gear's Paths sketch.
        gear = self.gears[index]
        axisPlane = self.axisPlanes[index]
        label = gear.label
        component = self.designOcc.component

        sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Paths'

        stations = (-self.sOut, -self.sIn, self.sIn, self.sOut)
        points = []
        for s in stations:
            world = self._gear_raw_point(gear, s)
            local = sketch.modelToSketchSpace(_point3d(world))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))

        pointMinusOut, pointMinusIn, pointPlusIn, pointPlusOut = points

        boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointMinusOut, pointMinusIn)
        borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointPlusIn, pointPlusOut)

        for p in points:
            p.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'"{label} Paths" sketch is not fully constrained')

        self.pathLines[index] = {'bore-': boreMinus, 'bore+': borePlus}

    def _loft_cell_sections(self, component, sketch, gear, sectionLines) -> adsk.fusion.LoftFeature:
        loftInput = component.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for lines in sectionLines:
            collection = adsk.core.ObjectCollection.create()
            for line in lines:
                collection.add(line)
            path = component.features.createPath(collection, False)
            loftInput.loftSections.add(path)
        loft = component.features.loftFeatures.add(loftInput)
        return loft

    def _draw_cell_sections(self, sketch, gear, stations):
        uB = -self.W / 2
        hv = self.T / 2
        sectionLines = []
        for s_k in stations:
            uF = self.W / 2 - self.H / 2 + (self.H / 2) * math.cos(
                2 * math.pi * (s_k - gear.z0) / self.P)
            corners = []
            for (u, v) in ((uB, -hv), (uF, -hv), (uF, hv), (uB, hv)):
                world = self._gear_point(gear, u, v, s_k)
                local = sketch.modelToSketchSpace(_point3d(world))
                corners.append(sketch.sketchPoints.add(local))
            lines = [
                sketch.sketchCurves.sketchLines.addByTwoPoints(corners[i], corners[(i + 1) % 4])
                for i in range(4)
            ]
            sectionLines.append(lines)
        return sectionLines

    def buildToothCell(self, index):
        gear = self.gears[index]
        axisPlane = self.axisPlanes[index]
        label = gear.label
        component = self.designOcc.component

        # S11: a gear's Cell Sections sketch.
        sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Cell Sections'
        sketch.isComputeDeferred = True

        c, nSteps = self.c, self.stepsPerTooth
        totalSections = c * nSteps + 1
        s0 = gear.z0 - self.L / 2
        stations = [s0 + k * self.P / nSteps for k in range(totalSections)]

        sectionLines = self._draw_cell_sections(sketch, gear, stations)

        for lines in sectionLines:
            for line in lines:
                line.startSketchPoint.isFixed = True
                line.endSketchPoint.isFixed = True
        sketch.isComputeDeferred = False

        if not sketch.isFullyConstrained:
            raise Exception(f'"{label} Cell Sections" sketch is not fully constrained')
        if sketch.profiles.count != totalSections:
            raise Exception(
                f'"{label} Cell Sections" has {sketch.profiles.count} profiles, '
                f'expected {totalSections}')

        # S12: a gear's cell loft.
        loft = self._loft_cell_sections(component, sketch, gear, sectionLines)
        if loft.bodies.count != 1 or not loft.bodies.item(0).isSolid:
            raise Exception(
                f'{label}: the cell loft produced {loft.bodies.count} body(ies)')

        return loft.bodies.item(0)

    def _screw_step_matrix(self, gear, k):
        axisVector = _vector3d(gear.dir)
        axisPoint = _point3d(gear.origin)

        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * self.P / self.Lambda, axisVector, axisPoint)

        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k * self.P / 10.0)

        mov = adsk.core.Matrix3D.create()
        mov.translation = shift

        rot.transformBy(mov)
        return rot

    def _copy_body(self, component, body, label):
        copyFeature = component.features.copyPasteBodies.add(body)
        if copyFeature.bodies.count != 1:
            raise Exception(f'{label}: copying a body produced {copyFeature.bodies.count} bodies')
        return copyFeature.bodies.item(0)

    def _move_body(self, component, body, matrix):
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = component.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(matrix)
        component.features.moveFeatures.add(moveInput)

    def _join_bodies(self, component, body, moved, label):
        tools = adsk.core.ObjectCollection.create()
        tools.add(moved)
        combineInput = component.features.combineFeatures.createInput(body, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature = component.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise Exception(
                f'{label}: joining left {combineFeature.bodies.count} bodies, expected 1')
        return combineFeature.bodies.item(0)

    def _doubling_schedule(self, q):
        ops = []
        asides = []
        m = 1
        top = 0
        while (1 << (top + 1)) <= q:
            top += 1
        for b in range(top):
            if q & (1 << b):
                ops.append(('aside', m, None))
                asides.append(m)
            ops.append(('double', m, m))
            m *= 2
        for cells in reversed(asides):
            ops.append(('place', cells, m))
            m += cells
        return ops

    def repeatCellByDoubling(self, index, cellBody):
        gear = self.gears[index]
        label = gear.label
        component = self.designOcc.component

        q, r = self.q, self.r
        body = cellBody
        asideBodies = {}

        for (kind, cells, by) in self._doubling_schedule(q):
            if kind == 'aside':
                asideBodies[cells] = self._copy_body(component, body, label)
            elif kind == 'double':
                copyBody = self._copy_body(component, body, label)
                matrix = self._screw_step_matrix(gear, cells * self.c)
                self._move_body(component, copyBody, matrix)
                body = self._join_bodies(component, body, copyBody, label)
            elif kind == 'place':
                asideBody = asideBodies.pop(cells)
                matrix = self._screw_step_matrix(gear, by * self.c)
                self._move_body(component, asideBody, matrix)
                body = self._join_bodies(component, body, asideBody, label)

        if r == 0:
            body.name = label
            self.gearBodies[index] = body
            return

        # S16, S17, S18: the remainder cell.
        axisPlane = self.axisPlanes[index]
        nSteps = self.stepsPerTooth
        sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Cell Remainder'
        sketch.isComputeDeferred = True

        totalSections = r * nSteps + 1
        s0 = gear.z0 - self.L / 2
        base = s0 + q * self.c * self.P
        stations = [base + k * self.P / nSteps for k in range(totalSections)]

        sectionLines = self._draw_cell_sections(sketch, gear, stations)

        for lines in sectionLines:
            for line in lines:
                line.startSketchPoint.isFixed = True
                line.endSketchPoint.isFixed = True
        sketch.isComputeDeferred = False

        if not sketch.isFullyConstrained:
            raise Exception(f'"{label} Cell Remainder" sketch is not fully constrained')
        if sketch.profiles.count != totalSections:
            raise Exception(
                f'"{label} Cell Remainder" has {sketch.profiles.count} profiles, '
                f'expected {totalSections}')

        loft = self._loft_cell_sections(component, sketch, gear, sectionLines)
        if loft.bodies.count != 1 or not loft.bodies.item(0).isSolid:
            raise Exception(
                f'{label}: the remainder loft produced {loft.bodies.count} body(ies)')
        remainderBody = loft.bodies.item(0)

        body = self._join_bodies(component, body, remainderBody, label)
        body.name = label
        self.gearBodies[index] = body

    # -----------------------------------------------------------------
    # S19 to S26: the sleeve, the four bores, and the two windows.
    # -----------------------------------------------------------------

    def buildCage(self):
        component = self.designOcc.component

        # S19: the Sleeve sketch.
        sketch = component.sketches.add(self.plane)
        sketch.name = 'Sleeve'

        centre = sketch.modelToSketchSpace(_point3d(self.C))
        centre.z = 0

        ringCandidates = []
        for radius_mm in (self.Ri, self.Ro):
            radius_cm = radius_mm / 10.0
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius_cm)
            circle.centerSketchPoint.isFixed = True
            textWorld = _vadd(self.C, _vscale(self.e, radius_cm))
            textPoint = sketch.modelToSketchSpace(_point3d(textWorld))
            textPoint.z = 0
            dimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2 * radius_cm

        if not sketch.isFullyConstrained:
            raise Exception('"Sleeve" sketch is not fully constrained')

        for profile in sketch.profiles:
            if profile.profileLoops.count == 2:
                ringCandidates.append(profile)
        if len(ringCandidates) != 1:
            raise Exception(
                f'"Sleeve" sketch has {len(ringCandidates)} profile(s) with two loops, '
                f'expected 1')
        ring = ringCandidates[0]

        # S20: extrude the tube.
        extrudeInput = component.features.extrudeFeatures.createInput(
            ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(self.cageRise / 10.0), False)
        tubeFeature = component.features.extrudeFeatures.add(extrudeInput)
        if tubeFeature.bodies.count != 1:
            raise Exception(
                f'"Sleeve" tube extrude produced {tubeFeature.bodies.count} bodies, expected 1')
        self.cageBody = tubeFeature.bodies.item(0)

        self._cutBores()
        self._cutWindows()

    def _angle_text_world(self, gear, O, theta, branch):
        hw = self.hw
        if branch == 'spine':
            ray1 = gear.u
            ray2 = _vadd(_vscale(gear.u, math.cos(theta)), _vscale(gear.v, math.sin(theta)))
            vertex = O
        else:
            ray1 = gear.u
            ray2 = _vadd(_vscale(gear.u, -math.sin(theta)), _vscale(gear.v, math.cos(theta)))
            vertex = _vadd(O, _vscale(gear.u, (hw / math.cos(theta)) / 10.0))
        bis = _vadd(ray1, ray2)
        bisLen = _vlen(bis)
        if bisLen == 0:
            bis = ray1
        else:
            bis = _vscale(bis, 1.0 / bisLen)
        return _vadd(vertex, _vscale(bis, (hw / 2.0) / 10.0))

    def _bore_seed_point(self, sketch: adsk.fusion.Sketch, gear, s, u, v) -> adsk.core.Point3D:
        world = self._gear_point(gear, u, v, s)
        local = sketch.modelToSketchSpace(_point3d(world))
        local.z = 0
        return local

    def _cutOneBore(self, component, gear, pathLine, sigma):
        label = gear.label
        side = '-R' if sigma < 0 else '+R'
        s1 = -self.sOut if sigma < 0 else self.sIn
        hw, ht = self.hw, self.ht
        theta = self._gear_theta(gear, s1)

        # S21: the bore's plane.
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(pathLine, adsk.core.ValueInput.createByReal(0))
        borePlane = component.constructionPlanes.add(planeInput)
        borePlane.name = f'{label} Bore {side} Plane'

        # S22: the bore's section sketch (the rectangle scheme).
        sketch = component.sketches.add(borePlane)
        sketch.name = f'{label} Bore {side}'
        sketch.isComputeDeferred = True

        O = self._gear_raw_point(gear, s1)
        Cp = self._gear_raw_point(gear, s1, alongU=self.A / 2)
        localO = sketch.modelToSketchSpace(_point3d(O))
        localO.z = 0
        localCp = sketch.modelToSketchSpace(_point3d(Cp))
        localCp.z = 0
        pointO = sketch.sketchPoints.add(localO)
        pointCp = sketch.sketchPoints.add(localCp)

        Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, pointCp)
        Ru.isConstruction = True

        seedE = sketch.sketchPoints.add(self._bore_seed_point(sketch, gear, s1, hw, 0))
        K = sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, seedE)
        K.isConstruction = True

        pointO.isFixed = True
        pointCp.isFixed = True

        seed1 = sketch.sketchPoints.add(self._bore_seed_point(sketch, gear, s1, -hw, -ht))
        seed2 = sketch.sketchPoints.add(self._bore_seed_point(sketch, gear, s1, hw, -ht))
        seed3 = sketch.sketchPoints.add(self._bore_seed_point(sketch, gear, s1, hw, ht))
        seed4 = sketch.sketchPoints.add(self._bore_seed_point(sketch, gear, s1, -hw, ht))

        L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(seed1, seed2)
        L2 = sketch.sketchCurves.sketchLines.addByTwoPoints(L1.endSketchPoint, seed3)
        L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(L2.endSketchPoint, seed4)
        L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)

        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addCoincident(K.endSketchPoint, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        sketch.geometricConstraints.addParallel(L4, L2)

        spineText = self._bore_seed_point(sketch, gear, s1, hw / 2, ht / 4)
        lowText = self._bore_seed_point(sketch, gear, s1, hw / 2, -ht / 2)
        highText = self._bore_seed_point(sketch, gear, s1, hw / 2, ht / 2)
        widthText = self._bore_seed_point(sketch, gear, s1, 0, -ht / 4)

        spineDim = sketch.sketchDimensions.addDistanceDimension(
            K.startSketchPoint, K.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, spineText)
        spineDim.parameter.value = hw / 10.0

        lowDim = sketch.sketchDimensions.addOffsetDimension(K, L1, lowText)
        lowDim.parameter.value = ht / 10.0
        highDim = sketch.sketchDimensions.addOffsetDimension(K, L3, highText)
        highDim.parameter.value = ht / 10.0
        widthDim = sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)
        widthDim.parameter.value = 2 * hw / 10.0

        psi = _fold_pi(theta)
        if abs(math.sin(psi)) >= math.sqrt(0.5):
            angleWorld = self._angle_text_world(gear, O, theta, 'spine')
            angleLocal = sketch.modelToSketchSpace(_point3d(angleWorld))
            angleLocal.z = 0
            angleDim = sketch.sketchDimensions.addAngularDimension(Ru, K, angleLocal)
            angleDim.parameter.value = abs(psi)
        else:
            angleWorld = self._angle_text_world(gear, O, theta, 'tooth')
            angleLocal = sketch.modelToSketchSpace(_point3d(angleWorld))
            angleLocal.z = 0
            angleDim = sketch.sketchDimensions.addAngularDimension(Ru, L2, angleLocal)
            phi = _fold_pi(psi + math.pi / 2)
            angleDim.parameter.value = abs(phi)

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'"{label} Bore {side}" sketch is not fully constrained')

        profile = find_profile_by_curve_counts(sketch, lines=4)

        # S23: cut a bore with a twisted sweep, and check its sense.
        path = component.features.createPath(pathLine, False)
        sweepInput = component.features.sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(self.twist)
        sweepInput.participantBodies = [self.cageBody]
        sweepFeature = component.features.sweepFeatures.add(sweepInput)
        if sweepFeature.bodies.count != 1:
            raise Exception(
                f'{label} Bore {side}: the sweep cut produced {sweepFeature.bodies.count} '
                f'bodies, expected 1')
        self.cageBody = sweepFeature.bodies.item(0)

        sc = -self.cageRadius if sigma < 0 else self.cageRadius
        thetaC = self._gear_theta(gear, sc)
        uVec = _vadd(_vscale(gear.u, math.cos(thetaC)), _vscale(gear.v, math.sin(thetaC)))
        probeOffset = self.W / 2 + self.clearance / 2
        axisPointC = _vadd(gear.origin, _vscale(gear.dir, sc / 10.0))
        probe1 = _vadd(axisPointC, _vscale(uVec, probeOffset / 10.0))
        probe2 = _vadd(axisPointC, _vscale(uVec, -probeOffset / 10.0))

        for probe in (probe1, probe2):
            containment = self.cageBody.pointContainment(_point3d(probe))
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise Exception(
                    f'{label} Bore {side}: a sense-check probe at {probe} read containment '
                    f'{containment}, expected outside')

    def _cutBores(self):
        component = self.designOcc.component
        for index in range(2):
            gear = self.gears[index]
            for sigma in (-1, 1):
                pathKey = 'bore-' if sigma < 0 else 'bore+'
                pathLine = self.pathLines[index][pathKey]
                self._cutOneBore(component, gear, pathLine, sigma)

    def _cutOneWindow(self, component, windowPlane, window):
        facing = window['facing']
        name = window['name']
        across = window['across']
        corners = window['corners']

        sketch = component.sketches.add(windowPlane)
        sketch.name = f'Window {name}'

        points = []
        for (t, z) in corners:
            world = _vadd(_vadd(self.C, _vscale(across, t / 10.0)), _vscale(self.n, z / 10.0))
            local = sketch.modelToSketchSpace(_point3d(world))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))

        count = len(points)
        for i in range(count):
            sketch.sketchCurves.sketchLines.addByTwoPoints(points[i], points[(i + 1) % count])

        for p in points:
            p.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'"Window {name}" sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(
                f'"Window {name}" has {sketch.profiles.count} profiles, expected 1')
        profile = sketch.profiles.item(0)

        # S26: cut a window, and check it.
        tc = sum(p[0] for p in corners) / len(corners)
        zc = sum(p[1] for p in corners) / len(corners)
        a0 = math.sqrt(max(0.0, self.Ri ** 2 - tc ** 2))
        a1 = math.sqrt(max(0.0, self.Ro ** 2 - tc ** 2))
        probeWorld = _vadd(
            _vadd(_vadd(self.C, _vscale(across, tc / 10.0)), _vscale(self.n, zc / 10.0)),
            _vscale(facing, (a0 + a1) / 20.0))
        probe = _point3d(probeWorld)

        containment = self.cageBody.pointContainment(probe)
        if containment != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(
                f'"Window {name}": the probe before the cut read containment {containment}, '
                f'expected inside')

        checkPoint = sketch.modelToSketchSpace(_point3d(_vadd(self.C, facing)))
        if checkPoint.z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection

        extrudeInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByReal((self.Ro + 1) / 10.0)),
            direction)
        extrudeInput.participantBodies = [self.cageBody]
        windowFeature = component.features.extrudeFeatures.add(extrudeInput)
        if windowFeature.bodies.count != 1:
            raise Exception(
                f'"Window {name}": the cut produced {windowFeature.bodies.count} bodies, '
                f'expected 1')
        self.cageBody = windowFeature.bodies.item(0)

        containmentAfter = self.cageBody.pointContainment(probe)
        if containmentAfter != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(
                f'"Window {name}": the probe after the cut read containment '
                f'{containmentAfter}, expected outside')

    def _cutWindows(self):
        component = self.designOcc.component
        if not self.windows:
            return

        planeInput = component.constructionPlanes.createInput()
        if self.Sigma <= math.pi / 2:
            planeInput.setByAngle(
                self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)
        else:
            planeInput.setByDistanceOnPath(
                self.anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane = component.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'

        for window in self.windows:
            self._cutOneWindow(component, windowPlane, window)

    # -----------------------------------------------------------------
    # S27: relocate the bodies and clean up.
    # -----------------------------------------------------------------

    def relocateBodies(self):
        self.cageBody.name = 'Cage'
        gearBodyA = adsk.fusion.BRepBody.cast(self.gearBodies[0])
        gearBodyB = adsk.fusion.BRepBody.cast(self.gearBodies[1])
        self.gearBodies[0] = gearBodyA.moveToComponent(self.gearOccs[0])
        self.gearBodies[1] = gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody = self.cageBody.moveToComponent(self.cageOcc)
