"""Screw gearing: two identical twisted toothed ribbons crossing at an angle, held by a sleeve.

Generated from spec/screwgear/steps.md. All values are precomputed in Python
([PB-PRECOMPUTED-MODE]); the generator registers no user parameter. Every computation is
written in millimetres and radians and divided by 10 when it is handed to the Fusion API.
"""

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import get_design
from .base import Generator, get_selection
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
GROUP_ID_RIBBON = 'ribbonGroup'
GROUP_ID_FRAME = 'frameGroup'
GROUP_ID_MESH = 'meshGroup'
CELL_TEETH = 4


# ---------------------------------------------------------------------------------------------
# Small vector and polygon helpers (tuples, millimetres unless stated otherwise)
# ---------------------------------------------------------------------------------------------

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    length = math.sqrt(_dot(a, a))
    return (a[0] / length, a[1] / length, a[2] / length)


def _rotateAbout(e, n, angle):
    """rotate(e, angle, about n) = cos(angle)*e + sin(angle)*(n x e)."""
    return _add(_scale(e, math.cos(angle)), _scale(_cross(n, e), math.sin(angle)))


def _gearFrames(e, n, A, Sigma, PhiA, PhiB, Phase):
    """The frame of section 1 with C at the origin: gear A (index 0) and gear B (index 1)."""
    dirA = _rotateAbout(e, n, Sigma / 2)
    uA = n
    frameA = {
        'label': 'Gear A',
        'origin': _scale(n, -A / 2),
        'dir': dirA,
        'u': uA,
        'v': _cross(dirA, uA),
        'phi': PhiA,
        'z0': 0.0,
    }
    dirB = _rotateAbout(e, n, -Sigma / 2)
    uB = _scale(n, -1.0)
    frameB = {
        'label': 'Gear B',
        'origin': _scale(n, A / 2),
        'dir': dirB,
        'u': uB,
        'v': _cross(dirB, uB),
        'phi': PhiB,
        'z0': Phase,
    }
    return [frameA, frameB]


def _framePoint(frame, lam, s, u, v):
    """point(g, s, u, v) = origin_g + s*dir_g + (u cos th - v sin th) u_g + (u sin th + v cos th) v_g."""
    theta = s / lam + frame['phi']
    c, sn = math.cos(theta), math.sin(theta)
    x = u * c - v * sn
    y = u * sn + v * c
    o, d, uu, vv = frame['origin'], frame['dir'], frame['u'], frame['v']
    return (o[0] + s * d[0] + x * uu[0] + y * vv[0],
            o[1] + s * d[1] + x * uu[1] + y * vv[1],
            o[2] + s * d[2] + x * uu[2] + y * vv[2])


def _clip(poly, a, b, c):
    """Keep the part of a convex polygon where a*x + b*y <= c, walking its edges in order."""
    out = []
    count = len(poly)
    for i in range(count):
        p = poly[i]
        q = poly[(i + 1) % count]
        fp = a * p[0] + b * p[1] - c
        fq = a * q[0] + b * q[1] - c
        if fp <= 0:
            out.append(p)
        if (fp < 0 and fq > 0) or (fp > 0 and fq < 0):
            k = fp / (fp - fq)
            out.append((p[0] + k * (q[0] - p[0]), p[1] + k * (q[1] - p[1])))
    return out


def _polygonArea(poly):
    area = 0.0
    count = len(poly)
    for i in range(count):
        j = (i + 1) % count
        area += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    return area / 2


def _convexHull(points):
    """Andrew's monotone chain; counter-clockwise, no repeated closing point."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _hullSeparation(first, second):
    """The largest separation along the unit normal of any edge of either hull; negative when
    the hulls overlap."""
    if not first or not second:
        return math.inf
    best = -math.inf
    found = False
    for hull in (first, second):
        count = len(hull)
        if count < 2:
            continue
        for i in range(count):
            p = hull[i]
            q = hull[(i + 1) % count]
            ex, ey = q[0] - p[0], q[1] - p[1]
            length = math.hypot(ex, ey)
            if length == 0:
                continue
            mx, my = -ey / length, ex / length
            ps = [mx * c[0] + my * c[1] for c in first]
            qs = [mx * c[0] + my * c[1] for c in second]
            separation = max(min(qs) - max(ps), min(ps) - max(qs))
            best = max(best, separation)
            found = True
    if not found:
        return math.hypot(first[0][0] - second[0][0], first[0][1] - second[0][1])
    return best


def _insideConvex(poly, x, y):
    """On the inner side of every edge of a counter-clockwise polygon; fewer than three
    corners never contains."""
    count = len(poly)
    if count < 3:
        return False
    for i in range(count):
        p = poly[i]
        q = poly[(i + 1) % count]
        if (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0]) < 0:
            return False
    return True


def _segmentDistance2(x, y, p, q):
    dx, dy = q[0] - p[0], q[1] - p[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return (x - p[0]) ** 2 + (y - p[1]) ** 2
    k = ((x - p[0]) * dx + (y - p[1]) * dy) / length2
    k = max(0.0, min(1.0, k))
    cx, cy = p[0] + k * dx, p[1] + k * dy
    return (x - cx) ** 2 + (y - cy) ** 2


def _pieceDistance2(poly, x, y):
    if _insideConvex(poly, x, y):
        return 0.0
    count = len(poly)
    best = math.inf
    for i in range(count):
        best = min(best, _segmentDistance2(x, y, poly[i], poly[(i + 1) % count]))
    return best


def _pieceCircle(poly):
    count = len(poly)
    cx = sum(p[0] for p in poly) / count
    cy = sum(p[1] for p in poly) / count
    radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in poly)
    return (poly, cx, cy, radius)


# ---------------------------------------------------------------------------------------------
# Dialog
# ---------------------------------------------------------------------------------------------

class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        # [PB-AUTOFOCUS-FIRST] the plane selection comes first; [PB-SELECTION-DECL],
        # [PB-SELECTION-FILTER-ENUM].
        planeInput = command.commandInputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to")
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        pointInput = command.commandInputs.addSelectionInput(
            INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1, 1)

        parentInput = command.commandInputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # [PB-DIALOG-DEFAULT-UNITS]: defaults in internal units (cm, radians, bare count).
        groups = (
            (GROUP_ID_RIBBON, 'Ribbon', True, (
                (INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', 1.5),
                (INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', 68),
                (INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', 4.95),
                (INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', 0.375),
                (INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', 0.2625),
                (INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', 0.2625),
            )),
            (GROUP_ID_FRAME, 'Frame', True, (
                (INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', 1.5),
                (INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', 1.875),
                (INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.045),
                (INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 0.3),
                (INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 0.3),
            )),
            (GROUP_ID_MESH, 'Mesh (from the mesh search)', False, (
                (INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', math.radians(80)),
                (INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 0.075),
                (INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', math.radians(15)),
                (INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', math.radians(15)),
                (INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -0.131),
            )),
        )
        for groupId, groupLabel, expanded, rows in groups:
            group = command.commandInputs.addGroupCommandInput(groupId, groupLabel)
            group.isExpanded = expanded
            for id, label, unit, default in rows:
                group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))


# ---------------------------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------------------------

class ScrewGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'ScrewGear'

    # -- orchestration ----------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        self.buildGear(0)
        self.buildGear(1)
        self.buildCage()
        self.relocateBodies()
        # [PB-TREE-CLEANUP]
        solids.hide_construction_geometry(self.designOcc.component)

    # -- step 2: inputs and range checks -----------------------------------------------------

    def _findInput(self, inputs: adsk.core.CommandInputs, id):
        found = inputs.itemById(id)
        if found is None:
            raise Exception(f'Input "{id}" was not found in the dialog')
        return found

    def _selectionOf(self, inputs: adsk.core.CommandInputs, selectionId):
        self._findInput(inputs, selectionId)
        entities = get_selection(inputs, selectionId)
        if len(entities) != 1:
            raise Exception(f'{selectionId}: expected exactly one selected entity, got {len(entities)}')
        return entities[0]

    def _readValue(self, inputs: adsk.core.CommandInputs, id, units):
        found = self._findInput(inputs, id)
        input = adsk.core.ValueCommandInput.cast(found)
        if input is None:
            raise Exception(f'Input "{id}" is not a value input')
        design: adsk.fusion.Design = self.design
        # [PB-EVAL-EXPRESSION]: internal units, cm for a length and radians for an angle.
        value = design.unitsManager.evaluateExpression(input.expression, units)
        if units == 'mm':
            return value * 10
        return value

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # [PB-SELECTION-STASH]: every selection is read before any occurrence exists.
        parent = self._selectionOf(inputs, INPUT_ID_PARENT)
        parentOccurrence = adsk.fusion.Occurrence.cast(parent)
        parentComponent = adsk.fusion.Component.cast(parent)
        if parentOccurrence:
            self.parentComponent = parentOccurrence.component
        elif parentComponent:
            self.parentComponent = parentComponent
        else:
            raise Exception(f'{INPUT_ID_PARENT}: the selection is neither an occurrence nor a component')
        self.targetPlane = self._selectionOf(inputs, INPUT_ID_PLANE)
        self.centrePoint = self._selectionOf(inputs, INPUT_ID_POINT)

        W = self._readValue(inputs, INPUT_ID_RIBBON_WIDTH, 'mm')
        Nraw = self._readValue(inputs, INPUT_ID_TOOTH_COUNT, '')
        TwistLead = self._readValue(inputs, INPUT_ID_TWIST_LEAD, 'mm')
        T = self._readValue(inputs, INPUT_ID_RIBBON_THICKNESS, 'mm')
        P = self._readValue(inputs, INPUT_ID_TOOTH_PITCH, 'mm')
        H = self._readValue(inputs, INPUT_ID_TOOTH_HEIGHT, 'mm')
        cageRadius = self._readValue(inputs, INPUT_ID_CAGE_RADIUS, 'mm')
        cageRise = self._readValue(inputs, INPUT_ID_CAGE_RISE, 'mm')
        clearance = self._readValue(inputs, INPUT_ID_CLEARANCE, 'mm')
        collarHalf = self._readValue(inputs, INPUT_ID_COLLAR_HALF, 'mm')
        collarWall = self._readValue(inputs, INPUT_ID_COLLAR_WALL, 'mm')
        Sigma = self._readValue(inputs, INPUT_ID_CROSS_ANGLE, 'deg')
        Eng = self._readValue(inputs, INPUT_ID_ENGAGEMENT, 'mm')
        PhiA = self._readValue(inputs, INPUT_ID_MOUNT_ANGLE_A, 'deg')
        PhiB = self._readValue(inputs, INPUT_ID_MOUNT_ANGLE_B, 'deg')
        Phase = self._readValue(inputs, INPUT_ID_ASSEMBLY_PHASE, 'mm')

        # Range checks, in order ([PB-SELF-DIAGNOSING]).
        for name, value in ((INPUT_ID_RIBBON_WIDTH, W), (INPUT_ID_RIBBON_THICKNESS, T),
                            (INPUT_ID_TOOTH_PITCH, P), (INPUT_ID_TWIST_LEAD, TwistLead),
                            (INPUT_ID_COLLAR_HALF, collarHalf), (INPUT_ID_COLLAR_WALL, collarWall),
                            (INPUT_ID_CLEARANCE, clearance)):
            if not value > 0:
                raise Exception(f'{name} must be > 0 mm (got {value:.4f} mm)')
        if abs(Nraw - round(Nraw)) > 1e-9 or Nraw < 4:
            raise Exception(f'{INPUT_ID_TOOTH_COUNT} must be a whole number >= 4 (got {Nraw})')
        N = int(round(Nraw))
        if not (H > 0 and H < W / 2):
            raise Exception(
                f'{INPUT_ID_TOOTH_HEIGHT} must be > 0 and < ribbonWidth/2 = {W / 2:.4f} mm '
                f'(got {H:.4f} mm)')
        if not (Eng > 0 and Eng <= H):
            raise Exception(
                f'{INPUT_ID_ENGAGEMENT} must be > 0 and <= toothHeight = {H:.4f} mm (got {Eng:.4f} mm)')
        if not (Sigma > 0 and Sigma < math.pi):
            raise Exception(
                f'{INPUT_ID_CROSS_ANGLE} must lie strictly between 0 and 180 deg '
                f'(got {math.degrees(Sigma):.4f} deg)')
        if not abs(Phase) < P:
            raise Exception(
                f'{INPUT_ID_ASSEMBLY_PHASE} must lie strictly within +-toothPitch = +-{P:.4f} mm '
                f'(got {Phase:.4f} mm)')
        if not (cageRadius + collarHalf + 1 < N * P / 2):
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: cageRadius + collarHalf + 1 mm = '
                f'{cageRadius + collarHalf + 1:.4f} mm must be < toothCount*toothPitch/2 = '
                f'{N * P / 2:.4f} mm')

        Lam = TwistLead / (2 * math.pi)
        A = W - Eng
        L = N * P
        Ri = cageRadius - collarHalf
        Ro = cageRadius + collarHalf
        hw = W / 2 + clearance
        ht = T / 2 + clearance
        c = math.hypot(hw, ht)
        axialWindow = 1.5 * math.sqrt(W * W - A * A) / math.sin(Sigma)

        if not c < Ri:
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: the channel must start in the hollow: the bore corner '
                f'radius c = {c:.4f} mm must be < Ri = cageRadius - collarHalf = {Ri:.4f} mm')
        visible = math.hypot(axialWindow, math.hypot(W / 2, T / 2)) + clearance
        if not visible <= Ri:
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: the mesh must stay visible along the axis: '
                f'hypot(axialWindow, hypot(W/2, T/2)) + clearance = {visible:.4f} mm must be <= '
                f'Ri = {Ri:.4f} mm')
        riseNeeded = A / 2 + c + collarWall
        if not cageRise >= riseNeeded:
            raise Exception(
                f'{INPUT_ID_CAGE_RISE}: the end faces must keep collarWall: cageRise = '
                f'{cageRise:.4f} mm must be >= A/2 + c + collarWall = {riseNeeded:.4f} mm')

        self.W, self.T, self.P, self.H, self.N = W, T, P, H, N
        self.TwistLead, self.Sigma, self.Eng = TwistLead, Sigma, Eng
        self.PhiA, self.PhiB, self.Phase = PhiA, PhiB, Phase
        self.cageRadius, self.cageRise, self.clearance = cageRadius, cageRise, clearance
        self.collarHalf, self.collarWall = collarHalf, collarWall
        self.Lam, self.A, self.L = Lam, A, L
        self.Ri, self.Ro, self.hw, self.ht, self.c = Ri, Ro, hw, ht, c
        self.axialWindow = axialWindow

        self.sIn = math.sqrt(Ri * Ri - c * c) - 1
        self.sOut = Ro + 1

        # The abstract frame of step 3: C at the origin, e on X, k on Y, n on Z.
        self.absE = (1.0, 0.0, 0.0)
        self.absK = (0.0, 1.0, 0.0)
        self.absN = (0.0, 0.0, 1.0)
        self.absFrames = _gearFrames(self.absE, self.absN, A, Sigma, PhiA, PhiB, Phase)
        self._stationTables = {}
        self._wallCornerCache = {}

        # Range check 11 (step 3).
        self._checkWallBetweenBores()

        self.cell = min(CELL_TEETH, N)
        self.n = max(math.ceil((P / Lam) / math.radians(2)), 8)
        self.q = N // self.cell
        self.r = N % self.cell

        # Step 4: the window search.
        self.windowsFaceK = Sigma <= math.pi / 2
        self.windows = self._searchWindows()

        # Per-gear handles the build fills in (steps 9 to 17).
        self.pathLines: list = [{}, {}]
        self.cellBodies: list = [None, None]
        self.gearBodies: list = [None, None]

    # -- bores in the abstract frame ----------------------------------------------------------

    def _bores(self):
        bores = []
        for index, sign in ((0, -1), (0, 1), (1, -1), (1, 1)):
            frame = self.absFrames[index]
            if sign > 0:
                start, end = self.sIn, self.sOut
            else:
                start, end = -self.sOut, -self.sIn
            boreName = '+R' if sign > 0 else '-R'
            bores.append({
                'gear': index,
                'sign': sign,
                'name': f'{frame["label"]} {boreName}',
                'frame': frame,
                'spanStart': start,
                'spanEnd': end,
                'crossing': _add(frame['origin'], _scale(frame['dir'], sign * self.cageRadius)),
            })
        return bores

    def _boreByName(self, bores, label, boreName):
        for bore in bores:
            if bore['name'] == f'{label} {boreName}':
                return bore
        raise Exception(f'No bore named {label} {boreName}')

    # -- step 3: the wall between the bores ---------------------------------------------------

    def _channelOutline(self, bore):
        """Kept outline points of a bore, sampled every 0.1 mm from sIn outward."""
        frame = bore['frame']
        sigma = bore['sign']
        hw, ht, Ri, Ro = self.hw, self.ht, self.Ri, self.Ro
        kept = []
        k = 0
        while True:
            s = sigma * (self.sIn + 0.1 * k)
            if abs(s) > self.sOut:
                break
            k += 1
            for i in range(17):
                v = -ht + 2 * ht * i / 16
                u = -hw + 2 * hw * i / 16
                for (pu, pv) in ((hw, v), (-hw, v), (u, ht), (u, -ht)):
                    point = _framePoint(frame, self.Lam, s, pu, pv)
                    radius = math.hypot(point[0], point[1])
                    if Ri - 0.5 <= radius <= Ro + 0.5:
                        kept.append(point)
        return kept

    def _checkWallBetweenBores(self):
        bores = self._bores()
        outlines = {bore['name']: self._channelOutline(bore) for bore in bores}
        E, K, N = self.absE, self.absK, self.absN
        gaps = (
            ('+k', K, ('Gear A', '+R'), ('Gear B', '-R')),
            ('-e', _scale(E, -1.0), ('Gear A', '-R'), ('Gear B', '-R')),
            ('-k', _scale(K, -1.0), ('Gear A', '-R'), ('Gear B', '+R')),
            ('+e', E, ('Gear A', '+R'), ('Gear B', '+R')),
        )
        least = None
        for gapName, d, first, second in gaps:
            across = _cross(N, d)
            hulls = []
            names = []
            for label, boreName in (first, second):
                bore = self._boreByName(bores, label, boreName)
                names.append(bore['name'])
                projected = [(_dot(p, across), _dot(p, N)) for p in outlines[bore['name']]]
                hulls.append(_convexHull(projected))
            separation = _hullSeparation(hulls[0], hulls[1])
            if least is None or separation < least[0]:
                least = (separation, gapName, names)
        if least is not None and not least[0] >= self.collarWall:
            separation, gapName, names = least
            raise Exception(
                f'{INPUT_ID_COLLAR_WALL}: the wall between the bores {names[0]} and {names[1]} '
                f'(the gap facing {gapName}) is {separation:.4f} mm, less than collarWall = '
                f'{self.collarWall:.4f} mm')

    # -- step 4: the window search ------------------------------------------------------------

    def _wallInner(self, t):
        return math.sqrt(max(0.0, self.Ri * self.Ri - t * t))

    def _wallOuter(self, t):
        return math.sqrt(max(0.0, self.Ro * self.Ro - t * t))

    def _sectionInWall(self, frame, s):
        """The two pieces of a bore's section in the wall at station s, in (x along u_g,
        y along v_g)."""
        Ri, Ro = self.Ri, self.Ro
        if abs(s) >= Ro:
            return [], []
        hw, ht = self.hw, self.ht
        theta = s / self.Lam + frame['phi']
        c, sn = math.cos(theta), math.sin(theta)
        poly = [(u * c - v * sn, u * sn + v * c)
                for (u, v) in ((-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht))]
        o = _dot(frame['origin'], self.absN)
        un = _dot(frame['u'], self.absN)
        poly = _clip(poly, un, 0.0, self.cageRise - o)
        poly = _clip(poly, -un, 0.0, self.cageRise + o)
        near = math.sqrt(max(0.0, Ri * Ri - s * s))
        far = math.sqrt(Ro * Ro - s * s)
        upper = _clip(_clip(poly, 0.0, -1.0, -near), 0.0, 1.0, far)
        lower = _clip(_clip(poly, 0.0, 1.0, -near), 0.0, -1.0, far)
        return upper, lower

    def _wallCorners(self, bore):
        """Every corner of every piece over the bore's cut span, every 0.001 mm (world mm)."""
        if bore['name'] in self._wallCornerCache:
            return self._wallCornerCache[bore['name']]
        frame = bore['frame']
        o, d, u, v = frame['origin'], frame['dir'], frame['u'], frame['v']
        start, end = bore['spanStart'], bore['spanEnd']
        count = int(math.floor((end - start) / 0.001 + 1e-9))
        points = []
        for k in range(count + 1):
            s = start + k * 0.001
            for piece in self._sectionInWall(frame, s):
                for (x, y) in piece:
                    points.append((o[0] + x * u[0] + y * v[0] + s * d[0],
                                   o[1] + x * u[1] + y * v[1] + s * d[1],
                                   o[2] + x * u[2] + y * v[2] + s * d[2]))
        self._wallCornerCache[bore['name']] = points
        return points

    def _channelTop(self, bores):
        """zLimit over all four bores."""
        zLimit = -math.inf
        hw, ht = self.hw, self.ht
        for bore in bores:
            frame = bore['frame']
            sigma = bore['sign']
            k = 0
            while True:
                s = sigma * (self.sIn + 0.01 * k)
                if abs(s) > self.sOut:
                    break
                k += 1
                if abs(s) > self.Ro:
                    continue
                theta = s / self.Lam + frame['phi']
                st, ct = abs(math.sin(theta)), abs(math.cos(theta))
                if math.hypot(s, hw * st + ht * ct) >= self.Ri:
                    zLimit = max(zLimit, self.A / 2 + hw * ct + ht * st)
        return zLimit

    @staticmethod
    def _presence(pieces):
        return tuple(len(piece) > 0 for piece in pieces)

    def _stationTable(self, bore):
        """The bore's station table, built once per bore before its first walk."""
        if bore['name'] in self._stationTables:
            return self._stationTables[bore['name']]
        frame = bore['frame']
        start, end = bore['spanStart'], bore['spanEnd']
        count = int(math.floor((end - start) / 0.002 + 1e-9))
        base = []
        for k in range(count + 1):
            s = start + k * 0.002
            base.append((s, self._sectionInWall(frame, s)))
        stations = []
        for i, (s, pieces) in enumerate(base):
            stations.append((s, pieces))
            if i + 1 < len(base):
                nextS, nextPieces = base[i + 1]
                if self._presence(pieces) != self._presence(nextPieces):
                    j = 1
                    while True:
                        sj = s + j * 0.0001
                        if sj >= nextS - 1e-12:
                            break
                        stations.append((sj, self._sectionInWall(frame, sj)))
                        j += 1
        table = [(s, [_pieceCircle(piece) for piece in pieces if len(piece) > 0])
                 for (s, pieces) in stations]
        self._stationTables[bore['name']] = table
        return table

    def _wallGap(self, point, bore, reach):
        """The distance from a point to one bore's channel, capped at reach."""
        frame = bore['frame']
        rel = _sub(point, frame['origin'])
        x = _dot(rel, frame['u'])
        y = _dot(rel, frame['v'])
        sq = _dot(rel, frame['dir'])
        clamped = min(max(sq, bore['spanStart']), bore['spanEnd'])
        if math.hypot(math.hypot(x, y), sq - clamped) - self.c >= reach:
            return reach
        table = self._stationTable(bore)
        best = reach * reach
        lo, hi = 0, len(table)
        while lo < hi:
            mid = (lo + hi) // 2
            if table[mid][0] < sq:
                lo = mid + 1
            else:
                hi = mid
        first = lo
        j = first
        while j < len(table):
            s, circles = table[j]
            ds2 = (sq - s) ** 2
            if ds2 >= best:
                break
            best = self._stationBest(circles, x, y, ds2, best)
            j += 1
        j = first - 1
        while j >= 0:
            s, circles = table[j]
            ds2 = (sq - s) ** 2
            if ds2 >= best:
                break
            best = self._stationBest(circles, x, y, ds2, best)
            j -= 1
        return math.sqrt(best)

    @staticmethod
    def _stationBest(circles, x, y, ds2, best):
        for (piece, cx, cy, radius) in circles:
            o = math.hypot(x - cx, y - cy) - radius
            if o > 0 and ds2 + o * o >= best:
                continue
            best = min(best, ds2 + _pieceDistance2(piece, x, y))
        return best

    def _windowClear(self, window, te, delta, farBores, need, zLimit):
        t = delta * te
        lean = window['lean']
        zLow = max(window['lo'] - lean * t, window['bottom'] + lean * t)
        zHigh = min(window['hi'] - lean * t, window['top'] + lean * t)
        if zLow > zHigh:
            return False
        a0 = self._wallInner(t)
        a1 = self._wallOuter(t)
        depths = []
        k = 0
        while a0 + k * 0.1 < a1:
            depths.append(a0 + k * 0.1)
            k += 1
        depths.append(a1)
        samples = []
        for z in (zLow, zHigh):
            for a in depths:
                samples.append((z, a))
        heights = []
        m = math.ceil((zHigh - zLow) / 0.1)
        for k in range(1, m):
            heights.append(zLow + (zHigh - zLow) * k / m)
        # zLimit is -inf when no station of any bore reaches the wall.
        if zLimit > -math.inf:
            j = 0
            while True:
                z = -zLimit + j * 0.1
                if z > zLimit:
                    break
                if zLow < z < zHigh:
                    heights.append(z)
                j += 1
        for a in (a0, a1):
            for z in heights:
                samples.append((z, a))
        across, N, d = window['across'], self.absN, window['d']
        for (z, a) in samples:
            point = _add(_add(_scale(across, t), _scale(N, z)), _scale(d, a))
            for bore in farBores:
                if self._wallGap(point, bore, need) < need:
                    return False
        return True

    def _windowEnd(self, window, delta, farBores, need, zLimit):
        low = 0.0
        high = min(self.Ri, self.Ro / math.sqrt(2)) * (1 - 1e-9)
        for _ in range(24):
            middle = (low + high) / 2
            if self._windowClear(window, middle, delta, farBores, need, zLimit):
                low = middle
            else:
                high = middle
        return low

    def _clipCorners(self, window):
        Ro = self.Ro
        lean = window['lean']
        poly = [(-2 * Ro, -2 * Ro), (2 * Ro, -2 * Ro), (2 * Ro, 2 * Ro), (-2 * Ro, 2 * Ro)]
        for (a, b, c) in ((lean, 1.0, window['hi']),
                          (-lean, -1.0, -window['lo']),
                          (-lean, 1.0, window['top']),
                          (lean, -1.0, -window['bottom']),
                          (1.0, 0.0, window['right']),
                          (-1.0, 0.0, -window['left'])):
            poly = _clip(poly, a, b, c)
        corners = []
        for p in poly:
            if corners and math.hypot(p[0] - corners[-1][0], p[1] - corners[-1][1]) < 0.001:
                continue
            corners.append(p)
        if len(corners) > 1 and math.hypot(corners[0][0] - corners[-1][0],
                                           corners[0][1] - corners[-1][1]) < 0.001:
            corners = corners[:-1]
        return corners

    def _searchWindows(self):
        bores = self._bores()
        E, K, N = self.absE, self.absK, self.absN
        if self.windowsFaceK:
            facings = (('+k', K), ('-k', _scale(K, -1.0)))
        else:
            facings = (('+e', E), ('-e', _scale(E, -1.0)))
        zLimit = self._channelTop(bores)
        need = self.collarWall + 0.1 / math.sqrt(2) + 0.005
        windows = []
        for d, dVec in facings:
            across = _cross(N, dVec)
            flanking = [bore for bore in bores if _dot(bore['crossing'], dVec) > 0]
            farBores = [bore for bore in bores if not _dot(bore['crossing'], dVec) > 0]
            if len(flanking) != 2:
                reason = f'{len(flanking)} bore(s) flank it, expected 2'
                futil.log(f'No window facing {d}: {reason}')
                continue
            flanking.sort(key=lambda bore: _dot(bore['crossing'], N))
            low, high = flanking[0], flanking[1]
            lean = 1.0 if _dot(high['crossing'], across) > _dot(low['crossing'], across) else -1.0

            lowCorners = self._wallCorners(low)
            highCorners = self._wallCorners(high)
            if not lowCorners or not highCorners:
                reason = 'a flanking bore has no piece in the wall'
                futil.log(f'No window facing {d}: {reason}')
                continue
            lowReach = max(_dot(p, N) + lean * _dot(p, across) for p in lowCorners)
            highReach = min(_dot(p, N) + lean * _dot(p, across) for p in highCorners)
            lo = lowReach + math.sqrt(2) * self.collarWall
            hi = highReach - math.sqrt(2) * self.collarWall
            if hi <= lo:
                reason = f'hi {hi:.3f} mm <= lo {lo:.3f} mm'
                futil.log(f'No window facing {d}: {reason}')
                continue
            top = min(2 * zLimit - hi, hi + math.sqrt(2) * self.Ri)
            bottom = max(-2 * zLimit - lo, lo - math.sqrt(2) * self.Ri)
            window = {
                'name': d,
                'd': dVec,
                'across': across,
                'lean': lean,
                'lo': lo,
                'hi': hi,
                'top': top,
                'bottom': bottom,
            }
            right = self._windowEnd(window, 1, farBores, need, zLimit)
            left = -self._windowEnd(window, -1, farBores, need, zLimit)
            window['right'] = right
            window['left'] = left
            if right <= left:
                reason = f'right {right:.3f} mm <= left {left:.3f} mm'
                futil.log(f'No window facing {d}: {reason}')
                continue
            corners = self._clipCorners(window)
            area = _polygonArea(corners) if len(corners) >= 3 else 0.0
            if len(corners) < 3 or abs(area) < 1e-9:
                reason = f'the hexagon has {len(corners)} distinct corner(s) and area {abs(area):.4f} mm^2'
                futil.log(f'No window facing {d}: {reason}')
                continue
            window['corners'] = corners
            futil.log(
                f'Window {d}: lo {lo:.3f}, hi {hi:.3f}, bottom {bottom:.3f}, top {top:.3f}, '
                f'left {left:.3f}, right {right:.3f}, lean {lean:+.0f}, area {abs(area):.1f} mm^2')
            windows.append(window)
        return windows

    # -- world helpers ------------------------------------------------------------------------

    def _world(self, vec):
        """A world point (cm) at the millimetre offset vec from C."""
        return adsk.core.Point3D.create(self.Cx + vec[0] / 10, self.Cy + vec[1] / 10, self.Cz + vec[2] / 10)

    def _sketchLocal(self, sketch: adsk.fusion.Sketch, vec):
        """[PB-SKETCH-ZERO-Z]: map a world point into the sketch and put it on the plane."""
        local = sketch.modelToSketchSpace(self._world(vec))
        local.z = 0
        return local

    def _worldDirection(self, name):
        if name == '+k':
            return self.kHat
        if name == '-k':
            return _scale(self.kHat, -1.0)
        if name == '+e':
            return self.eHat
        return _scale(self.eHat, -1.0)

    # -- step 5: the component tree -----------------------------------------------------------

    def buildComponentTree(self):
        occurrence = self.getOccurrence()
        top: adsk.fusion.Component = occurrence.component
        top.name = 'Screw Gearing'
        occurrences = top.occurrences
        children = []
        for name in ('Design', 'Gear A', 'Gear B', 'Cage'):
            child = occurrences.addNewComponent(adsk.core.Matrix3D.create())
            child.component.name = name
            children.append(child)
        self.designOcc: adsk.fusion.Occurrence = children[0]
        self.gearOccs = [children[1], children[2]]
        self.cageOcc: adsk.fusion.Occurrence = children[3]

    # -- steps 6 to 8: the Anchor sketch and the axis planes -----------------------------------

    def buildAnchor(self):
        comp: adsk.fusion.Component = self.designOcc.component
        sketches = comp.sketches
        sketch: adsk.fusion.Sketch = sketches.add(self.targetPlane)
        sketch.name = 'Anchor'
        point = self.centrePoint
        projectedPoint = sketch.project(point).item(0)
        projectedPoint = adsk.fusion.SketchPoint.cast(projectedPoint)
        if projectedPoint is None:
            raise Exception('Anchor: the selected point did not project to a sketch point')
        projected = projectedPoint.geometry
        p1 = adsk.core.Point3D.create(projected.x - 0.5, projected.y, 0)
        p2 = adsk.core.Point3D.create(projected.x + 0.5, projected.y, 0)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
        geometricConstraints = sketch.geometricConstraints
        geometricConstraints.addCoincident(projectedPoint, line)
        geometricConstraints.addMidPoint(projectedPoint, line)
        geometricConstraints.addHorizontal(line)
        sketchDimensions = sketch.sketchDimensions
        textPoint = adsk.core.Point3D.create(projected.x, projected.y - 0.3, 0)
        dimension = sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dimension.parameter.value = 1.0
        if not sketch.isFullyConstrained:
            raise Exception('Anchor: the sketch is not fully constrained')

        # [PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]
        C = projectedPoint.worldGeometry
        self.Cpoint = C
        self.Cx, self.Cy, self.Cz = C.x, C.y, C.z
        start = line.startSketchPoint.worldGeometry
        end = line.endSketchPoint.worldGeometry
        self.eHat = _unit((end.x - start.x, end.y - start.y, end.z - start.z))
        self.anchorLine = line

        # Step 7: the Gear A Axis Plane, and n.
        A = self.A
        constructionPlanes = comp.constructionPlanes
        planeInput = constructionPlanes.createInput()
        planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A / 2 / 10))
        planeA = constructionPlanes.add(planeInput)
        planeA.name = 'Gear A Axis Plane'
        geometryA = planeA.geometry
        normal = (geometryA.normal.x, geometryA.normal.y, geometryA.normal.z)
        towardsC = (C.x - geometryA.origin.x, C.y - geometryA.origin.y, C.z - geometryA.origin.z)
        if _dot(towardsC, normal) > 0:
            self.nHat = _unit(normal)
        else:
            self.nHat = _unit(_scale(normal, -1.0))
        self.kHat = _cross(self.nHat, self.eHat)
        self.worldFrames = _gearFrames(self.eHat, self.nHat, A, self.Sigma, self.PhiA, self.PhiB, self.Phase)

        # Step 8: the Gear B Axis Plane.
        planeInput = constructionPlanes.createInput()
        planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A / 2 / 10))
        planeB = constructionPlanes.add(planeInput)
        planeB.name = 'Gear B Axis Plane'
        for plane in (planeA, planeB):
            geometry = plane.geometry
            gap = abs((C.x - geometry.origin.x) * geometry.normal.x
                      + (C.y - geometry.origin.y) * geometry.normal.y
                      + (C.z - geometry.origin.z) * geometry.normal.z)
            if abs(gap - A / 2 / 10) > 1e-6:
                raise Exception(
                    f'{plane.name}: the centre stands {gap:.7f} cm from the plane, '
                    f'expected A/2 = {A / 2 / 10:.7f} cm')
        self.axisPlanes = [planeA, planeB]

    # -- steps 9 to 17: one gear ---------------------------------------------------------------

    def buildGear(self, index):
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, index):
        frame = self.worldFrames[index]
        label = frame['label']
        comp: adsk.fusion.Component = self.designOcc.component
        sketches = comp.sketches
        sketch: adsk.fusion.Sketch = sketches.add(self.axisPlanes[index])
        sketch.name = f'{label} Paths'
        sIn, sOut = self.sIn, self.sOut
        pt = {}
        for s in (-sOut, -sIn, sIn, sOut):
            point = self._world(_add(frame['origin'], _scale(frame['dir'], s)))
            local = sketch.modelToSketchSpace(point)
            local.z = 0
            pt[s] = sketch.sketchPoints.add(local)
        sketchLines = sketch.sketchCurves.sketchLines
        boreMinus = sketchLines.addByTwoPoints(pt[-sOut], pt[-sIn])
        borePlus = sketchLines.addByTwoPoints(pt[sIn], pt[sOut])
        for s in pt:
            pt[s].isFixed = True
        if not sketch.isFullyConstrained:
            raise Exception(f'{label} Paths: the sketch is not fully constrained')
        self.pathLines[index] = {'bore-': boreMinus, 'bore+': borePlus}

    def _buildSectionSketch(self, index, name, fromTooth, teeth):
        """Steps 10 and 16: teeth*n + 1 sections from tooth fromTooth, corners off-plane."""
        frame = self.worldFrames[index]
        comp: adsk.fusion.Component = self.designOcc.component
        sketches = comp.sketches
        sketch: adsk.fusion.Sketch = sketches.add(self.axisPlanes[index])
        sketch.name = name
        sketch.isComputeDeferred = True
        W, H, T, P, n, Lam = self.W, self.H, self.T, self.P, self.n, self.Lam
        Z0 = frame['z0']
        s0 = Z0 - self.L / 2 + fromTooth * P
        sketchLines = sketch.sketchCurves.sketchLines
        points = []
        sections = []
        for k in range(teeth * n + 1):
            s = s0 + k * P / n
            uB = -W / 2
            uF = W / 2 - H / 2 + (H / 2) * math.cos(2 * math.pi * (s - Z0) / P)
            hv = T / 2
            corners = []
            for (u, v) in ((uB, -hv), (uF, -hv), (uF, hv), (uB, hv)):
                world = self._world(_framePoint(frame, Lam, s, u, v))
                # [PB-3D-SKETCH-SECTIONS]: the corner keeps its z.
                corners.append(sketch.sketchPoints.add(sketch.modelToSketchSpace(world)))
            lines = []
            for i in range(4):
                corner = corners[i]
                nextCorner = corners[(i + 1) % 4]
                lines.append(sketchLines.addByTwoPoints(corner, nextCorner))
            sections.append(lines)
            points.extend(corners)
        for point in points:
            point.isFixed = True
        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{name}: the sketch is not fully constrained')
        expected = teeth * n + 1
        if sketch.profiles.count != expected:
            raise Exception(
                f'{name}: the sketch has {sketch.profiles.count} profiles, expected {expected}')
        return sections

    def _loftSections(self, index, sections, what):
        """Steps 11 and 17: one loft through every section, a new solid body."""
        label = self.worldFrames[index]['label']
        loftFeatures = self.designOcc.component.features.loftFeatures
        loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for lines in sections:
            collection = adsk.core.ObjectCollection.create()
            for line in lines:
                collection.add(line)
            path = self.designOcc.component.features.createPath(collection, False)
            loftInput.loftSections.add(path)
        feature = loftFeatures.add(loftInput)
        count = feature.bodies.count
        if count != 1:
            raise Exception(f'{label}: the {what} loft made {count} bodies, expected 1')
        body = feature.bodies.item(0)
        if not body.isSolid:
            raise Exception(f'{label}: the {what} loft made a body that is not solid')
        return body

    def buildToothCell(self, index):
        label = self.worldFrames[index]['label']
        sections = self._buildSectionSketch(index, f'{label} Cell Sections', 0, self.cell)
        self.cellBodies[index] = self._loftSections(index, sections, 'cell')

    def _copyBody(self, index, body):
        """Step 13."""
        label = self.worldFrames[index]['label']
        feature = self.designOcc.component.features.copyPasteBodies.add(body)
        count = feature.bodies.count
        if count != 1:
            raise Exception(f'{label}: the copy made {count} bodies, expected 1')
        return feature.bodies.item(0)

    def _moveByScrewStep(self, index, copy, k):
        """Step 14: move a copy by Step(k), k teeth."""
        frame = self.worldFrames[index]
        P, Lam = self.P, self.Lam
        direction = frame['dir']
        axisVector = adsk.core.Vector3D.create(direction[0], direction[1], direction[2])
        axisPoint = self._world(frame['origin'])
        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * P / Lam, axisVector, axisPoint)
        shift = axisVector.copy()
        shift.scaleBy(k * P / 10)
        mov = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)
        moveFeatures = self.designOcc.component.features.moveFeatures
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(copy)
        moveInput = moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        moveFeatures.add(moveInput)

    def _joinBodies(self, index, body, piece, description):
        """Step 15."""
        label = self.worldFrames[index]['label']
        combineFeatures = self.designOcc.component.features.combineFeatures
        tools = adsk.core.ObjectCollection.create()
        tools.add(piece)
        joinInput = combineFeatures.createInput(body, tools)
        joinInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        joinInput.isKeepToolBodies = False
        feature = combineFeatures.add(joinInput)
        count = feature.bodies.count
        if count != 1:
            raise Exception(f'{label}: the join of {description} made {count} bodies, expected 1')
        return feature.bodies.item(0)

    def repeatCellByDoubling(self, index):
        """Step 12: q cells by doubling, then the remainder."""
        label = self.worldFrames[index]['label']
        cell, q, r = self.cell, self.q, self.r
        body = self.cellBodies[index]
        m = 1
        asides = []
        top = 0
        while (q >> (top + 1)) > 0:
            top += 1
        for bit in range(top):
            if q & (1 << bit):
                aside = self._copyBody(index, body)
                asides.append((aside, m))
            copy = self._copyBody(index, body)
            self._moveByScrewStep(index, copy, m * cell)
            body = self._joinBodies(index, body, copy, f'{m} + {m} cells')
            m = 2 * m
        for aside, cells in reversed(asides):
            self._moveByScrewStep(index, aside, m * cell)
            body = self._joinBodies(index, body, aside, f'{m} + {cells} cells')
            m += cells
        if m != q:
            raise Exception(f'{label}: the doubling schedule built {m} cells, expected {q}')

        if r > 0:
            # Steps 16 and 17: the remainder, built where it belongs, then joined (step 15).
            sections = self._buildSectionSketch(index, f'{label} Cell Remainder', q * cell, r)
            piece = self._loftSections(index, sections, 'remainder')
            body = self._joinBodies(index, body, piece, f'{q} cells + {r} remainder teeth')

        body.name = label
        self.gearBodies[index] = body

    # -- steps 18 to 25: the cage -------------------------------------------------------------

    def buildCage(self):
        comp: adsk.fusion.Component = self.designOcc.component
        sketches = comp.sketches

        # Step 18: the Sleeve sketch.
        sketch: adsk.fusion.Sketch = sketches.add(self.targetPlane)
        sketch.name = 'Sleeve'
        centre = sketch.modelToSketchSpace(self.Cpoint)
        centre.z = 0
        sketchCircles = sketch.sketchCurves.sketchCircles
        sketchDimensions = sketch.sketchDimensions
        for radius in (self.Ri / 10, self.Ro / 10):
            circle = sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint = adsk.core.Point3D.create(centre.x + radius, centre.y, 0)
            dimension = sketchDimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2 * radius
        if not sketch.isFullyConstrained:
            raise Exception('Sleeve: the sketch is not fully constrained')
        rings = []
        loopCounts = []
        for i in range(sketch.profiles.count):
            profile = sketch.profiles.item(i)
            loops = profile.profileLoops.count
            loopCounts.append(loops)
            if loops == 2:
                rings.append(profile)
        if len(rings) != 1:
            raise Exception(
                f'Sleeve: expected exactly one profile with two loops, found {len(rings)} '
                f'(loop counts per profile: {loopCounts})')
        ring = rings[0]

        # Step 19: the tube.
        extrudeFeatures = comp.features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(self.cageRise / 10), False)
        feature = extrudeFeatures.add(extrudeInput)
        if feature.bodies.count != 1:
            raise Exception(f'Sleeve: the tube extrude made {feature.bodies.count} bodies, expected 1')
        self.cageBody: adsk.fusion.BRepBody = feature.bodies.item(0)

        # Steps 20 to 22: the four bores.
        constructionPlanes = comp.constructionPlanes
        for index, sign in ((0, -1), (0, 1), (1, -1), (1, 1)):
            frame = self.worldFrames[index]
            label = frame['label']
            boreName = '-R' if sign < 0 else '+R'
            line = self.pathLines[index]['bore-' if sign < 0 else 'bore+']
            s0 = -self.sOut if sign < 0 else self.sIn

            # Step 20: the bore's plane.
            planeInput = constructionPlanes.createInput()
            planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))
            plane = constructionPlanes.add(planeInput)
            plane.name = f'{label} Bore {boreName} Plane'

            profile = self._buildBoreSketch(index, boreName, plane, s0)
            self._cutBore(index, sign, boreName, line, profile)

        # Steps 23 to 25: the windows.
        if not self.windows:
            return
        planeInput = constructionPlanes.createInput()
        if self.windowsFaceK:
            planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)
        else:
            planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane = constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'
        for window in self.windows:
            self._cutWindow(window, windowPlane)

    def _buildBoreSketch(self, index, boreName, plane, s0):
        """Step 21: the rectangle scheme."""
        frame = self.worldFrames[index]
        label = frame['label']
        name = f'{label} Bore {boreName}'
        comp: adsk.fusion.Component = self.designOcc.component
        sketches = comp.sketches
        sketch: adsk.fusion.Sketch = sketches.add(plane)
        sketch.name = name
        sketch.isComputeDeferred = True

        Lam = self.Lam
        theta = s0 / Lam + frame['phi']
        uB, uF, hv = -self.hw, self.hw, self.ht
        u, v = frame['u'], frame['v']
        axisPoint = _add(frame['origin'], _scale(frame['dir'], s0))

        def S(su, sv):
            return _framePoint(frame, Lam, s0, su, sv)

        def local(vec):
            return self._sketchLocal(sketch, vec)

        sketchPoints = sketch.sketchPoints
        sketchLines = sketch.sketchCurves.sketchLines
        geometricConstraints = sketch.geometricConstraints
        sketchDimensions = sketch.sketchDimensions

        # References.
        O = sketchPoints.add(local(axisPoint))
        Cp = sketchPoints.add(local(_add(axisPoint, _scale(u, self.A / 2))))
        Ru = sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True

        # The spine.
        seedE = local(S(uF, 0))
        K = sketchLines.addByTwoPoints(O, seedE)
        K.isConstruction = True
        E = K.endSketchPoint
        O.isFixed = True
        Cp.isFixed = True

        # The rectangle ([PB-SHARE-XOR-COINCIDENT]).
        L1 = sketchLines.addByTwoPoints(local(S(uB, -hv)), local(S(uF, -hv)))
        L2 = sketchLines.addByTwoPoints(L1.endSketchPoint, local(S(uF, hv)))
        L3 = sketchLines.addByTwoPoints(L2.endSketchPoint, local(S(uB, hv)))
        L4 = sketchLines.addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)

        # The length.
        textPoint = local(S(uF / 2, hv / 4))
        lengthDimension = sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)
        lengthDimension.parameter.value = uF / 10

        # The angle ([PB-ANGULAR-DIM]); theta normalised into (-180, 180].
        th = math.atan2(math.sin(theta), math.cos(theta))
        if th <= -math.pi:
            th += 2 * math.pi
        if abs(math.sin(th)) >= math.sqrt(1 / 2):
            bisector = _add(_scale(u, math.cos(th / 2)), _scale(v, math.sin(th / 2)))
            textPoint = local(_add(axisPoint, _scale(bisector, uF / 2)))
            angleDimension = sketchDimensions.addAngularDimension(Ru, K, textPoint)
            angleDimension.parameter.value = math.acos(math.cos(th))
        else:
            X = _add(axisPoint, _scale(u, uF / math.cos(th)))
            alongL2 = _add(_scale(u, -math.sin(th)), _scale(v, math.cos(th)))
            bisector = _unit(_add(u, alongL2))
            textPoint = local(_add(X, _scale(bisector, uF / 2)))
            angleDimension = sketchDimensions.addAngularDimension(Ru, L2, textPoint)
            angleDimension.parameter.value = math.acos(-math.sin(th))

        # The rest of the rectangle ([PB-OFFSET-DIM], [PB-NO-OVERCONSTRAIN],
        # [PB-DIM-VALUE-SEMANTICS]: the side is the seed's, the value its magnitude).
        geometricConstraints.addParallel(L1, K)
        textPoint = local(S(uF / 2, -hv / 2))
        offset = sketchDimensions.addOffsetDimension(K, L1, textPoint)
        offset.parameter.value = hv / 10
        geometricConstraints.addParallel(L3, K)
        textPoint = local(S(uF / 2, hv / 2))
        offset = sketchDimensions.addOffsetDimension(K, L3, textPoint)
        offset.parameter.value = hv / 10
        geometricConstraints.addCoincident(E, L2)
        geometricConstraints.addPerpendicular(L2, K)
        geometricConstraints.addParallel(L4, L2)
        textPoint = local(S((uB + uF) / 2, -hv / 2))
        offset = sketchDimensions.addOffsetDimension(L2, L4, textPoint)
        offset.parameter.value = (uF - uB) / 10

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{name}: the sketch is not fully constrained')
        return find_profile_by_curve_counts(sketch, lines=4)

    def _cutBore(self, index, sign, boreName, line, profile):
        """Step 22: the twisted sweep cut, and its check."""
        frame = self.worldFrames[index]
        label = frame['label']
        name = f'{label} Bore {boreName}'
        path = self.designOcc.component.features.createPath(line, False)
        sweepFeatures = self.designOcc.component.features.sweepFeatures
        sweepInput = sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        twist = (self.sOut - self.sIn) / self.Lam
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)
        sweepInput.participantBodies = [self.cageBody]
        feature = sweepFeatures.add(sweepInput)
        if feature.bodies.count != 1:
            raise Exception(f'{name}: the sweep cut left {feature.bodies.count} bodies, expected 1')
        self.cageBody = feature.bodies.item(0)

        # [SCREW-F-SWEEP-CHECK]
        sc = sign * self.cageRadius
        thc = sc / self.Lam + frame['phi']
        uhat = _add(_scale(frame['u'], math.cos(thc)), _scale(frame['v'], math.sin(thc)))
        crossing = _add(frame['origin'], _scale(frame['dir'], sc))
        reach = self.W / 2 + self.clearance / 2
        readings = []
        failed = False
        for side in (1, -1):
            probe = self._world(_add(crossing, _scale(uhat, side * reach)))
            containment = self.cageBody.pointContainment(probe)
            readings.append(((probe.x, probe.y, probe.z), containment))
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                failed = True
        if failed:
            raise Exception(
                f'{name}: the channel does not clear both probes; readings (cm, containment): '
                f'{readings[0][0]} -> {readings[0][1]}, {readings[1][0]} -> {readings[1][1]}')

    def _cutWindow(self, window, windowPlane):
        """Steps 24 and 25."""
        comp: adsk.fusion.Component = self.designOcc.component
        name = window['name']
        d = self._worldDirection(name)
        across = _cross(self.nHat, d)

        # Step 24: the Window sketch.
        sketches = comp.sketches
        sketch: adsk.fusion.Sketch = sketches.add(windowPlane)
        sketch.name = f'Window {name}'
        points = []
        for (t, z) in window['corners']:
            local = sketch.modelToSketchSpace(self._world(_add(_scale(across, t), _scale(self.nHat, z))))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))
        sketchLines = sketch.sketchCurves.sketchLines
        count = len(points)
        for i in range(count):
            sketchLines.addByTwoPoints(points[i], points[(i + 1) % count])
        for point in points:
            point.isFixed = True
        if not sketch.isFullyConstrained:
            raise Exception(f'Window {name}: the sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(f'Window {name}: the sketch has {sketch.profiles.count} profiles, expected 1')
        profile = sketch.profiles.item(0)

        # Step 25: the probe.
        corners = window['corners']
        tc = sum(corner[0] for corner in corners) / len(corners)
        zc = sum(corner[1] for corner in corners) / len(corners)
        depth = (self._wallInner(tc) + self._wallOuter(tc)) / 2
        probe = self._world(_add(_add(_scale(across, tc), _scale(self.nHat, zc)), _scale(d, depth)))
        before = self.cageBody.pointContainment(probe)
        if before != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(
                f'Window {name}: the probe at ({probe.x:.4f}, {probe.y:.4f}, {probe.z:.4f}) cm '
                f'reads {before} before the cut, expected inside the wall')

        # The cut.
        extrudeFeatures = comp.features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        C = self.Cpoint
        ahead = adsk.core.Point3D.create(C.x + d[0], C.y + d[1], C.z + d[2])
        if sketch.modelToSketchSpace(ahead).z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
        extrudeInput.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((self.Ro + 1) / 10)),
            direction)
        extrudeInput.participantBodies = [self.cageBody]
        feature = extrudeFeatures.add(extrudeInput)
        if feature.bodies.count != 1:
            raise Exception(f'Window {name}: the cut left {feature.bodies.count} bodies, expected 1')
        self.cageBody = feature.bodies.item(0)
        after = self.cageBody.pointContainment(probe)
        if after != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(
                f'Window {name}: the probe at ({probe.x:.4f}, {probe.y:.4f}, {probe.z:.4f}) cm '
                f'reads {after} after the cut, expected outside (was the cut extruded the wrong way?)')

    # -- steps 26 to 28: relocate the bodies --------------------------------------------------

    def relocateBodies(self):
        self.cageBody.name = 'Cage'
        # Step 26: self.gearBodies[0].moveToComponent(self.gearOccs[0]), through a typed handle.
        gearBodyA: adsk.fusion.BRepBody = self.gearBodies[0]
        self.gearBodies[0] = gearBodyA.moveToComponent(self.gearOccs[0])
        # Step 27: self.gearBodies[1].moveToComponent(self.gearOccs[1]), likewise.
        gearBodyB: adsk.fusion.BRepBody = self.gearBodies[1]
        self.gearBodies[1] = gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody = self.cageBody.moveToComponent(self.cageOcc)
