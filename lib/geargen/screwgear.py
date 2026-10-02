import math

import adsk.core
import adsk.fusion

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
INPUT_ID_ROOF_ALLOWANCE = 'roofAllowance'
INPUT_ID_COLLAR_HALF = 'collarHalf'
INPUT_ID_COLLAR_WALL = 'collarWall'
INPUT_ID_CROSS_ANGLE = 'crossAngle'
INPUT_ID_ENGAGEMENT = 'engagement'
INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'
INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'
INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'
CELL_TEETH = 4


# ---------------------------------------------------------------------------
# Plain-tuple vector helpers. Every length below is in millimetres unless
# stated otherwise; these never touch adsk.* and are used for the pure-Python
# model math of processInputs (S02, S03) and the frame geometry of S06+.
# ---------------------------------------------------------------------------

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def _turn(u, v, th):
    # (x, y) = (u, v) turned by th: the section's coordinates on the plane
    # square to the axis, x along u_g and y along v_g (S02 "Theta_g", S06
    # "World_g").
    c, s = math.cos(th), math.sin(th)
    return u * c - v * s, u * s + v * c


def _fold(angle: float) -> float:
    # Fold an angle into (-pi, pi] (S18 "the angle"; sgSigned in the proof).
    # angle % (2*pi) is always in [0, 2*pi) for a positive divisor, so only
    # the upper half needs folding down.
    a = angle % (2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def _gear_axes(index, ehat, khat, nhat, center, sigma, axis_offset):
    # (origin, dir, u, v) for gear index 0 (A) or 1 (B), from S06 "The
    # frame" — also used at the abstract frame (center at the origin,
    # ehat/khat/nhat the axes) for S03's window and wall searches.
    half = axis_offset / 2.0
    sign = 1.0 if index == 0 else -1.0
    dirg = _add(_scale(ehat, math.cos(sigma / 2.0)), _scale(khat, sign * math.sin(sigma / 2.0)))
    origin = _sub(center, _scale(nhat, sign * half))
    u = _scale(nhat, sign)
    v = _cross(dirg, u)
    return origin, dirg, u, v


def _theta(gear, s):
    # Theta_g(s) = s/Lambda + Phi_g.
    return s / gear['lam'] + gear['phi']


def _world_point(gear, s, u, v):
    # World_g(s, u, v) = origin_g + s*dir_g + turn(u, v, Theta_g(s)) in the
    # (u_g, v_g) basis.
    th = _theta(gear, s)
    x, y = _turn(u, v, th)
    return _add(_add(gear['origin'], _scale(gear['dir'], s)),
                _add(_scale(gear['u'], x), _scale(gear['v'], y)))


# ---------------------------------------------------------------------------
# 2D polygon clipping (Andrew's monotone chain hull and Sutherland-Hodgman
# half-plane clip), used by the wall-separation and window searches of S03.
# Points are plain (x, y) tuples.
# ---------------------------------------------------------------------------

def _clip_poly(poly, a, b, c):
    # Keep the part of a convex polygon where a*px + b*py <= c, walking its
    # edges in order, keeping each corner on the kept side and adding the
    # point where an edge crosses the line (S03 "The hexagon" / sgClip).
    if not poly:
        return []
    out = []
    n = len(poly)
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        f0 = a * p0[0] + b * p0[1] - c
        f1 = a * p1[0] + b * p1[1] - c
        if f0 <= 0:
            out.append(p0)
        if (f0 < 0 and f1 > 0) or (f0 > 0 and f1 < 0):
            t = f0 / (f0 - f1)
            out.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
    return out


def _convex_hull(points):
    # Andrew's monotone chain, points as (x, y) tuples; returns the hull in
    # counter-clockwise order with no repeated closing point.
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross2(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross2(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross2(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _edge_normals(hull):
    # Unit vectors square to each edge of a convex hull given in order.
    normals = []
    n = len(hull)
    for i in range(n):
        p0 = hull[i]
        p1 = hull[(i + 1) % n]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        normals.append((-dy / length, dx / length))
    return normals


def _hull_separation(hullA, hullB):
    # S03 "The wall between the bores" step 3: for every edge of either
    # hull, with m square to that edge, the larger of
    # min(m.q) - max(m.p) and min(m.p) - max(m.q); the gap's separation is
    # the largest over all those edges.
    best = None
    for hull in (hullA, hullB):
        for m in _edge_normals(hull):
            pa = [m[0] * p[0] + m[1] * p[1] for p in hullA]
            pb = [m[0] * p[0] + m[1] * p[1] for p in hullB]
            sep = max(min(pb) - max(pa), min(pa) - max(pb))
            if best is None or sep > best:
                best = sep
    if best is None:
        return 0.0
    return best


def _tilt(gear, sigma, cageRadius, collarHalf):
    # S02 "tilt(sigma)": zero when some pi/2 + k*pi lies between the angles
    # at the wall's two faces on the bore's centre line, else the smaller
    # |cos theta| of the two.
    t1 = _theta(gear, sigma * (cageRadius - collarHalf))
    t2 = _theta(gear, sigma * (cageRadius + collarHalf))
    lo, hi = min(t1, t2), max(t1, t2)
    k = math.ceil((lo - math.pi / 2.0) / math.pi)
    if math.pi / 2.0 + k * math.pi <= hi:
        return 0.0
    return min(abs(math.cos(t1)), abs(math.cos(t2)))


def _level_sign(gear, cageRadius, collarHalf):
    # The level bore is the sign with the smaller tilt; on a tie, -R.
    if _tilt(gear, 1.0, cageRadius, collarHalf) < _tilt(gear, -1.0, cageRadius, collarHalf):
        return 1.0
    return -1.0


def _roof_is_plus_v(gear, sigma, cageRadius):
    # The roof is the +v face when -sin(Theta_g(sigma*cageRadius))*u_dot_n
    # is positive, else the -v face.
    return (-math.sin(_theta(gear, sigma * cageRadius)) * gear['uDotN']) > 0


def _section_in_wall(gear, s, hw, vLo, vHi, Ri, Ro, cageRise):
    # S03 "A bore's section in the wall" (sectionInWall): the bore's four
    # corners turned by Theta_g(s), clipped to the end faces and then to the
    # inner/outer radius band on each side. Returns (piece1, piece2), each
    # a list of (x, y) points in the (u_g, v_g) basis, or [] when that
    # piece is empty (fewer than three corners).
    if not (abs(s) < Ro):
        return [], []
    th = _theta(gear, s)
    corners = [_turn(u, v, th) for (u, v) in ((-hw, vLo), (hw, vLo), (hw, vHi), (-hw, vHi))]

    # Height clip: (origin_g + x*u_g).n_hat = origin_g.n_hat + x*uDotN lies
    # within +/- cageRise.
    ogN = gear['origin'][2]  # origin_g . n_hat, with n_hat = (0, 0, 1)
    uDotN = gear['uDotN']
    # ogN + uDotN*x <= cageRise  ->  uDotN*x <= cageRise - ogN
    poly = _clip_poly(corners, uDotN, 0.0, cageRise - ogN)
    # ogN + uDotN*x >= -cageRise  ->  -uDotN*x <= ogN + cageRise
    poly = _clip_poly(poly, -uDotN, 0.0, ogN + cageRise)
    if len(poly) < 3:
        return [], []

    near = math.sqrt(max(0.0, Ri * Ri - s * s))
    far = math.sqrt(max(0.0, Ro * Ro - s * s))

    # piece1: near <= y <= far  <=>  y <= far and y >= near.
    piece1 = _clip_poly(poly, 0.0, 1.0, far)
    piece1 = _clip_poly(piece1, 0.0, -1.0, -near)
    # piece2: -far <= y <= -near  <=>  y <= -near and y >= -far.
    piece2 = _clip_poly(poly, 0.0, 1.0, -near)
    piece2 = _clip_poly(piece2, 0.0, -1.0, far)

    if len(piece1) < 3:
        piece1 = []
    if len(piece2) < 3:
        piece2 = []
    return piece1, piece2


def _bore_pieces(gear, s, hw, vLo, vHi, Ri, Ro, cageRise):
    # Non-empty pieces only, as a plain list (for callers that don't need
    # to track which of the two slots a piece came from).
    piece1, piece2 = _section_in_wall(gear, s, hw, vLo, vHi, Ri, Ro, cageRise)
    pieces = []
    if piece1:
        pieces.append(piece1)
    if piece2:
        pieces.append(piece2)
    return pieces


def _point_in_poly(pt, poly):
    # S03 "The distance to a channel" (wallGap): a point is inside the
    # counter-clockwise polygon when it is on the inner side of every edge.
    if len(poly) < 3:
        return False
    for i in range(len(poly)):
        p0 = poly[i]
        p1 = poly[(i + 1) % len(poly)]
        cross = (p1[0] - p0[0]) * (pt[1] - p0[1]) - (p1[1] - p0[1]) * (pt[0] - p0[0])
        if cross < 0:
            return False
    return True


def _point_segment_distance2(pt, p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    len2 = dx * dx + dy * dy
    if len2 == 0:
        return (pt[0] - p0[0]) ** 2 + (pt[1] - p0[1]) ** 2
    t = ((pt[0] - p0[0]) * dx + (pt[1] - p0[1]) * dy) / len2
    t = max(0.0, min(1.0, t))
    px, py = p0[0] + t * dx, p0[1] + t * dy
    return (pt[0] - px) ** 2 + (pt[1] - py) ** 2


def _point_poly_distance2(pt, poly):
    if _point_in_poly(pt, poly):
        return 0.0
    best = None
    for i in range(len(poly)):
        d2 = _point_segment_distance2(pt, poly[i], poly[(i + 1) % len(poly)])
        if best is None or d2 < best:
            best = d2
    return best if best is not None else 0.0


def _poly_circle(poly):
    # The piece's "circle": centre the average of its corners, radius the
    # largest distance from that to a corner.
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in poly)
    return cx, cy, radius


def _build_bore_table(gear, bore, hw, Ri, Ro, cageRise):
    # S03 "The distance to a channel" (wallGap): a bore's station table,
    # built once per bore. Base stations every 0.002 mm over the cut span,
    # refined to 0.0001 mm wherever the set of non-empty pieces changes.
    s1, s2 = bore['from'], bore['to']

    def evalAt(s):
        p1, p2 = _section_in_wall(gear, s, hw, bore['vLo'], bore['vHi'], Ri, Ro, cageRise)
        return s, p1, p2

    baseStations = []
    k = 0
    while True:
        s = s1 + 0.002 * k
        if s > s2 + 1e-12:
            break
        baseStations.append(s)
        k += 1
    if not baseStations or abs(baseStations[-1] - s2) > 1e-9:
        baseStations.append(s2)

    rows = [evalAt(s) for s in baseStations]
    full = []
    for i in range(len(rows) - 1):
        s0, p1a, p2a = rows[i]
        s1b, p1b, p2b = rows[i + 1]
        full.append(rows[i])
        if (bool(p1a), bool(p2a)) != (bool(p1b), bool(p2b)):
            j = 1
            while True:
                extra = s0 + 0.0001 * j
                if extra >= s1b - 1e-12:
                    break
                full.append(evalAt(extra))
                j += 1
    full.append(rows[-1])

    table = []
    for s, p1, p2 in full:
        c1 = _poly_circle(p1) if p1 else None
        c2 = _poly_circle(p2) if p2 else None
        table.append((s, p1, p2, c1, c2))
    return table


def _wall_gap(point, gear, bore, table, cc, reach):
    # S03 "The distance to a channel" (wallGap).
    rel = _sub(point, gear['origin'])
    x = _dot(rel, gear['u'])
    y = _dot(rel, gear['v'])
    sq = _dot(rel, gear['dir'])
    s1, s2 = bore['from'], bore['to']
    clamped = min(max(sq, s1), s2)
    if math.hypot(math.hypot(x, y), sq - clamped) - cc >= reach:
        return reach

    state = {'best': reach * reach}

    def consider(i):
        s, p1, p2, c1, c2 = table[i]
        for piece, circ in ((p1, c1), (p2, c2)):
            if not piece:
                continue
            cx, cy, radius = circ
            o = math.hypot(x - cx, y - cy) - radius
            if o > 0 and (sq - s) ** 2 + o * o >= state['best']:
                continue
            if _point_in_poly((x, y), piece):
                d2 = 0.0
            else:
                d2 = _point_poly_distance2((x, y), piece)
            cand = (sq - s) ** 2 + d2
            if cand < state['best']:
                state['best'] = cand

    n = len(table)
    idx = 0
    while idx < n and table[idx][0] < sq:
        idx += 1

    i = idx
    while i < n and (sq - table[i][0]) ** 2 < state['best']:
        consider(i)
        i += 1
    i = idx - 1
    while i >= 0 and (sq - table[i][0]) ** 2 < state['best']:
        consider(i)
        i -= 1

    return math.sqrt(state['best'])


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command) -> None:
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
            INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', adsk.core.ValueInput.createByReal(1.5))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', adsk.core.ValueInput.createByReal(68))
        ribbonChildren.addValueInput(
            INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', adsk.core.ValueInput.createByReal(4.95))
        ribbonChildren.addValueInput(
            INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', adsk.core.ValueInput.createByReal(0.375))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', adsk.core.ValueInput.createByReal(0.2625))
        ribbonChildren.addValueInput(
            INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', adsk.core.ValueInput.createByReal(0.2625))

        frameGroup = inputs.addGroupCommandInput('frameGroup', 'Frame')
        frameGroup.isExpanded = True
        frameChildren = frameGroup.children
        frameChildren.addValueInput(
            INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', adsk.core.ValueInput.createByReal(1.5))
        frameChildren.addValueInput(
            INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', adsk.core.ValueInput.createByReal(1.875))
        frameChildren.addValueInput(
            INPUT_ID_CLEARANCE, 'Clearance', 'mm', adsk.core.ValueInput.createByReal(0.02))
        frameChildren.addValueInput(
            INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', adsk.core.ValueInput.createByReal(0.03))
        frameChildren.addValueInput(
            INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', adsk.core.ValueInput.createByReal(0.3))
        frameChildren.addValueInput(
            INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', adsk.core.ValueInput.createByReal(0.3))

        meshGroup = inputs.addGroupCommandInput('meshGroup', 'Mesh (from the mesh search)')
        meshGroup.isExpanded = False
        meshChildren = meshGroup.children
        meshChildren.addValueInput(
            INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(80)))
        meshChildren.addValueInput(
            INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', adsk.core.ValueInput.createByReal(0.09))
        meshChildren.addValueInput(
            INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14)))
        meshChildren.addValueInput(
            INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14)))
        meshChildren.addValueInput(
            INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm',
            adsk.core.ValueInput.createByReal(-0.13))


class ScrewGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'ScrewGear'

    # -----------------------------------------------------------------
    # S02 / S03: processInputs — read, check, derive, and run the two
    # frame-free searches (the wall between the bores, and the windows).
    # -----------------------------------------------------------------

    def processInputs(self, inputs: adsk.core.CommandInputs) -> None:
        design: adsk.fusion.Design = get_design()
        unitsManager: adsk.core.UnitsManager = design.unitsManager

        # Selections first ([PB-SELECTION-STASH]).
        parentSel = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentSel) != 1:
            raise Exception(f'"{INPUT_ID_PARENT}" must hold exactly one selection')
        parentEntity = parentSel[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        else:
            self.parentComponent = parentEntity

        planeSel = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeSel) != 1:
            raise Exception(f'"{INPUT_ID_PLANE}" must hold exactly one selection')
        self.plane = planeSel[0]

        pointSel = get_selection(inputs, INPUT_ID_POINT)
        if len(pointSel) != 1:
            raise Exception(f'"{INPUT_ID_POINT}" must hold exactly one selection')
        self.anchorPoint = pointSel[0]

        # Values ([PB-EVAL-EXPRESSION], [PB-PRECOMPUTED-MODE]): read in
        # Fusion internal units (cm, radians), then converted to millimetres
        # (angles stay in radians). Every check and derivation below works
        # in millimetres; every length handed to Fusion afterwards is
        # divided by 10.
        def readValue(inputs: adsk.core.CommandInputs, unitsManager: adsk.core.UnitsManager,
                      inputId: str, unit: str) -> float:
            found = inputs.itemById(inputId)
            if found is None:
                raise Exception(f'Input "{inputId}" not found')
            valueInput = adsk.core.ValueCommandInput.cast(found)
            return unitsManager.evaluateExpression(valueInput.expression, unit)

        ribbonWidth = readValue(inputs, unitsManager, INPUT_ID_RIBBON_WIDTH, 'mm') * 10.0
        toothCountRaw = readValue(inputs, unitsManager, INPUT_ID_TOOTH_COUNT, '')
        twistLead = readValue(inputs, unitsManager, INPUT_ID_TWIST_LEAD, 'mm') * 10.0
        ribbonThickness = readValue(inputs, unitsManager, INPUT_ID_RIBBON_THICKNESS, 'mm') * 10.0
        toothPitch = readValue(inputs, unitsManager, INPUT_ID_TOOTH_PITCH, 'mm') * 10.0
        toothHeight = readValue(inputs, unitsManager, INPUT_ID_TOOTH_HEIGHT, 'mm') * 10.0
        cageRadius = readValue(inputs, unitsManager, INPUT_ID_CAGE_RADIUS, 'mm') * 10.0
        cageRise = readValue(inputs, unitsManager, INPUT_ID_CAGE_RISE, 'mm') * 10.0
        clearance = readValue(inputs, unitsManager, INPUT_ID_CLEARANCE, 'mm') * 10.0
        roofAllowance = readValue(inputs, unitsManager, INPUT_ID_ROOF_ALLOWANCE, 'mm') * 10.0
        collarHalf = readValue(inputs, unitsManager, INPUT_ID_COLLAR_HALF, 'mm') * 10.0
        collarWall = readValue(inputs, unitsManager, INPUT_ID_COLLAR_WALL, 'mm') * 10.0
        crossAngle = readValue(inputs, unitsManager, INPUT_ID_CROSS_ANGLE, 'deg')
        engagement = readValue(inputs, unitsManager, INPUT_ID_ENGAGEMENT, 'mm') * 10.0
        mountAngleA = readValue(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_A, 'deg')
        mountAngleB = readValue(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_B, 'deg')
        assemblyPhase = readValue(inputs, unitsManager, INPUT_ID_ASSEMBLY_PHASE, 'mm') * 10.0

        # Range checks, in this order.
        if not (ribbonWidth > 0):
            raise Exception(f'"ribbonWidth" must be > 0 (got {ribbonWidth} mm)')
        if not (ribbonThickness > 0):
            raise Exception(f'"ribbonThickness" must be > 0 (got {ribbonThickness} mm)')
        if not (toothPitch > 0):
            raise Exception(f'"toothPitch" must be > 0 (got {toothPitch} mm)')
        if not (twistLead > 0):
            raise Exception(f'"twistLead" must be > 0 (got {twistLead} mm)')
        if not (collarHalf > 0):
            raise Exception(f'"collarHalf" must be > 0 (got {collarHalf} mm)')
        if not (collarWall > 0):
            raise Exception(f'"collarWall" must be > 0 (got {collarWall} mm)')
        if not (clearance > 0):
            raise Exception(f'"clearance" must be > 0 (got {clearance} mm)')

        if not (roofAllowance >= 0):
            raise Exception(f'"roofAllowance" must be >= 0 (got {roofAllowance} mm)')

        if abs(toothCountRaw - round(toothCountRaw)) > 1e-9 or round(toothCountRaw) < 4:
            raise Exception(
                f'"toothCount" must be a whole number >= 4 (got {toothCountRaw})')
        N = int(round(toothCountRaw))

        if not (toothHeight > 0 and toothHeight < ribbonWidth / 2.0):
            raise Exception(
                f'"toothHeight" must be > 0 and < ribbonWidth/2 '
                f'(got {toothHeight} mm, ribbonWidth/2 = {ribbonWidth / 2.0} mm)')

        if not (engagement > 0 and engagement <= toothHeight):
            raise Exception(
                f'"engagement" must be > 0 and <= toothHeight '
                f'(got {engagement} mm, toothHeight = {toothHeight} mm)')

        crossAngleDeg = math.degrees(crossAngle)
        if not (0 < crossAngleDeg < 180):
            raise Exception(
                f'"crossAngle" must lie strictly between 0 and 180 degrees (got {crossAngleDeg})')

        toothPitchMm = toothPitch
        if not (-toothPitchMm < assemblyPhase < toothPitchMm):
            raise Exception(
                f'"assemblyPhase" must lie strictly within +/- toothPitch '
                f'(got {assemblyPhase} mm, toothPitch = {toothPitchMm} mm)')

        if not (cageRadius + collarHalf + 1 < N * toothPitch / 2.0):
            raise Exception(
                f'"cageRadius" leaves both bores outside the ribbon '
                f'(cageRadius={cageRadius} mm, collarHalf={collarHalf} mm, '
                f'N*toothPitch/2={N * toothPitch / 2.0} mm)')

        # Derived values (millimetres and radians).
        W, T, P, H, Sigma = ribbonWidth, ribbonThickness, toothPitch, toothHeight, crossAngle
        Lambda = twistLead / (2.0 * math.pi)
        A = W - engagement
        n = max(math.ceil((P / Lambda) / math.radians(2.0)), 8)
        c = min(CELL_TEETH, N)
        q = N // c
        r = N % c
        L = N * P
        Ri = cageRadius - collarHalf
        Ro = cageRadius + collarHalf
        hw = W / 2.0 + clearance
        ht = T / 2.0 + clearance
        a = roofAllowance
        cc = math.hypot(hw, ht + a)
        sIn = math.sqrt(Ri * Ri - cc * cc) - 1.0
        sOut = Ro + 1.0
        axialWindow = 1.5 * math.sqrt(W * W - A * A) / math.sin(Sigma)

        self.W, self.T, self.P, self.H, self.Sigma = W, T, P, H, Sigma
        self.N, self.Lambda, self.A, self.n, self.c, self.q, self.r, self.L = (
            N, Lambda, A, n, c, q, r, L)
        self.Ri, self.Ro, self.hw, self.ht, self.roofAllowance, self.cc = Ri, Ro, hw, ht, a, cc
        self.sIn, self.sOut, self.axialWindow = sIn, sOut, axialWindow
        self.cageRadius, self.cageRise = cageRadius, cageRise
        self.collarHalf, self.collarWall, self.clearance = collarHalf, collarWall, clearance
        self.engagement, self.twistLead = engagement, twistLead
        self.crossAngle = crossAngle
        self.mountAngleA, self.mountAngleB = mountAngleA, mountAngleB
        self.assemblyPhase = assemblyPhase

        # The abstract frame of S03: C at the origin, e_hat = (1,0,0),
        # k_hat = (0,1,0), n_hat = (0,0,1); both gears' axes built from
        # these exactly as S06 builds them from the real frame.
        ehat = (1.0, 0.0, 0.0)
        khat = (0.0, 1.0, 0.0)
        nhat = (0.0, 0.0, 1.0)
        center = (0.0, 0.0, 0.0)

        gearFrames = []
        for idx, phi in enumerate((mountAngleA, mountAngleB)):
            origin, dirg, u, v = _gear_axes(idx, ehat, khat, nhat, center, Sigma, A)
            gearFrames.append({
                'label': 'Gear A' if idx == 0 else 'Gear B',
                'origin': origin, 'dir': dirg, 'u': u, 'v': v,
                'phi': phi, 'lam': Lambda, 'z0': 0.0 if idx == 0 else assemblyPhase,
                'uDotN': 1.0 if idx == 0 else -1.0,
            })

        # The sleeve's four checks, in this order: the first three need no
        # search; the fourth needs the wall-between-the-bores search below.
        if not (sIn > 0):
            raise Exception(
                f'"cageRadius" leaves a bore channel outside the hollow (sIn={sIn} mm)')
        if not (math.hypot(axialWindow, math.hypot(W / 2.0, T / 2.0)) + clearance <= Ri):
            raise Exception(
                f'"cageRadius" hides the mesh along the axis '
                f'(hypot(axialWindow, hypot(W/2,T/2)) + clearance = '
                f'{math.hypot(axialWindow, math.hypot(W / 2.0, T / 2.0)) + clearance} mm, '
                f'Ri={Ri} mm)')
        if not (cageRise >= A / 2.0 + cc + collarWall):
            raise Exception(
                f'"cageRise" does not keep collarWall at the end faces '
                f'(need >= {A / 2.0 + cc + collarWall} mm, got {cageRise} mm)')

        # Each gear's level bore and roof face (S02 "The roof allowance"),
        # and the bores in build order: gear A -R, gear A +R, gear B -R,
        # gear B +R.
        bores = []
        for gi, gear in enumerate(gearFrames):
            lvl = _level_sign(gear, cageRadius, collarHalf)
            for sigma in (-1.0, 1.0):
                vLo, vHi = -ht, ht
                level = (sigma == lvl)
                if level:
                    if _roof_is_plus_v(gear, sigma, cageRadius):
                        vHi = ht + a
                    else:
                        vLo = -ht - a
                if sigma < 0:
                    frm, to = -sOut, -sIn
                else:
                    frm, to = sIn, sOut
                bores.append({
                    'name': f'{gear["label"]} Bore {"-R" if sigma < 0 else "+R"}',
                    'gear': gi, 'sigma': sigma, 'from': frm, 'to': to,
                    'vLo': vLo, 'vHi': vHi, 'level': level,
                })

        boreCrossings = []
        for bore in bores:
            gear = gearFrames[bore['gear']]
            boreCrossings.append(
                _add(gear['origin'], _scale(gear['dir'], bore['sigma'] * cageRadius)))

        # The wall between the bores (channelSeparation): sample each
        # bore's outline, hull it per gap, and take the largest-separating
        # edge of either hull.
        def sampleOutline(bore):
            gear = gearFrames[bore['gear']]
            sigma = bore['sigma']
            points = []
            j = 0
            while True:
                s = sigma * sIn + sigma * 0.1 * j
                if abs(s) > sOut + 1e-9:
                    break
                for i in range(17):
                    v = bore['vLo'] + (bore['vHi'] - bore['vLo']) * i / 16.0
                    points.append(_world_point(gear, s, hw, v))
                    points.append(_world_point(gear, s, -hw, v))
                for i in range(17):
                    u = -hw + 2.0 * hw * i / 16.0
                    points.append(_world_point(gear, s, u, bore['vHi']))
                    points.append(_world_point(gear, s, u, bore['vLo']))
                j += 1
            kept = []
            for p in points:
                radial = math.hypot(_dot(p, ehat), _dot(p, khat))
                if Ri - 0.5 <= radial <= Ro + 0.5:
                    kept.append(p)
            return kept

        boreOutlines = [sampleOutline(b) for b in bores]

        def gapSeparation(d, i, j):
            across = _cross(nhat, d)
            projA = [(_dot(p, across), _dot(p, nhat)) for p in boreOutlines[i]]
            projB = [(_dot(p, across), _dot(p, nhat)) for p in boreOutlines[j]]
            hullA = _convex_hull(projA)
            hullB = _convex_hull(projB)
            return _hull_separation(hullA, hullB)

        negEhat = _scale(ehat, -1.0)
        negKhat = _scale(khat, -1.0)
        gaps = {
            '+k': gapSeparation(khat, 1, 2),
            '-e': gapSeparation(negEhat, 0, 2),
            '-k': gapSeparation(negKhat, 0, 3),
            '+e': gapSeparation(ehat, 1, 3),
        }
        worstGapName = min(gaps, key=lambda k: gaps[k])
        if not (gaps[worstGapName] >= collarWall):
            raise Exception(
                f'"collarWall" is not kept at the {worstGapName} gap between bores '
                f'(separation={gaps[worstGapName]} mm, collarWall={collarWall} mm)')

        self.gearFrames = gearFrames
        self.bores = bores
        self.boreCrossings = boreCrossings

        # The trims' shared zLimit (channelTop): over all four bores, how
        # far from the selected plane the wall itself reaches.
        zLimit = 0.0
        for bore in bores:
            gear = gearFrames[bore['gear']]
            sigma = bore['sigma']
            j = 0
            while True:
                s = sigma * sIn + sigma * 0.01 * j
                if abs(s) > sOut + 1e-9:
                    break
                if abs(s) <= Ro:
                    th = _theta(gear, s)
                    cornersXY = [_turn(u, v, th) for (u, v) in (
                        (-hw, bore['vLo']), (hw, bore['vLo']),
                        (hw, bore['vHi']), (-hw, bore['vHi']))]
                    ymax = max(abs(y) for (_, y) in cornersXY)
                    if math.hypot(s, ymax) >= Ri:
                        ogN = gear['origin'][2]
                        uDotN = gear['uDotN']
                        candidate = max(abs(ogN + x * uDotN) for (x, _) in cornersXY)
                        if candidate > zLimit:
                            zLimit = candidate
                j += 1

        boreTables = [
            _build_bore_table(gearFrames[b['gear']], b, hw, Ri, Ro, cageRise) for b in bores
        ]

        def a0(t, Ri=Ri):
            return math.sqrt(max(0.0, Ri * Ri - t * t))

        def a1(t, Ro=Ro):
            return math.sqrt(max(0.0, Ro * Ro - t * t))

        def searchWindow(name, d):
            across = _cross(nhat, d)
            crossDotD = [_dot(c, d) for c in boreCrossings]
            flank = [i for i in range(4) if crossDotD[i] > 0]
            far = [i for i in range(4) if i not in flank]
            if len(flank) != 2 or len(far) != 2:
                raise Exception(
                    f'Window {name}: expected two flanking and two far bores, '
                    f'got flank={flank} far={far}')
            crossDotN = [_dot(c, nhat) for c in boreCrossings]
            if crossDotN[flank[0]] <= crossDotN[flank[1]]:
                lowIdx, highIdx = flank[0], flank[1]
            else:
                lowIdx, highIdx = flank[1], flank[0]
            lowT = _dot(boreCrossings[lowIdx], across)
            highT = _dot(boreCrossings[highIdx], across)
            lean = 1.0 if highT > lowT else -1.0

            def walkReach(boreIdx, takeMax) -> float:
                bore = bores[boreIdx]
                gear = gearFrames[bore['gear']]
                s1, s2 = bore['from'], bore['to']
                best = None
                k = 0
                while True:
                    s = s1 + 0.001 * k
                    if s > s2 + 1e-9:
                        break
                    for piece in _bore_pieces(gear, s, hw, bore['vLo'], bore['vHi'], Ri, Ro, cageRise):
                        for (x, y) in piece:
                            p = _add(
                                _add(gear['origin'], _add(_scale(gear['u'], x), _scale(gear['v'], y))),
                                _scale(gear['dir'], s))
                            t = _dot(p, across)
                            z = _dot(p, nhat)
                            m = z + lean * t
                            if best is None or (takeMax and m > best) or ((not takeMax) and m < best):
                                best = m
                    k += 1
                if best is None:
                    raise Exception(
                        f'{bore["name"]}: no station in [{s1}, {s2}] mm produced a wall '
                        f'section; cannot find the window\'s long side')
                return best

            lowReach = walkReach(lowIdx, True)
            highReach = walkReach(highIdx, False)
            lo = lowReach + math.sqrt(2.0) * collarWall
            hi = highReach - math.sqrt(2.0) * collarWall

            top = min(2.0 * zLimit - hi, hi + math.sqrt(2.0) * Ri)
            bottom = max(-2.0 * zLimit - lo, lo - math.sqrt(2.0) * Ri)

            need = collarWall + 0.1 / math.sqrt(2.0) + 0.005

            def clearPoints(t, zLow, zHigh):
                a0t, a1t = a0(t), a1(t)
                pts = []
                for z in (zLow, zHigh):
                    av = a0t
                    while True:
                        if av >= a1t:
                            pts.append((t, z, a1t))
                            break
                        pts.append((t, z, av))
                        av += 0.1
                nz = max(1, math.ceil((zHigh - zLow) / 0.1)) if zHigh > zLow else 1
                for av in (a0t, a1t):
                    for k in range(1, nz):
                        z = zLow + (zHigh - zLow) * k / nz
                        pts.append((t, z, av))
                    j = 0
                    while True:
                        z = -zLimit + 0.1 * j
                        if z > zLimit + 1e-9:
                            break
                        if zLow < z < zHigh:
                            pts.append((t, z, av))
                        j += 1
                return pts

            def clear(te, delta):
                t = delta * te
                zLow = max(lo - lean * t, bottom + lean * t)
                zHigh = min(hi - lean * t, top + lean * t)
                if zLow > zHigh:
                    return False
                for (tt, z, av) in clearPoints(t, zLow, zHigh):
                    Q = _add(_add(_scale(across, tt), _scale(nhat, z)), _scale(d, av))
                    for fi in far:
                        gap = _wall_gap(
                            Q, gearFrames[bores[fi]['gear']], bores[fi], boreTables[fi], cc, need)
                        if gap < need:
                            return False
                return True

            def windowEnd(delta):
                bound = min(Ri, Ro / math.sqrt(2.0)) * (1.0 - 1e-9)
                low, high = 0.0, bound
                for _ in range(24):
                    mid = (low + high) / 2.0
                    if clear(mid, delta):
                        low = mid
                    else:
                        high = mid
                return low

            right = windowEnd(1.0)
            left = -windowEnd(-1.0)

            if hi <= lo:
                futil.log(f'No window facing {name}: hi <= lo')
                return None
            if right <= left:
                futil.log(f'No window facing {name}: right <= left')
                return None

            q = [(-2.0 * Ro, -2.0 * Ro), (2.0 * Ro, -2.0 * Ro),
                 (2.0 * Ro, 2.0 * Ro), (-2.0 * Ro, 2.0 * Ro)]
            q = _clip_poly(q, lean, 1.0, hi)
            q = _clip_poly(q, -lean, -1.0, -lo)
            q = _clip_poly(q, -lean, 1.0, top)
            q = _clip_poly(q, lean, -1.0, -bottom)
            q = _clip_poly(q, 1.0, 0.0, right)
            q = _clip_poly(q, -1.0, 0.0, -left)
            corners = []
            for c in q:
                if corners and math.hypot(c[0] - corners[-1][0], c[1] - corners[-1][1]) < 0.001:
                    continue
                corners.append(c)
            if len(corners) > 1 and math.hypot(
                    corners[0][0] - corners[-1][0], corners[0][1] - corners[-1][1]) < 0.001:
                corners.pop()

            if len(corners) < 3:
                futil.log(f'No window facing {name}: fewer than three corners')
                return None
            area = 0.0
            for i in range(len(corners)):
                p0, p1 = corners[i], corners[(i + 1) % len(corners)]
                area += p0[0] * p1[1] - p1[0] * p0[1]
            area = abs(area) / 2.0
            if area < 1e-9:
                futil.log(f'No window facing {name}: no area')
                return None

            return {'name': name, 'd': d, 'across': across, 'corners': corners}

        if crossAngleDeg <= 90.0:
            axisName, axisVec = 'k', khat
        else:
            axisName, axisVec = 'e', ehat
        candidateDirs = [
            ('+' + axisName, axisVec),
            ('-' + axisName, _scale(axisVec, -1.0)),
        ]

        self.windows = []
        for wname, wd in candidateDirs:
            result = searchWindow(wname, wd)
            if result is not None:
                self.windows.append(result)

    # -----------------------------------------------------------------
    # S04: generate and the component tree.
    # -----------------------------------------------------------------

    def generate(self, inputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        self.buildGear(0)
        self.buildGear(1)
        self.buildCage()
        self.relocateBodies()
        solids.hide_construction_geometry(self.designOcc.component)

    def buildComponentTree(self):
        topOcc = self.getOccurrence()
        topOcc.component.name = 'Screw Gearing'
        topComponent = topOcc.component

        self.designOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.designOcc.component.name = 'Design'

        gearAOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearAOcc.component.name = 'Gear A'
        gearBOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearBOcc.component.name = 'Gear B'
        self.gearOccs = [gearAOcc, gearBOcc]

        self.cageOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.cageOcc.component.name = 'Cage'

        # Field declarations in the cast(None) form, before anything reads
        # them: a concrete (never-Optional) element type for each container,
        # even though the runtime value is None until the real build fills
        # it in.
        self.gearBodies = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]
        self.realGears = [{}, {}]
        self._cellBody = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]
        self.axisPlanes = [
            adsk.fusion.ConstructionPlane.cast(None), adsk.fusion.ConstructionPlane.cast(None)
        ]

    # -----------------------------------------------------------------
    # S05: Anchor sketch.
    # -----------------------------------------------------------------

    def buildAnchor(self):
        component = self.designOcc.component
        sketch = component.sketches.add(self.plane)
        sketch.name = 'Anchor'

        projected = sketch.project(self.anchorPoint).item(0)
        px, py = projected.geometry.x, projected.geometry.y
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(px - 0.5, py, 0),
            adsk.core.Point3D.create(px + 0.5, py, 0))

        sketch.geometricConstraints.addCoincident(projected, line)
        sketch.geometricConstraints.addMidPoint(projected, line)
        sketch.geometricConstraints.addHorizontal(line)
        textPoint = adsk.core.Point3D.create(px, py + 0.3, 0)
        sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        centerPoint = projected.worldGeometry
        startW = line.startSketchPoint.worldGeometry
        endW = line.endSketchPoint.worldGeometry
        eVec = adsk.core.Vector3D.create(endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
        eVec.normalize()

        self.anchorLine = line
        self.centerPointCm = centerPoint
        self.center = (centerPoint.x * 10.0, centerPoint.y * 10.0, centerPoint.z * 10.0)
        self.eHat = (eVec.x, eVec.y, eVec.z)

    # -----------------------------------------------------------------
    # S06 (per gear, folded into buildGear) + S07, S08, S09, S10.
    # -----------------------------------------------------------------

    def buildGear(self, index):
        component = self.designOcc.component
        label = 'Gear A' if index == 0 else 'Gear B'

        offsetCm = (-1.0 if index == 0 else 1.0) * (self.A / 2.0 / 10.0)
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetCm))
        axisPlane = component.constructionPlanes.add(planeInput)
        axisPlane.name = f'{label} Axis Plane'

        gA = axisPlane.geometry
        nrm = gA.normal
        nrm.normalize()
        toC = adsk.core.Vector3D.create(
            self.centerPointCm.x - gA.origin.x,
            self.centerPointCm.y - gA.origin.y,
            self.centerPointCm.z - gA.origin.z)
        dotVal = toC.dotProduct(nrm)

        if index == 0:
            if dotVal > 0:
                self.nHat = (nrm.x, nrm.y, nrm.z)
            else:
                self.nHat = (-nrm.x, -nrm.y, -nrm.z)
            self.kHat = _cross(self.nHat, self.eHat)

        reading = abs(dotVal)
        wantCm = self.A / 2.0 / 10.0
        if abs(reading - wantCm) > 1e-6:
            raise Exception(f'{label} Axis Plane: reads {reading} cm, want {wantCm} cm')

        origin, dirg, u, v = _gear_axes(
            index, self.eHat, self.kHat, self.nHat, self.center, self.Sigma, self.A)
        self.realGears[index] = {
            'label': label, 'origin': origin, 'dir': dirg, 'u': u, 'v': v,
            'phi': self.mountAngleA if index == 0 else self.mountAngleB,
            'lam': self.Lambda,
            'z0': 0.0 if index == 0 else self.assemblyPhase,
            'uDotN': 1.0 if index == 0 else -1.0,
        }
        self.axisPlanes[index] = axisPlane

        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, index):
        gear = self.realGears[index]
        label = gear['label']
        sketch = self.designOcc.component.sketches.add(self.axisPlanes[index])
        sketch.name = f'{label} Paths'

        pts = []
        for s in (-self.sOut, -self.sIn, self.sIn, self.sOut):
            worldPoint = _add(gear['origin'], _scale(gear['dir'], s))
            local = sketch.modelToSketchSpace(adsk.core.Point3D.create(
                worldPoint[0] / 10.0, worldPoint[1] / 10.0, worldPoint[2] / 10.0))
            local.z = 0
            pts.append(sketch.sketchPoints.add(local))

        boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[0], pts[1])
        borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[2], pts[3])
        for pt in pts:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{label} Paths sketch is not fully constrained')

        self.pathLines[index] = {'bore-': boreMinus, 'bore+': borePlus}

    def buildToothCell(self, index):
        gear = self.realGears[index]
        label = gear['label']
        Z0 = gear['z0']
        s0 = Z0 - self.L / 2.0

        sketch = self.designOcc.component.sketches.add(self.axisPlanes[index])
        sketch.name = f'{label} Cell Sections'
        sketch.isComputeDeferred = True

        def utooth(s):
            return self.W / 2.0 - self.H / 2.0 + (self.H / 2.0) * math.cos(
                2.0 * math.pi * (s - Z0) / self.P)

        sections = []
        for k in range(self.c * self.n + 1):
            s = s0 + k * self.P / self.n
            uB, uF = -self.W / 2.0, utooth(s)
            corners = []
            for (u, v) in ((uB, -self.T / 2.0), (uF, -self.T / 2.0),
                           (uF, self.T / 2.0), (uB, self.T / 2.0)):
                worldPoint = _world_point(gear, s, u, v)
                local = sketch.modelToSketchSpace(adsk.core.Point3D.create(
                    worldPoint[0] / 10.0, worldPoint[1] / 10.0, worldPoint[2] / 10.0))
                corners.append(sketch.sketchPoints.add(local))
            lines = []
            for i in range(4):
                lines.append(sketch.sketchCurves.sketchLines.addByTwoPoints(
                    corners[i], corners[(i + 1) % 4]))
            sections.append((lines, corners))

        for lines, corners in sections:
            for pt in corners:
                pt.isFixed = True

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{label} Cell Sections sketch is not fully constrained')
        want = self.c * self.n + 1
        if sketch.profiles.count != want:
            raise Exception(
                f'{label} Cell Sections sketch has {sketch.profiles.count} profiles, want {want}')

        sectionLines = [lines for lines, _ in sections]
        self._cellBody[index] = self._loftSections(sectionLines, f'{label} Cell')

    def _loftSections(self, sectionsLines, pieceLabel):
        component = self.designOcc.component
        loftInput = component.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for lines in sectionsLines:
            coll = adsk.core.ObjectCollection.create()
            for line in lines:
                coll.add(line)
            path = component.features.createPath(coll, False)
            loftInput.loftSections.add(path)
        loft = component.features.loftFeatures.add(loftInput)
        if not (loft.bodies.count == 1 and loft.bodies.item(0).isSolid):
            raise Exception(f'{pieceLabel}: loft produced {loft.bodies.count} body(ies)')
        return loft.bodies.item(0)

    def repeatCellByDoubling(self, index):
        gear = self.realGears[index]
        label = gear['label']
        body = self._cellBody[index]
        c, q, r = self.c, self.q, self.r

        m = 1
        asides = []
        if q > 1:
            topBit = 0
            qScan = q
            while qScan > 1:
                qScan //= 2
                topBit += 1
            for i in range(topBit):
                if (q >> i) & 1:
                    asideBody = self._copyBody(body, label)
                    asides.append((m, asideBody))
                moveTool = self._copyBody(body, label)
                self._moveByScrewStep(gear, moveTool, m * c)
                body = self._joinBodies(body, moveTool, label)
                m *= 2
            asides.sort(key=lambda item: item[0], reverse=True)
            for cells, asideBody in asides:
                self._moveByScrewStep(gear, asideBody, m * c)
                body = self._joinBodies(body, asideBody, label)
                m += cells

        if r > 0:
            remainderLines = self.buildRemainderSections(index)
            remainderBody = self._loftSections(remainderLines, f'{label} Cell Remainder')
            body = self._joinBodies(body, remainderBody, label)

        body.name = label
        self.gearBodies[index] = body

    def _copyBody(self, body, label):
        component = self.designOcc.component
        copyFeature = component.features.copyPasteBodies.add(body)
        if copyFeature.bodies.count != 1:
            raise Exception(f'{label}: copy produced {copyFeature.bodies.count} body(ies)')
        return copyFeature.bodies.item(0)

    def _moveByScrewStep(self, gear, body, k):
        component = self.designOcc.component
        axisVector = adsk.core.Vector3D.create(gear['dir'][0], gear['dir'][1], gear['dir'][2])
        axisPoint = adsk.core.Point3D.create(
            gear['origin'][0] / 10.0, gear['origin'][1] / 10.0, gear['origin'][2] / 10.0)

        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * self.P / self.Lambda, axisVector, axisPoint)
        shift = axisVector.copy()
        shift.scaleBy(k * self.P / 10.0)
        mov = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = component.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        component.features.moveFeatures.add(moveInput)

    def _joinBodies(self, body, tool, label):
        component = self.designOcc.component
        tools = adsk.core.ObjectCollection.create()
        tools.add(tool)
        joinInput = component.features.combineFeatures.createInput(body, tools)
        joinInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        joinInput.isKeepToolBodies = False
        join = component.features.combineFeatures.add(joinInput)
        if join.bodies.count != 1:
            raise Exception(f'{label}: join produced {join.bodies.count} body(ies)')
        return join.bodies.item(0)

    def buildRemainderSections(self, index):
        gear = self.realGears[index]
        label = gear['label']
        Z0 = gear['z0']
        s0 = Z0 - self.L / 2.0
        base = s0 + self.q * self.c * self.P

        sketch = self.designOcc.component.sketches.add(self.axisPlanes[index])
        sketch.name = f'{label} Cell Remainder'
        sketch.isComputeDeferred = True

        def utooth(s):
            return self.W / 2.0 - self.H / 2.0 + (self.H / 2.0) * math.cos(
                2.0 * math.pi * (s - Z0) / self.P)

        sections = []
        for k in range(self.r * self.n + 1):
            s = base + k * self.P / self.n
            uF = utooth(s)
            corners = []
            for (u, v) in ((-self.W / 2.0, -self.T / 2.0), (uF, -self.T / 2.0),
                           (uF, self.T / 2.0), (-self.W / 2.0, self.T / 2.0)):
                worldPoint = _world_point(gear, s, u, v)
                local = sketch.modelToSketchSpace(adsk.core.Point3D.create(
                    worldPoint[0] / 10.0, worldPoint[1] / 10.0, worldPoint[2] / 10.0))
                corners.append(sketch.sketchPoints.add(local))
            lines = []
            for i in range(4):
                lines.append(sketch.sketchCurves.sketchLines.addByTwoPoints(
                    corners[i], corners[(i + 1) % 4]))
            sections.append((lines, corners))

        for lines, corners in sections:
            for pt in corners:
                pt.isFixed = True

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{label} Cell Remainder sketch is not fully constrained')
        want = self.r * self.n + 1
        if sketch.profiles.count != want:
            raise Exception(
                f'{label} Cell Remainder sketch has {sketch.profiles.count} profiles, want {want}')

        return [lines for lines, _ in sections]

    # -----------------------------------------------------------------
    # S16-S22: buildCage.
    # -----------------------------------------------------------------

    def buildCage(self):
        self.buildSleeveSketchAndTube()
        for boreIndex in range(4):
            self.buildBoreSectionAndCut(boreIndex)
        self.buildWindows()
        self.cageBody.name = 'Cage'
        futil.log(
            'Print the cage standing on its end below the selected plane: '
            'the roof allowance is on the bridged roofs that way up.')

    def buildSleeveSketchAndTube(self):
        component = self.designOcc.component
        sketch = component.sketches.add(self.plane)
        sketch.name = 'Sleeve'

        centre = sketch.modelToSketchSpace(self.centerPointCm)
        centre.z = 0

        riCm, roCm = self.Ri / 10.0, self.Ro / 10.0
        inner = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, riCm)
        inner.centerSketchPoint.isFixed = True
        innerText = adsk.core.Point3D.create(centre.x + riCm, centre.y, 0)
        sketch.sketchDimensions.addDiameterDimension(inner, innerText)

        outer = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, roCm)
        outer.centerSketchPoint.isFixed = True
        outerText = adsk.core.Point3D.create(centre.x + roCm, centre.y, 0)
        sketch.sketchDimensions.addDiameterDimension(outer, outerText)

        if not sketch.isFullyConstrained:
            raise Exception('Sleeve sketch is not fully constrained')

        ring = None
        ringCount = 0
        for i in range(sketch.profiles.count):
            profile = sketch.profiles.item(i)
            if profile.profileLoops.count == 2:
                ring = profile
                ringCount += 1
        if ringCount != 1 or ring is None:
            raise Exception(
                f'Sleeve sketch: {sketch.profiles.count} profiles, '
                f'{ringCount} with two loops; want 1')
        ring = adsk.fusion.Profile.cast(ring)

        extrudeInput = component.features.extrudeFeatures.createInput(
            ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(self.cageRise / 10.0), False)
        tube = component.features.extrudeFeatures.add(extrudeInput)
        if tube.bodies.count != 1:
            raise Exception(f'Sleeve tube: extrude produced {tube.bodies.count} body(ies)')
        self.cageBody = tube.bodies.item(0)

    def buildBoreSectionAndCut(self, boreIndex):
        bore = self.bores[boreIndex]
        gear = self.realGears[bore['gear']]
        label = gear['label']
        sigma = bore['sigma']
        sideName = '-R' if sigma < 0 else '+R'
        key = 'bore-' if sigma < 0 else 'bore+'
        line = self.pathLines[bore['gear']][key]
        component = self.designOcc.component

        planeInput = component.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))
        plane = component.constructionPlanes.add(planeInput)
        plane.name = f'{label} Bore {sideName} Plane'

        sketch = component.sketches.add(plane)
        sketch.name = f'{label} Bore {sideName}'
        sketch.isComputeDeferred = True

        s0 = bore['from']
        th = _theta(gear, s0)
        hw, vLo, vHi = self.hw, bore['vLo'], bore['vHi']
        uB, uF = -hw, hw
        axisOffset = self.A

        def toLocal(sketch: adsk.fusion.Sketch, worldMm) -> adsk.core.Point3D:
            p = adsk.core.Point3D.create(
                worldMm[0] / 10.0, worldMm[1] / 10.0, worldMm[2] / 10.0)
            local = sketch.modelToSketchSpace(p)
            local.z = 0
            return local

        # 1. References.
        oWorld = _world_point(gear, s0, 0.0, 0.0)
        cpWorld = _add(oWorld, _scale(gear['u'], axisOffset / 2.0))
        O = sketch.sketchPoints.add(toLocal(sketch, oWorld))
        Cp = sketch.sketchPoints.add(toLocal(sketch, cpWorld))
        Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True

        # 2. The spine.
        seedEWorld = _world_point(gear, s0, uF, 0.0)
        K = sketch.sketchCurves.sketchLines.addByTwoPoints(O, toLocal(sketch, seedEWorld))
        K.isConstruction = True
        E = K.endSketchPoint
        O.isFixed = True
        Cp.isFixed = True

        # 3. The rectangle.
        p1 = toLocal(sketch, _world_point(gear, s0, uB, vLo))
        p2 = toLocal(sketch, _world_point(gear, s0, uF, vLo))
        p3 = toLocal(sketch, _world_point(gear, s0, uF, vHi))
        p4 = toLocal(sketch, _world_point(gear, s0, uB, vHi))
        L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
        L2 = sketch.sketchCurves.sketchLines.addByTwoPoints(L1.endSketchPoint, p3)
        L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(L2.endSketchPoint, p4)
        L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)

        # 4. Constraints and dimensions, all driving, in order.
        textK = toLocal(sketch, _world_point(gear, s0, uF / 2.0, 0.5))
        lengthDim = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textK)
        lengthDim.parameter.value = uF / 10.0

        uG = gear['u']
        if abs(math.sin(th)) >= math.sqrt(0.5):
            second = K
            psi = th
            e1 = uG
            e2 = _unit(_sub(seedEWorld, oWorld))
            xWorld = oWorld
        else:
            second = L2
            psi = th + math.pi / 2.0
            e1 = uG
            l2startWorld = _world_point(gear, s0, uF, vLo)
            l2endWorld = _world_point(gear, s0, uF, vHi)
            e2 = _unit(_sub(l2endWorld, l2startWorld))
            xWorld = seedEWorld
        unsigned = abs(_fold(psi))

        unitSum = _unit(_add(e1, e2))
        xTextCm = _add(
            (xWorld[0] / 10.0, xWorld[1] / 10.0, xWorld[2] / 10.0),
            _scale(unitSum, 0.3))
        textA = sketch.modelToSketchSpace(adsk.core.Point3D.create(*xTextCm))
        textA.z = 0
        angleDim = sketch.sketchDimensions.addAngularDimension(Ru, second, textA)
        angleDim.parameter.value = unsigned

        textL1 = toLocal(sketch, _world_point(gear, s0, uF / 2.0, vLo / 2.0))
        sketch.geometricConstraints.addParallel(L1, K)
        offset1 = sketch.sketchDimensions.addOffsetDimension(K, L1, textL1)
        offset1.parameter.value = -vLo / 10.0

        textL3 = toLocal(sketch, _world_point(gear, s0, uF / 2.0, vHi / 2.0))
        sketch.geometricConstraints.addParallel(L3, K)
        offset3 = sketch.sketchDimensions.addOffsetDimension(K, L3, textL3)
        offset3.parameter.value = vHi / 10.0

        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)

        textL4 = toLocal(sketch, _world_point(gear, s0, 0.0, (vLo + vHi) / 2.0))
        sketch.geometricConstraints.addParallel(L4, L2)
        offset4 = sketch.sketchDimensions.addOffsetDimension(L2, L4, textL4)
        offset4.parameter.value = (uF - uB) / 10.0

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{label} Bore {sideName} sketch is not fully constrained')

        profile = adsk.fusion.Profile.cast(find_profile_by_curve_counts(sketch, lines=4))

        path = component.features.createPath(line, False)
        sweepInput = component.features.sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(
            (self.sOut - self.sIn) / self.Lambda)
        sweepInput.participantBodies = [self.cageBody]
        sweep = component.features.sweepFeatures.add(sweepInput)
        if sweep.bodies.count != 1:
            raise Exception(f'{bore["name"]}: sweep produced {sweep.bodies.count} body(ies)')
        self.cageBody = sweep.bodies.item(0)

        sc = sigma * self.cageRadius
        thc = _theta(gear, sc)
        uhat = _add(_scale(gear['u'], math.cos(thc)), _scale(gear['v'], math.sin(thc)))
        centreWorld = _add(gear['origin'], _scale(gear['dir'], sc))
        probeDist = self.W / 2.0 + self.clearance / 2.0
        for sign in (1.0, -1.0):
            probeWorld = _add(centreWorld, _scale(uhat, sign * probeDist))
            probePoint = adsk.core.Point3D.create(
                probeWorld[0] / 10.0, probeWorld[1] / 10.0, probeWorld[2] / 10.0)
            containment = self.cageBody.pointContainment(probePoint)
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise Exception(
                    f'{bore["name"]}: probe (sign {sign}) reads containment {containment}, '
                    f'want outside')

    def _abstractToReal(self, abstractVec):
        # Rebuild an abstract-frame unit vector (+-e_hat or +-k_hat of S03)
        # in the real frame of S06.
        x, y, _ = abstractVec
        if abs(x) > 0.5:
            return _scale(self.eHat, x)
        return _scale(self.kHat, y)

    def buildWindows(self):
        if not self.windows:
            return
        component = self.designOcc.component

        crossAngleDeg = math.degrees(self.crossAngle)
        planeInput = component.constructionPlanes.createInput()
        if crossAngleDeg <= 90.0:
            planeInput.setByAngle(
                self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)
        else:
            planeInput.setByDistanceOnPath(
                self.anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane = component.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'

        for window in self.windows:
            self.buildWindowSketchAndCut(windowPlane, window)

    def buildWindowSketchAndCut(self, windowPlane, window):
        component = self.designOcc.component
        name = window['name']
        d = self._abstractToReal(window['d'])
        across = _cross(self.nHat, d)

        sketch = component.sketches.add(windowPlane)
        sketch.name = f'Window {name}'

        pts = []
        for (t, z) in window['corners']:
            worldMm = _add(_add(self.center, _scale(across, t)), _scale(self.nHat, z))
            local = sketch.modelToSketchSpace(adsk.core.Point3D.create(
                worldMm[0] / 10.0, worldMm[1] / 10.0, worldMm[2] / 10.0))
            local.z = 0
            pts.append(sketch.sketchPoints.add(local))

        n = len(pts)
        for i in range(n):
            sketch.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % n])
        for pt in pts:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'Window {name} sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(f'Window {name} sketch has {sketch.profiles.count} profiles, want 1')
        profile = sketch.profiles.item(0)

        tc = sum(c[0] for c in window['corners']) / n
        zc = sum(c[1] for c in window['corners']) / n
        a0tc = math.sqrt(max(0.0, self.Ri * self.Ri - tc * tc))
        a1tc = math.sqrt(max(0.0, self.Ro * self.Ro - tc * tc))
        probeWorldMm = _add(
            _add(self.center, _add(_scale(across, tc), _scale(self.nHat, zc))),
            _scale(d, (a0tc + a1tc) / 2.0))
        probePoint = adsk.core.Point3D.create(
            probeWorldMm[0] / 10.0, probeWorldMm[1] / 10.0, probeWorldMm[2] / 10.0)
        containment = self.cageBody.pointContainment(probePoint)
        if containment != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(f'Window {name}: probe reads containment {containment}, want inside')

        cutInput = component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        dirCheck = sketch.modelToSketchSpace(adsk.core.Point3D.create(
            self.center[0] / 10.0 + d[0],
            self.center[1] / 10.0 + d[1],
            self.center[2] / 10.0 + d[2]))
        if dirCheck.z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
        cutInput.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByReal(self.Ro / 10.0 + 0.1)),
            direction)
        cutInput.participantBodies = [self.cageBody]
        cut = component.features.extrudeFeatures.add(cutInput)
        if cut.bodies.count != 1:
            raise Exception(f'Window {name}: cut produced {cut.bodies.count} body(ies)')
        self.cageBody = cut.bodies.item(0)

        containment = self.cageBody.pointContainment(probePoint)
        if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(
                f'Window {name}: probe after cut reads containment {containment}, want outside')

    # -----------------------------------------------------------------
    # S23: relocate the bodies.
    # -----------------------------------------------------------------

    def relocateBodies(self):
        gearBodyA: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[0])
        gearBodyB: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[1])
        gearBodyA.moveToComponent(self.gearOccs[0])
        gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody.moveToComponent(self.cageOcc)
