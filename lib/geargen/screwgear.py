# Screw gearing: two twisted toothed ribbons meshing at a crossing angle, held in
# a slotted sleeve. Generated from the compiled step list spec/screwgear/steps.md.
#
# Parameter mode: all-Python-precomputed ([PB-PRECOMPUTED-MODE]). Every value is
# computed in Python in internal cm and written numerically; no named user
# parameter is registered. The two searches that run in processInputs (the wall
# between the bores and the window search) work in millimetres in the frame of
# §1 (C at the origin, ê, k̂ and n̂ on X, Y and Z), and every length they hand on
# is divided by ten.

import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import get_design
from .base import Generator
from .utilities import find_profile_by_curve_counts
from . import solids


# Dialog input ids.
INPUT_ID_PLANE = 'plane'
INPUT_ID_POINT = 'point'
INPUT_ID_PARENT = 'parent'
INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'
INPUT_ID_TOOTH_COUNT = 'toothCount'
INPUT_ID_TWIST_LEAD = 'twistLead'
INPUT_ID_RIBBON_THICKNESS = 'ribbonThickness'
INPUT_ID_TOOTH_PITCH = 'toothPitch'
INPUT_ID_TOOTH_HEIGHT = 'toothHeight'
INPUT_ID_TOOTH_SLANT = 'toothSlant'
INPUT_ID_TOOTH_BOW = 'toothBow'
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

# Dialog groups.
GROUP_ID_RIBBON = 'ribbonGroup'
GROUP_ID_FRAME = 'frameGroup'
GROUP_ID_MESH = 'meshGroup'

# How many points each section's toothed side is fitted through (§2).
TOOTH_SPLINE_POINTS = 11
# How many teeth the lofted cell of §2 holds ('cellTeeth').
CELL_TEETH = 4

GEAR_LABELS = ('Gear A', 'Gear B')

PRINT_ORIENTATION_MESSAGE = ('Print the cage standing on its end below the selected plane: '
                             'the roof allowance is on the bridged roofs that way up.')

# The window search's sampling step and slack, in mm ('windowStep', 'windowSlack').
WINDOW_STEP_MM = 0.1
WINDOW_SLACK_MM = WINDOW_STEP_MM / math.sqrt(2) + 5e-3


# ---------------------------------------------------------------------------
# Plain vector arithmetic on 3-tuples.

def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vscale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _vlen(a):
    return math.sqrt(_vdot(a, a))


def _vunit(a, what):
    length = _vlen(a)
    if length <= 0:
        raise Exception(f'Screw gear: {what} has zero length and no direction')
    return (a[0] / length, a[1] / length, a[2] / length)


def _ceil_int(x):
    # Plain-arithmetic ceiling for a finite float.
    n = int(x)
    if n < x:
        n += 1
    return n


# ---------------------------------------------------------------------------
# The precompute searches of §4, in millimetres, in the frame of §1: C at the
# origin, ê on X, k̂ on Y and n̂ on Z. A gear is the list
# [origin, ex, ey, ez, mount, lambda], where ex is û, ey is v̂ and ez the axis
# direction; this is the proof's Gear with Hand +1.

def _gear_angle(g, s):
    return s / g[5] + g[4]


def _gear_world(g, u, v, s):
    th = _gear_angle(g, s)
    c = math.cos(th)
    sn = math.sin(th)
    x = u * c - v * sn
    y = u * sn + v * c
    o = g[0]
    ex = g[1]
    ey = g[2]
    ez = g[3]
    return (o[0] + ex[0] * x + ey[0] * y + ez[0] * s,
            o[1] + ex[1] * x + ey[1] * y + ez[1] * s,
            o[2] + ex[2] * x + ey[2] * y + ez[2] * s)


def _roof_side(sv, g, sign):
    # +1 when the +v face is the bore's roof with the sleeve on its -n end.
    th = _gear_angle(g, sign * sv['cageRadius'])
    if -math.sin(th) * g[1][2] + math.cos(th) * g[2][2] > 0:
        return 1
    return -1


def _bore_opening(sv, g, sign):
    hw = sv['hw']
    ht = sv['ht']
    vLo = -ht
    vHi = ht
    if _roof_side(sv, g, sign) > 0:
        return hw, vLo, vHi + sv['roof']
    return hw, vLo - sv['roof'], vHi


def _opening_corners(sv, g, sign):
    hw, vLo, vHi = _bore_opening(sv, g, sign)
    return [[-hw, vLo], [hw, vLo], [hw, vHi], [-hw, vHi]]


def _copysign1(s):
    if s < 0:
        return -1.0
    return 1.0


def _clip(poly, a, b, c):
    # Keep the part of a convex polygon where a*x + b*y <= c.
    out = []
    n = len(poly)
    for i in range(n):
        p = poly[i]
        r = poly[(i + 1) % n]
        fp = c - (a * p[0] + b * p[1])
        fr = c - (a * r[0] + b * r[1])
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fr >= 0):
            k = fp / (fp - fr)
            out.append([p[0] + k * (r[0] - p[0]), p[1] + k * (r[1] - p[1])])
    return out


def _gap2(poly, x, y):
    # Squared distance from (x, y) to a counter-clockwise polygon, zero inside.
    n = len(poly)
    inside = n >= 3
    best = math.inf
    for i in range(n):
        p = poly[i]
        r = poly[(i + 1) % n]
        ex = r[0] - p[0]
        ey = r[1] - p[1]
        if ex * (y - p[1]) - ey * (x - p[0]) < 0:
            inside = False
        k = 0.0
        l2 = ex * ex + ey * ey
        if l2 > 0:
            k = max(0.0, min(1.0, ((x - p[0]) * ex + (y - p[1]) * ey) / l2))
        dx = x - p[0] - k * ex
        dy = y - p[1] - k * ey
        best = min(best, dx * dx + dy * dy)
    if inside:
        return 0.0
    return best


def _section_in_wall(sv, g, s):
    # 'sectionInWall': the bore's section at station s cut to the two pieces
    # that lie in the tube, in the gear's own (x along û, y along v̂).
    out = [[], []]
    ro = sv['ro']
    ri = sv['ri']
    if abs(s) >= ro:
        return out
    th = _gear_angle(g, s)
    c = math.cos(th)
    sn = math.sin(th)
    rect = []
    for uv in _opening_corners(sv, g, _copysign1(s)):
        rect.append([uv[0] * c - uv[1] * sn, uv[0] * sn + uv[1] * c])
    zb = sv['zb']
    exz = g[1][2]
    xa = (-zb - g[0][2]) / exz
    xb = (zb - g[0][2]) / exz
    if xa > xb:
        xa, xb = xb, xa
    rect = _clip(_clip(rect, 1.0, 0.0, xb), -1.0, 0.0, -xa)
    near = math.sqrt(max(0.0, ri * ri - s * s))
    far = math.sqrt(ro * ro - s * s)
    sides = (-1.0, 1.0)
    for i in range(2):
        side = sides[i]
        out[i] = _clip(_clip(rect, 0.0, side, far), 0.0, -side, -near)
    return out


def _bores(sv):
    # The four bores in order: gear A -R, gear A +R, gear B -R, gear B +R.
    # Each is [gearIndex, sign, name, index].
    out = []
    names = ('gear A', 'gear B')
    for gi in range(2):
        signs = (-1.0, 1.0)
        for si in range(2):
            sign = signs[si]
            suffix = ' -R' if sign < 0 else ' +R'
            out.append([gi, sign, names[gi] + suffix, gi * 2 + si])
    return out


def _bore_span(sv, bore):
    if bore[1] > 0:
        return sv['sIn'], sv['sOut']
    return -sv['sOut'], -sv['sIn']


def _bore_crossing(sv, bore):
    g = sv['gears'][bore[0]]
    return _vadd(g[0], _vscale(g[3], bore[1] * sv['cageRadius']))


def _make_station(sv, g, s):
    pieces = _section_in_wall(sv, g, s)
    cx = [0.0, 0.0]
    cy = [0.0, 0.0]
    rr = [0.0, 0.0]
    for i in range(2):
        piece = pieces[i]
        n = len(piece)
        if n == 0:
            continue
        for k in range(n):
            cx[i] += piece[k][0] / n
            cy[i] += piece[k][1] / n
        for k in range(n):
            rr[i] = max(rr[i], math.hypot(piece[k][0] - cx[i], piece[k][1] - cy[i]))
    return [s, pieces, cx, cy, rr]


def _station_has(station):
    pieces = station[1]
    return (len(pieces[0]) > 0, len(pieces[1]) > 0)


def _channel_sections(sv, bore):
    # The bore's station table: every 2 microns along the cut, and every tenth
    # of a micron between two of those where the set of non-empty pieces
    # changes.
    step = 0.002
    fine = 0.0001
    lo, hi = _bore_span(sv, bore)
    g = sv['gears'][bore[0]]
    out = []
    k = 0
    while True:
        s = lo + k * step
        if s > hi:
            break
        st = _make_station(sv, g, s)
        if k > 0 and _station_has(st) != _station_has(out[len(out) - 1]):
            prev = out[len(out) - 1][0]
            j = 1
            while prev + j * fine < s:
                out.append(_make_station(sv, g, prev + j * fine))
                j += 1
        out.append(st)
        k += 1
    return out


def _station_table(sv, bore):
    tables = sv['sections']
    index = bore[3]
    if tables[index] is None:
        tables[index] = _channel_sections(sv, bore)
    return tables[index]


def _wall_visit(st, sq, x, y, best):
    # One station of the walk: returns (goOn, best).
    ds = (sq - st[0]) * (sq - st[0])
    if ds >= best:
        return False, best
    pieces = st[1]
    for i in range(2):
        piece = pieces[i]
        if len(piece) == 0:
            continue
        o = math.hypot(x - st[2][i], y - st[3][i]) - st[4][i]
        if o > 0 and ds + o * o >= best:
            continue
        best = min(best, ds + _gap2(piece, x, y))
    return True, best


def _wall_gap(sv, bore, q, reach):
    # 'wallGap': the distance from q to the bore's channel in the tube, or
    # reach when it is at least that far.
    g = sv['gears'][bore[0]]
    lo, hi = _bore_span(sv, bore)
    d = _vsub(q, g[0])
    x = _vdot(d, g[1])
    y = _vdot(d, g[2])
    sq = _vdot(d, g[3])
    axis = math.hypot(math.hypot(x, y), sq - max(lo, min(hi, sq)))
    if axis - sv['c'] >= reach:
        return reach
    best = reach * reach
    st = _station_table(sv, bore)
    # First station at or above sq, by plain bisection.
    left = 0
    right = len(st)
    while left < right:
        mid = (left + right) // 2
        if st[mid][0] >= sq:
            right = mid
        else:
            left = mid + 1
    at = left
    i = at
    while i < len(st):
        goOn, best = _wall_visit(st[i], sq, x, y, best)
        if not goOn:
            break
        i += 1
    i = at - 1
    while i >= 0:
        goOn, best = _wall_visit(st[i], sq, x, y, best)
        if not goOn:
            break
        i -= 1
    return math.sqrt(best)


def _wall_reach(sv, bore, across, lean, highest):
    # 'wallCorners': the largest (highest=True) or least m = z + lean*t over
    # every corner of every piece of the bore's channel in the tube, at
    # stations every 0.001 mm from the span's lower end.
    g = sv['gears'][bore[0]]
    lo, hi = _bore_span(sv, bore)
    reach = -math.inf if highest else math.inf
    s = lo
    while s <= hi:
        for piece in _section_in_wall(sv, g, s):
            for corner in piece:
                pt = (g[0][0] + g[1][0] * corner[0] + g[2][0] * corner[1] + g[3][0] * s,
                      g[0][1] + g[1][1] * corner[0] + g[2][1] * corner[1] + g[3][1] * s,
                      g[0][2] + g[1][2] * corner[0] + g[2][2] * corner[1] + g[3][2] * s)
                m = pt[2] + lean * _vdot(pt, across)
                if highest:
                    reach = max(reach, m)
                else:
                    reach = min(reach, m)
        s += 0.001
    return reach


def _section_reach(sv, g, s):
    th = _gear_angle(g, s)
    c = math.cos(th)
    sn = math.sin(th)
    alongLo = math.inf
    sideLo = math.inf
    alongHi = -math.inf
    sideHi = -math.inf
    for q in _opening_corners(sv, g, _copysign1(s)):
        x = q[0] * c - q[1] * sn
        y = q[0] * sn + q[1] * c
        alongLo = min(alongLo, x)
        alongHi = max(alongHi, x)
        sideLo = min(sideLo, y)
        sideHi = max(sideHi, y)
    return alongLo, alongHi, sideLo, sideHi


def _in_wall(sv, g, s):
    _, _, lo, hi = _section_reach(sv, g, s)
    return abs(s) <= sv['ro'] and math.hypot(s, max(-lo, hi)) >= sv['ri']


def _channel_top(sv):
    # 'channelTop': the furthest any channel reaches from the middle plane
    # inside the wall, up or down.
    top = 0.0
    for g in sv['gears']:
        for sign in (-1.0, 1.0):
            s = sign * sv['sIn']
            while abs(s) <= sv['sOut']:
                if _in_wall(sv, g, s):
                    lo, hi, _, _ = _section_reach(sv, g, s)
                    for x in (lo, hi):
                        z = abs(g[0][2] + x * g[1][2])
                        if z > top:
                            top = z
                s += sign * 0.01
    return top


def _chord(sv, t):
    ri = sv['ri']
    ro = sv['ro']
    return math.sqrt(max(0.0, ri * ri - t * t)), math.sqrt(max(0.0, ro * ro - t * t))


def _window_at(w, t, z, a):
    across = w['across']
    facing = w['facing']
    return (across[0] * t + facing[0] * a,
            across[1] * t + facing[1] * a,
            across[2] * t + facing[2] * a + z)


def _window_near(sv, w, far, need, t, z, a):
    q = _window_at(w, t, z, a)
    for bore in far:
        if _wall_gap(sv, bore, q, need) < need:
            return True
    return False


def _window_clear(sv, w, far, need, direction, te):
    t = direction * te
    lean = w['lean']
    zLow = max(w['lo'] - lean * t, w['bottom'] + lean * t)
    zHigh = min(w['hi'] - lean * t, w['top'] + lean * t)
    if zLow > zHigh:
        return False
    a0, a1 = _chord(sv, t)
    for z in (zLow, zHigh):
        a = a0
        while True:
            if _window_near(sv, w, far, need, t, z, a):
                return False
            if a >= a1:
                break
            a = min(a + WINDOW_STEP_MM, a1)
    n = _ceil_int((zHigh - zLow) / WINDOW_STEP_MM)
    for a in (a0, a1):
        for k in range(1, n):
            if _window_near(sv, w, far, need, t, zLow + (zHigh - zLow) * k / n, a):
                return False
        z = -w['zLimit']
        while z <= w['zLimit']:
            if z > zLow and z < zHigh and _window_near(sv, w, far, need, t, z, a):
                return False
            z += WINDOW_STEP_MM
    return True


def _window_end(sv, w, far, direction):
    # 'windowEnd': bisect 24 times for the farthest clear end on one side.
    need = sv['collarWall'] + WINDOW_SLACK_MM
    lo = 0.0
    hi = min(sv['ri'], sv['ro'] / math.sqrt(2)) * (1 - 1e-9)
    for _ in range(24):
        m = (lo + hi) / 2
        if _window_clear(sv, w, far, need, direction, m):
            lo = m
        else:
            hi = m
    return lo


def _clip_corners(q, a, b, c):
    # Keep the part of a convex polygon of any size where a*t + b*z <= c.
    return _clip(q, a, b, c)


def _polygon_area(corners):
    a = 0.0
    n = len(corners)
    for i in range(n):
        p = corners[i]
        q = corners[(i + 1) % n]
        a += p[0] * q[1] - q[0] * p[1]
    return a / 2


def _new_window(sv, facing, label, zLimit):
    # 'newWindow': the window facing a level direction, found from the bores.
    w = {
        'facing': facing,
        'across': _vcross((0.0, 0.0, 1.0), facing),
        'label': label,
        'zLimit': zLimit,
    }
    flanks = []
    far = []
    for bore in _bores(sv):
        if _vdot(_bore_crossing(sv, bore), facing) > 0:
            flanks.append(bore)
        else:
            far.append(bore)
    if len(flanks) != 2:
        raise Exception(f'Screw gear: the window facing {label} has {len(flanks)} flanking '
                        f'bore(s) on its side of the tube, expected 2')
    low = flanks[0]
    high = flanks[1]
    if _bore_crossing(sv, low)[2] > _bore_crossing(sv, high)[2]:
        low, high = high, low
    lean = 1.0
    if _vdot(_bore_crossing(sv, high), w['across']) < _vdot(_bore_crossing(sv, low), w['across']):
        lean = -1.0
    w['lean'] = lean

    lowReach = _wall_reach(sv, low, w['across'], lean, True)
    highReach = _wall_reach(sv, high, w['across'], lean, False)
    w['lo'] = lowReach + math.sqrt(2) * sv['collarWall']
    w['hi'] = highReach - math.sqrt(2) * sv['collarWall']

    w['top'] = min(2 * zLimit - w['hi'], w['hi'] + math.sqrt(2) * sv['ri'])
    w['bottom'] = max(-2 * zLimit - w['lo'], w['lo'] - math.sqrt(2) * sv['ri'])

    w['left'] = -_window_end(sv, w, far, -1.0)
    w['right'] = _window_end(sv, w, far, 1.0)

    ro = sv['ro']
    q = [[-2 * ro, -2 * ro], [2 * ro, -2 * ro], [2 * ro, 2 * ro], [-2 * ro, 2 * ro]]
    halfPlanes = (
        (lean, 1.0, w['hi']), (-lean, -1.0, -w['lo']),
        (-lean, 1.0, w['top']), (lean, -1.0, -w['bottom']),
        (1.0, 0.0, w['right']), (-1.0, 0.0, -w['left']),
    )
    for h in halfPlanes:
        q = _clip_corners(q, h[0], h[1], h[2])
    # A sketch line cannot have zero length: drop a corner within 0.001 mm of
    # the one before it.
    corners = []
    for p in q:
        if len(corners) > 0:
            prev = corners[len(corners) - 1]
            if math.hypot(p[0] - prev[0], p[1] - prev[1]) <= 0.001:
                continue
        corners.append(p)
    while len(corners) > 1:
        first = corners[0]
        last = corners[len(corners) - 1]
        if math.hypot(first[0] - last[0], first[1] - last[1]) <= 0.001:
            corners.pop()
        else:
            break
    w['corners'] = corners
    return w


def _window_room(w):
    # 'room': why the window is not cut, or '' when it is.
    if w['hi'] <= w['lo']:
        return 'the flanking bores leave no band between them'
    if w['right'] <= w['left'] or len(w['corners']) < 3 or _polygon_area(w['corners']) <= 0:
        return 'the far bores leave the band no length'
    return ''


def _channel_outline(sv, bore):
    # 'channelOutline': the bore's rectangle sides at seventeen points each, at
    # stations 0.1 mm apart, kept within half a millimetre of the tube's radii.
    g = sv['gears'][bore[0]]
    sign = bore[1]
    hw, vLo, vHi = _bore_opening(sv, g, sign)
    out = []
    s = sign * sv['sIn']
    while abs(s) <= sv['sOut']:
        for i in range(17):
            k = i / 16
            v = vLo + (vHi - vLo) * k
            for q in ((hw, v), (-hw, v), (-hw + 2 * hw * k, vHi), (-hw + 2 * hw * k, vLo)):
                pt = _gear_world(g, q[0], q[1], s)
                r = math.hypot(pt[0], pt[1])
                if r < sv['ri'] - 0.5 or r > sv['ro'] + 0.5:
                    continue
                out.append(pt)
        s += sign * 0.1
    return out


def _hull_turn(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(points):
    # Andrew's monotone chain.
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    lower = []
    for p in pts:
        while len(lower) >= 2 and _hull_turn(lower[len(lower) - 2], lower[len(lower) - 1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and _hull_turn(upper[len(upper) - 2], upper[len(upper) - 1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:len(lower) - 1] + upper[:len(upper) - 1]


def _hull_separation(hp, hq):
    # The largest, over every edge of either hull, of the separation along the
    # edge's unit normal; negative when the hulls overlap.
    best = -math.inf
    for hull in (hp, hq):
        n = len(hull)
        for i in range(n):
            a = hull[i]
            b = hull[(i + 1) % n]
            ex = b[0] - a[0]
            ey = b[1] - a[1]
            length = math.hypot(ex, ey)
            if length <= 0:
                continue
            mx = -ey / length
            my = ex / length
            minP = math.inf
            maxP = -math.inf
            for p in hp:
                d = mx * p[0] + my * p[1]
                minP = min(minP, d)
                maxP = max(maxP, d)
            minQ = math.inf
            maxQ = -math.inf
            for q in hq:
                d = mx * q[0] + my * q[1]
                minQ = min(minQ, d)
                maxQ = max(maxQ, d)
            best = max(best, max(minQ - maxP, minP - maxQ))
    if best == -math.inf:
        # Both hulls are single points: their distance apart.
        best = math.hypot(hq[0][0] - hp[0][0], hq[0][1] - hp[0][1])
    return best


# The four gaps between neighbouring bores, by facing, with the indices of
# the two bores that flank each (bore index = gear*2 + (0 for -R, 1 for +R)).
_GAPS = (
    ((0.0, 1.0, 0.0), '+k', 1, 2),
    ((-1.0, 0.0, 0.0), '-e', 0, 2),
    ((0.0, -1.0, 0.0), '-k', 0, 3),
    ((1.0, 0.0, 0.0), '+e', 1, 3),
)


def _channel_separation(sv):
    # 'The wall between the bores': the least, over the four gaps, of the
    # separation between the two flanking bores' projected outlines.
    bores = _bores(sv)
    outlines = []
    for bore in bores:
        outlines.append(_channel_outline(sv, bore))
    least = math.inf
    leastNames = ''
    for gap in _GAPS:
        d = gap[0]
        across = _vcross((0.0, 0.0, 1.0), d)
        hulls = []
        for bi in (gap[2], gap[3]):
            projected = []
            for pt in outlines[bi]:
                projected.append((_vdot(pt, across), pt[2]))
            if len(projected) == 0:
                raise Exception(f'Screw gear: the {bores[bi][2]} bore has no outline in the wall '
                                f'to measure the {gap[1]} gap against')
            hulls.append(_convex_hull(projected))
        sep = _hull_separation(hulls[0], hulls[1])
        if sep < least:
            least = sep
            leastNames = f'{bores[gap[2]][2]} and {bores[gap[3]][2]} (gap facing {gap[1]})'
    return least, leastNames


# ---------------------------------------------------------------------------
# Dialog.

class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        # Where the mechanism goes comes first: nothing can be built without it,
        # and Fusion focuses the first selection input ([PB-AUTOFOCUS-FIRST]).
        planeInput = cls._addSelection(
            command, INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to",
            (adsk.core.SelectionCommandInput.ConstructionPlanes,
             adsk.core.SelectionCommandInput.PlanarFaces))
        pointInput = cls._addSelection(
            command, INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism',
            (adsk.core.SelectionCommandInput.ConstructionPoints,
             adsk.core.SelectionCommandInput.SketchPoints))
        parentInput = adsk.core.SelectionCommandInput.cast(cls._addSelection(
            command, INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under',
            (adsk.core.SelectionCommandInput.Occurrences,
             adsk.core.SelectionCommandInput.RootComponents)))
        rootComponent = get_design().rootComponent
        parentInput.addSelection(rootComponent)

        ribbon = command.commandInputs.addGroupCommandInput(GROUP_ID_RIBBON, 'Ribbon')
        ribbon.isExpanded = True
        cls._addValue(ribbon, INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', 15 / 10)
        cls._addValue(ribbon, INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', 68)
        cls._addValue(ribbon, INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', 49.5 / 10)
        cls._addValue(ribbon, INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', 3.75 / 10)
        cls._addValue(ribbon, INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', 2.625 / 10)
        cls._addValue(ribbon, INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', 2.625 / 10)
        cls._addValue(ribbon, INPUT_ID_TOOTH_SLANT, 'Tooth Slant', 'deg', math.radians(25.8))
        cls._addValue(ribbon, INPUT_ID_TOOTH_BOW, 'Tooth Bow', '', 0.048)

        frame = command.commandInputs.addGroupCommandInput(GROUP_ID_FRAME, 'Frame')
        frame.isExpanded = True
        cls._addValue(frame, INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', 15.5 / 10)
        cls._addValue(frame, INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', 18.75 / 10)
        cls._addValue(frame, INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.20 / 10)
        cls._addValue(frame, INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', 0.60 / 10)
        cls._addValue(frame, INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 3 / 10)
        cls._addValue(frame, INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 3 / 10)

        mesh = command.commandInputs.addGroupCommandInput(GROUP_ID_MESH, 'Mesh (from the mesh search)')
        mesh.isExpanded = False
        cls._addValue(mesh, INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', math.radians(80))
        cls._addValue(mesh, INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 1.15 / 10)
        cls._addValue(mesh, INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', math.radians(0))
        cls._addValue(mesh, INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', math.radians(0))
        cls._addValue(mesh, INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -1.31 / 10)

    @classmethod
    def _addSelection(cls, command: adsk.core.Command, id: str, label: str, tooltip: str,
                      filters) -> adsk.core.SelectionCommandInput:
        selectionInput = command.commandInputs.addSelectionInput(id, label, tooltip)
        for filterConstant in filters:
            selectionInput.addSelectionFilter(filterConstant)
        selectionInput.setSelectionLimits(1, 1)
        return selectionInput

    @classmethod
    def _addValue(cls, group: adsk.core.GroupCommandInput, id: str, label: str, unit: str,
                  value: float) -> adsk.core.ValueCommandInput:
        # Defaults in internal units ([PB-DIALOG-DEFAULT-UNITS]): mm/10 for a
        # length, radians for an angle, the bare number otherwise.
        initialValue = adsk.core.ValueInput.createByReal(value)
        return group.children.addValueInput(id, label, unit, initialValue)


# ---------------------------------------------------------------------------
# Generator.

class ScrewGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        # The same one-argument constructor as base.Generator. It restates
        # self.design so its Fusion type is visible in this module.
        super().__init__(design)
        self.design = design

    def prefixBase(self) -> str:
        return 'ScrewGear'

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        for index in range(2):
            self.buildGear(index)
        self.buildCage()
        self.relocateBodies()
        solids.hide_construction_geometry(self.designOcc.component)
        futil.log(PRINT_ORIENTATION_MESSAGE)

    # -- inputs -------------------------------------------------------------

    def _lookup(self, inputs: adsk.core.CommandInputs, id: str) -> adsk.core.CommandInput:
        item = inputs.itemById(id)
        if item is None:
            raise Exception(f'Screw gear: dialog input "{id}" was not found')
        return item

    def _readValue(self, inputs: adsk.core.CommandInputs, id: str, units: str) -> float:
        input = adsk.core.ValueCommandInput.cast(self._lookup(inputs, id))
        if input is None:
            raise Exception(f'Screw gear: dialog input "{id}" is not a value input')
        return self.design.unitsManager.evaluateExpression(input.expression, units)

    def _readSelection(self, inputs: adsk.core.CommandInputs, id: str) -> adsk.core.Base:
        selectionInput = adsk.core.SelectionCommandInput.cast(self._lookup(inputs, id))
        if selectionInput is None:
            raise Exception(f'Screw gear: dialog input "{id}" is not a selection input')
        if selectionInput.selectionCount < 1:
            raise Exception(f'Screw gear: nothing is selected for "{id}"')
        return selectionInput.selection(0).entity

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Selections first, before any occurrence is created ([PB-SELECTION-STASH]).
        self.targetPlane = self._readSelection(inputs, INPUT_ID_PLANE)
        self.centrePoint = self._readSelection(inputs, INPUT_ID_POINT)
        parentEntity = self._readSelection(inputs, INPUT_ID_PARENT)
        parentOcc = adsk.fusion.Occurrence.cast(parentEntity)
        if parentOcc is not None:
            self.parentComponent = parentOcc.component
        else:
            parentComp = adsk.fusion.Component.cast(parentEntity)
            if parentComp is None:
                raise Exception('Screw gear: the Parent Component selection is neither an '
                                'occurrence nor a component')
            self.parentComponent = parentComp

        # Raw values, internal units (cm, radians, bare numbers) ([PB-EVAL-EXPRESSION]).
        W = self._readValue(inputs, INPUT_ID_RIBBON_WIDTH, 'mm')
        toothCount = self._readValue(inputs, INPUT_ID_TOOTH_COUNT, '')
        lead = self._readValue(inputs, INPUT_ID_TWIST_LEAD, 'mm')
        T = self._readValue(inputs, INPUT_ID_RIBBON_THICKNESS, 'mm')
        P = self._readValue(inputs, INPUT_ID_TOOTH_PITCH, 'mm')
        H = self._readValue(inputs, INPUT_ID_TOOTH_HEIGHT, 'mm')
        slant = self._readValue(inputs, INPUT_ID_TOOTH_SLANT, 'deg')
        bow = self._readValue(inputs, INPUT_ID_TOOTH_BOW, '')
        cageRadius = self._readValue(inputs, INPUT_ID_CAGE_RADIUS, 'mm')
        cageRise = self._readValue(inputs, INPUT_ID_CAGE_RISE, 'mm')
        clearance = self._readValue(inputs, INPUT_ID_CLEARANCE, 'mm')
        roof = self._readValue(inputs, INPUT_ID_ROOF_ALLOWANCE, 'mm')
        collarHalf = self._readValue(inputs, INPUT_ID_COLLAR_HALF, 'mm')
        collarWall = self._readValue(inputs, INPUT_ID_COLLAR_WALL, 'mm')
        Sigma = self._readValue(inputs, INPUT_ID_CROSS_ANGLE, 'deg')
        engagement = self._readValue(inputs, INPUT_ID_ENGAGEMENT, 'mm')
        mountA = self._readValue(inputs, INPUT_ID_MOUNT_ANGLE_A, 'deg')
        mountB = self._readValue(inputs, INPUT_ID_MOUNT_ANGLE_B, 'deg')
        phase = self._readValue(inputs, INPUT_ID_ASSEMBLY_PHASE, 'mm')

        # Range checks, each naming the field and the bound.
        positives = (
            (INPUT_ID_RIBBON_WIDTH, W), (INPUT_ID_RIBBON_THICKNESS, T),
            (INPUT_ID_TOOTH_PITCH, P), (INPUT_ID_TWIST_LEAD, lead),
            (INPUT_ID_COLLAR_HALF, collarHalf), (INPUT_ID_COLLAR_WALL, collarWall),
            (INPUT_ID_CLEARANCE, clearance),
        )
        for name, value in positives:
            if not value > 0:
                raise Exception(f'Screw gear: {name} must be > 0 (got {value * 10:.4g} mm)')
        if not roof >= 0:
            raise Exception(f'Screw gear: {INPUT_ID_ROOF_ALLOWANCE} must be >= 0 '
                            f'(got {roof * 10:.4g} mm)')
        if abs(toothCount - round(toothCount)) > 1e-9 or round(toothCount) < 4:
            raise Exception(f'Screw gear: {INPUT_ID_TOOTH_COUNT} must be a whole number >= 4 '
                            f'(got {toothCount:.6g})')
        N = int(round(toothCount))
        if not (H > 0 and H < W / 2):
            raise Exception(f'Screw gear: {INPUT_ID_TOOTH_HEIGHT} must be > 0 and < '
                            f'{INPUT_ID_RIBBON_WIDTH}/2 = {W * 10 / 2:.4g} mm '
                            f'(got {H * 10:.4g} mm)')
        slantDeg = math.degrees(slant)
        if not (slantDeg > -90 and slantDeg < 90):
            raise Exception(f'Screw gear: {INPUT_ID_TOOTH_SLANT} must lie strictly between '
                            f'-90 and 90 deg (got {slantDeg:.4g} deg)')
        # toothBow is in mm^-1: work its root depth in mm.
        if not bow >= 0:
            raise Exception(f'Screw gear: {INPUT_ID_TOOTH_BOW} must be >= 0 (got {bow:.4g})')
        bowDepth = H * 10 + bow * (T * 10 / 2) ** 2
        if not bowDepth < W * 10 / 2:
            raise Exception(f'Screw gear: {INPUT_ID_TOOTH_BOW} makes '
                            f'{INPUT_ID_TOOTH_HEIGHT} + {INPUT_ID_TOOTH_BOW}*'
                            f'({INPUT_ID_RIBBON_THICKNESS}/2)^2 = {bowDepth:.4g} mm, which must '
                            f'be < {INPUT_ID_RIBBON_WIDTH}/2 = {W * 10 / 2:.4g} mm')
        if not (engagement > 0 and engagement <= H):
            raise Exception(f'Screw gear: {INPUT_ID_ENGAGEMENT} must be > 0 and <= '
                            f'{INPUT_ID_TOOTH_HEIGHT} = {H * 10:.4g} mm '
                            f'(got {engagement * 10:.4g} mm)')
        sigmaDeg = math.degrees(Sigma)
        if not (sigmaDeg > 0 and sigmaDeg < 180):
            raise Exception(f'Screw gear: {INPUT_ID_CROSS_ANGLE} must lie strictly between '
                            f'0 and 180 deg (got {sigmaDeg:.4g} deg)')
        if not abs(phase) < P:
            raise Exception(f'Screw gear: {INPUT_ID_ASSEMBLY_PHASE} must lie strictly within '
                            f'+/-{INPUT_ID_TOOTH_PITCH} = +/-{P * 10:.4g} mm '
                            f'(got {phase * 10:.4g} mm)')
        if not (cageRadius + collarHalf + 0.1 < N * P / 2):
            raise Exception(f'Screw gear: {INPUT_ID_CAGE_RADIUS} + {INPUT_ID_COLLAR_HALF} + 1 mm '
                            f'= {(cageRadius + collarHalf) * 10 + 1:.4g} mm must be < '
                            f'{INPUT_ID_TOOTH_COUNT}*{INPUT_ID_TOOTH_PITCH}/2 = '
                            f'{N * P * 10 / 2:.4g} mm, so both bores lie within the ribbon')

        # The build's values, in cm and radians.
        self.W = W
        self.T = T
        self.H = H
        self.P = P
        self.N = N
        self.L = N * P
        self.lam = lead / (2 * math.pi)
        self.Sigma = Sigma
        self.A = W - engagement
        self.slantTan = math.tan(slant)
        # The bow lowers the edge by 10*toothBow*v^2 cm for v in cm.
        self.bow = 10 * bow
        self.cageRadius = cageRadius
        self.cageRise = cageRise
        self.clearance = clearance
        self.roof = roof
        self.collarHalf = collarHalf
        self.collarWall = collarWall
        self.mounts = (mountA, mountB)
        self.phases = (0.0, phase)
        self.hw = W / 2 + clearance
        self.ht = T / 2 + clearance
        self.Ri = cageRadius - collarHalf
        self.Ro = cageRadius + collarHalf
        # Steps to the tooth: twist under 2 degrees between sections, at least eight.
        self.cellSteps = max(_ceil_int((P / self.lam) / math.radians(2)), 8)

        # The sleeve's four checks and the window search, in millimetres.
        Wmm = W * 10
        Tmm = T * 10
        Amm = self.A * 10
        Rimm = self.Ri * 10
        c = math.hypot(Wmm / 2 + clearance * 10, Tmm / 2 + clearance * 10 + roof * 10)

        if not math.hypot(c, 1.0) < Rimm:
            raise Exception(f'Screw gear: {INPUT_ID_CAGE_RADIUS} leaves the sleeve an inner radius of '
                            f'{Rimm:.4g} mm, which must exceed hypot(bore corner {c:.4g} mm, 1 mm) = '
                            f'{math.hypot(c, 1.0):.4g} mm so each channel starts in the hollow')
        sInMm = math.sqrt(Rimm * Rimm - c * c) - 1.0
        if not sInMm > 0:
            raise Exception(f'Screw gear: {INPUT_ID_CAGE_RADIUS} puts the channel start at '
                            f'{sInMm:.4g} mm, which must be > 0')
        sOutMm = (cageRadius + collarHalf) * 10 + 1.0

        axialWindow = 1.5 * math.sqrt(Wmm * Wmm - Amm * Amm) / math.sin(Sigma)
        meshReach = math.hypot(axialWindow, math.hypot(Wmm / 2, Tmm / 2)) + clearance * 10
        if not meshReach <= Rimm:
            raise Exception(f'Screw gear: {INPUT_ID_CAGE_RADIUS} leaves the sleeve an inner radius of '
                            f'{Rimm:.4g} mm, which must be at least {meshReach:.4g} mm so the mesh '
                            f'stays visible along the axis')

        riseBound = Amm / 2 + c + collarWall * 10
        if not cageRise * 10 >= riseBound:
            raise Exception(f'Screw gear: {INPUT_ID_CAGE_RISE} = {cageRise * 10:.4g} mm must be '
                            f'>= {riseBound:.4g} mm so the end faces keep {INPUT_ID_COLLAR_WALL}')

        half = Sigma / 2
        dirA = (math.cos(half), math.sin(half), 0.0)
        dirB = (math.cos(half), -math.sin(half), 0.0)
        uA = (0.0, 0.0, 1.0)
        uB = (0.0, 0.0, -1.0)
        lamMm = lead * 10 / (2 * math.pi)
        gearA = [(0.0, 0.0, -Amm / 2), uA, _vcross(dirA, uA), dirA, mountA, lamMm]
        gearB = [(0.0, 0.0, Amm / 2), uB, _vcross(dirB, uB), dirB, mountB, lamMm]
        sv = {
            'gears': [gearA, gearB],
            'cageRadius': cageRadius * 10,
            'hw': Wmm / 2 + clearance * 10,
            'ht': Tmm / 2 + clearance * 10,
            'roof': roof * 10,
            'ri': Rimm,
            'ro': self.Ro * 10,
            'zb': cageRise * 10,
            'sIn': sInMm,
            'sOut': sOutMm,
            'c': c,
            'collarWall': collarWall * 10,
            'sections': [None, None, None, None],
        }

        separation, between = _channel_separation(sv)
        if not separation >= collarWall * 10:
            raise Exception(f'Screw gear: {INPUT_ID_COLLAR_WALL} = {collarWall * 10:.4g} mm is more '
                            f'than the wall between the bores {between}, whose separation is '
                            f'{separation:.4g} mm')
        futil.log(f'Screw gear: the wall between the bores is {separation:.3f} mm at its least, '
                  f'between {between}')

        self.sIn = sInMm / 10
        self.sOut = sOutMm / 10

        # The window search. It refuses nothing: a gap with no room gets no window.
        if sigmaDeg <= 90:
            facings = (((0.0, 1.0, 0.0), '+k'), ((0.0, -1.0, 0.0), '-k'))
        else:
            facings = (((1.0, 0.0, 0.0), '+e'), ((-1.0, 0.0, 0.0), '-e'))
        zLimit = _channel_top(sv)
        self.windows = []
        for facing, label in facings:
            w = _new_window(sv, facing, label, zLimit)
            reason = _window_room(w)
            if reason != '':
                futil.log(f'No window facing {label}: {reason}')
                continue
            # The window hands on its facing and corners; lengths stay in mm
            # here and are divided by ten where the build uses them.
            self.windows.append({
                'facing': facing,
                'across': w['across'],
                'label': label,
                'corners': w['corners'],
            })
            futil.log(f'Screw gear: window facing {label} has {len(w["corners"])} corners, '
                      f'lo {w["lo"]:.3f} hi {w["hi"]:.3f} bottom {w["bottom"]:.3f} '
                      f'top {w["top"]:.3f} left {w["left"]:.3f} right {w["right"]:.3f} mm')

    # -- component tree -----------------------------------------------------

    def _addChild(self, topComponent: adsk.fusion.Component, name: str) -> adsk.fusion.Occurrence:
        identity = adsk.core.Matrix3D.create()
        occurrence = topComponent.occurrences.addNewComponent(identity)
        occurrence.component.name = name
        return occurrence

    def buildComponentTree(self):
        # The top occurrence is the inherited getOccurrence(), created under
        # the selected parent; deleteComponent() removes it on failure
        # ([PB-OCCURRENCE-TREE]). Nothing is activated ([PB-NEVER-ACTIVATE]).
        top = self.getOccurrence()
        topComponent = top.component
        topComponent.name = 'Screw Gearing'
        self.designOcc = self._addChild(topComponent, 'Design')
        self.gearOccs = [self._addChild(topComponent, 'Gear A'),
                         self._addChild(topComponent, 'Gear B')]
        self.cageOcc = self._addChild(topComponent, 'Cage')
        self.designComponent = self.designOcc.component
        self.gearBodies = [adsk.fusion.BRepBody.cast(None)] * 2
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]

    # -- small helpers ------------------------------------------------------

    def _point(self, p) -> adsk.core.Point3D:
        return adsk.core.Point3D.create(p[0], p[1], p[2])

    def _local(self, sketch: adsk.fusion.Sketch, p, zeroZ: bool) -> adsk.core.Point3D:
        # A world point (cm tuple) in the sketch's space; its z is set to 0
        # when it is meant to lie on the sketch's plane ([PB-SKETCH-ZERO-Z]).
        worldPoint = self._point(p)
        local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
        if zeroZ:
            local.z = 0
        return local

    def _requireFullyConstrained(self, sketch: adsk.fusion.Sketch):
        if not sketch.isFullyConstrained:
            raise Exception(f'Screw gear: sketch "{sketch.name}" is not fully constrained '
                            f'([PB-FULL-CONSTRAINT])')

    def _frameWorld(self, x: float, y: float, z: float):
        # A point of the frame of §1 given in mm (x on ê, y on k̂, z on n̂), as
        # a world point in cm.
        e = self.eHat
        k = self.kHat
        n = self.nHat
        return (self.C[0] + (x * e[0] + y * k[0] + z * n[0]) / 10,
                self.C[1] + (x * e[1] + y * k[1] + z * n[1]) / 10,
                self.C[2] + (x * e[2] + y * k[2] + z * n[2]) / 10)

    def _frameVector(self, v):
        # A direction of the frame of §1 (on ê, k̂, n̂) as a world direction.
        e = self.eHat
        k = self.kHat
        n = self.nHat
        return (v[0] * e[0] + v[1] * k[0] + v[2] * n[0],
                v[0] * e[1] + v[1] * k[1] + v[2] * n[1],
                v[0] * e[2] + v[1] * k[2] + v[2] * n[2])

    def _theta(self, index: int, s: float) -> float:
        return s / self.lam + self.mounts[index]

    def _gearPoint(self, index: int, u: float, v: float, s: float):
        # origin_g + s*dir_g + (u cos θ - v sin θ) û_g + (u sin θ + v cos θ) v̂_g, in cm.
        th = self._theta(index, s)
        c = math.cos(th)
        sn = math.sin(th)
        x = u * c - v * sn
        y = u * sn + v * c
        o = self.gearOrigin[index]
        uu = self.gearU[index]
        vv = self.gearV[index]
        d = self.gearDir[index]
        return (o[0] + uu[0] * x + vv[0] * y + d[0] * s,
                o[1] + uu[1] * x + vv[1] * y + d[1] * s,
                o[2] + uu[2] * x + vv[2] * y + d[2] * s)

    def _roofSign(self, index: int, sigma: float) -> int:
        # +1 when the bore's +v face is its roof with the sleeve on its -n end.
        th = self._theta(index, sigma * self.cageRadius)
        if -math.sin(th) * _vdot(self.gearU[index], self.nHat) > 0:
            return 1
        return -1

    def _boreLimits(self, index: int, sigma: float):
        # The bore's v span: the roof face moves out by the roof allowance.
        if self._roofSign(index, sigma) > 0:
            return -self.ht, self.ht + self.roof
        return -self.ht - self.roof, self.ht

    # -- §1: anchor and frame -----------------------------------------------

    def buildAnchor(self):
        design = self.designComponent
        targetPlane = self.targetPlane
        selectedPoint = self.centrePoint

        sketch = design.sketches.add(targetPlane)
        sketch.name = 'Anchor'
        # The one projection in the build ([SCREW-F-REFERENCES]).
        projected = sketch.project(selectedPoint)
        if projected is None or projected.count < 1:
            raise Exception('Screw gear: projecting the Centre Point into the Anchor sketch '
                            'produced nothing')
        projectedPoint = adsk.fusion.SketchPoint.cast(projected.item(0))
        if projectedPoint is None:
            raise Exception('Screw gear: the projected Centre Point is not a sketch point')
        seed = projectedPoint.geometry
        startSeed = adsk.core.Point3D.create(seed.x - 0.5, seed.y, 0)
        endSeed = adsk.core.Point3D.create(seed.x + 0.5, seed.y, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)
        sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)
        sketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint = adsk.core.Point3D.create(seed.x, seed.y + 0.3, 0)
        lengthDim = sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        lengthDim.parameter.value = 1.0
        self._requireFullyConstrained(sketch)
        self.anchorLine = anchorLine

        # The frame, from world geometry of the constrained sketch
        # ([PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]).
        centre = projectedPoint.worldGeometry
        self.C = (centre.x, centre.y, centre.z)
        startWorld = anchorLine.startSketchPoint.worldGeometry
        endWorld = anchorLine.endSketchPoint.worldGeometry
        self.eHat = _vunit((endWorld.x - startWorld.x, endWorld.y - startWorld.y,
                            endWorld.z - startWorld.z), 'the Anchor Line')

        # The two axis planes ([PB-USE-SELECTED-PLANE], [PB-CONSTRUCTION-PLANES]).
        self.axisPlanes = [self._offsetPlane(targetPlane, -self.A / 2, 'Gear A Axis Plane'),
                           self._offsetPlane(targetPlane, self.A / 2, 'Gear B Axis Plane')]

        # n̂ is Gear A's plane normal, signed so C stands A/2 above it
        # ([SCREW-F-NORMAL-SIGN]).
        planeA: adsk.fusion.ConstructionPlane = self.axisPlanes[0]
        planeB: adsk.fusion.ConstructionPlane = self.axisPlanes[1]
        geomA = planeA.geometry
        normalA = _vunit((geomA.normal.x, geomA.normal.y, geomA.normal.z), 'Gear A Axis Plane normal')
        originA = (geomA.origin.x, geomA.origin.y, geomA.origin.z)
        distA = _vdot(_vsub(self.C, originA), normalA)
        if distA < 0:
            normalA = _vscale(normalA, -1.0)
            distA = -distA
        self.nHat = normalA
        geomB = planeB.geometry
        originB = (geomB.origin.x, geomB.origin.y, geomB.origin.z)
        distB = _vdot(_vsub(self.C, originB), self.nHat)
        tolerance = 1e-5
        if abs(distA - self.A / 2) > tolerance or abs(distB + self.A / 2) > tolerance:
            raise Exception(f'Screw gear: the axis planes stand {distA * 10:.6f} mm below and '
                            f'{-distB * 10:.6f} mm above the centre, expected '
                            f'{self.A * 10 / 2:.6f} mm each')
        self.kHat = _vcross(self.nHat, self.eHat)

        # The two gears' frames: û points at the other gear.
        half = self.Sigma / 2
        e = self.eHat
        k = self.kHat
        dirA = _vadd(_vscale(e, math.cos(half)), _vscale(k, math.sin(half)))
        dirB = _vadd(_vscale(e, math.cos(half)), _vscale(k, -math.sin(half)))
        uA = self.nHat
        uB = _vscale(self.nHat, -1.0)
        self.gearDir = [dirA, dirB]
        self.gearU = [uA, uB]
        self.gearV = [_vcross(dirA, uA), _vcross(dirB, uB)]
        self.gearOrigin = [_vsub(self.C, _vscale(self.nHat, self.A / 2)),
                           _vadd(self.C, _vscale(self.nHat, self.A / 2))]

    def _offsetPlane(self, targetPlane: adsk.core.Base, offset: float,
                     name: str) -> adsk.fusion.ConstructionPlane:
        design = self.designComponent
        planeInput = design.constructionPlanes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(offset)
        planeInput.setByOffset(targetPlane, offsetValue)
        plane = design.constructionPlanes.add(planeInput)
        plane.name = name
        return plane

    # -- per gear -----------------------------------------------------------

    def buildGear(self, index: int):
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, index: int):
        # The Paths sketch: two lines on the gear's axis, 'bore-' over
        # [-sOut, -sIn] and 'bore+' over [sIn, sOut], between four fixed
        # reference points ([PB-PATH-FROM-SKETCH], [PB-WORLDGEO-CONSTRAINED]).
        design = self.designComponent
        label = GEAR_LABELS[index]
        plane: adsk.fusion.ConstructionPlane = self.axisPlanes[index]
        sketch = design.sketches.add(plane)
        sketch.name = f'{label} Paths'
        points = []
        for s in (-self.sOut, -self.sIn, self.sIn, self.sOut):
            world = _vadd(self.gearOrigin[index], _vscale(self.gearDir[index], s))
            worldPoint = adsk.core.Point3D.create(world[0], world[1], world[2])
            local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            local.z = 0
            points.append(sketch.sketchPoints.add(local))
        p0: adsk.fusion.SketchPoint = points[0]
        p1: adsk.fusion.SketchPoint = points[1]
        p2: adsk.fusion.SketchPoint = points[2]
        p3: adsk.fusion.SketchPoint = points[3]
        lineMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
        linePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(p2, p3)
        p0.isFixed = True
        p1.isFixed = True
        p2.isFixed = True
        p3.isFixed = True
        self._requireFullyConstrained(sketch)
        self.pathLines[index] = {'bore-': lineMinus, 'bore+': linePlus}

    def _utooth(self, index: int, v: float, s: float, slantTan: float) -> float:
        # The toothed edge's u at (v, s), in cm.
        H = self.H
        return (self.W / 2 - H / 2
                + (H / 2) * math.cos(2 * math.pi * (s + slantTan * v - self.phases[index]) / self.P)
                - self.bow * v * v)

    def buildToothCell(self, index: int):
        # §2: one cell of c teeth at the ribbon's negative end, lofted through
        # c*n + 1 sections of one Cell Sections sketch.
        label = GEAR_LABELS[index]
        teeth = min(CELL_TEETH, self.N)
        s0 = self.phases[index] - self.L / 2
        self.cellBody = self._loftCell(index, f'{label} Cell Sections', s0, teeth)

    def _loftCell(self, index: int, name: str, start: float, teeth: int) -> adsk.fusion.BRepBody:
        design = self.designComponent
        plane: adsk.fusion.ConstructionPlane = self.axisPlanes[index]
        n = self.cellSteps
        M = TOOTH_SPLINE_POINTS
        sectionCount = teeth * n + 1

        sketch = design.sketches.add(plane)
        sketch.name = name
        sketch.isComputeDeferred = True

        addedPoints = []
        splines = []
        sections = []
        uB = -self.W / 2
        hv = self.T / 2
        for k in range(sectionCount):
            s = start + k * self.P / n
            # Every point keeps its mapped z: the sections lie off the plane
            # on purpose ([PB-3D-SKETCH-SECTIONS]).
            b0 = sketch.sketchPoints.add(self._local(sketch, self._gearPoint(index, uB, -hv, s), False))
            b1 = sketch.sketchPoints.add(self._local(sketch, self._gearPoint(index, uB, hv, s), False))
            addedPoints.append(b0)
            addedPoints.append(b1)
            toothed = []
            for j in range(M):
                v = -hv + j * self.T / (M - 1)
                u = self._utooth(index, v, s, self.slantTan)
                worldPoint = self._point(self._gearPoint(index, u, v, s))
                local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
                sectionPoint = sketch.sketchPoints.add(local)
                toothed.append(sectionPoint)
                addedPoints.append(sectionPoint)
            f0: adsk.fusion.SketchPoint = toothed[0]
            fLast: adsk.fusion.SketchPoint = toothed[M - 1]

            L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(b0, f0)
            fitPoints = adsk.core.ObjectCollection.create()
            for sectionPoint in toothed:
                fitPoints.add(sectionPoint)
            S = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
            if S is None or S.fitPoints.count != M:
                got = 'none' if S is None else str(S.fitPoints.count)
                raise Exception(f'Screw gear: {name} section {k} spline has {got} fit points, '
                                f'expected {M}')
            L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(fLast, b1)
            L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(b1, b0)
            splines.append(S)
            sections.append([L1, S, L3, L4])

        # Every point the build added and every spline fit point is fixed
        # once the last curve is drawn.
        for point in addedPoints:
            sketchPoint: adsk.fusion.SketchPoint = point
            sketchPoint.isFixed = True
        for item in splines:
            spline: adsk.fusion.SketchFittedSpline = item
            for i in range(spline.fitPoints.count):
                fitPoint = spline.fitPoints.item(i)
                fitPoint.isFixed = True
        sketch.isComputeDeferred = False
        self._requireFullyConstrained(sketch)
        if sketch.profiles.count != sectionCount:
            raise Exception(f'Screw gear: {name} has {sketch.profiles.count} profiles, expected '
                            f'{sectionCount}, one per section')
        for k in range(len(sections)):
            if len(sections[k]) != 4:
                raise Exception(f'Screw gear: {name} section {k} holds {len(sections[k])} curves, '
                                f'expected 4')

        # Loft the sections in station order, each as one four-curve path
        # ([PB-LOFT], [PB-PATH-FROM-SKETCH]).
        loftInput = design.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for section in sections:
            curves = adsk.core.ObjectCollection.create()
            for sectionCurve in section:
                curves.add(sectionCurve)
            sectionPath = design.features.createPath(curves, False)
            loftInput.loftSections.add(sectionPath)
        loftFeature = design.features.loftFeatures.add(loftInput)
        if loftFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the loft of {name} left {loftFeature.bodies.count} '
                            f'bodies, expected 1')
        cellBody = loftFeature.bodies.item(0)
        if not cellBody.isSolid:
            raise Exception(f'Screw gear: the loft of {name} is not a solid body')

        self._checkSlant(index, name, cellBody, start)
        return cellBody

    def _checkSlant(self, index: int, name: str, cellBody: adsk.fusion.BRepBody, start: float):
        # Probe the cell on and off a ridge to tell the slant's sign
        # ([PB-SELF-DIAGNOSING]).
        P = self.P
        Z0 = self.phases[index]
        sc = Z0 + P * _ceil_int((start + P / 2 - Z0) / P)
        inset = 0.025
        used = 0
        for sigma in (-1.0, 1.0):
            vp = sigma * (self.T / 2 - inset)
            up = self.W / 2 - self.bow * vp * vp - inset
            for kind, station in (('on-ridge', sc - self.slantTan * vp),
                                  ('off-ridge', sc + self.slantTan * vp)):
                m = self._utooth(index, vp, station, self.slantTan) - up
                mWrong = self._utooth(index, vp, station, -self.slantTan) - up
                if abs(m) < 0.01 or abs(mWrong) < 0.01 or (m > 0) == (mWrong > 0):
                    continue
                used += 1
                probePoint = self._point(self._gearPoint(index, up, vp, station))
                reading = cellBody.pointContainment(probePoint)
                if m > 0:
                    expected = adsk.fusion.PointContainment.PointInsidePointContainment
                else:
                    expected = adsk.fusion.PointContainment.PointOutsidePointContainment
                if reading != expected:
                    raise Exception(
                        f'Screw gear: {GEAR_LABELS[index]} {name}: the {kind} probe on the '
                        f'{"+" if sigma > 0 else "-"}v face at station {station * 10:.3f} mm read '
                        f'containment {reading}, expected {expected}; the tooth slant leans the '
                        f'wrong way')
        if used == 0:
            futil.log(f'Screw gear: {GEAR_LABELS[index]} {name}: the tooth slant\'s sign was not '
                      f'checked, no probe tells the two signs apart')

    # -- §3: repeat the cell by doubling --------------------------------------

    def _copyBody(self, sourceBody: adsk.fusion.BRepBody, what: str) -> adsk.fusion.BRepBody:
        design = self.designComponent
        copyFeature = design.features.copyPasteBodies.add(sourceBody)
        if copyFeature is None or copyFeature.bodies.count != 1:
            got = 0 if copyFeature is None else copyFeature.bodies.count
            raise Exception(f'Screw gear: copying {what} gave {got} bodies, expected 1')
        return copyFeature.bodies.item(0)

    def _screwMove(self, index: int, copyBody: adsk.fusion.BRepBody, k: int):
        # Step(k) = translate k*P along the axis after rotating k*P/Λ about it
        # ([SCREW-F-SCREW-STEP], [PB-MOVE-ROTATE]). k >= 1, so never zero.
        design = self.designComponent
        pitch = self.P
        lam = self.lam
        o = self.gearOrigin[index]
        d = self.gearDir[index]
        axisPoint = adsk.core.Point3D.create(o[0], o[1], o[2])
        axisVector = adsk.core.Vector3D.create(d[0], d[1], d[2])
        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * pitch / lam, axisVector, axisPoint)
        mov = adsk.core.Matrix3D.create()
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k * pitch)
        mov.translation = shift
        rot.transformBy(mov)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(copyBody)
        moveInput = design.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        design.features.moveFeatures.add(moveInput)

    def _join(self, targetBody: adsk.fusion.BRepBody, toolBody: adsk.fusion.BRepBody,
              what: str) -> adsk.fusion.BRepBody:
        design = self.designComponent
        tools = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput = design.features.combineFeatures.createInput(targetBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature = design.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise Exception(f'Screw gear: joining {what} left {combineFeature.bodies.count} '
                            f'bodies, expected 1')
        return combineFeature.bodies.item(0)

    def repeatCellByDoubling(self, index: int):
        label = GEAR_LABELS[index]
        teeth = min(CELL_TEETH, self.N)
        q = self.N // teeth
        r = self.N % teeth
        body: adsk.fusion.BRepBody = self.cellBody
        m = 1
        if q > 1:
            top = 0
            while (q >> (top + 1)) > 0:
                top += 1
            asides = []
            for bit in range(top):
                if (q >> bit) & 1:
                    asides.append([self._copyBody(body, f'{label} aside of {m} cell(s)'), m])
                copyBody = self._copyBody(body, f'{label} body of {m} cell(s)')
                self._screwMove(index, copyBody, m * teeth)
                body = self._join(body, copyBody, f'{label} doubling to {2 * m} cells')
                m = 2 * m
            for aside in reversed(asides):
                asideBody: adsk.fusion.BRepBody = aside[0]
                asideCells = aside[1]
                self._screwMove(index, asideBody, m * teeth)
                body = self._join(body, asideBody, f'{label} aside of {asideCells} cell(s)')
                m += asideCells
        if m != q:
            raise Exception(f'Screw gear: {label} doubling reached {m} cells, expected {q}')
        if r > 0:
            # The remainder is built in place: r teeth from s0 + q*c*P.
            start = self.phases[index] - self.L / 2 + q * teeth * self.P
            remainder = self._loftCell(index, f'{label} Cell Remainder', start, r)
            body = self._join(body, remainder, f'{label} remainder of {r} teeth')
        body.name = label
        self.gearBodies[index] = body

    # -- §4: the cage --------------------------------------------------------

    def buildCage(self):
        self._buildSleeve()
        # One twisted sweep cut per bore, in the order A -R, A +R, B -R, B +R.
        for index in range(2):
            for sigma in (-1.0, 1.0):
                self._cutBore(index, sigma)
        self._cutWindows()
        self._cutMarkers()
        futil.log(PRINT_ORIENTATION_MESSAGE)

    def _buildSleeve(self):
        design = self.designComponent
        plane = self.targetPlane
        sketch = design.sketches.add(plane)
        sketch.name = 'Sleeve'
        for radius in (self.Ri, self.Ro):
            centre = self._local(sketch, self.C, True)
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            # An off-centre text point, on the circle ([PB-RADIAL-DIM]).
            textPoint = adsk.core.Point3D.create(centre.x + radius, centre.y, 0)
            diameter = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameter.parameter.value = 2 * radius
        self._requireFullyConstrained(sketch)
        ringProfile = adsk.fusion.Profile.cast(None)
        rings = 0
        for i in range(sketch.profiles.count):
            candidate = sketch.profiles.item(i)
            if candidate.profileLoops.count == 2:
                ringProfile = candidate
                rings += 1
        if rings != 1:
            raise Exception(f'Screw gear: the Sleeve sketch has {rings} two-loop profiles among '
                            f'{sketch.profiles.count}, expected exactly 1 annulus')

        extrudeInput = design.features.extrudeFeatures.createInput(
            ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        riseValue = adsk.core.ValueInput.createByReal(self.cageRise)
        extrudeInput.setSymmetricExtent(riseValue, False)
        extrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the sleeve extrude left {extrudeFeature.bodies.count} '
                            f'bodies, expected 1')
        self.cageBody = extrudeFeature.bodies.item(0)

    def _cutBore(self, index: int, sigma: float):
        design = self.designComponent
        label = GEAR_LABELS[index]
        tag = '-R' if sigma < 0 else '+R'
        boreName = f'{label} Bore {tag}'
        lineKey = 'bore-' if sigma < 0 else 'bore+'
        boreLine: adsk.fusion.SketchLine = self.pathLines[index][lineKey]

        # The bore's own plane, at fraction 0 of its path line, passed directly
        # ([PB-CONSTRUCTION-PLANES]).
        planeInput = design.constructionPlanes.createInput()
        startFraction = adsk.core.ValueInput.createByReal(0)
        planeInput.setByDistanceOnPath(boreLine, startFraction)
        plane = design.constructionPlanes.add(planeInput)
        plane.name = f'{boreName} Plane'

        station = -self.sOut if sigma < 0 else self.sIn
        theta = self._theta(index, station)
        uB = -self.hw
        uF = self.hw
        vLo, vHi = self._boreLimits(index, sigma)

        sketch = design.sketches.add(plane)
        sketch.name = boreName
        sketch.isComputeDeferred = True

        # References: O where the path pierces the plane, Cp A/2 along the
        # unrotated û. Every mapped point lies on the plane: z = 0.
        oWorld = _vadd(self.gearOrigin[index], _vscale(self.gearDir[index], station))
        cpWorld = _vadd(oWorld, _vscale(self.gearU[index], self.A / 2))
        oLocal = self._local(sketch, oWorld, True)
        cpLocal = self._local(sketch, cpWorld, True)
        O = sketch.sketchPoints.add(oLocal)
        Cp = sketch.sketchPoints.add(cpLocal)
        Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True
        eLocal = self._local(sketch, self._gearPoint(index, uF, 0.0, station), True)
        K = sketch.sketchCurves.sketchLines.addByTwoPoints(O, eLocal)
        K.isConstruction = True
        E = K.endSketchPoint
        O.isFixed = True
        Cp.isFixed = True

        # The rectangle, four lines sharing their corners, seeded where they solve.
        corners = []
        for uv in ((uB, vLo), (uF, vLo), (uF, vHi), (uB, vHi)):
            corners.append(self._local(sketch, self._gearPoint(index, uv[0], uv[1], station), True))
        c0: adsk.core.Point3D = corners[0]
        c1: adsk.core.Point3D = corners[1]
        c2: adsk.core.Point3D = corners[2]
        c3: adsk.core.Point3D = corners[3]
        L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(c0, c1)
        L2 = sketch.sketchCurves.sketchLines.addByTwoPoints(L1.endSketchPoint, c2)
        L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(L2.endSketchPoint, c3)
        L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)

        # The spine's length and the angle come first ([SCREW-F-BORE-ANGLE-ORDER]).
        ox = oLocal.x
        oy = oLocal.y
        kx = eLocal.x - ox
        ky = eLocal.y - oy
        kLen = math.hypot(kx, ky)
        lengthText = adsk.core.Point3D.create(ox + kx / 2 - ky / kLen * 0.1,
                                              oy + ky / 2 + kx / kLen * 0.1, 0)
        lengthDim = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDim.parameter.value = uF

        if abs(math.sin(theta)) >= math.sqrt(0.5):
            ruX = cpLocal.x - ox
            ruY = cpLocal.y - oy
            ruLen = math.hypot(ruX, ruY)
            rayRu = (ruX / ruLen, ruY / ruLen)
            rayK = (kx / kLen, ky / kLen)
            angleText = adsk.core.Point3D.create(ox + (rayRu[0] + rayK[0]) * uF / 3,
                                                 oy + (rayRu[1] + rayK[1]) * uF / 3, 0)
            angleValue = math.acos(max(-1.0, min(1.0, rayRu[0] * rayK[0] + rayRu[1] * rayK[1])))
            angleDim = sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)
        else:
            # Intersect the infinite lines through O, Cp and P1, P2 in sketch space.
            ax = cpLocal.x - ox
            ay = cpLocal.y - oy
            bx = c2.x - c1.x
            by = c2.y - c1.y
            denom = ax * by - ay * bx
            if abs(denom) < 1e-12:
                raise Exception(f'Screw gear: {boreName}: Ru and L2 are parallel, the angle '
                                f'cannot be dimensioned')
            tParam = ((c1.x - ox) * by - (c1.y - oy) * bx) / denom
            ix = ox + tParam * ax
            iy = oy + tParam * ay
            towardX = cpLocal.x - ix
            towardY = cpLocal.y - iy
            if math.hypot(towardX, towardY) < 1e-9:
                towardX = ox - ix
                towardY = oy - iy
            towardLen = math.hypot(towardX, towardY)
            rayRu = (towardX / towardLen, towardY / towardLen)
            bLen = math.hypot(bx, by)
            rayL2 = (bx / bLen, by / bLen)
            angleText = adsk.core.Point3D.create(ix + (rayRu[0] + rayL2[0]) * uF / 3,
                                                 iy + (rayRu[1] + rayL2[1]) * uF / 3, 0)
            angleValue = math.acos(max(-1.0, min(1.0, rayRu[0] * rayL2[0] + rayRu[1] * rayL2[1])))
            angleDim = sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)
        angleDim.parameter.value = angleValue

        # Then the five geometric constraints, then the three offsets.
        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addParallel(L4, L2)
        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        lowerText = self._local(sketch, self._gearPoint(index, uF / 2, vLo / 2, station), True)
        upperText = self._local(sketch, self._gearPoint(index, uF / 2, vHi / 2, station), True)
        widthText = self._local(sketch, self._gearPoint(index, uB / 2, vHi / 2, station), True)
        lowerDim = sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)
        lowerDim.parameter.value = -vLo
        upperDim = sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)
        upperDim.parameter.value = vHi
        widthDim = sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)
        widthDim.parameter.value = uF - uB

        sketch.isComputeDeferred = False
        self._requireFullyConstrained(sketch)
        profile = find_profile_by_curve_counts(sketch, lines=4)

        # Every solved corner within 0.005 mm of its seed.
        solved = (L1.startSketchPoint.geometry, L1.endSketchPoint.geometry,
                  L2.endSketchPoint.geometry, L3.endSketchPoint.geometry)
        worst = 0.0
        worstIndex = -1
        for i in range(4):
            got: adsk.core.Point3D = solved[i]
            want: adsk.core.Point3D = corners[i]
            dist = math.sqrt((got.x - want.x) ** 2 + (got.y - want.y) ** 2 + (got.z - want.z) ** 2)
            if dist > worst:
                worst = dist
                worstIndex = i
        if worst > 0.0005:
            raise Exception(f'Screw gear: sketch "{boreName}" corner {worstIndex} solved '
                            f'{worst * 10:.5f} mm from its seed, more than 0.005 mm')

        # The twisted sweep cut ([SCREW-F-TWISTED-SLOT], [PB-SWEEP-TWIST]).
        borePath = design.features.createPath(boreLine, False)
        sweepInput = design.features.sweepFeatures.createInput(
            profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal((self.sOut - self.sIn) / self.lam)
        sweepInput.participantBodies = [self.cageBody]
        sweepFeature = design.features.sweepFeatures.add(sweepInput)
        if sweepFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the {boreName} sweep cut left '
                            f'{sweepFeature.bodies.count} bodies, expected 1')
        cageBody = sweepFeature.bodies.item(0)
        self.cageBody = cageBody

        # The sweep's sense, checked at the crossing ([SCREW-F-SWEEP-CHECK]).
        sc = sigma * self.cageRadius
        th = self._theta(index, sc)
        uDir = _vadd(_vscale(self.gearU[index], math.cos(th)), _vscale(self.gearV[index], math.sin(th)))
        vDir = _vadd(_vscale(self.gearU[index], -math.sin(th)), _vscale(self.gearV[index], math.cos(th)))
        crossing = _vadd(self.gearOrigin[index], _vscale(self.gearDir[index], sc))
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        inside = adsk.fusion.PointContainment.PointInsidePointContainment
        reach = self.W / 2 + self.clearance / 2
        for side in (-1.0, 1.0):
            probePoint = self._point(_vadd(crossing, _vscale(uDir, side * reach)))
            reading = cageBody.pointContainment(probePoint)
            if reading != outside:
                raise Exception(f'Screw gear: {boreName}: the probe on the '
                                f'{"+" if side > 0 else "-"}u side of the crossing read '
                                f'containment {reading}, expected outside; the sweep turned '
                                f'the wrong way')
        if self.roof > 0:
            roofSign = self._roofSign(index, sigma)
            depth = self.ht + self.roof / 2
            probePoint = self._point(_vadd(crossing, _vscale(vDir, roofSign * depth)))
            reading = cageBody.pointContainment(probePoint)
            if reading != outside:
                raise Exception(f'Screw gear: {boreName}: the roof probe read containment '
                                f'{reading}, expected outside in the added roof room')
            probePoint = self._point(_vadd(crossing, _vscale(vDir, -roofSign * depth)))
            reading = cageBody.pointContainment(probePoint)
            if reading != inside:
                raise Exception(f'Screw gear: {boreName}: the floor probe read containment '
                                f'{reading}, expected inside beyond the floor')

    def _cutWindows(self):
        if len(self.windows) == 0:
            return
        design = self.designComponent
        anchorLine: adsk.fusion.SketchLine = self.anchorLine
        targetPlane = self.targetPlane
        planeInput = design.constructionPlanes.createInput()
        if math.degrees(self.Sigma) <= 90:
            rightAngle = adsk.core.ValueInput.createByString('90 deg')
            planeInput.setByAngle(anchorLine, rightAngle, targetPlane)
        else:
            midpointFraction = adsk.core.ValueInput.createByReal(0.5)
            planeInput.setByDistanceOnPath(anchorLine, midpointFraction)
        windowPlane = design.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'
        for window in self.windows:
            self._cutWindow(windowPlane, window)

    def _cutWindow(self, plane: adsk.fusion.ConstructionPlane, window):
        design = self.designComponent
        label = window['label']
        across = window['across']
        facing = window['facing']
        corners = window['corners']
        sketch = design.sketches.add(plane)
        sketch.name = f'Window {label}'
        points = []
        for corner in corners:
            # C + t*across + z*n̂, across being level.
            worldPoint = self._point(self._frameWorld(corner[0] * across[0], corner[0] * across[1],
                                                      corner[1]))
            local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            local.z = 0
            points.append(sketch.sketchPoints.add(local))
        count = len(points)
        for i in range(count):
            startPoint: adsk.fusion.SketchPoint = points[i]
            endPoint: adsk.fusion.SketchPoint = points[(i + 1) % count]
            sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
        for item in points:
            point: adsk.fusion.SketchPoint = item
            point.isFixed = True
        self._requireFullyConstrained(sketch)
        if sketch.profiles.count != 1:
            raise Exception(f'Screw gear: sketch "Window {label}" has {sketch.profiles.count} '
                            f'profiles, expected 1')
        profile = sketch.profiles.item(0)

        # The probe: the average corner, half way through the wall.
        tc = 0.0
        zc = 0.0
        for corner in corners:
            tc += corner[0] / len(corners)
            zc += corner[1] / len(corners)
        ri = self.Ri * 10
        ro = self.Ro * 10
        a0 = math.sqrt(max(0.0, ri * ri - tc * tc))
        a1 = math.sqrt(max(0.0, ro * ro - tc * tc))
        a = (a0 + a1) / 2
        probeMm = (tc * across[0] + a * facing[0], tc * across[1] + a * facing[1], zc)
        cageBody: adsk.fusion.BRepBody = self.cageBody
        probePoint = self._point(self._frameWorld(probeMm[0], probeMm[1], probeMm[2]))
        reading = cageBody.pointContainment(probePoint)
        if reading != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(f'Screw gear: the window facing {label}: its probe read containment '
                            f'{reading} before the cut, expected inside the wall')

        # Out from the plane through the axis toward the facing, Ro + 1 mm.
        facingWorld = self._frameVector(facing)
        directionPoint = self._point(_vadd(self.C, facingWorld))
        towards: adsk.core.Point3D = sketch.modelToSketchSpace(directionPoint)
        if towards.z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
        operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        extrudeInput = design.features.extrudeFeatures.createInput(profile, operation)
        distanceValue = adsk.core.ValueInput.createByReal(self.Ro + 0.1)
        extent = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeInput.participantBodies = [cageBody]
        extrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the window cut facing {label} left '
                            f'{extrudeFeature.bodies.count} bodies, expected 1')
        cageBody = extrudeFeature.bodies.item(0)
        self.cageBody = cageBody
        reading = cageBody.pointContainment(probePoint)
        if reading != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(f'Screw gear: the window facing {label}: its probe read containment '
                            f'{reading} after the cut, expected outside')

    def _cutMarkers(self):
        # One plane depth below the top face, and four blind pockets on it:
        # a square over each -R bore and a circle over each +R bore.
        design = self.designComponent
        depth = min(0.08, self.collarWall / 3)
        overrun = min(0.01, self.collarWall / 20)
        height = self.cageRise - depth
        gearBAxisPlane: adsk.fusion.ConstructionPlane = self.axisPlanes[1]
        geomB = gearBAxisPlane.geometry
        normalB = (geomB.normal.x, geomB.normal.y, geomB.normal.z)
        sign = 1.0 if _vdot(normalB, self.nHat) > 0 else -1.0
        signedOffset = sign * (height - self.A / 2)
        planeInput = design.constructionPlanes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(signedOffset)
        planeInput.setByOffset(gearBAxisPlane, offsetValue)
        markerPlane = design.constructionPlanes.add(planeInput)
        markerPlane.name = 'Marker Plane'
        geomM = markerPlane.geometry
        originM = (geomM.origin.x, geomM.origin.y, geomM.origin.z)
        got = _vdot(_vsub(originM, self.C), self.nHat)
        if abs(got - height) > 1e-5:
            raise Exception(f'Screw gear: the Marker Plane stands {got * 10:.5f} mm above the '
                            f'centre, expected {height * 10:.5f} mm')
        halfSize = min(0.15, self.collarHalf / 2)
        for index in range(2):
            for sigma in (-1.0, 1.0):
                self._cutMarker(markerPlane, index, sigma, depth, overrun, halfSize)

    def _cutMarker(self, markerPlane: adsk.fusion.ConstructionPlane, index: int, sigma: float,
                   depth: float, overrun: float, halfSize: float):
        design = self.designComponent
        label = GEAR_LABELS[index]
        tag = '-R' if sigma < 0 else '+R'
        what = f'{label} bore {tag}'
        centre = _vadd(_vadd(self.C, _vscale(self.nHat, self.cageRise - depth)),
                       _vscale(self.gearDir[index], sigma * self.cageRadius))
        extent = halfSize if sigma > 0 else math.sqrt(2) * halfSize
        if not extent < self.collarHalf:
            raise Exception(f'Screw gear: the {what} mark reaches {extent * 10:.3f} mm from its '
                            f'centre, past the {self.collarHalf * 10:.3f} mm half wall of the top face')

        sketch = design.sketches.add(markerPlane)
        sketch.name = f'{label} Bore {tag} Mark'
        if sigma > 0:
            center = self._local(sketch, centre, True)
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(center, halfSize)
            circle.centerSketchPoint.isFixed = True
            textPoint = adsk.core.Point3D.create(center.x + halfSize, center.y, 0)
            diameter = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameter.parameter.value = 2 * halfSize
        else:
            points = []
            for du, dv in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
                corner = _vadd(_vadd(centre, _vscale(self.eHat, du * halfSize)),
                               _vscale(self.kHat, dv * halfSize))
                worldPoint = self._point(corner)
                localPoint: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
                localPoint.z = 0
                points.append(sketch.sketchPoints.add(localPoint))
            for i in range(4):
                p0: adsk.fusion.SketchPoint = points[i]
                p1: adsk.fusion.SketchPoint = points[(i + 1) % 4]
                sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
            for item in points:
                point: adsk.fusion.SketchPoint = item
                point.isFixed = True
        self._requireFullyConstrained(sketch)
        if sketch.profiles.count != 1:
            raise Exception(f'Screw gear: the {what} mark sketch has {sketch.profiles.count} '
                            f'profiles, expected 1')
        profile = sketch.profiles.item(0)

        # The tool: from the plane, depth below the top, out past the face.
        markCentrePlusNormal = self._point(_vadd(centre, self.nHat))
        towards: adsk.core.Point3D = sketch.modelToSketchSpace(markCentrePlusNormal)
        if towards.z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
        operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        extrudeInput = design.features.extrudeFeatures.createInput(profile, operation)
        distanceValue = adsk.core.ValueInput.createByReal(depth + overrun)
        extentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        extrudeInput.setOneSideExtent(extentDefinition, direction)
        extrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the {what} mark tool extrude left '
                            f'{extrudeFeature.bodies.count} bodies, expected 1')
        markBody = extrudeFeature.bodies.item(0)

        # The pocket: cut the tool out of the cage ([SCREW-F-BORE-MARKS]).
        cageBody: adsk.fusion.BRepBody = self.cageBody
        inside = adsk.fusion.PointContainment.PointInsidePointContainment
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        beforeProbe = self._point(_vadd(centre, _vscale(self.nHat, depth / 2)))
        reading = cageBody.pointContainment(beforeProbe)
        if reading != inside:
            raise Exception(f'Screw gear: the {what} mark: halfway down the pocket read '
                            f'containment {reading} before the cut, expected material')
        tools = adsk.core.ObjectCollection.create()
        tools.add(markBody)
        combineInput = design.features.combineFeatures.createInput(cageBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature = design.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise Exception(f'Screw gear: the {what} mark cut left {combineFeature.bodies.count} '
                            f'bodies, expected 1')
        cageBody = combineFeature.bodies.item(0)
        self.cageBody = cageBody
        afterProbe = self._point(_vadd(centre, _vscale(self.nHat, depth / 2)))
        reading = cageBody.pointContainment(afterProbe)
        if reading != outside:
            raise Exception(f'Screw gear: the {what} mark: halfway down the pocket read '
                            f'containment {reading} after the cut, expected air')
        floorProbe = self._point(_vadd(centre, _vscale(self.nHat, -0.01)))
        reading = cageBody.pointContainment(floorProbe)
        if reading != inside:
            raise Exception(f'Screw gear: the {what} mark: 0.1 mm below the pocket floor read '
                            f'containment {reading}, expected material')

    # -- relocation ----------------------------------------------------------

    def _relocate(self, body: adsk.fusion.BRepBody, targetOccurrence: adsk.fusion.Occurrence,
                  name: str):
        body.name = name
        moved = body.moveToComponent(targetOccurrence)
        if moved is None:
            raise Exception(f'Screw gear: moving body "{name}" into its component failed')

    def relocateBodies(self):
        # Finished bodies move out of Design, keeping their world position
        # ([PB-NO-CROSS-SIBLING]).
        gearA: adsk.fusion.BRepBody = self.gearBodies[0]
        gearB: adsk.fusion.BRepBody = self.gearBodies[1]
        self._relocate(gearA, self.gearOccs[0], 'Gear A')
        self._relocate(gearB, self.gearOccs[1], 'Gear B')
        self._relocate(self.cageBody, self.cageOcc, 'Cage')
