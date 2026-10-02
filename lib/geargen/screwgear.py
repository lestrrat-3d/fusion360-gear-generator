import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator
from .utilities import find_profile_by_curve_counts
from . import solids


# ---------------------------------------------------------------------------
# S01: module-level constants -- dialog input ids and the cell size.
# ---------------------------------------------------------------------------

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
# Plain 3-vector helpers. S03 and S04 run before any Fusion geometry exists,
# in an abstract frame of their own; S06 onward builds the real world frame
# (self.C, self.eHat, self.kHat, self.nHat) from actual Fusion geometry but
# does every per-gear sum in the same plain-tuple arithmetic, converting to
# adsk.core.Point3D / Vector3D only at the point of a Fusion call. This
# sidesteps Vector3D.add/scaleBy/normalize mutating their receiver in place.
# ---------------------------------------------------------------------------

def _add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale3(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0])


def _unit3(a):
    n = math.sqrt(_dot3(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _pt(t) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(t[0], t[1], t[2])


def _vec(t) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def _localOf(sketch: adsk.fusion.Sketch, world3) -> adsk.core.Point3D:
    """A world point (cm) mapped into `sketch`'s local coordinates, with z
    zeroed ([PB-SKETCH-ZERO-Z])."""
    p = sketch.modelToSketchSpace(_pt(world3))
    p.z = 0
    return p


def _faceName(d):
    if d[1] > 0:
        return '+k'
    if d[1] < 0:
        return '-k'
    if d[0] > 0:
        return '+e'
    return '-e'


# ---------------------------------------------------------------------------
# 2D polygon helpers for the window search (S04): a half-plane clip that
# keeps a corner on the line and adds the crossing point (Sutherland-Hodgman
# on a single linear half-plane a*x + b*y <= k), a convex hull (Andrew's
# monotone chain, counter-clockwise), the shoelace area, and the
# point/segment distance used by wallGap.
# ---------------------------------------------------------------------------

def _clip_line(poly, a, b, k):
    if not poly:
        return []
    out = []
    n = len(poly)
    for i in range(n):
        cx, cy = poly[i]
        nx, ny = poly[(i + 1) % n]
        dCur = a * cx + b * cy - k
        dNext = a * nx + b * ny - k
        if dCur <= 0:
            out.append((cx, cy))
        if (dCur < 0 and dNext > 0) or (dCur > 0 and dNext < 0):
            t = dCur / (dCur - dNext)
            out.append((cx + t * (nx - cx), cy + t * (ny - cy)))
    return out


def _convex_hull(points):
    pts = sorted(set(points))
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


def _shoelace_area(poly):
    area = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _point_segment_dist2(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / length2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _inside_or_on_convex(x, y, piece):
    if len(piece) < 3:
        return False
    n = len(piece)
    for i in range(n):
        ax, ay = piece[i]
        bx, by = piece[(i + 1) % n]
        cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
        if cross < -1e-9:
            return False
    return True


def _min_dist2_to_edges(x, y, piece):
    best = None
    n = len(piece)
    for i in range(n):
        ax, ay = piece[i]
        bx, by = piece[(i + 1) % n]
        d2 = _point_segment_dist2(x, y, ax, ay, bx, by)
        if best is None or d2 < best:
            best = d2
    return best


# ---------------------------------------------------------------------------
# S01: dialog inputs.
# ---------------------------------------------------------------------------

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

        ribbonGroupInput = inputs.addGroupCommandInput('ribbonGroup', 'Ribbon')
        ribbonGroupInput.isExpanded = True
        ribbonGroup = ribbonGroupInput.children
        ribbonGroup.addValueInput(
            INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', adsk.core.ValueInput.createByReal(1.5))
        ribbonGroup.addValueInput(
            INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', adsk.core.ValueInput.createByReal(68))
        ribbonGroup.addValueInput(
            INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', adsk.core.ValueInput.createByReal(4.95))
        ribbonGroup.addValueInput(
            INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm',
            adsk.core.ValueInput.createByReal(0.375))
        ribbonGroup.addValueInput(
            INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', adsk.core.ValueInput.createByReal(0.2625))
        ribbonGroup.addValueInput(
            INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm',
            adsk.core.ValueInput.createByReal(0.2625))

        frameGroupInput = inputs.addGroupCommandInput('frameGroup', 'Frame')
        frameGroupInput.isExpanded = True
        frameGroup = frameGroupInput.children
        frameGroup.addValueInput(
            INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', adsk.core.ValueInput.createByReal(1.5))
        frameGroup.addValueInput(
            INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', adsk.core.ValueInput.createByReal(1.875))
        frameGroup.addValueInput(
            INPUT_ID_CLEARANCE, 'Clearance', 'mm', adsk.core.ValueInput.createByReal(0.02))
        frameGroup.addValueInput(
            INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm',
            adsk.core.ValueInput.createByReal(0.3))
        frameGroup.addValueInput(
            INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', adsk.core.ValueInput.createByReal(0.3))

        meshGroupInput = inputs.addGroupCommandInput('meshGroup', 'Mesh (from the mesh search)')
        meshGroupInput.isExpanded = False
        meshGroup = meshGroupInput.children
        meshGroup.addValueInput(
            INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg',
            adsk.core.ValueInput.createByReal(1.3962634))
        meshGroup.addValueInput(
            INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', adsk.core.ValueInput.createByReal(0.09))
        meshGroup.addValueInput(
            INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg',
            adsk.core.ValueInput.createByReal(0.2443461))
        meshGroup.addValueInput(
            INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg',
            adsk.core.ValueInput.createByReal(0.2443461))
        meshGroup.addValueInput(
            INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm',
            adsk.core.ValueInput.createByReal(-0.13))


# ---------------------------------------------------------------------------
# S01: the generator.
# ---------------------------------------------------------------------------

class ScrewGearGenerator(Generator):

    def prefixBase(self) -> str:
        return 'ScrewGear'

    # =======================================================================
    # Entry point (S05).
    # =======================================================================

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        for index in range(2):
            self.buildGear(index)
        self.buildCage()
        self.relocateBodies()

    # =======================================================================
    # S02-S04: processInputs.
    # =======================================================================

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Field declarations, in the cast(None) form, before anything reads them.
        self.designOcc = adsk.fusion.Occurrence.cast(None)
        self.gearOccs = [adsk.fusion.Occurrence.cast(None), adsk.fusion.Occurrence.cast(None)]
        self.cageOcc = adsk.fusion.Occurrence.cast(None)
        self.gearBodies = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]
        self.windows = []

        def item(inputs: adsk.core.CommandInputs, name: str) -> adsk.core.CommandInput:
            found = inputs.itemById(name)
            if found is None:
                raise Exception(f'ScrewGear: dialog input "{name}" not found')
            return found

        # 1. Selections first ([PB-SELECTION-STASH]), before anything creates an occurrence.
        planeInput = adsk.core.SelectionCommandInput.cast(item(inputs, INPUT_ID_PLANE))
        if planeInput.selectionCount != 1:
            raise Exception(f'{INPUT_ID_PLANE}: must have exactly one selection')
        self.plane = planeInput.selection(0).entity

        pointInput = adsk.core.SelectionCommandInput.cast(item(inputs, INPUT_ID_POINT))
        if pointInput.selectionCount != 1:
            raise Exception(f'{INPUT_ID_POINT}: must have exactly one selection')
        self.point = pointInput.selection(0).entity

        parentInput = adsk.core.SelectionCommandInput.cast(item(inputs, INPUT_ID_PARENT))
        if parentInput.selectionCount != 1:
            raise Exception(f'{INPUT_ID_PARENT}: must have exactly one selection')
        parentEntity = parentInput.selection(0).entity
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = adsk.fusion.Occurrence.cast(parentEntity).component
        else:
            self.parentComponent = parentEntity

        # 2. Values, read in internal units (cm, rad) and converted to this
        # build's millimetres/radians working units ([PB-EVAL-EXPRESSION]).
        design = get_design()
        unitsManager = design.unitsManager

        def evalInput(inputs: adsk.core.CommandInputs, unitsManager: adsk.core.UnitsManager,
                       name: str, units: str) -> float:
            valueInput = adsk.core.ValueCommandInput.cast(item(inputs, name))
            return unitsManager.evaluateExpression(valueInput.expression, units)

        W = to_mm(evalInput(inputs, unitsManager, INPUT_ID_RIBBON_WIDTH, 'mm'))
        T = to_mm(evalInput(inputs, unitsManager, INPUT_ID_RIBBON_THICKNESS, 'mm'))
        P = to_mm(evalInput(inputs, unitsManager, INPUT_ID_TOOTH_PITCH, 'mm'))
        H = to_mm(evalInput(inputs, unitsManager, INPUT_ID_TOOTH_HEIGHT, 'mm'))
        NRaw = evalInput(inputs, unitsManager, INPUT_ID_TOOTH_COUNT, '')
        lead = to_mm(evalInput(inputs, unitsManager, INPUT_ID_TWIST_LEAD, 'mm'))
        Sigma = evalInput(inputs, unitsManager, INPUT_ID_CROSS_ANGLE, 'deg')
        E = to_mm(evalInput(inputs, unitsManager, INPUT_ID_ENGAGEMENT, 'mm'))
        PhiA = evalInput(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_A, 'deg')
        PhiB = evalInput(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_B, 'deg')
        Z0B = to_mm(evalInput(inputs, unitsManager, INPUT_ID_ASSEMBLY_PHASE, 'mm'))
        R = to_mm(evalInput(inputs, unitsManager, INPUT_ID_CAGE_RADIUS, 'mm'))
        rise = to_mm(evalInput(inputs, unitsManager, INPUT_ID_CAGE_RISE, 'mm'))
        clr = to_mm(evalInput(inputs, unitsManager, INPUT_ID_CLEARANCE, 'mm'))
        ch = to_mm(evalInput(inputs, unitsManager, INPUT_ID_COLLAR_HALF, 'mm'))
        cw = to_mm(evalInput(inputs, unitsManager, INPUT_ID_COLLAR_WALL, 'mm'))

        # 3. Range checks, in order, each naming the offending input id.
        for (value, inputId) in (
            (W, INPUT_ID_RIBBON_WIDTH), (T, INPUT_ID_RIBBON_THICKNESS),
            (P, INPUT_ID_TOOTH_PITCH), (lead, INPUT_ID_TWIST_LEAD),
            (ch, INPUT_ID_COLLAR_HALF), (cw, INPUT_ID_COLLAR_WALL), (clr, INPUT_ID_CLEARANCE),
        ):
            if not (value > 0):
                raise Exception(f'{inputId}: must be greater than 0 (got {value})')

        if NRaw != round(NRaw):
            raise Exception(f'{INPUT_ID_TOOTH_COUNT}: must be a whole number (got {NRaw})')
        N = int(round(NRaw))
        if N < 4:
            raise Exception(f'{INPUT_ID_TOOTH_COUNT}: must be at least 4 (got {N})')

        if not (H > 0 and H < W / 2.0):
            raise Exception(
                f'{INPUT_ID_TOOTH_HEIGHT}: must be greater than 0 and less than ribbonWidth / 2 '
                f'(got {H} mm, ribbonWidth / 2 = {W / 2.0} mm)')

        if not (E > 0 and E <= H):
            raise Exception(
                f'{INPUT_ID_ENGAGEMENT}: must be greater than 0 and at most toothHeight '
                f'(got {E} mm, toothHeight = {H} mm)')

        sigmaDeg = math.degrees(Sigma)
        if not (0 < sigmaDeg < 180):
            raise Exception(
                f'{INPUT_ID_CROSS_ANGLE}: must lie strictly between 0 and 180 degrees '
                f'(got {sigmaDeg} degrees)')

        if not (abs(Z0B) < P):
            raise Exception(
                f'{INPUT_ID_ASSEMBLY_PHASE}: must lie strictly within plus or minus toothPitch '
                f'(got {Z0B} mm, toothPitch = {P} mm)')

        if not (R + ch + 1.0 < N * P / 2.0):
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: both bores, each cut a millimetre past the wall, '
                f'must lie within the ribbon (R + collarHalf + 1 = {R + ch + 1.0} mm, '
                f'N * toothPitch / 2 = {N * P / 2.0} mm)')

        self.W, self.T, self.P, self.H, self.N = W, T, P, H, N
        self.lead, self.Sigma, self.E = lead, Sigma, E
        self.PhiA, self.PhiB, self.Z0B = PhiA, PhiB, Z0B
        self.R, self.rise, self.clr, self.ch, self.cw = R, rise, clr, ch, cw
        self.Lambda = lead / (2.0 * math.pi)
        self.A = W - E
        self.L = N * P

        # 4. The sleeve's four checks (S03), which also computes the search frame
        # and bore list the window search (S04) reuses.
        frame, bores = self._checkSleeve()

        # 5. Derived counts of the cell (S10, S12).
        self.c = min(CELL_TEETH, N)
        self.q = N // self.c
        self.r = N % self.c
        self.n = max(math.ceil((P / self.Lambda) / math.radians(2)), 8)

        # 6. The window search (S04).
        self.windows = self._searchWindows(frame, bores)

    # -----------------------------------------------------------------------
    # S03: the sleeve's four checks, and the abstract search frame they and
    # S04 share. All of this runs before any Fusion geometry exists; it is
    # pure arithmetic in the abstract frame C=(0,0,0), eHat=(1,0,0),
    # kHat=(0,1,0), nHat=(0,0,1), everything in millimetres.
    # -----------------------------------------------------------------------

    def _gearSearchFrame(self):
        half = self.Sigma / 2.0
        dirA = (math.cos(half), math.sin(half), 0.0)
        dirB = (math.cos(half), -math.sin(half), 0.0)
        originA = (0.0, 0.0, -self.A / 2.0)
        originB = (0.0, 0.0, self.A / 2.0)
        uA = (0.0, 0.0, 1.0)
        uB = (0.0, 0.0, -1.0)
        vA = _cross3(dirA, uA)
        vB = _cross3(dirB, uB)
        return {
            'A': {'dir': dirA, 'origin': originA, 'u': uA, 'v': vA, 'phi': self.PhiA},
            'B': {'dir': dirB, 'origin': originB, 'u': uB, 'v': vB, 'phi': self.PhiB},
        }

    def _searchAngle(self, gear, s):
        return s / self.Lambda + gear['phi']

    def _searchWorld(self, gear, u, v, s):
        th = self._searchAngle(gear, s)
        c, sn = math.cos(th), math.sin(th)
        pt = _add3(gear['origin'], _scale3(gear['dir'], s))
        pt = _add3(pt, _scale3(gear['u'], u * c - v * sn))
        pt = _add3(pt, _scale3(gear['v'], u * sn + v * c))
        return pt

    def _bores(self, frame):
        sIn, sOut, R = self._sIn, self._sOut, self.R
        bores = []
        for (label, sigma) in (('A', -1.0), ('A', 1.0), ('B', -1.0), ('B', 1.0)):
            gear = frame[label]
            span = (-sOut, -sIn) if sigma < 0 else (sIn, sOut)
            crossing = _add3(gear['origin'], _scale3(gear['dir'], sigma * R))
            bores.append({
                'gear': label, 'sigma': sigma, 'span': span, 'crossing': crossing,
                'name': 'gear {} {}'.format(label, '-R' if sigma < 0 else '+R'),
            })
        return bores

    def _checkSleeve(self):
        Ri = self.R - self.ch
        Ro = self.R + self.ch
        hw = self.W / 2.0 + self.clr
        ht = self.T / 2.0 + self.clr
        cCorner = math.hypot(hw, ht)
        sIn = math.sqrt(Ri * Ri - cCorner * cCorner) - 1.0
        sOut = Ro + 1.0
        axialWindow = 1.5 * math.sqrt(self.W * self.W - self.A * self.A) / math.sin(self.Sigma)

        if not (math.hypot(cCorner, 1.0) < Ri):
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: the channel does not start in the hollow')
        if not (math.hypot(axialWindow, math.hypot(self.W / 2.0, self.T / 2.0)) + self.clr <= Ri):
            raise Exception(
                f'{INPUT_ID_CAGE_RADIUS}: the mesh is not visible along the axis')
        if not (self.rise >= self.A / 2.0 + cCorner + self.cw):
            raise Exception(
                f'{INPUT_ID_CAGE_RISE}: the end faces do not keep collarWall')

        self.Ri, self.Ro, self.hw, self.ht = Ri, Ro, hw, ht
        self._sIn, self._sOut = sIn, sOut

        frame = self._gearSearchFrame()
        bores = self._bores(frame)
        separation, names = self._channelSeparation(frame, bores)
        if not (separation >= self.cw):
            raise Exception(
                f'{INPUT_ID_COLLAR_WALL}: the wall between {names[0]} and {names[1]} is '
                f'{separation:.4f} mm, less than collarWall ({self.cw} mm)')
        return frame, bores

    def _channelSeparation(self, frame, bores):
        hw, ht, Ri, Ro = self.hw, self.ht, self.Ri, self.Ro
        nHat = (0.0, 0.0, 1.0)
        kept = []
        for bore in bores:
            gear = frame[bore['gear']]
            sigma = bore['sigma']
            pts = []
            samples = []
            for i in range(17):
                v = -ht + 2 * ht * i / 16.0
                samples.append((hw, v))
                samples.append((-hw, v))
            for i in range(17):
                u = -hw + 2 * hw * i / 16.0
                samples.append((u, ht))
                samples.append((u, -ht))
            i = 0
            while True:
                s = sigma * self._sIn + sigma * 0.1 * i
                if abs(s) > self._sOut:
                    break
                for (u, v) in samples:
                    p = self._searchWorld(gear, u, v, s)
                    r = math.hypot(p[0], p[1])
                    if Ri - 0.5 <= r <= Ro + 0.5:
                        pts.append(p)
                i += 1
            kept.append(pts)

        gapDefs = (
            ((0.0, 1.0, 0.0), 1, 2),
            ((-1.0, 0.0, 0.0), 2, 0),
            ((0.0, -1.0, 0.0), 0, 3),
            ((1.0, 0.0, 0.0), 3, 1),
        )

        best = float('inf')
        bestNames = ('', '')
        for (d, firstIdx, secondIdx) in gapDefs:
            across = _cross3(nHat, d)

            def proj(pts):
                return [(_dot3(p, across), _dot3(p, nHat)) for p in pts]

            ptsA, ptsB = proj(kept[firstIdx]), proj(kept[secondIdx])
            hullA, hullB = _convex_hull(ptsA), _convex_hull(ptsB)
            if len(hullA) < 1 or len(hullB) < 1:
                raise Exception(
                    f'{bores[firstIdx]["name"]}/{bores[secondIdx]["name"]}: '
                    'the channel sampling found no points near the wall')
            gapSep = None
            for hull in (hullA, hullB):
                n = len(hull)
                for i in range(n):
                    ax, ay = hull[i]
                    bx, by = hull[(i + 1) % n]
                    ex, ey = bx - ax, by - ay
                    length = math.hypot(ex, ey)
                    if length == 0:
                        continue
                    mx, my = -ey / length, ex / length
                    pa = [mx * px + my * py for (px, py) in hullA]
                    pb = [mx * px + my * py for (px, py) in hullB]
                    edgeSep = max(min(pb) - max(pa), min(pa) - max(pb))
                    if gapSep is None or edgeSep > gapSep:
                        gapSep = edgeSep
            if gapSep is None:
                raise Exception(
                    f'{bores[firstIdx]["name"]}/{bores[secondIdx]["name"]}: '
                    'neither hull has an edge to separate along')
            if gapSep < best:
                best = gapSep
                bestNames = (bores[firstIdx]['name'], bores[secondIdx]['name'])
        return best, bestNames

    # -----------------------------------------------------------------------
    # S04: the window search, in the same abstract search frame.
    # -----------------------------------------------------------------------

    def _sectionInWall(self, frame, gearLabel, s):
        if abs(s) >= self.Ro:
            return [], []
        gear = frame[gearLabel]
        hw, ht = self.hw, self.ht
        th = self._searchAngle(gear, s)
        c, sn = math.cos(th), math.sin(th)
        corners = ((-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht))
        rotated = [(u * c - v * sn, u * sn + v * c) for (u, v) in corners]

        originN = gear['origin'][2]
        uN = gear['u'][2]
        loX = (-self.rise - originN) / uN
        hiX = (self.rise - originN) / uN
        xlo, xhi = min(loX, hiX), max(loX, hiX)
        poly = _clip_line(rotated, 1.0, 0.0, xhi)
        poly = _clip_line(poly, -1.0, 0.0, -xlo)

        near = math.sqrt(max(0.0, self.Ri * self.Ri - s * s))
        far = math.sqrt(self.Ro * self.Ro - s * s)
        piece1 = _clip_line(poly, 0.0, -1.0, -near)
        piece1 = _clip_line(piece1, 0.0, 1.0, far)
        piece2 = _clip_line(poly, 0.0, 1.0, -near)
        piece2 = _clip_line(piece2, 0.0, -1.0, far)
        return piece1, piece2

    def _searchWindows(self, frame, bores):
        windows = []
        if math.degrees(self.Sigma) <= 90.0:
            facings = ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0))
        else:
            facings = ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
        for d in facings:
            window = self._newWindow(frame, bores, d)
            if window is not None:
                windows.append(window)
        return windows

    def _newWindow(self, frame, bores, d):
        nHat = (0.0, 0.0, 1.0)
        across = _cross3(nHat, d)

        flank = [b for b in bores if _dot3(b['crossing'], d) > 0]
        far = [b for b in bores if _dot3(b['crossing'], d) <= 0]
        if len(flank) != 2 or len(far) != 2:
            raise Exception(
                f'window facing {_faceName(d)}: {len(flank)} bore(s) flank it, want 2')
        low, high = flank[0], flank[1]
        if _dot3(low['crossing'], nHat) > _dot3(high['crossing'], nHat):
            low, high = high, low
        lean = 1.0 if _dot3(high['crossing'], across) > _dot3(low['crossing'], across) else -1.0

        def walkReach(bore, takeMax) -> float:
            gear = frame[bore['gear']]
            spanLo, spanHi = sorted(bore['span'])
            best = None
            i = 0
            while True:
                s = spanLo + 0.001 * i
                if s > spanHi:
                    break
                for piece in self._sectionInWall(frame, bore['gear'], s):
                    for (u, v) in piece:
                        pt = _add3(gear['origin'], _add3(
                            _scale3(gear['u'], u), _add3(
                                _scale3(gear['v'], v), _scale3(gear['dir'], s))))
                        m = pt[2] + lean * _dot3(pt, across)
                        if best is None or (takeMax and m > best) or (not takeMax and m < best):
                            best = m
                i += 1
            if best is None:
                raise Exception(
                    f'{bore["name"]}: the walk along its cut span found no wall-section '
                    'corner to measure the window from')
            return best

        lowReach = walkReach(low, True)
        highReach = walkReach(high, False)
        lo = lowReach + math.sqrt(2) * self.cw
        hi = highReach - math.sqrt(2) * self.cw

        if hi <= lo:
            futil.log(f'No window facing {_faceName(d)}: '
                       'the flanking bores leave no band between them')
            return None

        zLimit = 0.0
        for bore in bores:
            gear = frame[bore['gear']]
            sigma = bore['sigma']
            i = 0
            while True:
                s = sigma * self._sIn + sigma * 0.01 * i
                if abs(s) > self._sOut:
                    break
                if abs(s) <= self.Ro:
                    th = self._searchAngle(gear, s)
                    reach = math.hypot(
                        s, self.hw * abs(math.sin(th)) + self.ht * abs(math.cos(th)))
                    if reach >= self.Ri:
                        candidate = (self.A / 2.0 + self.hw * abs(math.cos(th))
                                     + self.ht * abs(math.sin(th)))
                        if candidate > zLimit:
                            zLimit = candidate
                i += 1

        top = min(2 * zLimit - hi, hi + math.sqrt(2) * self.Ri)
        bottom = max(-2 * zLimit - lo, lo - math.sqrt(2) * self.Ri)
        need = self.cw + 0.1 / math.sqrt(2) + 0.005

        def wallBounds(t):
            a0 = math.sqrt(max(0.0, self.Ri * self.Ri - t * t))
            a1 = math.sqrt(max(0.0, self.Ro * self.Ro - t * t))
            return a0, a1

        stationTables = {}

        def stationTable(bore):
            key = bore['name']
            if key in stationTables:
                return stationTables[key]
            spanLo, spanHi = sorted(bore['span'])
            base = []
            i = 0
            while True:
                s = spanLo + 0.002 * i
                if s > spanHi:
                    break
                base.append(s)
                i += 1
            if not base or base[-1] < spanHi - 1e-12:
                base.append(spanHi)

            def signature(pieces):
                return tuple(len(p) >= 3 for p in pieces)

            raw = []
            prevSig = None
            for s in base:
                pieces = self._sectionInWall(frame, bore['gear'], s)
                sig = signature(pieces)
                if prevSig is not None and sig != prevSig:
                    prevS = raw[-1][0]
                    j = 1
                    while True:
                        s2 = prevS + 0.0001 * j
                        if s2 >= s:
                            break
                        raw.append((s2, self._sectionInWall(frame, bore['gear'], s2)))
                        j += 1
                raw.append((s, pieces))
                prevSig = sig

            stations = []
            for (s, pieces) in raw:
                circles = []
                for piece in pieces:
                    if len(piece) < 3:
                        circles.append(None)
                        continue
                    cx = sum(p[0] for p in piece) / len(piece)
                    cy = sum(p[1] for p in piece) / len(piece)
                    radius = max(math.hypot(px - cx, py - cy) for (px, py) in piece)
                    circles.append((cx, cy, radius))
                stations.append({'s': s, 'pieces': pieces, 'circles': circles})
            stations.sort(key=lambda e: e['s'])
            stationTables[key] = stations
            return stations

        def wallGap(bore, P, reach):
            gear = frame[bore['gear']]
            rel = _sub3(P, gear['origin'])
            x = _dot3(rel, gear['u'])
            y = _dot3(rel, gear['v'])
            sq = _dot3(rel, gear['dir'])
            spanStart, spanEnd = sorted(bore['span'])
            cCorner = math.hypot(self.hw, self.ht)
            clampSq = min(max(sq, spanStart), spanEnd)
            if math.hypot(math.hypot(x, y), sq - clampSq) - cCorner >= reach:
                return reach
            best = [reach * reach]
            stations = stationTable(bore)
            ss = [e['s'] for e in stations]
            # Manual bisect_left: first index with ss[index] >= sq.
            idxLo, idxHi = 0, len(ss)
            while idxLo < idxHi:
                idxMid = (idxLo + idxHi) // 2
                if ss[idxMid] < sq:
                    idxLo = idxMid + 1
                else:
                    idxHi = idxMid
            idx = idxLo

            def scan(i):
                s = stations[i]['s']
                if (sq - s) ** 2 >= best[0]:
                    return False
                for k in range(2):
                    circle = stations[i]['circles'][k]
                    if circle is None:
                        continue
                    cx, cy, radius = circle
                    o = math.hypot(x - cx, y - cy) - radius
                    if o > 0 and (sq - s) ** 2 + o ** 2 >= best[0]:
                        continue
                    piece = stations[i]['pieces'][k]
                    if _inside_or_on_convex(x, y, piece):
                        d2 = 0.0
                    else:
                        d2 = _min_dist2_to_edges(x, y, piece)
                    cand = (sq - s) ** 2 + d2
                    if cand < best[0]:
                        best[0] = cand
                return True

            i = idx
            while i < len(stations) and scan(i):
                i += 1
            i = idx - 1
            while i >= 0 and scan(i):
                i -= 1
            return math.sqrt(best[0])

        def clear(te, delta):
            t = delta * te
            zLow = max(lo - lean * t, bottom + lean * t)
            zHigh = min(hi - lean * t, top + lean * t)
            if zLow > zHigh:
                return False
            a0, a1 = wallBounds(t)
            pts = []
            for z in (zLow, zHigh):
                a = a0
                while True:
                    pts.append((a, z))
                    if a >= a1:
                        break
                    a = min(a + 0.1, a1)
            nz = max(1, math.ceil((zHigh - zLow) / 0.1))
            for a in (a0, a1):
                for k in range(1, nz):
                    pts.append((a, zLow + (zHigh - zLow) * k / nz))
                j = 0
                while True:
                    z = -zLimit + 0.1 * j
                    if z > zLimit:
                        break
                    if zLow < z < zHigh:
                        pts.append((a, z))
                    j += 1
            for (a, z) in pts:
                P = (t * across[0] + z * nHat[0] + a * d[0],
                     t * across[1] + z * nHat[1] + a * d[1],
                     t * across[2] + z * nHat[2] + a * d[2])
                for bore in far:
                    if wallGap(bore, P, need) < need:
                        return False
            return True

        def endSearch(delta):
            loEnd, hiEnd = 0.0, min(self.Ri, self.Ro / math.sqrt(2)) * (1.0 - 1e-9)
            for _ in range(24):
                mid = (loEnd + hiEnd) / 2.0
                if clear(mid, delta):
                    loEnd = mid
                else:
                    hiEnd = mid
            return loEnd

        right = endSearch(1.0)
        left = -endSearch(-1.0)

        Ro2 = self.Ro
        square = [(-2 * Ro2, -2 * Ro2), (2 * Ro2, -2 * Ro2), (2 * Ro2, 2 * Ro2), (-2 * Ro2, 2 * Ro2)]
        hexagon = _clip_line(square, lean, 1.0, hi)
        hexagon = _clip_line(hexagon, -lean, -1.0, -lo)
        hexagon = _clip_line(hexagon, -lean, 1.0, top)
        hexagon = _clip_line(hexagon, lean, -1.0, -bottom)
        hexagon = _clip_line(hexagon, 1.0, 0.0, right)
        hexagon = _clip_line(hexagon, -1.0, 0.0, -left)

        deduped = []
        for pt in hexagon:
            if deduped and math.hypot(pt[0] - deduped[-1][0], pt[1] - deduped[-1][1]) < 0.001:
                continue
            deduped.append(pt)
        if (len(deduped) >= 2
                and math.hypot(deduped[-1][0] - deduped[0][0],
                               deduped[-1][1] - deduped[0][1]) < 0.001):
            deduped.pop()
        hexagon = deduped

        if right <= left or len(hexagon) < 3 or _shoelace_area(hexagon) <= 0:
            futil.log(f'No window facing {_faceName(d)}: '
                       'the far bores leave the band no length')
            return None

        return {'facing': d, 'corners': hexagon}

    # =======================================================================
    # S05: component tree.
    # =======================================================================

    def buildComponentTree(self):
        topOccurrence = self.getOccurrence()
        topOccurrence.component.name = 'Screw Gearing'
        topComponent = topOccurrence.component

        designOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        gearAOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearAOcc.component.name = 'Gear A'
        gearBOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearBOcc.component.name = 'Gear B'
        cageOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        cageOcc.component.name = 'Cage'

        self.designOcc = designOcc
        self.gearOccs = [gearAOcc, gearBOcc]
        self.cageOcc = cageOcc

    # =======================================================================
    # S06-S08: the Anchor sketch and the two gear axis planes.
    # =======================================================================

    def buildAnchor(self):
        designComponent = self.designOcc.component

        # S06: Anchor sketch.
        sketch = designComponent.sketches.add(self.plane)
        sketch.name = 'Anchor'

        centreColl = sketch.project(self.point)
        centre = centreColl.item(0)

        g = centre.geometry
        startSeed = adsk.core.Point3D.create(g.x - 0.5, g.y, 0)
        endSeed = adsk.core.Point3D.create(g.x + 0.5, g.y, 0)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)

        sketch.geometricConstraints.addCoincident(centre, line)
        sketch.geometricConstraints.addMidPoint(centre, line)
        sketch.geometricConstraints.addHorizontal(line)
        textPoint = adsk.core.Point3D.create(g.x, g.y + 0.2, 0)
        dimension = sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dimension.parameter.value = 1.0

        if not sketch.isFullyConstrained:
            raise Exception('Anchor: sketch is not fully constrained')

        centreWorld = centre.worldGeometry
        startWorld = line.startSketchPoint.worldGeometry
        endWorld = line.endSketchPoint.worldGeometry
        self.anchorLine = line
        self.C = (centreWorld.x, centreWorld.y, centreWorld.z)
        self.eHat = _unit3((endWorld.x - startWorld.x, endWorld.y - startWorld.y,
                            endWorld.z - startWorld.z))

        # S07: Gear A Axis Plane, and the sign of n-hat.
        offsetA = -to_cm(self.A / 2.0)
        planeInputA = designComponent.constructionPlanes.createInput()
        planeInputA.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetA))
        axisPlaneA = designComponent.constructionPlanes.add(planeInputA)
        axisPlaneA.name = 'Gear A Axis Plane'

        planeGeom = axisPlaneA.geometry
        normal3 = _unit3((planeGeom.normal.x, planeGeom.normal.y, planeGeom.normal.z))
        origin3 = (planeGeom.origin.x, planeGeom.origin.y, planeGeom.origin.z)
        if _dot3(_sub3(self.C, origin3), normal3) > 0:
            self.nHat = normal3
        else:
            self.nHat = _scale3(normal3, -1.0)
        self.kHat = _cross3(self.nHat, self.eHat)

        half = self.Sigma / 2.0
        dirA = _add3(_scale3(self.eHat, math.cos(half)), _scale3(self.kHat, math.sin(half)))
        dirB = _sub3(_scale3(self.eHat, math.cos(half)), _scale3(self.kHat, math.sin(half)))
        originA = _sub3(self.C, _scale3(self.nHat, to_cm(self.A / 2.0)))
        originB = _add3(self.C, _scale3(self.nHat, to_cm(self.A / 2.0)))
        uHatA = self.nHat
        uHatB = _scale3(self.nHat, -1.0)
        vHatA = _cross3(dirA, uHatA)
        vHatB = _cross3(dirB, uHatB)

        self.gears = [
            {'label': 'Gear A', 'origin': originA, 'dir': dirA, 'uHat': uHatA, 'vHat': vHatA,
             'phi': self.PhiA, 'axisPlane': axisPlaneA},
            {'label': 'Gear B', 'origin': originB, 'dir': dirB, 'uHat': uHatB, 'vHat': vHatB,
             'phi': self.PhiB, 'axisPlane': None},
        ]

        # S08: Gear B Axis Plane, and the check on both.
        offsetB = to_cm(self.A / 2.0)
        planeInputB = designComponent.constructionPlanes.createInput()
        planeInputB.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetB))
        axisPlaneB = designComponent.constructionPlanes.add(planeInputB)
        axisPlaneB.name = 'Gear B Axis Plane'
        self.gears[1]['axisPlane'] = axisPlaneB

        expected = to_cm(self.A / 2.0)
        for plane in (axisPlaneA, axisPlaneB):
            geom = plane.geometry
            n3 = _unit3((geom.normal.x, geom.normal.y, geom.normal.z))
            o3 = (geom.origin.x, geom.origin.y, geom.origin.z)
            distance = abs(_dot3(_sub3(self.C, o3), n3))
            if abs(distance - expected) > 1e-5:
                raise Exception(
                    f'{plane.name}: distance from C is {distance} cm, want {expected} cm')

    # -----------------------------------------------------------------------
    # World-frame helpers shared by every per-gear sketch from S09 on.
    # -----------------------------------------------------------------------

    def _worldPoint(self, index, u, v, s):
        """wpt_g(u, v, s) of S07: a point on gear `index`'s section at
        station s (mm), offset (u, v) (mm) in the section's rotated frame.
        Returns a world point in centimetres."""
        gear = self.gears[index]
        th = s / self.Lambda + gear['phi']
        c, sn = math.cos(th), math.sin(th)
        pt = _add3(gear['origin'], _scale3(gear['dir'], to_cm(s)))
        pt = _add3(pt, _scale3(gear['uHat'], to_cm(u * c - v * sn)))
        pt = _add3(pt, _scale3(gear['vHat'], to_cm(u * sn + v * c)))
        return pt

    def _rotatedBasis(self, index, s):
        gear = self.gears[index]
        th = s / self.Lambda + gear['phi']
        c, sn = math.cos(th), math.sin(th)
        uHatTh = _add3(_scale3(gear['uHat'], c), _scale3(gear['vHat'], sn))
        vHatTh = _add3(_scale3(gear['uHat'], -sn), _scale3(gear['vHat'], c))
        return uHatTh, vHatTh, th

    # =======================================================================
    # S09-S15: per gear.
    # =======================================================================

    def buildGear(self, index):
        self.buildSweepPaths(index)
        cellBody = self.buildToothCell(index)
        self.repeatCellByDoubling(index, cellBody)

    def buildSweepPaths(self, index):
        designComponent = self.designOcc.component
        gear = self.gears[index]
        label = gear['label']

        sketch = designComponent.sketches.add(gear['axisPlane'])
        sketch.name = f'{label} Paths'

        stations = (-self._sOut, -self._sIn, self._sIn, self._sOut)
        points = []
        for s in stations:
            worldPoint = self._worldPoint(index, 0.0, 0.0, s)
            local = sketch.modelToSketchSpace(_pt(worldPoint))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))

        minusLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[0], points[1])
        plusLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[2], points[3])
        for pt in points:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')

        self.pathLines[index] = {'bore-': minusLine, 'bore+': plusLine}

    def _toothEdge(self, s, Z0):
        H, W, P = self.H, self.W, self.P
        return W / 2.0 - H / 2.0 + H / 2.0 * math.cos(2 * math.pi * (s - Z0) / P)

    def buildToothCell(self, index):
        gear = self.gears[index]
        sketchName = f'{gear["label"]} Cell Sections'
        return self._buildAndLoftCell(index, sketchName, self.c, 0)

    def _buildAndLoftCell(self, index, sketchName, teeth, firstSectionIndex):
        """S10 + S11: the Cell Sections (or Cell Remainder) sketch of `teeth`
        teeth starting at global section `firstSectionIndex`, then its loft.
        Returns the lofted body."""
        designComponent = self.designOcc.component
        gear = self.gears[index]

        sketch = designComponent.sketches.add(gear['axisPlane'])
        sketch.name = sketchName
        sketch.isComputeDeferred = True

        Z0 = 0.0 if index == 0 else self.Z0B
        s0 = Z0 - self.L / 2.0
        count = teeth * self.n

        sectionLines = []
        allPoints = []
        for k in range(count + 1):
            sk = s0 + (firstSectionIndex + k) * self.P / self.n
            uB = -self.W / 2.0
            uF = self._toothEdge(sk, Z0)
            hv = self.T / 2.0
            corners3 = (
                self._worldPoint(index, uB, -hv, sk),
                self._worldPoint(index, uF, -hv, sk),
                self._worldPoint(index, uF, hv, sk),
                self._worldPoint(index, uB, hv, sk),
            )
            # The corners lie off the sketch's plane on purpose: z is kept,
            # not zeroed ([SCREW-F-CELL-LOFT]).
            localPts = [sketch.sketchPoints.add(sketch.modelToSketchSpace(_pt(c3)))
                        for c3 in corners3]
            lines = [sketch.sketchCurves.sketchLines.addByTwoPoints(
                localPts[i], localPts[(i + 1) % 4]) for i in range(4)]
            sectionLines.append(lines)
            allPoints.extend(localPts)

        for pt in allPoints:
            pt.isFixed = True

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')
        if sketch.profiles.count != count + 1:
            raise Exception(
                f'{sketch.name}: sketch has {sketch.profiles.count} profiles, '
                f'want {count + 1}')

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for lines in sectionLines:
            collection = adsk.core.ObjectCollection.create()
            for line in lines:
                collection.add(line)
            path = designComponent.features.createPath(collection, False)
            loftInput.loftSections.add(path)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)

        if loftFeature.bodies.count != 1 or not loftFeature.bodies.item(0).isSolid:
            raise Exception(
                f'{gear["label"]}: cell loft produced {loftFeature.bodies.count} body(ies)')
        return loftFeature.bodies.item(0)

    def _copyBody(self, body):
        designComponent = self.designOcc.component
        copyFeature = designComponent.features.copyPasteBodies.add(body)
        return copyFeature.bodies.item(0)

    def _screwMove(self, index, body, teeth):
        designComponent = self.designOcc.component
        gear = self.gears[index]
        k = float(teeth)

        axisVector = _vec(gear['dir'])
        axisPoint = _pt(gear['origin'])
        rot = adsk.core.Matrix3D.create()
        angle = k * self.P / self.Lambda
        rot.setToRotation(angle, axisVector, axisPoint)

        shift: adsk.core.Vector3D = axisVector.copy()
        distance = to_cm(k * self.P)
        shift.scaleBy(distance)
        mov = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = designComponent.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        designComponent.features.moveFeatures.add(moveInput)

    def _joinBody(self, index, body, toolBody):
        designComponent = self.designOcc.component
        tools = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput = designComponent.features.combineFeatures.createInput(body, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature = designComponent.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise Exception(
                f'{self.gears[index]["label"]}: join produced '
                f'{combineFeature.bodies.count} body(ies)')
        return combineFeature.bodies.item(0)

    def repeatCellByDoubling(self, index, cellBody):
        gear = self.gears[index]
        label = gear['label']
        q, r, c = self.q, self.r, self.c

        body = cellBody
        m = 1
        # floor(log2(q)), without int.bit_length().
        top = 0
        bitsLeft = q
        while bitsLeft > 1:
            bitsLeft >>= 1
            top += 1
        asides = []
        for bit in range(top):
            if (q >> bit) & 1:
                asideBody = self._copyBody(body)
                asides.append((m, asideBody))
            copyBody = self._copyBody(body)
            self._screwMove(index, copyBody, m * c)
            body = self._joinBody(index, body, copyBody)
            m *= 2
        for (asideM, asideBody) in reversed(asides):
            self._screwMove(index, asideBody, m * c)
            body = self._joinBody(index, body, asideBody)
            m += asideM
        if m != q:
            raise Exception(f'{label}: doubling schedule ends with {m} cells, want {q}')

        if r > 0:
            remainderName = f'{label} Cell Remainder'
            remainderBody = self._buildAndLoftCell(index, remainderName, r, q * c * self.n)
            body = self._joinBody(index, body, remainderBody)

        body.name = label
        self.gearBodies[index] = body

    # =======================================================================
    # S16-S23: the cage.
    # =======================================================================

    def buildCage(self):
        designComponent = self.designOcc.component

        # S16: Sleeve sketch.
        sketch = designComponent.sketches.add(self.plane)
        sketch.name = 'Sleeve'
        local = sketch.modelToSketchSpace(_pt(self.C))
        local.z = 0

        innerRadiusCm = to_cm(self.Ri)
        outerRadiusCm = to_cm(self.Ro)
        innerCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(local, innerRadiusCm)
        outerCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(local, outerRadiusCm)
        innerCircle.centerSketchPoint.isFixed = True
        outerCircle.centerSketchPoint.isFixed = True

        innerTextPoint = adsk.core.Point3D.create(local.x + innerRadiusCm, local.y, local.z)
        innerDim = sketch.sketchDimensions.addDiameterDimension(innerCircle, innerTextPoint)
        innerDim.parameter.value = 2 * innerRadiusCm
        outerTextPoint = adsk.core.Point3D.create(local.x + outerRadiusCm, local.y, local.z)
        outerDim = sketch.sketchDimensions.addDiameterDimension(outerCircle, outerTextPoint)
        outerDim.parameter.value = 2 * outerRadiusCm

        if not sketch.isFullyConstrained:
            raise Exception('Sleeve: sketch is not fully constrained')

        matches = []
        for i in range(sketch.profiles.count):
            profile = sketch.profiles.item(i)
            if profile.profileLoops.count == 2:
                matches.append(profile)
        if len(matches) != 1:
            raise Exception(
                f'Sleeve: {len(matches)} profile(s) with 2 loops, want exactly 1 '
                f'(sketch has {sketch.profiles.count} profiles)')
        ring = matches[0]

        # S17: Sleeve extrude.
        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        riseCm = to_cm(self.rise)
        extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(riseCm), False)
        extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise Exception(
                f'Sleeve: extrude produced {extrudeFeature.bodies.count} body(ies), want 1')
        self.cageBody = extrudeFeature.bodies.item(0)

        # S18-S20: the four bores, in order.
        for (index, key) in ((0, 'bore-'), (0, 'bore+'), (1, 'bore-'), (1, 'bore+')):
            self._buildBore(index, key)

        # S21-S23: the windows, if any.
        if self.windows:
            self._buildWindows()

    def _buildBore(self, index, key):
        designComponent = self.designOcc.component
        gear = self.gears[index]
        label = gear['label']
        boreLabel = '-R' if key == 'bore-' else '+R'
        boreLine = self.pathLines[index][key]
        s = -self._sOut if key == 'bore-' else self._sIn

        # S18: Bore plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(boreLine, adsk.core.ValueInput.createByReal(0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{label} Bore {boreLabel} Plane'

        # S19: Bore section sketch.
        sketch = designComponent.sketches.add(borePlane)
        sketch.name = f'{label} Bore {boreLabel}'
        sketch.isComputeDeferred = True

        th = s / self.Lambda + gear['phi']
        hw, ht = self.hw, self.ht
        uB, uF, hv = -hw, hw, ht

        O3 = self._worldPoint(index, 0.0, 0.0, s)
        Cp3 = _add3(O3, _scale3(gear['uHat'], to_cm(self.A / 2.0)))
        E3 = self._worldPoint(index, uF, 0.0, s)

        o = sketch.sketchPoints.add(_localOf(sketch, O3))
        cp = sketch.sketchPoints.add(_localOf(sketch, Cp3))
        ru = sketch.sketchCurves.sketchLines.addByTwoPoints(o, cp)
        ru.isConstruction = True
        k = sketch.sketchCurves.sketchLines.addByTwoPoints(o, _localOf(sketch, E3))
        k.isConstruction = True
        o.isFixed = True
        cp.isFixed = True
        e = k.endSketchPoint

        corner0 = _localOf(sketch, self._worldPoint(index, uB, -hv, s))
        corner1 = _localOf(sketch, self._worldPoint(index, uF, -hv, s))
        corner2 = _localOf(sketch, self._worldPoint(index, uF, hv, s))
        corner3 = _localOf(sketch, self._worldPoint(index, uB, hv, s))
        l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(corner0, corner1)
        l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(l1.endSketchPoint, corner2)
        l3 = sketch.sketchCurves.sketchLines.addByTwoPoints(l2.endSketchPoint, corner3)
        l4 = sketch.sketchCurves.sketchLines.addByTwoPoints(l3.endSketchPoint, l1.startSketchPoint)

        distTextPoint3 = self._worldPoint(index, uF / 2.0, 0.5, s)
        distDim = sketch.sketchDimensions.addDistanceDimension(
            o, e, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _localOf(sketch, distTextPoint3))
        distDim.parameter.value = to_cm(uF)

        uHatTh, vHatTh, _ = self._rotatedBasis(index, s)
        if abs(math.sin(th)) >= math.sqrt(0.5):
            angleTextPoint3 = _add3(
                O3, _scale3(_unit3(_add3(gear['uHat'], uHatTh)), to_cm(hw / 2.0)))
            angleDim = sketch.sketchDimensions.addAngularDimension(
                ru, k, _localOf(sketch, angleTextPoint3))
            angleDim.parameter.value = math.acos(math.cos(th))
        else:
            X3 = _add3(O3, _scale3(gear['uHat'], to_cm(uF / math.cos(th))))
            angleTextPoint3 = _add3(
                X3, _scale3(_unit3(_add3(gear['uHat'], vHatTh)), to_cm(hw / 2.0)))
            angleDim = sketch.sketchDimensions.addAngularDimension(
                ru, l2, _localOf(sketch, angleTextPoint3))
            angleDim.parameter.value = math.acos(-math.sin(th))

        sketch.geometricConstraints.addParallel(l1, k)
        offset1TextPoint3 = self._worldPoint(index, uF / 2.0, -hv / 2.0, s)
        offset1Dim = sketch.sketchDimensions.addOffsetDimension(
            k, l1, _localOf(sketch, offset1TextPoint3))
        offset1Dim.parameter.value = to_cm(hv)

        sketch.geometricConstraints.addParallel(l3, k)
        offset2TextPoint3 = self._worldPoint(index, uF / 2.0, hv / 2.0, s)
        offset2Dim = sketch.sketchDimensions.addOffsetDimension(
            k, l3, _localOf(sketch, offset2TextPoint3))
        offset2Dim.parameter.value = to_cm(hv)

        sketch.geometricConstraints.addCoincident(e, l2)
        sketch.geometricConstraints.addPerpendicular(l2, k)

        sketch.geometricConstraints.addParallel(l4, l2)
        offset3TextPoint3 = self._worldPoint(index, (uF + uB) / 2.0, 0.0, s)
        offset3Dim = sketch.sketchDimensions.addOffsetDimension(
            l2, l4, _localOf(sketch, offset3TextPoint3))
        offset3Dim.parameter.value = to_cm(uF - uB)

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')
        profile = find_profile_by_curve_counts(sketch, lines=4)

        # S20: Bore sweep cut.
        path = designComponent.features.createPath(boreLine, False)
        sweepInput = designComponent.features.sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        twist = (self._sOut - self._sIn) / self.Lambda
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)
        sweepInput.participantBodies = [self.cageBody]
        sweepFeature = designComponent.features.sweepFeatures.add(sweepInput)
        if sweepFeature.bodies.count != 1:
            raise Exception(
                f'{label} Bore {boreLabel}: sweep cut produced '
                f'{sweepFeature.bodies.count} body(ies)')
        self.cageBody = sweepFeature.bodies.item(0)

        sigma = -1.0 if key == 'bore-' else 1.0
        sc = sigma * self.R
        probeOffset = self.W / 2.0 + self.clr / 2.0
        for (probeU, probeTag) in ((probeOffset, '+'), (-probeOffset, '-')):
            probe3 = self._worldPoint(index, probeU, 0.0, sc)
            containment = self.cageBody.pointContainment(_pt(probe3))
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise Exception(
                    f'{label} Bore {boreLabel}: probe {probeTag} reads containment '
                    f'{containment}, want outside')

    def _buildWindows(self):
        designComponent = self.designOcc.component
        sigmaDeg = math.degrees(self.Sigma)

        planeInput = designComponent.constructionPlanes.createInput()
        if sigmaDeg <= 90.0:
            planeInput.setByAngle(
                self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)
        else:
            planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane = designComponent.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'

        for window in self.windows:
            self._buildWindow(windowPlane, window)

    def _buildWindow(self, windowPlane, window):
        designComponent = self.designOcc.component
        d = window['facing']
        name = _faceName(d)
        dWorld = _add3(_scale3(self.eHat, d[0]), _scale3(self.kHat, d[1]))
        acrossWorld = _cross3(self.nHat, dWorld)

        # S22: Window sketch.
        sketch = designComponent.sketches.add(windowPlane)
        sketch.name = f'Window {name}'

        def toWorld(corner):
            t, z = corner
            return _add3(self.C, _add3(_scale3(acrossWorld, to_cm(t)), _scale3(self.nHat, to_cm(z))))

        points = []
        for corner in window['corners']:
            local = sketch.modelToSketchSpace(_pt(toWorld(corner)))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))
        n = len(points)
        for i in range(n):
            sketch.sketchCurves.sketchLines.addByTwoPoints(points[i], points[(i + 1) % n])
        for pt in points:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(
                f'{sketch.name}: sketch has {sketch.profiles.count} profiles, want 1')
        profile = sketch.profiles.item(0)

        # S23: Window extrude cut.
        tc = sum(c[0] for c in window['corners']) / len(window['corners'])
        zc = sum(c[1] for c in window['corners']) / len(window['corners'])
        a0 = math.sqrt(max(0.0, self.Ri * self.Ri - tc * tc))
        a1 = math.sqrt(max(0.0, self.Ro * self.Ro - tc * tc))
        probe3 = _add3(self.C, _add3(
            _scale3(acrossWorld, to_cm(tc)),
            _add3(_scale3(self.nHat, to_cm(zc)), _scale3(dWorld, to_cm((a0 + a1) / 2.0)))))
        containment = self.cageBody.pointContainment(_pt(probe3))
        if containment != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(
                f'Window {name}: probe reads containment {containment}, want inside')

        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        depthCm = to_cm(self.Ro + 1.0)
        depth = adsk.core.ValueInput.createByReal(depthCm)
        extent = adsk.fusion.DistanceExtentDefinition.create(depth)
        testPoint3 = _add3(self.C, dWorld)
        testLocal = sketch.modelToSketchSpace(_pt(testPoint3))
        if testLocal.z > 0:
            direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
        else:
            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeInput.participantBodies = [self.cageBody]
        extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise Exception(
                f'Window {name}: extrude cut produced {extrudeFeature.bodies.count} body(ies)')
        self.cageBody = extrudeFeature.bodies.item(0)

        containment = self.cageBody.pointContainment(_pt(probe3))
        if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(
                f'Window {name}: probe after cut reads containment {containment}, want outside')

    # =======================================================================
    # S24: relocate the bodies and hide construction geometry.
    # =======================================================================

    def relocateBodies(self):
        self.cageBody.name = 'Cage'
        gearBodyA = adsk.fusion.BRepBody.cast(self.gearBodies[0])
        gearBodyA.moveToComponent(self.gearOccs[0])
        gearBodyB = adsk.fusion.BRepBody.cast(self.gearBodies[1])
        gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody.moveToComponent(self.cageOcc)
        solids.hide_construction_geometry(self.designOcc.component)
