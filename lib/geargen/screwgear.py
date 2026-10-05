import math
import adsk.core
import adsk.fusion
from . import base, solids
from .misc import get_design
from .utilities import find_profile_by_curve_counts
from ...lib import fusion360utils as futil


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
TOOTH_SPLINE_POINTS = 11
CELL_TEETH = 4


def _add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _scale(a, k):
    return tuple(v * k for v in a)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def _unit(a):
    length = math.sqrt(_dot(a, a))
    if length <= 0:
        raise ValueError('A frame direction has zero length')
    return _scale(a, 1/length)


def _xyz(p: adsk.core.Point3D):
    return (p.x, p.y, p.z)


def _point(a) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(a[0], a[1], a[2])


def _clip(q, a, b, c):
    out = []
    for i, p in enumerate(q):
        r = q[(i+1) % len(q)]
        fp, fr = c-a*p[0]-b*p[1], c-a*r[0]-b*r[1]
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fr >= 0):
            k = fp/(fp-fr)
            out.append((p[0]+k*(r[0]-p[0]), p[1]+k*(r[1]-p[1])))
    return out


def _gap2(q, x, y):
    inside, best = len(q) >= 3, math.inf
    for i, p in enumerate(q):
        r = q[(i+1) % len(q)]
        ex, ey = r[0]-p[0], r[1]-p[1]
        if ex*(y-p[1])-ey*(x-p[0]) < 0:
            inside = False
        length2 = ex*ex+ey*ey
        k = max(0, min(1, ((x-p[0])*ex+(y-p[1])*ey)/length2)) if length2 > 0 else 0
        dx, dy = x-p[0]-k*ex, y-p[1]-k*ey
        best = min(best, dx*dx+dy*dy)
    return 0 if inside else best


def _hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        raise ValueError(f'Channel outline has only {len(points)} distinct projected points')
    def turn(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    low, high = [], []
    for p in points:
        while len(low) >= 2 and turn(low[-2], low[-1], p) <= 0:
            low.pop()
        low.append(p)
    for p in reversed(points):
        while len(high) >= 2 and turn(high[-2], high[-1], p) <= 0:
            high.pop()
        high.append(p)
    return low[:-1]+high[:-1]


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        plane: adsk.core.SelectionCommandInput = command.commandInputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to")
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        plane.setSelectionLimits(1, 1)
        point: adsk.core.SelectionCommandInput = command.commandInputs.addSelectionInput(
            INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism')
        point.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        point.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        point.setSelectionLimits(1, 1)
        parentInput: adsk.core.SelectionCommandInput = command.commandInputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)
        groups = (
            ('ribbonGroup', 'Ribbon', True, (
                (INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', 1.5),
                (INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', 68),
                (INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', 4.95),
                (INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', 0.375),
                (INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', 0.2625),
                (INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', 0.2625),
                (INPUT_ID_TOOTH_SLANT, 'Tooth Slant', 'deg', math.radians(25.8)),
                (INPUT_ID_TOOTH_BOW, 'Tooth Bow', '', 0.048))),
            ('frameGroup', 'Frame', True, (
                (INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', 1.5),
                (INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', 1.875),
                (INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.02),
                (INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', 0.06),
                (INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 0.3),
                (INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 0.3))),
            ('meshGroup', 'Mesh (from the mesh search)', False, (
                (INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', math.radians(80)),
                (INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 0.105),
                (INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', 0),
                (INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', 0),
                (INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -0.131))))
        for groupId, groupLabel, expanded, rows in groups:
            group: adsk.core.GroupCommandInput = command.commandInputs.addGroupCommandInput(groupId, groupLabel)
            group.isExpanded = expanded
            for inputId, label, unit, default in rows:
                initialValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(default)
                group.children.addValueInput(inputId, label, unit, initialValue)


class ScrewGearGenerator(base.Generator):
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

    def _selection(self, inputs: adsk.core.CommandInputs, name: str) -> adsk.core.Base:
        selectionInput: adsk.core.SelectionCommandInput = adsk.core.SelectionCommandInput.cast(inputs.itemById(name))
        if selectionInput is None:
            raise ValueError(f'Missing input {name}')
        if selectionInput.selectionCount != 1:
            raise ValueError(f'{name} requires exactly one selection; got {selectionInput.selectionCount}')
        return selectionInput.selection(0).entity

    def _value(self, inputs: adsk.core.CommandInputs, name: str, units: str):
        input: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(inputs.itemById(name))
        if input is None:
            raise ValueError(f'Missing input {name}')
        design: adsk.fusion.Design = self.design
        unitsManager: adsk.core.UnitsManager = design.unitsManager
        return unitsManager.evaluateExpression(input.expression, units)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        self.targetPlane = self._selection(inputs, INPUT_ID_PLANE)
        self.selectedPoint = self._selection(inputs, INPUT_ID_POINT)
        parent = self._selection(inputs, INPUT_ID_PARENT)
        occurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(parent)
        self.parentComponent = occurrence.component if occurrence else adsk.fusion.Component.cast(parent)
        if self.parentComponent is None:
            raise ValueError('parent must be an occurrence or root component')
        lengths = (INPUT_ID_RIBBON_WIDTH, INPUT_ID_TWIST_LEAD, INPUT_ID_RIBBON_THICKNESS,
                   INPUT_ID_TOOTH_PITCH, INPUT_ID_TOOTH_HEIGHT, INPUT_ID_CAGE_RADIUS, INPUT_ID_CAGE_RISE,
                   INPUT_ID_CLEARANCE, INPUT_ID_ROOF_ALLOWANCE, INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL,
                   INPUT_ID_ENGAGEMENT, INPUT_ID_ASSEMBLY_PHASE)
        self.values = {name: self._value(inputs, name, 'mm') for name in lengths}
        for name in (INPUT_ID_TOOTH_SLANT, INPUT_ID_CROSS_ANGLE, INPUT_ID_MOUNT_ANGLE_A, INPUT_ID_MOUNT_ANGLE_B):
            self.values[name] = self._value(inputs, name, 'deg')
        for name in (INPUT_ID_TOOTH_COUNT, INPUT_ID_TOOTH_BOW):
            self.values[name] = self._value(inputs, name, '')
        p = self.values
        for name in (INPUT_ID_RIBBON_WIDTH, INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH,
                     INPUT_ID_TWIST_LEAD, INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CLEARANCE):
            if p[name] <= 0:
                raise ValueError(f'{name} must be > 0')
        if p['roofAllowance'] < 0:
            raise ValueError('roofAllowance must be >= 0')
        if p['toothCount'] < 4 or int(p['toothCount']) != p['toothCount']:
            raise ValueError('toothCount must be a whole number >= 4')
        if not 0 < p['toothHeight'] < p['ribbonWidth']/2:
            raise ValueError('toothHeight must be > 0 and < ribbonWidth/2')
        if not -math.pi/2 < p['toothSlant'] < math.pi/2:
            raise ValueError('toothSlant must lie strictly between -90 and 90 degrees')
        if p['toothBow'] < 0 or p['toothHeight']+10*p['toothBow']*(p['ribbonThickness']/2)**2 >= p['ribbonWidth']/2:
            raise ValueError('toothBow must be >= 0 and toothHeight+toothBow*(ribbonThickness/2)^2 < ribbonWidth/2')
        if not 0 < p['engagement'] <= p['toothHeight']:
            raise ValueError('engagement must be > 0 and <= toothHeight')
        if not 0 < p['crossAngle'] < math.pi:
            raise ValueError('crossAngle must lie strictly between 0 and 180 degrees')
        if not -p['toothPitch'] < p['assemblyPhase'] < p['toothPitch']:
            raise ValueError('assemblyPhase must lie strictly within +/-toothPitch')
        self.W, self.T, self.P, self.H = p['ribbonWidth'], p['ribbonThickness'], p['toothPitch'], p['toothHeight']
        self.N = int(p['toothCount'])
        self.lam = p['twistLead']/(2*math.pi)
        self.A = self.W-p['engagement']
        self.L = self.N*self.P
        self.Sigma = p['crossAngle']
        self.mounts = [p['mountAngleA'], p['mountAngleB']]
        self.phases = [0, p['assemblyPhase']]
        self.slant, self.bow = p['toothSlant'], 10*p['toothBow']
        self.cageRadius, self.cageRise = p['cageRadius'], p['cageRise']
        self.collarHalf, self.collarWall = p['collarHalf'], p['collarWall']
        self.clearance, self.roofAllowance = p['clearance'], p['roofAllowance']
        self.Ri, self.Ro = self.cageRadius-self.collarHalf, self.cageRadius+self.collarHalf
        self.hw, self.ht = self.W/2+self.clearance, self.T/2+self.clearance
        self.corner = math.hypot(self.hw, self.ht+self.roofAllowance)
        if self.Ro+0.1 >= self.L/2:
            raise ValueError('cageRadius+collarHalf+1 mm must be < toothCount*toothPitch/2')
        if math.hypot(self.corner, 0.1) >= self.Ri:
            raise ValueError('cageRadius must give hypot(boreCorner, 1 mm) < cageRadius-collarHalf')
        axialWindow = 1.5*math.sqrt(self.W*self.W-self.A*self.A)/math.sin(self.Sigma)
        if math.hypot(axialWindow, math.hypot(self.W/2, self.T/2))+self.clearance > self.Ri:
            raise ValueError('cageRadius must keep the axial mesh footprint plus clearance <= cageRadius-collarHalf')
        if self.cageRise < self.A/2+self.corner+self.collarWall:
            raise ValueError('cageRise must be >= axisOffset/2+boreCorner+collarWall')
        self.sIn, self.sOut = math.sqrt(self.Ri*self.Ri-self.corner*self.corner)-0.1, self.Ro+0.1
        self.cellTeeth = min(CELL_TEETH, self.N)
        self.q, self.r = self.N//self.cellTeeth, self.N % self.cellTeeth
        self.stepsPerTooth = max(8, math.ceil((self.P/self.lam)/math.radians(2)))
        self.levelBores = [self._levelBore(g) for g in range(2)]
        self._precomputeSearch()
        self.designOcc = adsk.fusion.Occurrence.cast(None)
        self.gearOccs = [adsk.fusion.Occurrence.cast(None)]*2
        self.cageOcc = adsk.fusion.Occurrence.cast(None)
        self.gearBodies = [adsk.fusion.BRepBody.cast(None)]*2
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]

    def _levelBore(self, g):
        tilts = []
        for sigma in (-1, 1):
            lo = sigma*(self.cageRadius-self.collarHalf)/self.lam+self.mounts[g]
            hi = sigma*(self.cageRadius+self.collarHalf)/self.lam+self.mounts[g]
            lo, hi = min(lo, hi), max(lo, hi)
            level = math.pi/2+math.ceil((lo-math.pi/2)/math.pi)*math.pi
            tilts.append(0 if level <= hi else min(abs(math.cos(lo)), abs(math.cos(hi))))
        return 1 if tilts[1] < tilts[0] else -1

    def _roofSign(self, g, sigma):
        return 1 if -math.sin(sigma*self.cageRadius/self.lam+self.mounts[g])*(1 if g == 0 else -1) > 0 else -1

    def _opening(self, g, sigma, scale=1):
        lo, hi = -self.ht, self.ht
        if sigma == self.levelBores[g]:
            if self._roofSign(g, sigma) > 0:
                hi += self.roofAllowance
            else:
                lo -= self.roofAllowance
        return self.hw*scale, lo*scale, hi*scale

    def _precomputeSearch(self):
        self.mmRi, self.mmRo, self.mmRise = 10*self.Ri, 10*self.Ro, 10*self.cageRise
        self.mmIn, self.mmOut = 10*self.sIn, 10*self.sOut
        self.mmCorner, self.mmWall, self.mmLam = 10*self.corner, 10*self.collarWall, 10*self.lam
        self.searchFrames = []
        for g in range(2):
            angle = self.Sigma/2 if g == 0 else -self.Sigma/2
            direction = (math.cos(angle), math.sin(angle), 0)
            u = (0, 0, 1 if g == 0 else -1)
            origin = (0, 0, -5*self.A if g == 0 else 5*self.A)
            self.searchFrames.append((origin, direction, u, _cross(direction, u)))
        self.bores = [(0, -1), (0, 1), (1, -1), (1, 1)]
        outlines = [self._channelOutline(b) for b in self.bores]
        gaps = (((0, 1), (1, -1), (0, 1, 0), '+k'),
                ((0, -1), (1, -1), (-1, 0, 0), '-e'),
                ((0, -1), (1, 1), (0, -1, 0), '-k'),
                ((0, 1), (1, 1), (1, 0, 0), '+e'))
        for first, second, facing, name in gaps:
            across = _cross((0, 0, 1), facing)
            hulls = []
            for b in (first, second):
                outline = outlines[self.bores.index(b)]
                hulls.append(_hull([(_dot(p, across), p[2]) for p in outline]))
            separation = -math.inf
            for hull in hulls:
                for i, p in enumerate(hull):
                    r = hull[(i+1) % len(hull)]
                    length = math.hypot(r[0]-p[0], r[1]-p[1])
                    m = (-(r[1]-p[1])/length, (r[0]-p[0])/length)
                    a = [m[0]*q[0]+m[1]*q[1] for q in hulls[0]]
                    b = [m[0]*q[0]+m[1]*q[1] for q in hulls[1]]
                    separation = max(separation, min(b)-max(a), min(a)-max(b))
            if separation < self.mmWall:
                raise ValueError(f'collarWall requires >= {self.mmWall} mm between {first} and {second}; '
                                 f'{name} separation is {separation} mm')
        self.stationTables = {b: self._channelSections(b) for b in self.bores}
        self.zLimit = self._channelTop()
        facings = (((0, 1, 0), '+k'), ((0, -1, 0), '-k')) if self.Sigma <= math.pi/2 else (
            ((1, 0, 0), '+e'), ((-1, 0, 0), '-e'))
        self.windows = []
        for facing, name in facings:
            window = self._newWindow(facing, name)
            if window is not None:
                self.windows.append(window)

    def _searchWorld(self, g, u, v, s):
        origin, direction, ex, ey = self.searchFrames[g]
        theta = s/self.mmLam+self.mounts[g]
        x, y = u*math.cos(theta)-v*math.sin(theta), u*math.sin(theta)+v*math.cos(theta)
        return _add(_add(origin, _scale(direction, s)), _add(_scale(ex, x), _scale(ey, y)))

    def _span(self, sigma, mm=False):
        si, so = (self.mmIn, self.mmOut) if mm else (self.sIn, self.sOut)
        return (si, so) if sigma > 0 else (-so, -si)

    def _channelOutline(self, bore):
        g, sigma = bore
        hw, lo, hi = self._opening(g, sigma, 10)
        points = []
        k = 0
        while self.mmIn+k*0.1 <= self.mmOut:
            s = sigma*(self.mmIn+k*0.1)
            for i in range(17):
                v = lo+(hi-lo)*i/16
                u = -hw+2*hw*i/16
                for x, y in ((hw, v), (-hw, v), (u, hi), (u, lo)):
                    p = self._searchWorld(g, x, y, s)
                    if self.mmRi-0.5 <= math.hypot(p[0], p[1]) <= self.mmRo+0.5:
                        points.append(p)
            k += 1
        if not points:
            raise ValueError(f'collarWall channel outline is empty for {bore}')
        return points

    def _sectionInWall(self, g, s):
        if abs(s) >= self.mmRo:
            return ([], [])
        hw, lo, hi = self._opening(g, -1 if s < 0 else 1, 10)
        theta = s/self.mmLam+self.mounts[g]
        c, sn = math.cos(theta), math.sin(theta)
        rect = [(u*c-v*sn, u*sn+v*c) for u, v in ((-hw, lo), (hw, lo), (hw, hi), (-hw, hi))]
        origin, direction, ex, ey = self.searchFrames[g]
        xa, xb = (-self.mmRise-origin[2])/ex[2], (self.mmRise-origin[2])/ex[2]
        xa, xb = min(xa, xb), max(xa, xb)
        rect = _clip(_clip(rect, 1, 0, xb), -1, 0, -xa)
        near = math.sqrt(max(0, self.mmRi*self.mmRi-s*s))
        far = math.sqrt(self.mmRo*self.mmRo-s*s)
        return tuple(_clip(_clip(rect, 0, side, far), 0, -side, -near) for side in (-1, 1))

    def _makeStation(self, bore, s):
        pieces = self._sectionInWall(bore[0], s)
        circles = []
        for piece in pieces:
            if not piece:
                circles.append((0, 0, 0))
                continue
            cx = sum(p[0]/len(piece) for p in piece)
            cy = sum(p[1]/len(piece) for p in piece)
            radius = max(math.hypot(p[0]-cx, p[1]-cy) for p in piece)
            circles.append((cx, cy, radius))
        return (s, pieces, circles)

    def _channelSections(self, bore):
        lo, hi = self._span(bore[1], True)
        out = []
        k = 0
        while lo+k*0.002 <= hi:
            s = lo+k*0.002
            station = self._makeStation(bore, s)
            if out and tuple(bool(p) for p in station[1]) != tuple(bool(p) for p in out[-1][1]):
                previous = out[-1][0]
                j = 1
                while previous+j*0.0001 < s:
                    out.append(self._makeStation(bore, previous+j*0.0001))
                    j += 1
            out.append(station)
            k += 1
        return out

    def _wallGap(self, bore, point, reach):
        g, sigma = bore
        origin, direction, ex, ey = self.searchFrames[g]
        d = _sub(point, origin)
        x, y, sq = _dot(d, ex), _dot(d, ey), _dot(d, direction)
        lo, hi = self._span(sigma, True)
        axis = math.hypot(math.hypot(x, y), sq-max(lo, min(hi, sq)))
        if axis-self.mmCorner >= reach:
            return reach
        best = reach*reach
        table = self.stationTables[bore]
        lower, upper = 0, len(table)
        while lower < upper:
            middle = (lower+upper)//2
            if table[middle][0] < sq:
                lower = middle+1
            else:
                upper = middle
        for start, stop, step in ((lower, len(table), 1), (lower-1, -1, -1)):
            for i in range(start, stop, step):
                s, pieces, circles = table[i]
                ds = (sq-s)**2
                if ds >= best:
                    break
                for piece, (cx, cy, radius) in zip(pieces, circles):
                    if not piece:
                        continue
                    offset = math.hypot(x-cx, y-cy)-radius
                    if offset > 0 and ds+offset*offset >= best:
                        continue
                    best = min(best, ds+_gap2(piece, x, y))
        return math.sqrt(best)

    def _wallCorners(self, bore):
        g, sigma = bore
        origin, direction, ex, ey = self.searchFrames[g]
        lo, hi = self._span(sigma, True)
        k = 0
        while lo+k*0.001 <= hi:
            s = lo+k*0.001
            for piece in self._sectionInWall(g, s):
                for x, y in piece:
                    yield _add(_add(origin, _scale(direction, s)), _add(_scale(ex, x), _scale(ey, y)))
            k += 1

    def _channelTop(self):
        top = 0
        for g, sigma in self.bores:
            hw, lo, hi = self._opening(g, sigma, 10)
            origin, direction, ex, ey = self.searchFrames[g]
            k = 0
            while self.mmIn+k*0.01 <= self.mmOut:
                s = sigma*(self.mmIn+k*0.01)
                theta = s/self.mmLam+self.mounts[g]
                c, sn = math.cos(theta), math.sin(theta)
                corners = [(u*c-v*sn, u*sn+v*c) for u, v in ((-hw, lo), (hw, lo), (hw, hi), (-hw, hi))]
                y = max(abs(q[1]) for q in corners)
                if abs(s) <= self.mmRo and math.hypot(s, y) >= self.mmRi:
                    top = max(top, max(abs(origin[2]+q[0]*ex[2]) for q in corners))
                k += 1
        return top

    def _crossing(self, bore):
        origin, direction, ex, ey = self.searchFrames[bore[0]]
        return _add(origin, _scale(direction, bore[1]*10*self.cageRadius))

    def _chord(self, t):
        return math.sqrt(max(0, self.mmRi*self.mmRi-t*t)), math.sqrt(max(0, self.mmRo*self.mmRo-t*t))

    def _windowAt(self, w, t, z, a):
        return _add(_add(_scale(w['across'], t), _scale(w['facing'], a)), (0, 0, z))

    def _windowClear(self, w, far, side, te):
        t = side*te
        zLow = max(w['lo']-w['lean']*t, w['bottom']+w['lean']*t)
        zHigh = min(w['hi']-w['lean']*t, w['top']+w['lean']*t)
        if zLow > zHigh:
            return False
        need = self.mmWall+0.1/math.sqrt(2)+0.005
        a0, a1 = self._chord(t)
        for z in (zLow, zHigh):
            a = a0
            while True:
                p = self._windowAt(w, t, z, a)
                if any(self._wallGap(b, p, need) < need for b in far):
                    return False
                if a >= a1:
                    break
                a = min(a+0.1, a1)
        n = math.ceil((zHigh-zLow)/0.1)
        for a in (a0, a1):
            for k in range(1, n):
                p = self._windowAt(w, t, zLow+(zHigh-zLow)*k/n, a)
                if any(self._wallGap(b, p, need) < need for b in far):
                    return False
            j = 0
            while -self.zLimit+j*0.1 <= self.zLimit:
                z = -self.zLimit+j*0.1
                if zLow < z < zHigh:
                    p = self._windowAt(w, t, z, a)
                    if any(self._wallGap(b, p, need) < need for b in far):
                        return False
                j += 1
        return True

    def _windowEnd(self, w, far, side):
        lo, hi = 0, min(self.mmRi, self.mmRo/math.sqrt(2))*(1-1e-9)
        for unused in range(24):
            middle = (lo+hi)/2
            if self._windowClear(w, far, side, middle):
                lo = middle
            else:
                hi = middle
        return lo

    def _newWindow(self, facing, name):
        w = {'facing': facing, 'across': _cross((0, 0, 1), facing), 'name': name}
        flanks = [b for b in self.bores if _dot(self._crossing(b), facing) > 0]
        far = [b for b in self.bores if b not in flanks]
        if len(flanks) != 2:
            raise ValueError(f'Window {name} has {len(flanks)} flanking bores, expected two')
        low, high = sorted(flanks, key=lambda b: self._crossing(b)[2])
        w['lean'] = -1 if _dot(self._crossing(high), w['across']) < _dot(self._crossing(low), w['across']) else 1
        lowReach = max(p[2]+w['lean']*_dot(p, w['across']) for p in self._wallCorners(low))
        highReach = min(p[2]+w['lean']*_dot(p, w['across']) for p in self._wallCorners(high))
        w['lo'], w['hi'] = lowReach+math.sqrt(2)*self.mmWall, highReach-math.sqrt(2)*self.mmWall
        w['top'] = min(2*self.zLimit-w['hi'], w['hi']+math.sqrt(2)*self.mmRi)
        w['bottom'] = max(-2*self.zLimit-w['lo'], w['lo']-math.sqrt(2)*self.mmRi)
        if w['hi'] <= w['lo']:
            futil.log(f'No window facing {name}: the flanking bores leave no band between them')
            return None
        w['right'], w['left'] = self._windowEnd(w, far, 1), -self._windowEnd(w, far, -1)
        q = [(-2*self.mmRo, -2*self.mmRo), (2*self.mmRo, -2*self.mmRo),
             (2*self.mmRo, 2*self.mmRo), (-2*self.mmRo, 2*self.mmRo)]
        for a, b, c in ((w['lean'], 1, w['hi']), (-w['lean'], -1, -w['lo']),
                        (-w['lean'], 1, w['top']), (w['lean'], -1, -w['bottom']),
                        (1, 0, w['right']), (-1, 0, -w['left'])):
            q = _clip(q, a, b, c)
        corners = []
        for p in q:
            if not corners or math.hypot(p[0]-corners[-1][0], p[1]-corners[-1][1]) >= 0.001:
                corners.append(p)
        if len(corners) > 1 and math.hypot(corners[0][0]-corners[-1][0], corners[0][1]-corners[-1][1]) < 0.001:
            corners.pop()
        area = sum(p[0]*corners[(i+1) % len(corners)][1]-corners[(i+1) % len(corners)][0]*p[1]
                   for i, p in enumerate(corners))/2
        if w['right'] <= w['left'] or len(corners) < 3 or area <= 0:
            futil.log(f'No window facing {name}: the far bores leave the band no length')
            return None
        w['corners'] = [(t/10, z/10) for t, z in corners]
        return w

    def buildComponentTree(self):
        topComponent: adsk.fusion.Component = self.getOccurrence().component
        topComponent.name = 'Screw Gearing'
        self.designOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.designOcc.component.name = 'Design'
        for index, label in enumerate(('Gear A', 'Gear B')):
            occurrence: adsk.fusion.Occurrence = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occurrence.component.name = label
            self.gearOccs[index] = occurrence
        self.cageOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.cageOcc.component.name = 'Cage'

    def _requireSketch(self, sketch: adsk.fusion.Sketch, count=None):
        if not sketch.isFullyConstrained:
            raise ValueError(f'{sketch.name} isFullyConstrained={sketch.isFullyConstrained}')
        if count is not None and sketch.profiles.count != count:
            raise ValueError(f'{sketch.name} has {sketch.profiles.count} profiles; expected {count}')

    def buildAnchor(self):
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(self.targetPlane)
        sketch.name = 'Anchor'
        projected: adsk.core.ObjectCollection = sketch.project(self.selectedPoint)
        if projected.count != 1:
            raise ValueError(f'Anchor projection returned {projected.count} entities; expected one')
        projectedPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projected.item(0))
        if projectedPoint is None:
            raise ValueError('Anchor projection did not return a SketchPoint')
        centre: adsk.core.Point3D = projectedPoint.geometry
        startSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre.x-0.5, centre.y, 0)
        endSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+0.5, centre.y, 0)
        self.anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)
        anchorLine: adsk.fusion.SketchLine = self.anchorLine
        anchorLine.isConstruction = True
        sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)
        sketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x, centre.y+0.25, 0)
        dimension: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dimension.parameter.value = 1
        self._requireSketch(sketch)
        self.C = _xyz(projectedPoint.worldGeometry)
        self.eHat = _unit(_sub(_xyz(anchorLine.endSketchPoint.worldGeometry), _xyz(anchorLine.startSketchPoint.worldGeometry)))
        self.axisPlanes = []
        for g, label in enumerate(('Gear A', 'Gear B')):
            planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
            offsetValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(-self.A/2 if g == 0 else self.A/2)
            planeInput.setByOffset(self.targetPlane, offsetValue)
            plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
            plane.name = label+' Axis Plane'
            self.axisPlanes.append(plane)
        gearA: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[0])
        geometry: adsk.core.Plane = gearA.geometry
        normal: adsk.core.Vector3D = geometry.normal
        self.nHat = _unit((normal.x, normal.y, normal.z))
        if _dot(_sub(self.C, _xyz(geometry.origin)), self.nHat) < 0:
            self.nHat = _scale(self.nHat, -1)
        for g in range(2):
            plane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[g])
            distance = _dot(_sub(_xyz(plane.geometry.origin), self.C), self.nHat)
            expected = -self.A/2 if g == 0 else self.A/2
            if abs(distance-expected) > 0.0001:
                raise ValueError(f'{plane.name} signed distance {distance} cm; expected {expected} cm')
        self.kHat = _unit(_cross(self.nHat, self.eHat))
        self.dirVecs, self.origins, self.uVecs, self.vVecs = [], [], [], []
        for g in range(2):
            angle = self.Sigma/2 if g == 0 else -self.Sigma/2
            direction = _add(_scale(self.eHat, math.cos(angle)), _scale(self.kHat, math.sin(angle)))
            u = self.nHat if g == 0 else _scale(self.nHat, -1)
            self.dirVecs.append(direction)
            self.origins.append(_add(self.C, _scale(self.nHat, -self.A/2 if g == 0 else self.A/2)))
            self.uVecs.append(u)
            self.vVecs.append(_cross(direction, u))

    def _world(self, g, u, v, station) -> adsk.core.Point3D:
        theta = station/self.lam+self.mounts[g]
        x, y = u*math.cos(theta)-v*math.sin(theta), u*math.sin(theta)+v*math.cos(theta)
        return _point(_add(_add(self.origins[g], _scale(self.dirVecs[g], station)),
                           _add(_scale(self.uVecs[g], x), _scale(self.vVecs[g], y))))

    def _mapped(self, sketch: adsk.fusion.Sketch, world: adsk.core.Point3D, planar=True) -> adsk.core.Point3D:
        local: adsk.core.Point3D = sketch.modelToSketchSpace(world)
        if planar:
            local.z = 0
        return local

    def _sketchPoint(self, sketch: adsk.fusion.Sketch, world: adsk.core.Point3D, planar=True) -> adsk.fusion.SketchPoint:
        local: adsk.core.Point3D = self._mapped(sketch, world, planar)
        return sketch.sketchPoints.add(local)

    def buildGear(self, index):
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, index):
        design: adsk.fusion.Component = self.designOcc.component
        plane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[index])
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = ('Gear A', 'Gear B')[index]+' Paths'
        points = []
        for key, sigma in (('bore-', -1), ('bore+', 1)):
            start, end = self._span(sigma)
            startPoint: adsk.fusion.SketchPoint = self._sketchPoint(sketch, self._world(index, 0, 0, start))
            endPoint: adsk.fusion.SketchPoint = self._sketchPoint(sketch, self._world(index, 0, 0, end))
            line: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
            self.pathLines[index][key] = line
            points.extend((startPoint, endPoint))
        for item in points:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(item)
            point.isFixed = True
        self._requireSketch(sketch)

    def _edge(self, g, v, s, slant=None):
        lean = self.slant if slant is None else slant
        return self.W/2-self.H/2+self.H/2*math.cos(2*math.pi*(s+math.tan(lean)*v-self.phases[g])/self.P)-self.bow*v*v

    def buildToothCell(self, index):
        body: adsk.fusion.BRepBody = self._cell(index, self.phases[index]-self.L/2, self.cellTeeth, 'Cell Sections')
        self.gearBodies[index] = body

    def buildCage(self):
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(self.targetPlane)
        sketch.name = 'Sleeve'
        centre: adsk.core.Point3D = self._mapped(sketch, _point(self.C))
        for radius in (self.Ri, self.Ro):
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+radius, centre.y, 0)
            dimension: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2*radius
        self._requireSketch(sketch)
        rings = []
        for item in sketch.profiles:
            profile: adsk.fusion.Profile = adsk.fusion.Profile.cast(item)
            if profile.profileLoops.count == 2:
                rings.append(profile)
        if len(rings) != 1:
            raise ValueError(f'Sleeve has {len(rings)} two-loop annular profiles; expected one')
        ringProfile: adsk.fusion.Profile = adsk.fusion.Profile.cast(rings[0])
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        riseValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.cageRise)
        extrudeInput.setSymmetricExtent(riseValue, False)
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise ValueError(f'Sleeve extrude returned {extrudeFeature.bodies.count} bodies; expected one')
        self.cageBody = extrudeFeature.bodies.item(0)
        for g, sigma in self.bores:
            self._cutBore(g, sigma)
        if self.windows:
            planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
            anchorLine: adsk.fusion.SketchLine = self.anchorLine
            if self.Sigma <= math.pi/2:
                rightAngle: adsk.core.ValueInput = adsk.core.ValueInput.createByString('90 deg')
                planeInput.setByAngle(anchorLine, rightAngle, self.targetPlane)
            else:
                midpointFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0.5)
                planeInput.setByDistanceOnPath(anchorLine, midpointFraction)
            plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
            plane.name = 'Window Plane'
            for window in self.windows:
                self._cutWindow(plane, window)
        self._markers()
        futil.log('Print the cage standing on its end below the selected plane: '
                  'the roof allowance is on the bridged roofs that way up.')

    def _boreSketch(self, g, sigma, plane: adsk.fusion.ConstructionPlane) -> adsk.fusion.Sketch:
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = ('Gear A', 'Gear B')[g]+' Bore '+('-R' if sigma < 0 else '+R')
        sketch.isComputeDeferred = True
        s, unused = self._span(sigma)
        theta = s/self.lam+self.mounts[g]
        hw, vLo, vHi = self._opening(g, sigma)
        origin: adsk.core.Point3D = self._world(g, 0, 0, s)
        cpWorld: adsk.core.Point3D = _point(_add(_xyz(origin), _scale(self.uVecs[g], self.A/2)))
        O: adsk.fusion.SketchPoint = self._sketchPoint(sketch, origin)
        Cp: adsk.fusion.SketchPoint = self._sketchPoint(sketch, cpWorld)
        Ru: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True
        E: adsk.fusion.SketchPoint = self._sketchPoint(sketch, self._world(g, hw, 0, s))
        K: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, E)
        K.isConstruction = True
        expected = [self._mapped(sketch, self._world(g, u, v, s))
                    for u, v in ((-hw, vLo), (hw, vLo), (hw, vHi), (-hw, vHi))]
        corners = [sketch.sketchPoints.add(p) for p in expected]
        P0: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[0])
        P1: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[1])
        P2: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[2])
        P3: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[3])
        L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P0, P1)
        L2: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P1, P2)
        L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P2, P3)
        L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P3, P0)
        O.isFixed, Cp.isFixed = True, True
        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addParallel(L4, L2)
        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        lengthText: adsk.core.Point3D = self._mapped(sketch, self._world(g, hw/2, -self.ht/2, s))
        lengthDim: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDim.parameter.value = hw
        angle = theta if abs(math.sin(theta)) >= math.sqrt(0.5) else theta+math.pi/2
        ray = _add(_scale(self.uVecs[g], math.cos(angle)), _scale(self.vVecs[g], math.sin(angle)))
        reference = self.uVecs[g]
        cosine = max(-1, min(1, _dot(reference, ray)))
        if cosine < 0:
            ray = _scale(ray, -1)
            cosine = -cosine
        measured = math.acos(cosine)
        if measured < math.pi/4:
            reference = _scale(reference, -1)
            measured = math.pi-measured
        angleText: adsk.core.Point3D = self._mapped(
            sketch, _point(_add(_xyz(origin), _scale(_add(reference, ray), self.A/4))))
        if abs(math.sin(theta)) >= math.sqrt(0.5):
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)
        else:
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)
        angleDim.parameter.value = measured
        lowerText: adsk.core.Point3D = self._mapped(sketch, self._world(g, 0, vLo/2, s))
        upperText: adsk.core.Point3D = self._mapped(sketch, self._world(g, 0, vHi/2, s))
        widthText: adsk.core.Point3D = self._mapped(sketch, self._world(g, 0, (vHi+vLo)/2, s))
        lowerDim: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)
        lowerDim.parameter.value = -vLo
        upperDim: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)
        upperDim.parameter.value = vHi
        widthDim: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)
        widthDim.parameter.value = 2*hw
        sketch.isComputeDeferred = False
        self._requireSketch(sketch, 1)
        for i, item in enumerate(corners):
            corner: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(item)
            actual: adsk.core.Point3D = corner.geometry
            target: adsk.core.Point3D = expected[i]
            distance = math.sqrt((actual.x-target.x)**2+(actual.y-target.y)**2+(actual.z-target.z)**2)
            if distance > 0.0001:
                raise ValueError(f'{sketch.name} corner {i} moved {distance*10} mm; limit 0.001 mm')
        return sketch

    def _cutBore(self, g, sigma):
        design: adsk.fusion.Component = self.designOcc.component
        boreLine: adsk.fusion.SketchLine = adsk.fusion.SketchLine.cast(self.pathLines[g]['bore-' if sigma < 0 else 'bore+'])
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        startFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0)
        planeInput.setByDistanceOnPath(boreLine, startFraction)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = ('Gear A', 'Gear B')[g]+' Bore '+('-R' if sigma < 0 else '+R')+' Plane'
        sketch: adsk.fusion.Sketch = self._boreSketch(g, sigma, plane)
        profile: adsk.fusion.Profile = find_profile_by_curve_counts(sketch, lines=4)
        borePath: adsk.fusion.Path = design.features.createPath(boreLine, False)
        sweepInput: adsk.fusion.SweepFeatureInput = design.features.sweepFeatures.createInput(
            profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal((self.sOut-self.sIn)/self.lam)
        cageBody: adsk.fusion.BRepBody = self.cageBody
        sweepInput.participantBodies = [cageBody]
        sweepFeature: adsk.fusion.SweepFeature = design.features.sweepFeatures.add(sweepInput)
        if sweepFeature.bodies.count != 1:
            raise ValueError(f'{sketch.name} sweep returned {sweepFeature.bodies.count} bodies; expected one')
        self.cageBody = sweepFeature.bodies.item(0)
        cageBody = self.cageBody
        sc = sigma*self.cageRadius
        for side in (-1, 1):
            probePoint: adsk.core.Point3D = self._world(g, side*(self.W/2+self.clearance/2), 0, sc)
            observed = cageBody.pointContainment(probePoint)
            if observed != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise ValueError(f'{sketch.name} u-side {side} containment {observed}; expected outside')
        if sigma == self.levelBores[g] and self.roofAllowance > 0:
            roofSign = self._roofSign(g, sigma)
            for side, label, expected in ((roofSign, 'roof', adsk.fusion.PointContainment.PointOutsidePointContainment),
                                          (-roofSign, 'floor', adsk.fusion.PointContainment.PointInsidePointContainment)):
                probePoint: adsk.core.Point3D = self._world(g, 0, side*(self.ht+self.roofAllowance/2), sc)
                observed = cageBody.pointContainment(probePoint)
                if observed != expected:
                    raise ValueError(f'{sketch.name} {label} containment {observed}; expected {expected}')

    def _canonicalVector(self, a):
        return _add(_add(_scale(self.eHat, a[0]), _scale(self.kHat, a[1])), _scale(self.nHat, a[2]))

    def _polygon(self, sketch: adsk.fusion.Sketch, worldPoints):
        points = [self._sketchPoint(sketch, world) for world in worldPoints]
        for i, item in enumerate(points):
            startPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(item)
            endPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(points[(i+1) % len(points)])
            sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
        for item in points:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(item)
            point.isFixed = True
        self._requireSketch(sketch, 1)

    def _cutWindow(self, plane: adsk.fusion.ConstructionPlane, window):
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = 'Window '+window['name']
        across, facing = self._canonicalVector(window['across']), self._canonicalVector(window['facing'])
        self._polygon(sketch, [_point(_add(self.C, _add(_scale(across, t), _scale(self.nHat, z))))
                               for t, z in window['corners']])
        profile: adsk.fusion.Profile = sketch.profiles.item(0)
        tc = sum(p[0] for p in window['corners'])/len(window['corners'])
        zc = sum(p[1] for p in window['corners'])/len(window['corners'])
        a0, a1 = self._chord(tc*10)
        probePoint: adsk.core.Point3D = _point(_add(self.C, _add(_scale(across, tc),
            _add(_scale(self.nHat, zc), _scale(facing, (a0+a1)/20)))))
        cageBody: adsk.fusion.BRepBody = self.cageBody
        before = cageBody.pointContainment(probePoint)
        if before != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise ValueError(f'{sketch.name} probe before cut containment {before}; expected inside')
        directionPoint: adsk.core.Point3D = _point(_add(self.C, facing))
        local: adsk.core.Point3D = sketch.modelToSketchSpace(directionPoint)
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if local.z > 0 else
                     adsk.fusion.ExtentDirections.NegativeExtentDirection)
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.Ro+0.1)
        extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeInput.participantBodies = [cageBody]
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        if extrudeFeature.bodies.count != 1:
            raise ValueError(f'{sketch.name} cut returned {extrudeFeature.bodies.count} bodies; expected one')
        self.cageBody = extrudeFeature.bodies.item(0)
        cageBody = self.cageBody
        after = cageBody.pointContainment(probePoint)
        if after != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise ValueError(f'{sketch.name} probe after cut containment {after}; expected outside')

    def _markers(self):
        design: adsk.fusion.Component = self.designOcc.component
        halfSize, inset = min(0.1, self.collarHalf/2), min(0.01, self.collarWall/2)
        gearBAxisPlane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[1])
        normal: adsk.core.Vector3D = gearBAxisPlane.geometry.normal
        sign = 1 if _dot((normal.x, normal.y, normal.z), self.nHat) > 0 else -1
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        signedOffset: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(sign*(self.cageRise-inset-self.A/2))
        planeInput.setByOffset(gearBAxisPlane, signedOffset)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = 'Marker Plane'
        offset = _dot(_sub(_xyz(plane.geometry.origin), self.C), self.nHat)
        if abs(offset-(self.cageRise-inset)) > 0.0001:
            raise ValueError(f'Marker Plane offset {offset} cm; expected {self.cageRise-inset} cm')
        for g, sigma in self.bores:
            centre = _add(self.C, _add(_scale(self.nHat, self.cageRise-inset),
                                      _scale(self.dirVecs[g], sigma*self.cageRadius)))
            sketch: adsk.fusion.Sketch = design.sketches.add(plane)
            sketch.name = ('Gear A', 'Gear B')[g]+' Bore '+('-R Square Marker' if sigma < 0 else '+R Circle Marker')
            if sigma > 0:
                localCentre: adsk.core.Point3D = self._mapped(sketch, _point(centre))
                circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, halfSize)
                circle.centerSketchPoint.isFixed = True
                textPoint: adsk.core.Point3D = adsk.core.Point3D.create(localCentre.x+halfSize, localCentre.y, 0)
                dimension: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
                dimension.parameter.value = 2*halfSize
                self._requireSketch(sketch, 1)
            else:
                self._polygon(sketch, [_point(_add(centre, _add(_scale(self.eHat, x), _scale(self.kHat, y))))
                                      for x, y in ((-halfSize, -halfSize), (halfSize, -halfSize),
                                                   (halfSize, halfSize), (-halfSize, halfSize))])
            profile: adsk.fusion.Profile = sketch.profiles.item(0)
            markCentrePlusNormal: adsk.core.Point3D = _point(_add(centre, self.nHat))
            local: adsk.core.Point3D = sketch.modelToSketchSpace(markCentrePlusNormal)
            direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if local.z > 0 else
                         adsk.fusion.ExtentDirections.NegativeExtentDirection)
            extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
                profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(inset+0.04)
            extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
            extrudeInput.setOneSideExtent(extent, direction)
            extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
            if extrudeFeature.bodies.count != 1:
                raise ValueError(f'{sketch.name} extrude returned {extrudeFeature.bodies.count} bodies; expected one')
            markBody: adsk.fusion.BRepBody = extrudeFeature.bodies.item(0)
            cageBody: adsk.fusion.BRepBody = self.cageBody
            self.cageBody = self._join(cageBody, markBody)

    def relocateBodies(self):
        for g, label in enumerate(('Gear A', 'Gear B')):
            body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[g])
            targetOccurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(self.gearOccs[g])
            body.name = label
            self.gearBodies[g] = body.moveToComponent(targetOccurrence)
        body: adsk.fusion.BRepBody = self.cageBody
        targetOccurrence: adsk.fusion.Occurrence = self.cageOcc
        body.name = 'Cage'
        self.cageBody = body.moveToComponent(targetOccurrence)

    def _cell(self, index, start, teeth, suffix) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designOcc.component
        plane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[index])
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = ('Gear A', 'Gear B')[index]+' '+suffix
        sketch.isComputeDeferred = True
        points, splines, sections = [], [], []
        for k in range(teeth*self.stepsPerTooth+1):
            station = start+k*self.P/self.stepsPerTooth
            B0: adsk.fusion.SketchPoint = self._sketchPoint(sketch, self._world(index, -self.W/2, -self.T/2, station), False)
            B1: adsk.fusion.SketchPoint = self._sketchPoint(sketch, self._world(index, -self.W/2, self.T/2, station), False)
            front = []
            fitPoints: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for j in range(TOOTH_SPLINE_POINTS):
                v = -self.T/2+j*self.T/(TOOTH_SPLINE_POINTS-1)
                sectionPoint: adsk.fusion.SketchPoint = self._sketchPoint(
                    sketch, self._world(index, self._edge(index, v, station), v, station), False)
                front.append(sectionPoint)
                fitPoints.add(sectionPoint)
            F0: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(front[0])
            F1: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(front[-1])
            L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F0)
            spline: adsk.fusion.SketchFittedSpline = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
            if spline is None or spline.fitPoints.count != TOOTH_SPLINE_POINTS:
                raise ValueError(f'{sketch.name} section {k} fitted spline has an invalid fit-point count')
            L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(F1, B1)
            L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)
            sections.append((L1, spline, L3, L4))
            splines.append(spline)
            points.extend((B0, B1))
            points.extend(front)
        for item in points:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(item)
            point.isFixed = True
        for item in splines:
            spline: adsk.fusion.SketchFittedSpline = adsk.fusion.SketchFittedSpline.cast(item)
            for i in range(spline.fitPoints.count):
                point: adsk.fusion.SketchPoint = spline.fitPoints.item(i)
                point.isFixed = True
        sketch.isComputeDeferred = False
        self._requireSketch(sketch, teeth*self.stepsPerTooth+1)
        loftInput: adsk.fusion.LoftFeatureInput = design.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for section in sections:
            if len(section) != 4:
                raise ValueError(f'{sketch.name} loft section has {len(section)} curves; expected four')
            curves: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for item in section:
                sectionCurve: adsk.fusion.SketchCurve = adsk.fusion.SketchCurve.cast(item)
                curves.add(sectionCurve)
            sectionPath: adsk.fusion.Path = design.features.createPath(curves, False)
            loftInput.loftSections.add(sectionPath)
        loftFeature: adsk.fusion.LoftFeature = design.features.loftFeatures.add(loftInput)
        if loftFeature.bodies.count != 1:
            raise ValueError(f'{sketch.name} loft returned {loftFeature.bodies.count} bodies; expected one')
        cellBody: adsk.fusion.BRepBody = loftFeature.bodies.item(0)
        if not cellBody.isSolid:
            raise ValueError(f'{sketch.name} loft returned a non-solid body')
        sc = self.phases[index]+self.P*math.ceil((start+self.P/2-self.phases[index])/self.P)
        used = 0
        for sigma in (-1, 1):
            v = sigma*(self.T/2-0.025)
            u = self.W/2-self.bow*v*v-0.025
            for name, s in (('on-ridge', sc-math.tan(self.slant)*v), ('off-ridge', sc+math.tan(self.slant)*v)):
                margin, opposite = self._edge(index, v, s)-u, self._edge(index, v, s, -self.slant)-u
                if abs(margin) < 0.01 or abs(opposite) < 0.01 or margin*opposite >= 0:
                    continue
                probePoint: adsk.core.Point3D = self._world(index, u, v, s)
                observed = cellBody.pointContainment(probePoint)
                expected = (adsk.fusion.PointContainment.PointInsidePointContainment if margin > 0 else
                            adsk.fusion.PointContainment.PointOutsidePointContainment)
                if observed != expected:
                    raise ValueError(f'{sketch.name} {name} face {sigma} containment {observed}; '
                                     f'expected {expected}, margins {margin}, {opposite} cm')
                used += 1
        if used == 0:
            futil.log(f'{sketch.name}: tooth slant sign was not checked')
        return cellBody

    def _copy(self, sourceBody: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designOcc.component
        copyFeature: adsk.fusion.CopyPasteBody = design.features.copyPasteBodies.add(sourceBody)
        if copyFeature.bodies.count != 1:
            raise ValueError(f'Copy of {sourceBody.name} returned {copyFeature.bodies.count} bodies; expected one')
        return copyFeature.bodies.item(0)

    def _screwMove(self, index, copyBody: adsk.fusion.BRepBody, k):
        design: adsk.fusion.Component = self.designOcc.component
        d = self.dirVecs[index]
        axisVector: adsk.core.Vector3D = adsk.core.Vector3D.create(d[0], d[1], d[2])
        axisPoint: adsk.core.Point3D = _point(self.origins[index])
        rot: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        rot.setToRotation(k*self.P/self.lam, axisVector, axisPoint)
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k*self.P)
        mov: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)
        bodies: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        bodies.add(copyBody)
        moveInput: adsk.fusion.MoveFeatureInput = design.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        design.features.moveFeatures.add(moveInput)

    def _join(self, targetBody: adsk.fusion.BRepBody, toolBody: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designOcc.component
        tools: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput: adsk.fusion.CombineFeatureInput = design.features.combineFeatures.createInput(targetBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature: adsk.fusion.CombineFeature = design.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise ValueError(f'Join for {targetBody.name} returned {combineFeature.bodies.count} bodies; expected one')
        return combineFeature.bodies.item(0)

    def repeatCellByDoubling(self, index):
        body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[index])
        m, asides = 1, []
        while m*2 <= self.q:
            if (self.q//m) % 2:
                aside: adsk.fusion.BRepBody = self._copy(body)
                asides.append((m, aside))
            copyBody: adsk.fusion.BRepBody = self._copy(body)
            self._screwMove(index, copyBody, m*self.cellTeeth)
            body = self._join(body, copyBody)
            m *= 2
        for count, item in reversed(asides):
            aside: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(item)
            self._screwMove(index, aside, m*self.cellTeeth)
            body = self._join(body, aside)
            m += count
        if m != self.q:
            raise ValueError(f'Gear {index} doubling placed {m} cells; expected {self.q}')
        if self.r:
            start = self.phases[index]-self.L/2+self.q*self.cellTeeth*self.P
            remainder: adsk.fusion.BRepBody = self._cell(index, start, self.r, 'Cell Remainder')
            body = self._join(body, remainder)
        body.name = ('Gear A', 'Gear B')[index]
        self.gearBodies[index] = body
