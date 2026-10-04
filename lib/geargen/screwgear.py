import math
import adsk.core
import adsk.fusion
from . import base, misc, utilities, solids
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


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs: adsk.core.CommandInputs = command.commandInputs
        for id, label, tooltip, filters in (
            (INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to.",
             (adsk.core.SelectionCommandInput.ConstructionPlanes, adsk.core.SelectionCommandInput.PlanarFaces)),
            (INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism.',
             (adsk.core.SelectionCommandInput.ConstructionPoints, adsk.core.SelectionCommandInput.SketchPoints)),
            (INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under.',
             (adsk.core.SelectionCommandInput.Occurrences, adsk.core.SelectionCommandInput.RootComponents)),
        ):
            selectionInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(id, label, tooltip)
            for filterConstant in filters:
                selectionInput.addSelectionFilter(filterConstant)
            selectionInput.setSelectionLimits(1, 1)
            if id == INPUT_ID_PARENT:
                selectionInput.addSelection(misc.get_design().rootComponent)
        rows = (
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
                (INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.020),
                (INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', 0.030),
                (INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 0.3),
                (INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 0.3))),
            ('meshGroup', 'Mesh (from the mesh search)', False, (
                (INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', math.radians(80)),
                (INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 0.105),
                (INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', 0),
                (INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', 0),
                (INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -0.131))),
        )
        for groupId, groupLabel, expanded, values in rows:
            group: adsk.core.GroupCommandInput = inputs.addGroupCommandInput(groupId, groupLabel)
            group.isExpanded = expanded
            for id, label, unit, default in values:
                group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def _scale(a, k):
    return tuple(x*k for x in a)


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _clip(poly, a, b, limit):
    if not poly:
        return []
    out = []
    prev = poly[-1]
    fp = a*prev[0]+b*prev[1]-limit
    for current in poly:
        fc = a*current[0]+b*current[1]-limit
        if (fp <= 0) != (fc <= 0):
            fraction = fp/(fp-fc)
            out.append((prev[0]+fraction*(current[0]-prev[0]), prev[1]+fraction*(current[1]-prev[1])))
        if fc <= 0:
            out.append(current)
        prev, fp = current, fc
    return out


def _area(poly):
    return abs(sum(p[0]*poly[(i+1)%len(poly)][1]-p[1]*poly[(i+1)%len(poly)][0]
                   for i, p in enumerate(poly)))/2 if poly else 0


def _hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        raise ValueError('Bore outline has fewer than three distinct projected points')
    def turn(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def _separation(p, q):
    best = -math.inf
    for hull in (p, q):
        for i, a in enumerate(hull):
            b = hull[(i+1)%len(hull)]
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            m = (-dy/length, dx/length)
            pp = [_dot(m, v) for v in p]
            qq = [_dot(m, v) for v in q]
            best = max(best, min(qq)-max(pp), min(pp)-max(qq))
    return best


def _distance2(poly, x, y):
    inside = len(poly) >= 3
    best = math.inf
    for i, a in enumerate(poly):
        b = poly[(i+1)%len(poly)]
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx*(y-a[1])-dy*(x-a[0]) < 0:
            inside = False
        denom = dx*dx+dy*dy
        t = min(1, max(0, ((x-a[0])*dx+(y-a[1])*dy)/denom)) if denom else 0
        best = min(best, (x-a[0]-t*dx)**2+(y-a[1]-t*dy)**2)
    return 0 if inside else best


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

    def processInputs(self, inputs: adsk.core.CommandInputs):
        design: adsk.fusion.Design = self.design
        unitsManager: adsk.core.UnitsManager = design.unitsManager
        for id in (INPUT_ID_PLANE, INPUT_ID_POINT, INPUT_ID_PARENT):
            selectionInput: adsk.core.SelectionCommandInput = adsk.core.SelectionCommandInput.cast(inputs.itemById(id))
            if selectionInput is None:
                raise ValueError(f'Missing input {id}')
            if selectionInput.selectionCount != 1:
                raise ValueError(f'{id} requires exactly one selection')
            entity: adsk.core.Base = selectionInput.selection(0).entity
            if id == INPUT_ID_PLANE:
                self.targetPlane = entity
            elif id == INPUT_ID_POINT:
                self.point = entity
            else:
                occurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(entity)
                self.parentComponent = occurrence.component if occurrence else adsk.fusion.Component.cast(entity)
        lengths = (INPUT_ID_RIBBON_WIDTH, INPUT_ID_TWIST_LEAD, INPUT_ID_RIBBON_THICKNESS,
                   INPUT_ID_TOOTH_PITCH, INPUT_ID_TOOTH_HEIGHT, INPUT_ID_CAGE_RADIUS,
                   INPUT_ID_CAGE_RISE, INPUT_ID_CLEARANCE, INPUT_ID_ROOF_ALLOWANCE,
                   INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_ENGAGEMENT, INPUT_ID_ASSEMBLY_PHASE)
        angles = (INPUT_ID_TOOTH_SLANT, INPUT_ID_CROSS_ANGLE, INPUT_ID_MOUNT_ANGLE_A, INPUT_ID_MOUNT_ANGLE_B)
        values = {}
        for id in lengths+angles+(INPUT_ID_TOOTH_COUNT, INPUT_ID_TOOTH_BOW):
            input: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(inputs.itemById(id))
            if input is None:
                raise ValueError(f'Missing input {id}')
            unit = 'mm' if id in lengths else 'deg' if id in angles else ''
            value = unitsManager.evaluateExpression(input.expression, unit)
            if value != value or value == math.inf or value == -math.inf:
                raise ValueError(f'{id} must be finite')
            values[id] = value
        self.values = values
        for id in (INPUT_ID_RIBBON_WIDTH, INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH,
                   INPUT_ID_TWIST_LEAD, INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CLEARANCE):
            if values[id] <= 0:
                raise ValueError(f'{id} must be > 0')
        if values[INPUT_ID_ROOF_ALLOWANCE] < 0:
            raise ValueError('roofAllowance must be >= 0')
        count = values[INPUT_ID_TOOTH_COUNT]
        if count < 4 or count != math.floor(count):
            raise ValueError('toothCount must be a whole number >= 4')
        self.N = int(count)
        self.W = values[INPUT_ID_RIBBON_WIDTH]
        self.T = values[INPUT_ID_RIBBON_THICKNESS]
        self.P = values[INPUT_ID_TOOTH_PITCH]
        self.H = values[INPUT_ID_TOOTH_HEIGHT]
        self.slant = values[INPUT_ID_TOOTH_SLANT]
        self.bow = 10*values[INPUT_ID_TOOTH_BOW]
        self.engagement = values[INPUT_ID_ENGAGEMENT]
        self.sigma = values[INPUT_ID_CROSS_ANGLE]
        self.phase = [0, values[INPUT_ID_ASSEMBLY_PHASE]]
        self.phi = [values[INPUT_ID_MOUNT_ANGLE_A], values[INPUT_ID_MOUNT_ANGLE_B]]
        self.lam = values[INPUT_ID_TWIST_LEAD]/(2*math.pi)
        self.A = self.W-self.engagement
        self.L = self.N*self.P
        self.beta = math.atan(math.pi*self.W/values[INPUT_ID_TWIST_LEAD])
        self.cageRadius = values[INPUT_ID_CAGE_RADIUS]
        self.cageRise = values[INPUT_ID_CAGE_RISE]
        self.collarHalf = values[INPUT_ID_COLLAR_HALF]
        self.collarWall = values[INPUT_ID_COLLAR_WALL]
        self.clearance = values[INPUT_ID_CLEARANCE]
        self.roofAllowance = values[INPUT_ID_ROOF_ALLOWANCE]
        if not 0 < self.H < self.W/2:
            raise ValueError('toothHeight must be > 0 and < ribbonWidth/2')
        if not -math.pi/2 < self.slant < math.pi/2:
            raise ValueError('toothSlant must lie strictly between -90 deg and 90 deg')
        if self.bow < 0 or self.H+self.bow*(self.T/2)**2 >= self.W/2:
            raise ValueError('toothBow must be >= 0 and toothHeight + toothBow*(ribbonThickness/2)^2 < ribbonWidth/2')
        if not 0 < self.engagement <= self.H:
            raise ValueError('engagement must be > 0 and <= toothHeight')
        if not 0 < self.sigma < math.pi:
            raise ValueError('crossAngle must lie strictly between 0 deg and 180 deg')
        if not -self.P < self.phase[1] < self.P:
            raise ValueError('assemblyPhase must lie strictly within +/-toothPitch')
        if self.cageRadius+self.collarHalf+0.1 >= self.L/2:
            raise ValueError('cageRadius + collarHalf + 1 mm must be < toothCount*toothPitch/2')
        self.Ri = self.cageRadius-self.collarHalf
        self.Ro = self.cageRadius+self.collarHalf
        self.hw = self.W/2+self.clearance
        self.ht = self.T/2+self.clearance
        self.cornerRadius = math.hypot(self.hw, self.ht+self.roofAllowance)
        if math.hypot(self.cornerRadius, 0.1) >= self.Ri:
            raise ValueError('cageRadius must give hypot(c, 1 mm) < cageRadius - collarHalf')
        axialWindow = 1.5*math.sqrt(self.W*self.W-self.A*self.A)/math.sin(self.sigma)
        if math.hypot(axialWindow, math.hypot(self.W/2, self.T/2))+self.clearance > self.Ri:
            raise ValueError('cageRadius must keep hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri')
        if self.cageRise < self.A/2+self.cornerRadius+self.collarWall:
            raise ValueError('cageRise must be >= A/2 + c + collarWall')
        self.sIn = math.sqrt(self.Ri*self.Ri-self.cornerRadius*self.cornerRadius)-0.1
        self.sOut = self.Ro+0.1
        self.cellTeeth = min(CELL_TEETH, self.N)
        self.stepsPerTooth = max(8, math.ceil((self.P/self.lam)/math.radians(2)))
        self.cellSections = self.cellTeeth*self.stepsPerTooth+1
        self.wholeCells = self.N//self.cellTeeth
        self.remainderTeeth = self.N % self.cellTeeth
        self._prepareSearches()

    def _prepareSearches(self):
        self.searchRi = 10*self.Ri
        self.searchRo = 10*self.Ro
        self.searchRise = 10*self.cageRise
        self.searchWall = 10*self.collarWall
        self.searchC = 10*self.cornerRadius
        self.searchLam = 10*self.lam
        self.bores = []
        for g in (0, 1):
            angle = self.sigma/2 if g == 0 else -self.sigma/2
            direction = (math.cos(angle), math.sin(angle), 0)
            u = (0, 0, 1 if g == 0 else -1)
            v = _cross(direction, u)
            origin = (0, 0, (-1 if g == 0 else 1)*5*self.A)
            tilts = {}
            for sign in (-1, 1):
                a = sign*10*(self.cageRadius-self.collarHalf)/self.searchLam+self.phi[g]
                b = sign*10*(self.cageRadius+self.collarHalf)/self.searchLam+self.phi[g]
                lo, hi = min(a, b), max(a, b)
                k = math.ceil((lo-math.pi/2)/math.pi)
                tilts[sign] = 0 if math.pi/2+k*math.pi <= hi else min(abs(math.cos(a)), abs(math.cos(b)))
            level = -1 if tilts[-1] <= tilts[1] else 1
            for sign in (-1, 1):
                vLo, vHi = -10*self.ht, 10*self.ht
                theta = sign*10*self.cageRadius/self.searchLam+self.phi[g]
                if sign == level:
                    if -math.sin(theta)*u[2] > 0:
                        vHi += 10*self.roofAllowance
                    else:
                        vLo -= 10*self.roofAllowance
                start, end = ((-10*self.sOut, -10*self.sIn) if sign < 0 else (10*self.sIn, 10*self.sOut))
                self.bores.append(dict(g=g, sign=sign, dir=direction, u=u, v=v, origin=origin,
                                       lo=vLo, hi=vHi, start=start, end=end,
                                       name=f'Gear {"A" if g == 0 else "B"} Bore {"-R" if sign < 0 else "+R"}'))
        outlines = [self._channelOutline(b) for b in self.bores]
        gaps = (((0, 1, 0), 1, 2, '+k'), ((-1, 0, 0), 0, 2, '-e'),
                ((0, -1, 0), 0, 3, '-k'), ((1, 0, 0), 1, 3, '+e'))
        minimum, pair = math.inf, ''
        for d, i, j, name in gaps:
            across = _cross((0, 0, 1), d)
            p = _hull([(_dot(a, across), a[2]) for a in outlines[i]])
            q = _hull([(_dot(a, across), a[2]) for a in outlines[j]])
            separation = _separation(p, q)
            if separation < minimum:
                minimum, pair = separation, f'{self.bores[i]["name"]} and {self.bores[j]["name"]}'
        if minimum < self.searchWall:
            raise ValueError(f'collarWall must be <= separation {minimum:.6f} mm between {pair}')
        self.channelSeparation = minimum/10
        self.zLimit = self._channelTop()
        for bore in self.bores:
            bore['table'] = self._stationTable(bore)
        facings = (((0, 1, 0), '+k'), ((0, -1, 0), '-k')) if self.sigma <= math.pi/2 else (
            ((1, 0, 0), '+e'), ((-1, 0, 0), '-e'))
        self.windows = []
        for d, label in facings:
            window = self._newWindow(d, label)
            if window is not None:
                self.windows.append(window)

    def _boreWorld(self, bore, s, x, y):
        return _add(bore['origin'], _add(_scale(bore['dir'], s), _add(_scale(bore['u'], x), _scale(bore['v'], y))))

    def _turnedOutline(self, bore, s):
        theta = s/self.searchLam+self.phi[bore['g']]
        cs, sn = math.cos(theta), math.sin(theta)
        hw = 10*self.hw
        return [(u*cs-v*sn, u*sn+v*cs) for u, v in
                ((-hw, bore['lo']), (hw, bore['lo']), (hw, bore['hi']), (-hw, bore['hi']))]

    def _channelOutline(self, bore):
        points = []
        s = bore['sign']*10*self.sIn
        while abs(s) <= 10*self.sOut:
            theta = s/self.searchLam+self.phi[bore['g']]
            cs, sn = math.cos(theta), math.sin(theta)
            for i in range(17):
                v = bore['lo']+(bore['hi']-bore['lo'])*i/16
                u = -10*self.hw+20*self.hw*i/16
                for a, b in ((10*self.hw, v), (-10*self.hw, v), (u, bore['hi']), (u, bore['lo'])):
                    p = self._boreWorld(bore, s, a*cs-b*sn, a*sn+b*cs)
                    radius = math.hypot(p[0], p[1])
                    if self.searchRi-0.5 <= radius <= self.searchRo+0.5:
                        points.append(p)
            s += bore['sign']*0.1
        if not points:
            raise ValueError(f'{bore["name"]}: channel outline has no points within the sleeve wall')
        return points

    def _sectionInWall(self, bore, s):
        if abs(s) >= self.searchRo:
            return [[], []]
        poly = self._turnedOutline(bore, s)
        z = bore['origin'][2]
        uz = bore['u'][2]
        poly = _clip(poly, uz, 0, self.searchRise-z)
        poly = _clip(poly, -uz, 0, self.searchRise+z)
        near = math.sqrt(max(0, self.searchRi**2-s*s))
        far = math.sqrt(self.searchRo**2-s*s)
        positive = _clip(_clip(poly, 0, -1, -near), 0, 1, far)
        negative = _clip(_clip(poly, 0, -1, far), 0, 1, -near)
        return [positive, negative]

    def _channelTop(self):
        top = 0
        for bore in self.bores:
            s = bore['sign']*10*self.sIn
            while abs(s) <= 10*self.sOut:
                corners = self._turnedOutline(bore, s)
                largestY = max(abs(y) for x, y in corners)
                if abs(s) <= self.searchRo and math.hypot(s, largestY) >= self.searchRi:
                    top = max(top, max(abs(bore['origin'][2]+x*bore['u'][2]) for x, y in corners))
                s += bore['sign']*0.01
        return top

    def _stationTable(self, bore):
        stations = []
        count = int(math.floor((bore['end']-bore['start'])/0.002))
        previous = None
        for k in range(count+1):
            s = bore['start']+k*0.002
            pieces = self._sectionInWall(bore, s)
            present = tuple(bool(p) for p in pieces)
            if previous is not None and present != previous:
                for j in range(1, 20):
                    refined = s-0.002+j*0.0001
                    stations.append(self._tableEntry(refined, self._sectionInWall(bore, refined)))
            stations.append(self._tableEntry(s, pieces))
            previous = present
        return stations

    def _tableEntry(self, s, pieces):
        circles = []
        for poly in pieces:
            if not poly:
                continue
            cx = sum(p[0] for p in poly)/len(poly)
            cy = sum(p[1] for p in poly)/len(poly)
            radius = max(math.hypot(p[0]-cx, p[1]-cy) for p in poly)
            circles.append((poly, cx, cy, radius))
        return s, circles

    def _wallGap(self, p, bore, reach):
        relative = tuple(p[i]-bore['origin'][i] for i in range(3))
        x, y, sq = _dot(relative, bore['u']), _dot(relative, bore['v']), _dot(relative, bore['dir'])
        closest = min(max(sq, bore['start']), bore['end'])
        if math.hypot(math.hypot(x, y), sq-closest)-self.searchC >= reach:
            return reach
        table = bore['table']
        lower, upper = 0, len(table)
        while lower < upper:
            mid = (lower+upper)//2
            if table[mid][0] < sq:
                lower = mid+1
            else:
                upper = mid
        best = reach*reach
        for index, increment in ((lower, 1), (lower-1, -1)):
            while 0 <= index < len(table):
                s, pieces = table[index]
                ds2 = (sq-s)**2
                if ds2 >= best:
                    break
                for poly, cx, cy, radius in pieces:
                    o = math.hypot(x-cx, y-cy)-radius
                    if o > 0 and ds2+o*o >= best:
                        continue
                    best = min(best, ds2+_distance2(poly, x, y))
                index += increment
        return math.sqrt(best)

    def _wallCorners(self, bore):
        corners = []
        count = int(math.floor((bore['end']-bore['start'])/0.001))
        for k in range(count+1):
            s = bore['start']+k*0.001
            for poly in self._sectionInWall(bore, s):
                for x, y in poly:
                    corners.append(self._boreWorld(bore, s, x, y))
        if not corners:
            raise ValueError(f'{bore["name"]}: no channel section reaches the sleeve wall')
        return corners

    def _newWindow(self, d, label):
        across = _cross((0, 0, 1), d)
        flank, far = [], []
        for bore in self.bores:
            crossing = self._boreWorld(bore, bore['sign']*10*self.cageRadius, 0, 0)
            if _dot(crossing, d) > 0:
                flank.append((bore, crossing))
            else:
                far.append(bore)
        if len(flank) != 2 or len(far) != 2:
            raise ValueError(f'Window {label}: expected two flanking and two far bores')
        flank.sort(key=lambda item: item[1][2])
        low, high = flank
        lean = 1 if _dot(high[1], across) > _dot(low[1], across) else -1
        lowReach = max(p[2]+lean*_dot(p, across) for p in self._wallCorners(low[0]))
        highReach = min(p[2]+lean*_dot(p, across) for p in self._wallCorners(high[0]))
        lo = lowReach+math.sqrt(2)*self.searchWall
        hi = highReach-math.sqrt(2)*self.searchWall
        if hi <= lo:
            futil.log(f'No window facing {label}: flanking bores leave no band')
            return None
        top = min(2*self.zLimit-hi, hi+math.sqrt(2)*self.searchRi)
        bottom = max(-2*self.zLimit-lo, lo-math.sqrt(2)*self.searchRi)
        right = self._windowEnd(1, lean, lo, hi, bottom, top, d, across, far)
        left = -self._windowEnd(-1, lean, lo, hi, bottom, top, d, across, far)
        poly = [(-2*self.searchRo, -2*self.searchRo), (2*self.searchRo, -2*self.searchRo),
                (2*self.searchRo, 2*self.searchRo), (-2*self.searchRo, 2*self.searchRo)]
        for a, b, bound in ((lean, 1, hi), (-lean, -1, -lo), (-lean, 1, top),
                            (lean, -1, -bottom), (1, 0, right), (-1, 0, -left)):
            poly = _clip(poly, a, b, bound)
        distinct = []
        for p in poly:
            if not distinct or math.hypot(p[0]-distinct[-1][0], p[1]-distinct[-1][1]) >= 0.001:
                distinct.append(p)
        if len(distinct) > 1 and math.hypot(distinct[0][0]-distinct[-1][0], distinct[0][1]-distinct[-1][1]) < 0.001:
            distinct.pop()
        if right <= left or len(distinct) < 3 or _area(distinct) <= 0:
            futil.log(f'No window facing {label}: far bores leave no window length or area')
            return None
        return dict(label=label, d=d, across=across, corners=[(t/10, z/10) for t, z in distinct])

    def _windowEnd(self, sign, lean, lo, hi, bottom, top, d, across, far):
        low, high = 0, min(self.searchRi, self.searchRo/math.sqrt(2))*(1-1e-9)
        need = self.searchWall+0.1/math.sqrt(2)+0.005
        for iteration in range(24):
            te = (low+high)/2
            t = sign*te
            zLow, zHigh = max(lo-lean*t, bottom+lean*t), min(hi-lean*t, top+lean*t)
            clear = zLow <= zHigh
            if clear:
                a0 = math.sqrt(max(0, self.searchRi**2-t*t))
                a1 = math.sqrt(max(0, self.searchRo**2-t*t))
                probes = []
                for z in (zLow, zHigh):
                    a = a0
                    while a < a1:
                        probes.append(_add(_scale(across, t), _add((0, 0, z), _scale(d, a))))
                        a = min(a+0.1, a1)
                    probes.append(_add(_scale(across, t), _add((0, 0, z), _scale(d, a1))))
                n = math.ceil((zHigh-zLow)/0.1)
                heights = [zLow+(zHigh-zLow)*k/n for k in range(1, n)] if n else []
                j = 0
                z = -self.zLimit
                while z <= self.zLimit:
                    if zLow < z < zHigh:
                        heights.append(z)
                    j += 1
                    z = -self.zLimit+j*0.1
                for a in (a0, a1):
                    for z in heights:
                        probes.append(_add(_scale(across, t), _add((0, 0, z), _scale(d, a))))
                for p in probes:
                    if any(self._wallGap(p, bore, need) < need for bore in far):
                        clear = False
                        break
            if clear:
                low = te
            else:
                high = te
        return low

    def buildComponentTree(self):
        top: adsk.fusion.Occurrence = self.getOccurrence()
        parent: adsk.fusion.Component = top.component
        parent.name = 'Screw Gearing'
        self.designOcc = adsk.fusion.Occurrence.cast(None)
        self.gearOccs = [adsk.fusion.Occurrence.cast(None)]*2
        self.cageOcc = adsk.fusion.Occurrence.cast(None)
        self.gearBodies = [adsk.fusion.BRepBody.cast(None)]*2
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]
        for name in ('Design', 'Gear A', 'Gear B', 'Cage'):
            occurrence: adsk.fusion.Occurrence = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occurrence.component.name = name
            if name == 'Design':
                self.designOcc = occurrence
            elif name == 'Gear A':
                self.gearOccs[0] = occurrence
            elif name == 'Gear B':
                self.gearOccs[1] = occurrence
            else:
                self.cageOcc = occurrence

    def _component(self) -> adsk.fusion.Component:
        return self.designOcc.component

    def _checkSketch(self, sketch: adsk.fusion.Sketch):
        if not sketch.isFullyConstrained:
            raise ValueError(f'{sketch.name}: isFullyConstrained={sketch.isFullyConstrained}')

    def _local(self, sketch: adsk.fusion.Sketch, worldPoint: adsk.core.Point3D) -> adsk.core.Point3D:
        localPoint: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
        localPoint.z = 0
        return localPoint

    def _point(self, coordinates) -> adsk.core.Point3D:
        return adsk.core.Point3D.create(coordinates[0], coordinates[1], coordinates[2])

    def _vector(self, coordinates) -> adsk.core.Vector3D:
        return adsk.core.Vector3D.create(coordinates[0], coordinates[1], coordinates[2])

    def _xyz(self, value: adsk.core.Point3D):
        return value.x, value.y, value.z

    def _world(self, t=0, k=0, n=0) -> adsk.core.Point3D:
        return self._point(_add(self.center, _add(_scale(self.e, t), _add(_scale(self.k, k), _scale(self.n, n)))))

    def buildAnchor(self):
        component: adsk.fusion.Component = self._component()
        sketch: adsk.fusion.Sketch = component.sketches.add(self.targetPlane)
        sketch.name = 'Anchor'
        projected: adsk.core.ObjectCollection = sketch.project(self.point)
        if projected.count != 1:
            raise ValueError(f'Anchor: selected point projected {projected.count} entities, expected 1')
        projectedPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projected.item(0))
        if projectedPoint is None:
            raise ValueError('Anchor: selected point did not project a SketchPoint')
        localPoint: adsk.core.Point3D = projectedPoint.geometry
        startSeed: adsk.core.Point3D = adsk.core.Point3D.create(localPoint.x-0.5, localPoint.y, 0)
        endSeed: adsk.core.Point3D = adsk.core.Point3D.create(localPoint.x+0.5, localPoint.y, 0)
        anchorLine: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)
        sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)
        sketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint: adsk.core.Point3D = adsk.core.Point3D.create(localPoint.x, localPoint.y+0.5, 0)
        dimension: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dimension.parameter.value = 1.0
        self._checkSketch(sketch)
        self.anchorLine = anchorLine
        self.C = projectedPoint.worldGeometry
        self.center = self._xyz(self.C)
        eHat: adsk.core.Vector3D = anchorLine.startSketchPoint.worldGeometry.vectorTo(anchorLine.endSketchPoint.worldGeometry)
        if not eHat.normalize():
            raise ValueError('Anchor Line has zero world length')
        self.eHat = eHat
        self.e = (eHat.x, eHat.y, eHat.z)
        self.axisPlanes = [adsk.fusion.ConstructionPlane.cast(None)]*2
        for index, offset in ((0, -self.A/2), (1, self.A/2)):
            planeInput: adsk.fusion.ConstructionPlaneInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(offset))
            axisPlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
            axisPlane.name = f'Gear {"A" if index == 0 else "B"} Axis Plane'
            self.axisPlanes[index] = axisPlane
        planeA: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[0])
        nHat: adsk.core.Vector3D = planeA.geometry.normal.copy()
        if not nHat.normalize():
            raise ValueError('Gear A Axis Plane has zero normal')
        towardCenter: adsk.core.Vector3D = planeA.geometry.origin.vectorTo(self.C)
        if nHat.dotProduct(towardCenter) < 0:
            nHat.scaleBy(-1)
        self.nHat = nHat
        self.n = (nHat.x, nHat.y, nHat.z)
        self.k = _cross(self.n, self.e)
        self.kHat = self._vector(self.k)
        for index in (0, 1):
            axisPlane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[index])
            planePoint: adsk.core.Point3D = axisPlane.geometry.origin
            delta = tuple(self._xyz(planePoint)[i]-self.center[i] for i in range(3))
            actual = _dot(delta, self.n)
            expected = (-1 if index == 0 else 1)*self.A/2
            if abs(actual-expected) > 1e-5:
                raise ValueError(f'{axisPlane.name}: signed offset {actual} cm, expected {expected} cm')
        self.dirs, self.origins, self.us, self.vs = [], [], [], []
        self.dirVecs = [adsk.core.Vector3D.cast(None)]*2
        for index in (0, 1):
            angle = self.sigma/2 if index == 0 else -self.sigma/2
            direction = _add(_scale(self.e, math.cos(angle)), _scale(self.k, math.sin(angle)))
            u = _scale(self.n, 1 if index == 0 else -1)
            self.dirs.append(direction)
            self.origins.append(_add(self.center, _scale(self.n, (-1 if index == 0 else 1)*self.A/2)))
            self.us.append(u)
            self.vs.append(_cross(direction, u))
            self.dirVecs[index] = self._vector(direction)

    def _sectionWorld(self, index, s, u, v) -> adsk.core.Point3D:
        theta = s/self.lam+self.phi[index]
        x, y = u*math.cos(theta)-v*math.sin(theta), u*math.sin(theta)+v*math.cos(theta)
        return self._point(_add(self.origins[index], _add(_scale(self.dirs[index], s),
                                                      _add(_scale(self.us[index], x), _scale(self.vs[index], y)))))

    def _axisWorld(self, index, s) -> adsk.core.Point3D:
        return self._point(_add(self.origins[index], _scale(self.dirs[index], s)))

    def buildGear(self, index):
        futil.log(f'Building Gear {"A" if index == 0 else "B"}')
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, index):
        component: adsk.fusion.Component = self._component()
        axisPlane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[index])
        sketch: adsk.fusion.Sketch = component.sketches.add(axisPlane)
        sketch.name = f'Gear {"A" if index == 0 else "B"} Paths'
        points = []
        for key, start, end in (('bore-', -self.sOut, -self.sIn), ('bore+', self.sIn, self.sOut)):
            startPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(self._local(sketch, self._axisWorld(index, start)))
            endPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(self._local(sketch, self._axisWorld(index, end)))
            line: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
            points.extend((startPoint, endPoint))
            self.pathLines[index][key] = line
        for value in points:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(value)
            point.isFixed = True
        self._checkSketch(sketch)

    def _tooth(self, index, v, s, slant):
        return self.W/2-self.H/2+(self.H/2)*math.cos(
            2*math.pi*(s+math.tan(slant)*v-self.phase[index])/self.P)-self.bow*v*v

    def buildToothCell(self, index):
        start = self.phase[index]-self.L/2
        self.gearBodies[index] = self._loftCell(index, start, self.cellTeeth, 'Cell Sections')
        cellBody: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[index])
        self._checkSlant(index, start, cellBody)

    def _loftCell(self, index, start, teeth, suffix) -> adsk.fusion.BRepBody:
        component: adsk.fusion.Component = self._component()
        axisPlane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[index])
        sketch: adsk.fusion.Sketch = component.sketches.add(axisPlane)
        label = f'Gear {"A" if index == 0 else "B"}'
        sketch.name = f'{label} {suffix}'
        sketch.isComputeDeferred = True
        sections, allPoints, splines = [], [], []
        count = teeth*self.stepsPerTooth+1
        for k in range(count):
            s = start+k*self.P/self.stepsPerTooth
            B0: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
                sketch.modelToSketchSpace(self._sectionWorld(index, s, -self.W/2, -self.T/2)))
            B1: adsk.fusion.SketchPoint = sketch.sketchPoints.add(
                sketch.modelToSketchSpace(self._sectionWorld(index, s, -self.W/2, self.T/2)))
            fitPoints: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            front = []
            for j in range(TOOTH_SPLINE_POINTS):
                v = -self.T/2+j*self.T/(TOOTH_SPLINE_POINTS-1)
                worldPoint: adsk.core.Point3D = self._sectionWorld(index, s, self._tooth(index, v, s, self.slant), v)
                localPoint: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
                fitPoint: adsk.fusion.SketchPoint = sketch.sketchPoints.add(localPoint)
                fitPoints.add(fitPoint)
                front.append(fitPoint)
            F0: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(front[0])
            FLast: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(front[-1])
            L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F0)
            spline: adsk.fusion.SketchFittedSpline = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
            if spline is None or spline.fitPoints.count != TOOTH_SPLINE_POINTS:
                observed = 'None' if spline is None else spline.fitPoints.count
                raise ValueError(f'{sketch.name} section {k}: spline fitPoints={observed}, expected {TOOTH_SPLINE_POINTS}')
            L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(FLast, B1)
            L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)
            sections.append((L1, spline, L3, L4))
            allPoints.extend([B0, B1]+front)
            splines.append(spline)
        for value in allPoints:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(value)
            point.isFixed = True
        for value in splines:
            spline: adsk.fusion.SketchFittedSpline = adsk.fusion.SketchFittedSpline.cast(value)
            for i in range(spline.fitPoints.count):
                spline.fitPoints.item(i).isFixed = True
        sketch.isComputeDeferred = False
        self._checkSketch(sketch)
        if sketch.profiles.count != count:
            raise ValueError(f'{sketch.name}: {sketch.profiles.count} profiles, expected {count}')
        loftInput: adsk.fusion.LoftFeatureInput = component.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for k, section in enumerate(sections):
            if len(section) != 4:
                raise ValueError(f'{sketch.name} section {k}: {len(section)} curves, expected 4')
            curves: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for curve in section:
                curves.add(curve)
            sectionPath: adsk.fusion.Path = component.features.createPath(curves, False)
            loftInput.loftSections.add(sectionPath)
        loftFeature: adsk.fusion.LoftFeature = component.features.loftFeatures.add(loftInput)
        if loftFeature.bodies.count != 1:
            raise ValueError(f'{sketch.name} loft: {loftFeature.bodies.count} bodies, expected 1')
        cellBody: adsk.fusion.BRepBody = loftFeature.bodies.item(0)
        if not cellBody.isSolid:
            raise ValueError(f'{sketch.name} loft: isSolid={cellBody.isSolid}')
        return cellBody

    def _checkSlant(self, index, start, cellBody: adsk.fusion.BRepBody):
        crest = self.phase[index]+self.P*math.ceil((start+self.P/2-self.phase[index])/self.P)
        used = 0
        for sign in (-1, 1):
            v = sign*(self.T/2-0.025)
            u = self.W/2-self.bow*v*v-0.025
            for name, station in (('on-ridge', crest-math.tan(self.slant)*v),
                                  ('off-ridge', crest+math.tan(self.slant)*v)):
                margin = self._tooth(index, v, station, self.slant)-u
                opposite = self._tooth(index, v, station, -self.slant)-u
                if abs(margin) < 0.01 or abs(opposite) < 0.01 or margin*opposite >= 0:
                    continue
                used += 1
                probe: adsk.core.Point3D = self._sectionWorld(index, station, u, v)
                actual = cellBody.pointContainment(probe)
                expected = (adsk.fusion.PointContainment.PointInsidePointContainment if margin > 0
                            else adsk.fusion.PointContainment.PointOutsidePointContainment)
                if actual != expected:
                    raise ValueError(f'Gear {index} {name} face {sign}: containment={actual}, expected {expected}, margin={margin}')
        if not used:
            futil.log(f'Gear {"A" if index == 0 else "B"}: tooth slant sign was not checked')

    def _copyBody(self, body: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        component: adsk.fusion.Component = self._component()
        copyFeature: adsk.fusion.CopyPasteBody = component.features.copyPasteBodies.add(body)
        if copyFeature.bodies.count != 1:
            raise ValueError(f'{body.name} copy: {copyFeature.bodies.count} bodies, expected 1')
        return copyFeature.bodies.item(0)

    def _moveBlock(self, index, copiedBody: adsk.fusion.BRepBody, k):
        component: adsk.fusion.Component = self._component()
        axisVector: adsk.core.Vector3D = adsk.core.Vector3D.cast(self.dirVecs[index])
        axisPoint: adsk.core.Point3D = self._point(self.origins[index])
        pitch, lam = self.P, self.lam
        rotation: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        rotation.setToRotation(k*pitch/lam, axisVector, axisPoint)
        translationMatrix: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k*pitch)
        translationMatrix.translation = shift
        rotation.transformBy(translationMatrix)
        bodies: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        bodies.add(copiedBody)
        moveInput: adsk.fusion.MoveFeatureInput = component.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rotation)
        component.features.moveFeatures.add(moveInput)

    def _join(self, targetBody: adsk.fusion.BRepBody, toolBody: adsk.fusion.BRepBody, name) -> adsk.fusion.BRepBody:
        component: adsk.fusion.Component = self._component()
        tools: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput: adsk.fusion.CombineFeatureInput = component.features.combineFeatures.createInput(targetBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature: adsk.fusion.CombineFeature = component.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise ValueError(f'{name} join: {combineFeature.bodies.count} bodies, expected 1')
        return combineFeature.bodies.item(0)

    def repeatCellByDoubling(self, index):
        body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[index])
        q, c = self.wholeCells, self.cellTeeth
        m, asides = 1, []
        while 2*m <= q:
            if (q//m) % 2:
                aside: adsk.fusion.BRepBody = self._copyBody(body)
                asides.append((m, aside))
            copiedBody: adsk.fusion.BRepBody = self._copyBody(body)
            self._moveBlock(index, copiedBody, m*c)
            body = self._join(body, copiedBody, f'Gear {index} doubling {m} cells')
            m *= 2
        for count, value in reversed(asides):
            aside: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(value)
            self._moveBlock(index, aside, m*c)
            body = self._join(body, aside, f'Gear {index} aside {count} cells')
            m += count
        if self.remainderTeeth:
            start = self.phase[index]-self.L/2+q*c*self.P
            remainder: adsk.fusion.BRepBody = self._loftCell(index, start, self.remainderTeeth, 'Cell Remainder')
            body = self._join(body, remainder, f'Gear {index} remainder {self.remainderTeeth} teeth')
        body.name = f'Gear {"A" if index == 0 else "B"}'
        self.gearBodies[index] = body

    def buildCage(self):
        component: adsk.fusion.Component = self._component()
        sketch: adsk.fusion.Sketch = component.sketches.add(self.targetPlane)
        sketch.name = 'Sleeve'
        localCentre: adsk.core.Point3D = self._local(sketch, self.C)
        for radius in (self.Ri, self.Ro):
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(localCentre.x+radius, localCentre.y, 0)
            dimension: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            dimension.parameter.value = 2*radius
        self._checkSketch(sketch)
        rings = []
        for i in range(sketch.profiles.count):
            profile: adsk.fusion.Profile = sketch.profiles.item(i)
            if profile.profileLoops.count == 2:
                rings.append(profile)
        if len(rings) != 1:
            raise ValueError(f'Sleeve: {len(rings)} annular profiles among {sketch.profiles.count}, expected 1')
        ring: adsk.fusion.Profile = adsk.fusion.Profile.cast(rings[0])
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = component.features.extrudeFeatures.createInput(
            ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(self.cageRise), False)
        feature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(extrudeInput)
        if feature.bodies.count != 1:
            raise ValueError(f'Sleeve extrude: {feature.bodies.count} bodies, expected 1')
        self.cageBody = feature.bodies.item(0)
        for bore in self.bores:
            self._cutBore(bore)
        if self.windows:
            self._cutWindows()
        self._markBores()
        futil.log('Print the cage standing on its end below the selected plane: the roof '
                  'allowance is on the bridged roofs that way up.')

    def _boreSketch(self, bore, borePlane: adsk.fusion.ConstructionPlane) -> adsk.fusion.Sketch:
        component: adsk.fusion.Component = self._component()
        sketch: adsk.fusion.Sketch = component.sketches.add(borePlane)
        sketch.name = bore['name']
        sketch.isComputeDeferred = True
        index = bore['g']
        s = bore['start']/10
        theta = s/self.lam+self.phi[index]
        uB, uF, vLo, vHi = -self.hw, self.hw, bore['lo']/10, bore['hi']/10
        axisWorld: adsk.core.Point3D = self._axisWorld(index, s)
        cpWorld: adsk.core.Point3D = self._point(_add(self._xyz(axisWorld), _scale(self.us[index], self.A/2)))
        O: adsk.fusion.SketchPoint = sketch.sketchPoints.add(self._local(sketch, axisWorld))
        Cp: adsk.fusion.SketchPoint = sketch.sketchPoints.add(self._local(sketch, cpWorld))
        Ru: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True
        E: adsk.fusion.SketchPoint = sketch.sketchPoints.add(self._local(sketch, self._sectionWorld(index, s, uF, 0)))
        K: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, E)
        K.isConstruction = True
        corners = []
        for u, v in ((uB, vLo), (uF, vLo), (uF, vHi), (uB, vHi)):
            worldPoint: adsk.core.Point3D = self._sectionWorld(index, s, u, v)
            localPoint: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            localPoint.z = 0
            corners.append(sketch.sketchPoints.add(localPoint))
        p0: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[0])
        p1: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[1])
        p2: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[2])
        p3: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(corners[3])
        L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
        L2: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
        L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p2, p3)
        L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p3, p0)
        O.isFixed = True
        Cp.isFixed = True
        lengthText: adsk.core.Point3D = self._local(sketch, self._sectionWorld(index, s, uF/2, -self.ht/2))
        lengthDimension: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDimension.parameter.value = uF
        o: adsk.core.Point3D = O.geometry
        cp: adsk.core.Point3D = Cp.geometry
        e: adsk.core.Point3D = E.geometry
        ref = (cp.x-o.x, cp.y-o.y)
        length = math.hypot(*ref)
        ref = (ref[0]/length, ref[1]/length)
        if abs(math.sin(theta)) >= math.sqrt(0.5):
            ray = (e.x-o.x, e.y-o.y)
            length = math.hypot(*ray)
            ray = (ray[0]/length, ray[1]/length)
            angleText: adsk.core.Point3D = adsk.core.Point3D.create(
                o.x+(ref[0]+ray[0])*uF/3, o.y+(ref[1]+ray[1])*uF/3, 0)
            angleDimension: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)
            angleDimension.parameter.value = math.acos(min(1, max(-1, _dot(ref, ray))))
        else:
            a: adsk.core.Point3D = p1.geometry
            b: adsk.core.Point3D = p2.geometry
            ray = (b.x-a.x, b.y-a.y)
            length = math.hypot(*ray)
            ray = (ray[0]/length, ray[1]/length)
            den = ref[0]*ray[1]-ref[1]*ray[0]
            distance = ((a.x-o.x)*ray[1]-(a.y-o.y)*ray[0])/den
            intersection = (o.x+distance*ref[0], o.y+distance*ref[1])
            towardCp = (cp.x-intersection[0])*ref[0]+(cp.y-intersection[1])*ref[1]
            if abs(towardCp) < 1e-10:
                towardCp = (o.x-intersection[0])*ref[0]+(o.y-intersection[1])*ref[1]
            refRay = _scale(ref, 1 if towardCp >= 0 else -1)
            angleText: adsk.core.Point3D = adsk.core.Point3D.create(
                intersection[0]+(refRay[0]+ray[0])*uF/3,
                intersection[1]+(refRay[1]+ray[1])*uF/3, 0)
            angleDimension: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)
            angleDimension.parameter.value = math.acos(min(1, max(-1, _dot(refRay, ray))))
        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addParallel(L4, L2)
        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        text1: adsk.core.Point3D = self._local(sketch, self._sectionWorld(index, s, 0, vLo/2))
        text3: adsk.core.Point3D = self._local(sketch, self._sectionWorld(index, s, 0, vHi/2))
        text4: adsk.core.Point3D = self._local(sketch, self._sectionWorld(index, s, (uB+uF)/2, vLo/2))
        offset1: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L1, text1)
        offset1.parameter.value = -vLo
        offset3: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L3, text3)
        offset3.parameter.value = vHi
        offset4: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(L2, L4, text4)
        offset4.parameter.value = uF-uB
        sketch.isComputeDeferred = False
        self._checkSketch(sketch)
        return sketch

    def _cutBore(self, bore):
        component: adsk.fusion.Component = self._component()
        index = bore['g']
        key = 'bore-' if bore['sign'] < 0 else 'bore+'
        boreLine: adsk.fusion.SketchLine = adsk.fusion.SketchLine.cast(self.pathLines[index][key])
        planeInput: adsk.fusion.ConstructionPlaneInput = component.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(boreLine, adsk.core.ValueInput.createByReal(0))
        borePlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        borePlane.name = f'{bore["name"]} Plane'
        sketch: adsk.fusion.Sketch = self._boreSketch(bore, borePlane)
        profile: adsk.fusion.Profile = utilities.find_profile_by_curve_counts(sketch, lines=4)
        path: adsk.fusion.Path = component.features.createPath(boreLine, False)
        sweepInput: adsk.fusion.SweepFeatureInput = component.features.sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sOut, sIn, lam = self.sOut, self.sIn, self.lam
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal((sOut-sIn)/lam)
        cageBody: adsk.fusion.BRepBody = self.cageBody
        sweepInput.participantBodies = [cageBody]
        feature: adsk.fusion.SweepFeature = component.features.sweepFeatures.add(sweepInput)
        if feature.bodies.count != 1:
            raise ValueError(f'{bore["name"]} sweep cut: {feature.bodies.count} bodies, expected 1')
        cageBody = feature.bodies.item(0)
        self.cageBody = cageBody
        sc = bore['sign']*self.cageRadius
        for sign in (-1, 1):
            probe: adsk.core.Point3D = self._sectionWorld(index, sc, sign*(self.W/2+self.clearance/2), 0)
            actual = cageBody.pointContainment(probe)
            if actual != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise ValueError(f'{bore["name"]} sweep sense probe {sign}: containment={actual}, expected outside')

    def _windowWorld(self, window, t, z, depth: float = 0.0) -> adsk.core.Point3D:
        across = window['across']
        d = window['d']
        return self._world(t*across[0]+depth*d[0], t*across[1]+depth*d[1], z)

    def _fixedPolygon(self, sketch: adsk.fusion.Sketch, worldPoints):
        points = []
        for world in worldPoints:
            worldPoint: adsk.core.Point3D = adsk.core.Point3D.cast(world)
            localPoint: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            localPoint.z = 0
            points.append(sketch.sketchPoints.add(localPoint))
        for i in range(len(points)):
            startPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(points[i])
            endPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(points[(i+1)%len(points)])
            sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
        for value in points:
            point: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(value)
            point.isFixed = True

    def _singleProfile(self, sketch: adsk.fusion.Sketch) -> adsk.fusion.Profile:
        self._checkSketch(sketch)
        if sketch.profiles.count != 1:
            raise ValueError(f'{sketch.name}: {sketch.profiles.count} profiles, expected 1')
        return sketch.profiles.item(0)

    def _cutWindows(self):
        component: adsk.fusion.Component = self._component()
        planeInput: adsk.fusion.ConstructionPlaneInput = component.constructionPlanes.createInput()
        anchorLine: adsk.fusion.SketchLine = self.anchorLine
        if self.sigma <= math.pi/2:
            planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)
        else:
            planeInput.setByDistanceOnPath(anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'
        for window in self.windows:
            sketch: adsk.fusion.Sketch = component.sketches.add(windowPlane)
            sketch.name = f'Window {window["label"]}'
            self._fixedPolygon(sketch, [self._windowWorld(window, t, z) for t, z in window['corners']])
            profile: adsk.fusion.Profile = self._singleProfile(sketch)
            corners = window['corners']
            tc = sum(t for t, z in corners)/len(corners)
            zc = sum(z for t, z in corners)/len(corners)
            a0 = math.sqrt(max(0, self.Ri**2-tc*tc))
            a1 = math.sqrt(max(0, self.Ro**2-tc*tc))
            probe: adsk.core.Point3D = self._windowWorld(window, tc, zc, (a0+a1)/2)
            cageBody: adsk.fusion.BRepBody = self.cageBody
            actual = cageBody.pointContainment(probe)
            if actual != adsk.fusion.PointContainment.PointInsidePointContainment:
                raise ValueError(f'{sketch.name} before cut: containment={actual}, expected inside')
            directionProbe: adsk.core.Point3D = self._windowWorld(window, 0, 0, 1)
            localDirection: adsk.core.Point3D = sketch.modelToSketchSpace(directionProbe)
            direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if localDirection.z > 0
                         else adsk.fusion.ExtentDirections.NegativeExtentDirection)
            extrudeInput: adsk.fusion.ExtrudeFeatureInput = component.features.extrudeFeatures.createInput(
                profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
            Ro = self.Ro
            lengthValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(Ro+0.1)
            distanceExtent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(lengthValue)
            extrudeInput.setOneSideExtent(distanceExtent, direction)
            extrudeInput.participantBodies = [cageBody]
            feature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(extrudeInput)
            if feature.bodies.count != 1:
                raise ValueError(f'{sketch.name} cut: {feature.bodies.count} bodies, expected 1')
            cageBody = feature.bodies.item(0)
            self.cageBody = cageBody
            actual = cageBody.pointContainment(probe)
            if actual != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise ValueError(f'{sketch.name} after cut: containment={actual}, expected outside')

    def _markBores(self):
        component: adsk.fusion.Component = self._component()
        halfSize = min(0.1, self.collarHalf/2)
        inset = min(0.01, self.collarWall/2)
        gearBAxisPlane: adsk.fusion.ConstructionPlane = adsk.fusion.ConstructionPlane.cast(self.axisPlanes[1])
        normal: adsk.core.Vector3D = gearBAxisPlane.geometry.normal
        sign = 1 if normal.dotProduct(self.nHat) > 0 else -1
        signedOffset = sign*(self.cageRise-inset-self.A/2)
        planeInput: adsk.fusion.ConstructionPlaneInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(gearBAxisPlane, adsk.core.ValueInput.createByReal(signedOffset))
        markerPlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        markerPlane.name = 'Marker Plane'
        origin: adsk.core.Point3D = markerPlane.geometry.origin
        signedHeight = _dot(tuple(self._xyz(origin)[i]-self.center[i] for i in range(3)), self.n)
        if abs(signedHeight-(self.cageRise-inset)) > 1e-5:
            raise ValueError(f'Marker Plane: signed height {signedHeight} cm, expected {self.cageRise-inset} cm')
        for bore in self.bores:
            index, sigma = bore['g'], bore['sign']
            markCenter = _add(self.center, _add(_scale(self.n, self.cageRise-inset),
                                              _scale(self.dirs[index], sigma*self.cageRadius)))
            sketch: adsk.fusion.Sketch = component.sketches.add(markerPlane)
            sketch.name = f'{bore["name"]} {"Circle" if sigma > 0 else "Square"} Marker'
            if sigma > 0:
                localCentre: adsk.core.Point3D = self._local(sketch, self._point(markCenter))
                circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, halfSize)
                circle.centerSketchPoint.isFixed = True
                textPoint: adsk.core.Point3D = adsk.core.Point3D.create(localCentre.x+halfSize, localCentre.y, 0)
                dimension: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
                dimension.parameter.value = 2*halfSize
            else:
                worldPoints = [self._point(_add(markCenter, _add(_scale(self.e, t), _scale(self.k, k))))
                               for t, k in ((-halfSize, -halfSize), (halfSize, -halfSize),
                                            (halfSize, halfSize), (-halfSize, halfSize))]
                self._fixedPolygon(sketch, worldPoints)
            profile: adsk.fusion.Profile = self._singleProfile(sketch)
            directionProbe: adsk.core.Point3D = self._point(_add(markCenter, self.n))
            localDirection: adsk.core.Point3D = sketch.modelToSketchSpace(directionProbe)
            direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if localDirection.z > 0
                         else adsk.fusion.ExtentDirections.NegativeExtentDirection)
            extrudeInput: adsk.fusion.ExtrudeFeatureInput = component.features.extrudeFeatures.createInput(
                profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            lengthValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(inset+0.04)
            distanceExtent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(lengthValue)
            extrudeInput.setOneSideExtent(distanceExtent, direction)
            feature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(extrudeInput)
            if feature.bodies.count != 1:
                raise ValueError(f'{sketch.name} extrude: {feature.bodies.count} bodies, expected 1')
            markBody: adsk.fusion.BRepBody = feature.bodies.item(0)
            self.cageBody = self._join(self.cageBody, markBody, sketch.name)

    def relocateBodies(self):
        for index in (0, 1):
            body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[index])
            targetOccurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(self.gearOccs[index])
            body.moveToComponent(targetOccurrence)
        body: adsk.fusion.BRepBody = self.cageBody
        body.name = 'Cage'
        targetOccurrence: adsk.fusion.Occurrence = self.cageOcc
        body.moveToComponent(targetOccurrence)
