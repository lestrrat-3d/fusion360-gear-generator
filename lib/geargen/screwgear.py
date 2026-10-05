"""Build a pair of crossed screw ribbons and their printed sleeve."""

import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from . import solids
from .base import Generator
from .misc import get_design, to_cm
from .utilities import find_profile_by_curve_counts, get_normal


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


def _scale(a, x):
    return tuple(a[i] * x for i in range(3))


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    length = math.sqrt(_dot(a, a))
    if length <= 1e-12:
        raise RuntimeError('Screw gearing direction collapsed')
    return _scale(a, 1 / length)


def _xyz(point: adsk.core.Point3D):
    return (point.x, point.y, point.z)


def _point(v) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(v[0], v[1], v[2])


def _vector(v) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


def _local(sketch: adsk.fusion.Sketch, world, planar=True) -> adsk.core.Point3D:
    worldPoint: adsk.core.Point3D = adsk.core.Point3D.create(world[0], world[1], world[2])
    local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
    if planar:
        local.z = 0
    return local


def _fixed(sketch: adsk.fusion.Sketch, world, planar=True) -> adsk.fusion.SketchPoint:
    local: adsk.core.Point3D = _local(sketch, world, planar)
    return sketch.sketchPoints.add(local)


def _fully_constrained(sketch: adsk.fusion.Sketch):
    if not sketch.isFullyConstrained:
        raise RuntimeError(f'{sketch.name} is not fully constrained')


def _one_body(feature, label: str) -> adsk.fusion.BRepBody:
    featureBodies: adsk.fusion.BRepBodies = feature.bodies
    if featureBodies.count != 1:
        raise RuntimeError(f'{label} produced {featureBodies.count} bodies, expected one')
    body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(featureBodies.item(0))
    if not body or not body.isSolid:
        raise RuntimeError(f'{label} did not produce one solid body')
    return body


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs: adsk.core.CommandInputs = command.commandInputs
        planeInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to")
        planeInput.addSelectionFilter('ConstructionPlanes')
        planeInput.addSelectionFilter('PlanarFaces')
        planeInput.setSelectionLimits(1, 1)
        pointInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism')
        pointInput.addSelectionFilter('ConstructionPoints')
        pointInput.addSelectionFilter('SketchPoints')
        pointInput.setSelectionLimits(1, 1)
        parentInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under')
        parentInput.addSelectionFilter('Occurrences')
        parentInput.addSelectionFilter('RootComponents')
        parentInput.setSelectionLimits(1, 1)
        rootComponent: adsk.fusion.Component = get_design().rootComponent
        parentInput.addSelection(rootComponent)

        groups = (
            ('ribbonGroup', 'Ribbon', True, (
                (INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', 15),
                (INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', 68),
                (INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', 49.5),
                (INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', 3.75),
                (INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', 2.625),
                (INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', 2.625),
                (INPUT_ID_TOOTH_SLANT, 'Tooth Slant', 'deg', 25.8),
                (INPUT_ID_TOOTH_BOW, 'Tooth Bow', '', 0.048),
            )),
            ('frameGroup', 'Frame', True, (
                (INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', 15),
                (INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', 18.75),
                (INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.20),
                (INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', 0.60),
                (INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 3),
                (INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 3),
            )),
            ('meshGroup', 'Mesh (from the mesh search)', False, (
                (INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', 80),
                (INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 1.05),
                (INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', 0),
                (INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', 0),
                (INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -1.31),
            )),
        )
        for groupId, groupLabel, expanded, entries in groups:
            group: adsk.core.GroupCommandInput = inputs.addGroupCommandInput(groupId, groupLabel)
            group.isExpanded = expanded
            for inputId, label, unit, default in entries:
                real = math.radians(default) if unit == 'deg' else (to_cm(default) if unit == 'mm' else default)
                initialValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(real)
                group.children.addValueInput(inputId, label, unit, initialValue)


class ScrewGearGenerator(Generator):
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

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Read all selected entities before creating an occurrence.
        for inputId in (INPUT_ID_PLANE, INPUT_ID_POINT, INPUT_ID_PARENT):
            selectionInput: adsk.core.SelectionCommandInput = adsk.core.SelectionCommandInput.cast(inputs.itemById(inputId))
            if not selectionInput or selectionInput.selectionCount != 1:
                raise ValueError(f'{inputId} requires one selection')
            selected = selectionInput.selection(0).entity
            if inputId == INPUT_ID_PLANE:
                self.plane = selected
            elif inputId == INPUT_ID_POINT:
                self.centreSelection = selected
            else:
                occurrence: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(selected)
                self.parentComponent = occurrence.component if occurrence else adsk.fusion.Component.cast(selected)
        if not self.parentComponent:
            raise ValueError('parent must select a component or occurrence')
        units: adsk.core.UnitsManager = self.design.unitsManager
        self.p = {}
        for inputId in (INPUT_ID_RIBBON_WIDTH, INPUT_ID_TOOTH_COUNT, INPUT_ID_TWIST_LEAD,
                        INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH, INPUT_ID_TOOTH_HEIGHT,
                        INPUT_ID_TOOTH_SLANT, INPUT_ID_TOOTH_BOW, INPUT_ID_CAGE_RADIUS,
                        INPUT_ID_CAGE_RISE, INPUT_ID_CLEARANCE, INPUT_ID_ROOF_ALLOWANCE,
                        INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CROSS_ANGLE,
                        INPUT_ID_ENGAGEMENT, INPUT_ID_MOUNT_ANGLE_A, INPUT_ID_MOUNT_ANGLE_B,
                        INPUT_ID_ASSEMBLY_PHASE):
            input: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(inputs.itemById(inputId))
            if not input:
                raise ValueError(f'Missing input {inputId}')
            unit = 'deg' if inputId in (INPUT_ID_TOOTH_SLANT, INPUT_ID_CROSS_ANGLE,
                                       INPUT_ID_MOUNT_ANGLE_A, INPUT_ID_MOUNT_ANGLE_B) else (
                '' if inputId in (INPUT_ID_TOOTH_COUNT, INPUT_ID_TOOTH_BOW) else 'mm')
            self.p[inputId] = units.evaluateExpression(input.expression, unit)
        p = self.p
        for key in (INPUT_ID_RIBBON_WIDTH, INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH,
                    INPUT_ID_TWIST_LEAD, INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CLEARANCE):
            if p[key] <= 0:
                raise ValueError(f'{key} must be > 0')
        if p[INPUT_ID_ROOF_ALLOWANCE] < 0:
            raise ValueError('roofAllowance must be >= 0')
        count = p[INPUT_ID_TOOTH_COUNT]
        if count < 4 or count != int(count):
            raise ValueError('toothCount must be a whole number >= 4')
        if not (0 < p[INPUT_ID_TOOTH_HEIGHT] < p[INPUT_ID_RIBBON_WIDTH] / 2):
            raise ValueError('toothHeight must be > 0 and < ribbonWidth/2')
        if not (-math.pi / 2 < p[INPUT_ID_TOOTH_SLANT] < math.pi / 2):
            raise ValueError('toothSlant must be strictly between -90 and 90 degrees')
        if p[INPUT_ID_TOOTH_BOW] < 0 or (p[INPUT_ID_TOOTH_HEIGHT] +
            p[INPUT_ID_TOOTH_BOW] * (p[INPUT_ID_RIBBON_THICKNESS] * 5) ** 2 / 10 >=
            p[INPUT_ID_RIBBON_WIDTH] / 2):
            raise ValueError('toothBow must keep the face root below ribbonWidth/2')
        if not (0 < p[INPUT_ID_ENGAGEMENT] <= p[INPUT_ID_TOOTH_HEIGHT]):
            raise ValueError('engagement must be > 0 and <= toothHeight')
        if not (0 < p[INPUT_ID_CROSS_ANGLE] < math.pi):
            raise ValueError('crossAngle must be strictly between 0 and 180 degrees')
        if abs(p[INPUT_ID_ASSEMBLY_PHASE]) >= p[INPUT_ID_TOOTH_PITCH]:
            raise ValueError('assemblyPhase must be within +/-toothPitch')
        if (p[INPUT_ID_CAGE_RADIUS] + p[INPUT_ID_COLLAR_HALF] + to_cm(1) >=
            int(count) * p[INPUT_ID_TOOTH_PITCH] / 2):
            raise ValueError('cageRadius must keep bores inside ribbon length')
        self.lam = p[INPUT_ID_TWIST_LEAD] / (2 * math.pi)
        self.offset = p[INPUT_ID_RIBBON_WIDTH] - p[INPUT_ID_ENGAGEMENT]
        self.ri = p[INPUT_ID_CAGE_RADIUS] - p[INPUT_ID_COLLAR_HALF]
        self.ro = p[INPUT_ID_CAGE_RADIUS] + p[INPUT_ID_COLLAR_HALF]
        self.hw = p[INPUT_ID_RIBBON_WIDTH] / 2 + p[INPUT_ID_CLEARANCE]
        self.ht = p[INPUT_ID_RIBBON_THICKNESS] / 2 + p[INPUT_ID_CLEARANCE]
        c = math.hypot(self.hw, self.ht + p[INPUT_ID_ROOF_ALLOWANCE])
        if math.hypot(c, to_cm(1)) >= self.ri:
            raise ValueError('cageRadius must keep the channel start inside the hollow')
        self.sIn = math.sqrt(self.ri ** 2 - c ** 2) - to_cm(1)
        self.sOut = self.ro + to_cm(1)
        axialWindow = 1.5 * math.sqrt(p[INPUT_ID_RIBBON_WIDTH] ** 2 - self.offset ** 2) / math.sin(p[INPUT_ID_CROSS_ANGLE])
        if math.hypot(axialWindow, math.hypot(p[INPUT_ID_RIBBON_WIDTH]/2,
                                                p[INPUT_ID_RIBBON_THICKNESS]/2)) + p[INPUT_ID_CLEARANCE] > self.ri:
            raise ValueError('cageRadius must keep the mesh visible along the axis')
        if p[INPUT_ID_CAGE_RISE] < self.offset / 2 + c + p[INPUT_ID_COLLAR_WALL]:
            raise ValueError('cageRise must leave collarWall at each end')
        self.stepsPerTooth = max(8, math.ceil((p[INPUT_ID_TOOTH_PITCH] / self.lam) / math.radians(2)))
        self.cellTeeth = min(CELL_TEETH, int(count))
        self.wholeCells, self.remainder = divmod(int(count), self.cellTeeth)
        self._prepare_search_frame()
        self.levelSigns = [self._level_sign(g) for g in range(2)]
        self.bores = [self._bore(g, sigma) for g in range(2) for sigma in (-1, 1)]
        separation, names = self._channel_separation()
        if separation < p[INPUT_ID_COLLAR_WALL]:
            raise ValueError(f'collarWall requires {p[INPUT_ID_COLLAR_WALL]*10:.3f} mm between {names}; got {separation*10:.3f} mm')
        self.windows = self._search_windows()

    def _prepare_search_frame(self):
        sigma = self.p[INPUT_ID_CROSS_ANGLE] / 2
        e, n = (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
        self.searchE, self.searchN, self.searchK = e, n, (0.0, 1.0, 0.0)
        self.searchGears = []
        for index, hand in ((0, 1), (1, -1)):
            direction = (math.cos(sigma), hand * math.sin(sigma), 0.0)
            u = _scale(n, hand)
            v = _cross(direction, u)
            origin = (0.0, 0.0, -hand * self.offset / 2)
            self.searchGears.append(dict(index=index, label=f'Gear {"A" if index == 0 else "B"}',
                direction=direction, u=u, v=v, origin=origin,
                mount=self.p[INPUT_ID_MOUNT_ANGLE_A if index == 0 else INPUT_ID_MOUNT_ANGLE_B],
                phase=0 if index == 0 else self.p[INPUT_ID_ASSEMBLY_PHASE]))

    def _angle(self, gear, station):
        return station / self.lam + gear['mount']

    def _world(self, gear, u, v, station):
        theta = self._angle(gear, station)
        return _add(_add(gear['origin'], _scale(gear['direction'], station)),
                    _add(_scale(gear['u'], u * math.cos(theta) - v * math.sin(theta)),
                         _scale(gear['v'], u * math.sin(theta) + v * math.cos(theta))))

    def _crossing(self, bore):
        gear = self.searchGears[int(bore['gear'])]
        return _add(gear['origin'], _scale(gear['direction'], bore['sign'] * self.p[INPUT_ID_CAGE_RADIUS]))

    def _level_sign(self, index):
        gear = self.searchGears[index]
        radius, half = self.p[INPUT_ID_CAGE_RADIUS], self.p[INPUT_ID_COLLAR_HALF]
        def tilt(sign):
            lo = self._angle(gear, sign * (radius - half))
            hi = self._angle(gear, sign * (radius + half))
            lo, hi = min(lo, hi), max(lo, hi)
            level = math.pi/2 + math.ceil((lo-math.pi/2)/math.pi)*math.pi
            return 0.0 if level <= hi else min(abs(math.cos(lo)), abs(math.cos(hi)))
        return 1 if tilt(1) < tilt(-1) else -1

    def _bore(self, index, sign):
        gear = self.searchGears[index]
        vLo, vHi = -self.ht, self.ht
        roofSign = 1 if -math.sin(self._angle(gear, sign*self.p[INPUT_ID_CAGE_RADIUS])) * gear['u'][2] > 0 else -1
        if sign == self.levelSigns[index]:
            if roofSign > 0:
                vHi += self.p[INPUT_ID_ROOF_ALLOWANCE]
            else:
                vLo -= self.p[INPUT_ID_ROOF_ALLOWANCE]
        start, end = (self.sIn, self.sOut) if sign > 0 else (-self.sOut, -self.sIn)
        label = f'{gear["label"]} Bore {"+R" if sign > 0 else "-R"}'
        return dict(gear=index, sign=sign, roofSign=roofSign, vLo=vLo, vHi=vHi,
                    start=start, end=end, label=label)

    def _opening(self, bore):
        return [(-self.hw, bore['vLo']), (self.hw, bore['vLo']),
                (self.hw, bore['vHi']), (-self.hw, bore['vHi'])]

    def _outline(self, bore):
        gear = self.searchGears[int(bore['gear'])]
        sign = bore['sign']
        s = sign * self.sIn
        while abs(s) <= self.sOut:
            for i in range(17):
                k = i/16
                v = bore['vLo'] + (bore['vHi']-bore['vLo'])*k
                u = -self.hw + 2*self.hw*k
                for x, y in ((self.hw, v), (-self.hw, v), (u, bore['vHi']), (u, bore['vLo'])):
                    pt = self._world(gear, x, y, s)
                    r = math.hypot(pt[0], pt[1])
                    if self.ri-to_cm(0.5) <= r <= self.ro+to_cm(0.5):
                        yield pt
            s += sign * to_cm(0.1)

    @staticmethod
    def _hull(points):
        points = sorted(set(points))
        if len(points) <= 1:
            return points
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
        low, high = [], []
        for p in points:
            while len(low) >= 2 and cross(low[-2], low[-1], p) <= 0:
                low.pop()
            low.append(p)
        for p in reversed(points):
            while len(high) >= 2 and cross(high[-2], high[-1], p) <= 0:
                high.pop()
            high.append(p)
        return low[:-1]+high[:-1]

    def _channel_separation(self):
        # The four crossings alternate about the tube. Project each adjacent pair
        # to the plane through its angular gap and measure separated convex hulls.
        ordered = sorted(self.bores, key=lambda b: math.atan2(self._crossing(b)[1], self._crossing(b)[0]) % (2*math.pi))
        best, bestNames = math.inf, ''
        for i in range(4):
            a, b = ordered[i], ordered[(i+1) % 4]
            ca, cb = self._crossing(a), self._crossing(b)
            facing = _unit((ca[0]/math.hypot(ca[0], ca[1])+cb[0]/math.hypot(cb[0], cb[1]),
                            ca[1]/math.hypot(ca[0], ca[1])+cb[1]/math.hypot(cb[0], cb[1]), 0))
            across = _cross(self.searchN, facing)
            hulls = [self._hull([(_dot(pt, across), pt[2]) for pt in self._outline(item)]) for item in (a, b)]
            if any(len(h) < 2 for h in hulls):
                raise RuntimeError(f'Cannot measure wall between {a["label"]} and {b["label"]}')
            gap = -math.inf
            for hull in hulls:
                for j, p in enumerate(hull):
                    q = hull[(j+1) % len(hull)]
                    edge = (q[0]-p[0], q[1]-p[1])
                    norm = math.hypot(*edge)
                    if norm <= 1e-12:
                        continue
                    m = (-edge[1]/norm, edge[0]/norm)
                    proj = [[u*m[0]+v*m[1] for u, v in h] for h in hulls]
                    gap = max(gap, min(proj[1])-max(proj[0]), min(proj[0])-max(proj[1]))
            if gap < best:
                best, bestNames = gap, f'{a["label"]} and {b["label"]}'
        return best, bestNames

    @staticmethod
    def _clip(poly, a, b, c):
        if not poly:
            return []
        out = []
        for i, p in enumerate(poly):
            q = poly[(i+1) % len(poly)]
            fp, fq = c-a*p[0]-b*p[1], c-a*q[0]-b*q[1]
            if fp >= 0:
                out.append(p)
            if (fp >= 0) != (fq >= 0):
                t = fp/(fp-fq)
                out.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
        return out

    def _section_in_wall(self, bore, station):
        if abs(station) >= self.ro:
            return ([], [])
        gear = self.searchGears[int(bore['gear'])]
        theta = self._angle(gear, station)
        poly = [(u*math.cos(theta)-v*math.sin(theta), u*math.sin(theta)+v*math.cos(theta))
                for u, v in self._opening(bore)]
        xa = (-self.p[INPUT_ID_CAGE_RISE]-gear['origin'][2])/gear['u'][2]
        xb = (self.p[INPUT_ID_CAGE_RISE]-gear['origin'][2])/gear['u'][2]
        xa, xb = min(xa, xb), max(xa, xb)
        poly = self._clip(self._clip(poly, 1, 0, xb), -1, 0, -xa)
        near = math.sqrt(max(0, self.ri*self.ri-station*station))
        far = math.sqrt(self.ro*self.ro-station*station)
        return tuple(self._clip(self._clip(poly, 0, side, far), 0, -side, -near) for side in (-1, 1))

    def _wall_corners(self, bore):
        gear = self.searchGears[int(bore['gear'])]
        s = bore['start']
        while s <= bore['end']:
            for piece in self._section_in_wall(bore, s):
                for x, y in piece:
                    yield _add(_add(gear['origin'], _scale(gear['u'], x)),
                               _add(_scale(gear['v'], y), _scale(gear['direction'], s)))
            s += to_cm(0.001)

    def _channel_top(self):
        top = 0.0
        for bore in self.bores:
            gear = self.searchGears[int(bore['gear'])]
            s = bore['sign']*self.sIn
            while abs(s) <= self.sOut:
                if abs(s) <= self.ro:
                    theta = self._angle(gear, s)
                    corners = [(u*math.cos(theta)-v*math.sin(theta),
                                u*math.sin(theta)+v*math.cos(theta)) for u, v in self._opening(bore)]
                    reach = max(abs(y) for _, y in corners)
                    if math.hypot(s, reach) >= self.ri:
                        top = max(top, *(abs(gear['origin'][2]+x*gear['u'][2]) for x, _ in corners))
                s += bore['sign']*to_cm(0.01)
        return top

    def _channel_stations(self, bore):
        out = []
        def make(station):
            pieces = self._section_in_wall(bore, station)
            bounds = []
            for piece in pieces:
                if piece:
                    cx = sum(p[0] for p in piece)/len(piece)
                    cy = sum(p[1] for p in piece)/len(piece)
                    bounds.append((cx, cy, max(math.hypot(p[0]-cx, p[1]-cy) for p in piece)))
                else:
                    bounds.append((0, 0, 0))
            return station, pieces, bounds
        i = 0
        while True:
            s = bore['start']+i*to_cm(0.002)
            if s > bore['end']:
                break
            entry = make(s)
            if out and tuple(bool(p) for p in entry[1]) != tuple(bool(p) for p in out[-1][1]):
                prev = out[-1][0]
                j = 1
                while prev+j*to_cm(0.0001) < s:
                    out.append(make(prev+j*to_cm(0.0001)))
                    j += 1
            out.append(entry)
            i += 1
        return out

    @staticmethod
    def _polygon_gap2(piece, x, y):
        inside, best = len(piece) >= 3, math.inf
        for i, p in enumerate(piece):
            q = piece[(i+1) % len(piece)]
            ex, ey = q[0]-p[0], q[1]-p[1]
            if ex*(y-p[1])-ey*(x-p[0]) < 0:
                inside = False
            length2 = ex*ex+ey*ey
            t = max(0, min(1, ((x-p[0])*ex+(y-p[1])*ey)/length2)) if length2 > 0 else 0
            best = min(best, (x-p[0]-t*ex)**2+(y-p[1]-t*ey)**2)
        return 0 if inside else best

    def _wall_gap(self, bore, point, reach):
        gear = self.searchGears[int(bore['gear'])]
        d = _sub(point, gear['origin'])
        x, y, sq = _dot(d, gear['u']), _dot(d, gear['v']), _dot(d, gear['direction'])
        axis = math.hypot(math.hypot(x, y), sq-max(bore['start'], min(bore['end'], sq)))
        corner = math.hypot(self.hw, self.ht+self.p[INPUT_ID_ROOF_ALLOWANCE])
        if axis-corner >= reach:
            return reach
        best = reach*reach
        stations = self.stationTables[bore['label']]
        values = self.stationValues[bore['label']]
        low, high = 0, len(values)
        while low < high:
            middle = (low+high)//2
            if values[middle] < sq:
                low = middle+1
            else:
                high = middle
        at = low
        def visit(entry):
            nonlocal best
            s, pieces, bounds = entry
            ds = (sq-s)**2
            if ds >= best:
                return False
            for piece, (cx, cy, radius) in zip(pieces, bounds):
                if not piece:
                    continue
                outside = math.hypot(x-cx, y-cy)-radius
                if outside > 0 and ds+outside**2 >= best:
                    continue
                best = min(best, ds+self._polygon_gap2(piece, x, y))
            return True
        for i in range(at, len(stations)):
            if not visit(stations[i]):
                break
        for i in range(at-1, -1, -1):
            if not visit(stations[i]):
                break
        return math.sqrt(best)

    def _search_windows(self):
        if self.p[INPUT_ID_CROSS_ANGLE] <= math.pi/2:
            facings = [(self.searchK, '+k'), (_scale(self.searchK, -1), '-k')]
        else:
            facings = [(self.searchE, '+e'), (_scale(self.searchE, -1), '-e')]
        self.stationTables = {}
        self.stationValues = {}
        for bore in self.bores:
            table = self._channel_stations(bore)
            self.stationTables[bore['label']] = table
            self.stationValues[bore['label']] = [entry[0] for entry in table]
        zLimit = self._channel_top()
        windows = []
        for facing, name in facings:
            across = _cross(self.searchN, facing)
            flanks = [b for b in self.bores if _dot(self._crossing(b), facing) > 0]
            far = [b for b in self.bores if b not in flanks]
            if len(flanks) != 2 or len(far) != 2:
                raise RuntimeError(f'Window {name} does not have two flanking and two far bores')
            low, high = sorted(flanks, key=lambda b: self._crossing(b)[2])
            lean = 1 if _dot(self._crossing(high), across) >= _dot(self._crossing(low), across) else -1
            lowReach = max(_dot(pt, self.searchN)+lean*_dot(pt, across) for pt in self._wall_corners(low))
            highReach = min(_dot(pt, self.searchN)+lean*_dot(pt, across) for pt in self._wall_corners(high))
            wall = self.p[INPUT_ID_COLLAR_WALL]
            lo, hi = lowReach+math.sqrt(2)*wall, highReach-math.sqrt(2)*wall
            if hi <= lo:
                futil.log(f'No window facing {name}: flanking bores leave no band')
                continue
            top = min(2*zLimit-hi, hi+math.sqrt(2)*self.ri)
            bottom = max(-2*zLimit-lo, lo-math.sqrt(2)*self.ri)
            window: dict = dict(name=name, facing=facing, across=across, lean=lean,
                                lo=lo, hi=hi, top=top, bottom=bottom, zLimit=zLimit)
            window['left'] = -self._window_end(window, far, -1)
            window['right'] = self._window_end(window, far, 1)
            if window['right'] <= window['left']:
                futil.log(f'No window facing {name}: far bores leave no length')
                continue
            square = [(-2*self.ro, -2*self.ro), (2*self.ro, -2*self.ro),
                      (2*self.ro, 2*self.ro), (-2*self.ro, 2*self.ro)]
            limits = ((lean, 1, hi), (-lean, -1, -lo), (-lean, 1, top),
                      (lean, -1, -bottom), (1, 0, window['right']),
                      (-1, 0, -window['left']))
            for a, b, c in limits:
                square = self._clip(square, a, b, c)
            corners = []
            for pt in square:
                if not corners or math.hypot(pt[0]-corners[-1][0], pt[1]-corners[-1][1]) >= to_cm(0.001):
                    corners.append(pt)
            if len(corners) > 1 and math.hypot(corners[0][0]-corners[-1][0],
                                               corners[0][1]-corners[-1][1]) < to_cm(0.001):
                corners.pop()
            area = sum(corners[i][0]*corners[(i+1)%len(corners)][1] -
                       corners[(i+1)%len(corners)][0]*corners[i][1]
                       for i in range(len(corners))) / 2 if len(corners) >= 3 else 0
            if len(corners) < 3 or area <= 0:
                futil.log(f'No window facing {name}: no closed hexagon area')
                continue
            window['corners'] = corners
            windows.append(window)
        return windows

    def _window_end(self, window, far, side):
        need = self.p[INPUT_ID_COLLAR_WALL] + to_cm(0.1)/math.sqrt(2) + to_cm(0.005)
        step = to_cm(0.1)
        def at(t, z, a):
            return _add(_add(_scale(window['across'], t), _scale(self.searchN, z)),
                        _scale(window['facing'], a))
        def near(t, z, a):
            pt = at(t, z, a)
            return any(self._wall_gap(b, pt, need) < need for b in far)
        def clear(distance):
            t = side*distance
            zLow = max(window['lo']-window['lean']*t, window['bottom']+window['lean']*t)
            zHigh = min(window['hi']-window['lean']*t, window['top']+window['lean']*t)
            if zLow > zHigh:
                return False
            a0 = math.sqrt(max(0, self.ri**2-t**2))
            a1 = math.sqrt(max(0, self.ro**2-t**2))
            for z in (zLow, zHigh):
                a = a0
                while True:
                    if near(t, z, a):
                        return False
                    if a >= a1:
                        break
                    a = min(a+step, a1)
            n = math.ceil((zHigh-zLow)/step)
            for a in (a0, a1):
                for k in range(1, n):
                    if near(t, zLow+(zHigh-zLow)*k/n, a):
                        return False
                j = 0
                while -window['zLimit']+j*step <= window['zLimit']:
                    z = -window['zLimit']+j*step
                    if zLow < z < zHigh and near(t, z, a):
                        return False
                    j += 1
            return True
        lo, hi = 0.0, min(self.ri, self.ro/math.sqrt(2))*(1-1e-9)
        for _ in range(24):
            mid = (lo+hi)/2
            if clear(mid):
                lo = mid
            else:
                hi = mid
        return lo

    def buildComponentTree(self):
        self.designOcc: adsk.fusion.Occurrence = self.getOccurrence()
        topComponent: adsk.fusion.Component = self.designOcc.component
        topComponent.name = 'Screw Gearing'
        children = []
        for name in ('Design', 'Gear A', 'Gear B', 'Cage'):
            identity: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
            occ: adsk.fusion.Occurrence = topComponent.occurrences.addNewComponent(identity)
            occ.component.name = name
            children.append(occ)
        self.designChild: adsk.fusion.Occurrence = children[0]
        self.designComponent: adsk.fusion.Component = self.designChild.component
        self.gearOccs = children[1:3]
        self.cageOcc: adsk.fusion.Occurrence = children[3]
        self.gearBodies = [adsk.fusion.BRepBody.cast(None)] * 2
        self.cageBody: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]

    def buildAnchor(self):
        design: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = design.sketches.add(self.plane)
        sketch.name = 'Anchor'
        projected: adsk.core.ObjectCollection = sketch.project(self.centreSelection)
        if not projected or projected.count != 1:
            raise RuntimeError(f'Anchor projection produced {projected.count if projected else 0} points')
        projectedPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projected.item(0))
        centre = projectedPoint.geometry
        startSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre.x-0.5, centre.y, 0)
        endSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+0.5, centre.y, 0)
        anchorLine: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)
        sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)
        sketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x, centre.y+0.2, 0)
        lengthDim: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        lengthDim.parameter.value = 1.0
        _fully_constrained(sketch)
        self.anchorLine = anchorLine
        self.C = _xyz(projectedPoint.worldGeometry)
        self.eHat = _unit(_sub(_xyz(anchorLine.endSketchPoint.worldGeometry),
                               _xyz(anchorLine.startSketchPoint.worldGeometry)))
        self.axisPlanes = []
        for index, offset in ((0, -self.offset/2), (1, self.offset/2)):
            planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
            value: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(offset)
            planeInput.setByOffset(self.plane, value)
            axisPlane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
            axisPlane.name = f'Gear {"A" if index == 0 else "B"} Axis Plane'
            self.axisPlanes.append(axisPlane)
        normal: adsk.core.Vector3D = get_normal(self.axisPlanes[0])
        raw = _unit((normal.x, normal.y, normal.z))
        self.nHat = raw if _dot(_sub(self.C, _xyz(self.axisPlanes[0].geometry.origin)), raw) > 0 else _scale(raw, -1)
        for axisPlane in self.axisPlanes:
            if abs(abs(_dot(_sub(self.C, _xyz(axisPlane.geometry.origin)), self.nHat)) - self.offset/2) > 1e-4:
                raise RuntimeError(f'{axisPlane.name} is not offset A/2 from the centre')
        self.kHat = _unit(_cross(self.nHat, self.eHat))
        self.gears = []
        sigma = self.p[INPUT_ID_CROSS_ANGLE] / 2
        for index, sign in ((0, 1), (1, -1)):
            direction = _unit(_add(_scale(self.eHat, math.cos(sigma)),
                                   _scale(self.kHat, sign*math.sin(sigma))))
            u = _scale(self.nHat, sign)
            v = _unit(_cross(direction, u))
            origin = _add(self.C, _scale(self.nHat, -sign*self.offset/2))
            self.gears.append(dict(index=index, label=f'Gear {"A" if index == 0 else "B"}',
                direction=direction, u=u, v=v, origin=origin,
                mount=self.p[INPUT_ID_MOUNT_ANGLE_A if index == 0 else INPUT_ID_MOUNT_ANGLE_B],
                phase=0 if index == 0 else self.p[INPUT_ID_ASSEMBLY_PHASE]))

    def buildGear(self, index):
        self.buildSweepPaths(index)
        cell = self.buildToothCell(index)
        self.gearBodies[index] = self.repeatCellByDoubling(index, cell)
        self.gearBodies[index].name = self.gears[index]['label']

    def buildSweepPaths(self, index):
        design: adsk.fusion.Component = self.designComponent
        gear = self.gears[index]
        plane: adsk.fusion.ConstructionPlane = self.axisPlanes[index]
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = f'{gear["label"]} Paths'
        stations = (-self.sOut, -self.sIn, self.sIn, self.sOut)
        points = []
        for station in stations:
            world = _add(gear['origin'], _scale(gear['direction'], station))
            point: adsk.fusion.SketchPoint = _fixed(sketch, world)
            points.append(point)
        minus: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[0], points[1])
        plus: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[2], points[3])
        for point in points:
            point.isFixed = True
        _fully_constrained(sketch)
        self.pathLines[index] = {'bore-': minus, 'bore+': plus}

    def _tooth_edge(self, gear, station, v, slant=None):
        p = self.p
        if slant is None:
            slant = math.tan(p[INPUT_ID_TOOTH_SLANT])
        # Tooth bow is specified in mm^-1, while this generator uses cm.
        bow = 10*p[INPUT_ID_TOOTH_BOW]
        return (p[INPUT_ID_RIBBON_WIDTH]/2-p[INPUT_ID_TOOTH_HEIGHT]/2+
                p[INPUT_ID_TOOTH_HEIGHT]/2*math.cos(2*math.pi*(station+slant*v-gear['phase'])/
                                                    p[INPUT_ID_TOOTH_PITCH])-bow*v*v)

    def _cell_sections(self, index, count, start, name):
        design: adsk.fusion.Component = self.designComponent
        gear = self.gears[index]
        plane: adsk.fusion.ConstructionPlane = self.axisPlanes[index]
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = name
        sketch.isComputeDeferred = True
        sections = []
        pointsToFix = []
        splines = []
        width = self.p[INPUT_ID_RIBBON_WIDTH]
        thickness = self.p[INPUT_ID_RIBBON_THICKNESS]
        pitch = self.p[INPUT_ID_TOOTH_PITCH]
        for k in range(count*self.stepsPerTooth+1):
            station = start+k*pitch/self.stepsPerTooth
            B0: adsk.fusion.SketchPoint = _fixed(sketch, self._world(gear, -width/2, -thickness/2, station), False)
            B1: adsk.fusion.SketchPoint = _fixed(sketch, self._world(gear, -width/2, thickness/2, station), False)
            front = []
            for j in range(TOOTH_SPLINE_POINTS):
                v = -thickness/2+j*thickness/(TOOTH_SPLINE_POINTS-1)
                F: adsk.fusion.SketchPoint = _fixed(sketch, self._world(gear, self._tooth_edge(gear, station, v), v, station), False)
                front.append(F)
            L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, front[0])
            fitPoints: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for sectionPoint in front:
                fitPoints.add(sectionPoint)
            spline: adsk.fusion.SketchFittedSpline = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
            if not spline or spline.fitPoints.count != TOOTH_SPLINE_POINTS:
                raise RuntimeError(f'{name} section {k} spline has {spline.fitPoints.count if spline else 0} fit points')
            L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(front[-1], B1)
            L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)
            sections.append((L1, spline, L3, L4))
            pointsToFix.extend((B0, B1, *front))
            splines.append(spline)
        for point in pointsToFix:
            point.isFixed = True
        for spline in splines:
            for i in range(spline.fitPoints.count):
                fit: adsk.fusion.SketchPoint = spline.fitPoints.item(i)
                fit.isFixed = True
        sketch.isComputeDeferred = False
        _fully_constrained(sketch)
        if sketch.profiles.count != len(sections):
            raise RuntimeError(f'{name} has {sketch.profiles.count} profiles, expected {len(sections)}')
        return sections

    def _loft_cell(self, sections, label) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designComponent
        loftInput: adsk.fusion.LoftFeatureInput = design.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for section in sections:
            if len(section) != 4:
                raise RuntimeError(f'{label} section has {len(section)} curves, expected four')
            curves: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for sectionCurve in section:
                curves.add(sectionCurve)
            sectionPath: adsk.fusion.Path = design.features.createPath(curves, False)
            loftInput.loftSections.add(sectionPath)
        loftFeature: adsk.fusion.LoftFeature = design.features.loftFeatures.add(loftInput)
        return _one_body(loftFeature, label)

    def buildToothCell(self, index) -> adsk.fusion.BRepBody:
        gear = self.gears[index]
        start = gear['phase']-int(self.p[INPUT_ID_TOOTH_COUNT])*self.p[INPUT_ID_TOOTH_PITCH]/2
        sections = self._cell_sections(index, self.cellTeeth, start, f'{gear["label"]} Cell Sections')
        cellBody: adsk.fusion.BRepBody = self._loft_cell(sections, f'{gear["label"]} Cell')
        self._check_tooth_slant(gear, cellBody, start)
        return cellBody

    def _check_tooth_slant(self, gear, cellBody: adsk.fusion.BRepBody, start):
        p = self.p
        pitch = p[INPUT_ID_TOOTH_PITCH]
        crest = gear['phase']+pitch*math.ceil((start+pitch/2-gear['phase'])/pitch)
        slant = math.tan(p[INPUT_ID_TOOTH_SLANT])
        used = 0
        for faceSign in (-1, 1):
            v = faceSign*(p[INPUT_ID_RIBBON_THICKNESS]/2-to_cm(0.25))
            u = p[INPUT_ID_RIBBON_WIDTH]/2-10*p[INPUT_ID_TOOTH_BOW]*v*v-to_cm(0.25)
            for probeName, station in (('on-ridge', crest-slant*v), ('off-ridge', crest+slant*v)):
                correct = self._tooth_edge(gear, station, v, slant)-u
                wrong = self._tooth_edge(gear, station, v, -slant)-u
                if min(abs(correct), abs(wrong)) < to_cm(0.1) or correct*wrong >= 0:
                    continue
                probePoint: adsk.core.Point3D = _point(self._world(gear, u, v, station))
                observed = cellBody.pointContainment(probePoint)
                expected = (adsk.fusion.PointContainment.PointInsidePointContainment if correct > 0 else
                            adsk.fusion.PointContainment.PointOutsidePointContainment)
                if observed != expected:
                    raise RuntimeError(f'{gear["label"]} {probeName} slant probe at v={v*10:.3f} mm read {observed}; expected {expected}')
                used += 1
        if used == 0:
            futil.log(f'{gear["label"]} tooth slant sign was not checked at this angle')

    def _copy_body(self, sourceBody: adsk.fusion.BRepBody, label) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designComponent
        copyFeature: adsk.fusion.CopyPasteBody = design.features.copyPasteBodies.add(sourceBody)
        return _one_body(copyFeature, label)

    def _move_step(self, copyBody: adsk.fusion.BRepBody, index, k):
        design: adsk.fusion.Component = self.designComponent
        gear = self.gears[index]
        pitch, lam = self.p[INPUT_ID_TOOTH_PITCH], self.lam
        axisVector: adsk.core.Vector3D = _vector(gear['direction'])
        axisPoint: adsk.core.Point3D = _point(gear['origin'])
        rot: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        rot.setToRotation(k*pitch/lam, axisVector, axisPoint)
        mov: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k*pitch)
        mov.translation = shift
        rot.transformBy(mov)
        bodies: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        bodies.add(copyBody)
        moveInput: adsk.fusion.MoveFeatureInput = design.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        design.features.moveFeatures.add(moveInput)

    def _join_body(self, targetBody: adsk.fusion.BRepBody, toolBody: adsk.fusion.BRepBody, label) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designComponent
        tools: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput: adsk.fusion.CombineFeatureInput = design.features.combineFeatures.createInput(targetBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature: adsk.fusion.CombineFeature = design.features.combineFeatures.add(combineInput)
        return _one_body(combineFeature, label)

    def repeatCellByDoubling(self, index, cellBody: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        q = self.wholeCells
        m = 1
        body = cellBody
        asides = []
        top = 0
        power = 1
        while power*2 <= q:
            power *= 2
            top += 1
        for bit in range(top):
            if q & (1 << bit):
                asides.append((m, self._copy_body(body, f'{self.gears[index]["label"]} aside {m}')))
            copyBody = self._copy_body(body, f'{self.gears[index]["label"]} doubling {bit+1}')
            self._move_step(copyBody, index, m*self.cellTeeth)
            body = self._join_body(body, copyBody, f'{self.gears[index]["label"]} doubling {bit+1}')
            m *= 2
        for cells, aside in reversed(asides):
            self._move_step(aside, index, m*self.cellTeeth)
            body = self._join_body(body, aside, f'{self.gears[index]["label"]} aside join {cells}')
            m += cells
        if m != q:
            raise RuntimeError(f'{self.gears[index]["label"]} doubling made {m} cells, expected {q}')
        if self.remainder:
            gear = self.gears[index]
            start = (gear['phase']-int(self.p[INPUT_ID_TOOTH_COUNT])*self.p[INPUT_ID_TOOTH_PITCH]/2+
                     q*self.cellTeeth*self.p[INPUT_ID_TOOTH_PITCH])
            sections = self._cell_sections(index, self.remainder, start, f'{gear["label"]} Cell Remainder')
            remainderBody = self._loft_cell(sections, f'{gear["label"]} remainder')
            self._check_tooth_slant(gear, remainderBody, start)
            body = self._join_body(body, remainderBody, f'{gear["label"]} remainder join')
        return body

    def _sleeve_sketch(self) -> adsk.fusion.Profile:
        design: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = design.sketches.add(self.plane)
        sketch.name = 'Sleeve'
        centre: adsk.core.Point3D = _local(sketch, self.C)
        for radius in (self.ri, self.ro):
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+radius, centre.y, 0)
            diameter: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameter.parameter.value = 2*radius
        _fully_constrained(sketch)
        profiles = []
        for i in range(sketch.profiles.count):
            profile: adsk.fusion.Profile = sketch.profiles.item(i)
            if profile.profileLoops.count == 2:
                profiles.append(profile)
        if len(profiles) != 1:
            raise RuntimeError(f'Sleeve has {len(profiles)} annular profiles, expected one')
        return profiles[0]

    def _sleeve_extrude(self, ringProfile: adsk.fusion.Profile) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designComponent
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        riseValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.p[INPUT_ID_CAGE_RISE])
        extrudeInput.setSymmetricExtent(riseValue, False)
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        return _one_body(extrudeFeature, 'Sleeve')

    @staticmethod
    def _norm2(v):
        length = math.hypot(v[0], v[1])
        if length <= 1e-12:
            raise RuntimeError('Bore angular dimension has a zero-length ray')
        return (v[0]/length, v[1]/length)

    @staticmethod
    def _cross2(a, b):
        return a[0]*b[1]-a[1]*b[0]

    def _angle_text(self, O, Cp, E, P1, P2, useSpine, length):
        o = (O.x, O.y)
        cp = (Cp.x, Cp.y)
        e = (E.x, E.y)
        p1 = (P1.x, P1.y)
        p2 = (P2.x, P2.y)
        if useSpine:
            origin = o
            rayRu = self._norm2((cp[0]-o[0], cp[1]-o[1]))
            rayOther = self._norm2((e[0]-o[0], e[1]-o[1]))
        else:
            ru = (cp[0]-o[0], cp[1]-o[1])
            side = (p2[0]-p1[0], p2[1]-p1[1])
            det = self._cross2(ru, side)
            if abs(det) <= 1e-12:
                raise RuntimeError('Ru and L2 have no usable angular intersection')
            op = (p1[0]-o[0], p1[1]-o[1])
            along = self._cross2(op, side)/det
            origin = (o[0]+along*ru[0], o[1]+along*ru[1])
            target = cp if math.hypot(cp[0]-origin[0], cp[1]-origin[1]) > 1e-9 else o
            rayRu = self._norm2((target[0]-origin[0], target[1]-origin[1]))
            rayOther = self._norm2(side)
        dot = max(-1.0, min(1.0, rayRu[0]*rayOther[0]+rayRu[1]*rayOther[1]))
        angle = math.acos(dot)
        textPoint: adsk.core.Point3D = adsk.core.Point3D.create(
            origin[0]+(rayRu[0]+rayOther[0])*length/3,
            origin[1]+(rayRu[1]+rayOther[1])*length/3, 0)
        return angle, textPoint

    def _boreSketch(self, bore, plane: adsk.fusion.ConstructionPlane) -> adsk.fusion.Profile:
        design: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = bore['label']
        sketch.isComputeDeferred = True
        gear = self.gears[bore['gear']]
        station = bore['start']
        theta = self._angle(gear, station)
        axis = _add(gear['origin'], _scale(gear['direction'], station))
        CpWorld = _add(axis, _scale(gear['u'], self.offset/2))
        EWorld = self._world(gear, self.hw, 0, station)
        O: adsk.fusion.SketchPoint = _fixed(sketch, axis)
        Cp: adsk.fusion.SketchPoint = _fixed(sketch, CpWorld)
        E: adsk.fusion.SketchPoint = _fixed(sketch, EWorld)
        Ru: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True
        K: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, E)
        K.isConstruction = True
        corners = []
        expected = []
        for u, v in self._opening(bore):
            local: adsk.core.Point3D = _local(sketch, self._world(gear, u, v, station))
            expected.append((local.x, local.y))
            corner: adsk.fusion.SketchPoint = sketch.sketchPoints.add(local)
            corners.append(corner)
        P0, P1, P2, P3 = corners
        L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P0, P1)
        L2: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P1, P2)
        L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P2, P3)
        L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P3, P0)
        O.isFixed = True
        Cp.isFixed = True

        # Fusion's measured sequence: length and angle precede every rectangle constraint.
        Oloc, Cploc, Eloc = O.geometry, Cp.geometry, E.geometry
        lengthText: adsk.core.Point3D = adsk.core.Point3D.create(
            (Oloc.x+Eloc.x)/2, (Oloc.y+Eloc.y)/2+self.hw/5, 0)
        lengthDim: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDim.parameter.value = self.hw
        useSpine = abs(math.sin(theta)) >= math.sqrt(0.5)
        angleValue, angleText = self._angle_text(Oloc, Cploc, Eloc, P1.geometry, P2.geometry,
                                                 useSpine, self.hw)
        if useSpine:
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)
        else:
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)
        angleDim.parameter.value = angleValue

        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addParallel(L4, L2)
        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        def offset_text(a, b):
            return adsk.core.Point3D.create((a.x+b.x)/2, (a.y+b.y)/2, 0)
        lowerText: adsk.core.Point3D = offset_text(E.geometry, P1.geometry)
        upperText: adsk.core.Point3D = offset_text(E.geometry, P2.geometry)
        widthText: adsk.core.Point3D = offset_text(P0.geometry, P1.geometry)
        lower: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)
        lower.parameter.value = abs(bore['vLo'])
        upper: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)
        upper.parameter.value = abs(bore['vHi'])
        width: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)
        width.parameter.value = 2*self.hw
        sketch.isComputeDeferred = False
        _fully_constrained(sketch)
        largest, cornerIndex = 0.0, -1
        for index, corner in enumerate(corners):
            solved: adsk.core.Point3D = corner.geometry
            measured = math.hypot(solved.x-expected[index][0], solved.y-expected[index][1])
            if measured > largest:
                largest, cornerIndex = measured, index
        if largest > 0.0005:
            raise RuntimeError(f'{sketch.name} corner {cornerIndex} moved {largest*10:.4f} mm')
        if sketch.profiles.count != 1:
            raise RuntimeError(f'{sketch.name} has {sketch.profiles.count} profiles, expected one')
        return find_profile_by_curve_counts(sketch, lines=4)

    def _cut_bore(self, bore):
        design: adsk.fusion.Component = self.designComponent
        index, sign = bore['gear'], bore['sign']
        key = 'bore+' if sign > 0 else 'bore-'
        boreLine: adsk.fusion.SketchLine = self.pathLines[index][key]
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        startFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0)
        planeInput.setByDistanceOnPath(boreLine, startFraction)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = bore['label']+' Plane'
        profile: adsk.fusion.Profile = self._boreSketch(bore, plane)
        borePath: adsk.fusion.Path = design.features.createPath(boreLine, False)
        sweepInput: adsk.fusion.SweepFeatureInput = design.features.sweepFeatures.createInput(
            profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal((self.sOut-self.sIn)/self.lam)
        sweepInput.participantBodies = [self.cageBody]
        sweepFeature: adsk.fusion.SweepFeature = design.features.sweepFeatures.add(sweepInput)
        self.cageBody = _one_body(sweepFeature, bore['label']+' cut')
        gear = self.gears[index]
        station = sign*self.p[INPUT_ID_CAGE_RADIUS]
        probeU = self.p[INPUT_ID_RIBBON_WIDTH]/2+self.p[INPUT_ID_CLEARANCE]/2
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        for side in (-1, 1):
            probePoint: adsk.core.Point3D = _point(self._world(gear, side*probeU, 0, station))
            result = self.cageBody.pointContainment(probePoint)
            if result != outside:
                raise RuntimeError(f'{bore["label"]} twist probe side {side} read {result}; expected outside')
        if sign == self.levelSigns[index] and self.p[INPUT_ID_ROOF_ALLOWANCE] > 0:
            v = bore['roofSign']*(self.ht+self.p[INPUT_ID_ROOF_ALLOWANCE]/2)
            roofPoint: adsk.core.Point3D = _point(self._world(gear, 0, v, station))
            floorPoint: adsk.core.Point3D = _point(self._world(gear, 0, -v, station))
            roof = self.cageBody.pointContainment(roofPoint)
            floor = self.cageBody.pointContainment(floorPoint)
            if roof != outside:
                raise RuntimeError(f'{bore["label"]} roof probe read {roof}; expected outside')
            if floor != adsk.fusion.PointContainment.PointInsidePointContainment:
                raise RuntimeError(f'{bore["label"]} floor probe read {floor}; expected inside')

    def _search_to_world(self, point):
        return _add(self.C, _add(_scale(self.eHat, point[0]),
                                 _add(_scale(self.kHat, point[1]), _scale(self.nHat, point[2]))))

    def _window_sketch(self, window, plane: adsk.fusion.ConstructionPlane) -> adsk.fusion.Profile:
        design: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = f'Window {window["name"]}'
        points = []
        for t, z in window['corners']:
            world = self._search_to_world(_add(_scale(window['across'], t), _scale(self.searchN, z)))
            local: adsk.core.Point3D = _local(sketch, world)
            point: adsk.fusion.SketchPoint = sketch.sketchPoints.add(local)
            points.append(point)
        for i, startPoint in enumerate(points):
            endPoint = points[(i+1) % len(points)]
            sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
        for point in points:
            point.isFixed = True
        _fully_constrained(sketch)
        if sketch.profiles.count != 1:
            raise RuntimeError(f'{sketch.name} has {sketch.profiles.count} profiles, expected one')
        return sketch.profiles.item(0)

    def _cut_window(self, window, plane: adsk.fusion.ConstructionPlane):
        design: adsk.fusion.Component = self.designComponent
        profile: adsk.fusion.Profile = self._window_sketch(window, plane)
        sketch: adsk.fusion.Sketch = profile.parentSketch
        tc = sum(t for t, z in window['corners'])/len(window['corners'])
        zc = sum(z for t, z in window['corners'])/len(window['corners'])
        a0 = math.sqrt(max(0, self.ri*self.ri-tc*tc))
        a1 = math.sqrt(max(0, self.ro*self.ro-tc*tc))
        probe = _add(_add(_scale(window['across'], tc), _scale(self.searchN, zc)),
                     _scale(window['facing'], (a0+a1)/2))
        probePoint: adsk.core.Point3D = _point(self._search_to_world(probe))
        inside = adsk.fusion.PointContainment.PointInsidePointContainment
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        before = self.cageBody.pointContainment(probePoint)
        if before != inside:
            raise RuntimeError(f'Window {window["name"]} probe before cut read {before}; expected inside')
        directionPoint: adsk.core.Point3D = _point(self._search_to_world(window['facing']))
        localDirection: adsk.core.Point3D = sketch.modelToSketchSpace(directionPoint)
        centreLocal: adsk.core.Point3D = sketch.modelToSketchSpace(_point(self.C))
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if localDirection.z > centreLocal.z else
                     adsk.fusion.ExtentDirections.NegativeExtentDirection)
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.ro+to_cm(1))
        extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeInput.participantBodies = [self.cageBody]
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        self.cageBody = _one_body(extrudeFeature, f'Window {window["name"]} cut')
        after = self.cageBody.pointContainment(probePoint)
        if after != outside:
            raise RuntimeError(f'Window {window["name"]} probe after cut read {after}; expected outside')

    def _marker_plane(self) -> adsk.fusion.ConstructionPlane:
        design: adsk.fusion.Component = self.designComponent
        p = self.p
        inset = min(to_cm(0.1), p[INPUT_ID_COLLAR_WALL]/2)
        self.markerInset = inset
        target = p[INPUT_ID_CAGE_RISE]-inset
        gearBAxisPlane: adsk.fusion.ConstructionPlane = self.axisPlanes[1]
        normal: adsk.core.Vector3D = get_normal(gearBAxisPlane)
        signedOffset = (target-self.offset/2) * (1 if _dot(_unit((normal.x, normal.y, normal.z)), self.nHat) > 0 else -1)
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        value: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(signedOffset)
        planeInput.setByOffset(gearBAxisPlane, value)
        markerPlane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        markerPlane.name = 'Bore Marker Plane'
        measured = _dot(_sub(_xyz(markerPlane.geometry.origin), self.C), self.nHat)
        if abs(measured-target) > 1e-4:
            raise RuntimeError(f'Bore Marker Plane is {measured*10:.4f} mm above centre; expected {target*10:.4f} mm')
        return markerPlane

    def _marker_sketch(self, bore, plane: adsk.fusion.ConstructionPlane):
        design: adsk.fusion.Component = self.designComponent
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        shape = 'Circle' if bore['sign'] > 0 else 'Square'
        sketch.name = f'{bore["label"]} {shape} Marker'
        halfSize = min(to_cm(1), self.p[INPUT_ID_COLLAR_HALF]/2)
        centreWorld = _add(_add(self.C, _scale(self.nHat, self.p[INPUT_ID_CAGE_RISE]-self.markerInset)),
                           _scale(self.gears[bore['gear']]['direction'],
                                  bore['sign']*self.p[INPUT_ID_CAGE_RADIUS]))
        centre: adsk.core.Point3D = _local(sketch, centreWorld)
        if bore['sign'] > 0:
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, halfSize)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+halfSize, centre.y, 0)
            diameter: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameter.parameter.value = 2*halfSize
        else:
            pts = []
            for x, y in ((-halfSize, -halfSize), (halfSize, -halfSize),
                         (halfSize, halfSize), (-halfSize, halfSize)):
                world = _add(centreWorld, _add(_scale(self.eHat, x), _scale(self.kHat, y)))
                local: adsk.core.Point3D = _local(sketch, world)
                point: adsk.fusion.SketchPoint = sketch.sketchPoints.add(local)
                pts.append(point)
            for i, startPoint in enumerate(pts):
                endPoint = pts[(i+1) % 4]
                sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
            for point in pts:
                point.isFixed = True
        _fully_constrained(sketch)
        if sketch.profiles.count != 1:
            raise RuntimeError(f'{sketch.name} has {sketch.profiles.count} profiles, expected one')
        profile: adsk.fusion.Profile = sketch.profiles.item(0)
        return profile, sketch, centreWorld

    def _extrude_marker(self, profile: adsk.fusion.Profile, sketch: adsk.fusion.Sketch, centreWorld):
        design: adsk.fusion.Component = self.designComponent
        markCentrePlusNormal: adsk.core.Point3D = _point(_add(centreWorld, self.nHat))
        localPlus: adsk.core.Point3D = sketch.modelToSketchSpace(markCentrePlusNormal)
        localCentre: adsk.core.Point3D = sketch.modelToSketchSpace(_point(centreWorld))
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if localPlus.z > localCentre.z else
                     adsk.fusion.ExtentDirections.NegativeExtentDirection)
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.markerInset+to_cm(0.4))
        extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        return _one_body(extrudeFeature, 'Bore marker extrude')

    def buildCage(self):
        ringProfile = self._sleeve_sketch()
        self.cageBody = self._sleeve_extrude(ringProfile)
        for bore in self.bores:
            self._cut_bore(bore)
        if self.windows:
            design: adsk.fusion.Component = self.designComponent
            planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
            if self.p[INPUT_ID_CROSS_ANGLE] <= math.pi/2:
                rightAngle: adsk.core.ValueInput = adsk.core.ValueInput.createByString('90 deg')
                planeInput.setByAngle(self.anchorLine, rightAngle, self.plane)
            else:
                midpointFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0.5)
                planeInput.setByDistanceOnPath(self.anchorLine, midpointFraction)
            windowPlane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
            windowPlane.name = 'Window Plane'
            for window in self.windows:
                self._cut_window(window, windowPlane)
        markerPlane = self._marker_plane()
        for bore in self.bores:
            profile, sketch, centreWorld = self._marker_sketch(bore, markerPlane)
            markBody: adsk.fusion.BRepBody = self._extrude_marker(profile, sketch, centreWorld)
            self.cageBody = self._join_body(self.cageBody, markBody, str(bore['label'])+' marker join')
        futil.log('Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.')

    def relocateBodies(self):
        for index, gearBody in enumerate(self.gearBodies):
            body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(gearBody)
            body.name = self.gears[index]['label']
            targetOccurrence: adsk.fusion.Occurrence = self.gearOccs[index]
            body.moveToComponent(targetOccurrence)
        body: adsk.fusion.BRepBody = self.cageBody
        body.name = 'Cage'
        targetOccurrence: adsk.fusion.Occurrence = self.cageOcc
        body.moveToComponent(targetOccurrence)
