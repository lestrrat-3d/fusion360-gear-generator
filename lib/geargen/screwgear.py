import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from . import solids
from .base import Generator
from .misc import get_design
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


def _mul(a, k):
    return tuple(k * a[i] for i in range(3))


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _length(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    length = _length(a)
    if length <= 1e-12:
        raise ValueError('Cannot use a zero-length direction')
    return _mul(a, 1 / length)


def _rotate(v, axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    return _add(_add(_mul(v, c), _mul(_cross(axis, v), s)),
                _mul(axis, _dot(axis, v) * (1 - c)))


def _coords(point):
    return (point.x, point.y, point.z)


def _point(xyz):
    return adsk.core.Point3D.create(xyz[0], xyz[1], xyz[2])


def _vector(xyz):
    return adsk.core.Vector3D.create(xyz[0], xyz[1], xyz[2])


def _local(sketch: adsk.fusion.Sketch, world, flatten=True) -> adsk.core.Point3D:
    worldPoint: adsk.core.Point3D = _point(world)
    local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
    if flatten:
        local.z = 0
    return local


def _require_sketch(sketch: adsk.fusion.Sketch):
    if not sketch.isFullyConstrained:
        raise RuntimeError(f'{sketch.name} is not fully constrained')


def _one_body(feature, label: str) -> adsk.fusion.BRepBody:
    bodies: adsk.fusion.BRepBodies = feature.bodies
    if bodies.count != 1:
        raise RuntimeError(f'{label} produced {bodies.count} bodies, expected one')
    body: adsk.fusion.BRepBody = bodies.item(0)
    if not body.isSolid:
        raise RuntimeError(f'{label} produced a non-solid body')
    return body


def _hull(points):
    pts = sorted(set(points))
    if len(pts) < 2:
        return pts
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                a, b = out[-2], out[-1]
                if (b[0]-a[0])*(p[1]-b[1])-(b[1]-a[1])*(p[0]-b[0]) > 0:
                    break
                out.pop()
            out.append(p)
        return out
    return half(pts)[:-1] + half(reversed(pts))[:-1]


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


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs: adsk.core.CommandInputs = command.commandInputs
        for ident, label, tooltip, filters in (
            (INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to", ('ConstructionPlanes', 'PlanarFaces')),
            (INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism', ('ConstructionPoints', 'SketchPoints')),
            (INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under',
             ('Occurrences', 'RootComponents')),
        ):
            selectionInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(ident, label, tooltip)
            for filterConstant in filters:
                selectionInput.addSelectionFilter(filterConstant)
            selectionInput.setSelectionLimits(1, 1)
            if ident == INPUT_ID_PARENT:
                parentInput: adsk.core.SelectionCommandInput = selectionInput
                rootComponent: adsk.fusion.Component = get_design().rootComponent
                parentInput.addSelection(rootComponent)
        groups = (
            ('ribbonGroup', 'Ribbon', True, (
                (INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', 15/10),
                (INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', 68),
                (INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', 49.5/10),
                (INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm', 3.75/10),
                (INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', 2.625/10),
                (INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', 2.625/10),
                (INPUT_ID_TOOTH_SLANT, 'Tooth Slant', 'deg', math.radians(25.8)),
                (INPUT_ID_TOOTH_BOW, 'Tooth Bow', '', 0.048),
            )),
            ('frameGroup', 'Frame', True, (
                (INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', 15/10),
                (INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', 18.75/10),
                (INPUT_ID_CLEARANCE, 'Clearance', 'mm', 0.20/10),
                (INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', 0.60/10),
                (INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', 3/10),
                (INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', 3/10),
            )),
            ('meshGroup', 'Mesh (from the mesh search)', False, (
                (INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg', math.radians(80)),
                (INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', 1.05/10),
                (INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', 0),
                (INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', 0),
                (INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', -1.31/10),
            )),
        )
        for groupId, groupLabel, expanded, rows in groups:
            group: adsk.core.GroupCommandInput = inputs.addGroupCommandInput(groupId, groupLabel)
            group.isExpanded = expanded
            for ident, label, unit, value in rows:
                initialValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(value)
                group.children.addValueInput(ident, label, unit, initialValue)


class ScrewGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'ScrewGear'

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        self._precomputeSearch()
        self.buildGear(0)
        self.buildGear(1)
        self.buildCage()
        self.relocateBodies()
        solids.hide_construction_geometry(self.designOcc.component)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        def select(inputs: adsk.core.CommandInputs, ident):
            selectionInput: adsk.core.SelectionCommandInput = adsk.core.SelectionCommandInput.cast(inputs.itemById(ident))
            if selectionInput is None or selectionInput.selectionCount != 1:
                raise ValueError(f'{ident} requires one selection')
            return selectionInput.selection(0).entity
        self.plane = select(inputs, INPUT_ID_PLANE)
        self.selectedPoint = select(inputs, INPUT_ID_POINT)
        parent = select(inputs, INPUT_ID_PARENT)
        if isinstance(parent, adsk.fusion.Occurrence):
            self.parentComponent = parent.component
        else:
            self.parentComponent = adsk.fusion.Component.cast(parent)
        if self.parentComponent is None:
            raise ValueError('parent must select a component')
        values = {}
        units = {
            INPUT_ID_TOOTH_COUNT: '', INPUT_ID_TOOTH_BOW: '',
            INPUT_ID_TOOTH_SLANT: 'deg', INPUT_ID_CROSS_ANGLE: 'deg',
            INPUT_ID_MOUNT_ANGLE_A: 'deg', INPUT_ID_MOUNT_ANGLE_B: 'deg',
        }
        unitsManager: adsk.core.UnitsManager = self.design.unitsManager
        for ident in (
            INPUT_ID_RIBBON_WIDTH, INPUT_ID_TOOTH_COUNT, INPUT_ID_TWIST_LEAD,
            INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH, INPUT_ID_TOOTH_HEIGHT,
            INPUT_ID_TOOTH_SLANT, INPUT_ID_TOOTH_BOW, INPUT_ID_CAGE_RADIUS,
            INPUT_ID_CAGE_RISE, INPUT_ID_CLEARANCE, INPUT_ID_ROOF_ALLOWANCE,
            INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CROSS_ANGLE,
            INPUT_ID_ENGAGEMENT, INPUT_ID_MOUNT_ANGLE_A, INPUT_ID_MOUNT_ANGLE_B,
            INPUT_ID_ASSEMBLY_PHASE,
        ):
            input: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(inputs.itemById(ident))
            if input is None:
                raise ValueError(f'Missing input {ident}')
            unit = units[ident] if ident in units else 'mm'
            values[ident] = unitsManager.evaluateExpression(input.expression, unit)
        self.v = values
        for ident in (INPUT_ID_RIBBON_WIDTH, INPUT_ID_RIBBON_THICKNESS, INPUT_ID_TOOTH_PITCH,
                      INPUT_ID_TWIST_LEAD, INPUT_ID_COLLAR_HALF, INPUT_ID_COLLAR_WALL, INPUT_ID_CLEARANCE):
            if values[ident] <= 0:
                raise ValueError(f'{ident} must be > 0')
        if values[INPUT_ID_ROOF_ALLOWANCE] < 0:
            raise ValueError('roofAllowance must be >= 0')
        n = values[INPUT_ID_TOOTH_COUNT]
        if n < 4 or n != int(n):
            raise ValueError('toothCount must be a whole number >= 4')
        self.toothCount = int(n)
        w, h, t = values[INPUT_ID_RIBBON_WIDTH], values[INPUT_ID_TOOTH_HEIGHT], values[INPUT_ID_RIBBON_THICKNESS]
        if h <= 0 or h >= w/2:
            raise ValueError('toothHeight must be > 0 and < ribbonWidth/2')
        slant = values[INPUT_ID_TOOTH_SLANT]
        if abs(slant) >= math.pi/2:
            raise ValueError('toothSlant must be between -90 and 90 degrees')
        bow = values[INPUT_ID_TOOTH_BOW]
        if bow < 0 or h + (bow/10)*(t*10/2)**2 >= w/2:
            raise ValueError('toothBow must be >= 0 and keep the face root below ribbonWidth/2')
        engagement = values[INPUT_ID_ENGAGEMENT]
        if engagement <= 0 or engagement > h:
            raise ValueError('engagement must be > 0 and <= toothHeight')
        sigma = values[INPUT_ID_CROSS_ANGLE]
        if sigma <= 0 or sigma >= math.pi:
            raise ValueError('crossAngle must be between 0 and 180 degrees')
        pitch = values[INPUT_ID_TOOTH_PITCH]
        if abs(values[INPUT_ID_ASSEMBLY_PHASE]) >= pitch:
            raise ValueError('assemblyPhase must be strictly within +/-toothPitch')
        radius, half = values[INPUT_ID_CAGE_RADIUS], values[INPUT_ID_COLLAR_HALF]
        if radius + half + 0.1 >= self.toothCount*pitch/2:
            raise ValueError('cageRadius + collarHalf + 1 mm must fit within half the ribbon')
        self.lam = values[INPUT_ID_TWIST_LEAD]/(2*math.pi)
        self.axisOffset = w-engagement
        self.sigma = sigma
        self.hw = w/2 + values[INPUT_ID_CLEARANCE]
        self.ht = t/2 + values[INPUT_ID_CLEARANCE]
        self.ri = radius-half
        self.ro = radius+half
        self.boreCorner = math.hypot(self.hw, self.ht+values[INPUT_ID_ROOF_ALLOWANCE])
        if math.hypot(self.boreCorner, 0.1) >= self.ri:
            raise ValueError('cageRadius is too small for bore corners and 1 mm channel start')
        self.sIn = math.sqrt(self.ri*self.ri-self.boreCorner*self.boreCorner)-0.1
        self.sOut = self.ro+0.1
        axialWindow = 1.5*math.sqrt(w*w-self.axisOffset*self.axisOffset)/math.sin(sigma)
        if math.hypot(axialWindow, math.hypot(w/2, t/2))+values[INPUT_ID_CLEARANCE] > self.ri:
            raise ValueError('cageRadius hides the mesh along the axis')
        if values[INPUT_ID_CAGE_RISE] < self.axisOffset/2+self.boreCorner+values[INPUT_ID_COLLAR_WALL]:
            raise ValueError('cageRise must leave collarWall around the channels')
        self.cellTeeth = min(CELL_TEETH, self.toothCount)
        self.cells, self.remainder = divmod(self.toothCount, self.cellTeeth)
        self.cellSteps = max(8, math.ceil((pitch/self.lam)/math.radians(2)))

    def buildComponentTree(self):
        top: adsk.fusion.Occurrence = self.getOccurrence()
        topComponent: adsk.fusion.Component = top.component
        topComponent.name = 'Screw Gearing'
        identity: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        self.designOcc: adsk.fusion.Occurrence = topComponent.occurrences.addNewComponent(identity)
        self.designOcc.component.name = 'Design'
        self.gearOccs = []
        for name in ('Gear A', 'Gear B'):
            identity = adsk.core.Matrix3D.create()
            occurrence: adsk.fusion.Occurrence = topComponent.occurrences.addNewComponent(identity)
            occurrence.component.name = name
            self.gearOccs.append(occurrence)
        identity = adsk.core.Matrix3D.create()
        self.cageOcc: adsk.fusion.Occurrence = topComponent.occurrences.addNewComponent(identity)
        self.cageOcc.component.name = 'Cage'
        self.gearBodies = [adsk.fusion.BRepBody.cast(None)] * 2
        self.cageBody = adsk.fusion.BRepBody.cast(None)
        self.pathLines = [{}, {}]

    def _offsetPlane(self, source, distance: float, name: str) -> adsk.fusion.ConstructionPlane:
        design: adsk.fusion.Component = self.designOcc.component
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        offsetValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(distance)
        planeInput.setByOffset(source, offsetValue)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = name
        return plane

    def buildAnchor(self):
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(self.plane)
        sketch.name = 'Anchor'
        projected: adsk.core.ObjectCollection = sketch.project(self.selectedPoint)
        if projected.count != 1:
            raise RuntimeError(f'Anchor projected {projected.count} entities, expected one')
        projectedPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projected.item(0))
        centre = _coords(projectedPoint.geometry)
        startSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre[0]-0.5, centre[1], 0)
        endSeed: adsk.core.Point3D = adsk.core.Point3D.create(centre[0]+0.5, centre[1], 0)
        anchorLine: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)
        sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)
        sketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre[0], centre[1]+0.3, 0)
        dim = sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dim.parameter.value = 1.0
        _require_sketch(sketch)
        self.anchorLine = anchorLine
        self.C = _coords(projectedPoint.worldGeometry)
        self.eHat = _unit(_sub(_coords(anchorLine.endSketchPoint.worldGeometry),
                               _coords(anchorLine.startSketchPoint.worldGeometry)))
        normal: adsk.core.Vector3D = get_normal(self.plane)
        rawN = _unit(_coords(normal))
        self.gearAxisPlanes = [
            self._offsetPlane(self.plane, -self.axisOffset/2, 'Gear A Axis Plane'),
            self._offsetPlane(self.plane, self.axisOffset/2, 'Gear B Axis Plane'),
        ]
        aPlane: adsk.fusion.ConstructionPlane = self.gearAxisPlanes[0]
        aNormal = _unit(_coords(aPlane.geometry.normal))
        aOrigin = _coords(aPlane.geometry.origin)
        self.nHat = aNormal if _dot(_sub(self.C, aOrigin), aNormal) >= 0 else _mul(aNormal, -1)
        for plane in self.gearAxisPlanes:
            origin = _coords(plane.geometry.origin)
            measured = abs(_dot(_sub(self.C, origin), self.nHat))
            if abs(measured-self.axisOffset/2) > 1e-4:
                raise RuntimeError(f'{plane.name} is {measured*10:.4f} mm from centre, expected {self.axisOffset*5:.4f}')
        self.kHat = _unit(_cross(self.nHat, self.eHat))
        self.gears = []
        for index in (0, 1):
            direction = _unit(_rotate(self.eHat, self.nHat, (1 if index == 0 else -1)*self.sigma/2))
            u = self.nHat if index == 0 else _mul(self.nHat, -1)
            v = _unit(_cross(direction, u))
            origin = _add(self.C, _mul(self.nHat, (-1 if index == 0 else 1)*self.axisOffset/2))
            self.gears.append({'origin': origin, 'dir': direction, 'u': u, 'v': v,
                               'phi': self.v[INPUT_ID_MOUNT_ANGLE_A if index == 0 else INPUT_ID_MOUNT_ANGLE_B],
                               'phase': 0 if index == 0 else self.v[INPUT_ID_ASSEMBLY_PHASE],
                               'name': 'Gear A' if index == 0 else 'Gear B'})

    def _angle(self, g, s):
        return s/self.lam + g['phi']

    def _world(self, g, u, v, s):
        angle = self._angle(g, s)
        x = u*math.cos(angle)-v*math.sin(angle)
        y = u*math.sin(angle)+v*math.cos(angle)
        return _add(_add(_add(g['origin'], _mul(g['dir'], s)), _mul(g['u'], x)), _mul(g['v'], y))

    def _levelBore(self, g):
        def tilt(sign):
            lo = self._angle(g, sign*(self.v[INPUT_ID_CAGE_RADIUS]-self.v[INPUT_ID_COLLAR_HALF]))
            hi = self._angle(g, sign*(self.v[INPUT_ID_CAGE_RADIUS]+self.v[INPUT_ID_COLLAR_HALF]))
            lo, hi = min(lo, hi), max(lo, hi)
            level = math.pi/2 + math.ceil((lo-math.pi/2)/math.pi)*math.pi
            return 0 if level <= hi else min(abs(math.cos(lo)), abs(math.cos(hi)))
        return 1 if tilt(1) < tilt(-1) else -1

    def _roofSide(self, g, sign):
        angle = self._angle(g, sign*self.v[INPUT_ID_CAGE_RADIUS])
        up = -math.sin(angle)*_dot(g['u'], self.nHat) + math.cos(angle)*_dot(g['v'], self.nHat)
        return 1 if up > 0 else -1

    def _opening(self, g, sign):
        lo, hi = -self.ht, self.ht
        if sign == self._levelBore(g):
            if self._roofSide(g, sign) > 0:
                hi += self.v[INPUT_ID_ROOF_ALLOWANCE]
            else:
                lo -= self.v[INPUT_ID_ROOF_ALLOWANCE]
        return self.hw, lo, hi

    def _boreCorners(self, g, sign):
        hw, lo, hi = self._opening(g, sign)
        return [(-hw, lo), (hw, lo), (hw, hi), (-hw, hi)]

    def buildSweepPaths(self, index: int):
        g = self.gears[index]
        design: adsk.fusion.Component = self.designOcc.component
        plane: adsk.fusion.ConstructionPlane = self.gearAxisPlanes[index]
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = g['name'] + ' Paths'
        positions = [-self.sOut, -self.sIn, self.sIn, self.sOut]
        points = []
        for s in positions:
            world = _add(g['origin'], _mul(g['dir'], s))
            worldPoint: adsk.core.Point3D = _point(world)
            local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            local.z = 0
            point: adsk.fusion.SketchPoint = sketch.sketchPoints.add(local)
            points.append(point)
        negative: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[0], points[1])
        positive: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(points[2], points[3])
        for point in points:
            point.isFixed = True
        _require_sketch(sketch)
        self.pathLines[index] = {'bore-': negative, 'bore+': positive}

    def _tooth(self, g, s, v):
        w = self.v[INPUT_ID_RIBBON_WIDTH]
        h = self.v[INPUT_ID_TOOTH_HEIGHT]
        pitch = self.v[INPUT_ID_TOOTH_PITCH]
        slant = math.tan(self.v[INPUT_ID_TOOTH_SLANT])
        bow = self.v[INPUT_ID_TOOTH_BOW]*10
        return w/2-h/2+h/2*math.cos(2*math.pi*(s+slant*v-g['phase'])/pitch)-bow*v*v

    def _cellSections(self, index: int, teeth: int, start: float, name: str):
        g = self.gears[index]
        design: adsk.fusion.Component = self.designOcc.component
        plane: adsk.fusion.ConstructionPlane = self.gearAxisPlanes[index]
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = name
        sketch.isComputeDeferred = True
        thickness = self.v[INPUT_ID_RIBBON_THICKNESS]
        width = self.v[INPUT_ID_RIBBON_WIDTH]
        pitch = self.v[INPUT_ID_TOOTH_PITCH]
        sections = []
        allPoints = []
        splines = []
        for k in range(teeth*self.cellSteps+1):
            station = start+k*pitch/self.cellSteps
            uv = [(-width/2, -thickness/2), (-width/2, thickness/2)]
            uv += [(self._tooth(g, station, -thickness/2+j*thickness/(TOOTH_SPLINE_POINTS-1)),
                    -thickness/2+j*thickness/(TOOTH_SPLINE_POINTS-1))
                   for j in range(TOOTH_SPLINE_POINTS)]
            points = []
            for u, v in uv:
                world = self._world(g, u, v, station)
                worldPoint: adsk.core.Point3D = _point(world)
                local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
                point: adsk.fusion.SketchPoint = sketch.sketchPoints.add(local)
                points.append(point)
                allPoints.append(point)
            B0, B1 = points[:2]
            F = points[2:]
            L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F[0])
            fitPoints: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for sectionPoint in F:
                fitPoints.add(sectionPoint)
            spline: adsk.fusion.SketchFittedSpline = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
            if spline is None or spline.fitPoints.count != TOOTH_SPLINE_POINTS:
                raise RuntimeError(f'{name} section {k} failed to fit {TOOTH_SPLINE_POINTS} tooth points')
            L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(F[-1], B1)
            L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)
            sections.append((L1, spline, L3, L4))
            splines.append(spline)
        for point in allPoints:
            point.isFixed = True
        for spline in splines:
            for i in range(spline.fitPoints.count):
                spline.fitPoints.item(i).isFixed = True
        sketch.isComputeDeferred = False
        _require_sketch(sketch)
        if sketch.profiles.count != len(sections):
            raise RuntimeError(f'{name} has {sketch.profiles.count} profiles, expected {len(sections)}')
        return sections

    def _loftCell(self, sections, label: str) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designOcc.component
        loftInput: adsk.fusion.LoftFeatureInput = design.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for section in sections:
            if len(section) != 4:
                raise RuntimeError(f'{label} has a section with {len(section)} curves')
            curves: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            for sectionCurve in section:
                curves.add(sectionCurve)
            sectionPath: adsk.fusion.Path = design.features.createPath(curves, False)
            loftInput.loftSections.add(sectionPath)
        loftFeature: adsk.fusion.LoftFeature = design.features.loftFeatures.add(loftInput)
        return _one_body(loftFeature, label)

    def _checkToothSlant(self, body: adsk.fusion.BRepBody, g, start: float):
        pitch = self.v[INPUT_ID_TOOTH_PITCH]
        slant = math.tan(self.v[INPUT_ID_TOOTH_SLANT])
        sc = g['phase']+pitch*math.ceil((start+pitch/2-g['phase'])/pitch)
        used = 0
        for sign in (-1, 1):
            vp = sign*(self.v[INPUT_ID_RIBBON_THICKNESS]/2-0.025)
            up = self.v[INPUT_ID_RIBBON_WIDTH]/2-self.v[INPUT_ID_TOOTH_BOW]*10*vp*vp-0.025
            for tag, station in (('on-ridge', sc-slant*vp), ('off-ridge', sc+slant*vp)):
                m = self._tooth(g, station, vp)-up
                shadow = dict(g)
                old = self.v[INPUT_ID_TOOTH_SLANT]
                self.v[INPUT_ID_TOOTH_SLANT] = -old
                opposite = self._tooth(shadow, station, vp)-up
                self.v[INPUT_ID_TOOTH_SLANT] = old
                if abs(m) < 0.01 or abs(opposite) < 0.01 or m*opposite >= 0:
                    continue
                used += 1
                probePoint: adsk.core.Point3D = _point(self._world(g, up, vp, station))
                found = body.pointContainment(probePoint)
                expected = (adsk.fusion.PointContainment.PointInsidePointContainment if m > 0
                            else adsk.fusion.PointContainment.PointOutsidePointContainment)
                if found != expected:
                    raise RuntimeError(f"{g['name']} {tag} slant probe read {found}, expected {expected}")
        if used == 0:
            futil.log(f"{g['name']} tooth slant sign was not checked near zero")

    def _copyBody(self, sourceBody: adsk.fusion.BRepBody) -> adsk.fusion.BRepBody:
        design: adsk.fusion.Component = self.designOcc.component
        copyFeature = design.features.copyPasteBodies.add(sourceBody)
        return _one_body(copyFeature, 'Cell copy')

    def _moveScrew(self, copyBody: adsk.fusion.BRepBody, g, k: int):
        design: adsk.fusion.Component = self.designOcc.component
        pitch = self.v[INPUT_ID_TOOTH_PITCH]
        lam = self.lam
        axisVector: adsk.core.Vector3D = _vector(g['dir'])
        axisPoint: adsk.core.Point3D = _point(g['origin'])
        rot: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        rot.setToRotation(k*pitch/lam, axisVector, axisPoint)
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k*pitch)
        mov: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)
        bodies: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        bodies.add(copyBody)
        moveInput: adsk.fusion.MoveFeatureInput = design.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        design.features.moveFeatures.add(moveInput)

    def _join(self, targetBody: adsk.fusion.BRepBody, toolBody: adsk.fusion.BRepBody, label: str):
        design: adsk.fusion.Component = self.designOcc.component
        tools: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput: adsk.fusion.CombineFeatureInput = design.features.combineFeatures.createInput(targetBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature: adsk.fusion.CombineFeature = design.features.combineFeatures.add(combineInput)
        return _one_body(combineFeature, label)

    def repeatCellByDoubling(self, index: int, body: adsk.fusion.BRepBody, start: float):
        g = self.gears[index]
        q = self.cells
        c = self.cellTeeth
        aside = []
        current = 1
        while current*2 <= q:
            if q & current:
                aside.append((current, self._copyBody(body)))
            copyBody = self._copyBody(body)
            self._moveScrew(copyBody, g, current*c)
            body = self._join(body, copyBody, g['name']+' doubling join')
            current *= 2
        for count, asideBody in reversed(aside):
            self._moveScrew(asideBody, g, current*c)
            body = self._join(body, asideBody, g['name']+' aside join')
            current += count
        if current != q:
            raise RuntimeError(f"{g['name']} made {current} cells, expected {q}")
        if self.remainder:
            remainderStart = start+q*c*self.v[INPUT_ID_TOOTH_PITCH]
            sections = self._cellSections(index, self.remainder, remainderStart, g['name']+' Cell Remainder')
            remainderBody = self._loftCell(sections, g['name']+' remainder loft')
            body = self._join(body, remainderBody, g['name']+' remainder join')
        body.name = g['name']
        return body

    def buildGear(self, index: int):
        self.buildSweepPaths(index)
        g = self.gears[index]
        start = g['phase']-self.toothCount*self.v[INPUT_ID_TOOTH_PITCH]/2
        sections = self._cellSections(index, self.cellTeeth, start, g['name']+' Cell Sections')
        cellBody = self._loftCell(sections, g['name']+' cell loft')
        self._checkToothSlant(cellBody, g, start)
        self.gearBodies[index] = self.repeatCellByDoubling(index, cellBody, start)

    def _bores(self):
        return [{'g': self.gears[i], 'index': 2*i+j, 'sign': sign,
                 'name': f"{self.gears[i]['name']} Bore {'-R' if sign < 0 else '+R'}"}
                for i in (0, 1) for j, sign in enumerate((-1, 1))]

    def _span(self, bore):
        return (self.sIn, self.sOut) if bore['sign'] > 0 else (-self.sOut, -self.sIn)

    def _crossing(self, bore):
        return _add(bore['g']['origin'],
                    _mul(bore['g']['dir'], bore['sign']*self.v[INPUT_ID_CAGE_RADIUS]))

    def _channelOutline(self, bore):
        g, sign = bore['g'], bore['sign']
        hw, lo, hi = self._opening(g, sign)
        points = []
        count = int(math.floor((self.sOut-self.sIn)/0.01))+1
        for k in range(count):
            s = sign*(self.sIn+k*0.01)
            for j in range(17):
                f = j/16
                v = lo+(hi-lo)*f
                for u, vv in ((hw, v), (-hw, v), (-hw+2*hw*f, hi), (-hw+2*hw*f, lo)):
                    pt = self._world(g, u, vv, s)
                    fromCentre = _sub(pt, self.C)
                    radius = math.hypot(_dot(fromCentre, self.eHat), _dot(fromCentre, self.kHat))
                    if self.ri-0.05 <= radius <= self.ro+0.05:
                        points.append(pt)
        return points

    def _channelSeparation(self):
        outlines = [self._channelOutline(b) for b in self.bores]
        gaps = (
            (1, 2, self.kHat, '+k'),
            (0, 2, _mul(self.eHat, -1), '-e'),
            (0, 3, _mul(self.kHat, -1), '-k'),
            (1, 3, self.eHat, '+e'),
        )
        least = (math.inf, '')
        for i, j, direction, name in gaps:
            across = _cross(self.nHat, direction)
            def projection(pt):
                delta = _sub(pt, self.C)
                return (_dot(delta, across), _dot(delta, self.nHat))
            a = _hull(projection(pt) for pt in outlines[i])
            b = _hull(projection(pt) for pt in outlines[j])
            if len(a) < 2 or len(b) < 2:
                raise RuntimeError(f'{name} channel outlines have fewer than two hull points')
            best = -math.inf
            for poly in (a, b):
                for k, p in enumerate(poly):
                    q = poly[(k+1)%len(poly)]
                    normal = _unit((q[1]-p[1], p[0]-q[0], 0))
                    ax = (normal[0], normal[1])
                    av = [_dot((x[0], x[1], 0), normal) for x in a]
                    bv = [_dot((x[0], x[1], 0), normal) for x in b]
                    separation = max(min(bv)-max(av), min(av)-max(bv))
                    best = max(best, separation)
            if best < least[0]:
                least = (best, name)
        return least

    def _sectionInWall(self, g, s):
        if abs(s) >= self.ro:
            return ([], [])
        sign = -1 if s < 0 else 1
        angle = self._angle(g, s)
        co, si = math.cos(angle), math.sin(angle)
        rect = [(u*co-v*si, u*si+v*co) for u, v in self._boreCorners(g, sign)]
        offset = _dot(_sub(g['origin'], self.C), self.nHat)
        vertical = _dot(g['u'], self.nHat)
        rise = self.v[INPUT_ID_CAGE_RISE]
        xa, xb = (-rise-offset)/vertical, (rise-offset)/vertical
        xa, xb = min(xa, xb), max(xa, xb)
        rect = _clip(_clip(rect, 1, 0, xb), -1, 0, -xa)
        near = math.sqrt(max(0, self.ri*self.ri-s*s))
        far = math.sqrt(max(0, self.ro*self.ro-s*s))
        return tuple(_clip(_clip(rect, 0, side, far), 0, -side, -near) for side in (-1, 1))

    def _wallCorners(self, bore):
        g = bore['g']
        lo, hi = self._span(bore)
        count = int(math.floor((hi-lo)/0.0001))+1
        for k in range(count):
            s = lo+k*0.0001
            for piece in self._sectionInWall(g, s):
                for x, y in piece:
                    yield _add(_add(_add(g['origin'], _mul(g['u'], x)), _mul(g['v'], y)),
                               _mul(g['dir'], s))

    def _channelTop(self):
        top = 0
        for bore in self.bores:
            g, sign = bore['g'], bore['sign']
            count = int(math.floor((self.sOut-self.sIn)/0.001))+1
            for k in range(count):
                s = sign*(self.sIn+k*0.001)
                if abs(s) >= self.ro:
                    continue
                angle = self._angle(g, s)
                co, si = math.cos(angle), math.sin(angle)
                corners = self._boreCorners(g, sign)
                ys = [abs(u*si+v*co) for u, v in corners]
                if math.hypot(s, max(ys)) < self.ri:
                    continue
                for u, v in corners:
                    x = u*co-v*si
                    z = abs(_dot(_sub(g['origin'], self.C), self.nHat)+x*_dot(g['u'], self.nHat))
                    top = max(top, z)
        return top

    def _channelSections(self, bore):
        g = bore['g']
        lo, hi = self._span(bore)
        def make(s):
            pieces = self._sectionInWall(g, s)
            circles = []
            for poly in pieces:
                if not poly:
                    circles.append((0, 0, 0))
                    continue
                cx = sum(x for x, _ in poly)/len(poly)
                cy = sum(y for _, y in poly)/len(poly)
                radius = max(math.hypot(x-cx, y-cy) for x, y in poly)
                circles.append((cx, cy, radius))
            return (s, pieces, circles)
        out = []
        count = int(math.floor((hi-lo)/0.0002))+1
        for k in range(count):
            s = lo+k*0.0002
            row = make(s)
            if out and tuple(bool(p) for p in row[1]) != tuple(bool(p) for p in out[-1][1]):
                prev = out[-1][0]
                fineCount = int(math.floor((s-prev)/0.00001))
                for j in range(1, fineCount):
                    out.append(make(prev+j*0.00001))
            out.append(row)
        return out

    def _gap2(self, poly, x, y):
        if not poly:
            return math.inf
        inside = len(poly) >= 3
        best = math.inf
        for i, p in enumerate(poly):
            q = poly[(i+1)%len(poly)]
            ex, ey = q[0]-p[0], q[1]-p[1]
            if ex*(y-p[1])-ey*(x-p[0]) < 0:
                inside = False
            length2 = ex*ex+ey*ey
            t = max(0, min(1, ((x-p[0])*ex+(y-p[1])*ey)/length2)) if length2 > 0 else 0
            dx, dy = x-p[0]-t*ex, y-p[1]-t*ey
            best = min(best, dx*dx+dy*dy)
        return 0 if inside else best

    def _wallGap(self, bore, point, reach):
        g = bore['g']
        d = _sub(point, g['origin'])
        x, y, sq = _dot(d, g['u']), _dot(d, g['v']), _dot(d, g['dir'])
        lo, hi = self._span(bore)
        axis = math.hypot(math.hypot(x, y), sq-max(lo, min(hi, sq)))
        if axis-self.boreCorner >= reach:
            return reach
        rows = self.channelSections[bore['index']]
        stations = self.channelStations[bore['index']]
        best = reach*reach
        def visit(row):
            nonlocal best
            s, pieces, circles = row
            ds = (sq-s)**2
            if ds >= best:
                return False
            for poly, (cx, cy, radius) in zip(pieces, circles):
                if not poly:
                    continue
                outside = math.hypot(x-cx, y-cy)-radius
                if outside > 0 and ds+outside*outside >= best:
                    continue
                best = min(best, ds+self._gap2(poly, x, y))
            return True
        loIndex, hiIndex = 0, len(stations)
        while loIndex < hiIndex:
            middle = (loIndex+hiIndex)//2
            if stations[middle] < sq:
                loIndex = middle+1
            else:
                hiIndex = middle
        at = loIndex
        for i in range(at, len(rows)):
            if not visit(rows[i]):
                break
        for i in range(at-1, -1, -1):
            if not visit(rows[i]):
                break
        return math.sqrt(best)

    def _windowPoint(self, window, t, z, a):
        return _add(_add(_add(self.C, _mul(window['across'], t)), _mul(self.nHat, z)),
                    _mul(window['facing'], a))

    def _chord(self, t):
        return math.sqrt(max(0, self.ri*self.ri-t*t)), math.sqrt(max(0, self.ro*self.ro-t*t))

    def _windowEnd(self, window, far, side):
        need = self.v[INPUT_ID_COLLAR_WALL]+0.01/math.sqrt(2)+0.0005
        step = 0.01
        def clear(extent):
            t = side*extent
            lean = window['lean']
            zLow = max(window['lo']-lean*t, window['bottom']+lean*t)
            zHigh = min(window['hi']-lean*t, window['top']+lean*t)
            if zLow > zHigh:
                return False
            a0, a1 = self._chord(t)
            def near(z, a):
                point = self._windowPoint(window, t, z, a)
                return any(self._wallGap(b, point, need) < need for b in far)
            for z in (zLow, zHigh):
                count = int(math.ceil((a1-a0)/step))
                for k in range(count+1):
                    if near(z, min(a0+k*step, a1)):
                        return False
            n = max(1, int(math.ceil((zHigh-zLow)/step)))
            for a in (a0, a1):
                for k in range(1, n):
                    if near(zLow+(zHigh-zLow)*k/n, a):
                        return False
                count = int(math.floor(2*window['zLimit']/step))+1
                for k in range(count):
                    z = -window['zLimit']+k*step
                    if zLow < z < zHigh and near(z, a):
                        return False
            return True
        lower = 0
        upper = min(self.ri, self.ro/math.sqrt(2))*(1-1e-9)
        for _ in range(24):
            middle = (lower+upper)/2
            if clear(middle):
                lower = middle
            else:
                upper = middle
        return lower

    def _newWindow(self, facing):
        across = _cross(self.nHat, facing)
        window = {'facing': facing, 'across': across}
        flanks, far = [], []
        for bore in self.bores:
            (flanks if _dot(_sub(self._crossing(bore), self.C), facing) > 0 else far).append(bore)
        if len(flanks) != 2:
            raise RuntimeError(f'Window has {len(flanks)} flanking bores, expected two')
        flanks.sort(key=lambda b: _dot(_sub(self._crossing(b), self.C), self.nHat))
        low, high = flanks
        lean = 1 if _dot(_sub(self._crossing(high), self._crossing(low)), across) >= 0 else -1
        window['lean'] = lean
        lowReach, highReach = -math.inf, math.inf
        for point in self._wallCorners(low):
            delta = _sub(point, self.C)
            lowReach = max(lowReach, _dot(delta, self.nHat)+lean*_dot(delta, across))
        for point in self._wallCorners(high):
            delta = _sub(point, self.C)
            highReach = min(highReach, _dot(delta, self.nHat)+lean*_dot(delta, across))
        wall = self.v[INPUT_ID_COLLAR_WALL]
        window['lo'] = lowReach+math.sqrt(2)*wall
        window['hi'] = highReach-math.sqrt(2)*wall
        zLimit = self._channelTop()
        window['zLimit'] = zLimit
        window['top'] = min(2*zLimit-window['hi'], window['hi']+math.sqrt(2)*self.ri)
        window['bottom'] = max(-2*zLimit-window['lo'], window['lo']-math.sqrt(2)*self.ri)
        window['left'] = -self._windowEnd(window, far, -1)
        window['right'] = self._windowEnd(window, far, 1)
        poly = [(-2*self.ro, -2*self.ro), (2*self.ro, -2*self.ro),
                (2*self.ro, 2*self.ro), (-2*self.ro, 2*self.ro)]
        for a, b, c in ((lean, 1, window['hi']), (-lean, -1, -window['lo']),
                        (-lean, 1, window['top']), (lean, -1, -window['bottom']),
                        (1, 0, window['right']), (-1, 0, -window['left'])):
            poly = _clip(poly, a, b, c)
        corners = []
        for point in poly:
            if not corners or math.hypot(point[0]-corners[-1][0], point[1]-corners[-1][1]) >= 0.0001:
                corners.append(point)
        if len(corners) > 1 and math.hypot(corners[0][0]-corners[-1][0], corners[0][1]-corners[-1][1]) < 0.0001:
            corners.pop()
        window['corners'] = corners
        area = sum(p[0]*q[1]-q[0]*p[1] for p, q in zip(corners, corners[1:]+corners[:1]))/2 if corners else 0
        if window['hi'] <= window['lo']:
            window['reason'] = 'the flanking bores leave no band between them'
        elif window['right'] <= window['left'] or len(corners) < 3 or area <= 0:
            window['reason'] = 'the far bores leave the band no length'
        else:
            window['reason'] = ''
        return window

    def _precomputeSearch(self):
        self.bores = self._bores()
        separation, gap = self._channelSeparation()
        if separation < self.v[INPUT_ID_COLLAR_WALL]:
            raise ValueError(f'collarWall exceeds {gap} bore separation {separation*10:.3f} mm')
        self.channelSections = [self._channelSections(b) for b in self.bores]
        self.channelStations = [[row[0] for row in rows] for rows in self.channelSections]
        directions = (self.kHat, _mul(self.kHat, -1)) if self.sigma <= math.pi/2 else (
            self.eHat, _mul(self.eHat, -1))
        names = ('+k', '-k') if self.sigma <= math.pi/2 else ('+e', '-e')
        self.windows = []
        for direction, name in zip(directions, names):
            window = self._newWindow(direction)
            window['name'] = name
            if window['reason']:
                futil.log(f"No window facing {name}: {window['reason']}")
            else:
                self.windows.append(window)

    def _sleeveSketch(self) -> adsk.fusion.Profile:
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(self.plane)
        sketch.name = 'Sleeve'
        centre: adsk.core.Point3D = _local(sketch, self.C)
        for radius in (self.ri, self.ro):
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+radius, centre.y, 0)
            diameter: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameter.parameter.value = 2*radius
        _require_sketch(sketch)
        matches = []
        for profile in sketch.profiles:
            if profile.profileLoops.count == 2:
                matches.append(profile)
        if len(matches) != 1:
            raise RuntimeError(f'Sleeve has {len(matches)} annular profiles, expected one')
        return matches[0]

    def _extrudeSleeve(self, ringProfile: adsk.fusion.Profile):
        design: adsk.fusion.Component = self.designOcc.component
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        riseValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.v[INPUT_ID_CAGE_RISE])
        extrudeInput.setSymmetricExtent(riseValue, False)
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        self.cageBody = _one_body(extrudeFeature, 'Sleeve extrude')
        self.cageBody.name = 'Cage'

    def _borePlane(self, bore):
        design: adsk.fusion.Component = self.designOcc.component
        index = bore['index']//2
        key = 'bore-' if bore['sign'] < 0 else 'bore+'
        boreLine: adsk.fusion.SketchLine = self.pathLines[index][key]
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        startFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0)
        planeInput.setByDistanceOnPath(boreLine, startFraction)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = bore['name']+' Plane'
        return plane, boreLine

    def _boreSketch(self, bore, plane):
        design: adsk.fusion.Component = self.designOcc.component
        g = bore['g']
        lo, _ = self._span(bore)
        angle = self._angle(g, lo)
        hw, vLo, vHi = self._opening(g, bore['sign'])
        uB, uF = -hw, hw
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = bore['name']
        sketch.isComputeDeferred = True
        origin = _add(g['origin'], _mul(g['dir'], lo))
        oLocal: adsk.core.Point3D = _local(sketch, origin)
        cpLocal: adsk.core.Point3D = _local(sketch, _add(origin, _mul(g['u'], self.axisOffset/2)))
        O: adsk.fusion.SketchPoint = sketch.sketchPoints.add(oLocal)
        Cp: adsk.fusion.SketchPoint = sketch.sketchPoints.add(cpLocal)
        Ru: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True
        eLocal: adsk.core.Point3D = _local(sketch, self._world(g, uF, 0, lo))
        E: adsk.fusion.SketchPoint = sketch.sketchPoints.add(eLocal)
        K: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(O, E)
        K.isConstruction = True
        cornerLocals = [_local(sketch, self._world(g, u, v, lo))
                        for u, v in ((uB, vLo), (uF, vLo), (uF, vHi), (uB, vHi))]
        corners = [sketch.sketchPoints.add(local) for local in cornerLocals]
        P0, P1, P2, P3 = corners
        L1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P0, P1)
        L2: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P1, P2)
        L3: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P2, P3)
        L4: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(P3, P0)
        O.isFixed = True
        Cp.isFixed = True
        # Fusion creates the two driving dimensions before any rectangle constraints.
        lengthText: adsk.core.Point3D = adsk.core.Point3D.create(
            (oLocal.x+eLocal.x)/2, (oLocal.y+eLocal.y)/2+uF/4, 0)
        lengthDim = sketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDim.parameter.value = uF
        ru = _unit((cpLocal.x-oLocal.x, cpLocal.y-oLocal.y, 0))
        if abs(math.sin(angle)) >= math.sqrt(0.5):
            ray = _unit((eLocal.x-oLocal.x, eLocal.y-oLocal.y, 0))
            vertex = (oLocal.x, oLocal.y)
            angleText: adsk.core.Point3D = adsk.core.Point3D.create(
                vertex[0]+(ru[0]+ray[0])*uF/3,
                vertex[1]+(ru[1]+ray[1])*uF/3, 0)
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(
                Ru, K, angleText)
        else:
            p = cornerLocals[1]
            q = cornerLocals[2]
            line = (q.x-p.x, q.y-p.y)
            rhs = (p.x-oLocal.x, p.y-oLocal.y)
            determinant = ru[0]*line[1]-ru[1]*line[0]
            if abs(determinant) < 1e-12:
                raise RuntimeError(f"{bore['name']} Ru and L2 do not intersect distinctly")
            t = (rhs[0]*line[1]-rhs[1]*line[0])/determinant
            vertex = (oLocal.x+t*ru[0], oLocal.y+t*ru[1])
            toCp = (cpLocal.x-vertex[0], cpLocal.y-vertex[1], 0)
            if _length(toCp) < 1e-12:
                toCp = (oLocal.x-vertex[0], oLocal.y-vertex[1], 0)
            rayRu = _unit(toCp)
            rayL2 = _unit((line[0], line[1], 0))
            angleText: adsk.core.Point3D = adsk.core.Point3D.create(
                vertex[0]+(rayRu[0]+rayL2[0])*uF/3,
                vertex[1]+(rayRu[1]+rayL2[1])*uF/3, 0)
            angleDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(
                Ru, L2, angleText)
            ru, ray = rayRu, rayL2
        angleDim.parameter.value = math.acos(max(-1, min(1, _dot(ru, ray))))
        sketch.geometricConstraints.addParallel(L1, K)
        sketch.geometricConstraints.addParallel(L3, K)
        sketch.geometricConstraints.addParallel(L4, L2)
        sketch.geometricConstraints.addCoincident(E, L2)
        sketch.geometricConstraints.addPerpendicular(L2, K)
        lowerText: adsk.core.Point3D = adsk.core.Point3D.create(
            (cornerLocals[0].x+eLocal.x)/2, (cornerLocals[0].y+eLocal.y)/2, 0)
        upperText: adsk.core.Point3D = adsk.core.Point3D.create(
            (cornerLocals[2].x+eLocal.x)/2, (cornerLocals[2].y+eLocal.y)/2, 0)
        widthText: adsk.core.Point3D = adsk.core.Point3D.create(
            (cornerLocals[0].x+cornerLocals[1].x)/2,
            (cornerLocals[0].y+cornerLocals[1].y)/2+uF/4, 0)
        lower: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)
        lower.parameter.value = abs(vLo)
        upper: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)
        upper.parameter.value = abs(vHi)
        width: adsk.fusion.SketchOffsetDimension = sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)
        width.parameter.value = uF-uB
        sketch.isComputeDeferred = False
        _require_sketch(sketch)
        profile: adsk.fusion.Profile = find_profile_by_curve_counts(sketch, lines=4)
        for index, (corner, expected) in enumerate(zip(corners, cornerLocals)):
            solved: adsk.core.Point3D = corner.geometry
            measured = solved.distanceTo(expected)
            if measured > 0.0001:
                raise RuntimeError(f"{sketch.name} corner {index} moved {measured*10:.4f} mm")
        return profile

    def _cutBore(self, bore, boreLine: adsk.fusion.SketchLine, profile: adsk.fusion.Profile):
        design: adsk.fusion.Component = self.designOcc.component
        borePath: adsk.fusion.Path = design.features.createPath(boreLine, False)
        sweepInput: adsk.fusion.SweepFeatureInput = design.features.sweepFeatures.createInput(
            profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)
        value: adsk.core.ValueInput = adsk.core.ValueInput.createByReal((self.sOut-self.sIn)/self.lam)
        sweepInput.twistAngle = value
        cageBody: adsk.fusion.BRepBody = self.cageBody
        sweepInput.participantBodies = [cageBody]
        sweepFeature: adsk.fusion.SweepFeature = design.features.sweepFeatures.add(sweepInput)
        self.cageBody = _one_body(sweepFeature, bore['name']+' sweep')
        g = bore['g']
        sc = bore['sign']*self.v[INPUT_ID_CAGE_RADIUS]
        halfWidth = self.v[INPUT_ID_RIBBON_WIDTH]/2
        clearance = self.v[INPUT_ID_CLEARANCE]
        for side in (-1, 1):
            probePoint: adsk.core.Point3D = _point(self._world(g, side*(halfWidth+clearance/2), 0, sc))
            cageBody: adsk.fusion.BRepBody = self.cageBody
            result = cageBody.pointContainment(probePoint)
            if result != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise RuntimeError(f"{bore['name']} u-side {side} probe remained in cage: {result}")
        allowance = self.v[INPUT_ID_ROOF_ALLOWANCE]
        if bore['sign'] == self._levelBore(g) and allowance > 0:
            roofSign = self._roofSide(g, bore['sign'])
            for tag, v, expected in (
                ('roof', roofSign*(self.ht+allowance/2), adsk.fusion.PointContainment.PointOutsidePointContainment),
                ('floor', -roofSign*(self.ht+allowance/2), adsk.fusion.PointContainment.PointInsidePointContainment),
            ):
                probePoint: adsk.core.Point3D = _point(self._world(g, 0, v, sc))
                cageBody: adsk.fusion.BRepBody = self.cageBody
                result = cageBody.pointContainment(probePoint)
                if result != expected:
                    raise RuntimeError(f"{bore['name']} {tag} probe read {result}, expected {expected}")

    def _windowPlane(self):
        design: adsk.fusion.Component = self.designOcc.component
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        if self.sigma <= math.pi/2:
            rightAngle: adsk.core.ValueInput = adsk.core.ValueInput.createByString('90 deg')
            planeInput.setByAngle(self.anchorLine, rightAngle, self.plane)
        else:
            midpointFraction: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(0.5)
            planeInput.setByDistanceOnPath(self.anchorLine, midpointFraction)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = 'Window Plane'
        return plane

    def _windowSketch(self, window, plane):
        design: adsk.fusion.Component = self.designOcc.component
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = 'Window '+window['name']
        points = []
        for t, z in window['corners']:
            world = _add(_add(self.C, _mul(window['across'], t)), _mul(self.nHat, z))
            worldPoint: adsk.core.Point3D = _point(world)
            local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
            local.z = 0
            points.append(sketch.sketchPoints.add(local))
        for i, startPoint in enumerate(points):
            endPoint = points[(i+1)%len(points)]
            sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
        for point in points:
            point.isFixed = True
        _require_sketch(sketch)
        if sketch.profiles.count != 1:
            raise RuntimeError(f"{sketch.name} has {sketch.profiles.count} profiles, expected one")
        profile: adsk.fusion.Profile = sketch.profiles.item(0)
        return sketch, profile

    def _cutWindow(self, window, sketch: adsk.fusion.Sketch, profile: adsk.fusion.Profile):
        design: adsk.fusion.Component = self.designOcc.component
        tc = sum(p[0] for p in window['corners'])/len(window['corners'])
        zc = sum(p[1] for p in window['corners'])/len(window['corners'])
        a0, a1 = self._chord(tc)
        probePoint: adsk.core.Point3D = _point(self._windowPoint(window, tc, zc, (a0+a1)/2))
        cageBody: adsk.fusion.BRepBody = self.cageBody
        before = cageBody.pointContainment(probePoint)
        if before != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise RuntimeError(f"Window {window['name']} probe before cut read {before}, expected inside")
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(self.ro+0.1)
        extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        directionPoint: adsk.core.Point3D = _point(_add(self.C, window['facing']))
        local: adsk.core.Point3D = sketch.modelToSketchSpace(directionPoint)
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if local.z > 0
                     else adsk.fusion.ExtentDirections.NegativeExtentDirection)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeInput.participantBodies = [cageBody]
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        self.cageBody = _one_body(extrudeFeature, 'Window '+window['name'])
        cageBody: adsk.fusion.BRepBody = self.cageBody
        after = cageBody.pointContainment(probePoint)
        if after != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise RuntimeError(f"Window {window['name']} probe after cut read {after}, expected outside")

    def _markerPlane(self):
        design: adsk.fusion.Component = self.designOcc.component
        rise = self.v[INPUT_ID_CAGE_RISE]
        inset = min(0.01, self.v[INPUT_ID_COLLAR_WALL]/2)
        bPlane: adsk.fusion.ConstructionPlane = self.gearAxisPlanes[1]
        normal = _unit(_coords(bPlane.geometry.normal))
        signedOffset = (rise-inset-self.axisOffset/2)*_dot(normal, self.nHat)
        planeInput: adsk.fusion.ConstructionPlaneInput = design.constructionPlanes.createInput()
        offsetValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(signedOffset)
        planeInput.setByOffset(bPlane, offsetValue)
        plane: adsk.fusion.ConstructionPlane = design.constructionPlanes.add(planeInput)
        plane.name = 'Bore Marker Plane'
        measured = _dot(_sub(_coords(plane.geometry.origin), self.C), self.nHat)
        if abs(measured-(rise-inset)) > 1e-4:
            raise RuntimeError(f'Marker plane is {measured*10:.4f} mm above centre, expected {(rise-inset)*10:.4f}')
        return plane

    def _markerSketch(self, bore, plane):
        design: adsk.fusion.Component = self.designOcc.component
        sign = bore['sign']
        g = bore['g']
        halfSize = min(0.1, self.v[INPUT_ID_COLLAR_HALF]/2)
        inset = min(0.01, self.v[INPUT_ID_COLLAR_WALL]/2)
        markCentre = _add(_add(self.C, _mul(self.nHat, self.v[INPUT_ID_CAGE_RISE]-inset)),
                          _mul(g['dir'], sign*self.v[INPUT_ID_CAGE_RADIUS]))
        shape = 'Circle' if sign > 0 else 'Square'
        sketch: adsk.fusion.Sketch = design.sketches.add(plane)
        sketch.name = bore['name']+' '+shape+' Marker'
        if sign > 0:
            centre: adsk.core.Point3D = _local(sketch, markCentre)
            circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, halfSize)
            circle.centerSketchPoint.isFixed = True
            textPoint: adsk.core.Point3D = adsk.core.Point3D.create(centre.x+halfSize, centre.y, 0)
            dim: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            dim.parameter.value = 2*halfSize
        else:
            points = []
            for x, y in ((-halfSize, -halfSize), (halfSize, -halfSize),
                         (halfSize, halfSize), (-halfSize, halfSize)):
                world = _add(_add(markCentre, _mul(self.eHat, x)), _mul(self.kHat, y))
                worldPoint: adsk.core.Point3D = _point(world)
                local: adsk.core.Point3D = sketch.modelToSketchSpace(worldPoint)
                local.z = 0
                points.append(sketch.sketchPoints.add(local))
            for i, startPoint in enumerate(points):
                endPoint = points[(i+1)%4]
                sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)
            for point in points:
                point.isFixed = True
        _require_sketch(sketch)
        if sketch.profiles.count != 1:
            raise RuntimeError(f'{sketch.name} has {sketch.profiles.count} profiles, expected one')
        profile: adsk.fusion.Profile = sketch.profiles.item(0)
        return sketch, profile, markCentre

    def _extrudeMarker(self, sketch: adsk.fusion.Sketch, profile: adsk.fusion.Profile, centre):
        design: adsk.fusion.Component = self.designOcc.component
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inset = min(0.01, self.v[INPUT_ID_COLLAR_WALL]/2)
        distanceValue: adsk.core.ValueInput = adsk.core.ValueInput.createByReal(inset+0.04)
        extent: adsk.fusion.DistanceExtentDefinition = adsk.fusion.DistanceExtentDefinition.create(distanceValue)
        markCentrePlusNormal: adsk.core.Point3D = _point(_add(centre, self.nHat))
        local: adsk.core.Point3D = sketch.modelToSketchSpace(markCentrePlusNormal)
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if local.z > 0
                     else adsk.fusion.ExtentDirections.NegativeExtentDirection)
        extrudeInput.setOneSideExtent(extent, direction)
        extrudeFeature: adsk.fusion.ExtrudeFeature = design.features.extrudeFeatures.add(extrudeInput)
        return _one_body(extrudeFeature, sketch.name+' extrude')

    def buildCage(self):
        ringProfile = self._sleeveSketch()
        self._extrudeSleeve(ringProfile)
        for bore in self.bores:
            plane, boreLine = self._borePlane(bore)
            profile = self._boreSketch(bore, plane)
            self._cutBore(bore, boreLine, profile)
        if self.windows:
            plane = self._windowPlane()
            for window in self.windows:
                sketch, profile = self._windowSketch(window, plane)
                self._cutWindow(window, sketch, profile)
        markerPlane = self._markerPlane()
        for bore in self.bores:
            sketch, profile, centre = self._markerSketch(bore, markerPlane)
            markBody = self._extrudeMarker(sketch, profile, centre)
            cageBody: adsk.fusion.BRepBody = self.cageBody
            self.cageBody = self._join(cageBody, markBody, bore['name']+' marker join')
        self.cageBody.name = 'Cage'

    def relocateBodies(self):
        for index, targetOccurrence in enumerate(self.gearOccs):
            body: adsk.fusion.BRepBody = self.gearBodies[index]
            body.name = 'Gear A' if index == 0 else 'Gear B'
            body.moveToComponent(targetOccurrence)
        body: adsk.fusion.BRepBody = self.cageBody
        body.name = 'Cage'
        body.moveToComponent(self.cageOcc)
        futil.log('Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.')
