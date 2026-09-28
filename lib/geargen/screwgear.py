\
# Screw/screw gearing: two racks, each twisted into a helix about its own centre line, each
# moving by a screw motion in a cage that holds them. Generated from spec/screwgear/steps.md.

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
INPUT_ID_RING_RADIUS = 'ringRadius'
INPUT_ID_CAGE_RADIUS = 'cageRadius'
INPUT_ID_CAGE_RISE = 'cageRise'
INPUT_ID_CLEARANCE = 'clearance'
INPUT_ID_COLLAR_HALF = 'collarHalf'
INPUT_ID_COLLAR_WALL = 'collarWall'
INPUT_ID_ROD_DIAMETER = 'rodDiameter'
INPUT_ID_RING_WIRE = 'ringWire'
INPUT_ID_CROSS_ANGLE = 'crossAngle'
INPUT_ID_ENGAGEMENT = 'engagement'
INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'
INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'
INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'

GROUP_ID_RIBBON = 'ribbonGroup'
GROUP_ID_FRAME = 'frameGroup'
GROUP_ID_MESH = 'meshGroup'

CELL_TEETH = 4


# ---- plain (x, y, z) tuple vector math -------------------------------------------------------
# Fusion's own Point3D/Vector3D objects have no elementwise arithmetic beyond scaleBy/add/cross,
# so the frame math (S03, S06) is done on plain tuples and converted at the Fusion API boundary.

def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _vlen(a):
    return math.sqrt(_vdot(a, a))


def _vnorm(a):
    length = _vlen(a)
    return (a[0] / length, a[1] / length, a[2] / length)


def _rotate(v, angle, axis):
    # Rodrigues' rotation formula, axis assumed unit.
    c = math.cos(angle)
    s = math.sin(angle)
    dot = _vdot(axis, v)
    cross = _vcross(axis, v)
    return _vadd(_vadd(_vscale(v, c), _vscale(cross, s)), _vscale(axis, dot * (1 - c)))


def _point3d(v) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(v[0], v[1], v[2])


def _vector3d(v) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


def _from_point3d(p) -> tuple:
    return (p.x, p.y, p.z)


def _azimuth(point, centre, eHat, kHat):
    # Azimuth about +n̂ from ê, dropping the n̂ component (S03 "The rod search").
    rel = _vsub(point, centre)
    x = _vdot(rel, eHat)
    y = _vdot(rel, kHat)
    return math.atan2(y, x)


def _gear_frames(centre, eHat, nHat, sigma, axisOffset):
    # Spec §1: dirA/dirB = ê rotated by +-Sigma/2 about n̂; originA/originB = C -+ (A/2)n̂;
    # uA = +n̂, uB = -n̂; v = dir x u.
    dirA = _rotate(eHat, sigma / 2, nHat)
    dirB = _rotate(eHat, -sigma / 2, nHat)
    originA = _vsub(centre, _vscale(nHat, axisOffset / 2))
    originB = _vadd(centre, _vscale(nHat, axisOffset / 2))
    uA = nHat
    uB = _vscale(nHat, -1.0)
    vA = _vcross(dirA, uA)
    vB = _vcross(dirB, uB)
    return [
        {'origin': originA, 'dir': dirA, 'u': uA, 'v': vA},
        {'origin': originB, 'dir': dirB, 'u': uB, 'v': vB},
    ]


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs = command.commandInputs

        # Selections first: Fusion focuses the first SelectionCommandInput ([PB-AUTOFOCUS-FIRST]).
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

        ribbonGroup = inputs.addGroupCommandInput(GROUP_ID_RIBBON, 'Ribbon')
        ribbonGroup.isExpanded = True
        rg = ribbonGroup.children
        rg.addValueInput(INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm',
                          adsk.core.ValueInput.createByReal(1.5))
        rg.addValueInput(INPUT_ID_TOOTH_COUNT, 'Tooth Count', '',
                          adsk.core.ValueInput.createByReal(68))
        rg.addValueInput(INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm',
                          adsk.core.ValueInput.createByReal(4.95))
        rg.addValueInput(INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm',
                          adsk.core.ValueInput.createByReal(0.375))
        rg.addValueInput(INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm',
                          adsk.core.ValueInput.createByReal(0.2625))
        rg.addValueInput(INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm',
                          adsk.core.ValueInput.createByReal(0.2625))

        frameGroup = inputs.addGroupCommandInput(GROUP_ID_FRAME, 'Frame')
        frameGroup.isExpanded = True
        fg = frameGroup.children
        fg.addValueInput(INPUT_ID_RING_RADIUS, 'Ring Radius', 'mm',
                          adsk.core.ValueInput.createByReal(1.6875))
        fg.addValueInput(INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm',
                          adsk.core.ValueInput.createByReal(1.5))
        fg.addValueInput(INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm',
                          adsk.core.ValueInput.createByReal(2.025))
        fg.addValueInput(INPUT_ID_CLEARANCE, 'Clearance', 'mm',
                          adsk.core.ValueInput.createByReal(0.045))
        fg.addValueInput(INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm',
                          adsk.core.ValueInput.createByReal(0.3))
        fg.addValueInput(INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm',
                          adsk.core.ValueInput.createByReal(0.3))
        fg.addValueInput(INPUT_ID_ROD_DIAMETER, 'Rod Diameter', 'mm',
                          adsk.core.ValueInput.createByReal(0.3))
        fg.addValueInput(INPUT_ID_RING_WIRE, 'Ring Wire', 'mm',
                          adsk.core.ValueInput.createByReal(0.375))

        meshGroup = inputs.addGroupCommandInput(GROUP_ID_MESH, 'Mesh (from the mesh search)')
        meshGroup.isExpanded = False
        mg = meshGroup.children
        mg.addValueInput(INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg',
                          adsk.core.ValueInput.createByReal(math.radians(80)))
        mg.addValueInput(INPUT_ID_ENGAGEMENT, 'Engagement', 'mm',
                          adsk.core.ValueInput.createByReal(0.075))
        mg.addValueInput(INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg',
                          adsk.core.ValueInput.createByReal(math.radians(15)))
        mg.addValueInput(INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg',
                          adsk.core.ValueInput.createByReal(math.radians(15)))
        mg.addValueInput(INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm',
                          adsk.core.ValueInput.createByReal(-0.131))


class ScrewGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'ScrewGear'

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        for index in (0, 1):
            self.buildGear(index)
        self.buildCage()
        self.relocateBodies()

    # ---- selection helper --------------------------------------------------------------------

    def _selectOne(self, inputs, inputId, label):
        entities = get_selection(inputs, inputId)
        if len(entities) != 1:
            raise Exception(f'{label}: expected exactly one selection, got {len(entities)}')
        return entities[0]

    # ---- S03: read, check and precompute the inputs ------------------------------------------

    def processInputs(self, inputs: adsk.core.CommandInputs):
        parentEntity = self._selectOne(inputs, INPUT_ID_PARENT, 'Parent Component')
        if isinstance(parentEntity, adsk.fusion.Occurrence):
            self.parentComponent = parentEntity.component
        else:
            self.parentComponent = parentEntity
        self.plane = self._selectOne(inputs, INPUT_ID_PLANE, 'Target Plane')
        self.point = self._selectOne(inputs, INPUT_ID_POINT, 'Centre Point')

        design: adsk.fusion.Design = get_design()
        unitsManager: adsk.core.UnitsManager = design.unitsManager

        def readValue(cmdInputs: adsk.core.CommandInputs, um: adsk.core.UnitsManager, inputId, units):
            valueInput = cmdInputs.itemById(inputId)
            if valueInput is None:
                raise Exception(f'Missing dialog input "{inputId}"')
            return um.evaluateExpression(valueInput.expression, units)

        W = readValue(inputs, unitsManager, INPUT_ID_RIBBON_WIDTH, 'cm')
        toothCountRaw = readValue(inputs, unitsManager, INPUT_ID_TOOTH_COUNT, '')
        twistLead = readValue(inputs, unitsManager, INPUT_ID_TWIST_LEAD, 'cm')
        T = readValue(inputs, unitsManager, INPUT_ID_RIBBON_THICKNESS, 'cm')
        P = readValue(inputs, unitsManager, INPUT_ID_TOOTH_PITCH, 'cm')
        H = readValue(inputs, unitsManager, INPUT_ID_TOOTH_HEIGHT, 'cm')
        ringRadius = readValue(inputs, unitsManager, INPUT_ID_RING_RADIUS, 'cm')
        cageRadius = readValue(inputs, unitsManager, INPUT_ID_CAGE_RADIUS, 'cm')
        cageRise = readValue(inputs, unitsManager, INPUT_ID_CAGE_RISE, 'cm')
        clearance = readValue(inputs, unitsManager, INPUT_ID_CLEARANCE, 'cm')
        collarHalf = readValue(inputs, unitsManager, INPUT_ID_COLLAR_HALF, 'cm')
        collarWall = readValue(inputs, unitsManager, INPUT_ID_COLLAR_WALL, 'cm')
        rodDiameter = readValue(inputs, unitsManager, INPUT_ID_ROD_DIAMETER, 'cm')
        ringWire = readValue(inputs, unitsManager, INPUT_ID_RING_WIRE, 'cm')
        crossAngle = readValue(inputs, unitsManager, INPUT_ID_CROSS_ANGLE, 'rad')
        engagement = readValue(inputs, unitsManager, INPUT_ID_ENGAGEMENT, 'cm')
        mountAngleA = readValue(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_A, 'rad')
        mountAngleB = readValue(inputs, unitsManager, INPUT_ID_MOUNT_ANGLE_B, 'rad')
        assemblyPhase = readValue(inputs, unitsManager, INPUT_ID_ASSEMBLY_PHASE, 'cm')

        # Range check 1.
        for value, name in (
                (W, 'ribbonWidth'), (T, 'ribbonThickness'), (P, 'toothPitch'),
                (twistLead, 'twistLead'), (ringWire, 'ringWire'),
                (rodDiameter, 'rodDiameter'), (collarHalf, 'collarHalf'),
                (collarWall, 'collarWall'), (clearance, 'clearance')):
            if not (value > 0):
                raise Exception(f'{name} must be > 0, got {value}')
        if toothCountRaw != round(toothCountRaw) or toothCountRaw < 4:
            raise Exception(f'toothCount must be a whole number >= 4, got {toothCountRaw}')
        N = int(round(toothCountRaw))

        # 2.
        if not (0 < H < W / 2):
            raise Exception(f'toothHeight must be > 0 and < ribbonWidth/2 ({W / 2}), got {H}')

        # 3.
        if not (0 < engagement <= H):
            raise Exception(f'engagement must be > 0 and <= toothHeight ({H}), got {engagement}')

        # 4.
        if not (0 < crossAngle < math.pi):
            raise Exception(
                f'crossAngle must lie strictly between 0 and 180 deg, got '
                f'{math.degrees(crossAngle)}')

        # 5.
        if not (-P < assemblyPhase < P):
            raise Exception(
                f'assemblyPhase must lie strictly within +/- toothPitch ({P}), got '
                f'{assemblyPhase}')

        Sigma = crossAngle
        A = W - engagement
        Lambda = twistLead / (2 * math.pi)

        # 6.
        engagedHalfLength = 1.5 * math.sqrt(W * W - A * A) / math.sin(Sigma)
        if not (cageRadius - collarHalf > engagedHalfLength):
            raise Exception(
                f'cageRadius - collarHalf ({cageRadius - collarHalf}) must exceed the engaged '
                f'zone half-length ({engagedHalfLength})')
        if not (cageRadius + collarHalf + 0.1 < N * P / 2):
            raise Exception(
                f'cageRadius + collarHalf + 1 mm ({cageRadius + collarHalf + 0.1}) must be '
                f'under toothCount*toothPitch/2 ({N * P / 2})')

        L = N * P
        Z0 = [0.0, assemblyPhase]
        s0 = [Z0[0] - L / 2, Z0[1] - L / 2]
        Phi = [mountAngleA, mountAngleB]

        # 7: the rod search, in the frame's own local coordinates ([PB-... ] free of the world
        # frame S05/S06 build; azimuths and stations do not depend on where the frame stands).
        localC = (0.0, 0.0, 0.0)
        localE = (1.0, 0.0, 0.0)
        localN = (0.0, 0.0, 1.0)
        localK = (0.0, 1.0, 0.0)
        localFrames = _gear_frames(localC, localE, localN, Sigma, A)

        def theta(g, s):
            return s / Lambda + Phi[g]

        def hShadow(g, s):
            t = theta(g, s)
            return (W / 2) * abs(math.sin(t)) + (T / 2) * abs(math.cos(t))

        rodStep = 0.001  # cm, 0.01 mm

        def rodClears(psi):
            need = rodDiameter / 2 + clearance
            for g in (0, 1):
                gp = localFrames[g]
                foot = _vadd(localC, _vadd(
                    _vscale(localE, ringRadius * math.cos(psi)),
                    _vscale(localK, ringRadius * math.sin(psi))))
                rel = _vsub(foot, gp['origin'])
                sp = _vdot(rel, gp['dir'])
                wp = _vdot(rel, gp['v'])
                lo, hi = sp - need, sp + need
                k = math.ceil(lo / rodStep)
                while k * rodStep <= hi:
                    s = k * rodStep
                    h = hShadow(g, s)
                    if math.hypot(sp - s, max(0.0, abs(wp) - h)) < need:
                        return False
                    k += 1
            return True

        def rodShift(psiC):
            step = 0.25 * math.pi / 180
            fine = 0.001 * math.pi / 180
            shift = 0.0
            while shift <= 2 * math.pi:
                if rodClears(psiC + shift):
                    if shift == 0:
                        return 0.0
                    lo, hi = shift - step, shift
                    while hi - lo > fine:
                        mid = (lo + hi) / 2
                        if rodClears(psiC + mid):
                            hi = mid
                        else:
                            lo = mid
                    return hi
                shift += step
            return None

        collarLabels = [
            'gear A -R collar', 'gear A +R collar', 'gear B -R collar', 'gear B +R collar']
        collarGear = [0, 0, 1, 1]
        collarSc = [-cageRadius, cageRadius, -cageRadius, cageRadius]
        rodAzimuths = [0.0, 0.0, 0.0, 0.0]
        rodFeet = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
        for i in range(4):
            g = collarGear[i]
            sc = collarSc[i]
            gp = localFrames[g]
            crossing = _vadd(gp['origin'], _vscale(gp['dir'], sc))
            psiC = _azimuth(crossing, localC, localE, localK)
            shift = rodShift(psiC)
            if shift is None:
                raise Exception(f'ringRadius: no azimuth clears the {collarLabels[i]}')
            psi = psiC + shift
            rodAzimuths[i] = psi

            foot = _vadd(localC, _vadd(
                _vscale(localE, ringRadius * math.cos(psi)),
                _vscale(localK, ringRadius * math.sin(psi))))
            rel = _vsub(foot, gp['origin'])
            sp = _vdot(rel, gp['dir'])
            wp = _vdot(rel, gp['v'])
            t = theta(g, sp)
            hb = ((W / 2 + clearance) * abs(math.sin(t))
                  + (T / 2 + clearance) * abs(math.cos(t)))
            depth = abs(wp) - hb
            if not (0 < depth <= collarWall):
                raise Exception(
                    f'ringRadius: the rod for the {collarLabels[i]} stands {depth} cm outside '
                    f'its bore, expected (0, collarWall={collarWall}]')
            if not (abs(sp - sc) + rodDiameter / 2 <= collarHalf):
                raise Exception(
                    f'ringRadius: the rod for the {collarLabels[i]} misses the collar length '
                    f'(|sp-sc|={abs(sp - sc)}, rodDiameter/2={rodDiameter / 2}, '
                    f'collarHalf={collarHalf})')
            rodFeet[i] = foot

        for i in range(4):
            for j in range(i + 1, 4):
                d = _vlen(_vsub(rodFeet[i], rodFeet[j]))
                if d < rodDiameter + clearance:
                    raise Exception(
                        f'ringRadius: two rods ({collarLabels[i]}, {collarLabels[j]}) stand '
                        f'{d} cm apart, under rodDiameter + clearance ({rodDiameter + clearance})')

        # 8.
        hyp = math.hypot(W / 2, T / 2)
        spareLeft = cageRise - ringWire / 2 - (A / 2 + hyp)
        if not (spareLeft >= clearance):
            raise Exception(
                f'cageRise leaves {spareLeft} cm between the frame and the ribbons, under '
                f'clearance ({clearance})')

        # 9.
        if not (collarWall >= rodDiameter):
            raise Exception(f'collarWall ({collarWall}) must be at least rodDiameter ({rodDiameter})')

        n = int(max(math.ceil((P / Lambda) / math.radians(2)), 8))
        c = min(CELL_TEETH, N)
        q, r = divmod(N, c)

        self.W = W
        self.T = T
        self.H = H
        self.P = P
        self.N = N
        self.twistLead = twistLead
        self.ringRadius = ringRadius
        self.cageRadius = cageRadius
        self.cageRise = cageRise
        self.clearance = clearance
        self.collarHalf = collarHalf
        self.collarWall = collarWall
        self.rodDiameter = rodDiameter
        self.ringWire = ringWire
        self.engagement = engagement
        self.Lambda = Lambda
        self.Sigma = Sigma
        self.A = A
        self.L = L
        self.n = n
        self.c = c
        self.q = q
        self.r = r
        self.rodAzimuths = rodAzimuths
        footOrder = sorted(range(4), key=lambda i: rodAzimuths[i] % (2 * math.pi))
        self.footPsis = [rodAzimuths[i] % (2 * math.pi) for i in footOrder]
        self.gearParams = [
            {'Z0': Z0[0], 's0': s0[0], 'Phi': Phi[0]},
            {'Z0': Z0[1], 's0': s0[1], 'Phi': Phi[1]},
        ]
        self.pathLines = [{}, {}]
        self.gearBodies = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]
        futil.log('processInputs: rod search done, derived values computed')

    # ---- per-gear world geometry -------------------------------------------------------------

    def _theta(self, index, s):
        return s / self.Lambda + self.gearParams[index]['Phi']

    def _uTooth(self, index, s):
        Z0 = self.gearParams[index]['Z0']
        return self.W / 2 - self.H / 2 + (self.H / 2) * math.cos(2 * math.pi * (s - Z0) / self.P)

    def _point(self, index, s, u, v):
        gp = self.gearParams[index]
        theta = self._theta(index, s)
        base = _vadd(gp['origin'], _vscale(gp['dir'], s))
        uComp = u * math.cos(theta) - v * math.sin(theta)
        vComp = u * math.sin(theta) + v * math.cos(theta)
        return _vadd(base, _vadd(_vscale(gp['u'], uComp), _vscale(gp['v'], vComp)))

    # ---- S04: component tree ------------------------------------------------------------------

    def buildComponentTree(self):
        occurrence = self.getOccurrence()
        occurrence.component.name = 'Screw Gearing'
        topComponent = occurrence.component

        designOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        self.designOcc = designOcc

        gearAOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearAOcc.component.name = 'Gear A'
        gearBOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearBOcc.component.name = 'Gear B'
        self.gearOccs = [gearAOcc, gearBOcc]

        cageOcc = topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        cageOcc.component.name = 'Cage'
        self.cageOcc = cageOcc
        futil.log('buildComponentTree: Design, Gear A, Gear B, Cage created')

    # ---- S05, S06: Anchor sketch, axis planes, n̂, gear frames --------------------------------

    def buildAnchor(self):
        component = self.designOcc.component
        sketch = component.sketches.add(self.plane)
        sketch.name = 'Anchor'

        projected = sketch.project(self.point).item(0)
        p = projected.geometry
        start = adsk.core.Point3D.create(p.x - 0.5, p.y, 0)
        end = adsk.core.Point3D.create(p.x + 0.5, p.y, 0)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)

        sketch.geometricConstraints.addCoincident(projected, line)
        sketch.geometricConstraints.addMidPoint(projected, line)
        sketch.geometricConstraints.addHorizontal(line)
        textPoint = adsk.core.Point3D.create(p.x, p.y + 0.3, 0)
        dim = sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        dim.parameter.value = 1.0

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        C = _from_point3d(projected.worldGeometry)
        eHat = _vnorm(_vsub(
            _from_point3d(line.endSketchPoint.worldGeometry),
            _from_point3d(line.startSketchPoint.worldGeometry)))
        self.anchorLine = line
        self.C = C
        self.eHat = eHat

        constructionPlanes = component.constructionPlanes
        planeAInput = constructionPlanes.createInput()
        planeAInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-self.A / 2))
        planeA = constructionPlanes.add(planeAInput)
        planeA.name = 'Gear A Axis Plane'

        planeBInput = constructionPlanes.createInput()
        planeBInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(self.A / 2))
        planeB = constructionPlanes.add(planeBInput)
        planeB.name = 'Gear B Axis Plane'

        geomA = planeA.geometry
        originA = _from_point3d(geomA.origin)
        normalA = _from_point3d(geomA.normal)
        if _vdot(_vsub(C, originA), normalA) > 0:
            nHat = normalA
        else:
            nHat = _vscale(normalA, -1.0)

        for plane, label in ((planeA, 'Gear A Axis Plane'), (planeB, 'Gear B Axis Plane')):
            g = plane.geometry
            o = _from_point3d(g.origin)
            nrm = _from_point3d(g.normal)
            dist = abs(_vdot(_vsub(C, o), nrm))
            if abs(dist - self.A / 2) > 1e-6:
                raise Exception(
                    f'{label}: expected distance {self.A / 2} cm from the centre, measured {dist}')

        self.nHat = nHat
        self.kHat = _vcross(nHat, eHat)
        self.gearAxisPlanes = [planeA, planeB]

        frames = _gear_frames(C, eHat, nHat, self.Sigma, self.A)
        self.gearParams[0].update(frames[0])
        self.gearParams[1].update(frames[1])
        futil.log('buildAnchor: Anchor sketch and axis planes built, n̂ resolved')

    # ---- per-gear build: S07-S15 ---------------------------------------------------------------

    def buildGear(self, index):
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def _gearLabel(self, index):
        return 'Gear A' if index == 0 else 'Gear B'

    def buildSweepPaths(self, index):
        component = self.designOcc.component
        axisPlane = self.gearAxisPlanes[index]
        label = self._gearLabel(index)
        sketch: adsk.fusion.Sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Paths'

        stations = {
            'collar-': (-self.cageRadius - self.collarHalf, -self.cageRadius + self.collarHalf),
            'bore-': (-self.cageRadius - self.collarHalf - 0.1,
                      -self.cageRadius + self.collarHalf + 0.1),
            'collar+': (self.cageRadius - self.collarHalf, self.cageRadius + self.collarHalf),
            'bore+': (self.cageRadius - self.collarHalf - 0.1,
                      self.cageRadius + self.collarHalf + 0.1),
        }

        gp = self.gearParams[index]

        def stationPoint(sk: adsk.fusion.Sketch, s):
            world = _vadd(gp['origin'], _vscale(gp['dir'], s))
            local = sk.modelToSketchSpace(_point3d(world))
            local.z = 0
            return sk.sketchPoints.add(local)

        lines = {}
        fixedPoints = []
        for key in ('collar-', 'bore-', 'collar+', 'bore+'):
            sFrom, sTo = stations[key]
            fromPoint = stationPoint(sketch, sFrom)
            toPoint = stationPoint(sketch, sTo)
            fixedPoints.append(fromPoint)
            fixedPoints.append(toPoint)
            lines[key] = sketch.sketchCurves.sketchLines.addByTwoPoints(fromPoint, toPoint)

        for pt in fixedPoints:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')

        self.pathLines[index] = lines
        futil.log(f'buildSweepPaths: {sketch.name} built')

    def _buildCellSections(self, sketch, index, s0, teeth):
        # S08 / S13: teeth*n + 1 sections, each the rectangle uB..Utooth(s), -T/2..T/2, turned by
        # theta(g, s). Corners are kept off-plane ([PB-3D-SKETCH-SECTIONS], [SCREW-F-CELL-LOFT]),
        # the one exception to [PB-SKETCH-ZERO-Z].
        n = self.n
        P = self.P
        W = self.W
        T = self.T
        sections = []
        for k in range(teeth * n + 1):
            s = s0 + k * P / n
            uF = self._uTooth(index, s)
            uB = -W / 2
            corners = []
            for (u, v) in ((uB, -T / 2), (uF, -T / 2), (uF, T / 2), (uB, T / 2)):
                world = self._point(index, s, u, v)
                local = sketch.modelToSketchSpace(_point3d(world))
                corners.append(sketch.sketchPoints.add(local))
            lines = []
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                lines.append(sketch.sketchCurves.sketchLines.addByTwoPoints(corners[a], corners[b]))
            sections.append((corners, lines))
        for corners, _lines in sections:
            for pt in corners:
                pt.isFixed = True
        return sections

    def _loftCell(self, component, sections, label):
        loftFeatures = component.features.loftFeatures
        loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for _corners, lines in sections:
            collection = adsk.core.ObjectCollection.create()
            for line in lines:
                collection.add(line)
            path = component.features.createPath(collection, False)
            loftInput.loftSections.add(path)
        loft = loftFeatures.add(loftInput)
        if loft.bodies.count != 1 or not loft.bodies.item(0).isSolid:
            raise Exception(f'{label} loft: expected 1 solid body, got {loft.bodies.count}')
        return loft.bodies.item(0)

    def buildToothCell(self, index):
        component = self.designOcc.component
        axisPlane = self.gearAxisPlanes[index]
        gp = self.gearParams[index]
        label = self._gearLabel(index)

        sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Cell Sections'
        sketch.isComputeDeferred = True
        sections = self._buildCellSections(sketch, index, gp['s0'], self.c)
        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        expected = self.c * self.n + 1
        if sketch.profiles.count != expected:
            raise Exception(
                f'{sketch.name}: expected {expected} profiles, found {sketch.profiles.count}')

        body = self._loftCell(component, sections, f'{label} cell')
        self.gearBodies[index] = body
        futil.log(f'buildToothCell: {label} cell built ({expected} sections)')

    def _copyBody(self, component, body, label):
        copyFeature = component.features.copyPasteBodies.add(body)
        if copyFeature.bodies.count != 1:
            raise Exception(f'{label}: copy produced {copyFeature.bodies.count} bodies, expected 1')
        return copyFeature.bodies.item(0)

    def _screwMove(self, component, body, index, teeth, label):
        gp = self.gearParams[index]
        P = self.P
        Lambda = self.Lambda
        k = teeth
        dirVec = _vector3d(gp['dir'])
        axisPoint = _point3d(gp['origin'])

        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * P / Lambda, dirVec, axisPoint)

        shift: adsk.core.Vector3D = dirVec.copy()
        shift.scaleBy(k * P)
        mov = adsk.core.Matrix3D.create()
        mov.translation = shift

        rot.transformBy(mov)

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = component.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        component.features.moveFeatures.add(moveInput)
        return body

    def _joinRibbon(self, component, body, movedCopy, label):
        tools = adsk.core.ObjectCollection.create()
        tools.add(movedCopy)
        combineFeatures = component.features.combineFeatures
        combineInput = combineFeatures.createInput(body, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combine = combineFeatures.add(combineInput)
        if combine.bodies.count != 1:
            raise Exception(f'{label}: join produced {combine.bodies.count} bodies, expected 1')
        return combine.bodies.item(0)

    def _buildRemainder(self, component, index, gp):
        axisPlane = self.gearAxisPlanes[index]
        label = self._gearLabel(index)
        s0 = gp['s0'] + self.q * self.c * self.P
        sketch = component.sketches.add(axisPlane)
        sketch.name = f'{label} Cell Remainder'
        sketch.isComputeDeferred = True
        sections = self._buildCellSections(sketch, index, s0, self.r)
        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        expected = self.r * self.n + 1
        if sketch.profiles.count != expected:
            raise Exception(
                f'{sketch.name}: expected {expected} profiles, found {sketch.profiles.count}')
        return self._loftCell(component, sections, f'{label} remainder')

    def repeatCellByDoubling(self, index):
        component = self.designOcc.component
        gp = self.gearParams[index]
        label = self._gearLabel(index)
        body = self.gearBodies[index]
        c = self.c
        q = self.q
        r = self.r

        m = 1
        asides = []
        bit = 0
        while (1 << (bit + 1)) <= q:
            if q & (1 << bit):
                asideBody = self._copyBody(component, body, f'{label} round {bit} aside')
                asides.append((m, asideBody))
            movedCopy = self._copyBody(component, body, f'{label} round {bit} copy')
            movedCopy = self._screwMove(component, movedCopy, index, m * c, f'{label} round {bit}')
            body = self._joinRibbon(component, body, movedCopy, f'{label} round {bit}')
            m *= 2
            bit += 1

        for capturedM, asideBody in reversed(asides):
            moved = self._screwMove(component, asideBody, index, m * c, f'{label} aside m={capturedM}')
            body = self._joinRibbon(component, body, moved, f'{label} aside m={capturedM}')
            m += capturedM

        if r > 0:
            remainderBody = self._buildRemainder(component, index, gp)
            body = self._joinRibbon(component, body, remainderBody, f'{label} remainder')

        body.name = label
        self.gearBodies[index] = body
        futil.log(f'repeatCellByDoubling: {label} ribbon complete')

    # ---- S16-S34: the cage --------------------------------------------------------------------

    def buildCage(self):
        component = self.designOcc.component
        constructionPlanes = component.constructionPlanes

        ringPlaneInput = constructionPlanes.createInput()
        ringPlaneInput.setByAngle(
            self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)
        ringPlane = constructionPlanes.add(ringPlaneInput)
        ringPlane.name = 'Ring Plane'

        self._buildRing(component, ringPlane)
        self._buildRods(component)
        self._buildLoop(component)
        self._buildCollarsAndBores(component)
        futil.log('buildCage: ring, rods, loop, collars and bores complete')

    def _buildRing(self, component, ringPlane):
        C = self.C
        nHat = self.nHat
        eHat = self.eHat
        cageRise = self.cageRise
        ringRadius = self.ringRadius
        ringWire = self.ringWire

        sketch = component.sketches.add(ringPlane)
        sketch.name = 'Ring'

        apex1 = C
        apex2 = _vadd(C, _vscale(nHat, cageRise))
        local1 = sketch.modelToSketchSpace(_point3d(apex1))
        local1.z = 0
        local2 = sketch.modelToSketchSpace(_point3d(apex2))
        local2.z = 0
        pt1 = sketch.sketchPoints.add(local1)
        pt2 = sketch.sketchPoints.add(local2)
        axisLine = sketch.sketchCurves.sketchLines.addByTwoPoints(pt1, pt2)
        axisLine.isConstruction = True
        pt1.isFixed = True
        pt2.isFixed = True

        centreWorld = _vadd(_vadd(C, _vscale(nHat, cageRise)), _vscale(eHat, ringRadius))
        centreLocal = sketch.modelToSketchSpace(_point3d(centreWorld))
        centreLocal.z = 0
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centreLocal, ringWire / 2)
        circle.centerSketchPoint.isFixed = True

        textWorld = _vadd(centreWorld, _vscale(eHat, ringWire / 2))
        textLocal = sketch.modelToSketchSpace(_point3d(textWorld))
        textLocal.z = 0
        dim = sketch.sketchDimensions.addDiameterDimension(circle, textLocal)
        dim.parameter.value = ringWire

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(f'{sketch.name}: expected 1 profile, found {sketch.profiles.count}')
        profile = sketch.profiles.item(0)

        revolveFeatures = component.features.revolveFeatures
        revolveInput = revolveFeatures.createInput(
            profile, axisLine, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        ring = revolveFeatures.add(revolveInput)
        if ring.bodies.count != 1:
            raise Exception(f'Ring revolve: expected 1 body, got {ring.bodies.count}')
        self.cageBody = ring.bodies.item(0)

    def _buildRods(self, component):
        C = self.C
        eHat = self.eHat
        kHat = self.kHat
        rodDiameter = self.rodDiameter
        ringRadius = self.ringRadius

        sketch = component.sketches.add(self.plane)
        sketch.name = 'Rods'

        for psi in self.rodAzimuths:
            footWorld = _vadd(C, _vadd(
                _vscale(eHat, ringRadius * math.cos(psi)),
                _vscale(kHat, ringRadius * math.sin(psi))))
            footLocal = sketch.modelToSketchSpace(_point3d(footWorld))
            footLocal.z = 0
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(footLocal, rodDiameter / 2)
            circle.centerSketchPoint.isFixed = True
            textWorld = _vadd(footWorld, _vscale(eHat, rodDiameter / 2))
            textLocal = sketch.modelToSketchSpace(_point3d(textWorld))
            textLocal.z = 0
            dim = sketch.sketchDimensions.addDiameterDimension(circle, textLocal)
            dim.parameter.value = rodDiameter

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        if sketch.profiles.count != 4:
            raise Exception(f'{sketch.name}: expected 4 profiles, found {sketch.profiles.count}')

        self._extrudeAndJoinRods(component, sketch)

    def _extrudeAndJoinRods(self, component, sketch):
        profiles = adsk.core.ObjectCollection.create()
        for i in range(sketch.profiles.count):
            profiles.add(sketch.profiles.item(i))

        extrudeFeatures = component.features.extrudeFeatures
        extrudeInput = extrudeFeatures.createInput(
            profiles, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(self.cageRise), False)
        rods = extrudeFeatures.add(extrudeInput)
        if rods.bodies.count != 4:
            raise Exception(f'Rods extrude: expected 4 bodies, got {rods.bodies.count}')

        tools = adsk.core.ObjectCollection.create()
        for i in range(rods.bodies.count):
            tools.add(rods.bodies.item(i))
        combineFeatures = component.features.combineFeatures
        combineInput = combineFeatures.createInput(self.cageBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combine = combineFeatures.add(combineInput)
        if combine.bodies.count != 1:
            raise Exception(f'rods join: expected 1 body, got {combine.bodies.count}')
        self.cageBody = combine.bodies.item(0)

    def _buildLoop(self, component):
        constructionPlanes = component.constructionPlanes
        loopPlaneInput = constructionPlanes.createInput()
        loopPlaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-self.cageRise))
        loopPlane = constructionPlanes.add(loopPlaneInput)
        loopPlane.name = 'Loop Plane'

        loopCentre = _vsub(self.C, _vscale(self.nHat, self.cageRise))
        feet = []
        for psi in self.footPsis:
            foot = _vadd(loopCentre, _vadd(
                _vscale(self.eHat, self.ringRadius * math.cos(psi)),
                _vscale(self.kHat, self.ringRadius * math.sin(psi))))
            feet.append(foot)

        barBodies = []
        for i in range(4):
            j = (i + 1) % 4
            barBodies.append(
                self._buildLoopBar(component, loopPlane, i, feet[i], feet[j], loopCentre))

        ballBodies = []
        for i in range(4):
            ballBodies.append(self._buildLoopBall(component, loopPlane, i, feet[i]))

        self._joinLoop(component, barBodies, ballBodies)

    def _buildLoopBar(self, component, loopPlane, i, footI, footJ, loopCentre):
        d = _vnorm(_vsub(footJ, footI))
        mid = _vscale(_vadd(footI, footJ), 0.5)
        toCentreVec = _vsub(mid, loopCentre)
        perp = _vsub(toCentreVec, _vscale(d, _vdot(toCentreVec, d)))
        mHat = _vnorm(perp)
        w = self.ringWire / 2

        sketch = component.sketches.add(loopPlane)
        sketch.name = f'Loop Bar {i}'

        cornersWorld = [footI, footJ, _vadd(footJ, _vscale(mHat, w)), _vadd(footI, _vscale(mHat, w))]
        points = []
        for world in cornersWorld:
            local = sketch.modelToSketchSpace(_point3d(world))
            local.z = 0
            points.append(sketch.sketchPoints.add(local))

        lines = []
        for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
            lines.append(sketch.sketchCurves.sketchLines.addByTwoPoints(points[a], points[b]))

        for pt in points:
            pt.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        if sketch.profiles.count != 1:
            raise Exception(f'{sketch.name}: expected 1 profile, found {sketch.profiles.count}')
        profile = sketch.profiles.item(0)

        revolveFeatures = component.features.revolveFeatures
        revolveInput = revolveFeatures.createInput(
            profile, lines[0], adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolveFeatures.add(revolveInput)
        if revolve.bodies.count != 1:
            raise Exception(f'{sketch.name} revolve: expected 1 body, got {revolve.bodies.count}')
        return revolve.bodies.item(0)

    def _buildLoopBall(self, component, loopPlane, i, foot):
        w = self.ringWire / 2
        eHat = self.eHat
        kHat = self.kHat

        sketch = component.sketches.add(loopPlane)
        sketch.name = f'Loop Ball {i}'

        startWorld = _vsub(foot, _vscale(eHat, w))
        endWorld = _vadd(foot, _vscale(eHat, w))
        throughWorld = _vadd(foot, _vscale(kHat, w))

        startLocal = sketch.modelToSketchSpace(_point3d(startWorld))
        startLocal.z = 0
        endLocal = sketch.modelToSketchSpace(_point3d(endWorld))
        endLocal.z = 0
        throughLocal = sketch.modelToSketchSpace(_point3d(throughWorld))
        throughLocal.z = 0

        startPoint = sketch.sketchPoints.add(startLocal)
        endPoint = sketch.sketchPoints.add(endLocal)
        bl = sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)

        arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            bl.startSketchPoint, throughLocal, bl.endSketchPoint)

        bl.startSketchPoint.isFixed = True
        bl.endSketchPoint.isFixed = True
        sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, bl)

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')
        profile = find_profile_by_curve_counts(sketch, lines=1, arcs=1)

        revolveFeatures = component.features.revolveFeatures
        revolveInput = revolveFeatures.createInput(
            profile, bl, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolveFeatures.add(revolveInput)
        if revolve.bodies.count != 1:
            raise Exception(f'{sketch.name} revolve: expected 1 body, got {revolve.bodies.count}')
        return revolve.bodies.item(0)

    def _joinLoop(self, component, barBodies, ballBodies):
        tools = adsk.core.ObjectCollection.create()
        for body in barBodies:
            tools.add(body)
        for body in ballBodies:
            tools.add(body)
        combineFeatures = component.features.combineFeatures
        combineInput = combineFeatures.createInput(self.cageBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combine = combineFeatures.add(combineInput)
        if combine.bodies.count != 1:
            raise Exception(f'loop join: expected 1 body, got {combine.bodies.count}')
        self.cageBody = combine.bodies.item(0)

    # ---- S28-S34: collars and bores ------------------------------------------------------------

    def _buildDistanceOnPathPlane(self, component, line, name):
        constructionPlanes = component.constructionPlanes
        planeInput = constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))
        plane = constructionPlanes.add(planeInput)
        plane.name = name
        return plane

    def _buildRectangleScheme(self, component, plane, g, sc, outline, label):
        collarHalf = self.collarHalf
        st = (sc - collarHalf) if outline else (sc - collarHalf - 0.1)

        gp = self.gearParams[g]
        theta = self._theta(g, st)
        uB = -(self.W / 2 + self.clearance)
        uF = self.W / 2 + self.clearance
        hv = self.T / 2 + self.clearance
        w = self.collarWall

        sketch: adsk.fusion.Sketch = component.sketches.add(plane)
        sketch.name = label
        sketch.isComputeDeferred = True

        def worldAt(u, v):
            return self._point(g, st, u, v)

        def toLocal(sk: adsk.fusion.Sketch, world):
            local = sk.modelToSketchSpace(_point3d(world))
            local.z = 0
            return local

        def at(sk: adsk.fusion.Sketch, u, v):
            return sk.sketchPoints.add(toLocal(sk, worldAt(u, v)))

        def midWorld(uv1, uv2):
            return _vscale(_vadd(worldAt(*uv1), worldAt(*uv2)), 0.5)

        def textPointFor(sk: adsk.fusion.Sketch, pairA, pairB):
            m1 = midWorld(*pairA)
            m2 = midWorld(*pairB)
            return toLocal(sk, _vscale(_vadd(m1, m2), 0.5))

        # References: O is unrotated (raw axis point); Cp is O offset along the UNROTATED u_g,
        # never through point()/theta ([SCREW-F-REFERENCES]).
        O = worldAt(0, 0)
        Cp = _vadd(O, _vscale(gp['u'], self.A / 2))
        oPoint = sketch.sketchPoints.add(toLocal(sketch, O))
        cpPoint = sketch.sketchPoints.add(toLocal(sketch, Cp))
        ru = sketch.sketchCurves.sketchLines.addByTwoPoints(oPoint, cpPoint)
        ru.isConstruction = True

        # Spine.
        ePoint = at(sketch, uF, 0)
        k = sketch.sketchCurves.sketchLines.addByTwoPoints(oPoint, ePoint)
        k.isConstruction = True

        oPoint.isFixed = True
        cpPoint.isFixed = True

        # Rectangle.
        c0, c1, c2, c3 = (at(sketch, uB, -hv), at(sketch, uF, -hv),
                          at(sketch, uF, hv), at(sketch, uB, hv))
        l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(c0, c1)
        l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(c1, c2)
        l3 = sketch.sketchCurves.sketchLines.addByTwoPoints(c2, c3)
        l4 = sketch.sketchCurves.sketchLines.addByTwoPoints(c3, c0)
        if outline:
            for line in (l1, l2, l3, l4):
                line.isConstruction = True

        kPair = ((0.0, 0.0), (uF, 0.0))
        l1Pair = ((uB, -hv), (uF, -hv))
        l3Pair = ((uF, hv), (uB, hv))
        l2Pair = ((uF, -hv), (uF, hv))
        l4Pair = ((uB, hv), (uB, -hv))

        # Rows.
        dimK = sketch.sketchDimensions.addDistanceDimension(
            oPoint, ePoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            toLocal(sketch, midWorld(*kPair)))
        dimK.parameter.value = uF

        sketch.geometricConstraints.addParallel(l1, k)
        dim1 = sketch.sketchDimensions.addOffsetDimension(k, l1, textPointFor(sketch, kPair, l1Pair))
        dim1.parameter.value = hv

        sketch.geometricConstraints.addParallel(l3, k)
        dim3 = sketch.sketchDimensions.addOffsetDimension(k, l3, textPointFor(sketch, kPair, l3Pair))
        dim3.parameter.value = hv

        sketch.geometricConstraints.addCoincident(ePoint, l2)
        sketch.geometricConstraints.addPerpendicular(l2, k)

        sketch.geometricConstraints.addParallel(l4, l2)
        dim4 = sketch.sketchDimensions.addOffsetDimension(l2, l4, textPointFor(sketch, l2Pair, l4Pair))
        dim4.parameter.value = uF - uB

        # Angle ([PB-ANGULAR-DIM]).
        uHat, vHat = gp['u'], gp['v']
        if abs(math.sin(theta)) >= math.sqrt(0.5):
            value = math.acos(math.cos(theta))
            dirOE = _vadd(_vscale(uHat, math.cos(theta)), _vscale(vHat, math.sin(theta)))
            bisector = _vnorm(_vadd(uHat, dirOE))
            textWorld = _vadd(O, _vscale(bisector, uF / 2))
            angleDim = sketch.sketchDimensions.addAngularDimension(ru, k, toLocal(sketch, textWorld))
            angleDim.parameter.value = value
        else:
            value = math.acos(-math.sin(theta))
            X = _vadd(O, _vscale(uHat, uF / math.cos(theta)))
            dirL2 = _vadd(_vscale(uHat, -math.sin(theta)), _vscale(vHat, math.cos(theta)))
            bisector = _vnorm(_vadd(uHat, dirL2))
            textWorld = _vadd(X, _vscale(bisector, uF / 2))
            angleDim = sketch.sketchDimensions.addAngularDimension(ru, l2, toLocal(sketch, textWorld))
            angleDim.parameter.value = value

        if outline:
            o1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                at(sketch, uB, -hv - w), at(sketch, uF, -hv - w))
            o2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                at(sketch, uF + w, -hv), at(sketch, uF + w, hv))
            o3 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                at(sketch, uF, hv + w), at(sketch, uB, hv + w))
            o4 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                at(sketch, uB - w, hv), at(sketch, uB - w, -hv))
            outlineLines = [o1, o2, o3, o4]
            rect = [l1, l2, l3, l4]

            diag = w / math.sqrt(2)
            cornerPush = [
                (uF + diag, -hv - diag), (uF + diag, hv + diag),
                (uB - diag, hv + diag), (uB - diag, -hv - diag),
            ]
            arcs = []
            for j in range(4):
                centreSeed = toLocal(sketch, worldAt(*cornerPush[j]))
                nextLine = outlineLines[(j + 1) % 4]
                arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
                    outlineLines[j].endSketchPoint, centreSeed, nextLine.startSketchPoint)
                arcs.append(arc)

            liPairs = {0: l1Pair, 1: l2Pair, 2: l3Pair, 3: l4Pair}
            oiPairs = {
                0: ((uB, -hv - w), (uF, -hv - w)),
                1: ((uF + w, -hv), (uF + w, hv)),
                2: ((uF, hv + w), (uB, hv + w)),
                3: ((uB - w, hv), (uB - w, -hv)),
            }
            for j in range(4):
                sketch.geometricConstraints.addParallel(outlineLines[j], rect[j])
                dimOff = sketch.sketchDimensions.addOffsetDimension(
                    rect[j], outlineLines[j], textPointFor(sketch, liPairs[j], oiPairs[j]))
                dimOff.parameter.value = w
                prevRect = rect[(j + 3) % 4]
                nextRect = rect[(j + 1) % 4]
                sketch.geometricConstraints.addCoincident(outlineLines[j].startSketchPoint, prevRect)
                sketch.geometricConstraints.addCoincident(outlineLines[j].endSketchPoint, nextRect)
                sketch.geometricConstraints.addTangent(outlineLines[j], arcs[j])

        sketch.isComputeDeferred = False
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name} sketch is not fully constrained')

        if outline:
            profile = find_profile_by_curve_counts(sketch, lines=4, arcs=4)
        else:
            profile = find_profile_by_curve_counts(sketch, lines=4)
        return profile

    def _checkCollarSense(self, body, label, sc, g):
        collarHalf = self.collarHalf
        W, T, clearance, w = self.W, self.T, self.clearance, self.collarWall
        uB = -(W / 2 + clearance)
        uF = W / 2 + clearance
        hv = T / 2 + clearance
        outline = [(uB, -hv - w), (uF, -hv - w), (uF + w, -hv), (uF + w, hv),
                   (uF, hv + w), (uB, hv + w), (uB - w, hv), (uB - w, -hv)]
        vertexWorlds = [
            _from_point3d(body.vertices.item(i).geometry) for i in range(body.vertices.count)]

        for stationLabel, station in (('near', sc - collarHalf), ('far', sc + collarHalf)):
            for (u, v) in outline:
                target = self._point(g, station, u, v)
                best = min(_vlen(_vsub(target, vw)) for vw in vertexWorlds)
                if best > 0.005:
                    raise Exception(
                        f'{label}: at the {stationLabel} end, point (u={u}, v={v}) has no '
                        f'vertex within 0.005 cm (worst {best} cm) ([SCREW-F-SWEEP-CHECK])')

    def _sweepCollar(self, component, profile, line, label, sc, g):
        path = component.features.createPath(line, False)
        sweepFeatures = component.features.sweepFeatures
        sweepInput = sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(2 * self.collarHalf / self.Lambda)
        sweep = sweepFeatures.add(sweepInput)
        if sweep.bodies.count != 1:
            raise Exception(f'{label} sweep: expected 1 body, got {sweep.bodies.count}')
        body = sweep.bodies.item(0)
        self._checkCollarSense(body, label, sc, g)
        return body

    def _joinCollars(self, component, collarBodies):
        tools = adsk.core.ObjectCollection.create()
        for body in collarBodies:
            tools.add(body)
        combineFeatures = component.features.combineFeatures
        combineInput = combineFeatures.createInput(self.cageBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combine = combineFeatures.add(combineInput)
        if combine.bodies.count != 1:
            raise Exception(f'collars join: expected 1 body, got {combine.bodies.count}')
        self.cageBody = combine.bodies.item(0)

    def _checkBoreOpen(self, label, sc, g):
        s = sc + self.collarHalf / 2
        theta = self._theta(g, s)
        gp = self.gearParams[g]
        uHatS = _vadd(_vscale(gp['u'], math.cos(theta)), _vscale(gp['v'], math.sin(theta)))
        base = _vadd(gp['origin'], _vscale(gp['dir'], s))
        for sign, probeLabel in ((1.0, '+'), (-1.0, '-')):
            probe = _vadd(base, _vscale(uHatS, sign * (self.W / 2 + self.clearance / 2)))
            containment = self.cageBody.pointContainment(_point3d(probe))
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise Exception(
                    f'{label}: the channel is not open at probe {probeLabel} (station {s})')

    def _cutBore(self, component, profile, line, label, sc, g):
        path = component.features.createPath(line, False)
        sweepFeatures = component.features.sweepFeatures
        sweepInput = sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(
            2 * (self.collarHalf + 0.1) / self.Lambda)
        sweepInput.participantBodies = [self.cageBody]
        sweep = sweepFeatures.add(sweepInput)
        if sweep.bodies.count != 1:
            raise Exception(f'{label} cut: expected 1 body, got {sweep.bodies.count}')
        self.cageBody = sweep.bodies.item(0)
        self._checkBoreOpen(label, sc, g)

    def _buildCollarsAndBores(self, component):
        collarLabels = [
            'Gear A Collar -R', 'Gear A Collar +R', 'Gear B Collar -R', 'Gear B Collar +R']
        collarKeys = ['collar-', 'collar+', 'collar-', 'collar+']
        boreKeys = ['bore-', 'bore+', 'bore-', 'bore+']
        gearIndex = [0, 0, 1, 1]
        scValues = [-self.cageRadius, self.cageRadius, -self.cageRadius, self.cageRadius]

        collarBodies = []
        for i in range(4):
            g = gearIndex[i]
            sc = scValues[i]
            line = self.pathLines[g][collarKeys[i]]
            label = collarLabels[i]
            plane = self._buildDistanceOnPathPlane(component, line, f'{label} Plane')
            profile = self._buildRectangleScheme(component, plane, g, sc, True, label)
            collarBodies.append(self._sweepCollar(component, profile, line, label, sc, g))

        self._joinCollars(component, collarBodies)

        for i in range(4):
            g = gearIndex[i]
            sc = scValues[i]
            line = self.pathLines[g][boreKeys[i]]
            label = collarLabels[i].replace('Collar', 'Bore')
            plane = self._buildDistanceOnPathPlane(component, line, f'{label} Plane')
            profile = self._buildRectangleScheme(component, plane, g, sc, False, label)
            self._cutBore(component, profile, line, label, sc, g)

    # ---- S35: relocate bodies and hide construction geometry -----------------------------------

    def relocateBodies(self):
        self.cageBody.name = 'Cage'
        gearBodyA: adsk.fusion.BRepBody = self.gearBodies[0]
        gearBodyB: adsk.fusion.BRepBody = self.gearBodies[1]
        gearBodyA.moveToComponent(self.gearOccs[0])
        gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody.moveToComponent(self.cageOcc)
        solids.hide_construction_geometry(self.designOcc.component)
        futil.log('relocateBodies: bodies relocated, construction geometry hidden')
