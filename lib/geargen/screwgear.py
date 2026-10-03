import math
import adsk.core, adsk.fusion
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

# The number of points each cross-section's toothed side is fitted through (S09/S10).
TOOTH_SPLINE_POINTS = 11
# The number of teeth the lofted cell holds (S01/S03). Every count derived from it is
# computed in processInputs; nothing hard-codes 4 elsewhere.
CELL_TEETH = 4


# ---------------------------------------------------------------------------
# Plain 3-component tuple vector helpers. Every world-space point and
# direction the build computes is carried as a plain (x, y, z) tuple until
# the moment it is handed to a Fusion API call, at which point it is wrapped
# with _pt3/_vec3. This keeps the geometry formulas of the spec's §1-§4 close
# to their prose form and avoids the in-place mutation semantics of
# adsk.core.Vector3D/Point3D.
# ---------------------------------------------------------------------------

def _tup(entity):
    return (entity.x, entity.y, entity.z)


def _pt3(t):
    return adsk.core.Point3D.create(t[0], t[1], t[2])


def _vec3(t):
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


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


def _turn(u, v, th):
    # (x, y) = turn(u, v, th) of the spec's §1: a 2D rotation by th.
    c = math.cos(th)
    s = math.sin(th)
    return (u * c - v * s, u * s + v * c)


def _clip(poly, a, b, c):
    # Keep the part of a convex polygon with a*x + b*y <= c (Sutherland-Hodgman,
    # S03 "A bore's section in the wall").
    out = []
    n = len(poly)
    for i in range(n):
        p = poly[i]
        q = poly[(i + 1) % n]
        fp = a * p[0] + b * p[1] - c
        fq = a * q[0] + b * q[1] - c
        if fp <= 0:
            out.append(p)
        if (fp < 0 and fq > 0) or (fp > 0 and fq < 0):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def _hull(pts):
    # Andrew's monotone chain, counter-clockwise, without collinear points
    # (S03 "The wall between the bores").
    pts = sorted(pts)
    if len(pts) < 3:
        return list(pts)

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


def _area(poly):
    # Signed area of a polygon, positive counter-clockwise.
    a = 0.0
    n = len(poly)
    for i in range(n):
        p = poly[i]
        q = poly[(i + 1) % n]
        a += p[0] * q[1] - q[0] * p[1]
    return a / 2.0


class ScrewGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs: adsk.core.CommandInputs = command.commandInputs

        planeInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', "Plane the cage's axis is normal to")
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        pointInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_POINT, 'Centre Point', 'Centre of the mechanism')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1, 1)

        parentInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Component the mechanism is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        ribbonGroup: adsk.core.GroupCommandInput = inputs.addGroupCommandInput('ribbonGroup', 'Ribbon')
        ribbonGroup.isExpanded = True
        ribbonGroup.children.addValueInput(
            INPUT_ID_RIBBON_WIDTH, 'Ribbon Width', 'mm', adsk.core.ValueInput.createByReal(1.5))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TOOTH_COUNT, 'Tooth Count', '', adsk.core.ValueInput.createByReal(68))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TWIST_LEAD, 'Twist Lead', 'mm', adsk.core.ValueInput.createByReal(4.95))
        ribbonGroup.children.addValueInput(
            INPUT_ID_RIBBON_THICKNESS, 'Ribbon Thickness', 'mm',
            adsk.core.ValueInput.createByReal(0.375))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TOOTH_PITCH, 'Tooth Pitch', 'mm', adsk.core.ValueInput.createByReal(0.2625))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TOOTH_HEIGHT, 'Tooth Height', 'mm', adsk.core.ValueInput.createByReal(0.2625))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TOOTH_SLANT, 'Tooth Slant', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(25.8)))
        ribbonGroup.children.addValueInput(
            INPUT_ID_TOOTH_BOW, 'Tooth Bow', '', adsk.core.ValueInput.createByReal(0.048))

        frameGroup: adsk.core.GroupCommandInput = inputs.addGroupCommandInput('frameGroup', 'Frame')
        frameGroup.isExpanded = True
        frameGroup.children.addValueInput(
            INPUT_ID_CAGE_RADIUS, 'Cage Radius', 'mm', adsk.core.ValueInput.createByReal(1.5))
        frameGroup.children.addValueInput(
            INPUT_ID_CAGE_RISE, 'Cage Rise', 'mm', adsk.core.ValueInput.createByReal(1.875))
        frameGroup.children.addValueInput(
            INPUT_ID_CLEARANCE, 'Clearance', 'mm', adsk.core.ValueInput.createByReal(0.02))
        frameGroup.children.addValueInput(
            INPUT_ID_ROOF_ALLOWANCE, 'Roof Allowance', 'mm', adsk.core.ValueInput.createByReal(0.03))
        frameGroup.children.addValueInput(
            INPUT_ID_COLLAR_HALF, 'Collar Half Length', 'mm', adsk.core.ValueInput.createByReal(0.3))
        frameGroup.children.addValueInput(
            INPUT_ID_COLLAR_WALL, 'Collar Wall', 'mm', adsk.core.ValueInput.createByReal(0.3))

        meshGroup: adsk.core.GroupCommandInput = inputs.addGroupCommandInput(
            'meshGroup', 'Mesh (from the mesh search)')
        meshGroup.isExpanded = False
        meshGroup.children.addValueInput(
            INPUT_ID_CROSS_ANGLE, 'Crossing Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(80)))
        meshGroup.children.addValueInput(
            INPUT_ID_ENGAGEMENT, 'Engagement', 'mm', adsk.core.ValueInput.createByReal(0.105))
        meshGroup.children.addValueInput(
            INPUT_ID_MOUNT_ANGLE_A, 'Mounting Angle A', 'deg', adsk.core.ValueInput.createByReal(0))
        meshGroup.children.addValueInput(
            INPUT_ID_MOUNT_ANGLE_B, 'Mounting Angle B', 'deg', adsk.core.ValueInput.createByReal(0))
        meshGroup.children.addValueInput(
            INPUT_ID_ASSEMBLY_PHASE, 'Assembly Phase', 'mm', adsk.core.ValueInput.createByReal(-0.131))


class ScrewGearGenerator(Generator):

    def prefixBase(self):
        return 'ScrewGear'

    def generate(self, inputs):
        self.processInputs(inputs)
        self.buildComponentTree()
        self.buildAnchor()
        for g in (0, 1):
            self.buildGear(g)
        self.buildCage()
        self.relocateBodies()

    # -----------------------------------------------------------------
    # S03: processInputs. Reads the selections and dialog inputs, runs the
    # range checks, precomputes the derived values, runs the two searches
    # (the four bores and the windows) and stores every handle on self.
    # No generation context class: every handle is carried on self (S01).
    # -----------------------------------------------------------------

    def processInputs(self, inputs: adsk.core.CommandInputs):
        self.pathLines = [{}, {}]
        self.gearBodies = [adsk.fusion.BRepBody.cast(None), adsk.fusion.BRepBody.cast(None)]

        design: adsk.fusion.Design = get_design()
        unitsManager: adsk.core.UnitsManager = design.unitsManager

        selections = {}
        for sid in (INPUT_ID_PLANE, INPUT_ID_POINT, INPUT_ID_PARENT):
            selectionInput: adsk.core.SelectionCommandInput = adsk.core.SelectionCommandInput.cast(
                inputs.itemById(sid))
            if selectionInput is None:
                raise Exception(f'ScrewGear: missing selection input "{sid}"')
            selections[sid] = selectionInput.selection(0).entity

        parent = selections[INPUT_ID_PARENT]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent
        self.targetPlane = selections[INPUT_ID_PLANE]
        self.centrePoint = selections[INPUT_ID_POINT]

        def readValue(inputId: str, units: str,
                      inputs: adsk.core.CommandInputs = inputs,
                      unitsManager: adsk.core.UnitsManager = unitsManager) -> float:
            # inputs/unitsManager are re-declared as defaulted parameters because a
            # nested closure does not see the enclosing function's annotations.
            valueInput: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(
                inputs.itemById(inputId))
            if valueInput is None:
                raise Exception(f'ScrewGear: missing input "{inputId}"')
            return unitsManager.evaluateExpression(valueInput.expression, units)

        W = readValue(INPUT_ID_RIBBON_WIDTH, 'mm')
        N_raw = readValue(INPUT_ID_TOOTH_COUNT, '')
        TwistLead = readValue(INPUT_ID_TWIST_LEAD, 'mm')
        T = readValue(INPUT_ID_RIBBON_THICKNESS, 'mm')
        P = readValue(INPUT_ID_TOOTH_PITCH, 'mm')
        H = readValue(INPUT_ID_TOOTH_HEIGHT, 'mm')
        Slant = readValue(INPUT_ID_TOOTH_SLANT, 'deg')
        Bow = readValue(INPUT_ID_TOOTH_BOW, '')
        cageRadius = readValue(INPUT_ID_CAGE_RADIUS, 'mm')
        cageRise = readValue(INPUT_ID_CAGE_RISE, 'mm')
        clearance = readValue(INPUT_ID_CLEARANCE, 'mm')
        roofAllowance = readValue(INPUT_ID_ROOF_ALLOWANCE, 'mm')
        collarHalf = readValue(INPUT_ID_COLLAR_HALF, 'mm')
        collarWall = readValue(INPUT_ID_COLLAR_WALL, 'mm')
        Sigma = readValue(INPUT_ID_CROSS_ANGLE, 'deg')
        Engagement = readValue(INPUT_ID_ENGAGEMENT, 'mm')
        PhiA = readValue(INPUT_ID_MOUNT_ANGLE_A, 'deg')
        PhiB = readValue(INPUT_ID_MOUNT_ANGLE_B, 'deg')
        assemblyPhase = readValue(INPUT_ID_ASSEMBLY_PHASE, 'mm')

        def need(cond, inputId, message):
            if not cond:
                raise Exception(f'ScrewGear: "{inputId}" {message}')

        need(W > 0, INPUT_ID_RIBBON_WIDTH, f'must be > 0 mm (got {W * 10:.4f} mm)')
        need(T > 0, INPUT_ID_RIBBON_THICKNESS, f'must be > 0 mm (got {T * 10:.4f} mm)')
        need(P > 0, INPUT_ID_TOOTH_PITCH, f'must be > 0 mm (got {P * 10:.4f} mm)')
        need(TwistLead > 0, INPUT_ID_TWIST_LEAD, f'must be > 0 mm (got {TwistLead * 10:.4f} mm)')
        need(collarHalf > 0, INPUT_ID_COLLAR_HALF, f'must be > 0 mm (got {collarHalf * 10:.4f} mm)')
        need(collarWall > 0, INPUT_ID_COLLAR_WALL, f'must be > 0 mm (got {collarWall * 10:.4f} mm)')
        need(clearance > 0, INPUT_ID_CLEARANCE, f'must be > 0 mm (got {clearance * 10:.4f} mm)')
        need(roofAllowance >= 0, INPUT_ID_ROOF_ALLOWANCE,
             f'must be >= 0 mm (got {roofAllowance * 10:.4f} mm)')

        if N_raw != round(N_raw) or N_raw < 4:
            raise Exception(
                f'ScrewGear: "{INPUT_ID_TOOTH_COUNT}" must be a whole number >= 4 (got {N_raw})')
        N = int(round(N_raw))

        need(H > 0 and H < W / 2, INPUT_ID_TOOTH_HEIGHT,
             f'must be > 0 and < ribbonWidth/2 (got {H * 10:.4f} mm, limit {W * 10 / 2:.4f} mm)')

        SlantDeg = math.degrees(Slant)
        need(-90 < SlantDeg < 90, INPUT_ID_TOOTH_SLANT,
             f'must lie strictly between -90 and 90 degrees (got {SlantDeg:.4f})')

        need(Bow >= 0, INPUT_ID_TOOTH_BOW, f'must be >= 0 (got {Bow})')
        Bow_cm = 10.0 * Bow
        need(H + Bow_cm * (T / 2) ** 2 < W / 2, INPUT_ID_TOOTH_BOW,
             f'the tooth edge would reach past the ribbon half-width '
             f'(H + toothBow*(T/2)^2 = {(H + Bow_cm * (T / 2) ** 2) * 10:.4f} mm, '
             f'limit {W * 10 / 2:.4f} mm)')

        need(Engagement > 0 and Engagement <= H, INPUT_ID_ENGAGEMENT,
             f'must be > 0 and <= toothHeight (got {Engagement * 10:.4f} mm, limit {H * 10:.4f} mm)')

        SigmaDeg = math.degrees(Sigma)
        need(0 < SigmaDeg < 180, INPUT_ID_CROSS_ANGLE,
             f'must lie strictly between 0 and 180 degrees (got {SigmaDeg:.4f})')

        need(-P < assemblyPhase < P, INPUT_ID_ASSEMBLY_PHASE,
             f'must lie strictly within +/- toothPitch '
             f'(got {assemblyPhase * 10:.4f} mm, limit {P * 10:.4f} mm)')

        need(cageRadius + collarHalf + 0.1 < N * P / 2, INPUT_ID_CAGE_RADIUS,
             f'leaves no cut margin (cageRadius + collarHalf + 1 mm = '
             f'{(cageRadius + collarHalf + 0.1) * 10:.4f} mm, '
             f'must be < toothCount*toothPitch/2 = {(N * P / 2) * 10:.4f} mm)')

        # Derived values, in the build's cm and radians.
        Lambda = TwistLead / (2 * math.pi)
        tanSlant = math.tan(Slant)
        A = W - Engagement
        Ri = cageRadius - collarHalf
        Ro = cageRadius + collarHalf
        hw = W / 2 + clearance
        ht = T / 2 + clearance
        cornerDist = math.hypot(hw, ht + roofAllowance)
        sIn = math.sqrt(max(0.0, Ri * Ri - cornerDist * cornerDist)) - 0.1
        sOut = Ro + 0.1
        axialWindow = 1.5 * math.sqrt(W * W - A * A) / math.sin(Sigma)
        cell = min(CELL_TEETH, N)
        steps = max(math.ceil((P / Lambda) / math.radians(2) - 1e-12), 8)
        wholeCells = N // cell
        remainderTeeth = N % cell
        Z0 = [0.0, assemblyPhase]
        Phi = [PhiA, PhiB]

        # The sleeve's four checks, 1-3 (the fourth, the wall between
        # neighbouring bores, needs the search below).
        need(math.hypot(cornerDist, 0.1) < Ri, INPUT_ID_CAGE_RADIUS,
             f'the channel does not start in the hollow '
             f'(hypot(cornerDepth, 1mm) = {math.hypot(cornerDist, 0.1) * 10:.4f} mm, '
             f'must be < sleeve inner radius {Ri * 10:.4f} mm)')
        need(math.hypot(axialWindow, math.hypot(W / 2, T / 2)) + clearance <= Ri,
             INPUT_ID_CAGE_RADIUS,
             f'the mesh is not visible along the axis '
             f'({(math.hypot(axialWindow, math.hypot(W / 2, T / 2)) + clearance) * 10:.4f} mm, '
             f'must be <= sleeve inner radius {Ri * 10:.4f} mm)')
        need(cageRise >= A / 2 + cornerDist + collarWall, INPUT_ID_CAGE_RISE,
             f'the end faces do not keep collarWall '
             f'({cageRise * 10:.4f} mm, must be >= {(A / 2 + cornerDist + collarWall) * 10:.4f} mm)')

        # The frame of the two searches: an abstract frame, C at the origin,
        # ê, k̂ and n̂ the X, Y and Z axes, every length in millimetres.
        Hw_mm, Ht_mm = hw * 10, ht * 10
        Ri_mm, Ro_mm = Ri * 10, Ro * 10
        CageRadius_mm, CollarHalf_mm, CollarWall_mm = cageRadius * 10, collarHalf * 10, collarWall * 10
        SIn_mm, SOut_mm = sIn * 10, sOut * 10
        Lambda_mm, Roof_mm, A_mm = Lambda * 10, roofAllowance * 10, A * 10
        CageRise_mm = cageRise * 10

        sigmaHalf = Sigma / 2.0
        dirVecsMM = [
            (math.cos(sigmaHalf), math.sin(sigmaHalf), 0.0),
            (math.cos(sigmaHalf), -math.sin(sigmaHalf), 0.0),
        ]
        originVecsMM = [(0.0, 0.0, -A_mm / 2.0), (0.0, 0.0, A_mm / 2.0)]
        uHatVecsMM = [(0.0, 0.0, 1.0), (0.0, 0.0, -1.0)]
        vHatVecsMM = [_vcross(dirVecsMM[g], uHatVecsMM[g]) for g in (0, 1)]

        def theta_mm(g, s):
            return s / Lambda_mm + Phi[g]

        def local_mm(g, q):
            o = originVecsMM[g]
            rel = (q[0] - o[0], q[1] - o[1], q[2] - o[2])
            return (_vdot(rel, uHatVecsMM[g]), _vdot(rel, vHatVecsMM[g]), _vdot(rel, dirVecsMM[g]))

        def tilt_mm(g, sigma):
            t1 = theta_mm(g, sigma * (CageRadius_mm - CollarHalf_mm))
            t2 = theta_mm(g, sigma * (CageRadius_mm + CollarHalf_mm))
            lo, hi = min(t1, t2), max(t1, t2)
            k = math.ceil((lo - math.pi / 2) / math.pi)
            if math.pi / 2 + k * math.pi <= hi:
                return 0.0
            return min(abs(math.cos(t1)), abs(math.cos(t2)))

        # The four bores, in S03's fixed cut order: gear A -R, gear A +R,
        # gear B -R, gear B +R.
        bores = []
        boreNames = []
        for g in (0, 1):
            gearLabel = 'Gear A' if g == 0 else 'Gear B'
            levelSigma = -1.0
            if tilt_mm(g, 1.0) < tilt_mm(g, -1.0):
                levelSigma = 1.0
            for sigma in (-1.0, 1.0):
                vLo, vHi = -Ht_mm, Ht_mm
                level = (sigma == levelSigma)
                if level:
                    sc = sigma * CageRadius_mm
                    up = -math.sin(theta_mm(g, sc)) * uHatVecsMM[g][2]
                    if up > 0:
                        vHi = Ht_mm + Roof_mm
                    else:
                        vLo = -Ht_mm - Roof_mm
                if sigma < 0:
                    spanLo, spanHi = -SOut_mm, -SIn_mm
                else:
                    spanLo, spanHi = SIn_mm, SOut_mm
                bores.append({
                    'gear': g, 'sigma': sigma, 'vLo': vLo, 'vHi': vHi,
                    'spanLo': spanLo, 'spanHi': spanHi,
                })
                boreNames.append(f"{gearLabel} {'-R' if sigma < 0 else '+R'}")

        def section_in_wall_mm(bore, s):
            if abs(s) >= Ro_mm:
                return [[], []]
            g = bore['gear']
            th = theta_mm(g, s)
            poly = []
            for (cu, cv) in ((-Hw_mm, bore['vLo']), (Hw_mm, bore['vLo']),
                              (Hw_mm, bore['vHi']), (-Hw_mm, bore['vHi'])):
                x, y = _turn(cu, cv, th)
                poly.append((x, y))
            zg = originVecsMM[g][2]
            un = uHatVecsMM[g][2]
            poly = _clip(poly, un, 0.0, CageRise_mm - zg)
            poly = _clip(poly, -un, 0.0, CageRise_mm + zg)
            near = math.sqrt(max(0.0, Ri_mm * Ri_mm - s * s))
            far = math.sqrt(max(0.0, Ro_mm * Ro_mm - s * s))
            piece1 = _clip(_clip(poly, 0.0, -1.0, -near), 0.0, 1.0, far)
            piece2 = _clip(_clip(poly, 0.0, 1.0, -near), 0.0, -1.0, far)
            return [piece1, piece2]

        def channel_outline_mm(bore):
            g = bore['gear']
            sigma = bore['sigma']
            vLo, vHi = bore['vLo'], bore['vHi']
            kept = []
            k = 0
            while True:
                s = sigma * (SIn_mm + 0.1 * k)
                if abs(s) > SOut_mm + 1e-9:
                    break
                th = theta_mm(g, s)

                def emit(u, v):
                    x, y = _turn(u, v, th)
                    q = (
                        originVecsMM[g][0] + dirVecsMM[g][0] * s + uHatVecsMM[g][0] * x + vHatVecsMM[g][0] * y,
                        originVecsMM[g][1] + dirVecsMM[g][1] * s + uHatVecsMM[g][1] * x + vHatVecsMM[g][1] * y,
                        originVecsMM[g][2] + dirVecsMM[g][2] * s + uHatVecsMM[g][2] * x + vHatVecsMM[g][2] * y,
                    )
                    r = math.hypot(q[0], q[1])
                    if Ri_mm - 0.5 <= r <= Ro_mm + 0.5:
                        kept.append(q)

                for i in range(17):
                    f = i / 16.0
                    v = vLo + (vHi - vLo) * f
                    u = -Hw_mm + 2 * Hw_mm * f
                    emit(Hw_mm, v)
                    emit(-Hw_mm, v)
                    emit(u, vHi)
                    emit(u, vLo)
                k += 1
            return kept

        outlines = [channel_outline_mm(b) for b in bores]

        def channel_separation_mm():
            gaps = [
                ('+k', (0.0, 1.0, 0.0), (1, 2)),
                ('-e', (-1.0, 0.0, 0.0), (0, 2)),
                ('-k', (0.0, -1.0, 0.0), (0, 3)),
                ('+e', (1.0, 0.0, 0.0), (1, 3)),
            ]
            least, where, pair = math.inf, '', (0, 0)
            for name, d, (bi, bj) in gaps:
                across = _vcross((0.0, 0.0, 1.0), d)

                def project(idx):
                    return [(_vdot(q, across), q[2]) for q in outlines[idx]]

                hullA = _hull(project(bi))
                hullB = _hull(project(bj))
                best = -math.inf
                for h in (hullA, hullB):
                    m = len(h)
                    for i in range(m):
                        a_, b_ = h[i], h[(i + 1) % m]
                        ex, ey = b_[0] - a_[0], b_[1] - a_[1]
                        length = math.hypot(ex, ey)
                        if length == 0:
                            continue
                        mx, my = -ey / length, ex / length
                        aLo = min(mx * q[0] + my * q[1] for q in hullA)
                        aHi = max(mx * q[0] + my * q[1] for q in hullA)
                        bLo = min(mx * q[0] + my * q[1] for q in hullB)
                        bHi = max(mx * q[0] + my * q[1] for q in hullB)
                        best = max(best, bLo - aHi, aLo - bHi)
                if best < least:
                    least, where, pair = best, name, (bi, bj)
            return least, where, pair

        sep, sepWhere, sepPair = channel_separation_mm()
        need(sep >= CollarWall_mm, INPUT_ID_COLLAR_WALL,
             f'the wall between {boreNames[sepPair[0]]} and {boreNames[sepPair[1]]} '
             f'(facing {sepWhere}) is {sep:.4f} mm, must be >= collarWall {CollarWall_mm:.4f} mm')

        # The window search.
        def channel_top_mm():
            z = 0.0
            for bore in bores:
                g = bore['gear']
                sigma = bore['sigma']
                zg = originVecsMM[g][2]
                un = uHatVecsMM[g][2]
                corners = ((-Hw_mm, bore['vLo']), (Hw_mm, bore['vLo']),
                           (Hw_mm, bore['vHi']), (-Hw_mm, bore['vHi']))
                k = 0
                while True:
                    s = sigma * (SIn_mm + 0.01 * k)
                    if abs(s) > SOut_mm + 1e-9:
                        break
                    if abs(s) <= Ro_mm:
                        th = theta_mm(g, s)
                        ymax = 0.0
                        for (cu, cv) in corners:
                            _, y = _turn(cu, cv, th)
                            ymax = max(ymax, abs(y))
                        if math.hypot(s, ymax) >= Ri_mm:
                            for (cu, cv) in corners:
                                x, _ = _turn(cu, cv, th)
                                z = max(z, abs(zg + x * un))
                    k += 1
            return z

        def station_row_mm(bore, s):
            pieces, circles = [], []
            for pc in section_in_wall_mm(bore, s):
                if not pc:
                    continue
                cx = sum(p[0] for p in pc) / len(pc)
                cy = sum(p[1] for p in pc) / len(pc)
                r = max(math.hypot(p[0] - cx, p[1] - cy) for p in pc)
                pieces.append(pc)
                circles.append((cx, cy, r))
            return (s, pieces, circles)

        def station_signature_mm(bore, s):
            pcs = section_in_wall_mm(bore, s)
            return (len(pcs[0]) > 0, len(pcs[1]) > 0)

        def station_table_mm(bore):
            coarse = []
            k = 0
            while True:
                s = bore['spanLo'] + k * 0.002
                if s > bore['spanHi'] + 1e-9:
                    break
                coarse.append(station_row_mm(bore, s))
                k += 1
            rows = []
            for i, row in enumerate(coarse):
                if i > 0 and station_signature_mm(bore, coarse[i - 1][0]) != station_signature_mm(bore, row[0]):
                    k2 = 1
                    while True:
                        s = coarse[i - 1][0] + k2 * 0.0001
                        if s >= row[0] - 1e-9:
                            break
                        rows.append(station_row_mm(bore, s))
                        k2 += 1
                rows.append(row)
            return rows

        def point_in_piece(pc, x, y):
            if len(pc) < 3:
                return False
            n = len(pc)
            for i in range(n):
                a_, b_ = pc[i], pc[(i + 1) % n]
                if (b_[0] - a_[0]) * (y - a_[1]) - (b_[1] - a_[1]) * (x - a_[0]) < 0:
                    return False
            return True

        def seg_dist2(x, y, a_, b_):
            ex, ey = b_[0] - a_[0], b_[1] - a_[1]
            l2 = ex * ex + ey * ey
            t = 0.0
            if l2 > 0:
                t = max(0.0, min(1.0, ((x - a_[0]) * ex + (y - a_[1]) * ey) / l2))
            dx, dy = a_[0] + t * ex - x, a_[1] + t * ey - y
            return dx * dx + dy * dy

        def rows_bisect(rows, sq):
            lo, hi = 0, len(rows)
            while lo < hi:
                mid = (lo + hi) // 2
                if rows[mid][0] < sq:
                    lo = mid + 1
                else:
                    hi = mid
            return lo

        def wall_gap_mm(bore, rows, q, reach):
            g = bore['gear']
            x, y, sq = local_mm(g, q)
            clamped = min(max(sq, bore['spanLo']), bore['spanHi'])
            if math.hypot(math.hypot(x, y), sq - clamped) - cornerDist * 10 >= reach:
                return reach
            best = [reach * reach]

            def visit(row):
                s, pieces, circles = row
                ds = sq - s
                if ds * ds >= best[0]:
                    return False
                for pc, c in zip(pieces, circles):
                    o = math.hypot(x - c[0], y - c[1]) - c[2]
                    if o > 0 and ds * ds + o * o >= best[0]:
                        continue
                    if point_in_piece(pc, x, y):
                        d2 = 0.0
                    else:
                        d2 = min(seg_dist2(x, y, pc[j], pc[(j + 1) % len(pc)]) for j in range(len(pc)))
                    best[0] = min(best[0], ds * ds + d2)
                return True

            start = rows_bisect(rows, sq)
            i = start
            while i < len(rows):
                if not visit(rows[i]):
                    break
                i += 1
            i = start - 1
            while i >= 0:
                if not visit(rows[i]):
                    break
                i -= 1
            return math.sqrt(best[0])

        def crossing_point_mm(bore):
            g = bore['gear']
            sc = bore['sigma'] * CageRadius_mm
            return (
                originVecsMM[g][0] + dirVecsMM[g][0] * sc,
                originVecsMM[g][1] + dirVecsMM[g][1] * sc,
                originVecsMM[g][2] + dirVecsMM[g][2] * sc,
            )

        def new_window_mm(name, d, zLimit, tables):
            across = _vcross((0.0, 0.0, 1.0), d)
            flank, far = [], []
            for i, bore in enumerate(bores):
                crossing = crossing_point_mm(bore)
                if _vdot(crossing, d) > 0:
                    flank.append(i)
                else:
                    far.append(i)
            if len(flank) != 2 or len(far) != 2:
                return {'facing': name, 'corners': None, 'reason': f'{len(flank)} flanking bores'}

            low, high = flank[0], flank[1]
            crossLow = crossing_point_mm(bores[low])
            crossHigh = crossing_point_mm(bores[high])
            if crossHigh[2] < crossLow[2]:
                low, high = high, low
                crossLow, crossHigh = crossHigh, crossLow
            lean = -1.0
            tLow = _vdot(crossLow, across)
            tHigh = _vdot(crossHigh, across)
            if tHigh > tLow:
                lean = 1.0

            def reach(idx, highest):
                bore = bores[idx]
                best = -math.inf if highest else math.inf
                k = 0
                while True:
                    s = bore['spanLo'] + k * 0.001
                    if s > bore['spanHi'] + 1e-9:
                        break
                    g = bore['gear']
                    for pc in section_in_wall_mm(bore, s):
                        for (cu, cv) in pc:
                            q = (
                                originVecsMM[g][0] + uHatVecsMM[g][0] * cu + vHatVecsMM[g][0] * cv + dirVecsMM[g][0] * s,
                                originVecsMM[g][1] + uHatVecsMM[g][1] * cu + vHatVecsMM[g][1] * cv + dirVecsMM[g][1] * s,
                                originVecsMM[g][2] + uHatVecsMM[g][2] * cu + vHatVecsMM[g][2] * cv + dirVecsMM[g][2] * s,
                            )
                            t = _vdot(q, across)
                            z = q[2]
                            m = z + lean * t
                            if highest:
                                best = max(best, m)
                            else:
                                best = min(best, m)
                    k += 1
                return best

            lo = reach(low, True) + math.sqrt(2) * CollarWall_mm
            hi = reach(high, False) - math.sqrt(2) * CollarWall_mm
            top = min(2 * zLimit - hi, hi + math.sqrt(2) * Ri_mm)
            bottom = max(-2 * zLimit - lo, lo - math.sqrt(2) * Ri_mm)
            if hi <= lo:
                return {'facing': name, 'corners': None,
                        'reason': f'the flanking bores leave no band (lo {lo:.3f} mm, hi {hi:.3f} mm)'}

            needGap = CollarWall_mm + 0.1 / math.sqrt(2) + 0.005

            def a0(t):
                return math.sqrt(max(0.0, Ri_mm * Ri_mm - t * t))

            def a1(t):
                return math.sqrt(max(0.0, Ro_mm * Ro_mm - t * t))

            def at(t, z, a_):
                return (across[0] * t + d[0] * a_, across[1] * t + d[1] * a_, z + d[2] * a_)

            def clear(t):
                zLow = max(lo - lean * t, bottom + lean * t)
                zHigh = min(hi - lean * t, top + lean * t)
                if zLow > zHigh:
                    return False
                pts = []
                for z in (zLow, zHigh):
                    a_ = a0(t)
                    while True:
                        if a_ >= a1(t):
                            pts.append(at(t, z, a1(t)))
                            break
                        pts.append(at(t, z, a_))
                        a_ += 0.1
                nz = math.ceil((zHigh - zLow) / 0.1)
                zs = []
                if nz < 1:
                    nz = 1
                for k_ in range(1, int(nz)):
                    zs.append(zLow + (zHigh - zLow) * k_ / nz)
                j = 0
                while True:
                    z = -zLimit + 0.1 * j
                    if z > zLimit:
                        break
                    if zLow < z < zHigh:
                        zs.append(z)
                    j += 1
                for z in zs:
                    pts.append(at(t, z, a0(t)))
                    pts.append(at(t, z, a1(t)))
                for pq in pts:
                    for fi in far:
                        if wall_gap_mm(bores[fi], tables[fi], pq, needGap) < needGap:
                            return False
                return True

            def end(delta):
                loB, hiB = 0.0, min(Ri_mm, Ro_mm / math.sqrt(2)) * (1 - 1e-9)
                for _ in range(24):
                    mid = (loB + hiB) / 2.0
                    if clear(delta * mid):
                        loB = mid
                    else:
                        hiB = mid
                return loB

            right = end(1.0)
            left = -end(-1.0)
            if right <= left:
                return {'facing': name, 'corners': None,
                        'reason': f'the far bores leave the band no length '
                                  f'(left {left:.3f} mm, right {right:.3f} mm)'}

            corners = [(-2 * Ro_mm, -2 * Ro_mm), (2 * Ro_mm, -2 * Ro_mm),
                       (2 * Ro_mm, 2 * Ro_mm), (-2 * Ro_mm, 2 * Ro_mm)]
            corners = _clip(corners, lean, 1.0, hi)
            corners = _clip(corners, -lean, -1.0, -lo)
            corners = _clip(corners, -lean, 1.0, top)
            corners = _clip(corners, lean, -1.0, -bottom)
            corners = _clip(corners, 1.0, 0.0, right)
            corners = _clip(corners, -1.0, 0.0, -left)
            kept = []
            for pt_ in corners:
                if kept and math.hypot(pt_[0] - kept[-1][0], pt_[1] - kept[-1][1]) < 0.001:
                    continue
                kept.append(pt_)
            while len(kept) > 1 and math.hypot(kept[0][0] - kept[-1][0], kept[0][1] - kept[-1][1]) < 0.001:
                kept.pop()
            if len(kept) < 3 or _area(kept) <= 0:
                return {'facing': name, 'corners': None,
                        'reason': f'the hexagon has {len(kept)} distinct corners and no area'}
            return {'facing': name, 'd': d, 'across': across, 'lean': lean,
                    'lo': lo, 'hi': hi, 'bottom': bottom, 'top': top,
                    'left': left, 'right': right, 'corners': kept, 'reason': None}

        if Sigma <= math.pi / 2 + 1e-12:
            facings = [('+k', (0.0, 1.0, 0.0)), ('-k', (0.0, -1.0, 0.0))]
        else:
            facings = [('+e', (1.0, 0.0, 0.0)), ('-e', (-1.0, 0.0, 0.0))]

        tables = [station_table_mm(b) for b in bores]
        zLimit = channel_top_mm()
        windows = []
        for name, d in facings:
            w = new_window_mm(name, d, zLimit, tables)
            if w['corners'] is None:
                futil.log(f'No window facing {w["facing"]}: {w["reason"]}')
            else:
                windows.append(w)

        # Store every handle on self, in cm, in the real world frame terms
        # that S06/S07 will set up (d/across stay as frame components; the
        # corners come back to cm).
        self.windows = []
        for w in windows:
            self.windows.append({
                'facing': w['facing'],
                'd': w['d'],
                'across': w['across'],
                'lean': w['lean'],
                'corners': [(t / 10.0, z / 10.0) for (t, z) in w['corners']],
            })

        self.bores = []
        for b in bores:
            self.bores.append({
                'gear': b['gear'],
                'sigma': b['sigma'],
                'vLo': b['vLo'] / 10.0,
                'vHi': b['vHi'] / 10.0,
                'spanLo': b['spanLo'] / 10.0,
                'spanHi': b['spanHi'] / 10.0,
            })

        self.W, self.T, self.P, self.H = W, T, P, H
        self.Lambda, self.tanSlant, self.Bow_cm = Lambda, tanSlant, Bow_cm
        self.N = N
        self.cageRadius, self.cageRise = cageRadius, cageRise
        self.clearance, self.roofAllowance = clearance, roofAllowance
        self.collarHalf, self.collarWall = collarHalf, collarWall
        self.Sigma, self.A = Sigma, A
        self.Ri, self.Ro, self.hw, self.ht = Ri, Ro, hw, ht
        self.sIn, self.sOut = sIn, sOut
        self.cell, self.steps = cell, steps
        self.wholeCells, self.remainderTeeth = wholeCells, remainderTeeth
        self.Z0, self.Phi = Z0, Phi

    # -----------------------------------------------------------------
    # Real-world-frame helpers, built once buildAnchor (S05-S07) has set
    # self.C, self.eHat, self.kHat, self.nHat, self.dirVecs, self.originVecs,
    # self.uHatVecs and self.vHatVecs.
    # -----------------------------------------------------------------

    def _theta(self, g, s):
        return s / self.Lambda + self.Phi[g]

    def _world(self, g, s, u, v):
        th = self._theta(g, s)
        x, y = _turn(u, v, th)
        o = self.originVecs[g]
        d = self.dirVecs[g]
        uu = self.uHatVecs[g]
        vv = self.vHatVecs[g]
        return (
            o[0] + s * d[0] + x * uu[0] + y * vv[0],
            o[1] + s * d[1] + x * uu[1] + y * vv[1],
            o[2] + s * d[2] + x * uu[2] + y * vv[2],
        )

    def _utooth(self, g, v, s):
        arg = 2 * math.pi * (s + self.tanSlant * v - self.Z0[g]) / self.P
        return self.W / 2 - self.H / 2 + (self.H / 2) * math.cos(arg) - self.Bow_cm * v * v

    def _utoothWrongSlant(self, g, v, s):
        arg = 2 * math.pi * (s - self.tanSlant * v - self.Z0[g]) / self.P
        return self.W / 2 - self.H / 2 + (self.H / 2) * math.cos(arg) - self.Bow_cm * v * v

    def _cellStart(self, g):
        return self.Z0[g] - self.N * self.P / 2.0

    def _frameToWorld(self, comps):
        eX, kY, nZ = comps
        return _vadd(_vadd(_vscale(self.eHat, eX), _vscale(self.kHat, kY)), _vscale(self.nHat, nZ))

    # -----------------------------------------------------------------
    # S04: the component tree.
    # -----------------------------------------------------------------

    def buildComponentTree(self):
        topOcc = self.getOccurrence()
        topOcc.component.name = 'Screw Gearing'
        topComponent = topOcc.component

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

    # -----------------------------------------------------------------
    # S05-S07: the Anchor sketch and the world frame it establishes, and the
    # two Axis Planes.
    # -----------------------------------------------------------------

    def buildAnchor(self):
        design: adsk.fusion.Component = self.designOcc.component

        anchorSketch: adsk.fusion.Sketch = design.sketches.add(self.targetPlane)
        anchorSketch.name = 'Anchor'

        projectedEntities: adsk.core.ObjectCollection = anchorSketch.project(self.centrePoint)
        projected: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(projectedEntities.item(0))
        p = projected.geometry
        start = adsk.core.Point3D.create(p.x - 0.5, p.y, 0)
        end = adsk.core.Point3D.create(p.x + 0.5, p.y, 0)
        anchorLine = anchorSketch.sketchCurves.sketchLines.addByTwoPoints(start, end)
        anchorSketch.geometricConstraints.addCoincident(projected, anchorLine)
        anchorSketch.geometricConstraints.addMidPoint(projected, anchorLine)
        anchorSketch.geometricConstraints.addHorizontal(anchorLine)
        textPoint = adsk.core.Point3D.create(p.x, p.y + 0.3, 0)
        lengthDim = anchorSketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
        lengthDim.parameter.value = 1.0

        if not anchorSketch.isFullyConstrained:
            raise Exception('ScrewGear: sketch "Anchor" is not fully constrained')

        C = _tup(projected.worldGeometry)
        startWorld = _tup(anchorLine.startSketchPoint.worldGeometry)
        endWorld = _tup(anchorLine.endSketchPoint.worldGeometry)
        eRaw = _vsub(endWorld, startWorld)
        eHat = _vscale(eRaw, 1.0 / _vlen(eRaw))

        self.anchorLine = anchorLine
        self.C = C
        self.eHat = eHat

        # S06: the Gear A Axis Plane, and the sign of n̂.
        halfA = self.A / 2.0
        planeInputA = design.constructionPlanes.createInput()
        planeInputA.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-halfA))
        gearAPlane = design.constructionPlanes.add(planeInputA)
        gearAPlane.name = 'Gear A Axis Plane'

        planeGeomA = gearAPlane.geometry
        originA = _tup(planeGeomA.origin)
        normalA = _tup(planeGeomA.normal)
        dA = _vdot(_vsub(C, originA), normalA)
        if abs(abs(dA) - halfA) > 1e-6:
            raise Exception('ScrewGear: "Gear A Axis Plane" is not A/2 from the centre point')
        nHat = normalA if dA > 0 else _vscale(normalA, -1.0)

        kHat = _vcross(nHat, eHat)
        self.nHat = nHat
        self.kHat = kHat

        sigmaHalf = self.Sigma / 2.0
        cosH, sinH = math.cos(sigmaHalf), math.sin(sigmaHalf)
        dirA = _vadd(_vscale(eHat, cosH), _vscale(kHat, sinH))
        dirB = _vadd(_vscale(eHat, cosH), _vscale(kHat, -sinH))
        originAVec = _vsub(C, _vscale(nHat, halfA))
        originBVec = _vadd(C, _vscale(nHat, halfA))
        uHatA = nHat
        uHatB = _vscale(nHat, -1.0)
        vHatA = _vcross(dirA, uHatA)
        vHatB = _vcross(dirB, uHatB)

        self.dirVecs = [dirA, dirB]
        self.originVecs = [originAVec, originBVec]
        self.uHatVecs = [uHatA, uHatB]
        self.vHatVecs = [vHatA, vHatB]

        # S07: the Gear B Axis Plane.
        planeInputB = design.constructionPlanes.createInput()
        planeInputB.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(halfA))
        gearBPlane = design.constructionPlanes.add(planeInputB)
        gearBPlane.name = 'Gear B Axis Plane'
        planeGeomB = gearBPlane.geometry
        originB = _tup(planeGeomB.origin)
        normalB = _tup(planeGeomB.normal)
        dB = _vdot(_vsub(C, originB), normalB)
        if abs(abs(dB) - halfA) > 1e-6:
            raise Exception('ScrewGear: "Gear B Axis Plane" is not A/2 from the centre point')

        self.axisPlanes = [gearAPlane, gearBPlane]

    # -----------------------------------------------------------------
    # S08-S16: a gear's ribbon.
    # -----------------------------------------------------------------

    def buildGear(self, index):
        self.buildSweepPaths(index)
        self.buildToothCell(index)
        self.repeatCellByDoubling(index)

    def buildSweepPaths(self, g):
        design = self.designOcc.component
        gearLabel = 'Gear A' if g == 0 else 'Gear B'

        paths = design.sketches.add(self.axisPlanes[g])
        paths.name = f'{gearLabel} Paths'

        stations = [-self.sOut, -self.sIn, self.sIn, self.sOut]
        pts = []
        for s in stations:
            world = _vadd(self.originVecs[g], _vscale(self.dirVecs[g], s))
            local = paths.modelToSketchSpace(_pt3(world))
            local.z = 0.0
            pts.append(paths.sketchPoints.add(local))

        boreMinus = paths.sketchCurves.sketchLines.addByTwoPoints(pts[0], pts[1])
        borePlus = paths.sketchCurves.sketchLines.addByTwoPoints(pts[2], pts[3])

        for pt in pts:
            pt.isFixed = True

        if not paths.isFullyConstrained:
            raise Exception(f'ScrewGear: sketch "{paths.name}" is not fully constrained')

        self.pathLines[g] = {'bore-': boreMinus, 'bore+': borePlus}

    def _drawCellSection(self, sketch: adsk.fusion.Sketch, g, s):
        hv = self.T / 2.0
        uB = -self.W / 2.0
        M = TOOTH_SPLINE_POINTS

        def addPt(world, sketch: adsk.fusion.Sketch = sketch) -> adsk.fusion.SketchPoint:
            # sketch is re-declared as a defaulted parameter because a nested
            # closure does not see the enclosing method's annotation.
            local: adsk.core.Point3D = sketch.modelToSketchSpace(_pt3(world))
            return sketch.sketchPoints.add(local)

        B0 = addPt(self._world(g, s, uB, -hv))
        fPts = []
        for j in range(M):
            v = -hv + j * self.T / (M - 1)
            u = self._utooth(g, v, s)
            fPts.append(addPt(self._world(g, s, u, v)))
        B1 = addPt(self._world(g, s, uB, hv))

        L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, fPts[0])
        fitPoints = adsk.core.ObjectCollection.create()
        for fp in fPts:
            fitPoints.add(fp)
        S = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)
        if S is None or S.fitPoints.count != M:
            raise Exception(
                f'ScrewGear: cell section of "{sketch.name}" at station {s} '
                f'did not fit {M} points')
        L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(fPts[M - 1], B1)
        L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)

        return [L1, S, L3, L4], [B0] + fPts + [B1]

    def _loftSections(self, design: adsk.fusion.Component, label, sectionCurves) -> adsk.fusion.BRepBody:
        loftInput = design.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for curves in sectionCurves:
            collection = adsk.core.ObjectCollection.create()
            for curve in curves:
                collection.add(curve)
            path = design.features.createPath(collection, False)
            loftInput.loftSections.add(path)
        loftFeature = design.features.loftFeatures.add(loftInput)
        bodyResult: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(loftFeature.bodies.item(0))
        if loftFeature.bodies.count != 1 or not bodyResult.isSolid:
            raise Exception(f'ScrewGear: {label} loft did not produce one solid body')
        return bodyResult

    def buildToothCell(self, g):
        design = self.designOcc.component
        gearLabel = 'Gear A' if g == 0 else 'Gear B'

        cellSketch = design.sketches.add(self.axisPlanes[g])
        cellSketch.name = f'{gearLabel} Cell Sections'
        cellSketch.isComputeDeferred = True

        s0 = self._cellStart(g)
        sectionsCount = self.cell * self.steps
        sectionCurves = []
        allPoints = []
        for k in range(sectionsCount + 1):
            sk = s0 + k * self.P / self.steps
            curves, pts = self._drawCellSection(cellSketch, g, sk)
            sectionCurves.append(curves)
            allPoints.extend(pts)

        for pt in allPoints:
            pt.isFixed = True
        for curves in sectionCurves:
            spline: adsk.fusion.SketchFittedSpline = adsk.fusion.SketchFittedSpline.cast(curves[1])
            for i in range(spline.fitPoints.count):
                fitPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(spline.fitPoints.item(i))
                fitPoint.isFixed = True

        cellSketch.isComputeDeferred = False

        if not cellSketch.isFullyConstrained:
            raise Exception(f'ScrewGear: sketch "{cellSketch.name}" is not fully constrained')
        if cellSketch.profiles.count != sectionsCount + 1:
            raise Exception(
                f'ScrewGear: sketch "{cellSketch.name}" has {cellSketch.profiles.count} '
                f'profiles, want {sectionsCount + 1}')
        for curves in sectionCurves:
            if len(curves) != 4:
                raise Exception(
                    f'ScrewGear: a section of "{cellSketch.name}" has {len(curves)} curves, want 4')

        self._cellBody = self._loftSections(design, f'{gearLabel} cell', sectionCurves)
        self._checkSlantSign(g, s0)

    def _checkSlantSign(self, g, s0):
        gearLabel = 'Gear A' if g == 0 else 'Gear B'
        sc = self.Z0[g] + self.P * math.ceil((s0 + self.P / 2 - self.Z0[g]) / self.P)
        anyUsed = False
        for sf in (-1.0, 1.0):
            vp = sf * (self.T / 2 - 0.025)
            up = self.W / 2 - self.Bow_cm * vp * vp - 0.025
            for onRidge in (True, False):
                s = sc - self.tanSlant * vp if onRidge else sc + self.tanSlant * vp
                m = self._utooth(g, vp, s) - up
                mWrong = self._utoothWrongSlant(g, vp, s) - up
                used = abs(m) >= 0.01 and abs(mWrong) >= 0.01 and ((m > 0) != (mWrong > 0))
                if not used:
                    continue
                anyUsed = True
                probe = _pt3(self._world(g, s, up, vp))
                cellBody: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self._cellBody)
                containment = cellBody.pointContainment(probe)
                expectInside = m > 0
                ok = (
                    (expectInside and containment == adsk.fusion.PointContainment.PointInsidePointContainment)
                    or
                    ((not expectInside) and containment == adsk.fusion.PointContainment.PointOutsidePointContainment)
                )
                if not ok:
                    raise Exception(
                        f'ScrewGear: {gearLabel} tooth slant sign check failed at face {sf}, '
                        f'{"on" if onRidge else "off"}-ridge probe (containment read {containment})')
        if not anyUsed:
            futil.log(f"{gearLabel}: the tooth slant's sign was not checked")

    def _copyBody(self, design, body) -> adsk.fusion.BRepBody:
        copyFeature = design.features.copyPasteBodies.add(body)
        return adsk.fusion.BRepBody.cast(copyFeature.bodies.item(0))

    def _screwMove(self, design, g, body, k):
        axisVector: adsk.core.Vector3D = _vec3(self.dirVecs[g])
        axisPoint = _pt3(self.originVecs[g])
        rot = adsk.core.Matrix3D.create()
        rot.setToRotation(k * self.P / self.Lambda, axisVector, axisPoint)
        shift: adsk.core.Vector3D = axisVector.copy()
        shift.scaleBy(k * self.P)
        mov = adsk.core.Matrix3D.create()
        mov.translation = shift
        rot.transformBy(mov)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = design.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(rot)
        design.features.moveFeatures.add(moveInput)

    def _joinBodies(self, design, body, toolBody, label) -> adsk.fusion.BRepBody:
        tools = adsk.core.ObjectCollection.create()
        tools.add(toolBody)
        combineInput = design.features.combineFeatures.createInput(body, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isKeepToolBodies = False
        combineFeature = design.features.combineFeatures.add(combineInput)
        if combineFeature.bodies.count != 1:
            raise Exception(
                f'ScrewGear: {label} join left {combineFeature.bodies.count} bodies, want 1')
        return adsk.fusion.BRepBody.cast(combineFeature.bodies.item(0))

    def repeatCellByDoubling(self, g):
        design = self.designOcc.component
        gearLabel = 'Gear A' if g == 0 else 'Gear B'

        body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self._cellBody)
        m = 1
        q = self.wholeCells
        r = self.remainderTeeth

        if q > 1:
            # Plain arithmetic in place of int.bit_length, which the checker
            # cannot resolve: topBit is floor(log2(q)).
            topBit = 0
            qq = q
            while qq > 1:
                qq >>= 1
                topBit += 1
            asides = []
            roundIndex = 0
            for bitIndex in range(topBit):
                if (q >> bitIndex) & 1:
                    asides.append((self._copyBody(design, body), m))
                movedCopy = self._copyBody(design, body)
                self._screwMove(design, g, movedCopy, m * self.cell)
                roundIndex += 1
                body = self._joinBodies(design, body, movedCopy,
                                         f'{gearLabel} doubling round {roundIndex}')
                m = 2 * m
            for asideBody, cells in sorted(asides, key=lambda t: -t[1]):
                self._screwMove(design, g, asideBody, m * self.cell)
                body = self._joinBodies(design, body, asideBody,
                                         f'{gearLabel} aside of {cells} cell(s)')
                m = m + cells

        if r > 0:
            remainderSketch = design.sketches.add(self.axisPlanes[g])
            remainderSketch.name = f'{gearLabel} Cell Remainder'
            remainderSketch.isComputeDeferred = True

            s0r = self._cellStart(g) + q * self.cell * self.P
            sectionsCount = r * self.steps
            sectionCurves = []
            allPoints = []
            for k in range(sectionsCount + 1):
                sk = s0r + k * self.P / self.steps
                curves, pts = self._drawCellSection(remainderSketch, g, sk)
                sectionCurves.append(curves)
                allPoints.extend(pts)

            for pt in allPoints:
                pt.isFixed = True
            for curves in sectionCurves:
                spline: adsk.fusion.SketchFittedSpline = adsk.fusion.SketchFittedSpline.cast(curves[1])
                for i in range(spline.fitPoints.count):
                    fitPoint: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(spline.fitPoints.item(i))
                    fitPoint.isFixed = True

            remainderSketch.isComputeDeferred = False

            if not remainderSketch.isFullyConstrained:
                raise Exception(
                    f'ScrewGear: sketch "{remainderSketch.name}" is not fully constrained')
            if remainderSketch.profiles.count != sectionsCount + 1:
                raise Exception(
                    f'ScrewGear: sketch "{remainderSketch.name}" has '
                    f'{remainderSketch.profiles.count} profiles, want {sectionsCount + 1}')

            remainderBody = self._loftSections(design, f'{gearLabel} remainder', sectionCurves)
            body = self._joinBodies(design, body, remainderBody, f'{gearLabel} remainder')

        body.name = gearLabel
        self.gearBodies[g] = body

    # -----------------------------------------------------------------
    # S17-S26: the cage (the sleeve, its four bores and its windows).
    # -----------------------------------------------------------------

    def buildCage(self):
        design: adsk.fusion.Component = self.designOcc.component

        sleeve: adsk.fusion.Sketch = design.sketches.add(self.targetPlane)
        sleeve.name = 'Sleeve'
        centre = sleeve.modelToSketchSpace(_pt3(self.C))
        centre.z = 0.0

        for radius in (self.Ri, self.Ro):
            circle = sleeve.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)
            circle.centerSketchPoint.isFixed = True
            textPoint = adsk.core.Point3D.create(centre.x + radius, centre.y, 0.0)
            diameterDim = sleeve.sketchDimensions.addDiameterDimension(circle, textPoint)
            diameterDim.parameter.value = 2 * radius

        if not sleeve.isFullyConstrained:
            raise Exception('ScrewGear: sketch "Sleeve" is not fully constrained')

        ringProfile: adsk.fusion.Profile = adsk.fusion.Profile.cast(None)
        ringCount = 0
        for profile in sleeve.profiles:
            if profile.profileLoops.count == 2:
                ringCount += 1
                ringProfile = profile
        if ringCount != 1:
            raise Exception(
                f'ScrewGear: sketch "Sleeve" has {ringCount} profiles with two loops, want 1')

        tubeInput = design.features.extrudeFeatures.createInput(
            adsk.fusion.Profile.cast(ringProfile), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        tubeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(self.cageRise), False)
        tubeFeature = design.features.extrudeFeatures.add(tubeInput)
        if tubeFeature.bodies.count != 1:
            raise Exception(
                f'ScrewGear: Sleeve tube extrude left {tubeFeature.bodies.count} bodies, want 1')
        self.cageBody: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(tubeFeature.bodies.item(0))

        for bore in self.bores:
            self._buildBore(design, bore)

        futil.log(
            'Print the cage standing on its end below the selected plane: the roof '
            'allowance is on the bridged roofs that way up.')

        if self.windows:
            windowPlane = self._buildWindowPlane(design)
            for window in self.windows:
                self._buildWindow(design, windowPlane, window)

    def _buildBore(self, design, bore):
        g = bore['gear']
        sigma = bore['sigma']
        gearLabel = 'Gear A' if g == 0 else 'Gear B'
        sideLabel = '-R' if sigma < 0 else '+R'
        line = self.pathLines[g]['bore-'] if sigma < 0 else self.pathLines[g]['bore+']

        planeInput = design.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))
        borePlane = design.constructionPlanes.add(planeInput)
        borePlane.name = f'{gearLabel} Bore {sideLabel} Plane'

        s0 = bore['spanLo']
        theta = self._theta(g, s0)
        uB, uF = -self.hw, self.hw
        vLo, vHi = bore['vLo'], bore['vHi']

        boreSketch: adsk.fusion.Sketch = design.sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore {sideLabel}'
        boreSketch.isComputeDeferred = True

        def toLocal(world, boreSketch: adsk.fusion.Sketch = boreSketch) -> adsk.core.Point3D:
            # boreSketch is re-declared as a defaulted parameter because a
            # nested closure does not see the enclosing method's annotation.
            local: adsk.core.Point3D = boreSketch.modelToSketchSpace(_pt3(world))
            local.z = 0.0
            return local

        oWorld = _vadd(self.originVecs[g], _vscale(self.dirVecs[g], s0))
        cpWorld = _vadd(oWorld, _vscale(self.uHatVecs[g], self.A / 2.0))

        O = boreSketch.sketchPoints.add(toLocal(oWorld))
        Cp = boreSketch.sketchPoints.add(toLocal(cpWorld))
        Ru = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)
        Ru.isConstruction = True

        eSeedLocal = toLocal(self._world(g, s0, uF, 0.0))
        K = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, eSeedLocal)
        K.isConstruction = True
        E = K.endSketchPoint

        O.isFixed = True
        Cp.isFixed = True

        c0 = toLocal(self._world(g, s0, uB, vLo))
        c1 = toLocal(self._world(g, s0, uF, vLo))
        c2 = toLocal(self._world(g, s0, uF, vHi))
        c3 = toLocal(self._world(g, s0, uB, vHi))

        L1 = boreSketch.sketchCurves.sketchLines.addByTwoPoints(c0, c1)
        L2 = boreSketch.sketchCurves.sketchLines.addByTwoPoints(L1.endSketchPoint, c2)
        L3 = boreSketch.sketchCurves.sketchLines.addByTwoPoints(L2.endSketchPoint, c3)
        L4 = boreSketch.sketchCurves.sketchLines.addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)

        lengthText = toLocal(self._world(g, s0, uF / 2.0, 0.1))
        lengthDim = boreSketch.sketchDimensions.addDistanceDimension(
            O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)
        lengthDim.parameter.value = uF

        useK = abs(math.sin(theta)) >= math.sqrt(0.5)
        if useK:
            r1 = self.uHatVecs[g]
            r2 = _vadd(_vscale(self.uHatVecs[g], math.cos(theta)),
                       _vscale(self.vHatVecs[g], math.sin(theta)))
            vertexWorld = oWorld
        else:
            r1 = self.uHatVecs[g]
            r2 = _vadd(_vscale(self.uHatVecs[g], -math.sin(theta)),
                       _vscale(self.vHatVecs[g], math.cos(theta)))
            vertexWorld = _vadd(oWorld, _vscale(self.uHatVecs[g], uF / math.cos(theta)))

        cosAngle = max(-1.0, min(1.0, _vdot(r1, r2)))
        angleValue = math.acos(cosAngle)
        bisector = _vadd(r1, r2)
        bisLen = _vlen(bisector)
        bisector = _vscale(bisector, 1.0 / bisLen) if bisLen > 1e-12 else r1
        angleText = toLocal(_vadd(vertexWorld, _vscale(bisector, uF / 2.0)))

        if useK:
            angleDim = boreSketch.sketchDimensions.addAngularDimension(Ru, K, angleText)
        else:
            angleDim = boreSketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)
        angleDim.parameter.value = angleValue

        boreSketch.geometricConstraints.addParallel(L1, K)
        offsetText1 = toLocal(self._world(g, s0, (uB + uF) / 2.0, vLo))
        offsetDim1 = boreSketch.sketchDimensions.addOffsetDimension(K, L1, offsetText1)
        offsetDim1.parameter.value = -vLo

        boreSketch.geometricConstraints.addParallel(L3, K)
        offsetText2 = toLocal(self._world(g, s0, (uB + uF) / 2.0, vHi))
        offsetDim2 = boreSketch.sketchDimensions.addOffsetDimension(K, L3, offsetText2)
        offsetDim2.parameter.value = vHi

        boreSketch.geometricConstraints.addCoincident(E, L2)
        boreSketch.geometricConstraints.addPerpendicular(L2, K)

        boreSketch.geometricConstraints.addParallel(L4, L2)
        offsetText3 = toLocal(self._world(g, s0, uB, (vLo + vHi) / 2.0))
        offsetDim3 = boreSketch.sketchDimensions.addOffsetDimension(L2, L4, offsetText3)
        offsetDim3.parameter.value = uF - uB

        boreSketch.isComputeDeferred = False
        if not boreSketch.isFullyConstrained:
            raise Exception(f'ScrewGear: sketch "{boreSketch.name}" is not fully constrained')

        profile = find_profile_by_curve_counts(boreSketch, lines=4)

        path = design.features.createPath(line, False)
        sweepInput = design.features.sweepFeatures.createInput(
            profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(
            (bore['spanHi'] - bore['spanLo']) / self.Lambda)
        sweepInput.participantBodies = [self.cageBody]
        sweepFeature = design.features.sweepFeatures.add(sweepInput)
        if sweepFeature.bodies.count != 1:
            raise Exception(
                f'ScrewGear: {gearLabel} Bore {sideLabel} sweep cut left '
                f'{sweepFeature.bodies.count} bodies, want 1')
        self.cageBody = sweepFeature.bodies.item(0)

        sc = sigma * self.cageRadius
        thc = self._theta(g, sc)
        uDirAtCrossing = _vadd(_vscale(self.uHatVecs[g], math.cos(thc)),
                               _vscale(self.vHatVecs[g], math.sin(thc)))
        atCrossing = _vadd(self.originVecs[g], _vscale(self.dirVecs[g], sc))
        off = self.W / 2.0 + self.clearance / 2.0
        for probe in (_vadd(atCrossing, _vscale(uDirAtCrossing, off)),
                      _vsub(atCrossing, _vscale(uDirAtCrossing, off))):
            containment = self.cageBody.pointContainment(_pt3(probe))
            if containment != adsk.fusion.PointContainment.PointOutsidePointContainment:
                raise Exception(
                    f'ScrewGear: {gearLabel} Bore {sideLabel} sweep sense check failed '
                    f'(containment read {containment})')

    def _buildWindowPlane(self, design):
        facing = self.windows[0]['facing']
        planeInput = design.constructionPlanes.createInput()
        if facing in ('+k', '-k'):
            planeInput.setByAngle(
                self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)
        else:
            planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))
        windowPlane = design.constructionPlanes.add(planeInput)
        windowPlane.name = 'Window Plane'
        return windowPlane

    def _buildWindow(self, design, windowPlane, window):
        facing = window['facing']
        dWorld = self._frameToWorld(window['d'])
        acrossWorld = self._frameToWorld(window['across'])

        windowSketch = design.sketches.add(windowPlane)
        windowSketch.name = f'Window {facing}'

        pts = []
        for (t, z) in window['corners']:
            world = _vadd(self.C, _vadd(_vscale(acrossWorld, t), _vscale(self.nHat, z)))
            local = windowSketch.modelToSketchSpace(_pt3(world))
            local.z = 0.0
            pts.append(windowSketch.sketchPoints.add(local))

        n = len(pts)
        for i in range(n):
            windowSketch.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % n])
        for pt in pts:
            pt.isFixed = True

        if not windowSketch.isFullyConstrained:
            raise Exception(f'ScrewGear: sketch "{windowSketch.name}" is not fully constrained')
        if windowSketch.profiles.count != 1:
            raise Exception(
                f'ScrewGear: sketch "{windowSketch.name}" has {windowSketch.profiles.count} '
                f'profiles, want 1')
        profile = windowSketch.profiles.item(0)

        tAvg = sum(c[0] for c in window['corners']) / n
        zAvg = sum(c[1] for c in window['corners']) / n
        a0 = math.sqrt(max(0.0, self.Ri * self.Ri - tAvg * tAvg))
        a1 = math.sqrt(max(0.0, self.Ro * self.Ro - tAvg * tAvg))
        probeWorld = _vadd(
            self.C,
            _vadd(_vscale(acrossWorld, tAvg),
                  _vadd(_vscale(self.nHat, zAvg), _vscale(dWorld, (a0 + a1) / 2.0))))

        beforeContainment = self.cageBody.pointContainment(_pt3(probeWorld))
        if beforeContainment != adsk.fusion.PointContainment.PointInsidePointContainment:
            raise Exception(
                f'ScrewGear: window {facing} probe is not inside the cage before the cut '
                f'(containment read {beforeContainment})')

        cutInput = design.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        checkLocal = windowSketch.modelToSketchSpace(_pt3(_vadd(self.C, dWorld)))
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if checkLocal.z > 0
                     else adsk.fusion.ExtentDirections.NegativeExtentDirection)
        cutInput.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByReal(self.Ro + 0.1)),
            direction)
        cutInput.participantBodies = [self.cageBody]
        cutFeature = design.features.extrudeFeatures.add(cutInput)
        if cutFeature.bodies.count != 1:
            raise Exception(
                f'ScrewGear: window {facing} cut left {cutFeature.bodies.count} bodies, want 1')
        self.cageBody = cutFeature.bodies.item(0)

        afterContainment = self.cageBody.pointContainment(_pt3(probeWorld))
        if afterContainment != adsk.fusion.PointContainment.PointOutsidePointContainment:
            raise Exception(
                f'ScrewGear: window {facing} probe reads {afterContainment} after the cut, '
                f'want outside')

    # -----------------------------------------------------------------
    # S27: relocate the finished bodies and hide the construction geometry.
    # -----------------------------------------------------------------

    def relocateBodies(self):
        self.cageBody.name = 'Cage'
        gearBodyA: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[0])
        gearBodyA.moveToComponent(self.gearOccs[0])
        gearBodyB: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.gearBodies[1])
        gearBodyB.moveToComponent(self.gearOccs[1])
        self.cageBody.moveToComponent(self.cageOcc)
        solids.hide_construction_geometry(self.designOcc.component)
