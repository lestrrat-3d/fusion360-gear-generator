import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, get_value, get_selection

# ---------------------------------------------------------------------------
# Dialog input ids
# ---------------------------------------------------------------------------
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'
INPUT_ID_DISC_COUNT = 'discCount'
INPUT_ID_PIN_COUNT = 'pinCount'
INPUT_ID_PIN_CIRCLE_DIAMETER = 'pinCircleDiameter'
INPUT_ID_PIN_DIAMETER = 'pinDiameter'
INPUT_ID_ECCENTRICITY = 'eccentricity'
INPUT_ID_DISK_CLEARANCE = 'diskClearance'
INPUT_ID_DISC_THICKNESS = 'discThickness'
INPUT_ID_DISC_GAP = 'discGap'
INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER = 'outputPinCircleDiameter'
INPUT_ID_OUTPUT_PIN_COUNT = 'outputPinCount'
INPUT_ID_OUTPUT_PIN_DIAMETER = 'outputPinDiameter'
INPUT_ID_CENTER_BEARING_DIAMETER = 'centerBearingDiameter'
INPUT_ID_INPUT_SHAFT_DIAMETER = 'inputShaftDiameter'
INPUT_ID_BEARING_CLEARANCE = 'bearingClearance'
INPUT_ID_WALL = 'wall'
INPUT_ID_BASE_THICKNESS = 'baseThickness'
INPUT_ID_OUTPUT_PLATE_THICKNESS = 'outputPlateThickness'
INPUT_ID_CHAMFER_SIZE = 'chamferSize'

# ---------------------------------------------------------------------------
# User-parameter names (the <prefix>_<name> table)
# ---------------------------------------------------------------------------
PARAM_PIN_COUNT = 'PinCount'
PARAM_PIN_CIRCLE_DIAMETER = 'PinCircleDiameter'
PARAM_PIN_DIAMETER = 'PinDiameter'
PARAM_ECCENTRICITY = 'Eccentricity'
PARAM_DISK_CLEARANCE = 'DiskClearance'
PARAM_DISC_THICKNESS = 'DiscThickness'
PARAM_DISC_GAP = 'DiscGap'
PARAM_OUTPUT_PIN_CIRCLE_DIAMETER = 'OutputPinCircleDiameter'
PARAM_OUTPUT_PIN_COUNT = 'OutputPinCount'
PARAM_OUTPUT_PIN_DIAMETER = 'OutputPinDiameter'
PARAM_CENTER_BEARING_DIAMETER = 'CenterBearingDiameter'
PARAM_INPUT_SHAFT_DIAMETER = 'InputShaftDiameter'
PARAM_BEARING_CLEARANCE = 'BearingClearance'
PARAM_WALL = 'Wall'
PARAM_BASE_THICKNESS = 'BaseThickness'
PARAM_OUTPUT_PLATE_THICKNESS = 'OutputPlateThickness'
PARAM_CHAMFER_SIZE = 'ChamferSize'

# Derived parameter names
PARAM_LOBES = 'Lobes'
PARAM_PIN_CIRCLE_RADIUS = 'PinCircleRadius'
PARAM_OUTPUT_PIN_CIRCLE_RADIUS = 'OutputPinCircleRadius'
PARAM_PIN_RADIUS = 'PinRadius'
PARAM_OUTPUT_HOLE_DIAMETER = 'OutputHoleDiameter'
PARAM_HOUSING_INNER_DIAMETER = 'HousingInnerDiameter'
PARAM_HOUSING_OUTER_DIAMETER = 'HousingOuterDiameter'
PARAM_OUTPUT_PLATE_DIAMETER = 'OutputPlateDiameter'


class CycloidalDriveCommandInputsConfigurator:
    """Adds the dialog inputs, in the fixed display order, to the command."""

    @classmethod
    def configure(cls, command):
        inputs = command.commandInputs
        design = get_design()

        # 1. Target Plane
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane on which the rotor lobe is sketched')
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the drive axis is aligned with')
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Disc Count (dropdown 1/2)
        discCountInput = inputs.addDropDownCommandInput(
            INPUT_ID_DISC_COUNT, 'Disc Count',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        discCountInput.listItems.add('1', True)
        discCountInput.listItems.add('2', False)

        # 4. Pin Count
        inputs.addValueInput(
            INPUT_ID_PIN_COUNT, 'Pin Count', '',
            adsk.core.ValueInput.createByReal(16))

        # 5. Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_PIN_CIRCLE_DIAMETER, 'Pin Circle Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(90)))

        # 6. Pin Diameter (0 = auto)
        inputs.addValueInput(
            INPUT_ID_PIN_DIAMETER, 'Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 7. Eccentricity
        inputs.addValueInput(
            INPUT_ID_ECCENTRICITY, 'Eccentricity', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(1.5)))

        # 8. Disk Clearance
        inputs.addValueInput(
            INPUT_ID_DISK_CLEARANCE, 'Disk Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.3)))

        # 9. Disc Thickness
        inputs.addValueInput(
            INPUT_ID_DISC_THICKNESS, 'Disc Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))

        # 10. Disc Gap
        inputs.addValueInput(
            INPUT_ID_DISC_GAP, 'Disc Gap', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.5)))

        # 11. Center Bearing Diameter
        inputs.addValueInput(
            INPUT_ID_CENTER_BEARING_DIAMETER, 'Center Bearing Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(30)))

        # 12. Input Shaft Diameter (0 = no bore)
        inputs.addValueInput(
            INPUT_ID_INPUT_SHAFT_DIAMETER, 'Input Shaft Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))

        # 13. Bearing Clearance
        inputs.addValueInput(
            INPUT_ID_BEARING_CLEARANCE, 'Bearing Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.2)))

        # 14. Output Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'Output Pin Circle Diameter',
            'mm', adsk.core.ValueInput.createByReal(to_cm(50)))

        # 15. Output Pin Count
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', '',
            adsk.core.ValueInput.createByReal(6))

        # 16. Output Pin Diameter (0 = auto)
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 17. Housing Wall
        inputs.addValueInput(
            INPUT_ID_WALL, 'Housing Wall', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(3)))

        # 18. Base Thickness
        inputs.addValueInput(
            INPUT_ID_BASE_THICKNESS, 'Base Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))

        # 19. Output Plate Thickness
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PLATE_THICKNESS, 'Output Plate Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))

        # 20. Chamfer Size
        inputs.addValueInput(
            INPUT_ID_CHAMFER_SIZE, 'Chamfer Size', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.5)))

        # 21. Parent Component (last; defaults to the root component)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the new drive occurrence is nested under')
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(0, 1)
        parentInput.addSelection(design.rootComponent)


class CycloidalDriveGenerator(Generator):
    """Generates the cycloidal drive: rotor disc(s) + center bore, output holes,
    Housing (base annulus + pinless ring casing), eccentric input cam, and the
    output plate + output pins. Subclasses base.Generator directly; shares no
    involute math."""

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        # Selection inputs, stashed before any occurrence creation.
        self.plane = None
        self.anchorPoint = None
        # Per-disc handles are LISTS indexed by disc index d.
        self.diskBodies = []
        self.diskAxes = []
        self.lobeSplines = []
        self.outputHoles = []
        self.lobeDiskCentres = []
        self.discPlanes = []
        # Scalars.
        self.lobePinCircle = None
        self.driveAxis = None
        self.housingRing = None
        self.ringCasing = None
        self.cam = None
        self.outputPlate = None
        self._normalizedPlane = None
        # Python-precomputed dimensions (internal cm) + counts.
        self.discCount = 1
        self.pinCountInt = 16
        self.outputPinCountInt = 6
        self.lobesInt = 15
        self.R = 0.0
        self.Rop = 0.0
        self.Rr = 0.0
        self.c = 0.0
        self.E = 0.0
        self.RrEff = 0.0
        self.Rv = 0.0
        self.Dhole = 0.0
        self.Dpin = 0.0
        self.wall = 0.0
        self.chamferSize = 0.0
        self.discThickness = 0.0

    # -- subclass hooks ------------------------------------------------------
    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    def generateName(self):
        N = self.pinCountInt
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    # =======================================================================
    # Geometry oracle (epitrochoid-trace.md). All values internal cm.
    # =======================================================================
    def disk_point(self, t, cx, cy, phi):
        """Equidistant of a shortened epitrochoid. Returns (x, y) in cm."""
        R = self.R
        E = self.E
        N = self.pinCountInt
        RrEff = self.RrEff
        num = math.sin((1 - N) * t)
        den = (R / (E * N)) - math.cos((1 - N) * t)
        psi = math.atan2(num, den)            # uses R, E, N only — NOT Rr
        x0 = R * math.cos(t) - RrEff * math.cos(t + psi) - E * math.cos(N * t)
        y0 = -R * math.sin(t) + RrEff * math.sin(t + psi) + E * math.sin(N * t)
        x = cx + (x0 * math.cos(phi) - y0 * math.sin(phi))
        y = cy + (x0 * math.sin(phi) + y0 * math.cos(phi))
        return (x, y)

    def _rho_min_O(self):
        """Numeric smallest radius of curvature of the base trochoid at points
        whose centre of curvature lies toward O. Returns rho_min^O (cm)."""
        R = self.R
        E = self.E
        N = self.pinCountInt
        best = None
        steps = 2000
        for i in range(steps):
            t = 2.0 * math.pi * i / steps
            bx = R * math.cos(t) - E * math.cos(N * t)
            by = -R * math.sin(t) + E * math.sin(N * t)
            xp = -R * math.sin(t) + E * N * math.sin(N * t)
            yp = -R * math.cos(t) + E * N * math.cos(N * t)
            xpp = -R * math.cos(t) + E * N * N * math.cos(N * t)
            ypp = R * math.sin(t) - E * N * N * math.sin(N * t)
            k = xp * ypp - yp * xpp
            if abs(k) < 1e-9:
                continue
            s2 = xp * xp + yp * yp
            rho = (s2 ** 1.5) / k
            s = math.sqrt(s2)
            nx, ny = -yp / s, xp / s
            cx = bx + rho * nx
            cy = by + rho * ny
            if (cx * cx + cy * cy) < (bx * bx + by * by):
                arho = abs(rho)
                if best is None or arho < best:
                    best = arho
        return best

    def _sample_lobe(self, cx, cy, phi):
        """Adaptive (bounded-turn-angle) sampling of one lobe, t in [0, 2pi/L].
        Returns a list of (x, y) cm points; first/last land exactly on
        t=0 and t=2pi/L."""
        L = self.lobesInt
        tmax = 2.0 * math.pi / L
        fine = 2000
        pts = []
        for i in range(fine + 1):
            t = tmax * i / fine
            pts.append(self.disk_point(t, cx, cy, phi))

        threshold = math.radians(5.0)
        kept = [pts[0]]
        accum = 0.0
        prevDir = None
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            seg = math.hypot(dx, dy)
            if seg < 1e-12:
                continue
            curDir = math.atan2(dy, dx)
            if prevDir is not None:
                d = curDir - prevDir
                while d > math.pi:
                    d -= 2.0 * math.pi
                while d < -math.pi:
                    d += 2.0 * math.pi
                accum += abs(d)
                if accum >= threshold and i != len(pts) - 1:
                    kept.append(pts[i])
                    accum = 0.0
            prevDir = curDir
        kept.append(pts[-1])
        return kept

    def _contour_section(self):
        """One pin-pitch of the swept-envelope contour env(phi)+c, over
        phi in [-pi/N, pi/N]. Returns nbins+1 (x, y) cm points at bin EDGES,
        ordered by angle, first on -pi/N and last on +pi/N."""
        N = self.pinCountInt
        L = self.lobesInt
        E = self.E
        c = self.c
        Ntheta = 240
        Nt = 240
        nbins = 72
        lo = -math.pi / N
        hi = math.pi / N
        span = hi - lo
        binMax = [0.0] * nbins
        for it in range(Ntheta):
            theta = 2.0 * math.pi * it / Ntheta
            cx = E * math.cos(theta)
            cy = E * math.sin(theta)
            phiRot = -theta / L
            for jt in range(Nt):
                tt = 2.0 * math.pi * jt / Nt
                x, y = self.disk_point(tt, cx, cy, phiRot)
                a = math.atan2(y, x)
                if a < lo or a > hi:
                    continue
                r = math.hypot(x, y)
                b = int((a - lo) / span * nbins)
                if b < 0:
                    b = 0
                if b >= nbins:
                    b = nbins - 1
                if r > binMax[b]:
                    binMax[b] = r
        points = []
        for i in range(nbins + 1):
            phi_i = lo + (hi - lo) * i / nbins
            if i == 0:
                rad = c + binMax[0]
            elif i == nbins:
                rad = c + binMax[nbins - 1]
            else:
                rad = c + max(binMax[i - 1], binMax[i])
            points.append((rad * math.cos(phi_i), rad * math.sin(phi_i)))
        return points

    # =======================================================================
    # input reading
    # =======================================================================
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # 1. Pull every selection out FIRST (before occurrence creation).
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one Parent Component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent

        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one Target Plane must be selected')
        self.plane = planes[0]

        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception('Exactly one Anchor Point must be selected')
        self.anchorPoint = anchors[0]

        # Disc Count dropdown -> int (read before occurrence creation as well;
        # selection-shaped, not a get_value).
        discItem = inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem
        self.discCount = int(discItem.name)

        # 2. Register the input-sourced primary parameters. The first
        # addParameter call creates the occurrence (context shift), but the
        # selections are already stashed on self.
        self.addParameter(
            PARAM_PIN_COUNT, get_value(inputs, INPUT_ID_PIN_COUNT, ''),
            '', 'Number of ring pins (N); Lobes = N - 1')
        self.addParameter(
            PARAM_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Pin circle (pitch) diameter')
        self.addParameter(
            PARAM_PIN_DIAMETER, get_value(inputs, INPUT_ID_PIN_DIAMETER, 'mm'),
            'mm', 'Ring-pin diameter (0 = auto)')
        self.addParameter(
            PARAM_ECCENTRICITY, get_value(inputs, INPUT_ID_ECCENTRICITY, 'mm'),
            'mm', 'Eccentricity (E)')
        self.addParameter(
            PARAM_DISK_CLEARANCE,
            get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'),
            'mm', 'Backlash clearance cut into the profile')
        self.addParameter(
            PARAM_DISC_THICKNESS,
            get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'),
            'mm', 'Axial thickness of each rotor disk')
        self.addParameter(
            PARAM_DISC_GAP, get_value(inputs, INPUT_ID_DISC_GAP, 'mm'),
            'mm', 'Axial clearance between disc 1 and disc 2')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Output pin circle diameter')
        self.addParameter(
            PARAM_OUTPUT_PIN_COUNT,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, ''),
            '', 'Number of output pins/holes (M)')
        self.addParameter(
            PARAM_OUTPUT_PIN_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'mm'),
            'mm', 'Output-pin diameter (0 = auto)')
        self.addParameter(
            PARAM_CENTER_BEARING_DIAMETER,
            get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'),
            'mm', 'Disk center-bore = cam outer diameter')
        self.addParameter(
            PARAM_INPUT_SHAFT_DIAMETER,
            get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'),
            'mm', 'Input-shaft bore through the cam (0 = none)')
        self.addParameter(
            PARAM_BEARING_CLEARANCE,
            get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'),
            'mm', 'Diametral running clearance of the disk bore on the cam')
        self.addParameter(
            PARAM_WALL, get_value(inputs, INPUT_ID_WALL, 'mm'),
            'mm', 'Radial wall thickness of the pinless casing')
        self.addParameter(
            PARAM_BASE_THICKNESS,
            get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the ring-pin base (Housing Ring)')
        self.addParameter(
            PARAM_OUTPUT_PLATE_THICKNESS,
            get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the output plate')
        self.addParameter(
            PARAM_CHAMFER_SIZE, get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'),
            'mm', '45-degree equal-distance chamfer (0 = none)')

        # 3. Resolve the Python-side numeric values (internal cm).
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        self.pinCountInt = N
        self.outputPinCountInt = M
        self.lobesInt = N - 1

        R = self.getParameter(PARAM_PIN_CIRCLE_DIAMETER).value / 2.0
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2.0
        E = self.getParameter(PARAM_ECCENTRICITY).value
        c = self.getParameter(PARAM_DISK_CLEARANCE).value
        pinDiameter = self.getParameter(PARAM_PIN_DIAMETER).value
        outputPinDiameter = self.getParameter(PARAM_OUTPUT_PIN_DIAMETER).value
        centerBearingD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        inputShaftD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        bearingClr = self.getParameter(PARAM_BEARING_CLEARANCE).value
        wall = self.getParameter(PARAM_WALL).value

        self.R = R
        self.Rop = Rop
        self.E = E
        self.c = c
        self.wall = wall
        self.chamferSize = self.getParameter(PARAM_CHAMFER_SIZE).value
        self.discThickness = self.getParameter(PARAM_DISC_THICKNESS).value

        # Ring-pin radius (auto-vs-override).
        if pinDiameter > 0:
            Rr = pinDiameter / 2.0
        else:
            Rr = 0.5 * (E + R * math.sin(math.pi / N))
        self.Rr = Rr
        self.RrEff = Rr + c
        self.Rv = R - self.RrEff - E

        # Output-pin diameter + hole (auto-vs-override).
        if outputPinDiameter > 0:
            Dpin = outputPinDiameter
        else:
            Dpin = Rop * math.sin(math.pi / M) - E
        Dhole = Dpin + 2.0 * E
        self.Dpin = Dpin
        self.Dhole = Dhole

        # 4. Validity (on the resolved values; reject with a clear message).
        if self.discCount == 2 and (N % 2 != 0 or M % 2 != 0):
            raise Exception(
                'Two discs require an even Pin Count and even Output Pin Count')
        if not (E < Rr < R * math.sin(math.pi / N)):
            raise Exception(
                'Invalid Pin Diameter: require E < PinRadius < R*sin(pi/N) '
                '(E={:.3f} mm, PinRadius={:.3f} mm, bound={:.3f} mm)'.format(
                    E * 10, Rr * 10, R * math.sin(math.pi / N) * 10))
        if Dpin <= 0:
            raise Exception(
                'Invalid Output Pin Diameter: resolved D_pin must be > 0')
        if not (Dhole < 2.0 * Rop * math.sin(math.pi / M)):
            raise Exception(
                'Invalid output hole: the M holes overlap '
                '(D_hole >= 2*Rop*sin(pi/M))')
        if not (E < R / N):
            raise Exception(
                'Eccentricity too large: require E < R/N')
        if not (Rop < self.Rv):
            raise Exception(
                'Output Pin Circle Diameter too large: require Rop < Rv')
        rhoMinO = self._rho_min_O()
        if rhoMinO is not None and self.RrEff >= rhoMinO:
            # Estimate the safe E limit for the message (Rr_eff ~ Rr + c at the
            # boundary; report the curvature ceiling on the offset).
            raise Exception(
                'Eccentricity causes undercut: the offset profile '
                'self-intersects (Rr_eff={:.3f} mm >= rho_min^O={:.3f} mm). '
                'Reduce Eccentricity.'.format(self.RrEff * 10, rhoMinO * 10))
        if not (inputShaftD < centerBearingD):
            raise Exception(
                'Input Shaft Diameter must be smaller than '
                'Center Bearing Diameter')
        if not (E + inputShaftD / 2.0 < centerBearingD / 2.0):
            raise Exception(
                'Input bore does not fit inside the cam '
                '(E + InputShaftDiameter/2 >= CenterBearingDiameter/2)')
        if not ((centerBearingD + bearingClr) / 2.0 < Rop - Dhole / 2.0):
            raise Exception(
                'Enlarged disk center bore overlaps the output holes')

        # 5. Derived parameters. PinRadius and OutputHoleDiameter MUST be
        # createByReal numeric snapshots (resolved per the auto/override
        # branch); the rest stay live createByString expressions.
        self.addParameter(
            PARAM_LOBES,
            adsk.core.ValueInput.createByString(
                '{} - 1'.format(self.parameterName(PARAM_PIN_COUNT))),
            '', 'Number of rotor lobes (N - 1)')
        self.addParameter(
            PARAM_PIN_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_DIAMETER))),
            'mm', 'Pin circle radius (R)')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(
                    self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER))),
            'mm', 'Output pin circle radius (Rop)')
        self.addParameter(
            PARAM_PIN_RADIUS,
            adsk.core.ValueInput.createByReal(Rr),
            'mm', 'Resolved ring-pin radius (numeric snapshot)')
        self.addParameter(
            PARAM_OUTPUT_HOLE_DIAMETER,
            adsk.core.ValueInput.createByReal(Dhole),
            'mm', 'Resolved output-hole diameter (numeric snapshot)')
        self.addParameter(
            PARAM_HOUSING_INNER_DIAMETER,
            adsk.core.ValueInput.createByString(
                '2 * ({} - {} - {})'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
                    self.parameterName(PARAM_PIN_RADIUS),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Housing inner annulus diameter')
        self.addParameter(
            PARAM_HOUSING_OUTER_DIAMETER,
            adsk.core.ValueInput.createByString(
                '2 * ({} - {} + 2 * {} + {})'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
                    self.parameterName(PARAM_PIN_RADIUS),
                    self.parameterName(PARAM_ECCENTRICITY),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Housing outer annulus diameter')
        self.addParameter(
            PARAM_OUTPUT_PLATE_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} + ({} - 2 * {}) + 2 * {}'.format(
                    self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER),
                    self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
                    self.parameterName(PARAM_ECCENTRICITY),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Output plate diameter')

    # =======================================================================
    # orchestration
    # =======================================================================
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalize the plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(
                self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self._normalizedPlane = self.plane

        D = self.discCount

        # Allocate the per-disc lists.
        self.diskBodies = [None] * D
        self.diskAxes = [None] * D
        self.lobeSplines = [None] * D
        self.outputHoles = [None] * D
        self.lobeDiskCentres = [None] * D
        self.discPlanes = [None] * D

        for d in range(D):
            self.buildLobeSketch(d)
            self.buildDisk(d)
            self.buildOutputHoleSketch(d)
            self.buildOutputHoles(d)
            self.buildDiskBore(d)

        self.buildCam()
        self.buildRingPins()
        self.buildOutputPins()
        self.buildChamfers()

    # =======================================================================
    # §0 helpers — per-disc plane / centre / clocking
    # =======================================================================
    def _planeForDisc(self, d):
        """plane(d): self.plane for d==0, else a construction plane offset
        z_d = d*(T+g) using PREFIXED param names."""
        if d == 0:
            return self.plane
        component = self.getComponent()
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane,
            adsk.core.ValueInput.createByString(
                '{} * ({} + {})'.format(d, nT, nG)))
        plane = component.constructionPlanes.add(pi)
        plane.name = 'Disc Plane {}'.format(d + 1)
        return plane

    def _stackTopExpr(self):
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        if self.discCount == 1:
            return nT
        return '2 * {} + {}'.format(nT, nG)

    def _anchorLocalOrigin(self, sketch):
        """[CYCLOIDAL-F-ANCHOR-CHAIN]: project the user Anchor and coincide a
        fresh local origin to it. Returns the local origin SketchPoint."""
        localOrigin = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))
        projected = sketch.project(self.anchorPoint)
        projectedPoint = projected.item(0)
        sketch.geometricConstraints.addCoincident(projectedPoint, localOrigin)
        return localOrigin

    def _buildDiskCentre(self, sketch, localOrigin, signedE):
        """[CYCLOIDAL-F-DISK-CENTER]: a point at (signedE, 0) tied to the
        Eccentricity parameter via a driving distance dim. Returns diskCentre."""
        diskCentre = sketch.sketchPoints.add(
            adsk.core.Point3D.create(signedE, 0, 0))
        eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        textPoint = adsk.core.Point3D.create(signedE / 2.0, 0, 0)
        distDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            textPoint)
        distDim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return diskCentre

    def _alongPathLabel(self, sketch, circle, text):
        textInput = sketch.sketchTexts.createInput2(text, self.Rr)
        textInput.setAsAlongPath(
            circle, True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(textInput)

    def _profileContaining(self, sketch, entity):
        """Return the profile whose loop contains the given sketch entity by
        identity match. Raises if not found."""
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == entity:
                        return profile
        raise Exception(
            'Could not find profile containing entity in sketch "{}"'.format(
                sketch.name))

    def _smallestProfileContaining(self, sketch, entity):
        """Among profiles whose loop contains the entity, return the one with
        the smallest area. Raises if none found ([PB-EMPTY-RESULT])."""
        best = None
        bestArea = None
        for profile in sketch.profiles:
            contains = False
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == entity:
                        contains = True
                        break
                if contains:
                    break
            if not contains:
                continue
            area = profile.areaProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area
            if bestArea is None or area < bestArea:
                bestArea = area
                best = profile
        if best is None:
            raise Exception(
                'No profile containing the contour spline in sketch "{}"'.format(
                    sketch.name))
        return best

    def _profileByLoopCount(self, sketch, count):
        """Return the profile with exactly `count` profile loops (annulus = 2).
        Raises if none ([PB-EMPTY-RESULT])."""
        for profile in sketch.profiles:
            if profile.profileLoops.count == count:
                return profile
        raise Exception(
            'No profile with {} loop(s) in sketch "{}"'.format(
                count, sketch.name))

    def _profileSingleLoopContaining(self, sketch, entity):
        """Return the profile whose single (1-loop) profile is bounded by the
        given entity. Raises if none."""
        for profile in sketch.profiles:
            if profile.profileLoops.count != 1:
                continue
            loop = profile.profileLoops.item(0)
            for pc in loop.profileCurves:
                if pc.sketchEntity == entity:
                    return profile
        raise Exception(
            'No 1-loop profile bounded by the pin in sketch "{}"'.format(
                sketch.name))

    # =======================================================================
    # §2: buildLobeSketch(d) — the fully-constrained open lobe + frame
    # =======================================================================
    def buildLobeSketch(self, d):
        plane = self._planeForDisc(d)
        self.discPlanes[d] = plane

        sketch = self.createSketchObject('Rotor Lobe {}'.format(d + 1), plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        s_d = 1.0 if d == 0 else -1.0
        signedE = s_d * self.E
        phi = d * math.pi
        diskCentre = self._buildDiskCentre(sketch, localOrigin, signedE)

        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        R = self.R
        Rop = self.Rop
        Rv = self.Rv

        # 1. pin circle (construction, on O) — stash for d == 0 only.
        pinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), R)
        pinCircle.isConstruction = True
        cons.addCoincident(pinCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            pinCircle, adsk.core.Point3D.create(R, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, pinCircle, 'Pin Circle')
        if d == 0:
            self.lobePinCircle = pinCircle

        # 2. output-pin circle (construction, on Od).
        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rop)
        outPinCircle.isConstruction = True
        cons.addCoincident(outPinCircle.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            outPinCircle,
            adsk.core.Point3D.create(signedE + Rop, 0, 0)
        ).parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, outPinCircle, 'Output Pin Circle')

        # 3. root/valley circle (construction, on Od).
        rootCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rv)
        rootCircle.isConstruction = True
        cons.addCoincident(rootCircle.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            rootCircle,
            adsk.core.Point3D.create(signedE + Rv, 0, 0)
        ).parameter.expression = '2 * ({} - {} - {} - {})'.format(
            self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
            self.parameterName(PARAM_PIN_RADIUS),
            self.parameterName(PARAM_DISK_CLEARANCE),
            self.parameterName(PARAM_ECCENTRICITY))
        self._alongPathLabel(sketch, rootCircle, 'Root Circle')

        # 4. lobe spline (open, adaptively sampled, about Od).
        kept = self._sample_lobe(signedE, 0.0, phi)
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in kept:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline = sketch.sketchCurves.sketchFittedSplines.add(coll)
        startPt = spline.fitPoints.item(0)
        endPt = spline.fitPoints.item(spline.fitPoints.count - 1)

        # 5. lock the spline: fix interior points, coincide ends on root circle.
        for i in range(1, spline.fitPoints.count - 1):
            spline.fitPoints.item(i).isFixed = True
        cons.addCoincident(startPt, rootCircle)
        cons.addCoincident(endPt, rootCircle)

        # 6. spoke line 1 from Od to the lobe's first point + horizontal.
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(signedE + Rv, 0, 0))
        cons.addCoincident(line1.endSketchPoint, startPt)
        cons.addHorizontal(line1)

        # 7. spoke line 2 from Od to the lobe's last point.
        L = self.lobesInt
        # End valley local coords about Od (disc 0 frame), then clocked by phi.
        ex0 = Rv * math.cos(2.0 * math.pi / L)
        ey0 = -Rv * math.sin(2.0 * math.pi / L)
        ex = signedE + (ex0 * math.cos(phi) - ey0 * math.sin(phi))
        ey = (ex0 * math.sin(phi) + ey0 * math.cos(phi))
        line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(ex, ey, 0))
        cons.addCoincident(line2.endSketchPoint, endPt)

        # 8. lobe-pitch angle dim (driving), text in the minor wedge.
        bisAngle = phi - math.pi / L
        textPoint = adsk.core.Point3D.create(
            signedE + 0.4 * Rv * math.cos(bisAngle),
            0.4 * Rv * math.sin(bisAngle), 0)
        angDim = dims.addAngularDimension(line1, line2, textPoint)
        angDim.parameter.expression = '360 deg / {}'.format(
            self.parameterName(PARAM_LOBES))

        self.lobeDiskCentres[d] = diskCentre
        self.lobeSplines[d] = spline

    # =======================================================================
    # §2·A: buildDisk(d) — extrude sector, axis, pattern, join
    # =======================================================================
    def buildDisk(self, d):
        component = self.getComponent()
        sketch = self.lobeSplines[d].parentSketch
        L = self.lobesInt

        # Record the body baseline BEFORE the extrude (join only this disc's).
        base = component.bRepBodies.count

        # 1. Extrude the lobe sector (selected by the loop containing the spline).
        sectorProfile = self._profileContaining(sketch, self.lobeSplines[d])
        extrudes = component.features.extrudeFeatures
        ext = extrudes.createInput(
            sectorProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(ext)
        extrude.bodies.item(0).name = 'Cycloidal Disk {}'.format(d + 1)

        # 2. Disk axis from a planar cap face by NORMAL.
        self.buildDiskAxis(extrude.startFaces.item(0), d)

        # 3. Circular-pattern the extrude FEATURE x L about the disk axis.
        coll = adsk.core.ObjectCollection.create()
        coll.add(extrude)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll, self.diskAxes[d])
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(L)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

        # 4. Join only this disc's OWN L sectors.
        target = component.bRepBodies.item(base)
        tools = adsk.core.ObjectCollection.create()
        for i in range(base + 1, base + L):
            tools.add(component.bRepBodies.item(i))
        combines = component.features.combineFeatures
        ci = combines.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci)
        target.name = 'Cycloidal Disk {}'.format(d + 1)
        self.diskBodies[d] = target

    def buildDiskAxis(self, capFace, d):
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
        axis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis {}'.format(d + 1)
        self.diskAxes[d] = axis

    # =======================================================================
    # §3: buildOutputHoleSketch(d)
    # =======================================================================
    def buildOutputHoleSketch(self, d):
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Output Hole {}'.format(d + 1), plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        s_d = 1.0 if d == 0 else -1.0
        signedE = s_d * self.E
        diskCentre = self._buildDiskCentre(sketch, localOrigin, signedE)

        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints
        Rop = self.Rop
        Dhole = self.Dhole

        # Output-hole circle (construction, on Od).
        outHoleCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rop)
        outHoleCircle.isConstruction = True
        cons.addCoincident(outHoleCircle.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            outHoleCircle,
            adsk.core.Point3D.create(signedE + Rop, 0, 0)
        ).parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, outHoleCircle, 'Output Hole Circle')

        # One solid hole on the Rop circle.
        hole = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE + Rop, 0, 0), Dhole / 2.0)
        dims.addDiameterDimension(
            hole,
            adsk.core.Point3D.create(signedE + Rop + Dhole / 2.0, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        cons.addCoincident(hole.centerSketchPoint, outHoleCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        spoke.isConstruction = True
        cons.addHorizontal(spoke)

        self.outputHoles[d] = hole

    # =======================================================================
    # §4: buildOutputHoles(d) — cut + pattern x M
    # =======================================================================
    def buildOutputHoles(self, d):
        component = self.getComponent()
        sketch = self.outputHoles[d].parentSketch
        M = self.outputPinCountInt

        holeProfile = self._profileContaining(sketch, self.outputHoles[d])
        extrudes = component.features.extrudeFeatures
        ci = extrudes.createInput(
            holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        cut = extrudes.add(ci)

        coll = adsk.core.ObjectCollection.create()
        coll.add(cut)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll, self.diskAxes[d])
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

    # =======================================================================
    # §6: buildDiskBore(d) — center bore cut
    # =======================================================================
    def buildDiskBore(self, d):
        component = self.getComponent()
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Disc Bore {}'.format(d + 1), plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        s_d = 1.0 if d == 0 else -1.0
        signedE = s_d * self.E
        diskCentre = self._buildDiskCentre(sketch, localOrigin, signedE)

        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        boreRadius = (self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value +
                      self.getParameter(PARAM_BEARING_CLEARANCE).value) / 2.0
        bore = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), boreRadius)
        cons.addCoincident(bore.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            bore,
            adsk.core.Point3D.create(signedE + boreRadius, 0, 0)
        ).parameter.expression = '{} + {}'.format(
            self.parameterName(PARAM_CENTER_BEARING_DIAMETER),
            self.parameterName(PARAM_BEARING_CLEARANCE))

        coll = adsk.core.ObjectCollection.create()
        for profile in sketch.profiles:
            coll.add(profile)
        extrudes = component.features.extrudeFeatures
        ci = extrudes.createInput(
            coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        extrudes.add(ci)

    # =======================================================================
    # §6: buildCam() — D eccentric sections joined into one Eccentric Cam
    # =======================================================================
    def buildCam(self):
        component = self.getComponent()
        D = self.discCount
        inputShaftD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        centerBearingD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)

        sectionBodies = []
        for d in range(D):
            plane = self.discPlanes[d]
            sketch = self.createSketchObject(
                'Eccentric Cam {}'.format(d + 1), plane)
            sketch.isVisible = True

            localOrigin = self._anchorLocalOrigin(sketch)
            s_d = 1.0 if d == 0 else -1.0
            signedE = s_d * self.E
            diskCentre = self._buildDiskCentre(sketch, localOrigin, signedE)

            circles = sketch.sketchCurves.sketchCircles
            dims = sketch.sketchDimensions
            cons = sketch.geometricConstraints

            camOuter = circles.addByCenterRadius(
                adsk.core.Point3D.create(signedE, 0, 0), centerBearingD / 2.0)
            cons.addCoincident(camOuter.centerSketchPoint, diskCentre)
            dims.addDiameterDimension(
                camOuter,
                adsk.core.Point3D.create(signedE + centerBearingD / 2.0, 0, 0)
            ).parameter.expression = self.parameterName(
                PARAM_CENTER_BEARING_DIAMETER)

            if inputShaftD > 0:
                inputBore = circles.addByCenterRadius(
                    adsk.core.Point3D.create(0, 0, 0), inputShaftD / 2.0)
                cons.addCoincident(inputBore.centerSketchPoint, localOrigin)
                dims.addDiameterDimension(
                    inputBore,
                    adsk.core.Point3D.create(inputShaftD / 2.0, 0, 0)
                ).parameter.expression = self.parameterName(
                    PARAM_INPUT_SHAFT_DIAMETER)
                profile = self._profileByLoopCount(sketch, 2)
            else:
                profile = sketch.profiles.item(0)

            if d < D - 1:
                extentExpr = '{} + {}'.format(nT, nG)
            else:
                extentExpr = nT
            extrudes = component.features.extrudeFeatures
            ext = extrudes.createInput(
                profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            ext.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(
                    adsk.core.ValueInput.createByString(extentExpr)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection)
            extrude = extrudes.add(ext)
            body = extrude.bodies.item(0)
            body.name = 'Eccentric Cam'
            sectionBodies.append(body)

        # Join the D sections into one.
        cam = sectionBodies[0]
        if len(sectionBodies) > 1:
            tools = adsk.core.ObjectCollection.create()
            for b in sectionBodies[1:]:
                tools.add(b)
            combines = component.features.combineFeatures
            ci = combines.createInput(cam, tools)
            ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combines.add(ci)
        cam.name = 'Eccentric Cam'
        self.cam = cam

    # =======================================================================
    # §5: buildRingPins() — Housing base + pinless ring casing
    # =======================================================================
    def buildRingPins(self):
        component = self.getComponent()
        N = self.pinCountInt
        R = self.R
        Rr = self.Rr
        E = self.E
        wall = self.wall

        # 1. Housing plane, 1 mm below the disc.
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane = component.constructionPlanes.add(pi)
        housingPlane.name = 'Ring Housing Plane'

        # 2. Housing base — sketch Housing Ring (a plain annulus).
        baseSketch = self.createSketchObject('Housing Ring', housingPlane)
        baseSketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(baseSketch)
        circles = baseSketch.sketchCurves.sketchCircles
        dims = baseSketch.sketchDimensions
        cons = baseSketch.geometricConstraints

        outerR = R - Rr + 2.0 * E + wall
        innerR = R - Rr - wall
        outerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        cons.addCoincident(outerCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            outerCircle, adsk.core.Point3D.create(outerR, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)
        innerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), innerR)
        cons.addCoincident(innerCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            innerCircle, adsk.core.Point3D.create(innerR, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_HOUSING_INNER_DIAMETER)

        annulusProfile = self._profileByLoopCount(baseSketch, 2)
        extrudes = component.features.extrudeFeatures
        ext = extrudes.createInput(
            annulusProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_BASE_THICKNESS))),
            adsk.fusion.ExtentDirections.NegativeExtentDirection)
        housingExtrude = extrudes.add(ext)
        self.housingRing = housingExtrude.bodies.item(0)
        self.housingRing.name = 'Housing Ring'

        # 3. Drive axis at O, from the Housing-Ring cap face by NORMAL.
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(
            housingExtrude.startFaces.item(0), localOrigin)
        self.driveAxis = component.constructionAxes.add(axInput)
        self.driveAxis.name = 'Drive Axis'

        # 4. Ring casing — one section patterned x N.
        contourPts = self._contour_section()
        casingSketch = self.createSketchObject('Ring Casing', self.plane)
        casingSketch.isVisible = True
        casingOrigin = self._anchorLocalOrigin(casingSketch)
        cCircles = casingSketch.sketchCurves.sketchCircles
        cDims = casingSketch.sketchDimensions
        cCons = casingSketch.geometricConstraints

        casingOuter = cCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        cCons.addCoincident(casingOuter.centerSketchPoint, casingOrigin)
        cDims.addDiameterDimension(
            casingOuter, adsk.core.Point3D.create(outerR, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)

        coll = adsk.core.ObjectCollection.create()
        for (x, y) in contourPts:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        contour = casingSketch.sketchCurves.sketchFittedSplines.add(coll)
        contourStart = contour.fitPoints.item(0)
        contourEnd = contour.fitPoints.item(contour.fitPoints.count - 1)

        # Two radial spokes from each spline end out to the outer circle.
        x0, y0 = contourPts[0]
        a0 = math.atan2(y0, x0)
        x1, y1 = contourPts[-1]
        a1 = math.atan2(y1, x1)
        spoke1 = casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            contourStart,
            adsk.core.Point3D.create(
                outerR * math.cos(a0), outerR * math.sin(a0), 0))
        cCons.addCoincident(spoke1.endSketchPoint, casingOuter)
        spoke2 = casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            contourEnd,
            adsk.core.Point3D.create(
                outerR * math.cos(a1), outerR * math.sin(a1), 0))
        cCons.addCoincident(spoke2.endSketchPoint, casingOuter)

        # Extrude the sector wedge (smallest-area profile containing contour),
        # two-sided: up to stackTop, and 1 mm down to the base top.
        sectorProfile = self._smallestProfileContaining(casingSketch, contour)
        stackTopExpr = self._stackTopExpr()
        secExt = extrudes.createInput(
            sectorProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        secExt.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr)),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString('1 mm')))
        casingBase = component.bRepBodies.count
        sectorFeature = extrudes.add(secExt)

        coll2 = adsk.core.ObjectCollection.create()
        coll2.add(sectorFeature)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll2, self.driveAxis)
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(N)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

        # Join the N casing sectors into one casing body.
        casingTarget = component.bRepBodies.item(casingBase)
        casingTools = adsk.core.ObjectCollection.create()
        for i in range(casingBase + 1, casingBase + N):
            casingTools.add(component.bRepBodies.item(i))
        combines = component.features.combineFeatures
        ci = combines.createInput(casingTarget, casingTools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci)
        self.ringCasing = casingTarget
        self.ringCasing.name = 'Ring Casing'

        # 5. Combine casing into the base -> one Housing.
        housingTools = adsk.core.ObjectCollection.create()
        housingTools.add(self.ringCasing)
        ci2 = combines.createInput(self.housingRing, housingTools)
        ci2.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci2)
        self.housingRing.name = 'Housing'
        self.ringCasing = None

    # =======================================================================
    # §7: buildOutputPins() — plate + M output pins
    # =======================================================================
    def buildOutputPins(self):
        component = self.getComponent()
        M = self.outputPinCountInt
        Rop = self.Rop
        Dpin = self.Dpin
        stackTopExpr = self._stackTopExpr()

        # 1. Output plate plane, 1 mm above the top disc.
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane,
            adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))
        platePlane = component.constructionPlanes.add(pi)
        platePlane.name = 'Output Plate Plane'

        # 2. Output Plate sketch.
        sketch = self.createSketchObject('Output Plate', platePlane)
        sketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(sketch)
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        plateD = self.getParameter(PARAM_OUTPUT_PLATE_DIAMETER).value
        plateOuter = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), plateD / 2.0)
        cons.addCoincident(plateOuter.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            plateOuter, adsk.core.Point3D.create(plateD / 2.0, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)

        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), Rop)
        outPinCircle.isConstruction = True
        cons.addCoincident(outPinCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            outPinCircle, adsk.core.Point3D.create(Rop, 0, 0)
        ).parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)

        pin = circles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), Dpin / 2.0)
        dims.addDiameterDimension(
            pin, adsk.core.Point3D.create(Rop + Dpin / 2.0, 0, 0)
        ).parameter.expression = '{} - 2 * {}'.format(
            self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
            self.parameterName(PARAM_ECCENTRICITY))
        cons.addCoincident(pin.centerSketchPoint, outPinCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        spoke.isConstruction = True
        cons.addHorizontal(spoke)

        # 3. Output plate body — all profiles, away from the disk.
        extrudes = component.features.extrudeFeatures
        plateColl = adsk.core.ObjectCollection.create()
        for profile in sketch.profiles:
            plateColl.add(profile)
        ext = extrudes.createInput(
            plateColl, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        plateExtrude = extrudes.add(ext)
        self.outputPlate = plateExtrude.bodies.item(0)
        self.outputPlate.name = 'Output Plate'

        # 4. Output pin extrude — two-sided from the pin disc.
        pinProfile = self._profileSingleLoopContaining(sketch, pin)
        pinExt = extrudes.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pinExt.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm')))
        pinFeature = extrudes.add(pinExt)
        pinBody = pinFeature.bodies.item(0)
        pinBody.name = 'Output Pin'

        # 5. Socket (combine-cut, keep the pin).
        combines = component.features.combineFeatures
        tools = adsk.core.ObjectCollection.create()
        tools.add(pinBody)
        ci = combines.createInput(self.outputPlate, tools)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = True
        combineFeature = combines.add(ci)

        # 6. Chamfer the pin ends.
        chamferFeature = self._chamferCapRims(pinBody)

        # 7. Pattern pin, socket and chamfer x M about the drive axis.
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature:
            coll.add(chamferFeature)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll, self.driveAxis)
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

    # =======================================================================
    # §8: buildChamfers()
    # =======================================================================
    def buildChamfers(self):
        if self.chamferSize <= 0:
            return
        for body in self.diskBodies:
            if body is not None:
                self._chamferCapRims(body)
        if self.housingRing is not None:
            self._chamferCapRims(self.housingRing)
        if self.outputPlate is not None:
            self._chamferCapRims(self.outputPlate)

    # =======================================================================
    # [CYCLOIDAL-F-CHAMFERS] _chamferCapRims
    # =======================================================================
    def _chamferCapRims(self, body):
        if self.chamferSize <= 0:
            return None

        axis = self.plane.geometry.normal
        ref = self.plane.geometry.origin

        capFaces = []
        for face in body.faces:
            if face.geometry.surfaceType != \
                    adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            n = face.geometry.normal
            if abs(n.dotProduct(axis)) <= 0.999:
                continue
            o = face.geometry.origin
            h = ((o.x - ref.x) * axis.x + (o.y - ref.y) * axis.y +
                 (o.z - ref.z) * axis.z)
            capFaces.append((h, face))

        if len(capFaces) == 0:
            return None
        heights = [h for (h, _f) in capFaces]
        hmin = min(heights)
        hmax = max(heights)
        tol = 1e-4

        edges = adsk.core.ObjectCollection.create()
        for (h, face) in capFaces:
            if abs(h - hmin) > tol and abs(h - hmax) > tol:
                continue
            for loop in face.loops:
                if not loop.isOuter:
                    continue
                for edge in loop.edges:
                    edges.add(edge)

        if edges.count == 0:
            return None

        chamfers = self.getComponent().features.chamferFeatures
        ci = chamfers.createInput2()
        ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges,
            adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_CHAMFER_SIZE)),
            False)
        return chamfers.add(ci)
