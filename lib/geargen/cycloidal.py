import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator, get_value, get_selection
from . import solids

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

# Derived parameter names (kept verbatim).
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

        # 1. Target Plane (selection first)
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane on which the rotor lobe is sketched')
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point (selection)
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the drive axis is aligned with')
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Disc Count (dropdown 1/2, default '1')
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
            adsk.core.ValueInput.createByReal(to_cm(0)))

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
            INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'Output Pin Circle Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(50)))

        # 15. Output Pin Count
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', '',
            adsk.core.ValueInput.createByReal(6))

        # 16. Output Pin Diameter (0 = auto)
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

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
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        # Stashed selections.
        self.plane = None
        self.anchorPoint = None
        # Per-disc handles are LISTS indexed by the disc index d.
        self.diskBodies = []
        self.diskAxes = []
        self.lobeSplines = []
        self.outputHoles = []
        self.lobeDiskCentres = []
        self.discPlanes = []
        # Scalars.
        self.driveAxis = None
        self.housingRing = None
        self.ringCasing = None
        self.cam = None
        self.outputPlate = None
        self.lobePinCircle = None
        self._normalizedPlane = None

    # -- subclass hooks ------------------------------------------------------
    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    def generateName(self):
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    # =======================================================================
    # Input processing + parameter registration
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

        # Disc Count is a dropdown — read the selected item's name as int.
        discCountInput = inputs.itemById(INPUT_ID_DISC_COUNT)
        self.discCount = int(discCountInput.selectedItem.name)

        # 2. Register the primary input-sourced parameters. The first
        # addParameter call creates the occurrence (context shift); selections
        # are already stashed on self.
        self.addParameter(
            PARAM_PIN_COUNT, get_value(inputs, INPUT_ID_PIN_COUNT, ''),
            '', 'Number of ring pins (N)')
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
            PARAM_DISK_CLEARANCE, get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'),
            'mm', 'Disk profile clearance (backlash)')
        self.addParameter(
            PARAM_DISC_THICKNESS, get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'),
            'mm', 'Axial thickness of each rotor disk')
        self.addParameter(
            PARAM_DISC_GAP, get_value(inputs, INPUT_ID_DISC_GAP, 'mm'),
            'mm', 'Axial gap between disc 1 and disc 2')
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
            'mm', 'Output pin diameter (0 = auto)')
        self.addParameter(
            PARAM_CENTER_BEARING_DIAMETER,
            get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'),
            'mm', 'Center bearing / cam outer diameter')
        self.addParameter(
            PARAM_INPUT_SHAFT_DIAMETER,
            get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'),
            'mm', 'Input shaft bore diameter (0 = none)')
        self.addParameter(
            PARAM_BEARING_CLEARANCE,
            get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'),
            'mm', 'Diametral bearing clearance for the disk center bore')
        self.addParameter(
            PARAM_WALL, get_value(inputs, INPUT_ID_WALL, 'mm'),
            'mm', 'Radial wall thickness of the pinless casing')
        self.addParameter(
            PARAM_BASE_THICKNESS, get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the ring-pin base (Housing Ring)')
        self.addParameter(
            PARAM_OUTPUT_PLATE_THICKNESS,
            get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the output plate')
        self.addParameter(
            PARAM_CHAMFER_SIZE, get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'),
            'mm', 'Outer-rim / pin-end chamfer size (0 = none)')

        # 3. Resolve dimensions in Python (internal cm).
        self._resolveDimensions()

        # 4. ⚠️ Two-disc validity: require even N and even M.
        if self.discCount == 2:
            if (self.N % 2 != 0) or (self.M % 2 != 0):
                raise Exception(
                    'Two discs require an even Pin Count and even Output Pin Count')

        # 5. Register the derived parameters.
        self._registerDerivedParameters()

    def _resolveDimensions(self):
        # All raw values are in internal cm (lengths) / unitless (counts).
        self.N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        self.M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        self.L = self.N - 1

        N = self.N
        M = self.M

        self.R = self.getParameter(PARAM_PIN_CIRCLE_DIAMETER).value / 2.0
        self.Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2.0
        self.E = self.getParameter(PARAM_ECCENTRICITY).value
        self.c = self.getParameter(PARAM_DISK_CLEARANCE).value
        pinDiameter = self.getParameter(PARAM_PIN_DIAMETER).value
        outputPinDiameter = self.getParameter(PARAM_OUTPUT_PIN_DIAMETER).value
        self.centerBearingDiameter = self.getParameter(
            PARAM_CENTER_BEARING_DIAMETER).value
        self.inputShaftDiameter = self.getParameter(
            PARAM_INPUT_SHAFT_DIAMETER).value
        self.bearingClearance = self.getParameter(PARAM_BEARING_CLEARANCE).value
        self.wall = self.getParameter(PARAM_WALL).value
        self.chamferSize = self.getParameter(PARAM_CHAMFER_SIZE).value

        R = self.R
        E = self.E
        Rop = self.Rop

        # Ring-pin radius (auto-vs-override).
        if pinDiameter > 0:
            self.Rr = pinDiameter / 2.0
        else:
            self.Rr = 0.5 * (E + R * math.sin(math.pi / N))

        self.Rr_eff = self.Rr + self.c
        self.Rv = R - self.Rr_eff - E

        # Output-pin diameter (auto-vs-override) and output-hole diameter.
        if outputPinDiameter > 0:
            self.D_pin = outputPinDiameter
        else:
            self.D_pin = Rop * math.sin(math.pi / M) - E
        self.D_hole = self.D_pin + 2.0 * E

        # ⚠️ Validity (on the resolved values).
        if not (E < self.Rr < R * math.sin(math.pi / N)):
            raise Exception(
                'Invalid Pin Radius: require E < PinRadius < R*sin(pi/N) '
                '(resolved Rr={:.4f} cm)'.format(self.Rr))
        if not (self.D_pin > 0):
            raise Exception('Invalid Output Pin Diameter: resolved D_pin <= 0')
        if not (self.D_hole < 2.0 * Rop * math.sin(math.pi / M)):
            raise Exception(
                'Invalid Output Hole Diameter: the M output holes overlap '
                '(D_hole >= 2*Rop*sin(pi/M))')
        if not (E < R / N):
            raise Exception(
                'Eccentricity too large: require E < R/N '
                '(reduce Eccentricity below ~{:.3f} mm)'.format(
                    to_mm(R / N)))
        if not (Rop < self.Rv):
            raise Exception(
                'Output Pin Circle too large: require Rop < Rv (root radius)')

        # No-undercut guard (the binding eccentricity limit).
        rhoMinO = self._rhoMinTowardO()
        if rhoMinO is not None and not (self.Rr_eff < rhoMinO):
            # Solve for the eccentricity limit numerically for the message.
            raise Exception(
                'Eccentricity too large (undercut): the inward profile offset '
                'self-intersects. Reduce Eccentricity '
                '(Rr_eff={:.4f} cm >= rho_min^O={:.4f} cm).'.format(
                    self.Rr_eff, rhoMinO))

        # Cam / bore validity.
        if not (self.inputShaftDiameter < self.centerBearingDiameter):
            raise Exception(
                'Input Shaft Diameter must be < Center Bearing Diameter')
        if not (E + self.inputShaftDiameter / 2.0 < self.centerBearingDiameter / 2.0):
            raise Exception(
                'Input bore does not fit inside the cam: require '
                'E + InputShaftDiameter/2 < CenterBearingDiameter/2')
        enlargedBoreRadius = (self.centerBearingDiameter + self.bearingClearance) / 2.0
        if not (enlargedBoreRadius < Rop - self.D_hole / 2.0):
            raise Exception(
                'Enlarged disk center bore overlaps the output holes: require '
                '(CenterBearingDiameter + BearingClearance)/2 < Rop - D_hole/2')

    def _rhoMinTowardO(self):
        # No-undercut guard: minimum radius of curvature of the base trochoid
        # over points whose centre of curvature lies toward O. (cm.)
        R = self.R
        E = self.E
        N = self.N
        samples = 2000
        rhoMin = None
        for i in range(samples):
            t = 2.0 * math.pi * i / samples
            bx = R * math.cos(t) - E * math.cos(N * t)
            by = -R * math.sin(t) + E * math.sin(N * t)
            xp = -R * math.sin(t) + E * N * math.sin(N * t)
            yp = -R * math.cos(t) + E * N * math.cos(N * t)
            xpp = -R * math.cos(t) + E * N * N * math.cos(N * t)
            ypp = R * math.sin(t) - E * N * N * math.sin(N * t)
            k = xp * ypp - yp * xpp
            if abs(k) < 1e-12:
                continue
            speed2 = xp * xp + yp * yp
            s = math.sqrt(speed2)
            rho = (speed2 ** 1.5) / k
            nx = -yp / s
            ny = xp / s
            cx = bx + rho * nx
            cy = by + rho * ny
            if (cx * cx + cy * cy) < (bx * bx + by * by):
                a = abs(rho)
                if rhoMin is None or a < rhoMin:
                    rhoMin = a
        return rhoMin

    def _registerDerivedParameters(self):
        def addLive(name, expr, units, comment):
            self.addParameter(
                name, adsk.core.ValueInput.createByString(expr), units, comment)

        nPinCount = self.parameterName(PARAM_PIN_COUNT)
        nPinCircleD = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        nOutPinCircleD = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        nEcc = self.parameterName(PARAM_ECCENTRICITY)
        nWall = self.parameterName(PARAM_WALL)

        # Lobes = N - 1 (live).
        addLive(PARAM_LOBES, '{} - 1'.format(nPinCount), '', 'Number of rotor lobes')
        # PinCircleRadius, OutputPinCircleRadius (live).
        addLive(PARAM_PIN_CIRCLE_RADIUS, '{} / 2'.format(nPinCircleD), 'mm',
                'Pin circle (pitch) radius')
        addLive(PARAM_OUTPUT_PIN_CIRCLE_RADIUS, '{} / 2'.format(nOutPinCircleD),
                'mm', 'Output pin circle radius')

        # ⚠️ PinRadius and OutputHoleDiameter are createByReal numeric snapshots
        # of the Python-resolved values (internal cm).
        self.addParameter(
            PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(self.Rr),
            'mm', 'Resolved ring-pin radius (numeric snapshot)')
        self.addParameter(
            PARAM_OUTPUT_HOLE_DIAMETER,
            adsk.core.ValueInput.createByReal(self.D_hole),
            'mm', 'Resolved output hole diameter (numeric snapshot)')

        nPinCircleR = self.parameterName(PARAM_PIN_CIRCLE_RADIUS)
        nPinRadius = self.parameterName(PARAM_PIN_RADIUS)

        # HousingInnerDiameter = 2*(R - PinRadius - Wall) (live).
        addLive(PARAM_HOUSING_INNER_DIAMETER,
                '2 * ({} - {} - {})'.format(nPinCircleR, nPinRadius, nWall),
                'mm', 'Housing inner (floor lip) diameter')
        # HousingOuterDiameter = 2*(R - PinRadius + 2*E + Wall) (live).
        addLive(PARAM_HOUSING_OUTER_DIAMETER,
                '2 * ({} - {} + 2 * {} + {})'.format(
                    nPinCircleR, nPinRadius, nEcc, nWall),
                'mm', 'Housing / casing outer diameter')
        # OutputPlateDiameter = OutputPinCircleDiameter + D_pin + 2*Wall.
        # D_pin = OutputHoleDiameter - 2*Eccentricity (live).
        nOutHoleD = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        addLive(PARAM_OUTPUT_PLATE_DIAMETER,
                '{} + ({} - 2 * {}) + 2 * {}'.format(
                    nOutPinCircleD, nOutHoleD, nEcc, nWall),
                'mm', 'Output plate diameter')

    # =======================================================================
    # Geometry oracle — the cycloidal rotor point function
    # =======================================================================
    def disk_point(self, t, cx, cy, phi):
        # Returns (x, y) in cm. Reproduced exactly from epitrochoid-trace.md.
        R = self.R
        E = self.E
        N = self.N
        Rr_eff = self.Rr + self.c
        num = math.sin((1 - N) * t)
        den = (R / (E * N)) - math.cos((1 - N) * t)
        psi = math.atan2(num, den)
        x0 = R * math.cos(t) - Rr_eff * math.cos(t + psi) - E * math.cos(N * t)
        y0 = -R * math.sin(t) + Rr_eff * math.sin(t + psi) + E * math.sin(N * t)
        x = cx + (x0 * math.cos(phi) - y0 * math.sin(phi))
        y = cy + (x0 * math.sin(phi) + y0 * math.cos(phi))
        return (x, y)

    def _sampleLobeAdaptive(self, cx, cy, phi):
        # Adaptive (bounded-turn-angle) sampling of one lobe, t in [0, 2pi/L].
        L = self.L
        tEnd = 2.0 * math.pi / L
        fine = 2000
        pts = []
        for i in range(fine + 1):
            t = tEnd * i / fine
            pts.append(self.disk_point(t, cx, cy, phi))

        thresh = math.radians(5.0)
        kept = [pts[0]]
        accum = 0.0
        for i in range(1, len(pts) - 1):
            ax = pts[i][0] - pts[i - 1][0]
            ay = pts[i][1] - pts[i - 1][1]
            bx = pts[i + 1][0] - pts[i][0]
            by = pts[i + 1][1] - pts[i][1]
            la = math.hypot(ax, ay)
            lb = math.hypot(bx, by)
            if la > 1e-12 and lb > 1e-12:
                dot = (ax * bx + ay * by) / (la * lb)
                dot = max(-1.0, min(1.0, dot))
                accum += math.acos(dot)
            if accum >= thresh:
                kept.append(pts[i])
                accum = 0.0
        kept.append(pts[-1])
        return kept

    def _sweptContourPitch(self):
        # One pin-pitch of the swept envelope contour(phi) = env(phi) + c.
        # Returns the (nbins+1) points at bin EDGES, ordered by angle, in cm.
        N = self.N
        L = self.L
        E = self.E
        c = self.c
        Ntheta = 240
        Nt = 240
        nbins = 80
        binMax = [None] * nbins
        lo = -math.pi / N
        hi = math.pi / N
        span = hi - lo
        for it in range(Ntheta):
            theta = 2.0 * math.pi * it / Ntheta
            cx = E * math.cos(theta)
            cy = E * math.sin(theta)
            phi = -theta / L
            for iu in range(Nt):
                t = 2.0 * math.pi * iu / Nt
                x, y = self.disk_point(t, cx, cy, phi)
                a = math.atan2(y, x)
                if lo <= a <= hi:
                    r = math.hypot(x, y)
                    b = int((a - lo) / span * nbins)
                    if b >= nbins:
                        b = nbins - 1
                    if binMax[b] is None or r > binMax[b]:
                        binMax[b] = r
        # Emit points at bin EDGES so first/last land EXACTLY on -+pi/N.
        pts = []
        for i in range(nbins + 1):
            phi_i = lo + span * i / nbins
            neighbours = []
            if i - 1 >= 0 and binMax[i - 1] is not None:
                neighbours.append(binMax[i - 1])
            if i < nbins and binMax[i] is not None:
                neighbours.append(binMax[i])
            if not neighbours:
                # Fallback: should not happen for valid geometry.
                continue
            r_i = c + max(neighbours)
            pts.append((r_i * math.cos(phi_i), r_i * math.sin(phi_i)))
        return pts

    # =======================================================================
    # Sketch helpers
    # =======================================================================
    def _anchorLocalOrigin(self, sketch):
        # [CYCLOIDAL-F-ANCHOR-CHAIN]: project the user's Anchor and constrain a
        # fresh local origin to it. Returns the local-origin SketchPoint.
        localOrigin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        projected = sketch.project(self.anchorPoint)
        sketch.geometricConstraints.addCoincident(localOrigin, projected.item(0))
        return localOrigin

    def _buildDiskCentre(self, sketch, localOrigin, s_d):
        # [CYCLOIDAL-F-DISK-CENTER]: the eccentric disk centre Od = O + s_d*E*X.
        E = self.E
        diskCentre = sketch.sketchPoints.add(
            adsk.core.Point3D.create(s_d * E, 0, 0))
        eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        dim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(s_d * E / 2.0, 0, 0))
        dim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return diskCentre

    def _alongPathLabel(self, sketch, circle, name):
        textInput = sketch.sketchTexts.createInput2(name, self.Rr)
        textInput.setAsAlongPath(
            circle, True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(textInput)

    # -- per-disc plane ------------------------------------------------------
    def _planeForDisc(self, d):
        if d == 0:
            return self.plane
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        planes = self.getComponent().constructionPlanes
        pi = planes.createInput()
        pi.setByOffset(
            self.plane,
            adsk.core.ValueInput.createByString(
                '{} * ({} + {})'.format(d, nT, nG)))
        plane = planes.add(pi)
        plane.name = 'Disc Plane {}'.format(d + 1)
        return plane

    def _stackTopExpr(self):
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        if self.discCount == 1:
            return nT
        return '2 * {} + {}'.format(nT, nG)

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
        # Size per-disc lists.
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
        self.buildSubComponents()

    # =======================================================================
    # §2: lobe sketch — buildLobeSketch(d)
    # =======================================================================
    def buildLobeSketch(self, d):
        s_d = 1.0 if d == 0 else -1.0
        phi = d * math.pi
        plane = self._planeForDisc(d)
        self.discPlanes[d] = plane

        sketch = self.createSketchObject('Rotor Lobe {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._buildDiskCentre(sketch, localOrigin, s_d)

        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        R = self.R
        Rop = self.Rop
        Rv = self.Rv
        E = self.E
        L = self.L
        cx = s_d * E

        # 1. pin circle (construction, on O).
        pinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), R)
        pinCircle.isConstruction = True
        cons.addCoincident(pinCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            pinCircle, adsk.core.Point3D.create(R, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, pinCircle, 'Pin Circle')
        if d == 0:
            self.lobePinCircle = pinCircle

        # 2. output-pin circle (construction, on Od).
        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(cx, 0, 0), Rop)
        outPinCircle.isConstruction = True
        cons.addCoincident(outPinCircle.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            outPinCircle,
            adsk.core.Point3D.create(cx + Rop, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, outPinCircle, 'Output Pin Circle')

        # 3. root/valley circle (construction, on Od).
        rootCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(cx, 0, 0), Rv)
        rootCircle.isConstruction = True
        cons.addCoincident(rootCircle.centerSketchPoint, diskCentre)
        nPinCircleR = self.parameterName(PARAM_PIN_CIRCLE_RADIUS)
        nPinRadius = self.parameterName(PARAM_PIN_RADIUS)
        nDiskClr = self.parameterName(PARAM_DISK_CLEARANCE)
        nEcc = self.parameterName(PARAM_ECCENTRICITY)
        dims.addDiameterDimension(
            rootCircle,
            adsk.core.Point3D.create(cx + Rv, 0, 0)).parameter.expression = \
            '2 * ({} - {} - {} - {})'.format(
                nPinCircleR, nPinRadius, nDiskClr, nEcc)
        self._alongPathLabel(sketch, rootCircle, 'Root Circle')

        # 4. lobe spline (open, adaptively sampled) — NO arc.
        kept = self._sampleLobeAdaptive(cx, 0.0, phi)
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in kept:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline = sketch.sketchCurves.sketchFittedSplines.add(coll)
        startPt = spline.fitPoints.item(0)
        endPt = spline.fitPoints.item(spline.fitPoints.count - 1)

        # 5. lock the spline: fix interior fit points, coincide ends on root.
        for i in range(1, spline.fitPoints.count - 1):
            spline.fitPoints.item(i).isFixed = True
        cons.addCoincident(startPt, rootCircle)
        cons.addCoincident(endPt, rootCircle)

        # 6. spoke line 1 + horizontal (from the disk centre).
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(cx + Rv, 0, 0))
        cons.addCoincident(line1.endSketchPoint, startPt)
        cons.addHorizontal(line1)

        # 7. spoke line 2 (from the disk centre).
        line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(
                cx + Rv * math.cos(2.0 * math.pi / L),
                -Rv * math.sin(2.0 * math.pi / L), 0))
        cons.addCoincident(line2.endSketchPoint, endPt)

        # 8. lobe-pitch angle dim (driving).
        textPoint = adsk.core.Point3D.create(
            cx + 0.4 * Rv * math.cos(math.pi / L),
            -0.4 * Rv * math.sin(math.pi / L), 0)
        angDim = dims.addAngularDimension(line1, line2, textPoint)
        angDim.parameter.expression = '360 deg / {}'.format(
            self.parameterName(PARAM_LOBES))

        self.lobeDiskCentres[d] = diskCentre
        self.lobeSplines[d] = spline

    # =======================================================================
    # §2·A: extrude + axis + pattern → rotor disk — buildDisk(d)
    # =======================================================================
    def buildDisk(self, d):
        component = self.getComponent()
        plane = self.discPlanes[d]
        spline = self.lobeSplines[d]

        # Record body baseline BEFORE the sector extrude.
        base = component.bRepBodies.count

        # 1. Extrude the lobe-sector profile (the loop containing the spline).
        sectorProfile = self._profileContainingCurve(
            spline.parentSketch, spline)
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

        # 2. Disk axis from the extrude's planar cap face (by NORMAL).
        capFace = extrude.startFaces.item(0)
        self.buildDiskAxis(capFace, d)

        # 3. Circular-pattern the EXTRUDE FEATURE x L about the disk axis.
        coll = adsk.core.ObjectCollection.create()
        coll.add(extrude)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll, self.diskAxes[d])
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(self.L)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

        # 4. Join ONLY disc d's own L sectors.
        target = component.bRepBodies.item(base)
        tools = adsk.core.ObjectCollection.create()
        for i in range(base + 1, base + self.L):
            tools.add(component.bRepBodies.item(i))
        combines = component.features.combineFeatures
        ci = combines.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci)
        target.name = 'Cycloidal Disk {}'.format(d + 1)
        self.diskBodies[d] = target

    def buildDiskAxis(self, capFace, d):
        # [CYCLOIDAL-F-DISK-AXIS]: axis perpendicular to the sketch at Od_d.
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
        axis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis {}'.format(d + 1)
        self.diskAxes[d] = axis

    def _profileContainingCurve(self, sketch, curve):
        # The profile whose loop contains the given sketch curve (identity).
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == curve:
                        return profile
        raise Exception(
            'Could not find profile containing curve in sketch "{}"'.format(
                sketch.name))

    # =======================================================================
    # §3: output-hole sketch — buildOutputHoleSketch(d)
    # =======================================================================
    def buildOutputHoleSketch(self, d):
        s_d = 1.0 if d == 0 else -1.0
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Output Hole {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._buildDiskCentre(sketch, localOrigin, s_d)

        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        cx = s_d * self.E
        Rop = self.Rop
        D_hole = self.D_hole

        # 1. output-hole circle (construction, on Od).
        outHoleCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(cx, 0, 0), Rop)
        outHoleCircle.isConstruction = True
        cons.addCoincident(outHoleCircle.centerSketchPoint, diskCentre)
        dims.addDiameterDimension(
            outHoleCircle,
            adsk.core.Point3D.create(cx + Rop, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, outHoleCircle, 'Output Hole Circle')

        # 2. one solid hole.
        hole = circles.addByCenterRadius(
            adsk.core.Point3D.create(cx + Rop, 0, 0), D_hole / 2.0)
        dims.addDiameterDimension(
            hole,
            adsk.core.Point3D.create(cx + Rop + D_hole / 2.0, 0, 0)
        ).parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        cons.addCoincident(hole.centerSketchPoint, outHoleCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        spoke.isConstruction = True
        cons.addHorizontal(spoke)

        self.outputHoles[d] = hole

    # =======================================================================
    # §4: cut + pattern the output holes — buildOutputHoles(d)
    # =======================================================================
    def buildOutputHoles(self, d):
        component = self.getComponent()
        hole = self.outputHoles[d]
        holeProfile = self._profileContainingCurve(hole.parentSketch, hole)

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
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(self.M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

    # =======================================================================
    # §6: disc center bore — buildDiskBore(d)
    # =======================================================================
    def buildDiskBore(self, d):
        component = self.getComponent()
        s_d = 1.0 if d == 0 else -1.0
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Disc Bore {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._buildDiskCentre(sketch, localOrigin, s_d)

        cx = s_d * self.E
        boreRadius = (self.centerBearingDiameter + self.bearingClearance) / 2.0
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(cx, 0, 0), boreRadius)
        sketch.geometricConstraints.addCoincident(
            circle.centerSketchPoint, diskCentre)
        sketch.sketchDimensions.addDiameterDimension(
            circle,
            adsk.core.Point3D.create(cx + boreRadius, 0, 0)
        ).parameter.expression = '{} + {}'.format(
            self.parameterName(PARAM_CENTER_BEARING_DIAMETER),
            self.parameterName(PARAM_BEARING_CLEARANCE))

        coll = adsk.core.ObjectCollection.create()
        for p in sketch.profiles:
            coll.add(p)
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
    # §6: eccentric cam — buildCam()
    # =======================================================================
    def buildCam(self):
        component = self.getComponent()
        D = self.discCount
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        sectionBodies = []

        for d in range(D):
            s_d = 1.0 if d == 0 else -1.0
            plane = self.discPlanes[d]
            sketch = self.createSketchObject(
                'Eccentric Cam {}'.format(d + 1), plane=plane)
            sketch.isVisible = True

            localOrigin = self._anchorLocalOrigin(sketch)
            diskCentre = self._buildDiskCentre(sketch, localOrigin, s_d)

            circles = sketch.sketchCurves.sketchCircles
            dims = sketch.sketchDimensions
            cons = sketch.geometricConstraints

            cx = s_d * self.E
            camOuter = circles.addByCenterRadius(
                adsk.core.Point3D.create(cx, 0, 0),
                self.centerBearingDiameter / 2.0)
            cons.addCoincident(camOuter.centerSketchPoint, diskCentre)
            dims.addDiameterDimension(
                camOuter,
                adsk.core.Point3D.create(
                    cx + self.centerBearingDiameter / 2.0, 0, 0)
            ).parameter.expression = self.parameterName(
                PARAM_CENTER_BEARING_DIAMETER)

            hasBore = self.inputShaftDiameter > 0
            if hasBore:
                inputBore = circles.addByCenterRadius(
                    adsk.core.Point3D.create(0, 0, 0),
                    self.inputShaftDiameter / 2.0)
                cons.addCoincident(inputBore.centerSketchPoint, localOrigin)
                dims.addDiameterDimension(
                    inputBore,
                    adsk.core.Point3D.create(
                        self.inputShaftDiameter / 2.0, 0, 0)
                ).parameter.expression = self.parameterName(
                    PARAM_INPUT_SHAFT_DIAMETER)

            # Select the cross-section profile.
            if hasBore:
                camProfile = self._profileByLoopCount(sketch, 2)
            else:
                camProfile = sketch.profiles.item(0)

            extrudes = component.features.extrudeFeatures
            ext = extrudes.createInput(
                camProfile,
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            if d < D - 1:
                extentExpr = '{} + {}'.format(nT, nG)
            else:
                extentExpr = nT
            ext.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(
                    adsk.core.ValueInput.createByString(extentExpr)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection)
            extrude = extrudes.add(ext)
            sectionBody = extrude.bodies.item(0)
            sectionBody.name = 'Eccentric Cam {}'.format(d + 1)
            sectionBodies.append(sectionBody)

        # Join all sections into one 'Eccentric Cam'.
        target = sectionBodies[0]
        if len(sectionBodies) > 1:
            tools = adsk.core.ObjectCollection.create()
            for b in sectionBodies[1:]:
                tools.add(b)
            combines = component.features.combineFeatures
            ci = combines.createInput(target, tools)
            ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combines.add(ci)
        target.name = 'Eccentric Cam'
        self.cam = target

    def _profileByLoopCount(self, sketch, loopCount):
        for profile in sketch.profiles:
            if profile.profileLoops.count == loopCount:
                return profile
        raise Exception(
            'No profile with {} loops in sketch "{}"'.format(
                loopCount, sketch.name))

    # =======================================================================
    # §5: housing base + ring casing — buildRingPins()
    # =======================================================================
    def buildRingPins(self):
        component = self.getComponent()
        R = self.R
        Rr = self.Rr
        E = self.E
        wall = self.wall
        c = self.c

        # 1. Housing plane (1 mm below the disc).
        planes = component.constructionPlanes
        pi = planes.createInput()
        pi.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane = planes.add(pi)
        housingPlane.name = 'Ring Housing Plane'

        # 2. Housing base — sketch 'Housing Ring'.
        sketch = self.createSketchObject('Housing Ring', plane=housingPlane)
        sketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(sketch)
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        outerR = R - Rr + 2.0 * E + wall
        innerR = R - Rr - wall
        outer = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        cons.addCoincident(outer.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            outer, adsk.core.Point3D.create(outerR, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)
        inner = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), innerR)
        cons.addCoincident(inner.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            inner, adsk.core.Point3D.create(innerR, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_HOUSING_INNER_DIAMETER)

        annulusProfile = self._profileByLoopCount(sketch, 2)
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

        # 3. Drive axis at O.
        capFace = housingExtrude.startFaces.item(0)
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, localOrigin)
        self.driveAxis = component.constructionAxes.add(axInput)
        self.driveAxis.name = 'Drive Axis'

        # 4. Ring casing — one section patterned x N.
        contourPts = self._sweptContourPitch()

        casingSketch = self.createSketchObject('Ring Casing', plane=self.plane)
        casingSketch.isVisible = True
        casingOrigin = self._anchorLocalOrigin(casingSketch)
        cCircles = casingSketch.sketchCurves.sketchCircles
        cDims = casingSketch.sketchDimensions
        cCons = casingSketch.geometricConstraints

        # outer circle (SOLID).
        casingOuter = cCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        cCons.addCoincident(casingOuter.centerSketchPoint, casingOrigin)
        cDims.addDiameterDimension(
            casingOuter,
            adsk.core.Point3D.create(outerR, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)

        # fitted spline through the contour points (open).
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in contourPts:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        contour = casingSketch.sketchCurves.sketchFittedSplines.add(coll)

        # two radial spokes from the spline ends out to the outer circle.
        firstPt = contourPts[0]
        lastPt = contourPts[-1]
        a0 = math.atan2(firstPt[1], firstPt[0])
        a1 = math.atan2(lastPt[1], lastPt[0])
        lines = casingSketch.sketchCurves.sketchLines
        spoke0 = lines.addByTwoPoints(
            contour.startSketchPoint,
            adsk.core.Point3D.create(outerR * math.cos(a0),
                                     outerR * math.sin(a0), 0))
        cCons.addCoincident(spoke0.endSketchPoint, casingOuter)
        spoke1 = lines.addByTwoPoints(
            contour.endSketchPoint,
            adsk.core.Point3D.create(outerR * math.cos(a1),
                                     outerR * math.sin(a1), 0))
        cCons.addCoincident(spoke1.endSketchPoint, casingOuter)

        # Extrude the SECTOR profile — MINIMUM AREA among profiles containing
        # the contour spline.
        sectorProfile = self._minAreaProfileContaining(casingSketch, contour)
        stackTopExpr = self._stackTopExpr()
        casingBase = component.bRepBodies.count
        ext = extrudes.createInput(
            sectorProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr)),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString('1 mm')))
        sectorFeature = extrudes.add(ext)

        patternColl = adsk.core.ObjectCollection.create()
        patternColl.add(sectorFeature)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(patternColl, self.driveAxis)
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(self.N)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

        # Join the N sector bodies into one casing body.
        casingTarget = component.bRepBodies.item(casingBase)
        casingTools = adsk.core.ObjectCollection.create()
        for i in range(casingBase + 1, casingBase + self.N):
            casingTools.add(component.bRepBodies.item(i))
        combines = component.features.combineFeatures
        ci = combines.createInput(casingTarget, casingTools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci)
        self.ringCasing = casingTarget

        # 5. Combine casing into the base → one Housing.
        houseTools = adsk.core.ObjectCollection.create()
        houseTools.add(self.ringCasing)
        ci2 = combines.createInput(self.housingRing, houseTools)
        ci2.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci2)
        self.housingRing.name = 'Housing'
        self.ringCasing = None

    def _minAreaProfileContaining(self, sketch, curve):
        best = None
        bestArea = None
        for profile in sketch.profiles:
            contains = False
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == curve:
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

    # =======================================================================
    # §7: output pins + plate — buildOutputPins()
    # =======================================================================
    def buildOutputPins(self):
        component = self.getComponent()
        stackTopExpr = self._stackTopExpr()
        Rop = self.Rop
        D_pin = self.D_pin
        plateR = self.getParameter(PARAM_OUTPUT_PLATE_DIAMETER).value / 2.0

        # 1. Output plate plane (1 mm above the top disc).
        planes = component.constructionPlanes
        pi = planes.createInput()
        pi.setByOffset(
            self.plane,
            adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))
        platePlane = planes.add(pi)
        platePlane.name = 'Output Plate Plane'

        # 2. One sketch 'Output Plate'.
        sketch = self.createSketchObject('Output Plate', plane=platePlane)
        sketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(sketch)
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints

        plateOuter = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), plateR)
        cons.addCoincident(plateOuter.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            plateOuter, adsk.core.Point3D.create(plateR, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)

        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), Rop)
        outPinCircle.isConstruction = True
        cons.addCoincident(outPinCircle.centerSketchPoint, localOrigin)
        dims.addDiameterDimension(
            outPinCircle, adsk.core.Point3D.create(Rop, 0, 0)).parameter.expression = \
            self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)

        pin = circles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2.0)
        dims.addDiameterDimension(
            pin, adsk.core.Point3D.create(Rop + D_pin / 2.0, 0, 0)
        ).parameter.expression = '{} - 2 * {}'.format(
            self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
            self.parameterName(PARAM_ECCENTRICITY))
        cons.addCoincident(pin.centerSketchPoint, outPinCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        spoke.isConstruction = True
        cons.addHorizontal(spoke)

        extrudes = component.features.extrudeFeatures

        # 3. Output plate body — all profiles, away from the disk.
        plateColl = adsk.core.ObjectCollection.create()
        for p in sketch.profiles:
            plateColl.add(p)
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

        # 4. Output pin extrude — two-sided. Pin disc = the 1-loop profile
        #    whose loop curve is the pin.
        pinProfile = self._singleLoopProfileOfCurve(sketch, pin)
        pinBase = component.bRepBodies.count
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
        tools = adsk.core.ObjectCollection.create()
        tools.add(pinBody)
        combines = component.features.combineFeatures
        ci = combines.createInput(self.outputPlate, tools)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = True
        combineFeature = combines.add(ci)

        # 6. Chamfer the pin ends.
        chamferFeature = self._chamferCapRims(pinBody)

        # 7. Pattern pin, socket AND chamfer x M about the drive axis.
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature:
            coll.add(chamferFeature)
        patterns = component.features.circularPatternFeatures
        pat = patterns.createInput(coll, self.driveAxis)
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(self.M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        patterns.add(pat)

        # 8. Name all M pin bodies.
        for k in range(self.M):
            component.bRepBodies.item(pinBase + k).name = 'Output Pin {}'.format(k + 1)

    def _singleLoopProfileOfCurve(self, sketch, curve):
        for profile in sketch.profiles:
            if profile.profileLoops.count != 1:
                continue
            loop = profile.profileLoops.item(0)
            for pc in loop.profileCurves:
                if pc.sketchEntity == curve:
                    return profile
        raise Exception(
            'No single-loop profile of the given curve in sketch "{}"'.format(
                sketch.name))

    # =======================================================================
    # §8: edge chamfers — buildChamfers()
    # =======================================================================
    def buildChamfers(self):
        if self.chamferSize <= 0:
            return
        for body in self.diskBodies:
            if body is not None:
                self._chamferCapRims(body)
        if self.housingRing is not None:
            self._chamferCapRims(self.housingRing)
        if self.ringCasing is not None:
            self._chamferCapRims(self.ringCasing)
        if self.outputPlate is not None:
            self._chamferCapRims(self.outputPlate)

    def _chamferCapRims(self, body):
        # [CYCLOIDAL-F-CHAMFERS]: chamfer only the two axially-extreme caps.
        if self.chamferSize <= 0:
            return None
        axis = self.plane.geometry.normal
        ref = self.plane.geometry.origin

        capFaces = []
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            n = face.geometry.normal
            if abs(n.dotProduct(axis)) <= 0.999:
                continue
            origin = face.geometry.origin
            dx = origin.x - ref.x
            dy = origin.y - ref.y
            dz = origin.z - ref.z
            h = dx * axis.x + dy * axis.y + dz * axis.z
            capFaces.append((h, face))

        if len(capFaces) == 0:
            return None
        heights = [h for (h, _) in capFaces]
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

    # =======================================================================
    # §9: organize bodies into sub-components — buildSubComponents()
    # =======================================================================
    def buildSubComponents(self):
        component = self.getComponent()

        rotorDiscs = []
        housing = []
        eccentricCam = []
        output = []
        # Snapshot all bodies into name-keyed lists FIRST.
        for body in component.bRepBodies:
            name = body.name
            if name.startswith('Cycloidal Disk'):
                rotorDiscs.append(body)
            elif name == 'Housing':
                housing.append(body)
            elif name == 'Eccentric Cam':
                eccentricCam.append(body)
            elif name == 'Output Plate' or name.startswith('Output Pin'):
                output.append(body)

        groups = [
            ('Rotor Discs', rotorDiscs),
            ('Housing', housing),
            ('Eccentric Cam', eccentricCam),
            ('Output', output),
        ]
        for (groupName, bodies) in groups:
            if len(bodies) == 0:
                continue
            occ = component.occurrences.addNewComponent(
                adsk.core.Matrix3D.create())
            occ.component.name = groupName
            for body in bodies:
                body.moveToComponent(occ)

        # Final cleanup — hide all construction geometry.
        solids.hide_construction_geometry(component)
