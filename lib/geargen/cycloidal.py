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
INPUT_ID_PIN_COUNT = 'pinCount'
INPUT_ID_PIN_CIRCLE_DIAMETER = 'pinCircleDiameter'
INPUT_ID_PIN_DIAMETER = 'pinDiameter'
INPUT_ID_ECCENTRICITY = 'eccentricity'
INPUT_ID_DISK_CLEARANCE = 'diskClearance'
INPUT_ID_DISC_THICKNESS = 'discThickness'
INPUT_ID_CENTER_BEARING_DIAMETER = 'centerBearingDiameter'
INPUT_ID_INPUT_SHAFT_DIAMETER = 'inputShaftDiameter'
INPUT_ID_BEARING_CLEARANCE = 'bearingClearance'
INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER = 'outputPinCircleDiameter'
INPUT_ID_OUTPUT_PIN_COUNT = 'outputPinCount'
INPUT_ID_OUTPUT_PIN_DIAMETER = 'outputPinDiameter'
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

# Adaptive-sampling turn-angle threshold (radians) for the lobe spline.
TURN_ANGLE_THRESHOLD = math.radians(5.0)


class CycloidalDriveCommandInputsConfigurator:
    """Adds the dialog inputs, in the fixed display order, to the command."""

    @classmethod
    def configure(cls, command):
        inputs = command.commandInputs
        design = get_design()

        # 1. Target Plane (selection — first, owns initial focus).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane on which the rotor lobe is sketched')
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point (selection).
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the drive axis is aligned with')
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Pin Count
        inputs.addValueInput(
            INPUT_ID_PIN_COUNT, 'Pin Count', '',
            adsk.core.ValueInput.createByReal(16))

        # 4. Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_PIN_CIRCLE_DIAMETER, 'Pin Circle Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(90)))

        # 5. Pin Diameter (0 = auto)
        inputs.addValueInput(
            INPUT_ID_PIN_DIAMETER, 'Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 6. Eccentricity
        inputs.addValueInput(
            INPUT_ID_ECCENTRICITY, 'Eccentricity', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(1.5)))

        # 7. Disk Clearance
        inputs.addValueInput(
            INPUT_ID_DISK_CLEARANCE, 'Disk Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.3)))

        # 8. Disc Thickness
        inputs.addValueInput(
            INPUT_ID_DISC_THICKNESS, 'Disc Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))

        # 9. Center Bearing Diameter
        inputs.addValueInput(
            INPUT_ID_CENTER_BEARING_DIAMETER, 'Center Bearing Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(30)))

        # 10. Input Shaft Diameter (0 = no bore)
        inputs.addValueInput(
            INPUT_ID_INPUT_SHAFT_DIAMETER, 'Input Shaft Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))

        # 11. Bearing Clearance
        inputs.addValueInput(
            INPUT_ID_BEARING_CLEARANCE, 'Bearing Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.2)))

        # 12. Output Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'Output Pin Circle Diameter',
            'mm', adsk.core.ValueInput.createByReal(to_cm(50)))

        # 13. Output Pin Count
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', '',
            adsk.core.ValueInput.createByReal(6))

        # 14. Output Pin Diameter (0 = auto)
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 15. Housing Wall
        inputs.addValueInput(
            INPUT_ID_WALL, 'Housing Wall', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))

        # 16. Base Thickness
        inputs.addValueInput(
            INPUT_ID_BASE_THICKNESS, 'Base Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))

        # 17. Output Plate Thickness
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PLATE_THICKNESS, 'Output Plate Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))

        # 18. Chamfer Size (0 = none)
        inputs.addValueInput(
            INPUT_ID_CHAMFER_SIZE, 'Chamfer Size', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.5)))

        # 19. Parent Component (last; defaults to the root component).
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the new cycloidal drive occurrence is nested under')
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(
            adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(0, 1)
        parentInput.addSelection(design.rootComponent)


class CycloidalDriveGenerator(Generator):
    """Generator for the (minimal-step) cycloidal drive. Subclasses
    base.Generator directly — no involute math, no GenerationContext."""

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = None
        self.anchorPoint = None
        self.chamferSize = 0.0
        # Handles carried between build steps (no GenerationContext class).
        self.lobePinCircle = None
        self.lobeDiskCentre = None
        self.diskAxis = None
        self.diskBody = None
        self.outputHole = None
        self.outputHoleCut = None
        self.housingRing = None
        self.driveAxis = None
        self.cam = None
        self.outputPlate = None

    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    # -- geometry oracle (epitrochoid-trace.md, returns cm) ------------------
    def disk_point(self, t, cx, cy, phi, R, Rr_eff, E, N):
        """Cycloidal rotor profile point (internal cm). Reproduces the
        epitrochoid-trace.md point function exactly."""
        num = math.sin((1 - N) * t)
        den = (R / (E * N)) - math.cos((1 - N) * t)
        psi = math.atan2(num, den)              # uses R, E, N only — NOT Rr
        x0 = R * math.cos(t) - Rr_eff * math.cos(t + psi) - E * math.cos(N * t)
        y0 = -R * math.sin(t) + Rr_eff * math.sin(t + psi) + E * math.sin(N * t)
        x = cx + (x0 * math.cos(phi) - y0 * math.sin(phi))
        y = cy + (x0 * math.sin(phi) + y0 * math.cos(phi))
        return (x, y)

    def rho_min_toward_O(self, R, E, N):
        """No-undercut guard: the minimum |radius of curvature| of the base
        trochoid at points whose centre of curvature lies toward O. Numerically
        sampled. Returns None when no qualifying point exists."""
        best = None
        samples = 2000
        for i in range(samples):
            t = 2.0 * math.pi * i / samples
            bx = R * math.cos(t) - E * math.cos(N * t)
            by = -R * math.sin(t) + E * math.sin(N * t)
            xp = -R * math.sin(t) + E * N * math.sin(N * t)
            yp = -R * math.cos(t) + E * N * math.cos(N * t)
            xpp = -R * math.cos(t) + E * N * N * math.cos(N * t)
            ypp = R * math.sin(t) - E * N * N * math.sin(N * t)
            k = xp * ypp - yp * xpp
            if abs(k) < 1e-9:
                continue
            s = math.sqrt(xp * xp + yp * yp)
            if s < 1e-12:
                continue
            rho = (xp * xp + yp * yp) ** 1.5 / k
            nx, ny = -yp / s, xp / s
            Cx = bx + rho * nx
            Cy = by + rho * ny
            if (Cx * Cx + Cy * Cy) < (bx * bx + by * by):
                aRho = abs(rho)
                if best is None or aRho < best:
                    best = aRho
        return best

    def sample_lobe(self, cx, cy, phi, R, Rr_eff, E, N, tEnd):
        """Adaptive (bounded-turn-angle) sampling of one lobe over [0, tEnd].
        Keeps the first point, accumulates the turn angle between consecutive
        fine points, keeps a point each time the accumulator reaches the
        threshold, and always keeps the last. Returns a list of (x, y) cm."""
        fine = 2000
        pts = []
        for i in range(fine + 1):
            t = tEnd * i / fine
            pts.append(self.disk_point(t, cx, cy, phi, R, Rr_eff, E, N))

        kept = [pts[0]]
        accum = 0.0
        for i in range(1, len(pts) - 1):
            prev = pts[i - 1]
            cur = pts[i]
            nxt = pts[i + 1]
            ax, ay = cur[0] - prev[0], cur[1] - prev[1]
            bx, by = nxt[0] - cur[0], nxt[1] - cur[1]
            la = math.hypot(ax, ay)
            lb = math.hypot(bx, by)
            if la < 1e-12 or lb < 1e-12:
                continue
            cosang = (ax * bx + ay * by) / (la * lb)
            cosang = max(-1.0, min(1.0, cosang))
            accum += math.acos(cosang)
            if accum >= TURN_ANGLE_THRESHOLD:
                kept.append(cur)
                accum = 0.0
        kept.append(pts[-1])
        return kept

    # -- input reading -------------------------------------------------------
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

        # 2. Register the input-sourced numeric parameters. The first
        # addParameter call creates the occurrence (context shift), but the
        # selections are already stashed on self.
        self.addParameter(
            PARAM_PIN_COUNT, get_value(inputs, INPUT_ID_PIN_COUNT, ''),
            '', 'Number of ring pins (N)')
        self.addParameter(
            PARAM_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Pin circle diameter')
        self.addParameter(
            PARAM_PIN_DIAMETER, get_value(inputs, INPUT_ID_PIN_DIAMETER, 'mm'),
            'mm', 'Ring pin diameter (0 = auto)')
        self.addParameter(
            PARAM_ECCENTRICITY, get_value(inputs, INPUT_ID_ECCENTRICITY, 'mm'),
            'mm', 'Eccentricity')
        self.addParameter(
            PARAM_DISK_CLEARANCE,
            get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'),
            'mm', 'Disk profile clearance (backlash)')
        self.addParameter(
            PARAM_DISC_THICKNESS,
            get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the rotor disk')
        self.addParameter(
            PARAM_CENTER_BEARING_DIAMETER,
            get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'),
            'mm', 'Center bearing / cam outer diameter')
        self.addParameter(
            PARAM_INPUT_SHAFT_DIAMETER,
            get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'),
            'mm', 'Input shaft bore diameter (0 = no bore)')
        self.addParameter(
            PARAM_BEARING_CLEARANCE,
            get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'),
            'mm', 'Disk-bore running clearance over the cam')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Output pin circle diameter')
        self.addParameter(
            PARAM_OUTPUT_PIN_COUNT,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, ''),
            '', 'Number of output pins / holes (M)')
        self.addParameter(
            PARAM_OUTPUT_PIN_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'mm'),
            'mm', 'Output pin diameter (0 = auto)')
        self.addParameter(
            PARAM_WALL, get_value(inputs, INPUT_ID_WALL, 'mm'),
            'mm', 'Radial wall thickness of the housing ring')
        self.addParameter(
            PARAM_BASE_THICKNESS,
            get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the housing ring base')
        self.addParameter(
            PARAM_OUTPUT_PLATE_THICKNESS,
            get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'mm'),
            'mm', 'Axial thickness of the output plate')
        self.addParameter(
            PARAM_CHAMFER_SIZE, get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'),
            'mm', 'Outer-rim / pin-end chamfer size (0 = none)')

        # 3. Resolve all geometry values in Python (internal cm).
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        self.N = N
        self.M = M
        if N < 4:
            raise Exception('Pin Count must be an integer >= 4')
        if M < 3:
            raise Exception('Output Pin Count must be an integer >= 3')

        R = self.getParameter(PARAM_PIN_CIRCLE_DIAMETER).value / 2.0
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2.0
        E = self.getParameter(PARAM_ECCENTRICITY).value
        c = self.getParameter(PARAM_DISK_CLEARANCE).value
        pinDiaInput = self.getParameter(PARAM_PIN_DIAMETER).value
        outPinDiaInput = self.getParameter(PARAM_OUTPUT_PIN_DIAMETER).value
        centerBearing = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        inputShaft = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        bearingClear = self.getParameter(PARAM_BEARING_CLEARANCE).value
        wall = self.getParameter(PARAM_WALL).value

        if E <= 0:
            raise Exception('Eccentricity must be > 0')

        # Ring-pin radius Rr (resolved: override or auto = mean of bounds).
        if pinDiaInput > 0:
            Rr = pinDiaInput / 2.0
        else:
            Rr = 0.5 * (E + R * math.sin(math.pi / N))

        Rr_eff = Rr + c
        Rv = R - Rr_eff - E

        # Output-pin diameter D_pin (resolved) and hole D_hole = D_pin + 2E.
        if outPinDiaInput > 0:
            D_pin = outPinDiaInput
        else:
            D_pin = Rop * math.sin(math.pi / M) - E
        D_hole = D_pin + 2.0 * E

        # Stash resolved values (cm) for the build steps.
        self.R = R
        self.Rop = Rop
        self.E = E
        self.c = c
        self.Rr = Rr
        self.Rr_eff = Rr_eff
        self.Rv = Rv
        self.D_pin = D_pin
        self.D_hole = D_hole
        self.discThickness = self.getParameter(PARAM_DISC_THICKNESS).value
        self.chamferSize = self.getParameter(PARAM_CHAMFER_SIZE).value

        # --- Validity (on the resolved values) -----------------------------
        if not (E < Rr):
            raise Exception(
                'Invalid geometry: Eccentricity must be less than the pin '
                'radius (E < Rr)')
        if not (Rr < R * math.sin(math.pi / N)):
            raise Exception(
                'Invalid geometry: pin radius too large (Rr < R*sin(pi/N))')
        if not (D_pin > 0):
            raise Exception(
                'Invalid geometry: resolved output pin diameter is not '
                'positive (D_pin > 0)')
        if not (D_hole < 2.0 * Rop * math.sin(math.pi / M)):
            raise Exception(
                'Invalid geometry: output holes overlap '
                '(D_hole < 2*Rop*sin(pi/M))')
        if not (E < R / N):
            raise Exception(
                'Invalid geometry: Eccentricity exceeds the base-cycloid cusp '
                'limit (E < R/N)')
        if not (Rop < Rv):
            raise Exception(
                'Invalid geometry: output pin circle must sit inside the root '
                'circle (Rop < Rv)')

        # No-undercut guard (the binding eccentricity limit).
        rhoMinO = self.rho_min_toward_O(R, E, N)
        if rhoMinO is None or not (Rr_eff < rhoMinO):
            # Build a reject message with the actual limit when possible: find
            # the max safe E (with the current Rr_eff) by a coarse scan.
            limitMm = None
            try:
                lo, hi = 1e-4, R / N
                for _ in range(40):
                    mid = 0.5 * (lo + hi)
                    rm = self.rho_min_toward_O(R, mid, N)
                    if rm is not None and Rr_eff < rm:
                        lo = mid
                    else:
                        hi = mid
                limitMm = lo * 10.0  # cm -> mm
            except Exception:
                limitMm = None
            if limitMm is not None:
                raise Exception(
                    'Invalid geometry: Eccentricity causes profile undercut '
                    '(the inward equidistant self-intersects). Reduce '
                    'Eccentricity below ~{:.2f} mm.'.format(limitMm))
            raise Exception(
                'Invalid geometry: Eccentricity causes profile undercut '
                '(Rr_eff >= rho_min toward O).')

        # Cam / bore validity.
        if not (inputShaft < centerBearing):
            raise Exception(
                'Invalid geometry: Input Shaft Diameter must be less than '
                'Center Bearing Diameter')
        if not (E + inputShaft / 2.0 < centerBearing / 2.0):
            raise Exception(
                'Invalid geometry: the input bore does not fit inside the cam '
                '(E + InputShaftDiameter/2 < CenterBearingDiameter/2)')
        if not ((centerBearing + bearingClear) / 2.0 < Rop - D_hole / 2.0):
            raise Exception(
                'Invalid geometry: the enlarged disk center bore does not '
                'clear the output holes')

        # 4. Register derived parameters as live Fusion expression strings
        # (the values are reference-clarity; geometry is driven by name).
        def addDerived(name, expr, units, comment):
            self.addParameter(
                name, adsk.core.ValueInput.createByString(expr), units, comment)

        nPinCount = self.parameterName(PARAM_PIN_COUNT)
        nPinCircleD = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        nOutPinCircleD = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        nEcc = self.parameterName(PARAM_ECCENTRICITY)

        addDerived(PARAM_LOBES, '{} - 1'.format(nPinCount), '',
                   'Number of rotor lobes (N - 1)')
        addDerived(PARAM_PIN_CIRCLE_RADIUS, '{} / 2'.format(nPinCircleD),
                   'mm', 'Pin circle radius')
        addDerived(PARAM_OUTPUT_PIN_CIRCLE_RADIUS,
                   '{} / 2'.format(nOutPinCircleD),
                   'mm', 'Output pin circle radius')

        # PinRadius (resolved): registered as the numeric resolved value.
        self.addParameter(
            PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(Rr),
            'mm', 'Resolved ring-pin (roller) radius')
        # OutputHoleDiameter (resolved).
        self.addParameter(
            PARAM_OUTPUT_HOLE_DIAMETER,
            adsk.core.ValueInput.createByReal(D_hole),
            'mm', 'Resolved output hole diameter (D_pin + 2E)')

        nPinRadius = self.parameterName(PARAM_PIN_RADIUS)
        nWall = self.parameterName(PARAM_WALL)
        nPinCircleR = self.parameterName(PARAM_PIN_CIRCLE_RADIUS)
        nOutHoleD = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)

        addDerived(PARAM_HOUSING_INNER_DIAMETER,
                   '2 * ({} - {} - {})'.format(nPinCircleR, nPinRadius, nWall),
                   'mm', 'Housing ring inner diameter')
        addDerived(PARAM_HOUSING_OUTER_DIAMETER,
                   '2 * ({} + {} + {})'.format(nPinCircleR, nPinRadius, nWall),
                   'mm', 'Housing ring outer diameter')
        addDerived(PARAM_OUTPUT_PLATE_DIAMETER,
                   '{} + ({} - 2 * {}) + 2 * {}'.format(
                       nOutPinCircleD, nOutHoleD, nEcc, nWall),
                   'mm', 'Output plate diameter')

    def generateName(self):
        N = self.N
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    # -- orchestration -------------------------------------------------------
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

        self.buildLobeSketch()
        self.buildDisk()
        self.buildOutputHoleSketch()
        self.buildOutputHoles()
        self.buildRingPins()
        self.buildCam()
        self.buildOutputPins()
        self.buildChamfers()

    # -- helpers -------------------------------------------------------------
    def _anchorLocalOrigin(self, sketch):
        """Project the user's Anchor and constrain a fresh local origin to it.
        Returns the local origin SketchPoint ([CYCLOIDAL-F-ANCHOR-CHAIN])."""
        localOrigin = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))
        projected = sketch.project(self.anchorPoint)
        sketch.geometricConstraints.addCoincident(
            localOrigin, projected.item(0))
        return localOrigin

    def _makeDiskCentre(self, sketch, localOrigin):
        """Build the eccentric disk centre Od = O + E*X̂ explicitly, with a
        driving distance dim referencing the Eccentricity parameter
        ([CYCLOIDAL-F-DISK-CENTER]). Returns the diskCentre SketchPoint."""
        E = self.E
        diskCentre = sketch.sketchPoints.add(
            adsk.core.Point3D.create(E, 0, 0))
        eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        eccDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(E / 2.0, -0.2 * max(E, 0.1), 0))
        eccDim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return diskCentre

    def _labelCircle(self, sketch, circle, name):
        """Along-path text label on a construction circle ([PB-SKETCH-TEXT])."""
        textInput = sketch.sketchTexts.createInput2(name, self.Rr)
        textInput.setAsAlongPath(
            circle, True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(textInput)

    def _profileContaining(self, sketch, entity):
        """Return the profile whose loop curves include `entity` (identity)."""
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == entity:
                        return profile
        raise Exception(
            'Could not find a profile containing the target sketch entity in '
            'sketch "{}"'.format(sketch.name))

    def _singleLoopProfileOf(self, sketch, entity):
        """Return the profile with profileLoops.count == 1 whose single loop's
        curve is `entity` (the disc bounded by that circle)."""
        for profile in sketch.profiles:
            if profile.profileLoops.count != 1:
                continue
            loop = profile.profileLoops.item(0)
            for pc in loop.profileCurves:
                if pc.sketchEntity == entity:
                    return profile
        raise Exception(
            'Could not find the single-loop disc profile for the target '
            'circle in sketch "{}"'.format(sketch.name))

    def _chamferCapRims(self, body):
        """Chamfer the outer rim of a disc-like body on both cap faces (or a
        pin's two ends). Returns the ChamferFeature, or None
        ([CYCLOIDAL-F-CHAMFERS])."""
        if self.chamferSize <= 0:
            return None
        axis = self.plane.geometry.normal
        edges = adsk.core.ObjectCollection.create()
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            n = face.geometry.normal
            if abs(n.dotProduct(axis)) <= 0.999:
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

    # -- step 2 --------------------------------------------------------------
    def buildLobeSketch(self):
        """'Rotor Lobe': pin + output-pin circles, ecc disk-centre, root circle,
        the open locked lobe, two spokes, and an angle dim
        ([CYCLOIDAL-F-DISK-LOBE])."""
        sketch = self.createSketchObject('Rotor Lobe', self.plane)
        sketch.isVisible = True
        circles = sketch.sketchCurves.sketchCircles
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._makeDiskCentre(sketch, localOrigin)

        R = self.R
        Rop = self.Rop
        Rv = self.Rv
        E = self.E
        N = self.N
        L = N - 1
        tEnd = 2.0 * math.pi / L

        # 1. Pin circle (construction, on O).
        pinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), R)
        pinCircle.isConstruction = True
        constraints.addCoincident(pinCircle.centerSketchPoint, localOrigin)
        d = dims.addDiameterDimension(
            pinCircle, adsk.core.Point3D.create(R, 0, 0))
        d.parameter.expression = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        self._labelCircle(sketch, pinCircle, 'Pin Circle')
        self.lobePinCircle = pinCircle

        # 2. Output-pin circle (construction, on Od).
        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(E, 0, 0), Rop)
        outPinCircle.isConstruction = True
        constraints.addCoincident(outPinCircle.centerSketchPoint, diskCentre)
        d = dims.addDiameterDimension(
            outPinCircle, adsk.core.Point3D.create(E + Rop, 0, 0))
        d.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._labelCircle(sketch, outPinCircle, 'Output Pin Circle')

        # 3. Root / valley circle (construction, on Od).
        rootCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(E, 0, 0), Rv)
        rootCircle.isConstruction = True
        constraints.addCoincident(rootCircle.centerSketchPoint, diskCentre)
        d = dims.addDiameterDimension(
            rootCircle, adsk.core.Point3D.create(E + Rv, 0, 0))
        d.parameter.expression = (
            '2 * ({} - {} - {} - {})'.format(
                self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
                self.parameterName(PARAM_PIN_RADIUS),
                self.parameterName(PARAM_DISK_CLEARANCE),
                self.parameterName(PARAM_ECCENTRICITY)))
        self._labelCircle(sketch, rootCircle, 'Root Circle')

        # 4. Lobe spline (open, adaptively sampled) — NO arc.
        kept = self.sample_lobe(E, 0.0, 0.0, R, self.Rr_eff, E, N, tEnd)
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in kept:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline = sketch.sketchCurves.sketchFittedSplines.add(coll)

        startPt = spline.fitPoints.item(0)
        endPt = spline.fitPoints.item(spline.fitPoints.count - 1)

        # 5. Lock the spline: fix interior fit points, coincide ends to root.
        for i in range(1, spline.fitPoints.count - 1):
            spline.fitPoints.item(i).isFixed = True
        constraints.addCoincident(startPt, rootCircle)
        constraints.addCoincident(endPt, rootCircle)

        # 6. Spoke line 1 + horizontal (from the disk centre).
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(E + Rv, 0, 0))
        constraints.addCoincident(line1.endSketchPoint, startPt)
        constraints.addHorizontal(line1)

        # 7. Spoke line 2 (from the disk centre).
        line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(
                E + Rv * math.cos(tEnd), -Rv * math.sin(tEnd), 0))
        constraints.addCoincident(line2.endSketchPoint, endPt)

        # 8. Lobe-pitch angle dim (driving), text in the minor wedge.
        textPoint = adsk.core.Point3D.create(
            E + 0.4 * Rv * math.cos(math.pi / L),
            -0.4 * Rv * math.sin(math.pi / L), 0)
        angDim = dims.addAngularDimension(line1, line2, textPoint)
        angDim.parameter.expression = '360 deg / {}'.format(
            self.parameterName(PARAM_LOBES))

        self.lobeSpline = spline
        self.lobeDiskCentre = diskCentre

    # -- step 2.A ------------------------------------------------------------
    def buildDisk(self):
        """Extrude the lobe pie-sector, build the disk axis from the body face,
        circular-pattern x L, and join into one disk body
        ([CYCLOIDAL-F-DISK-BODY])."""
        sketch = self.lobeSpline.parentSketch
        component = self.getComponent()
        L = self.N - 1

        # 1. Extrude the lobe sector (profile containing the lobe spline).
        sectorProfile = self._profileContaining(sketch, self.lobeSpline)
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
        extrude.bodies.item(0).name = 'Cycloidal Disk'

        # 2. Disk axis — from the extrude's planar cap (selected by NORMAL).
        self.buildDiskAxis(extrude.startFaces.item(0))

        # 3. Circular-pattern the extrude feature x L about the Disk Axis.
        circulars = component.features.circularPatternFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(extrude)
        pat = circulars.createInput(coll, self.diskAxis)
        pat.patternComputeOption = (
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute)
        pat.quantity = adsk.core.ValueInput.createByReal(L)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        circulars.add(pat)

        # 4. Combine x L -> one disk body.
        combines = component.features.combineFeatures
        target = component.bRepBodies.item(0)
        tools = adsk.core.ObjectCollection.create()
        for i in range(1, component.bRepBodies.count):
            tools.add(component.bRepBodies.item(i))
        ci = combines.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(ci)
        target.name = 'Cycloidal Disk'
        self.diskBody = target

    def buildDiskAxis(self, capFace):
        """Construction axis ⟂ the sketch at Od, from a BODY cap face selected
        by NORMAL ([CYCLOIDAL-F-DISK-AXIS])."""
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentre)
        axis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis'
        self.diskAxis = axis

    # -- step 3 --------------------------------------------------------------
    def buildOutputHoleSketch(self):
        """'Output Hole' (separate sketch): output-hole circle + one solid hole
        on Od ([CYCLOIDAL-F-OUTPUT-HOLE])."""
        sketch = self.createSketchObject('Output Hole', self.plane)
        sketch.isVisible = True
        circles = sketch.sketchCurves.sketchCircles
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._makeDiskCentre(sketch, localOrigin)

        E = self.E
        Rop = self.Rop
        D_hole = self.D_hole

        # 1. Output-hole circle (construction, on Od).
        outHoleCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(E, 0, 0), Rop)
        outHoleCircle.isConstruction = True
        constraints.addCoincident(outHoleCircle.centerSketchPoint, diskCentre)
        d = dims.addDiameterDimension(
            outHoleCircle, adsk.core.Point3D.create(E + Rop, 0, 0))
        d.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._labelCircle(sketch, outHoleCircle, 'Output Hole Circle')

        # 2. One hole (solid), on the Rop circle on the +X̂ ray from Od.
        hole = circles.addByCenterRadius(
            adsk.core.Point3D.create(E + Rop, 0, 0), D_hole / 2.0)
        d = dims.addDiameterDimension(
            hole, adsk.core.Point3D.create(E + Rop + D_hole / 2.0, 0, 0))
        d.parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        constraints.addCoincident(hole.centerSketchPoint, outHoleCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        spoke.isConstruction = True
        constraints.addHorizontal(spoke)
        self.outputHole = hole

    # -- step 4 --------------------------------------------------------------
    def buildOutputHoles(self):
        """Cut the output hole through the disk + pattern x M
        ([CYCLOIDAL-F-OUTPUT-HOLES])."""
        sketch = self.outputHole.parentSketch
        component = self.getComponent()
        M = self.M

        holeProfile = self._profileContaining(sketch, self.outputHole)
        extrudes = component.features.extrudeFeatures
        ci = extrudes.createInput(
            holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBody]
        cut = extrudes.add(ci)
        self.outputHoleCut = cut

        circulars = component.features.circularPatternFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(cut)
        pat = circulars.createInput(coll, self.diskAxis)
        pat.patternComputeOption = (
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute)
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        circulars.add(pat)

    # -- step 5 --------------------------------------------------------------
    def buildRingPins(self):
        """Ring pins + Housing Ring base on O ([CYCLOIDAL-F-RING-PINS])."""
        component = self.getComponent()
        R = self.R
        Rr = self.Rr
        N = self.N

        # 1. Housing construction plane (-1 mm, opposite the disk).
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane = component.constructionPlanes.add(pi)
        housingPlane.name = 'Ring Housing Plane'

        # 2. One sketch on that plane, anchored to O.
        sketch = self.createSketchObject('Housing Ring', housingPlane)
        sketch.isVisible = True
        circles = sketch.sketchCurves.sketchCircles
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        localOrigin = self._anchorLocalOrigin(sketch)

        outerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), R + Rr + self.getParameter(
                PARAM_WALL).value)
        constraints.addCoincident(outerCircle.centerSketchPoint, localOrigin)
        d = dims.addDiameterDimension(
            outerCircle,
            adsk.core.Point3D.create(outerCircle.radius, 0, 0))
        d.parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)

        innerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0),
            R - Rr - self.getParameter(PARAM_WALL).value)
        constraints.addCoincident(innerCircle.centerSketchPoint, localOrigin)
        d = dims.addDiameterDimension(
            innerCircle,
            adsk.core.Point3D.create(innerCircle.radius, 0, 0))
        d.parameter.expression = self.parameterName(PARAM_HOUSING_INNER_DIAMETER)

        # Projected pin circle (construction).
        projected = sketch.project(self.lobePinCircle)
        for i in range(projected.count):
            projected.item(i).isConstruction = True
        projPinCircle = projected.item(0)

        # Ring pin (solid) seated on the projected pin circle at (R, 0).
        pin = circles.addByCenterRadius(
            adsk.core.Point3D.create(R, 0, 0), Rr)
        d = dims.addDiameterDimension(
            pin, adsk.core.Point3D.create(R + Rr, 0, 0))
        d.parameter.expression = '2 * {}'.format(
            self.parameterName(PARAM_PIN_RADIUS))
        constraints.addCoincident(pin.centerSketchPoint, projPinCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        spoke.isConstruction = True
        constraints.addHorizontal(spoke)

        # 3. Housing extrude — every profile EXCEPT the central hole.
        extrudes = component.features.extrudeFeatures
        coll = adsk.core.ObjectCollection.create()
        centralHole = self._singleLoopProfileOf(sketch, innerCircle)
        for profile in sketch.profiles:
            if profile == centralHole:
                continue
            coll.add(profile)
        ext = extrudes.createInput(
            coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_BASE_THICKNESS))),
            adsk.fusion.ExtentDirections.NegativeExtentDirection)
        housingExtrude = extrudes.add(ext)
        housingExtrude.bodies.item(0).name = 'Housing Ring'
        self.housingRing = housingExtrude.bodies.item(0)

        # 4. Drive axis at O (cap selected by NORMAL).
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(
            housingExtrude.startFaces.item(0), localOrigin)
        driveAxis = component.constructionAxes.add(axInput)
        driveAxis.name = 'Drive Axis'
        self.driveAxis = driveAxis

        # 5. Ring pin extrude — two-sided full-length post (pin DISC).
        pinProfile = self._singleLoopProfileOf(sketch, pin)
        pinExt = extrudes.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sideOne = adsk.fusion.DistanceExtentDefinition.create(
            adsk.core.ValueInput.createByString(
                '{} + 1 mm'.format(self.parameterName(PARAM_DISC_THICKNESS))))
        sideTwo = adsk.fusion.DistanceExtentDefinition.create(
            adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_BASE_THICKNESS)))
        pinExt.setTwoSidesExtent(sideOne, sideTwo)
        pinFeature = extrudes.add(pinExt)
        pinFeature.bodies.item(0).name = 'Ring Pin'
        pinBody = pinFeature.bodies.item(0)

        # 6. Combine-cut a socket in the housing, keep the pin.
        combines = component.features.combineFeatures
        tools = adsk.core.ObjectCollection.create()
        tools.add(pinBody)
        ci = combines.createInput(self.housingRing, tools)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = True
        combineFeature = combines.add(ci)

        # 7. Chamfer the pin ends.
        chamferFeature = self._chamferCapRims(pinBody)

        # 8. Pattern the pin, socket AND chamfer x N about the drive axis.
        circulars = component.features.circularPatternFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature:
            coll.add(chamferFeature)
        pat = circulars.createInput(coll, self.driveAxis)
        pat.patternComputeOption = (
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute)
        pat.quantity = adsk.core.ValueInput.createByReal(N)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        circulars.add(pat)

    # -- step 6 --------------------------------------------------------------
    def buildCam(self):
        """Eccentric cam + disk center bore ([CYCLOIDAL-F-CAM])."""
        component = self.getComponent()
        extrudes = component.features.extrudeFeatures
        E = self.E
        centerBearing = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        bearingClear = self.getParameter(PARAM_BEARING_CLEARANCE).value
        inputShaft = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value

        # 1. Disk center bore — sketch 'Disk Bore' on self.plane.
        boreSketch = self.createSketchObject('Disk Bore', self.plane)
        boreSketch.isVisible = True
        bConstraints = boreSketch.geometricConstraints
        bDims = boreSketch.sketchDimensions
        bLocalOrigin = self._anchorLocalOrigin(boreSketch)
        bDiskCentre = self._makeDiskCentre(boreSketch, bLocalOrigin)

        boreRadius = (centerBearing + bearingClear) / 2.0
        boreCircle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(E, 0, 0), boreRadius)
        bConstraints.addCoincident(boreCircle.centerSketchPoint, bDiskCentre)
        d = bDims.addDiameterDimension(
            boreCircle, adsk.core.Point3D.create(E + boreRadius, 0, 0))
        d.parameter.expression = '{} + {}'.format(
            self.parameterName(PARAM_CENTER_BEARING_DIAMETER),
            self.parameterName(PARAM_BEARING_CLEARANCE))

        coll = adsk.core.ObjectCollection.create()
        for profile in boreSketch.profiles:
            coll.add(profile)
        ci = extrudes.createInput(
            coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBody]
        extrudes.add(ci)

        # 2. Eccentric Cam — sketch 'Eccentric Cam' on self.plane.
        camSketch = self.createSketchObject('Eccentric Cam', self.plane)
        camSketch.isVisible = True
        cConstraints = camSketch.geometricConstraints
        cDims = camSketch.sketchDimensions
        cLocalOrigin = self._anchorLocalOrigin(camSketch)
        cDiskCentre = self._makeDiskCentre(camSketch, cLocalOrigin)
        camCircles = camSketch.sketchCurves.sketchCircles

        camOuterRadius = centerBearing / 2.0
        camOuter = camCircles.addByCenterRadius(
            adsk.core.Point3D.create(E, 0, 0), camOuterRadius)
        cConstraints.addCoincident(camOuter.centerSketchPoint, cDiskCentre)
        d = cDims.addDiameterDimension(
            camOuter, adsk.core.Point3D.create(E + camOuterRadius, 0, 0))
        d.parameter.expression = self.parameterName(
            PARAM_CENTER_BEARING_DIAMETER)

        hasBore = inputShaft > 0
        if hasBore:
            inputBore = camCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), inputShaft / 2.0)
            cConstraints.addCoincident(
                inputBore.centerSketchPoint, cLocalOrigin)
            d = cDims.addDiameterDimension(
                inputBore, adsk.core.Point3D.create(inputShaft / 2.0, 0, 0))
            d.parameter.expression = self.parameterName(
                PARAM_INPUT_SHAFT_DIAMETER)

        # Cam cross-section: 2-loop annulus when bored, else 1-loop disc.
        camProfile = None
        if hasBore:
            for profile in camSketch.profiles:
                if profile.profileLoops.count == 2:
                    camProfile = profile
                    break
            if camProfile is None:
                raise Exception(
                    'Eccentric Cam: 2-loop cam annulus profile not found')
        else:
            if camSketch.profiles.count != 1:
                raise Exception(
                    'Eccentric Cam: expected one profile for the solid cam '
                    'disc, found {}'.format(camSketch.profiles.count))
            camProfile = camSketch.profiles.item(0)

        camExt = extrudes.createInput(
            camProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        camExt.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        camFeature = extrudes.add(camExt)
        camFeature.bodies.item(0).name = 'Eccentric Cam'
        self.cam = camFeature.bodies.item(0)

    # -- step 7 --------------------------------------------------------------
    def buildOutputPins(self):
        """Output pins + plate on O ([CYCLOIDAL-F-OUTPUT-PINS])."""
        component = self.getComponent()
        extrudes = component.features.extrudeFeatures
        Rop = self.Rop
        D_pin = self.D_pin
        M = self.M

        # 1. Output plate plane (DiscThickness + 1 mm above the disk).
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane, adsk.core.ValueInput.createByString(
                '{} + 1 mm'.format(self.parameterName(PARAM_DISC_THICKNESS))))
        platePlane = component.constructionPlanes.add(pi)
        platePlane.name = 'Output Plate Plane'

        # 2. One sketch 'Output Plate', anchored to O.
        sketch = self.createSketchObject('Output Plate', platePlane)
        sketch.isVisible = True
        circles = sketch.sketchCurves.sketchCircles
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        localOrigin = self._anchorLocalOrigin(sketch)

        plateDiameter = (
            self.getParameter(PARAM_OUTPUT_PLATE_DIAMETER).value)
        plateOuter = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), plateDiameter / 2.0)
        constraints.addCoincident(plateOuter.centerSketchPoint, localOrigin)
        d = dims.addDiameterDimension(
            plateOuter, adsk.core.Point3D.create(plateDiameter / 2.0, 0, 0))
        d.parameter.expression = self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)

        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), Rop)
        outPinCircle.isConstruction = True
        constraints.addCoincident(outPinCircle.centerSketchPoint, localOrigin)
        d = dims.addDiameterDimension(
            outPinCircle, adsk.core.Point3D.create(Rop, 0, 0))
        d.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)

        pin = circles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2.0)
        d = dims.addDiameterDimension(
            pin, adsk.core.Point3D.create(Rop + D_pin / 2.0, 0, 0))
        d.parameter.expression = '{} - 2 * {}'.format(
            self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
            self.parameterName(PARAM_ECCENTRICITY))
        constraints.addCoincident(pin.centerSketchPoint, outPinCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        spoke.isConstruction = True
        constraints.addHorizontal(spoke)

        # 3. Output plate body — all profiles, away from the disk (+normal).
        coll = adsk.core.ObjectCollection.create()
        for profile in sketch.profiles:
            coll.add(profile)
        ext = extrudes.createInput(
            coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        plateExtrude = extrudes.add(ext)
        plateExtrude.bodies.item(0).name = 'Output Plate'
        self.outputPlate = plateExtrude.bodies.item(0)

        # 4. Output pin extrude — two-sided (pin DISC).
        pinProfile = self._singleLoopProfileOf(sketch, pin)
        pinExt = extrudes.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sideOne = adsk.fusion.DistanceExtentDefinition.create(
            adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS)))
        sideTwo = adsk.fusion.DistanceExtentDefinition.create(
            adsk.core.ValueInput.createByString(
                '{} + 1 mm'.format(self.parameterName(PARAM_DISC_THICKNESS))))
        pinExt.setTwoSidesExtent(sideOne, sideTwo)
        pinFeature = extrudes.add(pinExt)
        pinFeature.bodies.item(0).name = 'Output Pin'
        pinBody = pinFeature.bodies.item(0)

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

        # 7. Pattern pin, socket AND chamfer x M about the drive axis.
        circulars = component.features.circularPatternFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature:
            coll.add(chamferFeature)
        pat = circulars.createInput(coll, self.driveAxis)
        pat.patternComputeOption = (
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute)
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        circulars.add(pat)

    # -- step 8 --------------------------------------------------------------
    def buildChamfers(self):
        """Outer-rim chamfers on the disc-like bodies ([CYCLOIDAL-F-CHAMFERS])."""
        if self.chamferSize <= 0:
            return
        self._chamferCapRims(self.diskBody)
        self._chamferCapRims(self.housingRing)
        self._chamferCapRims(self.outputPlate)
