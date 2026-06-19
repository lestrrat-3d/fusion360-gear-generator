"""Cycloidal drive generator.

Generated purely from spec/cycloidal/instructions.md (+ fusion.md,
epitrochoid-trace.md) and PLAYBOOK.md. This module does NOT subclass any gear
class and shares no involute math: it subclasses base.Generator directly and
uses the generic occurrence / prefixed-user-parameter plumbing only.
"""

import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm
from .base import Generator, get_value, get_selection
from . import solids


# ---------------------------------------------------------------------------
# Dialog input ids (verbatim from the spec's "Exact input ids" table)
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

STATUS_SUFFIX = '__status'

# ---------------------------------------------------------------------------
# Registered user-parameter names (logical; the live names are prefixed
# `CycloidalDrive<N>_…` via self.parameterName()).
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

# Derived parameters
PARAM_LOBES = 'Lobes'
PARAM_PIN_CIRCLE_RADIUS = 'PinCircleRadius'
PARAM_OUTPUT_PIN_CIRCLE_RADIUS = 'OutputPinCircleRadius'
PARAM_PIN_RADIUS = 'PinRadius'
PARAM_OUTPUT_HOLE_DIAMETER = 'OutputHoleDiameter'
PARAM_HOUSING_INNER_DIAMETER = 'HousingInnerDiameter'
PARAM_HOUSING_OUTER_DIAMETER = 'HousingOuterDiameter'
PARAM_OUTPUT_PLATE_DIAMETER = 'OutputPlateDiameter'


# ===========================================================================
# Profile math — reproduced verbatim from epitrochoid-trace.md
# ===========================================================================
def disk_point(t, cx, cy, phi, R, Rr_eff, E, N):
    """The rotor profile point in internal cm. The equidistant of a shortened
    epitrochoid (epitrochoid-trace.md "The point function")."""
    num = math.sin((1 - N) * t)
    den = (R / (E * N)) - math.cos((1 - N) * t)
    psi = math.atan2(num, den)              # uses R, E, N only — NOT Rr
    x0 = R * math.cos(t) - Rr_eff * math.cos(t + psi) - E * math.cos(N * t)
    y0 = -R * math.sin(t) + Rr_eff * math.sin(t + psi) + E * math.sin(N * t)
    x = cx + (x0 * math.cos(phi) - y0 * math.sin(phi))
    y = cy + (x0 * math.sin(phi) + y0 * math.cos(phi))
    return (x, y)


def _rho_min_O(R, E, N):
    """The smallest |radius of curvature| of the base trochoid at points whose
    centre of curvature lies toward O (epitrochoid-trace.md "No-undercut
    guard"). Returns +inf if no such point is found."""
    best = float('inf')
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
        if abs(k) < 1e-12:
            continue
        speed2 = xp * xp + yp * yp
        rho = (speed2 ** 1.5) / k
        s = math.sqrt(speed2)
        nx, ny = -yp / s, xp / s
        cx = bx + rho * nx
        cy = by + rho * ny
        if (cx * cx + cy * cy) < (bx * bx + by * by):
            if abs(rho) < best:
                best = abs(rho)
    return best


def _resolve_Rr(R, E, N, pinDiameter):
    """Ring-pin radius: override Pin Diameter/2 if > 0, else the bounds-mean."""
    if pinDiameter > 0:
        return pinDiameter / 2.0
    return 0.5 * (E + R * math.sin(math.pi / N))


def _resolve_D_pin(Rop, E, M, outputPinDiameter):
    """Output-pin diameter: override if > 0, else the bounds-mean."""
    if outputPinDiameter > 0:
        return outputPinDiameter
    return Rop * math.sin(math.pi / M) - E


def _adaptive_lobe_points(cx, cy, phi, R, Rr_eff, E, N, L):
    """Adaptively-sampled lobe points (epitrochoid-trace.md "Sampling"): keep
    points so consecutive segments subtend a bounded turn angle (<= ~5 deg).
    Returns a list of (x, y) in cm spanning t in [0, 2*pi/L]."""
    fine = 2000
    span = 2.0 * math.pi / L
    pts = [disk_point(span * i / fine, cx, cy, phi, R, Rr_eff, E, N)
           for i in range(fine + 1)]
    threshold = math.radians(5.0)
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
        if accum >= threshold:
            kept.append(pts[i])
            accum = 0.0
    kept.append(pts[-1])
    return kept


def _contour_points(R, Rr_eff, E, N, L, c):
    """One pin-pitch of the pinless casing inner-wall contour
    (epitrochoid-trace.md "Pinless ring casing"): sweep the disc over a full
    cam cycle, bin by polar angle within [-pi/N, pi/N], keep max radius per
    bin, then emit nbins+1 points at the bin EDGES so the first/last land
    exactly on -+pi/N. Returns a list of (x, y) in cm ordered by angle."""
    nbins = 80
    binMax = [0.0] * nbins
    binHit = [False] * nbins
    lo = -math.pi / N
    hi = math.pi / N
    width = hi - lo
    Ntheta = 240
    Nt = 240
    for it in range(Ntheta):
        theta = 2.0 * math.pi * it / Ntheta
        cx = E * math.cos(theta)
        cy = E * math.sin(theta)
        phi = -theta / L
        for jt in range(Nt):
            t = 2.0 * math.pi * jt / Nt
            (x, y) = disk_point(t, cx, cy, phi, R, Rr_eff, E, N)
            a = math.atan2(y, x)
            if lo <= a <= hi:
                b = int((a - lo) / width * nbins)
                if b < 0:
                    b = 0
                if b >= nbins:
                    b = nbins - 1
                r = math.hypot(x, y)
                if r > binMax[b]:
                    binMax[b] = r
                binHit[b] = True
    pts = []
    for i in range(nbins + 1):
        phi_i = lo + width * i / nbins
        # radius at edge i = c + max(binMax[i-1], binMax[i]) (single neighbour
        # at the two ends).
        candidates = []
        if i - 1 >= 0 and binHit[i - 1]:
            candidates.append(binMax[i - 1])
        if i < nbins and binHit[i]:
            candidates.append(binMax[i])
        rmax = max(candidates) if candidates else 0.0
        r_i = c + rmax
        pts.append((r_i * math.cos(phi_i), r_i * math.sin(phi_i)))
    return pts


def evaluate_problems(vals):
    """The SINGLE source of validity math. Takes a dict of resolved scalar
    values (internal cm / ints) and returns a list of actionable problem
    strings (empty = valid). Shared by validate_inputs (raw read) and
    _resolveDimensions (registered-parameter read), so the two never drift.

    Required keys: R, Rop, E, c, N, M, D (disc count), CBD, clr, ISD, Rr,
    Rr_eff, Rv, D_pin, D_hole, pinDiameter, outputPinDiameter, rho_min_O,
    Estar (None if no positive solution).
    """
    problems = []
    R = vals['R']
    Rop = vals['Rop']
    E = vals['E']
    N = vals['N']
    M = vals['M']
    D = vals['D']
    CBD = vals['CBD']
    clr = vals['clr']
    ISD = vals['ISD']
    Rr = vals['Rr']
    Rr_eff = vals['Rr_eff']
    Rv = vals['Rv']
    D_pin = vals['D_pin']
    D_hole = vals['D_hole']
    pinDiameter = vals['pinDiameter']
    outputPinDiameter = vals['outputPinDiameter']
    rho_min_O = vals['rho_min_O']
    Estar = vals['Estar']

    sinN = math.sin(math.pi / N)
    sinM = math.sin(math.pi / M)

    # Two-disc evenness first (it needs only N, M, the dropdown).
    if D == 2 and (N % 2 != 0 or M % 2 != 0):
        problems.append(
            'Two discs require an even Pin Count and an even Output Pin Count '
            '(currently N={}, M={}).'.format(N, M))

    # E < Rr < R*sin(pi/N)
    if not (E < Rr < R * sinN):
        if pinDiameter <= 0:
            problems.append(
                'Pin geometry out of range — increase Pin Circle Diameter '
                'above {} mm or reduce Eccentricity below {} mm.'.format(
                    round(to_mm(2 * E / sinN), 2),
                    round(to_mm(R * sinN), 2)))
        else:
            problems.append(
                'Pin Diameter must be between {} mm and {} mm (currently '
                '{} mm).'.format(
                    round(to_mm(2 * E), 2),
                    round(to_mm(2 * R * sinN), 2),
                    round(to_mm(2 * Rr), 2)))

    # D_pin > 0
    if not (D_pin > 0):
        if outputPinDiameter <= 0:
            problems.append(
                'Output pins vanish (resolved diameter <= 0) — increase Output '
                'Pin Circle Diameter above {} mm, increase Output Pin Count, '
                'or reduce Eccentricity.'.format(
                    round(to_mm(2 * E / sinM), 2)))
        else:
            problems.append('Output Pin Diameter must be greater than 0.')

    # D_hole < 2*Rop*sin(pi/M)
    if not (D_hole < 2 * Rop * sinM):
        problems.append(
            'Output holes overlap — increase Output Pin Circle Diameter above '
            '{} mm, increase Output Pin Count, or reduce Output Pin Diameter / '
            'Eccentricity.'.format(round(to_mm((D_pin + 2 * E) / sinM), 2)))

    # E < R/N
    if not (E < R / N):
        problems.append(
            'Eccentricity too large — reduce it below {} mm (or increase Pin '
            'Circle Diameter / reduce Pin Count).'.format(
                round(to_mm(R / N), 2)))

    # Rop < Rv
    if not (Rop < Rv):
        problems.append(
            'Output Pin Circle too large — set Output Pin Circle Diameter '
            'below {} mm (currently {} mm).'.format(
                round(to_mm(2 * Rv), 2), round(to_mm(2 * Rop), 2)))

    # no-undercut: Rr_eff < rho_min_O
    if not (Rr_eff < rho_min_O):
        if Estar is not None:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/'
                'self-intersects. Reduce Eccentricity below {} mm.'.format(
                    round(to_mm(Estar), 2)))
        else:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/'
                'self-intersects. Reduce Eccentricity.')

    # ISD < CBD
    if not (ISD < CBD):
        problems.append(
            'Input Shaft Diameter must be less than Center Bearing Diameter '
            '({} mm).'.format(round(to_mm(CBD), 2)))

    # E + ISD/2 < CBD/2
    if not (E + ISD / 2.0 < CBD / 2.0):
        problems.append(
            "Input bore doesn't fit inside the cam — set Input Shaft Diameter "
            'below {} mm, or reduce Eccentricity / increase Center Bearing '
            'Diameter.'.format(round(to_mm(CBD - 2 * E), 2)))

    # (CBD + clr)/2 < Rop - D_hole/2
    if not ((CBD + clr) / 2.0 < Rop - D_hole / 2.0):
        problems.append(
            'Disk center bore overlaps the output holes — increase Output Pin '
            'Circle Diameter above {} mm, or reduce Center Bearing Diameter / '
            'Bearing Clearance / output pin size.'.format(
                round(to_mm(CBD + clr + D_hole), 2)))

    return problems


def _solve_Estar(R, E, N, c, pinDiameter):
    """Largest E* in (0, E] for which Rr_eff(E') < rho_min_O(E') still holds,
    by ~40-iteration bisection (instructions.md "Undercut bound E* (numeric)").
    Returns None if no positive E' satisfies it. Note both Rr_eff and rho_min_O
    depend on E' when Pin Diameter = 0."""
    def ok(Ep):
        if Ep <= 0:
            return False
        Rr = _resolve_Rr(R, Ep, N, pinDiameter)
        Rr_eff = Rr + c
        return Rr_eff < _rho_min_O(R, Ep, N)

    # Find the boundary in (0, E]. If even a tiny E' fails, give up.
    eps = E * 1e-4
    if eps <= 0:
        return None
    if not ok(eps):
        return None
    lo = eps
    hi = E
    # If the full E is already OK there is nothing to report; the caller only
    # asks for E* when the guard fails, but be robust anyway.
    if ok(hi):
        return hi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def _build_vals(R, Rop, E, c, N, M, D, CBD, clr, ISD, pinDiameter,
                outputPinDiameter):
    """Resolve the same derived dimensions both validation paths need, from raw
    scalar inputs in internal cm / ints."""
    Rr = _resolve_Rr(R, E, N, pinDiameter)
    Rr_eff = Rr + c
    Rv = R - Rr_eff - E
    D_pin = _resolve_D_pin(Rop, E, M, outputPinDiameter)
    D_hole = D_pin + 2 * E
    rho_min_O = _rho_min_O(R, E, N)
    Estar = None
    if not (Rr_eff < rho_min_O):
        Estar = _solve_Estar(R, E, N, c, pinDiameter)
    return {
        'R': R, 'Rop': Rop, 'E': E, 'c': c, 'N': N, 'M': M, 'D': D,
        'CBD': CBD, 'clr': clr, 'ISD': ISD, 'Rr': Rr, 'Rr_eff': Rr_eff,
        'Rv': Rv, 'D_pin': D_pin, 'D_hole': D_hole,
        'pinDiameter': pinDiameter, 'outputPinDiameter': outputPinDiameter,
        'rho_min_O': rho_min_O, 'Estar': Estar,
    }


# ===========================================================================
# Command-inputs configurator
# ===========================================================================
class CycloidalDriveCommandInputsConfigurator:
    @classmethod
    def configure(cls, command):
        from .misc import get_design
        inputs = command.commandInputs

        # --- Selections (first two) ---
        planeSel = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Plane the rotor lobe sits on')
        planeSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.PlanarFaces)
        planeSel.setSelectionLimits(1, 1)

        anchorSel = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Drive axis anchor')
        anchorSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.SketchPoints)
        anchorSel.setSelectionLimits(1, 1)

        # --- Disc Count dropdown (1/2) ---
        disc = inputs.addDropDownCommandInput(
            INPUT_ID_DISC_COUNT, 'Disc Count',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        disc.listItems.add('1', True)
        disc.listItems.add('2', False)
        cls._add_status_slot(inputs, INPUT_ID_DISC_COUNT)

        # --- Value inputs (in dialog order), each followed by its status slot ---
        def addValue(iid, label, default, unit='mm'):
            inputs.addValueInput(
                iid, label, unit,
                adsk.core.ValueInput.createByReal(default))
            cls._add_status_slot(inputs, iid)

        addValue(INPUT_ID_PIN_COUNT, 'Pin Count', 16, '')
        addValue(INPUT_ID_PIN_CIRCLE_DIAMETER, 'Pin Circle Diameter', to_cm(90))
        addValue(INPUT_ID_PIN_DIAMETER, 'Pin Diameter', 0)
        addValue(INPUT_ID_ECCENTRICITY, 'Eccentricity', to_cm(1.5))
        addValue(INPUT_ID_DISK_CLEARANCE, 'Disk Clearance', to_cm(0.3))
        addValue(INPUT_ID_DISC_THICKNESS, 'Disc Thickness', to_cm(8))
        addValue(INPUT_ID_DISC_GAP, 'Disc Gap', to_cm(0.5))
        addValue(INPUT_ID_CENTER_BEARING_DIAMETER, 'Center Bearing Diameter',
                 to_cm(30))
        addValue(INPUT_ID_INPUT_SHAFT_DIAMETER, 'Input Shaft Diameter',
                 to_cm(8))
        addValue(INPUT_ID_BEARING_CLEARANCE, 'Bearing Clearance', to_cm(0.2))
        addValue(INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER,
                 'Output Pin Circle Diameter', to_cm(50))
        addValue(INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', 6, '')
        addValue(INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 0)
        addValue(INPUT_ID_WALL, 'Housing Wall', to_cm(3))
        addValue(INPUT_ID_BASE_THICKNESS, 'Base Thickness', to_cm(5))
        addValue(INPUT_ID_OUTPUT_PLATE_THICKNESS, 'Output Plate Thickness',
                 to_cm(5))
        addValue(INPUT_ID_CHAMFER_SIZE, 'Chamfer Size', to_cm(0.5))

        # --- Parent Component (last) ---
        parentSel = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Parent component')
        parentSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.Occurrences)
        parentSel.addSelectionFilter(
            adsk.core.SelectionCommandInput.RootComponents)
        parentSel.setSelectionLimits(0, 1)
        parentSel.addSelection(get_design().rootComponent)

    @staticmethod
    def _add_status_slot(inputs, baseId):
        slot = inputs.addTextBoxCommandInput(
            baseId + STATUS_SUFFIX, '', '', 2, True)
        slot.isVisible = False
        return slot


# ===========================================================================
# Generator
# ===========================================================================
class CycloidalDriveGenerator(Generator):
    # Live-validation contract consulted by the shared GearCommand
    # ([PB-VALIDATE-INPUTS]).
    DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + STATUS_SUFFIX

    def prefixBase(self):
        return 'CycloidalDrive'

    # ----- live input validation -----
    @staticmethod
    def validate_inputs(inputs):
        """Pure check (no document writes / parameter registration). Reads raw
        values straight off the command inputs (internal cm for lengths, the
        rounded int for counts, the dropdown selectedItem name for Disc Count),
        resolves the same derived dimensions as _resolveDimensions, and returns
        a list of actionable problems (empty = valid). Runs on every keystroke;
        a half-typed expression raising is caught by the shared handler."""
        R = inputs.itemById(INPUT_ID_PIN_CIRCLE_DIAMETER).value / 2.0
        Rop = inputs.itemById(INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2.0
        E = inputs.itemById(INPUT_ID_ECCENTRICITY).value
        c = inputs.itemById(INPUT_ID_DISK_CLEARANCE).value
        N = int(round(inputs.itemById(INPUT_ID_PIN_COUNT).value))
        M = int(round(inputs.itemById(INPUT_ID_OUTPUT_PIN_COUNT).value))
        D = int(inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name)
        CBD = inputs.itemById(INPUT_ID_CENTER_BEARING_DIAMETER).value
        clr = inputs.itemById(INPUT_ID_BEARING_CLEARANCE).value
        ISD = inputs.itemById(INPUT_ID_INPUT_SHAFT_DIAMETER).value
        pinDiameter = inputs.itemById(INPUT_ID_PIN_DIAMETER).value
        outputPinDiameter = inputs.itemById(INPUT_ID_OUTPUT_PIN_DIAMETER).value

        vals = _build_vals(R, Rop, E, c, N, M, D, CBD, clr, ISD,
                           pinDiameter, outputPinDiameter)
        return evaluate_problems(vals)

    # ----- top-level generate -----
    def generate(self, inputs):
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()

        # Normalize the Target Plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(
                self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)

        D = self.discCount
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

        if self.chamfersSkipped > 0:
            adsk.core.Application.get().userInterface.messageBox(
                'Cycloidal drive generated, but {n} chamfer(s) could not be '
                'created at Chamfer Size {sz} mm and were skipped. Reduce '
                'Chamfer Size (or set it to 0) for this geometry.'.format(
                    n=self.chamfersSkipped, sz=to_mm(self.chamferSize)))

    def generateName(self):
        N = self.N
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    # ----- inputs / parameters -----
    def processInputs(self, inputs):
        from .misc import get_design

        # Pull both selection inputs AND Parent and stash on self BEFORE any
        # parameter registration (occurrence creation shifts active component).
        parentSel = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentSel) == 0:
            self.parentComponent = get_design().rootComponent
        else:
            ent = parentSel[0]
            if ent.objectType == adsk.fusion.Occurrence.classType():
                self.parentComponent = ent.component
            else:
                self.parentComponent = ent

        planeSel = get_selection(inputs, INPUT_ID_PLANE)
        self.plane = planeSel[0]
        anchorSel = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        self.anchorPoint = anchorSel[0]

        # Disc Count dropdown -> int.
        self.discCount = int(
            inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name)

        # Counts read from the dialog rounded to int for the Python formulas.
        self.N = int(round(inputs.itemById(INPUT_ID_PIN_COUNT).value))
        self.M = int(round(inputs.itemById(INPUT_ID_OUTPUT_PIN_COUNT).value))

        # Resilient-chamfer counter — initialized before any build step.
        self.chamfersSkipped = 0
        self.chamferSize = inputs.itemById(INPUT_ID_CHAMFER_SIZE).value

        # Register primary numeric user parameters.
        self.addParameter(PARAM_PIN_COUNT,
                          get_value(inputs, INPUT_ID_PIN_COUNT, ''), '',
                          'Number of ring pins')
        self.addParameter(PARAM_PIN_CIRCLE_DIAMETER,
                          get_value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'mm'),
                          'mm', 'Pin circle diameter')
        self.addParameter(PARAM_PIN_DIAMETER,
                          get_value(inputs, INPUT_ID_PIN_DIAMETER, 'mm'),
                          'mm', 'Ring-pin diameter (0=auto)')
        self.addParameter(PARAM_ECCENTRICITY,
                          get_value(inputs, INPUT_ID_ECCENTRICITY, 'mm'),
                          'mm', 'Eccentricity')
        self.addParameter(PARAM_DISK_CLEARANCE,
                          get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'),
                          'mm', 'Disk clearance')
        self.addParameter(PARAM_DISC_THICKNESS,
                          get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'),
                          'mm', 'Disc thickness')
        self.addParameter(PARAM_DISC_GAP,
                          get_value(inputs, INPUT_ID_DISC_GAP, 'mm'),
                          'mm', 'Disc gap')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Output pin circle diameter')
        self.addParameter(PARAM_OUTPUT_PIN_COUNT,
                          get_value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, ''), '',
                          'Number of output pins')
        self.addParameter(PARAM_OUTPUT_PIN_DIAMETER,
                          get_value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'mm'),
                          'mm', 'Output pin diameter (0=auto)')
        self.addParameter(
            PARAM_CENTER_BEARING_DIAMETER,
            get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'),
            'mm', 'Center bearing diameter')
        self.addParameter(PARAM_INPUT_SHAFT_DIAMETER,
                          get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'),
                          'mm', 'Input shaft diameter (0=no bore)')
        self.addParameter(PARAM_BEARING_CLEARANCE,
                          get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'),
                          'mm', 'Bearing clearance')
        self.addParameter(PARAM_WALL,
                          get_value(inputs, INPUT_ID_WALL, 'mm'),
                          'mm', 'Housing wall thickness')
        self.addParameter(PARAM_BASE_THICKNESS,
                          get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'),
                          'mm', 'Base thickness')
        self.addParameter(PARAM_OUTPUT_PLATE_THICKNESS,
                          get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS,
                                    'mm'),
                          'mm', 'Output plate thickness')
        self.addParameter(PARAM_CHAMFER_SIZE,
                          get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'),
                          'mm', 'Chamfer size')

        # Now compute the resolved dimensions (raises evaluate_problems joined).
        self._resolveDimensions()

        # Register derived parameters.
        # Lobes, PinCircleRadius, OutputPinCircleRadius, Housing*, OutputPlate*
        # stay LIVE createByString expressions composing from the inputs.
        self.addParameter(
            PARAM_LOBES,
            adsk.core.ValueInput.createByString(
                '{} - 1'.format(self.parameterName(PARAM_PIN_COUNT))),
            '', 'Number of rotor lobes')
        self.addParameter(
            PARAM_PIN_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_DIAMETER))),
            'mm', 'Pin circle radius')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString(
                '{} / 2'.format(
                    self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER))),
            'mm', 'Output pin circle radius')

        # PinRadius and OutputHoleDiameter MUST be createByReal numeric
        # snapshots of the Python-resolved values (auto-vs-override branch a
        # live expression can't reproduce).
        self.addParameter(
            PARAM_PIN_RADIUS,
            adsk.core.ValueInput.createByReal(self.Rr), 'mm',
            'Resolved ring-pin radius (snapshot)')
        self.addParameter(
            PARAM_OUTPUT_HOLE_DIAMETER,
            adsk.core.ValueInput.createByReal(self.D_hole), 'mm',
            'Resolved output-hole diameter (snapshot)')

        # HousingInnerDiameter = 2*(R - PinRadius - Wall)
        self.addParameter(
            PARAM_HOUSING_INNER_DIAMETER,
            adsk.core.ValueInput.createByString(
                '2 * ({} - {} - {})'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
                    self.parameterName(PARAM_PIN_RADIUS),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Housing inner diameter')
        # HousingOuterDiameter = 2*(R - PinRadius + 2*E + Wall)
        self.addParameter(
            PARAM_HOUSING_OUTER_DIAMETER,
            adsk.core.ValueInput.createByString(
                '2 * ({} - {} + 2 * {} + {})'.format(
                    self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
                    self.parameterName(PARAM_PIN_RADIUS),
                    self.parameterName(PARAM_ECCENTRICITY),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Housing outer diameter')
        # OutputPlateDiameter = 2*Rop + D_pin + 2*Wall
        #   where D_pin = OutputHoleDiameter - 2*Eccentricity
        self.addParameter(
            PARAM_OUTPUT_PLATE_DIAMETER,
            adsk.core.ValueInput.createByString(
                '{} + ({} - 2 * {}) + 2 * {}'.format(
                    self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER),
                    self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
                    self.parameterName(PARAM_ECCENTRICITY),
                    self.parameterName(PARAM_WALL))),
            'mm', 'Output plate diameter')

        # Per-disc list stashes.
        D = self.discCount
        self.diskBodies = [None] * D
        self.diskAxes = [None] * D
        self.lobeSplines = [None] * D
        self.outputHoles = [None] * D
        self.lobeDiskCentres = [None] * D
        self.discPlanes = [None] * D
        # Scalar stashes.
        self.lobePinCircle = None
        self.driveAxis = None
        self.housingRing = None
        self.ringCasing = None
        self.cam = None
        self.outputPlate = None

    def _resolveDimensions(self):
        """Build the resolved scalar values from the registered parameters and,
        if evaluate_problems is non-empty, raise the joined messages — the same
        checks validate_inputs shows live."""
        R = self.getParameter(PARAM_PIN_CIRCLE_DIAMETER).value / 2.0
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2.0
        E = self.getParameter(PARAM_ECCENTRICITY).value
        c = self.getParameter(PARAM_DISK_CLEARANCE).value
        N = self.N
        M = self.M
        D = self.discCount
        CBD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        clr = self.getParameter(PARAM_BEARING_CLEARANCE).value
        ISD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        pinDiameter = self.getParameter(PARAM_PIN_DIAMETER).value
        outputPinDiameter = self.getParameter(PARAM_OUTPUT_PIN_DIAMETER).value

        vals = _build_vals(R, Rop, E, c, N, M, D, CBD, clr, ISD,
                           pinDiameter, outputPinDiameter)
        problems = evaluate_problems(vals)
        if problems:
            raise Exception('\n'.join(problems))

        # Stash the resolved scalars (internal cm) on self for the build steps.
        self.R = R
        self.Rop = Rop
        self.E = E
        self.c = c
        self.Rr = vals['Rr']
        self.Rr_eff = vals['Rr_eff']
        self.Rv = vals['Rv']
        self.D_pin = vals['D_pin']
        self.D_hole = vals['D_hole']

    # ----- per-disc geometry helpers (§0) -----
    def _discPlane(self, d):
        """plane(d): self.plane for d==0; else a construction plane offset
        z_d = d*(T+g) with PREFIXED param names."""
        if d == 0:
            self.discPlanes[d] = self.plane
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
        self.discPlanes[d] = plane
        return plane

    def _stackTopExpr(self):
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        if self.discCount == 1:
            return nT
        return '2 * {} + {}'.format(nT, nG)

    def _anchorLocalOrigin(self, sketch):
        """Project the user's Anchor and constrain a fresh local origin to it
        ([CYCLOIDAL-F-ANCHOR-CHAIN]). Returns the local origin SketchPoint."""
        localOrigin = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))
        projected = sketch.project(self.anchorPoint).item(0)
        sketch.geometricConstraints.addCoincident(localOrigin, projected)
        return localOrigin

    def _eccentricCentre(self, sketch, localOrigin, signedE):
        """Build the eccentric disk centre Od = O + s_d*E*Xhat
        ([CYCLOIDAL-F-DISK-CENTER]). signedE in cm with sign. Returns the
        diskCentre SketchPoint."""
        diskCentre = sketch.sketchPoints.add(
            adsk.core.Point3D.create(signedE, 0, 0))
        eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        dim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(signedE / 2.0, 0.2 * abs(signedE) + 0.1, 0))
        dim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return diskCentre

    def _alongPathLabel(self, sketch, circle, text):
        ti = sketch.sketchTexts.createInput2(text, self.Rr)
        ti.setAsAlongPath(
            circle, True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(ti)

    def _diam_text_point(self, cx, cy, r):
        # An off-centre text point on/near the circle ([PB-RADIAL-DIM]).
        return adsk.core.Point3D.create(cx + r, cy, 0)

    # ----- §2: lobe sketch -----
    def buildLobeSketch(self, d):
        component = self.getComponent()
        plane = self._discPlane(d)
        sketch = self.createSketchObject('Rotor Lobe {}'.format(d + 1), plane)
        sketch.isVisible = True

        s_d = 1.0 if d == 0 else -1.0
        phi = d * math.pi
        E = self.E
        signedE = s_d * E
        R = self.R
        Rop = self.Rop
        Rv = self.Rv
        N = self.N
        L = N - 1

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._eccentricCentre(sketch, localOrigin, signedE)

        circles = sketch.sketchCurves.sketchCircles

        # 1. pin circle (construction, on O), d==0 stashed on lobePinCircle.
        pinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), R)
        pinCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            pinCircle.centerSketchPoint, localOrigin)
        pd = sketch.sketchDimensions.addDiameterDimension(
            pinCircle, self._diam_text_point(0, 0, R))
        pd.parameter.expression = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, pinCircle, 'Pin Circle')
        if d == 0:
            self.lobePinCircle = pinCircle

        # 2. output-pin circle (construction, on Od).
        opCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rop)
        opCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            opCircle.centerSketchPoint, diskCentre)
        opd = sketch.sketchDimensions.addDiameterDimension(
            opCircle, self._diam_text_point(signedE, 0, Rop))
        opd.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, opCircle, 'Output Pin Circle')

        # 3. root/valley circle (construction, on Od).
        rootCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rv)
        rootCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            rootCircle.centerSketchPoint, diskCentre)
        rd = sketch.sketchDimensions.addDiameterDimension(
            rootCircle, self._diam_text_point(signedE, 0, Rv))
        rd.parameter.expression = '2 * ({} - {} - {} - {})'.format(
            self.parameterName(PARAM_PIN_CIRCLE_RADIUS),
            self.parameterName(PARAM_PIN_RADIUS),
            self.parameterName(PARAM_DISK_CLEARANCE),
            self.parameterName(PARAM_ECCENTRICITY))
        self._alongPathLabel(sketch, rootCircle, 'Root Circle')

        # 4. lobe spline (open, adaptively sampled — NO arc).
        pts = _adaptive_lobe_points(
            signedE, 0.0, phi, R, self.Rr_eff, E, N, L)
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in pts:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline = sketch.sketchCurves.sketchFittedSplines.add(coll)

        startPt = spline.fitPoints.item(0)
        endPt = spline.fitPoints.item(spline.fitPoints.count - 1)

        # 5. lock the spline: fix interior fit points, coincide ends onto root.
        for i in range(1, spline.fitPoints.count - 1):
            spline.fitPoints.item(i).isFixed = True
        sketch.geometricConstraints.addCoincident(startPt, rootCircle)
        sketch.geometricConstraints.addCoincident(endPt, rootCircle)

        # 6. spoke line 1 from Od to first point + horizontal.
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(signedE + Rv, 0, 0))
        sketch.geometricConstraints.addCoincident(
            line1.endSketchPoint, startPt)
        sketch.geometricConstraints.addHorizontal(line1)

        # 7. spoke line 2 from Od to last point.
        end2x = signedE + Rv * math.cos(2 * math.pi / L)
        end2y = -Rv * math.sin(2 * math.pi / L)
        line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(end2x, end2y, 0))
        sketch.geometricConstraints.addCoincident(
            line2.endSketchPoint, endPt)

        # 8. driving angular dim between spokes = 360 deg / Lobes.
        textPoint = adsk.core.Point3D.create(
            signedE + 0.4 * Rv * math.cos(math.pi / L),
            -0.4 * Rv * math.sin(math.pi / L), 0)
        angDim = sketch.sketchDimensions.addAngularDimension(
            line1, line2, textPoint)
        angDim.parameter.expression = '360 deg / {}'.format(
            self.parameterName(PARAM_LOBES))

        self.lobeDiskCentres[d] = diskCentre
        self.lobeSplines[d] = spline

    # ----- §2·A: extrude + axis + pattern -----
    def buildDisk(self, d):
        component = self.getComponent()
        sketch = self.lobeSplines[d].parentSketch
        L = self.N - 1

        # Record body count BEFORE the extrude (for the disc-own join).
        base = component.bRepBodies.count

        # 1. select the sector profile by the loop containing this disc's spline.
        sectorProfile = self._profile_containing(sketch, self.lobeSplines[d])

        ext = component.features.extrudeFeatures.createInput(
            sectorProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(ext)
        extrude.bodies.item(0).name = 'Cycloidal Disk {}'.format(d + 1)

        # 2. disk axis (from a planar cap by NORMAL — startFaces.item(0)).
        capFace = extrude.startFaces.item(0)
        self.buildDiskAxis(capFace, d)

        # 3. circular-pattern the EXTRUDE FEATURE x L.
        coll = adsk.core.ObjectCollection.create()
        coll.add(extrude)
        pat = component.features.circularPatternFeatures.createInput(
            coll, self.diskAxes[d])
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(L)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

        # 4. join ONLY this disc's own L sectors.
        target = component.bRepBodies.item(base)
        tools = adsk.core.ObjectCollection.create()
        for i in range(base + 1, base + L):
            tools.add(component.bRepBodies.item(i))
        ci = component.features.combineFeatures.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(ci)
        target.name = 'Cycloidal Disk {}'.format(d + 1)
        self.diskBodies[d] = target

    def buildDiskAxis(self, capFace, d):
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
        axis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis {}'.format(d + 1)
        self.diskAxes[d] = axis

    # ----- §3: output hole sketch -----
    def buildOutputHoleSketch(self, d):
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Output Hole {}'.format(d + 1), plane)
        sketch.isVisible = True

        s_d = 1.0 if d == 0 else -1.0
        signedE = s_d * self.E
        Rop = self.Rop
        D_hole = self.D_hole

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._eccentricCentre(sketch, localOrigin, signedE)
        circles = sketch.sketchCurves.sketchCircles

        # output-hole circle (construction, on Od).
        ohCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), Rop)
        ohCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            ohCircle.centerSketchPoint, diskCentre)
        ohd = sketch.sketchDimensions.addDiameterDimension(
            ohCircle, self._diam_text_point(signedE, 0, Rop))
        ohd.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._alongPathLabel(sketch, ohCircle, 'Output Hole Circle')

        # one solid hole on the Rop circle (D_hole), fully pinned.
        hole = circles.addByCenterRadius(
            adsk.core.Point3D.create(signedE + Rop, 0, 0), D_hole / 2.0)
        hd = sketch.sketchDimensions.addDiameterDimension(
            hole, self._diam_text_point(signedE + Rop, 0, D_hole / 2.0))
        hd.parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        sketch.geometricConstraints.addCoincident(
            hole.centerSketchPoint, ohCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        spoke.isConstruction = True
        sketch.geometricConstraints.addHorizontal(spoke)

        self.outputHoles[d] = hole

    # ----- §4: cut + pattern the output holes -----
    def buildOutputHoles(self, d):
        component = self.getComponent()
        sketch = self.outputHoles[d].parentSketch
        M = self.M

        holeProfile = self._profile_containing(sketch, self.outputHoles[d])
        ci = component.features.extrudeFeatures.createInput(
            holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        cut = component.features.extrudeFeatures.add(ci)

        coll = adsk.core.ObjectCollection.create()
        coll.add(cut)
        pat = component.features.circularPatternFeatures.createInput(
            coll, self.diskAxes[d])
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

    # ----- §6 part 1: disc center bore -----
    def buildDiskBore(self, d):
        component = self.getComponent()
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Disc Bore {}'.format(d + 1), plane)
        sketch.isVisible = True

        s_d = 1.0 if d == 0 else -1.0
        signedE = s_d * self.E
        boreRadius = (self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
                      + self.getParameter(PARAM_BEARING_CLEARANCE).value) / 2.0

        localOrigin = self._anchorLocalOrigin(sketch)
        diskCentre = self._eccentricCentre(sketch, localOrigin, signedE)

        boreCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(signedE, 0, 0), boreRadius)
        sketch.geometricConstraints.addCoincident(
            boreCircle.centerSketchPoint, diskCentre)
        bd = sketch.sketchDimensions.addDiameterDimension(
            boreCircle, self._diam_text_point(signedE, 0, boreRadius))
        bd.parameter.expression = '{} + {}'.format(
            self.parameterName(PARAM_CENTER_BEARING_DIAMETER),
            self.parameterName(PARAM_BEARING_CLEARANCE))

        coll = adsk.core.ObjectCollection.create()
        for p in sketch.profiles:
            coll.add(p)
        ci = component.features.extrudeFeatures.createInput(
            coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        component.features.extrudeFeatures.add(ci)

    # ----- §6 part 2: eccentric cam -----
    def buildCam(self):
        component = self.getComponent()
        D = self.discCount
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        sectionBodies = []
        for d in range(D):
            plane = self.discPlanes[d]
            sketch = self.createSketchObject(
                'Eccentric Cam {}'.format(d + 1), plane)
            sketch.isVisible = True

            s_d = 1.0 if d == 0 else -1.0
            signedE = s_d * self.E
            camRadius = self.getParameter(
                PARAM_CENTER_BEARING_DIAMETER).value / 2.0
            ISD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value

            localOrigin = self._anchorLocalOrigin(sketch)
            diskCentre = self._eccentricCentre(sketch, localOrigin, signedE)
            circles = sketch.sketchCurves.sketchCircles

            camOuter = circles.addByCenterRadius(
                adsk.core.Point3D.create(signedE, 0, 0), camRadius)
            sketch.geometricConstraints.addCoincident(
                camOuter.centerSketchPoint, diskCentre)
            cd = sketch.sketchDimensions.addDiameterDimension(
                camOuter, self._diam_text_point(signedE, 0, camRadius))
            cd.parameter.expression = self.parameterName(
                PARAM_CENTER_BEARING_DIAMETER)

            hasBore = ISD > 0
            if hasBore:
                inputBore = circles.addByCenterRadius(
                    adsk.core.Point3D.create(0, 0, 0), ISD / 2.0)
                sketch.geometricConstraints.addCoincident(
                    inputBore.centerSketchPoint, localOrigin)
                ibd = sketch.sketchDimensions.addDiameterDimension(
                    inputBore, self._diam_text_point(0, 0, ISD / 2.0))
                ibd.parameter.expression = self.parameterName(
                    PARAM_INPUT_SHAFT_DIAMETER)

            # cross-section: 2-loop annulus if bore else 1-loop disc.
            if hasBore:
                camProfile = self._profile_by_loop_count(sketch, 2)
            else:
                camProfile = sketch.profiles.item(0)

            ext = component.features.extrudeFeatures.createInput(
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
            camFeature = component.features.extrudeFeatures.add(ext)
            body = camFeature.bodies.item(0)
            body.name = 'Eccentric Cam {}'.format(d + 1)
            sectionBodies.append(body)

        # Join the D sections into one 'Eccentric Cam'.
        target = sectionBodies[0]
        if len(sectionBodies) > 1:
            tools = adsk.core.ObjectCollection.create()
            for b in sectionBodies[1:]:
                tools.add(b)
            ci = component.features.combineFeatures.createInput(target, tools)
            ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            component.features.combineFeatures.add(ci)
        target.name = 'Eccentric Cam'
        self.cam = target

    # ----- §5: housing base + ring casing -----
    def buildRingPins(self):
        component = self.getComponent()

        # 1. Housing plane (1 mm below the disc).
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane = component.constructionPlanes.add(pi)
        housingPlane.name = 'Ring Housing Plane'

        # 2. Housing base — annulus.
        baseSketch = self.createSketchObject('Housing Ring', housingPlane)
        baseSketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(baseSketch)
        circles = baseSketch.sketchCurves.sketchCircles
        outerR = self.R - self.Rr + 2 * self.E + \
            self.getParameter(PARAM_WALL).value
        innerR = self.R - self.Rr - self.getParameter(PARAM_WALL).value

        outerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        baseSketch.geometricConstraints.addCoincident(
            outerCircle.centerSketchPoint, localOrigin)
        od = baseSketch.sketchDimensions.addDiameterDimension(
            outerCircle, self._diam_text_point(0, 0, outerR))
        od.parameter.expression = self.parameterName(
            PARAM_HOUSING_OUTER_DIAMETER)

        innerCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), innerR)
        baseSketch.geometricConstraints.addCoincident(
            innerCircle.centerSketchPoint, localOrigin)
        idd = baseSketch.sketchDimensions.addDiameterDimension(
            innerCircle, self._diam_text_point(0, 0, innerR))
        idd.parameter.expression = self.parameterName(
            PARAM_HOUSING_INNER_DIAMETER)

        annulusProfile = self._profile_by_loop_count(baseSketch, 2)
        ext = component.features.extrudeFeatures.createInput(
            annulusProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_BASE_THICKNESS))),
            adsk.fusion.ExtentDirections.NegativeExtentDirection)
        baseExtrude = component.features.extrudeFeatures.add(ext)
        self.housingRing = baseExtrude.bodies.item(0)
        self.housingRing.name = 'Housing Ring'

        # 3. Drive axis at O (from the Housing-Ring cap by NORMAL).
        capFace = baseExtrude.startFaces.item(0)
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, localOrigin)
        driveAxis = component.constructionAxes.add(axInput)
        driveAxis.name = 'Drive Axis'
        self.driveAxis = driveAxis

        # 4. Ring casing — one section patterned x N.
        N = self.N
        L = N - 1
        contourPts = _contour_points(
            self.R, self.Rr_eff, self.E, N, L, self.c)

        casingSketch = self.createSketchObject('Ring Casing', self.plane)
        casingSketch.isVisible = True
        casingOrigin = self._anchorLocalOrigin(casingSketch)
        cCircles = casingSketch.sketchCurves.sketchCircles
        casingOuter = cCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        casingSketch.geometricConstraints.addCoincident(
            casingOuter.centerSketchPoint, casingOrigin)
        cod = casingSketch.sketchDimensions.addDiameterDimension(
            casingOuter, self._diam_text_point(0, 0, outerR))
        cod.parameter.expression = self.parameterName(
            PARAM_HOUSING_OUTER_DIAMETER)

        coll = adsk.core.ObjectCollection.create()
        for (x, y) in contourPts:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        contour = casingSketch.sketchCurves.sketchFittedSplines.add(coll)

        # two radial spokes from each spline end out to the outer circle at the
        # same angle.
        first = contourPts[0]
        last = contourPts[-1]
        firstAngle = math.atan2(first[1], first[0])
        lastAngle = math.atan2(last[1], last[0])
        startPt = contour.fitPoints.item(0)
        endPt = contour.fitPoints.item(contour.fitPoints.count - 1)
        casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            startPt,
            adsk.core.Point3D.create(
                outerR * math.cos(firstAngle), outerR * math.sin(firstAngle),
                0))
        casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            endPt,
            adsk.core.Point3D.create(
                outerR * math.cos(lastAngle), outerR * math.sin(lastAngle), 0))

        # select the wedge by MINIMUM AREA among profiles whose loop contains
        # the contour spline.
        sectorProfile = self._min_area_profile_containing(casingSketch, contour)

        stackTopExpr = self._stackTopExpr()
        cext = component.features.extrudeFeatures.createInput(
            sectorProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        cext.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr)),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString('1 mm')))
        casingBase = component.bRepBodies.count
        sectorFeature = component.features.extrudeFeatures.add(cext)

        pcoll = adsk.core.ObjectCollection.create()
        pcoll.add(sectorFeature)
        cpat = component.features.circularPatternFeatures.createInput(
            pcoll, self.driveAxis)
        cpat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        cpat.quantity = adsk.core.ValueInput.createByReal(N)
        cpat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        cpat.isSymmetric = False
        component.features.circularPatternFeatures.add(cpat)

        # Join the N sector bodies into one casing body.
        casingTarget = component.bRepBodies.item(casingBase)
        casingTools = adsk.core.ObjectCollection.create()
        for i in range(casingBase + 1, casingBase + N):
            casingTools.add(component.bRepBodies.item(i))
        cci = component.features.combineFeatures.createInput(
            casingTarget, casingTools)
        cci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(cci)
        casingBody = casingTarget

        # 5. Combine casing into the base -> one 'Housing'.
        joinTools = adsk.core.ObjectCollection.create()
        joinTools.add(casingBody)
        hci = component.features.combineFeatures.createInput(
            self.housingRing, joinTools)
        hci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(hci)
        self.housingRing.name = 'Housing'
        self.ringCasing = None

    # ----- §7: output pins + plate -----
    def buildOutputPins(self):
        component = self.getComponent()
        stackTopExpr = self._stackTopExpr()

        # 1. output plate plane (1 mm above stackTop).
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(
            self.plane,
            adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))
        platePlane = component.constructionPlanes.add(pi)
        platePlane.name = 'Output Plate Plane'

        # 2. sketch.
        sketch = self.createSketchObject('Output Plate', platePlane)
        sketch.isVisible = True
        localOrigin = self._anchorLocalOrigin(sketch)
        circles = sketch.sketchCurves.sketchCircles

        plateR = self.getParameter(PARAM_OUTPUT_PLATE_DIAMETER).value / 2.0
        Rop = self.Rop
        D_pin = self.D_pin

        plateOuter = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), plateR)
        sketch.geometricConstraints.addCoincident(
            plateOuter.centerSketchPoint, localOrigin)
        pod = sketch.sketchDimensions.addDiameterDimension(
            plateOuter, self._diam_text_point(0, 0, plateR))
        pod.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PLATE_DIAMETER)

        outPinCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), Rop)
        outPinCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            outPinCircle.centerSketchPoint, localOrigin)
        opcd = sketch.sketchDimensions.addDiameterDimension(
            outPinCircle, self._diam_text_point(0, 0, Rop))
        opcd.parameter.expression = self.parameterName(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)

        pin = circles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2.0)
        pd = sketch.sketchDimensions.addDiameterDimension(
            pin, self._diam_text_point(Rop, 0, D_pin / 2.0))
        pd.parameter.expression = '{} - 2 * {}'.format(
            self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER),
            self.parameterName(PARAM_ECCENTRICITY))
        sketch.geometricConstraints.addCoincident(
            pin.centerSketchPoint, outPinCircle)
        spoke = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        spoke.isConstruction = True
        sketch.geometricConstraints.addHorizontal(spoke)

        # 3. output plate body — ALL profiles, away from the disk (+normal).
        plateColl = adsk.core.ObjectCollection.create()
        for p in sketch.profiles:
            plateColl.add(p)
        pext = component.features.extrudeFeatures.createInput(
            plateColl, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        plateFeature = component.features.extrudeFeatures.add(pext)
        self.outputPlate = plateFeature.bodies.item(0)
        self.outputPlate.name = 'Output Plate'

        # 4. output pin (two-sided) — capture pinBase BEFORE the pin extrude.
        pinBase = component.bRepBodies.count
        pinProfile = self._single_loop_profile_of(sketch, pin)
        pinExt = component.features.extrudeFeatures.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pinExt.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(
                    self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm')))
        pinFeature = component.features.extrudeFeatures.add(pinExt)
        pinBody = pinFeature.bodies.item(0)
        pinBody.name = 'Output Pin'

        # 5. socket (combine-cut, keep the pin).
        socketTools = adsk.core.ObjectCollection.create()
        socketTools.add(pinBody)
        sci = component.features.combineFeatures.createInput(
            self.outputPlate, socketTools)
        sci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        sci.isKeepToolBodies = True
        combineFeature = component.features.combineFeatures.add(sci)

        # 6. chamfer the pin ends.
        chamferFeature = self._chamferCapRims(pinBody)

        # 7. pattern pin, socket AND chamfer x M about driveAxis.
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature:
            coll.add(chamferFeature)
        M = self.M
        pat = component.features.circularPatternFeatures.createInput(
            coll, self.driveAxis)
        pat.patternComputeOption = \
            adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

        # 8. name all M pin bodies (contiguous block from pinBase).
        for k in range(M):
            component.bRepBodies.item(pinBase + k).name = \
                'Output Pin {}'.format(k + 1)

    # ----- §8: edge chamfers -----
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
        """Chamfer the outer rim of the two axially-EXTREME caps of `body`
        ([CYCLOIDAL-F-CHAMFERS]). Returns the ChamferFeature or None. Wraps the
        chamfers.add(...) in try/except: on failure logs, increments
        self.chamfersSkipped, returns None — never re-raises."""
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
            h = ((o.x - ref.x) * axis.x + (o.y - ref.y) * axis.y
                 + (o.z - ref.z) * axis.z)
            capFaces.append((h, face))
        if len(capFaces) == 0:
            return None
        hmin = min(h for (h, _) in capFaces)
        hmax = max(h for (h, _) in capFaces)

        edges = adsk.core.ObjectCollection.create()
        tol = 1e-4
        for (h, face) in capFaces:
            if abs(h - hmin) < tol or abs(h - hmax) < tol:
                for loop in face.loops:
                    if loop.isOuter:
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
        try:
            return chamfers.add(ci)
        except RuntimeError as e:
            futil.log('chamfer skipped: {}'.format(str(e)))
            self.chamfersSkipped += 1
            return None

    # ----- §9: organize bodies into sub-components -----
    def buildSubComponents(self):
        component = self.getComponent()

        # Snapshot first, move second.
        rotorDiscs = []
        housing = []
        cam = []
        output = []
        for body in component.bRepBodies:
            name = body.name
            if name.startswith('Cycloidal Disk'):
                rotorDiscs.append(body)
            elif name == 'Housing':
                housing.append(body)
            elif name == 'Eccentric Cam':
                cam.append(body)
            elif name == 'Output Plate' or name.startswith('Output Pin'):
                output.append(body)

        for (groupName, bodies) in (
                ('Rotor Discs', rotorDiscs),
                ('Housing', housing),
                ('Eccentric Cam', cam),
                ('Output', output)):
            if len(bodies) == 0:
                continue
            occ = component.occurrences.addNewComponent(
                adsk.core.Matrix3D.create())
            occ.component.name = groupName
            for body in bodies:
                body.moveToComponent(occ)

        # Final cleanup — hide all construction geometry.
        solids.hide_construction_geometry(component)

    # ----- profile-selection helpers -----
    def _profile_containing(self, sketch, curve):
        """The profile whose loop contains `curve` (identity match)."""
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == curve:
                        return profile
        raise Exception(
            'No profile in sketch "{}" contains the given curve'.format(
                sketch.name))

    def _min_area_profile_containing(self, sketch, curve):
        """Among profiles whose loop contains `curve`, the one with the
        smallest area (the casing wedge, not the large complement)."""
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
                'No profile in sketch "{}" contains the given curve'.format(
                    sketch.name))
        return best

    def _single_loop_profile_of(self, sketch, curve):
        """The profileLoops.count == 1 profile whose loop curve is `curve`."""
        for profile in sketch.profiles:
            if profile.profileLoops.count != 1:
                continue
            for loop in profile.profileLoops:
                for pc in loop.profileCurves:
                    if pc.sketchEntity == curve:
                        return profile
        raise Exception(
            'No single-loop profile in sketch "{}" for the given curve'.format(
                sketch.name))

    def _profile_by_loop_count(self, sketch, loopCount):
        """The profile with exactly `loopCount` loops (annulus = 2)."""
        for profile in sketch.profiles:
            if profile.profileLoops.count == loopCount:
                return profile
        raise Exception(
            'No profile with {} loops in sketch "{}"'.format(
                loopCount, sketch.name))
