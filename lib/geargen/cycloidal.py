import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator, get_value, get_selection
from . import solids


# ---------------------------------------------------------------------------
# Dialog input ids and registered parameter names (S01, S03).
# ---------------------------------------------------------------------------

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
INPUT_ID_PARENT = 'parentComponent'

STATUS_SUFFIX = '__status'

PARAM_PIN_COUNT = 'PinCount'
PARAM_PIN_CIRCLE_DIAMETER = 'PinCircleDiameter'
PARAM_PIN_DIAMETER = 'PinDiameter'
PARAM_ECCENTRICITY = 'Eccentricity'
PARAM_DISK_CLEARANCE = 'DiskClearance'
PARAM_DISC_THICKNESS = 'DiscThickness'
PARAM_DISC_GAP = 'DiscGap'
PARAM_CENTER_BEARING_DIAMETER = 'CenterBearingDiameter'
PARAM_INPUT_SHAFT_DIAMETER = 'InputShaftDiameter'
PARAM_BEARING_CLEARANCE = 'BearingClearance'
PARAM_OUTPUT_PIN_CIRCLE_DIAMETER = 'OutputPinCircleDiameter'
PARAM_OUTPUT_PIN_COUNT = 'OutputPinCount'
PARAM_OUTPUT_PIN_DIAMETER = 'OutputPinDiameter'
PARAM_WALL = 'Wall'
PARAM_BASE_THICKNESS = 'BaseThickness'
PARAM_OUTPUT_PLATE_THICKNESS = 'OutputPlateThickness'
PARAM_CHAMFER_SIZE = 'ChamferSize'

PARAM_PIN_RADIUS = 'PinRadius'
PARAM_OUTPUT_HOLE_DIAMETER = 'OutputHoleDiameter'
PARAM_LOBES = 'Lobes'
PARAM_PIN_CIRCLE_RADIUS = 'PinCircleRadius'
PARAM_OUTPUT_PIN_CIRCLE_RADIUS = 'OutputPinCircleRadius'
PARAM_HOUSING_INNER_DIAMETER = 'HousingInnerDiameter'
PARAM_HOUSING_OUTER_DIAMETER = 'HousingOuterDiameter'
PARAM_OUTPUT_PLATE_DIAMETER = 'OutputPlateDiameter'

FINE_STEPS = 2000
TURN_LIMIT = 5.0 * math.pi / 180.0
SWEEP_STEPS = 240
CONTOUR_BINS = 80
BISECT_ROUNDS = 40


# ---------------------------------------------------------------------------
# S01: dialog
# ---------------------------------------------------------------------------

class CycloidalDriveCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs: adsk.core.CommandInputs = command.commandInputs

        plane: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the drive on')
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        plane.setSelectionLimits(1, 1)

        anchor: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point the drive is centered on')
        anchor.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchor.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchor.setSelectionLimits(1, 1)

        discCount: adsk.core.DropDownCommandInput = inputs.addDropDownCommandInput(
            INPUT_ID_DISC_COUNT, 'Disc Count', adsk.core.DropDownStyles.TextListDropDownStyle)
        discCount.listItems.add('1', True)
        discCount.listItems.add('2', False)
        cls._slot(inputs, INPUT_ID_DISC_COUNT)

        cls._value(inputs, INPUT_ID_PIN_COUNT, 'Pin Count', '', 16)
        cls._value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'Pin Circle Diameter', 'mm', to_cm(90))
        cls._value(inputs, INPUT_ID_PIN_DIAMETER, 'Pin Diameter', 'mm', to_cm(0))
        cls._value(inputs, INPUT_ID_ECCENTRICITY, 'Eccentricity', 'mm', to_cm(1.5))
        cls._value(inputs, INPUT_ID_DISK_CLEARANCE, 'Disk Clearance', 'mm', to_cm(0.3))
        cls._value(inputs, INPUT_ID_DISC_THICKNESS, 'Disc Thickness', 'mm', to_cm(8))
        cls._value(inputs, INPUT_ID_DISC_GAP, 'Disc Gap', 'mm', to_cm(0.5))
        cls._value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'Center Bearing Diameter', 'mm', to_cm(30))
        cls._value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'Input Shaft Diameter', 'mm', to_cm(8))
        cls._value(inputs, INPUT_ID_BEARING_CLEARANCE, 'Bearing Clearance', 'mm', to_cm(0.2))
        cls._value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'Output Pin Circle Diameter', 'mm', to_cm(50))
        cls._value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', '', 6)
        cls._value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 'mm', to_cm(0))
        cls._value(inputs, INPUT_ID_WALL, 'Housing Wall', 'mm', to_cm(3))
        cls._value(inputs, INPUT_ID_BASE_THICKNESS, 'Base Thickness', 'mm', to_cm(5))
        cls._value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'Output Plate Thickness', 'mm', to_cm(5))
        cls._value(inputs, INPUT_ID_CHAMFER_SIZE, 'Chamfer Size', 'mm', to_cm(0.5))

        parent: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the component the drive is created in')
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parent.setSelectionLimits(0, 1)
        parent.addSelection(get_design().rootComponent)

    @classmethod
    def _value(cls, inputs: adsk.core.CommandInputs, input_id: str, label: str, unit: str, default: float):
        inputs.addValueInput(input_id, label, unit, adsk.core.ValueInput.createByReal(default))
        cls._slot(inputs, input_id)

    @classmethod
    def _slot(cls, inputs: adsk.core.CommandInputs, input_id: str):
        slot: adsk.core.TextBoxCommandInput = inputs.addTextBoxCommandInput(
            input_id + STATUS_SUFFIX, '', '', 2, True)
        slot.isVisible = False


# ---------------------------------------------------------------------------
# S02: one resolve + validate routine, shared by the dialog and the generator
# ---------------------------------------------------------------------------

def resolve_values(raw: dict) -> dict:
    """Resolve the derived dimensions from raw internal-cm dialog values.

    raw keys: N, M, D, R, Rop, E, c, pinDiameter, outputPinDiameter, CBD, clr, ISD.
    """
    N = raw['N']
    M = raw['M']
    R = raw['R']
    Rop = raw['Rop']
    E = raw['E']
    c = raw['c']
    vals = dict(raw)
    vals['auto_pin'] = not (raw['pinDiameter'] > 0)
    vals['auto_out'] = not (raw['outputPinDiameter'] > 0)
    if raw['pinDiameter'] > 0:
        Rr = raw['pinDiameter'] / 2
    else:
        Rr = 0.5 * (E + R * math.sin(math.pi / N))
    Rr_eff = Rr + c
    Rv = R - Rr_eff - E
    if raw['outputPinDiameter'] > 0:
        D_pin = raw['outputPinDiameter']
    else:
        D_pin = Rop * math.sin(math.pi / M) - E
    D_hole = D_pin + 2 * E
    vals['Rr'] = Rr
    vals['Rr_eff'] = Rr_eff
    vals['Rv'] = Rv
    vals['D_pin'] = D_pin
    vals['D_hole'] = D_hole
    vals['L'] = N - 1
    return vals


def rho_min_toward_origin(R: float, E: float, N: int) -> float:
    best = math.inf
    for i in range(FINE_STEPS):
        t = 2 * math.pi * i / FINE_STEPS
        bx = R * math.cos(t) - E * math.cos(N * t)
        by = -R * math.sin(t) + E * math.sin(N * t)
        xp = -R * math.sin(t) + E * N * math.sin(N * t)
        yp = -R * math.cos(t) + E * N * math.cos(N * t)
        xpp = -R * math.cos(t) + E * N * N * math.cos(N * t)
        ypp = R * math.sin(t) - E * N * N * math.sin(N * t)
        k = xp * ypp - yp * xpp
        if abs(k) < 1e-12:
            continue
        s = math.sqrt(xp * xp + yp * yp)
        rho = s ** 3 / k
        nx = -yp / s
        ny = xp / s
        cx = bx + rho * nx
        cy = by + rho * ny
        if cx * cx + cy * cy < bx * bx + by * by:
            best = min(best, abs(rho))
    return best


def undercut_limit(vals: dict) -> float:
    """E*: 40 bisection rounds on E' in (0, E], re-resolving Rr and rho_min_O each round.
    Returns 0 when no positive E' satisfies the guard."""
    R = vals['R']
    N = vals['N']
    c = vals['c']
    lo = 0.0
    hi = vals['E']
    for _ in range(BISECT_ROUNDS):
        mid = (lo + hi) / 2
        if not vals['auto_pin']:
            rr = vals['pinDiameter'] / 2
        else:
            rr = 0.5 * (mid + R * math.sin(math.pi / N))
        if rr + c < rho_min_toward_origin(R, mid, N):
            lo = mid
        else:
            hi = mid
    return lo


def _mm(cm: float) -> str:
    return '{:.2f}'.format(to_mm(cm))


def evaluate_problems(vals: dict) -> list:
    problems = []
    N = vals['N']
    M = vals['M']
    D = vals['D']
    R = vals['R']
    Rop = vals['Rop']
    E = vals['E']
    Rr = vals['Rr']
    Rr_eff = vals['Rr_eff']
    Rv = vals['Rv']
    D_pin = vals['D_pin']
    D_hole = vals['D_hole']
    CBD = vals['CBD']
    clr = vals['clr']
    ISD = vals['ISD']
    sinN = math.sin(math.pi / N)
    sinM = math.sin(math.pi / M)

    if D == 2 and (N % 2 != 0 or M % 2 != 0):
        problems.append(
            'Two discs require an even Pin Count and an even Output Pin Count '
            '(currently N={}, M={}).'.format(N, M))

    if not (E < Rr < R * sinN):
        if vals['auto_pin']:
            problems.append(
                'Pin geometry out of range — increase Pin Circle Diameter above {} mm '
                'or reduce Eccentricity below {} mm.'.format(_mm(2 * E / sinN), _mm(R * sinN)))
        else:
            problems.append(
                'Pin Diameter must be between {} mm and {} mm (currently {}).'.format(
                    _mm(2 * E), _mm(2 * R * sinN), _mm(2 * Rr)))

    if not (D_pin > 0):
        if vals['auto_out']:
            problems.append(
                'Output pins vanish (resolved diameter ≤ 0) — increase Output Pin '
                'Circle Diameter above {} mm, increase Output Pin Count, or reduce '
                'Eccentricity.'.format(_mm(2 * E / sinM)))
        else:
            problems.append('Output Pin Diameter must be greater than 0.')

    if not (D_hole < 2 * Rop * sinM):
        problems.append(
            'Output holes overlap — increase Output Pin Circle Diameter above {} mm, '
            'increase Output Pin Count, or reduce Output Pin Diameter / Eccentricity.'.format(
                _mm((D_pin + 2 * E) / sinM)))

    if not (E < R / N):
        problems.append(
            'Eccentricity too large — reduce it below {} mm (or increase Pin Circle '
            'Diameter / reduce Pin Count).'.format(_mm(R / N)))

    if not (Rop < Rv):
        problems.append(
            'Output Pin Circle too large — set Output Pin Circle Diameter below {} mm '
            '(currently {}).'.format(_mm(2 * Rv), _mm(2 * Rop)))

    if not (Rr_eff < rho_min_toward_origin(R, E, N)):
        limit = undercut_limit(vals)
        if limit > 0:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/self-intersects. '
                'Reduce Eccentricity below {} mm.'.format(_mm(limit)))
        else:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/self-intersects. '
                'Reduce Eccentricity.')

    if not (ISD < CBD):
        problems.append(
            'Input Shaft Diameter must be less than Center Bearing Diameter ({} mm).'.format(_mm(CBD)))

    if not (E + ISD / 2 < CBD / 2):
        problems.append(
            "Input bore doesn't fit inside the cam — set Input Shaft Diameter below {} mm, "
            'or reduce Eccentricity / increase Center Bearing Diameter.'.format(_mm(CBD - 2 * E)))

    if not ((CBD + clr) / 2 < Rop - D_hole / 2):
        problems.append(
            'Disk center bore overlaps the output holes — increase Output Pin Circle '
            'Diameter above {} mm, or reduce Center Bearing Diameter / Bearing Clearance / '
            'output pin size.'.format(_mm(CBD + clr + D_hole)))

    return problems


def read_raw_inputs(inputs: adsk.core.CommandInputs) -> dict:
    """Raw internal-cm values straight off the dialog inputs (no parameters, no document writes)."""
    def val(inputs: adsk.core.CommandInputs, input_id: str) -> float:
        vi: adsk.core.ValueCommandInput = adsk.core.ValueCommandInput.cast(inputs.itemById(input_id))
        return vi.value

    dropdown: adsk.core.DropDownCommandInput = adsk.core.DropDownCommandInput.cast(
        inputs.itemById(INPUT_ID_DISC_COUNT))
    return {
        'N': int(round(val(inputs, INPUT_ID_PIN_COUNT))),
        'M': int(round(val(inputs, INPUT_ID_OUTPUT_PIN_COUNT))),
        'D': int(dropdown.selectedItem.name),
        'R': val(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER) / 2,
        'Rop': val(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER) / 2,
        'E': val(inputs, INPUT_ID_ECCENTRICITY),
        'c': val(inputs, INPUT_ID_DISK_CLEARANCE),
        'pinDiameter': val(inputs, INPUT_ID_PIN_DIAMETER),
        'outputPinDiameter': val(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER),
        'CBD': val(inputs, INPUT_ID_CENTER_BEARING_DIAMETER),
        'clr': val(inputs, INPUT_ID_BEARING_CLEARANCE),
        'ISD': val(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER),
        'T': val(inputs, INPUT_ID_DISC_THICKNESS),
        'G': val(inputs, INPUT_ID_DISC_GAP),
        'Wall': val(inputs, INPUT_ID_WALL),
        'BaseT': val(inputs, INPUT_ID_BASE_THICKNESS),
        'PlateT': val(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS),
        'Chamfer': val(inputs, INPUT_ID_CHAMFER_SIZE),
    }


# ---------------------------------------------------------------------------
# Geometry transcriptions from the proof (S06, S23)
# ---------------------------------------------------------------------------

def disk_point(vals: dict, t: float, cx: float, cy: float, phi: float):
    """The rotor boundary point at parameter t, in internal cm."""
    n = vals['N']
    R = vals['R']
    E = vals['E']
    rr_eff = vals['Rr_eff']
    num = math.sin((1 - n) * t)
    den = R / (E * n) - math.cos((1 - n) * t)
    psi = math.atan2(num, den)
    x0 = R * math.cos(t) - rr_eff * math.cos(t + psi) - E * math.cos(n * t)
    y0 = -R * math.sin(t) + rr_eff * math.sin(t + psi) + E * math.sin(n * t)
    x = cx + x0 * math.cos(phi) - y0 * math.sin(phi)
    y = cy + x0 * math.sin(phi) + y0 * math.cos(phi)
    return (x, y)


def lobe_samples(vals: dict, cx: float, cy: float, phi: float) -> list:
    """One lobe, t in [0, 2*pi/L], sampled by bounded turn angle."""
    span = 2 * math.pi / vals['L']
    fine = [disk_point(vals, span * i / FINE_STEPS, cx, cy, phi) for i in range(FINE_STEPS + 1)]
    kept = [fine[0]]
    acc = 0.0
    for i in range(1, FINE_STEPS):
        a = math.atan2(fine[i][1] - fine[i - 1][1], fine[i][0] - fine[i - 1][0])
        b = math.atan2(fine[i + 1][1] - fine[i][1], fine[i + 1][0] - fine[i][0])
        acc += abs(math.atan2(math.sin(b - a), math.cos(b - a)))
        if acc >= TURN_LIMIT:
            kept.append(fine[i])
            acc = 0.0
    kept.append(fine[FINE_STEPS])
    return kept


def contour_pitch(vals: dict) -> list:
    """The casing's inner wall over one pin pitch, emitted at bin edges."""
    N = vals['N']
    L = vals['L']
    E = vals['E']
    half = math.pi / N
    bin_max = [0.0] * CONTOUR_BINS
    hit = [False] * CONTOUR_BINS
    for i in range(SWEEP_STEPS):
        th = 2 * math.pi * i / SWEEP_STEPS
        cx = E * math.cos(th)
        cy = E * math.sin(th)
        clock = -th / L
        for j in range(SWEEP_STEPS):
            q = disk_point(vals, 2 * math.pi * j / SWEEP_STEPS, cx, cy, clock)
            a = math.atan2(q[1], q[0])
            if a < -half or a > half:
                continue
            b = int((a + half) / (2 * half) * CONTOUR_BINS)
            b = min(max(b, 0), CONTOUR_BINS - 1)
            r = math.hypot(q[0], q[1])
            if (not hit[b]) or r > bin_max[b]:
                bin_max[b] = r
                hit[b] = True
    out = []
    for i in range(CONTOUR_BINS + 1):
        ang = -half + 2 * half * i / CONTOUR_BINS
        peak = 0.0
        if i > 0 and hit[i - 1]:
            peak = max(peak, bin_max[i - 1])
        if i < CONTOUR_BINS and hit[i]:
            peak = max(peak, bin_max[i])
        r = vals['c'] + peak
        out.append((r * math.cos(ang), r * math.sin(ang)))
    return out


# ---------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------

class CycloidalDriveGenerator(Generator):
    DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + STATUS_SUFFIX

    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.vals = {}
        self.Rr = 0.0
        self.D_hole = 0.0
        self.chamferSize = 0.0
        self.chamfersSkipped = 0
        self.diskBodies = [adsk.fusion.BRepBody.cast(None)] * 2
        self.diskAxes = [adsk.fusion.ConstructionAxis.cast(None)] * 2
        self.lobeSplines = [adsk.fusion.SketchFittedSpline.cast(None)] * 2
        self.outputHoles = [adsk.fusion.SketchCircle.cast(None)] * 2
        self.lobeDiskCentres = [adsk.fusion.SketchPoint.cast(None)] * 2
        self.discPlanes = [adsk.fusion.ConstructionPlane.cast(None)] * 2
        self.lobeSketches = [adsk.fusion.Sketch.cast(None)] * 2
        self.holeSketches = [adsk.fusion.Sketch.cast(None)] * 2
        self.driveAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.housingRing = adsk.fusion.BRepBody.cast(None)
        self.ringCasing = adsk.fusion.BRepBody.cast(None)
        self.cam = adsk.fusion.BRepBody.cast(None)
        self.outputPlate = adsk.fusion.BRepBody.cast(None)
        self.lobePinCircle = adsk.fusion.SketchCircle.cast(None)

    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    # -- S02 ---------------------------------------------------------------

    @staticmethod
    def validate_inputs(inputs: adsk.core.CommandInputs) -> list:
        raw = read_raw_inputs(inputs)
        return evaluate_problems(resolve_values(raw))

    # -- S03 ---------------------------------------------------------------

    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Selections first: creating the occurrence drops them ([PB-SELECTION-STASH]).
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) == 0:
            self.parentComponent = get_design().rootComponent
        else:
            occ: adsk.fusion.Occurrence = adsk.fusion.Occurrence.cast(parents[0])
            if occ:
                self.parentComponent = occ.component
            else:
                self.parentComponent = adsk.fusion.Component.cast(parents[0])
        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) == 0:
            raise Exception('A Target Plane must be selected.')
        self.plane = planes[0]
        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) == 0:
            raise Exception('An Anchor Point must be selected.')
        self.anchorPoint = anchors[0]

        dropdown: adsk.core.DropDownCommandInput = adsk.core.DropDownCommandInput.cast(
            inputs.itemById(INPUT_ID_DISC_COUNT))
        D = int(dropdown.selectedItem.name)

        self.chamfersSkipped = 0

        self.addParameter(PARAM_PIN_COUNT, get_value(inputs, INPUT_ID_PIN_COUNT, ''), '', 'Pin Count')
        self.addParameter(PARAM_PIN_CIRCLE_DIAMETER,
                          get_value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'mm'), 'mm', 'Pin Circle Diameter')
        self.addParameter(PARAM_PIN_DIAMETER, get_value(inputs, INPUT_ID_PIN_DIAMETER, 'mm'), 'mm',
                          'Pin Diameter (0 = automatic)')
        self.addParameter(PARAM_ECCENTRICITY, get_value(inputs, INPUT_ID_ECCENTRICITY, 'mm'), 'mm',
                          'Eccentricity')
        self.addParameter(PARAM_DISK_CLEARANCE, get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'), 'mm',
                          'Disk Clearance')
        self.addParameter(PARAM_DISC_THICKNESS, get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'), 'mm',
                          'Disc Thickness')
        self.addParameter(PARAM_DISC_GAP, get_value(inputs, INPUT_ID_DISC_GAP, 'mm'), 'mm', 'Disc Gap')
        self.addParameter(PARAM_CENTER_BEARING_DIAMETER,
                          get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'), 'mm',
                          'Center Bearing Diameter')
        self.addParameter(PARAM_INPUT_SHAFT_DIAMETER,
                          get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'), 'mm',
                          'Input Shaft Diameter')
        self.addParameter(PARAM_BEARING_CLEARANCE,
                          get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'), 'mm', 'Bearing Clearance')
        self.addParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER,
                          get_value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'mm'), 'mm',
                          'Output Pin Circle Diameter')
        self.addParameter(PARAM_OUTPUT_PIN_COUNT, get_value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, ''), '',
                          'Output Pin Count')
        self.addParameter(PARAM_OUTPUT_PIN_DIAMETER,
                          get_value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'mm'), 'mm',
                          'Output Pin Diameter (0 = automatic)')
        self.addParameter(PARAM_WALL, get_value(inputs, INPUT_ID_WALL, 'mm'), 'mm', 'Housing Wall')
        self.addParameter(PARAM_BASE_THICKNESS, get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'), 'mm',
                          'Base Thickness')
        self.addParameter(PARAM_OUTPUT_PLATE_THICKNESS,
                          get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'mm'), 'mm',
                          'Output Plate Thickness')
        self.addParameter(PARAM_CHAMFER_SIZE, get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'), 'mm',
                          'Chamfer Size')

        self._resolveDimensions(inputs)
        self.vals['D'] = D

        pn = self.parameterName
        self.addParameter(PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(self.Rr), 'mm',
                          'Resolved pin radius')
        self.addParameter(PARAM_OUTPUT_HOLE_DIAMETER, adsk.core.ValueInput.createByReal(self.D_hole), 'mm',
                          'Resolved output hole diameter')
        self.addParameter(PARAM_LOBES,
                          adsk.core.ValueInput.createByString('{} - 1'.format(pn(PARAM_PIN_COUNT))), '',
                          'Lobe count')
        self.addParameter(PARAM_PIN_CIRCLE_RADIUS,
                          adsk.core.ValueInput.createByString('{} / 2'.format(pn(PARAM_PIN_CIRCLE_DIAMETER))),
                          'mm', 'Pin circle radius')
        self.addParameter(PARAM_OUTPUT_PIN_CIRCLE_RADIUS,
                          adsk.core.ValueInput.createByString(
                              '{} / 2'.format(pn(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER))),
                          'mm', 'Output pin circle radius')
        self.addParameter(PARAM_HOUSING_INNER_DIAMETER,
                          adsk.core.ValueInput.createByString('2 * ({} - {} - {})'.format(
                              pn(PARAM_PIN_CIRCLE_RADIUS), pn(PARAM_PIN_RADIUS), pn(PARAM_WALL))),
                          'mm', 'Housing inner diameter')
        self.addParameter(PARAM_HOUSING_OUTER_DIAMETER,
                          adsk.core.ValueInput.createByString('2 * ({} - {} + 2 * {} + {})'.format(
                              pn(PARAM_PIN_CIRCLE_RADIUS), pn(PARAM_PIN_RADIUS),
                              pn(PARAM_ECCENTRICITY), pn(PARAM_WALL))),
                          'mm', 'Housing outer diameter')
        self.addParameter(PARAM_OUTPUT_PLATE_DIAMETER,
                          adsk.core.ValueInput.createByString('{} + ({} - 2 * {}) + 2 * {}'.format(
                              pn(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER), pn(PARAM_OUTPUT_HOLE_DIAMETER),
                              pn(PARAM_ECCENTRICITY), pn(PARAM_WALL))),
                          'mm', 'Output plate diameter')

    def _resolveDimensions(self, inputs: adsk.core.CommandInputs):
        raw = read_raw_inputs(inputs)
        vals = resolve_values(raw)
        problems = evaluate_problems(vals)
        if problems:
            raise Exception('\n'.join(problems))
        self.vals = vals
        self.Rr = vals['Rr']
        self.D_hole = vals['D_hole']
        self.chamferSize = vals['Chamfer']

    def generateName(self) -> str:
        N = self.vals['N']
        return 'Cycloidal Drive (N={}):{}'.format(N, N - 1)

    # -- S04 ---------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        component = self.getComponent()
        component.name = self.generateName()

        if not adsk.fusion.ConstructionPlane.cast(self.plane):
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('0 mm'))
            self.plane = component.constructionPlanes.add(planeInput)

        D = self.vals['D']
        for d in range(D):
            self._buildDiscPlane(d)
            futil.log('cycloidal: disc {}'.format(d + 1))
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
                'Cycloidal drive generated, but {n} chamfer(s) could not be created at Chamfer Size {sz} '
                'mm and were skipped. Reduce Chamfer Size (or set it to 0) for this geometry.'.format(
                    n=self.chamfersSkipped, sz=to_mm(self.chamferSize)))

    # -- shared helpers ----------------------------------------------------

    def _stackTopExpr(self) -> str:
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        if self.vals['D'] == 1:
            return nT
        return '2 * {} + {}'.format(nT, nG)

    def _planeFor(self, d: int) -> adsk.fusion.ConstructionPlane:
        if d == 0:
            return self.plane
        return self.discPlanes[d]

    def _sign(self, d: int) -> float:
        return 1.0 if d == 0 else -1.0

    def _newSketch(self, name: str, plane: adsk.fusion.ConstructionPlane) -> adsk.fusion.Sketch:
        sketch: adsk.fusion.Sketch = self.createSketchObject(name, plane)
        sketch.isVisible = True
        return sketch

    def _anchorSketch(self, sketch: adsk.fusion.Sketch) -> adsk.fusion.SketchPoint:
        """[CYCLOIDAL-F-ANCHOR-CHAIN] Project the Anchor and add a fresh local origin on it."""
        projected: adsk.fusion.SketchPoint = adsk.fusion.SketchPoint.cast(
            sketch.project(self.anchorPoint).item(0))
        localOrigin: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        sketch.geometricConstraints.addCoincident(localOrigin, projected)
        return localOrigin

    def _discCentre(self, sketch: adsk.fusion.Sketch, localOrigin: adsk.fusion.SketchPoint,
                    d: int) -> adsk.fusion.SketchPoint:
        """[CYCLOIDAL-F-DISK-CENTER] Od_d off the local origin, driven by Eccentricity."""
        E = self.vals['E']
        Rv = self.vals['Rv']
        s = self._sign(d)
        diskCentre: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(s * E, 0, 0))
        eccLine: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        dim: adsk.fusion.SketchLinearDimension = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(s * E / 2, -Rv / 10, 0))
        dim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return diskCentre

    def _circleOn(self, sketch: adsk.fusion.Sketch, cx: float, cy: float, r: float,
                  at: adsk.fusion.SketchPoint, expression: str,
                  construction: bool) -> adsk.fusion.SketchCircle:
        circle: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(cx, cy, 0), r)
        circle.isConstruction = construction
        sketch.geometricConstraints.addCoincident(circle.centerSketchPoint, at)
        dim: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(cx + 0.75 * r, cy + 0.75 * r, 0))
        dim.parameter.expression = expression
        return circle

    def _label(self, sketch: adsk.fusion.Sketch, text: str, curve: adsk.fusion.SketchCurve):
        textInput: adsk.fusion.SketchTextInput = sketch.sketchTexts.createInput2(text, self.Rr)
        alignment: adsk.core.HorizontalAlignments = adsk.core.HorizontalAlignments.CenterHorizontalAlignment
        textInput.setAsAlongPath(curve, True, alignment, 0)
        sketch.sketchTexts.add(textInput)

    def _profileHolding(self, sketch: adsk.fusion.Sketch,
                        target: adsk.fusion.SketchEntity) -> adsk.fusion.Profile:
        """The first profile one of whose loops carries `target`, by identity."""
        for i in range(sketch.profiles.count):
            profile: adsk.fusion.Profile = sketch.profiles.item(i)
            for j in range(profile.profileLoops.count):
                loop: adsk.fusion.ProfileLoop = profile.profileLoops.item(j)
                for k in range(loop.profileCurves.count):
                    pc: adsk.fusion.ProfileCurve = loop.profileCurves.item(k)
                    if pc.sketchEntity == target:
                        return profile
        raise Exception('No profile in sketch "{}" carries the expected curve.'.format(sketch.name))

    def _allProfiles(self, sketch: adsk.fusion.Sketch) -> adsk.core.ObjectCollection:
        coll = adsk.core.ObjectCollection.create()
        for i in range(sketch.profiles.count):
            coll.add(sketch.profiles.item(i))
        return coll

    def _extentOf(self, expression: str) -> adsk.fusion.DistanceExtentDefinition:
        return adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(expression))

    def _pattern(self, feature, axis: adsk.fusion.ConstructionAxis, quantity: int,
                 component: adsk.fusion.Component):
        coll = adsk.core.ObjectCollection.create()
        coll.add(feature)
        self._patternCollection(coll, axis, quantity, component)

    def _patternCollection(self, coll: adsk.core.ObjectCollection, axis: adsk.fusion.ConstructionAxis,
                           quantity: int, component: adsk.fusion.Component):
        pat = component.features.circularPatternFeatures.createInput(coll, axis)
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(quantity)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

    def _join(self, target: adsk.fusion.BRepBody, tools: adsk.core.ObjectCollection,
              component: adsk.fusion.Component):
        ci = component.features.combineFeatures.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(ci)

    def _joinRange(self, component: adsk.fusion.Component, base: int, count: int) -> adsk.fusion.BRepBody:
        """Join bodies base .. base+count-1 into the one at `base`; return it."""
        target: adsk.fusion.BRepBody = component.bRepBodies.item(base)
        if count > 1:
            tools = adsk.core.ObjectCollection.create()
            for i in range(base + 1, base + count):
                tools.add(component.bRepBodies.item(i))
            self._join(target, tools, component)
        return component.bRepBodies.item(base)

    def buildDiskAxis(self, capFace: adsk.fusion.BRepFace, d: int):
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
        axis: adsk.fusion.ConstructionAxis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis {}'.format(d + 1)
        self.diskAxes[d] = axis

    def _capAxis(self, extrude: adsk.fusion.ExtrudeFeature, point, name: str) -> adsk.fusion.ConstructionAxis:
        component = self.getComponent()
        capFace: adsk.fusion.BRepFace = extrude.startFaces.item(0)
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, point)
        axis: adsk.fusion.ConstructionAxis = component.constructionAxes.add(axInput)
        axis.name = name
        return axis

    # -- S05 ---------------------------------------------------------------

    def _buildDiscPlane(self, d: int):
        if d == 0:
            return
        component = self.getComponent()
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('{} * ({} + {})'.format(
            d, self.parameterName(PARAM_DISC_THICKNESS), self.parameterName(PARAM_DISC_GAP))))
        plane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        plane.name = 'Disc Plane {}'.format(d + 1)
        self.discPlanes[d] = plane

    # -- S06 ---------------------------------------------------------------

    def buildLobeSketch(self, d: int):
        v = self.vals
        E = v['E']
        R = v['R']
        Rop = v['Rop']
        Rv = v['Rv']
        L = v['L']
        s = self._sign(d)
        pn = self.parameterName

        sketch = self._newSketch('Rotor Lobe {}'.format(d + 1), self._planeFor(d))
        self.lobeSketches[d] = sketch
        localOrigin = self._anchorSketch(sketch)
        diskCentre = self._discCentre(sketch, localOrigin, d)
        self.lobeDiskCentres[d] = diskCentre

        pinCircle = self._circleOn(sketch, 0, 0, R, localOrigin, pn(PARAM_PIN_CIRCLE_DIAMETER), True)
        self._label(sketch, 'Pin Circle', pinCircle)
        if d == 0:
            self.lobePinCircle = pinCircle

        outCircle = self._circleOn(sketch, s * E, 0, Rop, diskCentre,
                                   pn(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER), True)
        self._label(sketch, 'Output Pin Circle', outCircle)

        rootCircle = self._circleOn(
            sketch, s * E, 0, Rv, diskCentre,
            '2 * ({} - {} - {} - {})'.format(pn(PARAM_PIN_CIRCLE_RADIUS), pn(PARAM_PIN_RADIUS),
                                             pn(PARAM_DISK_CLEARANCE), pn(PARAM_ECCENTRICITY)),
            True)
        self._label(sketch, 'Root Circle', rootCircle)

        # The lobe: one open fitted spline through the turn-bounded samples.
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in lobe_samples(v, s * E, 0.0, d * math.pi):
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline: adsk.fusion.SketchFittedSpline = sketch.sketchCurves.sketchFittedSplines.add(coll)
        self.lobeSplines[d] = spline

        count = spline.fitPoints.count
        for i in range(1, count - 1):
            interior: adsk.fusion.SketchPoint = spline.fitPoints.item(i)
            interior.isFixed = True
        firstFit: adsk.fusion.SketchPoint = spline.fitPoints.item(0)
        lastFit: adsk.fusion.SketchPoint = spline.fitPoints.item(count - 1)
        sketch.geometricConstraints.addCoincident(firstFit, rootCircle)
        sketch.geometricConstraints.addCoincident(lastFit, rootCircle)

        line1: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(s * E + Rv * math.cos(d * math.pi), Rv * math.sin(d * math.pi), 0))
        sketch.geometricConstraints.addCoincident(line1.endSketchPoint, firstFit)
        sketch.geometricConstraints.addHorizontal(line1)

        a2 = d * math.pi - 2 * math.pi / L
        line2: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre,
            adsk.core.Point3D.create(s * E + Rv * math.cos(a2), Rv * math.sin(a2), 0))
        sketch.geometricConstraints.addCoincident(line2.endSketchPoint, lastFit)

        a3 = d * math.pi - math.pi / L
        angDim: adsk.fusion.SketchAngularDimension = sketch.sketchDimensions.addAngularDimension(
            line1, line2,
            adsk.core.Point3D.create(s * E + 0.4 * Rv * math.cos(a3), 0.4 * Rv * math.sin(a3), 0))
        angDim.parameter.expression = '360 deg / {}'.format(pn(PARAM_LOBES))

    # -- S07-S10 -----------------------------------------------------------

    def buildDisk(self, d: int):
        component = self.getComponent()
        L = self.vals['L']
        sketch = self.lobeSketches[d]

        base = component.bRepBodies.count
        sectorProfile = self._profileHolding(sketch, self.lobeSplines[d])
        ext = component.features.extrudeFeatures.createInput(
            sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(self._extentOf(self.parameterName(PARAM_DISC_THICKNESS)),
                             adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(ext)
        sectorBody: adsk.fusion.BRepBody = extrude.bodies.item(0)
        sectorBody.name = 'Cycloidal Disk {}'.format(d + 1)

        # S08: Disk Axis at the disc centre, on the cap face picked by normal.
        capFace: adsk.fusion.BRepFace = extrude.startFaces.item(0)
        self.buildDiskAxis(capFace, d)

        # S09: circular pattern of the sector extrude about the Disk Axis.
        self._pattern(extrude, self.diskAxes[d], L, component)

        # S10: join this disc's own L sectors.
        joined = self._joinRange(component, base, L)
        joined.name = 'Cycloidal Disk {}'.format(d + 1)
        self.diskBodies[d] = joined

    # -- S11 ---------------------------------------------------------------

    def buildOutputHoleSketch(self, d: int):
        v = self.vals
        E = v['E']
        Rop = v['Rop']
        s = self._sign(d)
        pn = self.parameterName

        sketch = self._newSketch('Output Hole {}'.format(d + 1), self._planeFor(d))
        self.holeSketches[d] = sketch
        localOrigin = self._anchorSketch(sketch)
        diskCentre = self._discCentre(sketch, localOrigin, d)

        holeCircle = self._circleOn(sketch, s * E, 0, Rop, diskCentre,
                                    pn(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER), True)
        self._label(sketch, 'Output Hole Circle', holeCircle)

        hole: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E + Rop, 0, 0), self.D_hole / 2)
        r = self.D_hole / 2
        holeDim: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(
            hole, adsk.core.Point3D.create(s * E + Rop + 0.75 * r, 0.75 * r, 0))
        holeDim.parameter.expression = pn(PARAM_OUTPUT_HOLE_DIAMETER)
        sketch.geometricConstraints.addCoincident(hole.centerSketchPoint, holeCircle)
        spoke: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        spoke.isConstruction = True
        sketch.geometricConstraints.addHorizontal(spoke)
        self.outputHoles[d] = hole

    # -- S12, S13 ----------------------------------------------------------

    def buildOutputHoles(self, d: int):
        component = self.getComponent()
        sketch = self.holeSketches[d]
        holeProfile = self._profileHolding(sketch, self.outputHoles[d])

        ci = component.features.extrudeFeatures.createInput(
            holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(self._extentOf(self.parameterName(PARAM_DISC_THICKNESS)),
                            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        cut: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(ci)

        self._pattern(cut, self.diskAxes[d], self.vals['M'], component)

    # -- S14, S15 ----------------------------------------------------------

    def buildDiskBore(self, d: int):
        v = self.vals
        E = v['E']
        CBD = v['CBD']
        clr = v['clr']
        s = self._sign(d)
        pn = self.parameterName
        component = self.getComponent()

        sketch = self._newSketch('Disc Bore {}'.format(d + 1), self._planeFor(d))
        localOrigin = self._anchorSketch(sketch)
        diskCentre = self._discCentre(sketch, localOrigin, d)
        self._circleOn(sketch, s * E, 0, (CBD + clr) / 2, diskCentre,
                       '{} + {}'.format(pn(PARAM_CENTER_BEARING_DIAMETER), pn(PARAM_BEARING_CLEARANCE)),
                       False)

        ci = component.features.extrudeFeatures.createInput(
            self._allProfiles(sketch), adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(self._extentOf(pn(PARAM_DISC_THICKNESS)),
                            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        component.features.extrudeFeatures.add(ci)

    # -- S16-S18 -----------------------------------------------------------

    def buildCam(self):
        v = self.vals
        E = v['E']
        CBD = v['CBD']
        ISD = v['ISD']
        D = v['D']
        pn = self.parameterName
        component = self.getComponent()
        camBodies = []

        for d in range(D):
            s = self._sign(d)
            sketch = self._newSketch('Eccentric Cam {}'.format(d + 1), self._planeFor(d))
            localOrigin = self._anchorSketch(sketch)
            diskCentre = self._discCentre(sketch, localOrigin, d)
            self._circleOn(sketch, s * E, 0, CBD / 2, diskCentre, pn(PARAM_CENTER_BEARING_DIAMETER), False)
            if ISD > 0:
                self._circleOn(sketch, 0, 0, ISD / 2, localOrigin, pn(PARAM_INPUT_SHAFT_DIAMETER), False)

            # S17: the annulus has two loops; with no bore the sketch has one profile.
            camProfile: adsk.fusion.Profile = sketch.profiles.item(0)
            if ISD > 0:
                found = False
                for i in range(sketch.profiles.count):
                    candidate: adsk.fusion.Profile = sketch.profiles.item(i)
                    if candidate.profileLoops.count == 2:
                        camProfile = candidate
                        found = True
                        break
                if not found:
                    raise Exception('No two-loop cam profile in sketch "{}".'.format(sketch.name))

            ext = component.features.extrudeFeatures.createInput(
                camProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            nT = pn(PARAM_DISC_THICKNESS)
            nG = pn(PARAM_DISC_GAP)
            if d < D - 1:
                extent = '{} + {}'.format(nT, nG)
            else:
                extent = nT
            ext.setOneSideExtent(self._extentOf(extent), adsk.fusion.ExtentDirections.PositiveExtentDirection)
            extrude: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(ext)
            body: adsk.fusion.BRepBody = extrude.bodies.item(0)
            body.name = 'Eccentric Cam {}'.format(d + 1)
            camBodies.append(body)

        # S18
        target: adsk.fusion.BRepBody = camBodies[0]
        if D > 1:
            tools = adsk.core.ObjectCollection.create()
            for k in range(1, len(camBodies)):
                tools.add(camBodies[k])
            self._join(target, tools, component)
        target.name = 'Eccentric Cam'
        self.cam = target

    # -- S19-S27 -----------------------------------------------------------

    def buildRingPins(self):
        v = self.vals
        R = v['R']
        E = v['E']
        N = v['N']
        Wall = v['Wall']
        Rr = self.Rr
        pn = self.parameterName
        component = self.getComponent()

        # S19
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        housingPlane.name = 'Ring Housing Plane'

        # S20
        sketch = self._newSketch('Housing Ring', housingPlane)
        localOrigin = self._anchorSketch(sketch)
        self._circleOn(sketch, 0, 0, R - Rr + 2 * E + Wall, localOrigin, pn(PARAM_HOUSING_OUTER_DIAMETER), False)
        self._circleOn(sketch, 0, 0, R - Rr - Wall, localOrigin, pn(PARAM_HOUSING_INNER_DIAMETER), False)

        # S21
        annulus: adsk.fusion.Profile = sketch.profiles.item(0)
        found = False
        for i in range(sketch.profiles.count):
            candidate: adsk.fusion.Profile = sketch.profiles.item(i)
            if candidate.profileLoops.count == 2:
                annulus = candidate
                found = True
                break
        if not found:
            raise Exception('No two-loop annulus profile in sketch "Housing Ring".')
        ext = component.features.extrudeFeatures.createInput(
            annulus, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(self._extentOf(pn(PARAM_BASE_THICKNESS)),
                             adsk.fusion.ExtentDirections.NegativeExtentDirection)
        housingExtrude: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(ext)
        self.housingRing = housingExtrude.bodies.item(0)
        self.housingRing.name = 'Housing Ring'

        # S22
        self.driveAxis = self._capAxis(housingExtrude, localOrigin, 'Drive Axis')

        # S23
        ro = R - Rr + 2 * E + Wall
        casing = self._newSketch('Ring Casing', self.plane)
        casingOrigin = self._anchorSketch(casing)
        self._circleOn(casing, 0, 0, ro, casingOrigin, pn(PARAM_HOUSING_OUTER_DIAMETER), False)
        pts = adsk.core.ObjectCollection.create()
        for (x, y) in contour_pitch(v):
            pts.add(adsk.core.Point3D.create(x, y, 0))
        contour: adsk.fusion.SketchFittedSpline = casing.sketchCurves.sketchFittedSplines.add(pts)
        cCount = contour.fitPoints.count
        firstPt: adsk.fusion.SketchPoint = contour.fitPoints.item(0)
        lastPt: adsk.fusion.SketchPoint = contour.fitPoints.item(cCount - 1)
        half = math.pi / N
        casing.sketchCurves.sketchLines.addByTwoPoints(
            firstPt, adsk.core.Point3D.create(ro * math.cos(-half), ro * math.sin(-half), 0))
        casing.sketchCurves.sketchLines.addByTwoPoints(
            lastPt, adsk.core.Point3D.create(ro * math.cos(half), ro * math.sin(half), 0))

        # S24: the wedge is the smallest profile that carries the contour.
        wedge: adsk.fusion.Profile = adsk.fusion.Profile.cast(None)
        wedgeArea = math.inf
        for i in range(casing.profiles.count):
            candidate: adsk.fusion.Profile = casing.profiles.item(i)
            carries = False
            for j in range(candidate.profileLoops.count):
                loop: adsk.fusion.ProfileLoop = candidate.profileLoops.item(j)
                for k in range(loop.profileCurves.count):
                    pc: adsk.fusion.ProfileCurve = loop.profileCurves.item(k)
                    if pc.sketchEntity == contour:
                        carries = True
            if not carries:
                continue
            area = candidate.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area
            if area < wedgeArea:
                wedgeArea = area
                wedge = candidate
        if wedge is None:
            raise Exception('No casing wedge profile carries the contour.')

        base = component.bRepBodies.count
        sectorInput = component.features.extrudeFeatures.createInput(
            wedge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sectorInput.setTwoSidesExtent(self._extentOf(self._stackTopExpr()), self._extentOf('1 mm'))
        sectorFeature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(sectorInput)

        # S25
        self._pattern(sectorFeature, self.driveAxis, N, component)

        # S26
        casingBody = self._joinRange(component, base, N)

        # S27
        tools = adsk.core.ObjectCollection.create()
        tools.add(casingBody)
        self._join(self.housingRing, tools, component)
        self.housingRing.name = 'Housing'
        self.ringCasing = None

    # -- S28-S34 -----------------------------------------------------------

    def buildOutputPins(self):
        v = self.vals
        Rop = v['Rop']
        D_pin = v['D_pin']
        M = v['M']
        pn = self.parameterName
        component = self.getComponent()

        # S28
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString(self._stackTopExpr() + ' + 1 mm'))
        platePlane: adsk.fusion.ConstructionPlane = component.constructionPlanes.add(planeInput)
        platePlane.name = 'Output Plate Plane'

        # S29
        sketch = self._newSketch('Output Plate', platePlane)
        localOrigin = self._anchorSketch(sketch)
        plateRadius = Rop + D_pin / 2 + v['Wall']
        self._circleOn(sketch, 0, 0, plateRadius, localOrigin, pn(PARAM_OUTPUT_PLATE_DIAMETER), False)
        outPinCircle = self._circleOn(sketch, 0, 0, Rop, localOrigin,
                                      pn(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER), True)
        pin: adsk.fusion.SketchCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2)
        pinDim: adsk.fusion.SketchDiameterDimension = sketch.sketchDimensions.addDiameterDimension(
            pin, adsk.core.Point3D.create(Rop + 0.75 * D_pin / 2, 0.75 * D_pin / 2, 0))
        pinDim.parameter.expression = '{} - 2 * {}'.format(
            pn(PARAM_OUTPUT_HOLE_DIAMETER), pn(PARAM_ECCENTRICITY))
        sketch.geometricConstraints.addCoincident(pin.centerSketchPoint, outPinCircle)
        pinSpoke: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        pinSpoke.isConstruction = True
        sketch.geometricConstraints.addHorizontal(pinSpoke)

        # S30
        plateInput = component.features.extrudeFeatures.createInput(
            self._allProfiles(sketch), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        plateInput.setOneSideExtent(self._extentOf(pn(PARAM_OUTPUT_PLATE_THICKNESS)),
                                    adsk.fusion.ExtentDirections.PositiveExtentDirection)
        plateFeature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(plateInput)
        self.outputPlate = plateFeature.bodies.item(0)
        self.outputPlate.name = 'Output Plate'
        pinBase = component.bRepBodies.count

        # S31
        pinProfile: adsk.fusion.Profile = adsk.fusion.Profile.cast(None)
        for i in range(sketch.profiles.count):
            candidate: adsk.fusion.Profile = sketch.profiles.item(i)
            if candidate.profileLoops.count != 1:
                continue
            loop: adsk.fusion.ProfileLoop = candidate.profileLoops.item(0)
            if loop.profileCurves.count != 1:
                continue
            pc: adsk.fusion.ProfileCurve = loop.profileCurves.item(0)
            if pc.sketchEntity == pin:
                pinProfile = candidate
                break
        if pinProfile is None:
            raise Exception('No single-loop pin profile in sketch "Output Plate".')
        pinExt = component.features.extrudeFeatures.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pinExt.setTwoSidesExtent(self._extentOf(pn(PARAM_OUTPUT_PLATE_THICKNESS)),
                                 self._extentOf(self._stackTopExpr() + ' + 1 mm'))
        pinFeature: adsk.fusion.ExtrudeFeature = component.features.extrudeFeatures.add(pinExt)
        pinBody: adsk.fusion.BRepBody = pinFeature.bodies.item(0)
        pinBody.name = 'Output Pin'

        # S32
        tools = adsk.core.ObjectCollection.create()
        tools.add(pinBody)
        ci = component.features.combineFeatures.createInput(self.outputPlate, tools)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = True
        combineFeature: adsk.fusion.CombineFeature = component.features.combineFeatures.add(ci)

        # S33
        chamferFeature = self._chamferCapRims(pinBody)

        # S34
        coll = adsk.core.ObjectCollection.create()
        coll.add(pinFeature)
        coll.add(combineFeature)
        if chamferFeature is not None:
            coll.add(chamferFeature)
        self._patternCollection(coll, self.driveAxis, M, component)
        for k in range(M):
            patterned: adsk.fusion.BRepBody = component.bRepBodies.item(pinBase + k)
            patterned.name = 'Output Pin {}'.format(k + 1)

    def _chamferCapRims(self, body: adsk.fusion.BRepBody):
        """[CYCLOIDAL-F-CHAMFERS] Chamfer the outer loop of the two axially extreme caps."""
        if self.chamferSize <= 0:
            return None
        planeGeom: adsk.core.Plane = self.plane.geometry
        axis: adsk.core.Vector3D = planeGeom.normal
        ref: adsk.core.Point3D = planeGeom.origin

        capFaces = []
        heights = []
        for i in range(body.faces.count):
            face: adsk.fusion.BRepFace = body.faces.item(i)
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            facePlane: adsk.core.Plane = adsk.core.Plane.cast(face.geometry)
            n: adsk.core.Vector3D = facePlane.normal
            if abs(n.dotProduct(axis)) <= 0.999:
                continue
            o: adsk.core.Point3D = facePlane.origin
            h = (o.x - ref.x) * axis.x + (o.y - ref.y) * axis.y + (o.z - ref.z) * axis.z
            capFaces.append(face)
            heights.append(h)
        if len(capFaces) == 0:
            return None
        hmin = min(heights)
        hmax = max(heights)

        edges = adsk.core.ObjectCollection.create()
        for idx in range(len(capFaces)):
            h = heights[idx]
            if not (abs(h - hmin) < 1e-4 or abs(h - hmax) < 1e-4):
                continue
            capFace: adsk.fusion.BRepFace = capFaces[idx]
            for j in range(capFace.loops.count):
                loop: adsk.fusion.BRepLoop = capFace.loops.item(j)
                if not loop.isOuter:
                    continue
                for k in range(loop.edges.count):
                    edges.add(loop.edges.item(k))
        if edges.count == 0:
            return None

        chamferFeatures = self.getComponent().features.chamferFeatures
        ci = chamferFeatures.createInput2()
        ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByString(self.parameterName(PARAM_CHAMFER_SIZE)), False)
        try:
            return chamferFeatures.add(ci)
        except RuntimeError as err:
            futil.log('cycloidal: chamfer skipped on {}: {}'.format(body.name, err))
            self.chamfersSkipped += 1
            return None

    # -- S35 ---------------------------------------------------------------

    def buildChamfers(self):
        if self.chamferSize <= 0:
            return
        for d in range(self.vals['D']):
            self._chamferCapRims(self.diskBodies[d])
        if self.ringCasing is not None:
            self._chamferCapRims(self.ringCasing)
        if self.housingRing is not None:
            self._chamferCapRims(self.housingRing)
        if self.outputPlate is not None:
            self._chamferCapRims(self.outputPlate)

    # -- S36 ---------------------------------------------------------------

    def buildSubComponents(self):
        component = self.getComponent()
        rotors = []
        housing = []
        cam = []
        output = []
        for i in range(component.bRepBodies.count):
            body: adsk.fusion.BRepBody = component.bRepBodies.item(i)
            name = body.name
            if name.startswith('Cycloidal Disk'):
                rotors.append(body)
            elif name == 'Housing':
                housing.append(body)
            elif name == 'Eccentric Cam':
                cam.append(body)
            elif name == 'Output Plate' or name.startswith('Output Pin'):
                output.append(body)

        groups = [('Rotor Discs', rotors), ('Housing', housing), ('Eccentric Cam', cam), ('Output', output)]
        for (groupName, members) in groups:
            if len(members) == 0:
                continue
            occ: adsk.fusion.Occurrence = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = groupName
            for member in members:
                moving: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(member)
                moving.moveToComponent(occ)

        solids.hide_construction_geometry(component)
