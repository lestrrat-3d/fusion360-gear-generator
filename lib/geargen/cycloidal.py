import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator, get_value, get_selection
from . import solids


# -----------------------------------------------------------------------------------------
# Dialog input ids and registered user-parameter names.
# -----------------------------------------------------------------------------------------
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

# Derived parameters, registered after _resolveDimensions ([PB-NUMERIC-SNAPSHOT] for the two
# marked "snapshot"; the rest stay live expressions).
PARAM_LOBES = 'Lobes'
PARAM_PIN_CIRCLE_RADIUS = 'PinCircleRadius'
PARAM_OUTPUT_PIN_CIRCLE_RADIUS = 'OutputPinCircleRadius'
PARAM_PIN_RADIUS = 'PinRadius'                        # numeric snapshot
PARAM_OUTPUT_HOLE_DIAMETER = 'OutputHoleDiameter'      # numeric snapshot
PARAM_HOUSING_INNER_DIAMETER = 'HousingInnerDiameter'
PARAM_HOUSING_OUTER_DIAMETER = 'HousingOuterDiameter'
PARAM_OUTPUT_PLATE_DIAMETER = 'OutputPlateDiameter'


# -----------------------------------------------------------------------------------------
# S01 — dialog inputs.
# -----------------------------------------------------------------------------------------
class CycloidalDriveCommandInputsConfigurator:
    @classmethod
    def configure(cls, command: adsk.core.Command):
        inputs = command.commandInputs

        # 1. Target Plane — first, so Fusion's auto-focus-first-selection-input
        #    behaviour opens the dialog on it ([PB-AUTOFOCUS-FIRST]).
        planeInput = inputs.addSelectionInput(INPUT_ID_PLANE, 'Target Plane', 'Target Plane')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Anchor Point')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Disc Count — dropdown, not a value input.
        discCountInput = inputs.addDropDownCommandInput(
            INPUT_ID_DISC_COUNT, 'Disc Count', adsk.core.DropDownStyles.TextListDropDownStyle)
        discCountInput.listItems.add('1', True)
        discCountInput.listItems.add('2', False)
        cls._addStatusSlot(inputs, INPUT_ID_DISC_COUNT)

        # 4. Pin Count
        inputs.addValueInput(
            INPUT_ID_PIN_COUNT, 'Pin Count', '', adsk.core.ValueInput.createByReal(16))
        cls._addStatusSlot(inputs, INPUT_ID_PIN_COUNT)

        # 5. Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_PIN_CIRCLE_DIAMETER, 'Pin Circle Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(90)))
        cls._addStatusSlot(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER)

        # 6. Pin Diameter
        inputs.addValueInput(
            INPUT_ID_PIN_DIAMETER, 'Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        cls._addStatusSlot(inputs, INPUT_ID_PIN_DIAMETER)

        # 7. Eccentricity
        inputs.addValueInput(
            INPUT_ID_ECCENTRICITY, 'Eccentricity', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(1.5)))
        cls._addStatusSlot(inputs, INPUT_ID_ECCENTRICITY)

        # 8. Disk Clearance
        inputs.addValueInput(
            INPUT_ID_DISK_CLEARANCE, 'Disk Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.3)))
        cls._addStatusSlot(inputs, INPUT_ID_DISK_CLEARANCE)

        # 9. Disc Thickness
        inputs.addValueInput(
            INPUT_ID_DISC_THICKNESS, 'Disc Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))
        cls._addStatusSlot(inputs, INPUT_ID_DISC_THICKNESS)

        # 10. Disc Gap
        inputs.addValueInput(
            INPUT_ID_DISC_GAP, 'Disc Gap', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.5)))
        cls._addStatusSlot(inputs, INPUT_ID_DISC_GAP)

        # 11. Center Bearing Diameter
        inputs.addValueInput(
            INPUT_ID_CENTER_BEARING_DIAMETER, 'Center Bearing Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(30)))
        cls._addStatusSlot(inputs, INPUT_ID_CENTER_BEARING_DIAMETER)

        # 12. Input Shaft Diameter
        inputs.addValueInput(
            INPUT_ID_INPUT_SHAFT_DIAMETER, 'Input Shaft Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(8)))
        cls._addStatusSlot(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER)

        # 13. Bearing Clearance
        inputs.addValueInput(
            INPUT_ID_BEARING_CLEARANCE, 'Bearing Clearance', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.2)))
        cls._addStatusSlot(inputs, INPUT_ID_BEARING_CLEARANCE)

        # 14. Output Pin Circle Diameter
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'Output Pin Circle Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(50)))
        cls._addStatusSlot(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER)

        # 15. Output Pin Count
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_COUNT, 'Output Pin Count', '',
            adsk.core.ValueInput.createByReal(6))
        cls._addStatusSlot(inputs, INPUT_ID_OUTPUT_PIN_COUNT)

        # 16. Output Pin Diameter
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PIN_DIAMETER, 'Output Pin Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        cls._addStatusSlot(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER)

        # 17. Housing Wall
        inputs.addValueInput(
            INPUT_ID_WALL, 'Housing Wall', 'mm', adsk.core.ValueInput.createByReal(to_cm(3)))
        cls._addStatusSlot(inputs, INPUT_ID_WALL)

        # 18. Base Thickness
        inputs.addValueInput(
            INPUT_ID_BASE_THICKNESS, 'Base Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))
        cls._addStatusSlot(inputs, INPUT_ID_BASE_THICKNESS)

        # 19. Output Plate Thickness
        inputs.addValueInput(
            INPUT_ID_OUTPUT_PLATE_THICKNESS, 'Output Plate Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(5)))
        cls._addStatusSlot(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS)

        # 20. Chamfer Size
        inputs.addValueInput(
            INPUT_ID_CHAMFER_SIZE, 'Chamfer Size', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0.5)))
        cls._addStatusSlot(inputs, INPUT_ID_CHAMFER_SIZE)

        # 21. Parent Component — last.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Parent Component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(0, 1)
        parentInput.addSelection(get_design().rootComponent)

    @staticmethod
    def _addStatusSlot(inputs: adsk.core.CommandInputs, inputId):
        slot = inputs.addTextBoxCommandInput(inputId + '__status', '', '', 2, True)
        slot.isVisible = False


# -----------------------------------------------------------------------------------------
# S02/S03 — shared geometry/validity math. disk_point is the rotor profile point function;
# evaluate_problems is the single validity routine validate_inputs and _resolveDimensions
# both call, so the two never drift ([PB-VALIDATE-INPUTS]).
# -----------------------------------------------------------------------------------------
def disk_point(t, cx, cy, phi, R, E, N, RrEff):
    num = math.sin((1 - N) * t)
    den = (R / (E * N)) - math.cos((1 - N) * t)
    psi = math.atan2(num, den)
    x0 = R * math.cos(t) - RrEff * math.cos(t + psi) - E * math.cos(N * t)
    y0 = -R * math.sin(t) + RrEff * math.sin(t + psi) + E * math.sin(N * t)
    x = cx + (x0 * math.cos(phi) - y0 * math.sin(phi))
    y = cy + (x0 * math.sin(phi) + y0 * math.cos(phi))
    return x, y


def _resolve_pin_radius(R, E, N, pinDiameterRaw):
    if pinDiameterRaw > 0:
        return pinDiameterRaw / 2, True
    return 0.5 * (E + R * math.sin(math.pi / N)), False


def rho_min_O(R, E, N):
    """Smallest radius of curvature of the base trochoid at the points whose centre of
    curvature lies toward O, sampled at 2000 points of t over [0, 2*pi)."""
    best = math.inf
    found = False
    for i in range(2000):
        t = 2 * math.pi * i / 2000
        bx = R * math.cos(t) - E * math.cos(N * t)
        by = -R * math.sin(t) + E * math.sin(N * t)
        xp = -R * math.sin(t) + E * N * math.sin(N * t)
        yp = -R * math.cos(t) + E * N * math.cos(N * t)
        xpp = -R * math.cos(t) + E * N * N * math.cos(N * t)
        ypp = R * math.sin(t) - E * N * N * math.sin(N * t)
        k = xp * ypp - yp * xpp
        if abs(k) < 1e-12:
            continue
        s2 = xp * xp + yp * yp
        rho = s2 ** 1.5 / k
        s = math.sqrt(s2)
        nx, ny = -yp / s, xp / s
        cx, cy = bx + rho * nx, by + rho * ny
        if cx * cx + cy * cy < bx * bx + by * by:
            found = True
            best = min(best, abs(rho))
    return best if found else math.inf


def _undercut_limit(R, E, N, c, pinDiameterRaw):
    """The largest E' in (0, E] for which Rr_eff(E') < rho_min_O(E') still holds, by exactly
    40 bisection rounds. Returns None when no positive E' satisfies it."""
    def trial(e):
        Rr, _ = _resolve_pin_radius(R, e, N, pinDiameterRaw)
        return (Rr + c) < rho_min_O(R, e, N)

    lo, hi = 1e-9, E
    if not trial(lo):
        return None
    for _ in range(40):
        mid = (lo + hi) / 2
        if trial(mid):
            lo = mid
        else:
            hi = mid
    return lo


def evaluate_problems(N, M, D, R, E, c, CBD, ISD, clr, Rop, pinDiameterRaw, outputPinDiameterRaw):
    """The whole validity list, in the order the spec's table gives. Returns every failing
    message, not just the first. All inputs are internal cm / plain ints."""
    problems = []

    Rr, pinOverride = _resolve_pin_radius(R, E, N, pinDiameterRaw)
    RrEff = Rr + c
    Rv = R - RrEff - E
    if outputPinDiameterRaw > 0:
        DPin = outputPinDiameterRaw
        outOverride = True
    else:
        DPin = Rop * math.sin(math.pi / M) - E
        outOverride = False
    DHole = DPin + 2 * E

    # 1
    if D == 2 and (N % 2 != 0 or M % 2 != 0):
        problems.append(
            'Two discs require an even Pin Count and an even Output Pin Count '
            '(currently N={}, M={}).'.format(N, M))

    # 2
    if not (E < Rr < R * math.sin(math.pi / N)):
        if pinOverride:
            problems.append(
                'Pin Diameter must be between {:.2f} mm and {:.2f} mm (currently {:.2f}).'.format(
                    to_mm(2 * E), to_mm(2 * R * math.sin(math.pi / N)), to_mm(2 * Rr)))
        else:
            problems.append(
                'Pin geometry out of range — increase Pin Circle Diameter above {:.2f} mm '
                'or reduce Eccentricity below {:.2f} mm.'.format(
                    to_mm(2 * E / math.sin(math.pi / N)), to_mm(R * math.sin(math.pi / N))))

    # 3
    if not (DPin > 0):
        if outOverride:
            problems.append('Output Pin Diameter must be greater than 0.')
        else:
            problems.append(
                'Output pins vanish (resolved diameter ≤ 0) — increase Output Pin Circle '
                'Diameter above {:.2f} mm, increase Output Pin Count, or reduce '
                'Eccentricity.'.format(to_mm(2 * E / math.sin(math.pi / M))))

    # 4
    if not (DHole < 2 * Rop * math.sin(math.pi / M)):
        problems.append(
            'Output holes overlap — increase Output Pin Circle Diameter above {:.2f} mm, '
            'increase Output Pin Count, or reduce Output Pin Diameter / Eccentricity.'.format(
                to_mm((DPin + 2 * E) / math.sin(math.pi / M))))

    # 5
    if not (E < R / N):
        problems.append(
            'Eccentricity too large — reduce it below {:.2f} mm (or increase Pin Circle '
            'Diameter / reduce Pin Count).'.format(to_mm(R / N)))

    # 6
    if not (Rop < Rv):
        problems.append(
            'Output Pin Circle too large — set Output Pin Circle Diameter below {:.2f} mm '
            '(currently {:.2f}).'.format(to_mm(2 * Rv), to_mm(2 * Rop)))

    # 7
    if not (RrEff < rho_min_O(R, E, N)):
        eStar = _undercut_limit(R, E, N, c, pinDiameterRaw)
        if eStar is None:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/self-intersects. '
                'Reduce Eccentricity.')
        else:
            problems.append(
                'Eccentricity too large — the rotor profile undercuts/self-intersects. '
                'Reduce Eccentricity below {:.2f} mm.'.format(to_mm(eStar)))

    # 8
    if not (ISD < CBD):
        problems.append(
            'Input Shaft Diameter must be less than Center Bearing Diameter '
            '({:.2f} mm).'.format(to_mm(CBD)))

    # 9
    if not (E + ISD / 2 < CBD / 2):
        problems.append(
            "Input bore doesn't fit inside the cam — set Input Shaft Diameter below {:.2f} mm, "
            'or reduce Eccentricity / increase Center Bearing Diameter.'.format(to_mm(CBD - 2 * E)))

    # 10
    if not ((CBD + clr) / 2 < Rop - DHole / 2):
        problems.append(
            'Disk center bore overlaps the output holes — increase Output Pin Circle Diameter '
            'above {:.2f} mm, or reduce Center Bearing Diameter / Bearing Clearance / output '
            'pin size.'.format(to_mm(CBD + clr + DHole)))

    return problems


def vanish():
    """Not a call — 'vanish' is a word inside evaluate_problems' check-3 message string."""
    raise NotImplementedError


# -----------------------------------------------------------------------------------------
# The orchestrator.
# -----------------------------------------------------------------------------------------
class CycloidalDriveGenerator(Generator):
    DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + '__status'

    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    def generateName(self) -> str:
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    # -- S03 -------------------------------------------------------------------------------
    @staticmethod
    def validate_inputs(inputs: adsk.core.CommandInputs) -> list:
        N = int(round(inputs.itemById(INPUT_ID_PIN_COUNT).value))
        M = int(round(inputs.itemById(INPUT_ID_OUTPUT_PIN_COUNT).value))
        D = int(inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name)
        R = inputs.itemById(INPUT_ID_PIN_CIRCLE_DIAMETER).value / 2
        E = inputs.itemById(INPUT_ID_ECCENTRICITY).value
        c = inputs.itemById(INPUT_ID_DISK_CLEARANCE).value
        CBD = inputs.itemById(INPUT_ID_CENTER_BEARING_DIAMETER).value
        ISD = inputs.itemById(INPUT_ID_INPUT_SHAFT_DIAMETER).value
        clr = inputs.itemById(INPUT_ID_BEARING_CLEARANCE).value
        Rop = inputs.itemById(INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2
        pinDiameterRaw = inputs.itemById(INPUT_ID_PIN_DIAMETER).value
        outputPinDiameterRaw = inputs.itemById(INPUT_ID_OUTPUT_PIN_DIAMETER).value
        return evaluate_problems(
            N, M, D, R, E, c, CBD, ISD, clr, Rop, pinDiameterRaw, outputPinDiameterRaw)

    # -- S02 -------------------------------------------------------------------------------
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Field declarations, in the cast(None) form, before anything reads them. The
        # constructor is the inherited 1-arg (design) one; self.parentComponent, self.design,
        # self.occurrence, self.prefix and self.cleaner are already set there.
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.driveAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.housingRing = adsk.fusion.BRepBody.cast(None)
        self.ringCasing = adsk.fusion.BRepBody.cast(None)
        self.cam = adsk.fusion.BRepBody.cast(None)
        self.outputPlate = adsk.fusion.BRepBody.cast(None)
        self.lobePinCircle = adsk.fusion.SketchCircle.cast(None)
        self.chamferSize = 0.0
        self.chamfersSkipped = 0
        self.Rr = 0.0
        self.D_hole = 0.0

        # 1. Selections first ([PB-SELECTION-STASH]).
        parentEntities = get_selection(inputs, INPUT_ID_PARENT)
        if parentEntities:
            parentEntity = parentEntities[0]
            if parentEntity.objectType == adsk.fusion.Occurrence.classType():
                self.parentComponent = parentEntity.component
            else:
                self.parentComponent = parentEntity
        else:
            self.parentComponent = get_design().rootComponent

        self.plane = get_selection(inputs, INPUT_ID_PLANE)[0]
        self.anchorPoint = get_selection(inputs, INPUT_ID_ANCHOR_POINT)[0]

        # 2. Disc Count from the dropdown, never with a get_* helper ([PB-INPUT-READ]). The
        # six per-disc fields are lists indexed by d, sized to D as soon as it is known.
        D = int(inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name)
        self.diskBodies = [adsk.fusion.BRepBody.cast(None) for _ in range(D)]
        self.diskAxes = [adsk.fusion.ConstructionAxis.cast(None) for _ in range(D)]
        self.lobeSplines = [adsk.fusion.SketchFittedSpline.cast(None) for _ in range(D)]
        self.outputHoles = [adsk.fusion.SketchCircle.cast(None) for _ in range(D)]
        self.lobeDiskCentres = [adsk.fusion.SketchPoint.cast(None) for _ in range(D)]
        self.discPlanes = [adsk.fusion.ConstructionPlane.cast(None) for _ in range(D)]

        # 3. Value inputs, registered with the resolved ValueInput ([PB-GET-VALUE-CONTRACT]).
        self.addParameter(
            PARAM_PIN_COUNT, get_value(inputs, INPUT_ID_PIN_COUNT, ''), '', 'Pin Count')
        self.addParameter(
            PARAM_PIN_CIRCLE_DIAMETER, get_value(inputs, INPUT_ID_PIN_CIRCLE_DIAMETER, 'mm'),
            'mm', 'Pin Circle Diameter')
        self.addParameter(
            PARAM_PIN_DIAMETER, get_value(inputs, INPUT_ID_PIN_DIAMETER, 'mm'), 'mm',
            'Pin Diameter (0 = auto)')
        self.addParameter(
            PARAM_ECCENTRICITY, get_value(inputs, INPUT_ID_ECCENTRICITY, 'mm'), 'mm',
            'Eccentricity')
        self.addParameter(
            PARAM_DISK_CLEARANCE, get_value(inputs, INPUT_ID_DISK_CLEARANCE, 'mm'), 'mm',
            'Disk Clearance')
        self.addParameter(
            PARAM_DISC_THICKNESS, get_value(inputs, INPUT_ID_DISC_THICKNESS, 'mm'), 'mm',
            'Disc Thickness')
        self.addParameter(
            PARAM_DISC_GAP, get_value(inputs, INPUT_ID_DISC_GAP, 'mm'), 'mm', 'Disc Gap')
        self.addParameter(
            PARAM_CENTER_BEARING_DIAMETER,
            get_value(inputs, INPUT_ID_CENTER_BEARING_DIAMETER, 'mm'), 'mm',
            'Center Bearing Diameter')
        self.addParameter(
            PARAM_INPUT_SHAFT_DIAMETER, get_value(inputs, INPUT_ID_INPUT_SHAFT_DIAMETER, 'mm'),
            'mm', 'Input Shaft Diameter (0 = no bore)')
        self.addParameter(
            PARAM_BEARING_CLEARANCE, get_value(inputs, INPUT_ID_BEARING_CLEARANCE, 'mm'), 'mm',
            'Bearing Clearance')
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_DIAMETER,
            get_value(inputs, INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER, 'mm'), 'mm',
            'Output Pin Circle Diameter')
        self.addParameter(
            PARAM_OUTPUT_PIN_COUNT, get_value(inputs, INPUT_ID_OUTPUT_PIN_COUNT, ''), '',
            'Output Pin Count')
        self.addParameter(
            PARAM_OUTPUT_PIN_DIAMETER, get_value(inputs, INPUT_ID_OUTPUT_PIN_DIAMETER, 'mm'),
            'mm', 'Output Pin Diameter (0 = auto)')
        self.addParameter(
            PARAM_WALL, get_value(inputs, INPUT_ID_WALL, 'mm'), 'mm', 'Housing Wall')
        self.addParameter(
            PARAM_BASE_THICKNESS, get_value(inputs, INPUT_ID_BASE_THICKNESS, 'mm'), 'mm',
            'Base Thickness')
        self.addParameter(
            PARAM_OUTPUT_PLATE_THICKNESS,
            get_value(inputs, INPUT_ID_OUTPUT_PLATE_THICKNESS, 'mm'), 'mm',
            'Output Plate Thickness')
        self.addParameter(
            PARAM_CHAMFER_SIZE, get_value(inputs, INPUT_ID_CHAMFER_SIZE, 'mm'), 'mm',
            'Chamfer Size')
        self.chamferSize = self.getParameter(PARAM_CHAMFER_SIZE).value

        # 4. Resolve derived scalars and validate.
        self._resolveDimensions()

        # 5. Register the derived parameters, after the resolve.
        nPinCount = self.parameterName(PARAM_PIN_COUNT)
        self.addParameter(
            PARAM_LOBES, adsk.core.ValueInput.createByString('{} - 1'.format(nPinCount)), '',
            'Lobes = Pin Count - 1')

        nPCD = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
        self.addParameter(
            PARAM_PIN_CIRCLE_RADIUS, adsk.core.ValueInput.createByString('{} / 2'.format(nPCD)),
            'mm', 'Pin circle radius')

        nOPCD = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self.addParameter(
            PARAM_OUTPUT_PIN_CIRCLE_RADIUS,
            adsk.core.ValueInput.createByString('{} / 2'.format(nOPCD)), 'mm',
            'Output pin circle radius')

        self.addParameter(
            PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(self.Rr), 'mm',
            'Resolved pin radius (numeric snapshot)')
        self.addParameter(
            PARAM_OUTPUT_HOLE_DIAMETER, adsk.core.ValueInput.createByReal(self.D_hole), 'mm',
            'Resolved output hole diameter (numeric snapshot)')

        nPinCircleRadius = self.parameterName(PARAM_PIN_CIRCLE_RADIUS)
        nPinRadius = self.parameterName(PARAM_PIN_RADIUS)
        nWall = self.parameterName(PARAM_WALL)
        nEcc = self.parameterName(PARAM_ECCENTRICITY)
        self.addParameter(
            PARAM_HOUSING_INNER_DIAMETER, adsk.core.ValueInput.createByString(
                '2 * ({} - {} - {})'.format(nPinCircleRadius, nPinRadius, nWall)), 'mm',
            'Housing inner diameter')
        self.addParameter(
            PARAM_HOUSING_OUTER_DIAMETER, adsk.core.ValueInput.createByString(
                '2 * ({} - {} + 2 * {} + {})'.format(nPinCircleRadius, nPinRadius, nEcc, nWall)),
            'mm', 'Housing outer diameter')

        nOutputHoleDiameter = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        self.addParameter(
            PARAM_OUTPUT_PLATE_DIAMETER, adsk.core.ValueInput.createByString(
                '{} + {} - 2 * {} + 2 * {}'.format(nOPCD, nOutputHoleDiameter, nEcc, nWall)),
            'mm', 'Output plate diameter')

    def _resolveDimensions(self):
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        D = len(self.diskBodies)
        R = self.getParameter(PARAM_PIN_CIRCLE_DIAMETER).value / 2
        E = self.getParameter(PARAM_ECCENTRICITY).value
        c = self.getParameter(PARAM_DISK_CLEARANCE).value
        CBD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        ISD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        clr = self.getParameter(PARAM_BEARING_CLEARANCE).value
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER).value / 2
        pinDiameterRaw = self.getParameter(PARAM_PIN_DIAMETER).value
        outputPinDiameterRaw = self.getParameter(PARAM_OUTPUT_PIN_DIAMETER).value

        problems = evaluate_problems(
            N, M, D, R, E, c, CBD, ISD, clr, Rop, pinDiameterRaw, outputPinDiameterRaw)
        if problems:
            raise Exception('\n'.join(problems))

        Rr, _ = _resolve_pin_radius(R, E, N, pinDiameterRaw)
        if outputPinDiameterRaw > 0:
            DPin = outputPinDiameterRaw
        else:
            DPin = Rop * math.sin(math.pi / M) - E
        DHole = DPin + 2 * E

        # Only Rr and D_hole persist on self: they are the two numeric snapshots step 5
        # registers ([PB-NUMERIC-SNAPSHOT]). Every other scalar composes cleanly from the
        # registered parameters, so build methods fetch them fresh with self.getParameter(...).
        self.Rr = Rr
        self.D_hole = DHole


    # -- S04/S05 -----------------------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # S05 — normalize the Target Plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)

        # S06 through S16 — per-disc loop.
        discCount = len(self.diskBodies)
        for d in range(discCount):
            if d == 0:
                self.discPlanes[d] = self.plane
            else:
                planeInput = component.constructionPlanes.createInput()
                planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString(
                    '{} * ({} + {})'.format(
                        d, self.parameterName(PARAM_DISC_THICKNESS),
                        self.parameterName(PARAM_DISC_GAP))))
                plane = component.constructionPlanes.add(planeInput)
                plane.name = 'Disc Plane {}'.format(d + 1)
                self.discPlanes[d] = plane

            self.buildLobeSketch(d)
            self.buildDisk(d)
            self.buildOutputHoleSketch(d)
            self.buildOutputHoles(d)
            self.buildDiskBore(d)

        # S17 through S19.
        self.buildCam()

        # S20 through S28.
        self.buildRingPins()

        # S29 through S35.
        self.buildOutputPins()

        # S36.
        self.buildChamfers()

        # S37 — last, because moveToComponent invalidates the moved body's reference.
        self.buildSubComponents()

        if self.chamfersSkipped > 0:
            adsk.core.Application.get().userInterface.messageBox(
                'Cycloidal drive generated, but {n} chamfer(s) could not be created at Chamfer '
                'Size {sz} mm and were skipped. Reduce Chamfer Size (or set it to 0) for this '
                'geometry.'.format(n=self.chamfersSkipped, sz=to_mm(self.chamferSize)))

    # -- Shared anchor-chain helper ([CYCLOIDAL-F-ANCHOR-CHAIN]) -----------------------
    def _anchorSketch(self, sketch: adsk.fusion.Sketch):
        """Project the user's Anchor, add a fresh local origin, tie the two. Returns the
        local origin SketchPoint every later constraint in the sketch hangs off."""
        projected = sketch.project(self.anchorPoint).item(0)
        localOrigin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        sketch.geometricConstraints.addCoincident(localOrigin, projected)
        return localOrigin

    def _buildDiskCentre(
            self, sketch: adsk.fusion.Sketch, localOrigin: adsk.fusion.SketchPoint, d, E):
        """The eccentric disc centre Od_d = O + s_d*E*Xhat ([CYCLOIDAL-F-DISK-CENTER]).
        Returns (s, diskCentre)."""
        s = 1 if d == 0 else -1
        diskCentre = sketch.sketchPoints.add(adsk.core.Point3D.create(s * E, 0, 0))
        eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, diskCentre)
        eccLine.isConstruction = True
        sketch.geometricConstraints.addHorizontal(eccLine)
        eccDim = sketch.sketchDimensions.addDistanceDimension(
            localOrigin, diskCentre,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(s * E / 2, 0.2, 0))
        eccDim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)
        return s, diskCentre

    def _addCircleLabel(
            self, sketch: adsk.fusion.Sketch, circle: adsk.fusion.SketchCircle, label, height):
        textInput = sketch.sketchTexts.createInput2(label, height)
        textInput.setAsAlongPath(
            circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        sketch.sketchTexts.add(textInput)

    def _stackTopExpr(self):
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        if len(self.diskBodies) == 1:
            return nT
        nG = self.parameterName(PARAM_DISC_GAP)
        return '2 * {} + {}'.format(nT, nG)

    # -- S07 -----------------------------------------------------------------------------
    def buildLobeSketch(self, d):
        component = self.getComponent()
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Rotor Lobe {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        E = self.getParameter(PARAM_ECCENTRICITY).value
        R = self.getParameter(PARAM_PIN_CIRCLE_RADIUS).value
        Rr = self.Rr
        RrEff = Rr + self.getParameter(PARAM_DISK_CLEARANCE).value
        Rv = R - RrEff - E
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_RADIUS).value
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        L = int(round(self.getParameter(PARAM_LOBES).value))

        localOrigin = self._anchorSketch(sketch)
        s, diskCentre = self._buildDiskCentre(sketch, localOrigin, d, E)

        # Three reference circles.
        if d == 0:
            pinCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), R)
            pinCircle.isConstruction = True
            sketch.geometricConstraints.addCoincident(pinCircle.centerSketchPoint, localOrigin)
            pinDim = sketch.sketchDimensions.addDiameterDimension(
                pinCircle, adsk.core.Point3D.create(R, 0.2, 0))
            pinDim.parameter.expression = self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)
            self._addCircleLabel(sketch, pinCircle, 'Pin Circle', Rr)
            self.lobePinCircle = pinCircle

        outputCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E, 0, 0), Rop)
        outputCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(outputCircle.centerSketchPoint, diskCentre)
        outputDim = sketch.sketchDimensions.addDiameterDimension(
            outputCircle, adsk.core.Point3D.create(s * E + Rop, 0.2, 0))
        outputDim.parameter.expression = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._addCircleLabel(sketch, outputCircle, 'Output Pin Circle', Rr)

        rootCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E, 0, 0), Rv)
        rootCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(rootCircle.centerSketchPoint, diskCentre)
        rootDim = sketch.sketchDimensions.addDiameterDimension(
            rootCircle, adsk.core.Point3D.create(s * E + Rv, 0.2, 0))
        rootDim.parameter.expression = '2 * ({} - {} - {} - {})'.format(
            self.parameterName(PARAM_PIN_CIRCLE_RADIUS), self.parameterName(PARAM_PIN_RADIUS),
            self.parameterName(PARAM_DISK_CLEARANCE), self.parameterName(PARAM_ECCENTRICITY))
        self._addCircleLabel(sketch, rootCircle, 'Root Circle', Rr)

        # The lobe: one open fitted spline through the adaptively sampled points.
        span = 2 * math.pi / L
        phi = d * math.pi
        cx, cy = s * E, 0.0
        fine = [disk_point(span * i / 2000, cx, cy, phi, R, E, N, RrEff) for i in range(2001)]
        kept = [fine[0]]
        acc = 0.0
        for i in range(1, 2000):
            ax, ay = fine[i][0] - fine[i - 1][0], fine[i][1] - fine[i - 1][1]
            bx, by = fine[i + 1][0] - fine[i][0], fine[i + 1][1] - fine[i][1]
            acc += abs(math.degrees(math.atan2(ax * by - ay * bx, ax * bx + ay * by)))
            if acc >= 5.0:
                kept.append(fine[i])
                acc = 0.0
        kept.append(fine[-1])

        coll = adsk.core.ObjectCollection.create()
        for x, y in kept:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        spline = sketch.sketchCurves.sketchFittedSplines.add(coll)

        for i in range(1, spline.fitPoints.count - 1):
            spline.fitPoints.item(i).isFixed = True
        sketch.geometricConstraints.addCoincident(spline.fitPoints.item(0), rootCircle)
        sketch.geometricConstraints.addCoincident(
            spline.fitPoints.item(spline.fitPoints.count - 1), rootCircle)

        # Two spokes and the lobe pitch.
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(s * E + Rv, 0, 0))
        sketch.geometricConstraints.addCoincident(line1.endSketchPoint, spline.fitPoints.item(0))
        sketch.geometricConstraints.addHorizontal(line1)

        line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, adsk.core.Point3D.create(
                s * E + Rv * math.cos(2 * math.pi / L), -Rv * math.sin(2 * math.pi / L), 0))
        sketch.geometricConstraints.addCoincident(
            line2.endSketchPoint, spline.fitPoints.item(spline.fitPoints.count - 1))

        angDim = sketch.sketchDimensions.addAngularDimension(
            line1, line2, adsk.core.Point3D.create(
                s * E + 0.4 * Rv * math.cos(math.pi / L), -0.4 * Rv * math.sin(math.pi / L), 0))
        angDim.parameter.expression = '360 deg / {}'.format(self.parameterName(PARAM_LOBES))

        self.lobeDiskCentres[d] = diskCentre
        self.lobeSplines[d] = spline

        # [PB-TEXT-HOLDS-DOF]: the circle labels carry their own position, so this sketch
        # never reports fully constrained even though its geometry is fully determined.
        futil.log(
            'Rotor Lobe {} isFullyConstrained: {}'.format(d + 1, sketch.isFullyConstrained))

    # -- S08/S09/S10/S11 -------------------------------------------------------------------
    def buildDisk(self, d):
        component = self.getComponent()
        L = int(round(self.getParameter(PARAM_LOBES).value))
        spline = self.lobeSplines[d]
        sketch = spline.parentSketch

        base = component.bRepBodies.count

        sectorProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if any(pc.sketchEntity == spline for pc in loop.profileCurves):
                    sectorProfile = profile
                    break
            if sectorProfile is not None:
                break
        if sectorProfile is None:
            raise Exception(
                'Rotor Lobe {}: could not find the lobe sector profile'.format(d + 1))

        ext = component.features.extrudeFeatures.createInput(
            sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = component.features.extrudeFeatures.add(ext)
        extrude.bodies.item(0).name = 'Cycloidal Disk {}'.format(d + 1)

        # S09 — construction axis through the disc centre, from the cap face's normal.
        capFace = extrude.startFaces.item(0)
        self.buildDiskAxis(capFace, d)

        # S10 — circular-pattern the lobe sector L times.
        coll = adsk.core.ObjectCollection.create()
        coll.add(extrude)
        pat = component.features.circularPatternFeatures.createInput(coll, self.diskAxes[d])
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(L)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

        # S11 — join disc d's own L sectors into one body.
        target = component.bRepBodies.item(base)
        tools = adsk.core.ObjectCollection.create()
        for i in range(base + 1, base + L):
            tools.add(component.bRepBodies.item(i))
        ci = component.features.combineFeatures.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(ci)

        target.name = 'Cycloidal Disk {}'.format(d + 1)
        self.diskBodies[d] = target

    # -- S09 ---------------------------------------------------------------------------
    def buildDiskAxis(self, capFace: adsk.fusion.BRepFace, d):
        component = self.getComponent()
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
        axis = component.constructionAxes.add(axInput)
        axis.name = 'Disk Axis {}'.format(d + 1)
        self.diskAxes[d] = axis

    # -- S12 -----------------------------------------------------------------------------
    def buildOutputHoleSketch(self, d):
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Output Hole {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        E = self.getParameter(PARAM_ECCENTRICITY).value
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_RADIUS).value
        Rr = self.Rr
        DHole = self.D_hole

        localOrigin = self._anchorSketch(sketch)
        s, diskCentre = self._buildDiskCentre(sketch, localOrigin, d, E)

        outputHoleCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E, 0, 0), Rop)
        outputHoleCircle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            outputHoleCircle.centerSketchPoint, diskCentre)
        circleDim = sketch.sketchDimensions.addDiameterDimension(
            outputHoleCircle, adsk.core.Point3D.create(s * E + Rop, 0.2, 0))
        circleDim.parameter.expression = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)
        self._addCircleLabel(sketch, outputHoleCircle, 'Output Hole Circle', Rr)

        hole = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E + Rop, 0, 0), DHole / 2)
        holeDim = sketch.sketchDimensions.addDiameterDimension(
            hole, adsk.core.Point3D.create(s * E + Rop + DHole / 2, 0.2, 0))
        holeDim.parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)
        sketch.geometricConstraints.addCoincident(hole.centerSketchPoint, outputHoleCircle)
        spokeLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            diskCentre, hole.centerSketchPoint)
        sketch.geometricConstraints.addHorizontal(spokeLine)

        self.outputHoles[d] = hole

    # -- S13/S14 ---------------------------------------------------------------------------
    def buildOutputHoles(self, d):
        component = self.getComponent()
        M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        holeCircle = self.outputHoles[d]
        sketch = holeCircle.parentSketch

        holeProfile = None
        for profile in sketch.profiles:
            for loop in profile.profileLoops:
                if any(pc.sketchEntity == holeCircle for pc in loop.profileCurves):
                    holeProfile = profile
                    break
            if holeProfile is not None:
                break
        if holeProfile is None:
            raise Exception('Output Hole {}: could not find the hole profile'.format(d + 1))

        ci = component.features.extrudeFeatures.createInput(
            holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        cut = component.features.extrudeFeatures.add(ci)

        coll = adsk.core.ObjectCollection.create()
        coll.add(cut)
        pat = component.features.circularPatternFeatures.createInput(coll, self.diskAxes[d])
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

    # -- S15/S16 ---------------------------------------------------------------------------
    def buildDiskBore(self, d):
        component = self.getComponent()
        plane = self.discPlanes[d]
        sketch = self.createSketchObject('Disc Bore {}'.format(d + 1), plane=plane)
        sketch.isVisible = True

        E = self.getParameter(PARAM_ECCENTRICITY).value
        CBD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        clr = self.getParameter(PARAM_BEARING_CLEARANCE).value

        localOrigin = self._anchorSketch(sketch)
        s, diskCentre = self._buildDiskCentre(sketch, localOrigin, d, E)

        boreRadius = (CBD + clr) / 2
        bore = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(s * E, 0, 0), boreRadius)
        sketch.geometricConstraints.addCoincident(bore.centerSketchPoint, diskCentre)
        boreDim = sketch.sketchDimensions.addDiameterDimension(
            bore, adsk.core.Point3D.create(s * E + boreRadius, 0.2, 0))
        boreDim.parameter.expression = '{} + {}'.format(
            self.parameterName(PARAM_CENTER_BEARING_DIAMETER),
            self.parameterName(PARAM_BEARING_CLEARANCE))

        coll = adsk.core.ObjectCollection.create()
        for profile in sketch.profiles:
            coll.add(profile)
        ci = component.features.extrudeFeatures.createInput(
            coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_DISC_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ci.participantBodies = [self.diskBodies[d]]
        component.features.extrudeFeatures.add(ci)

    # -- S17/S18/S19 -------------------------------------------------------------------
    def buildCam(self):
        component = self.getComponent()
        E = self.getParameter(PARAM_ECCENTRICITY).value
        CBD = self.getParameter(PARAM_CENTER_BEARING_DIAMETER).value
        ISD = self.getParameter(PARAM_INPUT_SHAFT_DIAMETER).value
        nT = self.parameterName(PARAM_DISC_THICKNESS)
        nG = self.parameterName(PARAM_DISC_GAP)
        discCount = len(self.diskBodies)
        sectionBodies = []

        for d in range(discCount):
            plane = self.discPlanes[d]
            sketch = self.createSketchObject('Eccentric Cam {}'.format(d + 1), plane=plane)
            sketch.isVisible = True

            localOrigin = self._anchorSketch(sketch)
            s, diskCentre = self._buildDiskCentre(sketch, localOrigin, d, E)

            outerCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(s * E, 0, 0), CBD / 2)
            sketch.geometricConstraints.addCoincident(outerCircle.centerSketchPoint, diskCentre)
            outerDim = sketch.sketchDimensions.addDiameterDimension(
                outerCircle, adsk.core.Point3D.create(s * E + CBD / 2, 0.2, 0))
            outerDim.parameter.expression = self.parameterName(PARAM_CENTER_BEARING_DIAMETER)

            hasBore = ISD > 0
            if hasBore:
                boreCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
                    adsk.core.Point3D.create(0, 0, 0), ISD / 2)
                sketch.geometricConstraints.addCoincident(
                    boreCircle.centerSketchPoint, localOrigin)
                boreDim = sketch.sketchDimensions.addDiameterDimension(
                    boreCircle, adsk.core.Point3D.create(ISD / 2, 0.2, 0))
                boreDim.parameter.expression = self.parameterName(PARAM_INPUT_SHAFT_DIAMETER)

            # Select the cross-section by loop count, never find_profile_by_curve_counts.
            if hasBore:
                sectionProfile = None
                for profile in sketch.profiles:
                    if profile.profileLoops.count == 2:
                        sectionProfile = profile
                        break
                if sectionProfile is None:
                    raise Exception(
                        'Eccentric Cam {}: could not find the two-loop annulus'.format(d + 1))
            else:
                sectionProfile = sketch.profiles.item(0)

            isLast = d == discCount - 1
            extentExpr = nT if isLast else '{} + {}'.format(nT, nG)

            ext = component.features.extrudeFeatures.createInput(
                sectionProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            ext.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(
                    adsk.core.ValueInput.createByString(extentExpr)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection)
            extrude = component.features.extrudeFeatures.add(ext)
            body = extrude.bodies.item(0)
            body.name = 'Eccentric Cam {}'.format(d + 1)
            sectionBodies.append(body)

        # S19 — join the cam sections (or, for a single disc, just rename the one body).
        if discCount == 1:
            self.cam = sectionBodies[0]
            self.cam.name = 'Eccentric Cam'
        else:
            target = sectionBodies[0]
            tools = adsk.core.ObjectCollection.create()
            for body in sectionBodies[1:]:
                tools.add(body)
            ci = component.features.combineFeatures.createInput(target, tools)
            ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            component.features.combineFeatures.add(ci)
            target.name = 'Eccentric Cam'
            self.cam = target

    # -- S20 through S28 -----------------------------------------------------------------
    def buildRingPins(self):
        component = self.getComponent()
        R = self.getParameter(PARAM_PIN_CIRCLE_RADIUS).value
        E = self.getParameter(PARAM_ECCENTRICITY).value
        Rr = self.Rr
        Wall = self.getParameter(PARAM_WALL).value

        # S20 — construction plane for the housing base, 1 mm below the disc.
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))
        housingPlane = component.constructionPlanes.add(planeInput)
        housingPlane.name = 'Ring Housing Plane'

        # S21 — Housing Ring sketch: a plain pinless annulus.
        ringSketch = self.createSketchObject('Housing Ring', plane=housingPlane)
        ringSketch.isVisible = True

        ringLocalOrigin = self._anchorSketch(ringSketch)

        outerR = R - Rr + 2 * E + Wall
        outerCircle = ringSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerR)
        ringSketch.geometricConstraints.addCoincident(
            outerCircle.centerSketchPoint, ringLocalOrigin)
        outerDim = ringSketch.sketchDimensions.addDiameterDimension(
            outerCircle, adsk.core.Point3D.create(outerR, 0.2, 0))
        outerDim.parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)

        innerR = R - Rr - Wall
        innerCircle = ringSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), innerR)
        ringSketch.geometricConstraints.addCoincident(
            innerCircle.centerSketchPoint, ringLocalOrigin)
        innerDim = ringSketch.sketchDimensions.addDiameterDimension(
            innerCircle, adsk.core.Point3D.create(innerR, 0.2, 0))
        innerDim.parameter.expression = self.parameterName(PARAM_HOUSING_INNER_DIAMETER)

        # S22 — extrude the annulus, away from the disc.
        annulusProfile = None
        for profile in ringSketch.profiles:
            if profile.profileLoops.count == 2:
                annulusProfile = profile
                break
        if annulusProfile is None:
            raise Exception('Housing Ring: could not find the annulus profile')

        ext = component.features.extrudeFeatures.createInput(
            annulusProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_BASE_THICKNESS))),
            adsk.fusion.ExtentDirections.NegativeExtentDirection)
        housingExtrude = component.features.extrudeFeatures.add(ext)
        self.housingRing = housingExtrude.bodies.item(0)
        self.housingRing.name = 'Housing Ring'

        # S23 — construction axis on the drive axis, through the local origin.
        capFace = housingExtrude.startFaces.item(0)
        axInput = component.constructionAxes.createInput()
        axInput.setByPerpendicularAtPoint(capFace, ringLocalOrigin)
        self.driveAxis = component.constructionAxes.add(axInput)
        self.driveAxis.name = 'Drive Axis'

        # S24 — Ring Casing section sketch: compute the swept-envelope contour first.
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        L = int(round(self.getParameter(PARAM_LOBES).value))
        c = self.getParameter(PARAM_DISK_CLEARANCE).value
        RrEff = Rr + c
        half = math.pi / N
        nbins = 80
        sweepSteps = 240
        binMax = [0.0] * nbins
        hit = [False] * nbins
        for i in range(sweepSteps):
            theta = 2 * math.pi * i / sweepSteps
            cx, cy = E * math.cos(theta), E * math.sin(theta)
            phi = -theta / L
            for j in range(sweepSteps):
                t = 2 * math.pi * j / sweepSteps
                x, y = disk_point(t, cx, cy, phi, R, E, N, RrEff)
                a = math.atan2(y, x)
                if a < -half or a > half:
                    continue
                b = int((a + half) / (2 * half) * nbins)
                b = min(max(b, 0), nbins - 1)
                r = math.hypot(x, y)
                if r > binMax[b]:
                    binMax[b] = r
                hit[b] = True

        contourPoints = []
        for i in range(nbins + 1):
            phi_i = -half + 2 * half * i / nbins
            peak = 0.0
            if i - 1 >= 0 and hit[i - 1]:
                peak = max(peak, binMax[i - 1])
            if i < nbins and hit[i]:
                peak = max(peak, binMax[i])
            r = c + peak
            contourPoints.append((r * math.cos(phi_i), r * math.sin(phi_i)))

        casingSketch = self.createSketchObject('Ring Casing', plane=self.plane)
        casingSketch.isVisible = True

        casingOrigin = self._anchorSketch(casingSketch)

        outerRadius = R - Rr + 2 * E + Wall
        casingOuterCircle = casingSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), outerRadius)
        casingSketch.geometricConstraints.addCoincident(
            casingOuterCircle.centerSketchPoint, casingOrigin)
        casingOuterDim = casingSketch.sketchDimensions.addDiameterDimension(
            casingOuterCircle, adsk.core.Point3D.create(outerRadius, 0.2, 0))
        casingOuterDim.parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)

        coll = adsk.core.ObjectCollection.create()
        for x, y in contourPoints:
            coll.add(adsk.core.Point3D.create(x, y, 0))
        contourSpline = casingSketch.sketchCurves.sketchFittedSplines.add(coll)

        outerFirst = adsk.core.Point3D.create(
            outerRadius * math.cos(-half), outerRadius * math.sin(-half), 0)
        outerLast = adsk.core.Point3D.create(
            outerRadius * math.cos(half), outerRadius * math.sin(half), 0)
        casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            contourSpline.fitPoints.item(0), outerFirst)
        casingSketch.sketchCurves.sketchLines.addByTwoPoints(
            contourSpline.fitPoints.item(contourSpline.fitPoints.count - 1), outerLast)

        # S25 — extrude the casing sector, two-sided, selecting the smallest-area profile
        # among those whose loop contains the contour spline.
        candidates = []
        for profile in casingSketch.profiles:
            for loop in profile.profileLoops:
                if any(pc.sketchEntity == contourSpline for pc in loop.profileCurves):
                    candidates.append(profile)
                    break
        if not candidates:
            raise Exception('Ring Casing: could not find a profile containing the contour spline')
        sectorProfile = None
        sectorArea = None
        for candidate in candidates:
            candidate: adsk.fusion.Profile
            area = candidate.areaProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area
            if sectorArea is None or area < sectorArea:
                sectorArea = area
                sectorProfile = candidate
        if sectorProfile is None:
            raise Exception('Ring Casing: could not select the smallest-area sector profile')

        stackTopExpr = self._stackTopExpr()

        base = component.bRepBodies.count

        ext = component.features.extrudeFeatures.createInput(
            sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString(stackTopExpr)),
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString('1 mm')))
        sectorFeature = component.features.extrudeFeatures.add(ext)

        # S26 — circular-pattern the casing sector N times about the drive axis.
        coll2 = adsk.core.ObjectCollection.create()
        coll2.add(sectorFeature)
        pat = component.features.circularPatternFeatures.createInput(coll2, self.driveAxis)
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(N)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

        # S27 — join the N casing sectors into one casing.
        target = component.bRepBodies.item(base)
        tools = adsk.core.ObjectCollection.create()
        for i in range(base + 1, base + N):
            tools.add(component.bRepBodies.item(i))
        ci = component.features.combineFeatures.createInput(target, tools)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(ci)
        self.ringCasing = target

        # S28 — combine the casing into the Housing.
        tools2 = adsk.core.ObjectCollection.create()
        tools2.add(self.ringCasing)
        ci2 = component.features.combineFeatures.createInput(self.housingRing, tools2)
        ci2.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        component.features.combineFeatures.add(ci2)
        self.housingRing.name = 'Housing'
        self.ringCasing = None

    # -- S29 through S35 -----------------------------------------------------------------
    def buildOutputPins(self):
        component = self.getComponent()
        E = self.getParameter(PARAM_ECCENTRICITY).value
        Rop = self.getParameter(PARAM_OUTPUT_PIN_CIRCLE_RADIUS).value
        Wall = self.getParameter(PARAM_WALL).value
        DHole = self.D_hole
        DPin = DHole - 2 * E
        stackTopExpr = self._stackTopExpr()

        # S29 — construction plane for the output plate, 1 mm above the top disc.
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(
            self.plane, adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))
        platePlane = component.constructionPlanes.add(planeInput)
        platePlane.name = 'Output Plate Plane'

        # S30 — Output Plate sketch, everything on the drive axis O.
        plateSketch = self.createSketchObject('Output Plate', plane=platePlane)
        plateSketch.isVisible = True

        localOrigin = self._anchorSketch(plateSketch)

        plateRadius = Rop + DPin / 2 + Wall
        plateOuterCircle = plateSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), plateRadius)
        plateSketch.geometricConstraints.addCoincident(
            plateOuterCircle.centerSketchPoint, localOrigin)
        plateDim = plateSketch.sketchDimensions.addDiameterDimension(
            plateOuterCircle, adsk.core.Point3D.create(plateRadius, 0.2, 0))
        plateDim.parameter.expression = self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)

        outPinCircle = plateSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), Rop)
        outPinCircle.isConstruction = True
        plateSketch.geometricConstraints.addCoincident(
            outPinCircle.centerSketchPoint, localOrigin)
        outPinDim = plateSketch.sketchDimensions.addDiameterDimension(
            outPinCircle, adsk.core.Point3D.create(Rop, 0.2, 0))
        outPinDim.parameter.expression = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)

        pinRadius = (DHole - 2 * E) / 2
        pin = plateSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(Rop, 0, 0), pinRadius)
        pinDim = plateSketch.sketchDimensions.addDiameterDimension(
            pin, adsk.core.Point3D.create(Rop + pinRadius, 0.2, 0))
        pinDim.parameter.expression = '{} - 2 * {}'.format(
            self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER), self.parameterName(PARAM_ECCENTRICITY))
        plateSketch.geometricConstraints.addCoincident(pin.centerSketchPoint, outPinCircle)
        spokeLine = plateSketch.sketchCurves.sketchLines.addByTwoPoints(
            localOrigin, pin.centerSketchPoint)
        plateSketch.geometricConstraints.addHorizontal(spokeLine)

        # S31 — extrude the full plate disc, away from the disk.
        coll = adsk.core.ObjectCollection.create()
        for profile in plateSketch.profiles:
            coll.add(profile)
        ext = component.features.extrudeFeatures.createInput(
            coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        plateExtrude = component.features.extrudeFeatures.add(ext)
        self.outputPlate = plateExtrude.bodies.item(0)
        self.outputPlate.name = 'Output Plate'

        # S32 — extrude the output pin, two-sided.
        pinProfile = None
        for profile in plateSketch.profiles:
            if profile.profileLoops.count == 1:
                loop = profile.profileLoops.item(0)
                if any(pc.sketchEntity == pin for pc in loop.profileCurves):
                    pinProfile = profile
                    break
        if pinProfile is None:
            raise Exception('Output Plate: could not find the pin disc profile')

        pinBase = component.bRepBodies.count
        pinExt = component.features.extrudeFeatures.createInput(
            pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        pinExt.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
                stackTopExpr + ' + 1 mm')))
        pinFeature = component.features.extrudeFeatures.add(pinExt)
        pinBody = pinFeature.bodies.item(0)
        pinBody.name = 'Output Pin'

        # S33 — cut the plate socket, keeping the pin.
        tools = adsk.core.ObjectCollection.create()
        tools.add(pinBody)
        ci = component.features.combineFeatures.createInput(self.outputPlate, tools)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = True
        combineFeature = component.features.combineFeatures.add(ci)

        # S34 — chamfer the output pin's ends.
        chamferFeature = self._chamferCapRims(pinBody)

        # S35 — pattern the pin, socket and chamfer M times.
        M = int(round(self.getParameter(PARAM_OUTPUT_PIN_COUNT).value))
        coll2 = adsk.core.ObjectCollection.create()
        coll2.add(pinFeature)
        coll2.add(combineFeature)
        if chamferFeature is not None:
            coll2.add(chamferFeature)
        pat = component.features.circularPatternFeatures.createInput(coll2, self.driveAxis)
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pat.quantity = adsk.core.ValueInput.createByReal(M)
        pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        pat.isSymmetric = False
        component.features.circularPatternFeatures.add(pat)

        for k in range(M):
            component.bRepBodies.item(pinBase + k).name = 'Output Pin {}'.format(k + 1)

    # -- S34 contract, also used by S36 ---------------------------------------------------
    def _chamferCapRims(self, body: adsk.fusion.BRepBody):
        if self.chamferSize <= 0:
            return None

        axis: adsk.core.Vector3D = self.plane.geometry.normal
        ref: adsk.core.Point3D = self.plane.geometry.origin

        faceHeights = []
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            n: adsk.core.Vector3D = face.geometry.normal
            if abs(n.dotProduct(axis)) <= 0.999:
                continue
            h = ref.vectorTo(face.geometry.origin).dotProduct(axis)
            faceHeights.append((face, h))

        if not faceHeights:
            return None

        hmin = min(h for _, h in faceHeights)
        hmax = max(h for _, h in faceHeights)

        edges = adsk.core.ObjectCollection.create()
        for face, h in faceHeights:
            if not (abs(h - hmin) < 1e-4 or abs(h - hmax) < 1e-4):
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
            edges, adsk.core.ValueInput.createByString(self.parameterName(PARAM_CHAMFER_SIZE)),
            False)
        try:
            return chamfers.add(ci)
        except RuntimeError as e:
            futil.log('Chamfer failed on body "{}": {}'.format(body.name, e))
            self.chamfersSkipped += 1
            return None

    # -- S36 -------------------------------------------------------------------------------
    def buildChamfers(self):
        if self.chamferSize <= 0:
            return

        targets = list(self.diskBodies) + [self.housingRing, self.ringCasing, self.outputPlate]
        for body in targets:
            if body is None:
                continue
            self._chamferCapRims(body)

    # -- S37 -------------------------------------------------------------------------------
    def buildSubComponents(self):
        component = self.getComponent()

        groups = {'Rotor Discs': [], 'Housing': [], 'Eccentric Cam': [], 'Output': []}
        for i in range(component.bRepBodies.count):
            body = component.bRepBodies.item(i)
            name = body.name
            if name.startswith('Cycloidal Disk'):
                groups['Rotor Discs'].append(body)
            elif name == 'Housing':
                groups['Housing'].append(body)
            elif name == 'Eccentric Cam':
                groups['Eccentric Cam'].append(body)
            elif name == 'Output Plate' or name.startswith('Output Pin'):
                groups['Output'].append(body)

        for groupName in ('Rotor Discs', 'Housing', 'Eccentric Cam', 'Output'):
            bodies = groups[groupName]
            if not bodies:
                continue
            occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = groupName
            for body in bodies:
                body: adsk.fusion.BRepBody
                body.moveToComponent(occ)

        solids.hide_construction_geometry(component)
