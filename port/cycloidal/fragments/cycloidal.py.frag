import collections
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator, get_value, get_selection
from . import decadfusion as _fx
from . import solids


# -----------------------------------------------------------------------------------------
# Dialog input ids and the gallery's parameter table, as Fusion user parameters.
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


# @@constants@@

# The translated code runs in millimetres, as the gallery does; the runtime converts to Fusion's centimetres.
MILLIMETRE = 0.1


# -----------------------------------------------------------------------------------------
# The dialog.
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

        # @@inputs@@
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


# @@params_class@@

# @@gallery_parameters@@

# @@translated@@

def _begin(doc, p):
    """The run the gallery's Build starts from: the design, the Params and the dimensions they resolve to."""
    return _Run(doc=doc, world=doc, p=p, d=_resolve(p), frame=_placement(p))


def _params_of_inputs(inputs: adsk.core.CommandInputs):
    # @@params_from_inputs@@


# -----------------------------------------------------------------------------------------
# The orchestrator: the Fusion wiring around the translated build.
# -----------------------------------------------------------------------------------------
class CycloidalDriveGenerator(Generator):
    DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + '__status'

    # The groups the drive's bodies are moved into, as sub-components. Fusion-only organisation.
    COMPONENT_GROUPS = (
        ('Rotor Discs', lambda name: name.startswith('Cycloidal Disk')),
        ('Housing', lambda name: name == 'Housing'),
        ('Eccentric Cam', lambda name: name == 'Eccentric Cam'),
        ('Output', lambda name: name == 'Output Plate' or name.startswith('Output Pin')),
    )

    def prefixBase(self) -> str:
        return 'CycloidalDrive'

    def generateName(self) -> str:
        N = int(round(self.getParameter(PARAM_PIN_COUNT).value))
        L = N - 1
        return 'Cycloidal Drive (N={}):{}'.format(N, L)

    @staticmethod
    def validate_inputs(inputs: adsk.core.CommandInputs) -> list:
        return problems(_params_of_inputs(inputs))

    def _galleryParams(self):
        # @@params_from_parameters@@

    def processInputs(self, inputs: adsk.core.CommandInputs):
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self._runtime = None

        # Selections first ([PB-SELECTION-STASH]).
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

        # The dialog's values, registered as user parameters ([PB-GET-VALUE-CONTRACT]).
        # @@primary_parameters@@

        # The gallery's validity checks, then the dimensions the rest of the table needs.
        params = self._galleryParams()
        found = problems(params)
        if found:
            raise Exception('\n'.join(found))
        dims = _resolve(params)
        # @@derived_parameters@@

    def galleryParameter(self, name):
        """The Fusion name and user parameter of a gallery parameter table entry."""
        constant = _GALLERY_PARAMETERS[name]
        return self.parameterName(constant), self.getParameter(constant)

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalise the Target Plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self.plane.name = 'Drive Base Plane'

        toolsSketch = self._anchor(component)
        runtime = _fx.Runtime(component, self.plane, self._projectedAnchor, self.galleryParameter, False, MILLIMETRE)
        runtime.activate()
        self._runtime = runtime
        runtime.build_frame(toolsSketch)
        with _fx.timed('Cycloidal build'):
            drive = build(runtime, self._galleryParams())
        for named in drive_named(drive):
            named.Body.body.name = named.Name
        with _fx.timed('Cycloidal sub-components'):
            self._buildSubComponents(component)
        with _fx.timed('Cycloidal hide helpers'):
            runtime.hide_helpers()
            toolsSketch.isVisible = False
            solids.hide_construction_geometry(component)

    def _anchor(self, component):
        """A sketch on the plane that holds the Anchor Point, fully constrained, for every translated sketch to
        project. [CYCLOIDAL-F-ANCHOR-CHAIN]"""
        sketch: adsk.fusion.Sketch = component.sketches.add(self.plane)
        sketch.name = 'Tools'
        self._projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(self.anchorPoint).item(0))
        if self._projectedAnchor is None:
            raise RuntimeError('Tools: anchor projection produced no SketchPoint')
        if not sketch.isFullyConstrained:
            raise RuntimeError(f'{sketch.name}: sketch is not fully constrained')
        return sketch

    def _buildSubComponents(self, component: adsk.fusion.Component):
        """Move the drive's bodies into one sub-component per group, last, because moveToComponent invalidates the
        moved body's reference."""
        groups = {name: [] for name, _ in self.COMPONENT_GROUPS}
        for i in range(component.bRepBodies.count):
            body = component.bRepBodies.item(i)
            for name, belongs in self.COMPONENT_GROUPS:
                if belongs(body.name):
                    groups[name].append(body)
                    break
        for name, _ in self.COMPONENT_GROUPS:
            bodies = groups[name]
            if not bodies:
                continue
            occurrence = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occurrence.component.name = name
            with _fx.timed(f'Move {len(bodies)} bodies into {name}'):
                for body in bodies:
                    body.moveToComponent(occurrence)
