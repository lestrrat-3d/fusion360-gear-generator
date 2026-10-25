import collections
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import Generator, get_selection
from . import decadfusion as _fx
from . import spurgear as _spur
from . import solids


# -----------------------------------------------------------------------------------------
# Dialog input ids.
# -----------------------------------------------------------------------------------------
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER_POINT = 'centerPoint'
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_MODULE = 'module'
INPUT_ID_SHAFT_ANGLE = 'shaftAngle'
INPUT_ID_DRIVING_TEETH = 'drivingTeeth'
INPUT_ID_PINION_TEETH = 'pinionTeeth'
INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'
INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'
INPUT_ID_BORE_ENABLE = 'boreEnable'
INPUT_ID_DRIVING_BORE = 'drivingBore'
INPUT_ID_PINION_BORE = 'pinionBore'
INPUT_ID_FACE_WIDTH = 'faceWidth'
INPUT_ID_TOOTH_SPACING = 'toothSpacing'
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
INPUT_ID_TOE_EXTENSION = 'toeExtension'
INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'
INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# The translated code runs in millimetres, as the gallery does; the runtime converts to Fusion's centimetres.
MILLIMETRE = 0.1


# -----------------------------------------------------------------------------------------
# The dialog.
# -----------------------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    # S1: dialog inputs, in the contract row order.
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # @@inputs@@
        cls._updateSpiralInputVisibility(inputs)

    # S1: spiral-only inputs (Hand of Spiral, Cutter Radius) show only when
    # Mean Spiral Angle is above zero.
    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return
        try:
            design: adsk.fusion.Design = get_design()
            value = design.unitsManager.evaluateExpression(spiral.expression, 'rad')
            visible = value > 0
        except Exception:
            visible = True
        hand.isVisible = visible
        cutter.isVisible = visible

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


# @@params_class@@

# @@translated@@

def _build_spiral_member(doc, p, geometry, pinion):
    raise NotImplementedError('the spiral bevel member is not translated yet: set Mean Spiral Angle to 0')


def _build_spiral_pair(doc, p, pinionParams, drivingPlan, pinionPlan):
    raise NotImplementedError('the spiral bevel pair is not translated yet: set Mean Spiral Angle to 0')


def _validate_spiral_build(p):
    raise NotImplementedError('the spiral bevel pair is not translated yet: set Mean Spiral Angle to 0')


def _params_of_inputs(inputs: adsk.core.CommandInputs):
    # @@params_from_inputs@@


def _no_fusion_parameters(name):
    raise RuntimeError(f'{name} is not a Fusion parameter: the bevel pair keeps its dimensions in the runtime')


# -----------------------------------------------------------------------------------------
# The orchestrator: the Fusion wiring around the translated build.
# -----------------------------------------------------------------------------------------
class BevelGearGenerator(Generator):
    def prefixBase(self) -> str:
        return 'BevelGear'

    def generateName(self, params) -> str:
        return 'Bevel Gear Pair ({}:{})'.format(params.Teeth, params.MateTeeth)

    def generate(self, inputs: adsk.core.CommandInputs):
        params = _params_of_inputs(inputs)

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
        self.anchorPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]

        component = self.getComponent()
        component.name = self.generateName(params)

        # Normalise the Target Plane to a ConstructionPlane.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self.plane.name = 'Pair Base Plane'

        toolsSketch = self._anchor(component)
        runtime = _fx.Runtime(component, self.plane, self._projectedAnchor, _no_fusion_parameters, False, MILLIMETRE)
        runtime.activate()
        runtime.build_frame(toolsSketch)
        pair = build_pair(runtime, params)
        pair.Driving.body.name = 'Driving Gear'
        pair.Pinion.body.name = 'Pinion Gear'
        runtime.finish_containers()
        runtime.hide_helpers()
        toolsSketch.isVisible = False
        solids.hide_construction_geometry(component)

    def _anchor(self, component):
        """A sketch on the plane that holds the Center Point, fully constrained, for every translated sketch to
        project. [CYCLOIDAL-F-ANCHOR-CHAIN]"""
        sketch: adsk.fusion.Sketch = component.sketches.add(self.plane)
        sketch.name = 'Tools'
        self._projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(self.anchorPoint).item(0))
        if self._projectedAnchor is None:
            raise RuntimeError('Tools: anchor projection produced no SketchPoint')
        if not sketch.isFullyConstrained:
            raise RuntimeError(f'{sketch.name}: sketch is not fully constrained')
        return sketch
