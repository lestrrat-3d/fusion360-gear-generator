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
INPUT_ID_TOOTH_TYPE = 'toothType'
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
INPUT_ID_TOE_EXTENSION = 'toeExtension'
INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'
INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'

_TOOTH_STRAIGHT = 'Straight'
_TOOTH_SPIRAL = 'Spiral'
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

    # The spiral-only inputs show when the Tooth Type is Spiral and hide when it is Straight.
    SPIRAL_ONLY = (INPUT_ID_SPIRAL_ANGLE, INPUT_ID_HAND, INPUT_ID_CUTTER_RADIUS)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        visible = _is_spiral(inputs)
        for inputId in cls.SPIRAL_ONLY:
            item = inputs.itemById(inputId)
            if item is not None:
                item.isVisible = visible

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


def _is_spiral(inputs: adsk.core.CommandInputs) -> bool:
    """True when the Tooth Type is Spiral."""
    toothType = inputs.itemById(INPUT_ID_TOOTH_TYPE)
    return toothType is not None and toothType.selectedItem is not None and toothType.selectedItem.name == _TOOTH_SPIRAL


# @@params_class@@

# @@translated@@

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
        if _is_spiral(inputs) and params.SpiralAngle <= 0:
            raise RuntimeError('Mean Spiral Angle must be above 0 for spiral teeth')

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
        # The driving gear stands on its back face, so the pair's apex is the blank's heel station above the Center
        # Point, and the driving shaft runs from the apex down into the Target Plane.
        height = _blank_section_of(_prepare_build(params).geometry)[1].station
        pairPlane, pairTools, apex, shaft, across = self._pairPlane(component, toolsSketch, height)
        # Every sketch projects the apex, a point of the Pair Tools sketch that is fully constrained to the Center Point.
        runtime = _fx.Runtime(component, pairPlane, apex, _no_fusion_parameters, False, MILLIMETRE)
        runtime.activate()
        runtime.build_frame(pairTools, shaft, across)
        pair = build_pair(runtime, params)
        pair.Driving.body.name = 'Driving Gear'
        pair.Pinion.body.name = 'Pinion Gear'
        runtime.finish_containers()
        runtime.hide_helpers()
        toolsSketch.isVisible = False
        pairTools.isVisible = False
        pairPlane.isLightBulbOn = False
        solids.hide_construction_geometry(component)

    def _anchor(self, component):
        """A sketch on the Target Plane that holds the Center Point and a short line through it along the sketch's X
        axis, fully constrained. Every translated sketch projects the point, and the Pair Plane stands on the line.
        [CYCLOIDAL-F-ANCHOR-CHAIN] [PB-PAIR-PLANE]"""
        sketch: adsk.fusion.Sketch = component.sketches.add(self.plane)
        sketch.name = 'Tools'
        projected = adsk.fusion.SketchPoint.cast(sketch.project(self.anchorPoint).item(0))
        if projected is None:
            raise RuntimeError('Tools: anchor projection produced no SketchPoint')
        cx, cy = projected.geometry.x, projected.geometry.y
        line: adsk.fusion.SketchLine = sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(cx - 0.5, cy, 0.0), adsk.core.Point3D.create(cx + 0.5, cy, 0.0))
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        constraints.addCoincident(projected, line)
        constraints.addMidPoint(projected, line)
        sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cx, cy + 0.3, 0.0))
        constraints.addHorizontal(line)
        if not sketch.isFullyConstrained:
            raise RuntimeError(f'{sketch.name}: sketch is not fully constrained')
        self._projectedAnchor = projected
        self._anchorLine = line
        return sketch

    def _pairPlane(self, component, toolsSketch, height):
        """The plane the pair is drawn in: across the Target Plane, through the Center Point and along the sketch's X
        axis. The driving gear stands on the Target Plane on its back face with its axis through the Center Point, so
        the pair's apex is `height` (mm) above the point on the side the Target Plane's normal points to, and the
        driving shaft is the normal turned round. The pinion lies toward the greater X of the plane's sketch. Returns
        the plane, a sketch on it, the apex in that sketch, and the gallery's U and V in the design. This is the
        placement spec/bevelgear/instructions.md gave the previous add-in: the apex at the Center Point plus the pitch
        cone's along-shaft length and the base height, on the normal's side. [PB-PAIR-PLANE]"""
        x = _fx.vec_of(toolsSketch.xDirection)
        y = _fx.vec_of(toolsSketch.yDirection)
        normal = x.cross(y).unit()
        down = normal.scale(-1.0)
        planes: adsk.fusion.ConstructionPlanes = component.constructionPlanes
        for angle in ('90 deg', '-90 deg'):
            planeInput = planes.createInput()
            planeInput.setByAngle(self._anchorLine, adsk.core.ValueInput.createByString(angle), self.plane)
            plane = planes.add(planeInput)
            sketch: adsk.fusion.Sketch = component.sketches.add(plane)
            sx = _fx.vec_of(sketch.xDirection)
            sy = _fx.vec_of(sketch.yDirection)
            sketchNormal = sx.cross(sy).unit()
            # V completes a right-handed pair about the sketch's normal. The plane is kept when the pinion, which turns
            # toward V, lies toward the greater X of the plane's sketch; the other sense of the plane puts it opposite.
            across = sketchNormal.cross(down).unit()
            wanted = sx.sub(down.scale(sx.dot(down)))
            if wanted.len() < 1e-6:
                wanted = x
            if across.dot(wanted) > 0 or angle == '-90 deg':
                break
            sketch.deleteMe()
            plane.deleteMe()
        plane.name = 'Pair Plane'
        sketch.name = 'Pair Tools'
        center = adsk.fusion.SketchPoint.cast(sketch.project(self.anchorPoint).item(0))
        if center is None:
            raise RuntimeError('Pair Tools: the Center Point projected to no SketchPoint')
        lift = to_cm(height)
        apex = sketch.sketchPoints.add(adsk.core.Point3D.create(
            center.geometry.x + normal.dot(sx) * lift, center.geometry.y + normal.dot(sy) * lift, 0.0))
        constraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        for orientation, align in (
                (adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, constraints.addVerticalPoints),
                (adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, constraints.addHorizontalPoints)):
            try:
                sketch.sketchDimensions.addDistanceDimension(center, apex, orientation, apex.geometry)
            except RuntimeError:
                # The apex lies straight above or beside the point, and Fusion refuses a dimension of zero.
                align(center, apex)
        return plane, sketch, apex, down, across
