import collections
import math
from typing import cast
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .misc import to_cm, get_design
from . import solids
from . import decadfusion as _fx


INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'
INPUT_ID_SKETCH_ONLY = 'sketchOnly'
PARAM_SKETCH_ONLY = 'SketchOnly'
# The centimetres of one millimetre, the unit the gallery's checks print lengths in. [PB-UNITS]
_MILLIMETRE = 0.1
# @@constants@@

class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs: adsk.core.CommandInputs = cmd.commandInputs
        selection0: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the gear on')
        selection0.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        selection0.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        selection0.setSelectionLimits(1, 1)
        selection1: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point the gear is centered on')
        selection1.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        selection1.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        selection1.setSelectionLimits(1, 1)
        # @@inputs@@
        inputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True, '', False)
        selection9: adsk.core.SelectionCommandInput = inputs.addSelectionInput(INPUT_ID_PARENT, 'Parent Component', 'Select the component to build the gear in')
        selection9.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        selection9.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        selection9.setSelectionLimits(1, 1)
        selection9.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    def __init__(self):
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super().__init__(design)
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self._createdTargetPlane = False
        self._toolsSketch = adsk.fusion.Sketch.cast(None)
        self._referenceSketch = adsk.fusion.Sketch.cast(None)
        self._runtime = None

    def prefixBase(self) -> str:
        return 'SpurGear'

    def newContext(self) -> SpurGearGenerationContext:
        return SpurGearGenerationContext()

    def addExtraPrimaryParameters(self, inputs):
        pass

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def processInputs(self, inputs: adsk.core.CommandInputs):
        parent = get_selection(inputs, INPUT_ID_PARENT)[0]
        self.plane = get_selection(inputs, INPUT_ID_PLANE)[0]
        self.anchorPoint = get_selection(inputs, INPUT_ID_ANCHOR_POINT)[0]
        if isinstance(parent, adsk.fusion.Occurrence):
            self.parentComponent = parent.component
        else:
            self.parentComponent = parent
        self.getOccurrence()
        # @@primary_parameters@@
        self.addParameter(
            PARAM_SKETCH_ONLY, adsk.core.ValueInput.createByReal(1 if get_boolean(inputs, INPUT_ID_SKETCH_ONLY) else 0),
            '', 'Generate sketches only')
        self.addExtraPrimaryParameters(inputs)
        self.registerDerivedParameters()

    # @@derived_parameters@@

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)
        self.validateGallery()
        component: adsk.fusion.Component = self.getComponent()
        component.name = self.generateName()
        self._createdTargetPlane = False
        if not isinstance(self.plane, adsk.fusion.ConstructionPlane):
            selectedPlane = self.plane
            planes: adsk.fusion.ConstructionPlanes = component.constructionPlanes
            planeInput: adsk.fusion.ConstructionPlaneInput = planes.createInput()
            offset = adsk.core.ValueInput.createByReal(0)
            planeInput.setByOffset(selectedPlane, offset)
            self.plane = planes.add(planeInput)
            self.plane.name = 'Target Plane'
            self.plane.isLightBulbOn = True
            self._createdTargetPlane = True
        ctx = self.newContext()
        ctx.plane = self.plane
        self.prepareTools(ctx)
        self.drawReferenceCircles(ctx)
        self.buildGear(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx: SpurGearGenerationContext):
        futil.log('Creating Tools sketch')
        toolsSketch: adsk.fusion.Sketch = self.createSketchObject('Tools', ctx.plane)
        toolsSketch.isVisible = True
        self._toolsSketch = toolsSketch
        ctx.anchorPoint = adsk.fusion.SketchPoint.cast(toolsSketch.project(self.anchorPoint).item(0))
        if ctx.anchorPoint is None:
            raise RuntimeError('Tools: anchor projection produced no SketchPoint')
        if not toolsSketch.isFullyConstrained:
            raise RuntimeError(f'{toolsSketch.name}: sketch is not fully constrained')

    def drawReferenceCircles(self, ctx: SpurGearGenerationContext):
        """The root, tip, base and pitch circles, labelled, in a sketch of their own. The gallery does not model them."""
        sketch: adsk.fusion.Sketch = self.createSketchObject('Reference Circles', ctx.plane)
        sketch.isVisible = True
        self._referenceSketch = sketch
        circles: adsk.fusion.SketchCircles = sketch.sketchCurves.sketchCircles
        dims: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        localOrigin: adsk.fusion.SketchPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        size = self.getParameter(PARAM_TIP_RADIUS).value - self.getParameter(PARAM_ROOT_RADIUS).value
        for name, parameter in (
                ('Root Circle', PARAM_ROOT_RADIUS),
                ('Tip Circle', PARAM_TIP_RADIUS),
                ('Base Circle', PARAM_BASE_RADIUS),
                ('Pitch Circle', PARAM_PITCH_RADIUS)):
            radius = self.getParameter(parameter).value
            circle: adsk.fusion.SketchCircle = circles.addByCenterRadius(localOrigin, radius)
            circle.isConstruction = True
            offCenterTextPoint = adsk.core.Point3D.create(radius, radius, 0)
            dimension: adsk.fusion.SketchDiameterDimension = dims.addDiameterDimension(circle, offCenterTextPoint)
            dimension.parameter.value = 2 * radius
            text = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput: adsk.fusion.SketchTextInput = sketch.sketchTexts.createInput2(text, size)
            alignment = cast(adsk.core.HorizontalAlignments, adsk.core.HorizontalAlignments.CenterHorizontalAlignment)
            textInput.setAsAlongPath(circle, True, alignment, 0)
            sketch.sketchTexts.add(textInput)
        projectedAnchor = adsk.fusion.SketchPoint.cast(sketch.project(ctx.anchorPoint).item(0))
        if projectedAnchor is None:
            raise RuntimeError(f'{sketch.name}: anchor projection produced no SketchPoint')
        sketch.geometricConstraints.addCoincident(localOrigin, projectedAnchor)

    def variant(self):
        """The changes this gear makes to the spur build. The spur makes none; a subclass returns its own."""
        return _Variant()

    def galleryParameters(self):
        """The Fusion parameter constant of each gallery parameter table entry. A subclass adds its own."""
        return _GALLERY_PARAMETERS

    def galleryParameter(self, name):
        """The Fusion name and user parameter of a gallery parameter table entry."""
        constant = self.galleryParameters()[name]
        return self.parameterName(constant), self.getParameter(constant)

    def galleryParams(self, scale=1.0):
        """The gallery's Params, read from the user parameters. scale is the centimetres of one unit of length of the
        translated code. The module is a unitless parameter that holds millimetres, so it is read as it is."""
        value = lambda constant: self.getParameter(constant).value
        length = lambda constant: value(constant) / scale
        return _GalleryParams(
            Module=value(PARAM_MODULE), Teeth=int(round(value(PARAM_TOOTH_NUMBER))),
            PressureAngle=value(PARAM_PRESSURE_ANGLE), Thickness=length(PARAM_THICKNESS),
            Samples=int(value(PARAM_INVOLUTE_STEPS)), BoreDiameter=length(PARAM_BORE_DIAMETER),
            CapChamfer=length(PARAM_CHAMFER_TOOTH), Placement=None)

    def validateGallery(self):
        """Run the gallery's input checks, translated, before anything is built. They read the derived dimensions
        from the user parameters, so a runtime over them is made active. It runs in millimetres, so a message shows
        the numbers the dialog shows."""
        runtime = _fx.Runtime(self.getComponent(), None, None, self.galleryParameter, False, _MILLIMETRE)
        runtime.activate()
        _validate(self.galleryParams(_MILLIMETRE))

    def buildGear(self, ctx: SpurGearGenerationContext):
        """Run the gallery's build, translated, in this design."""
        sketchOnly = self.getParameterAsBoolean(PARAM_SKETCH_ONLY)
        runtime = _fx.Runtime(self.getComponent(), ctx.plane, ctx.anchorPoint, self.galleryParameter, sketchOnly)
        runtime.activate()
        self._runtime = runtime
        runtime.build_frame(self._toolsSketch)
        value = lambda constant: self.getParameter(constant).value
        params = self.galleryParams()
        dimensions = _Dimensions(
            value(PARAM_PITCH_RADIUS), value(PARAM_BASE_RADIUS), value(PARAM_ROOT_RADIUS),
            value(PARAM_TIP_RADIUS), value(PARAM_FILLET_RADIUS))
        frame = _placement(params)
        run = _Run(
            doc=runtime, world=runtime, p=params, d=dimensions, frame=frame, lower=runtime.xy(),
            extent=_fx.Distance(D=runtime.param('thickness'), Dir=_fx.DECAD_ALONG), teeth=params.Teeth)
        try:
            _variant_assemble(self.variant(), run)
        except _fx.SketchOnly:
            futil.log('Sketch only: stopped before the first feature')

    def cleanup(self, ctx: SpurGearGenerationContext):
        if self._createdTargetPlane and ctx.plane is not None:
            ctx.plane.isLightBulbOn = False
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        if self._toolsSketch is not None:
            self._toolsSketch.isVisible = False
        if self._referenceSketch is not None:
            self._referenceSketch.isVisible = False
        if self._runtime is not None:
            self._runtime.hide_helpers()


# @@geometry_helpers@@

# @@gallery_parameters@@

_GalleryParams = collections.namedtuple(
    'GalleryParams',
    ['Module', 'Teeth', 'PressureAngle', 'Thickness', 'Samples', 'BoreDiameter', 'CapChamfer', 'Placement'])
_Run = collections.namedtuple('Run', ['doc', 'world', 'p', 'd', 'frame', 'lower', 'extent', 'teeth'])
