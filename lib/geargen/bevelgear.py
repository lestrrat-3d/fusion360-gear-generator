"""Bevel gear generator, emitted from the compiled step list spec/bevelgear/steps.md.

Bevel is a standalone generator: it does not subclass base.Generator, registers no live Fusion
user parameters, and precomputes every value in Python (internal cm) before writing geometry
numerically ([PB-PRECOMPUTED-MODE]). Per-gear anchors travel in plain dicts (pinionCtx /
drivingCtx) rather than a GenerationContext class; shared anchors are stashed on self.

Every adsk.* call below is made on a local variable whose origin is a single, traceable step —
an assignment straight off a typed parameter, a prior call's return, or a property read — rather
than a chained expression, so the receiver of every call is always a bare name.
"""

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .base import get_boolean, get_selection
from .misc import get_design, to_cm, to_mm
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy
from .utilities import find_profile_by_curve_counts
from . import solids


# Dialog input ids (step 1), in the table's row order.
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

# 2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2631578..., rounded UP (step 2).
_MIN_TEETH_CONST = 5.2632

# The pinion's extra mesh-phase rotation (step 24) is zero teeth of phase, i.e. no move.
_PINION_MESH_PHASE_TEETH = 0


def _pinionMeshPhase(pinionTeeth):
    return _PINION_MESH_PHASE_TEETH * 2.0 * math.pi / pinionTeeth


def distAlong(point: adsk.core.Point3D, apexWorld: adsk.core.Point3D,
             coneVec: adsk.core.Vector3D) -> float:
    # Signed distance of `point` from `apexWorld` measured along the unit vector `coneVec`.
    apexToPoint: adsk.core.Vector3D = apexWorld.vectorTo(point)
    return apexToPoint.dotProduct(coneVec)


class BevelGearCommandInputsConfigurator:
    """Step 1: the command dialog's 20 inputs, added in exact row order."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane — first, so Fusion's auto-focus (FIRST SelectionCommandInput,
        #    [PB-AUTOFOCUS-FIRST]) opens the dialog on it.
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point.
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component — pre-selected to the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        design: adsk.fusion.Design = get_design()
        rootComponent = design.rootComponent
        parentInput.addSelection(rootComponent)

        # 4. Module.
        oneReal = adsk.core.ValueInput.createByReal(1)
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '', oneReal)

        # 5. Shaft Angle.
        ninetyDegString = adsk.core.ValueInput.createByString('90 deg')
        inputs.addValueInput(INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg', ninetyDegString)

        # 6. Driving Gear Teeth.
        thirtyOneReal = adsk.core.ValueInput.createByReal(31)
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '', thirtyOneReal)

        # 7. Pinion Gear Teeth.
        thirtyOneRealB = adsk.core.ValueInput.createByReal(31)
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '', thirtyOneRealB)

        # 8. Driving Gear Base Height.
        drivingBaseHeightDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            drivingBaseHeightDefault)

        # 9. Pinion Gear Base Height.
        pinionBaseHeightDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            pinionBaseHeightDefault)

        # 10. Enable Bore.
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11. Driving Gear Bore Diameter.
        drivingBoreDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm', drivingBoreDefault)

        # 12. Pinion Gear Bore Diameter.
        pinionBoreDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm', pinionBoreDefault)

        # 13. Face Width.
        faceWidthDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm', faceWidthDefault)

        # 14. Tooth Spacing.
        toothSpacingDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm', toothSpacingDefault)

        # 15. Mean Spiral Angle.
        thirtyFiveDegString = adsk.core.ValueInput.createByString('35 deg')
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg', thirtyFiveDegString)

        # 16. Hand of Spiral — text-list dropdown, Right selected.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
        handListItems = handInput.listItems
        handListItems.add(_HAND_RIGHT, True)
        handListItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius.
        cutterRadiusDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm', cutterRadiusDefault)

        # 18. Toe Extension.
        toeExtensionDefault = adsk.core.ValueInput.createByReal(0)
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension', '', toeExtensionDefault)

        # 19. Driving Gear Toe Radius.
        drivingToeRadiusDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
            drivingToeRadiusDefault)

        # 20. Pinion Gear Toe Radius.
        pinionToeRadiusDefault = adsk.core.ValueInput.createByReal(to_cm(0))
        inputs.addValueInput(
            INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
            pinionToeRadiusDefault)

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args):
        eventInputs = args.inputs
        cls._updateSpiralInputVisibility(eventInputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterRadiusInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterRadiusInput is None:
            return

        design: adsk.fusion.Design = get_design()
        unitsManager = design.unitsManager
        try:
            spiralExpression = spiralInput.expression
            spiralAngle_rad = unitsManager.evaluateExpression(spiralExpression, 'rad')
            visible = spiralAngle_rad > 0
        except Exception:
            visible = True

        handInput.isVisible = visible
        cutterRadiusInput.isVisible = visible


class BevelGearGenerator:
    """Steps 3-26. A standalone generator: no base.Generator, no live user parameters
    ([PB-PRECOMPUTED-MODE]). Never activates any occurrence ([PB-NEVER-ACTIVATE],
    [BEVEL-F-NEVER-ACTIVATE]) except the scale step's Design-occurrence activation in
    _transformToothBody (the one documented exception)."""

    # Step 17: tunable crown constant. 0 disables the crown; 0.5 is the shipped value.
    _CROWN_PER_RAD = 0.5

    # Declared attribute types so every self.* receiver below is resolvable: the checker
    # reads annotations, not how a value was produced, so each Fusion-object attribute
    # this class carries across methods is named here with its qualified class.
    design: adsk.fusion.Design
    bevelOccurrence: adsk.fusion.Occurrence
    bevelComponent: adsk.fusion.Component
    designOccurrence: adsk.fusion.Occurrence
    designComponent: adsk.fusion.Component
    _anchorCenterPoint: adsk.fusion.SketchPoint
    _gearProfilesPlane: adsk.fusion.ConstructionPlane
    _gpSketch: adsk.fusion.Sketch
    _apexSketchPoint: adsk.fusion.SketchPoint

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # ------------------------------------------------------------------------------------
    # Step 3: occurrence tree, then the per-step orchestration.
    # ------------------------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        design: adsk.fusion.Design = self.design

        (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)
        parentComponent: adsk.fusion.Component

        # Bevel Gear occurrence, under the user's Parent Component.
        parentOccurrences: adsk.fusion.Occurrences = parentComponent.occurrences
        bevelMatrix = adsk.core.Matrix3D.create()
        bevelOccurrence: adsk.fusion.Occurrence = parentOccurrences.addNewComponent(bevelMatrix)
        bevelComponent: adsk.fusion.Component = bevelOccurrence.component
        bevelComponent.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        self.bevelComponent = bevelComponent

        # Design occurrence, under Bevel Gear. Every sketch/construction/feature runs here.
        bevelOccurrences: adsk.fusion.Occurrences = bevelComponent.occurrences
        designMatrix = adsk.core.Matrix3D.create()
        designOccurrence: adsk.fusion.Occurrence = bevelOccurrences.addNewComponent(designMatrix)
        designComponent: adsk.fusion.Component = designOccurrence.component
        designComponent.name = 'Design'
        self.designOccurrence = designOccurrence
        self.designComponent = designComponent

        anchorSketch, anchorLine = self._buildAnchorSketch(design, targetPlane, centerPoint)
        anchorLine: adsk.fusion.SketchLine

        gearProfilesPlane, gpSketch = self._buildGearProfilesPlane(
            designComponent, targetPlane, anchorLine)
        gearProfilesPlane: adsk.fusion.ConstructionPlane
        gpSketch: adsk.fusion.Sketch
        self._gearProfilesPlane = gearProfilesPlane
        self._gpSketch = gpSketch

        pinionCtx, drivingCtx = self._buildGearProfiles(
            designComponent, gpSketch, targetPlane, anchorLine, module_mm, drivingTeeth,
            pinionTeeth, shaftAngle_deg)

        drivingMeshAngle_rad = math.pi / drivingTeeth
        pinionMeshAngle_rad = _pinionMeshPhase(pinionTeeth)

        self._createGearBody(
            'Pinion', module_mm, pinionTeeth, pinionCtx['gamma'],
            pinionCtx['toothCentreRefLine'], pinionCtx['toothCentrePoint'],
            pinionCtx['hexagon'], pinionCtx['toeEdge'][0], pinionCtx['toeEdge'][1],
            pinionCtx['heelEdge'][0], pinionCtx['heelEdge'][1], pinionCtx['toeCone'],
            pinionCtx['heelCone'], pinionMeshAngle_rad)

        self._createGearBody(
            'Driving', module_mm, drivingTeeth, drivingCtx['gamma'],
            drivingCtx['toothCentreRefLine'], drivingCtx['toothCentrePoint'],
            drivingCtx['hexagon'], drivingCtx['toeEdge'][0], drivingCtx['toeEdge'][1],
            drivingCtx['heelEdge'][0], drivingCtx['heelEdge'][1], drivingCtx['toeCone'],
            drivingCtx['heelCone'], drivingMeshAngle_rad)

        bevelComponentFinal = self.bevelComponent
        solids.hide_construction_geometry(bevelComponentFinal)

    def deleteComponent(self):
        bevelOccurrence: adsk.fusion.Occurrence = self.bevelOccurrence
        if bevelOccurrence is not None:
            bevelOccurrence.deleteMe()

    # ------------------------------------------------------------------------------------
    # Step 2: read every input, resolve every derived value and bound.
    # ------------------------------------------------------------------------------------

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        design: adsk.fusion.Design = self.design
        unitsManager = design.unitsManager

        parentEntities = get_selection(inputs, INPUT_ID_PARENT)
        parentComponent = parentEntities[0]
        targetEntities = get_selection(inputs, INPUT_ID_PLANE)
        targetPlane = targetEntities[0]
        centerEntities = get_selection(inputs, INPUT_ID_CENTER_POINT)
        centerPoint = centerEntities[0]

        moduleInput = inputs.itemById(INPUT_ID_MODULE)
        moduleExpression = moduleInput.expression
        module_mm = unitsManager.evaluateExpression(moduleExpression, '')

        shaftAngleInput = inputs.itemById(INPUT_ID_SHAFT_ANGLE)
        shaftAngleExpression = shaftAngleInput.expression
        shaftAngle_rad_raw = unitsManager.evaluateExpression(shaftAngleExpression, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad_raw)

        drivingTeethInput = inputs.itemById(INPUT_ID_DRIVING_TEETH)
        drivingTeethExpression = drivingTeethInput.expression
        drivingTeethRaw = unitsManager.evaluateExpression(drivingTeethExpression, '')
        drivingTeeth = int(round(drivingTeethRaw))

        pinionTeethInput = inputs.itemById(INPUT_ID_PINION_TEETH)
        pinionTeethExpression = pinionTeethInput.expression
        pinionTeethRaw = unitsManager.evaluateExpression(pinionTeethExpression, '')
        pinionTeeth = int(round(pinionTeethRaw))

        drivingBaseHeightInput = inputs.itemById(INPUT_ID_DRIVING_BASE_HEIGHT)
        drivingBaseHeightExpression = drivingBaseHeightInput.expression
        drivingBaseHeight_cm = unitsManager.evaluateExpression(drivingBaseHeightExpression, 'mm')

        pinionBaseHeightInput = inputs.itemById(INPUT_ID_PINION_BASE_HEIGHT)
        pinionBaseHeightExpression = pinionBaseHeightInput.expression
        pinionBaseHeight_cm = unitsManager.evaluateExpression(pinionBaseHeightExpression, 'mm')

        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)

        drivingBoreInput = inputs.itemById(INPUT_ID_DRIVING_BORE)
        drivingBoreExpression = drivingBoreInput.expression
        drivingBore_cm = unitsManager.evaluateExpression(drivingBoreExpression, 'mm')

        pinionBoreInput = inputs.itemById(INPUT_ID_PINION_BORE)
        pinionBoreExpression = pinionBoreInput.expression
        pinionBore_cm = unitsManager.evaluateExpression(pinionBoreExpression, 'mm')

        faceWidthInput = inputs.itemById(INPUT_ID_FACE_WIDTH)
        faceWidthExpression = faceWidthInput.expression
        faceWidth_cm = unitsManager.evaluateExpression(faceWidthExpression, 'mm')

        toothSpacingInput = inputs.itemById(INPUT_ID_TOOTH_SPACING)
        toothSpacingExpression = toothSpacingInput.expression
        toothSpacing_cm = unitsManager.evaluateExpression(toothSpacingExpression, 'mm')

        spiralAngleInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        spiralAngleExpression = spiralAngleInput.expression
        spiralAngle_rad = unitsManager.evaluateExpression(spiralAngleExpression, 'deg')

        handDropDown = inputs.itemById(INPUT_ID_HAND)
        selectedHandItem = handDropDown.selectedItem
        if selectedHandItem is None:
            hand = _HAND_RIGHT
        else:
            hand = selectedHandItem.name

        cutterRadiusInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        cutterRadiusExpression = cutterRadiusInput.expression
        cutterRadius_cm = unitsManager.evaluateExpression(cutterRadiusExpression, 'mm')

        toeExtensionInput = inputs.itemById(INPUT_ID_TOE_EXTENSION)
        toeExtensionExpression = toeExtensionInput.expression
        toeExtension = unitsManager.evaluateExpression(toeExtensionExpression, '')

        drivingToeRadiusInput = inputs.itemById(INPUT_ID_DRIVING_TOE_RADIUS)
        drivingToeRadiusExpression = drivingToeRadiusInput.expression
        drivingToeRadius_cm = unitsManager.evaluateExpression(drivingToeRadiusExpression, 'mm')

        pinionToeRadiusInput = inputs.itemById(INPUT_ID_PINION_TOE_RADIUS)
        pinionToeRadiusExpression = pinionToeRadiusInput.expression
        pinionToeRadius_cm = unitsManager.evaluateExpression(pinionToeRadiusExpression, 'mm')

        # Range checks.
        if module_mm <= 0:
            raise Exception(f'Module must be > 0 (got {module_mm})')
        if pinionTeeth < 3:
            raise Exception(f'Pinion Gear Teeth must be >= 3 (got {pinionTeeth})')
        if drivingTeeth < 3:
            raise Exception(f'Driving Gear Teeth must be >= 3 (got {drivingTeeth})')
        if shaftAngle_deg < 30:
            raise Exception(f'Shaft Angle must be >= 30 deg (got {shaftAngle_deg})')

        PPD_mm = module_mm * pinionTeeth
        DPD_mm = module_mm * drivingTeeth
        coneDistanceRaw_mm = math.sqrt(DPD_mm * DPD_mm + PPD_mm * PPD_mm)
        coneDistance_cm = to_cm(coneDistanceRaw_mm)

        smallerPD_mm = min(DPD_mm, PPD_mm)
        largerPD_mm = max(DPD_mm, PPD_mm)
        coneLimitDeg = math.degrees(math.acos(-smallerPD_mm / largerPD_mm))
        if coneLimitDeg <= 150:
            if shaftAngle_deg >= coneLimitDeg:
                raise Exception(
                    f'Shaft Angle must be < {coneLimitDeg} deg for this tooth pair '
                    f'(got {shaftAngle_deg})')
        else:
            if shaftAngle_deg > 150:
                raise Exception(f'Shaft Angle must be <= 150 deg (got {shaftAngle_deg})')

        sigma_rad = math.radians(shaftAngle_deg)
        gamma_p = math.atan2(
            math.sin(sigma_rad) * PPD_mm, DPD_mm + PPD_mm * math.cos(sigma_rad))
        gamma_g = sigma_rad - gamma_p

        R_mm = (PPD_mm / 2.0) / math.sin(gamma_p)

        minTeethFloor_p = _MIN_TEETH_CONST * math.cos(gamma_p)
        if pinionTeeth < minTeethFloor_p:
            raise Exception(
                f'Pinion Gear Teeth must be >= {minTeethFloor_p:.4f} at this Shaft Angle '
                f'(got {pinionTeeth})')
        minTeethFloor_g = _MIN_TEETH_CONST * math.cos(gamma_g)
        if drivingTeeth < minTeethFloor_g:
            raise Exception(
                f'Driving Gear Teeth must be >= {minTeethFloor_g:.4f} at this Shaft Angle '
                f'(got {drivingTeeth})')

        r_p_mm = PPD_mm / 2.0
        r_g_mm = DPD_mm / 2.0
        minBaseHeight_p_mm = 1.05 * 1.25 * module_mm * math.sin(gamma_p)
        maxBaseHeight_p_mm = (
            0.95 * (r_p_mm - 1.25 * module_mm * math.cos(gamma_p)) * math.tan(gamma_p))
        minBaseHeight_g_mm = 1.05 * 1.25 * module_mm * math.sin(gamma_g)
        maxBaseHeight_g_mm = (
            0.95 * (r_g_mm - 1.25 * module_mm * math.cos(gamma_g)) * math.tan(gamma_g))

        # Driving Gear Base Height resolves first.
        if drivingBaseHeight_cm == 0:
            drivingFallback_mm = module_mm * drivingTeeth / 8.0
            drivingFallback_mm = max(drivingFallback_mm, minBaseHeight_g_mm)
            drivingFallback_mm = min(drivingFallback_mm, maxBaseHeight_g_mm)
            drivingBaseHeightResolved_cm = to_cm(drivingFallback_mm)
        else:
            minBaseHeight_g_cm = to_cm(minBaseHeight_g_mm)
            maxBaseHeight_g_cm = to_cm(maxBaseHeight_g_mm)
            if drivingBaseHeight_cm < minBaseHeight_g_cm or drivingBaseHeight_cm > maxBaseHeight_g_cm:
                raise Exception(
                    f'Driving Gear Base Height must be within '
                    f'[{minBaseHeight_g_cm}, {maxBaseHeight_g_cm}] cm '
                    f'(got {drivingBaseHeight_cm})')
            drivingBaseHeightResolved_cm = drivingBaseHeight_cm

        # The pinion's fallback is keyed off the driving side's RESOLVED value.
        if pinionBaseHeight_cm == 0:
            drivingResolved_mm = to_mm(drivingBaseHeightResolved_cm)
            pinionFallback_mm = drivingResolved_mm * (pinionTeeth / drivingTeeth)
            pinionFallback_mm = min(pinionFallback_mm, maxBaseHeight_p_mm)
            pinionBaseHeightResolved_cm = to_cm(pinionFallback_mm)
        else:
            minBaseHeight_p_cm = to_cm(minBaseHeight_p_mm)
            maxBaseHeight_p_cm = to_cm(maxBaseHeight_p_mm)
            if pinionBaseHeight_cm < minBaseHeight_p_cm or pinionBaseHeight_cm > maxBaseHeight_p_cm:
                raise Exception(
                    f'Pinion Gear Base Height must be within '
                    f'[{minBaseHeight_p_cm}, {maxBaseHeight_p_cm}] cm '
                    f'(got {pinionBaseHeight_cm})')
            pinionBaseHeightResolved_cm = pinionBaseHeight_cm

        if toothSpacing_cm < 0:
            raise Exception(f'Tooth Spacing must be >= 0 (got {toothSpacing_cm})')
        if cutterRadius_cm < 0:
            raise Exception(f'Cutter Radius must be >= 0 (got {cutterRadius_cm})')
        if drivingToeRadius_cm < 0:
            raise Exception(f'Driving Gear Toe Radius must be >= 0 (got {drivingToeRadius_cm})')
        if pinionToeRadius_cm < 0:
            raise Exception(f'Pinion Gear Toe Radius must be >= 0 (got {pinionToeRadius_cm})')

        spiralAngleDeg = math.degrees(spiralAngle_rad)
        if spiralAngleDeg < 0 or spiralAngleDeg >= 60:
            raise Exception(f'Mean Spiral Angle must be in [0, 60) deg (got {spiralAngleDeg})')
        if toeExtension < 0 or toeExtension > 100:
            raise Exception(f'Toe Extension must be in [0, 100] (got {toeExtension})')

        self._drivingBaseHeight_cm = drivingBaseHeightResolved_cm
        self._pinionBaseHeight_cm = pinionBaseHeightResolved_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBore_cm
        self._pinionBore_cm = pinionBore_cm
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm

        # The rest _readInputs resolves for later steps, stashed the same way.
        self._toeExtension = toeExtension
        self._drivingToeRadius_cm = drivingToeRadius_cm
        self._pinionToeRadius_cm = pinionToeRadius_cm
        self._coneDistance_cm = coneDistance_cm
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._R_mm = R_mm
        self._PPD_mm = PPD_mm
        self._DPD_mm = DPD_mm

        return (parentComponent, targetPlane, centerPoint, module_mm, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # ------------------------------------------------------------------------------------
    # Step 4: the Anchor sketch, directly on the user-selected target plane.
    # ------------------------------------------------------------------------------------

    def _buildAnchorSketch(self, design: adsk.fusion.Design, plane: adsk.core.Base,
                           center: adsk.core.Base):
        designComponent: adsk.fusion.Component = self.designComponent
        sketches: adsk.fusion.Sketches = designComponent.sketches
        sketch: adsk.fusion.Sketch = sketches.add(plane)
        sketch.name = 'Anchor'

        projectedCenterColl: adsk.core.ObjectCollection = sketch.project(center)
        projectedCenter: adsk.fusion.SketchPoint = projectedCenterColl.item(0)

        centerGeom = projectedCenter.geometry
        startPoint = adsk.core.Point3D.create(centerGeom.x - 0.5, centerGeom.y, centerGeom.z)
        endPoint = adsk.core.Point3D.create(centerGeom.x + 0.5, centerGeom.y, centerGeom.z)

        sketchCurves: adsk.fusion.SketchCurves = sketch.sketchCurves
        sketchLines: adsk.fusion.SketchLines = sketchCurves.sketchLines
        anchorLine: adsk.fusion.SketchLine = sketchLines.addByTwoPoints(startPoint, endPoint)

        geometricConstraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        geometricConstraints.addCoincident(projectedCenter, anchorLine)
        geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        anchorLineStart = anchorLine.startSketchPoint
        anchorLineEnd = anchorLine.endSketchPoint
        sketchDimensions: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        dimTextPoint = adsk.core.Point3D.create(centerGeom.x, centerGeom.y - 0.3, centerGeom.z)
        alignedOrientation = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation
        sketchDimensions.addDistanceDimension(
            anchorLineStart, anchorLineEnd, alignedOrientation, dimTextPoint)
        # Deliberately no .parameter.value assignment: the dimension locks the seeded 10 mm
        # length and its value is arbitrary because nothing downstream reads it.

        geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorCenterPoint = projectedCenter
        return sketch, anchorLine

    # ------------------------------------------------------------------------------------
    # Step 5: the Gear Profiles plane and sketch.
    # ------------------------------------------------------------------------------------

    def _buildGearProfilesPlane(self, designComponent: adsk.fusion.Component,
                                plane: adsk.core.Base, anchorLine: adsk.fusion.SketchLine):
        constructionPlanes: adsk.fusion.ConstructionPlanes = designComponent.constructionPlanes
        planeInput: adsk.fusion.ConstructionPlaneInput = constructionPlanes.createInput()
        ninetyDeg = adsk.core.ValueInput.createByString('90 deg')
        planeInput.setByAngle(anchorLine, ninetyDeg, plane)
        gearProfilesPlane: adsk.fusion.ConstructionPlane = constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketches: adsk.fusion.Sketches = designComponent.sketches
        gpSketch: adsk.fusion.Sketch = sketches.add(gearProfilesPlane)
        gpSketch.name = 'Gear Profiles'

        return gearProfilesPlane, gpSketch

    # ------------------------------------------------------------------------------------
    # Step 6: the Gear Profiles sketch -- the whole §2 lattice for both gears.
    # ------------------------------------------------------------------------------------

    def _buildGearProfiles(self, designComponent: adsk.fusion.Component,
                           gpSketch: adsk.fusion.Sketch, targetPlane: adsk.core.Base,
                           anchorLine: adsk.fusion.SketchLine, module_mm: float,
                           drivingTeeth: int, pinionTeeth: int, shaftAngle_deg: float):
        sketch: adsk.fusion.Sketch = gpSketch
        sketchCurves: adsk.fusion.SketchCurves = sketch.sketchCurves
        sketchLines: adsk.fusion.SketchLines = sketchCurves.sketchLines
        geometricConstraints: adsk.fusion.GeometricConstraints = sketch.geometricConstraints
        sketchDimensions: adsk.fusion.SketchDimensions = sketch.sketchDimensions
        alignedOrientation = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        def newLine(x1, y1, x2, y2, lines: adsk.fusion.SketchLines = sketchLines
                   ) -> adsk.fusion.SketchLine:
            p1 = adsk.core.Point3D.create(x1, y1, 0.0)
            p2 = adsk.core.Point3D.create(x2, y2, 0.0)
            line: adsk.fusion.SketchLine = lines.addByTwoPoints(p1, p2)
            line.isConstruction = True
            return line

        def lineIntersect(p1, d1, p2, d2):
            denom = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(denom) < 1e-12:
                return (p2[0], p2[1])
            t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
            return (p1[0] + t * d1[0], p1[1] + t * d1[1])

        def perpDistToLine(px, py, x1, y1, x2, y2):
            dx = x2 - x1
            dy = y2 - y1
            segLen = math.hypot(dx, dy)
            if segLen == 0:
                return math.hypot(px - x1, py - y1)
            cross = abs(dx * (y1 - py) - (x1 - px) * dy)
            return cross / segLen

        # Project the Anchor sketch's centre and its line -- not the user's raw selections.
        anchorCenterPoint: adsk.fusion.SketchPoint = self._anchorCenterPoint
        projectedCenterColl: adsk.core.ObjectCollection = sketch.project(anchorCenterPoint)
        projectedCenter: adsk.fusion.SketchPoint = projectedCenterColl.item(0)
        projectedAnchorLineColl: adsk.core.ObjectCollection = sketch.project(anchorLine)
        projectedAnchorLine: adsk.fusion.SketchLine = projectedAnchorLineColl.item(0)

        centerGeom = projectedCenter.geometry
        cx0, cy0 = centerGeom.x, centerGeom.y

        lineStartPoint = projectedAnchorLine.startSketchPoint
        lineEndPoint = projectedAnchorLine.endSketchPoint
        lineStartGeom = lineStartPoint.geometry
        lineEndGeom = lineEndPoint.geometry
        dx = lineEndGeom.x - lineStartGeom.x
        dy = lineEndGeom.y - lineStartGeom.y
        dlen = math.hypot(dx, dy)
        dirX = dx / dlen
        dirY = dy / dlen
        perpX = -dirY
        perpY = dirX

        # The sign of `perp` is chosen by the target-plane normal -- the single permitted
        # world use in §2 ([BEVEL-F-GROW-SIDE]).
        targetGeometry = targetPlane.geometry
        targetNormal = targetGeometry.normal

        sampleLocal = adsk.core.Point3D.create(cx0 + perpX, cy0 + perpY, 0.0)
        sampleWorld = sketch.sketchToModelSpace(sampleLocal)
        centerLocal = adsk.core.Point3D.create(cx0, cy0, 0.0)
        centerWorld = sketch.sketchToModelSpace(centerLocal)
        perpWorldVec = centerWorld.vectorTo(sampleWorld)
        perpDot = perpWorldVec.dotProduct(targetNormal)
        if perpDot < 0:
            perpX = -perpX
            perpY = -perpY

        R_mm = self._R_mm
        gamma_p = self._gamma_p
        gamma_g = self._gamma_g
        sigma_rad = math.radians(shaftAngle_deg)
        drivingBaseHeight_cm = self._drivingBaseHeight_cm
        pinionBaseHeight_cm = self._pinionBaseHeight_cm
        PPD_mm = self._PPD_mm
        DPD_mm = self._DPD_mm

        # Centre -> Apex.
        apexDist_cm = to_cm(R_mm * math.cos(gamma_g)) + drivingBaseHeight_cm
        apexX = cx0 + perpX * apexDist_cm
        apexY = cy0 + perpY * apexDist_cm

        centerToApex = newLine(cx0, cy0, apexX, apexY)
        centerToApexStart = centerToApex.startSketchPoint
        geometricConstraints.addCoincident(centerToApexStart, projectedCenter)
        geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)
        apexSketchPoint = centerToApex.endSketchPoint

        # Driving Gear Shaft Axis: from the Apex toward the anchor line, along -perp.
        drivingDirX = -perpX
        drivingDirY = -perpY
        drivingAxisDist_cm = to_cm(R_mm * math.cos(gamma_g))
        bX = apexX + drivingDirX * drivingAxisDist_cm
        bY = apexY + drivingDirY * drivingAxisDist_cm

        drivingShaftAxis = newLine(apexX, apexY, bX, bY)
        drivingShaftAxisStart = drivingShaftAxis.startSketchPoint
        geometricConstraints.addCoincident(drivingShaftAxisStart, apexSketchPoint)
        geometricConstraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Gear Shaft Axis: rotate the driving direction by +/- Sigma, keep the
        # candidate whose endpoint has the greater X.
        def rotate2d(x, y, theta):
            cosT = math.cos(theta)
            sinT = math.sin(theta)
            return (x * cosT - y * sinT, x * sinT + y * cosT)

        candidate1 = rotate2d(drivingDirX, drivingDirY, sigma_rad)
        candidate2 = rotate2d(drivingDirX, drivingDirY, -sigma_rad)
        pinionAxisDist_cm = to_cm(R_mm * math.cos(gamma_p))
        cand1EndX = apexX + candidate1[0] * pinionAxisDist_cm
        cand2EndX = apexX + candidate2[0] * pinionAxisDist_cm
        if cand1EndX >= cand2EndX:
            pinionDirX, pinionDirY = candidate1
        else:
            pinionDirX, pinionDirY = candidate2

        aX = apexX + pinionDirX * pinionAxisDist_cm
        aY = apexY + pinionDirY * pinionAxisDist_cm

        pinionShaftAxis = newLine(apexX, apexY, aX, aY)
        pinionShaftAxisStart = pinionShaftAxis.startSketchPoint
        geometricConstraints.addCoincident(pinionShaftAxisStart, apexSketchPoint)

        sumX = pinionDirX + drivingDirX
        sumY = pinionDirY + drivingDirY
        sumLen = math.hypot(sumX, sumY)
        normSumX = sumX / sumLen
        normSumY = sumY / sumLen
        angularTextDist_cm = to_cm(PPD_mm) / 4.0
        angularTextX = apexX + normSumX * angularTextDist_cm
        angularTextY = apexY + normSumY * angularTextDist_cm
        angularTextPoint = adsk.core.Point3D.create(angularTextX, angularTextY, 0.0)

        angularDim = sketchDimensions.addAngularDimension(
            pinionShaftAxis, drivingShaftAxis, angularTextPoint)
        angularDimParameter = angularDim.parameter
        angularDimParameter.value = sigma_rad
        pointA = pinionShaftAxis.endSketchPoint

        # The two drops, closing at Apex 2. From A, perpendicular to the Pinion Shaft Axis,
        # toward B (the interior wedge). From B, perpendicular to the Driving Shaft Axis,
        # toward A -- NOT by a "toward the anchor line" test, which is near-degenerate here.
        aGeom0 = pointA.geometry
        abX = bX - aX
        abY = bY - aY
        dropACand1 = (-pinionDirY, pinionDirX)
        dropACand2 = (pinionDirY, -pinionDirX)
        dotA1 = dropACand1[0] * abX + dropACand1[1] * abY
        dropADir = dropACand1 if dotA1 > 0 else dropACand2

        dropAValue_cm = to_cm(PPD_mm) / 2.0
        dropAEndX = aGeom0.x + dropADir[0] * dropAValue_cm
        dropAEndY = aGeom0.y + dropADir[1] * dropAValue_cm
        dropFromA = newLine(aGeom0.x, aGeom0.y, dropAEndX, dropAEndY)
        dropFromAStart = dropFromA.startSketchPoint
        geometricConstraints.addCoincident(dropFromAStart, pointA)
        geometricConstraints.addPerpendicular(dropFromA, pinionShaftAxis)
        dropAStart2 = dropFromA.startSketchPoint
        dropAEnd2 = dropFromA.endSketchPoint
        dropATextPoint = adsk.core.Point3D.create(
            (aGeom0.x + dropAEndX) / 2.0, (aGeom0.y + dropAEndY) / 2.0, 0.0)
        dropADim = sketchDimensions.addDistanceDimension(
            dropAStart2, dropAEnd2, alignedOrientation, dropATextPoint)
        dropADimParameter = dropADim.parameter
        dropADimParameter.value = dropAValue_cm

        bGeom0 = pointB.geometry
        baX = aX - bX
        baY = aY - bY
        dropBCand1 = (perpY, -perpX)
        dropBCand2 = (-perpY, perpX)
        dotB1 = dropBCand1[0] * baX + dropBCand1[1] * baY
        dropBDir = dropBCand1 if dotB1 > 0 else dropBCand2

        dropBValue_cm = to_cm(DPD_mm) / 2.0
        dropBEndX = bGeom0.x + dropBDir[0] * dropBValue_cm
        dropBEndY = bGeom0.y + dropBDir[1] * dropBValue_cm
        dropFromB = newLine(bGeom0.x, bGeom0.y, dropBEndX, dropBEndY)
        dropFromBStart = dropFromB.startSketchPoint
        geometricConstraints.addCoincident(dropFromBStart, pointB)
        geometricConstraints.addPerpendicular(dropFromB, drivingShaftAxis)
        dropBStart2 = dropFromB.startSketchPoint
        dropBEnd2 = dropFromB.endSketchPoint
        dropBTextPoint = adsk.core.Point3D.create(
            (bGeom0.x + dropBEndX) / 2.0, (bGeom0.y + dropBEndY) / 2.0, 0.0)
        dropBDim = sketchDimensions.addDistanceDimension(
            dropBStart2, dropBEnd2, alignedOrientation, dropBTextPoint)
        dropBDimParameter = dropBDim.parameter
        dropBDimParameter.value = dropBValue_cm

        dropAEndPoint = dropFromA.endSketchPoint
        dropBEndPoint = dropFromB.endSketchPoint
        geometricConstraints.addCoincident(dropAEndPoint, dropBEndPoint)
        apex2Point = dropFromA.endSketchPoint

        # The Pitch Line and the two dedendum lines.
        apex2Geom = apex2Point.geometry
        pitchLine = newLine(apexX, apexY, apex2Geom.x, apex2Geom.y)
        pitchLineStart = pitchLine.startSketchPoint
        pitchLineEnd = pitchLine.endSketchPoint
        geometricConstraints.addCoincident(pitchLineStart, apexSketchPoint)
        geometricConstraints.addCoincident(pitchLineEnd, apex2Point)

        pitchDirX = apex2Geom.x - apexX
        pitchDirY = apex2Geom.y - apexY
        pitchDirLen = math.hypot(pitchDirX, pitchDirY)
        pitchDirX /= pitchDirLen
        pitchDirY /= pitchDirLen
        dedCand1 = (-pitchDirY, pitchDirX)
        dedCand2 = (pitchDirY, -pitchDirX)
        apex2ToCenterX = cx0 - apex2Geom.x
        apex2ToCenterY = cy0 - apex2Geom.y
        dotDed1 = dedCand1[0] * apex2ToCenterX + dedCand1[1] * apex2ToCenterY
        if dotDed1 > 0:
            drivingDedDir = dedCand1
            pinionDedDir = dedCand2
        else:
            drivingDedDir = dedCand2
            pinionDedDir = dedCand1

        dedLen_cm = to_cm(module_mm * 1.25)
        dGx = apex2Geom.x + drivingDedDir[0] * dedLen_cm
        dGy = apex2Geom.y + drivingDedDir[1] * dedLen_cm
        drivingDedendumLine = newLine(apex2Geom.x, apex2Geom.y, dGx, dGy)
        drivingDedStart = drivingDedendumLine.startSketchPoint
        geometricConstraints.addPerpendicular(drivingDedendumLine, pitchLine)
        geometricConstraints.addCoincident(drivingDedStart, apex2Point)
        drivingDedStart2 = drivingDedendumLine.startSketchPoint
        drivingDedEnd2 = drivingDedendumLine.endSketchPoint
        drivingDedTextPoint = adsk.core.Point3D.create(
            (apex2Geom.x + dGx) / 2.0, (apex2Geom.y + dGy) / 2.0, 0.0)
        drivingDedDim = sketchDimensions.addDistanceDimension(
            drivingDedStart2, drivingDedEnd2, alignedOrientation, drivingDedTextPoint)
        drivingDedDimParameter = drivingDedDim.parameter
        drivingDedDimParameter.value = dedLen_cm
        pointD = drivingDedendumLine.endSketchPoint

        cGx = apex2Geom.x + pinionDedDir[0] * dedLen_cm
        cGy = apex2Geom.y + pinionDedDir[1] * dedLen_cm
        pinionDedendumLine = newLine(apex2Geom.x, apex2Geom.y, cGx, cGy)
        pinionDedStart = pinionDedendumLine.startSketchPoint
        geometricConstraints.addPerpendicular(pinionDedendumLine, pitchLine)
        geometricConstraints.addCoincident(pinionDedStart, apex2Point)
        pinionDedStart2 = pinionDedendumLine.startSketchPoint
        pinionDedEnd2 = pinionDedendumLine.endSketchPoint
        pinionDedTextPoint = adsk.core.Point3D.create(
            (apex2Geom.x + cGx) / 2.0, (apex2Geom.y + cGy) / 2.0, 0.0)
        pinionDedDim = sketchDimensions.addDistanceDimension(
            pinionDedStart2, pinionDedEnd2, alignedOrientation, pinionDedTextPoint)
        pinionDedDimParameter = pinionDedDim.parameter
        pinionDedDimParameter.value = dedLen_cm
        pointC = pinionDedendumLine.endSketchPoint

        # Root Axes: Apex->C and Apex->D.
        cGeom0 = pointC.geometry
        rootAxisPinion = newLine(apexX, apexY, cGeom0.x, cGeom0.y)
        rootAxisPinionStart = rootAxisPinion.startSketchPoint
        rootAxisPinionEnd = rootAxisPinion.endSketchPoint
        geometricConstraints.addCoincident(rootAxisPinionStart, apexSketchPoint)
        geometricConstraints.addCoincident(rootAxisPinionEnd, pointC)

        dGeom0 = pointD.geometry
        rootAxisDriving = newLine(apexX, apexY, dGeom0.x, dGeom0.y)
        rootAxisDrivingStart = rootAxisDriving.startSketchPoint
        rootAxisDrivingEnd = rootAxisDriving.endSketchPoint
        geometricConstraints.addCoincident(rootAxisDrivingStart, apexSketchPoint)
        geometricConstraints.addCoincident(rootAxisDrivingEnd, pointD)

        # The module-length extension chains.
        lenModule_cm = to_cm(module_mm)

        aGeom1 = pointA.geometry
        eX = aGeom1.x + pinionDirX * lenModule_cm
        eY = aGeom1.y + pinionDirY * lenModule_cm
        lineAE = newLine(aGeom1.x, aGeom1.y, eX, eY)
        lineAEStart = lineAE.startSketchPoint
        geometricConstraints.addCoincident(lineAEStart, pointA)
        geometricConstraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        eGeom0 = pointE.geometry
        lineCE = newLine(cGeom0.x, cGeom0.y, eGeom0.x, eGeom0.y)
        lineCEStart = lineCE.startSketchPoint
        lineCEEnd = lineCE.endSketchPoint
        geometricConstraints.addCoincident(lineCEStart, pointC)
        geometricConstraints.addCoincident(lineCEEnd, pointE)
        geometricConstraints.addPerpendicular(lineAE, lineCE)

        bGeom1 = pointB.geometry
        fX = bGeom1.x + drivingDirX * lenModule_cm
        fY = bGeom1.y + drivingDirY * lenModule_cm
        lineBF = newLine(bGeom1.x, bGeom1.y, fX, fY)
        lineBFStart = lineBF.startSketchPoint
        geometricConstraints.addCoincident(lineBFStart, pointB)
        geometricConstraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        fGeom0 = pointF.geometry
        lineDF = newLine(dGeom0.x, dGeom0.y, fGeom0.x, fGeom0.y)
        lineDFStart = lineDF.startSketchPoint
        lineDFEnd = lineDF.endSketchPoint
        geometricConstraints.addCoincident(lineDFStart, pointD)
        geometricConstraints.addCoincident(lineDFEnd, pointF)
        geometricConstraints.addPerpendicular(lineBF, lineDF)

        # From E, collinear with A->E (not Apex->A): one Module, end G. From C, collinear
        # with Apex2->C (the pinion dedendum line): one Module, end H. Twins F->I, D->J.
        eGeom1 = pointE.geometry
        gSeedX = eGeom1.x + pinionDirX * lenModule_cm
        gSeedY = eGeom1.y + pinionDirY * lenModule_cm
        lineEG = newLine(eGeom1.x, eGeom1.y, gSeedX, gSeedY)
        lineEGStart = lineEG.startSketchPoint
        geometricConstraints.addCoincident(lineEGStart, pointE)
        geometricConstraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        cGeom1 = pointC.geometry
        hSeedX = cGeom1.x + pinionDedDir[0] * lenModule_cm
        hSeedY = cGeom1.y + pinionDedDir[1] * lenModule_cm
        lineCH = newLine(cGeom1.x, cGeom1.y, hSeedX, hSeedY)
        lineCHStart = lineCH.startSketchPoint
        geometricConstraints.addCoincident(lineCHStart, pointC)
        geometricConstraints.addCollinear(lineCH, pinionDedendumLine)
        pointH = lineCH.endSketchPoint

        fGeom1 = pointF.geometry
        iSeedX = fGeom1.x + drivingDirX * lenModule_cm
        iSeedY = fGeom1.y + drivingDirY * lenModule_cm
        lineFI = newLine(fGeom1.x, fGeom1.y, iSeedX, iSeedY)
        lineFIStart = lineFI.startSketchPoint
        geometricConstraints.addCoincident(lineFIStart, pointF)
        geometricConstraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        dGeom1 = pointD.geometry
        jSeedX = dGeom1.x + drivingDedDir[0] * lenModule_cm
        jSeedY = dGeom1.y + drivingDedDir[1] * lenModule_cm
        lineDJ = newLine(dGeom1.x, dGeom1.y, jSeedX, jSeedY)
        lineDJStart = lineDJ.startSketchPoint
        geometricConstraints.addCoincident(lineDJStart, pointD)
        geometricConstraints.addCollinear(lineDJ, drivingDedendumLine)
        pointJ = lineDJ.endSketchPoint

        # The heel edges and their base-height offsets.
        gGeom0 = pointG.geometry
        hGeom0 = pointH.geometry
        lineGH = newLine(gGeom0.x, gGeom0.y, hGeom0.x, hGeom0.y)
        lineGHStart = lineGH.startSketchPoint
        lineGHEnd = lineGH.endSketchPoint
        geometricConstraints.addCoincident(lineGHStart, pointG)
        geometricConstraints.addCoincident(lineGHEnd, pointH)
        geometricConstraints.addPerpendicular(lineEG, lineGH)

        iGeom0 = pointI.geometry
        jGeom0 = pointJ.geometry
        lineIJ = newLine(iGeom0.x, iGeom0.y, jGeom0.x, jGeom0.y)
        lineIJStart = lineIJ.startSketchPoint
        lineIJEnd = lineIJ.endSketchPoint
        geometricConstraints.addCoincident(lineIJStart, pointI)
        geometricConstraints.addCoincident(lineIJEnd, pointJ)
        geometricConstraints.addPerpendicular(lineFI, lineIJ)

        bDropIjTextPoint = adsk.core.Point3D.create(
            (iGeom0.x + jGeom0.x) / 2.0, (iGeom0.y + jGeom0.y) / 2.0, 0.0)
        bDropIjDim = sketchDimensions.addOffsetDimension(dropFromB, lineIJ, bDropIjTextPoint)
        bDropIjDimParameter = bDropIjDim.parameter
        bDropIjDimParameter.value = drivingBaseHeight_cm

        aDropGhTextPoint = adsk.core.Point3D.create(
            (gGeom0.x + hGeom0.x) / 2.0, (gGeom0.y + hGeom0.y) / 2.0, 0.0)
        aDropGhDim = sketchDimensions.addOffsetDimension(dropFromA, lineGH, aDropGhTextPoint)
        aDropGhDimParameter = aDropGhDim.parameter
        aDropGhDimParameter.value = pinionBaseHeight_cm

        # Pin the figure.
        geometricConstraints.addCoincident(pointI, projectedCenter)

        # The tooth centres K (pinion) and L (driving): pinned onto the shaft axis AND the
        # dedendum line by two point-on-line coincidences, seeded at their exact intersection.
        gGeom1 = pointG.geometry
        kSeed = lineIntersect(
            (apexX, apexY), (pinionDirX, pinionDirY), (apex2Geom.x, apex2Geom.y), pinionDedDir)
        lineGK = newLine(gGeom1.x, gGeom1.y, kSeed[0], kSeed[1])
        lineGKStart = lineGK.startSketchPoint
        geometricConstraints.addCoincident(lineGKStart, pointG)
        pointK = lineGK.endSketchPoint
        geometricConstraints.addCoincident(pointK, pinionShaftAxis)
        geometricConstraints.addCoincident(pointK, pinionDedendumLine)

        kGeom0 = pointK.geometry
        lineCK = newLine(cGeom1.x, cGeom1.y, kGeom0.x, kGeom0.y)
        lineCKStart = lineCK.startSketchPoint
        lineCKEnd = lineCK.endSketchPoint
        geometricConstraints.addCoincident(lineCKStart, pointC)
        geometricConstraints.addCoincident(lineCKEnd, pointK)

        iGeom1 = pointI.geometry
        lSeed = lineIntersect(
            (apexX, apexY), (drivingDirX, drivingDirY), (apex2Geom.x, apex2Geom.y),
            drivingDedDir)
        lineIL = newLine(iGeom1.x, iGeom1.y, lSeed[0], lSeed[1])
        lineILStart = lineIL.startSketchPoint
        geometricConstraints.addCoincident(lineILStart, pointI)
        pointL = lineIL.endSketchPoint
        geometricConstraints.addCoincident(pointL, drivingShaftAxis)
        geometricConstraints.addCoincident(pointL, drivingDedendumLine)

        lGeom0 = pointL.geometry
        lineDL = newLine(dGeom1.x, dGeom1.y, lGeom0.x, lGeom0.y)
        lineDLStart = lineDL.startSketchPoint
        lineDLEnd = lineDL.endSketchPoint
        geometricConstraints.addCoincident(lineDLStart, pointD)
        geometricConstraints.addCoincident(lineDLEnd, pointL)

        # Tooth Spacing: K' (pinion) and L' (driving).
        toothSpacing_cm = self._toothSpacing_cm
        if toothSpacing_cm == 0:
            pointKPrime = pointK
            lineCKPrime = lineCK
        else:
            kGeom1 = pointK.geometry
            kpX = kGeom1.x + pinionDedDir[0] * toothSpacing_cm
            kpY = kGeom1.y + pinionDedDir[1] * toothSpacing_cm
            lineKKPrime = newLine(kGeom1.x, kGeom1.y, kpX, kpY)
            lineKKPrimeStart = lineKKPrime.startSketchPoint
            geometricConstraints.addCoincident(lineKKPrimeStart, pointK)
            pointKPrime = lineKKPrime.endSketchPoint
            geometricConstraints.addCoincident(pointKPrime, pinionDedendumLine)
            lineKKPrimeStart2 = lineKKPrime.startSketchPoint
            lineKKPrimeEnd2 = lineKKPrime.endSketchPoint
            kkpTextPoint = adsk.core.Point3D.create(
                (kGeom1.x + kpX) / 2.0, (kGeom1.y + kpY) / 2.0, 0.0)
            kkpDim = sketchDimensions.addDistanceDimension(
                lineKKPrimeStart2, lineKKPrimeEnd2, alignedOrientation, kkpTextPoint)
            kkpDimParameter = kkpDim.parameter
            kkpDimParameter.value = toothSpacing_cm

            cGeom2 = pointC.geometry
            kpGeom0 = pointKPrime.geometry
            lineCKPrime = newLine(cGeom2.x, cGeom2.y, kpGeom0.x, kpGeom0.y)
            lineCKPrimeStart = lineCKPrime.startSketchPoint
            lineCKPrimeEnd = lineCKPrime.endSketchPoint
            geometricConstraints.addCoincident(lineCKPrimeStart, pointC)
            geometricConstraints.addCoincident(lineCKPrimeEnd, pointKPrime)

        if toothSpacing_cm == 0:
            pointLPrime = pointL
            lineDLPrime = lineDL
        else:
            lGeom1 = pointL.geometry
            lpX = lGeom1.x + drivingDedDir[0] * toothSpacing_cm
            lpY = lGeom1.y + drivingDedDir[1] * toothSpacing_cm
            lineLLPrime = newLine(lGeom1.x, lGeom1.y, lpX, lpY)
            lineLLPrimeStart = lineLLPrime.startSketchPoint
            geometricConstraints.addCoincident(lineLLPrimeStart, pointL)
            pointLPrime = lineLLPrime.endSketchPoint
            geometricConstraints.addCoincident(pointLPrime, drivingDedendumLine)
            lineLLPrimeStart2 = lineLLPrime.startSketchPoint
            lineLLPrimeEnd2 = lineLLPrime.endSketchPoint
            llpTextPoint = adsk.core.Point3D.create(
                (lGeom1.x + lpX) / 2.0, (lGeom1.y + lpY) / 2.0, 0.0)
            llpDim = sketchDimensions.addDistanceDimension(
                lineLLPrimeStart2, lineLLPrimeEnd2, alignedOrientation, llpTextPoint)
            llpDimParameter = llpDim.parameter
            llpDimParameter.value = toothSpacing_cm

            dGeom2 = pointD.geometry
            lpGeom0 = pointLPrime.geometry
            lineDLPrime = newLine(dGeom2.x, dGeom2.y, lpGeom0.x, lpGeom0.y)
            lineDLPrimeStart = lineDLPrime.startSketchPoint
            lineDLPrimeEnd = lineDLPrime.endSketchPoint
            geometricConstraints.addCoincident(lineDLPrimeStart, pointD)
            geometricConstraints.addCoincident(lineDLPrimeEnd, pointLPrime)

        # Resolve the Maximum Face Width from SOLVED geometry ([PB-SOLVED-GEOMETRY]).
        aGeomSolved = pointA.geometry
        bGeomSolved = pointB.geometry
        cGeomSolved = pointC.geometry
        dGeomSolved = pointD.geometry
        hGeomSolved = pointH.geometry
        jGeomSolved = pointJ.geometry

        distA = perpDistToLine(
            aGeomSolved.x, aGeomSolved.y, cGeomSolved.x, cGeomSolved.y,
            hGeomSolved.x, hGeomSolved.y)
        distB = perpDistToLine(
            bGeomSolved.x, bGeomSolved.y, dGeomSolved.x, dGeomSolved.y,
            jGeomSolved.x, jGeomSolved.y)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        faceWidthInput_cm = self._faceWidth_cm
        coneDistance_cm = self._coneDistance_cm
        if faceWidthInput_cm == 0:
            faceWidthResolved_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)
        else:
            if faceWidthInput_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width must be <= {maxFaceWidth_cm} cm (got {faceWidthInput_cm})')
            faceWidthResolved_cm = faceWidthInput_cm
        self._faceWidthResolved_cm = faceWidthResolved_cm

        # Root Length, Toe Radius, Toe Radius Ceiling and Toe Limit, per gear.
        R_cm = to_cm(R_mm)
        dedendum_cm = to_cm(module_mm * 1.25)
        apexDed_cm = math.sqrt(R_cm * R_cm + dedendum_cm * dedendum_cm)
        rootLengthAtZero_cm = faceWidthResolved_cm * apexDed_cm / R_cm

        pinionR_cm = to_cm(PPD_mm) / 2.0
        drivingR_cm = to_cm(DPD_mm) / 2.0

        pinionToeCeiling_cm = (
            (pinionR_cm - dedendum_cm * math.cos(gamma_p)) * (1.0 - faceWidthResolved_cm / R_cm))
        drivingToeCeiling_cm = (
            (drivingR_cm - dedendum_cm * math.cos(gamma_g)) * (1.0 - faceWidthResolved_cm / R_cm))

        pinionToeRadiusInput_cm = self._pinionToeRadius_cm
        if pinionToeRadiusInput_cm == 0:
            pinionToeRadiusResolved_cm = pinionR_cm - faceWidthResolved_cm / math.sin(gamma_p)
        else:
            if pinionToeRadiusInput_cm >= pinionToeCeiling_cm:
                raise Exception(
                    f'Pinion Gear Toe Radius must be < {pinionToeCeiling_cm} cm '
                    f'(got {pinionToeRadiusInput_cm})')
            pinionToeRadiusResolved_cm = pinionToeRadiusInput_cm

        drivingToeRadiusInput_cm = self._drivingToeRadius_cm
        if drivingToeRadiusInput_cm == 0:
            drivingToeRadiusResolved_cm = drivingR_cm - faceWidthResolved_cm / math.sin(gamma_g)
        else:
            if drivingToeRadiusInput_cm >= drivingToeCeiling_cm:
                raise Exception(
                    f'Driving Gear Toe Radius must be < {drivingToeCeiling_cm} cm '
                    f'(got {drivingToeRadiusInput_cm})')
            drivingToeRadiusResolved_cm = drivingToeRadiusInput_cm

        pinionGammaRoot = gamma_p - math.atan(dedendum_cm / R_cm)
        pinionToeLimit_cm = apexDed_cm - pinionToeRadiusResolved_cm / math.sin(pinionGammaRoot)
        drivingGammaRoot = gamma_g - math.atan(dedendum_cm / R_cm)
        drivingToeLimit_cm = apexDed_cm - drivingToeRadiusResolved_cm / math.sin(drivingGammaRoot)

        toeExtension = self._toeExtension
        toeLimitMin_cm = min(pinionToeLimit_cm, drivingToeLimit_cm)
        if toeExtension > 0 and toeLimitMin_cm < rootLengthAtZero_cm:
            if pinionToeLimit_cm <= drivingToeLimit_cm:
                raise Exception(
                    f'Pinion Gear: Toe Extension needs a Toe Limit past the Toe Extension 0 '
                    f'root length (Toe Radius Ceiling {pinionToeCeiling_cm} cm)')
            else:
                raise Exception(
                    f'Driving Gear: Toe Extension needs a Toe Limit past the Toe Extension 0 '
                    f'root length (Toe Radius Ceiling {drivingToeCeiling_cm} cm)')

        rootLengthTarget_cm = (
            rootLengthAtZero_cm
            + (toeExtension / 100.0) * 0.99 * (toeLimitMin_cm - rootLengthAtZero_cm))

        # The toe lines M->N (pinion) and O->P (driving).
        cGeomFinal = pointC.geometry
        hGeomFinal = pointH.geometry
        mSeedX = (apexX + cGeomFinal.x) / 2.0
        mSeedY = (apexY + cGeomFinal.y) / 2.0
        chDirX = hGeomFinal.x - cGeomFinal.x
        chDirY = hGeomFinal.y - cGeomFinal.y
        chLen = math.hypot(chDirX, chDirY)
        chDirX /= chLen
        chDirY /= chLen
        nSeedX = mSeedX + chDirX * rootLengthTarget_cm
        nSeedY = mSeedY + chDirY * rootLengthTarget_cm

        lineMN = newLine(mSeedX, mSeedY, nSeedX, nSeedY)
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        geometricConstraints.addCoincident(pointM, rootAxisPinion)
        geometricConstraints.addParallel(lineMN, lineCH)
        mnTextPoint = adsk.core.Point3D.create(
            (mSeedX + cGeomFinal.x) / 2.0, (mSeedY + cGeomFinal.y) / 2.0, 0.0)
        mnOffsetDim = sketchDimensions.addOffsetDimension(lineCH, lineMN, mnTextPoint)
        mnOffsetDimParameter = mnOffsetDim.parameter
        mnOffsetDimParameter.value = rootLengthTarget_cm * R_cm / apexDed_cm

        mGeomFinal = pointM.geometry
        lineMC = newLine(mGeomFinal.x, mGeomFinal.y, cGeomFinal.x, cGeomFinal.y)
        lineMCStart = lineMC.startSketchPoint
        lineMCEnd = lineMC.endSketchPoint
        geometricConstraints.addCoincident(lineMCStart, pointM)
        geometricConstraints.addCoincident(lineMCEnd, pointC)

        # The front face A'->N.
        nGeomFinal = pointN.geometry
        toNx = nGeomFinal.x - apexX
        toNy = nGeomFinal.y - apexY
        projLenPinion = toNx * pinionDirX + toNy * pinionDirY
        aPrimeSeedX = apexX + pinionDirX * projLenPinion
        aPrimeSeedY = apexY + pinionDirY * projLenPinion

        lineNAPrime = newLine(nGeomFinal.x, nGeomFinal.y, aPrimeSeedX, aPrimeSeedY)
        lineNAPrimeStart = lineNAPrime.startSketchPoint
        geometricConstraints.addCoincident(lineNAPrimeStart, pointN)
        pointAPrime = lineNAPrime.endSketchPoint
        geometricConstraints.addCoincident(pointAPrime, pinionShaftAxis)
        geometricConstraints.addPerpendicular(lineNAPrime, pinionShaftAxis)
        lineNAPrimeStart2 = lineNAPrime.startSketchPoint
        lineNAPrimeEnd2 = lineNAPrime.endSketchPoint
        naPrimeTextPoint = adsk.core.Point3D.create(
            (nGeomFinal.x + aPrimeSeedX) / 2.0, (nGeomFinal.y + aPrimeSeedY) / 2.0 - 0.3, 0.0)
        naPrimeDim = sketchDimensions.addDistanceDimension(
            lineNAPrimeStart2, lineNAPrimeEnd2, alignedOrientation, naPrimeTextPoint)
        naPrimeDimParameter = naPrimeDim.parameter
        naPrimeDimParameter.value = pinionToeRadiusResolved_cm

        aPrimeGeomFinal = pointAPrime.geometry
        gGeomFinal = pointG.geometry
        lineAPrimeG = newLine(aPrimeGeomFinal.x, aPrimeGeomFinal.y, gGeomFinal.x, gGeomFinal.y)
        lineAPrimeGStart = lineAPrimeG.startSketchPoint
        lineAPrimeGEnd = lineAPrimeG.endSketchPoint
        geometricConstraints.addCoincident(lineAPrimeGStart, pointAPrime)
        geometricConstraints.addCoincident(lineAPrimeGEnd, pointG)

        # The driving side: O->P mirrors M->N, B'->P mirrors A'->N.
        dGeomFinal = pointD.geometry
        jGeomFinal = pointJ.geometry
        oSeedX = (apexX + dGeomFinal.x) / 2.0
        oSeedY = (apexY + dGeomFinal.y) / 2.0
        djDirX = jGeomFinal.x - dGeomFinal.x
        djDirY = jGeomFinal.y - dGeomFinal.y
        djLen = math.hypot(djDirX, djDirY)
        djDirX /= djLen
        djDirY /= djLen
        pSeedX = oSeedX + djDirX * rootLengthTarget_cm
        pSeedY = oSeedY + djDirY * rootLengthTarget_cm

        lineOP = newLine(oSeedX, oSeedY, pSeedX, pSeedY)
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        geometricConstraints.addCoincident(pointO, rootAxisDriving)
        geometricConstraints.addParallel(lineOP, lineDJ)
        opTextPoint = adsk.core.Point3D.create(
            (oSeedX + dGeomFinal.x) / 2.0, (oSeedY + dGeomFinal.y) / 2.0, 0.0)
        opOffsetDim = sketchDimensions.addOffsetDimension(lineDJ, lineOP, opTextPoint)
        opOffsetDimParameter = opOffsetDim.parameter
        opOffsetDimParameter.value = rootLengthTarget_cm * R_cm / apexDed_cm

        oGeomFinal = pointO.geometry
        lineOD = newLine(oGeomFinal.x, oGeomFinal.y, dGeomFinal.x, dGeomFinal.y)
        lineODStart = lineOD.startSketchPoint
        lineODEnd = lineOD.endSketchPoint
        geometricConstraints.addCoincident(lineODStart, pointO)
        geometricConstraints.addCoincident(lineODEnd, pointD)

        pGeomFinal = pointP.geometry
        toPx = pGeomFinal.x - apexX
        toPy = pGeomFinal.y - apexY
        projLenDriving = toPx * drivingDirX + toPy * drivingDirY
        bPrimeSeedX = apexX + drivingDirX * projLenDriving
        bPrimeSeedY = apexY + drivingDirY * projLenDriving

        lineforPBPrime = newLine(pGeomFinal.x, pGeomFinal.y, bPrimeSeedX, bPrimeSeedY)
        linePBPrimeStart = lineforPBPrime.startSketchPoint
        geometricConstraints.addCoincident(linePBPrimeStart, pointP)
        pointBPrime = lineforPBPrime.endSketchPoint
        geometricConstraints.addCoincident(pointBPrime, drivingShaftAxis)
        geometricConstraints.addPerpendicular(lineforPBPrime, drivingShaftAxis)
        linePBPrimeStart2 = lineforPBPrime.startSketchPoint
        linePBPrimeEnd2 = lineforPBPrime.endSketchPoint
        pbPrimeTextPoint = adsk.core.Point3D.create(
            (pGeomFinal.x + bPrimeSeedX) / 2.0, (pGeomFinal.y + bPrimeSeedY) / 2.0 - 0.3, 0.0)
        pbPrimeDim = sketchDimensions.addDistanceDimension(
            linePBPrimeStart2, linePBPrimeEnd2, alignedOrientation, pbPrimeTextPoint)
        pbPrimeDimParameter = pbPrimeDim.parameter
        pbPrimeDimParameter.value = drivingToeRadiusResolved_cm

        bPrimeGeomFinal = pointBPrime.geometry
        iGeomFinal = pointI.geometry
        lineBPrimeI = newLine(
            bPrimeGeomFinal.x, bPrimeGeomFinal.y, iGeomFinal.x, iGeomFinal.y)
        lineBPrimeIStart = lineBPrimeI.startSketchPoint
        lineBPrimeIEnd = lineBPrimeI.endSketchPoint
        geometricConstraints.addCoincident(lineBPrimeIStart, pointBPrime)
        geometricConstraints.addCoincident(lineBPrimeIEnd, pointI)

        # Gate the sketch.
        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        self._coneDistance_cm = coneDistance_cm
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._apexSketchPoint = apexSketchPoint

        pinionCtx = {
            'gamma': gamma_p,
            'toothCentreRefLine': lineCKPrime,
            'toothCentrePoint': pointKPrime,
            'hexagon': [pointAPrime, pointG, pointH, pointC, pointM, pointN],
            'toeEdge': (pointM, pointN),
            'heelEdge': (pointC, pointH),
            'toeCone': pointM,
            'heelCone': pointC,
        }
        drivingCtx = {
            'gamma': gamma_g,
            'toothCentreRefLine': lineDLPrime,
            'toothCentrePoint': pointLPrime,
            'hexagon': [pointBPrime, pointI, pointJ, pointD, pointO, pointP],
            'toeEdge': (pointO, pointP),
            'heelEdge': (pointD, pointJ),
            'toeCone': pointO,
            'heelCone': pointD,
        }
        return pinionCtx, drivingCtx

    # ------------------------------------------------------------------------------------
    # Steps 7-25: run once per gear (pinion first, driving second).
    # ------------------------------------------------------------------------------------

    def _createGearBody(self, gearLabel: str, module_mm: float, teethNumber: int, gamma: float,
                        toothCentreRefLine: adsk.fusion.SketchLine,
                        toothCentrePoint: adsk.fusion.SketchPoint, hexagonSrcPoints,
                        toeEdgeStart: adsk.fusion.SketchPoint,
                        toeEdgeEnd: adsk.fusion.SketchPoint,
                        heelEdgeStart: adsk.fusion.SketchPoint,
                        heelEdgeEnd: adsk.fusion.SketchPoint,
                        toeConeWorldPoint: adsk.fusion.SketchPoint,
                        heelConeWorldPoint: adsk.fusion.SketchPoint, meshAngle_rad: float):
        designComponent: adsk.fusion.Component = self.designComponent
        gearProfilesPlane: adsk.fusion.ConstructionPlane = self._gearProfilesPlane

        # Step 7: the per-gear tooth plane, and the virtual (back-cone) tooth number.
        toothPlane: adsk.fusion.ConstructionPlane = solids.plane_by_angle(
            designComponent, toothCentreRefLine, gearProfilesPlane, 90)
        toothPlane.name = f'{gearLabel} Plane'

        pitchRadius_mm = (module_mm * teethNumber) / 2.0
        virtualPitchRadius_mm = pitchRadius_mm / math.cos(gamma)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / module_mm))

        # Step 8: the virtual spur tooth sketch.
        toothSketch, embedded = self._buildVirtualSpurProfile(
            designComponent, toothPlane, gearLabel, module_mm, virtualTeeth, toothCentrePoint)
        toothSketch: adsk.fusion.Sketch

        # Step 9: the per-gear tooth axis.
        self._buildToothAxis(designComponent, gearProfilesPlane, toothCentreRefLine, gearLabel)

        # Step 10: the per-gear component.
        bevelComponent: adsk.fusion.Component = self.bevelComponent
        gearOccurrence = self._createGearComponent(bevelComponent, gearLabel)
        gearOccurrence: adsk.fusion.Occurrence

        # Step 11: the per-gear Profile sketch.
        profileSketch, shaftAxisEdge = self._buildProfileSketch(
            designComponent, gearProfilesPlane, gearLabel, hexagonSrcPoints)
        profileSketch: adsk.fusion.Sketch
        shaftAxisEdge: adsk.fusion.SketchLine

        # Step 12: revolve the Gear Body.
        gearBody = self._revolveGearBody(designComponent, profileSketch, shaftAxisEdge, gearLabel)
        gearBody: adsk.fusion.BRepBody

        # Step 13: loft the uncut Tooth Body.
        apexSketchPoint: adsk.fusion.SketchPoint = self._apexSketchPoint
        toothBody = self._loftUncutTooth(
            designComponent, apexSketchPoint, toothSketch, embedded, gearLabel)
        toothBody: adsk.fusion.BRepBody

        # Steps 14-19: the tooth-body hook.
        apexWorld: adsk.core.Point3D = apexSketchPoint.worldGeometry

        toeStartWorld: adsk.core.Point3D = toeEdgeStart.worldGeometry
        toeEndWorld: adsk.core.Point3D = toeEdgeEnd.worldGeometry
        toeVec: adsk.core.Vector3D = toeStartWorld.vectorTo(toeEndWorld)
        toeMid: adsk.core.Point3D = solids.combine_point(toeStartWorld, 0.5, toeVec)

        heelStartWorld: adsk.core.Point3D = heelEdgeStart.worldGeometry
        heelEndWorld: adsk.core.Point3D = heelEdgeEnd.worldGeometry
        heelVec: adsk.core.Vector3D = heelStartWorld.vectorTo(heelEndWorld)
        heelMid: adsk.core.Point3D = solids.combine_point(heelStartWorld, 0.5, heelVec)

        toeConeWorld: adsk.core.Point3D = toeConeWorldPoint.worldGeometry
        heelConeWorld: adsk.core.Point3D = heelConeWorldPoint.worldGeometry

        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
            toeMid, heelMid, toeConeWorld, heelConeWorld, toothPlane, gearLabel, teethNumber,
            gamma)
        toothBody: adsk.fusion.BRepBody

        # Step 20: circular-pattern the tooth.
        toolCollection = self._circularPatternTooth(
            designComponent, toothBody, shaftAxisEdge, teethNumber, gearLabel)
        toolCollection: adsk.core.ObjectCollection

        # Step 21: Combine-Join the teeth into the Gear Body.
        self._combineJoinTeeth(designComponent, gearBody, toolCollection, gearLabel)

        # Steps 22-23: the bore, when enabled.
        if self._boreEnable:
            boreInput_cm = self._drivingBore_cm if gearLabel == 'Driving' else self._pinionBore_cm
            pitchDia_cm = to_cm(module_mm * teethNumber)
            if boreInput_cm == 0:
                boreDiameter_cm = pitchDia_cm / 4.0
            else:
                boreDiameter_cm = boreInput_cm
            self._buildAndCutBore(
                designComponent, gearBody, shaftAxisEdge, boreDiameter_cm, gearLabel)

        # Step 24: rotate the driving gear (and the pinion's zero-phase no-op) into mesh.
        solids.rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, meshAngle_rad)

        # Step 25: relocate the finished body into the {gearLabel} Gear component.
        gearBody.moveToComponent(gearOccurrence)

        return gearBody

    # ------------------------------------------------------------------------------------
    # Step 8: the virtual spur tooth sketch, borrowing the spur tooth generator.
    # ------------------------------------------------------------------------------------

    def _buildVirtualSpurProfile(self, designComponent: adsk.fusion.Component,
                                 toothPlane: adsk.fusion.ConstructionPlane, gearLabel: str,
                                 module_mm: float, virtualTeeth: int,
                                 toothCentrePoint: adsk.fusion.SketchPoint):
        sketches: adsk.fusion.Sketches = designComponent.sketches
        toothSketch: adsk.fusion.Sketch = sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'

        proxy = VirtualSpurProxy(module_mm=module_mm, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCentrePoint, angle=math.radians(180))

        embedded = proxy._lastToothEmbedded

        if not toothSketch.isFullyConstrained:
            futil.log(
                f'{gearLabel} Tooth sketch is not fully constrained '
                f'(exempt: the four circle labels hold DOF, [PB-TEXT-HOLDS-DOF])')

        return toothSketch, embedded

    # ------------------------------------------------------------------------------------
    # Step 9: the per-gear tooth axis.
    # ------------------------------------------------------------------------------------

    def _buildToothAxis(self, designComponent: adsk.fusion.Component,
                        gearProfilesPlane: adsk.fusion.ConstructionPlane,
                        toothCentreRefLine: adsk.fusion.SketchLine, gearLabel: str):
        constructionPlanes: adsk.fusion.ConstructionPlanes = designComponent.constructionPlanes
        helperPlaneInput: adsk.fusion.ConstructionPlaneInput = constructionPlanes.createInput()
        onePointZero = adsk.core.ValueInput.createByReal(1.0)
        helperPlaneInput.setByDistanceOnPath(toothCentreRefLine, onePointZero)
        helperPlane: adsk.fusion.ConstructionPlane = constructionPlanes.add(helperPlaneInput)

        constructionAxes: adsk.fusion.ConstructionAxes = designComponent.constructionAxes
        axisInput: adsk.fusion.ConstructionAxisInput = constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis: adsk.fusion.ConstructionAxis = constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'
        return toothAxis

    # ------------------------------------------------------------------------------------
    # Step 10: the per-gear component, a child of Bevel Gear.
    # ------------------------------------------------------------------------------------

    def _createGearComponent(self, bevelComponent: adsk.fusion.Component, gearLabel: str):
        occurrences: adsk.fusion.Occurrences = bevelComponent.occurrences
        identityMatrix = adsk.core.Matrix3D.create()
        gearOccurrence: adsk.fusion.Occurrence = occurrences.addNewComponent(identityMatrix)
        gearComponent: adsk.fusion.Component = gearOccurrence.component
        gearComponent.name = f'{gearLabel} Gear'
        return gearOccurrence

    # ------------------------------------------------------------------------------------
    # Step 11: the per-gear Profile sketch, on fixed vertices (recreate-share-fix).
    # ------------------------------------------------------------------------------------

    def _buildProfileSketch(self, designComponent: adsk.fusion.Component,
                            gearProfilesPlane: adsk.fusion.ConstructionPlane, gearLabel: str,
                            hexagonSrcPoints):
        sketches: adsk.fusion.Sketches = designComponent.sketches
        profileSketch: adsk.fusion.Sketch = sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'

        sketchPoints: adsk.fusion.SketchPoints = profileSketch.sketchPoints
        newPoints = []
        for srcPoint in hexagonSrcPoints:
            srcPoint: adsk.fusion.SketchPoint
            worldGeom: adsk.core.Point3D = srcPoint.worldGeometry
            localGeom: adsk.core.Point3D = profileSketch.modelToSketchSpace(worldGeom)
            newPoint: adsk.fusion.SketchPoint = sketchPoints.add(localGeom)
            newPoints.append(newPoint)

        sketchCurves: adsk.fusion.SketchCurves = profileSketch.sketchCurves
        sketchLines: adsk.fusion.SketchLines = sketchCurves.sketchLines
        edges = []
        vertexCount = len(newPoints)
        for i in range(vertexCount):
            startPoint = newPoints[i]
            endPoint = newPoints[(i + 1) % vertexCount]
            edge = sketchLines.addByTwoPoints(startPoint, endPoint)
            edges.append(edge)

        for edge in edges:
            edgeStart = edge.startSketchPoint
            edgeEnd = edge.endSketchPoint
            edgeStart.isFixed = True
            edgeEnd.isFixed = True

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Profile sketch is not fully constrained')

        shaftAxisEdge = edges[0]
        return profileSketch, shaftAxisEdge

    # ------------------------------------------------------------------------------------
    # Step 12: revolve the Gear Body.
    # ------------------------------------------------------------------------------------

    def _revolveGearBody(self, designComponent: adsk.fusion.Component,
                         profileSketch: adsk.fusion.Sketch,
                         shaftAxisEdge: adsk.fusion.SketchLine, gearLabel: str):
        profiles: adsk.fusion.Profiles = profileSketch.profiles
        profile: adsk.fusion.Profile = profiles.item(0)

        features: adsk.fusion.Features = designComponent.features
        revolveFeatures: adsk.fusion.RevolveFeatures = features.revolveFeatures
        newBodyOp = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        revolveInput: adsk.fusion.RevolveFeatureInput = revolveFeatures.createInput(
            profile, shaftAxisEdge, newBodyOp)
        fullTurn = adsk.core.ValueInput.createByString('360 deg')
        revolveInput.setAngleExtent(False, fullTurn)
        revolveFeature: adsk.fusion.RevolveFeature = revolveFeatures.add(revolveInput)

        bodies: adsk.fusion.BRepBodies = revolveFeature.bodies
        gearBody: adsk.fusion.BRepBody = bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'
        return gearBody

    # ------------------------------------------------------------------------------------
    # Step 13: loft the uncut Tooth Body from the apex point to the tooth profile.
    # ------------------------------------------------------------------------------------

    def _loftUncutTooth(self, designComponent: adsk.fusion.Component,
                        apexSketchPoint: adsk.fusion.SketchPoint, toothSketch: adsk.fusion.Sketch,
                        embedded: bool, gearLabel: str):
        wantLines = 0 if embedded else 2
        toothProfile: adsk.fusion.Profile = find_profile_by_curve_counts(
            toothSketch, nurbs=2, arcs=2, lines=wantLines)

        features: adsk.fusion.Features = designComponent.features
        loftFeatures: adsk.fusion.LoftFeatures = features.loftFeatures
        newBodyOp = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        loftInput: adsk.fusion.LoftFeatureInput = loftFeatures.createInput(newBodyOp)
        loftSections: adsk.fusion.LoftSections = loftInput.loftSections
        loftSections.add(apexSketchPoint)
        loftSections.add(toothProfile)
        loftFeature: adsk.fusion.LoftFeature = loftFeatures.add(loftInput)

        bodies: adsk.fusion.BRepBodies = loftFeature.bodies
        toothBody: adsk.fusion.BRepBody = bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth Body'
        return toothBody

    # ------------------------------------------------------------------------------------
    # Steps 14-19: the tooth-body hook. Steps 15-18 (E-I) are inlined here so the
    # slabHeelFace/slabToeFace helpers can close over this call's apexWorld/coneVec.
    # ------------------------------------------------------------------------------------

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                            toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                            shaftAxisEdge: adsk.fusion.SketchLine, apexWorld: adsk.core.Point3D,
                            apexSketchPoint: adsk.fusion.SketchPoint, toeMid: adsk.core.Point3D,
                            heelMid: adsk.core.Point3D, toeConeWorld: adsk.core.Point3D,
                            heelConeWorld: adsk.core.Point3D,
                            parentToothPlane: adsk.fusion.ConstructionPlane, gearLabel: str,
                            teethNumber: int, gamma: float):
        if self._spiralAngle_rad <= 0:
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        # A. The frame.
        shaftStartPoint: adsk.fusion.SketchPoint = shaftAxisEdge.startSketchPoint
        shaftEndPoint: adsk.fusion.SketchPoint = shaftAxisEdge.endSketchPoint
        shaftStartWorld: adsk.core.Point3D = shaftStartPoint.worldGeometry
        shaftEndWorld: adsk.core.Point3D = shaftEndPoint.worldGeometry
        axisDir: adsk.core.Vector3D = shaftStartWorld.vectorTo(shaftEndWorld)
        axisDir.normalize()

        # The heel MUST be the outer end.
        heelDist0 = apexWorld.distanceTo(heelMid)
        toeDist0 = apexWorld.distanceTo(toeMid)
        if heelDist0 < toeDist0:
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec: adsk.core.Vector3D = apexWorld.vectorTo(heelConeWorld)
        coneVec.normalize()
        circumferentialVec: adsk.core.Vector3D = axisDir.crossProduct(coneVec)
        circumferentialVec.normalize()
        tangentNormal: adsk.core.Vector3D = coneVec.crossProduct(circumferentialVec)
        tangentNormal.normalize()

        rToe = distAlong(toeMid, apexWorld, coneVec)
        rHeel = distAlong(heelMid, apexWorld, coneVec)
        rMean = (rToe + rHeel) / 2.0
        span = rHeel - rToe

        # B. The cutter arc's geometry.
        cutterRadius_cm = self._cutterRadius_cm
        if cutterRadius_cm == 0:
            cutterRadius_cm = rMean

        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign

        psi = self._spiralAngle_rad
        cx = rMean - cutterRadius_cm * math.sin(psi)
        cy = handSign * cutterRadius_cm * math.cos(psi)

        rLo = rToe - 0.06 * span
        rHi = rHeel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(rLo, cx, cy, cutterRadius_cm, rMean, 0.0)
        heel2d = solids.circle_intersect_nearest(rHi, cx, cy, cutterRadius_cm, rMean, 0.0)

        # C. The sketches.
        sketches: adsk.fusion.Sketches = designComponent.sketches
        gearProfilesPlane: adsk.fusion.ConstructionPlane = self._gearProfilesPlane
        coneElementSketch: adsk.fusion.Sketch = sketches.add(gearProfilesPlane)
        coneElementSketch.name = f'{gearLabel} Cone Element'

        coneEndWorld: adsk.core.Point3D = solids.combine_point(apexWorld, rHeel, coneVec)
        coneElementSketchCurves: adsk.fusion.SketchCurves = coneElementSketch.sketchCurves
        coneElementSketchLines: adsk.fusion.SketchLines = coneElementSketchCurves.sketchLines
        coneElementLine: adsk.fusion.SketchLine = coneElementSketchLines.addByTwoPoints(
            apexWorld, coneEndWorld)

        tracePlane: adsk.fusion.ConstructionPlane = solids.plane_by_angle(
            designComponent, coneElementLine, gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        traceSketch: adsk.fusion.Sketch = sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        traceSketchCurves: adsk.fusion.SketchCurves = traceSketch.sketchCurves
        traceSketchCircles: adsk.fusion.SketchCircles = traceSketchCurves.sketchCircles
        cutterCenterWorld: adsk.core.Point3D = solids.combine_point(
            apexWorld, cx, coneVec, cy, circumferentialVec)
        cutterCircle: adsk.fusion.SketchCircle = traceSketchCircles.addByCenterRadius(
            cutterCenterWorld, cutterRadius_cm)
        cutterCircle.isConstruction = True
        cutterCircleCenter: adsk.fusion.SketchPoint = cutterCircle.centerSketchPoint
        cutterCircleCenter.isFixed = True

        traceSketchDimensions: adsk.fusion.SketchDimensions = traceSketch.sketchDimensions
        circleDimTextWorld: adsk.core.Point3D = solids.combine_point(
            apexWorld, cx + cutterRadius_cm, coneVec, cy, circumferentialVec)
        diameterDim = traceSketchDimensions.addDiameterDimension(cutterCircle, circleDimTextWorld)
        diameterDimParameter = diameterDim.parameter
        diameterDimParameter.value = 2.0 * cutterRadius_cm

        toePoint3d: adsk.core.Point3D = solids.combine_point(
            apexWorld, toe2d[0], coneVec, toe2d[1], circumferentialVec)
        meanPoint3d: adsk.core.Point3D = solids.combine_point(
            apexWorld, rMean, coneVec, 0.0, circumferentialVec)
        heelPoint3d: adsk.core.Point3D = solids.combine_point(
            apexWorld, heel2d[0], coneVec, heel2d[1], circumferentialVec)
        traceSketchArcs: adsk.fusion.SketchArcs = traceSketchCurves.sketchArcs
        traceArc: adsk.fusion.SketchArc = traceSketchArcs.addByThreePoints(
            toePoint3d, meanPoint3d, heelPoint3d)

        traceGeometricConstraints: adsk.fusion.GeometricConstraints = (
            traceSketch.geometricConstraints)
        traceArcCenter: adsk.fusion.SketchPoint = traceArc.centerSketchPoint
        traceGeometricConstraints.addCoincident(traceArcCenter, cutterCircleCenter)
        radialDim = traceSketchDimensions.addRadialDimension(traceArc, meanPoint3d)
        radialDimParameter = radialDim.parameter
        radialDimParameter.value = cutterRadius_cm

        # E. Slice into cross-section slabs.
        planeGeometry: adsk.core.Plane = parentToothPlane.geometry
        planeOrigin: adsk.core.Point3D = planeGeometry.origin
        planeNormal: adsk.core.Vector3D = planeGeometry.normal
        apexToOrigin: adsk.core.Vector3D = apexWorld.vectorTo(planeOrigin)
        sliceTest = apexToOrigin.dotProduct(planeNormal)
        sliceSign = 1.0 if sliceTest >= 0 else -1.0

        offsets = [sliceSign * (k + 1) * span / 6.0 for k in range(8)]
        pieces = solids.slice_body_by_offset_planes(
            designComponent, toothBody, parentToothPlane, offsets)

        if len(pieces) < 2:
            sliceSign = -sliceSign
            offsets = [sliceSign * (k + 1) * span / 6.0 for k in range(8)]
            pieces = solids.slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)

        if len(pieces) < 2:
            raise Exception(
                f'{gearLabel}: tooth slice produced {len(pieces)} piece(s), expected >=2 '
                f'(span={span}, sign tried={sliceSign})')

        def pieceCentroidDist(body):
            physicalProperties = body.physicalProperties
            centerOfMass = physicalProperties.centerOfMass
            return distAlong(centerOfMass, apexWorld, coneVec)

        pieces.sort(key=pieceCentroidDist)

        # F. Drop the apex-most scrap.
        scrap = pieces[0]
        segments = pieces[1:]
        features: adsk.fusion.Features = designComponent.features
        removeFeatures: adsk.fusion.RemoveFeatures = features.removeFeatures
        removeFeatures.add(scrap)

        if len(segments) == 0:
            raise Exception(f'{gearLabel}: no segments left after dropping the apex scrap')

        # slabHeelFace/slabToeFace: the all-faces-by-centroid rule, used in steps 16-18.
        def slabHeelFace(body: adsk.fusion.BRepBody) -> adsk.fusion.BRepFace:
            bestFace = None
            bestDist = None
            bodyFaces: adsk.fusion.BRepFaces = body.faces
            for face in bodyFaces:
                face: adsk.fusion.BRepFace
                centroid: adsk.core.Point3D = face.centroid
                dist = distAlong(centroid, apexWorld, coneVec)
                if bestDist is None or dist > bestDist:
                    bestDist = dist
                    bestFace = face
            return bestFace

        def slabToeFace(body: adsk.fusion.BRepBody) -> adsk.fusion.BRepFace:
            bestFace = None
            bestDist = None
            bodyFaces: adsk.fusion.BRepFaces = body.faces
            for face in bodyFaces:
                face: adsk.fusion.BRepFace
                centroid: adsk.core.Point3D = face.centroid
                dist = distAlong(centroid, apexWorld, coneVec)
                if bestDist is None or dist < bestDist:
                    bestDist = dist
                    bestFace = face
            return bestFace

        # G. Twist each slab about the shaft axis.
        phiCrownToe = math.atan2(toe2d[1], toe2d[0])
        phiCrownHeel = math.atan2(heel2d[1], heel2d[0])
        phiCrown = phiCrownHeel - phiCrownToe
        total = abs(phiCrown) / math.sin(gamma)

        moveFeatures: adsk.fusion.MoveFeatures = features.moveFeatures
        for segment in segments:
            heelFace: adsk.fusion.BRepFace = slabHeelFace(segment)
            heelFaceCentroid: adsk.core.Point3D = heelFace.centroid
            rHeelFace = distAlong(heelFaceCentroid, apexWorld, coneVec)
            ang = -handSign * total * (rMean - rHeelFace) / span

            bodyCollection: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
            bodyCollection.add(segment)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            moveInput: adsk.fusion.MoveFeatureInput = moveFeatures.createInput2(bodyCollection)
            moveInput.defineAsFreeMove(matrix)
            moveFeatures.add(moveInput)

        # H. Crown the slabs lengthwise. scaleFeatures is the one exception to
        # never-activate ([PB-CONSTRUCTION-NEEDS-ACTIVE]).
        designOccurrence: adsk.fusion.Occurrence = self.designOccurrence
        designOccurrence.activate()
        try:
            segmentsByHeelDist = sorted(
                segments,
                key=lambda body: distAlong(slabHeelFace(body).centroid, apexWorld, coneVec))
            scaleFeatures: adsk.fusion.ScaleFeatures = features.scaleFeatures
            for segment in segmentsByHeelDist[:-1]:
                heelFace: adsk.fusion.BRepFace = slabHeelFace(segment)
                heelFaceCentroid: adsk.core.Point3D = heelFace.centroid
                rHeelFaceNow = distAlong(heelFaceCentroid, apexWorld, coneVec)
                u = (rHeel - rHeelFaceNow) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor} is not positive (u={u})')

                heelVertices: adsk.fusion.BRepVertices = heelFace.vertices
                candidates = []
                for vertex in heelVertices:
                    vertex: adsk.fusion.BRepVertex
                    vertexGeometry: adsk.core.Point3D = vertex.geometry
                    apexToVertex: adsk.core.Vector3D = apexWorld.vectorTo(vertexGeometry)
                    alongAxis = apexToVertex.dotProduct(axisDir)
                    axisPoint: adsk.core.Point3D = solids.combine_point(
                        apexWorld, alongAxis, axisDir)
                    offVec: adsk.core.Vector3D = axisPoint.vectorTo(vertexGeometry)
                    perpDist = offVec.length
                    candidates.append((perpDist, vertexGeometry))
                candidates.sort(key=lambda entry: entry[0])
                rootVertexA: adsk.core.Point3D = candidates[0][1]
                rootVertexB: adsk.core.Point3D = candidates[1][1]
                rootAToB: adsk.core.Vector3D = rootVertexA.vectorTo(rootVertexB)
                rootMidWorld: adsk.core.Point3D = solids.combine_point(rootVertexA, 0.5, rootAToB)

                sketches2: adsk.fusion.Sketches = designComponent.sketches
                heelFaceSketch: adsk.fusion.Sketch = sketches2.add(heelFace)
                heelFaceSketchPoints: adsk.fusion.SketchPoints = heelFaceSketch.sketchPoints
                rootMidLocal: adsk.core.Point3D = heelFaceSketch.modelToSketchSpace(rootMidWorld)
                baseSketchPoint: adsk.fusion.SketchPoint = heelFaceSketchPoints.add(rootMidLocal)

                segmentCollection: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
                segmentCollection.add(segment)
                scaleFactorValue = adsk.core.ValueInput.createByReal(factor)
                scaleInput: adsk.fusion.ScaleFeatureInput = scaleFeatures.createInput(
                    segmentCollection, baseSketchPoint, scaleFactorValue)
                scaleFeatures.add(scaleInput)
        finally:
            design: adsk.fusion.Design = self.design
            design.activateRootComponent()

        # I. Re-sort by post-twist-and-crown heel-face cone distance, then loft.
        orderedSegments = sorted(
            segments,
            key=lambda body: distAlong(slabHeelFace(body).centroid, apexWorld, coneVec))

        loftFeatures: adsk.fusion.LoftFeatures = features.loftFeatures
        newBodyOp = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        loftInput: adsk.fusion.LoftFeatureInput = loftFeatures.createInput(newBodyOp)
        loftSections: adsk.fusion.LoftSections = loftInput.loftSections

        toeSegment = orderedSegments[0]
        toeFace: adsk.fusion.BRepFace = slabToeFace(toeSegment)
        loftSections.add(toeFace)
        for segment in orderedSegments:
            heelFace: adsk.fusion.BRepFace = slabHeelFace(segment)
            loftSections.add(heelFace)
        loftFeature: adsk.fusion.LoftFeature = loftFeatures.add(loftInput)
        loftBodies: adsk.fusion.BRepBodies = loftFeature.bodies
        spiralTooth: adsk.fusion.BRepBody = loftBodies.item(0)
        spiralTooth.name = f'{gearLabel} Spiral Tooth'

        for segment in segments:
            removeFeatures.add(segment)

        return solids.cut_conical_ends(
            designComponent, spiralTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # ------------------------------------------------------------------------------------
    # Step 20: circular-pattern the tooth around the shaft-axis edge.
    # ------------------------------------------------------------------------------------

    def _circularPatternTooth(self, designComponent: adsk.fusion.Component,
                              toothBody: adsk.fusion.BRepBody,
                              shaftAxisEdge: adsk.fusion.SketchLine, teethNumber: int,
                              gearLabel: str):
        bodyCollection: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        bodyCollection.add(toothBody)

        features: adsk.fusion.Features = designComponent.features
        circularPatternFeatures: adsk.fusion.CircularPatternFeatures = (
            features.circularPatternFeatures)
        patternInput: adsk.fusion.CircularPatternFeatureInput = (
            circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge))
        quantityValue = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.quantity = quantityValue
        totalAngleValue = adsk.core.ValueInput.createByString('360 deg')
        patternInput.totalAngle = totalAngleValue
        patternInput.isSymmetric = False
        pattern: adsk.fusion.CircularPatternFeature = circularPatternFeatures.add(patternInput)

        # The pattern's bodies already include the seed body plus the copies
        # ([PB-PATTERN-BODIES]); copy them into a fresh ObjectCollection for combine.
        patternedBodies: adsk.fusion.BRepBodies = pattern.bodies
        toolCollection: adsk.core.ObjectCollection = adsk.core.ObjectCollection.create()
        for body in patternedBodies:
            toolCollection.add(body)
        return toolCollection

    # ------------------------------------------------------------------------------------
    # Step 21: Combine-Join the patterned teeth into the Gear Body.
    # ------------------------------------------------------------------------------------

    def _combineJoinTeeth(self, designComponent: adsk.fusion.Component,
                          gearBody: adsk.fusion.BRepBody,
                          toolCollection: adsk.core.ObjectCollection, gearLabel: str):
        features: adsk.fusion.Features = designComponent.features
        combineFeatures: adsk.fusion.CombineFeatures = features.combineFeatures
        combineInput: adsk.fusion.CombineFeatureInput = combineFeatures.createInput(
            gearBody, toolCollection)
        joinOp = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.operation = joinOp
        combineFeatures.add(combineInput)

    # ------------------------------------------------------------------------------------
    # Steps 22-23: the Bore sketch and its through-cut.
    # ------------------------------------------------------------------------------------

    def _buildAndCutBore(self, designComponent: adsk.fusion.Component,
                         gearBody: adsk.fusion.BRepBody, shaftAxisEdge: adsk.fusion.SketchLine,
                         boreDiameter_cm: float, gearLabel: str):
        constructionPlanes: adsk.fusion.ConstructionPlanes = designComponent.constructionPlanes
        borePlaneInput: adsk.fusion.ConstructionPlaneInput = constructionPlanes.createInput()
        zeroValue = adsk.core.ValueInput.createByReal(0.0)
        borePlaneInput.setByDistanceOnPath(shaftAxisEdge, zeroValue)
        borePlane: adsk.fusion.ConstructionPlane = constructionPlanes.add(borePlaneInput)

        sketches: adsk.fusion.Sketches = designComponent.sketches
        boreSketch: adsk.fusion.Sketch = sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore'

        origin = adsk.core.Point3D.create(0.0, 0.0, 0.0)
        sketchCurves: adsk.fusion.SketchCurves = boreSketch.sketchCurves
        sketchCircles: adsk.fusion.SketchCircles = sketchCurves.sketchCircles
        boreCircle: adsk.fusion.SketchCircle = sketchCircles.addByCenterRadius(
            origin, boreDiameter_cm / 2.0)
        boreCircleCenter: adsk.fusion.SketchPoint = boreCircle.centerSketchPoint
        boreCircleCenter.isFixed = True

        sketchDimensions: adsk.fusion.SketchDimensions = boreSketch.sketchDimensions
        dimTextPoint = adsk.core.Point3D.create(boreDiameter_cm / 2.0, 0.0, 0.0)
        boreDim = sketchDimensions.addDiameterDimension(boreCircle, dimTextPoint)
        boreDimParameter = boreDim.parameter
        boreDimParameter.value = boreDiameter_cm

        if not boreSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Bore sketch is not fully constrained')

        profiles: adsk.fusion.Profiles = boreSketch.profiles
        boreProfile: adsk.fusion.Profile = profiles.item(0)

        features: adsk.fusion.Features = designComponent.features
        extrudeFeatures: adsk.fusion.ExtrudeFeatures = features.extrudeFeatures
        cutOp = adsk.fusion.FeatureOperations.CutFeatureOperation
        extrudeInput: adsk.fusion.ExtrudeFeatureInput = extrudeFeatures.createInput(
            boreProfile, cutOp)
        coneDistance_cm = self._coneDistance_cm
        cutHalfLength = adsk.core.ValueInput.createByReal(2.0 * coneDistance_cm)
        extrudeInput.setSymmetricExtent(cutHalfLength, False)
        extrudeInput.participantBodies = [gearBody]
        extrudeFeatures.add(extrudeInput)
