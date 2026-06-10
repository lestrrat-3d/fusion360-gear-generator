import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the 17 reproduced-surface strings)
# ---------------------------------------------------------------------------
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER = 'centerPoint'
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

# Hand-of-spiral list-item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Spur generator parameter keys the proxy must serve
PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
PARAM_PRESSURE_ANGLE = 'PressureAngle'
PARAM_PITCH_CIRCLE_DIAMETER = 'PitchCircleDiameter'
PARAM_PITCH_CIRCLE_RADIUS = 'PitchCircleRadius'
PARAM_BASE_CIRCLE_DIAMETER = 'BaseCircleDiameter'
PARAM_BASE_CIRCLE_RADIUS = 'BaseCircleRadius'
PARAM_ROOT_CIRCLE_DIAMETER = 'RootCircleDiameter'
PARAM_ROOT_CIRCLE_RADIUS = 'RootCircleRadius'
PARAM_TIP_CIRCLE_DIAMETER = 'TipCircleDiameter'
PARAM_TIP_CIRCLE_RADIUS = 'TipCircleRadius'
PARAM_INVOLUTE_STEPS = 'InvoluteSteps'

# bevel hardcodes a 20deg pressure angle (not a dialog input)
_PRESSURE_ANGLE_DEG = 20.0
_INVOLUTE_STEPS = 15

# slice count for the spiral tooth body (not user-configurable)
_SPIRAL_SLICE_COUNT = 8


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so Fusion auto-focus lands here
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point (selection)
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Select the point the driving gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component (selection) -- pre-selected to root
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the parent component for the new gear pair')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6. Driving Gear Teeth
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 8. Driving Gear Base Height
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 9. Pinion Gear Base Height
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11. Driving Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 12. Pinion Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 13. Face Width
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 14. Tooth Spacing
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 15. Mean Spiral Angle (always visible -- the controller)
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral (text-list dropdown)
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # initial conditional visibility (default psi = 35deg -> both shown)
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            value = get_design().unitsManager.evaluateExpression(
                spiralInput.expression, 'deg')
            show = value > 0
        except:
            # half-typed expression mid-edit -> leave both shown
            show = True
        handInput.isVisible = show
        cutterInput.isVisible = show

    @classmethod
    def handle_input_changed(cls, args):
        cls._updateSpiralInputVisibility(args.inputs)


# ---------------------------------------------------------------------------
# Virtual spur proxy -- a fake spur `parent` so the borrowed tooth generator
# can run without registering Fusion user parameters.
# ---------------------------------------------------------------------------
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        # OUTPUT slot the spur generator writes during draw()
        self._lastToothEmbedded = False

        pressureAngle = math.radians(_PRESSURE_ANGLE_DEG)
        # Standard spur formulas (Module here is raw mm):
        pitch = virtualTeeth * module_mm        # pitch diameter (mm)
        base = pitch * math.cos(pressureAngle)  # base diameter (mm)
        root = pitch - 2.5 * module_mm          # root diameter (mm)
        tip = pitch + 2.0 * module_mm           # tip diameter (mm)

        # Serve every key the spur drawer reads, in internal cm.
        self._params = {
            PARAM_MODULE: to_cm(module_mm),
            PARAM_TOOTH_NUMBER: virtualTeeth,
            PARAM_PRESSURE_ANGLE: pressureAngle,
            PARAM_PITCH_CIRCLE_DIAMETER: to_cm(pitch),
            PARAM_PITCH_CIRCLE_RADIUS: to_cm(pitch / 2.0),
            PARAM_BASE_CIRCLE_DIAMETER: to_cm(base),
            PARAM_BASE_CIRCLE_RADIUS: to_cm(base / 2.0),
            PARAM_ROOT_CIRCLE_DIAMETER: to_cm(root),
            PARAM_ROOT_CIRCLE_RADIUS: to_cm(root / 2.0),
            PARAM_TIP_CIRCLE_DIAMETER: to_cm(tip),
            PARAM_TIP_CIRCLE_RADIUS: to_cm(tip / 2.0),
            PARAM_INVOLUTE_STEPS: _INVOLUTE_STEPS,
        }

    def getParameter(self, name):
        return _Val(self._params[name])


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    # Tunable crown relief constant (0 disables crown); default 0.5
    _CROWN_PER_RAD = 0.5
    # Extra pinion mesh-phase, in tooth-fractions (0 by default)
    _PINION_MESH_PHASE_TEETH = 0.0

    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # =======================================================================
    # generate -- top-level orchestration
    # =======================================================================
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        shaftAngle_rad = math.radians(shaftAngle_deg)

        # Resolve pitch diameters (internal cm) -- Module is raw mm.
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        # Cone distance (cm)
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        # Pitch-cone half angles (closed form). PPD/DPD ratios reduce to teeth.
        # tan gamma_p = sin(S)*PPD / (DPD + PPD*cos(S))
        ppd = module * pinionTeeth
        dpd = module * drivingTeeth
        self._gamma_p = math.atan2(
            math.sin(shaftAngle_rad) * ppd,
            dpd + ppd * math.cos(shaftAngle_rad))
        self._gamma_g = shaftAngle_rad - self._gamma_p

        # Resolve bores (cm). 0 means auto = pitch diameter / 4.
        if self._drivingBore_cm > 0:
            drivingBore_cm = self._drivingBore_cm
        else:
            drivingBore_cm = drivingPitchDiameter_cm / 4.0
        if self._pinionBore_cm > 0:
            pinionBore_cm = self._pinionBore_cm
        else:
            pinionBore_cm = pinionPitchDiameter_cm / 4.0

        # ---- build component tree ----
        # Bevel Gear component as a child of the parent component.
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelComponent = self.bevelOccurrence.component
        bevelComponent.name = 'Bevel Gear'

        # Design component as a child of the Bevel Gear component.
        designOccurrence = self.bevelOccurrence.component.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designComponent = designOccurrence.component
        designComponent.name = 'Design'

        # ---- geometry ----
        # Stash precomputed cm/numeric values commonly needed downstream.
        self._module_mm = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._drivingPitchDiameter_cm = drivingPitchDiameter_cm
        self._pinionPitchDiameter_cm = pinionPitchDiameter_cm
        self._coneDistance_cm = coneDistance_cm

        # §1 Anchor Sketch
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear bodies
        self._buildGearProfiles(
            designComponent, designOccurrence, bevelComponent,
            targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_rad,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm,
            drivingBore_cm, pinionBore_cm)

        # Cleanup
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # =======================================================================
    # Input reading / validation
    # =======================================================================
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def evalExpr(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections
        (parentList, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentList) != 1:
            raise Exception('Exactly one Parent Component must be selected')
        parentEntity = parentList[0]
        if isinstance(parentEntity, adsk.fusion.Occurrence) or \
                parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentEntity.component
        else:
            parentComponent = parentEntity

        (planeList, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeList) != 1:
            raise Exception('Exactly one Target Plane must be selected')
        targetPlane = planeList[0]

        (centerList, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centerList) != 1:
            raise Exception('Exactly one Center Point must be selected')
        centerPoint = centerList[0]

        # Module -- raw mm number (unit '')
        module = evalExpr(INPUT_ID_MODULE, '')

        # Shaft Angle -- comes back in radians; convert to degrees for range check
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception(
                f'Shaft Angle must be between 30 and 150 degrees (got {shaftAngle_deg:.2f})')

        # Teeth
        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')

        # 'mm' inputs come back already in internal cm.
        self._drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = evalExpr(INPUT_ID_TOOTH_SPACING, 'mm')

        (self._boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)

        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        # Spiral inputs
        spiralAngle_rad = evalExpr(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                f'Mean Spiral Angle must be in [0, 60) degrees (got {spiralAngle_deg:.2f})')
        self._spiralAngle_rad = spiralAngle_rad

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem = handInput.selectedItem
        self._spiralHand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        cutterRadius_cm = evalExpr(INPUT_ID_CUTTER_RADIUS, 'mm')
        if cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')
        self._cutterRadius_cm = cutterRadius_cm

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # =======================================================================
    # §1 Anchor Sketch
    # =======================================================================
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Mark the center point by projecting the user center point.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        # Anchor line through the projected center: ~10mm length.
        cg = projectedCenter.geometry
        half = to_cm(5.0)
        line = lines.addByTwoPoints(
            adsk.core.Point3D.create(cg.x - half, cg.y, 0),
            adsk.core.Point3D.create(cg.x + half, cg.y, 0))

        # BOTH: pin center onto the line, and center bisects the line.
        constraints.addCoincident(projectedCenter, line)
        constraints.addMidPoint(projectedCenter, line)
        dimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + half, 0))
        # Pin direction (sketch-local) so the sketch has zero DOF.
        constraints.addHorizontal(line)

        # Stash the projected-center SketchPoint for §2 to re-project.
        self._anchorCenterPoint = projectedCenter

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorSketch = sketch
        return line

    # =======================================================================
    # §2 + §3 + per-gear bodies
    # =======================================================================
    def _buildGearProfiles(self, designComponent, designOccurrence, bevelComponent,
                           targetPlane, anchorLine,
                           module, drivingTeeth, pinionTeeth, shaftAngle_rad,
                           drivingPitchDiameter_cm, pinionPitchDiameter_cm,
                           drivingBore_cm, pinionBore_cm):
        # ---- §2: Gear Profiles sketch on a plane perpendicular to target ----
        planes = designComponent.constructionPlanes
        gpInput = planes.createInput()
        gpInput.setByAngle(anchorLine,
                           adsk.core.ValueInput.createByString('90 deg'),
                           targetPlane)
        gearProfilesPlane = planes.add(gpInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True
        self._gearProfilesPlane = gearProfilesPlane
        self._gearProfilesSketch = sketch

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm

        # Project the Anchor Sketch's center point (NOT the raw user center).
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchor = sketch.project(anchorLine).item(0)

        c = projectedCenter.geometry
        # 2-D unit direction of the projected anchor line.
        ag0 = projectedAnchor.startSketchPoint.geometry
        ag1 = projectedAnchor.endSketchPoint.geometry
        dvec = adsk.core.Vector3D.create(ag1.x - ag0.x, ag1.y - ag0.y, 0)
        dlen = dvec.length
        dx, dy = dvec.x / dlen, dvec.y / dlen
        # in-plane perpendicular
        perpx, perpy = -dy, dx

        # Grow side: choose the perp sign so it points toward the target normal.
        targetNormal = get_normal(targetPlane)
        # map the sketch-local perp to world via the sketch transform
        perpWorld = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perpx, c.y + perpy, 0))
        cWorld = sketch.sketchToModelSpace(adsk.core.Point3D.create(c.x, c.y, 0))
        perpWorldVec = adsk.core.Vector3D.create(
            perpWorld.x - cWorld.x, perpWorld.y - cWorld.y, perpWorld.z - cWorld.z)
        if perpWorldVec.dotProduct(targetNormal) < 0:
            perpx, perpy = -perpx, -perpy

        def P(x, y):
            return adsk.core.Point3D.create(x, y, 0)

        # ---- center -> Apex construction line ----
        apexSeedX = c.x + perpx * DPD
        apexSeedY = c.y + perpy * DPD
        centerToApex = lines.addByTwoPoints(P(c.x, c.y), P(apexSeedX, apexSeedY))
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projectedAnchor)
        apex = centerToApex.endSketchPoint

        # ---- closed-form along-shaft seed geometry ----
        Sigma = shaftAngle_rad
        gamma_p = self._gamma_p
        R = (PPD / 2.0) / math.sin(gamma_p)
        lenA = R * math.cos(gamma_p)          # |Apex -> A|
        lenB = R * math.cos(self._gamma_g)    # |Apex -> B|

        # ---- Driving Gear Shaft Axis (Apex -> B), parallel to center->apex ----
        # seed point B at apex + (-perp) * lenB
        bSeedX = apexSeedX - perpx * lenB
        bSeedY = apexSeedY - perpy * lenB
        drivingShaftAxis = lines.addByTwoPoints(P(apexSeedX, apexSeedY), P(bSeedX, bSeedY))
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # ---- Pinion Gear Shaft Axis (Apex -> A) ----
        # candidate A positions: rotate driving-shaft dir about apex by +/- Sigma
        # driving-shaft direction (apex -> B) is -perp
        ddirx, ddiry = -perpx, -perpy
        cosS, sinS = math.cos(Sigma), math.sin(Sigma)
        # +Sigma rotation
        plusx = ddirx * cosS - ddiry * sinS
        plusy = ddirx * sinS + ddiry * cosS
        # -Sigma rotation
        minusx = ddirx * cosS + ddiry * sinS
        minusy = -ddirx * sinS + ddiry * cosS
        aPlus = (apexSeedX + plusx * lenA, apexSeedY + plusy * lenA)
        aMinus = (apexSeedX + minusx * lenA, apexSeedY + minusy * lenA)
        # keep the candidate with the GREATER X
        if aPlus[0] >= aMinus[0]:
            aSeed = aPlus
        else:
            aSeed = aMinus
        pinionShaftAxis = lines.addByTwoPoints(P(apexSeedX, apexSeedY), P(aSeed[0], aSeed[1]))
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        # angular dimension between the two shaft axes = Shaft Angle, measured at Sigma
        midWedge = P((aSeed[0] + bSeedX) / 2.0, (aSeed[1] + bSeedY) / 2.0)
        angDim = dimensions.addAngularDimension(drivingShaftAxis, pinionShaftAxis, midWedge)
        angDim.parameter.value = Sigma
        pointA = pinionShaftAxis.endSketchPoint

        # ---- A->Apex2 perpendicular drop (PPD/2) ----
        ag = pointA.geometry
        # perpendicular to the pinion shaft axis (Apex->A direction) toward Apex2
        paxd = adsk.core.Vector3D.create(
            ag.x - apexSeedX, ag.y - apexSeedY, 0)
        pl = paxd.length
        paxd_x, paxd_y = paxd.x / pl, paxd.y / pl
        # perpendicular candidates; choose toward the anchor (toward apex/center side)
        n1 = (-paxd_y, paxd_x)
        # pick the one pointing toward c
        toC = adsk.core.Vector3D.create(c.x - ag.x, c.y - ag.y, 0)
        if (n1[0] * toC.x + n1[1] * toC.y) < 0:
            n1 = (-n1[0], -n1[1])
        a2SeedFromA = (ag.x + n1[0] * (PPD / 2.0), ag.y + n1[1] * (PPD / 2.0))
        dropA = lines.addByTwoPoints(P(ag.x, ag.y), P(a2SeedFromA[0], a2SeedFromA[1]))
        dropA.isConstruction = True
        constraints.addCoincident(dropA.startSketchPoint, pointA)
        constraints.addPerpendicular(dropA, pinionShaftAxis)
        dimensions.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P(a2SeedFromA[0], a2SeedFromA[1])).parameter.value = PPD / 2.0

        # ---- B->Apex2 perpendicular drop (DPD/2) ----
        bg = pointB.geometry
        dbxd = adsk.core.Vector3D.create(bg.x - apexSeedX, bg.y - apexSeedY, 0)
        bl = dbxd.length
        dbxd_x, dbxd_y = dbxd.x / bl, dbxd.y / bl
        n2 = (-dbxd_y, dbxd_x)
        toC2 = adsk.core.Vector3D.create(c.x - bg.x, c.y - bg.y, 0)
        if (n2[0] * toC2.x + n2[1] * toC2.y) < 0:
            n2 = (-n2[0], -n2[1])
        a2SeedFromB = (bg.x + n2[0] * (DPD / 2.0), bg.y + n2[1] * (DPD / 2.0))
        dropB = lines.addByTwoPoints(P(bg.x, bg.y), P(a2SeedFromB[0], a2SeedFromB[1]))
        dropB.isConstruction = True
        constraints.addCoincident(dropB.startSketchPoint, pointB)
        constraints.addPerpendicular(dropB, drivingShaftAxis)
        dimensions.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P(a2SeedFromB[0], a2SeedFromB[1])).parameter.value = DPD / 2.0

        # ---- Apex2: coincident the two drop endpoints ----
        constraints.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2 = dropA.endSketchPoint

        # ---- Pitch Line: Apex -> Apex2 ----
        a2g = apex2.geometry
        pitchLine = lines.addByTwoPoints(P(apexSeedX, apexSeedY), P(a2g.x, a2g.y))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apex)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2)

        # ---- Dedendum lines from Apex2 (Module * 1.25), perpendicular to Pitch Line ----
        ded_cm = to_cm(1.25 * module)
        # pitch line direction
        pld = adsk.core.Vector3D.create(a2g.x - apexSeedX, a2g.y - apexSeedY, 0)
        pld.normalize()
        # perpendicular dirs
        pp = (-pld.y, pld.x)
        # toward anchor line = toward c
        towardC = adsk.core.Vector3D.create(c.x - a2g.x, c.y - a2g.y, 0)
        if (pp[0] * towardC.x + pp[1] * towardC.y) >= 0:
            dirToD = pp
            dirToC = (-pp[0], -pp[1])
        else:
            dirToD = (-pp[0], -pp[1])
            dirToC = pp
        # Driving Gear Dedendum -> point D (toward anchor line)
        dSeed = (a2g.x + dirToD[0] * ded_cm, a2g.y + dirToD[1] * ded_cm)
        drivingDedendum = lines.addByTwoPoints(P(a2g.x, a2g.y), P(dSeed[0], dSeed[1]))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P(dSeed[0], dSeed[1])).parameter.value = ded_cm
        pointD = drivingDedendum.endSketchPoint
        # Pinion Gear Dedendum -> point C (away from anchor line)
        cSeed = (a2g.x + dirToC[0] * ded_cm, a2g.y + dirToC[1] * ded_cm)
        pinionDedendum = lines.addByTwoPoints(P(a2g.x, a2g.y), P(cSeed[0], cSeed[1]))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            P(cSeed[0], cSeed[1])).parameter.value = ded_cm
        pointC = pinionDedendum.endSketchPoint

        # ---- Root Axes: Apex -> D (driving), Apex -> C (pinion) ----
        dg = pointD.geometry
        cg2 = pointC.geometry
        drivingRootAxis = lines.addByTwoPoints(P(apexSeedX, apexSeedY), P(dg.x, dg.y))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apex)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)
        pinionRootAxis = lines.addByTwoPoints(P(apexSeedX, apexSeedY), P(cg2.x, cg2.y))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apex)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        mod_cm = to_cm(module)

        # ---- A->E : collinear with Apex->A, length module (undimensioned) ----
        # seed E one module beyond A along Apex->A direction
        aSeedDir = (paxd_x, paxd_y)  # unit Apex->A
        eSeed = (ag.x + aSeedDir[0] * mod_cm, ag.y + aSeedDir[1] * mod_cm)
        lineAE = lines.addByTwoPoints(P(ag.x, ag.y), P(eSeed[0], eSeed[1]))
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        # ---- C->E : perpendicular to A->E ----
        eg = pointE.geometry
        lineCE = lines.addByTwoPoints(P(cg2.x, cg2.y), P(eg.x, eg.y))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # ---- B->F : collinear with Apex->B, length module ----
        bAxisDir = (dbxd_x, dbxd_y)  # unit Apex->B
        fSeed = (bg.x + bAxisDir[0] * mod_cm, bg.y + bAxisDir[1] * mod_cm)
        lineBF = lines.addByTwoPoints(P(bg.x, bg.y), P(fSeed[0], fSeed[1]))
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        # ---- D->F : perpendicular to B->F ----
        fg = pointF.geometry
        lineDF = lines.addByTwoPoints(P(dg.x, dg.y), P(fg.x, fg.y))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # ---- E->G : collinear to A->E, length module ----
        gSeed = (eg.x + aSeedDir[0] * mod_cm, eg.y + aSeedDir[1] * mod_cm)
        lineEG = lines.addByTwoPoints(P(eg.x, eg.y), P(gSeed[0], gSeed[1]))
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # ---- C->H : length module, collinear with Apex2->C ----
        # direction Apex2 -> C
        a2cDir = (dirToC[0], dirToC[1])
        hSeed = (cg2.x + a2cDir[0] * mod_cm, cg2.y + a2cDir[1] * mod_cm)
        lineCH = lines.addByTwoPoints(P(cg2.x, cg2.y), P(hSeed[0], hSeed[1]))
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # ---- G->H : connect, perpendicular E->G and H->G ----
        gg = pointG.geometry
        hg = pointH.geometry
        lineGH = lines.addByTwoPoints(P(gg.x, gg.y), P(hg.x, hg.y))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # ---- F->I : collinear to B->F, length module ----
        iSeed = (fg.x + bAxisDir[0] * mod_cm, fg.y + bAxisDir[1] * mod_cm)
        lineFI = lines.addByTwoPoints(P(fg.x, fg.y), P(iSeed[0], iSeed[1]))
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # ---- D->J : length module, collinear with Apex2->D ----
        a2dDir = (dirToD[0], dirToD[1])
        jSeed = (dg.x + a2dDir[0] * mod_cm, dg.y + a2dDir[1] * mod_cm)
        lineDJ = lines.addByTwoPoints(P(dg.x, dg.y), P(jSeed[0], jSeed[1]))
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # ---- I->J : connect, perpendicular F->I and J->I ----
        ig = pointI.geometry
        jg = pointJ.geometry
        lineIJ = lines.addByTwoPoints(P(ig.x, ig.y), P(jg.x, jg.y))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # ---- offset dim between B->Apex2 drop (dropB) and J->I, = driving base height ----
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8.0)
        dimensions.addOffsetDimension(
            dropB, lineIJ, P(ig.x, ig.y)).parameter.value = drivingBaseHeight_cm

        # ---- offset dim between A->Apex2 drop (dropA) and G->H, = pinion base height ----
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        dimensions.addOffsetDimension(
            dropA, lineGH, P(gg.x, gg.y)).parameter.value = pinionBaseHeight_cm

        # ---- A->G connector ----
        lineAG = lines.addByTwoPoints(P(ag.x, ag.y), P(gg.x, gg.y))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # ---- Constrain Point I with center point ----
        constraints.addCoincident(pointI, projectedCenter)

        # ---- K: point on Apex->A and on Pinion Dedendum (Apex2->C extended) ----
        # seed K near intersection of Apex->A extended and dedendum line
        kSeed = (gg.x, gg.y)
        lineGK = lines.addByTwoPoints(P(gg.x, gg.y), P(kSeed[0] + aSeedDir[0] * mod_cm,
                                                      kSeed[1] + aSeedDir[1] * mod_cm))
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)
        kg = pointK.geometry
        # reference line C->K
        lineCK = lines.addByTwoPoints(P(cg2.x, cg2.y), P(kg.x, kg.y))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # ---- K' tooth-center (Tooth Spacing offset) ----
        pinionCenterRefLine = lineCK
        pinionCenterPoint = pointK
        if self._toothSpacing_cm > 0:
            # direction C -> K (away from C, the lower corner) along dedendum
            ckdir = adsk.core.Vector3D.create(kg.x - cg2.x, kg.y - cg2.y, 0)
            ckdir.normalize()
            kpSeed = (kg.x + ckdir.x * self._toothSpacing_cm,
                      kg.y + ckdir.y * self._toothSpacing_cm)
            lineKKp = lines.addByTwoPoints(P(kg.x, kg.y), P(kpSeed[0], kpSeed[1]))
            lineKKp.isConstruction = True
            constraints.addCoincident(lineKKp.startSketchPoint, pointK)
            pointKp = lineKKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            dimensions.addDistanceDimension(
                lineKKp.startSketchPoint, lineKKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                P(kpSeed[0], kpSeed[1])).parameter.value = self._toothSpacing_cm
            kpg = pointKp.geometry
            lineCKp = lines.addByTwoPoints(P(cg2.x, cg2.y), P(kpg.x, kpg.y))
            lineCKp.isConstruction = True
            constraints.addCoincident(lineCKp.startSketchPoint, pointC)
            constraints.addCoincident(lineCKp.endSketchPoint, pointKp)
            pinionCenterRefLine = lineCKp
            pinionCenterPoint = pointKp

        # ---- Maximum Face Width from SOLVED geometry ----
        maxFaceWidth_cm = self._computeMaxFaceWidth(
            pointA, pointB, pointC, pointD, pointH, pointJ)
        # Apply the Face Width bound.
        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width exceeds maximum allowed ({to_mm(maxFaceWidth_cm):.3f} mm)')
            faceWidth_cm = self._faceWidth_cm
        else:
            faceWidth_cm = min(self._coneDistance_cm / 6.0, maxFaceWidth_cm)
        self._resolvedFaceWidth_cm = faceWidth_cm

        # ---- M->N (pinion toe line) ----
        # seed M near midpoint of Apex->C, N slid along C->H toward line A->Apex2
        apexG = apex.geometry
        mSeedX = (apexG.x + cg2.x) / 2.0
        mSeedY = (apexG.y + cg2.y) / 2.0
        # C->H direction
        chdir = adsk.core.Vector3D.create(hg.x - cg2.x, hg.y - cg2.y, 0)
        chdir.normalize()
        # distance from M-seed to A as the slide amount
        slideMN = math.hypot(ag.x - mSeedX, ag.y - mSeedY)
        nSeedX = mSeedX + chdir.x * slideMN
        nSeedY = mSeedY + chdir.y * slideMN
        lineMN = lines.addByTwoPoints(P(mSeedX, mSeedY), P(nSeedX, nSeedY))
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, dropA)
        constraints.addParallel(lineMN, lineCH)
        dimensions.addOffsetDimension(
            lineCH, lineMN, P(nSeedX, nSeedY)).parameter.value = faceWidth_cm
        mg = pointM.geometry
        ng = pointN.geometry
        # M->C and N->A reference lines
        lineMC = lines.addByTwoPoints(P(mg.x, mg.y), P(cg2.x, cg2.y))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(P(ng.x, ng.y), P(ag.x, ag.y))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # ---- L: point on Apex->B and on Driving Dedendum (Apex2->D extended) ----
        lineIL = lines.addByTwoPoints(P(ig.x, ig.y), P(ig.x + bAxisDir[0] * mod_cm,
                                                      ig.y + bAxisDir[1] * mod_cm))
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)
        lg = pointL.geometry
        lineDL = lines.addByTwoPoints(P(dg.x, dg.y), P(lg.x, lg.y))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # ---- L' tooth-center ----
        drivingCenterRefLine = lineDL
        drivingCenterPoint = pointL
        if self._toothSpacing_cm > 0:
            dldir = adsk.core.Vector3D.create(lg.x - dg.x, lg.y - dg.y, 0)
            dldir.normalize()
            lpSeed = (lg.x + dldir.x * self._toothSpacing_cm,
                      lg.y + dldir.y * self._toothSpacing_cm)
            lineLLp = lines.addByTwoPoints(P(lg.x, lg.y), P(lpSeed[0], lpSeed[1]))
            lineLLp.isConstruction = True
            constraints.addCoincident(lineLLp.startSketchPoint, pointL)
            pointLp = lineLLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            dimensions.addDistanceDimension(
                lineLLp.startSketchPoint, lineLLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                P(lpSeed[0], lpSeed[1])).parameter.value = self._toothSpacing_cm
            lpg = pointLp.geometry
            lineDLp = lines.addByTwoPoints(P(dg.x, dg.y), P(lpg.x, lpg.y))
            lineDLp.isConstruction = True
            constraints.addCoincident(lineDLp.startSketchPoint, pointD)
            constraints.addCoincident(lineDLp.endSketchPoint, pointLp)
            drivingCenterRefLine = lineDLp
            drivingCenterPoint = pointLp

        # ---- O->P (driving toe line) ----
        oSeedX = (apexG.x + dg.x) / 2.0
        oSeedY = (apexG.y + dg.y) / 2.0
        djdir = adsk.core.Vector3D.create(jg.x - dg.x, jg.y - dg.y, 0)
        djdir.normalize()
        slideOP = math.hypot(bg.x - oSeedX, bg.y - oSeedY)
        pSeedX = oSeedX + djdir.x * slideOP
        pSeedY = oSeedY + djdir.y * slideOP
        lineOP = lines.addByTwoPoints(P(oSeedX, oSeedY), P(pSeedX, pSeedY))
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, dropB)
        constraints.addParallel(lineOP, lineDJ)
        dimensions.addOffsetDimension(
            lineDJ, lineOP, P(pSeedX, pSeedY)).parameter.value = faceWidth_cm
        og = pointO.geometry
        pg = pointP.geometry
        lineOD = lines.addByTwoPoints(P(og.x, og.y), P(dg.x, dg.y))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(P(pg.x, pg.y), P(bg.x, bg.y))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(P(bg.x, bg.y), P(ig.x, ig.y))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        # ---- full-constraint gate on the Gear Profiles sketch ----
        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        # ---- world apex sketch point for lofts ----
        self._apexSketchPoint = apex

        # ===================================================================
        # §3 + per-gear bodies (pinion first, then driving)
        # ===================================================================
        # Pinion
        pinionProfile = self._buildVirtualSpurProfile(
            designComponent, sketch, gearProfilesPlane,
            module, self._pinionTeeth, self._gamma_p,
            pinionCenterRefLine, pinionCenterPoint, 'Pinion')
        self._createGearBody(
            designComponent, designOccurrence, bevelComponent,
            sketch, gearProfilesPlane, apex,
            'Pinion', self._pinionTeeth, pinionBore_cm,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            (pointM, pointN), (pointC, pointH),
            pinionProfile, isDriving=False)

        # Driving
        drivingProfile = self._buildVirtualSpurProfile(
            designComponent, sketch, gearProfilesPlane,
            module, self._drivingTeeth, self._gamma_g,
            drivingCenterRefLine, drivingCenterPoint, 'Driving')
        self._createGearBody(
            designComponent, designOccurrence, bevelComponent,
            sketch, gearProfilesPlane, apex,
            'Driving', self._drivingTeeth, drivingBore_cm,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            (pointO, pointP), (pointD, pointJ),
            drivingProfile, isDriving=True)

    def _computeMaxFaceWidth(self, pointA, pointB, pointC, pointD, pointH, pointJ):
        # perpendicular distance from A to line through C and H (pinion dedendum)
        def perpDist(p, l0, l1):
            pg = p.geometry
            g0 = l0.geometry
            g1 = l1.geometry
            dx = g1.x - g0.x
            dy = g1.y - g0.y
            ln = math.hypot(dx, dy)
            if ln == 0:
                return float('inf')
            # cross product magnitude / length
            return abs((pg.x - g0.x) * dy - (pg.y - g0.y) * dx) / ln
        dA = perpDist(pointA, pointC, pointH)
        dB = perpDist(pointB, pointD, pointJ)
        return 0.95 * min(dA, dB)

    # =======================================================================
    # §3 -- build the virtual spur tooth profile for one gear
    # =======================================================================
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesSketch,
                                 gearProfilesPlane, module, teeth, gamma,
                                 centerRefLine, centerPoint, gearLabel):
        # 1. virtual (back-cone) tooth number
        thisPitchRadius_mm = (teeth * module) / 2.0
        virtualPitchRadius_mm = thisPitchRadius_mm / math.cos(gamma)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / module))

        planes = designComponent.constructionPlanes

        # 2. plane that includes the tooth-center reference line, perpendicular
        #    to the Gear Profiles sketch plane.
        tpInput = planes.createInput()
        tpInput.setByAngle(centerRefLine,
                          adsk.core.ValueInput.createByString('90 deg'),
                          gearProfilesPlane)
        toothPlane = planes.add(tpInput)
        toothPlane.name = f'{gearLabel} Plane'

        # 3. create the spur tooth profile, rotated 180deg.
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        # The tooth-center sketch point is the anchor for the draw projection.
        drawer.draw(centerPoint, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded

        if not toothSketch.isFullyConstrained:
            futil.log(f'{gearLabel} tooth sketch is not fully constrained (expected for embedded teeth)')

        # 4. construction axis through the tooth-center, normal to the tooth plane.
        helperInput = planes.createInput()
        helperInput.setByDistanceOnPath(centerRefLine,
                                       adsk.core.ValueInput.createByReal(1.0))
        helperPlane = planes.add(helperInput)
        helperPlane.name = f'{gearLabel} Tooth Axis Helper'
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'

        return {
            'toothSketch': toothSketch,
            'toothPlane': toothPlane,
            'toothAxis': toothAxis,
            'embedded': embedded,
            'virtualTeeth': virtualTeeth,
            'gamma': gamma,
        }

    def _findSpurToothProfile(self, toothSketch, embedded):
        wantLines = 0 if embedded else 2
        Curve3DTypes = adsk.core.Curve3DTypes
        for profile in toothSketch.profiles:
            for loop in profile.profileLoops:
                nurbs = arcs = linesN = others = 0
                for pc in loop.profileCurves:
                    ct = pc.geometry.curveType
                    if ct == Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == Curve3DTypes.Line3DCurveType:
                        linesN += 1
                    else:
                        others += 1
                if nurbs == 2 and arcs == 2 and linesN == wantLines and others == 0:
                    return profile
        raise Exception(
            f'Could not find spur tooth profile (2 NURBS + 2 arcs + {wantLines} lines)')

    # =======================================================================
    # Per-gear body creation
    # =======================================================================
    def _createGearBody(self, designComponent, designOccurrence, bevelComponent,
                        gearProfilesSketch, gearProfilesPlane, apexSketchPoint,
                        gearLabel, teethNumber, boreDiameter_cm,
                        hexVertices, toeEdge, heelEdge,
                        profileInfo, isDriving):
        features = designComponent.features

        # Output component for this gear, child of Bevel Gear.
        gearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearComponent = gearOccurrence.component
        gearComponent.name = f'{gearLabel} Gear'

        # ---- Profile sketch (fresh, on the axial Gear Profiles plane) ----
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'
        profileSketch.isVisible = True
        plines = profileSketch.sketchCurves.sketchLines

        # recreate the six vertices as new points at their world-mapped positions
        verts = []
        for v in hexVertices:
            local = profileSketch.modelToSketchSpace(v.worldGeometry)
            verts.append(profileSketch.sketchPoints.add(local))
        # draw the closed hexagon sharing those points
        hexLines = []
        for i in range(len(verts)):
            ln = plines.addByTwoPoints(verts[i], verts[(i + 1) % len(verts)])
            hexLines.append(ln)
        # fix the lines' endpoints AFTER the lines exist
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Profile sketch is not fully constrained')

        # the shaft-axis edge is the hexagon's FIRST edge
        shaftAxisEdge = hexLines[0]

        # ---- Revolve the single hexagon profile around the shaft axis ----
        profile = profileSketch.profiles.item(0)
        revInput = features.revolveFeatures.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = features.revolveFeatures.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # ---- Tooth loft: §2 Apex sketch point -> §3 tooth profile ----
        toothProfile = self._findSpurToothProfile(
            profileInfo['toothSketch'], profileInfo['embedded'])
        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = features.loftFeatures.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth'

        # ---- world hand-off points per §3a "Caller hand-off" table ----
        apexWorld = apexSketchPoint.worldGeometry
        toeStartW = toeEdge[0].worldGeometry   # M / O
        toeEndW = toeEdge[1].worldGeometry     # N / P
        heelStartW = heelEdge[0].worldGeometry  # C / D
        heelEndW = heelEdge[1].worldGeometry    # H / J
        toeMid = adsk.core.Point3D.create(
            (toeStartW.x + toeEndW.x) / 2.0,
            (toeStartW.y + toeEndW.y) / 2.0,
            (toeStartW.z + toeEndW.z) / 2.0)
        heelMid = adsk.core.Point3D.create(
            (heelStartW.x + heelEndW.x) / 2.0,
            (heelStartW.y + heelEndW.y) / 2.0,
            (heelStartW.z + heelEndW.z) / 2.0)
        toeConeWorld = toeStartW    # M / O (on root cone element)
        heelConeWorld = heelStartW  # C / D (dedendum corner, on root axis)

        # ---- tooth-body step (straight cut OR spiral build) ----
        toothBody = self._transformToothBody(
            designComponent, designOccurrence, toothBody, gearBody, shaftAxisEdge,
            apexWorld, apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
            profileInfo['toothPlane'], gearLabel, teethNumber)

        # ---- pattern around the shaft-axis edge ----
        patternEntities = adsk.core.ObjectCollection.create()
        patternEntities.add(toothBody)
        patInput = features.circularPatternFeatures.createInput(
            patternEntities, shaftAxisEdge)
        patInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patInput.isSymmetric = False
        pattern = features.circularPatternFeatures.add(patInput)

        # ---- combine: join all patterned tooth pieces into the gear body ----
        toolBodies = adsk.core.ObjectCollection.create()
        for b in pattern.bodies:
            toolBodies.add(b)
        combineInput = features.combineFeatures.createInput(gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        features.combineFeatures.add(combineInput)

        # ---- bore ----
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, boreDiameter_cm)

        # ---- mesh-rotate ----
        if isDriving:
            self._rotateBodyAboutEdge(
                designComponent, gearBody, shaftAxisEdge,
                math.radians(180.0) / teethNumber)
        else:
            phase = self._pinionMeshPhase()
            if phase != 0:
                self._rotateBodyAboutEdge(
                    designComponent, gearBody, shaftAxisEdge,
                    phase * (2.0 * math.pi / teethNumber))

        # ---- move the finished body to its gear component ----
        gearBody.moveToComponent(gearOccurrence)

    def _pinionMeshPhase(self):
        # 0 by default (mid-face section unrotated and already meshes).
        return self._PINION_MESH_PHASE_TEETH

    def _rotateBodyAboutEdge(self, designComponent, body, edge, angle):
        startW = edge.startSketchPoint.worldGeometry
        endW = edge.endSketchPoint.worldGeometry
        axisVec = adsk.core.Vector3D.create(
            endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
        axisVec.normalize()
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angle, axisVec, startW)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moveInput = designComponent.features.moveFeatures.createInput2(bodies)
        moveInput.defineAsFreeMove(matrix)
        designComponent.features.moveFeatures.add(moveInput)

    # =======================================================================
    # Tooth-body hook (the spiral gate)
    # =======================================================================
    def _transformToothBody(self, designComponent, designOccurrence, toothBody, gearBody,
                            shaftAxisEdge, apexWorld, apexSketchPoint,
                            toeMid, heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber):
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)

        # ===== ψ > 0 : spiral tooth body =====
        # A. Gate & frame.
        sStart = shaftAxisEdge.startSketchPoint.worldGeometry
        sEnd = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = adsk.core.Vector3D.create(
            sEnd.x - sStart.x, sEnd.y - sStart.y, sEnd.z - sStart.z)
        axisDir.normalize()

        apex = apexWorld

        # swap-guard: heel must be the OUTER end
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = adsk.core.Vector3D.create(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y, heelConeWorld.z - apex.z)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x +
                    (p.y - apex.y) * coneVec.y +
                    (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        gamma = self._gamma_p if gearLabel == 'Pinion' else self._gamma_g

        # B. Cutter-arc geometry.
        if self._cutterRadius_cm > 0:
            r_c = self._cutterRadius_cm
        else:
            r_c = R_mean
        psi = self._spiralAngle_rad
        handSign = 1.0 if self._spiralHand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = self._circleIntersectNearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        # helper: tangent-plane 2-D coords -> world point
        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        # C. 2-D trace sketch.
        # cone-element construction line on the axial plane.
        coneSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneSketch.isVisible = True
        clines = coneSketch.sketchCurves.sketchLines
        apexHeel = self._combine(apex, R_heel, coneVec, 0.0, v)
        coneElementLine = clines.addByTwoPoints(
            coneSketch.modelToSketchSpace(apex),
            coneSketch.modelToSketchSpace(apexHeel))

        tracePlane = self._planeByAngle(
            designComponent, coneElementLine, self._gearProfilesPlane, 90.0)
        tracePlane.name = f'{gearLabel} Trace Plane'

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        traceSketch.isVisible = True
        tcircles = traceSketch.sketchCurves.sketchCircles
        tarcs = traceSketch.sketchCurves.sketchArcs
        tdims = traceSketch.sketchDimensions
        tcons = traceSketch.geometricConstraints

        cutterCenterW = tanW(Cx, Cy)
        cutterCircle = tcircles.addByCenterRadius(
            traceSketch.modelToSketchSpace(cutterCenterW), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        tdims.addDiameterDimension(
            cutterCircle,
            traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy))).parameter.value = 2.0 * r_c

        toeW = tanW(toe2d[0], toe2d[1])
        meanW = tanW(R_mean, 0.0)
        heelW = tanW(heel2d[0], heel2d[1])
        traceArc = tarcs.addByThreePoints(
            traceSketch.modelToSketchSpace(toeW),
            traceSketch.modelToSketchSpace(meanW),
            traceSketch.modelToSketchSpace(heelW))
        tcons.addCoincident(traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        tdims.addRadialDimension(
            traceArc,
            traceSketch.modelToSketchSpace(meanW)).parameter.value = r_c
        # (deliberately left with free DOF -- exempt from the gate)

        # E. Slice the straight tooth.
        segments = self._sliceTooth(
            designComponent, toothBody, parentToothPlane, apex, coneVec,
            span, distAlong, gearLabel)

        # F. Order & drop scrap.
        def centroidDist(body):
            return distAlong(body.physicalProperties.centerOfMass)
        segments.sort(key=centroidDist)
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                f'{gearLabel}: spiral slice left no working segments after dropping scrap')

        # G. Twist.
        phi_crown = (math.atan2(heel2d[1], heel2d[0]) -
                     math.atan2(toe2d[1], toe2d[0]))
        total = abs(phi_crown) / math.sin(gamma)

        def slabHeelFace(seg):
            best = None
            bestD = -float('inf')
            for f in seg.faces:
                d = distAlong(f.centroid)
                if d > bestD:
                    bestD = d
                    best = f
            return best, bestD

        for seg in segments:
            (hf, R_heelFace) = slabHeelFace(seg)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apex)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # H. Lengthwise crown (relief).
        # Sort by post-twist heel-face distance to find the outermost (heel) seg.
        segOrder = sorted(
            segments, key=lambda s: slabHeelFace(s)[1])
        heelSeg = segOrder[-1]

        designOccurrence.activate()
        try:
            for seg in segments:
                if seg is heelSeg:
                    continue  # skip the outermost (heel) segment
                (hf, R_heelFace) = slabHeelFace(seg)
                u = (R_heel - R_heelFace) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                basePoint = self._heelFaceRootBasePoint(
                    designComponent, hf, apex, axisDir)
                scaleEntities = adsk.core.ObjectCollection.create()
                scaleEntities.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    scaleEntities, basePoint,
                    adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # I. Loft -> curved tooth.
        order = sorted(
            range(len(segments)),
            key=lambda i: slabHeelFace(segments[i])[1])

        def slabToeFace(seg):
            best = None
            bestD = float('inf')
            for f in seg.faces:
                d = distAlong(f.centroid)
                if d < bestD:
                    bestD = d
                    best = f
            return best

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # toe-most segment's toe-facing face first
        toeSeg = segments[order[0]]
        loftInput.loftSections.add(slabToeFace(toeSeg))
        for i in order:
            (hf, _d) = slabHeelFace(segments[i])
            loftInput.loftSections.add(hf)
        curvedLoft = designComponent.features.loftFeatures.add(loftInput)
        curvedTooth = curvedLoft.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        # remove the segment scaffolding (loft captured their faces)
        for seg in segments:
            try:
                designComponent.features.removeFeatures.add(seg)
            except:
                pass

        # J. Flush trim.
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apexWorld, gearLabel)

    # =======================================================================
    # Spiral helpers
    # =======================================================================
    def _combine(self, base, a, e1, b=0.0, e2=None):
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, designComponent, line, refPlane, angleDeg):
        planes = designComponent.constructionPlanes
        pinput = planes.createInput()
        pinput.setByAngle(line,
                         adsk.core.ValueInput.createByString(f'{angleDeg} deg'),
                         refPlane)
        return planes.add(pinput)

    def _circleIntersectNearest(self, R, Cx, Cy, r_c, refX, refY):
        # intersect apex circle radius R (center origin) with cutter circle
        # (center (Cx,Cy), radius r_c); keep the solution nearest (refX,refY).
        d = math.hypot(Cx, Cy)
        if d == 0:
            raise Exception('cutter circle centered at apex; cannot intersect')
        # distance from origin to the radical line
        a = (R * R - r_c * r_c + d * d) / (2.0 * d)
        h2 = R * R - a * a
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)
        # base point along the center line
        ux, uy = Cx / d, Cy / d
        px = a * ux
        py = a * uy
        # perpendicular offset
        s1 = (px - h * uy, py + h * ux)
        s2 = (px + h * uy, py - h * ux)
        d1 = math.hypot(s1[0] - refX, s1[1] - refY)
        d2 = math.hypot(s2[0] - refX, s2[1] - refY)
        return s1 if d1 <= d2 else s2

    def _sliceTooth(self, designComponent, toothBody, parentToothPlane, apex,
                    coneVec, span, distAlong, gearLabel):
        planes = designComponent.constructionPlanes
        features = designComponent.features

        # choose the sign so the offset moves toward the apex
        planeOrigin = parentToothPlane.geometry.origin
        normal = parentToothPlane.geometry.normal
        toApex = adsk.core.Vector3D.create(
            apex.x - planeOrigin.x, apex.y - planeOrigin.y, apex.z - planeOrigin.z)
        sign = 1.0 if (toApex.dotProduct(normal) > 0) else -1.0

        def attemptCut(useSign):
            pieces = [toothBody]
            step = span / 6.0
            for k in range(_SPIRAL_SLICE_COUNT):
                offset = useSign * (k + 1) * step
                cpInput = planes.createInput()
                cpInput.setByOffset(parentToothPlane,
                                   adsk.core.ValueInput.createByReal(offset))
                cutPlane = planes.add(cpInput)
                newPieces = []
                for piece in pieces:
                    try:
                        splitInput = features.splitBodyFeatures.createInput(
                            piece, cutPlane, True)
                        split = features.splitBodyFeatures.add(splitInput)
                        for b in split.bodies:
                            newPieces.append(b)
                    except:
                        newPieces.append(piece)
                pieces = newPieces
            return pieces

        pieces = attemptCut(sign)
        if len(pieces) == 1:
            # retry with the opposite sign
            pieces = attemptCut(-sign)
        if len(pieces) == 1:
            raise Exception(
                f'{gearLabel}: spiral slice failed to cut the tooth '
                f'(1 piece; span={span:.4f}, sign tried={sign})')
        return pieces

    def _heelFaceRootBasePoint(self, designComponent, heelFace, apex, axisDir):
        # root corners = the two vertices nearest the shaft axis.
        verts = []
        for vtx in heelFace.vertices:
            verts.append(vtx.geometry)

        def perpDist(p):
            ap = adsk.core.Vector3D.create(p.x - apex.x, p.y - apex.y, p.z - apex.z)
            along = ap.dotProduct(axisDir)
            perp = adsk.core.Vector3D.create(
                ap.x - along * axisDir.x,
                ap.y - along * axisDir.y,
                ap.z - along * axisDir.z)
            return perp.length

        verts.sort(key=perpDist)
        r1 = verts[0]
        r2 = verts[1]
        midW = adsk.core.Point3D.create(
            (r1.x + r2.x) / 2.0, (r1.y + r2.y) / 2.0, (r1.z + r2.z) / 2.0)
        # place a sketch point on the heel face mapped via a sketch on that face.
        faceSketch = designComponent.sketches.add(heelFace)
        faceSketch.isVisible = False
        return faceSketch.sketchPoints.add(faceSketch.modelToSketchSpace(midW))

    # =======================================================================
    # Conical cuts / straight-tooth flush trim
    # =======================================================================
    def _surfaceDistance(self, surface, worldPoint):
        try:
            evaluator = surface.evaluator
            (ok, param) = evaluator.getParameterAtPoint(worldPoint)
            if not ok:
                return float('inf')
            (ok2, projected) = evaluator.getPointAtParameter(param)
            if not ok2:
                return float('inf')
            return projected.distanceTo(worldPoint)
        except:
            return float('inf')

    def _findConeFacesByMidpoint(self, frustumBody, edgeMidWorld):
        coneFaces = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                dist = self._surfaceDistance(face.geometry, edgeMidWorld)
                coneFaces.append((dist, face))
        coneFaces.sort(key=lambda t: t[0])
        return coneFaces

    def _applyConicalCut(self, designComponent, targetBody, frustumBody,
                        edgeMidWorld, apexWorld, gearLabel, cutLabel,
                        lenient=False):
        features = designComponent.features
        coneFaces = self._findConeFacesByMidpoint(frustumBody, edgeMidWorld)
        history = []
        for (dist, face) in coneFaces:
            try:
                splitInput = features.splitBodyFeatures.createInput(
                    targetBody, face, True)
                split = features.splitBodyFeatures.add(splitInput)
                if split.bodies.count > 1:
                    pieces = [split.bodies.item(i) for i in range(split.bodies.count)]
                    keeper = self._selectKeeper(
                        designComponent, pieces, apexWorld, gearLabel, cutLabel)
                    futil.log(
                        f'{gearLabel} {cutLabel}: split into {split.bodies.count} '
                        f'pieces (face dist={dist})', force_console=True)
                    return keeper
                else:
                    history.append(f'dist={dist}: split produced <=1 piece')
            except RuntimeError as e:
                history.append(f'dist={dist}: {str(e)}')
        # no face split the target
        message = (f'{gearLabel} {cutLabel}: no cone face split the tooth body '
                   f'(faces found={len(coneFaces)}; history={history})')
        if lenient:
            # propagate the non-intersect signal as a RuntimeError carrying the text
            raise RuntimeError(message)
        raise RuntimeError(message)

    def _selectKeeper(self, designComponent, pieces, apexWorld, gearLabel, cutLabel):
        nonApex = []
        for piece in pieces:
            containment = piece.pointContainment(apexWorld)
            if containment == adsk.fusion.PointContainment.PointInsidePointContainment:
                designComponent.features.removeFeatures.add(piece)
            else:
                nonApex.append(piece)
        if len(nonApex) == 0:
            raise RuntimeError(
                f'{gearLabel} {cutLabel}: no non-apex piece after cut')
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for extra in nonApex[1:]:
            designComponent.features.removeFeatures.add(extra)
        return keeper

    def _cutConicalEnds(self, designComponent, toothBody, gearBody,
                       toeMid, heelMid, apexWorld, gearLabel):
        # toe cut first
        keeper = self._applyConicalCut(
            designComponent, toothBody, gearBody, toeMid, apexWorld,
            gearLabel, 'toe cut (#1)', lenient=False)
        # heel cut on the keeper alone -- lenient on non-intersection
        try:
            keeper = self._applyConicalCut(
                designComponent, keeper, gearBody, heelMid, apexWorld,
                gearLabel, 'heel cut (#2)', lenient=True)
        except RuntimeError as e:
            msg = str(e)
            if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                futil.log(
                    f'{gearLabel} heel cut (#2): tool did not intersect, kept intact',
                    force_console=True)
            else:
                raise
        return keeper

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, boreDiameter_cm):
        features = designComponent.features
        planes = designComponent.constructionPlanes

        # bore plane normal to the shaft at its start
        bpInput = planes.createInput()
        bpInput.setByDistanceOnPath(shaftAxisEdge,
                                   adsk.core.ValueInput.createByReal(0.0))
        borePlane = planes.add(bpInput)
        borePlane.name = f'{gearBody.name} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearBody.name} Bore'
        boreSketch.isVisible = True
        circles = boreSketch.sketchCurves.sketchCircles
        dims = boreSketch.sketchDimensions

        circle = circles.addByCenterRadius(
            boreSketch.originPoint.geometry, boreDiameter_cm / 2.0)
        circle.centerSketchPoint.isFixed = True
        dims.addDiameterDimension(
            circle,
            adsk.core.Point3D.create(boreDiameter_cm / 2.0, 0, 0)).parameter.value = boreDiameter_cm

        if not boreSketch.isFullyConstrained:
            raise Exception(f'{gearBody.name} bore sketch is not fully constrained')

        boreProfile = boreSketch.profiles.item(0)
        extInput = features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        largeDist = adsk.core.ValueInput.createByReal(self._coneDistance_cm * 2.0)
        extInput.setSymmetricExtent(largeDist, False)
        extInput.participantBodies = [gearBody]
        features.extrudeFeatures.add(extInput)

    # =======================================================================
    # Cleanup
    # =======================================================================
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            for sketch in component.sketches:
                token = sketch.entityToken
                if token not in seen:
                    seen.add(token)
                    sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                token = plane.entityToken
                if token not in seen:
                    seen.add(token)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                token = axis.entityToken
                if token not in seen:
                    seen.add(token)
                    axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
