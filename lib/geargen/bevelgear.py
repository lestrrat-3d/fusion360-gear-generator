# Bevel gear generator, transliterated from spec/bevelgear/steps.md (S1-S34).
#
# Standalone generator: does NOT subclass base.Generator and uses no GenerationContext
# ([S3]). No live Fusion user parameters are registered; every value is precomputed in
# Python (internal cm / radians) and written into geometry numerically ([PB-PRECOMPUTED-MODE]).

import math

import adsk.core
import adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_selection, get_boolean
from .utilities import find_profile_by_curve_counts, get_normal
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator


# ---- S1: dialog input ids (20 inputs, 20 constants, in table row order) ----
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


class BevelGearCommandInputsConfigurator:
    """[S1] [S2] Dialog inputs and their conditional visibility."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        plane = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        plane.setSelectionLimits(1, 1)

        centerPoint = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerPoint.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerPoint.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerPoint.setSelectionLimits(1, 1)

        parent = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parent.setSelectionLimits(1, 1)
        parent.addSelection(get_design().rootComponent)

        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        hand = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
        hand.listItems.add(_HAND_RIGHT, True)
        hand.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
            adsk.core.ValueInput.createByReal(0))
        inputs.addValueInput(
            INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

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


# ---- pure-math helpers (no Fusion calls) ----

def _midpoint(p1, p2):
    return adsk.core.Point3D.create(
        (p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0)


def _dist_point_to_line_2d(p, lp1, lp2):
    dx = lp2.x - lp1.x
    dy = lp2.y - lp1.y
    length = math.hypot(dx, dy)
    if length < 1e-12:
        return 0.0
    return abs((p.x - lp1.x) * dy - (p.y - lp1.y) * dx) / length


def _project_point_onto_line_2d(p, lp1, lp2):
    dx = lp2.x - lp1.x
    dy = lp2.y - lp1.y
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return adsk.core.Point3D.create(lp1.x, lp1.y, 0)
    t = ((p.x - lp1.x) * dx + (p.y - lp1.y) * dy) / length2
    return adsk.core.Point3D.create(lp1.x + t * dx, lp1.y + t * dy, 0)


class BevelGearGenerator:
    """[S3] Standalone bevel-gear generator. One class builds both straight and
    spiral bevels; the spiral is a branch inside the tooth-body step, gated on
    the Mean Spiral Angle, not a separate subclass or command."""

    # [S25] Tunable crown relief; 0 disables the crown.
    _CROWN_PER_RAD = 0.5
    # [S32] The pinion's own mesh phase is 0 for both straight and spiral builds.
    _PINION_MESH_PHASE_TEETH = 0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    def deleteComponent(self):
        # [S34] Bevel registers no user parameters, so there is nothing to clean up
        # beyond rolling back the occurrence tree.
        if self.bevelOccurrence is not None:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = None

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)

        self._resolveConeGeometry(module, drivingTeeth, pinionTeeth, shaftAngle_deg)
        self._resolveBaseHeights(module, drivingTeeth, pinionTeeth)

        self._createComponents(parentComponent)
        self._buildAnchorSketch(targetPlane, centerPoint)
        self._buildGearProfilesPlane(targetPlane)
        pinionCtx, drivingCtx = self._buildGearProfiles(
            targetPlane, module, drivingTeeth, pinionTeeth)

        # [S11] Profile and body are built INTERLEAVED per gear: pinion profile,
        # pinion body, driving profile, driving body.
        for ctx in (pinionCtx, drivingCtx):
            self._buildGearProfile(ctx)
            self._buildGearBody(ctx)

        # [S34] Cleanup: hide construction geometry across the whole Bevel Gear tree.
        solids.hide_construction_geometry(self.bevelComponent)

    # ------------------------------------------------------------------
    # S3: read every input and check its range
    # ------------------------------------------------------------------
    def _readInputs(self, inputs: adsk.core.CommandInputs):
        unitsManager = self.design.unitsManager

        parentComponent: adsk.fusion.Component = get_selection(inputs, INPUT_ID_PARENT)[0]
        targetPlane: adsk.core.Base = get_selection(inputs, INPUT_ID_PLANE)[0]
        centerPoint: adsk.core.Base = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]

        def evalExpr(inputId, units, inputs: adsk.core.CommandInputs = inputs,
                     unitsManager: adsk.core.UnitsManager = unitsManager):
            item = inputs.itemById(inputId)
            return unitsManager.evaluateExpression(item.expression, units)

        module = evalExpr(INPUT_ID_MODULE, '')
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))

        self._module_mm = module
        self._drivingBaseHeightInput_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeightInput_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidthInput_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = evalExpr(INPUT_ID_TOOTH_SPACING, 'mm')
        self._spiralAngle_rad = evalExpr(INPUT_ID_SPIRAL_ANGLE, 'deg')

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedHand = handInput.selectedItem
        self._hand = selectedHand.name if selectedHand is not None else _HAND_RIGHT

        self._cutterRadius_cm = evalExpr(INPUT_ID_CUTTER_RADIUS, 'mm')
        self._toeExtension_pct = evalExpr(INPUT_ID_TOE_EXTENSION, '')
        self._drivingToeRadius_cm = evalExpr(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        self._pinionToeRadius_cm = evalExpr(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        # -- range checks, in the order the spec lists them --
        if module <= 0:
            raise Exception('Module must be greater than 0')
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')
        if shaftAngle_deg < 30:
            raise Exception('Shaft Angle must be at least 30 degrees')
        # Shaft Angle's upper bound depends on both tooth counts; checked in
        # _resolveConeGeometry once they are both read and coerced.

        for label, value in (
            ('Driving Gear Base Height', self._drivingBaseHeightInput_cm),
            ('Pinion Gear Base Height', self._pinionBaseHeightInput_cm),
            ('Driving Gear Bore Diameter', self._drivingBore_cm),
            ('Pinion Gear Bore Diameter', self._pinionBore_cm),
            ('Face Width', self._faceWidthInput_cm),
            ('Tooth Spacing', self._toothSpacing_cm),
            ('Driving Gear Toe Radius', self._drivingToeRadius_cm),
            ('Pinion Gear Toe Radius', self._pinionToeRadius_cm),
            ('Cutter Radius', self._cutterRadius_cm),
        ):
            if value < 0:
                raise Exception(f'{label} must not be negative')

        spiralAngle_deg = math.degrees(self._spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')

        if self._toeExtension_pct < 0 or self._toeExtension_pct > 100:
            raise Exception('Toe Extension must be in [0, 100]')

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # ------------------------------------------------------------------
    # S4: cone angles, Pitch Cone Distance, per-gear windows
    # ------------------------------------------------------------------
    def _resolveConeGeometry(self, module, drivingTeeth, pinionTeeth, shaftAngle_deg):
        self._shaftAngle_deg = shaftAngle_deg

        PPD = to_cm(module * pinionTeeth)
        DPD = to_cm(module * drivingTeeth)
        sigma = math.radians(shaftAngle_deg)

        smaller = min(DPD, PPD)
        larger = max(DPD, PPD)
        coneLimitDeg = math.degrees(math.acos(-smaller / larger))
        # The cone-angle half is exclusive; the 150-degree ceiling is inclusive.
        if coneLimitDeg <= 150.0:
            if shaftAngle_deg >= coneLimitDeg:
                raise Exception(
                    f'Shaft Angle must be below {coneLimitDeg:.4f} degrees '
                    f'(Maximum Shaft Angle)')
        else:
            if shaftAngle_deg > 150.0:
                raise Exception('Shaft Angle must not exceed 150 degrees (Maximum Shaft Angle)')

        gamma_p = math.atan2(math.sin(sigma) * PPD, DPD + PPD * math.cos(sigma))
        gamma_g = sigma - gamma_p
        R = (PPD / 2.0) / math.sin(gamma_p)
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))

        minFloorPinion = 5.27 * math.cos(gamma_p)
        if pinionTeeth < minFloorPinion:
            raise Exception(f'Pinion Gear Teeth must be at least {minFloorPinion:.4f}')
        minFloorDriving = 5.27 * math.cos(gamma_g)
        if drivingTeeth < minFloorDriving:
            raise Exception(f'Driving Gear Teeth must be at least {minFloorDriving:.4f}')

        self._PPD_cm = PPD
        self._DPD_cm = DPD
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._R_cm = R
        self._coneDistance_cm = coneDistance_cm

    def _resolveBaseHeights(self, module, drivingTeeth, pinionTeeth):
        module_cm = to_cm(module)
        r_g = self._DPD_cm / 2.0
        r_p = self._PPD_cm / 2.0
        gamma_g = self._gamma_g
        gamma_p = self._gamma_p

        minDriving = 1.05 * 1.25 * module_cm * math.sin(gamma_g)
        maxDriving = 0.95 * (r_g - 1.25 * module_cm * math.cos(gamma_g)) * math.tan(gamma_g)
        minPinion = 1.05 * 1.25 * module_cm * math.sin(gamma_p)
        maxPinion = 0.95 * (r_p - 1.25 * module_cm * math.cos(gamma_p)) * math.tan(gamma_p)

        if self._drivingBaseHeightInput_cm > 0:
            if self._drivingBaseHeightInput_cm < minDriving:
                raise Exception(
                    f'Driving Gear Base Height must be at least {to_mm(minDriving):.4f} mm')
            if self._drivingBaseHeightInput_cm > maxDriving:
                raise Exception(
                    f'Driving Gear Base Height must not exceed {to_mm(maxDriving):.4f} mm')
            drivingResolved = self._drivingBaseHeightInput_cm
        else:
            fallback = to_cm(module * drivingTeeth) / 8.0
            drivingResolved = min(max(fallback, minDriving), maxDriving)

        if self._pinionBaseHeightInput_cm > 0:
            if self._pinionBaseHeightInput_cm < minPinion:
                raise Exception(
                    f'Pinion Gear Base Height must be at least {to_mm(minPinion):.4f} mm')
            if self._pinionBaseHeightInput_cm > maxPinion:
                raise Exception(
                    f'Pinion Gear Base Height must not exceed {to_mm(maxPinion):.4f} mm')
            pinionResolved = self._pinionBaseHeightInput_cm
        else:
            scaled = drivingResolved * (pinionTeeth / drivingTeeth)
            pinionResolved = min(scaled, maxPinion)

        self._drivingBaseHeight_cm = drivingResolved
        self._pinionBaseHeight_cm = pinionResolved

    # ------------------------------------------------------------------
    # S6 / S7: Bevel Gear and Design components
    # ------------------------------------------------------------------
    def _createComponents(self, parentComponent: adsk.fusion.Component):
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

    # ------------------------------------------------------------------
    # S8: Anchor sketch
    # ------------------------------------------------------------------
    def _buildAnchorSketch(self, targetPlane: adsk.core.Base, centerPoint: adsk.core.Base):
        sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projectedCenter = sketch.project(centerPoint).item(0)
        c = projectedCenter.geometry
        p1 = adsk.core.Point3D.create(c.x - 0.5, c.y, 0)
        p2 = adsk.core.Point3D.create(c.x + 0.5, c.y, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)

        textPoint = adsk.core.Point3D.create(c.x, c.y + 0.2, 0)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)

        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorCenterPoint = projectedCenter
        self._anchorLine = anchorLine

    # ------------------------------------------------------------------
    # S9: Gear Profiles Plane
    # ------------------------------------------------------------------
    def _buildGearProfilesPlane(self, targetPlane: adsk.core.Base):
        planeInput = self.designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            self._anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = self.designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # ------------------------------------------------------------------
    # S10: Gear Profiles sketch -- the section 2 lattice
    # ------------------------------------------------------------------
    def _buildGearProfiles(self, targetPlane: adsk.core.Base, module, drivingTeeth, pinionTeeth):
        designComponent = self.designComponent
        sketch = designComponent.sketches.add(self._gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch

        lines = sketch.sketchCurves.sketchLines
        dims = sketch.sketchDimensions
        cons = sketch.geometricConstraints
        Aligned = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation
        P3 = adsk.core.Point3D.create

        module_cm = to_cm(module)
        dedendum_cm = 1.25 * module_cm
        R = self._R_cm
        gamma_p = self._gamma_p
        gamma_g = self._gamma_g
        PPD = self._PPD_cm
        DPD = self._DPD_cm
        drivingBaseHeight_cm = self._drivingBaseHeight_cm
        pinionBaseHeight_cm = self._pinionBaseHeight_cm
        sigma = math.radians(self._shaftAngle_deg)

        # -- project the Anchor Sketch's centre point and line --
        projectedCenter = sketch.project(self._anchorCenterPoint).item(0)
        projectedAnchorLine = sketch.project(self._anchorLine).item(0)

        c = projectedCenter.geometry
        lineGeom = projectedAnchorLine.geometry
        dxAnchor = lineGeom.endPoint.x - lineGeom.startPoint.x
        dyAnchor = lineGeom.endPoint.y - lineGeom.startPoint.y
        anchorLen = math.hypot(dxAnchor, dyAnchor)
        d = (dxAnchor / anchorLen, dyAnchor / anchorLen)
        perp = (-d[1], d[0])

        # [BEVEL-F-GROW-SIDE]: orient perp toward the target plane's normal (world test).
        normal = targetPlane.geometry.normal
        probeWorld = sketch.sketchToModelSpace(P3(c.x + perp[0], c.y + perp[1], 0))
        cWorld = sketch.sketchToModelSpace(P3(c.x, c.y, 0))
        worldDir = (probeWorld.x - cWorld.x, probeWorld.y - cWorld.y, probeWorld.z - cWorld.z)
        dot = worldDir[0] * normal.x + worldDir[1] * normal.y + worldDir[2] * normal.z
        if dot < 0:
            perp = (-perp[0], -perp[1])

        # -- Centre -> Apex --
        apexDist = R * math.cos(gamma_g) + drivingBaseHeight_cm
        apexPt = P3(c.x + perp[0] * apexDist, c.y + perp[1] * apexDist, 0)
        centerToApex = lines.addByTwoPoints(P3(c.x, c.y, 0), apexPt)
        centerToApex.isConstruction = True
        cons.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        cons.addPerpendicular(centerToApex, projectedAnchorLine)
        apex = centerToApex.endSketchPoint
        self._centerToApex = centerToApex

        # -- Driving Gear Shaft Axis --
        bEndPt = P3(c.x + perp[0] * drivingBaseHeight_cm, c.y + perp[1] * drivingBaseHeight_cm, 0)
        drivingShaftAxis = (
            lines.addByTwoPoints(P3(apexPt.x, apexPt.y, 0), bEndPt)
        )
        drivingShaftAxis.isConstruction = True
        cons.addCoincident(drivingShaftAxis.startSketchPoint, apex)
        cons.addParallel(drivingShaftAxis, centerToApex)
        B = drivingShaftAxis.endSketchPoint
        drivingDir = (-perp[0], -perp[1])

        # -- Pinion Gear Shaft Axis: rotate +/- Sigma, keep the greater-X candidate --
        def rotate2d(vx, vy, ang):
            ca, sa = math.cos(ang), math.sin(ang)
            return (vx * ca - vy * sa, vx * sa + vy * ca)

        candPlus = rotate2d(drivingDir[0], drivingDir[1], sigma)
        candMinus = rotate2d(drivingDir[0], drivingDir[1], -sigma)
        pinionDir = (candPlus if (apexPt.x + candPlus[0]) > (apexPt.x + candMinus[0])
                     else candMinus)

        aDist = R * math.cos(gamma_p)
        aPt = P3(apexPt.x + pinionDir[0] * aDist, apexPt.y + pinionDir[1] * aDist, 0)
        pinionShaftAxis = lines.addByTwoPoints(P3(apexPt.x, apexPt.y, 0), aPt)
        pinionShaftAxis.isConstruction = True
        cons.addCoincident(pinionShaftAxis.startSketchPoint, apex)
        A = pinionShaftAxis.endSketchPoint

        bisector = (pinionDir[0] + drivingDir[0], pinionDir[1] + drivingDir[1])
        bnorm = math.hypot(*bisector)
        bisector = perp if bnorm < 1e-9 else (bisector[0] / bnorm, bisector[1] / bnorm)
        angTextPt = P3(apexPt.x + bisector[0] * (PPD / 4.0), apexPt.y + bisector[1] * (PPD / 4.0), 0)
        angDim = dims.addAngularDimension(drivingShaftAxis, pinionShaftAxis, angTextPt)
        angDim.parameter.value = sigma

        # -- A -> Apex2 drop, perpendicular to the Pinion Shaft Axis, toward B --
        perpPinion = (-pinionDir[1], pinionDir[0])
        abLen = math.hypot(bEndPt.x - aPt.x, bEndPt.y - aPt.y)
        abDir = ((bEndPt.x - aPt.x) / abLen, (bEndPt.y - aPt.y) / abLen)
        if perpPinion[0] * abDir[0] + perpPinion[1] * abDir[1] < 0:
            perpPinion = (-perpPinion[0], -perpPinion[1])
        apex2SeedA = P3(aPt.x + perpPinion[0] * (PPD / 2.0), aPt.y + perpPinion[1] * (PPD / 2.0), 0)
        dropA = lines.addByTwoPoints(P3(aPt.x, aPt.y, 0), apex2SeedA)
        dropA.isConstruction = True
        cons.addCoincident(dropA.startSketchPoint, A)
        cons.addPerpendicular(dropA, pinionShaftAxis)
        dimA = dims.addDistanceDimension(
            dropA.startSketchPoint, dropA.endSketchPoint, Aligned,
            _midpoint(P3(aPt.x, aPt.y, 0), apex2SeedA))
        dimA.parameter.value = PPD / 2.0
        apex2FromA = dropA.endSketchPoint

        # -- B -> Apex2 drop, perpendicular to the Driving Shaft Axis, toward A --
        perpDriving = (-drivingDir[1], drivingDir[0])
        baLen = math.hypot(aPt.x - bEndPt.x, aPt.y - bEndPt.y)
        baDir = ((aPt.x - bEndPt.x) / baLen, (aPt.y - bEndPt.y) / baLen)
        if perpDriving[0] * baDir[0] + perpDriving[1] * baDir[1] < 0:
            perpDriving = (-perpDriving[0], -perpDriving[1])
        apex2SeedB = P3(bEndPt.x + perpDriving[0] * (DPD / 2.0),
                         bEndPt.y + perpDriving[1] * (DPD / 2.0), 0)
        dropB = lines.addByTwoPoints(P3(bEndPt.x, bEndPt.y, 0), apex2SeedB)
        dropB.isConstruction = True
        cons.addCoincident(dropB.startSketchPoint, B)
        cons.addPerpendicular(dropB, drivingShaftAxis)
        dimB = dims.addDistanceDimension(
            dropB.startSketchPoint, dropB.endSketchPoint, Aligned,
            _midpoint(P3(bEndPt.x, bEndPt.y, 0), apex2SeedB))
        dimB.parameter.value = DPD / 2.0
        apex2FromB = dropB.endSketchPoint

        cons.addCoincident(apex2FromA, apex2FromB)
        apex2 = apex2FromA
        apex2Pos = apex2SeedA

        # -- Pitch Line: Apex -> Apex2 --
        pitchLine = (
            lines.addByTwoPoints(P3(apexPt.x, apexPt.y, 0), P3(apex2Pos.x, apex2Pos.y, 0))
        )
        pitchLine.isConstruction = True
        cons.addCoincident(pitchLine.startSketchPoint, apex)
        cons.addCoincident(pitchLine.endSketchPoint, apex2)

        # -- The two dedendum lines, perpendicular to the Pitch Line --
        pitchLen = math.hypot(apex2Pos.x - apexPt.x, apex2Pos.y - apexPt.y)
        pitchDir = ((apex2Pos.x - apexPt.x) / pitchLen, (apex2Pos.y - apexPt.y) / pitchLen)
        dedCand1 = (-pitchDir[1], pitchDir[0])
        dedCand2 = (-dedCand1[0], -dedCand1[1])
        towardAnchor = (-perp[0], -perp[1])
        if dedCand1[0] * towardAnchor[0] + dedCand1[1] * towardAnchor[1] > 0:
            drivingDedDir, pinionDedDir = dedCand1, dedCand2
        else:
            drivingDedDir, pinionDedDir = dedCand2, dedCand1

        DPt = P3(apex2Pos.x + drivingDedDir[0] * dedendum_cm,
                 apex2Pos.y + drivingDedDir[1] * dedendum_cm, 0)
        lineDrivingDed = (
            lines.addByTwoPoints(P3(apex2Pos.x, apex2Pos.y, 0), DPt)
        )
        lineDrivingDed.isConstruction = True
        cons.addCoincident(lineDrivingDed.startSketchPoint, apex2)
        cons.addPerpendicular(lineDrivingDed, pitchLine)
        dimD = dims.addDistanceDimension(
            lineDrivingDed.startSketchPoint, lineDrivingDed.endSketchPoint, Aligned,
            _midpoint(P3(apex2Pos.x, apex2Pos.y, 0), DPt))
        dimD.parameter.value = dedendum_cm
        D = lineDrivingDed.endSketchPoint

        CPt = P3(apex2Pos.x + pinionDedDir[0] * dedendum_cm,
                 apex2Pos.y + pinionDedDir[1] * dedendum_cm, 0)
        linePinionDed = (
            lines.addByTwoPoints(P3(apex2Pos.x, apex2Pos.y, 0), CPt)
        )
        linePinionDed.isConstruction = True
        cons.addCoincident(linePinionDed.startSketchPoint, apex2)
        cons.addPerpendicular(linePinionDed, pitchLine)
        dimC = dims.addDistanceDimension(
            linePinionDed.startSketchPoint, linePinionDed.endSketchPoint, Aligned,
            _midpoint(P3(apex2Pos.x, apex2Pos.y, 0), CPt))
        dimC.parameter.value = dedendum_cm
        C = linePinionDed.endSketchPoint

        # -- Root axes: Apex -> D, Apex -> C --
        rootAxisDriving = (
            lines.addByTwoPoints(P3(apexPt.x, apexPt.y, 0), P3(DPt.x, DPt.y, 0))
        )
        rootAxisDriving.isConstruction = True
        cons.addCoincident(rootAxisDriving.startSketchPoint, apex)
        cons.addCoincident(rootAxisDriving.endSketchPoint, D)

        rootAxisPinion = (
            lines.addByTwoPoints(P3(apexPt.x, apexPt.y, 0), P3(CPt.x, CPt.y, 0))
        )
        rootAxisPinion.isConstruction = True
        cons.addCoincident(rootAxisPinion.startSketchPoint, apex)
        cons.addCoincident(rootAxisPinion.endSketchPoint, C)

        # -- Pinion module chain: A -> E -> G, C -> H --
        EPt = P3(aPt.x + pinionDir[0] * module_cm, aPt.y + pinionDir[1] * module_cm, 0)
        lineAE = lines.addByTwoPoints(P3(aPt.x, aPt.y, 0), EPt)
        lineAE.isConstruction = True
        cons.addCoincident(lineAE.startSketchPoint, A)
        cons.addCollinear(lineAE, pinionShaftAxis)
        E = lineAE.endSketchPoint

        lineCE = lines.addByTwoPoints(P3(CPt.x, CPt.y, 0), P3(EPt.x, EPt.y, 0))
        lineCE.isConstruction = True
        cons.addCoincident(lineCE.startSketchPoint, C)
        cons.addCoincident(lineCE.endSketchPoint, E)
        cons.addPerpendicular(lineAE, lineCE)

        GPt = P3(EPt.x + pinionDir[0] * module_cm, EPt.y + pinionDir[1] * module_cm, 0)
        lineEG = lines.addByTwoPoints(P3(EPt.x, EPt.y, 0), GPt)
        lineEG.isConstruction = True
        cons.addCoincident(lineEG.startSketchPoint, E)
        cons.addCollinear(lineEG, lineAE)
        G = lineEG.endSketchPoint

        HPt = P3(CPt.x + pinionDedDir[0] * module_cm, CPt.y + pinionDedDir[1] * module_cm, 0)
        lineCH = lines.addByTwoPoints(P3(CPt.x, CPt.y, 0), HPt)
        lineCH.isConstruction = True
        cons.addCoincident(lineCH.startSketchPoint, C)
        cons.addCollinear(lineCH, linePinionDed)
        H = lineCH.endSketchPoint

        lineGH = lines.addByTwoPoints(P3(GPt.x, GPt.y, 0), P3(HPt.x, HPt.y, 0))
        lineGH.isConstruction = True
        cons.addCoincident(lineGH.startSketchPoint, G)
        cons.addCoincident(lineGH.endSketchPoint, H)
        cons.addPerpendicular(lineEG, lineGH)

        # -- Driving module chain: B -> F -> I, D -> J --
        FPt = P3(bEndPt.x + drivingDir[0] * module_cm, bEndPt.y + drivingDir[1] * module_cm, 0)
        lineBF = lines.addByTwoPoints(P3(bEndPt.x, bEndPt.y, 0), FPt)
        lineBF.isConstruction = True
        cons.addCoincident(lineBF.startSketchPoint, B)
        cons.addCollinear(lineBF, drivingShaftAxis)
        F = lineBF.endSketchPoint

        lineDF = lines.addByTwoPoints(P3(DPt.x, DPt.y, 0), P3(FPt.x, FPt.y, 0))
        lineDF.isConstruction = True
        cons.addCoincident(lineDF.startSketchPoint, D)
        cons.addCoincident(lineDF.endSketchPoint, F)
        cons.addPerpendicular(lineBF, lineDF)

        IPt = P3(FPt.x + drivingDir[0] * module_cm, FPt.y + drivingDir[1] * module_cm, 0)
        lineFI = lines.addByTwoPoints(P3(FPt.x, FPt.y, 0), IPt)
        lineFI.isConstruction = True
        cons.addCoincident(lineFI.startSketchPoint, F)
        cons.addCollinear(lineFI, lineBF)
        I = lineFI.endSketchPoint

        JPt = P3(DPt.x + drivingDedDir[0] * module_cm, DPt.y + drivingDedDir[1] * module_cm, 0)
        lineDJ = lines.addByTwoPoints(P3(DPt.x, DPt.y, 0), JPt)
        lineDJ.isConstruction = True
        cons.addCoincident(lineDJ.startSketchPoint, D)
        cons.addCollinear(lineDJ, lineDrivingDed)
        J = lineDJ.endSketchPoint

        lineIJ = lines.addByTwoPoints(P3(IPt.x, IPt.y, 0), P3(JPt.x, JPt.y, 0))
        lineIJ.isConstruction = True
        cons.addCoincident(lineIJ.startSketchPoint, I)
        cons.addCoincident(lineIJ.endSketchPoint, J)
        cons.addPerpendicular(lineFI, lineIJ)

        # -- The two base-height offsets --
        offB = dims.addOffsetDimension(
            dropB, lineIJ, _midpoint(P3(bEndPt.x, bEndPt.y, 0), P3(IPt.x, IPt.y, 0)))
        offB.parameter.value = drivingBaseHeight_cm
        offA = dims.addOffsetDimension(
            dropA, lineGH, _midpoint(P3(aPt.x, aPt.y, 0), P3(GPt.x, GPt.y, 0)))
        offA.parameter.value = pinionBaseHeight_cm

        # -- Pin the figure: I coincides with the projected centre --
        cons.addCoincident(I, projectedCenter)

        # -- The two back-cone points K, L --
        KPt = P3(GPt.x + pinionDir[0] * module_cm, GPt.y + pinionDir[1] * module_cm, 0)
        lineGK = lines.addByTwoPoints(P3(GPt.x, GPt.y, 0), KPt)
        lineGK.isConstruction = True
        cons.addCoincident(lineGK.startSketchPoint, G)
        cons.addCoincident(lineGK.endSketchPoint, pinionShaftAxis)
        cons.addCoincident(lineGK.endSketchPoint, linePinionDed)
        K = lineGK.endSketchPoint
        lineCK = lines.addByTwoPoints(P3(CPt.x, CPt.y, 0), KPt)
        lineCK.isConstruction = True
        cons.addCoincident(lineCK.startSketchPoint, C)
        cons.addCoincident(lineCK.endSketchPoint, K)

        LPt = P3(IPt.x + drivingDir[0] * module_cm, IPt.y + drivingDir[1] * module_cm, 0)
        lineIL = lines.addByTwoPoints(P3(IPt.x, IPt.y, 0), LPt)
        lineIL.isConstruction = True
        cons.addCoincident(lineIL.startSketchPoint, I)
        cons.addCoincident(lineIL.endSketchPoint, drivingShaftAxis)
        cons.addCoincident(lineIL.endSketchPoint, lineDrivingDed)
        L = lineIL.endSketchPoint
        lineDL = lines.addByTwoPoints(P3(DPt.x, DPt.y, 0), LPt)
        lineDL.isConstruction = True
        cons.addCoincident(lineDL.startSketchPoint, D)
        cons.addCoincident(lineDL.endSketchPoint, L)

        # -- Tooth-centre points K', L' (Tooth Spacing offset) --
        toothSpacing_cm = self._toothSpacing_cm
        if toothSpacing_cm <= 0:
            Kprime = K
            lineCKprime = lineCK
        else:
            KprimePt = P3(KPt.x + pinionDedDir[0] * toothSpacing_cm,
                          KPt.y + pinionDedDir[1] * toothSpacing_cm, 0)
            lineKKprime = lines.addByTwoPoints(P3(KPt.x, KPt.y, 0), KprimePt)
            lineKKprime.isConstruction = True
            cons.addCoincident(lineKKprime.startSketchPoint, K)
            cons.addCoincident(lineKKprime.endSketchPoint, linePinionDed)
            dimKp = dims.addDistanceDimension(
                lineKKprime.startSketchPoint, lineKKprime.endSketchPoint, Aligned,
                _midpoint(P3(KPt.x, KPt.y, 0), KprimePt))
            dimKp.parameter.value = toothSpacing_cm
            Kprime = lineKKprime.endSketchPoint
            lineCKprime = lines.addByTwoPoints(P3(CPt.x, CPt.y, 0), KprimePt)
            lineCKprime.isConstruction = True
            cons.addCoincident(lineCKprime.startSketchPoint, C)
            cons.addCoincident(lineCKprime.endSketchPoint, Kprime)

        if toothSpacing_cm <= 0:
            Lprime = L
            lineDLprime = lineDL
        else:
            LprimePt = P3(LPt.x + drivingDedDir[0] * toothSpacing_cm,
                          LPt.y + drivingDedDir[1] * toothSpacing_cm, 0)
            lineLLprime = lines.addByTwoPoints(P3(LPt.x, LPt.y, 0), LprimePt)
            lineLLprime.isConstruction = True
            cons.addCoincident(lineLLprime.startSketchPoint, L)
            cons.addCoincident(lineLLprime.endSketchPoint, lineDrivingDed)
            dimLp = dims.addDistanceDimension(
                lineLLprime.startSketchPoint, lineLLprime.endSketchPoint, Aligned,
                _midpoint(P3(LPt.x, LPt.y, 0), LprimePt))
            dimLp.parameter.value = toothSpacing_cm
            Lprime = lineLLprime.endSketchPoint
            lineDLprime = lines.addByTwoPoints(P3(DPt.x, DPt.y, 0), LprimePt)
            lineDLprime.isConstruction = True
            cons.addCoincident(lineDLprime.startSketchPoint, D)
            cons.addCoincident(lineDLprime.endSketchPoint, Lprime)

        # -- Resolve the Maximum Face Width from SOLVED geometry ([PB-SOLVED-GEOMETRY]) --
        Ageo, Bgeo = A.geometry, B.geometry
        Cgeo, Dgeo = C.geometry, D.geometry
        Hgeo, Jgeo = H.geometry, J.geometry
        distA_CH = _dist_point_to_line_2d(Ageo, Cgeo, Hgeo)
        distB_DJ = _dist_point_to_line_2d(Bgeo, Dgeo, Jgeo)
        maxFaceWidth = 0.95 * min(distA_CH, distB_DJ)
        self._faceWidthMax_cm = maxFaceWidth

        faceWidthInput = self._faceWidthInput_cm
        if faceWidthInput > 0:
            if faceWidthInput > maxFaceWidth:
                raise Exception(
                    f'Face Width must not exceed the maximum of {to_mm(maxFaceWidth):.4f} mm')
            faceWidthResolved = faceWidthInput
        else:
            faceWidthResolved = min(self._coneDistance_cm / 6.0, maxFaceWidth)
        self._faceWidthResolved_cm = faceWidthResolved

        apexToDed = math.sqrt(R ** 2 + dedendum_cm ** 2)
        self._apexToDed_cm = apexToDed
        rootLength0 = faceWidthResolved * apexToDed / R

        def toe_radius_ceiling(pitchRadius, gamma):
            return (pitchRadius - dedendum_cm * math.cos(gamma)) * (1.0 - faceWidthResolved / R)

        def toe_limit(resolvedToeRadius, gamma):
            gammaRoot = gamma - math.atan(dedendum_cm / R)
            return apexToDed - resolvedToeRadius / math.sin(gammaRoot)

        pinionToeCeiling = toe_radius_ceiling(PPD / 2.0, gamma_p)
        drivingToeCeiling = toe_radius_ceiling(DPD / 2.0, gamma_g)

        def resolve_toe_radius(userValue, pitchRadius, gamma, ceiling, label):
            if userValue > 0:
                if userValue >= ceiling:
                    raise Exception(
                        f'{label} Gear Toe Radius must be below {to_mm(ceiling):.4f} mm')
                return userValue
            return pitchRadius - faceWidthResolved / math.sin(gamma)

        pinionToeRadius = resolve_toe_radius(
            self._pinionToeRadius_cm, PPD / 2.0, gamma_p, pinionToeCeiling, 'Pinion')
        drivingToeRadius = resolve_toe_radius(
            self._drivingToeRadius_cm, DPD / 2.0, gamma_g, drivingToeCeiling, 'Driving')

        pinionToeLimit = toe_limit(pinionToeRadius, gamma_p)
        drivingToeLimit = toe_limit(drivingToeRadius, gamma_g)

        toeExtension_pct = self._toeExtension_pct
        if toeExtension_pct > 0:
            for label, limit, ceiling in (
                ('Pinion', pinionToeLimit, pinionToeCeiling),
                ('Driving', drivingToeLimit, drivingToeCeiling),
            ):
                if limit <= rootLength0:
                    raise Exception(
                        f'{label} Gear: Toe Extension requires a Toe Radius below '
                        f'{to_mm(ceiling):.4f} mm')
            minToeLimit = min(pinionToeLimit, drivingToeLimit)
            rootLength = rootLength0 + (toeExtension_pct / 100.0) * 0.99 * (minToeLimit - rootLength0)
        else:
            rootLength = rootLength0

        self._rootLength_cm = rootLength
        offsetValue = rootLength * R / apexToDed

        # -- Pinion toe line M -> N --
        fracM = 1.0 - rootLength / apexToDed
        Mseed = P3(apexPt.x + fracM * (Cgeo.x - apexPt.x), apexPt.y + fracM * (Cgeo.y - apexPt.y), 0)
        perpDistM = _dist_point_to_line_2d(Mseed, apexPt, Ageo)
        slideM = (perpDistM - pinionToeRadius) / math.cos(gamma_p)
        chLen = math.hypot(Hgeo.x - Cgeo.x, Hgeo.y - Cgeo.y)
        chDir = ((Hgeo.x - Cgeo.x) / chLen, (Hgeo.y - Cgeo.y) / chLen)
        Nseed = P3(Mseed.x + chDir[0] * slideM, Mseed.y + chDir[1] * slideM, 0)

        lineMN = lines.addByTwoPoints(Mseed, Nseed)
        lineMN.isConstruction = True
        cons.addCoincident(lineMN.startSketchPoint, rootAxisPinion)
        cons.addParallel(lineMN, lineCH)
        offMN = dims.addOffsetDimension(
            lineCH, lineMN, _midpoint(Mseed, P3(Cgeo.x, Cgeo.y, 0)))
        offMN.parameter.value = offsetValue
        M = lineMN.startSketchPoint
        N = lineMN.endSketchPoint

        lineMC = (
            lines.addByTwoPoints(P3(Mseed.x, Mseed.y, 0), P3(Cgeo.x, Cgeo.y, 0))
        )
        lineMC.isConstruction = True
        cons.addCoincident(lineMC.startSketchPoint, M)
        cons.addCoincident(lineMC.endSketchPoint, C)

        # -- Pinion front face A' -> N --
        AprimeSeed = _project_point_onto_line_2d(Nseed, apexPt, Ageo)
        lineNAprime = lines.addByTwoPoints(Nseed, AprimeSeed)
        lineNAprime.isConstruction = True
        cons.addCoincident(lineNAprime.startSketchPoint, N)
        Aprime = lineNAprime.endSketchPoint
        cons.addCoincident(Aprime, pinionShaftAxis)
        cons.addPerpendicular(lineNAprime, pinionShaftAxis)
        dimNAp = dims.addDistanceDimension(
            lineNAprime.startSketchPoint, lineNAprime.endSketchPoint, Aligned,
            _midpoint(Nseed, AprimeSeed))
        dimNAp.parameter.value = pinionToeRadius

        lineAprimeG = (
            lines.addByTwoPoints(P3(AprimeSeed.x, AprimeSeed.y, 0), P3(GPt.x, GPt.y, 0))
        )
        lineAprimeG.isConstruction = True
        cons.addCoincident(lineAprimeG.startSketchPoint, Aprime)
        cons.addCoincident(lineAprimeG.endSketchPoint, G)

        # -- Driving toe line O -> P (mirror of M -> N) --
        fracO = 1.0 - rootLength / apexToDed
        Oseed = P3(apexPt.x + fracO * (Dgeo.x - apexPt.x), apexPt.y + fracO * (Dgeo.y - apexPt.y), 0)
        perpDistO = _dist_point_to_line_2d(Oseed, apexPt, Bgeo)
        slideO = (perpDistO - drivingToeRadius) / math.cos(gamma_g)
        djLen = math.hypot(Jgeo.x - Dgeo.x, Jgeo.y - Dgeo.y)
        djDir = ((Jgeo.x - Dgeo.x) / djLen, (Jgeo.y - Dgeo.y) / djLen)
        Pseed = P3(Oseed.x + djDir[0] * slideO, Oseed.y + djDir[1] * slideO, 0)

        lineOP = lines.addByTwoPoints(Oseed, Pseed)
        lineOP.isConstruction = True
        cons.addCoincident(lineOP.startSketchPoint, rootAxisDriving)
        cons.addParallel(lineOP, lineDJ)
        offOP = dims.addOffsetDimension(
            lineDJ, lineOP, _midpoint(Oseed, P3(Dgeo.x, Dgeo.y, 0)))
        offOP.parameter.value = offsetValue
        O = lineOP.startSketchPoint
        Pd = lineOP.endSketchPoint

        lineOD = (
            lines.addByTwoPoints(P3(Oseed.x, Oseed.y, 0), P3(Dgeo.x, Dgeo.y, 0))
        )
        lineOD.isConstruction = True
        cons.addCoincident(lineOD.startSketchPoint, O)
        cons.addCoincident(lineOD.endSketchPoint, D)

        BprimeSeed = _project_point_onto_line_2d(Pseed, apexPt, Bgeo)
        linePBprime = lines.addByTwoPoints(Pseed, BprimeSeed)
        linePBprime.isConstruction = True
        cons.addCoincident(linePBprime.startSketchPoint, Pd)
        Bprime = linePBprime.endSketchPoint
        cons.addCoincident(Bprime, drivingShaftAxis)
        cons.addPerpendicular(linePBprime, drivingShaftAxis)
        dimPBp = dims.addDistanceDimension(
            linePBprime.startSketchPoint, linePBprime.endSketchPoint, Aligned,
            _midpoint(Pseed, BprimeSeed))
        dimPBp.parameter.value = drivingToeRadius

        lineBprimeI = (
            lines.addByTwoPoints(P3(BprimeSeed.x, BprimeSeed.y, 0), P3(IPt.x, IPt.y, 0))
        )
        lineBprimeI.isConstruction = True
        cons.addCoincident(lineBprimeI.startSketchPoint, Bprime)
        cons.addCoincident(lineBprimeI.endSketchPoint, I)

        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'gamma': gamma_p,
            'pitchDiameter_cm': PPD,
            'toothCenterPoint': Kprime,
            'toothCenterRefLine': lineCKprime,
            'hexVertices': [Aprime, G, H, C, M, N],
            'toeEdgePoints': (M, N),
            'heelEdgePoints': (C, H),
            'boreInput_cm': self._pinionBore_cm,
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'gamma': gamma_g,
            'pitchDiameter_cm': DPD,
            'toothCenterPoint': Lprime,
            'toothCenterRefLine': lineDLprime,
            'hexVertices': [Bprime, I, J, D, O, Pd],
            'toeEdgePoints': (O, Pd),
            'heelEdgePoints': (D, J),
            'boreInput_cm': self._drivingBore_cm,
        }
        return pinionCtx, drivingCtx

    # ------------------------------------------------------------------
    # S11-S15: per-gear tooth plane, virtual spur tooth, tooth axis,
    # Gear component, profile hexagon
    # ------------------------------------------------------------------
    def _buildGearProfile(self, ctx):
        designComponent = self.designComponent
        label = ctx['label']

        # S11: per-gear tooth plane
        toothPlane = solids.plane_by_angle(
            designComponent, ctx['toothCenterRefLine'], self._gearProfilesPlane, 90)
        toothPlane.name = f'{label} Plane'
        ctx['toothPlane'] = toothPlane

        # S12: the virtual spur tooth sketch
        pitchDia_cm = ctx['pitchDiameter_cm']
        gamma = ctx['gamma']
        module_mm = self._module_mm
        virtualPitchRadius_mm = (pitchDia_cm * 10.0 / 2.0) / math.cos(gamma)
        virtualTeeth = 2.0 * virtualPitchRadius_mm / module_mm
        rootSink_mm = 0.05 * 2.25 * module_mm

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{label} Tooth'

        proxy = VirtualSpurProxy(
            module_mm=module_mm, virtualTeeth=virtualTeeth, rootSink_mm=rootSink_mm)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))

        if not toothSketch.isFullyConstrained:
            # [PB-TEXT-HOLDS-DOF]: the four circle labels hold their own DOF; log, never raise.
            futil.log(
                f'{label} Tooth sketch reports under-constrained '
                f'(sketch text holds its own DOF; not raised, [PB-TEXT-HOLDS-DOF])')

        ctx['toothSketch'] = toothSketch
        ctx['toothEmbedded'] = proxy._lastToothEmbedded

        # S13: the per-gear tooth axis
        helperPlaneInput = designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(
            ctx['toothCenterRefLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperPlaneInput)

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Tooth Axis'
        ctx['toothAxis'] = toothAxis

        # S14: the per-gear Gear component
        gearOcc = self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOcc.component.name = f'{label} Gear'
        ctx['gearOccurrence'] = gearOcc

        # S15: the per-gear Profile sketch -- the frustum hexagon
        profileSketch = designComponent.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{label} Profile'
        newPts = [
            profileSketch.sketchPoints.add(profileSketch.modelToSketchSpace(src.worldGeometry))
            for src in ctx['hexVertices']
        ]
        n = len(newPts)
        hexLines = []
        for i in range(n):
            a = newPts[i]
            b = newPts[(i + 1) % n]
            hexLines.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(a, b))
        for hexLine in hexLines:
            hexLine.startSketchPoint.isFixed = True
            hexLine.endSketchPoint.isFixed = True

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{label} Profile sketch is not fully constrained')

        ctx['profileSketch'] = profileSketch
        # The hexagon's first edge is the gear's shaft axis for every body operation below.
        ctx['shaftAxisEdge'] = hexLines[0]

    # ------------------------------------------------------------------
    # S16-S33: revolve, loft, trim, (spiral S19-S27), pattern, combine,
    # bore, meshing rotation, move to component
    # ------------------------------------------------------------------
    def _buildGearBody(self, ctx):
        designComponent = self.designComponent
        label = ctx['label']

        # S16: revolve the hexagon into the Gear Body
        profileSketch: adsk.fusion.Sketch = ctx['profileSketch']
        shaftAxisEdge: adsk.fusion.SketchLine = ctx['shaftAxisEdge']
        profile = profileSketch.profiles.item(0)
        revInput = designComponent.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revFeature = designComponent.features.revolveFeatures.add(revInput)
        gearBody = revFeature.bodies.item(0)
        ctx['gearBody'] = gearBody

        # S17: loft the Apex sketch point to the tooth profile
        apexSketchPoint: adsk.fusion.SketchPoint = self._centerToApex.endSketchPoint
        toothSketch: adsk.fusion.Sketch = ctx['toothSketch']
        wantLines = 0 if ctx['toothEmbedded'] else 2
        toothProfile = find_profile_by_curve_counts(
            toothSketch, nurbs=2, arcs=2, lines=wantLines)

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        toothBody = loftFeature.bodies.item(0)

        toeP1, toeP2 = ctx['toeEdgePoints']
        heelP1, heelP2 = ctx['heelEdgePoints']
        toeMid = _midpoint(toeP1.worldGeometry, toeP2.worldGeometry)
        heelMid = _midpoint(heelP1.worldGeometry, heelP2.worldGeometry)
        apexWorld = apexSketchPoint.worldGeometry
        toeConeWorld = toeP1.worldGeometry
        heelConeWorld = heelP1.worldGeometry

        # S18/S19-S27: the straight (ψ = 0) or spiral (ψ > 0) tooth-body hook
        finalToothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
            ctx['toothPlane'], label, ctx['teeth'], ctx['gamma'])

        # S28: circular-pattern the tooth around the shaft axis
        toolColl = adsk.core.ObjectCollection.create()
        toolColl.add(finalToothBody)
        patternInput = designComponent.features.circularPatternFeatures.createInput(
            toolColl, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternFeature = designComponent.features.circularPatternFeatures.add(patternInput)

        allTeethColl = adsk.core.ObjectCollection.create()
        for i in range(patternFeature.bodies.count):
            allTeethColl.add(patternFeature.bodies.item(i))

        # S29: Combine-Join the patterned teeth into the Gear Body
        combineInput = designComponent.features.combineFeatures.createInput(
            gearBody, allTeethColl)
        designComponent.features.combineFeatures.add(combineInput)

        # S30/S31: the bore, only when enabled
        if self._boreEnable:
            self._buildBore(ctx)

        # S32: the meshing rotation
        if label == 'Driving':
            meshAngle = math.pi / ctx['teeth']
        else:
            meshAngle = self._pinionMeshPhase(ctx['teeth'])
        solids.rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, meshAngle)

        # S33: move the finished body into its gear component
        gearBody.moveToComponent(ctx['gearOccurrence'])

    def _pinionMeshPhase(self, pinionTeeth):
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # ------------------------------------------------------------------
    # S19-S27: spiral tooth-body transform (ψ > 0 only); S18 straight fallback
    # ------------------------------------------------------------------
    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                             toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                             shaftAxisEdge: adsk.fusion.SketchLine, apexWorld: adsk.core.Point3D,
                             apexSketchPoint: adsk.fusion.SketchPoint, toeMid: adsk.core.Point3D,
                             heelMid: adsk.core.Point3D, toeConeWorld: adsk.core.Point3D,
                             heelConeWorld: adsk.core.Point3D,
                             parentToothPlane: adsk.fusion.ConstructionPlane,
                             gearLabel, teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            # S18: byte-for-byte the prior (straight-bevel) behaviour.
            return solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        P3 = adsk.core.Point3D.create
        V3 = adsk.core.Vector3D.create

        # S19.A: gate and frame -- fix a swapped toe/heel before building coneVec.
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        startW = shaftAxisEdge.startSketchPoint.worldGeometry
        endW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir: adsk.core.Vector3D = V3(endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
        axisDir.normalize()

        coneRaw = (heelConeWorld.x - apexWorld.x, heelConeWorld.y - apexWorld.y,
                   heelConeWorld.z - apexWorld.z)
        coneLen = math.sqrt(sum(v * v for v in coneRaw))
        coneVec = V3(coneRaw[0] / coneLen, coneRaw[1] / coneLen, coneRaw[2] / coneLen)

        def cross(a, b):
            return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

        axisT = (axisDir.x, axisDir.y, axisDir.z)
        coneT = (coneVec.x, coneVec.y, coneVec.z)
        vT = cross(axisT, coneT)
        vLen = math.sqrt(sum(x * x for x in vT))
        v = V3(vT[0] / vLen, vT[1] / vLen, vT[2] / vLen)

        def dist_along(p):
            return ((p.x - apexWorld.x) * coneVec.x + (p.y - apexWorld.y) * coneVec.y
                    + (p.z - apexWorld.z) * coneVec.z)

        R_toe = dist_along(toeMid)
        R_heel = dist_along(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # S19: the cone-element sketch -- raw world coordinates, no modelToSketchSpace
        # (deliberate: this sketch is construction/reference-only, per the step's note).
        coneElementSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneElementSketch.name = f'{gearLabel} Cone Element'
        apexRaw = P3(apexWorld.x, apexWorld.y, apexWorld.z)
        farRaw = P3(apexWorld.x + R_heel * coneVec.x, apexWorld.y + R_heel * coneVec.y,
                    apexWorld.z + R_heel * coneVec.z)
        coneElementLine = coneElementSketch.sketchCurves.sketchLines.addByTwoPoints(apexRaw, farRaw)
        coneElementLine.isConstruction = True

        # S20: the Trace Plane
        tracePlane = solids.plane_by_angle(
            designComponent, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # S21.B: cutter-arc geometry
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        psi = self._spiralAngle_rad
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
        heel2d = solids.circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)

        # S21.C: the trace sketch
        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        def tanW(px, py):
            return solids.combine_point(apexWorld, px, coneVec, py, v)

        cutterCenterWorld = tanW(Cx, Cy)
        circle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(cutterCenterWorld, r_c)
        circle.isConstruction = True
        circle.centerSketchPoint.isFixed = True
        diaDim = traceSketch.sketchDimensions.addDiameterDimension(circle, tanW(Cx + r_c, Cy))
        diaDim.parameter.value = 2 * r_c

        startPt = tanW(toe2d[0], toe2d[1])
        meanPt = tanW(R_mean, 0)
        endPt = tanW(heel2d[0], heel2d[1])
        arc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(startPt, meanPt, endPt)
        traceSketch.geometricConstraints.addCoincident(arc.centerSketchPoint, circle.centerSketchPoint)
        radDim = traceSketch.sketchDimensions.addRadialDimension(arc, meanPt)
        radDim.parameter.value = r_c

        # S22: slice the straight tooth into cross-section slabs
        normal = get_normal(parentToothPlane)
        planeOrigin = parentToothPlane.geometry.origin
        testDot = ((apexWorld.x - planeOrigin.x) * normal.x
                   + (apexWorld.y - planeOrigin.y) * normal.y
                   + (apexWorld.z - planeOrigin.z) * normal.z)
        sign = 1.0 if testDot > 0 else -1.0

        offsets = [sign * (k + 1) * span / 6.0 for k in range(8)]
        pieces = solids.slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
        if len(pieces) <= 1:
            offsets = [-o for o in offsets]
            pieces = solids.slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)
            if len(pieces) <= 1:
                raise Exception(
                    f'{gearLabel}: tooth slice produced {len(pieces)} piece(s) '
                    f'(span={span}, sign tried={sign} and {-sign}) - cut planes missed the tooth')

        # S23: order the segments and drop the apex scrap
        def com_dist_along(body):
            return dist_along(body.physicalProperties.centerOfMass)

        pieces.sort(key=com_dist_along)
        scrap = pieces[0]
        segments = pieces[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(f'{gearLabel}: no segments remain after dropping the apex scrap')

        # S24: twist each segment about the shaft axis
        def heel_face_dist(body):
            best_face = None
            best_dist = None
            for face in body.faces:
                d = dist_along(face.centroid)
                if best_dist is None or d > best_dist:
                    best_dist = d
                    best_face = face
            return best_face, best_dist

        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        for seg in segments:
            _, R_heelFace = heel_face_dist(seg)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            if ang == 0:
                # [PB-MOVE-ROTATE]: a zero angle is a no-op, not a move -- the mid-face
                # section is deliberately left unrotated.
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            coll = adsk.core.ObjectCollection.create()
            coll.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(coll)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # S25: the lengthwise crown
        orderedAfterTwist = sorted(segments, key=lambda s: heel_face_dist(s)[1])
        toScale = orderedAfterTwist[:-1]  # skip the outermost (heel) segment

        self.designOccurrence.activate()
        try:
            for seg in toScale:
                heelFace, R_heelFace = heel_face_dist(seg)
                heelFace: adsk.fusion.BRepFace = heelFace  # re-typed: heel_face_dist() is untyped
                u = (R_heel - R_heelFace) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor} <= 0 at u={u}')

                candidates = []
                for i in range(heelFace.vertices.count):
                    vtx = heelFace.vertices.item(i)
                    p = vtx.geometry
                    vec = (p.x - apexWorld.x, p.y - apexWorld.y, p.z - apexWorld.z)
                    along = vec[0] * axisDir.x + vec[1] * axisDir.y + vec[2] * axisDir.z
                    perpV = (vec[0] - along * axisDir.x, vec[1] - along * axisDir.y,
                             vec[2] - along * axisDir.z)
                    perpDist = math.sqrt(sum(c * c for c in perpV))
                    candidates.append((perpDist, p))
                candidates.sort(key=lambda t: t[0])
                p1, p2 = candidates[0][1], candidates[1][1]
                rootMid = P3((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0)

                baseSketch = designComponent.sketches.add(heelFace)
                baseSketchPoint = baseSketch.sketchPoints.add(
                    baseSketch.modelToSketchSpace(rootMid))

                coll = adsk.core.ObjectCollection.create()
                coll.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    coll, baseSketchPoint, adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # S26: loft the curved tooth -- re-sort AFTER twist and crown
        finalOrder = sorted(segments, key=lambda s: heel_face_dist(s)[1])

        def toe_face(body):
            best_face = None
            best_dist = None
            for face in body.faces:
                d = dist_along(face.centroid)
                if best_dist is None or d < best_dist:
                    best_dist = d
                    best_face = face
            return best_face

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toe_face(finalOrder[0]))
        for seg in finalOrder:
            heelFace, _ = heel_face_dist(seg)
            loftInput.loftSections.add(heelFace)
        loftFeature = designComponent.features.loftFeatures.add(loftInput)
        curvedTooth = loftFeature.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in finalOrder:
            designComponent.features.removeFeatures.add(seg)

        # S27: trim the curved tooth flush
        return solids.cut_conical_ends(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # ------------------------------------------------------------------
    # S30/S31: the bore sketch and the through-cut
    # ------------------------------------------------------------------
    def _buildBore(self, ctx):
        designComponent = self.designComponent
        label = ctx['label']

        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            ctx['shaftAxisEdge'], adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{label} Bore'

        boreDiameter_cm = (ctx['boreInput_cm'] if ctx['boreInput_cm'] > 0
                            else ctx['pitchDiameter_cm'] / 4.0)
        radius = boreDiameter_cm / 2.0
        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        dim = boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, 0, 0))
        dim.parameter.value = boreDiameter_cm

        if not boreSketch.isFullyConstrained:
            raise Exception(f'{label} Bore sketch is not fully constrained')

        boreProfile = boreSketch.profiles.item(0)
        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [ctx['gearBody']]
        designComponent.features.extrudeFeatures.add(extrudeInput)
