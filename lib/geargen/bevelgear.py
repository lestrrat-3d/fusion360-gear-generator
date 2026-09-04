import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from .solids import (cut_conical_ends, slice_body_by_offset_planes, rotate_body_about_edge,
                     plane_by_angle, combine_point, circle_intersect_nearest,
                     hide_construction_geometry)
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy


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
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'
_CROWN_PER_RAD = 0.5
_PINION_MESH_PHASE_TEETH = 0


# --- small local helpers -------------------------------------------------------------------
#
# These name plain arithmetic / point construction, not Fusion calls; see the per-step
# "check-step-calls: ignore" notes in spec/bevelgear/steps.md for the ones they cover
# (normalize, distAlong, distanceTo, combine_point's own 2-D coordinate helper, etc).

def _point2(p):
    """[PB-POINT-HELPER] Tolerate a raw (x, y) tuple or an object with .x/.y
    (Point3D / SketchPoint.geometry) — this step mixes seed tuples with solved geometry."""
    if isinstance(p, tuple):
        return adsk.core.Point3D.create(p[0], p[1], 0)
    return adsk.core.Point3D.create(p.x, p.y, 0)


def _midpoint2(p0, p1):
    return ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)


def _midpointWorld(p0, p1):
    return adsk.core.Point3D.create(
        (p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0, (p0.z + p1.z) / 2.0)


def _lineIntersect2(p0, dir0, p1, p2):
    """Intersect the line through p0 with direction dir0 with the line through p1-p2.
    All 2-D. Used to seed a point that a later addCoincident pins onto the second line."""
    x1, y1 = p0
    dx1, dy1 = dir0
    x2, y2 = p1
    x3, y3 = p2
    dx2, dy2 = x3 - x2, y3 - y2
    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-12:
        return (x2, y2)
    t = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / denom
    return (x1 + dx1 * t, y1 + dy1 * t)


def _perpDistance2(pt, linePt1, linePt2):
    """Perpendicular distance from a 2-D point to the infinite line through linePt1-linePt2."""
    x1, y1 = linePt1
    x2, y2 = linePt2
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    return abs(dx * (y1 - pt[1]) - (x1 - pt[0]) * dy) / length


class BevelGearCommandInputsConfigurator:
    """S2, S3: the command dialog and its one conditional-visibility rule."""

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

        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))
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

        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)

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
        except Exception:
            hand.isVisible = True
            cutter.isVisible = True
            return
        hand.isVisible = (value > 0)
        cutter.isVisible = (value > 0)


class BevelGearGenerator:
    """Standalone generator: no base.Generator, no GenerationContext, no user parameters
    ([PB-PRECOMPUTED-MODE]). Every value is precomputed in Python, in internal cm, and written
    into geometry numerically."""

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designOccurrence: adsk.fusion.Occurrence
        self.designComponent: adsk.fusion.Component
        self.bevelComponent: adsk.fusion.Component

    # --- S6: orchestration ------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        futil.log('BevelGearGenerator.generate: reading inputs')
        readResult = self._readInputs(inputs)
        parentComponent: adsk.fusion.Component = readResult[0]
        targetPlane: adsk.fusion.ConstructionPlane = readResult[1]
        centerPoint = readResult[2]
        module = readResult[3]
        drivingTeeth = readResult[4]
        pinionTeeth = readResult[5]
        shaftAngle_deg = readResult[6]

        # Resolve pitch diameters in Python, in cm ([PB-PRECOMPUTED-MODE]).
        drivingPitchDiameter_cm = to_cm(module) * drivingTeeth
        pinionPitchDiameter_cm = to_cm(module) * pinionTeeth

        # S7: occurrence tree.
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

        futil.log('BevelGearGenerator.generate: building anchor sketch')
        self._buildAnchorSketch(self.designComponent, targetPlane, centerPoint)

        futil.log('BevelGearGenerator.generate: building the Gear Profiles lattice')
        pinionCtx, drivingCtx = self._buildGearProfiles(
            self.designComponent, targetPlane, module, drivingTeeth, pinionTeeth,
            shaftAngle_deg, drivingPitchDiameter_cm, pinionPitchDiameter_cm)

        for ctx in (pinionCtx, drivingCtx):
            futil.log(f'BevelGearGenerator.generate: building {ctx["label"]} tooth profile',
                      force_console=True)
            self._buildVirtualSpurProfile(module, ctx)
            futil.log(f'BevelGearGenerator.generate: building {ctx["label"]} gear body',
                      force_console=True)
            self._createGearBody(module, ctx)

        self._hideConstructionGeometry()

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()

    # --- S4, S5: read every input, in internal units, and range-check it --------------------

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        design: adsk.fusion.Design = get_design()
        um = design.unitsManager

        parentEntity = get_selection(inputs, INPUT_ID_PARENT)[0]
        parentComponent: adsk.fusion.Component = (
            parentEntity.component
            if parentEntity.objectType == adsk.fusion.Occurrence.classType()
            else parentEntity)
        targetPlane: adsk.fusion.ConstructionPlane = get_selection(inputs, INPUT_ID_PLANE)[0]
        centerPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]

        module = um.evaluateExpression(inputs.itemById(INPUT_ID_MODULE).expression, '')
        if module <= 0:
            raise Exception(f'Module must be greater than 0 (got {module})')

        drivingTeeth = int(round(um.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_TEETH).expression, '')))
        pinionTeeth = int(round(um.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_TEETH).expression, '')))
        if drivingTeeth < 3:
            raise Exception(f'Driving Gear Teeth must be at least 3 (got {drivingTeeth})')
        if pinionTeeth < 3:
            raise Exception(f'Pinion Gear Teeth must be at least 3 (got {pinionTeeth})')

        shaftAngle_rad = um.evaluateExpression(
            inputs.itemById(INPUT_ID_SHAFT_ANGLE).expression, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30:
            raise Exception(f'Shaft Angle must be at least 30 degrees (got {shaftAngle_deg})')

        drivingBaseHeight_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_BASE_HEIGHT).expression, 'mm')
        pinionBaseHeight_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_BASE_HEIGHT).expression, 'mm')
        if drivingBaseHeight_cm < 0:
            raise Exception(
                f'Driving Gear Base Height must be non-negative (got {drivingBaseHeight_cm * 10} mm)')
        if pinionBaseHeight_cm < 0:
            raise Exception(
                f'Pinion Gear Base Height must be non-negative (got {pinionBaseHeight_cm * 10} mm)')

        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_DRIVING_BORE).expression, 'mm')
        pinionBore_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_PINION_BORE).expression, 'mm')
        if drivingBore_cm < 0:
            raise Exception(
                f'Driving Gear Bore Diameter must be non-negative (got {drivingBore_cm * 10} mm)')
        if pinionBore_cm < 0:
            raise Exception(
                f'Pinion Gear Bore Diameter must be non-negative (got {pinionBore_cm * 10} mm)')

        faceWidth_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_FACE_WIDTH).expression, 'mm')
        if faceWidth_cm < 0:
            raise Exception(f'Face Width must be non-negative (got {faceWidth_cm * 10} mm)')

        toothSpacing_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_TOOTH_SPACING).expression, 'mm')
        if toothSpacing_cm < 0:
            raise Exception(f'Tooth Spacing must be non-negative (got {toothSpacing_cm * 10} mm)')

        spiralAngle_rad = um.evaluateExpression(
            inputs.itemById(INPUT_ID_SPIRAL_ANGLE).expression, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if not (0 <= spiralAngle_deg < 60):
            raise Exception(
                f'Mean Spiral Angle must be in [0, 60) degrees (got {spiralAngle_deg})')

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        hand = handItem.name if handItem is not None else _HAND_RIGHT

        cutterRadius_cm = um.evaluateExpression(
            inputs.itemById(INPUT_ID_CUTTER_RADIUS).expression, 'mm')
        if cutterRadius_cm < 0:
            raise Exception(f'Cutter Radius must be non-negative (got {cutterRadius_cm * 10} mm)')

        # --- S5: derived values and computed bounds, in this order, in cm ---
        # Every length derived from Module is to_cm-converted before it touches geometry
        # ([PB-EVAL-EXPRESSION] / S4 "the one trap").
        moduleCm = to_cm(module)
        DPD_cm = moduleCm * drivingTeeth
        PPD_cm = moduleCm * pinionTeeth

        maxOf = max(DPD_cm, PPD_cm)
        minOf = min(DPD_cm, PPD_cm)
        acosLimit_deg = math.degrees(math.acos(-minOf / maxOf))
        if acosLimit_deg <= 150.0:
            if shaftAngle_deg >= acosLimit_deg:
                raise Exception(
                    f'Shaft Angle must be less than {acosLimit_deg} degrees '
                    f'(got {shaftAngle_deg})')
        else:
            if shaftAngle_deg > 150.0:
                raise Exception(
                    f'Shaft Angle must be at most 150 degrees (got {shaftAngle_deg})')

        sigma = math.radians(shaftAngle_deg)
        tanGammaP = (math.sin(sigma) * PPD_cm) / (DPD_cm + PPD_cm * math.cos(sigma))
        gamma_p = math.atan(tanGammaP)
        gamma_g = sigma - gamma_p

        pinionFloor = 5.27 * math.cos(gamma_p)
        if pinionTeeth < pinionFloor:
            raise Exception(
                f'Pinion Gear Teeth must be at least {pinionFloor} (got {pinionTeeth})')
        drivingFloor = 5.27 * math.cos(gamma_g)
        if drivingTeeth < drivingFloor:
            raise Exception(
                f'Driving Gear Teeth must be at least {drivingFloor} (got {drivingTeeth})')

        def minBaseHeight(gamma):
            return 1.05 * 1.25 * moduleCm * math.sin(gamma)

        def maxBaseHeight(r_cm, gamma):
            return 0.95 * (r_cm - 1.25 * moduleCm * math.cos(gamma)) * math.tan(gamma)

        drivingMin = minBaseHeight(gamma_g)
        drivingMax = maxBaseHeight(DPD_cm / 2.0, gamma_g)
        if drivingBaseHeight_cm == 0:
            resolvedDrivingBaseHeight_cm = moduleCm * drivingTeeth / 8.0
            if resolvedDrivingBaseHeight_cm < drivingMin:
                resolvedDrivingBaseHeight_cm = drivingMin
            elif resolvedDrivingBaseHeight_cm > drivingMax:
                resolvedDrivingBaseHeight_cm = drivingMax
        else:
            if drivingBaseHeight_cm < drivingMin or drivingBaseHeight_cm > drivingMax:
                raise Exception(
                    f'Driving Gear Base Height must be between {drivingMin * 10} mm and '
                    f'{drivingMax * 10} mm (got {drivingBaseHeight_cm * 10} mm)')
            resolvedDrivingBaseHeight_cm = drivingBaseHeight_cm

        pinionMin = minBaseHeight(gamma_p)
        pinionMax = maxBaseHeight(PPD_cm / 2.0, gamma_p)
        if pinionBaseHeight_cm == 0:
            resolvedPinionBaseHeight_cm = (
                resolvedDrivingBaseHeight_cm * pinionTeeth / drivingTeeth)
            if resolvedPinionBaseHeight_cm < pinionMin:
                resolvedPinionBaseHeight_cm = pinionMin
            elif resolvedPinionBaseHeight_cm > pinionMax:
                resolvedPinionBaseHeight_cm = pinionMax
        else:
            if pinionBaseHeight_cm < pinionMin or pinionBaseHeight_cm > pinionMax:
                raise Exception(
                    f'Pinion Gear Base Height must be between {pinionMin * 10} mm and '
                    f'{pinionMax * 10} mm (got {pinionBaseHeight_cm * 10} mm)')
            resolvedPinionBaseHeight_cm = pinionBaseHeight_cm

        if boreEnable:
            resolvedDrivingBore_cm = drivingBore_cm if drivingBore_cm != 0 else DPD_cm / 4.0
            resolvedPinionBore_cm = pinionBore_cm if pinionBore_cm != 0 else PPD_cm / 4.0
        else:
            resolvedDrivingBore_cm = 0.0
            resolvedPinionBore_cm = 0.0

        self._drivingBaseHeight_cm = resolvedDrivingBaseHeight_cm
        self._pinionBaseHeight_cm = resolvedPinionBaseHeight_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = resolvedDrivingBore_cm
        self._pinionBore_cm = resolvedPinionBore_cm
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # --- S8: Anchor sketch -------------------------------------------------------------------

    def _buildAnchorSketch(self, designComponent: adsk.fusion.Component,
                            targetPlane: adsk.fusion.ConstructionPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        # [PB-USE-SELECTED-PLANE]: sketch directly on the user-selected plane, never re-derived.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        centerLocal = projectedCenter.geometry
        p0 = adsk.core.Point3D.create(centerLocal.x - 0.5, centerLocal.y, 0)
        p1 = adsk.core.Point3D.create(centerLocal.x + 0.5, centerLocal.y, 0)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)
        textPoint = adsk.core.Point3D.create(centerLocal.x, centerLocal.y + 0.3, 0)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)
        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorCenterPoint = projectedCenter
        self._anchorLine = anchorLine
        return anchorLine

    # --- S9, S10: Gear Profiles plane and the §2 lattice --------------------------------------

    def _buildGearProfiles(self, designComponent: adsk.fusion.Component,
                            targetPlane: adsk.fusion.ConstructionPlane, module, drivingTeeth,
                            pinionTeeth, shaftAngle_deg, DPD_cm, PPD_cm):
        # S9: Gear Profiles Plane, through the Anchor Line at 90 deg to the target plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            self._anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = gearProfilesPlane

        # S10: the lattice.
        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch

        lines = sketch.sketchCurves.sketchLines
        dims = sketch.sketchDimensions
        gc = sketch.geometricConstraints
        # Nested closures below are their own scope for the api-call checker's simple type
        # tracker, so `lines`/`gc`/`dims` (outer-function locals) don't resolve inside them —
        # self.-prefixed mirrors do, since field types are tracked class-wide.
        self._latticeLines: adsk.fusion.SketchLines = lines
        self._latticeDims: adsk.fusion.SketchDimensions = dims
        self._latticeGC: adsk.fusion.GeometricConstraints = gc

        moduleCm = to_cm(module)
        sigma = math.radians(shaftAngle_deg)
        coneDistance_cm = math.sqrt(DPD_cm ** 2 + PPD_cm ** 2)
        tanGammaP = (math.sin(sigma) * PPD_cm) / (DPD_cm + PPD_cm * math.cos(sigma))
        gamma_p = math.atan(tanGammaP)
        gamma_g = sigma - gamma_p
        R_cm = (PPD_cm / 2.0) / math.sin(gamma_p)

        def newLine(p0, p1):
            line = self._latticeLines.addByTwoPoints(_point2(p0), _point2(p1))
            line.isConstruction = True
            return line

        def pin(sketchPoint, existing):
            self._latticeGC.addCoincident(sketchPoint, existing)

        def lenDim(sp0, sp1, length_cm):
            mid = _midpoint2((sp0.geometry.x, sp0.geometry.y), (sp1.geometry.x, sp1.geometry.y))
            d = self._latticeDims.addDistanceDimension(
                sp0, sp1, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(mid[0], mid[1], 0))
            d.parameter.value = length_cm
            return d

        # --- project the anchor centre and the anchor line ---
        centreProjectedColl = sketch.project(self._anchorCenterPoint)
        centreProjected = centreProjectedColl.item(0)

        lineProjectedColl = sketch.project(self._anchorLine)
        projectedAnchorLine = lineProjectedColl.item(0)

        startLocal = projectedAnchorLine.geometry.startPoint
        endLocal = projectedAnchorLine.geometry.endPoint
        ddx, ddy = endLocal.x - startLocal.x, endLocal.y - startLocal.y
        dlen = math.hypot(ddx, ddy)
        dx, dy = ddx / dlen, ddy / dlen
        perpx, perpy = -dy, dx

        # [BEVEL-F-GROW-SIDE]: the one permitted world use — pick the growth side by comparing
        # against the target plane's normal, a one-bit comparison, never a position.
        normalWorld = targetPlane.geometry.normal
        xDirWorld = sketch.xDirection
        yDirWorld = sketch.yDirection
        perpWorld = adsk.core.Vector3D.create(
            xDirWorld.x * perpx + yDirWorld.x * perpy,
            xDirWorld.y * perpx + yDirWorld.y * perpy,
            xDirWorld.z * perpx + yDirWorld.z * perpy)
        if perpWorld.dotProduct(normalWorld) < 0:
            perpx, perpy = -perpx, -perpy

        cLocal = centreProjected.geometry
        cSeedXY = (cLocal.x, cLocal.y)

        # --- centre -> Apex ---
        apexDist = R_cm * math.cos(gamma_g) + self._drivingBaseHeight_cm
        apexSeed = (cSeedXY[0] + perpx * apexDist, cSeedXY[1] + perpy * apexDist)
        centerToApex = newLine(cSeedXY, apexSeed)
        gc.addPerpendicular(centerToApex, projectedAnchorLine)
        pin(centerToApex.startSketchPoint, centreProjected)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint: adsk.fusion.SketchPoint = apexPoint
        self._apex2d = apexSeed

        # --- Driving Gear Shaft Axis, Apex -> B ---
        drivingDir = (-perpx, -perpy)
        bDist = R_cm * math.cos(gamma_g)
        bSeed = (apexSeed[0] + drivingDir[0] * bDist, apexSeed[1] + drivingDir[1] * bDist)
        drivingShaftAxis = newLine(apexSeed, bSeed)
        gc.addParallel(drivingShaftAxis, centerToApex)
        pin(drivingShaftAxis.startSketchPoint, apexPoint)
        pointB = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis, Apex -> A ---
        aDist = R_cm * math.cos(gamma_p)

        def rotate2(vx, vy, ang):
            c_, s_ = math.cos(ang), math.sin(ang)
            return (vx * c_ - vy * s_, vx * s_ + vy * c_)

        candPlus = rotate2(drivingDir[0], drivingDir[1], sigma)
        candMinus = rotate2(drivingDir[0], drivingDir[1], -sigma)
        candPlusEnd = (apexSeed[0] + candPlus[0] * aDist, apexSeed[1] + candPlus[1] * aDist)
        candMinusEnd = (apexSeed[0] + candMinus[0] * aDist, apexSeed[1] + candMinus[1] * aDist)
        if candPlusEnd[0] > candMinusEnd[0]:
            pinionDir, aSeed = candPlus, candPlusEnd
        else:
            pinionDir, aSeed = candMinus, candMinusEnd

        pinionShaftAxis = newLine(apexSeed, aSeed)
        pin(pinionShaftAxis.startSketchPoint, apexPoint)
        pointA = pinionShaftAxis.endSketchPoint

        # --- the Shaft Angle dimension ---
        bisectorRaw = (pinionDir[0] + drivingDir[0], pinionDir[1] + drivingDir[1])
        bisLen = math.hypot(*bisectorRaw)
        bisector = (bisectorRaw[0] / bisLen, bisectorRaw[1] / bisLen)
        angleTextPoint = adsk.core.Point3D.create(
            apexSeed[0] + bisector[0] * (PPD_cm / 4.0),
            apexSeed[1] + bisector[1] * (PPD_cm / 4.0), 0)
        angDim = dims.addAngularDimension(pinionShaftAxis, drivingShaftAxis, angleTextPoint)
        angDim.parameter.value = sigma

        # --- A -> Apex 2, the PPD/2 drop ---
        abDir = (bSeed[0] - aSeed[0], bSeed[1] - aSeed[1])
        abLen = math.hypot(*abDir)
        abUnit = (abDir[0] / abLen, abDir[1] / abLen)
        perpToPinion = (-pinionDir[1], pinionDir[0])
        if perpToPinion[0] * abUnit[0] + perpToPinion[1] * abUnit[1] < 0:
            perpToPinion = (-perpToPinion[0], -perpToPinion[1])
        aDropDist = PPD_cm / 2.0
        aDropSeed = (aSeed[0] + perpToPinion[0] * aDropDist, aSeed[1] + perpToPinion[1] * aDropDist)
        aDrop = newLine(aSeed, aDropSeed)
        gc.addPerpendicular(aDrop, pinionShaftAxis)
        lenDim(aDrop.startSketchPoint, aDrop.endSketchPoint, aDropDist)
        pin(aDrop.startSketchPoint, pointA)

        # --- B -> Apex 2, the DPD/2 drop ---
        baDir = (aSeed[0] - bSeed[0], aSeed[1] - bSeed[1])
        baLen = math.hypot(*baDir)
        baUnit = (baDir[0] / baLen, baDir[1] / baLen)
        perpToDriving = (-drivingDir[1], drivingDir[0])
        if perpToDriving[0] * baUnit[0] + perpToDriving[1] * baUnit[1] < 0:
            perpToDriving = (-perpToDriving[0], -perpToDriving[1])
        bDropDist = DPD_cm / 2.0
        bDropSeed = (bSeed[0] + perpToDriving[0] * bDropDist, bSeed[1] + perpToDriving[1] * bDropDist)
        bDrop = newLine(bSeed, bDropSeed)
        gc.addPerpendicular(bDrop, drivingShaftAxis)
        lenDim(bDrop.startSketchPoint, bDrop.endSketchPoint, bDropDist)
        pin(bDrop.startSketchPoint, pointB)

        # --- close them: Apex 2 ---
        gc.addCoincident(aDrop.endSketchPoint, bDrop.endSketchPoint)
        apex2Point = aDrop.endSketchPoint

        # --- Pitch Line ---
        pitchLine = newLine(apexSeed, aDropSeed)
        pin(pitchLine.startSketchPoint, apexPoint)
        pin(pitchLine.endSketchPoint, apex2Point)

        # --- the two dedendum lines ---
        pitchDir = (aDropSeed[0] - apexSeed[0], aDropSeed[1] - apexSeed[1])
        pitchLen = math.hypot(*pitchDir)
        pitchUnit = (pitchDir[0] / pitchLen, pitchDir[1] / pitchLen)
        perpToPitch = (-pitchUnit[1], pitchUnit[0])
        dedLen = moduleCm * 1.25
        cand1 = (perpToPitch[0] * dedLen, perpToPitch[1] * dedLen)
        cand2 = (-cand1[0], -cand1[1])
        dot1 = cand1[0] * perpx + cand1[1] * perpy
        dot2 = cand2[0] * perpx + cand2[1] * perpy
        if dot1 < dot2:
            towardAnchorDir, awayDir = cand1, cand2
        else:
            towardAnchorDir, awayDir = cand2, cand1

        dSeed = (aDropSeed[0] + towardAnchorDir[0], aDropSeed[1] + towardAnchorDir[1])
        cSeed = (aDropSeed[0] + awayDir[0], aDropSeed[1] + awayDir[1])
        dedUnitPinion = (awayDir[0] / dedLen, awayDir[1] / dedLen)
        dedUnitDriving = (towardAnchorDir[0] / dedLen, towardAnchorDir[1] / dedLen)

        drivingDedendum = newLine(aDropSeed, dSeed)
        gc.addPerpendicular(drivingDedendum, pitchLine)
        lenDim(drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint, dedLen)
        pin(drivingDedendum.startSketchPoint, apex2Point)
        pointD = drivingDedendum.endSketchPoint

        pinionDedendum = newLine(aDropSeed, cSeed)
        gc.addPerpendicular(pinionDedendum, pitchLine)
        lenDim(pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint, dedLen)
        pin(pinionDedendum.startSketchPoint, apex2Point)
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axes ---
        pinionRootAxis = newLine(apexSeed, cSeed)
        pin(pinionRootAxis.startSketchPoint, apexPoint)
        pin(pinionRootAxis.endSketchPoint, pointC)

        drivingRootAxis = newLine(apexSeed, dSeed)
        pin(drivingRootAxis.startSketchPoint, apexPoint)
        pin(drivingRootAxis.endSketchPoint, pointD)

        # --- A -> E ---
        eSeed = (aSeed[0] + pinionDir[0] * moduleCm, aSeed[1] + pinionDir[1] * moduleCm)
        aToE = newLine(aSeed, eSeed)
        gc.addCollinear(aToE, pinionShaftAxis)
        pin(aToE.startSketchPoint, pointA)
        pointE = aToE.endSketchPoint

        # --- C -> E ---
        cToE = newLine(cSeed, eSeed)
        pin(cToE.startSketchPoint, pointC)
        pin(cToE.endSketchPoint, pointE)
        gc.addPerpendicular(aToE, cToE)

        # --- B -> F ---
        fSeed = (bSeed[0] + drivingDir[0] * moduleCm, bSeed[1] + drivingDir[1] * moduleCm)
        bToF = newLine(bSeed, fSeed)
        gc.addCollinear(bToF, drivingShaftAxis)
        pin(bToF.startSketchPoint, pointB)
        pointF = bToF.endSketchPoint

        # --- D -> F ---
        dToF = newLine(dSeed, fSeed)
        pin(dToF.startSketchPoint, pointD)
        pin(dToF.endSketchPoint, pointF)
        gc.addPerpendicular(bToF, dToF)

        # --- E -> G (collinear with A->E, never Apex->A, [BEVEL-F-COLLINEAR-CHAIN]) ---
        gSeed = (eSeed[0] + pinionDir[0] * moduleCm, eSeed[1] + pinionDir[1] * moduleCm)
        eToG = newLine(eSeed, gSeed)
        gc.addCollinear(eToG, aToE)
        pin(eToG.startSketchPoint, pointE)
        pointG = eToG.endSketchPoint

        # --- C -> H (collinear with the Pinion Dedendum line Apex2->C) ---
        hSeed = (cSeed[0] + dedUnitPinion[0] * moduleCm, cSeed[1] + dedUnitPinion[1] * moduleCm)
        cToH = newLine(cSeed, hSeed)
        gc.addCollinear(cToH, pinionDedendum)
        pin(cToH.startSketchPoint, pointC)
        pointH = cToH.endSketchPoint

        # --- G -> H, with the required (proof-omitted) perpendicular ---
        gToH = newLine(gSeed, hSeed)
        pin(gToH.startSketchPoint, pointG)
        pin(gToH.endSketchPoint, pointH)
        gc.addPerpendicular(eToG, gToH)

        # --- F -> I (driving twin of E->G) ---
        iSeed = (fSeed[0] + drivingDir[0] * moduleCm, fSeed[1] + drivingDir[1] * moduleCm)
        fToI = newLine(fSeed, iSeed)
        gc.addCollinear(fToI, bToF)
        pin(fToI.startSketchPoint, pointF)
        pointI = fToI.endSketchPoint

        # --- D -> J (driving twin of C->H, collinear with the Driving Dedendum line Apex2->D) ---
        jSeed = (dSeed[0] + dedUnitDriving[0] * moduleCm, dSeed[1] + dedUnitDriving[1] * moduleCm)
        dToJ = newLine(dSeed, jSeed)
        gc.addCollinear(dToJ, drivingDedendum)
        pin(dToJ.startSketchPoint, pointD)
        pointJ = dToJ.endSketchPoint

        # --- I -> J (driving twin of G->H) ---
        iToJ = newLine(iSeed, jSeed)
        pin(iToJ.startSketchPoint, pointI)
        pin(iToJ.endSketchPoint, pointJ)
        gc.addPerpendicular(fToI, iToJ)

        # --- the driving base-height offset ---
        drivingOffsetText = adsk.core.Point3D.create(
            *_midpoint2(bDropSeed, jSeed), 0)
        drivingOffsetDim = dims.addOffsetDimension(bDrop, iToJ, drivingOffsetText)
        drivingOffsetDim.parameter.value = self._drivingBaseHeight_cm

        # --- the pinion base-height offset ---
        pinionOffsetText = adsk.core.Point3D.create(
            *_midpoint2(aDropSeed, hSeed), 0)
        pinionOffsetDim = dims.addOffsetDimension(aDrop, gToH, pinionOffsetText)
        pinionOffsetDim.parameter.value = self._pinionBaseHeight_cm

        # --- A -> G ---
        aToG = newLine(aSeed, gSeed)
        pin(aToG.startSketchPoint, pointA)
        pin(aToG.endSketchPoint, pointG)

        # --- constrain point I with the projected centre point: closes the whole figure ---
        gc.addCoincident(pointI, centreProjected)

        # --- K, the back-cone point (pinion) ---
        backConeDistPinion = (PPD_cm / 2.0) / math.cos(gamma_p)
        kSeed = (aDropSeed[0] + dedUnitPinion[0] * backConeDistPinion,
                  aDropSeed[1] + dedUnitPinion[1] * backConeDistPinion)
        kLine = newLine(gSeed, kSeed)
        pin(kLine.startSketchPoint, pointG)
        gc.addCoincident(kLine.endSketchPoint, pinionShaftAxis)
        gc.addCoincident(kLine.endSketchPoint, pinionDedendum)
        pointK = kLine.endSketchPoint

        cToKRef = newLine(cSeed, kSeed)
        pin(cToKRef.startSketchPoint, pointC)
        pin(cToKRef.endSketchPoint, pointK)

        # --- K', the tooth centre (pinion) ---
        if self._toothSpacing_cm == 0:
            pointKPrime = pointK
            pinionToothCenterRef = cToKRef
        else:
            kPrimeSeed = (kSeed[0] + dedUnitPinion[0] * self._toothSpacing_cm,
                          kSeed[1] + dedUnitPinion[1] * self._toothSpacing_cm)
            kPrimeLine = newLine(kSeed, kPrimeSeed)
            pin(kPrimeLine.startSketchPoint, pointK)
            gc.addCoincident(kPrimeLine.endSketchPoint, pinionDedendum)
            lenDim(kPrimeLine.startSketchPoint, kPrimeLine.endSketchPoint, self._toothSpacing_cm)
            pointKPrime = kPrimeLine.endSketchPoint

            pinionToothCenterRef = newLine(cSeed, kPrimeSeed)
            pin(pinionToothCenterRef.startSketchPoint, pointC)
            pin(pinionToothCenterRef.endSketchPoint, pointKPrime)

        # --- L, the back-cone point (driving twin of K) ---
        backConeDistDriving = (DPD_cm / 2.0) / math.cos(gamma_g)
        lSeed = (aDropSeed[0] + dedUnitDriving[0] * backConeDistDriving,
                  aDropSeed[1] + dedUnitDriving[1] * backConeDistDriving)
        lLine = newLine(iSeed, lSeed)
        pin(lLine.startSketchPoint, pointI)
        gc.addCoincident(lLine.endSketchPoint, drivingShaftAxis)
        gc.addCoincident(lLine.endSketchPoint, drivingDedendum)
        pointL = lLine.endSketchPoint

        dToLRef = newLine(dSeed, lSeed)
        pin(dToLRef.startSketchPoint, pointD)
        pin(dToLRef.endSketchPoint, pointL)

        # --- L', the tooth centre (driving twin of K') ---
        if self._toothSpacing_cm == 0:
            pointLPrime = pointL
            drivingToothCenterRef = dToLRef
        else:
            lPrimeSeed = (lSeed[0] + dedUnitDriving[0] * self._toothSpacing_cm,
                          lSeed[1] + dedUnitDriving[1] * self._toothSpacing_cm)
            lPrimeLine = newLine(lSeed, lPrimeSeed)
            pin(lPrimeLine.startSketchPoint, pointL)
            gc.addCoincident(lPrimeLine.endSketchPoint, drivingDedendum)
            lenDim(lPrimeLine.startSketchPoint, lPrimeLine.endSketchPoint, self._toothSpacing_cm)
            pointLPrime = lPrimeLine.endSketchPoint

            drivingToothCenterRef = newLine(dSeed, lPrimeSeed)
            pin(drivingToothCenterRef.startSketchPoint, pointD)
            pin(drivingToothCenterRef.endSketchPoint, pointLPrime)

        # --- Resolve the Maximum Face Width, from SOLVED geometry ([PB-SOLVED-GEOMETRY]) ---
        aXY = (pointA.geometry.x, pointA.geometry.y)
        bXY = (pointB.geometry.x, pointB.geometry.y)
        cXY = (pointC.geometry.x, pointC.geometry.y)
        dXY = (pointD.geometry.x, pointD.geometry.y)
        hXY = (pointH.geometry.x, pointH.geometry.y)
        jXY = (pointJ.geometry.x, pointJ.geometry.y)
        distA = _perpDistance2(aXY, cXY, hXY)
        distB = _perpDistance2(bXY, dXY, jXY)
        maxFaceWidth_cm = 0.95 * min(distA, distB)
        autoFaceWidth_cm = min(coneDistance_cm / 6.0, maxFaceWidth_cm)
        if self._faceWidth_cm == 0:
            resolvedFaceWidth_cm = autoFaceWidth_cm
        else:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    f'Face Width must not exceed {maxFaceWidth_cm * 10} mm '
                    f'(got {self._faceWidth_cm * 10} mm)')
            resolvedFaceWidth_cm = self._faceWidth_cm
        self._faceWidthResolved_cm = resolvedFaceWidth_cm

        # --- M -> N, the pinion toe line ---
        fraction = 1.0 - resolvedFaceWidth_cm / R_cm
        mSeed = (apexSeed[0] + (cSeed[0] - apexSeed[0]) * fraction,
                  apexSeed[1] + (cSeed[1] - apexSeed[1]) * fraction)
        chDir = (hSeed[0] - cSeed[0], hSeed[1] - cSeed[1])
        chLen = math.hypot(*chDir)
        chUnit = (chDir[0] / chLen, chDir[1] / chLen)
        nSeed = _lineIntersect2(mSeed, chUnit, aSeed, aDropSeed)

        mLine = newLine(mSeed, nSeed)
        gc.addCoincident(mLine.startSketchPoint, pinionRootAxis)
        gc.addCoincident(mLine.endSketchPoint, aDrop)
        gc.addParallel(mLine, cToH)
        toeOffsetTextPinion = adsk.core.Point3D.create(*_midpoint2(mSeed, cSeed), 0)
        toeOffsetDimPinion = dims.addOffsetDimension(cToH, mLine, toeOffsetTextPinion)
        toeOffsetDimPinion.parameter.value = resolvedFaceWidth_cm
        pointM = mLine.startSketchPoint
        pointN = mLine.endSketchPoint

        mToC = newLine(mSeed, cSeed)
        pin(mToC.startSketchPoint, pointM)
        pin(mToC.endSketchPoint, pointC)
        nToA = newLine(nSeed, aSeed)
        pin(nToA.startSketchPoint, pointN)
        pin(nToA.endSketchPoint, pointA)

        # --- O -> P, the driving toe line (twin of M->N) ---
        oSeed = (apexSeed[0] + (dSeed[0] - apexSeed[0]) * fraction,
                  apexSeed[1] + (dSeed[1] - apexSeed[1]) * fraction)
        djDir = (jSeed[0] - dSeed[0], jSeed[1] - dSeed[1])
        djLen = math.hypot(*djDir)
        djUnit = (djDir[0] / djLen, djDir[1] / djLen)
        pSeed = _lineIntersect2(oSeed, djUnit, bSeed, bDropSeed)

        oLine = newLine(oSeed, pSeed)
        gc.addCoincident(oLine.startSketchPoint, drivingRootAxis)
        gc.addCoincident(oLine.endSketchPoint, bDrop)
        gc.addParallel(oLine, dToJ)
        toeOffsetTextDriving = adsk.core.Point3D.create(*_midpoint2(oSeed, dSeed), 0)
        toeOffsetDimDriving = dims.addOffsetDimension(dToJ, oLine, toeOffsetTextDriving)
        toeOffsetDimDriving.parameter.value = resolvedFaceWidth_cm
        pointO = oLine.startSketchPoint
        pointP = oLine.endSketchPoint

        oToD = newLine(oSeed, dSeed)
        pin(oToD.startSketchPoint, pointO)
        pin(oToD.endSketchPoint, pointD)
        pToB = newLine(pSeed, bSeed)
        pin(pToB.startSketchPoint, pointP)
        pin(pToB.endSketchPoint, pointB)
        bToI = newLine(bSeed, iSeed)
        pin(bToI.startSketchPoint, pointB)
        pin(bToI.endSketchPoint, pointI)

        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        self._coneDistance_cm = coneDistance_cm
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        pinionCtx = {
            'label': 'Pinion',
            'teeth': pinionTeeth,
            'pitchDiameter_cm': PPD_cm,
            'gamma': gamma_p,
            'toothCenterPoint': pointKPrime,
            'toothCenterRefLine': pinionToothCenterRef,
            'hexagonVertices': [pointA, pointG, pointH, pointC, pointM, pointN],
            'toeEdgePoints': (pointM, pointN),
            'heelEdgePoints': (pointC, pointH),
            'toeConePoint': pointM,
            'heelConePoint': pointC,
            'rootAxisLine': pinionRootAxis,
            'boreDiameter_cm': self._pinionBore_cm,
            'meshAngle': self._pinionMeshPhase(pinionTeeth),
        }
        drivingCtx = {
            'label': 'Driving',
            'teeth': drivingTeeth,
            'pitchDiameter_cm': DPD_cm,
            'gamma': gamma_g,
            'toothCenterPoint': pointLPrime,
            'toothCenterRefLine': drivingToothCenterRef,
            'hexagonVertices': [pointB, pointI, pointJ, pointD, pointO, pointP],
            'toeEdgePoints': (pointO, pointP),
            'heelEdgePoints': (pointD, pointJ),
            'toeConePoint': pointO,
            'heelConePoint': pointD,
            'rootAxisLine': drivingRootAxis,
            'boreDiameter_cm': self._drivingBore_cm,
            'meshAngle': math.pi / drivingTeeth,
        }
        return pinionCtx, drivingCtx

    # --- S11, S12, S13: the virtual spur tooth profile, per gear -----------------------------

    def _buildVirtualSpurProfile(self, module, ctx):
        gearLabel = ctx['label']
        pitchDiaCm = ctx['pitchDiameter_cm']
        gamma = ctx['gamma']

        virtualPitchRadius_mm = (pitchDiaCm * 10.0 / 2.0) / math.cos(gamma)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / module))

        toothPlane = plane_by_angle(
            self.designComponent, ctx['toothCenterRefLine'], self._gearProfilesPlane, 90)
        toothPlane.name = f'{gearLabel} Plane'

        toothSketch = self.designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'

        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(ctx['toothCenterPoint'], angle=math.radians(180))

        embedded = proxy._lastToothEmbedded

        # [PB-TEXT-HOLDS-DOF]: the sketch text this drawer adds carries its own DOF. Log, never raise.
        if not toothSketch.isFullyConstrained:
            futil.log(f'{gearLabel} Tooth sketch not fully constrained '
                      f'(expected: the drawer\'s circle labels hold DOF)')

        # S13: Tooth Axis.
        helperInput = self.designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            ctx['toothCenterRefLine'], adsk.core.ValueInput.createByReal(1.0))
        helperPlane = self.designComponent.constructionPlanes.add(helperInput)

        axisInput = self.designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        axis = self.designComponent.constructionAxes.add(axisInput)
        axis.name = f'{gearLabel} Tooth Axis'

        ctx['toothPlane'] = toothPlane
        ctx['toothSketch'] = toothSketch
        ctx['embedded'] = embedded
        ctx['virtualTeeth'] = virtualTeeth

    # --- S14 .. S33: the finished gear body, per gear -----------------------------------------

    def _createGearBody(self, module, ctx):
        gearLabel = ctx['label']
        dc = self.designComponent

        # S14: {gearLabel} Gear component, a child of Bevel Gear.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{gearLabel} Gear'

        # S15: {gearLabel} Profile sketch — the frustum hexagon, recreate-share-fix.
        profileSketch = dc.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'

        verts = [profileSketch.sketchPoints.add(profileSketch.modelToSketchSpace(v.worldGeometry))
                 for v in ctx['hexagonVertices']]
        profileLines = profileSketch.sketchCurves.sketchLines
        hexLines = []
        for i in range(6):
            p0, p1 = verts[i], verts[(i + 1) % 6]
            hexLines.append(profileLines.addByTwoPoints(p0, p1))
        for e in hexLines:
            e.startSketchPoint.isFixed = True
            e.endSketchPoint.isFixed = True

        if not profileSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Profile sketch is not fully constrained')

        shaftAxisEdge = hexLines[0]

        # S16: revolve into the Gear Body.
        profile = profileSketch.profiles.item(0)
        revolveInput = dc.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        gearBody = dc.features.revolveFeatures.add(revolveInput).bodies.item(0)

        # S17: loft the Apex point to the tooth profile.
        toothProfile = find_profile_by_curve_counts(
            ctx['toothSketch'], nurbs=2, arcs=2, lines=(0 if ctx['embedded'] else 2))

        loftInput = dc.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        toothBody = dc.features.loftFeatures.add(loftInput).bodies.item(0)

        # S18 / S19..S27: flush-trim or spiral-transform the tooth body.
        apexWorld = self._apexSketchPoint.worldGeometry
        toeA, toeB = ctx['toeEdgePoints']
        heelA, heelB = ctx['heelEdgePoints']
        toeMid = _midpointWorld(toeA.worldGeometry, toeB.worldGeometry)
        heelMid = _midpointWorld(heelA.worldGeometry, heelB.worldGeometry)
        toeConeWorld = ctx['toeConePoint'].worldGeometry
        heelConeWorld = ctx['heelConePoint'].worldGeometry

        toothBody = self._transformToothBody(
            dc, toothBody, gearBody, shaftAxisEdge, apexWorld, self._apexSketchPoint,
            toeMid, heelMid, toeConeWorld, heelConeWorld, ctx['toothPlane'], gearLabel,
            ctx['teeth'], ctx['gamma'])

        # S28: circular pattern.
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(toothBody)
        patternInput = dc.features.circularPatternFeatures.createInput(bodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = dc.features.circularPatternFeatures.add(patternInput)

        # S29: combine-join.
        tools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(i))
        combineInput = dc.features.combineFeatures.createInput(gearBody, tools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        dc.features.combineFeatures.add(combineInput)

        # S30 / S31: bore.
        if self._boreEnable:
            self._cutBore(dc, gearBody, shaftAxisEdge, ctx['boreDiameter_cm'], gearLabel)

        # S32: meshing rotation, still in Design.
        rotate_body_about_edge(dc, gearBody, shaftAxisEdge, ctx['meshAngle'])

        # S33: move the finished body into {gearLabel} Gear.
        gearBody.moveToComponent(gearOccurrence)

    def _pinionMeshPhase(self, pinionTeeth):
        return _PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    def _cutBore(self, designComponent: adsk.fusion.Component, gearBody: adsk.fusion.BRepBody,
                 shaftAxisEdge: adsk.fusion.SketchLine, boreDiameter_cm, gearLabel):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore'

        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2.0)
        circle.centerSketchPoint.isFixed = True
        textPoint = adsk.core.Point3D.create(boreDiameter_cm / 2.0, 0, 0)
        dim = boreSketch.sketchDimensions.addDiameterDimension(circle, textPoint)
        dim.parameter.value = boreDiameter_cm

        if not boreSketch.isFullyConstrained:
            raise Exception(f'{gearLabel} Bore sketch is not fully constrained')

        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            boreSketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistance_cm), False)
        extrudeInput.participantBodies = [gearBody]
        designComponent.features.extrudeFeatures.add(extrudeInput)

    # --- S18 / S19..S27: straight flush trim, or the full spiral transform -------------------

    def _transformToothBody(self, designComponent: adsk.fusion.Component,
                             toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                             shaftAxisEdge: adsk.fusion.SketchLine, apexWorld: adsk.core.Point3D,
                             apexSketchPoint: adsk.fusion.SketchPoint,
                             toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                             toeConeWorld: adsk.core.Point3D, heelConeWorld: adsk.core.Point3D,
                             parentToothPlane: adsk.fusion.ConstructionPlane,
                             gearLabel, teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            return cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        # --- S19 Step A: the frame ---
        startW = shaftAxisEdge.startSketchPoint.worldGeometry
        endW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = startW.vectorTo(endW)
        axisDir.normalize()

        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = apexWorld.vectorTo(heelConeWorld)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()

        # Nested closures below are their own scope for the api-call checker's simple type
        # tracker, so `apexWorld`/`coneVec`/`v`/`axisDir` (outer-function locals) don't resolve
        # inside them — self.-prefixed mirrors do, since field types are tracked class-wide.
        self._spiralApexWorld: adsk.core.Point3D = apexWorld
        self._spiralConeVec: adsk.core.Vector3D = coneVec
        self._spiralV: adsk.core.Vector3D = v
        self._spiralAxisDir: adsk.core.Vector3D = axisDir

        def distAlong(p):
            return self._spiralApexWorld.vectorTo(p).dotProduct(self._spiralConeVec)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- S19 Step B: the cutter-arc geometry ---
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm != 0 else R_mean
        baseSign = 1 if self._hand == _HAND_RIGHT else -1
        handSign = -baseSign if gearLabel == 'Pinion' else baseSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        def tanW(px, py):
            return combine_point(self._spiralApexWorld, px, self._spiralConeVec, py, self._spiralV)

        # --- S20: {gearLabel} Cone Element sketch ---
        coneSketch = designComponent.sketches.add(self._gearProfilesPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneEnd = combine_point(apexWorld, R_heel, coneVec)
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(apexWorld, coneEnd)
        coneElementLine.isConstruction = True

        # --- S21: {gearLabel} Trace Plane ---
        tracePlane = plane_by_angle(designComponent, coneElementLine, self._gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # --- S22: {gear} 2D Tooth Trace sketch — the genuine cutter arc ---
        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        cutterCenterWorld = tanW(Cx, Cy)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            cutterCenterWorld, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        circleTextPoint = tanW(Cx + r_c, Cy)
        circleDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, circleTextPoint)
        circleDim.parameter.value = 2 * r_c

        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            tanW(*toe2d), tanW(R_mean, 0.0), tanW(*heel2d))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        arcTextPoint = tanW(R_mean, 0.0)
        arcDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, arcTextPoint)
        arcDim.parameter.value = r_c

        # --- S23: slice into cross-section slabs ---
        planeGeom = parentToothPlane.geometry
        toApex = planeGeom.origin.vectorTo(apexWorld)
        sign = 1 if toApex.dotProduct(planeGeom.normal) > 0 else -1

        offsets = [sign * (k + 1) * span / 6.0 for k in range(8)]
        pieces = slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
        if len(pieces) == 1:
            sign = -sign
            offsets = [sign * (k + 1) * span / 6.0 for k in range(8)]
            pieces = slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)
            if len(pieces) == 1:
                raise Exception(
                    f'{gearLabel}: slice produced 1 piece after retrying with the opposite sign '
                    f'(span={span}, sign tried={sign}) — cut planes missed the tooth')

        # --- S24: order the slabs and drop the apex scrap ---
        pieces.sort(key=lambda body_: distAlong(body_.physicalProperties.centerOfMass))
        scrap = pieces[0]
        segments = pieces[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(f'{gearLabel}: no segments left after dropping the apex scrap')

        def slabFaces(seg):
            faces = list(seg.faces)
            faces.sort(key=lambda f: distAlong(f.centroid))
            return faces[0], faces[-1]

        def slabToeFace(seg):
            return slabFaces(seg)[0]

        def slabHeelFace(seg):
            return slabFaces(seg)[1]

        # --- S25: twist the slabs about the shaft axis ---
        phi_toe = math.atan2(toe2d[1], toe2d[0])
        phi_heel = math.atan2(heel2d[1], heel2d[0])
        phi_crown = phi_heel - phi_toe
        total = abs(phi_crown) / math.sin(gamma)

        for seg in segments:
            R_heelFace = distAlong(slabHeelFace(seg).centroid)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            if ang == 0:
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            bodyColl = adsk.core.ObjectCollection.create()
            bodyColl.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(bodyColl)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # --- S26: crown the slabs lengthwise ---
        segsByHeel = sorted(segments, key=lambda seg: distAlong(slabHeelFace(seg).centroid))
        toCrown = segsByHeel[:-1]

        def axisDist(p):
            toP = self._spiralApexWorld.vectorTo(p)
            along = toP.dotProduct(self._spiralAxisDir)
            perpVec = adsk.core.Vector3D.create(
                toP.x - along * self._spiralAxisDir.x, toP.y - along * self._spiralAxisDir.y,
                toP.z - along * self._spiralAxisDir.z)
            return perpVec.length

        try:
            self.designOccurrence.activate()
            for seg in toCrown:
                heelFace = slabHeelFace(seg)
                R_heelFace = distAlong(heelFace.centroid)
                u = (R_heel - R_heelFace) / span
                factor = 1.0 - _CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor} <= 0 for segment u={u}')

                verts = sorted((vtx.geometry for vtx in heelFace.vertices), key=axisDist)
                rootA, rootB = verts[0], verts[1]
                rootMidWorld = _midpointWorld(rootA, rootB)

                baseSketch = designComponent.sketches.add(heelFace)
                baseLocal = baseSketch.modelToSketchSpace(rootMidWorld)
                basePoint = baseSketch.sketchPoints.add(baseLocal)

                entities = adsk.core.ObjectCollection.create()
                entities.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    entities, basePoint, adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # --- S27: loft the curved tooth ---
        order = sorted(segments, key=lambda seg: distAlong(slabHeelFace(seg).centroid))
        toeFace = slabToeFace(order[0])

        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toeFace)
        for seg in order:
            loftInput.loftSections.add(slabHeelFace(seg))
        curvedTooth = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in segments:
            designComponent.features.removeFeatures.add(seg)

        return cut_conical_ends(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # --- S34: cleanup -------------------------------------------------------------------------

    def _hideConstructionGeometry(self):
        hide_construction_geometry(self.bevelComponent)
