import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_selection, get_boolean
from .utilities import find_profile_by_curve_counts, get_normal
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator


# -----------------------------------------------------------------------------------------
# Dialog input ids (S1). Bevel registers no Fusion user parameters ([PB-PRECOMPUTED-MODE]),
# so there are no PARAM_* names.
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

_MIN_TEETH_FACTOR = 5.27  # [PB-...] rounded UP from 2*(1.05*1.25/0.95 + 1.25) = 5.2632...


# -----------------------------------------------------------------------------------------
# Small module-level geometry helpers. Kept at module scope (never nested inside a method)
# so every parameter that holds a Fusion object carries its own explicit type annotation,
# rather than relying on an annotation from an enclosing scope reaching into a closure.
# -----------------------------------------------------------------------------------------

def _pt(x: float, y: float, z: float = 0.0) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(x, y, z)


def _v2add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _v2sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _v2mul(a, s: float):
    return (a[0] * s, a[1] * s)


def _v2dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _v2cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _v2norm(a):
    n = math.hypot(a[0], a[1])
    return (a[0] / n, a[1] / n)


def _v2rot(a, angleRad: float):
    c, s = math.cos(angleRad), math.sin(angleRad)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def _perpDistToLine(p, l1, l2) -> float:
    dx, dy = l2[0] - l1[0], l2[1] - l1[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return 0.0
    cross = (p[0] - l1[0]) * dy - (p[1] - l1[1]) * dx
    return abs(cross) / length


def _perpDistFromAxis(p, origin, axisDir) -> float:
    return abs(_v2cross(_v2sub(p, origin), axisDir))


def _rawLine(sketch: adsk.fusion.Sketch, p0, p1) -> adsk.fusion.SketchLine:
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt(*p0), _pt(*p1))
    line.isConstruction = True
    return line


def _pinPoint(sketch: adsk.fusion.Sketch, point: adsk.fusion.SketchPoint, target) -> None:
    sketch.geometricConstraints.addCoincident(point, target)


def _alignedDim(sketch: adsk.fusion.Sketch, p0: adsk.fusion.SketchPoint, p1: adsk.fusion.SketchPoint,
                 valueCm: float, textXY) -> adsk.fusion.SketchLinearDimension:
    dim = sketch.sketchDimensions.addDistanceDimension(
        p0, p1, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, _pt(*textXY))
    dim.parameter.value = valueCm
    return dim


def _offsetDim(sketch: adsk.fusion.Sketch, lineA: adsk.fusion.SketchLine, lineB: adsk.fusion.SketchLine,
               valueCm: float, textXY) -> adsk.fusion.SketchOffsetDimension:
    dim = sketch.sketchDimensions.addOffsetDimension(lineA, lineB, _pt(*textXY))
    dim.parameter.value = valueCm
    return dim


def _angularDim(sketch: adsk.fusion.Sketch, lineA: adsk.fusion.SketchLine, lineB: adsk.fusion.SketchLine,
                 valueRad: float, textXY) -> adsk.fusion.SketchAngularDimension:
    dim = sketch.sketchDimensions.addAngularDimension(lineA, lineB, _pt(*textXY))
    dim.parameter.value = valueRad
    return dim


# ---- World-space (Vector3D/Point3D) helpers for the spiral branch (S14-S20) -------------

def _wsub(p1: adsk.core.Point3D, p2: adsk.core.Point3D) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)


def _wadd(p: adsk.core.Point3D, v: adsk.core.Vector3D, scale: float = 1.0) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(p.x + v.x * scale, p.y + v.y * scale, p.z + v.z * scale)


def _wunit(pFrom: adsk.core.Point3D, pTo: adsk.core.Point3D) -> adsk.core.Vector3D:
    v: adsk.core.Vector3D = _wsub(pTo, pFrom)
    v.normalize()
    return v


def _wmidpoint(p1: adsk.core.Point3D, p2: adsk.core.Point3D) -> adsk.core.Point3D:
    return _pt((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0)


# ===========================================================================================
# S1 -- Dialog inputs
# ===========================================================================================

class BevelGearCommandInputsConfigurator:
    """Plain class -- no base. `commands/bevelgear/entry.py` binds `configure` and
    `handle_input_changed` by name; `_updateSpiralInputVisibility` is the private helper
    `handle_input_changed` delegates to."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command) -> None:
        inputs: adsk.core.CommandInputs = cmd.commandInputs

        # Target Plane first so it wins Fusion's auto-focus ([PB-AUTOFOCUS-FIRST]).
        planeInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        centerInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        parentInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '',
                              adsk.core.ValueInput.createByReal(1))
        inputs.addValueInput(INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
                              adsk.core.ValueInput.createByString('90 deg'))
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
                              adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
                              adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        inputs.addValueInput(INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
                              adsk.core.ValueInput.createByString('35 deg'))

        handInput: adsk.core.DropDownCommandInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension', '',
                              adsk.core.ValueInput.createByReal(0))
        inputs.addValueInput(INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
                              adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs) -> None:
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs) -> None:
        spiralInput: adsk.core.ValueCommandInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput: adsk.core.DropDownCommandInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput: adsk.core.ValueCommandInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            design = get_design()
            unitsManager: adsk.core.UnitsManager = design.unitsManager
            value = unitsManager.evaluateExpression(spiralInput.expression, 'rad')
            visible = value > 0
        except Exception:
            visible = True
        handInput.isVisible = visible
        cutterInput.isVisible = visible


# ===========================================================================================
# S2-S27 -- the generator
# ===========================================================================================

class BevelGearGenerator:
    """Plain class -- no `base.Generator`, no `GenerationContext`. Bevel registers no Fusion
    user parameters ([PB-PRECOMPUTED-MODE])."""

    _PINION_MESH_PHASE_TEETH = 0
    _CROWN_PER_RAD = 0.5

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    # -- rollback ---------------------------------------------------------------------------

    def deleteComponent(self) -> None:
        if self.bevelOccurrence is not None:
            try:
                self.bevelOccurrence.deleteMe()
            except Exception:
                pass
        self.bevelOccurrence = None

    # -- S2 -----------------------------------------------------------------------------------

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        design = self.design
        unitsManager: adsk.core.UnitsManager = design.unitsManager

        parentEntities = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentEntities) != 1:
            raise Exception(f'Parent Component: expected exactly one selection, got {len(parentEntities)}')
        parentEntity = parentEntities[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentEntity.component
        elif parentEntity.objectType == adsk.fusion.Component.classType():
            parentComponent = parentEntity
        else:
            raise Exception(f'Parent Component: unexpected selection type {parentEntity.objectType}')

        planeEntities = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeEntities) != 1:
            raise Exception(f'Target Plane: expected exactly one selection, got {len(planeEntities)}')
        targetPlane = planeEntities[0]

        centerEntities = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centerEntities) != 1:
            raise Exception(f'Center Point: expected exactly one selection, got {len(centerEntities)}')
        centerPoint = centerEntities[0]

        def evalExpr(inputId: str, units: str) -> float:
            # An annotation in the enclosing _readInputs scope does not reach this nested
            # def, so both receivers are re-annotated locally, inside the closure.
            commandInputs: adsk.core.CommandInputs = inputs
            valueInput: adsk.core.ValueCommandInput = commandInputs.itemById(inputId)
            um: adsk.core.UnitsManager = unitsManager
            return um.evaluateExpression(valueInput.expression, units)

        module = evalExpr(INPUT_ID_MODULE, '')
        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))

        drivingBaseHeight_cm = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeight_cm = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore_cm = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore_cm = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        faceWidth_cm = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing_cm = evalExpr(INPUT_ID_TOOTH_SPACING, 'mm')
        spiralAngle_rad = evalExpr(INPUT_ID_SPIRAL_ANGLE, 'deg')

        handInput: adsk.core.DropDownCommandInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem: adsk.core.ListItem = handInput.selectedItem
        hand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        cutterRadius_cm = evalExpr(INPUT_ID_CUTTER_RADIUS, 'mm')
        toeExtension = evalExpr(INPUT_ID_TOE_EXTENSION, '')

        # [DEFECT] S2's enumerated stash list omits Toe Extension and both Toe Radii even
        # though S6 needs all three by name; stashed here under the same naming convention.
        drivingToeRadius_cm = evalExpr(INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        pinionToeRadius_cm = evalExpr(INPUT_ID_PINION_TOE_RADIUS, 'mm')

        # ---- Validation 1: basic ranges -----------------------------------------------
        if module <= 0:
            raise Exception('Module must be greater than 0')
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')
        for (label, value) in (
                ('Driving Gear Base Height', drivingBaseHeight_cm),
                ('Pinion Gear Base Height', pinionBaseHeight_cm),
                ('Driving Gear Bore Diameter', drivingBore_cm),
                ('Pinion Gear Bore Diameter', pinionBore_cm),
                ('Face Width', faceWidth_cm),
                ('Tooth Spacing', toothSpacing_cm),
                ('Driving Gear Toe Radius', drivingToeRadius_cm),
                ('Pinion Gear Toe Radius', pinionToeRadius_cm),
                ('Cutter Radius', cutterRadius_cm)):
            if value < 0:
                raise Exception(f'{label} must not be negative')
        if not (0 <= toeExtension <= 100):
            raise Exception('Toe Extension must be between 0 and 100')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if not (0 <= spiralAngle_deg < 60):
            raise Exception('Mean Spiral Angle must be between 0 and 60 degrees')

        # ---- Validation 2: Shaft Angle range -------------------------------------------
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        dpd_mm = module * drivingTeeth
        ppd_mm = module * pinionTeeth
        coneLimit_deg = math.degrees(math.acos(-min(dpd_mm, ppd_mm) / max(dpd_mm, ppd_mm)))
        if coneLimit_deg <= 150.0:
            if shaftAngle_deg >= coneLimit_deg:
                raise Exception(
                    f'Shaft Angle must be below {coneLimit_deg:.6f} degrees (cone-angle limit)')
        else:
            if shaftAngle_deg > 150.0:
                raise Exception('Shaft Angle must be at most 150 degrees')
        if shaftAngle_deg < 30.0:
            raise Exception('Shaft Angle must be at least 30 degrees')

        # ---- Closed-form cone angles ----------------------------------------------------
        gamma_p = math.atan2(
            math.sin(shaftAngle_rad) * ppd_mm, dpd_mm + ppd_mm * math.cos(shaftAngle_rad))
        gamma_g = shaftAngle_rad - gamma_p

        # ---- Validation 3: Minimum Teeth floor -------------------------------------------
        drivingFloor = _MIN_TEETH_FACTOR * math.cos(gamma_g)
        if drivingTeeth < drivingFloor:
            raise Exception(f'Driving Gear Teeth must be at least {drivingFloor:.4f}')
        pinionFloor = _MIN_TEETH_FACTOR * math.cos(gamma_p)
        if pinionTeeth < pinionFloor:
            raise Exception(f'Pinion Gear Teeth must be at least {pinionFloor:.4f}')

        # ---- Validation 4: Base Height bounds, resolved per gear -------------------------
        def resolveBaseHeight(label, userValue_cm, r_mm, gamma, fallback_cm):
            minH_cm = to_cm(1.05 * 1.25 * module * math.sin(gamma))
            maxH_cm = to_cm(0.95 * (r_mm - 1.25 * module * math.cos(gamma)) * math.tan(gamma))
            if userValue_cm > 0:
                if userValue_cm < minH_cm or userValue_cm > maxH_cm:
                    raise Exception(
                        f'{label} must be between {to_mm(minH_cm):.6f} mm and {to_mm(maxH_cm):.6f} mm')
                return userValue_cm
            return min(max(fallback_cm, minH_cm), maxH_cm)

        drivingFallback_cm = to_cm(module * drivingTeeth / 8.0)
        drivingBaseHeightResolved_cm = resolveBaseHeight(
            'Driving Gear Base Height', drivingBaseHeight_cm, dpd_mm / 2.0, gamma_g, drivingFallback_cm)

        pinionFallback_cm = drivingBaseHeightResolved_cm * (pinionTeeth / drivingTeeth)
        pinionBaseHeightResolved_cm = resolveBaseHeight(
            'Pinion Gear Base Height', pinionBaseHeight_cm, ppd_mm / 2.0, gamma_p, pinionFallback_cm)

        pitchConeDistance_cm = to_cm((ppd_mm / 2.0) / math.sin(gamma_p))
        coneDistanceDiag_cm = to_cm(math.sqrt(dpd_mm ** 2 + ppd_mm ** 2))

        # ---- stash --------------------------------------------------------------------
        self._drivingBaseHeight_cm = drivingBaseHeight_cm
        self._pinionBaseHeight_cm = pinionBaseHeight_cm
        self._boreEnable = boreEnable
        self._drivingBore_cm = drivingBore_cm
        self._pinionBore_cm = pinionBore_cm
        self._faceWidth_cm = faceWidth_cm
        self._toothSpacing_cm = toothSpacing_cm
        self._spiralAngle_rad = spiralAngle_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius_cm
        self._toeExtension = toeExtension
        self._drivingToeRadius_cm = drivingToeRadius_cm
        self._pinionToeRadius_cm = pinionToeRadius_cm

        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._moduleMm = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._drivingPitchDia_cm = to_cm(dpd_mm)
        self._pinionPitchDia_cm = to_cm(ppd_mm)
        self._dedendum_cm = to_cm(1.25 * module)
        self._pitchConeDistance_cm = pitchConeDistance_cm
        self._coneDistanceDiag_cm = coneDistanceDiag_cm
        self._drivingBaseHeightResolved_cm = drivingBaseHeightResolved_cm
        self._pinionBaseHeightResolved_cm = pinionBaseHeightResolved_cm

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # -- S3 -----------------------------------------------------------------------------------

    def _stepComponentTree(self, parentComponent: adsk.fusion.Component) -> None:
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelComponent = self.bevelOccurrence.component

        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.designOccurrence.component.name = 'Design'
        self.designComponent = self.designOccurrence.component

    # -- S4 -----------------------------------------------------------------------------------

    def _stepAnchorSketch(self, targetPlane, centerPoint) -> adsk.fusion.SketchLine:
        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projected = sketch.project(centerPoint)
        projectedCentre: adsk.fusion.SketchPoint = projected.item(0)
        cx, cy = projectedCentre.geometry.x, projectedCentre.geometry.y

        line = sketch.sketchCurves.sketchLines.addByTwoPoints(_pt(cx - 0.5, cy), _pt(cx + 0.5, cy))
        line.isConstruction = True

        sketch.geometricConstraints.addCoincident(projectedCentre, line)
        sketch.geometricConstraints.addMidPoint(projectedCentre, line)
        sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, _pt(cx, cy + 0.3))
        sketch.geometricConstraints.addHorizontal(line)

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')

        self._anchorCenterPoint = projectedCentre
        self._anchorLine = line
        return line

    # -- S5 -----------------------------------------------------------------------------------

    def _stepGearProfilesPlane(self, targetPlane,
                                anchorLine: adsk.fusion.SketchLine) -> adsk.fusion.ConstructionPlane:
        planeInput = self.designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = self.designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        return plane

    # -- S6 -----------------------------------------------------------------------------------

    def _stepGearProfiles(self, targetPlane, gearProfilesPlane: adsk.fusion.ConstructionPlane) -> dict:
        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch

        gammaP = self._gamma_p
        gammaG = self._gamma_g
        R = self._pitchConeDistance_cm
        ded = self._dedendum_cm
        ppd = self._pinionPitchDia_cm
        dpd = self._drivingPitchDia_cm
        drivingBH = self._drivingBaseHeightResolved_cm
        pinionBH = self._pinionBaseHeightResolved_cm
        sigma = gammaP + gammaG

        # ---- project the Anchor sketch's centre and anchor line ------------------------
        projectedCentre: adsk.fusion.SketchPoint = sketch.project(self._anchorCenterPoint).item(0)
        cxy = (projectedCentre.geometry.x, projectedCentre.geometry.y)
        projectedAnchorLine: adsk.fusion.SketchLine = sketch.project(self._anchorLine).item(0)
        sx = projectedAnchorLine.startSketchPoint.geometry.x
        sy = projectedAnchorLine.startSketchPoint.geometry.y
        ex = projectedAnchorLine.endSketchPoint.geometry.x
        ey = projectedAnchorLine.endSketchPoint.geometry.y
        dXY = _v2norm((ex - sx, ey - sy))

        # ---- growSide: the one permitted world reading -----------------------------------
        perpCandidate = (-dXY[1], dXY[0])
        cWorld = sketch.sketchToModelSpace(_pt(*cxy))
        cPerpWorld = sketch.sketchToModelSpace(_pt(cxy[0] + perpCandidate[0], cxy[1] + perpCandidate[1]))
        worldDir = adsk.core.Vector3D.create(
            cPerpWorld.x - cWorld.x, cPerpWorld.y - cWorld.y, cPerpWorld.z - cWorld.z)
        normal = get_normal(targetPlane)
        g = 1.0 if worldDir.dotProduct(normal) >= 0 else -1.0
        perp = (g * perpCandidate[0], g * perpCandidate[1])

        # ---- seed directions --------------------------------------------------------------
        drivingDir = (-perp[0], -perp[1])  # apex -> B

        # pinionDir: form both candidates by rotating drivingDir by +-Sigma about the apex,
        # keep the candidate with the GREATER X ([PB-SEED-NEAR] disambiguation rule).
        candPlus = _v2rot(drivingDir, sigma)
        candMinus = _v2rot(drivingDir, -sigma)
        pinionDir = candPlus if candPlus[0] > candMinus[0] else candMinus

        apexSeed = _v2add(cxy, _v2mul(perp, R * math.cos(gammaG) + drivingBH))
        bSeed = _v2add(apexSeed, _v2mul(drivingDir, R * math.cos(gammaG)))
        aSeed = _v2add(apexSeed, _v2mul(pinionDir, R * math.cos(gammaP)))

        # dropA/dropB sense: each drop points toward the OTHER shaft axis.
        dropACand1 = _v2rot(pinionDir, math.pi / 2)
        dropACand2 = _v2rot(pinionDir, -math.pi / 2)
        dropADir = dropACand1 if _v2dot(dropACand1, drivingDir) > 0 else dropACand2
        dropBCand1 = _v2rot(drivingDir, math.pi / 2)
        dropBCand2 = _v2rot(drivingDir, -math.pi / 2)
        dropBDir = dropBCand1 if _v2dot(dropBCand1, pinionDir) > 0 else dropBCand2

        apex2Seed = _v2add(aSeed, _v2mul(dropADir, ppd / 2.0))
        apex2SeedFromB = _v2add(bSeed, _v2mul(dropBDir, dpd / 2.0))

        # dedendum direction: pinion ends away from the anchor line, driving ends toward it.
        pitchDir = _v2norm(_v2sub(apex2Seed, apexSeed))
        dedCand1 = _v2rot(pitchDir, math.pi / 2)
        dedCand2 = _v2rot(pitchDir, -math.pi / 2)
        towardCentre = _v2sub(cxy, apex2Seed)
        if _v2dot(dedCand1, towardCentre) > 0:
            towardAnchorDir, awayFromAnchorDir = dedCand1, dedCand2
        else:
            towardAnchorDir, awayFromAnchorDir = dedCand2, dedCand1
        pinionDedDir = awayFromAnchorDir
        drivingDedDir = towardAnchorDir

        cSeed = _v2add(apex2Seed, _v2mul(pinionDedDir, ded))
        dSeed = _v2add(apex2Seed, _v2mul(drivingDedDir, ded))
        eSeed = _v2add(apexSeed, _v2mul(pinionDir, R * math.cos(gammaP) + ded * math.sin(gammaP)))
        fSeed = _v2add(apexSeed, _v2mul(drivingDir, R * math.cos(gammaG) + ded * math.sin(gammaG)))
        gSeed = _v2add(apexSeed, _v2mul(pinionDir, R * math.cos(gammaP) + pinionBH))
        iSeed = _v2add(apexSeed, _v2mul(drivingDir, R * math.cos(gammaG) + drivingBH))
        hSeed = _v2add(cSeed, _v2mul(pinionDedDir, pinionBH / math.sin(gammaP) - ded))
        jSeed = _v2add(dSeed, _v2mul(drivingDedDir, drivingBH / math.sin(gammaG) - ded))
        kSeed = _v2add(apexSeed, _v2mul(pinionDir, R / math.cos(gammaP)))
        lSeed = _v2add(apexSeed, _v2mul(drivingDir, R / math.cos(gammaG)))

        # =====================================================================================
        # Build the figure
        # =====================================================================================
        gc = sketch.geometricConstraints

        centerToApex = _rawLine(sketch, cxy, apexSeed)
        _pinPoint(sketch, centerToApex.startSketchPoint, projectedCentre)
        gc.addPerpendicular(centerToApex, projectedAnchorLine)
        apexPoint = centerToApex.endSketchPoint

        drivingShaftAxis = _rawLine(sketch, apexSeed, bSeed)
        _pinPoint(sketch, drivingShaftAxis.startSketchPoint, apexPoint)
        gc.addParallel(drivingShaftAxis, centerToApex)

        pinionShaftAxis = _rawLine(sketch, apexSeed, aSeed)
        _pinPoint(sketch, pinionShaftAxis.startSketchPoint, apexPoint)
        bisector = _v2norm(_v2add(pinionDir, drivingDir))
        angleText = _v2add(apexSeed, _v2mul(bisector, ppd / 4.0))
        _angularDim(sketch, pinionShaftAxis, drivingShaftAxis, sigma, angleText)

        dropA = _rawLine(sketch, aSeed, apex2Seed)
        _pinPoint(sketch, dropA.startSketchPoint, pinionShaftAxis.endSketchPoint)
        gc.addPerpendicular(dropA, pinionShaftAxis)
        _alignedDim(sketch, dropA.startSketchPoint, dropA.endSketchPoint, ppd / 2.0,
                    _v2add(aSeed, _v2mul(dropADir, ppd / 4.0)))

        dropB = _rawLine(sketch, bSeed, apex2SeedFromB)
        _pinPoint(sketch, dropB.startSketchPoint, drivingShaftAxis.endSketchPoint)
        gc.addPerpendicular(dropB, drivingShaftAxis)
        _alignedDim(sketch, dropB.startSketchPoint, dropB.endSketchPoint, dpd / 2.0,
                    _v2add(bSeed, _v2mul(dropBDir, dpd / 4.0)))
        gc.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        pitchLine = _rawLine(sketch, apexSeed, apex2Seed)
        _pinPoint(sketch, pitchLine.startSketchPoint, apexPoint)
        _pinPoint(sketch, pitchLine.endSketchPoint, apex2Point)

        pinionDedendumLine = _rawLine(sketch, apex2Seed, cSeed)
        _pinPoint(sketch, pinionDedendumLine.startSketchPoint, apex2Point)
        gc.addPerpendicular(pinionDedendumLine, pitchLine)
        _alignedDim(sketch, pinionDedendumLine.startSketchPoint, pinionDedendumLine.endSketchPoint,
                    ded, _v2add(apex2Seed, _v2mul(pinionDedDir, ded / 2.0)))
        cPoint = pinionDedendumLine.endSketchPoint

        drivingDedendumLine = _rawLine(sketch, apex2Seed, dSeed)
        _pinPoint(sketch, drivingDedendumLine.startSketchPoint, apex2Point)
        gc.addPerpendicular(drivingDedendumLine, pitchLine)
        _alignedDim(sketch, drivingDedendumLine.startSketchPoint, drivingDedendumLine.endSketchPoint,
                    ded, _v2add(apex2Seed, _v2mul(drivingDedDir, ded / 2.0)))
        dPoint = drivingDedendumLine.endSketchPoint

        pinionRootAxis = _rawLine(sketch, apexSeed, cSeed)
        _pinPoint(sketch, pinionRootAxis.startSketchPoint, apexPoint)
        _pinPoint(sketch, pinionRootAxis.endSketchPoint, cPoint)

        drivingRootAxis = _rawLine(sketch, apexSeed, dSeed)
        _pinPoint(sketch, drivingRootAxis.startSketchPoint, apexPoint)
        _pinPoint(sketch, drivingRootAxis.endSketchPoint, dPoint)

        # ---- pinion module-length chain --------------------------------------------------
        segmentAE = _rawLine(sketch, aSeed, eSeed)
        _pinPoint(sketch, segmentAE.startSketchPoint, pinionShaftAxis.endSketchPoint)
        gc.addCollinear(segmentAE, pinionShaftAxis)
        aPoint = segmentAE.startSketchPoint

        segmentCE = _rawLine(sketch, cSeed, eSeed)
        _pinPoint(sketch, segmentCE.startSketchPoint, cPoint)
        _pinPoint(sketch, segmentCE.endSketchPoint, segmentAE.endSketchPoint)
        gc.addPerpendicular(segmentAE, segmentCE)
        ePoint = segmentAE.endSketchPoint

        segmentEG = _rawLine(sketch, eSeed, gSeed)
        _pinPoint(sketch, segmentEG.startSketchPoint, ePoint)
        gc.addCollinear(segmentEG, segmentAE)
        gPoint = segmentEG.endSketchPoint

        segmentCH = _rawLine(sketch, cSeed, hSeed)
        _pinPoint(sketch, segmentCH.startSketchPoint, cPoint)
        gc.addCollinear(segmentCH, pinionDedendumLine)
        hPoint = segmentCH.endSketchPoint

        segmentGH = _rawLine(sketch, gSeed, hSeed)
        _pinPoint(sketch, segmentGH.startSketchPoint, gPoint)
        _pinPoint(sketch, segmentGH.endSketchPoint, hPoint)
        gc.addPerpendicular(segmentEG, segmentGH)

        # ---- driving module-length chain (twin) -------------------------------------------
        segmentBF = _rawLine(sketch, bSeed, fSeed)
        _pinPoint(sketch, segmentBF.startSketchPoint, drivingShaftAxis.endSketchPoint)
        gc.addCollinear(segmentBF, drivingShaftAxis)
        bPoint = segmentBF.startSketchPoint

        segmentDF = _rawLine(sketch, dSeed, fSeed)
        _pinPoint(sketch, segmentDF.startSketchPoint, dPoint)
        _pinPoint(sketch, segmentDF.endSketchPoint, segmentBF.endSketchPoint)
        gc.addPerpendicular(segmentBF, segmentDF)

        segmentFI = _rawLine(sketch, fSeed, iSeed)
        _pinPoint(sketch, segmentFI.startSketchPoint, segmentBF.endSketchPoint)
        gc.addCollinear(segmentFI, segmentBF)
        iPoint = segmentFI.endSketchPoint

        segmentDJ = _rawLine(sketch, dSeed, jSeed)
        _pinPoint(sketch, segmentDJ.startSketchPoint, dPoint)
        gc.addCollinear(segmentDJ, drivingDedendumLine)
        jPoint = segmentDJ.endSketchPoint

        segmentIJ = _rawLine(sketch, iSeed, jSeed)
        _pinPoint(sketch, segmentIJ.startSketchPoint, iPoint)
        _pinPoint(sketch, segmentIJ.endSketchPoint, jPoint)
        gc.addPerpendicular(segmentFI, segmentIJ)

        # ---- base-height offsets -----------------------------------------------------------
        _offsetDim(sketch, dropB, segmentIJ, drivingBH,
                   _v2add(apex2SeedFromB, _v2mul(dropBDir, dpd / 4.0)))
        _offsetDim(sketch, dropA, segmentGH, pinionBH,
                   _v2add(apex2Seed, _v2mul(dropADir, ppd / 4.0)))

        # ---- constrain I with the projected centre -----------------------------------------
        gc.addCoincident(iPoint, projectedCentre)

        # ---- tooth centres K, L --------------------------------------------------------------
        lineGK = _rawLine(sketch, gSeed, kSeed)
        _pinPoint(sketch, lineGK.startSketchPoint, gPoint)
        kPoint = lineGK.endSketchPoint
        gc.addCoincident(kPoint, pinionShaftAxis)
        gc.addCoincident(kPoint, pinionDedendumLine)
        lineCK = _rawLine(sketch, cSeed, kSeed)
        _pinPoint(sketch, lineCK.startSketchPoint, cPoint)
        _pinPoint(sketch, lineCK.endSketchPoint, kPoint)

        lineIL = _rawLine(sketch, iSeed, lSeed)
        _pinPoint(sketch, lineIL.startSketchPoint, iPoint)
        lPoint = lineIL.endSketchPoint
        gc.addCoincident(lPoint, drivingShaftAxis)
        gc.addCoincident(lPoint, drivingDedendumLine)
        lineDL = _rawLine(sketch, dSeed, lSeed)
        _pinPoint(sketch, lineDL.startSketchPoint, dPoint)
        _pinPoint(sketch, lineDL.endSketchPoint, lPoint)

        # ---- Tooth Spacing shift: K', L' ------------------------------------------------------
        toothSpacing = self._toothSpacing_cm
        if toothSpacing > 0:
            kPrimeSeed = _v2add(kSeed, _v2mul(pinionDedDir, toothSpacing))
            shiftK = _rawLine(sketch, kSeed, kPrimeSeed)
            _pinPoint(sketch, shiftK.startSketchPoint, kPoint)
            kPrimePoint = shiftK.endSketchPoint
            gc.addCoincident(kPrimePoint, pinionDedendumLine)
            _alignedDim(sketch, shiftK.startSketchPoint, shiftK.endSketchPoint, toothSpacing,
                        _v2add(kSeed, _v2mul(pinionDedDir, toothSpacing / 2.0)))
            lineCKPrime = _rawLine(sketch, cSeed, kPrimeSeed)
            _pinPoint(sketch, lineCKPrime.startSketchPoint, cPoint)
            _pinPoint(sketch, lineCKPrime.endSketchPoint, kPrimePoint)
            pinionToothCentreLine = lineCKPrime
            kPrimePointFinal = kPrimePoint

            lPrimeSeed = _v2add(lSeed, _v2mul(drivingDedDir, toothSpacing))
            shiftL = _rawLine(sketch, lSeed, lPrimeSeed)
            _pinPoint(sketch, shiftL.startSketchPoint, lPoint)
            lPrimePoint = shiftL.endSketchPoint
            gc.addCoincident(lPrimePoint, drivingDedendumLine)
            _alignedDim(sketch, shiftL.startSketchPoint, shiftL.endSketchPoint, toothSpacing,
                        _v2add(lSeed, _v2mul(drivingDedDir, toothSpacing / 2.0)))
            lineDLPrime = _rawLine(sketch, dSeed, lPrimeSeed)
            _pinPoint(sketch, lineDLPrime.startSketchPoint, dPoint)
            _pinPoint(sketch, lineDLPrime.endSketchPoint, lPrimePoint)
            drivingToothCentreLine = lineDLPrime
            lPrimePointFinal = lPrimePoint
        else:
            pinionToothCentreLine = lineCK
            kPrimePointFinal = kPoint
            drivingToothCentreLine = lineDL
            lPrimePointFinal = lPoint

        # ---- Resolve the Maximum Face Width (SOLVED geometry, [PB-SOLVED-GEOMETRY]) ----------
        aXY = (aPoint.geometry.x, aPoint.geometry.y)
        bXY = (bPoint.geometry.x, bPoint.geometry.y)
        cXY = (cPoint.geometry.x, cPoint.geometry.y)
        dXY2 = (dPoint.geometry.x, dPoint.geometry.y)
        hXY = (hPoint.geometry.x, hPoint.geometry.y)
        jXY = (jPoint.geometry.x, jPoint.geometry.y)

        distA = _perpDistToLine(aXY, cXY, hXY)
        distB = _perpDistToLine(bXY, dXY2, jXY)
        maxFaceWidth_cm = 0.95 * min(distA, distB)
        autoFaceWidth_cm = min(self._coneDistanceDiag_cm / 6.0, maxFaceWidth_cm)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(f'Face Width must not exceed {to_mm(maxFaceWidth_cm):.6f} mm')
            faceWidthResolved_cm = self._faceWidth_cm
        else:
            faceWidthResolved_cm = autoFaceWidth_cm
        self._faceWidthResolved_cm = faceWidthResolved_cm

        # ---- Resolve the Root Length and the two Toe Radii -------------------------------------
        apexDed_cm = math.sqrt(R ** 2 + ded ** 2)
        rootLength0_cm = faceWidthResolved_cm * apexDed_cm / R

        def resolveToeRadius(label, pitchRadius_cm, gamma, toeRadiusInput_cm):
            autoToeRadius = pitchRadius_cm - faceWidthResolved_cm / math.sin(gamma)
            ceiling = (pitchRadius_cm - ded * math.cos(gamma)) * (1 - faceWidthResolved_cm / R)
            if toeRadiusInput_cm > 0:
                if toeRadiusInput_cm >= ceiling:
                    raise Exception(
                        f'{label} Gear Toe Radius must be strictly below the ceiling '
                        f'{to_mm(ceiling):.6f} mm')
                return toeRadiusInput_cm, ceiling
            return autoToeRadius, ceiling

        def toeLimit(toeRadius_cm, gamma):
            gammaRoot = gamma - math.atan(ded / R)
            return apexDed_cm - toeRadius_cm / math.sin(gammaRoot)

        pinionToeRadiusResolved_cm, pinionToeCeiling_cm = resolveToeRadius(
            'Pinion', ppd / 2.0, gammaP, self._pinionToeRadius_cm)
        drivingToeRadiusResolved_cm, drivingToeCeiling_cm = resolveToeRadius(
            'Driving', dpd / 2.0, gammaG, self._drivingToeRadius_cm)

        pinionToeLimit_cm = toeLimit(pinionToeRadiusResolved_cm, gammaP)
        drivingToeLimit_cm = toeLimit(drivingToeRadiusResolved_cm, gammaG)
        minToeLimit_cm = min(pinionToeLimit_cm, drivingToeLimit_cm)

        if self._toeExtension > 0:
            if self._pinionToeRadius_cm <= 0 and pinionToeLimit_cm < rootLength0_cm:
                raise Exception(
                    f'Toe Extension must be 0: Pinion Gear Toe Radius needs to be below '
                    f'{to_mm(pinionToeCeiling_cm):.6f} mm before a positive Toe Extension is possible')
            if self._drivingToeRadius_cm <= 0 and drivingToeLimit_cm < rootLength0_cm:
                raise Exception(
                    f'Toe Extension must be 0: Driving Gear Toe Radius needs to be below '
                    f'{to_mm(drivingToeCeiling_cm):.6f} mm before a positive Toe Extension is possible')

        rootLength_cm = rootLength0_cm + (self._toeExtension / 100.0) * 0.99 * (
            minToeLimit_cm - rootLength0_cm)

        # ---- pinion toe line M -> N -------------------------------------------------------------
        rootHatP = _v2norm(_v2sub(cSeed, apexSeed))
        mSeed = _v2add(apexSeed, _v2mul(rootHatP, apexDed_cm - rootLength_cm))
        slideP = (_perpDistFromAxis(mSeed, apexSeed, pinionDir) - pinionToeRadiusResolved_cm) / math.cos(gammaP)
        nSeed = _v2add(mSeed, _v2mul(pinionDedDir, slideP))

        lineMN = _rawLine(sketch, mSeed, nSeed)
        mPoint = lineMN.startSketchPoint
        nPoint = lineMN.endSketchPoint
        gc.addCoincident(mPoint, pinionRootAxis)
        gc.addParallel(lineMN, segmentCH)
        offsetTextP = _v2mul(_v2add(mSeed, cSeed), 0.5)
        _offsetDim(sketch, segmentCH, lineMN, rootLength_cm * R / apexDed_cm, offsetTextP)

        lineMC = _rawLine(sketch, mSeed, cSeed)
        _pinPoint(sketch, lineMC.startSketchPoint, mPoint)
        _pinPoint(sketch, lineMC.endSketchPoint, cPoint)

        # ---- pinion front face N -> A' -----------------------------------------------------------
        aPrimeSeed = _v2add(apexSeed, _v2mul(pinionDir, _v2dot(_v2sub(nSeed, apexSeed), pinionDir)))
        lineNAPrime = _rawLine(sketch, nSeed, aPrimeSeed)
        _pinPoint(sketch, lineNAPrime.startSketchPoint, nPoint)
        aPrimePoint = lineNAPrime.endSketchPoint
        gc.addCoincident(aPrimePoint, pinionShaftAxis)
        gc.addPerpendicular(lineNAPrime, pinionShaftAxis)
        _alignedDim(sketch, lineNAPrime.startSketchPoint, lineNAPrime.endSketchPoint,
                    pinionToeRadiusResolved_cm, _v2mul(_v2add(nSeed, aPrimeSeed), 0.5))

        lineAPrimeG = _rawLine(sketch, aPrimeSeed, gSeed)
        _pinPoint(sketch, lineAPrimeG.startSketchPoint, aPrimePoint)
        _pinPoint(sketch, lineAPrimeG.endSketchPoint, gPoint)

        # ---- driving toe line O -> P (mirror) -----------------------------------------------------
        rootHatG = _v2norm(_v2sub(dSeed, apexSeed))
        oSeed = _v2add(apexSeed, _v2mul(rootHatG, apexDed_cm - rootLength_cm))
        slideG = (_perpDistFromAxis(oSeed, apexSeed, drivingDir) - drivingToeRadiusResolved_cm) / math.cos(gammaG)
        pSeed = _v2add(oSeed, _v2mul(drivingDedDir, slideG))

        lineOP = _rawLine(sketch, oSeed, pSeed)
        oPoint = lineOP.startSketchPoint
        pPoint = lineOP.endSketchPoint
        gc.addCoincident(oPoint, drivingRootAxis)
        gc.addParallel(lineOP, segmentDJ)
        offsetTextG = _v2mul(_v2add(oSeed, dSeed), 0.5)
        _offsetDim(sketch, segmentDJ, lineOP, rootLength_cm * R / apexDed_cm, offsetTextG)

        lineOD = _rawLine(sketch, oSeed, dSeed)
        _pinPoint(sketch, lineOD.startSketchPoint, oPoint)
        _pinPoint(sketch, lineOD.endSketchPoint, dPoint)

        bPrimeSeed = _v2add(apexSeed, _v2mul(drivingDir, _v2dot(_v2sub(pSeed, apexSeed), drivingDir)))
        linePBPrime = _rawLine(sketch, pSeed, bPrimeSeed)
        _pinPoint(sketch, linePBPrime.startSketchPoint, pPoint)
        bPrimePoint = linePBPrime.endSketchPoint
        gc.addCoincident(bPrimePoint, drivingShaftAxis)
        gc.addPerpendicular(linePBPrime, drivingShaftAxis)
        _alignedDim(sketch, linePBPrime.startSketchPoint, linePBPrime.endSketchPoint,
                    drivingToeRadiusResolved_cm, _v2mul(_v2add(pSeed, bPrimeSeed), 0.5))

        lineBPrimeI = _rawLine(sketch, bPrimeSeed, iSeed)
        _pinPoint(sketch, lineBPrimeI.startSketchPoint, bPrimePoint)
        _pinPoint(sketch, lineBPrimeI.endSketchPoint, iPoint)

        # ---- gate --------------------------------------------------------------------------------
        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')

        self._apexSketchPoint = apexPoint

        return {
            'sketch': sketch,
            'apex': apexPoint,
            'apex2': apex2Point,
            'a': aPoint, 'b': bPoint, 'c': cPoint, 'd': dPoint,
            'g': gPoint, 'h': hPoint, 'i': iPoint, 'j': jPoint,
            'k': kPoint, 'l': lPoint, 'kPrime': kPrimePointFinal, 'lPrime': lPrimePointFinal,
            'm': mPoint, 'n': nPoint, 'o': oPoint, 'p': pPoint,
            'aPrime': aPrimePoint, 'bPrime': bPrimePoint,
            'pinionToothCentreLine': pinionToothCentreLine,
            'drivingToothCentreLine': drivingToothCentreLine,
        }

    # -- S7 -----------------------------------------------------------------------------------

    def _stepToothPlane(self, gearLabel: str, toothCentreLine: adsk.fusion.SketchLine,
                         gearProfilesPlane: adsk.fusion.ConstructionPlane) -> adsk.fusion.ConstructionPlane:
        plane = solids.plane_by_angle(self.designComponent, toothCentreLine, gearProfilesPlane, 90)
        plane.name = f'{gearLabel} Plane'
        return plane

    # -- S8 -----------------------------------------------------------------------------------

    def _stepVirtualSpurTooth(self, gearLabel: str, toothPlane: adsk.fusion.ConstructionPlane,
                               pitchDia_cm: float, gamma: float, anchorPoint: adsk.fusion.SketchPoint):
        virtualPitchRadius_mm = (pitchDia_cm * 10.0 / 2.0) / math.cos(gamma)
        virtualTeeth = int(math.floor(2 * virtualPitchRadius_mm / self._moduleMm))

        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(toothPlane)
        sketch.name = f'{gearLabel} Tooth'

        proxy = VirtualSpurProxy(module_mm=self._moduleMm, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
        drawer.draw(anchorPoint, angle=math.radians(180))

        if not sketch.isFullyConstrained:
            futil.log(f'{sketch.name}: sketch is not fully constrained (labelled sketch, [PB-TEXT-HOLDS-DOF])')

        return sketch, proxy._lastToothEmbedded

    # -- S9 -----------------------------------------------------------------------------------

    def _stepToothAxis(self, gearLabel: str, gearProfilesPlane: adsk.fusion.ConstructionPlane,
                        toothCentreLine: adsk.fusion.SketchLine) -> adsk.fusion.ConstructionAxis:
        planeInput = self.designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(toothCentreLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = self.designComponent.constructionPlanes.add(planeInput)

        axisInput = self.designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        axis = self.designComponent.constructionAxes.add(axisInput)
        axis.name = f'{gearLabel} Tooth Axis'
        return axis

    # -- S10 ----------------------------------------------------------------------------------

    def _stepGearProfileHexagon(self, gearLabel: str, gearProfilesPlane: adsk.fusion.ConstructionPlane,
                                 vertexSources):
        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(gearProfilesPlane)
        sketch.name = f'{gearLabel} Profile'

        newPoints = []
        for source in vertexSources:
            newPoints.append(sketch.sketchPoints.add(sketch.modelToSketchSpace(source.worldGeometry)))

        lines = []
        n = len(newPoints)
        for idx in range(n):
            p0 = newPoints[idx]
            p1 = newPoints[(idx + 1) % n]
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
            lines.append(line)

        for line in lines:
            line.startSketchPoint.isFixed = True
            line.endSketchPoint.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')

        return sketch, lines[0]

    # -- S11 ----------------------------------------------------------------------------------

    def _stepRevolveGearBody(self, profileSketch: adsk.fusion.Sketch,
                              shaftAxisEdge: adsk.fusion.SketchLine) -> adsk.fusion.BRepBody:
        profile = profileSketch.profiles.item(0)
        revolveInput = self.designComponent.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolveFeature = self.designComponent.features.revolveFeatures.add(revolveInput)
        return revolveFeature.bodies.item(0)

    # -- S12 ----------------------------------------------------------------------------------

    def _stepLoftTooth(self, toothSketch: adsk.fusion.Sketch, embedded: bool) -> adsk.fusion.BRepBody:
        wantLines = 0 if embedded else 2
        toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)

        loftInput = self.designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loftFeature = self.designComponent.features.loftFeatures.add(loftInput)
        return loftFeature.bodies.item(0)

    # -- S13 / S14-S20 hook ---------------------------------------------------------------------

    def _stepToothBody(self, gearLabel: str, toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                        toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                        toeConeWorld: adsk.core.Point3D, heelConeWorld: adsk.core.Point3D,
                        gearProfilesPlane: adsk.fusion.ConstructionPlane,
                        toothPlane: adsk.fusion.ConstructionPlane,
                        shaftAxisEdge: adsk.fusion.SketchLine, gamma: float) -> adsk.fusion.BRepBody:
        apexWorld = self._apexSketchPoint.worldGeometry
        if self._spiralAngle_rad <= 0:
            return solids.cut_conical_ends(
                self.designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
        return self._stepSpiralToothBody(
            gearLabel, toothBody, gearBody, toeMid, heelMid, toeConeWorld, heelConeWorld, apexWorld,
            gearProfilesPlane, toothPlane, shaftAxisEdge, gamma)

    # -- S14-S20 ----------------------------------------------------------------------------------

    def _stepSpiralToothBody(self, gearLabel: str, toothBody: adsk.fusion.BRepBody, gearBody: adsk.fusion.BRepBody,
                              toeMid: adsk.core.Point3D, heelMid: adsk.core.Point3D,
                              toeConeWorld: adsk.core.Point3D, heelConeWorld: adsk.core.Point3D,
                              apexWorld: adsk.core.Point3D,
                              gearProfilesPlane: adsk.fusion.ConstructionPlane,
                              toothPlane: adsk.fusion.ConstructionPlane,
                              shaftAxisEdge: adsk.fusion.SketchLine, gamma: float) -> adsk.fusion.BRepBody:
        dc = self.designComponent

        # ---- S14: world frame ---------------------------------------------------------------
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        axisStart = shaftAxisEdge.startSketchPoint.worldGeometry
        axisEnd = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir: adsk.core.Vector3D = _wunit(axisStart, axisEnd)
        coneVec: adsk.core.Vector3D = _wunit(apexWorld, heelConeWorld)
        v: adsk.core.Vector3D = axisDir.crossProduct(coneVec)
        v.normalize()

        def distAlong(p: adsk.core.Point3D) -> float:
            delta: adsk.core.Vector3D = _wsub(p, apexWorld)
            return delta.dotProduct(coneVec)

        rToe = distAlong(toeMid)
        rHeel = distAlong(heelMid)
        rMean = 0.5 * (rToe + rHeel)
        span = rHeel - rToe

        coneElementSketch: adsk.fusion.Sketch = dc.sketches.add(gearProfilesPlane)
        coneElementSketch.name = f'{gearLabel} Cone Element'
        coneEnd = _wadd(apexWorld, coneVec, rHeel)
        coneElementLine = coneElementSketch.sketchCurves.sketchLines.addByTwoPoints(apexWorld, coneEnd)
        coneElementLine.isConstruction = True

        tracePlane = solids.plane_by_angle(dc, coneElementLine, gearProfilesPlane, 90)
        tracePlane.name = f'{gearLabel} Trace Plane'

        # ---- S15: cutter arc -----------------------------------------------------------------
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        psi = self._spiralAngle_rad
        rC = self._cutterRadius_cm if self._cutterRadius_cm > 0 else rMean
        cx = rMean - rC * math.sin(psi)
        cy = handSign * rC * math.cos(psi)

        rLo = rToe - 0.06 * span
        rHi = rHeel + 0.06 * span
        toe2d = solids.circle_intersect_nearest(rLo, cx, cy, rC, rMean, 0.0)
        heel2d = solids.circle_intersect_nearest(rHi, cx, cy, rC, rMean, 0.0)

        def tanW(px: float, py: float) -> adsk.core.Point3D:
            return solids.combine_point(apexWorld, px, coneVec, py, v)

        traceSketch: adsk.fusion.Sketch = dc.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(tanW(cx, cy), rC)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        diaDim = traceSketch.sketchDimensions.addDiameterDimension(cutterCircle, tanW(cx + rC, cy))
        diaDim.parameter.value = 2 * rC

        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            tanW(toe2d[0], toe2d[1]), tanW(rMean, 0.0), tanW(heel2d[0], heel2d[1]))
        traceSketch.geometricConstraints.addCoincident(traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, tanW(rMean, 0.0))
        radDim.parameter.value = rC

        # ---- S16: slice into slabs -------------------------------------------------------------
        toothPlaneNormal = get_normal(toothPlane)
        planeOrigin = toothPlane.geometry.origin
        apexToOrigin: adsk.core.Vector3D = _wsub(apexWorld, planeOrigin)
        sideTest = apexToOrigin.dotProduct(toothPlaneNormal)
        sign = 1.0 if sideTest > 0 else -1.0
        offsets = [sign * (k + 1) * (span / 6.0) for k in range(8)]
        pieces = solids.slice_body_by_offset_planes(dc, toothBody, toothPlane, offsets)
        if len(pieces) == 1:
            offsets = [-o for o in offsets]
            pieces = solids.slice_body_by_offset_planes(dc, toothBody, toothPlane, offsets)
            if len(pieces) == 1:
                raise Exception(
                    f'{gearLabel}: slice produced 1 piece after retry, expected >=2 '
                    f'(span={span}, sign tried={sign} then {-sign})')

        # ---- S17: order + drop apex scrap ------------------------------------------------------
        pieces.sort(key=lambda body: distAlong(body.physicalProperties.centerOfMass))
        scrap = pieces[0]
        segments = pieces[1:]
        dc.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(f'{gearLabel}: no segments survive after dropping the apex scrap')

        # ---- S18: twist ---------------------------------------------------------------------------
        phiCrown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phiCrown) / math.sin(gamma)

        def heelFaceOf(body: adsk.fusion.BRepBody):
            best = None
            bestVal = None
            for face in body.faces:
                val = distAlong(face.centroid)
                if bestVal is None or val > bestVal:
                    bestVal = val
                    best = face
            # A BRepBody always has at least one face, so this never actually fires; it
            # narrows best/bestVal away from Optional for every caller instead of letting
            # a None silently reach a later subtraction.
            if best is None or bestVal is None:
                raise Exception(f'{gearLabel}: a body has no faces to find the heel face on')
            return best, bestVal

        def toeFaceOf(body: adsk.fusion.BRepBody):
            best = None
            bestVal = None
            for face in body.faces:
                val = distAlong(face.centroid)
                if bestVal is None or val < bestVal:
                    bestVal = val
                    best = face
            if best is None or bestVal is None:
                raise Exception(f'{gearLabel}: a body has no faces to find the toe face on')
            return best, bestVal

        for seg in segments:
            _, rHeelFaceVal = heelFaceOf(seg)
            ang = -handSign * total * (rMean - rHeelFaceVal) / span
            if ang == 0:
                continue
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moveInput = dc.features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            dc.features.moveFeatures.add(moveInput)

        # ---- S19: crown ------------------------------------------------------------------------
        segments.sort(key=lambda body: heelFaceOf(body)[1])
        crownSegments = segments[:-1]

        self.designOccurrence.activate()
        try:
            for seg in crownSegments:
                face, rHeelFaceVal = heelFaceOf(seg)
                u = (rHeel - rHeelFaceVal) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(f'{gearLabel}: crown factor {factor} is not positive (u={u})')

                candidates = []
                for vtx in face.vertices:
                    p = vtx.geometry
                    toApex: adsk.core.Vector3D = _wsub(p, apexWorld)
                    alongAxis = toApex.dotProduct(axisDir)
                    perpVec = _wsub(p, _wadd(apexWorld, axisDir, alongAxis))
                    dist = math.sqrt(perpVec.x ** 2 + perpVec.y ** 2 + perpVec.z ** 2)
                    candidates.append((dist, p))
                candidates.sort(key=lambda t: t[0])
                root0 = candidates[0][1]
                root1 = candidates[1][1] if len(candidates) > 1 else candidates[0][1]
                midWorld = _wmidpoint(root0, root1)

                faceSketch: adsk.fusion.Sketch = dc.sketches.add(face)
                basePoint = faceSketch.sketchPoints.add(faceSketch.modelToSketchSpace(midWorld))

                bodies = adsk.core.ObjectCollection.create()
                bodies.add(seg)
                scaleInput = dc.features.scaleFeatures.createInput(
                    bodies, basePoint, adsk.core.ValueInput.createByReal(factor))
                dc.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # ---- S20: re-sort, loft, trim ------------------------------------------------------------
        segments.sort(key=lambda body: heelFaceOf(body)[1])
        loftInput = dc.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        firstFace, _ = toeFaceOf(segments[0])
        loftInput.loftSections.add(firstFace)
        for seg in segments:
            face, _ = heelFaceOf(seg)
            loftInput.loftSections.add(face)
        loftFeature = dc.features.loftFeatures.add(loftInput)
        curvedTooth = loftFeature.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in segments:
            dc.features.removeFeatures.add(seg)

        return solids.cut_conical_ends(dc, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # -- S21 ----------------------------------------------------------------------------------

    def _stepCircularPattern(self, gearLabel: str, toothBody: adsk.fusion.BRepBody,
                              shaftAxisEdge: adsk.fusion.SketchLine, teeth: int):
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{gearLabel} Gear'

        bodyCollection = adsk.core.ObjectCollection.create()
        bodyCollection.add(toothBody)
        patternInput = self.designComponent.features.circularPatternFeatures.createInput(
            bodyCollection, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teeth)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternFeature = self.designComponent.features.circularPatternFeatures.add(patternInput)
        return gearOccurrence, patternFeature

    # -- S22 ----------------------------------------------------------------------------------

    def _stepCombineTeeth(self, gearBody: adsk.fusion.BRepBody,
                           patternFeature: adsk.fusion.CircularPatternFeature) -> adsk.fusion.BRepBody:
        toolCollection = adsk.core.ObjectCollection.create()
        for i in range(patternFeature.bodies.count):
            toolCollection.add(patternFeature.bodies.item(i))
        combineInput = self.designComponent.features.combineFeatures.createInput(gearBody, toolCollection)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        self.designComponent.features.combineFeatures.add(combineInput)
        return gearBody

    # -- S23 ----------------------------------------------------------------------------------

    def _stepBoreSketch(self, gearLabel: str, shaftAxisEdge: adsk.fusion.SketchLine, boreDiameter_cm: float):
        planeInput = self.designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        plane = self.designComponent.constructionPlanes.add(planeInput)

        sketch: adsk.fusion.Sketch = self.designComponent.sketches.add(plane)
        sketch.name = f'{gearLabel} Bore'
        radius = boreDiameter_cm / 2.0
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        diaDim = sketch.sketchDimensions.addDiameterDimension(circle, _pt(radius, 0, 0))
        diaDim.parameter.value = boreDiameter_cm

        if not sketch.isFullyConstrained:
            raise Exception(f'{sketch.name}: sketch is not fully constrained')

        return sketch

    # -- S24 ----------------------------------------------------------------------------------

    def _stepBoreCut(self, boreSketch: adsk.fusion.Sketch, gearBody: adsk.fusion.BRepBody) -> None:
        boreProfile = boreSketch.profiles.item(0)
        extrudeInput = self.designComponent.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extrudeInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2 * self._coneDistanceDiag_cm), False)
        extrudeInput.participantBodies = [gearBody]
        self.designComponent.features.extrudeFeatures.add(extrudeInput)

    # -- S25 ----------------------------------------------------------------------------------

    def _pinionMeshPhase(self, pinionTeeth: int) -> float:
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    def _stepMeshRotation(self, gearLabel: str, gearBody: adsk.fusion.BRepBody,
                           shaftAxisEdge: adsk.fusion.SketchLine, drivingTeeth: int, pinionTeeth: int) -> None:
        if gearLabel == 'Driving':
            angle = math.pi / drivingTeeth
        else:
            angle = self._pinionMeshPhase(pinionTeeth)
        solids.rotate_body_about_edge(self.designComponent, gearBody, shaftAxisEdge, angle)

    # -- generate ---------------------------------------------------------------------------------

    def generate(self, inputs: adsk.core.CommandInputs) -> None:
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)

        # -- S3: component tree ---------------------------------------------------------------
        self._stepComponentTree(parentComponent)

        # -- S4: anchor sketch -------------------------------------------------------------------
        self._stepAnchorSketch(targetPlane, centerPoint)

        # -- S5: gear profiles plane --------------------------------------------------------------
        gearProfilesPlane = self._stepGearProfilesPlane(targetPlane, self._anchorLine)

        # -- S6: gear profiles sketch (the Sec.2 lattice) -------------------------------------------
        lattice = self._stepGearProfiles(targetPlane, gearProfilesPlane)

        # -- per-gear tables, pinion first, driving second (S2's generation order) ------------------
        gearPlans = [
            dict(label='Pinion', teeth=pinionTeeth, otherTeeth=drivingTeeth, gamma=self._gamma_p,
                 pitchDia_cm=self._pinionPitchDia_cm, bore_cm=self._pinionBore_cm,
                 toothCentreLine=lattice['pinionToothCentreLine'],
                 anchorPoint=lattice['kPrime'],
                 vertexSources=[lattice['aPrime'], lattice['g'], lattice['h'],
                                lattice['c'], lattice['m'], lattice['n']],
                 toeMidSources=(lattice['m'], lattice['n']),
                 heelMidSources=(lattice['c'], lattice['h']),
                 toeConeSource=lattice['m'], heelConeSource=lattice['c']),
            dict(label='Driving', teeth=drivingTeeth, otherTeeth=pinionTeeth, gamma=self._gamma_g,
                 pitchDia_cm=self._drivingPitchDia_cm, bore_cm=self._drivingBore_cm,
                 toothCentreLine=lattice['drivingToothCentreLine'],
                 anchorPoint=lattice['lPrime'],
                 vertexSources=[lattice['bPrime'], lattice['i'], lattice['j'],
                                lattice['d'], lattice['o'], lattice['p']],
                 toeMidSources=(lattice['o'], lattice['p']),
                 heelMidSources=(lattice['d'], lattice['j']),
                 toeConeSource=lattice['o'], heelConeSource=lattice['d']),
        ]

        for plan in gearPlans:
            gearLabel = plan['label']

            # -- S7 ------------------------------------------------------------------------------
            toothPlane = self._stepToothPlane(gearLabel, plan['toothCentreLine'], gearProfilesPlane)

            # -- S8 ------------------------------------------------------------------------------
            toothSketch, embedded = self._stepVirtualSpurTooth(
                gearLabel, toothPlane, plan['pitchDia_cm'], plan['gamma'], plan['anchorPoint'])

            # -- S9 ------------------------------------------------------------------------------
            self._stepToothAxis(gearLabel, gearProfilesPlane, plan['toothCentreLine'])

            # -- S10 -----------------------------------------------------------------------------
            profileSketch, shaftAxisEdge = self._stepGearProfileHexagon(
                gearLabel, gearProfilesPlane, plan['vertexSources'])

            # -- S11 -----------------------------------------------------------------------------
            gearBody = self._stepRevolveGearBody(profileSketch, shaftAxisEdge)

            # -- S12 -----------------------------------------------------------------------------
            toothBody = self._stepLoftTooth(toothSketch, embedded)

            # -- S13 or S14-S20 --------------------------------------------------------------------
            toeMid = _wmidpoint(plan['toeMidSources'][0].worldGeometry, plan['toeMidSources'][1].worldGeometry)
            heelMid = _wmidpoint(plan['heelMidSources'][0].worldGeometry, plan['heelMidSources'][1].worldGeometry)
            toeConeWorld = plan['toeConeSource'].worldGeometry
            heelConeWorld = plan['heelConeSource'].worldGeometry
            toothBody = self._stepToothBody(
                gearLabel, toothBody, gearBody, toeMid, heelMid, toeConeWorld, heelConeWorld,
                gearProfilesPlane, toothPlane, shaftAxisEdge, plan['gamma'])

            # -- S21 -----------------------------------------------------------------------------
            gearOccurrence, patternFeature = self._stepCircularPattern(
                gearLabel, toothBody, shaftAxisEdge, plan['teeth'])

            # -- S22 -----------------------------------------------------------------------------
            gearBody = self._stepCombineTeeth(gearBody, patternFeature)

            # -- S23/S24: bore, if enabled ---------------------------------------------------------
            if self._boreEnable:
                boreDiameter_cm = plan['bore_cm'] if plan['bore_cm'] > 0 else plan['pitchDia_cm'] / 4.0
                boreSketch = self._stepBoreSketch(gearLabel, shaftAxisEdge, boreDiameter_cm)
                self._stepBoreCut(boreSketch, gearBody)

            # -- S25 -----------------------------------------------------------------------------
            self._stepMeshRotation(gearLabel, gearBody, shaftAxisEdge, drivingTeeth, pinionTeeth)

            # -- S26: move the finished body into the gear's own component --------------------------
            gearBody.moveToComponent(gearOccurrence)

        # -- S27: cleanup ---------------------------------------------------------------------------
        solids.hide_construction_geometry(self.bevelComponent)

