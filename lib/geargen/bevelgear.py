import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .base import get_selection, get_boolean
from .solids import (
    cut_conical_ends,
    apply_conical_cut,
    select_keeper,
    find_cone_faces_by_midpoint,
    surface_distance,
    slice_body_by_offset_planes,
    rotate_body_about_edge,
    plane_by_angle,
    combine_point,
    circle_intersect_nearest,
    hide_construction_geometry,
)
from .utilities import find_profile_by_curve_counts
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .misc import to_cm, get_design

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


# ---------------------------------------------------------------------------
# Small plane / space vector helpers (plain tuples, internal cm).
# ---------------------------------------------------------------------------

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _mul(a, k):
    return (a[0] * k, a[1] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _norm(a):
    return math.hypot(a[0], a[1])


def normalize(a):
    # Unit vector of a 2-D or 3-D tuple.
    n = math.sqrt(sum(x * x for x in a))
    if n == 0:
        raise Exception('bevel gear: cannot normalise a zero-length vector')
    return tuple(x / n for x in a)


def _rot(d, ang):
    s = math.sin(ang)
    c = math.cos(ang)
    return (d[0] * c - d[1] * s, d[0] * s + d[1] * c)


def _sign(x):
    if x < 0:
        return -1.0
    return 1.0


def _mid2(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _p3(t) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(t[0], t[1], 0)


def _xy(p: adsk.core.Point3D):
    return (p.x, p.y)


def _t3(p: adsk.core.Point3D):
    return (p.x, p.y, p.z)


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul3(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _vec3(t) -> adsk.core.Vector3D:
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def _pt3(t) -> adsk.core.Point3D:
    return adsk.core.Point3D.create(t[0], t[1], t[2])


def _first_point(collection: adsk.core.ObjectCollection) -> adsk.fusion.SketchPoint:
    if collection is None or collection.count < 1:
        raise Exception('bevel gear: sketch.project returned nothing for a point')
    return adsk.fusion.SketchPoint.cast(collection.item(0))


def _first_line(collection: adsk.core.ObjectCollection) -> adsk.fusion.SketchLine:
    if collection is None or collection.count < 1:
        raise Exception('bevel gear: sketch.project returned nothing for a line')
    return adsk.fusion.SketchLine.cast(collection.item(0))


class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs: adsk.core.CommandInputs = cmd.commandInputs

        # 1 Target Plane (first so it wins the auto-focus, [PB-AUTOFOCUS-FIRST])
        plane = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        plane.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        plane.setSelectionLimits(1, 1)

        # 2 Center Point
        center = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        center.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        center.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        center.setSelectionLimits(1, 1)

        # 3 Parent Component
        parent = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parent.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parent.setSelectionLimits(1, 1)
        parent.addSelection(get_design().rootComponent)

        # 4 Module
        inputs.addValueInput(INPUT_ID_MODULE, 'Module', '',
                             adsk.core.ValueInput.createByReal(1))
        # 5 Shaft Angle
        inputs.addValueInput(INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
                             adsk.core.ValueInput.createByString('90 deg'))
        # 6 / 7 teeth
        inputs.addValueInput(INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
                             adsk.core.ValueInput.createByReal(31))
        # 8 / 9 base heights
        inputs.addValueInput(INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        # 10 Enable Bore
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)
        # 11 / 12 bores
        inputs.addValueInput(INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        # 13 Face Width
        inputs.addValueInput(INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        # 14 Tooth Spacing
        inputs.addValueInput(INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        # 15 Mean Spiral Angle
        inputs.addValueInput(INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
                             adsk.core.ValueInput.createByString('35 deg'))
        # 16 Hand of Spiral
        hand = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        hand.listItems.add(_HAND_RIGHT, True)
        hand.listItems.add(_HAND_LEFT, False)
        # 17 Cutter Radius
        inputs.addValueInput(INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        # 18 Toe Extension (%)
        inputs.addValueInput(INPUT_ID_TOE_EXTENSION, 'Toe Extension (%)', '',
                             adsk.core.ValueInput.createByReal(0))
        # 19 / 20 toe radii
        inputs.addValueInput(INPUT_ID_DRIVING_TOE_RADIUS, 'Driving Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(INPUT_ID_PINION_TOE_RADIUS, 'Pinion Gear Toe Radius', 'mm',
                             adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        spiral = adsk.core.ValueCommandInput.cast(inputs.itemById(INPUT_ID_SPIRAL_ANGLE))
        hand = adsk.core.DropDownCommandInput.cast(inputs.itemById(INPUT_ID_HAND))
        cutter = adsk.core.ValueCommandInput.cast(inputs.itemById(INPUT_ID_CUTTER_RADIUS))
        if spiral is None or hand is None or cutter is None:
            return
        try:
            units: adsk.core.UnitsManager = adsk.core.UnitsManager.cast(get_design().unitsManager)
            value = units.evaluateExpression(spiral.expression, 'rad')
        except Exception:
            # A half-typed expression: leave both inputs shown.
            hand.isVisible = True
            cutter.isVisible = True
            return
        hand.isVisible = value > 0
        cutter.isVisible = value > 0

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)


class BevelGearGenerator:
    _CROWN_PER_RAD = 0.5
    _PINION_MESH_PHASE_TEETH = 0.0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)
        self.bevelComponent = adsk.fusion.Component.cast(None)
        self.designOccurrence = adsk.fusion.Occurrence.cast(None)
        self.designComponent = adsk.fusion.Component.cast(None)
        self._gearProfilesPlane = adsk.fusion.ConstructionPlane.cast(None)
        self._gpSketch = adsk.fusion.Sketch.cast(None)
        self._anchorSketch = adsk.fusion.Sketch.cast(None)
        self._anchorLine = adsk.fusion.SketchLine.cast(None)
        self._anchorCenterPoint = adsk.fusion.SketchPoint.cast(None)
        self._apexSketchPoint = adsk.fusion.SketchPoint.cast(None)
        self._apex2d = (0.0, 0.0)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = adsk.fusion.Occurrence.cast(None)

    # ------------------------------------------------------------------
    # generate
    # ------------------------------------------------------------------
    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        self._buildComponentTree(parentComponent)
        self._buildAnchorSketch(targetPlane, centerPoint)
        self._buildGearProfilesPlane(targetPlane)
        pinionCtx, drivingCtx = self._buildGearProfiles(targetPlane)

        for ctx in (pinionCtx, drivingCtx):
            self._createGearBody(ctx, module, drivingTeeth)

        hide_construction_geometry(self.bevelComponent)

    # ------------------------------------------------------------------
    # S04 read and validate
    # ------------------------------------------------------------------
    def _evalInput(self, inputs: adsk.core.CommandInputs, inputId: str, units: str) -> float:
        item = adsk.core.ValueCommandInput.cast(inputs.itemById(inputId))
        if item is None:
            raise Exception(f'bevel gear: dialog input "{inputId}" is missing')
        um = self.design.unitsManager
        if not um.isValidExpression(item.expression, units):
            raise Exception(f'Invalid expression "{item.expression}" for input "{inputId}"')
        return um.evaluateExpression(item.expression, units)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        planes = get_selection(inputs, INPUT_ID_PLANE)
        points = get_selection(inputs, INPUT_ID_CENTER_POINT)
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(planes) < 1:
            raise Exception('Bevel gear: select a Target Plane')
        if len(points) < 1:
            raise Exception('Bevel gear: select a Center Point')
        if len(parents) < 1:
            raise Exception('Bevel gear: select a Parent Component')
        targetPlane = planes[0]
        centerPoint = points[0]
        parentEntity = parents[0]
        parentOcc = adsk.fusion.Occurrence.cast(parentEntity)
        if parentOcc is not None:
            parentComponent = parentOcc.component
        else:
            parentComponent = adsk.fusion.Component.cast(parentEntity)
        if parentComponent is None:
            raise Exception('Bevel gear: the Parent Component selection is not a component')

        module = self._evalInput(inputs, INPUT_ID_MODULE, '')
        shaftAngle_rad = self._evalInput(inputs, INPUT_ID_SHAFT_ANGLE, 'deg')
        drivingTeeth = int(round(self._evalInput(inputs, INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(self._evalInput(inputs, INPUT_ID_PINION_TEETH, '')))
        drivingBase = self._evalInput(inputs, INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBase = self._evalInput(inputs, INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        drivingBore = self._evalInput(inputs, INPUT_ID_DRIVING_BORE, 'mm')
        pinionBore = self._evalInput(inputs, INPUT_ID_PINION_BORE, 'mm')
        faceWidth = self._evalInput(inputs, INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing = self._evalInput(inputs, INPUT_ID_TOOTH_SPACING, 'mm')
        spiral_rad = self._evalInput(inputs, INPUT_ID_SPIRAL_ANGLE, 'deg')
        cutterRadius = self._evalInput(inputs, INPUT_ID_CUTTER_RADIUS, 'mm')
        toeExtension = self._evalInput(inputs, INPUT_ID_TOE_EXTENSION, '')
        drivingToe = self._evalInput(inputs, INPUT_ID_DRIVING_TOE_RADIUS, 'mm')
        pinionToe = self._evalInput(inputs, INPUT_ID_PINION_TOE_RADIUS, 'mm')

        handInput = adsk.core.DropDownCommandInput.cast(inputs.itemById(INPUT_ID_HAND))
        hand = _HAND_RIGHT
        if handInput is not None:
            selected = handInput.selectedItem
            if selected is not None:
                hand = selected.name

        # 1. basic ranges
        if not module > 0:
            raise Exception(f'Module must be greater than 0 (got {module})')
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception(
                f'Teeth numbers must be at least 3 (driving {drivingTeeth}, pinion {pinionTeeth})')
        for (label, value) in (
                ('Driving Gear Base Height', drivingBase), ('Pinion Gear Base Height', pinionBase),
                ('Driving Gear Bore Diameter', drivingBore), ('Pinion Gear Bore Diameter', pinionBore),
                ('Face Width', faceWidth), ('Tooth Spacing', toothSpacing),
                ('Driving Gear Toe Radius', drivingToe), ('Pinion Gear Toe Radius', pinionToe),
                ('Cutter Radius', cutterRadius)):
            if value < 0:
                raise Exception(f'{label} must not be negative')
        if toeExtension < 0 or toeExtension > 100:
            raise Exception(f'Toe Extension must be between 0 and 100 percent (got {toeExtension})')
        spiralDeg = math.degrees(spiral_rad)
        if spiralDeg < 0 or spiralDeg >= 60:
            raise Exception(f'Mean Spiral Angle must be in [0, 60) degrees (got {spiralDeg})')

        # 2. Shaft Angle
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        module_cm = to_cm(module)
        ppd = module_cm * pinionTeeth
        dpd = module_cm * drivingTeeth
        smaller = min(ppd, dpd)
        larger = max(ppd, dpd)
        maxShaft = min(math.degrees(math.acos(-smaller / larger)), 150.0)
        coneLimit = math.degrees(math.acos(-smaller / larger))
        if shaftAngle_deg < 30:
            raise Exception(f'Shaft Angle must be at least 30 degrees (got {shaftAngle_deg:.3f})')
        if shaftAngle_deg >= coneLimit or shaftAngle_deg > 150.0:
            raise Exception(
                f'Shaft Angle {shaftAngle_deg:.3f} degrees is not below the Maximum Shaft Angle '
                f'{maxShaft:.3f} degrees for {drivingTeeth}/{pinionTeeth} teeth')

        # 3. cone angles and Pitch Cone Distance
        gamma_p = math.atan2(math.sin(shaftAngle_rad) * ppd, dpd + ppd * math.cos(shaftAngle_rad))
        gamma_g = shaftAngle_rad - gamma_p
        pitchCone = (ppd / 2.0) / math.sin(gamma_p)

        # 4. Minimum Teeth, per gear
        for (label, teeth, gamma) in (('Pinion', pinionTeeth, gamma_p), ('Driving', drivingTeeth, gamma_g)):
            floor = 5.27 * math.cos(gamma)
            if teeth < floor:
                raise Exception(
                    f'{label} Gear Teeth {teeth} is below the minimum {floor:.4f} for this Shaft Angle')

        # 5. base heights, driving first
        ded = 1.25 * module_cm
        drivingBaseResolved = self._resolveBaseHeight(
            'Driving', drivingBase, module_cm * drivingTeeth / 8.0,
            ded, dpd / 2.0, gamma_g)
        pinionBaseResolved = self._resolveBaseHeight(
            'Pinion', pinionBase, drivingBaseResolved * pinionTeeth / drivingTeeth,
            ded, ppd / 2.0, gamma_p)

        self._module_mm = module
        self._module_cm = module_cm
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_rad = shaftAngle_rad
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._ppd_cm = ppd
        self._dpd_cm = dpd
        self._pitchCone_cm = pitchCone
        self._coneDistance_cm = math.sqrt(ppd ** 2 + dpd ** 2)
        self._drivingBaseHeight_cm = drivingBase
        self._pinionBaseHeight_cm = pinionBase
        self._drivingBaseHeightResolved_cm = drivingBaseResolved
        self._pinionBaseHeightResolved_cm = pinionBaseResolved
        self._drivingBore_cm = drivingBore
        self._pinionBore_cm = pinionBore
        self._faceWidth_cm = faceWidth
        self._toothSpacing_cm = toothSpacing
        self._spiralAngle_rad = spiral_rad
        self._hand = hand
        self._cutterRadius_cm = cutterRadius
        self._toeExtension_pct = toeExtension
        self._drivingToeRadius_cm = drivingToe
        self._pinionToeRadius_cm = pinionToe

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    def _resolveBaseHeight(self, label: str, userValue: float, fallback: float,
                           dedendum: float, pitchRadius: float, gamma: float) -> float:
        minBase = 1.05 * dedendum * math.sin(gamma)
        maxBase = 0.95 * (pitchRadius - dedendum * math.cos(gamma)) * math.tan(gamma)
        if userValue != 0:
            if userValue < minBase:
                raise Exception(
                    f'{label} Gear Base Height {userValue * 10:.4f} mm is below the Minimum Base Height '
                    f'{minBase * 10:.4f} mm')
            if userValue > maxBase:
                raise Exception(
                    f'{label} Gear Base Height {userValue * 10:.4f} mm is above the Maximum Base Height '
                    f'{maxBase * 10:.4f} mm')
            return userValue
        if minBase > maxBase:
            raise Exception(
                f'{label} Gear: Minimum Base Height {minBase * 10:.4f} mm exceeds Maximum Base Height '
                f'{maxBase * 10:.4f} mm')
        if fallback < minBase:
            return minBase
        if fallback > maxBase:
            return maxBase
        return fallback

    # ------------------------------------------------------------------
    # S05 component tree
    # ------------------------------------------------------------------
    def _buildComponentTree(self, parentComponent: adsk.fusion.Component):
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.bevelComponent = self.bevelOccurrence.component
        self.bevelComponent.name = 'Bevel Gear'
        self.designOccurrence = self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.designComponent = self.designOccurrence.component
        self.designComponent.name = 'Design'

    # ------------------------------------------------------------------
    # small sketch helpers
    # ------------------------------------------------------------------
    def _ln(self, sketch: adsk.fusion.Sketch, a, b) -> adsk.fusion.SketchLine:
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(_p3(a), _p3(b))
        line.isConstruction = True
        return line

    def _textAt(self, a, b):
        # A text point beside the segment a-b (never on its centre line).
        mid = _mid2(a, b)
        d = _sub(b, a)
        n = _norm(d)
        if n == 0:
            return (mid[0], mid[1] + 0.3)
        return (mid[0] - d[1] / n * 0.3, mid[1] + d[0] / n * 0.3)

    def _lenDim(self, sketch: adsk.fusion.Sketch, p1: adsk.fusion.SketchPoint,
                p2: adsk.fusion.SketchPoint, a, b, value: float):
        dim = sketch.sketchDimensions.addDistanceDimension(
            p1, p2, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3(self._textAt(a, b)))
        if value is not None:
            dim.parameter.value = value
        return dim

    def _targetNormal(self, plane) -> adsk.core.Vector3D:
        cp = adsk.fusion.ConstructionPlane.cast(plane)
        geom = None
        if cp is not None:
            geom = adsk.core.Plane.cast(cp.geometry)
        else:
            face = adsk.fusion.BRepFace.cast(plane)
            if face is not None:
                geom = adsk.core.Plane.cast(face.geometry)
        if geom is None:
            raise Exception('Bevel gear: the Target Plane is neither a construction plane nor a planar face')
        return geom.normal

    # ------------------------------------------------------------------
    # S06 Anchor sketch
    # ------------------------------------------------------------------
    def _buildAnchorSketch(self, targetPlane, centerPoint):
        sketch: adsk.fusion.Sketch = adsk.fusion.Sketch.cast(self.designComponent.sketches.add(targetPlane))
        sketch.name = 'Anchor'
        self._anchorSketch = sketch
        projected = _first_point(sketch.project(centerPoint))
        c = _xy(projected.geometry)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            _p3((c[0] - 0.5, c[1])), _p3((c[0] + 0.5, c[1])))
        line.isConstruction = True
        gc = sketch.geometricConstraints
        gc.addCoincident(projected, line)
        gc.addMidPoint(projected, line)
        sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            _p3((c[0], c[1] + 0.5)))
        gc.addHorizontal(line)
        if not sketch.isFullyConstrained:
            raise Exception('Bevel gear: the Anchor sketch is not fully constrained')
        self._anchorCenterPoint = projected
        self._anchorLine = line

    # ------------------------------------------------------------------
    # S07 Gear Profiles Plane
    # ------------------------------------------------------------------
    def _buildGearProfilesPlane(self, targetPlane):
        planes = self.designComponent.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByAngle(self._anchorLine,
                              adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = planes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = plane

    # ------------------------------------------------------------------
    # S08 Gear Profiles sketch (the lattice)
    # ------------------------------------------------------------------
    def _buildGearProfiles(self, targetPlane):
        dc = self.designComponent
        sketch: adsk.fusion.Sketch = adsk.fusion.Sketch.cast(dc.sketches.add(self._gearProfilesPlane))
        sketch.name = 'Gear Profiles'
        self._gpSketch = sketch
        gc = sketch.geometricConstraints
        Aligned = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        m = self._module_cm
        ded1 = 1.25 * m
        R = self._pitchCone_cm
        sigma = self._shaftAngle_rad
        gp = self._gamma_p
        gg = self._gamma_g
        spacing = self._toothSpacing_cm
        apexToDed = math.hypot(R, ded1)

        # 1. project the Anchor centre and the anchor line
        cproj = _first_point(sketch.project(self._anchorCenterPoint))
        aproj = _first_line(sketch.project(self._anchorLine))
        c = _xy(cproj.geometry)
        d = normalize(_sub(_xy(aproj.endSketchPoint.geometry), _xy(aproj.startSketchPoint.geometry)))
        perp = (-d[1], d[0])
        normal = self._targetNormal(targetPlane)
        w0 = sketch.sketchToModelSpace(_p3(c))
        w1 = sketch.sketchToModelSpace(_p3(_add(c, perp)))
        if ((w1.x - w0.x) * normal.x + (w1.y - w0.y) * normal.y + (w1.z - w0.z) * normal.z) < 0:
            perp = _mul(perp, -1.0)

        sides = []
        for (label, gamma, teeth, pd, bh) in (
                ('Pinion', gp, self._pinionTeeth, self._ppd_cm, self._pinionBaseHeightResolved_cm),
                ('Driving', gg, self._drivingTeeth, self._dpd_cm, self._drivingBaseHeightResolved_cm)):
            sides.append({'label': label, 'gamma': gamma, 'teeth': teeth, 'pd': pd,
                          'pr': pd / 2.0, 'bh': bh})
        pin = sides[0]
        drv = sides[1]

        # closed-form seeds, all in this sketch's own 2-D frame
        apex = _add(c, _mul(perp, R * math.cos(gg) + drv['bh']))
        drivingDir = _mul(perp, -1.0)
        plus = _rot(drivingDir, sigma)
        minus = _rot(drivingDir, -sigma)
        reach = R * math.cos(gp)
        pinionDir = plus
        if _add(apex, _mul(minus, reach))[0] > _add(apex, _mul(plus, reach))[0]:
            pinionDir = minus
        pin['shaftDir'] = pinionDir
        drv['shaftDir'] = drivingDir
        pin['shaftEnd'] = _add(apex, _mul(pinionDir, R * math.cos(gp)))
        drv['shaftEnd'] = _add(apex, _mul(drivingDir, R * math.cos(gg)))
        pitchDir = _rot(drivingDir, gg * _sign(_cross(drivingDir, pinionDir)))
        apex2 = _add(apex, _mul(pitchDir, R))
        u = (-pitchDir[1], pitchDir[0])
        if _dot(u, pinionDir) < 0:
            u = _mul(u, -1.0)
        pin['dedDir'] = u
        drv['dedDir'] = _mul(u, -1.0)
        for s in sides:
            sg = math.sin(s['gamma'])
            s['ded'] = _add(apex2, _mul(s['dedDir'], ded1))
            s['ext'] = _add(s['shaftEnd'], _mul(s['shaftDir'], ded1 * sg))
            s['axisHeel'] = _add(s['shaftEnd'], _mul(s['shaftDir'], s['bh']))
            s['dedHeel'] = _add(apex2, _mul(s['dedDir'], s['bh'] / sg))
            s['vpr'] = s['pr'] / math.cos(s['gamma'])
            s['center'] = _add(apex2, _mul(s['dedDir'], s['vpr']))
            s['centerOff'] = _add(apex2, _mul(s['dedDir'], s['vpr'] + spacing))
            s['checks'] = []
        self._apex2d = apex

        # 2. centre -> Apex
        centerToApex = self._ln(sketch, c, apex)
        gc.addCoincident(centerToApex.startSketchPoint, cproj)
        gc.addPerpendicular(centerToApex, aproj)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint

        # 3. Apex -> B, the Driving Gear Shaft Axis
        drivingAxis = self._ln(sketch, apex, drv['shaftEnd'])
        gc.addCoincident(drivingAxis.startSketchPoint, apexPoint)
        gc.addParallel(drivingAxis, centerToApex)
        drv['axisLine'] = drivingAxis

        # 4. Apex -> A, the Pinion Gear Shaft Axis
        pinionAxis = self._ln(sketch, apex, pin['shaftEnd'])
        gc.addCoincident(pinionAxis.startSketchPoint, apexPoint)
        bisector = normalize(_add(pinionDir, drivingDir))
        angText = _add(apex, _mul(bisector, self._ppd_cm / 4.0))
        angDim = sketch.sketchDimensions.addAngularDimension(pinionAxis, drivingAxis, _p3(angText))
        angDim.parameter.value = sigma
        pin['axisLine'] = pinionAxis

        # 5. A -> Apex 2, the PPD/2 drop
        dropA = self._ln(sketch, pin['shaftEnd'], apex2)
        gc.addCoincident(dropA.startSketchPoint, pinionAxis.endSketchPoint)
        gc.addPerpendicular(dropA, pinionAxis)
        self._lenDim(sketch, dropA.startSketchPoint, dropA.endSketchPoint,
                     pin['shaftEnd'], apex2, self._ppd_cm / 2.0)
        pin['dropLine'] = dropA

        # 6. B -> Apex 2, the DPD/2 drop
        dropB = self._ln(sketch, drv['shaftEnd'], apex2)
        gc.addCoincident(dropB.startSketchPoint, drivingAxis.endSketchPoint)
        gc.addPerpendicular(dropB, drivingAxis)
        self._lenDim(sketch, dropB.startSketchPoint, dropB.endSketchPoint,
                     drv['shaftEnd'], apex2, self._dpd_cm / 2.0)
        drv['dropLine'] = dropB

        # 7. the two drops close on Apex 2
        gc.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # 8. the Pitch Line
        pitchLine = self._ln(sketch, apex, apex2)
        gc.addCoincident(pitchLine.startSketchPoint, apexPoint)
        gc.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # 9 / 10. dedendum lines and root axes
        for s in sides:
            dl = self._ln(sketch, apex2, s['ded'])
            gc.addCoincident(dl.startSketchPoint, apex2Point)
            gc.addPerpendicular(dl, pitchLine)
            self._lenDim(sketch, dl.startSketchPoint, dl.endSketchPoint, apex2, s['ded'], ded1)
            s['dedLine'] = dl
            rl = self._ln(sketch, apex, s['ded'])
            gc.addCoincident(rl.startSketchPoint, apexPoint)
            gc.addCoincident(rl.endSketchPoint, dl.endSketchPoint)
            s['rootLine'] = rl

        # 11. A -> E / B -> F and the C -> E / D -> F closers
        for s in sides:
            axisLine = s['axisLine']
            ext = self._ln(sketch, s['shaftEnd'], s['ext'])
            gc.addCoincident(ext.startSketchPoint, axisLine.endSketchPoint)
            gc.addCollinear(ext, axisLine)
            foot = self._ln(sketch, s['ded'], s['ext'])
            gc.addCoincident(foot.startSketchPoint, s['dedLine'].endSketchPoint)
            gc.addCoincident(foot.endSketchPoint, ext.endSketchPoint)
            gc.addPerpendicular(ext, foot)
            s['extLine'] = ext

        # 12 / 13 / 14. base-height offsets
        for s in sides:
            ext = s['extLine']
            eg = self._ln(sketch, s['ext'], s['axisHeel'])
            gc.addCoincident(eg.startSketchPoint, ext.endSketchPoint)
            gc.addCollinear(eg, ext)
            ch = self._ln(sketch, s['ded'], s['dedHeel'])
            gc.addCoincident(ch.startSketchPoint, s['dedLine'].endSketchPoint)
            gc.addCollinear(ch, s['dedLine'])
            back = self._ln(sketch, s['axisHeel'], s['dedHeel'])
            gc.addCoincident(back.startSketchPoint, eg.endSketchPoint)
            gc.addCoincident(back.endSketchPoint, ch.endSketchPoint)
            gc.addPerpendicular(eg, back)
            offDim = sketch.sketchDimensions.addOffsetDimension(
                s['dropLine'], back, _p3(self._textAt(s['axisHeel'], s['dedHeel'])))
            offDim.parameter.value = s['bh']
            s['egLine'] = eg
            s['heelLine'] = ch

        # 16. point I coincident with the projected centre
        gc.addCoincident(drv['egLine'].endSketchPoint, cproj)

        # 17 / 18. K, L and the tooth centres K', L'
        for s in sides:
            eg = s['egLine']
            kl = self._ln(sketch, s['axisHeel'], s['center'])
            gc.addCoincident(kl.startSketchPoint, eg.endSketchPoint)
            gc.addCoincident(kl.endSketchPoint, s['axisLine'])
            gc.addCoincident(kl.endSketchPoint, s['dedLine'])
            kPoint = kl.endSketchPoint
            s['kPoint'] = kPoint
            if spacing > 0:
                kk = self._ln(sketch, s['center'], s['centerOff'])
                gc.addCoincident(kk.startSketchPoint, kPoint)
                gc.addCoincident(kk.endSketchPoint, s['dedLine'])
                self._lenDim(sketch, kk.startSketchPoint, kk.endSketchPoint,
                             s['center'], s['centerOff'], spacing)
                ref = self._ln(sketch, s['ded'], s['centerOff'])
                gc.addCoincident(ref.startSketchPoint, s['dedLine'].endSketchPoint)
                gc.addCoincident(ref.endSketchPoint, kk.endSketchPoint)
                s['toothPoint'] = kk.endSketchPoint
                s['toothRef'] = ref
            else:
                ref = self._ln(sketch, s['ded'], s['center'])
                gc.addCoincident(ref.startSketchPoint, s['dedLine'].endSketchPoint)
                gc.addCoincident(ref.endSketchPoint, kPoint)
                s['toothPoint'] = kPoint
                s['toothRef'] = ref

        # 19. Maximum Face Width from the SOLVED geometry
        def distToLine(p, a, b):
            return abs(_cross(normalize(_sub(b, a)), _sub(p, a)))

        pointA = _xy(pinionAxis.endSketchPoint.geometry)
        pointB = _xy(drivingAxis.endSketchPoint.geometry)
        pinC = _xy(pin['dedLine'].endSketchPoint.geometry)
        pinH = _xy(pin['heelLine'].endSketchPoint.geometry)
        drvD = _xy(drv['dedLine'].endSketchPoint.geometry)
        drvJ = _xy(drv['heelLine'].endSketchPoint.geometry)
        maxFace = 0.95 * min(distToLine(pointA, pinC, pinH), distToLine(pointB, drvD, drvJ))
        if self._faceWidth_cm == 0:
            faceWidth = min(self._coneDistance_cm / 6.0, maxFace)
        else:
            faceWidth = self._faceWidth_cm
            if faceWidth > maxFace:
                raise Exception(
                    f'Face Width {faceWidth * 10:.4f} mm exceeds the Maximum Face Width '
                    f'{maxFace * 10:.4f} mm')
        self._faceWidthResolved_cm = faceWidth

        # 20. Root Length, Toe Radii and Maximum Bore Diameters
        rootLength0 = faceWidth * apexToDed / R
        dedAngle = math.atan(ded1 / R)
        for s in sides:
            gamma = s['gamma']
            s['gammaRoot'] = gamma - dedAngle
            s['toeCeil'] = (s['pr'] - ded1 * math.cos(gamma)) * (1.0 - faceWidth / R)
            if s['label'] == 'Pinion':
                userToe = self._pinionToeRadius_cm
            else:
                userToe = self._drivingToeRadius_cm
            if userToe != 0:
                if userToe >= s['toeCeil']:
                    raise Exception(
                        f'{s["label"]} Gear Toe Radius {userToe * 10:.4f} mm must be below the Toe Radius '
                        f'Ceiling {s["toeCeil"] * 10:.4f} mm')
                s['toeRadius'] = userToe
            else:
                s['toeRadius'] = s['pr'] - faceWidth / math.sin(gamma)
            if not s['toeRadius'] > 0:
                raise Exception(
                    f'{s["label"]} Gear Toe Radius resolved to {s["toeRadius"] * 10:.4f} mm, '
                    f'which is not positive')
            s['toeLimit'] = apexToDed - s['toeRadius'] / math.sin(s['gammaRoot'])
        limit = min(pin['toeLimit'], drv['toeLimit'])
        rootLength = rootLength0
        if self._toeExtension_pct > 0:
            if limit <= rootLength0:
                tight = pin if pin['toeLimit'] <= drv['toeLimit'] else drv
                raise Exception(
                    f'Toe Extension above 0 is not possible on this pair: the {tight["label"]} Gear Toe Limit '
                    f'{limit * 10:.4f} mm is below the Toe Extension 0 root length {rootLength0 * 10:.4f} mm; '
                    f'its Toe Radius must come below the Toe Radius Ceiling {tight["toeCeil"] * 10:.4f} mm')
            rootLength = rootLength0 + (self._toeExtension_pct / 100.0) * 0.99 * (limit - rootLength0)
        for s in sides:
            s['bore'] = 0.0
            if not self._boreEnable:
                continue
            rHeel = s['pr'] - s['bh'] / math.tan(s['gamma'])
            rToe = (apexToDed - rootLength) * math.sin(s['gammaRoot'])
            maxBore = 2.0 * 0.95 * min(rHeel, rToe)
            if s['label'] == 'Pinion':
                userBore = self._pinionBore_cm
            else:
                userBore = self._drivingBore_cm
            if userBore == 0:
                s['bore'] = min(s['pd'] / 4.0, maxBore)
            else:
                if userBore > maxBore:
                    raise Exception(
                        f'{s["label"]} Gear Bore Diameter {userBore * 10:.4f} mm exceeds the Maximum Bore '
                        f'Diameter {maxBore * 10:.4f} mm')
                s['bore'] = userBore
        self._pinionToeRadiusResolved_cm = pin['toeRadius']
        self._drivingToeRadiusResolved_cm = drv['toeRadius']

        # 15 / 21 / 22 / 23. toe lattice and front faces
        for s in sides:
            shaftDir = s['shaftDir']
            gamma = s['gamma']
            fraction = 1.0 - rootLength / apexToDed
            M = _add(apex, _mul(_sub(s['ded'], apex), fraction))
            radiusM = abs(_cross(_sub(M, apex), shaftDir))
            slide = (radiusM - s['toeRadius']) / math.cos(gamma)
            N = _add(M, _mul(s['dedDir'], slide))
            axisToe = _add(apex, _mul(shaftDir, _dot(_sub(N, apex), shaftDir)))
            s['toeRoot'] = M
            s['toeInner'] = N
            s['axisToe'] = axisToe
            eg = s['egLine']
            ch = s['heelLine']

            ag = None
            if s['label'] == 'Pinion':
                # A' -> G creates A'
                ag = self._ln(sketch, axisToe, s['axisHeel'])
                gc.addCoincident(ag.endSketchPoint, eg.endSketchPoint)

            toe = self._ln(sketch, M, N)
            gc.addCoincident(toe.startSketchPoint, s['rootLine'])
            gc.addParallel(toe, ch)
            toeDim = sketch.sketchDimensions.addOffsetDimension(
                ch, toe, _p3(_mid2(M, s['ded'])))
            toeDim.parameter.value = rootLength * R / apexToDed
            mc = self._ln(sketch, M, s['ded'])
            gc.addCoincident(mc.startSketchPoint, toe.startSketchPoint)
            gc.addCoincident(mc.endSketchPoint, s['dedLine'].endSketchPoint)

            front = self._ln(sketch, N, axisToe)
            gc.addCoincident(front.startSketchPoint, toe.endSketchPoint)
            if ag is not None:
                gc.addCoincident(front.endSketchPoint, ag.startSketchPoint)
                axisToePoint = ag.startSketchPoint
            else:
                axisToePoint = front.endSketchPoint
            gc.addCoincident(axisToePoint, s['axisLine'])
            gc.addPerpendicular(front, s['axisLine'])
            self._lenDim(sketch, front.startSketchPoint, front.endSketchPoint, N, axisToe, s['toeRadius'])
            if ag is None:
                bi = self._ln(sketch, axisToe, s['axisHeel'])
                gc.addCoincident(bi.startSketchPoint, front.endSketchPoint)
                gc.addCoincident(bi.endSketchPoint, eg.endSketchPoint)

            s['hex'] = [axisToePoint, eg.endSketchPoint, ch.endSketchPoint,
                        s['dedLine'].endSketchPoint, toe.startSketchPoint, toe.endSketchPoint]
            s['toePoints'] = (toe.startSketchPoint, toe.endSketchPoint)
            s['heelPoints'] = (s['dedLine'].endSketchPoint, ch.endSketchPoint)
            s['checks'] = [
                (f'{s["label"]} C/D', s['dedLine'].endSketchPoint, s['ded']),
                (f'{s["label"]} E/F', s['extLine'].endSketchPoint, s['ext']),
                (f'{s["label"]} G/I', eg.endSketchPoint, s['axisHeel']),
                (f'{s["label"]} H/J', ch.endSketchPoint, s['dedHeel']),
                (f'{s["label"]} K/L', s['kPoint'], s['center']),
            ]
            if spacing > 0:
                s['checks'].append((f'{s["label"]} K\'/L\'', s['toothPoint'], s['centerOff']))
            s['checks'].append((f'{s["label"]} M/O', toe.startSketchPoint, M))
            s['checks'].append((f'{s["label"]} N/P', toe.endSketchPoint, N))
            s['checks'].append((f'{s["label"]} A\'/B\'', axisToePoint, axisToe))

        # 24. full-constraint gate
        if not sketch.isFullyConstrained:
            raise Exception('Bevel gear: the Gear Profiles sketch is not fully constrained')

        # 25. every named point against its own seed
        order = [('Apex', apexPoint, apex), ('B', drivingAxis.endSketchPoint, drv['shaftEnd']),
                 ('A', pinionAxis.endSketchPoint, pin['shaftEnd']), ('Apex 2', apex2Point, apex2)]
        for s in sides:
            order.extend(s['checks'])
        for (name, point, seed) in order:
            solved = _xy(point.geometry)
            if _norm(_sub(solved, seed)) > 1e-4:
                raise Exception(
                    f'Bevel gear: lattice point {name} moved off its seed: solved '
                    f'({solved[0] * 10:.4f}, {solved[1] * 10:.4f}) mm, seeded '
                    f'({seed[0] * 10:.4f}, {seed[1] * 10:.4f}) mm')

        contexts = []
        for s in sides:
            contexts.append({
                'label': s['label'],
                'teeth': int(s['teeth']),
                'gamma': s['gamma'],
                'pitchDiameter_cm': s['pd'],
                'toothCenterPoint': s['toothPoint'],
                'toothCenterRefLine': s['toothRef'],
                'hexVertices': s['hex'],
                'toeEdgePoints': s['toePoints'],
                'heelEdgePoints': s['heelPoints'],
                'boreDiameter_cm': s['bore'],
                'toothPlane': None,
                'toothSketch': None,
                'toothEmbedded': None,
                'toothAxis': None,
                'gearOccurrence': None,
                'profileSketch': None,
                'shaftAxisEdge': None,
                'gearBody': None,
            })
        return contexts[0], contexts[1]

    # ------------------------------------------------------------------
    # S09 - S26, once per gear
    # ------------------------------------------------------------------
    def _createGearBody(self, ctx, module, drivingTeeth):
        dc = self.designComponent
        label = ctx['label']
        teeth = ctx['teeth']
        gamma = ctx['gamma']
        Zero = adsk.core.ValueInput.createByReal(0.0)

        # S09 tooth plane
        toothCenterRefLine = adsk.fusion.SketchLine.cast(ctx['toothCenterRefLine'])
        toothPlane = adsk.fusion.ConstructionPlane.cast(
            plane_by_angle(dc, toothCenterRefLine, self._gearProfilesPlane, 90))
        toothPlane.name = f'{label} Plane'
        ctx['toothPlane'] = toothPlane

        # S10 tooth sketch: the virtual spur tooth
        virtualPitchRadius_mm = (ctx['pitchDiameter_cm'] * 10.0 / 2.0) / math.cos(gamma)
        virtualTeeth = 2.0 * virtualPitchRadius_mm / module
        rootSink = 0.05 * 2.25 * module
        toothSketch: adsk.fusion.Sketch = adsk.fusion.Sketch.cast(dc.sketches.add(toothPlane))
        toothSketch.name = f'{label} Tooth'
        toothCenterPoint = adsk.fusion.SketchPoint.cast(ctx['toothCenterPoint'])
        anchorPoint = _first_point(toothSketch.project(toothCenterPoint))
        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(anchorPoint, angle=math.radians(180))
        toothEmbedded = bool(proxy._lastToothEmbedded)
        if not toothSketch.isFullyConstrained:
            futil.log(f'{label} Tooth sketch is not reported fully constrained (labels hold a DOF)')
        ctx['toothSketch'] = toothSketch
        ctx['toothEmbedded'] = toothEmbedded

        # S11 tooth axis
        planes = dc.constructionPlanes
        helperInput = planes.createInput()
        helperInput.setByDistanceOnPath(toothCenterRefLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = planes.add(helperInput)
        helperPlane.name = f'{label} Tooth Helper Plane'
        axes = dc.constructionAxes
        axisInput = axes.createInput()
        axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = axes.add(axisInput)
        toothAxis.name = f'{label} Tooth Axis'
        ctx['toothAxis'] = toothAxis

        # S12 gear component
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOccurrence.component.name = f'{label} Gear'
        ctx['gearOccurrence'] = gearOccurrence

        # S13 profile sketch: the hexagon on fixed points
        profileSketch = dc.sketches.add(self._gearProfilesPlane)
        profileSketch.name = f'{label} Profile'
        hexVertices = ctx['hexVertices']
        points = []
        for i in range(len(hexVertices)):
            src = adsk.fusion.SketchPoint.cast(hexVertices[i])
            points.append(profileSketch.sketchPoints.add(
                profileSketch.modelToSketchSpace(src.worldGeometry)))
        hexLines = []
        for i in range(len(points)):
            startPoint = adsk.fusion.SketchPoint.cast(points[i])
            endPoint = adsk.fusion.SketchPoint.cast(points[(i + 1) % len(points)])
            hexLines.append(profileSketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint))
        for i in range(len(hexLines)):
            edge = adsk.fusion.SketchLine.cast(hexLines[i])
            edge.startSketchPoint.isFixed = True
            edge.endSketchPoint.isFixed = True
        if not profileSketch.isFullyConstrained:
            raise Exception(f'Bevel gear: the {label} Profile sketch is not fully constrained')
        shaftAxisEdge = adsk.fusion.SketchLine.cast(hexLines[0])
        ctx['profileSketch'] = profileSketch
        ctx['shaftAxisEdge'] = shaftAxisEdge

        # S14 revolve the Gear Body
        profile = profileSketch.profiles.item(0)
        revolveInput = dc.features.revolveFeatures.createInput(
            profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = dc.features.revolveFeatures.add(revolveInput)
        gearBody = adsk.fusion.BRepBody.cast(revolve.bodies.item(0))
        gearBody.name = f'{label} Gear Body'
        ctx['gearBody'] = gearBody

        # S15 loft the Tooth Body from the Apex point to the tooth profile
        wantLines = 0 if toothEmbedded else 2
        toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput = dc.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        toothLoft = dc.features.loftFeatures.add(loftInput)
        toothBody = adsk.fusion.BRepBody.cast(toothLoft.bodies.item(0))

        # hand-off points, in world space
        toeEdge = ctx['toeEdgePoints']
        heelEdge = ctx['heelEdgePoints']
        toeConePoint = adsk.fusion.SketchPoint.cast(toeEdge[0])
        toeOtherPoint = adsk.fusion.SketchPoint.cast(toeEdge[1])
        heelConePoint = adsk.fusion.SketchPoint.cast(heelEdge[0])
        heelOtherPoint = adsk.fusion.SketchPoint.cast(heelEdge[1])
        toeConeWorld = toeConePoint.worldGeometry
        heelConeWorld = heelConePoint.worldGeometry
        toeMid = adsk.core.Point3D.create(
            (toeConeWorld.x + toeOtherPoint.worldGeometry.x) / 2.0,
            (toeConeWorld.y + toeOtherPoint.worldGeometry.y) / 2.0,
            (toeConeWorld.z + toeOtherPoint.worldGeometry.z) / 2.0)
        heelMid = adsk.core.Point3D.create(
            (heelConeWorld.x + heelOtherPoint.worldGeometry.x) / 2.0,
            (heelConeWorld.y + heelOtherPoint.worldGeometry.y) / 2.0,
            (heelConeWorld.z + heelOtherPoint.worldGeometry.z) / 2.0)
        apexWorld = self._apexSketchPoint.worldGeometry

        # S16 - S21 spiral (when psi > 0) and the conical end cuts
        trimmed = self._transformToothBody(
            dc, toothBody, gearBody, shaftAxisEdge, apexWorld, self._apexSketchPoint,
            toeMid, heelMid, toeConeWorld, heelConeWorld, toothPlane, label, teeth, gamma)
        tooth = adsk.fusion.BRepBody.cast(trimmed)

        # S22 circular pattern
        seedBodies = adsk.core.ObjectCollection.create()
        seedBodies.add(tooth)
        patternInput = dc.features.circularPatternFeatures.createInput(seedBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teeth)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = dc.features.circularPatternFeatures.add(patternInput)
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))

        # S23 combine-join
        combineInput = dc.features.combineFeatures.createInput(gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        dc.features.combineFeatures.add(combineInput)

        # S24 / S25 bore
        boreDiameter = ctx['boreDiameter_cm']
        if self._boreEnable and boreDiameter > 0:
            borePlaneInput = planes.createInput()
            borePlaneInput.setByDistanceOnPath(shaftAxisEdge, Zero)
            borePlane = planes.add(borePlaneInput)
            borePlane.name = f'{label} Bore Plane'
            boreSketch = dc.sketches.add(borePlane)
            boreSketch.name = f'{label} Bore'
            boreCircle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), boreDiameter / 2.0)
            boreCircle.centerSketchPoint.isFixed = True
            boreDim = boreSketch.sketchDimensions.addDiameterDimension(
                boreCircle, adsk.core.Point3D.create(boreDiameter * 0.9, boreDiameter * 0.9, 0))
            boreDim.parameter.value = boreDiameter
            if not boreSketch.isFullyConstrained:
                raise Exception(f'Bevel gear: the {label} Bore sketch is not fully constrained')
            boreInput = dc.features.extrudeFeatures.createInput(
                boreSketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
            boreInput.setSymmetricExtent(
                adsk.core.ValueInput.createByReal(2.0 * self._coneDistance_cm), False)
            boreInput.participantBodies = [gearBody]
            dc.features.extrudeFeatures.add(boreInput)

        # S26 meshing rotation, then move the finished body out
        if label == 'Driving':
            meshAngle = math.pi / drivingTeeth
        else:
            meshAngle = self._pinionMeshPhase(teeth)
        rotate_body_about_edge(dc, gearBody, shaftAxisEdge, meshAngle)
        gearBody.moveToComponent(gearOccurrence)

    def _pinionMeshPhase(self, pinionTeeth) -> float:
        return self._PINION_MESH_PHASE_TEETH * 2.0 * math.pi / pinionTeeth

    # ------------------------------------------------------------------
    # spiral helpers
    # ------------------------------------------------------------------
    def _distAlong(self, p: adsk.core.Point3D, apex3, coneVec3) -> float:
        return _dot3(_sub3(_t3(p), apex3), coneVec3)

    def _tanW(self, apexWorld: adsk.core.Point3D, coneVec3, v3, px: float, py: float) -> adsk.core.Point3D:
        return combine_point(apexWorld, px, _vec3(coneVec3), py, _vec3(v3))

    def _bodyCentroidDist(self, body: adsk.fusion.BRepBody, apex3, coneVec3) -> float:
        return self._distAlong(body.physicalProperties.centerOfMass, apex3, coneVec3)

    def _heelFace(self, body: adsk.fusion.BRepBody, apex3, coneVec3) -> adsk.fusion.BRepFace:
        # The face whose centroid has the greatest distAlong, across ALL faces.
        best = adsk.fusion.BRepFace.cast(None)
        bestDist = 0.0
        for i in range(body.faces.count):
            face = body.faces.item(i)
            dist = self._distAlong(face.centroid, apex3, coneVec3)
            if best is None or dist > bestDist:
                best = face
                bestDist = dist
        if best is None:
            raise Exception('Bevel gear: slab has no faces')
        return best

    def _toeFace(self, body: adsk.fusion.BRepBody, apex3, coneVec3) -> adsk.fusion.BRepFace:
        best = adsk.fusion.BRepFace.cast(None)
        bestDist = 0.0
        for i in range(body.faces.count):
            face = body.faces.item(i)
            dist = self._distAlong(face.centroid, apex3, coneVec3)
            if best is None or dist < bestDist:
                best = face
                bestDist = dist
        if best is None:
            raise Exception('Bevel gear: slab has no faces')
        return best

    def _orderByHeelFace(self, segments, apex3, coneVec3):
        keyed = []
        for i in range(len(segments)):
            seg = adsk.fusion.BRepBody.cast(segments[i])
            heel = self._heelFace(seg, apex3, coneVec3)
            keyed.append((self._distAlong(heel.centroid, apex3, coneVec3), i))
        keyed.sort()
        return keyed

    # ------------------------------------------------------------------
    # S16 - S21 the single tooth-body hook
    # ------------------------------------------------------------------
    def _transformToothBody(self, designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
                            apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            return cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid,
                                    apexWorld, gearLabel)

        dc: adsk.fusion.Component = designComponent
        apex3 = _t3(apexWorld)
        psi = self._spiralAngle_rad

        # A. gate and frame
        if apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid):
            swapMid = toeMid
            toeMid = heelMid
            heelMid = swapMid
            swapCone = toeConeWorld
            toeConeWorld = heelConeWorld
            heelConeWorld = swapCone
        startW = shaftAxisEdge.startSketchPoint.worldGeometry
        endW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = normalize(_sub3(_t3(endW), _t3(startW)))
        coneVec = normalize(_sub3(_t3(heelConeWorld), apex3))
        v = normalize(_cross3(axisDir, coneVec))
        tpNormal = normalize(_cross3(coneVec, v))  # completes the frame; nothing consumes it
        R_toe = self._distAlong(toeMid, apex3, coneVec)
        R_heel = self._distAlong(heelMid, apex3, coneVec)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # B. cutter-arc geometry
        r_c = self._cutterRadius_cm if self._cutterRadius_cm != 0 else R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)
        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
        heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)

        # C. the trace sketch (inspection only, deliberately left with free DOF)
        coneSketch = dc.sketches.add(self._gearProfilesPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneEnd = _pt3(_add3(apex3, _mul3(coneVec, R_heel)))
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(apexWorld, coneEnd)
        tracePlane = adsk.fusion.ConstructionPlane.cast(
            plane_by_angle(dc, coneElementLine, self._gearProfilesPlane, 90))
        tracePlane.name = f'{gearLabel} Trace Plane'
        traceSketch = dc.sketches.add(tracePlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            self._tanW(apexWorld, coneVec, v, Cx, Cy), r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        circleDim = traceSketch.sketchDimensions.addDiameterDimension(
            cutterCircle, self._tanW(apexWorld, coneVec, v, Cx + r_c, Cy))
        circleDim.parameter.value = 2.0 * r_c
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            self._tanW(apexWorld, coneVec, v, toe2d[0], toe2d[1]),
            self._tanW(apexWorld, coneVec, v, R_mean, 0.0),
            self._tanW(apexWorld, coneVec, v, heel2d[0], heel2d[1]))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        arcDim = traceSketch.sketchDimensions.addRadialDimension(
            traceArc, self._tanW(apexWorld, coneVec, v, R_mean, 0.0))
        arcDim.parameter.value = r_c

        # E. slice into eight cross-section slabs
        planeGeom = adsk.core.Plane.cast(parentToothPlane.geometry)
        planeOrigin = _t3(planeGeom.origin)
        planeNormalVec: adsk.core.Vector3D = planeGeom.normal
        planeNormal = (planeNormalVec.x, planeNormalVec.y, planeNormalVec.z)
        sign = 1.0 if _dot3(_sub3(apex3, planeOrigin), planeNormal) > 0 else -1.0
        pieces = slice_body_by_offset_planes(
            dc, toothBody, parentToothPlane, [sign * (k + 1) * span / 6.0 for k in range(8)])
        if len(pieces) < 2:
            sign = -sign
            pieces = slice_body_by_offset_planes(
                dc, toothBody, parentToothPlane, [sign * (k + 1) * span / 6.0 for k in range(8)])
            if len(pieces) < 2:
                raise Exception(
                    f'{gearLabel}: slice produced {len(pieces)} piece(s), expected >= 2; '
                    f'span={span}, last offset sign={sign}')

        # F. order and drop the apex scrap
        keyed = []
        for i in range(len(pieces)):
            piece = adsk.fusion.BRepBody.cast(pieces[i])
            keyed.append((self._bodyCentroidDist(piece, apex3, coneVec), i))
        keyed.sort()
        ordered = [pieces[i] for (key, i) in keyed]
        scrap = adsk.fusion.BRepBody.cast(ordered[0])
        segments = ordered[1:]
        dc.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(f'{gearLabel}: the slice left no segments after dropping the apex scrap')

        # G. twist
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)
        for i in range(len(segments)):
            seg = adsk.fusion.BRepBody.cast(segments[i])
            heelFace = self._heelFace(seg, apex3, coneVec)
            R_hf = self._distAlong(heelFace.centroid, apex3, coneVec)
            ang = -handSign * total * (R_mean - R_hf) / span
            if ang == 0:
                continue
            moving = adsk.core.ObjectCollection.create()
            moving.add(seg)
            moveInput = dc.features.moveFeatures.createInput2(moving)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, _vec3(axisDir), apexWorld)
            moveInput.defineAsFreeMove(matrix)
            dc.features.moveFeatures.add(moveInput)

        # H. crown (the one place a component is activated)
        self.designOccurrence.activate()
        try:
            order = self._orderByHeelFace(segments, apex3, coneVec)
            for position in range(len(order) - 1):
                (R_hf, idx) = order[position]
                seg = adsk.fusion.BRepBody.cast(segments[idx])
                u = (R_heel - R_hf) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        f'{gearLabel}: crown factor {factor} is not positive for segment u={u}')
                if factor == 1.0:
                    continue
                heelFace = self._heelFace(seg, apex3, coneVec)
                ranked = []
                for k in range(heelFace.vertices.count):
                    p = _t3(heelFace.vertices.item(k).geometry)
                    rel = _sub3(p, apex3)
                    along = _dot3(rel, axisDir)
                    radial = _sub3(rel, _mul3(axisDir, along))
                    ranked.append((math.sqrt(_dot3(radial, radial)), p))
                if len(ranked) < 2:
                    raise Exception(f'{gearLabel}: heel face has fewer than two vertices')
                ranked.sort(key=lambda item: item[0])
                p0 = ranked[0][1]
                p1 = ranked[1][1]
                base = _mul3(_add3(p0, p1), 0.5)
                heelSketch = dc.sketches.add(heelFace)
                basePoint = heelSketch.sketchPoints.add(heelSketch.modelToSketchSpace(_pt3(base)))
                scaling = adsk.core.ObjectCollection.create()
                scaling.add(seg)
                scaleInput = dc.features.scaleFeatures.createInput(
                    scaling, basePoint, adsk.core.ValueInput.createByReal(factor))
                dc.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # I. loft the curved tooth through the recomputed order
        order = self._orderByHeelFace(segments, apex3, coneVec)
        firstSeg = adsk.fusion.BRepBody.cast(segments[order[0][1]])
        loftInput = dc.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._toeFace(firstSeg, apex3, coneVec))
        for (R_hf, idx) in order:
            seg = adsk.fusion.BRepBody.cast(segments[idx])
            loftInput.loftSections.add(self._heelFace(seg, apex3, coneVec))
        spiralLoft = dc.features.loftFeatures.add(loftInput)
        spiralTooth = adsk.fusion.BRepBody.cast(spiralLoft.bodies.item(0))
        spiralTooth.name = f'{gearLabel} Spiral Tooth'
        for i in range(len(segments)):
            seg = adsk.fusion.BRepBody.cast(segments[i])
            if seg.isValid:
                dc.features.removeFeatures.add(seg)

        # J. conical end cuts
        return cut_conical_ends(dc, spiralTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
