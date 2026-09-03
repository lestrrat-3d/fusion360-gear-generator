"""Bevel gear generator, emitted from the compiled step list spec/bevelgear/steps.md.

The geometry is transliterated from proof/bevelgear/ (sketches_test.go, tooth_test.go,
frustum_test.go, solids_test.go, spiral_test.go) for the steps the step list tags [GO]; nothing
here is re-derived. Bevel registers no Fusion user parameters: every value is precomputed in
Python (internal centimetres for length, radians for angle) and written into geometry numerically
([PB-PRECOMPUTED-MODE]), so bevel does not subclass base.Generator.

Units: every length below is centimetres, Fusion's internal length unit, from the moment it is
read (the Module dialog input is the one exception -- it is read unitless and means millimetres,
so it is converted to centimetres immediately in _readInputs). Angles are radians unless a
variable name says otherwise (e.g. shaftAngle_deg).
"""

import math

import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, to_mm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts, get_normal
from . import solids
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy

# Dialog input ids (S1), in display order.
INPUT_ID_TARGET_PLANE = 'targetPlane'
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
INPUT_ID_SPIRAL_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'


# --- plain 2-D math for the Gear Profiles lattice (sketch-local, x/y tuples) ----------------

def _dir(theta):
    """Unit vector at angle theta measured from the driving shaft direction (0, -1), turning
    toward the pinion side -- the one angular measure the whole §2 figure is written in."""
    return (math.sin(theta), -math.cos(theta))


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _vscale(a, s):
    return (a[0] * s, a[1] * s)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _vnorm(a):
    return math.hypot(a[0], a[1])


def _vnormalize(a):
    n = _vnorm(a)
    return (a[0] / n, a[1] / n)


def _rotate2(v, theta):
    ca, sa = math.cos(theta), math.sin(theta)
    return (v[0] * ca - v[1] * sa, v[0] * sa + v[1] * ca)


def _dist_point_line(p, a, b):
    """Perpendicular distance from p to the infinite line through a and b."""
    d = _vsub(b, a)
    n = _vnorm(d)
    return abs(d[0] * (p[1] - a[1]) - d[1] * (p[0] - a[0])) / n


def _slide_to_drop(fromAbs, along, axisPointAbs, axisDir):
    """Slide from `fromAbs` along `along` until it meets the perpendicular drop through
    axisPointAbs -- where a toe edge's far end is pinned."""
    travel = _vdot(_vsub(axisPointAbs, fromAbs), along) / _vdot(along, axisDir)
    return _vadd(fromAbs, _vscale(along, travel))


# --- plain 3-D math for the spiral branch (world Point3D / Vector3D, read-only) -------------

def _v3(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def _sub3(p, q):
    """p - q, for any two objects exposing .x/.y/.z (Point3D or Vector3D)."""
    return _v3(p.x - q.x, p.y - q.y, p.z - q.z)


def _dot3(u, v):
    return u.x * v.x + u.y * v.y + u.z * v.z


def _cross3(u, v):
    return _v3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x)


def _length3(v):
    return math.sqrt(_dot3(v, v))


def normalize(v):
    n = _length3(v)
    return _v3(v.x / n, v.y / n, v.z / n)


def _midpoint3(p, q):
    return adsk.core.Point3D.create((p.x + q.x) / 2.0, (p.y + q.y) / 2.0, (p.z + q.z) / 2.0)


def _distance3(p: adsk.core.Point3D, q: adsk.core.Point3D):
    return p.distanceTo(q)


def _perp_dist_to_axis(point, axisOrigin, axisDir):
    """Perpendicular distance from `point` to the infinite line through axisOrigin along the
    unit vector axisDir."""
    rel = _sub3(point, axisOrigin)
    along = _dot3(rel, axisDir)
    perp = _v3(rel.x - along * axisDir.x, rel.y - along * axisDir.y, rel.z - along * axisDir.z)
    return _length3(perp)


class BevelGearCommandInputsConfigurator:
    """S1: the seventeen dialog inputs, in the order Target Plane, Center Point, Parent
    Component, then the numeric and derived inputs. `configure` and `handle_input_changed` are
    bound by name from commands/bevelgear/entry.py, so this module defines them for the framework
    to call rather than calling them itself."""

    @classmethod
    def configure(cls, cmd: adsk.core.Command):
        inputs = cmd.commandInputs

        # 1. Target Plane -- first, so Fusion's auto-focus lands on it ([PB-AUTOFOCUS-FIRST]).
        planeInput = inputs.addSelectionInput(
            INPUT_ID_TARGET_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point.
        pointInput = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1, 1)

        # 3. Parent Component -- pre-selected on the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)

        # 4. Module: unitless, so the unit string is empty ([PB-DIALOG-DEFAULT-UNITS]).
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 5. Shaft Angle.
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            adsk.core.ValueInput.createByString('90 deg'))

        # 6 and 7. Tooth counts.
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 8 and 9. Base heights, defaulted to 0 (unspecified -> resolved from the fallback).
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 10. Enable Bore, checked by default.
        inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11 and 12. Bore diameters, 0 means auto (Pitch Diameter / 4).
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 13 and 14. Face Width and Tooth Spacing, 0 means auto / none.
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 15. Mean Spiral Angle. Above zero, it shows the two spiral-only inputs below.
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_SPIRAL_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius, 0 means auto (the mean cone distance).
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args: adsk.core.InputChangedEventArgs):
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs: adsk.core.CommandInputs):
        """Hand of Spiral and Cutter Radius are shown only when the Mean Spiral Angle is above
        zero. Leaves both shown when the spiral input's expression cannot be evaluated."""
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_SPIRAL_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            design: adsk.fusion.Design = get_design()
            psi = design.unitsManager.evaluateExpression(spiralInput.expression, 'rad')
        except Exception:
            handInput.isVisible = True
            cutterInput.isVisible = True
            return
        isVisible = psi > 0
        handInput.isVisible = isVisible
        cutterInput.isVisible = isVisible


class BevelGearGenerator:
    """S2 onward: reading and validating the dialog, and building the pair. Bevel does not
    subclass base.Generator ([PB-PRECOMPUTED-MODE]), so none of getOccurrence, addParameter or
    createSketchObject is used; occurrences are created directly with
    parent.occurrences.addNewComponent(...) ([PB-OCCURRENCE-TREE])."""

    # Tunable class constants the step list names by name.
    _CROWN_PER_RAD = 0.5
    _PINION_MESH_PHASE_TEETH = 0

    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

    def deleteComponent(self):
        if self.bevelOccurrence is not None:
            bevelOccurrence: adsk.fusion.Occurrence = self.bevelOccurrence
            bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    def _pinionMeshPhase(self, pinionTeeth):
        return self._PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth

    # --- S2: read, validate, resolve ---------------------------------------------------------

    def _resolveBaseHeight(self, label, given, fallback, pitchRadius, gamma, module):
        low = 1.05 * 1.25 * module * math.sin(gamma)
        high = 0.95 * (pitchRadius - 1.25 * module * math.cos(gamma)) * math.tan(gamma)
        if given != 0:
            if given < low or given > high:
                raise Exception(
                    '{}: {:.6f} mm is outside the valid range [{:.6f}, {:.6f}] mm'.format(
                        label, to_mm(given), to_mm(low), to_mm(high)))
            return given
        return min(max(fallback, low), high)

    def _readInputs(self, inputs: adsk.core.CommandInputs):
        design: adsk.fusion.Design = self.design
        um: adsk.core.UnitsManager = design.unitsManager

        def evalExpr(inputId, units, inputs: adsk.core.CommandInputs = inputs,
                     um: adsk.core.UnitsManager = um):
            return um.evaluateExpression(inputs.itemById(inputId).expression, units)

        parentEntity = get_selection(inputs, INPUT_ID_PARENT)[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentEntity.component
        else:
            parentComponent = parentEntity

        targetPlane = get_selection(inputs, INPUT_ID_TARGET_PLANE)[0]
        centerPoint = get_selection(inputs, INPUT_ID_CENTER_POINT)[0]

        # Module is read unitless and means millimetres; every length derived from it has to be
        # converted to centimetres before it touches geometry. Converting it here, once, lets
        # every other length in this generator stay in centimetres from the moment it is read.
        moduleRaw = evalExpr(INPUT_ID_MODULE, '')
        if moduleRaw <= 0:
            raise Exception('Module must be above zero, got {}'.format(moduleRaw))
        module = to_cm(moduleRaw)

        drivingTeeth = int(round(evalExpr(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalExpr(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception(
                'Both tooth counts must be at least three; got driving={}, pinion={}'.format(
                    drivingTeeth, pinionTeeth))

        # The mm and deg inputs already come back in internal units (cm, rad) and must not be
        # converted again ([PB-EVAL-EXPRESSION]).
        drivingBaseHeightIn = evalExpr(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        pinionBaseHeightIn = evalExpr(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        drivingBoreIn = evalExpr(INPUT_ID_DRIVING_BORE, 'mm')
        pinionBoreIn = evalExpr(INPUT_ID_PINION_BORE, 'mm')
        faceWidthIn = evalExpr(INPUT_ID_FACE_WIDTH, 'mm')
        toothSpacing = evalExpr(INPUT_ID_TOOTH_SPACING, 'mm')
        cutterRadiusIn = evalExpr(INPUT_ID_CUTTER_RADIUS, 'mm')
        for (label, value) in (
                ('Driving Gear Base Height', drivingBaseHeightIn),
                ('Pinion Gear Base Height', pinionBaseHeightIn),
                ('Driving Gear Bore Diameter', drivingBoreIn),
                ('Pinion Gear Bore Diameter', pinionBoreIn),
                ('Face Width', faceWidthIn),
                ('Tooth Spacing', toothSpacing),
                ('Cutter Radius', cutterRadiusIn)):
            if value < 0:
                raise Exception('{} must be non-negative, got {:.6f} mm'.format(
                    label, to_mm(value)))

        spiralAngle_rad = evalExpr(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception(
                'Mean Spiral Angle must be in [0, 60) degrees, got {:.6f}'.format(spiralAngle_deg))

        shaftAngle_rad = evalExpr(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)

        drivingPitchDia = module * drivingTeeth
        pinionPitchDia = module * pinionTeeth
        coneDistance = math.hypot(drivingPitchDia, pinionPitchDia)

        smaller = min(drivingPitchDia, pinionPitchDia)
        larger = max(drivingPitchDia, pinionPitchDia)
        maxShaftAngle_deg = min(math.degrees(math.acos(-smaller / larger)), 150.0)
        if shaftAngle_deg < 30.0:
            raise Exception(
                'Shaft Angle must be at least 30 degrees, got {:.6f}'.format(shaftAngle_deg))
        if shaftAngle_deg >= maxShaftAngle_deg:
            raise Exception(
                'Shaft Angle must be below the Maximum Shaft Angle {:.6f} degrees, got {:.6f}'
                .format(maxShaftAngle_deg, shaftAngle_deg))

        gammaP = math.atan2(
            math.sin(shaftAngle_rad) * pinionPitchDia,
            drivingPitchDia + pinionPitchDia * math.cos(shaftAngle_rad))
        gammaG = shaftAngle_rad - gammaP
        R = (pinionPitchDia / 2.0) / math.sin(gammaP)

        minTeethPinion = 5.27 * math.cos(gammaP)
        minTeethDriving = 5.27 * math.cos(gammaG)
        if pinionTeeth < minTeethPinion:
            raise Exception(
                "Pinion Gear Teeth must be at least {:.6f} (this gear's Minimum Teeth floor), "
                'got {}'.format(minTeethPinion, pinionTeeth))
        if drivingTeeth < minTeethDriving:
            raise Exception(
                "Driving Gear Teeth must be at least {:.6f} (this gear's Minimum Teeth floor), "
                'got {}'.format(minTeethDriving, drivingTeeth))

        drivingHeightFallback = module * drivingTeeth / 8.0
        drivingHeight = self._resolveBaseHeight(
            'Driving Gear Base Height', drivingBaseHeightIn, drivingHeightFallback,
            drivingPitchDia / 2.0, gammaG, module)
        pinionHeightFallback = drivingHeight * pinionTeeth / drivingTeeth
        pinionHeight = self._resolveBaseHeight(
            'Pinion Gear Base Height', pinionBaseHeightIn, pinionHeightFallback,
            pinionPitchDia / 2.0, gammaP, module)

        self._module = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._drivingPitchDia = drivingPitchDia
        self._pinionPitchDia = pinionPitchDia
        self._coneDistance = coneDistance
        self._gamma_p = gammaP
        self._gamma_g = gammaG
        self._R = R
        self._shaftAngle_rad = shaftAngle_rad
        self._drivingBaseHeight = drivingHeight
        self._pinionBaseHeight = pinionHeight
        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBoreInput = drivingBoreIn
        self._pinionBoreInput = pinionBoreIn
        self._faceWidthInput = faceWidthIn
        self._faceWidth = None  # resolved against the solved lattice in S6
        self._toothSpacing = toothSpacing
        self._spiralAngle_rad = spiralAngle_rad
        handItem = inputs.itemById(INPUT_ID_SPIRAL_HAND).selectedItem
        handName = handItem.name if handItem is not None else _HAND_RIGHT
        self._handSign = 1.0 if handName == _HAND_RIGHT else -1.0
        self._cutterRadiusInput = cutterRadiusIn

        return (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
                shaftAngle_deg)

    # --- S3: occurrence tree, S4: Anchor Sketch, S5: Gear Profiles Plane ---------------------

    def _buildAnchorSketch(self, designComponent: adsk.fusion.Component, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)

        # Seed the two endpoints at exactly +/- 0.5 cm from the PROJECTED centre's own local
        # position along sketch-local X -- not from the sketch's raw (0, 0), which the projected
        # centre need not sit at.
        c = projectedCenter.geometry
        half = 0.5
        p0 = adsk.core.Point3D.create(c.x - half, c.y, c.z)
        p1 = adsk.core.Point3D.create(c.x + half, c.y, c.z)
        anchorLine = sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
        anchorLine.isConstruction = True

        sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)
        sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)
        textPoint = adsk.core.Point3D.create(c.x, c.y + 0.2, c.z)
        sketch.sketchDimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)
        sketch.geometricConstraints.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('{} did not reach full constraint'.format(sketch.name))

        return anchorLine, projectedCenter

    def _buildGearProfilesPlane(self, designComponent: adsk.fusion.Component, targetPlane,
                                anchorLine):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        plane = designComponent.constructionPlanes.add(planeInput)
        plane.name = 'Gear Profiles Plane'
        return plane

    # --- S6: the Gear Profiles sketch, the whole §2 lattice ----------------------------------

    def _buildGearProfilesSketch(self, designComponent: adsk.fusion.Component, gearProfilesPlane,
                                  targetPlane, anchorSketchCenter, anchorLine):
        sketch: adsk.fusion.Sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'

        projectedCenter = sketch.project(anchorSketchCenter).item(0)
        projectedAnchorLine = sketch.project(anchorLine).item(0)

        # c: the projected centre's own local position. d: the projected anchor line's unit
        # direction. perp: the in-plane perpendicular, signed toward the target plane's normal --
        # the single permitted world use, per [BEVEL-F-GROW-SIDE] / [BEVEL-F-APEX-LOCAL].
        c0 = (projectedCenter.geometry.x, projectedCenter.geometry.y)
        startGeom = projectedAnchorLine.startSketchPoint.geometry
        endGeom = projectedAnchorLine.endSketchPoint.geometry
        d0 = _vnormalize((endGeom.x - startGeom.x, endGeom.y - startGeom.y))
        perp0 = (-d0[1], d0[0])

        targetNormal = get_normal(targetPlane)
        worldC = sketch.sketchToModelSpace(adsk.core.Point3D.create(c0[0], c0[1], 0))
        worldCPerp = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c0[0] + perp0[0], c0[1] + perp0[1], 0))
        worldPerp = (worldCPerp.x - worldC.x, worldCPerp.y - worldC.y, worldCPerp.z - worldC.z)
        towardNormal = (worldPerp[0] * targetNormal.x + worldPerp[1] * targetNormal.y
                        + worldPerp[2] * targetNormal.z)
        if towardNormal < 0:
            perp0 = (-perp0[0], -perp0[1])

        def toXY(v):
            return (c0[0] + v[0] * d0[0] + v[1] * perp0[0],
                    c0[1] + v[0] * d0[1] + v[1] * perp0[1])

        def toPoint3D(v):
            xy = toXY(v)
            return adsk.core.Point3D.create(xy[0], xy[1], 0)

        def newLine(fromAbs, toAbs, sketch: adsk.fusion.Sketch = sketch):
            l = sketch.sketchCurves.sketchLines.addByTwoPoints(toPoint3D(fromAbs), toPoint3D(toAbs))
            l.isConstruction = True
            return l

        def pin(sketchPoint, target, sketch: adsk.fusion.Sketch = sketch):
            sketch.geometricConstraints.addCoincident(sketchPoint, target)

        def alignedDim(p0pt, p1pt, textAbs, sketch: adsk.fusion.Sketch = sketch):
            return sketch.sketchDimensions.addDistanceDimension(
                p0pt, p1pt, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                toPoint3D(textAbs))

        module = self._module
        dedendum = 1.25 * module
        R = self._R
        gammaP = self._gamma_p
        gammaG = self._gamma_g
        Sigma = self._shaftAngle_rad
        drivingHeight = self._drivingBaseHeight
        pinionHeight = self._pinionBaseHeight
        toothSpacing = self._toothSpacing

        drivingDir = _dir(0.0)
        pinionDir = _dir(Sigma)
        pitchDir = _dir(gammaG)
        towardAnchor = _dir(gammaG - math.pi / 2.0)
        awayFromAnchor = _dir(gammaG + math.pi / 2.0)

        # The closed-form lattice, in the (d0, perp0) frame -- also the exact solved positions,
        # so these seeds are the truth the constraint net is built to reproduce, never just an
        # approximation ([PB-SEED-NEAR]).
        apexAbs = (0.0, R * math.cos(gammaG) + drivingHeight)
        bAbs = _vadd(apexAbs, _vscale(drivingDir, R * math.cos(gammaG)))
        aAbs = _vadd(apexAbs, _vscale(pinionDir, R * math.cos(gammaP)))
        apex2Abs = _vadd(apexAbs, _vscale(pitchDir, R))
        dAbs = _vadd(apex2Abs, _vscale(towardAnchor, dedendum))
        cAbs = _vadd(apex2Abs, _vscale(awayFromAnchor, dedendum))
        eAbs = _vadd(aAbs, _vscale(pinionDir, dedendum * math.sin(gammaP)))
        fAbs = _vadd(bAbs, _vscale(drivingDir, dedendum * math.sin(gammaG)))
        gAbs = _vadd(apexAbs, _vscale(pinionDir, R * math.cos(gammaP) + pinionHeight))
        iAbs = _vadd(apexAbs, _vscale(drivingDir, R * math.cos(gammaG) + drivingHeight))
        hAbs = _vadd(cAbs, _vscale(awayFromAnchor, pinionHeight / math.sin(gammaP) - dedendum))
        jAbs = _vadd(dAbs, _vscale(towardAnchor, drivingHeight / math.sin(gammaG) - dedendum))
        kAbs = _vadd(apexAbs, _vscale(pinionDir, R / math.cos(gammaP)))
        lAbs = _vadd(apexAbs, _vscale(drivingDir, R / math.cos(gammaG)))
        kpAbs = _vadd(kAbs, _vscale(awayFromAnchor, toothSpacing))
        lpAbs = _vadd(lAbs, _vscale(towardAnchor, toothSpacing))

        # 1. Centre to Apex: perpendicular to the projected anchor line, undimensioned.
        centerToApex = newLine((0.0, 0.0), apexAbs)
        pin(centerToApex.startSketchPoint, projectedCenter)
        sketch.geometricConstraints.addPerpendicular(projectedAnchorLine, centerToApex)
        apexPoint = centerToApex.endSketchPoint

        # 2. Driving Gear Shaft Axis: parallel to the centre-to-apex line, undimensioned.
        drivingAxis = newLine(apexAbs, bAbs)
        pin(drivingAxis.startSketchPoint, apexPoint)
        sketch.geometricConstraints.addParallel(drivingAxis, centerToApex)
        bPoint = drivingAxis.endSketchPoint

        # 3. Pinion Gear Shaft Axis at the Shaft Angle. Form both candidate ends and keep the one
        # with the greater X in THIS sketch ([PB-ANGULAR-DIM]).
        drivingDirLocal = _vnormalize(_vsub(bAbs, apexAbs))
        candA = _vadd(apexAbs, _vscale(_rotate2(drivingDirLocal, Sigma), R * math.cos(gammaP)))
        candB = _vadd(apexAbs, _vscale(_rotate2(drivingDirLocal, -Sigma), R * math.cos(gammaP)))
        pinionEndAbs = candA if toXY(candA)[0] > toXY(candB)[0] else candB
        pinionAxis = newLine(apexAbs, pinionEndAbs)
        pin(pinionAxis.startSketchPoint, apexPoint)
        pinionDirLocal = _vnormalize(_vsub(pinionEndAbs, apexAbs))
        bisector = _vnormalize(_vadd(pinionDirLocal, drivingDirLocal))
        angleTextAbs = _vadd(apexAbs, _vscale(bisector, self._pinionPitchDia / 4.0))
        angDim = sketch.sketchDimensions.addAngularDimension(
            pinionAxis, drivingAxis, toPoint3D(angleTextAbs))
        angDim.parameter.value = Sigma
        aPoint = pinionAxis.endSketchPoint

        # 4. The two perpendicular drops to Apex 2.
        dropA = newLine(aAbs, apex2Abs)
        pin(dropA.startSketchPoint, aPoint)
        sketch.geometricConstraints.addPerpendicular(pinionAxis, dropA)
        dimA = alignedDim(dropA.startSketchPoint, dropA.endSketchPoint,
                          _vadd(aAbs, _vscale(_vnormalize(_vsub(apex2Abs, aAbs)), 0.3)))
        dimA.parameter.value = self._pinionPitchDia / 2.0

        dropB = newLine(bAbs, apex2Abs)
        pin(dropB.startSketchPoint, bPoint)
        sketch.geometricConstraints.addPerpendicular(drivingAxis, dropB)
        dimB = alignedDim(dropB.startSketchPoint, dropB.endSketchPoint,
                          _vadd(bAbs, _vscale(_vnormalize(_vsub(apex2Abs, bAbs)), 0.3)))
        dimB.parameter.value = self._drivingPitchDia / 2.0

        pin(dropA.endSketchPoint, dropB.endSketchPoint)
        apex2Point = dropA.endSketchPoint

        # 5. Pitch Line, coincident at both ends.
        pitchLine = newLine(apexAbs, apex2Abs)
        pin(pitchLine.startSketchPoint, apexPoint)
        pin(pitchLine.endSketchPoint, apex2Point)

        # 6. The two dedendum lines.
        dedD = newLine(apex2Abs, dAbs)
        pin(dedD.startSketchPoint, apex2Point)
        sketch.geometricConstraints.addPerpendicular(pitchLine, dedD)
        dimD = alignedDim(dedD.startSketchPoint, dedD.endSketchPoint, dAbs)
        dimD.parameter.value = dedendum
        dPoint = dedD.endSketchPoint

        dedC = newLine(apex2Abs, cAbs)
        pin(dedC.startSketchPoint, apex2Point)
        sketch.geometricConstraints.addPerpendicular(pitchLine, dedC)
        dimC = alignedDim(dedC.startSketchPoint, dedC.endSketchPoint, cAbs)
        dimC.parameter.value = dedendum
        cPoint = dedC.endSketchPoint

        # 7. The two Root Axes.
        rootC = newLine(apexAbs, cAbs)
        pin(rootC.startSketchPoint, apexPoint)
        pin(rootC.endSketchPoint, cPoint)
        rootD = newLine(apexAbs, dAbs)
        pin(rootD.startSketchPoint, apexPoint)
        pin(rootD.endSketchPoint, dPoint)

        # 8. The module-length extensions, undimensioned ([BEVEL-F-DRIVEN-DIMS]).
        ae = newLine(aAbs, eAbs)
        pin(ae.startSketchPoint, aPoint)
        sketch.geometricConstraints.addCollinear(ae, pinionAxis)
        ePoint = ae.endSketchPoint
        ce = newLine(cAbs, eAbs)
        pin(ce.startSketchPoint, cPoint)
        pin(ce.endSketchPoint, ePoint)
        sketch.geometricConstraints.addPerpendicular(ae, ce)
        eg = newLine(eAbs, gAbs)
        pin(eg.startSketchPoint, ePoint)
        sketch.geometricConstraints.addCollinear(eg, pinionAxis)
        gPoint = eg.endSketchPoint

        bf = newLine(bAbs, fAbs)
        pin(bf.startSketchPoint, bPoint)
        sketch.geometricConstraints.addCollinear(bf, drivingAxis)
        fPoint = bf.endSketchPoint
        df = newLine(dAbs, fAbs)
        pin(df.startSketchPoint, dPoint)
        pin(df.endSketchPoint, fPoint)
        sketch.geometricConstraints.addPerpendicular(bf, df)
        fi = newLine(fAbs, iAbs)
        pin(fi.startSketchPoint, fPoint)
        sketch.geometricConstraints.addCollinear(fi, drivingAxis)
        iPoint = fi.endSketchPoint

        # 9. The two heel edges.
        ch = newLine(cAbs, hAbs)
        pin(ch.startSketchPoint, cPoint)
        sketch.geometricConstraints.addCollinear(ch, dedC)
        hPoint = ch.endSketchPoint
        gh = newLine(gAbs, hAbs)
        pin(gh.startSketchPoint, gPoint)
        pin(gh.endSketchPoint, hPoint)
        sketch.geometricConstraints.addPerpendicular(eg, gh)

        dj = newLine(dAbs, jAbs)
        pin(dj.startSketchPoint, dPoint)
        sketch.geometricConstraints.addCollinear(dj, dedD)
        jPoint = dj.endSketchPoint
        ij = newLine(iAbs, jAbs)
        pin(ij.startSketchPoint, iPoint)
        pin(ij.endSketchPoint, jPoint)
        sketch.geometricConstraints.addPerpendicular(fi, ij)

        # 10. The two base-height offsets. Both lines are already parallel by construction, so no
        # addParallel ([PB-OFFSET-DIM]).
        offsetB = sketch.sketchDimensions.addOffsetDimension(dropB, ij, toPoint3D(jAbs))
        offsetB.parameter.value = drivingHeight
        offsetA = sketch.sketchDimensions.addOffsetDimension(dropA, gh, toPoint3D(hAbs))
        offsetA.parameter.value = pinionHeight

        # 11. Close the figure: A->G, B->I, and Point I on the projected centre.
        ag = newLine(aAbs, gAbs)
        pin(ag.startSketchPoint, aPoint)
        pin(ag.endSketchPoint, gPoint)
        bi = newLine(bAbs, iAbs)
        pin(bi.startSketchPoint, bPoint)
        pin(bi.endSketchPoint, iPoint)
        sketch.geometricConstraints.addCoincident(iPoint, projectedCenter)

        # 12. The tooth centres K and L, pinned by two point-on-line coincidents each.
        gk = newLine(gAbs, kAbs)
        pin(gk.startSketchPoint, gPoint)
        sketch.geometricConstraints.addCoincident(gk.endSketchPoint, pinionAxis)
        sketch.geometricConstraints.addCoincident(gk.endSketchPoint, dedC)
        kPoint = gk.endSketchPoint
        ck = newLine(cAbs, kAbs)
        pin(ck.startSketchPoint, cPoint)
        pin(ck.endSketchPoint, kPoint)

        il = newLine(iAbs, lAbs)
        pin(il.startSketchPoint, iPoint)
        sketch.geometricConstraints.addCoincident(il.endSketchPoint, drivingAxis)
        sketch.geometricConstraints.addCoincident(il.endSketchPoint, dedD)
        lPoint = il.endSketchPoint
        dl = newLine(dAbs, lAbs)
        pin(dl.startSketchPoint, dPoint)
        pin(dl.endSketchPoint, lPoint)

        # 13. The Tooth Spacing offsets. At zero spacing K' is K and the C->K line built above is
        # the reference line; a positive spacing draws K' and a fresh C->K' line.
        if toothSpacing == 0:
            kpPoint, pinionRefLine = kPoint, ck
            lpPoint, drivingRefLine = lPoint, dl
        else:
            kkp = newLine(kAbs, kpAbs)
            pin(kkp.startSketchPoint, kPoint)
            sketch.geometricConstraints.addCoincident(kkp.endSketchPoint, dedC)
            dimKp = alignedDim(kkp.startSketchPoint, kkp.endSketchPoint, kpAbs)
            dimKp.parameter.value = toothSpacing
            kpPoint = kkp.endSketchPoint
            ckp = newLine(cAbs, kpAbs)
            pin(ckp.startSketchPoint, cPoint)
            pin(ckp.endSketchPoint, kpPoint)
            pinionRefLine = ckp

            llp = newLine(lAbs, lpAbs)
            pin(llp.startSketchPoint, lPoint)
            sketch.geometricConstraints.addCoincident(llp.endSketchPoint, dedD)
            dimLp = alignedDim(llp.startSketchPoint, llp.endSketchPoint, lpAbs)
            dimLp.parameter.value = toothSpacing
            lpPoint = llp.endSketchPoint
            dlp = newLine(dAbs, lpAbs)
            pin(dlp.startSketchPoint, dPoint)
            pin(dlp.endSketchPoint, lpPoint)
            drivingRefLine = dlp

        # 14. The Maximum Face Width, read off SOLVED geometry ([PB-SOLVED-GEOMETRY]) -- A, B, C,
        # D, H and J are fully determined by the constraints added so far.
        aXY = (aPoint.geometry.x, aPoint.geometry.y)
        cXY = (cPoint.geometry.x, cPoint.geometry.y)
        hXY = (hPoint.geometry.x, hPoint.geometry.y)
        bXY = (bPoint.geometry.x, bPoint.geometry.y)
        dXY = (dPoint.geometry.x, dPoint.geometry.y)
        jXY = (jPoint.geometry.x, jPoint.geometry.y)
        pinionReach = _dist_point_line(aXY, cXY, hXY)
        drivingReach = _dist_point_line(bXY, dXY, jXY)
        maxFaceWidth = 0.95 * min(pinionReach, drivingReach)

        faceWidthInput = self._faceWidthInput
        if faceWidthInput != 0:
            if faceWidthInput > maxFaceWidth:
                raise Exception(
                    'Face Width {:.6f} mm exceeds the Maximum Face Width {:.6f} mm'.format(
                        to_mm(faceWidthInput), to_mm(maxFaceWidth)))
            faceWidth = faceWidthInput
        else:
            faceWidth = min(self._coneDistance / 6.0, maxFaceWidth)
        self._faceWidth = faceWidth

        # 15. The two toe edges, offset from their heel edges by the resolved Face Width.
        mAbs = _vadd(apexAbs, _vscale(_vsub(cAbs, apexAbs), 1.0 - faceWidth / R))
        oAbs = _vadd(apexAbs, _vscale(_vsub(dAbs, apexAbs), 1.0 - faceWidth / R))
        nAbs = _slide_to_drop(mAbs, awayFromAnchor, aAbs, pinionDir)
        pAbs = _slide_to_drop(oAbs, towardAnchor, bAbs, drivingDir)

        mn = newLine(mAbs, nAbs)
        sketch.geometricConstraints.addCoincident(mn.startSketchPoint, rootC)
        sketch.geometricConstraints.addCoincident(mn.endSketchPoint, dropA)
        sketch.geometricConstraints.addParallel(mn, ch)
        offsetMN = sketch.sketchDimensions.addOffsetDimension(
            ch, mn, toPoint3D(_vadd(mAbs, (0.1, 0.1))))
        offsetMN.parameter.value = faceWidth
        mPoint, nPoint = mn.startSketchPoint, mn.endSketchPoint
        mc = newLine(mAbs, cAbs)
        pin(mc.startSketchPoint, mPoint)
        pin(mc.endSketchPoint, cPoint)
        na = newLine(nAbs, aAbs)
        pin(na.startSketchPoint, nPoint)
        pin(na.endSketchPoint, aPoint)

        op = newLine(oAbs, pAbs)
        sketch.geometricConstraints.addCoincident(op.startSketchPoint, rootD)
        sketch.geometricConstraints.addCoincident(op.endSketchPoint, dropB)
        sketch.geometricConstraints.addParallel(op, dj)
        offsetOP = sketch.sketchDimensions.addOffsetDimension(
            dj, op, toPoint3D(_vadd(oAbs, (0.1, 0.1))))
        offsetOP.parameter.value = faceWidth
        oPoint, pPoint = op.startSketchPoint, op.endSketchPoint
        od = newLine(oAbs, dAbs)
        pin(od.startSketchPoint, oPoint)
        pin(od.endSketchPoint, dPoint)
        pb = newLine(pAbs, bAbs)
        pin(pb.startSketchPoint, pPoint)
        pin(pb.endSketchPoint, bPoint)

        if not sketch.isFullyConstrained:
            raise Exception('{} did not reach full constraint'.format(sketch.name))

        pinionInfo = {
            'hex': [aPoint, gPoint, hPoint, cPoint, mPoint, nPoint],
            'toothCenterPoint': kpPoint,
            'toothCenterLine': pinionRefLine,
            'teeth': self._pinionTeeth,
            'pitchDia': self._pinionPitchDia,
            'gamma': gammaP,
            'boreInput': self._pinionBoreInput,
            'handSign': -self._handSign,
        }
        drivingInfo = {
            'hex': [bPoint, iPoint, jPoint, dPoint, oPoint, pPoint],
            'toothCenterPoint': lpPoint,
            'toothCenterLine': drivingRefLine,
            'teeth': self._drivingTeeth,
            'pitchDia': self._drivingPitchDia,
            'gamma': gammaG,
            'boreInput': self._drivingBoreInput,
            'handSign': self._handSign,
        }
        return sketch, apexPoint, pinionInfo, drivingInfo

    # --- S7-S9: tooth plane, tooth sketch, tooth axis ----------------------------------------

    def _buildToothPlane(self, designComponent, gearProfilesPlane, toothCenterLine, label):
        toothPlane = solids.plane_by_angle(designComponent, toothCenterLine, gearProfilesPlane, 90)
        toothPlane.name = '{} Plane'.format(label)
        return toothPlane

    def _buildToothSketch(self, designComponent: adsk.fusion.Component, toothPlane, toothCenterPoint, pitchDia, gamma,
                          label):
        virtualPitchRadius_mm = (to_mm(pitchDia) / 2.0) / math.cos(gamma)
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / to_mm(self._module)))

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = '{} Tooth'.format(label)
        proxy = VirtualSpurProxy(module_mm=to_mm(self._module), virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCenterPoint, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded

        # Exempt from the full-constraint gate ([BEVEL-F-FULL-CONSTRAINT]); logged only.
        if not toothSketch.isFullyConstrained:
            futil.log(
                '{}: not fully constrained aside from the along-path label exemption'.format(
                    toothSketch.name), force_console=True)

        return toothSketch, embedded

    def _buildToothAxis(self, designComponent: adsk.fusion.Component, gearProfilesPlane,
                        toothCenterLine, label):
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(toothCenterLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = '{} Tooth Axis'.format(label)
        return toothAxis

    # --- S11: the frustum hexagon Profile sketch, and S12: its revolve -----------------------

    def _buildProfileSketch(self, designComponent: adsk.fusion.Component, gearProfilesPlane, label,
                            hexSourcePoints):
        sketch: adsk.fusion.Sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = '{} Profile'.format(label)

        # [PB-PROJECT-NOT-FIXED]'s recreate-share-fix recipe: recreate, then draw sharing the
        # recreated points, then fix the lines' endpoints only after the lines exist.
        pts = [sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))
               for src in hexSourcePoints]
        n = len(pts)
        lines = [sketch.sketchCurves.sketchLines.addByTwoPoints(pts[idx], pts[(idx + 1) % n])
                 for idx in range(n)]
        for l in lines:
            l.startSketchPoint.isFixed = True
            l.endSketchPoint.isFixed = True

        if not sketch.isFullyConstrained:
            raise Exception('{} did not reach full constraint'.format(sketch.name))

        return sketch, pts, lines

    def _revolveGearBody(self, designComponent: adsk.fusion.Component,
                         profileSketch: adsk.fusion.Sketch, shaftAxisLine, label):
        profile = profileSketch.profiles.item(0)  # [PB-SINGLE-PROFILE]
        revInput = designComponent.features.revolveFeatures.createInput(
            profile, shaftAxisLine, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        gearBody = designComponent.features.revolveFeatures.add(revInput).bodies.item(0)
        gearBody.name = '{} Gear Body'.format(label)
        return gearBody

    # --- S13: loft the Apex to the tooth profile ---------------------------------------------

    def _loftToothBody(self, designComponent: adsk.fusion.Component, apexPoint, toothSketch,
                       embedded, label):
        wantLines = 0 if embedded else 2
        toothProfile = find_profile_by_curve_counts(
            toothSketch, nurbs=2, arcs=2, lines=wantLines)
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexPoint)
        loftInput.loftSections.add(toothProfile)
        toothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
        toothBody.name = '{} Tooth Body'.format(label)
        return toothBody

    # --- S14-S20: the spiral branch (psi > 0 only) -------------------------------------------

    def _buildSpiralTooth(self, designComponent: adsk.fusion.Component,
                          designOccurrence: adsk.fusion.Occurrence, gearProfilesPlane, toothBody,
                          toothPlane, apexWorld, axisDir, heelConeWorld, toeMidWorld,
                          heelMidWorld, gamma, handSign, label):
        # S14: the frame and the cone-element / trace planes.
        coneVec = normalize(_sub3(heelConeWorld, apexWorld))
        v = normalize(_cross3(axisDir, coneVec))

        def distAlong(p):
            return _dot3(_sub3(p, apexWorld), coneVec)

        R_toe = distAlong(toeMidWorld)
        R_heel = distAlong(heelMidWorld)
        R_mean = (R_toe + R_heel) / 2.0
        span = R_heel - R_toe
        if span <= 0:
            raise Exception(
                '{}: the toe-to-heel span is {:.6f}; a negative span inverts the spiral frame'
                .format(label, span))

        farPoint = solids.combine_point(apexWorld, R_heel, coneVec)
        coneElementSketch = designComponent.sketches.add(gearProfilesPlane)
        coneElementSketch.name = '{} Cone Element'.format(label)
        # World points fed straight into the sketch call, with no modelToSketchSpace conversion:
        # deliberate, and safe only because nothing downstream consumes this sketch's geometry.
        coneElementLine = coneElementSketch.sketchCurves.sketchLines.addByTwoPoints(
            apexWorld, farPoint)
        coneElementLine.isConstruction = True

        tracePlane = solids.plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)
        tracePlane.name = '{} Trace Plane'.format(label)

        # S15: the cutter arc.
        r_c = self._cutterRadiusInput if self._cutterRadiusInput != 0 else R_mean
        psi = self._spiralAngle_rad
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)
        toe2d = solids.circle_intersect_nearest(R_toe - 0.06 * span, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = solids.circle_intersect_nearest(R_heel + 0.06 * span, Cx, Cy, r_c, R_mean, 0.0)

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = '{} 2D Tooth Trace'.format(label)
        centerWorld = solids.combine_point(apexWorld, Cx, coneVec, Cy, v)
        cutterCircle = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(centerWorld, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True  # [PB-CIRCLE-CENTER]
        diamTextPoint = solids.combine_point(centerWorld, r_c, coneVec)
        diamDim = traceSketch.sketchDimensions.addDiameterDimension(cutterCircle, diamTextPoint)
        diamDim.parameter.value = 2 * r_c

        toeWorld = solids.combine_point(apexWorld, toe2d[0], coneVec, toe2d[1], v)
        heelWorld = solids.combine_point(apexWorld, heel2d[0], coneVec, heel2d[1], v)
        meanWorld = solids.combine_point(apexWorld, R_mean, coneVec, 0.0, v)
        traceArc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            toeWorld, meanWorld, heelWorld)
        traceArc.isConstruction = True
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radialTextPoint = solids.combine_point(centerWorld, r_c, v)
        radDim = traceSketch.sketchDimensions.addRadialDimension(traceArc, radialTextPoint)
        radDim.parameter.value = r_c
        # Deliberately left with free DOF; exempt from the full-constraint gate. Do not gate it.

        # S16: slice the uncut tooth into slabs.
        planeGeom = toothPlane.geometry
        testVec = _sub3(apexWorld, planeGeom.origin)
        sliceSign = 1.0 if _dot3(testVec, planeGeom.normal) > 0 else -1.0
        offsets = [sliceSign * (k + 1) * span / 6.0 for k in range(8)]
        pieces = solids.slice_body_by_offset_planes(designComponent, toothBody, toothPlane, offsets)
        if len(pieces) <= 1:
            offsets = [-o for o in offsets]
            pieces = solids.slice_body_by_offset_planes(
                designComponent, toothBody, toothPlane, offsets)
            if len(pieces) <= 1:
                raise Exception(
                    '{}: slice produced {} piece(s) after retrying with the opposite sign; '
                    'span={:.6f}, offsets tried={}'.format(label, len(pieces), span, offsets))

        # S17: order the segments and drop the apex scrap.
        def centroidDistAlong(body):
            return distAlong(body.physicalProperties.centerOfMass)

        pieces.sort(key=centroidDistAlong)
        scrap = pieces[0]
        segments = pieces[1:]
        if len(segments) == 0:
            raise Exception('{}: no segments remain after dropping the apex scrap'.format(label))
        designComponent.features.removeFeatures.add(scrap)

        # S18: twist the segments about the shaft axis.
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        def heelFaceOf(body):
            best, bestVal = None, None
            for face in body.faces:
                val = distAlong(face.centroid)
                if bestVal is None or val > bestVal:
                    bestVal, best = val, face
            return best, bestVal

        def toeFaceOf(body):
            best, bestVal = None, None
            for face in body.faces:
                val = distAlong(face.centroid)
                if bestVal is None or val < bestVal:
                    bestVal, best = val, face
            return best, bestVal

        for seg in segments:
            _, rHeelFace = heelFaceOf(seg)
            ang = -handSign * total * (R_mean - rHeelFace) / span
            if ang == 0:
                continue  # a zero rotation is a no-op, not a move (mirrors rotate_body_about_edge)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apexWorld)
            coll = adsk.core.ObjectCollection.create()
            coll.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(coll)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # S19: lengthwise crown -- scale every segment except the outermost.
        heelPostTwist = [heelFaceOf(seg) for seg in segments]
        outermostIdx = max(range(len(segments)), key=lambda idx: heelPostTwist[idx][1])

        anchorSketch = designComponent.sketches.add(gearProfilesPlane)
        anchorSketch.name = '{} Crown Anchors'.format(label)

        designOccurrence.activate()
        try:
            for idx, seg in enumerate(segments):
                if idx == outermostIdx:
                    continue
                heelFace, rHeelFace = heelPostTwist[idx]
                u = (R_heel - rHeelFace) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise Exception(
                        '{}: crown factor {:.6f} at u={:.6f} is not positive for segment {}'
                        .format(label, factor, u, idx))

                verts = sorted(
                    heelFace.vertices,
                    key=lambda vt: _perp_dist_to_axis(vt.geometry, apexWorld, axisDir))
                rootA, rootB = verts[0], verts[1]
                midWorld = _midpoint3(rootA.geometry, rootB.geometry)
                anchorLocal = anchorSketch.modelToSketchSpace(midWorld)
                anchorPoint = anchorSketch.sketchPoints.add(anchorLocal)

                scaleColl = adsk.core.ObjectCollection.create()
                scaleColl.add(seg)
                scaleInput = designComponent.features.scaleFeatures.createInput(
                    scaleColl, anchorPoint, adsk.core.ValueInput.createByReal(factor))
                designComponent.features.scaleFeatures.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # S20: re-sort by post-twist, post-crown heel-face cone distance and loft.
        def heelFaceDistAlong(body):
            return heelFaceOf(body)[1]

        sortedSegs = sorted(segments, key=heelFaceDistAlong)
        loftInput = designComponent.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        firstToeFace, _ = toeFaceOf(sortedSegs[0])
        loftInput.loftSections.add(firstToeFace)
        for seg in sortedSegs:
            heelFace, _ = heelFaceOf(seg)
            loftInput.loftSections.add(heelFace)
        spiralToothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
        spiralToothBody.name = '{} Spiral Tooth'.format(label)

        for seg in segments:
            designComponent.features.removeFeatures.add(seg)

        return spiralToothBody

    # --- S22: circular pattern, S23: combine-join --------------------------------------------

    def _patternAndCombine(self, designComponent: adsk.fusion.Component, trimmedBody, gearBody,
                           shaftAxisLine, teeth):
        seedColl = adsk.core.ObjectCollection.create()
        seedColl.add(trimmedBody)
        patternInput = designComponent.features.circularPatternFeatures.createInput(
            seedColl, shaftAxisLine)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teeth)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternFeature = designComponent.features.circularPatternFeatures.add(patternInput)

        # The pattern's bodies collection already includes the seed ([PB-PATTERN-BODIES]).
        toolColl = adsk.core.ObjectCollection.create()
        for idx in range(patternFeature.bodies.count):
            toolColl.add(patternFeature.bodies.item(idx))
        combineInput = designComponent.features.combineFeatures.createInput(gearBody, toolColl)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        designComponent.features.combineFeatures.add(combineInput)

    # --- S24: bore ----------------------------------------------------------------------------

    def _buildBore(self, designComponent: adsk.fusion.Component, shaftAxisLine, gearBody, boreDia,
                  label):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(shaftAxisLine, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = '{} Bore'.format(label)
        boreCircle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDia / 2.0)
        boreCircle.centerSketchPoint.isFixed = True  # [PB-CIRCLE-CENTER]
        textPoint = adsk.core.Point3D.create(boreDia / 2.0, 0, 0)
        diamDim = boreSketch.sketchDimensions.addDiameterDimension(boreCircle, textPoint)
        diamDim.parameter.value = boreDia

        if not boreSketch.isFullyConstrained:
            raise Exception('{} did not reach full constraint'.format(boreSketch.name))

        boreProfile = boreSketch.profiles.item(0)
        extrudeInput = designComponent.features.extrudeFeatures.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        # A generous half-length that clears the whole blank regardless of face width or base
        # height; the Pitch Cone Distance R is comfortably larger than the body's axial extent.
        extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * self._R), False)
        extrudeInput.participantBodies = [gearBody]
        designComponent.features.extrudeFeatures.add(extrudeInput)

    # --- one gear's whole chain, S7 through S25 ----------------------------------------------

    def _buildGear(self, designComponent: adsk.fusion.Component, bevelComponent: adsk.fusion.Component,
                   designOccurrence: adsk.fusion.Occurrence, gearProfilesPlane, apexPoint, label,
                   info):
        toothCenterLine = info['toothCenterLine']
        toothCenterPoint = info['toothCenterPoint']
        teeth = info['teeth']
        pitchDia = info['pitchDia']
        gamma = info['gamma']
        boreInput = info['boreInput']
        handSign = info['handSign']

        # S7.
        toothPlane = self._buildToothPlane(designComponent, gearProfilesPlane, toothCenterLine,
                                            label)
        # S8.
        toothSketch, embedded = self._buildToothSketch(
            designComponent, toothPlane, toothCenterPoint, pitchDia, gamma, label)
        # S9.
        self._buildToothAxis(designComponent, gearProfilesPlane, toothCenterLine, label)

        # S10.
        gearOccurrence = bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOccurrence.component.name = '{} Gear'.format(label)

        # S11.
        profileSketch, hexPts, hexLines = self._buildProfileSketch(
            designComponent, gearProfilesPlane, label, info['hex'])
        shaftAxisLine = hexLines[0]

        # S12.
        gearBody = self._revolveGearBody(designComponent, profileSketch, shaftAxisLine, label)

        # S13.
        toothBody = self._loftToothBody(designComponent, apexPoint, toothSketch, embedded, label)

        # The four-point hand-off for the tooth-body hook (S14/S21), with the swap guard.
        apexWorld = apexPoint.worldGeometry
        startW = shaftAxisLine.startSketchPoint.worldGeometry
        endW = shaftAxisLine.endSketchPoint.worldGeometry
        axisDir = normalize(_sub3(endW, startW))

        heelConeWorld = hexPts[3].worldGeometry
        toeConeWorld = hexPts[4].worldGeometry
        heelMidWorld = _midpoint3(hexPts[3].worldGeometry, hexPts[2].worldGeometry)
        toeMidWorld = _midpoint3(hexPts[4].worldGeometry, hexPts[5].worldGeometry)
        if _distance3(heelMidWorld, apexWorld) < _distance3(toeMidWorld, apexWorld):
            heelMidWorld, toeMidWorld = toeMidWorld, heelMidWorld
            heelConeWorld, toeConeWorld = toeConeWorld, heelConeWorld

        if self._spiralAngle_rad <= 0:
            trimmedBody = solids.cut_conical_ends(
                designComponent, toothBody, gearBody, toeMidWorld, heelMidWorld, apexWorld, label)
        else:
            spiralBody = self._buildSpiralTooth(
                designComponent, designOccurrence, gearProfilesPlane, toothBody, toothPlane,
                apexWorld, axisDir, heelConeWorld, toeMidWorld, heelMidWorld, gamma, handSign,
                label)
            trimmedBody = solids.cut_conical_ends(
                designComponent, spiralBody, gearBody, toeMidWorld, heelMidWorld, apexWorld, label)

        # S22, S23.
        self._patternAndCombine(designComponent, trimmedBody, gearBody, shaftAxisLine, teeth)

        # S24.
        if self._boreEnable:
            boreDia = boreInput if boreInput != 0 else pitchDia / 4.0
            self._buildBore(designComponent, shaftAxisLine, gearBody, boreDia, label)

        # S25. Driving gets a half-tooth-pitch offset; the pinion's mesh phase is zero by default,
        # and rotate_body_about_edge absorbs the resulting zero-angle no-op.
        if label == 'Driving':
            meshAngle = math.radians(180.0 / self._drivingTeeth)
        else:
            meshAngle = self._pinionMeshPhase(self._pinionTeeth)
        solids.rotate_body_about_edge(designComponent, gearBody, shaftAxisLine, meshAngle)

        return gearBody, gearOccurrence

    # --- S3 orchestration, S26: move the bodies, S27: cleanup --------------------------------

    def generate(self, inputs: adsk.core.CommandInputs):
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
         shaftAngle_deg) = self._readInputs(inputs)
        parentComponent: adsk.fusion.Component = parentComponent

        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        self.bevelOccurrence.component.name = 'Bevel Gear'
        bevelComponent = self.bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        anchorLine, anchorSketchCenter = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)
        gearProfilesPlane = self._buildGearProfilesPlane(designComponent, targetPlane, anchorLine)
        (gearProfilesSketch, apexPoint, pinionInfo, drivingInfo) = self._buildGearProfilesSketch(
            designComponent, gearProfilesPlane, targetPlane, anchorSketchCenter, anchorLine)

        results = {}
        for label, info in (('Pinion', pinionInfo), ('Driving', drivingInfo)):
            results[label] = self._buildGear(
                designComponent, bevelComponent, designOccurrence, gearProfilesPlane, apexPoint,
                label, info)

        # S26: relocate the finished bodies, which needs no activation ([PB-NO-CROSS-SIBLING]).
        for gearBody, gearOccurrence in results.values():
            gearBody: adsk.fusion.BRepBody = gearBody
            gearOccurrence: adsk.fusion.Occurrence = gearOccurrence
            gearBody.moveToComponent(gearOccurrence)

        # S27: cleanup.
        solids.hide_construction_geometry(bevelComponent)
