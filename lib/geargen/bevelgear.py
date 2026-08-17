# Bevel gear generator — generated from spec/bevelgear/instructions.md
# (+ fusion.md, spiral-tooth-trace.md) and the shared PLAYBOOK.md.
#
# Standalone generator: does NOT subclass base.Generator and carries no
# GenerationContext. One generator builds both straight (Mean Spiral Angle
# psi == 0) and spiral (psi > 0) bevels. Bevel registers no Fusion user
# parameters; every value is precomputed in Python (internal cm) and written
# into geometry numerically ([PB-PRECOMPUTED-MODE]).

import math
import adsk.core, adsk.fusion

from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator
from . import solids
from .solids import (
    cut_conical_ends,
    slice_body_by_offset_planes,
    rotate_body_about_edge,
    plane_by_angle,
    combine_point,
    circle_intersect_nearest,
    hide_construction_geometry,
)


# -- dialog input ids (reproduced surface; row order per the spec table) -------
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

# Hand-of-spiral list-item strings (reproduced surface).
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'


def _vec(a, b):
    """World vector b - a as a Vector3D (both Point3D)."""
    return adsk.core.Vector3D.create(b.x - a.x, b.y - a.y, b.z - a.z)


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _scale2(a, s):
    return (a[0] * s, a[1] * s)


def _norm2(a):
    n = math.hypot(a[0], a[1])
    if n == 0:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def _world_point(p):
    """A SketchPoint's world geometry as a Point3D."""
    return p.worldGeometry


def _midpoint(a, b):
    return adsk.core.Point3D.create(
        (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


class BevelGearCommandInputsConfigurator:
    """Adds the 17 dialog inputs (display order per the spec table) and drives
    the conditional visibility of the spiral-only inputs. Bound by name from
    commands/bevelgear/entry.py (configure / handle_input_changed)."""

    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs
        ValueInput = adsk.core.ValueInput
        Sel = adsk.core.SelectionCommandInput

        # 1: Target Plane (added first so it wins Fusion auto-focus —
        # [PB-AUTOFOCUS-FIRST]).
        plane = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the bottom of the driving gear sits flush against')
        plane.addSelectionFilter(Sel.ConstructionPlanes)
        plane.addSelectionFilter(Sel.PlanarFaces)
        plane.setSelectionLimits(1)

        # 2: Center Point
        center = inputs.addSelectionInput(
            INPUT_ID_CENTER_POINT, 'Center Point',
            'Point the driving bevel gear is centered on')
        center.addSelectionFilter(Sel.ConstructionPoints)
        center.addSelectionFilter(Sel.SketchPoints)
        center.setSelectionLimits(1)

        # 3: Parent Component (root pre-selected)
        parent = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component the gear pair is created under')
        parent.addSelectionFilter(Sel.Occurrences)
        parent.addSelectionFilter(Sel.RootComponents)
        parent.setSelectionLimits(1)
        parent.addSelection(get_design().rootComponent)

        # 4: Module
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', ValueInput.createByReal(1))

        # 5: Shaft Angle
        inputs.addValueInput(
            INPUT_ID_SHAFT_ANGLE, 'Shaft Angle', 'deg',
            ValueInput.createByString('90 deg'))

        # 6: Driving Gear Teeth
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            ValueInput.createByReal(31))

        # 7: Pinion Gear Teeth
        inputs.addValueInput(
            INPUT_ID_PINION_TEETH, 'Pinion Gear Teeth', '',
            ValueInput.createByReal(31))

        # 8: Driving Gear Base Height
        inputs.addValueInput(
            INPUT_ID_DRIVING_BASE_HEIGHT, 'Driving Gear Base Height', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 9: Pinion Gear Base Height
        inputs.addValueInput(
            INPUT_ID_PINION_BASE_HEIGHT, 'Pinion Gear Base Height', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 10: Enable Bore
        inputs.addBoolValueInput(
            INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

        # 11: Driving Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_DRIVING_BORE, 'Driving Gear Bore Diameter', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 12: Pinion Gear Bore Diameter
        inputs.addValueInput(
            INPUT_ID_PINION_BORE, 'Pinion Gear Bore Diameter', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 13: Face Width
        inputs.addValueInput(
            INPUT_ID_FACE_WIDTH, 'Face Width', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 14: Tooth Spacing
        inputs.addValueInput(
            INPUT_ID_TOOTH_SPACING, 'Tooth Spacing', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # 15: Mean Spiral Angle (controller; always visible)
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            ValueInput.createByString('35 deg'))

        # 16: Hand of Spiral (text-list dropdown)
        hand = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        hand.listItems.add(_HAND_RIGHT, True)
        hand.listItems.add(_HAND_LEFT, False)

        # 17: Cutter Radius
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            ValueInput.createByReal(to_cm(0)))

        # Last step: set the initial spiral-input visibility (default psi=35 ->
        # both shown).
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs):
        spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        hand = inputs.itemById(INPUT_ID_HAND)
        cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiral is None or hand is None or cutter is None:
            return
        try:
            value = get_design().unitsManager.evaluateExpression(
                spiral.expression, 'rad')
            show = value > 0
        except Exception:
            # A half-typed expression can raise mid-edit; leave both shown.
            show = True
        hand.isVisible = show
        cutter.isVisible = show

    @classmethod
    def handle_input_changed(cls, args):
        cls._updateSpiralInputVisibility(args.inputs)


class BevelGearGenerator:
    """Standalone generator building a straight (psi==0) or spiral (psi>0)
    bevel pair. Bound by name from commands/bevelgear/entry.py."""

    # Lengthwise-crown relief factor (0 disables the crown).
    _CROWN_PER_RAD = 0.5
    # Pinion's extra mesh phase, in tooth fractions (0 -> no extra nudge).
    _PINION_MESH_PHASE_TEETH = 0

    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # -- input reading ------------------------------------------------------
    def _readInputs(self, inputs):
        """Read + validate every input. Returns the 7-tuple
        (parentComponent, targetPlane, centerPoint, module, drivingTeeth,
        pinionTeeth, shaftAngle_deg) and stashes the rest on self."""
        unitsManager = self.design.unitsManager

        def evalv(name, units):
            inp = inputs.itemById(name)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections.
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Select exactly one Parent Component')
        parentSel = parents[0]
        if isinstance(parentSel, adsk.fusion.Occurrence) or \
                parentSel.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parentSel.component
        else:
            parentComponent = parentSel

        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Select exactly one Target Plane')
        targetPlane = planes[0]

        centers = get_selection(inputs, INPUT_ID_CENTER_POINT)
        if len(centers) != 1:
            raise Exception('Select exactly one Center Point')
        centerPoint = centers[0]

        # Module is read unit '' -> raw number that means MILLIMETRES.
        module = evalv(INPUT_ID_MODULE, '')
        if module <= 0:
            raise Exception('Module must be > 0')

        drivingTeeth = int(round(evalv(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(evalv(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Teeth number must be >= 3')

        # Shaft Angle reads back in radians; range-check in degrees.
        shaftAngle_rad = evalv(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception('Shaft Angle must be between 30 and 150 degrees')

        # 'mm' inputs come back already in internal cm — do NOT to_cm again.
        self._drivingBaseHeight_cm = evalv(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = evalv(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')

        self._boreEnable = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = evalv(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = evalv(INPUT_ID_PINION_BORE, 'mm')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')

        self._faceWidth_cm = evalv(INPUT_ID_FACE_WIDTH, 'mm')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')

        self._toothSpacing_cm = evalv(INPUT_ID_TOOTH_SPACING, 'mm')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        # Mean Spiral Angle reads back in radians; range-check [0, 60) deg.
        self._spiralAngle_rad = evalv(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(self._spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem = handInput.selectedItem
        self._hand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        # Cutter Radius reads back in internal cm via 'mm'. 0 -> auto.
        self._cutterRadius_cm = evalv(INPUT_ID_CUTTER_RADIUS, 'mm')
        if self._cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # -- orchestration ------------------------------------------------------
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        shaftAngle_rad = math.radians(shaftAngle_deg)

        # Resolve pitch diameters & bores in Python (internal cm).
        drivingPitchDia_cm = to_cm(module * drivingTeeth)
        pinionPitchDia_cm = to_cm(module * pinionTeeth)

        if self._drivingBore_cm > 0:
            drivingBoreDia_cm = self._drivingBore_cm
        else:
            drivingBoreDia_cm = drivingPitchDia_cm / 4.0
        if self._pinionBore_cm > 0:
            pinionBoreDia_cm = self._pinionBore_cm
        else:
            pinionBoreDia_cm = pinionPitchDia_cm / 4.0

        # Cone distance (internal cm).
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2
                      + (module * pinionTeeth) ** 2))
        self._coneDistance_cm = coneDistance_cm

        # Pitch-cone half-angles (used both for A/B seeds and §3 virtual radii).
        # tan gamma_p = sin Sigma * PPD / (DPD + PPD * cos Sigma)
        ppd = pinionPitchDia_cm
        dpd = drivingPitchDia_cm
        gamma_p = math.atan2(math.sin(shaftAngle_rad) * ppd,
                             dpd + ppd * math.cos(shaftAngle_rad))
        gamma_g = shaftAngle_rad - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        # Build the component tree: Bevel Gear under Parent, Design under it.
        Matrix3D = adsk.core.Matrix3D
        self.bevelOccurrence = parentComponent.occurrences.addNewComponent(
            Matrix3D.create())
        bevelComponent = self.bevelOccurrence.component
        bevelComponent.name = 'Bevel Gear'

        self.designOccurrence = bevelComponent.occurrences.addNewComponent(
            Matrix3D.create())
        designComponent = self.designOccurrence.component
        designComponent.name = 'Design'
        self.designComponent = designComponent
        self.bevelComponent = bevelComponent

        # §1 Anchor sketch.
        anchorLine = self._buildAnchorSketch(
            designComponent, targetPlane, centerPoint)

        # §2 + §3 + per-gear bodies.
        self._buildGearProfiles(
            designComponent, targetPlane, anchorLine, module,
            drivingTeeth, pinionTeeth, shaftAngle_rad,
            drivingPitchDia_cm, pinionPitchDia_cm,
            drivingBoreDia_cm, pinionBoreDia_cm)

        # Cleanup.
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
        self.bevelOccurrence = None

    # -- §1: Anchor Sketch --------------------------------------------------
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        # Start the sketch DIRECTLY on the selected target plane/face
        # ([PB-USE-SELECTED-PLANE]).
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor'

        # Project the user-specified center point onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        # Draw a reference line through the projected center.
        lines = sketch.sketchCurves.sketchLines
        cg = projectedCenter.geometry
        seedA = adsk.core.Point3D.create(cg.x - 0.5, cg.y, cg.z)
        seedB = adsk.core.Point3D.create(cg.x + 0.5, cg.y, cg.z)
        anchorLine = lines.addByTwoPoints(seedA, seedB)

        gc = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        # BOTH coincident (pins center onto line) and midpoint (center bisects).
        gc.addCoincident(projectedCenter, anchorLine)
        gc.addMidPoint(projectedCenter, anchorLine)
        # ~10mm reference length.
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + 0.2, cg.z))
        # Pin direction sketch-locally so the sketch is fully constrained.
        gc.addHorizontal(anchorLine)

        if not sketch.isFullyConstrained:
            raise Exception('Anchor sketch is not fully constrained')

        self._anchorSketch = sketch
        return anchorLine

    # -- §2 + §3 + per-gear bodies -----------------------------------------
    def _buildGearProfiles(self, designComponent, targetPlane, anchorLine,
                           module, drivingTeeth, pinionTeeth, shaftAngle_rad,
                           dpd_cm, ppd_cm, drivingBoreDia_cm, pinionBoreDia_cm):
        planes = designComponent.constructionPlanes
        ValueInput = adsk.core.ValueInput

        # Gear Profiles plane: includes the anchor line, set 90deg off the
        # ORIGINAL target plane (perpendicular) — [PB-USE-SELECTED-PLANE].
        gpInput = planes.createInput()
        gpInput.setByAngle(anchorLine,
                           ValueInput.createByString('90 deg'), targetPlane)
        gpPlane = planes.add(gpInput)
        gpPlane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = gpPlane

        sketch = designComponent.sketches.add(gpPlane)
        sketch.name = 'Gear Profiles'
        lines = sketch.sketchCurves.sketchLines
        gc = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        # Project the ANCHOR sketch's stashed center point (not the raw input).
        projected = sketch.project(self._anchorCenterPoint)
        c_pt = projected.item(0)
        c = c_pt.geometry  # 2-D (local) coords; (x, y)

        # Also need the projected anchor-line direction. Project the anchor line
        # to read its local direction.
        projLine = sketch.project(anchorLine)
        anchorProj = projLine.item(0)
        ap0 = anchorProj.startSketchPoint.geometry
        ap1 = anchorProj.endSketchPoint.geometry
        d = _norm2((ap1.x - ap0.x, ap1.y - ap0.y))
        perp = (-d[1], d[0])

        # Grow side: pick perp sign toward the target-plane NORMAL
        # ([BEVEL-F-GROW-SIDE]). Express the target normal in the sketch's local
        # frame by mapping two world points.
        normalWorld = self._targetNormal(targetPlane)
        # A world point a unit along the normal from the projected center.
        cWorld = c_pt.worldGeometry
        normEnd = adsk.core.Point3D.create(
            cWorld.x + normalWorld.x, cWorld.y + normalWorld.y,
            cWorld.z + normalWorld.z)
        normEndLocal = sketch.modelToSketchSpace(normEnd)
        normLocal = _norm2((normEndLocal.x - c.x, normEndLocal.y - c.y))
        if (perp[0] * normLocal[0] + perp[1] * normLocal[1]) < 0:
            perp = (-perp[0], -perp[1])

        DPD = dpd_cm
        PPD = ppd_cm

        # Apex position is sketch-local: c + perp * DPD ([BEVEL-F-APEX-LOCAL]).
        apex2d = (c.x + perp[0] * DPD, c.y + perp[1] * DPD)

        # center -> apex construction line (perpendicular to the anchor line).
        centerToApex = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x, c.y, 0),
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0))
        centerToApex.isConstruction = True
        gc.addCoincident(centerToApex.startSketchPoint, c_pt)
        gc.addPerpendicular(centerToApex, anchorProj)
        apexPoint = centerToApex.endSketchPoint

        # Closed-form along-shaft seed lengths.
        R = (PPD / 2.0) / math.sin(self._gamma_p)
        lenA = R * math.cos(self._gamma_p)
        lenB = R * math.cos(self._gamma_g)

        # Driving Gear Shaft Axis: from apex toward the anchor line (-perp), end
        # point B. Parallel to centerToApex.
        Bseed = (apex2d[0] - perp[0] * lenB, apex2d[1] - perp[1] * lenB)
        drivingShaft = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0),
            adsk.core.Point3D.create(Bseed[0], Bseed[1], 0))
        drivingShaft.isConstruction = True
        gc.addCoincident(drivingShaft.startSketchPoint, apexPoint)
        gc.addParallel(drivingShaft, centerToApex)
        pointB = drivingShaft.endSketchPoint

        # Pinion Gear Shaft Axis: driving direction rotated about apex by
        # +-Sigma; choose the sense whose endpoint has the greater X.
        drivingDir = _norm2((Bseed[0] - apex2d[0], Bseed[1] - apex2d[1]))

        def rot(vec, ang):
            ca, sa = math.cos(ang), math.sin(ang)
            return (vec[0] * ca - vec[1] * sa, vec[0] * sa + vec[1] * ca)

        candPlus = rot(drivingDir, shaftAngle_rad)
        candMinus = rot(drivingDir, -shaftAngle_rad)
        Aplus = (apex2d[0] + candPlus[0] * lenA, apex2d[1] + candPlus[1] * lenA)
        Aminus = (apex2d[0] + candMinus[0] * lenA,
                  apex2d[1] + candMinus[1] * lenA)
        if Aplus[0] >= Aminus[0]:
            pinionDir = candPlus
            Aseed = Aplus
        else:
            pinionDir = candMinus
            Aseed = Aminus

        pinionShaft = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0),
            adsk.core.Point3D.create(Aseed[0], Aseed[1], 0))
        pinionShaft.isConstruction = True
        gc.addCoincident(pinionShaft.startSketchPoint, apexPoint)
        pointA = pinionShaft.endSketchPoint

        # Angular dimension Sigma between the two shaft axes; text point inside
        # the Sigma wedge ([PB-ANGULAR-DIM]).
        bis = _norm2(_add2(pinionDir, drivingDir))
        textPt = (apex2d[0] + bis[0] * (PPD / 4.0),
                  apex2d[1] + bis[1] * (PPD / 4.0))
        dims.addAngularDimension(
            pinionShaft, drivingShaft,
            adsk.core.Point3D.create(textPt[0], textPt[1], 0))

        # A -> Apex2 : perpendicular drop from A toward the driving shaft / B,
        # length PPD/2.
        abDir = _norm2((Bseed[0] - Aseed[0], Bseed[1] - Aseed[1]))
        # perpendicular to pinion shaft, chosen toward A->B.
        pinionPerp = (-pinionDir[1], pinionDir[0])
        if (pinionPerp[0] * abDir[0] + pinionPerp[1] * abDir[1]) < 0:
            pinionPerp = (-pinionPerp[0], -pinionPerp[1])
        apex2seedA = (Aseed[0] + pinionPerp[0] * (PPD / 2.0),
                      Aseed[1] + pinionPerp[1] * (PPD / 2.0))
        aToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(Aseed[0], Aseed[1], 0),
            adsk.core.Point3D.create(apex2seedA[0], apex2seedA[1], 0))
        aToApex2.isConstruction = True
        gc.addCoincident(aToApex2.startSketchPoint, pointA)
        gc.addPerpendicular(aToApex2, pinionShaft)
        dims.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (Aseed[0] + apex2seedA[0]) / 2.0 + 0.1,
                (Aseed[1] + apex2seedA[1]) / 2.0, 0)
        ).parameter.value = PPD / 2.0

        # B -> Apex2 : perpendicular drop from B toward the pinion shaft / A,
        # length DPD/2.
        baDir = _norm2((Aseed[0] - Bseed[0], Aseed[1] - Bseed[1]))
        drivingPerp = (-drivingDir[1], drivingDir[0])
        if (drivingPerp[0] * baDir[0] + drivingPerp[1] * baDir[1]) < 0:
            drivingPerp = (-drivingPerp[0], -drivingPerp[1])
        apex2seedB = (Bseed[0] + drivingPerp[0] * (DPD / 2.0),
                      Bseed[1] + drivingPerp[1] * (DPD / 2.0))
        bToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(Bseed[0], Bseed[1], 0),
            adsk.core.Point3D.create(apex2seedB[0], apex2seedB[1], 0))
        bToApex2.isConstruction = True
        gc.addCoincident(bToApex2.startSketchPoint, pointB)
        gc.addPerpendicular(bToApex2, drivingShaft)
        dims.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (Bseed[0] + apex2seedB[0]) / 2.0 + 0.1,
                (Bseed[1] + apex2seedB[1]) / 2.0, 0)
        ).parameter.value = DPD / 2.0

        # Close the two drops at Apex 2.
        gc.addCoincident(aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint

        # Pitch Line Apex -> Apex2.
        pitchLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0),
            adsk.core.Point3D.create(apex2seedA[0], apex2seedA[1], 0))
        pitchLine.isConstruction = True
        gc.addCoincident(pitchLine.startSketchPoint, apexPoint)
        gc.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # Dedendum drops from Apex 2, perpendicular to the Pitch Line, length
        # Module*1.25, one to each side. The one toward the anchor line = D
        # (Driving Dedendum); away = C (Pinion Dedendum).
        ded_cm = to_cm(1.25 * module)
        ap2 = apex2seedA
        # pitch direction apex->apex2
        pitchDir = _norm2((ap2[0] - apex2d[0], ap2[1] - apex2d[1]))
        pitchPerp = (-pitchDir[1], pitchDir[0])
        # "toward the anchor line" = -perp (grow direction points away from it).
        towardAnchor = (-perp[0], -perp[1])
        if (pitchPerp[0] * towardAnchor[0] + pitchPerp[1] * towardAnchor[1]) >= 0:
            dDir = pitchPerp
            cDir = (-pitchPerp[0], -pitchPerp[1])
        else:
            dDir = (-pitchPerp[0], -pitchPerp[1])
            cDir = pitchPerp

        Dseed = (ap2[0] + dDir[0] * ded_cm, ap2[1] + dDir[1] * ded_cm)
        drivingDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap2[0], ap2[1], 0),
            adsk.core.Point3D.create(Dseed[0], Dseed[1], 0))
        drivingDedendum.isConstruction = True
        gc.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        gc.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (ap2[0] + Dseed[0]) / 2.0, (ap2[1] + Dseed[1]) / 2.0 + 0.1, 0)
        ).parameter.value = ded_cm
        pointD = drivingDedendum.endSketchPoint

        Cseed = (ap2[0] + cDir[0] * ded_cm, ap2[1] + cDir[1] * ded_cm)
        pinionDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(ap2[0], ap2[1], 0),
            adsk.core.Point3D.create(Cseed[0], Cseed[1], 0))
        pinionDedendum.isConstruction = True
        gc.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        gc.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (ap2[0] + Cseed[0]) / 2.0, (ap2[1] + Cseed[1]) / 2.0 + 0.1, 0)
        ).parameter.value = ded_cm
        pointC = pinionDedendum.endSketchPoint

        # Root Axes: Apex -> D (driving), Apex -> C (pinion).
        drivingRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0),
            adsk.core.Point3D.create(Dseed[0], Dseed[1], 0))
        drivingRootAxis.isConstruction = True
        gc.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        gc.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2d[0], apex2d[1], 0),
            adsk.core.Point3D.create(Cseed[0], Cseed[1], 0))
        pinionRootAxis.isConstruction = True
        gc.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        gc.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # Module-length extensions.
        mod_cm = to_cm(module)

        def collinear_ext(fromPoint, alongLine, dirVec, baseSeed):
            seed = (baseSeed[0] + dirVec[0] * mod_cm,
                    baseSeed[1] + dirVec[1] * mod_cm)
            ln = lines.addByTwoPoints(
                adsk.core.Point3D.create(baseSeed[0], baseSeed[1], 0),
                adsk.core.Point3D.create(seed[0], seed[1], 0))
            ln.isConstruction = True
            gc.addCoincident(ln.startSketchPoint, fromPoint)
            gc.addCollinear(ln, alongLine)
            return ln, seed

        # A -> E along Apex->A.
        aDirOut = _norm2((Aseed[0] - apex2d[0], Aseed[1] - apex2d[1]))
        lineAE, Eseed = collinear_ext(pointA, pinionShaft, aDirOut, Aseed)
        pointE = lineAE.endSketchPoint

        # C -> E perpendicular to A->E.
        lineCE = lines.addByTwoPoints(
            adsk.core.Point3D.create(Cseed[0], Cseed[1], 0),
            adsk.core.Point3D.create(Eseed[0], Eseed[1], 0))
        lineCE.isConstruction = True
        gc.addCoincident(lineCE.startSketchPoint, pointC)
        gc.addCoincident(lineCE.endSketchPoint, pointE)
        gc.addPerpendicular(lineAE, lineCE)

        # B -> F along Apex->B.
        bDirOut = _norm2((Bseed[0] - apex2d[0], Bseed[1] - apex2d[1]))
        lineBF, Fseed = collinear_ext(pointB, drivingShaft, bDirOut, Bseed)
        pointF = lineBF.endSketchPoint

        # D -> F perpendicular to B->F.
        lineDF = lines.addByTwoPoints(
            adsk.core.Point3D.create(Dseed[0], Dseed[1], 0),
            adsk.core.Point3D.create(Fseed[0], Fseed[1], 0))
        lineDF.isConstruction = True
        gc.addCoincident(lineDF.startSketchPoint, pointD)
        gc.addCoincident(lineDF.endSketchPoint, pointF)
        gc.addPerpendicular(lineBF, lineDF)

        # E -> G collinear to A->E.
        lineEG, Gseed = collinear_ext(pointE, lineAE, aDirOut, Eseed)
        pointG = lineEG.endSketchPoint

        # C -> H collinear to Apex2->C (pinion dedendum line).
        Hseed = (Cseed[0] + cDir[0] * mod_cm, Cseed[1] + cDir[1] * mod_cm)
        lineCH = lines.addByTwoPoints(
            adsk.core.Point3D.create(Cseed[0], Cseed[1], 0),
            adsk.core.Point3D.create(Hseed[0], Hseed[1], 0))
        lineCH.isConstruction = True
        gc.addCoincident(lineCH.startSketchPoint, pointC)
        gc.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # G -> H, with E->G perpendicular to H->G.
        lineGH = lines.addByTwoPoints(
            adsk.core.Point3D.create(Gseed[0], Gseed[1], 0),
            adsk.core.Point3D.create(Hseed[0], Hseed[1], 0))
        lineGH.isConstruction = True
        gc.addCoincident(lineGH.startSketchPoint, pointG)
        gc.addCoincident(lineGH.endSketchPoint, pointH)
        gc.addPerpendicular(lineEG, lineGH)

        # F -> I collinear to B->F.
        lineFI, Iseed = collinear_ext(pointF, lineBF, bDirOut, Fseed)
        pointI = lineFI.endSketchPoint

        # D -> J collinear to Apex2->D (driving dedendum line).
        Jseed = (Dseed[0] + dDir[0] * mod_cm, Dseed[1] + dDir[1] * mod_cm)
        lineDJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(Dseed[0], Dseed[1], 0),
            adsk.core.Point3D.create(Jseed[0], Jseed[1], 0))
        lineDJ.isConstruction = True
        gc.addCoincident(lineDJ.startSketchPoint, pointD)
        gc.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # I -> J, with F->I perpendicular to J->I.
        lineIJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(Iseed[0], Iseed[1], 0),
            adsk.core.Point3D.create(Jseed[0], Jseed[1], 0))
        lineIJ.isConstruction = True
        gc.addCoincident(lineIJ.startSketchPoint, pointI)
        gc.addCoincident(lineIJ.endSketchPoint, pointJ)
        gc.addPerpendicular(lineFI, lineIJ)

        # Driving base-height offset: between B->Apex2 drop and J->I.
        drivingBaseHeight_cm = self._drivingBaseHeight_cm
        if drivingBaseHeight_cm <= 0:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8.0)
        dims.addOffsetDimension(
            bToApex2, lineIJ,
            adsk.core.Point3D.create(
                (Jseed[0] + apex2seedB[0]) / 2.0,
                (Jseed[1] + apex2seedB[1]) / 2.0, 0)
        ).parameter.value = drivingBaseHeight_cm

        # Pinion base-height offset: between A->Apex2 drop and G->H.
        pinionBaseHeight_cm = self._pinionBaseHeight_cm
        if pinionBaseHeight_cm <= 0:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (
                pinionTeeth / float(drivingTeeth))
        dims.addOffsetDimension(
            aToApex2, lineGH,
            adsk.core.Point3D.create(
                (Hseed[0] + apex2seedA[0]) / 2.0,
                (Hseed[1] + apex2seedA[1]) / 2.0, 0)
        ).parameter.value = pinionBaseHeight_cm

        # Line A -> G.
        lineAG = lines.addByTwoPoints(
            adsk.core.Point3D.create(Aseed[0], Aseed[1], 0),
            adsk.core.Point3D.create(Gseed[0], Gseed[1], 0))
        lineAG.isConstruction = True
        gc.addCoincident(lineAG.startSketchPoint, pointA)
        gc.addCoincident(lineAG.endSketchPoint, pointG)

        # Constrain point I with the (projected) center point.
        gc.addCoincident(pointI, c_pt)

        # Line B -> I.
        lineBI = lines.addByTwoPoints(
            adsk.core.Point3D.create(Bseed[0], Bseed[1], 0),
            adsk.core.Point3D.create(Iseed[0], Iseed[1], 0))
        lineBI.isConstruction = True
        gc.addCoincident(lineBI.startSketchPoint, pointB)
        gc.addCoincident(lineBI.endSketchPoint, pointI)

        # K (pinion): from G along Apex->A, end pinned to Apex->A and the
        # pinion dedendum line (Apex2->C extended).
        Kseed = (Gseed[0] + aDirOut[0] * mod_cm, Gseed[1] + aDirOut[1] * mod_cm)
        lineGK = lines.addByTwoPoints(
            adsk.core.Point3D.create(Gseed[0], Gseed[1], 0),
            adsk.core.Point3D.create(Kseed[0], Kseed[1], 0))
        lineGK.isConstruction = True
        gc.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        gc.addCoincident(pointK, pinionShaft)
        gc.addCoincident(pointK, pinionDedendum)
        # Reference line C -> K.
        lineCK = lines.addByTwoPoints(
            adsk.core.Point3D.create(Cseed[0], Cseed[1], 0),
            adsk.core.Point3D.create(Kseed[0], Kseed[1], 0))
        lineCK.isConstruction = True
        gc.addCoincident(lineCK.startSketchPoint, pointC)
        gc.addCoincident(lineCK.endSketchPoint, pointK)

        # K' tooth-center (Tooth Spacing offset along the dedendum, away from C).
        pointKp, lineCKp = self._buildToothCenter(
            sketch, pointK, pointC, pinionDedendum, Cseed, Kseed, cDir)

        # L (driving): from I along Apex->B, pinned to Apex->B and the driving
        # dedendum line (Apex2->D extended).
        Lseed = (Iseed[0] + bDirOut[0] * mod_cm, Iseed[1] + bDirOut[1] * mod_cm)
        lineIL = lines.addByTwoPoints(
            adsk.core.Point3D.create(Iseed[0], Iseed[1], 0),
            adsk.core.Point3D.create(Lseed[0], Lseed[1], 0))
        lineIL.isConstruction = True
        gc.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        gc.addCoincident(pointL, drivingShaft)
        gc.addCoincident(pointL, drivingDedendum)
        lineDL = lines.addByTwoPoints(
            adsk.core.Point3D.create(Dseed[0], Dseed[1], 0),
            adsk.core.Point3D.create(Lseed[0], Lseed[1], 0))
        lineDL.isConstruction = True
        gc.addCoincident(lineDL.startSketchPoint, pointD)
        gc.addCoincident(lineDL.endSketchPoint, pointL)

        pointLp, lineDLp = self._buildToothCenter(
            sketch, pointL, pointD, drivingDedendum, Dseed, Lseed, dDir)

        # --- Maximum Face Width from SOLVED geometry ([PB-SOLVED-GEOMETRY]) ---
        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distA = self._perpDistPointToLine(
            (gA.x, gA.y), (gC.x, gC.y), (gH.x, gH.y))
        distB = self._perpDistPointToLine(
            (gB.x, gB.y), (gD.x, gD.y), (gJ.x, gJ.y))
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        faceWidth_cm = self._faceWidth_cm
        if faceWidth_cm <= 0:
            faceWidth_cm = min(self._coneDistance_cm / 6.0, maxFaceWidth_cm)
        elif faceWidth_cm > maxFaceWidth_cm:
            raise Exception(
                'Face Width %.4f cm exceeds the maximum %.4f cm' % (
                    faceWidth_cm, maxFaceWidth_cm))
        self._faceWidthResolved_cm = faceWidth_cm

        # M -> N (pinion toe line).
        pointM, pointN = self._buildToeLine(
            sketch, pinionRootAxis, aToApex2, lineCH, pointC, pointA,
            (gC.x, gC.y), (gH.x, gH.y), apex2d, faceWidth_cm)

        # O -> P (driving toe line).
        pointO, pointP = self._buildToeLine(
            sketch, drivingRootAxis, bToApex2, lineDJ, pointD, pointB,
            (gD.x, gD.y), (gJ.x, gJ.y), apex2d, faceWidth_cm)

        # B -> I already drawn above (spec also lists "draw line from B to I"
        # in the O->P paragraph; it is the same single line — created once,
        # [BEVEL-F-LINE-ONCE]).

        # Full-constraint gate for the permanent Gear Profiles sketch.
        if not sketch.isFullyConstrained:
            raise Exception('Gear Profiles sketch is not fully constrained')

        # Stash everything the per-gear body steps need.
        self._gpSketch = sketch
        self._apexSketchPoint = apexPoint
        self._apex2d = apex2d

        # §3 + per-gear bodies: pinion first, driving second.
        pinionCtx = {
            'gearLabel': 'Pinion',
            'teeth': pinionTeeth,
            'pitchDia_cm': ppd_cm,
            'gamma': self._gamma_p,
            'toothCenterPoint': pointKp,
            'toothCenterRefLine': lineCKp,
            'hexVerts': [pointA, pointG, pointH, pointC, pointM, pointN],
            'shaftEdgePoints': (pointA, pointG),
            'toeEdge': (pointM, pointN),
            'heelEdge': (pointC, pointH),
            'toeConePoint': pointM,
            'heelConePoint': pointC,
            'rootAxis': pinionRootAxis,
            'boreDia_cm': pinionBoreDia_cm,
            'meshAngle': self._pinionMeshPhase(pinionTeeth),
        }
        drivingCtx = {
            'gearLabel': 'Driving',
            'teeth': drivingTeeth,
            'pitchDia_cm': dpd_cm,
            'gamma': self._gamma_g,
            'toothCenterPoint': pointLp,
            'toothCenterRefLine': lineDLp,
            'hexVerts': [pointB, pointI, pointJ, pointD, pointO, pointP],
            'shaftEdgePoints': (pointB, pointI),
            'toeEdge': (pointO, pointP),
            'heelEdge': (pointD, pointJ),
            'toeConePoint': pointO,
            'heelConePoint': pointD,
            'rootAxis': drivingRootAxis,
            'boreDia_cm': drivingBoreDia_cm,
            'meshAngle': math.radians(180.0) / drivingTeeth,
        }

        for ctx in (pinionCtx, drivingCtx):
            self._buildVirtualSpurProfile(designComponent, module, ctx)
            self._createGearBody(designComponent, module, ctx)

    # -- Tooth-center (K' / L') --------------------------------------------
    def _buildToothCenter(self, sketch, basePoint, lowerCorner, dedendumLine,
                          cornerSeed, baseSeed, awayDir):
        """Build the tooth-center point shifted from basePoint outward along the
        dedendum line (away from lowerCorner) by Tooth Spacing. When spacing is
        0, returns basePoint and the C->K / D->L reference line."""
        lines = sketch.sketchCurves.sketchLines
        gc = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        spacing = self._toothSpacing_cm
        if spacing <= 0:
            # K' == K; reuse the C->K reference line (drawn by the caller).
            refLine = lines.addByTwoPoints(
                adsk.core.Point3D.create(cornerSeed[0], cornerSeed[1], 0),
                adsk.core.Point3D.create(baseSeed[0], baseSeed[1], 0))
            refLine.isConstruction = True
            gc.addCoincident(refLine.startSketchPoint, lowerCorner)
            gc.addCoincident(refLine.endSketchPoint, basePoint)
            return basePoint, refLine

        kpSeed = (baseSeed[0] + awayDir[0] * spacing,
                  baseSeed[1] + awayDir[1] * spacing)
        offLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(baseSeed[0], baseSeed[1], 0),
            adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
        offLine.isConstruction = True
        gc.addCoincident(offLine.startSketchPoint, basePoint)
        kpPoint = offLine.endSketchPoint
        gc.addCoincident(kpPoint, dedendumLine)
        dims.addDistanceDimension(
            offLine.startSketchPoint, offLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (baseSeed[0] + kpSeed[0]) / 2.0,
                (baseSeed[1] + kpSeed[1]) / 2.0 + 0.05, 0)
        ).parameter.value = spacing
        # Tooth-center reference line C->K' / D->L'.
        refLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(cornerSeed[0], cornerSeed[1], 0),
            adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
        refLine.isConstruction = True
        gc.addCoincident(refLine.startSketchPoint, lowerCorner)
        gc.addCoincident(refLine.endSketchPoint, kpPoint)
        return kpPoint, refLine

    # -- Toe line (M->N / O->P) --------------------------------------------
    def _buildToeLine(self, sketch, rootAxis, dropLine, heelLine,
                      cornerPoint, axisPoint, cornerSeed, heelEndSeed,
                      apex2d, faceWidth_cm):
        lines = sketch.sketchCurves.sketchLines
        gc = sketch.geometricConstraints
        dims = sketch.sketchDimensions

        # Seed M near midpoint of Apex->corner; seed N by sliding from M along
        # corner->heelEnd direction far enough to roughly reach the drop line.
        Mseed = ((apex2d[0] + cornerSeed[0]) / 2.0,
                 (apex2d[1] + cornerSeed[1]) / 2.0)
        chDir = _norm2(_sub2(heelEndSeed, cornerSeed))
        aPt = axisPoint.geometry
        distMtoA = math.hypot(Mseed[0] - aPt.x, Mseed[1] - aPt.y)
        Nseed = (Mseed[0] + chDir[0] * distMtoA, Mseed[1] + chDir[1] * distMtoA)

        toeLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(Mseed[0], Mseed[1], 0),
            adsk.core.Point3D.create(Nseed[0], Nseed[1], 0))
        toeLine.isConstruction = True
        pointM = toeLine.startSketchPoint
        pointN = toeLine.endSketchPoint
        gc.addCoincident(pointM, rootAxis)
        gc.addCoincident(pointN, dropLine)
        gc.addParallel(toeLine, heelLine)
        textPt = ((Mseed[0] + cornerSeed[0]) / 2.0,
                  (Mseed[1] + cornerSeed[1]) / 2.0)
        dims.addOffsetDimension(
            heelLine, toeLine,
            adsk.core.Point3D.create(textPt[0], textPt[1], 0)
        ).parameter.value = faceWidth_cm

        # M -> corner, N -> axisPoint reference lines.
        lineMC = lines.addByTwoPoints(
            adsk.core.Point3D.create(Mseed[0], Mseed[1], 0),
            adsk.core.Point3D.create(cornerSeed[0], cornerSeed[1], 0))
        lineMC.isConstruction = True
        gc.addCoincident(lineMC.startSketchPoint, pointM)
        gc.addCoincident(lineMC.endSketchPoint, cornerPoint)

        aSeed = (aPt.x, aPt.y)
        lineNA = lines.addByTwoPoints(
            adsk.core.Point3D.create(Nseed[0], Nseed[1], 0),
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0))
        lineNA.isConstruction = True
        gc.addCoincident(lineNA.startSketchPoint, pointN)
        gc.addCoincident(lineNA.endSketchPoint, axisPoint)

        return pointM, pointN

    # -- §3: virtual spur tooth profile ------------------------------------
    def _buildVirtualSpurProfile(self, designComponent, module, ctx):
        gearLabel = ctx['gearLabel']
        gamma = ctx['gamma']
        toothCenterPoint = ctx['toothCenterPoint']
        refLine = ctx['toothCenterRefLine']

        # Virtual (back-cone / Tredgold) tooth number from closed form.
        virtualPitchRadius_mm = (
            (ctx['pitchDia_cm'] * 10.0 / 2.0) / math.cos(gamma))
        virtualTeeth = int(math.floor(2.0 * virtualPitchRadius_mm / module))

        # Tooth plane: includes the tooth-center reference line, perpendicular
        # to the Gear Profiles sketch plane.
        toothPlane = plane_by_angle(
            designComponent, refLine, self._gearProfilesPlane, 90)
        toothPlane.name = '%s Plane' % gearLabel

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = '%s Tooth' % gearLabel

        proxy = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(toothCenterPoint, angle=math.radians(180))

        embedded = proxy._lastToothEmbedded

        if not toothSketch.isFullyConstrained:
            futil.log('%s Tooth sketch is not fully constrained '
                      '(exempt from the gate)' % gearLabel)

        # Tooth axis through the tooth-center point, normal to the tooth plane,
        # via setByTwoPlanes ([PB-CONSTRUCTION-AXES]).
        helperPlanes = designComponent.constructionPlanes
        hpInput = helperPlanes.createInput()
        hpInput.setByDistanceOnPath(
            refLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = helperPlanes.add(hpInput)

        axes = designComponent.constructionAxes
        axInput = axes.createInput()
        axInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
        toothAxis = axes.add(axInput)
        toothAxis.name = '%s Tooth Axis' % gearLabel

        ctx['toothSketch'] = toothSketch
        ctx['toothPlane'] = toothPlane
        ctx['embedded'] = embedded
        ctx['virtualTeeth'] = virtualTeeth

    # -- Create the Gear Bodies --------------------------------------------
    def _createGearBody(self, designComponent, module, ctx):
        gearLabel = ctx['gearLabel']
        features = designComponent.features

        # New component under Bevel Gear for the finished bodies.
        gearOccurrence = self.bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearComponent = gearOccurrence.component
        gearComponent.name = '%s Gear' % gearLabel

        # Profile sketch on the axial (Gear Profiles) plane — one per gear.
        profSketch = designComponent.sketches.add(self._gearProfilesPlane)
        profSketch.name = '%s Profile' % gearLabel
        lines = profSketch.sketchCurves.sketchLines

        # Recreate the six §2 vertices as new points at their world positions
        # ([PB-PROJECT-NOT-FIXED] recreate-share-fix recipe).
        verts = [
            profSketch.sketchPoints.add(
                profSketch.modelToSketchSpace(v.worldGeometry))
            for v in ctx['hexVerts']
        ]
        hexLines = []
        n = len(verts)
        for i in range(n):
            hexLines.append(
                lines.addByTwoPoints(verts[i], verts[(i + 1) % n]))
        # Fix the lines' endpoints AFTER the lines exist.
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        if not profSketch.isFullyConstrained:
            raise Exception('%s Profile sketch is not fully constrained'
                            % gearLabel)

        shaftAxisEdge = hexLines[0]  # A->G (pinion) / B->I (driving).

        # Revolve the single profile around the shaft-axis edge -> Gear Body.
        profile = profSketch.profiles.item(0)
        revolves = features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)

        # Tooth loft: Apex sketch point -> §3 tooth profile.
        toothProfile = find_profile_by_curve_counts(
            ctx['toothSketch'], nurbs=2, arcs=2,
            lines=(0 if ctx['embedded'] else 2))
        lofts = features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)

        # World hand-off points (§3a "Caller hand-off" table).
        toeEdge = ctx['toeEdge']
        heelEdge = ctx['heelEdge']
        toeMid = _midpoint(toeEdge[0].worldGeometry, toeEdge[1].worldGeometry)
        heelMid = _midpoint(heelEdge[0].worldGeometry,
                            heelEdge[1].worldGeometry)
        toeConeWorld = ctx['toeConePoint'].worldGeometry
        heelConeWorld = ctx['heelConePoint'].worldGeometry
        apexWorld = self._apexSketchPoint.worldGeometry

        # The tooth-body step (straight = 2 conical trims; spiral = curved).
        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            self._apexSketchPoint, toeMid, heelMid, toeConeWorld,
            heelConeWorld, ctx['toothPlane'], gearLabel, ctx['teeth'],
            ctx['gamma'])

        # Pattern around the shaft-axis edge.
        patterns = features.circularPatternFeatures
        bodyColl = adsk.core.ObjectCollection.create()
        bodyColl.add(toothBody)
        patInput = patterns.createInput(
            bodyColl, shaftAxisEdge)
        patInput.quantity = adsk.core.ValueInput.createByReal(ctx['teeth'])
        patInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patInput.isSymmetric = False
        pattern = patterns.add(patInput)

        # Combine-Join: gear body (target) + all patterned tooth bodies (tools).
        tools = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            tools.add(pattern.bodies.item(i))
        combines = features.combineFeatures
        combInput = combines.createInput(gearBody, tools)
        combInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combInput)

        # Bore.
        if self._boreEnable:
            self._cutBore(designComponent, gearComponent, gearBody,
                          shaftAxisEdge, ctx['boreDia_cm'], gearLabel)

        # Meshing rotation about the shaft axis.
        meshAngle = ctx['meshAngle']
        if meshAngle != 0:
            rotate_body_about_edge(
                designComponent, gearBody, shaftAxisEdge, meshAngle)

        # Relocate the finished body into its gear component.
        gearBody.moveToComponent(gearOccurrence)

    # -- Tooth-body hook ----------------------------------------------------
    def _transformToothBody(self, designComponent, toothBody, gearBody,
                            shaftAxisEdge, apexWorld, apexSketchPoint,
                            toeMid, heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber, gamma):
        # Straight bevel: byte-for-byte the prior two-cone flush trim.
        if self._spiralAngle_rad <= 0:
            return cut_conical_ends(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)
        # Spiral build (psi > 0).
        return self._buildSpiralTooth(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane,
            gearLabel, gamma)

    def _pinionMeshPhase(self, pinionTeeth):
        return self._PINION_MESH_PHASE_TEETH * 2.0 * math.pi / pinionTeeth

    # -- §3a: spiral tooth body --------------------------------------------
    def _buildSpiralTooth(self, designComponent, toothBody, gearBody,
                          shaftAxisEdge, apexWorld, toeMid, heelMid,
                          toeConeWorld, heelConeWorld, parentToothPlane,
                          gearLabel, gamma):
        features = designComponent.features

        # A. Gate & frame.
        startW = shaftAxisEdge.startSketchPoint.worldGeometry
        endW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = _vec(startW, endW)
        axisDir.normalize()
        apex = apexWorld

        # Fix swapped toe/heel: heel must be the OUTER end.
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = _vec(apex, heelConeWorld)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x
                    + (p.y - apex.y) * coneVec.y
                    + (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # B. Cutter-arc geometry.
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)
        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        # C. 2-D trace sketch (genuine cutter arc).
        axialPlane = self._gearProfilesPlane
        coneSketch = designComponent.sketches.add(axialPlane)
        coneSketch.name = '%s Cone Element' % gearLabel
        coneEnd = combine_point(apex, R_heel, coneVec)
        coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(
            apex, coneEnd)

        tracePlane = plane_by_angle(
            designComponent, coneElementLine, axialPlane, 90)
        tracePlane.name = '%s Trace Plane' % gearLabel

        traceSketch = designComponent.sketches.add(tracePlane)
        traceSketch.name = '%s 2D Tooth Trace' % gearLabel

        def tanW(px, py):
            return combine_point(apex, px, coneVec, py, v)

        circles = traceSketch.sketchCurves.sketchCircles
        arcs = traceSketch.sketchCurves.sketchArcs
        tdims = traceSketch.sketchDimensions

        cutterCenterW = tanW(Cx, Cy)
        cutterCircle = circles.addByCenterRadius(cutterCenterW, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        tdims.addDiameterDimension(
            cutterCircle, tanW(Cx + r_c, Cy)).parameter.value = 2.0 * r_c

        traceArc = arcs.addByThreePoints(
            tanW(toe2d[0], toe2d[1]),
            tanW(R_mean, 0.0),
            tanW(heel2d[0], heel2d[1]))
        traceSketch.geometricConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        tdims.addRadialDimension(
            traceArc, tanW(R_mean, 0.0)).parameter.value = r_c

        # E. Slice the straight tooth into cross-section slabs.
        normal = self._planeNormalWorld(parentToothPlane)
        planeOrigin = self._planeOriginWorld(parentToothPlane)
        toApex = _vec(planeOrigin, apex)
        dotv = (toApex.x * normal.x + toApex.y * normal.y + toApex.z * normal.z)
        sign = 1.0 if dotv >= 0 else -1.0

        def do_slice(sgn):
            offsets = [sgn * (k + 1) * span / 6.0 for k in range(8)]
            return slice_body_by_offset_planes(
                designComponent, toothBody, parentToothPlane, offsets)

        pieces = do_slice(sign)
        if len(pieces) <= 1:
            # Retry with the opposite sign.
            pieces = do_slice(-sign)
            if len(pieces) <= 1:
                raise RuntimeError(
                    '%s spiral: slice produced %d piece(s) (span=%.4f, '
                    'sign=%s) — cut planes missed' % (
                        gearLabel, len(pieces), span, sign))

        # F. Order & drop scrap.
        def centroid(piece):
            return piece.physicalProperties.centerOfMass

        pieces.sort(key=lambda p: distAlong(centroid(p)))
        scrap = pieces[0]
        segments = pieces[1:]
        features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise RuntimeError(
                '%s spiral: no segments after dropping scrap' % gearLabel)

        # G. Twist.
        phi_crown = (math.atan2(heel2d[1], heel2d[0])
                     - math.atan2(toe2d[1], toe2d[0]))
        total = abs(phi_crown) / math.sin(gamma)

        def slabFaceByExtreme(seg, want_max):
            best = None
            bestVal = None
            for face in seg.faces:
                cen = face.centroid
                val = distAlong(cen)
                if bestVal is None or (val > bestVal if want_max
                                       else val < bestVal):
                    bestVal = val
                    best = face
            return best, bestVal

        for seg in segments:
            heelFace, R_heelFace = slabFaceByExtreme(seg, want_max=True)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apex)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moveInput = features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            features.moveFeatures.add(moveInput)

        # H. Lengthwise crown (relief).
        # Sort by post-twist heel-face cone distance to find the outermost.
        segHeel = []
        for seg in segments:
            hf, rhf = slabFaceByExtreme(seg, want_max=True)
            segHeel.append((seg, hf, rhf))
        segHeel.sort(key=lambda t: t[2])
        outermostIdx = len(segHeel) - 1  # greatest distAlong -> heel

        self.designOccurrence.activate()
        try:
            for idx, (seg, heelFace, R_heelFace) in enumerate(segHeel):
                if idx == outermostIdx:
                    continue  # skip the heel segment (kept full).
                u = (R_heel - R_heelFace) / span
                factor = 1.0 - self._CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor <= 0:
                    raise RuntimeError(
                        '%s spiral: crown factor %.4f <= 0 (u=%.4f)' % (
                            gearLabel, factor, u))
                self._scaleSegment(designComponent, seg, heelFace, factor,
                                   apex, axisDir)
        finally:
            self.design.activateRootComponent()

        # I. Loft -> curved tooth (re-sort by post-twist heel-face distance).
        indexed = []
        for seg in segments:
            hf, rhf = slabFaceByExtreme(seg, want_max=True)
            indexed.append((seg, hf, rhf))
        indexed.sort(key=lambda t: t[2])

        toeSeg = indexed[0][0]
        toeFace, _ = slabFaceByExtreme(toeSeg, want_max=False)

        lofts = features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(toeFace)
        for (seg, heelFace, rhf) in indexed:
            loftInput.loftSections.add(heelFace)
        loft = lofts.add(loftInput)
        curvedTooth = loft.bodies.item(0)
        curvedTooth.name = '%s Spiral Tooth' % gearLabel

        # Remove the segment scaffolding (loft has captured their faces).
        for seg in segments:
            features.removeFeatures.add(seg)

        # J. Flush trim.
        return cut_conical_ends(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apexWorld, gearLabel)

    def _scaleSegment(self, designComponent, seg, heelFace, factor, apex,
                      axisDir):
        # Base point on the heel face's ROOT edge (gotcha 3): midpoint of the
        # two vertices nearest the shaft axis.
        verts = []
        for vtx in heelFace.vertices:
            verts.append(vtx.geometry)

        def perpDist(p):
            ap = _vec(apex, p)
            along = (ap.x * axisDir.x + ap.y * axisDir.y + ap.z * axisDir.z)
            perp = adsk.core.Vector3D.create(
                ap.x - along * axisDir.x,
                ap.y - along * axisDir.y,
                ap.z - along * axisDir.z)
            return perp.length

        verts.sort(key=perpDist)
        rootCorners = verts[:2]
        baseWorld = adsk.core.Point3D.create(
            (rootCorners[0].x + rootCorners[1].x) / 2.0,
            (rootCorners[0].y + rootCorners[1].y) / 2.0,
            (rootCorners[0].z + rootCorners[1].z) / 2.0)

        # A sketch point on the heel face for the scale base.
        baseSketch = designComponent.sketches.add(heelFace)
        basePt = baseSketch.sketchPoints.add(
            baseSketch.modelToSketchSpace(baseWorld))

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(seg)
        scales = designComponent.features.scaleFeatures
        scaleInput = scales.createInput(
            bodies, basePt, adsk.core.ValueInput.createByReal(factor))
        scales.add(scaleInput)

    # -- Bore ---------------------------------------------------------------
    def _cutBore(self, designComponent, gearComponent, gearBody, shaftAxisEdge,
                 boreDia_cm, gearLabel):
        planes = designComponent.constructionPlanes
        bpInput = planes.createInput()
        bpInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = planes.add(bpInput)

        sketch = designComponent.sketches.add(borePlane)
        sketch.name = '%s Bore' % gearLabel
        circles = sketch.sketchCurves.sketchCircles
        boreCircle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), boreDia_cm / 2.0)
        boreCircle.centerSketchPoint.isFixed = True
        sketch.sketchDimensions.addDiameterDimension(
            boreCircle,
            adsk.core.Point3D.create(boreDia_cm / 2.0, 0, 0)
        ).parameter.value = boreDia_cm

        if not sketch.isFullyConstrained:
            raise Exception('%s Bore sketch is not fully constrained'
                            % gearLabel)

        profile = sketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(2.0 * self._coneDistance_cm),
            False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # -- Cleanup ------------------------------------------------------------
    def _hideConstructionGeometry(self, bevelComponent):
        hide_construction_geometry(bevelComponent)

    # -- small geometry helpers --------------------------------------------
    def _targetNormal(self, targetPlane):
        if targetPlane.objectType == adsk.fusion.BRepFace.classType():
            return targetPlane.geometry.normal
        return targetPlane.geometry.normal

    def _planeNormalWorld(self, plane):
        return plane.geometry.normal

    def _planeOriginWorld(self, plane):
        return plane.geometry.origin

    def _perpDistPointToLine(self, p, a, b):
        # Perpendicular distance from 2-D point p to the line through a and b.
        ab = _sub2(b, a)
        n = math.hypot(ab[0], ab[1])
        if n == 0:
            return float('inf')
        ap = _sub2(p, a)
        cross = ap[0] * ab[1] - ap[1] * ab[0]
        return abs(cross) / n

    def _perpToAxis(self, p, apex, axisDir):
        ap = _vec(apex, p)
        along = (ap.x * axisDir.x + ap.y * axisDir.y + ap.z * axisDir.z)
        perp = adsk.core.Vector3D.create(
            ap.x - along * axisDir.x,
            ap.y - along * axisDir.y,
            ap.z - along * axisDir.z)
        return perp.length

    def _bottomEdgeMid(self, edge):
        return _midpoint(edge[0].worldGeometry, edge[1].worldGeometry)

    def _distDim(self, sketch, p0, p1, value, textPoint):
        dim = sketch.sketchDimensions.addDistanceDimension(
            p0, p1,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            textPoint)
        dim.parameter.value = value
        return dim
