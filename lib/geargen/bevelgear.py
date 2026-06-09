import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the only module-level name strings bevel reproduces)
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

# Hand-of-spiral dropdown list-item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so it wins Fusion's auto-focus.
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point (selection)
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Select the point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component (selection) -- pre-selected to the root component.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Select the parent component for the new bevel gear pair')
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

        # 6. Driving Gear Teeth Number
        inputs.addValueInput(
            INPUT_ID_DRIVING_TEETH, 'Driving Gear Teeth', '',
            adsk.core.ValueInput.createByReal(31))

        # 7. Pinion Gear Teeth Number
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

        # 10. Enable Bore (checkbox)
        inputs.addBoolValueInput(
            INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)

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

        # Initial conditional visibility of the spiral-only inputs (last step).
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            # Reads back in internal radians.
            value = get_design().unitsManager.evaluateExpression(
                spiralInput.expression, 'deg')
        except Exception:
            # A half-typed expression can raise mid-edit; leave both shown.
            handInput.isVisible = True
            cutterInput.isVisible = True
            return
        show = (value > 0)
        handInput.isVisible = show
        cutterInput.isVisible = show

    @classmethod
    def handle_input_changed(cls, args):
        cls._updateSpiralInputVisibility(args.inputs)


# ---------------------------------------------------------------------------
# Virtual spur proxy -- a fake spur `parent` so the borrowed spur tooth
# generator can run without registering Fusion user parameters.
# ---------------------------------------------------------------------------
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth):
        alpha = math.radians(20)  # bevel hardcodes 20deg pressure angle
        # Standard spur formulas (in mm), then to_cm so served lengths are in cm.
        pitch_mm = virtualTeeth * module_mm
        base_mm = pitch_mm * math.cos(alpha)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._params = {
            'Module': _Val(module_mm),
            'ToothNumber': _Val(virtualTeeth),
            'PressureAngle': _Val(alpha),
            'PitchCircleDiameter': _Val(to_cm(pitch_mm)),
            'PitchCircleRadius': _Val(to_cm(pitch_mm / 2)),
            'BaseCircleDiameter': _Val(to_cm(base_mm)),
            'BaseCircleRadius': _Val(to_cm(base_mm / 2)),
            'RootCircleDiameter': _Val(to_cm(root_mm)),
            'RootCircleRadius': _Val(to_cm(root_mm / 2)),
            'TipCircleDiameter': _Val(to_cm(tip_mm)),
            'TipCircleRadius': _Val(to_cm(tip_mm / 2)),
            'InvoluteSteps': _Val(15),
        }
        # Output the spur generator writes during draw(); absorb it up front.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return self._params[name]


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    # Spiral tuning constants.
    _CROWN_PER_RAD = 0.5            # lengthwise crown strength (0 disables)
    _PINION_MESH_PHASE_TEETH = 0    # extra pinion mesh nudge, in tooth-fractions

    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # -----------------------------------------------------------------------
    # Orchestration
    # -----------------------------------------------------------------------
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        sigma = math.radians(shaftAngle_deg)

        # Resolve pitch diameters & bores in Python (internal cm).
        drivingPitchDiameter = to_cm(module * drivingTeeth)
        pinionPitchDiameter = to_cm(module * pinionTeeth)

        drivingBore_cm = self._drivingBore_cm
        if drivingBore_cm <= 0:
            drivingBore_cm = drivingPitchDiameter / 4
        pinionBore_cm = self._pinionBore_cm
        if pinionBore_cm <= 0:
            pinionBore_cm = pinionPitchDiameter / 4

        # Cone distance (cm), pitch-cone half angles.
        coneDistance = to_cm(math.sqrt((module * drivingTeeth) ** 2
                                       + (module * pinionTeeth) ** 2))
        # tan gamma_p = sin S * PPD / (DPD + PPD*cos S)  (ratios; unit-free).
        gamma_p = math.atan2(
            math.sin(sigma) * pinionPitchDiameter,
            drivingPitchDiameter + pinionPitchDiameter * math.cos(sigma))
        gamma_g = sigma - gamma_p
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g
        self._coneDistance_cm = coneDistance

        # --- Build the component tree ---
        bevelOcc = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOcc.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOcc
        bevelComponent = bevelOcc.component

        designOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        designComponent = designOcc.component
        self._designOccurrence = designOcc

        # --- §1 Anchor Sketch ---
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # --- §2 + §3 + per-gear bodies ---
        self._buildGearProfiles(
            designComponent, targetPlane, anchorLine, bevelComponent,
            module, drivingTeeth, pinionTeeth, sigma,
            drivingPitchDiameter, pinionPitchDiameter,
            drivingBore_cm, pinionBore_cm)

        # --- Cleanup ---
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence is not None:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # -----------------------------------------------------------------------
    # Input reading / validation
    # -----------------------------------------------------------------------
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def rawValue(input_id, units):
            inp = inputs.itemById(input_id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Selections.
        (parents, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Exactly one parent component must be selected')
        parent = parents[0]
        if parent.objectType == adsk.fusion.Occurrence.classType():
            parentComponent = parent.component
        elif parent.objectType == adsk.fusion.Component.classType():
            parentComponent = parent
        else:
            raise Exception('Selected parent is not a component or occurrence')

        (planes, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Exactly one target plane must be selected')
        targetPlane = planes[0]

        (centers, _) = get_selection(inputs, INPUT_ID_CENTER)
        if len(centers) != 1:
            raise Exception('Exactly one center point must be selected')
        centerPoint = centers[0]

        # Numeric inputs.
        module = rawValue(INPUT_ID_MODULE, '')      # raw number meaning mm
        if module <= 0:
            raise Exception('Module must be positive')

        shaftAngle_rad = rawValue(INPUT_ID_SHAFT_ANGLE, 'deg')  # returns radians
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception('Shaft Angle must be between 30 and 150 degrees')

        drivingTeeth = int(round(rawValue(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(rawValue(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Teeth numbers must be at least 3')

        # 'mm' inputs come back already in internal cm.
        self._drivingBaseHeight_cm = rawValue(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = rawValue(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')

        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable

        self._drivingBore_cm = rawValue(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = rawValue(INPUT_ID_PINION_BORE, 'mm')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')

        self._faceWidth_cm = rawValue(INPUT_ID_FACE_WIDTH, 'mm')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')

        self._toothSpacing_cm = rawValue(INPUT_ID_TOOTH_SPACING, 'mm')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        spiralAngle_rad = rawValue(INPUT_ID_SPIRAL_ANGLE, 'deg')  # radians
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')
        self._spiralAngle_rad = spiralAngle_rad

        handInput = inputs.itemById(INPUT_ID_HAND)
        selectedItem = handInput.selectedItem
        self._spiralHand = selectedItem.name if selectedItem is not None else _HAND_RIGHT

        self._cutterRadius_cm = rawValue(INPUT_ID_CUTTER_RADIUS, 'mm')
        if self._cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # -----------------------------------------------------------------------
    # Shared helpers
    # -----------------------------------------------------------------------
    def _assertFullyConstrained(self, sketch):
        if not sketch.isFullyConstrained:
            raise Exception(
                'Sketch "{}" is not fully constrained (free DOF)'.format(sketch.name))

    def _pointWorldGeometry(self, point):
        # World Point3D for a SketchPoint (.worldGeometry) or a
        # ConstructionPoint (.geometry).
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _combine(self, base, a, e1, b=0.0, e2=None):
        # world point = base + a*e1 (+ b*e2); lengths in cm.
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, degrees):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByAngle(
            line, adsk.core.ValueInput.createByString('{} deg'.format(degrees)),
            refPlane)
        return comp.constructionPlanes.add(planeInput)

    def _circleIntersectNearest(self, R, Cx, Cy, r_c, refx, refy):
        # Intersect apex circle (centre origin, radius R) with cutter circle
        # (centre (Cx,Cy), radius r_c); return the solution nearest (refx,refy).
        d = math.hypot(Cx, Cy)
        if d == 0:
            raise Exception('Spiral: degenerate cutter circle (centre at apex)')
        # Distance from origin along the line to the radical axis.
        a = (R * R - r_c * r_c + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            raise Exception('Spiral: cutter circle does not meet apex circle '
                            '(R={:.4f})'.format(R))
        h = math.sqrt(h2)
        ux = Cx / d
        uy = Cy / d
        midx = a * ux
        midy = a * uy
        # Two solutions, perpendicular offset.
        s1 = (midx - h * uy, midy + h * ux)
        s2 = (midx + h * uy, midy - h * ux)
        d1 = math.hypot(s1[0] - refx, s1[1] - refy)
        d2 = math.hypot(s2[0] - refx, s2[1] - refy)
        return s1 if d1 <= d2 else s2

    # -----------------------------------------------------------------------
    # §1: Anchor Sketch
    # -----------------------------------------------------------------------
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # A ~10mm reference line; the projected center bisects it.
        cg = projectedCenter.geometry
        half = to_cm(5)
        p0 = adsk.core.Point3D.create(cg.x - half, cg.y, cg.z)
        p1 = adsk.core.Point3D.create(cg.x + half, cg.y, cg.z)
        anchorLine = lines.addByTwoPoints(p0, p1)

        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)
        dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + to_cm(2), cg.z))
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch)
        return anchorLine

    # -----------------------------------------------------------------------
    # §2 + §3 + per-gear bodies
    # -----------------------------------------------------------------------
    def _buildGearProfiles(self, designComponent, targetPlane, anchorLine,
                           bevelComponent, module, drivingTeeth, pinionTeeth,
                           sigma, drivingPitchDiameter, pinionPitchDiameter,
                           drivingBore_cm, pinionBore_cm):
        DPD = drivingPitchDiameter
        PPD = pinionPitchDiameter
        moduleLen = to_cm(module)
        dedendum = to_cm(1.25 * module)

        # --- Gear Profiles plane: perpendicular to targetPlane through anchor ---
        gpPlane = self._planeByAngle(designComponent, anchorLine, targetPlane, 90)
        gpPlane.name = 'Gear Profiles Plane'
        sketch = designComponent.sketches.add(gpPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True
        self._gearProfilesPlane = gpPlane
        self._gearProfilesSketch = sketch

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        def P2(p):
            if isinstance(p, (tuple, list)):
                return adsk.core.Point3D.create(
                    p[0], p[1], p[2] if len(p) > 2 else 0.0)
            return adsk.core.Point3D.create(p.x, p.y, p.z)

        # Project the anchor-sketch center point (NOT the raw external point).
        projected = sketch.project(self._anchorCenterPoint)
        centerProj = projected.item(0)
        c = centerProj.geometry

        # Also project the anchor line to obtain its 2-D direction in this sketch.
        projAnchor = sketch.project(anchorLine)
        anchorProjLine = projAnchor.item(0)
        ap0 = anchorProjLine.startSketchPoint.geometry
        ap1 = anchorProjLine.endSketchPoint.geometry
        dx = ap1.x - ap0.x
        dy = ap1.y - ap0.y
        dlen = math.hypot(dx, dy)
        d = (dx / dlen, dy / dlen)
        perp = (-d[1], d[0])

        # Grow-side: pick perp's sign so it points toward the target normal.
        targetNormal = get_normal(targetPlane)
        targetNormal.normalize()
        # Map the two candidate in-plane perp directions to world and compare.
        sketchOrigin = sketch.sketchToModelSpace(adsk.core.Point3D.create(0, 0, 0))
        sketchPerpW = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(perp[0], perp[1], 0))
        perpWorld = adsk.core.Vector3D.create(
            sketchPerpW.x - sketchOrigin.x,
            sketchPerpW.y - sketchOrigin.y,
            sketchPerpW.z - sketchOrigin.z)
        if perpWorld.dotProduct(targetNormal) < 0:
            perp = (-perp[0], -perp[1])

        def add_point(coord):
            return adsk.core.Point3D.create(coord[0], coord[1], 0)

        # --- center -> Apex construction line (apex = c + perp*DPD) ---
        apexSeed = (c.x + perp[0] * DPD, c.y + perp[1] * DPD)
        centerToApex = lines.addByTwoPoints(P2(c), add_point(apexSeed))
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, centerProj)
        constraints.addPerpendicular(centerToApex, anchorProjLine)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint

        # Seed along-shaft lengths from closed-form cone geometry.
        gamma_p = self._gamma_p
        gamma_g = self._gamma_g
        R = (PPD / 2) / math.sin(gamma_p)
        lenA = R * math.cos(gamma_p)  # |Apex->A|
        lenB = R * math.cos(gamma_g)  # |Apex->B|

        apx = apexSeed
        # Driving Gear Shaft Axis: from apex toward anchor line (-perp), -> point B.
        bSeed = (apx[0] - perp[0] * lenB, apx[1] - perp[1] * lenB)
        drivingShaftAxis = lines.addByTwoPoints(add_point(apx), add_point(bSeed))
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Gear Shaft Axis: driving direction rotated about apex by Sigma.
        # Choose the sense whose endpoint X is the larger.
        drivingDir = (-perp[0], -perp[1])

        def rot(vec, ang):
            ca = math.cos(ang)
            sa = math.sin(ang)
            return (vec[0] * ca - vec[1] * sa, vec[0] * sa + vec[1] * ca)

        dirPlus = rot(drivingDir, sigma)
        dirMinus = rot(drivingDir, -sigma)
        aPlus = (apx[0] + dirPlus[0] * lenA, apx[1] + dirPlus[1] * lenA)
        aMinus = (apx[0] + dirMinus[0] * lenA, apx[1] + dirMinus[1] * lenA)
        aSeed = aPlus if aPlus[0] >= aMinus[0] else aMinus

        pinionShaftAxis = lines.addByTwoPoints(add_point(apx), add_point(aSeed))
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        dimSigma = dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis,
            add_point((apx[0] - perp[0] * (lenA * 0.3),
                       apx[1] - perp[1] * (lenA * 0.3))))
        dimSigma.parameter.value = sigma
        pointA = pinionShaftAxis.endSketchPoint

        # Seed Apex2 roughly between the two shaft axes, above the anchor line.
        apex2Seed = (c.x + perp[0] * (DPD * 0.5), c.y + perp[1] * (DPD * 0.5))

        # A->Apex2 : perpendicular drop of length PPD/2 toward the anchor line.
        aToApex2 = lines.addByTwoPoints(P2(pointA.geometry), add_point(apex2Seed))
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaftAxis)
        dimensions.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            add_point(((pointA.geometry.x + apex2Seed[0]) / 2,
                       (pointA.geometry.y + apex2Seed[1]) / 2))
        ).parameter.value = PPD / 2

        # B->Apex2 : perpendicular drop of length DPD/2.
        bToApex2 = lines.addByTwoPoints(P2(pointB.geometry), add_point(apex2Seed))
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaftAxis)
        dimensions.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            add_point(((pointB.geometry.x + apex2Seed[0]) / 2,
                       (pointB.geometry.y + apex2Seed[1]) / 2))
        ).parameter.value = DPD / 2

        # Apex2 = coincidence of the two drop endpoints.
        constraints.addCoincident(aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint

        # Pitch Line Apex -> Apex2.
        pitchLine = lines.addByTwoPoints(P2(apx), P2(apex2Point.geometry))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # Dedendum lines from Apex2, perpendicular to Pitch Line, length 1.25*module.
        a2 = apex2Point.geometry
        pdx = a2.x - apx[0]
        pdy = a2.y - apx[1]
        plen = math.hypot(pdx, pdy)
        pdir = (pdx / plen, pdy / plen)
        pperp = (-pdir[1], pdir[0])
        candD1 = (a2.x + pperp[0] * dedendum, a2.y + pperp[1] * dedendum)
        candD2 = (a2.x - pperp[0] * dedendum, a2.y - pperp[1] * dedendum)
        # D is toward the anchor line == toward -perp (smaller perp-projection).
        projAlong1 = (candD1[0] - c.x) * perp[0] + (candD1[1] - c.y) * perp[1]
        projAlong2 = (candD2[0] - c.x) * perp[0] + (candD2[1] - c.y) * perp[1]
        if projAlong1 <= projAlong2:
            dSeed, cSeed = candD1, candD2
        else:
            dSeed, cSeed = candD2, candD1

        drivingDedendum = lines.addByTwoPoints(P2(a2), add_point(dSeed))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            add_point(((a2.x + dSeed[0]) / 2, (a2.y + dSeed[1]) / 2))
        ).parameter.value = dedendum
        pointD = drivingDedendum.endSketchPoint

        pinionDedendum = lines.addByTwoPoints(P2(a2), add_point(cSeed))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            add_point(((a2.x + cSeed[0]) / 2, (a2.y + cSeed[1]) / 2))
        ).parameter.value = dedendum
        pointC = pinionDedendum.endSketchPoint

        # Root Axes: Apex->D (driving), Apex->C (pinion).
        drivingRootAxis = lines.addByTwoPoints(P2(apx), P2(pointD.geometry))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(P2(apx), P2(pointC.geometry))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- module-length extensions and base-height hexagon corners ---
        ag = pointA.geometry
        aDir = (ag.x - apx[0], ag.y - apx[1])
        aDirLen = math.hypot(aDir[0], aDir[1])
        aDir = (aDir[0] / aDirLen, aDir[1] / aDirLen)
        eSeed = (ag.x + aDir[0] * moduleLen, ag.y + aDir[1] * moduleLen)
        lineAE = lines.addByTwoPoints(P2(ag), add_point(eSeed))
        lineAE.isConstruction = True
        constraints.addCoincident(lineAE.startSketchPoint, pointA)
        constraints.addCollinear(lineAE, pinionShaftAxis)
        pointE = lineAE.endSketchPoint

        # C->E (perpendicular to A->E).
        lineCE = lines.addByTwoPoints(P2(pointC.geometry), P2(pointE.geometry))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # B->F collinear with Apex->B, length module.
        bg = pointB.geometry
        bDir = (bg.x - apx[0], bg.y - apx[1])
        bDirLen = math.hypot(bDir[0], bDir[1])
        bDir = (bDir[0] / bDirLen, bDir[1] / bDirLen)
        fSeed = (bg.x + bDir[0] * moduleLen, bg.y + bDir[1] * moduleLen)
        lineBF = lines.addByTwoPoints(P2(bg), add_point(fSeed))
        lineBF.isConstruction = True
        constraints.addCoincident(lineBF.startSketchPoint, pointB)
        constraints.addCollinear(lineBF, drivingShaftAxis)
        pointF = lineBF.endSketchPoint

        # D->F (perpendicular to B->F).
        lineDF = lines.addByTwoPoints(P2(pointD.geometry), P2(pointF.geometry))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # E->G collinear with A->E, length module.
        eg = pointE.geometry
        gSeed = (eg.x + aDir[0] * moduleLen, eg.y + aDir[1] * moduleLen)
        lineEG = lines.addByTwoPoints(P2(eg), add_point(gSeed))
        lineEG.isConstruction = True
        constraints.addCoincident(lineEG.startSketchPoint, pointE)
        constraints.addCollinear(lineEG, lineAE)
        pointG = lineEG.endSketchPoint

        # C->H collinear with Apex2->C (pinionDedendum), length module.
        cg2 = pointC.geometry
        cDir = (cg2.x - a2.x, cg2.y - a2.y)
        cDirLen = math.hypot(cDir[0], cDir[1])
        cDir = (cDir[0] / cDirLen, cDir[1] / cDirLen)
        hSeed = (cg2.x + cDir[0] * moduleLen, cg2.y + cDir[1] * moduleLen)
        lineCH = lines.addByTwoPoints(P2(cg2), add_point(hSeed))
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # G->H, with E->G ⊥ H->G.
        lineGH = lines.addByTwoPoints(P2(pointG.geometry), P2(pointH.geometry))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # F->I collinear with B->F, length module.
        fg = pointF.geometry
        iSeed = (fg.x + bDir[0] * moduleLen, fg.y + bDir[1] * moduleLen)
        lineFI = lines.addByTwoPoints(P2(fg), add_point(iSeed))
        lineFI.isConstruction = True
        constraints.addCoincident(lineFI.startSketchPoint, pointF)
        constraints.addCollinear(lineFI, lineBF)
        pointI = lineFI.endSketchPoint

        # D->J collinear with Apex2->D (drivingDedendum), length module.
        dg = pointD.geometry
        dDir = (dg.x - a2.x, dg.y - a2.y)
        dDirLen = math.hypot(dDir[0], dDir[1])
        dDir = (dDir[0] / dDirLen, dDir[1] / dDirLen)
        jSeed = (dg.x + dDir[0] * moduleLen, dg.y + dDir[1] * moduleLen)
        lineDJ = lines.addByTwoPoints(P2(dg), add_point(jSeed))
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # I->J, with F->I ⊥ J->I.
        lineIJ = lines.addByTwoPoints(P2(pointI.geometry), P2(pointJ.geometry))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # --- base-height offset dimensions ---
        drivingBaseHeight = self._drivingBaseHeight_cm
        if drivingBaseHeight <= 0:
            drivingBaseHeight = to_cm(module * drivingTeeth / 8)
        jg = pointJ.geometry
        dimensions.addOffsetDimension(
            bToApex2, lineIJ,
            add_point(((jg.x + a2.x) / 2, (jg.y + a2.y) / 2))
        ).parameter.value = drivingBaseHeight

        pinionBaseHeight = self._pinionBaseHeight_cm
        if pinionBaseHeight <= 0:
            pinionBaseHeight = drivingBaseHeight * (pinionTeeth / drivingTeeth)
        hg = pointH.geometry
        dimensions.addOffsetDimension(
            aToApex2, lineGH,
            add_point(((hg.x + a2.x) / 2, (hg.y + a2.y) / 2))
        ).parameter.value = pinionBaseHeight

        # A->G connector.
        lineAG = lines.addByTwoPoints(P2(pointA.geometry), P2(pointG.geometry))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # Constrain point I with center point.
        constraints.addCoincident(pointI, centerProj)

        # --- K (pinion) and K' (tooth center) ---
        gg = pointG.geometry
        kSeed = (gg.x + aDir[0] * moduleLen, gg.y + aDir[1] * moduleLen)
        lineGK = lines.addByTwoPoints(P2(gg), add_point(kSeed))
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, lineCH)  # Apex2->C extended
        lineCK = lines.addByTwoPoints(P2(pointC.geometry), P2(pointK.geometry))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        pointKp, lineCKp = self._buildToothCenter(
            sketch, pointK, pointC, lineCH, lineCK, cDir)

        # --- L (driving) and L' ---
        ig = pointI.geometry
        lSeed = (ig.x + bDir[0] * moduleLen, ig.y + bDir[1] * moduleLen)
        lineIL = lines.addByTwoPoints(P2(ig), add_point(lSeed))
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, lineDJ)  # Apex2->D extended
        lineDL = lines.addByTwoPoints(P2(pointD.geometry), P2(pointL.geometry))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        pointLp, lineDLp = self._buildToothCenter(
            sketch, pointL, pointD, lineDJ, lineDL, dDir)

        # --- Maximum Face Width from SOLVED geometry ---
        maxFaceWidth = self._maxFaceWidth(
            pointA, pointB, pointC, pointD, pointH, pointJ)

        faceWidth = self._faceWidth_cm
        if faceWidth <= 0:
            faceWidth = min(self._coneDistance_cm / 6, maxFaceWidth)
        elif faceWidth > maxFaceWidth:
            raise Exception(
                'Face Width {:.3f} mm exceeds the maximum {:.3f} mm'.format(
                    to_mm(faceWidth), to_mm(maxFaceWidth)))
        self._resolvedFaceWidth = faceWidth

        # --- M->N (pinion toe line) ---
        self._buildToeLine(
            sketch, pointA, pointC, pointH, pinionRootAxis, aToApex2, lineCH,
            faceWidth, apx)
        pointM = self._lastM
        pointN = self._lastN

        # M->C and N->A connectors.
        lineMC = lines.addByTwoPoints(P2(pointM.geometry), P2(pointC.geometry))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(P2(pointN.geometry), P2(pointA.geometry))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # --- O->P (driving toe line) ---
        self._buildToeLine(
            sketch, pointB, pointD, pointJ, drivingRootAxis, bToApex2, lineDJ,
            faceWidth, apx)
        pointO = self._lastM
        pointP = self._lastN

        lineOD = lines.addByTwoPoints(P2(pointO.geometry), P2(pointD.geometry))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(P2(pointP.geometry), P2(pointB.geometry))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(P2(pointB.geometry), P2(pointI.geometry))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch)

        # World apex for the loft point + spiral frame.
        apexWorld = self._apexSketchPoint.worldGeometry

        # ----- §3: per-gear virtual spur profiles + bodies -----
        # Pinion first.
        pinionTooth = self._buildVirtualSpurProfile(
            designComponent, sketch, gpPlane, module, pinionPitchDiameter,
            gamma_p, pointKp, lineCKp, 'Pinion')
        self._createGearBody(
            designComponent, bevelComponent, sketch, gpPlane, apexWorld,
            self._apexSketchPoint, pinionTooth,
            pointM, pointN, pointC, pointH, lineAG,
            pinionTeeth, pinionBore_cm, gamma_p, 'Pinion')

        # Driving second.
        drivingTooth = self._buildVirtualSpurProfile(
            designComponent, sketch, gpPlane, module, drivingPitchDiameter,
            gamma_g, pointLp, lineDLp, 'Driving')
        self._createGearBody(
            designComponent, bevelComponent, sketch, gpPlane, apexWorld,
            self._apexSketchPoint, drivingTooth,
            pointO, pointP, pointD, pointJ, lineBI,
            drivingTeeth, drivingBore_cm, gamma_g, 'Driving')

    def _buildToothCenter(self, sketch, basePoint, lowerCorner, dedendumExtLine,
                          baseRefLine, dedDir):
        # Returns (toothCenterPoint, toothCenterRefLine).
        if self._toothSpacing_cm <= 0:
            return (basePoint, baseRefLine)

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        bp = basePoint.geometry
        kpSeed = (bp.x + dedDir[0] * self._toothSpacing_cm,
                  bp.y + dedDir[1] * self._toothSpacing_cm)
        line = lines.addByTwoPoints(
            adsk.core.Point3D.create(bp.x, bp.y, 0),
            adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
        line.isConstruction = True
        constraints.addCoincident(line.startSketchPoint, basePoint)
        toothCenter = line.endSketchPoint
        constraints.addCoincident(toothCenter, dedendumExtLine)
        dimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((bp.x + kpSeed[0]) / 2,
                                     (bp.y + kpSeed[1]) / 2, 0)
        ).parameter.value = self._toothSpacing_cm

        lc = lowerCorner.geometry
        refLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(lc.x, lc.y, 0),
            adsk.core.Point3D.create(toothCenter.geometry.x,
                                     toothCenter.geometry.y, 0))
        refLine.isConstruction = True
        constraints.addCoincident(refLine.startSketchPoint, lowerCorner)
        constraints.addCoincident(refLine.endSketchPoint, toothCenter)
        return (toothCenter, refLine)

    def _maxFaceWidth(self, pointA, pointB, pointC, pointD, pointH, pointJ):
        # Compute from SOLVED .geometry (NOT seed coords) -- see Parameters.
        a = pointA.geometry
        b = pointB.geometry
        cc = pointC.geometry
        dd = pointD.geometry
        h = pointH.geometry
        j = pointJ.geometry

        def perpDist(p, l0, l1):
            vx = l1[0] - l0[0]
            vy = l1[1] - l0[1]
            vlen = math.hypot(vx, vy)
            wx = p[0] - l0[0]
            wy = p[1] - l0[1]
            return abs(wx * vy - wy * vx) / vlen

        distA = perpDist((a.x, a.y), (cc.x, cc.y), (h.x, h.y))
        distB = perpDist((b.x, b.y), (dd.x, dd.y), (j.x, j.y))
        return 0.95 * min(distA, distB)

    def _buildToeLine(self, sketch, pointA, pointC, pointH, rootAxis, aToApex2,
                      lineCH, faceWidth, apx):
        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        cg = pointC.geometry
        # Seed M at midpoint of Apex->C.
        mSeed = ((apx[0] + cg.x) / 2, (apx[1] + cg.y) / 2)
        # Seed N by sliding along C->H direction far enough to reach line A->Apex2.
        hg = pointH.geometry
        chDir = (hg.x - cg.x, hg.y - cg.y)
        chLen = math.hypot(chDir[0], chDir[1])
        chDir = (chDir[0] / chLen, chDir[1] / chLen)
        ag = pointA.geometry
        slide = math.hypot(mSeed[0] - ag.x, mSeed[1] - ag.y)
        nSeed = (mSeed[0] + chDir[0] * slide, mSeed[1] + chDir[1] * slide)

        line = lines.addByTwoPoints(
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0),
            adsk.core.Point3D.create(nSeed[0], nSeed[1], 0))
        line.isConstruction = True
        pointM = line.startSketchPoint
        pointN = line.endSketchPoint
        constraints.addCoincident(pointM, rootAxis)
        constraints.addCoincident(pointN, aToApex2)
        constraints.addParallel(line, lineCH)
        dimensions.addOffsetDimension(
            lineCH, line,
            adsk.core.Point3D.create((mSeed[0] + cg.x) / 2,
                                     (mSeed[1] + cg.y) / 2, 0)
        ).parameter.value = faceWidth

        self._lastM = pointM
        self._lastN = pointN

    # -----------------------------------------------------------------------
    # §3: virtual spur tooth profile (one per gear)
    # -----------------------------------------------------------------------
    def _buildVirtualSpurProfile(self, designComponent, gpSketch, gpPlane,
                                 module, pitchDiameter, gamma, toothCenterPoint,
                                 toothCenterLine, gearLabel):
        # Virtual (Tredgold back-cone) tooth number.
        pitchRadius_cm = (pitchDiameter / 2)
        virtualPitchRadius = pitchRadius_cm / math.cos(gamma)
        # Module is mm; the radius is cm. Use mm units consistently.
        virtualTeeth = int(math.floor(2 * to_mm(virtualPitchRadius) / module))

        # Tooth plane: includes the center reference line, perpendicular to gpPlane.
        toothPlane = self._planeByAngle(designComponent, toothCenterLine, gpPlane, 90)
        toothPlane.name = '{} Plane'.format(gearLabel)

        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = '{} Tooth'.format(gearLabel)
        toothSketch.isVisible = True

        # Anchor point for the spur drawer: the tooth-center sketch point.
        anchorWorld = toothCenterPoint.worldGeometry
        anchorLocal = toothSketch.modelToSketchSpace(anchorWorld)
        anchorPoint = toothSketch.sketchPoints.add(anchorLocal)

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(anchorPoint, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded

        # Tooth-profile sketches are EXEMPT from the hard full-constraint gate:
        # the borrowed spur generator under-constrains the "embedded" (low
        # tooth-count) tooth (no flank-to-root radial pin), but every point is
        # placed at its computed involute position and the profile is consumed
        # immediately by the loft, so a residual DOF is benign. Log, never raise.
        if not toothSketch.isFullyConstrained:
            futil.log(
                '{} tooth sketch under-constrained (benign; gate exempt)'.format(
                    gearLabel))

        # Construction axis through the tooth center, normal to the tooth plane.
        helperPlaneInput = designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(
            toothCenterLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperPlaneInput)
        helperPlane.name = '{} Tooth Axis Helper'.format(gearLabel)
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gpPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = '{} Tooth Axis'.format(gearLabel)

        return {
            'sketch': toothSketch,
            'plane': toothPlane,
            'embedded': embedded,
            'axis': toothAxis,
            'label': gearLabel,
        }

    def _findSpurToothProfile(self, toothSketch, embedded):
        wantLines = 0 if embedded else 2
        for prof in toothSketch.profiles:
            for loop in prof.profileLoops:
                nLines = 0
                nArcs = 0
                nNurbs = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        nLines += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        nArcs += 1
                    elif ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nNurbs += 1
                if nNurbs == 2 and nArcs == 2 and nLines == wantLines:
                    return prof
        raise Exception(
            'Could not find spur tooth profile '
            '(2 NURBS + 2 arcs + {} lines, embedded={})'.format(wantLines, embedded))

    # -----------------------------------------------------------------------
    # Per-gear body creation
    # -----------------------------------------------------------------------
    def _createGearBody(self, designComponent, bevelComponent, gpSketch, gpPlane,
                        apexWorld, apexSketchPoint, toothInfo,
                        toeP0, toeP1, heelP0, heelP1, shaftAxisProfileEdge,
                        teethNumber, bore_cm, gamma, gearLabel):
        # Output component for this gear.
        gearOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOcc.component.name = '{} Gear'.format(gearLabel)

        # World points of the §2 vertices used for the toe/heel hand-off.
        toeP0w = self._pointWorldGeometry(toeP0)    # M / O
        toeP1w = self._pointWorldGeometry(toeP1)    # N / P
        heelP0w = self._pointWorldGeometry(heelP0)  # C / D
        heelP1w = self._pointWorldGeometry(heelP1)  # H / J

        # --- Profile sketch (one hexagon loop, fixed vertices) ---
        profSketch = designComponent.sketches.add(gpPlane)
        profSketch.name = '{} Profile'.format(gearLabel)
        profSketch.isVisible = True
        lines = profSketch.sketchCurves.sketchLines

        # Hexagon order A,G,H,C,M,N (pinion) / B,I,J,D,O,P (driving):
        #   V0=A/B, V1=G/I, V2=H/J, V3=C/D, V4=M/O, V5=N/P.
        # shaftAxisProfileEdge is the §2 A->G / B->I line (its endpoints are A/G).
        v0w = shaftAxisProfileEdge.startSketchPoint.worldGeometry  # A / B
        v1w = shaftAxisProfileEdge.endSketchPoint.worldGeometry    # G / I
        v2w = heelP1w  # H / J
        v3w = heelP0w  # C / D
        v4w = toeP0w   # M / O
        v5w = toeP1w   # N / P
        worldVerts = [v0w, v1w, v2w, v3w, v4w, v5w]
        verts = [profSketch.sketchPoints.add(profSketch.modelToSketchSpace(w))
                 for w in worldVerts]

        hexLines = []
        for i in range(6):
            ln = lines.addByTwoPoints(verts[i], verts[(i + 1) % 6])
            hexLines.append(ln)
        # Fix endpoints AFTER the lines exist.
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profSketch)

        # The shaft axis for all body ops is the first hexagon edge (A->G / B->I).
        shaftAxisEdge = hexLines[0]

        # --- Revolve the profile around that edge ---
        profile = profSketch.profiles.item(0)
        revolves = designComponent.features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = '{} Gear Body'.format(gearLabel)

        # --- Loft Apex point -> tooth profile (uncut tooth) ---
        toothProfile = self._findSpurToothProfile(
            toothInfo['sketch'], toothInfo['embedded'])
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = '{} Gear Tooth Body'.format(gearLabel)

        # --- Caller hand-off for the tooth-body hook (§3a table) ---
        toeMid = adsk.core.Point3D.create(
            (toeP0w.x + toeP1w.x) / 2, (toeP0w.y + toeP1w.y) / 2,
            (toeP0w.z + toeP1w.z) / 2)
        heelMid = adsk.core.Point3D.create(
            (heelP0w.x + heelP1w.x) / 2, (heelP0w.y + heelP1w.y) / 2,
            (heelP0w.z + heelP1w.z) / 2)
        toeConeWorld = toeP0w    # M / O
        heelConeWorld = heelP0w  # C / D

        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge,
            apexWorld, apexSketchPoint, toeMid, heelMid,
            toeConeWorld, heelConeWorld, toothInfo['plane'], gearLabel,
            teethNumber, gamma)

        # --- Pattern around the shaft axis ---
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(toothBody)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # --- Combine-join all tooth pieces into the gear body ---
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ---
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel)

        # --- Mesh-rotate ---
        axisStart = shaftAxisEdge.startSketchPoint.worldGeometry
        axisEnd = shaftAxisEdge.endSketchPoint.worldGeometry
        axisVec = adsk.core.Vector3D.create(
            axisEnd.x - axisStart.x, axisEnd.y - axisStart.y, axisEnd.z - axisStart.z)
        axisVec.normalize()
        if gearLabel == 'Driving':
            meshAngle = math.radians(180.0 / teethNumber)
        else:
            meshAngle = self._pinionMeshPhase() * (2 * math.pi / teethNumber)
        if meshAngle != 0:
            self._rotateBody(designComponent, gearBody, meshAngle, axisVec, axisStart)

        # --- Relocate the finished body to its gear component ---
        gearBody.moveToComponent(gearOcc)

    def _pinionMeshPhase(self):
        # 0 unless a spiral pair needs a nudge (mid-face section is unrotated).
        return self._PINION_MESH_PHASE_TEETH

    def _rotateBody(self, designComponent, body, angle, axisVec, origin):
        coll = adsk.core.ObjectCollection.create()
        coll.add(body)
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angle, axisVec, origin)
        moves = designComponent.features.moveFeatures
        moveInput = moves.createInput2(coll)
        moveInput.defineAsFreeMove(matrix)
        moves.add(moveInput)

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = '{} Bore Plane'.format(gearLabel)

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = '{} Bore'.format(gearLabel)
        boreSketch.isVisible = True
        circles = boreSketch.sketchCurves.sketchCircles
        dimensions = boreSketch.sketchDimensions

        radius = bore_cm / 2
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0)
        ).parameter.value = bore_cm

        self._assertFullyConstrained(boreSketch)

        profile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(to_cm(1000)), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # -----------------------------------------------------------------------
    # Tooth-body hook -- straight (cone trims) or spiral (ψ>0)
    # -----------------------------------------------------------------------
    def _transformToothBody(self, designComponent, toothBody, gearBody,
                            shaftAxisEdge, apexWorld, apexSketchPoint,
                            toeMid, heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber, gamma):
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)

        return self._buildSpiralTooth(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane,
            gearLabel, gamma)

    # -----------------------------------------------------------------------
    # Conical end trim (straight tooth, also flushes spiral ends)
    # -----------------------------------------------------------------------
    def _findConeFaceForCutLine(self, body, edgeMidWorld):
        # Return cone faces of `body` ordered best-first by surface distance to
        # the cut-edge midpoint (unevaluable distance -> tried last).
        candidates = []
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            dist = self._surfaceDistance(face.geometry, edgeMidWorld)
            candidates.append((dist if dist is not None else float('inf'), face))
        candidates.sort(key=lambda t: t[0])
        return [face for (_, face) in candidates]

    def _surfaceDistance(self, surface, worldPoint):
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return projected.distanceTo(worldPoint)

    def _applyConicalCut(self, designComponent, pieces, gearBody, edgeMidWorld,
                         apexWorld, cutLabel):
        coneFaces = self._findConeFaceForCutLine(gearBody, edgeMidWorld)
        splits = designComponent.features.splitBodyFeatures
        for face in coneFaces:
            newPieces = []
            didSplit = False
            for piece in pieces:
                try:
                    splitInput = splits.createInput(piece, face, True)
                    split = splits.add(splitInput)
                    resultBodies = list(split.bodies)
                except RuntimeError:
                    resultBodies = [piece]
                if len(resultBodies) > 1:
                    didSplit = True
                newPieces.extend(resultBodies)
            if didSplit:
                futil.log(
                    '{}: cut split into {} piece(s) (cone faces tried)'.format(
                        cutLabel, len(newPieces)), force_console=True)
                return self._keepLargestNonApex(
                    designComponent, newPieces, apexWorld, cutLabel)
        raise Exception(
            '{}: cut found no cone face that splits the body '
            '(cone faces tried={})'.format(cutLabel, len(coneFaces)))

    def _keepLargestNonApex(self, designComponent, pieces, apexWorld, cutLabel):
        removes = designComponent.features.removeFeatures
        nonApex = []
        apexPieces = []
        for piece in pieces:
            containment = piece.pointContainment(apexWorld)
            if containment == adsk.fusion.PointContainment.PointOutsidePointContainment:
                nonApex.append(piece)
            else:
                apexPieces.append(piece)
        if len(nonApex) == 0:
            raise Exception(
                '{}: keeper selection found 0 non-apex pieces '
                '(total={})'.format(cutLabel, len(pieces)))
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for piece in apexPieces:
            removes.add(piece)
        for piece in nonApex[1:]:
            removes.add(piece)
        return keeper

    def _cutConicalEnds(self, designComponent, toothBody, gearBody,
                        toeMid, heelMid, apexWorld, gearLabel):
        # Cut 1: toe cone, keep largest non-apex; Cut 2: heel cone on keeper only.
        keeper = self._applyConicalCut(
            designComponent, [toothBody], gearBody, toeMid, apexWorld,
            '{} toe'.format(gearLabel))

        # Heel cut on the keeper alone; the heel cone may not intersect.
        coneFaces = self._findConeFaceForCutLine(gearBody, heelMid)
        splits = designComponent.features.splitBodyFeatures
        for face in coneFaces:
            try:
                splitInput = splits.createInput(keeper, face, True)
                split = splits.add(splitInput)
                resultBodies = list(split.bodies)
            except RuntimeError:
                continue
            if len(resultBodies) > 1:
                return self._keepLargestNonApex(
                    designComponent, resultBodies, apexWorld,
                    '{} heel'.format(gearLabel))
        # Heel cone did not reach the keeper -- it is already the tooth band.
        futil.log('{} heel: tool did not intersect, kept keeper intact'.format(
            gearLabel), force_console=True)
        return keeper

    # -----------------------------------------------------------------------
    # §3a: spiral tooth body
    # -----------------------------------------------------------------------
    def _buildSpiralTooth(self, designComponent, toothBody, gearBody,
                          shaftAxisEdge, apexWorld, toeMid, heelMid,
                          toeConeWorld, heelConeWorld, parentToothPlane,
                          gearLabel, gamma):
        apex = apexWorld

        # --- A. Frame ---
        as0 = shaftAxisEdge.startSketchPoint.worldGeometry
        as1 = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = adsk.core.Vector3D.create(
            as1.x - as0.x, as1.y - as0.y, as1.z - as0.z)
        axisDir.normalize()

        # Swap guard: heel must be the OUTER end (farther from the apex).
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = adsk.core.Vector3D.create(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y,
            heelConeWorld.z - apex.z)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x + (p.y - apex.y) * coneVec.y
                    + (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe
        if span <= 0:
            raise Exception(
                '{}: spiral span non-positive ({:.4f}) -- toe/heel swapped'.format(
                    gearLabel, span))

        # --- B. Cutter-arc geometry ---
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = 1.0 if self._spiralHand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = self._circleIntersectNearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        # --- C. 2-D trace sketch (the genuine cutter arc) ---
        gpPlane = self._gearProfilesPlane
        coneSketch = designComponent.sketches.add(gpPlane)
        coneSketch.name = '{} Cone Element'.format(gearLabel)
        coneSketch.isVisible = True
        coneEndWorld = self._combine(apex, R_heel, coneVec)
        coneLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(
            coneSketch.modelToSketchSpace(apex),
            coneSketch.modelToSketchSpace(coneEndWorld))
        coneLine.isConstruction = True

        tangentPlane = self._planeByAngle(designComponent, coneLine, gpPlane, 90)
        tangentPlane.name = '{} Trace Plane'.format(gearLabel)

        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = '{} 2D Tooth Trace'.format(gearLabel)
        traceSketch.isVisible = True

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        circles = traceSketch.sketchCurves.sketchCircles
        arcs = traceSketch.sketchCurves.sketchArcs
        dims = traceSketch.sketchDimensions
        constraints = traceSketch.geometricConstraints

        centerLocal = traceSketch.modelToSketchSpace(tanW(Cx, Cy))
        cutterCircle = circles.addByCenterRadius(centerLocal, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        offCircleLocal = traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy))
        dims.addDiameterDimension(cutterCircle, offCircleLocal).parameter.value = 2 * r_c

        toeLocal = traceSketch.modelToSketchSpace(tanW(toe2d[0], toe2d[1]))
        meanLocal = traceSketch.modelToSketchSpace(tanW(R_mean, 0.0))
        heelLocal = traceSketch.modelToSketchSpace(tanW(heel2d[0], heel2d[1]))
        traceArc = arcs.addByThreePoints(toeLocal, meanLocal, heelLocal)
        constraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        # Radial dimension text point must be ON the arc (the mean point).
        dims.addRadialDimension(traceArc, meanLocal).parameter.value = r_c
        # (Trace sketch is deliberately under-constrained -- exempt from gate.)

        # --- E. Slice the straight tooth ---
        segments = self._sliceTooth(
            designComponent, toothBody, parentToothPlane, apex, coneVec,
            span, distAlong, gearLabel)

        # --- F. Order & drop scrap ---
        segments.sort(key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                '{}: spiral slice left 0 working segments'.format(gearLabel))

        # --- G. Twist ---
        phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
        total = abs(phi_crown) / math.sin(gamma)

        def slabHeelFace(seg):
            best = None
            bestD = None
            for face in seg.faces:
                dd = distAlong(face.centroid)
                if bestD is None or dd > bestD:
                    bestD = dd
                    best = face
            return best

        for seg in segments:
            heelFace = slabHeelFace(seg)
            R_heelFace = distAlong(heelFace.centroid)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            self._rotateBody(designComponent, seg, ang, axisDir, apex)

        # --- H. Lengthwise crown ---
        self._crownSegments(
            designComponent, segments, slabHeelFace, distAlong,
            axisDir, apex, R_heel, span, total)

        # --- I. Loft -> curved tooth (re-sort post-twist) ---
        order = sorted(range(len(segments)),
                       key=lambda i: distAlong(slabHeelFace(segments[i]).centroid))
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # toe-most segment's toe (apex-side) face first.
        toeSeg = segments[order[0]]
        toeFace = self._slabToeFace(toeSeg, distAlong)
        loftInput.loftSections.add(toeFace)
        for i in order:
            loftInput.loftSections.add(slabHeelFace(segments[i]))
        loft = lofts.add(loftInput)
        curvedTooth = loft.bodies.item(0)
        curvedTooth.name = '{} Spiral Tooth'.format(gearLabel)

        # Remove the segment scaffolding (the loft captured their faces).
        removes = designComponent.features.removeFeatures
        for seg in segments:
            removes.add(seg)

        # --- J. Flush trim ---
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apex, gearLabel)

    def _slabToeFace(self, seg, distAlong):
        best = None
        bestD = None
        for face in seg.faces:
            dd = distAlong(face.centroid)
            if bestD is None or dd < bestD:
                bestD = dd
                best = face
        return best

    def _sliceTooth(self, designComponent, toothBody, parentToothPlane, apex,
                    coneVec, span, distAlong, gearLabel):
        normal = get_normal(parentToothPlane)
        normal.normalize()
        planeOrigin = parentToothPlane.geometry.origin
        toApex = adsk.core.Vector3D.create(
            apex.x - planeOrigin.x, apex.y - planeOrigin.y, apex.z - planeOrigin.z)
        baseSign = 1.0 if (normal.dotProduct(toApex) >= 0) else -1.0

        def attempt(sign):
            pieces = [toothBody]
            step = span / 6
            cutPlanes = []
            for k in range(8):
                pinput = designComponent.constructionPlanes.createInput()
                pinput.setByOffset(
                    parentToothPlane,
                    adsk.core.ValueInput.createByReal(sign * (k + 1) * step))
                cp = designComponent.constructionPlanes.add(pinput)
                cp.isLightBulbOn = False
                cutPlanes.append(cp)
            splits = designComponent.features.splitBodyFeatures
            for cp in cutPlanes:
                newPieces = []
                for piece in pieces:
                    try:
                        splitInput = splits.createInput(piece, cp, True)
                        split = splits.add(splitInput)
                        newPieces.extend(list(split.bodies))
                    except RuntimeError:
                        newPieces.append(piece)
                pieces = newPieces
            return pieces

        pieces = attempt(baseSign)
        if len(pieces) <= 1:
            pieces = attempt(-baseSign)
        if len(pieces) <= 1:
            raise Exception(
                '{}: spiral slice produced {} piece(s) (span={:.4f}, both signs '
                'tried) -- parentToothPlane outside tooth span'.format(
                    gearLabel, len(pieces), span))
        return pieces

    def _crownSegments(self, designComponent, segments, slabHeelFace, distAlong,
                       axisDir, apex, R_heel, span, total):
        if self._CROWN_PER_RAD <= 0:
            return
        # Identify the outermost (heel) segment to skip.
        heelDists = [distAlong(slabHeelFace(seg).centroid) for seg in segments]
        outerIdx = max(range(len(segments)), key=lambda i: heelDists[i])

        designOcc = self._designOccurrence
        scales = designComponent.features.scaleFeatures
        designOcc.activate()
        try:
            for idx, seg in enumerate(segments):
                if idx == outerIdx:
                    continue
                heelFace = slabHeelFace(seg)
                R_heelFace = distAlong(heelFace.centroid)
                u = (R_heel - R_heelFace) / span
                factor = 1 - self._CROWN_PER_RAD * (abs(total) / 2) * u
                if factor >= 1 or factor <= 0:
                    continue

                basePoint = self._heelRootBasePoint(
                    designComponent, heelFace, axisDir, apex)
                coll = adsk.core.ObjectCollection.create()
                coll.add(seg)
                scaleInput = scales.createInput(
                    coll, basePoint, adsk.core.ValueInput.createByReal(factor))
                scales.add(scaleInput)
        finally:
            self.design.activateRootComponent()

    def _heelRootBasePoint(self, designComponent, heelFace, axisDir, apex):
        # Two vertices of the heel face nearest the shaft axis -> root corners.
        verts = []
        for vtx in heelFace.vertices:
            p = vtx.geometry
            ap = adsk.core.Vector3D.create(
                p.x - apex.x, p.y - apex.y, p.z - apex.z)
            along = ap.dotProduct(axisDir)
            perp = adsk.core.Vector3D.create(
                ap.x - along * axisDir.x,
                ap.y - along * axisDir.y,
                ap.z - along * axisDir.z)
            verts.append((perp.length, p))
        verts.sort(key=lambda t: t[0])
        r0 = verts[0][1]
        r1 = verts[1][1]
        midWorld = adsk.core.Point3D.create(
            (r0.x + r1.x) / 2, (r0.y + r1.y) / 2, (r0.z + r1.z) / 2)

        faceSketch = designComponent.sketches.add(heelFace)
        faceSketch.name = 'Crown Base'
        faceSketch.isVisible = False
        local = faceSketch.modelToSketchSpace(midWorld)
        return faceSketch.sketchPoints.add(local)

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(comp):
            for sketch in comp.sketches:
                if sketch.entityToken not in seen:
                    seen.add(sketch.entityToken)
                    sketch.isVisible = False
            for plane in comp.constructionPlanes:
                if plane.entityToken not in seen:
                    seen.add(plane.entityToken)
                    plane.isLightBulbOn = False
            for axis in comp.constructionAxes:
                if axis.entityToken not in seen:
                    seen.add(axis.entityToken)
                    axis.isLightBulbOn = False
            for occ in comp.occurrences:
                walk(occ.component)

        walk(bevelComponent)
