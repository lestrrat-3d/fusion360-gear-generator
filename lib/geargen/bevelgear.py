import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (18 inputs, in display order)
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
INPUT_ID_SECTIONS = 'toothSections'

# Hand of spiral list-item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Lengthwise-crown tuning constant (default 0.5; 0 disables the crown)
_CROWN_PER_RAD = 0.5

# Pinion extra mesh-phase, in tooth-fractions (0 by default — mid-face section unrotated)
_PINION_MESH_PHASE_TEETH = 0


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
            'Plane the bottom of the driving gear sits flush against')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Center Point (selection)
        centerInput = inputs.addSelectionInput(
            INPUT_ID_CENTER, 'Center Point',
            'Point the driving bevel gear is centered on')
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        centerInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        centerInput.setSelectionLimits(1, 1)

        # 3. Parent Component (selection) -- pre-selected to root.
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Parent component for the new bevel gear pair')
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

        # 10. Enable Bore (checkbox)
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

        # 15. Mean Spiral Angle
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

        # 18. Tooth Sections
        inputs.addValueInput(
            INPUT_ID_SECTIONS, 'Tooth Sections', '',
            adsk.core.ValueInput.createByReal(8))


# ---------------------------------------------------------------------------
# Virtual spur proxy -- a fake spur `parent` so the borrowed spur tooth
# generator can run without registering Fusion user parameters.
# ---------------------------------------------------------------------------
class _Val:
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """Exposes getParameter(name).value for SpurGearInvoluteToothDesignGenerator.

    module_mm is the raw mm module; circle radii/diameters are served in
    internal cm (matching what the spur tooth generator expects)."""

    def __init__(self, module_mm, virtualTeeth):
        pressureAngle = math.radians(20)  # bevel hardcodes 20 deg

        pitchDiameter_mm = virtualTeeth * module_mm
        baseDiameter_mm = pitchDiameter_mm * math.cos(pressureAngle)
        rootDiameter_mm = pitchDiameter_mm - 2.5 * module_mm
        tipDiameter_mm = pitchDiameter_mm + 2.0 * module_mm

        self._params = {
            'Module': _Val(to_cm(module_mm)),
            'ToothNumber': _Val(virtualTeeth),
            'PressureAngle': _Val(pressureAngle),
            'PitchCircleDiameter': _Val(to_cm(pitchDiameter_mm)),
            'PitchCircleRadius': _Val(to_cm(pitchDiameter_mm / 2)),
            'BaseCircleDiameter': _Val(to_cm(baseDiameter_mm)),
            'BaseCircleRadius': _Val(to_cm(baseDiameter_mm / 2)),
            'RootCircleDiameter': _Val(to_cm(rootDiameter_mm)),
            'RootCircleRadius': _Val(to_cm(rootDiameter_mm / 2)),
            'TipCircleDiameter': _Val(to_cm(tipDiameter_mm)),
            'TipCircleRadius': _Val(to_cm(tipDiameter_mm / 2)),
            'InvoluteSteps': _Val(15),
        }
        # spur's drawTooth sets self.parent._lastToothEmbedded; absorb it harmlessly.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return self._params[name]


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- no base.Generator, no GenerationContext)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

        # stashed per-run instance state (set by _readInputs)
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0
        self._toothSpacing_cm = 0.0
        self._spiralAngle_rad = 0.0
        self._cutterRadius_cm = 0.0
        self._toothSections = 8
        self._handSign = 1  # +1 Right, -1 Left (driving gear hand)

        self._module = 1.0
        self._drivingTeeth = 31
        self._pinionTeeth = 31
        self._shaftAngle_deg = 90.0
        self._gpPlaneRef = None
        self._designOccurrence = None

    # -- error rollback (entry point calls on exception) --------------------
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # =======================================================================
    # Input reading / validation
    # =======================================================================
    def _readValue(self, inputs, inputId, units):
        """Evaluate a numeric/angle input -> internal units (cm / radians)."""
        inp = inputs.itemById(inputId)
        return self.design.unitsManager.evaluateExpression(inp.expression, units)

    def _readInputs(self, inputs):
        # ---- selections ----
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

        # ---- Module: read with '' -> raw number meaning millimetres ----
        module = self._readValue(inputs, INPUT_ID_MODULE, '')
        if module <= 0:
            raise Exception('Module must be positive')

        # ---- teeth ----
        drivingTeeth = int(round(self._readValue(inputs, INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(self._readValue(inputs, INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3 or pinionTeeth < 3:
            raise Exception('Each gear must have at least 3 teeth')

        # ---- Shaft Angle: read back in radians; range-check in degrees ----
        shaftAngle_rad = self._readValue(inputs, INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception('Shaft Angle must be between 30 and 150 degrees')

        # ---- base heights ('mm' -> already internal cm) ----
        self._drivingBaseHeight_cm = self._readValue(inputs, INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = self._readValue(inputs, INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        if self._drivingBaseHeight_cm < 0 or self._pinionBaseHeight_cm < 0:
            raise Exception('Base heights must be non-negative')

        # ---- bore ----
        (self._boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._drivingBore_cm = self._readValue(inputs, INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = self._readValue(inputs, INPUT_ID_PINION_BORE, 'mm')
        if self._drivingBore_cm < 0 or self._pinionBore_cm < 0:
            raise Exception('Bore diameters must be non-negative')

        # ---- face width / tooth spacing ----
        self._faceWidth_cm = self._readValue(inputs, INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = self._readValue(inputs, INPUT_ID_TOOTH_SPACING, 'mm')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        # ---- spiral angle: read back in radians; range-check [0,60) deg ----
        self._spiralAngle_rad = self._readValue(inputs, INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralAngle_deg = math.degrees(self._spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')

        # ---- hand of spiral ----
        handInput = inputs.itemById(INPUT_ID_HAND)
        handName = _HAND_RIGHT
        if handInput.selectedItem is not None:
            handName = handInput.selectedItem.name
        self._handSign = 1 if handName == _HAND_RIGHT else -1

        # ---- cutter radius ----
        self._cutterRadius_cm = self._readValue(inputs, INPUT_ID_CUTTER_RADIUS, 'mm')
        if self._cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        # ---- tooth sections (informational; range-checked) ----
        self._toothSections = int(round(self._readValue(inputs, INPUT_ID_SECTIONS, '')))
        if self._toothSections < 3 or self._toothSections > 20:
            raise Exception('Tooth Sections must be in [3, 20]')

        return (parentComponent, targetPlane, centerPoint,
                module, drivingTeeth, pinionTeeth, shaftAngle_deg)

    # =======================================================================
    # Small geometry helpers
    # =======================================================================
    def _assertFullyConstrained(self, sketch):
        if not sketch.isFullyConstrained:
            raise Exception(f'Sketch "{sketch.name}" is not fully constrained')

    def _pointWorldGeometry(self, point):
        """World Point3D for a SketchPoint (.worldGeometry) / ConstructionPoint (.geometry)."""
        if point.objectType == adsk.fusion.SketchPoint.classType():
            return point.worldGeometry
        return point.geometry

    def _combine(self, base, a, e1, b=None, e2=None):
        """World point = base + a*e1 (+ b*e2). Lengths in cm; e1/e2 unit Vector3D."""
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if b is not None and e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, angleDeg):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByAngle(
            line, adsk.core.ValueInput.createByString(f'{angleDeg} deg'), refPlane)
        return comp.constructionPlanes.add(planeInput)

    def _perpToAxis(self, p, axisDir, axisOrigin):
        """Component of (p - axisOrigin) perpendicular to axisDir, as a Vector3D."""
        v = axisOrigin.vectorTo(p)
        along = v.dotProduct(axisDir)
        return adsk.core.Vector3D.create(
            v.x - along * axisDir.x,
            v.y - along * axisDir.y,
            v.z - along * axisDir.z)

    def _bottomEdgeMid(self, body, axisDir, apex):
        """Midpoint (world) of the body's farthest-along-axis straight edge."""
        best = None
        bestDist = None
        for edge in body.edges:
            geom = edge.geometry
            if geom.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                continue
            sp = geom.startPoint
            ep = geom.endPoint
            mid = adsk.core.Point3D.create(
                (sp.x + ep.x) / 2, (sp.y + ep.y) / 2, (sp.z + ep.z) / 2)
            d = apex.vectorTo(mid).dotProduct(axisDir)
            if bestDist is None or d > bestDist:
                bestDist = d
                best = mid
        return best

    def _distDim(self, sketch, lineA, lineB, value, textPoint):
        """Offset (parallel-gap) dimension between two lines, set numerically."""
        dim = sketch.sketchDimensions.addOffsetDimension(lineA, lineB, textPoint)
        dim.parameter.value = value
        return dim

    def _surfaceDistance(self, surface, worldPoint):
        """Distance from worldPoint to surface; None when unevaluable (e.g. apex)."""
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, onSurface) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return worldPoint.distanceTo(onSurface)

    def _circleIntersectNearest(self, R, Cx, Cy, r_c, refX, refY):
        """Intersect the apex circle (centre origin, radius R) with the cutter circle
        (centre (Cx,Cy), radius r_c); return the (x,y) nearest (refX, refY)."""
        d = math.hypot(Cx, Cy)
        if d == 0:
            return (R, 0.0)
        a = (R * R - r_c * r_c + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)
        ux = Cx / d
        uy = Cy / d
        px = a * ux
        py = a * uy
        ox = -uy * h
        oy = ux * h
        cand1 = (px + ox, py + oy)
        cand2 = (px - ox, py - oy)
        d1 = math.hypot(cand1[0] - refX, cand1[1] - refY)
        d2 = math.hypot(cand2[0] - refX, cand2[1] - refY)
        return cand1 if d1 <= d2 else cand2

    def _edgeMidWorld(self, ptA, ptB):
        a = self._pointWorldGeometry(ptA)
        b = self._pointWorldGeometry(ptB)
        return adsk.core.Point3D.create(
            (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)

    # =======================================================================
    # Pinion mesh phase
    # =======================================================================
    def _pinionMeshPhase(self):
        """Pinion's extra mesh rotation about its own shaft axis (radians).
        0 for straight bevels and for spiral by default (mid-face section unrotated)."""
        if self._spiralAngle_rad <= 0 or _PINION_MESH_PHASE_TEETH == 0:
            return 0.0
        return _PINION_MESH_PHASE_TEETH * (2 * math.pi) / self._pinionTeeth

    # =======================================================================
    # generate -- orchestration
    # =======================================================================
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint,
         module, drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        self._module = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngle_deg = shaftAngle_deg

        # ---- resolve pitch diameters & bores (Python, internal cm) ----
        drivingPitch_cm = to_cm(module * drivingTeeth)
        pinionPitch_cm = to_cm(module * pinionTeeth)

        drivingBore_cm = self._drivingBore_cm if self._drivingBore_cm > 0 else drivingPitch_cm / 4
        pinionBore_cm = self._pinionBore_cm if self._pinionBore_cm > 0 else pinionPitch_cm / 4

        # ---- build component tree: Bevel Gear -> Design + per-gear ----
        bevelOcc = parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        bevelOcc.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOcc
        bevelComponent = bevelOcc.component

        designOcc = bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        designComponent = designOcc.component
        self._designOccurrence = designOcc

        # ---- §1: Anchor Sketch ----
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # ---- §2 + §3 + per-gear bodies ----
        self._buildGearProfiles(
            designComponent, bevelComponent, targetPlane, anchorLine,
            module, drivingTeeth, pinionTeeth, math.radians(shaftAngle_deg),
            drivingPitch_cm, pinionPitch_cm, drivingBore_cm, pinionBore_cm)

        # ---- Cleanup ----
        self._hideConstructionGeometry(bevelComponent)

    # =======================================================================
    # §1: Anchor Sketch
    # =======================================================================
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        cg = projectedCenter.geometry
        lines = sketch.sketchCurves.sketchLines
        p0 = adsk.core.Point3D.create(cg.x - 0.5, cg.y, cg.z)
        p1 = adsk.core.Point3D.create(cg.x + 0.5, cg.y, cg.z)
        anchorLine = lines.addByTwoPoints(p0, p1)

        constraints = sketch.geometricConstraints
        constraints.addCoincident(projectedCenter, anchorLine)   # intersection
        constraints.addMidPoint(projectedCenter, anchorLine)     # bisects

        dims = sketch.sketchDimensions
        dims.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + 0.3, cg.z)
        ).parameter.value = to_cm(10)

        constraints.addHorizontal(anchorLine)  # sketch-local direction lock

        self._assertFullyConstrained(sketch)
        return anchorLine

    # =======================================================================
    # §2 + §3: Gear Profiles, virtual spur teeth, per-gear bodies
    # =======================================================================
    def _buildGearProfiles(self, designComponent, bevelComponent, targetPlane,
                           anchorLine, module, drivingTeeth, pinionTeeth,
                           shaftAngle_rad, drivingPitch_cm, pinionPitch_cm,
                           drivingBore_cm, pinionBore_cm):
        # ---- gear-profiles plane: 90 deg to targetPlane through the anchor line ----
        gpPlane = self._planeByAngle(designComponent, anchorLine, targetPlane, 90)
        self._gpPlaneRef = gpPlane
        sketch = designComponent.sketches.add(gpPlane)
        sketch.name = 'Gear Profiles'
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # ---- project the anchor-sketch center point ----
        projected = sketch.project(self._anchorCenterPoint)
        projectedCenter = projected.item(0)
        c = projectedCenter.geometry

        # ---- projected anchor-line 2-D unit direction ----
        projAnchor = sketch.project(anchorLine)
        projAnchorLine = projAnchor.item(0)
        aS = projAnchorLine.startSketchPoint.geometry
        aE = projAnchorLine.endSketchPoint.geometry
        dlen = math.hypot(aE.x - aS.x, aE.y - aS.y)
        d = ((aE.x - aS.x) / dlen, (aE.y - aS.y) / dlen)
        perp = (-d[1], d[0])

        # ---- pick perp sign by the target-plane normal (toward the normal) ----
        targetNormal = get_normal(targetPlane)
        targetNormal.normalize()
        perpWorldA = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perp[0], c.y + perp[1], 0))
        perpWorldOrigin = sketch.sketchToModelSpace(adsk.core.Point3D.create(c.x, c.y, 0))
        perpWorldVec = perpWorldOrigin.vectorTo(perpWorldA)
        if perpWorldVec.dotProduct(targetNormal) < 0:
            perp = (-perp[0], -perp[1])

        DPD = drivingPitch_cm
        PPD = pinionPitch_cm
        DPR = DPD / 2
        PPR = PPD / 2

        # ---- cone geometry seeds (closed form) ----
        sigma = shaftAngle_rad
        gamma_p = math.atan2(math.sin(sigma) * PPD, (DPD + PPD * math.cos(sigma)))
        gamma_g = sigma - gamma_p
        R_cone = PPR / math.sin(gamma_p)
        lenA = R_cone * math.cos(gamma_p)   # |Apex->A|
        lenB = R_cone * math.cos(gamma_g)   # |Apex->B|
        self._gamma_p = gamma_p
        self._gamma_g = gamma_g

        def sk(px, py):
            return adsk.core.Point3D.create(px, py, 0)

        def unitDir(p_from, p_to):
            ux = p_to[0] - p_from[0]
            uy = p_to[1] - p_from[1]
            ul = math.hypot(ux, uy)
            return (ux / ul, uy / ul)

        # ---- Apex: center + perp*DPD (sketch-local) ----
        apex2d = (c.x + perp[0] * DPD, c.y + perp[1] * DPD)
        centerToApex = lines.addByTwoPoints(sk(c.x, c.y), sk(apex2d[0], apex2d[1]))
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)
        constraints.addPerpendicular(centerToApex, projAnchorLine)
        apexPoint = centerToApex.endSketchPoint
        apex2d = (apexPoint.geometry.x, apexPoint.geometry.y)
        apexPos = (apex2d[0], apex2d[1])

        # ---- Driving Gear Shaft Axis: from apex toward anchor line (-perp), end = B ----
        bSeed = (apex2d[0] - perp[0] * lenB, apex2d[1] - perp[1] * lenB)
        drivingShaftAxis = lines.addByTwoPoints(sk(apex2d[0], apex2d[1]), sk(bSeed[0], bSeed[1]))
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # ---- Pinion Gear Shaft Axis: driving direction rotated about apex by Sigma ----
        def rot(vx, vy, ang):
            cc = math.cos(ang)
            ss = math.sin(ang)
            return (vx * cc - vy * ss, vx * ss + vy * cc)

        ddir = (-perp[0], -perp[1])  # apex -> B direction
        candPlus = rot(ddir[0], ddir[1], sigma)
        candMinus = rot(ddir[0], ddir[1], -sigma)
        aPlus = (apex2d[0] + candPlus[0] * lenA, apex2d[1] + candPlus[1] * lenA)
        aMinus = (apex2d[0] + candMinus[0] * lenA, apex2d[1] + candMinus[1] * lenA)
        aSeed = aPlus if aPlus[0] >= aMinus[0] else aMinus  # keep greater-X candidate
        pinionShaftAxis = lines.addByTwoPoints(sk(apex2d[0], apex2d[1]), sk(aSeed[0], aSeed[1]))
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        pointA = pinionShaftAxis.endSketchPoint
        midDir = rot(ddir[0], ddir[1], sigma / 2.0)
        angTextLen = lenA * 0.5
        angText = sk(apex2d[0] + midDir[0] * angTextLen, apex2d[1] + midDir[1] * angTextLen)
        dims.addAngularDimension(drivingShaftAxis, pinionShaftAxis, angText).parameter.value = sigma

        ag = pointA.geometry
        bg = pointB.geometry

        def perpDropSeed(fromPt, length):
            # two perpendiculars to (apex->fromPt); choose the one pointing toward center c
            sdx, sdy = unitDir(apexPos, fromPt)
            n1 = (-sdy, sdx)
            n2 = (sdy, -sdx)
            toC = (c.x - fromPt[0], c.y - fromPt[1])
            n = n1 if (n1[0] * toC[0] + n1[1] * toC[1]) >= (n2[0] * toC[0] + n2[1] * toC[1]) else n2
            return (fromPt[0] + n[0] * length, fromPt[1] + n[1] * length)

        # ---- A->Apex2: perpendicular drop from A of length PPD/2 ----
        aPos = (ag.x, ag.y)
        a2Seed = perpDropSeed(aPos, PPR)
        aToApex2 = lines.addByTwoPoints(sk(aPos[0], aPos[1]), sk(a2Seed[0], a2Seed[1]))
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaftAxis)
        dims.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sk((aPos[0] + a2Seed[0]) / 2, (aPos[1] + a2Seed[1]) / 2)).parameter.value = PPR

        # ---- B->Apex2: perpendicular drop from B of length DPD/2 ----
        bPos = (bg.x, bg.y)
        b2Seed = perpDropSeed(bPos, DPR)
        bToApex2 = lines.addByTwoPoints(sk(bPos[0], bPos[1]), sk(b2Seed[0], b2Seed[1]))
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaftAxis)
        dims.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sk((bPos[0] + b2Seed[0]) / 2, (bPos[1] + b2Seed[1]) / 2)).parameter.value = DPR

        # ---- Apex 2: coincide the two drop ends ----
        constraints.addCoincident(aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint
        ap2g = apex2Point.geometry

        # ---- Pitch Line: Apex -> Apex 2 ----
        pitchLine = lines.addByTwoPoints(sk(apex2d[0], apex2d[1]), sk(ap2g.x, ap2g.y))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # ---- dedendum lines from Apex2, perpendicular to the Pitch Line ----
        ded_cm = to_cm(1.25 * module)
        pdx, pdy = unitDir((apex2d[0], apex2d[1]), (ap2g.x, ap2g.y))
        pn1 = (-pdy, pdx)
        pn2 = (pdy, -pdx)
        # pinion dedendum (C) drawn AWAY from anchor line (+perp side); driving (D) toward it.
        if pn1[0] * perp[0] + pn1[1] * perp[1] >= 0:
            cDir, dDir = pn1, pn2
        else:
            cDir, dDir = pn2, pn1
        cSeed = (ap2g.x + cDir[0] * ded_cm, ap2g.y + cDir[1] * ded_cm)
        dSeed = (ap2g.x + dDir[0] * ded_cm, ap2g.y + dDir[1] * ded_cm)

        pinionDedendum = lines.addByTwoPoints(sk(ap2g.x, ap2g.y), sk(cSeed[0], cSeed[1]))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dims.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sk((ap2g.x + cSeed[0]) / 2, (ap2g.y + cSeed[1]) / 2)).parameter.value = ded_cm
        pointC = pinionDedendum.endSketchPoint

        drivingDedendum = lines.addByTwoPoints(sk(ap2g.x, ap2g.y), sk(dSeed[0], dSeed[1]))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dims.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sk((ap2g.x + dSeed[0]) / 2, (ap2g.y + dSeed[1]) / 2)).parameter.value = ded_cm
        pointD = drivingDedendum.endSketchPoint

        cg2 = pointC.geometry
        dg2 = pointD.geometry

        # ---- Root Axes: Apex -> C, Apex -> D ----
        pinionRootAxis = lines.addByTwoPoints(sk(apex2d[0], apex2d[1]), sk(cg2.x, cg2.y))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        drivingRootAxis = lines.addByTwoPoints(sk(apex2d[0], apex2d[1]), sk(dg2.x, dg2.y))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        module_cm = to_cm(module)

        def collinearExtend(refLine, fromPoint, alongDirGuess):
            fp = fromPoint.geometry
            endSeed = (fp.x + alongDirGuess[0] * module_cm, fp.y + alongDirGuess[1] * module_cm)
            ln = lines.addByTwoPoints(sk(fp.x, fp.y), sk(endSeed[0], endSeed[1]))
            ln.isConstruction = True
            constraints.addCoincident(ln.startSketchPoint, fromPoint)
            constraints.addCollinear(ln, refLine)
            return ln

        dirApexA = unitDir(apexPos, (ag.x, ag.y))
        dirApexB = unitDir(apexPos, (bg.x, bg.y))

        # ---- A->E (collinear with Apex->A), length module ----
        lineAE = collinearExtend(pinionShaftAxis, pointA, dirApexA)
        pointE = lineAE.endSketchPoint

        # ---- C->E perpendicular to A->E ----
        eg = pointE.geometry
        lineCE = lines.addByTwoPoints(sk(cg2.x, cg2.y), sk(eg.x, eg.y))
        lineCE.isConstruction = True
        constraints.addCoincident(lineCE.startSketchPoint, pointC)
        constraints.addCoincident(lineCE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, lineCE)

        # ---- B->F (collinear with Apex->B), length module ----
        lineBF = collinearExtend(drivingShaftAxis, pointB, dirApexB)
        pointF = lineBF.endSketchPoint

        fg = pointF.geometry
        lineDF = lines.addByTwoPoints(sk(dg2.x, dg2.y), sk(fg.x, fg.y))
        lineDF.isConstruction = True
        constraints.addCoincident(lineDF.startSketchPoint, pointD)
        constraints.addCoincident(lineDF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, lineDF)

        # ---- E->G (collinear with A->E), length module ----
        lineEG = collinearExtend(lineAE, pointE, dirApexA)
        pointG = lineEG.endSketchPoint

        # ---- C->H, length module, collinear with Apex2->C (pinionDedendum) ----
        cToHDir = unitDir((ap2g.x, ap2g.y), (cg2.x, cg2.y))
        lineCH = collinearExtend(pinionDedendum, pointC, cToHDir)
        pointH = lineCH.endSketchPoint

        # ---- G->H ----
        gg = pointG.geometry
        hg = pointH.geometry
        lineGH = lines.addByTwoPoints(sk(gg.x, gg.y), sk(hg.x, hg.y))
        lineGH.isConstruction = True
        constraints.addCoincident(lineGH.startSketchPoint, pointG)
        constraints.addCoincident(lineGH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, lineGH)

        # ---- F->I (collinear with B->F), length module ----
        lineFI = collinearExtend(lineBF, pointF, dirApexB)
        pointI = lineFI.endSketchPoint

        # ---- D->J, length module, collinear with Apex2->D (drivingDedendum) ----
        dToJDir = unitDir((ap2g.x, ap2g.y), (dg2.x, dg2.y))
        lineDJ = collinearExtend(drivingDedendum, pointD, dToJDir)
        pointJ = lineDJ.endSketchPoint

        # ---- I->J ----
        ig = pointI.geometry
        jg = pointJ.geometry
        lineIJ = lines.addByTwoPoints(sk(ig.x, ig.y), sk(jg.x, jg.y))
        lineIJ.isConstruction = True
        constraints.addCoincident(lineIJ.startSketchPoint, pointI)
        constraints.addCoincident(lineIJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, lineIJ)

        # ---- offset dim: B->Apex2 drop vs J->I = driving base height ----
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(module * drivingTeeth / 8)
        self._distDim(sketch, bToApex2, lineIJ, drivingBaseHeight_cm,
                      sk((ig.x + jg.x) / 2, (ig.y + jg.y) / 2 + 0.1))

        # ---- offset dim: A->Apex2 drop vs G->H = pinion base height ----
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (pinionTeeth / drivingTeeth)
        self._distDim(sketch, aToApex2, lineGH, pinionBaseHeight_cm,
                      sk((gg.x + hg.x) / 2, (gg.y + hg.y) / 2 + 0.1))

        # ---- A->G line ----
        lineAG = lines.addByTwoPoints(sk(ag.x, ag.y), sk(gg.x, gg.y))
        lineAG.isConstruction = True
        constraints.addCoincident(lineAG.startSketchPoint, pointA)
        constraints.addCoincident(lineAG.endSketchPoint, pointG)

        # ---- Constrain Point I with center ----
        constraints.addCoincident(pointI, projectedCenter)

        # ---- K: from G along Apex->A, pinned by two point-on-line coincidents ----
        kSeed = (gg.x + dirApexA[0] * module_cm, gg.y + dirApexA[1] * module_cm)
        lineGK = lines.addByTwoPoints(sk(gg.x, gg.y), sk(kSeed[0], kSeed[1]))
        lineGK.isConstruction = True
        constraints.addCoincident(lineGK.startSketchPoint, pointG)
        pointK = lineGK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)   # on Apex->A
        constraints.addCoincident(pointK, pinionDedendum)    # on Pinion Dedendum (Apex2->C ext.)
        kg = pointK.geometry
        lineCK = lines.addByTwoPoints(sk(cg2.x, cg2.y), sk(kg.x, kg.y))
        lineCK.isConstruction = True
        constraints.addCoincident(lineCK.startSketchPoint, pointC)
        constraints.addCoincident(lineCK.endSketchPoint, pointK)

        # ---- K' tooth center (Tooth Spacing offset) ----
        cToKDir = unitDir((cg2.x, cg2.y), (kg.x, kg.y))
        if self._toothSpacing_cm > 0:
            kpSeed = (kg.x + cToKDir[0] * self._toothSpacing_cm,
                      kg.y + cToKDir[1] * self._toothSpacing_cm)
            lineKKp = lines.addByTwoPoints(sk(kg.x, kg.y), sk(kpSeed[0], kpSeed[1]))
            lineKKp.isConstruction = True
            constraints.addCoincident(lineKKp.startSketchPoint, pointK)
            pointKp = lineKKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            dims.addDistanceDimension(
                lineKKp.startSketchPoint, lineKKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                sk((kg.x + kpSeed[0]) / 2, (kg.y + kpSeed[1]) / 2)
            ).parameter.value = self._toothSpacing_cm
            kpg = pointKp.geometry
            lineCKp = lines.addByTwoPoints(sk(cg2.x, cg2.y), sk(kpg.x, kpg.y))
            lineCKp.isConstruction = True
            constraints.addCoincident(lineCKp.startSketchPoint, pointC)
            constraints.addCoincident(lineCKp.endSketchPoint, pointKp)
        else:
            pointKp = pointK
            lineCKp = lineCK

        # ---- Maximum Face Width from SOLVED geometry; cap Face Width ----
        faceWidth_cm = self._resolveFaceWidth(pointA, pointB, pointC, pointD, pointH, pointJ)

        # ---- M->N (pinion toe line) ----
        pA = pointA.geometry
        pC = pointC.geometry
        pH = pointH.geometry
        mSeed = ((apex2d[0] + pC.x) / 2, (apex2d[1] + pC.y) / 2)
        chDir = unitDir((pC.x, pC.y), (pH.x, pH.y))
        distMtoA = math.hypot(pA.x - mSeed[0], pA.y - mSeed[1])
        nSeed = (mSeed[0] + chDir[0] * distMtoA, mSeed[1] + chDir[1] * distMtoA)
        lineMN = lines.addByTwoPoints(sk(mSeed[0], mSeed[1]), sk(nSeed[0], nSeed[1]))
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)   # M on Apex->C root axis
        constraints.addCoincident(pointN, aToApex2)          # N on A->Apex2 drop
        constraints.addParallel(lineMN, lineCH)
        self._distDim(sketch, lineCH, lineMN, faceWidth_cm,
                      sk((mSeed[0] + nSeed[0]) / 2, (mSeed[1] + nSeed[1]) / 2))
        mg = pointM.geometry
        ng = pointN.geometry
        lineMC = lines.addByTwoPoints(sk(mg.x, mg.y), sk(pC.x, pC.y))
        lineMC.isConstruction = True
        constraints.addCoincident(lineMC.startSketchPoint, pointM)
        constraints.addCoincident(lineMC.endSketchPoint, pointC)
        lineNA = lines.addByTwoPoints(sk(ng.x, ng.y), sk(pA.x, pA.y))
        lineNA.isConstruction = True
        constraints.addCoincident(lineNA.startSketchPoint, pointN)
        constraints.addCoincident(lineNA.endSketchPoint, pointA)

        # ---- L: from I along Apex->B, pinned by two point-on-line coincidents ----
        lSeed = (ig.x + dirApexB[0] * module_cm, ig.y + dirApexB[1] * module_cm)
        lineIL = lines.addByTwoPoints(sk(ig.x, ig.y), sk(lSeed[0], lSeed[1]))
        lineIL.isConstruction = True
        constraints.addCoincident(lineIL.startSketchPoint, pointI)
        pointL = lineIL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)   # on Apex->B
        constraints.addCoincident(pointL, drivingDedendum)    # on Driving Dedendum (Apex2->D ext.)
        lg = pointL.geometry
        lineDL = lines.addByTwoPoints(sk(dg2.x, dg2.y), sk(lg.x, lg.y))
        lineDL.isConstruction = True
        constraints.addCoincident(lineDL.startSketchPoint, pointD)
        constraints.addCoincident(lineDL.endSketchPoint, pointL)

        # ---- L' driving tooth center ----
        dToLDir = unitDir((dg2.x, dg2.y), (lg.x, lg.y))
        if self._toothSpacing_cm > 0:
            lpSeed = (lg.x + dToLDir[0] * self._toothSpacing_cm,
                      lg.y + dToLDir[1] * self._toothSpacing_cm)
            lineLLp = lines.addByTwoPoints(sk(lg.x, lg.y), sk(lpSeed[0], lpSeed[1]))
            lineLLp.isConstruction = True
            constraints.addCoincident(lineLLp.startSketchPoint, pointL)
            pointLp = lineLLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            dims.addDistanceDimension(
                lineLLp.startSketchPoint, lineLLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                sk((lg.x + lpSeed[0]) / 2, (lg.y + lpSeed[1]) / 2)
            ).parameter.value = self._toothSpacing_cm
            lpg = pointLp.geometry
            lineDLp = lines.addByTwoPoints(sk(dg2.x, dg2.y), sk(lpg.x, lpg.y))
            lineDLp.isConstruction = True
            constraints.addCoincident(lineDLp.startSketchPoint, pointD)
            constraints.addCoincident(lineDLp.endSketchPoint, pointLp)
        else:
            pointLp = pointL
            lineDLp = lineDL

        # ---- O->P (driving toe line, mirror of M->N) ----
        pB = pointB.geometry
        pD = pointD.geometry
        pJ = pointJ.geometry
        oSeed = ((apex2d[0] + pD.x) / 2, (apex2d[1] + pD.y) / 2)
        djDir = unitDir((pD.x, pD.y), (pJ.x, pJ.y))
        distOtoB = math.hypot(pB.x - oSeed[0], pB.y - oSeed[1])
        pSeed = (oSeed[0] + djDir[0] * distOtoB, oSeed[1] + djDir[1] * distOtoB)
        lineOP = lines.addByTwoPoints(sk(oSeed[0], oSeed[1]), sk(pSeed[0], pSeed[1]))
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)   # O on Apex->D
        constraints.addCoincident(pointP, bToApex2)          # P on B->Apex2 drop
        constraints.addParallel(lineOP, lineDJ)
        self._distDim(sketch, lineDJ, lineOP, faceWidth_cm,
                      sk((oSeed[0] + pSeed[0]) / 2, (oSeed[1] + pSeed[1]) / 2))
        og = pointO.geometry
        pg = pointP.geometry
        lineOD = lines.addByTwoPoints(sk(og.x, og.y), sk(pD.x, pD.y))
        lineOD.isConstruction = True
        constraints.addCoincident(lineOD.startSketchPoint, pointO)
        constraints.addCoincident(lineOD.endSketchPoint, pointD)
        linePB = lines.addByTwoPoints(sk(pg.x, pg.y), sk(pB.x, pB.y))
        linePB.isConstruction = True
        constraints.addCoincident(linePB.startSketchPoint, pointP)
        constraints.addCoincident(linePB.endSketchPoint, pointB)
        lineBI = lines.addByTwoPoints(sk(pB.x, pB.y), sk(ig.x, ig.y))
        lineBI.isConstruction = True
        constraints.addCoincident(lineBI.startSketchPoint, pointB)
        constraints.addCoincident(lineBI.endSketchPoint, pointI)

        self._assertFullyConstrained(sketch)

        anchors = {
            'apexPoint': apexPoint, 'apex2Point': apex2Point,
            'A': pointA, 'B': pointB, 'C': pointC, 'D': pointD,
            'E': pointE, 'F': pointF, 'G': pointG, 'H': pointH,
            'I': pointI, 'J': pointJ, 'K': pointK, 'Kp': pointKp,
            'L': pointL, 'Lp': pointLp, 'M': pointM, 'N': pointN,
            'O': pointO, 'P': pointP,
            'lineCKp': lineCKp, 'lineDLp': lineDLp,
        }

        # ---- §3: build virtual spur profiles (pinion then driving) ----
        pinionToothInfo = self._buildVirtualSpurProfile(
            designComponent, gpPlane, module, 'Pinion',
            anchors['Kp'], anchors['lineCKp'], pinionPitch_cm, gamma_p)
        drivingToothInfo = self._buildVirtualSpurProfile(
            designComponent, gpPlane, module, 'Driving',
            anchors['Lp'], anchors['lineDLp'], drivingPitch_cm, gamma_g)

        # ---- per-gear body creation: pinion first, driving second ----
        self._createGearBody(
            designComponent, bevelComponent, anchors, gpPlane,
            'Pinion', pinionTeeth, pinionToothInfo,
            ['A', 'G', 'H', 'C', 'M', 'N'],
            ('M', 'N'), ('C', 'H'), pinionBore_cm,
            meshRotateRad=self._pinionMeshPhase())
        self._createGearBody(
            designComponent, bevelComponent, anchors, gpPlane,
            'Driving', drivingTeeth, drivingToothInfo,
            ['B', 'I', 'J', 'D', 'O', 'P'],
            ('O', 'P'), ('D', 'J'), drivingBore_cm,
            meshRotateRad=math.radians(180) / drivingTeeth)

    # -----------------------------------------------------------------------
    def _resolveFaceWidth(self, pointA, pointB, pointC, pointD, pointH, pointJ):
        """Max Face Width from SOLVED geometry; cap default / reject too-large user value."""
        def perpDistToLine(p, a, b):
            abx = b.x - a.x
            aby = b.y - a.y
            ablen = math.hypot(abx, aby)
            if ablen == 0:
                return 0.0
            return abs((p.x - a.x) * aby - (p.y - a.y) * abx) / ablen

        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distPinion = perpDistToLine(gA, gC, gH)   # A to line C->H
        distDriving = perpDistToLine(gB, gD, gJ)  # B to line D->J
        maxFaceWidth_cm = 0.95 * min(distPinion, distDriving)

        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face Width {:.4f} cm exceeds the maximum {:.4f} cm'.format(
                        self._faceWidth_cm, maxFaceWidth_cm))
            return self._faceWidth_cm
        coneDistance_cm = to_cm(math.hypot(
            self._module * self._drivingTeeth, self._module * self._pinionTeeth))
        return min(coneDistance_cm / 6.0, maxFaceWidth_cm)

    # -----------------------------------------------------------------------
    def _findSpurToothProfile(self, sketch, embedded):
        """Match the tooth cross-section loop by curve-type mix, not count alone.
        non-embedded: 2 NURBS + 2 arcs + 2 lines; embedded: 2 NURBS + 2 arcs + 0 lines."""
        wantLines = 0 if embedded else 2
        for prof in sketch.profiles:
            for loop in prof.profileLoops:
                nurbs = arcs = lns = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        lns += 1
                if nurbs == 2 and arcs == 2 and lns == wantLines:
                    return prof
        return None

    # -----------------------------------------------------------------------
    def _buildVirtualSpurProfile(self, designComponent, gpPlane, module, gearLabel,
                                 centerPoint, centerRefLine, pitch_cm, gamma):
        """§3: tooth plane + spur tooth + axis for one gear. Returns dict."""
        virtualPitchRadius_mm = (to_mm(pitch_cm) / 2) / math.cos(gamma)
        virtualTeeth = int(math.floor(2 * virtualPitchRadius_mm / module))

        toothPlane = self._planeByAngle(designComponent, centerRefLine, gpPlane, 90)
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Plane'

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        self._assertFullyConstrained(toothSketch)

        # construction axis through center point, normal to the tooth plane:
        # setByTwoPlanes(gpPlane, helper plane perpendicular to centerRefLine at K'/L').
        helperPlaneInput = designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(
            centerRefLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperPlaneInput)
        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gpPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'

        return {
            'toothPlane': toothPlane,
            'toothSketch': toothSketch,
            'toothAxis': toothAxis,
            'virtualTeeth': virtualTeeth,
            'embedded': proxy._lastToothEmbedded,
            'gearLabel': gearLabel,
        }

    # =======================================================================
    # Per-gear body creation
    # =======================================================================
    def _createGearBody(self, designComponent, bevelComponent, anchors, gpPlane,
                        gearLabel, teethNumber, toothInfo,
                        hexVertexKeys, toeEdgeKeys, heelEdgeKeys, bore_cm,
                        meshRotateRad):
        gearOcc = bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        gearOcc.component.name = f'{gearLabel} Gear'

        # ---- fresh profile sketch on the axial (Gear Profiles) plane ----
        profileSketch = designComponent.sketches.add(gpPlane)
        profileSketch.name = f'{gearLabel} Profile'
        lines = profileSketch.sketchCurves.sketchLines

        # recreate the six §2 vertices as new local points (world-mapped); share; fix after.
        verts = []
        for key in hexVertexKeys:
            srcWorld = anchors[key].worldGeometry
            local = profileSketch.modelToSketchSpace(srcWorld)
            verts.append(profileSketch.sketchPoints.add(local))

        hexLines = []
        n = len(verts)
        for i in range(n):
            hexLines.append(lines.addByTwoPoints(verts[i], verts[(i + 1) % n]))
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch)

        # first edge (vertex0 -> vertex1) is the shaft axis (A->G / B->I)
        shaftAxisEdge = hexLines[0]

        # ---- revolve the single hexagon profile about the shaft axis ----
        profile = profileSketch.profiles.item(0)
        revolves = designComponent.features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # ---- loft Apex sketch point -> tooth profile = uncut Tooth Body ----
        apexSketchPoint = anchors['apexPoint']
        toothProfile = self._findSpurToothProfile(
            toothInfo['toothSketch'], toothInfo['embedded'])
        if toothProfile is None:
            raise Exception(f'{gearLabel}: could not find spur tooth profile')
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth Body'

        # ---- world anchors needed by the tooth-body transform / cuts ----
        apexWorld = self._pointWorldGeometry(anchors['apexPoint'])
        toeMid = self._edgeMidWorld(anchors[toeEdgeKeys[0]], anchors[toeEdgeKeys[1]])
        heelMid = self._edgeMidWorld(anchors[heelEdgeKeys[0]], anchors[heelEdgeKeys[1]])
        toeConeWorld = self._pointWorldGeometry(anchors[toeEdgeKeys[0]])    # M / O
        heelConeWorld = self._pointWorldGeometry(anchors[heelEdgeKeys[0]])  # C / D

        # ---- transform the tooth body (straight trims or spiral build) ----
        finalToothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge,
            apexWorld, apexSketchPoint, toeMid, heelMid,
            toeConeWorld, heelConeWorld, toothInfo['toothPlane'],
            gearLabel, teethNumber)

        # ---- circular pattern of the tooth around the shaft axis ----
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(finalToothBody)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # ---- combine-join all tooth pieces into the gear body ----
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # ---- bore (optional) ----
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, bore_cm)

        # ---- meshing rotation about the shaft axis (world geometry) ----
        if meshRotateRad != 0:
            self._rotateBodyAboutEdge(designComponent, gearBody, shaftAxisEdge, meshRotateRad)

        # ---- relocate the finished body into the gear component ----
        gearBody.moveToComponent(gearOcc)

    # -----------------------------------------------------------------------
    def _rotateBodyAboutEdge(self, designComponent, body, edge, angleRad):
        sp = edge.worldGeometry.startPoint
        ep = edge.worldGeometry.endPoint
        axisVec = sp.vectorTo(ep)
        axisVec.normalize()
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angleRad, axisVec, sp)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moves = designComponent.features.moveFeatures
        moveInput = moves.createInput2(bodies)
        moveInput.defineAsFreeMove(matrix)
        moves.add(moveInput)

    # =======================================================================
    # Tooth-body transform hook -- psi-gated
    # =======================================================================
    def _transformToothBody(self, designComponent, toothBody, gearBody, shaftAxisEdge,
                            apexWorld, apexSketchPoint, toeMid, heelMid,
                            toeConeWorld, heelConeWorld, parentToothPlane,
                            gearLabel, teethNumber):
        # straight bevel: byte-for-byte the prior behavior.
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody,
                toeMid, heelMid, apexWorld, gearLabel)

        # ---------- A. Gate & frame ----------
        sp = shaftAxisEdge.worldGeometry.startPoint
        ep = shaftAxisEdge.worldGeometry.endPoint
        axisDir = sp.vectorTo(ep)
        axisDir.normalize()
        apex = apexWorld
        coneVec = apex.vectorTo(heelConeWorld)
        coneVec.normalize()
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)
        tpNormal.normalize()

        def distAlong(p):
            return apex.vectorTo(p).dotProduct(coneVec)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # ---------- B. Cutter-arc geometry ----------
        psi = self._spiralAngle_rad
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        handSign = self._handSign  # +1 Right, -1 Left (driving)
        if gearLabel == 'Pinion':
            handSign = -handSign   # opposite hand on the pinion

        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)   # hand sign on the cos/Cy term

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
        heel2d = self._circleIntersectNearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        # ---------- C. 2-D trace sketch (genuine cutter arc) ----------
        gpPlane = self._gpPlaneRef
        coneSketch = designComponent.sketches.add(gpPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneEndWorld = self._combine(apex, R_heel, coneVec)
        coneStartLocal = coneSketch.modelToSketchSpace(apex)
        coneEndLocal = coneSketch.modelToSketchSpace(coneEndWorld)
        coneLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(coneStartLocal, coneEndLocal)
        coneLine.isConstruction = True

        tangentPlane = self._planeByAngle(designComponent, coneLine, gpPlane, 90)
        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'

        traceCircles = traceSketch.sketchCurves.sketchCircles
        traceArcs = traceSketch.sketchCurves.sketchArcs
        traceConstraints = traceSketch.geometricConstraints
        traceDims = traceSketch.sketchDimensions

        centerLocal = traceSketch.modelToSketchSpace(tanW(Cx, Cy))
        cutterCircle = traceCircles.addByCenterRadius(centerLocal, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        traceDims.addDiameterDimension(
            cutterCircle, traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy)))

        toeLocal = traceSketch.modelToSketchSpace(tanW(toe2d[0], toe2d[1]))
        meanLocal = traceSketch.modelToSketchSpace(tanW(R_mean, 0.0))
        heelLocal = traceSketch.modelToSketchSpace(tanW(heel2d[0], heel2d[1]))
        traceArc = traceArcs.addByThreePoints(toeLocal, meanLocal, heelLocal)
        traceConstraints.addCoincident(
            traceArc.centerSketchPoint, cutterCircle.centerSketchPoint)
        traceDims.addRadialDimension(
            traceArc, traceSketch.modelToSketchSpace(tanW(Cx, Cy + r_c * 0.5)))
        # deliberately left with free DOF -- exempt from the full-constraint gate.

        # ---------- D. 3-D trace (project onto the root cone) ----------
        dedMidWorld = adsk.core.Point3D.create(
            (toeConeWorld.x + heelConeWorld.x) / 2,
            (toeConeWorld.y + heelConeWorld.y) / 2,
            (toeConeWorld.z + heelConeWorld.z) / 2)
        rootFace = self._findRootConeFace(gearBody, dedMidWorld)

        projDirSketch = designComponent.sketches.add(gpPlane)
        projDirSketch.name = f'{gearLabel} Proj Dir'
        dirLine = projDirSketch.sketchCurves.sketchLines.addByTwoPoints(
            projDirSketch.modelToSketchSpace(apex),
            projDirSketch.modelToSketchSpace(self._combine(apex, 1.0, tpNormal)))
        dirLine.isConstruction = True

        proj3dSketch = designComponent.sketches.add(gpPlane)
        proj3dSketch.name = f'{gearLabel} 3D Tooth Trace'
        # faces/curves MUST be plain Python lists (C++ vector<>s), not ObjectCollections.
        projected = proj3dSketch.projectToSurface(
            [rootFace], [traceArc],
            adsk.fusion.SurfaceProjectTypes.AlongVectorSurfaceProjectType, dirLine)
        # projectToSurface returns a SketchEntityVector: size with len(...), index with [i].
        if len(projected) == 0:
            raise Exception(f'{gearLabel}: 3D trace projection produced no curve')
        trace3d = projected[0]
        samplePts = self._sampleCurveWorld(trace3d, 24)

        # ---------- E. Slice the straight tooth ----------
        segments = self._sliceTooth(
            designComponent, toothBody, parentToothPlane, apex, span)

        # ---------- F. Order & drop scrap ----------
        segments.sort(key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)

        # ---------- G. Twist (the spiral) ----------
        # Analytic crown-gear law: total = |phi_crown| / sin(pitch cone angle). Robust, no projection.
        _gam = self._gamma_p if gearLabel == 'Pinion' else self._gamma_g
        total = abs(math.atan2(heel2d[1], heel2d[0])
                    - math.atan2(toe2d[1], toe2d[0])) / math.sin(_gam)

        def heelFaceCenter(seg):
            return self._farthestFace(seg, apex, coneVec).centroid

        segAngles = []
        for seg in segments:
            R_heelFace = distAlong(heelFaceCenter(seg))
            ang = -handSign * total * (R_mean - R_heelFace) / span
            segAngles.append(ang)
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apex)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moveInput = designComponent.features.moveFeatures.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            designComponent.features.moveFeatures.add(moveInput)

        # ---------- H. Lengthwise crown (relief) ----------
        order = sorted(range(len(segments)),
                       key=lambda i: distAlong(heelFaceCenter(segments[i])))
        heelIdx = order[-1]
        self._activateDesign()
        try:
            for i in order:
                if i == heelIdx:
                    continue  # skip the outermost (heel) segment
                scale = 1 - _CROWN_PER_RAD * abs(segAngles[i])
                if scale <= 0 or scale == 1:
                    continue
                self._crownScale(designComponent, segments[i], apex, coneVec, scale)
        finally:
            self._restoreActive()

        # ---------- I. Loft -> curved tooth ----------
        toeSeg = segments[order[0]]
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(self._apexSideFace(toeSeg, apex, coneVec))
        for i in order:
            loftInput.loftSections.add(self._farthestFace(segments[i], apex, coneVec))
        loft = lofts.add(loftInput)
        curvedTooth = loft.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        for seg in segments:
            try:
                designComponent.features.removeFeatures.add(seg)
            except RuntimeError:
                pass

        # ---------- J. Flush trim ----------
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody,
            toeMid, heelMid, apexWorld, gearLabel)

    # -----------------------------------------------------------------------
    # Spiral helper geometry
    # -----------------------------------------------------------------------
    def _sampleCurveWorld(self, sketchEntity, count):
        geom = sketchEntity.worldGeometry
        evaluator = geom.evaluator
        (ok, startParam, endParam) = evaluator.getParameterExtents()
        pts = []
        if not ok or count < 2:
            return pts
        for i in range(count):
            t = startParam + (endParam - startParam) * (i / (count - 1))
            (ok2, p) = evaluator.getPointAtParameter(t)
            if ok2:
                pts.append(p)
        return pts

    def _twistAzimuth(self, samplePts, axisDir, apex):
        if len(samplePts) < 2:
            return 0.0
        first = self._perpToAxis(samplePts[0], axisDir, apex)
        last = self._perpToAxis(samplePts[-1], axisDir, apex)
        if first.length == 0 or last.length == 0:
            return 0.0
        first.normalize()
        last.normalize()
        dot = max(-1.0, min(1.0, first.dotProduct(last)))
        return abs(math.acos(dot))

    def _sliceTooth(self, designComponent, toothBody, parentToothPlane, apex, span):
        planeOrigin = parentToothPlane.geometry.origin
        planeNormal = parentToothPlane.geometry.normal
        planeNormal.normalize()
        toApex = planeOrigin.vectorTo(apex)
        sign = 1.0 if toApex.dotProduct(planeNormal) > 0 else -1.0

        step = span / 6.0
        cutPlanes = []
        for k in range(8):
            offset = sign * (k + 1) * step
            planeInput = designComponent.constructionPlanes.createInput()
            planeInput.setByOffset(
                parentToothPlane, adsk.core.ValueInput.createByReal(offset))
            cutPlanes.append(designComponent.constructionPlanes.add(planeInput))

        pieces = [toothBody]
        splits = designComponent.features.splitBodyFeatures
        for plane in cutPlanes:
            newPieces = []
            for piece in pieces:
                try:
                    splitInput = splits.createInput(piece, plane, True)
                    split = splits.add(splitInput)
                    for b in split.bodies:
                        newPieces.append(b)
                except RuntimeError:
                    newPieces.append(piece)
            pieces = newPieces
        return pieces

    def _farthestFace(self, body, apex, coneVec):
        best = None
        bestDist = None
        for face in body.faces:
            c = face.centroid
            d = apex.vectorTo(c).dotProduct(coneVec)
            if bestDist is None or d > bestDist:
                bestDist = d
                best = face
        return best

    def _apexSideFace(self, body, apex, coneVec):
        best = None
        bestDist = None
        for face in body.faces:
            c = face.centroid
            d = apex.vectorTo(c).dotProduct(coneVec)
            if bestDist is None or d < bestDist:
                bestDist = d
                best = face
        return best

    def _crownScale(self, designComponent, body, apex, coneVec, scale):
        heelFace = self._farthestFace(body, apex, coneVec)
        # base must be a sketch point on the heel face (setByPoint raises here).
        baseSketch = designComponent.sketches.add(heelFace)
        basePoint = baseSketch.sketchPoints.add(
            baseSketch.modelToSketchSpace(heelFace.centroid))

        inputColl = adsk.core.ObjectCollection.create()
        inputColl.add(body)
        scales = designComponent.features.scaleFeatures
        scaleInput = scales.createInput(
            inputColl, basePoint, adsk.core.ValueInput.createByReal(scale))
        scales.add(scaleInput)

    def _activateDesign(self):
        # scaleFeatures needs the Design occurrence to be the active edit target.
        try:
            self._designOccurrence.activate()
        except Exception:
            pass

    def _restoreActive(self):
        try:
            self.design.rootComponent.occurrences  # restore root edit target
            self.design.activateRootComponent()
        except Exception:
            pass

    def _findRootConeFace(self, gearBody, dedMidWorld):
        best = None
        bestDist = None
        for face in gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            d = self._surfaceDistance(face.geometry, dedMidWorld)
            if d is None:
                continue
            if bestDist is None or d < bestDist:
                bestDist = d
                best = face
        if best is None:
            raise Exception('Could not find root cone face for spiral projection')
        return best

    # =======================================================================
    # Conical end trims (straight tooth + spiral flush) and bore
    # =======================================================================
    def _cutConicalEnds(self, designComponent, toothBody, gearBody,
                        toeMid, heelMid, apexWorld, gearLabel):
        keeper = self._applyConicalCut(
            designComponent, [toothBody], gearBody, toeMid, apexWorld,
            gearLabel, 'toe', dropApex=True, mayBeNoOp=False)
        return self._applyConicalCut(
            designComponent, [keeper], gearBody, heelMid, apexWorld,
            gearLabel, 'heel', dropApex=False, mayBeNoOp=True)

    def _findConeFaceForCutLine(self, gearBody, edgeMidWorld):
        """The ConeSurfaceType faces of the frustum, ordered best-first by surface
        distance to the cut-edge midpoint (unevaluable -> +inf, tried last)."""
        coneFaces = []
        for face in gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            d = self._surfaceDistance(face.geometry, edgeMidWorld)
            coneFaces.append((d, face))
        coneFaces.sort(key=lambda item: float('inf') if item[0] is None else item[0])
        return coneFaces

    def _applyConicalCut(self, designComponent, pieces, gearBody, edgeMidWorld,
                         apexWorld, gearLabel, cutName, dropApex, mayBeNoOp):
        coneFaces = self._findConeFaceForCutLine(gearBody, edgeMidWorld)
        diagnostics = [d for (d, _f) in coneFaces]

        splits = designComponent.features.splitBodyFeatures
        resultPieces = None
        for (d, face) in coneFaces:
            allPieces = []
            anySplit = False
            for piece in pieces:
                try:
                    splitInput = splits.createInput(piece, face, True)
                    split = splits.add(splitInput)
                    if split.bodies.count > 1:
                        anySplit = True
                        for b in split.bodies:
                            allPieces.append(b)
                    else:
                        allPieces.append(piece)
                except RuntimeError:
                    allPieces.append(piece)
            if anySplit:
                resultPieces = allPieces
                break

        if resultPieces is None:
            if mayBeNoOp:
                futil.log(
                    f'{gearLabel}: {cutName} cut tool did not intersect, kept intact',
                    force_console=True)
                return pieces[0]
            raise Exception(
                f'{gearLabel}: {cutName} cut produced no split '
                f'(cone face surface-distances: {diagnostics})')

        futil.log(
            f'{gearLabel}: {cutName} cut produced {len(resultPieces)} pieces '
            f'(cone face dists: {diagnostics})',
            force_console=True)

        # keeper selection: drop apex-containing pieces, keep largest by volume.
        nonApex = []
        for piece in resultPieces:
            containment = piece.pointContainment(apexWorld)
            isApex = (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                      or containment == adsk.fusion.PointContainment.PointOnPointContainment)
            if dropApex and isApex:
                designComponent.features.removeFeatures.add(piece)
                continue
            nonApex.append(piece)
        if len(nonApex) == 0:
            raise Exception(f'{gearLabel}: {cutName} cut left no non-apex piece')

        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for extra in nonApex[1:]:
            try:
                designComponent.features.removeFeatures.add(extra)
            except RuntimeError:
                pass
        return keeper

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, bore_cm):
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearBody.name} Bore'
        circles = boreSketch.sketchCurves.sketchCircles
        radius = bore_cm / 2
        circle = circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0)).parameter.value = bore_cm
        self._assertFullyConstrained(boreSketch)

        profile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(100.0), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # =======================================================================
    # Cleanup
    # =======================================================================
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            for sketch in component.sketches:
                if sketch.entityToken not in seen:
                    seen.add(sketch.entityToken)
                    sketch.isVisible = False
                    sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                if plane.entityToken not in seen:
                    seen.add(plane.entityToken)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                if axis.entityToken not in seen:
                    seen.add(axis.entityToken)
                    axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
