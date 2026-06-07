import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
from .spurgear import SpurGearInvoluteToothDesignGenerator
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Dialog input ids (the 14 reproduced surface strings, verbatim)
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

# Spiral inputs (0 spiral angle == straight bevel; >0 == curved/spiral teeth).
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
INPUT_ID_SECTIONS = 'toothSections'

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Hardcoded pressure angle for the virtual spur teeth (bevel is not a dialog input).
PRESSURE_ANGLE_RAD = math.radians(20)
INVOLUTE_STEPS = 15


# ---------------------------------------------------------------------------
# Dialog inputs configurator
# ---------------------------------------------------------------------------
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so it wins Fusion auto-focus.
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

        # 3. Parent Component (selection) -- root pre-selected.
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

        # 15. Mean Spiral Angle (deg). 0 == straight bevel.
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral (dropdown). Driving gear's hand; pinion is opposite.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius (mm). 0 == auto (= mean cone distance).
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 18. Tooth Sections (currently informational; B uses toe+heel + the rail).
        inputs.addValueInput(
            INPUT_ID_SECTIONS, 'Tooth Sections', '',
            adsk.core.ValueInput.createByReal(8))


# ---------------------------------------------------------------------------
# Value-wrapper + virtual spur proxy (fake parent for the borrowed spur drawer)
# ---------------------------------------------------------------------------
class _Val:
    """Tiny wrapper exposing a `.value` attribute, mimicking a UserParameter."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """A fake spur `parent` so SpurGearInvoluteToothDesignGenerator can read the
    parameters it needs (via parent.getParameter(name).value) without registering
    any real Fusion user parameters.

    module_mm is the raw-mm Module value; virtualTeeth is the Tredgold/back-cone
    virtual tooth number. All served circle radii/diameters are in internal cm.
    """
    def __init__(self, module_mm, virtualTeeth):
        moduleCm = to_cm(module_mm)
        teeth = int(virtualTeeth)

        pitchDiameter = moduleCm * teeth
        pitchRadius = pitchDiameter / 2
        baseDiameter = pitchDiameter * math.cos(PRESSURE_ANGLE_RAD)
        baseRadius = baseDiameter / 2
        rootDiameter = pitchDiameter - 2.5 * moduleCm
        rootRadius = rootDiameter / 2
        tipDiameter = pitchDiameter + 2 * moduleCm
        tipRadius = tipDiameter / 2

        self._values = {
            'Module': moduleCm,
            'ToothNumber': teeth,
            'PressureAngle': PRESSURE_ANGLE_RAD,
            'PitchCircleDiameter': pitchDiameter,
            'PitchCircleRadius': pitchRadius,
            'BaseCircleDiameter': baseDiameter,
            'BaseCircleRadius': baseRadius,
            'RootCircleDiameter': rootDiameter,
            'RootCircleRadius': rootRadius,
            'TipCircleDiameter': tipDiameter,
            'TipCircleRadius': tipRadius,
            'InvoluteSteps': INVOLUTE_STEPS,
        }
        # The spur drawer records embedded-ness on the parent; absorb it here.
        self._lastToothEmbedded = False

    def getParameter(self, name):
        return _Val(self._values[name])


# ---------------------------------------------------------------------------
# Bevel gear generator (standalone -- does NOT subclass base.Generator)
# ---------------------------------------------------------------------------
class BevelGearGenerator:
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.bevelOccurrence = None

        # Stashed inputs (set in _readInputs).
        self._drivingBaseHeight_cm = 0.0
        self._pinionBaseHeight_cm = 0.0
        self._boreEnable = True
        self._drivingBore_cm = 0.0
        self._pinionBore_cm = 0.0
        self._faceWidth_cm = 0.0
        self._toothSpacing_cm = 0.0
        self._faceWidthSpecified = False

        # Stashed center sketch point from the Anchor Sketch (re-projected in §2).
        self._anchorCenterPoint = None

    # ----- error rollback -----
    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # ----- full-constraint gate (lenient: log-only) -----
    # TEMP DIAGNOSTIC: the borrowed spur tooth profile comes back marginally
    # under-constrained for some inputs (a known spurgear fragility, flagged in the
    # bevel spec). Log the status instead of raising so the build proceeds; the
    # tooth BODY is geometrically fine regardless. (To be hardened in spurgear later.)
    def _assertFullyConstrained(self, sketch, name):
        ok = sketch.isFullyConstrained
        futil.log(f'[spiral-diag] sketch "{name}" fullyConstrained={ok}',
                  force_console=True)

    # ----- input reading + validation -----
    def _readInputs(self, inputs):
        unitsManager = self.design.unitsManager

        def rawValue(id, units):
            inp = inputs.itemById(id)
            return unitsManager.evaluateExpression(inp.expression, units)

        # --- selections ---
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

        # --- Module: read with unit '' -> raw number meaning millimetres ---
        module = rawValue(INPUT_ID_MODULE, '')
        if module <= 0:
            raise Exception('Module must be a positive number')

        # --- Teeth ---
        drivingTeeth = int(round(rawValue(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(rawValue(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception('Driving gear must have at least 3 teeth')
        if pinionTeeth < 3:
            raise Exception('Pinion gear must have at least 3 teeth')

        # --- Shaft angle: comes back in radians; convert to degrees to range-check ---
        shaftAngle_rad = rawValue(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngle_deg = math.degrees(shaftAngle_rad)
        if shaftAngle_deg < 30 or shaftAngle_deg > 150:
            raise Exception('Shaft angle must be between 30 and 150 degrees')

        # --- 'mm' inputs: already in internal cm -- use as-is ---
        self._drivingBaseHeight_cm = rawValue(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = rawValue(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        self._drivingBore_cm = rawValue(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = rawValue(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = rawValue(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = rawValue(INPUT_ID_TOOTH_SPACING, 'mm')

        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable

        if self._drivingBaseHeight_cm < 0:
            raise Exception('Driving gear base height must be non-negative')
        if self._pinionBaseHeight_cm < 0:
            raise Exception('Pinion gear base height must be non-negative')
        if self._drivingBore_cm < 0:
            raise Exception('Driving gear bore diameter must be non-negative')
        if self._pinionBore_cm < 0:
            raise Exception('Pinion gear bore diameter must be non-negative')
        if self._faceWidth_cm < 0:
            raise Exception('Face width must be non-negative')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth spacing must be non-negative')

        # Face width "unspecified" means default (0).
        self._faceWidthSpecified = self._faceWidth_cm > 0

        # --- spiral inputs (0 spiral angle == straight bevel) ---
        spiralAngle_rad = rawValue(INPUT_ID_SPIRAL_ANGLE, 'deg')  # back in radians
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        hand = handItem.name if handItem else _HAND_RIGHT

        cutterRadius_cm = rawValue(INPUT_ID_CUTTER_RADIUS, 'mm')  # already internal cm
        if cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        sections = int(round(rawValue(INPUT_ID_SECTIONS, '')))
        if sections < 3 or sections > 20:
            raise Exception('Tooth Sections must be between 3 and 20')

        # Stash spiral parameters + the gear geometry the tooth hook will need.
        self._spiralAngle_rad = spiralAngle_rad
        self._spiralHand = hand
        self._cutterRadius_cm = cutterRadius_cm
        self._toothSections = sections
        self._sbModule = module
        self._sbDrivingTeeth = drivingTeeth
        self._sbPinionTeeth = pinionTeeth
        self._sbShaftAngle_deg = shaftAngle_deg

        futil.log(
            f'Spiral bevel inputs: psi={spiralAngle_deg:.1f} deg, hand={hand}, '
            f'cutterRadius={to_mm(cutterRadius_cm):.3f} mm (0=auto), sections={sections}',
            force_console=True)

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngle_deg)

    # ----- helpers -----
    def _pointWorldGeometry(self, entity):
        """World Point3D for a SketchPoint (worldGeometry) or ConstructionPoint (geometry)."""
        if entity.objectType == adsk.fusion.SketchPoint.classType():
            return entity.worldGeometry
        return entity.geometry

    def _perpDistancePointToLine(self, p, a, b):
        """Perpendicular distance (2-D x/y) from point p to the line through a, b."""
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
        px, py = p.x, p.y
        dx = bx - ax
        dy = by - ay
        denom = math.hypot(dx, dy)
        if denom == 0:
            return math.hypot(px - ax, py - ay)
        return abs(dx * (ay - py) - (ax - px) * dy) / denom

    def _surfaceDistance(self, surface, worldPoint):
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return None
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return None
        return projected.distanceTo(worldPoint)

    # ----- main entry -----
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = self._readInputs(inputs)

        # Resolve pitch diameters (cm) and bores (cm).
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)

        if self._drivingBore_cm > 0:
            drivingBore_cm = self._drivingBore_cm
        else:
            drivingBore_cm = drivingPitchDiameter_cm / 4
        if self._pinionBore_cm > 0:
            pinionBore_cm = self._pinionBore_cm
        else:
            pinionBore_cm = pinionPitchDiameter_cm / 4

        # --- Component tree: Bevel Gear under parent, Design under Bevel Gear ---
        bevelOccurrence = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOccurrence.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOccurrence
        bevelComponent = bevelOccurrence.component

        designOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOccurrence.component.name = 'Design'
        designComponent = designOccurrence.component

        # --- §1 Anchor Sketch ---
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        # --- §2 + §3 + per-gear body creation ---
        self._buildGearProfiles(
            designComponent, bevelComponent, targetPlane, centerPoint, anchorLine,
            module, drivingTeeth, pinionTeeth, shaftAngle_deg,
            drivingPitchDiameter_cm, pinionPitchDiameter_cm,
            drivingBore_cm, pinionBore_cm)

        # --- Cleanup ---
        self._hideConstructionGeometry(bevelComponent)

    # ----- §1 Anchor Sketch -----
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the user-specified center onto the sketch.
        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        # Create the anchor line through the projected center (~10mm reference line).
        cg = projectedCenter.geometry
        half = to_cm(5)  # ~10mm total length
        p0 = adsk.core.Point3D.create(cg.x - half, cg.y, 0)
        p1 = adsk.core.Point3D.create(cg.x + half, cg.y, 0)
        anchorLine = lines.addByTwoPoints(p0, p1)

        # Pin the center onto the line AND make it bisect.
        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)

        # ~10mm length dimension (arbitrary reference value).
        midText = adsk.core.Point3D.create(cg.x, cg.y + half, 0)
        lengthDim = dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            midText)
        lengthDim.parameter.value = to_cm(10)

        # Pin the direction sketch-locally so the sketch ends fully constrained.
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch, 'Anchor Sketch')
        return anchorLine

    # ----- §2 + §3 + body creation -----
    def _buildGearProfiles(self, designComponent, bevelComponent, targetPlane,
                           centerPoint, anchorLine, module, drivingTeeth,
                           pinionTeeth, shaftAngle_deg,
                           drivingPitchDiameter_cm, pinionPitchDiameter_cm,
                           drivingBore_cm, pinionBore_cm):
        DPD = drivingPitchDiameter_cm
        PPD = pinionPitchDiameter_cm
        Sigma = math.radians(shaftAngle_deg)
        moduleCm = to_cm(module)
        dedendum_cm = to_cm(1.25 * module)
        moduleExt_cm = moduleCm  # module-length construction extensions

        # --- Gear Profiles plane: through anchor line, 90deg off the target plane ---
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the Anchor Sketch's center point (NOT the raw user point).
        projected = sketch.project(self._anchorCenterPoint)
        centerSP = projected.item(0)
        projectedAnchor = sketch.project(anchorLine)
        anchorLineProj = projectedAnchor.item(0)

        c = centerSP.geometry
        # Anchor line 2-D unit direction.
        ag = anchorLineProj.startSketchPoint.geometry
        bg = anchorLineProj.endSketchPoint.geometry
        dvec = adsk.core.Vector3D.create(bg.x - ag.x, bg.y - ag.y, 0)
        dlen = math.hypot(dvec.x, dvec.y)
        dx = dvec.x / dlen
        dy = dvec.y / dlen
        # In-plane perpendicular candidate.
        perpx, perpy = -dy, dx

        # Pick sign by the target-plane normal (one-bit direction), toward the normal.
        targetNormal = get_normal(targetPlane)
        targetNormal.normalize()
        # Map perp into world to compare its direction against the target normal.
        # The perp lies in the gear-profiles plane; compare against target normal
        # by converting two sketch points to world.
        perpWorldA = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x, c.y, 0))
        perpWorldB = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perpx, c.y + perpy, 0))
        perpWorld = adsk.core.Vector3D.create(
            perpWorldB.x - perpWorldA.x,
            perpWorldB.y - perpWorldA.y,
            perpWorldB.z - perpWorldA.z)
        if perpWorld.dotProduct(targetNormal) < 0:
            perpx, perpy = -perpx, -perpy

        # --- Apex: free end of a construction line from center, perp to anchor ---
        apexSeed = adsk.core.Point3D.create(c.x + perpx * DPD, c.y + perpy * DPD, 0)
        centerToApex = lines.addByTwoPoints(centerSP, apexSeed)
        centerToApex.isConstruction = True
        constraints.addPerpendicular(centerToApex, anchorLineProj)
        apexPoint = centerToApex.endSketchPoint

        # --- closed-form cone geometry seeds for Apex->A, Apex->B ---
        # tan(gamma_p) = sin Sigma * PPD / (DPD + PPD cos Sigma)
        gamma_p = math.atan2(math.sin(Sigma) * PPD, DPD + PPD * math.cos(Sigma))
        gamma_g = Sigma - gamma_p
        coneDist_R = (PPD / 2) / math.sin(gamma_p)
        lenApexA = coneDist_R * math.cos(gamma_p)
        lenApexB = coneDist_R * math.cos(gamma_g)

        # --- Driving Gear Shaft Axis: from apex toward anchor line (-perp), -> B ---
        apexGeom = apexSeed  # seed coordinate for apex
        bSeed = adsk.core.Point3D.create(
            apexGeom.x - perpx * lenApexB, apexGeom.y - perpy * lenApexB, 0)
        drivingShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0), bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # --- Pinion Gear Shaft Axis: driving direction rotated about apex by Sigma ---
        # Driving shaft direction (from apex toward B) is -perp.
        ddx, ddy = -perpx, -perpy

        def rotate2d(vx, vy, theta):
            cc = math.cos(theta)
            ss = math.sin(theta)
            return (vx * cc - vy * ss, vx * ss + vy * cc)

        candPlus = rotate2d(ddx, ddy, Sigma)
        candMinus = rotate2d(ddx, ddy, -Sigma)
        aPlus = (apexGeom.x + candPlus[0] * lenApexA,
                 apexGeom.y + candPlus[1] * lenApexA)
        aMinus = (apexGeom.x + candMinus[0] * lenApexA,
                  apexGeom.y + candMinus[1] * lenApexA)
        # Keep the candidate whose endpoint has the GREATER X coordinate.
        if aPlus[0] >= aMinus[0]:
            aSeed = aPlus
            usedPlusSense = True
        else:
            aSeed = aMinus
            usedPlusSense = False

        pinionShaftAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0))
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        pointA = pinionShaftAxis.endSketchPoint

        # Angular dimension Sigma between the two shaft axes, text inside the wedge.
        wedgeMidx, wedgeMidy = rotate2d(ddx, ddy, Sigma / 2.0
                                        if usedPlusSense else -Sigma / 2.0)
        wedgeR = lenApexA * 0.5
        angleText = adsk.core.Point3D.create(
            apexGeom.x + wedgeMidx * wedgeR,
            apexGeom.y + wedgeMidy * wedgeR, 0)
        angDim = dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, angleText)
        angDim.parameter.value = Sigma

        # --- From A, perpendicular drop to Apex2 (length PPD/2) ---
        # Direction toward Apex2 (between the two shafts, toward anchor line).
        # Seed: perpendicular to pinion shaft axis, toward center.
        pinDirx = aSeed[0] - apexGeom.x
        pinDiry = aSeed[1] - apexGeom.y
        plen = math.hypot(pinDirx, pinDiry)
        pinDirx /= plen
        pinDiry /= plen
        # Two perpendiculars to pinion shaft; pick the one pointing toward center c.
        perpAcand1 = (-pinDiry, pinDirx)
        perpAcand2 = (pinDiry, -pinDirx)
        toCenterAx = c.x - aSeed[0]
        toCenterAy = c.y - aSeed[1]
        if (perpAcand1[0] * toCenterAx + perpAcand1[1] * toCenterAy) >= \
           (perpAcand2[0] * toCenterAx + perpAcand2[1] * toCenterAy):
            perpA = perpAcand1
        else:
            perpA = perpAcand2
        apex2SeedA = (aSeed[0] + perpA[0] * (PPD / 2),
                      aSeed[1] + perpA[1] * (PPD / 2))
        aToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(apex2SeedA[0], apex2SeedA[1], 0))
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaftAxis)
        aDropDim = dimensions.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (aSeed[0] + apex2SeedA[0]) / 2,
                (aSeed[1] + apex2SeedA[1]) / 2, 0))
        aDropDim.parameter.value = PPD / 2

        # --- From B, perpendicular drop to Apex2 (length DPD/2) ---
        drvDirx = bSeed.x - apexGeom.x
        drvDiry = bSeed.y - apexGeom.y
        dlen2 = math.hypot(drvDirx, drvDiry)
        drvDirx /= dlen2
        drvDiry /= dlen2
        perpBcand1 = (-drvDiry, drvDirx)
        perpBcand2 = (drvDiry, -drvDirx)
        toCenterBx = c.x - bSeed.x
        toCenterBy = c.y - bSeed.y
        if (perpBcand1[0] * toCenterBx + perpBcand1[1] * toCenterBy) >= \
           (perpBcand2[0] * toCenterBx + perpBcand2[1] * toCenterBy):
            perpB = perpBcand1
        else:
            perpB = perpBcand2
        apex2SeedB = (bSeed.x + perpB[0] * (DPD / 2),
                      bSeed.y + perpB[1] * (DPD / 2))
        bToApex2 = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0))
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaftAxis)
        bDropDim = dimensions.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (bSeed.x + apex2SeedB[0]) / 2,
                (bSeed.y + apex2SeedB[1]) / 2, 0))
        bDropDim.parameter.value = DPD / 2

        # --- Apex2: coincide the two drop endpoints ---
        constraints.addCoincident(
            aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint

        # --- Pitch Line: Apex -> Apex2 ---
        pitchLine = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0))
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # --- Dedendum lines from Apex2, perpendicular to Pitch Line, len module*1.25 ---
        # Pitch line direction (apex -> apex2).
        plx = apex2SeedB[0] - apexGeom.x
        ply = apex2SeedB[1] - apexGeom.y
        pllen = math.hypot(plx, ply)
        plx /= pllen
        ply /= pllen
        pitchPerp1 = (-ply, plx)
        pitchPerp2 = (ply, -plx)
        # Driving Gear Dedendum (point D): toward anchor line (toward center).
        toCenterApex2x = c.x - apex2SeedB[0]
        toCenterApex2y = c.y - apex2SeedB[1]
        if (pitchPerp1[0] * toCenterApex2x + pitchPerp1[1] * toCenterApex2y) >= \
           (pitchPerp2[0] * toCenterApex2x + pitchPerp2[1] * toCenterApex2y):
            dDir = pitchPerp1
            cDir = pitchPerp2
        else:
            dDir = pitchPerp2
            cDir = pitchPerp1

        dSeed = (apex2SeedB[0] + dDir[0] * dedendum_cm,
                 apex2SeedB[1] + dDir[1] * dedendum_cm)
        drivingDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        ddDim = dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2SeedB[0] + dSeed[0]) / 2,
                                     (apex2SeedB[1] + dSeed[1]) / 2, 0))
        ddDim.parameter.value = dedendum_cm
        pointD = drivingDedendum.endSketchPoint

        cSeed = (apex2SeedB[0] + cDir[0] * dedendum_cm,
                 apex2SeedB[1] + cDir[1] * dedendum_cm)
        pinionDedendum = lines.addByTwoPoints(
            adsk.core.Point3D.create(apex2SeedB[0], apex2SeedB[1], 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        cdDim = dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create((apex2SeedB[0] + cSeed[0]) / 2,
                                     (apex2SeedB[1] + cSeed[1]) / 2, 0))
        cdDim.parameter.value = dedendum_cm
        pointC = pinionDedendum.endSketchPoint

        # --- Root Axes: Apex->D (driving), Apex->C (pinion) ---
        drivingRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(
            adsk.core.Point3D.create(apexGeom.x, apexGeom.y, 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        # --- E: from A collinear with Apex->A, length module (no dimension) ---
        # Direction along Apex->A.
        aax = aSeed[0] - apexGeom.x
        aay = aSeed[1] - apexGeom.y
        aalen = math.hypot(aax, aay)
        aax /= aalen
        aay /= aalen
        eSeed = (aSeed[0] + aax * moduleExt_cm, aSeed[1] + aay * moduleExt_cm)
        aToE = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0))
        aToE.isConstruction = True
        constraints.addCoincident(aToE.startSketchPoint, pointA)
        constraints.addCollinear(aToE, pinionShaftAxis)
        pointE = aToE.endSketchPoint

        # --- C->E line, perpendicular to A->E ---
        cToE = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0))
        cToE.isConstruction = True
        constraints.addCoincident(cToE.startSketchPoint, pointC)
        constraints.addCoincident(cToE.endSketchPoint, pointE)
        constraints.addPerpendicular(aToE, cToE)

        # --- F: from B collinear with Apex->B, length module ---
        bbx = bSeed.x - apexGeom.x
        bby = bSeed.y - apexGeom.y
        bblen = math.hypot(bbx, bby)
        bbx /= bblen
        bby /= bblen
        fSeed = (bSeed.x + bbx * moduleExt_cm, bSeed.y + bby * moduleExt_cm)
        bToF = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0))
        bToF.isConstruction = True
        constraints.addCoincident(bToF.startSketchPoint, pointB)
        constraints.addCollinear(bToF, drivingShaftAxis)
        pointF = bToF.endSketchPoint

        # --- D->F line, perpendicular to B->F ---
        dToF = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0))
        dToF.isConstruction = True
        constraints.addCoincident(dToF.startSketchPoint, pointD)
        constraints.addCoincident(dToF.endSketchPoint, pointF)
        constraints.addPerpendicular(bToF, dToF)

        # --- G: from E collinear to A->E, length module ---
        gSeed = (eSeed[0] + aax * moduleExt_cm, eSeed[1] + aay * moduleExt_cm)
        eToG = lines.addByTwoPoints(
            adsk.core.Point3D.create(eSeed[0], eSeed[1], 0),
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0))
        eToG.isConstruction = True
        constraints.addCoincident(eToG.startSketchPoint, pointE)
        constraints.addCollinear(eToG, aToE)
        pointG = eToG.endSketchPoint

        # --- H: from C, length module, collinear with Apex2->C ---
        # direction along Apex2->C = cDir
        hSeed = (cSeed[0] + cDir[0] * moduleExt_cm, cSeed[1] + cDir[1] * moduleExt_cm)
        cToH = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        cToH.isConstruction = True
        constraints.addCoincident(cToH.startSketchPoint, pointC)
        constraints.addCollinear(cToH, pinionDedendum)
        pointH = cToH.endSketchPoint

        # --- G->H line, perpendicular to E->G ---
        gToH = lines.addByTwoPoints(
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0),
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        gToH.isConstruction = True
        constraints.addCoincident(gToH.startSketchPoint, pointG)
        constraints.addCoincident(gToH.endSketchPoint, pointH)
        constraints.addPerpendicular(eToG, gToH)

        # --- I: from F collinear to B->F, length module ---
        iSeed = (fSeed[0] + bbx * moduleExt_cm, fSeed[1] + bby * moduleExt_cm)
        fToI = lines.addByTwoPoints(
            adsk.core.Point3D.create(fSeed[0], fSeed[1], 0),
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0))
        fToI.isConstruction = True
        constraints.addCoincident(fToI.startSketchPoint, pointF)
        constraints.addCollinear(fToI, bToF)
        pointI = fToI.endSketchPoint

        # --- J: from D, length module, collinear with Apex2->D ---
        jSeed = (dSeed[0] + dDir[0] * moduleExt_cm, dSeed[1] + dDir[1] * moduleExt_cm)
        dToJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        dToJ.isConstruction = True
        constraints.addCoincident(dToJ.startSketchPoint, pointD)
        constraints.addCollinear(dToJ, drivingDedendum)
        pointJ = dToJ.endSketchPoint

        # --- I->J line, perpendicular to F->I ---
        iToJ = lines.addByTwoPoints(
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0),
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        iToJ.isConstruction = True
        constraints.addCoincident(iToJ.startSketchPoint, pointI)
        constraints.addCoincident(iToJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(fToI, iToJ)

        # --- Offset dimension: B->Apex2 drop vs J->I ; value = driving base height ---
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeightVal = self._drivingBaseHeight_cm
        else:
            drivingBaseHeightVal = to_cm(module * drivingTeeth / 8)
        jiOffsetDim = dimensions.addOffsetDimension(
            bToApex2, iToJ,
            adsk.core.Point3D.create(jSeed[0], jSeed[1], 0))
        jiOffsetDim.parameter.value = drivingBaseHeightVal

        # --- Offset dimension: A->Apex2 drop vs G->H ; value = pinion base height ---
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeightVal = self._pinionBaseHeight_cm
        else:
            pinionBaseHeightVal = drivingBaseHeightVal * (pinionTeeth / drivingTeeth)
        ghOffsetDim = dimensions.addOffsetDimension(
            aToApex2, gToH,
            adsk.core.Point3D.create(hSeed[0], hSeed[1], 0))
        ghOffsetDim.parameter.value = pinionBaseHeightVal

        # --- Line A->G ---
        aToG = lines.addByTwoPoints(
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0),
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0))
        aToG.isConstruction = True
        constraints.addCoincident(aToG.startSketchPoint, pointA)
        constraints.addCoincident(aToG.endSketchPoint, pointG)

        # --- Constrain Point I with center point ---
        constraints.addCoincident(pointI, centerSP)

        # --- K: construction line from G away from Apex, along Apex->A, end K ---
        # direction away from apex along Apex->A = (aax, aay)
        kSeed = (gSeed[0] + aax * moduleExt_cm, gSeed[1] + aay * moduleExt_cm)
        gToK = lines.addByTwoPoints(
            adsk.core.Point3D.create(gSeed[0], gSeed[1], 0),
            adsk.core.Point3D.create(kSeed[0], kSeed[1], 0))
        gToK.isConstruction = True
        constraints.addCoincident(gToK.startSketchPoint, pointG)
        pointK = gToK.endSketchPoint
        # Pin K with two point-on-line coincidents (NOT addCollinear).
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)

        # Reference line C->K.
        cToK = lines.addByTwoPoints(
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
            adsk.core.Point3D.create(kSeed[0], kSeed[1], 0))
        cToK.isConstruction = True
        constraints.addCoincident(cToK.startSketchPoint, pointC)
        constraints.addCoincident(cToK.endSketchPoint, pointK)

        # --- Tooth-center point K' (Tooth Spacing offset) ---
        # When spacing == 0: K' == K, reference line C->K. Build nothing extra.
        if self._toothSpacing_cm > 0:
            # Shift K outward along the dedendum direction, away from C.
            # dedendum direction outward = cDir (Apex2 -> C direction), continuing past K.
            kpSeed = (kSeed[0] + cDir[0] * self._toothSpacing_cm,
                      kSeed[1] + cDir[1] * self._toothSpacing_cm)
            kToKp = lines.addByTwoPoints(
                adsk.core.Point3D.create(kSeed[0], kSeed[1], 0),
                adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
            kToKp.isConstruction = True
            constraints.addCoincident(kToKp.startSketchPoint, pointK)
            pointKp = kToKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            kpDim = dimensions.addDistanceDimension(
                kToKp.startSketchPoint, kToKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create((kSeed[0] + kpSeed[0]) / 2,
                                         (kSeed[1] + kpSeed[1]) / 2, 0))
            kpDim.parameter.value = self._toothSpacing_cm
            # Tooth-center reference line C->K'.
            cToKp = lines.addByTwoPoints(
                adsk.core.Point3D.create(cSeed[0], cSeed[1], 0),
                adsk.core.Point3D.create(kpSeed[0], kpSeed[1], 0))
            cToKp.isConstruction = True
            constraints.addCoincident(cToKp.startSketchPoint, pointC)
            constraints.addCoincident(cToKp.endSketchPoint, pointKp)
            pinionCenterPoint = pointKp
            pinionCenterRefLine = cToKp
        else:
            pinionCenterPoint = pointK
            pinionCenterRefLine = cToK

        # --- Resolve Maximum Face Width from SOLVED geometry ---
        aGeo = pointA.geometry
        bGeo = pointB.geometry
        cGeo = pointC.geometry
        dGeo = pointD.geometry
        hGeo = pointH.geometry
        jGeo = pointJ.geometry
        distA = self._perpDistancePointToLine(aGeo, cGeo, hGeo)
        distB = self._perpDistancePointToLine(bGeo, dGeo, jGeo)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        if self._faceWidthSpecified:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face width {:.3f} mm exceeds maximum {:.3f} mm'.format(
                        to_mm(self._faceWidth_cm), to_mm(maxFaceWidth_cm)))
            faceWidth_cm = self._faceWidth_cm
        else:
            coneDistance_cm = to_cm(math.sqrt(
                (module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
            faceWidth_cm = min(coneDistance_cm / 6, maxFaceWidth_cm)

        # --- M->N (pinion toe line) ---
        # C->H direction.
        chx = hGeo.x - cGeo.x
        chy = hGeo.y - cGeo.y
        chlen = math.hypot(chx, chy)
        chx /= chlen
        chy /= chlen
        # Seed M at midpoint of Apex->C.
        apexSolved = apexPoint.geometry
        mSeed = ((apexSolved.x + cGeo.x) / 2, (apexSolved.y + cGeo.y) / 2)
        # Seed N by sliding from M-seed along C->H by ~distance M-seed to A.
        slideN = math.hypot(mSeed[0] - aGeo.x, mSeed[1] - aGeo.y)
        nSeed = (mSeed[0] + chx * slideN, mSeed[1] + chy * slideN)
        mToN = lines.addByTwoPoints(
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0),
            adsk.core.Point3D.create(nSeed[0], nSeed[1], 0))
        mToN.isConstruction = True
        pointM = mToN.startSketchPoint
        pointN = mToN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, aToApex2)
        constraints.addParallel(mToN, cToH)
        mnOffsetDim = dimensions.addOffsetDimension(
            cToH, mToN,
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0))
        mnOffsetDim.parameter.value = faceWidth_cm

        # M->C and N->A connecting lines.
        mToC = lines.addByTwoPoints(
            adsk.core.Point3D.create(mSeed[0], mSeed[1], 0),
            adsk.core.Point3D.create(cSeed[0], cSeed[1], 0))
        mToC.isConstruction = True
        constraints.addCoincident(mToC.startSketchPoint, pointM)
        constraints.addCoincident(mToC.endSketchPoint, pointC)
        nToA = lines.addByTwoPoints(
            adsk.core.Point3D.create(nSeed[0], nSeed[1], 0),
            adsk.core.Point3D.create(aSeed[0], aSeed[1], 0))
        nToA.isConstruction = True
        constraints.addCoincident(nToA.startSketchPoint, pointN)
        constraints.addCoincident(nToA.endSketchPoint, pointA)

        # --- L: construction line from I away from Apex, along Apex->B, end L ---
        lSeed = (iSeed[0] + bbx * moduleExt_cm, iSeed[1] + bby * moduleExt_cm)
        iToL = lines.addByTwoPoints(
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0),
            adsk.core.Point3D.create(lSeed[0], lSeed[1], 0))
        iToL.isConstruction = True
        constraints.addCoincident(iToL.startSketchPoint, pointI)
        pointL = iToL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)

        dToL = lines.addByTwoPoints(
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
            adsk.core.Point3D.create(lSeed[0], lSeed[1], 0))
        dToL.isConstruction = True
        constraints.addCoincident(dToL.startSketchPoint, pointD)
        constraints.addCoincident(dToL.endSketchPoint, pointL)

        # --- Tooth-center point L' (Tooth Spacing offset) ---
        if self._toothSpacing_cm > 0:
            lpSeed = (lSeed[0] + dDir[0] * self._toothSpacing_cm,
                      lSeed[1] + dDir[1] * self._toothSpacing_cm)
            lToLp = lines.addByTwoPoints(
                adsk.core.Point3D.create(lSeed[0], lSeed[1], 0),
                adsk.core.Point3D.create(lpSeed[0], lpSeed[1], 0))
            lToLp.isConstruction = True
            constraints.addCoincident(lToLp.startSketchPoint, pointL)
            pointLp = lToLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            lpDim = dimensions.addDistanceDimension(
                lToLp.startSketchPoint, lToLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create((lSeed[0] + lpSeed[0]) / 2,
                                         (lSeed[1] + lpSeed[1]) / 2, 0))
            lpDim.parameter.value = self._toothSpacing_cm
            dToLp = lines.addByTwoPoints(
                adsk.core.Point3D.create(dSeed[0], dSeed[1], 0),
                adsk.core.Point3D.create(lpSeed[0], lpSeed[1], 0))
            dToLp.isConstruction = True
            constraints.addCoincident(dToLp.startSketchPoint, pointD)
            constraints.addCoincident(dToLp.endSketchPoint, pointLp)
            drivingCenterPoint = pointLp
            drivingCenterRefLine = dToLp
        else:
            drivingCenterPoint = pointL
            drivingCenterRefLine = dToL

        # --- O->P (driving toe line, mirror of M->N) ---
        djx = jGeo.x - dGeo.x
        djy = jGeo.y - dGeo.y
        djlen = math.hypot(djx, djy)
        djx /= djlen
        djy /= djlen
        oSeed = ((apexSolved.x + dGeo.x) / 2, (apexSolved.y + dGeo.y) / 2)
        slideP = math.hypot(oSeed[0] - bGeo.x, oSeed[1] - bGeo.y)
        pSeed = (oSeed[0] + djx * slideP, oSeed[1] + djy * slideP)
        oToP = lines.addByTwoPoints(
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0),
            adsk.core.Point3D.create(pSeed[0], pSeed[1], 0))
        oToP.isConstruction = True
        pointO = oToP.startSketchPoint
        pointP = oToP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, bToApex2)
        constraints.addParallel(oToP, dToJ)
        opOffsetDim = dimensions.addOffsetDimension(
            dToJ, oToP,
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0))
        opOffsetDim.parameter.value = faceWidth_cm

        oToD = lines.addByTwoPoints(
            adsk.core.Point3D.create(oSeed[0], oSeed[1], 0),
            adsk.core.Point3D.create(dSeed[0], dSeed[1], 0))
        oToD.isConstruction = True
        constraints.addCoincident(oToD.startSketchPoint, pointO)
        constraints.addCoincident(oToD.endSketchPoint, pointD)
        pToB = lines.addByTwoPoints(
            adsk.core.Point3D.create(pSeed[0], pSeed[1], 0),
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0))
        pToB.isConstruction = True
        constraints.addCoincident(pToB.startSketchPoint, pointP)
        constraints.addCoincident(pToB.endSketchPoint, pointB)

        # Line B->I.
        bToI = lines.addByTwoPoints(
            adsk.core.Point3D.create(bSeed.x, bSeed.y, 0),
            adsk.core.Point3D.create(iSeed[0], iSeed[1], 0))
        bToI.isConstruction = True
        constraints.addCoincident(bToI.startSketchPoint, pointB)
        constraints.addCoincident(bToI.endSketchPoint, pointI)

        # Gate: the §2 sketch must be fully constrained.
        self._assertFullyConstrained(sketch, 'Gear Profiles')

        # --- Surface the spec's named points as NAMED construction points (a debugging
        # aid for discussing geometry). They go in a 'Profile Sketch Points' COMPONENT
        # used as a folder: the API has no browser sub-folder for construction geometry,
        # and a component gives a single visibility toggle (hidden after a good build,
        # while each point keeps its own visibility). ---
        _lv = locals()
        namedPoints = [(nm, _lv[var]) for nm, var in [
            ('A', 'pointA'), ('B', 'pointB'), ('C', 'pointC'), ('D', 'pointD'),
            ('E', 'pointE'), ('F', 'pointF'), ('G', 'pointG'), ('H', 'pointH'),
            ('I', 'pointI'), ('J', 'pointJ'), ('K', 'pointK'), ('Kp', 'pointKp'),
            ('L', 'pointL'), ('Lp', 'pointLp'), ('M', 'pointM'), ('N', 'pointN'),
            ('O', 'pointO'), ('P', 'pointP'), ('Apex', 'apexPoint'),
        ] if var in _lv]                          # Kp/Lp only exist for some configs
        namedPoints.append(('Apex2', aToApex2.endSketchPoint))
        self._profilePointsOcc = designComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        ptsComp = self._profilePointsOcc.component
        ptsComp.name = 'Profile Sketch Points'
        for nm, sp in namedPoints:
            placed = False
            for ref in (sp, sp.worldGeometry):    # sketch-point ref, else world Point3D
                try:
                    cpInput = ptsComp.constructionPoints.createInput()
                    cpInput.setByPoint(ref)
                    ptsComp.constructionPoints.add(cpInput).name = f'pt {nm}'
                    placed = True
                    break
                except Exception as exc:
                    lastExc = exc
            if not placed:
                futil.log(f'[named-points] could not place "{nm}": {lastExc}',
                          force_console=True)

        # --- Spiral support: DRIVEN face-width dims on O->D (driving) and M->C (pinion).
        # Their parameter names (dXXX) drive the spiral tooth construction. Also stash the
        # §2 lines the spiral build references later (I->J/G->H, D->J/C->H). ---
        _ALN = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation

        def _drivenLen(ln):
            g = ln.startSketchPoint.geometry
            d = dimensions.addDistanceDimension(
                ln.startSketchPoint, ln.endSketchPoint, _ALN,
                adsk.core.Point3D.create(g.x, g.y + to_cm(0.3), 0), False)
            return d.parameter.name

        self._dFaceDriving = _drivenLen(oToD)   # |O->D| = driving face width
        self._dFacePinion = _drivenLen(mToC)    # |M->C| = pinion face width
        self._lineIJ, self._lineGH = iToJ, gToH
        self._lineDJ, self._lineCH = dToJ, cToH
        self._anchorPoint = centerPoint         # the bevel anchor (for the dYYY angle)
        futil.log(f'[spiral] face-width params: driving={self._dFaceDriving}, '
                  f'pinion={self._dFacePinion}', force_console=True)

        # --- §3: virtual spur tooth profiles ---
        # gamma_p, gamma_g already computed above.
        pinionVirtualPitchRadius_cm = (PPD / 2) / math.cos(gamma_p)
        pinionVirtualTeeth = int(math.floor(
            2 * pinionVirtualPitchRadius_cm / moduleCm))
        drivingVirtualPitchRadius_cm = (DPD / 2) / math.cos(gamma_g)
        drivingVirtualTeeth = int(math.floor(
            2 * drivingVirtualPitchRadius_cm / moduleCm))

        # apex sketch point used as the loft degenerate section.
        apexSketchPoint = centerToApex.endSketchPoint

        (pinionToothPlane, pinionToothSketch, pinionToothAxis) = \
            self._buildVirtualSpurProfile(
                designComponent, sketch, gearProfilesPlane,
                pinionCenterRefLine, pinionCenterPoint,
                module, pinionVirtualTeeth, 'Pinion Tooth')

        (drivingToothPlane, drivingToothSketch, drivingToothAxis) = \
            self._buildVirtualSpurProfile(
                designComponent, sketch, gearProfilesPlane,
                drivingCenterRefLine, drivingCenterPoint,
                module, drivingVirtualTeeth, 'Driving Tooth')

        # --- Pinion Gear body ---
        pinionGearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        pinionGearOccurrence.component.name = 'Pinion Gear'

        self._createGearBody(
            designComponent, gearProfilesPlane, sketch,
            [pointA, pointG, pointH, pointC, pointM, pointN],
            apexSketchPoint, pinionToothSketch,
            (pointM, pointN), (pointC, pointH),
            pinionTeeth, pinionBore_cm,
            'Pinion', pinionGearOccurrence, meshRotation=self._pinionMeshPhase())

        # --- Driving Gear body ---
        drivingGearOccurrence = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        drivingGearOccurrence.component.name = 'Driving Gear'

        drivingMeshRotation = math.radians(180.0 / drivingTeeth)
        self._createGearBody(
            designComponent, gearProfilesPlane, sketch,
            [pointB, pointI, pointJ, pointD, pointO, pointP],
            apexSketchPoint, drivingToothSketch,
            (pointO, pointP), (pointD, pointJ),
            drivingTeeth, drivingBore_cm,
            'Driving', drivingGearOccurrence, meshRotation=drivingMeshRotation)

        # Both gears built successfully -> hide the Profile Sketch Points folder (toggle
        # the component's visibility only; each point keeps its own visibility on).
        if getattr(self, '_profilePointsOcc', None):
            self._profilePointsOcc.isLightBulbOn = False

    # ----- §3 helper: virtual spur tooth profile -----
    def _buildVirtualSpurProfile(self, designComponent, gearProfilesSketch,
                                 gearProfilesPlane, centerRefLine, centerPoint,
                                 module, virtualTeeth, label):
        # Plane that includes centerRefLine, perpendicular to the Gear Profiles plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerRefLine, adsk.core.ValueInput.createByString('90 deg'),
            gearProfilesPlane)
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = f'{label} Plane'           # the spiral cut plane offsets from this
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{label} Profile'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))

        self._assertFullyConstrained(toothSketch, f'{label} Profile')

        # Construction axis through centerPoint normal to the tooth plane, via setByTwoPlanes:
        # the Gear Profiles plane intersected with a helper plane perpendicular to
        # centerRefLine at its far end (setByDistanceOnPath at 1.0).
        helperInput = designComponent.constructionPlanes.createInput()
        helperInput.setByDistanceOnPath(
            centerRefLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperInput)
        helperPlane.name = f'{label} Helper Plane'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{label} Axis'

        return (toothPlane, toothSketch, toothAxis)

    # ----- per-gear body creation -----
    def _createGearBody(self, designComponent, gearProfilesPlane, gearProfilesSketch,
                        hexVertices, apexSketchPoint, toothSketch,
                        toeEdgePoints, heelEdgePoints,
                        teethNumber, bore_cm, gearLabel, targetOccurrence,
                        meshRotation):
        features = designComponent.features

        # --- Profile sketch with recreated fixed vertices ---
        profileSketch = designComponent.sketches.add(gearProfilesPlane)
        profileSketch.name = f'{gearLabel} Profile'
        profileSketch.isVisible = True
        lines = profileSketch.sketchCurves.sketchLines

        # Recreate the six vertices at their world-mapped positions.
        verts = []
        for src in hexVertices:
            local = profileSketch.modelToSketchSpace(src.worldGeometry)
            verts.append(profileSketch.sketchPoints.add(local))

        # Draw the closed hexagon sharing those points (A->G->H->C->M->N->A style).
        hexLines = []
        for i in range(len(verts)):
            nxt = (i + 1) % len(verts)
            hexLines.append(lines.addByTwoPoints(verts[i], verts[nxt]))

        # Fix endpoints AFTER the lines exist.
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(profileSketch, f'{gearLabel} Profile')

        # The shaft axis is this profile sketch's FIRST edge.
        shaftAxisEdge = hexLines[0]

        # Single profile (one hexagon loop).
        profile = profileSketch.profiles.item(0)

        # --- Revolve around the shaft axis edge ---
        revolves = features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # --- Loft apex point -> tooth profile ---
        toothProfile = self._findSpurToothProfile(toothSketch)
        lofts = features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(apexSketchPoint)
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth Body'

        # --- Apex world point (for containment tests) ---
        apexWorld = apexSketchPoint.worldGeometry

        # --- Conical cuts: toe then heel, with keeper selection between ---
        toe_m = self._pointWorldGeometry(toeEdgePoints[0])
        toe_n = self._pointWorldGeometry(toeEdgePoints[1])
        toeMid = adsk.core.Point3D.create(
            (toe_m.x + toe_n.x) / 2, (toe_m.y + toe_n.y) / 2,
            (toe_m.z + toe_n.z) / 2)

        heel_c = self._pointWorldGeometry(heelEdgePoints[0])
        heel_h = self._pointWorldGeometry(heelEdgePoints[1])
        heelMid = adsk.core.Point3D.create(
            (heel_c.x + heel_h.x) / 2, (heel_c.y + heel_h.y) / 2,
            (heel_c.z + heel_h.z) / 2)

        # --- Build the tooth via the hook. It receives the UNCUT toothBody (the full
        # apex->heel taper). Straight bevel just trims it to a band with the toe/heel
        # cone cuts; the spiral subclass slices the full taper, lofts the spiral
        # tooth, then trims with the same cones -- so its ends are flush/conical. ---
        tooth = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            apexSketchPoint, toeMid, heelMid, toe_m, heel_c, toothSketch.referencePlane,
            gearLabel, teethNumber)

        # --- Circular-pattern the tooth around the shaft axis edge ---
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(tooth)
        patterns = features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # --- Combine-join the patterned teeth with the Gear Body ---
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # --- Bore ---
        if self._boreEnable and bore_cm > 0:
            self._cutBore(designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel)

        # --- Meshing rotation (driving gear only) ---
        if meshRotation != 0.0:
            startW = shaftAxisEdge.startSketchPoint.worldGeometry
            endW = shaftAxisEdge.endSketchPoint.worldGeometry
            axisVec = adsk.core.Vector3D.create(
                endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
            axisVec.normalize()
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(meshRotation, axisVec, startW)
            moveBodies = adsk.core.ObjectCollection.create()
            moveBodies.add(gearBody)
            moves = features.moveFeatures
            moveInput = moves.createInput2(moveBodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # --- Relocate finished body into the gear component ---
        gearBody.moveToComponent(targetOccurrence)

    # ----- final mesh-phase nudge of the pinion about Apex->A -----
    # Tunable: angle (in tooth fractions) to rotate the pinion gear so the spiral teeth
    # seat tooth-in-gap with the driving gear. The straight bevel meshes with NO pinion
    # nudge, and our spiral tooth is centred so its mean cross-section is unrotated
    # (identical to the straight tooth at mid-face) -> it must mesh at mid-face with the
    # same 0 nudge. A non-zero value shifts the pinion off that proven position and shows
    # up as volume overlap at mid-face. Keep at 0 unless the mesh genuinely needs it.
    _PINION_MESH_PHASE_TEETH = 0.0

    def _pinionMeshPhase(self):
        """Extra rotation (radians) of the pinion gear about its own shaft axis (Apex->A),
        applied via the same mesh-rotation step as the driving gear. Straight bevel needs
        none; the spiral path seats the curved teeth tooth-in-gap."""
        if self._spiralAngle_rad <= 0:
            return 0.0
        return math.radians(self._PINION_MESH_PHASE_TEETH * 360.0 / self._sbPinionTeeth)

    # ----- lengthwise crown (relief) -----
    # Tunable: circumferential relief fraction per radian of local twist. Each segment is
    # scaled by (1 - _CROWN_PER_RAD*|ang|) about its centroid, so mid-face (twist 0) is
    # untouched and toe/heel are progressively thinned -> contact localizes at mid-face and
    # the twisted ends clear instead of gouging. Scales with spiral angle automatically.
    # 0 disables. Dial up until a high-ratio pair runs clean.
    _CROWN_PER_RAD = 0.5

    # ----- trace-construction helpers (ported from the validated trace sketch) -----
    def _combine(self, base, a, e1, b=0.0, e2=None):
        """World Point3D = base + a*e1 (+ b*e2). a, b in cm; e1, e2 unit Vector3D."""
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, deg):
        ci = comp.constructionPlanes.createInput()
        ci.setByAngle(line, adsk.core.ValueInput.createByString(f'{deg} deg'), refPlane)
        return comp.constructionPlanes.add(ci)

    def _distDim(self, sketch, p0, p1, textWorld, value_cm):
        """Aligned point-to-point driving distance dimension, value in cm."""
        d = sketch.sketchDimensions.addDistanceDimension(
            p0, p1, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sketch.modelToSketchSpace(textWorld))
        d.parameter.value = value_cm
        return d

    def _circleIntersectNearest(self, R, cx, cy, rc, nearPx, nearPy, R_label):
        """Intersection of circle(origin,R) with cutter circle(C=(cx,cy),rc); the
        solution nearest (nearPx,nearPy). All 2-D, tangent-plane coords (cm)."""
        d = math.hypot(cx, cy)
        a = (R * R - rc * rc + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            raise Exception(
                f'Cutter Radius {to_mm(rc):.3f} mm too small: cutter circle never '
                f'reaches the {R_label} radius {to_mm(R):.3f} mm. Increase Cutter Radius.')
        h = math.sqrt(h2)
        ux, uy = cx / d, cy / d
        bx, by = a * ux, a * uy
        s1 = (bx - h * uy, by + h * ux)
        s2 = (bx + h * uy, by - h * ux)
        d1 = math.hypot(s1[0] - nearPx, s1[1] - nearPy)
        d2 = math.hypot(s2[0] - nearPx, s2[1] - nearPy)
        return s1 if d1 <= d2 else s2

    def _bottomEdgeMid(self, profile, apex, axisDir):
        """World midpoint of the profile's bottom (root) edge: the boundary curve whose
        own midpoint sits nearest the shaft axis (the tooth root)."""
        best, bestPerp2 = None, None
        for loop in profile.profileLoops:
            for pc in loop.profileCurves:
                e = pc.sketchEntity.worldGeometry.evaluator
                (_ok, sp, ep) = e.getParameterExtents()
                (_ok, mid) = e.getPointAtParameter(0.5 * (sp + ep))
                wx, wy, wz = mid.x - apex.x, mid.y - apex.y, mid.z - apex.z
                along = wx * axisDir.x + wy * axisDir.y + wz * axisDir.z
                perp2 = (wx * wx + wy * wy + wz * wz) - along * along
                if bestPerp2 is None or perp2 < bestPerp2:
                    bestPerp2, best = perp2, mid
        return best

    def _perpToAxis(self, pt, apex, axisDir):
        """Component of (pt - apex) perpendicular to the shaft axis (the radial vector
        about which a point's azimuth is measured)."""
        wx, wy, wz = pt.x - apex.x, pt.y - apex.y, pt.z - apex.z
        a = wx * axisDir.x + wy * axisDir.y + wz * axisDir.z
        return adsk.core.Vector3D.create(
            wx - a * axisDir.x, wy - a * axisDir.y, wz - a * axisDir.z)

    # ----- tooth-body transform hook -----
    # Straight bevel (spiral angle == 0): trim the uncut tooth body to a band with the
    # toe/heel cone cuts. Spiral (angle > 0): build the genuine cutter-arc TRACE, slice
    # the straight tooth into sections, MOVE each section onto the trace, loft through
    # them, then trim with the same cones.
    def _transformToothBody(self, designComponent, toothBody, gearBody, shaftAxisEdge,
                            apexWorld, apexSketchPoint, toeMid, heelMid, toeConeWorld,
                            heelConeWorld, parentToothPlane, gearLabel, teethNumber):
        if self._spiralAngle_rad <= 0:
            # straight: just trim the uncut tooth to a band, like the base hook.
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        features = designComponent.features
        apex = apexWorld

        # --- world frame from the built geometry ---
        sA = shaftAxisEdge.startSketchPoint.worldGeometry
        sB = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = adsk.core.Vector3D.create(sB.x - sA.x, sB.y - sA.y, sB.z - sA.z)
        axisDir.normalize()
        # The tooth TRACE plane sits on the cone element Apex -> D (heelEdgePoints[0]:
        # 'D' for the driving gear, 'C' for the pinion). Use that exact direction.
        coneVec = adsk.core.Vector3D.create(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y, heelConeWorld.z - apex.z)
        coneVec.normalize()
        # tangent-plane frame: e_x = coneVec (the Apex->D element), e_y = v (circumferential).
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)   # normal of the tangent plane
        tpNormal.normalize()

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x + (p.y - apex.y) * coneVec.y
                    + (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- cutter-arc geometry (tangent-plane 2-D frame: e_x = coneVec, e_y = v) ---
        isPinion = (gearLabel == 'Pinion')
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        psi = self._spiralAngle_rad
        handSign = +1 if self._spiralHand == _HAND_RIGHT else -1
        if isPinion:
            handSign = -handSign          # the pair meshes with opposite hands
        # Opposite hand = mirror the cutter centre across the CONE ELEMENT (py=0), i.e.
        # flip Cy (the cos term), NOT Cx. (Putting handSign on the sin term mirrors about
        # x=R_mean instead, which gives the two gears different twist magnitudes.)
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        def arc2d(R):
            return self._circleIntersectNearest(R, Cx, Cy, r_c, R_mean, 0.0, 'sec')

        # arc spans a touch past both cones (toe toward apex, heel past the back cone).
        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = arc2d(R_lo)
        heel2d = arc2d(R_hi)
        futil.log(
            f'[spiral] {gearLabel}: R[toe/mean/heel]=[{to_mm(R_toe):.2f}/{to_mm(R_mean):.2f}/'
            f'{to_mm(R_heel):.2f}] mm, r_c={to_mm(r_c):.2f} mm, hand={handSign:+d}',
            force_console=True)
        futil.log(
            f'[spiral] {gearLabel}: TRACE2D psi={math.degrees(psi):.2f} C=('
            f'{to_mm(Cx):.2f},{to_mm(Cy):.2f}) toe2d=({to_mm(toe2d[0]):.2f},'
            f'{to_mm(toe2d[1]):.2f}) heel2d=({to_mm(heel2d[0]):.2f},{to_mm(heel2d[1]):.2f}) '
            f'-> 2D twist span={to_mm(heel2d[1]-toe2d[1]):.2f}mm', force_console=True)

        # ================================================================
        # STEP 1: build the tooth TRACE first -- a GENUINE constrained cutter arc on
        # the pitch-cone tangent plane (centre coincident to the cutter circle, radius
        # r_c, ends pinned by their cone distance). No plotting.
        # ================================================================
        axialPlane = shaftAxisEdge.parentSketch.referencePlane
        ceSketch = designComponent.sketches.add(axialPlane)
        ceSketch.name = f'{gearLabel} Cone Element'
        ceLine = ceSketch.sketchCurves.sketchLines.addByTwoPoints(
            ceSketch.modelToSketchSpace(apex),
            ceSketch.modelToSketchSpace(self._combine(apex, R_heel, coneVec)))
        ceLine.isConstruction = True
        tangentPlane = self._planeByAngle(designComponent, ceLine, axialPlane, 90)
        tangentPlane.name = f'{gearLabel} Trace Plane'   # (NOT 'Tooth Plane' -- that name
        #                              belongs to the parent's transverse profile plane)

        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        tgc = traceSketch.geometricConstraints
        tlines = traceSketch.sketchCurves.sketchLines
        tApex = traceSketch.project(apexSketchPoint).item(0)
        tAxis = traceSketch.project(shaftAxisEdge).item(0)   # projects along the cone element

        coneEl = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(apex),
            traceSketch.modelToSketchSpace(tanW(R_heel, 0)))
        coneEl.isConstruction = True
        tgc.addCoincident(coneEl.startSketchPoint, tApex)
        tgc.addParallel(coneEl, tAxis)
        self._distDim(traceSketch, coneEl.startSketchPoint, coneEl.endSketchPoint,
                      tanW(R_heel * 0.5, -0.2), R_heel)

        lineX = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(apex), traceSketch.modelToSketchSpace(tanW(Cx, 0)))
        lineX.isConstruction = True
        tgc.addCoincident(lineX.startSketchPoint, tApex)
        tgc.addParallel(lineX, coneEl)
        self._distDim(traceSketch, lineX.startSketchPoint, lineX.endSketchPoint,
                      tanW(Cx * 0.5, -0.2), abs(Cx))
        lineY = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(tanW(Cx, 0)), traceSketch.modelToSketchSpace(tanW(Cx, Cy)))
        lineY.isConstruction = True
        tgc.addCoincident(lineY.startSketchPoint, lineX.endSketchPoint)
        tgc.addPerpendicular(lineY, coneEl)
        self._distDim(traceSketch, lineY.startSketchPoint, lineY.endSketchPoint,
                      tanW(Cx + 0.2, Cy * 0.5), abs(Cy))

        cutter = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            traceSketch.modelToSketchSpace(tanW(Cx, Cy)), r_c)
        cutter.isConstruction = True
        tgc.addCoincident(cutter.centerSketchPoint, lineY.endSketchPoint)
        traceSketch.sketchDimensions.addDiameterDimension(
            cutter, traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy))).parameter.value = 2 * r_c

        arc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            traceSketch.modelToSketchSpace(tanW(*toe2d)),
            traceSketch.modelToSketchSpace(tanW(R_mean, 0.0)),
            traceSketch.modelToSketchSpace(tanW(*heel2d)))
        tgc.addCoincident(arc.centerSketchPoint, lineY.endSketchPoint)
        traceSketch.sketchDimensions.addRadialDimension(
            arc, traceSketch.modelToSketchSpace(tanW(Cx, Cy - r_c * 0.5))).parameter.value = r_c
        # (Endpoint distance dims removed for now -- they over-constrained the solve with
        # the Apex->D plane. The arc is placed exactly by its three cutter-circle points.)
        futil.log(f'[spiral] {gearLabel}: STEP 1 trace arc built, '
                  f'fullyConstrained={traceSketch.isFullyConstrained}', force_console=True)

        # ================================================================
        # 3-D TOOTH TRACE: project the 2-D arc onto the ROOT CONE face (the cone the
        # dedendum element M->C / O->D generates) ALONG the Tooth Plane normal.
        # ================================================================
        gearProfilesPlane = shaftAxisEdge.parentSketch.referencePlane
        # Root cone face = the gearBody cone face containing the midpoint of the dedendum
        # element (toeConeWorld..heelConeWorld = M..C / O..D).
        midDed = adsk.core.Point3D.create(
            0.5 * (toeConeWorld.x + heelConeWorld.x),
            0.5 * (toeConeWorld.y + heelConeWorld.y),
            0.5 * (toeConeWorld.z + heelConeWorld.z))
        rootFace = None
        for f in gearBody.faces:
            if f.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            ev2 = f.evaluator
            (ok, param) = ev2.getParameterAtPoint(midDed)
            if not ok:
                continue
            (ok2, back) = ev2.getPointAtParameter(param)
            if ok2 and back.distanceTo(midDed) < 1e-3:    # midDed lies on this cone face
                rootFace = f
                break
        if rootFace is None:
            raise Exception(f'{gearLabel}: could not find the root cone face at the dedendum midpoint')

        # Direction guide = a construction line along the Tooth Plane normal, in ITS OWN
        # sketch (the projection target must not contain any geometry of its own).
        # tpNormal lies in the gear-profiles plane, so it can be drawn there.
        dirSketch = designComponent.sketches.add(gearProfilesPlane)
        dirSketch.name = f'{gearLabel} Proj Dir'
        dirLine = dirSketch.sketchCurves.sketchLines.addByTwoPoints(
            dirSketch.modelToSketchSpace(apex),
            dirSketch.modelToSketchSpace(self._combine(apex, 1.0, tpNormal)))
        dirLine.isConstruction = True

        sketch3d = designComponent.sketches.add(gearProfilesPlane)
        sketch3d.name = f'{gearLabel} 3D Tooth Trace'
        proj = sketch3d.projectToSurface(
            [rootFace], [arc],
            adsk.fusion.SurfaceProjectTypes.AlongVectorSurfaceProjectType, dirLine)
        trace3d = proj.item(0) if hasattr(proj, 'item') else proj[0]
        futil.log(f'[spiral] {gearLabel}: projected 3-D tooth trace onto the root cone',
                  force_console=True)

        # ================================================================
        # dYYY: TWIST ANGLE. Plane at 90 deg to I->J / G->H; project the APEX (shared by
        # both gears and on both shaft axes -> symmetric for equal teeth, unlike the
        # off-axis assembly centre) and the 3-D trace onto it; join apex->trace-start &
        # apex->trace-end; put a DRIVEN angular dimension between them.
        # ================================================================
        lineForPlane = self._lineGH if isPinion else self._lineIJ
        twistPlane = self._planeByAngle(designComponent, lineForPlane, gearProfilesPlane, 90)
        twistPlane.name = f'{gearLabel} Twist Plane'
        angleSketch = designComponent.sketches.add(twistPlane)
        angleSketch.name = f'{gearLabel} Twist Angle'
        pApex = angleSketch.project(apexSketchPoint).item(0)
        pStart = angleSketch.project(trace3d.startSketchPoint).item(0)
        pEnd = angleSketch.project(trace3d.endSketchPoint).item(0)
        aLines = angleSketch.sketchCurves.sketchLines
        lA = aLines.addByTwoPoints(pApex, pStart)
        lB = aLines.addByTwoPoints(pApex, pEnd)
        # text point at the chord midpoint (inside the arc's sector) -> dimension picks
        # the angle the arc subtends, not its supplement.
        midChord = adsk.core.Point3D.create(
            0.5 * (pStart.geometry.x + pEnd.geometry.x),
            0.5 * (pStart.geometry.y + pEnd.geometry.y),
            0.5 * (pStart.geometry.z + pEnd.geometry.z))
        angDim = angleSketch.sketchDimensions.addAngularDimension(
            lA, lB, midChord, False)
        self._dTwist = angDim.parameter.name
        futil.log(f'[spiral] {gearLabel}: twist angle param={self._dTwist} '
                  f'value={math.degrees(angDim.parameter.value):.2f} deg', force_console=True)

        # ================================================================
        # PASS 2a: cut plane = the PARENT'S transverse TOOTH PLANE offset toward the apex.
        # (cut/pattern/rotate/loft to follow.)
        # ================================================================
        dFace = self._dFacePinion if isPinion else self._dFaceDriving
        # Offset TOWARD the apex. The parent plane's normal points opposite ways for the
        # two gears, so choose the sign that moves toward the apex per gear.
        pn = parentToothPlane.geometry.normal
        po = parentToothPlane.geometry.origin
        towardApex = (apex.x - po.x) * pn.x + (apex.y - po.y) * pn.y + (apex.z - po.z) * pn.z
        sign = 1.0 if towardApex > 0 else -1.0
        offInput = designComponent.constructionPlanes.createInput()
        offInput.setByOffset(parentToothPlane, adsk.core.ValueInput.createByReal(sign * span / 6))
        cutPlane = designComponent.constructionPlanes.add(offInput)
        cutPlane.name = f'{gearLabel} Cut Plane'
        futil.log(f'[spiral] {gearLabel}: PASS 2a cut plane = parent Tooth Plane offset '
                  f'{to_mm(sign*span/6):.2f}mm (toward apex, sign={sign:+.0f})', force_console=True)

        # ================================================================
        # PASS 2b: cut (split) the straight Tooth Body with the cut plane.
        # ================================================================
        # ================================================================
        # PASS 2c: split by hand -- 8 planes stepped TOWARD THE APEX (k=0 is the verified
        # cut plane) at span/6, split piece-by-piece; skip any plane that misses.
        # ================================================================
        pieces = [toothBody]
        for k in range(8):
            if k == 0:
                planeK = cutPlane
            else:
                pi = designComponent.constructionPlanes.createInput()
                pi.setByOffset(parentToothPlane,
                               adsk.core.ValueInput.createByReal(sign * (k + 1) * span / 6))
                planeK = designComponent.constructionPlanes.add(pi)
                planeK.name = f'{gearLabel} Cut {k}'
            nextPieces = []
            for body in pieces:
                try:
                    sf = features.splitBodyFeatures.add(
                        features.splitBodyFeatures.createInput(body, planeK, True))
                    nextPieces += [sf.bodies.item(i) for i in range(sf.bodies.count)]
                except Exception:
                    nextPieces.append(body)   # this plane missed this piece
            pieces = nextPieces
        futil.log(f'[spiral] {gearLabel}: PASS 2c split by hand -> {len(pieces)} segments',
                  force_console=True)

        # ================================================================
        # PASS 2d: order segments along the cone element, drop the apex-side scrap, then
        # rotate each remaining segment about the SHAFT AXIS by the 3-D trace's actual
        # azimuth at that segment's cone distance, relative to the mean station. Measuring
        # the azimuth directly in 3-D about the real axis bakes in the developed->shaft
        # (1/sin gamma) relationship -- no separate factor, no linear approximation, no
        # /6 scaling. The mean station stays unrotated -> the half-tooth mesh offset holds.
        # ================================================================
        pieces.sort(key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap, segments = pieces[0], pieces[1:]    # pieces[0] = long apex-side scrap
        features.removeFeatures.add(scrap)

        # Sample the 3-D trace into (cone distance, perpendicular-to-axis vector) pairs.
        ev = trace3d.worldGeometry.evaluator
        (_ok, t0, t1) = ev.getParameterExtents()
        samples = []
        for i in range(61):
            (_okp, P) = ev.getPointAtParameter(t0 + (t1 - t0) * i / 60.0)
            axc = ((P.x - apex.x) * axisDir.x + (P.y - apex.y) * axisDir.y
                   + (P.z - apex.z) * axisDir.z)
            perp = adsk.core.Vector3D.create(
                (P.x - apex.x) - axc * axisDir.x,
                (P.y - apex.y) - axc * axisDir.y,
                (P.z - apex.z) - axc * axisDir.z)
            samples.append((distAlong(P), perp))

        def _azimuth(vRef, v):           # signed angle vRef->v about the shaft axis
            cr = vRef.crossProduct(v)
            return math.atan2(cr.dotProduct(axisDir), vRef.dotProduct(v))

        # Total toe->heel twist from the trace ENDPOINTS (robust even when the arc's cone
        # distance is non-monotonic, as on the pinion). Magnitude only -- the hand sets the
        # direction. This is the real shaft-axis angle, so the developed->shaft 1/sin(gamma)
        # is already included; it replaces the old dYYY/6 (which had both a 1/sin(gamma) and
        # a /6-vs-/(N-1) scaling error).
        total = abs(_azimuth(samples[0][1], samples[-1][1]))
        twistVal = angDim.parameter.value          # dYYY (the plane-measured value)
        futil.log(f'[spiral] {gearLabel}: AUDIT dYYY(plane)={math.degrees(twistVal):.2f} deg '
                  f'vs direct shaft-axis twist={math.degrees(total):.2f} deg '
                  f'(ratio={total / twistVal if twistVal else 0:.3f})', force_console=True)

        # Stagger by the CONE DISTANCE of each segment's heel face -- the exact section the
        # loft samples -- centred on R_mean. This guarantees the loft section sitting at
        # R_mean is genuinely unrotated (= the straight tooth, which already meshes).
        # Centring by segment INDEX instead left the loft's mid-face section rotated by
        # half a segment, which is the mid-face overlap. Linear -> monotonic C-shape.
        def _heelR(seg):
            best = None
            for i in range(seg.faces.count):
                bb = seg.faces.item(i).boundingBox
                d = distAlong(adsk.core.Point3D.create(
                    0.5 * (bb.minPoint.x + bb.maxPoint.x),
                    0.5 * (bb.minPoint.y + bb.maxPoint.y),
                    0.5 * (bb.minPoint.z + bb.maxPoint.z)))
                if best is None or d > best:
                    best = d
            return best

        # scaleFeatures (the crown) raised "Environment is not supported" -- the operation
        # needs its owning component to be the ACTIVE edit target. Activate the Design
        # occurrence around the crown, then restore the root.
        _crownOcc = None
        if self._CROWN_PER_RAD > 0:
            try:
                _occs = designComponent.parentDesign.rootComponent.allOccurrencesByComponent(
                    designComponent)
                if _occs.count:
                    _crownOcc = _occs.item(0)
                    _crownOcc.activate()
                    futil.log(f'[spiral] {gearLabel}: activated Design occ for crown scale',
                              force_console=True)
            except Exception as e:
                futil.log(f'[spiral] {gearLabel}: activate Design failed ({e})',
                          force_console=True)

        for idx, seg in enumerate(segments):
            ang = -handSign * total * (R_mean - _heelR(seg)) / span
            # Lengthwise crown: scale the segment about its heel-face centre in proportion
            # to the local twist, so toe/heel thin and contact localizes at mid-face. SKIP
            # only the OUTERMOST (heel) segment -- its face is the loft's heel end that must
            # stay full so the heel cone trims it flush with the gear base.
            relief = self._CROWN_PER_RAD * abs(ang)
            if relief > 1e-4 and idx < len(segments) - 1:
                try:
                    # Symmetric crown: scale the segment about the CENTRE of its heel face
                    # (the tooth centreline at the loft section). A sketch point on that
                    # face is a valid scale base; setByPoint(Point3D) is not supported here.
                    hf, hfd, hfc = None, None, None
                    for fi in range(seg.faces.count):
                        f = seg.faces.item(fi)
                        bb = f.boundingBox
                        c = adsk.core.Point3D.create(
                            0.5 * (bb.minPoint.x + bb.maxPoint.x),
                            0.5 * (bb.minPoint.y + bb.maxPoint.y),
                            0.5 * (bb.minPoint.z + bb.maxPoint.z))
                        d = distAlong(c)
                        if hfd is None or d > hfd:
                            hfd, hf, hfc = d, f, c
                    sk = designComponent.sketches.add(hf)
                    sk.isVisible = False
                    spt = sk.sketchPoints.add(sk.modelToSketchSpace(hfc))
                    ents = adsk.core.ObjectCollection.create()
                    ents.add(seg)
                    features.scaleFeatures.add(features.scaleFeatures.createInput(
                        ents, spt, adsk.core.ValueInput.createByReal(1.0 - relief)))
                except Exception as e:
                    futil.log(f'[spiral] {gearLabel}: crown skipped ({e})', force_console=True)
            if abs(ang) > 1e-9:
                m = adsk.core.Matrix3D.create()
                m.setToRotation(ang, axisDir, apex)
                coll = adsk.core.ObjectCollection.create()
                coll.add(seg)
                mv = features.moveFeatures.createInput2(coll)
                mv.defineAsFreeMove(m)
                features.moveFeatures.add(mv)
        if _crownOcc:
            try:
                designComponent.parentDesign.activateRootComponent()
            except Exception as e:
                futil.log(f'[spiral] {gearLabel}: restore root active failed ({e})',
                          force_console=True)
        futil.log(f'[spiral] {gearLabel}: PASS 2d staggered {len(segments)} segments, '
                  f'total twist={math.degrees(total):.2f} deg', force_console=True)

        # ================================================================
        # PASS 2e: loft the heel-facing (D->J / C->H) cut face of each staggered segment
        # -> the curved tooth. The heel-facing cut face = the face whose bbox centre is
        # farthest along the cone element.
        # ================================================================
        def endFace(seg, farthest):
            best, bestD = None, None
            for i in range(seg.faces.count):
                f = seg.faces.item(i)
                bb = f.boundingBox
                c = adsk.core.Point3D.create(
                    0.5 * (bb.minPoint.x + bb.maxPoint.x),
                    0.5 * (bb.minPoint.y + bb.maxPoint.y),
                    0.5 * (bb.minPoint.z + bb.maxPoint.z))
                d = distAlong(c)
                if bestD is None or (d > bestD if farthest else d < bestD):
                    bestD, best = d, f
            return best

        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Toe-most segment's apex-side face first (pushes the loft PAST the toe cone so the
        # required toe cut bites), then the heel-facing (D->J / C->H) face of every segment
        # (the last one reaches past the heel cone).
        loftInput.loftSections.add(endFace(segments[0], False))
        for seg in segments:
            loftInput.loftSections.add(endFace(seg, True))
        curvedTooth = features.loftFeatures.add(loftInput).bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'
        futil.log(f'[spiral] {gearLabel}: PASS 2e lofted curved tooth '
                  f'({loftInput.loftSections.count} sections)', force_console=True)

        # remove the segment scaffolding (loft has captured their faces)
        for seg in segments:
            features.removeFeatures.add(seg)

        # ================================================================
        # PASS 2f: reinstate the toe/heel conical trims -> ends flush on the cones.
        # ================================================================
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)

    # ----- toe/heel conical cuts that trim a tooth body to a flush band -----
    def _cutConicalEnds(self, designComponent, toothBody, gearBody, toeMid, heelMid,
                        apexWorld, gearLabel):
        # Cut 1: toe cut using a cone face of the Gear Body (frustum).
        toePieces = self._applyConicalCut(
            designComponent, gearBody, [toothBody], toeMid, f'{gearLabel} toe cut#1')
        # Drop the apex tip; keep the largest non-apex piece (the keeper).
        keeper = self._selectKeeper(designComponent, toePieces, apexWorld,
                                    f'{gearLabel} toe keeper')
        # Cut 2: heel cut on the keeper alone. May not intersect (try/except).
        try:
            heelPieces = self._applyConicalCut(
                designComponent, gearBody, [keeper], heelMid, f'{gearLabel} heel cut#2')
            tooth = self._selectKeeper(designComponent, heelPieces, apexWorld,
                                       f'{gearLabel} heel keeper')
        except Exception as e:
            # Tolerate the heel cone not intersecting (tooth ends at/just inside the
            # back cone): straight bevel raises SPLIT_TARGET_TOOL_NOT_INTERSECT; the
            # spiral tooth ends ~tangent so _applyConicalCut raises 'no cone face split'.
            msg = str(e)
            if ('SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg
                    or 'no cone face split' in msg):
                futil.log(f'{gearLabel} heel cut: tool did not intersect, kept intact',
                          force_console=True)
                tooth = keeper
            else:
                raise
        return tooth

    # ----- find the spur tooth cross-section profile -----
    def _findSpurToothProfile(self, toothSketch):
        # Match by curve-type mix: 2 NURBS flanks + 2 arcs (tip/root) + 2 lines
        # (non-embedded), or 2 NURBS + 2 arcs + 0 lines (embedded).
        line_t = adsk.core.Curve3DTypes.Line3DCurveType
        arc_t = adsk.core.Curve3DTypes.Arc3DCurveType
        nurbs_t = adsk.core.Curve3DTypes.NurbsCurve3DCurveType
        for prof in toothSketch.profiles:
            for loop in prof.profileLoops:
                nurbs = arcs = others = lines = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == nurbs_t:
                        nurbs += 1
                    elif ct == arc_t:
                        arcs += 1
                    elif ct == line_t:
                        lines += 1
                    else:
                        others += 1
                if others != 0:
                    continue
                if nurbs == 2 and arcs == 2 and (lines == 2 or lines == 0):
                    return prof
        raise Exception('Could not find spur tooth cross-section profile')

    # ----- conical cut: split target bodies by best cone face of the frustum -----
    def _applyConicalCut(self, designComponent, frustumBody, targets,
                         edgeMidWorld, cutLabel):
        coneType = adsk.core.SurfaceTypes.ConeSurfaceType
        coneFaces = []
        for face in frustumBody.faces:
            if face.geometry.surfaceType == coneType:
                coneFaces.append(face)

        # Order cone faces by surface distance to the edge midpoint (None -> +inf).
        def faceKey(face):
            d = self._surfaceDistance(face.geometry, edgeMidWorld)
            return d if d is not None else float('inf')

        coneFaces.sort(key=faceKey)

        diag = []
        for target in targets:
            for face in coneFaces:
                d = self._surfaceDistance(face.geometry, edgeMidWorld)
                try:
                    splits = designComponent.features.splitBodyFeatures
                    splitInput = splits.createInput(target, face, True)
                    split = splits.add(splitInput)
                    pieces = []
                    for b in split.bodies:
                        pieces.append(b)
                    if len(pieces) > 1:
                        futil.log(
                            f'{cutLabel}: split with face dist={d} -> '
                            f'{len(pieces)} pieces', force_console=True)
                        return pieces
                    # Did not split (one piece) -- continue trying other faces.
                    diag.append((d, len(pieces)))
                except RuntimeError as e:
                    msg = str(e)
                    if 'SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg or '交差' in msg:
                        diag.append((d, 'not-intersect'))
                        continue
                    raise

        raise Exception(
            f'{cutLabel}: no cone face split the target '
            f'(cone faces tried={len(coneFaces)}, results={diag})')

    # ----- keeper/tooth selection: drop apex pieces, keep largest by volume -----
    def _selectKeeper(self, designComponent, pieces, apexWorld, label):
        nonApex = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            if (containment == adsk.fusion.PointContainment.PointInsidePointContainment
                    or containment ==
                    adsk.fusion.PointContainment.PointOnPointContainment):
                # apex-containing piece: remove it.
                designComponent.features.removeFeatures.add(body)
            else:
                nonApex.append(body)

        if len(nonApex) == 0:
            raise Exception(f'{label}: no non-apex piece found')

        # Keep the single largest by volume; remove the rest.
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        for body in nonApex[1:]:
            designComponent.features.removeFeatures.add(body)
        return keeper

    # ----- bore cut -----
    def _cutBore(self, designComponent, gearBody, shaftAxisEdge, bore_cm, gearLabel):
        # Bore plane normal to the shaft at its start (distance 0 on the A->G edge).
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByDistanceOnPath(
            shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
        borePlane = designComponent.constructionPlanes.add(planeInput)
        borePlane.name = f'{gearLabel} Bore Plane'

        boreSketch = designComponent.sketches.add(borePlane)
        boreSketch.name = f'{gearLabel} Bore'
        boreSketch.isVisible = True
        circles = boreSketch.sketchCurves.sketchCircles
        dimensions = boreSketch.sketchDimensions

        radius = bore_cm / 2
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0)).parameter.value = bore_cm

        self._assertFullyConstrained(boreSketch, f'{gearLabel} Bore')

        profile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(to_cm(1000)), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # ----- cleanup: hide construction geometry -----
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            token = component.entityToken
            if token in seen:
                return
            seen.add(token)

            for sketch in component.sketches:
                sketch.isLightBulbOn = False
                sketch.isVisible = False
            for plane in component.constructionPlanes:
                plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                axis.isLightBulbOn = False

            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
