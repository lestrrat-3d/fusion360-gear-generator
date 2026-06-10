import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
import adsk.core, adsk.fusion

from .spurgear import SpurGearInvoluteToothDesignGenerator


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

# Hand-of-spiral list-item strings (reproduced surface)
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'

# Pressure angle is not a bevel dialog input; bevel hardcodes 20 deg.
_PRESSURE_ANGLE_DEG = 20.0

# Number of involute sample steps the spur drawer reads.
_INVOLUTE_STEPS = 15

# Tunable crown-relief constant (0 disables the lengthwise crown).
_CROWN_PER_RAD = 0.5

# Pinion extra mesh phase, in tooth-fractions (0 by default).
_PINION_MESH_PHASE_TEETH = 0.0


# ===========================================================================
# Value-wrapper + virtual spur proxy
# ===========================================================================
class _Val:
    """Tiny wrapper exposing a `.value` attribute, mimicking a UserParameter."""
    def __init__(self, value):
        self.value = value


class _VirtualSpurProxy:
    """A fake spur `parent` so the borrowed spur tooth generator can run
    without registering Fusion user parameters. Precomputes, in internal cm,
    exactly the keys the spur drawer reads via parent.getParameter(name).value.
    """
    def __init__(self, module_mm, virtualTeeth):
        # _lastToothEmbedded is an OUTPUT the spur generator writes back.
        self._lastToothEmbedded = False

        pressureAngle = math.radians(_PRESSURE_ANGLE_DEG)
        teeth = int(virtualTeeth)

        # Standard spur formulas, in mm, then converted to internal cm for the
        # radii/diameters the spur tooth generator consumes.
        pitch_mm = teeth * module_mm
        base_mm = pitch_mm * math.cos(pressureAngle)
        root_mm = pitch_mm - 2.5 * module_mm
        tip_mm = pitch_mm + 2.0 * module_mm

        self._params = {
            'Module': _Val(to_cm(module_mm)),
            'ToothNumber': _Val(teeth),
            'PressureAngle': _Val(pressureAngle),
            'PitchCircleDiameter': _Val(to_cm(pitch_mm)),
            'PitchCircleRadius': _Val(to_cm(pitch_mm / 2.0)),
            'BaseCircleDiameter': _Val(to_cm(base_mm)),
            'BaseCircleRadius': _Val(to_cm(base_mm / 2.0)),
            'RootCircleDiameter': _Val(to_cm(root_mm)),
            'RootCircleRadius': _Val(to_cm(root_mm / 2.0)),
            'TipCircleDiameter': _Val(to_cm(tip_mm)),
            'TipCircleRadius': _Val(to_cm(tip_mm / 2.0)),
            'InvoluteSteps': _Val(_INVOLUTE_STEPS),
        }

    def getParameter(self, name):
        return self._params[name]


# ===========================================================================
# Dialog inputs configurator
# ===========================================================================
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (selection) -- first so it wins auto-focus.
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Select the plane the driving gear sits flush against')
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

        # 3. Parent Component (selection) -- pre-selected to root.
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

        # 10. Enable Bore
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

        # Initial conditional visibility (default psi = 35 deg -> both shown).
        cls._updateSpiralInputVisibility(inputs)

    @classmethod
    def handle_input_changed(cls, args):
        cls._updateSpiralInputVisibility(args.inputs)

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs):
        spiralInput = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)
        handInput = inputs.itemById(INPUT_ID_HAND)
        cutterInput = inputs.itemById(INPUT_ID_CUTTER_RADIUS)
        if spiralInput is None or handInput is None or cutterInput is None:
            return
        try:
            # value is internal radians.
            value = spiralInput.value
        except:
            # A half-typed expression can raise mid-edit; leave both shown.
            handInput.isVisible = True
            cutterInput.isVisible = True
            return
        show = (value > 0)
        handInput.isVisible = show
        cutterInput.isVisible = show


# ===========================================================================
# Generator (standalone -- does NOT subclass base.Generator)
# ===========================================================================
class BevelGearGenerator:
    def __init__(self, design):
        self.design = design
        self.bevelOccurrence = None

    # -----------------------------------------------------------------------
    # entry point
    # -----------------------------------------------------------------------
    def generate(self, inputs):
        (parentComponent, targetPlane, centerPoint, module,
         drivingTeeth, pinionTeeth, shaftAngleDeg) = self._readInputs(inputs)

        self._module_mm = module
        self._drivingTeeth = drivingTeeth
        self._pinionTeeth = pinionTeeth
        self._shaftAngleDeg = shaftAngleDeg
        self._shaftAngle_rad = math.radians(shaftAngleDeg)

        # Resolve pitch diameters (Python, internal cm).
        drivingPitchDiameter_cm = to_cm(module * drivingTeeth)
        pinionPitchDiameter_cm = to_cm(module * pinionTeeth)
        self._drivingPitchDiameter_cm = drivingPitchDiameter_cm
        self._pinionPitchDiameter_cm = pinionPitchDiameter_cm

        # Resolve bore diameters (0 -> auto = pitch diameter / 4).
        if self._drivingBore_cm > 0:
            self._drivingBoreResolved_cm = self._drivingBore_cm
        else:
            self._drivingBoreResolved_cm = drivingPitchDiameter_cm / 4.0
        if self._pinionBore_cm > 0:
            self._pinionBoreResolved_cm = self._pinionBore_cm
        else:
            self._pinionBoreResolved_cm = pinionPitchDiameter_cm / 4.0

        # Cone distance (internal cm).
        coneDistance_cm = to_cm(
            math.sqrt((module * drivingTeeth) ** 2 + (module * pinionTeeth) ** 2))
        self._coneDistance_cm = coneDistance_cm

        # Pitch cone half-angles (gamma_p pinion, gamma_g driving).
        Sigma = self._shaftAngle_rad
        PPD = pinionPitchDiameter_cm
        DPD = drivingPitchDiameter_cm
        self._gamma_p = math.atan2(
            math.sin(Sigma) * PPD, (DPD + PPD * math.cos(Sigma)))
        self._gamma_g = Sigma - self._gamma_p

        # ----- build the occurrence tree -----
        bevelOcc = parentComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        bevelOcc.component.name = 'Bevel Gear'
        self.bevelOccurrence = bevelOcc
        bevelComponent = bevelOcc.component

        designOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        designOcc.component.name = 'Design'
        self._designOccurrence = designOcc
        designComponent = designOcc.component
        self._designComponent = designComponent

        # ----- geometry -----
        anchorLine = self._buildAnchorSketch(designComponent, targetPlane, centerPoint)

        self._buildGearProfiles(
            designComponent, targetPlane, anchorLine, bevelComponent)

        # ----- cleanup -----
        self._hideConstructionGeometry(bevelComponent)

    def deleteComponent(self):
        if self.bevelOccurrence:
            self.bevelOccurrence.deleteMe()
            self.bevelOccurrence = None

    # -----------------------------------------------------------------------
    # input reading & validation
    # -----------------------------------------------------------------------
    def _readInputs(self, inputs):
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

        unitsManager = self.design.unitsManager

        def rawValue(inputId, units):
            inp = inputs.itemById(inputId)
            return unitsManager.evaluateExpression(inp.expression, units)

        # Module read with '' -> raw number meaning millimetres.
        module = rawValue(INPUT_ID_MODULE, '')

        drivingTeeth = int(round(rawValue(INPUT_ID_DRIVING_TEETH, '')))
        pinionTeeth = int(round(rawValue(INPUT_ID_PINION_TEETH, '')))
        if drivingTeeth < 3:
            raise Exception('Driving Gear Teeth must be at least 3')
        if pinionTeeth < 3:
            raise Exception('Pinion Gear Teeth must be at least 3')

        # Shaft angle read in radians; convert to degrees for the range check.
        shaftAngle_rad = rawValue(INPUT_ID_SHAFT_ANGLE, 'deg')
        shaftAngleDeg = math.degrees(shaftAngle_rad)
        if shaftAngleDeg < 30.0 or shaftAngleDeg > 150.0:
            raise Exception('Shaft Angle must be between 30 and 150 degrees')

        # 'mm' inputs come back already in internal cm -- use as-is.
        self._drivingBaseHeight_cm = rawValue(INPUT_ID_DRIVING_BASE_HEIGHT, 'mm')
        self._pinionBaseHeight_cm = rawValue(INPUT_ID_PINION_BASE_HEIGHT, 'mm')
        (boreEnable, _) = get_boolean(inputs, INPUT_ID_BORE_ENABLE)
        self._boreEnable = boreEnable
        self._drivingBore_cm = rawValue(INPUT_ID_DRIVING_BORE, 'mm')
        self._pinionBore_cm = rawValue(INPUT_ID_PINION_BORE, 'mm')
        self._faceWidth_cm = rawValue(INPUT_ID_FACE_WIDTH, 'mm')
        self._toothSpacing_cm = rawValue(INPUT_ID_TOOTH_SPACING, 'mm')

        if self._drivingBaseHeight_cm < 0:
            raise Exception('Driving Gear Base Height must be non-negative')
        if self._pinionBaseHeight_cm < 0:
            raise Exception('Pinion Gear Base Height must be non-negative')
        if self._drivingBore_cm < 0:
            raise Exception('Driving Gear Bore Diameter must be non-negative')
        if self._pinionBore_cm < 0:
            raise Exception('Pinion Gear Bore Diameter must be non-negative')
        if self._faceWidth_cm < 0:
            raise Exception('Face Width must be non-negative')
        if self._toothSpacing_cm < 0:
            raise Exception('Tooth Spacing must be non-negative')

        # Spiral inputs.
        spiral_rad = rawValue(INPUT_ID_SPIRAL_ANGLE, 'deg')
        spiralDeg = math.degrees(spiral_rad)
        if spiralDeg < 0.0 or spiralDeg >= 60.0:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')
        self._spiralAngle_rad = spiral_rad

        handInput = inputs.itemById(INPUT_ID_HAND)
        if handInput.selectedItem is not None:
            self._hand = handInput.selectedItem.name
        else:
            self._hand = _HAND_RIGHT

        self._cutterRadius_cm = rawValue(INPUT_ID_CUTTER_RADIUS, 'mm')
        if self._cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        return (parentComponent, targetPlane, centerPoint, module,
                drivingTeeth, pinionTeeth, shaftAngleDeg)

    # -----------------------------------------------------------------------
    # small geometry helpers
    # -----------------------------------------------------------------------
    def _pointWorldGeometry(self, entity):
        """World Point3D for a SketchPoint (via .worldGeometry) or a
        ConstructionPoint (via .geometry)."""
        if entity.objectType == adsk.fusion.SketchPoint.classType():
            return entity.worldGeometry
        return entity.geometry

    def _combine(self, base, a, e1, b=0.0, e2=None):
        """World point = base + a*e1 (+ b*e2). Lengths in cm."""
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _vecFromTo(self, a, b):
        return adsk.core.Vector3D.create(b.x - a.x, b.y - a.y, b.z - a.z)

    def _normalize(self, v):
        out = v.copy()
        out.normalize()
        return out

    def _assertFullyConstrained(self, sketch):
        if not sketch.isFullyConstrained:
            raise Exception(
                f'Sketch "{sketch.name}" is not fully constrained '
                f'(free degrees of freedom remain)')

    # -----------------------------------------------------------------------
    # 1: Anchor Sketch
    # -----------------------------------------------------------------------
    def _buildAnchorSketch(self, designComponent, targetPlane, centerPoint):
        sketch = designComponent.sketches.add(targetPlane)
        sketch.name = 'Anchor Sketch'
        sketch.isVisible = True

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        projected = sketch.project(centerPoint)
        projectedCenter = projected.item(0)
        self._anchorCenterPoint = projectedCenter

        cg = projectedCenter.geometry
        # ~10mm reference line through the projected center.
        half = to_cm(5)
        p0 = adsk.core.Point3D.create(cg.x - half, cg.y, 0)
        p1 = adsk.core.Point3D.create(cg.x + half, cg.y, 0)
        anchorLine = lines.addByTwoPoints(p0, p1)

        constraints.addCoincident(projectedCenter, anchorLine)
        constraints.addMidPoint(projectedCenter, anchorLine)
        dimensions.addDistanceDimension(
            anchorLine.startSketchPoint, anchorLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(cg.x, cg.y + half, 0))
        constraints.addHorizontal(anchorLine)

        self._assertFullyConstrained(sketch)
        return anchorLine

    # -----------------------------------------------------------------------
    # 2 (+3 + body) : Gear Profiles
    # -----------------------------------------------------------------------
    def _buildGearProfiles(self, designComponent, targetPlane, anchorLine,
                           bevelComponent):
        # Plane that includes the Anchor Line, perpendicular to targetPlane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
        gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
        gearProfilesPlane.name = 'Gear Profiles Plane'
        self._gearProfilesPlane = gearProfilesPlane

        sketch = designComponent.sketches.add(gearProfilesPlane)
        sketch.name = 'Gear Profiles'
        sketch.isVisible = True
        self._gearProfilesSketch = sketch

        constraints = sketch.geometricConstraints
        dimensions = sketch.sketchDimensions
        lines = sketch.sketchCurves.sketchLines

        # Project the anchor-sketch center SketchPoint (not the raw external).
        projAnchor = sketch.project(self._anchorCenterPoint)
        projCenter = projAnchor.item(0)
        # Project the anchor line to get its 2-D direction.
        projLineColl = sketch.project(anchorLine)
        projLine = projLineColl.item(0)

        c = projCenter.geometry
        lp0 = projLine.startSketchPoint.geometry
        lp1 = projLine.endSketchPoint.geometry
        dvec = adsk.core.Vector3D.create(lp1.x - lp0.x, lp1.y - lp0.y, 0)
        dvec.normalize()
        # in-plane perpendicular
        perp = adsk.core.Vector3D.create(-dvec.y, dvec.x, 0)

        # Pick perp sign by the target-plane normal (one bit, grow toward normal).
        targetNormal = get_normal(targetPlane)
        targetNormal.normalize()
        # Map perp (sketch-local) into a world direction to compare against normal.
        perpWorldA = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x + perp.x, c.y + perp.y, 0))
        perpWorldO = sketch.sketchToModelSpace(
            adsk.core.Point3D.create(c.x, c.y, 0))
        perpWorld = adsk.core.Vector3D.create(
            perpWorldA.x - perpWorldO.x,
            perpWorldA.y - perpWorldO.y,
            perpWorldA.z - perpWorldO.z)
        if perpWorld.dotProduct(targetNormal) < 0:
            perp = adsk.core.Vector3D.create(-perp.x, -perp.y, 0)

        DPD = self._drivingPitchDiameter_cm
        PPD = self._pinionPitchDiameter_cm
        Sigma = self._shaftAngle_rad

        # Apex at c + perp*DPD (sketch-local).
        apexSeed = adsk.core.Point3D.create(
            c.x + perp.x * DPD, c.y + perp.y * DPD, 0)

        # center -> apex construction line (coincident style).
        centerToApex = lines.addByTwoPoints(
            adsk.core.Point3D.create(c.x, c.y, 0), apexSeed)
        centerToApex.isConstruction = True
        constraints.addCoincident(centerToApex.startSketchPoint, projCenter)
        constraints.addPerpendicular(centerToApex, projLine)
        apexPoint = centerToApex.endSketchPoint
        self._apexSketchPoint = apexPoint

        apex2d = apexPoint.geometry

        # Closed-form cone geometry to seed along-shaft lengths.
        gamma_p = self._gamma_p
        gamma_g = self._gamma_g
        R_seed = (PPD / 2.0) / math.sin(gamma_p)
        lenA = R_seed * math.cos(gamma_p)   # |Apex->A|
        lenB = R_seed * math.cos(gamma_g)   # |Apex->B|

        # Driving Gear Shaft Axis: from apex toward the anchor line (-perp), end = B.
        bSeed = adsk.core.Point3D.create(
            apex2d.x - perp.x * lenB, apex2d.y - perp.y * lenB, 0)
        drivingShaftAxis = lines.addByTwoPoints(apex2d, bSeed)
        drivingShaftAxis.isConstruction = True
        constraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)
        constraints.addParallel(drivingShaftAxis, centerToApex)
        pointB = drivingShaftAxis.endSketchPoint

        # Pinion Gear Shaft Axis: driving direction rotated about apex by +/-Sigma.
        drivDir = adsk.core.Vector3D.create(-perp.x, -perp.y, 0)

        def rot2d(vx, vy, theta):
            cc = math.cos(theta)
            ss = math.sin(theta)
            return (vx * cc - vy * ss, vx * ss + vy * cc)

        (apx, apy) = rot2d(drivDir.x, drivDir.y, Sigma)
        (amx, amy) = rot2d(drivDir.x, drivDir.y, -Sigma)
        candPlus = adsk.core.Point3D.create(
            apex2d.x + apx * lenA, apex2d.y + apy * lenA, 0)
        candMinus = adsk.core.Point3D.create(
            apex2d.x + amx * lenA, apex2d.y + amy * lenA, 0)
        # Keep the candidate with the greater X coordinate.
        if candPlus.x >= candMinus.x:
            aSeed = candPlus
            sigmaSign = 1.0
        else:
            aSeed = candMinus
            sigmaSign = -1.0
        self._sigmaSign = sigmaSign

        pinionShaftAxis = lines.addByTwoPoints(apex2d, aSeed)
        pinionShaftAxis.isConstruction = True
        constraints.addCoincident(pinionShaftAxis.startSketchPoint, apexPoint)
        pointA = pinionShaftAxis.endSketchPoint
        # Angular dimension Sigma between the two shaft axes (text on the wedge).
        midDir = adsk.core.Vector3D.create(
            (drivDir.x + (aSeed.x - apex2d.x)),
            (drivDir.y + (aSeed.y - apex2d.y)), 0)
        if midDir.length > 1e-9:
            midDir.normalize()
        angTextLen = lenA * 0.4
        angText = adsk.core.Point3D.create(
            apex2d.x + midDir.x * angTextLen, apex2d.y + midDir.y * angTextLen, 0)
        dimensions.addAngularDimension(
            drivingShaftAxis, pinionShaftAxis, angText).parameter.value = Sigma

        # A->Apex2 : perpendicular drop from A, length PPD/2, toward Apex2 side.
        aGeom = pointA.geometry
        # direction perpendicular to pinion shaft, toward apex-side (toward anchor).
        psDir = adsk.core.Vector3D.create(
            aGeom.x - apex2d.x, aGeom.y - apex2d.y, 0)
        psDir.normalize()
        dropAdir = adsk.core.Vector3D.create(-psDir.y, psDir.x, 0)
        # choose the sense pointing toward the driving shaft (toward Apex2)
        toDriving = adsk.core.Vector3D.create(
            apex2d.x - aGeom.x, apex2d.y - aGeom.y, 0)
        if dropAdir.dotProduct(toDriving) < 0:
            dropAdir = adsk.core.Vector3D.create(-dropAdir.x, -dropAdir.y, 0)
        apex2SeedFromA = adsk.core.Point3D.create(
            aGeom.x + dropAdir.x * (PPD / 2.0),
            aGeom.y + dropAdir.y * (PPD / 2.0), 0)
        aToApex2 = lines.addByTwoPoints(aGeom, apex2SeedFromA)
        aToApex2.isConstruction = True
        constraints.addCoincident(aToApex2.startSketchPoint, pointA)
        constraints.addPerpendicular(aToApex2, pinionShaftAxis)
        dimensions.addDistanceDimension(
            aToApex2.startSketchPoint, aToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (aGeom.x + apex2SeedFromA.x) / 2,
                (aGeom.y + apex2SeedFromA.y) / 2, 0)).value = PPD / 2.0

        # B->Apex2 : perpendicular drop from B, length DPD/2, toward Apex2 side.
        bGeom = pointB.geometry
        bsDir = adsk.core.Vector3D.create(
            bGeom.x - apex2d.x, bGeom.y - apex2d.y, 0)
        bsDir.normalize()
        dropBdir = adsk.core.Vector3D.create(-bsDir.y, bsDir.x, 0)
        toApexFromB = adsk.core.Vector3D.create(
            apex2d.x - bGeom.x, apex2d.y - bGeom.y, 0)
        if dropBdir.dotProduct(toApexFromB) < 0:
            dropBdir = adsk.core.Vector3D.create(-dropBdir.x, -dropBdir.y, 0)
        apex2SeedFromB = adsk.core.Point3D.create(
            bGeom.x + dropBdir.x * (DPD / 2.0),
            bGeom.y + dropBdir.y * (DPD / 2.0), 0)
        bToApex2 = lines.addByTwoPoints(bGeom, apex2SeedFromB)
        bToApex2.isConstruction = True
        constraints.addCoincident(bToApex2.startSketchPoint, pointB)
        constraints.addPerpendicular(bToApex2, drivingShaftAxis)
        dimensions.addDistanceDimension(
            bToApex2.startSketchPoint, bToApex2.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (bGeom.x + apex2SeedFromB.x) / 2,
                (bGeom.y + apex2SeedFromB.y) / 2, 0)).value = DPD / 2.0

        # Apex 2 = coincidence of the two drop endpoints.
        constraints.addCoincident(
            aToApex2.endSketchPoint, bToApex2.endSketchPoint)
        apex2Point = aToApex2.endSketchPoint
        self._apex2SketchPoint = apex2Point

        # Pitch Line (Apex -> Apex2).
        pitchLine = lines.addByTwoPoints(apex2d, apex2Point.geometry)
        pitchLine.isConstruction = True
        constraints.addCoincident(pitchLine.startSketchPoint, apexPoint)
        constraints.addCoincident(pitchLine.endSketchPoint, apex2Point)

        # Dedendum lines from Apex2 (length 1.25*Module each), perpendicular to
        # the Pitch Line.  D toward anchor line (driving), C away (pinion).
        ded_cm = to_cm(1.25 * self._module_mm)
        apex2g = apex2Point.geometry
        pitchDir = adsk.core.Vector3D.create(
            apex2g.x - apex2d.x, apex2g.y - apex2d.y, 0)
        pitchDir.normalize()
        dedPerp = adsk.core.Vector3D.create(-pitchDir.y, pitchDir.x, 0)
        # Decide which perpendicular sense points toward the anchor line (= -perp).
        towardAnchor = adsk.core.Vector3D.create(-perp.x, -perp.y, 0)
        if dedPerp.dotProduct(towardAnchor) >= 0:
            ddir_D = dedPerp
            ddir_C = adsk.core.Vector3D.create(-dedPerp.x, -dedPerp.y, 0)
        else:
            ddir_D = adsk.core.Vector3D.create(-dedPerp.x, -dedPerp.y, 0)
            ddir_C = dedPerp

        dSeed = adsk.core.Point3D.create(
            apex2g.x + ddir_D.x * ded_cm, apex2g.y + ddir_D.y * ded_cm, 0)
        drivingDedendum = lines.addByTwoPoints(apex2g, dSeed)
        drivingDedendum.isConstruction = True
        constraints.addCoincident(drivingDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(drivingDedendum, pitchLine)
        dimensions.addDistanceDimension(
            drivingDedendum.startSketchPoint, drivingDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (apex2g.x + dSeed.x) / 2, (apex2g.y + dSeed.y) / 2, 0)).value = ded_cm
        pointD = drivingDedendum.endSketchPoint

        cSeed = adsk.core.Point3D.create(
            apex2g.x + ddir_C.x * ded_cm, apex2g.y + ddir_C.y * ded_cm, 0)
        pinionDedendum = lines.addByTwoPoints(apex2g, cSeed)
        pinionDedendum.isConstruction = True
        constraints.addCoincident(pinionDedendum.startSketchPoint, apex2Point)
        constraints.addPerpendicular(pinionDedendum, pitchLine)
        dimensions.addDistanceDimension(
            pinionDedendum.startSketchPoint, pinionDedendum.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (apex2g.x + cSeed.x) / 2, (apex2g.y + cSeed.y) / 2, 0)).value = ded_cm
        pointC = pinionDedendum.endSketchPoint

        # Root Axes : Apex->D (driving) and Apex->C (pinion).
        drivingRootAxis = lines.addByTwoPoints(apex2d, pointD.geometry)
        drivingRootAxis.isConstruction = True
        constraints.addCoincident(drivingRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(drivingRootAxis.endSketchPoint, pointD)

        pinionRootAxis = lines.addByTwoPoints(apex2d, pointC.geometry)
        pinionRootAxis.isConstruction = True
        constraints.addCoincident(pinionRootAxis.startSketchPoint, apexPoint)
        constraints.addCoincident(pinionRootAxis.endSketchPoint, pointC)

        module_cm = to_cm(self._module_mm)

        # ----- module-length extensions and frustum lattice -----
        def collinearExtension(fromPoint, alongLine, refDir, refStartPoint):
            """Create a module-length construction line from `fromPoint`
            collinear with `alongLine`; return the new line (and its endpoint)."""
            fg = fromPoint.geometry
            seed = adsk.core.Point3D.create(
                fg.x + refDir.x * module_cm, fg.y + refDir.y * module_cm, 0)
            ln = lines.addByTwoPoints(fg, seed)
            ln.isConstruction = True
            constraints.addCoincident(ln.startSketchPoint, fromPoint)
            constraints.addCollinear(ln, alongLine)
            return ln

        # direction Apex->A (away from apex), Apex->B (away from apex)
        dirApexA = self._normalize(self._vecFromTo(apex2d, pointA.geometry))
        dirApexB = self._normalize(self._vecFromTo(apex2d, pointB.geometry))

        # A->E along Apex->A
        lineAE = collinearExtension(pointA, pinionShaftAxis, dirApexA, pointA)
        pointE = lineAE.endSketchPoint
        # C->E perpendicular to A->E
        cToE = lines.addByTwoPoints(pointC.geometry, pointE.geometry)
        cToE.isConstruction = True
        constraints.addCoincident(cToE.startSketchPoint, pointC)
        constraints.addCoincident(cToE.endSketchPoint, pointE)
        constraints.addPerpendicular(lineAE, cToE)

        # B->F along Apex->B
        lineBF = collinearExtension(pointB, drivingShaftAxis, dirApexB, pointB)
        pointF = lineBF.endSketchPoint
        # D->F perpendicular to B->F
        dToF = lines.addByTwoPoints(pointD.geometry, pointF.geometry)
        dToF.isConstruction = True
        constraints.addCoincident(dToF.startSketchPoint, pointD)
        constraints.addCoincident(dToF.endSketchPoint, pointF)
        constraints.addPerpendicular(lineBF, dToF)

        # E->G along A->E
        lineEG = collinearExtension(pointE, lineAE, dirApexA, pointE)
        pointG = lineEG.endSketchPoint

        # C->H collinear with Apex2->C (the pinion dedendum line), length module
        dirApex2C = self._normalize(self._vecFromTo(apex2g, pointC.geometry))
        cgGeom = pointC.geometry
        hSeed = adsk.core.Point3D.create(
            cgGeom.x + dirApex2C.x * module_cm,
            cgGeom.y + dirApex2C.y * module_cm, 0)
        lineCH = lines.addByTwoPoints(cgGeom, hSeed)
        lineCH.isConstruction = True
        constraints.addCoincident(lineCH.startSketchPoint, pointC)
        constraints.addCollinear(lineCH, pinionDedendum)
        pointH = lineCH.endSketchPoint

        # G->H line, E->G perpendicular H->G
        gToH = lines.addByTwoPoints(pointG.geometry, pointH.geometry)
        gToH.isConstruction = True
        constraints.addCoincident(gToH.startSketchPoint, pointG)
        constraints.addCoincident(gToH.endSketchPoint, pointH)
        constraints.addPerpendicular(lineEG, gToH)

        # F->I along B->F
        lineFI = collinearExtension(pointF, lineBF, dirApexB, pointF)
        pointI = lineFI.endSketchPoint

        # D->J collinear with Apex2->D, length module
        dirApex2D = self._normalize(self._vecFromTo(apex2g, pointD.geometry))
        dgGeom = pointD.geometry
        jSeed = adsk.core.Point3D.create(
            dgGeom.x + dirApex2D.x * module_cm,
            dgGeom.y + dirApex2D.y * module_cm, 0)
        lineDJ = lines.addByTwoPoints(dgGeom, jSeed)
        lineDJ.isConstruction = True
        constraints.addCoincident(lineDJ.startSketchPoint, pointD)
        constraints.addCollinear(lineDJ, drivingDedendum)
        pointJ = lineDJ.endSketchPoint

        # I->J line, F->I perpendicular J->I
        iToJ = lines.addByTwoPoints(pointI.geometry, pointJ.geometry)
        iToJ.isConstruction = True
        constraints.addCoincident(iToJ.startSketchPoint, pointI)
        constraints.addCoincident(iToJ.endSketchPoint, pointJ)
        constraints.addPerpendicular(lineFI, iToJ)

        # Driving base height offset dim: between B->Apex2 drop and J->I.
        if self._drivingBaseHeight_cm > 0:
            drivingBaseHeight_cm = self._drivingBaseHeight_cm
        else:
            drivingBaseHeight_cm = to_cm(
                self._module_mm * self._drivingTeeth / 8.0)
        jiMid = iToJ.startSketchPoint.geometry
        dimensions.addOffsetDimension(
            bToApex2, iToJ,
            adsk.core.Point3D.create(jiMid.x, jiMid.y, 0)).parameter.value = \
            drivingBaseHeight_cm

        # Pinion base height offset dim: between A->Apex2 drop and G->H.
        if self._pinionBaseHeight_cm > 0:
            pinionBaseHeight_cm = self._pinionBaseHeight_cm
        else:
            pinionBaseHeight_cm = drivingBaseHeight_cm * (
                self._pinionTeeth / self._drivingTeeth)
        ghMid = gToH.startSketchPoint.geometry
        dimensions.addOffsetDimension(
            aToApex2, gToH,
            adsk.core.Point3D.create(ghMid.x, ghMid.y, 0)).parameter.value = \
            pinionBaseHeight_cm

        # A->G line.
        aToG = lines.addByTwoPoints(pointA.geometry, pointG.geometry)
        aToG.isConstruction = False
        constraints.addCoincident(aToG.startSketchPoint, pointA)
        constraints.addCoincident(aToG.endSketchPoint, pointG)

        # Constrain Point I with center point.
        constraints.addCoincident(pointI, projCenter)

        # ----- K (pinion) -----
        dirApexAaway = dirApexA  # away from apex along Apex->A
        gGeom = pointG.geometry
        kSeed = adsk.core.Point3D.create(
            gGeom.x + dirApexAaway.x * module_cm,
            gGeom.y + dirApexAaway.y * module_cm, 0)
        gToK = lines.addByTwoPoints(gGeom, kSeed)
        gToK.isConstruction = True
        constraints.addCoincident(gToK.startSketchPoint, pointG)
        pointK = gToK.endSketchPoint
        constraints.addCoincident(pointK, pinionShaftAxis)
        constraints.addCoincident(pointK, pinionDedendum)
        # C->K reference line.
        cToK = lines.addByTwoPoints(pointC.geometry, pointK.geometry)
        cToK.isConstruction = True
        constraints.addCoincident(cToK.startSketchPoint, pointC)
        constraints.addCoincident(cToK.endSketchPoint, pointK)

        # K' tooth-center (Tooth Spacing offset on the pinion dedendum line).
        pinionCenterRefLine = cToK
        if self._toothSpacing_cm > 0:
            # direction away from C along the dedendum line (Apex2->C extended).
            kGeom = pointK.geometry
            awayFromC = self._normalize(self._vecFromTo(pointC.geometry, kGeom))
            kpSeed = adsk.core.Point3D.create(
                kGeom.x + awayFromC.x * self._toothSpacing_cm,
                kGeom.y + awayFromC.y * self._toothSpacing_cm, 0)
            kToKp = lines.addByTwoPoints(kGeom, kpSeed)
            kToKp.isConstruction = True
            constraints.addCoincident(kToKp.startSketchPoint, pointK)
            pointKp = kToKp.endSketchPoint
            constraints.addCoincident(pointKp, pinionDedendum)
            dimensions.addDistanceDimension(
                kToKp.startSketchPoint, kToKp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (kGeom.x + kpSeed.x) / 2,
                    (kGeom.y + kpSeed.y) / 2, 0)).value = self._toothSpacing_cm
            cToKp = lines.addByTwoPoints(pointC.geometry, pointKp.geometry)
            cToKp.isConstruction = True
            constraints.addCoincident(cToKp.startSketchPoint, pointC)
            constraints.addCoincident(cToKp.endSketchPoint, pointKp)
            pinionCenterRefLine = cToKp
        else:
            pointKp = pointK

        # ----- Maximum Face Width (from SOLVED geometry) -----
        def perpDistance(pt, la, lb):
            # perpendicular distance from pt to line through la, lb (2-D).
            ax, ay = la.x, la.y
            bx, by = lb.x, lb.y
            px, py = pt.x, pt.y
            dx, dy = bx - ax, by - ay
            denom = math.hypot(dx, dy)
            if denom < 1e-12:
                return math.hypot(px - ax, py - ay)
            return abs(dx * (ay - py) - dy * (ax - px)) / denom

        gA = pointA.geometry
        gB = pointB.geometry
        gC = pointC.geometry
        gD = pointD.geometry
        gH = pointH.geometry
        gJ = pointJ.geometry
        distA = perpDistance(gA, gC, gH)   # A to line C-H (pinion dedendum)
        distB = perpDistance(gB, gD, gJ)   # B to line D-J (driving dedendum)
        maxFaceWidth_cm = 0.95 * min(distA, distB)

        # Resolve Face Width with the cap.
        if self._faceWidth_cm > 0:
            if self._faceWidth_cm > maxFaceWidth_cm:
                raise Exception(
                    'Face Width exceeds the maximum allowed '
                    f'({to_mm(maxFaceWidth_cm):.4f} mm); reduce it')
            faceWidth_cm = self._faceWidth_cm
        else:
            faceWidth_cm = min(self._coneDistance_cm / 6.0, maxFaceWidth_cm)
        self._faceWidthResolved_cm = faceWidth_cm

        # ----- M->N (pinion toe) -----
        # Seed M near midpoint of Apex->C, N slid along C->H toward A->Apex2.
        apex2dG = apexPoint.geometry
        mSeed = adsk.core.Point3D.create(
            (apex2dG.x + gC.x) / 2.0, (apex2dG.y + gC.y) / 2.0, 0)
        dirCH = self._normalize(self._vecFromTo(gC, gH))
        distMtoA = math.hypot(mSeed.x - gA.x, mSeed.y - gA.y)
        nSeed = adsk.core.Point3D.create(
            mSeed.x + dirCH.x * distMtoA, mSeed.y + dirCH.y * distMtoA, 0)
        lineMN = lines.addByTwoPoints(mSeed, nSeed)
        lineMN.isConstruction = True
        pointM = lineMN.startSketchPoint
        pointN = lineMN.endSketchPoint
        constraints.addCoincident(pointM, pinionRootAxis)
        constraints.addCoincident(pointN, aToApex2)
        constraints.addParallel(lineMN, lineCH)
        mnMid = adsk.core.Point3D.create(
            (mSeed.x + nSeed.x) / 2, (mSeed.y + nSeed.y) / 2, 0)
        dimensions.addOffsetDimension(
            lineCH, lineMN, mnMid).parameter.value = faceWidth_cm
        # M->C and N->A.
        mToC = lines.addByTwoPoints(pointM.geometry, pointC.geometry)
        mToC.isConstruction = True
        constraints.addCoincident(mToC.startSketchPoint, pointM)
        constraints.addCoincident(mToC.endSketchPoint, pointC)
        nToA = lines.addByTwoPoints(pointN.geometry, pointA.geometry)
        nToA.isConstruction = True
        constraints.addCoincident(nToA.startSketchPoint, pointN)
        constraints.addCoincident(nToA.endSketchPoint, pointA)

        # ----- L (driving) -----
        iGeom = pointI.geometry
        lSeed = adsk.core.Point3D.create(
            iGeom.x + dirApexB.x * module_cm,
            iGeom.y + dirApexB.y * module_cm, 0)
        iToL = lines.addByTwoPoints(iGeom, lSeed)
        iToL.isConstruction = True
        constraints.addCoincident(iToL.startSketchPoint, pointI)
        pointL = iToL.endSketchPoint
        constraints.addCoincident(pointL, drivingShaftAxis)
        constraints.addCoincident(pointL, drivingDedendum)
        dToL = lines.addByTwoPoints(pointD.geometry, pointL.geometry)
        dToL.isConstruction = True
        constraints.addCoincident(dToL.startSketchPoint, pointD)
        constraints.addCoincident(dToL.endSketchPoint, pointL)

        drivingCenterRefLine = dToL
        if self._toothSpacing_cm > 0:
            lGeom = pointL.geometry
            awayFromD = self._normalize(self._vecFromTo(pointD.geometry, lGeom))
            lpSeed = adsk.core.Point3D.create(
                lGeom.x + awayFromD.x * self._toothSpacing_cm,
                lGeom.y + awayFromD.y * self._toothSpacing_cm, 0)
            lToLp = lines.addByTwoPoints(lGeom, lpSeed)
            lToLp.isConstruction = True
            constraints.addCoincident(lToLp.startSketchPoint, pointL)
            pointLp = lToLp.endSketchPoint
            constraints.addCoincident(pointLp, drivingDedendum)
            dimensions.addDistanceDimension(
                lToLp.startSketchPoint, lToLp.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (lGeom.x + lpSeed.x) / 2,
                    (lGeom.y + lpSeed.y) / 2, 0)).value = self._toothSpacing_cm
            dToLp = lines.addByTwoPoints(pointD.geometry, pointLp.geometry)
            dToLp.isConstruction = True
            constraints.addCoincident(dToLp.startSketchPoint, pointD)
            constraints.addCoincident(dToLp.endSketchPoint, pointLp)
            drivingCenterRefLine = dToLp
        else:
            pointLp = pointL

        # ----- O->P (driving toe) -----
        oSeed = adsk.core.Point3D.create(
            (apex2dG.x + gD.x) / 2.0, (apex2dG.y + gD.y) / 2.0, 0)
        dirDJ = self._normalize(self._vecFromTo(gD, gJ))
        distOtoB = math.hypot(oSeed.x - gB.x, oSeed.y - gB.y)
        pSeed = adsk.core.Point3D.create(
            oSeed.x + dirDJ.x * distOtoB, oSeed.y + dirDJ.y * distOtoB, 0)
        lineOP = lines.addByTwoPoints(oSeed, pSeed)
        lineOP.isConstruction = True
        pointO = lineOP.startSketchPoint
        pointP = lineOP.endSketchPoint
        constraints.addCoincident(pointO, drivingRootAxis)
        constraints.addCoincident(pointP, bToApex2)
        constraints.addParallel(lineOP, lineDJ)
        opMid = adsk.core.Point3D.create(
            (oSeed.x + pSeed.x) / 2, (oSeed.y + pSeed.y) / 2, 0)
        dimensions.addOffsetDimension(
            lineDJ, lineOP, opMid).parameter.value = faceWidth_cm
        oToD = lines.addByTwoPoints(pointO.geometry, pointD.geometry)
        oToD.isConstruction = True
        constraints.addCoincident(oToD.startSketchPoint, pointO)
        constraints.addCoincident(oToD.endSketchPoint, pointD)
        pToB = lines.addByTwoPoints(pointP.geometry, pointB.geometry)
        pToB.isConstruction = True
        constraints.addCoincident(pToB.startSketchPoint, pointP)
        constraints.addCoincident(pToB.endSketchPoint, pointB)
        # B->I line.
        bToI = lines.addByTwoPoints(pointB.geometry, pointI.geometry)
        bToI.isConstruction = False
        constraints.addCoincident(bToI.startSketchPoint, pointB)
        constraints.addCoincident(bToI.endSketchPoint, pointI)

        # Gate the §2 sketch.
        self._assertFullyConstrained(sketch)

        # ----- bundle the anchors needed downstream -----
        anchors = {
            'apexPoint': apexPoint,
            'apex2Point': apex2Point,
            'A': pointA, 'B': pointB, 'C': pointC, 'D': pointD,
            'E': pointE, 'F': pointF, 'G': pointG, 'H': pointH,
            'I': pointI, 'J': pointJ,
            'K': pointK, 'L': pointL, 'Kp': pointKp, 'Lp': pointLp,
            'M': pointM, 'N': pointN, 'O': pointO, 'P': pointP,
            'pinionRootAxis': pinionRootAxis,
            'drivingRootAxis': drivingRootAxis,
            'pinionCenterRefLine': pinionCenterRefLine,
            'drivingCenterRefLine': drivingCenterRefLine,
            'gearProfilesPlane': gearProfilesPlane,
            'gearProfilesSketch': sketch,
        }

        # ----- §3 + bodies, pinion first then driving -----
        # Pinion.
        pinionTooth = self._buildVirtualSpurProfile(
            designComponent, anchors, 'Pinion',
            self._pinionTeeth, self._pinionPitchDiameter_cm,
            self._gamma_p, pinionCenterRefLine, pointKp)
        self._createGearBody(
            designComponent, bevelComponent, anchors, 'Pinion',
            self._pinionTeeth, self._pinionBoreResolved_cm,
            self._pinionPitchDiameter_cm, pinionTooth,
            ['A', 'G', 'H', 'C', 'M', 'N'],
            ('M', 'N'), ('C', 'H'), 'M', 'C',
            self._gamma_p)

        # Driving.
        drivingTooth = self._buildVirtualSpurProfile(
            designComponent, anchors, 'Driving',
            self._drivingTeeth, self._drivingPitchDiameter_cm,
            self._gamma_g, drivingCenterRefLine, pointLp)
        self._createGearBody(
            designComponent, bevelComponent, anchors, 'Driving',
            self._drivingTeeth, self._drivingBoreResolved_cm,
            self._drivingPitchDiameter_cm, drivingTooth,
            ['B', 'I', 'J', 'D', 'O', 'P'],
            ('O', 'P'), ('D', 'J'), 'O', 'D',
            self._gamma_g)

    # -----------------------------------------------------------------------
    # 3 : virtual spur tooth profile (once per gear)
    # -----------------------------------------------------------------------
    def _buildVirtualSpurProfile(self, designComponent, anchors, gearLabel,
                                 teeth, pitchDiameter_cm, gamma,
                                 centerRefLine, centerPoint):
        # 1. virtual (Tredgold) tooth number.
        virtualPitchRadius_cm = (pitchDiameter_cm / 2.0) / math.cos(gamma)
        # virtualPitchRadius is a length (cm); Module is raw mm.  Compare in mm.
        virtualTeeth = int(math.floor(
            2.0 * to_mm(virtualPitchRadius_cm) / self._module_mm))

        # 2. plane including the tooth-center reference line, perpendicular to
        #    the Gear Profiles sketch plane.
        planeInput = designComponent.constructionPlanes.createInput()
        planeInput.setByAngle(
            centerRefLine,
            adsk.core.ValueInput.createByString('90 deg'),
            anchors['gearProfilesPlane'])
        toothPlane = designComponent.constructionPlanes.add(planeInput)
        toothPlane.name = f'{gearLabel} Plane'

        # 3. spur gear tooth profile on the new plane, centered at the tooth
        #    center, drawn already rotated 180 deg.
        toothSketch = designComponent.sketches.add(toothPlane)
        toothSketch.name = f'{gearLabel} Tooth'
        toothSketch.isVisible = True

        proxy = _VirtualSpurProxy(module_mm=self._module_mm, virtualTeeth=virtualTeeth)
        drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
        drawer.draw(centerPoint, angle=math.radians(180))
        embedded = proxy._lastToothEmbedded

        if not toothSketch.isFullyConstrained:
            futil.log(
                f'{gearLabel} Tooth sketch is not fully constrained '
                f'(embedded={embedded}) -- expected for low tooth counts')

        # 4. construction axis through the tooth center, normal to tooth plane.
        helperPlaneInput = designComponent.constructionPlanes.createInput()
        helperPlaneInput.setByDistanceOnPath(
            centerRefLine, adsk.core.ValueInput.createByReal(1.0))
        helperPlane = designComponent.constructionPlanes.add(helperPlaneInput)
        helperPlane.name = f'{gearLabel} Tooth Normal Helper'

        axisInput = designComponent.constructionAxes.createInput()
        axisInput.setByTwoPlanes(anchors['gearProfilesPlane'], helperPlane)
        toothAxis = designComponent.constructionAxes.add(axisInput)
        toothAxis.name = f'{gearLabel} Tooth Axis'

        return {
            'toothSketch': toothSketch,
            'toothPlane': toothPlane,
            'toothAxis': toothAxis,
            'embedded': embedded,
            'virtualTeeth': virtualTeeth,
        }

    def _findSpurToothProfile(self, toothSketch, embedded):
        wantLines = 0 if embedded else 2
        for prof in toothSketch.profiles:
            for loop in prof.profileLoops:
                nLines = 0
                nArcs = 0
                nNurbs = 0
                other = 0
                for curve in loop.profileCurves:
                    ct = curve.geometry.curveType
                    if ct == adsk.core.Curve3DTypes.Line3DCurveType:
                        nLines += 1
                    elif ct == adsk.core.Curve3DTypes.Arc3DCurveType:
                        nArcs += 1
                    elif ct == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nNurbs += 1
                    else:
                        other += 1
                if (nNurbs == 2 and nArcs == 2 and nLines == wantLines
                        and other == 0):
                    return prof
        raise Exception(
            'Could not find spur tooth profile loop '
            f'(2 NURBS + 2 arcs + {wantLines} lines)')

    # -----------------------------------------------------------------------
    # body creation (once per gear)
    # -----------------------------------------------------------------------
    def _createGearBody(self, designComponent, bevelComponent, anchors,
                        gearLabel, teeth, boreDiameter_cm, pitchDiameter_cm,
                        toothInfo, hexKeys, toeKeys, heelKeys,
                        toeConeKey, heelConeKey, gamma):
        sketch = designComponent.sketches.add(anchors['gearProfilesPlane'])
        sketch.name = f'{gearLabel} Profile'
        sketch.isVisible = True
        constraints = sketch.geometricConstraints
        lines = sketch.sketchCurves.sketchLines

        # Recreate the six hexagon vertices as new fixed points (recreate-share-fix).
        srcPoints = [anchors[k] for k in hexKeys]
        verts = []
        for src in srcPoints:
            world = self._pointWorldGeometry(src)
            local = sketch.modelToSketchSpace(world)
            verts.append(sketch.sketchPoints.add(local))

        hexLines = []
        n = len(verts)
        for i in range(n):
            ln = lines.addByTwoPoints(verts[i], verts[(i + 1) % n])
            hexLines.append(ln)

        # Fix the lines' endpoints AFTER the lines exist.
        for ln in hexLines:
            ln.startSketchPoint.isFixed = True
            ln.endSketchPoint.isFixed = True

        self._assertFullyConstrained(sketch)

        shaftAxisEdge = hexLines[0]  # first edge = shaft axis

        # ----- revolve -----
        profile = sketch.profiles.item(0)
        revolves = designComponent.features.revolveFeatures
        revInput = revolves.createInput(
            profile, shaftAxisEdge,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        revInput.setAngleExtent(
            False, adsk.core.ValueInput.createByString('360 deg'))
        revolve = revolves.add(revInput)
        gearBody = revolve.bodies.item(0)
        gearBody.name = f'{gearLabel} Gear Body'

        # ----- tooth loft (apex sketch point -> tooth profile) -----
        toothProfile = self._findSpurToothProfile(
            toothInfo['toothSketch'], toothInfo['embedded'])
        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput.loftSections.add(anchors['apexPoint'])
        loftInput.loftSections.add(toothProfile)
        loft = lofts.add(loftInput)
        toothBody = loft.bodies.item(0)
        toothBody.name = f'{gearLabel} Tooth'

        # ----- toe/heel world points for the hook -----
        apexWorld = self._pointWorldGeometry(anchors['apexPoint'])
        toeA = self._pointWorldGeometry(anchors[toeKeys[0]])
        toeB = self._pointWorldGeometry(anchors[toeKeys[1]])
        heelA = self._pointWorldGeometry(anchors[heelKeys[0]])
        heelB = self._pointWorldGeometry(anchors[heelKeys[1]])
        toeMid = adsk.core.Point3D.create(
            (toeA.x + toeB.x) / 2, (toeA.y + toeB.y) / 2, (toeA.z + toeB.z) / 2)
        heelMid = adsk.core.Point3D.create(
            (heelA.x + heelB.x) / 2, (heelA.y + heelB.y) / 2,
            (heelA.z + heelB.z) / 2)
        toeConeWorld = self._pointWorldGeometry(anchors[toeConeKey])
        heelConeWorld = self._pointWorldGeometry(anchors[heelConeKey])

        # ----- tooth-body transform (straight cuts or spiral build) -----
        toothBody = self._transformToothBody(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            anchors['apexPoint'], toeMid, heelMid, toeConeWorld, heelConeWorld,
            toothInfo['toothPlane'], gearLabel, teeth)

        # ----- pattern -----
        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(toothBody)
        patterns = designComponent.features.circularPatternFeatures
        patternInput = patterns.createInput(inputBodies, shaftAxisEdge)
        patternInput.quantity = adsk.core.ValueInput.createByReal(teeth)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patterns.add(patternInput)

        # ----- combine -----
        combineTools = adsk.core.ObjectCollection.create()
        for body in pattern.bodies:
            combineTools.add(body)
        combines = designComponent.features.combineFeatures
        combineInput = combines.createInput(gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combines.add(combineInput)

        # ----- bore -----
        if self._boreEnable:
            self._cutBore(designComponent, gearBody, shaftAxisEdge,
                          boreDiameter_cm, gearLabel)

        # ----- meshing rotation -----
        if gearLabel == 'Driving':
            meshAngle = math.radians(180.0 / self._drivingTeeth)
        else:
            meshAngle = self._pinionMeshPhase()
        if abs(meshAngle) > 1e-12:
            self._rotateBodyAboutEdge(
                designComponent, gearBody, shaftAxisEdge, meshAngle)

        # ----- move the finished body into this gear's own component -----
        gearOcc = bevelComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create())
        gearOcc.component.name = f'{gearLabel} Gear'
        gearBody.moveToComponent(gearOcc)

    def _pinionMeshPhase(self):
        # tooth-fractions of extra mesh rotation about the pinion shaft axis.
        if _PINION_MESH_PHASE_TEETH == 0.0:
            return 0.0
        return _PINION_MESH_PHASE_TEETH * (2.0 * math.pi / self._pinionTeeth)

    def _rotateBodyAboutEdge(self, designComponent, body, axisEdge, angle):
        startW = axisEdge.startSketchPoint.worldGeometry
        endW = axisEdge.endSketchPoint.worldGeometry
        axisVec = adsk.core.Vector3D.create(
            endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
        axisVec.normalize()
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(angle, axisVec, startW)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)
        moves = designComponent.features.moveFeatures
        moveInput = moves.createInput2(bodies)
        moveInput.defineAsFreeMove(matrix)
        moves.add(moveInput)

    def _cutBore(self, designComponent, gearBody, shaftAxisEdge,
                 boreDiameter_cm, gearLabel):
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

        radius = boreDiameter_cm / 2.0
        circle = circles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), radius)
        circle.centerSketchPoint.isFixed = True
        dimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(radius, radius, 0)).value = \
            boreDiameter_cm

        self._assertFullyConstrained(boreSketch)

        boreProfile = boreSketch.profiles.item(0)
        extrudes = designComponent.features.extrudeFeatures
        extInput = extrudes.createInput(
            boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        big = self._coneDistance_cm * 4.0
        extInput.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(big), False)
        extInput.participantBodies = [gearBody]
        extrudes.add(extInput)

    # -----------------------------------------------------------------------
    # tooth-body hook : straight (psi=0) or spiral (psi>0)
    # -----------------------------------------------------------------------
    def _transformToothBody(self, designComponent, toothBody, gearBody,
                            shaftAxisEdge, apexWorld, apexSketchPoint,
                            toeMid, heelMid, toeConeWorld, heelConeWorld,
                            parentToothPlane, gearLabel, teethNumber):
        if self._spiralAngle_rad <= 0:
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid,
                apexWorld, gearLabel)
        return self._buildSpiralTooth(
            designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
            toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane,
            gearLabel, teethNumber)

    # ----- straight bevel two-cone trim -----
    def _cutConicalEnds(self, designComponent, toothBody, gearBody,
                        toeMid, heelMid, apexWorld, gearLabel):
        keeper = self._applyConicalCut(
            designComponent, toothBody, gearBody, toeMid, apexWorld,
            gearLabel, 'toe')
        try:
            keeper = self._applyConicalCut(
                designComponent, keeper, gearBody, heelMid, apexWorld,
                gearLabel, 'heel')
        except RuntimeError as e:
            msg = str(e)
            if ('SPLIT_TARGET_TOOL_NOT_INTERSECT' in msg) or ('交差' in msg):
                futil.log(
                    f'{gearLabel}: heel cut tool did not intersect, kept intact',
                    force_console=True)
            else:
                raise
        return keeper

    def _applyConicalCut(self, designComponent, targetBody, gearBody,
                         edgeMidWorld, apexWorld, gearLabel, cutLabel):
        coneFaces = []
        for face in gearBody.faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
                coneFaces.append(face)
        # order best-first by surface distance to the edge midpoint.
        ranked = sorted(
            coneFaces,
            key=lambda f: self._surfaceDistance(f.geometry, edgeMidWorld))

        history = []
        for face in ranked:
            d = self._surfaceDistance(face.geometry, edgeMidWorld)
            try:
                splits = designComponent.features.splitBodyFeatures
                splitInput = splits.createInput(targetBody, face, True)
                split = splits.add(splitInput)
                pieces = [b for b in split.bodies]
                if len(pieces) > 1:
                    history.append((d, len(pieces), 'split'))
                    return self._selectKeeper(
                        designComponent, pieces, apexWorld, gearLabel, cutLabel)
                else:
                    history.append((d, len(pieces), 'no-split'))
            except RuntimeError as e:
                history.append((d, None, f'error:{e}'))
                continue
        raise Exception(
            f'{gearLabel}: {cutLabel} cut found no cone face that splits the '
            f'tooth body. cone-face history (dist, pieces, status)={history}')

    def _selectKeeper(self, designComponent, pieces, apexWorld, gearLabel,
                      cutLabel):
        nonApex = []
        toRemove = []
        for body in pieces:
            containment = body.pointContainment(apexWorld)
            if containment == adsk.fusion.PointContainment.PointInsidePointContainment:
                toRemove.append(body)
            elif containment == adsk.fusion.PointContainment.PointOnPointContainment:
                toRemove.append(body)
            else:
                nonApex.append(body)
        if len(nonApex) == 0:
            raise Exception(
                f'{gearLabel}: {cutLabel} cut left no non-apex piece '
                f'(pieces={len(pieces)})')
        # keep the largest non-apex piece.
        nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
        keeper = nonApex[0]
        removes = designComponent.features.removeFeatures
        for body in toRemove + nonApex[1:]:
            removes.add(body)
        return keeper

    def _findConeFaceForCutLine(self, gearBody, p0World, p1World):
        for face in gearBody.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            d0 = self._surfaceDistance(face.geometry, p0World)
            d1 = self._surfaceDistance(face.geometry, p1World)
            if d0 < 1e-2 and d1 < 1e-2:
                return face
        return None

    def _surfaceDistance(self, surface, worldPoint):
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return float('inf')
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return float('inf')
        return projected.distanceTo(worldPoint)

    # -----------------------------------------------------------------------
    # 3a : spiral tooth body (psi > 0)
    # -----------------------------------------------------------------------
    def _buildSpiralTooth(self, designComponent, toothBody, gearBody,
                          shaftAxisEdge, apexWorld, toeMid, heelMid,
                          toeConeWorld, heelConeWorld, parentToothPlane,
                          gearLabel, teethNumber):
        # ----- A. Gate & frame -----
        apex = apexWorld
        startW = shaftAxisEdge.startSketchPoint.worldGeometry
        endW = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = self._normalize(self._vecFromTo(startW, endW))

        # swap guard: heel must be the outer end.
        if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
            toeMid, heelMid = heelMid, toeMid
            toeConeWorld, heelConeWorld = heelConeWorld, toeConeWorld

        coneVec = self._normalize(self._vecFromTo(apex, heelConeWorld))
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

        # ----- B. cutter-arc geometry -----
        psi = self._spiralAngle_rad
        if self._cutterRadius_cm > 0:
            r_c = self._cutterRadius_cm
        else:
            r_c = R_mean
        handSign = 1.0 if self._hand == _HAND_RIGHT else -1.0
        if gearLabel == 'Pinion':
            handSign = -handSign
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = self._circleIntersectNearest(
            R_lo, Cx, Cy, r_c, (R_mean, 0.0))
        heel2d = self._circleIntersectNearest(
            R_hi, Cx, Cy, r_c, (R_mean, 0.0))

        # ----- C. 2-D trace sketch -----
        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        # cone-element line on the axial / Gear Profiles plane.
        axialPlane = self._gearProfilesPlane
        coneSketch = designComponent.sketches.add(axialPlane)
        coneSketch.name = f'{gearLabel} Cone Element'
        coneSketch.isVisible = True
        coneLines = coneSketch.sketchCurves.sketchLines
        apexLocal = coneSketch.modelToSketchSpace(apex)
        heelElemWorld = tanW(R_heel, 0.0)
        heelElemLocal = coneSketch.modelToSketchSpace(heelElemWorld)
        coneElementLine = coneLines.addByTwoPoints(apexLocal, heelElemLocal)
        coneElementLine.isConstruction = True

        # tangent plane = axial plane rotated 90 deg about the cone-element line.
        tangentPlane = self._planeByAngle(
            designComponent, coneElementLine, axialPlane, 90)
        tangentPlane.name = f'{gearLabel} Trace Plane'

        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        traceSketch.isVisible = True
        traceCircles = traceSketch.sketchCurves.sketchCircles
        traceArcs = traceSketch.sketchCurves.sketchArcs
        traceDims = traceSketch.sketchDimensions
        traceConstraints = traceSketch.geometricConstraints

        centerWorld = tanW(Cx, Cy)
        centerLocal = traceSketch.modelToSketchSpace(centerWorld)
        cutterCircle = traceCircles.addByCenterRadius(centerLocal, r_c)
        cutterCircle.isConstruction = True
        cutterCircle.centerSketchPoint.isFixed = True
        diaTextWorld = tanW(Cx + r_c, Cy)
        diaTextLocal = traceSketch.modelToSketchSpace(diaTextWorld)
        traceDims.addDiameterDimension(cutterCircle, diaTextLocal).value = 2.0 * r_c

        toeArcWorld = tanW(toe2d[0], toe2d[1])
        meanArcWorld = tanW(R_mean, 0.0)
        heelArcWorld = tanW(heel2d[0], heel2d[1])
        arc = traceArcs.addByThreePoints(
            traceSketch.modelToSketchSpace(toeArcWorld),
            traceSketch.modelToSketchSpace(meanArcWorld),
            traceSketch.modelToSketchSpace(heelArcWorld))
        traceConstraints.addCoincident(
            arc.centerSketchPoint, cutterCircle.centerSketchPoint)
        radTextLocal = traceSketch.modelToSketchSpace(meanArcWorld)
        traceDims.addRadialDimension(arc, radTextLocal).value = r_c
        # (deliberately left with free DOF -- exempt from full-constraint gate)

        # ----- E. slice the straight tooth -----
        segments = self._sliceToothBody(
            designComponent, toothBody, parentToothPlane, apex, coneVec,
            span, gearLabel)

        # ----- F. order & drop scrap -----
        def centroidDistAlong(body):
            com = body.physicalProperties.centerOfMass
            return distAlong(com)

        segments.sort(key=centroidDistAlong)
        scrap = segments[0]
        segments = segments[1:]
        designComponent.features.removeFeatures.add(scrap)
        if len(segments) == 0:
            raise Exception(
                f'{gearLabel}: spiral slice left no working segments after '
                f'dropping scrap (span={span})')

        # ----- G. twist -----
        gamma = self._gamma_p if gearLabel == 'Pinion' else self._gamma_g
        phi_crown = (math.atan2(heel2d[1], heel2d[0])
                     - math.atan2(toe2d[1], toe2d[0]))
        total = abs(phi_crown) / math.sin(gamma)

        def slabHeelFace(body):
            best = None
            bestD = None
            for face in body.faces:
                cd = distAlong(face.centroid)
                if bestD is None or cd > bestD:
                    bestD = cd
                    best = face
            return best, bestD

        def slabToeFace(body):
            best = None
            bestD = None
            for face in body.faces:
                cd = distAlong(face.centroid)
                if bestD is None or cd < bestD:
                    bestD = cd
                    best = face
            return best, bestD

        for seg in segments:
            (_, R_heelFace) = slabHeelFace(seg)
            ang = -handSign * total * (R_mean - R_heelFace) / span
            matrix = adsk.core.Matrix3D.create()
            matrix.setToRotation(ang, axisDir, apex)
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(seg)
            moves = designComponent.features.moveFeatures
            moveInput = moves.createInput2(bodies)
            moveInput.defineAsFreeMove(matrix)
            moves.add(moveInput)

        # ----- H. lengthwise crown -----
        # outermost (heel) segment held full.
        segHeelDist = [(seg, slabHeelFace(seg)[1]) for seg in segments]
        segHeelDist.sort(key=lambda t: t[1])
        outermost = segHeelDist[-1][0]

        self._designOccurrence.activate()
        try:
            for seg in segments:
                if seg is outermost:
                    continue
                (heelFace, R_heelFace) = slabHeelFace(seg)
                u = (R_heel - R_heelFace) / span
                factor = 1.0 - _CROWN_PER_RAD * (abs(total) / 2.0) * u
                if factor >= 1.0 or factor <= 0.0:
                    continue
                basePoint = self._crownBasePoint(
                    designComponent, heelFace, apex, axisDir, gearLabel)
                scales = designComponent.features.scaleFeatures
                inputColl = adsk.core.ObjectCollection.create()
                inputColl.add(seg)
                scaleInput = scales.createInput(
                    inputColl, basePoint,
                    adsk.core.ValueInput.createByReal(factor))
                scales.add(scaleInput)
        finally:
            self.design.activateRootComponent()

        # ----- I. loft -> curved tooth -----
        indexed = list(range(len(segments)))
        order = sorted(
            indexed, key=lambda i: slabHeelFace(segments[i])[1])

        lofts = designComponent.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        toeSeg = segments[order[0]]
        (toeFace, _) = slabToeFace(toeSeg)
        loftInput.loftSections.add(toeFace)
        for i in order:
            (heelFace, _) = slabHeelFace(segments[i])
            loftInput.loftSections.add(heelFace)
        loft = lofts.add(loftInput)
        curvedTooth = loft.bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'

        # remove the segment scaffolding.
        removes = designComponent.features.removeFeatures
        for seg in segments:
            removes.add(seg)

        # ----- J. flush trim -----
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid,
            apex, gearLabel)

    def _crownBasePoint(self, designComponent, heelFace, apex, axisDir, gearLabel):
        # find the two root corners (smallest perp distance to the shaft axis).
        def perpDist(p):
            dx = p.x - apex.x
            dy = p.y - apex.y
            dz = p.z - apex.z
            dot = dx * axisDir.x + dy * axisDir.y + dz * axisDir.z
            rx = dx - dot * axisDir.x
            ry = dy - dot * axisDir.y
            rz = dz - dot * axisDir.z
            return math.sqrt(rx * rx + ry * ry + rz * rz)

        verts = [vtx.geometry for vtx in heelFace.vertices]
        verts.sort(key=perpDist)
        root0 = verts[0]
        root1 = verts[1]
        mid = adsk.core.Point3D.create(
            (root0.x + root1.x) / 2,
            (root0.y + root1.y) / 2,
            (root0.z + root1.z) / 2)

        # add a sketch point on the heel face at that midpoint.
        faceSketch = designComponent.sketches.add(heelFace)
        faceSketch.name = f'{gearLabel} Crown Base'
        faceSketch.isVisible = False
        localMid = faceSketch.modelToSketchSpace(mid)
        return faceSketch.sketchPoints.add(localMid)

    def _sliceToothBody(self, designComponent, toothBody, parentToothPlane,
                        apex, coneVec, span, gearLabel):
        def doSlice(sign):
            pieces = [toothBody]
            planeOrigin = parentToothPlane.geometry.origin
            # first cut plane: parentToothPlane offset toward apex by span/6.
            step = span / 6.0
            planes = []
            for k in range(8):
                pin = designComponent.constructionPlanes.createInput()
                pin.setByOffset(
                    parentToothPlane,
                    adsk.core.ValueInput.createByReal(sign * (k + 1) * step))
                pl = designComponent.constructionPlanes.add(pin)
                pl.isLightBulbOn = False
                planes.append(pl)
            for pl in planes:
                newPieces = []
                for piece in pieces:
                    try:
                        splits = designComponent.features.splitBodyFeatures
                        sin = splits.createInput(piece, pl, True)
                        split = splits.add(sin)
                        for b in split.bodies:
                            newPieces.append(b)
                    except RuntimeError:
                        newPieces.append(piece)
                pieces = newPieces
            return pieces

        # choose sign so it moves toward the apex.
        planeOrigin = parentToothPlane.geometry.origin
        normal = parentToothPlane.geometry.normal
        toApex = adsk.core.Vector3D.create(
            apex.x - planeOrigin.x, apex.y - planeOrigin.y,
            apex.z - planeOrigin.z)
        sign = 1.0 if (normal.dotProduct(toApex) >= 0) else -1.0

        pieces = doSlice(sign)
        if len(pieces) <= 1:
            pieces = doSlice(-sign)
            sign = -sign
        if len(pieces) <= 1:
            raise Exception(
                f'{gearLabel}: spiral slice failed to split the tooth body '
                f'(final pieces={len(pieces)}, span={span}, sign={sign})')
        return pieces

    # ----- spiral-only geometry helpers -----
    def _planeByAngle(self, comp, line, refPlane, degrees):
        planeInput = comp.constructionPlanes.createInput()
        planeInput.setByAngle(
            line,
            adsk.core.ValueInput.createByString(f'{degrees} deg'),
            refPlane)
        return comp.constructionPlanes.add(planeInput)

    def _circleIntersectNearest(self, R, Cx, Cy, r_c, refPoint):
        """Intersect the apex circle (radius R, centre origin) with the cutter
        circle (centre (Cx,Cy), radius r_c); return the solution nearest
        refPoint as a (x, y) tuple."""
        d = math.hypot(Cx, Cy)
        if d < 1e-12:
            # concentric; fall back to a point on the apex circle toward refPoint.
            rr = math.hypot(refPoint[0], refPoint[1])
            if rr < 1e-12:
                return (R, 0.0)
            return (R * refPoint[0] / rr, R * refPoint[1] / rr)
        # a = distance from origin to the radical line along the centre line.
        a = (R * R - r_c * r_c + d * d) / (2.0 * d)
        h2 = R * R - a * a
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)
        # base point along the centre line.
        ex = Cx / d
        ey = Cy / d
        bx = a * ex
        by = a * ey
        # perpendicular offsets.
        sol1 = (bx + h * (-ey), by + h * ex)
        sol2 = (bx - h * (-ey), by - h * ex)
        d1 = math.hypot(sol1[0] - refPoint[0], sol1[1] - refPoint[1])
        d2 = math.hypot(sol2[0] - refPoint[0], sol2[1] - refPoint[1])
        return sol1 if d1 <= d2 else sol2

    def _bottomEdgeMid(self, edgeA, edgeB):
        return adsk.core.Point3D.create(
            (edgeA.x + edgeB.x) / 2,
            (edgeA.y + edgeB.y) / 2,
            (edgeA.z + edgeB.z) / 2)

    def _perpToAxis(self, point, apex, axisDir):
        dx = point.x - apex.x
        dy = point.y - apex.y
        dz = point.z - apex.z
        dot = dx * axisDir.x + dy * axisDir.y + dz * axisDir.z
        rx = dx - dot * axisDir.x
        ry = dy - dot * axisDir.y
        rz = dz - dot * axisDir.z
        return math.sqrt(rx * rx + ry * ry + rz * rz)

    def _distDim(self, sketch, p0, p1, textPoint, value):
        dim = sketch.sketchDimensions.addDistanceDimension(
            p0, p1,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            textPoint)
        dim.value = value
        return dim

    # -----------------------------------------------------------------------
    # cleanup
    # -----------------------------------------------------------------------
    def _hideConstructionGeometry(self, bevelComponent):
        seen = set()

        def walk(component):
            for sketch in component.sketches:
                tok = sketch.entityToken
                if tok not in seen:
                    seen.add(tok)
                    sketch.isLightBulbOn = False
            for plane in component.constructionPlanes:
                tok = plane.entityToken
                if tok not in seen:
                    seen.add(tok)
                    plane.isLightBulbOn = False
            for axis in component.constructionAxes:
                tok = axis.entityToken
                if tok not in seen:
                    seen.add(tok)
                    axis.isLightBulbOn = False
            for occ in component.occurrences:
                walk(occ.component)

        walk(bevelComponent)
