import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts


# --- Dialog input ids (verbatim, part of the reproduced surface) ---
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'
INPUT_ID_MODULE = 'module'
INPUT_ID_TOOTH_NUMBER = 'toothNumber'
INPUT_ID_PRESSURE_ANGLE = 'pressureAngle'
INPUT_ID_BORE_DIAMETER = 'boreDiameter'
INPUT_ID_THICKNESS = 'thickness'
INPUT_ID_CHAMFER_TOOTH = 'chamferTooth'
INPUT_ID_SKETCH_ONLY = 'sketchOnly'

# --- User-parameter names (verbatim) ---
PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
PARAM_PRESSURE_ANGLE = 'PressureAngle'
PARAM_BORE_DIAMETER = 'BoreDiameter'
PARAM_THICKNESS = 'Thickness'
PARAM_CHAMFER_TOOTH = 'ChamferTooth'
PARAM_SKETCH_ONLY = 'SketchOnly'
PARAM_PITCH_DIAMETER = 'PitchCircleDiameter'
PARAM_PITCH_RADIUS = 'PitchCircleRadius'
PARAM_BASE_DIAMETER = 'BaseCircleDiameter'
PARAM_BASE_RADIUS = 'BaseCircleRadius'
PARAM_ROOT_DIAMETER = 'RootCircleDiameter'
PARAM_ROOT_RADIUS = 'RootCircleRadius'
PARAM_TIP_DIAMETER = 'TipCircleDiameter'
PARAM_TIP_RADIUS = 'TipCircleRadius'
PARAM_INVOLUTE_STEPS = 'InvoluteSteps'
PARAM_TOOTH_SPACE_ANGLE = 'ToothSpaceAngleAtRoot'
PARAM_TOOTH_SPACE_ARC = 'ToothSpaceArcAtRoot'
PARAM_FILLET_CLEARANCE = 'FilletClearance'
PARAM_FILLET_RADIUS = 'FilletRadius'


class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        # 1. Target Plane (first selection => owns initial focus, [PB-AUTOFOCUS-FIRST])
        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane', 'Select the plane to build the gear on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1, 1)

        # 2. Anchor Point
        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point', 'Select the point to center the gear on')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1, 1)

        # 3. Module (unitless)
        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))

        # 4. Tooth Number (unitless)
        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))

        # 5. Pressure Angle (angle, default 20 deg)
        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(20)))

        # 6. Bore Diameter (string value so it accepts expressions; default '0 mm')
        inputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

        # 7. Thickness (default 10 mm)
        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))

        # 8. Apply chamfer to teeth (default 0 mm)
        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(0))

        # 9. Generate sketches, but do not build body (boolean, default false)
        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY, 'Generate sketches, but do not build body', True, '', False)

        # 10. Parent Component (last; defaults to root)
        parentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component', 'Select the parent component')
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        parentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        parentInput.setSelectionLimits(1, 1)
        parentInput.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    def __init__(self):
        self.plane = adsk.fusion.ConstructionPlane.cast(None)
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.extrusionExtent = adsk.fusion.BRepFace.cast(None)
        self.toothProfileIsEmbedded = False


class SpurGearInvoluteToothDesignGenerator:
    def __init__(self, sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        # Retained incidental field; the live rotation always comes from draw()'s
        # runtime argument, NOT this value (do not use inside drawTooth).
        self.toothAngle = angle
        # Movable local origin: a fresh (0,0,0) SketchPoint (NOT sketch.originPoint).
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        # References filled by drawCircles / drawTooth.
        self.rootCircle = None
        self.tipCircle = None
        self.baseCircle = None
        self.pitchCircle = None
        self.spineAngularDimension = None

    # --- parameter accessors (reproduced surface) ---
    def getParameter(self, name):
        return self.parent.getParameter(name)

    def getParameterValue(self, name):
        return self.parent.getParameter(name).value

    # --- exact involute math (pinned; do not infer) ---
    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        if intersectionRadius < baseRadius:
            return None
        alpha = math.acos(baseRadius / intersectionRadius)
        t = math.tan(alpha)  # curve parameter is tan(alpha), NOT inv(alpha)
        x = baseRadius * (math.cos(t) + t * math.sin(t))
        y = baseRadius * (math.sin(t) - t * math.cos(t))
        return adsk.core.Point3D.create(x, y, 0)

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        self.drawTooth(angle)
        # Step 5 anchoring: chain to the user's anchor via projection, then
        # coincident the projected point with the local origin.
        projected = self.sketch.project(anchorPoint)
        self.sketch.geometricConstraints.addCoincident(projected.item(0), self.anchorPoint)
        # Confirm the requested rotation as the very last action ([SPUR-F-ROTATE-CONFIRM]).
        if angle != 0 and self.spineAngularDimension is not None:
            self.spineAngularDimension.parameter.value = angle

    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        origin = self.anchorPoint

        rootR = self.getParameterValue(PARAM_ROOT_RADIUS)
        tipR = self.getParameterValue(PARAM_TIP_RADIUS)
        baseR = self.getParameterValue(PARAM_BASE_RADIUS)
        pitchR = self.getParameterValue(PARAM_PITCH_RADIUS)
        size = tipR - rootR

        # (name, radius, isConstruction) in the exact spec order.
        specs = [
            ('Root Circle', rootR, False),
            ('Tip Circle', tipR, True),
            ('Base Circle', baseR, True),
            ('Pitch Circle', pitchR, True),
        ]
        drawn = []
        for name, radius, isConstruction in specs:
            # Pass the SketchPoint directly so all four share the one center.
            circle = circles.addByCenterRadius(origin, radius)
            circle.isConstruction = isConstruction
            # Driving diameter dimension (text point off-centre, [PB-RADIAL-DIM]).
            textPoint = adsk.core.Point3D.create(radius, 0, 0)
            sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
            # Along-path label ([PB-SKETCH-TEXT]).
            label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)
            textInput = sketch.sketchTexts.createInput2(label, size)
            textInput.setAsAlongPath(
                circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            sketch.sketchTexts.add(textInput)
            drawn.append(circle)

        self.rootCircle, self.tipCircle, self.baseCircle, self.pitchCircle = drawn

    def drawTooth(self, angle=0):
        sketch = self.sketch
        origin = self.anchorPoint

        toothNumber = int(self.getParameterValue(PARAM_TOOTH_NUMBER))
        baseR = self.getParameterValue(PARAM_BASE_RADIUS)
        tipR = self.getParameterValue(PARAM_TIP_RADIUS)
        rootR = self.getParameterValue(PARAM_ROOT_RADIUS)
        pitchR = self.getParameterValue(PARAM_PITCH_RADIUS)
        steps = int(self.getParameterValue(PARAM_INVOLUTE_STEPS))

        # 1. Sample the involute flank, base -> tip in equal radial steps. First
        #    sample radius is exactly baseR; drop any None (below the base circle).
        samples = []
        for i in range(steps):
            r = baseR + (tipR - baseR) * i / (steps - 1)
            p = self.calculateInvolutePoint(baseR, r)
            if p is None:
                continue
            samples.append(p)

        # 2. Mirror across +X (negate y) so the spiral matches a left flank.
        mirrored = [(p.x, -p.y) for p in samples]

        # 3. Rotate so the pitch-circle crossing lands at +pi/(2N) (analytic).
        pitchPoint = self.calculateInvolutePoint(baseR, pitchR)
        rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-pitchPoint.y, pitchPoint.x)

        def rot(pt, a):
            ca, sa = math.cos(a), math.sin(a)
            return (pt[0] * ca - pt[1] * sa, pt[0] * sa + pt[1] * ca)

        # 4. left = rotate(mirrored, rotate_angle); right = mirror(left); then
        #    rotate BOTH by the requested angle (no-op for angle == 0).
        left0 = [rot(p, rotate_angle) for p in mirrored]
        right0 = [(p[0], -p[1]) for p in left0]
        leftPts = [rot(p, angle) for p in left0]
        rightPts = [rot(p, angle) for p in right0]

        # 5. Two flanks as fitted splines.
        leftColl = adsk.core.ObjectCollection.create()
        for p in leftPts:
            leftColl.add(adsk.core.Point3D.create(p[0], p[1], 0))
        leftSpline = sketch.sketchCurves.sketchFittedSplines.add(leftColl)

        rightColl = adsk.core.ObjectCollection.create()
        for p in rightPts:
            rightColl.add(adsk.core.Point3D.create(p[0], p[1], 0))
        rightSpline = sketch.sketchCurves.sketchFittedSplines.add(rightColl)

        # 6. Tooth-top arc ([SPUR-F-TOOTHTOP-ARC]).
        toothTop = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipR * math.cos(angle), tipR * math.sin(angle), 0))
        sketch.geometricConstraints.addCoincident(toothTop, self.tipCircle)
        topArc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            rightSpline.endSketchPoint, toothTop.geometry, leftSpline.endSketchPoint)
        sketch.sketchDimensions.addDiameterDimension(topArc, toothTop.geometry)

        # 7. Spine + horizontal reference + angular pin ([SPUR-F-SPINE]).
        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(origin, toothTop)
        spine.isConstruction = True
        if angle == 0:
            sketch.geometricConstraints.addHorizontal(spine)
        else:
            horizontal = sketch.sketchCurves.sketchLines.addByTwoPoints(
                origin, adsk.core.Point3D.create(tipR, 0, 0))
            horizontal.isConstruction = True
            sketch.geometricConstraints.addHorizontal(horizontal)
            sketch.geometricConstraints.addCoincident(horizontal.endSketchPoint, self.tipCircle)
            bisectorText = adsk.core.Point3D.create(
                tipR * math.cos(angle / 2), tipR * math.sin(angle / 2), 0)
            self.spineAngularDimension = sketch.sketchDimensions.addAngularDimension(
                spine, horizontal, bisectorText)

        # 8. Ribs ([SPUR-F-RIBS]) — exact order, midpoint chain from the origin.
        leftFit = leftSpline.fitPoints
        rightFit = rightSpline.fitPoints
        prevMid = origin
        for i in range(leftFit.count):
            lp = leftFit.item(i)
            rp = rightFit.item(i)
            # 1. rib shares the two fit points; construction.
            rib = sketch.sketchCurves.sketchLines.addByTwoPoints(lp, rp)
            rib.isConstruction = True
            # 2. aligned length dimension at its current measured value.
            lenText = adsk.core.Point3D.create(
                (lp.geometry.x + rp.geometry.x) / 2, (lp.geometry.y + rp.geometry.y) / 2, 0)
            sketch.sketchDimensions.addDistanceDimension(
                lp, rp, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lenText)
            # 3. fresh midpoint seeded at the foot of the left fit point on the spine.
            t = lp.geometry.x * math.cos(angle) + lp.geometry.y * math.sin(angle)
            midSeed = adsk.core.Point3D.create(t * math.cos(angle), t * math.sin(angle), 0)
            mid = sketch.sketchPoints.add(midSeed)
            # 4. pin onto the spine first.
            sketch.geometricConstraints.addCoincident(mid, spine)
            # 5. then make it the rib's midpoint.
            sketch.geometricConstraints.addMidPoint(mid, rib)
            # 6. then make the rib perpendicular to the spine.
            sketch.geometricConstraints.addPerpendicular(spine, rib)
            # chain distance from the previous midpoint (origin for the first rib).
            pg = prevMid.geometry
            chainText = adsk.core.Point3D.create(
                (pg.x + midSeed.x) / 2, (pg.y + midSeed.y) / 2, 0)
            sketch.sketchDimensions.addDistanceDimension(
                prevMid, mid, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                chainText)
            prevMid = mid

        # 9. Close the tooth at the root ([SPUR-F-FLANK-ROOT]); detect embedded case
        #    from where the flank start lands relative to the root circle.
        firstStart = leftFit.item(0)
        firstRadius = math.hypot(firstStart.geometry.x, firstStart.geometry.y)
        embedded = firstRadius < rootR
        self.parent._lastToothEmbedded = embedded
        if not embedded:
            self._drawFlankToRoot(leftFit.item(0))
            self._drawFlankToRoot(rightFit.item(0))

    def _drawFlankToRoot(self, flankStart):
        sketch = self.sketch
        origin = self.anchorPoint
        rootR = self.getParameterValue(PARAM_ROOT_RADIUS)
        g = flankStart.geometry
        n = math.hypot(g.x, g.y)
        rootEnd = adsk.core.Point3D.create(rootR * g.x / n, rootR * g.y / n, 0)
        # Share the flank start SketchPoint directly as the far endpoint.
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flankStart)
        # (a) root end on the root circle.
        sketch.geometricConstraints.addCoincident(line.startSketchPoint, self.rootCircle)
        # (b) local origin on the line (radial direction).
        sketch.geometricConstraints.addCoincident(origin, line)

    def drawBore(self, anchorPoint, diameter):
        sketch = self.sketch
        projected = sketch.project(anchorPoint)
        center = projected.item(0)
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(center, diameter / 2)
        cg = center.geometry
        textPoint = adsk.core.Point3D.create(cg.x + diameter / 2, cg.y, 0)
        sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
        return circle


class SpurGearGenerator(Generator):
    def __init__(self, design):
        super().__init__(design)
        # Pre-initialised state ([SPUR-F-FLANK-ROOT] embedded-flag mechanism).
        self._lastToothEmbedded = False
        self.toolsSketch = None
        self.boreSketch = None
        self.normalizedPlane = None

    def prefixBase(self):
        return 'SpurGear'

    def newContext(self):
        return SpurGearGenerationContext()

    # --- overridable hooks (subclasses vary these) ---
    def addExtraPrimaryParameters(self, inputs):
        pass

    def chamferWantEdges(self):
        return 6

    def filletHelixFactorExpression(self):
        return '1'

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(
            module.expression, toothNumber.expression, thickness.expression)

    def processInputs(self, inputs):
        # 1. Pull every selection out FIRST (before occurrence creation shifts the
        #    active-component context and can drop selections) ([PB-SELECTION-STASH]).
        parents = get_selection(inputs, INPUT_ID_PARENT)
        if len(parents) != 1:
            raise Exception('Spur gear: exactly one Parent Component is required')
        parentEntity = parents[0]
        if parentEntity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = parentEntity.component
        else:
            self.parentComponent = parentEntity

        planes = get_selection(inputs, INPUT_ID_PLANE)
        if len(planes) != 1:
            raise Exception('Spur gear: exactly one Target Plane is required')
        self.plane = planes[0]

        anchors = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchors) != 1:
            raise Exception('Spur gear: exactly one Anchor Point is required')
        self.anchorPoint = anchors[0]

        # 2/3. Register input-sourced numeric/bool parameters (this creates the
        #      occurrence via addParameter -> parameterName -> getOccurrence).
        self.addParameter(PARAM_MODULE, get_value(inputs, INPUT_ID_MODULE, ''), '',
                          'Module of the gear')
        self.addParameter(PARAM_TOOTH_NUMBER, get_value(inputs, INPUT_ID_TOOTH_NUMBER, ''), '',
                          'Number of teeth')
        self.addParameter(PARAM_PRESSURE_ANGLE, get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad'),
                          'rad', 'Pressure angle')
        self.addParameter(PARAM_BORE_DIAMETER, get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm'),
                          'mm', 'Bore diameter')
        self.addParameter(PARAM_THICKNESS, get_value(inputs, INPUT_ID_THICKNESS, 'mm'),
                          'mm', 'Thickness')
        self.addParameter(PARAM_CHAMFER_TOOTH, get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm'),
                          'mm', 'Tooth chamfer distance')
        # Boolean read with get_boolean, persisted as a 1/0 real parameter.
        sketchOnly = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(PARAM_SKETCH_ONLY,
                          adsk.core.ValueInput.createByReal(1 if sketchOnly else 0), '',
                          'Generate sketches only (1) or full body (0)')

        # 4. Subclass primary-parameter hook (before derived references them).
        self.addExtraPrimaryParameters(inputs)

        # 5/6. Derived parameters.
        self.registerDerivedParameters()

    def _addDerived(self, name, expression, units, comment):
        self.addParameter(name, adsk.core.ValueInput.createByString(expression), units, comment)

    def registerDerivedParameters(self):
        pn = self.parameterName

        self._addDerived(PARAM_PITCH_DIAMETER,
                         '{} * {}'.format(pn(PARAM_MODULE), pn(PARAM_TOOTH_NUMBER)),
                         'mm', 'Pitch circle diameter')
        self._addDerived(PARAM_PITCH_RADIUS,
                         '{} / 2'.format(pn(PARAM_PITCH_DIAMETER)),
                         'mm', 'Pitch circle radius')
        self._addDerived(PARAM_BASE_DIAMETER,
                         '{} * cos({})'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_PRESSURE_ANGLE)),
                         'mm', 'Base circle diameter')
        self._addDerived(PARAM_BASE_RADIUS,
                         '{} / 2'.format(pn(PARAM_BASE_DIAMETER)),
                         'mm', 'Base circle radius')
        self._addDerived(PARAM_ROOT_DIAMETER,
                         '{} - 2.5 * {}'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_MODULE)),
                         'mm', 'Root circle diameter')
        self._addDerived(PARAM_ROOT_RADIUS,
                         '{} / 2'.format(pn(PARAM_ROOT_DIAMETER)),
                         'mm', 'Root circle radius')
        self._addDerived(PARAM_TIP_DIAMETER,
                         '{} + 2 * {}'.format(pn(PARAM_PITCH_DIAMETER), pn(PARAM_MODULE)),
                         'mm', 'Tip circle diameter')
        self._addDerived(PARAM_TIP_RADIUS,
                         '{} / 2'.format(pn(PARAM_TIP_DIAMETER)),
                         'mm', 'Tip circle radius')
        self._addDerived(PARAM_INVOLUTE_STEPS, '15', '', 'Involute sample count')

        # Tooth Space Angle At Root: pre-evaluated in Python (Fusion's expression
        # engine refuses to mix tan()'s unitless output with the radian
        # PressureAngle in a subtraction). Registered UNITLESS (radians are
        # dimensionless) so the dependent length product reads as pure mm.
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngle = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)
        self.addParameter(PARAM_TOOTH_SPACE_ANGLE,
                          adsk.core.ValueInput.createByReal(toothSpaceAngle), '',
                          'Tooth space angle at the root (radians, stored unitless)')

        self._addDerived(PARAM_TOOTH_SPACE_ARC,
                         '{} * {}'.format(pn(PARAM_ROOT_RADIUS), pn(PARAM_TOOTH_SPACE_ANGLE)),
                         'mm', 'Tooth space arc at the root')
        self._addDerived(PARAM_FILLET_CLEARANCE, '0.9', '', 'Fillet clearance fraction')
        self._addDerived(PARAM_FILLET_RADIUS,
                         '({} / 2) * {} * {}'.format(
                             pn(PARAM_TOOTH_SPACE_ARC), pn(PARAM_FILLET_CLEARANCE),
                             self.filletHelixFactorExpression()),
                         'mm', 'Root fillet radius')

    def generate(self, inputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()
        futil.log(f'Generating {component.name}')

        # Normalize the target plane to a ConstructionPlane (step 1).
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.normalizedPlane = component.constructionPlanes.add(planeInput)
            self.plane = self.normalizedPlane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)
        self.cleanup(ctx)

    def prepareTools(self, ctx):
        component = self.getComponent()

        # Tools sketch owns the canonical anchor projection (step 2). Keep it
        # visible while later sketches project from it.
        toolsSketch = self.createSketchObject('Tools', ctx.plane)
        toolsSketch.isVisible = True
        self.toolsSketch = toolsSketch
        projected = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projected.item(0)

        # Extrusion End Plane at distance Thickness (to-entity for both extrudes).
        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(ctx.plane, adsk.core.ValueInput.createByReal(thickness))
        endPlane = component.constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    def buildMainGearBody(self, ctx):
        self.buildSketches(ctx)
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 6: show the Gear Profile sketch and stop.
            ctx.gearProfileSketch.isVisible = True
            return
        self.buildTooth(ctx)
        self.buildBody(ctx)
        self.patternTeeth(ctx)

    def buildSketches(self, ctx):
        gearProfileSketch = self.createSketchObject('Gear Profile', ctx.plane)
        gearProfileSketch.isVisible = True
        ctx.gearProfileSketch = gearProfileSketch

        generator = SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self)
        generator.draw(ctx.anchorPoint, angle=0)

        # Copy the embedded flag the tooth generator wrote onto self.
        ctx.toothProfileIsEmbedded = self._lastToothEmbedded

    def buildTooth(self, ctx):
        component = self.getComponent()
        sketch = ctx.gearProfileSketch

        profile = find_profile_by_curve_counts(
            sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude tooth'
        ctx.toothBody = extrude.bodies.item(0)

        # Chamfer is triggered from inside buildTooth (subclass boundary).
        self.chamferTooth(ctx)

    def chamferTooth(self, ctx):
        chamferValue = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferValue <= 0:
            return

        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        wantEdges = self.chamferWantEdges()
        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry

        frontFace = None
        for face in ctx.toothBody.faces:
            if face.edges.count == wantEdges and sketchPlane.isCoPlanarTo(face.geometry):
                frontFace = face
                break
        if frontFace is None:
            raise Exception('Spur gear: could not find the tooth front face to chamfer')

        edges = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            geom = edge.geometry
            if (geom.curveType == adsk.core.Curve3DTypes.Arc3DCurveType
                    and abs(geom.radius - rootRadius) < 0.001):
                # Skip the root arc — chamfering it would eat the neighbour tooth.
                continue
            edges.add(edge)

        chamferFeatures = component.features.chamferFeatures
        chamferInput = chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edges, adsk.core.ValueInput.createByReal(chamferValue), False)
        chamferFeatures.add(chamferInput)

    def buildBody(self, ctx):
        component = self.getComponent()
        sketch = ctx.gearProfileSketch

        profile = find_profile_by_curve_counts(sketch, arcs=2)

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrude = extrudes.add(extrudeInput)
        extrude.name = 'Extrude body'
        body = extrude.bodies.item(0)
        body.name = 'Gear Body'

        sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
        centerAxis = None
        extrusionExtent = None
        for face in body.faces:
            surfaceType = face.geometry.surfaceType
            if surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and centerAxis is None:
                axisInput = component.constructionAxes.createInput()
                axisInput.setByCircularFace(face)
                centerAxis = component.constructionAxes.add(axisInput)
                centerAxis.name = 'Gear Center'
                centerAxis.isLightBulbOn = False
            elif surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                if (sketchPlane.isParallelToPlane(face.geometry)
                        and not sketchPlane.isCoPlanarTo(face.geometry)):
                    extrusionExtent = face

        if centerAxis is None or extrusionExtent is None:
            raise Exception('Spur gear: failed to find the center axis or far end-cap face')

        ctx.centerAxis = centerAxis
        ctx.extrusionExtent = extrusionExtent
        ctx.gearBody = body

    def patternTeeth(self, ctx):
        component = self.getComponent()
        toothNumber = int(self.getParameter(PARAM_TOOTH_NUMBER).value)

        inputBodies = adsk.core.ObjectCollection.create()
        inputBodies.add(ctx.toothBody)

        patternFeatures = component.features.circularPatternFeatures
        patternInput = patternFeatures.createInput(inputBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        pattern = patternFeatures.add(patternInput)

        # Pattern.bodies already includes the seed + copies; feed all as-is.
        toolBodies = adsk.core.ObjectCollection.create()
        for i in range(pattern.bodies.count):
            toolBodies.add(pattern.bodies.item(i))

        combineFeatures = component.features.combineFeatures
        combineInput = combineFeatures.createInput(ctx.gearBody, toolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineFeatures.add(combineInput)

        self.createFillets(ctx)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value
        axisNormal = get_normal(ctx.plane)

        edges = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            surface = face.geometry
            if surface.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            if abs(surface.radius - rootRadius) > 0.001:
                continue
            for edge in face.edges:
                if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                geom = edge.geometry
                direction = geom.startPoint.vectorTo(geom.endPoint)
                direction.normalize()
                dot = direction.dotProduct(axisNormal)
                if abs(abs(dot) - 1.0) < 0.01:
                    edges.add(edge)

        if edges.count == 0:
            return

        filletFeatures = component.features.filletFeatures
        filletInput = filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(filletRadius), False)
        filletFeatures.add(filletInput)

    def buildBore(self, ctx):
        # Runs unconditionally from generate(); early-return in the two cases
        # where there is nothing (or no body) to cut.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return
        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            return

        component = self.getComponent()
        boreSketch = self.createSketchObject('Bore Profile', ctx.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch

        generator = SpurGearInvoluteToothDesignGenerator(boreSketch, self)
        generator.drawBore(ctx.anchorPoint, boreDiameter)

        profile = boreSketch.profiles.item(0)
        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(
            extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(extrudeInput)

    def cleanup(self, ctx):
        sketchOnly = self.getParameterAsBoolean(PARAM_SKETCH_ONLY)

        # Construction planes/axes: ALWAYS hidden (both modes).
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if self.normalizedPlane is not None:
            self.normalizedPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False

        # Sketches: hidden only on the full-build path.
        if not sketchOnly:
            if self.toolsSketch is not None:
                self.toolsSketch.isVisible = False
            if ctx.gearProfileSketch is not None:
                ctx.gearProfileSketch.isVisible = False
            if self.boreSketch is not None:
                self.boreSketch.isVisible = False
