import math

from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal

# ---------------------------------------------------------------------------
# Dialog input ids and registered user-parameter names.
#
# These literal strings are part of the reproduced surface: they appear in the
# command dialog and the post-generation user-parameter table, and saved
# designs reference them. They must be reproduced verbatim.
# ---------------------------------------------------------------------------
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

PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
PARAM_PRESSURE_ANGLE = 'PressureAngle'
PARAM_BORE_DIAMETER = 'BoreDiameter'
PARAM_THICKNESS = 'Thickness'
PARAM_CHAMFER_TOOTH = 'ChamferTooth'
PARAM_SKETCH_ONLY = 'SketchOnly'

PARAM_PITCH_CIRCLE_DIAMETER = 'PitchCircleDiameter'
PARAM_PITCH_CIRCLE_RADIUS = 'PitchCircleRadius'
PARAM_BASE_CIRCLE_DIAMETER = 'BaseCircleDiameter'
PARAM_BASE_CIRCLE_RADIUS = 'BaseCircleRadius'
PARAM_ROOT_CIRCLE_DIAMETER = 'RootCircleDiameter'
PARAM_ROOT_CIRCLE_RADIUS = 'RootCircleRadius'
PARAM_TIP_CIRCLE_DIAMETER = 'TipCircleDiameter'
PARAM_TIP_CIRCLE_RADIUS = 'TipCircleRadius'
PARAM_INVOLUTE_STEPS = 'InvoluteSteps'
PARAM_TOOTH_SPACE_ANGLE_AT_ROOT = 'ToothSpaceAngleAtRoot'
PARAM_TOOTH_SPACE_ARC_AT_ROOT = 'ToothSpaceArcAtRoot'
PARAM_FILLET_CLEARANCE = 'FilletClearance'
PARAM_FILLET_RADIUS = 'FilletRadius'


class SpurGearCommandInputsConfigurator:
    """Adds the Spur Gear dialog inputs, in the order the spec lists them.

    Selection inputs come first (Parent, Target Plane, Anchor Point are stashed
    before occurrence creation), then the numeric/boolean inputs, and the
    Parent Component selection is listed last because its default (the root
    component) is correct for most uses.
    """

    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        planeInput = inputs.addSelectionInput(
            INPUT_ID_PLANE, 'Target Plane',
            'Plane the gear front face is sketched on')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        anchorInput = inputs.addSelectionInput(
            INPUT_ID_ANCHOR_POINT, 'Anchor Point',
            'Point the gear center is aligned with')
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        anchorInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        anchorInput.setSelectionLimits(1)

        inputs.addValueInput(
            INPUT_ID_MODULE, 'Module', '',
            adsk.core.ValueInput.createByReal(1))

        inputs.addValueInput(
            INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '',
            adsk.core.ValueInput.createByReal(17))

        inputs.addValueInput(
            INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg',
            adsk.core.ValueInput.createByString('20 deg'))

        # Bore Diameter is a STRING value input so it accepts expressions
        # ('5 mm', a user-parameter name, ...). Default '0 mm' = no bore.
        inputs.addStringValueInput(
            INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')

        inputs.addValueInput(
            INPUT_ID_THICKNESS, 'Thickness', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(10)))

        inputs.addValueInput(
            INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        inputs.addBoolValueInput(
            INPUT_ID_SKETCH_ONLY,
            'Generate sketches, but do not build body', True, '', False)

        componentInput = inputs.addSelectionInput(
            INPUT_ID_PARENT, 'Parent Component',
            'Component that will contain the new Spur Gear')
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
        componentInput.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
        componentInput.setSelectionLimits(1)
        componentInput.addSelection(get_design().rootComponent)


class SpurGearGenerationContext(GenerationContext):
    """Data carrier passed between the generation steps. The field names here
    are public API — helical/herringbone subclasses reach in by name, so they
    must not be renamed."""

    def __init__(self):
        self.plane = None
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.extrusionExtent = adsk.fusion.BRepFace.cast(None)
        self.toothProfileIsEmbedded = False


class SpurGearInvoluteToothDesignGenerator:
    """Draws the four gear circles + a single involute tooth into the given
    sketch, then anchors the whole drawing onto the user's anchor point.

    Constructed as (sketch, parent[, angle]); `parent` is the generator and is
    used to read the registered user parameters via parent.getParameter(...).
    """

    def __init__(self, sketch, parent, angle=0):
        self.sketch = sketch
        self.parent = parent
        self.toothAngle = angle
        self.toothProfileIsEmbedded = False

        # The movable local origin. A FRESH SketchPoint at (0,0,0) — NOT
        # sketch.originPoint (which is immutable and can't be coincident
        # constrained to an entity projected in from elsewhere). All geometry
        # is drawn relative to this point, then it is constrained coincident to
        # the projected anchor as draw()'s final action.
        self.anchorPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(0, 0, 0))

    # -- parameter access helpers --------------------------------------------

    def parameterValue(self, name):
        return self.parent.getParameter(name).value

    # -- public surface ------------------------------------------------------

    def draw(self, anchorPoint, angle=0):
        # draw() performs, in order: drawCircles(), drawTooth(angle), then the
        # step-5 anchoring as its own final action. The anchoring lives HERE
        # (not in buildSketches) so helical/herringbone, which call this
        # method directly on their twisted loft sketch, also get anchored.
        self.drawCircles()
        self.drawTooth(angle)

        # Step 5: chain this sketch to the Tools-sketch anchor and slide the
        # whole drawing onto it.
        projectedList = self.sketch.project(anchorPoint)
        projectedAnchor = projectedList.item(0)
        self.sketch.geometricConstraints.addCoincident(
            projectedAnchor, self.anchorPoint)

    def calculateInvolutePoint(self, baseRadius, intersectionRadius):
        """Return the Point3D on the involute (unrolled from baseRadius) whose
        distance from the center equals intersectionRadius. Returns None when
        the intersection radius is inside the base circle."""
        if intersectionRadius < baseRadius:
            return None
        # The pressure angle at intersectionRadius.
        alpha = math.acos(baseRadius / intersectionRadius)
        # The involute (roll) parameter: tan(alpha) - alpha.
        invAlpha = math.tan(alpha) - alpha
        x = intersectionRadius * math.cos(invAlpha)
        y = intersectionRadius * math.sin(invAlpha)
        return adsk.core.Point3D.create(x, y, 0)

    # -- circles -------------------------------------------------------------

    def drawCircles(self):
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions
        texts = sketch.sketchTexts

        rootRadius = self.parameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.parameterValue(PARAM_TIP_CIRCLE_RADIUS)
        baseRadius = self.parameterValue(PARAM_BASE_CIRCLE_RADIUS)
        pitchRadius = self.parameterValue(PARAM_PITCH_CIRCLE_RADIUS)

        size = tipRadius - rootRadius

        def labelCircle(circle, name, radius):
            try:
                textInput = texts.createInput2(
                    f'{name} (r={to_mm(radius):.3f}, size={to_mm(size):.3f})',
                    size / 4 if size > 0 else to_cm(1))
                # Place the along-path text along this circle's curve.
                textInput.setAsAlongPath(circle, True,
                    adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
                texts.add(textInput)
            except Exception:
                # Labeling is cosmetic; never let it break generation.
                futil.log(f'Could not label circle {name}')

        # 1. Root Circle (solid, not construction). Center on the shared local
        #    origin SketchPoint by passing it DIRECTLY (share, don't
        #    re-coincident).
        rootCircle = circles.addByCenterRadius(self.anchorPoint, rootRadius)
        dims.addDiameterDimension(
            rootCircle,
            adsk.core.Point3D.create(rootRadius, rootRadius, 0))
        labelCircle(rootCircle, 'Root Circle', rootRadius)

        # 2. Tip Circle (construction).
        tipCircle = circles.addByCenterRadius(self.anchorPoint, tipRadius)
        tipCircle.isConstruction = True
        dims.addDiameterDimension(
            tipCircle,
            adsk.core.Point3D.create(tipRadius, tipRadius, 0))
        labelCircle(tipCircle, 'Tip Circle', tipRadius)

        # 3. Base Circle (construction).
        baseCircle = circles.addByCenterRadius(self.anchorPoint, baseRadius)
        baseCircle.isConstruction = True
        dims.addDiameterDimension(
            baseCircle,
            adsk.core.Point3D.create(baseRadius, baseRadius, 0))
        labelCircle(baseCircle, 'Base Circle', baseRadius)

        # 4. Pitch Circle (construction).
        pitchCircle = circles.addByCenterRadius(self.anchorPoint, pitchRadius)
        pitchCircle.isConstruction = True
        dims.addDiameterDimension(
            pitchCircle,
            adsk.core.Point3D.create(pitchRadius, pitchRadius, 0))
        labelCircle(pitchCircle, 'Pitch Circle', pitchRadius)

    # -- tooth ---------------------------------------------------------------

    def drawTooth(self, angle=0):
        sketch = self.sketch
        constraints = sketch.geometricConstraints
        dims = sketch.sketchDimensions
        splines = sketch.sketchCurves.sketchFittedSplines
        lines = sketch.sketchCurves.sketchLines
        arcs = sketch.sketchCurves.sketchArcs

        toothNumber = int(round(self.parameterValue(PARAM_TOOTH_NUMBER)))
        involuteSteps = int(round(self.parameterValue(PARAM_INVOLUTE_STEPS)))
        baseRadius = self.parameterValue(PARAM_BASE_CIRCLE_RADIUS)
        rootRadius = self.parameterValue(PARAM_ROOT_CIRCLE_RADIUS)
        tipRadius = self.parameterValue(PARAM_TIP_CIRCLE_RADIUS)
        pitchRadius = self.parameterValue(PARAM_PITCH_CIRCLE_RADIUS)

        # --- 1. Sample involute points from base circle out to tip circle. ---
        # The flank starts on the base circle (or the root circle if the base
        # circle is inside it) and walks outward in equal radial steps. Drop
        # any sample whose involute parameter would be non-positive.
        startRadius = max(baseRadius, rootRadius)
        rawPoints = []
        for i in range(involuteSteps):
            if involuteSteps > 1:
                r = startRadius + (tipRadius - startRadius) * (i / (involuteSteps - 1))
            else:
                r = startRadius
            p = self.calculateInvolutePoint(baseRadius, r)
            if p is None:
                # Sample sits inside the base circle — no valid involute.
                continue
            rawPoints.append(p)

        # --- 2. Mirror across +X (negate y) so the spiral matches a LEFT
        #        flank's narrowing-toward-tip shape. ---
        mirrored = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in rawPoints]

        # --- 3. Rotation so the tooth ends symmetric about +X. ---
        # Analytic pitch-circle crossing angle of the (un-mirrored) involute.
        pitchPoint = self.calculateInvolutePoint(baseRadius, pitchRadius)
        pitchAngle = math.atan2(pitchPoint.y, pitchPoint.x)
        # After mirroring, the pitch crossing sits at -pitchAngle. We want the
        # left flank's pitch crossing at +pi/(2*toothNumber).
        targetHalfAngle = math.pi / (2 * toothNumber)
        rotateAngle = targetHalfAngle - (-pitchAngle)

        # --- 4. Rotate mirrored samples -> LEFT flank; mirror that -> RIGHT. -
        def rotate(points, theta):
            c = math.cos(theta)
            s = math.sin(theta)
            return [adsk.core.Point3D.create(
                p.x * c - p.y * s, p.x * s + p.y * c, 0) for p in points]

        leftPoints = rotate(mirrored, rotateAngle)
        rightPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in leftPoints]

        # --- 5. Draw the two flanks as fitted splines. ---
        leftCollection = adsk.core.ObjectCollection.create()
        for p in leftPoints:
            leftCollection.add(p)
        leftSpline = splines.add(leftCollection)

        rightCollection = adsk.core.ObjectCollection.create()
        for p in rightPoints:
            rightCollection.add(p)
        rightSpline = splines.add(rightCollection)

        # --- 6. Tooth-top arc. Minimal constraint set only. ---
        # Tooth-top point at (tipRadius, 0), constrained coincident to the tip
        # circle (the only coincidence here). Do NOT constrain the arc center.
        toothTopPoint = sketch.sketchPoints.add(
            adsk.core.Point3D.create(tipRadius, 0, 0))
        tipCircle = None
        for c in sketch.sketchCurves.sketchCircles:
            if abs(c.radius - tipRadius) < 1e-7:
                tipCircle = c
                break
        if tipCircle is not None:
            constraints.addCoincident(toothTopPoint, tipCircle)

        # Arc shares the two flank END SketchPoints directly; the tooth-top
        # point's *geometry* is the middle through-point.
        topArc = arcs.addByThreePoints(
            rightSpline.endSketchPoint,
            toothTopPoint.geometry,
            leftSpline.endSketchPoint)
        # A single driving diameter dimension equal to the tip diameter; that
        # plus the two shared flank ends fully determine the arc.
        dims.addDiameterDimension(
            topArc,
            adsk.core.Point3D.create(0, tipRadius, 0))

        # --- 7. Spine: construction line sharing localOrigin and toothTop. ---
        # Pass both existing SketchPoints DIRECTLY (share), no .geometry, no
        # extra start-coincident, no end-on-arc constraint.
        spine = lines.addByTwoPoints(self.anchorPoint, toothTopPoint)
        spine.isConstruction = True

        if angle == 0:
            # Spur: the spine's only additional constraint is horizontal.
            constraints.addHorizontal(spine)
            angularDim = None
        else:
            # Subclass (helical/herringbone): horizontal reference line + an
            # angular dimension measured FROM the spine TO the horizontal, in
            # that argument order (the order fixes the tilt sign / helix hand).
            refEnd = adsk.core.Point3D.create(tipRadius, 0, 0)
            horizontal = lines.addByTwoPoints(self.anchorPoint, refEnd)
            horizontal.isConstruction = True
            constraints.addHorizontal(horizontal)
            angularDim = dims.addAngularDimension(
                spine, horizontal,
                adsk.core.Point3D.create(tipRadius / 2, tipRadius / 4, 0))

        # --- 8. Ribs between matching left/right flank fit points. ---
        leftFit = leftSpline.fitPoints
        rightFit = rightSpline.fitPoints
        ribCount = min(leftFit.count, rightFit.count)
        previousMidpoint = self.anchorPoint  # chain starts at local origin
        for i in range(ribCount):
            leftFitPoint = leftFit.item(i)
            rightFitPoint = rightFit.item(i)

            # (1) rib shares the two fit-point SketchPoints directly.
            rib = lines.addByTwoPoints(leftFitPoint, rightFitPoint)
            rib.isConstruction = True

            # (2) dimension the rib's aligned length to its measured value.
            dims.addDistanceDimension(
                rib.startSketchPoint, rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    (leftFitPoint.geometry.x + rightFitPoint.geometry.x) / 2,
                    (leftFitPoint.geometry.y + rightFitPoint.geometry.y) / 2,
                    0))

            # (3) midpoint created ALREADY on the spine at (fit.x, 0, 0).
            midpoint = sketch.sketchPoints.add(
                adsk.core.Point3D.create(leftFitPoint.geometry.x, 0, 0))

            # (4) coincident(midpoint, spine) FIRST,
            #     (5) midPoint(midpoint, rib),
            #     (6) perpendicular(spine, rib). Order matters.
            constraints.addCoincident(midpoint, spine)
            constraints.addMidPoint(midpoint, rib)
            constraints.addPerpendicular(spine, rib)

            # Chain distance from the previous midpoint; for the FIRST rib the
            # previous is the local origin (without this the chain has a
            # residual DOF and the sketch never fully constrains).
            dims.addDistanceDimension(
                previousMidpoint, midpoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(
                    midpoint.geometry.x, -tipRadius / 8, 0))
            previousMidpoint = midpoint

        # --- 9. Flank-to-root lines (only when flank start is OUTSIDE root). -
        flankStartRadius = math.hypot(
            leftFit.item(0).geometry.x, leftFit.item(0).geometry.y)
        rootCircle = None
        for c in sketch.sketchCurves.sketchCircles:
            if abs(c.radius - rootRadius) < 1e-7:
                rootCircle = c
                break

        if flankStartRadius > rootRadius + 1e-9:
            # Non-embedded: draw a short radial line on each side from the root
            # circle up to the flank's start point. Loop has 6 curves.
            self.toothProfileIsEmbedded = False
            for flankStartPoint in (leftFit.item(0), rightFit.item(0)):
                # Root end seeded on the origin->flank-start ray at root radius.
                fx = flankStartPoint.geometry.x
                fy = flankStartPoint.geometry.y
                flen = math.hypot(fx, fy)
                rootEnd = adsk.core.Point3D.create(
                    fx / flen * rootRadius, fy / flen * rootRadius, 0)
                # Share the flank START SketchPoint directly as the far end.
                line = lines.addByTwoPoints(rootEnd, flankStartPoint)
                # Exactly these two constraints, no others:
                #  (a) the line's root-end coincident to the root circle,
                if rootCircle is not None:
                    constraints.addCoincident(line.startSketchPoint, rootCircle)
                #  (b) the local origin coincident to the line (point-on-line).
                constraints.addCoincident(self.anchorPoint, line)
        else:
            # Embedded: no flank-to-root stubs; the loop is 4 curves.
            self.toothProfileIsEmbedded = True

        # --- The tilt is DRIVEN, applied as the very last action, after the
        #     whole constraint network exists. For a spur (angle==0) there is
        #     no angular dimension. ---
        if angle != 0 and angularDim is not None:
            angularDim.parameter.value = angle

    # -- bore (drawn into a dedicated Bore Profile sketch) -------------------

    def drawBore(self, anchorPoint, boreRadius):
        """Draw a single bore circle of `boreRadius` centered on the projected
        anchor in this generator's sketch. Used by the bore step (step 12)."""
        sketch = self.sketch
        circles = sketch.sketchCurves.sketchCircles
        dims = sketch.sketchDimensions

        projectedList = sketch.project(anchorPoint)
        projectedAnchor = projectedList.item(0)

        circle = circles.addByCenterRadius(projectedAnchor, boreRadius)
        dims.addDiameterDimension(
            circle,
            adsk.core.Point3D.create(boreRadius, boreRadius, 0))
        return circle


class SpurGearGenerator(Generator):
    """Orchestrates the full spur gear build. Subclassed by helicalgear and
    herringbonegear; the method names and call-graph boundaries below are
    public API."""

    # -- overridable hooks ---------------------------------------------------

    def newContext(self):
        return SpurGearGenerationContext()

    def prefixBase(self):
        return 'SpurGear'

    def generateName(self):
        return 'Spur Gear'

    def addExtraPrimaryParameters(self, inputs):
        # Spur has no extra primary parameters. Helical injects HelixAngle
        # here, BEFORE the derived parameters reference it.
        pass

    def filletHelixFactorExpression(self):
        # Spur: the fillet radius factor is always 1. Helical returns
        # f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'.
        return '1'

    def chamferWantEdges(self):
        # Expected front-face edge count for a spur tooth loop: 6 when the
        # tooth is non-embedded (2 splines + 2 arcs + 2 flank-to-root lines).
        return 6

    # -- input reading + parameter registration -----------------------------

    def processInputs(self, inputs):
        # Order is load-bearing (see the spec's "Generation Order"): pull EVERY
        # selection out of `inputs` and stash it on self BEFORE anything
        # creates the occurrence (which shifts Fusion's active component
        # context and can drop selections). Numeric/bool inputs are immune.

        # 1. Parent component.
        (parentEntities, _) = get_selection(inputs, INPUT_ID_PARENT)
        if len(parentEntities) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PARENT}' missing")
        entity = parentEntities[0]
        if entity.objectType == adsk.fusion.Occurrence.classType():
            self.parentComponent = entity.component
        elif entity.objectType == adsk.fusion.Component.classType():
            self.parentComponent = entity
        else:
            raise Exception(f'Invalid parent component type: {entity.objectType}')

        # 2. Target plane and anchor point.
        (planeEntities, _) = get_selection(inputs, INPUT_ID_PLANE)
        if len(planeEntities) != 1:
            raise Exception(f"Required selection '{INPUT_ID_PLANE}' missing")
        self.plane = planeEntities[0]

        (anchorEntities, _) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(anchorEntities) != 1:
            raise Exception(f"Required selection '{INPUT_ID_ANCHOR_POINT}' missing")
        self.anchorPoint = anchorEntities[0]

        # 3. Numeric / boolean inputs -> user parameters. Reading these (via
        #    addParameter) creates the occurrence; selections are already
        #    stashed, so the context shift is harmless from here on.
        (module, _) = get_value(inputs, INPUT_ID_MODULE, '')
        self.addParameter(PARAM_MODULE, module, '', 'Module of the gear')

        (toothNumber, _) = get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')
        self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '',
                          'Number of teeth')

        (pressureAngle, _) = get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'rad')
        self.addParameter(PARAM_PRESSURE_ANGLE, pressureAngle, 'rad',
                          'Pressure angle')

        # Bore Diameter is an addStringValueInput field, so get_value can
        # return ok=False with the EVALUATED NUMBER (internal cm) rather than a
        # ValueInput. Wrap THAT number — wrapping 0 would silently ignore a
        # typed bore value like '5 mm' and no hole would be cut.
        (boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
        if not ok:
            # get_value returned the evaluated value as a raw number
            # (internal cm). Wrap THAT — wrapping 0 would silently ignore a
            # typed bore value.
            boreDiameter = adsk.core.ValueInput.createByReal(
                boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
        self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm',
                          'Size of the bore')

        (thickness, _) = get_value(inputs, INPUT_ID_THICKNESS, 'mm')
        self.addParameter(PARAM_THICKNESS, thickness, 'mm',
                          'Axial length of the gear body')

        (chamferTooth, _) = get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')
        self.addParameter(PARAM_CHAMFER_TOOTH, chamferTooth, 'mm',
                          'Chamfer distance on the tooth front face')

        # SketchOnly persisted as a real-valued parameter (1=true, 0=false)
        # because the framework reads numeric parameters as booleans.
        (sketchOnly, _) = get_boolean(inputs, INPUT_ID_SKETCH_ONLY)
        self.addParameter(
            PARAM_SKETCH_ONLY,
            adsk.core.ValueInput.createByReal(1 if sketchOnly else 0),
            '', 'Generate sketches only (1=true, 0=false)')

        # 4. Hook for subclass primary parameters (HelixAngle, ...), BEFORE the
        #    derived parameters reference them.
        self.addExtraPrimaryParameters(inputs)

        # 5. Derived parameters as live Fusion expression strings.
        def expr(s):
            return adsk.core.ValueInput.createByString(s)

        moduleName = self.parameterName(PARAM_MODULE)
        toothNumberName = self.parameterName(PARAM_TOOTH_NUMBER)
        pressureAngleName = self.parameterName(PARAM_PRESSURE_ANGLE)

        self.addParameter(
            PARAM_PITCH_CIRCLE_DIAMETER,
            expr(f'{moduleName} * {toothNumberName}'), 'mm',
            'Pitch circle diameter')
        pitchDiameterName = self.parameterName(PARAM_PITCH_CIRCLE_DIAMETER)

        self.addParameter(
            PARAM_PITCH_CIRCLE_RADIUS,
            expr(f'{pitchDiameterName} / 2'), 'mm', 'Pitch circle radius')

        self.addParameter(
            PARAM_BASE_CIRCLE_DIAMETER,
            expr(f'{pitchDiameterName} * cos({pressureAngleName})'), 'mm',
            'Base circle diameter')
        baseDiameterName = self.parameterName(PARAM_BASE_CIRCLE_DIAMETER)

        self.addParameter(
            PARAM_BASE_CIRCLE_RADIUS,
            expr(f'{baseDiameterName} / 2'), 'mm', 'Base circle radius')

        self.addParameter(
            PARAM_ROOT_CIRCLE_DIAMETER,
            expr(f'{pitchDiameterName} - 2.5 * {moduleName}'), 'mm',
            'Root circle diameter')
        rootDiameterName = self.parameterName(PARAM_ROOT_CIRCLE_DIAMETER)

        self.addParameter(
            PARAM_ROOT_CIRCLE_RADIUS,
            expr(f'{rootDiameterName} / 2'), 'mm', 'Root circle radius')

        self.addParameter(
            PARAM_TIP_CIRCLE_DIAMETER,
            expr(f'{pitchDiameterName} + 2 * {moduleName}'), 'mm',
            'Tip circle diameter')
        tipDiameterName = self.parameterName(PARAM_TIP_CIRCLE_DIAMETER)

        self.addParameter(
            PARAM_TIP_CIRCLE_RADIUS,
            expr(f'{tipDiameterName} / 2'), 'mm', 'Tip circle radius')

        self.addParameter(
            PARAM_INVOLUTE_STEPS,
            adsk.core.ValueInput.createByReal(15), '',
            'Number of involute sample points')

        # 6. ToothSpaceAngleAtRoot — pre-computed in Python because Fusion's
        #    expression engine refuses to mix the unitless tan() output with
        #    the radian-valued pressure angle in a subtraction. Registered as
        #    UNITLESS ('') even though it holds a radian value, so the next
        #    parameter's product with a length (RootCircleRadius) reads as a
        #    pure length (mm); registering it as 'rad' makes that product
        #    mm·rad and Fusion rejects the dependent parameter.
        toothNumberValue = self.getParameter(PARAM_TOOTH_NUMBER).value
        pressureAngleValue = self.getParameter(PARAM_PRESSURE_ANGLE).value
        toothSpaceAngle = (
            math.pi / toothNumberValue
            - 2 * (math.tan(pressureAngleValue) - pressureAngleValue))
        self.addParameter(
            PARAM_TOOTH_SPACE_ANGLE_AT_ROOT,
            adsk.core.ValueInput.createByReal(toothSpaceAngle), '',
            'Angular width of the valley at the root circle (radians, '
            'stored unitless)')

        rootRadiusName = self.parameterName(PARAM_ROOT_CIRCLE_RADIUS)
        toothSpaceAngleName = self.parameterName(PARAM_TOOTH_SPACE_ANGLE_AT_ROOT)
        self.addParameter(
            PARAM_TOOTH_SPACE_ARC_AT_ROOT,
            expr(f'{rootRadiusName} * {toothSpaceAngleName}'), 'mm',
            'Arc length of the valley along the root circle')
        toothSpaceArcName = self.parameterName(PARAM_TOOTH_SPACE_ARC_AT_ROOT)

        self.addParameter(
            PARAM_FILLET_CLEARANCE,
            adsk.core.ValueInput.createByReal(0.9), '',
            'Fraction of the half-valley arc used for the fillet radius')
        filletClearanceName = self.parameterName(PARAM_FILLET_CLEARANCE)

        # FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>.
        # The factor is 1 for spur; helical overrides filletHelixFactorExpression.
        self.addParameter(
            PARAM_FILLET_RADIUS,
            expr(f'({toothSpaceArcName} / 2) * {filletClearanceName} * '
                 f'{self.filletHelixFactorExpression()}'), 'mm',
            'Root fillet radius')

    # -- generation orchestration -------------------------------------------

    def generate(self, inputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # Normalize the plane to a ConstructionPlane (e.g. when the user picked
        # a planar face). A coplanar construction plane keeps profile-detection
        # from tripping over the selected face's native profile. When we create
        # one here, keep a handle so the end-of-build cleanup can switch its
        # light bulb off (a ConstructionPlane hides with isLightBulbOn, not
        # isVisible).
        self.normalizedPlane = None
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            planeInput = component.constructionPlanes.createInput()
            planeInput.setByOffset(
                self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(planeInput)
            self.normalizedPlane = self.plane

        ctx = self.newContext()
        ctx.plane = self.plane

        self.prepareTools(ctx)
        self.buildMainGearBody(ctx)
        self.buildBore(ctx)

    # -- step 1-2: tools sketch + extrusion end plane -----------------------

    def prepareTools(self, ctx):
        component = self.getComponent()

        # Tools sketch on the target plane: owns the projected anchor — the
        # canonical handle every later sketch re-projects from. Keep it visible
        # while later sketches still project from it.
        toolsSketch = self.createSketchObject('Tools', self.plane)
        toolsSketch.isVisible = True

        projectedList = toolsSketch.project(self.anchorPoint)
        ctx.anchorPoint = projectedList.item(0)
        self.toolsSketch = toolsSketch

        # Offset construction plane 'Extrusion End Plane' at Thickness; serves
        # as the to-entity target for the tooth and body extrudes.
        thickness = self.getParameter(PARAM_THICKNESS).value
        planeInput = component.constructionPlanes.createInput()
        planeInput.setByOffset(
            self.plane, adsk.core.ValueInput.createByReal(thickness))
        endPlane = component.constructionPlanes.add(planeInput)
        endPlane.name = 'Extrusion End Plane'
        ctx.extrusionEndPlane = endPlane

    # -- main gear body ------------------------------------------------------

    def buildMainGearBody(self, ctx):
        self.buildSketches(ctx)

        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            # Step 6: show the Gear Profile sketch and stop.
            ctx.gearProfileSketch.isVisible = True
            return

        self.buildTooth(ctx)   # extrude tooth, then chamferTooth (steps 7-8)
        self.buildBody(ctx)    # extrude annular body + axis + extent (step 9)
        self.patternTeeth(ctx) # pattern + combine, then createFillets (10-11)

    # -- step 3-5: gear profile sketch + tooth generator --------------------

    def buildSketches(self, ctx):
        gearProfileSketch = self.createSketchObject('Gear Profile', self.plane)
        gearProfileSketch.isVisible = True
        ctx.gearProfileSketch = gearProfileSketch

        generator = SpurGearInvoluteToothDesignGenerator(
            gearProfileSketch, self)
        generator.draw(ctx.anchorPoint, angle=0)
        ctx.toothProfileIsEmbedded = generator.toothProfileIsEmbedded

    # -- step 7-8: extrude tooth + chamfer ----------------------------------

    def buildTooth(self, ctx):
        component = self.getComponent()
        sketch = ctx.gearProfileSketch

        toothProfile = self._findToothProfile(sketch, ctx.toothProfileIsEmbedded)
        if toothProfile is None:
            raise Exception('Could not find the tooth cross-section profile')

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            toothProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extentToEnd = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extentToEnd,
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeFeature = extrudes.add(extrudeInput)
        extrudeFeature.name = 'Extrude tooth'
        ctx.toothBody = extrudeFeature.bodies.item(0)

        # buildTooth MUST call chamferTooth as its last action (so the chamfer
        # is triggered from inside buildTooth, not separately).
        self.chamferTooth(ctx)

    def chamferTooth(self, ctx):
        chamferDistance = self.getParameter(PARAM_CHAMFER_TOOTH).value
        if chamferDistance <= 0:
            return

        component = self.getComponent()
        wantEdges = self.chamferWantEdges()

        # Find the tooth's front face: the planar face on the target plane
        # whose edge count matches the loop we drew (6 for a spur tooth).
        frontFace = None
        planeNormal = get_normal(self.plane)
        for face in ctx.toothBody.faces:
            if face.geometry.objectType != adsk.core.Plane.classType():
                continue
            if face.edges.count != wantEdges:
                continue
            faceNormal = get_normal(face)
            if faceNormal is None:
                continue
            # Front face shares the target plane's normal direction.
            if abs(abs(faceNormal.dotProduct(planeNormal)) - 1.0) > 1e-6:
                continue
            frontFace = face
            break

        if frontFace is None:
            futil.log('Chamfer: could not identify the tooth front face')
            return

        # Chamfer every edge of that face EXCEPT the arc shared with the root
        # valley (chamfering it would eat into the neighbouring tooth).
        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        edgeCollection = adsk.core.ObjectCollection.create()
        for edge in frontFace.edges:
            geo = edge.geometry
            if geo.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(geo.radius - rootRadius) < 1e-5:
                    # Skip the root-valley arc.
                    continue
            edgeCollection.add(edge)

        if edgeCollection.count == 0:
            return

        chamfers = component.features.chamferFeatures
        chamferInput = chamfers.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            edgeCollection,
            adsk.core.ValueInput.createByReal(chamferDistance), True)
        chamfers.add(chamferInput)

    # -- step 9: extrude the annular body -----------------------------------

    def buildBody(self, ctx):
        component = self.getComponent()
        sketch = ctx.gearProfileSketch

        bodyProfile = self._findBodyProfile(sketch)
        if bodyProfile is None:
            raise Exception('Could not find the annular gear-body profile')

        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            bodyProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extentToEnd = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionEndPlane, False)
        extrudeInput.setOneSideExtent(
            extentToEnd,
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeFeature = extrudes.add(extrudeInput)
        extrudeFeature.name = 'Extrude body'
        gearBody = extrudeFeature.bodies.item(0)
        gearBody.name = 'Gear Body'
        ctx.gearBody = gearBody

        # Capture the Gear Center axis and the far end-cap face.
        planeNormal = get_normal(self.plane)
        planeGeo = self.plane.geometry
        for face in gearBody.faces:
            geo = face.geometry
            if geo.objectType == adsk.core.Cylinder.classType():
                if ctx.centerAxis is None:
                    axisInput = component.constructionAxes.createInput()
                    axisInput.setByCircularFace(face)
                    centerAxis = component.constructionAxes.add(axisInput)
                    centerAxis.name = 'Gear Center'
                    centerAxis.isLightBulbOn = False
                    ctx.centerAxis = centerAxis
            elif geo.objectType == adsk.core.Plane.classType():
                faceNormal = get_normal(face)
                if faceNormal is None:
                    continue
                # Planar face parallel to the target plane...
                if abs(abs(faceNormal.dotProduct(planeNormal)) - 1.0) > 1e-6:
                    continue
                # ...but not coplanar with it => the far end cap.
                toOrigin = planeGeo.origin.vectorTo(geo.origin)
                if abs(toOrigin.dotProduct(planeNormal)) > 1e-6:
                    ctx.extrusionExtent = face

        if ctx.centerAxis is None:
            raise Exception('Could not build the Gear Center axis')

    # -- step 10-11: pattern teeth + combine + fillets ----------------------

    def patternTeeth(self, ctx):
        component = self.getComponent()
        toothNumber = int(round(self.getParameter(PARAM_TOOTH_NUMBER).value))

        seedBodies = adsk.core.ObjectCollection.create()
        seedBodies.add(ctx.toothBody)

        patterns = component.features.circularPatternFeatures
        patternInput = patterns.createInput(seedBodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(
            float(toothNumber))
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        patternInput.isSymmetric = False
        patternResult = patterns.add(patternInput)
        patternResult.name = 'Pattern teeth'

        # CircularPatternFeature.bodies already includes the seed body at
        # position 0 plus the N-1 copies; feed the whole collection to Combine
        # as-is (do NOT re-add ctx.toothBody or it gets double-counted).
        combineTools = adsk.core.ObjectCollection.create()
        for i in range(patternResult.bodies.count):
            combineTools.add(patternResult.bodies.item(i))

        combines = component.features.combineFeatures
        combineInput = combines.createInput(ctx.gearBody, combineTools)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isNewComponent = False
        combineInput.isKeepToolBodies = False
        combineFeature = combines.add(combineInput)
        combineFeature.name = 'Combine teeth'

        # patternTeeth MUST end by calling createFillets.
        self.createFillets(ctx)

    def createFillets(self, ctx):
        filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value
        if filletRadius <= 0:
            return

        component = self.getComponent()
        rootRadius = self.getParameter(PARAM_ROOT_CIRCLE_RADIUS).value
        planeNormal = get_normal(self.plane)

        # Collect EVERY cylindrical face whose radius equals the root radius
        # (the root cylinder is usually split into one patch per valley after
        # the pattern-and-combine), and on each keep the AXIAL straight edges.
        edgeCollection = adsk.core.ObjectCollection.create()
        for face in ctx.gearBody.faces:
            geo = face.geometry
            if geo.objectType != adsk.core.Cylinder.classType():
                continue
            if abs(geo.radius - rootRadius) > 1e-5:
                continue
            for edge in face.edges:
                ec = edge.geometry
                # Drop circular end-cap rims; keep only straight axial edges.
                if ec.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
                    continue
                (ok, startPt, endPt) = edge.geometry.evaluator.getEndPoints()
                if not ok:
                    continue
                direction = startPt.vectorTo(endPt)
                direction.normalize()
                # Tangent parallel to the gear main axis (target-plane normal).
                if abs(abs(direction.dotProduct(planeNormal)) - 1.0) > 1e-4:
                    continue
                edgeCollection.add(edge)

        if edgeCollection.count == 0:
            return

        fillets = component.features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            edgeCollection,
            adsk.core.ValueInput.createByReal(filletRadius), True)
        fillets.add(filletInput)

    # -- step 12: bore (optional) -------------------------------------------

    def buildBore(self, ctx):
        # In SketchOnly mode the body was never built (step 6 stopped after the
        # Gear Profile sketch and left it visible on purpose). Skip the bore and
        # the end-of-build cleanup entirely so that sketch stays visible.
        if self.getParameterAsBoolean(PARAM_SKETCH_ONLY):
            return

        boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value
        if boreDiameter <= 0:
            # Default '0 mm' => no bore. With the get_value wrap above, a typed
            # value such as '5 mm' arrives here non-zero and a hole IS cut.
            self._finishBuild(ctx)
            return

        component = self.getComponent()

        # Separate Bore Profile sketch on the target plane.
        boreSketch = self.createSketchObject('Bore Profile', self.plane)
        boreSketch.isVisible = True
        self.boreSketch = boreSketch

        projectedList = boreSketch.project(self.anchorPoint)
        projectedAnchor = projectedList.item(0)

        boreRadius = boreDiameter / 2
        circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
            projectedAnchor, boreRadius)
        boreSketch.sketchDimensions.addDiameterDimension(
            circle, adsk.core.Point3D.create(boreRadius, boreRadius, 0))

        boreProfile = None
        for p in boreSketch.profiles:
            boreProfile = p
            break
        if boreProfile is None:
            raise Exception('Could not find the bore profile')

        # Extrude-cut from the target plane to the far end cap, affecting only
        # the gear body. ToEntityExtentDefinition guarantees through-cut
        # regardless of Thickness.
        extrudes = component.features.extrudeFeatures
        extrudeInput = extrudes.createInput(
            boreProfile,
            adsk.fusion.FeatureOperations.CutFeatureOperation)
        extentToEnd = adsk.fusion.ToEntityExtentDefinition.create(
            ctx.extrusionExtent, False)
        extrudeInput.setOneSideExtent(
            extentToEnd,
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        extrudeInput.participantBodies = [ctx.gearBody]
        extrudeFeature = extrudes.add(extrudeInput)
        extrudeFeature.name = 'Bore'

        self._finishBuild(ctx)

    def _finishBuild(self, ctx):
        # End-of-build cleanup. Run only after EVERY feature has consumed the
        # construction sketches, planes and axes (the gear is fully built by
        # now). Leaves only the finished gear body visible.
        #
        # Two different "hide" mechanisms, and using the wrong one is a known
        # Fusion gotcha:
        #   * A Sketch is hidden with isVisible = False.
        #   * A ConstructionPlane / ConstructionAxis is NOT affected by
        #     isVisible at all — it hides only with isLightBulbOn = False. The
        #     stock generator left a commented-out extrusionEndPlane.isVisible
        #     = False, which is exactly why the Extrusion End Plane stayed
        #     visible. Use the light bulb instead.

        # Sketches: Tools, Gear Profile, and (if drawn) Bore Profile.
        if getattr(self, 'toolsSketch', None) is not None:
            self.toolsSketch.isVisible = False
        if ctx.gearProfileSketch is not None:
            ctx.gearProfileSketch.isVisible = False
        if getattr(self, 'boreSketch', None) is not None:
            self.boreSketch.isVisible = False

        # Construction planes and axes: every one created during the build.
        #   * Extrusion End Plane (to-entity target for the tooth/body extrudes)
        #   * Gear Center axis (already off, but assert it here too)
        #   * the normalized target plane, if step 1 created one
        if ctx.extrusionEndPlane is not None:
            ctx.extrusionEndPlane.isLightBulbOn = False
        if ctx.centerAxis is not None:
            ctx.centerAxis.isLightBulbOn = False
        if getattr(self, 'normalizedPlane', None) is not None:
            self.normalizedPlane.isLightBulbOn = False

    # -- profile finders -----------------------------------------------------

    def _curveTypeCounts(self, loop):
        counts = {
            adsk.core.Curve3DTypes.NurbsCurve3DCurveType: 0,
            adsk.core.Curve3DTypes.Arc3DCurveType: 0,
            adsk.core.Curve3DTypes.Line3DCurveType: 0,
            adsk.core.Curve3DTypes.Circle3DCurveType: 0,
        }
        for curve in loop.profileCurves:
            ct = curve.geometry.curveType
            if ct in counts:
                counts[ct] += 1
        return counts

    def _findToothProfile(self, sketch, embedded):
        # Non-embedded tooth loop: 2 NURBS flanks + 2 arcs (tooth top + root
        # arc) + 2 line segments (flank-to-root). Embedded: 2 NURBS + 2 arcs.
        wantNurbs = 2
        wantArcs = 2
        wantLines = 0 if embedded else 2
        for p in sketch.profiles:
            for loop in p.profileLoops:
                counts = self._curveTypeCounts(loop)
                if (counts[adsk.core.Curve3DTypes.NurbsCurve3DCurveType] == wantNurbs
                        and counts[adsk.core.Curve3DTypes.Arc3DCurveType] == wantArcs
                        and counts[adsk.core.Curve3DTypes.Line3DCurveType] == wantLines):
                    return p
        return None

    def _findBodyProfile(self, sketch):
        # The annular body loop is bounded by exactly 2 arcs/circles (the root
        # circle and the tip circle) and nothing else.
        for p in sketch.profiles:
            for loop in p.profileLoops:
                counts = self._curveTypeCounts(loop)
                arcLike = (counts[adsk.core.Curve3DTypes.Arc3DCurveType]
                           + counts[adsk.core.Curve3DTypes.Circle3DCurveType])
                others = (counts[adsk.core.Curve3DTypes.NurbsCurve3DCurveType]
                          + counts[adsk.core.Curve3DTypes.Line3DCurveType])
                if arcLike == 2 and others == 0:
                    return p
        return None
