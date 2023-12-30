import math
import traceback
from .misc import *
from .base import *

class SpurGearSpecification(Specification):
    def __init__(self, module=1, toothNumber=17, pressureAngle=math.radians(20), boreDiameter=None, thickness=5, chamferTooth=0):
        # Note: all angles are in radians
        self.module = module
        self.toothNumber = toothNumber
        self.pressureAngle = pressureAngle
        self.pitchCircleDiameter = toothNumber * module
        self.pitchCircleRadius = self.pitchCircleDiameter / 2.0
        self.baseCircleDiameter = self.pitchCircleDiameter * math.cos(pressureAngle)
        self.baseCircleRadius = self.baseCircleDiameter / 2.0
        self.tipClearance = 0 if self.toothNumber < 3 else module / 6
        self.rootCircleDiameter = self.pitchCircleDiameter - 2 * (module + self.tipClearance)
        self.rootCircleRadius = self.rootCircleDiameter / 2.0
        self.tipCircleDiameter  = self.pitchCircleDiameter + 2 * module
        self.tipCircleRadius  = self.tipCircleDiameter / 2.0
        self.circularPitch = self.pitchCircleDiameter * math.pi / toothNumber
        self.boreDiameter = None
        if boreDiameter is not None:
            if boreDiameter < 0:
                self.boreDiameter = self.baseCircleDiameter / 4
            elif boreDiameter > 0 and boreDiameter < 2:
                self.boreDiameter = 2
            else:
                self.boreDiameter = boreDiameter

        # s = (math.acos(self.baseCircleDiameter/self.pitchCircleDiameter)/16)
        self.involuteSteps = 15
        self.thickness = thickness
        self.chamferTooth = chamferTooth

class SpurGearCommandInputs:
    def __init__(self, cmd):
        inputs = cmd.commandInputs
        self.container = inputs
        self.inputs = {
            'module': inputs.addValueInput('module', 'Module', '', adsk.core.ValueInput.createByReal(1)),
            'toothNumber': inputs.addValueInput('toothNumber', 'Tooth Number', '', adsk.core.ValueInput.createByReal(17)),
            'pressureAngle': inputs.addValueInput('pressureAngle', 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20))),
            'boreDiameter': inputs.addValueInput('boreDiameter', 'Bore Diameter', 'mm', adsk.core.ValueInput.createByReal(0)),
            'thickness': inputs.addValueInput('thickness', 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10))),
            'chamferTooth': inputs.addValueInput('chamferTooth', 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0)),
        }

    def setValue(self, name, value: adsk.core.CommandInput):
        self.inputs[name] = value

    def getValue(self, name):
        unitsManager = get_design().unitsManager

        if isinstance(self.inputs[name], adsk.core.BoolValueCommandInput):
            return (self.inputs[name].value, True)

        if not unitsManager.isValidExpression(self.inputs[name].expression, self.inputs[name].unitType):
            return (None, False)
        
        return (unitsManager.evaluateExpression(self.inputs[name].expression, self.inputs[name].unitType), True)
    
    def toSpecificationArgs(self):
        args = {}
        (module, ok) = self.getValue('module')
        if not ok:
            raise Exception('Invalid module value')
        args['module'] = module

        (toothNumber, ok) = self.getValue('toothNumber')
        if ok:
            args['toothNumber'] = toothNumber

        (pressureAngle, ok) = self.getValue('pressureAngle')
        if ok:
            args['pressureAngle'] = pressureAngle

        (boreDiameter, ok) = self.getValue('boreDiameter')
        if ok:
            args['boreDiameter'] = to_mm(boreDiameter)

        (thickness, ok) = self.getValue('thickness')
        if ok:
            args['thickness'] = to_mm(thickness)
        
        (chamferTooth, ok) = self.getValue('chamferTooth')
        if ok:
            args['chamferTooth'] = to_mm(chamferTooth)

        return args

    def toSpecification(kwargs):
        return SpurGearSpecification(**kwargs)

    def validate(self):
        return True

# The SpurGenerationContext represents an object to carry around context data
# while generating a gear.
class SpurGearGenerationContext(GenerationContext):
    def __init__(self):
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)

# The spur gear tooth profile is used in a few different places, so
# it is separated out into a standalone object
class SpurGearInvoluteToothDesignGenerator():
    def __init__(self, sketch: adsk.fusion.Sketch, spec: SpurGearSpecification, angle=0):
        self.sketch = sketch
        self.spec = spec
        # The angle to rotate the tooth at the end
        self.toothAngle = angle
        # The anchor point is where we draw the circle from. We move this point
        # as necessary later.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.baseCircle = adsk.fusion.SketchCircle.cast(None)
        self.pitchCircle = adsk.fusion.SketchCircle.cast(None)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)

    def drawCircle(self, name, radius, anchorPoint, dimensionAngle=0, isConstruction=True):
        curves = self.sketch.sketchCurves
        dimensions = self.sketch.sketchDimensions
        texts = self.sketch.sketchTexts

        obj = curves.sketchCircles.addByCenterRadius(anchorPoint, to_cm(radius))
        obj.isConstruction = isConstruction

        # Draw the diameter dimension at the specified angle
        x = 0
        y = 0
        if dimensionAngle == 0:
            x = to_cm(radius)
        elif dimensionAngle != 0:
            x = to_cm((radius/2)*math.sin(dimensionAngle))
            y = to_cm((radius/2)*math.cos(dimensionAngle))
        dimensions.addDiameterDimension(
            obj,
            adsk.core.Point3D.create(x, y, 0)
        )
        input = texts.createInput2('{} (r={:.2f})'.format(name, radius), to_cm((self.spec.tipCircleRadius-self.spec.rootCircleRadius)/3))
        input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        texts.add(input)
        return obj

    def drawCircles(self):
        # Root circle
        self.rootCircle = self.drawCircle('Root Circle', self.spec.rootCircleRadius, self.anchorPoint, dimensionAngle=math.radians(15), isConstruction=False)
        # Tip circle
        self.tipCircle = self.drawCircle('Tip Circle', self.spec.tipCircleRadius, self.anchorPoint, dimensionAngle=math.radians(30))

        # These two circles are mainly just for debugging purposes, except for
        # the tip circle, which is used to determine the center point for the
        # tooth tip curve.

        # Base circle
        self.baseCircle = self.drawCircle('Base Circle', self.spec.baseCircleRadius, self.anchorPoint, dimensionAngle=math.radians(45))
        # Pitch circle (reference)
        self.pitchCircle = self.drawCircle('Pitch Circle', self.spec.pitchCircleRadius, self.anchorPoint, dimensionAngle=math.radians(60))

    def draw(self, anchorPoint, angle=0):
        self.drawCircles()
        # Draw a single tooth at the specified angle relative to the x axis
        self.drawTooth(angle=angle)

        # Now we have all the sketches necessary. Move the entire drawing by
        # moving the anchor point to its intended location (where we defined it in
        # a separate Tools sketch)
        projectedAnchorPoint = self.sketch.project(anchorPoint).item(0)
        self.sketch.geometricConstraints.addCoincident(projectedAnchorPoint, self.anchorPoint)

    def drawTooth(self, angle=0):
        sketch = self.sketch
        spec = self.spec
        anchorPoint = self.anchorPoint

        constraints = sketch.geometricConstraints
        curves = sketch.sketchCurves
        dimensions = sketch.sketchDimensions
        points = sketch.sketchPoints
        involutePoints = []
        involuteSize = spec.tipCircleRadius - spec.baseCircleRadius
        involuteSpinePoints = []
        for i in (range(0, spec.involuteSteps)):
            intersectionRadius = spec.baseCircleRadius + ((involuteSize / (spec.involuteSteps-1))*i)
            involutePoint = self.calculateInvolutePoint(spec.baseCircleRadius, intersectionRadius)
            if involutePoint is not None:
                involutePoints.append(involutePoint)
                involuteSpinePoints.append(adsk.core.Point3D.create(involutePoint.x, 0, 0))
    
        pitchInvolutePoint = self.calculateInvolutePoint(spec.baseCircleRadius, spec.pitchCircleRadius)
        pitchPointAngle = math.atan(pitchInvolutePoint.y / pitchInvolutePoint.x)
    
        # Determine the angle defined by the tooth thickness as measured at
        # the pitch diameter circle.
        toothThicknessAngle = math.pi / spec.toothNumber
            
        backlash = 0
        # Determine the angle needed for the specified backlash.
        backlashAngle = (backlash / (spec.pitchCircleRadius)) * .25

        # Determine the angle to rotate the curve.
        rotateAngle = -((toothThicknessAngle/2) + pitchPointAngle - backlashAngle)
        
        # Rotate the involute so the middle of the tooth lies on the x axis.
        cosAngle = math.cos(rotateAngle)
        sinAngle = math.sin(rotateAngle)
        for i in range(0, len(involutePoints)):
            newX = involutePoints[i].x * cosAngle - involutePoints[i].y * sinAngle
            newY = involutePoints[i].x * sinAngle + involutePoints[i].y * cosAngle
            involutePoints[i] = adsk.core.Point3D.create(newX, newY, 0)

        pointCollection = adsk.core.ObjectCollection.create()
        for i in (range(0, len(involutePoints))): #spec.involuteSteps)):
            pointCollection.add(involutePoints[i])
        lline = sketch.sketchCurves.sketchFittedSplines.add(pointCollection)

        pointCollection = adsk.core.ObjectCollection.create()
        for i in (range(0, len(involutePoints))): #spec.involuteSteps)):
            pointCollection.add(adsk.core.Point3D.create(involutePoints[i].x, -involutePoints[i].y, 0))
        rline = sketch.sketchCurves.sketchFittedSplines.add(pointCollection)

        # Draw the the top of the tooth
        toothTopPoint = points.add(adsk.core.Point3D.create(to_cm(spec.tipCircleRadius), 0, 0))
        constraints.addCoincident(toothTopPoint, self.tipCircle)
        top = sketch.sketchCurves.sketchArcs.addByThreePoints(
            rline.endSketchPoint,
            toothTopPoint.geometry,
            lline.endSketchPoint
        )
        dimensions.addDiameterDimension(
            top,
            adsk.core.Point3D.create(toothTopPoint.geometry.x, 0, 0)
        )
    
        # Create a "spine" for the involutes so that they can be fully constrainted
        # without having to fix them
        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(anchorPoint, toothTopPoint)
        spine.isConstruction = True
        angleDimension = None
        if angle == 0:
            constraints.addHorizontal(spine)
        else:
            horizontal = sketch.sketchCurves.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create(anchorPoint.geometry.x, anchorPoint.geometry.y, 0),
                adsk.core.Point3D.create(toothTopPoint.geometry.x, toothTopPoint.geometry.y, 0),
            )
            horizontal.isConsutruction = True
            constraints.addHorizontal(horizontal)
            constraints.addCoincident(horizontal.startSketchPoint, anchorPoint)
            constraints.addCoincident(horizontal.endSketchPoint, self.tipCircle)

            angleDimension = dimensions.addAngularDimension(spine, horizontal,
                adsk.core.Point3D.create(anchorPoint.geometry.x, anchorPoint.geometry.y, 0))

        # Create a series of lines (ribs) from one involute to the other
        priv = anchorPoint
        for i in range(0, len(involutePoints)):
            # First create a point on the spine where the ribs are going to
            # be constrained on 
            lpoint = lline.fitPoints.item(i)
            rib = curves.sketchLines.addByTwoPoints(
                lpoint,
                rline.fitPoints.item(i)
            )
            rib.isConstruction = True
            dimensions.addDistanceDimension(
                rib.startSketchPoint,
                rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(rib.startSketchPoint.geometry.x, rib.startSketchPoint.geometry.y/2, 0)
            )
            spinePoint = points.add(adsk.core.Point3D.create(lpoint.geometry.x, 0, 0))
            constraints.addCoincident(spinePoint, spine)
            constraints.addMidPoint(spinePoint, rib)
            constraints.addPerpendicular(spine, rib)
            dimensions.addDistanceDimension(priv, spinePoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, adsk.core.Point3D.create((lpoint.geometry.x-priv.geometry.x)/2+priv.geometry.x, 0, 0))
            priv = spinePoint

            # Create the point where the involutes will connect to the root circle
            def drawRootToInvoluteLine(root, inv, x, y):
                angle = math.atan(y/ x)
                point = adsk.core.Point3D.create(
                    to_cm(spec.rootCircleRadius) * math.cos(angle),
                    to_cm(spec.rootCircleRadius) * math.sin(angle),
                    0,
                )
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(point, inv.startSketchPoint)
                constraints.addTangent(inv, line)
                constraints.addCoincident(line.startSketchPoint, root)
                dimensions.addAngularDimension(spine, line, line.endSketchPoint.geometry)
                return line
        
            rlline = drawRootToInvoluteLine(self.rootCircle, lline, involutePoints[0].x, involutePoints[0].y)
            rrline = drawRootToInvoluteLine(self.rootCircle, rline, involutePoints[0].x, -involutePoints[0].y)
        if angle != 0:
            # Only do this _AFTER_ all the lines have been drawn
            angleDimension.value = angle


    def calculateInvolutePoint(self, baseCircleRadius, intersectionRadius):
        alpha = math.acos( baseCircleRadius / intersectionRadius)
        if alpha <= 0:
            return None
        invAlpha = math.tan(alpha) - alpha
        return adsk.core.Point3D.create(
            to_cm(intersectionRadius*math.cos(invAlpha)),
            to_cm(intersectionRadius*math.sin(invAlpha)),
            0)

class SpurGearGenerator(Generator):
    def newContext(self):
        return SpurGearGenerationContext()
    def generateName(self, spec):
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(spec.module, spec.toothNumber, spec.thickness)

    def generate(self, spec):
        self.component.name = self.generateName(spec)
        ctx = self.newContext()

        # Create tools to draw and otherwise position the gear with.
        self.prepareTools(ctx)

        # Create the main body of the gear
        self.buildMainGearBody(ctx, spec)
        #self.buildBore(ctx, spec)

    def toothThickness(self, spec: SpurGearSpecification):
        return adsk.core.ValueInput.createByReal(to_cm(spec.thickness))
    
    def prepareTools(self, ctx: GenerationContext):
        # Create a sketch that contains the anchorPoint and the lines that
        # define its position
        sketch = self.createSketchObject('Tools')

        ctx.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        projectedConstructionPoint = sketch.project(self.component.originConstructionPoint).item(0)
        sketch.geometricConstraints.addCoincident(ctx.anchorPoint, projectedConstructionPoint)
        sketch.isVisible = False
    
    def buildSketches(self, ctx: GenerationContext, spec: SpurGearSpecification):
        sketch = self.createSketchObject('Gear Profile')

        SpurGearInvoluteToothDesignGenerator(sketch, spec).draw(ctx.anchorPoint)
        ctx.gearProfileSketch = sketch

    def buildMainGearBody(self, ctx: GenerationContext, spec: SpurGearSpecification):
        self.buildSketches(ctx, spec)
        self.buildTooth(ctx, spec)
        self.buildBody(ctx, spec)
        self.patternTeeth(ctx, spec)

    def buildTooth(self, ctx: GenerationContext, spec :SpurGearSpecification):
        extrudes = self.component.features.extrudeFeatures
        profiles = ctx.gearProfileSketch.profiles
        distance = self.toothThickness(spec)

        toothProfile = profiles.item(0)
        toothExtrude = extrudes.addSimple(toothProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # note: toothBody must be populated before chamferTooth
        ctx.toothBody = toothExtrude.bodies.item(0)
        if spec.chamferTooth > 0:
            self.chamferTooth(ctx, spec)
    
    def buildBody(self, ctx: GenerationContext, spec: SpurGearSpecification):
        extrudes = self.component.features.extrudeFeatures
        distance = self.toothThickness(spec)

        # First create the cylindrical part so we can construct a
        # perpendicular axis
        gearBodyProfile = ctx.gearProfileSketch.profiles.item(1)
        gearBodyExtrude = extrudes.addSimple(gearBodyProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        gearBodyExtrude.bodies.item(0).name = 'Gear Body'

        circularFace = None
        for face in gearBodyExtrude.bodies.item(0).faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                circularFace = face
                break
        
        if circularFace is None:
            raise Exception("Could not find circular face")
        
        axisInput = self.component.constructionAxes.createInput()
        axisInput.setByCircularFace(circularFace)
        centerAxis = self.component.constructionAxes.add(axisInput)
        if centerAxis is None:
            raise Exception("Could not create axis")

        centerAxis.name = 'Gear Center'
        centerAxis.isVisibile = False
        ctx.centerAxis = centerAxis
        # store the gear body for later use
        ctx.gearBody = self.component.bRepBodies.itemByName('Gear Body')

    def patternTeeth(self, ctx: GenerationContext, spec :SpurGearSpecification):
        circular = self.component.features.circularPatternFeatures
        toothBody = ctx.toothBody

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(toothBody)

        patternInput = circular.createInput(bodies, ctx.centerAxis)
        patternInput.quantity = adsk.core.ValueInput.createByReal(spec.toothNumber)
        patternedTeeth = circular.add(patternInput)

        toolBodies = adsk.core.ObjectCollection.create()
        for body in patternedTeeth.bodies:
            toolBodies.add(body)
        combineInput = self.component.features.combineFeatures.createInput(
            self.component.bRepBodies.itemByName('Gear Body'),
            toolBodies
        )
        self.component.features.combineFeatures.add(combineInput)

    def chamferWantEdges(self, spec :SpurGearSpecification):
        return 6

    def chamferTooth(self, ctx: GenerationContext, spec :SpurGearSpecification):
        # Note: this does not take into account when the tooth is thin enough
        # that the chamfer will not be applied. We need to figure out a soft limite
        # to apply a chamfer, and avoid doing this calculation altogeher if it
        # doesn't work
        toothBody = ctx.toothBody
        splineEdges = adsk.core.ObjectCollection.create()

        wantSurfaceType = adsk.core.SurfaceTypes.PlaneSurfaceType

        wantEdges = self.chamferWantEdges(spec)
        # The selection of face/edge is ... very hard coded. If there's a better way
        # (more deterministic) way, we should use that instead
        for face in toothBody.faces:
            # Surface must be a plane, not like a cylindrical surface
            if face.geometry.surfaceType != wantSurfaceType:
                continue
            # face must contain exactly 6 edges... if it's a regular spur gear
            # otherwise if it's a herringbone gear, it's going to be 7
            if len(face.edges) != wantEdges:
                continue

            if wantEdges == 4:
                # Only accept this face if the edges contain exactly two splines
                splineCount = 0
                for edge in face.edges:
                    if edge.geometry.objectType == 'adsk::core::NurbsCurve3D':
                        splineCount += 1

                
                if splineCount != 2:
                    continue


            # We don't want to chamfer the Arc that is part of the root circle.
            for edge in face.edges:
                if edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                    if abs(edge.geometry.radius - to_cm(spec.rootCircleRadius)) < 0.001:
                        continue
                splineEdges.add(edge)

        chamferInput = self.component.features.chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(splineEdges, adsk.core.ValueInput.createByReal(to_cm(spec.chamferTooth)), False)
        self.component.features.chamferFeatures.add(chamferInput)