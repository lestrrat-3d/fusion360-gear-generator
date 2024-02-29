import math
from ...lib import fusion360utils as futil
from .misc import *
from .base import *
from .utilities import *

PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'

def as_param_value(value: adsk.core.ValueInput) -> str:
    vt = value.valueType
    match vt:
        case adsk.core.ValueTypes.RealValueType:
            return f'{value.realValue}'
        case adsk.core.ValueTypes.StringValueType:
            return value.stringValue
        case _:
            return None
        
class SpurGearSpecification(Specification):
    def __init__(self, component, occurrence, plane=None, module=1, toothNumber=17, pressureAngle=math.radians(20), boreDiameter=None, thickness=5, chamferTooth=0, sketchOnly=False, anchorPoint=None, filletRadius=None):
        pass
    @classmethod
    def get_value(cls, inputs: adsk.core.CommandInputs, name):
        input = inputs.itemById(name)
        futil.log(f'input is {input}')

        design = get_design()
        unitsManager = design.unitsManager
        userParameters = design.userParameters
        if input.classType == adsk.core.StringValueCommandInput.classType:
            value = input.value
            if userParameters.itemByName(value) == None:
                evaluated = unitsManager.evaluateExpression(value, 'mm')
                if evaluated == None:
                    raise(f'Failed to evaluate expression "{value}"')
                return (evaluated, False)
            return (adsk.core.ValueInput.createByString(value), True)

        if not unitsManager.isValidExpression(input.expression, input.unitType):
            return (None, False)

        evaluated = unitsManager.evaluateExpression(input.expression, input.unitType)
        if input.unitType == 'cm':
            evaluated = to_mm(evaluated)
        return (adsk.core.ValueInput.createByReal(evaluated), True)
    

    @classmethod
    def to_args(cls, inputs: adsk.core.CommandInputs):
        args = {}
        (values, ok) = cls.get_selection(inputs, INPUT_ID_PLANE)
        if len(values) == 1 and ok:
            args[INPUT_ID_PLANE] = values[0] # must be exactly one item
        
        (values, ok) = cls.get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(values) == 1 and ok:
            args[INPUT_ID_ANCHOR_POINT] = values[0] # must be exactly one item

        (module, ok) = cls.get_value(inputs, 'module')
        if not ok:
            raise Exception('Invalid module value')
        args['module'] = module

        (toothNumber, ok) = cls.get_value(inputs, 'toothNumber')
        if ok:
            args['toothNumber'] = toothNumber

        (pressureAngle, ok) = cls.get_value(inputs, 'pressureAngle')
        if ok:
            args['pressureAngle'] = pressureAngle

        (boreDiameter, ok) = cls.get_value(inputs, 'boreDiameter')
        if ok:
            args['boreDiameter'] = boreDiameter

        (thickness, ok) = cls.get_value(inputs, 'thickness')
        if ok:
            args['thickness'] = thickness
        
        (chamferTooth, ok) = cls.get_value(inputs, 'chamferTooth')
        if ok:
            args['chamferTooth'] = chamferTooth

        (sketchOnly, ok) = cls.get_boolean(inputs, 'sketchOnly')
        if ok:
            args['sketchOnly'] = sketchOnly

        return args
    
    @classmethod
    def from_inputs(cls, component, occurrence, inputs):
        args = SpurGearSpecification.to_args(inputs)
        return SpurGearSpecification(component, occurrence, **args)
    
class SpurGearCommandInputValidator:
    @classmethod
    def validate(cls, cmd):
        inputs = cmd.commandInputs
        moduleInput = inputs.itemById('module')

# Configures the command input
class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        inputs = cmd.commandInputs

        planeInput = inputs.addSelectionInput(INPUT_ID_PLANE, 'Select Plane', 'Select a plane and to position the gear')
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
        planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
        planeInput.setSelectionLimits(1)

        pointInput = inputs.addSelectionInput(INPUT_ID_ANCHOR_POINT, 'Select Point', 'Select a point to anchor the gear')
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
        pointInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
        pointInput.setSelectionLimits(1)

        moduleInput = inputs.addValueInput('module', PARAM_MODULE, '', adsk.core.ValueInput.createByReal(1))
        moduleInput.isFullWidth = False
        inputs.addValueInput('toothNumber', 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))
        inputs.addValueInput('pressureAngle', 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))
        inputs.addStringValueInput('boreDiameter', 'Bore Diameter', '0 mm')
        inputs.addValueInput('thickness', 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))
        inputs.addValueInput('chamferTooth', 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))
        inputs.addBoolValueInput('sketchOnly', 'Generate sketches, but do not build body', True, '', False)

# The SpurGenerationContext represents an object to carry around context data
# while generating a gear.
class SpurGearGenerationContext(GenerationContext):
    def __init__(self):
        self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
        self.gearBody = adsk.fusion.BRepBody.cast(None)
        self.toothBody = adsk.fusion.BRepBody.cast(None)
        self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
        self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
        self.extrusionExtent = adsk.core.Surface.cast(None)
        self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.toothProfileIsEmbedded = False

# The spur gear tooth profile is used in a few different places, so
# it is separated out into a standalone object
class SpurGearInvoluteToothDesignGenerator():
    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
        self.parent = parent
        self.sketch = sketch
        # The angle to rotate the tooth at the end
        self.toothAngle = angle
        # The anchor point is where we draw the circle from. This value must
        # initially be (0, 0, 0) for ease of computation.
        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
        self.baseCircle = adsk.fusion.SketchCircle.cast(None)
        self.pitchCircle = adsk.fusion.SketchCircle.cast(None)
        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
        self.toothProfileIsEmbedded = False

    def drawCircle(self, name, radius, anchorPoint, dimensionAngle=0, isConstruction=True):
        curves = self.sketch.sketchCurves
        dimensions = self.sketch.sketchDimensions
        texts = self.sketch.sketchTexts

        obj = curves.sketchCircles.addByCenterRadius(anchorPoint, radius)
        obj.isConstruction = isConstruction

        # Draw the diameter dimension at the specified angle
        x = 0
        y = 0
        if dimensionAngle == 0:
            x = radius
        elif dimensionAngle != 0:
            x = (radius/2)*math.sin(dimensionAngle)
            y = (radius/2)*math.cos(dimensionAngle)
        dimensions.addDiameterDimension(
            obj,
            adsk.core.Point3D.create(x, y, 0)
        )
        size = self.parent.getParameter('TipCircleRadius').value - self.parent.getParameter('RootCircleRadius').value
        input = texts.createInput2('{} (r={:.2f}, size={:.2f})'.format(name, radius, size), size)
        input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        texts.add(input)
        return obj

    def drawCircles(self):
        # Root circle
        self.rootCircle = self.drawCircle('Root Circle', self.parent.getParameter('RootCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(15), isConstruction=False)
        # Tip circle
        self.tipCircle = self.drawCircle('Tip Circle', self.parent.getParameter('TipCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(30))

        # These two circles are mainly just for debugging purposes, except for
        # the tip circle, which is used to determine the center point for the
        # tooth tip curve.

        # Base circle
        self.baseCircle = self.drawCircle('Base Circle', self.parent.getParameter('BaseCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(45))
        # Pitch circle (reference)
        self.pitchCircle = self.drawCircle('Pitch Circle', self.parent.getParameter('PitchCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(60))

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
        anchorPoint = self.anchorPoint

        constraints = sketch.geometricConstraints
        curves = sketch.sketchCurves
        dimensions = sketch.sketchDimensions
        points = sketch.sketchPoints
        involutePoints = []
        tipCircleRadius = self.parent.getParameter('TipCircleRadius').value
        baseCircleRadius = self.parent.getParameter('BaseCircleRadius').value
        pitchCircleRadius = self.parent.getParameter('PitchCircleRadius').value
        rootCircleRadius = self.parent.getParameter('RootCircleRadius').value

        # The involutes must always go to the smaller of the root/base circle
        biggerCircleRadius = baseCircleRadius # if baseCircleRadius > rootCircleRadius else rootCircleRadius

        involuteSize =  tipCircleRadius - biggerCircleRadius
        involuteSteps = int(self.parent.getParameter('InvoluteSteps').value)
        involuteSpinePoints = []
        for i in range(0, involuteSteps):
            intersectionRadius = biggerCircleRadius + ((involuteSize / (involuteSteps-1))*i)
            involutePoint = self.calculateInvolutePoint(biggerCircleRadius, intersectionRadius)
            if involutePoint is not None:
                involutePoints.append(involutePoint)
                involuteSpinePoints.append(adsk.core.Point3D.create(involutePoint.x, 0, 0))
    
        pitchInvolutePoint = self.calculateInvolutePoint(biggerCircleRadius, pitchCircleRadius)
        pitchPointAngle = math.atan(pitchInvolutePoint.y / pitchInvolutePoint.x)
    
        # Determine the angle defined by the tooth thickness as measured at
        # the pitch diameter circle.
        toothThicknessAngle = math.pi / self.parent.getParameter(PARAM_TOOTH_NUMBER).value

        backlash = 0
        # Determine the angle needed for the specified backlash.
        backlashAngle = (backlash / (pitchCircleRadius)) * .25

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
        toothTopPoint = points.add(adsk.core.Point3D.create(tipCircleRadius, 0, 0))
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
        def drawRootToInvoluteLine(root, rootRadius, inv, x, y):
            angle = math.atan(y/ x)
            point = adsk.core.Point3D.create(
                rootRadius * math.cos(angle),
                rootRadius * math.sin(angle),
                0,
            )

            line = sketch.sketchCurves.sketchLines.addByTwoPoints(point, inv.startSketchPoint)
            constraints.addCoincident(line.startSketchPoint, root)
            constraints.addHorizontal(line)
            return line

#        if rootCircleRadius > baseCircleRadius:
        if math.sqrt(involutePoints[0].x**2 + involutePoints[0].y**2) > rootCircleRadius:
            drawRootToInvoluteLine(self.rootCircle, rootCircleRadius, lline, involutePoints[0].x, involutePoints[0].y)
            drawRootToInvoluteLine(self.rootCircle, rootCircleRadius, rline, involutePoints[0].x, -involutePoints[0].y)
        else:
            self.toothProfileIsEmbedded = True
#            drawRootToInvoluteLine(self.baseCircle, baseCircleRadius, lline, involutePoints[i].x, involutePoints[i].y)
#            drawRootToInvoluteLine(self.baseCircle, baseCircleRadius, rline, involutePoints[i].x, -involutePoints[i].y)

        if angle != 0:
            # Only do this _AFTER_ all the lines have been drawn
            angleDimension.value = angle


    def calculateInvolutePoint(self, baseCircleRadius, intersectionRadius):
        alpha = math.acos( baseCircleRadius / intersectionRadius)
        if alpha <= 0:
            return None
        invAlpha = math.tan(alpha) - alpha
        return adsk.core.Point3D.create(
            intersectionRadius*math.cos(invAlpha),
            intersectionRadius*math.sin(invAlpha),
            0)

    def drawBore(self, anchorPoint=None):
        projectedAnchorPoint = self.sketch.project(anchorPoint).item(0)
        self.drawCircle('Bore Circle', self.parent.getParameter('BoreDiameter').value/2, projectedAnchorPoint, isConstruction=False)


class SpurGearGenerator(Generator):
    def __init__(self, design: adsk.fusion.Design):
        super(SpurGearGenerator, self).__init__(design)
        self.plane = None
        self.anchorPoint = None

    def newContext(self):
        return SpurGearGenerationContext()

    def create_specification_from_inputs(self, inputs):
        return SpurGearSpecification.from_inputs(self.getComponent(), self.getOccurence(), inputs)

    def generateName(self):
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter('Thickness')
        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)
    
    def processInputs(self, inputs: adsk.core.CommandInputs):
        # Note: all angles are in radians
        futil.log(f"inputs 1 {inputs}")
        (values, ok) = get_selection(inputs, INPUT_ID_PLANE)
        if len(values) == 1 and ok:
            self.plane = values[0] # must be exactly one item
        else:
            raise Exception("Require parameter '{INPUT_ID_PLANE}' not available")
    
        futil.log(f"inputs 2 {inputs}")
        (values, ok) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
        if len(values) == 1 and ok:
            self.anchorPoint = values[0] # must be exactly one item
        else:
            raise Exception(f"Require parameter '{INPUT_ID_ANCHOR_POINT}' not available (selected {len(values)} points)")

        (module, ok) = get_value(inputs, 'module', '')
        if not ok:
            raise Exception('Invalid module value')
        self.addParameter(PARAM_MODULE, module, '', 'Module for the spur gear')

        (toothNumber, ok) = get_value(inputs, 'toothNumber', '')
        if ok:
            self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Number of tooth on the spur gera')

        (pressureAngle, ok) = get_value(inputs, 'pressureAngle', 'rad')
        if ok:
            self.addParameter('PressureAngle', pressureAngle, 'rad', 'Pressure angle for spur gear')

        (boreDiameter, ok) = get_value(inputs, 'boreDiameter', 'mm')
        if not ok:
            boreDiameter = adsk.core.ValueInput.createByReal(0)
        self.addParameter('BoreDiameter', boreDiameter, 'mm', 'Size of the bore')

        (thickness, ok) = get_value(inputs, 'thickness', 'mm')
        if ok:
            self.addParameter('Thickness', thickness, 'mm', 'Thickness of the spur gear')
        
        (chamferTooth, ok) = get_value(inputs, 'chamferTooth', 'mm')
        if ok:
            self.addParameter('ChamferTooth', chamferTooth, 'mm', 'Chamfer size')

        (sketchOnly, ok) = get_boolean(inputs, 'sketchOnly')
        if ok:
            self.addParameter('SketchOnly', adsk.core.ValueInput.createByReal(1 if sketchOnly else 0), '', 'Draw sketch only')
    
        self.addParameter('PitchCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("ToothNumber")} * {self.parameterName("Module")}'), 'mm', 'Pitch circle diameter')
        self.addParameter('PitchCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} / 2'), 'mm', 'Pitch circle radius')

        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
        self.addParameter('BaseCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} * cos({self.parameterName("PressureAngle")})'), 'mm', 'Base circle diameter')
        self.addParameter('BaseCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("BaseCircleDiameter")} / 2'), 'mm', 'Base circle radius')

        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
        self.addParameter('RootCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} - 2.5 * {self.parameterName("Module")}'), 'mm', 'Root circle diameter')
        self.addParameter('RootCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("RootCircleDiameter")} / 2'), 'mm', 'Root circle radius')

        self.addParameter('TipCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} + 2 * {self.parameterName("Module")}'), 'mm', 'Tip circle diameter')
        self.addParameter('TipCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("TipCircleDiameter")} / 2'), 'mm', 'Tip circle radius')

        # currently unused
        # self.tipClearance = 0 if self.toothNumber < 3 else module / 6
        # currently unused
        # self.circularPitch = self.pitchCircleDiameter * math.pi / toothNumber
        # s = (math.acos(self.baseCircleDiameter/self.pitchCircleDiameter)/16)
        self.addParameter('InvoluteSteps', adsk.core.ValueInput.createByReal(15), '', 'Number of segments to use when drawing involute')

        self.addParameter('FilletThreshold', adsk.core.ValueInput.createByString(f'{self.parameterName("BaseCircleDiameter")} * PI / ({self.parameterName("ToothNumber")} * 2) * 0.4'), 'mm', 'Maximum possible threshold')
        # For now, filletRadius = filletThreshold
        self.addParameter('FilletRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("FilletThreshold")}'), 'mm', '')

    def drawCircle(self, name: str, sketch: adsk.fusion.Sketch, radius: adsk.core.ValueInput, anchorPoint, dimensionAngle=0, isConstruction=True):
        curves = sketch.sketchCurves
        dimensions = sketch.sketchDimensions
        texts = sketch.sketchTexts

        obj = curves.sketchCircles.addByCenterRadius(anchorPoint, radius)
        obj.isConstruction = isConstruction

        # Draw the diameter dimension at the specified angle
        x = 0
        y = 0
        if dimensionAngle == 0:
            x = radius.value
        elif dimensionAngle != 0:
            x = (radius.value/2)*math.sin(dimensionAngle)
            y = (radius.value/2)*math.cos(dimensionAngle)
        dimensions.addDiameterDimension(
            obj,
            adsk.core.Point3D.create(x, y, 0)
        )
        size = self.parent.getParameter('TipCircleRadius').value - self.parent.getParameter('RootCircleRadius').value
        input = texts.createInput2('{} (r={:.2f})'.format(name, radius), size)
        input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
        texts.add(input)
        return obj

    def generate(self, inputs: adsk.core.CommandInputs):
        self.processInputs(inputs)

        component = self.getComponent()
        component.name = self.generateName()

        # The first thing we want to do is to "fix" the spec so that the plane
        # is a construction plane.
        # This plane _MUST_ be a construction plane in order to avoid having to deal with
        # profile artifacts.
        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
            # Create a co-planer construction plane
            cplaneInput = component.constructionPlanes.createInput()
            cplaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
            self.plane = component.constructionPlanes.add(cplaneInput)

        ctx = self.newContext()

        # Create tools to draw and otherwise position the gear with.
        self.prepareTools(ctx)

        # Create the main body of the gear
        self.buildMainGearBody(ctx)

        self.buildBore(ctx)


    def buildBore(self, ctx: GenerationContext):
        diameter = self.getParameter('BoreDiameter').value
        if diameter <= 0:
            return
        sketch = self.createSketchObject('Bore Profile', self.plane)

        SpurGearInvoluteToothDesignGenerator(sketch,self).drawBore(ctx.anchorPoint)

        extrudes = self.getComponent().features.extrudeFeatures
        boreProfile = None
        for profile in sketch.profiles:
            # There should be a single loop and a single curve
            if profile.profileLoops.count != 1:
                continue

            loop = profile.profileLoops.item(0)
            if loop.profileCurves.count != 1:
                continue
            curve = loop.profileCurves.item(0)
            if curve.geometryType == adsk.core.Curve3DTypes.Circle3DCurveType:
                if abs(curve.geometry.radius - diameter/2) < 0.001:
                    boreProfile = profile
                    break

        if boreProfile is None:
            raise Exception('could not find bore profile')

        boreExtrudeInput = extrudes.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)

        direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
#        if not get_normal(spec.plane).isEqualTo(get_normal(ctx.extrusionExtent)):
#            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection

        boreExtrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False),
            direction,
        )
        boreExtrudeInput.participantBodies = [ctx.gearBody]
        extrudes.add(boreExtrudeInput)

    def prepareTools(self, ctx: GenerationContext):
        # Create a sketch that contains the anchorPoint and the lines that
        # define its position
        sketch = self.createSketchObject('Tools', plane=self.plane)

        # TODO: I think we should standardize on storing these elements in ctx object instead of self
        ctx.anchorPoint = sketch.project(self.anchorPoint)
#        ctx.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
#        projectedConstructionPoint = sketch.project(self.getComponent().originConstructionPoint).item(0)
#        sketch.geometricConstraints.addCoincident(ctx.anchorPoint, projectedConstructionPoint)
        sketch.isVisible = False

        # Create plane to perform extrusions against.
        input = self.getComponent().constructionPlanes.createInput()

        thickness = self.getParameterAsValueInput('Thickness')
        input.setByOffset(self.plane, thickness)
        extrusionEndPlane = self.getComponent().constructionPlanes.add(input)
        # TODO: I can't make this work
        # extrusionEndPlane.isVisible = False
        ctx.extrusionEndPlane = extrusionEndPlane

    
    def buildSketches(self, ctx: GenerationContext):
        sketch = self.createSketchObject('Gear Profile', plane=self.plane)
        designgen = SpurGearInvoluteToothDesignGenerator(sketch, self)
        designgen.draw(ctx.anchorPoint)

        # remember if some specifics of this probile
        if designgen.toothProfileIsEmbedded:
            ctx.toothProfileIsEmbedded = True
        ctx.gearProfileSketch = sketch

    def buildMainGearBody(self, ctx: GenerationContext):
        ui = adsk.core.Application.get().userInterface
        ui.isComputeDeferred = True
        self.buildSketches(ctx)
        ui.isComputeDeferred = False

        # The user could simply want the involute tooth and the circles.
        # In that case, pass on building the body
        if self.getParameterAsBoolean('SketchOnly'):
            ctx.gearProfileSketch.isVisible = True
        else:
            ui.isComputeDeferred = True
            self.buildTooth(ctx)
            ui.isComputeDeferred = False
            ui.isComputeDeferred = True
            self.buildBody(ctx)
            ui.isComputeDeferred = False
            ui.isComputeDeferred = True
            self.patternTeeth(ctx)
            ui.isComputeDeferred = False

    def buildTooth(self, ctx: GenerationContext):
        extrudes = self.getComponent().features.extrudeFeatures
        profiles = ctx.gearProfileSketch.profiles

        # The tooth profile has a very specific shape. We look for that shape
        # in the list of profiles that we have.

        toothProfile = None
        for profile in profiles:
            for loop in profile.profileLoops:
                expectArcs = 2
                expectNurbs = 2
                expectLines = 0
                if ctx.toothProfileIsEmbedded:
                    # The loop must have exactly 4 curves.
                    if loop.profileCurves.count != 4:
                        continue
                else:
                    # The loop must have exactly 6 curves.
                    if loop.profileCurves.count != 6:
                        continue
                    expectLines = 2

                # The curve must consist of Line3D, NurbsCurve3D and Arc3D
                arcs = 0
                nurbs = 0
                lines = 0
                for curve in loop.profileCurves:
                    ctyp = curve.geometry.curveType
                    if ctyp == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    elif  ctyp == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                        nurbs += 1
                    elif  ctyp == adsk.core.Curve3DTypes.Line3DCurveType:
                        lines += 1
                    else:
                        break
                
                if nurbs == expectNurbs and arcs == expectArcs and lines == expectLines:
                    toothProfile = profile
                    break
            if toothProfile:
                break

        if not toothProfile:
            raise Exception("could not find tooth profile")

        toothExtrudeInput = extrudes.createInput(toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        toothExtrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
            adsk.fusion.ExtentDirections.PositiveExtentDirection
        )
        toothExtrude = extrudes.add(toothExtrudeInput)
        toothExtrude.name = 'Extrude tooth'

        # note: toothBody must be populated before chamferTooth
        ctx.toothBody = toothExtrude.bodies.item(0)
        self.chamferTooth(ctx)
    
    def buildBody(self, ctx: GenerationContext):
        extrudes = self.getComponent().features.extrudeFeatures
        # distance = self.toothThickness(spec)

        # First create the cylindrical part so we can construct a
        # perpendicular axis
        profiles = ctx.gearProfileSketch.profiles
        gearBodyProfile = None
        for profile in profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count != 2:
                    continue
                arcs = 0
                for curve in loop.profileCurves:
                    if curve.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                        arcs += 1
                    else:
                        break
                if arcs == 2:
                    gearBodyProfile = profile
                    break

            if gearBodyProfile:
                break

        if not gearBodyProfile:
            raise Exception("could not find gear body profile")

        gearBodyExtrudeInput = extrudes.createInput(
            gearBodyProfile,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        gearBodyExtrudeInput.setOneSideExtent(
            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
            adsk.fusion.ExtentDirections.PositiveExtentDirection
        )
        gearBodyExtrude = extrudes.add(gearBodyExtrudeInput)
        gearBodyExtrude.name = 'Extrude body'
        gearBodyExtrude.bodies.item(0).name = 'Gear Body'

        circularFace = None
        for face in gearBodyExtrude.bodies.item(0).faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                # This face is used to find the axis
                circularFace = face
            elif face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                # If the plane is parallel but NOT coplanar, it's the
                # face that was just created
                sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
                if sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry):
                    ctx.extrusionExtent = face
        
            if circularFace and ctx.extrusionExtent:
                break

        if not circularFace:
            raise Exception("Could not find circular face")
        if not ctx.extrusionExtent: 
            raise Exception("Could not find extrusion extent face")

        axisInput = self.getComponent().constructionAxes.createInput()
        axisInput.setByCircularFace(circularFace)
        centerAxis = self.getComponent().constructionAxes.add(axisInput)
        if centerAxis is None:
            raise Exception("Could not create axis")

        centerAxis.name = 'Gear Center'
        centerAxis.isVisibile = False
        ctx.centerAxis = centerAxis
        # store the gear body for later use
        ctx.gearBody = self.getComponent().bRepBodies.itemByName('Gear Body')

    def patternTeeth(self, ctx: GenerationContext):
        circular = self.getComponent().features.circularPatternFeatures
        toothBody = ctx.toothBody

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(toothBody)

        patternInput = circular.createInput(bodies, ctx.centerAxis)
        patternInput.quantity = self.getParameterAsValueInput(PARAM_TOOTH_NUMBER)
        patternedTeeth = circular.add(patternInput)

        toolBodies = adsk.core.ObjectCollection.create()
        for body in patternedTeeth.bodies:
            toolBodies.add(body)
        combineInput = self.getComponent().features.combineFeatures.createInput(
            self.getComponent().bRepBodies.itemByName('Gear Body'),
            toolBodies
        )
        self.getComponent().features.combineFeatures.add(combineInput)

        self.createFillets(ctx)
    
    def createFillets(self, ctx: GenerationContext):
        fr = self.getParameter('FilletRadius')
        if fr.value <= 0:
            return
        gearBody = self.getComponent().bRepBodies.itemByName('Gear Body')
        edges = adsk.core.ObjectCollection.create()
        rootCircleRadiusCm = self.getParameter('RootCircleRadius').value
        for face in gearBody.faces:
            if face.geometry.objectType == adsk.core.Cylinder.classType():
                if abs(face.geometry.radius - rootCircleRadiusCm) < 0.001:
                    for edge in face.edges:
                        dir = edge.endVertex.geometry.vectorTo(edge.startVertex.geometry)
                        dir.normalize()

                        normal = self.plane.geometry.normal

                        # XXX Hmmm, I thought the edges with dot product = 0
                        # would be the right ones to fillet, but something isn't
                        # working as I expected to...
                        if abs(dir.dotProduct(normal)) > 0.001:
                            edges.add(edge)
                            
        if edges.count > 0:
            filletInput = self.getComponent().features.filletFeatures.createInput()
            filletInput.edgeSetInputs.addConstantRadiusEdgeSet(
                edges,
                # TODO to_cm
                self.getParameterAsValueInput('FilletRadius'),
                False,
            )
            self.getComponent().features.filletFeatures.add(filletInput)


    def chamferWantEdges(self):
        return 6

    def chamferTooth(self, ctx: GenerationContext):
        ct = self.getParameter('ChamferTooth')
        if ct.value <= 0:
            return
        # Note: this does not take into account when the tooth is thin enough
        # that the chamfer will not be applied. We need to figure out a soft limite
        # to apply a chamfer, and avoid doing this calculation altogeher if it
        # doesn't work
        toothBody = ctx.toothBody
        splineEdges = adsk.core.ObjectCollection.create()

        wantSurfaceType = adsk.core.SurfaceTypes.PlaneSurfaceType

        wantEdges = self.chamferWantEdges()
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
            rootCircleRadiusCm = self.getParameter('RootCircleRadius').value
            for edge in face.edges:
                if edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                    if abs(edge.geometry.radius - rootCircleRadiusCm) < 0.001:
                        continue
                splineEdges.add(edge)

        chamferInput = self.getComponent().features.chamferFeatures.createInput2()
        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(splineEdges, self.getParameterAsValueInput('ChamferTooth'), False)
        self.getComponent().features.chamferFeatures.add(chamferInput)