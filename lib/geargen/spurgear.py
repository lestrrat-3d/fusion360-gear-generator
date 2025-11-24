import math
from typing import Optional, Any
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil
from .misc import *
from .base import *
from .utilities import *
from .types import GenerationState, SpurGearSpec
from .core import create_gear_occurrence, ensure_construction_plane, create_sketch, get_parameter, hide_construction_planes
from .inputs import parse_spur_gear_inputs, get_selection_input
from .components import get_parent_component
from .parameters import create_spur_gear_parameters
from .involute import draw_involute_tooth_profile, ToothProfileConfig

PARAM_MODULE = 'Module'
PARAM_TOOTH_NUMBER = 'ToothNumber'
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_PLANE = 'plane'
INPUT_ID_ANCHOR_POINT = 'anchorPoint'

# ==============================================================================
# OLD CLASS-BASED API - COMMENTED OUT (No longer used, replaced by functional API)
# ==============================================================================
# The code below is the old class-based implementation that has been replaced
# by the functional API starting at line ~876. It is commented out to ensure
# it's not accidentally used. The only class still in use is
# SpurGearCommandInputsConfigurator which appears after this section.
# ==============================================================================

# def as_param_value(value: adsk.core.ValueInput) -> str:
#     vt = value.valueType
#     match vt:
#         case adsk.core.ValueTypes.RealValueType:
#             return f'{value.realValue}'
#         case adsk.core.ValueTypes.StringValueType:
#             return value.stringValue
#         case _:
#             return None
#
# class SpurGearSpecification(Specification):
#     def __init__(self, component, occurrence, plane=None, module=1, toothNumber=17, pressureAngle=math.radians(20), boreDiameter=None, thickness=5, chamferTooth=0, sketchOnly=False, anchorPoint=None, filletRadius=None):
#         pass
#     @classmethod
#     def get_value(cls, inputs: adsk.core.CommandInputs, name):
#         input = inputs.itemById(name)
#         futil.log(f'input is {input}')
#
#         design = get_design()
#         unitsManager = design.unitsManager
#         userParameters = design.userParameters
#         if input.classType == adsk.core.StringValueCommandInput.classType:
#             value = input.value
#             if userParameters.itemByName(value) == None:
#                 evaluated = unitsManager.evaluateExpression(value, 'mm')
#                 if evaluated == None:
#                     raise(f'Failed to evaluate expression "{value}"')
#                 return (evaluated, False)
#             return (adsk.core.ValueInput.createByString(value), True)
#
#         if not unitsManager.isValidExpression(input.expression, input.unitType):
#             return (None, False)
#
#         evaluated = unitsManager.evaluateExpression(input.expression, input.unitType)
#         if input.unitType == 'cm':
#             evaluated = to_mm(evaluated)
#         return (adsk.core.ValueInput.createByReal(evaluated), True)
#
#
#     @classmethod
#     def to_args(cls, inputs: adsk.core.CommandInputs):
#         args = {}
#         (values, ok) = cls.get_selection(inputs, INPUT_ID_PLANE)
#         if len(values) == 1 and ok:
#             args[INPUT_ID_PLANE] = values[0] # must be exactly one item
#
#         (values, ok) = cls.get_selection(inputs, INPUT_ID_ANCHOR_POINT)
#         if len(values) == 1 and ok:
#             args[INPUT_ID_ANCHOR_POINT] = values[0] # must be exactly one item
#
#         (module, ok) = cls.get_value(inputs, 'module')
#         if not ok:
#             raise Exception('Invalid module value')
#         args['module'] = module
#
#         (toothNumber, ok) = cls.get_value(inputs, 'toothNumber')
#         if ok:
#             args['toothNumber'] = toothNumber
#
#         (pressureAngle, ok) = cls.get_value(inputs, 'pressureAngle')
#         if ok:
#             args['pressureAngle'] = pressureAngle
#
#         (boreDiameter, ok) = cls.get_value(inputs, 'boreDiameter')
#         if ok:
#             args['boreDiameter'] = boreDiameter
#
#         (thickness, ok) = cls.get_value(inputs, 'thickness')
#         if ok:
#             args['thickness'] = thickness
#
#         (chamferTooth, ok) = cls.get_value(inputs, 'chamferTooth')
#         if ok:
#             args['chamferTooth'] = chamferTooth
#
#         (sketchOnly, ok) = cls.get_boolean(inputs, 'sketchOnly')
#         if ok:
#             args['sketchOnly'] = sketchOnly
#
#         return args
#
#     @classmethod
#     def from_inputs(cls, component, occurrence, inputs):
#         args = SpurGearSpecification.to_args(inputs)
#         return SpurGearSpecification(component, occurrence, **args)
#
# class SpurGearCommandInputValidator:
#     @classmethod
#     def validate(cls, cmd):
#         inputs = cmd.commandInputs
#         moduleInput = inputs.itemById('module')
#
# ==============================================================================
# OLD CLASS-BASED API CONTINUES - COMMENTED OUT
# ==============================================================================

# # The SpurGenerationContext represents an object to carry around context data
# # while generating a gear.
# class SpurGearGenerationContext(GenerationContext):
#     def __init__(self):
#         self.anchorPoint = adsk.fusion.SketchPoint.cast(None)
#         self.gearBody = adsk.fusion.BRepBody.cast(None)
#         self.toothBody = adsk.fusion.BRepBody.cast(None)
#         self.centerAxis = adsk.fusion.ConstructionAxis.cast(None)
#         self.gearProfileSketch = adsk.fusion.Sketch.cast(None)
#         self.extrusionExtent = adsk.core.Surface.cast(None)
#         self.extrusionEndPlane = adsk.fusion.ConstructionPlane.cast(None)
#         self.toothProfileIsEmbedded = False
#
# # The spur gear tooth profile is used in a few different places, so
# # it is separated out into a standalone object
# class SpurGearInvoluteToothDesignGenerator():
#    def __init__(self, sketch: adsk.fusion.Sketch, parent, angle=0):
#        self.parent = parent
#        self.sketch = sketch
#        # The angle to rotate the tooth at the end
#        self.toothAngle = angle
#        # The anchor point is where we draw the circle from. This value must
#        # initially be (0, 0, 0) for ease of computation.
#        self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
#
#        self.rootCircle = adsk.fusion.SketchCircle.cast(None)
#        self.baseCircle = adsk.fusion.SketchCircle.cast(None)
#        self.pitchCircle = adsk.fusion.SketchCircle.cast(None)
#        self.tipCircle = adsk.fusion.SketchCircle.cast(None)
#        self.toothProfileIsEmbedded = False
#
#    def drawCircle(self, name, radius, anchorPoint, dimensionAngle=0, isConstruction=True):
#        curves = self.sketch.sketchCurves
#        dimensions = self.sketch.sketchDimensions
#        texts = self.sketch.sketchTexts
#
#        obj = curves.sketchCircles.addByCenterRadius(anchorPoint, radius)
#        obj.isConstruction = isConstruction
#
#        # Draw the diameter dimension at the specified angle
#        x = 0
#        y = 0
#        if dimensionAngle == 0:
#            x = radius
#        elif dimensionAngle != 0:
#            x = (radius/2)*math.sin(dimensionAngle)
#            y = (radius/2)*math.cos(dimensionAngle)
#        dimensions.addDiameterDimension(
#            obj,
#            adsk.core.Point3D.create(x, y, 0)
#        )
#        size = self.parent.getParameter('TipCircleRadius').value - self.parent.getParameter('RootCircleRadius').value
#        input = texts.createInput2('{} (r={:.2f}, size={:.2f})'.format(name, radius, size), size)
#        input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
#        texts.add(input)
#        return obj
#
#    def drawCircles(self):
#        # Root circle
#        self.rootCircle = self.drawCircle('Root Circle', self.parent.getParameter('RootCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(15), isConstruction=False)
#        # Tip circle
#        self.tipCircle = self.drawCircle('Tip Circle', self.parent.getParameter('TipCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(30))
#
#        # These two circles are mainly just for debugging purposes, except for
#        # the tip circle, which is used to determine the center point for the
#        # tooth tip curve.
#
#        # Base circle
#        self.baseCircle = self.drawCircle('Base Circle', self.parent.getParameter('BaseCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(45))
#        # Pitch circle (reference)
#        self.pitchCircle = self.drawCircle('Pitch Circle', self.parent.getParameter('PitchCircleRadius').value, self.anchorPoint, dimensionAngle=math.radians(60))
#
#    def draw(self, anchorPoint, angle=0):
#        self.drawCircles()
#        # Draw a single tooth at the specified angle relative to the x axis
#        self.drawTooth(angle=angle)
#
#        # Now we have all the sketches necessary. Move the entire drawing by
#        # moving the anchor point to its intended location (where we defined it in
#        # a separate Tools sketch)
#        projectedAnchorPoint = self.sketch.project(anchorPoint).item(0)
#        self.sketch.geometricConstraints.addCoincident(projectedAnchorPoint, self.anchorPoint)
#
#    def drawTooth(self, angle=0):
#        sketch = self.sketch
#        anchorPoint = self.anchorPoint
#
#        constraints = sketch.geometricConstraints
#        curves = sketch.sketchCurves
#        dimensions = sketch.sketchDimensions
#        points = sketch.sketchPoints
#        involutePoints = []
#        tipCircleRadius = self.parent.getParameter('TipCircleRadius').value
#        baseCircleRadius = self.parent.getParameter('BaseCircleRadius').value
#        pitchCircleRadius = self.parent.getParameter('PitchCircleRadius').value
#        rootCircleRadius = self.parent.getParameter('RootCircleRadius').value
#
#        # The involutes must always go to the smaller of the root/base circle
#        biggerCircleRadius = baseCircleRadius # if baseCircleRadius > rootCircleRadius else rootCircleRadius
#
#        involuteSize =  tipCircleRadius - biggerCircleRadius
#        involuteSteps = int(self.parent.getParameter('InvoluteSteps').value)
#        involuteSpinePoints = []
#        for i in range(0, involuteSteps):
#            intersectionRadius = biggerCircleRadius + ((involuteSize / (involuteSteps-1))*i)
#            involutePoint = self.calculateInvolutePoint(biggerCircleRadius, intersectionRadius)
#            if involutePoint is not None:
#                involutePoints.append(involutePoint)
#                involuteSpinePoints.append(adsk.core.Point3D.create(involutePoint.x, 0, 0))
#    
#        pitchInvolutePoint = self.calculateInvolutePoint(biggerCircleRadius, pitchCircleRadius)
#        pitchPointAngle = math.atan(pitchInvolutePoint.y / pitchInvolutePoint.x)
#    
#        # Determine the angle defined by the tooth thickness as measured at
#        # the pitch diameter circle.
#        toothThicknessAngle = math.pi / self.parent.getParameter(PARAM_TOOTH_NUMBER).value
#
#        backlash = 0
#        # Determine the angle needed for the specified backlash.
#        backlashAngle = (backlash / (pitchCircleRadius)) * .25
#
#        # Determine the angle to rotate the curve.
#        rotateAngle = -((toothThicknessAngle/2) + pitchPointAngle - backlashAngle)
#        
#        # Rotate the involute so the middle of the tooth lies on the x axis.
#        cosAngle = math.cos(rotateAngle)
#        sinAngle = math.sin(rotateAngle)
#        for i in range(0, len(involutePoints)):
#            newX = involutePoints[i].x * cosAngle - involutePoints[i].y * sinAngle
#            newY = involutePoints[i].x * sinAngle + involutePoints[i].y * cosAngle
#            involutePoints[i] = adsk.core.Point3D.create(newX, newY, 0)
#
#        pointCollection = adsk.core.ObjectCollection.create()
#        for i in (range(0, len(involutePoints))): #spec.involuteSteps)):
#            pointCollection.add(involutePoints[i])
#        lline = sketch.sketchCurves.sketchFittedSplines.add(pointCollection)
#
#        pointCollection = adsk.core.ObjectCollection.create()
#        for i in (range(0, len(involutePoints))): #spec.involuteSteps)):
#            pointCollection.add(adsk.core.Point3D.create(involutePoints[i].x, -involutePoints[i].y, 0))
#        rline = sketch.sketchCurves.sketchFittedSplines.add(pointCollection)
#
#        # Draw the the top of the tooth
#        toothTopPoint = points.add(adsk.core.Point3D.create(tipCircleRadius, 0, 0))
#        constraints.addCoincident(toothTopPoint, self.tipCircle)
#        top = sketch.sketchCurves.sketchArcs.addByThreePoints(
#            rline.endSketchPoint,
#            toothTopPoint.geometry,
#            lline.endSketchPoint
#        )
#        dimensions.addDiameterDimension(
#            top,
#            adsk.core.Point3D.create(toothTopPoint.geometry.x, 0, 0)
#        )
#    
#        # Create a "spine" for the involutes so that they can be fully constrainted
#        # without having to fix them
#        spine = sketch.sketchCurves.sketchLines.addByTwoPoints(anchorPoint, toothTopPoint)
#        spine.isConstruction = True
#        angleDimension = None
#        if angle == 0:
#            constraints.addHorizontal(spine)
#        else:
#            horizontal = sketch.sketchCurves.sketchLines.addByTwoPoints(
#                adsk.core.Point3D.create(anchorPoint.geometry.x, anchorPoint.geometry.y, 0),
#                adsk.core.Point3D.create(toothTopPoint.geometry.x, toothTopPoint.geometry.y, 0),
#            )
#            horizontal.isConsutruction = True
#            constraints.addHorizontal(horizontal)
#            constraints.addCoincident(horizontal.startSketchPoint, anchorPoint)
#            constraints.addCoincident(horizontal.endSketchPoint, self.tipCircle)
#
#            angleDimension = dimensions.addAngularDimension(spine, horizontal,
#                adsk.core.Point3D.create(anchorPoint.geometry.x, anchorPoint.geometry.y, 0))
#
#        # Create a series of lines (ribs) from one involute to the other
#        priv = anchorPoint
#        for i in range(0, len(involutePoints)):
#            # First create a point on the spine where the ribs are going to
#            # be constrained on 
#            lpoint = lline.fitPoints.item(i)
#            rib = curves.sketchLines.addByTwoPoints(
#                lpoint,
#                rline.fitPoints.item(i)
#            )
#            rib.isConstruction = True
#            dimensions.addDistanceDimension(
#                rib.startSketchPoint,
#                rib.endSketchPoint,
#                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
#                adsk.core.Point3D.create(rib.startSketchPoint.geometry.x, rib.startSketchPoint.geometry.y/2, 0)
#            )
#            spinePoint = points.add(adsk.core.Point3D.create(lpoint.geometry.x, 0, 0))
#            constraints.addCoincident(spinePoint, spine)
#            constraints.addMidPoint(spinePoint, rib)
#            constraints.addPerpendicular(spine, rib)
#            dimensions.addDistanceDimension(priv, spinePoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, adsk.core.Point3D.create((lpoint.geometry.x-priv.geometry.x)/2+priv.geometry.x, 0, 0))
#            priv = spinePoint
#
#            # Create the point where the involutes will connect to the root circle
#        def drawRootToInvoluteLine(root, rootRadius, inv, x, y):
#            angle = math.atan(y/ x)
#            point = adsk.core.Point3D.create(
#                rootRadius * math.cos(angle),
#                rootRadius * math.sin(angle),
#                0,
#            )
#
#            line = sketch.sketchCurves.sketchLines.addByTwoPoints(point, inv.startSketchPoint)
#            constraints.addCoincident(line.startSketchPoint, root)
#            constraints.addHorizontal(line)
#            return line
#
##        if rootCircleRadius > baseCircleRadius:
#        if math.sqrt(involutePoints[0].x**2 + involutePoints[0].y**2) > rootCircleRadius:
#            drawRootToInvoluteLine(self.rootCircle, rootCircleRadius, lline, involutePoints[0].x, involutePoints[0].y)
#            drawRootToInvoluteLine(self.rootCircle, rootCircleRadius, rline, involutePoints[0].x, -involutePoints[0].y)
#        else:
#            self.toothProfileIsEmbedded = True
##            drawRootToInvoluteLine(self.baseCircle, baseCircleRadius, lline, involutePoints[i].x, involutePoints[i].y)
##            drawRootToInvoluteLine(self.baseCircle, baseCircleRadius, rline, involutePoints[i].x, -involutePoints[i].y)
#
#        if angle != 0:
#            # Only do this _AFTER_ all the lines have been drawn
#            angleDimension.value = angle
#
#
#    def calculateInvolutePoint(self, baseCircleRadius, intersectionRadius):
#        alpha = math.acos( baseCircleRadius / intersectionRadius)
#        if alpha <= 0:
#            return None
#        invAlpha = math.tan(alpha) - alpha
#        return adsk.core.Point3D.create(
#            intersectionRadius*math.cos(invAlpha),
#            intersectionRadius*math.sin(invAlpha),
#            0)
#
#    def drawBore(self, anchorPoint=None):
#        projectedAnchorPoint = self.sketch.project(anchorPoint).item(0)
#        self.drawCircle('Bore Circle', self.parent.getParameter('BoreDiameter').value/2, projectedAnchorPoint, isConstruction=False)
#
#
#class SpurGearGenerator(Generator):
#    def __init__(self, design: adsk.fusion.Design):
#        super(SpurGearGenerator, self).__init__(design)
#        self.parent = None
#        self.plane = None
#        self.anchorPoint = None
#
#    def newContext(self):
#        return SpurGearGenerationContext()
#
#    def create_specification_from_inputs(self, inputs):
#        return SpurGearSpecification.from_inputs(self.getComponent(), self.getOccurence(), inputs)
#
#    def generateName(self):
#        module = self.getParameter(PARAM_MODULE)
#        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
#        thickness = self.getParameter('Thickness')
#        return 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)
#    
#    def processInputs(self, inputs: adsk.core.CommandInputs):
#        # Note: all angles are in radians
#        (values, ok) = get_selection(inputs, INPUT_ID_PARENT)
#        if len(values) == 1 and ok:
#            v = values[0]
#            if v.objectType == adsk.fusion.Occurrence.classType():
#                self.parentComponent = v.component
#            elif v.objectType == adsk.fusion.Component.classType():
#                self.parentComponent = v
#            else:
#                raise Exception(f'Invalid object type {v.objectType}')
#            futil.log(f'parentComponent is {self.parentComponent.name}')
#        else:
#            raise Exception("Require parameter '{INPUT_ID_PARENT}' not available")
#
#        (values, ok) = get_selection(inputs, INPUT_ID_PLANE)
#        if len(values) == 1 and ok:
#            self.plane = values[0] # must be exactly one item
#        else:
#            raise Exception("Require parameter '{INPUT_ID_PLANE}' not available")
#    
#        (values, ok) = get_selection(inputs, INPUT_ID_ANCHOR_POINT)
#        if len(values) == 1 and ok:
#            self.anchorPoint = values[0] # must be exactly one item
#        else:
#            raise Exception(f"Require parameter '{INPUT_ID_ANCHOR_POINT}' not available (selected {len(values)} points)")
#
#        (module, ok) = get_value(inputs, 'module', '')
#        if not ok:
#            raise Exception('Invalid module value')
#        self.addParameter(PARAM_MODULE, module, '', 'Module for the spur gear')
#
#        (toothNumber, ok) = get_value(inputs, 'toothNumber', '')
#        if ok:
#            self.addParameter(PARAM_TOOTH_NUMBER, toothNumber, '', 'Number of tooth on the spur gera')
#
#        (pressureAngle, ok) = get_value(inputs, 'pressureAngle', 'rad')
#        if ok:
#            self.addParameter('PressureAngle', pressureAngle, 'rad', 'Pressure angle for spur gear')
#
#        (boreDiameter, ok) = get_value(inputs, 'boreDiameter', 'mm')
#        if not ok:
#            boreDiameter = adsk.core.ValueInput.createByReal(0)
#        self.addParameter('BoreDiameter', boreDiameter, 'mm', 'Size of the bore')
#
#        (thickness, ok) = get_value(inputs, 'thickness', 'mm')
#        if ok:
#            self.addParameter('Thickness', thickness, 'mm', 'Thickness of the spur gear')
#        
#        (chamferTooth, ok) = get_value(inputs, 'chamferTooth', 'mm')
#        if ok:
#            self.addParameter('ChamferTooth', chamferTooth, 'mm', 'Chamfer size')
#
#        (sketchOnly, ok) = get_boolean(inputs, 'sketchOnly')
#        if ok:
#            self.addParameter('SketchOnly', adsk.core.ValueInput.createByReal(1 if sketchOnly else 0), '', 'Draw sketch only')
#    
#        self.addParameter('PitchCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("ToothNumber")} * {self.parameterName("Module")}'), 'mm', 'Pitch circle diameter')
#        self.addParameter('PitchCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} / 2'), 'mm', 'Pitch circle radius')
#
#        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
#        self.addParameter('BaseCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} * cos({self.parameterName("PressureAngle")})'), 'mm', 'Base circle diameter')
#        self.addParameter('BaseCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("BaseCircleDiameter")} / 2'), 'mm', 'Base circle radius')
#
#        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
#        self.addParameter('RootCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} - 2.5 * {self.parameterName("Module")}'), 'mm', 'Root circle diameter')
#        self.addParameter('RootCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("RootCircleDiameter")} / 2'), 'mm', 'Root circle radius')
#
#        self.addParameter('TipCircleDiameter', adsk.core.ValueInput.createByString(f'{self.parameterName("PitchCircleDiameter")} + 2 * {self.parameterName("Module")}'), 'mm', 'Tip circle diameter')
#        self.addParameter('TipCircleRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("TipCircleDiameter")} / 2'), 'mm', 'Tip circle radius')
#
#        # currently unused
#        # self.tipClearance = 0 if self.toothNumber < 3 else module / 6
#        # currently unused
#        # self.circularPitch = self.pitchCircleDiameter * math.pi / toothNumber
#        # s = (math.acos(self.baseCircleDiameter/self.pitchCircleDiameter)/16)
#        self.addParameter('InvoluteSteps', adsk.core.ValueInput.createByReal(15), '', 'Number of segments to use when drawing involute')
#
#        self.addParameter('FilletThreshold', adsk.core.ValueInput.createByString(f'{self.parameterName("BaseCircleDiameter")} * PI / ({self.parameterName("ToothNumber")} * 2) * 0.4'), 'mm', 'Maximum possible threshold')
#        # For now, filletRadius = filletThreshold
#        self.addParameter('FilletRadius', adsk.core.ValueInput.createByString(f'{self.parameterName("FilletThreshold")}'), 'mm', '')
#
#    def drawCircle(self, name: str, sketch: adsk.fusion.Sketch, radius: adsk.core.ValueInput, anchorPoint, dimensionAngle=0, isConstruction=True):
#        curves = sketch.sketchCurves
#        dimensions = sketch.sketchDimensions
#        texts = sketch.sketchTexts
#
#        obj = curves.sketchCircles.addByCenterRadius(anchorPoint, radius)
#        obj.isConstruction = isConstruction
#
#        # Draw the diameter dimension at the specified angle
#        x = 0
#        y = 0
#        if dimensionAngle == 0:
#            x = radius.value
#        elif dimensionAngle != 0:
#            x = (radius.value/2)*math.sin(dimensionAngle)
#            y = (radius.value/2)*math.cos(dimensionAngle)
#        dimensions.addDiameterDimension(
#            obj,
#            adsk.core.Point3D.create(x, y, 0)
#        )
#        size = self.parent.getParameter('TipCircleRadius').value - self.parent.getParameter('RootCircleRadius').value
#        input = texts.createInput2('{} (r={:.2f})'.format(name, radius), size)
#        input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
#        texts.add(input)
#        return obj
#
#    def generate(self, inputs: adsk.core.CommandInputs):
#        self.processInputs(inputs)
#
#        component = self.getComponent()
#        component.name = self.generateName()
#
#        # The first thing we want to do is to "fix" the spec so that the plane
#        # is a construction plane.
#        # This plane _MUST_ be a construction plane in order to avoid having to deal with
#        # profile artifacts.
#        if self.plane.objectType != adsk.fusion.ConstructionPlane.classType():
#            # Create a co-planer construction plane
#            cplaneInput = component.constructionPlanes.createInput()
#            cplaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
#            self.plane = component.constructionPlanes.add(cplaneInput)
#
#        ctx = self.newContext()
#
#        # Create tools to draw and otherwise position the gear with.
#        self.prepareTools(ctx)
#
#        # Create the main body of the gear
#        self.buildMainGearBody(ctx)
#
#        self.buildBore(ctx)
#
#
#    def buildBore(self, ctx: GenerationContext):
#        diameter = self.getParameter('BoreDiameter').value
#        if diameter <= 0:
#            return
#        sketch = self.createSketchObject('Bore Profile', self.plane)
#
#        SpurGearInvoluteToothDesignGenerator(sketch,self).drawBore(ctx.anchorPoint)
#
#        extrudes = self.getComponent().features.extrudeFeatures
#        boreProfile = None
#        for profile in sketch.profiles:
#            # There should be a single loop and a single curve
#            if profile.profileLoops.count != 1:
#                continue
#
#            loop = profile.profileLoops.item(0)
#            if loop.profileCurves.count != 1:
#                continue
#            curve = loop.profileCurves.item(0)
#            if curve.geometryType == adsk.core.Curve3DTypes.Circle3DCurveType:
#                if abs(curve.geometry.radius - diameter/2) < 0.001:
#                    boreProfile = profile
#                    break
#
#        if boreProfile is None:
#            raise Exception('could not find bore profile')
#
#        boreExtrudeInput = extrudes.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
#
#        direction = adsk.fusion.ExtentDirections.PositiveExtentDirection
##        if not get_normal(spec.plane).isEqualTo(get_normal(ctx.extrusionExtent)):
##            direction = adsk.fusion.ExtentDirections.NegativeExtentDirection
#
#        boreExtrudeInput.setOneSideExtent(
#            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False),
#            direction,
#        )
#        boreExtrudeInput.participantBodies = [ctx.gearBody]
#        extrudes.add(boreExtrudeInput)
#
#    def prepareTools(self, ctx: GenerationContext):
#        # Create a sketch that contains the anchorPoint and the lines that
#        # define its position
#        sketch = self.createSketchObject('Tools', plane=self.plane)
#
#        # TODO: I think we should standardize on storing these elements in ctx object instead of self
#        ctx.anchorPoint = sketch.project(self.anchorPoint)
##        ctx.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
##        projectedConstructionPoint = sketch.project(self.getComponent().originConstructionPoint).item(0)
##        sketch.geometricConstraints.addCoincident(ctx.anchorPoint, projectedConstructionPoint)
#        sketch.isVisible = False
#
#        # Create plane to perform extrusions against.
#        input = self.getComponent().constructionPlanes.createInput()
#
#        thickness = self.getParameterAsValueInput('Thickness')
#        input.setByOffset(self.plane, thickness)
#        extrusionEndPlane = self.getComponent().constructionPlanes.add(input)
#        # TODO: I can't make this work
#        # extrusionEndPlane.isVisible = False
#        ctx.extrusionEndPlane = extrusionEndPlane
#
#    
#    def buildSketches(self, ctx: GenerationContext):
#        sketch = self.createSketchObject('Gear Profile', plane=self.plane)
#        designgen = SpurGearInvoluteToothDesignGenerator(sketch, self)
#        designgen.draw(ctx.anchorPoint)
#
#        # remember if some specifics of this probile
#        if designgen.toothProfileIsEmbedded:
#            ctx.toothProfileIsEmbedded = True
#        ctx.gearProfileSketch = sketch
#
#    def buildMainGearBody(self, ctx: GenerationContext):
#        ui = adsk.core.Application.get().userInterface
#        ui.isComputeDeferred = True
#        self.buildSketches(ctx)
#        ui.isComputeDeferred = False
#
#        # The user could simply want the involute tooth and the circles.
#        # In that case, pass on building the body
#        if self.getParameterAsBoolean('SketchOnly'):
#            ctx.gearProfileSketch.isVisible = True
#        else:
#            ui.isComputeDeferred = True
#            self.buildTooth(ctx)
#            ui.isComputeDeferred = False
#            ui.isComputeDeferred = True
#            self.buildBody(ctx)
#            ui.isComputeDeferred = False
#            ui.isComputeDeferred = True
#            self.patternTeeth(ctx)
#            ui.isComputeDeferred = False
#
#    def buildTooth(self, ctx: GenerationContext):
#        extrudes = self.getComponent().features.extrudeFeatures
#        profiles = ctx.gearProfileSketch.profiles
#
#        # The tooth profile has a very specific shape. We look for that shape
#        # in the list of profiles that we have.
#
#        toothProfile = None
#        for profile in profiles:
#            for loop in profile.profileLoops:
#                expectArcs = 2
#                expectNurbs = 2
#                expectLines = 0
#                if ctx.toothProfileIsEmbedded:
#                    # The loop must have exactly 4 curves.
#                    if loop.profileCurves.count != 4:
#                        continue
#                else:
#                    # The loop must have exactly 6 curves.
#                    if loop.profileCurves.count != 6:
#                        continue
#                    expectLines = 2
#
#                # The curve must consist of Line3D, NurbsCurve3D and Arc3D
#                arcs = 0
#                nurbs = 0
#                lines = 0
#                for curve in loop.profileCurves:
#                    ctyp = curve.geometry.curveType
#                    if ctyp == adsk.core.Curve3DTypes.Arc3DCurveType:
#                        arcs += 1
#                    elif  ctyp == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
#                        nurbs += 1
#                    elif  ctyp == adsk.core.Curve3DTypes.Line3DCurveType:
#                        lines += 1
#                    else:
#                        break
#                
#                if nurbs == expectNurbs and arcs == expectArcs and lines == expectLines:
#                    toothProfile = profile
#                    break
#            if toothProfile:
#                break
#
#        if not toothProfile:
#            raise Exception("could not find tooth profile")
#
#        toothExtrudeInput = extrudes.createInput(toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
#        toothExtrudeInput.setOneSideExtent(
#            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
#            adsk.fusion.ExtentDirections.PositiveExtentDirection
#        )
#        toothExtrude = extrudes.add(toothExtrudeInput)
#        toothExtrude.name = 'Extrude tooth'
#
#        # note: toothBody must be populated before chamferTooth
#        ctx.toothBody = toothExtrude.bodies.item(0)
#        self.chamferTooth(ctx)
#    
#    def buildBody(self, ctx: GenerationContext):
#        extrudes = self.getComponent().features.extrudeFeatures
#        # distance = self.toothThickness(spec)
#
#        # First create the cylindrical part so we can construct a
#        # perpendicular axis
#        profiles = ctx.gearProfileSketch.profiles
#        gearBodyProfile = None
#        for profile in profiles:
#            for loop in profile.profileLoops:
#                if loop.profileCurves.count != 2:
#                    continue
#                arcs = 0
#                for curve in loop.profileCurves:
#                    if curve.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
#                        arcs += 1
#                    else:
#                        break
#                if arcs == 2:
#                    gearBodyProfile = profile
#                    break
#
#            if gearBodyProfile:
#                break
#
#        if not gearBodyProfile:
#            raise Exception("could not find gear body profile")
#
#        gearBodyExtrudeInput = extrudes.createInput(
#            gearBodyProfile,
#            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
#        )
#        gearBodyExtrudeInput.setOneSideExtent(
#            adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
#            adsk.fusion.ExtentDirections.PositiveExtentDirection
#        )
#        gearBodyExtrude = extrudes.add(gearBodyExtrudeInput)
#        gearBodyExtrude.name = 'Extrude body'
#        gearBodyExtrude.bodies.item(0).name = 'Gear Body'
#
#        circularFace = None
#        for face in gearBodyExtrude.bodies.item(0).faces:
#            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
#                # This face is used to find the axis
#                circularFace = face
#            elif face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
#                # If the plane is parallel but NOT coplanar, it's the
#                # face that was just created
#                sketchPlane = ctx.gearProfileSketch.referencePlane.geometry
#                if sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry):
#                    ctx.extrusionExtent = face
#        
#            if circularFace and ctx.extrusionExtent:
#                break
#
#        if not circularFace:
#            raise Exception("Could not find circular face")
#        if not ctx.extrusionExtent: 
#            raise Exception("Could not find extrusion extent face")
#
#        axisInput = self.getComponent().constructionAxes.createInput()
#        axisInput.setByCircularFace(circularFace)
#        centerAxis = self.getComponent().constructionAxes.add(axisInput)
#        if centerAxis is None:
#            raise Exception("Could not create axis")
#
#        centerAxis.name = 'Gear Center'
#        centerAxis.isVisibile = False
#        ctx.centerAxis = centerAxis
#        # store the gear body for later use
#        ctx.gearBody = self.getComponent().bRepBodies.itemByName('Gear Body')
#
#    def patternTeeth(self, ctx: GenerationContext):
#        circular = self.getComponent().features.circularPatternFeatures
#        toothBody = ctx.toothBody
#
#        bodies = adsk.core.ObjectCollection.create()
#        bodies.add(toothBody)
#
#        patternInput = circular.createInput(bodies, ctx.centerAxis)
#        patternInput.quantity = self.getParameterAsValueInput(PARAM_TOOTH_NUMBER)
#        patternedTeeth = circular.add(patternInput)
#
#        toolBodies = adsk.core.ObjectCollection.create()
#        for body in patternedTeeth.bodies:
#            toolBodies.add(body)
#        combineInput = self.getComponent().features.combineFeatures.createInput(
#            self.getComponent().bRepBodies.itemByName('Gear Body'),
#            toolBodies
#        )
#        self.getComponent().features.combineFeatures.add(combineInput)
#
#        self.createFillets(ctx)
#    
#    def createFillets(self, ctx: GenerationContext):
#        fr = self.getParameter('FilletRadius')
#        if fr.value <= 0:
#            return
#        gearBody = self.getComponent().bRepBodies.itemByName('Gear Body')
#        edges = adsk.core.ObjectCollection.create()
#        rootCircleRadiusCm = self.getParameter('RootCircleRadius').value
#        for face in gearBody.faces:
#            if face.geometry.objectType == adsk.core.Cylinder.classType():
#                if abs(face.geometry.radius - rootCircleRadiusCm) < 0.001:
#                    for edge in face.edges:
#                        dir = edge.endVertex.geometry.vectorTo(edge.startVertex.geometry)
#                        dir.normalize()
#
#                        normal = self.plane.geometry.normal
#
#                        # XXX Hmmm, I thought the edges with dot product = 0
#                        # would be the right ones to fillet, but something isn't
#                        # working as I expected to...
#                        if abs(dir.dotProduct(normal)) > 0.001:
#                            edges.add(edge)
#                            
#        if edges.count > 0:
#            filletInput = self.getComponent().features.filletFeatures.createInput()
#            filletInput.edgeSetInputs.addConstantRadiusEdgeSet(
#                edges,
#                # TODO to_cm
#                self.getParameterAsValueInput('FilletRadius'),
#                False,
#            )
#            self.getComponent().features.filletFeatures.add(filletInput)
#
#
#    def chamferWantEdges(self):
#        return 6
#
#    def chamferTooth(self, ctx: GenerationContext):
#        ct = self.getParameter('ChamferTooth')
#        if ct.value <= 0:
#            return
#        # Note: this does not take into account when the tooth is thin enough
#        # that the chamfer will not be applied. We need to figure out a soft limite
#        # to apply a chamfer, and avoid doing this calculation altogeher if it
#        # doesn't work
#        toothBody = ctx.toothBody
#        splineEdges = adsk.core.ObjectCollection.create()
#
#        wantSurfaceType = adsk.core.SurfaceTypes.PlaneSurfaceType
#
#        wantEdges = self.chamferWantEdges()
#        # The selection of face/edge is ... very hard coded. If there's a better way
#        # (more deterministic) way, we should use that instead
#        for face in toothBody.faces:
#            # Surface must be a plane, not like a cylindrical surface
#            if face.geometry.surfaceType != wantSurfaceType:
#                continue
#            # face must contain exactly 6 edges... if it's a regular spur gear
#            # otherwise if it's a herringbone gear, it's going to be 7
#            if len(face.edges) != wantEdges:
#                continue
#
#            if wantEdges == 4:
#                # Only accept this face if the edges contain exactly two splines
#                splineCount = 0
#                for edge in face.edges:
#                    if edge.geometry.objectType == 'adsk::core::NurbsCurve3D':
#                        splineCount += 1
#
#                
#                if splineCount != 2:
#                    continue
#
#
#            # We don't want to chamfer the Arc that is part of the root circle.
#            rootCircleRadiusCm = self.getParameter('RootCircleRadius').value
#            for edge in face.edges:
#                if edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
#                    if abs(edge.geometry.radius - rootCircleRadiusCm) < 0.001:
#                        continue
#                splineEdges.add(edge)
#
#        chamferInput = self.getComponent().features.chamferFeatures.createInput2()
#        chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(splineEdges, self.getParameterAsValueInput('ChamferTooth'), False)
#        self.getComponent().features.chamferFeatures.add(chamferInput)
#
#
# ==============================================================================
# Functional API - New implementation
# ==============================================================================

def create_spur_gear_spec(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> SpurGearSpec:
    """
    Parse inputs and create a SpurGearSpec dataclass.

    This function extracts all spur gear parameters from the command dialog
    inputs and creates a SpurGearSpec dataclass instance. The SpurGearSpec
    will automatically compute derived parameters (pitch diameter, base circle
    radius, etc.) in its __post_init__ method.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A fully populated SpurGearSpec instance with both input and derived
        parameters

    Raises:
        Exception: If required parameters are invalid or missing
    """
    params = parse_spur_gear_inputs(inputs, design)

    return SpurGearSpec(
        module=params['module'],
        tooth_number=params['tooth_number'],
        pressure_angle=params['pressure_angle'],
        thickness=params['thickness'],
        bore_diameter=params['bore_diameter'],
        chamfer_tooth=params['chamfer_tooth'],
        sketch_only=params['sketch_only']
    )


def make_spur_gear_name(spec: SpurGearSpec) -> str:
    """
    Generate a descriptive name for the gear component.

    Creates a human-readable name for the gear component that includes the
    key parameters. This makes it easier to identify different gears in the
    design tree.

    Args:
        spec: The spur gear specification

    Returns:
        A descriptive string like "Spur Gear (M=1, Teeth=17, T=5)"
    """
    return f'Spur Gear (M={spec.module}, Teeth={spec.tooth_number}, T={spec.thickness})'


def generate_spur_gear(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> GenerationState:
    """
    Main entry point for spur gear generation.

    This function coordinates the entire spur gear generation process. It
    handles input parsing, component creation, parameter setup, and calls
    the appropriate sketch/body generation functions based on whether
    sketch-only mode is enabled.

    The generation process follows these steps:
    1. Parse inputs and create specification
    2. Extract parent component and plane/point selections
    3. Create new component and occurrence
    4. Initialize generation state
    5. Set descriptive component name
    6. Create all user parameters
    7. Normalize plane to construction plane
    8. Prepare tools (anchor point, extrusion plane)
    9. Generate sketches or full body based on sketch_only flag

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design where the gear will be created

    Returns:
        The final GenerationState containing all created components, sketches,
        and bodies

    Raises:
        Exception: If inputs are invalid, required selections are missing, or
                  generation fails
    """
    # 1. Parse inputs and create specification
    spec = create_spur_gear_spec(inputs, design)

    # 2. Extract parent component and selections
    parent_component = get_parent_component(inputs)

    plane_selections, _ = get_selection_input(inputs, INPUT_ID_PLANE)
    if len(plane_selections) != 1:
        raise Exception(f"Must select exactly one plane (got {len(plane_selections)})")
    plane = plane_selections[0]

    point_selections, _ = get_selection_input(inputs, INPUT_ID_ANCHOR_POINT)
    if len(point_selections) != 1:
        raise Exception(f"Must select exactly one anchor point (got {len(point_selections)})")
    anchor_point = point_selections[0]

    # 3. Create component and occurrence
    occurrence, prefix = create_gear_occurrence(parent_component, 'SpurGear')

    # 4. Initialize generation state
    state = GenerationState(
        design=design,
        parent_component=parent_component,
        component=occurrence.component,
        occurrence=occurrence,
        param_prefix=prefix,
    )

    # 5. Set component name
    state.component.name = make_spur_gear_name(spec)

    # 6. Create parameters
    create_spur_gear_parameters(state, spec)

    # 7. Normalize plane to construction plane
    state.plane = ensure_construction_plane(state.component, plane)

    # 8. Prepare tools (anchor point and extrusion plane)
    state = prepare_spur_gear_tools(state, state.plane, anchor_point)

    # 9. Generate geometry based on sketch_only flag
    if spec.sketch_only:
        # For sketch-only mode, just build the sketches and make them visible
        state = build_spur_gear_sketches(state, spec)
        state.sketch.isVisible = True
    else:
        # For full body mode, build the complete 3D gear
        state = build_spur_gear_body(state, spec)

    # Hide all construction planes (they're only needed during generation)
    hide_construction_planes(state)

    return state


def prepare_spur_gear_tools(
    state: GenerationState,
    plane: adsk.fusion.ConstructionPlane,
    anchor_point: Any
) -> GenerationState:
    """
    Create tool sketch with anchor point and extrusion plane.

    This function creates a "Tools" sketch that contains the projected anchor
    point and creates a construction plane for extrusion. The anchor point
    defines the center of the gear, and the extrusion plane defines where
    the gear extrusion will end (based on the thickness parameter).

    Args:
        state: The current generation state
        plane: The construction plane to create the tools sketch on
        anchor_point: The point entity selected by the user (to be projected)

    Returns:
        Updated GenerationState with anchor_point and extrusion_end_plane fields
        populated
    """
    # Create tools sketch on the specified plane
    sketch = create_sketch(state.component, 'Tools', plane)

    # Project the anchor point into this sketch
    projected = sketch.project(anchor_point)
    projected_anchor = projected.item(0) if projected.count > 0 else None

    if projected_anchor is None:
        raise Exception("Failed to project anchor point")

    # Hide the tools sketch
    sketch.isVisible = False

    # Create extrusion end plane offset by thickness
    thickness_param = get_parameter(state.design, state.param_prefix, 'Thickness')
    if thickness_param is None:
        raise Exception("Thickness parameter not found")

    thickness_value = adsk.core.ValueInput.createByReal(thickness_param.value)

    plane_input = state.component.constructionPlanes.createInput()
    plane_input.setByOffset(plane, thickness_value)
    extrusion_end_plane = state.component.constructionPlanes.add(plane_input)
    extrusion_end_plane.name = 'Spur Gear Extrusion End Plane'

    # Return updated state with new fields
    return GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=projected_anchor,
        extrusion_end_plane=extrusion_end_plane,
        sketch=state.sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent
    )


def build_spur_gear_sketches(
    state: GenerationState,
    spec: SpurGearSpec
) -> GenerationState:
    """
    Create the gear profile sketch with involute tooth.

    This function creates the main gear profile sketch including all circles
    (root, base, pitch, tip) and the involute tooth profile. The sketch is
    fully constrained and ready for extrusion.

    Args:
        state: The current generation state with anchor_point populated
        spec: The spur gear specification

    Returns:
        Updated GenerationState with sketch field populated and tooth_profile_is_embedded flag set
    """
    # Create the gear profile sketch
    sketch = create_sketch(state.component, 'Gear Profile', state.plane)

    # Project the anchor point into this sketch first
    # This projected point will be used as the center for all circles and the tooth
    projected_collection = sketch.project(state.anchor_point)
    if projected_collection.count == 0:
        raise Exception("Failed to project anchor point into gear profile sketch")

    # Update state with the projected anchor point for this sketch
    projected_anchor = projected_collection.item(0)
    state_with_projected_anchor = GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=projected_anchor,  # Use projected anchor for drawing
        extrusion_end_plane=state.extrusion_end_plane,
        sketch=state.sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent
    )

    # Draw all circles (root, base, pitch, tip) using the projected anchor
    draw_spur_gear_circles(sketch, state_with_projected_anchor, spec)

    # Draw the involute tooth profile using the projected anchor
    draw_involute_tooth(sketch, state_with_projected_anchor, spec)

    # Return updated state with sketch populated
    # tooth_profile_is_embedded is updated inside draw_involute_tooth
    return GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=state.anchor_point,
        sketch=sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent,
        extrusion_end_plane=state.extrusion_end_plane,
        tooth_profile_is_embedded=state.tooth_profile_is_embedded
    )


def draw_spur_gear_circles(
    sketch: adsk.fusion.Sketch,
    state: GenerationState,
    spec: SpurGearSpec
) -> None:
    """
    Draw root, base, pitch, and tip circles with dimensions and text labels.

    This function creates all four reference circles for the gear profile
    and stores them in state fields for use by draw_involute_tooth().
    Each circle is drawn with diameter dimensions at specific angles to
    avoid overlaps, and includes text annotations with radius and size info.

    Args:
        sketch: The sketch to draw circles in
        state: The current generation state (contains projected anchor_point,
               will be updated with circle fields)
        spec: The spur gear specification (for radius values)

    Returns:
        None (circles are stored in state.root_circle, state.base_circle, etc.)
    """
    curves = sketch.sketchCurves
    dimensions = sketch.sketchDimensions
    texts = sketch.sketchTexts

    # Use the anchor point from state (already projected into this sketch)
    anchor_point = state.anchor_point

    # Helper to draw a single circle with dimension and text
    def draw_circle(name: str, radius: float, dimension_angle: float, is_construction: bool):
        # Create the circle
        circle = curves.sketchCircles.addByCenterRadius(anchor_point, radius)
        circle.isConstruction = is_construction

        # Add diameter dimension at the specified angle
        x = 0
        y = 0
        if dimension_angle == 0:
            x = radius
        else:
            x = (radius / 2) * math.sin(dimension_angle)
            y = (radius / 2) * math.cos(dimension_angle)

        dimensions.addDiameterDimension(
            circle,
            adsk.core.Point3D.create(x, y, 0)
        )

        # Add text annotation along the circle
        size = spec.tip_circle_radius - spec.root_circle_radius
        text_input = texts.createInput2(
            f'{name} (r={radius:.2f}, size={size:.2f})',
            size
        )
        text_input.setAsAlongPath(
            circle,
            True,
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
            0
        )
        texts.add(text_input)

        return circle

    # Draw all four circles at different dimension angles to avoid overlap
    root_circle = draw_circle(
        'Root Circle',
        spec.root_circle_radius,
        math.radians(15),
        False  # Not construction - this is the actual root
    )

    tip_circle = draw_circle(
        'Tip Circle',
        spec.tip_circle_radius,
        math.radians(30),
        True  # Construction line
    )

    base_circle = draw_circle(
        'Base Circle',
        spec.base_circle_radius,
        math.radians(45),
        True  # Construction line
    )

    pitch_circle = draw_circle(
        'Pitch Circle',
        spec.pitch_circle_radius,
        math.radians(60),
        True  # Construction line (reference)
    )

    # Store circle references in state for use by draw_involute_tooth()
    state.root_circle = root_circle
    state.base_circle = base_circle
    state.pitch_circle = pitch_circle
    state.tip_circle = tip_circle


def draw_involute_tooth(
    sketch: adsk.fusion.Sketch,
    state: GenerationState,
    spec: SpurGearSpec,
    angle: float = 0
) -> None:
    """
    Draw a single involute tooth profile with full constraints.

    This is the most complex part of the gear generation. It creates the
    characteristic involute curve shape of gear teeth by calculating involute
    curve points, creating mirrored splines, adding constraints and dimensions.

    The function relies on circles stored in state fields (populated by
    draw_spur_gear_circles()) to position the tooth profile correctly and
    add geometric constraints.

    Args:
        sketch: The sketch to draw in
        state: The current generation state (contains anchor_point and circle fields)
        spec: The spur gear specification
        angle: Optional angle to rotate the tooth (in radians, default 0)

    Returns:
        None (tooth profile is drawn in sketch, tooth_profile_is_embedded flag is updated in state)
    """
    # Validate circles exist
    if state.root_circle is None:
        raise ValueError(
            "Circle fields must be populated by draw_spur_gear_circles() "
            "before calling draw_involute_tooth()"
        )

    # Calculate tooth thickness angle
    tooth_thickness_angle = math.pi / spec.tooth_number

    # Create configuration for shared function
    # NOTE: Spec radii are in cm (Fusion API units), pass directly without conversion
    config = ToothProfileConfig(
        root_radius=spec.root_circle_radius,
        base_radius=spec.base_circle_radius,
        pitch_radius=spec.pitch_circle_radius,
        tip_radius=spec.tip_circle_radius,
        tooth_thickness_angle=tooth_thickness_angle,
        involute_steps=spec.involute_steps,
        backlash=0.0,  # Backlash in cm (matches original default)
        rotation_offset=0.0,  # Not used for spur gears (set to 0 for clarity)
        center_offset=None,  # No translation
        use_dimension_expressions=False,  # Numeric dimensions only
        param_prefix=None,
        tip_circle_param_name=None,
        root_circle_param_name=None,
        add_construction_geometry=True,  # Spine, ribs, constraints
        angle=angle
    )

    # Call shared function
    draw_involute_tooth_profile(sketch, state, config)


def extrude_spur_tooth(
    state: GenerationState,
    spec: SpurGearSpec
) -> GenerationState:
    """
    Extrude the tooth profile to create a single tooth body.

    This function finds the tooth profile in the gear sketch and extrudes it
    to create a single tooth body. The tooth profile is identified by its
    characteristic shape consisting of specific curve types (arcs and nurbs).

    Args:
        state: The current generation state with sketch populated
        spec: The spur gear specification

    Returns:
        Updated GenerationState with tooth_body field populated

    Raises:
        Exception: If the tooth profile cannot be found in the sketch
    """
    extrudes = state.component.features.extrudeFeatures
    profiles = state.sketch.profiles

    # The tooth profile has a very specific shape. We look for that shape
    # in the list of profiles that we have.
    tooth_profile = None
    for profile in profiles:
        for loop in profile.profileLoops:
            expect_arcs = 2
            expect_nurbs = 2
            expect_lines = 0
            if state.tooth_profile_is_embedded:
                # The loop must have exactly 4 curves.
                if loop.profileCurves.count != 4:
                    continue
            else:
                # The loop must have exactly 6 curves.
                if loop.profileCurves.count != 6:
                    continue
                expect_lines = 2

            # The curve must consist of Line3D, NurbsCurve3D and Arc3D
            arcs = 0
            nurbs = 0
            lines = 0
            for curve in loop.profileCurves:
                ctyp = curve.geometry.curveType
                if ctyp == adsk.core.Curve3DTypes.Arc3DCurveType:
                    arcs += 1
                elif ctyp == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                    nurbs += 1
                elif ctyp == adsk.core.Curve3DTypes.Line3DCurveType:
                    lines += 1
                else:
                    break

            if nurbs == expect_nurbs and arcs == expect_arcs and lines == expect_lines:
                tooth_profile = profile
                break
        if tooth_profile:
            break

    if not tooth_profile:
        raise Exception("could not find tooth profile")

    tooth_extrude_input = extrudes.createInput(tooth_profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    tooth_extrude_input.setOneSideExtent(
        adsk.fusion.ToEntityExtentDefinition.create(state.extrusion_end_plane, False),
        adsk.fusion.ExtentDirections.PositiveExtentDirection
    )
    tooth_extrude = extrudes.add(tooth_extrude_input)
    tooth_extrude.name = 'Extrude tooth'

    # Store the tooth body in state
    state.tooth_body = tooth_extrude.bodies.item(0)

    # Apply chamfer to tooth if needed
    chamfer_spur_tooth(state, spec)

    return state


def chamfer_spur_tooth(
    state: GenerationState,
    spec: SpurGearSpec
) -> None:
    """
    Apply chamfer to the tooth edges if chamfer_tooth > 0.

    This function finds the appropriate edges on the tooth body and applies
    a chamfer to them. It looks for planar faces with the correct number of
    edges and excludes edges that are part of the root circle.

    Args:
        state: The current generation state with tooth_body populated
        spec: The spur gear specification
    """
    if spec.chamfer_tooth <= 0:
        return

    tooth_body = state.tooth_body
    spline_edges = adsk.core.ObjectCollection.create()

    want_surface_type = adsk.core.SurfaceTypes.PlaneSurfaceType
    want_edges = 6  # Regular spur gear has 6 edges per face

    # The selection of face/edge is very hard coded. If there's a better way
    # (more deterministic) way, we should use that instead
    for face in tooth_body.faces:
        # Surface must be a plane, not like a cylindrical surface
        if face.geometry.surfaceType != want_surface_type:
            continue
        # Face must contain exactly 6 edges for a regular spur gear
        if len(face.edges) != want_edges:
            continue

        # We don't want to chamfer the Arc that is part of the root circle.
        root_circle_radius_cm = spec.root_circle_radius
        for edge in face.edges:
            if edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(edge.geometry.radius - root_circle_radius_cm) < 0.001:
                    continue
            spline_edges.add(edge)

    if spline_edges.count > 0:
        chamfer_input = state.component.features.chamferFeatures.createInput2()
        chamfer_input.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            spline_edges,
            adsk.core.ValueInput.createByReal(spec.chamfer_tooth),
            False
        )
        state.component.features.chamferFeatures.add(chamfer_input)


def extrude_spur_body(
    state: GenerationState,
    spec: SpurGearSpec
) -> GenerationState:
    """
    Extrude the gear body profile to create the cylindrical gear body.

    This function finds the gear body profile (the annular ring between root
    and tip circles) and extrudes it. It also creates a construction axis
    through the center of the gear for use in patterning.

    Args:
        state: The current generation state with sketch populated
        spec: The spur gear specification

    Returns:
        Updated GenerationState with gear_body, center_axis, and extrusion_extent populated

    Raises:
        Exception: If the gear body profile, circular face, or extrusion extent cannot be found
    """
    extrudes = state.component.features.extrudeFeatures

    # Find the gear body profile (annular ring with 2 arcs)
    profiles = state.sketch.profiles
    gear_body_profile = None
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
                gear_body_profile = profile
                break

        if gear_body_profile:
            break

    if not gear_body_profile:
        raise Exception("could not find gear body profile")

    gear_body_extrude_input = extrudes.createInput(
        gear_body_profile,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    gear_body_extrude_input.setOneSideExtent(
        adsk.fusion.ToEntityExtentDefinition.create(state.extrusion_end_plane, False),
        adsk.fusion.ExtentDirections.PositiveExtentDirection
    )
    gear_body_extrude = extrudes.add(gear_body_extrude_input)
    gear_body_extrude.name = 'Extrude body'
    gear_body_extrude.bodies.item(0).name = 'Gear Body'

    # Find the circular face and extrusion extent face
    circular_face = None
    for face in gear_body_extrude.bodies.item(0).faces:
        if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
            # This face is used to find the axis
            circular_face = face
        elif face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            # If the plane is parallel but NOT coplanar, it's the
            # face that was just created
            sketch_plane = state.sketch.referencePlane.geometry
            if sketch_plane.isParallelToPlane(face.geometry) and not sketch_plane.isCoPlanarTo(face.geometry):
                state.extrusion_extent = face

        if circular_face and state.extrusion_extent:
            break

    if not circular_face:
        raise Exception("Could not find circular face")
    if not state.extrusion_extent:
        raise Exception("Could not find extrusion extent face")

    # Create construction axis through center
    axis_input = state.component.constructionAxes.createInput()
    axis_input.setByCircularFace(circular_face)
    center_axis = state.component.constructionAxes.add(axis_input)
    if center_axis is None:
        raise Exception("Could not create axis")

    center_axis.name = 'Gear Center'
    center_axis.isLightBulbOn = False
    state.center_axis = center_axis

    # Store the gear body for later use
    state.gear_body = state.component.bRepBodies.itemByName('Gear Body')

    return state


def pattern_spur_teeth(
    state: GenerationState,
    spec: SpurGearSpec
) -> None:
    """
    Create a circular pattern of teeth around the gear center.

    This function creates a circular pattern of the single tooth body around
    the center axis, then combines all the patterned teeth with the gear body.

    Args:
        state: The current generation state with tooth_body, center_axis, and gear_body populated
        spec: The spur gear specification
    """
    circular = state.component.features.circularPatternFeatures
    tooth_body = state.tooth_body

    bodies = adsk.core.ObjectCollection.create()
    bodies.add(tooth_body)

    pattern_input = circular.createInput(bodies, state.center_axis)
    pattern_input.quantity = adsk.core.ValueInput.createByReal(spec.tooth_number)
    patterned_teeth = circular.add(pattern_input)

    # Combine all patterned teeth with the gear body
    tool_bodies = adsk.core.ObjectCollection.create()
    for body in patterned_teeth.bodies:
        tool_bodies.add(body)
    combine_input = state.component.features.combineFeatures.createInput(
        state.component.bRepBodies.itemByName('Gear Body'),
        tool_bodies
    )
    state.component.features.combineFeatures.add(combine_input)


def create_spur_fillets(
    state: GenerationState,
    spec: SpurGearSpec
) -> None:
    """
    Create fillets at the root of the teeth if fillet_radius > 0.

    This function finds edges at the root circle where teeth meet the body
    and applies fillets to them. It looks for edges on cylindrical faces
    that match the root circle radius and are oriented perpendicular to
    the gear plane.

    Note: This function works best for spur gears with simple extruded geometry.
    For helical/herringbone gears with lofted surfaces, the fillet operation may
    fail due to geometry complexity. In such cases, the error is caught and logged.

    Args:
        state: The current generation state with gear_body populated
        spec: The spur gear specification
    """
    if spec.fillet_radius <= 0:
        return

    try:
        gear_body = state.component.bRepBodies.itemByName('Gear Body')
        edges = adsk.core.ObjectCollection.create()
        root_circle_radius_cm = spec.root_circle_radius

        for face in gear_body.faces:
            if face.geometry.objectType == adsk.core.Cylinder.classType():
                if abs(face.geometry.radius - root_circle_radius_cm) < 0.001:
                    for edge in face.edges:
                        dir = edge.endVertex.geometry.vectorTo(edge.startVertex.geometry)
                        dir.normalize()

                        normal = state.plane.geometry.normal

                        # XXX Hmmm, I thought the edges with dot product = 0
                        # would be the right ones to fillet, but something isn't
                        # working as I expected to...
                        if abs(dir.dotProduct(normal)) > 0.001:
                            edges.add(edge)

        if edges.count > 0:
            fillet_input = state.component.features.filletFeatures.createInput()
            fillet_input.edgeSetInputs.addConstantRadiusEdgeSet(
                edges,
                adsk.core.ValueInput.createByReal(spec.fillet_radius),
                False,
            )
            state.component.features.filletFeatures.add(fillet_input)
    except:
        # Fillet operation can fail on complex geometry (e.g., lofted helical gears)
        # This is non-critical - fillets are cosmetic, so we log and continue
        import traceback
        # Log to console for debugging
        print('Fillet operation skipped due to geometry complexity (this is normal for helical/herringbone gears)')
        print('Error details:')
        traceback.print_exc()


def create_spur_bore(
    state: GenerationState,
    spec: SpurGearSpec
) -> None:
    """
    Create a bore hole through the center of the gear if bore_diameter > 0.

    This function creates a new sketch on the gear plane, draws a circle for
    the bore, and extrudes it as a cut through the gear body.

    Args:
        state: The current generation state with gear_body and plane populated
        spec: The spur gear specification
    """
    if spec.bore_diameter <= 0:
        return

    # Create bore profile sketch
    sketch = state.component.sketches.add(state.plane)
    sketch.name = 'Bore Profile'

    # Project the anchor point
    projected_anchor = sketch.project(state.anchor_point).item(0)

    # Draw bore circle
    circles = sketch.sketchCurves.sketchCircles
    bore_circle = circles.addByCenterRadius(
        projected_anchor.worldGeometry,
        spec.bore_diameter / 2
    )

    # Find the bore profile
    extrudes = state.component.features.extrudeFeatures
    bore_profile = None
    for profile in sketch.profiles:
        # There should be a single loop and a single curve
        if profile.profileLoops.count != 1:
            continue

        loop = profile.profileLoops.item(0)
        if loop.profileCurves.count != 1:
            continue
        curve = loop.profileCurves.item(0)
        if curve.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
            if abs(curve.geometry.radius - spec.bore_diameter/2) < 0.001:
                bore_profile = profile
                break

    if bore_profile is None:
        raise Exception('could not find bore profile')

    # Create cut extrusion
    bore_extrude_input = extrudes.createInput(bore_profile, adsk.fusion.FeatureOperations.CutFeatureOperation)

    direction = adsk.fusion.ExtentDirections.PositiveExtentDirection

    bore_extrude_input.setOneSideExtent(
        adsk.fusion.ToEntityExtentDefinition.create(state.extrusion_extent, False),
        direction,
    )
    bore_extrude_input.participantBodies = [state.gear_body]
    extrudes.add(bore_extrude_input)


def build_spur_gear_body(
    state: GenerationState,
    spec: SpurGearSpec
) -> GenerationState:
    """
    Build the 3D gear body by extruding tooth, body, and applying finishing operations.

    This function orchestrates the entire 3D body generation process for a spur
    gear. It calls all the necessary functions in the correct order to create
    the complete gear body including tooth extrusion, body extrusion, circular
    patterning, fillets, and bore creation.

    The steps are:
    1. Build sketches (tooth profile and circles)
    2. Extrude tooth profile to create single tooth
    3. Extrude gear body (annular ring)
    4. Pattern tooth around center axis
    5. Apply fillets to root edges (if specified)
    6. Create bore hole (if specified)

    Args:
        state: The current generation state
        spec: The spur gear specification

    Returns:
        Updated GenerationState with gear_body field populated and all features created
    """
    # Build sketches first
    state = build_spur_gear_sketches(state, spec)

    # Extrude tooth
    state = extrude_spur_tooth(state, spec)

    # Extrude body
    state = extrude_spur_body(state, spec)

    # Pattern teeth around center
    pattern_spur_teeth(state, spec)

    # Create fillets if specified
    create_spur_fillets(state, spec)

    # Create bore if specified
    if spec.bore_diameter > 0:
        create_spur_bore(state, spec)

    return state