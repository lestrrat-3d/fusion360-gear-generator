#Author-
#Description-

import math
import adsk.core, adsk.fusion, adsk.cam, traceback

_cmdName = 'lestrratSpurGearGenerator'
_handlers = []
_inputs = None
_debug = True

def getUI(app=adsk.core.Application.get()):
    ui = app.userInterface
    if not ui:
        raise Exception('No UI object available. Please run this script from within Fusion 360')
    return ui

def getDesign(app=adsk.core.Application.get()):
    des = adsk.fusion.Design.cast(app.activeProduct)
    if not des:
        raise Exception('A Fusion design must be active when invoking this command.')
    return des

def writelog(s, ui=getUI()):
    try:
        if not _debug:
            return
        palettes = ui.palettes
        palette = palettes.itemById("TextCommands")
        palette.forceUpdate = True
        palette.isVisible = True 
        palette.writeText(s)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def run(context):
    try:
        global _cmdName

        ui = getUI()
        cmd = ui.commandDefinitions.itemById(_cmdName)
        if not cmd:
            cmd = ui.commandDefinitions.addButtonDefinition(
                _cmdName,
                'Spur Gear',
                'Generate a spur gear',
                'resources/SpurGear'
            )

        # The commandDefinition only has the onCommandCreated evnet.
        onCommandCreated = SpurGearCommandCreated()
        cmd.commandCreated.add(onCommandCreated)
        addHandler(onCommandCreated)

        if not cmd.execute():
            raise Exception('Command failed to execute')

        adsk.autoTerminate(False)
        writelog('Spur Gear Generator loaded')
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        
        adsk.terminate()

def addHandler(handler):
    global _handlers
    # Keep the handlers referenced beyond their regular lifecycle
    _handlers.append(handler)

class SpurGearCommandCreated(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        writelog('SpurGearCommandCreated')
        ui = None
        try:
            global _inputs

            ui = getUI()
            eventArgs = adsk.core.CommandCreatedEventArgs.cast(args)

            cmd = eventArgs.command
            cmd.isExecutedWhenPreEmpted = False

            # Setup the container for the command inputs
            _inputs = SpurGearCommandInput(cmd)

            onCommandExecuted = SpurGearCommandExecuted()
            cmd.execute.add(onCommandExecuted)
            addHandler(onCommandExecuted)

            onCommandDestroyed = SpurGearCommandDestroyed()
            cmd.destroy.add(onCommandDestroyed)
            addHandler(onCommandDestroyed)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        writelog("END SpurGearCommandCreated")

class SpurGearCommandExecuted(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        ui = None
        rootComponent = None
        try:
            global _inputs
            ui = getUI()
            design = getDesign()
            
            (module, ok) = _inputs.getValue('module')
            if not ok:
                raise Exception('Invalid module value')
            
            args = {}
            (toothNumber, ok) = _inputs.getValue('toothNumber')
            if ok:
                args['toothNumber'] = toothNumber

            spec = SpurGearSpecification(module, **args)
            
            rootComponent = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            g = SpurGearGenerator(adsk.fusion.Component.cast(rootComponent.component))
            g.generate(spec)
        except:
            writelog('Failed to execute command:\n{}'.format(traceback.format_exc()))
            if ui:
                ui.messageBox('Failed to execute command:\n{}'.format(traceback.format_exc()))
                
            # Clear component if there was an error
            if rootComponent:
                rootComponent.deleteMe()

class SpurGearCommandDestroyed(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            adsk.terminate()
        except:
            if _ui:
                _ui.messageBox('Failed to terminate:\n{}'.format(traceback.format_exc()))

def toCm(mm :float) -> float:
    return mm/10

class SpurGearCommandInput:
    def __init__(self, cmd):
        inputs = cmd.commandInputs
        self.container = inputs
        self.inputs = {
            'module': inputs.addValueInput('module', 'Module', '', adsk.core.ValueInput.createByReal(1)),
            'toothNumber': inputs.addValueInput('toothNumber', 'Tooth Number', '', adsk.core.ValueInput.createByReal(17)),
            'pressureAngle': inputs.addValueInput('pressureAngle', 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20))),
            'boreDiameter': inputs.addValueInput('boreDiameter', 'Bore Diameter', 'mm', adsk.core.ValueInput.createByReal(0)),
            'thickness': inputs.addValueInput('thickness', 'Thickness', 'mm', adsk.core.ValueInput.createByReal(toCm(5))),
            'helixAngle': inputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(0))
        }

    def getValue(self, name):
        unitsManager = getDesign().unitsManager
        if not unitsManager.isValidExpression(self.inputs[name].expression, self.inputs[name].unitType):
            return (None, False)
        
        return (unitsManager.evaluateExpression(self.inputs[name].expression, self.inputs[name].unitType), True)

class SpurGearSpecification:
    # The base implementation uses ISO specs. For specs using diamteral pitches,
    # use a different constructor (TODO)
    def __init__(self, module, toothNumber=17, pressureAngle=math.radians(20), boreDiameter=0, thickness=5, helixAngle=0):
        # Note: all angles are in radians
        self.module = module
        writelog('module: {}'.format(module))
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
            if boreDiameter <= 0:
                self.boreDiameter = self.baseCircleDiameter / 4
            if self.boreDiameter > 0 and self.boreDiameter < 2:
                self.boreDiameter = 2

        s = (math.acos(self.baseCircleDiameter/self.pitchCircleDiameter)/16)
        self.involuteSteps = 15
        self.thickness = thickness
        self.helixAngle = helixAngle
            

class SpurGearGenerator:
    def __init__(self, component: adsk.fusion.Component):
        self.component = component

    def generate(self, spec):
        base_sketch = self.component.sketches.add(self.component.xYConstructionPlane)
        self.component.name = 'Spur Gear (M={}, Tooth={}, Thickness={})'.format(spec.module, spec.toothNumber, spec.thickness)
        base_sketch.name = 'Gear Profile'
        self.draw(base_sketch, spec)
        self.build(base_sketch, self.component, spec)

    def draw(self, sketch: adsk.fusion.Sketch, spec: SpurGearSpecification):
        constraints = sketch.geometricConstraints
        curves = sketch.sketchCurves
        dimensions = sketch.sketchDimensions
        points = sketch.sketchPoints
        texts = sketch.sketchTexts

        # The anchor point is where we draw the circle from.
        # At the end of the drawing process, we create a new anchor point and
        # move the entire thing to the new anchor point
        anchorPoint = points.add(adsk.core.Point3D.create(0, 0, 0))

        def drawCircle(name, radius, isConstruction=True):
            obj = curves.sketchCircles.addByCenterRadius(anchorPoint, toCm(radius))
            obj.isConstruction = isConstruction
            dimensions.addDiameterDimension(
                obj,
                adsk.core.Point3D.create(toCm(radius/2), 0, 0)
            )
            input = texts.createInput2('{} (r={:.2f})'.format(name, radius), toCm(2.5))
            input.setAsAlongPath(obj, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
            texts.add(input)
            return obj

        # Root circle
        root = drawCircle('Root Circle', spec.rootCircleRadius, isConstruction=False)
        # Base circle
        base = drawCircle('Base Circle', spec.baseCircleRadius)
        # Pitch circle (reference)
        pitch = drawCircle('Pitch Circle', spec.pitchCircleRadius)
        # Tip circle
        tip = drawCircle('Tip Circle', spec.tipCircleRadius)
    
        # TODO: because the presence (or lack thereof) of the bore changes the
        # profile to extrude, it is easier to work on the bore _AFTER_ the
        # gear has been generated
        #if spec.boreDiameter != None:
        #    # Note: radius is in cm
        #    inner = curves.sketchCircles.addByCenterRadius(
        #        anchorPoint,
        #        toCm(spec.boreDiameter)/2
        #    )
        #    dimensions.addDiameterDimension(
        #        inner,
        #        adsk.core.Point3D.create(toCm(spec.boreDiameter)/4, toCm(spec.boreDiameter)/4, 0)   ,
        #        True
        #    )

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
        toothTopPoint = points.add(adsk.core.Point3D.create(toCm(spec.tipCircleRadius), 0, 0))
        constraints.addCoincident(toothTopPoint, tip)
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
        constraints.addHorizontal(spine)
        #constraints.addCoincident(spine.startSketchPoint, anchorPoint)

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
        def drawRootToInvoluteLine(x, y, inv):
            angle = math.atan(y/ x)
            point = adsk.core.Point3D.create(
                toCm(spec.rootCircleRadius) * math.cos(angle),
                toCm(spec.rootCircleRadius) * math.sin(angle),
                0,
            )
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(point, inv.startSketchPoint)
            constraints.addTangent(inv, line)
            constraints.addCoincident(line.startSketchPoint, root)
            dimensions.addAngularDimension(line, spine, line.endSketchPoint.geometry)
            return line

        rlline = drawRootToInvoluteLine(involutePoints[0].x, involutePoints[0].y, lline)
        rrline = drawRootToInvoluteLine(involutePoints[0].x, -involutePoints[0].y, rline)

        # Now we have all the sketches necessary.
        # Create an anchor point to base all of the calculations from by
        # creating a pair of vertical and horizontal construction lines
        # that can be dimensioned to the user's liking later.
        # This allows for far easier manipulation of the gear position.

        projectedConstructionPoint = sketch.project(self.component.originConstructionPoint).item(0)

        anchorHorizontalLine = curves.sketchLines.addByTwoPoints(
            projectedConstructionPoint,
            adsk.core.Point3D.create(10, 0, 0)
        )
        anchorHorizontalLine.isConstruction = True
        constraints.addHorizontal(anchorHorizontalLine)
        dimensions.addDistanceDimension(
            anchorHorizontalLine.startSketchPoint,
            anchorHorizontalLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(anchorHorizontalLine.endSketchPoint.geometry.x/2, 0, 0)
        )
        anchorVerticalLine = curves.sketchLines.addByTwoPoints(
            anchorHorizontalLine.endSketchPoint,
            adsk.core.Point3D.create(
                anchorHorizontalLine.endSketchPoint.geometry.x,
                -10,
                0
            )
        )
        anchorVerticalLine.isConstruction = True
        constraints.addVertical(anchorVerticalLine)
        dimensions.addDistanceDimension(
            anchorVerticalLine.startSketchPoint,
            anchorVerticalLine.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(anchorHorizontalLine.endSketchPoint.geometry.x, anchorHorizontalLine.endSketchPoint.geometry.y/2, 0)
        )
        constraints.addCoincident(anchorVerticalLine.endSketchPoint, anchorPoint)

    def calculateInvolutePoint(self, baseCircleRadius, intersectionRadius):
        alpha = math.acos( baseCircleRadius / intersectionRadius)
        if alpha <= 0:
            return None
        invAlpha = math.tan(alpha) - alpha
        return adsk.core.Point3D.create(
            toCm(intersectionRadius*math.cos(invAlpha)),
            toCm(intersectionRadius*math.sin(invAlpha)),
            0)

    # radius is the base circle radius, and distance is the distance from
    # the center of the circle to the involute point
    def calculateInvolutePointOld(self, radius, distance):
        # Note: this is taken from the AutoDesk Fusion360 sample
        # Calculate the other side of the right-angle triangle defined by the base circle and the current distance radius.
        # This is also the length of the involute chord as it comes off of the base circle.
        triangleSide = math.sqrt(math.pow(distance,2) - math.pow(radius,2)) 
        
        # Calculate the angle of the involute.
        alpha = triangleSide / distance

        # Calculate the angle where the current involute point is.
        theta = alpha - math.acos(radius / distance)

        # Calculate the coordinates of the involute point.    
        x = distance * math.cos(theta)
        y = distance * math.sin(theta)

        # Create a point to return.        
        return adsk.core.Point3D.create(toCm(x), toCm(y), 0)
    
    def build(self, sketch: adsk.fusion.Sketch, component: adsk.fusion.Component, spec :SpurGearSpecification):
        extrudes = component.features.extrudeFeatures
        circular = component.features.circularPatternFeatures
        profiles = sketch.profiles
        # Note: the order of sketch creation is extremely important as we can
        # only specify which profiles to extrude by guessing which one is the
        # one we want by the order in the list in sketch.profile

        distance = adsk.core.ValueInput.createByReal(toCm(spec.thickness))

        # First create the cylindrical part so we can construct a
        # perpendicular axis
        gearBodyProfile = profiles.item(1)
        gearBodyExtrude = extrudes.addSimple(gearBodyProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        gearBodyExtrude.bodies.item(0).name = 'Gear Body'

        circularFace = None
        for face in gearBodyExtrude.bodies.item(0).faces:
            if face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                circularFace = face
                break
        
        if circularFace is None:
            raise("could not find circular face")
        
        axisInput = self.component.constructionAxes.createInput()
        axisInput.setByCircularFace(circularFace)
        centerAxis = self.component.constructionAxes.add(axisInput)
        if centerAxis is None:
            raise("Could not create axis")
        centerAxis.name = 'Gear Center'

        toothProfile = profiles.item(0)
        toothExtrude = extrudes.addSimple(toothProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)    
        toothBody = toothExtrude.bodies.item(0)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(toothBody)

        patternInput = circular.createInput(bodies, centerAxis)
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
        centerAxis.isVisibile = False
