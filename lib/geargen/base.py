from abc import ABC, abstractmethod
from .misc import *
from ...lib import fusion360utils as futil
import adsk.core, adsk.fusion

def get_boolean(inputs: adsk.core.CommandInputs, name):
    input = inputs.itemById(name)
    return (input.value, True)

def get_selection(inputs: adsk.core.CommandInputs, name: str):
    input = inputs.itemById(name)
    futil.log(f'get_selection: item {name} has {input.selectionCount} selections')
    list = []
    for i in range(0, input.selectionCount):
        selection = input.selection(i)
        list.append(selection.entity)
    return (list, True)

def get_value(inputs: adsk.core.CommandInputs, name: str, units: str):
    input = inputs.itemById(name)
    design = get_design()
    unitsManager = design.unitsManager
    userParameters = design.userParameters

    # If it's a string, we attempt to convert 
    if input.classType == adsk.core.StringValueCommandInput.classType:
        value = input.value
        if userParameters.itemByName(value) == None:
            evaluated = unitsManager.evaluateExpression(value, units)
            if evaluated == None:
                raise(f'Failed to evaluate expression "{value}"')
            return (evaluated, False)
        return (adsk.core.ValueInput.createByString(value), True)

    if units == None:
        units = input.unitType
    if not unitsManager.isValidExpression(input.expression, units):
        return (None, False)

    evaluated = unitsManager.evaluateExpression(input.expression, units)
    return (adsk.core.ValueInput.createByReal(evaluated), True)


# Base object for generation contexts
class GenerationContext: pass

class Specification: pass

class ParamNamePrefix:
    def __init__(self, prefix):
        self.prefix = prefix
    
    def name(self, s: str) -> str:
        return f'{self.prefix}_{s}'

class ComponentCleaner:
    def __init__(self, prefix: str, occurrence: adsk.fusion.Occurrence):
        self.prefix = prefix
        self.occurrence = occurrence
    
    def deleteAll(self):
        userParameters = get_design().userParameters
        toDelete = {}
        for param in userParameters:
            if not param.name.startswith(self.prefix):
                continue

            toDelete[param.name] = param
        
        toDeleteSize = len(toDelete)
        while toDeleteSize > 0:
            for name in list(toDelete):
                param = toDelete[name]
                if param.isDeletable:
                    param.deleteMe()
                    del toDelete[name]

            currentDeleteSize = len(toDelete)
            if toDeleteSize == currentDeleteSize: # nothing was deleted
                break
            toDeleteSize = currentDeleteSize
        
        self.occurrence.deleteMe()



# Base object for generators
class Generator(ABC):
    def __init__(self, design: adsk.fusion.Design):
        self.design = design
        self.parentComponent = None # TODO: nothing is protecting this from being used when value is None
        self.component = adsk.fusion.Component.cast(None)
        self.occurrence = adsk.fusion.Component.cast(None)
        self.prefix = None
        self.cleaner = None
    
    def getOccurrence(self) -> adsk.fusion.Occurrence:
        if self.occurrence == None:
#            self.occurrence = self.design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            futil.log(f'parent is {self.parentComponent.name}')
            futil.log(f'root component is {self.design.rootComponent.name}')
            self.occurrence = self.parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            prefixStr = f'SpurGear_{self.occurrence.component.id.replace("-", "")}'
            self.prefix = ParamNamePrefix(prefixStr)
            self.cleaner = ComponentCleaner(prefixStr, self.occurrence)
        return self.occurrence
    
    def getComponent(self) -> adsk.fusion.Component:
        return self.getOccurrence().component
    
    def addParameter(self, name: str, value: adsk.core.ValueInput, units: str, comment: str):
        self.design.userParameters.add(self.parameterName(name), value, units, comment)

    def getParameter(self, name: str) -> adsk.fusion.UserParameter:
        return self.design.userParameters.itemByName(self.parameterName(name))
    
    def getParameterAsValueInput(self, name: str) -> adsk.core.ValueInput:
        param = self.getParameter(name)
        if param == None:
            return None
        return adsk.core.ValueInput.createByReal(param.value)
    
    # Used when we are using a user parameter (a double) as a boolean
    # 0 is false, any other value is true
    def getParameterAsBoolean(self, name: str) -> bool:
        param = self.getParameter(name)
        if param == None:
            return False
        return param.value != 0
    
    def parameterName(self, name) -> str:
        if self.prefix == None:
            self.getOccurrence()
        return self.prefix.name(name)

    def deleteComponent(self):
        if self.occurrence:
            self.occurrence.deleteMe()
            self.component = None

    @abstractmethod
    def generate(self, spec: Specification):
        pass

    def createSketchObject(self, name, plane=None):
        if plane is None:
            plane = self.getComponent().xYConstructionPlane

        sketch = self.getComponent().sketches.add(plane)
        sketch.name = name
        sketch.isVisible = False
        return sketch
    