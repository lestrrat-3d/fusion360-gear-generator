from abc import ABC, abstractmethod
from .misc import get_design
from ...lib import fusion360utils as futil
import adsk.core, adsk.fusion

def get_boolean(inputs: adsk.core.CommandInputs, name: str) -> bool:
    return inputs.itemById(name).value

def get_selection(inputs: adsk.core.CommandInputs, name: str):
    input = inputs.itemById(name)
    futil.log(f'get_selection: item {name} has {input.selectionCount} selections')
    entities = []
    for i in range(0, input.selectionCount):
        entities.append(input.selection(i).entity)
    return entities

def get_value(inputs: adsk.core.CommandInputs, name: str, units: str) -> adsk.core.ValueInput:
    """Read a dialog input and return a ValueInput ready for
    userParameters.add(). Raises on an invalid expression — never returns
    None or a raw number ([PB-GET-VALUE-CONTRACT]).

    - StringValueCommandInput whose text names an existing user parameter ->
      ValueInput.createByString(text) (a live reference to that parameter).
    - StringValueCommandInput holding a literal expression (e.g. '5 mm') ->
      ValueInput.createByReal(evaluated value, internal units).
    - Numeric ValueCommandInput -> ValueInput.createByReal(evaluated value).
    """
    input = inputs.itemById(name)
    design = get_design()
    unitsManager = design.unitsManager

    if input.classType == adsk.core.StringValueCommandInput.classType:
        text = input.value
        if design.userParameters.itemByName(text) is not None:
            return adsk.core.ValueInput.createByString(text)
        if not unitsManager.isValidExpression(text, units):
            raise Exception(f'Invalid expression "{text}" for input "{name}"')
        return adsk.core.ValueInput.createByReal(
            unitsManager.evaluateExpression(text, units))

    if units is None:
        units = input.unitType
    if not unitsManager.isValidExpression(input.expression, units):
        raise Exception(
            f'Invalid expression "{input.expression}" for input "{name}"')
    return adsk.core.ValueInput.createByReal(
        unitsManager.evaluateExpression(input.expression, units))


# Base object for generation contexts
class GenerationContext: pass

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
        self.occurrence = adsk.fusion.Occurrence.cast(None)
        self.prefix = None
        self.cleaner = None
    
    def getOccurrence(self) -> adsk.fusion.Occurrence:
        if self.occurrence == None:
#            self.occurrence = self.design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            futil.log(f'parent is {self.parentComponent.name}')
            futil.log(f'root component is {self.design.rootComponent.name}')
            self.occurrence = self.parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            prefixStr = f'{self.prefixBase()}_{self.occurrence.component.id.replace("-", "")}'
            self.prefix = ParamNamePrefix(prefixStr)
            self.cleaner = ComponentCleaner(prefixStr, self.occurrence)
        return self.occurrence

    def prefixBase(self) -> str:
        return 'Gear'
    
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
        if self.cleaner:
            # Deletes the registered user parameters AND the occurrence.
            self.cleaner.deleteAll()
        elif self.occurrence:
            self.occurrence.deleteMe()
        self.occurrence = None
        self.cleaner = None
        self.prefix = None

    @abstractmethod
    def generate(self, inputs: adsk.core.CommandInputs):
        pass

    def createSketchObject(self, name, plane=None):
        if plane is None:
            plane = self.getComponent().xYConstructionPlane

        sketch = self.getComponent().sketches.add(plane)
        sketch.name = name
        sketch.isVisible = False
        return sketch
    