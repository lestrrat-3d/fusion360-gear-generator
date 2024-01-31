from abc import ABC, abstractmethod
import adsk.core, adsk.fusion

# Base object for generation contexts
class GenerationContext: pass

class Specification: pass

# Base object for generators
class Generator(ABC):
    def __init__(self, design: adsk.fusion.Design):
        occurence = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.component = occurence.component
        self.occurence = occurence

    def deleteComponent(self):
        if self.occurence:
            self.occurence.deleteMe()
            self.component = None

    @abstractmethod
    def generate(self, spec: Specification):
        pass

    def createSketchObject(self, name, plane=None):
        if plane is None:
            plane = self.component.xYConstructionPlane

        sketch = self.component.sketches.add(plane)
        sketch.name = name
        sketch.isVisible = False
        return sketch
    