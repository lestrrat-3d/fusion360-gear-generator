from abc import ABC, abstractmethod
import adsk.fusion

# Base object for generation contexts
class GenerationContext: pass

class Specification: pass

# Base object for generators
class Generator(ABC):
    def __init__(self, component: adsk.fusion.Component):
        self.component = component

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
    