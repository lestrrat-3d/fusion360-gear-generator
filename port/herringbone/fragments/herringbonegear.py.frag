import adsk.core
import adsk.fusion
from . import decadfusion as _fx
from . import helicalgear as _helical
from . import spurgear as _spur
from .spurgear import PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS
from .helicalgear import (
    PARAM_HELIX_ANGLE,
    HelicalGearCommandConfigurator,
    HelicalGearGenerationContext,
    HelicalGearGenerator,
)


class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator):
    pass


class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    pass


class HerringboneGearGenerator(HelicalGearGenerator):
    def newContext(self) -> HerringboneGearGenerationContext:
        return HerringboneGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HerringboneGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)

    def helixPlaneOffsetExpression(self) -> str:
        # @@plane_offset@@
        return expression

    def variant(self):
        """The herringbone build: the helical build with a mirrored tooth and the twisted section at mid-height."""
        return _spur._Variant(Tooth=_chevron, RootFillet=_helical._fillet_root_edges)


# @@translated@@
