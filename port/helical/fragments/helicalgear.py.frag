import math

import adsk.core
import adsk.fusion

from .base import get_value
from . import decadfusion as _fx
from . import spurgear as _spur
from .spurgear import (
    PARAM_MODULE,
    PARAM_TOOTH_NUMBER,
    PARAM_THICKNESS,
    PARAM_FILLET_RADIUS,
    SpurGearCommandInputsConfigurator,
    SpurGearGenerationContext,
    SpurGearGenerator,
)


# @@constants@@

class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        super().configure(cmd)
        inputs: adsk.core.CommandInputs = cmd.commandInputs
        # @@inputs@@


class HelicalGearGenerationContext(SpurGearGenerationContext):
    pass


class HelicalGearGenerator(SpurGearGenerator):
    def newContext(self) -> HelicalGearGenerationContext:
        return HelicalGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HelicalGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)

    def addExtraPrimaryParameters(self, inputs):
        self.addParameter(
            PARAM_HELIX_ANGLE, get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad'),
            'rad', 'Helix angle for the helical gear')

    def validateGallery(self):
        """The gallery's helical build checks the helix angle, then runs the spur's checks."""
        validate_helix_angle(self.getParameter(PARAM_HELIX_ANGLE).value)
        super().validateGallery()

    def registerDerivedParameters(self):
        super().registerDerivedParameters()
        # @@derived_parameters@@

    def helixPlaneOffsetExpression(self) -> str:
        # @@plane_offset@@
        return expression

    def galleryParameters(self):
        return {
            **super().galleryParameters(),
            'helix_angle': PARAM_HELIX_ANGLE,
            'helix_plane_offset': PARAM_HELIX_PLANE_OFFSET,
            'helical_fillet_radius': PARAM_HELICAL_FILLET_RADIUS,
        }

    def variant(self):
        """The helical build: the spur build with a lofted tooth and a root fillet scaled by the helix angle."""
        return _spur._Variant(Tooth=loft_tooth, RootFillet=_fillet_root_edges)


# @@translated@@
