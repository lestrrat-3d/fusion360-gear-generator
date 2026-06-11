# A fake spur `parent` for SpurGearInvoluteToothDesignGenerator, so a gear
# can borrow the spur tooth drawer without registering Fusion user parameters
# ([PB-PRECOMPUTED-MODE]). It serves, in internal cm, exactly the parameter
# keys the spur drawer reads, and carries the `_lastToothEmbedded` output
# slot the drawer writes during draw() — read it back after drawing.

import math
from .misc import to_cm


class Val:
    def __init__(self, value):
        self.value = value


class VirtualSpurProxy:
    def __init__(self, module_mm, virtualTeeth,
                 pressureAngleRad=math.radians(20.0), involuteSteps=15):
        # OUTPUT slot the spur drawer writes during draw().
        self._lastToothEmbedded = False

        # Standard spur formulas (Module here is raw mm):
        pitch = virtualTeeth * module_mm               # pitch diameter (mm)
        base = pitch * math.cos(pressureAngleRad)      # base diameter (mm)
        root = pitch - 2.5 * module_mm                 # root diameter (mm)
        tip = pitch + 2.0 * module_mm                  # tip diameter (mm)

        # Serve every key the spur drawer reads, in internal cm.
        self._params = {
            'Module': to_cm(module_mm),
            'ToothNumber': virtualTeeth,
            'PressureAngle': pressureAngleRad,
            'PitchCircleDiameter': to_cm(pitch),
            'PitchCircleRadius': to_cm(pitch / 2.0),
            'BaseCircleDiameter': to_cm(base),
            'BaseCircleRadius': to_cm(base / 2.0),
            'RootCircleDiameter': to_cm(root),
            'RootCircleRadius': to_cm(root / 2.0),
            'TipCircleDiameter': to_cm(tip),
            'TipCircleRadius': to_cm(tip / 2.0),
            'InvoluteSteps': involuteSteps,
        }

    def getParameter(self, name):
        return Val(self._params[name])
