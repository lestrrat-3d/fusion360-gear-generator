"""
String constants for the gear generator library.

This package provides centralized string constants used throughout the gear
generation code. Constants are organized by category and prefixed for clarity:

Parameters and Inputs (params.py):
- PARAM_*: Parameter names (Module, ToothNumber, etc.)
- INPUT_*: Input field IDs (module, toothNumber, etc.)

Messages (messages.py):
- ERR_*: Error messages
- LABEL_*: UI labels
- TOOLTIP_*: UI tooltips
- PROMPT_*: UI prompts
- VALIDATION_*: Validation messages

Entities (entities.py):
- COMPONENT_*: Component names
- BODY_NAME_*: Body names
- SKETCH_*: Sketch names
- PLANE_*: Construction plane names
- AXIS_*: Construction axis names
- FEATURE_*: Feature names

Units (units.py):
- UNIT_*: Unit strings

Usage:
    from lib.geargen.constants import PARAM_MODULE, INPUT_MODULE, ERR_INVALID_MODULE
"""

# Import and re-export all constants
from .params import *
from .messages import *
from .entities import *
from .units import *

# Define __all__ for controlled exports
__all__ = [
    # params.py - Parameter and Input Field IDs
    "PARAM_MODULE",
    "INPUT_MODULE",
    "PARAM_TOOTH_NUMBER",
    "INPUT_TOOTH_NUMBER",
    "PARAM_MATING_TOOTH_NUMBER",
    "INPUT_MATING_TOOTH_NUMBER",
    "PARAM_PRESSURE_ANGLE",
    "INPUT_PRESSURE_ANGLE",
    "PARAM_HELIX_ANGLE",
    "INPUT_HELIX_ANGLE",
    "PARAM_BORE_DIAMETER",
    "INPUT_BORE_DIAMETER",
    "PARAM_THICKNESS",
    "INPUT_THICKNESS",
    "PARAM_CHAMFER_TOOTH",
    "INPUT_CHAMFER_TOOTH",
    "PARAM_SKETCH_ONLY",
    "INPUT_SKETCH_ONLY",
    "INPUT_ANCHOR_POINT",
    "INPUT_PLANE",
    "INPUT_PARENT_COMPONENT",
    "PARAM_PITCH_CIRCLE_DIAMETER",
    "PARAM_PITCH_CIRCLE_RADIUS",
    "PARAM_BASE_CIRCLE_DIAMETER",
    "PARAM_BASE_CIRCLE_RADIUS",
    "PARAM_ROOT_CIRCLE_DIAMETER",
    "PARAM_ROOT_CIRCLE_RADIUS",
    "PARAM_TIP_CIRCLE_DIAMETER",
    "PARAM_TIP_CIRCLE_RADIUS",
    "PARAM_INVOLUTE_STEPS",
    "PARAM_FILLET_THRESHOLD",
    "PARAM_FILLET_RADIUS",
    "PARAM_GEAR_HEIGHT",
    "PARAM_MATING_GEAR_HEIGHT",
    "PARAM_VIRTUAL_TEETH_NUMBER",
    "PARAM_DRIVING_GEAR_BASE_THICKNESS",
    "PARAM_TEETH_LENGTH",
    "PARAM_SHAFT_ANGLE",
    "PARAM_FACE_WIDTH",
    "INPUT_SHAFT_ANGLE",
    "INPUT_FACE_WIDTH",
    "INPUT_DRIVING_GEAR_BASE_THICKNESS",
    "INPUT_TEETH_LENGTH",

    # messages.py - Error Messages
    "ERR_GENERATION",
    "ERR_GENERATION_FAILED",
    "ERR_INVALID_MODULE",
    "ERR_INVALID_PRESSURE_ANGLE",
    "ERR_INVALID_TOOTH_NUMBER",
    "ERR_INVALID_HELIX_ANGLE",
    "ERR_INVALID_THICKNESS",
    "ERR_MODULE_PARAM_NOT_FOUND",
    "ERR_PARAM_NOT_FOUND",
    "ERR_PARAM_CREATE_FAILED",
    "ERR_ANCHOR_POINT_PROJECTION",
    "ERR_SKETCH_CREATION",
    "ERR_PROFILE_NOT_FOUND",
    "ERR_TOOTH_PROFILE_NOT_FOUND",
    "ERR_GEAR_BODY_PROFILE_NOT_FOUND",
    "ERR_EXTRUDE_FAILED",
    "ERR_LOFT_CREATION",
    "ERR_REVOLVE_FAILED",
    "ERR_JOIN_BODIES",
    "ERR_PATTERN_FAILED",
    "ERR_COMPONENT_CREATION",
    "ERR_PLANE_CREATION",
    "ERR_AXIS_CREATION",
    "ERR_PHASE1_INCOMPLETE",
    "ERR_PHASE2_INCOMPLETE",
    "ERR_FOUNDATION_SKETCH_NOT_FOUND",
    "ERR_MISSING_STATE_ATTR",
    "ERR_INVALID_STATE",

    # messages.py - UI Labels
    "LABEL_MODULE",
    "LABEL_TOOTH_NUMBER",
    "LABEL_PRESSURE_ANGLE",
    "LABEL_HELIX_ANGLE",
    "LABEL_BORE_DIAMETER",
    "LABEL_THICKNESS",
    "LABEL_CHAMFER_TOOTH",
    "LABEL_SKETCH_ONLY",

    # messages.py - Tooltips
    "TOOLTIP_MODULE",
    "TOOLTIP_TOOTH_NUMBER",
    "TOOLTIP_PRESSURE_ANGLE",
    "TOOLTIP_SKETCH_ONLY",
    "TOOLTIP_PLANE",
    "TOOLTIP_ANCHOR_POINT",
    "TOOLTIP_PARENT_COMPONENT",

    # messages.py - Selection Prompts
    "PROMPT_SELECT_PLANE",
    "PROMPT_SELECT_POINT",
    "PROMPT_SELECT_COMPONENT",
    "PROMPT_SELECT_FACE",
    "PROMPT_SELECT_EDGE",

    # messages.py - Validation Messages
    "VALIDATION_POSITIVE",
    "VALIDATION_NON_NEGATIVE",
    "VALIDATION_MIN_ONE",
    "VALIDATION_BETWEEN_0_AND_1",
    "VALIDATION_INTEGER_REQUIRED",
    "VALIDATION_NUMERIC_REQUIRED",
    "VALIDATION_MISSING_STATE",
    "VALIDATION_INVALID_STATE",
    "VALIDATION_NO_SELECTION",
    "VALIDATION_INVALID_SELECTION",

    # entities.py - Component Names
    "COMPONENT_DESIGN",
    "COMPONENT_DRIVING_GEAR",
    "COMPONENT_MATING_GEAR",

    # entities.py - Body Names
    "BODY_NAME_GEAR",
    "BODY_NAME_TOOTH",
    "BODY_NAME_BASE",
    "BODY_NAME_BORE",

    # entities.py - Sketch Names
    "SKETCH_FOUNDATION",
    "SKETCH_TOOTH_PROFILE",
    "SKETCH_GEAR_PROFILE",
    "SKETCH_BORE",
    "SKETCH_TOOLS",

    # entities.py - Construction Plane Names
    "PLANE_FOUNDATION",
    "PLANE_TOOTH_PROFILE",
    "PLANE_BOTTOM",
    "PLANE_TOP",
    "PLANE_GEAR_BASE_COPLANAR",

    # entities.py - Construction Axis Names
    "AXIS_GEAR_CENTER",
    "AXIS_TOOTH",

    # entities.py - Feature Names
    "FEATURE_PATTERN",
    "FEATURE_EXTRUDE_GEAR",
    "FEATURE_EXTRUDE_TOOTH",
    "FEATURE_LOFT_TOOTH",
    "FEATURE_REVOLVE_BORE",

    # units.py - Unit Constants
    "UNIT_MM",
    "UNIT_CM",
    "UNIT_M",
    "UNIT_IN",
    "UNIT_RAD",
    "UNIT_DEG",
    "UNIT_NONE",

    # entities.py - Tooth Profile and Chamfer Constants
    "TOOTH_PROFILE_CURVE_COUNT_EMBEDDED",
    "TOOTH_PROFILE_CURVE_COUNT_STANDARD",
    "HELICAL_GEAR_CHAMFER_EDGE_COUNT",
    "RADIUS_COMPARISON_TOLERANCE_CM",
]
