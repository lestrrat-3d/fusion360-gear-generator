"""
Input processing functions for gear generation.

This module contains functions for extracting and validating user inputs from
the command dialog. All functions follow a (value, is_valid) return pattern
for validation.
"""

from typing import Tuple, List, Any, Dict
import math
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .constants import (
    INPUT_MODULE, INPUT_TOOTH_NUMBER, INPUT_PRESSURE_ANGLE, INPUT_THICKNESS,
    INPUT_BORE_DIAMETER, INPUT_CHAMFER_TOOTH, INPUT_SKETCH_ONLY,
    INPUT_HELIX_ANGLE, INPUT_MATING_TOOTH_NUMBER,
    INPUT_SHAFT_ANGLE, INPUT_FACE_WIDTH, INPUT_DRIVING_GEAR_BASE_THICKNESS, INPUT_TEETH_LENGTH,
    INPUT_PARENT_COMPONENT, INPUT_PLANE, INPUT_ANCHOR_POINT,
    UNIT_NONE, UNIT_MM, UNIT_RAD,
    ERR_INVALID_MODULE, ERR_INVALID_TOOTH_NUMBER, ERR_INVALID_PRESSURE_ANGLE,
    ERR_INVALID_THICKNESS, ERR_INVALID_HELIX_ANGLE,
    PROMPT_SELECT_COMPONENT, PROMPT_SELECT_PLANE, PROMPT_SELECT_POINT,
    TOOLTIP_PARENT_COMPONENT, TOOLTIP_PLANE, TOOLTIP_ANCHOR_POINT,
    DEFAULT_MODULE_MM, DEFAULT_TOOTH_NUMBER, DEFAULT_PRESSURE_ANGLE_DEG,
    DEFAULT_BORE_DIAMETER_STR, DEFAULT_THICKNESS_MM, DEFAULT_CHAMFER_TOOTH_MM,
    DEFAULT_SKETCH_ONLY, DEFAULT_MATING_TOOTH_NUMBER, DEFAULT_SHAFT_ANGLE_DEG,
    DEFAULT_DRIVING_GEAR_BASE_THICKNESS_MM, DEFAULT_TEETH_LENGTH_MM
)


def get_selection_input(
    inputs: adsk.core.CommandInputs,
    name: str
) -> Tuple[List[Any], bool]:
    """
    Extract selection input items from a command input.

    Retrieves all selected entities from a selection input control. This is
    used for inputs like plane selection, point selection, or component
    selection where users can select one or more Fusion 360 entities.

    Args:
        inputs: The CommandInputs collection from the command dialog
        name: The ID of the selection input to retrieve

    Returns:
        A tuple containing (list_of_entities, is_valid) where list_of_entities
        is a list of the selected Fusion 360 entities and is_valid is always
        True (selection inputs are always valid if they exist)
    """
    input_item = inputs.itemById(name)
    futil.log(f'get_selection_input: item {name} has {input_item.selectionCount} selections')

    items = []
    for i in range(input_item.selectionCount):
        selection = input_item.selection(i)
        items.append(selection.entity)

    return items, True


def get_value_input(
    inputs: adsk.core.CommandInputs,
    name: str,
    units: str,
    design: adsk.fusion.Design
) -> Tuple[adsk.core.ValueInput, bool]:
    """
    Extract and evaluate value input from a command input.

    Processes numeric or string inputs from the command dialog. Handles both
    direct numeric values and string expressions (including parameter
    references). If the input is a string expression, it's evaluated to a
    numeric value unless it's a reference to an existing parameter.

    Args:
        inputs: The CommandInputs collection from the command dialog
        name: The ID of the value input to retrieve
        units: The expected units for the value (e.g., 'mm', 'deg', '' for unitless)
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A tuple containing (value_input, is_valid) where value_input is a
        ValueInput object representing either a real value or a string
        expression, and is_valid indicates whether the input was successfully
        processed. Returns (None, False) if the input is invalid.
    """
    input_item = inputs.itemById(name)
    units_manager = design.unitsManager
    user_parameters = design.userParameters

    # Handle string inputs (expressions or parameter references)
    if input_item.classType == adsk.core.StringValueCommandInput.classType:
        value = input_item.value
        # Check if this is a reference to an existing parameter
        if user_parameters.itemByName(value) is None:
            # Not a parameter reference, evaluate as expression
            evaluated = units_manager.evaluateExpression(value, units)
            if evaluated is None:
                futil.log(f'Failed to evaluate expression "{value}"')
                return None, False
            return adsk.core.ValueInput.createByReal(evaluated), False
        # It's a parameter reference, keep it as a string
        return adsk.core.ValueInput.createByString(value), True

    # Handle numeric inputs
    if units is None or units == '':
        units = input_item.unitType

    if not units_manager.isValidExpression(input_item.expression, units):
        futil.log(f'Invalid expression "{input_item.expression}" for units "{units}"')
        return None, False

    evaluated = units_manager.evaluateExpression(input_item.expression, units)

    return adsk.core.ValueInput.createByReal(evaluated), True


def get_boolean_input(
    inputs: adsk.core.CommandInputs,
    name: str
) -> Tuple[bool, bool]:
    """
    Extract boolean input from a command input.

    Retrieves the boolean value from a checkbox or boolean input control.

    Args:
        inputs: The CommandInputs collection from the command dialog
        name: The ID of the boolean input to retrieve

    Returns:
        A tuple containing (value, is_valid) where value is the boolean state
        and is_valid is always True (boolean inputs are always valid)
    """
    input_item = inputs.itemById(name)
    return input_item.value, True


def parse_spur_gear_inputs(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> Dict[str, Any]:
    """
    Parse UI inputs for spur gear into a dictionary of parameters.

    Extracts and validates all input values from the spur gear command dialog.
    The returned dictionary contains all the parameters needed to create a
    SpurGearSpec dataclass.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A dictionary with keys matching SpurGearSpec field names and values
        containing the parsed input data. The dictionary can be unpacked to
        create a SpurGearSpec instance.

    Raises:
        Exception: If required parameters are invalid or missing
    """
    params = {}

    # Required parameters
    module, ok = get_value_input(inputs, INPUT_MODULE, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_MODULE)
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, INPUT_TOOTH_NUMBER, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_TOOTH_NUMBER)
    params['tooth_number'] = int(tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, INPUT_PRESSURE_ANGLE, UNIT_RAD, design)
    if not ok:
        raise Exception(ERR_INVALID_PRESSURE_ANGLE)
    params['pressure_angle'] = pressure_angle.realValue

    thickness, ok = get_value_input(inputs, INPUT_THICKNESS, UNIT_MM, design)
    if not ok:
        raise Exception(ERR_INVALID_THICKNESS)

    params['thickness'] = thickness.realValue

    # Optional parameters with defaults
    bore_diameter, ok = get_value_input(inputs, INPUT_BORE_DIAMETER, UNIT_MM, design)
    if not ok:
        bore_diameter_value = 0
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    chamfer_tooth, ok = get_value_input(inputs, INPUT_CHAMFER_TOOTH, UNIT_MM, design)
    if not ok:
        chamfer_tooth_value = 0
    else:
        chamfer_tooth_value = chamfer_tooth.realValue
    params['chamfer_tooth'] = chamfer_tooth_value

    sketch_only, ok = get_boolean_input(inputs, INPUT_SKETCH_ONLY)
    if not ok:
        sketch_only = False
    params['sketch_only'] = sketch_only

    return params



def configure_helical_gear_inputs(cmd: adsk.core.Command) -> adsk.core.CommandInputs:
    """
    Configure the command dialog inputs for helical gear generation.

    Args:
        cmd: The Fusion 360 command object

    Returns:
        The CommandInputs collection with all required inputs added
    """
    inputs = cmd.commandInputs

    # Add parent component selection
    inputs.addSelectionInput(INPUT_PARENT_COMPONENT, 'Parent Component', 'Select the parent component')
    # Add plane selection
    inputs.addSelectionInput(INPUT_PLANE, 'Plane', 'Select the plane for gear placement')
    # Add anchor point selection
    inputs.addSelectionInput(INPUT_ANCHOR_POINT, 'Anchor Point', 'Select the anchor point for gear')

    # Add gear parameters
    inputs.addValueInput(INPUT_MODULE, 'Module', 'mm', adsk.core.ValueInput.createByReal(to_cm(DEFAULT_MODULE_MM)))
    inputs.addValueInput(INPUT_TOOTH_NUMBER, 'Tooth Number', UNIT_NONE, adsk.core.ValueInput.createByReal(DEFAULT_TOOTH_NUMBER))
    inputs.addValueInput(INPUT_PRESSURE_ANGLE, 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(DEFAULT_PRESSURE_ANGLE_DEG)))
    inputs.addValueInput(INPUT_THICKNESS, 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(DEFAULT_THICKNESS_MM)))
    inputs.addValueInput(INPUT_BORE_DIAMETER, 'Bore Diameter', UNIT_MM, adsk.core.ValueInput.createByString(DEFAULT_BORE_DIAMETER_STR))
    inputs.addValueInput(INPUT_CHAMFER_TOOTH, 'Chamfer Tooth', 'mm', adsk.core.ValueInput.createByReal(to_cm(DEFAULT_CHAMFER_TOOTH_MM)))
    inputs.addValueInput(INPUT_HELIX_ANGLE, 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))
    inputs.addBoolValueInput(INPUT_SKETCH_ONLY, 'Sketch Only', True, '', DEFAULT_SKETCH_ONLY)

    return inputs


def parse_helical_gear_inputs(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> Dict[str, Any]:
    """
    Parse UI inputs for helical gear into a dictionary of parameters.

    Extracts and validates all input values from the helical gear command dialog.
    The returned dictionary contains all the parameters needed to create a
    HelicalGearSpec dataclass. Helical gears extend spur gears with the addition
    of a helix angle parameter.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A dictionary with keys matching HelicalGearSpec field names and values
        containing the parsed input data. The dictionary can be unpacked to
        create a HelicalGearSpec instance.

    Raises:
        Exception: If required parameters are invalid or missing
    """
    params = {}

    # Required parameters
    module, ok = get_value_input(inputs, INPUT_MODULE, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_MODULE)
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, INPUT_TOOTH_NUMBER, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_TOOTH_NUMBER)
    params['tooth_number'] = int(tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, INPUT_PRESSURE_ANGLE, UNIT_RAD, design)
    if not ok:
        raise Exception(ERR_INVALID_PRESSURE_ANGLE)
    params['pressure_angle'] = pressure_angle.realValue

    thickness, ok = get_value_input(inputs, INPUT_THICKNESS, UNIT_MM, design)
    if not ok:
        raise Exception(ERR_INVALID_THICKNESS)

    params['thickness'] = thickness.realValue

    helix_angle, ok = get_value_input(inputs, INPUT_HELIX_ANGLE, UNIT_RAD, design)
    if not ok:
        raise Exception(ERR_INVALID_HELIX_ANGLE)
    params['helix_angle'] = helix_angle.realValue

    # Optional parameters with defaults
    bore_diameter, ok = get_value_input(inputs, INPUT_BORE_DIAMETER, UNIT_MM, design)
    if not ok:
        bore_diameter_value = 0
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    chamfer_tooth, ok = get_value_input(inputs, INPUT_CHAMFER_TOOTH, UNIT_MM, design)
    if not ok:
        chamfer_tooth_value = 0
    else:
        chamfer_tooth_value = chamfer_tooth.realValue
    params['chamfer_tooth'] = chamfer_tooth_value

    sketch_only, ok = get_boolean_input(inputs, INPUT_SKETCH_ONLY)
    if not ok:
        sketch_only = False
    params['sketch_only'] = sketch_only

    return params


def parse_herringbone_gear_inputs(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> Dict[str, Any]:
    """
    Parse UI inputs for herringbone gear into a dictionary of parameters.

    Extracts and validates all input values from the herringbone gear command
    dialog. Since herringbone gears use the same parameters as helical gears
    (they're just mirrored), this function is identical to
    parse_helical_gear_inputs.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A dictionary with keys matching HelicalGearSpec field names and values
        containing the parsed input data. The dictionary can be unpacked to
        create a HelicalGearSpec instance.

    Raises:
        Exception: If required parameters are invalid or missing
    """
    return parse_helical_gear_inputs(inputs, design)


def parse_bevel_gear_inputs(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> Dict[str, Any]:
    """
    Parse UI inputs for bevel gear into a dictionary of parameters.

    Extracts and validates all input values from the bevel gear command dialog.
    Uses helper functions (get_value_input, get_boolean_input) for consistent
    input extraction and error handling.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A dictionary with keys matching BevelGearSpec field names and values
        containing the parsed input data:
        - module: Base module value (float, mm)
        - tooth_number: Number of teeth on main gear (int)
        - mating_tooth_number: Number of teeth on mating gear (int)
        - pressure_angle: Pressure angle (float, radians)
        - shaft_angle: Angle between gear shafts (float, radians)
        - face_width: Optional face width (float or None, mm)
        - bore_diameter: Optional bore diameter (float or None, mm)
        - driving_gear_base_thickness: Base thickness for driving gear (float, mm)
        - teeth_length: Distance between diagonal profile lines (float, mm)
        - sketch_only: Boolean flag for sketch-only generation

    Raises:
        Exception: If required parameters are invalid or missing
    """
    params = {}

    # Required parameters
    module, ok = get_value_input(inputs, INPUT_MODULE, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_MODULE)
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, INPUT_TOOTH_NUMBER, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_TOOTH_NUMBER)
    params['tooth_number'] = int(tooth_number.realValue)

    mating_tooth_number, ok = get_value_input(inputs, INPUT_MATING_TOOTH_NUMBER, UNIT_NONE, design)
    if not ok:
        raise Exception(ERR_INVALID_TOOTH_NUMBER)
    params['mating_tooth_number'] = int(mating_tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, INPUT_PRESSURE_ANGLE, UNIT_RAD, design)
    if not ok:
        raise Exception(ERR_INVALID_PRESSURE_ANGLE)
    params['pressure_angle'] = pressure_angle.realValue

    shaft_angle, ok = get_value_input(inputs, INPUT_SHAFT_ANGLE, UNIT_RAD, design)
    if not ok:
        raise Exception(ERR_INVALID_PRESSURE_ANGLE)
    params['shaft_angle'] = shaft_angle.realValue

    # Optional parameters with None defaults
    face_width, ok = get_value_input(inputs, INPUT_FACE_WIDTH, UNIT_MM, design)
    if not ok or (face_width and face_width.realValue <= 0):
        face_width_value = None
    else:
        face_width_value = face_width.realValue
    params['face_width'] = face_width_value

    bore_diameter, ok = get_value_input(inputs, INPUT_BORE_DIAMETER, UNIT_MM, design)
    if not ok or (bore_diameter and bore_diameter.realValue <= 0):
        bore_diameter_value = None
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    # Driving Gear Base Thickness (required for dual-trapezoid extensions)
    driving_gear_base_thickness, ok = get_value_input(inputs, INPUT_DRIVING_GEAR_BASE_THICKNESS, UNIT_MM, design)
    if not ok:
        # Provide sensible default based on module if not specified
        driving_gear_base_thickness_value = params.get('module', 1.0) * 5
    else:
        driving_gear_base_thickness_value = driving_gear_base_thickness.realValue
    params['driving_gear_base_thickness'] = driving_gear_base_thickness_value

    # Teeth Length (distance between diagonal profile lines)
    teeth_length, ok = get_value_input(inputs, INPUT_TEETH_LENGTH, UNIT_MM, design)
    if not ok:
        # Provide sensible default if not specified
        teeth_length_value = 10.0
    else:
        teeth_length_value = teeth_length.realValue
    params['teeth_length'] = teeth_length_value

    sketch_only, ok = get_boolean_input(inputs, INPUT_SKETCH_ONLY)
    if not ok:
        sketch_only = False
    params['sketch_only'] = sketch_only

    return params


def add_parent_component_input(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> adsk.core.SelectionCommandInput:
    """
    Create parent component selection input.

    Creates a selection input that allows the user to select a parent component
    (either an Occurrence or RootComponent) to attach the gear to. The root
    component is pre-selected by default.

    Args:
        inputs: The CommandInputs collection to add the input to
        design: The Fusion 360 design (needed to get root component)

    Returns:
        The created SelectionCommandInput
    """
    component_input = inputs.addSelectionInput(
        INPUT_PARENT_COMPONENT,
        PROMPT_SELECT_COMPONENT,
        TOOLTIP_PARENT_COMPONENT
    )
    component_input.addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)
    component_input.addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)
    component_input.setSelectionLimits(1)
    component_input.addSelection(design.rootComponent)
    return component_input


def add_plane_input(
    inputs: adsk.core.CommandInputs
) -> adsk.core.SelectionCommandInput:
    """
    Create plane selection input.

    Creates a selection input that allows the user to select a plane to position
    the gear on. Accepts both construction planes and planar faces.

    Args:
        inputs: The CommandInputs collection to add the input to

    Returns:
        The created SelectionCommandInput
    """
    plane_input = inputs.addSelectionInput(
        INPUT_PLANE,
        PROMPT_SELECT_PLANE,
        TOOLTIP_PLANE
    )
    plane_input.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)
    plane_input.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)
    plane_input.setSelectionLimits(1)
    return plane_input


def add_anchor_point_input(
    inputs: adsk.core.CommandInputs
) -> adsk.core.SelectionCommandInput:
    """
    Create anchor point selection input.

    Creates a selection input that allows the user to select a point to anchor
    the gear center. Accepts both construction points and sketch points.

    Args:
        inputs: The CommandInputs collection to add the input to

    Returns:
        The created SelectionCommandInput
    """
    point_input = inputs.addSelectionInput(
        INPUT_ANCHOR_POINT,
        PROMPT_SELECT_POINT,
        TOOLTIP_ANCHOR_POINT
    )
    point_input.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)
    point_input.addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)
    point_input.setSelectionLimits(1)
    return point_input


def configure_spur_gear_inputs(cmd: adsk.core.Command) -> adsk.core.CommandInputs:
    """
    Configure command inputs for spur gear generation.

    Creates and configures all UI input controls needed for spur gear generation.
    This includes selection inputs for parent component, plane, and anchor point,
    plus value inputs for all gear parameters (module, tooth number, pressure
    angle, bore diameter, thickness, chamfer, and sketch-only mode).

    The inputs are configured with the following defaults:
    - Parent component: Root component (pre-selected)
    - Plane: No default (user must select)
    - Anchor point: No default (user must select)
    - Module: 1.0 mm (displayed as "1 mm", stored as 0.1 cm internally)
    - Tooth number: 17
    - Pressure angle: 20 degrees (displayed in degrees, stored as radians)
    - Bore diameter: "0 mm" (string input)
    - Thickness: 10.0 mm (displayed as "10 mm", stored as 1.0 cm internally)
    - Chamfer tooth: 0.0 mm (no chamfer by default)
    - Sketch only: False (generate full 3D body by default)

    Args:
        cmd: The Command object that will own these inputs

    Returns:
        The CommandInputs collection with all spur gear inputs configured
    """
    inputs = cmd.commandInputs
    design = get_design()

    # Common selection inputs
    add_parent_component_input(inputs, design)
    add_plane_input(inputs)
    add_anchor_point_input(inputs)

    # Spur gear specific inputs
    module_input = inputs.addValueInput(
        INPUT_MODULE,
        'Module',
        'mm',
        adsk.core.ValueInput.createByReal(to_cm(DEFAULT_MODULE_MM))
    )
    module_input.isFullWidth = False

    inputs.addValueInput(
        INPUT_TOOTH_NUMBER,
        'Tooth Number',
        '',
        adsk.core.ValueInput.createByReal(DEFAULT_TOOTH_NUMBER)
    )

    inputs.addValueInput(
        INPUT_PRESSURE_ANGLE,
        'Pressure Angle',
        'deg',
        adsk.core.ValueInput.createByReal(math.radians(DEFAULT_PRESSURE_ANGLE_DEG))
    )

    inputs.addStringValueInput(
        INPUT_BORE_DIAMETER,
        'Bore Diameter',
        DEFAULT_BORE_DIAMETER_STR
    )

    inputs.addValueInput(
        INPUT_THICKNESS,
        'Thickness',
        'mm',
        adsk.core.ValueInput.createByReal(to_cm(DEFAULT_THICKNESS_MM))
    )

    inputs.addValueInput(
        INPUT_CHAMFER_TOOTH,
        'Apply chamfer to teeth',
        'mm',
        adsk.core.ValueInput.createByReal(to_cm(DEFAULT_CHAMFER_TOOTH_MM))
    )

    inputs.addBoolValueInput(
        INPUT_SKETCH_ONLY,
        'Generate sketches, but do not build body',
        True,
        '',
        DEFAULT_SKETCH_ONLY
    )

    return inputs


def configure_bevel_gear_inputs(cmd: adsk.core.Command) -> adsk.core.CommandInputs:
    """
    Configure command inputs for bevel gear generation.

    Creates and configures all UI input controls needed for bevel gear generation.
    This includes selection inputs for parent component, plane, and anchor point,
    plus value inputs for all bevel gear parameters (module, tooth numbers, angles,
    dimensions, and sketch-only mode).

    The inputs are configured with the following defaults:
    - Parent component: Root component (pre-selected)
    - Plane: No default (user must select)
    - Anchor point: No default (user must select)
    - Module: 1.0 (unitless, displayed as "1")
    - Tooth number: 17
    - Mating tooth number: 17
    - Pressure angle: 20 degrees
    - Shaft angle: 90 degrees
    - Face width: "0 mm" (string input, optional for Phase 2+)
    - Bore diameter: "0 mm" (string input, optional for Phase 2+)
    - Driving gear base thickness: 5.0 mm (displayed as "5 mm", stored as 0.5 cm)
    - Teeth length: 10.0 mm (displayed as "10 mm", stored as 1.0 cm)
    - Sketch only: False (generate full 3D body by default)

    Args:
        cmd: The Command object that will own these inputs

    Returns:
        The CommandInputs collection with all bevel gear inputs configured
    """
    inputs = cmd.commandInputs
    design = get_design()

    # Common selection inputs (reuse helpers)
    add_parent_component_input(inputs, design)
    add_plane_input(inputs)
    add_anchor_point_input(inputs)

    # Module (unitless)
    module_input = inputs.addValueInput(
        INPUT_MODULE,
        'Module',
        '',
        adsk.core.ValueInput.createByReal(DEFAULT_MODULE_MM)
    )
    module_input.isFullWidth = False

    # Tooth numbers
    inputs.addValueInput(
        INPUT_TOOTH_NUMBER,
        'Tooth Number',
        '',
        adsk.core.ValueInput.createByReal(DEFAULT_TOOTH_NUMBER)
    )

    inputs.addValueInput(
        INPUT_MATING_TOOTH_NUMBER,
        'Mating Tooth Number',
        '',
        adsk.core.ValueInput.createByReal(DEFAULT_MATING_TOOTH_NUMBER)
    )

    # Angles (in degrees, converted to radians internally)
    inputs.addValueInput(
        INPUT_PRESSURE_ANGLE,
        'Pressure Angle',
        'deg',
        adsk.core.ValueInput.createByReal(math.radians(DEFAULT_PRESSURE_ANGLE_DEG))
    )

    inputs.addValueInput(
        INPUT_SHAFT_ANGLE,
        'Shaft Angle',
        'deg',
        adsk.core.ValueInput.createByReal(math.radians(DEFAULT_SHAFT_ANGLE_DEG))
    )

    # Optional string inputs for Phase 2+
    inputs.addStringValueInput(
        INPUT_FACE_WIDTH,
        'Face Width',
        DEFAULT_BORE_DIAMETER_STR
    )

    inputs.addStringValueInput(
        INPUT_BORE_DIAMETER,
        'Bore Diameter',
        DEFAULT_BORE_DIAMETER_STR
    )

    # Dimensions (in mm, converted to cm for Fusion 360 API)
    inputs.addValueInput(
        INPUT_DRIVING_GEAR_BASE_THICKNESS,
        'Driving Gear Base Thickness',
        'mm',
        adsk.core.ValueInput.createByReal(to_cm(DEFAULT_DRIVING_GEAR_BASE_THICKNESS_MM))
    )

    inputs.addValueInput(
        INPUT_TEETH_LENGTH,
        'Teeth Length',
        'mm',
        adsk.core.ValueInput.createByReal(to_cm(DEFAULT_TEETH_LENGTH_MM))
    )

    # Boolean input
    inputs.addBoolValueInput(
        INPUT_SKETCH_ONLY,
        'Generate sketches, but do not build body',
        True,
        '',
        DEFAULT_SKETCH_ONLY
    )

    return inputs
