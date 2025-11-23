"""
Input processing functions for gear generation.

This module contains functions for extracting and validating user inputs from
the command dialog. All functions follow a (value, is_valid) return pattern
for validation.
"""

from typing import Tuple, List, Any, Dict
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil


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
    module, ok = get_value_input(inputs, 'module', '', design)
    if not ok:
        raise Exception('Invalid module value')
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, 'toothNumber', '', design)
    if not ok:
        raise Exception('Invalid tooth number value')
    params['tooth_number'] = int(tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, 'pressureAngle', 'rad', design)
    if not ok:
        raise Exception('Invalid pressure angle value')
    params['pressure_angle'] = pressure_angle.realValue

    thickness, ok = get_value_input(inputs, 'thickness', 'mm', design)
    if not ok:
        raise Exception('Invalid thickness value')
    params['thickness'] = thickness.realValue

    # Optional parameters with defaults
    bore_diameter, ok = get_value_input(inputs, 'boreDiameter', 'mm', design)
    if not ok:
        bore_diameter_value = 0
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    chamfer_tooth, ok = get_value_input(inputs, 'chamferTooth', 'mm', design)
    if not ok:
        chamfer_tooth_value = 0
    else:
        chamfer_tooth_value = chamfer_tooth.realValue
    params['chamfer_tooth'] = chamfer_tooth_value

    sketch_only, ok = get_boolean_input(inputs, 'sketchOnly')
    if not ok:
        sketch_only = False
    params['sketch_only'] = sketch_only

    return params


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
    module, ok = get_value_input(inputs, 'module', '', design)
    if not ok:
        raise Exception('Invalid module value')
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, 'toothNumber', '', design)
    if not ok:
        raise Exception('Invalid tooth number value')
    params['tooth_number'] = int(tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, 'pressureAngle', 'rad', design)
    if not ok:
        raise Exception('Invalid pressure angle value')
    params['pressure_angle'] = pressure_angle.realValue

    thickness, ok = get_value_input(inputs, 'thickness', 'mm', design)
    if not ok:
        raise Exception('Invalid thickness value')
    params['thickness'] = thickness.realValue

    helix_angle, ok = get_value_input(inputs, 'helixAngle', 'rad', design)
    if not ok:
        raise Exception('Invalid helix angle value')
    params['helix_angle'] = helix_angle.realValue

    # Optional parameters with defaults
    bore_diameter, ok = get_value_input(inputs, 'boreDiameter', 'mm', design)
    if not ok:
        bore_diameter_value = 0
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    chamfer_tooth, ok = get_value_input(inputs, 'chamferTooth', 'mm', design)
    if not ok:
        chamfer_tooth_value = 0
    else:
        chamfer_tooth_value = chamfer_tooth.realValue
    params['chamfer_tooth'] = chamfer_tooth_value

    sketch_only, ok = get_boolean_input(inputs, 'sketchOnly')
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
    module, ok = get_value_input(inputs, 'module', '', design)
    if not ok:
        raise Exception('Invalid module value')
    params['module'] = module.realValue

    tooth_number, ok = get_value_input(inputs, 'toothNumber', '', design)
    if not ok:
        raise Exception('Invalid tooth number value')
    params['tooth_number'] = int(tooth_number.realValue)

    mating_tooth_number, ok = get_value_input(inputs, 'matingToothNumber', '', design)
    if not ok:
        raise Exception('Invalid mating tooth number value')
    params['mating_tooth_number'] = int(mating_tooth_number.realValue)

    pressure_angle, ok = get_value_input(inputs, 'pressureAngle', 'rad', design)
    if not ok:
        raise Exception('Invalid pressure angle value')
    params['pressure_angle'] = pressure_angle.realValue

    shaft_angle, ok = get_value_input(inputs, 'shaftAngle', 'rad', design)
    if not ok:
        raise Exception('Invalid shaft angle value')
    params['shaft_angle'] = shaft_angle.realValue

    # Optional parameters with None defaults
    face_width, ok = get_value_input(inputs, 'faceWidth', 'mm', design)
    if not ok or (face_width and face_width.realValue <= 0):
        face_width_value = None
    else:
        face_width_value = face_width.realValue
    params['face_width'] = face_width_value

    bore_diameter, ok = get_value_input(inputs, 'boreDiameter', 'mm', design)
    if not ok or (bore_diameter and bore_diameter.realValue <= 0):
        bore_diameter_value = None
    else:
        bore_diameter_value = bore_diameter.realValue
    params['bore_diameter'] = bore_diameter_value

    # Driving Gear Base Thickness (required for dual-trapezoid extensions)
    driving_gear_base_thickness, ok = get_value_input(inputs, 'drivingGearBaseThickness', 'mm', design)
    if not ok:
        # Provide sensible default based on module if not specified
        driving_gear_base_thickness_value = params.get('module', 1.0) * 5
    else:
        driving_gear_base_thickness_value = driving_gear_base_thickness.realValue
    params['driving_gear_base_thickness'] = driving_gear_base_thickness_value

    # Teeth Length (distance between diagonal profile lines)
    teeth_length, ok = get_value_input(inputs, 'teethLength', 'mm', design)
    if not ok:
        # Provide sensible default if not specified
        teeth_length_value = 10.0
    else:
        teeth_length_value = teeth_length.realValue
    params['teeth_length'] = teeth_length_value

    sketch_only, ok = get_boolean_input(inputs, 'sketchOnly')
    if not ok:
        sketch_only = False
    params['sketch_only'] = sketch_only

    return params
