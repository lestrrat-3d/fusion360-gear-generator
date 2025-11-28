"""
Parameter management functions for gear generation.

This module contains functions for creating user parameters in Fusion 360
for different gear types. Parameters are created with proper prefixes and
can reference other parameters using expressions.
"""

from typing import Optional
import math
import adsk.core
import adsk.fusion

from .types import GenerationState, SpurGearSpec, BevelGearSpec, HelicalGearSpec
from .core import add_parameter, make_param_name, get_parameter
from .param_expression import ParamExpression
from .constants import (
    PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_PRESSURE_ANGLE, PARAM_THICKNESS,
    PARAM_BORE_DIAMETER, PARAM_CHAMFER_TOOTH, PARAM_SKETCH_ONLY,
    PARAM_HELIX_ANGLE, PARAM_MATING_TOOTH_NUMBER,
    PARAM_PITCH_CIRCLE_DIAMETER, PARAM_PITCH_CIRCLE_RADIUS,
    PARAM_BASE_CIRCLE_DIAMETER, PARAM_BASE_CIRCLE_RADIUS,
    PARAM_ROOT_CIRCLE_DIAMETER, PARAM_ROOT_CIRCLE_RADIUS,
    PARAM_TIP_CIRCLE_DIAMETER, PARAM_TIP_CIRCLE_RADIUS,
    PARAM_INVOLUTE_STEPS, PARAM_FILLET_THRESHOLD, PARAM_FILLET_RADIUS,
    PARAM_GEAR_HEIGHT, PARAM_MATING_GEAR_HEIGHT, PARAM_VIRTUAL_TEETH_NUMBER,
    PARAM_DRIVING_GEAR_BASE_THICKNESS, PARAM_TEETH_LENGTH,
    UNIT_NONE, UNIT_MM, UNIT_RAD
)


def create_spur_gear_parameters(state: GenerationState, spec: SpurGearSpec) -> None:
    """
    Create all user parameters for a spur gear.

    This function creates both basic input parameters (module, tooth number,
    pressure angle, etc.) and derived parameters that use expressions to
    reference other parameters. The derived parameters maintain parametric
    relationships, so changing a basic parameter will automatically update
    all dependent values.

    Parameters are created with a unique prefix to avoid conflicts when
    multiple gears exist in the same design.

    Args:
        state: The generation state containing design and parameter prefix
        spec: The spur gear specification with all parameter values

    Returns:
        None (modifies design by adding parameters as side effect)
    """
    prefix = state.param_prefix
    design = state.design
    expr = ParamExpression(prefix)

    # Basic input parameters
    add_parameter(
        design, prefix, PARAM_MODULE,
        adsk.core.ValueInput.createByReal(spec.module),
        UNIT_NONE,
        'Module for the spur gear'
    )

    add_parameter(
        design, prefix, PARAM_TOOTH_NUMBER,
        adsk.core.ValueInput.createByReal(spec.tooth_number),
        UNIT_NONE,
        'Number of teeth on the spur gear'
    )

    add_parameter(
        design, prefix, PARAM_PRESSURE_ANGLE,
        adsk.core.ValueInput.createByReal(spec.pressure_angle),
        UNIT_RAD,
        'Pressure angle for spur gear'
    )

    add_parameter(
        design, prefix, PARAM_BORE_DIAMETER,
        adsk.core.ValueInput.createByReal(spec.bore_diameter),
        UNIT_MM,
        'Size of the bore'
    )

    # DIAGNOSTIC: Check value before creating parameter (SPUR)
    from ...lib import fusion360utils as futil
    futil.log(f"[DIAG_SPUR_PARAM] Creating Thickness parameter with spec.thickness = {spec.thickness}, units = 'mm'")

    add_parameter(
        design, prefix, PARAM_THICKNESS,
        adsk.core.ValueInput.createByReal(spec.thickness),
        UNIT_MM,
        'Thickness of the spur gear'
    )

    # DIAGNOSTIC: Read back the parameter value (SPUR)
    thickness_param_spur = get_parameter(design, prefix, PARAM_THICKNESS)
    if thickness_param_spur:
        futil.log(f"[DIAG_SPUR_PARAM] Parameter created. thickness_param.value = {thickness_param_spur.value} (Fusion internal units: cm)")
        futil.log(f"[DIAG_SPUR_PARAM] Parameter expression: {thickness_param_spur.expression}")

    add_parameter(
        design, prefix, PARAM_CHAMFER_TOOTH,
        adsk.core.ValueInput.createByReal(spec.chamfer_tooth),
        UNIT_MM,
        'Chamfer size'
    )

    add_parameter(
        design, prefix, PARAM_SKETCH_ONLY,
        adsk.core.ValueInput.createByReal(1 if spec.sketch_only else 0),
        UNIT_NONE,
        'Draw sketch only'
    )

    # Derived parameters using expressions
    # Pitch circle calculations
    add_parameter(
        design, prefix, 'PitchCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "ToothNumber")} * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Pitch circle diameter'
    )

    add_parameter(
        design, prefix, 'PitchCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} / 2'
        ),
        'mm',
        'Pitch circle radius'
    )

    # Base circle calculations
    # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
    add_parameter(
        design, prefix, 'BaseCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} * cos({make_param_name(prefix, "PressureAngle")})'
        ),
        'mm',
        'Base circle diameter'
    )

    add_parameter(
        design, prefix, 'BaseCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "BaseCircleDiameter")} / 2'
        ),
        'mm',
        'Base circle radius'
    )

    # Root circle calculations
    # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
    add_parameter(
        design, prefix, 'RootCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} - 2.5 * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Root circle diameter'
    )

    add_parameter(
        design, prefix, 'RootCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "RootCircleDiameter")} / 2'
        ),
        'mm',
        'Root circle radius'
    )

    # Tip circle calculations
    add_parameter(
        design, prefix, 'TipCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} + 2 * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Tip circle diameter'
    )

    add_parameter(
        design, prefix, 'TipCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "TipCircleDiameter")} / 2'
        ),
        'mm',
        'Tip circle radius'
    )

    # Involute calculation parameters
    add_parameter(
        design, prefix, 'InvoluteSteps',
        adsk.core.ValueInput.createByReal(15),
        '',
        'Number of segments to use when drawing involute'
    )

    # Fillet parameters
    add_parameter(
        design, prefix, 'FilletThreshold',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "BaseCircleDiameter")} * PI / ({make_param_name(prefix, "ToothNumber")} * 2) * 0.4'
        ),
        'mm',
        'Maximum possible threshold'
    )

    add_parameter(
        design, prefix, 'FilletRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "FilletThreshold")}'
        ),
        'mm',
        ''
    )


def create_helical_gear_parameters(state: GenerationState, spec: HelicalGearSpec) -> None:
    """
    Create all user parameters for a helical gear.

    This function creates both basic input parameters (module, tooth number,
    pressure angle, helix angle, etc.) and derived parameters that use
    expressions to reference other parameters. Helical gears have the same
    derived parameters as spur gears, with the addition of the helix angle.

    Parameters are created with a unique prefix to avoid conflicts when
    multiple gears exist in the same design.

    Args:
        state: The generation state containing design and parameter prefix
        spec: The helical gear specification with all parameter values

    Returns:
        None (modifies design by adding parameters as side effect)
    """
    prefix = state.param_prefix
    design = state.design

    # Basic input parameters
    add_parameter(
        design, prefix, 'Module',
        adsk.core.ValueInput.createByReal(spec.module),
        '',
        'Module for the helical gear'
    )

    add_parameter(
        design, prefix, 'ToothNumber',
        adsk.core.ValueInput.createByReal(spec.tooth_number),
        '',
        'Number of teeth on the helical gear'
    )

    add_parameter(
        design, prefix, 'PressureAngle',
        adsk.core.ValueInput.createByReal(spec.pressure_angle),
        'rad',
        'Pressure angle for helical gear'
    )

    add_parameter(
        design, prefix, 'HelixAngle',
        adsk.core.ValueInput.createByReal(spec.helix_angle),
        'rad',
        'Helix angle for helical gear'
    )

    add_parameter(
        design, prefix, 'BoreDiameter',
        adsk.core.ValueInput.createByReal(spec.bore_diameter),
        'mm',
        'Size of the bore'
    )

    # DIAGNOSTIC: Check value before creating parameter
    from ...lib import fusion360utils as futil
    futil.log(f"[DIAG_PARAM] Creating Thickness parameter with spec.thickness = {spec.thickness}, units = 'mm'")

    add_parameter(
        design, prefix, 'Thickness',
        adsk.core.ValueInput.createByReal(spec.thickness),
        'mm',
        'Thickness of the helical gear'
    )

    # DIAGNOSTIC: Read back the parameter value
    thickness_param = get_parameter(design, prefix, 'Thickness')
    if thickness_param:
        futil.log(f"[DIAG_PARAM] Parameter created. thickness_param.value = {thickness_param.value} (Fusion internal units: cm)")
        futil.log(f"[DIAG_PARAM] Parameter expression: {thickness_param.expression}")

    add_parameter(
        design, prefix, 'ChamferTooth',
        adsk.core.ValueInput.createByReal(spec.chamfer_tooth),
        'mm',
        'Chamfer size'
    )

    add_parameter(
        design, prefix, 'SketchOnly',
        adsk.core.ValueInput.createByReal(1 if spec.sketch_only else 0),
        '',
        'Draw sketch only'
    )

    # Derived parameters using expressions (same as spur gear)
    # Pitch circle calculations
    add_parameter(
        design, prefix, 'PitchCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "ToothNumber")} * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Pitch circle diameter'
    )

    add_parameter(
        design, prefix, 'PitchCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} / 2'
        ),
        'mm',
        'Pitch circle radius'
    )

    # Base circle calculations
    # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
    add_parameter(
        design, prefix, 'BaseCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} * cos({make_param_name(prefix, "PressureAngle")})'
        ),
        'mm',
        'Base circle diameter'
    )

    add_parameter(
        design, prefix, 'BaseCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "BaseCircleDiameter")} / 2'
        ),
        'mm',
        'Base circle radius'
    )

    # Root circle calculations
    # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
    add_parameter(
        design, prefix, 'RootCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} - 2.5 * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Root circle diameter'
    )

    add_parameter(
        design, prefix, 'RootCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "RootCircleDiameter")} / 2'
        ),
        'mm',
        'Root circle radius'
    )

    # Tip circle calculations
    add_parameter(
        design, prefix, 'TipCircleDiameter',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "PitchCircleDiameter")} + 2 * {make_param_name(prefix, "Module")}'
        ),
        'mm',
        'Tip circle diameter'
    )

    add_parameter(
        design, prefix, 'TipCircleRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "TipCircleDiameter")} / 2'
        ),
        'mm',
        'Tip circle radius'
    )

    # Involute calculation parameters
    add_parameter(
        design, prefix, 'InvoluteSteps',
        adsk.core.ValueInput.createByReal(15),
        '',
        'Number of segments to use when drawing involute'
    )

    # Fillet parameters
    add_parameter(
        design, prefix, 'FilletThreshold',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "BaseCircleDiameter")} * PI / ({make_param_name(prefix, "ToothNumber")} * 2) * 0.4'
        ),
        'mm',
        'Maximum possible threshold'
    )

    add_parameter(
        design, prefix, 'FilletRadius',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "FilletThreshold")}'
        ),
        'mm',
        ''
    )


def create_herringbone_gear_parameters(state: GenerationState, spec: 'HerringboneGearSpec') -> None:
    """
    Create all user parameters for a herringbone gear.

    This function creates both basic input parameters (module, tooth number,
    pressure angle, helix angle, etc.) and derived parameters that use
    expressions to reference other parameters. Since herringbone gears use
    the same parameters as helical gears (they're just mirrored), this
    function is identical to create_helical_gear_parameters.

    Parameters are created with a unique prefix to avoid conflicts when
    multiple gears exist in the same design.

    Args:
        state: The generation state containing design and parameter prefix
        spec: The herringbone gear specification with all parameter values

    Returns:
        None (modifies design by adding parameters as side effect)
    """
    return create_helical_gear_parameters(state, spec)


def create_bevel_gear_parameters(state: GenerationState, spec: BevelGearSpec) -> None:
    """
    Create user parameters for Phase 1 bevel gear foundation sketch with dual-trapezoid extensions.

    This function creates only the parameters needed for Phase 1 (foundation
    sketch with dual-trapezoid extensions). Phase 1 creates the component hierarchy
    and foundation sketch showing the pitch cone cross-section as a rectangle with
    dual-trapezoid extensions on both sides.

    Phase 1 parameters (8 parameters total):
    - Module, ToothNumber, MatingToothNumber (basic inputs)
    - VirtualTeethNumber (calculated as roundup(Z / cos(delta)))
    - GearHeight, MatingGearHeight (derived via expressions)
    - DrivingGearBaseThickness (base thickness for dual-trapezoid extensions)
    - TeethLength (distance between diagonal profile lines for tooth extrusion)

    Future phases will add pressure_angle, shaft_angle, face_width, bore_diameter
    and other advanced parameters.

    Parameters are created with a unique prefix to avoid conflicts when
    multiple gears exist in the same design.

    Args:
        state: The generation state containing design and parameter prefix
        spec: The bevel gear specification with all parameter values

    Returns:
        None (modifies design by adding parameters as side effect)
    """
    prefix = state.param_prefix
    design = state.design

    # Basic input parameters
    add_parameter(
        design, prefix, 'Module',
        adsk.core.ValueInput.createByReal(spec.module),
        '',
        'Module for the bevel gear'
    )

    add_parameter(
        design, prefix, 'ToothNumber',
        adsk.core.ValueInput.createByReal(spec.tooth_number),
        '',
        'Number of teeth on main gear'
    )

    add_parameter(
        design, prefix, 'MatingToothNumber',
        adsk.core.ValueInput.createByReal(spec.mating_tooth_number),
        '',
        'Number of teeth on mating gear'
    )

    # Pressure angle (needed for tooth profile generation)
    add_parameter(
        design, prefix, 'PressureAngle',
        adsk.core.ValueInput.createByReal(spec.pressure_angle),
        'rad',
        'Pressure angle for bevel gear'
    )

    # VirtualTeethNumber (calculated as roundup(Z / cos(delta)) for bevel gears)
    add_parameter(
        design, prefix, 'VirtualTeethNumber',
        adsk.core.ValueInput.createByReal(spec.driving_gear_virtual_teeth_number),
        '',
        'Virtual teeth number for bevel gear (Zv = roundup(Z / cos(pitch_cone_angle)))'
    )

    # Derived parameters using expressions
    # GearHeight = Module * ToothNumber (pitch radius of main gear)
    add_parameter(
        design, prefix, 'GearHeight',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "Module")} * {make_param_name(prefix, "ToothNumber")}'
        ),
        'mm',
        'Pitch radius of main gear'
    )

    # MatingGearHeight = Module * MatingToothNumber (pitch radius of mating gear)
    add_parameter(
        design, prefix, 'MatingGearHeight',
        adsk.core.ValueInput.createByString(
            f'{make_param_name(prefix, "Module")} * {make_param_name(prefix, "MatingToothNumber")}'
        ),
        'mm',
        'Pitch radius of mating gear'
    )

    # DrivingGearBaseThickness (vertical distance for P6->P8 and horizontal distance for P10->P12 connector lines)
    add_parameter(
        design, prefix, 'DrivingGearBaseThickness',
        adsk.core.ValueInput.createByReal(spec.driving_gear_base_thickness),
        'mm',
        'Base thickness for the driving gear (vertical distance P6->P8 and horizontal distance P10->P12)'
    )

    # TeethLength (distance between diagonal lines in gear extension for tooth profile)
    add_parameter(
        design, prefix, 'TeethLength',
        adsk.core.ValueInput.createByReal(spec.teeth_length),
        'mm',
        'Distance between diagonal lines in gear extension for tooth profile extrusion'
    )
