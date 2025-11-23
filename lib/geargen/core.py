"""
Core utility functions for gear generation.

This module contains shared functionality used across all gear types. These are
gear-agnostic functions that handle component creation, parameter management,
sketch creation, and plane normalization.
"""

from typing import Tuple, Optional, Union
import adsk.core
import adsk.fusion
from .constants import PLANE_GEAR_BASE_COPLANAR


def create_gear_occurrence(
    parent_component: adsk.fusion.Component,
    gear_type: str
) -> Tuple[adsk.fusion.Occurrence, str]:
    """
    Create a new component occurrence for a gear and generate its unique prefix.

    This function creates a new component occurrence as a child of the parent
    component and generates a unique parameter prefix based on the gear type
    and the component's unique ID. The prefix is used to namespace all user
    parameters for this gear to avoid conflicts when multiple gears are
    generated in the same design.

    Args:
        parent_component: The parent component where the gear will be created
        gear_type: The type of gear being created (e.g., 'SpurGear', 'BevelGear')

    Returns:
        A tuple containing (occurrence, param_prefix) where occurrence is the
        newly created component occurrence and param_prefix is the unique
        string to use for parameter naming
    """
    occurrence = parent_component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    prefix = f'{gear_type}_{occurrence.component.id.replace("-", "")}'
    return occurrence, prefix


def make_param_name(prefix: str, name: str) -> str:
    """
    Create a prefixed parameter name.

    Combines the parameter prefix with the parameter name to create a unique
    parameter name. This ensures that parameters from different gears don't
    conflict with each other.

    Args:
        prefix: The parameter prefix (typically from create_gear_occurrence)
        name: The base parameter name (e.g., 'Module', 'ToothNumber')

    Returns:
        The prefixed parameter name in the format 'prefix_name'
    """
    return f'{prefix}_{name}'


def add_parameter(
    design: adsk.fusion.Design,
    prefix: str,
    name: str,
    value: adsk.core.ValueInput,
    units: str,
    comment: str
) -> adsk.fusion.UserParameter:
    """
    Add a user parameter with the given prefix.

    Creates a new user parameter in the design with a prefixed name. The
    parameter can use either a direct value or an expression that references
    other parameters.

    Args:
        design: The Fusion 360 design where the parameter will be added
        prefix: The parameter prefix for namespacing
        name: The base parameter name
        value: The parameter value (can be a real value or expression string)
        units: The units for the parameter (e.g., 'mm', 'deg', or '' for unitless)
        comment: A descriptive comment for the parameter

    Returns:
        The newly created UserParameter object
    """
    param_name = make_param_name(prefix, name)
    return design.userParameters.add(param_name, value, units, comment)


def get_parameter(
    design: adsk.fusion.Design,
    prefix: str,
    name: str
) -> Optional[adsk.fusion.UserParameter]:
    """
    Get a user parameter by prefixed name.

    Looks up a user parameter using the prefixed name format. Returns None
    if the parameter doesn't exist.

    Args:
        design: The Fusion 360 design to search in
        prefix: The parameter prefix for namespacing
        name: The base parameter name

    Returns:
        The UserParameter object if found, None otherwise
    """
    param_name = make_param_name(prefix, name)
    return design.userParameters.itemByName(param_name)


def create_sketch(
    component: adsk.fusion.Component,
    name: str,
    plane: Optional[adsk.fusion.ConstructionPlane] = None
) -> adsk.fusion.Sketch:
    """
    Create a sketch on the given plane.

    Creates a new sketch in the specified component on the given plane. If no
    plane is provided, uses the component's XY construction plane. The sketch
    is created with visibility set to false by default to keep the design tree
    clean.

    Args:
        component: The component where the sketch will be created
        name: The name to assign to the sketch
        plane: The construction plane to create the sketch on (defaults to XY plane)

    Returns:
        The newly created Sketch object
    """
    if plane is None:
        plane = component.xYConstructionPlane
    sketch = component.sketches.add(plane)
    sketch.name = name
    sketch.isVisible = False
    return sketch


def ensure_construction_plane(
    component: adsk.fusion.Component,
    plane: Union[adsk.fusion.ConstructionPlane, adsk.core.Base]
) -> adsk.fusion.ConstructionPlane:
    """
    Ensure the plane is a construction plane.

    If the provided plane is already a construction plane, returns it as-is.
    If it's a face or other planar surface, creates a coplanar construction
    plane at the same location. This allows users to select faces, planes,
    or other planar entities when selecting the generation plane.

    Args:
        component: The component where a construction plane may need to be created
        plane: The plane or planar surface to normalize

    Returns:
        A ConstructionPlane object representing the same plane
    """
    if plane.objectType == adsk.fusion.ConstructionPlane.classType():
        return plane

    plane_input = component.constructionPlanes.createInput()
    plane_input.setByOffset(plane, adsk.core.ValueInput.createByReal(0))
    construction_plane = component.constructionPlanes.add(plane_input)
    construction_plane.name = PLANE_GEAR_BASE_COPLANAR
    return construction_plane


def hide_construction_planes(state: 'GenerationState') -> None:
    """
    Hide all construction planes in the generation state.

    This function iterates through all fields in the GenerationState and
    hides (sets isLightBulbOn = False) any construction planes it finds.
    This is useful for cleaning up the UI after gear generation is complete.

    Args:
        state: The generation state containing construction planes to hide
    """
    import dataclasses

    # Iterate through all fields in the dataclass
    for field in dataclasses.fields(state):
        value = getattr(state, field.name)
        # Check if this field is a ConstructionPlane
        if value and hasattr(value, 'objectType'):
            if value.objectType == adsk.fusion.ConstructionPlane.classType():
                value.isLightBulbOn = False
