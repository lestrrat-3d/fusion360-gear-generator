"""
Component management functions for gear generation.

This module contains functions for managing Fusion 360 components and
occurrences, including cleanup operations and parent component extraction.
"""

from typing import Tuple, List, Any
import adsk.core
import adsk.fusion
from .types import ComponentCleanupInfo
from .inputs import get_selection_input


def delete_component_and_parameters(
    cleanup_info: ComponentCleanupInfo,
    design: adsk.fusion.Design
) -> None:
    """
    Delete all parameters with the given prefix and the component occurrence.

    This function handles parameter dependencies by iterating until all deletable
    parameters are removed. Some parameters may depend on others, so we need to
    delete them in multiple passes. Parameters that cannot be deleted (because
    they're still referenced) will remain after the deletion process completes.

    The function will iterate through all user parameters, identify those that
    match the prefix from cleanup_info, and delete them in waves. Each wave
    deletes all parameters that are currently deletable. The iteration continues
    until either all matching parameters are deleted or no progress is made
    (indicating remaining parameters have unresolvable dependencies).

    After parameter deletion, the component occurrence itself is deleted.

    Args:
        cleanup_info: ComponentCleanupInfo containing the parameter prefix and
                     occurrence to clean up
        design: The Fusion 360 design containing the parameters and occurrence

    Returns:
        None - This function modifies the design by deleting parameters and the
        occurrence
    """
    user_parameters = design.userParameters
    to_delete = {}

    # Find all parameters with this prefix
    for param in user_parameters:
        if param.name.startswith(cleanup_info.param_prefix):
            to_delete[param.name] = param

    # Delete in multiple passes to handle dependencies
    prev_size = len(to_delete)
    while prev_size > 0:
        # Try to delete all remaining parameters
        for name in list(to_delete.keys()):
            param = to_delete[name]
            if param.isDeletable:
                param.deleteMe()
                del to_delete[name]

        # Check if we made progress
        current_size = len(to_delete)
        if current_size == prev_size:
            # No progress made, stop trying
            break
        prev_size = current_size

    # Delete the occurrence
    cleanup_info.occurrence.deleteMe()


def get_parent_component(
    inputs: adsk.core.CommandInputs
) -> adsk.fusion.Component:
    """
    Extract the parent component from command inputs.

    This function retrieves the parent component selection from the command
    dialog inputs. The parent component determines where the new gear component
    will be created. The function handles both Occurrence and Component types,
    as users can select either in the Fusion 360 UI.

    When an Occurrence is selected, we extract its underlying Component. When
    a Component is selected directly, we use it as-is.

    Args:
        inputs: The CommandInputs collection from the command dialog

    Returns:
        The parent Component where the new gear should be created

    Raises:
        Exception: If no parent component is selected, multiple selections are
                  made, or the selected entity is neither an Occurrence nor a
                  Component
    """
    selections, ok = get_selection_input(inputs, 'parentComponent')
    if not ok or len(selections) != 1:
        raise Exception("Must select exactly one parent component")

    parent = selections[0]

    # Handle Occurrence type - extract the component
    if parent.objectType == adsk.fusion.Occurrence.classType():
        return parent.component
    # Handle Component type - use directly
    elif parent.objectType == adsk.fusion.Component.classType():
        return parent
    else:
        raise Exception(f'Invalid parent type: {parent.objectType}')
