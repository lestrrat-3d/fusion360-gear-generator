from .spurgear import *
from .misc import *
from .types import HelicalGearSpec, GenerationState
from .inputs import parse_helical_gear_inputs
from .parameters import create_helical_gear_parameters
from .core import create_gear_occurrence, ensure_construction_plane, create_sketch, get_parameter
from .components import get_parent_component


def create_helical_gear_spec(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> 'HelicalGearSpec':
    """
    Parse inputs and create a HelicalGearSpec dataclass.

    This function extracts all helical gear parameters from the command dialog
    inputs and creates a HelicalGearSpec dataclass instance. The HelicalGearSpec
    will automatically compute derived parameters (pitch diameter, base circle
    radius, etc.) in its __post_init__ method.

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A fully populated HelicalGearSpec instance with both input and derived
        parameters

    Raises:
        Exception: If required parameters are invalid or missing
    """
    params = parse_helical_gear_inputs(inputs, design)

    return HelicalGearSpec(
        module=params['module'],
        tooth_number=params['tooth_number'],
        pressure_angle=params['pressure_angle'],
        thickness=params['thickness'],
        bore_diameter=params['bore_diameter'],
        chamfer_tooth=params['chamfer_tooth'],
        helix_angle=params['helix_angle'],
        sketch_only=params['sketch_only']
    )


def make_helical_gear_name(spec: 'HelicalGearSpec') -> str:
    """
    Generate a descriptive name for the helical gear component.

    Creates a human-readable name for the gear component that includes the
    key parameters. This makes it easier to identify different gears in the
    design tree.

    Args:
        spec: The helical gear specification

    Returns:
        A descriptive string like "Helical Gear (M=1, Teeth=17, T=5, Angle=14.5°)"
    """
    import math
    angle_deg = math.degrees(spec.helix_angle)
    return f'Helical Gear (M={spec.module}, Teeth={spec.tooth_number}, T={spec.thickness}, Angle={angle_deg:.1f}°)'


def generate_helical_gear(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> 'GenerationState':
    """
    Main entry point for helical gear generation.

    This function coordinates the entire helical gear generation process. It
    handles input parsing, component creation, parameter setup, and calls
    the appropriate sketch/body generation functions based on whether
    sketch-only mode is enabled.

    The generation process follows these steps:
    1. Parse inputs and create specification
    2. Extract parent component and plane/point selections
    3. Create new component and occurrence
    4. Initialize generation state
    5. Set descriptive component name
    6. Create all user parameters
    7. Normalize plane to construction plane
    8. Prepare tools (anchor point, extrusion plane)
    9. Generate sketches or full body based on sketch_only flag

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design where the gear will be created

    Returns:
        The final GenerationState containing all created components, sketches,
        and bodies

    Raises:
        Exception: If inputs are invalid, required selections are missing, or
                  generation fails
    """
    INPUT_ID_PLANE = 'plane'
    INPUT_ID_ANCHOR_POINT = 'anchorPoint'

    # 1. Parse inputs and create specification
    spec = create_helical_gear_spec(inputs, design)

    # 2. Extract parent component and selections
    parent_component = get_parent_component(inputs)

    plane_selections, _ = get_selection_input(inputs, INPUT_ID_PLANE)
    if len(plane_selections) != 1:
        raise Exception(f"Must select exactly one plane (got {len(plane_selections)})")
    plane = plane_selections[0]

    point_selections, _ = get_selection_input(inputs, INPUT_ID_ANCHOR_POINT)
    if len(point_selections) != 1:
        raise Exception(f"Must select exactly one anchor point (got {len(point_selections)})")
    anchor_point = point_selections[0]

    # 3. Create component and occurrence
    occurrence, prefix = create_gear_occurrence(parent_component, 'HelicalGear')

    # 4. Initialize generation state
    state = GenerationState(
        design=design,
        parent_component=parent_component,
        component=occurrence.component,
        occurrence=occurrence,
        param_prefix=prefix,
    )

    # 5. Set component name
    state.component.name = make_helical_gear_name(spec)

    # 6. Create parameters
    create_helical_gear_parameters(state, spec)

    # 7. Normalize plane to construction plane
    state.plane = ensure_construction_plane(state.component, plane)

    # 8. Prepare tools (anchor point and extrusion plane)
    state = prepare_helical_gear_tools(state, spec, state.plane, anchor_point)

    # 9. Generate geometry based on sketch_only flag
    if spec.sketch_only:
        # For sketch-only mode, just build the sketches and make them visible
        state = build_helical_gear_sketches(state, spec)
        state.sketch.isVisible = True
        if state.twisted_sketch:
            state.twisted_sketch.isVisible = True
    else:
        # For full body mode, build the complete 3D gear
        state = build_helical_gear_body(state, spec)

    # Hide all construction planes (they're only needed during generation)
    hide_construction_planes(state)

    return state


def prepare_helical_gear_tools(
    state: 'GenerationState',
    spec: 'HelicalGearSpec',
    plane: adsk.fusion.ConstructionPlane,
    anchor_point: any
) -> 'GenerationState':
    """
    Create tool sketch with anchor point, extrusion plane, and helix plane.

    This function creates a "Tools" sketch that contains the projected anchor
    point, creates a construction plane for extrusion, and creates an additional
    helix plane offset by the thickness for the twisted tooth profile.

    Args:
        state: The current generation state
        spec: The helical gear specification
        plane: The construction plane to create the tools sketch on
        anchor_point: The point entity selected by the user (to be projected)

    Returns:
        Updated GenerationState with anchor_point, extrusion_end_plane, and
        helix_plane fields populated
    """
    # Create tools sketch on the specified plane
    sketch = create_sketch(state.component, 'Tools', plane)

    # Project the anchor point into this sketch
    projected = sketch.project(anchor_point)
    projected_anchor = projected.item(0) if projected.count > 0 else None

    if projected_anchor is None:
        raise Exception("Failed to project anchor point")

    # Hide the tools sketch
    sketch.isVisible = False

    # Create extrusion end plane offset by thickness
    thickness_param = get_parameter(state.design, state.param_prefix, 'Thickness')
    if thickness_param is None:
        raise Exception("Thickness parameter not found")

    thickness_value = adsk.core.ValueInput.createByReal(thickness_param.value)

    plane_input = state.component.constructionPlanes.createInput()
    plane_input.setByOffset(plane, thickness_value)
    extrusion_end_plane = state.component.constructionPlanes.add(plane_input)
    extrusion_end_plane.name = 'Helical Gear Extrusion End Plane'

    # Create helix plane for the twisted tooth profile
    # This plane is at the same location as the extrusion end plane
    helix_plane_input = state.component.constructionPlanes.createInput()
    helix_plane_input.setByOffset(plane, adsk.core.ValueInput.createByReal(to_cm(spec.thickness)))
    helix_plane = state.component.constructionPlanes.add(helix_plane_input)
    helix_plane.name = 'Helical Gear Twisted Profile Plane'

    # Return updated state with new fields
    return state.update(
        anchor_point=projected_anchor,
        extrusion_end_plane=extrusion_end_plane,
        helix_plane=helix_plane
    )


def build_helical_gear_sketches(
    state: 'GenerationState',
    spec: 'HelicalGearSpec'
) -> 'GenerationState':
    """
    Create the gear profile sketches with involute teeth.

    This function creates TWO sketches for a helical gear:
    1. Bottom sketch at angle=0 (on the base plane)
    2. Top sketch at angle=helix_angle (on the helix plane)

    Both sketches include all circles (root, base, pitch, tip) and the
    involute tooth profile. The sketches are fully constrained and ready
    for lofting.

    Args:
        state: The current generation state with anchor_point and helix_plane populated
        spec: The helical gear specification

    Returns:
        Updated GenerationState with sketch and twisted_sketch fields populated
    """
    # Create the bottom gear profile sketch (angle=0)
    sketch = create_sketch(state.component, 'Gear Profile', state.plane)

    # Project the anchor point into this sketch first
    projected_collection = sketch.project(state.anchor_point)
    if projected_collection.count == 0:
        raise Exception("Failed to project anchor point into gear profile sketch")

    # Create temporary state with projected anchor for this sketch
    state_with_projected_anchor = state.update(
        anchor_point=projected_collection.item(0)  # Use projected anchor
    )

    # Draw all circles (root, base, pitch, tip) using projected anchor
    draw_spur_gear_circles(sketch, state_with_projected_anchor, spec)

    # Draw the involute tooth profile at angle 0 using projected anchor
    draw_involute_tooth(sketch, state_with_projected_anchor, spec, angle=0)

    # Create the top gear profile sketch (twisted by helix angle)
    twisted_sketch = create_sketch(state.component, 'Twisted Gear Profile', state.extrusion_end_plane)

    # Project the anchor point into the twisted sketch
    twisted_projected_collection = twisted_sketch.project(state.anchor_point)
    if twisted_projected_collection.count == 0:
        raise Exception("Failed to project anchor point into twisted gear profile sketch")

    # Create temporary state with projected anchor for the twisted sketch
    temp_state = state.update(
        plane=state.extrusion_end_plane,
        anchor_point=twisted_projected_collection.item(0)  # Use projected anchor for twisted sketch
    )

    # Draw all circles again (they'll be at the same radii but on the offset plane)
    draw_spur_gear_circles(twisted_sketch, temp_state, spec)

    # Draw the involute tooth profile at the helix angle
    draw_involute_tooth(twisted_sketch, temp_state, spec, angle=spec.helix_angle)

    # Return updated state with both sketches populated
    return state.update(
        sketch=sketch,
        twisted_sketch=twisted_sketch,
        tooth_profile_is_embedded=temp_state.tooth_profile_is_embedded
    )


def loft_helical_tooth(
    state: 'GenerationState',
    spec: 'HelicalGearSpec'
) -> 'GenerationState':
    """
    Loft the tooth profile between bottom and top sketches.

    This function creates a loft feature that connects the tooth profile from
    the bottom sketch to the tooth profile in the top sketch. This creates the
    characteristic twisted shape of helical gear teeth.

    Args:
        state: The current generation state with sketch and twisted_sketch populated
        spec: The helical gear specification

    Returns:
        Updated GenerationState with tooth_body field populated

    Raises:
        Exception: If the tooth profiles cannot be found in the sketches
    """
    lofts = state.component.features.loftFeatures

    bottom_sketch = state.sketch
    top_sketch = state.twisted_sketch

    # Find the tooth profile in both sketches
    bottom_tooth_profile = find_tooth_profile(bottom_sketch)
    top_tooth_profile = find_tooth_profile(top_sketch)

    if bottom_tooth_profile is None:
        raise Exception("Could not find bottom tooth profile")
    if top_tooth_profile is None:
        raise Exception("Could not find top tooth profile")

    # Create loft input
    loft_input = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loft_input.loftSections.add(bottom_tooth_profile)
    loft_input.loftSections.add(top_tooth_profile)

    # Create the loft
    loft_result = lofts.add(loft_input)
    state.tooth_body = loft_result.bodies.item(0)
    state.tooth_body.name = 'Tooth Body'

    # Apply chamfer to tooth if needed
    # For helical gears, we expect 4 edges per face (not 6 like spur)
    if spec.chamfer_tooth > 0:
        chamfer_lofted_tooth(state, spec)

    return state


def build_helical_gear_body(
    state: 'GenerationState',
    spec: 'HelicalGearSpec'
) -> 'GenerationState':
    """
    Build the 3D helical gear body by lofting tooth and extruding body.

    This function orchestrates the entire 3D body generation process for a helical
    gear. It calls all the necessary functions in the correct order to create
    the complete gear body including tooth lofting, body extrusion, circular
    patterning, fillets, and bore creation.

    The steps are:
    1. Build sketches (bottom and top tooth profiles)
    2. Loft tooth profile between sketches
    3. Extrude gear body (annular ring)
    4. Pattern tooth around center axis
    5. Apply fillets to root edges (if specified)
    6. Create bore hole (if specified)

    Args:
        state: The current generation state
        spec: The helical gear specification

    Returns:
        Updated GenerationState with gear_body field populated and all features created
    """
    # Build both sketches (bottom and top)
    state = build_helical_gear_sketches(state, spec)

    # Loft the tooth between the two profiles
    state = loft_helical_tooth(state, spec)

    # Extrude body (same as spur gear)
    state = extrude_spur_body(state, spec)

    # Pattern teeth around center
    pattern_spur_teeth(state, spec)

    # Create fillets if specified
    create_spur_fillets(state, spec)

    # Create bore if specified
    if spec.bore_diameter > 0:
        create_spur_bore(state, spec)

    return state