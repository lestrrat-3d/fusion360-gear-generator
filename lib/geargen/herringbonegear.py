"""
Herringbone gear generation module.

This module provides both OOP (legacy) and functional (new) APIs for generating
herringbone gears in Fusion 360. Herringbone gears are double helical gears
where the teeth are mirrored at the midpoint, forming a V-shape pattern.

Herringbone gears extend helical gears by mirroring the tooth geometry across
a midpoint plane, which creates the characteristic V-shaped tooth pattern and
helps balance axial thrust forces.
"""

from .helicalgear import *
from .misc import *
from .types import HerringboneGearSpec, GenerationState
from .inputs import parse_herringbone_gear_inputs
from .parameters import create_herringbone_gear_parameters
from .core import create_gear_occurrence, ensure_construction_plane, create_sketch, get_parameter
from .components import get_parent_component


# ==============================================================================
# OLD CLASS-BASED API - COMMENTED OUT (No longer used, replaced by functional API)
# ==============================================================================
# The code below is the old class-based implementation that has been replaced
# by the functional API starting at line ~68. It is commented out to ensure
# it's not accidentally used.
# ==============================================================================

# class HerringboneGearSpecification(HelicalGearSpecification): pass
# 
# class HerringboneGearGenerationContext(HelicalGearGenerationContext):
#     def __init__(self):
#         super().__init__()
# 
# class HerringboneGearGenerator(HelicalGearGenerator):
#     def __init__(self, component: adsk.fusion.Component):
#         super().__init__(component)
# 
#     def newContext(self):
#         return HerringboneGearGenerationContext()
# 
#     def generateName(self, spec):
#         return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(spec.module, spec.toothNumber, spec.thickness, math.degrees(spec.helixAngle))
# 
#     def helicalPlaneOffset(self, spec: HelicalGearSpecification):
#         return super().helicalPlaneOffset(spec) / 2
# 
#     def buildTooth(self, ctx: GenerationContext, spec: SpurGearSpecification):
#         self.loftTooth(ctx, spec)
# 
#         # mirror the single tooth
#         entities = adsk.core.ObjectCollection.create()
#         entities.add(ctx.toothBody)
#         input = self.component.features.mirrorFeatures.createInput(entities, ctx.helixPlane)
#         mirrorResult = self.component.features.mirrorFeatures.add(input)
#         mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'
# 
#         entities = adsk.core.ObjectCollection.create()
#         entities.add(mirrorResult.bodies.item(0))
#         combineInput = self.component.features.combineFeatures.createInput(
#             self.component.bRepBodies.itemByName('Tooth Body'),
#             entities,
#         )
#         self.component.features.combineFeatures.add(combineInput)
#         if spec.chamferTooth > 0:
#             self.chamferTooth(ctx, spec)
# 
# class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator): pass


# ==============================================================================
# Functional API - New implementation
# ==============================================================================

def create_herringbone_gear_spec(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> 'HerringboneGearSpec':
    """
    Parse inputs and create a HerringboneGearSpec dataclass.

    This function extracts all herringbone gear parameters from the command
    dialog inputs and creates a HerringboneGearSpec dataclass instance.
    Herringbone gears use the same specification as helical gears since they
    have identical parameters (the difference is in the geometry generation).

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design (needed for expression evaluation)

    Returns:
        A fully populated HerringboneGearSpec (alias to HelicalGearSpec)
        instance with both input and derived parameters

    Raises:
        Exception: If required parameters are invalid or missing
    """
    from .inputs import parse_herringbone_gear_inputs
    from .types import HerringboneGearSpec

    params = parse_herringbone_gear_inputs(inputs, design)

    return HerringboneGearSpec(
        module=params['module'],
        tooth_number=params['tooth_number'],
        pressure_angle=params['pressure_angle'],
        thickness=params['thickness'],
        bore_diameter=params['bore_diameter'],
        chamfer_tooth=params['chamfer_tooth'],
        helix_angle=params['helix_angle'],
        sketch_only=params['sketch_only']
    )


def make_herringbone_gear_name(spec: 'HerringboneGearSpec') -> str:
    """
    Generate a descriptive name for the herringbone gear component.

    Creates a human-readable name for the gear component that includes the
    key parameters. This makes it easier to identify different gears in the
    design tree.

    Args:
        spec: The herringbone gear specification

    Returns:
        A descriptive string like "Herringbone Gear (M=1, Teeth=17, T=5, Angle=14.5°)"
    """
    import math
    angle_deg = math.degrees(spec.helix_angle)
    return f'Herringbone Gear (M={spec.module}, Teeth={spec.tooth_number}, T={spec.thickness}, Angle={angle_deg:.1f}°)'


def generate_herringbone_gear(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> 'GenerationState':
    """
    Main entry point for herringbone gear generation.

    This function coordinates the entire herringbone gear generation process.
    It handles input parsing, component creation, parameter setup, and calls
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
    8. Prepare tools (anchor point, extrusion plane, helix plane at half thickness)
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
    from .inputs import get_selection_input
    from .components import get_parent_component
    from .core import create_gear_occurrence, ensure_construction_plane
    from .parameters import create_herringbone_gear_parameters
    from .types import GenerationState

    INPUT_ID_PLANE = 'plane'
    INPUT_ID_ANCHOR_POINT = 'anchorPoint'

    # 1. Parse inputs and create specification
    spec = create_herringbone_gear_spec(inputs, design)

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
    occurrence, prefix = create_gear_occurrence(parent_component, 'HerringboneGear')

    # 4. Initialize generation state
    state = GenerationState(
        design=design,
        parent_component=parent_component,
        component=occurrence.component,
        occurrence=occurrence,
        param_prefix=prefix,
    )

    # 5. Set component name
    state.component.name = make_herringbone_gear_name(spec)

    # 6. Create parameters
    create_herringbone_gear_parameters(state, spec)

    # 7. Normalize plane to construction plane
    state.plane = ensure_construction_plane(state.component, plane)

    # 8. Prepare tools (anchor point, extrusion plane, and helix plane)
    # NOTE: For herringbone, the helix plane is at HALF thickness (not full thickness like helical)
    state = prepare_herringbone_gear_tools(state, spec, state.plane, anchor_point)

    # 9. Generate geometry based on sketch_only flag
    if spec.sketch_only:
        # For sketch-only mode, just build the sketches and make them visible
        state = build_herringbone_gear_sketches(state, spec)
        state.sketch.isVisible = True
        if state.twisted_sketch:
            state.twisted_sketch.isVisible = True
    else:
        # For full body mode, build the complete 3D gear
        state = build_herringbone_gear_body(state, spec)

    # Hide all construction planes (they're only needed during generation)
    from .core import hide_construction_planes
    hide_construction_planes(state)

    return state


def prepare_herringbone_gear_tools(
    state: 'GenerationState',
    spec: 'HerringboneGearSpec',
    plane: adsk.fusion.ConstructionPlane,
    anchor_point: any
) -> 'GenerationState':
    """
    Create tool sketch with anchor point, extrusion plane, and helix plane.

    This function creates a "Tools" sketch that contains the projected anchor
    point, creates a construction plane for extrusion, and creates an additional
    helix plane offset by HALF the thickness (not full thickness like helical).
    This half-thickness offset is critical for herringbone gears because the
    tooth will be mirrored at this midpoint to create the V-shape.

    Args:
        state: The current generation state
        spec: The herringbone gear specification
        plane: The construction plane to create the tools sketch on
        anchor_point: The point entity selected by the user (to be projected)

    Returns:
        Updated GenerationState with anchor_point, extrusion_end_plane, and
        helix_plane fields populated
    """
    from .core import create_sketch, get_parameter
    from .types import GenerationState
    from .misc import to_cm

    # Create tools sketch on the specified plane
    sketch = create_sketch(state.component, 'Tools', plane)

    # Project the anchor point into this sketch
    projected = sketch.project(anchor_point)
    projected_anchor = projected.item(0) if projected.count > 0 else None

    if projected_anchor is None:
        raise Exception("Failed to project anchor point")

    # Hide the tools sketch
    sketch.isVisible = False

    # Create extrusion end plane offset by full thickness
    thickness_param = get_parameter(state.design, state.param_prefix, 'Thickness')
    if thickness_param is None:
        raise Exception("Thickness parameter not found")

    thickness_value = adsk.core.ValueInput.createByReal(thickness_param.value)

    plane_input = state.component.constructionPlanes.createInput()
    plane_input.setByOffset(plane, thickness_value)
    extrusion_end_plane = state.component.constructionPlanes.add(plane_input)
    extrusion_end_plane.name = 'Herringbone Gear Extrusion End Plane (Full Thickness)'

    # Create helix plane for the twisted tooth profile
    # CRITICAL: For herringbone, this is at HALF thickness (midpoint) not full thickness
    helix_plane_input = state.component.constructionPlanes.createInput()
    helix_plane_input.setByOffset(plane, adsk.core.ValueInput.createByReal(thickness_param.value / 2))
    helix_plane = state.component.constructionPlanes.add(helix_plane_input)
    helix_plane.name = 'Herringbone Gear Mid Plane (Half Thickness)'

    # Return updated state with new fields
    return GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=projected_anchor,
        extrusion_end_plane=extrusion_end_plane,
        helix_plane=helix_plane,
        sketch=state.sketch,
        twisted_sketch=state.twisted_sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent
    )


def build_herringbone_gear_sketches(
    state: 'GenerationState',
    spec: 'HerringboneGearSpec'
) -> 'GenerationState':
    """
    Create the gear profile sketches with involute teeth.

    This function creates TWO sketches for a herringbone gear:
    1. Bottom sketch at angle=0 (on the base plane)
    2. Midpoint sketch at angle=helix_angle (on the helix plane at half thickness)

    The second sketch will be used for lofting and then mirroring to create the
    V-shaped herringbone tooth pattern. Both sketches include all circles
    (root, base, pitch, tip) and the involute tooth profile. The sketches are
    fully constrained and ready for lofting and mirroring.

    Args:
        state: The current generation state with anchor_point and helix_plane populated
        spec: The herringbone gear specification

    Returns:
        Updated GenerationState with sketch and twisted_sketch fields populated
    """
    from .core import create_sketch
    from .spurgear import draw_spur_gear_circles, draw_involute_tooth
    from .types import GenerationState

    # Create the bottom gear profile sketch (angle=0)
    sketch = create_sketch(state.component, 'Gear Profile', state.plane)

    # Project the anchor point into this sketch first
    projected_collection = sketch.project(state.anchor_point)
    if projected_collection.count == 0:
        raise Exception("Failed to project anchor point into gear profile sketch")

    # Create temporary state with projected anchor for this sketch
    state_with_projected_anchor = GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=projected_collection.item(0),  # Use projected anchor
        helix_plane=state.helix_plane,
        sketch=state.sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent
    )

    # Draw all circles (root, base, pitch, tip) using projected anchor
    draw_spur_gear_circles(sketch, state_with_projected_anchor, spec)

    # Draw the involute tooth profile at angle 0 using projected anchor
    draw_involute_tooth(sketch, state_with_projected_anchor, spec, angle=0)

    # Create the midpoint gear profile sketch (twisted by helix angle)
    # This is at half thickness for herringbone (the V-point)
    twisted_sketch = create_sketch(state.component, 'Twisted Gear Profile', state.helix_plane)

    # Project the anchor point into the twisted sketch
    twisted_projected_collection = twisted_sketch.project(state.anchor_point)
    if twisted_projected_collection.count == 0:
        raise Exception("Failed to project anchor point into twisted gear profile sketch")

    # Create temporary state with projected anchor for the twisted sketch
    temp_state = GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.helix_plane,
        anchor_point=twisted_projected_collection.item(0),  # Use projected anchor for twisted sketch
        helix_plane=state.helix_plane,
        sketch=state.sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent
    )

    # Draw all circles again (they'll be at the same radii but on the offset plane)
    draw_spur_gear_circles(twisted_sketch, temp_state, spec)

    # Draw the involute tooth profile at the helix angle
    draw_involute_tooth(twisted_sketch, temp_state, spec, angle=spec.helix_angle)

    # Return updated state with both sketches populated
    return GenerationState(
        design=state.design,
        parent_component=state.parent_component,
        component=state.component,
        occurrence=state.occurrence,
        param_prefix=state.param_prefix,
        plane=state.plane,
        anchor_point=state.anchor_point,
        sketch=sketch,
        twisted_sketch=twisted_sketch,
        gear_body=state.gear_body,
        tooth_body=state.tooth_body,
        center_axis=state.center_axis,
        extrusion_extent=state.extrusion_extent,
        extrusion_end_plane=state.extrusion_end_plane,
        helix_plane=state.helix_plane,
        tooth_profile_is_embedded=temp_state.tooth_profile_is_embedded
    )


def loft_and_mirror_herringbone_tooth(
    state: 'GenerationState',
    spec: 'HerringboneGearSpec'
) -> 'GenerationState':
    """
    Loft the tooth profile between bottom and midpoint sketches, then mirror.

    This function creates a loft feature that connects the tooth profile from
    the bottom sketch to the tooth profile at the midpoint sketch. Then it
    mirrors this lofted tooth across the helix plane to create the second half
    of the herringbone tooth. Finally, it combines both halves into a single
    tooth body.

    The steps are:
    1. Loft from bottom (angle=0) to midpoint (angle=helix_angle)
    2. Mirror the lofted tooth across the helix plane
    3. Combine the original and mirrored tooth bodies
    4. Apply chamfer if specified

    Args:
        state: The current generation state with sketch and twisted_sketch populated
        spec: The herringbone gear specification

    Returns:
        Updated GenerationState with tooth_body field populated

    Raises:
        Exception: If the tooth profiles cannot be found in the sketches
    """
    from .spurgear import chamfer_spur_tooth

    lofts = state.component.features.loftFeatures

    bottom_sketch = state.sketch
    midpoint_sketch = state.twisted_sketch

    # Find the tooth profile in both sketches
    # The tooth profile has exactly 6 curves (or 4 if embedded)
    def find_tooth_profile(profiles):
        for profile in profiles:
            for loop in profile.profileLoops:
                if loop.profileCurves.count == 6 or loop.profileCurves.count == 4:
                    # Verify it's actually a tooth profile by checking curve types
                    # Should have nurbs (involutes) and arcs
                    has_nurbs = False
                    has_arcs = False
                    for curve in loop.profileCurves:
                        if curve.geometry.curveType == adsk.core.Curve3DTypes.NurbsCurve3DCurveType:
                            has_nurbs = True
                        elif curve.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                            has_arcs = True
                    if has_nurbs and has_arcs:
                        return profile
        return None

    bottom_tooth_profile = find_tooth_profile(bottom_sketch.profiles)
    midpoint_tooth_profile = find_tooth_profile(midpoint_sketch.profiles)

    if bottom_tooth_profile is None:
        raise Exception("Could not find bottom tooth profile")
    if midpoint_tooth_profile is None:
        raise Exception("Could not find midpoint tooth profile")

    # Step 1: Create loft from bottom to midpoint
    loft_input = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loft_input.loftSections.add(bottom_tooth_profile)
    loft_input.loftSections.add(midpoint_tooth_profile)

    # Create the loft
    loft_result = lofts.add(loft_input)
    state.tooth_body = loft_result.bodies.item(0)
    state.tooth_body.name = 'Tooth Body'

    # Step 2: Mirror the lofted tooth across the helix plane
    entities = adsk.core.ObjectCollection.create()
    entities.add(state.tooth_body)
    mirror_input = state.component.features.mirrorFeatures.createInput(entities, state.helix_plane)
    mirror_result = state.component.features.mirrorFeatures.add(mirror_input)
    mirrored_body = mirror_result.bodies.item(0)
    mirrored_body.name = 'Tooth Body (Mirrored)'

    # Step 3: Combine the original and mirrored tooth bodies
    entities = adsk.core.ObjectCollection.create()
    entities.add(mirrored_body)
    combine_input = state.component.features.combineFeatures.createInput(
        state.tooth_body,
        entities,
    )
    state.component.features.combineFeatures.add(combine_input)

    # Step 4: Apply chamfer to tooth if needed
    if spec.chamfer_tooth > 0:
        chamfer_herringbone_tooth(state, spec)

    return state


def chamfer_herringbone_tooth(
    state: 'GenerationState',
    spec: 'HerringboneGearSpec'
) -> None:
    """
    Apply chamfer to the herringbone tooth edges if chamfer_tooth > 0.

    This function finds the appropriate edges on the tooth body and applies
    a chamfer to them. For herringbone gears, the tooth geometry is more
    complex due to the mirroring operation, so we need to carefully select
    the edges to chamfer.

    Args:
        state: The current generation state with tooth_body populated
        spec: The herringbone gear specification
    """
    if spec.chamfer_tooth <= 0:
        return

    tooth_body = state.tooth_body
    spline_edges = adsk.core.ObjectCollection.create()

    want_surface_type = adsk.core.SurfaceTypes.PlaneSurfaceType
    want_edges = 4  # Herringbone gear has 4 edges per planar face (like helical)

    # The selection of face/edge is very hard coded
    for face in tooth_body.faces:
        # Surface must be a plane
        if face.geometry.surfaceType != want_surface_type:
            continue
        # Face must contain exactly 4 edges
        if len(face.edges) != want_edges:
            continue

        # Only accept this face if the edges contain exactly two splines (nurbs)
        spline_count = 0
        for edge in face.edges:
            if edge.geometry.objectType == 'adsk::core::NurbsCurve3D':
                spline_count += 1

        if spline_count != 2:
            continue

        # We don't want to chamfer the Arc that is part of the root circle
        root_circle_radius_cm = spec.root_circle_radius
        for edge in face.edges:
            if edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType:
                if abs(edge.geometry.radius - root_circle_radius_cm) < 0.001:
                    continue
            spline_edges.add(edge)

    if spline_edges.count > 0:
        chamfer_input = state.component.features.chamferFeatures.createInput2()
        chamfer_input.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
            spline_edges,
            adsk.core.ValueInput.createByReal(spec.chamfer_tooth),
            False
        )
        state.component.features.chamferFeatures.add(chamfer_input)


def build_herringbone_gear_body(
    state: 'GenerationState',
    spec: 'HerringboneGearSpec'
) -> 'GenerationState':
    """
    Build the 3D herringbone gear body by lofting, mirroring tooth and extruding body.

    This function orchestrates the entire 3D body generation process for a
    herringbone gear. It calls all the necessary functions in the correct order
    to create the complete gear body including tooth lofting, mirroring, body
    extrusion, circular patterning, fillets, and bore creation.

    The steps are:
    1. Build sketches (bottom and midpoint tooth profiles)
    2. Loft tooth profile between sketches
    3. Mirror the lofted tooth across the helix plane
    4. Combine original and mirrored tooth into single body
    5. Extrude gear body (annular ring)
    6. Pattern tooth around center axis
    7. Apply fillets to root edges (if specified)
    8. Create bore hole (if specified)

    Args:
        state: The current generation state
        spec: The herringbone gear specification

    Returns:
        Updated GenerationState with gear_body field populated and all features created
    """
    from .spurgear import extrude_spur_body, pattern_spur_teeth, create_spur_fillets, create_spur_bore

    # Build both sketches (bottom and midpoint)
    state = build_herringbone_gear_sketches(state, spec)

    # Loft the tooth between the two profiles and mirror it
    state = loft_and_mirror_herringbone_tooth(state, spec)

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
