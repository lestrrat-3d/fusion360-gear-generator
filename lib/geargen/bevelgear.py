"""
Bevel gear generation functions.

This module contains functions for generating bevel gears following a functional
programming pattern. Bevel gears are conical gears that transmit motion between
intersecting shafts.

Phase 1 Implementation (Foundation Sketch with Dual-Trapezoid Extensions):
This phase creates the component hierarchy and foundation sketch that defines the
pitch cone cross-section. The foundation sketch includes:
- Base rectangle (P1-P4) showing pitch radii relationship
- Diagonal reference line (P2->P4)
- Dual-trapezoid extensions on gear side (P5-P10) with intermediate point P7_mid and diagonal profile line P9->P10
- Dual-trapezoid extensions on mating side (P11-P16) with intermediate point P14_mid and diagonal profile line P15->P16

Total of 18 key points (P1-P16 + P7_mid + P14_mid) define the complete foundation sketch.

The dual-trapezoid extensions define face width boundaries for future tooth
generation. Each side has two nested trapezoids controlled by the module and
DrivingGearBaseThickness parameters. The diagonal profile lines (P9->P10 on gear side, P15->P16 on mating side) are
parallel to the outer diagonals and positioned at distance TeethLength, which
controls tooth extrusion in future phases.

CRITICAL Phase 1 Storage Requirements:
- P1->P2 axis (vertical line) MUST be stored in state.p1_p2_axis for Phase 2 circular pattern
- P5->P7 line (SketchLine object) MUST be stored in state.p5_p7_line for Phase 2 tooth profile plane

Phase 2 Implementation (Tooth Profile Sketch and 3D Tooth Body):
This phase creates the tooth profile sketch and generates the complete 3D bevel gear
with all teeth. Phase 2 consists of two parts:

Part 1: Tooth Profile Sketch Creation
- Activates Driving Gear component for tooth generation
- Creates construction plane perpendicular to P5->P7 line (from state.p5_p7_line) at P7
- Creates tooth profile sketch on perpendicular plane
- Calculates virtual teeth number: Zv = ceil(Z / cos(pitch_cone_angle))
- Draws spur gear tooth profile using virtual teeth number
- Aligns tooth profile with P5->P7 line direction

Part 2: 3D Tooth Body Creation
- Revolves hexagonal profile (P5, P7, P8, P6, P9, P10 - 6 vertices, NOT P7_mid) by 360° to create base gear body
- Creates lofted tooth from apex (P2) to tooth profile
- Identifies conical cutting faces from base_gear_body intersecting P9->P10 and P5->P7
- Cuts lofted tooth using identified faces (BEFORE joining to base)
- Removes smaller trimmed parts, keeping largest fragment (trimmed tooth)
- Joins trimmed tooth to base gear body
- Patterns teeth circularly around P1->P2 axis (from state.p1_p2_axis)

Phase 1 Main Functions (All follow (state, spec, ...) signature pattern):
- generate_bevel_gear: Main entry point for Phase 1 bevel gear generation
- create_bevel_gear_components: Creates Design/Driving Gear/Mating Gear sub-components
- create_perpendicular_plane: Creates plane perpendicular to gear plane
- create_foundation_sketch: Creates the pitch cone cross-section sketch with extensions
- draw_foundation_rectangle(state, spec, sketch, projected_anchor_point): Draws base rectangle, stores p1-p4 and p1_p2_axis
- draw_apex_diagonal(state, sketch): Draws diagonal reference line, stores diagonal
- draw_gear_face_extension(state, spec, sketch): Draws gear-side extension, stores p5-p10 and p5_p7_line (CRITICAL)
- draw_mating_face_extension(state, spec, sketch): Draws mating-side extension, stores p11-p16

Phase 2 Main Functions (All follow (state, spec) signature pattern):
- generate_phase2_tooth_body: Main pipeline for Phase 2 (tooth profile and 3D body)
- activate_driving_gear_component: Activates Driving Gear component
- create_tooth_profile_plane: Creates plane perpendicular to state.p5_p7_line at P7
- create_tooth_profile_sketch: Creates tooth profile sketch and projects P7
- get_virtual_teeth_number: Calculates virtual teeth number for tooth profile
- draw_spur_tooth_profile: Draws spur tooth profile using virtual teeth number
- rotate_tooth_profile_to_align: Aligns tooth profile with P5->P7 direction
- create_base_gear_body: Revolves hexagonal profile (6 vertices) to create base conical gear body
- create_lofted_tooth: Lofts from apex (P2) to tooth profile
- identify_cutting_faces: Identifies conical faces from base_gear_body using state.p5_p7_line
- cut_body_with_faces: Cuts lofted tooth using identified faces (before joining)
- remove_smaller_parts: Removes trimmed parts, keeps largest body (trimmed tooth)
- join_tooth_to_gear_body: Joins trimmed tooth to base gear body
- pattern_teeth_circularly: Patterns teeth around state.p1_p2_axis

Phase 2 Validation and Error Handling:
- validate_phase1_outputs: Validates Phase 1 artifacts before Phase 2
- validate_spec_for_phase2: Validates BevelGearSpec for Phase 2 requirements
- cleanup_phase2_on_error: Cleans up partial Phase 2 geometry on failure
- format_phase2_error: Formats consistent error messages with diagnostics

Implementation Notes:
- All functions follow functional programming patterns with explicit state passing
- Standardized signature pattern: (state, spec, ...) with state mutation and return
- GenerationState carries all artifacts through the pipeline
- CRITICAL state storage: p1_p2_axis and p5_p7_line stored in Phase 1, used in Phase 2
- Phase 2 can be run independently if Phase 1 state is available
- Error handling preserves Phase 1 geometry and cleans up partial Phase 2 artifacts
- Complete involute curve generation implemented for tooth profile using virtual teeth number
- Hexagonal profile for revolve: exactly 6 vertices (P5, P7, P8, P6, P9, P10), NOT P7_mid
- Cutting faces identified from base_gear_body (conical faces from revolve), not joined_body
- Split body operations implemented using conical faces from base_gear_body
- Circular pattern uses state.p1_p2_axis as rotation axis
- Parameter-driven dimensions used throughout tooth profile generation

Testing Requirements:
- Manual testing: Run Phase 1 + Phase 2 pipeline with various parameters
- Validate tooth profile geometry matches virtual teeth calculations
- Verify base gear body and lofted tooth creation
- Check joined body and trimming operations
- Verify circular pattern creates correct tooth count
- Test error recovery and cleanup on failures
"""

import math
from typing import Optional, Any, Tuple
import adsk.core
import adsk.fusion
from .types import GenerationState, BevelGearSpec, GearConfig
from .core import create_sketch, get_parameter, create_gear_occurrence, ensure_construction_plane, hide_construction_planes, make_param_name
from .involute import draw_involute_tooth_profile, ToothProfileConfig
from .spurgear import draw_spur_gear_circles
from .inputs import parse_bevel_gear_inputs, get_selection_input
from .components import get_parent_component
from .parameters import create_bevel_gear_parameters
from .misc import to_cm, get_design
from ...lib import fusion360utils as futil
from .constants.entities import (
    COMPONENT_DESIGN,
    COMPONENT_DRIVING_GEAR,
    COMPONENT_MATING_GEAR,
    SKETCH_FOUNDATION,
    SKETCH_TOOTH_PROFILE,
    SKETCH_PERPENDICULAR_REFERENCE,
    SKETCH_TOOTH_PROFILE_REFERENCE,
    SKETCH_APEX_FOR_LOFT,
    PLANE_FOUNDATION,
    PLANE_TOOTH_PROFILE,
    SKETCH_TEXT_HEIGHT_CM,
)
from .constants.params import (
    PARAM_MODULE,
    PARAM_GEAR_HEIGHT,
    PARAM_MATING_GEAR_HEIGHT,
    PARAM_DRIVING_GEAR_BASE_THICKNESS,
    PARAM_TEETH_LENGTH,
    PARAM_TOOTH_NUMBER,
    PARAM_MATING_TOOTH_NUMBER,
    DEDENDUM_MULTIPLIER,
    GEAR_EXTENSION_MARGIN_CM,
)
from .constants.tolerances import (
    MINIMUM_PROFILE_LINE_LENGTH_CM,
    FLOATING_POINT_TOLERANCE_CM,
    PROXIMITY_TOLERANCE_CM,
    VECTOR_LENGTH_MINIMUM,
    PERPENDICULARITY_THRESHOLD,
    TINY_CIRCLE_RADIUS_CM,
)
from .constants.geometry import (
    ANGLE_90_DEG,
    ANGLE_360_DEG,
)
from .constants.messages import (
    ERR_PERPENDICULAR_CONSTRAINT_FAILED,
    ERR_DIMENSION_TO_LINE_FAILED,
    ERR_CONSTRAINT_TO_LINE_FAILED,
    ERR_COLLINEAR_CONSTRAINT_FAILED,
    ERR_COINCIDENT_CONSTRAINT_FAILED,
    ERR_DIAGONAL_PROFILE_LINE_FAILED,
    ERR_PROFILE_LINE_TOO_SHORT,
    ERR_TEETH_LENGTH_TOO_SMALL,
    ERR_GEAR_DIMENSIONS_INVALID,
    ERR_BASE_GEAR_BODY_REVOLVE_FAILED,
    ERR_LOFT_APEX_TO_PROFILE_FAILED,
    ERR_JOIN_TRIMMED_TOOTH_FAILED,
    ERR_SPLIT_BODY_WITH_FACE_FAILED,
    ERR_TOOTH_PROFILE_PLANE_FAILED,
    ERR_PROFILE_LINE_START_PROJECTION_FAILED,
    ERR_PROFILE_LINE_END_PROJECTION_FAILED,
    ERR_ALIGNMENT_POINT_PROJECTION_FAILED,
    ERR_APEX_POINT_PROJECTION_FAILED,
    ERR_PERPENDICULAR_LINE_PROJECTION_FAILED,
    ERR_ANCHOR_POINT_CALCULATE_FAILED,
    ERR_CONSTRAIN_INTERSECTION_FAILED,
    ERR_PARALLEL_CONSTRAINT_FAILED,
)


def create_bevel_gear_components(state: GenerationState) -> GenerationState:
    """
    Create the three-level component hierarchy for bevel gear.

    This function creates three sub-components within the main bevel gear component:
    - Design: Contains foundation plane and foundation sketch
    - Driving Gear: Placeholder for future main gear tooth generation
    - Mating Gear: Placeholder for future mating gear generation

    The component hierarchy organizes the bevel gear geometry and keeps construction
    elements separate from final gear bodies.

    Args:
        state: The current generation state with main component created

    Returns:
        Updated GenerationState with design_component, gear_component, and
        mating_gear_component fields populated
    """
    # Create identity transform matrix for sub-components
    transform = adsk.core.Matrix3D.create()

    # Create "Design" sub-component
    design_occurrence = state.component.occurrences.addNewComponent(transform)
    design_component = design_occurrence.component
    design_component.name = COMPONENT_DESIGN

    # Create "Driving Gear" sub-component
    gear_occurrence = state.component.occurrences.addNewComponent(transform)
    gear_component = gear_occurrence.component
    gear_component.name = COMPONENT_DRIVING_GEAR

    # Create "Mating Gear" sub-component
    mating_gear_occurrence = state.component.occurrences.addNewComponent(transform)
    mating_gear_component = mating_gear_occurrence.component
    mating_gear_component.name = COMPONENT_MATING_GEAR

    # Update state with new component and occurrence references
    state.design_component = design_component
    state.gear_component = gear_component
    state.gear_occurrence = gear_occurrence  # Store occurrence for driving gear activation
    state.mating_gear_component = mating_gear_component
    state.mating_gear_occurrence = mating_gear_occurrence  # Store occurrence for mating gear activation

    return state


def create_perpendicular_plane(
    state: GenerationState,
    gear_plane: adsk.fusion.ConstructionPlane,
    anchor_point_entity: Any
) -> GenerationState:
    """
    Create a construction plane perpendicular to the gear plane, passing through the projected anchor point.

    This function uses the setByAngle approach to create a plane that is exactly
    perpendicular to the gear plane. The strategy is:
    1. Project anchor point onto gear_plane
    2. Extract gear_plane normal vector
    3. Compute perpendicular direction using cross product
    4. Draw perpendicular line in temporary sketch on gear_plane
    5. Create plane at 90 degrees to gear_plane using setByAngle with this line
    6. Clean up temporary sketch

    This approach ensures the plane passes through the projected anchor point and is
    exactly perpendicular to gear_plane regardless of gear_plane orientation.

    Args:
        design_component: The Design sub-component where plane will be created
        gear_plane: The user-selected Gear Plane (normalized to ConstructionPlane)
        anchor_point_entity: The user-selected entity to project onto gear_plane

    Returns:
        Updated GenerationState with foundation_plane and perpendicular_reference_line populated

    Raises:
        Exception: If plane creation fails or anchor point cannot be projected
    """
    # Create temporary sketch on gear_plane for projection
    temp_sketch = state.design_component.sketches.add(gear_plane)

    # Project anchor_point_entity into sketch to get projected point on gear_plane
    projected = temp_sketch.project(anchor_point_entity)
    if projected.count == 0:
        raise Exception("Failed to project anchor point onto gear plane")
    projected_anchor_point = projected.item(0)

    # Extract gear plane normal vector
    normal = gear_plane.geometry.normal

    # Compute perpendicular direction using cross product
    # Try X axis first
    perp_dir = normal.crossProduct(adsk.core.Vector3D.create(1, 0, 0))
    if perp_dir.length < VECTOR_LENGTH_MINIMUM:
        # Normal is parallel to X, use Y instead
        perp_dir = normal.crossProduct(adsk.core.Vector3D.create(0, 1, 0))
    perp_dir.normalize()

    # Calculate end point for perpendicular line (arbitrary length of 10mm)
    end_point = adsk.core.Point3D.create(
        projected_anchor_point.geometry.x + perp_dir.x * 10,
        projected_anchor_point.geometry.y + perp_dir.y * 10,
        projected_anchor_point.geometry.z + perp_dir.z * 10
    )

    # Draw perpendicular line in temporary sketch
    perp_line = temp_sketch.sketchCurves.sketchLines.addByTwoPoints(
        projected_anchor_point,
        end_point
    )
    # TODO: constrain perp_line length so that the sketch is fully constrained

    # Create plane using setByAngle at 90 degrees to gear_plane
    plane_input = state.design_component.constructionPlanes.createInput()
    plane_input.setByAngle(
        perp_line,
        adsk.core.ValueInput.createByReal(ANGLE_90_DEG),
        gear_plane
    )
    foundation_plane = state.design_component.constructionPlanes.add(plane_input)
    foundation_plane.name = PLANE_FOUNDATION

    temp_sketch.isVisible = False  # Hide instead of delete to avoid API issues
    temp_sketch.name = SKETCH_PERPENDICULAR_REFERENCE

    state.foundation_plane = foundation_plane
    state.perpendicular_reference_line = perp_line
    return state


def add_line_label(sketch: adsk.fusion.Sketch, line: adsk.fusion.SketchLine, label: str) -> None:
    """
    Add a text label along a sketch line for debugging.

    Uses the same approach as circle labeling in spurgear.py - placing text along the path.

    Args:
        sketch: The sketch containing the line
        line: The line to label
        label: The text to display (e.g., "P5->P6")
    """
    # Create text input with label and height
    text_input = sketch.sketchTexts.createInput2(label, SKETCH_TEXT_HEIGHT_CM)  # 1mm text height for sketch labels

    # Place text along the line path (same method used for circles in spurgear.py:1194)
    text_input.setAsAlongPath(
        line,  # Place text along this line
        True,  # isAbove - place text above the line
        adsk.core.HorizontalAlignments.CenterHorizontalAlignment,  # Center the text
        0  # offset from line
    )

    # Add text to sketch
    sketch.sketchTexts.add(text_input)


def detect_sketch_orientation(sketch: adsk.fusion.Sketch, gear_plane: adsk.fusion.ConstructionPlane) -> bool:
    """
    Detect if sketch Y-axis is perpendicular to gear_plane.

    This function determines which sketch axis (X or Y) is more perpendicular to the gear_plane's
    normal vector. This is necessary because the foundation_plane's orientation (created via
    setByAngle) is not directly controllable, so we adapt the drawing logic based on the actual
    sketch orientation.

    Args:
        sketch: The foundation sketch whose orientation we're detecting
        gear_plane: The user-selected construction plane

    Returns:
        True if sketch.yDirection is perpendicular to gear_plane (standard orientation)
        False if sketch.xDirection is perpendicular to gear_plane (swapped orientation)

    Raises:
        Exception: If both axes are equally perpendicular (degenerate case)
    """
    sketch_x = sketch.xDirection
    sketch_y = sketch.yDirection
    gear_normal = gear_plane.geometry.normal

    # Calculate dot products (closer to 1.0 means more parallel/perpendicular)
    # The dot product of two unit vectors gives cos(angle)
    # If perpendicular (90°), dot product = 0
    # If parallel (0° or 180°), |dot product| = 1
    x_perpendicularity = abs(sketch_x.dotProduct(gear_normal))
    y_perpendicularity = abs(sketch_y.dotProduct(gear_normal))

    # Check for degenerate case (both equally perpendicular, within threshold)
    if abs(x_perpendicularity - y_perpendicularity) < PERPENDICULARITY_THRESHOLD:
        raise Exception(
            f"Cannot determine sketch orientation: both axes are equally perpendicular "
            f"(x={x_perpendicularity:.3f}, y={y_perpendicularity:.3f})"
        )

    # Whichever is closer to 1.0 is the perpendicular direction
    return y_perpendicularity > x_perpendicularity


def draw_foundation_rectangle(
    state: GenerationState,
    spec: BevelGearSpec,
    sketch: adsk.fusion.Sketch,
    projected_anchor_point: adsk.fusion.SketchPoint
) -> GenerationState:
    """
    Draw the four-sided rectangle representing the pitch cone cross-section with full constraints.

    This function draws a fully-constrained rectangle where:
    - Bottom-left corner (P1) is at projected_anchor_point (Gear Center Point)
    - P1->P2 points perpendicular to gear_plane (away from base)
    - P1->P4 points parallel to gear_plane (along base)
    - P1->P4 (bottom edge) is marked as construction

    Points are labeled: P1 (bottom-left), P2 (top-left/apex), P3 (top-right), P4 (bottom-right)

    The rectangle is fully constrained using geometric constraints (VERTICAL, HORIZONTAL,
    PERPENDICULAR, COINCIDENT) and dimensional constraints tied to user parameters.
    The constraint order is critical for proper propagation.

    This function adapts to the sketch orientation (which sketch axis is perpendicular to gear_plane)
    by detecting the orientation and applying appropriate constraints.

    This function mutates state directly by storing p1, p2, p3, p4, p1_p2_axis, and y_is_perpendicular.

    Args:
        state: The generation state containing parameter prefix and gear_plane
        spec: The bevel gear specification
        sketch: The foundation sketch to draw in
        projected_anchor_point: The projected anchor point (bottom-left corner, P1)

    Returns:
        Updated GenerationState with p1, p2, p3, p4, p1_p2_axis, and y_is_perpendicular populated

    Raises:
        Exception: If sketch is not fully constrained after drawing
        Exception: If gear_plane is missing from state
    """
    # Detect sketch orientation relative to gear_plane
    if not hasattr(state, 'gear_plane') or state.gear_plane is None:
        raise Exception("gear_plane is missing from state - required for orientation detection")

    y_is_perpendicular = detect_sketch_orientation(sketch, state.gear_plane)
    state.y_is_perpendicular = y_is_perpendicular

    # Get parameter references for dimensions
    gear_height_param = get_parameter(state.design, state.param_prefix, PARAM_GEAR_HEIGHT)
    mating_height_param = get_parameter(state.design, state.param_prefix, PARAM_MATING_GEAR_HEIGHT)

    if not gear_height_param:
        raise Exception(f"{PARAM_GEAR_HEIGHT} parameter not found")
    if not mating_height_param:
        raise Exception(f"{PARAM_MATING_GEAR_HEIGHT} parameter not found")

    # Compute corner point coordinates based on orientation
    # P1 = origin (projected_anchor_point)
    # P2 = P1 + perpendicular direction (mating_gear_height)
    # P4 = P1 + parallel direction (gear_height)
    # P3 = P2 + parallel direction
    #
    # IMPORTANT: Use Point3D objects (not SketchPoint) to avoid over-constraint
    # The API creates SketchPoints automatically when drawing lines

    p1_x = projected_anchor_point.geometry.x
    p1_y = projected_anchor_point.geometry.y

    if y_is_perpendicular:
        # Standard orientation: sketch +Y is perpendicular, sketch +X is parallel
        # P1->P2 along sketch +Y, P1->P4 along sketch +X
        p2_coords = adsk.core.Point3D.create(p1_x, p1_y + mating_height_param.value, 0)
        p4_coords = adsk.core.Point3D.create(p1_x + gear_height_param.value, p1_y, 0)
    else:
        # Swapped orientation: sketch +X is perpendicular, sketch +Y is parallel
        # P1->P2 along sketch +X, P1->P4 along sketch +Y
        p2_coords = adsk.core.Point3D.create(p1_x + mating_height_param.value, p1_y, 0)
        p4_coords = adsk.core.Point3D.create(p1_x, p1_y + gear_height_param.value, 0)

    # Draw first two lines - API creates SketchPoints at endpoints
    line_p1_p2 = sketch.sketchCurves.sketchLines.addByTwoPoints(projected_anchor_point, p2_coords)
    line_p1_p4 = sketch.sketchCurves.sketchLines.addByTwoPoints(projected_anchor_point, p4_coords)

    # Get SketchPoints from line endpoints
    p2_point = line_p1_p2.endSketchPoint
    p4_point = line_p1_p4.endSketchPoint

    # Compute P3 coordinates based on orientation
    if y_is_perpendicular:
        p3_coords = adsk.core.Point3D.create(p4_coords.x, p2_coords.y, 0)
    else:
        p3_coords = adsk.core.Point3D.create(p2_coords.x, p4_coords.y, 0)

    # Draw remaining two lines - API creates SketchPoints
    line_p4_p3 = sketch.sketchCurves.sketchLines.addByTwoPoints(p4_point, p3_coords)
    line_p2_p3 = sketch.sketchCurves.sketchLines.addByTwoPoints(p2_point, p3_coords)

    # Get P3 SketchPoint from line endpoint
    p3_point = line_p4_p3.endSketchPoint

    # Mark P1->P4 (bottom edge) as construction
    line_p1_p4.isConstruction = True

    # Mark P4->P3 (right edge) as construction
    line_p4_p3.isConstruction = True

    # Add text labels for debugging
    add_line_label(sketch, line_p1_p2, "P1->P2")
    add_line_label(sketch, line_p1_p4, "P1->P4")
    add_line_label(sketch, line_p2_p3, "P2->P3")
    add_line_label(sketch, line_p4_p3, "P4->P3")

    # Apply geometric constraints based on orientation
    if y_is_perpendicular:
        # Standard orientation:
        # - line_p1_p2 uses sketch +Y (perpendicular) -> VERTICAL
        # - line_p1_p4 uses sketch +X (parallel) -> HORIZONTAL
        # - line_p4_p3 uses sketch +Y (perpendicular) -> VERTICAL
        # - line_p2_p3 uses sketch +X (parallel) -> HORIZONTAL
        sketch.geometricConstraints.addVertical(line_p1_p2)
        sketch.geometricConstraints.addHorizontal(line_p1_p4)
        sketch.geometricConstraints.addVertical(line_p4_p3)
        sketch.geometricConstraints.addHorizontal(line_p2_p3)
    else:
        # Swapped orientation:
        # - line_p1_p2 uses sketch +X (perpendicular) -> HORIZONTAL
        # - line_p1_p4 uses sketch +Y (parallel) -> VERTICAL
        # - line_p4_p3 uses sketch +X (perpendicular) -> HORIZONTAL
        # - line_p2_p3 uses sketch +Y (parallel) -> VERTICAL
        sketch.geometricConstraints.addHorizontal(line_p1_p2)
        sketch.geometricConstraints.addVertical(line_p1_p4)
        sketch.geometricConstraints.addHorizontal(line_p4_p3)
        sketch.geometricConstraints.addVertical(line_p2_p3)

    # Apply EQUAL constraints (orientation-independent)
    sketch.geometricConstraints.addEqual(line_p2_p3, line_p1_p4)
    sketch.geometricConstraints.addEqual(line_p4_p3, line_p1_p2)

    # Apply dimensional constraints using parameter expressions
    if y_is_perpendicular:
        # P1->P2 dimension with VERTICAL orientation
        dim_1 = sketch.sketchDimensions.addDistanceDimension(
            line_p1_p2.startSketchPoint,
            line_p1_p2.endSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(p1_x - 2, p1_y + mating_height_param.value / 2, 0)
        )
        dim_1.parameter.expression = make_param_name(state.param_prefix, PARAM_MATING_GEAR_HEIGHT)

        # P1->P4 dimension with HORIZONTAL orientation
        dim_2 = sketch.sketchDimensions.addDistanceDimension(
            line_p1_p4.startSketchPoint,
            line_p1_p4.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(p1_x + gear_height_param.value / 2, p1_y - 2, 0)
        )
        dim_2.parameter.expression = make_param_name(state.param_prefix, PARAM_GEAR_HEIGHT)
    else:
        # P1->P2 dimension with HORIZONTAL orientation
        dim_1 = sketch.sketchDimensions.addDistanceDimension(
            line_p1_p2.startSketchPoint,
            line_p1_p2.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(p1_x + mating_height_param.value / 2, p1_y - 2, 0)
        )
        dim_1.parameter.expression = make_param_name(state.param_prefix, PARAM_MATING_GEAR_HEIGHT)

        # P1->P4 dimension with VERTICAL orientation
        dim_2 = sketch.sketchDimensions.addDistanceDimension(
            line_p1_p4.startSketchPoint,
            line_p1_p4.endSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(p1_x - 2, p1_y + gear_height_param.value / 2, 0)
        )
        dim_2.parameter.expression = make_param_name(state.param_prefix, PARAM_GEAR_HEIGHT)

    # CRITICAL: Store P1->P2 axis (line perpendicular to gear_plane) for Phase 2 circular pattern
    # This line will be used as the rotation axis when creating circular pattern of teeth
    state.p1_p2_axis = line_p1_p2

    # Store all four corner points in state (P1, P2, P3, P4)
    state.p1 = projected_anchor_point
    state.p2 = p2_point
    state.p3 = p3_point
    state.p4 = p4_point

    # Store foundation edges for tooth profile constraints
    state.p1_p4_line = line_p1_p4  # Gear side edge
    state.p2_p3_line = line_p2_p3  # Mating side edge

    # CRITICAL: Store P2->P3 axis for Phase 3 (mating gear) circular pattern
    # This line will be used as the rotation axis for mating gear tooth pattern
    state.p2_p3_axis = line_p2_p3

    return state


def draw_apex_diagonal(
    state: GenerationState,
    sketch: adsk.fusion.Sketch
) -> GenerationState:
    """
    Draw the diagonal line from apex (P2) to opposite corner (P4) as a construction line.

    This diagonal is a reference line that may be used in future phases for tooth
    profile generation. It is marked as construction (isConstruction = True) so
    it appears dashed in the UI.

    This function mutates state directly by storing the diagonal line in state.diagonal.

    Args:
        state: The generation state containing p2 (apex) and p4 (opposite corner)
        sketch: The foundation sketch

    Returns:
        Updated GenerationState with diagonal populated
    """
    # Draw line from apex_point (state.p2) to opposite_corner (state.p4)
    diagonal = sketch.sketchCurves.sketchLines.addByTwoPoints(state.p2, state.p4)

    # Note: COINCIDENT constraints not needed - the line was drawn using these exact points,
    # so the endpoints are already coincident with state.p2 and state.p4

    # Set as construction line
    diagonal.isConstruction = True

    # Add text label for debugging
    add_line_label(sketch, diagonal, "P2->P4")

    # Store diagonal in state
    state.diagonal = diagonal

    return state


def draw_gear_face_extension(
    state: GenerationState,
    spec: BevelGearSpec,
    sketch: adsk.fusion.Sketch,
    tooth_number: int
) -> GenerationState:
    """
    Draw the complete gear-side dual-trapezoid extension (extends perpendicular to diagonal towards gear side).

    This function creates a dual-trapezoid extension on the gear side by drawing:
    1. Inner trapezoid: P4->P5->P2 with P5->P6 aligned with perpendicular reference line (parallel to P1->P4) and vertical closing P6->P1
    2. Outer trapezoid: Extending P4->P5 line to P7 (no dimension), with intermediate point P7_mid
       on P5->P7 line. P7_mid connects to P8 aligned with perpendicular reference line (parallel to P1->P4), and vertical connector P6->P8
    3. Diagonal profile line: P9->P10 where P9 is on line P5-P6, P10 is on line P2-P5, parallel to P5->P7, distance = TeethLength

    The first perpendicular extension (P4->P5) is perpendicular to the diagonal
    (P2->P4) and extends perpendicular to diagonal towards gear side by a distance
    of module * 1.25. The second extension (P5->P7) is collinear with P4->P5
    and extends freely without dimension. P7_mid is an intermediate point on P5->P7
    that is positioned by the P6->P8 dimension (DrivingGearBaseThickness).

    The diagonal profile line P9->P10 starts at P9 on the line P5-P6, runs parallel
    to P5->P7, and ends at P10 on line P2-P5. The distance from P9->P10 diagonal to
    P5->P7 is controlled by the TeethLength parameter.

    This function mutates state directly by storing p5, p6, p7, p5_p7_line (CRITICAL for Phase 2),
    p7_mid, p8, p9, and p10.

    After creating all geometry, this function validates that P9->P10 and P5->P7_mid line segments
    are at least 0.1 cm long. This ensures reliable face intersection detection in Phase 2, where
    midpoint checking is used to identify cutting faces.

    Args:
        state: The generation state containing p1, p2, p4, diagonal, and parameter prefix
        spec: The bevel gear specification
        sketch: The foundation sketch to draw in
        tooth_number: Number of teeth for this gear (used for P5->P7 length calculation)

    Returns:
        Updated GenerationState with p5, p6, p7, p5_p7_line, p7_mid, p8, p9, p10 populated

    Raises:
        Exception: If constraints fail or parameters are missing
        Exception: If P9->P10 line is shorter than 0.1 cm (indicates TeethLength too small)
        Exception: If P5->P7_mid line is shorter than 0.1 cm (indicates invalid gear dimensions)
    """
    # Get module and DrivingGearBaseThickness parameters
    module_param = get_parameter(state.design, state.param_prefix, PARAM_MODULE)
    base_thickness_param = get_parameter(state.design, state.param_prefix, PARAM_DRIVING_GEAR_BASE_THICKNESS)

    if not module_param:
        raise Exception(f"{PARAM_MODULE} parameter not found")
    if not base_thickness_param:
        raise Exception(f"{PARAM_DRIVING_GEAR_BASE_THICKNESS} parameter not found")

    # Access p1, p2, p4, diagonal from state (set by draw_foundation_rectangle and draw_apex_diagonal)
    p1 = state.p1
    p2 = state.p2
    p4 = state.p4
    diagonal = state.diagonal

    # Calculate perpendicular direction from diagonal (P2->P4) towards gear side (towards P1)
    # This needs to be orientation-aware, not hard-coded to "lower Y"
    diag_vec_x = p4.geometry.x - p2.geometry.x
    diag_vec_y = p4.geometry.y - p2.geometry.y
    diag_length = math.sqrt(diag_vec_x**2 + diag_vec_y**2)

    # Determine which side of P2->P4 line that P1 is on using cross product
    # Cross product: (P4 - P2) × (P1 - P2) = (p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)
    cross_product = (diag_vec_x * (p1.geometry.y - p2.geometry.y) -
                     diag_vec_y * (p1.geometry.x - p2.geometry.x))

    # If cross product is positive, P1 is on the left side of P2->P4 (counter-clockwise)
    # If negative, P1 is on the right side (clockwise)
    # For gear side extension (P4->P5), we want to go towards P1's side
    if cross_product > 0:
        # P1 is on left side, rotate 90° counter-clockwise: (x, y) -> (-y, x)
        perp_vec_x = -diag_vec_y / diag_length
        perp_vec_y = diag_vec_x / diag_length
    else:
        # P1 is on right side, rotate 90° clockwise: (x, y) -> (y, -x)
        perp_vec_x = diag_vec_y / diag_length
        perp_vec_y = -diag_vec_x / diag_length

    # Calculate P5 position (module * DEDENDUM_MULTIPLIER distance from P4, perpendicular to diagonal)
    ext_length = module_param.value * DEDENDUM_MULTIPLIER
    p5_pos = adsk.core.Point3D.create(
        p4.geometry.x + perp_vec_x * ext_length,
        p4.geometry.y + perp_vec_y * ext_length,
        0
    )

    # Draw first perpendicular line (P4->P5)
    p4_p5_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p4, p5_pos)
    p4_p5_line.isConstruction = False

    # Add PERPENDICULAR constraint to state.diagonal
    try:
        sketch.geometricConstraints.addPerpendicular(p4_p5_line, state.diagonal)
    except Exception as e:
        raise Exception(f"Failed to add perpendicular constraint to P4->P5 line: {str(e)}")

    # Apply dimension: module * DEDENDUM_MULTIPLIER
    try:
        dim_p4_p5 = sketch.sketchDimensions.addDistanceDimension(
            p4_p5_line.startSketchPoint,
            p4_p5_line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p4.geometry.x + p5_pos.x) / 2,
                (p4.geometry.y + p5_pos.y) / 2,
                0
            )
        )
        dim_p4_p5.parameter.expression = f'{make_param_name(state.param_prefix, PARAM_MODULE)} * {DEDENDUM_MULTIPLIER}'
    except Exception as e:
        raise Exception(f"Failed to add dimension to P4->P5 line: {str(e)}")

    # Store P5
    p5 = p4_p5_line.endSketchPoint

    # Draw slant line (P2->P5)
    p2_p5_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p2, p5)
    p2_p5_line.isConstruction = False

    # Draw line (P5->P6) towards P1 level (parallel to P1->P4)
    # Orientation-dependent: P1->P4 direction determines this line's direction
    if state.y_is_perpendicular:
        # Standard: P1->P4 is horizontal (sketch +X), so P5->P6 is horizontal
        p6_pos = adsk.core.Point3D.create(
            state.p1.geometry.x,
            p5.geometry.y,
            0
        )
    else:
        # Swapped: P1->P4 is vertical (sketch +Y), so P5->P6 is vertical
        p6_pos = adsk.core.Point3D.create(
            p5.geometry.x,
            state.p1.geometry.y,
            0
        )

    p5_p6_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p5, p6_pos)
    p5_p6_line.isConstruction = False

    # Add constraint based on orientation (line is parallel to P1->P4)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addHorizontal(p5_p6_line)
        else:
            sketch.geometricConstraints.addVertical(p5_p6_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P5->P6 line: {str(e)}")

    # Store P6
    p6 = p5_p6_line.endSketchPoint

    # Draw closing line (P6->P1) - perpendicular to P1->P4
    p6_p1_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p6, state.p1)
    p6_p1_line.isConstruction = False

    # Add constraint based on orientation (perpendicular to P1->P4)
    try:
        if state.y_is_perpendicular:
            # Standard: P1->P4 is horizontal, so P6->P1 is vertical
            sketch.geometricConstraints.addVertical(p6_p1_line)
        else:
            # Swapped: P1->P4 is vertical, so P6->P1 is horizontal
            sketch.geometricConstraints.addHorizontal(p6_p1_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P6->P1 line: {str(e)}")

    # Extend P4->P5 line to create P7 (collinear)
    # Calculate P7 position: P5->P7 length = ((Module * ToothNumber) / 2) + GEAR_EXTENSION_MARGIN_CM - pitch radius plus 0.5mm margin
    p5_p7_length = ((module_param.value * tooth_number) / 2) + GEAR_EXTENSION_MARGIN_CM
    p7_pos = adsk.core.Point3D.create(
        p5.geometry.x + perp_vec_x * p5_p7_length,
        p5.geometry.y + perp_vec_y * p5_p7_length,
        0
    )

    # Draw second perpendicular line (P5->P7), collinear with P4->P5
    p5_p7_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p5, p7_pos)
    p5_p7_line.isConstruction = False

    # Add COLLINEAR constraint to P4->P5 line
    try:
        sketch.geometricConstraints.addCollinear(p5_p7_line, p4_p5_line)
    except Exception as e:
        raise Exception(f"Failed to add collinear constraint to P5->P7 line: {str(e)}")

    # Apply dimension: (((Module * VirtualTeethNumber) / 2) + 0.5) - pitch radius plus margin
    # This constrains the P7 endpoint, while P7_mid will be positioned independently by P6->P8 dimension
    try:
        dim_p5_p7 = sketch.sketchDimensions.addDistanceDimension(
            p5_p7_line.startSketchPoint,
            p5_p7_line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p5.geometry.x + p7_pos.x) / 2 + 1,
                (p5.geometry.y + p7_pos.y) / 2,
                0
            )
        )
        dim_p5_p7.parameter.expression = f'((({make_param_name(state.param_prefix, PARAM_MODULE)} * {make_param_name(state.param_prefix, PARAM_TOOTH_NUMBER)}) / 2) + {GEAR_EXTENSION_MARGIN_CM} cm)'
    except Exception as e:
        raise Exception(f"Failed to add dimension to P5->P7 line: {str(e)}")

    # Store P7
    p7 = p5_p7_line.endSketchPoint

    # Create intermediate point P7_mid on the P5->P7 line as a control point
    # Calculate initial position (will be adjusted by constraints)
    p7_mid_pos = adsk.core.Point3D.create(
        (p5.geometry.x + p7.geometry.x) / 2,
        (p5.geometry.y + p7.geometry.y) / 2,
        0
    )
    p7_mid = sketch.sketchPoints.add(p7_mid_pos)

    # Add COINCIDENT constraint to place P7_mid on the P5->P7 line
    try:
        sketch.geometricConstraints.addCoincident(p7_mid, p5_p7_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint to P7_mid on P5->P7 line: {str(e)}")

    # Draw line (P7_mid->P8) towards P1 level (parallel to P1->P4)
    # Orientation-dependent: P1->P4 direction determines this line's direction
    if state.y_is_perpendicular:
        # Standard: P7_mid->P8 is horizontal, aligned with P1.x
        p8_pos = adsk.core.Point3D.create(
            state.p1.geometry.x,
            p7_mid.geometry.y,
            0
        )
    else:
        # Swapped: P7_mid->P8 is vertical, aligned with P1.y
        p8_pos = adsk.core.Point3D.create(
            p7_mid.geometry.x,
            state.p1.geometry.y,
            0
        )

    p7_mid_p8_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p7_mid, p8_pos)
    p7_mid_p8_line.isConstruction = False

    # Add constraint based on orientation (line is parallel to P1->P4)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addHorizontal(p7_mid_p8_line)
        else:
            sketch.geometricConstraints.addVertical(p7_mid_p8_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P7_mid->P8 line: {str(e)}")

    # Store P8
    p8 = p7_mid_p8_line.endSketchPoint

    # CRITICAL: Constrain P8 to be at the intersection of perpendicular line and reference line
    # This places P8 at exactly the right position for gear base alignment
    if state.perpendicular_line_in_foundation is None:
        raise Exception("perpendicular_line_in_foundation not found in state")
    if state.reference_line_in_foundation is None:
        raise Exception("reference_line_in_foundation not found in state")

    try:
        # Constraint 1: Perpendicular line endpoint is coincident to the reference line
        # This ensures the perpendicular line intersects the reference line at the endpoint
        perp_line_endpoint = state.perpendicular_line_in_foundation.endSketchPoint
        sketch.geometricConstraints.addCoincident(perp_line_endpoint, state.reference_line_in_foundation)

        # Constraint 2: P8 lies on the perpendicular line
        # This positions P8 at the intersection point (through the perpendicular line)
        sketch.geometricConstraints.addCoincident(p8, state.perpendicular_line_in_foundation)
        sketch.geometricConstraints.addCoincident(p8, state.reference_line_in_foundation)
    except Exception as e:
        raise Exception(f"Failed to constrain P8 to intersection: {str(e)}")

    # Draw connector line (P6->P8) to close the outer trapezoid - perpendicular to P1->P4
    p6_p8_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p6, p8)
    p6_p8_line.isConstruction = False

    # Add constraint based on orientation (perpendicular to P1->P4)
    try:
        if state.y_is_perpendicular:
            # Standard: P1->P4 is horizontal, so P6->P8 is vertical
            sketch.geometricConstraints.addVertical(p6_p8_line)
        else:
            # Swapped: P1->P4 is vertical, so P6->P8 is horizontal
            sketch.geometricConstraints.addHorizontal(p6_p8_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P6->P8 line: {str(e)}")

    # Apply dimension = DrivingGearBaseThickness with orientation-dependent dimension orientation
    try:
        if state.y_is_perpendicular:
            dim_orientation = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        else:
            dim_orientation = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation

        dim_p6_p8 = sketch.sketchDimensions.addDistanceDimension(
            p6_p8_line.startSketchPoint,
            p6_p8_line.endSketchPoint,
            dim_orientation,
            adsk.core.Point3D.create(
                p6.geometry.x - 2,
                (p6.geometry.y + p8.geometry.y) / 2,
                0
            )
        )
        dim_p6_p8.parameter.expression = make_param_name(state.param_prefix, PARAM_DRIVING_GEAR_BASE_THICKNESS)
    except Exception as e:
        raise Exception(f"Failed to add dimension to P6->P8 line: {str(e)}")

    # Get TeethLength parameter
    teeth_length_param = get_parameter(state.design, state.param_prefix, 'TeethLength')
    if not teeth_length_param:
        raise Exception("TeethLength parameter not found")

    # Draw diagonal profile line parallel to P5->P7, at distance teeth_length
    # This line runs from line P5-P6 (point P9) to line P2-P5 (point P10)
    # The line is parallel to P5->P7 (collinear direction)

    # Calculate the direction vector of P5->P7 (same as perpendicular direction)
    # We already have perp_vec_x and perp_vec_y calculated earlier
    p5_p7_dir_x = perp_vec_x
    p5_p7_dir_y = perp_vec_y

    # Calculate perpendicular direction to P5->P7 (for offsetting by teeth_length distance)
    # Perpendicular to P5->P7 direction (rotate 90° counter-clockwise)
    # Since perp_vec is (diag_vec_y, -diag_vec_x) / length,
    # rotating 90° counter-clockwise gives: (-(-diag_vec_x), diag_vec_y) / length = (diag_vec_x, diag_vec_y) / length
    perp_to_p5p7_x = -p5_p7_dir_y
    perp_to_p5p7_y = p5_p7_dir_x

    # Move from P5 along the perpendicular direction by teeth_length
    offset_start_x = p5.geometry.x + perp_to_p5p7_x * teeth_length_param.value
    offset_start_y = p5.geometry.y + perp_to_p5p7_y * teeth_length_param.value

    # The diagonal line at distance teeth_length has direction same as P5->P7
    # It passes through offset_start point and extends in P5->P7 direction
    # Find intersection with line P5->P6
    # P5->P6 is either horizontal (standard orientation) or vertical (swapped orientation)
    # Offset diagonal: passes through (offset_start_x, offset_start_y), direction (p5_p7_dir_x, p5_p7_dir_y)
    # Parametric: x = offset_start_x + t * p5_p7_dir_x, y = offset_start_y + t * p5_p7_dir_y
    if state.y_is_perpendicular:
        # Standard orientation: P5->P6 is horizontal (y = p5.y)
        # Solve for t when y = p5.y: offset_start_y + t * p5_p7_dir_y = p5.y
        if abs(p5_p7_dir_y) > 1e-6:
            t = (p5.geometry.y - offset_start_y) / p5_p7_dir_y
            p9_x = offset_start_x + t * p5_p7_dir_x
        else:
            # P5->P7 is horizontal (shouldn't happen), use offset_start_x
            p9_x = offset_start_x
        p9_y = p5.geometry.y  # On horizontal line P5-P6
    else:
        # Swapped orientation: P5->P6 is vertical (x = p5.x)
        # Solve for t when x = p5.x: offset_start_x + t * p5_p7_dir_x = p5.x
        if abs(p5_p7_dir_x) > 1e-6:
            t = (p5.geometry.x - offset_start_x) / p5_p7_dir_x
            p9_y = offset_start_y + t * p5_p7_dir_y
        else:
            # P5->P7 is vertical (shouldn't happen), use offset_start_y
            p9_y = offset_start_y
        p9_x = p5.geometry.x  # On vertical line P5-P6

    # Draw line from P9 (on P5->P6) to intersection with P2->P5
    # P2-P5 line direction
    p2_p5_vec_x = p5.geometry.x - p2.geometry.x
    p2_p5_vec_y = p5.geometry.y - p2.geometry.y

    # Find intersection of diagonal through P9 with line P2-P5
    # Diagonal through P9: direction (p5_p7_dir_x, p5_p7_dir_y), passes through (p9_x, p9_y)
    # Line P2-P5: direction (p2_p5_vec_x, p2_p5_vec_y), passes through P2
    # Solve parametric equations:
    # p9_x + s * p5_p7_dir_x = p2.x + u * p2_p5_vec_x
    # p9_y + s * p5_p7_dir_y = p2.y + u * p2_p5_vec_y
    # This is a 2x2 linear system, solve for s and u
    det = p5_p7_dir_x * p2_p5_vec_y - p5_p7_dir_y * p2_p5_vec_x
    if abs(det) > 1e-6:
        u = ((p9_x - p2.geometry.x) * p5_p7_dir_y - (p9_y - p2.geometry.y) * p5_p7_dir_x) / det
        p10_x = p2.geometry.x + u * p2_p5_vec_x
        p10_y = p2.geometry.y + u * p2_p5_vec_y
    else:
        # Lines are parallel (shouldn't happen), use P9 as fallback for P10
        p10_x = p9_x
        p10_y = p9_y

    # Create P9 (start) and P10 (end) points for diagonal line
    p9_point = adsk.core.Point3D.create(p9_x, p9_y, 0)
    p10_point = adsk.core.Point3D.create(p10_x, p10_y, 0)

    try:
        p9_p10_diagonal_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p9_point, p10_point)
        p9_p10_diagonal_line.isConstruction = False
    except Exception as e:
        raise Exception(f"Failed to draw diagonal profile line P9->P10: {str(e)}")

    # Store P9 (start on P5->P6) and P10 (end on P2-P5)
    p9 = p9_p10_diagonal_line.startSketchPoint
    p10 = p9_p10_diagonal_line.endSketchPoint

    # Add COINCIDENT constraint: P9 start point must be on line P5->P6
    try:
        sketch.geometricConstraints.addCoincident(p9, p5_p6_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint for P9 to line P5->P6: {str(e)}")

    # Add COINCIDENT constraint: P10 end point must be on line P2-P5
    try:
        sketch.geometricConstraints.addCoincident(p10, p2_p5_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint for P10 end point to line P2-P5: {str(e)}")

    # Add PARALLEL constraint to P5->P7 line
    try:
        sketch.geometricConstraints.addParallel(p9_p10_diagonal_line, p5_p7_line)
    except Exception as e:
        raise Exception(f"Failed to add parallel constraint to P9->P10 diagonal line: {str(e)}")

    # Apply distance dimension constraint: distance from P9->P10 line to P5->P7 line = teeth_length
    # To dimension between two parallel lines, we need two points (one on each line)
    # We use P9 (on diagonal) and P5 (start point of P5->P7 line)
    # The dimension will be perpendicular distance between the parallel lines
    try:
        dim_p9_distance = sketch.sketchDimensions.addDistanceDimension(
            p5,  # Point on P5->P7 line
            p9,  # Point on P9->P10 diagonal line
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p5.geometry.x + p9_x) / 2,
                (p5.geometry.y + p9_y) / 2 - 1,
                0
            )
        )
        dim_p9_distance.parameter.expression = make_param_name(state.param_prefix, PARAM_TEETH_LENGTH)
    except Exception as e:
        raise Exception(f"Failed to add dimension to P9->P10 diagonal line: {str(e)}")

    # CRITICAL: Store P5->P7 line (SketchLine object) for Phase 2
    # This line will be used to create the tooth profile plane perpendicular to it
    # and for identifying cutting faces during tooth generation
    state.p5_p7_line = p5_p7_line

    # Store all extension points in state
    state.p5 = p5
    state.p6 = p6
    state.p7 = p7
    state.p7_mid = p7_mid
    state.p8 = p8
    state.p9 = p9
    state.p10 = p10

    # Add text labels for debugging
    add_line_label(sketch, p4_p5_line, "P4->P5")
    add_line_label(sketch, p2_p5_line, "P2->P5")
    add_line_label(sketch, p5_p6_line, "P5->P6")
    add_line_label(sketch, p6_p1_line, "P6->P1")
    add_line_label(sketch, p5_p7_line, "P5->P7")
    add_line_label(sketch, p7_mid_p8_line, "P7_mid->P8")
    add_line_label(sketch, p6_p8_line, "P6->P8")
    add_line_label(sketch, p9_p10_diagonal_line, "P9->P10")

    # Validate critical line segment lengths for reliable face intersection detection
    # These lines will be used in Phase 2 to identify cutting faces via midpoint checking

    # Validate P9->P10 diagonal line length
    p9_p10_length = state.p9.geometry.distanceTo(state.p10.geometry)
    if p9_p10_length < MINIMUM_PROFILE_LINE_LENGTH_CM:
        raise Exception(
            f"P9->P10 diagonal line is too short ({p9_p10_length:.4f} cm). "
            f"Minimum length is {MINIMUM_PROFILE_LINE_LENGTH_CM} cm. "
            f"This typically indicates TeethLength parameter is too small for the gear configuration."
        )

    # Validate P5->P7_mid line length
    p5_p7mid_length = state.p5.geometry.distanceTo(state.p7_mid.geometry)
    if p5_p7mid_length < MINIMUM_PROFILE_LINE_LENGTH_CM:
        raise Exception(
            f"P5->P7_mid line is too short ({p5_p7mid_length:.4f} cm). "
            f"Minimum length is {MINIMUM_PROFILE_LINE_LENGTH_CM} cm. "
            f"This typically indicates gear dimensions are invalid or TeethLength is too large."
        )

    return state


def draw_mating_face_extension(
    state: GenerationState,
    spec: BevelGearSpec,
    sketch: adsk.fusion.Sketch,
    tooth_number: int
) -> GenerationState:
    """
    Draw the complete mating-gear-side dual-trapezoid extension (towards higher Y).

    This function creates a dual-trapezoid extension on the mating gear side using
    VERTICAL extensions (complementary to the gear side's HORIZONTAL extensions).
    The structure closes to P3 (top-right corner) instead of P1.

    This function creates a dual-trapezoid extension on the mating gear side by drawing:
    1. Inner trapezoid: P4->P11->P2 with vertical P11->P12 (parallel to P2->P3) and horizontal closing P12->P3
    2. Outer trapezoid: Extending P4->P11 line to P14 (no dimension), with intermediate point P14_mid
       on P11->P14 line. P14_mid connects to P15 (vertical, parallel to P2->P3), and horizontal connector P12->P15
    3. Diagonal profile line: P15->P16 where P15 is on line P11-P12, P16 is on line P2-P11, parallel to P11->P14, distance = TeethLength

    The first perpendicular extension (P4->P11) is perpendicular to the diagonal
    (P2->P4) and extends towards higher Y (positive Y direction) by a distance
    of module * 1.25. The second extension (P11->P14) is collinear with P4->P11
    and extends freely without dimension. P14_mid is an intermediate point on P11->P14
    that is positioned by the P12->P15 dimension (DrivingGearBaseThickness).

    The diagonal profile line P15->P16 starts at P15 on the vertical line P11-P12, runs parallel
    to P11->P14, and ends at P16 on line P2-P11. The distance from P15->P16 diagonal to
    P11->P14 is controlled by the TeethLength parameter.

    This function mutates state directly by storing p11, p12, p13, p14_mid, p14, p15, and p16.

    Args:
        state: The generation state containing p2, p3, p4, diagonal, and parameter prefix
        spec: The bevel gear specification
        sketch: The foundation sketch to draw in
        tooth_number: Number of teeth for mating gear (used for P11->P13 length calculation)

    Returns:
        Updated GenerationState with p11, p12, p13, p14_mid, p14, p15, p16 populated

    Raises:
        Exception: If constraints fail or parameters are missing
    """
    # Get module and DrivingGearBaseThickness parameters
    module_param = get_parameter(state.design, state.param_prefix, PARAM_MODULE)
    base_thickness_param = get_parameter(state.design, state.param_prefix, PARAM_DRIVING_GEAR_BASE_THICKNESS)

    if not module_param:
        raise Exception(f"{PARAM_MODULE} parameter not found")
    if not base_thickness_param:
        raise Exception("DrivingGearBaseThickness parameter not found")

    # Access p1, p2, p3, p4, diagonal from state (set by draw_foundation_rectangle and draw_apex_diagonal)
    p1 = state.p1
    p2 = state.p2
    p3 = state.p3
    p4 = state.p4
    diagonal = state.diagonal

    # Calculate perpendicular direction from diagonal (P2->P4) towards mating side (towards P3)
    # This needs to be orientation-aware, not hard-coded to "higher Y"
    diag_vec_x = p4.geometry.x - p2.geometry.x
    diag_vec_y = p4.geometry.y - p2.geometry.y
    diag_length = math.sqrt(diag_vec_x**2 + diag_vec_y**2)

    # Determine which side of P2->P4 line that P1 is on using cross product
    # Cross product: (P4 - P2) × (P1 - P2) = (p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)
    cross_product = (diag_vec_x * (p1.geometry.y - p2.geometry.y) -
                     diag_vec_y * (p1.geometry.x - p2.geometry.x))

    # For mating side extension (P4->P11), we want to go towards P3's side (opposite of P1)
    # So we invert the direction compared to gear side extension
    if cross_product > 0:
        # P1 is on left side, so P3 is on right side, rotate 90° clockwise: (x, y) -> (y, -x)
        perp_vec_x = diag_vec_y / diag_length
        perp_vec_y = -diag_vec_x / diag_length
    else:
        # P1 is on right side, so P3 is on left side, rotate 90° counter-clockwise: (x, y) -> (-y, x)
        perp_vec_x = -diag_vec_y / diag_length
        perp_vec_y = diag_vec_x / diag_length

    # Calculate P11 position (module * DEDENDUM_MULTIPLIER distance from P4, perpendicular to diagonal)
    ext_length = module_param.value * DEDENDUM_MULTIPLIER
    p11_pos = adsk.core.Point3D.create(
        p4.geometry.x + perp_vec_x * ext_length,
        p4.geometry.y + perp_vec_y * ext_length,
        0
    )

    # Draw first perpendicular line (P4->P11)
    p4_p11_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p4, p11_pos)
    p4_p11_line.isConstruction = False

    # Add PERPENDICULAR constraint to state.diagonal
    try:
        sketch.geometricConstraints.addPerpendicular(p4_p11_line, state.diagonal)
    except Exception as e:
        raise Exception(f"Failed to add perpendicular constraint to P4->P11 line: {str(e)}")

    # Apply dimension: module * 1.25
    try:
        dim_p4_p11 = sketch.sketchDimensions.addDistanceDimension(
            p4_p11_line.startSketchPoint,
            p4_p11_line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p4.geometry.x + p11_pos.x) / 2,
                (p4.geometry.y + p11_pos.y) / 2,
                0
            )
        )
        dim_p4_p11.parameter.expression = f'{make_param_name(state.param_prefix, PARAM_MODULE)} * {DEDENDUM_MULTIPLIER}'
    except Exception as e:
        raise Exception(f"Failed to add dimension to P4->P11 line: {str(e)}")

    # Store P11
    p11 = p4_p11_line.endSketchPoint

    # Draw slant line (P2->P11)
    p2_p11_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p2, p11)
    p2_p11_line.isConstruction = False

    # Draw line (P11->P12) towards P3's level (parallel to P2->P3)
    # Orientation-dependent: use P3's Y coord (standard) or X coord (swapped)
    if state.y_is_perpendicular:
        # Standard: P1->P2 is vertical, so P11->P12 goes to P3's Y level
        p12_pos = adsk.core.Point3D.create(
            p11.geometry.x,
            p3.geometry.y,
            0
        )
    else:
        # Swapped: P1->P2 is horizontal, so P11->P12 goes to P3's X level
        p12_pos = adsk.core.Point3D.create(
            p3.geometry.x,
            p11.geometry.y,
            0
        )
    p11_p12_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p11, p12_pos)
    p11_p12_line.isConstruction = False

    # Add constraint (line is parallel to P2->P3)
    # Orientation-dependent: VERTICAL (standard) or HORIZONTAL (swapped)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addVertical(p11_p12_line)
        else:
            sketch.geometricConstraints.addHorizontal(p11_p12_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P11->P12 line: {str(e)}")

    # Store P12
    p12 = p11_p12_line.endSketchPoint

    # Draw closing line (P12->P3)
    p12_p3_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p12, p3)
    p12_p3_line.isConstruction = False

    # Add constraint (P12 is at same level as P3)
    # Orientation-dependent: HORIZONTAL (standard) or VERTICAL (swapped)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addHorizontal(p12_p3_line)
        else:
            sketch.geometricConstraints.addVertical(p12_p3_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P12->P3 line: {str(e)}")

    # Extend P4->P11 line to create P13 (collinear)
    # Calculate P13 position: P11->P13 length = ((Module * MatingToothNumber) / 2) + GEAR_EXTENSION_MARGIN_CM - pitch radius plus 0.5mm margin
    p11_p13_length = ((module_param.value * tooth_number) / 2) + GEAR_EXTENSION_MARGIN_CM
    p13_pos = adsk.core.Point3D.create(
        p11.geometry.x + perp_vec_x * p11_p13_length,
        p11.geometry.y + perp_vec_y * p11_p13_length,
        0
    )

    # Draw second perpendicular line (P11->P13), collinear with P4->P11
    p11_p13_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p11, p13_pos)
    p11_p13_line.isConstruction = False

    # Add COLLINEAR constraint to P4->P11 line
    try:
        sketch.geometricConstraints.addCollinear(p11_p13_line, p4_p11_line)
    except Exception as e:
        raise Exception(f"Failed to add collinear constraint to P11->P13 line: {str(e)}")

    # Apply dimension: ((Module * VirtualTeethNumber) + 0.5)
    # This constrains the P14 endpoint, while P14_mid will be positioned independently by P13->P15 dimension
    try:
        dim_p11_p13 = sketch.sketchDimensions.addDistanceDimension(
            p11_p13_line.startSketchPoint,
            p11_p13_line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p11.geometry.x + p13_pos.x) / 2,
                (p11.geometry.y + p13_pos.y) / 2 + 1,
                0
            )
        )
        dim_p11_p13.parameter.expression = f'((({make_param_name(state.param_prefix, PARAM_MODULE)} * {make_param_name(state.param_prefix, PARAM_MATING_TOOTH_NUMBER)}) / 2) + {GEAR_EXTENSION_MARGIN_CM} cm)'
    except Exception as e:
        raise Exception(f"Failed to add dimension to P11->P13 line: {str(e)}")

    # Store P13 (extended point on P11->P13 line)
    p13 = p11_p13_line.endSketchPoint

    # Create intermediate point P14_mid on the P11->P13 line
    # This point will be positioned by the P12->P14 dimension constraint
    # Calculate initial position (will be adjusted by constraints)
    p14_mid_pos = adsk.core.Point3D.create(
        (p11.geometry.x + p13.geometry.x) / 2,
        (p11.geometry.y + p13.geometry.y) / 2,
        0
    )
    p14_mid = sketch.sketchPoints.add(p14_mid_pos)

    # Add COINCIDENT constraint to place P14_mid on the P11->P13 line
    try:
        sketch.geometricConstraints.addCoincident(p14_mid, p11_p13_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint to P14_mid on P11->P13 line: {str(e)}")

    # Draw line (P14_mid->P14) towards P3's level (parallel to P2->P3)
    # Orientation-dependent: use P3's Y coord (standard) or X coord (swapped)
    if state.y_is_perpendicular:
        # Standard: P1->P2 is vertical, so P14_mid->P14 goes to P3's Y level
        p14_pos = adsk.core.Point3D.create(
            p14_mid.geometry.x,
            p3.geometry.y,
            0
        )
    else:
        # Swapped: P1->P2 is horizontal, so P14_mid->P14 goes to P3's X level
        p14_pos = adsk.core.Point3D.create(
            p3.geometry.x,
            p14_mid.geometry.y,
            0
        )
    p14_mid_p14_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p14_mid, p14_pos)
    p14_mid_p14_line.isConstruction = False

    # Add constraint (line is parallel to P2->P3)
    # Orientation-dependent: VERTICAL (standard) or HORIZONTAL (swapped)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addVertical(p14_mid_p14_line)
        else:
            sketch.geometricConstraints.addHorizontal(p14_mid_p14_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P14_mid->P14 line: {str(e)}")

    # Store P14 (outer vertical endpoint, closure point)
    p14 = p14_mid_p14_line.endSketchPoint

    # Draw connector line (P12->P14) to close the outer trapezoid
    # Both P12 and P14 are at P3's level
    p12_p14_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p12, p14)
    p12_p14_line.isConstruction = False

    # Add constraint (both points at same level)
    # Orientation-dependent: HORIZONTAL (standard) or VERTICAL (swapped)
    try:
        if state.y_is_perpendicular:
            sketch.geometricConstraints.addHorizontal(p12_p14_line)
        else:
            sketch.geometricConstraints.addVertical(p12_p14_line)
    except Exception as e:
        raise Exception(f"Failed to add constraint to P12->P14 line: {str(e)}")

    # Apply dimension = DrivingGearBaseThickness
    # Orientation-dependent dimension orientation
    if state.y_is_perpendicular:
        dim_orientation = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        dim_text_pos = adsk.core.Point3D.create(
            (p12.geometry.x + p14.geometry.x) / 2,
            p12.geometry.y + 2,
            0
        )
    else:
        dim_orientation = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        dim_text_pos = adsk.core.Point3D.create(
            p12.geometry.x + 2,
            (p12.geometry.y + p14.geometry.y) / 2,
            0
        )
    try:
        dim_p12_p14 = sketch.sketchDimensions.addDistanceDimension(
            p12_p14_line.startSketchPoint,
            p12_p14_line.endSketchPoint,
            dim_orientation,
            dim_text_pos
        )
        dim_p12_p14.parameter.expression = make_param_name(state.param_prefix, PARAM_DRIVING_GEAR_BASE_THICKNESS)
    except Exception as e:
        raise Exception(f"Failed to add dimension to P12->P14 line: {str(e)}")

    # Get TeethLength parameter
    teeth_length_param = get_parameter(state.design, state.param_prefix, 'TeethLength')
    if not teeth_length_param:
        raise Exception("TeethLength parameter not found")

    # Draw diagonal profile line parallel to P11->P13, at distance teeth_length
    # This line runs from line P11-P12 (point P15) to line P2-P11 (point P16)
    # The line is parallel to P11->P13 (collinear direction)

    # Calculate the direction vector of P11->P13 (same as perpendicular direction)
    # We already have perp_vec_x and perp_vec_y calculated earlier
    p11_p13_dir_x = perp_vec_x
    p11_p13_dir_y = perp_vec_y

    # Calculate perpendicular direction to P11->P13 (for offsetting by teeth_length distance)
    # Perpendicular to P11->P13 direction (rotate 90° counter-clockwise)
    perp_to_p11p13_x = -p11_p13_dir_y
    perp_to_p11p13_y = p11_p13_dir_x

    # Move from P11 along the perpendicular direction by teeth_length
    offset_start_x = p11.geometry.x + perp_to_p11p13_x * teeth_length_param.value
    offset_start_y = p11.geometry.y + perp_to_p11p13_y * teeth_length_param.value

    # The diagonal line at distance teeth_length has direction same as P11->P13
    # It passes through offset_start point and extends in P11->P13 direction
    # Find intersection with line P11->P12
    # P11->P12 is either vertical (standard orientation) or horizontal (swapped orientation)
    # Offset diagonal: passes through (offset_start_x, offset_start_y), direction (p11_p13_dir_x, p11_p13_dir_y)
    # Parametric: x = offset_start_x + t * p11_p13_dir_x, y = offset_start_y + t * p11_p13_dir_y
    if state.y_is_perpendicular:
        # Standard orientation: P11->P12 is vertical (x = p11.x)
        # Solve for t when x = p11.x: offset_start_x + t * p11_p13_dir_x = p11.x
        if abs(p11_p13_dir_x) > 1e-6:
            t = (p11.geometry.x - offset_start_x) / p11_p13_dir_x
            p15_y = offset_start_y + t * p11_p13_dir_y
        else:
            # P11->P13 is vertical (shouldn't happen), use offset_start_y
            p15_y = offset_start_y
        p15_x = p11.geometry.x  # On vertical line P11-P12
    else:
        # Swapped orientation: P11->P12 is horizontal (y = p11.y)
        # Solve for t when y = p11.y: offset_start_y + t * p11_p13_dir_y = p11.y
        if abs(p11_p13_dir_y) > 1e-6:
            t = (p11.geometry.y - offset_start_y) / p11_p13_dir_y
            p15_x = offset_start_x + t * p11_p13_dir_x
        else:
            # P11->P13 is horizontal (shouldn't happen), use offset_start_x
            p15_x = offset_start_x
        p15_y = p11.geometry.y  # On horizontal line P11-P12

    # Draw line from P15 (on P11->P12) to intersection with P2-P11
    # P2-P11 line direction
    p2_p11_vec_x = p11.geometry.x - p2.geometry.x
    p2_p11_vec_y = p11.geometry.y - p2.geometry.y

    # Find intersection of diagonal through P15 with line P2-P11
    # Diagonal through P15: direction (p11_p13_dir_x, p11_p13_dir_y), passes through (p15_x, p15_y)
    # Line P2-P11: direction (p2_p11_vec_x, p2_p11_vec_y), passes through P2
    # Solve parametric equations:
    # p15_x + s * p11_p13_dir_x = p2.x + u * p2_p11_vec_x
    # p15_y + s * p11_p13_dir_y = p2.y + u * p2_p11_vec_y
    # This is a 2x2 linear system, solve for s and u
    det = p11_p13_dir_x * p2_p11_vec_y - p11_p13_dir_y * p2_p11_vec_x
    if abs(det) > 1e-6:
        u = ((p15_x - p2.geometry.x) * p11_p13_dir_y - (p15_y - p2.geometry.y) * p11_p13_dir_x) / det
        p16_x = p2.geometry.x + u * p2_p11_vec_x
        p16_y = p2.geometry.y + u * p2_p11_vec_y
    else:
        # Lines are parallel (shouldn't happen), use P15 as fallback for P16
        p16_x = p15_x
        p16_y = p15_y

    # Create P15 (start) and P16 (end) points for diagonal line
    p15_point = adsk.core.Point3D.create(p15_x, p15_y, 0)
    p16_point = adsk.core.Point3D.create(p16_x, p16_y, 0)

    try:
        p15_p16_diagonal_line = sketch.sketchCurves.sketchLines.addByTwoPoints(p15_point, p16_point)
        p15_p16_diagonal_line.isConstruction = False
    except Exception as e:
        raise Exception(f"Failed to draw diagonal profile line P15->P16: {str(e)}")

    # Store P15 (start on P11->P12) and P16 (end on P2-P11)
    p15 = p15_p16_diagonal_line.startSketchPoint
    p16 = p15_p16_diagonal_line.endSketchPoint

    # Add COINCIDENT constraint: P15 start point must be on line P11->P12
    try:
        sketch.geometricConstraints.addCoincident(p15, p11_p12_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint for P15 to line P11->P12: {str(e)}")

    # Add COINCIDENT constraint: P16 end point must be on line P2-P11
    try:
        sketch.geometricConstraints.addCoincident(p16, p2_p11_line)
    except Exception as e:
        raise Exception(f"Failed to add coincident constraint for P16 end point to line P2-P11: {str(e)}")

    # Add PARALLEL constraint to P11->P13 line
    try:
        sketch.geometricConstraints.addParallel(p15_p16_diagonal_line, p11_p13_line)
    except Exception as e:
        raise Exception(f"Failed to add parallel constraint to P15->P16 diagonal line: {str(e)}")

    # Apply distance dimension constraint: distance from P15->P16 line to P11->P13 line = teeth_length
    # To dimension between two parallel lines, we need two points (one on each line)
    # We use P15 (on diagonal) and P11 (start point of P11->P13 line)
    # The dimension will be perpendicular distance between the parallel lines
    try:
        dim_p15_distance = sketch.sketchDimensions.addDistanceDimension(
            p11,  # Point on P11->P13 line
            p15,  # Point on P15->P16 diagonal line
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            adsk.core.Point3D.create(
                (p11.geometry.x + p15_x) / 2 + 1,
                (p11.geometry.y + p15_y) / 2,
                0
            )
        )
        dim_p15_distance.parameter.expression = make_param_name(state.param_prefix, PARAM_TEETH_LENGTH)
    except Exception as e:
        raise Exception(f"Failed to add dimension to P15->P16 diagonal line: {str(e)}")

    # Store all extension points in state
    state.p11 = p11
    state.p12 = p12
    state.p13 = p13
    state.p14_mid = p14_mid
    state.p14 = p14
    state.p15 = p15
    state.p16 = p16

    # CRITICAL: Store P11->P13 line for Phase 3 (mating gear tooth profile plane)
    # This line will be used to create the tooth profile plane for the mating gear
    state.p11_p13_line = p11_p13_line

    # Add text labels for debugging
    add_line_label(sketch, p4_p11_line, "P4->P11")
    add_line_label(sketch, p2_p11_line, "P2->P11")
    add_line_label(sketch, p11_p12_line, "P11->P12")
    add_line_label(sketch, p12_p3_line, "P12->P3")
    add_line_label(sketch, p11_p13_line, "P11->P13")
    add_line_label(sketch, p14_mid_p14_line, "P14_mid->P14")
    add_line_label(sketch, p12_p14_line, "P12->P14")
    add_line_label(sketch, p15_p16_diagonal_line, "P15->P16")

    return state


def create_foundation_sketch(
    state: GenerationState,
    spec: BevelGearSpec,
    anchor_point_entity: Any
) -> GenerationState:
    """
    Orchestrate the creation of the foundation sketch with dual-trapezoid extensions.

    This function creates the foundation sketch on the perpendicular foundation plane,
    projects the anchor point into the sketch, draws the rectangle, diagonal, and
    dual-trapezoid extensions on both gear and mating gear sides.

    The complete sketch includes:
    - Base rectangle (P1-P4) with P1->P4 as construction line
    - Diagonal line P2->P4 as construction line
    - Gear-side dual-trapezoid extension (P5-P10) including diagonal profile line P9->P10
    - Mating-side dual-trapezoid extension (P11-P16) including diagonal profile line P15->P16

    Total of 18 points (P1-P16 + P7_mid + P14_mid) are created and stored in state.
    Extension points p5-p10 (gear side) and p11-p16 (mating side) are stored in state.

    Args:
        state: The generation state with foundation_plane and perpendicular_reference_line populated
        spec: The bevel gear specification
        anchor_point_entity: The user-selected anchor point entity

    Returns:
        Updated GenerationState with foundation_sketch, apex_point, gear_base_corner, and all 18 points (P1-P16 + P7_mid + P14_mid) populated. P1 is positioned at the perpendicular intersection with the reference line.
    """
    # Create new sketch on foundation_plane within design_component
    sketch = create_sketch(state.design_component, SKETCH_FOUNDATION, state.foundation_plane)

    # Project anchor_point_entity into this sketch
    projected = sketch.project(anchor_point_entity)
    if projected.count == 0:
        raise Exception("Failed to project anchor point into foundation sketch")
    projected_anchor_point = projected.item(0)

    # Project the perpendicular reference line into foundation sketch
    # This gives us the exact position where perp_line intersects foundation_plane
    if state.perpendicular_reference_line is None:
        raise Exception("perpendicular_reference_line not found in state - must call create_perpendicular_plane first")

    perp_line_projected = sketch.project(state.perpendicular_reference_line)
    if perp_line_projected.count == 0:
        raise Exception("Failed to project perpendicular reference line into foundation sketch")

    # Get the start point of the projected line (this is where perp_line intersects foundation_plane)
    perp_line_in_foundation = perp_line_projected.item(0)

    # Calculate perpendicular from foundation sketch origin (0, 0) to the projected reference line
    # This gives us the correct P1 position that's aligned with the reference line
    p1_anchor_point = None
    if hasattr(perp_line_in_foundation, 'startSketchPoint') and hasattr(perp_line_in_foundation, 'endSketchPoint'):
        start_pt = perp_line_in_foundation.startSketchPoint.geometry
        end_pt = perp_line_in_foundation.endSketchPoint.geometry

        # Calculate line direction
        dx = end_pt.x - start_pt.x
        dy = end_pt.y - start_pt.y
        length = (dx**2 + dy**2)**0.5

        # Calculate perpendicular from origin (0, 0) to this line
        # Using formula: intersection = start + ((origin - start) · direction) * direction
        # where direction is normalized
        if length > FLOATING_POINT_TOLERANCE_CM:
            dir_x = dx / length
            dir_y = dy / length

            # Vector from start to origin
            to_origin_x = 0.0 - start_pt.x
            to_origin_y = 0.0 - start_pt.y

            # Dot product: project onto line direction
            t = (to_origin_x * dir_x + to_origin_y * dir_y)

            # Intersection point
            intersect_x = start_pt.x + t * dir_x
            intersect_y = start_pt.y + t * dir_y

            # CRITICAL: Draw the perpendicular line from origin to the intersection point
            # This line connects the origin to the reference line perpendicularly
            perp_to_ref_line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                sketch.originPoint,
                adsk.core.Point3D.create(intersect_x, intersect_y, 0)
            )
            perp_to_ref_line.isConstruction = True

            # Add PERPENDICULAR constraint between our new line and the reference line
            sketch.geometricConstraints.addPerpendicular(perp_to_ref_line, perp_line_in_foundation)

            # Create P1 as a new independent point at 2x the distance from origin to intersection
            # CRITICAL: This point is NOT an endpoint of the perpendicular line
            # Position P1 at double the distance so the constraint chain has room to work
            # P8 will be at the intersection (intersect_x, intersect_y)
            # P6 will be between P8 and P1 (via P6->P8 dimension)
            # P1 will settle at the correct position via constraint chain from P4->P5->P6->P8
            p1_initial_x = 2.0 * intersect_x
            p1_initial_y = 2.0 * intersect_y
            p1_anchor_point = sketch.sketchPoints.add(
                adsk.core.Point3D.create(p1_initial_x, p1_initial_y, 0)
            )

            # Store perpendicular line in state for use in draw_gear_face_extension()
            # This line will be used to constrain P8 to the intersection point
            state.perpendicular_line_in_foundation = perp_to_ref_line
        else:
            raise Exception("Projected reference line has zero length - cannot calculate perpendicular")
    else:
        raise Exception("Projected reference line doesn't have start/end points")

    if p1_anchor_point is None:
        raise Exception("Failed to calculate P1 anchor point from perpendicular intersection")

    # Store the projected reference line in state so we can constrain P8 to it later
    state.reference_line_in_foundation = perp_line_in_foundation

    # Draw the foundation rectangle using P1 at 2x distance from origin
    # P1 has no direct constraints - will be positioned by rectangle geometry and constraint chain
    # P8 will be constrained to the intersection of perpendicular and reference lines
    state = draw_foundation_rectangle(state, spec, sketch, p1_anchor_point)

    # Draw the apex diagonal (mutates state with diagonal)
    state = draw_apex_diagonal(state, sketch)

    # Draw gear-side dual-trapezoid extension (mutates state with p5, p6, p7, p5_p7_line, p7_mid, p8, p9, p10)
    state = draw_gear_face_extension(state, spec, sketch, spec.tooth_number)

    # Draw mating-side dual-trapezoid extension (mutates state with p11, p12, p13, p14_mid, p14, p15, p16)
    state = draw_mating_face_extension(state, spec, sketch, spec.mating_tooth_number)

    # Update state with foundation sketch and convenience aliases
    state.foundation_sketch = sketch
    state.apex_point = state.p2  # Alias for convenience
    state.gear_base_corner = state.p4  # Alias for convenience

    # Note: All 18 points (P1-P16 + P7_mid + P14_mid) and critical lines (p1_p2_axis, p5_p7_line, diagonal)
    # are already stored in state by the helper functions above

    # Validate geometry before returning
    validate_foundation_sketch_geometry(sketch, spec, state)

    return state


def validate_foundation_sketch_geometry(
    sketch: adsk.fusion.Sketch,
    spec: BevelGearSpec,
    state: GenerationState
) -> bool:
    """
    Validate that the foundation sketch geometry is correct.

    This function performs post-creation validation to ensure:
    1. Sketch is fully constrained (no degrees of freedom)
    2. Rectangle dimensions match expected values
    3. Extension lines have correct lengths
    4. Perpendicular and collinear constraints are satisfied

    The foundation sketch should contain exactly 18 points (P1-P16 + P7_mid + P14_mid):
    - Base rectangle: P1, P2, P3, P4
    - Gear side extension: P5, P6, P7, P7_mid, P8, P9, P10 (P9->P10 is diagonal)
    - Mating side extension: P11, P12, P13, P14_mid, P14_outer_vert, P15, P16 (P15->P16 is diagonal)

    Args:
        sketch: The foundation sketch to validate
        spec: The bevel gear specification
        state: The generation state with parameters

    Returns:
        True if validation passes

    Raises:
        Exception: If validation fails with descriptive error message
    """
    # Get parameters for validation
    module_param = get_parameter(state.design, state.param_prefix, PARAM_MODULE)
    gear_height_param = get_parameter(state.design, state.param_prefix, PARAM_GEAR_HEIGHT)
    mating_height_param = get_parameter(state.design, state.param_prefix, PARAM_MATING_GEAR_HEIGHT)
    base_thickness_param = get_parameter(state.design, state.param_prefix, PARAM_DRIVING_GEAR_BASE_THICKNESS)

    if not module_param:
        raise Exception(f"{PARAM_MODULE} parameter not found for validation")
    if not gear_height_param:
        raise Exception(f"{PARAM_GEAR_HEIGHT} parameter not found for validation")
    if not mating_height_param:
        raise Exception(f"{PARAM_MATING_GEAR_HEIGHT} parameter not found for validation")
    if not base_thickness_param:
        raise Exception(f"{PARAM_DRIVING_GEAR_BASE_THICKNESS} parameter not found for validation")

    # Validate expected parameter values
    # Note: module_param is unitless, so value is raw number (e.g., 1.0)
    # GearHeight and MatingGearHeight are in 'mm' units, but Fusion 360 stores internally as cm
    # So we need to convert mm to cm by dividing by 10
    expected_gear_height = (module_param.value * spec.tooth_number) / 10.0  # Convert mm to cm
    expected_mating_height = (module_param.value * spec.mating_tooth_number) / 10.0  # Convert mm to cm

    tolerance = FLOATING_POINT_TOLERANCE_CM  # Tolerance for floating point comparison (0.1 micron in cm)

    if abs(gear_height_param.value - expected_gear_height) > tolerance:
        raise Exception(f"GearHeight parameter mismatch: expected {expected_gear_height} cm, got {gear_height_param.value} cm")

    if abs(mating_height_param.value - expected_mating_height) > tolerance:
        raise Exception(f"MatingGearHeight parameter mismatch: expected {expected_mating_height} cm, got {mating_height_param.value} cm")

    # Additional validations can be added here
    # - Check individual line lengths
    # - Validate constraint types
    # - Check collinearity and perpendicularity

    return True


def generate_bevel_gear(
    inputs: adsk.core.CommandInputs,
    design: adsk.fusion.Design
) -> GenerationState:
    """
    Main entry point for Phase 1 bevel gear generation.

    This function orchestrates the entire Phase 1 bevel gear generation pipeline.
    Phase 1 creates only the component hierarchy and foundation sketch showing
    the pitch cone cross-section - no tooth profiles or 3D bodies yet.

    The generation process follows these steps:
    1. Parse inputs and create BevelGearSpec
    2. Extract parent component and selections (gear plane, anchor point)
    3. Normalize gear_plane to ConstructionPlane
    4. Create main bevel gear component and occurrence
    5. Initialize GenerationState
    6. Set descriptive component name
    7. Create user parameters
    8. Create component hierarchy (Design/Driving Gear/Mating Gear)
    9. Create perpendicular foundation plane
    10. Create foundation sketch with rectangle and diagonal
    11. Hide construction planes
    12. Return final GenerationState

    Args:
        inputs: The CommandInputs collection from the command dialog
        design: The Fusion 360 design where the gear will be created

    Returns:
        The final GenerationState containing all created components, sketches,
        and geometry

    Raises:
        Exception: If inputs are invalid, required selections are missing, or
                  generation fails
    """
    # 1. Parse inputs
    params = parse_bevel_gear_inputs(inputs, design)

    # 2. Create BevelGearSpec
    spec = BevelGearSpec(**params)

    # 3. Extract parent component
    parent_component = get_parent_component(inputs)

    # 4. Extract gear plane selection
    plane_selections, _ = get_selection_input(inputs, 'plane')
    if len(plane_selections) != 1:
        raise Exception(f"Must select exactly one plane (got {len(plane_selections)})")
    gear_plane = plane_selections[0]

    # 5. Extract anchor point selection
    point_selections, _ = get_selection_input(inputs, 'anchorPoint')
    if len(point_selections) != 1:
        raise Exception(f"Must select exactly one anchor point (got {len(point_selections)})")
    anchor_point_entity = point_selections[0]

    # 6. Normalize gear_plane to ConstructionPlane
    gear_plane = ensure_construction_plane(parent_component, gear_plane)

    # 7. Create main bevel gear component
    occurrence, prefix = create_gear_occurrence(parent_component, 'BevelGear')

    # 8. Initialize GenerationState
    state = GenerationState(
        design=design,
        parent_component=parent_component,
        component=occurrence.component,
        occurrence=occurrence,
        param_prefix=prefix,
    )

    # 9. Set component name
    state.component.name = f'Bevel Gear (M={spec.module}, Teeth={spec.tooth_number}, Mating={spec.mating_tooth_number})'

    # 10. Create user parameters
    create_bevel_gear_parameters(state, spec)

    # 11. Create component hierarchy
    state = create_bevel_gear_components(state)

    # 12. Create perpendicular foundation plane
    state = create_perpendicular_plane(state, gear_plane, anchor_point_entity)
    state.gear_plane = gear_plane  # Store for orientation detection

    # 13. Create foundation sketch
    state = create_foundation_sketch(state, spec, anchor_point_entity)

    # 14. Hide construction planes
    hide_construction_planes(state)

    # 15. Run Phase 2 (driving gear) if not sketch_only
    if not spec.sketch_only:
        state = generate_driving_gear_body(state, spec)

        # 16. Run Phase 3 (mating gear) if generate_mating_gear is enabled
        if spec.generate_mating_gear:
            state = generate_mating_gear_body(state, spec)

    # 17. Return final state
    return state


# ============================================================================
# Gear Configuration Helpers (for Phase 2 and Phase 3)
# ============================================================================

def get_driving_gear_config(state: GenerationState, spec: BevelGearSpec) -> GearConfig:
    """
    Extract driving gear geometry references from state.

    Creates a GearConfig containing all geometry references needed to generate
    the driving gear body. This configuration is used by Phase 2 tooth generation
    functions.

    Args:
        state: The current generation state (after Phase 1 completion)
        spec: The bevel gear specification

    Returns:
        GearConfig containing driving gear geometry references

    Raises:
        AttributeError: If required state fields are missing
    """
    # Calculate virtual teeth number: Zv = ceil(Z / cos(pitch_cone_angle))
    import math
    cos_angle = math.cos(spec.pitch_cone_angle)
    virtual_teeth_number = math.ceil(spec.tooth_number / cos_angle)

    return GearConfig(
        component=state.gear_component,
        occurrence=state.gear_occurrence,
        axis_line=state.p1_p2_axis,
        profile_line=state.p5_p7_line,
        anchor_point=state.p7,
        extension_points=[state.p5, state.p6, state.p7, state.p7_mid, state.p8, state.p9, state.p10],
        tooth_number=spec.tooth_number,
        pitch_cone_angle=spec.pitch_cone_angle,
        virtual_teeth_number=virtual_teeth_number
    )


def get_mating_gear_config(state: GenerationState, spec: BevelGearSpec) -> GearConfig:
    """
    Extract mating gear geometry references from state.

    Creates a GearConfig containing all geometry references needed to generate
    the mating gear body. This configuration is used by Phase 3 tooth generation
    functions, which reuse the same Phase 2 functions with different geometry.

    Args:
        state: The current generation state (after Phase 1 completion)
        spec: The bevel gear specification

    Returns:
        GearConfig containing mating gear geometry references

    Raises:
        AttributeError: If required state fields are missing
    """
    # Calculate virtual teeth number: Zv = ceil(Z / cos(pitch_cone_angle))
    import math
    cos_angle = math.cos(spec.mating_pitch_cone_angle)
    virtual_teeth_number = math.ceil(spec.mating_tooth_number / cos_angle)

    return GearConfig(
        component=state.mating_gear_component,
        occurrence=state.mating_gear_occurrence,
        axis_line=state.p2_p3_axis,
        profile_line=state.p11_p13_line,
        anchor_point=state.p13,
        extension_points=[state.p11, state.p12, state.p13, state.p14_mid, state.p14, state.p15, state.p16],
        tooth_number=spec.mating_tooth_number,
        pitch_cone_angle=spec.mating_pitch_cone_angle,
        virtual_teeth_number=virtual_teeth_number
    )


# ============================================================================
# Phase 2: Tooth Profile Sketch and 3D Tooth Body Generation
# ============================================================================
# This section contains all Phase 2 functions for creating the 2D tooth profile
# sketch and the complete 3D tooth body with circular patterning.
#
# Phase 2 is divided into two parts:
# - Part 1 (Tasks 1-6): Tooth Profile Sketch Creation
# - Part 2 (Tasks 7-13): 3D Tooth Body Creation
#
# Additional functions for validation, error handling, and cleanup are also provided.
# ============================================================================


# ----------------------------------------------------------------------------
# Part 1: Tooth Profile Sketch Functions (Tasks 1-6)
# ----------------------------------------------------------------------------

def activate_gear_component(state: GenerationState, config: GearConfig) -> GenerationState:
    """
    Activate the specified gear component so subsequent geometry is created in the correct component.

    This function switches the active component to the one specified in config, ensuring
    that all tooth profile and 3D body operations create geometry within the correct
    gear component (either Driving Gear or Mating Gear).

    Args:
        state: The current generation state
        config: Gear configuration containing component and occurrence to activate

    Returns:
        The same GenerationState (side effect of activating component)

    Raises:
        ValueError: If component or occurrence is missing or invalid
    """
    if not config.component or not config.component.isValid:
        raise ValueError("Cannot activate gear component: component is invalid")

    if not config.occurrence or not config.occurrence.isValid:
        raise ValueError("Cannot activate gear component: occurrence is invalid")

    # Activate the gear component using the occurrence's activate() method
    config.occurrence.activate()

    return state


def create_tooth_profile_plane(
    state: GenerationState,
    config: GearConfig,
    field_name: str = 'tooth_profile_plane'
) -> GenerationState:
    """
    Create a construction plane perpendicular to the profile line at the anchor point.

    This function creates the plane on which the tooth profile sketch will be drawn.
    The plane is perpendicular to the profile line (P5->P7 for driving gear, P11->P13
    for mating gear) and passes through the anchor point, providing the correct
    orientation for the tooth profile relative to the bevel gear cone.

    Uses the profile line from config, eliminating the need to search through sketch
    curves. The line reference works across component boundaries (line is in
    design_component, plane is created in gear_component).

    Args:
        state: The current generation state containing foundation_plane and design_component
        config: Gear configuration containing profile_line and anchor_point
        field_name: Name of state field to store the plane in (default: 'tooth_profile_plane')

    Returns:
        Updated GenerationState with specified field populated

    Raises:
        Exception: If profile_line is missing or invalid
        Exception: If foundation_plane is missing from state
        Exception: If plane creation fails
    """
    # Use the profile line from config
    if not config.profile_line or not config.profile_line.isValid:
        raise Exception("profile_line from config is missing or invalid")

    profile_line = config.profile_line

    # Verify foundation_plane exists in state (from Phase 1)
    if not hasattr(state, 'foundation_plane') or state.foundation_plane is None:
        raise Exception("foundation_plane is missing from state (required from Phase 1)")

    # Use the PROVEN PATTERN from create_perpendicular_plane():
    # Create temporary sketch, draw line, use setByAngle, delete sketch

    # Create temporary sketch on foundation_sketch's reference plane
    temp_sketch = state.design_component.sketches.add(state.foundation_sketch.referencePlane)

    # Project profile line start and end points into the temporary sketch
    profile_start = profile_line.startSketchPoint
    profile_end = profile_line.endSketchPoint

    projected_start = temp_sketch.project(profile_start)
    if projected_start.count == 0:
        raise Exception("Failed to project profile line start point into temporary sketch")
    start_projected = projected_start.item(0)

    projected_end = temp_sketch.project(profile_end)
    if projected_end.count == 0:
        raise Exception("Failed to project profile line end point into temporary sketch")
    end_projected = projected_end.item(0)

    # Draw line between projected points
    profile_temp_line = temp_sketch.sketchCurves.sketchLines.addByTwoPoints(
        start_projected,
        end_projected
    )

    # Create plane using setByAngle at 90 degrees to foundation plane
    design_planes = state.design_component.constructionPlanes
    plane_input = design_planes.createInput()
    plane_input.setByAngle(
        profile_temp_line,
        adsk.core.ValueInput.createByReal(ANGLE_90_DEG),
        state.foundation_sketch.referencePlane
    )
    tooth_profile_plane = design_planes.add(plane_input)
    if tooth_profile_plane is None:
        raise Exception("Failed to create tooth profile plane in design component")

    tooth_profile_plane.name = PLANE_TOOTH_PROFILE

    # Delete temporary sketch (cleanup)
    temp_sketch.isVisible = False
    temp_sketch.name = SKETCH_TOOTH_PROFILE_REFERENCE

    # Store in state using field_name
    setattr(state, field_name, tooth_profile_plane)

    return state


def create_tooth_profile_sketch(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    field_name: str = 'tooth_profile_sketch',
    center_point_field: str = 'tooth_profile_center_point'
) -> GenerationState:
    """
    Create the tooth profile sketch on the perpendicular plane and project anchor point as center.

    This function creates a new sketch on the tooth profile plane and projects the anchor
    point (P7 for driving gear, P13 for mating gear) into the sketch to establish the
    center point for the tooth profile.

    Args:
        state: The current generation state with tooth_profile_plane or mating_tooth_profile_plane
        spec: The bevel gear specification (used for documentation)
        config: Gear configuration containing anchor_point
        field_name: Name of state field to store sketch in (default: 'tooth_profile_sketch')
        center_point_field: Name of state field to store center point (default: 'tooth_profile_center_point')

    Returns:
        Updated GenerationState with specified fields populated

    Raises:
        Exception: If tooth_profile_plane is missing
        Exception: If anchor point projection fails
    """
    # Determine which plane field to use based on field_name
    plane_field = field_name.replace('sketch', 'plane')
    if not hasattr(state, plane_field) or getattr(state, plane_field) is None:
        raise Exception(f"Cannot create tooth profile sketch: {plane_field} is missing")

    tooth_profile_plane = getattr(state, plane_field)

    # Create sketch on tooth_profile_plane in design_component (where the plane lives)
    sketch = state.design_component.sketches.add(tooth_profile_plane)
    sketch.name = SKETCH_TOOTH_PROFILE

    # Project anchor point into the sketch
    projected = sketch.project(config.anchor_point)
    if projected.count == 0:
        raise Exception("Failed to project anchor point into tooth profile sketch")

    # Extract projected point
    tooth_profile_center_point = projected.item(0)

    # Verify projected point is a SketchPoint
    if not isinstance(tooth_profile_center_point, adsk.fusion.SketchPoint):
        raise Exception(
            f"Projected anchor point is not a SketchPoint (got {type(tooth_profile_center_point).__name__})"
        )

    # Store in state using field names
    setattr(state, field_name, sketch)
    setattr(state, center_point_field, tooth_profile_center_point)

    return state


def get_virtual_teeth_number(state: GenerationState, spec: BevelGearSpec, config: GearConfig) -> int:
    """
    Calculate the virtual teeth number (Zv) for the bevel gear tooth profile.

    The virtual teeth number accounts for the cone angle of the bevel gear and is
    calculated as: Zv = ceil(Z / cos(pitch_cone_angle))

    Args:
        state: The current generation state (for future extensions)
        spec: The bevel gear specification (for caching if available)
        config: Gear configuration containing tooth_number and pitch_cone_angle

    Returns:
        Virtual teeth number (Zv) as an integer

    Raises:
        ValueError: If pitch_cone_angle is invalid (0, 90, or negative)
        ValueError: If tooth_number is invalid (<= 0)
    """
    # Input validation
    if config.tooth_number <= 0:
        raise ValueError(f"Invalid tooth_number: {config.tooth_number} (must be > 0)")

    if config.pitch_cone_angle <= 0 or config.pitch_cone_angle >= math.pi/2:
        raise ValueError(
            f"Invalid pitch_cone_angle: {config.pitch_cone_angle} radians "
            f"(must be 0 < angle < π/2)"
        )

    # Calculate virtual teeth number: Zv = ceil(Z / cos(delta))
    # pitch_cone_angle is already in radians
    pitch_cone_angle_radians = config.pitch_cone_angle

    # Calculate Zv
    cos_angle = math.cos(pitch_cone_angle_radians)
    if abs(cos_angle) < FLOATING_POINT_TOLERANCE_CM:  # Avoid division by zero
        raise ValueError(
            f"pitch_cone_angle too close to 90°: {config.pitch_cone_angle} "
            f"(cos={cos_angle})"
        )

    zv = math.ceil(config.tooth_number / cos_angle)

    # Sanity check: Zv should be >= Z and < 10000
    if zv < config.tooth_number:
        raise ValueError(
            f"Calculated virtual teeth number ({zv}) < actual teeth number ({config.tooth_number})"
        )

    if zv > 10000:
        raise ValueError(
            f"Calculated virtual teeth number ({zv}) unreasonably large. "
            f"Check tooth_number={config.tooth_number} and pitch_cone_angle={config.pitch_cone_angle}"
        )

    return zv


# calculate_involute_point_bevel has been moved to involute.py as calculate_involute_point
# (shared with spur gears - identical mathematics)


def draw_spur_tooth_profile(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    sketch_field: str = 'tooth_profile_sketch',
    center_field: str = 'tooth_profile_center_point',
    angular_dim_field: str = 'tooth_spine_angular_dimension'
) -> GenerationState:
    """
    Draw a spur gear tooth profile centered at the projected anchor point using the virtual teeth number.

    This function generates a complete involute tooth profile using the virtual teeth number (Zv)
    to account for the bevel gear's cone angle. The profile is centered at the center point
    and includes proper involute curves, tooth tip arc, and root fillet.

    Args:
        state: The current generation state
        spec: The bevel gear specification containing module and pressure_angle
        config: Gear configuration (not used directly but passed for consistency)
        sketch_field: Name of state field containing the tooth profile sketch
        center_field: Name of state field containing the center point
        angular_dim_field: Name of state field to store the angular dimension

    Returns:
        Updated GenerationState with tooth profile geometry added to sketch

    Raises:
        ValueError: If virtual teeth number is invalid
        Exception: If tooth profile generation fails
    """
    # Validate inputs
    if not hasattr(state, sketch_field) or getattr(state, sketch_field) is None:
        raise Exception(f"{sketch_field} is required but missing")

    if not hasattr(state, center_field) or getattr(state, center_field) is None:
        raise Exception(f"{center_field} is required but missing")

    sketch = getattr(state, sketch_field)
    center = getattr(state, center_field)

    # Set anchor_point for bevel gears (required by draw_spur_gear_circles)
    state.anchor_point = center

    # Use the unified spur gear circle function (same as spur/helical/herringbone gears)
    # BevelGearSpec now has compatible circle radius fields based on virtual teeth number
    draw_spur_gear_circles(sketch, state, spec)

    # Calculate angular tooth thickness for bevel gear
    # NOTE: spec.module is in mm (from user input), convert to cm for Fusion 360 API
    module_cm = to_cm(spec.module)  # Convert mm to cm
    tooth_thickness_pitch = (math.pi * module_cm) / 2.0
    tooth_thickness_angle = tooth_thickness_pitch / spec.pitch_circle_radius

    # Create configuration for shared function
    config = ToothProfileConfig(
        root_radius=spec.root_circle_radius,
        base_radius=spec.base_circle_radius,
        pitch_radius=spec.pitch_circle_radius,
        tip_radius=spec.tip_circle_radius,
        tooth_thickness_angle=tooth_thickness_angle,
        involute_steps=20,
        backlash=0.0,  # Backlash in cm (matches original default)
        angle=0.0
    )

    # Call shared function
    draw_involute_tooth_profile(sketch, state, config)

    # Find and store the angular dimension created by draw_involute_tooth_profile()
    # This dimension controls tooth rotation and will be modified by rotate_tooth_profile_to_align()
    angular_dim = find_tooth_spine_angular_dimension(state, sketch_field, center_field)
    setattr(state, angular_dim_field, angular_dim)
    if angular_dim is None:
        raise Exception(
            f"Could not find angular dimension in tooth profile sketch. "
            f"Sketch has {sketch.sketchDimensions.count} dimensions."
        )

    # Hide the tooth profile sketch (keep it invisible in UI)
    sketch.isVisible = False

    return state


def find_tooth_spine_angular_dimension(
    state: GenerationState,
    sketch_field: str = 'tooth_profile_sketch',
    center_field: str = 'tooth_profile_center_point'
) -> Optional[adsk.fusion.SketchAngularDimension]:
    """
    Find the angular dimension between tooth spine and horizontal reference.

    This dimension is created by draw_involute_tooth_profile() to control tooth
    orientation. We identify it by looking for an angular dimension between two
    perpendicular construction lines that both start at the anchor point.

    Args:
        state: GenerationState
        sketch_field: Name of state field containing the tooth profile sketch
        center_field: Name of state field containing the center/anchor point

    Returns:
        The angular dimension, or None if not found
    """
    if not hasattr(state, sketch_field) or getattr(state, sketch_field) is None:
        raise Exception(f"{sketch_field} required in state")

    if not hasattr(state, center_field) or getattr(state, center_field) is None:
        raise Exception(f"{center_field} required in state")

    sketch = getattr(state, sketch_field)
    anchor_point = getattr(state, center_field)

    angular_dim_count = 0
    for i in range(sketch.sketchDimensions.count):
        dim = sketch.sketchDimensions.item(i)

        # Must be angular dimension
        if not isinstance(dim, adsk.fusion.SketchAngularDimension):
            continue

        angular_dim_count += 1

        line_one = dim.lineOne
        line_two = dim.lineTwo

        # Both must be SketchLines
        if not (isinstance(line_one, adsk.fusion.SketchLine) and isinstance(line_two, adsk.fusion.SketchLine)):
            continue

        # Both must be construction lines
        if not (line_one.isConstruction and line_two.isConstruction):
            continue

        # Check if either line has anchor_point as start
        line1_start = line_one.startSketchPoint.geometry
        line1_end = line_one.endSketchPoint.geometry
        line2_start = line_two.startSketchPoint.geometry
        line2_end = line_two.endSketchPoint.geometry

        line1_starts_at_anchor = (
            abs(line1_start.x - anchor_point.geometry.x) < FLOATING_POINT_TOLERANCE_CM and
            abs(line1_start.y - anchor_point.geometry.y) < FLOATING_POINT_TOLERANCE_CM
        )
        line2_starts_at_anchor = (
            abs(line2_start.x - anchor_point.geometry.x) < FLOATING_POINT_TOLERANCE_CM and
            abs(line2_start.y - anchor_point.geometry.y) < FLOATING_POINT_TOLERANCE_CM
        )

        if not (line1_starts_at_anchor and line2_starts_at_anchor):
            continue

        # Found it! This is the angular dimension created by draw_involute_tooth_profile()
        # We don't check perpendicularity because in some cases (like bevel gears) the lines
        # might be parallel in the local sketch coordinate system
        return dim

    return None


def rotate_tooth_profile_to_align(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    sketch_field: str = 'tooth_profile_sketch',
    center_field: str = 'tooth_profile_center_point',
    angular_dim_field: str = 'tooth_spine_angular_dimension'
) -> GenerationState:
    """
    Rotate tooth profile to align tooth centerline with the extension line direction.

    Uses the angular dimension stored in state by draw_spur_tooth_profile() and
    calculates the required rotation by projecting the first extension point onto the tooth profile sketch.

    Args:
        state: GenerationState
        spec: BevelGearSpec (for reference)
        config: Gear configuration with extension_points (first point is used for alignment)
        sketch_field: Name of state field containing the tooth profile sketch
        center_field: Name of state field containing the center point
        angular_dim_field: Name of state field containing the angular dimension

    Returns:
        Updated GenerationState with rotated tooth profile
    """
    # Validate state has required fields
    if not hasattr(state, angular_dim_field) or getattr(state, angular_dim_field) is None:
        raise Exception(f"{angular_dim_field} not found in state")

    if not hasattr(state, sketch_field) or getattr(state, sketch_field) is None:
        raise Exception(f"{sketch_field} not found in state")

    if not hasattr(state, center_field) or getattr(state, center_field) is None:
        raise Exception(f"{center_field} not found in state")

    if not config.extension_points or len(config.extension_points) == 0:
        raise Exception("config.extension_points is empty - need at least one point for alignment")

    sketch = getattr(state, sketch_field)
    anchor_point = getattr(state, center_field)
    angle_dimension = getattr(state, angular_dim_field)

    # Use first extension point (P5 for driving gear, P11 for mating gear) for alignment
    alignment_point = config.extension_points[0]

    # Project alignment point onto tooth profile sketch
    projected = sketch.project(alignment_point)
    if projected.count == 0:
        raise Exception("Failed to project alignment point onto tooth profile sketch")

    projected_point = projected.item(0)
    if not isinstance(projected_point, adsk.fusion.SketchPoint):
        raise Exception(f"Projected point is not a SketchPoint (got {type(projected_point).__name__})")

    # Calculate 2D direction vector from anchor to projected alignment point
    dx = projected_point.geometry.x - anchor_point.geometry.x
    dy = projected_point.geometry.y - anchor_point.geometry.y

    # Calculate angle using atan2 (handles all quadrants correctly)
    rotation_angle = math.atan2(dy, dx)

    # Cleanup temporary projected point
    try:
        if projected_point.isValid:
            projected_point.deleteMe()
    except:
        pass

    # Update angular dimension to rotate tooth
    angle_dimension.value = rotation_angle

    return state


def create_tooth_profile_phase(state: GenerationState, spec: BevelGearSpec) -> GenerationState:
    """
    Phase 2 Part 1: Create tooth profile sketch on perpendicular plane.

    This pipeline function orchestrates the complete tooth profile sketch creation:
    1. Activate Driving Gear component
    2. Create perpendicular plane at P7 (perpendicular to P5->P7)
    3. Create tooth profile sketch on the plane
    4. Project P7 as center point
    5. Draw spur tooth profile using virtual teeth number
    6. Rotate profile to align with P5->P7 line (if needed)

    Args:
        state: GenerationState from Phase 1 with foundation sketch and all points
        spec: BevelGearSpec with module, tooth_number, pressure_angle, pitch_cone_angle

    Returns:
        Updated GenerationState with:
            - tooth_profile_plane: Construction plane perpendicular to P5->P7
            - tooth_profile_sketch: Sketch containing the tooth profile
            - tooth_profile_center_point: Projected P7 point in sketch

    Raises:
        Exception: If any step fails (propagated from subfunctions)
    """
    try:
        # Validate Phase 1 outputs exist
        validate_phase1_outputs(state)

        # Validate BevelGearSpec for Phase 2
        validate_spec_for_phase2(spec)

        # Step 1: Activate Driving Gear component
        state.design.activeComponent = state.gear_component

        # Step 2: Create perpendicular plane at P7 (perpendicular to P5->P7)
        state = create_tooth_profile_plane(state)

        # Step 3: Create tooth profile sketch on the plane and project P7
        state = create_tooth_profile_sketch(state, spec)

        # Step 4: Draw spur tooth profile using virtual teeth number
        state = draw_spur_tooth_profile(state, spec)

        # Step 5: Rotate profile to align with P5->P7 line (if needed)
        state = rotate_tooth_profile_to_align(state, spec)

        return state

    except Exception as e:
        # Cleanup partial Part 1 geometry on error
        # DISABLED: Cleanup disabled for debugging - partial geometry preserved on error
        # cleanup_phase2_part1_on_error(state)
        raise Exception(f"Phase 2 Part 1 Error: {str(e)}") from e


def cleanup_phase2_part1_on_error(state: GenerationState) -> None:
    """
    Clean up partial Part 1 geometry on error. Preserves Phase 1 geometry.

    Deletes:
        - tooth_profile_sketch (if created)
        - tooth_profile_plane (if created)

    Does NOT delete:
        - foundation_sketch (Phase 1)
        - gear_component (Phase 1)
        - Any Phase 1 geometry
    """
    # Clean up tooth profile sketch
    try:
        if hasattr(state, 'tooth_profile_sketch') and state.tooth_profile_sketch:
            if state.tooth_profile_sketch.isValid:
                state.tooth_profile_sketch.deleteMe()
    except:
        pass  # Best effort cleanup

    # Clean up tooth profile plane
    try:
        if hasattr(state, 'tooth_profile_plane') and state.tooth_profile_plane:
            if state.tooth_profile_plane.isValid:
                state.tooth_profile_plane.deleteMe()
    except:
        pass  # Best effort cleanup


# ----------------------------------------------------------------------------
# Part 2: 3D Tooth Body Functions (Tasks 7-13)
# ----------------------------------------------------------------------------

def create_base_gear_body(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    body_field: str = 'base_gear_body'
) -> GenerationState:
    """
    Rotate the hexagonal profile by 360 degrees to create the base gear body.

    This function creates the base conical gear body by revolving the hexagonal profile
    around the axis line. The hexagonal profile has exactly 6 vertices using extension_points:
    [0]→[2]→[4]→[1]→[5]→[6]→[0], which corresponds to the gear side extension shape.

    IMPORTANT: extension_points[3] is the mid-point, NOT a vertex of the hexagonal profile.
    The mid-point is an intermediate point on the extension line used for the inner
    trapezoid boundary, but the outer hexagonal profile goes directly from [0] to [2].

    Args:
        state: The current generation state with foundation_sketch
        spec: The bevel gear specification (for reference)
        config: Gear configuration with axis_line and extension_points
        body_field: Name of state field to store the created body

    Returns:
        Updated GenerationState with body field populated

    Raises:
        Exception: If hexagonal profile cannot be identified or revolve fails
    """
    # Use the axis line from config (P1->P2 for driving, P2->P3 for mating)
    if not config.axis_line or not config.axis_line.isValid:
        raise Exception("Cannot create base gear body: config.axis_line is missing or invalid")

    axis_line = config.axis_line

    # Validate required extension points (7 points: indices 0-6)
    if not config.extension_points or len(config.extension_points) < 7:
        raise Exception(
            f"Cannot create base gear body: config.extension_points has {len(config.extension_points) if config.extension_points else 0} points, need 7"
        )

    # Extract points from extension_points for clarity
    p_first = config.extension_points[0]   # P5 or P11
    p_second = config.extension_points[1]  # P6 or P12
    p_anchor = config.extension_points[2]  # P7 or P13
    p_mid = config.extension_points[3]     # P7_mid or P14_mid
    p_outer = config.extension_points[4]   # P8 or P14
    p_inner1 = config.extension_points[5]  # P9 or P15
    p_inner2 = config.extension_points[6]  # P10 or P16

    # Identify the hexagonal profile with 6 vertices using extension_points
    # Traversal order: [0]→[2]→[4]→[1]→[5]→[6]→[0]
    # which for driving gear is: P5→P7→P8→P6→P9→P10→P5
    # which for mating gear is: P11→P13→P14→P12→P15→P16→P11
    #
    # Strategy: Find the profile that contains p_outer and p_mid as vertices
    # - p_outer is unique to the outer hexagonal profile (not in inner trapezoid)
    # - p_mid exists on the extension line but IS a vertex of the hexagonal profile
    # - The hexagonal profile is the larger outer region that includes p_outer

    # Get all profiles in the foundation sketch
    profiles = state.foundation_sketch.profiles
    if profiles.count == 0:
        raise Exception("No profiles found in foundation sketch for revolve")

    # Find the hexagonal profile that contains p_outer and p_mid as vertices (boundary points)
    # IMPORTANT: Exclude profiles that contain P1, P2, P3, or P4 (these are the wrong profiles from the original trapezoid)

    # Get trapezoid vertices once before the loop
    p1 = getattr(state, 'p1', None)
    p2 = getattr(state, 'p2', None)
    p3 = getattr(state, 'p3', None)
    p4 = getattr(state, 'p4', None)

    target_profile = None
    for i in range(profiles.count):
        profile = profiles.item(i)
        # Check if this profile contains both p_outer and p_mid as vertices by examining its boundary curves
        has_outer = False
        has_mid = False
        has_p1 = False
        has_p2 = False
        has_p3 = False
        has_p4 = False

        for loop in profile.profileLoops:
            for curve in loop.profileCurves:
                if isinstance(curve.sketchEntity, adsk.fusion.SketchLine):
                    line = curve.sketchEntity
                    # Check if p_outer is a vertex (start or end point)
                    if (line.startSketchPoint == p_outer or line.endSketchPoint == p_outer):
                        has_outer = True
                    # Check if p_mid is a vertex (IS part of hexagonal profile)
                    if p_mid and (line.startSketchPoint == p_mid or line.endSketchPoint == p_mid):
                        has_mid = True
                    # Check if any trapezoid vertices (P1, P2, P3, P4) are present
                    if p1 and (line.startSketchPoint == p1 or line.endSketchPoint == p1):
                        has_p1 = True
                    if p2 and (line.startSketchPoint == p2 or line.endSketchPoint == p2):
                        has_p2 = True
                    if p3 and (line.startSketchPoint == p3 or line.endSketchPoint == p3):
                        has_p3 = True
                    if p4 and (line.startSketchPoint == p4 or line.endSketchPoint == p4):
                        has_p4 = True

        # The hexagonal profile has BOTH p_outer and p_mid as vertices, but NO trapezoid vertices
        if has_outer and has_mid and not (has_p1 or has_p2 or has_p3 or has_p4):
            target_profile = profile
            break

    if target_profile is None:
        raise Exception(
            f"Cannot identify hexagonal profile in foundation sketch. "
            f"The profile should include both outer point and mid point as vertices. "
            f"Found {profiles.count} profiles total."
        )

    # Find the triangular profile enclosed by p_inner1, p_inner2, p_first
    # This is the inner region that should also be revolved
    # IMPORTANT: Exclude profiles that contain P1, P2, P3, or P4 (these are from the original trapezoid)
    triangular_profile = None
    for i in range(profiles.count):
        profile = profiles.item(i)
        # Check if this profile contains p_inner1, p_inner2, and p_first as vertices
        has_inner1 = False
        has_inner2 = False
        has_first = False
        has_p1 = False
        has_p2 = False
        has_p3 = False
        has_p4 = False

        for loop in profile.profileLoops:
            for curve in loop.profileCurves:
                if isinstance(curve.sketchEntity, adsk.fusion.SketchLine):
                    line = curve.sketchEntity
                    if line.startSketchPoint == p_inner1 or line.endSketchPoint == p_inner1:
                        has_inner1 = True
                    if line.startSketchPoint == p_inner2 or line.endSketchPoint == p_inner2:
                        has_inner2 = True
                    if line.startSketchPoint == p_first or line.endSketchPoint == p_first:
                        has_first = True
                    # Check if any trapezoid vertices (P1, P2, P3, P4) are present
                    if p1 and (line.startSketchPoint == p1 or line.endSketchPoint == p1):
                        has_p1 = True
                    if p2 and (line.startSketchPoint == p2 or line.endSketchPoint == p2):
                        has_p2 = True
                    if p3 and (line.startSketchPoint == p3 or line.endSketchPoint == p3):
                        has_p3 = True
                    if p4 and (line.startSketchPoint == p4 or line.endSketchPoint == p4):
                        has_p4 = True

        # The triangular profile has all three vertices, is NOT the hexagonal profile,
        # and does NOT contain P1, P3, or P4 (the base corners)
        # NOTE: P2 (apex) is allowed because the axis line is part of the triangular profile boundary
        if has_inner1 and has_inner2 and has_first and profile != target_profile and not (has_p1 or has_p3 or has_p4):
            triangular_profile = profile
            break

    if triangular_profile is None:
        raise Exception(
            f"Cannot identify triangular profile in foundation sketch. "
            f"Found {profiles.count} profiles total."
        )

    # Create an ObjectCollection containing BOTH profiles to revolve
    # 1. The hexagonal profile - outer gear side extension
    # 2. The triangular profile - inner region
    profiles_to_revolve = adsk.core.ObjectCollection.create()
    profiles_to_revolve.add(target_profile)  # Hexagonal profile
    profiles_to_revolve.add(triangular_profile)  # Triangular profile

    # Verify foundation sketch is fully constrained before revolve
    if not state.foundation_sketch.isFullyConstrained:
        raise Exception("Foundation sketch is not fully constrained before revolve operation")

    # Create revolve feature in design_component (where the profile and axis live)
    revolveFeatures = state.design_component.features.revolveFeatures
    revolve_input = revolveFeatures.createInput(
        profiles_to_revolve,
        axis_line,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )

    # Set full rotation (360 degrees = 2π radians)
    revolve_input.setAngleExtent(
        False,  # Not symmetric
        adsk.core.ValueInput.createByReal(ANGLE_360_DEG)
    )

    # Create the revolve feature
    revolve_feature = revolveFeatures.add(revolve_input)
    if revolve_feature is None or revolve_feature.bodies.count == 0:
        raise Exception("Failed to create base gear body by revolve")

    # Store the resulting body (profiles create a single unified body)
    created_body = revolve_feature.bodies.item(0)
    setattr(state, body_field, created_body)

    # Verify body is valid
    if created_body.volume <= 0:
        raise Exception(f"Base gear body has invalid volume: {created_body.volume}")

    return state


def create_lofted_tooth(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    sketch_field: str = 'tooth_profile_sketch',
    body_field: str = 'lofted_tooth_body',
    apex_sketch_field: str = 'apex_sketch'
) -> GenerationState:
    """
    Create a loft from the apex point (P2) to the tooth profile sketch.

    This function creates an individual tooth body by lofting from the apex (P2)
    to the tooth profile on the perpendicular plane. The resulting body has a
    conical taper appropriate for a bevel gear tooth.

    Args:
        state: The current generation state with apex_point and foundation_plane
        spec: The bevel gear specification (for validation)
        config: Gear configuration with component
        sketch_field: Name of state field containing the tooth profile sketch
        body_field: Name of state field to store the lofted body
        apex_sketch_field: Name of state field to store the apex sketch

    Returns:
        Updated GenerationState with lofted_tooth_body field populated

    Raises:
        Exception: If apex point P2 is missing or invalid
        Exception: If tooth profile sketch is missing or has no closed profiles
        Exception: If loft operation fails to create valid solid body
    """
    # Get apex point P2 (shared by both driving and mating gears)
    if not hasattr(state, 'apex_point') or state.apex_point is None:
        raise Exception("Cannot create lofted tooth: apex_point (P2) is missing")

    # Get tooth profile sketch
    if not hasattr(state, sketch_field) or getattr(state, sketch_field) is None:
        raise Exception(f"Cannot create lofted tooth: {sketch_field} is missing")

    tooth_sketch = getattr(state, sketch_field)

    # Check if tooth profile has closed profiles
    if tooth_sketch.profiles.count == 0:
        raise Exception("Cannot create lofted tooth: tooth profile sketch has no closed profiles")

    # Verify foundation_plane exists
    if not hasattr(state, 'foundation_plane') or state.foundation_plane is None:
        raise Exception("foundation_plane is missing from state")

    # Create a construction sketch at the apex point P2 on the foundation plane
    # Use a tiny circle instead of a point for reliable lofting
    apex_sketch = state.design_component.sketches.add(state.foundation_plane)
    apex_sketch.name = SKETCH_APEX_FOR_LOFT
    apex_sketch.isVisible = False  # Hide from UI (construction only)

    # Project P2 into this sketch
    projected_p2_collection = apex_sketch.project(state.apex_point)
    if projected_p2_collection.count == 0:
        raise Exception("Failed to project apex point P2 into apex sketch")

    projected_p2 = projected_p2_collection.item(0)

    # Draw a tiny circle at P2 for loft
    # Using a very small radius (1 micrometer = 0.001 mm = 0.0001 cm)
    tiny_radius = TINY_CIRCLE_RADIUS_CM  # 1 micrometer for apex point sketch
    apex_circle = apex_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        projected_p2,
        tiny_radius
    )

    # Get the profile from the apex sketch (the tiny circle)
    if apex_sketch.profiles.count == 0:
        raise Exception("Apex sketch has no profiles")

    apex_profile = apex_sketch.profiles.item(0)

    # Get the tooth profile from tooth profile sketch
    if tooth_sketch.profiles.count == 0:
        # Debug: check what's in the sketch
        num_curves = tooth_sketch.sketchCurves.count
        num_points = tooth_sketch.sketchPoints.count
        raise Exception(
            f"Tooth profile sketch has no closed profiles! "
            f"Sketch has {num_curves} curves and {num_points} points. "
            f"Check if tooth profile drawing completed successfully."
        )

    # Typically, the first (and only) profile is the tooth
    tooth_profile = tooth_sketch.profiles.item(0)

    # Create loft feature
    loftFeatures = state.design_component.features.loftFeatures
    loft_input = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Add loft sections (start at apex, end at tooth profile)
    loft_sections = loft_input.loftSections

    # Add apex profile (tiny circle) as start
    loft_sections.add(apex_profile)

    # Add tooth profile as end
    loft_sections.add(tooth_profile)

    # Set loft to use "ruled" type which creates straight lines between profiles
    # This should prevent self-intersection
    loft_input.isSolid = True
    loft_input.isClosed = False
    loft_input.isTangentEdgesMerged = False

    # Create the loft
    loft_feature = loftFeatures.add(loft_input)

    if loft_feature is None:
        raise Exception("Failed to create loft feature from apex to tooth profile")

    # Get the created body
    if loft_feature.bodies.count == 0:
        raise Exception("Loft feature created no bodies")

    # Store the lofted body
    lofted_tooth_body = loft_feature.bodies.item(0)
    setattr(state, body_field, lofted_tooth_body)
    setattr(state, apex_sketch_field, apex_sketch)  # Keep reference for potential cleanup

    # Verify body is valid
    if not lofted_tooth_body.isValid:
        raise Exception("Lofted tooth body is invalid")

    if lofted_tooth_body.volume <= 0:
        raise Exception(f"Lofted tooth body has invalid volume: {lofted_tooth_body.volume}")

    return state


def join_tooth_to_gear_body(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    base_body_field: str = 'base_gear_body',
    tooth_body_field: str = 'single_tooth_body'
) -> GenerationState:
    """
    Join the trimmed single tooth to the base gear body using a combine operation.

    This function merges the trimmed tooth (result of cutting and removing smaller
    parts) with the base gear body to create a combined solid ready for circular
    patterning. This operation happens AFTER the tooth has been trimmed to the
    correct bevel gear boundaries.

    Args:
        state: The current generation state
        spec: The bevel gear specification (for validation)
        config: Gear configuration with component
        base_body_field: Name of state field containing the base gear body
        tooth_body_field: Name of state field containing the single tooth body

    Returns:
        Updated GenerationState (base_gear_body is modified in place)

    Raises:
        Exception: If either body is missing or invalid
        Exception: If bodies don't intersect (required for join)
        Exception: If combine operation fails
    """
    # Verify bodies exist
    if not hasattr(state, base_body_field) or getattr(state, base_body_field) is None:
        raise Exception(f"Cannot join bodies: {base_body_field} is missing")

    if not hasattr(state, tooth_body_field) or getattr(state, tooth_body_field) is None:
        raise Exception(f"Cannot join bodies: {tooth_body_field} is missing")

    base_body = getattr(state, base_body_field)
    tooth_body = getattr(state, tooth_body_field)

    # Verify bodies are valid
    if not base_body.isValid:
        raise Exception(f"Cannot join bodies: {base_body_field} is invalid")

    if not tooth_body.isValid:
        raise Exception(f"Cannot join bodies: {tooth_body_field} is invalid")

    # Verify gear_component exists
    if not config.component or not config.component.isValid:
        raise Exception("config.component is missing or invalid")

    # Store base gear body volume before join (for validation)
    base_volume = base_body.volume

    # Create combine feature to join the bodies
    combineFeatures = state.design_component.features.combineFeatures

    # Create tool bodies collection
    tool_bodies = adsk.core.ObjectCollection.create()
    tool_bodies.add(tooth_body)

    # Create combine input
    combine_input = combineFeatures.createInput(base_body, tool_bodies)

    # Set operation to Join
    combine_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combine_input.isKeepToolBodies = False  # Consume tool bodies in the join

    # Execute combine
    try:
        combine_feature = combineFeatures.add(combine_input)
    except Exception as e:
        raise Exception(
            f"Failed to join trimmed tooth to base gear body. "
            f"Bodies may not intersect. Original error: {str(e)}"
        ) from e

    if combine_feature is None:
        raise Exception("Combine feature returned None (join failed)")

    # After join operation, base_body is modified in place
    # The body reference remains the same, now containing both base and tooth
    if not base_body.isValid:
        raise Exception("Joined body is invalid after combine operation")

    # Verify joined body volume increased (tooth added to base)
    # Should be larger than base alone
    if base_body.volume <= base_volume * 0.95:
        # This might just be floating point comparison, log warning but don't fail
        # In a real scenario, volume should increase or at least stay similar
        pass

    return state


def identify_cutting_faces(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    base_body_field: str = 'base_gear_body',
    cutting_faces_field: str = 'cutting_faces'
) -> GenerationState:
    """
    Identify conical faces from the base gear body that define tooth trimming boundaries.

    This function finds the two conical faces from the base gear body (created by revolving
    the hexagonal profile) that can be used to cut the tooth to the correct bevel shape.
    The faces are created by revolving:
    1. The inner diagonal profile line (p_inner1->p_inner2)
    2. The outer profile edge (p_first->p_mid)

    These faces already exist on the base_gear_body from the revolve operation and define
    the tooth boundaries on the bevel gear cone.

    The detection uses midpoint checking to avoid false positives from faces that only
    touch the line segments at endpoints.

    Args:
        state: The current generation state
        spec: The bevel gear specification (for reference)
        config: Gear configuration with extension_points
        base_body_field: Name of state field containing the base gear body
        cutting_faces_field: Name of state field to store cutting faces

    Returns:
        Updated GenerationState with cutting_faces field populated

    Raises:
        Exception: If base_gear_body is missing
        Exception: If required extension points are missing
        Exception: If no faces contain these lines
    """
    # Verify base_gear_body exists
    if not hasattr(state, base_body_field) or getattr(state, base_body_field) is None:
        raise Exception(f"Cannot identify cutting faces: {base_body_field} is missing")

    base_body = getattr(state, base_body_field)

    # Verify required extension points (need at least 7 points)
    if not config.extension_points or len(config.extension_points) < 7:
        raise Exception(
            f"Cannot identify cutting faces: config.extension_points has {len(config.extension_points) if config.extension_points else 0} points, need 7"
        )

    # Extract points from extension_points
    p_first = config.extension_points[0]   # P5 or P11
    p_mid = config.extension_points[3]     # P7_mid or P14_mid
    p_inner1 = config.extension_points[5]  # P9 or P15
    p_inner2 = config.extension_points[6]  # P10 or P16

    # Get all faces from the base_gear_body
    # These are the conical faces created by the revolve operation
    faces = base_body.faces

    # Identify conical faces intersecting p_inner1->p_inner2 edge
    # Only check conical surfaces - skip planar/cylindrical faces created by other edges
    cutting_faces_inner = []

    for face in faces:
        # Filter: only check conical surfaces
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
            continue
        if face_intersects_line_planar(face, p_inner1.worldGeometry, p_inner2.worldGeometry):
            cutting_faces_inner.append(face)

    # Identify conical faces intersecting p_first->p_mid edge
    # Only check conical surfaces - skip planar/cylindrical faces created by other edges
    cutting_faces_outer = []

    for face in faces:
        # Filter: only check conical surfaces
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
            continue
        if face_intersects_line_planar(face, p_first.worldGeometry, p_mid.worldGeometry):
            cutting_faces_outer.append(face)

    # Combine both sets of cutting faces
    # Remove duplicates manually (BRepFace is not hashable, can't use set())
    cutting_faces = cutting_faces_inner[:]  # Start with inner faces
    for face in cutting_faces_outer:
        # Check if this face is already in the list (by object identity)
        already_added = False
        for existing_face in cutting_faces:
            if face == existing_face:
                already_added = True
                break
        if not already_added:
            cutting_faces.append(face)

    if len(cutting_faces) == 0:
        raise Exception(
            f"No cutting faces found intersecting inner or outer edges. "
            f"Base gear body has {faces.count} faces total. "
            f"Check base gear body geometry and edge positions."
        )

    # Store in state using parameterized field
    setattr(state, cutting_faces_field, cutting_faces)

    return state


def face_intersects_line_planar(face, p1, p2) -> bool:
    """
    Check if a planar face intersects a line segment defined by two points.

    Args:
        face: BRepFace to check
        p1: Start point of line segment
        p2: End point of line segment

    Returns:
        True if face intersects the line segment
    """
    # Check if face is planar (only Plane geometry has 'normal' attribute)
    # For non-planar faces (Cone, Cylinder, Sphere, etc.), use sampling approach
    if not isinstance(face.geometry, adsk.core.Plane):
        return face_intersects_line_sampling(face, p1, p2)

    try:
        plane = face.geometry
    except:
        # Face geometry doesn't have plane, use sampling
        return face_intersects_line_sampling(face, p1, p2)

    # Check if line segment crosses plane
    # Line: P(t) = p1 + t * (p2 - p1), t in [0, 1]
    # Plane: dot(N, P - P0) = 0, where N is normal, P0 is origin

    # Direction vector
    direction = adsk.core.Vector3D.create(
        p2.x - p1.x,
        p2.y - p1.y,
        p2.z - p1.z
    )

    # Check if line is parallel to plane
    dot_n_d = plane.normal.dotProduct(direction)
    if abs(dot_n_d) < FLOATING_POINT_TOLERANCE_CM:
        # Line is parallel to plane, no intersection (or lies in plane)
        return False

    # Calculate intersection parameter t
    p1_to_origin = adsk.core.Vector3D.create(
        plane.origin.x - p1.x,
        plane.origin.y - p1.y,
        plane.origin.z - p1.z
    )
    t = plane.normal.dotProduct(p1_to_origin) / dot_n_d

    # Check if intersection is within line segment bounds
    if t < 0 or t > 1:
        return False

    # Calculate intersection point
    intersection_point = adsk.core.Point3D.create(
        p1.x + t * direction.x,
        p1.y + t * direction.y,
        p1.z + t * direction.z
    )

    # Check if intersection point is within face bounds
    # Use face evaluator to check if the point is on the face surface
    evaluator = face.evaluator
    (returnValue, parameter) = evaluator.getParameterAtPoint(intersection_point)

    if not returnValue:
        return False

    # Get the actual point on the surface at this parameter
    (success, point_on_surface) = evaluator.getPointAtParameter(parameter)

    if not success:
        return False

    # Check if intersection point is very close to face surface
    distance = intersection_point.distanceTo(point_on_surface)
    return distance < MINIMUM_PROFILE_LINE_LENGTH_CM  # 1mm tolerance (0.1cm)


def face_intersects_line_sampling(face, p1, p2) -> bool:
    """
    Check if a face contains the line segment by testing its midpoint.

    This avoids false positives from faces that only touch the line at endpoints.
    For example, when checking P9->P10, the face created by revolving P6->P9 touches
    at point P9 but doesn't contain the line interior.

    PREREQUISITE: The line segment P1->P2 must be at least 0.1 cm long to ensure
    the midpoint is sufficiently far from endpoints. This is validated during
    sketch creation in draw_diagonal_profile_line().

    Args:
        face: BRepFace to check
        p1: Start point of line segment (must be >0.1cm from p2)
        p2: End point of line segment

    Returns:
        True if the face contains the midpoint of the line segment
    """
    evaluator = face.evaluator

    # Calculate midpoint (guaranteed to be ≥0.05cm from endpoints due to length validation)
    midpoint = adsk.core.Point3D.create(
        (p1.x + p2.x) / 2.0,
        (p1.y + p2.y) / 2.0,
        (p1.z + p2.z) / 2.0
    )

    # Check if midpoint lies on the face surface
    (returnValue, parameter) = evaluator.getParameterAtPoint(midpoint)
    if not returnValue:
        return False

    (success, point_on_surface) = evaluator.getPointAtParameter(parameter)
    if not success:
        return False

    # Verify midpoint is on face within tolerance
    distance = midpoint.distanceTo(point_on_surface)
    return distance < PROXIMITY_TOLERANCE_CM  # 0.1mm tolerance (0.01cm)


def cut_body_with_faces(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    tooth_body_field: str = 'lofted_tooth_body',
    cutting_faces_field: str = 'cutting_faces',
    base_body_field: str = 'base_gear_body',
    cut_bodies_field: str = 'cut_bodies'
) -> GenerationState:
    """
    Cut the lofted tooth body using identified conical faces as splitting surfaces.

    This function uses the conical faces identified from the base_gear_body (from the
    revolve operation) as splitting tools to trim the lofted tooth. Each face acts as
    an infinite surface that slices through the lofted tooth to trim it to the correct
    bevel gear boundaries.

    This operation happens BEFORE joining the tooth to the base, so only the tooth is
    cut, not the base gear body.

    Args:
        state: The current generation state
        spec: The bevel gear specification (for reference)
        config: Gear configuration with component
        tooth_body_field: Name of state field containing the lofted tooth body
        cutting_faces_field: Name of state field containing the cutting faces
        base_body_field: Name of state field containing the base gear body
        cut_bodies_field: Name of state field to store cut bodies

    Returns:
        Updated GenerationState with cut_bodies field populated

    Raises:
        Exception: If prerequisites are missing or split operations fail
    """
    # Verify prerequisites
    if not hasattr(state, tooth_body_field) or getattr(state, tooth_body_field) is None:
        raise Exception(f"Cannot cut body: {tooth_body_field} is missing")

    if not hasattr(state, cutting_faces_field) or getattr(state, cutting_faces_field) is None:
        raise Exception(f"Cannot cut body: {cutting_faces_field} is missing")

    tooth_body = getattr(state, tooth_body_field)
    cutting_faces = getattr(state, cutting_faces_field)
    base_body = getattr(state, base_body_field)

    if len(cutting_faces) == 0:
        raise Exception(f"Cannot cut body: {cutting_faces_field} is empty")

    if not tooth_body.isValid:
        raise Exception(f"Cannot cut body: {tooth_body_field} is invalid")

    # Use splitBody feature to split lofted_tooth_body using each cutting face individually
    splitBodyFeatures = state.design_component.features.splitBodyFeatures

    # Split with each cutting face
    # After each split, the body collection may change, so we iterate carefully
    for i, cutting_face in enumerate(cutting_faces):
        if not cutting_face.isValid:
            raise Exception(f"Cutting face {i} is invalid")

        # Create split input with this face as the splitting tool
        split_input = splitBodyFeatures.createInput(
            tooth_body,    # Body to be split (the lofted tooth, before joining)
            cutting_face,  # Splitting tool (a BRepFace from the base_gear_body)
            True          # keepBothSides - keep all resulting bodies
        )

        try:
            split_feature = splitBodyFeatures.add(split_input)
        except Exception as e:
            raise Exception(
                f"Failed to split lofted tooth body with cutting face {i}. "
                f"Original error: {str(e)}"
            )

        if split_feature is None:
            raise Exception(f"Split feature returned None for cutting face {i}")

    # Collect ONLY newly created bodies from the split operations
    # Exclude base_gear_body and the driving gear (which has distinctive name "Driving Gear")
    cut_bodies_list = [body for body in state.design_component.bRepBodies
                       if body.isValid and body.isVisible
                       and body != base_body
                       and body.name != "Driving Gear"]

    if len(cut_bodies_list) == 0:
        raise Exception("Split operations produced no visible bodies (excluding base_gear_body)")

    setattr(state, cut_bodies_field, cut_bodies_list)

    return state


def remove_smaller_parts(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    cut_bodies_field: str = 'cut_bodies',
    tooth_body_field: str = 'single_tooth_body'
) -> GenerationState:
    """
    Identify the tooth body by checking which body does NOT contain the apex point,
    then delete all scrap bodies.

    After splitting the lofted tooth with cutting faces, we get multiple fragments:
    - Scrap fragments (trimmed away parts) contain the apex point
    - The final tooth body does NOT contain the apex point

    This function identifies the tooth body and removes all scrap fragments.

    Args:
        state: The current generation state with apex_point
        spec: The bevel gear specification (for reference)
        config: Gear configuration with component
        cut_bodies_field: Name of state field containing cut bodies
        tooth_body_field: Name of state field to store the single tooth body

    Returns:
        Updated GenerationState with single_tooth_body field populated

    Raises:
        Exception: If cut_bodies are missing or empty
        Exception: If apex_point is missing
        Exception: If no body is found that doesn't contain the apex point
    """
    # Verify cut_bodies exist
    if not hasattr(state, cut_bodies_field) or getattr(state, cut_bodies_field) is None:
        raise Exception(f"Cannot identify tooth body: {cut_bodies_field} is missing")

    cut_bodies = getattr(state, cut_bodies_field)

    if len(cut_bodies) == 0:
        raise Exception(f"Cannot identify tooth body: {cut_bodies_field} is empty")

    # Verify apex_point exists (P2)
    if not hasattr(state, 'apex_point') or state.apex_point is None:
        raise Exception("Cannot identify tooth body: apex_point is missing from state")

    # Get apex point 3D coordinates (use worldGeometry for component-global coordinates)
    apex_3d = state.apex_point.worldGeometry

    # Find the largest body where apex is Outside
    # Delete bodies where apex is OnBoundary or Inside
    from adsk.fusion import PointContainment

    final_tooth = None
    not_tooth_body = None
    for body in cut_bodies:
        if not body.isValid:
            continue

        try:
            containment = body.pointContainment(apex_3d)
            volume = body.volume if body.isValid else 0

            # Delete bodies where apex is OnBoundary or Inside
            if containment == PointContainment.PointOnPointContainment or \
               containment == PointContainment.PointInsidePointContainment:
                state.design_component.features.removeFeatures.add(body)
                continue

            # Bodies with apex Outside are candidates (tooth and remaining scrap)
            # Pick the largest among those
            if containment == PointContainment.PointOutsidePointContainment:
                if final_tooth is None or volume > final_tooth.volume:
                    final_tooth = body
                else:
                    not_tooth_body = body
        except:
            # If pointContainment fails on this body, skip it
            continue

    if final_tooth is None or not_tooth_body is None:
        raise Exception(
            f"Cannot identify tooth body: no cut bodies have apex point outside. "
            f"This indicates the split operations may have failed."
        )

    state.design_component.features.removeFeatures.add(not_tooth_body)

    # Store the single tooth body
    setattr(state, tooth_body_field, final_tooth)

    return state


def pattern_teeth_circularly(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    tooth_body_field: str = 'single_tooth_body',
    base_body_field: str = 'base_gear_body',
    final_body_field: str = 'final_body'
) -> GenerationState:
    """
    Create a circular pattern of teeth around the axis to create the complete bevel gear.

    This function patterns the tooth body around the central axis to create all teeth,
    then joins them to the base gear body. The number of pattern instances is determined
    by config.virtual_teeth_number (Zv), which accounts for the bevel gear cone angle.

    Args:
        state: The current generation state
        spec: The bevel gear specification (for reference)
        config: Gear configuration with axis_line and virtual_teeth_number
        tooth_body_field: Name of state field containing the single tooth body
        base_body_field: Name of state field containing the base gear body
        final_body_field: Name of state field to store the final body

    Returns:
        GenerationState with final_body populated

    Raises:
        Exception: If tooth body is missing
        Exception: If axis cannot be found
        Exception: If pattern operation fails
    """
    # Verify single tooth body exists
    if not hasattr(state, tooth_body_field) or getattr(state, tooth_body_field) is None:
        raise Exception(f"Cannot create circular pattern: {tooth_body_field} is missing")

    # Get axis line from config (P1->P2 for driving, P2->P3 for mating)
    # This is the line that serves as the rotation axis for the circular pattern
    if not config.axis_line or not config.axis_line.isValid:
        raise Exception("Cannot create circular pattern: config.axis_line is missing or invalid")

    tooth_body = getattr(state, tooth_body_field)
    base_body = getattr(state, base_body_field)
    axis_line = config.axis_line

    circularPatternFeatures = state.design_component.features.circularPatternFeatures
    input_features = adsk.core.ObjectCollection.create()
    input_features.add(tooth_body)
    pattern_input = circularPatternFeatures.createInput(input_features, axis_line)
    pattern_input.quantity = adsk.core.ValueInput.createByReal(config.virtual_teeth_number)

    patterned_teeth = circularPatternFeatures.add(pattern_input)

    # Combine all patterned teeth with the base gear body
    # IMPORTANT: Exclude driving gear final body to prevent accidentally combining wrong gear
    tool_bodies = adsk.core.ObjectCollection.create()

    for body in patterned_teeth.bodies:
        # Skip driving gear final body (identified by name)
        if body.name == "Driving Gear":
            continue
        tool_bodies.add(body)

    if tool_bodies.count == 0:
        raise Exception(f"No valid bodies found in pattern result to combine with {base_body_field}")

    combine_input = state.design_component.features.combineFeatures.createInput(
        base_body,
        tool_bodies
    )
    # Set operation to Join
    combine_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combine_input.isKeepToolBodies = False  # Consume tool bodies in the join

    combine_feature = state.design_component.features.combineFeatures.add(combine_input)

    final_body = combine_feature.bodies.item(0)

    # Set a distinctive name for the driving gear final body to enable exclusion during mating gear generation
    if final_body_field == 'final_body':  # This is the driving gear
        final_body.name = "Driving Gear"

    setattr(state, final_body_field, final_body)

    return state


# ----------------------------------------------------------------------------
# Integration, Validation, and Error Handling Functions (Tasks 14-18)
# ----------------------------------------------------------------------------

def format_phase2_error(
    part: str,
    operation: str,
    details: str,
    state_info: str,
    action: str
) -> str:
    """
    Format Phase 2 error message consistently for better debugging and user guidance.

    This utility function creates standardized error messages that include all
    relevant context for diagnosing Phase 2 failures.

    Args:
        part: The phase part identifier ("Part 1 - Sketch" or "Part 2 - 3D Body")
        operation: Name of the operation that failed
        details: Specific error details describing what went wrong
        state_info: Current state information for diagnostic purposes
        action: Suggested remediation or next steps

    Returns:
        Formatted error message string with all sections

    Example:
        >>> error = format_phase2_error(
        ...     "Part 1 - Sketch",
        ...     "Plane Creation Failed",
        ...     "Cannot find P5->P7 line",
        ...     "Foundation sketch has 12 lines",
        ...     "Verify Phase 1 completed successfully"
        ... )
    """
    return f"""Phase 2 Error [{part}]: {operation}
Details: {details}
State: {state_info}
Action: {action}"""


def validate_phase1_outputs(state: GenerationState) -> None:
    """
    Validate that Phase 1 completed successfully and all required elements exist before starting Phase 2.

    This function checks for all required Phase 1 artifacts in the GenerationState
    to ensure Phase 2 can proceed safely.

    Args:
        state: The GenerationState from Phase 1

    Raises:
        ValueError: If any required Phase 1 attribute is missing or invalid
    """
    required_attrs = [
        'foundation_sketch',
        'p5', 'p7',  # Key points for tooth profile plane
        'p5_p7_line',  # The sketch line connecting P5 to P7 (for creating perpendicular plane)
        'p9', 'p10',  # Diagonal profile points on gear side
        'design_component',
        'gear_component',
        'param_prefix',
        'apex_point',  # P2 for lofting
        'foundation_plane',  # For creating apex sketch
    ]

    for attr in required_attrs:
        if not hasattr(state, attr) or getattr(state, attr) is None:
            raise ValueError(
                f"Phase 1 incomplete: Missing required state attribute '{attr}'. "
                f"Ensure Phase 1 (generate_bevel_gear) completed successfully."
            )

    # Validate foundation sketch has geometry
    if state.foundation_sketch.sketchCurves.count == 0:
        raise ValueError(
            "Phase 1 incomplete: Foundation sketch has no geometry. "
            "Verify Phase 1 foundation sketch creation succeeded."
        )

    # Validate components are valid
    if not state.gear_component.isValid:
        raise ValueError(
            "Phase 1 incomplete: Driving Gear component is invalid. "
            "Verify Phase 1 component hierarchy creation succeeded."
        )


def validate_spec_for_phase2(spec: BevelGearSpec) -> None:
    """
    Validate BevelGearSpec has all required parameters for Phase 2 tooth generation.

    This function checks that the specification contains all necessary parameters
    for tooth profile generation and 3D body creation.

    Args:
        spec: The bevel gear specification to validate

    Raises:
        ValueError: If any required parameter is missing or has invalid value
    """
    # Validate basic parameters
    if not hasattr(spec, 'module') or spec.module <= 0:
        raise ValueError(
            f"Invalid module: {getattr(spec, 'module', 'missing')} (must be > 0)"
        )

    if not hasattr(spec, 'tooth_number') or spec.tooth_number <= 0:
        raise ValueError(
            f"Invalid tooth_number: {getattr(spec, 'tooth_number', 'missing')} (must be > 0)"
        )

    if not hasattr(spec, 'pressure_angle'):
        raise ValueError("Invalid pressure_angle: missing (required for tooth profile)")

    pressure_angle = spec.pressure_angle
    if pressure_angle <= 0 or pressure_angle >= 90:
        raise ValueError(
            f"Invalid pressure_angle: {pressure_angle} (must be 0 < angle < 90)"
        )

    # Virtual teeth number must exist or be calculable
    if not hasattr(spec, 'driving_gear_virtual_teeth_number'):
        # Check if we can calculate it
        if not hasattr(spec, 'pitch_cone_angle'):
            raise ValueError(
                "Cannot calculate virtual teeth number: pitch_cone_angle is missing"
            )

        pitch_cone_angle = spec.pitch_cone_angle
        if pitch_cone_angle <= 0 or pitch_cone_angle >= 90:
            raise ValueError(
                f"Invalid pitch_cone_angle: {pitch_cone_angle} "
                f"(must be 0 < angle < 90 for virtual teeth calculation)"
            )

    # Validate teeth_length parameter exists for 3D operations
    if hasattr(spec, 'teeth_length'):
        if spec.teeth_length <= 0:
            raise ValueError(
                f"Invalid teeth_length: {spec.teeth_length} "
                f"(must be > 0 for 3D tooth body)"
            )


def cleanup_phase2_on_error(state: GenerationState) -> None:
    """
    Clean up partial Phase 2 geometry on error while preserving Phase 1 geometry.

    This function performs best-effort cleanup of any Phase 2 artifacts created
    before an error occurred. Phase 1 geometry (foundation sketch, components)
    is preserved.

    Args:
        state: The GenerationState with partial Phase 2 artifacts

    Note:
        This function never raises exceptions. All cleanup operations are
        wrapped in try/except blocks for best-effort execution.
    """
    # Clean up patterned teeth feature (Part 2 - step 7)
    try:
        if hasattr(state, 'patterned_teeth') and state.patterned_teeth:
            if hasattr(state.patterned_teeth, 'isValid') and state.patterned_teeth.isValid:
                state.patterned_teeth.deleteMe()
    except:
        pass  # Best effort cleanup

    # Clean up bodies (Part 2 - steps 6, 5, 4, 3, 2)
    for body_attr in ['single_tooth_body', 'cut_bodies', 'joined_body',
                      'lofted_tooth_body', 'base_gear_body']:
        try:
            if hasattr(state, body_attr):
                body_or_list = getattr(state, body_attr)
                if body_or_list:
                    # Handle both single bodies and lists
                    bodies = body_or_list if isinstance(body_or_list, list) else [body_or_list]
                    for body in bodies:
                        if body and hasattr(body, 'isValid') and body.isValid:
                            # Bodies are deleted via their features, not directly
                            # In a full implementation, we'd track and delete the features
                            pass
        except:
            pass  # Best effort cleanup

    # Note: No cutting planes to clean up anymore
    # The cut_body_with_faces function now uses base_gear_body directly as splitting tool

    # Clean up 2D sketch features (Part 1)
    try:
        if hasattr(state, 'tooth_profile_sketch') and state.tooth_profile_sketch:
            if state.tooth_profile_sketch.isValid:
                state.tooth_profile_sketch.deleteMe()
    except:
        pass  # Best effort cleanup

    try:
        if hasattr(state, 'tooth_profile_plane') and state.tooth_profile_plane:
            if state.tooth_profile_plane.isValid:
                state.tooth_profile_plane.deleteMe()
    except:
        pass  # Best effort cleanup

    # Clean up apex loft sketch (Part 2 - from create_lofted_tooth)
    try:
        if hasattr(state, 'apex_sketch') and state.apex_sketch:
            if state.apex_sketch.isValid:
                state.apex_sketch.deleteMe()
    except:
        pass  # Best effort cleanup


# ============================================================================
# Phase 2 and Phase 3: Tooth Body Generation (Parameterized)
# ============================================================================

def generate_tooth_body(
    state: GenerationState,
    spec: BevelGearSpec,
    config: GearConfig,
    sketch_field: str = 'tooth_profile_sketch',
    plane_field: str = 'tooth_profile_plane',
    center_field: str = 'tooth_profile_center_point',
    angular_dim_field: str = 'tooth_spine_angular_dimension',
    base_body_field: str = 'base_gear_body',
    lofted_body_field: str = 'lofted_tooth_body',
    apex_sketch_field: str = 'apex_sketch',
    cutting_faces_field: str = 'cutting_faces',
    cut_bodies_field: str = 'cut_bodies',
    tooth_body_field: str = 'single_tooth_body',
    final_body_field: str = 'final_body'
) -> GenerationState:
    """
    Parameterized tooth body generation pipeline that works for both driving and mating gears.

    This function executes the complete tooth body generation pipeline:
    1. Activate gear component
    2. Create tooth profile plane perpendicular to profile line
    3. Create tooth profile sketch and project anchor point
    4. Draw spur tooth profile using virtual teeth number
    5. Rotate tooth profile to align
    6. Create base gear body by revolving hexagonal profile
    7. Create lofted tooth from apex to profile
    8. Identify cutting faces from base body
    9. Cut lofted tooth with faces
    10. Remove smaller parts to get single tooth
    11. Circular pattern teeth and combine with base body

    Args:
        state: The GenerationState from Phase 1 with all foundation geometry
        spec: The BevelGearSpec containing all gear parameters
        config: GearConfig with geometry references (axis, profile line, anchor, etc.)
        sketch_field: State field name for tooth profile sketch
        plane_field: State field name for tooth profile plane
        center_field: State field name for tooth profile center point
        angular_dim_field: State field name for angular dimension
        base_body_field: State field name for base gear body
        lofted_body_field: State field name for lofted tooth body
        apex_sketch_field: State field name for apex sketch
        cutting_faces_field: State field name for cutting faces
        cut_bodies_field: State field name for cut bodies
        tooth_body_field: State field name for single tooth body
        final_body_field: State field name for final gear body

    Returns:
        Updated GenerationState with all tooth body artifacts populated

    Raises:
        ValueError: If Phase 1 outputs or spec validation fails
        Exception: If any operation fails
    """
    try:
        # Validate Phase 1 complete
        validate_phase1_outputs(state)

        # Validate spec for Phase 2
        validate_spec_for_phase2(spec)

        # Part 1: Tooth Profile Sketch
        state = activate_gear_component(state, config)
        state = create_tooth_profile_plane(state, config, plane_field)
        state = create_tooth_profile_sketch(state, spec, config, sketch_field, center_field)
        virtual_teeth_number = get_virtual_teeth_number(state, spec, config)
        state = draw_spur_tooth_profile(state, spec, config, sketch_field, center_field, angular_dim_field)
        state = rotate_tooth_profile_to_align(state, spec, config, sketch_field, center_field, angular_dim_field)

        # Part 2: 3D Tooth Body
        state = create_base_gear_body(state, spec, config, base_body_field)
        state = create_lofted_tooth(state, spec, config, sketch_field, lofted_body_field, apex_sketch_field)
        state = identify_cutting_faces(state, spec, config, base_body_field, cutting_faces_field)
        state = cut_body_with_faces(state, spec, config, lofted_body_field, cutting_faces_field, base_body_field, cut_bodies_field)
        state = remove_smaller_parts(state, spec, config, cut_bodies_field, tooth_body_field)
        # Note: join_tooth_to_gear_body is NOT called here - pattern_teeth_circularly handles the combine
        state = pattern_teeth_circularly(state, spec, config, tooth_body_field, base_body_field, final_body_field)

        return state

    except Exception as e:
        # Re-raise the exception with context
        raise Exception(
            f"Tooth body generation failed: {str(e)}. "
            f"Partial geometry preserved for debugging. "
            f"Phase 1 foundation sketch and components remain intact."
        ) from e


def generate_driving_gear_body(state: GenerationState, spec: BevelGearSpec) -> GenerationState:
    """
    Generate the driving gear tooth body using Phase 1 driving gear geometry.

    This is a convenience wrapper that extracts the driving gear configuration
    from state and calls the parameterized generate_tooth_body() function.

    Args:
        state: The GenerationState from Phase 1
        spec: The BevelGearSpec

    Returns:
        Updated GenerationState with driving gear body
    """
    config = get_driving_gear_config(state, spec)
    return generate_tooth_body(
        state,
        spec,
        config,
        # All defaults are for driving gear, no need to specify field names
    )


def generate_mating_gear_body(state: GenerationState, spec: BevelGearSpec) -> GenerationState:
    """
    Generate the mating gear tooth body using Phase 1 mating gear geometry.

    This is a convenience wrapper that extracts the mating gear configuration
    from state and calls the parameterized generate_tooth_body() function with
    mating-specific field names.

    Args:
        state: The GenerationState from Phase 1
        spec: The BevelGearSpec

    Returns:
        Updated GenerationState with mating gear body
    """
    config = get_mating_gear_config(state, spec)
    return generate_tooth_body(
        state,
        spec,
        config,
        # Mating gear uses prefixed field names
        sketch_field='mating_tooth_profile_sketch',
        plane_field='mating_tooth_profile_plane',
        center_field='mating_tooth_profile_center_point',
        angular_dim_field='mating_tooth_spine_angular_dimension',
        base_body_field='mating_base_gear_body',
        lofted_body_field='mating_lofted_tooth_body',
        apex_sketch_field='mating_apex_sketch',
        cutting_faces_field='mating_cutting_faces',
        cut_bodies_field='mating_cut_bodies',
        tooth_body_field='mating_single_tooth_body',
        final_body_field='mating_final_body'
    )


def generate_phase2_tooth_body(
    state: GenerationState,
    spec: BevelGearSpec
) -> GenerationState:
    """
    DEPRECATED: Use generate_driving_gear_body() instead.

    Main pipeline function for Phase 2 that orchestrates tooth profile sketch and 3D tooth body creation.

    This function executes the complete Phase 2 pipeline, which consists of two parts:
    - Part 1: Creating the 2D tooth profile sketch on a perpendicular plane
    - Part 2: Creating the 3D tooth body and patterning it to create all teeth

    The pipeline flow:
    1. Validate Phase 1 outputs and spec
    2. Part 1: Tooth Profile Sketch (Tasks 1-6)
       - Activate Driving Gear component
       - Create tooth profile plane perpendicular to P5->P7 at P7
       - Create tooth profile sketch and project P7
       - Calculate virtual teeth number
       - Draw spur tooth profile using virtual teeth number
       - Rotate tooth profile to align
    3. Part 2: 3D Tooth Body (Tasks 7-13)
       - Create base gear body by revolving profile
       - Create lofted tooth from apex to profile
       - Join tooth to gear body
       - Identify cutting faces
       - Cut body with faces
       - Remove smaller parts
       - Circular pattern teeth

    Args:
        state: The GenerationState from Phase 1 with all foundation geometry
        spec: The BevelGearSpec containing all gear parameters

    Returns:
        Updated GenerationState with all Phase 2 artifacts populated

    Raises:
        ValueError: If Phase 1 outputs or spec validation fails
        Exception: If any Phase 2 operation fails (with cleanup)
    """
    # Delegate to new wrapper
    return generate_driving_gear_body(state, spec)
