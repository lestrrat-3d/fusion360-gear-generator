"""
Shared involute tooth profile generation for gear types.

This module contains the common mathematics and geometry generation for involute
tooth profiles. Both spur and bevel gears use identical involute mathematics,
differing only in orientation, tooth number calculation (Z vs Zv), and whether
dimension expressions are used for parametric relationships.

This refactoring eliminates approximately 200 lines of code duplication between
spurgear.py and bevelgear.py by extracting the common tooth profile generation
into reusable functions.
"""

from dataclasses import dataclass
from typing import Optional
import math
import adsk.core
import adsk.fusion
from ...lib import fusion360utils as futil

from .types import GenerationState, BevelGearSpec
from .core import make_param_name


@dataclass
class ToothProfileCircles:
    """
    Circle references needed for tooth profile generation.

    These four circles define the fundamental geometry of a gear tooth:
    - Root circle: Inner boundary where teeth meet the gear body
    - Base circle: Circle from which the involute curve is generated
    - Pitch circle: Reference circle for tooth spacing and meshing
    - Tip circle: Outer boundary of the tooth tips
    """
    root_circle: adsk.fusion.SketchCircle
    base_circle: adsk.fusion.SketchCircle
    pitch_circle: adsk.fusion.SketchCircle
    tip_circle: adsk.fusion.SketchCircle


@dataclass
class ToothProfileConfig:
    """
    Configuration for involute tooth profile generation.

    This dataclass captures all parameters and options needed to generate a
    complete tooth profile, allowing both spur and bevel gears to share the
    same generation logic while specifying their gear-specific requirements.

    Fields:
        root_radius: Radius of root circle (cm, Fusion 360 API units)
        base_radius: Radius of base circle (cm, Fusion 360 API units)
        pitch_radius: Radius of pitch circle (cm, Fusion 360 API units)
        tip_radius: Radius of tip circle (cm, Fusion 360 API units)
        tooth_thickness_angle: Angular tooth thickness at pitch circle (radians)
        involute_steps: Number of points to generate along involute curve
        backlash: Backlash amount (cm, Fusion 360 API units, default 0)
        rotation_offset: Rotation to apply after involute generation (0 for spur X-axis, π for bevel Y-axis)
        center_offset: Translation to apply after rotation (None for spur, center point for bevel)
        use_dimension_expressions: True for bevel (parametric), False for spur (numeric)
        param_prefix: Parameter name prefix for expressions (required if use_dimension_expressions=True)
        tip_circle_param_name: Name of tip circle diameter parameter (for bevel dimension expressions)
        root_circle_param_name: Name of root circle diameter parameter (for bevel dimension expressions)
        add_construction_geometry: True to add spine/ribs for constraints (spur only)
        angle: Angle for circular pattern (spur only, in radians)
    """
    # Circle parameters (all in cm - Fusion 360 API units)
    root_radius: float
    base_radius: float
    pitch_radius: float
    tip_radius: float

    # Tooth geometry
    tooth_thickness_angle: float
    involute_steps: int = 15
    backlash: float = 0.0  # Backlash amount in cm

    # Orientation (spur vs bevel differences)
    rotation_offset: float = 0.0  # 0 for X-axis (spur), π for Y-axis (bevel)
    center_offset: Optional[adsk.core.Point3D] = None  # Translation after rotation (bevel only)

    # Dimension expressions (spur vs bevel differences)
    use_dimension_expressions: bool = False  # True for bevel, False for spur
    param_prefix: Optional[str] = None  # Required if use_dimension_expressions=True

    # Parameter names for dimension expressions (bevel only)
    tip_circle_param_name: Optional[str] = None
    root_circle_param_name: Optional[str] = None

    # Additional geometry (spur only)
    add_construction_geometry: bool = False  # True for spur (spine, ribs)
    angle: float = 0.0  # For spur gear circular pattern


def calculate_involute_point(
    base_radius: float,
    intersection_radius: float
) -> Optional[adsk.core.Point3D]:
    """
    Calculate a single point on an involute curve.

    The involute of a circle is the curve traced by a point on a taut string
    as it is unwound from the circle. This is the fundamental curve shape
    used for gear teeth.

    Args:
        base_radius: The radius of the base circle in cm (Fusion 360 API units)
        intersection_radius: The radius at which to calculate the involute point in cm

    Returns:
        The involute point at origin (0,0,0) in cm (Fusion 360 API units), or None if
        calculation not possible (when intersection_radius <= base_radius).
    """
    if intersection_radius <= base_radius:
        return None

    alpha = math.acos(base_radius / intersection_radius)
    if alpha <= 0:
        return None

    # Involute angle (inv α = tan α - α)
    inv_alpha = math.tan(alpha) - alpha

    # Return point in cm (Fusion 360 API units)
    return adsk.core.Point3D.create(
        intersection_radius * math.cos(inv_alpha),
        intersection_radius * math.sin(inv_alpha),
        0
    )


def create_tooth_profile_circles_for_bevel(
    sketch: adsk.fusion.Sketch,
    center_point: adsk.fusion.SketchPoint,
    state: GenerationState,
    spec: BevelGearSpec,
    virtual_teeth_number: int
) -> None:
    """
    Create construction circles with parametric dimension expressions for bevel gear.

    Creates four circles (root, base, pitch, tip) with dimension expressions that
    link to user parameters. Stores circle references and their parameter names
    in state fields for later use by the tooth profile drawing function.

    Stores in state:
        - state.root_circle, state.base_circle, state.pitch_circle, state.tip_circle
        - state.root_circle_param_name, state.base_circle_param_name, etc.

    Args:
        sketch: The tooth profile sketch to draw circles in
        center_point: The center point for all circles (projected P7)
        state: Generation state to store circle references and parameter names
        spec: Bevel gear specification
        virtual_teeth_number: The virtual teeth number (Zv) for this gear

    Returns:
        None (circles and parameter names stored in state fields)
    """
    circles = sketch.sketchCurves.sketchCircles
    dimensions = sketch.sketchDimensions

    # Calculate radii using virtual teeth number
    # NOTE: spec.module is in cm (Fusion API units), use directly without conversion
    module_cm = spec.module
    pressure_angle_rad = math.radians(spec.pressure_angle)
    pitch_radius = (module_cm * virtual_teeth_number) / 2.0
    base_radius = pitch_radius * math.cos(pressure_angle_rad)
    outer_radius = pitch_radius + module_cm
    root_radius = pitch_radius - 1.25 * module_cm

    # Pitch circle (construction) - constrained to Module * VirtualTeethNumber
    pitch_circle = circles.addByCenterRadius(center_point, pitch_radius)
    pitch_circle.isConstruction = True

    pitch_dim_point = adsk.core.Point3D.create(
        center_point.geometry.x + pitch_radius,
        center_point.geometry.y,
        0
    )
    pitch_dim = dimensions.addDiameterDimension(pitch_circle, pitch_dim_point)
    pitch_dim.parameter.expression = f'{make_param_name(state.param_prefix, "Module")} * {make_param_name(state.param_prefix, "VirtualTeethNumber")}'

    # Base circle (construction) - constrained to PitchCircleDiameter * cos(PressureAngle)
    base_circle = circles.addByCenterRadius(center_point, base_radius)
    base_circle.isConstruction = True

    base_dim_point = adsk.core.Point3D.create(
        center_point.geometry.x,
        center_point.geometry.y + base_radius,
        0
    )
    base_dim = dimensions.addDiameterDimension(base_circle, base_dim_point)
    pitch_param_name = pitch_dim.parameter.name
    base_dim.parameter.expression = f'{pitch_param_name} * cos({make_param_name(state.param_prefix, "PressureAngle")})'

    # Outer/tip circle (construction) - constrained to PitchCircleDiameter + 2 * Module
    outer_circle = circles.addByCenterRadius(center_point, outer_radius)
    outer_circle.isConstruction = True

    outer_dim_point = adsk.core.Point3D.create(
        center_point.geometry.x - outer_radius,
        center_point.geometry.y,
        0
    )
    outer_dim = dimensions.addDiameterDimension(outer_circle, outer_dim_point)
    outer_dim.parameter.expression = f'{pitch_param_name} + 2 * {make_param_name(state.param_prefix, "Module")}'

    # Root circle (construction) - constrained to PitchCircleDiameter - 2.5 * Module
    root_circle = circles.addByCenterRadius(center_point, root_radius)
    root_circle.isConstruction = True

    root_dim_point = adsk.core.Point3D.create(
        center_point.geometry.x,
        center_point.geometry.y - root_radius,
        0
    )
    root_dim = dimensions.addDiameterDimension(root_circle, root_dim_point)
    root_dim.parameter.expression = f'{pitch_param_name} - 2.5 * {make_param_name(state.param_prefix, "Module")}'

    # Store circle references in state
    state.root_circle = root_circle
    state.base_circle = base_circle
    state.pitch_circle = pitch_circle
    state.tip_circle = outer_circle

    # Store parameter names in state for later dimension expressions
    state.root_circle_param_name = root_dim.parameter.name
    state.base_circle_param_name = base_dim.parameter.name
    state.pitch_circle_param_name = pitch_param_name
    state.tip_circle_param_name = outer_dim.parameter.name


def draw_involute_tooth_profile(
    sketch: adsk.fusion.Sketch,
    state: GenerationState,
    config: ToothProfileConfig
) -> None:
    """
    Draw a complete involute tooth profile with involute curves, tip arc, and root connection.

    This is the main function for generating tooth profiles. It handles both spur and
    bevel gear profiles through configuration parameters. The function expects circles
    to already exist in state (state.root_circle, state.base_circle, etc.).

    Process:
        1. Validate circles exist in state
        2. Generate involute curve points from base to tip radius
        3. Calculate rotation angle to center tooth (based on tooth_thickness_angle)
        4. Apply rotation and optional translation to all points
        5. Draw left and right involute splines (right is mirrored)
        6. Draw tip arc connecting involute endpoints
        7. Add dimension constraints (parametric expressions for bevel, numeric for spur)
        8. Connect involutes to root circle if base_radius > root_radius
        9. Optionally add construction geometry for constraints (spur only)

    Args:
        sketch: The sketch to draw tooth profile in
        state: Generation state containing:
            - anchor_point: Center point for tooth profile
            - root_circle, base_circle, pitch_circle, tip_circle: Circle references
        config: Configuration specifying all tooth profile parameters and options

    Returns:
        None (tooth profile geometry added to sketch, state.tooth_profile_is_embedded
        updated if involute is fully within root circle)

    Raises:
        ValueError: If required circles are missing from state
        Exception: If involute point generation fails
    """
    # Step 1: Validate circles in state
    if state.root_circle is None or state.base_circle is None:
        raise ValueError(
            "Circles must exist in state before calling draw_involute_tooth_profile(). "
            "Call draw_spur_gear_circles() or create_tooth_profile_circles_for_bevel() first."
        )

    if state.pitch_circle is None or state.tip_circle is None:
        raise ValueError(
            "All four circles (root, base, pitch, tip) must exist in state."
        )

    # Get shortcuts to sketch objects
    constraints = sketch.geometricConstraints
    curves = sketch.sketchCurves
    dimensions = sketch.sketchDimensions
    points = sketch.sketchPoints

    # Use the anchor point from state
    anchor_point = state.anchor_point

    # Step 2: Generate involute points
    # The involutes must always go from the base circle to the tip circle
    involute_size = config.tip_radius - config.base_radius
    involute_points = []

    for i in range(config.involute_steps):
        intersection_radius = config.base_radius + ((involute_size / (config.involute_steps - 1)) * i)
        involute_point = calculate_involute_point(config.base_radius, intersection_radius)
        if involute_point is not None:
            involute_points.append(involute_point)
    
    for point in involute_points:
        futil.log(f'Involute Point: x={point.x}, y={point.y}')

    if len(involute_points) == 0:
        raise Exception("Failed to generate involute points - check radii values")

    # Step 3: Calculate rotation angle to center tooth
    pitch_involute_point = calculate_involute_point(config.base_radius, config.pitch_radius)
    if pitch_involute_point is None:
        raise Exception("Failed to calculate pitch involute point")

    # Use atan (not atan2) to match original
    pitch_point_angle = math.atan(pitch_involute_point.y / pitch_involute_point.x)

    # Calculate backlash angle (matches original formula)
    backlash_angle = (config.backlash / config.pitch_radius) * 0.25

    # Use original rotation formula
    rotate_angle = -((config.tooth_thickness_angle / 2) + pitch_point_angle - backlash_angle)

    # Step 4: Rotate and translate points
    # NOTE: All values are in cm (Fusion 360 API units)
    cos_angle = math.cos(rotate_angle)
    sin_angle = math.sin(rotate_angle)

    # Rotate points in-place (matching original style)
    for i in range(len(involute_points)):
        new_x = involute_points[i].x * cos_angle - involute_points[i].y * sin_angle
        new_y = involute_points[i].x * sin_angle + involute_points[i].y * cos_angle
        involute_points[i] = adsk.core.Point3D.create(new_x, new_y, 0)
    for point in involute_points:
        futil.log(f'Rotated Involute Point: x={point.x}, y={point.y}')

    # For bevel gears, apply translation after rotation
    if config.center_offset is not None:
        for i in range(len(involute_points)):
            involute_points[i] = adsk.core.Point3D.create(
                involute_points[i].x + config.center_offset.x,
                involute_points[i].y + config.center_offset.y,
                0
            )

    # Step 5: Draw involute splines
    # Left spline
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        point_collection.add(point)
    left_spline = curves.sketchFittedSplines.add(point_collection)

    # Right spline (mirrored)
    # Mirror logic depends on whether we have center_offset (bevel vs spur)
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        if config.center_offset is not None:
            # Bevel: mirror across horizontal through center
            mirrored_y = 2 * config.center_offset.y - point.y
            point_collection.add(adsk.core.Point3D.create(point.x, mirrored_y, 0))
        else:
            # Spur: simple Y negation
            point_collection.add(adsk.core.Point3D.create(point.x, -point.y, 0))
    right_spline = curves.sketchFittedSplines.add(point_collection)

    # Step 6: Draw tip arc
    # Store tooth_top_point for use in construction geometry (spur only)
    tooth_top_point = None

    if not config.use_dimension_expressions:
        # Spur gear style: simpler approach
        # config.tip_radius is in cm (Fusion 360 API units)
        tooth_top_point = points.add(adsk.core.Point3D.create(config.tip_radius, 0, 0))
        constraints.addCoincident(tooth_top_point, state.tip_circle)

        tip_arc = curves.sketchArcs.addByThreePoints(
            right_spline.endSketchPoint,
            tooth_top_point.geometry,
            left_spline.endSketchPoint
        )

        dimensions.addDiameterDimension(
            tip_arc,
            adsk.core.Point3D.create(config.tip_radius, 0, 0)
        )
    else:
        # Bevel gear style: calculate midpoint for SHORT arc with expressions
        tip_start = right_spline.endSketchPoint
        tip_end = left_spline.endSketchPoint

        # Calculate angles of endpoints
        tip_start_x = tip_start.geometry.x - config.center_offset.x
        tip_start_y = tip_start.geometry.y - config.center_offset.y
        tip_start_angle = math.atan2(tip_start_y, tip_start_x)

        tip_end_x = tip_end.geometry.x - config.center_offset.x
        tip_end_y = tip_end.geometry.y - config.center_offset.y
        tip_end_angle = math.atan2(tip_end_y, tip_end_x)

        # Calculate average angle (midpoint of SHORT arc)
        # Properly handle angle wrapping
        angle_diff = tip_end_angle - tip_start_angle
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        tip_mid_angle = tip_start_angle + angle_diff / 2.0

        # Create midpoint on the outer circle (all in cm)
        tip_mid_x = config.center_offset.x + config.tip_radius * math.cos(tip_mid_angle)
        tip_mid_y = config.center_offset.y + config.tip_radius * math.sin(tip_mid_angle)
        tip_mid_point = adsk.core.Point3D.create(tip_mid_x, tip_mid_y, 0)

        tip_arc = curves.sketchArcs.addByThreePoints(tip_start, tip_mid_point, tip_end)

        # Add coincident constraint and radius dimension with expression
        constraints.addCoincident(tip_arc.centerSketchPoint, anchor_point)

        tip_arc_dim_point = adsk.core.Point3D.create(
            config.center_offset.x + config.tip_radius,
            config.center_offset.y,
            0
        )
        tip_arc_radius_dim = dimensions.addRadialDimension(tip_arc, tip_arc_dim_point)
        tip_arc_radius_dim.parameter.expression = f'{config.tip_circle_param_name} / 2'

    # Create central spine for constraints
    if tooth_top_point is None:
        # Note from user: this should be created regardless
        # of gear type.
        raise Exception("tooth_top_point should have been created in Step 6 for spur gears")

    spine = curves.sketchLines.addByTwoPoints(anchor_point, tooth_top_point)
    spine.isConstruction = True

    # Step 7: Connect to root
    if not config.use_dimension_expressions:
        def draw_root_to_involute_line(root, root_radius, inv_line, spine, x, y):
            angle_calc = math.atan(y / x)
            point = adsk.core.Point3D.create(
                root_radius * math.cos(angle_calc),
                root_radius * math.sin(angle_calc),
                0
            )
            line_to_circle = curves.sketchLines.addByTwoPoints(point, inv_line.startSketchPoint)
            constraints.addCoincident(line_to_circle.startSketchPoint, root)

            line_to_center = curves.sketchLines.addByTwoPoints(root.centerSketchPoint, line_to_circle.startSketchPoint)
            line_to_center.isConstruction = True

            angle_dim = dimensions.addAngularDimension(
                line_to_circle,
                spine,
                root.centerSketchPoint.geometry
            )
            angle_dim.value = angle_calc

            constraints.addCollinear(line_to_center, line_to_circle)

            return line_to_circle

        # Check if first involute point is outside root circle
        first_point = involute_points[0]
        first_point_radius = math.sqrt(first_point.x ** 2 + first_point.y ** 2)

        if first_point_radius > config.root_radius:
            # Involute extends beyond root circle, connect them
            draw_root_to_involute_line(state.root_circle, config.root_radius, left_spline, spine, first_point.x, first_point.y)
            draw_root_to_involute_line(state.root_circle, config.root_radius, right_spline, spine, first_point.x, -first_point.y)
            state.tooth_profile_is_embedded = False
        else:
            # Involute is embedded within root circle
            state.tooth_profile_is_embedded = True
    else:
        # Bevel style: lines + arc with dimension expressions
        if config.base_radius > config.root_radius:
            root_start_point = right_spline.startSketchPoint
            root_end_point = left_spline.startSketchPoint

            # Calculate angles for root points
            right_start_x = root_start_point.geometry.x - config.center_offset.x
            right_start_y = root_start_point.geometry.y - config.center_offset.y
            right_start_angle = math.atan2(right_start_y, right_start_x)

            # Root point on root circle at same angle (all in cm)
            root_right_x = config.center_offset.x + config.root_radius * math.cos(right_start_angle)
            root_right_y = config.center_offset.y + config.root_radius * math.sin(right_start_angle)
            root_right_point = adsk.core.Point3D.create(root_right_x, root_right_y, 0)

            # Left side (mirrored)
            left_start_x = root_end_point.geometry.x - config.center_offset.x
            left_start_y = root_end_point.geometry.y - config.center_offset.y
            left_start_angle = math.atan2(left_start_y, left_start_x)

            root_left_x = config.center_offset.x + config.root_radius * math.cos(left_start_angle)
            root_left_y = config.center_offset.y + config.root_radius * math.sin(left_start_angle)
            root_left_point = adsk.core.Point3D.create(root_left_x, root_left_y, 0)

            # Draw lines from involute starts to root circle
            curves.sketchLines.addByTwoPoints(root_start_point, root_right_point)
            curves.sketchLines.addByTwoPoints(root_end_point, root_left_point)

            # Draw root arc using SHORT arc between the two points
            # Calculate midpoint angle properly handling angle wrapping
            angle_diff = left_start_angle - right_start_angle
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            root_mid_angle = right_start_angle + angle_diff / 2.0
            root_mid_x = config.center_offset.x + config.root_radius * math.cos(root_mid_angle)
            root_mid_y = config.center_offset.y + config.root_radius * math.sin(root_mid_angle)
            root_mid_point = adsk.core.Point3D.create(root_mid_x, root_mid_y, 0)

            root_arc = curves.sketchArcs.addByThreePoints(
                root_right_point,
                root_mid_point,
                root_left_point
            )

            # Add coincident constraint: root arc center to sketch center
            constraints.addCoincident(root_arc.centerSketchPoint, anchor_point)

            # Add radius dimension constraint to root arc with expression
            root_arc_dim_point = adsk.core.Point3D.create(
                config.center_offset.x + config.root_radius,
                config.center_offset.y,
                0
            )
            root_arc_radius_dim = dimensions.addRadialDimension(root_arc, root_arc_dim_point)
            root_arc_radius_dim.parameter.expression = f'{config.root_circle_param_name} / 2'
        else:
            # Base radius <= root radius, draw direct arc between involute starts
            # This is the case where the involute starts within the root circle
            root_start_point = right_spline.startSketchPoint
            root_end_point = left_spline.startSketchPoint

            # Calculate midpoint angle for SHORT arc
            root_start_x = root_start_point.geometry.x - config.center_offset.x
            root_start_y = root_start_point.geometry.y - config.center_offset.y
            root_start_angle = math.atan2(root_start_y, root_start_x)

            root_end_x = root_end_point.geometry.x - config.center_offset.x
            root_end_y = root_end_point.geometry.y - config.center_offset.y
            root_end_angle = math.atan2(root_end_y, root_end_x)

            # Calculate midpoint angle properly handling angle wrapping
            angle_diff = root_end_angle - root_start_angle
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            root_mid_angle = root_start_angle + angle_diff / 2.0
            root_mid_x = config.center_offset.x + config.base_radius * math.cos(root_mid_angle)
            root_mid_y = config.center_offset.y + config.base_radius * math.sin(root_mid_angle)
            root_mid_point = adsk.core.Point3D.create(root_mid_x, root_mid_y, 0)

            root_arc = curves.sketchArcs.addByThreePoints(
                root_start_point,
                root_mid_point,
                root_end_point
            )

            # Add coincident constraint: root arc center to sketch center
            constraints.addCoincident(root_arc.centerSketchPoint, anchor_point)

            # Add radius dimension constraint to root arc (uses base circle since base <= root)
            root_arc_dim_point = adsk.core.Point3D.create(
                config.center_offset.x + config.base_radius,
                config.center_offset.y,
                0
            )
            root_arc_radius_dim = dimensions.addRadialDimension(root_arc, root_arc_dim_point)
            root_arc_radius_dim.parameter.expression = f'{config.base_circle_param_name} / 2'

    # Step 8: Add construction geometry (spur only)
    if config.add_construction_geometry:
        # Handle angle constraint
        angle_dimension = None
        if config.angle == 0:
            # Tooth is horizontal (default)
            constraints.addHorizontal(spine)
        else:
            # Tooth is at an angle - create horizontal reference and angular dimension
            horizontal = curves.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create(anchor_point.geometry.x, anchor_point.geometry.y, 0),
                adsk.core.Point3D.create(tooth_top_point.geometry.x, tooth_top_point.geometry.y, 0)
            )
            horizontal.isConstruction = True
            constraints.addHorizontal(horizontal)
            constraints.addCoincident(horizontal.startSketchPoint, anchor_point)
            constraints.addCoincident(horizontal.endSketchPoint, state.tip_circle)

            angle_dimension = dimensions.addAngularDimension(
                spine,
                horizontal,
                adsk.core.Point3D.create(anchor_point.geometry.x, anchor_point.geometry.y, 0)
            )

        # Create ribs between left and right involutes for full constraint
        prev_point = anchor_point
        for i in range(len(involute_points)):
            left_point = left_spline.fitPoints.item(i)

            # Create rib connecting left and right involutes
            rib = curves.sketchLines.addByTwoPoints(
                left_point,
                right_spline.fitPoints.item(i)
            )
            rib.isConstruction = True

            # Add dimension for rib width
            dimensions.addDistanceDimension(
                rib.startSketchPoint,
                rib.endSketchPoint,
                adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                adsk.core.Point3D.create(rib.startSketchPoint.geometry.x, rib.startSketchPoint.geometry.y / 2, 0)
            )

            # Create spine point for this rib
            spine_point = points.add(adsk.core.Point3D.create(left_point.geometry.x, 0, 0))
            constraints.addCoincident(spine_point, spine)
            constraints.addMidPoint(spine_point, rib)
            if i > 0:
                constraints.addPerpendicular(spine, rib)
                # Add distance dimension along spine
                dimensions.addDistanceDimension(
                    prev_point,
                    spine_point,
                    adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                    adsk.core.Point3D.create(
                        (left_point.geometry.x - prev_point.geometry.x) / 2 + prev_point.geometry.x,
                        0,
                        0
                    )
                )
            prev_point = spine_point

        # Set the angle dimension value if we created one
        # This must be done AFTER all the lines have been drawn
        if config.angle != 0 and angle_dimension is not None:
            angle_dimension.value = config.angle
