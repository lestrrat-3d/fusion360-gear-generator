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
    Configuration for tooth profile generation (works for both spur and bevel gears).

    This unified configuration is used by draw_involute_tooth_profile() to generate
    tooth profiles for ANY gear type. The key difference between gear types is encoded
    in tooth_thickness_angle, which is calculated differently by each gear type's caller:
    - Spur gear: tooth_thickness_angle based on circular pitch
    - Bevel gear: tooth_thickness_angle based on pitch cone geometry

    All geometry is generated relative to the anchor_point passed to the function.

    Fields:
        root_radius: Radius of root circle (cm, Fusion 360 API units)
        base_radius: Radius of base circle (cm, Fusion 360 API units)
        pitch_radius: Radius of pitch circle (cm, Fusion 360 API units)
        tip_radius: Radius of tip circle (cm, Fusion 360 API units)
        tooth_thickness_angle: Angular tooth thickness at pitch circle (radians)
            - KEY DIFFERENCE: Calculated differently by spur vs bevel callers
        involute_steps: Number of points to generate along involute curve
        backlash: Backlash amount (cm, Fusion 360 API units, default 0)
        add_construction_geometry: True to add spine/ribs for constraints (optional)
        angle: Angle for circular pattern (in radians, typically used by spur gears)
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

    # Additional geometry (spur only)
    add_construction_geometry: bool = False  # True for spur (spine, ribs)
    angle: float = 0.0  # For spur gear circular pattern


def create_point_relative_to(
    anchor: adsk.fusion.SketchPoint,
    rel_x: float,
    rel_y: float
) -> adsk.core.Point3D:
    """
    Create a Point3D at coordinates relative to an anchor point.

    This helper function creates a Point3D offset from the anchor point's position.
    This is necessary because sketch geometry may be centered at the anchor_point
    rather than the sketch origin (0,0,0).

    For spur gears: anchor_point is typically at/near (0,0,0), so offset is minimal
    For bevel gears: anchor_point is at P7's projected location, so offset is required

    Args:
        anchor: The anchor SketchPoint to offset from
        rel_x: X coordinate relative to anchor (in cm, Fusion 360 API units)
        rel_y: Y coordinate relative to anchor (in cm, Fusion 360 API units)

    Returns:
        Point3D at absolute coordinates (anchor.x + rel_x, anchor.y + rel_y, 0)

    Example:
        # Create point 1.5cm to the right of and 0.5cm above anchor_point
        point = create_point_relative_to(anchor_point, 1.5, 0.5)
    """
    return adsk.core.Point3D.create(
        anchor.geometry.x + rel_x,
        anchor.geometry.y + rel_y,
        0
    )


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
    # NOTE: spec.module is in mm (from user input), convert to cm for Fusion 360 API
    module_cm = spec.module / 10.0  # Convert mm to cm
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
    Generate involute tooth profile geometry for spur or bevel gears.

    This unified function generates tooth profile geometry, constraints, and dimensions
    for ANY gear type using the same battle-tested code path. There is NO branching
    based on gear type - the same implementation works universally.

    Key principles:
    - All geometry is calculated relative to state.anchor_point
    - Both spur and bevel gears use identical code path
    - The difference between gear types is encoded in tooth_thickness_angle,
      which is calculated differently by each caller
    - Uses constraint-based positioning (coincident to circles) for robustness
    - Generates numeric dimensions (not parametric expressions)

    Algorithm:
    1. Generate involute points from base circle to tip circle
    2. Rotate points to center tooth based on tooth_thickness_angle
    3. Create left and right splines from involute points (mirrored)
    4. Create tooth_top_point and tip arc, constrain to tip circle
    5. Add diameter dimension to tip arc
    6. Create spine (construction line through tooth center)
    7. If needed: Create root connection lines, constrain to root circle
    8. If needed: Add angular dimensions to root connection
    9. If requested: Add construction geometry (ribs, etc.)

    Args:
        sketch: Fusion 360 sketch to draw in
        state: GenerationState with all required circles and anchor_point
        config: ToothProfileConfig with all parameters

    Returns:
        None (modifies sketch in place)

    Raises:
        ValueError: If required circles missing from state
        Exception: If involute generation fails
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
    # This unified function works for BOTH spur and bevel gears
    # The only difference is tooth_thickness_angle, calculated by each caller
    # NO branching based on gear type - same code path for all
    #
    # The involutes must always go from the base circle to the tip circle
    # Involute mathematics are universal - same for all gear types
    involute_size = config.tip_radius - config.base_radius
    involute_points = []

    for i in range(config.involute_steps):
        intersection_radius = config.base_radius + ((involute_size / (config.involute_steps - 1)) * i)
        involute_point = calculate_involute_point(config.base_radius, intersection_radius)
        if involute_point is not None:
            involute_points.append(involute_point)

    if len(involute_points) == 0:
        raise Exception("Failed to generate involute points - check radii values")

    # Step 3: Calculate rotation angle to center tooth
    # KEY DIFFERENCE: tooth_thickness_angle is calculated differently by each caller:
    #   - Spur: based on circular pitch
    #   - Bevel: based on pitch cone geometry
    # Once passed here, both types are treated identically
    pitch_involute_point = calculate_involute_point(config.base_radius, config.pitch_radius)
    if pitch_involute_point is None:
        raise Exception("Failed to calculate pitch involute point")

    # Use atan (not atan2) to match original battle-tested code
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

    futil.log(f"[INVOLUTE] rotate_angle = {math.degrees(rotate_angle):.2f}°")
    futil.log(f"[INVOLUTE] First point after rotation: ({involute_points[0].x:.4f}, {involute_points[0].y:.4f})")
    futil.log(f"[INVOLUTE] Last point after rotation: ({involute_points[-1].x:.4f}, {involute_points[-1].y:.4f})")

    # Step 5: Draw involute splines
    # Left spline
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        point_collection.add(point)
    left_spline = curves.sketchFittedSplines.add(point_collection)

    # Right spline (mirrored)
    # Simple Y-axis negation for mirroring (works universally for both gear types)
    # No complex bevel-specific mirroring needed - this approach works for all
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        point_collection.add(adsk.core.Point3D.create(point.x, -point.y, 0))
    right_spline = curves.sketchFittedSplines.add(point_collection)

    # Step 6: Draw tip arc
    # Create tip midpoint relative to anchor_point
    # Constraint-based positioning (coincident to tip_circle) handles final placement
    # config.tip_radius is in cm (Fusion 360 API units)
    futil.log(f"[TOOTH_TOP] Creating at ({config.tip_radius:.4f}, 0, 0)")
    tooth_top_point = points.add(adsk.core.Point3D.create(config.tip_radius, 0, 0))
    futil.log(f"[TOOTH_TOP] Before constraint: ({tooth_top_point.geometry.x:.4f}, {tooth_top_point.geometry.y:.4f})")
    constraints.addCoincident(tooth_top_point, state.tip_circle)
    futil.log(f"[TOOTH_TOP] After coincident to tip_circle: ({tooth_top_point.geometry.x:.4f}, {tooth_top_point.geometry.y:.4f})")

    # Create tip arc
    tip_arc = curves.sketchArcs.addByThreePoints(
        right_spline.endSketchPoint,
        tooth_top_point.geometry,
        left_spline.endSketchPoint
    )

    # Add diameter dimension to tip arc
    dimensions.addDiameterDimension(
        tip_arc,
        adsk.core.Point3D.create(config.tip_radius, 0, 0)
    )

    # Create spine (vertical construction line through tooth center)
    futil.log(f"[SPINE] anchor_point at: ({anchor_point.geometry.x:.4f}, {anchor_point.geometry.y:.4f})")
    futil.log(f"[SPINE] tooth_top_point at: ({tooth_top_point.geometry.x:.4f}, {tooth_top_point.geometry.y:.4f})")
    spine = curves.sketchLines.addByTwoPoints(anchor_point, tooth_top_point)
    spine.isConstruction = True
    futil.log(f"[SPINE] Created from ({spine.startSketchPoint.geometry.x:.4f}, {spine.startSketchPoint.geometry.y:.4f}) to ({spine.endSketchPoint.geometry.x:.4f}, {spine.endSketchPoint.geometry.y:.4f})")

    # Step 7: Connect to root
    def draw_root_to_involute_line(root, root_radius, inv_line, spine, x, y):
        angle_calc = math.atan(y / x)
        point = adsk.core.Point3D.create(
            root_radius * math.cos(angle_calc),
            root_radius * math.sin(angle_calc),
            0
        )
        futil.log(f"[ROOT_CONNECT] Connecting at angle {math.degrees(angle_calc):.2f}°, point: ({point.x:.4f}, {point.y:.4f})")
        futil.log(f"[ROOT_CONNECT] Involute start at: ({inv_line.startSketchPoint.geometry.x:.4f}, {inv_line.startSketchPoint.geometry.y:.4f})")
        line_to_circle = curves.sketchLines.addByTwoPoints(point, inv_line.startSketchPoint)
        constraints.addCoincident(line_to_circle.startSketchPoint, root)
        futil.log(f"[ROOT_CONNECT] After coincident constraint: ({line_to_circle.startSketchPoint.geometry.x:.4f}, {line_to_circle.startSketchPoint.geometry.y:.4f})")

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

    futil.log(f"[ROOT_CHECK] first_point_radius = {first_point_radius:.4f} cm, root_radius = {config.root_radius:.4f} cm")
    futil.log(f"[ROOT_CHECK] Root connection needed: {first_point_radius > config.root_radius}")

    if first_point_radius > config.root_radius:
        # Involute extends beyond root circle, connect them
        draw_root_to_involute_line(state.root_circle, config.root_radius, left_spline, spine, first_point.x, first_point.y)
        draw_root_to_involute_line(state.root_circle, config.root_radius, right_spline, spine, first_point.x, -first_point.y)
        state.tooth_profile_is_embedded = False
    else:
        # Involute is embedded within root circle
        state.tooth_profile_is_embedded = True

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
