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

from .types import GenerationState


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

    # Angular position for tooth orientation
    angle: float = 0.0  # Angle for circular pattern (in radians)


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
        involute_points[i] = create_point_relative_to(anchor_point, new_x, new_y)

    # Step 5: Draw involute splines
    # Left spline
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        point_collection.add(point)
    left_spline = curves.sketchFittedSplines.add(point_collection)

    # Right spline (mirrored)
    # Mirror Y coordinate around anchor_point
    point_collection = adsk.core.ObjectCollection.create()
    for point in involute_points:
        # Calculate relative coordinates, mirror Y, then convert back to absolute
        rel_x = point.x - anchor_point.geometry.x
        rel_y = point.y - anchor_point.geometry.y
        mirrored_point = create_point_relative_to(anchor_point, rel_x, -rel_y)
        point_collection.add(mirrored_point)
    right_spline = curves.sketchFittedSplines.add(point_collection)

    # Step 6: Draw tip arc
    # Create tip midpoint relative to anchor_point
    # Constraint-based positioning (coincident to tip_circle) handles final placement
    # config.tip_radius is in cm (Fusion 360 API units)
    tooth_top_point = points.add(create_point_relative_to(anchor_point, config.tip_radius, 0))
    constraints.addCoincident(tooth_top_point, state.tip_circle)

    # Create tip arc
    tip_arc = curves.sketchArcs.addByThreePoints(
        right_spline.endSketchPoint,
        tooth_top_point.geometry,
        left_spline.endSketchPoint
    )

    # Add diameter dimension to tip arc
    dimensions.addDiameterDimension(
        tip_arc,
        create_point_relative_to(anchor_point, config.tip_radius, 0)
    )

    # Create spine (vertical construction line through tooth center)
    spine = curves.sketchLines.addByTwoPoints(anchor_point, tooth_top_point)
    spine.isConstruction = True

    # Step 7: Connect to root
    def draw_root_to_involute_line(root, root_radius, inv_line, spine, x, y):
        # Calculate angle from relative coordinates
        rel_x = x - anchor_point.geometry.x
        rel_y = y - anchor_point.geometry.y
        angle_calc = math.atan(rel_y / rel_x)
        # Create point on root circle at calculated angle, relative to anchor
        point = create_point_relative_to(
            anchor_point,
            root_radius * math.cos(angle_calc),
            root_radius * math.sin(angle_calc)
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
    # Calculate radius relative to anchor_point (circle center)
    first_point = involute_points[0]
    rel_x = first_point.x - anchor_point.geometry.x
    rel_y = first_point.y - anchor_point.geometry.y
    first_point_radius = math.sqrt(rel_x ** 2 + rel_y ** 2)

    if first_point_radius > config.root_radius:
        # Involute extends beyond root circle, connect them
        draw_root_to_involute_line(state.root_circle, config.root_radius, left_spline, spine, first_point.x, first_point.y)
        draw_root_to_involute_line(state.root_circle, config.root_radius, right_spline, spine, first_point.x, -first_point.y)
        state.tooth_profile_is_embedded = False
    else:
        # Involute is embedded within root circle
        state.tooth_profile_is_embedded = True

    # Step 8: Add construction geometry for full constraint
    # Always create horizontal reference and angular dimension (consistent for all angles)
    horizontal = curves.sketchLines.addByTwoPoints(
        create_point_relative_to(anchor_point, 0, 0),
        create_point_relative_to(anchor_point, config.tip_radius, 0)
    )
    horizontal.isConstruction = True
    constraints.addHorizontal(horizontal)
    constraints.addCoincident(horizontal.startSketchPoint, anchor_point)
    constraints.addCoincident(horizontal.endSketchPoint, state.tip_circle)

    # Add angular dimension between spine and horizontal reference
    angle_dimension = dimensions.addAngularDimension(
        spine,
        horizontal,
        create_point_relative_to(anchor_point, 0, 0)
    )
    # Set angle value (0 for horizontal tooth, or specified angle)
    angle_dimension.value = config.angle

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
