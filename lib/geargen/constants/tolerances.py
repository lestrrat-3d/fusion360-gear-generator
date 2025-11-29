"""Geometric tolerance constants for gear generation.

All tolerance values are in centimeters (Fusion 360's internal unit).
"""

# Length Tolerances (in cm)
MINIMUM_PROFILE_LINE_LENGTH_CM = 0.1  # 1mm minimum for reliable face intersection detection
PROXIMITY_TOLERANCE_CM = 0.01  # 0.1mm tolerance for proximity detection
FLOATING_POINT_TOLERANCE_CM = 0.0001  # 0.1 micron for floating point comparisons

# Direction Vector Tolerances
VECTOR_LENGTH_MINIMUM = 0.01  # Minimum vector length to avoid parallel/degenerate cases
PERPENDICULARITY_THRESHOLD = 0.1  # Threshold for detecting perpendicular orientations

# Tiny Geometry Constants
TINY_CIRCLE_RADIUS_CM = 0.0001  # For apex point sketch (1 micrometer = 0.001 mm)
