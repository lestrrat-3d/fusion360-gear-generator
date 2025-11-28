"""Fusion 360 entity name constants for gear generation."""

# Component Names
COMPONENT_DESIGN = "Design"  # Root design component for bevel gears
COMPONENT_DRIVING_GEAR = "Driving Gear"  # Driving (main) bevel gear component
COMPONENT_MATING_GEAR = "Mating Gear"  # Mating (secondary) bevel gear component

# Body Names
BODY_NAME_GEAR = "Gear Body"  # Main gear body (shaft with teeth)
BODY_NAME_TOOTH = "Tooth Body"  # Individual tooth body before patterning
BODY_NAME_BASE = "Base Body"  # Base gear body (before adding teeth)
BODY_NAME_BORE = "Bore"  # Bore/shaft hole body (for cutting operation)

# Sketch Names
SKETCH_FOUNDATION = "Foundation Sketch"  # Foundation sketch for bevel gear (pitch cone)
SKETCH_TOOTH_PROFILE = "Tooth Profile"  # Tooth profile for extrusion/lofting
SKETCH_GEAR_PROFILE = "Gear Profile"  # Full gear profile (spur/helical gears)
SKETCH_BORE = "Bore Sketch"  # Bore/shaft hole sketch
SKETCH_TOOLS = "Tools"  # Utility sketch for construction geometry
SKETCH_PERPENDICULAR_REFERENCE = "Perpendicular Reference"  # Temporary sketch for perpendicular plane construction
SKETCH_TOOTH_PROFILE_REFERENCE = "Tooth Profile Reference"  # Temporary sketch for tooth profile plane construction
SKETCH_APEX_FOR_LOFT = "Apex Point for Loft"  # Apex sketch for lofting tooth body

# Construction Plane Names
PLANE_FOUNDATION = "Foundation Plane"  # Construction plane for bevel gear foundation
PLANE_TOOTH_PROFILE = "Tooth Profile Plane"  # Construction plane for tooth profile
PLANE_BOTTOM = "Bottom Plane"  # Bottom plane (for helical gears)
PLANE_TOP = "Top Plane"  # Top plane (for helical gears)
PLANE_GEAR_BASE_COPLANAR = "Gear Base Plane (Coplanar)"  # Coplanar construction plane for gear base

# Construction Axis Names
AXIS_GEAR_CENTER = "Gear Center"  # Gear center axis (axis of rotation)
AXIS_TOOTH = "Tooth Axis"  # Individual tooth axis

# Feature Names
FEATURE_PATTERN = "Tooth Pattern"  # Circular pattern for teeth duplication
FEATURE_EXTRUDE_GEAR = "Gear Extrude"  # Extrusion feature for gear body
FEATURE_EXTRUDE_TOOTH = "Tooth Extrude"  # Extrusion feature for tooth
FEATURE_LOFT_TOOTH = "Tooth Loft"  # Loft feature for helical tooth
FEATURE_REVOLVE_BORE = "Bore Revolve"  # Revolve feature for bore
