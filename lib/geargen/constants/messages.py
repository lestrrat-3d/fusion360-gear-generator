"""User-facing message constants for gear generation."""

# Error Messages - Generic
ERR_GENERATION = "Generation error"  # Generic gear generation error
ERR_GENERATION_FAILED = "Gear generation failed"  # Generic failure message

# Error Messages - Input Validation
ERR_INVALID_MODULE = "Invalid module value"  # Module out of range
ERR_INVALID_PRESSURE_ANGLE = "Invalid pressure angle value"  # Pressure angle out of range
ERR_INVALID_TOOTH_NUMBER = "Invalid tooth number value"  # Tooth number out of range
ERR_INVALID_HELIX_ANGLE = "Invalid helix angle value"  # Helix angle out of range
ERR_INVALID_THICKNESS = "Invalid thickness value"  # Thickness out of range

# Error Messages - Parameters
ERR_MODULE_PARAM_NOT_FOUND = "Module parameter not found"  # Module param doesn't exist
ERR_PARAM_NOT_FOUND = "Parameter not found"  # Generic missing parameter
ERR_PARAM_CREATE_FAILED = "Failed to create parameter"  # Failed to add parameter

# Error Messages - Sketch and Geometry
ERR_ANCHOR_POINT_PROJECTION = "Failed to project anchor point"  # Anchor projection failed
ERR_SKETCH_CREATION = "Failed to create sketch"  # Sketch creation failed
ERR_PROFILE_NOT_FOUND = "Profile not found"  # Expected profile missing
ERR_TOOTH_PROFILE_NOT_FOUND = "could not find tooth profile"  # Tooth profile missing
ERR_GEAR_BODY_PROFILE_NOT_FOUND = "could not find gear body profile"  # Gear body profile missing

# Error Messages - 3D Operations
ERR_EXTRUDE_FAILED = "Failed to extrude profile"  # Extrusion failed
ERR_LOFT_CREATION = "Cannot create lofted tooth"  # Loft creation failed
ERR_REVOLVE_FAILED = "Failed to revolve profile"  # Revolve failed
ERR_JOIN_BODIES = "Cannot join bodies"  # Body combine failed
ERR_PATTERN_FAILED = "Failed to create pattern"  # Circular pattern failed

# Error Messages - Component and Features
ERR_COMPONENT_CREATION = "Failed to create component"  # Component creation failed
ERR_PLANE_CREATION = "Failed to create construction plane"  # Plane creation failed
ERR_AXIS_CREATION = "Failed to create construction axis"  # Axis creation failed

# Error Messages - Bevel Gear Phases
ERR_PHASE1_INCOMPLETE = "Phase 1 incomplete"  # Foundation sketch not complete
ERR_PHASE2_INCOMPLETE = "Phase 2 incomplete"  # Tooth profile not complete
ERR_FOUNDATION_SKETCH_NOT_FOUND = "Foundation sketch not found"  # Can't find foundation sketch

# Error Messages - State
ERR_MISSING_STATE_ATTR = "Missing required state attribute"  # State missing attribute
ERR_INVALID_STATE = "Invalid generation state"  # State is inconsistent

# UI Labels
LABEL_MODULE = "Module"  # Label for module input field
LABEL_TOOTH_NUMBER = "Number of Teeth"  # Label for tooth number input
LABEL_PRESSURE_ANGLE = "Pressure Angle"  # Label for pressure angle input
LABEL_HELIX_ANGLE = "Helix Angle"  # Label for helix angle input
LABEL_BORE_DIAMETER = "Bore Diameter"  # Label for bore diameter input
LABEL_THICKNESS = "Thickness"  # Label for thickness input
LABEL_CHAMFER_TOOTH = "Chamfer"  # Label for chamfer input
LABEL_SKETCH_ONLY = "Sketch Only"  # Label for sketch-only checkbox

# Tooltips
TOOLTIP_MODULE = "The module of the gear (metric sizing unit)"  # Module tooltip
TOOLTIP_TOOTH_NUMBER = "The number of teeth on the gear"  # Tooth number tooltip
TOOLTIP_PRESSURE_ANGLE = "The pressure angle of the gear teeth"  # Pressure angle tooltip
TOOLTIP_SKETCH_ONLY = "Generate sketches, but do not build body"  # Sketch-only tooltip
TOOLTIP_PLANE = "Select a plane to position the gear"  # Plane selection tooltip
TOOLTIP_ANCHOR_POINT = "Select a point to anchor the gear"  # Anchor point tooltip
TOOLTIP_PARENT_COMPONENT = "Select the parent component for the gear"  # Parent component tooltip

# Selection Prompts
PROMPT_SELECT_PLANE = "Select Plane"  # Prompt for plane selection
PROMPT_SELECT_POINT = "Select Point"  # Prompt for point selection
PROMPT_SELECT_COMPONENT = "Select Component"  # Prompt for component selection
PROMPT_SELECT_FACE = "Select Face"  # Prompt for face selection
PROMPT_SELECT_EDGE = "Select Edge"  # Prompt for edge selection

# Validation Messages
VALIDATION_POSITIVE = "must be > 0"  # Value must be positive
VALIDATION_NON_NEGATIVE = "must be >= 0"  # Value must be non-negative
VALIDATION_MIN_ONE = "must be at least 1"  # Value must be at least 1
VALIDATION_BETWEEN_0_AND_1 = "must be between 0 and 1"  # Value in range [0,1]
VALIDATION_INTEGER_REQUIRED = "must be an integer"  # Value must be integer
VALIDATION_NUMERIC_REQUIRED = "must be a number"  # Value must be numeric
VALIDATION_MISSING_STATE = "Missing required state attribute"  # State missing attribute
VALIDATION_INVALID_STATE = "Invalid generation state"  # State is invalid
VALIDATION_NO_SELECTION = "No selection made"  # No entity selected
VALIDATION_INVALID_SELECTION = "Invalid selection"  # Wrong entity type selected
