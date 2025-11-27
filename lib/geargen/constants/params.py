"""Parameter and input field ID constants for gear generation."""

# Core Parameter Names & Input IDs
PARAM_MODULE = "Module"  # Module parameter - fundamental gear sizing unit (metric)
INPUT_MODULE = "module"  # Input field ID for module parameter

PARAM_TOOTH_NUMBER = "ToothNumber"  # Number of teeth on the gear
INPUT_TOOTH_NUMBER = "toothNumber"  # Input field ID for tooth number

PARAM_MATING_TOOTH_NUMBER = "MatingToothNumber"  # Number of teeth on mating gear (bevel gears)
INPUT_MATING_TOOTH_NUMBER = "matingToothNumber"  # Input field ID for mating tooth number

PARAM_PRESSURE_ANGLE = "PressureAngle"  # Pressure angle - angle of gear tooth face
INPUT_PRESSURE_ANGLE = "pressureAngle"  # Input field ID for pressure angle

PARAM_HELIX_ANGLE = "HelixAngle"  # Helix angle - angle of helical twist
INPUT_HELIX_ANGLE = "helixAngle"  # Input field ID for helix angle

PARAM_BORE_DIAMETER = "BoreDiameter"  # Diameter of central bore/shaft hole
INPUT_BORE_DIAMETER = "boreDiameter"  # Input field ID for bore diameter

PARAM_THICKNESS = "Thickness"  # Thickness/width of the gear
INPUT_THICKNESS = "thickness"  # Input field ID for gear thickness

PARAM_CHAMFER_TOOTH = "ChamferTooth"  # Chamfer size for tooth edges
INPUT_CHAMFER_TOOTH = "chamferTooth"  # Input field ID for chamfer size

PARAM_SKETCH_ONLY = "SketchOnly"  # Flag: 1 for sketch-only, 0 for full 3D body
INPUT_SKETCH_ONLY = "sketchOnly"  # Input field ID for sketch-only checkbox

# Selection Input Field IDs
INPUT_ANCHOR_POINT = "anchorPoint"  # Input field ID for anchor point selection
INPUT_PLANE = "plane"  # Input field ID for plane selection
INPUT_PARENT_COMPONENT = "parentComponent"  # Input field ID for parent component selection

# Derived Circle Parameters
PARAM_PITCH_CIRCLE_DIAMETER = "PitchCircleDiameter"  # Pitch circle diameter
PARAM_PITCH_CIRCLE_RADIUS = "PitchCircleRadius"  # Pitch circle radius
PARAM_BASE_CIRCLE_DIAMETER = "BaseCircleDiameter"  # Base circle diameter
PARAM_BASE_CIRCLE_RADIUS = "BaseCircleRadius"  # Base circle radius
PARAM_ROOT_CIRCLE_DIAMETER = "RootCircleDiameter"  # Root circle diameter
PARAM_ROOT_CIRCLE_RADIUS = "RootCircleRadius"  # Root circle radius
PARAM_TIP_CIRCLE_DIAMETER = "TipCircleDiameter"  # Tip circle diameter
PARAM_TIP_CIRCLE_RADIUS = "TipCircleRadius"  # Tip circle radius

# Tooth Profile Parameters
PARAM_INVOLUTE_STEPS = "InvoluteSteps"  # Number of line segments for involute curve
PARAM_FILLET_THRESHOLD = "FilletThreshold"  # Maximum fillet radius at tooth root
PARAM_FILLET_RADIUS = "FilletRadius"  # Actual fillet radius at tooth root

# Bevel Gear Specific Parameters
PARAM_GEAR_HEIGHT = "GearHeight"  # Height/pitch radius of main bevel gear
PARAM_MATING_GEAR_HEIGHT = "MatingGearHeight"  # Height/pitch radius of mating gear
PARAM_VIRTUAL_TEETH_NUMBER = "VirtualTeethNumber"  # Virtual tooth count for bevel gears
PARAM_DRIVING_GEAR_BASE_THICKNESS = "DrivingGearBaseThickness"  # Base thickness for driving gear
PARAM_TEETH_LENGTH = "TeethLength"  # Distance between diagonal lines in gear extension
PARAM_SHAFT_ANGLE = "ShaftAngle"  # Angle between gear shafts (bevel gears)
PARAM_FACE_WIDTH = "FaceWidth"  # Face width of bevel gear

# Bevel Gear Specific Input IDs
INPUT_SHAFT_ANGLE = "shaftAngle"  # Input field ID for shaft angle
INPUT_FACE_WIDTH = "faceWidth"  # Input field ID for face width
INPUT_DRIVING_GEAR_BASE_THICKNESS = "drivingGearBaseThickness"  # Input field ID for driving gear base thickness
INPUT_TEETH_LENGTH = "teethLength"  # Input field ID for teeth length

# Default values for UI inputs
DEFAULT_MODULE_MM = 1.0
DEFAULT_TOOTH_NUMBER = 17
DEFAULT_PRESSURE_ANGLE_DEG = 20.0
DEFAULT_BORE_DIAMETER_STR = "0 mm"
DEFAULT_THICKNESS_MM = 10.0
DEFAULT_CHAMFER_TOOTH_MM = 0.0
DEFAULT_SKETCH_ONLY = False
DEFAULT_HELIX_ANGLE_DEG = 14.5  # for helical/herringbone gears (matches current implementation)

# Bevel Gear Specific Defaults
DEFAULT_MATING_TOOTH_NUMBER = 17
DEFAULT_SHAFT_ANGLE_DEG = 90.0
DEFAULT_DRIVING_GEAR_BASE_THICKNESS_MM = 5.0
DEFAULT_TEETH_LENGTH_MM = 10.0
