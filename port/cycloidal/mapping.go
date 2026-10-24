// Package cycloidal ports the decad-gallery cycloidal drive to the Fusion add-in file
// lib/geargen/cycloidal.py.
//
// The add-in builds the drive with Fusion calls that the stored fragment holds. From the
// gallery source the port takes the dialog inputs and their defaults, and it stops when the
// gallery's inputs, parameter table, build operations or reviewed code change.
package cycloidal

// Kind tells how a gallery default becomes a dialog default.
type Kind int

const (
	// Number writes the default as a unitless real.
	Number Kind = iota
	// Length writes to_cm(default) for a millimetre default.
	Length
	// Count writes a drop-down of 1 and 2 with the default selected.
	Count
)

// Input maps one gallery Params field to a dialog input.
type Input struct {
	// Field is the gallery Params field.
	Field string
	// Const is the Python constant of the input id.
	Const string
	Label string
	Kind  Kind
}

// Inputs lists the dialog inputs in dialog order. A gallery Params field that is in neither
// Inputs nor GalleryOnly stops the port: it needs a decision before it can be generated.
var Inputs = []Input{
	{Field: "DiscCount", Const: "INPUT_ID_DISC_COUNT", Label: "Disc Count", Kind: Count},
	{Field: "PinCount", Const: "INPUT_ID_PIN_COUNT", Label: "Pin Count", Kind: Number},
	{Field: "PinCircleDiameter", Const: "INPUT_ID_PIN_CIRCLE_DIAMETER", Label: "Pin Circle Diameter", Kind: Length},
	{Field: "PinDiameter", Const: "INPUT_ID_PIN_DIAMETER", Label: "Pin Diameter", Kind: Length},
	{Field: "Eccentricity", Const: "INPUT_ID_ECCENTRICITY", Label: "Eccentricity", Kind: Length},
	{Field: "DiskClearance", Const: "INPUT_ID_DISK_CLEARANCE", Label: "Disk Clearance", Kind: Length},
	{Field: "DiscThickness", Const: "INPUT_ID_DISC_THICKNESS", Label: "Disc Thickness", Kind: Length},
	{Field: "DiscGap", Const: "INPUT_ID_DISC_GAP", Label: "Disc Gap", Kind: Length},
	{Field: "CenterBearingDiameter", Const: "INPUT_ID_CENTER_BEARING_DIAMETER", Label: "Center Bearing Diameter", Kind: Length},
	{Field: "InputShaftDiameter", Const: "INPUT_ID_INPUT_SHAFT_DIAMETER", Label: "Input Shaft Diameter", Kind: Length},
	{Field: "BearingClearance", Const: "INPUT_ID_BEARING_CLEARANCE", Label: "Bearing Clearance", Kind: Length},
	{Field: "OutputPinCircleDiameter", Const: "INPUT_ID_OUTPUT_PIN_CIRCLE_DIAMETER", Label: "Output Pin Circle Diameter", Kind: Length},
	{Field: "OutputPinCount", Const: "INPUT_ID_OUTPUT_PIN_COUNT", Label: "Output Pin Count", Kind: Number},
	{Field: "OutputPinDiameter", Const: "INPUT_ID_OUTPUT_PIN_DIAMETER", Label: "Output Pin Diameter", Kind: Length},
	{Field: "HousingWall", Const: "INPUT_ID_WALL", Label: "Housing Wall", Kind: Length},
	{Field: "BaseThickness", Const: "INPUT_ID_BASE_THICKNESS", Label: "Base Thickness", Kind: Length},
	{Field: "OutputPlateThickness", Const: "INPUT_ID_OUTPUT_PLATE_THICKNESS", Label: "Output Plate Thickness", Kind: Length},
	{Field: "ChamferSize", Const: "INPUT_ID_CHAMFER_SIZE", Label: "Chamfer Size", Kind: Length},
}

// GalleryOnly lists the Params fields with no dialog input. Placement is the decad document's
// placement; the add-in takes a Target Plane and an Anchor Point instead.
var GalleryOnly = []string{"Placement"}

// RegisteredParams maps the gallery parameter table names that the add-in registers as Fusion user
// parameters to the Python constants it registers them under. The test compares them with the
// fragment, so a gallery rename or a new derived dimension stops the port.
var RegisteredParams = map[string]string{
	"pin_count": "PARAM_PIN_COUNT", "pin_circle_diameter": "PARAM_PIN_CIRCLE_DIAMETER",
	"pin_diameter": "PARAM_PIN_DIAMETER", "eccentricity": "PARAM_ECCENTRICITY",
	"disk_clearance": "PARAM_DISK_CLEARANCE", "disc_thickness": "PARAM_DISC_THICKNESS",
	"disc_gap": "PARAM_DISC_GAP", "center_bearing_diameter": "PARAM_CENTER_BEARING_DIAMETER",
	"input_shaft_diameter": "PARAM_INPUT_SHAFT_DIAMETER", "bearing_clearance": "PARAM_BEARING_CLEARANCE",
	"output_pin_circle_diameter": "PARAM_OUTPUT_PIN_CIRCLE_DIAMETER",
	"output_pin_count":           "PARAM_OUTPUT_PIN_COUNT", "output_pin_diameter": "PARAM_OUTPUT_PIN_DIAMETER",
	"wall": "PARAM_WALL", "base_thickness": "PARAM_BASE_THICKNESS",
	"output_plate_thickness": "PARAM_OUTPUT_PLATE_THICKNESS", "chamfer_size": "PARAM_CHAMFER_SIZE",
	"lobes": "PARAM_LOBES", "pin_circle_radius": "PARAM_PIN_CIRCLE_RADIUS",
	"output_pin_circle_radius": "PARAM_OUTPUT_PIN_CIRCLE_RADIUS", "pin_radius": "PARAM_PIN_RADIUS",
	"output_hole_diameter": "PARAM_OUTPUT_HOLE_DIAMETER", "housing_inner_diameter": "PARAM_HOUSING_INNER_DIAMETER",
	"housing_outer_diameter": "PARAM_HOUSING_OUTER_DIAMETER", "output_plate_diameter": "PARAM_OUTPUT_PLATE_DIAMETER",
}

// TableGalleryOnly lists the table names the gallery defines for its own build and the add-in
// does not register: the add-in composes these from the parameters above.
// disc_count is the drop-down selection, which the add-in reads and does not register.
var TableGalleryOnly = []string{
	"disc_count", "root_circle_diameter", "disc_bore_diameter", "resolved_output_pin_diameter",
	"plane_gap", "disc_pitch", "stack_height", "output_plate_height", "output_pin_length",
}

// ExpectedOps is the sequence of decad feature operations the gallery Build performs. The port stops
// when it changes, because the Fusion construction in the fragment was reviewed against this sequence.
var ExpectedOps = []string{
	"Extrude", "Extrude", "Union", "Extrude", "Cut", "Extrude", "Extrude", "Cut", "Extrude", "Cut",
	"Chamfer", "Extrude", "Extrude",
}

// ReviewedFuncs are the gallery functions the fragment's Fusion construction follows. The fragment
// records a digest of their code (reviewed.json); a change to any of them stops the port until the
// fragment is reviewed and the digest accepted.
var ReviewedFuncs = []string{
	"Build", "resolve", "Problems", "Validate", "undercutBound", "undercuts", "minInwardCurvatureRadius",
	"defineParameters",
}
