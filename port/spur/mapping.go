// Package spur ports the decad-gallery spur gear to the Fusion add-in file
// lib/geargen/spurgear.py.
//
// The output is a stored Python fragment with generated parts filled in: the
// dialog inputs, the registered parameters and the involute math come from the
// gallery source, and the rest is Fusion knowledge kept in the fragment.
package spur

// ParamKind tells how a mapped parameter reaches Fusion.
type ParamKind int

const (
	// Primary parameters are read from a dialog input.
	Primary ParamKind = iota
	// Fixed parameters get a constant from the gallery's default and have no dialog input.
	Fixed
	// Derived parameters are Fusion expressions translated from the gallery table.
	Derived
	// GalleryOnly entries exist for the decad build and have no Fusion parameter.
	GalleryOnly
)

// Param maps one gallery table entry to its Fusion identity.
type Param struct {
	// Table is the gallery parameter table name.
	Table string
	Kind  ParamKind
	// Const and Name are the Python constant and the Fusion parameter name. They
	// keep the names existing gears import, such as PARAM_TOOTH_NUMBER.
	Const, Name string
	// Unit is the Fusion parameter unit string.
	Unit    string
	Comment string
	// Input is the dialog input of a primary parameter.
	Input *Input
	// HelixFactor multiplies the parameter's expression by the generator's
	// filletHelixFactorExpression hook, which the helical gear overrides.
	HelixFactor bool
}

// Convert tells how a gallery default becomes a Python dialog default.
type Convert int

const (
	// AsNumber writes the default unchanged.
	AsNumber Convert = iota
	// AsDegrees writes math.radians(default in degrees) for a radian default.
	AsDegrees
	// AsLength writes to_cm(default) for a millimetre default.
	AsLength
)

// Input is a dialog input.
type Input struct {
	Const, ID, Label string
	// GoField is the gallery Params field the default comes from.
	GoField string
	// DialogUnit is the unit the dialog shows, and ValueUnit the unit get_value converts to.
	DialogUnit, ValueUnit string
	Convert               Convert
	// Text makes the input a string input holding TextDefault, for a value the
	// user may enter as an expression or a parameter name.
	Text        bool
	TextDefault string
}

// Params lists every gallery table entry the port knows, in dialog order for the
// primary ones and in registration order for the rest. A gallery entry missing
// from this list stops the port: it needs a decision before it can be generated.
var Params = []Param{
	{Table: "module", Kind: Primary, Const: "PARAM_MODULE", Name: "Module", Comment: "Module of the gear",
		Input: &Input{Const: "INPUT_ID_MODULE", ID: "module", Label: "Module", GoField: "Module"}},
	{Table: "teeth", Kind: Primary, Const: "PARAM_TOOTH_NUMBER", Name: "ToothNumber", Comment: "Number of teeth",
		Input: &Input{Const: "INPUT_ID_TOOTH_NUMBER", ID: "toothNumber", Label: "Tooth Number", GoField: "Teeth"}},
	{Table: "pressure_angle", Kind: Primary, Const: "PARAM_PRESSURE_ANGLE", Name: "PressureAngle", Unit: "rad",
		Comment: "Pressure angle",
		Input: &Input{Const: "INPUT_ID_PRESSURE_ANGLE", ID: "pressureAngle", Label: "Pressure Angle",
			GoField: "PressureAngle", DialogUnit: "deg", ValueUnit: "rad", Convert: AsDegrees}},
	{Table: "bore_diameter", Kind: Primary, Const: "PARAM_BORE_DIAMETER", Name: "BoreDiameter", Unit: "mm",
		Comment: "Bore diameter",
		Input: &Input{Const: "INPUT_ID_BORE_DIAMETER", ID: "boreDiameter", Label: "Bore Diameter",
			GoField: "BoreDiameter", ValueUnit: "mm", Text: true, TextDefault: "0 mm"}},
	{Table: "thickness", Kind: Primary, Const: "PARAM_THICKNESS", Name: "Thickness", Unit: "mm",
		Comment: "Thickness of the gear",
		Input: &Input{Const: "INPUT_ID_THICKNESS", ID: "thickness", Label: "Thickness", GoField: "Thickness",
			DialogUnit: "mm", ValueUnit: "mm", Convert: AsLength}},
	{Table: "cap_chamfer", Kind: Primary, Const: "PARAM_CHAMFER_TOOTH", Name: "ChamferTooth", Unit: "mm",
		Comment: "Chamfer distance applied to the teeth",
		Input: &Input{Const: "INPUT_ID_CHAMFER_TOOTH", ID: "chamferTooth", Label: "Apply chamfer to teeth",
			GoField: "CapChamfer", DialogUnit: "mm", ValueUnit: "mm"}},
	{Table: "samples", Kind: Fixed, Const: "PARAM_INVOLUTE_STEPS", Name: "InvoluteSteps",
		Comment: "Number of points sampled along each involute flank"},
	{Table: "tool_margin", Kind: GalleryOnly},
	{Table: "pitch_radius", Kind: Derived, Const: "PARAM_PITCH_RADIUS", Name: "PitchCircleRadius", Unit: "mm",
		Comment: "Pitch circle radius"},
	{Table: "base_radius", Kind: Derived, Const: "PARAM_BASE_RADIUS", Name: "BaseCircleRadius", Unit: "mm",
		Comment: "Base circle radius"},
	{Table: "root_radius", Kind: Derived, Const: "PARAM_ROOT_RADIUS", Name: "RootCircleRadius", Unit: "mm",
		Comment: "Root circle radius"},
	{Table: "tip_radius", Kind: Derived, Const: "PARAM_TIP_RADIUS", Name: "TipCircleRadius", Unit: "mm",
		Comment: "Tip circle radius"},
	{Table: "involute_angle", Kind: Derived, Const: "PARAM_INVOLUTE_ANGLE", Name: "InvoluteAngle",
		Comment: "Involute function of the pressure angle"},
	{Table: "fillet_radius", Kind: Derived, Const: "PARAM_FILLET_RADIUS", Name: "FilletRadius", Unit: "mm",
		Comment: "Radius of the root fillets", HelixFactor: true},
	{Table: "bore_tool_length", Kind: GalleryOnly},
}

// ExpectedOps is the sequence of decad feature operations the fragment implements.
// The port refuses to generate when the gallery's build performs a different
// sequence, because the fragment's Fusion feature order would no longer match.
var ExpectedOps = []string{
	"Extrude",       // root disc
	"Extrude",       // tooth
	"PatternCopies", // teeth
	"Union",         // join
	"Fillet",        // ordinary root fillet
	"Fillet",        // embedded root fillet
	"Extrude",       // bore tool
	"Cut",           // bore
	"Chamfer",       // cap chamfer
}

// ReviewedFuncs are the gallery functions the fragment's Fusion construction
// follows. The fragment records a digest of their code (reviewed.json); a change
// to any of them stops the port until the fragment is reviewed and the digest accepted.
var ReviewedFuncs = []string{
	"assemble", "tooth", "filletSpurRoots", "buildToothSection", "buildEmbeddedTooth", "addToothPoints",
	"closingRadius", "halfAngle", "filletRootsValue", "filletEmbedded", "buildBoreProfile", "cutBore",
	"chamferOuterCaps",
}

// MathStructs and MathFuncs are the gallery helpers translated to Python.
var (
	MathStructs = []string{"point"}
	MathFuncs   = []string{"rotate", "involutePoint", "flankBeta", "sampleFlanks"}
)
