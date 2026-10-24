// Package spur ports the decad-gallery spur gear to the Fusion add-in file
// lib/geargen/spurgear.py.
//
// The output is a stored Python fragment with generated parts filled in: the
// dialog inputs, the registered parameters and the involute math come from the
// gallery source, and the rest is Fusion knowledge kept in the fragment.
package spur

import "github.com/lestrrat-3d/fusion360-gear-generator/port/gotopy"

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
	{Table: "tool_margin", Kind: Derived, Const: "PARAM_TOOL_MARGIN", Name: "ToolMargin", Unit: "mm",
		Comment: "Distance the bore tool starts below the gear"},
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
	{Table: "bore_tool_length", Kind: Derived, Const: "PARAM_BORE_TOOL_LENGTH", Name: "BoreToolLength", Unit: "mm",
		Comment: "Length of the bore tool"},
}

// FeatureSpec is what the port translates from the gallery model into the Fusion add-in: the
// involute sampling, the dimensioned points of one tooth, and the whole build (tooth, root disc,
// pattern, join, root fillet, bore, cap chamfer) in the order the gallery performs it. The Fusion
// runtime that executes the calls is lib/geargen/decadfusion.py.
//
// The tooth section is translated twice. buildToothSection builds a sketch of its own, which the
// spur uses. The drawing steps (draw_tooth_section) draw into a sketch the caller made, which the
// helical, herringbone and bevel add-ins still use until their constructions are translated too.
var FeatureSpec = gotopy.Spec{
	Prefix:   "_",
	Structs:  []string{"point", "dimensions", "toothPoints"},
	TypeOnly: []string{"Params", "run"},
	Zero:     []string{"Variant"},
	// The add-in registers the parameters itself, from the mapping above.
	Provided: []string{"defineParameters"},
	Funcs: []string{
		"rotate", "involutePoint", "flankBeta", "sampleFlanks", "rootContact", "toothAngle", "addToothPoints",
		"toWorld", "shifted", "placement", "singleProfile", "solveAndVerify", "halfAngle", "closingRadius",
		"buildToothSection", "buildEmbeddedTooth", "tooth", "filletSpurRoots", "filletRootsValue",
		"filletEmbedded", "buildBoreProfile", "cutBore", "chamferOuterCaps", "assemble",
	},
	Steps: []gotopy.Steps{{
		Name: "buildToothSection", PyName: "draw_tooth_section", StopAt: "solveAndVerify",
		Args: []gotopy.StepArg{
			{Name: "s", Type: "*sketch.Sketch"}, {Name: "p", Type: "Params"},
			{Name: "d", Type: "dimensions"}, {Name: "angle", Type: "float64"},
		},
	}},
	Roles: Roles,
}

// Roles names each object the translated build creates in Fusion, by its role. A key is the Go
// function, the operation and the number of the call in that function.
var Roles = map[string]gotopy.Role{
	"buildToothSection:sketch.World.CreateSketch#1":        {Feature: "Tooth Profile"},
	"buildEmbeddedTooth:sketch.World.CreateSketch#1":       {Feature: "Tooth Profile"},
	"tooth:decad.Extrude#1":                                {Feature: "Tooth Extrude", Body: "Tooth"},
	"assemble:sketch.World.CreateSketch#1":                 {Feature: "Root Disc Profile"},
	"assemble:decad.Extrude#1":                             {Feature: "Root Disc Extrude", Body: "Gear Body"},
	"assemble:decad.PatternCopies#1":                       {Feature: "Tooth Pattern"},
	"assemble:decad.Union#1":                               {Feature: "Teeth Join"},
	"filletRootsValue:decad.Fillet#1":                      {Feature: "Root Fillet"},
	"filletEmbedded:decad.Fillet#1":                        {Feature: "Root Fillet"},
	"buildBoreProfile:sketch.World.CreatePlaneFromFrame#1": {Feature: "Bore Tool Plane"},
	"buildBoreProfile:sketch.World.CreateSketch#1":         {Feature: "Bore Profile"},
	"cutBore:decad.Extrude#1":                              {Feature: "Bore Tool Extrude", Body: "Bore Tool"},
	"cutBore:decad.Cut#1":                                  {Feature: "Bore Cut"},
	"chamferOuterCaps:decad.Chamfer#1":                     {Feature: "Cap Chamfer"},
}
