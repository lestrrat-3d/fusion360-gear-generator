// Package cycloidal ports the decad-gallery cycloidal drive to the Fusion add-in file
// lib/geargen/cycloidal.py.
//
// The drive is translated from the gallery: the rotor discs, cam, housing, output plate and pins, their
// chamfers, the validity checks and the dimensions all come from FeatureSpec. From the gallery source the
// port also takes the dialog inputs and their defaults and the whole parameter table. The fragment holds the
// Fusion wiring: the dialog, the component groups and the call of the translated build.
package cycloidal

import (
	"strings"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
)

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

// ParamConst is the Python constant of the Fusion parameter for a gallery table name.
func ParamConst(table string) string { return "PARAM_" + strings.ToUpper(table) }

// ParamName is the Fusion parameter name for a gallery table name: its words capitalised.
func ParamName(table string) string {
	var sb strings.Builder
	for _, w := range strings.Split(table, "_") {
		if w != "" {
			sb.WriteString(strings.ToUpper(w[:1]) + w[1:])
		}
	}
	return sb.String()
}

// FeatureSpec is what the port translates from the gallery model: the rotor profile, the casing
// contour, the dimensions, the validity checks and the whole build, in the order the gallery performs it.
// The Fusion runtime that executes the calls is lib/geargen/decadfusion.py, run in millimetres.
//
// The retry loops over lobeTurnLimits and contourStrides run once, with their first entry:
// docs/gallery-translation-goal.md, "Retry loops".
var FeatureSpec = go2py.Spec{
	Prefix:    "_",
	Structs:   []string{"point", "dimensions", "run", "Drive", "NamedBody"},
	TypeOnly:  []string{"Params"},
	Mutable:   []string{"dimensions", "Params"},
	Bound:     map[string]go2py.Bound{"begin": {Python: "_begin", Result: "run"}},
	FirstOnly: []string{"lobeTurnLimits", "contourStrides"},
	Values: []string{
		"undercutScanPoints", "HousingName", "CamName", "OutputPlateName", "planeGap", "camSeparation",
		"camOverlap", "lobeTurnLimits", "contourStrides", "lobeFineSteps", "contourPoints", "contourCamSteps",
		"contourCurveSteps",
	},
	Funcs: []string{
		"placement", "regionWithHoles", "solveAndVerify", "discPoint", "lobeParameters", "profileCurvature",
		"inflections", "lobeSegments", "housingContour", "splitAtInflections", "thinned", "resolve",
		"minInwardCurvatureRadius", "undercuts", "undercutBound", "Problems", "planeAt", "circleAt",
		"discGeometry", "discHeight", "buildDisc", "buildDiscSampled", "buildHousing", "buildCavity", "buildCam",
		"buildCamBridge", "outputPinCenter", "buildOutput", "chamferedBy", "housingOuterCircles", "drawPatternedLoop",
		"chamfered", "chamferedDisc", "Build", "DiscName", "OutputPinName", "Named",
	},
	Roles: map[string]go2py.Role{
		"planeAt:sketch.World.CreatePlaneFromFrame#1":  {Feature: "Section Plane"},
		"buildDiscSampled:sketch.World.CreateSketch#1": {Feature: "Disc Profile"},
		"buildDiscSampled:decad.Extrude#1":             {Feature: "Disc Extrude", Body: "Disc"},
		"buildHousing:sketch.World.CreateSketch#1":     {Feature: "Housing Outer Profile"},
		"buildHousing:sketch.World.CreateSketch#2":     {Feature: "Housing Hole Profile"},
		"buildHousing:decad.Extrude#1":                 {Feature: "Housing Outer Extrude", Body: "Housing"},
		"buildHousing:decad.Extrude#2":                 {Feature: "Housing Hole Extrude", Body: "Housing Hole Tool"},
		"buildHousing:decad.Cut#1":                     {Feature: "Casing Cavity Cut"},
		"buildHousing:decad.Cut#2":                     {Feature: "Housing Hole Cut"},
		"buildCavity:sketch.World.CreateSketch#1":      {Feature: "Casing Profile"},
		"buildCavity:decad.Extrude#1":                  {Feature: "Casing Cavity Extrude", Body: "Casing Cavity Tool"},
		"buildCam:sketch.World.CreateSketch#1":         {Feature: "Cam Section Profile"},
		"buildCam:sketch.World.CreateSketch#2":         {Feature: "Input Bore Profile"},
		"buildCam:decad.Extrude#1":                     {Feature: "Cam Section Extrude", Body: "Cam"},
		"buildCam:decad.Extrude#2":                     {Feature: "Input Bore Extrude", Body: "Input Bore Tool"},
		"buildCam:decad.Union#1":                       {Feature: "Cam Join"},
		"buildCam:decad.Cut#1":                         {Feature: "Input Bore Cut"},
		"buildCamBridge:sketch.World.CreateSketch#1":   {Feature: "Cam Bridge Profile"},
		"buildCamBridge:decad.Extrude#1":               {Feature: "Cam Bridge Extrude", Body: "Cam Bridge"},
		"buildOutput:sketch.World.CreateSketch#1":      {Feature: "Output Plate Profile"},
		"buildOutput:sketch.World.CreateSketch#2":      {Feature: "Output Pin Profile"},
		"buildOutput:decad.Extrude#1":                  {Feature: "Output Plate Extrude", Body: "Output Plate"},
		"buildOutput:decad.Extrude#2":                  {Feature: "Output Pin Extrude", Body: "Output Pin"},
		"chamferedBy:decad.Chamfer#1":                  {Feature: "Chamfer", Qualify: "what"},
	},
}
