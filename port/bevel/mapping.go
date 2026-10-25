// Package bevel ports the decad-gallery bevel gear to the Fusion add-in file
// lib/geargen/bevelgear.py.
//
// The pair is translated from the gallery: the blanks, the teeth, the cones, the bore, the containers and the
// validity checks all come from FeatureSpec. From the gallery source the port also takes the dialog inputs and
// their defaults. The fragment holds the Fusion wiring: the dialog, the Tools sketch and the call of the
// translated build. The Fusion runtime that executes the calls is lib/geargen/decadfusion.py, run in millimetres.
package bevel

import (
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
)

// Kind tells how a gallery default becomes a dialog default.
type Kind int

const (
	// Number writes the default as a unitless real.
	Number Kind = iota
	// Angle writes the default as a string such as '90 deg'.
	Angle
	// Length writes to_cm(default) for a millimetre default.
	Length
	// Bool writes True or False.
	Bool
	// Hand writes a drop-down of Right and Left with the default selected.
	Hand
	// ToothType writes a drop-down of Straight and Spiral that no gallery field backs. Choosing Straight hides the
	// spiral-only inputs and builds with a spiral angle of zero.
	ToothType
)

// Input maps one gallery Params field to a dialog input.
type Input struct {
	// Field is the gallery Params field. It is empty for an input the gallery does not have.
	Field string
	// Const is the Python constant of the input id.
	Const string
	Label string
	Kind  Kind
	// Fallback is the dialog default when the gallery default is zero. The spiral angle is zero by default
	// (straight teeth), and a spiral pair needs an angle above zero, so the field starts at a working value.
	Fallback float64
}

// Inputs lists the dialog inputs in dialog order. A gallery Params field that is in
// neither Inputs nor GalleryOnly stops the port: it needs a decision before it can be generated.
var Inputs = []Input{
	{Field: "Module", Const: "INPUT_ID_MODULE", Label: "Module", Kind: Number},
	{Field: "ShaftAngle", Const: "INPUT_ID_SHAFT_ANGLE", Label: "Shaft Angle", Kind: Angle},
	{Field: "Teeth", Const: "INPUT_ID_DRIVING_TEETH", Label: "Driving Gear Teeth", Kind: Number},
	{Field: "MateTeeth", Const: "INPUT_ID_PINION_TEETH", Label: "Pinion Gear Teeth", Kind: Number},
	{Field: "BaseHeight", Const: "INPUT_ID_DRIVING_BASE_HEIGHT", Label: "Driving Gear Base Height", Kind: Length},
	{Field: "MateBaseHeight", Const: "INPUT_ID_PINION_BASE_HEIGHT", Label: "Pinion Gear Base Height", Kind: Length},
	{Field: "EnableBore", Const: "INPUT_ID_BORE_ENABLE", Label: "Enable Bore", Kind: Bool},
	{Field: "BoreDiameter", Const: "INPUT_ID_DRIVING_BORE", Label: "Driving Gear Bore Diameter", Kind: Length},
	{Field: "MateBoreDiameter", Const: "INPUT_ID_PINION_BORE", Label: "Pinion Gear Bore Diameter", Kind: Length},
	{Field: "FaceWidth", Const: "INPUT_ID_FACE_WIDTH", Label: "Face Width", Kind: Length},
	{Field: "ToothSpacing", Const: "INPUT_ID_TOOTH_SPACING", Label: "Tooth Spacing", Kind: Length},
	{Const: "INPUT_ID_TOOTH_TYPE", Label: "Tooth Type", Kind: ToothType},
	{Field: "SpiralAngle", Const: "INPUT_ID_SPIRAL_ANGLE", Label: "Mean Spiral Angle", Kind: Angle,
		Fallback: 35 * math.Pi / 180},
	{Field: "SpiralHand", Const: "INPUT_ID_HAND", Label: "Hand of Spiral", Kind: Hand},
	{Field: "CutterRadius", Const: "INPUT_ID_CUTTER_RADIUS", Label: "Cutter Radius", Kind: Length},
	{Field: "ToeExtension", Const: "INPUT_ID_TOE_EXTENSION", Label: "Toe Extension (%)", Kind: Number},
	{Field: "ToeRadius", Const: "INPUT_ID_DRIVING_TOE_RADIUS", Label: "Driving Gear Toe Radius", Kind: Length},
	{Field: "MateToeRadius", Const: "INPUT_ID_PINION_TOE_RADIUS", Label: "Pinion Gear Toe Radius", Kind: Length},
}

// GalleryOnly lists the Params fields with no dialog input. PressureAngle and Samples keep the gallery default.
// Placement is the decad document's placement and has no Fusion counterpart: the add-in takes a Target Plane and a
// Center Point instead.
var GalleryOnly = []string{"PressureAngle", "Samples", "Placement"}

// FeatureSpec is what the port translates from the gallery model: the whole build of a straight or spiral pair, in
// the order the gallery performs it.
var FeatureSpec = go2py.Spec{
	Prefix: "_",
	Structs: []string{"axialPoint", "shaftFrame", "dimensions", "memberBuildPlan", "GearPair", "pairContainers",
		"spiralSliceFrame", "spiralTrace", "serialEnds", "spiralHeelSection"},
	TypeOnly:   []string{"Params"},
	Mutable:    []string{"axialPoint", "Params", "dimensions", "shaftFrame"},
	Aliases:    map[string]string{"joinOrder": "int"},
	IotaConsts: []string{"joinOneByOne", "joinTeethGrouped", "joinTeethFirst", "joinTeethBalancedFirst"},
	FirstOnly:  []string{"spiralOvershoots", "orders", "sliceSpiralPointTooth:reach"},
	Values: []string{"spiralOvershoots", "spiralCrownPerRad", "verifiedSpiralAngle",
		"directionBits", "gridBits", "localShaft", "pairContainerName", "drivingContainerName", "pinionContainerName",
	},
	Bound: map[string]go2py.Bound{
		"buildSpiralMember":          {Python: "_build_spiral_member", Result: "decad.Body"},
		"buildSpiralPair":            {Python: "_build_spiral_pair", Result: "GearPair"},
		"validateSpiralBuild":        {Python: "_validate_spiral_build", Result: ""},
		"snap":                       {Python: "_fx.shaft_snap", Result: "[]axialPoint"},
		"selectEmbeddedToothProfile": {Python: "_fx.select_embedded_tooth_profile", Result: "sketch.Profile"},
	},
	Funcs: []string{
		"finite", "derive", "defineParameters", "unitOf", "recordParameters", "Build", "BuildPair", "prepareBuild",
		"buildMember", "buildStraightMember", "placePair", "resolvedBaseHeight", "blankSection", "blankSectionOf",
		"revolveBlankIn", "revolveAxialSectionIn", "revolveSharedBlanks", "profileUses", "buildToeConeIn",
		"buildHeelConeIn", "endConeSection", "buildEndCone", "buildPointToothIn", "ResolveBoreDiameter",
		"maximumBoreDiameter", "cutBore", "newPairContainers", "uniqueChild", "addSketch", "addPlane", "moveBody",
		"newShaftFrame", "roundBits", "apexFrame", "heelStation", "buildSpiralMember", "buildSpiralMemberSerial",
		"buildSpiralMemberSerialTrace", "spiralJoinGroupsFirst", "buildSpiralMemberOvershoot", "buildSpiralTooth",
		"patternSpiralTooth", "trimSpiralToothBy", "sliceSpiralPointTooth", "spiralSliceSheet", "validateSpiralBuild",
		"buildSpiralTrace", "cutterIntersectionNear", "twistSpiralSlabs", "crownSpiralSlabs", "spiralHeelRootAnchor",
		"spiralLoftFaces", "buildSerialEnds", "DefaultParams", "buildSpiralPair", "BuildPointTooth", "buildBackConeToothProfile", "buildVirtualToothProfile", "local", "axis", "sketchAxis", "unit", "vec",
		"sectionXY", "revolveAxialSection", "buildShiftedEndCone",
	},
	Roles: map[string]go2py.Role{
		"buildSpiralMemberSerialTrace:decad.Remove#1":                   {Feature: "Failed Spiral Member Removal"},
		"buildSpiralMemberOvershoot:decad.Union#1":                      {Feature: "Spiral Tooth Pair Join"},
		"buildSpiralMemberOvershoot:decad.Union#2":                      {Feature: "Spiral Teeth Join"},
		"buildSpiralMemberOvershoot:decad.Union#3":                      {Feature: "Spiral Tooth Pair Join"},
		"buildSpiralMemberOvershoot:decad.Union#4":                      {Feature: "Spiral Teeth Group Join"},
		"buildSpiralMemberOvershoot:decad.Union#5":                      {Feature: "Spiral Tooth Join"},
		"buildSpiralMemberOvershoot:decad.Union#6":                      {Feature: "Spiral Teeth Join"},
		"buildSpiralMemberOvershoot:decad.Union#7":                      {Feature: "Spiral Tooth Join"},
		"buildSpiralMemberOvershoot:decad.Intersect#1":                  {Feature: "Spiral Member Cap Trim"},
		"buildSpiralTooth:decad.LoftFaces#1":                            {Feature: "Spiral Tooth Loft", Body: "Spiral Tooth"},
		"buildSpiralTooth:decad.Remove#1":                               {Feature: "Spiral Slab Removal"},
		"crownSpiralSlabs:decad.ScaledAt#1":                             {Feature: "Spiral Crown Scale"},
		"patternSpiralTooth:decad.Copy#1":                               {Feature: "Spiral Tooth Copy"},
		"sliceSpiralPointTooth:decad.Split#1":                           {Feature: "Spiral Slice"},
		"sliceSpiralPointTooth:decad.Remove#1":                          {Feature: "Short Slice Sheet Removal"},
		"sliceSpiralPointTooth:decad.Remove#2":                          {Feature: "Missed Slice Sheet Removal"},
		"sliceSpiralPointTooth:decad.Remove#3":                          {Feature: "Refused Slice Sheet Removal"},
		"sliceSpiralPointTooth:decad.Remove#4":                          {Feature: "Spiral Apex Scrap Removal"},
		"spiralSliceSheet:sketch.World.CreateSketch#1":                  {Feature: "Slice Sheet"},
		"spiralSliceSheet:decad.Extrude#1":                              {Feature: "Slice Sheet Extrude", Body: "Slice Sheet"},
		"trimSpiralToothBy:decad.Cut#1":                                 {Feature: "Spiral Tooth Toe Trim"},
		"trimSpiralToothBy:decad.Intersect#1":                           {Feature: "Spiral Tooth Heel Trim"},
		"twistSpiralSlabs:decad.Transform#1":                            {Feature: "Spiral Slab Twist"},
		"buildSpiralPair:decad.MirroredCopy#1":                          {Feature: "Spiral Driving Gear Mirror", Body: "Spiral Driving Gear"},
		"buildSpiralPair:decad.Transform#1":                             {Feature: "Pinion Tooth Phase Turn"},
		"buildSpiralPair:decad.Transform#2":                             {Feature: "Pinion Shaft Turn"},
		"BuildPair:decad.Transform#1":                                   {Feature: "Pinion Tooth Phase Turn"},
		"placePair:decad.Transform#1":                                   {Feature: "Driving Gear Phase Turn"},
		"buildStraightMember:decad.Cut#1":                               {Feature: "Tooth Toe Trim"},
		"buildStraightMember:decad.Intersect#1":                         {Feature: "Tooth Heel Trim"},
		"buildStraightMember:decad.PatternCopies#1":                     {Feature: "Tooth Pattern"},
		"buildStraightMember:decad.Union#1":                             {Feature: "Teeth Join"},
		"revolveAxialSectionIn:sketch.World.CreateSketch#1":             {Feature: "Axial Section", Qualify: "label"},
		"revolveAxialSectionIn:decad.Revolve#1":                         {Feature: "Revolve", Qualify: "label"},
		"revolveSharedBlanks:sketch.World.CreateSketch#1":               {Feature: "Gear Profiles"},
		"revolve:decad.Revolve#1":                                       {Feature: "Revolve", Qualify: "label"},
		"buildPointToothIn:decad.LoftFromPoint#1":                       {Feature: "Tooth Loft", Body: "Tooth"},
		"cutBore:sketch.World.CreatePlaneFromFrame#1":                   {Feature: "Bore Plane"},
		"cutBore:sketch.World.CreateSketch#1":                           {Feature: "Bore Profile"},
		"cutBore:decad.Extrude#1":                                       {Feature: "Bore Extrude", Body: "Bore Tool"},
		"cutBore:decad.Cut#1":                                           {Feature: "Bore Cut"},
		"buildBackConeToothProfile:sketch.World.CreatePlaneFromFrame#1": {Feature: "Tooth Plane"},
		"buildVirtualToothProfile:sketch.World.CreateSketch#1":          {Feature: "Tooth Profile"},
	},
	External: map[string]go2py.External{
		"placement.Place":         {Python: "_fx.placement_place", Results: []string{"decad.Body"}},
		"spurgear.InvoluteFlanks": {Python: "_spur.involute_flanks", Results: []string{"[][2]float", "[][2]float"}},
	},
}
