// Package bevel ports the decad-gallery bevel gear to the Fusion add-in file
// lib/geargen/bevelgear.py.
//
// The add-in builds the pair with Fusion calls the stored fragment holds. From the
// gallery source the port takes the dialog inputs and their defaults, and it stops
// when the gallery's inputs, build operations or reviewed code change.
package bevel

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
	{Field: "SpiralAngle", Const: "INPUT_ID_SPIRAL_ANGLE", Label: "Mean Spiral Angle", Kind: Angle},
	{Field: "SpiralHand", Const: "INPUT_ID_HAND", Label: "Hand of Spiral", Kind: Hand},
	{Field: "CutterRadius", Const: "INPUT_ID_CUTTER_RADIUS", Label: "Cutter Radius", Kind: Length},
	{Field: "ToeExtension", Const: "INPUT_ID_TOE_EXTENSION", Label: "Toe Extension (%)", Kind: Number},
	{Field: "ToeRadius", Const: "INPUT_ID_DRIVING_TOE_RADIUS", Label: "Driving Gear Toe Radius", Kind: Length},
	{Field: "MateToeRadius", Const: "INPUT_ID_PINION_TOE_RADIUS", Label: "Pinion Gear Toe Radius", Kind: Length},
}

// GalleryOnly lists the Params fields with no dialog input. PressureAngle and Samples
// are fixed in lib/geargen/spurproxy.py, which the tests compare with the gallery defaults.
// Placement is the decad document's placement and has no Fusion counterpart.
var GalleryOnly = []string{"PressureAngle", "Samples", "Placement"}

// ExpectedOps is the sequence of decad feature operations the gallery Build performs,
// following both its straight and its spiral branch. The port stops when it changes,
// because the Fusion construction in the fragment was reviewed against this sequence.
var ExpectedOps = []string{
	"Revolve", "Revolve", "Extrude", "Revolve", "Revolve", "Cut", "Revolve", "Revolve", "Intersect",
	"Union", "Union", "Union", "Intersect", "Revolve", "Revolve", "Cut", "Revolve", "Intersect", "Union",
	"Union", "Extrude", "Cut",
}

// ReviewedFuncs are the gallery functions the fragment's Fusion construction follows.
// The fragment records a digest of their code (reviewed.json); a change to any of them
// stops the port until the fragment is reviewed and the digest accepted.
var ReviewedFuncs = []string{
	"Build", "BuildPair", "prepareBuild", "buildMember", "buildStraightMember", "buildSpiralMember",
	"derive", "defineParameters",
}
