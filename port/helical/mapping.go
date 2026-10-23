// Package helical ports the decad-gallery helical gear to the Fusion add-in file
// lib/geargen/helicalgear.py.
//
// The gallery helical gear is the spur gear plus one input, HelixAngle, and a loft
// between two twisted tooth sections. The add-in builds on the generated spur add-in,
// so only the helix input comes from the gallery source here; the spur parameters,
// involute math and root fillet come from port/spur.
package helical

// Input maps the gallery HelixAngle field to its dialog input and Fusion parameter.
var Input = struct {
	GoField, Label        string
	InputConst, InputID   string
	ParamConst, ParamName string
	DialogUnit            string
}{
	GoField: "HelixAngle", Label: "Helix Angle",
	InputConst: "INPUT_ID_HELIX_ANGLE", InputID: "helixAngle",
	ParamConst: "PARAM_HELIX_ANGLE", ParamName: "HelixAngle",
	DialogUnit: "deg",
}

// ExpectedOps is the sequence of decad feature operations the fragment implements.
// The spur fragment implements the finishing operations, which the gallery calls in
// the spurgear package.
var ExpectedOps = []string{
	"Loft",    // twisted tooth
	"Extrude", // root disc
	"Union",   // join
	"Fillet",  // root fillet, scaled by cos(HelixAngle)
}

// ReviewedFuncs are the gallery functions the fragment's Fusion construction follows.
var ReviewedFuncs = []string{
	"Build", "Assemble", "PreparePattern", "FinishAssembly", "buildTooth", "buildRootDisc", "filletRootEdges",
}
