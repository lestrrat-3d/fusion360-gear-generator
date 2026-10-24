// Package herringbone ports the decad-gallery herringbone gear to the Fusion add-in
// file lib/geargen/herringbonegear.py.
//
// The gallery herringbone gear takes the helical gear's Params unchanged and differs
// only in its tooth: it lofts half the twist to the mid-plane, mirrors that half and
// joins the two. The add-in builds on the generated helical add-in, so it has no
// dialog input or parameter of its own and the stored fragment carries the whole file.
package herringbone

// ParamsAlias is the type the gallery Params must alias, so that the dialog inputs
// the helical add-in registers are the inputs this gear needs.
const ParamsAlias = "helicalgear.Params"

// ExpectedOps is the sequence of decad feature operations the gallery Build performs
// in its own package. The helical package performs the pattern, join and finishing
// operations, and port/helical checks those.
var ExpectedOps = []string{
	"Union", // join the mirrored halves
}

// ReviewedFuncs are the gallery functions the fragment's Fusion construction follows.
var ReviewedFuncs = []string{"Build", "variant", "chevron"}
