// Package helical ports the decad-gallery helical gear to the Fusion add-in file
// lib/geargen/helicalgear.py.
//
// The gallery helical gear is the spur gear plus one input, HelixAngle, a lofted tooth and a
// root fillet scaled by cos(HelixAngle). The tooth and the fillet are translated from the
// gallery (FeatureSpec) and plug into the spur build as a Variant; the rest comes from
// port/spur.
package helical

import "github.com/lestrrat-3d/fusion360-gear-generator/port/fusionexpr"

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

// Parameter is a parameter the gallery's Variant defines for the helical build, which the
// reader of the gallery's parameter table does not see.
type Parameter struct {
	// Table is the gallery name, Const and Name the Fusion identity.
	Table, Const, Name, Unit, Comment string
	// Expr is the gallery expression over table names.
	Expr string
}

// PlaneOffset is the distance from the lower cap to the twisted section. The herringbone
// gear passes half the thickness instead.
var PlaneOffset = Parameter{
	Table: "helix_plane_offset", Const: "PARAM_HELIX_PLANE_OFFSET", Name: "HelixPlaneOffset", Unit: "mm",
	Comment: "Distance from the lower cap to the twisted section", Expr: "thickness",
}

// FilletRadius is the root fillet radius, scaled by cos of the helix angle.
var FilletRadius = Parameter{
	Table: "helical_fillet_radius", Const: "PARAM_HELICAL_FILLET_RADIUS", Name: "HelicalFilletRadius", Unit: "mm",
	Comment: "Radius of the helical root fillets", Expr: "fillet_radius * cos(helix_angle)",
}

// Constants maps a gallery table name to the Python constant of its Fusion parameter, for the
// names the helical expressions use.
var Constants = map[string]string{
	"thickness": "PARAM_THICKNESS", "fillet_radius": "PARAM_FILLET_RADIUS", "helix_angle": "PARAM_HELIX_ANGLE",
}

var kinds = map[string]fusionexpr.Kind{
	"thickness": fusionexpr.KindLength, "fillet_radius": fusionexpr.KindLength, "helix_angle": fusionexpr.KindAngle,
}

// ReviewedFuncs are the gallery functions whose code the parameters above and the Variant
// wiring in the fragment copy. The fragment records a digest of them (reviewed.json); a
// change stops the port until it is reviewed.
var ReviewedFuncs = []string{"Build", "Variant"}
