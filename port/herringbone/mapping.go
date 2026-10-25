// Package herringbone ports the decad-gallery herringbone gear to the Fusion add-in
// file lib/geargen/herringbonegear.py.
//
// The gallery herringbone gear takes the helical gear's Params unchanged and differs
// only in its tooth: it lofts half the twist to the mid-plane, mirrors that half and
// joins the two. The chevron tooth is translated from the gallery (FeatureSpec). The
// add-in builds on the generated helical add-in, so it has no dialog input of its own.
package herringbone

import (
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/helical"
)

// ParamsAlias is the type the gallery Params must alias, so that the dialog inputs
// the helical add-in registers are the inputs this gear needs.
const ParamsAlias = "helicalgear.Params"

// PlaneOffset is the helical plane offset the gallery's variant passes: half the thickness.
var PlaneOffset = helical.Parameter{
	Table: helical.PlaneOffset.Table, Const: helical.PlaneOffset.Const, Name: helical.PlaneOffset.Name,
	Unit: helical.PlaneOffset.Unit, Comment: helical.PlaneOffset.Comment, Expr: "thickness / 2",
}

// ReviewedFuncs are the gallery functions whose code PlaneOffset and the Variant wiring in
// the fragment copy. The fragment records a digest of them (reviewed.json).
var ReviewedFuncs = []string{"variant"}

// FeatureSpec is what the port translates from the gallery's herringbone model: the
// chevron tooth. It calls the helical loft and the spur's tooth input.
var FeatureSpec = go2py.Spec{
	Prefix: "_",
	Funcs:  []string{"chevron"},
	Roles: map[string]go2py.Role{
		"chevron:decad.MirroredCopy#1": {Feature: "Tooth Half Mirror", Body: "Tooth Body (Mirrored)"},
		"chevron:decad.Union#1":        {Feature: "Tooth Halves Join"},
	},
	External:      external(),
	ExternalTypes: helical.SpurExternalTypes,
}

func external() map[string]go2py.External {
	out := helical.SpurExternals("spurgear.ToothInput.Shifted")
	out["helicalgear.LoftTooth"] = go2py.External{Python: "_helical.loft_tooth", Results: []string{"decad.Body"}}
	return out
}
