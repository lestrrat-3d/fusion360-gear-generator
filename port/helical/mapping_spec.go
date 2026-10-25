package helical

import "github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"

// FeatureSpec is what the port translates from the gallery's helical model: the lofted tooth and
// the root fillet. They plug into the spur build as a Variant, and the spur's translation
// (port/spur) supplies the rest. lib/geargen/spurgear.py provides the spur functions they call.
var FeatureSpec = go2py.Spec{
	Prefix: "_",
	Funcs:  []string{"ValidateHelixAngle", "LoftTooth", "filletRootEdges"},
	Roles: map[string]go2py.Role{
		"LoftTooth:decad.Loft#1":         {Feature: "Twisted Tooth Loft", Body: "Tooth"},
		"filletRootEdges:decad.Fillet#1": {Feature: "Root Fillet"},
	},
	External:      SpurExternals("spurgear.BuildToothSection", "spurgear.ToothInput.PlaneAt"),
	ExternalTypes: SpurExternalTypes,
}

// SpurExternals returns the entries of SpurExternal named by keys, for a Spec that calls only those. A Spec must
// list only the externals its translation calls.
func SpurExternals(keys ...string) map[string]go2py.External {
	out := make(map[string]go2py.External, len(keys))
	for _, k := range keys {
		ext, ok := SpurExternal[k]
		if !ok {
			panic("helical.SpurExternals: no entry " + k)
		}
		out[k] = ext
	}
	return out
}

// SpurExternal lists the spur functions and methods the helical and herringbone models call, and the Python
// that stands for each in the generated add-in, which imports lib/geargen/spurgear.py as _spur.
var SpurExternal = map[string]go2py.External{
	"spurgear.BuildToothSection": {
		Python: "_spur.build_tooth_section", Results: []string{"sketch.Sketch", "sketch.Profile"},
	},
	"spurgear.ToothInput.Shifted": {Python: "_spur.tooth_input_shifted", Results: []string{"r3.Frame"}, Method: true},
	"spurgear.ToothInput.PlaneAt": {Python: "_spur.tooth_input_plane_at", Results: []string{"sketch.Plane"}, Method: true},
}

// SpurExternalTypes lists the spur structs the helical model reads, with the fields it reads.
var SpurExternalTypes = map[string]map[string]string{
	"spurgear.ToothInput": {
		"Doc": "decad.Document", "World": "sketch.World", "Params": "spurgear.Params",
		"Frame": "r3.Frame", "Lower": "sketch.Plane",
	},
	"spurgear.Params": {},
}
