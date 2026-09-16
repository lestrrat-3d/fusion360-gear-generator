package bevelgear_test

import (
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// bevelParams is the dialog's own defaults with a case's overrides applied. Writing
// every case as a delta from the shipped defaults is what makes a case say what
// it is FOR, and it keeps the seventeen inputs from being restated twenty times.
//
// The keys are the proof's own; they are not the dialog input ids, which are
// reproduced in the step list where the emit stage can read them.
func bevelParams(over map[string]float64) map[string]float64 {
	base := map[string]float64{
		"module":            1,
		"drivingTeeth":      31,
		"pinionTeeth":       31,
		"shaftAngle":        90,
		"drivingBaseHeight": 0,
		"pinionBaseHeight":  0,
		"boreEnable":        1,
		"drivingBore":       0,
		"pinionBore":        0,
		"faceWidth":         0,
		"toothSpacing":      0,
		"spiralAngle":       35,
		"handLeft":          0,
		"cutterRadius":      0,
		"toeExtension":      0,
		"drivingToeRadius":  0,
		"pinionToeRadius":   0,
		"gear":              0, // 0 = Pinion, 1 = Driving
		"declaredRefusal":   0,
	}
	for k, v := range over {
		base[k] = v
	}
	return base
}

func bevelSketchCase(name string, over map[string]float64) proofkit.Case {
	return proofkit.Case{Name: name, Params: bevelParams(over)}
}

func bevelSolidCase(name string, over map[string]float64) proofkit3d.Case {
	return proofkit3d.Case{Name: name, Params: bevelParams(over)}
}

// bevelAnchorCases sweeps nothing the Anchor Sketch depends on, because it depends
// on nothing: the line is seeded at a fixed 10 mm and its dimension locks that
// seeded length, the value being arbitrary. The two cases exist so the sketch
// is proved at more than one placement of the projected centre, which is the
// only thing about it that varies in Fusion.
var bevelAnchorCases = []proofkit.Case{
	bevelSketchCase("default", nil),
	bevelSketchCase("offset_centre", map[string]float64{"centreX": 12.5, "centreY": -4}),
}

// bevelLatticeCases is the §2 regime. The Shaft Angle range is covered at both ends
// and at the default; each tooth-count ratio is covered from BOTH directions,
// because the Maximum Face Width binds on whichever bevelSide carries the smaller
// pitch diameter and a table that only ever made the pinion the smaller one
// would pass a bound written with the pinion's diameter by name; and the two
// virtual-tooth-count cases the spec names are carried here as well as in the
// solid table.
var bevelLatticeCases = []proofkit.Case{
	bevelSketchCase("default_31_31_at_90", nil),
	// The Shaft Angle floor the spec documents, and the one configuration this
	// net cannot reach. Measured here at conditioning 2.83e-05 against the
	// engine's 4e-05 trust floor — the same reading the spec records for two of
	// the three independently written lattices. It stays in the table as a
	// DECLARED REFUSAL rather than being dropped or having the range narrowed
	// around it: a configuration the spec admits and this particular net cannot
	// reach is a property of the net.
	bevelSketchCase("shaft_angle_30_floor", map[string]float64{"shaftAngle": 30, "declaredRefusal": 1}),
	bevelSketchCase("shaft_angle_35", map[string]float64{"shaftAngle": 35}),
	bevelSketchCase("shaft_angle_142", map[string]float64{"shaftAngle": 142}),
	bevelSketchCase("shaft_angle_150_ceiling", map[string]float64{"shaftAngle": 150}),
	bevelSketchCase("ratio_driving_17_pinion_31", map[string]float64{"drivingTeeth": 17}),
	bevelSketchCase("ratio_driving_31_pinion_17", map[string]float64{"pinionTeeth": 17}),
	bevelSketchCase("ratio_43_31_at_75", map[string]float64{"drivingTeeth": 43, "shaftAngle": 75}),
	bevelSketchCase("teeth_16_12_at_90", map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12}),
	bevelSketchCase("teeth_4_4_at_90", map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4}),
	bevelSketchCase("tooth_spacing_positive", map[string]float64{"toothSpacing": 0.4}),
	bevelSketchCase("tooth_spacing_ratio_pair", map[string]float64{
		"drivingTeeth": 43, "shaftAngle": 75, "toothSpacing": 0.5}),
	bevelSketchCase("toe_extension_50", map[string]float64{"toeExtension": 50}),
	bevelSketchCase("toe_extension_100", map[string]float64{"toeExtension": 100}),
	bevelSketchCase("toe_extension_100_ratio_pair", map[string]float64{
		"drivingTeeth": 43, "shaftAngle": 75, "toeExtension": 100}),
	bevelSketchCase("base_heights_user", map[string]float64{
		"drivingBaseHeight": 5, "pinionBaseHeight": 5}),
	bevelSketchCase("face_width_user", map[string]float64{"faceWidth": 4}),
	bevelSketchCase("toe_radius_user", map[string]float64{
		"drivingToeRadius": 4, "pinionToeRadius": 4}),
}

// bevelToothCases draws the virtual spur tooth for both members of each pair, which
// is what makes the two virtual-tooth-count cases say anything: on 16/12 one
// member's exact count is an integer and the other's is not, and reading only
// one of them would not tell a rounded count from an exact one.
var bevelToothCases = []proofkit.Case{
	bevelSketchCase("default_pinion", map[string]float64{"gear": 0}),
	bevelSketchCase("default_driving", map[string]float64{"gear": 1}),
	bevelSketchCase("teeth_16_12_pinion", map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12, "gear": 0}),
	bevelSketchCase("teeth_16_12_driving", map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12, "gear": 1}),
	bevelSketchCase("teeth_4_4_pinion", map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}),
	bevelSketchCase("teeth_4_4_driving", map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 1}),
	bevelSketchCase("ratio_43_31_pinion", map[string]float64{"drivingTeeth": 43, "shaftAngle": 75, "gear": 0}),
	bevelSketchCase("ratio_43_31_driving", map[string]float64{"drivingTeeth": 43, "shaftAngle": 75, "gear": 1}),
	bevelSketchCase("module_4_pinion", map[string]float64{"module": 4, "gear": 0}),
	bevelSketchCase("shaft_angle_35_driving", map[string]float64{"shaftAngle": 35, "gear": 1}),
	// The other side of the embedded branch: at Shaft Angle 142 the driving
	// bevelSide's pitch cone angle carries its virtual tooth count past 95, the root
	// circle rises above the base circle even with the sink applied, and the
	// drawer draws the tooth with no connecting lines at all.
	bevelSketchCase("shaft_angle_142_driving", map[string]float64{"shaftAngle": 142, "gear": 1}),
}

// bevelHexCases proves the per-bevelSide Profile sketch, which is the same six vertices
// for every configuration but a different shape, so the sweep is over the
// figures whose hexagon is least like the default's.
var bevelHexCases = []proofkit.Case{
	bevelSketchCase("default_pinion", map[string]float64{"gear": 0}),
	bevelSketchCase("default_driving", map[string]float64{"gear": 1}),
	bevelSketchCase("ratio_43_31_at_75_driving", map[string]float64{
		"drivingTeeth": 43, "shaftAngle": 75, "gear": 1}),
	bevelSketchCase("toe_extension_100_pinion", map[string]float64{"toeExtension": 100, "gear": 0}),
	bevelSketchCase("teeth_4_4_pinion", map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}),
	bevelSketchCase("shaft_angle_35_driving", map[string]float64{"shaftAngle": 35, "gear": 1}),
}

// bevelTraceCases is the spiral cutter arc. Its regime is the Mean Spiral Angle
// range [0, 60) and the two hands, on both members of a pair, because the hand
// sign is negated for the pinion and an equal-teeth pair's two traces must come
// out exact mirror images.
var bevelTraceCases = []proofkit.Case{
	bevelSketchCase("psi_35_right_pinion", map[string]float64{"gear": 0}),
	bevelSketchCase("psi_35_right_driving", map[string]float64{"gear": 1}),
	bevelSketchCase("psi_35_left_pinion", map[string]float64{"handLeft": 1, "gear": 0}),
	bevelSketchCase("psi_35_left_driving", map[string]float64{"handLeft": 1, "gear": 1}),
	bevelSketchCase("psi_zero_limit_driving", map[string]float64{"spiralAngle": 0, "gear": 1}),
	bevelSketchCase("psi_55_driving", map[string]float64{"spiralAngle": 55, "gear": 1}),
	bevelSketchCase("psi_35_ratio_pinion", map[string]float64{
		"drivingTeeth": 43, "shaftAngle": 75, "gear": 0}),
	bevelSketchCase("psi_35_ratio_driving", map[string]float64{
		"drivingTeeth": 43, "shaftAngle": 75, "gear": 1}),
	bevelSketchCase("cutter_radius_user_driving", map[string]float64{"cutterRadius": 30, "gear": 1}),
}

// bevelSolidCases runs at Module 4 to 8 and never at Module 1: decad's mesh bound
// has an absolute floor, so a figure small enough brings every measurement
// inside it and the gate reports Suspect on geometry that is in fact correct.
// Module is a pure scale on this figure, so a case at Module 4 proves the same
// shape as one at Module 1 and clears the floor.
var bevelSolidCases = []proofkit3d.Case{
	bevelSolidCase("m4_31_31_at_90_pinion", map[string]float64{"module": 4, "gear": 0}),
	bevelSolidCase("m4_31_31_at_90_driving", map[string]float64{"module": 4, "gear": 1}),
	bevelSolidCase("m8_31_31_at_90_pinion", map[string]float64{"module": 8, "gear": 0}),
	bevelSolidCase("m4_16_12_at_90_pinion", map[string]float64{
		"module": 4, "drivingTeeth": 16, "pinionTeeth": 12, "gear": 0}),
	bevelSolidCase("m4_16_12_at_90_driving", map[string]float64{
		"module": 4, "drivingTeeth": 16, "pinionTeeth": 12, "gear": 1}),
	bevelSolidCase("m4_4_4_at_90_pinion", map[string]float64{
		"module": 4, "drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}),
	bevelSolidCase("m4_43_31_at_75_driving", map[string]float64{
		"module": 4, "drivingTeeth": 43, "shaftAngle": 75, "gear": 1}),
	bevelSolidCase("m4_shaft_angle_35_pinion", map[string]float64{
		"module": 4, "shaftAngle": 35, "gear": 0}),
	bevelSolidCase("m4_shaft_angle_142_driving", map[string]float64{
		"module": 4, "shaftAngle": 142, "gear": 1}),
	bevelSolidCase("m4_toe_extension_100_pinion", map[string]float64{
		"module": 4, "toeExtension": 100, "gear": 0}),
}

// bevelBoreCases adds the two ends of the bore branch to the solid regime: the
// auto-calculated diameter, a user diameter, and the unchecked Enable Bore that
// cuts nothing at all.
var bevelBoreCases = []proofkit3d.Case{
	bevelSolidCase("m4_auto_bore_pinion", map[string]float64{"module": 4, "gear": 0}),
	bevelSolidCase("m4_auto_bore_driving", map[string]float64{"module": 4, "gear": 1}),
	bevelSolidCase("m4_user_bore_pinion", map[string]float64{
		"module": 4, "pinionBore": 12, "gear": 0}),
	bevelSolidCase("m4_bore_disabled_pinion", map[string]float64{
		"module": 4, "boreEnable": 0, "gear": 0}),
	bevelSolidCase("m4_heel_bound_bore_driving", map[string]float64{
		"module": 4, "shaftAngle": 35, "gear": 1}),
}

// bevelPatternCases stays deliberately small: the step is the one serial runner in
// this package, so every case it carries is wall time nothing else pays.
var bevelPatternCases = []proofkit3d.Case{
	bevelSolidCase("m4_31_31_pinion", map[string]float64{"module": 4, "gear": 0}),
	bevelSolidCase("m4_16_12_driving", map[string]float64{
		"module": 4, "drivingTeeth": 16, "pinionTeeth": 12, "gear": 1}),
	bevelSolidCase("m4_4_4_pinion", map[string]float64{
		"module": 4, "drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}),
}

// bevelMeshCases covers the driving bevelSide's half-tooth-pitch offset and the pinion's,
// which is zero by default, so the step sees both a real rotation and the
// no-op one the framework helper absorbs.
var bevelMeshCases = []proofkit3d.Case{
	bevelSolidCase("m4_driving_half_pitch", map[string]float64{"module": 4, "gear": 1}),
	bevelSolidCase("m4_pinion_no_phase", map[string]float64{"module": 4, "gear": 0}),
	bevelSolidCase("m4_4_4_driving_half_pitch", map[string]float64{
		"module": 4, "drivingTeeth": 4, "pinionTeeth": 4, "gear": 1}),
}

// bevelSpiralCases is the psi > 0 regime of the tooth-body chain. psi = 0 is not in
// it: at psi = 0 the hook returns before any of this construction runs, which
// the straight-tooth steps are what prove.
var bevelSpiralCases = []proofkit3d.Case{
	bevelSolidCase("m4_psi_35_right_driving", map[string]float64{"module": 4, "gear": 1}),
	bevelSolidCase("m4_psi_35_right_pinion", map[string]float64{"module": 4, "gear": 0}),
	bevelSolidCase("m4_psi_35_left_driving", map[string]float64{"module": 4, "handLeft": 1, "gear": 1}),
	bevelSolidCase("m4_psi_55_driving", map[string]float64{"module": 4, "spiralAngle": 55, "gear": 1}),
	bevelSolidCase("m4_psi_10_driving", map[string]float64{"module": 4, "spiralAngle": 10, "gear": 1}),
	bevelSolidCase("m4_psi_35_ratio_pinion", map[string]float64{
		"module": 4, "drivingTeeth": 43, "shaftAngle": 75, "gear": 0}),
	bevelSolidCase("m4_psi_35_ratio_driving", map[string]float64{
		"module": 4, "drivingTeeth": 43, "shaftAngle": 75, "gear": 1}),
}
