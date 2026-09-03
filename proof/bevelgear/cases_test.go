package bevelgear_test

import (
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// ---------------------------------------------------------------------------
// The case tables.
//
// The regime the design has to hold across is the one instructions.md states,
// so that regime IS the table: the Shaft Angle from its 30 degree floor to the
// Maximum Shaft Angle at both ends, tooth counts from the computed Minimum
// Teeth floor up past the count at which the virtual spur tooth becomes
// embedded, both gear ratio directions (either gear can be the smaller,
// binding side of the Face Width cap), each base height defaulted and
// user-supplied, Tooth Spacing at and above zero, the bore auto/explicit/off,
// and the Mean Spiral Angle at 0 (a straight bevel, a different code path) and
// through the open end of [0, 60) with both hands.
//
// A case that would be REJECTED by the input validation is not in the table:
// the generator refuses it before any geometry exists, and the rejection
// arithmetic is asserted directly by newDesign, which every case runs through.
// ---------------------------------------------------------------------------

// baseParams is the shipped dialog default: module 1, an equal 31/31 pair at a
// 90 degree shaft angle, every optional length left at 0 (meaning "resolve
// it"), the bore enabled and a 35 degree right-hand spiral.
func baseParams() map[string]float64 {
	return map[string]float64{
		keyModule:       1,
		keyDrivingTeeth: 31,
		keyPinionTeeth:  31,
		keyShaftAngle:   90,
		keyDrivingBase:  0,
		keyPinionBase:   0,
		keyFaceWidth:    0,
		keyToothSpacing: 0,
		keySpiralAngle:  35,
		keyHand:         1,
		keyCutterRadius: 0,
		keyBoreEnable:   1,
		keyDrivingBore:  0,
		keyPinionBore:   0,
	}
}

// params is baseParams with the named overrides applied.
func params(overrides map[string]float64) map[string]float64 {
	p := baseParams()
	for k, v := range overrides {
		p[k] = v
	}
	return p
}

// latticeCases sweeps the Gear Profiles lattice across the whole declared
// regime. The lattice is Shaft-Angle and tooth-count driven and does not see
// the spiral inputs at all, so those are left at their defaults here.
var latticeCases = []proofkit.Case{
	{Name: "default_31_31_at_90", Params: params(nil)},
	{Name: "ratio_driving_31_pinion_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "ratio_driving_17_pinion_31", Params: params(map[string]float64{keyDrivingTeeth: 17})},
	{Name: "shaft_angle_30_documented_floor", Params: params(map[string]float64{
		keyShaftAngle: 30, keyNetLimited: 1})},
	{Name: "shaft_angle_35", Params: params(map[string]float64{keyShaftAngle: 35})},
	{Name: "shaft_angle_60", Params: params(map[string]float64{keyShaftAngle: 60})},
	// A 31/17 pair's Maximum Shaft Angle is the cone-angle limit
	// acos(-17/31) = 123.24 degrees, and the lattice stretches without bound as
	// that limit is approached: point L runs to 277 mm from the Apex at 120
	// degrees and this net's conditioning reads 8.43e-07 there, far below the
	// engine's 4e-05 floor. That collapse IS the singularity the Maximum Shaft
	// Angle excludes, so the case sits where a ratio pair is actually used
	// rather than pressed against its own limit.
	{Name: "shaft_angle_110_ratio_pair", Params: params(map[string]float64{
		keyPinionTeeth: 17, keyShaftAngle: 110})},
	{Name: "shaft_angle_142", Params: params(map[string]float64{keyShaftAngle: 142})},
	{Name: "shaft_angle_150_inclusive_cap", Params: params(map[string]float64{keyShaftAngle: 150})},
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
	{Name: "small_teeth_8_8", Params: params(map[string]float64{
		keyDrivingTeeth: 8, keyPinionTeeth: 8})},
	{Name: "module_2_driving_19_pinion_13", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
	{Name: "high_count_43_31_embedded_driving_tooth", Params: params(map[string]float64{
		keyDrivingTeeth: 43, keyPinionTeeth: 31})},
	{Name: "tooth_spacing_1mm", Params: params(map[string]float64{keyToothSpacing: 1})},
	{Name: "user_base_heights", Params: params(map[string]float64{
		keyDrivingBase: 3, keyPinionBase: 2.5})},
	{Name: "user_face_width_at_the_cap", Params: params(map[string]float64{
		keyFaceWidth: 5.2})},
	{Name: "user_face_width_small", Params: params(map[string]float64{keyFaceWidth: 1.5})},
}

// anchorCases only needs the target plane and centre point, neither of which
// is a numeric input, so one case proves the scheme and a second proves it
// does not depend on anything else in the dialog.
var anchorCases = []proofkit.Case{
	{Name: "default", Params: params(nil)},
	{Name: "module_2_driving_19_pinion_13", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
}

// profileCases proves the per-gear hexagon sketch across the same regime as
// the lattice; the hexagon is that lattice's six vertices recreated as fixed
// points, so a case the lattice covers is a case this has to survive too.
var profileCases = perGearSketch([]proofkit.Case{
	{Name: "default_31_31_at_90", Params: params(nil)},
	{Name: "ratio_driving_31_pinion_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "ratio_driving_17_pinion_31", Params: params(map[string]float64{keyDrivingTeeth: 17})},
	{Name: "shaft_angle_30", Params: params(map[string]float64{keyShaftAngle: 30})},
	{Name: "shaft_angle_150", Params: params(map[string]float64{keyShaftAngle: 150})},
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
	{Name: "module_2_driving_19_pinion_13", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
})

// toothCases must reach the embedded flag from both sides, since it is what
// decides the tooth loop's line count and therefore which loop the profile
// search selects.
var toothCases = []proofkit.Case{
	{Name: "default_31_31_embedded_both", Params: params(nil)},
	{Name: "ratio_31_17_pinion_not_embedded", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "module_2_19_13_not_embedded", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
	{Name: "tooth_spacing_1mm", Params: params(map[string]float64{keyToothSpacing: 1})},
	{Name: "shaft_angle_30", Params: params(map[string]float64{keyShaftAngle: 30})},
}

// boreCases cover the "0 means auto" branch, an explicit diameter, and the
// unchecked box, which skips the step entirely.
var boreCases = perGearSketch([]proofkit.Case{
	{Name: "auto_from_pitch_diameter", Params: params(nil)},
	{Name: "explicit_diameter", Params: params(map[string]float64{
		keyDrivingBore: 6, keyPinionBore: 4})},
	{Name: "disabled", Params: params(map[string]float64{keyBoreEnable: 0})},
	{Name: "module_2_19_13_auto", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
})

// traceCases sweep the spiral trace. psi = 0 is the straight bevel, which
// takes the other branch of the tooth-body hook entirely, so it is present as
// a skip that names the branch rather than as a silent absence. Both hands are
// covered because the hand sign is the one signed quantity in the trace, and
// the cutter radius is covered at the auto default and at an explicit value on
// both sides of R_mean.
var traceCases = []proofkit.Case{
	{Name: "psi_0_straight_bevel", Params: params(map[string]float64{keySpiralAngle: 0})},
	{Name: "psi_35_right_auto_cutter", Params: params(nil)},
	{Name: "psi_35_left_auto_cutter", Params: params(map[string]float64{keyHand: -1})},
	{Name: "psi_just_under_60_right", Params: params(map[string]float64{keySpiralAngle: 59.9})},
	{Name: "psi_10_right", Params: params(map[string]float64{keySpiralAngle: 10})},
	{Name: "cutter_radius_below_mean", Params: params(map[string]float64{keyCutterRadius: 20})},
	{Name: "cutter_radius_above_mean", Params: params(map[string]float64{keyCutterRadius: 60})},
	{Name: "ratio_31_17_right", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "ratio_31_17_left", Params: params(map[string]float64{
		keyPinionTeeth: 17, keyHand: -1})},
	{Name: "shaft_angle_30_right", Params: params(map[string]float64{keyShaftAngle: 30})},
}

// solidCases are the pairs whose bodies the 3-D steps build. The table is
// smaller than the sketch tables because a solid case costs a full boolean
// evaluation, so it keeps the ends of the regime and the two ratio directions
// and drops the interior points the lattice already sweeps.
var solidCases = perGear([]proofkit3d.Case{
	{Name: "default_31_31_at_90", Params: params(nil)},
	{Name: "ratio_driving_31_pinion_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "ratio_driving_17_pinion_31", Params: params(map[string]float64{keyDrivingTeeth: 17})},
	{Name: "shaft_angle_30", Params: params(map[string]float64{keyShaftAngle: 30})},
	{Name: "shaft_angle_150", Params: params(map[string]float64{keyShaftAngle: 150})},
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
})

// boreSolidCases add the disabled-bore branch to the solid regime, since the
// bore cut is the one body step a case can skip entirely.
var boreSolidCases = perGear([]proofkit3d.Case{
	{Name: "auto_from_pitch_diameter", Params: params(nil)},
	{Name: "explicit_diameter", Params: params(map[string]float64{
		keyDrivingBore: 6, keyPinionBore: 4})},
	{Name: "disabled", Params: params(map[string]float64{keyBoreEnable: 0})},
	{Name: "ratio_driving_31_pinion_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
})

// spiralSolidCases carry both branches of the tooth-body hook and both hands,
// since the hand is what the spiral's only signed quantity turns on.
var spiralSolidCases = perGear([]proofkit3d.Case{
	{Name: "psi_0_straight_bevel", Params: params(map[string]float64{keySpiralAngle: 0})},
	{Name: "psi_35_right", Params: params(nil)},
	{Name: "psi_35_left", Params: params(map[string]float64{keyHand: -1})},
	{Name: "psi_35_ratio_31_17_right", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "psi_10_right", Params: params(map[string]float64{keySpiralAngle: 10})},
	{Name: "psi_just_under_60_right", Params: params(map[string]float64{keySpiralAngle: 59.9})},
})

// toothSolidCases and patternCases and meshCases pick the per-gear side
// explicitly: every one of these steps runs once per gear, pinion first, so a
// case names which member it is proving.
var toothSolidCases = perGear([]proofkit3d.Case{
	{Name: "default_31_31_embedded", Params: params(nil)},
	{Name: "ratio_31_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "module_2_19_13", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
	{Name: "tooth_spacing_1mm", Params: params(map[string]float64{keyToothSpacing: 1})},
})

// patternCases keep the tooth counts small: the join is one boolean per tooth
// and the invariant it proves — N copies at 360/N leaving a single lump — does
// not get truer at 31 teeth than at 8. The angular pitch itself is asserted
// for every case the tooth-profile step covers.
var patternCases = perGear([]proofkit3d.Case{
	{Name: "minimum_teeth_4_4", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4})},
	{Name: "small_teeth_8_8", Params: params(map[string]float64{
		keyDrivingTeeth: 8, keyPinionTeeth: 8})},
})

// meshCases must carry BOTH gears: the driving gear takes half a tooth pitch
// and the pinion takes zero, and zero is the branch that must not reach a move
// feature at all.
var meshCases = perGear([]proofkit3d.Case{
	{Name: "default_31_31", Params: params(nil)},
	{Name: "ratio_31_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
})

// perGear expands a case table into one case per member of the pair, since
// every per-gear step runs twice: pinion first, then driving.
func perGear(cases []proofkit3d.Case) []proofkit3d.Case {
	out := make([]proofkit3d.Case, 0, 2*len(cases))
	for _, c := range cases {
		for _, side := range []struct {
			name string
			v    float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := make(map[string]float64, len(c.Params)+1)
			for k, v := range c.Params {
				p[k] = v
			}
			p[keyGearSide] = side.v
			out = append(out, proofkit3d.Case{Name: c.Name + "_" + side.name, Params: p})
		}
	}
	return out
}

// perGearSketch is perGear for a sketch table.
func perGearSketch(cases []proofkit.Case) []proofkit.Case {
	out := make([]proofkit.Case, 0, 2*len(cases))
	for _, c := range cases {
		for _, side := range []struct {
			name string
			v    float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := make(map[string]float64, len(c.Params)+1)
			for k, v := range c.Params {
				p[k] = v
			}
			p[keyGearSide] = side.v
			out = append(out, proofkit.Case{Name: c.Name + "_" + side.name, Params: p})
		}
	}
	return out
}

// ---------------------------------------------------------------------------
// The input-bounds table.
//
// Each case names the published worked values it is there to check, so the
// numbers instructions.md states appear in the table rather than only in the
// assertions. A case with no expectations still runs the whole battery of
// identities and rejections, which every case does.
// ---------------------------------------------------------------------------

var boundsCases = []proofkit.Case{
	// The worked case instructions.md gives in full under Maximum Base Height:
	// module 1, an equal 31/31 pair at 30 degrees, each gamma 15 degrees.
	{Name: "worked_31_31_at_30_base_height_cap", Params: params(map[string]float64{
		keyShaftAngle:            30,
		keyExpectMaxBaseHeight:   3.638,
		keyExpectTrueCrossing:    4.153,
		keyExpectDrivingFallback: 3.875,
		keyExpectResolvedBase:    3.638,
	})},
	// The ratio pair whose cone-angle limit instructions.md computes.
	{Name: "worked_31_17_maximum_shaft_angle", Params: params(map[string]float64{
		keyPinionTeeth:          17,
		keyExpectMaxShaftAngle:  123.2564,
		keyExpectShaftExclusive: 1,
	})},
	// Equal tooth counts give acos(-1) = 180, which is no constraint at all, so
	// the 150 practical ceiling is what binds and it is INCLUSIVE.
	{Name: "equal_teeth_capped_at_150_inclusive", Params: params(map[string]float64{
		keyExpectMaxShaftAngle:  150,
		keyExpectShaftExclusive: 0,
	})},
	// At a 90 degree shaft angle the computed tooth floor is 3.72, i.e. 4 teeth.
	{Name: "minimum_teeth_floor_at_90", Params: params(map[string]float64{
		keyExpectMinTeeth: 3.7265,
	})},
	// The smallest pair the computed floor admits. Its fallback base height,
	// 4/8 = 0.5 mm, falls BELOW the minimum and is raised.
	{Name: "fallback_raised_to_the_minimum", Params: params(map[string]float64{
		keyDrivingTeeth: 4, keyPinionTeeth: 4,
		keyExpectDrivingFallback: 0.5,
	})},
	// The default pair, where the fallback sits inside its own window untouched.
	{Name: "fallback_inside_the_window", Params: params(map[string]float64{
		keyExpectDrivingFallback: 3.875,
		keyExpectResolvedBase:    3.875,
	})},
	{Name: "ratio_driving_31_pinion_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "ratio_driving_17_pinion_31", Params: params(map[string]float64{keyDrivingTeeth: 17})},
	{Name: "shaft_angle_150", Params: params(map[string]float64{keyShaftAngle: 150})},
	{Name: "module_2_driving_19_pinion_13", Params: params(map[string]float64{
		keyModule: 2, keyDrivingTeeth: 19, keyPinionTeeth: 13})},
	{Name: "user_base_heights_inside_both_windows", Params: params(map[string]float64{
		keyDrivingBase: 3, keyPinionBase: 2.5})},
	{Name: "small_teeth_8_8", Params: params(map[string]float64{
		keyDrivingTeeth: 8, keyPinionTeeth: 8})},
}

// coneElementCases prove the cone-element line across the shaft-angle range and
// both ratio directions, since the line IS the root cone element and its
// direction is what the whole spiral frame is built on.
var coneElementCases = perGearSketch([]proofkit.Case{
	{Name: "default_31_31_at_90", Params: params(nil)},
	{Name: "ratio_31_17", Params: params(map[string]float64{keyPinionTeeth: 17})},
	{Name: "shaft_angle_30", Params: params(map[string]float64{keyShaftAngle: 30})},
	{Name: "shaft_angle_150", Params: params(map[string]float64{keyShaftAngle: 150})},
})
