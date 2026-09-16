// This file holds the case tables every step is proved against, and nothing
// else. A table is the regime the spec says the design has to hold across, so
// it is written here once and read by the steps rather than restated per step.
//
// Two rules from the spec shape every table below.
//
// The SOLID tables run at Module 4 to 8 and never at Module 1. decad's mesh
// bound has an absolute floor, so a figure small enough brings every measurement
// inside it and the gate reports Suspect on geometry that is in fact correct.
// Module is a pure scale on this figure, so a case at Module 4 through 8 proves
// the same shape as one at Module 1 and clears the floor. The sketch tables are
// unaffected and stay at the dialog's own default of Module 1.
//
// Two pairs appear in BOTH the sketch and the solid tables because the virtual
// tooth count is read by the tooth profile and by every body built on it:
//
//   - 16 driving / 12 pinion at Shaft Angle 90. The pinion's virtual tooth count
//     is exactly 15 and the driving gear's is 26.667, so one member of the pair
//     is a case where an exact count and a rounded one agree and the other is a
//     case where they do not. Whatever error rounding introduces, it is not the
//     same error on both members.
//   - 4 / 4 at Shaft Angle 90. Its virtual count is 5.657, the lowest the table
//     carries, and its root arc carries the largest corner float of any pair the
//     spec admits, 0.027 module. That is the case the root sink has to clear.
package bevelgear_test

import (
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

func radiansOf(deg float64) float64 { return deg * math.Pi / 180 }

// sketchParams names one case by the dialog values it comes from, with every
// input at its documented default except the four named here.
func sketchParams(module, drivingTeeth, pinionTeeth, shaftAngleDeg float64) map[string]float64 {
	return map[string]float64{
		"module":            module,
		"drivingTeeth":      drivingTeeth,
		"pinionTeeth":       pinionTeeth,
		"shaftAngle":        radiansOf(shaftAngleDeg),
		"drivingBaseHeight": 0,
		"pinionBaseHeight":  0,
		"boreEnable":        1,
		"drivingBore":       0,
		"pinionBore":        0,
		"faceWidth":         0,
		"toothSpacing":      0,
		"spiralAngle":       0,
		"leftHanded":        0,
		"cutterRadius":      0,
		"toeExtension":      0,
		"drivingToeRadius":  0,
		"pinionToeRadius":   0,
		"centerX":           0,
		"centerY":           0,
		"anchorAngle":       0,
		"growSign":          1,
		"gear":              0,
		"declaredRefusal":   0,
	}
}

func withParam(p map[string]float64, key string, value float64) map[string]float64 {
	p[key] = value
	return p
}

// withCentre moves the user's centre point off the sketch origin. The dialog
// does not require it to sit there, and §2 builds the whole figure relative to
// wherever it lands.
func withCentre(p map[string]float64, x, y float64) map[string]float64 {
	p["centerX"], p["centerY"] = x, y
	return p
}

// withTilt turns the projected anchor line. Its absolute direction is arbitrary
// — §2 derives every direction relative to it — so a case that turns it proves
// the figure is built in the sketch's own frame and not against a world axis.
func withTilt(p map[string]float64, deg float64) map[string]float64 {
	p["anchorAngle"] = radiansOf(deg)
	return p
}

// forDriving switches a per-gear step from the pinion to the driving gear. Every
// per-gear step runs on both, pinion first, and the two differ in pitch cone
// angle whenever the tooth counts do.
func forDriving(p map[string]float64) map[string]float64 { return withParam(p, "gear", 1) }

// ---------------------------------------------------------------- sketch tables

// gearProfileCases sweep the regime the spec declares the §2 lattice has to
// hold across.
//
//   - the whole Shaft Angle range, from the documented 30-degree floor to the
//     150-degree ceiling, including the 35 degrees two of three independently
//     written lattices first clear at;
//   - both directions of asymmetry, since EITHER gear can be the smaller and the
//     Maximum Face Width binds on whichever it is;
//   - the two virtual-tooth-count pairs and the 4/4 pair at the Minimum Teeth
//     floor;
//   - the anchor line tilted and the centre off the sketch origin, and the grow
//     side flipped, which is the one bit the target-plane normal decides;
//   - Tooth Spacing above 0, which builds geometry that exists at no other
//     setting, and Toe Extension at both ends of its [0, 100] range.
var gearProfileCases = []proofkit.Case{
	{Name: "default_31_31_S90", Params: sketchParams(1, 31, 31, 90)},
	{Name: "S35_31_31", Params: sketchParams(1, 31, 31, 35)},
	{Name: "S60_driving31_pinion17", Params: sketchParams(1, 31, 17, 60)},
	{Name: "S100_driving17_pinion31", Params: sketchParams(1, 17, 31, 100)},
	{Name: "S110_driving31_pinion17", Params: sketchParams(1, 31, 17, 110)},
	{Name: "S90_driving17_pinion31", Params: sketchParams(1, 17, 31, 90)},
	{Name: "S142_31_31", Params: sketchParams(1, 31, 31, 142)},
	{Name: "virtual_counts_16_12_S90", Params: sketchParams(1, 16, 12, 90)},
	{Name: "lowest_virtual_count_4_4_S90", Params: sketchParams(1, 4, 4, 90)},
	{Name: "tilted_anchor_line", Params: withTilt(sketchParams(1, 31, 31, 90), 37)},
	{Name: "centre_off_origin", Params: withCentre(sketchParams(1, 31, 17, 75), 14, -9)},
	{Name: "grow_side_flipped", Params: withParam(sketchParams(1, 31, 31, 90), "growSign", -1)},
	{Name: "tooth_spacing_0p3", Params: withParam(sketchParams(1, 31, 31, 90), "toothSpacing", 0.3)},
	{Name: "toe_extension_50", Params: withParam(sketchParams(1, 31, 31, 90), "toeExtension", 50)},
	{Name: "toe_extension_100", Params: withParam(sketchParams(1, 31, 31, 90), "toeExtension", 100)},
	{Name: "user_base_heights", Params: withParam(withParam(sketchParams(1, 31, 31, 90),
		"drivingBaseHeight", 6), "pinionBaseHeight", 2)},
	{Name: "user_face_width_and_toe_radii", Params: withParam(withParam(withParam(sketchParams(1, 31, 31, 90),
		"faceWidth", 5), "pinionToeRadius", 8), "drivingToeRadius", 8)},
	{Name: "module_4_31_31_S90", Params: sketchParams(4, 31, 31, 90)},
	// Three configurations the spec admits and THIS lattice cannot reach. Each
	// stays in the table and is marked, rather than the advertised Shaft Angle
	// range being narrowed on one net's evidence: a build that fails on
	// conditioning has found a fact about its own constraint net, and the remedy
	// is to change how the lattice is built, never to loosen the gate.
	//
	// Measured on this net: 30 degrees reads 2.831e-05 against the engine's
	// 4e-05 floor, which is the same reading the spec records for one of the two
	// nets that refuse that end; 150 degrees reads 3.973e-05, a hair below, where
	// the sibling net the spec measured read 4.07e-05 and cleared; and the
	// 17/31 pair at 120 degrees reads 7.63e-07, deep below, with its pinion pitch
	// cone angle climbing toward the 90 degrees the Maximum Shaft Angle exists to
	// keep it under.
	{Name: "declared_refusal_S30_31_31", Params: withParam(sketchParams(1, 31, 31, 30), "declaredRefusal", 1)},
	{Name: "declared_refusal_S150_31_31", Params: withParam(sketchParams(1, 31, 31, 150), "declaredRefusal", 1)},
	{Name: "declared_refusal_S120_driving17_pinion31", Params: withParam(sketchParams(1, 17, 31, 120), "declaredRefusal", 1)},
}

// toothCases run the virtual spur tooth on both members of every pair whose
// virtual count matters, since the count is what decides the drawn tooth's size
// and thickness.
var toothCases = []proofkit.Case{
	{Name: "pinion_31_31_S90", Params: sketchParams(1, 31, 31, 90)},
	{Name: "driving_31_31_S90", Params: forDriving(sketchParams(1, 31, 31, 90))},
	{Name: "pinion_exact_15_of_16_12", Params: sketchParams(1, 16, 12, 90)},
	{Name: "driving_26p667_of_16_12", Params: forDriving(sketchParams(1, 16, 12, 90))},
	{Name: "pinion_lowest_count_4_4", Params: sketchParams(1, 4, 4, 90)},
	{Name: "driving_lowest_count_4_4", Params: forDriving(sketchParams(1, 4, 4, 90))},
	{Name: "pinion_S60_driving31_pinion17", Params: sketchParams(1, 31, 17, 60)},
	{Name: "driving_S60_driving31_pinion17", Params: forDriving(sketchParams(1, 31, 17, 60))},
	{Name: "pinion_module_4", Params: sketchParams(4, 31, 31, 90)},
}

// hexagonCases prove the frustum profile on both gears, across the pairs whose
// toe end differs most, and with the Toe Extension at both ends of its range.
var hexagonCases = []proofkit.Case{
	{Name: "pinion_31_31_S90", Params: sketchParams(1, 31, 31, 90)},
	{Name: "driving_31_31_S90", Params: forDriving(sketchParams(1, 31, 31, 90))},
	{Name: "pinion_16_12_S90", Params: sketchParams(1, 16, 12, 90)},
	{Name: "driving_16_12_S90", Params: forDriving(sketchParams(1, 16, 12, 90))},
	{Name: "pinion_4_4_S90", Params: sketchParams(1, 4, 4, 90)},
	{Name: "driving_4_4_S90", Params: forDriving(sketchParams(1, 4, 4, 90))},
	{Name: "pinion_S60_31_17", Params: sketchParams(1, 31, 17, 60)},
	{Name: "driving_S60_31_17", Params: forDriving(sketchParams(1, 31, 17, 60))},
	{Name: "pinion_toe_extension_100", Params: withParam(sketchParams(1, 31, 31, 90), "toeExtension", 100)},
	{Name: "driving_toe_extension_100", Params: forDriving(withParam(sketchParams(1, 31, 31, 90), "toeExtension", 100))},
}

// boreCases carry both sides of the Enable Bore branch and both the auto and the
// user bore diameter, on both gears.
var boreCases = []proofkit.Case{
	{Name: "pinion_auto_diameter", Params: sketchParams(1, 31, 31, 90)},
	{Name: "driving_auto_diameter", Params: forDriving(sketchParams(1, 31, 31, 90))},
	{Name: "pinion_user_diameter", Params: withParam(sketchParams(1, 31, 31, 90), "pinionBore", 6)},
	{Name: "driving_user_diameter", Params: forDriving(withParam(sketchParams(1, 31, 31, 90), "drivingBore", 9))},
	{Name: "bore_disabled", Params: withParam(sketchParams(1, 31, 31, 90), "boreEnable", 0)},
}

// ---------------------------------------------------------------- solid tables

func solidCase(name string, p map[string]float64) proofkit3d.Case {
	return proofkit3d.Case{Name: name, Params: p}
}

// solidCases run the body steps at Module 4 through 8, on both gears, across the
// pairs whose geometry differs most.
var solidCases = []proofkit3d.Case{
	solidCase("pinion_M4_31_31_S90", sketchParams(4, 31, 31, 90)),
	solidCase("driving_M4_31_31_S90", forDriving(sketchParams(4, 31, 31, 90))),
	solidCase("pinion_M6_16_12_S90", sketchParams(6, 16, 12, 90)),
	solidCase("driving_M6_16_12_S90", forDriving(sketchParams(6, 16, 12, 90))),
	solidCase("pinion_M8_4_4_S90", sketchParams(8, 4, 4, 90)),
	solidCase("driving_M8_4_4_S90", forDriving(sketchParams(8, 4, 4, 90))),
	solidCase("pinion_M4_31_17_S60", sketchParams(4, 31, 17, 60)),
	solidCase("driving_M4_31_17_S60", forDriving(sketchParams(4, 31, 17, 60))),
	solidCase("pinion_M4_toe_extension_100", withParam(sketchParams(4, 31, 31, 90), "toeExtension", 100)),
}

// boreSolidCases add the Enable Bore branch's other side to the solid table, and
// a user bore diameter beside the auto one.
var boreSolidCases = []proofkit3d.Case{
	solidCase("pinion_M4_auto_bore", sketchParams(4, 31, 31, 90)),
	solidCase("driving_M4_auto_bore", forDriving(sketchParams(4, 31, 31, 90))),
	solidCase("pinion_M4_user_bore", withParam(sketchParams(4, 31, 31, 90), "pinionBore", 20)),
	solidCase("pinion_M4_bore_disabled", withParam(sketchParams(4, 31, 31, 90), "boreEnable", 0)),
	solidCase("pinion_M6_16_12_auto_bore", sketchParams(6, 16, 12, 90)),
}

// spiralCases carry the psi > 0 branch: both hands, the auto cutter radius and a
// user one, an equal pair and a ratio pair — the ratio pair being the one that
// separates a correct 1/sin gamma roll ratio from a wrong one, since equal teeth
// mesh under any method that gets it wrong.
var spiralCases = []proofkit3d.Case{
	solidCase("pinion_M4_31_31_psi35_right", withSpiralInputs(sketchParams(4, 31, 31, 90), 35, 0, false)),
	solidCase("driving_M4_31_31_psi35_right", withSpiralInputs(forDriving(sketchParams(4, 31, 31, 90)), 35, 0, false)),
	solidCase("pinion_M4_31_31_psi35_left", withSpiralInputs(sketchParams(4, 31, 31, 90), 35, 0, true)),
	solidCase("pinion_M4_31_17_psi35_right", withSpiralInputs(sketchParams(4, 31, 17, 90), 35, 0, false)),
	solidCase("driving_M4_31_17_psi35_right", withSpiralInputs(forDriving(sketchParams(4, 31, 17, 90)), 35, 0, false)),
	solidCase("pinion_M4_31_31_psi15_cutter60", withSpiralInputs(sketchParams(4, 31, 31, 90), 15, 60, false)),
	solidCase("pinion_M4_31_31_psi55_right", withSpiralInputs(sketchParams(4, 31, 31, 90), 55, 0, false)),
	solidCase("pinion_M6_16_12_psi35_right", withSpiralInputs(sketchParams(6, 16, 12, 90), 35, 0, false)),
}

// spiralSketchCases are the same configurations for the flat trace sketch, which
// is drawn once per gear and needs no solid.
var spiralSketchCases = []proofkit.Case{
	{Name: "pinion_psi35_right_auto_cutter", Params: withSpiralInputs(sketchParams(4, 31, 31, 90), 35, 0, false)},
	{Name: "driving_psi35_right_auto_cutter", Params: withSpiralInputs(forDriving(sketchParams(4, 31, 31, 90)), 35, 0, false)},
	{Name: "pinion_psi35_left_auto_cutter", Params: withSpiralInputs(sketchParams(4, 31, 31, 90), 35, 0, true)},
	{Name: "driving_psi35_left_auto_cutter", Params: withSpiralInputs(forDriving(sketchParams(4, 31, 31, 90)), 35, 0, true)},
	{Name: "pinion_psi0_straight", Params: withSpiralInputs(sketchParams(4, 31, 31, 90), 0, 0, false)},
	{Name: "pinion_psi15_cutter60", Params: withSpiralInputs(sketchParams(4, 31, 31, 90), 15, 60, false)},
	{Name: "pinion_psi55_right", Params: withSpiralInputs(sketchParams(4, 31, 31, 90), 55, 0, false)},
	{Name: "ratio_pinion_psi35_right", Params: withSpiralInputs(sketchParams(4, 31, 17, 90), 35, 0, false)},
	{Name: "ratio_driving_psi35_right", Params: withSpiralInputs(forDriving(sketchParams(4, 31, 17, 90)), 35, 0, false)},
}

// spiral sets the three spiral inputs on a case. Mean Spiral Angle 0 means a
// STRAIGHT bevel and the other two are then ignored; Cutter Radius 0 means auto,
// which is the mean cone distance.
func withSpiralInputs(p map[string]float64, psiDeg, cutter float64, left bool) map[string]float64 {
	p["spiralAngle"] = radiansOf(psiDeg)
	p["cutterRadius"] = cutter
	if left {
		p["leftHanded"] = 1
	}
	return p
}
