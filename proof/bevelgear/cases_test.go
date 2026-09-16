package bevelgear_test

// The parameter tables every step is proved against.
//
// A branch a step takes needs a case on each side of it, and a branch the spec
// says is reachable in more than one way needs a case for each way. So the
// tables below carry both ends of every range the spec states -- the Shaft
// Angle floor and its ceiling, Toe Extension 0 and 100, a straight bevel and a
// spiral one at each hand -- and both members of every pair, because one
// resolved case holds two gears and the per-gear values differ.
//
// Two tooth counts are here because the virtual tooth count needs them. A
// 16 driving / 12 pinion pair at Shaft Angle 90 gives the pinion a virtual count
// of exactly 15 and the driving gear 26.667, so one member is a case where an
// exact count and a rounded one agree and the other is a case where they do
// not: whatever error rounding introduces, it is not the same error on both
// members. A 4/4 pair has the lowest virtual count the table carries, 5.657,
// and the largest root-arc corner float of any pair the spec admits, 0.027
// module -- the case that fixes whether the root sink is large enough.
//
// The solid tables run at Module 4 to 8 and NEVER at Module 1. decad's mesh
// bound has an absolute floor, so a figure small enough brings every
// measurement inside it and the gate reports Suspect on geometry that is in
// fact correct. Module is a pure scale on this figure, so a case at Module 4
// through 8 proves the same shape and clears the floor. The sketch tables are
// unaffected and stay at the dialog's own default.

import (
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// keyDeclaredRefusal marks a case this gear's section 2 lattice cannot reach
// although the spec admits it. It is not a dialog input.
//
// Where the lattice's conditioning reads below the sketch engine's trust floor,
// the case STAYS in the table carrying this flag, rather than the advertised
// Shaft Angle range being narrowed to whatever this one net happens to reach.
// The spec is explicit that the conditioning limit is a property of how a
// lattice is built and not of the geometry: three independently written nets,
// each holding DOF 0 with nothing redundant, do not agree on which end of the
// range is reachable. So a refusal here is a fact about this net, and the table
// is where that fact is recorded.
const keyDeclaredRefusal = "declaredRefusal"

// keyRefusalConditioning is the conditioning this net measured on a declared
// refusal, so the number that made the call is recorded beside the flag rather
// than only in whoever ran it once.
const keyRefusalConditioning = "refusalConditioning"

// keySpiralBranch selects which side of the psi gate a spiral step is proved
// on. It is not a dialog input; the dialog reaches the same branch through Mean
// Spiral Angle alone.
const keySpiralBranch = "spiralBranch"

// dialogDefaults is the shipped dialog default: Module 1, an equal 31/31 pair at
// a 90 degree Shaft Angle, every optional length left at 0 so it resolves, the
// bore enabled, and a 35 degree right-hand spiral.
//
// The values are the defaults column of the spec's input table, reproduced here
// rather than pointed at, because a step that reads this map is the only place
// they reach the proof.
func dialogDefaults() map[string]float64 {
	return map[string]float64{
		keyModule:           1,
		keyDrivingTeeth:     31,
		keyPinionTeeth:      31,
		keyShaftAngle:       90,
		keyDrivingBase:      0,
		keyPinionBase:       0,
		keyBoreEnable:       1,
		keyDrivingBore:      0,
		keyPinionBore:       0,
		keyFaceWidth:        0,
		keyToothSpacing:     0,
		keySpiralAngle:      35,
		keyHand:             1,
		keyCutterRadius:     0,
		keyToeExtension:     0,
		keyDrivingToeRadius: 0,
		keyPinionToeRadius:  0,
		keyGearSide:         0,
	}
}

// caseParams is the defaults with the named overrides applied.
func caseParams(overrides map[string]float64) map[string]float64 {
	p := dialogDefaults()
	for k, v := range overrides {
		p[k] = v
	}
	return p
}

// config is one dialog configuration, before it is split into the two gears.
type config struct {
	Name      string
	Overrides map[string]float64
}

// sharedConfigs are the configurations the whole gear is proved over. They are
// the regime the design has to hold across: both ends of the Shaft Angle range,
// both orders of an unequal ratio, the two virtual-tooth-count pairs, and each
// optional input once specified and once left to resolve.
var sharedConfigs = []config{
	{"default-31-31", nil},

	// The Shaft Angle range, both ends. 30 degrees is the documented floor and
	// is NOT known to be reachable: of three independently written lattices,
	// two refuse the default pair there on conditioning and first clear at 35.
	// The floor stays in the table either way.
	// Measured on this net: conditioning 2.832e-05 against the engine's 4e-05
	// trust floor. The spec records 2.83e-5 and 2.94e-5 for two of the three
	// lattices written from it, so this one behaves like those two -- it
	// refuses 30 degrees, first clears at 35, and passes 142 and 150. That is a
	// fact about this construction, not about the geometry, and the remedy is
	// to change how the lattice is built rather than to loosen the gate or
	// narrow the advertised range.
	{"shaft-30", map[string]float64{
		keyShaftAngle: 30, keyDeclaredRefusal: 1, keyRefusalConditioning: 2.832e-05}},
	{"shaft-35", map[string]float64{keyShaftAngle: 35}},
	{"shaft-60", map[string]float64{keyShaftAngle: 60}},
	{"shaft-120", map[string]float64{keyShaftAngle: 120}},
	{"shaft-142", map[string]float64{keyShaftAngle: 142}},
	// Equal tooth counts give acos(-1) = 180, so the 150 degree practical cap
	// is what binds, and it is INCLUSIVE.
	{"shaft-150", map[string]float64{keyShaftAngle: 150}},

	// Both orders of the same ratio. The Maximum Face Width is written from the
	// SMALLER pitch diameter and never from the pinion's by name: on driving 17
	// / pinion 31 the real bound is 3.883 mm where the pinion form gives 13.591,
	// so the naive Cone Distance / 6 default exceeds it and the gear fails to
	// generate for any ratio above roughly sqrt(2).
	{"ratio-31-17", map[string]float64{keyDrivingTeeth: 31, keyPinionTeeth: 17}},
	{"ratio-17-31", map[string]float64{keyDrivingTeeth: 17, keyPinionTeeth: 31}},
	// The cone-angle ceiling for this pair is acos(-17/31) = 123.26 degrees, so
	// 120 is inside it and the flat 150 this spec once promised never was.
	// Measured on this net: conditioning 7.149e-07, two orders below the floor.
	// The configuration is well inside every bound the spec states -- the
	// cone-angle ceiling for this pair is 123.26 degrees -- so this is the same
	// kind of refusal as the 30 degree one and is recorded the same way. It is
	// a refusal the spec does not mention, and it is this net's to report.
	{"ratio-31-17-shaft-120", map[string]float64{
		keyDrivingTeeth: 31, keyPinionTeeth: 17, keyShaftAngle: 120,
		keyDeclaredRefusal: 1, keyRefusalConditioning: 7.149e-07}},

	// The two the virtual tooth count needs.
	{"teeth-16-12", map[string]float64{keyDrivingTeeth: 16, keyPinionTeeth: 12}},
	{"teeth-4-4", map[string]float64{keyDrivingTeeth: 4, keyPinionTeeth: 4}},

	// Each optional input, specified rather than resolved.
	{"base-height-given", map[string]float64{keyDrivingBase: 3, keyPinionBase: 3}},
	{"face-width-given", map[string]float64{keyFaceWidth: 5}},
	{"tooth-spacing", map[string]float64{keyToothSpacing: 0.3}},
	{"toe-extension-50", map[string]float64{keyToeExtension: 50}},
	{"toe-extension-100", map[string]float64{keyToeExtension: 100}},
	{"toe-radius-given", map[string]float64{
		keyDrivingToeRadius: 9, keyPinionToeRadius: 9}},
	{"bore-off", map[string]float64{keyBoreEnable: 0}},
	{"bore-given", map[string]float64{keyDrivingBore: 6, keyPinionBore: 6}},
}

// bothSides expands one configuration into its two gears. Nothing else
// separates them: keyGearSide is how a per-gear step says which member of the
// pair it is building.
func bothSides(c config) []map[string]float64 {
	pinion := caseParams(c.Overrides)
	pinion[keyGearSide] = 0
	driving := caseParams(c.Overrides)
	driving[keyGearSide] = 1
	return []map[string]float64{pinion, driving}
}

// sketchCases is one case per configuration, for the steps that build the
// shared figure both gears come out of.
var sketchCases = func() []proofkit.Case {
	cases := make([]proofkit.Case, 0, len(sharedConfigs))
	for _, c := range sharedConfigs {
		cases = append(cases, proofkit.Case{Name: c.Name, Params: caseParams(c.Overrides)})
	}
	return cases
}()

// perGearSketchCases is two cases per configuration, for the steps that build
// one gear's own sketch.
var perGearSketchCases = func() []proofkit.Case {
	cases := make([]proofkit.Case, 0, 2*len(sharedConfigs))
	for _, c := range sharedConfigs {
		sides := bothSides(c)
		cases = append(cases,
			proofkit.Case{Name: c.Name + "/pinion", Params: sides[0]},
			proofkit.Case{Name: c.Name + "/driving", Params: sides[1]})
	}
	return cases
}()

// solidConfigs are the configurations the bodies are proved over, at Module 4
// to 8. They are the sketch configurations that say something different about a
// solid, scaled off Module 1 for the reason at the top of this file.
var solidConfigs = []config{
	{"default-31-31-m4", map[string]float64{keyModule: 4}},
	{"default-31-31-m8", map[string]float64{keyModule: 8}},
	{"shaft-60-m4", map[string]float64{keyModule: 4, keyShaftAngle: 60}},
	{"shaft-120-m4", map[string]float64{keyModule: 4, keyShaftAngle: 120}},
	{"ratio-31-17-m4", map[string]float64{
		keyModule: 4, keyDrivingTeeth: 31, keyPinionTeeth: 17}},
	{"ratio-17-31-m4", map[string]float64{
		keyModule: 4, keyDrivingTeeth: 17, keyPinionTeeth: 31}},
	// The two the virtual tooth count needs, at the solid table's module.
	{"teeth-16-12-m4", map[string]float64{
		keyModule: 4, keyDrivingTeeth: 16, keyPinionTeeth: 12}},
	{"teeth-4-4-m4", map[string]float64{
		keyModule: 4, keyDrivingTeeth: 4, keyPinionTeeth: 4}},
	{"toe-extension-100-m4", map[string]float64{keyModule: 4, keyToeExtension: 100}},
	{"tooth-spacing-m4", map[string]float64{keyModule: 4, keyToothSpacing: 1.2}},
}

// solidCases is two cases per solid configuration, one per gear.
var solidCases = func() []proofkit3d.Case {
	cases := make([]proofkit3d.Case, 0, 2*len(solidConfigs))
	for _, c := range solidConfigs {
		sides := bothSides(c)
		cases = append(cases,
			proofkit3d.Case{Name: c.Name + "/pinion", Params: sides[0]},
			proofkit3d.Case{Name: c.Name + "/driving", Params: sides[1]})
	}
	return cases
}()

// boreConfigs carry the bore branch on both sides: cut and not cut, resolved
// and given.
var boreConfigs = []config{
	{"bore-auto-m4", map[string]float64{keyModule: 4}},
	{"bore-given-m4", map[string]float64{keyModule: 4, keyDrivingBore: 20, keyPinionBore: 20}},
	{"bore-off-m4", map[string]float64{keyModule: 4, keyBoreEnable: 0}},
	{"bore-auto-ratio-m4", map[string]float64{
		keyModule: 4, keyDrivingTeeth: 31, keyPinionTeeth: 17}},
}

var boreCases = func() []proofkit3d.Case {
	cases := make([]proofkit3d.Case, 0, 2*len(boreConfigs))
	for _, c := range boreConfigs {
		sides := bothSides(c)
		cases = append(cases,
			proofkit3d.Case{Name: c.Name + "/pinion", Params: sides[0]},
			proofkit3d.Case{Name: c.Name + "/driving", Params: sides[1]})
	}
	return cases
}()

// spiralConfigs carry both sides of the psi gate and both hands. psi = 0 is a
// STRAIGHT bevel and every spiral input is ignored; any value above 0 builds a
// curved tooth. The hand is read on the driving gear and the pinion is built
// with the opposite hand, so an equal-teeth pair's two traces must come out as
// exact mirror images.
var spiralConfigs = []config{
	{"psi-35-right-m4", map[string]float64{keyModule: 4, keySpiralAngle: 35, keyHand: 1}},
	{"psi-35-left-m4", map[string]float64{keyModule: 4, keySpiralAngle: 35, keyHand: 0}},
	// The top of the [0, 60) range, exclusive at 60.
	{"psi-59-right-m4", map[string]float64{keyModule: 4, keySpiralAngle: 59, keyHand: 1}},
	// A cutter radius given rather than defaulted to R_mean.
	{"psi-35-cutter-given-m4", map[string]float64{
		keyModule: 4, keySpiralAngle: 35, keyCutterRadius: 40}},
	// The ratio pair whose two members legitimately get DIFFERENT twists: same
	// cutter and same psi, but gamma differs, so 1/sin(gamma) does.
	{"psi-35-ratio-19-13-m4", map[string]float64{
		keyModule: 4, keySpiralAngle: 35, keyDrivingTeeth: 19, keyPinionTeeth: 13}},
}

var spiralCases = func() []proofkit3d.Case {
	cases := make([]proofkit3d.Case, 0, 2*len(spiralConfigs))
	for _, c := range spiralConfigs {
		sides := bothSides(c)
		cases = append(cases,
			proofkit3d.Case{Name: c.Name + "/pinion", Params: sides[0]},
			proofkit3d.Case{Name: c.Name + "/driving", Params: sides[1]})
	}
	return cases
}()

// spiralSketchCases are the spiral's own two sketches, which are 2-D.
var spiralSketchCases = func() []proofkit.Case {
	cases := make([]proofkit.Case, 0, 2*len(spiralConfigs)+2)
	for _, c := range spiralConfigs {
		sides := bothSides(c)
		cases = append(cases,
			proofkit.Case{Name: c.Name + "/pinion", Params: sides[0]},
			proofkit.Case{Name: c.Name + "/driving", Params: sides[1]})
	}
	// psi = 0 takes the straight branch and authors none of these sketches.
	straight := caseParams(map[string]float64{keyModule: 4, keySpiralAngle: 0})
	straight[keySpiralBranch] = 0
	cases = append(cases, proofkit.Case{Name: "psi-0-straight-m4", Params: straight})
	return cases
}()

// patternCases are the Pattern step's own, and this step is the one that stays
// SERIAL. See stepCircularPattern for why.
var patternCases = func() []proofkit3d.Case {
	keep := map[string]bool{
		"default-31-31-m4/pinion":  true,
		"default-31-31-m4/driving": true,
		"teeth-4-4-m4/pinion":      true,
		"teeth-4-4-m4/driving":     true,
		"teeth-16-12-m4/pinion":    true,
		"teeth-16-12-m4/driving":   true,
		"ratio-31-17-m4/pinion":    true,
		"ratio-31-17-m4/driving":   true,
	}
	cases := make([]proofkit3d.Case, 0, len(keep))
	for _, c := range solidCases {
		if keep[c.Name] {
			cases = append(cases, c)
		}
	}
	return cases
}()
