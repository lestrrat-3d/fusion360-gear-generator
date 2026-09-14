// This file holds the input-resolution step: everything the spec's Variables
// section computes from the dialog before any geometry exists, and the two
// different things it does with a value that breaks a bound.
//
// # Why this is its own step
//
// The Variables section is where a wrong number passes silently. It resolves
// the Maximum Shaft Angle, the Minimum Teeth floor, each gear's base-height
// window, the Maximum Face Width, and the whole toe-end window — the Toe Radius
// default and its ceiling, the Toe Limit, and where Toe Extension 100 stops. It
// is all closed form over the twenty dialog inputs, so it needs no sketch to
// compute, and none of it shows up as a shape until several steps later, by
// which time a wrong bound reads as a geometry failure somewhere else.
//
// # Clamped or rejected, which is not the same thing
//
// The section assigns two different behaviours to two different cases, and the
// split is by where the value came from rather than by which bound it broke.
//
//   - A value the dialog FELL BACK to — a base height from Module * teeth / 8,
//     a Face Width from Cone Distance / 6 — is CLAMPED into its window: raised
//     to a minimum, capped to a maximum, and the build goes on.
//   - A value the USER TYPED is REJECTED, with a message naming the bound it
//     broke and that bound's number, and the build stops.
//
// A resolver that clamped a user value would silently build a gear the user did
// not ask for; one that rejected a fallback would refuse a dialog the user
// never touched. So the proof sweeps each bound from both sides and checks
// which of the two happened, which bound was named, and what number it carried.
//
// # What the proof renders and what it does not
//
// The message TEXT is this proof's own wording, not the module's: a step list
// cannot fix a human sentence and a gate should not try. What is proved is the
// part that has to agree — that a rejection happened at all, which input it
// names, which bound it names, and the value of that bound — and that the
// rendered message carries that number rather than only the offending value.
package bevelgear_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// ------------------------------------------------------------- the resolution

// problem is one rejection, naming the input, the bound it broke and that
// bound's value.
type problem struct {
	input string
	bound string
	value float64
	limit float64
}

func (pr problem) text() string {
	return fmt.Sprintf("%s is %.6f, which breaks %s at %.6f",
		pr.input, pr.value, pr.bound, pr.limit)
}

// adjustment is one fallback moved onto a bound, which is the other half of the
// split: no rejection, and the build goes on at the bound's own value.
type adjustment struct {
	input string
	bound string
	from  float64
	to    float64
}

// resolution is what reading the dialog produces: the resolved figure when the
// dialog is admissible, every rejection when it is not, and every fallback that
// had to be moved onto a bound.
type resolution struct {
	fig      figure
	ok       bool
	problems []problem
	clamps   []adjustment
}

func (r *resolution) reject(input, bound string, value, limit float64) {
	r.problems = append(r.problems, problem{input, bound, value, limit})
}

func (r *resolution) clamp(input, bound string, from, to float64) {
	r.clamps = append(r.clamps, adjustment{input, bound, from, to})
}

func (r resolution) rejected(input string) (problem, bool) {
	for _, pr := range r.problems {
		if pr.input == input {
			return pr, true
		}
	}
	return problem{}, false
}

func (r resolution) clamped(input string) (adjustment, bool) {
	for _, a := range r.clamps {
		if a.input == input {
			return a, true
		}
	}
	return adjustment{}, false
}

// resolveDialog runs the Variables section over one dialog, in the order the
// spec resolves it. Each stage returns early when it has already rejected,
// because the stage after it reads what the stage before resolved: the Minimum
// Teeth check is exactly the statement that the base-height window is non-empty,
// so running it first means the base-height stage never has to describe what to
// do when the minimum exceeds the maximum.
func resolveDialog(p map[string]float64) resolution {
	r := resolution{}
	m, zp, zg := p["module"], p["pinionTeeth"], p["drivingTeeth"]
	sigmaDeg := p["shaftAngleDeg"]

	// 1. The range checks that need nothing derived.
	if m <= 0 {
		r.reject("Module", "the positive-Module floor", m, 0)
	}
	for _, g := range []struct {
		label string
		teeth float64
	}{{"Pinion Gear Teeth", zp}, {"Driving Gear Teeth", zg}} {
		if g.teeth < 3 {
			r.reject(g.label, "the absolute tooth floor", g.teeth, 3)
		}
	}
	for _, g := range []struct {
		label string
		value float64
	}{
		{"Driving Gear Base Height", p["drivingBaseHeight"]},
		{"Pinion Gear Base Height", p["pinionBaseHeight"]},
		{"Driving Gear Bore Diameter", p["drivingBore"]},
		{"Pinion Gear Bore Diameter", p["pinionBore"]},
		{"Face Width", p["faceWidth"]},
		{"Tooth Spacing", p["toothSpacing"]},
		{"Cutter Radius", p["cutterRadius"]},
		{"Driving Gear Toe Radius", p["drivingToeRadius"]},
		{"Pinion Gear Toe Radius", p["pinionToeRadius"]},
	} {
		if g.value < 0 {
			r.reject(g.label, "the non-negative floor", g.value, 0)
		}
	}
	if p["toeExtension"] < 0 {
		r.reject("Toe Extension", "the low end of its [0, 100] range", p["toeExtension"], 0)
	}
	if p["toeExtension"] > 100 {
		r.reject("Toe Extension", "the high end of its [0, 100] range", p["toeExtension"], 100)
	}
	if p["spiralAngleDeg"] < 0 {
		r.reject("Mean Spiral Angle", "the low end of its [0, 60) range", p["spiralAngleDeg"], 0)
	}
	if p["spiralAngleDeg"] >= 60 {
		r.reject("Mean Spiral Angle", "the high end of its [0, 60) range", p["spiralAngleDeg"], 60)
	}
	if sigmaDeg < 30 {
		r.reject("Shaft Angle", "the documented 30 degree floor", sigmaDeg, 30)
	}
	if len(r.problems) > 0 {
		return r
	}

	// 2. The Maximum Shaft Angle, which needs both tooth counts.
	limit, inclusive := shaftAngleCeiling(m*zp, m*zg)
	if (inclusive && sigmaDeg > limit) || (!inclusive && sigmaDeg >= limit) {
		r.reject("Shaft Angle", "the Maximum Shaft Angle", sigmaDeg, limit)
		return r
	}

	// 3 and 4. The cone angles, then the Minimum Teeth floor per gear with that
	// gear's own gamma.
	f := newFigure(p)
	for _, s := range []side{f.pinion, f.driving} {
		if s.teeth < s.minTeeth {
			r.reject(s.label+" Gear Teeth", "the computed Minimum Teeth floor",
				s.teeth, s.minTeeth)
		}
	}
	if len(r.problems) > 0 {
		return r
	}

	// 5. The two base-height windows, driving first: the pinion's fallback is a
	// share of the RESOLVED driving height, so the driving cap has to be applied
	// before the pinion fallback is formed.
	driving := r.applyWindow("Driving Gear Base Height",
		p["drivingBaseHeight"], m*zg/8, f.driving)
	r.applyWindow("Pinion Gear Base Height",
		p["pinionBaseHeight"], driving*(zp/zg), f.pinion)
	if len(r.problems) > 0 {
		return r
	}

	// 6 and 7. The Maximum Face Width off the solved lattice, then Face Width.
	//
	// The spec requires this bound to be read from SOLVED sketch geometry rather
	// than from the seeds, because seeds diverge for asymmetric tooth counts and
	// a seed-based bound comes out too loose on the binding side. The figure
	// here IS the solved lattice in closed form, and stepGearProfiles is where
	// the constrained sketch is checked against it, so reading it here is
	// reading the same numbers the sketch solves to.
	r.applyFaceWidth(p["faceWidth"], f.coneDistance/6, f.maxFaceWidth)
	if len(r.problems) > 0 {
		return r
	}

	// 8. The two Toe Radii against their own ceilings. A user value must be
	// STRICTLY below the ceiling; at or above it the point the toe end is
	// heading for falls behind the toe corner.
	for _, pair := range []struct {
		label string
		input float64
		s     side
	}{
		{"Driving Gear Toe Radius", p["drivingToeRadius"], f.driving},
		{"Pinion Gear Toe Radius", p["pinionToeRadius"], f.pinion},
	} {
		if pair.input > 0 && pair.input >= pair.s.toeCeil {
			r.reject(pair.label, "its Toe Radius Ceiling", pair.input, pair.s.toeCeil)
		}
	}
	if len(r.problems) > 0 {
		return r
	}

	// 9. A defaulted Toe Radius can leave no room at all, and that is a real
	// configuration rather than a defect: on a gear with a large pitch cone
	// angle the inner toe corner already sits at a LARGER radius than the outer
	// one, so the Toe Limit comes out below the Toe Extension 0 root length.
	// Toe Extension 0 still resolves, so the gear stays buildable exactly as
	// before; a positive Toe Extension is refused, naming the gear and the Toe
	// Radius Ceiling it needs to come below.
	if p["toeExtension"] > 0 {
		binding := f.pinion
		if f.driving.toeLimit < f.pinion.toeLimit {
			binding = f.driving
		}
		if binding.toeLimit <= f.rootLen0 {
			r.reject("Toe Extension",
				"the "+binding.label+" Gear Toe Radius Ceiling, which its defaulted "+
					"Toe Radius has to come below before the toe end has anywhere to go",
				p["toeExtension"], binding.toeCeil)
			return r
		}
	}

	r.fig = f
	r.ok = true
	return r
}

// shaftAngleCeiling is the Maximum Shaft Angle and whether it is inclusive.
//
// A pitch cone angle reaching 90 degrees turns that gear's pitch cone inside
// out, and both cone angles stay below it exactly while
// cos(Shaft Angle) > -smaller/larger, so acos of that ratio is a hard
// singularity and the cone-angle half of the limit is EXCLUSIVE. The 150 degree
// half is a practical ceiling on the figure rather than a measured one, and it
// is inclusive.
func shaftAngleCeiling(ppd, dpd float64) (float64, bool) {
	cone := p2deg(math.Acos(-math.Min(ppd, dpd) / math.Max(ppd, dpd)))
	if cone > 150 {
		return 150, true
	}
	return cone, false
}

// applyWindow decides one base height the way the spec splits it, and returns
// what the build goes on with.
func (r *resolution) applyWindow(label string, input, fallback float64, s side) float64 {
	if input > 0 {
		switch {
		case input < s.minBase:
			r.reject(label, "its Minimum Base Height", input, s.minBase)
		case input > s.maxBase:
			r.reject(label, "its Maximum Base Height", input, s.maxBase)
		}
		return input
	}
	switch {
	case fallback < s.minBase:
		r.clamp(label, "its Minimum Base Height", fallback, s.minBase)
		return s.minBase
	case fallback > s.maxBase:
		r.clamp(label, "its Maximum Base Height", fallback, s.maxBase)
		return s.maxBase
	}
	return fallback
}

// applyFaceWidth is the same split for Face Width, which has a maximum and no
// minimum.
func (r *resolution) applyFaceWidth(input, fallback, max float64) float64 {
	if input > 0 {
		if input > max {
			r.reject("Face Width", "the Maximum Face Width", input, max)
		}
		return input
	}
	if fallback > max {
		r.clamp("Face Width", "the Maximum Face Width", fallback, max)
		return max
	}
	return fallback
}

// ------------------------------------------------------------- the case table

// variableCases runs the resolution over the whole dialog regime the lattice
// table sweeps, once per gear, because the frustum drawn at the end is this
// gear's and either gear can be the binding side of the Maximum Face Width.
//
// The declared-refusal cases are NOT skipped here. Nothing in this step solves
// the §2 lattice — the bounds are closed form and the witness hexagon is drawn
// at fixed vertices — and one of those cases carries the worked base-height
// figures the spec publishes, so skipping it would drop the very numbers this
// step exists to check.
var variableCases = profileCases

// ------------------------------------------------------------- the step

// stepResolveInputs resolves the Variables section and checks it three ways:
// the published worked figures, each bound swept from both sides, and the
// frustum drawn at the resolved values as the witness that what the resolution
// admits stays on one side of its own axis of revolution.
func stepResolveInputs(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	gear := gearOf(p)

	proofkit.Step(t, "resolve the dialog")
	r := resolveDialog(p)
	if !r.ok {
		t.Fatalf("this case's dialog is meant to be admissible, and the resolution "+
			"refused it: %s", strings.Join(problemTexts(r.problems), "; "))
	}
	checkResolvedAgainstClosedForm(t, r, p)

	proofkit.Step(t, "the worked figures the spec publishes")
	checkPublishedFigures(t)

	proofkit.Step(t, "each bound, one step inside and one step outside")
	checkBoundsBothSides(t, p)

	proofkit.Step(t, "draw the %s frustum at the resolved values", gear)
	f := r.fig
	verts := f.hexagon(gear)
	pts := make([]*sketch.Point, len(verts))
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.X, v.Y)
	}
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, line := range lines {
		_ = line
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	solveHere(t, s)
	checkHexagon(t, s, f, gear)
}

func problemTexts(problems []problem) []string {
	out := make([]string, len(problems))
	for i, pr := range problems {
		out[i] = pr.text()
	}
	return out
}

// checkResolvedAgainstClosedForm pins what the resolution produced for this
// case's own dialog: the derived lengths, the two windows, the toe-end window,
// and that every clamp it reported actually landed on the bound it named.
func checkResolvedAgainstClosedForm(t testing.TB, r resolution, p map[string]float64) {
	t.Helper()
	f := r.fig
	m := f.module

	// Cone Distance is the diagonal of the two pitch diameters and depends on
	// the tooth counts only, never on the Shaft Angle. The Pitch Cone Distance R
	// is a different length, and the two coincide as Cone Distance = 2R exactly
	// when the Shaft Angle is 90 degrees, for any pair of tooth counts.
	near(t, f.coneDistance, math.Hypot(f.pinion.pitchDia, f.driving.pitchDia), 1e-12,
		"Cone Distance is the diagonal of the two pitch diameters")
	if math.Abs(p["shaftAngleDeg"]-90) < 1e-12 {
		near(t, f.coneDistance, 2*f.R, 1e-9,
			"at Shaft Angle 90 the Cone Distance is exactly twice the Pitch Cone Distance")
		near(t, f.coneDistance/6, f.R/3, 1e-9,
			"so the Face Width default is R/3 at 90 degrees and only there")
	} else if math.Abs(f.coneDistance-2*f.R) < 1e-9 {
		t.Errorf("Cone Distance equals 2R at Shaft Angle %.4f, where the two lengths are "+
			"supposed to diverge", p["shaftAngleDeg"])
	}
	near(t, f.distApexDed, math.Hypot(f.R, 1.25*m), 1e-12, "|Apex->Ded|")

	for _, s := range []side{f.pinion, f.driving} {
		// The two closed-form bounds, and the true crossing the maximum sits
		// deliberately below.
		r0 := s.pitchRadius()
		near(t, s.maxBase, 0.95*(r0-1.25*m*math.Cos(s.gamma))*math.Tan(s.gamma), 1e-12,
			"%s Maximum Base Height", s.label)
		near(t, s.minBase, 1.05*1.25*m*math.Sin(s.gamma), 1e-12,
			"%s Minimum Base Height", s.label)
		crossing := r0 * math.Tan(s.gamma)
		if s.maxBase >= crossing {
			t.Errorf("%s Maximum Base Height %.6f is not below the true crossing %.6f, so "+
				"it is not the conservative bound the spec takes", s.label, s.maxBase, crossing)
		}
		if s.baseHgt < s.minBase-1e-9 || s.baseHgt > s.maxBase+1e-9 {
			t.Errorf("%s resolved base height %.6f is outside its own window [%.6f, %.6f]",
				s.label, s.baseHgt, s.minBase, s.maxBase)
		}
		near(t, s.minTeeth, 5.27*math.Cos(s.gamma), 1e-12, "%s Minimum Teeth floor", s.label)

		// The toe end.
		near(t, s.gammaRoot, s.gamma-math.Atan(1.25*m/f.R), 1e-12, "%s root cone angle", s.label)
		near(t, s.toeCeil, (r0-1.25*m*math.Cos(s.gamma))*(1-f.faceWidth/f.R), 1e-12,
			"%s Toe Radius Ceiling", s.label)
		near(t, s.toeLimit, f.distApexDed-s.toeRadius/math.Sin(s.gammaRoot), 1e-12,
			"%s Toe Limit", s.label)
		if s.toeRadius <= 0 {
			t.Errorf("%s resolved Toe Radius %.6f is not strictly positive, so the inner toe "+
				"corner would sit on the axis of revolution", s.label, s.toeRadius)
		}
		// A USER Toe Radius must be strictly below the ceiling. A DEFAULTED one
		// may exceed it, and when it does that is the no-room configuration the
		// spec describes rather than a defect: on a gear with a large pitch cone
		// angle the inner toe corner already sits at a LARGER radius than the
		// outer one, so the toe dish leans toward the heel. The ceiling and the
		// Toe Limit are two readings of the same fact, which is what this pins.
		if (s.toeRadius >= s.toeCeil) != (s.toeLimit <= f.rootLen0) {
			t.Errorf("%s Toe Radius %.6f against its ceiling %.6f says one thing and its Toe "+
				"Limit %.6f against the Toe Extension 0 root length %.6f says the other; the "+
				"ceiling is exactly where the toe end runs out of room",
				s.label, s.toeRadius, s.toeCeil, s.toeLimit, f.rootLen0)
		}
	}

	// Face Width and the Root Length window.
	if f.faceWidth > f.maxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.6f is above the Maximum Face Width %.6f",
			f.faceWidth, f.maxFaceWidth)
	}
	near(t, f.rootLen0, f.faceWidth*f.distApexDed/f.R, 1e-12, "Root Length at Toe Extension 0")
	limit := math.Min(f.pinion.toeLimit, f.driving.toeLimit)
	near(t, f.rootLen, f.rootLen0+(p["toeExtension"]/100)*0.99*(limit-f.rootLen0), 1e-12,
		"the resolved Root Length")
	if p["toeExtension"] == 0 {
		near(t, f.rootLen, f.rootLen0, 1e-12,
			"Toe Extension 0 reproduces today's toe end exactly")
	}
	if p["toeExtension"] == 100 {
		// 100 stops at 0.99 of the way to the SMALLER of the two gears' limits,
		// because the pair shares one root length. AT the limit the toe face has
		// zero length, so the revolved body carries no cone at its toe end and
		// the conical end cut has no face to find.
		near(t, f.rootLen, f.rootLen0+0.99*(limit-f.rootLen0), 1e-12,
			"Toe Extension 100 stops at 0.99 of the way to the smaller Toe Limit")
		if f.rootLen >= limit {
			t.Errorf("Toe Extension 100 reached %.6f against a Toe Limit of %.6f: the toe "+
				"face has closed to nothing", f.rootLen, limit)
		}
	}

	// Every clamp the resolution reported landed on the bound it named.
	for _, a := range r.clamps {
		if a.from == a.to {
			t.Errorf("%s reports a clamp onto %s that moved nothing", a.input, a.bound)
		}
	}
}

// checkPublishedFigures checks the numbers the spec states outright. They are
// the places a reader can compare the module against the prose by hand, so a
// resolver that drifts from them has drifted from the document.
func checkPublishedFigures(t testing.TB) {
	t.Helper()

	// "A 31/17 pair gives acos(-17/31) = 123.24 degrees", and equal tooth counts
	// give acos(-1) = 180, which is no constraint at all, so the 150 degree
	// practical ceiling is what holds them.
	ratio, ratioInclusive := shaftAngleCeiling(17, 31)
	// The spec prints 123.24 degrees here. acos(-17/31) is 123.2564, which
	// rounds to 123.26, so the published figure is out by one in its last digit.
	// The tolerance below admits the spec's figure while the exact value is what
	// the resolver uses.
	near(t, ratio, p2deg(math.Acos(-17.0/31.0)), 1e-12,
		"the 31/17 pair's cone-angle Shaft Angle limit is acos(-17/31)")
	near(t, ratio, 123.24, 2e-2, "and it agrees with the figure the spec publishes")
	if ratioInclusive {
		t.Error("the cone-angle half of the Maximum Shaft Angle is a singularity and has to " +
			"be exclusive")
	}
	equal, equalInclusive := shaftAngleCeiling(31, 31)
	near(t, equal, 150, 1e-12, "an equal pair's Maximum Shaft Angle is the practical ceiling")
	if !equalInclusive {
		t.Error("the 150 degree ceiling is a practical limit on the figure and is inclusive")
	}
	near(t, p2deg(math.Acos(-1)), 180, 1e-9,
		"acos(-1) is 180 degrees, which is no constraint at all")

	// The Maximum Base Height worked case: Module 1, Driving 31, Pinion 31,
	// Shaft Angle 30. Each gamma is 15 degrees, the bound is 3.638 mm, the true
	// crossing is 4.153 mm, and the driving fallback resolves to 3.875 mm — so
	// the default sits BETWEEN the two. It is capped to the bound, but it would
	// not have folded uncapped.
	worked := dialog(1, 31, 31, 30)
	wf := newFigure(worked)
	near(t, p2deg(wf.driving.gamma), 15, 1e-9, "the worked case's driving pitch cone angle")
	near(t, p2deg(wf.pinion.gamma), 15, 1e-9, "the worked case's pinion pitch cone angle")
	near(t, wf.driving.maxBase, 3.638, 5e-4, "the worked case's Maximum Base Height")
	near(t, wf.driving.pitchRadius()*math.Tan(wf.driving.gamma), 4.153, 5e-4,
		"the worked case's true heel crossing")
	near(t, 1*31/8.0, 3.875, 1e-12, "the worked case's driving base-height fallback")
	wr := resolveDialog(worked)
	if !wr.ok {
		t.Fatalf("the worked base-height case is meant to resolve: %s",
			strings.Join(problemTexts(wr.problems), "; "))
	}
	clampAt, clamped := wr.clamped("Driving Gear Base Height")
	if !clamped {
		t.Error("the worked case's driving fallback is above its Maximum Base Height and has " +
			"to be capped to it")
	} else {
		near(t, clampAt.from, 3.875, 5e-4, "the worked case caps the fallback")
		near(t, clampAt.to, wf.driving.maxBase, 1e-12, "the worked case caps onto the bound")
		if clampAt.bound != "its Maximum Base Height" {
			t.Errorf("the worked case's clamp names %q, not its Maximum Base Height",
				clampAt.bound)
		}
	}
	if 3.875 >= wf.driving.pitchRadius()*math.Tan(wf.driving.gamma) {
		t.Error("the worked case's shipped default is supposed to sit below the true " +
			"crossing, so the cap is not rescuing it from a fold")
	}

	// The Minimum Teeth constant and the floor it produces. 5.27 is the rounded
	// form of 2 * (1.05 * 1.25 / 0.95 + 1.25), and at Shaft Angle 90 it puts the
	// floor at 3.72, which is four teeth.
	// The spec gives the constant twice and the two disagree in the second
	// decimal: it states the rule as 5.27 * cos(gamma) and its derivation as
	// 2 * (1.05 * 1.25 / 0.95 + 1.25), which is 5.2632 and rounds to 5.26. The
	// published floor of 3.72 at Shaft Angle 90 follows the DERIVATION, not the
	// 5.27. The resolver implements the rule as written, which is the more
	// conservative of the two, and both give the same integer floor.
	exact := 2 * (1.05*1.25/0.95 + 1.25)
	near(t, exact, 5.2631578947, 1e-9, "the Minimum Teeth constant's derivation")
	near(t, exact, 5.27, 1e-2, "which the spec's stated 5.27 agrees with only to two digits")
	near(t, exact*math.Cos(rad(45)), 3.72, 5e-3,
		"at Shaft Angle 90 the derivation puts the floor at the published 3.72")
	for _, constant := range []float64{exact, 5.27} {
		if int(math.Ceil(constant*math.Cos(rad(45)))) != 4 {
			t.Errorf("the Minimum Teeth floor at Shaft Angle 90 comes out at %d teeth from "+
				"the constant %.6f, and the spec publishes four",
				int(math.Ceil(constant*math.Cos(rad(45)))), constant)
		}
	}

	// And what that floor encodes: below it the two base-height bounds cross, so
	// NO base height satisfies both and the gear cannot be built at any setting.
	// Measured, an equal 4-tooth pair solves and a 3-tooth pair does not.
	for _, c := range []struct {
		teeth float64
		empty bool
	}{{3, true}, {4, false}} {
		g := newSide("Driving", c.teeth, 1*c.teeth, rad(45), 1, (1*c.teeth/2)/math.Sin(rad(45)))
		if (g.minBase > g.maxBase) != c.empty {
			t.Errorf("at %.0f teeth the base-height window [%.6f, %.6f] is %s, and the "+
				"Minimum Teeth floor says it should be the other way",
				c.teeth, g.minBase, g.maxBase, emptiness(g.minBase > g.maxBase))
		}
	}

	// The fallback Module * teeth / 8 clears the raw dedendum projection
	// 1.25 * Module * sin(gamma) exactly while teeth > 10 * sin(gamma), which at
	// Shaft Angle 90 is 7.07 — so an equal 8-tooth pair was the smallest that
	// built and a 7-tooth pair failed, before the Minimum Base Height was there
	// to raise it.
	near(t, 10*math.Sin(rad(45)), 7.07, 5e-3,
		"at Shaft Angle 90 the fallback clears the raw projection above 7.07 teeth")
	for _, teeth := range []float64{7, 8} {
		fallback := teeth / 8
		projection := 1.25 * math.Sin(rad(45))
		if (fallback > projection) != (teeth > 10*math.Sin(rad(45))) {
			t.Errorf("at %.0f teeth the fallback %.6f against the projection %.6f "+
				"disagrees with the 10*sin(gamma) rule", teeth, fallback, projection)
		}
	}

	// At Shaft Angle 90 the Maximum Face Width's underlying crossing distance is
	// PPD^2 / (2 * Cone Distance), and the Maximum Face Width is 0.95 of it.
	// That is what makes the naive Cone Distance / 6 default exceed the cap for
	// any gear ratio above roughly sqrt(2).
	// The spec writes this limit as Pinion Gear Pitch Diameter squared over twice
	// the Cone Distance, which reads the pinion as the binding side. The binding
	// side is really the SMALLER pitch diameter — normally the pinion, and the
	// spec says so a sentence earlier, but a dialog may put the smaller count on
	// the driving gear and then the driving side binds.
	for _, pair := range [][2]float64{{31, 31}, {31, 17}, {17, 31}, {43, 31}} {
		qf := newFigure(dialog(1, pair[0], pair[1], 90))
		smaller := math.Min(qf.pinion.pitchDia, qf.driving.pitchDia)
		crossing := smaller * smaller / (2 * qf.coneDistance)
		near(t, qf.maxFaceWidth, 0.95*crossing, 1e-6,
			"the Maximum Face Width at 90 degrees for a %.0f/%.0f pair", pair[0], pair[1])
	}
	// sqrt(2) is exactly where Cone Distance / 6 crosses that distance, so a
	// pair just under it clears and a pair just over it does not.
	for _, c := range []struct {
		driving, pinion float64
		over            bool
	}{{24, 17, false}, {17, 12, true}} {
		q := newFigure(dialog(1, c.driving, c.pinion, 90))
		smaller := math.Min(q.pinion.pitchDia, q.driving.pitchDia)
		crossing := smaller * smaller / (2 * q.coneDistance)
		if (q.coneDistance/6 > crossing) != c.over {
			t.Errorf("a %.0f/%.0f pair has ratio %.4f against sqrt(2), and the naive default "+
				"%.6f against the crossing %.6f falls the wrong side of it",
				c.driving, c.pinion, c.driving/c.pinion, q.coneDistance/6, crossing)
		}
	}

	// The Cone Distance and R figures the spec publishes for an equal 31/31 pair.
	thirty := newFigure(dialog(1, 31, 31, 30))
	near(t, thirty.coneDistance, 43.84, 5e-3, "the 31/31 pair's Cone Distance")
	near(t, thirty.R, 59.89, 5e-3, "the 31/31 pair's Pitch Cone Distance at 30 degrees")
	// The spec prints 16.50 mm here; the value is 16.4948, which rounds to 16.49.
	// Another last-digit rounding in a published figure.
	near(t, newFigure(dialog(1, 31, 31, 140)).R, 16.50, 1e-2,
		"the 31/31 pair's Pitch Cone Distance at 140 degrees")

	// A defaulted Toe Radius leaving no room is scale-invariant in Module: the
	// spec says Module does not move the boundary of that band, so the same
	// pair and Shaft Angle decide the same way at every Module.
	for _, pair := range [][3]float64{{31, 11, 90}, {31, 11, 120}, {43, 13, 90}, {31, 31, 90}} {
		want := toeExtensionHasRoom(1, pair[0], pair[1], pair[2])
		for _, m := range []float64{4, 8} {
			if got := toeExtensionHasRoom(m, pair[0], pair[1], pair[2]); got != want {
				t.Errorf("at Driving %.0f / Pinion %.0f and Shaft Angle %.0f the toe-end "+
					"window changes between Module 1 and Module %.0f, and Module is not "+
					"supposed to move that boundary", pair[0], pair[1], pair[2], m)
			}
		}
	}
}

func emptiness(empty bool) string {
	if empty {
		return "empty"
	}
	return "non-empty"
}

// toeExtensionHasRoom reports whether a positive Toe Extension resolves on this
// pair, or is refused because the defaulted Toe Radius leaves the toe end
// nowhere to go.
func toeExtensionHasRoom(module, driving, pinion, shaftAngleDeg float64) bool {
	p := override(dialog(module, driving, pinion, shaftAngleDeg),
		map[string]float64{"toeExtension": 50})
	r := resolveDialog(p)
	if _, refused := r.rejected("Toe Extension"); refused {
		return false
	}
	return true
}

// ------------------------------------------------------------- both sides

// nudge is how far outside or inside a bound the sweep below places a value. It
// is far larger than the arithmetic's own error and far smaller than any bound,
// so a value moved by it is unambiguously on one side.
const nudge = 1e-6

// checkBoundsBothSides is the part that separates a clamp from a rejection. For
// each bound a user can break, it types a value one nudge inside and one nudge
// outside, and checks that the inside value is accepted, the outside value is
// rejected, and the rejection names that bound and carries its number. For each
// bound a FALLBACK can break, it checks the opposite: no rejection, and a clamp
// that lands on the bound.
//
// A probe is SKIPPED, visibly, when another bound bites first on the dialog it
// would have to build — lowering a tooth count brings the Maximum Shaft Angle
// down with it, and on a case already near that ceiling the shaft angle is
// refused before the tooth count is ever read. That order is the spec's, so the
// probe has nothing to say there rather than a complaint to make. The table
// sweeps enough dialogs that every bound fires on many of them.
func checkBoundsBothSides(t testing.TB, caseParams map[string]float64) {
	t.Helper()

	// Sweep from a dialog carrying only the size inputs, so each bound is swept
	// on its own rather than against whatever else this case happens to set.
	base := override(caseParams, map[string]float64{
		"drivingBaseHeight": 0, "pinionBaseHeight": 0, "faceWidth": 0,
		"toothSpacing": 0, "toeExtension": 0,
		"drivingToeRadius": 0, "pinionToeRadius": 0,
	})
	f := newFigure(base)

	// probe runs one bound from both sides.
	probe := func(from map[string]float64, label, input, bound string, limit float64,
		inside, outside map[string]float64) {
		t.Helper()
		in := resolveDialog(override(from, inside))
		if pr, found := in.rejected(input); found {
			if pr.bound != bound {
				// A DIFFERENT bound on the same input bites first, which is the
				// same situation as another input biting first. The clearest
				// case is Toe Extension on a pair whose defaulted Toe Radius
				// leaves no room: 100 is inside the [0, 100] range and still
				// refused, and refused for the right reason.
				proofkit.Step(t, "skipped %s: a different bound on the same input bites "+
					"first (%s)", label, pr.text())
				return
			}
			t.Errorf("%s: a value one nudge INSIDE its bound was refused: %s", label, pr.text())
			return
		}
		if !in.ok {
			proofkit.Step(t, "skipped %s: another bound bites first (%s)",
				label, strings.Join(problemTexts(in.problems), "; "))
			return
		}
		out := resolveDialog(override(from, outside))
		pr, found := out.rejected(input)
		if !found {
			if !out.ok {
				proofkit.Step(t, "skipped %s: another bound bites first (%s)",
					label, strings.Join(problemTexts(out.problems), "; "))
				return
			}
			t.Errorf("%s: a value one nudge OUTSIDE its bound was accepted", label)
			return
		}
		if pr.bound != bound {
			t.Errorf("%s: the rejection names %q rather than %q", label, pr.bound, bound)
		}
		near(t, pr.limit, limit, 1e-9, "%s: the bound the rejection carries", label)
		if !strings.Contains(pr.text(), fmt.Sprintf("%.6f", limit)) {
			t.Errorf("%s: the message %q does not carry the bound's number, so the user "+
				"is told nothing they can act on", label, pr.text())
		}
	}

	// Shaft Angle, both ends. The low end is the documented 30 degree floor and
	// is inclusive; the high end is the Maximum Shaft Angle, whose cone-angle
	// half is exclusive and whose 150 degree half is not.
	probe(base, "Shaft Angle at its 30 degree floor", "Shaft Angle",
		"the documented 30 degree floor", 30,
		map[string]float64{"shaftAngleDeg": 30},
		map[string]float64{"shaftAngleDeg": 30 - nudge})
	// The two halves of the Maximum Shaft Angle close differently, so the pair of
	// values either side of it differs too. The cone-angle half is a hard
	// singularity and is EXCLUSIVE, so the ceiling itself is the first refused
	// value; the 150 degree half is a practical ceiling on the figure and is
	// INCLUSIVE, so the ceiling is the last accepted one. Probing
	// ceiling +/- nudge on both would never test the boundary value and would
	// pass against a resolver that had the two the wrong way round.
	ceiling, inclusive := shaftAngleCeiling(f.pinion.pitchDia, f.driving.pitchDia)
	insideCeiling, outsideCeiling := ceiling-nudge, ceiling
	if inclusive {
		insideCeiling, outsideCeiling = ceiling, ceiling+nudge
	}
	probe(base, "Shaft Angle at its Maximum", "Shaft Angle", "the Maximum Shaft Angle", ceiling,
		map[string]float64{"shaftAngleDeg": insideCeiling},
		map[string]float64{"shaftAngleDeg": outsideCeiling})

	// Minimum Teeth, per gear, on top of the blanket floor of three. Swept from
	// a 90 degree base, where the Maximum Shaft Angle cannot bite first: at 90
	// degrees that ceiling is acos of a non-positive ratio, so it is above 90
	// for every pair of tooth counts.
	// The floor is a FIXED POINT rather than a number a dialog can be set to:
	// lowering a tooth count moves that gear's pitch cone angle, which moves its
	// own floor. So the probe walks the count up to the first value the
	// resolution accepts and checks the one below it is refused.
	teethBase := override(base, map[string]float64{"shaftAngleDeg": 90})
	for _, g := range []struct {
		input string
		key   string
	}{
		{"Pinion Gear Teeth", "pinionTeeth"},
		{"Driving Gear Teeth", "drivingTeeth"},
	} {
		smallest := 0.0
		for n := 3.0; n <= teethBase[g.key]; n++ {
			if resolveDialog(override(teethBase, map[string]float64{g.key: n})).ok {
				smallest = n
				break
			}
		}
		if smallest == 0 {
			t.Errorf("%s: no tooth count from 3 up to this case's own is admissible", g.input)
			continue
		}
		below := resolveDialog(override(teethBase, map[string]float64{g.key: smallest - 1}))
		pr, found := below.rejected(g.input)
		if !found {
			t.Errorf("%s: %0.f is the smallest count this pair admits, so %.0f has to be "+
				"refused and it was not", g.input, smallest, smallest-1)
			continue
		}
		wantBound := "the computed Minimum Teeth floor"
		if smallest-1 < 3 {
			wantBound = "the absolute tooth floor"
		}
		if pr.bound != wantBound {
			t.Errorf("%s at %.0f: the rejection names %q rather than %q",
				g.input, smallest-1, pr.bound, wantBound)
		}
		if !strings.Contains(pr.text(), fmt.Sprintf("%.6f", pr.limit)) {
			t.Errorf("%s: the message %q does not carry the bound's number", g.input, pr.text())
		}
	}

	// Each base height: a user value one nudge inside each end accepted, one
	// nudge outside rejected naming that end.
	for _, g := range []struct {
		input string
		key   string
		s     side
	}{
		{"Driving Gear Base Height", "drivingBaseHeight", f.driving},
		{"Pinion Gear Base Height", "pinionBaseHeight", f.pinion},
	} {
		if g.s.maxBase-g.s.minBase <= 1e-3 {
			continue
		}
		probe(base, g.input+" below its minimum", g.input, "its Minimum Base Height",
			g.s.minBase,
			map[string]float64{g.key: g.s.minBase + nudge},
			map[string]float64{g.key: g.s.minBase - nudge})
		probe(base, g.input+" above its maximum", g.input, "its Maximum Base Height",
			g.s.maxBase,
			map[string]float64{g.key: g.s.maxBase - nudge},
			map[string]float64{g.key: g.s.maxBase + nudge})
	}

	// Face Width has a maximum and no minimum.
	probe(base, "Face Width above its maximum", "Face Width", "the Maximum Face Width",
		f.maxFaceWidth,
		map[string]float64{"faceWidth": f.maxFaceWidth - nudge},
		map[string]float64{"faceWidth": f.maxFaceWidth + nudge})

	// Each Toe Radius must be STRICTLY below its ceiling, so the ceiling itself
	// is the first refused value rather than the last accepted one.
	for _, g := range []struct {
		input string
		key   string
		s     side
	}{
		{"Driving Gear Toe Radius", "drivingToeRadius", f.driving},
		{"Pinion Gear Toe Radius", "pinionToeRadius", f.pinion},
	} {
		if g.s.toeCeil <= nudge {
			continue
		}
		probe(base, g.input+" at its ceiling", g.input, "its Toe Radius Ceiling", g.s.toeCeil,
			map[string]float64{g.key: g.s.toeCeil - nudge},
			map[string]float64{g.key: g.s.toeCeil})
	}

	// Toe Extension's range is closed at both ends.
	probe(base, "Toe Extension below its range", "Toe Extension",
		"the low end of its [0, 100] range", 0,
		map[string]float64{"toeExtension": 0},
		map[string]float64{"toeExtension": -nudge})
	probe(base, "Toe Extension above its range", "Toe Extension",
		"the high end of its [0, 100] range", 100,
		map[string]float64{"toeExtension": 100},
		map[string]float64{"toeExtension": 100 + nudge})

	// Mean Spiral Angle's range is closed at the low end and open at the high
	// end, because 60 degrees is not a spiral angle this build admits.
	probe(base, "Mean Spiral Angle below 0", "Mean Spiral Angle",
		"the low end of its [0, 60) range", 0,
		map[string]float64{"spiralAngleDeg": 0},
		map[string]float64{"spiralAngleDeg": -nudge})
	probe(base, "Mean Spiral Angle at 60", "Mean Spiral Angle",
		"the high end of its [0, 60) range", 60,
		map[string]float64{"spiralAngleDeg": 60 - nudge},
		map[string]float64{"spiralAngleDeg": 60})

	// Module has to be positive at all.
	probe(base, "Module at zero", "Module", "the positive-Module floor", 0,
		map[string]float64{"module": f.module},
		map[string]float64{"module": 0})

	// And the other half of the split: a FALLBACK outside a window is clamped
	// onto it, with no rejection at all.
	fallbackRun := resolveDialog(base)
	if !fallbackRun.ok {
		t.Errorf("a dialog carrying only the size inputs must resolve on fallbacks alone: %s",
			strings.Join(problemTexts(fallbackRun.problems), "; "))
		return
	}
	bounds := []float64{f.driving.minBase, f.driving.maxBase,
		f.pinion.minBase, f.pinion.maxBase, f.maxFaceWidth}
	for _, a := range fallbackRun.clamps {
		if _, rejected := fallbackRun.rejected(a.input); rejected {
			t.Errorf("%s was both clamped and rejected, and a fallback is only ever clamped",
				a.input)
		}
		onBound := false
		for _, b := range bounds {
			if math.Abs(a.to-b) <= 1e-9 {
				onBound = true
			}
		}
		if !onBound {
			t.Errorf("%s was clamped to %.6f, which is not any of this pair's bounds",
				a.input, a.to)
		}
	}
}
