// This file holds the bevel pair's sketch steps: the Anchor sketch (§1), the
// Gear Profiles lattice (§2), the per-gear virtual-spur tooth (§3), the per-gear
// Profile hexagon, the Bore sketch, and the spiral build's 2-D cutter-arc trace
// (§3a step C).
//
// Three things in them are outside what a sketch engine can hold, and each is
// recorded beside the thing it cannot reach.
//
//  1. The proof SEEDS every §2 point at its closed form. So what the lattice
//     assertion proves is that the constraints solve FROM a correct seed, and
//     never that the generated module's seed is correct. A seed defect therefore
//     reaches Fusion untested, which is how the toe-line seed defect §2 records
//     got there, and it is why [BEVEL-F-SEED-HELD] has to be a gate inside the
//     module rather than only a case here. See stepGearProfiles.
//
//  2. `sketch.project(...)` brings a Fusion point in ASSOCIATIVELY and leaves it
//     carrying free degrees of freedom ([PB-PROJECT-NOT-FIXED]). The engine's
//     CreateReferencePoint and CreateReferenceLine are coordinate-LOCKED: the
//     solver never moves them. So the projection is modelled as already pinned,
//     and the free-DOF half of that rule is not reproduced.
//
//  3. Three constraints Fusion needs are LEFT OUT here, because the engine's
//     counterpart carries more rows than Fusion's and writing Fusion's arity
//     makes the lattice redundant at DOF 0 ([PB-NO-OVERCONSTRAIN]). Each omission
//     is written at its own site: the two base-height perpendiculars (G->H, I->J),
//     the two toe-line parallels (M->N, O->P), and the second row of the
//     "Constrain Point I with center point" coincidence.
package bevelgear_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// anchorProjectionSource is the source id every projection of the user's Center
// Point carries. §1 stashes the Anchor sketch's own projected point and §2
// re-projects THAT rather than the raw user selection; the engine refuses a
// reference to another sketch's point outright, so each sketch below carries its
// own reference point tagged with this id. What that models is one link of the
// chain, not the chain.
const anchorProjectionSource = "Anchor sketch centre projection"

// anchorLineSource is the same for the Anchor Line itself, which §2 projects so
// it has something to take the perpendicular against.
const anchorLineSource = "Anchor sketch anchor line projection"

// anchorSeed is the ±0.5 cm §1 seeds the Anchor Line's endpoints at, in this
// proof's millimetres: the seeded length is 10 mm and the dimension locks it
// there.
const anchorSeed = 5.0

// ---------------------------------------------------------------- case tables

// sketchCase names one case by the dialog values it comes from. The base is
// `params`, the shipped dialog default this package already declares — module 1,
// an equal 31/31 pair at a 90 degree shaft angle, every optional length left at 0
// so it resolves, the bore enabled and a 35 degree right-hand spiral — so a case
// here and a case the drawings take are the same set of values.
func sketchCase(mut ...func(map[string]float64)) map[string]float64 {
	p := params(nil)
	for _, f := range mut {
		f(p)
	}
	return p
}

func with(key string, value float64) func(map[string]float64) {
	return func(p map[string]float64) { p[key] = value }
}

func shaft(deg float64) func(map[string]float64) {
	return func(p map[string]float64) { p[keyShaftAngle] = deg }
}

func pair(module, driving, pinion float64) func(map[string]float64) {
	return func(p map[string]float64) {
		p[keyModule], p[keyDrivingTeeth], p[keyPinionTeeth] = module, driving, pinion
	}
}

// anchorCases put the user's Center Point on the sketch origin and off it.
// Nothing in the dialog requires it to sit at the origin, and §2 derives every
// direction relative to the projected anchor line rather than to the sketch's
// own axes, so a case off the origin is not decoration.
var anchorCases = []proofkit.Case{
	{Name: "centre_on_sketch_origin", Params: map[string]float64{"centreX": 0, "centreY": 0}},
	{Name: "centre_off_sketch_origin", Params: map[string]float64{"centreX": 8, "centreY": -5}},
	{Name: "centre_far_off_sketch_origin", Params: map[string]float64{"centreX": -37, "centreY": 21}},
}

// latticeCases sweep the regime §2 has to hold across.
//
//   - the whole admitted Shaft Angle range, at both ends and through the default,
//     because every seed in §2 is a function of Sigma and the figure is not
//     symmetric in it;
//   - both directions of asymmetry, since EITHER gear can carry the smaller pitch
//     diameter and so be the binding side of the Maximum Face Width;
//   - each toe input alone AND the two together, which is the coupling neither
//     reaches on its own: at Toe Extension 0 a user Toe Radius only moves N and P
//     along a toe line of the resolved Face Width, while a defaulted Toe Radius
//     makes the Toe Limit the one radius that reproduces Toe Extension 0 exactly;
//   - Tooth Spacing above zero, which is the only setting that builds K' and L'
//     at all and so the only one where the lattice is 22 points rather than 20;
//   - the two virtual-tooth-count cases, 16/12 and 4/4, which the tooth profile
//     and every body built on it read.
//
// Shaft Angle 30 and Shaft Angle 150 stay in this table as DECLARED REFUSALS:
// the flag is set, the step reads it, and the engine's verdict on them is
// required rather than avoided. See assertDeclaredRefusal.
var latticeCases = []proofkit.Case{
	{Name: "default_31_31_sigma90", Params: sketchCase()},
	{Name: "sigma30_declared_refusal", Params: sketchCase(shaft(30), with(keyLatticeRefused, 1))},
	{Name: "sigma35", Params: sketchCase(shaft(35))},
	{Name: "sigma60", Params: sketchCase(shaft(60))},
	{Name: "sigma120", Params: sketchCase(shaft(120))},
	{Name: "sigma142", Params: sketchCase(shaft(142))},
	{Name: "sigma150_declared_refusal", Params: sketchCase(shaft(150), with(keyLatticeRefused, 1))},
	{Name: "driving31_pinion17", Params: sketchCase(pair(1, 31, 17))},
	{Name: "driving17_pinion31", Params: sketchCase(pair(1, 17, 31))},
	{Name: "driving43_pinion31_sigma75_spacing", Params: sketchCase(
		pair(4, 43, 31), shaft(75), with(keyToothSpacing, 0.5))},
	{Name: "virtual_count_16_12", Params: sketchCase(pair(4, 16, 12))},
	{Name: "virtual_count_4_4", Params: sketchCase(pair(4, 4, 4))},
	{Name: "toe_extension_50", Params: sketchCase(with(keyToeExtension, 50))},
	{Name: "toe_extension_100", Params: sketchCase(with(keyToeExtension, 100))},
	{Name: "toe_radius_user", Params: sketchCase(
		with(keyDrivingToeRadius, 3), with(keyPinionToeRadius, 3))},
	{Name: "toe_extension_50_toe_radius_user", Params: sketchCase(
		with(keyToeExtension, 50), with(keyDrivingToeRadius, 3), with(keyPinionToeRadius, 3))},
	{Name: "tooth_spacing_positive", Params: sketchCase(with(keyToothSpacing, 0.4))},
	{Name: "base_heights_user", Params: sketchCase(
		with(keyDrivingBase, 3), with(keyPinionBase, 3))},
	{Name: "face_width_user", Params: sketchCase(with(keyFaceWidth, 4))},
	{Name: "bore_user", Params: sketchCase(with(keyDrivingBore, 5), with(keyPinionBore, 5))},
	{Name: "bore_disabled", Params: sketchCase(with(keyBoreEnable, 0))},
}

// perGearCases run each of the cases above on both members, because every §3 and
// Profile-sketch quantity is per gear and the two members of an unequal pair are
// different figures.
func perGearCases(base []proofkit.Case) []proofkit.Case {
	out := make([]proofkit.Case, 0, 2*len(base))
	for _, c := range base {
		for _, side := range []struct {
			name string
			gear float64
		}{{"pinion", 0}, {"driving", 1}} {
			params := map[string]float64{keyGearSide: side.gear}
			for k, v := range c.Params {
				params[k] = v
			}
			out = append(out, proofkit.Case{Name: c.Name + "_" + side.name, Params: params})
		}
	}
	return out
}

var (
	toothCases   = perGearCases(latticeCases)
	profileCases = perGearCases(latticeCases)
	boreCases    = perGearCases(latticeCases)
	traceCases   = perGearCases(spiralSketchCases)
)

// spiralSketchCases are the lattice cases with a spiral angle set, plus the ends
// of the [0, 60) range the dialog admits and both hands. The trace is built only
// when psi > 0, so the psi = 0 straight bevel is not a case here: the hook
// returns before any of this construction runs, which is what the straight solid
// steps assert instead.
var spiralSketchCases = []proofkit.Case{
	{Name: "psi35_right_default", Params: sketchCase()},
	{Name: "psi35_left", Params: sketchCase(with(keyHand, -1))},
	{Name: "psi_just_above_zero", Params: sketchCase(with(keySpiralAngle, 0.5))},
	{Name: "psi55_near_range_top", Params: sketchCase(with(keySpiralAngle, 55))},
	{Name: "psi35_cutter_radius_user", Params: sketchCase(with(keyCutterRadius, 12))},
	{Name: "psi35_ratio_pair", Params: sketchCase(pair(4, 43, 31), shaft(75))},
	{Name: "psi35_sigma142", Params: sketchCase(shaft(142))},
}

// ---------------------------------------------------------------- §1 Anchor

// stepAnchorSketch builds the Anchor sketch: the projected Center Point, and one
// reference line through it whose direction is pinned sketch-locally.
//
// The line is seeded at exactly ±0.5 cm from the projected centre along the
// sketch-local X, so the seeded length is 10 mm, and the dimension locks it
// there; the value is arbitrary because nothing downstream reads it. The
// direction is pinned with a sketch-local Horizontal ([PB-REFLINE-DIRECTION]), not
// a world-axis lock, so it survives a tilted target plane.
//
// SUBSTITUTION. §1 applies BOTH addCoincident(projectedCenter, anchorLine) and
// addMidPoint(projectedCenter, anchorLine), and says to use both rather than the
// midpoint alone. The engine's NewMidpoint is a two-row constraint that already
// places the point ON the line, so adding NewPointOnLine beside it asserts a row
// the midpoint has already asserted and the engine reports it redundant. The
// proof therefore writes the midpoint alone ([PB-NO-OVERCONSTRAIN]). The cost is
// that Fusion's own arity at this site is not the arity solved here — whether
// Fusion absorbs its point-on-line row is a fact about Fusion's solver, and
// nothing in this repository runs it.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user's Center Point")
	centre := s.CreateReferencePoint(p["centreX"], p["centreY"], anchorProjectionSource)

	proofkit.Step(t, "the Anchor Line, seeded at ±0.5 cm from the projected centre")
	start := s.CreatePoint(p["centreX"]-anchorSeed, p["centreY"])
	end := s.CreatePoint(p["centreX"]+anchorSeed, p["centreY"])
	line := s.CreateLine(start, end)
	line.SetConstruction(true)

	proofkit.Step(t, "midpoint, length and a sketch-local direction")
	s.AddConstraint(
		sketch.NewMidpoint(centre, line),
		// SUBSTITUTION. Fusion's aligned distance dimension is one UNSIGNED row
		// whose direction is captured from the seeded geometry when it is created
		// ([PB-DIM-VALUE-SEMANTICS]), so the line's two end-for-end orientations
		// both satisfy it and the seed picks one. The engine's signed component is
		// one row too, so the arity is unchanged, and it carries the orientation
		// the unsigned form leaves to the seed — which the ambiguity probe
		// otherwise reports, correctly, as two configurations.
		sketch.NewHorizontalDistance(start, end, 2*anchorSeed),
		sketch.NewHorizontal(line),
	)

	proofkit.Step(t, "the same line with its direction left free")
	loose := proofkit.NewSketch(t)
	looseCentre := loose.CreateReferencePoint(p["centreX"], p["centreY"], anchorProjectionSource)
	looseStart := loose.CreatePoint(p["centreX"]-anchorSeed, p["centreY"])
	looseEnd := loose.CreatePoint(p["centreX"]+anchorSeed, p["centreY"])
	looseLine := loose.CreateLine(looseStart, looseEnd)
	loose.AddConstraint(
		sketch.NewMidpoint(looseCentre, looseLine),
		sketch.NewHorizontalDistance(looseStart, looseEnd, 2*anchorSeed),
	)
	if dof := solvedDOF(t, loose); dof != 1 {
		t.Errorf("the Anchor Line without its direction constraint has DOF %d, want the one free "+
			"rotation [PB-REFLINE-DIRECTION] exists to remove", dof)
	}
}

// solvedDOF solves a scratch sketch and reports the degrees of freedom the engine
// finds left in it.
func solvedDOF(t testing.TB, s *sketch.Sketch) int {
	t.Helper()
	ctx := context.Background()
	if _, err := s.Solve(ctx); err != nil {
		t.Fatalf("solve scratch sketch: %v", err)
	}
	report := s.Verify(ctx)
	if !report.Analysed() {
		t.Fatal("scratch sketch was not analysed, so its DOF reading means nothing")
	}
	return report.DOF
}

// ---------------------------------------------------------------- §2 lattice

// latticeBuild is the §2 figure as the engine holds it: the named points, so the
// end-of-section assertion can read each one's solved position back by name.
type latticeBuild struct {
	l      lattice
	points map[string]*sketch.Point
}

// signedOffset is the value a NewOffset from the line start->end to a target
// point carries: positive to the LEFT of the source direction, which is the
// engine's own convention. §2's offset dimensions are unsigned in Fusion and the
// seed is what picks their side; here the side is carried by this number, which
// is what [BEVEL-F-MIRROR-FIGURE] asks the proof to do.
func signedOffset(start, end, target pt2) float64 {
	return end.sub(start).unit().cross(target.sub(start))
}

// signedAlong pins p2 relative to p1 by the larger component of the unit
// direction d, signed. It stands in for a Fusion ALIGNED length dimension, which
// is one unsigned row: the engine's signed component is one row too, so the arity
// is unchanged and the side is no longer left to the seed.
func signedAlong(p1, p2 *sketch.Point, d pt2, length float64) sketch.Constraint {
	if math.Abs(d.X) >= math.Abs(d.Y) {
		return sketch.NewHorizontalDistance(p1, p2, length*d.X)
	}
	return sketch.NewVerticalDistance(p1, p2, length*d.Y)
}

// stepGearProfiles builds the whole §2 Gear Profiles lattice and holds it to
// [BEVEL-F-SEED-HELD]: every named point's SOLVED position against the
// closed-form position §2 seeds it at, in the sketch's own 2-D frame with no
// world round-trip, compared in creation order so the first site that moved is
// the one named.
//
// Every line is built in the COINCIDENT style ([BEVEL-F-COINCIDENT-STYLE]): from
// raw coordinates, with one coincident per endpoint that already exists. No line
// shares an existing point, and no segment carries two lines
// ([BEVEL-F-LINE-ONCE]).
//
// THE SEED IS WHAT THIS CANNOT REACH. Every point below is seeded at its closed
// form, which is the rule §2 states, so what passes here is that the constraints
// SOLVE from a correct seed. It is not that the generated module seeds them
// correctly — the toe line is the site where that difference has already shipped
// a defect to Fusion, and the [BEVEL-F-SEED-HELD] gate inside the module is the
// only thing that catches it.
//
// FOUR CONSTRAINTS FUSION NEEDS ARE OMITTED HERE, each because the engine's
// counterpart carries rows Fusion's does not:
//
//   - The two base-height perpendiculars, E->G ⊥ H->G and F->I ⊥ J->I. Fusion's
//     addOffsetDimension requires the two lines to be parallel already and
//     controls only the distance, so the perpendicular is what supplies the
//     parallelism. The engine's NewOffset holds BOTH endpoints of the target at
//     the same signed distance, so it carries the parallelism itself; adding the
//     perpendicular is a third row for two freedoms and the lattice comes back
//     overconstrained at DOF 0 with the two base-height offsets named redundant.
//   - The two toe-line parallels, addParallel(M->N, C->H) and addParallel(O->P,
//     D->J), omitted for exactly the same reason and at the same arity.
//
// [PB-COLLINEAR-CHAIN] is the fifth: a collinear is two point-on-line rows in
// both engines, one of which the endpoint coincidence has already asserted, so
// every §2 collinear is written here as the single NewPointOnLine row that is not
// implied. Which of the two readings Fusion absorbs is a Fusion fact no proof can
// reach; the playbook records that measurement.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	if p[keyLatticeRefused] >= 0.5 {
		assertDeclaredRefusal(t, p)
	}
	b := buildLattice(t, s, p)

	proofkit.Step(t, "solve and compare every named point against its own seed")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the Gear Profiles lattice: %v", err)
	}
	assertSeedsHeld(t, b)
	assertResolvedBounds(t, b)
}

// buildLattice draws §2 and returns the named points. It is separated from the
// step so the Profile-sketch and tooth steps can rebuild the same figure without
// restating it.
func buildLattice(t testing.TB, s *sketch.Sketch, p map[string]float64) latticeBuild {
	t.Helper()
	l := deriveLattice(t, p)
	seed := latticePoints(l)
	pts := map[string]*sketch.Point{}

	// A line from raw coordinates, by the coincident style: nothing is shared.
	at := func(name string) pt2 { return seed[name] }
	line := func(a, bv pt2) *sketch.Line {
		ln := s.CreateLine(s.CreatePoint(a.X, a.Y), s.CreatePoint(bv.X, bv.Y))
		ln.SetConstruction(true)
		return ln
	}

	proofkit.Step(t, "project the Anchor sketch's centre point and its Anchor Line")
	centre := s.CreateReferencePoint(0, 0, anchorProjectionSource)
	anchorEnd := s.CreateReferencePoint(anchorSeed, 0, anchorProjectionSource)
	anchorLine, err := s.CreateReferenceLine(centre, anchorEnd, anchorLineSource)
	if err != nil {
		t.Fatalf("project the Anchor Line: %v", err)
	}

	proofkit.Step(t, "centre -> Apex, perpendicular to the projected anchor line")
	centreToApex := line(pt2{0, 0}, at("Apex"))
	apex := centreToApex.End
	pts["Apex"] = apex
	s.AddConstraint(
		sketch.NewCoincident(centreToApex.Start, centre),
		sketch.NewPerpendicular(centreToApex, anchorLine),
	)

	proofkit.Step(t, "the two shaft axes")
	drivingShaft := line(at("Apex"), at("B"))
	pts["B"] = drivingShaft.End
	s.AddConstraint(
		sketch.NewCoincident(drivingShaft.Start, apex),
		sketch.NewParallel(drivingShaft, centreToApex),
	)
	pinionShaft := line(at("Apex"), at("A"))
	pts["A"] = pinionShaft.End
	s.AddConstraint(
		sketch.NewCoincident(pinionShaft.Start, apex),
		// Signed: counter-clockwise from the driving shaft's direction to the
		// pinion's is +Sigma, which is the side the +X-most seed candidate picks.
		sketch.NewAngle(drivingShaft, pinionShaft, degrees(l.sigma)),
	)

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	dropPinion := line(at("A"), at("Apex2"))
	dropDriving := line(at("B"), at("Apex2"))
	pts["Apex2"] = dropPinion.End
	s.AddConstraint(
		sketch.NewCoincident(dropPinion.Start, pts["A"]),
		sketch.NewPerpendicular(dropPinion, pinionShaft),
		// SUBSTITUTION, and it is the one [BEVEL-F-MIRROR-FIGURE] is about. Fusion
		// writes an unsigned aligned length here, so the drop satisfies its
		// dimension on EITHER side of the shaft axis and only the seed picks the
		// interior wedge; if the two drops seed opposite sides the closure below
		// flips the whole figure to its mirror and nothing in the build refuses it.
		// The engine's signed component is the same single row with the side
		// carried in the number.
		signedAlong(dropPinion.Start, dropPinion.End,
			at("Apex2").sub(at("A")).unit(), l.pinion.radius),
		sketch.NewCoincident(dropDriving.Start, pts["B"]),
		sketch.NewPerpendicular(dropDriving, drivingShaft),
		signedAlong(dropDriving.Start, dropDriving.End,
			at("Apex2").sub(at("B")).unit(), l.driving.radius),
		sketch.NewCoincident(dropPinion.End, dropDriving.End),
	)

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	pitchLine := line(at("Apex"), at("Apex2"))
	s.AddConstraint(
		sketch.NewCoincident(pitchLine.Start, apex),
		sketch.NewCoincident(pitchLine.End, pts["Apex2"]),
	)
	dedPinion := line(at("Apex2"), at("C"))
	pts["C"] = dedPinion.End
	dedDriving := line(at("Apex2"), at("D"))
	pts["D"] = dedDriving.End
	s.AddConstraint(
		sketch.NewCoincident(dedPinion.Start, pts["Apex2"]),
		sketch.NewPerpendicular(dedPinion, pitchLine),
		// The two sites where "C collapses onto D" lives. The perpendicular fixes
		// the line's direction and the length its magnitude; neither picks a side,
		// so each end has two solutions and in Fusion the seed takes one. Flip the
		// pinion seed and C solves exactly onto D. The signed component below is
		// what carries that choice here.
		signedAlong(dedPinion.Start, dedPinion.End,
			at("C").sub(at("Apex2")).unit(), l.dedendum),
		sketch.NewCoincident(dedDriving.Start, pts["Apex2"]),
		sketch.NewPerpendicular(dedDriving, pitchLine),
		signedAlong(dedDriving.Start, dedDriving.End,
			at("D").sub(at("Apex2")).unit(), l.dedendum),
	)

	proofkit.Step(t, "the two root axes Apex->C and Apex->D")
	rootPinion := line(at("Apex"), at("C"))
	rootDriving := line(at("Apex"), at("D"))
	s.AddConstraint(
		sketch.NewCoincident(rootPinion.Start, apex),
		sketch.NewCoincident(rootPinion.End, pts["C"]),
		sketch.NewCoincident(rootDriving.Start, apex),
		sketch.NewCoincident(rootDriving.End, pts["D"]),
	)

	proofkit.Step(t, "the extension feet E and F, closed by their perpendiculars")
	extendE := line(at("A"), at("E"))
	pts["E"] = extendE.End
	s.AddConstraint(
		sketch.NewCoincident(extendE.Start, pts["A"]),
		sketch.NewPointOnLine(extendE.End, pinionShaft),
	)
	ce := line(at("C"), at("E"))
	s.AddConstraint(
		sketch.NewCoincident(ce.Start, pts["C"]),
		sketch.NewCoincident(ce.End, pts["E"]),
		sketch.NewPerpendicular(ce, extendE),
	)
	extendF := line(at("B"), at("F"))
	pts["F"] = extendF.End
	s.AddConstraint(
		sketch.NewCoincident(extendF.Start, pts["B"]),
		sketch.NewPointOnLine(extendF.End, drivingShaft),
	)
	df := line(at("D"), at("F"))
	s.AddConstraint(
		sketch.NewCoincident(df.Start, pts["D"]),
		sketch.NewCoincident(df.End, pts["F"]),
		sketch.NewPerpendicular(df, extendF),
	)

	proofkit.Step(t, "the two heel edges and their signed base-height offsets")
	extendG := line(at("E"), at("G"))
	pts["G"] = extendG.End
	s.AddConstraint(
		sketch.NewCoincident(extendG.Start, pts["E"]),
		// The collinear names A->E, never the Apex->A axis further up the chain
		// ([BEVEL-F-COLLINEAR-CHAIN]); the row the endpoint coincidence has not
		// already asserted is this one.
		sketch.NewPointOnLine(extendG.End, extendE),
	)
	extendH := line(at("C"), at("H"))
	pts["H"] = extendH.End
	s.AddConstraint(
		sketch.NewCoincident(extendH.Start, pts["C"]),
		sketch.NewPointOnLine(extendH.End, dedPinion),
	)
	heelPinion := line(at("G"), at("H"))
	s.AddConstraint(
		sketch.NewCoincident(heelPinion.Start, pts["G"]),
		sketch.NewCoincident(heelPinion.End, pts["H"]),
		// No perpendicular here: see the file comment. The offset below carries
		// the parallelism, and its value is signed, so the side Fusion leaves to
		// the G and H seeds is pinned by this number.
		sketch.NewOffset(dropPinion, heelPinion,
			signedOffset(at("A"), at("Apex2"), at("G"))),
	)
	extendI := line(at("F"), at("I"))
	pts["I"] = extendI.End
	s.AddConstraint(
		sketch.NewCoincident(extendI.Start, pts["F"]),
		sketch.NewPointOnLine(extendI.End, extendF),
	)
	extendJ := line(at("D"), at("J"))
	pts["J"] = extendJ.End
	s.AddConstraint(
		sketch.NewCoincident(extendJ.Start, pts["D"]),
		sketch.NewPointOnLine(extendJ.End, dedDriving),
	)
	heelDriving := line(at("I"), at("J"))
	s.AddConstraint(
		sketch.NewCoincident(heelDriving.Start, pts["I"]),
		sketch.NewCoincident(heelDriving.End, pts["J"]),
		sketch.NewOffset(dropDriving, heelDriving,
			signedOffset(at("B"), at("Apex2"), at("J"))),
	)

	proofkit.Step(t, "constrain point I with the projected centre")
	// SUBSTITUTION. Fusion writes addCoincident(I, projected centre), which is two
	// rows. I already sits on the centre->Apex line — the driving shaft is
	// parallel to it and shares the Apex, so it IS that line — so one of the two
	// rows is already asserted and the engine reports it redundant. The row that
	// is not implied is this one, and with it I solves onto the centre exactly.
	s.AddConstraint(sketch.NewPointOnLine(pts["I"], anchorLine))

	proofkit.Step(t, "the tooth centres K and L, and K' / L' when Tooth Spacing is above zero")
	dedPinionDir := at("C").sub(at("Apex2")).unit()
	dedDrivingDir := at("D").sub(at("Apex2")).unit()
	buildCentre(t, s, l, at, pts, line, "K", "K'", pinionShaft, dedPinion, dedPinionDir, "G", "C")
	buildCentre(t, s, l, at, pts, line, "L", "L'", drivingShaft, dedDriving, dedDrivingDir, "I", "D")

	proofkit.Step(t, "the two toe lines and the two front faces")
	buildToe(t, s, at, pts, line, l.pinion, toeNames{"M", "N", "A'", "C", "H", "G"},
		rootPinion, extendH, pinionShaft)
	buildToe(t, s, at, pts, line, l.driving, toeNames{"O", "P", "B'", "D", "J", "I"},
		rootDriving, extendJ, drivingShaft)

	return latticeBuild{l: l, points: pts}
}

// buildCentre draws one gear's tooth-centre chain: the G->K (resp. I->L) line
// that creates the centre, the C->K (resp. D->L) reference line §3 takes as its
// anchor, and — only when Tooth Spacing is above zero — the K->K' line that
// offsets the centre outward along the dedendum line.
//
// K is pinned with two point-on-line rows and no collinear: by the time it is
// added G and C are both fixed, so a collinear over-determines
// ([BEVEL-F-COLLINEAR-CHAIN]).
//
// SUBSTITUTION at K->K'. Fusion writes an unsigned aligned length dimension of
// Tooth Spacing, and the point-on-line plus that length admit K' one Tooth
// Spacing on the C side of K just as readily — two candidates 2x Tooth Spacing
// apart, which is 0.8 mm at the table's 0.4 mm spacing. The engine's NewDistance
// is unsigned in the same way, so the proof writes the SIGNED component of the
// same one row instead. The sign is the whole of what is being carried:
// K' - K projected on Apex2->C is +Tooth Spacing and never -Tooth Spacing.
func buildCentre(t testing.TB, s *sketch.Sketch, l lattice, at func(string) pt2,
	pts map[string]*sketch.Point, line func(a, b pt2) *sketch.Line,
	centreName, offsetName string, shaft, dedendum *sketch.Line, dedDir pt2,
	fromHeel, fromCorner string) {
	t.Helper()
	reach := line(at(fromHeel), at(centreName))
	pts[centreName] = reach.End
	s.AddConstraint(
		sketch.NewCoincident(reach.Start, pts[fromHeel]),
		sketch.NewPointOnLine(reach.End, shaft),
		sketch.NewPointOnLine(reach.End, dedendum),
	)
	anchorName := centreName
	if l.toothSpacing > 0 {
		spacing := line(at(centreName), at(offsetName))
		pts[offsetName] = spacing.End
		s.AddConstraint(
			sketch.NewCoincident(spacing.Start, pts[centreName]),
			sketch.NewPointOnLine(spacing.End, dedendum),
			signedAlong(pts[centreName], spacing.End, dedDir, l.toothSpacing),
		)
		anchorName = offsetName
	}
	reference := line(at(fromCorner), at(anchorName))
	s.AddConstraint(
		sketch.NewCoincident(reference.Start, pts[fromCorner]),
		sketch.NewCoincident(reference.End, pts[anchorName]),
	)
}

// buildToe draws one gear's toe lattice: the shaft-axis edge that creates the
// front face's foot, the toe line, the front face that holds the toe corner off
// the axis at the resolved Toe Radius, and the M->C (resp. O->D) reference line.
//
// The toe corner is NEVER pinned to the shaft axis. Only the foot touches it, and
// the Toe Radius is strictly positive, which is what keeps the revolved hexagon
// off its own axis of revolution.
//
// SUBSTITUTION at the toe line. Fusion writes addParallel(M->N, C->H) and then an
// unsigned addOffsetDimension. The engine's NewOffset holds both endpoints of the
// target at one signed distance, so it carries the parallelism itself and its
// value carries the side; writing the parallel beside it is a third row for two
// freedoms. The front face's length dimension is written as its signed component
// for the reason buildCentre gives: the toe line meets the Toe Radius on BOTH
// sides of the shaft axis, and a length is unsigned.
func buildToe(t testing.TB, s *sketch.Sketch, at func(string) pt2,
	pts map[string]*sketch.Point, line func(a, b pt2) *sketch.Line, g member,
	n toeNames, rootAxis, heelEdge, shaft *sketch.Line) {
	t.Helper()
	shaftEdge := line(at(n.foot), at(n.heelFoot))
	pts[n.foot] = shaftEdge.Start
	s.AddConstraint(sketch.NewCoincident(shaftEdge.End, pts[n.heelFoot]))

	toe := line(at(n.toe), at(n.corner))
	pts[n.toe], pts[n.corner] = toe.Start, toe.End
	s.AddConstraint(
		sketch.NewPointOnLine(toe.Start, rootAxis),
		sketch.NewOffset(heelEdge, toe,
			signedOffset(at(n.dedCorner), at(n.heelEnd), at(n.toe))),
	)

	front := line(at(n.corner), at(n.foot))
	radial := at(n.corner).sub(at(n.foot)).unit()
	s.AddConstraint(
		sketch.NewCoincident(front.Start, pts[n.corner]),
		sketch.NewCoincident(front.End, pts[n.foot]),
		sketch.NewPointOnLine(pts[n.foot], shaft),
		sketch.NewPerpendicular(front, shaft),
		signedAlong(pts[n.foot], pts[n.corner], radial, g.toeRadius),
	)

	connector := line(at(n.toe), at(n.dedCorner))
	s.AddConstraint(
		sketch.NewCoincident(connector.Start, pts[n.toe]),
		sketch.NewCoincident(connector.End, pts[n.dedCorner]),
	)
}

// toeNames is one gear's toe lattice by §2's own point names: M/N/A' on the
// pinion and O/P/B' on the driving gear, with the dedendum corner C/D, the heel
// end H/J the toe line offsets from, and the heel foot G/I the shaft-axis edge
// runs to.
type toeNames struct {
	toe, corner, foot, dedCorner, heelEnd, heelFoot string
}

// assertSeedsHeld is the proof's copy of the [BEVEL-F-SEED-HELD] gate: every
// named point's solved geometry against the closed form §2 seeded it at, in
// creation order, at the gate's own 0.001 mm tolerance.
//
// The list is 22 points only when Tooth Spacing is above zero. At Tooth Spacing 0
// K' and L' are not built at all, so it is 20; comparing a K' that was never
// created is the one way this gate can raise on a correct figure.
func assertSeedsHeld(t testing.TB, b latticeBuild) {
	t.Helper()
	seed := latticePoints(b.l)
	order := latticeOrder(b.l)
	if want := 20 + 2*btoi(b.l.toothSpacing > 0); len(order) != want {
		t.Fatalf("the gate compares %d points, want %d", len(order), want)
	}
	for _, name := range order {
		q, ok := b.points[name]
		if !ok {
			t.Fatalf("§2 never created point %s, so the gate has nothing to compare", name)
		}
		got := pt2{q.X(), q.Y()}
		if d := got.sub(seed[name]).length(); d > 1e-3 {
			t.Fatalf("point %s solved to (%.6f, %.6f), seeded at (%.6f, %.6f) — %.6f mm apart, "+
				"above the 0.001 mm [BEVEL-F-SEED-HELD] tolerance",
				name, got.X, got.Y, seed[name].X, seed[name].Y, d)
		}
	}
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}

// assertResolvedBounds holds §2's own resolve step to the solved figure rather
// than to the closed form that seeded it: the Maximum Face Width is the
// perpendicular distance from A to the Pinion Dedendum line C->H and from B to
// D->J, taken on the SOLVED points, and each gear's resolved bore is inside its
// own Maximum Bore Diameter.
//
// Reading it from the solved geometry is the point ([PB-SOLVED-GEOMETRY]): the
// seeds diverge from the solve for asymmetric tooth counts, and a seed-based
// bound is too loose on the binding side.
func assertResolvedBounds(t testing.TB, b latticeBuild) {
	t.Helper()
	solved := func(name string) pt2 {
		q := b.points[name]
		return pt2{q.X(), q.Y()}
	}
	perp := func(from, lineA, lineB string) float64 {
		d := solved(lineB).sub(solved(lineA)).unit()
		return math.Abs(d.cross(solved(from).sub(solved(lineA))))
	}
	want := guardFactor * math.Min(perp("A", "C", "H"), perp("B", "D", "J"))
	if d := math.Abs(want - b.l.maxFaceWidth); d > 1e-6 {
		t.Errorf("the Maximum Face Width read off the solved lattice is %.9f mm, the closed form "+
			"gives %.9f mm — %.9f mm apart", want, b.l.maxFaceWidth, d)
	}
	if b.l.faceWidth > b.l.maxFaceWidth+1e-12 {
		t.Errorf("the resolved Face Width %.6f mm is above the maximum %.6f mm",
			b.l.faceWidth, b.l.maxFaceWidth)
	}
	for _, g := range []member{b.l.pinion, b.l.driving} {
		if g.bore > g.maxBore+1e-12 {
			t.Errorf("%s resolved Bore Diameter %.6f mm is above the maximum %.6f mm",
				g.label, g.bore, g.maxBore)
		}
		if g.baseHeight < g.minBaseHeight-1e-12 || g.baseHeight > g.maxBaseHeight+1e-12 {
			t.Errorf("%s resolved Base Height %.6f mm is outside [%.6f, %.6f]",
				g.label, g.baseHeight, g.minBaseHeight, g.maxBaseHeight)
		}
	}
	// The Tooth Spacing sign, standing in for the signed constraint the engine has
	// no shape for at this site: K' - K projected on Apex2->C is +Tooth Spacing.
	if b.l.toothSpacing > 0 {
		for _, pair := range [][3]string{{"K", "K'", "C"}, {"L", "L'", "D"}} {
			d := solved(pair[2]).sub(solved("Apex2")).unit()
			got := solved(pair[1]).sub(solved(pair[0])).dot(d)
			if math.Abs(got-b.l.toothSpacing) > 1e-6 {
				t.Errorf("%s - %s projected on Apex2->%s is %+.6f mm, want +%.6f mm — the flipped "+
					"twin sits one Tooth Spacing on the %s side",
					pair[1], pair[0], pair[2], got, b.l.toothSpacing, pair[2])
			}
		}
	}
}

// assertDeclaredRefusal is what a case flagged latticeRefused gets instead of the
// positive gate: the net is built on a scratch sketch, solved and verified, and
// the engine's verdict on it is REQUIRED to be exactly the refusal the table
// declares — fully constrained, DOF 0, and near-singularity as the sole reason.
// A case that starts passing, or starts failing some other way, fails here rather
// than passing quietly.
//
// Both ends of the Shaft Angle range approach the engine's 4e-5 trust floor and
// this net falls under it at both: 2.93e-5 at Shaft Angle 30 and 3.99e-5 at 150,
// against 4.37e-5 at 35, 9.44e-5 at 142 and 3.34e-4 at the default 90. That is a
// fact about THIS construction and not a bound on the spec. Three independently
// written lattices, each holding DOF 0 with nothing redundant and each asserting
// its cone angles against the closed form, do not agree about which end is
// reachable: two refuse 30 and pass 150, one does the opposite. So the spec states
// the geometric range and the proof records what its own net reaches. The remedy
// for a refusal is to change how the lattice is built; it is never to loosen the
// gate and never to narrow the advertised range on one net's evidence.
//
// Shaft Angle 150 sits 0.3% under the floor, the thinnest margin in the table and
// the one most likely to move if the net is rebuilt.
func assertDeclaredRefusal(t testing.TB, p map[string]float64) {
	t.Helper()
	scratch := proofkit.NewSketch(t)
	buildLattice(t, scratch, p)
	ctx := context.Background()
	if _, err := scratch.Solve(ctx); err != nil {
		t.Fatalf("a declared refusal must still SOLVE; this one did not: %v", err)
	}
	report := scratch.Verify(ctx, sketch.WithProbe())
	if !report.Analysed() {
		t.Fatal("the declared refusal was not analysed, so its verdict means nothing")
	}
	if report.Status != sketch.FullyConstrained || report.DOF != 0 {
		t.Fatalf("declared refusal solved to status %s at DOF %d, want fully constrained at DOF 0 "+
			"— the refusal is conditioning alone, not a hole in the net",
			report.Status, report.DOF)
	}
	failed := report.Check()
	if failed == nil {
		t.Fatalf("the declared refusal PASSED at conditioning %.4e; this net now reaches a "+
			"configuration the table says it does not, so the declaration is stale",
			report.Conditioning)
	}
	reasons := failed.Unwrap()
	if len(reasons) != 1 || !errors.Is(reasons[0], sketch.ErrNearSingular) {
		t.Fatalf("declared refusal failed for %v, want near-singularity alone", reasons)
	}
	proofkit.Unmodelled(t, "declared refusal: this net's conditioning reads %.4e, below the "+
		"engine's 4e-5 trust floor, at a configuration the spec admits",
		report.Conditioning)
}

// ---------------------------------------------------------------- §3 tooth

// toothDimensions is the four circle radii the virtual-spur proxy serves, built
// from the EXACT virtual tooth count and with the root circle sunk one root sink
// inside the dedendum corner.
func toothDimensions(l lattice, g member) (involute.Dimensions, bool) {
	d := involute.Derive(l.module, g.virtualTeeth, toothPressureAngle)
	d.Root -= l.rootSink
	return d, d.Embedded()
}

// stepToothProfile draws one gear's `{gearLabel} Tooth` sketch: the borrowed spur
// tooth, at this gear's EXACT virtual tooth number, already rotated 180 degrees
// by the drawer's own angle argument, anchored on the tooth-centre point K'/L'.
//
// The virtual tooth number is a real number and is never rounded. The drawer
// reads it in one place, the angular half-thickness pi/(2*z_v), and with
// z_v = 2*r_v/Module that angle gives a tooth thickness of pi*Module/2 at the
// pitch circle — the standard thickness. An integer count drawn at the exact
// radius gives pi*r_v/round(z_v) instead, which misses nominal by a different
// amount on each member of an unequal pair.
//
// The root circle is drawn one root sink inside the dedendum corner. At the
// corner exactly, the tooth's root arc touches the gear body's root cone only
// where the arc crosses the tooth's own centreline, and its two corners stand
// outside it; the sink pushes the whole arc inside so the Combine-Join meets the
// body across the root. It is one figure, not a proof-only offset: the generated
// module passes the same value to the proxy as rootSink_mm.
//
// THE TOOTH-TOP ARC IS A CENTRE-POINT ARC WITH A PINNED CENTRE AND NO DIMENSION.
// §3 hands the sketch to the shared spur drawer, which creates it with
// addByCenterStartEnd(localOrigin, rightFlankEnd, leftFlankEnd) and then pins the
// COPIED centre with addCoincident(arc.centerSketchPoint, localOrigin)
// ([SPUR-F-TOOTHTOP-ARC]). That coincident is written here as the drawer writes
// it, and the engine solves with it, so the cost at this site is nil. It is NOT a
// three-point arc with a radius dimension, and the §3a trace arc below — which
// genuinely is one — must not be read from the same template.
func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	d, embedded := toothDimensions(l, g)
	angle := math.Pi // the drawer's own draw(anchorPoint, angle=radians(180))

	proofkit.Step(t, "%s: virtual tooth number %.6f, root sunk %.6f mm",
		g.label, g.virtualTeeth, l.rootSink)
	origin := s.CreatePoint(0, 0)

	proofkit.Step(t, "the four circles")
	mk := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	mk(d.Root, false)
	tip := mk(d.Tip, true)
	mk(d.Base, true)
	mk(d.Pitch, true)

	proofkit.Step(t, "involute flanks")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, g.virtualTeeth, toothInvoluteSteps, angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	proofkit.Step(t, "tooth-top arc, centre pinned and undimensioned")
	topX, topY := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))

	proofkit.Step(t, "spine and the confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, degrees(angle)))

	proofkit.Step(t, "ribs")
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prev := origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		tt := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		mx, my := tt*math.Cos(angle), tt*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, my-prevY))
		}
		prev, prevX, prevY = mid, mx, my
	}

	proofkit.Step(t, "flank-to-root lines, drawn only when the flank starts outside the root")
	if !embedded {
		foot := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := d.Root*seed.X/n, d.Root*seed.Y/n
			re := s.CreatePoint(rx, ry)
			s.CreateLine(re, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, re, rx),
				sketch.NewVerticalDistance(origin, re, ry),
			)
		}
		foot(leftPts[0], left[0])
		foot(rightPts[0], right[0])
	}

	proofkit.Step(t, "anchor the tooth on the tooth-centre point")
	// The anchor is Tooth Spacing off the tooth plane's own crossing of the shaft
	// axis, which is exactly what the K -> K' offset does to the centre: K' rides
	// the dedendum line, and that line lies in this plane. At Tooth Spacing 0 the
	// two coincide and nothing is dragged.
	anchor := s.CreateReferencePoint(l.toothSpacing, 0, anchorProjectionSource)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the loop the tooth-profile selection keys on")
	assertToothLoops(t, s, d, embedded)
}

// assertToothLoops holds the drawn sketch to the curve counts
// find_profile_by_curve_counts selects the tooth loop by: 2 NURBS, 2 arcs, and
// exactly 2 lines or exactly 0, DETERMINED by the embedded flag and never
// accepted either way. An unrelated loop between the drawn circles can carry the
// same NURBS and arc counts with the other line count, and selecting it makes the
// apex loft fail with LOFT_NO_TOOLBODY.
func assertToothLoops(t testing.TB, s *sketch.Sketch, d involute.Dimensions, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before reading profiles: %v", err)
	}
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	tooth, disc := 0, 0
	regions := s.Profiles()
	for _, profile := range regions {
		nurbs, arcs, lines := entityCounts(profile.Entities)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth++
		case nurbs == 0 && arcs == 1 && lines == 0:
			disc++
			if want := math.Pi * d.Root * d.Root; math.Abs(profile.Area-want) > 1e-6*want {
				t.Errorf("disc region area %.6f mm2, want the sunk root circle's %.6f mm2",
					profile.Area, want)
			}
		default:
			t.Errorf("unexpected region: %d NURBS, %d arcs, %d lines", nurbs, arcs, lines)
		}
		if !profile.Valid {
			t.Errorf("region with %d NURBS, %d arcs, %d lines is not extrudable", nurbs, arcs, lines)
		}
	}
	if tooth != 1 {
		t.Errorf("tooth regions of 2 NURBS, 2 arcs, %d lines: %d, want 1", wantLines, tooth)
	}
	if disc != 1 {
		t.Errorf("disc regions inside the root circle: %d, want 1", disc)
	}
}

// entityCounts classifies a region's DISTINCT boundary entities the way
// find_profile_by_curve_counts classifies Fusion's profile curves: a fitted
// spline is a NURBS, the tooth-top arc and the root circle are each an arc, and a
// flank-to-root stub is a line.
func entityCounts(entities []sketch.Entity) (nurbs, arcs, lines int) {
	for _, entity := range entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}

// ------------------------------------------------------- per-gear Profile sketch

// stepProfileSketch builds one gear's `{gearLabel} Profile` sketch: the six §2
// vertices recreated as NEW points at their exact positions, the closed hexagon
// drawn in the table's walk order SHARING those points, and the endpoints fixed
// AFTER the lines exist.
//
// The order is load-bearing ([PB-PROJECT-NOT-FIXED]): fixing a bare point before
// it is consumed as a line endpoint does not leave the sketch fully constrained.
// Recreating rather than projecting is what makes the hexagon's first edge carry
// a trustworthy world position, which every body operation below reads
// ([PB-WORLDGEO-CONSTRAINED]).
//
// The sketch holds exactly ONE hexagon loop, which is what lets the revolve take
// its single profile with no search ([PB-SINGLE-PROFILE]). Drawing both gears'
// hexagons in the shared §2 sketch would leave two identically-shaped loops.
func stepProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	hex := profileHexagon(l, g)

	proofkit.Step(t, "%s: recreate the six vertices A'/G/H/C/M/N in draw order", g.label)
	verts := make([]*sketch.Point, 0, 6)
	for _, v := range hex.points() {
		verts = append(verts, s.CreatePoint(v.X, v.Y))
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, 0, 6)
	for i := range verts {
		lines = append(lines, s.CreateLine(verts[i], verts[(i+1)%len(verts)]))
	}

	proofkit.Step(t, "fix the endpoints, AFTER the lines exist")
	for _, ln := range lines {
		s.Fix(ln.Start)
		s.Fix(ln.End)
	}

	proofkit.Step(t, "the one region the revolve consumes")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the Profile sketch: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("%s Profile holds %d regions, want the one hexagon loop", g.label, len(regions))
	}
	if want := polygonArea(hex.points()); math.Abs(regions[0].Area-want) > 1e-9*want {
		t.Errorf("%s Profile region area %.9f mm2, want %.9f mm2", g.label, regions[0].Area, want)
	}
	if !regions[0].Valid {
		t.Errorf("%s Profile region is not extrudable", g.label)
	}

	proofkit.Step(t, "the first edge is the shaft axis and the toe corner is off it")
	if math.Abs(hex.aPrime.Y) > 1e-12 || math.Abs(hex.g.Y) > 1e-12 {
		t.Errorf("%s first edge A'->G is %.12f / %.12f off the shaft axis; the revolve, the "+
			"pattern, the bore plane and the mesh rotation all take it as the axis",
			g.label, hex.aPrime.Y, hex.g.Y)
	}
	if hex.n.Y <= 0 {
		t.Errorf("%s toe corner N sits at radius %.9f mm; the Toe Radius is strictly positive and "+
			"N on the axis of revolution is what the later conical split fails on",
			g.label, hex.n.Y)
	}
	if hex.m.X >= hex.c.X {
		t.Errorf("%s toe M is at station %.6f, at or beyond the heel corner C at %.6f — the toe "+
			"has landed outside the heel and the revolved frustum is degenerate",
			g.label, hex.m.X, hex.c.X)
	}
}

// ---------------------------------------------------------------- Bore sketch

// stepBoreSketch builds one gear's `{gearLabel} Bore` sketch: the bore circle on
// the sketch origin, with its centre FIXED and a driving diameter dimension.
//
// The plane is rooted at the shaft start, so the origin is already on the axis.
// The centre is fixed rather than made coincident to the sketch's own origin
// point: a circle created at (0, 0) does not reuse that point, its centre is a
// free point that happens to sit there, and constraining it to the origin has
// been observed to throw VCS_SKETCH_SOLVING_FAILED ([PB-CIRCLE-CENTER]).
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	if g.bore <= 0 {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no Bore sketch is drawn at all")
	}

	proofkit.Step(t, "%s: bore circle of %.6f mm, centre fixed", g.label, g.bore)
	centre := s.CreatePoint(0, 0)
	circle := s.CreateCircle(centre, g.bore/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, g.bore))

	proofkit.Step(t, "the region the through-cut consumes")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the Bore sketch: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("%s Bore holds %d regions, want the one bore disc", g.label, len(regions))
	}
	want := math.Pi * g.bore * g.bore / 4
	if math.Abs(regions[0].Area-want) > 1e-9*want {
		t.Errorf("%s bore region area %.9f mm2, want %.9f mm2", g.label, regions[0].Area, want)
	}
	if !regions[0].Valid {
		t.Errorf("%s bore region is not extrudable", g.label)
	}
}

// ------------------------------------------------------- §3a 2-D cutter-arc trace

// stepTraceSketch builds the `{gear} 2D Tooth Trace` sketch: the cutter circle
// with its centre pinned and a diameter dimension, and the trace arc with its
// centre coincident to that centre and a radius dimension, so it is the genuine
// cutter circle and not a look-alike spline.
//
// THIS ARC REALLY IS A THREE-POINT ARC WITH A RADIUS DIMENSION, which is what
// separates it from the tooth-top arc of §3; the two must not be written from one
// template.
//
// SUBSTITUTION. Fusion draws it through three points — the toe end, the mean
// point on the cone element, and the heel end — and deliberately leaves the
// sketch with free degrees of freedom, because dimensioning the endpoints
// over-constrains the solve against the cone-element plane. proofkit gates on
// DOF 0 and waives nothing, so the two ends are pinned here by the construction
// §6 of the trace derivation states instead: the toe end lies on the apex circle
// of radius R_toe - 0.06*span and the heel end on the one of radius
// R_heel + 0.06*span. That is one row each, and it is the circle-circle
// intersection the framework helper solves rather than a coordinate dimension.
// What it costs is that the sketch proved here is fully constrained where
// Fusion's is not, so the free-DOF exemption [BEVEL-F-FULL-CONSTRAINT] grants
// this sketch is not what is being exercised.
//
// The MEAN point is asserted rather than constrained, because the engine's arc is
// built from a centre and two ends and has no third through-point: that the arc
// passes through (R_mean, 0) is invariant 3 of the trace derivation, and reading
// it back off the solved arc is what proves the construction reached it.
func stepTraceSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)

	proofkit.Step(t, "%s: r_c %.6f mm, hand %+.0f, centre (%.6f, %.6f)",
		g.label, sp.cutterRadius, sp.handSign, sp.centre.X, sp.centre.Y)
	apex := s.CreatePoint(0, 0)
	s.Fix(apex)

	cutterCentre := s.CreatePoint(sp.centre.X, sp.centre.Y)
	cutter := s.CreateCircle(cutterCentre, sp.cutterRadius)
	cutter.SetConstruction(true)
	s.Fix(cutterCentre)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*sp.cutterRadius))
	datumEnd := s.CreatePoint(sp.centre.X+sp.cutterRadius, sp.centre.Y)
	s.Fix(datumEnd)
	datum := s.CreateLine(cutterCentre, datumEnd)
	datum.SetConstruction(true)

	proofkit.Step(t, "the trace arc, centre coincident to the cutter's and radius r_c")
	arcCentre := s.CreatePoint(sp.centre.X, sp.centre.Y)
	toe := s.CreatePoint(sp.toe2d.X, sp.toe2d.Y)
	heel := s.CreatePoint(sp.heel2d.X, sp.heel2d.Y)
	arc := s.CreateArc(arcCentre, toe, heel)
	arc.SetConstruction(true)
	rLo := sp.rToe - traceOvershoot*sp.span
	rHi := sp.rHeel + traceOvershoot*sp.span
	toeSpoke := s.CreateLine(cutterCentre, toe)
	toeSpoke.SetConstruction(true)
	heelSpoke := s.CreateLine(cutterCentre, heel)
	heelSpoke.SetConstruction(true)
	spokeAngle := func(p pt2) float64 {
		d := p.sub(sp.centre)
		return degrees(math.Atan2(d.Y, d.X))
	}
	s.AddConstraint(
		sketch.NewCoincident(arcCentre, cutterCentre),
		sketch.NewRadius(arc, sp.cutterRadius),
		// SUBSTITUTION, and it is the second one this step makes. Each end is a
		// circle-circle intersection with TWO solutions — the near branch the mean
		// point sits on, and the far one that gives a kinked or back-bent trace —
		// and the end carries exactly one degree of freedom, so no single unsigned
		// row can separate them: a distance from the apex admits the reflection
		// across the apex-centre line, and a signed component or an apex azimuth
		// admits the circle's second crossing. The angle about the CUTTER CENTRE is
		// monotonic in the one freedom it removes, so it is the one row that picks
		// a branch. The circle-circle conditions §6 of the trace derivation states
		// — the toe end at cone distance R_toe - 0.06*span, the heel end at
		// R_heel + 0.06*span — are ASSERTED on the solved arc below instead.
		sketch.NewAngle(datum, toeSpoke, spokeAngle(sp.toe2d)),
		sketch.NewAngle(datum, heelSpoke, spokeAngle(sp.heel2d)),
	)

	proofkit.Step(t, "the invariants the trace derivation states")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the trace sketch: %v", err)
	}
	assertTraceInvariants(t, sp, arc, rLo, rHi, radians(p[keySpiralAngle]))
}

// assertTraceInvariants checks the trace against §9 of the derivation on the
// SOLVED arc: it is one circle of the cutter radius, its centre is r_c from the
// mean point so the arc passes through it, its ends sit on the toe and heel
// circles about the apex, and its tangent at the mean point makes the spiral
// angle psi with the cone element.
func assertTraceInvariants(t testing.TB, sp spiral, arc *sketch.Arc, rLo, rHi, psi float64) {
	t.Helper()
	centre := pt2{arc.Center.X(), arc.Center.Y()}
	mean := pt2{sp.rMean, 0}
	if d := math.Abs(centre.sub(mean).length() - sp.cutterRadius); d > 1e-9 {
		t.Errorf("the cutter centre is %.9f mm off r_c from the mean point, so the arc does not "+
			"pass through it", d)
	}
	if d := math.Abs(arc.R() - sp.cutterRadius); d > 1e-9 {
		t.Errorf("the trace arc solved to radius %.9f mm, want the cutter radius %.9f mm",
			arc.R(), sp.cutterRadius)
	}
	toe := pt2{arc.Start.X(), arc.Start.Y()}
	heel := pt2{arc.End.X(), arc.End.Y()}
	if d := math.Abs(toe.length() - rLo); d > 1e-9 {
		t.Errorf("the toe end is at cone distance %.9f mm, want %.9f mm — the wrong intersection "+
			"branch was kept", toe.length(), rLo)
	}
	if d := math.Abs(heel.length() - rHi); d > 1e-9 {
		t.Errorf("the heel end is at cone distance %.9f mm, want %.9f mm", heel.length(), rHi)
	}
	// The tangent at the mean point makes psi with the cone element (the x axis).
	// The radius mean->centre is perpendicular to it, so the angle the radius makes
	// with the y axis is psi, with the hand on the cos/y term.
	radial := centre.sub(mean)
	gotPsi := math.Atan2(-radial.X, sp.handSign*radial.Y)
	if d := math.Abs(gotPsi - psi); d > 1e-9 {
		t.Errorf("the trace makes %.9f rad with the cone element at the mean point, want the "+
			"Mean Spiral Angle %.9f rad", gotPsi, psi)
	}
	// The hand sign belongs on the cos/Cy term. Mirroring across the cone element
	// flips Cy and nothing else; putting the sign on Cx would mirror about
	// x = R_mean instead, a different curve that gives the two gears unequal twist.
	if sp.handSign*sp.centre.Y < 0 {
		t.Errorf("the cutter centre sits at Cy %.9f for hand %+.0f; the hand sign belongs on the "+
			"cos term", sp.centre.Y, sp.handSign)
	}
	if want := sp.rMean - sp.cutterRadius*math.Sin(psi); math.Abs(sp.centre.X-want) > 1e-12 {
		t.Errorf("Cx is %.12f, want R_mean - r_c*sin(psi) = %.12f", sp.centre.X, want)
	}
}
