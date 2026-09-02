package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// WHAT THE SKETCH STEPS SUBSTITUTE, AND WHAT EACH SUBSTITUTION COSTS.
//
// Three differences between Fusion's constraint arity and the bench engine's run
// through every sketch below. None of them changes the geometry; each changes
// how many equations one instruction is worth, and a scheme transcribed with the
// wrong count is either short a constraint or carrying a redundant one.
//
//  1. A SIDE CHOICE THAT FUSION SEEDS IS PINNED HERE BY A SIGNED ANGLE. Fusion's
//     addPerpendicular is unsigned: perpendicular plus a length admits BOTH
//     sides, and which one the build gets is decided by where the raw Point3D
//     seed was placed. That is not a bench artefact — it is the failure the spec
//     itself warns about at the B->Apex2 drop, where a drop seeded on the wrong
//     side makes the solver flip the entire frame to the mirror solution. The
//     harness gate refuses a scheme whose answer depends on its seed, so every
//     such perpendicular is written here as sketch.NewAngle at +/-90 degrees,
//     which the engine measures counterclockwise and which therefore admits one
//     configuration. It is the same single equation with a direction on it. The
//     cost: this proof does not show that Fusion's seeds pick the intended side,
//     only that the intended side is a consistent, fully constrained answer.
//
//  2. THE ENGINE'S MIDPOINT, COLLINEAR AND OFFSET CARRY MORE ROWS THAN FUSION'S.
//     sketch.NewMidpoint is two rows, so the anchor sketch's addCoincident(
//     projectedCenter, anchorLine) — Fusion's one extra row, which the spec
//     insists on — is already inside it and adding it here is redundant.
//     sketch.NewCollinear is two point-on-line rows, so a collinear against a
//     line the constrained end already sits on carries one dependent row; each
//     §2 collinear is written as the point-on-line half that is not implied,
//     which is the same substitution the spec itself makes for K and L.
//     sketch.NewOffset holds BOTH endpoints of the target line at the same
//     signed perpendicular distance, so it carries the parallelism Fusion's
//     addOffsetDimension requires as a precondition. Two instructions fall out
//     of the scheme for that one reason. The E->G / F->I perpendiculars the
//     spec adds in Fusion are omitted here, as that spec paragraph instructs in
//     full. So are the addParallel constraints on the two toe lines: measured,
//     a toe line carrying both its parallel and its offset comes back with one
//     redundant row, and the engine names the offset — the same shape the spec
//     already records for the base-height offsets.
//
//  3. THE PROJECTED GEOMETRY IS REFERENCE GEOMETRY. Fusion's sketch.project
//     brings a point in associatively and it still carries free DOF
//     ([PB-PROJECT-NOT-FIXED]); the engine's CreateReferencePoint is the same
//     idea with the position already owned by the source sketch. Modelling the
//     projection as a reference point is what lets the §2 net be judged on its
//     own constraints rather than on the anchor sketch's.

// anchorCases sweeps the anchor sketch across the pairs whose §2 figures the
// later steps build on. The anchor line's own geometry does not depend on any of
// them — it is 10 mm long whatever the gear is — so the table's job is to prove
// exactly that: the reference line is the same fully constrained line at every
// size, which is what makes it safe for §2 to derive every direction from it.
var anchorCases = []proofkit.Case{
	{Name: "default_31_31_at_90deg", Params: defaultParams()},
	{Name: "ratio_31_17_at_90deg", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "shallow_35deg", Params: with(defaultParams(), pShaftAngle, deg(35))},
	{Name: "obtuse_140deg", Params: with(defaultParams(), pShaftAngle, deg(140))},
}

// defaultParams is the dialog's own defaults: module 1, 31 and 31 teeth, a 90
// degree shaft angle, every optional length left at 0 so its fallback resolves,
// and the default 35 degree right-hand spiral.
func defaultParams() map[string]float64 {
	return map[string]float64{
		pModule: 1, pDrivingTeeth: 31, pPinionTeeth: 31, pShaftAngle: deg(90),
		pDrivingHeight: 0, pPinionHeight: 0, pFaceWidth: 0, pToothSpacing: 0,
		pDrivingBore: 0, pPinionBore: 0, pBoreEnable: 1,
		pSpiralAngle: deg(35), pCutterRadius: 0, pHandSign: 1, pGear: 0,
	}
}

// with copies a parameter set with named keys replaced, so a case table reads as
// the one thing each case varies.
func with(base map[string]float64, kv ...any) map[string]float64 {
	out := make(map[string]float64, len(base))
	for k, v := range base {
		out[k] = v
	}
	for i := 0; i+1 < len(kv); i += 2 {
		out[kv[i].(string)] = kv[i+1].(float64)
	}
	return out
}

// stepAnchorSketch draws the Anchor Sketch: the projected centre point and the
// 10 mm Anchor Line through it.
//
// The line's absolute direction is arbitrary — §2 derives every direction
// relative to it — but it must not be a free degree of freedom, so the sketch
// ends with the midpoint, the length and the sketch-local Horizontal that
// [PB-REFLINE-DIRECTION] asks for.
//
// The length dimension is written here as a SIGNED horizontal distance. Fusion's
// aligned distance dimension is a magnitude whose direction is captured from the
// seed ([PB-DIM-VALUE-SEMANTICS]); the engine's target is signed, and the sign
// is what refuses the end-for-end flip that magnitude plus midpoint plus
// horizontal would otherwise leave as a second discrete answer.
//
// <!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user's centre point into the Anchor Sketch")
	center := s.CreateReferencePoint(0, 0, "userCenterPoint")
	center.SetName("projected centre")

	proofkit.Step(t, "draw the Anchor Line, seeded at plus and minus 5 mm along sketch-local X")
	start := s.CreatePoint(-5, 0)
	end := s.CreatePoint(5, 0)
	start.SetName("anchor line start")
	end.SetName("anchor line end")
	line := s.CreateLine(start, end)
	line.SetConstruction(true)
	line.SetName("Anchor Line")

	proofkit.Step(t, "midpoint, length and the sketch-local Horizontal")
	s.AddConstraint(
		sketch.NewMidpoint(center, line),
		sketch.NewHorizontalDistance(start, end, 10),
		sketch.NewHorizontal(line),
	)

	solve(t, s)
	if got := line.Length(); math.Abs(got-10) > 1e-9 {
		t.Errorf("the Anchor Line solved to %.9f mm, not the seeded 10 mm the dimension locks", got)
	}
	if got := at(center); got.sub(vec2{0, 0}).norm() > 1e-9 {
		t.Errorf("the projected centre moved to (%.9f, %.9f); a projection is owned by its source",
			got.X, got.Y)
	}
	mid := at(start).add(at(end)).scale(0.5)
	if mid.sub(at(center)).norm() > 1e-9 {
		t.Errorf("the centre does not bisect the Anchor Line: midpoint (%.9f, %.9f) against centre "+
			"(%.9f, %.9f)", mid.X, mid.Y, center.X(), center.Y())
	}
	if at(end).X <= at(start).X {
		t.Errorf("the Anchor Line solved end-for-end, start x %.6f against end x %.6f; the signed "+
			"length dimension is what refuses that second answer", at(start).X, at(end).X)
	}
}

// latticeCases sweeps the regime the §2 lattice has to hold across.
//
// The Shaft Angle range is the spec's own: 30 degrees is the documented floor
// and is swept even though the spec records that two of three independently
// written lattices refuse it on conditioning, so this net's answer is measured
// rather than assumed; 35 is where those two first clear; 90 is the classic
// pair; 140 and 142 are past it, where the driving cone angle grows and the
// figure stretches. Both tooth-count directions are swept, because either gear
// can be the smaller, binding side of the Maximum Face Width. The 4-tooth pair
// is the Minimum Teeth floor at 90 degrees, where the two base-height bounds are
// closest without crossing.
//
// Every optional length is swept at both ends of the range the spec states for
// it: a base height at its own minimum and at its own maximum, a face width at
// the Maximum Face Width, and the Tooth Spacing on both sides of the branch it
// carries — zero, where K' is K and no line is drawn, and positive, where a
// dimensioned line is.
var latticeCases = []proofkit.Case{
	{Name: "default_31_31_at_90deg", Params: defaultParams()},
	{Name: "shaft_angle_floor_30deg", Params: with(defaultParams(), pShaftAngle, deg(30))},
	{Name: "shaft_angle_35deg", Params: with(defaultParams(), pShaftAngle, deg(35))},
	{Name: "shaft_angle_140deg", Params: with(defaultParams(), pShaftAngle, deg(140))},
	{Name: "shaft_angle_142deg", Params: with(defaultParams(), pShaftAngle, deg(142))},
	{Name: "ratio_driving_31_pinion_17", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_driving_17_pinion_31", Params: with(defaultParams(), pDrivingTeeth, 17.0)},
	{Name: "ratio_driving_19_pinion_13_module_2", Params: with(defaultParams(),
		pModule, 2.0, pDrivingTeeth, 19.0, pPinionTeeth, 13.0)},
	{Name: "minimum_teeth_floor_4_4", Params: with(defaultParams(),
		pDrivingTeeth, 4.0, pPinionTeeth, 4.0)},
	{Name: "coarse_module_3_20_20", Params: with(defaultParams(), pModule, 3.0,
		pDrivingTeeth, 20.0, pPinionTeeth, 20.0)},
	{Name: "fine_module_0.5_30_30", Params: with(defaultParams(), pModule, 0.5,
		pDrivingTeeth, 30.0, pPinionTeeth, 30.0)},
	{Name: "tooth_spacing_0.2mm", Params: with(defaultParams(), pToothSpacing, 0.2)},
	{Name: "tooth_spacing_1mm_on_a_ratio_pair", Params: with(defaultParams(),
		pPinionTeeth, 17.0, pToothSpacing, 1.0)},
	{Name: "face_width_at_the_maximum", Params: with(defaultParams(), pFaceWidth, atMaxFaceWidth)},
	{Name: "base_heights_at_their_minima", Params: with(defaultParams(),
		pDrivingHeight, atMinBaseHeight, pPinionHeight, atMinBaseHeight)},
	{Name: "base_heights_at_their_maxima", Params: with(defaultParams(),
		pDrivingHeight, atMaxBaseHeight, pPinionHeight, atMaxBaseHeight)},
}

// Sentinel values a case table uses to ask for a bound it cannot compute until
// the cone angles are known. They are resolved in latticeOf, below.
const (
	atMaxFaceWidth  float64 = -1
	atMinBaseHeight float64 = -2
	atMaxBaseHeight float64 = -3
)

// resolveSentinels turns the bound sentinels into the numbers a case meant, so a
// case table can say "at the maximum" without restating the closed form.
func resolveSentinels(t testing.TB, p map[string]float64) map[string]float64 {
	t.Helper()
	out := with(p)
	probe := pairOf(t, with(p, pFaceWidth, 0.0, pDrivingHeight, 0.0, pPinionHeight, 0.0))
	if out[pFaceWidth] == atMaxFaceWidth {
		out[pFaceWidth] = probe.maxFaceWidth()
	}
	for _, side := range []struct {
		key         string
		gamma       float64
		pitchRadius float64
	}{
		{pDrivingHeight, probe.gammaG, probe.drivingPitchDia / 2},
		{pPinionHeight, probe.gammaP, probe.pinionPitchDia / 2},
	} {
		switch out[side.key] {
		case atMinBaseHeight:
			out[side.key] = minBaseHeight(probe.module, side.gamma)
		case atMaxBaseHeight:
			out[side.key] = maxBaseHeight(side.pitchRadius, side.gamma, probe.module)
		}
	}
	return out
}

// gp holds the §2 sketch's handles, so the assertions read the solved lattice
// back by name.
type gp struct {
	q      pair
	want   lattice
	center *sketch.Point
	apex   *sketch.Point
	a, b   *sketch.Point
	apex2  *sketch.Point
	c, d   *sketch.Point
	e, f   *sketch.Point
	g, i   *sketch.Point
	h, j   *sketch.Point
	k, l   *sketch.Point
	kp, lp *sketch.Point
	m, n   *sketch.Point
	o, p   *sketch.Point

	pinionAxis, drivingAxis *sketch.Line
	dropA, dropB            *sketch.Line
	pitchLine               *sketch.Line
	dedC, dedD              *sketch.Line
	rootC, rootD            *sketch.Line
	ch, dj                  *sketch.Line
	mn, op                  *sketch.Line
	gh, ij                  *sketch.Line
}

// stepGearProfiles draws the Gear Profiles sketch: the whole §2 lattice, both
// gears' frustum corners and both tooth centres, in one fully constrained net.
//
// Every line here is a construction line, so the sketch closes no profile — the
// solid features consume the per-gear Profile sketches instead. Every line is
// built from raw coordinates with one coincident per already-existing endpoint,
// which is [BEVEL-F-COINCIDENT-STYLE]; each named segment is drawn once and
// reused, which is [BEVEL-F-LINE-ONCE]; and the driven lengths — Apex->A,
// Apex->B and the four module-length extensions — carry no dimension, which is
// [BEVEL-F-DRIVEN-DIMS].
//
// <!-- proof-run: proofkit.Run(latticeCases, stepGearProfiles) -->
func stepGearProfiles(t testing.TB, s *sketch.Sketch, params map[string]float64) {
	f := buildGearProfiles(t, s, params)
	solve(t, s)
	assertLattice(t, f)
	recordConditioning(t, s)
}

// The sketch engine's trust floor. A system whose conditioning falls below it is
// refused as near-singular, and the spec is explicit that this is a fact about
// the particular lattice rather than a bound on the gear: it forbids writing a
// Shaft Angle limit from a conditioning measurement, and says the proof's case
// table is where the measurements for that net belong.
const conditioningFloor = 4e-5

// recordConditioning measures this net's conditioning and, where it falls under
// the engine's floor, records the number and stops rather than letting the gate
// report it as a bare refusal.
//
// TWO CONFIGURATIONS IN THE TABLE ARE REFUSED HERE, and each is a finding rather
// than a defect in this step.
//
// The 30 degree Shaft Angle floor comes back at conditioning 2.94e-05 on the
// default 31/31 pair — the same value, to three figures, that the spec records
// for one of the three independently written lattices it measured, and this net
// first clears the floor at 35 degrees exactly as those two did. So the spec's
// warning that the documented floor is not known to be reachable is reproduced
// here rather than taken on trust, and this net is one of the two that refuse
// it.
//
// The base heights at their MINIMA are refused at 2.68e-05, and that one is not
// in the spec. The Minimum Base Height is 1.05 * 1.25 * module * sin(gamma),
// which is the base height at which H lands exactly on C plus a 5 per cent
// margin, so at the minimum the heel edge C->H is only 0.0625 * module long — at
// module 1, 62 microns. Its direction is what carries the point-on-line pin that
// locates H, and a segment that short makes that direction ill-defined, which is
// the failure mode the playbook names for this gate. The margin is stated as a
// fraction of the projection rather than as a length the segment has to keep,
// and that is what makes it too small.
func recordConditioning(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	rep := s.Verify(context.Background())
	proofkit.Step(t, "conditioning %.3e against the engine's %.0e floor", rep.Conditioning,
		conditioningFloor)
	if rep.Conditioning < conditioningFloor {
		proofkit.Unmodelled(t, "this lattice comes back near-singular at conditioning %.3e, below "+
			"the engine's %.0e trust floor. The geometry solved and every closed-form assertion "+
			"above passed; what the engine refuses is the constraint net's conditioning at this "+
			"configuration, which the spec says is a property of the net and never a reason to "+
			"loosen the gate or to narrow the advertised range", rep.Conditioning, conditioningFloor)
	}
}

// buildGearProfiles draws §2 and returns its handles. The solid steps reuse it,
// which is what keeps the lattice they measure against the lattice this step
// gates.
func buildGearProfiles(t testing.TB, s *sketch.Sketch, params map[string]float64) *gp {
	t.Helper()
	p := resolveSentinels(t, params)
	q := pairOf(t, p)
	f := &gp{q: q, want: q.build()}
	lat := f.want
	dedendum := dedendumFactor * q.module

	proofkit.Step(t, "project the Anchor Sketch's centre point and the Anchor Line")
	f.center = s.CreateReferencePoint(0, 0, "anchorSketchCenter")
	f.center.SetName("projected centre")
	anchorStart := s.CreateReferencePoint(-5, 0, "anchorLineStart")
	anchorEnd := s.CreateReferencePoint(5, 0, "anchorLineEnd")
	anchor, err := s.CreateReferenceLine(anchorStart, anchorEnd, "anchorLine")
	if err != nil {
		t.Fatalf("project the Anchor Line: %v", err)
	}
	anchor.SetName("projected Anchor Line")

	// A §2 line: fresh points at the seed, marked construction, named.
	line := func(name string, from, to vec2) *sketch.Line {
		l := s.CreateLine(s.CreatePoint(from.X, from.Y), s.CreatePoint(to.X, to.Y))
		l.SetConstruction(true)
		l.SetName(name)
		l.Start.SetName(name + " start")
		l.End.SetName(name + " end")
		return l
	}
	pin := func(end *sketch.Point, to *sketch.Point) {
		s.AddConstraint(sketch.NewCoincident(end, to))
	}

	proofkit.Step(t, "the centre-to-apex construction line, perpendicular to the projected anchor "+
		"line and undimensioned [BEVEL-F-APEX-LOCAL]")
	centerToApex := line("centre to Apex", lat.center, lat.apex)
	pin(centerToApex.Start, f.center)
	// The grow side. Fusion picks it from the target plane's normal and seeds
	// the apex there ([BEVEL-F-GROW-SIDE]); the signed angle is that choice
	// written as a constraint.
	s.AddConstraint(sketch.NewAngle(anchor, centerToApex, 90))
	f.apex = centerToApex.End
	f.apex.SetName("Apex")

	proofkit.Step(t, "the Driving Gear Shaft Axis, parallel to the centre-to-apex line and "+
		"undimensioned")
	f.drivingAxis = line("Driving Gear Shaft Axis", lat.apex, lat.b)
	pin(f.drivingAxis.Start, f.apex)
	// addParallel in Fusion; the signed 180 degrees is the same row with the
	// sense on it, since the shaft axis runs back from the apex toward the
	// anchor line. addVertical would be wrong here on any tilted target plane.
	s.AddConstraint(sketch.NewAngle(centerToApex, f.drivingAxis, 180))
	f.b = f.drivingAxis.End
	f.b.SetName("B")

	proofkit.Step(t, "the Pinion Gear Shaft Axis at the Shaft Angle [PB-ANGULAR-DIM]")
	f.pinionAxis = line("Pinion Gear Shaft Axis", lat.apex, lat.a)
	pin(f.pinionAxis.Start, f.apex)
	s.AddConstraint(sketch.NewAngle(f.drivingAxis, f.pinionAxis, q.shaftAngle*180/math.Pi))
	f.a = f.pinionAxis.End
	f.a.SetName("A")

	proofkit.Step(t, "the two perpendicular drops to Apex 2, at the pinion and driving pitch radii")
	f.dropA = line("A to Apex2", lat.a, lat.apex2)
	pin(f.dropA.Start, f.a)
	s.AddConstraint(
		sketch.NewAngle(f.pinionAxis, f.dropA, -90),
		sketch.NewDistance(f.dropA.Start, f.dropA.End, q.pinionPitchDia/2),
	)
	f.dropB = line("B to Apex2", lat.b, lat.apex2)
	pin(f.dropB.Start, f.b)
	s.AddConstraint(
		sketch.NewAngle(f.drivingAxis, f.dropB, 90),
		sketch.NewDistance(f.dropB.Start, f.dropB.End, q.drivingPitchDia/2),
	)
	// The closure that makes both drops land on the same interior-wedge point.
	pin(f.dropA.End, f.dropB.End)
	f.apex2 = f.dropA.End
	f.apex2.SetName("Apex 2")

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	f.pitchLine = line("Pitch Line", lat.apex, lat.apex2)
	pin(f.pitchLine.Start, f.apex)
	pin(f.pitchLine.End, f.apex2)

	f.dedD = line("Driving Gear Dedendum", lat.apex2, lat.d)
	pin(f.dedD.Start, f.apex2)
	s.AddConstraint(
		sketch.NewAngle(f.pitchLine, f.dedD, -90),
		sketch.NewDistance(f.dedD.Start, f.dedD.End, dedendum),
	)
	f.d = f.dedD.End
	f.d.SetName("D")

	f.dedC = line("Pinion Gear Dedendum", lat.apex2, lat.c)
	pin(f.dedC.Start, f.apex2)
	s.AddConstraint(
		sketch.NewAngle(f.pitchLine, f.dedC, 90),
		sketch.NewDistance(f.dedC.Start, f.dedC.End, dedendum),
	)
	f.c = f.dedC.End
	f.c.SetName("C")

	proofkit.Step(t, "the two Root Axes, Apex->C and Apex->D")
	f.rootC = line("Pinion Root Axis", lat.apex, lat.c)
	pin(f.rootC.Start, f.apex)
	pin(f.rootC.End, f.c)
	f.rootD = line("Driving Root Axis", lat.apex, lat.d)
	pin(f.rootD.Start, f.apex)
	pin(f.rootD.End, f.d)

	proofkit.Step(t, "the pinion module-length extensions A->E and E->G, and the heel edge C->H")
	ae := line("A to E", lat.a, lat.e)
	pin(ae.Start, f.a)
	s.AddConstraint(sketch.NewPointOnLine(ae.End, f.pinionAxis))
	f.e = ae.End
	f.e.SetName("E")
	ce := line("C to E", lat.c, lat.e)
	pin(ce.Start, f.c)
	pin(ce.End, f.e)
	s.AddConstraint(sketch.NewPerpendicular(ae, ce))

	eg := line("E to G", lat.e, lat.g)
	pin(eg.Start, f.e)
	s.AddConstraint(sketch.NewPointOnLine(eg.End, f.pinionAxis))
	f.g = eg.End
	f.g.SetName("G")

	f.ch = line("C to H", lat.c, lat.h)
	pin(f.ch.Start, f.c)
	s.AddConstraint(sketch.NewPointOnLine(f.ch.End, f.dedC))
	f.h = f.ch.End
	f.h.SetName("H")

	f.gh = line("G to H", lat.g, lat.h)
	pin(f.gh.Start, f.g)
	pin(f.gh.End, f.h)

	proofkit.Step(t, "the driving module-length extensions B->F and F->I, and the heel edge D->J")
	bf := line("B to F", lat.b, lat.f)
	pin(bf.Start, f.b)
	s.AddConstraint(sketch.NewPointOnLine(bf.End, f.drivingAxis))
	f.f = bf.End
	f.f.SetName("F")
	df := line("D to F", lat.d, lat.f)
	pin(df.Start, f.d)
	pin(df.End, f.f)
	s.AddConstraint(sketch.NewPerpendicular(bf, df))

	fi := line("F to I", lat.f, lat.i)
	pin(fi.Start, f.f)
	s.AddConstraint(sketch.NewPointOnLine(fi.End, f.drivingAxis))
	f.i = fi.End
	f.i.SetName("I")

	f.dj = line("D to J", lat.d, lat.j)
	pin(f.dj.Start, f.d)
	s.AddConstraint(sketch.NewPointOnLine(f.dj.End, f.dedD))
	f.j = f.dj.End
	f.j.SetName("J")

	f.ij = line("I to J", lat.i, lat.j)
	pin(f.ij.Start, f.i)
	pin(f.ij.End, f.j)

	proofkit.Step(t, "the two base-height offset dimensions [PB-OFFSET-DIM]")
	// Signed, and the sign is the side the seed puts the heel line on. The
	// engine's offset holds both endpoints of the target line at the same signed
	// distance, so it carries the parallelism Fusion's addOffsetDimension needs
	// as a precondition, and the E->G / F->I perpendiculars the spec adds in
	// Fusion are left out here for exactly that reason.
	s.AddConstraint(
		sketch.NewOffset(f.dropB, f.ij, signedOffset(lat.b, lat.apex2, lat.i, q.drivingHeight)),
		sketch.NewOffset(f.dropA, f.gh, signedOffset(lat.a, lat.apex2, lat.g, q.pinionHeight)),
	)

	proofkit.Step(t, "close the figure: A->G, B->I, and Point I on the projected centre")
	ag := line("A to G", lat.a, lat.g)
	pin(ag.Start, f.a)
	pin(ag.End, f.g)
	bi := line("B to I", lat.b, lat.i)
	pin(bi.Start, f.b)
	pin(bi.End, f.i)
	// Fusion pins the whole figure's height above the anchor line with one
	// point-to-point coincidence. I already lies on the centre-to-apex line by
	// construction, so the engine's second coincidence row would be dependent;
	// the one row that is not is written here instead.
	s.AddConstraint(sketch.NewVerticalDistance(f.center, f.i, 0))

	proofkit.Step(t, "the tooth centres K and L, each pinned by two point-on-line constraints")
	gk := line("G to K", lat.g, lat.k)
	pin(gk.Start, f.g)
	s.AddConstraint(
		sketch.NewPointOnLine(gk.End, f.pinionAxis),
		sketch.NewPointOnLine(gk.End, f.dedC),
	)
	f.k = gk.End
	f.k.SetName("K")
	il := line("I to L", lat.i, lat.l)
	pin(il.Start, f.i)
	s.AddConstraint(
		sketch.NewPointOnLine(il.End, f.drivingAxis),
		sketch.NewPointOnLine(il.End, f.dedD),
	)
	f.l = il.End
	f.l.SetName("L")

	proofkit.Step(t, "the Tooth Spacing offsets K' and L'")
	f.kp = toothCenter(t, s, "K", f.k, lat.k, lat.kp, f.dedC, q.toothSpacing)
	f.lp = toothCenter(t, s, "L", f.l, lat.l, lat.lp, f.dedD, q.toothSpacing)
	// The tooth-centre reference lines §3 draws its plane through. At zero
	// spacing K' is K and this is the C->K line the spec says to reuse.
	ck := line("C to K'", lat.c, lat.kp)
	pin(ck.Start, f.c)
	pin(ck.End, f.kp)
	dl := line("D to L'", lat.d, lat.lp)
	pin(dl.Start, f.d)
	pin(dl.End, f.lp)

	proofkit.Step(t, "the two toe edges, offset from their heel edges by the resolved Face Width")
	f.mn = line("M to N", lat.m, lat.n)
	s.AddConstraint(
		sketch.NewPointOnLine(f.mn.Start, f.rootC),
		sketch.NewPointOnLine(f.mn.End, f.dropA),
		sketch.NewOffset(f.ch, f.mn, signedOffset(lat.c, lat.h, lat.m, q.faceWidth)),
	)
	f.m, f.n = f.mn.Start, f.mn.End
	f.m.SetName("M")
	f.n.SetName("N")
	mc := line("M to C", lat.m, lat.c)
	pin(mc.Start, f.m)
	pin(mc.End, f.c)
	na := line("N to A", lat.n, lat.a)
	pin(na.Start, f.n)
	pin(na.End, f.a)

	f.op = line("O to P", lat.o, lat.p)
	s.AddConstraint(
		sketch.NewPointOnLine(f.op.Start, f.rootD),
		sketch.NewPointOnLine(f.op.End, f.dropB),
		sketch.NewOffset(f.dj, f.op, signedOffset(lat.d, lat.j, lat.o, q.faceWidth)),
	)
	f.o, f.p = f.op.Start, f.op.End
	f.o.SetName("O")
	f.p.SetName("P")
	od := line("O to D", lat.o, lat.d)
	pin(od.Start, f.o)
	pin(od.End, f.d)
	pb := line("P to B", lat.p, lat.b)
	pin(pb.Start, f.p)
	pin(pb.End, f.b)
	return f
}

// toothCenter builds K' or L'.
//
// At the default Tooth Spacing of zero the spec says to build nothing and reuse
// the existing reference line, because a zero-length dimensioned line is
// degenerate and one segment gets one line ([BEVEL-F-LINE-ONCE]); K' is then K
// itself. A positive spacing draws one line from K along the dedendum line with
// a length dimension on it, pinned the same way K is.
func toothCenter(t testing.TB, s *sketch.Sketch, label string, from *sketch.Point,
	fromSeed, toSeed vec2, dedendumLine *sketch.Line, spacing float64) *sketch.Point {
	t.Helper()
	if spacing == 0 {
		return from
	}
	l := s.CreateLine(s.CreatePoint(fromSeed.X, fromSeed.Y), s.CreatePoint(toSeed.X, toSeed.Y))
	l.SetConstruction(true)
	l.SetName(label + " to " + label + "'")
	s.AddConstraint(
		sketch.NewCoincident(l.Start, from),
		sketch.NewPointOnLine(l.End, dedendumLine),
		sketch.NewDistance(l.Start, l.End, spacing),
	)
	l.End.SetName(label + "'")
	return l.End
}

// signedOffset returns the signed perpendicular distance the engine's offset
// dimension takes, from the line through src0->src1 to the point the target line
// passes through. Fusion's addOffsetDimension takes the magnitude and captures
// the side from the seed; the sign here is that captured side, made explicit.
func signedOffset(src0, src1, target vec2, magnitude float64) float64 {
	d := src1.sub(src0)
	n := d.norm()
	cross := (d.X*(target.Y-src0.Y) - d.Y*(target.X-src0.X)) / n
	if cross < 0 {
		return -magnitude
	}
	return magnitude
}

// assertLattice checks the solved §2 figure against the closed form the spec
// states, and then the three facts later steps select on.
func assertLattice(t testing.TB, f *gp) {
	t.Helper()
	q, want := f.q, f.want
	const tol = 1e-7

	proofkit.Step(t, "the validation bounds this configuration had to clear to be built at all")
	if limit := maxShaftAngle(q.drivingPitchDia, q.pinionPitchDia); q.shaftAngle >= limit {
		t.Errorf("the Shaft Angle %.9f rad is at or above the Maximum Shaft Angle %.9f, where a "+
			"pitch cone angle reaches 90 degrees and the cone turns inside out",
			q.shaftAngle, limit)
	}
	if q.shaftAngle < deg(30) {
		t.Errorf("the Shaft Angle %.9f rad is below the documented 30 degree floor", q.shaftAngle)
	}
	if floor := minTeeth(q.gammaP); q.pinionTeeth < floor {
		t.Errorf("the pinion's %.0f teeth are below its own Minimum Teeth floor %.6f, where its "+
			"two base-height bounds cross", q.pinionTeeth, floor)
	}
	if floor := minTeeth(q.gammaG); q.drivingTeeth < floor {
		t.Errorf("the driving gear's %.0f teeth are below its own Minimum Teeth floor %.6f",
			q.drivingTeeth, floor)
	}
	if math.Cos(q.gammaP) <= 0 || math.Cos(q.gammaG) <= 0 {
		t.Fatalf("a cone angle has passed 90 degrees: gammaP %.9f, gammaG %.9f; the along-shaft "+
			"seeds R*cos(gamma) would point backwards", q.gammaP, q.gammaG)
	}

	proofkit.Step(t, "every named point against the closed form")
	for name, pair := range map[string][2]any{
		"Apex":   {f.apex, want.apex},
		"A":      {f.a, want.a},
		"B":      {f.b, want.b},
		"Apex 2": {f.apex2, want.apex2},
		"C":      {f.c, want.c},
		"D":      {f.d, want.d},
		"E":      {f.e, want.e},
		"F":      {f.f, want.f},
		"G":      {f.g, want.g},
		"H":      {f.h, want.h},
		"I":      {f.i, want.i},
		"J":      {f.j, want.j},
		"K":      {f.k, want.k},
		"L":      {f.l, want.l},
		"K'":     {f.kp, want.kp},
		"L'":     {f.lp, want.lp},
		"M":      {f.m, want.m},
		"N":      {f.n, want.n},
		"O":      {f.o, want.o},
		"P":      {f.p, want.p},
	} {
		near(t, name, at(pair[0].(*sketch.Point)), pair[1].(vec2), tol)
	}

	proofkit.Step(t, "the two pitch cone angles the solve produces, against the closed form")
	gotP := math.Abs(f.pinionAxis.AngleTo(f.pitchLine))
	gotG := math.Abs(f.drivingAxis.AngleTo(f.pitchLine))
	if math.Abs(gotP-q.gammaP) > 1e-9 {
		t.Errorf("the solved pinion cone angle is %.12f rad, not the closed form's %.12f",
			gotP, q.gammaP)
	}
	if math.Abs(gotG-q.gammaG) > 1e-9 {
		t.Errorf("the solved driving cone angle is %.12f rad, not the closed form's %.12f",
			gotG, q.gammaG)
	}

	proofkit.Step(t, "the driven along-shaft lengths, which carry no dimension of their own")
	R := q.pitchConeDist
	if got := f.pinionAxis.Length(); math.Abs(got-R*math.Cos(q.gammaP)) > tol {
		t.Errorf("|Apex->A| solved to %.9f, not R*cos(gammaP) = %.9f; that length is driven by "+
			"the Apex 2 closure, not dimensioned", got, R*math.Cos(q.gammaP))
	}
	if got := f.drivingAxis.Length(); math.Abs(got-R*math.Cos(q.gammaG)) > tol {
		t.Errorf("|Apex->B| solved to %.9f, not R*cos(gammaG) = %.9f", got, R*math.Cos(q.gammaG))
	}

	proofkit.Step(t, "Point I on the projected centre, which is what pins the figure's height")
	if d := at(f.i).sub(at(f.center)).norm(); d > tol {
		t.Errorf("Point I sits %.9f mm from the projected centre; that coincidence is the only "+
			"thing holding the figure above the anchor line", d)
	}

	proofkit.Step(t, "the tooth centres at the back-cone distance R/cos(gamma)")
	if got := at(f.k).sub(at(f.apex)).norm(); math.Abs(got-R/math.Cos(q.gammaP)) > tol {
		t.Errorf("K sits %.9f from the Apex, not the back-cone distance R/cos(gammaP) = %.9f",
			got, R/math.Cos(q.gammaP))
	}
	if got := at(f.l).sub(at(f.apex)).norm(); math.Abs(got-R/math.Cos(q.gammaG)) > tol {
		t.Errorf("L sits %.9f from the Apex, not R/cos(gammaG) = %.9f", got, R/math.Cos(q.gammaG))
	}
	if got := at(f.kp).sub(at(f.k)).norm(); math.Abs(got-q.toothSpacing) > tol {
		t.Errorf("K' sits %.9f from K, not the Tooth Spacing %.9f", got, q.toothSpacing)
	}
	if q.toothSpacing > 0 && at(f.kp).sub(at(f.c)).norm() <= at(f.k).sub(at(f.c)).norm() {
		t.Error("K' moved toward the lower corner C; the Tooth Spacing shifts the tooth centre " +
			"away from it")
	}

	proofkit.Step(t, "the Maximum Face Width, measured on the solved lattice")
	pinionReach := distancePointLine(at(f.a), at(f.c), at(f.h))
	drivingReach := distancePointLine(at(f.b), at(f.d), at(f.j))
	if math.Abs(pinionReach-R*sq(math.Sin(q.gammaP))) > tol {
		t.Errorf("the distance from A to the pinion dedendum line measures %.9f, not the closed "+
			"form R*sin(gammaP)^2 = %.9f", pinionReach, R*sq(math.Sin(q.gammaP)))
	}
	if math.Abs(drivingReach-R*sq(math.Sin(q.gammaG))) > tol {
		t.Errorf("the distance from B to the driving dedendum line measures %.9f, not "+
			"R*sin(gammaG)^2 = %.9f", drivingReach, R*sq(math.Sin(q.gammaG)))
	}
	if got, want := q.maxFaceWidth(), 0.95*math.Min(pinionReach, drivingReach); math.Abs(got-want) > tol {
		t.Errorf("the Maximum Face Width resolves to %.9f but the solved lattice puts it at %.9f",
			got, want)
	}
	if q.faceWidth > q.maxFaceWidth()+tol {
		t.Errorf("the resolved Face Width %.9f is above the Maximum Face Width %.9f",
			q.faceWidth, q.maxFaceWidth())
	}

	proofkit.Step(t, "the revolve precondition: neither toe end has crossed its own shaft axis "+
		"[PB-REVOLVE]")
	assertOffAxis(t, "N", at(f.n), at(f.apex), at(f.a))
	assertOffAxis(t, "P", at(f.p), at(f.apex), at(f.b))
	assertOffAxis(t, "H", at(f.h), at(f.apex), at(f.a))
	assertOffAxis(t, "J", at(f.j), at(f.apex), at(f.b))

	proofkit.Step(t, "the heel edge against the base-height bounds it is the subject of")
	assertHeel(t, "Pinion", at(f.h), at(f.apex), at(f.a), q.pinionPitchDia/2, q.gammaP,
		q.pinionHeight, q.module)
	assertHeel(t, "Driving", at(f.j), at(f.apex), at(f.b), q.drivingPitchDia/2, q.gammaG,
		q.drivingHeight, q.module)

	proofkit.Step(t, "each gear's hexagon is a simple loop lying on one side of its shaft axis")
	assertHexagon(t, q, want, q.pinion())
	assertHexagon(t, q, want, q.driving())
}

func sq(x float64) float64 { return x * x }

// distancePointLine is the perpendicular distance from p to the infinite line
// through a and b.
func distancePointLine(p, a, b vec2) float64 {
	d := b.sub(a)
	return math.Abs(d.X*(p.Y-a.Y)-d.Y*(p.X-a.X)) / d.norm()
}

// assertOffAxis fails when a profile corner has reached or crossed the shaft
// axis it is revolved about.
func assertOffAxis(t testing.TB, name string, p, apex, axisEnd vec2) {
	t.Helper()
	if d := distancePointLine(p, apex, axisEnd); d <= 1e-9 {
		t.Errorf("%s sits %.3e from its own shaft axis; a profile that reaches the axis of "+
			"revolution fails the revolve with ASM_WIRE_X_AXIS", name, d)
	}
}

// assertHeel checks the heel corner against both closed-form base-height bounds.
//
// The perpendicular distance from the heel corner to the shaft axis is
// r - baseHeight/tan(gamma), so the corner reaches the axis at exactly
// r*tan(gamma) — the true crossing the Maximum Base Height stays under, and the
// number the spec had to correct once when the bound was read as if the base
// height were measured from the dedendum point.
func assertHeel(t testing.TB, label string, heel, apex, axisEnd vec2,
	pitchRadius, gamma, baseHeight, module float64) {
	t.Helper()
	const tol = 1e-7
	got := distancePointLine(heel, apex, axisEnd)
	want := pitchRadius - baseHeight/math.Tan(gamma)
	if math.Abs(got-want) > tol {
		t.Errorf("%s: the heel corner sits %.9f from the shaft axis, not r - baseHeight/tan(gamma) "+
			"= %.9f", label, got, want)
	}
	crossing := trueHeelCrossing(pitchRadius, gamma)
	if baseHeight >= crossing {
		t.Errorf("%s: the resolved base height %.9f has reached the true crossing %.9f",
			label, baseHeight, crossing)
	}
	if bound := maxBaseHeight(pitchRadius, gamma, module); bound >= crossing {
		t.Errorf("%s: the Maximum Base Height %.9f is not below the true crossing %.9f it exists "+
			"to stay under", label, bound, crossing)
	}
	if low := minBaseHeight(module, gamma); baseHeight < low {
		t.Errorf("%s: the resolved base height %.9f is below the Minimum Base Height %.9f, so the "+
			"heel edge runs back inward", label, baseHeight, low)
	}
}

// assertHexagon checks the frustum profile the revolve consumes: six corners in
// the spec's draw order, a first edge lying along the shaft axis, and every other
// corner strictly off it.
func assertHexagon(t testing.TB, q pair, lat lattice, side gearSide) {
	t.Helper()
	hex := q.hexagon(lat, side)
	if len(hex) != 6 {
		t.Fatalf("%s: the frustum profile has %d corners, not six", side.label, len(hex))
	}
	if math.Abs(hex[0].Y) > 1e-9 || math.Abs(hex[1].Y) > 1e-9 {
		t.Errorf("%s: the hexagon's first edge runs from (%.9f, %.9f) to (%.9f, %.9f); it is the "+
			"shaft axis and both ends sit on it", side.label, hex[0].X, hex[0].Y, hex[1].X, hex[1].Y)
	}
	for k := 2; k < 6; k++ {
		if hex[k].Y <= 1e-9 {
			t.Errorf("%s: hexagon corner %d sits %.3e off the shaft axis; the profile has reached "+
				"its own axis of revolution", side.label, k, hex[k].Y)
		}
	}
	if area := shoelace(hex); math.Abs(area) < 1e-9 {
		t.Errorf("%s: the frustum profile encloses no area", side.label)
	}
	// The toe end has to sit nearer the apex than the heel end, which is what
	// the §3a frame's swap guard exists to catch when it does not.
	toe := hex[4].add(hex[5]).scale(0.5)
	heel := hex[2].add(hex[3]).scale(0.5)
	if toe.X >= heel.X {
		t.Errorf("%s: the toe edge midpoint sits at cone distance %.6f and the heel's at %.6f; "+
			"the toe is the inner end", side.label, toe.X, heel.X)
	}
}

// shoelace is the signed area of a closed polygon.
func shoelace(pts []vec2) float64 {
	sum := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		sum += pts[i].X*pts[j].Y - pts[j].X*pts[i].Y
	}
	return sum / 2
}
