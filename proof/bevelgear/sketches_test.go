package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// keyGearSide selects which member of the pair a per-gear step is run for:
// 0 is the pinion (built first), 1 the driving gear.
const keyGearSide = "gearSide"

// sideOf returns the gear a per-gear case names, and its lattice geometry in
// that gear's own (along the shaft, radial) frame.
func sideOf(d design, p map[string]float64) (gear, gearFrame) {
	g := d.Pinion
	if p[keyGearSide] != 0 {
		g = d.Driving
	}
	return g, gearLattice(g, d.Module, d.R, d.FaceWidth, d.ToothSpacing)
}

// ---------------------------------------------------------------------------
// Section 1 — the Anchor sketch.
// ---------------------------------------------------------------------------

// stepAnchorSketch draws the Anchor sketch: the user's centre point projected
// onto the user-selected target plane, and one reference line through it.
//
// The sketch has to end fully constrained with nothing redundant, which is the
// whole point of the step: the line is what section 2 measures every direction
// against, and a free rotation there would let the gear turn between rebuilds.
//
// Two substitutions, both arity differences:
//
//   - Fusion applies BOTH addCoincident(projectedCentre, anchorLine) and
//     addMidPoint(projectedCentre, anchorLine). The engine's Midpoint already
//     carries two rows — the point IS the midpoint, not merely on the line —
//     so the coincident's single point-on-line row is implied and the engine
//     reports it redundant. The proof keeps the midpoint.
//   - Fusion's aligned distance dimension is unsigned, so the line's two ends
//     can swap: a second configuration the seed resolves and the probe would
//     report. Because addHorizontal has already made the line horizontal in
//     the sketch's own frame, its length IS its signed x span, so the proof
//     writes the dimension as that signed span. The cost is that the proof
//     cannot show the aligned orientation is the right one to have asked
//     Fusion for; the step list carries that separately.
//
// <!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	_ = newDesign(t, p) // the anchor sketch depends on no numeric input

	proofkit.Step(t, "project the user's Center Point onto the target plane")
	centre := s.CreateReferencePoint(0, 0, "user Center Point")
	centre.SetName("projected centre")

	// Seed the two endpoints at exactly +-0.5 cm from the projected centre
	// along sketch-local X, so the seeded length is 10 mm.
	proofkit.Step(t, "the Anchor Line, seeded at +-5 mm about the projected centre")
	start := s.CreatePoint(-5, 0)
	end := s.CreatePoint(5, 0)
	start.SetName("anchor line start")
	end.SetName("anchor line end")
	line := s.CreateLine(start, end)
	line.SetName("Anchor Line")

	s.AddConstraint(sketch.NewMidpoint(centre, line))
	span := sketch.NewHorizontalDistance(start, end, 10)
	if err := span.SetValue(units.Millimeters(10)); err != nil {
		t.Fatalf("anchor line length: %v", err)
	}
	s.AddConstraint(span)
	// Sketch-local Horizontal, never a world-axis lock: the target plane can be
	// tilted any way at all and a world lock would mis-orient the figure.
	s.AddConstraint(sketch.NewHorizontal(line))

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("anchor sketch: solve: %v", err)
	}
	requireClose(t, line.Length(), 10, slackTol, "Anchor Line length")
	mid := solved(start).add(solved(end)).scale(0.5)
	requireCloseVec(t, mid, solved(centre), slackTol, "projected centre bisects the Anchor Line")
}

// ---------------------------------------------------------------------------
// The per-gear Profile sketch — the hexagon the gear body is revolved from.
// ---------------------------------------------------------------------------

// hexagonVertexNames are the section 2 vertices each gear's Profile sketch
// recreates, in the draw order the substitution table fixes.
func hexagonVertexNames(g gear) []string {
	if g.Label == "Driving" {
		return []string{"B", "I", "J", "D", "O", "P"}
	}
	return []string{"A", "G", "H", "C", "M", "N"}
}

// drawFrustumHexagon is the recreate-share-fix recipe: the six section 2
// vertices recreated as new points at their exact positions, the closed hexagon
// drawn SHARING those points, and the endpoints fixed only AFTER the lines
// exist. The order is the whole recipe.
//
// The sketch frame is the axial plane: +X is the radial distance from this
// gear's shaft axis and +Y is the distance from the Apex along it, so the shaft
// axis is the line x = 0.
func drawFrustumHexagon(t testing.TB, s *sketch.Sketch, g gear, f gearFrame) (
	[]*sketch.Point, []*sketch.Line, []string) {
	t.Helper()
	proofkit.Step(t, "recreate the six %s vertices", g.Label)
	verts := f.hexagon()
	names := hexagonVertexNames(g)
	pts := make([]*sketch.Point, len(verts))
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.Y, v.X)
		pts[i].SetName(names[i])
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		lines[i].SetName(names[i] + "->" + names[(i+1)%len(names)])
	}

	proofkit.Step(t, "fix the endpoints now that the lines exist")
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s frustum hexagon: solve: %v", g.Label, err)
	}
	return pts, lines, names
}

// requireFrustumProfile checks that the hexagon is the one closed all-line loop
// the revolve takes without filtering, and that it never crosses the axis it is
// spun about.
func requireFrustumProfile(t testing.TB, s *sketch.Sketch, g gear,
	pts []*sketch.Point, lines []*sketch.Line, names []string) {
	t.Helper()
	// The hexagon's FIRST edge is the shaft axis every body operation uses —
	// the revolve, the pattern, the bore plane and the meshing rotation — so
	// both its endpoints must sit exactly on that axis.
	requireClose(t, pts[0].X(), 0, tightTol, "%s shaft edge start on the axis", g.Label)
	requireClose(t, pts[1].X(), 0, tightTol, "%s shaft edge end on the axis", g.Label)
	requireClose(t, lines[0].Length(), g.BaseHeight, slackTol,
		"%s shaft edge length equals the resolved base height", g.Label)

	// Every other vertex is strictly off the axis and on ONE side of it. A
	// profile that crosses its axis of revolution aborts the revolve with
	// ASM_WIRE_X_AXIS, and the Maximum Face Width and Maximum Base Height caps
	// are exactly what keeps it from doing so.
	for i := 2; i < len(pts); i++ {
		if pts[i].X() <= 0 {
			t.Fatalf("%s hexagon vertex %s is at radial %.6f mm, on or across the shaft axis",
				g.Label, names[i], pts[i].X())
		}
	}

	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("%s Profile sketch holds %d closed regions, want exactly 1", g.Label, len(profiles))
	}
	prof := profiles[0]
	if !prof.Valid || prof.SelfIntersecting {
		t.Fatalf("%s hexagon is not an extrudable region: valid=%v selfIntersecting=%v",
			g.Label, prof.Valid, prof.SelfIntersecting)
	}
	if len(prof.Entities) != 6 {
		t.Fatalf("%s hexagon boundary holds %d curves, want 6 lines", g.Label, len(prof.Entities))
	}
	for _, e := range prof.Entities {
		if _, ok := e.(*sketch.Line); !ok {
			t.Fatalf("%s hexagon boundary holds a %T, want only lines", g.Label, e)
		}
	}
	if prof.Area <= 0 {
		t.Fatalf("%s hexagon has area %.6f mm^2", g.Label, prof.Area)
	}
}

// stepGearProfileHexagon draws one gear's Profile sketch on the axial (Gear
// Profiles) plane: the six section 2 vertices recreated as new points at their
// exact positions, the closed hexagon drawn SHARING those points, and the
// endpoints fixed only AFTER the lines exist.
//
// That order is the whole recipe. A projected point is a reference, not a
// fixed point, so a sketch hung off projections reports under-constrained even
// though every point is already in the right place; and fixing a bare point
// BEFORE it is consumed as a line endpoint does not leave the sketch fully
// constrained either. Recreate, share, then fix.
//
// The sketch holds exactly one closed loop, which is what lets the revolve take
// its single profile without filtering. The proof asserts that count, the six
// line curves, a positive area, and — the fact the revolve depends on — that
// the loop never crosses the shaft axis it is spun about.
//
// <!-- proof-run: proofkit.RunParallel(profileCases, stepGearProfileHexagon) -->
func stepGearProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)

	pts, lines, names := drawFrustumHexagon(t, s, g, f)

	requireFrustumProfile(t, s, g, pts, lines, names)
	// The revolved frustum's volume follows from the profile by Pappus, so a
	// positive area is what makes the revolve meaningful; the solid step
	// measures the body itself.
}

// ---------------------------------------------------------------------------
// The Bore sketch.
// ---------------------------------------------------------------------------

// stepBoreSketch draws one gear's Bore sketch on the plane rooted at the start
// of its shaft-axis edge, so the sketch origin already lies on the axis.
//
// A circle's centre is FREE even when it is created at the origin: the
// creation call does not reuse the sketch's own origin point. Fix the centre
// and dimension the diameter — do not coincident the centre to the origin,
// which has been observed to fail the solve outright on a
// setByDistanceOnPath plane.
//
// <!-- proof-run: proofkit.RunParallel(boreCases, stepBoreSketch) -->
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	g, _ := sideOf(d, p)

	if !d.BoreEnable {
		if g.Bore != 0 {
			t.Fatalf("%s bore diameter is %.6f mm with Enable Bore unchecked", g.Label, g.Bore)
		}
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so this step draws no sketch at all")
	}

	proofkit.Step(t, "%s bore circle, centre fixed at the sketch origin", g.Label)
	centre := s.CreatePoint(0, 0)
	centre.SetName(g.Label + " bore centre")
	s.Fix(centre)
	circle := s.CreateCircle(centre, g.Bore/2)
	circle.SetName(g.Label + " Bore")
	dim := sketch.NewDiameter(circle, g.Bore)
	if err := dim.SetValue(units.Millimeters(g.Bore)); err != nil {
		t.Fatalf("bore diameter: %v", err)
	}
	s.AddConstraint(dim)

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s Bore: solve: %v", g.Label, err)
	}
	requireClose(t, 2*circle.R(), g.Bore, slackTol, "%s bore diameter", g.Label)

	// "0 means auto" resolves to this gear's Pitch Diameter / 4.
	if p[keyDrivingBore] == 0 && g.Label == "Driving" {
		requireClose(t, g.Bore, g.PitchDiameter/4, tightTol, "driving auto bore diameter")
	}
	if p[keyPinionBore] == 0 && g.Label == "Pinion" {
		requireClose(t, g.Bore, g.PitchDiameter/4, tightTol, "pinion auto bore diameter")
	}

	// The bore has to pierce the body, so it must be smaller than the smallest
	// radius the frustum reaches: the toe end's root radius.
	f := gearLattice(g, d.Module, d.R, d.FaceWidth, d.ToothSpacing)
	if g.Bore/2 >= f.Toe.Y {
		t.Fatalf("%s bore radius %.6f mm reaches the toe root radius %.6f mm",
			g.Label, g.Bore/2, f.Toe.Y)
	}
}

// ---------------------------------------------------------------------------
// Section 3a step C — the 2-D tooth trace.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Section 3a step C — the cone-element sketch.
// ---------------------------------------------------------------------------

// stepConeElementSketch draws the `{gear} Cone Element` sketch: one
// construction line from the Apex out along the root cone element, which is
// what plane_by_angle then rotates the axial plane about to make the Trace
// Plane.
//
// The line is drawn in the axial (Gear Profiles) plane, whose own frame here is
// (radial, along): the Apex is the origin and the shaft axis is the line
// radial = 0. Both of its points are placed from raw coordinates and neither is
// dimensioned, so the sketch is fully constrained by grounding alone — the
// spiral build's auxiliary sketches are exempt from the full-constraint gate,
// but this one passes it anyway, which is worth knowing rather than assuming.
//
// What the assertions pin is the one thing the derivation's own debugging
// checklist puts first: the tangent plane must be built on the ROOT CONE
// ELEMENT Apex->C, not on Apex->Apex2 and not on the shaft axis. Built on
// either of those the whole spiral frame is skewed, and nothing downstream
// reports it.
//
// <!-- proof-run: proofkit.RunParallel(coneElementCases, stepConeElementSketch) -->
func stepConeElementSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	_, rHeel := coneDistances(f)
	coneVec := f.Ded.sub(f.Apex).unit()

	proofkit.Step(t, "%s cone element, Apex to Apex + R_heel * coneVec", g.Label)
	// (u, v) = (radial, along), so a lattice point (along, radial) maps to
	// (p.Y, p.X).
	apex := s.CreatePoint(f.Apex.Y, f.Apex.X)
	far := f.Apex.add(coneVec.scale(rHeel))
	end := s.CreatePoint(far.Y, far.X)
	apex.SetName("Apex")
	end.SetName("cone element far end")
	line := s.CreateLine(apex, end)
	line.SetName(g.Label + " Cone Element")
	line.SetConstruction(true)
	s.Fix(apex)
	s.Fix(end)

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s Cone Element: solve: %v", g.Label, err)
	}

	requireClose(t, math.Hypot(apex.X(), apex.Y()), 0, tightTol,
		"%s cone element starts at the Apex", g.Label)
	requireClose(t, line.Length(), rHeel, slackTol,
		"%s cone element reaches the heel cone distance", g.Label)

	// Its direction IS the root cone element: the unit Apex->C for the pinion,
	// Apex->D for the driving gear.
	dir := v2(end.X()-apex.X(), end.Y()-apex.Y()).unit()
	requireCloseVec(t, dir, v2(coneVec.Y, coneVec.X), slackTol,
		"%s cone element direction is the root cone element", g.Label)

	// And it is neither of the two lines the checklist warns against. The angle
	// to the shaft axis is the ROOT cone angle, smaller than the pitch cone
	// angle by the dedendum angle; the angle to the pitch line Apex->Apex2 is
	// that dedendum angle, and both are strictly positive.
	axis := v2(0, 1) // the shaft axis in this frame: radial 0, along +1
	rootAngle := math.Acos(dir.dot(axis))
	requireClose(t, rootAngle, math.Atan2(f.Ded.Y, f.Ded.X), slackTol,
		"%s root cone angle", g.Label)
	if !(rootAngle > 0) {
		t.Fatalf("%s cone element lies along the shaft axis", g.Label)
	}
	pitch := v2(f.Apex2.Y, f.Apex2.X).sub(v2(f.Apex.Y, f.Apex.X)).unit()
	dedendumAngle := math.Acos(pitch.dot(axis)) - rootAngle
	if !(dedendumAngle > 1e-6) {
		t.Fatalf("%s cone element lies along Apex->Apex2: dedendum angle %.9f rad",
			g.Label, dedendumAngle)
	}
	// The dedendum angle is atan(1.25*Module / R), which is what separates the
	// root cone this trace is laid on from the canonical PITCH cone.
	requireClose(t, dedendumAngle, math.Atan2(1.25*d.Module, d.R), slackTol,
		"%s dedendum angle between the root cone and the pitch cone", g.Label)
}

// traceSamples is how finely the cutter arc is sampled when it is substituted
// for the engine's arc entity. It only has to be dense enough to measure the
// tangent at the mean point.
const traceSamples = 61

// stepSpiralTrace draws the `{gear} 2D Tooth Trace` sketch: the cutter circle
// and the trace arc that is the tooth's lengthwise centreline, in the tangent
// plane whose origin is the Apex, whose x is the cone element and whose y is
// circumferential.
//
// SUBSTITUTION, and what it costs. Fusion draws the trace as a three-point arc
// through the toe endpoint, the mean point and the heel endpoint, with its
// centre coincident to the cutter circle's centre and a radius dimension, and
// deliberately leaves that sketch with free degrees of freedom. The harness
// gates every sketch, so the proof cannot leave it free; and it cannot pin the
// engine's arc either, because an Arc carries an internal
// "start and end are equidistant from the centre" row that becomes either
// redundant (when all three points are grounded) or ambiguous (when the centre
// is free, since the two sides of the chord both solve). So the arc entity is
// substituted by a chord-wise sample of the GENUINE cutter circle, drawn as a
// fitted spline through grounded points, alongside the real cutter circle with
// its own fixed centre and diameter dimension.
//
// The cost is that the proof does not exercise Fusion's three-point-arc
// construction or its centre coincidence. Everything the derivation actually
// asserts is still checked, and checked on the same numbers the twist is
// computed from: the samples lie exactly on the cutter circle, the curve passes
// through the mean point, its tangent there makes the mean spiral angle with
// the cone element, its ends sit on the toe and heel apex circles, and the
// opposite hand is the exact mirror across the cone element.
//
// <!-- proof-run: proofkit.RunParallel(traceCases, stepSpiralTrace) -->
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	if d.Psi <= 0 {
		proofkit.Unmodelled(t, "Mean Spiral Angle is 0, so this is a STRAIGHT bevel: the "+
			"tooth-body hook returns cut_conical_ends and no trace geometry is built at all")
	}

	// The pinion is the member whose hand is negated, so proving it proves the
	// sign rule as well as the arc.
	g := d.Pinion
	f := gearLattice(g, d.Module, d.R, d.FaceWidth, d.ToothSpacing)
	rToe, rHeel := coneDistances(f)
	tf := newTraceFrame(d, g, rToe, rHeel)

	proofkit.Step(t, "the apex circles at R_toe - 6%% span and R_heel + 6%% span")
	apex := s.CreateReferencePoint(0, 0, "the shared Apex")
	apex.SetName("Apex")
	lo := tf.RToe - 0.06*tf.Span
	hi := tf.RHeel + 0.06*tf.Span
	toeCircle, err := s.CreateReferenceCircle(apex, lo, "toe apex circle")
	if err != nil {
		t.Fatalf("toe apex circle: %v", err)
	}
	toeCircle.SetName("toe apex circle")
	heelCircle, err := s.CreateReferenceCircle(apex, hi, "heel apex circle")
	if err != nil {
		t.Fatalf("heel apex circle: %v", err)
	}
	heelCircle.SetName("heel apex circle")

	proofkit.Step(t, "the cutter circle, centre fixed, diameter dimensioned to 2*r_c")
	centre := s.CreatePoint(tf.Centre.X, tf.Centre.Y)
	centre.SetName("cutter circle centre")
	s.Fix(centre)
	cutter := s.CreateCircle(centre, tf.CutterRadius)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	dim := sketch.NewDiameter(cutter, 2*tf.CutterRadius)
	if err := dim.SetValue(units.Millimeters(2 * tf.CutterRadius)); err != nil {
		t.Fatalf("cutter diameter: %v", err)
	}
	s.AddConstraint(dim)

	proofkit.Step(t, "the trace, sampled along the genuine cutter circle from toe to heel")
	samples := traceArcSamples(tf, traceSamples)
	fit := make([]*sketch.Point, len(samples))
	for i, q := range samples {
		fit[i] = s.CreatePoint(q.X, q.Y)
		s.Fix(fit[i])
	}
	fit[0].SetName("trace toe endpoint")
	fit[len(fit)-1].SetName("trace heel endpoint")
	trace, err := s.CreateFitSpline(fit...)
	if err != nil {
		t.Fatalf("trace curve: %v", err)
	}
	trace.SetName("tooth trace")
	trace.SetConstruction(true)

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("2D Tooth Trace: solve: %v", err)
	}

	// Invariant 2: the arc's radius is r_c everywhere — it is one circle.
	requireClose(t, 2*cutter.R(), 2*tf.CutterRadius, slackTol, "cutter circle diameter")
	for i, q := range samples {
		requireClose(t, q.distanceTo(tf.Centre), tf.CutterRadius, tightTol,
			"trace sample %d lies on the cutter circle", i)
	}
	// Invariant 3: the trace passes through the mean point, and the centre is
	// exactly r_c from it.
	mean := v2(tf.RMean, 0)
	requireClose(t, tf.Centre.distanceTo(mean), tf.CutterRadius, tightTol,
		"the cutter centre is r_c from the mean point")
	nearest := math.Inf(1)
	for _, q := range samples {
		nearest = math.Min(nearest, q.distanceTo(mean))
	}
	if nearest > tf.Span/float64(traceSamples) {
		t.Fatalf("the trace misses the mean point by %.6g mm", nearest)
	}
	// Invariant 4: the spiral angle is realised AT the mean point. The tangent
	// to a circle is perpendicular to its radius, so the angle between the
	// tangent at the mean point and the cone element (the x axis) is psi.
	radial := mean.sub(tf.Centre).unit()
	tangent := radial.leftNormal()
	psi := math.Abs(math.Atan2(math.Abs(tangent.Y), math.Abs(tangent.X)))
	requireClose(t, psi, d.Psi, tightTol, "spiral angle at the mean point")

	// Invariant 6: the ends sit on the toe and heel apex circles, on the branch
	// the mean point is on. A "toe" point at the wrong radius means the far
	// intersection branch was kept.
	requireClose(t, tf.Toe2D.len(), lo, tightTol, "trace toe endpoint cone distance")
	requireClose(t, tf.Heel2D.len(), hi, tightTol, "trace heel endpoint cone distance")

	// Invariant 5: swapping the hand reflects the whole construction across the
	// cone element and changes nothing else, so an equal-tooth pair's two
	// traces are exact mirror images.
	mirrored := newTraceFrame(design{
		Module: d.Module, Sigma: d.Sigma, R: d.R, ConeDistance: d.ConeDistance,
		FaceWidth: d.FaceWidth, ToothSpacing: d.ToothSpacing, Psi: d.Psi,
		HandSign: -d.HandSign, CutterRadius: d.CutterRadius,
		Pinion: d.Pinion, Driving: d.Driving,
	}, g, rToe, rHeel)
	requireCloseVec(t, mirrored.Centre, v2(tf.Centre.X, -tf.Centre.Y), tightTol,
		"the opposite hand mirrors the cutter centre across the cone element")
	requireCloseVec(t, mirrored.Toe2D, v2(tf.Toe2D.X, -tf.Toe2D.Y), tightTol,
		"the opposite hand mirrors the toe endpoint")
	requireCloseVec(t, mirrored.Heel2D, v2(tf.Heel2D.X, -tf.Heel2D.Y), tightTol,
		"the opposite hand mirrors the heel endpoint")
	requireClose(t, mirrored.Total, tf.Total, tightTol,
		"the opposite hand gives the same twist magnitude")

	// The hand sign belongs on the cos (y) term. Mirroring about x = R_mean
	// instead — the bug this checks for — would leave Cy unchanged and move Cx,
	// so pinning the sign of Cy and the value of Cx is what separates the two.
	requireClose(t, tf.Centre.X, tf.RMean-tf.CutterRadius*math.Sin(d.Psi), tightTol,
		"cutter centre x is R_mean - r_c sin psi, with no hand sign on it")
	if tf.Centre.Y*tf.HandSign <= 0 {
		t.Fatalf("cutter centre y %.6f does not carry the hand sign %+.0f", tf.Centre.Y, tf.HandSign)
	}

	// The twist law: total = |phi_crown| / sin(gamma), with gamma the PITCH
	// cone angle from section 2 and never acos(coneVec . axisDir), which is the
	// root cone angle and inflates the twist.
	requireClose(t, tf.Total, math.Abs(tf.PhiCrown)/math.Sin(g.Gamma), tightTol,
		"toe-to-heel twist from the crown-gear roll ratio")
	if tf.Total <= 0 {
		t.Fatalf("a spiral bevel with psi = %.4f rad came out with zero twist", d.Psi)
	}
}

// coneDistances is what section 3a step A derives from the caller's four
// hand-off points: the cone distances of the TOE edge midpoint and the HEEL
// edge midpoint. They are two different edges — passing the two ends of one
// edge collapses the span to zero and inverts the spiral.
func coneDistances(f gearFrame) (rToe, rHeel float64) {
	coneVec := f.Ded.sub(f.Apex).unit()
	toeMid := f.Toe.add(f.ToeIn).scale(0.5)
	heelMid := f.Ded.add(f.Heel).scale(0.5)
	return toeMid.sub(f.Apex).dot(coneVec), heelMid.sub(f.Apex).dot(coneVec)
}

// traceArcSamples walks the cutter circle from the toe endpoint to the heel
// endpoint along the short way round, so the sampled curve is the arc the
// derivation keeps and not its complement.
func traceArcSamples(tf traceFrame, n int) []vec2 {
	a0 := math.Atan2(tf.Toe2D.Y-tf.Centre.Y, tf.Toe2D.X-tf.Centre.X)
	a1 := math.Atan2(tf.Heel2D.Y-tf.Centre.Y, tf.Heel2D.X-tf.Centre.X)
	sweep := a1 - a0
	for sweep > math.Pi {
		sweep -= 2 * math.Pi
	}
	for sweep < -math.Pi {
		sweep += 2 * math.Pi
	}
	out := make([]vec2, n)
	for i := range n {
		a := a0 + sweep*float64(i)/float64(n-1)
		out[i] = tf.Centre.add(v2(math.Cos(a), math.Sin(a)).scale(tf.CutterRadius))
	}
	return out
}
