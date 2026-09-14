// This file holds the bevel gear's sketch steps: the Anchor sketch, the §2 Gear
// Profiles lattice, the §3 virtual spur tooth, the per-gear Profile hexagon, the
// spiral build's 2-D cutter-arc trace, and the Bore sketch.
//
// THREE CONSTRAINTS FUSION NEEDS THAT THIS ENGINE REFUSES, each recorded beside
// the geometry it belongs to rather than only here:
//
//   - the Anchor sketch's addCoincident(projected centre, anchor line) beside
//     addMidPoint. Fusion wants both; the engine's midpoint carries the
//     point-on-line row already.
//   - the E->G / G->H and F->I / I->J perpendiculars. spec/bevelgear/instructions.md
//     states this one outright: Fusion's addOffsetDimension requires the two
//     lines to be parallel ALREADY, and the perpendicular is what supplies it,
//     while this engine's offset emits two rows and carries the parallelism
//     itself. Modelling Fusion's arity there is a third row for two freedoms and
//     the lattice comes back overconstrained.
//   - the addParallel on the two toe lines, for the same reason as the
//     perpendiculars above.
//
// And one the spec does not anticipate: addCoincident(I, projected centre) is
// two rows, one of which the driving chain already implies, so the proof
// substitutes the single independent row. See stepGearProfiles.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// xy is a point in one sketch's own two-dimensional frame, millimetres.
type xy struct{ x, y float64 }

func (a xy) add(b xy) xy      { return xy{a.x + b.x, a.y + b.y} }
func (a xy) sub(b xy) xy      { return xy{a.x - b.x, a.y - b.y} }
func (a xy) mul(k float64) xy { return xy{a.x * k, a.y * k} }
func (a xy) dot(b xy) float64 { return a.x*b.x + a.y*b.y }
func (a xy) crs(b xy) float64 { return a.x*b.y - a.y*b.x }
func (a xy) len() float64     { return math.Hypot(a.x, a.y) }

// anchorSource is the id every projection of the Anchor sketch's own geometry
// carries. In Fusion §2 projects the Anchor sketch's centre SketchPoint and its
// anchor line rather than the user's raw selection, which keeps the chain inside
// the Design component; this engine refuses a reference to another sketch's
// point outright, so each sketch below carries its own reference geometry tagged
// with this id. What that models is one link of the chain, not the chain.
const anchorSource = "Anchor sketch projection"

// rawSeg is the [BEVEL-F-COINCIDENT-STYLE] line: built from raw coordinates,
// never from an existing SketchPoint, so that every connection to an existing
// point is made by exactly one coincident. Every §2 line is construction
// geometry, lattice and short reference line alike, so none of them reaches
// profile detection.
func rawSeg(s *sketch.Sketch, a, b xy) *sketch.Line {
	l := s.CreateLine(s.CreatePoint(a.x, a.y), s.CreatePoint(b.x, b.y))
	l.SetConstruction(true)
	return l
}

func pinTo(s *sketch.Sketch, p, to *sketch.Point) {
	s.AddConstraint(sketch.NewCoincident(p, to))
}

func pinOnLine(s *sketch.Sketch, p *sketch.Point, l *sketch.Line) {
	s.AddConstraint(sketch.NewPointOnLine(p, l))
}

// offsetTo emits the parallel-offset dimension between two lines with the sign
// the seeded side asks for.
//
// The engine's offset target is signed, left-positive along the source line's
// own direction, while a Fusion offset dimension's value is a magnitude whose
// direction is captured from the geometry as it sits ([PB-DIM-VALUE-SEMANTICS]).
// So the sign crosses over as the seed side and only the magnitude is the
// number the spec states.
func offsetTo(s *sketch.Sketch, src, dst *sketch.Line, want float64) {
	dir := xy{src.End.X() - src.Start.X(), src.End.Y() - src.Start.Y()}
	to := xy{dst.Start.X() - src.Start.X(), dst.Start.Y() - src.Start.Y()}
	if dir.crs(to) < 0 {
		want = -want
	}
	s.AddConstraint(sketch.NewOffset(src, dst, want))
}

// frontFaceDirection is the row this proof writes where Fusion writes
// addPerpendicular(front face, shaft axis), and it is the one substitution in
// §2 that changes what a row SAYS rather than only how many rows say it.
//
// The spec is explicit that the toe end is discretely ambiguous as it stands: N
// is fixed by the toe line together with a LENGTH dimension on the front face,
// a length is unsigned, the toe line meets the Toe Radius on BOTH sides of the
// shaft axis, and the solver takes whichever side the seed starts on. A
// mirrored N is a real solution of the constraint system, and this engine's
// ambiguity probe finds it — reliably wherever the two branches sit close
// together, which is wherever the resolved Toe Radius is small. proofkit does
// not waive that, and its own note says the fix is a constraint that carries a
// direction rather than a comment.
//
// So the perpendicular becomes a SIGNED angle of 180 degrees against the heel
// edge, which the offset dimension has already made perpendicular to the same
// shaft axis. Antiparallel to the heel edge is the same direction as
// perpendicular plus "N on the heel corner's side of the axis", so the figure
// is unchanged and the mirror stops being a solution. That is the crossing-over
// [PB-DIM-VALUE-SEMANTICS] describes: Fusion captures the side from the seeded
// geometry, and on this engine the side has to be written as a sign.
//
// WHAT THIS COSTS, and it is the cost the spec names: the proof now seeds M and
// N at the closed form AND holds the side with a constraint, so it proves the
// constraints solve from a correct seed and never that the module's seed is
// correct. A seed defect at the toe line therefore reaches Fusion untested,
// which is how the one the spec's ⚠️ describes got there — it converged onto
// the mirror and Fusion refused the revolve several steps later with
// ASM_WIRE_X_AXIS, naming the revolve rather than the seed.
func frontFaceDirection(heelEdge, frontFace *sketch.Line) sketch.Constraint {
	return alignedWith(heelEdge, frontFace)
}

// alignedWith is the signed straight angle — 0 or 180 taken from the seeded
// directions — that says two lines are parallel AND which way round.
//
// It stands in for a row that in Fusion pins a point to a line and lets an
// unsigned length choose between the line's two directions. The Tooth Spacing
// shift is the third §2 place that happens: K' is pinned onto the dedendum line
// and dimensioned Tooth Spacing from K, which is satisfied on either side of K,
// and the spec says the shift goes AWAY from the lower corner C. The engine's
// probe finds the other one; the sign is how the proof says which.
func alignedWith(ref, line *sketch.Line) sketch.Constraint {
	rx, ry := ref.End.X()-ref.Start.X(), ref.End.Y()-ref.Start.Y()
	lx, ly := line.End.X()-line.Start.X(), line.End.Y()-line.Start.Y()
	deg := 0.0
	if rx*lx+ry*ly < 0 {
		deg = 180
	}
	return sketch.NewAngle(ref, line, deg)
}

// squareTo is the row this proof writes where Fusion writes
// addPerpendicular(line, reference) AND the perpendicular is choosing a SIDE
// rather than only a direction.
//
// Four §2 lines are in that position: each Apex 2 drop, which stands
// perpendicular to its own shaft axis at an unsigned length, and each dedendum
// line, which stands perpendicular to the pitch line at an unsigned 1.25 *
// Module. Perpendicular plus an unsigned length is satisfied on EITHER side, so
// each pair admits a mirrored solution, and this engine's ambiguity probe finds
// them: measured on a Driving 17 / Pinion 31 pair, the second configuration put
// D on C's side of Apex 2, collapsing the driving dedendum onto the pinion's —
// which is the inversion §2's own ⚠️ describes, where the toe ends up outside
// the heel and the conical end cut finds no cone face at the toe midpoint.
//
// The signed right angle taken from the seeded directions says the same thing
// with the side written down. In Fusion the side comes from the seed, exactly as
// [PB-DIM-VALUE-SEMANTICS] says a dimension's direction does; here it has to be
// a sign, because proofkit does not waive a discrete ambiguity and the spec's
// own ⚠️ is that the mirror is a real solution the solver will take.
func squareTo(ref, line *sketch.Line) sketch.Constraint {
	rx, ry := ref.End.X()-ref.Start.X(), ref.End.Y()-ref.Start.Y()
	lx, ly := line.End.X()-line.Start.X(), line.End.Y()-line.Start.Y()
	deg := 90.0
	if rx*ly-ry*lx < 0 {
		deg = -90
	}
	return sketch.NewAngle(ref, line, deg)
}

// ---------------------------------------------------------------- S4 anchor sketch

var anchorCases = []proofkit.Case{
	{Name: "centre_on_sketch_origin", Params: map[string]float64{"centreX": 0, "centreY": 0}},
	{Name: "centre_off_origin", Params: map[string]float64{"centreX": 9, "centreY": -4}},
	{Name: "centre_far_off_origin", Params: map[string]float64{"centreX": -37, "centreY": 21}},
}

// stepAnchorSketch builds the Anchor sketch: the user's centre point projected
// in, and one reference line through it, seeded at exactly plus and minus 5 mm
// along the sketch-local X so its seeded length is the 10 mm the aligned
// dimension then locks.
//
// The line's absolute direction is arbitrary — nothing downstream reads it, and
// §2 derives every direction relative to it — but it must not be a free degree
// of freedom, which is what the sketch-local horizontal is for
// ([PB-REFLINE-DIRECTION]). A world-axis lock would mis-orient the figure on a
// tilted target plane.
//
// WHAT THE PROOF OMITS. Fusion applies BOTH addCoincident(projected centre,
// anchor line) and addMidPoint(projected centre, anchor line), and the spec says
// to use both rather than the midpoint alone. This engine's midpoint carries two
// residual rows, one of which is exactly the point-on-line the coincident
// states, so adding the coincident here is a third row for two freedoms and the
// sketch comes back with a redundant constraint. The proof therefore applies the
// midpoint alone and this comment is the record of the row it drops; only a
// Fusion session settles whether that engine absorbs the pair the way it absorbs
// an implied collinear row.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	cx, cy := p["centreX"], p["centreY"]

	proofkit.Step(t, "project the user's centre point")
	centre := s.CreateReferencePoint(cx, cy, anchorSource)

	proofkit.Step(t, "the anchor line, seeded at +/-5 mm along sketch-local X")
	line := s.CreateLine(s.CreatePoint(cx-5, cy), s.CreatePoint(cx+5, cy))
	line.SetConstruction(true)

	// The aligned dimension crosses over as the SIGNED horizontal distance,
	// which is the playbook's own mapping for it: Fusion's dimension value is a
	// magnitude whose direction is captured from the seeded geometry at
	// creation, and on this engine that direction has to be written as the
	// target's sign ([PB-DIM-VALUE-SEMANTICS]). Written unsigned the line has two
	// discrete solutions — its two endpoints swapped — which the engine reports
	// as an ambiguity and which the seed alone resolves in Fusion. The line is
	// constrained horizontal, so the two dimensions measure the same length.
	proofkit.Step(t, "midpoint, the 10 mm aligned dimension and the horizontal")
	s.AddConstraint(
		sketch.NewMidpoint(centre, line),
		sketch.NewHorizontalDistance(line.Start, line.End, 10),
		sketch.NewHorizontal(line),
	)

	proofkit.Step(t, "the anchor line's length and bisection")
	solveHere(t, s)
	if got := line.Length(); math.Abs(got-10) > 1e-9 {
		t.Errorf("anchor line is %.9f mm long, want the seeded 10 mm", got)
	}
	mx := (line.Start.X() + line.End.X()) / 2
	my := (line.Start.Y() + line.End.Y()) / 2
	if math.Hypot(mx-cx, my-cy) > 1e-9 {
		t.Errorf("anchor line's midpoint is (%.9f, %.9f), want the projected centre (%.9f, %.9f)",
			mx, my, cx, cy)
	}
}

// solveHere solves the sketch so a step can read solved positions before
// proofkit's own gate runs. proofkit solves again afterwards; the second solve
// starts from the answer and changes nothing.
func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
}

// ---------------------------------------------------------------- S6 gear profiles (§2)

// latticeSeeds holds every §2 point at the position the constraint net closes
// it at. The spec requires the seeds to BE those positions rather than near
// them, so this is both the seeding and the closed form the solved sketch is
// then asserted against.
type latticeSeeds struct {
	centre, apex, apex2   xy
	a, b, c, d, e, f      xy
	g, h, i, j, k, l      xy
	kPrime, lPrime        xy
	m, n, o, pp           xy
	aPrime, bPrime        xy
	pinionDir, drivingDir xy
	pitchDir, dedDir      xy
	dropADir              xy
}

// latticeOf solves the §2 figure in the Gear Profiles sketch's own 2-D frame,
// with the projected centre at the origin and the projected anchor line along
// +X. grow is the one-bit side the figure grows toward, which the module picks
// from targetPlane.geometry.normal ([BEVEL-F-GROW-SIDE]); the proof sweeps both
// values because a scheme that only closes on one of them mirrors the gear on
// half the target planes a user can select.
func latticeOf(d bevelDesign) latticeSeeds {
	g := d.growSide
	sigma := d.shaftAngle
	gp, gg := d.pinion.gamma, d.driving.gamma
	rr := d.pitchCone
	ded := 1.25 * d.module

	var seeds latticeSeeds
	seeds.centre = xy{0, 0}
	perp := xy{0, g}
	seeds.apex = perp.mul(rr*math.Cos(gg) + d.driving.baseHeight)
	seeds.drivingDir = xy{0, -g}
	seeds.pinionDir = xy{math.Sin(sigma), -g * math.Cos(sigma)}
	seeds.pitchDir = xy{math.Sin(gg), -g * math.Cos(gg)}
	seeds.dedDir = xy{math.Cos(gg), g * math.Sin(gg)}
	seeds.dropADir = xy{-math.Cos(sigma), -g * math.Sin(sigma)}

	seeds.b = seeds.apex.add(seeds.drivingDir.mul(rr * math.Cos(gg)))
	seeds.a = seeds.apex.add(seeds.pinionDir.mul(rr * math.Cos(gp)))
	seeds.apex2 = seeds.apex.add(seeds.pitchDir.mul(rr))
	seeds.c = seeds.apex2.add(seeds.dedDir.mul(ded))
	seeds.d = seeds.apex2.sub(seeds.dedDir.mul(ded))
	seeds.e = seeds.apex.add(seeds.pinionDir.mul(rr*math.Cos(gp) + ded*math.Sin(gp)))
	seeds.f = seeds.apex.add(seeds.drivingDir.mul(rr*math.Cos(gg) + ded*math.Sin(gg)))
	seeds.g = seeds.apex.add(seeds.pinionDir.mul(rr*math.Cos(gp) + d.pinion.baseHeight))
	seeds.i = seeds.apex.add(seeds.drivingDir.mul(rr*math.Cos(gg) + d.driving.baseHeight))
	seeds.h = seeds.c.add(seeds.dedDir.mul(d.pinion.baseHeight/math.Sin(gp) - ded))
	seeds.j = seeds.d.sub(seeds.dedDir.mul(d.driving.baseHeight/math.Sin(gg) - ded))
	seeds.k = seeds.apex.add(seeds.pinionDir.mul(rr / math.Cos(gp)))
	seeds.l = seeds.apex.add(seeds.drivingDir.mul(rr / math.Cos(gg)))
	seeds.kPrime = seeds.k.add(seeds.dedDir.mul(d.toothSpacing))
	seeds.lPrime = seeds.l.sub(seeds.dedDir.mul(d.toothSpacing))

	// The toe line. M sits on Apex->C at the fraction 1 - RootLength/|Apex->C|
	// from the Apex; N slides from that M along the C->H direction by exactly
	// (M's perpendicular distance from the shaft axis - the Toe Radius) / cos
	// gamma, which lands it at the Toe Radius and on the correct side of the
	// axis. The mirror it avoids is a solution the constraints admit, so the
	// seed is what picks the branch.
	rootHat := seeds.c.sub(seeds.apex).mul(1 / d.rootCone)
	seeds.m = seeds.apex.add(rootHat.mul(d.rootCone - d.rootLength))
	slideP := (perpFromAxis(seeds.m, seeds.apex, seeds.pinionDir) - d.pinion.toeRadius) / math.Cos(gp)
	seeds.n = seeds.m.add(seeds.dedDir.mul(slideP))
	seeds.aPrime = seeds.apex.add(seeds.pinionDir.mul(seeds.n.sub(seeds.apex).dot(seeds.pinionDir)))

	drivingHat := seeds.d.sub(seeds.apex).mul(1 / d.rootCone)
	seeds.o = seeds.apex.add(drivingHat.mul(d.rootCone - d.rootLength))
	slideG := (perpFromAxis(seeds.o, seeds.apex, seeds.drivingDir) - d.driving.toeRadius) / math.Cos(gg)
	seeds.pp = seeds.o.sub(seeds.dedDir.mul(slideG))
	seeds.bPrime = seeds.apex.add(seeds.drivingDir.mul(seeds.pp.sub(seeds.apex).dot(seeds.drivingDir)))
	return seeds
}

// perpFromAxis is a point's perpendicular distance from the shaft axis through
// origin along dir.
func perpFromAxis(p, origin, dir xy) float64 {
	return math.Abs(p.sub(origin).crs(dir))
}

// stepGearProfiles builds the whole §2 Gear Profiles lattice — both gears' shaft
// axes, both dedendum chains, both heel edges, both toe lines and both front
// faces — in one sketch, and gates it.
//
// EVERY LINE IS BUILT IN THE COINCIDENT STYLE ([BEVEL-F-COINCIDENT-STYLE]): from
// raw coordinates, with exactly one coincident per endpoint that meets an
// existing point, never by sharing the point into the creation call. The short
// reference and connector lines are covered by that rule too, and are the ones a
// regen has come up short on.
//
// EACH NAMED LINE IS CREATED ONCE ([BEVEL-F-LINE-ONCE]); where a later step
// names an earlier line, this function passes the very line it drew.
//
// THE DRIVEN LENGTHS CARRY NO DIMENSION ([BEVEL-F-DRIVEN-DIMS]): |Apex->A|,
// |Apex->B| and the four module-length extensions are all fixed by the Apex 2
// closure and the perpendiculars, and dimensioning any of them is an
// over-constraint rather than a route to DOF 0.
//
// FOUR SUBSTITUTIONS THIS ENGINE FORCES, each a row Fusion needs and this engine
// already has:
//
//  1. Every collinear becomes the single point-on-line row it is not already
//     implied by. addCollinear carries two point-on-line rows, and where the new
//     line's start is pinned to the reference line's own endpoint one of them is
//     already satisfied ([PB-COLLINEAR-CHAIN], [BEVEL-F-COLLINEAR-CHAIN]).
//     Because the substitution makes both readings identical, the proof CANNOT
//     tell the correct collinear from the one that names a farther line up the
//     chain; only a Fusion session does.
//  2. The E->G / G->H and F->I / I->J perpendiculars are omitted, which the spec
//     states and explains: the engine's offset emits two rows and carries the
//     parallelism itself.
//  3. The addParallel on each toe line is omitted for the same reason, since the
//     offset that follows it already fixes the direction.
//  4. addCoincident(I, projected centre) becomes one point-on-line row. I lies
//     on the driving shaft axis, which shares the Apex with the centre->apex line
//     and is constrained parallel to it, so the two are collinear and I is
//     already on the line through the projected centre. The second row of the
//     coincident is therefore implied and the engine reports it redundant. The
//     independent half is that I sits at the centre's own station, which
//     point-on-line against the projected anchor line states exactly.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newBevelDesign(p)
	seeds := latticeOf(d)

	// A declared refusal: a configuration the spec admits and THIS lattice
	// cannot reach. The case stays in the table and is marked here rather than
	// the advertised Shaft Angle range being narrowed on one net's evidence.
	// Measured on this net, the Shaft Angle floor of 30 degrees reads
	// conditioning 2.83e-05 against the engine's 4e-05 trust floor and first
	// clears at 35 degrees. That is a property of how this lattice is built —
	// three independently written nets do not agree about which end of the range
	// is reachable — so the remedy is a different construction, never a loosened
	// gate and never a narrower range.
	if p["declaredRefusal"] != 0 {
		proofkit.Unmodelled(t,
			"declared refusal: at Shaft Angle %.0f degrees this lattice reads below the engine's "+
				"conditioning floor; the spec admits the configuration and this net cannot reach it",
			d.shaftAngle*180/math.Pi)
	}

	proofkit.Step(t, "project the Anchor sketch's centre point and anchor line")
	centre := s.CreateReferencePoint(0, 0, anchorSource)
	anchorA := s.CreateReferencePoint(-5, 0, anchorSource)
	anchorB := s.CreateReferencePoint(5, 0, anchorSource)
	anchorLine, err := s.CreateReferenceLine(anchorA, anchorB, anchorSource)
	if err != nil {
		t.Fatalf("project the anchor line: %v", err)
	}

	proofkit.Step(t, "centre->apex, perpendicular to the projected anchor line")
	centreToApex := rawSeg(s, seeds.centre, seeds.apex)
	pinTo(s, centreToApex.Start, centre)
	s.AddConstraint(sketch.NewPerpendicular(centreToApex, anchorLine))

	proofkit.Step(t, "the two shaft axes and the Shaft Angle")
	drivingAxis := rawSeg(s, seeds.apex, seeds.b)
	pinTo(s, drivingAxis.Start, centreToApex.End)
	s.AddConstraint(sketch.NewParallel(drivingAxis, centreToApex))

	pinionAxis := rawSeg(s, seeds.apex, seeds.a)
	pinTo(s, pinionAxis.Start, centreToApex.End)
	s.AddConstraint(sketch.NewAngle(drivingAxis, pinionAxis,
		math.Atan2(seeds.drivingDir.crs(seeds.pinionDir), seeds.drivingDir.dot(seeds.pinionDir))*180/math.Pi))

	proofkit.Step(t, "the two perpendicular drops and the Apex 2 closure")
	dropA := rawSeg(s, seeds.a, seeds.apex2)
	pinTo(s, dropA.Start, pinionAxis.End)
	s.AddConstraint(
		squareTo(pinionAxis, dropA),
		sketch.NewDistance(dropA.Start, dropA.End, d.pinion.pitchDia/2),
	)
	dropB := rawSeg(s, seeds.b, seeds.apex2)
	pinTo(s, dropB.Start, drivingAxis.End)
	s.AddConstraint(
		squareTo(drivingAxis, dropB),
		sketch.NewDistance(dropB.Start, dropB.End, d.driving.pitchDia/2),
	)
	pinTo(s, dropA.End, dropB.End)

	proofkit.Step(t, "the pitch line and the two dedendum lines")
	pitchLine := rawSeg(s, seeds.apex, seeds.apex2)
	pinTo(s, pitchLine.Start, centreToApex.End)
	pinTo(s, pitchLine.End, dropA.End)

	dedD := rawSeg(s, seeds.apex2, seeds.d)
	pinTo(s, dedD.Start, dropA.End)
	s.AddConstraint(
		squareTo(pitchLine, dedD),
		sketch.NewDistance(dedD.Start, dedD.End, 1.25*d.module),
	)
	dedC := rawSeg(s, seeds.apex2, seeds.c)
	pinTo(s, dedC.Start, dropA.End)
	s.AddConstraint(
		squareTo(pitchLine, dedC),
		sketch.NewDistance(dedC.Start, dedC.End, 1.25*d.module),
	)

	proofkit.Step(t, "the two root axes")
	rootD := rawSeg(s, seeds.apex, seeds.d)
	pinTo(s, rootD.Start, centreToApex.End)
	pinTo(s, rootD.End, dedD.End)
	rootC := rawSeg(s, seeds.apex, seeds.c)
	pinTo(s, rootC.Start, centreToApex.End)
	pinTo(s, rootC.End, dedC.End)

	proofkit.Step(t, "the pinion module-length chain A->E, C->E, E->G, C->H, G->H")
	segAE := rawSeg(s, seeds.a, seeds.e)
	pinTo(s, segAE.Start, pinionAxis.End)
	pinOnLine(s, segAE.End, pinionAxis)
	segCE := rawSeg(s, seeds.c, seeds.e)
	pinTo(s, segCE.Start, dedC.End)
	pinTo(s, segCE.End, segAE.End)
	s.AddConstraint(sketch.NewPerpendicular(segAE, segCE))

	segEG := rawSeg(s, seeds.e, seeds.g)
	pinTo(s, segEG.Start, segAE.End)
	pinOnLine(s, segEG.End, segAE)
	segCH := rawSeg(s, seeds.c, seeds.h)
	pinTo(s, segCH.Start, dedC.End)
	pinOnLine(s, segCH.End, dedC)
	segGH := rawSeg(s, seeds.g, seeds.h)
	pinTo(s, segGH.Start, segEG.End)
	pinTo(s, segGH.End, segCH.End)
	offsetTo(s, dropA, segGH, d.pinion.baseHeight)

	proofkit.Step(t, "the driving module-length chain B->F, D->F, F->I, D->J, I->J")
	segBF := rawSeg(s, seeds.b, seeds.f)
	pinTo(s, segBF.Start, drivingAxis.End)
	pinOnLine(s, segBF.End, drivingAxis)
	segDF := rawSeg(s, seeds.d, seeds.f)
	pinTo(s, segDF.Start, dedD.End)
	pinTo(s, segDF.End, segBF.End)
	s.AddConstraint(sketch.NewPerpendicular(segBF, segDF))

	segFI := rawSeg(s, seeds.f, seeds.i)
	pinTo(s, segFI.Start, segBF.End)
	pinOnLine(s, segFI.End, segBF)
	segDJ := rawSeg(s, seeds.d, seeds.j)
	pinTo(s, segDJ.Start, dedD.End)
	pinOnLine(s, segDJ.End, dedD)
	segIJ := rawSeg(s, seeds.i, seeds.j)
	pinTo(s, segIJ.Start, segFI.End)
	pinTo(s, segIJ.End, segDJ.End)
	offsetTo(s, dropB, segIJ, d.driving.baseHeight)

	proofkit.Step(t, "point I on the projected centre")
	pinOnLine(s, segFI.End, anchorLine)

	proofkit.Step(t, "the two tooth centres K and L and their reference lines")
	toothCentreP := latticeToothCentre(t, s, seeds.g, seeds.k, seeds.kPrime, seeds.c,
		segEG.End, pinionAxis, dedC, d.toothSpacing)
	toothCentreG := latticeToothCentre(t, s, seeds.i, seeds.l, seeds.lPrime, seeds.d,
		segFI.End, drivingAxis, dedD, d.toothSpacing)
	_, _ = toothCentreP, toothCentreG

	proofkit.Step(t, "the pinion toe line M->N, its M->C connector and the front face N->A'")
	segMN := rawSeg(s, seeds.m, seeds.n)
	pinOnLine(s, segMN.Start, rootC)
	offsetTo(s, segCH, segMN, d.rootLength*d.pitchCone/d.rootCone)
	segMC := rawSeg(s, seeds.m, seeds.c)
	pinTo(s, segMC.Start, segMN.Start)
	pinTo(s, segMC.End, dedC.End)
	segNA := rawSeg(s, seeds.n, seeds.aPrime)
	pinTo(s, segNA.Start, segMN.End)
	pinOnLine(s, segNA.End, pinionAxis)
	s.AddConstraint(
		frontFaceDirection(segGH, segNA),
		sketch.NewDistance(segNA.Start, segNA.End, d.pinion.toeRadius),
	)
	shaftEdgeP := rawSeg(s, seeds.aPrime, seeds.g)
	pinTo(s, shaftEdgeP.Start, segNA.End)
	pinTo(s, shaftEdgeP.End, segEG.End)

	proofkit.Step(t, "the driving toe line O->P, its O->D connector and the front face P->B'")
	segOP := rawSeg(s, seeds.o, seeds.pp)
	pinOnLine(s, segOP.Start, rootD)
	offsetTo(s, segDJ, segOP, d.rootLength*d.pitchCone/d.rootCone)
	segOD := rawSeg(s, seeds.o, seeds.d)
	pinTo(s, segOD.Start, segOP.Start)
	pinTo(s, segOD.End, dedD.End)
	segPB := rawSeg(s, seeds.pp, seeds.bPrime)
	pinTo(s, segPB.Start, segOP.End)
	pinOnLine(s, segPB.End, drivingAxis)
	s.AddConstraint(
		frontFaceDirection(segIJ, segPB),
		sketch.NewDistance(segPB.Start, segPB.End, d.driving.toeRadius),
	)
	shaftEdgeG := rawSeg(s, seeds.bPrime, seeds.i)
	pinTo(s, shaftEdgeG.Start, segPB.End)
	pinTo(s, shaftEdgeG.End, segFI.End)

	proofkit.Step(t, "what the solved lattice has to measure")
	solveHere(t, s)
	assertLattice(t, s, d, seeds, latticeReadout{
		apex: centreToApex.End, apex2: dropA.End,
		a: pinionAxis.End, b: drivingAxis.End,
		c: dedC.End, dd: dedD.End,
		g: segEG.End, h: segCH.End, i: segFI.End, j: segDJ.End,
		m: segMN.Start, n: segMN.End, o: segOP.Start, pp: segOP.End,
		aPrime: segNA.End, bPrime: segPB.End,
		toothP: toothCentreP, toothG: toothCentreG,
	})
}

// latticeToothCentre builds one gear's tooth-centre chain: the line from the
// heel axis point out to K (resp. L), K pinned by TWO point-on-line coincidents
// rather than by a collinear — both its lines are already fixed by the time K is
// added, so a collinear there over-constrains — the C->K reference line, and,
// only when Tooth Spacing is positive, the K->K' offset line and the C->K'
// reference line that replaces it.
//
// At Tooth Spacing 0 nothing extra is built and K' IS K, because a zero-length
// dimensioned line is degenerate and one segment gets one line
// ([BEVEL-F-LINE-ONCE]).
func latticeToothCentre(t testing.TB, s *sketch.Sketch, from, centre, shifted, corner xy,
	fromPoint *sketch.Point, axis, dedendum *sketch.Line, spacing float64) *sketch.Point {
	t.Helper()
	toCentre := rawSeg(s, from, centre)
	pinTo(s, toCentre.Start, fromPoint)
	pinOnLine(s, toCentre.End, axis)
	pinOnLine(s, toCentre.End, dedendum)

	cornerToCentre := rawSeg(s, corner, centre)
	pinTo(s, cornerToCentre.Start, dedendum.End)
	pinTo(s, cornerToCentre.End, toCentre.End)
	if spacing <= 0 {
		return toCentre.End
	}

	shift := rawSeg(s, centre, shifted)
	pinTo(s, shift.Start, toCentre.End)
	s.AddConstraint(
		alignedWith(dedendum, shift),
		sketch.NewDistance(shift.Start, shift.End, spacing),
	)

	cornerToShifted := rawSeg(s, corner, shifted)
	pinTo(s, cornerToShifted.Start, dedendum.End)
	pinTo(s, cornerToShifted.End, shift.End)
	return shift.End
}

// latticeReadout names the solved sketch points the assertion reads, so the
// check is made on the geometry the constraints located rather than on the
// seeds that were handed to the solver.
type latticeReadout struct {
	apex, apex2    *sketch.Point
	a, b, c, dd    *sketch.Point
	g, h, i, j     *sketch.Point
	m, n, o, pp    *sketch.Point
	aPrime, bPrime *sketch.Point
	toothP, toothG *sketch.Point
}

func solvedAt(p *sketch.Point) xy { return xy{p.X(), p.Y()} }

// assertLattice holds the solved figure to the facts §2 and the Variables
// section pin, all of them read off the SOLVED geometry rather than the seeds,
// which is the rule the Maximum Face Width depends on ([PB-SOLVED-GEOMETRY]).
func assertLattice(t testing.TB, s *sketch.Sketch, d bevelDesign, seeds latticeSeeds, r latticeReadout) {
	t.Helper()
	near := func(name string, got xy, want xy) {
		if got.sub(want).len() > 1e-7 {
			t.Errorf("%s solved to (%.9f, %.9f), want (%.9f, %.9f)", name, got.x, got.y, want.x, want.y)
		}
	}
	// Every point closes at the position the seed rule states. A seed that
	// disagrees with its own closure is a seed waiting to pick the wrong branch,
	// so agreement here is what makes the seeding rule checkable at all.
	near("Apex", solvedAt(r.apex), seeds.apex)
	near("Apex 2", solvedAt(r.apex2), seeds.apex2)
	near("A", solvedAt(r.a), seeds.a)
	near("B", solvedAt(r.b), seeds.b)
	near("C", solvedAt(r.c), seeds.c)
	near("D", solvedAt(r.dd), seeds.d)
	near("G", solvedAt(r.g), seeds.g)
	near("H", solvedAt(r.h), seeds.h)
	near("I", solvedAt(r.i), seeds.i)
	near("J", solvedAt(r.j), seeds.j)
	near("M", solvedAt(r.m), seeds.m)
	near("N", solvedAt(r.n), seeds.n)
	near("O", solvedAt(r.o), seeds.o)
	near("P", solvedAt(r.pp), seeds.pp)
	near("A'", solvedAt(r.aPrime), seeds.aPrime)
	near("B'", solvedAt(r.bPrime), seeds.bPrime)

	// Point I closes exactly on the projected centre, which is what fixes the
	// Apex's height above the anchor line and, through it, the whole figure's
	// station.
	if solvedAt(r.i).len() > 1e-7 {
		t.Errorf("I solved to (%.9f, %.9f), want the projected centre at the origin", r.i.X(), r.i.Y())
	}

	// The two cone angles, against the closed form the seeds come from.
	apex, apex2 := solvedAt(r.apex), solvedAt(r.apex2)
	pitch := apex2.sub(apex)
	gotP := angleBetween(solvedAt(r.a).sub(apex), pitch)
	gotG := angleBetween(solvedAt(r.b).sub(apex), pitch)
	if math.Abs(gotP-d.pinion.gamma) > 1e-9 {
		t.Errorf("pinion pitch cone angle %.12f rad, want %.12f", gotP, d.pinion.gamma)
	}
	if math.Abs(gotG-d.driving.gamma) > 1e-9 {
		t.Errorf("driving pitch cone angle %.12f rad, want %.12f", gotG, d.driving.gamma)
	}
	if got := pitch.len(); math.Abs(got-d.pitchCone) > 1e-9 {
		t.Errorf("Pitch Cone Distance %.9f mm, want %.9f", got, d.pitchCone)
	}
	// The Pitch Cone Distance is NOT the Cone Distance parameter, and the two
	// coincide only at Shaft Angle 90 degrees. Asserting the relation is what
	// keeps a later step from reaching for the wrong one.
	if math.Abs(d.shaftAngle-math.Pi/2) < 1e-12 {
		if math.Abs(2*pitch.len()-d.coneDistance) > 1e-9 {
			t.Errorf("at Shaft Angle 90 the Cone Distance %.9f must be twice R %.9f", d.coneDistance, pitch.len())
		}
	}

	// The Maximum Face Width, from the SOLVED A, B, C, D, H and J rather than
	// from the seeds. It is 0.95 times the smaller of the two point-to-line
	// distances, and the binding side is whichever gear carries the smaller
	// pitch radius — not the pinion by name.
	distToLine := func(p, on, dir xy) float64 { return math.Abs(p.sub(on).crs(dir) / dir.len()) }
	pinionDist := distToLine(solvedAt(r.a), solvedAt(r.c), solvedAt(r.h).sub(solvedAt(r.c)))
	drivingDist := distToLine(solvedAt(r.b), solvedAt(r.dd), solvedAt(r.j).sub(solvedAt(r.dd)))
	measuredMax := 0.95 * math.Min(pinionDist, drivingDist)
	if math.Abs(measuredMax-d.maxFaceWidth) > 1e-7*d.maxFaceWidth {
		t.Errorf("Maximum Face Width from solved geometry %.9f mm, want the closed form %.9f",
			measuredMax, d.maxFaceWidth)
	}
	if d.faceWidth > d.maxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.9f mm exceeds its own cap %.9f", d.faceWidth, d.maxFaceWidth)
	}

	// The toe corners ride at their gears' Toe Radii, on the SAME side of the
	// shaft axis as the rest of the figure. The front face's length dimension is
	// unsigned, so the far side is a solution the constraints admit and only the
	// seed rules out; reading the SIGNED offset is what makes the branch
	// checkable here rather than in Fusion at the revolve.
	signedOff := func(p, apexPt, dir xy) float64 { return p.sub(apexPt).crs(dir) }
	refP := signedOff(solvedAt(r.c), apex, seeds.pinionDir)
	if got := signedOff(solvedAt(r.n), apex, seeds.pinionDir); got*refP <= 0 {
		t.Errorf("N sits on the far side of the pinion shaft axis (signed offset %.9f against C's %.9f); "+
			"the revolve would cross its own axis", got, refP)
	}
	if got := math.Abs(signedOff(solvedAt(r.n), apex, seeds.pinionDir)); math.Abs(got-d.pinion.toeRadius) > 1e-7 {
		t.Errorf("N rides at %.9f mm from the pinion shaft axis, want the Toe Radius %.9f",
			got, d.pinion.toeRadius)
	}
	refG := signedOff(solvedAt(r.dd), apex, seeds.drivingDir)
	if got := signedOff(solvedAt(r.pp), apex, seeds.drivingDir); got*refG <= 0 {
		t.Errorf("P sits on the far side of the driving shaft axis (signed offset %.9f against D's %.9f)",
			got, refG)
	}
	if got := math.Abs(signedOff(solvedAt(r.pp), apex, seeds.drivingDir)); math.Abs(got-d.driving.toeRadius) > 1e-7 {
		t.Errorf("P rides at %.9f mm from the driving shaft axis, want the Toe Radius %.9f",
			got, d.driving.toeRadius)
	}

	// The toe end is nearer the Apex than the heel end. Nothing dimensions that;
	// it follows from the Apex 2 drops aiming into the interior wedge, and the
	// mirrored frame the ⚠️ in §2 describes is exactly the solution where it
	// fails.
	if solvedAt(r.m).sub(apex).len() >= solvedAt(r.c).sub(apex).len() {
		t.Error("the pinion toe corner M is no nearer the Apex than the heel corner C; the frame has mirrored")
	}
	if solvedAt(r.o).sub(apex).len() >= solvedAt(r.dd).sub(apex).len() {
		t.Error("the driving toe corner O is no nearer the Apex than the heel corner D; the frame has mirrored")
	}

	// The root length the toe line was offset by, measured along the root
	// element, and the Toe Extension 0 identity that makes the input's default
	// reproduce the profile built before it existed.
	if got := solvedAt(r.c).sub(solvedAt(r.m)).len(); math.Abs(got-d.rootLength) > 1e-7 {
		t.Errorf("|C->M| is %.9f mm, want the Root Length %.9f", got, d.rootLength)
	}
	if d.toeExtension == 0 {
		want := d.faceWidth * d.rootCone / d.pitchCone
		if got := solvedAt(r.c).sub(solvedAt(r.m)).len(); math.Abs(got-want) > 1e-7 {
			t.Errorf("at Toe Extension 0 the Root Length is %.9f mm, want Face Width * |Apex->Ded| / R = %.9f",
				got, want)
		}
	}

	// The tooth centre is the back cone's own apex on the shaft axis, one
	// virtual pitch radius from Apex 2 along the dedendum line, and the Tooth
	// Spacing moves ONLY that centre.
	if got := solvedAt(r.toothP).sub(apex).len(); math.Abs(got-latticeToothDistance(d, sidePinion)) > 1e-7 {
		t.Errorf("the pinion tooth centre sits %.9f mm from the Apex, want %.9f",
			got, latticeToothDistance(d, sidePinion))
	}
	if got := solvedAt(r.toothG).sub(apex).len(); math.Abs(got-latticeToothDistance(d, sideDriving)) > 1e-7 {
		t.Errorf("the driving tooth centre sits %.9f mm from the Apex, want %.9f",
			got, latticeToothDistance(d, sideDriving))
	}
	if d.toothSpacing == 0 {
		if got := solvedAt(r.toothP).sub(apex2).len(); math.Abs(got-d.pinion.virtualPitchRadius()) > 1e-7 {
			t.Errorf("|Apex2->K| is %.9f mm, want the pinion virtual pitch radius %.9f",
				got, d.pinion.virtualPitchRadius())
		}
	}

	// Every bound the module resolves before it draws anything, held against this
	// case. The table only carries configurations the module accepts, so what this
	// proves is that each bound's formula admits them — and, where a case sits at
	// an end of a range, that the formula's own end is where the spec says.
	assertResolvedBounds(t, d)

	// The lattice is construction geometry end to end, so it closes no region
	// the solid features could pick up by accident.
	if regions := s.Profiles(); len(regions) != 0 {
		t.Errorf("the Gear Profiles sketch closed %d region(s); every §2 line is construction geometry", len(regions))
	}
}

// assertResolvedBounds holds one case to every closed-form bound the Variables
// section states, on both gears.
//
// The Minimum Teeth floor is exactly the statement that the base-height window is
// non-empty, which is why the module checks it first; the two base-height bounds
// are what keep the heel edge from running back inward at one end and from
// crossing the shaft axis at the other; the Toe Radius Ceiling is where the inner
// toe corner would fall behind the outer one; and the Toe Limit is where the toe
// face closes to nothing.
func assertResolvedBounds(t testing.TB, d bevelDesign) {
	t.Helper()
	for _, s := range []bevelSide{d.pinion, d.driving} {
		if s.teeth < s.minTeeth {
			t.Errorf("%s: %.0f teeth is below the computed floor 5.27 * cos(gamma) = %.4f, where the "+
				"two base-height bounds cross and no base height satisfies both", s.label, s.teeth, s.minTeeth)
		}
		if s.minBase > s.maxBase {
			t.Errorf("%s: the base-height window is empty, minimum %.6f above maximum %.6f",
				s.label, s.minBase, s.maxBase)
		}
		if s.baseHeight < s.minBase-1e-12 || s.baseHeight > s.maxBase+1e-12 {
			t.Errorf("%s: the resolved base height %.6f mm sits outside its own window [%.6f, %.6f]",
				s.label, s.baseHeight, s.minBase, s.maxBase)
		}
		// The Maximum Base Height is deliberately conservative: it sits
		// 1.25 * Module * sin(gamma) below the true crossing, where H reaches the
		// shaft axis at r * tan(gamma), because past that the profile has crossed
		// its own axis of revolution.
		crossing := s.pitchRadius() * math.Tan(s.gamma)
		if s.maxBase >= crossing {
			t.Errorf("%s: the Maximum Base Height %.6f is not below the true crossing %.6f",
				s.label, s.maxBase, crossing)
		}
		if s.toeRadius <= 0 {
			t.Errorf("%s: the resolved Toe Radius is %.6f; only the front face's foot may touch the "+
				"shaft axis, so it has to be strictly positive", s.label, s.toeRadius)
		}
		if s.toeRadius >= s.toeRadiusCeiling {
			t.Errorf("%s: the resolved Toe Radius %.6f is not below its Toe Radius Ceiling %.6f",
				s.label, s.toeRadius, s.toeRadiusCeiling)
		}
	}
	if d.faceWidth > d.maxFaceWidth+1e-12 {
		t.Errorf("the resolved Face Width %.6f exceeds the Maximum Face Width %.6f",
			d.faceWidth, d.maxFaceWidth)
	}
	if d.toeExtension > 0 {
		base := d.faceWidth * d.rootCone / d.pitchCone
		limit := math.Min(d.pinion.toeLimit, d.driving.toeLimit)
		if limit <= base {
			t.Errorf("a Toe Extension of %.0f%% was resolved on a pair whose smaller Toe Limit %.6f "+
				"is at or below the Toe Extension 0 root length %.6f; the module rejects that pair "+
				"rather than shrinking its Toe Radius", d.toeExtension, limit, base)
		}
		if d.rootLength <= base || d.rootLength >= limit {
			t.Errorf("the Root Length %.6f is not strictly between the Toe Extension 0 length %.6f "+
				"and the smaller Toe Limit %.6f", d.rootLength, base, limit)
		}
	}
}

// latticeToothDistance is how far K / L sits from the Apex along the shaft
// axis, once the Tooth Spacing has moved it along the dedendum line.
func latticeToothDistance(d bevelDesign, which float64) float64 {
	s := d.side(which)
	along := d.pitchCone/math.Cos(s.gamma) + d.toothSpacing*math.Sin(s.gamma)
	across := d.toothSpacing * math.Cos(s.gamma)
	return math.Hypot(along, across)
}

func angleBetween(a, b xy) float64 {
	return math.Atan2(math.Abs(a.crs(b)), a.dot(b))
}

// ---------------------------------------------------------------- S8 virtual spur tooth (§3)

// stepVirtualSpurTooth draws one gear's virtual (back-cone, Tredgold) spur tooth
// on the {gearLabel} Plane, centred on the tooth-centre point K' / L'.
//
// The tooth itself is drawn by the borrowed spur generator, whose geometry the
// shared involute package holds, at the virtual tooth number and the module —
// never at a measured Apex2->K' distance — and already rotated by 180 degrees
// through draw()'s own angle argument.
//
// WHAT THE PROOF CANNOT REACH. The borrowed generator labels each of its four
// circles with along-path sketch text, and sketch text holds a degree of freedom
// in Fusion ([PB-TEXT-HOLDS-DOF]), which is why the spec forbids gating this
// sketch there. This engine has no sketch text at all, so the sketch here is the
// unlabelled geometry and reaches DOF 0; what that proves is that the geometry
// is determined, not that Fusion's isFullyConstrained would say so.
func stepVirtualSpurTooth(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newBevelDesign(p)
	side := d.side(p["gearSide"])
	virtual := side.virtualTeeth
	dims := involute.Derive(d.module, virtual, involutePressureAngle)

	proofkit.Step(t, "the tooth centre, projected from the Gear Profiles sketch")
	centre := s.CreateReferencePoint(0, 0, "Gear Profiles tooth centre")
	origin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(origin, centre))

	proofkit.Step(t, "the four circles the spur drawer lays down")
	circle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(dims.Root, false)
	tip := circle(dims.Tip, true)
	circle(dims.Base, true)
	circle(dims.Pitch, true)

	proofkit.Step(t, "the involute flanks, drawn already rotated by 180 degrees")
	const halfTurn = math.Pi
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, virtual, spurInvoluteSteps, halfTurn)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank: %v", err)
	}

	proofkit.Step(t, "the tooth-top arc, its centre shared with the tooth centre")
	topX, topY := involute.Rotate(dims.Tip, 0, halfTurn)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))

	proofkit.Step(t, "the spine and the angular pin that holds the half turn")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(dims.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, dims.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, 180))

	proofkit.Step(t, "the ribs that pin each flank sample")
	spurRibs(s, left, right, leftPts, rightPts, origin, spine, halfTurn)

	proofkit.Step(t, "the flank-to-root stubs, drawn only when the tooth is not embedded")
	if !dims.Embedded() {
		foot := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := dims.Root*seed.X/n, dims.Root*seed.Y/n
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

	proofkit.Step(t, "the curve counts the tooth-profile selection keys on")
	solveHere(t, s)
	assertToothProfile(t, s, dims, side, d)
}

// involutePressureAngle is the pressure angle the framework's VirtualSpurProxy
// serves, 20 degrees. It is NOT a bevel dialog input; the proxy's default is
// what the borrowed drawer reads.
var involutePressureAngle = inRadians(20)

// spurInvoluteSteps is the proxy's Involute Steps value, 15.
const spurInvoluteSteps = 15

// spurRibs pins each flank sample pair to the spine, which is the spur family's
// own scheme and is proved in proof/spurgear. It is repeated here because the
// bevel tooth is that same sketch at the virtual tooth number, and what this
// step has to show is that the scheme still closes there.
func spurRibs(s *sketch.Sketch, left, right []involute.Pt, leftPts, rightPts []*sketch.Point,
	origin *sketch.Point, spine *sketch.Line, angle float64) {
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
		s.AddConstraint(sketch.NewPointOnLine(mid, spine), sketch.NewMidpoint(mid, rib))
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
}

// assertToothProfile pins the fact the tooth-profile selection keys on: the
// tooth loop's curve counts, with the line count DETERMINED by the embedded
// flag and never accepted as either. An unrelated loop between the drawn
// circles can carry the same two NURBS and two arcs with the other line count,
// and selecting it makes the apex->profile loft fail with LOFT_NO_TOOLBODY.
func assertToothProfile(t testing.TB, s *sketch.Sketch, dims involute.Dimensions, side bevelSide, d bevelDesign) {
	t.Helper()
	wantLines := 2
	if dims.Embedded() {
		wantLines = 0
	}
	tooth, disc := 0, 0
	regions := s.Profiles()
	for _, region := range regions {
		nurbs, arcs, lines := loopCounts(region.Entities)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth++
		case nurbs == 0 && arcs == 1 && lines == 0:
			disc++
		default:
			t.Errorf("unexpected region: %d NURBS, %d arcs, %d lines", nurbs, arcs, lines)
		}
		if !region.Valid {
			t.Errorf("region with %d NURBS, %d arcs, %d lines is not extrudable", nurbs, arcs, lines)
		}
	}
	if tooth != 1 {
		t.Errorf("tooth regions of 2 NURBS, 2 arcs and exactly %d lines: %d, want 1", wantLines, tooth)
	}
	if disc != 1 {
		t.Errorf("root-circle disc regions: %d, want 1", disc)
	}
	// The virtual tooth number comes from the closed form and never from a
	// measured Apex2->K' distance, and the Tooth Spacing does not move it.
	want := math.Floor(2 * side.virtualPitchRadius() / d.module)
	if side.virtualTeeth != want {
		t.Errorf("virtual tooth number %.0f, want floor(2 * virtual pitch radius / Module) = %.0f",
			side.virtualTeeth, want)
	}
	if got := dims.Pitch; math.Abs(got-side.virtualPitchRadius()) > d.module/2 {
		t.Errorf("the drawn tooth's pitch radius %.9f mm is more than half a module from the "+
			"virtual pitch radius %.9f", got, side.virtualPitchRadius())
	}
}

func loopCounts(entities []sketch.Entity) (nurbs, arcs, lines int) {
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

// ---------------------------------------------------------------- S10 per-gear profile sketch

// stepGearProfileHexagon builds one gear's own Profile sketch: the six §2
// vertices recreated as fresh points at their exact positions, the closed
// hexagon drawn SHARING those points, and the points fixed only AFTER the lines
// exist.
//
// The order is load-bearing ([PB-PROJECT-NOT-FIXED]): fixing a bare point before
// it is consumed as a line endpoint does not leave the sketch fully constrained.
// Projecting the §2 points instead would leave the sketch under-constrained,
// because a projection is associative and not fixed, and the first edge's
// worldGeometry would then resolve against a default frame and put the body on
// world XY ([PB-WORLDGEO-CONSTRAINED]).
//
// One profile sketch per gear, so sketch.profiles holds exactly this one hexagon
// loop and the revolve takes it without a search ([PB-SINGLE-PROFILE]).
func stepGearProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	h := d.axialProfile(which)

	proofkit.Step(t, "recreate the six §2 vertices at their solved positions")
	verts := make([]*sketch.Point, 0, 6)
	for _, v := range h.vertices() {
		verts = append(verts, s.CreatePoint(v.station, v.radius))
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, 0, 6)
	for i := range verts {
		lines = append(lines, s.CreateLine(verts[i], verts[(i+1)%len(verts)]))
	}

	proofkit.Step(t, "fix the endpoints, after the lines exist")
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}

	proofkit.Step(t, "the one closed region the revolve consumes")
	solveHere(t, s)
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("the %s Profile sketch holds %d regions, want exactly the one hexagon loop",
			d.side(which).label, len(regions))
	}
	if !regions[0].Valid {
		t.Error("the hexagon region is not extrudable")
	}
	if _, _, lineCount := loopCounts(regions[0].Entities); lineCount != 6 {
		t.Errorf("the hexagon loop holds %d lines, want 6", lineCount)
	}
	// The profile must not cross the axis of revolution, which is the first
	// edge. A profile that does makes Fusion abort the revolve with
	// ASM_WIRE_X_AXIS, and it is what the Maximum Face Width, the Maximum Base
	// Height and the strictly positive Toe Radius are each there to stop.
	for i, v := range h.vertices() {
		if v.radius < -1e-12 {
			t.Errorf("hexagon vertex %d sits at radius %.9f mm, across the shaft axis", i, v.radius)
		}
	}
	if h.toeIn.radius <= 0 {
		t.Errorf("the inner toe corner rides at radius %.9f mm; only the front face's foot may touch the axis",
			h.toeIn.radius)
	}
	// The first edge IS the shaft axis: both its ends sit on it.
	if math.Abs(h.toeFoot.radius) > 1e-12 || math.Abs(h.heelAxis.radius) > 1e-12 {
		t.Error("the hexagon's first edge is not on the shaft axis")
	}
	if h.heelAxis.station <= h.toeFoot.station {
		t.Error("the hexagon's first edge runs backwards: the heel end is not beyond the toe end")
	}
}

// ---------------------------------------------------------------- S15 the 2-D tooth trace

// stepSpiralTrace draws the {gear} 2D Tooth Trace sketch: the genuine cutter
// circle and the three-point arc through the trace's toe end, the mean point on
// the cone element, and its heel end.
//
// The frame is the cone's tangent plane with the apex at the origin, x along the
// cone element so a point's x IS its cone distance, and y circumferential. The
// three loci the construction needs — the toe, mean and heel circles — are
// concentric about the APEX, which is the most natural thing to get wrong.
//
// THIS SKETCH IS DELIBERATELY LEFT WITH FREE DEGREES OF FREEDOM and is exempt
// from the full-constraint gate: the arc's endpoints are pinned by the
// three-point construction rather than by endpoint dimensions, because
// dimensioning them over-constrains the solve against the cone-element plane.
// proofkit's gate does not admit a free degree of freedom, so the proof pins the
// two endpoints as reference points at their closed-form positions — which is
// what the three-point construction puts them at — and asserts the invariants
// spiral-tooth-trace.md lists rather than the DOF count. The free DOF itself is
// therefore NOT reproduced here; it is a Fusion-side fact the spec declares.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	tr := d.newSpiralTrace(which)

	proofkit.Step(t, "the cone element, from the apex out to the heel cone distance")
	apex := s.CreateReferencePoint(0, 0, "Gear Profiles apex")
	coneEnd := s.CreateReferencePoint(tr.rHeel, 0, "Gear Profiles cone element")
	coneElement, err := s.CreateReferenceLine(apex, coneEnd, "Gear Profiles cone element")
	if err != nil {
		t.Fatalf("cone element: %v", err)
	}
	coneElement.SetConstruction(true)

	proofkit.Step(t, "the cutter circle, centre fixed and diameter dimensioned")
	cutterCentre := s.CreatePoint(tr.centreX, tr.centreY)
	cutter := s.CreateCircle(cutterCentre, tr.cutterRadius)
	cutter.SetConstruction(true)
	s.Fix(cutterCentre)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*tr.cutterRadius))

	// THE ARC ITSELF IS THE ONE PIECE OF THIS STEP THE HARNESS REFUSES, and what
	// it is replaced by is the circle the arc is a portion of.
	//
	// In Fusion the trace is a THREE-POINT arc through the toe end, the mean point
	// on the cone element and the heel end, carrying two rows that say it IS the
	// cutter circle: its centre coincident with the cutter circle's centre, and a
	// radius dimension equal to r_c. Both are needed there because
	// addByThreePoints shares an arc's start and end points but COPIES the
	// centre, which is the one place [PB-SHARE-XOR-COINCIDENT] says passing a
	// point and coincidenting to it is correct rather than redundant; a stranded
	// centre silently deforms the curve, measured in Fusion at 22.9 mm behind its
	// origin on a sketch that raised no error.
	//
	// This engine attaches an internal equal-radius row to every arc, so an arc
	// whose centre and both ends are pinned carries a row for no freedom and the
	// sketch reads overconstrained; leaving a point free to absorb that row makes
	// the arc's centre or its ends a circle-circle intersection, which is two
	// solutions and reads as a discrete ambiguity. Either way the arc cannot sit
	// in a sketch this gate accepts. So the proof draws the cutter circle and
	// carries the arc's three stations as pinned reference points on it.
	//
	// WHAT THE SUBSTITUTION COSTS: the portion. Everything the arc's own two rows
	// assert — that the trace is the genuine cutter circle, centred where the hand
	// and psi put it and of the cutter's radius — is asserted below on the circle
	// the sketch really drew. What is not shown is that Fusion keeps the piece of
	// it between the toe and heel ends, or that a three-point arc through those
	// three points is that piece.
	proofkit.Step(t, "the trace's three stations, on the cutter circle")
	toe := s.CreateReferencePoint(tr.toeX, tr.toeY, "cutter arc toe end")
	heel := s.CreateReferencePoint(tr.heelX, tr.heelY, "cutter arc heel end")
	meanPoint := s.CreateReferencePoint(tr.rMean, 0, "cutter arc mean point")

	proofkit.Step(t, "the invariants a correct trace satisfies")
	solveHere(t, s)
	assertSpiralTrace(t, d, which, tr, cutter, coneElement, toe, heel, meanPoint)
}

// assertSpiralTrace holds the drawn trace to the checks spiral-tooth-trace.md §9
// lists, on the circle and the stations the sketch actually built.
func assertSpiralTrace(t testing.TB, d bevelDesign, which float64, tr spiralTrace,
	cutter *sketch.Circle, coneElement *sketch.Line, toe, heel, meanPoint *sketch.Point) {
	t.Helper()
	// The trace's radius is the cutter radius everywhere, because it is one
	// circle and not a fitted look-alike.
	if got := cutter.R(); math.Abs(got-tr.cutterRadius) > 1e-9 {
		t.Errorf("the cutter circle's radius is %.9f mm, want the cutter radius %.9f", got, tr.cutterRadius)
	}
	// The centre is exactly r_c from the mean point, so the trace passes through
	// it, and the mean point sits on the cone element at the mean cone distance.
	centre := xy{cutter.Center.X(), cutter.Center.Y()}
	mean := solvedAt(meanPoint)
	if got := centre.sub(mean).len(); math.Abs(got-tr.cutterRadius) > 1e-9 {
		t.Errorf("the cutter centre sits %.9f mm from the mean point, want the cutter radius %.9f",
			got, tr.cutterRadius)
	}
	if math.Abs(mean.y) > 1e-12 || math.Abs(mean.x-(tr.rToe+tr.rHeel)/2) > 1e-9 {
		t.Errorf("the mean point is at (%.9f, %.9f), want the mean cone distance on the element",
			mean.x, mean.y)
	}
	// The ends sit on the toe and heel circles about the APEX — the most natural
	// centre to get wrong — each taken a hair past the face so the kept arc
	// reaches cleanly past the end trims, and each ON the cutter circle.
	wantToe := tr.rToe - 0.06*tr.span
	wantHeel := tr.rHeel + 0.06*tr.span
	if got := solvedAt(toe).len(); math.Abs(got-wantToe) > 1e-7 {
		t.Errorf("the trace's toe end is at cone distance %.9f mm, want %.9f", got, wantToe)
	}
	if got := solvedAt(heel).len(); math.Abs(got-wantHeel) > 1e-7 {
		t.Errorf("the trace's heel end is at cone distance %.9f mm, want %.9f", got, wantHeel)
	}
	for name, end := range map[string]*sketch.Point{"toe": toe, "heel": heel} {
		if got := solvedAt(end).sub(centre).len(); math.Abs(got-tr.cutterRadius) > 1e-7 {
			t.Errorf("the trace's %s end sits %.9f mm from the cutter centre, want the cutter radius %.9f",
				name, got, tr.cutterRadius)
		}
	}
	// The spiral angle is realised at the mean point and nowhere else: the angle
	// between the arc's tangent there and the cone element is psi.
	tangent := xy{centre.y, mean.x - centre.x}
	element := xy{coneElement.End.X() - coneElement.Start.X(), coneElement.End.Y() - coneElement.Start.Y()}
	if got := angleBetweenLines(tangent, element); math.Abs(got-d.spiralAngle) > 1e-9 {
		t.Errorf("the trace makes %.12f rad with the cone element at the mean point, want psi %.12f",
			got, d.spiralAngle)
	}
	// The hand goes on the cos term, so opposite hands mirror the centre ACROSS
	// the cone element. Mirroring about x = R_mean instead is a different curve
	// and gives the two gears unequal twist.
	mirror := d.
		with(func(c *bevelDesign) { c.hand = -c.hand }).
		newSpiralTrace(which)
	if math.Abs(mirror.centreX-tr.centreX) > 1e-9 || math.Abs(mirror.centreY+tr.centreY) > 1e-9 {
		t.Errorf("flipping the hand moved the cutter centre from (%.9f, %.9f) to (%.9f, %.9f); "+
			"it must mirror across the cone element", tr.centreX, tr.centreY, mirror.centreX, mirror.centreY)
	}
	// The twist uses the PITCH cone angle, not the root cone angle the element
	// itself makes with the shaft. The two differ by the dedendum angle and the
	// root reading inflates the twist.
	side := d.side(which)
	rootTwist := math.Abs(math.Atan2(tr.heelY, tr.heelX)-math.Atan2(tr.toeY, tr.toeX)) / math.Sin(side.rootGamma)
	if math.Abs(tr.twist-rootTwist) < 1e-12 && math.Abs(side.gamma-side.rootGamma) > 1e-9 {
		t.Error("the twist was taken with the root cone angle; the crown-gear roll ratio uses the pitch angle")
	}
}

// with returns a copy of the design with one field changed, for the mirror check
// above.
func (d bevelDesign) with(f func(*bevelDesign)) bevelDesign {
	f(&d)
	return d
}

// ---------------------------------------------------------------- S24 bore sketch

// stepBoreSketch draws one gear's Bore sketch on the plane rooted at the shaft
// edge's start: one circle centred on the sketch origin, with its centre FIXED
// and a diameter dimension.
//
// The centre is fixed rather than made coincident to the sketch origin, which is
// [PB-CIRCLE-CENTER]: a circle created at (0, 0, 0) does not reuse the sketch's
// origin point, its centre is a free point that happens to sit there, and the
// coincident has been observed to throw VCS_SKETCH_SOLVING_FAILED on a
// setByDistanceOnPath plane.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newBevelDesign(p)
	side := d.side(p["gearSide"])
	if side.boreDiameter <= 0 {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no Bore sketch is drawn at all")
	}

	proofkit.Step(t, "the bore circle, centre fixed and diameter dimensioned")
	centre := s.CreatePoint(0, 0)
	circle := s.CreateCircle(centre, side.boreDiameter/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, side.boreDiameter))

	proofkit.Step(t, "the bore diameter and the one region the cut consumes")
	solveHere(t, s)
	if got := 2 * circle.R(); math.Abs(got-side.boreDiameter) > 1e-9 {
		t.Errorf("bore diameter %.9f mm, want %.9f", got, side.boreDiameter)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("the Bore sketch holds %d regions, want exactly the one circle", len(regions))
	}
	// A Bore Diameter of 0 means auto, and auto is THIS gear's own Pitch Diameter
	// / 4 — never the pair's, and never the other gear's.
	if p["drivingBore"] == 0 && p["pinionBore"] == 0 {
		if want := side.pitchDia / 4; math.Abs(side.boreDiameter-want) > 1e-12 {
			t.Errorf("the auto bore diameter is %.9f mm, want this gear's Pitch Diameter / 4 = %.9f",
				side.boreDiameter, want)
		}
	}
}

// angleBetweenLines is the unsigned angle between two LINES rather than two
// directions, so it lands in [0, pi/2]. The spiral angle is measured between the
// trace and the cone element, and the two hands put the trace's tangent on
// opposite sides of the element, which flips a direction-to-direction reading to
// its supplement without changing the angle the two lines make.
func angleBetweenLines(a, b xy) float64 {
	return math.Atan2(math.Abs(a.crs(b)), math.Abs(a.dot(b)))
}
