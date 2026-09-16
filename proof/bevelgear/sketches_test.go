// This file holds the bevel gear's sketch steps: the Anchor sketch, the §2 Gear
// Profiles lattice, the per-gear virtual spur tooth, the per-gear Profile
// hexagon and the Bore sketch.
//
// Three things in these sketches are outside what the sketch engine can hold,
// and each is recorded beside the thing it cannot reach.
//
//   - Fusion's `addOffsetDimension` is a DISTANCE dimension that requires its
//     two lines to be parallel already, so the spec supplies that parallelism
//     with a perpendicular (E->G against H->G, F->I against J->I) or with an
//     explicit `addParallel` (M->N against C->H, O->P against D->J). This
//     engine's offset emits TWO residual rows, holding both endpoints of the
//     target line at the same signed perpendicular distance from the source, so
//     it carries the parallelism itself. Adding the perpendicular or the
//     parallel here is a third row for the same two freedoms and the lattice
//     comes back over-constrained at DOF 0 ([PB-NO-OVERCONSTRAIN]). The proof
//     therefore leaves all four out. The spec states this for the two
//     base-height offsets; the two toe-line offsets have the same shape and the
//     same consequence, and that is a gap in the spec rather than a decision
//     taken here.
//
//   - `addCoincident(I, projectedCenter)` is two rows in Fusion, of which one is
//     already implied: I lies on B->F, which is collinear with the driving shaft
//     axis, which passes through the projected centre by construction. Fusion
//     absorbs the implied row; this engine reports it as redundant. The proof
//     states the independent row — I on the projected anchor line — and then
//     asserts that the solved I IS the projected centre, which is the content
//     the coincidence carries.
//
//   - Every LENGTH dimension of §2 is an aligned distance in Fusion, whose value
//     is a magnitude and whose direction is captured from the seeded geometry
//     ([PB-DIM-VALUE-SEMANTICS]). Stated that way here, each one leaves the
//     mirrored answer standing beside the intended one and the engine refuses a
//     net that admits both. The proof therefore states each as the SIGNED
//     axis-aligned distance the engine offers — the mapping [PB-SKETCH-FIRST]
//     gives — with the sign the seed chose, and then asserts after the solve
//     that the dimensioned length has the magnitude the spec names. That keeps
//     the spec's numbers proven and gives up exactly one thing: a seed on the
//     wrong side of one of these dimensions is a defect this proof cannot see,
//     which is the same limit the toe-line seeding records below.
//
//   - `addCollinear` carries two point-on-line rows and one of them is always
//     already satisfied where the new line starts on the reference line's own
//     endpoint ([PB-COLLINEAR-CHAIN]). The engine counts the same two rows, so
//     the proof substitutes the single point-on-line row that is not implied.
//     That substitution makes `addCollinear(E->G, A->E)` and
//     `addCollinear(E->G, Apex->A)` read identically here, so the proof cannot
//     tell the two apart and only a Fusion session can.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// projectionSource is the id every projection of the user's centre point and
// anchor line carries into a later sketch. In Fusion §2 projects the Anchor
// sketch's own centre SketchPoint, never the raw user selection; the engine
// refuses a reference to another sketch's point outright, so each sketch here
// carries its own reference geometry tagged with this id. What that models is
// one link of the chain, not the chain.
const projectionSource = "Anchor sketch centre projection"

// ---------------------------------------------------------------- anchor sketch

// anchorCases put the user's centre on the sketch origin and off it, on a
// target plane whose local frame is not the world's. Nothing in the dialog
// requires either.
var anchorCases = []proofkit.Case{
	{Name: "centre_on_origin", Params: sketchParams(1, 31, 31, 90)},
	{Name: "centre_off_origin", Params: withCentre(sketchParams(1, 31, 31, 90), 12, -7)},
	{Name: "centre_far_from_origin", Params: withCentre(sketchParams(1, 31, 31, 90), -40, 25)},
}

// anchorSeedHalfLength is the ±0.5 cm the spec seeds the Anchor Line's two
// endpoints at, in this proof's millimetres. The dimension locks the seeded
// length and the value is arbitrary: nothing downstream reads it.
const anchorSeedHalfLength = 5.0

// stepAnchorSketch draws the Anchor sketch: the projected centre point and the
// Anchor Line through it.
//
// The line is seeded at ±5 mm along the sketch-local X, so its seeded length is
// the 10 mm the aligned dimension then locks. Its absolute direction is
// arbitrary — §2 derives every direction relative to it — but it must not be a
// free degree of freedom, so it is pinned sketch-local with Horizontal
// ([PB-REFLINE-DIRECTION]); a world-axis lock would mis-orient the figure on a
// tilted target plane.
//
// Fusion needs BOTH `addCoincident(projectedCentre, anchorLine)` and
// `addMidPoint(projectedCentre, anchorLine)` — the spec is explicit that
// midpoint alone is not enough. This engine's midpoint carries both rows: it
// states that the centre IS the average of the two endpoints, which already
// puts it on the line. The point-on-line row is therefore redundant here and is
// left out; what Fusion needs the pair for is a Fusion fact this proof cannot
// reach.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	centre := vec{p["centerX"], p["centerY"]}

	proofkit.Step(t, "project the user centre point")
	projected := s.CreateReferencePoint(centre.X, centre.Y, projectionSource)
	projected.SetName("projected centre")

	proofkit.Step(t, "the Anchor Line, seeded at +/- 5 mm along sketch-local X")
	start := s.CreatePoint(centre.X-anchorSeedHalfLength, centre.Y)
	end := s.CreatePoint(centre.X+anchorSeedHalfLength, centre.Y)
	line := s.CreateLine(start, end)
	line.SetName("Anchor Line")
	line.SetConstruction(true)

	s.AddConstraint(
		sketch.NewMidpoint(projected, line),
		// Fusion's aligned distance dimension is a magnitude and the direction is
		// captured from the seed ([PB-DIM-VALUE-SEMANTICS]). Stated that way here
		// the two endpoints may swap and the engine reports two discrete
		// configurations, which this harness refuses. The signed horizontal
		// distance is the same 10 mm with the seeded direction kept, which is what
		// crosses back to Fusion as the seed side.
		sketch.NewHorizontalDistance(start, end, 2*anchorSeedHalfLength),
		sketch.NewHorizontal(line),
	)

	proofkit.Step(t, "the line the Gear Profiles sketch will project")
	solveHere(t, s)
	if got := line.Length(); math.Abs(got-2*anchorSeedHalfLength) > 1e-9 {
		t.Errorf("the Anchor Line solved to %.9f mm, want the seeded %.1f mm the dimension locks",
			got, 2*anchorSeedHalfLength)
	}
	mid := vec{(start.X() + end.X()) / 2, (start.Y() + end.Y()) / 2}
	if got := vecLen(vecSub(mid, centre)); got > 1e-9 {
		t.Errorf("the projected centre sits %.9f mm off the Anchor Line's midpoint; §2 re-projects "+
			"THIS point and the whole figure is built from it", got)
	}
	if got := math.Abs(start.Y() - end.Y()); got > 1e-9 {
		t.Errorf("the Anchor Line is %.9f mm off sketch-local horizontal", got)
	}
}

// ---------------------------------------------------------------- §2 lattice

// latticeBuild carries the one sketch the §2 step draws and the names it gives
// the lines, so a failure says which line of the figure is wrong rather than
// which entity index.
type latticeBuild struct {
	t testing.TB
	s *sketch.Sketch
}

// line is the §2 construction line, built in the COINCIDENT style
// ([BEVEL-F-COINCIDENT-STYLE]): from raw coordinates, with each end that
// connects to an already-existing point pinned by exactly one coincident and
// never shared. A nil pin leaves that end free, and it becomes a new named
// point of the figure. Every §2 line is a construction line, lattice and short
// reference/connector line alike, and each named line is created ONCE
// ([BEVEL-F-LINE-ONCE]).
func (b *latticeBuild) line(name string, aSeed, bSeed vec, pinA, pinB *sketch.Point) *sketch.Line {
	start := b.s.CreatePoint(aSeed.X, aSeed.Y)
	end := b.s.CreatePoint(bSeed.X, bSeed.Y)
	start.SetName(name + " start")
	end.SetName(name + " end")
	l := b.s.CreateLine(start, end)
	l.SetName(name)
	l.SetConstruction(true)
	if pinA != nil {
		b.s.AddConstraint(namedConstraint(b.s, sketch.NewCoincident(start, pinA), name+" start pin"))
	}
	if pinB != nil {
		b.s.AddConstraint(namedConstraint(b.s, sketch.NewCoincident(end, pinB), name+" end pin"))
	}
	return l
}

// signedLength states one of §2's aligned length dimensions the way the engine
// can hold it: as the signed distance along whichever axis the seeded delta
// leans on, which pins the side the seed chose. The magnitude is the spec's own
// and is asserted after the solve by dimensionedLength.
func (b *latticeBuild) signedLength(from, to *sketch.Point, delta vec, name string) {
	if math.Abs(delta.X) >= math.Abs(delta.Y) {
		b.s.AddConstraint(namedConstraint(b.s, sketch.NewHorizontalDistance(from, to, delta.X), name))
		return
	}
	b.s.AddConstraint(namedConstraint(b.s, sketch.NewVerticalDistance(from, to, delta.Y), name))
}

// stationPin states where a point sits ALONG a gear's shaft axis, as the signed
// distance from the axis's own end point on whichever coordinate the axis runs
// on. It is one row, the same as the length dimension it stands in for, and it
// cannot be satisfied by the mirrored position the unsigned length admits.
func (b *latticeBuild) stationPin(from, to *sketch.Point, axisDir vec, delta vec, name string) {
	if math.Abs(axisDir.X) >= math.Abs(axisDir.Y) {
		b.s.AddConstraint(namedConstraint(b.s, sketch.NewHorizontalDistance(from, to, delta.X), name))
		return
	}
	b.s.AddConstraint(namedConstraint(b.s, sketch.NewVerticalDistance(from, to, delta.Y), name))
}

// named registers a constraint under a readable name so the harness's failure
// detail says which rule of §2 conflicted rather than printing a Go type.
func namedConstraint(s *sketch.Sketch, c sketch.Constraint, name string) sketch.Constraint {
	s.SetConstraintName(c, name)
	return c
}

// stepGearProfiles draws the §2 Gear Profiles sketch: the whole lattice, both
// dedendum chains, both base-height offsets, both tooth centres and both toe
// ends.
//
// Every seed is the closed form's own answer. For M and N the spec requires
// exactly that and says why the proof cannot check it: a seed is not a
// constraint, so proving the constraints solve FROM a correct seed says nothing
// about whether the module's seed is correct, and a seed defect therefore
// reaches Fusion untested. That is recorded again at the toe-line seeding below.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := newLattice(t, p)
	b := &latticeBuild{t: t, s: s}

	proofkit.Step(t, "project the Anchor sketch centre and its line")
	centre := s.CreateReferencePoint(l.centre.X, l.centre.Y, projectionSource)
	centre.SetName("projected centre")
	anchorA := s.CreateReferencePoint(
		l.centre.X-anchorSeedHalfLength*l.anchorDir.X,
		l.centre.Y-anchorSeedHalfLength*l.anchorDir.Y, projectionSource)
	anchorB := s.CreateReferencePoint(
		l.centre.X+anchorSeedHalfLength*l.anchorDir.X,
		l.centre.Y+anchorSeedHalfLength*l.anchorDir.Y, projectionSource)
	anchor, err := s.CreateReferenceLine(anchorA, anchorB, projectionSource)
	if err != nil {
		t.Fatalf("project the Anchor Line: %v", err)
	}
	anchor.SetName("projected Anchor Line")

	proofkit.Step(t, "centre -> Apex, perpendicular to the projected anchor line")
	centreToApex := b.line("centre->Apex", l.centre, l.apex, centre, nil)
	apex := centreToApex.End
	apex.SetName("Apex")
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(centreToApex, anchor), "centre->Apex ⟂ anchor"))

	proofkit.Step(t, "the two shaft axes")
	drivingAxis := b.line("Apex->B", l.apex, l.driving.axis, apex, nil)
	pointB := drivingAxis.End
	pointB.SetName("B")
	s.AddConstraint(namedConstraint(s, sketch.NewParallel(drivingAxis, centreToApex), "Apex->B ∥ centre->Apex"))

	pinionAxis := b.line("Apex->A", l.apex, l.pinion.axis, apex, nil)
	pointA := pinionAxis.End
	pointA.SetName("A")
	// Fusion's angular dimension is a magnitude and the text point picks the
	// wedge ([PB-ANGULAR-DIM]); this engine's angle is signed counter-clockwise
	// from the first line to the second, so the sense the spec chooses by the
	// +X-most candidate seed is carried here as the sign instead. Signed is the
	// stricter statement: it admits one configuration where the magnitude admits
	// the mirror.
	s.AddConstraint(namedConstraint(s, sketch.NewAngle(drivingAxis, pinionAxis,
		l.turnSign*l.shaftAngle*180/math.Pi), "Shaft Angle"))

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	dropA := b.line("A->Apex2", l.pinion.axis, l.apex2, pointA, nil)
	apex2 := dropA.End
	apex2.SetName("Apex 2")
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(dropA, pinionAxis), "A->Apex2 ⟂ Apex->A"))
	b.signedLength(dropA.Start, dropA.End, vecSub(l.apex2, l.pinion.axis), "|A->Apex2| = PPD/2")
	dropB := b.line("B->Apex2", l.driving.axis, l.apex2, pointB, apex2)
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(dropB, drivingAxis), "B->Apex2 ⟂ Apex->B"))
	b.signedLength(dropB.Start, dropB.End, vecSub(l.apex2, l.driving.axis), "|B->Apex2| = DPD/2")

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	pitchLine := b.line("Apex->Apex2 (Pitch Line)", l.apex, l.apex2, apex, apex2)
	pinionDed := b.line("Apex2->C (Pinion Dedendum)", l.apex2, l.pinion.ded, apex2, nil)
	pointC := pinionDed.End
	pointC.SetName("C")
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(pinionDed, pitchLine), "Apex2->C ⟂ Pitch Line"))
	b.signedLength(pinionDed.Start, pinionDed.End, vecSub(l.pinion.ded, l.apex2), "|Apex2->C| = 1.25 Module")
	drivingDed := b.line("Apex2->D (Driving Dedendum)", l.apex2, l.driving.ded, apex2, nil)
	pointD := drivingDed.End
	pointD.SetName("D")
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(drivingDed, pitchLine), "Apex2->D ⟂ Pitch Line"))
	b.signedLength(drivingDed.Start, drivingDed.End, vecSub(l.driving.ded, l.apex2), "|Apex2->D| = 1.25 Module")

	proofkit.Step(t, "the two root axes")
	pinionRoot := b.line("Apex->C (Pinion Root Axis)", l.apex, l.pinion.ded, apex, pointC)
	drivingRoot := b.line("Apex->D (Driving Root Axis)", l.apex, l.driving.ded, apex, pointD)

	pinionRim := b.rim(&l, l.pinion, "Pinion", apex, pointA, pointC, pinionAxis, pinionDed, dropA)
	drivingRim := b.rim(&l, l.driving, "Driving", apex, pointB, pointD, drivingAxis, drivingDed, dropB)

	proofkit.Step(t, "close the figure: I on the projected anchor line")
	// In Fusion this is `addCoincident(I, projectedCentre)`. One of its two rows
	// is already implied — I sits on B->F, collinear with the driving shaft axis,
	// which passes through the projected centre — and Fusion absorbs it while
	// this engine reports it as redundant. The independent row is the one below;
	// the coincidence's own content is asserted after the solve.
	s.AddConstraint(namedConstraint(s, sketch.NewPointOnLine(drivingRim.ext2, anchor), "I on the anchor line"))

	proofkit.Step(t, "K and L, and the tooth centres K' and L'")
	pinionK := b.toothCentre(&l, "Pinion", "K", l.pinion, pinionRim.ext2, pointC, pinionAxis, pinionDed)
	drivingK := b.toothCentre(&l, "Driving", "L", l.driving, drivingRim.ext2, pointD, drivingAxis, drivingDed)

	// Everything the Maximum Face Width is measured from now exists and is
	// solved, so the bound is resolved here, from `.geometry` and never from the
	// seeds ([PB-SOLVED-GEOMETRY]).
	proofkit.Step(t, "resolve the Maximum Face Width from the solved A, B, C, D, H, J")
	solveHere(t, s)
	assertClosedForm(t, &l, s, apex, pointA, pointB, apex2, pointC, pointD,
		pinionRim, drivingRim, pinionK, drivingK, centre)

	proofkit.Step(t, "the pinion toe end M->N and its front face N->A'")
	b.toe(&l, l.pinion, "Pinion", "M", "N", "A'", apex, pointC, pointA, pinionAxis, pinionRoot, pinionRim.dedendumExtension)
	proofkit.Step(t, "the driving toe end O->P and its front face P->B'")
	b.toe(&l, l.driving, "Driving", "O", "P", "B'", apex, pointD, pointB, drivingAxis, drivingRoot, drivingRim.dedendumExtension)

	proofkit.Step(t, "the two hexagon shaft-axis edges A'->G and B'->I")
	// The spec lists this line before A' exists. It cannot be drawn there, and
	// the ordering is a defect in the spec rather than a choice made here.
	b.line("A'->G", l.pinion.foot, l.pinion.ext2, b.pointByName("A'"), pinionRim.ext2)
	b.line("B'->I", l.driving.foot, l.driving.ext2, b.pointByName("B'"), drivingRim.ext2)

	proofkit.Step(t, "the lengths the aligned dimensions carry, read off the solved figure")
	solveHere(t, s)
	assertDimensionedLengths(t, &l, b)

	if p["declaredRefusal"] != 0 {
		declaredRefusal(t, s, p)
	}
}

// assertDimensionedLengths reads back every magnitude §2 states as an aligned
// distance dimension. The proof pins each of them as a signed axis distance, for
// the reason this file's header gives, so the magnitude the spec names is proved
// here rather than imposed by the constraint.
func assertDimensionedLengths(t testing.TB, l *lattice, b *latticeBuild) {
	t.Helper()
	apex2 := solvedPointOf(b.pointByName("Apex 2"))
	checks := []struct {
		name string
		got  float64
		want float64
	}{
		{"|A->Apex2|", vecLen(vecSub(apex2, solvedPointOf(b.pointByName("A")))), l.pinion.pitchDia / 2},
		{"|B->Apex2|", vecLen(vecSub(apex2, solvedPointOf(b.pointByName("B")))), l.driving.pitchDia / 2},
		{"|Apex2->C|", vecLen(vecSub(solvedPointOf(b.pointByName("C")), apex2)), dedendumModules * l.module},
		{"|Apex2->D|", vecLen(vecSub(solvedPointOf(b.pointByName("D")), apex2)), dedendumModules * l.module},
		{"the pinion front face N->A'", vecLen(vecSub(solvedPointOf(b.pointByName("Pinion N")),
			solvedPointOf(b.pointByName("Pinion A'")))), l.pinion.toeRadius},
		{"the driving front face P->B'", vecLen(vecSub(solvedPointOf(b.pointByName("Driving P")),
			solvedPointOf(b.pointByName("Driving B'")))), l.driving.toeRadius},
		{"the pinion root length |C->M|", vecLen(vecSub(solvedPointOf(b.pointByName("Pinion M")),
			solvedPointOf(b.pointByName("C")))), l.rootLength},
		{"the driving root length |D->O|", vecLen(vecSub(solvedPointOf(b.pointByName("Driving O")),
			solvedPointOf(b.pointByName("D")))), l.rootLength},
	}
	for _, c := range checks {
		if math.Abs(c.got-c.want) > 1e-7*math.Max(1, c.want) {
			t.Errorf("%s solved to %.9f mm, want the dimensioned %.9f mm", c.name, c.got, c.want)
		}
	}

	// The two front faces stand square to their own shaft axes, which is what
	// makes the revolve sweep each into a flat annulus.
	for _, c := range []struct {
		label string
		side  latticeSide
		inner vec
		foot  vec
	}{
		{"pinion", l.pinion, solvedPointOf(b.pointByName("Pinion N")), solvedPointOf(b.pointByName("Pinion A'"))},
		{"driving", l.driving, solvedPointOf(b.pointByName("Driving P")), solvedPointOf(b.pointByName("Driving B'"))},
	} {
		if d := math.Abs(vecDot(vecUnit(vecSub(c.foot, c.inner)), c.side.axisDir)); d > 1e-9 {
			t.Errorf("the %s front face is %.9f off square to its shaft axis", c.label, d)
		}
		// N (P) never sits ON the axis of revolution: only its foot does. Pinning
		// N there makes the later conical split fail with ASM_API_FAILED for
		// asymmetric tooth counts even though the symmetric case survives.
		if r := c.side.radius(l.apex, c.inner); r <= 0 {
			t.Errorf("the %s inner toe corner sits on the shaft axis at radius %.9f mm", c.label, r)
		}
	}
}

// rimParts are the points and lines one gear's base-height chain leaves behind.
type rimParts struct {
	ext, ext2, heel *sketch.Point
	// dedendumExtension is C->H (D->J), the line the toe step offsets from. It is
	// NOT the heel edge G->H (I->J): the spec's offset dimension names C->H.
	dedendumExtension *sketch.Line
}

// rim draws one gear's dedendum chain: the module-length extension off the
// shaft axis, the perpendicular connector to the dedendum corner, the second
// extension, the dedendum-line extension, the closing line and the base-height
// offset.
//
// Three collinears become one point-on-line row each, for the reason this
// file's header gives, and the closing line's perpendicular is left out because
// the offset below already carries the parallelism it exists to supply.
func (b *latticeBuild) rim(l *lattice, side latticeSide, label string, apex, axisEnd, ded *sketch.Point,
	axis, dedLine, drop *sketch.Line) rimParts {
	s := b.s
	proofkit.Step(b.t, "%s rim: the module-length extensions and the base-height offset", label)

	// A->E (B->F): collinear with the shaft axis, seeded one module beyond its
	// end, its length DRIVEN by the perpendicular below and never dimensioned
	// ([BEVEL-F-DRIVEN-DIMS]).
	extSeed := vecAdd(side.axis, vecScale(side.axisDir, l.module))
	extLine := b.line(label+" axis extension", side.axis, extSeed, axisEnd, nil)
	ext := extLine.End
	ext.SetName(label + " E/F")
	s.AddConstraint(namedConstraint(s, sketch.NewPointOnLine(ext, axis), label+" E/F on the shaft axis"))

	// C->E (D->F), perpendicular to that extension, is what puts E (F) at the
	// dedendum corner's own station on the axis.
	connector := b.line(label+" dedendum connector", side.ded, extSeed, ded, ext)
	s.AddConstraint(namedConstraint(s, sketch.NewPerpendicular(extLine, connector),
		label+" dedendum connector ⟂ axis extension"))

	// E->G (F->I): collinear with A->E (B->F), never with the shaft axis further
	// up the chain ([BEVEL-F-COLLINEAR-CHAIN]).
	ext2Seed := vecAdd(side.ext, vecScale(side.axisDir, l.module))
	ext2Line := b.line(label+" second extension", side.ext, ext2Seed, ext, nil)
	ext2 := ext2Line.End
	ext2.SetName(label + " G/I")
	s.AddConstraint(namedConstraint(s, sketch.NewPointOnLine(ext2, extLine), label+" G/I on the axis extension"))

	// C->H (D->J): collinear with the dedendum line Apex2->C (Apex2->D).
	heelSeed := vecAdd(side.ded, vecScale(side.dedDir, l.module))
	heelStub := b.line(label+" dedendum extension", side.ded, heelSeed, ded, nil)
	heel := heelStub.End
	heel.SetName(label + " H/J")
	s.AddConstraint(namedConstraint(s, sketch.NewPointOnLine(heel, dedLine), label+" H/J on the dedendum line"))

	// G->H (I->J), the heel edge, and the base-height offset that drives it.
	heelLine := b.line(label+" heel edge", side.ext, heelSeed, ext2, heel)
	dropDir := vecUnit(vecSub(l.apex2, side.axis))
	offset := signedOffset(side.ext2, side.axis, dropDir)
	s.AddConstraint(namedConstraint(s, sketch.NewOffset(drop, heelLine, offset), label+" Base Height offset"))
	return rimParts{ext: ext, ext2: ext2, heel: heel, dedendumExtension: heelStub}
}

// toothCentre draws the K (L) pin and the tooth-centre reference line C->K'
// (D->L').
//
// K is the intersection of the shaft axis with the dedendum line extended, and
// both of its ends are already fixed by the time it is added, so it takes two
// point-on-line coincidents and no collinear at all ([BEVEL-F-COLLINEAR-CHAIN]);
// an `addCollinear` there over-constrains the sketch and Fusion errors.
//
// At Tooth Spacing 0 nothing more is built: K' IS K, the existing C->K line is
// what §3 uses, and a zero-length dimensioned line would be degenerate
// ([BEVEL-F-LINE-ONCE]). Above 0 the spacing line is built here, inside this
// sketch, so the end-of-step full-constraint gate covers it.
func (b *latticeBuild) toothCentre(l *lattice, label, name string, side latticeSide,
	ext2 *sketch.Point, ded *sketch.Point, axis, dedLine *sketch.Line) *sketch.Point {
	s := b.s
	centreLine := b.line(label+" "+name+" extension", side.ext2, side.centre, ext2, nil)
	k := centreLine.End
	k.SetName(label + " " + name)
	s.AddConstraint(
		namedConstraint(s, sketch.NewPointOnLine(k, axis), label+" "+name+" on the shaft axis"),
		namedConstraint(s, sketch.NewPointOnLine(k, dedLine), label+" "+name+" on the dedendum line"),
	)
	if l.toothSpacing <= 0 {
		b.line(label+" tooth centre reference", side.ded, side.centre, ded, k)
		return k
	}
	spacing := b.line(label+" Tooth Spacing", side.centre, side.tooth, k, nil)
	shifted := spacing.End
	shifted.SetName(label + " " + name + "'")
	s.AddConstraint(namedConstraint(s, sketch.NewPointOnLine(shifted, dedLine),
		label+" "+name+"' on the dedendum line"))
	b.signedLength(spacing.Start, spacing.End, vecScale(side.dedDir, l.toothSpacing),
		label+" Tooth Spacing")
	b.line(label+" tooth centre reference", side.ded, side.tooth, ded, shifted)
	return shifted
}

// toe draws one gear's toe line M->N (O->P), its connector M->C (O->D) and its
// front face N->A' (P->B').
//
// ⚠️ The seeds below are the closed form, which is the rule the spec states, so
// what this proves is that the constraints solve FROM a correct seed and never
// that the generated module's seed is correct. A wrong seed there reaches Fusion
// untested: N's station is fixed by the toe line together with an UNSIGNED
// length on the front face, so the toe line meets the Toe Radius on both sides
// of the shaft axis and the solver takes whichever side the seed started on. On
// the wrong side the revolved hexagon crosses its own axis of revolution and
// Fusion aborts with ASM_WIRE_X_AXIS ([PB-REVOLVE]) at the revolve, pointing at
// the revolve rather than at the seed. This proof cannot see that, and that is
// the honest edge of what this stage checks.
//
// The front face's own dimension is stated here as a SIGNED point-to-line
// distance rather than the aligned length Fusion uses, for the same reason: an
// unsigned length leaves two discrete configurations, one on each side of the
// axis, and this harness refuses a net whose answer depends on where it was
// seeded. The signed form pins the side the closed form chose. What it costs is
// exactly the check named above, which the proof could not make either way.
func (b *latticeBuild) toe(l *lattice, side latticeSide, label, toeName, innerName, footName string,
	apex, ded, axisEnd *sketch.Point, axis, rootAxis, dedendumExtension *sketch.Line) {
	s := b.s
	toeLine := b.line(label+" toe edge", side.toe, side.toeIn, nil, nil)
	toe := toeLine.Start
	toe.SetName(label + " " + toeName)
	inner := toeLine.End
	inner.SetName(label + " " + innerName)

	offset := signedOffset(side.toe, side.ded, side.dedDir)
	s.AddConstraint(
		namedConstraint(s, sketch.NewPointOnLine(toe, rootAxis), label+" "+toeName+" on the root axis"),
		namedConstraint(s, sketch.NewOffset(dedendumExtension, toeLine, offset), label+" Root Length offset"),
	)
	b.line(label+" toe connector", side.toe, side.ded, toe, ded)

	frontFace := b.line(label+" front face", side.toeIn, side.foot, inner, nil)
	foot := frontFace.End
	foot.SetName(label + " " + footName)
	s.AddConstraint(
		namedConstraint(s, sketch.NewPointOnLine(foot, axis), label+" "+footName+" on the shaft axis"),
		namedConstraint(s, sketch.NewPerpendicular(frontFace, axis), label+" front face ⟂ the shaft axis"),
	)
	// Fusion dimensions this face's LENGTH, which is the resolved Toe Radius. A
	// length is unsigned, so stated that way the toe line meets the Toe Radius on
	// BOTH sides of the shaft axis and the net admits the mirror — the very
	// failure the spec's toe-seeding warning describes. The proof pins the same
	// one freedom as the SIGNED station of A' (B') along that axis, and the
	// length the spec dimensions is asserted after the solve instead.
	b.stationPin(axisEnd, foot, side.axisDir, vecSub(side.foot, side.axis),
		label+" front face station (the Toe Radius, signed)")
}

// pointByName finds a point the build named, so a later line can pin to it
// without threading it through every helper.
func (b *latticeBuild) pointByName(name string) *sketch.Point {
	for _, p := range b.s.Points() {
		if p.Name() == name || p.Name() == "Pinion "+name || p.Name() == "Driving "+name {
			return p
		}
	}
	b.t.Fatalf("the figure has no point named %q", name)
	return nil
}

// solveHere solves the sketch mid-step, the way Fusion solves incrementally, so
// the step can read solved positions before it draws the geometry that depends
// on them.
func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
}

// at reads a solved point's plane coordinates.
func solvedPointOf(p *sketch.Point) vec { return vec{p.X(), p.Y()} }

// assertClosedForm holds the solved lattice to the closed form §2 states, to
// nine decimals, and resolves the Maximum Face Width from the solved points the
// way the spec requires.
func assertClosedForm(t testing.TB, l *lattice, s *sketch.Sketch,
	apex, pointA, pointB, apex2, pointC, pointD *sketch.Point,
	pinionRim, drivingRim rimParts, pinionK, drivingK *sketch.Point, centre *sketch.Point) {
	t.Helper()
	const tol = 1e-9

	solvedApex, solvedA, solvedB, solved2 := solvedPointOf(apex), solvedPointOf(pointA), solvedPointOf(pointB), solvedPointOf(apex2)
	// The two cone angles, measured off the solved figure against the closed
	// form the seeds came from. This is the reading the spec's conditioning note
	// says every independent lattice agrees on to nine decimals.
	gotP := math.Atan2(math.Abs(vecCross(vecUnit(vecSub(solvedA, solvedApex)), vecUnit(vecSub(solved2, solvedApex)))),
		vecDot(vecUnit(vecSub(solvedA, solvedApex)), vecUnit(vecSub(solved2, solvedApex))))
	gotG := math.Atan2(math.Abs(vecCross(vecUnit(vecSub(solvedB, solvedApex)), vecUnit(vecSub(solved2, solvedApex)))),
		vecDot(vecUnit(vecSub(solvedB, solvedApex)), vecUnit(vecSub(solved2, solvedApex))))
	if math.Abs(gotP-l.pinion.gamma) > tol {
		t.Errorf("the solved pinion pitch cone angle is %.9f rad, want the closed form %.9f rad",
			gotP, l.pinion.gamma)
	}
	if math.Abs(gotG-l.driving.gamma) > tol {
		t.Errorf("the solved driving pitch cone angle is %.9f rad, want the closed form %.9f rad",
			gotG, l.driving.gamma)
	}
	if got := vecLen(vecSub(solved2, solvedApex)); math.Abs(got-l.pitchCone) > tol*l.pitchCone {
		t.Errorf("|Apex->Apex2| solved to %.9f mm, want the Pitch Cone Distance R = %.9f mm",
			got, l.pitchCone)
	}

	// The coincidence the proof could not state as a coincidence: I IS the
	// projected centre, and the figure closes there.
	if got := vecLen(vecSub(solvedPointOf(drivingRim.ext2), solvedPointOf(centre))); got > 1e-8 {
		t.Errorf("point I solved %.9f mm from the projected centre; the figure does not close", got)
	}

	// The base heights, read as the offsets they were dimensioned as.
	checkBaseHeightOffset(t, "pinion", solvedPointOf(pinionRim.ext2), solvedA, solved2, l.pinion.baseHeight)
	checkBaseHeightOffset(t, "driving", solvedPointOf(drivingRim.ext2), solvedB, solved2, l.driving.baseHeight)

	// The Maximum Face Width, from the solved points and not from the seeds. The
	// pinion is only usually the binding side, so both are measured.
	pinionReach := perpDistance(solvedA, solvedPointOf(pointC), vecUnit(vecSub(solvedPointOf(pinionRim.heel), solvedPointOf(pointC))))
	drivingReach := perpDistance(solvedB, solvedPointOf(pointD), vecUnit(vecSub(solvedPointOf(drivingRim.heel), solvedPointOf(pointD))))
	got := faceWidthMargin * math.Min(pinionReach, drivingReach)
	if math.Abs(got-l.maxFaceWidth) > 1e-7*l.maxFaceWidth {
		t.Errorf("the Maximum Face Width read off the solved figure is %.9f mm, want %.9f mm",
			got, l.maxFaceWidth)
	}
	if l.faceWidth > got+1e-9 {
		t.Errorf("the resolved Face Width %.9f mm exceeds the Maximum Face Width %.9f mm; the "+
			"toe line would push N across the shaft axis and the revolve would fail with "+
			"ASM_WIRE_X_AXIS", l.faceWidth, got)
	}

	// The tooth centres sit on both the shaft axis and the dedendum line, which
	// is what the two point-on-line coincidents say, and Tooth Spacing moves only
	// the centre.
	for _, c := range []struct {
		label string
		got   vec
		want  vec
	}{
		{"pinion tooth centre", solvedPointOf(pinionK), l.pinion.tooth},
		{"driving tooth centre", solvedPointOf(drivingK), l.driving.tooth},
	} {
		if d := vecLen(vecSub(c.got, c.want)); d > 1e-6*l.pitchCone {
			t.Errorf("the %s solved %.9f mm from its closed-form position", c.label, d)
		}
	}
}

// checkBaseHeightOffset reads a base-height offset off the solved figure: the
// perpendicular distance from the axis->Apex2 drop to the point the offset
// drove.
func checkBaseHeightOffset(t testing.TB, label string, got, axisEnd, apex2 vec, want float64) {
	t.Helper()
	if d := perpDistance(got, axisEnd, vecUnit(vecSub(apex2, axisEnd))); math.Abs(d-want) > 1e-8 {
		t.Errorf("the %s base-height offset solved to %.9f mm, want the resolved %.9f mm",
			label, d, want)
	}
}

// declaredRefusal is how the case table records a configuration the spec admits
// and THIS lattice cannot reach.
//
// The Shaft Angle floor of 30 degrees is the one measured today: the net's
// conditioning falls below the sketch engine's 4e-5 trust floor and the engine
// refuses it. That is a fact about this particular constraint net, not about the
// spec — of three independently written lattices two refuse 30 degrees and the
// third passes it and refuses the top of the range instead — so the case stays
// in the table and is marked here rather than the advertised range being
// narrowed. The remedy for a refusal is to change how the lattice is built,
// never to loosen the gate.
func declaredRefusal(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	t.Helper()
	ctx := context.Background()
	if _, err := s.Solve(ctx); err != nil {
		proofkit.Unmodelled(t, "declared refusal: the lattice does not solve at these values: %v", err)
	}
	report := s.Verify(ctx, sketch.WithProbe())
	if report.Check() == nil {
		t.Fatalf("the case table declares this configuration a refusal of this lattice, but it "+
			"passed: DOF %d, conditioning %.3e. Re-measure the declaration rather than leaving it",
			report.DOF, report.Conditioning)
	}
	proofkit.Unmodelled(t, "declared refusal: this lattice reads DOF %d at conditioning %.3e and the "+
		"engine refuses it (%v). The configuration is one the spec admits; the refusal belongs to "+
		"this net's construction", report.DOF, report.Conditioning, report.Check())
}

// ---------------------------------------------------------------- virtual tooth

// stepVirtualSpurTooth draws one gear's §3 tooth profile: the borrowed spur
// tooth, at the EXACT back-cone radius, with the root circle sunk one root sink
// inside the dedendum corner, drawn already rotated 180 degrees.
//
// The virtual tooth number is a real number and is never rounded. It reaches the
// spur drawer only as the angular half-thickness pi/(2*z_v), and with
// z_v = 2*r_v/Module that angle gives the standard tooth thickness pi*Module/2
// at the pitch circle — the same thickness the spur gear of this module carries.
// An integer count drawn at the exact radius gives pi*r_v/round(z_v) instead,
// which misses nominal by a different amount on each member of an unequal pair.
func stepVirtualSpurTooth(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := newLattice(t, p)
	side := l.pinion
	if p["gear"] != 0 {
		side = l.driving
	}
	module := l.module
	virtualRadius := side.virtualPitchRadius()
	virtualTeeth := side.virtualTeeth(module)
	sink := rootSinkFraction * module

	proofkit.Step(t, "%s: virtual pitch radius %.6f mm, virtual tooth number %.6f",
		side.label, virtualRadius, virtualTeeth)
	if got := virtualTeeth * module / 2; math.Abs(got-virtualRadius) > 1e-12*virtualRadius {
		t.Errorf("the virtual tooth number %.9f rebuilds a pitch radius of %.9f mm, want the exact "+
			"back-cone radius %.9f mm", virtualTeeth, got, virtualRadius)
	}
	thickness := 2 * virtualRadius * (math.Pi / (2 * virtualTeeth))
	if want := math.Pi * module / 2; math.Abs(thickness-want) > 1e-12*want {
		t.Errorf("the drawn tooth is %.9f mm thick at the pitch circle, want the standard "+
			"pi*Module/2 = %.9f mm; a rounded count would not reach it", thickness, want)
	}

	d := involute.Derive(module, virtualTeeth, proxyPressureAngle)
	root := d.Root - sink
	if want := virtualRadius - dedendumModules*module - sink; math.Abs(root-want) > 1e-12*virtualRadius {
		t.Errorf("the root circle is at %.9f mm, want virtualPitchRadius - 1.25*Module - rootSink "+
			"= %.9f mm", root, want)
	}

	proofkit.Step(t, "the four circles")
	centre := s.CreatePoint(0, 0)
	centre.SetName("tooth centre")
	circle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(centre, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(root, false)
	tip := circle(d.Tip, true)
	circle(d.Base, true)
	circle(d.Pitch, true)

	proofkit.Step(t, "the tooth, drawn already rotated 180 degrees by the draw() angle")
	angle := math.Pi
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, virtualTeeth, proxyInvoluteSteps, angle)
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

	topX, topY := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, centre))

	spine := s.CreateLine(centre, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(centre, refEnd, d.Tip),
		sketch.NewVerticalDistance(centre, refEnd, 0),
	)
	reference := s.CreateLine(centre, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	proofkit.Step(t, "the ribs")
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prev := centre
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

	proofkit.Step(t, "the flank-to-root lines")
	sunk := involute.Dimensions{Pitch: d.Pitch, Base: d.Base, Root: root, Tip: d.Tip}
	if !sunk.Embedded() {
		foot := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := root*seed.X/n, root*seed.Y/n
			re := s.CreatePoint(rx, ry)
			s.CreateLine(re, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(centre, re, rx),
				sketch.NewVerticalDistance(centre, re, ry),
			)
		}
		foot(leftPts[0], left[0])
		foot(rightPts[0], right[0])
	}

	proofkit.Step(t, "anchor the tooth on the §2 tooth centre K' / L'")
	anchor := s.CreateReferencePoint(0, 0, projectionSource)
	s.AddConstraint(sketch.NewCoincident(centre, anchor))

	proofkit.Step(t, "the loop the profile search keys on")
	assertToothLoop(t, s, root, sunk.Embedded())
}

// assertToothLoop holds the drawn sketch to the loop
// find_profile_by_curve_counts selects: 2 NURBS, 2 arcs, and a line count
// DETERMINED by the embedded flag — 0 when embedded, 2 when not.
//
// The count is not a preference. An unrelated loop between the drawCircles
// circles can also carry 2 NURBS and 2 arcs with the OTHER line count, and
// selecting it makes the apex->profile loft fail with
// ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY.
func assertToothLoop(t testing.TB, s *sketch.Sketch, root float64, embedded bool) {
	t.Helper()
	solveHere(t, s)
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	teeth, discs := 0, 0
	regions := s.Profiles()
	for _, profile := range regions {
		nurbs, arcs, lines := entityCounts(profile.Entities)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			teeth++
		case nurbs == 0 && arcs == 1 && lines == 0:
			discs++
			if want := math.Pi * root * root; math.Abs(profile.Area-want) > 1e-6*want {
				t.Errorf("the root disc measures %.6f mm2, want the sunk root circle's %.6f mm2",
					profile.Area, want)
			}
		default:
			t.Errorf("unexpected region: %d NURBS, %d arcs, %d lines", nurbs, arcs, lines)
		}
		if !profile.Valid {
			t.Errorf("region with %d NURBS, %d arcs, %d lines is not extrudable", nurbs, arcs, lines)
		}
	}
	if teeth != 1 {
		t.Errorf("tooth loops of 2 NURBS, 2 arcs, %d lines: %d, want exactly 1 — the line count is "+
			"decided by the embedded flag, never accepted either way", wantLines, teeth)
	}
	if discs != 1 {
		t.Errorf("root discs: %d, want 1", discs)
	}
}

// entityCounts classifies a region's DISTINCT boundary entities the way
// find_profile_by_curve_counts classifies Fusion's profile curves: a fitted
// spline is a NURBS, the tooth-top arc and the root circle are each an arc, and
// a flank-to-root stub is a line.
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

// ---------------------------------------------------------------- profile hexagon

// stepGearProfileHexagon draws one gear's Profile sketch: the six §2 vertices
// recreated as new points at their exact positions, the closed hexagon drawn
// SHARING those points, and the points fixed AFTER the lines exist.
//
// That order is the whole recipe ([PB-PROJECT-NOT-FIXED]): a projected point is
// associative but still free, so a sketch hung off projections reads
// under-constrained even though every point is already in the right place; and
// fixing a bare point BEFORE it is consumed as a line endpoint does not leave
// the sketch fully constrained either. The hexagon's first edge is this gear's
// shaft axis for the revolve, the pattern, the bore plane and the meshing
// rotation, so it has to carry a trustworthy world position
// ([PB-WORLDGEO-CONSTRAINED]).
func stepGearProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := newLattice(t, p)
	side := l.pinion
	if p["gear"] != 0 {
		side = l.driving
	}
	order := []string{"A'", "G", "H", "C", "M", "N"}
	if p["gear"] != 0 {
		order = []string{"B'", "I", "J", "D", "O", "P"}
	}

	proofkit.Step(t, "%s Profile: recreate the six §2 vertices", side.label)
	verts := make([]*sketch.Point, 6)
	for i, v := range side.hexagon() {
		verts[i] = s.CreatePoint(v.X, v.Y)
		verts[i].SetName(order[i])
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, 6)
	for i := range verts {
		lines[i] = s.CreateLine(verts[i], verts[(i+1)%6])
		lines[i].SetName(order[i] + "->" + order[(i+1)%6])
	}

	proofkit.Step(t, "fix the endpoints, after the lines exist")
	for _, line := range lines {
		s.Fix(line.Start)
		s.Fix(line.End)
	}

	proofkit.Step(t, "the one loop the revolve takes")
	solveHere(t, s)
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("the %s Profile sketch closes %d regions, want exactly one hexagon loop — a second "+
			"identically-shaped loop is what drawing both gears in one sketch would leave to "+
			"disambiguate ([PB-SINGLE-PROFILE])", side.label, len(regions))
	}
	region := regions[0]
	if !region.Valid {
		t.Fatalf("the %s hexagon is not an extrudable region; a profile that has crossed its own "+
			"axis of revolution fails the revolve with ASM_WIRE_X_AXIS ([PB-REVOLVE])", side.label)
	}
	_, arcs, lineCount := entityCounts(region.Entities)
	if arcs != 0 || lineCount != 6 {
		t.Errorf("the %s hexagon's boundary is %d arcs and %d lines, want 6 lines",
			side.label, arcs, lineCount)
	}
	want := hexagonArea(side.section(l.apex))
	if math.Abs(region.Area-want) > 1e-7*want {
		t.Errorf("the %s hexagon measures %.9f mm2, want the closed form's %.9f mm2",
			side.label, region.Area, want)
	}

	// The first edge IS the shaft axis: both its endpoints sit on it, which is
	// what the revolve, the pattern and the bore plane all read.
	for _, v := range []vec{side.foot, side.ext2} {
		if r := side.radius(l.apex, v); r > 1e-7 {
			t.Errorf("the %s hexagon's first edge has an endpoint %.9f mm off the shaft axis; that "+
				"edge IS the axis for the revolve, the pattern, the bore plane and the meshing "+
				"rotation", side.label, r)
		}
	}
	// Every other vertex stands clear of the axis. N (P) never touches it — only
	// A' (B') does, and it is a foot rather than a corner — because the Toe
	// Radius is strictly positive.
	if r := side.radius(l.apex, side.toeIn); r <= 0 {
		t.Errorf("the %s inner toe corner sits ON the axis of revolution at radius %.9f mm; the "+
			"later conical split then fails with ASM_API_FAILED", side.label, r)
	}
}

// ---------------------------------------------------------------- bore sketch

// stepBoreSketch draws one gear's Bore sketch on the plane rooted at the shaft
// edge's start, so the sketch origin sits on the axis.
//
// The circle's centre is FIXED rather than coincident to the origin
// ([PB-CIRCLE-CENTER]): `addByCenterRadius` at (0,0,0) does not reuse the
// sketch's originPoint, its centre is a free point that happens to sit there,
// and an `addCoincident` to the origin has been observed to throw
// VCS_SKETCH_SOLVING_FAILED on a setByDistanceOnPath plane.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := newLattice(t, p)
	side := l.pinion
	if p["gear"] != 0 {
		side = l.driving
	}
	if !l.boreEnable {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no bore sketch is drawn on either gear")
	}
	proofkit.Step(t, "%s Bore: one circle of diameter %.6f mm, centred on the axis",
		side.label, side.boreDia)
	centre := s.CreatePoint(0, 0)
	centre.SetName("bore centre")
	s.Fix(centre)
	circle := s.CreateCircle(centre, side.boreDia/2)
	circle.SetName("bore circle")
	s.AddConstraint(sketch.NewDiameter(circle, side.boreDia))

	solveHere(t, s)
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("the Bore sketch closes %d regions, want the one circle", len(regions))
	}
	want := math.Pi * side.boreDia * side.boreDia / 4
	if math.Abs(regions[0].Area-want) > 1e-9*want {
		t.Errorf("the bore circle encloses %.9f mm2, want %.9f mm2 for diameter %.9f mm",
			regions[0].Area, want, side.boreDia)
	}
}
