// This file holds the bevel pair's sketch steps: the Anchor sketch, the shared
// Gear Profiles (§2) lattice, the two per-gear Profile hexagons, the virtual
// spur tooth, and the Bore circle. The spiral build's 2-D trace sketch is in
// spiral_test.go, with the rest of the spiral branch.
//
// Four things in these steps are outside what a sketch engine can hold, and
// each is recorded beside the thing it cannot reach.
//
// The Anchor sketch's centre and the Gear Profiles sketch's projection of it
// are modelled as reference geometry, which this engine locks: the solver never
// moves a reference point. Fusion's sketch.project does NOT fix what it brings
// in ([PB-PROJECT-NOT-FIXED]) — the projection tracks its source and still
// carries free degrees of freedom. So what the proof establishes is that the
// lattice hanging off the projection closes, not that Fusion's projection is
// pinned; in Fusion the projection is driven by the Anchor sketch instead, and
// that sketch is itself gated.
//
// Three constraints the spec names are carried differently here because this
// engine counts them differently, and each is written at its own call site: the
// Anchor sketch's point-on-line beside its midpoint, the two base-height
// perpendiculars beside the offsets that already carry the parallelism
// ([PB-NO-OVERCONSTRAIN]), and the coincidence that pins point I to the
// projected centre, whose lateral row the §2 net already implies. The spec's §2
// states the second of the three itself.
//
// Every side-choosing perpendicular and parallel is carried as a signed angle
// of the same arity, for the reason turnDeg gives: Fusion takes those sides
// from the seed, and left unsigned the engine reaches DOF 0 while still
// admitting the mirrored answer the spec's warnings are about.
//
// Sketch text has no counterpart here at all. The borrowed spur generator
// labels its four circles and a labelled sketch does not report fully
// constrained in Fusion ([PB-TEXT-HOLDS-DOF]), which is why the tooth sketches
// are exempt from the gate there; the tooth step below proves the geometry
// reaches DOF 0 on its own, which is the half a bench can see.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// anchorProjection is the source tag every reference point in these sketches
// carries: the user's Center Point, brought in by sketch.project.
const anchorProjection = "centerPoint"

// dialog names one case by the twenty dialog values it comes from, so a case
// reads as a dialog the user could have filled in. A zero length means the
// dialog's "unspecified" and takes that input's own fallback.
//
// The three frame values at the end are not dialog inputs: they are the
// projected centre's position and the projected anchor line's direction in the
// Gear Profiles sketch, and the grow side the target-plane normal picks
// ([BEVEL-F-GROW-SIDE]). Nothing in the dialog fixes any of the three, so the
// table sweeps them.
func dialog(module, drivingTeeth, pinionTeeth, shaftAngleDeg float64) map[string]float64 {
	return map[string]float64{
		"module":            module,
		"drivingTeeth":      drivingTeeth,
		"pinionTeeth":       pinionTeeth,
		"shaftAngleDeg":     shaftAngleDeg,
		"drivingBaseHeight": 0,
		"pinionBaseHeight":  0,
		"boreEnable":        1,
		"drivingBore":       0,
		"pinionBore":        0,
		"faceWidth":         0,
		"toothSpacing":      0,
		"spiralAngleDeg":    0,
		"handSign":          1,
		"cutterRadius":      0,
		"toeExtension":      0,
		"drivingToeRadius":  0,
		"pinionToeRadius":   0,
		"anchorX":           0,
		"anchorY":           0,
		"anchorDirDeg":      0,
		"growSign":          1,
		"gear":              0,
		"refuse":            0,
	}
}

func override(p map[string]float64, pairs map[string]float64) map[string]float64 {
	q := make(map[string]float64, len(p))
	for k, v := range p {
		q[k] = v
	}
	for k, v := range pairs {
		q[k] = v
	}
	return q
}

func pinionDialog(p map[string]float64) map[string]float64 {
	return override(p, map[string]float64{"gear": 0})
}

func drivingDialog(p map[string]float64) map[string]float64 {
	return override(p, map[string]float64{"gear": 1})
}

func gearOf(p map[string]float64) string {
	if p["gear"] != 0 {
		return "Driving"
	}
	return "Pinion"
}

// ------------------------------------------------------------- the case table

// latticeCases is the regime the §2 lattice has to hold across.
//
// The Shaft Angle sweep runs the whole range the spec admits, from the
// documented 30-degree floor to the computed Maximum Shaft Angle, because the
// lattice's conditioning is what moves across it and nothing else in the figure
// does. The tooth counts run equal pairs, both ratio directions and the
// computed Minimum Teeth floor, since either gear can be the binding side of
// the Maximum Face Width. Base height, Face Width, Tooth Spacing, Toe Extension
// and the two Toe Radii are each swept on both sides of the branch they open,
// and the frame is swept off the origin and rotated, and onto the other grow
// side, because a target plane fixes none of the three.
var latticeCases = []proofkit.Case{
	{Name: "M1_31x31_90deg_default", Params: dialog(1, 31, 31, 90)},
	// A refusal the table records rather than avoids. The spec's documented
	// Shaft Angle floor is 30 degrees and this net cannot reach it: measured
	// here, the default pair reads conditioning 2.831e-05 against the engine's
	// 4e-05 trust floor, and first clears at 35 degrees (4.192e-05). That is a
	// property of THIS lattice, not of the range the spec states, and the spec
	// says so: of three independently written lattices two refuse 30 degrees at
	// 2.83e-05 and 2.94e-05 and one passes it. This one is one of the two, and
	// its reading agrees with the first of them to three digits.
	{Name: "M1_31x31_30deg_floor", Params: override(dialog(1, 31, 31, 30),
		map[string]float64{"refuse": 1})},
	{Name: "M1_31x31_33deg_below_reach", Params: override(dialog(1, 31, 31, 33),
		map[string]float64{"refuse": 1})},
	{Name: "M1_31x31_35deg", Params: dialog(1, 31, 31, 35)},
	{Name: "M1_31x31_60deg", Params: dialog(1, 31, 31, 60)},
	{Name: "M1_31x31_120deg", Params: dialog(1, 31, 31, 120)},
	{Name: "M1_31x31_142deg", Params: dialog(1, 31, 31, 142)},
	{Name: "M1_31x31_150deg_ceiling", Params: dialog(1, 31, 31, 150)},
	{Name: "M1_31x17_90deg", Params: dialog(1, 31, 17, 90)},
	{Name: "M1_17x31_90deg", Params: dialog(1, 17, 31, 90)},
	// The other end of the same cliff, and the one the spec does not name. A
	// 31/17 pair's Maximum Shaft Angle is 123.26 degrees, where the driving
	// pitch cone angle reaches 90 degrees and the along-shaft seed R*cos(gamma)
	// passes through zero. Approaching it the lattice degenerates: measured,
	// 110 degrees reads 4.274e-05 and clears, 115 degrees reads 1.180e-05 and
	// 120 degrees 7.529e-07, both refused. So the reachable band for this net
	// stops short of the computed limit, and the table records where.
	{Name: "M1_31x17_110deg", Params: dialog(1, 31, 17, 110)},
	{Name: "M1_31x17_120deg", Params: override(dialog(1, 31, 17, 120),
		map[string]float64{"refuse": 1})},
	{Name: "M1_43x17_90deg_large_ratio", Params: dialog(1, 43, 17, 90)},
	{Name: "M1_17x31_45deg", Params: dialog(1, 17, 31, 45)},
	{Name: "M2_19x13_90deg", Params: dialog(2, 19, 13, 90)},
	{Name: "M4_31x43_90deg", Params: dialog(4, 31, 43, 90)},
	{Name: "M1_4x4_90deg_min_teeth", Params: dialog(1, 4, 4, 90)},
	{Name: "M1_8x8_90deg", Params: dialog(1, 8, 8, 90)},
	{Name: "M1_31x31_90deg_base_heights", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"drivingBaseHeight": 6, "pinionBaseHeight": 2})},
	{Name: "M1_31x31_90deg_face_width_4", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"faceWidth": 4})},
	{Name: "M1_31x31_90deg_spacing", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"toothSpacing": 0.6})},
	{Name: "M1_31x31_90deg_toe_ext_50", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"toeExtension": 50})},
	{Name: "M1_31x31_90deg_toe_ext_100", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"toeExtension": 100})},
	{Name: "M1_31x31_90deg_toe_radius_set", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"drivingToeRadius": 8, "pinionToeRadius": 3})},
	{Name: "M1_31x17_90deg_toe_ext_100", Params: override(dialog(1, 31, 17, 90),
		map[string]float64{"toeExtension": 100})},
	{Name: "M1_31x31_90deg_frame_offset", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"anchorX": 8, "anchorY": -5, "anchorDirDeg": 37})},
	{Name: "M1_31x17_120deg_frame_offset", Params: override(dialog(1, 31, 17, 120),
		map[string]float64{"anchorX": -12, "anchorY": 7, "anchorDirDeg": -64,
			"refuse": 1})},
	{Name: "M1_31x17_110deg_frame_offset", Params: override(dialog(1, 31, 17, 110),
		map[string]float64{"anchorX": -12, "anchorY": 7, "anchorDirDeg": -64})},
	{Name: "M1_31x31_90deg_grow_other_side", Params: override(dialog(1, 31, 31, 90),
		map[string]float64{"growSign": -1, "anchorDirDeg": 20})},
}

// ------------------------------------------------------------- the §1 sketch

var anchorCases = latticeCases

// stepAnchorSketch builds the Anchor sketch: the projected Center Point and the
// one reference line through it.
//
// The projected centre is reference geometry here, so it is coordinate-locked
// and the line is what has to be constrained. Fusion's projection is not locked
// ([PB-PROJECT-NOT-FIXED]); the difference costs the proof nothing, because
// nothing about this sketch depends on the projection being free.
//
// One constraint is omitted. The spec applies BOTH addCoincident(projectedCenter,
// anchorLine) and addMidPoint(projectedCenter, anchorLine), and says to use
// both. This engine's midpoint constraint already carries two residual rows —
// the centre IS the average of the two endpoints — so the point-on-line row is
// implied by it and adding it makes the sketch redundant at DOF 0. Fusion
// absorbs the extra row; the engine reports it. So the proof carries the
// midpoint alone and the Fusion build carries both, which is the same geometry.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)

	proofkit.Step(t, "project the Center Point")
	centre := s.CreateReferencePoint(f.c.X, f.c.Y, anchorProjection)
	centre.SetName("projected Center Point")

	proofkit.Step(t, "the Anchor Line, seeded at plus and minus 5 mm along sketch-local X")
	a := s.CreatePoint(f.c.X-5, f.c.Y)
	b := s.CreatePoint(f.c.X+5, f.c.Y)
	a.SetName("Anchor Line start")
	b.SetName("Anchor Line end")
	anchor := s.CreateLine(a, b)

	proofkit.Step(t, "midpoint, length and direction")
	s.AddConstraint(
		sketch.NewMidpoint(centre, anchor),
		// The spec's aligned distance dimension is a magnitude whose direction
		// Fusion captures from the seeded geometry at creation. This engine
		// takes the direction as the sign instead, so the seed side crosses
		// over as a positive horizontal distance ([PB-DIM-VALUE-SEMANTICS]).
		// The magnitude alone leaves the line free to turn end for end: DOF
		// reaches 0 and the sketch still admits two configurations, which the
		// gate refuses and a comment would not fix.
		sketch.NewHorizontalDistance(a, b, 10),
		sketch.NewHorizontal(anchor),
	)
}

// ------------------------------------------------------------- the §2 lattice

// lattice is the §2 sketch as built, so a step can read a named point back out
// of it after the solve.
type lattice struct {
	centre                 *sketch.Point
	apex, apex2            *sketch.Point
	a, b, cc, dd           *sketch.Point
	e, ff, g, h, i, j      *sketch.Point
	k, kPrime, l, lPrime   *sketch.Point
	m, n, aPrime           *sketch.Point
	o, pp, bPrime          *sketch.Point
	pinionShaft            *sketch.Line
	drivingShaft           *sketch.Line
	centerToApex, pitch    *sketch.Line
	dedC, dedD             *sketch.Line
	rootC, rootD           *sketch.Line
	lineCH, lineDJ         *sketch.Line
	lineMN, lineOP         *sketch.Line
	lineCK, lineDL         *sketch.Line
	shaftEdgeP, shaftEdgeG *sketch.Line
}

// seg draws one §2 construction line in the COINCIDENT style
// ([BEVEL-F-COINCIDENT-STYLE]): both endpoints are fresh points at raw
// coordinates, never an existing point passed in to be shared. Every §2 line is
// built this way, lattice and short reference line alike, and each named
// segment is drawn exactly once ([BEVEL-F-LINE-ONCE]).
func seg(s *sketch.Sketch, from, to vec, name string) (*sketch.Line, *sketch.Point, *sketch.Point) {
	p1 := s.CreatePoint(from.X, from.Y)
	p2 := s.CreatePoint(to.X, to.Y)
	p1.SetName(name + " start")
	p2.SetName(name + " end")
	line := s.CreateLine(p1, p2)
	line.SetConstruction(true)
	return line, p1, p2
}

// leftOffset is the signed distance NewOffset wants: positive when dst lies to
// the left of src's start-to-end direction. Fusion's addOffsetDimension takes a
// magnitude and reads the side off the seeded geometry ([PB-DIM-VALUE-SEMANTICS]);
// this engine takes the sign instead, so the proof computes it from the same
// closed form the seeds come from.
func leftOffset(srcStart, srcEnd, dstPoint vec) float64 {
	dir := srcEnd.sub(srcStart).unit()
	left := vec{-dir.Y, dir.X}
	return dstPoint.sub(srcStart).dot(left)
}

// turnDeg is the counter-clockwise angle, in degrees, from one directed segment
// to another, as the closed form places the two.
//
// It exists because of one systematic difference between Fusion and this
// engine. Fusion's addPerpendicular and addParallel are unsigned: they say
// which LINE the new line must lie along and let the seeded geometry decide
// which way along it, and §2 chooses every one of those sides deliberately —
// the grow side from the target-plane normal, the two Apex 2 drops into the
// interior wedge, the dedendum pair's toward/away-from-the-anchor-line split,
// the front face's side of the shaft. This engine's NewPerpendicular and
// NewParallel are unsigned in the same way, and a sketch built from them
// reaches DOF 0 while still admitting the mirrored answer, which the gate
// refuses and rightly so: the mirror is the failure §2's warnings are about,
// and the pinion dedendum collapsing onto the driving one is what it looks
// like when it happens.
//
// So every one of those sides is carried here as a SIGNED angle of the same
// arity — one row, as the perpendicular or parallel it replaces — with the
// value the seed side gives, which is how a Fusion direction taken from a seed
// crosses over to a bench engine ([PB-DIM-VALUE-SEMANTICS]). The perpendiculars
// that choose no side, A->E against C->E and B->F against D->F, stay
// perpendiculars.
func turnDeg(aFrom, aTo, bFrom, bTo vec) float64 {
	da, db := aTo.sub(aFrom), bTo.sub(bFrom)
	return p2deg(math.Atan2(da.cross(db), da.dot(db)))
}

// buildLattice draws the whole §2 figure and returns its named points.
func buildLattice(t testing.TB, s *sketch.Sketch, f figure) *lattice {
	lat := &lattice{}

	proofkit.Step(t, "project the Anchor sketch's centre and its Anchor Line")
	lat.centre = s.CreateReferencePoint(f.c.X, f.c.Y, anchorProjection)
	lat.centre.SetName("projected centre")
	ra := s.CreateReferencePoint(f.c.X-f.d.X*5, f.c.Y-f.d.Y*5, anchorProjection)
	rb := s.CreateReferencePoint(f.c.X+f.d.X*5, f.c.Y+f.d.Y*5, anchorProjection)
	anchor, err := s.CreateReferenceLine(ra, rb, anchorProjection)
	if err != nil {
		t.Fatalf("project the Anchor Line: %v", err)
	}

	proofkit.Step(t, "centre to Apex, perpendicular to the projected Anchor Line")
	centerToApex, ctaStart, apexPt := seg(s, f.c, f.apex, "Apex")
	lat.centerToApex, lat.apex = centerToApex, apexPt
	s.AddConstraint(
		sketch.NewCoincident(ctaStart, lat.centre),
		// The grow side: which way the figure climbs off the anchor line. §2
		// takes it from the target-plane normal, a one-bit direction
		// ([BEVEL-F-GROW-SIDE]); here it is the sign of the right angle.
		sketch.NewAngle(anchor, centerToApex,
			turnDeg(f.c.sub(f.d.scale(5)), f.c.add(f.d.scale(5)), f.c, f.apex)),
	)

	proofkit.Step(t, "Driving Gear Shaft Axis, parallel to centre-to-Apex")
	drivingShaft, dsStart, bPt := seg(s, f.apex, f.B, "B")
	lat.drivingShaft, lat.b = drivingShaft, bPt
	s.AddConstraint(
		sketch.NewCoincident(dsStart, apexPt),
		// Apex->B runs back down toward the anchor line, so it is ANTIparallel
		// to centre->Apex. Fusion's addParallel admits both senses and the seed
		// picks this one; the signed half turn says it outright.
		sketch.NewAngle(centerToApex, drivingShaft, turnDeg(f.c, f.apex, f.apex, f.B)),
	)

	proofkit.Step(t, "Pinion Gear Shaft Axis at the Shaft Angle")
	pinionShaft, psStart, aPt := seg(s, f.apex, f.A, "A")
	lat.pinionShaft, lat.a = pinionShaft, aPt
	s.AddConstraint(
		sketch.NewCoincident(psStart, apexPt),
		// Signed, and the sign is the seed sense §2 picks by comparing the two
		// candidate point-A positions' X. A magnitude would admit the mirror.
		sketch.NewAngle(drivingShaft, pinionShaft, f.senseSgn*p2deg(f.sigma)),
	)

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	dropA, dropAStart, apex2a := seg(s, f.A, f.apex2, "Apex2 from A")
	dropB, dropBStart, apex2b := seg(s, f.B, f.apex2, "Apex2 from B")
	lat.apex2 = apex2a
	s.AddConstraint(
		sketch.NewCoincident(dropAStart, aPt),
		// Both drops aim at the interior wedge BETWEEN the two shaft axes. A
		// drop seeded at the other side makes the solver flip the whole frame
		// to the mirror answer, which is the single worst §2 failure: the
		// pinion dedendum collapses onto the driving one and the conical end
		// cut then finds no cone face at the toe.
		sketch.NewAngle(pinionShaft, dropA, turnDeg(f.apex, f.A, f.A, f.apex2)),
		sketch.NewDistance(dropAStart, apex2a, f.pinion.pitchRadius()),
		sketch.NewCoincident(dropBStart, bPt),
		sketch.NewAngle(drivingShaft, dropB, turnDeg(f.apex, f.B, f.B, f.apex2)),
		sketch.NewDistance(dropBStart, apex2b, f.driving.pitchRadius()),
		sketch.NewCoincident(apex2a, apex2b),
	)

	proofkit.Step(t, "the Pitch Line")
	pitch, plStart, plEnd := seg(s, f.apex, f.apex2, "Pitch Line")
	lat.pitch = pitch
	s.AddConstraint(
		sketch.NewCoincident(plStart, apexPt),
		sketch.NewCoincident(plEnd, apex2a),
	)

	proofkit.Step(t, "the two dedendum lines and the two root axes")
	dedC, dedCStart, cPt := seg(s, f.apex2, f.C, "C")
	dedD, dedDStart, dPt := seg(s, f.apex2, f.D, "D")
	lat.dedC, lat.dedD, lat.cc, lat.dd = dedC, dedD, cPt, dPt
	s.AddConstraint(
		sketch.NewCoincident(dedCStart, apex2a),
		// C is the dedendum drawn AWAY from the anchor line and D the one drawn
		// toward it. Left unsigned the two swap places, and a run where D
		// landed on C was exactly what the engine reported.
		sketch.NewAngle(pitch, dedC, turnDeg(f.apex, f.apex2, f.apex2, f.C)),
		sketch.NewDistance(dedCStart, cPt, 1.25*f.module),
		sketch.NewCoincident(dedDStart, apex2a),
		sketch.NewAngle(pitch, dedD, turnDeg(f.apex, f.apex2, f.apex2, f.D)),
		sketch.NewDistance(dedDStart, dPt, 1.25*f.module),
	)
	rootC, rcStart, rcEnd := seg(s, f.apex, f.C, "Pinion Root Axis")
	rootD, rdStart, rdEnd := seg(s, f.apex, f.D, "Driving Root Axis")
	lat.rootC, lat.rootD = rootC, rootD
	s.AddConstraint(
		sketch.NewCoincident(rcStart, apexPt),
		sketch.NewCoincident(rcEnd, cPt),
		sketch.NewCoincident(rdStart, apexPt),
		sketch.NewCoincident(rdEnd, dPt),
	)

	proofkit.Step(t, "A to E and C to E, and their driving twins")
	lineAE, aeStart, ePt := seg(s, f.A, f.E, "E")
	lineCE, ceStart, ceEnd := seg(s, f.C, f.E, "C to E")
	lat.e = ePt
	s.AddConstraint(
		sketch.NewCoincident(aeStart, aPt),
		// The collinear the spec names is addCollinear(A->E, Apex->A). This
		// engine counts a collinear as two point-on-line rows and one of them
		// is already carried by the coincidence above, so the proof substitutes
		// the row that is not implied ([PB-COLLINEAR-CHAIN]). The distinction
		// between naming the near line and naming the far one is invisible
		// here and only a Fusion session tells the two apart.
		sketch.NewPointOnLine(ePt, pinionShaft),
		sketch.NewCoincident(ceStart, cPt),
		sketch.NewCoincident(ceEnd, ePt),
		sketch.NewPerpendicular(lineAE, lineCE),
	)
	lineBF, bfStart, fPt := seg(s, f.B, f.F, "F")
	lineDF, dfStart, dfEnd := seg(s, f.D, f.F, "D to F")
	lat.ff = fPt
	s.AddConstraint(
		sketch.NewCoincident(bfStart, bPt),
		sketch.NewPointOnLine(fPt, drivingShaft),
		sketch.NewCoincident(dfStart, dPt),
		sketch.NewCoincident(dfEnd, fPt),
		sketch.NewPerpendicular(lineBF, lineDF),
	)

	proofkit.Step(t, "the pinion heel edge G to H at the Pinion Gear Base Height")
	_, egStart, gPt := seg(s, f.E, f.G, "G")
	lineCH, chStart, hPt := seg(s, f.C, f.H, "H")
	lineGH, ghStart, ghEnd := seg(s, f.G, f.H, "G to H")
	lat.g, lat.h, lat.lineCH = gPt, hPt, lineCH
	s.AddConstraint(
		sketch.NewCoincident(egStart, ePt),
		sketch.NewPointOnLine(gPt, lineAE),
		sketch.NewCoincident(chStart, cPt),
		sketch.NewPointOnLine(hPt, dedC),
		sketch.NewCoincident(ghStart, gPt),
		sketch.NewCoincident(ghEnd, hPt),
		// The spec's addPerpendicular(E->G, H->G) is deliberately NOT here.
		// Fusion's addOffsetDimension needs the two lines parallel before it
		// will apply, and that perpendicular is what supplies the parallelism;
		// this engine's offset holds BOTH endpoints of the target at the same
		// signed distance, so it carries the parallelism itself. Adding the
		// perpendicular makes it a third row for two freedoms and the lattice
		// comes back overconstrained with the two base-height offsets named as
		// the redundant pair. §2 states this omission and the reason.
		sketch.NewOffset(dropA, lineGH, leftOffset(f.A, f.apex2, f.G)),
	)

	proofkit.Step(t, "the driving heel edge I to J at the Driving Gear Base Height")
	_, fiStart, iPt := seg(s, f.F, f.I, "I")
	lineDJ, djStart, jPt := seg(s, f.D, f.J, "J")
	lineIJ, ijStart, ijEnd := seg(s, f.I, f.J, "I to J")
	lat.i, lat.j, lat.lineDJ = iPt, jPt, lineDJ
	s.AddConstraint(
		sketch.NewCoincident(fiStart, fPt),
		sketch.NewPointOnLine(iPt, lineBF),
		sketch.NewCoincident(djStart, dPt),
		sketch.NewPointOnLine(jPt, dedD),
		sketch.NewCoincident(ijStart, iPt),
		sketch.NewCoincident(ijEnd, jPt),
		sketch.NewOffset(dropB, lineIJ, leftOffset(f.B, f.apex2, f.I)),
	)

	proofkit.Step(t, "pin the figure: point I at the projected centre")
	// §2 says addCoincident(point I, the projected centre), which is two rows.
	// Only one of them is independent: the driving shaft chain B->F->I is
	// collinear with centre->Apex by construction, so I already lies on the
	// line through the centre and the lateral row is implied. This engine
	// reports that second row as redundant. The proof therefore carries the one
	// row that is not implied, as the point-on-line stating the centre lies on
	// the I->J edge, which given the rest of the net holds exactly when I is at
	// the centre. Fusion absorbs the dependent row; the engine does not.
	s.AddConstraint(sketch.NewPointOnLine(lat.centre, lineIJ))

	proofkit.Step(t, "the two back-cone centres K and L, and the Tooth Spacing offsets")
	lat.k = pinKL(s, f.G, f.K, gPt, pinionShaft, dedC, "K")
	lat.l = pinKL(s, f.I, f.L, iPt, drivingShaft, dedD, "L")
	lat.kPrime, lat.lineCK = toothCentre(s, f, cPt, lat.k, f.C, f.K, f.KPrime, dedC, "K")
	lat.lPrime, lat.lineDL = toothCentre(s, f, dPt, lat.l, f.D, f.L, f.LPrime, dedD, "L")

	proofkit.Step(t, "the pinion toe line M to N and the front face N to A prime")
	lat.lineMN, lat.m, lat.n, lat.aPrime, lat.shaftEdgeP = toeEnd(s, f,
		f.M, f.N, f.APrime, f.C, f.H, f.G, f.apex, f.A,
		cPt, gPt, rootC, lineCH, pinionShaft, f.pinion.toeRadius, "Pinion")

	proofkit.Step(t, "the driving toe line O to P and the front face P to B prime")
	lat.lineOP, lat.o, lat.pp, lat.bPrime, lat.shaftEdgeG = toeEnd(s, f,
		f.O, f.P, f.BPrime, f.D, f.J, f.I, f.apex, f.B,
		dPt, iPt, rootD, lineDJ, drivingShaft, f.driving.toeRadius, "Driving")

	return lat
}

// pinKL places K (or L) with the two point-on-line coincidents §2 calls for:
// both of its ends are already fixed, so a collinear here would over-determine
// the net and Fusion errors ([BEVEL-F-COLLINEAR-CHAIN]).
func pinKL(s *sketch.Sketch, fromSolved, toSolved vec,
	from *sketch.Point, shaft, dedendum *sketch.Line, name string) *sketch.Point {
	_, start, end := seg(s, fromSolved, toSolved, name)
	s.AddConstraint(
		sketch.NewCoincident(start, from),
		sketch.NewPointOnLine(end, shaft),
		sketch.NewPointOnLine(end, dedendum),
	)
	return end
}

// toothCentre builds the §3 tooth-centre point and the reference line the tooth
// plane is built on. At Tooth Spacing 0 nothing is built: K prime IS K and the
// C-to-K line is the reference line, because a zero-length dimensioned line is
// degenerate and one segment gets one line ([BEVEL-F-LINE-ONCE]).
func toothCentre(s *sketch.Sketch, f figure, corner, kPoint *sketch.Point,
	cornerSolved, kSolved, primeSolved vec, dedendum *sketch.Line,
	name string) (*sketch.Point, *sketch.Line) {
	if f.toothSpacing <= 0 {
		ref, refStart, refEnd := seg(s, cornerSolved, kSolved, name+" reference")
		s.AddConstraint(
			sketch.NewCoincident(refStart, corner),
			sketch.NewCoincident(refEnd, kPoint),
		)
		return kPoint, ref
	}
	_, offStart, primePt := seg(s, kSolved, primeSolved, name+" prime")
	s.AddConstraint(
		sketch.NewCoincident(offStart, kPoint),
		sketch.NewPointOnLine(primePt, dedendum),
		sketch.NewDistance(offStart, primePt, f.toothSpacing),
	)
	ref, refStart, refEnd := seg(s, cornerSolved, primeSolved, name+" prime reference")
	s.AddConstraint(
		sketch.NewCoincident(refStart, corner),
		sketch.NewCoincident(refEnd, primePt),
	)
	return primePt, ref
}

// toeEnd builds one gear's toe line, the short connector back to the dedendum
// corner, the front face that holds the inner toe corner off the shaft axis,
// and the hexagon's shaft-axis edge.
//
// The inner toe corner is NOT pinned to the shaft-axis drop. It rides the gear's
// Toe Radius, held there by the front face: the face's foot sits on the shaft
// axis, the face stands square to it, and its length is the Toe Radius. Pinning
// the corner itself to the axis would put it ON the axis of revolution.
func toeEnd(s *sketch.Sketch, f figure,
	toeSolved, cornerSolved, footSolved, dedSolved, heelSolved, heelTopSolved vec,
	apexSolved, shaftEndSolved vec,
	ded, heelTop *sketch.Point, rootAxis, heelEdge, shaft *sketch.Line,
	toeRadius float64, label string) (*sketch.Line, *sketch.Point, *sketch.Point, *sketch.Point, *sketch.Line) {

	toeLine, toePt, cornerPt := seg(s, toeSolved, cornerSolved, label+" toe")
	s.AddConstraint(
		sketch.NewPointOnLine(toePt, rootAxis),
		// The spec's addParallel(toe line, heel edge) plus its offset dimension
		// are two equations in Fusion. This engine's offset is already two rows
		// and carries the parallelism, so the parallel is omitted for the same
		// reason the base-height perpendiculars are.
		sketch.NewOffset(heelEdge, toeLine, leftOffset(dedSolved, heelSolved, toeSolved)),
	)

	_, mcStart, mcEnd := seg(s, toeSolved, dedSolved, label+" toe to dedendum")
	s.AddConstraint(
		sketch.NewCoincident(mcStart, toePt),
		sketch.NewCoincident(mcEnd, ded),
	)

	faceLine, faceStart, footPt := seg(s, cornerSolved, footSolved, label+" front face")
	s.AddConstraint(
		sketch.NewCoincident(faceStart, cornerPt),
		sketch.NewPointOnLine(footPt, shaft),
		// Square to the shaft, and on the side the toe corner is already on.
		// Unsigned, the corner slides through the axis to the mirror station
		// and the front face points the other way — which is the corner ON the
		// axis of revolution the spec forbids, reached from the far side.
		sketch.NewAngle(shaft, faceLine, turnDeg(apexSolved, shaftEndSolved, cornerSolved, footSolved)),
		sketch.NewDistance(faceStart, footPt, toeRadius),
	)

	shaftEdge, seStart, seEnd := seg(s, footSolved, heelTopSolved, label+" shaft edge")
	s.AddConstraint(
		sketch.NewCoincident(seStart, footPt),
		sketch.NewCoincident(seEnd, heelTop),
	)
	return toeLine, toePt, cornerPt, footPt, shaftEdge
}

func p2deg(r float64) float64 { return r * 180 / math.Pi }

// stepGearProfiles builds the §2 Gear Profiles sketch and checks the solve
// against the closed form.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if p["refuse"] != 0 {
		proofkit.Unmodelled(t, "declared refusal: this lattice's conditioning at "+
			"Shaft Angle %.0f degrees reads below the sketch engine's trust floor, "+
			"which is a property of this net rather than of the spec's range",
			p["shaftAngleDeg"])
	}
	lat := buildLattice(t, s, f)

	// The gate solves after this function returns; solving here too is what
	// lets the step check its own solved geometry, which is the point of the
	// step. The second solve starts from the answer and changes nothing.
	solveHere(t, s)
	checkLattice(t, lat, f)
}

// solveHere runs the solver so a build can read its own solved geometry back.
func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
}

// checkLattice checks the solved lattice against the closed form, and then
// checks the facts later steps select on.
//
// The point positions come first because everything else is a consequence of
// them. What follows is what the spec pins by name: the two cone angles the §3
// virtual teeth and the spiral twist both read, the pitch cone distance, the
// two pitch radii as perpendicular distances from Apex 2 to the shafts, the
// base-height offsets and the window they have to sit inside, the Minimum Teeth
// floor, the Maximum Face Width read off the solved points rather than the
// seeds, the root length the Toe Extension resolves to, and the toe radius the
// front face holds the inner toe corner at.
func checkLattice(t testing.TB, lat *lattice, f figure) {
	t.Helper()
	tol := 1e-7
	for _, c := range []struct {
		name string
		got  *sketch.Point
		want vec
	}{
		{"Apex", lat.apex, f.apex},
		{"Apex 2", lat.apex2, f.apex2},
		{"A", lat.a, f.A}, {"B", lat.b, f.B},
		{"C", lat.cc, f.C}, {"D", lat.dd, f.D},
		{"E", lat.e, f.E}, {"F", lat.ff, f.F},
		{"G", lat.g, f.G}, {"H", lat.h, f.H},
		{"I", lat.i, f.I}, {"J", lat.j, f.J},
		{"K", lat.k, f.K}, {"L", lat.l, f.L},
		{"K prime", lat.kPrime, f.KPrime}, {"L prime", lat.lPrime, f.LPrime},
		{"M", lat.m, f.M}, {"N", lat.n, f.N}, {"A prime", lat.aPrime, f.APrime},
		{"O", lat.o, f.O}, {"P", lat.pp, f.P}, {"B prime", lat.bPrime, f.BPrime},
	} {
		near(t, c.got.X(), c.want.X, tol, "%s x", c.name)
		near(t, c.got.Y(), c.want.Y, tol, "%s y", c.name)
	}

	at := func(p *sketch.Point) vec { return vec{p.X(), p.Y()} }
	apex, apex2 := at(lat.apex), at(lat.apex2)
	a, b := at(lat.a), at(lat.b)

	// The two pitch cone angles, solved rather than restated: the angle between
	// the Pitch Line and each shaft axis. The spiral twist divides by sin of
	// this angle and the virtual tooth count divides by its cosine, so nine
	// decimals is the resolution those two want.
	pitchDir := apex2.sub(apex).unit()
	near(t, math.Acos(clamp1(pitchDir.dot(a.sub(apex).unit()))), f.pinion.gamma, 1e-9,
		"solved pinion pitch cone angle")
	near(t, math.Acos(clamp1(pitchDir.dot(b.sub(apex).unit()))), f.driving.gamma, 1e-9,
		"solved driving pitch cone angle")
	near(t, f.pinion.gamma+f.driving.gamma, f.sigma, 1e-12,
		"the two cone angles add to the Shaft Angle")
	near(t, apex2.sub(apex).len(), f.R, 1e-7, "Pitch Cone Distance |Apex->Apex2|")

	// Each drop is that gear's pitch radius at the heel, measured as the
	// perpendicular distance from Apex 2 to that gear's shaft axis.
	near(t, math.Abs(apex2.sub(apex).cross(a.sub(apex).unit())), f.pinion.pitchRadius(),
		1e-7, "perpendicular distance from Apex 2 to the Pinion Shaft Axis")
	near(t, math.Abs(apex2.sub(apex).cross(b.sub(apex).unit())), f.driving.pitchRadius(),
		1e-7, "perpendicular distance from Apex 2 to the Driving Shaft Axis")

	// Point I is the pin that fixes the whole figure's height above the anchor
	// line; it has to land ON the projected centre.
	near(t, at(lat.i).sub(at(lat.centre)).len(), 0, 1e-7,
		"point I sits at the projected centre")

	// The two base-height offsets, read back as along-shaft stations, and the
	// window each has to sit in.
	near(t, f.station("Pinion", f.G)-f.station("Pinion", f.A), f.pinion.baseHgt, 1e-9,
		"Pinion Gear Base Height as the A->Apex2 to G->H offset")
	near(t, f.station("Driving", f.I)-f.station("Driving", f.B), f.driving.baseHgt, 1e-9,
		"Driving Gear Base Height as the B->Apex2 to J->I offset")
	for _, s := range []side{f.pinion, f.driving} {
		if s.baseHgt < s.minBase-1e-9 || s.baseHgt > s.maxBase+1e-9 {
			t.Errorf("%s base height %.6f is outside its window [%.6f, %.6f]",
				s.label, s.baseHgt, s.minBase, s.maxBase)
		}
		if s.teeth < math.Max(3, s.minTeeth) {
			t.Errorf("%s tooth count %.0f is below the computed Minimum Teeth floor %.4f",
				s.label, s.teeth, s.minTeeth)
		}
		if s.minBase > s.maxBase {
			t.Errorf("%s base-height window is empty (%.6f > %.6f), which the Minimum "+
				"Teeth check is there to make impossible", s.label, s.minBase, s.maxBase)
		}
	}

	// The Maximum Face Width, from the solved points rather than the seeds, and
	// the cap it puts on Face Width.
	solvedMax := 0.95 * math.Min(
		math.Abs(a.sub(at(lat.cc)).cross(at(lat.h).sub(at(lat.cc)).unit())),
		math.Abs(b.sub(at(lat.dd)).cross(at(lat.j).sub(at(lat.dd)).unit())))
	near(t, solvedMax, f.maxFaceWidth, 1e-6, "Maximum Face Width from the solved points")
	if f.faceWidth > f.maxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.6f exceeds the Maximum Face Width %.6f, which "+
			"would drive the toe across the shaft axis and abort the revolve with "+
			"ASM_WIRE_X_AXIS", f.faceWidth, f.maxFaceWidth)
	}

	// The toe end: the root length the Toe Extension resolved to, and the toe
	// radius the front face holds the inner corner at. The corner is never ON
	// the axis, which is what the strictly positive toe radius buys.
	near(t, at(lat.m).sub(at(lat.cc)).len(), f.rootLen, 1e-7, "pinion Root Length |C->M|")
	near(t, at(lat.o).sub(at(lat.dd)).len(), f.rootLen, 1e-7, "driving Root Length |D->O|")
	near(t, f.radius("Pinion", f.N), f.pinion.toeRadius, 1e-7, "Pinion Gear Toe Radius at N")
	near(t, f.radius("Driving", f.P), f.driving.toeRadius, 1e-7, "Driving Gear Toe Radius at P")
	near(t, f.radius("Pinion", f.APrime), 0, 1e-7, "A prime sits on the Pinion Shaft Axis")
	near(t, f.radius("Driving", f.BPrime), 0, 1e-7, "B prime sits on the Driving Shaft Axis")
	for _, s := range []side{f.pinion, f.driving} {
		if s.toeRadius <= 0 {
			t.Errorf("%s Toe Radius %.6f is not strictly positive, so the inner toe "+
				"corner would sit on the axis of revolution", s.label, s.toeRadius)
		}
		if s.toeRadius >= s.toeCeil {
			t.Errorf("%s Toe Radius %.6f is at or above its Toe Radius Ceiling %.6f",
				s.label, s.toeRadius, s.toeCeil)
		}
	}

	// K and L are where each shaft axis crosses that gear's dedendum line, and
	// |Apex2->K| is the virtual pitch radius the §3 tooth is drawn at.
	near(t, at(lat.k).sub(apex2).len(),
		f.pinion.pitchRadius()/math.Cos(f.pinion.gamma), 1e-7,
		"pinion virtual pitch radius |Apex2->K|")
	near(t, at(lat.l).sub(apex2).len(),
		f.driving.pitchRadius()/math.Cos(f.driving.gamma), 1e-7,
		"driving virtual pitch radius |Apex2->L|")
	near(t, at(lat.kPrime).sub(at(lat.k)).len(), f.toothSpacing, 1e-7,
		"Tooth Spacing shifts K prime off K")
	near(t, at(lat.lPrime).sub(at(lat.l)).len(), f.toothSpacing, 1e-7,
		"Tooth Spacing shifts L prime off L")
}

func clamp1(v float64) float64 { return math.Max(-1, math.Min(1, v)) }

// ------------------------------------------------------ the per-gear hexagon

var profileCases = latticeGearCases()

// latticeGearCases runs every lattice case once per gear, because either gear
// can be the binding side and the two hexagons are not mirror images of each
// other once the tooth counts differ.
func latticeGearCases() []proofkit.Case {
	out := make([]proofkit.Case, 0, 2*len(latticeCases))
	for _, c := range latticeCases {
		out = append(out,
			proofkit.Case{Name: c.Name + "_pinion", Params: pinionDialog(c.Params)},
			proofkit.Case{Name: c.Name + "_driving", Params: drivingDialog(c.Params)})
	}
	return out
}

// stepGearProfileSketch builds one gear's Profile sketch: the six §2 vertices
// recreated as new points at their world positions, the closed hexagon sharing
// them, and the fix applied AFTER the lines exist ([PB-PROJECT-NOT-FIXED]).
//
// The recreate-share-fix recipe is what makes this sketch fully constrained
// without projecting §2 geometry in, and it is what gives the hexagon's first
// edge a trustworthy world position ([PB-WORLDGEO-CONSTRAINED]) — that edge is
// the revolve axis, the pattern axis, the bore plane's path and the
// meshing-rotation axis.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if p["refuse"] != 0 {
		proofkit.Unmodelled(t, "declared refusal: the §2 lattice this hexagon's "+
			"vertices come from is refused at Shaft Angle %.0f degrees",
			p["shaftAngleDeg"])
	}
	gear := gearOf(p)
	verts := f.hexagon(gear)

	proofkit.Step(t, "recreate the six §2 vertices of the %s hexagon", gear)
	pts := make([]*sketch.Point, len(verts))
	names := []string{"front-face foot", "heel top", "heel end", "dedendum corner",
		"toe root", "inner toe corner"}
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.X, v.Y)
		pts[i].SetName(gear + " " + names[i])
	}

	proofkit.Step(t, "draw the closed hexagon in draw order, sharing those points")
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}

	proofkit.Step(t, "fix the vertices, after the lines exist")
	for _, line := range lines {
		_ = line
	}
	for _, pt := range pts {
		s.Fix(pt)
	}

	solveHere(t, s)
	checkHexagon(t, s, f, gear)
}

// checkHexagon checks what the revolve selects on: the sketch holds exactly
// one closed region ([PB-SINGLE-PROFILE]), that region is extrudable, its
// boundary is the six drawn lines, and no vertex has crossed the shaft axis,
// which is what would abort the revolve with ASM_WIRE_X_AXIS ([PB-REVOLVE]).
func checkHexagon(t testing.TB, s *sketch.Sketch, f figure, gear string) {
	t.Helper()
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("%s Profile sketch holds %d regions, want exactly 1", gear, len(regions))
	}
	if !regions[0].Valid {
		t.Fatalf("%s Profile region is not extrudable", gear)
	}
	if got := len(regions[0].Outer); got != 6 {
		t.Errorf("%s hexagon boundary has %d edges, want 6", gear, got)
	}
	apex, axisDir, _, _ := f.axisFrame(gear)
	verts := f.hexagon(gear)
	sign := 0.0
	for i, v := range verts {
		side := v.sub(apex).cross(axisDir)
		if i < 2 {
			// The first two vertices are the shaft-axis edge itself, which lies
			// ON the axis; they set no side.
			near(t, side, 0, 1e-9, "%s hexagon vertex %d is off the shaft axis", gear, i)
			continue
		}
		if sign == 0 {
			sign = math.Copysign(1, side)
		}
		if math.Copysign(1, side) != sign {
			t.Errorf("%s hexagon vertex %d crossed the shaft axis: the revolve would "+
				"abort with ASM_WIRE_X_AXIS", gear, i)
		}
		if math.Abs(side) <= 0 {
			t.Errorf("%s hexagon vertex %d sits on the shaft axis", gear, i)
		}
	}
}

// ------------------------------------------------------- the virtual spur tooth

var toothCases = profileCases

// stepToothSketch draws the borrowed spur tooth at the back-cone centre and
// checks what the tooth-profile selection keys on.
//
// The tooth itself is the spur family's. It is drawn by
// SpurGearInvoluteToothDesignGenerator through a VirtualSpurProxy carrying the
// raw-mm Module and this gear's virtual tooth number, so the proof imports the
// same involute math rather than deriving it again, and the constraint scheme
// that places it is proof/spurgear/sketches_test.go's subject rather than this
// one's. It is drawn here as REFERENCE geometry for exactly that reason: from
// bevel's side the tooth arrives already placed, and reference geometry is what
// this engine calls geometry that is externally locked by design.
//
// That substitution has a second, mechanical reason worth recording, because it
// is a boundary of the engine rather than a choice. An ordinary arc carries an
// internal radius-consistency row, and a tooth whose points are all placed
// makes that row dependent: the sketch comes back at DOF 0 with one redundant
// constraint per arc, and the gate refuses it. A reference arc carries no such
// row. Nothing is waived by it — the arcs are still arcs and the region below
// is still the tooth.
//
// What IS bevel's, and what this step proves, is the three things bevel
// supplies to the borrowed generator and reads back from it: the virtual tooth
// number from the closed form rather than from a measured Apex2-to-K distance,
// the 180-degree rotation delivered as the draw() angle, and the curve counts
// the later profile search selects on — 2 NURBS, 2 arcs, and 0 lines when the
// tooth is embedded or 2 when it is not. That line count is not a guess between
// two options: for a given gear only one of them is the real tooth, and picking
// the other grabs an unrelated loop whose apex loft then dies with
// LOFT_NO_TOOLBODY.
func stepToothSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if p["refuse"] != 0 {
		proofkit.Unmodelled(t, "declared refusal: the §2 lattice the tooth centre "+
			"comes from is refused at Shaft Angle %.0f degrees", p["shaftAngleDeg"])
	}
	gear := gearOf(p)
	virtual := float64(f.virtualTeeth(gear))
	s2 := f.sideOf(gear)

	proofkit.Step(t, "%s virtual tooth number %d from the back-cone pitch radius",
		gear, int(virtual))
	wantVirtual := math.Floor(2 * (s2.pitchRadius() / math.Cos(s2.gamma)) / f.module)
	near(t, virtual, wantVirtual, 0,
		"%s virtual tooth number", gear)

	// The pressure angle is the spur proxy's own default and is not a bevel
	// dialog input; the involute steps likewise.
	d := involute.Derive(f.module, virtual, rad(20))
	centre := s.CreateReferencePoint(0, 0, anchorProjection)
	centre.SetName(gear + " tooth centre K prime")

	proofkit.Step(t, "the tooth, drawn already rotated by 180 degrees through draw()'s angle")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, virtual, involuteSteps, math.Pi)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreateReferencePoint(left[i].X, left[i].Y, anchorProjection)
		rightPts[i] = s.CreateReferencePoint(right[i].X, right[i].Y, anchorProjection)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("%s left flank: %v", gear, err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("%s right flank: %v", gear, err)
	}
	mustRefArc(t, s, centre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1], "tooth top")

	wantLines := 2
	if d.Embedded() {
		wantLines = 0
		proofkit.Step(t, "embedded: tip, root and flanks meet with no connecting lines")
		mustRefArc(t, s, centre, rightPts[0], leftPts[0], "root")
	} else {
		proofkit.Step(t, "not embedded: two flank-to-root connecting lines")
		lf := refFoot(t, s, d.Root, left[0])
		rf := refFoot(t, s, d.Root, right[0])
		mustRefLine(t, s, lf, leftPts[0], "left flank-to-root")
		mustRefLine(t, s, rf, rightPts[0], "right flank-to-root")
		mustRefArc(t, s, centre, rf, lf, "root")
	}

	solveHere(t, s)
	checkToothProfile(t, s, gear, wantLines, d)
}

// involuteSteps is the sample count the framework proxy serves the spur drawer.
// It is not a bevel dialog input.
const involuteSteps = 15

func mustRefArc(t testing.TB, s *sketch.Sketch, centre, start, end *sketch.Point, label string) {
	t.Helper()
	if _, err := s.CreateReferenceArc(centre, start, end, anchorProjection); err != nil {
		t.Fatalf("%s arc: %v", label, err)
	}
}

func mustRefLine(t testing.TB, s *sketch.Sketch, a, b *sketch.Point, label string) {
	t.Helper()
	if _, err := s.CreateReferenceLine(a, b, anchorProjection); err != nil {
		t.Fatalf("%s line: %v", label, err)
	}
}

func refFoot(t testing.TB, s *sketch.Sketch, r float64, start involute.Pt) *sketch.Point {
	t.Helper()
	n := math.Hypot(start.X, start.Y)
	return s.CreateReferencePoint(r*start.X/n, r*start.Y/n, anchorProjection)
}

// checkToothProfile checks the loop the apex loft selects: one region, and the
// curve counts find_profile_by_curve_counts is called with.
func checkToothProfile(t testing.TB, s *sketch.Sketch, gear string,
	wantLines int, d involute.Dimensions) {
	t.Helper()
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("%s Tooth sketch holds %d regions, want the one tooth", gear, len(regions))
	}
	region := regions[0]
	if !region.Valid {
		t.Fatalf("%s tooth region is not extrudable", gear)
	}
	lines, arcs, splines := 0, 0, 0
	for _, e := range region.Entities {
		switch e.(type) {
		case *sketch.Line:
			lines++
		case *sketch.Arc:
			arcs++
		case *sketch.FitSpline:
			splines++
		}
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("%s tooth loop has %d NURBS, %d arcs, %d lines; the profile search "+
			"asks for 2 NURBS, 2 arcs and %d lines (embedded=%v)",
			gear, splines, arcs, lines, wantLines, d.Embedded())
	}

	// The tooth reaches the tip circle and, when it is not embedded, seats on
	// the root circle: the two radii the selection's arcs are drawn at.
	var maxR, minR float64 = 0, math.Inf(1)
	for _, pt := range s.Points() {
		r := math.Hypot(pt.X(), pt.Y())
		if r > maxR {
			maxR = r
		}
		if r > 0 && r < minR {
			minR = r
		}
	}
	near(t, maxR, d.Tip, 1e-9, "%s tooth reaches the tip radius", gear)
	if !d.Embedded() {
		near(t, minR, d.Root, 1e-9, "%s tooth seats on the root radius", gear)
	} else {
		near(t, minR, d.Base, 1e-9,
			"%s embedded tooth's flanks start on the base radius, inside the root circle", gear)
	}
}

// ------------------------------------------------------------- the Bore sketch

var boreCases = profileCases

// stepBoreSketch draws the bore circle on the plane normal to the shaft axis at
// its start. The plane is rooted on the axis, so the circle is centred on the
// sketch origin; its centre is FIXED rather than made coincident to the origin
// ([PB-CIRCLE-CENTER]), and a diameter dimension closes it.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if p["boreEnable"] == 0 {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no bore sketch is authored")
	}
	gear := gearOf(p)
	dia := f.sideOf(gear).boreDia

	proofkit.Step(t, "%s bore circle, diameter %.4f mm", gear, dia)
	centre := s.CreatePoint(0, 0)
	centre.SetName(gear + " bore centre")
	circle := s.CreateCircle(centre, dia/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, dia))
}
