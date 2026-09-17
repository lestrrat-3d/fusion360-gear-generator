package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// §1 Anchor sketch
// ---------------------------------------------------------------------------

// stepAnchorSketch proves the Anchor sketch closes: the projected centre bisects
// the anchor line, the line carries a length, and its direction is pinned
// sketch-locally rather than against a world axis ([PB-REFLINE-DIRECTION]).
//
// Substitution, and what it costs: Fusion needs BOTH addCoincident(projectedCentre,
// anchorLine) and addMidPoint, because its midpoint constraint alone leaves the
// point free to slide off the line. This engine's NewMidpoint is a two-row
// constraint that already pins both coordinates, so a point-on-line row beside it
// is a third row for two freedoms and the lattice comes back redundant. The proof
// therefore carries the midpoint alone. The cost is that nothing here shows the
// coincident is required in Fusion; only a Fusion session does.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user centre point onto the target plane")
	centre := s.CreateReferencePoint(0, 0, "centerPoint")
	centre.SetName("projected centre")

	proofkit.Step(t, "the Anchor Line, seeded at exactly ±5 mm from the projected centre")
	start := s.CreatePoint(-5, 0)
	end := s.CreatePoint(5, 0)
	start.SetName("anchor start")
	end.SetName("anchor end")
	line := s.CreateLine(start, end)
	line.SetName("Anchor Line")

	s.AddConstraint(sketch.NewMidpoint(centre, line))
	s.AddConstraint(sketch.NewHorizontal(line))
	// Fusion's aligned distance dimension is a magnitude whose direction is captured
	// from the seeded geometry, which leaves the line free to solve end-for-end: DOF
	// 0 with two configurations, and the ambiguity probe names both. The engine's
	// signed counterpart is the crossover [PB-DIM-VALUE-SEMANTICS] describes, so the
	// proof states the seed side as the sign and the figure is unique. The cost is
	// that Fusion's own unsigned dimension is not the one proved here.
	dim := sketch.NewHorizontalDistance(start, end, 10)
	s.AddConstraint(dim)
	s.SetConstraintName(dim, "anchor length")

	proofkit.Step(t, "the seeded length is the 10 mm the spec states")
	if got := start.DistanceTo(end); rel(got, 10) > 1e-9 {
		t.Errorf("anchor line seeded at %.6f mm, want 10", got)
	}
}

// ---------------------------------------------------------------------------
// §2 Gear Profiles — the lattice
// ---------------------------------------------------------------------------

// lattice builds the §2 figure in the coincident style [BEVEL-F-COINCIDENT-STYLE]
// requires: every line, lattice and reference alike, is created from raw
// coordinates and each endpoint that already exists is pinned with exactly one
// coincident. No §2 line is exempt and no line is drawn twice
// ([BEVEL-F-LINE-ONCE]).
type lattice struct {
	t     testing.TB
	s     *sketch.Sketch
	f     figure
	pts   map[string]*sketch.Point
	lines map[string]*sketch.Line
}

// seg draws one §2 construction line between two named points, seeding BOTH ends
// at the closed-form positions §2 states. A name already bound to a point is
// pinned with one coincident; a name met for the first time is bound to the fresh
// endpoint, which the constraints below then locate.
func (l *lattice) seg(name, from, to string) *sketch.Line {
	a := l.point(from)
	b := l.point(to)
	line := l.s.CreateLine(a, b)
	line.SetConstruction(true) // every line in this sketch is a construction line
	line.SetName(name)
	l.lines[name] = line
	return line
}

func (l *lattice) point(name string) *sketch.Point {
	seed, ok := l.f.pts[name]
	if !ok {
		l.t.Fatalf("§2 names no point %q", name)
	}
	p := l.s.CreatePoint(seed.X, seed.Y)
	p.SetName(name)
	if prev, ok := l.pts[name]; ok {
		l.s.AddConstraint(sketch.NewCoincident(p, prev))
		return p
	}
	l.pts[name] = p
	return p
}

func (l *lattice) add(name string, c sketch.Constraint) {
	l.s.AddConstraint(c)
	l.s.SetConstraintName(c, name)
}

// signedTurn is the signed angle, in degrees, from one seeded direction to another,
// counter-clockwise, which is what this engine's NewAngle measures.
func signedTurn(from, to pt) float64 {
	return math.Atan2(from.X*to.Y-from.Y*to.X, from.X*to.X+from.Y*to.Y) * 180 / math.Pi
}

// square pins one §2 line square to another with a SIGNED right angle, and checks
// against the seeds that it really is a right angle.
//
// Substitution, and what it costs: Fusion uses addPerpendicular here, which is
// undirected — it admits the drop on either side of the line it is square to, and
// the seed is the only thing that picks one ([BEVEL-F-MIRROR-FIGURE]). This engine's
// perpendicular is undirected in the same way, and the ambiguity probe reports the
// twin, so the proof states the same right angle as a signed one. The magnitude
// still comes from the unsigned length dimension beside it, exactly as in Fusion,
// so the pair has the same arity and the same shape; only the side is held by the
// constraint here and by the seed there.
func (l *lattice) square(name string, ref, line *sketch.Line, from, to pt) {
	a := signedTurn(from, to)
	if d := math.Abs(math.Abs(a) - 90); d > 1e-9 {
		l.t.Errorf("%s is seeded at %.6f° to its reference, not a right angle", name, a)
	}
	l.add(name, sketch.NewAngle(ref, line, a))
}

// along pins one §2 line's direction to another's with a SIGNED angle of 0 or 180
// degrees, for the two places §2 states a direction rather than a right angle: the
// driving shaft axis, which runs back from the Apex along the centre->Apex line,
// and a Tooth Spacing offset, which runs outward along the dedendum line away from
// the lower corner. Fusion's addParallel admits the antiparallel solution and its
// point-on-line admits either side, so both are seed-decided there.
func (l *lattice) along(name string, ref, line *sketch.Line, from, to pt, want float64) {
	a := signedTurn(from, to)
	if d := math.Abs(a - want); d > 1e-9 && math.Abs(math.Abs(a)-180) > 1e-9 {
		l.t.Errorf("%s is seeded at %.6f° to its reference, want %.0f°", name, a, want)
	}
	l.add(name, sketch.NewAngle(ref, line, a))
}

// dir is the seeded direction of a named §2 segment.
func (l *lattice) dir(from, to string) pt { return l.f.pts[to].sub(l.f.pts[from]) }

// offsetTo is the signed perpendicular offset the engine's two-row NewOffset
// takes, read off the closed-form seed positions. The magnitude is the value the
// spec states and is asserted against it at each call site; the SIGN is the seed
// side, which is the crossover [PB-DIM-VALUE-SEMANTICS] describes — Fusion's
// addOffsetDimension is unsigned and takes its side from the seed, so a signed
// target here is the same figure stated in the form this engine signs.
func (l *lattice) offsetTo(src [2]string, dst string) float64 {
	a, b := l.f.pts[src[0]], l.f.pts[src[1]]
	ab := b.sub(a)
	ap := l.f.pts[dst].sub(a)
	return (ab.X*ap.Y - ab.Y*ap.X) / ab.norm()
}

// stepGearProfiles builds the whole §2 lattice and proves it reaches DOF 0 with
// nothing redundant, nothing conflicting and no discrete ambiguity, then compares
// every named point's solved position against the closed form §2 seeded it at —
// the proof's copy of the [BEVEL-F-SEED-HELD] gate.
//
// ⚠️ THE PROOF CANNOT CATCH A WRONG SEED, and this is where that limit lives. Every
// point below is seeded at the closed form, which is the rule §2 states, so what
// this case proves is that the constraints solve FROM a correct seed and never that
// the generated module's seed is correct. The toe line M->N is the measured
// instance: two earlier seeding rules put the N seed at -0.27 mm from the shaft
// axis against a solved +5.17 mm, and Fusion refused the revolve with
// ASM_WIRE_X_AXIS at the revolve rather than at the seed. A seed defect therefore
// reaches Fusion untested, which is why the gate has to exist inside the generated
// module and not only here.
//
// ⚠️ A clean ambiguity probe is NOT evidence that no twin figure exists — the
// engine documents its probe as a LOWER bound on the number of solutions. What
// rules the twins out here is that every site the spec lists as seed-decided is
// pinned with a constraint this engine signs: NewAngle for the Shaft Angle, and
// NewOffset for the two base heights and the two toe lines. The two Tooth Spacing
// sites and the two front faces take signed axis components in place of Fusion's
// unsigned aligned length, which is the same arity and the same figure.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	if declaredRefusal(t, p) {
		return
	}
	f := newFigure(p)
	l := &lattice{t: t, s: s, f: f, pts: map[string]*sketch.Point{}, lines: map[string]*sketch.Line{}}

	proofkit.Step(t, "project the Anchor sketch's centre point and its anchor line")
	centre := s.CreateReferencePoint(0, 0, "anchorCentre")
	centre.SetName("c")
	anchorEnd := s.CreateReferencePoint(5, 0, "anchorEnd")
	anchorLine, err := s.CreateReferenceLine(centre, anchorEnd, "anchorLine")
	if err != nil {
		t.Fatalf("projected anchor line: %v", err)
	}
	anchorLine.SetName("Anchor Line (projected)")
	l.pts["c"] = centre

	proofkit.Step(t, "the centre->Apex construction line, undimensioned")
	centreToApex := l.seg("c->Apex", "c", "Apex")
	// The grow side. In Fusion this is addPerpendicular to the projected anchor line,
	// and which of the two perpendicular senses the apex takes is decided by the
	// TARGET-PLANE NORMAL as a one-bit direction, never by the sketch's local +Y
	// ([BEVEL-F-GROW-SIDE]). The proof fixes the anchor line, so it states that same
	// one bit as the sign of a right angle.
	l.square("apex drop square to the anchor line", anchorLine, centreToApex,
		pt{1, 0}, l.dir("c", "Apex"))

	proofkit.Step(t, "the two shaft axes; the pinion's is the +X-most of the two senses")
	drivingShaft := l.seg("Apex->B", "Apex", "B")
	l.along("driving shaft along the apex drop", centreToApex, drivingShaft,
		l.dir("c", "Apex"), l.dir("Apex", "B"), 180)
	pinionShaft := l.seg("Apex->A", "Apex", "A")
	// A SIGNED angle, measured counter-clockwise from the driving shaft's
	// start->end direction to the pinion's. In Fusion the magnitude comes from
	// addAngularDimension and the quadrant from the text point ([PB-ANGULAR-DIM]);
	// here the sign carries both, which is what pins this mirror site.
	l.add("Shaft Angle", sketch.NewAngle(drivingShaft, pinionShaft, f.sigma*180/math.Pi))

	// ⚠️ Both drops must aim at the SAME interior-wedge point. The A->Apex2 drop is
	// square to the pinion shaft axis and the B->Apex2 drop to the driving one, and
	// each is a mirror site of its own: flip one and the coincidence that closes them
	// makes the solver take the mirror figure, with A, C, D, G, H, K, M, N and A' all
	// at negative X. Nothing in the build refuses that figure — the end-of-§2 gate is
	// what catches it.
	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	aDrop := l.seg("A->Apex2", "A", "Apex2")
	l.square("A->Apex2 square to the pinion shaft", pinionShaft, aDrop,
		l.dir("Apex", "A"), l.dir("A", "Apex2"))
	l.add("PPD/2", sketch.NewDistance(l.pts["A"], l.pts["Apex2"], f.pinion.pitchRadius))
	bDrop := l.seg("B->Apex2", "B", "Apex2")
	l.square("B->Apex2 square to the driving shaft", drivingShaft, bDrop,
		l.dir("Apex", "B"), l.dir("B", "Apex2"))
	l.add("DPD/2", sketch.NewDistance(l.pts["B"], l.pts["Apex2"], f.driving.pitchRadius))

	// ⚠️ These two sites are where the "C collapses onto D" symptom lives. The right
	// angle fixes each dedendum line's direction and the length fixes its magnitude;
	// neither picks a side, so in Fusion each end has two solutions and only the seed
	// rules one out. Flip the pinion seed and C solves exactly onto D, which inverts
	// that gear: the toe ends up outside the heel and the conical end-cut finds no
	// cone face at the toe midpoint.
	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	pitchLine := l.seg("Apex->Apex2", "Apex", "Apex2")
	pinionDed := l.seg("Apex2->C", "Apex2", "C")
	l.square("pinion dedendum square to the Pitch Line", pitchLine, pinionDed,
		l.dir("Apex", "Apex2"), l.dir("Apex2", "C"))
	l.add("pinion dedendum length", sketch.NewDistance(l.pts["Apex2"], l.pts["C"], dedendumFactor*f.module))
	drivingDed := l.seg("Apex2->D", "Apex2", "D")
	l.square("driving dedendum square to the Pitch Line", pitchLine, drivingDed,
		l.dir("Apex", "Apex2"), l.dir("Apex2", "D"))
	l.add("driving dedendum length", sketch.NewDistance(l.pts["Apex2"], l.pts["D"], dedendumFactor*f.module))
	// The two dedendum directions are the ones whose dot with their own shaft axis is
	// sin(gamma), strictly positive for every configuration the range checks admit —
	// unlike the "toward the anchor line" test the B->Apex 2 drop warns about, which
	// reads about zero by construction.
	for _, side := range []struct {
		g    member
		name string
	}{{f.pinion, "C"}, {f.driving, "D"}} {
		d := l.dir("Apex2", side.name)
		axis := f.pts[axisEnd(side.g)].sub(f.pts["Apex"])
		if got := d.dot(axis) / (d.norm() * axis.norm()); got <= 0 {
			t.Errorf("%s dedendum direction has dot %.6f with its own shaft axis, want sin(gamma) > 0",
				side.g.label, got)
		}
	}

	proofkit.Step(t, "the two root axes")
	pinionRootAxis := l.seg("Apex->C", "Apex", "C")
	drivingRootAxis := l.seg("Apex->D", "Apex", "D")

	// The collinear chains. [PB-COLLINEAR-CHAIN] and [BEVEL-F-COLLINEAR-CHAIN] name
	// which line each collinear must name in Fusion — A->E names Apex->A, E->G names
	// A->E and never Apex->A. This engine counts a collinear as the same two
	// point-on-line rows, one of which the coincident at the shared endpoint already
	// asserts, so the proof carries the single row that is not implied. ⚠️ THAT
	// SUBSTITUTION MAKES BOTH READINGS IDENTICAL HERE: a proof written this way
	// cannot tell the correct collinear from the over-constraining one, and only a
	// Fusion session can. The cost is that the chain rule reaches Fusion untested.
	proofkit.Step(t, "the pinion extension chain A->E, C->E, E->G")
	aToE := l.seg("A->E", "A", "E")
	l.add("E on the pinion shaft axis", sketch.NewPointOnLine(l.pts["E"], pinionShaft))
	cToE := l.seg("C->E", "C", "E")
	l.add("C->E perpendicular to A->E", sketch.NewPerpendicular(cToE, aToE))
	l.seg("E->G", "E", "G")
	l.add("G on A->E", sketch.NewPointOnLine(l.pts["G"], aToE))

	proofkit.Step(t, "the driving extension chain B->F, D->F, F->I")
	bToF := l.seg("B->F", "B", "F")
	l.add("F on the driving shaft axis", sketch.NewPointOnLine(l.pts["F"], drivingShaft))
	dToF := l.seg("D->F", "D", "F")
	l.add("D->F perpendicular to B->F", sketch.NewPerpendicular(dToF, bToF))
	l.seg("F->I", "F", "I")
	l.add("I on B->F", sketch.NewPointOnLine(l.pts["I"], bToF))

	// ⚠️ The Fusion build constrains E->G perpendicular to H->G, and F->I
	// perpendicular to J->I, because addOffsetDimension REQUIRES its second entity to
	// be a line already parallel to the first and that perpendicular is what supplies
	// the parallelism. This engine's NewOffset is a different shape: it holds BOTH
	// endpoints of the target line at the same signed perpendicular distance from the
	// source, so it carries the parallelism itself. Adding the perpendicular here is
	// a third row for the same two freedoms — measured, the lattice comes back DOF 0
	// with two redundant constraints and the engine names the two base-height offsets
	// as the redundant pair. The proof therefore leaves both perpendiculars out
	// ([PB-NO-OVERCONSTRAIN]); the cost is that neither reaches this stage at all.
	proofkit.Step(t, "the two heel edges, and the base-height offsets that place them")
	cToH := l.seg("C->H", "C", "H")
	l.add("H on the pinion dedendum line", sketch.NewPointOnLine(l.pts["H"], pinionDed))
	gToH := l.seg("G->H", "G", "H")
	dToJ := l.seg("D->J", "D", "J")
	l.add("J on the driving dedendum line", sketch.NewPointOnLine(l.pts["J"], drivingDed))
	iToJ := l.seg("I->J", "I", "J")

	drivingOffset := l.offsetTo([2]string{"B", "Apex2"}, "I")
	if rel(math.Abs(drivingOffset), f.driving.baseHeight) > 1e-9 {
		t.Errorf("driving base-height offset %.6f is not the resolved height %.6f",
			math.Abs(drivingOffset), f.driving.baseHeight)
	}
	l.add("driving base height", sketch.NewOffset(bDrop, iToJ, drivingOffset))

	pinionOffset := l.offsetTo([2]string{"A", "Apex2"}, "G")
	if rel(math.Abs(pinionOffset), f.pinion.baseHeight) > 1e-9 {
		t.Errorf("pinion base-height offset %.6f is not the resolved height %.6f",
			math.Abs(pinionOffset), f.pinion.baseHeight)
	}
	l.add("pinion base height", sketch.NewOffset(aDrop, gToH, pinionOffset))

	proofkit.Step(t, "the pinion shaft-axis edge A'->G, drawn before the front face creates A'")
	l.seg("A'->G", "A'", "G")

	// The whole figure still carries one freedom at this point — how far the Apex
	// sits from the projected centre along the perpendicular — and this is what
	// closes it. Fusion applies addCoincident(I, projected centre), which is two rows;
	// I is already on the centre->Apex line by construction, so one of those rows is
	// dependent and this engine reports it as redundant. The proof carries the row
	// that is not implied, which is the same substitution [PB-COLLINEAR-CHAIN] names
	// for a collinear whose second row is already asserted. The cost is that the
	// coincident's own arity reaches Fusion untested.
	proofkit.Step(t, "constrain point I with the centre point")
	l.add("I closes on the projected centre", sketch.NewPointOnLine(l.pts["I"], anchorLine))

	proofkit.Step(t, "the tooth centres K and L, each pinned by two point-on-line rows")
	l.seg("G->K", "G", "K")
	l.add("K on the pinion shaft axis", sketch.NewPointOnLine(l.pts["K"], pinionShaft))
	l.add("K on the pinion dedendum line", sketch.NewPointOnLine(l.pts["K"], pinionDed))
	l.seg("I->L", "I", "L")
	l.add("L on the driving shaft axis", sketch.NewPointOnLine(l.pts["L"], drivingShaft))
	l.add("L on the driving dedendum line", sketch.NewPointOnLine(l.pts["L"], drivingDed))

	// |Apex2->K| is the EXACT back-cone radius r / cos(gamma) that §3 step 1 defines,
	// never a radius rebuilt from a rounded tooth count: the dedendum line is
	// perpendicular to the Pitch Line, which meets the shaft axis at gamma, so
	// walking r/cos(gamma) from Apex 2 lands on the axis at K.
	for _, side := range []struct {
		g    member
		name string
	}{{f.pinion, "K"}, {f.driving, "L"}} {
		got := f.pts["Apex2"].sub(f.pts[side.name]).norm()
		if rel(got, side.g.virtualPitchRadius) > 1e-12 {
			t.Errorf("|Apex2->%s| = %.6f is not the back-cone radius %.6f",
				side.name, got, side.g.virtualPitchRadius)
		}
	}

	if f.toothSpacing > 0 {
		proofkit.Step(t, "the Tooth Spacing offsets K->K' and L->L'")
		l.seg("C->K", "C", "K")
		l.seg("D->L", "D", "L")
		l.spacing("K->K'", "K", "K'", "Apex2->C", "C", f.pinion)
		l.spacing("L->L'", "L", "L'", "Apex2->D", "D", f.driving)
		l.seg("C->K'", "C", "K'")
		l.seg("D->L'", "D", "L'")
	} else {
		// At Tooth Spacing 0 K' is K and L' is L, so nothing is built here and the
		// existing reference line is reused — a zero-length dimensioned line would be
		// degenerate, and one segment gets one line ([BEVEL-F-LINE-ONCE]).
		proofkit.Step(t, "Tooth Spacing 0: reuse C->K and D->L as the tooth-centre lines")
		l.seg("C->K", "C", "K")
		l.seg("D->L", "D", "L")
	}

	proofkit.Step(t, "the pinion toe line M->N and the front face N->A'")
	l.toe(f.pinion, "M", "N", "A'", pinionRootAxis, pinionShaft, cToH, [2]string{"C", "H"})
	l.seg("M->C", "M", "C")

	proofkit.Step(t, "the driving toe line O->P and the front face P->B'")
	l.toe(f.driving, "O", "P", "B'", drivingRootAxis, drivingShaft, dToJ, [2]string{"D", "J"})
	l.seg("O->D", "O", "D")
	l.seg("B'->I", "B'", "I")

	// The proof's copy of the [BEVEL-F-SEED-HELD] gate: after the solve, every named
	// point is compared against its closed-form seed, in §2's own creation order, at
	// the tolerance the anchor states. The list is 22 points when Tooth Spacing is
	// above zero and 20 at the default, because K' and L' are not built at 0 and
	// comparing a point that was never created is the one way this gate can raise on
	// a correct figure.
	proofkit.Step(t, "gate the solved figure against its own seeds, in §2's creation order")
	solve(t, s)
	const seedTolerance = 0.001 // mm
	for _, name := range f.seedOrder() {
		got := l.pts[name]
		if got == nil {
			t.Fatalf("§2 point %s was never built", name)
			return
		}
		want := f.pts[name]
		if d := math.Hypot(got.X()-want.X, got.Y()-want.Y); d > seedTolerance {
			t.Fatalf("§2 point %s solved to (%.6f, %.6f), seeded at (%.6f, %.6f), %.6f mm away — "+
				"the first point that moved names the earliest site that flipped",
				name, got.X(), got.Y(), want.X, want.Y, d)
			return
		}
	}

	proofkit.Step(t, "the figure lies on the intended side of the anchor line")
	if l.pts["Apex"].Y() <= 0 {
		t.Errorf("the Apex solved below the anchor line at y=%.6f; the grow side is chosen by "+
			"the target-plane normal, never by the sketch's local +Y", l.pts["Apex"].Y())
	}
	for _, name := range []string{"A", "C", "G", "H", "K", "M", "N", "A'"} {
		if l.pts[name].X() < 0 {
			t.Errorf("§2 point %s solved at x=%.6f: the figure is mirrored about the driving "+
				"shaft axis", name, l.pts[name].X())
		}
	}
}

// spacing pins a tooth centre one Tooth Spacing beyond K/L along the dedendum
// line, away from the lower corner.
//
// Substitution, and what it costs: Fusion pins K' with a point-on-line coincident
// to the dedendum line plus an aligned LENGTH dimension, and a length is unsigned —
// the two candidates sit 2 x Tooth Spacing apart and only the seed rules the wrong
// one out. A flipped K' tightens the mesh by the clearance the input asked to add
// and builds a gear that looks right. This engine's NewDistance is unsigned in the
// same way, so the proof takes the two SIGNED axis components of the same offset
// instead: the same two rows, the same figure, and a side this engine holds rather
// than one the seed alone holds. The cost is that Fusion's own pair of constraints
// is not the pair proved here.
func (l *lattice) spacing(name, from, to, dedLine, corner string, g member) {
	l.seg(name, from, to)
	delta := l.f.pts[to].sub(l.f.pts[from])
	if rel(delta.norm(), l.f.toothSpacing) > 1e-9 {
		l.t.Errorf("%s spans %.6f mm, want one Tooth Spacing of %.6f",
			name, delta.norm(), l.f.toothSpacing)
	}
	if delta.dot(g.dedendum) <= 0 {
		l.t.Errorf("%s runs toward the lower corner; the offset is away from it", name)
	}
	l.along(name+" runs outward along the dedendum line", l.lines[dedLine], l.lines[name],
		l.dir("Apex2", corner), delta, 0)
	l.add(name+" length", sketch.NewDistance(l.pts[from], l.pts[to], l.f.toothSpacing))
}

// toe builds one gear's toe line and front face: M on the root axis, the toe line
// held one root length off the heel edge, and the front face standing square to the
// shaft at this gear's Toe Radius.
//
// Six freedoms, six constraints — the arity Fusion reaches with
// addCoincident(M, root axis), addParallel, addOffsetDimension,
// addCoincident(A', shaft axis), addPerpendicular and an aligned length. Two
// substitutions, each the same arity:
//
//   - NewOffset carries the parallelism itself, so the proof adds NO parallel
//     constraint. In Fusion addParallel IS required, because the toe line is drawn
//     at an arbitrary angle and addOffsetDimension needs its second entity already
//     parallel ([PB-OFFSET-DIM]).
//   - The front face takes the two signed axis components of N - A' in place of a
//     perpendicular plus an unsigned aligned length. The length is what makes this
//     a mirror site in Fusion: the toe line meets the Toe Radius on BOTH sides of
//     the shaft axis, and a seed below the axis converges happily onto the mirror,
//     after which the revolved hexagon crosses its own axis of revolution and Fusion
//     aborts with ASM_WIRE_X_AXIS at the revolve rather than at the seed.
//
// ⚠️ N is never pinned to the shaft axis itself. A' sits on the axis and is a FOOT,
// not a corner; N is held off it by a strictly positive Toe Radius. Pinning N there
// puts it on the axis of revolution and the later conical split fails with
// ASM_API_FAILED for asymmetric tooth counts.
func (l *lattice) toe(g member, mName, nName, primeName string,
	rootAxis, shaftAxis, heelEdge *sketch.Line, heelEnds [2]string) {
	toeLine := l.seg(mName+"->"+nName, mName, nName)
	l.add(mName+" on the root axis", sketch.NewPointOnLine(l.pts[mName], rootAxis))

	offset := l.offsetTo(heelEnds, mName)
	want := l.f.rootLength * l.f.pitchCone / l.f.apexDed
	if rel(math.Abs(offset), want) > 1e-9 {
		l.t.Errorf("%s toe offset %.6f is not the Root Length re-measured perpendicular to the "+
			"pitch line, %.6f", g.label, math.Abs(offset), want)
	}
	if l.f.toeExtension == 0 && rel(want, l.f.faceWidth) > 1e-9 {
		l.t.Errorf("%s: at Toe Extension 0 the offset must be the resolved Face Width %.6f, got %.6f",
			g.label, l.f.faceWidth, want)
	}
	l.add(g.label+" root length", sketch.NewOffset(heelEdge, toeLine, offset))

	front := l.seg(nName+"->"+primeName, nName, primeName)
	l.add(primeName+" on the shaft axis", sketch.NewPointOnLine(l.pts[primeName], shaftAxis))
	l.square(g.label+" front face square to the shaft", shaftAxis, front,
		l.f.pts[axisEnd(g)].sub(l.f.pts["Apex"]), l.dir(nName, primeName))
	delta := l.f.pts[nName].sub(l.f.pts[primeName])
	if rel(delta.norm(), g.toeRadius) > 1e-9 {
		l.t.Errorf("%s front face spans %.6f mm, want the resolved Toe Radius %.6f",
			g.label, delta.norm(), g.toeRadius)
	}
	l.add(g.label+" toe radius", sketch.NewDistance(l.pts[nName], l.pts[primeName], g.toeRadius))
}

// axisEnd names the §2 point this gear's shaft axis ends at.
func axisEnd(g member) string {
	if g.label == "Pinion" {
		return "A"
	}
	return "B"
}

// solve runs the solver once so a step can read solved positions before proofkit's
// own gate runs. proofkit solves again and gates that solve; solving twice is
// idempotent on a converged system.
func solve(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve: %v", err)
	}
}

// ---------------------------------------------------------------------------
// §3 the virtual spur tooth
// ---------------------------------------------------------------------------

// toothCases carries the two pairs the virtual tooth count needs, on top of the
// default and the ratio pairs. 16 driving / 12 pinion gives the pinion an exact
// virtual count of 15 and the driving gear 26.667, so one member is a case where a
// rounded count agrees and the other is one where it does not — whatever error the
// rounding introduces, it is not the same error on both members. 4/4 carries the
// lowest virtual count the table reaches, 5.657, and the largest root-arc corner
// float of any pair the spec admits.
var toothCases = []proofkit.Case{
	{Name: "default_31_31_90", Params: caseParams(nil)},
	{Name: "teeth_16_12", Params: caseParams(map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12})},
	{Name: "teeth_4_4", Params: caseParams(map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4})},
	{Name: "ratio_driving_31_pinion_17", Params: caseParams(map[string]float64{"pinionTeeth": 17})},
	{Name: "shaft_angle_60", Params: caseParams(map[string]float64{"shaftAngleDeg": 60})},
	{Name: "tooth_spacing_positive", Params: caseParams(map[string]float64{
		"drivingTeeth": 43, "pinionTeeth": 31, "shaftAngleDeg": 75, "toothSpacing": 0.4})},
}

// stepToothProfile draws both members' virtual spur teeth on their back-cone
// planes, with the exact (never rounded) virtual tooth count and the root sink
// applied, and proves the four circle radii, the embedded flag and the tooth loop's
// curve counts.
//
// Substitution, and what it costs: the two involute flanks are laid down as fitted
// splines through fixed sample points rather than through the spur family's own
// constraint scheme. That scheme is proved in the spur family's own proof, and
// reproducing it here would prove it twice; what this case owns is the radii bevel
// hands the borrowed drawer and the loop those radii produce. The cost is that
// nothing here shows the flank scheme reaches DOF 0.
func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	for _, g := range []member{f.pinion, f.driving} {
		proofkit.Step(t, "%s: the four circles the proxy is asked for", g.label)
		drawVirtualTooth(t, s, f, g)
	}
}

func drawVirtualTooth(t testing.TB, s *sketch.Sketch, f figure, g member) { //nolint:gocyclo
	m := f.module
	vpr := g.virtualPitchRadius

	// The four radii, from the back-cone radius and the raw-mm Module. The root
	// circle is drawn one root sink INSIDE the dedendum corner so the Combine-Join
	// meets the gear body across the root rather than along the tooth's centreline.
	pitchR := vpr
	baseR := vpr * math.Cos(bevelPressureAngle)
	tipR := vpr + addendumFactor*m
	rootR := vpr - dedendumFactor*m - f.rootSink

	if rel(2*pitchR/m, g.virtualTeeth) > 1e-12 {
		t.Errorf("%s: the drawn pitch circle does not reach the back cone", g.label)
	}
	if got := f.rootSink; rel(got, 0.05*2.25*m) > 1e-12 {
		t.Errorf("%s: root sink %.6f is not 0.05 * 2.25 * Module", g.label, got)
	}

	// Each gear's tooth sits on its own plane; the proof lays the two side by side in
	// one sketch, which changes no radius and no angle.
	cx := 0.0
	if g.label == "Driving" {
		cx = 4 * tipR
	}
	centre := s.CreateReferencePoint(cx, 0, g.label+" tooth centre")
	centre.SetName(g.label + " tooth centre")

	for _, c := range []struct {
		name string
		r    float64
	}{{"pitch", pitchR}, {"base", baseR}, {"tip", tipR}, {"root", rootR}} {
		circle := s.CreateCircle(s.CreatePoint(cx, 0), c.r)
		circle.SetConstruction(true)
		circle.SetName(g.label + " " + c.name + " circle")
		s.AddConstraint(sketch.NewCoincident(circle.Center, centre))
		dim := sketch.NewDiameter(circle, 2*c.r)
		s.AddConstraint(dim)
		s.SetConstraintName(dim, g.label+" "+c.name+" diameter")
	}

	// The real count reaches the drawer only as the angular half-thickness
	// pi / (2 * z_v), which at z_v = 2*r_v/Module gives the standard tooth thickness
	// pi*Module/2 at the pitch circle — the same thickness the spur gear of this
	// module carries. An integer count at the same radius gives pi*r_v/round(z_v),
	// which misses nominal by a different amount on each member of an unequal pair.
	thickness := 2 * pitchR * math.Pi / (2 * g.virtualTeeth)
	if rel(thickness, math.Pi*m/2) > 1e-12 {
		t.Errorf("%s: pitch-circle tooth thickness %.6f is not the standard pi*Module/2 %.6f",
			g.label, thickness, math.Pi*m/2)
	}

	left, right, embedded := virtualFlanks(m, vpr, f.rootSink, g.virtualTeeth)
	if len(left) < 2 || len(right) < 2 {
		t.Fatalf("%s: the involute flanks produced %d/%d samples", g.label, len(left), len(right))
		return
	}

	place := func(q pt) *sketch.Point {
		p := s.CreatePoint(cx+q.X, q.Y)
		s.Fix(p)
		return p
	}
	leftPts := make([]*sketch.Point, 0, len(left))
	rightPts := make([]*sketch.Point, 0, len(right))
	for i := range left {
		leftPts = append(leftPts, place(left[i]))
		rightPts = append(rightPts, place(right[i]))
	}

	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("%s: left flank: %v", g.label, err)
		return
	}
	leftFlank.SetName(g.label + " left flank")
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("%s: right flank: %v", g.label, err)
		return
	}
	rightFlank.SetName(g.label + " right flank")

	// The tip arc runs between the two flanks' outer samples; the root arc runs
	// between their inner ends, either directly (embedded) or through the two
	// connecting lines the non-embedded tooth carries.
	//
	// Substitution, and what it costs: an arc's centre is pinned here by ONE signed
	// horizontal component beside the arc's own equidistance row, because the engine's
	// unsigned alternatives leave the centre's mirror twin standing and the ambiguity
	// probe names it. The radius is asserted rather than dimensioned, for the same
	// reason the trace arc's is. The cost is that the drawer's own pair of constraints
	// is not the pair proved here.
	arcAt := func(name string, start, end *sketch.Point, radius float64) *sketch.Arc {
		anchor := s.CreateReferencePoint(cx, 0, g.label+" "+name+" centre")
		arc := s.CreateArc(s.CreatePoint(cx, 0), start, end)
		arc.SetName(g.label + " " + name)
		pin := sketch.NewHorizontalDistance(anchor, arc.Center, 0)
		s.AddConstraint(pin)
		s.SetConstraintName(pin, g.label+" "+name+" centre")
		return arc
	}
	tipArc := arcAt("tip arc", rightPts[len(rightPts)-1], leftPts[len(leftPts)-1], tipR)

	rootStart, rootEnd := leftPts[0], rightPts[0]
	lines := 0
	if !embedded {
		// The flank starts outside the root circle, so the drawer adds one radial stub
		// per flank down to the root arc.
		toRoot := func(from *sketch.Point) *sketch.Point {
			dx, dy := from.X()-cx, from.Y()
			k := rootR / math.Hypot(dx, dy)
			p := s.CreatePoint(cx+dx*k, dy*k)
			s.Fix(p)
			line := s.CreateLine(from, p)
			line.SetName(g.label + " flank-to-root line")
			lines++
			return p
		}
		rootStart = toRoot(leftPts[0])
		rootEnd = toRoot(rightPts[0])
	}
	rootArc := arcAt("root arc", rootStart, rootEnd, rootR)

	proofkit.Step(t, "%s: virtual count %.4f, embedded=%v, %d connecting line(s)",
		g.label, g.virtualTeeth, embedded, lines)

	wantLines := 2
	if embedded {
		wantLines = 0
	}
	if lines != wantLines {
		t.Errorf("%s: the tooth loop carries %d connecting line(s), want %d — the count is "+
			"DETERMINED by the embedded flag and is never accepted as 0 or 2",
			g.label, lines, wantLines)
	}
	// The default pair is the configuration where the sink drops the root circle below
	// the base circle, so its tooth is drawn NON-embedded and the two stubs appear.
	if embedded != (baseR < rootR) {
		t.Errorf("%s: the embedded flag disagrees with the sunk root radius", g.label)
	}
	if got := tipArc.R(); rel(got, tipR) > 1e-9 {
		t.Errorf("%s: tip arc solved at %.6f, want virtualPitchRadius + Module = %.6f",
			g.label, got, tipR)
	}
	if got := rootArc.R(); rel(got, rootR) > 1e-9 {
		t.Errorf("%s: root arc solved at %.6f, want the sunk root %.6f", g.label, got, rootR)
	}
}

// ---------------------------------------------------------------------------
// The per-gear Profile sketch
// ---------------------------------------------------------------------------

// stepProfileSketch recreates one gear's six §2 vertices as fresh points, draws the
// closed hexagon sharing them, and fixes the lines' endpoints AFTER the lines exist
// — the [PB-PROJECT-NOT-FIXED] recreate-share-fix recipe. Fixing a bare point
// before it is consumed as a line endpoint does not leave the sketch fully
// constrained, so the order is load-bearing.
//
// The hexagon's FIRST edge is the gear's shaft axis, and every body operation below
// — the revolve, the pattern, the bore plane and the meshing rotation — takes it
// from here rather than from the §2 Apex->A / Apex->B construction line, which
// lives in a different sketch.
func stepProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	if declaredRefusal(t, p) {
		return
	}
	f := newFigure(p)
	for _, g := range []member{f.pinion, f.driving} {
		proofkit.Step(t, "%s Profile: six recreated vertices, one closed loop", g.label)
		hex := f.hexagon(g)
		lay := 0.0
		if g.label == "Driving" {
			lay = 3 * f.apexDed // the two hexagons sit side by side in one proof sketch
		}
		pts := make([]*sketch.Point, 0, len(hex))
		for i, v := range hex {
			q := s.CreatePoint(v.X, v.Y+lay)
			q.SetName(g.label + " hexagon vertex " + string(rune('0'+i)))
			pts = append(pts, q)
		}
		lines := make([]*sketch.Line, 0, len(pts))
		for i := range pts {
			line := s.CreateLine(pts[i], pts[(i+1)%len(pts)])
			line.SetName(g.label + " hexagon edge " + string(rune('0'+i)))
			lines = append(lines, line)
		}
		for _, line := range lines {
			s.Fix(line.Start)
			s.Fix(line.End)
		}

		proofkit.Step(t, "%s: the first edge is the shaft axis", g.label)
		if math.Abs(hex[0].Y) > 1e-9 || math.Abs(hex[1].Y) > 1e-9 {
			t.Errorf("%s: the first edge runs from (%.6f, %.6f) to (%.6f, %.6f); both ends must "+
				"sit on the shaft axis", g.label, hex[0].X, hex[0].Y, hex[1].X, hex[1].Y)
		}
		if hex[1].X <= hex[0].X {
			t.Errorf("%s: the shaft edge runs toward the apex; A'/B' is the toe end", g.label)
		}

		proofkit.Step(t, "%s: the profile stays on one side of the axis of revolution", g.label)
		for i, v := range hex {
			if v.Y < -1e-12 {
				t.Errorf("%s: hexagon vertex %d sits at radius %.6f — a profile that crosses its "+
					"own axis of revolution fails the revolve with ASM_WIRE_X_AXIS",
					g.label, i, v.Y)
			}
		}
		if got, want := polygonArea(hex), 0.0; got <= want {
			t.Errorf("%s: the hexagon encloses no area", g.label)
		}
	}
}

// ---------------------------------------------------------------------------
// The Bore sketch
// ---------------------------------------------------------------------------

// stepBoreSketch draws the bore circle on the plane normal to the shaft at its
// start. The plane is rooted on the axis, so the circle is centred on the sketch
// origin; its centre is FIXED rather than made coincident with the origin point
// ([PB-CIRCLE-CENTER] — a circle's centre is free even when created at (0,0,0), and
// a coincident to the origin has thrown VCS_SKETCH_SOLVING_FAILED on exactly this
// kind of setByDistanceOnPath plane).
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if !f.boreEnabled {
		proofkit.Step(t, "Enable Bore is unchecked: no bore is cut on either gear")
		s.CreateReferencePoint(0, 0, "boreDisabled").SetName("bore skipped")
		return
	}
	for _, g := range []member{f.pinion, f.driving} {
		cx := 0.0
		if g.label == "Driving" {
			cx = 4 * g.pitchRadius
		}
		centre := s.CreatePoint(cx, 0)
		centre.SetName(g.label + " bore centre")
		s.Fix(centre)
		circle := s.CreateCircle(centre, g.boreDiameter/2)
		circle.SetName(g.label + " Bore")
		dim := sketch.NewDiameter(circle, g.boreDiameter)
		s.AddConstraint(dim)
		s.SetConstraintName(dim, g.label+" bore diameter")

		proofkit.Step(t, "%s: bore diameter %.4f against its maximum %.4f",
			g.label, g.boreDiameter, g.maxBore)
		if g.boreDiameter > g.maxBore+1e-12 {
			t.Errorf("%s: the bore diameter reaching the sketch is not bounded", g.label)
		}
		if g.boreDiameter <= 0 {
			t.Errorf("%s: resolved bore diameter %.6f", g.label, g.boreDiameter)
		}
	}
}
