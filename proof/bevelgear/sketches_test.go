// Sketch steps. Each function below is one Fusion sketch, rebuilt in the
// sketch engine with the constraint scheme the step list prescribes, and gated
// by proofkit on the engine's own verdict: DOF 0, nothing conflicting or
// redundant, valid profiles, not near-singular, no discrete ambiguity.
//
// Three places the engine's constraint arity differs from Fusion's, so the
// proof carries one row fewer than the module must:
//
//  1. §1 applies BOTH addCoincident(centre, anchorLine) and
//     addMidPoint(centre, anchorLine). The engine's NewMidpoint already holds
//     the point at (a+b)/2, which subsumes point-on-line, so adding
//     NewPointOnLine as well is a third row for two freedoms and the engine
//     reports it redundant. The proof applies the midpoint alone.
//  2. §2's G->H and I->J each take addPerpendicular against E->G and F->I in
//     Fusion, because addOffsetDimension there requires the second line to be
//     parallel to the first already. The engine's NewOffset emits two rows
//     holding BOTH endpoints of the target at the same signed perpendicular
//     distance, so it carries the parallelism itself; adding the perpendicular
//     makes the lattice overconstrained at DOF 0 with the two base-height
//     offsets named as the redundant pair. The proof leaves both out.
//  3. For the same reason the proof leaves out addParallel(M->N, C->H) and
//     addParallel(O->P, D->J): the offset that follows each of them already
//     carries the direction.
//
// Two places the engine is STRONGER than Fusion, which the spec asks for
// ("Pin every one of the 15 sites with a constraint the sketch engine SIGNS"):
// the shaft-angle dimension is a signed NewAngle rather than Fusion's unsigned
// addAngularDimension plus a text point, and the two base-height offsets and
// the two toe offsets are signed NewOffset rather than Fusion's unsigned
// addOffsetDimension. The Tooth Spacing sites K->K' and L->L' have no signed
// constraint of the right shape, so they are pinned with a signed axis
// distance and the sign is asserted directly; see stepGearProfiles.
package bevelgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
)

// seedTolerance is the [BEVEL-F-SEED-HELD] gate's own tolerance, 0.001 mm. The
// proof holds its solved lattice to the same figure the module's gate holds
// Fusion's to.
const seedTolerance = 0.001

// proxyPressureAngle is not a bevel dialog input: the virtual spur proxy's own
// default is 20 degrees and bevel takes it.
const proxyPressureAngle = 20 * math.Pi / 180

// proxyInvoluteSteps is the proxy's own default sample count.
const proxyInvoluteSteps = 15

// solved reads a point's position after the build has solved the sketch
// itself. proofkit solves and gates again afterwards; this second solve is
// what lets a step assert against the figure the constraints actually reached
// rather than against the coordinates it seeded.
func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
}

func at(p *sketch.Point) pt { return pt{p.X(), p.Y()} }

// requireNear compares a solved point against the closed form the step seeded
// it at. The message names both positions, exactly as the module's own
// [BEVEL-F-SEED-HELD] gate does.
func requireNear(t testing.TB, name string, got, want pt, tol float64) {
	t.Helper()
	if d := got.distance(want); d > tol {
		t.Errorf("%s solved at (%.6f, %.6f) but was seeded at (%.6f, %.6f), %.6f mm away (tolerance %.4f mm)",
			name, got.X, got.Y, want.X, want.Y, d, tol)
	}
}

func requireClose(t testing.TB, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s is %.8f, want %.8f (tolerance %.2e)", what, got, want, tol)
	}
}

// ---------------------------------------------------------------------------
// S05 Anchor sketch
// ---------------------------------------------------------------------------

// stepAnchorSketch rebuilds §1: the projected centre, the 10 mm reference line
// through it, and the three constraints that leave it with no free degree of
// freedom. The line's absolute direction is arbitrary — §2 derives every
// direction relative to it — but a free rotation would be a defect, which is
// what addHorizontal (NewHorizontal here) rules out.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user-selected centre point")
	centre := s.CreateReferencePoint(0, 0, "user centre point projected into the Anchor sketch")
	centre.SetName("projected centre")

	proofkit.Step(t, "anchor line seeded at +/- 0.5 cm from the projected centre")
	a := s.CreatePoint(-5, 0)
	a.SetName("anchor line start")
	b := s.CreatePoint(5, 0)
	b.SetName("anchor line end")
	line := s.CreateLine(a, b)
	line.SetName("Anchor Line")
	line.SetConstruction(true)

	proofkit.Step(t, "midpoint, aligned length and the sketch-local direction lock")
	// Fusion applies addCoincident(centre, line) as well; see the file comment
	// for why the engine's NewMidpoint makes that row redundant here.
	//
	// Fusion writes the length as an aligned addDistanceDimension and the
	// direction as addHorizontal. Those two rows leave the line's two ends
	// interchangeable — a magnitude and an undirected direction admit a and b
	// swapped — and Fusion takes the seeded one. proofkit waives nothing, so
	// the proof SUBSTITUTES the signed pair that pins the same figure: a
	// horizontal distance of +10 mm from start to end, and a vertical
	// distance of 0, which is the same length and the same sketch-local
	// direction with the flip ruled out rather than seeded.
	s.AddConstraint(
		sketch.NewMidpoint(centre, line),
		sketch.NewHorizontalDistance(a, b, 10),
		sketch.NewVerticalDistance(a, b, 0),
	)

	solveHere(t, s)
	requireClose(t, "anchor line length", at(a).distance(at(b)), 10, 1e-9)
	requireClose(t, "anchor line half length to the projected centre", at(a).distance(at(centre)), 5, 1e-9)
	requireClose(t, "anchor line rise", at(b).Y-at(a).Y, 0, 1e-9)
}

// ---------------------------------------------------------------------------
// S07 Gear Profiles sketch — the §2 lattice
// ---------------------------------------------------------------------------

// lattice is the handle set stepGearProfiles builds, so the assertions below
// can name a point the way §2 names it.
type lattice struct {
	apex, a, b, apex2, c, d, e, f, g, h, i, j, k, kp, m, n, ap, l, lp, o, pp, bp *sketch.Point
	centre                                                                       *sketch.Point
	dedP, dedG, cToH, dToJ, mToN, oToP                                           *sketch.Line
}

// connect builds one §2 line in the COINCIDENT style
// ([BEVEL-F-COINCIDENT-STYLE]): both endpoints are fresh points at raw
// coordinates, and each end that meets an existing point takes exactly one
// coincident. Nothing is ever shared into a creation call, because sharing and
// coinciding together is the redundancy that kills the Fusion solve outright,
// and sharing alone leaves the sketch under-constrained.
func connect(s *sketch.Sketch, name string, from, to pt, pin ...*sketch.Point) (*sketch.Line, *sketch.Point, *sketch.Point) {
	start := s.CreatePoint(from.X, from.Y)
	end := s.CreatePoint(to.X, to.Y)
	line := s.CreateLine(start, end)
	line.SetName(name)
	line.SetConstruction(true)
	start.SetName(name + " start")
	end.SetName(name + " end")
	if len(pin) > 0 && pin[0] != nil {
		s.AddConstraint(sketch.NewCoincident(start, pin[0]))
	}
	if len(pin) > 1 && pin[1] != nil {
		s.AddConstraint(sketch.NewCoincident(end, pin[1]))
	}
	return line, start, end
}

// leftOffset is the signed perpendicular distance the engine's NewOffset
// measures: positive on the LEFT of src's start->end direction. Fusion's
// addOffsetDimension is unsigned and takes its side from the seed, so the sign
// computed here is what the proof carries in place of that seed.
func leftOffset(srcA, srcB, target pt) float64 {
	u := srcB.sub(srcA).unit()
	d := target.sub(srcA)
	return u.X*d.Y - u.Y*d.X
}

func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	if declaredRefusal(p) {
		proofkit.Unmodelled(t, "declared refusal: the spec admits this configuration and this §2 "+
			"lattice cannot reach it — the net's conditioning reads below the sketch engine's trust "+
			"floor. That is a property of this net, not of the geometry, and the spec's rule is to "+
			"record it here rather than narrow the advertised range on one net's evidence")
		return
	}
	g := newGeometry(t, p)
	var lat lattice

	proofkit.Step(t, "project the Anchor sketch's centre point and its line")
	lat.centre = s.CreateReferencePoint(0, 0, "Anchor sketch centre point projected into Gear Profiles")
	lat.centre.SetName("projected centre")
	anchorA := s.CreateReferencePoint(-5, 0, "Anchor Line projected into Gear Profiles")
	anchorB := s.CreateReferencePoint(5, 0, "Anchor Line projected into Gear Profiles")
	anchorLine, err := s.CreateReferenceLine(anchorA, anchorB, "Anchor Line projected into Gear Profiles")
	if err != nil {
		t.Fatalf("project the anchor line: %v", err)
	}
	anchorLine.SetName("projected Anchor Line")

	proofkit.Step(t, "centre -> Apex, perpendicular to the anchor line, undimensioned")
	centerToApex, ctaStart, apexPt := connect(s, "centre->Apex", g.C0, g.Apex, lat.centre, nil)
	_ = ctaStart
	apexPt.SetName("Apex")
	lat.apex = apexPt
	// Every perpendicular and parallel below is written as a SIGNED angle.
	// Fusion's addPerpendicular and addParallel are undirected — antiparallel
	// satisfies a parallel, and a perpendicular admits both sides — so in the
	// module each of these sites is held by its seed alone
	// ([BEVEL-F-MIRROR-FIGURE]). The engine signs them, and the spec's
	// "Proving the §2 figure" section asks for exactly that: measured, leaving
	// the driving dedendum undirected makes the probe report the second
	// configuration with D solved onto C, which is the collapse the spec names.
	s.AddConstraint(sketch.NewAngle(anchorLine, centerToApex, signedAngleDeg(pt{1, 0}, g.Perp)))

	proofkit.Step(t, "Driving Gear Shaft Axis, parallel to centre->Apex")
	drivingShaft, _, bPt := connect(s, "Driving Gear Shaft Axis", g.Apex, g.B, lat.apex, nil)
	bPt.SetName("B")
	lat.b = bPt
	s.AddConstraint(sketch.NewAngle(centerToApex, drivingShaft, signedAngleDeg(g.Perp, g.DrivingDir)))

	proofkit.Step(t, "Pinion Gear Shaft Axis at the Shaft Angle")
	pinionShaft, _, aPt := connect(s, "Pinion Gear Shaft Axis", g.Apex, g.A, lat.apex, nil)
	aPt.SetName("A")
	lat.a = aPt
	// Signed: the counter-clockwise angle from Apex->B to Apex->A. Fusion's
	// addAngularDimension is an unsigned magnitude plus a text point that only
	// picks the quadrant, so the side there is held by the seed alone.
	s.AddConstraint(sketch.NewAngle(drivingShaft, pinionShaft, signedAngleDeg(g.DrivingDir, g.PinionDir)))

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	aDrop, aDropStart, apex2Pt := connect(s, "A->Apex2", g.A, g.Apex2, lat.a, nil)
	apex2Pt.SetName("Apex 2")
	lat.apex2 = apex2Pt
	s.AddConstraint(
		sketch.NewAngle(pinionShaft, aDrop, signedAngleDeg(g.PinionDir, g.Apex2.sub(g.A))),
		sketch.NewDistance(aDropStart, apex2Pt, g.PPD/2),
	)
	bDrop, bDropStart, bDropEnd := connect(s, "B->Apex2", g.B, g.Apex2, lat.b, nil)
	s.AddConstraint(
		sketch.NewAngle(drivingShaft, bDrop, signedAngleDeg(g.DrivingDir, g.Apex2.sub(g.B))),
		sketch.NewDistance(bDropStart, bDropEnd, g.DPD/2),
		sketch.NewCoincident(bDropEnd, apex2Pt),
	)

	proofkit.Step(t, "Pitch Line and the two dedendum lines")
	pitchLine, _, _ := connect(s, "Pitch Line", g.Apex, g.Apex2, lat.apex, lat.apex2)
	pinionDed, pinionDedStart, cPt := connect(s, "Pinion Gear Dedendum", g.Apex2, g.C, lat.apex2, nil)
	cPt.SetName("C")
	lat.c = cPt
	lat.dedP = pinionDed
	s.AddConstraint(
		sketch.NewAngle(pitchLine, pinionDed, signedAngleDeg(g.PitchDir, g.DedP)),
		sketch.NewDistance(pinionDedStart, cPt, g.Dedendum),
	)
	drivingDed, drivingDedStart, dPt := connect(s, "Driving Gear Dedendum", g.Apex2, g.D, lat.apex2, nil)
	dPt.SetName("D")
	lat.d = dPt
	lat.dedG = drivingDed
	s.AddConstraint(
		sketch.NewAngle(pitchLine, drivingDed, signedAngleDeg(g.PitchDir, g.DedG)),
		sketch.NewDistance(drivingDedStart, dPt, g.Dedendum),
	)

	proofkit.Step(t, "the two Root Axes, Apex->C and Apex->D")
	pinionRootAxis, _, _ := connect(s, "Pinion Root Axis", g.Apex, g.C, lat.apex, lat.c)
	drivingRootAxis, _, _ := connect(s, "Driving Root Axis", g.Apex, g.D, lat.apex, lat.d)

	proofkit.Step(t, "the pinion extension chain A->E, C->E, E->G, C->H, G->H")
	aToE, _, ePt := connect(s, "A->E", g.A, g.E, lat.a, nil)
	ePt.SetName("E")
	lat.e = ePt
	s.AddConstraint(sketch.NewPointOnLine(ePt, pinionShaft))
	cToE, _, _ := connect(s, "C->E", g.C, g.E, lat.c, lat.e)
	s.AddConstraint(sketch.NewAngle(aToE, cToE, signedAngleDeg(g.PinionDir, g.E.sub(g.C))))

	_, _, gPt := connect(s, "E->G", g.E, g.G, lat.e, nil)
	gPt.SetName("G")
	lat.g = gPt
	// The collinear names A->E, never the Apex->A shaft axis further up the
	// same chain: addCollinear carries two point-on-line rows and the farther
	// reading asserts one of them twice ([BEVEL-F-COLLINEAR-CHAIN]).
	s.AddConstraint(sketch.NewPointOnLine(gPt, aToE))

	cToH, _, hPt := connect(s, "C->H", g.C, g.H, lat.c, nil)
	hPt.SetName("H")
	lat.h = hPt
	lat.cToH = cToH
	s.AddConstraint(sketch.NewPointOnLine(hPt, pinionDed))
	gToH, _, _ := connect(s, "G->H", g.G, g.H, lat.g, lat.h)

	proofkit.Step(t, "the driving extension chain B->F, D->F, F->I, D->J, I->J")
	bToF, _, fPt := connect(s, "B->F", g.B, g.F, lat.b, nil)
	fPt.SetName("F")
	lat.f = fPt
	s.AddConstraint(sketch.NewPointOnLine(fPt, drivingShaft))
	dToF, _, _ := connect(s, "D->F", g.D, g.F, lat.d, lat.f)
	s.AddConstraint(sketch.NewAngle(bToF, dToF, signedAngleDeg(g.DrivingDir, g.F.sub(g.D))))

	_, _, iPt := connect(s, "F->I", g.F, g.I, lat.f, nil)
	iPt.SetName("I")
	lat.i = iPt
	s.AddConstraint(sketch.NewPointOnLine(iPt, bToF))

	dToJ, _, jPt := connect(s, "D->J", g.D, g.J, lat.d, nil)
	jPt.SetName("J")
	lat.j = jPt
	lat.dToJ = dToJ
	s.AddConstraint(sketch.NewPointOnLine(jPt, drivingDed))
	iToJ, _, _ := connect(s, "I->J", g.I, g.J, lat.i, lat.j)

	proofkit.Step(t, "the two base-height offsets, signed")
	s.AddConstraint(
		sketch.NewOffset(bDrop, iToJ, leftOffset(g.B, g.Apex2, g.I)),
		sketch.NewOffset(aDrop, gToH, leftOffset(g.A, g.Apex2, g.G)),
	)

	proofkit.Step(t, "A'->G, the hexagon's shaft-axis edge, which is what creates A'")
	_, apPt, _ := connect(s, "A'->G", g.Ap, g.G, nil, lat.g)
	apPt.SetName("A'")
	lat.ap = apPt

	proofkit.Step(t, "Constrain Point I with the projected centre")
	// Fusion writes addCoincident(I, projectedCenter), two rows. Only one of
	// them is independent here: the chain centre -> Apex -> B -> I runs along
	// the in-plane perpendicular by construction, so I already shares the
	// centre's coordinate across that perpendicular and the engine reports the
	// second row redundant. The proof applies the one row that closes the
	// chain, along the grow direction.
	s.AddConstraint(sketch.NewVerticalDistance(lat.centre, lat.i, 0))

	proofkit.Step(t, "K and the pinion tooth-centre reference line")
	_, _, kPt := connect(s, "G->K", g.G, g.K, lat.g, nil)
	kPt.SetName("K")
	lat.k = kPt
	s.AddConstraint(
		sketch.NewPointOnLine(kPt, pinionShaft),
		sketch.NewPointOnLine(kPt, pinionDed),
	)
	lat.kp = kPt
	if g.ToothSpacing > 0 {
		_, _, kpPt := connect(s, "K->K'", g.K, g.Kp, lat.k, nil)
		kpPt.SetName("K'")
		lat.kp = kpPt
		s.AddConstraint(sketch.NewPointOnLine(kpPt, pinionDed))
		addSignedStep(s, kPt, kpPt, g.Kp.sub(g.K))
		connect(s, "C->K'", g.C, g.Kp, lat.c, lat.kp)
	} else {
		connect(s, "C->K", g.C, g.K, lat.c, lat.k)
	}

	proofkit.Step(t, "L and the driving tooth-centre reference line")
	_, _, lPt := connect(s, "I->L", g.I, g.L, lat.i, nil)
	lPt.SetName("L")
	lat.l = lPt
	s.AddConstraint(
		sketch.NewPointOnLine(lPt, drivingShaft),
		sketch.NewPointOnLine(lPt, drivingDed),
	)
	lat.lp = lPt
	if g.ToothSpacing > 0 {
		_, _, lpPt := connect(s, "L->L'", g.L, g.Lp, lat.l, nil)
		lpPt.SetName("L'")
		lat.lp = lpPt
		s.AddConstraint(sketch.NewPointOnLine(lpPt, drivingDed))
		addSignedStep(s, lPt, lpPt, g.Lp.sub(g.L))
		connect(s, "D->L'", g.D, g.Lp, lat.d, lat.lp)
	} else {
		connect(s, "D->L", g.D, g.L, lat.d, lat.l)
	}

	proofkit.Step(t, "the pinion toe line M->N and the front face N->A'")
	mToN, mPt, nPt := connect(s, "M->N", g.M, g.N, nil, nil)
	mPt.SetName("M")
	nPt.SetName("N")
	lat.m, lat.n, lat.mToN = mPt, nPt, mToN
	s.AddConstraint(
		sketch.NewPointOnLine(mPt, pinionRootAxis),
		// Signed. Fusion needs addParallel(M->N, C->H) first because its own
		// offset dimension requires the two lines to be parallel already; the
		// engine's offset carries the direction itself.
		sketch.NewOffset(cToH, mToN, leftOffset(g.C, g.H, g.M)),
	)
	connect(s, "M->C", g.M, g.C, lat.m, lat.c)
	nToAp, _, _ := connect(s, "N->A'", g.N, g.Ap, lat.n, lat.ap)
	s.AddConstraint(
		sketch.NewPointOnLine(lat.ap, pinionShaft),
		// Fusion writes addPerpendicular(N->A', Apex->A), which is undirected,
		// and an unsigned length dimension on N->A'. Between them the toe line
		// meets the Toe Radius on BOTH sides of the shaft axis, so Fusion takes
		// whichever side the seed starts on and a seed below the axis builds
		// the mirror — the hexagon then crosses its own axis of revolution and
		// the revolve aborts with ASM_WIRE_X_AXIS. Measured here: the engine's
		// probe reports exactly those two configurations when the direction is
		// left undirected, N sliding along the toe line to the far side. The
		// proof SUBSTITUTES a signed angle for the perpendicular, which is the
		// same +/- 90 degrees with the side pinned instead of seeded.
		sketch.NewAngle(pinionShaft, nToAp, signedAngleDeg(g.PinionDir, g.Ap.sub(g.N))),
		sketch.NewDistance(lat.n, lat.ap, g.ToeRadiusP),
	)

	proofkit.Step(t, "the driving toe line O->P and the front face P->B'")
	oToP, oPt, pPt := connect(s, "O->P", g.O, g.P, nil, nil)
	oPt.SetName("O")
	pPt.SetName("P")
	lat.o, lat.pp, lat.oToP = oPt, pPt, oToP
	s.AddConstraint(
		sketch.NewPointOnLine(oPt, drivingRootAxis),
		sketch.NewOffset(dToJ, oToP, leftOffset(g.D, g.J, g.O)),
	)
	connect(s, "O->D", g.O, g.D, lat.o, lat.d)
	pToBp, _, bpPt := connect(s, "P->B'", g.P, g.Bp, lat.pp, nil)
	bpPt.SetName("B'")
	lat.bp = bpPt
	s.AddConstraint(
		sketch.NewPointOnLine(bpPt, drivingShaft),
		// Signed for the reason the pinion front face gives in full.
		sketch.NewAngle(drivingShaft, pToBp, signedAngleDeg(g.DrivingDir, g.Bp.sub(g.P))),
		sketch.NewDistance(lat.pp, bpPt, g.ToeRadiusG),
	)
	connect(s, "B'->I", g.Bp, g.I, lat.bp, lat.i)

	// ---------------------------------------------------------------------
	// End of §2. Everything below is assertion.
	// ---------------------------------------------------------------------
	solveHere(t, s)

	proofkit.Step(t, "the lattice assertion — the proof's copy of the [BEVEL-F-SEED-HELD] gate")
	// The module's gate compares the solved figure against the same closed
	// forms, in creation order, and raises on the first point that moved. What
	// neither reaches: the proof SEEDS at the closed form, so it proves the
	// constraints solve from a correct seed and never that the generated
	// module's seed is correct. That is why the gate has to exist inside the
	// module as well, and it is the same limit the toe-line seeding records.
	type named struct {
		name string
		got  *sketch.Point
		want pt
	}
	points := []named{
		{"Apex", lat.apex, g.Apex}, {"B", lat.b, g.B}, {"A", lat.a, g.A},
		{"Apex 2", lat.apex2, g.Apex2}, {"C", lat.c, g.C}, {"D", lat.d, g.D},
		{"E", lat.e, g.E}, {"F", lat.f, g.F}, {"G", lat.g, g.G}, {"H", lat.h, g.H},
		{"I", lat.i, g.I}, {"J", lat.j, g.J}, {"K", lat.k, g.K},
	}
	if g.ToothSpacing > 0 {
		points = append(points, named{"K'", lat.kp, g.Kp})
	}
	points = append(points,
		named{"M", lat.m, g.M}, named{"N", lat.n, g.N}, named{"A'", lat.ap, g.Ap},
		named{"L", lat.l, g.L})
	if g.ToothSpacing > 0 {
		points = append(points, named{"L'", lat.lp, g.Lp})
	}
	points = append(points,
		named{"O", lat.o, g.O}, named{"P", lat.pp, g.P}, named{"B'", lat.bp, g.Bp})

	want := 20
	if g.ToothSpacing > 0 {
		want = 22
	}
	if len(points) != want {
		t.Fatalf("the lattice assertion covers %d points, want %d at Tooth Spacing %g",
			len(points), want, g.ToothSpacing)
	}
	for _, n := range points {
		requireNear(t, n.name, at(n.got), n.want, seedTolerance)
	}

	proofkit.Step(t, "the two Tooth Spacing sites carry a sign, not just a magnitude")
	// NewDistance is unsigned, and the twin it would admit puts K' one Tooth
	// Spacing on the C side of K — two candidates 2 x Tooth Spacing apart. The
	// step pins the pair with a signed axis distance instead; this assertion
	// says which of the two signs that stands in for, and a clean ambiguity
	// probe is NOT what rules the twin out, because the engine documents its
	// probe as reporting a LOWER BOUND on the number of solutions.
	if g.ToothSpacing > 0 {
		requireClose(t, "K' - K projected on Apex2->C",
			at(lat.kp).sub(at(lat.k)).dot(g.DedP), +g.ToothSpacing, 1e-6)
		requireClose(t, "L' - L projected on Apex2->D",
			at(lat.lp).sub(at(lat.l)).dot(g.DedG), +g.ToothSpacing, 1e-6)
	}

	proofkit.Step(t, "Maximum Face Width from the SOLVED geometry, not from the seeds")
	dp := distanceToLine(at(lat.a), at(lat.c), at(lat.h).sub(at(lat.c)))
	dg := distanceToLine(at(lat.b), at(lat.d), at(lat.j).sub(at(lat.d)))
	measured := 0.95 * math.Min(dp, dg)
	// The closed form the two perpendicular distances must equal is
	// R sin^2(gamma) on each side, which at Shaft Angle 90 degrees reduces to
	// the spec's 0.95 * min(DPD, PPD)^2 / (2 * Cone Distance) — the SMALLER
	// pitch diameter, never the pinion's by name.
	requireClose(t, "perpendicular distance A to the pinion dedendum line",
		dp, g.R*math.Pow(math.Sin(g.GammaP), 2), 1e-6)
	requireClose(t, "perpendicular distance B to the driving dedendum line",
		dg, g.R*math.Pow(math.Sin(g.GammaG), 2), 1e-6)
	requireClose(t, "Maximum Face Width", measured, g.MaxFaceWidth, 1e-6)
	if g.FaceWidth > measured+1e-9 {
		t.Errorf("resolved Face Width %.6f mm exceeds the Maximum Face Width %.6f mm", g.FaceWidth, measured)
	}
	if math.Abs(g.Sigma-math.Pi/2) < 1e-12 {
		smaller := math.Min(g.DPD, g.PPD)
		requireClose(t, "Maximum Face Width at Shaft Angle 90",
			measured, 0.95*smaller*smaller/(2*g.ConeDist), 1e-6)
	}

	proofkit.Step(t, "the Maximum Bore Diameter, whose toe term needs the Root Length")
	for _, which := range []gearSide{pinion, driving} {
		sd := g.side(which)
		if !g.BoreEnable {
			continue
		}
		if sd.bore > sd.maxBore+1e-9 {
			t.Errorf("%s resolved bore diameter %.6f mm exceeds its Maximum Bore Diameter %.6f mm",
				which.label(), sd.bore, sd.maxBore)
		}
		// r_heel is where H (resp. J) lands: the base height is measured from
		// Apex 2's plane, so H sits <base height>/tan(gamma) inside the pitch
		// radius. Read it off the solved heel point rather than restating it.
		heel := sd.heelEdge[1]
		requireClose(t, which.label()+" r_heel from the solved heel point",
			sd.radius(g, heel), sd.pitchRadius-sd.baseHeight/math.Tan(sd.gamma), 1e-6)
	}

	proofkit.Step(t, "the toe sits inside the heel on both gears")
	// addOffsetDimension is unsigned in Fusion, so a correctly built frame
	// still admits the toe line one root length on the FAR side of the heel,
	// where the revolved frustum is degenerate and the conical end cut finds
	// no cone face at the toe midpoint. The seed is the whole of the rule in
	// Fusion; here the signed offset carries it and this reading confirms it.
	for _, which := range []gearSide{pinion, driving} {
		sd := g.side(which)
		toe := sd.distAlong(g, sd.toeEdge[0])
		heel := sd.distAlong(g, sd.heelEdge[0])
		if toe >= heel {
			t.Errorf("%s toe cone distance %.6f is not inside its heel %.6f — the figure is inverted",
				which.label(), toe, heel)
		}
		requireClose(t, which.label()+" |Ded->Toe| along the root element", heel-toe, g.RootLen, 1e-6)
	}

	proofkit.Step(t, "N and P stay off their shaft axes, at the resolved Toe Radius")
	requireClose(t, "N's perpendicular distance from the pinion shaft axis",
		distanceToLine(at(lat.n), at(lat.apex), g.PinionDir), g.ToeRadiusP, 1e-6)
	requireClose(t, "P's perpendicular distance from the driving shaft axis",
		distanceToLine(at(lat.pp), at(lat.apex), g.DrivingDir), g.ToeRadiusG, 1e-6)
	if g.ToeRadiusP <= 0 || g.ToeRadiusG <= 0 {
		t.Errorf("both Toe Radii must be strictly positive so N and P never sit on the axis of "+
			"revolution, got pinion %.6f driving %.6f", g.ToeRadiusP, g.ToeRadiusG)
	}

	proofkit.Step(t, "|Apex2 -> K| is the exact back-cone radius, never a rounded count")
	requireClose(t, "|Apex2 -> K|", at(lat.k).distance(at(lat.apex2)), g.VirtualRadiusP, 1e-6)
	requireClose(t, "|Apex2 -> L|", at(lat.l).distance(at(lat.apex2)), g.VirtualRadiusG, 1e-6)
	rounded := math.Round(g.VirtualTeethP) * g.Module / 2
	if math.Abs(rounded-g.VirtualRadiusP) > seedTolerance {
		proofkit.Step(t, "a rounded virtual count would put the K seed %.4f mm out, %.0f times the "+
			"[BEVEL-F-SEED-HELD] tolerance", math.Abs(rounded-g.VirtualRadiusP),
			math.Abs(rounded-g.VirtualRadiusP)/seedTolerance)
	}
}

// addSignedStep pins a Tooth Spacing offset with the signed axis distance that
// carries the larger component of the step, so the pin never degenerates as
// the dedendum direction swings with the Shaft Angle. Fusion has only an
// unsigned length dimension here; this is the signed constraint the spec's
// "Proving the §2 figure" section asks for in its place.
func addSignedStep(s *sketch.Sketch, from, to *sketch.Point, delta pt) {
	if math.Abs(delta.X) >= math.Abs(delta.Y) {
		s.AddConstraint(sketch.NewHorizontalDistance(from, to, delta.X))
		return
	}
	s.AddConstraint(sketch.NewVerticalDistance(from, to, delta.Y))
}

// ---------------------------------------------------------------------------
// S09 {gearLabel} Tooth sketch
// ---------------------------------------------------------------------------

// virtualToothRing is the tooth's closed boundary in the back-cone plane, centred on
// the tooth-centre point and already rotated 180 degrees, which is the angle
// bevel passes to the borrowed spur drawer's draw().
type virtualToothRing struct {
	left, right []involute.Pt
	dims        involute.Dimensions
	root        float64 // the SUNK root radius
	embedded    bool
	startRadius float64
}

// buildVirtualToothRing samples the two flanks from wherever the tooth's root
// actually starts out to the tip, using the shared involute math rather than
// deriving it again. The root circle is drawn one root sink INSIDE the
// dedendum corner, which is what makes the Combine-Join meet the gear body
// across the whole root rather than along the tooth's centreline alone.
func buildVirtualToothRing(g geometry, sd side) virtualToothRing {
	d := involute.Derive(g.Module, sd.virtualTeeth, proxyPressureAngle)
	root := d.Root - g.RootSink
	start := math.Max(d.Base, root)
	embedded := d.Base < root

	mirrored := make([]involute.Pt, 0, proxyInvoluteSteps)
	for i := 0; i < proxyInvoluteSteps; i++ {
		r := start + (d.Tip-start)*float64(i)/float64(proxyInvoluteSteps-1)
		x, y, ok := involute.Point(d.Base, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, involute.Pt{X: x, Y: -y})
	}
	px, py, _ := involute.Point(d.Base, d.Pitch)
	// The same placement rule involute.Flanks uses: rotate so the pitch
	// crossing lands at +pi/(2N), which centres the tooth on +X, then apply
	// the 180 degree draw angle to both flanks together.
	place := math.Pi/(2*sd.virtualTeeth) - math.Atan2(-py, px) + math.Pi

	ring := virtualToothRing{dims: d, root: root, embedded: embedded, startRadius: start}
	for _, m := range mirrored {
		lx, ly := involute.Rotate(m.X, m.Y, place-math.Pi)
		rx, ry := lx, -ly
		lx, ly = involute.Rotate(lx, ly, math.Pi)
		rx, ry = involute.Rotate(rx, ry, math.Pi)
		ring.left = append(ring.left, involute.Pt{X: lx, Y: ly})
		ring.right = append(ring.right, involute.Pt{X: rx, Y: ry})
	}
	return ring
}

func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	ring := buildVirtualToothRing(g, sd)

	proofkit.Step(t, "%s: virtual pitch radius %.6f mm, virtual tooth number %.6f (never rounded)",
		sd.which.label(), sd.virtualRadius, sd.virtualTeeth)

	origin := s.CreatePoint(0, 0)
	origin.SetName("tooth centre")
	anchor := s.CreateReferencePoint(0, 0, "§2 tooth-centre point K'/L' projected into the Tooth sketch")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the four circles the proxy is asked for")
	// pitch, base, tip and root, all construction: Fusion's profile finder
	// also sees the regions these circles cut, and the proof does not
	// reproduce that search. What it pins is the curve-count key the finder
	// uses, which is the tooth loop's own mix.
	for _, c := range []struct {
		name string
		r    float64
	}{
		{"pitch circle", sd.virtualRadius},
		{"base circle", ring.dims.Base},
		{"tip circle", ring.dims.Tip},
		{"root circle", ring.root},
	} {
		centre := s.CreatePoint(0, 0)
		circle := s.CreateCircle(centre, c.r)
		circle.SetName(c.name)
		circle.SetConstruction(true)
		s.AddConstraint(
			sketch.NewCoincident(centre, origin),
			sketch.NewDiameter(circle, 2*c.r),
		)
	}
	requireClose(t, sd.which.label()+" pitch circle radius", sd.virtualRadius, ring.dims.Pitch, 1e-9)
	requireClose(t, sd.which.label()+" tip circle radius", ring.dims.Tip, sd.virtualRadius+g.Module, 1e-9)
	requireClose(t, sd.which.label()+" root circle radius", ring.root,
		sd.virtualRadius-1.25*g.Module-g.RootSink, 1e-9)
	requireClose(t, sd.which.label()+" root sink", g.RootSink, 0.05*2.25*g.Module, 1e-12)

	proofkit.Step(t, "the two involute flanks, each point pinned by a signed axis pair")
	leftPts := make([]*sketch.Point, len(ring.left))
	rightPts := make([]*sketch.Point, len(ring.right))
	for i := range ring.left {
		leftPts[i] = pinned(s, origin, ring.left[i].X, ring.left[i].Y, fmt.Sprintf("left flank %d", i))
		rightPts[i] = pinned(s, origin, ring.right[i].X, ring.right[i].Y, fmt.Sprintf("right flank %d", i))
	}
	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("right flank")

	proofkit.Step(t, "tip arc, root arc and — when the tooth is not embedded — the two root stubs")
	last := len(ring.left) - 1
	tipArc := s.CreateArc(s.CreatePoint(0, 0), rightPts[last], leftPts[last])
	tipArc.SetName("tip arc")
	// Fusion draws this as a three-point arc and dimensions its radius. Here
	// the two ends are already pinned, so the arc's own radius-consistency row
	// holds the centre on their perpendicular bisector — which is the tooth's
	// centreline, because the two flanks are exact mirrors — and a radius
	// dimension on top of it would leave the centre free to reflect across the
	// chord. Measured: with the dimension the engine's probe reports four
	// configurations, two per arc. The proof pins the remaining freedom with
	// one signed row instead and ASSERTS the radius below.
	s.AddConstraint(sketch.NewHorizontalDistance(origin, tipArc.Center, 0))

	rootStart, rootEnd := rightPts[0], leftPts[0]
	lines := 0
	if !ring.embedded {
		// The flank starts at the base circle, outside the sunk root circle,
		// so the tooth carries the two connecting lines the spur drawer adds.
		lines = 2
		lx, ly := radialTo(ring.left[0], ring.root)
		rx, ry := radialTo(ring.right[0], ring.root)
		lRoot := pinned(s, origin, lx, ly, "left root")
		rRoot := pinned(s, origin, rx, ry, "right root")
		s.CreateLine(leftPts[0], lRoot).SetName("left flank-to-root line")
		s.CreateLine(rightPts[0], rRoot).SetName("right flank-to-root line")
		rootStart, rootEnd = rRoot, lRoot
	}
	rootArc := s.CreateArc(s.CreatePoint(0, 0), rootStart, rootEnd)
	rootArc.SetName("root arc")
	s.AddConstraint(sketch.NewHorizontalDistance(origin, rootArc.Center, 0))

	solveHere(t, s)

	proofkit.Step(t, "the tooth loop carries the curve mix the profile finder keys on")
	// find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)
	// with wantLines DETERMINED BY the embedded flag, never accepted as either.
	wantLines := 2
	if ring.embedded {
		wantLines = 0
	}
	if wantLines != lines {
		t.Fatalf("the embedded flag says %d connecting lines but the tooth was drawn with %d",
			wantLines, lines)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		t.Fatalf("%s tooth: want exactly one valid region, got %d", sd.which.label(), len(profiles))
	}
	var splines, arcs, straights int
	for _, e := range profiles[0].Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			straights++
		}
	}
	if splines != 2 || arcs != 2 || straights != wantLines {
		t.Errorf("%s tooth loop carries %d NURBS, %d arcs and %d lines; the finder is asked for "+
			"2, 2 and %d", sd.which.label(), splines, arcs, straights, wantLines)
	}

	proofkit.Step(t, "the arc centres are not stranded")
	// addByCenterStartEnd COPIES the centre in Fusion rather than sharing it,
	// which stranded the bevel tooth-top arc 22.9 mm behind its origin and gave
	// a 0.5743 mm arc where 22.5 mm was intended, on a sketch that raised no
	// error. Here the centre is free and is pinned by the two shared ends plus
	// the diameter, so this reads whether that pinning actually lands it.
	requireNear(t, "tip arc centre", at(tipArc.Center), pt{0, 0}, 1e-6)
	requireNear(t, "root arc centre", at(rootArc.Center), pt{0, 0}, 1e-6)
	requireClose(t, sd.which.label()+" tip arc radius", tipArc.R(), ring.dims.Tip, 1e-6)
	requireClose(t, sd.which.label()+" root arc radius", rootArc.R(), ring.root, 1e-6)

	proofkit.Step(t, "the tooth carries the standard thickness at the pitch circle")
	// The real count reaches the drawer only as the angular half-thickness
	// pi/(2*z_v). With z_v = 2*r_v/Module that gives pi*Module/2 of tooth at
	// the pitch circle — the same thickness the spur gear of this module
	// carries. An INTEGER count at the exact radius gives pi*r_v/round(z_v),
	// which misses nominal by a different amount on each member of a pair.
	requireClose(t, sd.which.label()+" tooth thickness at the pitch circle",
		2*sd.virtualRadius*math.Pi/(2*sd.virtualTeeth), math.Pi*g.Module/2, 1e-9)
}

// pinned creates a point and holds it at a SIGNED offset from the anchor on
// both axes, which is the engine's way of writing a placed point that stays
// inside the parameter model.
func pinned(s *sketch.Sketch, anchor *sketch.Point, x, y float64, name string) *sketch.Point {
	p := s.CreatePoint(x, y)
	p.SetName(name)
	s.AddConstraint(
		sketch.NewHorizontalDistance(anchor, p, x),
		sketch.NewVerticalDistance(anchor, p, y),
	)
	return p
}

// radialTo returns the point at radius r in the same direction as p.
func radialTo(p involute.Pt, r float64) (float64, float64) {
	l := math.Hypot(p.X, p.Y)
	return p.X / l * r, p.Y / l * r
}

// ---------------------------------------------------------------------------
// S12 {gearLabel} Profile sketch
// ---------------------------------------------------------------------------

// stepProfileSketch rebuilds the per-gear hexagon. Fusion recreates the six §2
// vertices as new points at their world-mapped positions, draws the six lines
// SHARING those points, then fixes the lines' endpoints AFTER the lines exist;
// the engine has no equivalent of that ordering rule, so the proof grounds the
// first vertex on the sketch origin and holds the other five at signed offsets
// from it, which is the same placed figure inside the parameter model.
func stepProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	hex := sd.hexProfile(g)

	proofkit.Step(t, "%s Profile: the six §2 vertices in the draw order the first-edge rule needs",
		sd.which.label())
	anchor := s.CreateReferencePoint(hex[0].X, hex[0].Y, "§2 hexagon vertex A'/B' recreated in the Profile sketch")
	first := s.CreatePoint(hex[0].X, hex[0].Y)
	first.SetName(vertexName(sd.which, 0))
	s.AddConstraint(sketch.NewCoincident(first, anchor))

	verts := make([]*sketch.Point, 6)
	verts[0] = first
	for i := 1; i < 6; i++ {
		verts[i] = pinned(s, first, hex[i].X-hex[0].X, hex[i].Y-hex[0].Y, vertexName(sd.which, i))
		// pinned holds a signed offset FROM the anchor, so recentre the point
		// on its true position before the lines are drawn.
		verts[i].MoveTo(hex[i].X, hex[i].Y)
	}

	edges := make([]*sketch.Line, 6)
	for i := range verts {
		edges[i] = s.CreateLine(verts[i], verts[(i+1)%6])
		edges[i].SetName(fmt.Sprintf("%s hexagon edge %d", sd.which.label(), i))
	}

	solveHere(t, s)

	proofkit.Step(t, "exactly one closed loop, so the revolve takes profiles.item(0) without filtering")
	profiles := s.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		t.Fatalf("%s Profile: want exactly one valid hexagon loop, got %d", sd.which.label(), len(profiles))
	}
	area := math.Abs(shoelace(hex[:]))
	requireClose(t, sd.which.label()+" hexagon area", math.Abs(profiles[0].Area), area, 1e-6)

	proofkit.Step(t, "the FIRST edge is the shaft axis, and both its ends sit on that axis")
	// Every body operation takes this edge: the revolve axis, the pattern
	// axis, the bore plane's path and the meshing rotation. A free edge
	// resolves against a default frame and silently moves the body onto world
	// XY, which is why its endpoints have to be pinned before it is read.
	requireClose(t, sd.which.label()+" first edge start radius", hex[0].Y, 0, 1e-9)
	requireClose(t, sd.which.label()+" first edge end radius", hex[1].Y, 0, 1e-9)
	for i := 2; i < 6; i++ {
		if hex[i].Y <= 0 {
			t.Errorf("%s hexagon vertex %s sits on or across the axis of revolution at radius %.6f; "+
				"the revolve would abort with ASM_WIRE_X_AXIS",
				sd.which.label(), vertexName(sd.which, i), hex[i].Y)
		}
	}

	proofkit.Step(t, "the two end faces stand square to the shaft, and the toe sits inside the heel")
	// The front face N->A' and the back face G->H are each perpendicular to
	// the shaft axis, so each pair shares a station: that is what makes the
	// revolve sweep them into flat annuli rather than cones.
	requireClose(t, sd.which.label()+" front face station", hex[5].X, hex[0].X, 1e-9)
	requireClose(t, sd.which.label()+" back face station", hex[2].X, hex[1].X, 1e-9)
	if !(hex[4].X < hex[3].X && hex[3].X < hex[1].X) {
		t.Errorf("%s hexagon stations are not ordered M/O < C/D < G/I: %v", sd.which.label(), hex)
	}
}

func vertexName(which gearSide, i int) string {
	pinionNames := [6]string{"A'", "G", "H", "C", "M", "N"}
	drivingNames := [6]string{"B'", "I", "J", "D", "O", "P"}
	if which == driving {
		return drivingNames[i]
	}
	return pinionNames[i]
}

func shoelace(poly []pt) float64 {
	total := 0.0
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		total += a.X*b.Y - b.X*a.Y
	}
	return total / 2
}

// ---------------------------------------------------------------------------
// S26 {gearLabel} Bore sketch
// ---------------------------------------------------------------------------

func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	if !g.BoreEnable {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no bore sketch is authored at all")
		return
	}

	proofkit.Step(t, "%s Bore: the circle on the plane rooted at the shaft start", sd.which.label())
	// The plane is setByDistanceOnPath(shaft edge, 0.0), so its origin is on
	// the axis and the circle is centred on the sketch origin. In Fusion the
	// centre is FIXED rather than coincident to originPoint, which has thrown
	// VCS_SKETCH_SOLVING_FAILED on such a plane ([PB-CIRCLE-CENTER]); the
	// engine has no such failure and a coincidence to its own origin is the
	// same pin inside the parameter model.
	centre := s.CreatePoint(0, 0)
	centre.SetName("bore centre")
	s.AddConstraint(sketch.NewCoincident(centre, s.Origin()))
	circle := s.CreateCircle(centre, sd.bore/2)
	circle.SetName(sd.which.label() + " bore circle")
	s.AddConstraint(sketch.NewDiameter(circle, sd.bore))

	solveHere(t, s)

	requireClose(t, sd.which.label()+" bore diameter", 2*circle.R(), sd.bore, 1e-9)
	if sd.bore > sd.maxBore+1e-9 {
		t.Errorf("%s bore diameter %.6f mm exceeds its Maximum Bore Diameter %.6f mm",
			sd.which.label(), sd.bore, sd.maxBore)
	}
	if profiles := s.Profiles(); len(profiles) != 1 || !profiles[0].Valid {
		t.Fatalf("%s Bore: want exactly one valid region, got %d", sd.which.label(), len(profiles))
	}
}

// ---------------------------------------------------------------------------
// S16 {gear} 2D Tooth Trace sketch
// ---------------------------------------------------------------------------

// trace is the cutter-arc construction of spiral-tooth-trace.md, in the flat
// tangent-plane frame: apex at the origin, x along the cone element (so a
// point's x is its cone distance) and y circumferential.
type trace struct {
	rToe, rHeel, rMean, span float64
	rc                       float64
	handSign                 float64
	cx, cy                   float64
	toe2d, heel2d            pt
	phiCrown                 float64
	total                    float64
}

func newTrace(g geometry, sd side, psi, hand, cutter float64) trace {
	var tr trace
	toeMid := sd.toeEdge[0].add(sd.toeEdge[1]).scale(0.5)
	heelMid := sd.heelEdge[0].add(sd.heelEdge[1]).scale(0.5)
	tr.rToe = sd.distAlong(g, toeMid)
	tr.rHeel = sd.distAlong(g, heelMid)
	tr.rMean = 0.5 * (tr.rToe + tr.rHeel)
	tr.span = tr.rHeel - tr.rToe

	tr.rc = cutter
	if tr.rc == 0 {
		tr.rc = tr.rMean
	}
	// The driving gear uses the dialog's hand; the meshing pinion is built
	// with the opposite one.
	tr.handSign = hand
	if sd.which == pinion {
		tr.handSign = -hand
	}
	// The hand sign goes on the cos/Cy term, NOT the sin/Cx term: opposite
	// hands mirror the cutter centre across the cone element, which flips Cy.
	// Putting it on Cx mirrors about x = R_mean instead, a different curve
	// that gives the two gears unequal twist.
	tr.cx = tr.rMean - tr.rc*math.Sin(psi)
	tr.cy = tr.handSign * tr.rc * math.Cos(psi)

	rLo := tr.rToe - 0.06*tr.span
	rHi := tr.rHeel + 0.06*tr.span
	tr.toe2d = circleIntersectNearest(rLo, tr.cx, tr.cy, tr.rc, tr.rMean, 0)
	tr.heel2d = circleIntersectNearest(rHi, tr.cx, tr.cy, tr.rc, tr.rMean, 0)

	tr.phiCrown = math.Atan2(tr.heel2d.Y, tr.heel2d.X) - math.Atan2(tr.toe2d.Y, tr.toe2d.X)
	tr.total = math.Abs(tr.phiCrown) / math.Sin(sd.gamma)
	return tr
}

// circleIntersectNearest intersects the apex circle of radius r with the
// cutter circle and keeps the solution nearest the reference point, which is
// the branch the mean point sits on. A non-overlapping pair clamps to
// tangency, exactly as the framework helper does.
func circleIntersectNearest(r, cx, cy, rc, refX, refY float64) pt {
	d := math.Hypot(cx, cy)
	a := (d*d + r*r - rc*rc) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	base := pt{cx / d * a, cy / d * a}
	off := pt{-cy / d * h, cx / d * h}
	one, two := base.add(off), base.sub(off)
	ref := pt{refX, refY}
	if one.distance(ref) <= two.distance(ref) {
		return one
	}
	return two
}

func stepTraceSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	psi := p["spiralAngle"] * math.Pi / 180
	if psi <= 0 {
		proofkit.Unmodelled(t, "Mean Spiral Angle is 0, so the tooth-body hook returns the straight "+
			"path before any trace geometry is built; there is no trace sketch to prove")
		return
	}
	tr := newTrace(g, sd, psi, p["hand"], p["cutterRadius"])

	proofkit.Step(t, "%s 2D Tooth Trace: R_toe %.4f R_mean %.4f R_heel %.4f r_c %.4f handSign %+.0f",
		sd.which.label(), tr.rToe, tr.rMean, tr.rHeel, tr.rc, tr.handSign)

	// The Fusion sketch is deliberately left with free DOF — the arc's
	// endpoints are pinned by the 3-point construction rather than dimensioned
	// — and is exempt from the full-constraint gate. proofkit waives nothing,
	// so the proof SUBSTITUTES a fully determined figure: the two endpoints
	// are held at the closed-form circle-circle solutions by signed axis
	// distances, and the arc is then built through them with its radius
	// dimensioned. The cost is the branch selection: the engine has no signed
	// circle-intersection constraint, so which of the two intersections the
	// build takes is ASSERTED below rather than constrained.
	apex := s.CreatePoint(0, 0)
	apex.SetName("apex")
	s.AddConstraint(sketch.NewCoincident(apex, s.Origin()))

	cc := pinned(s, apex, tr.cx, tr.cy, "cutter circle centre")
	cutter := s.CreateCircle(cc, tr.rc)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*tr.rc))

	mean := pinned(s, apex, tr.rMean, 0, "mean point on the cone element")
	toe := pinned(s, apex, tr.toe2d.X, tr.toe2d.Y, "trace toe end")
	heel := pinned(s, apex, tr.heel2d.X, tr.heel2d.Y, "trace heel end")

	start, end := toe, heel
	if cross := tr.toe2d.sub(pt{tr.cx, tr.cy}).cross(tr.heel2d.sub(pt{tr.cx, tr.cy})); cross < 0 {
		start, end = heel, toe
	}
	arc := s.CreateArc(s.CreatePoint(tr.cx, tr.cy), start, end)
	arc.SetName("trace arc")
	arc.SetConstruction(true)
	// Fusion dimensions the arc's radius at r_c. Here both ends are already
	// pinned, so the arc's own radius-consistency row holds its centre on
	// their perpendicular bisector and a radius dimension on top of that
	// leaves the centre free to reflect across the chord — the engine's probe
	// reports both. One signed row along the bisector pins it instead, taken
	// on whichever axis the bisector leans toward so the pin never
	// degenerates, and the radius is asserted below.
	bisector := pt{-(tr.heel2d.Y - tr.toe2d.Y), tr.heel2d.X - tr.toe2d.X}
	if math.Abs(bisector.X) >= math.Abs(bisector.Y) {
		s.AddConstraint(sketch.NewHorizontalDistance(apex, arc.Center, tr.cx))
	} else {
		s.AddConstraint(sketch.NewVerticalDistance(apex, arc.Center, tr.cy))
	}

	coneElement := s.CreateLine(apex, mean)
	coneElement.SetName("cone element")
	coneElement.SetConstruction(true)

	solveHere(t, s)

	proofkit.Step(t, "the arc is the genuine cutter circle, not a look-alike")
	requireNear(t, "trace arc centre against the cutter circle centre", at(arc.Center), at(cc), 1e-6)
	requireClose(t, "trace arc radius", arc.R(), tr.rc, 1e-6)

	proofkit.Step(t, "invariants 1, 3, 4 and 6 of spiral-tooth-trace.md §9")
	requireClose(t, "|C - M|, so the cutter circle passes through the mean point",
		at(cc).distance(at(mean)), tr.rc, 1e-6)
	requireClose(t, "toe end cone distance", at(toe).len(), tr.rToe-0.06*tr.span, 1e-6)
	requireClose(t, "heel end cone distance", at(heel).len(), tr.rHeel+0.06*tr.span, 1e-6)
	// The tangent at the mean point makes psi with the element. The radius
	// M->C is perpendicular to that tangent, so the angle between M->C and the
	// y axis is psi.
	toCentre := at(cc).sub(at(mean)).unit()
	requireClose(t, "spiral angle realised at the mean point",
		math.Abs(math.Asin(toCentre.X*-1)), psi, 1e-9)

	proofkit.Step(t, "the hand mirrors the construction across the cone element and nothing else")
	mirror := newTrace(g, sd, psi, -p["hand"], p["cutterRadius"])
	requireClose(t, "opposite hand mirrors Cy", mirror.cy, -tr.cy, 1e-9)
	requireClose(t, "opposite hand leaves Cx alone", mirror.cx, tr.cx, 1e-12)
	requireClose(t, "opposite hand leaves the twist magnitude alone", mirror.total, tr.total, 1e-9)
}

// signedAngleDeg is the counter-clockwise angle from one direction to another,
// in degrees, which is what the engine's NewAngle takes. Fusion's
// addAngularDimension is an unsigned magnitude whose quadrant comes from a text
// point, so every place this proof passes a signed angle is a place the module
// has to hold the side with its seed instead.
func signedAngleDeg(from, to pt) float64 {
	return math.Atan2(from.cross(to), from.dot(to)) * 180 / math.Pi
}
