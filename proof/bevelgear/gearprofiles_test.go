package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// stepGearProfiles draws the §2 Gear Profiles sketch — the whole lattice, in one
// sketch, exactly as one Fusion timeline entry.
//
// Two substitutions run through it, both stated where they are made:
//
//   - Every Fusion perpendicular or parallel constraint whose SIDE the drawing
//     seed decides is written here as a signed angle, and every length dimension
//     whose side the seed decides as a signed component. Fusion carries the side
//     in the seed and the magnitude in the dimension; the engine refuses that,
//     because a scheme that reaches DOF 0 while still admitting the mirror answer
//     fails its ambiguity probe. What this costs: the proof shows the scheme is
//     rigid ONCE the side is stated, not that Fusion's seeds state it.
//
//   - The two base-height offset dimensions and the two toe-line offset
//     dimensions are the engine's signed offset, which carries the parallelism
//     Fusion splits between a perpendicular constraint and the dimension. The
//     equation count is the same; writing both would be the redundancy the
//     playbook's offset-dimension rule warns about.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	sd := seedLattice(p)
	want := solveLattice(p)
	c := coneOf(p)
	f := newFigure(s)

	proofkit.Step(t, "project the anchor sketch's centre point and its anchor line")
	anchorStart := f.ref("anchorStart", sub(sd.Centre, vec{5, 0}))
	anchorEnd := f.ref("anchorEnd", add(sd.Centre, vec{5, 0}))
	anchor, err := s.CreateReferenceLine(anchorStart, anchorEnd, "anchor sketch")
	if err != nil {
		t.Fatalf("project the anchor line: %v", err)
	}
	anchor.SetConstruction(true)
	centre := f.ref("centre", sd.Centre)
	anchorDir := vec{1, 0}

	proofkit.Step(t, "centre to Apex, perpendicular to the projected anchor line")
	centerToApex := f.line("centerToApex", sd.Centre, sd.Apex)
	f.meet(centerToApex.Start, centre)
	f.angle(anchor, centerToApex, signedAngle(anchorDir, unit(sub(sd.Apex, sd.Centre))))
	apex := f.pin("Apex", centerToApex.End)

	proofkit.Step(t, "the two shaft axes, and the shaft angle between them")
	drivingShaft := f.line("Apex->B", sd.Apex, sd.B)
	f.meet(drivingShaft.Start, apex)
	s.AddConstraint(sketch.NewParallel(drivingShaft, centerToApex))
	pointB := f.pin("B", drivingShaft.End)

	pinionShaft := f.line("Apex->A", sd.Apex, sd.A)
	f.meet(pinionShaft.Start, apex)
	f.angle(drivingShaft, pinionShaft, signedAngle(sd.DrivingDir, sd.PinionDir))
	pointA := f.pin("A", pinionShaft.End)

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	dropA := f.line("A->Apex2", sd.A, sd.Apex2)
	f.meet(dropA.Start, pointA)
	f.angle(pinionShaft, dropA, signedAngle(sd.PinionDir, unit(sub(sd.Apex2, sd.A))))
	s.AddConstraint(sketch.NewDistance(dropA.Start, dropA.End, c.PPD/2))

	dropB := f.line("B->Apex2", sd.B, sd.Apex2)
	f.meet(dropB.Start, pointB)
	f.angle(drivingShaft, dropB, signedAngle(sd.DrivingDir, unit(sub(sd.Apex2, sd.B))))
	s.AddConstraint(sketch.NewDistance(dropB.Start, dropB.End, c.DPD/2))

	f.meet(dropA.End, dropB.End)
	apex2 := f.pin("Apex2", dropA.End)

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	pitchLine := f.line("Apex->Apex2", sd.Apex, sd.Apex2)
	f.meet(pitchLine.Start, apex)
	f.meet(pitchLine.End, apex2)

	dedD := f.line("Apex2->D", sd.Apex2, sd.D)
	f.meet(dedD.Start, apex2)
	f.angle(pitchLine, dedD, signedAngle(unit(sub(sd.Apex2, sd.Apex)), sd.DHat))
	s.AddConstraint(sketch.NewDistance(dedD.Start, dedD.End, 1.25*p.Module))
	pointD := f.pin("D", dedD.End)

	dedC := f.line("Apex2->C", sd.Apex2, sd.C)
	f.meet(dedC.Start, apex2)
	f.angle(pitchLine, dedC, signedAngle(unit(sub(sd.Apex2, sd.Apex)), sd.CHat))
	s.AddConstraint(sketch.NewDistance(dedC.Start, dedC.End, 1.25*p.Module))
	pointC := f.pin("C", dedC.End)

	proofkit.Step(t, "the two root axes, Apex->C and Apex->D")
	rootC := f.line("Apex->C", sd.Apex, sd.C)
	f.meet(rootC.Start, apex)
	f.meet(rootC.End, pointC)
	rootD := f.line("Apex->D", sd.Apex, sd.D)
	f.meet(rootD.Start, apex)
	f.meet(rootD.End, pointD)

	proofkit.Step(t, "the module-length extensions A->E, C->E and B->F, D->F")
	ae := f.line("A->E", sd.A, sd.E)
	f.meet(ae.Start, pointA)
	s.AddConstraint(sketch.NewParallel(ae, pinionShaft))
	pointE := f.pin("E", ae.End)
	ce := f.line("C->E", sd.C, sd.E)
	f.meet(ce.Start, pointC)
	f.meet(ce.End, pointE)
	s.AddConstraint(sketch.NewPerpendicular(ae, ce))

	bf := f.line("B->F", sd.B, sd.F)
	f.meet(bf.Start, pointB)
	s.AddConstraint(sketch.NewParallel(bf, drivingShaft))
	pointF := f.pin("F", bf.End)
	df := f.line("D->F", sd.D, sd.F)
	f.meet(df.Start, pointD)
	f.meet(df.End, pointF)
	s.AddConstraint(sketch.NewPerpendicular(bf, df))

	proofkit.Step(t, "E->G, C->H, G->H and the pinion base-height offset")
	drivingBase, pinionBase := resolvedBaseHeights(p)
	eg := f.line("E->G", sd.E, sd.G)
	f.meet(eg.Start, pointE)
	s.AddConstraint(sketch.NewParallel(eg, ae))
	pointG := f.pin("G", eg.End)
	ch := f.line("C->H", sd.C, sd.H)
	f.meet(ch.Start, pointC)
	s.AddConstraint(sketch.NewParallel(ch, dedC))
	pointH := f.pin("H", ch.End)
	gh := f.line("G->H", sd.G, sd.H)
	f.meet(gh.Start, pointG)
	f.meet(gh.End, pointH)
	s.AddConstraint(sketch.NewOffset(dropA, gh,
		offsetSign(want.A, want.Apex2, want.G, pinionBase)))

	proofkit.Step(t, "F->I, D->J, I->J and the driving base-height offset")
	fi := f.line("F->I", sd.F, sd.I)
	f.meet(fi.Start, pointF)
	s.AddConstraint(sketch.NewParallel(fi, bf))
	pointI := f.pin("I", fi.End)
	dj := f.line("D->J", sd.D, sd.J)
	f.meet(dj.Start, pointD)
	s.AddConstraint(sketch.NewParallel(dj, dedD))
	pointJ := f.pin("J", dj.End)
	ij := f.line("I->J", sd.I, sd.J)
	f.meet(ij.Start, pointI)
	f.meet(ij.End, pointJ)
	s.AddConstraint(sketch.NewOffset(dropB, ij,
		offsetSign(want.B, want.Apex2, want.I, drivingBase)))

	proofkit.Step(t, "A->G, and point I onto the projected centre")
	ag := f.line("A->G", sd.A, sd.G)
	f.meet(ag.Start, pointA)
	f.meet(ag.End, pointG)

	// The spec closes the figure with addCoincident(I, projected centre). By this
	// point the lattice has exactly one degree of freedom left — it slides along
	// the perpendicular through the centre — and I already sits on that line, so
	// one of the coincident's two equations is redundant. Fusion accepts the
	// redundancy; the engine reports it and the gate fails on it. So the proof
	// adds only the equation the closure actually decides, along the grow
	// direction, and asserts below that the other one already holds. That is what
	// makes the redundancy a measured fact rather than an assumption. The
	// substitution costs the proof nothing else: the solved figure is the same
	// figure, and it is only available in this frame because the bench sketch
	// puts the anchor line on local X, so the grow direction is local Y.
	s.AddConstraint(sketch.NewVerticalDistance(pointI, centre, 0))

	proofkit.Step(t, "solve what is placed, so the Maximum Face Width is read from "+
		"SOLVED geometry rather than from the seeds")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the lattice before the face-width bound: %v", err)
	}
	solvedA, solvedB := f.at("A"), f.at("B")
	solvedC, solvedD := f.at("C"), f.at("D")
	solvedH, solvedJ := f.at("H"), f.at("J")
	pinionReach := math.Abs(cross(sub(solvedA, solvedC), unit(sub(solvedH, solvedC))))
	drivingReach := math.Abs(cross(sub(solvedB, solvedD), unit(sub(solvedJ, solvedD))))
	maxFace := 0.95 * math.Min(pinionReach, drivingReach)
	near(t, "Maximum Face Width from solved geometry", maxFace, want.MaxFace, 1e-6)

	face := p.FaceWidth
	switch {
	case face == 0:
		face = math.Min(coneDistanceVariable(p)/6, maxFace)
	case face > maxFace:
		// The spec rejects this input rather than building. The rejection is
		// exactly the statement that the toe end N would be driven past A, across
		// the gear's own axis of revolution, so the proof measures that first and
		// only then reports the case as one with no sketch to draw.
		if face <= math.Min(pinionReach, drivingReach) {
			t.Errorf("face width %.4f is over the 0.95 bound %.4f but still short of the "+
				"reach %.4f at which N lands on A", face, maxFace, math.Min(pinionReach, drivingReach))
		}
		proofkit.Unmodelled(t, "face width %.4f mm exceeds the Maximum Face Width %.4f mm "+
			"resolved from the solved lattice, so the spec rejects the input and no Gear "+
			"Profiles sketch exists to gate", face, maxFace)
		return
	}
	near(t, "resolved Face Width", face, want.FaceWidth, 1e-9)

	proofkit.Step(t, "K, the tooth-centre point K prime and the C->K prime reference line")
	gk := f.line("G->K", sd.G, sd.K)
	f.meet(gk.Start, pointG)
	f.onLine(gk.End, pinionShaft)
	f.onLine(gk.End, dedC)
	pointK := f.pin("K", gk.End)
	ck := f.line("C->K", sd.C, sd.K)
	f.meet(ck.Start, pointC)
	f.meet(ck.End, pointK)
	if p.ToothSpacing > 0 {
		kk := f.line("K->K'", sd.K, sd.KPrime)
		f.meet(kk.Start, pointK)
		f.onLine(kk.End, dedC)
		f.component(pointK, kk.End, scale(want.CHat, p.ToothSpacing))
		kPrime := f.pin("K'", kk.End)
		ckp := f.line("C->K'", sd.C, sd.KPrime)
		f.meet(ckp.Start, pointC)
		f.meet(ckp.End, kPrime)
	} else {
		// At zero Tooth Spacing K prime IS K and the existing C->K line is the
		// tooth-centre reference line. Drawing a zero-length line here would be
		// degenerate, and a second line over C->K would be a duplicate segment.
		f.pin("K'", pointK)
	}

	proofkit.Step(t, "L, the tooth-centre point L prime and the D->L prime reference line")
	il := f.line("I->L", sd.I, sd.L)
	f.meet(il.Start, pointI)
	f.onLine(il.End, drivingShaft)
	f.onLine(il.End, dedD)
	pointL := f.pin("L", il.End)
	dl := f.line("D->L", sd.D, sd.L)
	f.meet(dl.Start, pointD)
	f.meet(dl.End, pointL)
	if p.ToothSpacing > 0 {
		ll := f.line("L->L'", sd.L, sd.LPrime)
		f.meet(ll.Start, pointL)
		f.onLine(ll.End, dedD)
		f.component(pointL, ll.End, scale(want.DHat, p.ToothSpacing))
		lPrime := f.pin("L'", ll.End)
		dlp := f.line("D->L'", sd.D, sd.LPrime)
		f.meet(dlp.Start, pointD)
		f.meet(dlp.End, lPrime)
	} else {
		f.pin("L'", pointL)
	}

	proofkit.Step(t, "the pinion toe line M->N and its two reference lines")
	mn := f.line("M->N", sd.M, sd.N)
	f.onLine(mn.Start, rootC)
	f.onLine(mn.End, dropA)
	s.AddConstraint(sketch.NewOffset(ch, mn, offsetSign(want.C, want.H, want.M, face)))
	pointM := f.pin("M", mn.Start)
	pointN := f.pin("N", mn.End)
	mc := f.line("M->C", sd.M, sd.C)
	f.meet(mc.Start, pointM)
	f.meet(mc.End, pointC)
	na := f.line("N->A", sd.N, sd.A)
	f.meet(na.Start, pointN)
	f.meet(na.End, pointA)

	proofkit.Step(t, "the driving toe line O->P and its three reference lines")
	op := f.line("O->P", sd.O, sd.P)
	f.onLine(op.Start, rootD)
	f.onLine(op.End, dropB)
	s.AddConstraint(sketch.NewOffset(dj, op, offsetSign(want.D, want.J, want.O, face)))
	pointO := f.pin("O", op.Start)
	pointP := f.pin("P", op.End)
	od := f.line("O->D", sd.O, sd.D)
	f.meet(od.Start, pointO)
	f.meet(od.End, pointD)
	pb := f.line("P->B", sd.P, sd.B)
	f.meet(pb.Start, pointP)
	f.meet(pb.End, pointB)
	bi := f.line("B->I", sd.B, sd.I)
	f.meet(bi.Start, pointB)
	f.meet(bi.End, pointI)

	proofkit.Step(t, "check the solved lattice against the closed-form cone geometry")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the finished lattice: %v", err)
	}
	assertLattice(t, f, p, c, want, face, maxFace)
}

// assertLattice holds the solved sketch to the geometry the spec pins: every
// lattice point, the cone angles the virtual tooth numbers are computed from,
// the redundant half of the I-onto-centre closure, and the face-width bound.
func assertLattice(t testing.TB, f *figure, p params, c cone, want lattice, face, maxFace float64) {
	const tol = 1e-6
	for _, named := range []struct {
		name string
		want vec
	}{
		{"Apex", want.Apex}, {"A", want.A}, {"B", want.B}, {"Apex2", want.Apex2},
		{"C", want.C}, {"D", want.D}, {"E", want.E}, {"F", want.F},
		{"G", want.G}, {"H", want.H}, {"I", want.I}, {"J", want.J},
		{"K", want.K}, {"L", want.L}, {"M", want.M}, {"N", want.N},
		{"O", want.O}, {"P", want.P},
	} {
		nearPoint(t, "point "+named.name, f.at(named.name), named.want, tol)
	}
	if p.ToothSpacing > 0 {
		nearPoint(t, "point K'", f.at("K'"), want.KPrime, tol)
		nearPoint(t, "point L'", f.at("L'"), want.LPrime, tol)
	}

	apex, a, b := f.at("Apex"), f.at("A"), f.at("B")
	apex2 := f.at("Apex2")

	// The closed-form cone geometry, measured off the sketch rather than restated.
	near(t, "cone distance |Apex->Apex2|", norm(sub(apex2, apex)), c.R, tol)
	near(t, "pitch cone half angle gamma_p",
		math.Abs(signedAngle(sub(a, apex), sub(apex2, apex))), c.GammaP*180/math.Pi, 1e-9)
	near(t, "pitch cone half angle gamma_g",
		math.Abs(signedAngle(sub(b, apex), sub(apex2, apex))), c.GammaG*180/math.Pi, 1e-9)
	near(t, "|Apex->A|", norm(sub(a, apex)), c.AlongA, tol)
	near(t, "|Apex->B|", norm(sub(b, apex)), c.AlongB, tol)
	near(t, "perpendicular distance from Apex 2 to the pinion shaft",
		math.Abs(cross(sub(apex2, apex), unit(sub(a, apex)))), c.PPD/2, tol)
	near(t, "perpendicular distance from Apex 2 to the driving shaft",
		math.Abs(cross(sub(apex2, apex), unit(sub(b, apex)))), c.DPD/2, tol)

	// Point A is the +X-most of the two shaft-angle senses, which is the rule the
	// spec gives for choosing which side the pinion lies on.
	mirror := add(apex, scale(rot(unit(sub(b, apex)), -c.Sigma), c.AlongA))
	if a.X <= mirror.X {
		t.Errorf("point A is at x=%.4f, but the rejected shaft-angle sense puts it at x=%.4f; "+
			"the spec keeps the greater-x candidate", a.X, mirror.X)
	}

	// The half of the I-onto-centre coincidence the proof did not add. It holds
	// anyway, which is the measurement behind the redundancy note above.
	near(t, "the component of I->centre the closure does not constrain",
		f.at("I").X, f.at("centre").X, tol)

	// The virtual tooth numbers §3 draws its teeth with come from the closed form,
	// and the lattice agrees: Apex2->K is the pinion's virtual pitch radius.
	if p.ToothSpacing == 0 {
		near(t, "|Apex2->K| against the pinion virtual pitch radius",
			norm(sub(f.at("K"), apex2)), (c.PPD/2)/math.Cos(c.GammaP), tol)
		near(t, "|Apex2->L| against the driving virtual pitch radius",
			norm(sub(f.at("L"), apex2)), (c.DPD/2)/math.Cos(c.GammaG), tol)
	}

	// The heel edge C->H must run OUTWARD from the dedendum corner: H lies further
	// along the shaft than C. It does so only while the resolved base height
	// exceeds the dedendum's projection onto the shaft, 1.25 * module * sin(gamma).
	// Below that the heel edge reverses, the profile hexagon folds over, and the
	// body built from it is not a frustum. The input validation admits three teeth,
	// which is well inside the folded region for the default base heights.
	pinionDirSolved := unit(sub(a, apex))
	drivingDirSolved := unit(sub(b, apex))
	if reach := dot(sub(f.at("H"), f.at("C")), pinionDirSolved); reach <= 0 {
		t.Errorf("the pinion heel edge C->H runs inward: H is %.4f mm along the shaft "+
			"BEHIND C, so the resolved base height does not clear the dedendum's "+
			"projection %.4f mm", -reach, 1.25*p.Module*math.Sin(c.GammaP))
	}
	if reach := dot(sub(f.at("J"), f.at("D")), drivingDirSolved); reach <= 0 {
		t.Errorf("the driving heel edge D->J runs inward: J is %.4f mm along the shaft "+
			"BEHIND D, so the resolved base height does not clear the dedendum's "+
			"projection %.4f mm", -reach, 1.25*p.Module*math.Sin(c.GammaG))
	}

	// The face width is the perpendicular gap between heel and toe, the toe is the
	// end nearer the apex, and N has not been driven past A.
	cc, h, m, n := f.at("C"), f.at("H"), f.at("M"), f.at("N")
	near(t, "offset between C->H and M->N", math.Abs(cross(sub(m, cc), unit(sub(h, cc)))), face, tol)
	if norm(sub(m, apex)) >= norm(sub(cc, apex)) {
		t.Errorf("the toe corner M is %.4f from the apex and the heel corner C is %.4f; "+
			"the toe must be the inner end", norm(sub(m, apex)), norm(sub(cc, apex)))
	}
	if got, limit := norm(sub(n, apex2)), norm(sub(a, apex2)); got > limit {
		t.Errorf("N is %.4f from Apex 2 and A is %.4f; the capped face width must keep N "+
			"between Apex 2 and A", got, limit)
	}
	// The bound's meaning: at the smaller of the two reaches the toe end lands
	// exactly on its shaft corner. Recomputing the toe from the SOLVED lattice at
	// that reach is what shows the cap is the value where N (or P) reaches A (or
	// B), rather than a number restated from the spec.
	pinionReach := math.Abs(cross(sub(a, cc), unit(sub(h, cc))))
	d, j := f.at("D"), f.at("J")
	drivingReach := math.Abs(cross(sub(b, d), unit(sub(j, d))))
	near(t, "Maximum Face Width against the smaller reach",
		maxFace, 0.95*math.Min(pinionReach, drivingReach), tol)
	_, nAtCap := toeEdge(cc, unit(sub(h, cc)), apex, a, apex2, pinionReach)
	nearPoint(t, "N at the pinion reach", nAtCap, a, 1e-6)
	_, pAtCap := toeEdge(d, unit(sub(j, d)), apex, b, apex2, drivingReach)
	nearPoint(t, "P at the driving reach", pAtCap, b, 1e-6)
}

// gearProfilesCases is the §2 regime: the default pair, both ratio directions so
// either gear can be the binding side of the face-width bound, both ends of the
// shaft-angle range, both sides of the Tooth Spacing branch, both sides of each
// base-height fallback, and both sides of the face-width branch including the
// value the spec rejects.
var gearProfilesCases = []proofkit.Case{
	{Name: "default_31_31_90deg", Params: map[string]float64{}},
	{Name: "ratio_driving_31_pinion_17", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17}},
	{Name: "ratio_driving_17_pinion_31", Params: map[string]float64{
		"drivingTeeth": 17, "pinionTeeth": 31}},
	{Name: "module_2_driving_19_pinion_13", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13}},
	{Name: "shaft_angle_30_ratio", Params: map[string]float64{
		"shaftAngle": 30, "drivingTeeth": 31, "pinionTeeth": 17}},
	{Name: "shaft_angle_135_equal", Params: map[string]float64{"shaftAngle": 135}},
	{Name: "low_tooth_count_8_8", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8}},
	{Name: "tooth_spacing_0p4", Params: map[string]float64{"toothSpacing": 0.4}},
	{Name: "base_heights_given", Params: map[string]float64{
		"drivingBaseHeight": 6, "pinionBaseHeight": 2.5}},
	{Name: "face_width_given_below_cap", Params: map[string]float64{"faceWidth": 4}},
	{Name: "face_width_over_the_cap", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "faceWidth": 7.31}},
	{Name: "centre_off_the_sketch_origin", Params: map[string]float64{
		"centerX": 12.5, "centerY": -8}},
}
