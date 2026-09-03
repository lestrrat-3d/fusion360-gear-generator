package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// anchorCases prove the §1 Anchor Sketch. Its only branch is the side the
// figure grows on, which the target plane's normal decides
// ([BEVEL-F-GROW-SIDE]); the sketch itself carries no gear parameter.
var anchorCases = []proofkit.Case{
	{Name: "grow-toward-normal", Params: map[string]float64{"growSign": 1}},
	{Name: "grow-against-normal", Params: map[string]float64{"growSign": -1}},
}

// latticeCases prove the §2 Gear Profiles sketch and the per-gear Profile
// sketch across the regime the spec states: the Shaft Angle from its 30 degree
// floor to the 150 degree ceiling, both ratio directions, the computed Minimum
// Teeth floor, a non-zero Tooth Spacing, and both grow sides.
var latticeCases = []proofkit.Case{
	{Name: "default-31-31-90", Params: map[string]float64{}},
	{Name: "shaft-30", Params: map[string]float64{"shaftAngleDeg": 30}},
	{Name: "shaft-150", Params: map[string]float64{"shaftAngleDeg": 150}},
	{Name: "ratio-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17}},
	{Name: "ratio-17-31", Params: map[string]float64{"drivingTeeth": 17, "pinionTeeth": 31}},
	{Name: "teeth-floor-4", Params: map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4}},
	{Name: "spacing-and-mirror", Params: map[string]float64{"toothSpacing": 0.4, "growSign": -1, "module": 2,
		"drivingTeeth": 19, "pinionTeeth": 13}},
}

// conditioningFloor is the sketch engine's own trust floor at the default
// tolerance, max(1e-6, 4*sqrt(tolerance)). A lattice below it is refused as
// near-singular, and the spec forbids answering that by loosening the gate.
const conditioningFloor = 4e-5

// stepAnchorSketch draws §1: the Anchor Sketch on the user's target plane, the
// projected centre point, and the Anchor Line through it.
//
// Fusion's addCoincident(projectedCenter, anchorLine) is a point-on-curve row.
// The bench's Midpoint already carries that row, so adding both here leaves the
// sketch redundant at DOF 0 — measured, with the engine naming the pair. The
// proof therefore keeps Midpoint alone and records the difference: this is the
// same arity mismatch the spec documents for the G->H perpendicular, and only a
// Fusion session can say whether Fusion absorbs the extra row (it does, on the
// working builds) or reports it.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user centre point into the Anchor Sketch")
	centre := s.CreateReferencePoint(0, 0, "user centre point")
	centre.SetName("projected centre")

	proofkit.Step(t, "draw the Anchor Line seeded at +-0.5 cm about the projected centre")
	grow := param(p, "growSign", 1)
	start := s.CreatePoint(-5, 0)
	start.SetName("anchor line start")
	end := s.CreatePoint(5, 0)
	end.SetName("anchor line end")
	anchor := s.CreateLine(start, end)
	anchor.SetName("Anchor Line")

	s.AddConstraint(sketch.NewMidpoint(centre, anchor))
	// Fusion's aligned distance dimension is a magnitude whose direction is
	// captured from the seed ([PB-DIM-VALUE-SEMANTICS]); the bench's target is
	// signed, so the seed side crosses over as the sign here. Written as a plain
	// unsigned length the line still solves end-for-end reversed, and the
	// engine's probe reports the two configurations.
	s.AddConstraint(sketch.NewHorizontalDistance(start, end, 10))
	s.AddConstraint(sketch.NewHorizontal(anchor))

	solveHere(t, s)
	near(t, anchor.Length(), 10, 1e-9, "Anchor Line length")
	near(t, centre.X(), 0.5*(start.X()+end.X()), 1e-9, "projected centre bisects the Anchor Line in x")
	near(t, centre.Y(), 0.5*(start.Y()+end.Y()), 1e-9, "projected centre bisects the Anchor Line in y")
	near(t, start.Y()-end.Y(), 0, 1e-9, "Anchor Line is sketch-local horizontal")
	// The grow side is a one-bit direction read off the target plane's normal;
	// it never reaches the Anchor Line, whose absolute direction is arbitrary.
	near(t, math.Abs(grow), 1, 1e-12, "grow side is a unit sign")
}

// latticeNet is the §2 figure as built entities, so the assertions can read
// solved positions rather than the seeds ([PB-SOLVED-GEOMETRY]).
type latticeNet struct {
	centre *sketch.Point
	anchor *sketch.Line
	apex   *sketch.Point
	apex2  *sketch.Point
	pitch  *sketch.Line
	sides  map[string]*latticeSide
	config config
}

type latticeSide struct {
	name    string
	axis    *sketch.Line // Apex->A / Apex->B
	axisEnd *sketch.Point
	drop    *sketch.Line // A->Apex2 / B->Apex2
	ded     *sketch.Line // Apex2->C / Apex2->D
	dedEnd  *sketch.Point
	root    *sketch.Line // Apex->C / Apex->D
	lineCH  *sketch.Line
	lineMN  *sketch.Line
	e, g, h *sketch.Point
	k, kp   *sketch.Point
	m, n    *sketch.Point
}

// stepGearProfiles draws §2: the whole Gear Profiles lattice for both gears in
// one sketch, in the sketch's own 2-D frame ([BEVEL-F-APEX-LOCAL]).
//
// Three places the bench and Fusion do not have the same constraint arity, all
// of the kind the spec already documents for the G->H perpendicular:
//
//   - the G->H and I->J perpendiculars are omitted, because the bench's Offset
//     holds both endpoints of the target line and so carries the parallelism
//     itself (the spec states this one and requires the omission);
//   - addParallel(M->N, C->H) and addParallel(O->P, D->J) are omitted for the
//     same reason — measured here: with them the engine reports the two toe
//     offsets as redundant at DOF 0;
//   - addCoincident(I, projected centre) is replaced by a single point-on-line
//     row (I on the Anchor Line). The driving shaft axis is parallel to the
//     centre->apex line through the centre and passes through the apex, so it
//     IS that line: one of the coincidence's two rows is already implied, and
//     the engine names the second as redundant.
//
// Each of the three keeps the row count the freedoms need. A Fusion build makes
// the calls the step list names; only a Fusion session can say whether Fusion
// absorbs the extra rows.
//
// The perpendicular drops to Apex 2 are written as SIGNED 90 degree angles
// rather than perpendicular constraints. The spec picks their sense by a seed
// (the dot product with the A->B direction) and warns that the wrong sense
// flips the whole frame to the mirror solution. Measured with plain
// perpendicular constraints, the engine's ambiguity probe finds exactly that
// second configuration for unequal tooth counts (31/17 and an equal 4/4 pair),
// so the seed is the only thing separating them and the gate refuses it. A
// signed angle is one row, like the perpendicular it replaces, and carries the
// direction the seed carries.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	net := buildLatticeNet(t, s, p)
	c := net.config

	// The engine refuses a near-singular system, and the spec forbids answering
	// that by loosening the gate or by narrowing the advertised Shaft Angle
	// range on one net's evidence. Measure this net and record where it stands.
	rep := s.Verify(context.Background())
	if rep.DOF == 0 && rep.Conditioning < conditioningFloor {
		proofkit.Unmodelled(t, "this lattice's conditioning at Shaft Angle %.0f deg is %.3e, below the engine's %.0e trust floor; "+
			"the spec documents that independently written lattices disagree about which end of the range is reachable and leaves the reachable floor to the net",
			c.SigmaDeg, rep.Conditioning, conditioningFloor)
	}

	assertLattice(t, net)
}

// buildLatticeNet builds the §2 figure and solves it. Every line is a
// construction line, and every line is built from raw coordinates with one
// coincident per connected end ([BEVEL-F-COINCIDENT-STYLE]); each named line is
// created once and reused ([BEVEL-F-LINE-ONCE]); the driven lengths (Apex->A,
// Apex->B and the module-length extensions) carry no dimension
// ([BEVEL-F-DRIVEN-DIMS]).
func buildLatticeNet(t testing.TB, s *sketch.Sketch, p map[string]float64) *latticeNet {
	t.Helper()
	c := mustResolve(t, p)

	line := func(a, b vec2, name string) (*sketch.Line, *sketch.Point, *sketch.Point) {
		ap := s.CreatePoint(a.X, a.Y)
		ap.SetName(name + " start")
		bp := s.CreatePoint(b.X, b.Y)
		bp.SetName(name + " end")
		l := s.CreateLine(ap, bp)
		l.SetName(name)
		l.SetConstruction(true)
		return l, ap, bp
	}

	proofkit.Step(t, "project the Anchor Sketch centre point and the Anchor Line into Gear Profiles")
	centre := s.CreateReferencePoint(c.Center.X, c.Center.Y, "Anchor Sketch centre point")
	centre.SetName("projected centre")
	a1 := s.CreateReferencePoint(-5, 0, "Anchor Line start")
	a2 := s.CreateReferencePoint(5, 0, "Anchor Line end")
	anchor, err := s.CreateReferenceLine(a1, a2, "Anchor Line")
	if err != nil {
		t.Fatalf("project the Anchor Line: %v", err)
	}
	anchor.SetName("projected Anchor Line")

	proofkit.Step(t, "centre->apex construction line, perpendicular to the projected Anchor Line")
	centreToApex, ctaStart, apex := line(c.Center, c.Apex, "centre->apex")
	s.AddConstraint(sketch.NewCoincident(ctaStart, centre), sketch.NewPerpendicular(centreToApex, anchor))

	proofkit.Step(t, "the two shaft axes: Apex->B parallel to centre->apex, Apex->A at the Shaft Angle")
	drivingAxis, dStart, bPoint := line(c.Apex, c.Driving.Axis, "Apex->B")
	s.AddConstraint(sketch.NewCoincident(dStart, apex), sketch.NewParallel(drivingAxis, centreToApex))
	pinionAxis, pStart, aPoint := line(c.Apex, c.Pinion.Axis, "Apex->A")
	s.AddConstraint(sketch.NewCoincident(pStart, apex))
	sigma := math.Atan2(v2cross(c.Driving.AxisDir, c.Pinion.AxisDir), v2dot(c.Driving.AxisDir, c.Pinion.AxisDir))
	s.AddConstraint(sketch.NewAngle(drivingAxis, pinionAxis, sigma*180/math.Pi))

	proofkit.Step(t, "the two perpendicular drops close at Apex 2")
	drop := func(g gearSide, axis *sketch.Line, from *sketch.Point, name string) (*sketch.Line, *sketch.Point) {
		l, st, en := line(g.Axis, c.Apex2, name)
		dir := v2unit(v2sub(c.Apex2, g.Axis))
		signed := math.Atan2(v2cross(g.AxisDir, dir), v2dot(g.AxisDir, dir))
		s.AddConstraint(sketch.NewCoincident(st, from), sketch.NewAngle(axis, l, signed*180/math.Pi))
		s.AddConstraint(sketch.NewDistance(st, en, g.PitchDia/2))
		return l, en
	}
	dropP, apex2P := drop(c.Pinion, pinionAxis, aPoint, "A->Apex2")
	dropG, apex2G := drop(c.Driving, drivingAxis, bPoint, "B->Apex2")
	s.AddConstraint(sketch.NewCoincident(apex2P, apex2G))

	proofkit.Step(t, "Pitch Line and the two dedendum lines")
	pitch, pl0, pl1 := line(c.Apex, c.Apex2, "Pitch Line")
	s.AddConstraint(sketch.NewCoincident(pl0, apex), sketch.NewCoincident(pl1, apex2P))

	net := &latticeNet{centre: centre, anchor: anchor, apex: apex, apex2: apex2P, pitch: pitch,
		sides: map[string]*latticeSide{}, config: c}

	for _, g := range []gearSide{c.Pinion, c.Driving} {
		axis, axisEnd, dropLine := pinionAxis, aPoint, dropP
		if g.Label == "Driving" {
			axis, axisEnd, dropLine = drivingAxis, bPoint, dropG
		}
		ded, dedStart, dedEnd := line(c.Apex2, g.Ded, g.Label+" Dedendum")
		dedAngle := math.Atan2(v2cross(v2unit(v2sub(c.Apex2, c.Apex)), g.DedDir), v2dot(v2unit(v2sub(c.Apex2, c.Apex)), g.DedDir))
		s.AddConstraint(sketch.NewCoincident(dedStart, apex2P), sketch.NewAngle(pitch, ded, dedAngle*180/math.Pi))
		s.AddConstraint(sketch.NewDistance(dedStart, dedEnd, 1.25*c.Module))

		root, rootStart, rootEnd := line(c.Apex, g.Ded, g.Label+" Root Axis")
		s.AddConstraint(sketch.NewCoincident(rootStart, apex), sketch.NewCoincident(rootEnd, dedEnd))

		side := &latticeSide{name: g.Label, axis: axis, axisEnd: axisEnd, drop: dropLine,
			ded: ded, dedEnd: dedEnd, root: root}
		net.sides[g.Label] = side

		proofkit.Step(t, "%s: the module-length extensions A->E, E->G and the dedendum walk C->H", g.Label)
		lineAE, aeStart, ePoint := line(g.Axis, g.E, g.Label+" A->E")
		s.AddConstraint(sketch.NewCoincident(aeStart, axisEnd), sketch.NewPointOnLine(ePoint, axis))
		lineCE, ceStart, ceEnd := line(g.Ded, g.E, g.Label+" C->E")
		s.AddConstraint(sketch.NewCoincident(ceStart, dedEnd), sketch.NewCoincident(ceEnd, ePoint),
			sketch.NewPerpendicular(lineCE, lineAE))
		_, egStart, gPoint := line(g.E, g.G, g.Label+" E->G")
		s.AddConstraint(sketch.NewCoincident(egStart, ePoint), sketch.NewPointOnLine(gPoint, lineAE))
		lineCH, chStart, hPoint := line(g.Ded, g.H, g.Label+" C->H")
		s.AddConstraint(sketch.NewCoincident(chStart, dedEnd), sketch.NewPointOnLine(hPoint, ded))
		lineGH, ghStart, ghEnd := line(g.G, g.H, g.Label+" G->H")
		s.AddConstraint(sketch.NewCoincident(ghStart, gPoint), sketch.NewCoincident(ghEnd, hPoint))
		s.AddConstraint(sketch.NewOffset(dropLine, lineGH, signedOffsetOf(g.Axis, c.Apex2, g.G)))

		_, agStart, agEnd := line(g.Axis, g.G, g.Label+" A->G")
		s.AddConstraint(sketch.NewCoincident(agStart, axisEnd), sketch.NewCoincident(agEnd, gPoint))

		proofkit.Step(t, "%s: the tooth centre K on the shaft axis and the dedendum line", g.Label)
		_, gkStart, kPoint := line(g.G, g.K, g.Label+" G->K")
		s.AddConstraint(sketch.NewCoincident(gkStart, gPoint),
			sketch.NewPointOnLine(kPoint, axis), sketch.NewPointOnLine(kPoint, ded))
		centreEnd := kPoint
		if c.ToothSpacing > 0 {
			// K' is K shifted outward along the dedendum line, away from C.
			// The spacing runs AWAY from C, which the spec fixes by the seed.
			// A point-on-line row leaves both sides of K solvable and the
			// engine's probe reports the two; a signed zero angle against the
			// dedendum line is the same one row and carries the direction.
			lineKK, kkStart, kpPoint := line(g.K, g.KPrime, g.Label+" K->K'")
			s.AddConstraint(sketch.NewCoincident(kkStart, kPoint), sketch.NewAngle(ded, lineKK, 0))
			s.AddConstraint(sketch.NewDistance(kkStart, kpPoint, c.ToothSpacing))
			centreEnd = kpPoint
		}
		_, ckStart, ckEnd := line(g.Ded, g.KPrime, g.Label+" C->K'")
		s.AddConstraint(sketch.NewCoincident(ckStart, dedEnd), sketch.NewCoincident(ckEnd, centreEnd))

		proofkit.Step(t, "%s: the toe line M->N, offset from C->H by the resolved Face Width", g.Label)
		lineMN, mPoint, nPoint := line(g.M, g.N, g.Label+" M->N")
		s.AddConstraint(sketch.NewPointOnLine(mPoint, root), sketch.NewPointOnLine(nPoint, dropLine))
		s.AddConstraint(sketch.NewOffset(lineCH, lineMN, signedOffsetOf(g.Ded, g.H, g.M)))
		_, mcStart, mcEnd := line(g.M, g.Ded, g.Label+" M->C")
		s.AddConstraint(sketch.NewCoincident(mcStart, mPoint), sketch.NewCoincident(mcEnd, dedEnd))
		_, naStart, naEnd := line(g.N, g.Axis, g.Label+" N->A")
		s.AddConstraint(sketch.NewCoincident(naStart, nPoint), sketch.NewCoincident(naEnd, axisEnd))

		side.lineCH, side.lineMN = lineCH, lineMN
		side.e, side.g, side.h = ePoint, gPoint, hPoint
		side.k, side.kp, side.m, side.n = kPoint, centreEnd, mPoint, nPoint

		if g.Label == "Driving" {
			// "Constrain Point I with center point" — I is on the driving shaft
			// axis, which is the line through the projected centre, so pinning it
			// to the Anchor Line lands it on the centre with one row.
			s.AddConstraint(sketch.NewPointOnLine(gPoint, anchor))
		}
	}

	solveHere(t, s)
	return net
}

// signedOffsetOf is the signed perpendicular distance of dst from the infinite
// line src->through, positive to the left of the src->through direction. The
// bench's Offset takes a signed target where Fusion's addOffsetDimension takes a
// magnitude whose side comes from the seeded geometry ([PB-DIM-VALUE-SEMANTICS]).
func signedOffsetOf(src, through, dst vec2) float64 {
	return v2cross(v2unit(v2sub(through, src)), v2sub(dst, src))
}

func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e DOF %d", res.Residual, res.DOF)
	}
}

func at(p *sketch.Point) vec2 { return vec2{p.X(), p.Y()} }

// assertLattice checks the solved figure against the closed form the spec
// states, from .geometry rather than from the seeds ([PB-SOLVED-GEOMETRY]).
func assertLattice(t testing.TB, net *latticeNet) {
	t.Helper()
	c := net.config
	apex := at(net.apex)
	apex2 := at(net.apex2)

	// The closure at Apex 2 drives the two undimensioned along-shaft lengths.
	near(t, v2len(v2sub(at(net.sides["Pinion"].axisEnd), apex)), c.R*math.Cos(c.Pinion.Gamma), 1e-9,
		"|Apex->A| is the driven R*cos(gamma_p)")
	near(t, v2len(v2sub(at(net.sides["Driving"].axisEnd), apex)), c.R*math.Cos(c.Driving.Gamma), 1e-9,
		"|Apex->B| is the driven R*cos(gamma_g)")

	// Point I closes the figure onto the projected centre; that is what fixes
	// the apex's undimensioned height above the Anchor Line.
	near(t, v2len(v2sub(at(net.sides["Driving"].g), at(net.centre))), 0, 1e-9, "point I lands on the projected centre")
	near(t, v2len(v2sub(apex, at(net.centre))), c.R*math.Cos(c.Driving.Gamma)+c.Driving.BaseHeight, 1e-9,
		"the apex sits R*cos(gamma_g) plus the resolved Driving Base Height above the centre")

	for _, g := range []gearSide{c.Pinion, c.Driving} {
		side := net.sides[g.Label]
		axisDir := v2unit(v2sub(at(side.axisEnd), apex))
		pitchDir := v2unit(v2sub(apex2, apex))
		gamma := math.Acos(math.Min(1, v2dot(axisDir, pitchDir)))
		near(t, gamma, g.Gamma, 1e-9, "%s solved pitch cone angle", g.Label)

		// The drop is the gear's pitch radius, perpendicular to its own shaft.
		near(t, v2len(v2sub(apex2, at(side.axisEnd))), g.PitchDia/2, 1e-9, "%s drop to Apex 2 is its pitch radius", g.Label)
		near(t, v2dot(v2unit(v2sub(apex2, at(side.axisEnd))), axisDir), 0, 1e-9, "%s drop is perpendicular to its shaft", g.Label)

		// Apex 2 lies in the interior wedge between the two shafts: the drop
		// points at the other gear's shaft end.
		other := c.Driving
		if g.Label == "Driving" {
			other = c.Pinion
		}
		otherEnd := at(net.sides[other.Label].axisEnd)
		if v2dot(v2sub(apex2, at(side.axisEnd)), v2sub(otherEnd, at(side.axisEnd))) <= 0 {
			t.Errorf("%s: the drop to Apex 2 points away from the other shaft — the frame has mirrored", g.Label)
		}

		ded := at(side.dedEnd)
		near(t, v2len(v2sub(ded, apex2)), 1.25*c.Module, 1e-9, "%s dedendum length", g.Label)
		near(t, v2distToLine(ded, apex, axisDir), g.PitchDia/2-1.25*c.Module*math.Cos(g.Gamma), 1e-9,
			"%s dedendum corner sits one dedendum inside its pitch radius", g.Label)

		// The heel edge: H is one base height along the shaft beyond the drop,
		// which is the Minimum/Maximum Base Height window made geometric.
		h := at(side.h)
		near(t, v2dot(v2sub(h, apex2), axisDir), g.BaseHeight, 1e-9, "%s base-height offset from the A->Apex2 drop", g.Label)
		if v2dot(v2sub(h, ded), v2sub(ded, apex2)) <= 0 {
			t.Errorf("%s: H sits behind C, so the heel edge C->H runs back inward (base height below the Minimum)", g.Label)
		}
		if v2distToLine(h, apex, axisDir) <= 0 {
			t.Errorf("%s: H has reached the shaft axis (base height at or past r*tan(gamma))", g.Label)
		}
		near(t, v2distToLine(at(side.g), apex, axisDir), 0, 1e-9, "%s point G lies on its shaft axis", g.Label)
		near(t, v2distToLine(at(side.e), apex, axisDir), 0, 1e-9, "%s point E lies on its shaft axis", g.Label)

		// K is where the dedendum line crosses the shaft axis; K' is K shifted
		// out by the Tooth Spacing, and the tooth size does not move with it.
		k := at(side.k)
		near(t, v2distToLine(k, apex, axisDir), 0, 1e-9, "%s tooth centre K lies on the shaft axis", g.Label)
		near(t, v2distToLine(k, apex2, g.DedDir), 0, 1e-9, "%s tooth centre K lies on the dedendum line", g.Label)
		near(t, v2len(v2sub(at(side.kp), k)), c.ToothSpacing, 1e-9, "%s K' sits one Tooth Spacing beyond K", g.Label)
		if c.ToothSpacing > 0 && v2dot(v2sub(at(side.kp), k), v2sub(k, ded)) <= 0 {
			t.Errorf("%s: K' moved toward C instead of away from it", g.Label)
		}
		near(t, g.VirtualTeeth, math.Floor(2*(g.PitchDia/2/math.Cos(g.Gamma))/c.Module), 1e-12,
			"%s virtual tooth number is independent of the Tooth Spacing", g.Label)

		// The toe edge and the Maximum Face Width: N is pinned to the A->Apex2
		// drop, so the cap is exactly the offset at which N would reach A.
		m, n := at(side.m), at(side.n)
		near(t, v2distToLine(m, apex, v2unit(v2sub(ded, apex))), 0, 1e-9, "%s point M lies on the root axis", g.Label)
		near(t, v2distToLine(n, at(side.axisEnd), v2unit(v2sub(apex2, at(side.axisEnd)))), 0, 1e-9,
			"%s point N lies on the A->Apex2 drop", g.Label)
		near(t, v2distToLine(m, ded, g.DedDir), c.FaceWidth, 1e-9, "%s toe line is one Face Width off C->H", g.Label)
		if c.FaceWidth > c.MaxFaceWidth+1e-12 {
			t.Errorf("%s: the resolved Face Width %.4f exceeds the Maximum Face Width %.4f", g.Label, c.FaceWidth, c.MaxFaceWidth)
		}
		if v2dot(v2sub(n, apex2), v2sub(at(side.axisEnd), apex2)) < 0 {
			t.Errorf("%s: point N fell outside the segment Apex2->A, so the toe has crossed the shaft axis", g.Label)
		}
		rootDir := v2unit(v2sub(ded, apex))
		if v2dot(v2sub(m, apex), rootDir) >= v2dot(v2sub(ded, apex), rootDir) {
			t.Errorf("%s: the toe is not nearer the apex than the heel", g.Label)
		}

		// [PB-REVOLVE]: the hexagon that gets revolved must stay on one side of
		// its own shaft axis. A and G sit on it; the other four must not cross.
		for _, v := range []struct {
			name string
			p    vec2
		}{{"H", h}, {"C", ded}, {"M", m}, {"N", n}} {
			if v2cross(axisDir, v2sub(v.p, apex))*v2cross(axisDir, v2sub(ded, apex)) < 0 {
				t.Errorf("%s: hexagon vertex %s is on the far side of the shaft axis; the revolve would fail with ASM_WIRE_X_AXIS", g.Label, v.name)
			}
		}
	}
}
