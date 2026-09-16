package bevelgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// §3a — the spiral tooth body, the psi > 0 branch of the tooth-body hook.
//
// When psi = 0 the hook returns immediately with the framework's
// cut_conical_ends and the tooth is the straight one, byte-for-byte the prior
// behaviour. Everything here runs only when psi > 0, and the psi = 0 cases in
// these tables are what hold that gate: they assert the straight-bevel limit
// rather than skipping, because the arc construction has a limit at psi = 0 and
// it is one of spiral-tooth-trace.md's own invariants.

// bgSwapKey is the proof's own case knob: it hands the frame builder its toe and
// heel the wrong way round, so the swap guard of §3a step A is exercised rather
// than assumed. It is not a dialog input.
const bgSwapKey = "handToeAndHeelSwapped"

// bgFrame is the §3a world frame for one gear, in the shaft frame this package
// works in: the shaft axis is +Z, the apex is the origin, and the axial (Gear
// Profiles) plane is XZ.
type bgFrame struct {
	AxisDir  r3.Vec // the shaft axis, from the profile edge A'->G / B'->I
	ConeVec  r3.Vec // the dedendum (root) cone element Apex->C / Apex->D
	V        r3.Vec // axisDir x coneVec, the circumferential direction
	TPNormal r3.Vec // coneVec x V, the tangent-plane normal
	Apex     r3.Vec

	ToeMid, HeelMid             r3.Vec
	ToeConeWorld, HeelConeWorld r3.Vec
	Swapped                     bool

	RToe, RHeel, RMean, Span float64
	RC, HandSign             float64
	CX, CY                   float64
	RLo, RHi                 float64
	Toe2D, Heel2D            bgPt
	PhiCrown, Total          float64
}

func (f bgFrame) DistAlong(p r3.Vec) float64 { return p.Sub(f.Apex).Dot(f.ConeVec) }

// bgBuildFrame realizes §3a step A on the four world points _createGearBody
// hands the hook, in the order toeMid, heelMid, toeConeWorld, heelConeWorld.
//
// PIN THE HAND-OFF EXACTLY. The toe edge is M->N (pinion) / O->P (driving) and
// the heel edge is C->H / D->J; toeMid and heelMid are those two edges'
// MIDPOINTS and toeConeWorld / heelConeWorld are M/O and C/D. Passing the two
// endpoints of ONE edge as toeMid/heelMid collapses the span to about zero and
// inverts the spiral silently; taking H/J for heelConeWorld skews coneVec off
// the root cone element, because H/J sit on the Apex2->C / Apex2->D dedendum
// line, one Module beyond C/D and off that element.
func bgBuildFrame(l bgLattice, g bgMember, toeMid, heelMid, toeCone, heelCone r3.Vec) bgFrame {
	f := bgFrame{
		Apex:    r3.NewVec(0, 0, 0),
		AxisDir: r3.NewVec(0, 0, 1),
		ToeMid:  toeMid, HeelMid: heelMid,
		ToeConeWorld: toeCone, HeelConeWorld: heelCone,
	}
	// The heel MUST be the outer end, so coneVec points outward and span > 0. A
	// negative span silently inverts the whole spiral frame — the cutter-arc
	// direction, the slice direction and the per-segment twist all flip — and
	// the gear comes out completely wrong with no error raised.
	if f.Apex.Sub(heelMid).Len() < f.Apex.Sub(toeMid).Len() {
		f.ToeMid, f.HeelMid = heelMid, toeMid
		f.ToeConeWorld, f.HeelConeWorld = heelCone, toeCone
		f.Swapped = true
	}
	f.ConeVec, _ = f.HeelConeWorld.Sub(f.Apex).Normalize()
	f.V, _ = f.AxisDir.Cross(f.ConeVec).Normalize()
	f.TPNormal, _ = f.ConeVec.Cross(f.V).Normalize()

	f.RToe = f.DistAlong(f.ToeMid)
	f.RHeel = f.DistAlong(f.HeelMid)
	f.RMean = (f.RToe + f.RHeel) / 2
	f.Span = f.RHeel - f.RToe

	f.RC = l.In.CutterRadius
	if f.RC == 0 {
		f.RC = f.RMean // 0 means auto
	}
	f.HandSign = l.In.Hand
	if g.Label == "Pinion" {
		f.HandSign = -f.HandSign // the pair meshes with opposite hands
	}
	psi := l.In.SpiralAngle
	// ⚠ The hand sign goes on the cos / Cy term, NOT the sin / Cx term.
	// Opposite hand mirrors the cutter centre ACROSS THE CONE ELEMENT (y = 0),
	// which flips Cy. Putting handSign on Cx mirrors about x = R_mean instead —
	// a different curve that gives the two gears unequal twist.
	f.CX = f.RMean - f.RC*math.Sin(psi)
	f.CY = f.HandSign * f.RC * math.Cos(psi)

	f.RLo = f.RToe - bgTraceOvershoot*f.Span
	f.RHi = f.RHeel + bgTraceOvershoot*f.Span
	f.Toe2D = bgCircleIntersectNearest(f.RLo, f.CX, f.CY, f.RC, f.RMean, 0)
	f.Heel2D = bgCircleIntersectNearest(f.RHi, f.CX, f.CY, f.RC, f.RMean, 0)

	f.PhiCrown = math.Atan2(f.Heel2D.Y, f.Heel2D.X) - math.Atan2(f.Toe2D.Y, f.Toe2D.X)
	f.Total = math.Abs(f.PhiCrown) / math.Sin(g.Gamma)
	return f
}

// bgCircleIntersectNearest intersects the apex circle of radius R with the
// cutter circle (centre (cx, cy), radius rc) and keeps the solution nearest
// (refX, refY) — the branch the mean point sits on. A non-overlap clamps to
// tangency.
func bgCircleIntersectNearest(R, cx, cy, rc, refX, refY float64) bgPt {
	d := math.Hypot(cx, cy)
	if d == 0 {
		return bgPt{R, 0}
	}
	a := (d*d + R*R - rc*rc) / (2 * d)
	h2 := R*R - a*a
	if h2 < 0 {
		h2 = 0 // tangency
	}
	h := math.Sqrt(h2)
	ux, uy := cx/d, cy/d
	mx, my := a*ux, a*uy
	p1 := bgPt{mx + h*(-uy), my + h*ux}
	p2 := bgPt{mx - h*(-uy), my - h*ux}
	if math.Hypot(p1.X-refX, p1.Y-refY) <= math.Hypot(p2.X-refX, p2.Y-refY) {
		return p1
	}
	return p2
}

// bgGearFrame builds the §3a frame for a case, from the §2 lattice, with the
// caller hand-off spelled out.
func bgGearFrame(l bgLattice, g bgMember, swapped bool) bgFrame {
	w := func(p bgPt) r3.Vec { return r3.NewVec(l.bgRadius(g, p), 0, l.bgStation(g, p)) }
	toeMid := w(g.Toe).Add(w(g.ToeInner)).Scale(0.5) // 1/2 (M + N) / 1/2 (O + P)
	heelMid := w(g.Ded).Add(w(g.Heel)).Scale(0.5)    // 1/2 (C + H) / 1/2 (D + J)
	toeCone, heelCone := w(g.Toe), w(g.Ded)          // M / O, and C / D
	if swapped {
		return bgBuildFrame(l, g, heelMid, toeMid, heelCone, toeCone)
	}
	return bgBuildFrame(l, g, toeMid, heelMid, toeCone, heelCone)
}

// ----------------------------------------------------------------------------
// The spiral case tables.

func bgSpiralCases(module float64) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, bc := range bgSpiralParams(module) {
		out = append(out, proofkit3d.Case{Name: bc.name, Params: bc.p})
	}
	return out
}

func bgSpiralSketchCases(module float64) []proofkit.Case {
	var out []proofkit.Case
	for _, bc := range bgSpiralParams(module) {
		out = append(out, proofkit.Case{Name: bc.name, Params: bc.p})
	}
	return out
}

type bgNamedParams struct {
	name string
	p    map[string]float64
}

// bgSpiralParams reaches every branch the spiral offers from every direction:
// both gear sides, both hands, psi at its floor and just under its ceiling, the
// cutter radius both auto and given, the ratio both ways round, and the toe/heel
// hand-off deliberately swapped so the frame's own guard is exercised.
func bgSpiralParams(module float64) []bgNamedParams {
	var out []bgNamedParams
	base := map[string]float64{idModule: module, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90}
	add := func(name string, over map[string]float64) {
		p := map[string]float64{}
		for k, v := range base {
			p[k] = v
		}
		for k, v := range over {
			p[k] = v
		}
		out = append(out, bgNamedParams{name, p})
	}
	for _, side := range []struct {
		name string
		v    float64
	}{{"pinion", 0}, {"driving", 1}} {
		add("psi_0_straight_limit_"+side.name, map[string]float64{idSide: side.v, idSpiralAngle: 0})
		add("psi_35_right_auto_cutter_"+side.name, map[string]float64{idSide: side.v, idSpiralAngle: 35})
		add("psi_35_left_auto_cutter_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 35, idHand: bgHandLeft})
		add("psi_59_right_"+side.name, map[string]float64{idSide: side.v, idSpiralAngle: 59})
		add("psi_15_given_cutter_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 15, idCutterRadius: 30 * module})
		add("psi_35_equal_teeth_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 35, idDrivingTeeth: 31, idPinionTeeth: 31})
		add("psi_35_ratio_reversed_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 35, idDrivingTeeth: 17, idPinionTeeth: 31})
		add("psi_35_swapped_handoff_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 35, bgSwapKey: 1})
		add("psi_35_shaft_60_toe_ext_"+side.name, map[string]float64{idSide: side.v,
			idSpiralAngle: 35, idShaftAngle: 60, idToeExtension: 50})
	}
	return out
}

var (
	coneElementCases     = bgSpiralSketchCases(1)
	tracePlaneCases      = bgSpiralSketchCases(1)
	traceSketchCases     = bgSpiralSketchCases(1)
	spiralSliceCases     = bgSpiralCases(6)
	spiralScrapCases     = bgSpiralCases(6)
	spiralTwistCases     = bgSpiralCases(6)
	spiralCrownCases     = bgSpiralCases(6)
	spiralLoftCases      = bgSpiralCases(6)
	spiralFlushTrimCases = bgSpiralCases(6)
)

// ----------------------------------------------------------------------------
// S19 — the `{gear} Cone Element` sketch.

// stepSpiralConeElement draws the cone-element construction line
// Apex -> (Apex + R_heel * coneVec) in a sketch on the axial (Gear Profiles)
// plane, named `{gear} Cone Element`. The Trace Plane is built by rotating the
// axial plane about this line, so its direction is the whole of its job.
//
// COORDINATES. The raw apex and cone-end world points are passed DIRECTLY into
// addByTwoPoints, where they are consumed as sketch-space input — no
// modelToSketchSpace conversion is applied, even though Sketch offers exactly
// that call and the points really are model-space coordinates. That is
// deliberate: the only thing built on the Trace Plane is the inspection-only
// trace sketch, and the whole chain ends there. If a later revision ever makes
// any feature consume the trace sketch or the Trace Plane, the shortcut stops
// being safe and both sketches need modelToSketchSpace on every point.
func stepSpiralConeElement(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	f := bgGearFrame(l, g, bgGet(p, bgSwapKey, 0) != 0)

	proofkit.Step(t, "%s Cone Element: Apex -> Apex + R_heel * coneVec", g.Label)
	d := &bgDraw{t: t, s: s}
	// The axial plane's own 2-D frame is (radius, station), which is the frame
	// the §2 lattice already lives in.
	apex := s.CreateReferencePoint(0, 0, "Apex")
	end := bgPt{f.ConeVec.X * f.RHeel, f.ConeVec.Z * f.RHeel}
	endRef := s.CreateReferencePoint(end.X, end.Y, "cone element end")
	a := d.pt(bgPt{0, 0}, "cone element start")
	b := d.pt(end, "cone element end")
	line := d.line(a, b, g.Label+" Cone Element")
	d.add("the cone element starts at the Apex", sketch.NewCoincident(a, apex))
	d.add("the cone element ends at R_heel along the element", sketch.NewCoincident(b, endRef))
	_ = line

	// It runs along the ROOT cone element Apex->C / Apex->D, never along
	// Apex->Apex2 and never along the shaft axis.
	bgCloseTB(t, g.Label+" cone element runs along the root cone element",
		bgCross(bgUnit(bgPt{b.X(), b.Y()}), bgPt{f.ConeVec.X, f.ConeVec.Z}), 0, 1e-12)
	bgCloseTB(t, g.Label+" cone element reaches R_heel", math.Hypot(b.X(), b.Y()), f.RHeel, 1e-9)
	// And the heel is the OUTER end, so span is positive.
	if f.Span <= 0 {
		t.Errorf("%s: span came out %.6f — the toe is farther from the apex than the heel, "+
			"which inverts the whole spiral frame", g.Label, f.Span)
	}
	if bgGet(p, bgSwapKey, 0) != 0 && !f.Swapped {
		t.Errorf("%s: the toe and heel were handed over swapped and the frame's guard did not fire", g.Label)
	}
}

// ----------------------------------------------------------------------------
// S20 — the `{gear} Trace Plane`.

// stepSpiralTracePlane builds the tangent plane: the axial plane rotated 90°
// about the cone-element line, through plane_by_angle(comp, coneElementLine,
// axialPlane, 90). Its in-plane axes are x = coneVec (so a point's x is its cone
// distance) and y = v (circumferential).
func stepSpiralTracePlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	f := bgGearFrame(l, g, bgGet(p, bgSwapKey, 0) != 0)

	proofkit.Step(t, "%s Trace Plane: the axial plane turned 90 deg about the cone element", g.Label)
	plane, err := s.World().CreatePlaneFromFrame(mustFrame(t, f.Apex, f.ConeVec, f.V))
	if err != nil {
		t.Fatalf("%s Trace Plane: %v", g.Label, err)
	}
	frame, err := plane.Frame()
	if err != nil {
		t.Fatalf("%s Trace Plane frame: %v", g.Label, err)
	}
	// It contains the cone element, and it is square to the axial plane, which
	// is what "rotated 90 degrees about that line" means.
	bgCloseTB(t, "the Trace Plane contains the cone element", frame.N().Dot(f.ConeVec), 0, 1e-12)
	axialNormal := r3.NewVec(0, 1, 0) // the axial plane is XZ in the shaft frame
	bgCloseTB(t, "the Trace Plane stands square to the axial plane",
		frame.N().Dot(axialNormal), 0, 1e-12)
	// Its y axis is the circumferential direction, perpendicular to both the
	// element and the shaft axis.
	bgCloseTB(t, "the Trace Plane's y axis is circumferential", f.V.Dot(f.ConeVec), 0, 1e-12)
	bgCloseTB(t, "the Trace Plane's y axis is square to the shaft axis", f.V.Dot(f.AxisDir), 0, 1e-12)
	bgCloseTB(t, "the tangent-plane normal closes the frame",
		f.TPNormal.Cross(f.ConeVec).Sub(f.V).Len(), 0, 1e-9)

	stepSpiralConeElement(t, s, p)
}

func mustFrame(t testing.TB, o, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(o, u, v)
	if err != nil {
		t.Fatalf("frame: %v", err)
	}
	return frame
}

// ----------------------------------------------------------------------------
// S21 — the `{gear} 2D Tooth Trace` sketch.

// stepSpiralTrace draws the genuine cutter arc and proves every invariant
// spiral-tooth-trace.md §9 lists.
//
// SUBSTITUTION AND COST. In Fusion this sketch is DELIBERATELY left with free
// DOF — the arc's endpoints are pinned by the three-point construction, not by
// endpoint dimensions, because dimensioning them over-constrains the solve
// against the cone-element plane — and it is exempt from the full-constraint
// gate. The bench has no cone-element plane to over-constrain against, and
// proofkit's gate is not waivable, so the proof pins the two endpoints on their
// own apex circles instead. That is the construction's own definition (§6: the
// toe and heel points are circle-circle intersections), so it proves more than
// the free version rather than less. What it does not exercise is Fusion's
// three-point arc.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	f := bgGearFrame(l, g, bgGet(p, bgSwapKey, 0) != 0)
	psi := in.SpiralAngle

	proofkit.Step(t, "%s 2D Tooth Trace: the cutter circle, centred at (R_mean - r_c sin psi, handSign r_c cos psi)",
		g.Label)
	apex := s.CreateReferencePoint(0, 0, "apex, the trace frame's origin")
	// The cutter circle: construction, its centre pinned with isFixed rather
	// than coincident to the origin ([PB-CIRCLE-CENTER]), plus a diameter
	// dimension of 2 r_c.
	cutterCentre := s.CreatePoint(f.CX, f.CY)
	cutterCentre.SetName("cutter circle centre")
	cutter := s.CreateCircle(cutterCentre, f.RC)
	cutter.SetConstruction(true)
	s.Fix(cutterCentre)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*f.RC))

	// The two apex circles the trace's ends sit on, taken a hair past the face
	// so the kept arc reaches cleanly past the end-trims.
	toeCircle := s.CreateCircle(apex, f.RLo)
	toeCircle.SetConstruction(true)
	s.AddConstraint(sketch.NewRadius(toeCircle, f.RLo))
	heelCircle := s.CreateCircle(apex, f.RHi)
	heelCircle.SetConstruction(true)
	s.AddConstraint(sketch.NewRadius(heelCircle, f.RHi))

	proofkit.Step(t, "%s 2D Tooth Trace: the trace arc, the genuine cutter circle and not a look-alike spline",
		g.Label)
	// In Fusion the arc is a three-point arc with its centre coincident to the
	// cutter circle's centre and a radius dimension of r_c, and its endpoints
	// are left free — the sketch is exempt from the gate. On the bench every one
	// of those rows is implied once the three points are placed consistently, so
	// that scheme comes back either under-constrained or redundant, and the two
	// circle-circle intersections leave four discrete configurations the probe
	// reports. The proof therefore pins the two endpoints at their own solved
	// circle-circle positions with SIGNED rows — the bench's spelling of the seed
	// side ([PB-DIM-VALUE-SEMANTICS]) — and pins the centre's x, leaving the arc's
	// own radius-consistency row to close it. The centre's coincidence with the
	// cutter circle and the arc's radius are then read back below rather than
	// constrained: they are consequences, and asserting them is a stronger check
	// than declaring them.
	arcCentre := s.CreatePoint(f.CX, f.CY)
	arcCentre.SetName("trace arc centre")
	start := s.CreatePoint(f.Toe2D.X, f.Toe2D.Y)
	start.SetName("trace toe end")
	end := s.CreatePoint(f.Heel2D.X, f.Heel2D.Y)
	end.SetName("trace heel end")
	arc := s.CreateArc(arcCentre, start, end)
	s.AddConstraint(
		sketch.NewHorizontalDistance(apex, start, f.Toe2D.X),
		sketch.NewVerticalDistance(apex, start, f.Toe2D.Y),
		sketch.NewHorizontalDistance(apex, end, f.Heel2D.X),
		sketch.NewVerticalDistance(apex, end, f.Heel2D.Y),
	)
	// The arc's own radius-consistency row puts the centre on the perpendicular
	// bisector of its two ends, so the one row left to add must pin the
	// coordinate that bisector does NOT fix. Pinning the other one is a
	// near-singular system rather than a wrong answer, and at psi = 0 — where the
	// bisector stands almost vertical — it reads below the engine's trust floor.
	bisector := bgPt{-(f.Heel2D.Y - f.Toe2D.Y), f.Heel2D.X - f.Toe2D.X}
	if math.Abs(bisector.Y) > math.Abs(bisector.X) {
		s.AddConstraint(sketch.NewVerticalDistance(apex, arcCentre, f.CY))
	} else {
		s.AddConstraint(sketch.NewHorizontalDistance(apex, arcCentre, f.CX))
	}
	bgCloseTB(t, "the trace arc's centre solves onto the cutter circle's centre",
		math.Hypot(arcCentre.X()-cutterCentre.X(), arcCentre.Y()-cutterCentre.Y()), 0, 1e-9)
	_, _ = toeCircle, heelCircle

	// §9.1 apex-centred: the toe and heel loci are circles about the apex.
	bgCloseTB(t, "the trace's toe end sits on the toe circle", math.Hypot(start.X(), start.Y()), f.RLo, 1e-9)
	bgCloseTB(t, "the trace's heel end sits on the heel circle", math.Hypot(end.X(), end.Y()), f.RHi, 1e-9)
	// §9.2 cutter radius, and §9.3 it passes through the mean point.
	bgCloseTB(t, "the trace arc's radius is the cutter radius", arc.R(), f.RC, 1e-9)
	bgCloseTB(t, "the cutter centre sits r_c from the mean point",
		math.Hypot(f.CX-f.RMean, f.CY), f.RC, 1e-9)
	// §9.4 the spiral angle is realised AT the mean point: the radius M->C makes
	// psi with the y axis, so the tangent makes psi with the element.
	tangent := bgPt{-(f.CY - 0), f.CX - f.RMean}
	got := math.Abs(math.Atan2(bgCross(bgPt{1, 0}, bgUnit(tangent)), bgDot(bgPt{1, 0}, bgUnit(tangent))))
	if got > math.Pi/2 {
		got = math.Pi - got
	}
	bgCloseTB(t, "the mean spiral angle is realised at the mean point", got, psi, 1e-9)
	// §9.5 mirror symmetry: flipping the hand reflects the centre across the
	// cone element and changes nothing else. This is the check that catches the
	// hand sign being put on the sin/Cx term.
	other := f
	other.HandSign = -f.HandSign
	other.CX = f.RMean - f.RC*math.Sin(psi)
	other.CY = other.HandSign * f.RC * math.Cos(psi)
	bgCloseTB(t, "the opposite hand keeps the same x", other.CX, f.CX, 0)
	bgCloseTB(t, "the opposite hand mirrors y across the cone element", other.CY, -f.CY, 0)
	// §9.7 the straight-bevel limit: psi = 0 puts the centre straight north of
	// the mean point, so the arc is tangent to the element there.
	if psi == 0 {
		bgCloseTB(t, "psi = 0 puts the cutter centre straight north of the mean point", f.CX, f.RMean, 1e-12)
		bgCloseTB(t, "psi = 0 makes |Cy| the cutter radius", math.Abs(f.CY), f.RC, 1e-12)
		// The tangent check above already reads 0 here, which is the whole of the
		// limit: at psi = 0 the arc is tangent to the cone element at the mean
		// point. It does NOT leave zero twist, because the trace is still an arc
		// of finite radius and its ends still subtend an angle at the apex — the
		// straight tooth comes from the hook's own gate, which returns
		// cut_conical_ends before any of this runs, not from the arc degenerating.
		if f.Total <= 0 {
			t.Errorf("psi = 0 gave a zero twist; the straight tooth comes from the hook's gate, " +
				"and a zero here would mean the arc had degenerated instead")
		}
	}

	proofkit.Step(t, "the twist the trace fixes, by the conjugate crown-gear law")
	// phi_crown is the angle the arc's toe and heel endpoints subtend AT THE
	// APEX in the flat crown frame; the shaft-axis twist is that angle divided
	// by sin(gamma), with gamma this gear's PITCH cone angle from §2 — never
	// acos(coneVec . axisDir), which is the ROOT cone angle and inflates the
	// twist.
	phi := math.Atan2(end.Y(), end.X()) - math.Atan2(start.Y(), start.X())
	bgCloseTB(t, "phi_crown off the solved arc", phi, f.PhiCrown, 1e-9)
	bgCloseTB(t, "the toe-heel twist", math.Abs(phi)/math.Sin(g.Gamma), f.Total, 1e-9)
	rootConeAngle := math.Acos(f.ConeVec.Dot(f.AxisDir))
	if in.SpiralAngle > 0 && math.Abs(rootConeAngle-g.Gamma) < 1e-6 {
		t.Errorf("%s: the root cone angle and the pitch cone angle came out equal, so this case "+
			"cannot tell the correct 1/sin(gamma) roll ratio from the wrong one", g.Label)
	}
}

// ----------------------------------------------------------------------------
// The slab machinery §3a steps E through I share.
//
// SUBSTITUTION AND COST. Fusion splits the lofted tooth with
// slice_body_by_offset_planes, moves each piece with a free-move matrix, scales
// it with scaleFeatures and lofts through the pieces' faces. At the decad
// revision this repo pins there is no split and no scale, and Loft takes exactly
// two profiles, so the proof builds the same pieces directly: because the uncut
// tooth is a cone over its heel section, a plane PARALLEL to the parent tooth
// plane cuts it in that same section scaled by its share of the apex distance,
// which makes each slab exact rather than approximate. Every piece is laid apart
// along +X about its own parallel axis.
//
// What that leaves unproved is named once here and not repeated: the split
// itself, the scale feature and its base point, and the multi-section loft. What
// it does prove is that the pieces those operations must produce exist, partition
// the tooth, carry the faces the next step reads, and land at the angles and
// scales the closed form fixes.

// bgSlabPlan is one cross-section slab, computed WITHOUT building anything.
//
// Both the build and its assertion read this same plan, so the assertion never
// rebuilds bodies into the document it is judging — two copies of one slab
// laid on top of each other are a pair the evaluator cannot decide, and the gate
// reports that rather than the geometry.
type bgSlabPlan struct {
	Near, Far float64 // the section scales at the apex-side and heel-side faces
	Lateral   float64 // how far along +X this piece is laid apart
	PreHeel   float64 // distAlong of its heel face, BEFORE the twist
	Twist     float64 // its share of the total twist, about the shaft axis
	PostHeel  float64 // distAlong of its heel face, AFTER the twist
	U         float64 // its heel-distance fraction, 0 at the heel and 1 at the toe
	Factor    float64 // the crown scale it carries, 1 at the held-full heel
}

// bgSliceOffsets is the fixed slice scheme: the first cut plane is the parent
// tooth plane offset toward the apex by span/6, then a sequence stepped further
// toward the apex in span/6 increments — sign * (k+1) * span/6 for k = 0..7. The
// count is NOT user-configurable.
func bgSliceOffsets(span float64) []float64 {
	out := make([]float64, bgSlicePlanes)
	for k := range out {
		out[k] = float64(k+1) * span / 6
	}
	return out
}

// bgSlabScales turns those offsets into the scale each cut section carries. The
// parent tooth plane sits at perpendicular distance h from the apex, so a plane
// offset by o toward the apex cuts the cone at (h - o) / h of full size.
func bgSlabScales(h float64, offsets []float64) []float64 {
	out := make([]float64, 0, len(offsets)+1)
	out = append(out, 1) // the parent tooth plane itself, the tooth's heel end
	for _, o := range offsets {
		out = append(out, (h-o)/h)
	}
	return out
}

// bgApexHeight is the parent tooth plane's perpendicular distance from the apex.
func bgApexHeight(b *bgSolid) float64 {
	return b.lat.bgStation(b.g, b.g.ToothCtr) * math.Cos(b.g.Gamma)
}

// bgSectionCentroid is where a section's centroid lands in the shaft frame, for
// a given scale, crown factor and twist — the reading step G keys its rotation
// on and step H recomputes after it.
func bgSectionCentroid(b *bgSolid, poly []bgPt, scale, factor, twist float64) r3.Vec {
	cx, cy := bgPolygonCentroid(poly)
	frame := bgTwistedFrame(b, scale, twist)
	return frame.ToWorldUV(cx*scale*factor, cy*scale*factor)
}

// bgPolygonCentroid is the outline's own area centroid.
func bgPolygonCentroid(poly []bgPt) (float64, float64) {
	a, cx, cy := 0.0, 0.0, 0.0
	for i := range poly {
		j := (i + 1) % len(poly)
		cross := poly[i].X*poly[j].Y - poly[j].X*poly[i].Y
		a += cross
		cx += (poly[i].X + poly[j].X) * cross
		cy += (poly[i].Y + poly[j].Y) * cross
	}
	if a == 0 {
		return 0, 0
	}
	return cx / (3 * a), cy / (3 * a)
}

// bgPlanSlabs realizes §3a steps E, F, G and H as arithmetic: the nine pieces
// the eight cut planes leave, the apex scrap dropped, the per-segment twist, and
// the crown factor each kept segment carries.
func bgPlanSlabs(b *bgSolid, f bgFrame) (kept []bgSlabPlan, scrap bgSlabPlan) {
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	scales := bgSlabScales(bgApexHeight(b), bgSliceOffsets(f.Span))
	spread := bgSpread * b.lat.ConeDist

	all := make([]bgSlabPlan, 0, len(scales))
	for i := 0; i+1 < len(scales); i++ {
		all = append(all, bgSlabPlan{Near: scales[i+1], Far: scales[i], Factor: 1})
	}
	// The apex-side scrap: everything below the last cut plane, down to the nose
	// section that stands in for the loft's degenerate apex point.
	all = append(all, bgSlabPlan{Near: bgNose, Far: scales[len(scales)-1], Factor: 1})

	// Step F: sort by the distAlong of each piece's centroid; the first
	// (apex-most) is the long apex-side scrap below the toe. Re-slice the list
	// FIRST and only then drop the piece, so the kept list never holds it.
	sort.Slice(all, func(i, j int) bool {
		ci := bgSectionCentroid(b, poly, (all[i].Near+all[i].Far)/2, 1, 0)
		cj := bgSectionCentroid(b, poly, (all[j].Near+all[j].Far)/2, 1, 0)
		return f.DistAlong(ci) < f.DistAlong(cj)
	})
	scrap = all[0]
	kept = append(kept, all[1:]...)

	for i := range kept {
		// Step G: the rotation is a LINEAR SHARE keyed to the cone distance of
		// the segment's HEEL FACE — the exact section the later loft samples —
		// and NOT to its centroid. Centroid-keying leaves the loft's mid-face
		// section rotated by half a segment and the mid-faces overlap. The twist
		// is centred on R_mean so the mid-face section stays unrotated, which is
		// what the pinion's zero mesh nudge depends on.
		kept[i].PreHeel = f.DistAlong(bgSectionCentroid(b, poly, kept[i].Far, 1, 0))
		kept[i].Twist = -f.HandSign * f.Total * (f.RMean - kept[i].PreHeel) / f.Span
		kept[i].PostHeel = f.DistAlong(bgSectionCentroid(b, poly, kept[i].Far, 1, kept[i].Twist))
	}
	// Step H: "outermost (heel) segment" = the one with the GREATEST POST-TWIST
	// heel-face distAlong. The heel-face readings are RECOMPUTED after the twist
	// has moved the slabs, never reused from before it.
	sort.Slice(kept, func(i, j int) bool { return kept[i].PostHeel < kept[j].PostHeel })
	for i := range kept {
		kept[i].Lateral = float64(i) * spread
		if i == len(kept)-1 {
			kept[i].U, kept[i].Factor = 0, 1 // the heel segment is held full
			continue
		}
		kept[i].U = (f.RHeel - kept[i].PostHeel) / f.Span
		kept[i].Factor = 1 - bgCrownPerRad*(math.Abs(f.Total)/2)*kept[i].U
	}
	scrap.Lateral = float64(len(kept)) * spread
	return kept, scrap
}

// bgTwistedFrame is the tooth plane turned about the shaft axis by a segment's
// own twist and laid apart along +X, which is where that segment's faces sit
// once step G has moved them.
func bgTwistedFrame(b *bgSolid, scale, twist float64) r3.Frame {
	b.t.Helper()
	sK := b.lat.bgStation(b.g, b.g.ToothCtr)
	c, s := math.Cos(twist), math.Sin(twist)
	rot := func(v r3.Vec) r3.Vec { return r3.NewVec(v.X*c-v.Y*s, v.X*s+v.Y*c, v.Z) }
	u := rot(r3.NewVec(-math.Cos(b.g.Gamma), 0, math.Sin(b.g.Gamma)))
	v := rot(r3.NewVec(0, 1, 0))
	return mustFrame(b.t, r3.NewVec(0, 0, sK*scale), u, v)
}

// bgBuildSlab builds one planned slab, twisted or not, laid apart at its own
// lateral. The twist is applied as a rigid motion about the shaft axis — the
// bench's free-move — rather than baked into the section, so the rotation is a
// real transform of a real body.
func bgBuildSlab(b *bgSolid, poly []bgPt, sl bgSlabPlan, twisted bool) *decad.Body {
	b.t.Helper()
	section := func(scale float64) []bgPt {
		out := make([]bgPt, len(poly))
		for i, p := range poly {
			out[i] = bgScale(p, scale*sl.Factor)
		}
		return out
	}
	s0, p0 := b.polygon(bgLaidApart(b, sl.Near, sl.Lateral), section(sl.Near))
	s1, p1 := b.polygon(bgLaidApart(b, sl.Far, sl.Lateral), section(sl.Far))
	body, err := b.doc.Loft(s0, p0, s1, p1)
	if err != nil {
		b.t.Fatalf("%s slab loft: %v", b.g.Label, err)
	}
	if !twisted || sl.Twist == 0 {
		return body
	}
	rot, err := r3.RotationAround(r3.NewVec(sl.Lateral, 0, 0), r3.NewVec(0, 0, 1), units.Radians(sl.Twist))
	if err != nil {
		b.t.Fatalf("twist rotation: %v", err)
	}
	// Placed retires the receiver, so the untwisted piece does not linger in the
	// document beside the twisted one.
	moved, err := body.Placed(rot)
	if err != nil {
		b.t.Fatalf("twist: %v", err)
	}
	return moved
}

// bgLaidApart is the tooth plane at a given scale, laid apart along +X.
func bgLaidApart(b *bgSolid, scale, lateral float64) r3.Frame {
	f := bgTwistedFrame(b, scale, 0)
	return mustFrame(b.t, f.Origin().Add(r3.NewVec(lateral, 0, 0)), f.U(), f.V())
}

// bgSlabHeelFace reads a built slab's heel face.
//
// DEFINE A SLAB'S HEEL FACE PRECISELY: the face whose centroid has the GREATEST
// distAlong, searched across ALL of the slab's faces WITH NO SURFACE-TYPE
// FILTER; its toe face is the least-centroid one. ⚠ Do NOT restrict the search
// to planar faces — a sliced slab is bounded by a mix of the two planar cut
// faces and ruled side faces, and a type filter can pick the wrong face or miss
// the cut face, which makes the step-I loft fail with
// ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY. The same all-faces-by-centroid
// rule is used everywhere a slab end face is needed: the twist key, the crown
// base and the loft sections.
func bgSlabHeelFace(f bgFrame, body *decad.Body, lateral float64) (heel, toe float64) {
	heel, toe = math.Inf(-1), math.Inf(1)
	for _, face := range body.Faces() {
		c, ok := bgFaceCentroid(face)
		if !ok {
			continue
		}
		c.X -= lateral
		d := f.DistAlong(c)
		heel = math.Max(heel, d)
		toe = math.Min(toe, d)
	}
	return heel, toe
}

// bgFaceCentroid is a planar face's AREA centroid, taken over its outer loop —
// the same reading the generated module takes from physicalProperties
// .centerOfMass, and not the unweighted mean of its vertices, which sits
// somewhere else on any face whose edges differ in length.
func bgFaceCentroid(face *decad.Face) (r3.Vec, bool) {
	for _, loop := range face.Loops() {
		if !loop.IsOuter() {
			continue
		}
		pts := make([]r3.Vec, 0, 8)
		for _, ce := range loop.CoEdges() {
			if ce.IsForward() {
				pts = append(pts, ce.Start().Position().Value)
			} else {
				pts = append(pts, ce.End().Position().Value)
			}
		}
		if len(pts) < 3 {
			continue
		}
		var normal r3.Vec
		for i := 1; i+1 < len(pts); i++ {
			normal = normal.Add(pts[i].Sub(pts[0]).Cross(pts[i+1].Sub(pts[0])))
		}
		unit, ok := normal.Normalize()
		if !ok {
			continue
		}
		var sum r3.Vec
		area := 0.0
		for i := 1; i+1 < len(pts); i++ {
			cross := pts[i].Sub(pts[0]).Cross(pts[i+1].Sub(pts[0]))
			a := cross.Dot(unit) / 2
			area += a
			sum = sum.Add(pts[0].Add(pts[i]).Add(pts[i+1]).Scale(a / 3))
		}
		if area == 0 {
			continue
		}
		return sum.Scale(1 / area), true
	}
	return r3.Vec{}, false
}

// bgSectionAzimuth reads where a built slab's section sits round the shaft axis.
func bgSectionAzimuth(body *decad.Body, lateral float64) float64 {
	var sx, sy, n float64
	for _, v := range body.Vertices() {
		q := v.Position().Value
		sx, sy, n = sx+q.X-lateral, sy+q.Y, n+1
	}
	if n == 0 {
		return math.NaN()
	}
	return math.Atan2(sy/n, sx/n)
}

// bgStraightBranch is what every §3a step returns when psi = 0: the hook's first
// line is the gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`,
// so the straight tooth is built and none of §3a runs.
func bgStraightBranch(t *testing.T, b *bgSolid) []*decad.Body {
	body, _ := b.toothBody(bgNose, 0, 0)
	return []*decad.Body{body}
}

// ----------------------------------------------------------------------------
// S22 — slice the straight tooth.

func stepSpiralSlice(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return bgStraightBranch(t, b)
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, scrap := bgPlanSlabs(b, f)
	out := make([]*decad.Body, 0, len(kept)+1)
	for _, sl := range kept {
		sl.Factor = 1
		out = append(out, bgBuildSlab(b, poly, sl, false))
	}
	return append(out, bgBuildSlab(b, poly, scrap, false))
}

func assertSpiralSlice(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		if len(bodies) != 1 {
			t.Errorf("psi = 0 takes the straight path: %d bodies built, want the one uncut tooth", len(bodies))
		}
		return
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	offsets := bgSliceOffsets(f.Span)
	if len(offsets) != bgSlicePlanes {
		t.Fatalf("the slice scheme is fixed at %d planes, got %d", bgSlicePlanes, len(offsets))
	}
	bgClose(t, "the first cut plane sits span/6 from the parent tooth plane", offsets[0], f.Span/6, 1e-12)
	for k := 1; k < len(offsets); k++ {
		bgClose(t, "the cut planes step by span/6", offsets[k]-offsets[k-1], f.Span/6, 1e-12)
	}
	// ⚠ THE SLICE MUST ACTUALLY SPLIT THE TOOTH. If the body is still in one
	// piece after the cut loop the offset sign was wrong, or the parent tooth
	// plane sits outside the tooth's span; the generated module retries the whole
	// cut once with the opposite sign and then raises a self-diagnosing error
	// naming the gear, the final piece count, span and the sign tried. It must
	// NOT return an unsliced single piece: step F then drops that one piece as
	// the apex scrap, leaving segments empty, and the crown crashes with
	// `max() iterable argument is empty` far from the cause.
	if len(bodies) != bgSlicePlanes+1 {
		t.Fatalf("the slice left %d piece(s); %d cut planes must leave %d",
			len(bodies), bgSlicePlanes, bgSlicePlanes+1)
	}
	// Every plane moves TOWARD the apex, which is the whole of the sign rule: the
	// parent plane's normal points opposite ways for the two gears, so the module
	// picks the sign that makes sign * normal point apex-ward, testing
	// (apex - planeOrigin) . normal rather than assuming one.
	h := bgApexHeight(b)
	if offsets[len(offsets)-1] >= h {
		t.Errorf("the last cut plane at %.4f is past the apex at %.4f", offsets[len(offsets)-1], h)
	}
	// The pieces partition the tooth: their volumes sum to the cone between the
	// nose and the parent tooth plane.
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	whole := bgPolygonArea(poly) * h / 3 * (1 - bgNose*bgNose*bgNose)
	sum, bound := 0.0, 0.0
	for _, body := range bodies {
		v, bd := bgVolume(t, body)
		sum += v
		bound += bd
	}
	bgClose(t, "the slabs partition the tooth", sum, whole, math.Max(bound, 1e-6*whole))
}

// ----------------------------------------------------------------------------
// S23 — order the segments and drop the apex scrap.

func stepSpiralScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return bgStraightBranch(t, b)
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, _ := bgPlanSlabs(b, f)
	if len(kept) == 0 {
		t.Fatal("the slice failed: no cross-section segment survived the scrap drop")
	}
	out := make([]*decad.Body, 0, len(kept))
	for _, sl := range kept {
		sl.Factor = 1
		out = append(out, bgBuildSlab(b, poly, sl, false))
	}
	return out
}

func assertSpiralScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	kept, scrap := bgPlanSlabs(b, f)
	if len(bodies) != bgSlicePlanes {
		t.Fatalf("after dropping the apex scrap %d segments remain, want %d", len(bodies), bgSlicePlanes)
	}
	// After the drop, segments must be NON-EMPTY: the twist and the crown both
	// assume at least one cross-section.
	if len(kept) == 0 {
		t.Fatal("segments came back empty")
	}
	// The dropped piece really is the apex-most, and it is the long one.
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	scrapCentroid := f.DistAlong(bgSectionCentroid(b, poly, (scrap.Near+scrap.Far)/2, 1, 0))
	for _, sl := range kept {
		keptCentroid := f.DistAlong(bgSectionCentroid(b, poly, (sl.Near+sl.Far)/2, 1, 0))
		if scrapCentroid >= keptCentroid {
			t.Errorf("the dropped piece was not the apex-most: scrap at %.6f, a kept segment at %.6f",
				scrapCentroid, keptCentroid)
		}
	}
	if scrap.Far-scrap.Near <= kept[0].Far-kept[0].Near {
		t.Errorf("the dropped piece was not the long apex-side scrap")
	}
	for _, body := range bodies {
		if v, _ := bgVolume(t, body); v <= 0 {
			t.Errorf("a kept segment came out with no volume")
		}
	}
}

// ----------------------------------------------------------------------------
// S24 — the twist.

func stepSpiralTwist(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return bgStraightBranch(t, b)
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, _ := bgPlanSlabs(b, f)
	out := make([]*decad.Body, 0, len(kept))
	for _, sl := range kept {
		sl.Factor = 1
		out = append(out, bgBuildSlab(b, poly, sl, true))
	}
	return out
}

func assertSpiralTwist(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, _ := bgPlanSlabs(b, f)
	if len(bodies) != len(kept) {
		t.Fatalf("the twist returned %d segments, want %d", len(bodies), len(kept))
	}
	// The untwisted section's own azimuth, so the reading below measures the
	// rotation and not where the tooth happened to be drawn.
	cx, cy := bgPolygonCentroid(poly)
	base := bgTwistedFrame(b, 1, 0).ToWorldUV(cx, cy)
	baseAzimuth := math.Atan2(base.Y, base.X)

	for i, sl := range kept {
		want := -f.HandSign * f.Total * (f.RMean - sl.PreHeel) / f.Span
		bgClose(t, "the segment's share of the total twist", sl.Twist, want, 1e-12)
		// Read the rotation off the body rather than trusting the plan.
		got := bgSectionAzimuth(bodies[i], sl.Lateral) - baseAzimuth
		got = math.Mod(got+3*math.Pi, 2*math.Pi) - math.Pi
		bgClose(t, "the segment turned by its own share", got, sl.Twist, 1e-6)
		// And its heel face is where the twist put it.
		heel, _ := bgSlabHeelFace(f, bodies[i], sl.Lateral)
		bgClose(t, "the segment's post-twist heel face", heel, sl.PostHeel, 1e-6*math.Abs(sl.PostHeel))
	}
	// The twist is a linear share centred on R_mean, so the MID-FACE SECTION
	// STAYS UNROTATED and meshes exactly like the straight tooth — which is what
	// the pinion's zero mesh nudge depends on.
	lo, hi := kept[0], kept[len(kept)-1]
	if (lo.PreHeel-f.RMean)*(hi.PreHeel-f.RMean) >= 0 {
		t.Errorf("the segments do not straddle R_mean (%.4f), so nothing is left unrotated at mid-face",
			f.RMean)
	}
	gotTotal := math.Abs(hi.Twist-lo.Twist) * f.Span / math.Abs(hi.PreHeel-lo.PreHeel)
	bgClose(t, "the toe-to-heel twist is the crown-gear law's own total", gotTotal, f.Total, 1e-9)

	// ⚠ The two members of a meshing pair legitimately get DIFFERENT twists:
	// same cutter, same psi, but gamma differs, so the roll ratio 1/sin(gamma)
	// differs. This is why an equal-teeth pair meshes under any method that gets
	// that ratio wrong, while a ratio pair fails.
	other := b.lat.Pinion
	if !b.lat.In.Driving {
		other = b.lat.Driving
	}
	otherTotal := bgGearFrame(b.lat, other, false).Total
	if b.lat.In.PinionTeeth != b.lat.In.DrivingTeeth && math.Abs(otherTotal-f.Total) < 1e-9 {
		t.Errorf("a ratio pair came out with equal twists (%.9f), so 1/sin(gamma) was not applied per gear",
			f.Total)
	}
	// And the roll ratio really is the PITCH cone angle's, not the root cone's.
	rootAngle := math.Acos(f.ConeVec.Dot(f.AxisDir))
	bgClose(t, "the twist uses the pitch cone angle's roll ratio",
		f.Total, math.Abs(f.PhiCrown)/math.Sin(b.g.Gamma), 1e-12)
	if math.Abs(math.Abs(f.PhiCrown)/math.Sin(rootAngle)-f.Total) < 1e-9 {
		t.Errorf("this case cannot tell the pitch cone angle from the root cone angle")
	}
}

// ----------------------------------------------------------------------------
// S25 — the lengthwise crown.

func stepSpiralCrown(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return bgStraightBranch(t, b)
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, _ := bgPlanSlabs(b, f)
	out := make([]*decad.Body, 0, len(kept))
	for _, sl := range kept {
		if sl.Factor <= 0 {
			t.Fatalf("%s: the crown factor came out %.6f at u = %.6f — never scale by a "+
				"non-positive factor", b.g.Label, sl.Factor, sl.U)
		}
		out = append(out, bgBuildSlab(b, poly, sl, true))
	}
	return out
}

func assertSpiralCrown(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	kept, _ := bgPlanSlabs(b, f)

	// _CROWN_PER_RAD is a tunable class constant, default 0.5 — 0 disables the
	// crown, and it must not be left unset.
	bgClose(t, "_CROWN_PER_RAD", bgCrownPerRad, 0.5, 0)
	if len(bodies) != len(kept) {
		t.Fatalf("the crown returned %d segments, want %d", len(bodies), len(kept))
	}
	prev := math.Inf(-1)
	for i, sl := range kept {
		if i == len(kept)-1 {
			// Skip the outermost (heel) segment: its heel face is the loft's heel
			// end and must stay full so the heel cone trims it flush with the
			// gear base.
			bgClose(t, "the heel segment is held full", sl.Factor, 1, 0)
			continue
		}
		// ⚠ Key the relief on the MONOTONIC heel-distance u, never on |ang|.
		// |ang| is symmetric about mid-face — maximal at BOTH ends — so with the
		// heel slab held full the slab just inside the heel becomes the most
		// relieved one and dips below both its neighbours, a notch that reverses
		// the heel-to-toe taper. Measured, that bug gave the heel-adjacent slab
		// factor 0.932 while the next slab inward was 0.972, taller.
		bgClose(t, "the crown factor", sl.Factor,
			1-bgCrownPerRad*(math.Abs(f.Total)/2)*sl.U, 1e-12)
		bgClose(t, "the heel-distance fraction u", sl.U, (f.RHeel-sl.PostHeel)/f.Span, 1e-12)
		if sl.Factor <= 0 {
			t.Errorf("segment %d scaled by a non-positive factor %.6f", i, sl.Factor)
		}
		// Relief grows monotonically from the held-full heel to the toe, so the
		// slab heights stay strictly ordered heel to toe and the natural cone
		// taper is never reversed.
		if sl.Factor <= prev {
			t.Errorf("the crown is not monotonic: segment %d at factor %.6f follows %.6f",
				i, sl.Factor, prev)
		}
		prev = sl.Factor
		// The maximum relief is now at the TOE and keeps the magnitude the old
		// per-end peak had. u itself runs from 0 at the held-full heel out past
		// 1 at the toe, because the eight cut planes at span/6 reach 8/6 of a
		// span beyond the parent tooth plane.
		if sl.U < -1e-9 {
			t.Errorf("segment %d has a negative heel-distance fraction u = %.6f", i, sl.U)
		}
	}
	// Read the relief off the bodies: each crowned segment reaches its own
	// factor times the radius it would have reached full.
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	reach := 0.0
	for _, q := range poly {
		reach = math.Max(reach, math.Hypot(q.X*math.Cos(b.g.Gamma), q.Y))
	}
	for i, sl := range kept {
		got := 0.0
		for _, v := range bodies[i].Vertices() {
			q := v.Position().Value
			got = math.Max(got, math.Hypot(q.X-sl.Lateral, q.Y))
		}
		bgClose(t, "the crowned segment's own reach", got, sl.Far*sl.Factor*reach, 1e-6*reach)
	}
}

// ----------------------------------------------------------------------------
// S26 — loft the curved tooth.

func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return bgStraightBranch(t, b)
	}
	return bgSpiralToothBodies(t, b, p)
}

// bgSpiralToothBodies lofts `{gear} Spiral Tooth` through the ordered sections.
//
// ⚠ RE-SORT THE SEGMENTS BY THEIR HEEL-FACE CONE DISTANCE HERE, AFTER THE TWIST
// AND THE CROWN — never reuse the pre-twist slice order. The twist rotates each
// slab about the shaft axis, and for high-twist unequal-ratio pairs that
// rotation changes the slabs' along-cone order enough to reorder adjacent slabs;
// lofting in the stale order assembles the cross-sections out of sequence and
// the crowned tooth comes out distorted, so the pair interferes. Equal and
// low-twist pairs give the same order either way, which is the single thing that
// makes a ratio pair like 31/17 fail while 31/31 looks fine.
//
// The loft's first section is the TOE-MOST segment's apex-side (toe-facing)
// face, added first so the loft reaches past the toe cone and the toe trim
// bites; then the heel-facing face of every segment in order, the last reaching
// past the heel cone.
//
// SUBSTITUTION AND COST: decad's Loft takes exactly two profiles, so the proof
// builds the chain as one lofted piece per consecutive section pair, laid apart.
// What that does not show is the evaluator making one body out of the whole
// chain; what it does show is that the sections exist, are in the right order,
// and carry the twist and crown each one is due.
func bgSpiralToothBodies(t *testing.T, b *bgSolid, p map[string]float64) []*decad.Body {
	t.Helper()
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	kept, _ := bgPlanSlabs(b, f)
	sort.Slice(kept, func(i, j int) bool { return kept[i].PostHeel < kept[j].PostHeel })

	type section struct{ scale, factor, twist float64 }
	sections := []section{{kept[0].Near, kept[0].Factor, kept[0].Twist}}
	for _, sl := range kept {
		sections = append(sections, section{sl.Far, sl.Factor, sl.Twist})
	}
	spread := bgSpread * b.lat.ConeDist
	out := make([]*decad.Body, 0, len(sections)-1)
	for i := 0; i+1 < len(sections); i++ {
		lateral := float64(i) * spread
		build := func(sec section) (*sketch.Sketch, *sketch.Profile) {
			pts := make([]bgPt, len(poly))
			for k, q := range poly {
				pts[k] = bgScale(q, sec.scale*sec.factor)
			}
			frame := bgTwistedFrame(b, sec.scale, sec.twist)
			return b.polygon(mustFrame(b.t, frame.Origin().Add(r3.NewVec(lateral, 0, 0)),
				frame.U(), frame.V()), pts)
		}
		s0, p0 := build(sections[i])
		s1, p1 := build(sections[i+1])
		piece, err := b.doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("%s Spiral Tooth loft section %d: %v", b.g.Label, i, err)
		}
		out = append(out, piece)
	}
	return out
}

func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	if b.lat.In.SpiralAngle <= 0 {
		return
	}
	f := bgGearFrame(b.lat, b.g, bgGet(p, bgSwapKey, 0) != 0)
	kept, _ := bgPlanSlabs(b, f)
	sort.Slice(kept, func(i, j int) bool { return kept[i].PostHeel < kept[j].PostHeel })

	if len(bodies) != len(kept) {
		t.Fatalf("the spiral loft produced %d pieces, want one per section pair (%d)", len(bodies), len(kept))
	}
	// The order is strictly increasing in POST-TWIST heel-face cone distance.
	for i := 1; i < len(kept); i++ {
		if kept[i].PostHeel <= kept[i-1].PostHeel {
			t.Errorf("the post-twist loft order is not strictly increasing in cone distance: "+
				"%.6f follows %.6f", kept[i].PostHeel, kept[i-1].PostHeel)
		}
	}
	// The chain reaches past the toe and past the heel, which is what makes the
	// two conical trims bite.
	if kept[0].Near >= kept[0].Far {
		t.Errorf("the toe-most section is not the apex-side face of the toe segment")
	}
	spread := bgSpread * b.lat.ConeDist
	for i, body := range bodies {
		v, _ := bgVolume(t, body)
		if v <= 0 {
			t.Errorf("%s Spiral Tooth piece %d came out with no volume", b.g.Label, i)
		}
		heel, toe := bgSlabHeelFace(f, body, float64(i)*spread)
		if heel <= toe {
			t.Errorf("%s Spiral Tooth piece %d has its heel face inside its toe face", b.g.Label, i)
		}
	}
	if f.Total <= 0 {
		t.Errorf("%s: psi is %.4f deg but the twist came out zero",
			b.g.Label, b.lat.In.SpiralAngle*180/math.Pi)
	}
}

// ----------------------------------------------------------------------------
// S27 — the flush trim on the curved tooth.

// stepSpiralFlushTrim is the same two-cone trim the straight tooth takes —
// cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid,
// apexWorld, gearLabel) — so the curved tooth's ends sit flush on the gear base.
// Neither cut is performed here, for the reason stepConicalEndCuts gives; the
// operands are built and laid apart and the crossings are solved from their own
// readings. The toe and heel MESH PHASING is handled outside this hook, by
// _createGearBody's mesh-rotate step.
func stepSpiralFlushTrim(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	out := make([]*decad.Body, 0, 4)
	if l.In.SpiralAngle <= 0 {
		out = append(out, bgStraightBranch(t, b)...)
	} else {
		out = append(out, bgSpiralToothBodies(t, b, p)...)
	}
	far := float64(len(out)+1) * spread
	out = append(out,
		b.band(l.bgStation(g, g.ToeInner), l.bgRadius(g, g.ToeInner),
			l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe), far),
		b.band(l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded),
			l.bgStation(g, g.Heel), l.bgRadius(g, g.Heel), far+spread),
		b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
			l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), far+2*spread))
	return out
}

func assertSpiralFlushTrim(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	n := len(bodies)
	far := float64(n-3+1) * spread
	toe := bgReadCone(t, bodies[n-3], far)
	heel := bgReadCone(t, bodies[n-2], far+spread)
	root := bgReadCone(t, bodies[n-1], far+2*spread)
	cross := func(c bgConeReading, m float64) float64 {
		return c.ApexStation * math.Abs(c.Slope) / (m + math.Abs(c.Slope))
	}
	bgClose(t, "the curved tooth's toe cut lands on the toe end of the flush band",
		cross(toe, math.Abs(root.Slope)), l.bgStation(g, g.Toe), 1e-4*l.bgStation(g, g.Toe))
	bgClose(t, "the curved tooth's heel cut lands on the heel end of the flush band",
		cross(heel, math.Abs(root.Slope)), l.bgStation(g, g.Ded), 1e-4*l.bgStation(g, g.Ded))
	// The pinion's extra phase is 0 by default because the mid-face section is
	// unrotated and already meshes.
	bgClose(t, "the pinion's extra mesh phase is zero by default", bgMeshPhaseTeeth, 0, 0)
}
