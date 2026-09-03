package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// radiansOf wraps a plain radian magnitude for the transform builders.
func radiansOf(a float64) units.Value { return units.Radians(a) }

// The end-trim overshoot §3a step B takes the cutter arc's ends a hair past the
// face by, as a fraction of the span.
const traceOvershoot = 0.06

// spiralCases sweeps the spiral branch across the regime it is defined on.
//
// The Mean Spiral Angle's stated range is [0, 60), and psi = 0 is the OTHER
// side of the branch — it is not a small spiral, it is the straight tooth, and
// the straight build is what the tables in solids_test.go and bodies_test.go
// sweep. So this table starts just above zero and runs to just under 60, which
// is where the cutter centre's along-element offset r_c*sin(psi) is largest.
//
// Both hands are here, and both gears of a pair, because the hand sign is
// negated for the pinion and the twist divides by sin(gamma), so the two members
// of one pair legitimately get different twists from the same cutter. The cutter
// radius is swept on both sides of its own branch: zero, where it resolves to
// the mean cone distance, and a given value, both smaller and larger than that.
var spiralCases = []proofkit.Case{
	{Name: "default_35deg_right_pinion", Params: defaultParams()},
	{Name: "default_35deg_right_driving", Params: with(defaultParams(), pGear, 1.0)},
	{Name: "default_35deg_left_pinion", Params: with(defaultParams(), pHandSign, -1.0)},
	{Name: "default_35deg_left_driving", Params: with(defaultParams(), pHandSign, -1.0, pGear, 1.0)},
	{Name: "shallow_5deg_right_pinion", Params: with(defaultParams(), pSpiralAngle, deg(5))},
	{Name: "steep_59deg_right_pinion", Params: with(defaultParams(), pSpiralAngle, deg(59))},
	{Name: "steep_59deg_left_driving", Params: with(defaultParams(),
		pSpiralAngle, deg(59), pHandSign, -1.0, pGear, 1.0)},
	{Name: "given_cutter_radius_20mm_pinion", Params: with(defaultParams(), pCutterRadius, 20.0)},
	{Name: "given_cutter_radius_60mm_driving", Params: with(defaultParams(),
		pCutterRadius, 60.0, pGear, 1.0)},
	{Name: "ratio_31_17_pinion", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_31_17_driving", Params: with(defaultParams(), pPinionTeeth, 17.0, pGear, 1.0)},
	{Name: "shaft_angle_35deg_pinion", Params: with(defaultParams(), pShaftAngle, deg(35))},
	{Name: "shaft_angle_140deg_driving", Params: with(defaultParams(),
		pShaftAngle, deg(140), pGear, 1.0)},
}

// spiralSolidCases is spiralCases as a solid table.
var spiralSolidCases = []proofkit3d.Case{
	{Name: "default_35deg_right_pinion", Params: defaultParams()},
	{Name: "default_35deg_right_driving", Params: with(defaultParams(), pGear, 1.0)},
	{Name: "default_35deg_left_pinion", Params: with(defaultParams(), pHandSign, -1.0)},
	{Name: "shallow_5deg_right_pinion", Params: with(defaultParams(), pSpiralAngle, deg(5))},
	{Name: "steep_59deg_left_driving", Params: with(defaultParams(),
		pSpiralAngle, deg(59), pHandSign, -1.0, pGear, 1.0)},
	{Name: "given_cutter_radius_20mm_pinion", Params: with(defaultParams(), pCutterRadius, 20.0)},
	{Name: "ratio_31_17_pinion", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_31_17_driving", Params: with(defaultParams(), pPinionTeeth, 17.0, pGear, 1.0)},
}

// spiral is one gear's §3a frame and the cutter-arc geometry derived in it.
//
// The frame is the one step A builds, written in the gear's own axial frame: the
// apex at the origin, coneVec along the root cone element Apex->C (pinion) or
// Apex->D (driving), and v = axisDir x coneVec as the circumferential direction.
// The 2-D trace coordinates are (x along coneVec, y along v) with the apex at
// the origin, which is the flat crown-gear plane §5 of spiral-tooth-trace.md
// works in.
type spiral struct {
	q        pair
	side     gearSide
	coneVec  vec2 // in the axial frame: (along the shaft axis, radial)
	rToe     float64
	rHeel    float64
	rMean    float64
	span     float64
	cutter   float64
	handSign float64
	cx, cy   float64
	toe2d    vec2
	heel2d   vec2
	phiCrown float64
	total    float64
}

// spiralOf builds one case's frame and cutter arc.
func spiralOf(t testing.TB, params map[string]float64) spiral {
	t.Helper()
	q, side, hex := hexagonOf(t, params)
	// The four points §3a's caller hand-off passes in, from the §2 lattice: the
	// midpoints of two DIFFERENT edges, and the dedendum corner C/D — never H/J,
	// which sit off the root cone element.
	heelCone := hex[3]
	toeCone := hex[4]
	heelMid := hex[2].add(hex[3]).scale(0.5)
	toeMid := hex[4].add(hex[5]).scale(0.5)

	s := spiral{q: q, side: side, handSign: side.handSign}
	// The swap guard: the heel must be the OUTER end, or coneVec points inward
	// and the whole spiral frame inverts silently.
	if heelMid.norm() < toeMid.norm() {
		toeMid, heelMid = heelMid, toeMid
		toeCone, heelCone = heelCone, toeCone
		t.Errorf("%s: the toe/heel hand-off arrived swapped and the guard had to fix it; the §2 "+
			"edges are supposed to arrive in the right roles", side.label)
	}
	s.coneVec = heelCone.scale(1 / heelCone.norm())
	s.rToe = toeMid.dot(s.coneVec)
	s.rHeel = heelMid.dot(s.coneVec)
	s.rMean = (s.rToe + s.rHeel) / 2
	s.span = s.rHeel - s.rToe

	s.cutter = q.cutterRadius
	if s.cutter == 0 {
		s.cutter = s.rMean
	}
	// The cutter-circle centre. The hand sign belongs on the cos term, so
	// opposite hands mirror the centre across the cone element rather than
	// across x = R_mean, which would be a different curve.
	s.cx = s.rMean - s.cutter*math.Sin(q.spiralAngle)
	s.cy = s.handSign * s.cutter * math.Cos(q.spiralAngle)

	s.toe2d = circleIntersectNearest(t, s.rToe-traceOvershoot*s.span, s.cx, s.cy, s.cutter,
		s.rMean, 0)
	s.heel2d = circleIntersectNearest(t, s.rHeel+traceOvershoot*s.span, s.cx, s.cy, s.cutter,
		s.rMean, 0)

	s.phiCrown = math.Atan2(s.heel2d.Y, s.heel2d.X) - math.Atan2(s.toe2d.Y, s.toe2d.X)
	s.total = math.Abs(s.phiCrown) / math.Sin(side.gamma)
	return s
}

// circleIntersectNearest intersects the apex circle of radius r with the cutter
// circle and keeps the solution nearest the reference point — the branch the
// mean point sits on.
func circleIntersectNearest(t testing.TB, r, cx, cy, rc, refX, refY float64) vec2 {
	t.Helper()
	d := math.Hypot(cx, cy)
	if d == 0 {
		t.Fatalf("the cutter circle is centred on the apex, so no apex circle meets it in two points")
	}
	// Non-overlap clamps to tangency, which is what the framework helper does.
	a := (r*r - rc*rc + d*d) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	ux, uy := cx/d, cy/d
	px, py := a*ux, a*uy
	first := vec2{px + h*uy, py - h*ux}
	second := vec2{px - h*uy, py + h*ux}
	if first.sub(vec2{refX, refY}).norm() <= second.sub(vec2{refX, refY}).norm() {
		return first
	}
	return second
}

// stepToothTrace draws the `{gear} 2D Tooth Trace` sketch: the cutter circle and
// the genuine circular tooth trace on it.
//
// WHAT THIS SKETCH SUBSTITUTES. In Fusion the arc's toe and heel ends are placed
// by the three-point construction and are DELIBERATELY left with free DOF, which
// is why [BEVEL-F-FULL-CONSTRAINT] exempts this sketch from the gate. The
// harness has no such exemption, so the two ends are pinned here by signed
// horizontal and vertical distances from the apex — the coordinates
// circle_intersect_nearest computes. What that costs is the constraint version
// of §6: an end pinned instead by its apex distance and the arc's radius would
// carry the two-branch ambiguity the framework helper resolves by picking the
// solution nearest the mean point, and a scheme whose answer depends on that
// choice is one the harness refuses. So this step proves the ARITHMETIC of the
// construction — that the computed ends really do lie on both circles, and that
// the arc through them is the cutter circle — rather than a constraint net that
// derives them.
//
// The invariants asserted are spiral-tooth-trace.md §9's, one for one.
//
// <!-- proof-run: proofkit.Run(spiralCases, stepToothTrace) -->
func stepToothTrace(t testing.TB, sk *sketch.Sketch, params map[string]float64) {
	s := spiralOf(t, params)

	proofkit.Step(t, "%s: R_toe %.6f, R_mean %.6f, R_heel %.6f, span %.6f, cutter %.6f",
		s.side.label, s.rToe, s.rMean, s.rHeel, s.span, s.cutter)
	if s.span <= 0 {
		t.Fatalf("%s: the span is %.6f; a negative span inverts the entire spiral frame silently",
			s.side.label, s.span)
	}

	proofkit.Step(t, "the cone-element reference point at the apex, and the mean point on it")
	apex := sk.CreateReferencePoint(0, 0, "apex")
	apex.SetName("Apex")
	mean := sk.CreatePoint(s.rMean, 0)
	mean.SetName("mean point")
	sk.AddConstraint(
		sketch.NewHorizontalDistance(apex, mean, s.rMean),
		sketch.NewVerticalDistance(apex, mean, 0),
	)

	proofkit.Step(t, "the cutter circle, sharing the arc's centre, with its diameter dimensioned "+
		"[PB-CIRCLE-CENTER]")
	center := sk.CreatePoint(s.cx, s.cy)
	center.SetName("cutter centre")
	circle := sk.CreateCircle(center, s.cutter)
	circle.SetConstruction(true)
	circle.SetName("cutter circle")
	sk.AddConstraint(sketch.NewDiameter(circle, 2*s.cutter))

	proofkit.Step(t, "the trace arc through the toe end, the mean point and the heel end")
	toe := sk.CreatePoint(s.toe2d.X, s.toe2d.Y)
	toe.SetName("toe end")
	heel := sk.CreatePoint(s.heel2d.X, s.heel2d.Y)
	heel.SetName("heel end")
	sk.AddConstraint(
		sketch.NewHorizontalDistance(apex, toe, s.toe2d.X),
		sketch.NewVerticalDistance(apex, toe, s.toe2d.Y),
		sketch.NewHorizontalDistance(apex, heel, s.heel2d.X),
		sketch.NewVerticalDistance(apex, heel, s.heel2d.Y),
	)
	arc := sk.CreateArc(center, toe, heel)
	arc.SetConstruction(true)
	arc.SetName("tooth trace")
	// The engine's arc carries its own equal-radius row, so the centre is held on
	// the perpendicular bisector of the two ends; one signed coordinate closes it.
	// Fixing the centre outright, the way [PB-CIRCLE-CENTER] has Fusion do it,
	// would make that implicit row redundant and the gate would refuse the sketch.
	sk.AddConstraint(sketch.NewHorizontalDistance(apex, center, s.cx))

	solve(t, sk)
	assertTraceInvariants(t, s, apex, center, mean, toe, heel)
}

// assertTraceInvariants checks spiral-tooth-trace.md §9 against the solved
// sketch, in its own order.
func assertTraceInvariants(t testing.TB, s spiral, apex, center, mean, toe, heel *sketch.Point) {
	t.Helper()
	const tol = 1e-9
	c := at(center)

	proofkit.Step(t, "1 and 6: apex-centred loci, and the ends on the right apex circles")
	if got := at(toe).sub(at(apex)).norm(); math.Abs(got-(s.rToe-traceOvershoot*s.span)) > tol {
		t.Errorf("%s: the toe end sits at cone distance %.9f, not R_toe less the %.0f per cent "+
			"overshoot, %.9f", s.side.label, got, 100*traceOvershoot, s.rToe-traceOvershoot*s.span)
	}
	if got := at(heel).sub(at(apex)).norm(); math.Abs(got-(s.rHeel+traceOvershoot*s.span)) > tol {
		t.Errorf("%s: the heel end sits at cone distance %.9f, not R_heel plus the overshoot, %.9f",
			s.side.label, got, s.rHeel+traceOvershoot*s.span)
	}

	proofkit.Step(t, "2 and 3: the arc is the cutter circle, and it passes through the mean point")
	for name, p := range map[string]*sketch.Point{"toe end": toe, "heel end": heel, "mean point": mean} {
		if got := at(p).sub(c).norm(); math.Abs(got-s.cutter) > 1e-7 {
			t.Errorf("%s: the %s sits %.9f from the cutter centre, not the cutter radius %.9f",
				s.side.label, name, got, s.cutter)
		}
	}

	proofkit.Step(t, "4: the trace makes the Mean Spiral Angle with the cone element at the mean point")
	// The tangent at the mean point is perpendicular to the radius from the
	// centre, and the element is the x axis, so the angle between them is what
	// psi is defined as.
	radial := at(mean).sub(c)
	tangent := vec2{-radial.Y, radial.X}
	got := math.Abs(math.Atan2(tangent.Y, tangent.X))
	if got > math.Pi/2 {
		got = math.Pi - got
	}
	if math.Abs(got-s.q.spiralAngle) > 1e-9 {
		t.Errorf("%s: the trace meets the cone element at %.12f rad at the mean point, not the "+
			"Mean Spiral Angle %.12f", s.side.label, got, s.q.spiralAngle)
	}

	proofkit.Step(t, "5: the hand mirrors the construction across the cone element and nothing else")
	other := s
	other.handSign = -s.handSign
	other.cy = other.handSign * other.cutter * math.Cos(s.q.spiralAngle)
	if math.Abs(other.cx-s.cx) > tol {
		t.Errorf("%s: flipping the hand moved the cutter centre's along-element coordinate from "+
			"%.9f to %.9f; the hand sign belongs on the cos term, not the sin term",
			s.side.label, s.cx, other.cx)
	}
	if math.Abs(other.cy+s.cy) > tol {
		t.Errorf("%s: flipping the hand took the cutter centre's circumferential coordinate from "+
			"%.9f to %.9f, which is not a mirror across the cone element",
			s.side.label, s.cy, other.cy)
	}

	proofkit.Step(t, "7: the straight-bevel limit — at psi = 0 the centre is due north of the mean point")
	straightCx := s.rMean - s.cutter*math.Sin(0)
	if math.Abs(straightCx-s.rMean) > tol {
		t.Errorf("%s: at psi = 0 the cutter centre's along-element coordinate would be %.9f, not "+
			"R_mean %.9f, so the arc would not be tangent to the element", s.side.label,
			straightCx, s.rMean)
	}

	proofkit.Step(t, "the twist the crown-gear law derives from this arc")
	assertTwistLaw(t, s)
}

// assertTwistLaw checks §3a step G's analytic twist against the arc just drawn.
//
// Three things are pinned. The developed azimuth phi_crown is the angle the
// arc's two ends subtend AT THE APEX, which is what makes it a crown-plane
// quantity rather than an arc length. The shaft-axis twist is that angle divided
// by sin(gamma) for this gear's PITCH cone angle — not the root cone angle
// acos(coneVec . axisDir), which is smaller and would inflate the twist. And a
// meshing pair's two members legitimately get different twists, in the ratio of
// their sines, which is why an equal-teeth pair meshes under a method that gets
// the factor wrong while a ratio pair does not.
func assertTwistLaw(t testing.TB, s spiral) {
	t.Helper()
	if s.total <= 0 {
		t.Fatalf("%s: the toe-to-heel twist is %.9f; a spiral tooth turns", s.side.label, s.total)
	}
	rootConeAngle := math.Acos(s.coneVec.X)
	if math.Abs(rootConeAngle-s.side.gamma) < 1e-9 {
		t.Errorf("%s: the root cone angle %.9f and the pitch cone angle %.9f are the same here, so "+
			"this case cannot show that using the wrong one inflates the twist",
			s.side.label, rootConeAngle, s.side.gamma)
	}
	if rootConeAngle >= s.side.gamma {
		t.Errorf("%s: the root cone angle %.9f is not below the pitch cone angle %.9f; the root "+
			"cone lies one dedendum inside the pitch cone", s.side.label, rootConeAngle, s.side.gamma)
	}
	wrong := math.Abs(s.phiCrown) / math.Sin(rootConeAngle)
	if wrong <= s.total {
		t.Errorf("%s: keying the twist on the root cone angle would give %.9f against the pitch "+
			"cone's %.9f; the wrong angle is supposed to come out larger", s.side.label, wrong, s.total)
	}
}

// stepTwistTooth applies §3a step G's twist to the tooth.
//
// SUBSTITUTION, and it stands in for the whole slice-twist-crown-loft chain,
// because decad has neither a multi-section loft nor a scale. Instead of slicing
// the straight tooth into eight slabs, rotating each by its linear share and
// lofting through their heel faces, the proof lofts ONE band of the tooth
// directly between its toe and heel sections, each already turned by the share
// step G gives it. The two ends therefore carry the same total twist, in the same
// sense, that the sliced build's outermost slabs do.
//
// What the substitution costs is everything that lives between those two ends:
// the per-slab discretisation, the retry-with-the-opposite-sign guard on the
// slice, the scrap drop, and the crown. Those are [PROSE] steps, and each says so
// in the step list next to this one.
//
// The slice planes are also not quite the build's. Step E cuts perpendicular to
// the cone element; the sections here sit on planes parallel to the tooth plane,
// which is perpendicular to the PITCH line instead. The two differ by the
// dedendum angle atan(1.25 * module / R) — under two degrees on every case in
// this table — and the twist law the step asserts does not depend on which of
// them the band is cut by.
//
// <!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepTwistTooth, assertTwistTooth) -->
func stepTwistTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s := spiralOf(t, params)
	q, side := s.q, s.side
	virtual := side.virtualTeeth(q.module)
	d := involute.Derive(q.module, virtual, proxyPressureAngle)
	outline := toothSectionPoints(t, d, virtual, proxyInvoluteSteps)
	f := toothFrameOf(q, side)

	world := sketch.NewWorld()
	toeSketch, toeProfile := f.twistedSection(t, world, outline, s.scaleAt(s.rToe), s.angleAt(s.rToe))
	heelSketch, heelProfile := f.twistedSection(t, world, outline, s.scaleAt(s.rHeel), s.angleAt(s.rHeel))

	body, err := doc.Loft(toeSketch, toeProfile, heelSketch, heelProfile)
	if err != nil {
		t.Fatalf("%s: loft the twisted toe section to the twisted heel section: %v", side.label, err)
	}
	return []*decad.Body{body}
}

// scaleAt is the tooth-plane-parallel section's scale at a cone distance: the
// point on the root cone element at that distance, projected onto the tooth
// plane's normal, over the plane's own distance R from the apex.
func (s spiral) scaleAt(coneDistance float64) float64 {
	normal := vec2{math.Cos(s.side.gamma), math.Sin(s.side.gamma)}
	return s.coneVec.scale(coneDistance).dot(normal) / s.q.pitchConeDist
}

// angleAt is §3a step G's linear share of the total twist for a section at this
// cone distance, centred on R_mean so the mid-face section stays unrotated.
func (s spiral) angleAt(coneDistance float64) float64 {
	return -s.handSign * s.total * (s.rMean - coneDistance) / s.span
}

// twistedSection draws a scaled tooth section on a plane parallel to the tooth
// plane and turned about the shaft axis by angle.
func (f toothFrame) twistedSection(t *testing.T, world *sketch.World, outline []vec2,
	scale, angle float64) (*sketch.Sketch, *sketch.Profile) {
	return f.placedSection(t, world, outline, scale, angle, 0, r3.NewVec(0, 0, 0), 1)
}

// placedSection draws one tooth section, scaled about the apex by scale, turned
// about the shaft axis by angle, uniformly scaled about base by crown, and slid
// along the shaft axis by slide so several sections can be judged together.
//
// The crown scale is the one scaleFeatures would apply and decad has no feature
// for: a uniform scale about a point carries a plane to a parallel plane and a
// section to a scaled copy of itself, so it is drawn rather than performed.
func (f toothFrame) placedSection(t *testing.T, world *sketch.World, outline []vec2,
	scale, angle, slide float64, base r3.Vec, crown float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), radiansOf(angle))
	if err != nil {
		t.Fatalf("twist rotation of %.9f rad: %v", angle, err)
	}
	origin := f.origin.Scale(scale)
	origin = base.Add(origin.Sub(base).Scale(crown))
	origin = turn.Apply(origin).Add(r3.NewVec(slide, 0, 0))
	frame, err := r3.NewFrame(origin, turn.ApplyDir(f.u), turn.ApplyDir(f.v))
	if err != nil {
		t.Fatalf("twisted tooth plane frame: %v", err)
	}
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("twisted tooth plane: %v", err)
	}
	sk, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("twisted tooth section sketch: %v", err)
	}
	pts := make([]*sketch.Point, len(outline))
	for i, p := range outline {
		pts[i] = sk.CreatePoint(p.X*scale*crown, p.Y*scale*crown)
	}
	for i := range pts {
		sk.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	profiles := sk.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		t.Fatalf("the tooth section closes %d regions, want one extrudable loop", len(profiles))
	}
	return sk, profiles[0]
}

// assertTwistTooth measures the twist the two ends carry.
//
// The two section angles have to be symmetric about zero, which is what "centred
// on R_mean so the mid-face section stays unrotated" means and what the pinion's
// zero mesh nudge depends on. Their difference has to be the whole toe-to-heel
// twist, with the hand's sign on it. And the body's centroid has to sit at the
// mean of the two, which is how the twist is read back off the solid rather than
// off the numbers that built it.
func assertTwistTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s := spiralOf(t, params)
	if len(bodies) != 1 {
		t.Fatalf("%s: the twist left %d bodies, want the one tooth band", s.side.label, len(bodies))
	}
	toe := s.angleAt(s.rToe)
	heel := s.angleAt(s.rHeel)

	if math.Abs(toe+heel) > 1e-9 {
		t.Errorf("%s: the toe end turns %.9f rad and the heel end %.9f; they are supposed to be "+
			"symmetric about the unrotated mid-face section", s.side.label, toe, heel)
	}
	if got, want := heel-toe, s.handSign*s.total; math.Abs(got-want) > 1e-9 {
		t.Errorf("%s: the toe-to-heel twist measures %.9f rad, not the crown-gear law's %.9f with "+
			"the hand's sign on it", s.side.label, got, want)
	}
	if math.Abs(s.angleAt(s.rMean)) > 1e-12 {
		t.Errorf("%s: the mid-face section turns %.9f rad; it is the section that meshes exactly "+
			"like the straight tooth and must not move", s.side.label, s.angleAt(s.rMean))
	}

	assertBandCentroid(t, s, bodies[0], params)
}

// assertBandCentroid reads the twist back off the solid rather than off the
// numbers that built it.
//
// The band tapers outward, so its centroid is weighted toward the heel end and
// sits on that end's side of the unrotated mid face, strictly inside the band's
// own angular span. And flipping the Hand of Spiral has to mirror it exactly:
// same magnitude, opposite sign, nothing else changed. That mirror is invariant
// 5 of spiral-tooth-trace.md read off the finished solid, and it is what fails
// when the hand sign is put on the sin term instead of the cos term.
func assertBandCentroid(t *testing.T, s spiral, band *decad.Body, params map[string]float64) {
	t.Helper()
	got := azimuthOf(t, band)
	heel := s.angleAt(s.rHeel)
	if got*heel <= 0 {
		t.Errorf("%s: the band's centroid sits at %.9f rad while its heel end turns %.9f; a band "+
			"that tapers outward is weighted toward the heel", s.side.label, got, heel)
	}
	if math.Abs(got) >= math.Abs(heel) {
		t.Errorf("%s: the band's centroid sits at %.9f rad, outside its own toe-to-heel span of "+
			"+/-%.9f", s.side.label, got, math.Abs(heel))
	}

	mirrored := mirroredBandAzimuth(t, params)
	if mirrored*got >= 0 {
		t.Errorf("%s: both hands put the band's centroid on the same side, at %.9f and %.9f rad; "+
			"the hand is what decides which way the tooth leans", s.side.label, mirrored, got)
	}
	// The two hands are exact mirrors of each other analytically, and the trace
	// step asserts that on the cutter centre. They are only approximate mirrors as
	// SOLIDS, because decad rules each wall between a pair of segments as two
	// triangles and picks that diagonal relative to the FROM section, which is not
	// a mirror-symmetric choice. Measured across this table the two magnitudes
	// agree to 0.3 per cent at a 5 degree spiral angle and to 10 per cent at 59
	// degrees, growing with the twist, so the bound below is on the faceting
	// rather than on the geometry.
	if d := math.Abs(math.Abs(mirrored)-math.Abs(got)) / math.Abs(got); d > 0.12 {
		t.Errorf("%s: the opposite hand puts the band's centroid at %.9f rad against this hand's "+
			"%.9f, %.1f per cent apart; the two hands mirror to within the ruled walls' own "+
			"asymmetry", s.side.label, mirrored, got, 100*d)
	}
}

// azimuthOf is a body's centroid azimuth about the shaft axis, which is the
// axial frame's x axis.
func azimuthOf(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("band centroid: %v", err)
	}
	return math.Atan2(centroid.Value.Z, centroid.Value.Y)
}

// mirroredBandAzimuth builds the same band with the Hand of Spiral flipped and
// returns its centroid azimuth.
func mirroredBandAzimuth(t *testing.T, params map[string]float64) float64 {
	t.Helper()
	doc := decad.New()
	bodies := stepTwistTooth(t, doc, with(params, pHandSign, -params[pHandSign]))
	return azimuthOf(t, bodies[0])
}
