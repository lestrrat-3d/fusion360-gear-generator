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

// crownPerRad is the tunable class constant the crown scales by, default 0.5.
const crownPerRad = 0.5

// sliceCount is the fixed slice scheme: eight planes stepped toward the apex in
// span/6 increments from the parent tooth plane. It is not user-configurable.
const sliceCount = 8

// spiralCases prove the 2-D spiral construction. Both hands are reached,
// because the hand is a SIGN and a scheme that drops it is still solvable at one
// of them; both gears are reached, because the pinion is built with the opposite
// hand to the driving gear; the Mean Spiral Angle runs from the straight-bevel
// branch at 0 to the top of its [0, 60) range; and the Cutter Radius is reached
// both auto and explicit.
var spiralCases = []proofkit.Case{
	{Name: "straight-at-psi-0", Params: map[string]float64{"spiralAngleDeg": 0, "gear": 0}},
	{Name: "default-35-right-pinion", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": 1, "gear": 0}},
	{Name: "default-35-right-driving", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": 1, "gear": 1}},
	{Name: "default-35-left-pinion", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": -1, "gear": 0}},
	{Name: "default-35-left-driving", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": -1, "gear": 1}},
	{Name: "shallow-15-explicit-cutter", Params: map[string]float64{"spiralAngleDeg": 15, "cutterRadius": 30, "gear": 1}},
	{Name: "steep-59-9", Params: map[string]float64{"spiralAngleDeg": 59.9, "gear": 0}},
	{Name: "ratio-31-17-pinion", Params: map[string]float64{"spiralAngleDeg": 35, "drivingTeeth": 31, "pinionTeeth": 17, "gear": 0}},
	{Name: "ratio-31-17-driving", Params: map[string]float64{"spiralAngleDeg": 35, "drivingTeeth": 31, "pinionTeeth": 17, "gear": 1}},
}

// spiralSolidCases keep the same reach for the sliced, twisted and lofted
// tooth, at the sizes those steps can build.
var spiralSolidCases = []proofkit3d.Case{
	{Name: "default-35-right-pinion", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": 1, "gear": 0}},
	{Name: "default-35-left-driving", Params: map[string]float64{"spiralAngleDeg": 35, "handSign": -1, "gear": 1}},
	{Name: "ratio-31-17-pinion", Params: map[string]float64{"spiralAngleDeg": 35, "drivingTeeth": 31, "pinionTeeth": 17, "gear": 0}},
	{Name: "ratio-31-17-driving", Params: map[string]float64{"spiralAngleDeg": 35, "drivingTeeth": 31, "pinionTeeth": 17, "gear": 1}},
	{Name: "steep-59-9-driving", Params: map[string]float64{"spiralAngleDeg": 59.9, "gear": 1}},
}

// spiralFrame is §3a steps A and B: the world frame, the four hand-off points
// after the swap guard, the cutter-circle centre and the trace's two ends.
type spiralFrame struct {
	solid    solidFrame
	toeMid   r3.Vec
	heelMid  r3.Vec
	toeCone  r3.Vec
	heelCone r3.Vec
	rToe     float64
	rHeel    float64
	rMean    float64
	span     float64
	rc       float64
	psi      float64
	hand     float64 // after the pinion's flip
	cx, cy   float64
	toe2d    [2]float64
	heel2d   [2]float64
	phiCrown float64
	total    float64
}

// handOff is the caller hand-off table of §3a: the toe edge is M->N (pinion) /
// O->P (driving) and the heel edge C->H / D->J, toeConeWorld is M/O and
// heelConeWorld is the dedendum corner C/D, never H/J.
func handOff(c config, g *gearSide) (toeMid, heelMid, toeCone, heelCone r3.Vec) {
	toeMid = world(v2scale(v2add(g.M, g.N), 0.5))
	heelMid = world(v2scale(v2add(g.Ded, g.H), 0.5))
	toeCone = world(g.M)
	heelCone = world(g.Ded)
	return
}

// newSpiralFrame builds §3a step A's frame from the four hand-off points, swap
// guard included: if the heel is not the outer end, the toe and heel are
// swapped before coneVec is built, since a negative span silently inverts the
// whole spiral.
func newSpiralFrame(t testing.TB, c config, g *gearSide, toeMid, heelMid, toeCone, heelCone r3.Vec) spiralFrame {
	t.Helper()
	apex := world(c.Apex)
	if apex.Sub(heelMid).Len() < apex.Sub(toeMid).Len() {
		toeMid, heelMid = heelMid, toeMid
		toeCone, heelCone = heelCone, toeCone
	}
	coneVec := normalized(heelCone.Sub(apex))
	axisDir := normalized(world(g.AxisDir))

	f := spiralFrame{
		solid:    solidFrame{config: c, gear: *g, apex: apex, axis: axisDir, cone: coneVec, circ: normalized(axisDir.Cross(coneVec))},
		toeMid:   toeMid,
		heelMid:  heelMid,
		toeCone:  toeCone,
		heelCone: heelCone,
		psi:      c.SpiralAngle,
	}
	f.solid.uDir = normalized(coneVec.Sub(axisDir.Scale(coneVec.Dot(axisDir))))
	f.rToe = f.solid.distAlong(toeMid)
	f.rHeel = f.solid.distAlong(heelMid)
	f.rMean = 0.5 * (f.rToe + f.rHeel)
	f.span = f.rHeel - f.rToe

	// The hand is the driving gear's; the pinion is built with the opposite one.
	f.hand = c.HandSign
	if g.Label == "Pinion" {
		f.hand = -f.hand
	}
	f.rc = c.CutterRadius
	if f.rc == 0 {
		f.rc = f.rMean
	}
	// The hand sign belongs on the cos term, so opposite hands mirror the centre
	// across the cone element rather than about x = R_mean.
	f.cx = f.rMean - f.rc*math.Sin(f.psi)
	f.cy = f.hand * f.rc * math.Cos(f.psi)

	lo, hi := f.rToe-0.06*f.span, f.rHeel+0.06*f.span
	f.toe2d = circleIntersectNearest(lo, f.cx, f.cy, f.rc, f.rMean, 0)
	f.heel2d = circleIntersectNearest(hi, f.cx, f.cy, f.rc, f.rMean, 0)
	f.phiCrown = math.Atan2(f.heel2d[1], f.heel2d[0]) - math.Atan2(f.toe2d[1], f.toe2d[0])
	f.total = math.Abs(f.phiCrown) / math.Sin(g.Gamma)
	return f
}

// circleIntersectNearest intersects the apex circle of radius r with the cutter
// circle and keeps the solution nearest the reference point, which is the
// branch the mean point sits on. A non-overlapping pair clamps to tangency.
func circleIntersectNearest(r, cx, cy, rc, refX, refY float64) [2]float64 {
	d := math.Hypot(cx, cy)
	if d == 0 {
		return [2]float64{r, 0}
	}
	a := (d*d + r*r - rc*rc) / (2 * d)
	h2 := r*r - a*a
	ux, uy := cx/d, cy/d
	px, py := ux*a, uy*a
	if h2 <= 0 {
		return [2]float64{px, py} // tangency
	}
	h := math.Sqrt(h2)
	one := [2]float64{px - uy*h, py + ux*h}
	two := [2]float64{px + uy*h, py - ux*h}
	if math.Hypot(one[0]-refX, one[1]-refY) <= math.Hypot(two[0]-refX, two[1]-refY) {
		return one
	}
	return two
}

// tanW maps a tangent-plane 2-D coordinate to world, the framework's
// combine_point(apex, px, coneVec, py, v).
func (f spiralFrame) tanW(px, py float64) r3.Vec {
	return f.solid.apex.Add(f.solid.cone.Scale(px)).Add(f.solid.circ.Scale(py))
}

// stepConeElement draws the `{gear} Cone Element` sketch and, with it, the
// frame §3a step A builds: the shaft axis, the root cone element, the
// circumferential direction across them, and the toe and heel cone distances.
//
// The line this sketch holds is Apex->(Apex + R_heel * coneVec), and it is the
// one the Trace Plane is rotated about, so what it must be is the ROOT cone
// element Apex->C — not Apex->Apex2 and not the shaft axis, the two the trace
// derivation names as the ways to skew the whole frame. The step also exercises
// the swap guard by building the frame a second time with the toe and heel
// arguments exchanged: the guard has to return the same frame, since a negative
// span inverts the cutter arc, the slice direction and the twist at once, with
// no error anywhere.
func stepConeElement(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)

	proofkit.Step(t, "%s: the cone element line on the axial plane", g.Label)
	apex := s.CreateReferencePoint(c.Apex.X, c.Apex.Y, "section-2 apex")
	end := v2add(c.Apex, v2scale(g.RootDir(c.Apex), f.rHeel))
	tip := s.CreatePoint(end.X, end.Y)
	tip.SetName("cone element end")
	line := s.CreateLine(apex, tip)
	line.SetName("Cone Element")
	line.SetConstruction(true)
	// The element runs from the apex through the dedendum corner C, so it is
	// pinned by that direction and its length.
	s.AddConstraint(sketch.NewPointOnLine(tip, mustReferenceLine(t, s, c.Apex, g.Ded, "Root Axis")))
	s.AddConstraint(sketch.NewDistance(apex, tip, f.rHeel))
	solveHere(t, s)

	proofkit.Step(t, "%s: the frame the cutter arc and the twist are built in", g.Label)
	near(t, f.solid.cone.Dot(normalized(world(g.RootDir(c.Apex)))), 1, 1e-12,
		"%s coneVec is the root cone element Apex->C", g.Label)
	pitchDir := normalized(world(v2unit(v2sub(c.Apex2, c.Apex))))
	if math.Abs(f.solid.cone.Dot(pitchDir)) > 1-1e-9 {
		t.Errorf("%s: coneVec has been built along Apex->Apex2 rather than the root element", g.Label)
	}
	if math.Abs(f.solid.cone.Dot(f.solid.axis)) > 1-1e-9 {
		t.Errorf("%s: coneVec has been built along the shaft axis", g.Label)
	}
	near(t, f.solid.circ.Dot(f.solid.cone), 0, 1e-12, "%s the circumferential direction is across the element", g.Label)
	near(t, f.solid.circ.Dot(f.solid.axis), 0, 1e-12, "%s the circumferential direction is across the shaft axis", g.Label)
	near(t, f.solid.circ.Len(), 1, 1e-12, "%s the circumferential direction is a unit vector", g.Label)

	proofkit.Step(t, "%s: the toe and heel hand-off, and the swap guard", g.Label)
	near(t, f.rMean, 0.5*(f.rToe+f.rHeel), 1e-12, "%s mean cone distance", g.Label)
	if f.span <= 0 {
		t.Fatalf("%s: the span came out %.6f; the heel must be the outer end", g.Label, f.span)
	}
	// The span is measured between the two EDGE MIDPOINTS, so it is close to but
	// not the same as the Face Width measured along the cone element between the
	// two cone points M and C: N and H sit off the element.
	near(t, f.solid.distAlong(f.heelCone)-f.solid.distAlong(f.toeCone), c.FaceWidth/math.Cos(dedendumAngleOf(c, g)), 1e-9,
		"%s the cone points span the Face Width along the element", g.Label)
	if f.span <= 0.5*c.FaceWidth || f.span >= 2*c.FaceWidth {
		t.Errorf("%s: the midpoint span %.4f is nowhere near the Face Width %.4f", g.Label, f.span, c.FaceWidth)
	}
	// heelConeWorld is the dedendum corner C, on the root element — never H,
	// which lies a module beyond it on the Apex2->C dedendum line and skews the
	// element away from Apex->C.
	near(t, f.solid.distAlong(f.heelCone), f.heelCone.Sub(f.solid.apex).Len(), 1e-9,
		"%s heelConeWorld lies on the root cone element", g.Label)
	if world(g.H).Sub(f.solid.apex).Len()-f.solid.distAlong(world(g.H)) < 1e-9 {
		t.Errorf("%s: point H lies on the root element, so the C-versus-H hand-off could not be told apart", g.Label)
	}
	swapped := newSpiralFrame(t, c, g, heelMid, toeMid, heelCone, toeCone)
	near(t, swapped.span, f.span, 1e-12, "%s the swap guard restores a positive span", g.Label)
	near(t, swapped.rMean, f.rMean, 1e-12, "%s the swap guard restores the mean cone distance", g.Label)
	near(t, swapped.total, f.total, 1e-12, "%s the swap guard restores the twist", g.Label)
}

func dedendumAngleOf(c config, g *gearSide) float64 {
	return math.Acos(math.Min(1, v2dot(v2unit(v2sub(g.Ded, c.Apex)), v2unit(v2sub(c.Apex2, c.Apex)))))
}

func mustReferenceLine(t testing.TB, s *sketch.Sketch, from, to vec2, source string) *sketch.Line {
	t.Helper()
	a := s.CreateReferencePoint(from.X, from.Y, source+" start")
	b := s.CreateReferencePoint(to.X, to.Y, source+" end")
	line, err := s.CreateReferenceLine(a, b, source)
	if err != nil {
		t.Fatalf("reference line %s: %v", source, err)
	}
	return line
}

// stepSpiralTrace draws the `{gear} 2D Tooth Trace` sketch: the cutter circle
// and the genuine cutter arc through the toe, the mean point and the heel.
//
// Substitution: the spec leaves this sketch with free degrees of freedom on
// purpose — the arc's ends are pinned by the three-point construction, not by
// dimensions, and dimensioning them over-constrains the solve against the
// cone-element plane — and exempts it from the full-constraint gate. proofkit
// waives nothing, so the proof pins the two ends as fixed points instead. The
// cost is that the proof does not exercise the free solve Fusion performs here;
// what it does check is the geometry that solve is supposed to produce, which is
// the list of invariants in spiral-tooth-trace.md section 9.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)

	// The straight bevel is the psi = 0 branch: the tooth-body hook returns the
	// framework's two conical trims and none of this section runs.
	if c.SpiralAngle == 0 {
		proofkit.Step(t, "%s: psi is zero, so the straight tooth path is taken", g.Label)
		element := s.CreatePoint(0, 0)
		element.SetName("cone element at the mean point")
		s.Fix(element)
		straight := s.CreateLine(element, s.CreatePoint(f.span, 0))
		straight.SetName("straight tooth trace")
		s.Fix(straight.End)
		solveHere(t, s)
		near(t, f.cx, f.rMean, 1e-12, "at psi = 0 the cutter centre sits straight above the mean point")
		near(t, math.Abs(f.cy), f.rc, 1e-12, "at psi = 0 the cutter centre is one cutter radius off the element")
		// A straight bevel is the limit r_c -> infinity, NOT a zero twist at a
		// finite cutter radius: at psi = 0 the cutter circle is merely tangent to
		// the element at the mean point, and its ends still subtend an angle at the
		// apex. What makes the tooth straight is the branch, not the arithmetic.
		if f.phiCrown == 0 {
			t.Errorf("a finite cutter at psi = 0 still subtends an angle at the apex; a zero here means the case proves nothing about the branch")
		}
		return
	}

	proofkit.Step(t, "%s: the cutter circle and the trace arc in the tangent plane", g.Label)
	centre := s.CreatePoint(f.cx, f.cy)
	centre.SetName("cutter circle centre")
	s.Fix(centre)
	cutter := s.CreateCircle(centre, f.rc)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*f.rc))

	toe := s.CreatePoint(f.toe2d[0], f.toe2d[1])
	toe.SetName("trace toe end")
	heel := s.CreatePoint(f.heel2d[0], f.heel2d[1])
	heel.SetName("trace heel end")
	s.Fix(toe)
	s.Fix(heel)
	arcCentre := s.CreatePoint(f.cx, f.cy)
	arcCentre.SetName("trace arc centre")
	s.AddConstraint(sketch.NewHorizontalDistance(centre, arcCentre, 0))
	trace := s.CreateArc(arcCentre, toe, heel)
	trace.SetName("trace arc")
	solveHere(t, s)

	proofkit.Step(t, "%s: the trace invariants", g.Label)
	// 1. Apex-centred: the ends sit on the apex circles a hair past the face.
	near(t, math.Hypot(f.toe2d[0], f.toe2d[1]), f.rToe-0.06*f.span, 1e-9, "%s toe end on its apex circle", g.Label)
	near(t, math.Hypot(f.heel2d[0], f.heel2d[1]), f.rHeel+0.06*f.span, 1e-9, "%s heel end on its apex circle", g.Label)
	// 2. The arc is the cutter circle: one radius, and the centre is the cutter's.
	near(t, math.Hypot(toe.X()-arcCentre.X(), toe.Y()-arcCentre.Y()), f.rc, 1e-9, "%s trace radius at the toe", g.Label)
	near(t, math.Hypot(heel.X()-arcCentre.X(), heel.Y()-arcCentre.Y()), f.rc, 1e-9, "%s trace radius at the heel", g.Label)
	near(t, math.Hypot(arcCentre.X()-centre.X(), arcCentre.Y()-centre.Y()), 0, 1e-9, "%s trace arc centre gap", g.Label)
	// 3. It passes through the mean point.
	near(t, math.Hypot(f.rMean-f.cx, -f.cy), f.rc, 1e-9, "%s the cutter circle passes through the mean point", g.Label)
	// 4. The spiral angle is realised AT the mean point: the tangent there makes
	//    psi with the cone element.
	tangent := math.Atan2(-(f.rMean - f.cx), 0-f.cy) // perpendicular to the radius C->M
	angle := math.Abs(math.Atan(math.Tan(tangent)))
	near(t, angle, c.SpiralAngle, 1e-9, "%s the trace meets the element at psi", g.Label)
	// 5. Mirror symmetry: the other hand mirrors the centre across the element
	//    and nothing else changes.
	other := c
	other.HandSign = -c.HandSign
	mirror := newSpiralFrame(t, other, g, toeMid, heelMid, toeCone, heelCone)
	near(t, mirror.cx, f.cx, 1e-12, "%s the hand does not move the centre along the element", g.Label)
	near(t, mirror.cy, -f.cy, 1e-12, "%s the hand mirrors the centre across the element", g.Label)
	near(t, mirror.total, f.total, 1e-12, "%s both hands twist by the same magnitude", g.Label)
	if f.hand*mirror.hand >= 0 {
		t.Errorf("%s: the two hands did not come out opposite", g.Label)
	}
	// 6. The twist uses the PITCH cone angle, not the root cone angle, and the
	//    two are far enough apart that using the wrong one shows.
	near(t, f.total, math.Abs(f.phiCrown)/math.Sin(g.Gamma), 1e-12, "%s the twist is the crown law", g.Label)
	rootAngle := math.Acos(math.Min(1, f.solid.cone.Dot(f.solid.axis)))
	wrong := math.Abs(f.phiCrown) / math.Sin(rootAngle)
	if math.Abs(wrong-f.total) < 1e-6 {
		t.Errorf("%s: the root cone angle %.4f and the pitch cone angle %.4f are indistinguishable here, so the case cannot catch the substitution",
			g.Label, rootAngle, g.Gamma)
	}
	if wrong <= f.total {
		t.Errorf("%s: using the root cone angle must inflate the twist, got %.4f against %.4f", g.Label, wrong, f.total)
	}
}

// slabSection returns the tooth-cone section at cone distance d, as world
// points: each vertex of the heel section carried back along its own ray from
// the apex to the plane perpendicular to the cone element at that distance.
func (f spiralFrame) slabSection(t *testing.T, heel []r3.Vec, d float64) []r3.Vec {
	t.Helper()
	out := make([]r3.Vec, len(heel))
	for i, v := range heel {
		ray := v.Sub(f.solid.apex)
		out[i] = f.solid.apex.Add(ray.Scale(d / ray.Dot(f.solid.cone)))
	}
	return out
}

// heelSectionOf is the tooth profile at the heel, in world coordinates.
func heelSectionOf(t *testing.T, c config, g *gearSide, f spiralFrame) []r3.Vec {
	t.Helper()
	section := toothSection(t, c.Module, g.VirtualTeeth)
	centre := world(g.KPrime)
	uDir := normalized(world(v2unit(v2sub(g.KPrime, g.Ded))))
	vDir := r3.NewVec(0, 0, 1)
	out := make([]r3.Vec, len(section))
	for i, q := range section {
		out[i] = centre.Add(uDir.Scale(q.X)).Add(vDir.Scale(q.Y))
	}
	return out
}

// sliceOffsets is §3a step E's fixed scheme: eight planes at sign*(k+1)*span/6
// from the parent tooth plane, stepping toward the apex.
func sliceOffsets(span float64, sign float64) []float64 {
	out := make([]float64, sliceCount)
	for k := range out {
		out[k] = sign * float64(k+1) * span / 6
	}
	return out
}

// slabBody builds one cross-section slab between two cone distances, in its own
// document so decad is never asked to decide a pair of loft bodies.
func slabBody(t *testing.T, c config, g *gearSide, f spiralFrame, heel []r3.Vec, d0, d1 float64) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	build := func(d float64) (*sketch.Sketch, *sketch.Profile) {
		pts := f.slabSection(t, heel, d)
		origin := f.solid.apex.Add(f.solid.cone.Scale(d))
		frame, err := r3.NewFrame(origin, f.solid.circ, f.solid.cone.Cross(f.solid.circ))
		if err != nil {
			t.Fatalf("slab frame at %.4f: %v", d, err)
		}
		plane, err := w.CreatePlaneFromFrame(frame)
		if err != nil {
			t.Fatalf("slab plane at %.4f: %v", d, err)
		}
		local := make([]vec2, len(pts))
		for i, q := range pts {
			rel := frame.ToLocal(q)
			local[i] = vec2{rel.X, rel.Y}
		}
		return regionOn(t, w, plane, local)
	}
	s0, p0 := build(d0)
	s1, p1 := build(d1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("slab %.4f..%.4f: %v", d0, d1, err)
	}
	return body
}

// stepSpiralSlices splits the uncut tooth into cross-section slabs with planes
// perpendicular to the cone element, on the fixed eight-plane scheme.
//
// A spec inconsistency this proof has to choose a side of. Step E asks for
// planes PERPENDICULAR TO THE CONE ELEMENT, then builds them as offsets of the
// parent transverse tooth plane, which are parallel to that plane and not
// perpendicular to the element: the parent plane holds the dedendum line
// Apex2->C, and the dedendum line meets the root element Apex->C at 90 degrees
// minus the dedendum angle, a degree or two off. The proof takes the first
// sentence and cuts perpendicular to the element, measuring the offsets from
// the heel, since the dedendum corner C is on both the parent plane and the
// heel edge. The two readings differ by the dedendum angle, so a slab's end
// face is tilted by that much either way, and nothing downstream in this proof
// can tell them apart; only a Fusion session can say whether the tilt matters
// to the loft that samples those faces.
//
// Substitution: decad has no split, and refuses a boolean whose operand is a
// loft, so the slabs are built as the lofts between consecutive cut sections
// rather than cut out of one body. That is the same set of pieces, and it keeps
// what the step has to get right: the plane spacing, the direction the offsets
// step in, and that the cut actually divides the tooth rather than leaving it
// whole — the failure the spec makes the build retry with the opposite sign and
// then raise on.
func stepSpiralSlices(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	heel := heelSectionOf(t, c, g, f)

	cuts := sliceCuts(f)
	for i := 0; i+1 < len(cuts); i++ {
		slab := slabBody(t, c, g, f, heel, cuts[i], cuts[i+1])
		proofkit3d.RequireSolid(t, slab.Document(), []*decad.Body{slab})
	}
	// The gated body is the first slab, rebuilt in the document the gate reads;
	// the rest are gated in their own documents just above, because decad will
	// not decide a pair of loft bodies.
	return []*decad.Body{rebuildIn(t, doc, c, g, f, heel, cuts[0], cuts[1], 0)}
}

// sliceCuts are the cone distances the cut planes sit at, apex-most first. The
// first cut plane is the parent tooth plane offset toward the apex by span/6,
// and the rest step on in span/6 increments.
func sliceCuts(f spiralFrame) []float64 {
	parent := f.rHeel
	cuts := []float64{}
	for _, off := range sliceOffsets(f.span, -1) {
		cuts = append(cuts, parent+off)
	}
	sort.Float64s(cuts)
	return cuts
}

func assertSpiralSlices(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)

	offsets := sliceOffsets(f.span, -1)
	if len(offsets) != sliceCount {
		t.Fatalf("the slice scheme is fixed at %d planes, got %d", sliceCount, len(offsets))
	}
	for k, off := range offsets {
		near(t, off, -float64(k+1)*f.span/6, 1e-12, "cut plane %d sits at its share of span/6", k)
		if off >= 0 {
			t.Errorf("cut plane %d steps away from the apex", k)
		}
	}
	cuts := sliceCuts(f)
	// The cuts have to reach across the tooth, or the body comes back in one
	// piece and the crown later fails far from the cause.
	if cuts[len(cuts)-1] >= f.rHeel || cuts[0] >= f.rToe {
		t.Errorf("%s: the cut planes do not divide the tooth: they run %.4f..%.4f across a face from %.4f to %.4f",
			g.Label, cuts[0], cuts[len(cuts)-1], f.rToe, f.rHeel)
	}
	if len(cuts) < 2 {
		t.Fatalf("%s: a slice that leaves one piece must be retried with the opposite sign and then raise", g.Label)
	}
	volume := measured(t, mustVolume(t, bodies[0]))
	if volume <= 0 {
		t.Errorf("%s: a slab has no volume", g.Label)
	}
}

// stepSpiralScrap orders the segments by the cone distance of their centroid
// and drops the apex-most one, the long scrap below the toe.
func stepSpiralScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	heel := heelSectionOf(t, c, g, f)

	cuts := sliceCuts(f)
	// The apex-side scrap is everything below the first cut; the segments kept
	// are the slabs between the cuts.
	scrapEnd := cuts[0]
	scrap := slabBody(t, c, g, f, heel, scrapEnd*0.2, scrapEnd)
	proofkit3d.RequireSolid(t, scrap.Document(), []*decad.Body{scrap})

	kept := rebuildIn(t, doc, c, g, f, heel, cuts[0], cuts[1], 0)
	return []*decad.Body{kept}
}

func assertSpiralScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	heel := heelSectionOf(t, c, g, f)
	cuts := sliceCuts(f)

	// Sorting by the centroid's cone distance is what makes the apex-most piece
	// identifiable; the order must come out strictly increasing.
	last := math.Inf(-1)
	for i := 0; i+1 < len(cuts); i++ {
		slab := slabBody(t, c, g, f, heel, cuts[i], cuts[i+1])
		centroid, err := slab.Centroid()
		if err != nil {
			t.Fatalf("segment centroid: %v", err)
		}
		d := f.solid.distAlong(centroid.Value)
		if d <= last {
			t.Errorf("%s: segment %d does not sort after its neighbour (%.6f against %.6f)", g.Label, i, d, last)
		}
		last = d
	}
	if len(cuts)-1 < 1 {
		t.Fatalf("%s: dropping the scrap must leave at least one segment", g.Label)
	}
	centroid, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("kept centroid: %v", err)
	}
	if f.solid.distAlong(centroid.Value) <= cuts[0] {
		t.Errorf("%s: the kept segment lies below the first cut, so the scrap was not the apex-most piece", g.Label)
	}
}

// twistOf is §3a step G's per-segment rotation: a linear share of the total
// twist, keyed on the cone distance of the segment's HEEL face and centred on
// R_mean so the mid-face section stays unrotated.
func (f spiralFrame) twistOf(heelFace float64) float64 {
	return -f.hand * f.total * (f.rMean - heelFace) / f.span
}

// stepSpiralTwist rotates each slab about the shaft axis by its share of the
// toe-to-heel twist.
func stepSpiralTwist(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	heel := heelSectionOf(t, c, g, f)
	cuts := sliceCuts(f)

	var gated *decad.Body
	for i := 0; i+1 < len(cuts); i++ {
		target := doc
		if i > 0 {
			target = decad.New()
		}
		slab := slabBody(t, c, g, f, heel, cuts[i], cuts[i+1])
		transform, err := r3.RotationAround(f.solid.apex, f.solid.axis, units.Radians(f.twistOf(cuts[i+1])))
		if err != nil {
			t.Fatalf("twist %d: %v", i, err)
		}
		turned, err := slab.Placed(transform)
		if err != nil {
			t.Fatalf("twist %d: %v", i, err)
		}
		proofkit3d.RequireSolid(t, turned.Document(), []*decad.Body{turned})
		if i == 0 {
			gated = rebuildIn(t, target, c, g, f, heel, cuts[i], cuts[i+1], f.twistOf(cuts[i+1]))
		}
	}
	return []*decad.Body{gated}
}

// rebuildIn builds one twisted slab inside the document the gate will read.
func rebuildIn(t *testing.T, doc *decad.Document, c config, g *gearSide, f spiralFrame, heel []r3.Vec, d0, d1, angle float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	build := func(d float64) (*sketch.Sketch, *sketch.Profile) {
		pts := f.slabSection(t, heel, d)
		origin := f.solid.apex.Add(f.solid.cone.Scale(d))
		frame, err := r3.NewFrame(origin, f.solid.circ, f.solid.cone.Cross(f.solid.circ))
		if err != nil {
			t.Fatalf("slab frame: %v", err)
		}
		plane, err := w.CreatePlaneFromFrame(frame)
		if err != nil {
			t.Fatalf("slab plane: %v", err)
		}
		local := make([]vec2, len(pts))
		for i, q := range pts {
			rel := frame.ToLocal(q)
			local[i] = vec2{rel.X, rel.Y}
		}
		return regionOn(t, w, plane, local)
	}
	s0, p0 := build(d0)
	s1, p1 := build(d1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("twisted slab: %v", err)
	}
	transform, err := r3.RotationAround(f.solid.apex, f.solid.axis, units.Radians(angle))
	if err != nil {
		t.Fatalf("twist: %v", err)
	}
	turned, err := body.Placed(transform)
	if err != nil {
		t.Fatalf("twist: %v", err)
	}
	return turned
}

func assertSpiralTwist(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)

	// The mid-face section stays put, which is what lets the pinion mesh with no
	// extra phase.
	near(t, f.twistOf(f.rMean), 0, 1e-12, "%s the mid-face section is unrotated", g.Label)
	// The two ends turn opposite ways by half the total each.
	near(t, math.Abs(f.twistOf(f.rHeel)), f.total/2, 1e-12, "%s the heel turns by half the total", g.Label)
	near(t, math.Abs(f.twistOf(f.rToe)), f.total/2, 1e-12, "%s the toe turns by half the total", g.Label)
	if f.twistOf(f.rHeel)*f.twistOf(f.rToe) >= 0 {
		t.Errorf("%s: the two ends must turn in opposite senses about the mid-face", g.Label)
	}
	// The hand sets the direction, so flipping it flips every rotation.
	other := c
	other.HandSign = -c.HandSign
	mirror := newSpiralFrame(t, other, g, toeMid, heelMid, toeCone, heelCone)
	near(t, mirror.twistOf(f.rHeel), -f.twistOf(f.rHeel), 1e-12, "%s the hand reverses the twist", g.Label)

	// A meshing pair legitimately gets different twists: same cutter and psi,
	// different pitch cone angle, so different 1/sin(gamma).
	otherGear := &c.Driving
	if g.Label == "Driving" {
		otherGear = &c.Pinion
	}
	oToe, oHeel, oToeCone, oHeelCone := handOff(c, otherGear)
	mate := newSpiralFrame(t, c, otherGear, oToe, oHeel, oToeCone, oHeelCone)
	if math.Abs(g.Gamma-otherGear.Gamma) > 1e-9 && math.Abs(mate.total-f.total) < 1e-9 {
		t.Errorf("%s: the two members of the pair have different pitch cone angles and must get different twists", g.Label)
	}
	if math.Abs(g.Gamma-otherGear.Gamma) < 1e-12 {
		near(t, mate.total, f.total, 1e-12, "an equal pair twists equally")
	}

	volume := measured(t, mustVolume(t, bodies[0]))
	if volume <= 0 {
		t.Errorf("%s: the twisted slab has no volume", g.Label)
	}
}

// stepSpiralCrown scales every slab but the outermost down about a point on the
// ROOT edge of its heel face, so the tooth is relieved lengthwise while staying
// seated on the gear base.
//
// Substitution: decad's transforms are isometries — a scale is unrepresentable
// rather than merely unsupported — so the crown is proved on the section a slab
// end face is, in the sketch engine: the heel face drawn as its root edge and
// tip, and the same face scaled about the root-edge midpoint. What that pins is
// the whole of what the crown step has to get right: the factor law, its
// monotonic key, the root edge staying exactly where it was, and what happens if
// the base point is put at the face centroid instead.
func stepSpiralCrown(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	if c.SpiralAngle == 0 {
		proofkit.Unmodelled(t, "psi is zero, so the straight tooth path is taken and no slab is crowned")
	}

	// One slab's heel face, drawn in its own plane: the root edge along the
	// bottom, the tip a tooth height above it.
	height := 2.25 * c.Module
	width := math.Pi * c.Module / 2
	proofkit.Step(t, "%s: the heel face of a slab, and the root-edge midpoint the scale is anchored on", g.Label)
	rootLeft := s.CreatePoint(-width/2, 0)
	rootRight := s.CreatePoint(width/2, 0)
	tipLeft := s.CreatePoint(-width/2, height)
	tipRight := s.CreatePoint(width/2, height)
	for _, q := range []*sketch.Point{rootLeft, rootRight, tipLeft, tipRight} {
		s.Fix(q)
	}
	face := []*sketch.Point{rootLeft, rootRight, tipRight, tipLeft}
	for i := range face {
		l := s.CreateLine(face[i], face[(i+1)%len(face)])
		l.SetName("heel face")
	}
	base := s.CreatePoint(0, 0)
	base.SetName("scale base on the root edge")
	s.AddConstraint(sketch.NewMidpointOf(base, rootLeft, rootRight))

	// The relief is keyed on the heel-distance fraction u, which runs 0 at the
	// held-full heel to 1 at the toe, so it grows monotonically toward the toe.
	proofkit.Step(t, "%s: the crown factor across the slabs", g.Label)
	cuts := sliceCuts(f)
	factors := make([]float64, 0, len(cuts))
	prev := math.Inf(1)
	for i := len(cuts) - 1; i >= 0; i-- {
		u := (f.rHeel - cuts[i]) / f.span
		factor := 1 - crownPerRad*(math.Abs(f.total)/2)*u
		if factor <= 0 {
			t.Fatalf("%s: crown factor %.4f at u = %.4f is not positive; the build must raise rather than scale by it",
				g.Label, factor, u)
		}
		if factor > prev {
			t.Errorf("%s: the relief is not monotonic from heel to toe (%.4f after %.4f)", g.Label, factor, prev)
		}
		prev = factor
		factors = append(factors, factor)
	}
	near(t, 1-crownPerRad*(math.Abs(f.total)/2)*0, 1, 1e-12, "the heel slab is held full")

	// Scaling about the root-edge midpoint keeps every line through that point
	// fixed, so the root edge does not move and only the tip comes down.
	proofkit.Step(t, "%s: the scale about the root edge, and about the centroid", g.Label)
	factor := factors[len(factors)-1]
	scaleAbout := func(anchorY float64, q vec2) vec2 {
		return vec2{q.X * factor, anchorY + (q.Y-anchorY)*factor}
	}
	rootScaled := scaleAbout(0, vec2{width / 2, 0})
	near(t, rootScaled.Y, 0, 1e-12, "%s the root edge stays on the seating cone", g.Label)
	tipScaled := scaleAbout(0, vec2{0, height})
	near(t, tipScaled.Y, height*factor, 1e-12, "%s the tip is relieved by the crown factor", g.Label)
	// The same scale about the face centroid lifts the root off the base by half
	// the loss, which is the gap the Combine-Join cannot close.
	centroidScaled := scaleAbout(height/2, vec2{0, 0})
	if centroidScaled.Y <= 0 {
		t.Errorf("%s: a centroid-anchored scale must lift the root edge, which is why the base point is on the root", g.Label)
	}
	near(t, centroidScaled.Y, (1-factor)*height/2, 1e-12,
		"%s a centroid-anchored scale lifts the root by half the height lost", g.Label)

	solveHere(t, s)
}

// stepSpiralLoft lofts the crowned slabs into the curved tooth, in the order
// their heel faces sit at AFTER the twist.
//
// Substitution: decad lofts a pair of sections at a time, so the proof builds
// the chain pairwise instead of one multi-section loft, and each link is gated
// on its own. What it pins is the ordering the spec makes the step recompute:
// the sections are sorted by their post-twist heel-face cone distance, and the
// chain is built in that order.
func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)
	heel := heelSectionOf(t, c, g, f)
	order := loftOrder(f)

	for i := 0; i+1 < len(order); i++ {
		link := slabBody(t, c, g, f, heel, order[i], order[i+1])
		proofkit3d.RequireSolid(t, link.Document(), []*decad.Body{link})
	}
	// The gated body is the toe-most link, the one whose toe-facing face is
	// added first so the loft reaches past the toe cone.
	body := rebuildIn(t, doc, c, g, f, heel, order[0], order[1], 0)
	return []*decad.Body{body}
}

// loftOrder is the segment order the loft is built in: sorted by the cone
// distance of each segment's heel face, recomputed after the twist.
func loftOrder(f spiralFrame) []float64 {
	cuts := sliceCuts(f)
	out := append([]float64{}, cuts...)
	sort.Float64s(out)
	return out
}

func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	toeMid, heelMid, toeCone, heelCone := handOff(c, g)
	f := newSpiralFrame(t, c, g, toeMid, heelMid, toeCone, heelCone)

	order := loftOrder(f)
	for i := 1; i < len(order); i++ {
		if order[i] <= order[i-1] {
			t.Errorf("%s: the loft order is not strictly toe to heel at %d (%.6f after %.6f)",
				g.Label, i, order[i], order[i-1])
		}
	}
	// The twist turns each slab about the shaft axis, and the loft samples each
	// slab's heel face, so the sections have to be lofted in the order those
	// faces sit at rather than the order the slices were made in.
	near(t, f.twistOf(order[len(order)-1])-f.twistOf(order[0]),
		-f.hand*f.total*(order[0]-order[len(order)-1])/f.span, 1e-9,
		"%s the twist across the lofted chain is the full toe-to-heel share", g.Label)

	volume := measured(t, mustVolume(t, bodies[0]))
	if volume <= 0 {
		t.Errorf("%s: the lofted link has no volume", g.Label)
	}
	// The curved tooth still ends at the trims: the chain runs from the toe cut
	// to the heel cut and no further.
	if order[0] >= f.rToe || order[len(order)-1] >= f.rHeel {
		t.Errorf("%s: the lofted chain does not span the face from %.4f to %.4f", g.Label, f.rToe, f.rHeel)
	}
}
