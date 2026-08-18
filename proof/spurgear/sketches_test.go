package spurgear_test

// The three sketch steps of the spur build, proved in the sketch engine.
//
// Each one is a single Fusion timeline entry: one sketch, however much geometry
// goes into it. The Gear Profile step carries the whole constraint scheme the
// spec's [SPUR-F-…] recipes describe, and it is the one that has to reach DOF 0
// with no redundant constraint, no ambiguity and no near-singular system across
// the regime the spec declares.

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// profileCases is the regime the Gear Profile scheme has to hold across.
//
// Size, the whole signed range of the angle argument, the rib count and both
// routes into the embedded shape are each swept, because the spec says the
// scheme fails differently outside each of them.
var profileCases = []proofkit.Case{
	// Size: the default gear, a fine one and a coarse one.
	{Name: "m1_t17_pa20_a0_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m0.5_t12_pa20_a0_s15", Params: map[string]float64{
		"module": 0.5, "toothNumber": 12, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m5_t24_pa20_a0_s15", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m1_t17_pa14.5_a0_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": 14.5 * math.Pi / 180, "angle": 0, "involuteSteps": 15}},

	// The signed angle range: a right-hand helix, the left-hand helix that is
	// the same magnitude negated, both quarter turns where |sin| > |cos| swaps
	// which axis the rib and chain dimensions take, and the bevel virtual
	// tooth's half turn. A scheme that drops the confirming dimension's sign
	// still solves at +angle and comes out mirrored at -angle.
	{Name: "m1_t17_pa20_a+30_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": math.Pi / 6, "involuteSteps": 15}},
	{Name: "m1_t17_pa20_a-30_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": -math.Pi / 6, "involuteSteps": 15}},
	{Name: "m1_t17_pa20_a+90_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": math.Pi / 2, "involuteSteps": 15}},
	{Name: "m1_t17_pa20_a-90_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": -math.Pi / 2, "involuteSteps": 15}},
	{Name: "m1_t17_pa20_a180_s15", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": math.Pi, "involuteSteps": 15}},

	// The rib count: one rib, one across-spine dimension and one chain
	// dimension per involute sample, so the low end of the count is where a
	// single missing or redundant dimension is a large fraction of the system.
	{Name: "m1_t17_pa20_a0_s3", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 3}},
	{Name: "m1_t17_pa20_a-90_s3", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": -math.Pi / 2, "involuteSteps": 3}},

	// Both routes into the embedded shape, and both sides of each threshold.
	// A high tooth count carries it at the ordinary 20 degrees (the branch
	// turns over at 41.5 teeth); a large pressure angle carries it at a
	// moderate tooth count (26.7 teeth at 25 degrees).
	{Name: "m1_t41_pa20_a0_s15_stub", Params: map[string]float64{
		"module": 1, "toothNumber": 41, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m1_t42_pa20_a0_s15_embedded", Params: map[string]float64{
		"module": 1, "toothNumber": 42, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m1_t45_pa20_a0_s15_embedded", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": math.Pi / 9, "angle": 0, "involuteSteps": 15}},
	{Name: "m2_t26_pa25_a0_s15_stub", Params: map[string]float64{
		"module": 2, "toothNumber": 26, "pressureAngle": 25 * math.Pi / 180, "angle": 0, "involuteSteps": 15}},
	{Name: "m2_t27_pa25_a0_s15_embedded", Params: map[string]float64{
		"module": 2, "toothNumber": 27, "pressureAngle": 25 * math.Pi / 180, "angle": 0, "involuteSteps": 15}},
	{Name: "m2_t30_pa25_a+30_s15_embedded", Params: map[string]float64{
		"module": 2, "toothNumber": 30, "pressureAngle": 25 * math.Pi / 180, "angle": math.Pi / 6, "involuteSteps": 15}},
}

func TestToolsSketch(t *testing.T) {
	proofkit.Run(t, profileCases, stepToolsSketch)
}

func TestGearProfileSketch(t *testing.T) {
	proofkit.Run(t, profileCases, stepGearProfileSketch)
}

func TestBoreProfileSketch(t *testing.T) {
	proofkit.Run(t, boreCases, stepBoreProfileSketch)
}

// stepToolsSketch is step 2: the Tools sketch.
//
// It draws no geometry of its own. Its whole content is the projection of the
// user's Anchor Point, which every later sketch re-projects from, so what the
// step has to prove is that the sketch is sound while holding nothing but that
// one reference — the projection is modelled with CreateReferencePoint, the
// engine's externally-locked point, because a Fusion projection is a reference
// and not a point the sketch is free to move.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Anchor Point into the Tools sketch")
	s.CreateReferencePoint(0, 0, "user anchor point")

	if n := len(s.Entities()); n != 0 {
		t.Errorf("Tools sketch drew %d curve(s); the spec says it draws none", n)
	}
	if n := len(s.Points()); n != 1 {
		t.Errorf("Tools sketch holds %d authored point(s), want the one projected anchor", n)
	}
}

// stepGearProfileSketch is steps 3, 4 and 5: the Gear Profile sketch, its
// involute tooth, and the anchoring that slides the whole drawing onto the
// user's anchor. All three are one timeline entry, and the anchoring happens
// inside the tooth generator's draw(), so they are one step here too.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGear(p)
	d := g.dims

	proofkit.Step(t, "project the Tools anchor in and add the movable local origin")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	anchor.SetName("projectedAnchor")
	origin := s.CreatePoint(0, 0)
	origin.SetName("localOrigin")

	proofkit.Step(t, "draw the four gear circles on the local origin")
	circle := func(r float64, construction bool, name string) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetName(name)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	root := circle(d.Root, false, "rootCircle")
	tip := circle(d.Tip, true, "tipCircle")
	circle(d.Base, true, "baseCircle")
	circle(d.Pitch, true, "pitchCircle")

	proofkit.Step(t, "sample the involute flanks and draw them as fitted splines")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, g.toothNumber, g.steps, g.angle)
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(left[i].X, left[i].Y)
		rp[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	leftFlank, err := s.CreateFitSpline(lp...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("leftFlank")
	rightFlank, err := s.CreateFitSpline(rp...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("rightFlank")
	last := len(lp) - 1

	proofkit.Step(t, "cap the tooth with an arc centred on the local origin")
	toothTop := s.CreatePoint(d.Tip*math.Cos(g.angle), d.Tip*math.Sin(g.angle))
	toothTop.SetName("toothTopPoint")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	toothTopArc := s.CreateArc(origin, rp[last], lp[last])
	toothTopArc.SetName("toothTopArc")

	proofkit.Step(t, "draw the spine and pin its absolute rotation against a +X reference")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	referenceEnd := s.CreatePoint(d.Tip, 0)
	referenceEnd.SetName("plusXReferenceEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, referenceEnd, d.Tip),
		sketch.NewVerticalDistance(origin, referenceEnd, 0),
	)
	reference := s.CreateLine(origin, referenceEnd)
	reference.SetConstruction(true)
	reference.SetName("plusXReference")
	// The engine reads a bare angle in the sketch's default angle unit, which is
	// degrees; the spec's angle is in radians. Passing radians here is solvable
	// and wrong — the tooth lands at angle-in-degrees — so the conversion is
	// part of the constraint, not presentation.
	spineAngle := sketch.NewAngle(reference, spine, g.angle*180/math.Pi)
	s.AddConstraint(spineAngle)

	proofkit.Step(t, "chain one rib per involute sample across the spine")
	cos, sin := math.Cos(g.angle), math.Sin(g.angle)
	acrossIsVertical := math.Abs(cos) >= math.Abs(sin)
	previous := origin
	for i := range lp {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], rp[i].Y()-lp[i].Y()))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], rp[i].X()-lp[i].X()))
		}
		// Seed the midpoint at the foot of the left fit point on the spine, not
		// at the rib's true midpoint: the point has to start on the line it is
		// about to be constrained onto.
		foot := lp[i].X()*cos + lp[i].Y()*sin
		mid := s.CreatePoint(foot*cos, foot*sin)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			// The last rib carries no perpendicular. The tooth-top arc shares
			// the local origin as its centre, which already holds the two flank
			// tips at equal radius either side of the spine.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, mid.X()-previous.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, mid.Y()-previous.Y()))
		}
		previous = mid
	}

	proofkit.Step(t, "close the tooth at the root")
	embedded := d.Embedded()
	if !embedded {
		for _, start := range []*sketch.Point{lp[0], rp[0]} {
			r := math.Hypot(start.X(), start.Y())
			x, y := start.X()*d.Root/r, start.Y()*d.Root/r
			end := s.CreatePoint(x, y)
			stub := s.CreateLine(end, start)
			stub.SetName("flankToRoot")
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, end, x),
				sketch.NewVerticalDistance(origin, end, y),
			)
		}
	}

	proofkit.Step(t, "anchor the local origin on the projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	// Everything below reads the solved drawing. proofkit solves and gates
	// again afterwards; solving here is what makes the facts the later steps
	// select on readable on the geometry this step actually built.
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve gear profile: %v", err)
	}

	// The requested rotation is drawn AND confirmed. Reading the solved spine
	// back is what catches a confirming dimension that dropped or flipped the
	// sign: at -angle a dropped sign still solves, mirrored.
	if got := reference.AngleTo(spine); math.Abs(wrapPi(got-g.angle)) > 1e-6 {
		t.Errorf("spine sits at %.6f rad from +X, want the requested %.6f", got, g.angle)
	}

	// The two regions and their curve counts are a contract the extrude steps
	// match on, so they are checked on the sketch that was actually drawn.
	tooth, disc := regions(t, s, d.Root)
	wantCurves := 6
	if embedded {
		wantCurves = 4
	}
	if got := len(tooth.Entities); got != wantCurves {
		t.Errorf("tooth region is bounded by %d curves, want %d (embedded=%v)", got, wantCurves, embedded)
	}
	if got := countKind[*sketch.FitSpline](tooth.Entities); got != 2 {
		t.Errorf("tooth region has %d spline flank(s), want 2", got)
	}
	if got := countKind[*sketch.Line](tooth.Entities); got != wantCurves-4 {
		t.Errorf("tooth region has %d flank-to-root line(s), want %d", got, wantCurves-4)
	}
	if !boundsOn(tooth, root) {
		t.Error("the tooth region does not close on the root circle, so the root circle was never split")
	}
	if !splits(tooth, root) {
		t.Error("the tooth region takes the whole root circle, not the arc under the tooth")
	}
	if got := math.Abs(disc.Area - g.discArea()); got > 1e-6*g.discArea() {
		t.Errorf("disc region area %.6f, want the root circle's %.6f", disc.Area, g.discArea())
	}
}

// stepBoreProfileSketch is the first half of step 12: the Bore Profile sketch.
//
// The tooth generator is instantiated on this sketch only to draw the bore
// circle, so its constructor's local-origin point is left over with nothing
// using it. The spec keeps that point and grounds it on the same projected
// anchor the circle's centre uses; the sketch is under-constrained without it,
// which is what this step proves.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGear(p)
	if g.bore <= 0 {
		proofkit.Unmodelled(t, "a bore of %.3f mm draws no Bore Profile sketch; buildBore returns before creating it", g.bore)
	}

	proofkit.Step(t, "project the Tools anchor in and draw the bore circle on it")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	bore := s.CreateCircle(anchor, g.bore/2)
	bore.SetName("boreCircle")
	s.AddConstraint(sketch.NewDiameter(bore, g.bore))

	proofkit.Step(t, "ground the tooth generator's leftover local origin on that same projection")
	origin := s.CreatePoint(0, 0)
	origin.SetName("localOrigin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve bore profile: %v", err)
	}
	regionsOf := s.Profiles()
	if len(regionsOf) != 1 {
		t.Fatalf("Bore Profile closed %d regions, want 1", len(regionsOf))
	}
	want := math.Pi * g.bore * g.bore / 4
	if math.Abs(regionsOf[0].Area-want) > 1e-9*want {
		t.Errorf("bore region area %.9f, want %.9f", regionsOf[0].Area, want)
	}
}

var boreCases = []proofkit.Case{
	{Name: "bore0_none", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "boreDiameter": 0}},
	{Name: "bore3", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "boreDiameter": 3}},
	{Name: "bore12_near_root", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "boreDiameter": 12}},
	{Name: "bore40_coarse", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "boreDiameter": 40}},
}

// wrapPi folds an angle difference into (-pi, pi].
func wrapPi(a float64) float64 {
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	return a
}

// regions returns the tooth region and the disc region of a solved Gear
// Profile, identified by area: the disc is the larger of exactly two.
func regions(t testing.TB, s *sketch.Sketch, rootRadius float64) (tooth, disc *sketch.Profile) {
	t.Helper()
	found := s.Profiles()
	if len(found) != 2 {
		t.Fatalf("Gear Profile closed %d regions, want exactly 2 (the tooth and the disc)", len(found))
	}
	for _, p := range found {
		if !p.Valid {
			t.Errorf("a Gear Profile region is not extrudable (self-intersecting=%v)", p.SelfIntersecting)
		}
	}
	tooth, disc = found[0], found[1]
	if tooth.Area > disc.Area {
		tooth, disc = disc, tooth
	}
	return tooth, disc
}

func countKind[T sketch.Entity](es []sketch.Entity) int {
	n := 0
	for _, e := range es {
		if _, ok := e.(T); ok {
			n++
		}
	}
	return n
}

// boundsOn reports whether the region's boundary uses the given entity.
func boundsOn(p *sketch.Profile, e sketch.Entity) bool {
	for _, got := range p.Entities {
		if got == e {
			return true
		}
	}
	return false
}

// splits reports whether the region takes only part of the entity — the fact
// that makes both loops exist at all, since neither closes until the tooth
// meets the root circle and cuts it in two.
func splits(p *sketch.Profile, e sketch.Entity) bool {
	span := 0.0
	used := false
	for _, edge := range p.Outer {
		if edge.Entity != e {
			continue
		}
		used = true
		span += edge.TEnd - edge.TStart
	}
	return used && span < 0.999
}
