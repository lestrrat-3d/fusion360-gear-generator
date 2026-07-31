// Package spurgear_test proves the sketch steps of the spur gear build.
//
// The build has three sketches — Tools, Gear Profile and Bore Profile — and one
// function here per sketch step of the step list. Everything else the build does
// is 3D (extrudes, a chamfer, a pattern, a combine, fillets, a cut), which the
// engine models nothing of, so those steps are prose in the step list and absent
// here.
//
// Compiled from spec/spurgear/instructions.md, spec/spurgear/fusion.md and
// .claude/skills/generate-gear/PLAYBOOK.md. See .tmp/spurgear.steps.md for the
// step list these functions are named by.
package spurgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// cases sweeps the sizes the scheme has to hold across.
//
// The first four are the baseline sweep. The rest reach branches the baseline
// does not: the embedded profile (which needs a HIGH tooth count, not a low one
// — see the report on step S5), the rotation the tooth generator's angle
// argument asks for, an anchor away from the sketch origin, a coarser involute
// sampling, and the no-bore path.
//
// Embedded is base radius < root radius, i.e. N·cos(pressureAngle) < N − 2.5,
// i.e. N > 2.5/(1−cos(pressureAngle)). At 20° that is N > 41.45; at 30° it is
// N > 18.66.
var cases = []proofkit.Case{
	{Name: "m1_t12", Params: map[string]float64{"module": 1, "toothNumber": 12, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 4}},
	{Name: "m1_t17", Params: map[string]float64{"module": 1, "toothNumber": 17, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 5}},
	{Name: "m2_t20", Params: map[string]float64{"module": 2, "toothNumber": 20, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 8}},
	{Name: "m3_t15", Params: map[string]float64{"module": 3, "toothNumber": 15, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 10}},

	{Name: "m05_t24_fine", Params: map[string]float64{"module": 0.5, "toothNumber": 24, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 3}},
	{Name: "m1_t45_embedded", Params: map[string]float64{"module": 1, "toothNumber": 45, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 6}},
	{Name: "m2_t60_embedded", Params: map[string]float64{"module": 2, "toothNumber": 60, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 20}},
	{Name: "m1_t30_pa30_embedded", Params: map[string]float64{"module": 1, "toothNumber": 30, "pressureAngle": 30, "involuteSteps": 15, "boreDiameter": 6}},
	{Name: "m1_t41_shortstub", Params: map[string]float64{"module": 1, "toothNumber": 41, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 6}},
	{Name: "m1_t42_barelyembedded", Params: map[string]float64{"module": 1, "toothNumber": 42, "pressureAngle": 20, "involuteSteps": 15, "boreDiameter": 6}},

	{Name: "m2_t20_rot30", Params: map[string]float64{"module": 2, "toothNumber": 20, "pressureAngle": 20, "involuteSteps": 15, "angle": 30, "boreDiameter": 8}},
	{Name: "m2_t20_rot90", Params: map[string]float64{"module": 2, "toothNumber": 20, "pressureAngle": 20, "involuteSteps": 15, "angle": 90, "boreDiameter": 8}},
	{Name: "m2_t20_rot180", Params: map[string]float64{"module": 2, "toothNumber": 20, "pressureAngle": 20, "involuteSteps": 15, "angle": 180, "boreDiameter": 8}},
	{Name: "m1_t17_rotneg45", Params: map[string]float64{"module": 1, "toothNumber": 17, "pressureAngle": 20, "involuteSteps": 15, "angle": -45, "boreDiameter": 5}},
	{Name: "m1_t45_rot120_embedded", Params: map[string]float64{"module": 1, "toothNumber": 45, "pressureAngle": 20, "involuteSteps": 15, "angle": 120, "boreDiameter": 6}},

	{Name: "m2_t20_offanchor", Params: map[string]float64{"module": 2, "toothNumber": 20, "pressureAngle": 20, "involuteSteps": 15, "anchorX": 37, "anchorY": -21, "boreDiameter": 8}},
	{Name: "m1_t17_steps8", Params: map[string]float64{"module": 1, "toothNumber": 17, "pressureAngle": 20, "involuteSteps": 8, "boreDiameter": 5}},
	{Name: "m1_t17_steps24", Params: map[string]float64{"module": 1, "toothNumber": 17, "pressureAngle": 20, "involuteSteps": 24, "boreDiameter": 5}},
	{Name: "m1_t17_nobore", Params: map[string]float64{"module": 1, "toothNumber": 17, "pressureAngle": 20, "involuteSteps": 15}},
}

func TestToolsSketch(t *testing.T)       { proofkit.Run(t, cases, stepToolsSketch) }
func TestGearProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepGearProfileSketch) }
func TestBoreProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepBoreProfileSketch) }

// gear is one case's derived geometry: radii in millimetres, angles in radians.
type gear struct {
	involute.Dimensions
	teeth  float64
	steps  int
	angle  float64
	ax, ay float64 // the anchor point's position in sketch-local coordinates
	bore   float64
}

func derive(p map[string]float64) gear {
	teeth := p["toothNumber"]
	return gear{
		Dimensions: involute.Derive(p["module"], teeth, p["pressureAngle"]*math.Pi/180),
		teeth:      teeth,
		steps:      int(p["involuteSteps"]),
		angle:      p["angle"] * math.Pi / 180,
		ax:         p["anchorX"],
		ay:         p["anchorY"],
		bore:       p["boreDiameter"],
	}
}

// stepToolsSketch is step S3: the Tools sketch.
//
// It draws no geometry of its own and exists to own one reference — the
// projection of the user's Anchor Point, which every later sketch re-projects
// from. A projection is placed by the layer above and never moved by the solver,
// which is what reference geometry is.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := derive(p)
	proofkit.Step(t, "project the Anchor Point in and keep it as ctx.anchorPoint")
	a := s.CreateReferencePoint(g.ax, g.ay, "Anchor Point")
	a.SetName("ctx.anchorPoint")
}

// stepGearProfileSketch is step S5: the Gear Profile sketch.
//
// One Fusion timeline entry covers the whole of the spec's steps 3, 4 and 5 —
// the four circles, the involute tooth and the anchoring — because the tooth
// generator's draw() runs all three inside the one sketch.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := derive(p)

	proofkit.Step(t, "constructor: the movable local origin, a fresh point at (0,0)")
	o := s.CreatePoint(0, 0)
	o.SetName("local origin")

	proofkit.Step(t, "drawCircles: four circles sharing the local origin, each with a driving diameter")
	root := s.CreateCircle(o, g.Root)
	root.SetName("root circle")
	tip := s.CreateCircle(o, g.Tip)
	tip.SetName("tip circle")
	tip.SetConstruction(true)
	base := s.CreateCircle(o, g.Base)
	base.SetName("base circle")
	base.SetConstruction(true)
	pitch := s.CreateCircle(o, g.Pitch)
	pitch.SetName("pitch circle")
	pitch.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(root, 2*g.Root),
		sketch.NewDiameter(tip, 2*g.Tip),
		sketch.NewDiameter(base, 2*g.Base),
		sketch.NewDiameter(pitch, 2*g.Pitch),
	)
	// The along-path label on each circle is text, which carries no constraint
	// and no boundary, so there is nothing here for the engine to hold.

	proofkit.Step(t, "drawTooth: sample both flanks and draw them as fitted splines")
	left, right := involute.Flanks(g.Base, g.Tip, g.Pitch, g.teeth, g.steps, g.angle)
	n := len(left)
	lp := make([]*sketch.Point, n)
	rp := make([]*sketch.Point, n)
	for i := range n {
		lp[i] = s.CreatePoint(left[i].X, left[i].Y)
		rp[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	ls, err := s.CreateFitSpline(lp...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	ls.SetName("left flank")
	rs, err := s.CreateFitSpline(rp...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rs.SetName("right flank")

	proofkit.Step(t, "tooth-top arc: a point on the tip circle, then an arc sharing the local origin as its centre")
	top := s.CreatePoint(g.Tip*math.Cos(g.angle), g.Tip*math.Sin(g.angle))
	top.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))
	// Sharing the centre is the whole constraint set: the arc's own radius
	// consistency then says the two flank ends are equidistant from the centre,
	// which is exactly what the last rib's perpendicular would have said. That
	// is why the rib loop below skips it.
	//
	// What this cannot reach: the ⚠️ in [SPUR-F-TOOTHTOP-ARC] says a FREE centre
	// plus a diameter dimension leaves an inward-bulging alternative. Built that
	// way (free centre, diameter dimension, last rib's perpendicular restored)
	// the engine's probe still finds one configuration, so the proof neither
	// confirms nor refutes that warning. It is a Fusion solver-branch claim; the
	// engine picks its arc direction from the centre/start/end triple instead.
	arc := s.CreateArc(o, rp[n-1], lp[n-1])
	arc.SetName("tooth top arc")

	proofkit.Step(t, "spine, +X reference line, and the angular dimension that says which way the spine points")
	spine := s.CreateLine(o, top)
	spine.SetName("spine")
	spine.SetConstruction(true)
	xend := s.CreatePoint(g.Tip, 0)
	xend.SetName("+X reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(o, xend, g.Tip),
		sketch.NewVerticalDistance(o, xend, 0),
	)
	xref := s.CreateLine(o, xend)
	xref.SetName("+X reference")
	xref.SetConstruction(true)
	// Signed, and from the reference to the spine in that order. A plain
	// horizontal on the spine instead leaves the tooth free to settle 180° round
	// — the engine's probe reports two configurations for that variant.
	s.AddConstraint(sketch.NewAngle(xref, spine, g.angle*180/math.Pi))

	proofkit.Step(t, "ribs: one per fit-point index, each with a midpoint chained along the spine")
	ca, sa := math.Cos(g.angle), math.Sin(g.angle)
	// The spine runs along (cos, sin) and a rib across (-sin, cos), so a signed
	// dimension has to be taken on whichever axis carries the larger component:
	// across the rib that is the vertical one when |cos| dominates, and along the
	// spine chain it is the other way round.
	alongX := math.Abs(ca) >= math.Abs(sa)
	prevX, prevY := 0.0, 0.0
	prevPt := o
	for i := range n {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetName(fmt.Sprintf("rib %d", i))
		rib.SetConstruction(true)
		if alongX {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}
		// Seeded at the foot of the left fit point on the spine, not at the rib's
		// own midpoint.
		foot := left[i].X*ca + left[i].Y*sa
		mx, my := foot*ca, foot*sa
		mid := s.CreatePoint(mx, my)
		mid.SetName(fmt.Sprintf("rib %d midpoint", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != n-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		// The chain starts at the local origin. Without that first link the whole
		// rib chain slides along the spine as a unit and the sketch never closes.
		if alongX {
			s.AddConstraint(sketch.NewHorizontalDistance(prevPt, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevPt, mid, my-prevY))
		}
		prevX, prevY, prevPt = mx, my, mid
	}

	if g.Embedded() {
		proofkit.Step(t, "embedded profile: the flank starts inside the root circle, so no flank-to-root lines")
	} else {
		proofkit.Step(t, "flank-to-root: a radial stub each side, its root end pinned by two signed dimensions")
		for _, f := range []struct {
			name string
			at   involute.Pt
			fit  *sketch.Point
		}{{"left", left[0], lp[0]}, {"right", right[0], rp[0]}} {
			r := math.Hypot(f.at.X, f.at.Y)
			ex, ey := f.at.X*g.Root/r, f.at.Y*g.Root/r
			end := s.CreatePoint(ex, ey)
			end.SetName(f.name + " root end")
			stub := s.CreateLine(end, f.fit)
			stub.SetName(f.name + " flank to root")
			// Two signed offsets from the local origin, and nothing else. Putting
			// the end on the root circle instead would admit the point diametrically
			// opposite and turn the stub into a line across the gear.
			s.AddConstraint(
				sketch.NewHorizontalDistance(o, end, ex),
				sketch.NewVerticalDistance(o, end, ey),
			)
		}
	}

	proofkit.Step(t, "anchor the sketch: project the Tools anchor in, then make the local origin coincident with it")
	anchor := s.CreateReferencePoint(g.ax, g.ay, "Anchor Point")
	anchor.SetName("projected anchor")
	s.AddConstraint(sketch.NewCoincident(o, anchor))

	requireExtrudableLoops(t, s, g.Embedded())
}

// requireExtrudableLoops checks that the two loops the later extrudes look for
// are the two the sketch actually holds.
//
// Step S7 asks for the tooth loop by curve counts — 2 splines, 2 arcs, and 2
// lines unless the profile is embedded — and step S9 asks for the gear body loop
// by its 2 arcs. Both counts are Fusion's, which sees the root circle as the two
// arcs it is split into; the engine keeps it as one Circle entity and reports the
// pieces as fragments of it, so the equivalent test here is on entities.
func requireExtrudableLoops(t testing.TB, s *sketch.Sketch, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before reading profiles: %v", err)
	}
	kinds := func(p *sketch.Profile) map[string]int {
		m := map[string]int{}
		for _, e := range p.Entities {
			switch e.(type) {
			case *sketch.FitSpline:
				m["spline"]++
			case *sketch.Arc:
				m["arc"]++
			case *sketch.Circle:
				m["circle"]++
			case *sketch.Line:
				m["line"]++
			}
		}
		return m
	}
	wantTooth := map[string]int{"spline": 2, "arc": 1, "circle": 1, "line": 2}
	if embedded {
		delete(wantTooth, "line")
	}
	wantBody := map[string]int{"circle": 1}

	var tooth, body int
	profiles := s.Profiles()
	for _, p := range profiles {
		switch k := kinds(p); {
		case eq(k, wantTooth):
			tooth++
		case eq(k, wantBody):
			body++
		default:
			t.Errorf("unexpected region: %v (area %.3f)", k, p.Area)
		}
	}
	if tooth != 1 || body != 1 {
		t.Errorf("want one tooth loop and one gear body loop, got %d and %d out of %d region(s)",
			tooth, body, len(profiles))
	}
}

func eq(a, b map[string]int) bool {
	if len(a) != len(b) {
		return false
	}
	for k, v := range a {
		if b[k] != v {
			return false
		}
	}
	return true
}

// stepBoreProfileSketch is step S14: the Bore Profile sketch.
//
// It carries the tooth generator's unused local-origin point, which the
// constructor places at (0,0) and nothing afterwards moves. Placed and never
// moved is what reference geometry is, so it is modelled as one — the sketch is
// fully constrained with it in, and the spec's carve-out for it in Sketch
// Discipline turns out not to be needed here.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := derive(p)
	if g.bore <= 0 {
		proofkit.Unmodelled(t, "Bore Diameter is %g, so buildBore returns before any Bore Profile sketch exists", g.bore)
	}

	proofkit.Step(t, "constructor: the unused local-origin point the tooth generator always adds")
	stray := s.CreateReferencePoint(0, 0, "tooth generator local origin")
	stray.SetName("unused local origin")

	proofkit.Step(t, "drawBore: project the anchor in, draw the bore circle on it, dimension its diameter")
	anchor := s.CreateReferencePoint(g.ax, g.ay, "Anchor Point")
	anchor.SetName("projected anchor")
	bore := s.CreateCircle(anchor, g.bore/2)
	bore.SetName("bore circle")
	s.AddConstraint(sketch.NewDiameter(bore, g.bore))
}
