package helicalgear_test

import (
	"maps"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// Standalone construction planes and axes return neither authored sketch
// geometry nor a body to a proof harness. The solid builds instead consume real
// offset planes and the cylinder axis. They do not prove Fusion's occurrence
// context, visibility, or face/axis classification. A Tools sketch containing
// only a projection fails proofkit's authored-geometry gate; the profile builds
// use reference anchors without claiming to prove Fusion's projection chain.

var bottomCases = []proofkit.Case{
	{Name: "default", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180}},
	{Name: "fine", Params: map[string]float64{"module": .5, "teeth": 12, "pressure": 14.5 * math.Pi / 180, "samples": 3}},
	{Name: "coarse", Params: map[string]float64{"module": 3, "teeth": 30, "pressure": 20 * math.Pi / 180}},
}

func stepBottomProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	params := maps.Clone(p)
	params["angle"] = 0
	stepTwistedProfile(t, s, params)
}

var boreSketchCases = []proofkit.Case{
	{Name: "small", Params: map[string]float64{"bore": 1}},
	{Name: "large", Params: map[string]float64{"bore": 3}},
}

func stepBoreProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Tools anchor and ground the drawer's local origin")
	anchor := s.CreateReferencePoint(2, -3, "Tools.anchor")
	origin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	circle := s.CreateCircle(anchor, p["bore"])
	s.AddConstraint(sketch.NewDiameter(circle, 2*p["bore"]))
	sketchtest.Solve(t, s)
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	// The circle formula is exact; slack covers floating-point evaluation.
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*p["bore"]*p["bore"], sketchtest.Within(1e-8))
	if len(profile.Entities) != 1 || len(profile.Outer) != 1 {
		t.Fatal("bore profile must contain one circle")
	}
}

var sketchCases = []proofkit.Case{
	{Name: "default_right", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": 14.5 * math.Pi / 180}},
	{Name: "default_left", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": -14.5 * math.Pi / 180}},
	{Name: "zero", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180}},
	{Name: "quarter_right", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": math.Pi / 2}},
	{Name: "quarter_left", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": -math.Pi / 2}},
	{Name: "half_right", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": math.Pi}},
	{Name: "half_left", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180, "angle": -math.Pi}},
	{Name: "fine_low_samples", Params: map[string]float64{"module": .5, "teeth": 12, "pressure": 14.5 * math.Pi / 180, "angle": -.4, "samples": 3}},
	{Name: "coarse", Params: map[string]float64{"module": 3, "teeth": 30, "pressure": 20 * math.Pi / 180, "angle": .4}},
	{Name: "embedded_count", Params: map[string]float64{"module": 1, "teeth": 50, "pressure": 20 * math.Pi / 180, "angle": -.2}},
	{Name: "embedded_pressure", Params: map[string]float64{"module": 1, "teeth": 30, "pressure": 25 * math.Pi / 180, "angle": .2}},
}

func stepTwistedProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Tools anchor and derive the four circles")
	anchor := s.CreateReferencePoint(2, -3, "Tools.anchor")
	origin := s.CreatePoint(2, -3)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	d := involute.Derive(p["module"], p["teeth"], p["pressure"])
	var tip *sketch.Circle
	for i, r := range []float64{d.Pitch, d.Base, d.Root, d.Tip} {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(i != 2)
		s.AddConstraint(sketch.NewRadius(c, r))
		if i == 3 {
			tip = c
		}
	}
	angle := p["angle"]
	ca, sa := math.Cos(angle), math.Sin(angle)
	samples := 15
	if p["samples"] > 0 {
		samples = int(p["samples"])
	}
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], samples, angle)
	lp, rp := make([]*sketch.Point, len(left)), make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(2+left[i].X, -3+left[i].Y)
		rp[i] = s.CreatePoint(2+right[i].X, -3+right[i].Y)
	}
	if _, err := s.CreateFitSpline(lp...); err != nil {
		t.Fatal(err)
	}
	if _, err := s.CreateFitSpline(rp...); err != nil {
		t.Fatal(err)
	}
	proofkit.Step(t, "share the cap endpoints and confirm the directed spine")
	top := s.CreatePoint(2+d.Tip*ca, -3+d.Tip*sa)
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))
	center := s.CreatePoint(2, -3)
	s.CreateArc(center, rp[len(rp)-1], lp[len(lp)-1])
	s.AddConstraint(sketch.NewCoincident(center, origin))
	spine := s.CreateLine(origin, top)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(2+d.Tip, -3)
	s.AddConstraint(sketch.NewHorizontalDistance(origin, refEnd, d.Tip), sketch.NewVerticalDistance(origin, refEnd, 0))
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	angular := sketch.NewAngle(reference, spine, angle*180/math.Pi)
	s.AddConstraint(angular)
	proofkit.Step(t, "dimension the rib chain across and along the rotated spine")
	previous := origin
	previousX, previousY := 0.0, 0.0
	for i := range lp {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetConstruction(true)
		if math.Abs(ca) >= math.Abs(sa) {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}
		along := left[i].X*ca + left[i].Y*sa
		mx, my := along*ca, along*sa
		mid := s.CreatePoint(2+mx, -3+my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine), sketch.NewMidpoint(mid, rib))
		if i != len(lp)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if math.Abs(ca) >= math.Abs(sa) {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, mx-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, my-previousY))
		}
		previous, previousX, previousY = mid, mx, my
	}
	proofkit.Step(t, "draw the two root stubs and let profile detection split the root circle")
	for i, pts := range [][]involute.Pt{left, right} {
		if d.Embedded() {
			break
		}
		x, y := pts[0].X*d.Root/d.Base, pts[0].Y*d.Root/d.Base
		end := s.CreatePoint(2+x, -3+y)
		flank := lp[0]
		if i == 1 {
			flank = rp[0]
		}
		s.CreateLine(end, flank)
		s.AddConstraint(sketch.NewHorizontalDistance(origin, end, x), sketch.NewVerticalDistance(origin, end, y))
	}
	sketchtest.Solve(t, s)
	sketchtest.Satisfies(t, angular, sketchtest.Within(1e-8))
	// Coordinates use analytic rotation; slack covers the numerical solve only.
	sketchtest.MeasuresPoint(t, top, 2+d.Tip*ca, -3+d.Tip*sa, sketchtest.Within(1e-7))
	found := 0
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("gear profile regions: got %d, want 2", len(profiles))
	}
	wantCurves, wantLines := 6, 2
	if d.Embedded() {
		wantCurves, wantLines = 4, 0
	}
	for _, profile := range profiles {
		if len(profile.Entities) != wantCurves {
			continue
		}
		sketchtest.IsValidProfile(t, profile)
		sketchtest.IsCurrentProfile(t, profile)
		splines, arcs, lines := 0, 0, 0
		for _, e := range profile.Entities {
			switch e.(type) {
			case *sketch.FitSpline:
				splines++
			case *sketch.Circle, *sketch.Arc:
				arcs++
			case *sketch.Line:
				lines++
			}
		}
		if splines != 2 || arcs != 2 || lines != wantLines {
			t.Fatalf("tooth boundary: splines=%d arcs=%d lines=%d", splines, arcs, lines)
		}
		found++
	}
	if found != 1 {
		t.Fatalf("%d-curve tooth profiles: got %d, want 1", wantCurves, found)
	}
}
