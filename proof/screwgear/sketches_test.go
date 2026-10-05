package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/stretchr/testify/require"
)

var anchorCases = []proofkit.Case{
	{Name: "origin", Params: map[string]float64{"x": 0, "y": 0}},
	{Name: "offset", Params: map[string]float64{"x": 13, "y": -7}},
}

func buildAnchor(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the selected centre and constrain the Anchor Line")
	c := s.CreateReferencePoint(p["x"], p["y"], "selected centre")
	a := s.CreatePoint(p["x"]-5, p["y"])
	b := s.CreatePoint(p["x"]+5, p["y"])
	line := s.CreateLine(a, b)
	// The sketch midpoint already supplies Fusion's additional coincidence row.
	midpoint := sketch.NewMidpoint(c, line)
	horizontal := sketch.NewHorizontal(line)
	length := sketch.NewHorizontalDistance(a, b, 10)
	s.AddConstraint(midpoint, horizontal, length)
	sketchtest.Solve(t, s)
	sketchtest.Satisfies(t, midpoint, sketchtest.Within(1e-8))
	sketchtest.Satisfies(t, horizontal, sketchtest.Within(1e-8))
	sketchtest.Satisfies(t, length, sketchtest.Within(1e-8))
	// The exact horizontal endpoint formula has only floating point roundoff.
	sketchtest.MeasuresPoint(t, a, p["x"]-5, p["y"], sketchtest.Within(1e-8))
	sketchtest.MeasuresPoint(t, b, p["x"]+5, p["y"], sketchtest.Within(1e-8))
}

func defaults() map[string]float64 {
	return map[string]float64{
		"W": 15, "T": 3.75, "P": 2.625, "H": 2.625, "N": 68,
		"lead": 49.5, "slant": 25.8, "bow": 0.048, "cross": 80,
		"engagement": 1.05, "phase": -1.31, "radius": 15, "half": 3,
		"rise": 18.75, "wall": 3, "clearance": 0.2, "roof": 0.6,
		"gear": 0, "sigma": -1, "mount": 0,
	}
}

func changed(name string, value float64) map[string]float64 {
	p := defaults()
	p[name] = value
	return p
}

var pathCases = []proofkit.Case{
	{Name: "defaults", Params: defaults()},
	{Name: "thin_clearance", Params: changed("clearance", 0.05)},
	{Name: "no_roof", Params: changed("roof", 0)},
}

func channelSpan(p map[string]float64) (float64, float64) {
	c := math.Hypot(p["W"]/2+p["clearance"], p["T"]/2+p["clearance"]+p["roof"])
	ri := p["radius"] - p["half"]
	return math.Sqrt(ri*ri-c*c) - 1, p["radius"] + p["half"] + 1
}

func buildPaths(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "draw the two fixed axis paths")
	si, so := channelSpan(p)
	xs := []float64{-so, -si, si, so}
	pts := make([]*sketch.Point, 4)
	for i, x := range xs {
		pts[i] = s.CreatePoint(x, 0)
	}
	s.CreateLine(pts[0], pts[1])
	s.CreateLine(pts[2], pts[3])
	for _, pt := range pts {
		s.Fix(pt)
	}
	require.Len(t, s.Entities(), 2)
	// These station formulas contain only floating point roundoff.
	for i, pt := range pts {
		sketchtest.MeasuresPoint(t, pt, xs[i], 0, sketchtest.Within(1e-9))
	}
}

func tooth(p map[string]float64, station, v float64) float64 {
	return p["W"]/2 - p["H"]/2 + p["H"]/2*math.Cos(2*math.Pi*
		(station+math.Tan(p["slant"]*math.Pi/180)*v-p["phase"])/p["P"]) - p["bow"]*v*v
}

func sectionPoints(p map[string]float64, station float64) [][2]float64 {
	pts := [][2]float64{{-p["W"] / 2, -p["T"] / 2}}
	for j := 0; j < 11; j++ {
		v := -p["T"]/2 + float64(j)*p["T"]/10
		pts = append(pts, [2]float64{tooth(p, station, v), v})
	}
	return append(pts, [2]float64{-p["W"] / 2, p["T"] / 2})
}

func fixedPolygon(s *sketch.Sketch, xy [][2]float64) []*sketch.Point {
	pts := make([]*sketch.Point, len(xy))
	for i, p := range xy {
		pts[i] = s.CreatePoint(p[0], p[1])
	}
	for i, p := range pts {
		s.CreateLine(p, pts[(i+1)%len(pts)])
	}
	for _, p := range pts {
		s.Fix(p)
	}
	return pts
}

var sectionCases = []proofkit.Case{
	{Name: "positive_slant", Params: defaults()},
	{Name: "negative_slant", Params: changed("slant", -25.8)},
	{Name: "straight_ridge", Params: changed("slant", 0)},
	{Name: "no_bow", Params: changed("bow", 0)},
	{Name: "long_lead", Params: changed("lead", 400)},
	{Name: "short_lead", Params: changed("lead", 20)},
}

func buildCellSections(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	// The planar engine cannot represent Fusion's one off-plane sketch.
	// A separate planar polygon at each station substitutes its eleven-point
	// fitted spline with ten chords; no fitted-curve or 3D-sketch verdict is claimed.
	proofkit.Step(t, "draw every section as its own checked planar polygon")
	n := int(math.Max(8, math.Ceil(p["P"]/(p["lead"]/(2*math.Pi))/(2*math.Pi/180))))
	start := p["phase"] - p["N"]*p["P"]/2
	count := 4
	if v, ok := p["cell"]; ok {
		count = int(v)
	}
	if v, ok := p["start"]; ok {
		start = v
	}
	for k := 0; k <= count*n; k++ {
		target := s
		if k > 0 {
			target = proofkit.NewSketch(t)
		}
		station := start + float64(k)*p["P"]/float64(n)
		xy := sectionPoints(p, station)
		pts := fixedPolygon(target, xy)
		proofkit.RequireSound(t, target)
		report := sketchtest.Verify(t, target)
		profile := sketchtest.SingleProfile(t, report)
		sketchtest.IsValidProfile(t, profile)
		sketchtest.IsCurrentProfile(t, profile)
		sketchtest.HasExactCuts(t, profile)
		require.Len(t, profile.Outer, 13)
		// Point formulas are exact samples; the comparison allows roundoff only.
		for i, pt := range pts {
			sketchtest.MeasuresPoint(t, pt, xy[i][0], xy[i][1], sketchtest.Within(1e-9))
		}
	}
}

var sleeveSketchCases = []proofkit.Case{
	{Name: "defaults", Params: defaults()},
	{Name: "larger_radius", Params: changed("radius", 20)},
	{Name: "thin_wall", Params: changed("half", 2)},
}

func drawAnnulus(s *sketch.Sketch, p map[string]float64) {
	for _, r := range []float64{p["radius"] - p["half"], p["radius"] + p["half"]} {
		c := s.CreatePoint(0, 0)
		circle := s.CreateCircle(c, r)
		s.Fix(c)
		s.AddConstraint(sketch.NewDiameter(circle, 2*r))
	}
}

func buildSleeveSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "draw the concentric sleeve circles")
	drawAnnulus(s, p)
	sketchtest.Solve(t, s)
	require.Len(t, s.Profiles(), 2)
	found := 0
	for _, profile := range s.Profiles() {
		if len(profile.Holes) != 1 {
			continue
		}
		found++
		sketchtest.IsValidProfile(t, profile)
		// Analytic annulus area has only arithmetic roundoff.
		sketchtest.MeasuresProfileArea(t, profile, math.Pi*
			(math.Pow(p["radius"]+p["half"], 2)-math.Pow(p["radius"]-p["half"], 2)),
			sketchtest.Within(1e-7))
	}
	require.Equal(t, 1, found)
}

var markerCases = markerTable()

func markerTable() []proofkit.Case {
	var out []proofkit.Case
	for g := 0; g < 2; g++ {
		for _, sigma := range []float64{-1, 1} {
			p := defaults()
			p["gear"] = float64(g)
			p["sigma"] = sigma
			name := "square"
			if sigma > 0 {
				name = "circle"
			}
			if g == 0 {
				name = "A_" + name
			} else {
				name = "B_" + name
			}
			out = append(out, proofkit.Case{Name: name, Params: p})
		}
	}
	return out
}
func markerCentre(p map[string]float64) (float64, float64) {
	angle := p["cross"] * math.Pi / 360
	if p["gear"] == 1 {
		angle = -angle
	}
	return p["sigma"] * p["radius"] * math.Cos(angle),
		p["sigma"] * p["radius"] * math.Sin(angle)
}
func drawMarker(s *sketch.Sketch, p map[string]float64) {
	h := math.Min(1, p["half"]/2)
	x, y := markerCentre(p)
	if p["sigma"] > 0 {
		c := s.CreatePoint(x, y)
		circle := s.CreateCircle(c, h)
		s.Fix(c)
		s.AddConstraint(sketch.NewDiameter(circle, 2*h))
		return
	}
	fixedPolygon(s, [][2]float64{{x - h, y - h}, {x + h, y - h}, {x + h, y + h}, {x - h, y + h}})
}
func buildMarkerSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "draw the bore identification mark")
	drawMarker(s, p)
	sketchtest.Solve(t, s)
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	h := math.Min(1, p["half"]/2)
	area := 4 * h * h
	if p["sigma"] > 0 {
		area = math.Pi * h * h
	}
	// Circle and square formulas have only floating point roundoff.
	sketchtest.MeasuresProfileArea(t, profile, area, sketchtest.Within(1e-8))
	require.Less(t, math.Sqrt(2)*h, p["half"])
}

var boreSketchCases = func() []proofkit.Case {
	var out []proofkit.Case
	for _, c := range markerCases {
		out = append(out, c)
		p := make(map[string]float64, len(c.Params))
		for k, v := range c.Params {
			p[k] = v
		}
		p["roof"] = 0
		out = append(out, proofkit.Case{Name: c.Name + "_no_roof", Params: p})
	}
	return out
}()

func buildBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "constrain the asymmetric roof rectangle")
	si, so := channelSpan(p)
	station := si
	if p["sigma"] < 0 {
		station = -so
	}
	theta := station/(p["lead"]/(2*math.Pi)) + p["mount"]*math.Pi/180
	hw := p["W"]/2 + p["clearance"]
	lo, hi := boreLimits(p)
	turn := func(u, v float64) [2]float64 {
		return [2]float64{u*math.Cos(theta) - v*math.Sin(theta),
			u*math.Sin(theta) + v*math.Cos(theta)}
	}
	o := s.CreatePoint(0, 0)
	cp := s.CreatePoint((p["W"]-p["engagement"])/2, 0)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	exy := turn(hw, 0)
	e := s.CreatePoint(exy[0], exy[1])
	spine := s.CreateLine(o, e)
	spine.SetConstruction(true)
	xy := [][2]float64{turn(-hw, lo), turn(hw, lo), turn(hw, hi), turn(-hw, hi)}
	pts := make([]*sketch.Point, 4)
	lines := make([]*sketch.Line, 4)
	for i, v := range xy {
		pts[i] = s.CreatePoint(v[0], v[1])
	}
	for i, v := range pts {
		lines[i] = s.CreateLine(v, pts[(i+1)%4])
	}
	s.Fix(o)
	s.Fix(cp)
	// Fusion offsets remember the seeded side. The signed offset carries that
	// same side explicitly, and incorporates the parallel row itself.
	s.AddConstraint(sketch.NewDistance(o, e, hw),
		sketch.NewOffset(spine, lines[0], lo),
		sketch.NewOffset(spine, lines[2], hi),
		sketch.NewPointOnLine(e, lines[1]),
		sketch.NewPerpendicular(lines[1], spine),
		sketch.NewOffset(lines[1], lines[3], 2*hw))
	angle := theta * 180 / math.Pi
	if math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) {
		s.AddConstraint(sketch.NewAngle(ru, spine, angle))
	} else {
		s.AddConstraint(sketch.NewAngle(ru, lines[1], angle+90))
	}
	sketchtest.Solve(t, s)
	for i, point := range pts {
		// Fusion's post-solve corner check allows one micron.
		sketchtest.MeasuresPoint(t, point, xy[i][0], xy[i][1], sketchtest.Within(0.001))
	}
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	require.Len(t, profile.Outer, 4)
	// Rectangle area is analytic; this slack accounts only for solver roundoff.
	sketchtest.MeasuresProfileArea(t, profile, 2*hw*(hi-lo), sketchtest.Within(1e-5))
}

var windowCases = []proofkit.Case{
	{Name: "positive_k", Params: changed("sigma", 1)},
	{Name: "negative_k", Params: changed("sigma", -1)},
}

func windowCorners(p map[string]float64) [][2]float64 {
	// The compile proof consumes the window search's recorded default numbers.
	// Their printed precision is 0.01 mm; it does not reproduce the search.
	if p["sigma"] > 0 {
		return [][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.57, 10.58},
			{-11.57, 6.39}, {8.49, -13.67}, {11.57, -10.58}}
	}
	return [][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.50, 10.65},
		{-11.50, 6.91}, {8.49, -13.07}, {11.57, -9.99}}
}
func buildWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "draw the window hexagon from the search result")
	fixedPolygon(s, windowCorners(p))
	sketchtest.Solve(t, s)
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	require.Len(t, profile.Outer, 6)
	area := 220.7
	if p["sigma"] < 0 {
		area = 206.7
	}
	// Printed corners are rounded to 0.01 mm, so the area oracle allows 0.5 mm².
	sketchtest.MeasuresProfileArea(t, profile, area, sketchtest.Within(0.5))
}
