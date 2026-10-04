package screwgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var boreCases = makeBoreCases()

func makeBoreCases() []proofkit.Case {
	var out []proofkit.Case
	for _, gear := range []float64{0, 1} {
		for _, sign := range []float64{-1, 1} {
			for _, theta := range []float64{-180, -135, -90, -45, -1, 0, 1, 45, 90, 135, 180} {
				for _, roof := range []float64{0, 0.3} {
					out = append(out, proofkit.Case{Name: fmt.Sprintf("gear%g_sign%g_angle%g_roof%g", gear, sign, theta, roof),
						Params: changed(map[string]float64{"gear": gear, "sign": sign, "theta": theta, "phi": theta, "roof": roof})})
				}
			}
		}
	}
	return out
}

var boreSolidCases = []proofkit3d.Case{
	{Name: "a-minus", Params: dimensions()},
	{Name: "a-plus", Params: changed(map[string]float64{"sign": 1})},
	{Name: "b-minus", Params: changed(map[string]float64{"gear": 1})},
	{Name: "b-plus", Params: changed(map[string]float64{"gear": 1, "sign": 1})},
	{Name: "tight-clearance", Params: changed(map[string]float64{"clear": 0.05})},
	{Name: "large-clearance", Params: changed(map[string]float64{"clear": 0.9})},
}

func boreOutline(p map[string]float64) [][2]float64 {
	hw, ht := p["W"]/2+p["clear"], p["T"]/2+p["clear"]
	lo, hi := -ht, ht
	if p["sign"] == levelBoreSign(p) {
		theta := p["sign"]*p["radius"]/(p["lead"]/(2*math.Pi)) + p["phi"]*math.Pi/180
		uNormal := 1.0
		if p["gear"] == 1 {
			uNormal = -1
		}
		if -math.Sin(theta)*uNormal > 0 {
			hi += p["roof"]
		} else {
			lo -= p["roof"]
		}
	}
	return [][2]float64{{-hw, lo}, {hw, lo}, {hw, hi}, {-hw, hi}}
}

func levelBoreSign(p map[string]float64) float64 {
	lambda := p["lead"] / (2 * math.Pi)
	tilt := func(sign float64) float64 {
		a := sign*(p["radius"]-p["half"])/lambda + p["phi"]*math.Pi/180
		b := sign*(p["radius"]+p["half"])/lambda + p["phi"]*math.Pi/180
		lo, hi := math.Min(a, b), math.Max(a, b)
		k := math.Ceil((lo - math.Pi/2) / math.Pi)
		if math.Pi/2+k*math.Pi <= hi {
			return 0
		}
		return math.Min(math.Abs(math.Cos(a)), math.Abs(math.Cos(b)))
	}
	if tilt(-1) <= tilt(1) {
		return -1
	}
	return 1
}
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "rectangle constrained by a spine, signed offsets, and an angular reference")
	outline := boreOutline(p)
	theta := p["theta"] * math.Pi / 180
	o := s.CreateReferencePoint(0, 0, "bore-axis")
	cp := s.CreateReferencePoint((p["W"]-p["engage"])/2, 0, "unrotated-u")
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	x, y := turned(outline[1][0], 0, theta)
	e := s.CreatePoint(x, y)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	points := make([]*sketch.Point, 4)
	for i, c := range outline {
		x, y := turned(c[0], c[1], theta)
		points[i] = s.CreatePoint(x, y)
	}
	lines := make([]*sketch.Line, 4)
	for i, pt := range points {
		lines[i] = s.CreateLine(pt, points[(i+1)%4])
	}
	s.Fix(o)
	s.Fix(cp)
	// NewOffset includes the parallel row, unlike Fusion's offset dimension.
	// Its signed target carries the seed-side direction Fusion dimensions capture.
	constraints := []sketch.Constraint{
		sketch.NewDistance(o, e, outline[1][0]),
		sketch.NewOffset(k, lines[0], outline[0][1]),
		sketch.NewOffset(k, lines[2], outline[2][1]),
		sketch.NewPointOnLine(e, lines[1]),
		sketch.NewPerpendicular(lines[1], k),
		sketch.NewOffset(lines[1], lines[3], outline[1][0]-outline[0][0]),
	}
	if math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) {
		constraints = append(constraints, sketch.NewAngle(ru, k, p["theta"]))
	} else {
		constraints = append(constraints, sketch.NewAngle(ru, lines[1], p["theta"]+90))
	}
	s.AddConstraint(constraints...)
	sketchtest.Solve(t, s)
	for _, constraint := range constraints {
		sketchtest.Satisfies(t, constraint, sketchtest.Within(1e-7))
	}
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	sketchtest.HasExactCuts(t, profile)
	require.Len(t, profile.Outer, 4)
	// The rectangle's exact area has only floating point roundoff.
	sketchtest.MeasuresProfileArea(t, profile, polygonArea(outline), sketchtest.Within(1e-7))
}
func stepBoreSweep(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// The engine has no twisted sweep. Rotated rectangular sections are lofted
	// pairwise; their triangulated walls can depart by 0.32 mm from a ruled wall.
	// No clearance is asserted from this substituted channel.
	c := math.Hypot(p["W"]/2+p["clear"], p["T"]/2+p["clear"]+p["roof"])
	ri := p["radius"] - p["half"]
	in, out := math.Sqrt(ri*ri-c*c)-1, p["radius"]+p["half"]+1
	start, end := in, out
	if p["sign"] < 0 {
		start, end = -out, -in
	}
	lambda := p["lead"] / (2 * math.Pi)
	step := math.Min(5*math.Pi/180, 2*math.Acos(1-0.04*p["clear"]/c))
	count := int(math.Ceil((end-start)/lambda/step)) + 1
	outline := boreOutline(p)
	channel := loftChain(t, doc, p, start, end, count, func(float64) [][2]float64 { return outline })
	tube := buildTube(t, doc, p)
	cut, err := decad.Cut(t.Context(), tube, channel)
	require.NoError(t, err)
	return []*decad.Body{cut}
}
func assertBoreSweep(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.Len(t, b[0].Lumps(), 1)
	volume, err := b[0].Volume()
	require.NoError(t, err)
	// This oracle pins nonempty material and a nonzero removed volume. Its
	// interval covers all volumes between 0.1 mm³ and the tube less 0.1 mm³;
	// it does not measure channel clearance or the nominal cage's full volume.
	want := 8 * math.Pi * p["radius"] * p["half"] * p["rise"]
	decadtest.Measures(t, "cage after one cut volume", volume, units.CubicMillimeters(want/2),
		decadtest.Within(units.CubicMillimeters(want/2-0.1)))
}

var windowCases = []proofkit.Case{
	{Name: "plus-k", Params: changed(map[string]float64{"sign": 1})},
	{Name: "minus-k", Params: changed(map[string]float64{"sign": -1})},
}
var windowSolidCases = solidCasesFrom(windowCases)

func windowCorners(p map[string]float64) [][2]float64 {
	// These are the search results recorded in the spec, rounded to 0.01 mm.
	if p["sign"] > 0 {
		return [][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.57, 10.58}, {-11.57, 6.39}, {8.49, -13.67}, {11.57, -10.58}}
	}
	return [][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.54, 10.61}, {-11.54, 6.65}, {8.49, -13.37}, {11.57, -10.29}}
}
func stepWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "fixed hexagon through the searched window corners")
	corners := windowCorners(p)
	fixedPolygon(s, corners)
	sketchtest.Solve(t, s)
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	sketchtest.HasExactCuts(t, profile)
	require.Len(t, profile.Outer, 6)
	// Both sides of the comparison use the spec's rounded coordinates.
	sketchtest.MeasuresProfileArea(t, profile, polygonArea(corners), sketchtest.Within(1e-7))
}
func stepWindowCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// Cut each window from a fresh uncut sleeve. The proof does not chain the
	// four bore cuts and two window cuts into one body, as the spec records.
	tube := buildTube(t, doc, p)
	w := sketch.NewWorld()
	d := r3.Vec{Y: p["sign"]}
	across := r3.Vec{Z: 1}.Cross(d)
	f, err := r3.NewFrame(r3.Vec{}, across, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixedPolygon(s, windowCorners(p))
	tool := decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(p["radius"]+p["half"]+1))
	cut, err := decad.Cut(t.Context(), tube, tool)
	require.NoError(t, err)
	return []*decad.Body{cut}
}
func assertWindowCut(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	assertBoreSweep(t, doc, b, p)
}
