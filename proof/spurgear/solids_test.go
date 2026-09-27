package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var boreCases = []proofkit3d.Case{
	{Name: "default_no_bore", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "thickness": 10}},
	{Name: "negative_bore_guard", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "thickness": 10, "bore": -1}},
	{Name: "default_with_bore", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "thickness": 10, "bore": 4}},
	{Name: "fine_large_bore", Params: map[string]float64{"module": 0.5, "teeth": 28, "pressure": 25, "thickness": 3, "bore": 5}},
}

var bodyCases = []proofkit3d.Case{
	{Name: "default", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "thickness": 10}},
	{Name: "fine", Params: map[string]float64{"module": 0.5, "teeth": 28, "pressure": 25, "thickness": 3}},
}

// stepExtrudeBody substitutes a whole root circle for Fusion's two split root
// arcs. stepGearProfile proves that the actual sketch closes the split disc;
// this solid check proves the extrusion's extent and root-disc volume.
// Directly extruding that sketch's tooth region fails in the pinned decad
// engine: its root-circle fragment has an uncertified trim (TExact = false).
// This disc substitute cannot prove the tooth prism or its Fusion profile pick.
func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	t.Helper()
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	s := decadtest.NewSketch(t)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	circle := s.CreateCircle(center, d.Root)
	s.AddConstraint(sketch.NewDiameter(circle, 2*d.Root))
	profile := decadtest.SolveRegion(t, s)
	// The full circle and Fusion's two cut arcs enclose the same root disc.
	sketchtest.Measures(t, "root disc area", profile.Area, math.Pi*d.Root*d.Root,
		sketchtest.WithinRel(1e-8))
	return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["thickness"]))}
}

func assertExtrudeBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	t.Helper()
	require.Len(t, bodies, 1)
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	volume, err := bodies[0].Volume()
	require.NoError(t, err)
	// Analytic cylinder volume has less than 1e-8 relative float error.
	decadtest.Measures(t, "root disc volume", volume,
		units.CubicMillimeters(math.Pi*d.Root*d.Root*p["thickness"]),
		decadtest.WithinRel(units.Scalar(1e-8)))
}

// stepBore uses the tested annular-profile construction from proof/examples.
// It replaces Fusion's separate bore cut, which decad rejects for sufficiently
// segmented gears. The substitute proves the through-hole and its removed
// volume in the root disc; it does not prove Fusion's cut feature, participant
// body selection, or timeline ordering. The patterned teeth and fillets are
// outside this isolated bore construction.
func stepBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	t.Helper()
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	boreRadius := p["bore"] / 2
	s := decadtest.NewSketch(t)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	outer := s.CreateCircle(center, d.Root)
	s.AddConstraint(sketch.NewDiameter(outer, 2*d.Root))
	if boreRadius > 0 {
		inner := s.CreateCircle(center, boreRadius)
		s.AddConstraint(sketch.NewDiameter(inner, 2*boreRadius))
	}
	sketchtest.Solve(t, s)
	var region *sketch.Profile
	expectedHoles := 0
	if boreRadius > 0 {
		expectedHoles = 1
	}
	for _, candidate := range s.Profiles() {
		if !candidate.Valid || len(candidate.Holes) != expectedHoles {
			continue
		}
		if region != nil {
			t.Fatal("multiple root-disc regions with the expected bore count")
		}
		region = candidate
	}
	require.NotNil(t, region, "the root disc must have exactly the requested hole count")
	require.Len(t, region.Holes, expectedHoles)
	wantArea := math.Pi * (d.Root*d.Root - math.Max(0, boreRadius)*math.Max(0, boreRadius))
	// Circle area uses an analytic formula; 1e-8 relative slack covers its float operations.
	sketchtest.Measures(t, "root disc profile area", region.Area, wantArea, sketchtest.WithinRel(1e-8))
	body := decadtest.NewPrism(t, doc, s, region, units.Millimeters(p["thickness"]))
	return []*decad.Body{body}
}

func assertBore(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	t.Helper()
	require.Len(t, bodies, 1)
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	boreRadius := math.Max(0, p["bore"]/2)
	wantVolume := math.Pi * (d.Root*d.Root - boreRadius*boreRadius) * p["thickness"]
	volume, err := bodies[0].Volume()
	require.NoError(t, err)
	// The extruded analytic circle formula accumulates floating-point error below 1e-8 relative.
	decadtest.Measures(t, "bored root disc volume", volume, units.CubicMillimeters(wantVolume),
		decadtest.WithinRel(units.Scalar(1e-8)))
}
