package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
)

// Root fillets and the completed-gear chamfer remain PROSE steps. The current
// decad evaluator only fillets/chamfers straight-prism receivers; the actual
// helical loft and boolean union are non-prism bodies. A straight-prism example
// would remove the requested helix and its joined root edges, so it would not
// establish either completed-gear operation. No equivalent substitute has
// passed that boundary. The proof does not claim Fusion BRep edge selection,
// especially the two end-cap bore-edge exclusions after root fillets.

var solidCases = []proofkit3d.Case{
	{Name: "zero", Params: map[string]float64{"angle": 0, "height": 10}},
	{Name: "right", Params: map[string]float64{"angle": 14.5 * math.Pi / 180, "height": 10}},
	{Name: "left", Params: map[string]float64{"angle": -14.5 * math.Pi / 180, "height": 10}},
	{Name: "thin", Params: map[string]float64{"angle": 14.5 * math.Pi / 180, "height": 2}},
}

var boreCases = []proofkit3d.Case{
	{Name: "disabled", Params: map[string]float64{"angle": 14.5 * math.Pi / 180, "height": 10, "bore": 0}},
	{Name: "right", Params: map[string]float64{"angle": 14.5 * math.Pi / 180, "height": 10, "bore": 1.5}},
	{Name: "left", Params: map[string]float64{"angle": -14.5 * math.Pi / 180, "height": 10, "bore": 1.5}},
}

// The bore step stays serial: its build records the consumed body's bounded
// volume for the assertion after the cut and common solid gate have completed.
var boreExpected decad.Measurement

func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	body := stepCombineTeeth(t, doc, p)[0]
	var err error
	boreExpected, err = body.Volume()
	if err != nil {
		t.Fatal(err)
	}
	if p["bore"] <= 0 {
		return []*decad.Body{body}
	}
	s := decadtest.NewSketch(t)
	center := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(center, s.Origin()))
	circle := s.CreateCircle(center, p["bore"])
	s.AddConstraint(sketch.NewRadius(circle, p["bore"]))
	profile := decadtest.SolveRegion(t, s)
	// The cutting cylinder extends past the proof-only root overhang so it
	// reproduces a through bore. Its extent is not Fusion's to-entity feature.
	tool := decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]+.04))
	shift, err := r3.Translation(r3.NewVec(0, 0, -.02))
	if err != nil {
		t.Fatal(err)
	}
	tool, err = tool.Placed(t.Context(), shift)
	if err != nil {
		t.Fatal(err)
	}
	body, err = decad.Cut(t.Context(), body, tool)
	if err != nil {
		t.Fatal(err)
	}
	removed := units.CubicMillimeters(math.Pi * p["bore"] * p["bore"] * (p["height"] + .02))
	boreExpected.Value, err = boreExpected.Value.Sub(removed)
	if err != nil {
		t.Fatal(err)
	}
	return []*decad.Body{body}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 || len(doc.Bodies()) != 1 {
		t.Fatal("through bore must leave one gear body")
	}
	v, err := bodies[0].Volume()
	if err != nil {
		t.Fatal(err)
	}
	// Subtracting the exact cylindrical bore preserves the original reading's
	// bound. Agree includes both it and the cut body's independent bound.
	decadtest.Agree(t, "gear through-bore volume", v, boreExpected, decadtest.Within(units.CubicMillimeters(1e-7)))
}

// toothPolygon substitutes chords for the fitted involutes and circular cap/root.
// decad Loft does not accept free-form section pairs. This preserves all 15
// authored involute samples and exact radial endpoints, but does not prove
// the spline surface or Fusion's BRep edge counts. The sketch proof checks the
// actual six-curve profile independently, without this substitution.
func toothPolygon() []involute.Pt {
	d := involute.Derive(1, 17, 20*math.Pi/180)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, 17, 15, 0)
	rootRight := involute.Pt{X: right[0].X * d.Root / d.Base, Y: right[0].Y * d.Root / d.Base}
	rootLeft := involute.Pt{X: left[0].X * d.Root / d.Base, Y: left[0].Y * d.Root / d.Base}
	pts := []involute.Pt{rootRight}
	pts = append(pts, right...)
	start := math.Atan2(right[14].Y, right[14].X)
	end := math.Atan2(left[14].Y, left[14].X)
	for i := 1; i < 4; i++ {
		a := start + (end-start)*float64(i)/4
		pts = append(pts, involute.Pt{X: d.Tip * math.Cos(a), Y: d.Tip * math.Sin(a)})
	}
	for i := len(left) - 1; i >= 0; i-- {
		pts = append(pts, left[i])
	}
	pts = append(pts, rootLeft)
	start, end = math.Atan2(rootLeft.Y, rootLeft.X), math.Atan2(rootRight.Y, rootRight.X)
	for i := 1; i < 4; i++ {
		a := start + (end-start)*float64(i)/4
		pts = append(pts, involute.Pt{X: d.Root * math.Cos(a), Y: d.Root * math.Sin(a)})
	}
	return pts
}

func polygonSection(t *testing.T, z, angle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := decadtest.NewSketch(t)
	if z != 0 {
		plane, err := s.World().CreateOffsetPlane(s.Plane(), z)
		if err != nil {
			t.Fatal(err)
		}
		s, err = s.World().CreateSketch(plane)
		if err != nil {
			t.Fatal(err)
		}
	}
	pts := toothPolygon()
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		x, y := involute.Rotate(p.X, p.Y, angle)
		points[i] = s.CreatePoint(x, y)
		s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), points[i], x), sketch.NewVerticalDistance(s.Origin(), points[i], y))
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	profile := decadtest.SolveRegion(t, s)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, polygonArea(pts), sketchtest.Within(1e-8))
	return s, profile
}

func polygonArea(pts []involute.Pt) float64 {
	area := 0.0
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		area += p.X*q.Y - q.X*p.Y
	}
	return math.Abs(area) / 2
}

func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	bottom, bp := polygonSection(t, 0, 0)
	top, tp := polygonSection(t, p["height"], p["angle"])
	body, err := doc.Loft(t.Context(), bottom, bp, top, tp)
	if err != nil {
		t.Fatal(err)
	}
	return []*decad.Body{body}
}

func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("tooth bodies: got %d, want 1", len(bodies))
	}
	// decad joins each corresponding polygon edge with two planar triangles.
	// These differ from continuously ruled quadrilaterals when the section twists.
	// The signed tetrahedron sum is the continuous volume minus
	// height*sin(angle)*sum(squared edge length)/12 for this diagonal direction.
	// This proves the faceted substitute, not the Fusion spline loft's volume.
	// The formula is exact for this triangulation; slack covers roundoff only.
	pts := toothPolygon()
	squaredEdges := 0.0
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		squaredEdges += (b.X-a.X)*(b.X-a.X) + (b.Y-a.Y)*(b.Y-a.Y)
	}
	want := polygonArea(pts)*p["height"]*(2+math.Cos(p["angle"]))/3 -
		p["height"]*math.Sin(p["angle"])*squaredEdges/12
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(want), decadtest.Within(units.CubicMillimeters(1e-7)))
}

func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := decadtest.NewSketch(t)
	d := involute.Derive(1, 17, 20*math.Pi/180)
	center := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(center, s.Origin()))
	circle := s.CreateCircle(center, d.Root)
	s.AddConstraint(sketch.NewRadius(circle, d.Root))
	profile := decadtest.SolveRegion(t, s)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]))}
}

func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := involute.Derive(1, 17, 20*math.Pi/180)
	// The circle-cylinder formula is exact; slack covers floating-point evaluation.
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(math.Pi*d.Root*d.Root*p["height"]), decadtest.Within(units.CubicMillimeters(1e-7)))
	decadtest.MeasuresBounds(t, bodies[0], r3.NewVec(-d.Root, -d.Root, 0), r3.NewVec(d.Root, d.Root, p["height"]), decadtest.Within(units.Millimeters(1e-7)))
}

func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	seed := stepLoftTooth(t, doc, p)[0]
	seedVolume, err := seed.Volume()
	if err != nil {
		t.Fatal(err)
	}
	// The evaluator cannot decide separation of these faceted copies in a
	// shared document (undecided_pair). Independent documents prove each of the
	// 17 placements and its solidity, but not mutual separation of the pattern.
	for i := 1; i < 17; i++ {
		copyDoc := decad.New()
		copySeed := stepLoftTooth(t, copyDoc, p)[0]
		rotation, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(2*math.Pi*float64(i)/17))
		if err != nil {
			t.Fatal(err)
		}
		copy, err := copySeed.Placed(t.Context(), rotation)
		if err != nil {
			t.Fatal(err)
		}
		proofkit3d.RequireSolid(t, copyDoc, []*decad.Body{copy})
		v, err := copy.Volume()
		if err != nil {
			t.Fatal(err)
		}
		decadtest.Agree(t, "pattern copy volume", v, seedVolume, decadtest.Within(units.CubicMillimeters(1e-7)))
		lo, hi := r3.NewVec(math.Inf(1), math.Inf(1), 0), r3.NewVec(math.Inf(-1), math.Inf(-1), p["height"])
		for _, vertex := range toothPolygon() {
			for _, twist := range []float64{0, p["angle"]} {
				x, y := involute.Rotate(vertex.X, vertex.Y, 2*math.Pi*float64(i)/17+twist)
				lo.X, lo.Y = math.Min(lo.X, x), math.Min(lo.Y, y)
				hi.X, hi.Y = math.Max(hi.X, x), math.Max(hi.Y, y)
			}
		}
		// A faceted loft's coordinate extrema occur at section vertices.
		decadtest.MeasuresBounds(t, copy, lo, hi, decadtest.Within(units.Millimeters(1e-7)))
	}
	return []*decad.Body{seed}
}

func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	assertLoftTooth(t, doc, bodies, p)
}

func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	body := facetedRootBody(t, doc, p)
	for i := 0; i < 17; i++ {
		tooth := stepLoftTooth(t, doc, p)[0]
		if i > 0 {
			rotation, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(2*math.Pi*float64(i)/17))
			if err != nil {
				t.Fatal(err)
			}
			tooth, err = tooth.Placed(t.Context(), rotation)
			if err != nil {
				t.Fatal(err)
			}
		}
		combined, err := decad.Union(t.Context(), body, tooth)
		if err != nil {
			t.Fatalf("combine tooth %d: %v", i, err)
		}
		body = combined
	}
	return []*decad.Body{body}
}

// facetedRootBody substitutes a circumscribed 68-sided prism because the exact
// cylinder/tooth union is refused at near-chord contact. Its inradius is the
// specified 7.25 mm root radius, and its maximum outward radial error is
// 7.25*(sec(pi/68)-1), about 0.00775 mm. It preserves the root envelope with
// that bound, but loses the exact cylindrical root surface.
// Coplanar overlapping caps are also refused by the boolean evaluator. The
// root prism therefore extends 0.01 mm beyond each tooth cap, adding exactly
// 0.02 mm to its axial extent. Tooth thickness is unchanged, but the proof
// no longer establishes flush root/tooth caps at either end of the Fusion gear.
// A 0.01-radian polygon phase avoids a collinear intermediate cap triangle
// found at phase zero. Rotation preserves the stated radial envelope bounds.
func facetedRootBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	s := decadtest.NewSketch(t)
	const count = 68
	const phase = 0.01
	radius := 7.25 / math.Cos(math.Pi/count)
	points := make([]*sketch.Point, count)
	for i := range points {
		a := phase + 2*math.Pi*float64(i)/count
		x, y := radius*math.Cos(a), radius*math.Sin(a)
		points[i] = s.CreatePoint(x, y)
		s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), points[i], x), sketch.NewVerticalDistance(s.Origin(), points[i], y))
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%count])
	}
	profile := decadtest.SolveRegion(t, s)
	// The regular polygon area is exact; slack covers the numerical solve.
	sketchtest.MeasuresProfileArea(t, profile, count*7.25*7.25*math.Tan(math.Pi/count), sketchtest.Within(1e-7))
	const overhang = 0.01
	body := decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]+2*overhang))
	shift, err := r3.Translation(r3.NewVec(0, 0, -overhang))
	if err != nil {
		t.Fatal(err)
	}
	body, err = body.Placed(t.Context(), shift)
	if err != nil {
		t.Fatal(err)
	}
	// Since 68 is divisible by four and phase is below half a polygon sector,
	// the four coordinate extrema are radius*cos(phase). This box is exact.
	coordinateExtent := radius * math.Cos(phase)
	decadtest.MeasuresBounds(t, body, r3.NewVec(-coordinateExtent, -coordinateExtent, -overhang),
		r3.NewVec(coordinateExtent, coordinateExtent, p["height"]+overhang), decadtest.Within(units.Millimeters(1e-7)))
	return body
}

func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 || len(doc.Bodies()) != 1 {
		t.Fatal("combine must leave exactly one gear body")
	}
	if len(bodies[0].Lumps()) != 1 {
		t.Fatal("combine must leave one connected lump")
	}
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatal(err)
	}
	decadtest.Encloses(t, "combined lower cap", box, r3.NewVec(0, 0, 0))
	decadtest.Encloses(t, "combined upper cap", box, r3.NewVec(0, 0, p["height"]))
}
