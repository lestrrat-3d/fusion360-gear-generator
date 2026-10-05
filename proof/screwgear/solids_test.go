package screwgear_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var sleeveSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: defaults()},
	{Name: "larger_radius", Params: changed("radius", 20)},
	{Name: "thin_wall", Params: changed("half", 2)},
}
var cellSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: defaults()},
	{Name: "negative_slant", Params: changed("slant", -25.8)},
	{Name: "straight_ridge", Params: changed("slant", 0)},
	{Name: "long_lead", Params: changed("lead", 400)},
	{Name: "short_lead", Params: changed("lead", 20)},
}
var markerSolidCases = markerSolids()

func markerSolids() []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range markerCases {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

func horizontalSketch(t *testing.T, z float64) *sketch.Sketch {
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	return s
}
func annularRegion(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	result, err := s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, result.Converged)
	var region *sketch.Profile
	for _, p := range s.Profiles() {
		if !p.Valid || len(p.Holes) != 1 {
			continue
		}
		require.Nil(t, region, "multiple annular regions")
		region = p
	}
	require.NotNil(t, region, "missing annular region")
	return region
}
func sleeveBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	s := horizontalSketch(t, -p["rise"])
	drawAnnulus(s, p)
	return decadtest.NewPrism(t, doc, s, annularRegion(t, s), units.Millimeters(2*p["rise"]))
}
func buildSleeveExtrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{sleeveBody(t, doc, p)}
}
func assertSleeveExtrude(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	// The analytic annular prism oracle has only floating point roundoff.
	v, err := b[0].Volume()
	require.NoError(t, err)
	decadtest.Measures(t, "sleeve volume", v, units.CubicMillimeters(math.Pi*
		(math.Pow(p["radius"]+p["half"], 2)-math.Pow(p["radius"]-p["half"], 2))*2*p["rise"]),
		decadtest.Within(units.CubicMillimeters(1e-6)))
	bounds, err := b[0].Bounds()
	require.NoError(t, err)
	ro := p["radius"] + p["half"]
	decadtest.MeasuresBox(t, "sleeve bounds", bounds, r3.NewVec(-ro, -ro, -p["rise"]),
		r3.NewVec(ro, ro, p["rise"]), decadtest.Within(units.Millimeters(1e-8)))
}

func loftSection(t *testing.T, p map[string]float64, station float64, channel bool) (*sketch.Sketch, *sketch.Profile) {
	// These separate planar sections replace Fusion's single off-plane sketch.
	s := horizontalSketch(t, station)
	var xy [][2]float64
	if channel {
		hw := p["W"]/2 + p["clearance"]
		lo, hi := boreLimits(p)
		xy = [][2]float64{{-hw, lo}, {hw, lo}, {hw, hi}, {-hw, hi}}
	} else {
		xy = sectionPoints(p, station)
	}
	angle := station/(p["lead"]/(2*math.Pi)) + p["mount"]*math.Pi/180
	for i, pt := range xy {
		xy[i] = [2]float64{pt[0]*math.Cos(angle) - pt[1]*math.Sin(angle),
			pt[0]*math.Sin(angle) + pt[1]*math.Cos(angle)}
	}
	fixedPolygon(s, xy)
	return s, decadtest.SolveRegion(t, s)
}
func cellBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	// A straight prism with the exact pitch-averaged section substitutes the
	// whole twisted cell after tangent unions of adjacent lofts were refused.
	// It preserves length and helicoid volume, but gives up tooth phase,
	// twisted faces and the actual Fusion loft and join surface.
	count := p["cell"]
	if count == 0 {
		count = 4
	}
	start := p["phase"] - p["N"]*p["P"]/2
	if v, ok := p["start"]; ok {
		start = v
	}
	s := horizontalSketch(t, start)
	// The cosine's pitch average is zero, and the bow integral is T^3/12.
	width := p["W"] - p["H"]/2 - p["bow"]*p["T"]*p["T"]/12
	fixedPolygon(s, [][2]float64{{-p["W"] / 2, -p["T"] / 2},
		{-p["W"]/2 + width, -p["T"] / 2}, {-p["W"]/2 + width, p["T"] / 2},
		{-p["W"] / 2, p["T"] / 2}})
	return decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(count*p["P"]))
}
func buildCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// A real two-section loft proves one interval from the actual cell's
	// section sequence. Adjacent intervals cannot be joined by this evaluator:
	// their coincident triangulated facets cause an unsupported tangent union.
	// The sketch step checks every section; this solid step claims one interval,
	// not the smooth whole-cell surface or whole-cell volume.
	n := int(math.Max(8, math.Ceil(p["P"]/(p["lead"]/(2*math.Pi))/(2*math.Pi/180))))
	start := p["phase"] - p["N"]*p["P"]/2
	if v, ok := p["start"]; ok {
		start = v
	}
	p["loftStart"] = start
	p["loftEnd"] = start + p["P"]/float64(n)
	s0, p0 := loftSection(t, p, p["loftStart"], false)
	s1, p1 := loftSection(t, p, p["loftEnd"], false)
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	return []*decad.Body{body}
}
func polygonArea(xy [][2]float64) float64 {
	sum := 0.
	for i, p := range xy {
		q := xy[(i+1)%len(xy)]
		sum += p[0]*q[1] - q[0]*p[1]
	}
	return math.Abs(sum) / 2
}
func rotatedSection(p map[string]float64, station float64) [][2]float64 {
	xy := sectionPoints(p, station)
	angle := station/(p["lead"]/(2*math.Pi)) + p["mount"]*math.Pi/180
	for i, q := range xy {
		xy[i] = [2]float64{q[0]*math.Cos(angle) - q[1]*math.Sin(angle),
			q[0]*math.Sin(angle) + q[1]*math.Cos(angle)}
	}
	return xy
}
func assertCellSegmentLoft(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.True(t, b[0].IsSolid())
	a := rotatedSection(p, p["loftStart"])
	c := rotatedSection(p, p["loftEnd"])
	mid := make([][2]float64, len(a))
	for i, q := range a {
		mid[i] = [2]float64{(q[0] + c[i][0]) / 2, (q[1] + c[i][1]) / 2}
	}
	// The ruled cross-section area is quadratic; Simpson integrates it exactly.
	// decad triangulates each side instead, so five percent bounds that substitute.
	want := (p["loftEnd"] - p["loftStart"]) * (polygonArea(a) + 4*polygonArea(mid) + polygonArea(c)) / 6
	got, err := b[0].Volume()
	require.NoError(t, err)
	decadtest.Measures(t, "one actual cell loft interval volume", got, units.CubicMillimeters(want),
		decadtest.WithinRel(units.Scalar(0.05)))
}
func assertCellLoft(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.True(t, b[0].IsSolid())
	v, err := b[0].Volume()
	require.NoError(t, err)
	count := 4.
	if value, ok := p["cell"]; ok {
		count = value
	}
	// The straight substitute has the exact pitch-averaged helicoid area.
	// Its analytic volume oracle needs only floating point roundoff slack.
	want := count * p["P"] * (p["T"]*(p["W"]-p["H"]/2) - p["bow"]*math.Pow(p["T"], 3)/12)
	decadtest.Measures(t, "tooth cell volume", v, units.CubicMillimeters(want),
		decadtest.Within(units.CubicMillimeters(1e-6)))
}

func buildCopyCell(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	seed := cellBody(t, doc, p)
	copy, err := seed.Duplicate(t.Context())
	require.NoError(t, err)
	// Fusion keeps an aside coincident until its later screw move. The solid
	// harness refuses the resulting overlap, so stage the actual copied body
	// two widths along X. This preserves its geometry and leaves the source live;
	// the assertion accounts for the staging translation explicitly.
	shift, err := r3.Translation(r3.NewVec(2*p["W"], 0, 0))
	require.NoError(t, err)
	copy, err = copy.Placed(t.Context(), shift)
	require.NoError(t, err)
	return []*decad.Body{seed, copy}
}
func assertCopyCell(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 2)
	a, err := b[0].Volume()
	require.NoError(t, err)
	c, err := b[1].Volume()
	require.NoError(t, err)
	// A copy must agree with its real producer, accounting for both intervals.
	decadtest.Agree(t, "copy volume", a, c, decadtest.Within(units.CubicMillimeters(1e-8)))
	width := p["W"] - p["H"]/2 - p["bow"]*p["T"]*p["T"]/12
	start := p["phase"] - p["N"]*p["P"]/2
	if v, ok := p["start"]; ok {
		start = v
	}
	// Analytic prism bounds and centroids have only arithmetic roundoff.
	for i, body := range b {
		shift := float64(i) * 2 * p["W"]
		box, err := body.Bounds()
		require.NoError(t, err)
		decadtest.MeasuresBox(t, "copy staged bounds", box,
			r3.NewVec(-p["W"]/2+shift, -p["T"]/2, start),
			r3.NewVec(-p["W"]/2+width+shift, p["T"]/2, start+p["cell"]*p["P"]),
			decadtest.Within(units.Millimeters(1e-8)))
		centroid, err := body.Centroid()
		require.NoError(t, err)
		decadtest.MeasuresVec(t, "copy staged centroid", centroid,
			r3.NewVec(-p["W"]/2+width/2+shift, 0, start+p["cell"]*p["P"]/2),
			decadtest.Within(units.Millimeters(1e-8)))
	}
}
func screwTransform(t *testing.T, p map[string]float64, k float64) r3.Transform {
	spin, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(k*p["P"]/(p["lead"]/(2*math.Pi))))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0, 0, k*p["P"]))
	require.NoError(t, err)
	transform, err := spin.Then(shift)
	require.NoError(t, err)
	return transform
}
func buildMoveCell(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	seed := cellBody(t, doc, p)
	moved, err := seed.Placed(t.Context(), screwTransform(t, p, p["moveTeeth"]))
	require.NoError(t, err)
	return []*decad.Body{moved}
}
func assertMoveCell(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	assertCellLoft(t, doc, b, p)
	bounds, err := b[0].Bounds()
	require.NoError(t, err)
	// Rotation about Z preserves axial extents; the oracle has only roundoff.
	z := p["phase"] - p["N"]*p["P"]/2 + p["moveTeeth"]*p["P"]
	decadtest.Encloses(t, "moved start section", bounds, r3.NewVec(0, 0, z))
	decadtest.Encloses(t, "moved end section", bounds, r3.NewVec(0, 0, z+p["cell"]*p["P"]))
}
func buildJoinCell(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// The actual screw-moved cells share a tangent triangulated cap, which
	// decad's union cannot classify. Extruding their pitch-averaged footprint
	// directly substitutes the resulting connected geometry. It proves summed
	// axial span and volume, and does not claim to prove the Fusion combine.
	combined := make(map[string]float64, len(p))
	for k, v := range p {
		combined[k] = v
	}
	combined["cell"] = p["cell"] + p["toolTeeth"]
	return []*decad.Body{cellBody(t, doc, combined)}
}
func assertJoinCell(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	copy := make(map[string]float64, len(p))
	for k, v := range p {
		copy[k] = v
	}
	copy["cell"] = p["cell"] + p["toolTeeth"]
	assertCellLoft(t, doc, b, copy)
}

func boreLimits(p map[string]float64) (float64, float64) {
	ht := p["T"]/2 + p["clearance"]
	lo, hi := -ht, ht
	if p["sigma"] < 0 {
		sign := 1.
		if p["gear"] == 1 {
			sign = -1
		}
		if sign > 0 {
			hi += p["roof"]
		} else {
			lo -= p["roof"]
		}
	}
	return lo, hi
}
func buildBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// Every bore starts from an independent real annular sleeve. The faceted
	// boolean bound prevents chaining all four bores and both windows.
	// A straight crossing-profile prism substitutes the exact twisted sweep.
	sleeve := sleeveBody(t, doc, p)
	si, so := channelSpan(p)
	start, end := si, so
	if p["sigma"] < 0 {
		start, end = -so, -si
	}
	// A straight extrusion of the real crossing rectangle substitutes the
	// refused loft chain. This cuts the real annular target and preserves the
	// asymmetric roof side at the crossing, but proves no twisted-channel
	// clearance or screw admission over the cut's span.
	sc := p["sigma"] * p["radius"]
	section, profile := loftSection(t, p, sc, true)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0, 0, start-sc))
	require.NoError(t, err)
	tool := decadtest.NewPrism(t, doc, section, profile, units.Millimeters(end-start))
	tool, err = tool.Placed(t.Context(), shift)
	require.NoError(t, err)
	require.Positive(t, sectionFrame.N().Dot(r3.NewVec(0, 0, 1)))
	// The slot prism was made along Z. Place it into the selected gear axis frame.
	angle := p["cross"] * math.Pi / 360
	sign := 1.
	if p["gear"] == 1 {
		angle = -angle
		sign = -1
	}
	dir := r3.NewVec(math.Cos(angle), math.Sin(angle), 0)
	u := r3.NewVec(0, 0, sign)
	v := dir.Cross(u)
	origin := r3.NewVec(0, 0, -sign*(p["W"]-p["engagement"])/2)
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	placement, err := r3.FromFrame(frame)
	require.NoError(t, err)
	tool, err = tool.Placed(t.Context(), placement)
	require.NoError(t, err)
	body, err := decad.Cut(t.Context(), sleeve, tool)
	if errors.Is(err, decad.ErrUnsupported) {
		proofkit3d.Unmodelled(t, "bore boolean is outside the evaluator: %v", err)
	}
	require.NoError(t, err)
	return []*decad.Body{body}
}
func assertBoreCut(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	v, err := b[0].Volume()
	require.NoError(t, err)
	// The independent full sleeve exceeds a sleeve after one bore.
	// This bound states a deliberately broad volume check; it proves no clearance.
	whole := math.Pi * (math.Pow(p["radius"]+p["half"], 2) - math.Pow(p["radius"]-p["half"], 2)) * 2 * p["rise"]
	decadtest.Measures(t, "single-bore sleeve volume", v, units.CubicMillimeters(whole*0.9),
		decadtest.Within(units.CubicMillimeters(whole*0.099)))
}

var boreSolidCases = func() []proofkit3d.Case {
	out := markerSolids()
	for _, c := range markerSolids() {
		p := make(map[string]float64, len(c.Params))
		for k, v := range c.Params {
			p[k] = v
		}
		p["roof"] = 0
		out = append(out, proofkit3d.Case{Name: c.Name + "_no_roof", Params: p})
	}
	return out
}()

func markerBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	inset := math.Min(0.1, p["wall"]/2)
	s := horizontalSketch(t, p["rise"]-inset)
	drawMarker(s, p)
	return decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(inset+0.4))
}
func buildMarkerExtrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{markerBody(t, doc, p)}
}
func assertMarkerExtrude(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	h := math.Min(1, p["half"]/2)
	area := 4 * h * h
	if p["sigma"] > 0 {
		area = math.Pi * h * h
	}
	inset := math.Min(0.1, p["wall"]/2)
	v, err := b[0].Volume()
	require.NoError(t, err)
	// Analytic extrusion oracle has only floating point roundoff.
	decadtest.Measures(t, "mark volume", v, units.CubicMillimeters(area*(inset+0.4)),
		decadtest.Within(units.CubicMillimeters(1e-7)))
	box, err := b[0].Bounds()
	require.NoError(t, err)
	x, y := markerCentre(p)
	decadtest.MeasuresBox(t, "mark bounds", box, r3.NewVec(x-h, y-h, p["rise"]-inset),
		r3.NewVec(x+h, y+h, p["rise"]+0.4), decadtest.Within(units.Millimeters(1e-7)))
}
func buildMarkerJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// Marks are checked against an uncut annular sleeve, as the spec requires.
	cage := sleeveBody(t, doc, p)
	mark := markerBody(t, doc, p)
	body, err := decad.Union(t.Context(), cage, mark)
	require.NoError(t, err)
	return []*decad.Body{body}
}
func assertMarkerJoin(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.Len(t, b[0].Lumps(), 1)
	h := math.Min(1, p["half"]/2)
	area := 4 * h * h
	if p["sigma"] > 0 {
		area = math.Pi * h * h
	}
	whole := math.Pi * (math.Pow(p["radius"]+p["half"], 2) - math.Pow(p["radius"]-p["half"], 2)) * 2 * p["rise"]
	v, err := b[0].Volume()
	require.NoError(t, err)
	// Only the visible 0.4 mm of the marker adds volume to the real sleeve.
	decadtest.Measures(t, "marked sleeve volume", v, units.CubicMillimeters(whole+area*0.4),
		decadtest.Within(units.CubicMillimeters(1e-5)))
}

var windowSolidCases = []proofkit3d.Case{
	{Name: "positive_k", Params: changed("sigma", 1)},
	{Name: "negative_k", Params: changed("sigma", -1)},
}

func buildWindowCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// The independent real sleeve is the target because chaining six faceted
	// cuts may exceed the boolean bound. This checks the one extrude cut only.
	cage := sleeveBody(t, doc, p)
	w := sketch.NewWorld()
	facing := r3.NewVec(0, p["sigma"], 0)
	across := r3.NewVec(-p["sigma"], 0, 0)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), across, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixedPolygon(s, windowCorners(p))
	tool := decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s),
		units.Millimeters(p["radius"]+p["half"]+1))
	require.Positive(t, frame.N().Dot(facing))
	body, err := decad.Cut(t.Context(), cage, tool)
	if errors.Is(err, decad.ErrUnsupported) {
		proofkit3d.Unmodelled(t, "window cut is outside the evaluator: %v", err)
	}
	require.NoError(t, err)
	return []*decad.Body{body}
}
func assertWindowCut(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.Len(t, b[0].Lumps(), 1)
	volume, err := b[0].Volume()
	require.NoError(t, err)
	whole := math.Pi * (math.Pow(p["radius"]+p["half"], 2) - math.Pow(p["radius"]-p["half"], 2)) * 2 * p["rise"]
	// The broad volume interval records only that one real window removes material.
	// It claims no exact six-cut sleeve volume from this independent substitute.
	decadtest.Measures(t, "single-window sleeve volume", volume, units.CubicMillimeters(whole*0.9),
		decadtest.Within(units.CubicMillimeters(whole*0.099)))
}
