package spurgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var solidCases = []proofkit3d.Case{{Name: "default", Params: defaultParams()}}

// The substitute facets the real sketch's detected tooth, including its root.
// The body uses those same root vertices, rotated by the exact transformations
// used for the tooth pattern. Both prisms therefore share identical planar
// interfaces. This proves neither analytic-circle contact nor Fusion's two-arc
// profile count; the sketch proof records the latter limit separately.
// The spline and tooth-top facets also approximate their original curves.
type toothSection struct {
	sketch  *sketch.Sketch
	profile *sketch.Profile
	area    float64
	root    [][2]float64
	outline [][2]float64
}

func newToothSection(t *testing.T, p map[string]float64) toothSection {
	source := decadtest.NewSketch(t)
	drawGearProfile(t, source, p)
	var tooth *sketch.Profile
	for _, profile := range source.Profiles() {
		for _, entity := range profile.Entities {
			if _, ok := entity.(*sketch.FitSpline); ok {
				tooth = profile
				break
			}
		}
	}
	require.NotNil(t, tooth)
	sketchtest.IsValidProfile(t, tooth)
	var boundary, fullBoundary, root [][2]float64
	for _, edge := range tooth.Outer {
		fullBoundary = append(fullBoundary, edge.Polyline[:len(edge.Polyline)-1]...)
		stride := 1
		if _, ok := edge.Entity.(*sketch.FitSpline); ok {
			stride = 8
		}
		for i := 0; i < len(edge.Polyline)-1; i += stride {
			boundary = append(boundary, edge.Polyline[i])
		}
		if _, ok := edge.Entity.(*sketch.Circle); ok {
			root = append(root, edge.Polyline...)
		}
	}
	require.GreaterOrEqual(t, len(root), 2, "the real tooth must provide the root interface")
	s, profile, area := polygonRegion(t, boundary, p)
	fullArea := polygonArea(fullBoundary, p)
	require.InDelta(t, fullArea, area, fullArea*0.001,
		"the faceted substitute must stay within 0.1%% of the source tooth area")
	return toothSection{sketch: s, profile: profile, area: area, root: root, outline: boundary}
}

func polygonArea(boundary [][2]float64, p map[string]float64) float64 {
	var twiceArea float64
	for i, xy := range boundary {
		next := boundary[(i+1)%len(boundary)]
		twiceArea += (xy[0]-p["x"])*(next[1]-p["y"]) -
			(next[0]-p["x"])*(xy[1]-p["y"])
	}
	return math.Abs(twiceArea) / 2
}

func polygonRegion(t *testing.T, boundary [][2]float64, p map[string]float64) (*sketch.Sketch, *sketch.Profile, float64) {
	t.Helper()
	s := decadtest.NewSketch(t)
	points := make([]*sketch.Point, len(boundary))
	for i, xy := range boundary {
		points[i] = s.CreatePoint(xy[0], xy[1])
		s.Fix(points[i])
	}
	for i := range boundary {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	profile := decadtest.SolveRegion(t, s)
	area := polygonArea(boundary, p)
	// The source polygon and the solved polygon use identical facets.
	// This slack covers floating arithmetic only, not curve approximation.
	sketchtest.MeasuresProfileArea(t, profile, area, sketchtest.WithinRel(1e-8))
	return s, profile, area
}

func toothRegion(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile, float64) {
	section := newToothSection(t, p)
	return section.sketch, section.profile, section.area
}

func toothBodyFromSection(t *testing.T, doc *decad.Document, p map[string]float64, section toothSection) *decad.Body {
	return decadtest.NewPrism(t, doc, section.sketch, section.profile, units.Millimeters(p["height"]))
}

func toothBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	return toothBodyFromSection(t, doc, p, newToothSection(t, p))
}

func toothRotation(t *testing.T, p map[string]float64, index int) r3.Transform {
	t.Helper()
	if index == 0 {
		return r3.Identity()
	}
	rotation, err := r3.RotationAround(r3.NewVec(p["x"], p["y"], 0), r3.NewVec(0, 0, 1),
		units.Radians(2*math.Pi*float64(index)/p["teeth"]))
	require.NoError(t, err)
	return rotation
}

func rootFacetBoundary(t *testing.T, p map[string]float64, section toothSection) [][2]float64 {
	t.Helper()
	// Include every seed-root vertex under every planned tooth transform.
	// Between tooth interfaces a single chord approximates each root valley.
	// No independently sampled circle introduces extra vertices along an
	// interface or places a curved surface across a tooth's planar facets.
	unique := make(map[[2]float64]struct{})
	var boundary [][2]float64
	for i := 0; i < int(p["teeth"]); i++ {
		rotation := toothRotation(t, p, i)
		for _, xy := range section.root {
			point := xy
			if i != 0 {
				v := rotation.Apply(r3.NewVec(xy[0], xy[1], 0))
				point = [2]float64{v.X, v.Y}
			}
			if _, exists := unique[point]; exists {
				continue
			}
			unique[point] = struct{}{}
			boundary = append(boundary, point)
		}
	}
	sort.Slice(boundary, func(i, j int) bool {
		return math.Atan2(boundary[i][1]-p["y"], boundary[i][0]-p["x"]) <
			math.Atan2(boundary[j][1]-p["y"], boundary[j][0]-p["x"])
	})
	return boundary
}

func rootRegion(t *testing.T, p map[string]float64, section toothSection) (*sketch.Sketch, *sketch.Profile, float64) {
	return polygonRegion(t, rootFacetBoundary(t, p, section), p)
}

func rootBodyFromSection(t *testing.T, doc *decad.Document, p map[string]float64, section toothSection) *decad.Body {
	s, profile, _ := rootRegion(t, p, section)
	return decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]))
}

func rootBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	return rootBodyFromSection(t, doc, p, newToothSection(t, p))
}

func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{toothBody(t, doc, p)}
}

func assertExtrudeTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	_, _, area := toothRegion(t, p)
	measureVolume(t, "Extrude tooth", bodies[0], area*p["height"])
}

func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{rootBody(t, doc, p)}
}

func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	section := newToothSection(t, p)
	_, _, area := rootRegion(t, p, section)
	measureVolume(t, "Gear Body", bodies[0], area*p["height"])
	boundary := rootFacetBoundary(t, p, section)
	minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
	for _, xy := range boundary {
		minX = math.Min(minX, xy[0])
		minY = math.Min(minY, xy[1])
		maxX = math.Max(maxX, xy[0])
		maxY = math.Max(maxY, xy[1])
	}
	decadtest.MeasuresBounds(t, bodies[0], r3.NewVec(minX, minY, 0), r3.NewVec(maxX, maxY, p["height"]),
		decadtest.Within(units.Millimeters(1e-7)))
}

func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return patternFromSection(t, doc, p, newToothSection(t, p))
}

func patternFromSection(t *testing.T, doc *decad.Document, p map[string]float64, section toothSection) []*decad.Body {
	seed := toothBodyFromSection(t, doc, p, section)
	bodies := []*decad.Body{seed}
	for i := 1; i < int(p["teeth"]); i++ {
		rotation := toothRotation(t, p, i)
		copy, err := seed.PlacedCopy(t.Context(), rotation)
		require.NoError(t, err)
		bodies = append(bodies, copy)
	}
	return bodies
}

func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, int(p["teeth"]))
	seed, err := bodies[0].Volume()
	require.NoError(t, err)
	for _, body := range bodies[1:] {
		volume, err := body.Volume()
		require.NoError(t, err)
		decadtest.Agree(t, "pattern tooth volume", volume, seed, decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	section := newToothSection(t, p)
	boundary := finalGearBoundary(t, p, section)
	s, profile, _ := polygonRegion(t, boundary, p)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]))}
}

func finalGearBoundary(t *testing.T, p map[string]float64, section toothSection) [][2]float64 {
	t.Helper()
	type point = [2]float64
	type edge = [2]point
	canonical := func(a, b point) edge {
		if a[0] > b[0] || (a[0] == b[0] && a[1] > b[1]) {
			a, b = b, a
		}
		return edge{a, b}
	}
	rotate := func(points []point, index int) []point {
		out := make([]point, len(points))
		transform := toothRotation(t, p, index)
		for i, xy := range points {
			if index == 0 {
				out[i] = xy
				continue
			}
			v := transform.Apply(r3.NewVec(xy[0], xy[1], 0))
			out[i] = point{v.X, v.Y}
		}
		return out
	}
	rootEdges := map[edge]struct{}{}
	for index := 0; index < int(p["teeth"]); index++ {
		root := rotate(section.root, index)
		for i := 1; i < len(root); i++ {
			if root[i-1] != root[i] {
				rootEdges[canonical(root[i-1], root[i])] = struct{}{}
			}
		}
	}
	adjacent := map[point][]point{}
	seen := map[edge]struct{}{}
	add := func(a, b point) {
		if a == b {
			return
		}
		key := canonical(a, b)
		if _, isRoot := rootEdges[key]; isRoot {
			return
		}
		if _, exists := seen[key]; exists {
			return
		}
		seen[key] = struct{}{}
		adjacent[a] = append(adjacent[a], b)
		adjacent[b] = append(adjacent[b], a)
	}
	root := rootFacetBoundary(t, p, section)
	for i, xy := range root {
		add(xy, root[(i+1)%len(root)])
	}
	for index := 0; index < int(p["teeth"]); index++ {
		outline := rotate(section.outline, index)
		for i, xy := range outline {
			add(xy, outline[(i+1)%len(outline)])
		}
	}
	require.NotEmpty(t, adjacent)
	var start point
	first := true
	for xy, neighbors := range adjacent {
		require.Len(t, neighbors, 2, "the final contour must have two edges at every point")
		if first || xy[0] < start[0] || (xy[0] == start[0] && xy[1] < start[1]) {
			start, first = xy, false
		}
	}
	boundary := make([]point, 0, len(adjacent))
	current := start
	var previous point
	for {
		boundary = append(boundary, current)
		neighbors := adjacent[current]
		next := neighbors[0]
		if len(boundary) > 1 && next == previous {
			next = neighbors[1]
		}
		if next == start {
			break
		}
		previous, current = current, next
		require.LessOrEqual(t, len(boundary), len(adjacent), "the contour must close")
	}
	require.Len(t, boundary, len(adjacent), "the final contour must include every edge")
	return boundary
}

func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	section := newToothSection(t, p)
	_, _, rootArea := rootRegion(t, p, section)
	measureVolume(t, "combined gear", bodies[0], (rootArea+p["teeth"]*section.area)*p["height"])
}

func stepRootFillets(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	bodies := stepCombineTeeth(t, doc, p)
	radius := dimensions(p).Root * (math.Pi/p["teeth"] - 2*(math.Tan(p["pressure"])-p["pressure"])) * 0.9 / 2
	if radius <= 0 {
		return bodies
	}
	query := decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(0, 0, 1)))
	edges, err := query.SelectEdges(bodies[0])
	require.NoError(t, err)
	require.NotEmpty(t, edges, "the final contour must expose concave root edges for the fillet")
	filleted, err := bodies[0].Fillet(t.Context(), query, units.Millimeters(radius))
	require.NoError(t, err)
	return []*decad.Body{filleted}
}

func assertRootFillets(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	// Concave root rounding adds material. Surface validity is checked by the
	// solid gate; the axial extent must survive the operation exactly.
	box, err := bodies[0].Bounds()
	require.NoError(t, err)
	decadtest.Encloses(t, "root fillet gear extent", box, r3.NewVec(p["x"], p["y"], p["height"]))
}

func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	section := newToothSection(t, p)
	boundary := finalGearBoundary(t, p, section)
	if p["bore"] <= 0 {
		s, profile, _ := polygonRegion(t, boundary, p)
		return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]))}
	}
	// The earlier GO steps prove the tooth pattern and root fillets. Decad's
	// boolean scene exceeds its segment cap after those fillets, so this step
	// proves the final section's bore as a hole in one profile. It does not
	// exercise Fusion's separate Cut feature or the post-fillet operation order.
	s := decadtest.NewSketch(t)
	points := make([]*sketch.Point, len(boundary))
	for i, xy := range boundary {
		points[i] = s.CreatePoint(xy[0], xy[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	center := s.CreatePoint(p["x"], p["y"])
	s.Fix(center)
	s.CreateCircle(center, p["bore"]/2)
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile, "the final gear section must enclose one bore hole")
	sketchtest.IsValidProfile(t, profile)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["height"]))}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	section := newToothSection(t, p)
	area := polygonArea(finalGearBoundary(t, p, section), p)
	if p["bore"] > 0 {
		area -= math.Pi * p["bore"] * p["bore"] / 4
	}
	measureVolume(t, "final gear with through bore", bodies[0], area*p["height"])
}

func measureVolume(t *testing.T, label string, body *decad.Body, want float64) {
	t.Helper()
	v, err := body.Volume()
	require.NoError(t, err)
	// The oracle measures the actual facets, not an analytic gear. The
	// evaluator adds its own bound; this slack covers floating arithmetic.
	decadtest.Measures(t, label, v, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-8)))
}

// Completed-gear chamfer is not modelled in this draft. The tested engine
// recipe only establishes one cap per independent cylinder, not both caps of
// a patterned, filleted, bored gear. The current sealed EdgeSelector API also
// gives no confirmed union of both cap sets excluding only bore circles.
// Do not replace the completed gear with a cylinder and call it proven.

// Command setup, construction planes, the construction axis, and final cleanup
// are PROSE timeline/setup steps: their Fusion document and visibility state
// has no corresponding geometric assertion in these engine APIs.
