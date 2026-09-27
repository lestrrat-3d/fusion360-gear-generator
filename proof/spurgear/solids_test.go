package spurgear_test

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
	"github.com/stretchr/testify/require"
)

var solidCases = []proofkit3d.Case{
	{Name: "standard", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10}},
	{Name: "embedded_count", Params: map[string]float64{"module": 1, "teeth": 60, "pressure": 20, "samples": 15, "thickness": 3}},
	{Name: "embedded_pressure", Params: map[string]float64{"module": 0.5, "teeth": 30, "pressure": 30, "samples": 5, "thickness": 2}},
}

var boreSolidCases = []proofkit3d.Case{
	{Name: "no_bore", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10, "bore": 0}},
	{Name: "bore", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10, "bore": 2}},
	{Name: "embedded_bore", Params: map[string]float64{"module": 0.5, "teeth": 30, "pressure": 30, "samples": 5, "thickness": 2, "bore": 1}},
}

var chamferCases = []proofkit3d.Case{
	{Name: "disabled", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10}},
	{Name: "front_cap", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10, "chamfer": 0.02, "front": 1}},
	{Name: "back_cap", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10, "chamfer": 0.02}},
	{Name: "bore_exclusion", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "thickness": 10, "chamfer": 0.02, "bore": 2}},
}

// These steps remain PROSE: Fusion's target-plane normalization, construction
// end-plane feature, and center-axis feature have no timeline analogue in decad.
// The neighboring prism proofs check their common axial extent and center.
// They do not prove Fusion occurrence context, visibility, or construction handles.

// The chamfer step is serial because these pre-feature readings are carried
// into its assertion after the chamfer retires the original body.
var chamferBounds decad.Box
var chamferVertices int

func stepChamferCompletedGear(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	if p["bore"] > 0 {
		proofkit3d.Unmodelled(t, "decad's sealed conjunction-only edge selector cannot select both outer cap loops while excluding the bore loops")
	}
	// A cap chamfer on the completed filleted gear was attempted and refused:
	// "the offset changes the section's topology; a trimmed-offset kernel is
	// not available". Use the actual root-disc extrusion from the proven Gear
	// Profile as the narrow solid demonstration. This is not a proof of the
	// completed gear's chamfer: tooth caps, root fillets, both caps on one body,
	// and bore-edge exclusion remain outside the demonstrated construction.
	var body *decad.Body
	if p["chamfer"] <= 0 {
		body = stepRootFillets(t, doc, p)[0]
	} else {
		body = stepExtrudeBody(t, doc, p)[0]
	}
	var err error
	chamferBounds, err = body.Bounds()
	require.NoError(t, err)
	chamferVertices = len(body.Vertices())
	if p["chamfer"] <= 0 {
		return []*decad.Body{body}
	}
	// The public selector cannot OR CapStart and CapEnd. A second chamfer on
	// a cap-blend body is also refused. As in the tested cap example, build
	// each end on a separate root disc. This proves each end's setback,
	// analytic removed volume, and solid verdict on that prerequisite body.
	feature := decad.CapEnd(body)
	if p["front"] != 0 {
		feature = decad.CapStart(body)
	}
	result, err := body.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(feature)), units.Millimeters(p["chamfer"]))
	require.NoError(t, err)
	return []*decad.Body{result}
}

func assertChamferCompletedGear(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	bounds, err := bodies[0].Bounds()
	require.NoError(t, err)
	// The opposite, unchamfered cap keeps the original bounding box. Include
	// that box's proven bound as well as floating-point formula error.
	slack, err := chamferBounds.Bound.Add(units.Millimeters(1e-8))
	require.NoError(t, err)
	decadtest.MeasuresBox(t, "one-cap demonstration bounds", bounds, chamferBounds.Min, chamferBounds.Max, decadtest.Within(slack))
	if p["chamfer"] <= 0 {
		return
	}
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	r, h, setback := d.Root, p["thickness"], p["chamfer"]
	// The cap band is an exact conical frustum. Its analytic volume incurs
	// only floating-point roundoff; decadtest includes the reading's bound.
	wantVolume := math.Pi*r*r*(h-setback) + math.Pi*setback/3*(r*r+r*(r-setback)+(r-setback)*(r-setback))
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(wantVolume),
		decadtest.WithinRel(units.Scalar(1e-8)))
	require.Greater(t, len(bodies[0].Vertices()), chamferVertices, "chamfer adds the offset ring")
	level := p["thickness"] - p["chamfer"]
	if p["front"] != 0 {
		level = p["chamfer"]
	}
	interior := 0
	for _, vertex := range bodies[0].Vertices() {
		position := vertex.Position()
		if position.Value.Z == 0 || position.Value.Z == p["thickness"] {
			continue
		}
		interior++
		// The interface ring lies exactly one setback inside its cap. X and Y
		// are deliberately unconstrained here; only the axial setback is checked.
		decadtest.MeasuresVec(t, "chamfer interface axial level", position,
			r3.NewVec(position.Value.X, position.Value.Y, level), decadtest.Within(units.Millimeters(1e-8)))
	}
	require.Positive(t, interior, "chamfer produced an interior interface ring")
}

func filletRadius(p map[string]float64) float64 {
	pressure := p["pressure"] * math.Pi / 180
	d := involute.Derive(p["module"], p["teeth"], pressure)
	return d.Root * (math.Pi/p["teeth"] - 2*(math.Tan(pressure)-pressure)) / 2 * 0.9
}

func stepRootFillets(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	var body *decad.Body
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	if d.Embedded() && filletRadius(p) > 0 {
		// The embedded flank's first clipped chord can be shorter than the
		// fillet setback, and decad cannot merge that rewrite into its next
		// chord. Coalesce only the initial flank chords until their span is at
		// least two fillet radii. Keep the root contact and specified radius.
		// This proves root rounding on a coarser local flank approximation.
		s := gearOutline(t, p, 2*filletRadius(p))
		body = decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(p["thickness"]))
	} else {
		body = stepJoinTeeth(t, doc, p)[0]
	}
	return []*decad.Body{roundRootBody(t, body, p)}
}

func roundRootBody(t *testing.T, body *decad.Body, p map[string]float64) *decad.Body {
	radius := filletRadius(p)
	if radius <= 0 {
		return body
	}
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	axial, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).SelectEdges(body)
	require.NoError(t, err)
	var corners []r3.Vec
	for _, edge := range axial {
		for _, face := range edge.Faces() {
			cylinder, ok := face.Surface().(decad.Cylinder)
			// Fusion's 0.0001 cm radius tolerance is 0.001 mm here.
			if ok && math.Abs(cylinder.Radius.Base()-d.Root) <= 0.001 {
				corners = append(corners, edge.Start().Position().Value)
				break
			}
		}
	}
	require.Len(t, corners, 2*int(p["teeth"]), "axial root-cylinder corners")
	for _, point := range corners {
		// EndpointAt requires bit-exact coordinates, so use the actual body's
		// vertex, not a independently recomputed trigonometric position.
		selected := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(point)).Exactly(1)
		next, err := body.Fillet(t.Context(), selected, units.Millimeters(radius))
		require.NoError(t, err)
		body = next
	}
	return body
}

func assertRootFillets(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	radius := filletRadius(p)
	if radius <= 0 {
		assertJoinTeeth(t, doc, bodies, p)
		return
	}
	rounded := 0
	for _, face := range bodies[0].Faces() {
		cylinder, ok := face.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		// Surface radius is the feature's typed analytic parameter, not an
		// interval measurement. The solid gate checks the resulting topology.
		if math.Abs(cylinder.Radius.Base()-radius) <= 1e-8 {
			rounded++
		}
	}
	require.Equal(t, 2*int(p["teeth"]), rounded, "one fillet cylinder per root corner")
}

// The bore step is serial because its assertion needs the pre-cut volume
// reading of the body the cut retires. Keep that reading's bound in the oracle.
var boreExpected decad.Measurement

func stepCutBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	comparison := doc
	if p["bore"] > 0 {
		comparison = decad.New()
	}
	gear := stepRootFillets(t, comparison, p)[0]
	before, err := gear.Volume()
	require.NoError(t, err)
	boreExpected = before
	if p["bore"] <= 0 {
		return []*decad.Body{gear}
	}
	proofkit3d.RequireSolid(t, comparison, []*decad.Body{gear})
	// The explicit cut exceeds decad's 4096-segment analytic arrangement cap.
	// Draw the bore in the complete gear outline and extrude the holed region,
	// then round its root corners. This substitutes the final geometry and
	// proves the removed volume, not Fusion's cut participant selection or
	// its timeline order after filleting. The comparison body has its own doc.
	s := gearOutline(t, p)
	projected := s.CreateReferencePoint(0, 0, "Tools anchor for bore")
	circle := s.CreateCircle(projected, p["bore"]/2)
	s.AddConstraint(sketch.NewDiameter(circle, p["bore"]))
	sketchtest.Solve(t, s)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) != 1 {
			continue
		}
		require.Nil(t, profile, "one holed gear region")
		sketchtest.IsValidProfile(t, candidate)
		sketchtest.HasExactCuts(t, candidate)
		profile = candidate
	}
	require.NotNil(t, profile, "gear profile with bore hole")
	require.Len(t, profile.Holes[0], 1, "one bore circle")
	cut := decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["thickness"]))
	cut = roundRootBody(t, cut, p)
	boreExpected.Value, err = before.Value.Sub(units.CubicMillimeters(math.Pi * p["bore"] * p["bore"] / 4 * p["thickness"]))
	require.NoError(t, err)
	return []*decad.Body{cut}
}

func assertCutBore(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	volume, err := bodies[0].Volume()
	require.NoError(t, err)
	// Subtracting the analytic bore cylinder has only floating-point roundoff;
	// Agree includes both the original and the cut body's measurement bounds.
	decadtest.Agree(t, "bore removes its full-height cylinder", volume, boreExpected,
		decadtest.WithinRel(units.Scalar(1e-8)))
}

// chordedProfile first builds the same proven Gear Profile sketch, then replaces
// its two fitted splines with chords joining their solved fit points. A spline
// makes every cut in that sketch inexact, so decad cannot record the trimmed
// root interval. Chords retain the sampled involute and let the real arrangement
// split the root circle. This proves a polygonal approximation, not smooth flanks.
func chordedProfile(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile, *sketch.Profile) {
	s := decadtest.NewSketch(t)
	stepGearProfile(t, s, p)
	for _, entity := range s.Entities() {
		spline, ok := entity.(*sketch.FitSpline)
		if !ok {
			continue
		}
		require.True(t, s.RemoveEntity(spline))
		for i := 1; i < len(spline.Fit); i++ {
			s.CreateLine(spline.Fit[i-1], spline.Fit[i])
		}
	}
	sketchtest.Solve(t, s)
	var tooth, disc *sketch.Profile
	for _, profile := range s.Profiles() {
		sketchtest.IsValidProfile(t, profile)
		sketchtest.HasExactCuts(t, profile)
		if len(profile.Entities) == 1 {
			if _, ok := profile.Entities[0].(*sketch.Circle); ok {
				disc = profile
				continue
			}
		}
		tooth = profile
	}
	require.NotNil(t, tooth)
	require.NotNil(t, disc)
	return s, tooth, disc
}

func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, tooth, _ := chordedProfile(t, p)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, tooth, units.Millimeters(p["thickness"]))}
}

func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, _, disc := chordedProfile(t, p)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, disc, units.Millimeters(p["thickness"]))}
}

func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	// A circular cylinder's closed-form volume has only roundoff error.
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(math.Pi*d.Root*d.Root*p["thickness"]),
		decadtest.WithinRel(units.Scalar(1e-8)))
	decadtest.MeasuresBounds(t, bodies[0], r3.NewVec(-d.Root, -d.Root, 0),
		r3.NewVec(d.Root, d.Root, p["thickness"]), decadtest.Within(units.Millimeters(1e-8)))
}

// clippedFlank computes the root intersection of the sampled flank's first
// crossing chord independently of the sketch's profile detection.
func clippedFlank(p map[string]float64) ([]involute.Pt, involute.Dimensions) {
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	left, _ := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], int(p["samples"]), 0)
	if !d.Embedded() {
		a := left[0]
		return append([]involute.Pt{{X: a.X * d.Root / d.Base, Y: a.Y * d.Root / d.Base}}, left...), d
	}
	for i := 1; i < len(left); i++ {
		b := left[i]
		if b.X*b.X+b.Y*b.Y < d.Root*d.Root {
			continue
		}
		a := left[i-1]
		dx, dy := b.X-a.X, b.Y-a.Y
		aa, bb, cc := dx*dx+dy*dy, 2*(a.X*dx+a.Y*dy), a.X*a.X+a.Y*a.Y-d.Root*d.Root
		u := (-bb + math.Sqrt(bb*bb-4*aa*cc)) / (2 * aa)
		return append([]involute.Pt{{X: a.X + u*dx, Y: a.Y + u*dy}}, left[i:]...), d
	}
	panic("tip does not reach root circle")
}

func toothArea(p map[string]float64) float64 {
	left, d := clippedFlank(p)
	area := 0.0
	for i := 1; i < len(left); i++ {
		a, b := left[i-1], left[i]
		area -= a.X*b.Y - a.Y*b.X
	}
	first, last := left[0], left[len(left)-1]
	return area + d.Tip*d.Tip*math.Atan2(last.Y, last.X) - d.Root*d.Root*math.Atan2(first.Y, first.X)
}

func assertExtrudeTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	// Green's theorem on the chord chain and the two exact circular arcs is
	// analytic for the substituted profile; only floating-point roundoff remains.
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(toothArea(p)*p["thickness"]),
		decadtest.WithinRel(units.Scalar(1e-8)))
}

// The pattern is serial because its assertion reads this build's count.
var patternCopiesVerified int

func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, tooth, _ := chordedProfile(t, p)
	seed := decadtest.NewPrism(t, doc, s, tooth, units.Millimeters(p["thickness"]))
	seedVolume, err := seed.Volume()
	require.NoError(t, err)
	seedCenter, err := seed.Centroid()
	require.NoError(t, err)
	patternCopiesVerified = 1
	// Coexisting copies give undecided_pair in the engine's pair partition
	// proof. Verify each placement in its own document instead. This proves
	// all copy positions and volumes, not simultaneous inter-copy separation.
	for i := 1; i < int(p["teeth"]); i++ {
		transform, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(2*math.Pi*float64(i)/p["teeth"]))
		require.NoError(t, err)
		separate := decad.New()
		unplaced := decadtest.NewPrism(t, separate, s, tooth, units.Millimeters(p["thickness"]))
		copy, err := unplaced.Placed(t.Context(), transform)
		require.NoError(t, err)
		proofkit3d.RequireSolid(t, separate, []*decad.Body{copy})
		volume, err := copy.Volume()
		require.NoError(t, err)
		decadtest.Agree(t, "isolated pattern copy volume", volume, seedVolume, decadtest.WithinRel(units.Scalar(1e-8)))
		center, err := copy.Centroid()
		require.NoError(t, err)
		// Rotation preserves the original centroid's ball bound; add only
		// floating-point transformation slack to that bound.
		slack, err := seedCenter.Bound.Add(units.Millimeters(1e-8))
		require.NoError(t, err)
		decadtest.MeasuresVec(t, "isolated pattern copy centroid", center, transform.Apply(seedCenter.Value), decadtest.Within(slack))
		patternCopiesVerified++
	}
	return []*decad.Body{seed}
}

func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1, "the returned document owns the seed only")
	require.Equal(t, int(p["teeth"]), patternCopiesVerified, "seed and all isolated copies verified")
	seed, err := bodies[0].Volume()
	require.NoError(t, err)
	for _, body := range bodies {
		volume, err := body.Volume()
		require.NoError(t, err)
		// Rigid rotation preserves volume exactly apart from floating-point error.
		decadtest.Agree(t, "pattern copy volume", volume, seed, decadtest.WithinRel(units.Scalar(1e-8)))
	}
}

func stepJoinTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// The real union attempt refuses the coincident tooth/disc root surfaces:
	// held facets cannot prove interpenetration. Extruding the complete outline
	// substitutes the joined geometry; it does not prove Fusion Combine-Join,
	// its tool-body collection, or its consumption of the original seed.
	s := gearOutline(t, p)
	profile := decadtest.SolveRegion(t, s)
	return []*decad.Body{decadtest.NewPrism(t, doc, s, profile, units.Millimeters(p["thickness"]))}
}

func gearOutline(t *testing.T, p map[string]float64, minimumRootChord ...float64) *sketch.Sketch {
	_, tooth, _ := chordedProfile(t, p)
	// Start immediately after the root-circle interval in the detected loop.
	start := 0
	for i, edge := range tooth.Outer {
		if _, ok := edge.Entity.(*sketch.Circle); ok {
			start = (i + 1) % len(tooth.Outer)
		}
	}
	var path []sketch.BoundaryEdge
	for j := 0; j < len(tooth.Outer); j++ {
		edge := tooth.Outer[(start+j)%len(tooth.Outer)]
		if _, ok := edge.Entity.(*sketch.Circle); !ok {
			path = append(path, edge)
		}
	}
	if len(minimumRootChord) > 0 {
		minimum := minimumRootChord[0]
		coalesce := func(edges []sketch.BoundaryEdge) []sketch.BoundaryEdge {
			start := edges[0].Polyline[0]
			last := 0
			for i, edge := range edges {
				if _, ok := edge.Entity.(*sketch.Line); !ok {
					break
				}
				last = i
				end := edge.Polyline[len(edge.Polyline)-1]
				if math.Hypot(end[0]-start[0], end[1]-start[1]) >= minimum {
					break
				}
			}
			combined := edges[0]
			end := edges[last].Polyline[len(edges[last].Polyline)-1]
			combined.Polyline = [][2]float64{start, end}
			return append([]sketch.BoundaryEdge{combined}, edges[last+1:]...)
		}
		reverse := func(edges []sketch.BoundaryEdge) []sketch.BoundaryEdge {
			result := make([]sketch.BoundaryEdge, len(edges))
			for i, edge := range edges {
				edge.Polyline = [][2]float64{edge.Polyline[len(edge.Polyline)-1], edge.Polyline[0]}
				result[len(edges)-1-i] = edge
			}
			return result
		}
		path = reverse(coalesce(reverse(coalesce(path))))
	}
	s := decadtest.NewSketch(t)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	var first, previous *sketch.Point
	for i := 0; i < int(p["teeth"]); i++ {
		angle := 2 * math.Pi * float64(i) / p["teeth"]
		makePoint := func(xy [2]float64) *sketch.Point {
			x, y := involute.Rotate(xy[0], xy[1], angle)
			point := s.CreatePoint(x, y)
			s.Fix(point)
			return point
		}
		begin := makePoint(path[0].Polyline[0])
		if first == nil {
			first = begin
		} else {
			s.CreateArc(center, previous, begin)
		}
		current := begin
		for _, edge := range path {
			end := makePoint(edge.Polyline[len(edge.Polyline)-1])
			if _, ok := edge.Entity.(*sketch.Arc); ok {
				s.CreateArc(center, current, end)
			} else {
				s.CreateLine(current, end)
			}
			current = end
		}
		previous = current
	}
	s.CreateArc(center, previous, first)
	return s
}

func assertJoinTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	require.Len(t, bodies, 1)
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	area := math.Pi*d.Root*d.Root + p["teeth"]*toothArea(p)
	// Tooth sectors touch the disc at their root arcs and have disjoint interiors.
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(area*p["thickness"]),
		decadtest.WithinRel(units.Scalar(1e-8)))
}
