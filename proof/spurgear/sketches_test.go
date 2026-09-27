package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var profileCases = []proofkit.Case{
	{Name: "default", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "steps": 15}},
	{Name: "small_coarse", Params: map[string]float64{"module": 2, "teeth": 8, "pressure": 20, "steps": 4}},
	{Name: "small_fine", Params: map[string]float64{"module": 0.5, "teeth": 13, "pressure": 14.5, "steps": 5}},
	{Name: "positive_twist", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "steps": 5, "angle": 25}},
	{Name: "negative_twist", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "steps": 5, "angle": -25}},
	{Name: "quarter_turn", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "steps": 5, "angle": 90}},
	{Name: "half_turn", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "steps": 5, "angle": 180}},
	{Name: "embedded_by_count", Params: map[string]float64{"module": 1, "teeth": 43, "pressure": 20, "steps": 5}},
	{Name: "embedded_by_pressure", Params: map[string]float64{"module": 1, "teeth": 28, "pressure": 25, "steps": 5}},
}

// stepGearProfile reproduces the shared-point tooth, its construction ribs, and
// the two profile boundaries. Fusion's along-path labels are display geometry;
// they have no corresponding sketch-engine constraint primitive.
func stepGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	t.Helper()
	module, toothCount := p["module"], p["teeth"]
	pressure := p["pressure"] * math.Pi / 180
	angle := p["angle"] * math.Pi / 180
	steps := int(p["steps"])
	d := involute.Derive(module, toothCount, pressure)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothCount, steps, angle)
	require.Len(t, left, steps)
	require.Len(t, right, steps)

	proofkit.Step(t, "draw four concentric circles about the movable local origin")
	origin := s.CreatePoint(0, 0)
	root := s.CreateCircle(origin, d.Root)
	tip := s.CreateCircle(origin, d.Tip)
	base := s.CreateCircle(origin, d.Base)
	pitch := s.CreateCircle(origin, d.Pitch)
	tip.SetConstruction(true)
	base.SetConstruction(true)
	pitch.SetConstruction(true)
	for _, circle := range []struct {
		curve  *sketch.Circle
		radius float64
	}{{root, d.Root}, {tip, d.Tip}, {base, d.Base}, {pitch, d.Pitch}} {
		s.AddConstraint(sketch.NewDiameter(circle.curve, 2*circle.radius))
	}

	proofkit.Step(t, "draw endpoint-inclusive involute flanks and the shared tooth-top arc")
	leftPoints := make([]*sketch.Point, steps)
	rightPoints := make([]*sketch.Point, steps)
	for i := range steps {
		leftPoints[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPoints[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	_, err := s.CreateFitSpline(leftPoints...)
	require.NoError(t, err)
	_, err = s.CreateFitSpline(rightPoints...)
	require.NoError(t, err)
	toothTop := s.CreatePoint(d.Tip*math.Cos(angle), d.Tip*math.Sin(angle))
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCenter := s.CreatePoint(0, 0)
	s.CreateArc(arcCenter, rightPoints[steps-1], leftPoints[steps-1])
	s.AddConstraint(sketch.NewCoincident(arcCenter, origin))

	proofkit.Step(t, "pin the spine angle and each rib without an extra last-rib perpendicular")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewHorizontalDistance(origin, refEnd, d.Tip))
	s.AddConstraint(sketch.NewVerticalDistance(origin, refEnd, 0))
	angular := sketch.NewAngle(reference, spine, 0)
	angular.SetValue(units.Radians(angle))
	s.AddConstraint(angular)
	previous := origin
	acrossVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	for i := range steps {
		rib := s.CreateLine(leftPoints[i], rightPoints[i])
		rib.SetConstruction(true)
		if acrossVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPoints[i], rightPoints[i],
				right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPoints[i], rightPoints[i],
				right[i].X-left[i].X))
		}
		projection := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midpoint := s.CreatePoint(projection*math.Cos(angle), projection*math.Sin(angle))
		s.AddConstraint(sketch.NewPointOnLine(midpoint, spine))
		s.AddConstraint(sketch.NewMidpoint(midpoint, rib))
		if i != steps-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, midpoint,
				midpoint.X()-previous.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, midpoint,
				midpoint.Y()-previous.Y()))
		}
		previous = midpoint
	}

	proofkit.Step(t, "close at the root with two directed dimensions per nonembedded stub")
	// flank-to-root lines: root endpoint pinned by signed horizontal and vertical
	// distances. In the constraint map, the recipe is
	// NewHorizontalDistance(origin, rootEnd, dx) and
	// NewVerticalDistance(origin, rootEnd, dy): rootEnd is the shared stub end,
	// and dx/dy retain its signed side of the local origin. The code below uses
	// re/rx/ry for those same entities and signed values.
	if !d.Embedded() {
		for _, flank := range []struct {
			seed  involute.Pt
			point *sketch.Point
		}{{left[0], leftPoints[0]}, {right[0], rightPoints[0]}} {
			ratio := d.Root / math.Hypot(flank.seed.X, flank.seed.Y)
			rx, ry := flank.seed.X*ratio, flank.seed.Y*ratio
			re := s.CreatePoint(rx, ry)
			s.CreateLine(re, flank.point)
			s.AddConstraint(sketch.NewHorizontalDistance(origin, re, rx))
			s.AddConstraint(sketch.NewVerticalDistance(origin, re, ry))
		}
	}

	proofkit.Step(t, "project and anchor after drawing the entire tooth")
	projected := s.CreateReferencePoint(0, 0, "Tools anchor projection")
	s.Fix(projected)
	anchorCoincident := sketch.NewCoincident(origin, projected)
	s.AddConstraint(anchorCoincident)

	// The harness performs the final soundness check. This preliminary solve
	// lets the step assert profile topology on the geometry just built.
	sketchtest.Solve(t, s)
	profiles := s.Profiles()
	require.Len(t, profiles, 2, "the root disc and tooth section must both close")
	var disc, tooth *sketch.Profile
	for _, profile := range profiles {
		if len(profile.Entities) == 1 {
			disc = profile
		} else {
			tooth = profile
		}
		sketchtest.IsValidProfile(t, profile)
	}
	require.NotNil(t, disc)
	require.NotNil(t, tooth)
	_, discIsCircle := disc.Entities[0].(*sketch.Circle)
	require.True(t, discIsCircle, "the root disc must use the root circle")
	wantLines := 2
	if d.Embedded() {
		wantLines = 0
	}
	counts := map[string]int{"spline": 0, "tip arc": 0, "root arc": 0, "stub": 0}
	for _, entity := range tooth.Entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			counts["spline"]++
		case *sketch.Arc:
			counts["tip arc"]++
		case *sketch.Circle:
			counts["root arc"]++
		case *sketch.Line:
			counts["stub"]++
		default:
			t.Fatalf("unexpected tooth boundary source %T", entity)
		}
	}
	require.Equal(t, map[string]int{"spline": 2, "tip arc": 1, "root arc": 1, "stub": wantLines}, counts)
	// Profile detection in sketch leaves the disc's source as one circle. Fusion
	// reports two root arcs after its own curve splitting; the enclosed disc is
	// the same, and the proof asserts its area rather than inventing split arcs.
	sketchtest.Measures(t, "root disc area", disc.Area, math.Pi*d.Root*d.Root,
		sketchtest.WithinRel(1e-8))
	sketchtest.Satisfies(t, anchorCoincident, sketchtest.Within(1e-8))
	// The fixed formula has floating-point error near one part in 1e12.
	sketchtest.Measures(t, "root radius", root.R(), d.Root, sketchtest.WithinRel(1e-10))
}
