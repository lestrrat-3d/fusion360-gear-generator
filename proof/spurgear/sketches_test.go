package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/stretchr/testify/require"
)

var sketchCases = []proofkit.Case{
	{Name: "standard", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15}},
	{Name: "negative_rotation", Params: map[string]float64{"module": 1, "teeth": 17, "pressure": 20, "samples": 15, "angle": -0.4, "x": 8, "y": -3}},
	{Name: "positive_rotation", Params: map[string]float64{"module": 2, "teeth": 24, "pressure": 20, "samples": 5, "angle": 0.4}},
	{Name: "quarter_turn", Params: map[string]float64{"module": 0.5, "teeth": 17, "pressure": 20, "samples": 5, "angle": math.Pi / 2}},
	{Name: "half_turn", Params: map[string]float64{"module": 1, "teeth": 31, "pressure": 20, "samples": 15, "angle": math.Pi}},
	{Name: "embedded_count", Params: map[string]float64{"module": 1, "teeth": 60, "pressure": 20, "samples": 15}},
	{Name: "embedded_pressure", Params: map[string]float64{"module": 1, "teeth": 30, "pressure": 30, "samples": 5}},
}

var boreCases = []proofkit.Case{
	{Name: "bore", Params: map[string]float64{"bore": 2}},
	{Name: "translated_bore", Params: map[string]float64{"bore": 4, "x": 8, "y": -3}},
}

func stepBoreProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project Tools anchor into Bore Profile")
	projected := s.CreateReferencePoint(p["x"], p["y"], "Tools anchor")
	localOrigin := s.CreatePoint(0, 0)
	circle := s.CreateCircle(projected, p["bore"]/2)
	s.AddConstraint(sketch.NewDiameter(circle, p["bore"]))
	s.AddConstraint(sketch.NewCoincident(localOrigin, projected))
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.IsCurrentProfile(t, profile)
	// The analytic circular area has only floating-point roundoff.
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*p["bore"]*p["bore"]/4,
		sketchtest.WithinRel(1e-8))
	sketchtest.MeasuresPoint(t, localOrigin, p["x"], p["y"], sketchtest.Within(1e-8))
}

func stepTools(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project user anchor")
	anchor := s.CreateReferencePoint(p["x"], p["y"], "user anchor")
	sketchtest.MeasuresPoint(t, anchor, p["x"], p["y"], sketchtest.Within(1e-9))
}

// The flank-to-root lines: root endpoint pinned by signed
// NewHorizontalDistance(origin, rootEnd, dx) and
// NewVerticalDistance(origin, rootEnd, dy).
// Text labels are omitted: the sketch engine has no Fusion along-path text.
// The strict base/root equality branch creates zero-length stubs in Fusion and
// is outside this nondegenerate sweep; no tolerance changes that branch.
func stepGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "four circles and shared local origin")
	d := involute.Derive(p["module"], p["teeth"], p["pressure"]*math.Pi/180)
	ox, oy, angle := p["x"], p["y"], p["angle"]
	origin := s.CreatePoint(ox, oy)
	var circles []*sketch.Circle
	for i, radius := range []float64{d.Root, d.Tip, d.Base, d.Pitch} {
		c := s.CreateCircle(origin, radius)
		c.SetConstruction(i != 0)
		s.AddConstraint(sketch.NewDiameter(c, 2*radius))
		circles = append(circles, c)
	}
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], int(p["samples"]), angle)
	lp, rp := make([]*sketch.Point, len(left)), make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(ox+left[i].X, oy+left[i].Y)
		rp[i] = s.CreatePoint(ox+right[i].X, oy+right[i].Y)
	}
	_, err := s.CreateFitSpline(lp...)
	require.NoError(t, err)
	_, err = s.CreateFitSpline(rp...)
	require.NoError(t, err)
	proofkit.Step(t, "tip arc and directed spine")
	arcCenter := s.CreatePoint(ox, oy)
	s.CreateArc(arcCenter, rp[len(rp)-1], lp[len(lp)-1])
	arcCenterAnchor := sketch.NewCoincident(arcCenter, origin)
	s.AddConstraint(arcCenterAnchor)
	ca, sa := math.Cos(angle), math.Sin(angle)
	top := s.CreatePoint(ox+d.Tip*ca, oy+d.Tip*sa)
	s.AddConstraint(sketch.NewPointOnCircle(top, circles[1]))
	spine := s.CreateLine(origin, top)
	spine.SetConstruction(true)
	referenceEnd := s.CreatePoint(ox+d.Tip, oy)
	s.AddConstraint(sketch.NewHorizontalDistance(origin, referenceEnd, d.Tip),
		sketch.NewVerticalDistance(origin, referenceEnd, 0))
	reference := s.CreateLine(origin, referenceEnd)
	reference.SetConstruction(true)
	// NewAngle uses the sketch's default angle unit (metric degrees), while
	// the involute math and Fusion angle parameter use radians.
	rotation := sketch.NewAngle(reference, spine, angle*180/math.Pi)
	s.AddConstraint(rotation)
	proofkit.Step(t, "ordered rib constraints")
	previous := origin
	previousT := 0.0
	for i := range left {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetConstruction(true)
		if math.Abs(ca) >= math.Abs(sa) {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}
		along := left[i].X*ca + left[i].Y*sa
		mid := s.CreatePoint(ox+along*ca, oy+along*sa)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if math.Abs(ca) >= math.Abs(sa) {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, (along-previousT)*ca))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, (along-previousT)*sa))
		}
		previous, previousT = mid, along
	}
	if !d.Embedded() {
		proofkit.Step(t, "radial stubs")
		for i, flank := range []*sketch.Point{lp[0], rp[0]} {
			sample := left[0]
			if i == 1 {
				sample = right[0]
			}
			rx, ry := sample.X*d.Root/d.Base, sample.Y*d.Root/d.Base
			re := s.CreatePoint(ox+rx, oy+ry)
			s.CreateLine(re, flank)
			s.AddConstraint(sketch.NewHorizontalDistance(origin, re, rx),
				sketch.NewVerticalDistance(origin, re, ry))
		}
	}
	proofkit.Step(t, "project and anchor completed network")
	anchor := s.CreateReferencePoint(ox, oy, "Tools anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	sketchtest.Solve(t, s)
	sketchtest.Satisfies(t, rotation, sketchtest.Within(1e-8))
	// Formula error is floating-point evaluation of the exact radius and pose.
	sketchtest.MeasuresPoint(t, arcCenter, ox, oy, sketchtest.Within(1e-8))
	sketchtest.MeasuresPoint(t, top, ox+d.Tip*ca, oy+d.Tip*sa, sketchtest.Within(1e-8))
	profiles := s.Profiles()
	require.Len(t, profiles, 2, "tooth and root disc")
	var tooth, disc *sketch.Profile
	for _, profile := range profiles {
		sketchtest.IsValidProfile(t, profile)
		sketchtest.IsCurrentProfile(t, profile)
		if len(profile.Entities) == 1 && profile.Entities[0] == circles[0] {
			disc = profile
		} else {
			tooth = profile
		}
	}
	require.NotNil(t, tooth, "tooth profile")
	require.NotNil(t, disc, "root disc profile")
	// The engine emits a circle's parameter seam as an extra edge in a tooth
	// crossing +X, while it coalesces the disc's complete circle into one edge.
	// Fusion instead exposes the root circle as two arcs cut at the tooth's
	// contacts. Check those contacts on the actual detected tooth boundary,
	// and count the connected root-circle interval once, independent of its seam.
	// This proves the Fusion curve-count mapping, not identical raw edge arrays.
	contacts := map[float64]struct{}{}
	var rootEdges []sketch.BoundaryEdge
	lines, flanks, caps := 0, 0, 0
	for _, edge := range tooth.Outer {
		switch edge.Entity.(type) {
		case *sketch.Circle:
			require.Same(t, circles[0], edge.Entity, "only root circle bounds the tooth")
			require.True(t, edge.Partial, "tooth uses an interval of the root circle")
			rootEdges = append(rootEdges, edge)
			if edge.TStart != 0 {
				contacts[edge.TStart] = struct{}{}
			}
			if edge.TEnd != 1 {
				contacts[edge.TEnd] = struct{}{}
			}
		case *sketch.Line:
			lines++
		case *sketch.FitSpline:
			flanks++
		case *sketch.Arc:
			caps++
		default:
			t.Fatalf("unexpected tooth boundary source %T", edge.Entity)
		}
	}
	require.Len(t, contacts, 2, "two root contacts divide the root disc into two Fusion arcs")
	require.Contains(t, []int{1, 2}, len(rootEdges), "one connected root arc, possibly split at the parameter seam")
	if len(rootEdges) == 2 {
		starts, ends := 0, 0
		for _, edge := range rootEdges {
			if edge.TStart == 0 {
				starts++
			}
			if edge.TEnd == 1 {
				ends++
			}
		}
		require.Equal(t, 1, starts, "one root fragment starts at the seam")
		require.Equal(t, 1, ends, "the other root fragment ends at the seam")
	}
	require.Equal(t, 2, flanks, "two fitted-spline flank curves")
	require.Equal(t, 1, caps, "one tip arc plus one connected root arc")
	wantLines := 2
	if d.Embedded() {
		wantLines = 0
	}
	require.Equal(t, wantLines, lines, "root stub curve count")
	// The root disc's analytic area is exact apart from floating-point roundoff.
	sketchtest.MeasuresProfileArea(t, disc, math.Pi*d.Root*d.Root, sketchtest.WithinRel(1e-8))
	if angle == 0 && p["teeth"] == 17 && p["module"] == 1 {
		// The specified negative control leaves the arc's copied center free.
		// Require exactly its two missing constraints, then restore the real
		// construction before the harness performs its positive gate.
		require.True(t, s.RemoveConstraint(arcCenterAnchor))
		sketchtest.Solve(t, s)
		negative := sketchtest.Verify(t, s)
		sketchtest.HasStatus(t, negative, sketch.Underconstrained)
		sketchtest.HasDOF(t, negative, 2)
		sketchtest.HasOnlyReasons(t, negative, sketch.ErrNotFullyConstrained)
		sketchtest.FindReasons(t, negative, sketch.ErrNotFullyConstrained)
		s.AddConstraint(arcCenterAnchor)
		sketchtest.Solve(t, s)
	}
}
