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

// All runs are serial: the source spec does not request parallel cases.
var sketchCases = []proofkit.Case{{Name: "default", Params: defaultParams()}}

func defaultParams() map[string]float64 {
	return map[string]float64{"module": 1, "teeth": 17, "pressure": 20 * math.Pi / 180,
		"angle": 0, "samples": 15, "x": 0, "y": 0, "bore": 2, "height": 10, "chamfer": 0.1}
}

func dimensions(p map[string]float64) involute.Dimensions {
	return involute.Derive(p["module"], p["teeth"], p["pressure"])
}

// Tools owns only a projected point in Fusion. The harness requires authored
// geometry, so the coincident local point is a substitute observer of that
// reference; it is not an extra point required in the Fusion Tools sketch.
func stepTools(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project anchor")
	anchor := s.CreateReferencePoint(p["x"], p["y"], "user-anchor")
	observer := s.CreatePoint(p["x"], p["y"])
	c := sketch.NewCoincident(observer, anchor)
	s.AddConstraint(c)
	sketchtest.Solve(t, s)
	// Exact coordinate transfer; 1e-8 mm allows solver roundoff only.
	sketchtest.Satisfies(t, c, sketchtest.Within(1e-8))
	sketchtest.MeasuresPoint(t, observer, p["x"], p["y"], sketchtest.Within(1e-8))
}

func stepGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	drawGearProfile(t, s, p)
	sketchtest.Solve(t, s)
	profiles := s.Profiles()
	require.Len(t, profiles, 2, "the real root circle and tooth must produce exactly two regions")
	var tooth, disc *sketch.Profile
	for _, profile := range profiles {
		sketchtest.IsValidProfile(t, profile)
		sketchtest.IsCurrentProfile(t, profile)
		flanks, arcs, lines := 0, 0, 0
		circleFragments := make(map[*sketch.Circle][]sketch.BoundaryEdge)
		for _, edge := range profile.Outer {
			switch entity := edge.Entity.(type) {
			case *sketch.FitSpline:
				flanks++
			case *sketch.Arc:
				arcs++
			case *sketch.Circle:
				circleFragments[entity] = append(circleFragments[entity], edge)
			case *sketch.Line:
				lines++
			default:
				t.Fatalf("unexpected profile entity %T", edge.Entity)
			}
		}
		require.Len(t, circleFragments, 1, "only the solid root circle bounds these regions")
		if flanks == 0 {
			require.Zero(t, arcs)
			require.Zero(t, lines)
			var coverage float64
			for _, fragments := range circleFragments {
				for _, fragment := range fragments {
					coverage += fragment.TEnd - fragment.TStart
				}
			}
			// The engine coalesces a full circle across tooth contacts. Its
			// fragments do not prove Fusion's required two profile-arc count.
			// Check the actual full-circle coverage; the tooth below checks
			// the two geometric contact positions that split Fusion's circle.
			sketchtest.Measures(t, "root circle parameter coverage", coverage, 1, sketchtest.Within(1e-9))
			disc = profile
			continue
		}
		contacts := make(map[float64][2]float64)
		for _, fragments := range circleFragments {
			// A circle's parameter seam divides one geometric arc into two
			// BoundaryEdges. Merge that artificial split only; retain the cuts
			// made by the two tooth contacts, including both arcs of the disc.
			atStart, atEnd := false, false
			for _, fragment := range fragments {
				atStart = atStart || fragment.TStart == 0
				atEnd = atEnd || fragment.TEnd == 1
				start, end := fragment.TStart, fragment.TEnd
				if fragment.Reversed {
					start, end = end, start
				}
				if start != 0 && start != 1 {
					contacts[start] = fragment.Polyline[0]
				}
				if end != 0 && end != 1 {
					contacts[end] = fragment.Polyline[len(fragment.Polyline)-1]
				}
			}
			count := len(fragments)
			if count > 1 && atStart && atEnd {
				count--
			}
			arcs += count
		}
		require.Len(t, contacts, 2, "the tooth meets the root circle at two positions")
		d := dimensions(p)
		left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], int(p["samples"]), p["angle"])
		// This initial case has radial stubs; embedded spline intersections
		// need their own contact oracle before that deferred case is enabled.
		require.False(t, d.Embedded(), "contact oracle currently covers the default radial-stub case")
		for _, flank := range []involute.Pt{left[0], right[0]} {
			wantX, wantY := p["x"]+flank.X*d.Root/d.Base, p["y"]+flank.Y*d.Root/d.Base
			nearest, distance := [2]float64{}, math.Inf(1)
			for _, point := range contacts {
				if delta := math.Hypot(point[0]-wantX, point[1]-wantY); delta < distance {
					nearest, distance = point, delta
				}
			}
			// Root endpoints are analytically placed; slack covers solver and
			// arrangement roundoff rather than a changed contact position.
			sketchtest.Measures(t, "root contact x", nearest[0], wantX, sketchtest.Within(1e-7))
			sketchtest.Measures(t, "root contact y", nearest[1], wantY, sketchtest.Within(1e-7))
		}
		require.Equal(t, 2, flanks)
		require.Equal(t, 2, arcs)
		wantLines := 2
		if dimensions(p).Embedded() {
			wantLines = 0
		}
		require.Equal(t, wantLines, lines)
		tooth = profile
	}
	require.NotNil(t, tooth)
	require.NotNil(t, disc)
	// The engine reports a sampled area: 0.1% covers circular chord integration.
	r := dimensions(p).Root
	sketchtest.MeasuresProfileArea(t, disc, math.Pi*r*r, sketchtest.WithinRel(0.001))
}

func drawGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	t.Helper()
	d := dimensions(p)
	a := p["angle"]
	ox, oy := p["x"], p["y"]
	proofkit.Step(t, "circles and local origin")
	origin := s.CreatePoint(ox, oy)
	for i, radius := range []float64{d.Root, d.Tip, d.Base, d.Pitch} {
		circle := s.CreateCircle(origin, radius)
		circle.SetConstruction(i != 0)
		s.AddConstraint(sketch.NewDiameter(circle, 2*radius))
	}
	// Along-path text has no geometric counterpart. Fusion logs the labelled
	// sketch's constraint status; all geometric constraints remain gated here.
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], int(p["samples"]), a)
	lp, rp := make([]*sketch.Point, len(left)), make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(ox+left[i].X, oy+left[i].Y)
		rp[i] = s.CreatePoint(ox+right[i].X, oy+right[i].Y)
	}
	_, err := s.CreateFitSpline(lp...)
	require.NoError(t, err)
	_, err = s.CreateFitSpline(rp...)
	require.NoError(t, err)
	proofkit.Step(t, "tooth top, spine, and signed angle")
	arcCenter := s.CreatePoint(ox, oy)
	s.CreateArc(arcCenter, rp[len(rp)-1], lp[len(lp)-1])
	s.AddConstraint(sketch.NewCoincident(arcCenter, origin))
	top := s.CreatePoint(ox+d.Tip*math.Cos(a), oy+d.Tip*math.Sin(a))
	// Reuse the actual construction tip circle.
	tip := s.Entities()[1].(*sketch.Circle)
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))
	spine := s.CreateLine(origin, top)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(ox+d.Tip, oy)
	s.AddConstraint(sketch.NewHorizontalDistance(origin, refEnd, d.Tip), sketch.NewVerticalDistance(origin, refEnd, 0))
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	angular := sketch.NewAngle(reference, spine, a*180/math.Pi)
	s.AddConstraint(angular)
	proofkit.Step(t, "all ribs, endpoint ribs included")
	previous := origin
	for i := range lp {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetConstruction(true)
		acrossVertical := math.Abs(math.Cos(a)) >= math.Abs(math.Sin(a))
		if acrossVertical {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}
		along := left[i].X*math.Cos(a) + left[i].Y*math.Sin(a)
		mid := s.CreatePoint(ox+along*math.Cos(a), oy+along*math.Sin(a))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(lp)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, mid.X()-previous.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, mid.Y()-previous.Y()))
		}
		previous = mid
	}
	proofkit.Step(t, "flank-to-root lines and anchor chain")
	// flank-to-root lines: root endpoint pinned by signed axis dimensions.
	// Mapping: NewHorizontalDistance(origin, rootEnd, dx) and
	// NewVerticalDistance(origin, rootEnd, dy); Fusion stores abs(delta) with
	// the direction captured by the seed. No extra root-end constraints exist.
	if !d.Embedded() {
		for _, flank := range []*sketch.Point{lp[0], rp[0]} {
			rx, ry := (flank.X()-ox)*d.Root/d.Base, (flank.Y()-oy)*d.Root/d.Base
			re := s.CreatePoint(ox+rx, oy+ry)
			s.CreateLine(re, flank)
			s.AddConstraint(sketch.NewHorizontalDistance(origin, re, rx))
			s.AddConstraint(sketch.NewVerticalDistance(origin, re, ry))
		}
	}
	anchor := s.CreateReferencePoint(ox, oy, "Tools/projected-anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	// Geometry was pre-rotated and the signed angular constraint already carries
	// the final value. Fusion sets that value last after anchoring.
	sketchtest.Solve(t, s)
	// Analytic point arithmetic has only roundoff; solver slack is 1e-7 mm.
	sketchtest.MeasuresPoint(t, top, ox+d.Tip*math.Cos(a), oy+d.Tip*math.Sin(a), sketchtest.Within(1e-7))
	sketchtest.Satisfies(t, angular, sketchtest.Within(1e-7))
	// Strict base<root leaves exact equality with zero-length stubs. No positive
	// case claims that degenerate transition can satisfy the soundness gate.
}

func stepBoreProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "bore projection and otherwise unused local origin")
	anchor := s.CreateReferencePoint(p["x"], p["y"], "Tools/projected-anchor")
	origin := s.CreatePoint(p["x"], p["y"])
	coincident := sketch.NewCoincident(origin, anchor)
	s.AddConstraint(coincident)
	circle := s.CreateCircle(anchor, p["bore"]/2)
	s.AddConstraint(sketch.NewDiameter(circle, p["bore"]))
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.IsCurrentProfile(t, profile)
	sketchtest.HasExactCuts(t, profile)
	// Circle area integration is sampled; 0.1% covers that approximation.
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*p["bore"]*p["bore"]/4, sketchtest.WithinRel(0.001))
	sketchtest.Satisfies(t, coincident, sketchtest.Within(1e-8))
}
