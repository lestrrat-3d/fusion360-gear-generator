package spurgear_test

// Shared geometry for the spur proof.
//
// The sketch steps model the Gear Profile as it is really drawn: fitted splines
// through the involute samples, a tooth-top arc sharing the local origin, the
// rib chain, and the flank-to-root stubs. The solid steps cannot use that
// boundary. decad refuses to record a profile whose trim it cannot certify, and
// the sketch engine withholds trim exactness from EVERY edge of a sketch that
// holds one free-form entity, so a spline anywhere in the Gear Profile makes the
// tooth region unrecordable; extruding a spline-walled loop is separately
// refused ("a free-form wall edge's chain has curvature signs that conflict").
//
// So the solid steps build the same tooth with each flank CHORDED — a polyline
// through exactly the same involute samples the spline interpolates. What the
// substitution costs is stated at each step that uses it. What it does not touch
// is the sampling, the tooth's angular placement, the root and tip radii, the
// extrude extents, the edge topology the later steps select on, or any volume
// difference between two bodies built the same way.

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
)

// gear is one case's derived geometry, in millimetres and radians.
type gear struct {
	module        float64
	toothNumber   float64
	pressureAngle float64
	angle         float64
	steps         int
	thickness     float64
	bore          float64
	dims          involute.Dimensions
}

func newGear(p map[string]float64) gear {
	g := gear{
		module:        p["module"],
		toothNumber:   p["toothNumber"],
		pressureAngle: p["pressureAngle"],
		angle:         p["angle"],
		steps:         int(p["involuteSteps"]),
		thickness:     p["thickness"],
		bore:          p["boreDiameter"],
	}
	g.dims = involute.Derive(g.module, g.toothNumber, g.pressureAngle)
	return g
}

// filletRadius is the spec's derived FilletRadius parameter:
// (ToothSpaceArcAtRoot / 2) * FilletClearance * 1, with ToothSpaceArcAtRoot the
// root-circle arc of the valley and FilletClearance 0.9.
func (g gear) filletRadius() float64 {
	spaceAngle := math.Pi/g.toothNumber - 2*(math.Tan(g.pressureAngle)-g.pressureAngle)
	return (g.dims.Root * spaceAngle / 2) * 0.9
}

// discArea is the area of the region inside the root circle.
func (g gear) discArea() float64 { return math.Pi * g.dims.Root * g.dims.Root }

// flankBoundary returns one flank's boundary polyline, walking outward from the
// root circle to the tip.
//
// Two shapes come out of it, and which one is the spec's step-9 branch. When the
// first involute sample sits OUTSIDE radius r the walk starts at that sample's
// radial foot on r — the flank-to-root stub, drawn as its first chord. When the
// sample sits inside (the embedded profile), the samples inside r are dropped
// and the walk starts where the flank crosses r, which is what Fusion's profile
// split leaves behind when no stub is drawn.
func flankBoundary(pts []involute.Pt, r float64) []involute.Pt {
	radius := func(p involute.Pt) float64 { return math.Hypot(p.X, p.Y) }
	if radius(pts[0]) >= r {
		s := r / radius(pts[0])
		return append([]involute.Pt{{X: pts[0].X * s, Y: pts[0].Y * s}}, pts...)
	}
	for i := 1; i < len(pts); i++ {
		if radius(pts[i]) < r {
			continue
		}
		a, b := pts[i-1], pts[i]
		lo, hi := 0.0, 1.0
		for range 60 {
			mid := (lo + hi) / 2
			q := involute.Pt{X: a.X + (b.X-a.X)*mid, Y: a.Y + (b.Y-a.Y)*mid}
			if radius(q) < r {
				lo = mid
			} else {
				hi = mid
			}
		}
		crossing := involute.Pt{X: a.X + (b.X-a.X)*hi, Y: a.Y + (b.Y-a.Y)*hi}
		rest := pts[i:]
		// A crossing that lands almost on the sample it precedes would leave a
		// chord far shorter than the ones around it, which is an artefact of
		// chording and not a feature of the flank. Drop that sample.
		if hi > 0.5 {
			rest = pts[i+1:]
		}
		return append([]involute.Pt{crossing}, rest...)
	}
	return nil
}

// toothOutline draws one chorded tooth as a single closed loop of whole
// entities, at angular position angle.
//
// footRadius is where the two flanks are closed onto: g.dims.Root draws the
// tooth exactly as the Gear Profile sketch does, and a smaller radius sinks the
// tooth into the disc. The sunk shape exists only for the combine step, whose
// boolean cannot classify the tangent contact two operands that merely touch
// along the root circle make.
//
// wrap picks which of the two arcs the flank feet cut the root circle into
// closes the loop: the short one under the tooth (the tooth region the tooth
// extrude consumes) or the long one the rest of the way round (the disc and one
// tooth as a single region, which is what the root fillet needs a body of).
func toothOutline(t testing.TB, g gear, angle, footRadius float64, wrap bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	left, right := involute.Flanks(g.dims.Base, g.dims.Tip, g.dims.Pitch, g.toothNumber, g.steps, angle)
	lb := flankBoundary(left, footRadius)
	rb := flankBoundary(right, footRadius)
	if lb == nil || rb == nil {
		t.Fatalf("flank never reaches radius %.4f", footRadius)
	}

	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	place := func(ps []involute.Pt) []*sketch.Point {
		out := make([]*sketch.Point, len(ps))
		for i, p := range ps {
			out[i] = s.CreatePoint(p.X, p.Y)
			s.Fix(out[i])
		}
		return out
	}
	lp, rp := place(lb), place(rb)
	for i := 0; i < len(lp)-1; i++ {
		s.CreateLine(lp[i], lp[i+1])
	}
	for i := 0; i < len(rp)-1; i++ {
		s.CreateLine(rp[i], rp[i+1])
	}
	s.CreateArc(origin, rp[len(rp)-1], lp[len(lp)-1])
	if wrap {
		s.CreateArc(origin, lp[0], rp[0])
	} else {
		s.CreateArc(origin, rp[0], lp[0])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve tooth outline: %v", err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("tooth outline closed %d regions, want 1", len(profiles))
	}
	return s, profiles[0]
}

// circleProfile draws one circle of radius r, the shape both the body extrude
// (the root circle) and the bore cut consume.
func circleProfile(t testing.TB, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	c := s.CreateCircle(centre, r)
	s.AddConstraint(sketch.NewDiameter(c, 2*r))
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve circle: %v", err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("circle closed %d regions, want 1", len(profiles))
	}
	return s, profiles[0]
}
