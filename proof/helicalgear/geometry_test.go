// Package helicalgear_test proves the two things helical adds to the spur
// build: the second, twisted "Twisted Gear Profile" sketch that becomes the top
// loft section, and the loft that replaces spur's tooth extrude.
//
// Everything else in a helical gear is spur's, inherited unchanged, so nothing
// else is proved here. The tooth math itself is imported from
// proof/involute rather than derived again.
//
// What no harness here reaches, and why. Neither harness models a command
// dialog, a Fusion user parameter, a class hierarchy or an entity's
// visibility, so the steps that add the Helix Angle input, register the
// HelixAngle parameter in radians, return the cos(HelixAngle) fillet factor,
// and leave the helix plane lit while the Twisted Gear Profile sketch stays
// hidden are all outside it. They are checked by the emit-stage gates against
// the generated module instead. The one geometric step that is also outside is
// the offset construction plane: a plane on its own closes no sketch and bounds
// no body, so there is nothing for either harness to gate. What the plane is
// for is proved anyway, one step later — see the note in solids_test.go.
package helicalgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
)

// gear is one parameter case, read out of the harness's float map so a case
// table stays a plain table.
type gear struct {
	module        float64
	toothNumber   float64
	pressureAngle float64
	helixAngle    float64
	thickness     float64
	involuteSteps int
}

func gearOf(p map[string]float64) gear {
	return gear{
		module:        p["module"],
		toothNumber:   p["toothNumber"],
		pressureAngle: p["pressureAngle"],
		helixAngle:    p["helixAngle"],
		thickness:     p["thickness"],
		involuteSteps: int(p["involuteSteps"]),
	}
}

func (g gear) dims() involute.Dimensions {
	return involute.Derive(g.module, g.toothNumber, g.pressureAngle)
}

// deg converts for case tables, which read better in degrees.
func deg(d float64) float64 { return d * math.Pi / 180 }

// profileStyle selects how faithfully a section is drawn.
//
// faithfulProfile is the sketch as Fusion draws it: the two flanks are fitted
// splines, the root circle is solid geometry, and the tooth's root boundary is
// the arc that profile detection cuts out of that circle where the two
// flank-to-root lines land on it. That is the sketch the constraint scheme has
// to close, so the sketch step proves this one.
//
// chordedProfile is the substitute the solid harness accepts. decad's loft
// pairs the two sections segment by segment and admits only line, arc and
// circle segments, so a fitted spline is refused outright; and a circular pair
// is chorded into faceted walls, whose area, volume and centroid readings then
// fall outside decad's default tolerance and are reported as diagnostics that
// no harness gate accepts. Replacing each flank spline with a polyline through
// the SAME sample points, and the tooth-top and root arcs with their chords,
// leaves an all-line section that lofts exactly. The constraint scheme is
// untouched — every rib, dimension and midpoint is the same, because the ribs
// attach to the flank sample points either way.
//
// What the substitution costs: the lofted body is the tooth with its flanks,
// tip and root flattened to chords, so its cross-section is slightly smaller
// than the real tooth's, and the loop no longer carries the curve types
// [HELI-F-LOFT]'s profile search matches on. Both of those are asserted on the
// faithful sketch instead, in stepTwistedGearProfile. One constraint also moves:
// the chord that replaces the tooth-top arc no longer implies the last rib's
// perpendicular, so the chorded section adds back the one perpendicular
// [SPUR-F-RIBS] omits, and is fully constrained on the same terms.
type profileStyle int

const (
	faithfulProfile profileStyle = iota
	chordedProfile
)

// section is the drawn tooth profile and the handles a later assertion reads.
type section struct {
	origin    *sketch.Point
	reference *sketch.Line
	spine     *sketch.Line
	toothTop  *sketch.Point
	leftPts   []*sketch.Point
	rightPts  []*sketch.Point
	dims      involute.Dimensions
	embedded  bool
}

// drawSection draws one gear profile at g.helixAngle into s.
//
// This is the spur tooth generator's draw(anchorPoint, angle) as helical calls
// it: the whole tooth is drawn already rotated by the helix angle in the point
// math ([SPUR-F-ROTATE-CONFIRM]), and the angular dimension from the +X
// reference to the spine carries that same angle, sign included. Helical adds
// nothing to it — it only passes a non-zero angle.
func drawSection(t testing.TB, s *sketch.Sketch, g gear, style profileStyle) *section {
	t.Helper()
	d := g.dims()
	angle := g.helixAngle

	// The Tools-sketch projection of the user's anchor. Fusion projects it in;
	// the engine's reference point is the same thing — a point this sketch does
	// not own the coordinates of. The local origin is a separate, movable sketch
	// point, and the coincidence between the two is the step-5 anchoring
	// ([SPUR-F-LOCAL-ORIGIN], [SPUR-F-ANCHOR-CHAIN]).
	anchor := s.CreateReferencePoint(0, 0, "toolsSketchAnchor")
	anchor.SetName("projected anchor")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	circle := func(r float64, construction bool, name string) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		c.SetName(name)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	// Only the root circle is solid geometry; the other three are construction
	// and so bound no profile. In the chorded substitute the root circle is
	// construction too, because the chord that replaces the root arc is drawn
	// explicitly and a solid circle would arrange with it.
	circle(d.Root, style == chordedProfile, "root circle")
	tip := circle(d.Tip, true, "tip circle")
	circle(d.Base, true, "base circle")
	circle(d.Pitch, true, "pitch circle")

	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, g.toothNumber, g.involuteSteps, angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if style == chordedProfile {
		for i := 1; i < len(leftPts); i++ {
			s.CreateLine(leftPts[i-1], leftPts[i])
			s.CreateLine(rightPts[i-1], rightPts[i])
		}
	} else {
		if _, err := s.CreateFitSpline(leftPts...); err != nil {
			t.Fatalf("left flank spline: %v", err)
		}
		if _, err := s.CreateFitSpline(rightPts...); err != nil {
			t.Fatalf("right flank spline: %v", err)
		}
	}

	// The tooth-top arc caps the tooth at the tip circle. Its centre is the
	// local origin and it carries no diameter dimension ([SPUR-F-TOOTHTOP-ARC]).
	// Fusion's addByCenterStartEnd copies the centre and so needs an explicit
	// addCoincident back to the origin; the engine shares the point handle it is
	// given, which is the same constraint by the other of the two routes
	// [PB-SHARE-XOR-COINCIDENT] allows.
	last := len(leftPts) - 1
	if style == chordedProfile {
		s.CreateLine(rightPts[last], leftPts[last])
	} else {
		s.CreateArc(origin, rightPts[last], leftPts[last])
	}

	// Spine, +X reference and the angular pin ([SPUR-F-SPINE]). The reference
	// line's far end is pinned with signed horizontal and vertical distances
	// rather than put on the tip circle, and the angular dimension runs from the
	// reference to the spine, so the twist carries a direction and a sign.
	ttx, tty := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(ttx, tty)
	toothTop.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	reference.SetName("+X reference")
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	// Ribs, in the exact [SPUR-F-RIBS] order. The rib takes the axis across the
	// spine and the midpoint chain the axis along it, and which axis that is
	// swaps once the tooth passes 45 degrees.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prevMid := origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		mx, my := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		// The last rib joins the two flank tips, which the tooth-top arc already
		// holds either side of the spine, so its perpendicular is redundant and
		// Fusion rejects it. The chorded substitute has no arc there, so it puts
		// that one perpendicular back: without it the two tips keep a shared
		// degree of freedom and the section is not fully constrained.
		if i != last || style == chordedProfile {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prevMid, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevMid, mid, my-prevY))
		}
		prevMid, prevX, prevY = mid, mx, my
	}

	// Flank-to-root lines, and the strict embedded test ([SPUR-F-FLANK-ROOT]).
	// When the flank starts inside the root circle no stub is drawn and the loop
	// loses its two lines — the shape helical has no branch for.
	embedded := d.Embedded()
	if !embedded {
		stub := func(flankStart *sketch.Point, seed involute.Pt) *sketch.Point {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := d.Root*seed.X/n, d.Root*seed.Y/n
			foot := s.CreatePoint(rx, ry)
			s.CreateLine(foot, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, foot, rx),
				sketch.NewVerticalDistance(origin, foot, ry),
			)
			return foot
		}
		leftFoot := stub(leftPts[0], left[0])
		rightFoot := stub(rightPts[0], right[0])
		if style == chordedProfile {
			s.CreateLine(rightFoot, leftFoot)
		}
	}

	return &section{
		origin:    origin,
		reference: reference,
		spine:     spine,
		toothTop:  toothTop,
		leftPts:   leftPts,
		rightPts:  rightPts,
		dims:      d,
		embedded:  embedded,
	}
}

// solve runs the solver so every later reading is taken off solved geometry
// rather than off the seed coordinates ([PB-SOLVED-GEOMETRY]).
func solve(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e, DOF %d", res.Residual, res.DOF)
	}
}

// curveCounts is one profile's boundary, counted by curve type.
type curveCounts struct {
	splines int
	lines   int
	arcs    int
	circles int
}

func countCurves(p *sketch.Profile) curveCounts {
	var c curveCounts
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			c.splines++
		case *sketch.Line:
			c.lines++
		case *sketch.Arc:
			c.arcs++
		case *sketch.Circle:
			c.circles++
		}
	}
	return c
}

// toothProfile returns the tooth region of a chorded section — the only region
// such a section closes, since its root circle is construction geometry.
func toothProfile(t testing.TB, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("a chorded section closes exactly one region, the tooth; got %d", len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatal("the chorded tooth region is not an extrudable profile")
	}
	return profiles[0]
}
