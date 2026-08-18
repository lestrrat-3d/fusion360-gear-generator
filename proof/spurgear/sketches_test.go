// Package spurgear_test is the compiled proof of the spur gear step list.
//
// Every step function here is named by exactly one step of
// spec/spurgear/steps.md, and every [GO] step of that list names one of these.
// The sketch steps run through proofkit and are gated on the sketch engine's
// own verdict; the solid steps run through proofkit3d and are gated on decad's.
package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys. Lengths are millimetres, angles radians, exactly as the spec
// states them; the sketch engine's base length unit is the millimetre.
const (
	pModule        = "module"
	pToothNumber   = "toothNumber"
	pPressureAngle = "pressureAngle"
	pSteps         = "involuteSteps"
	pAngle         = "angle"
	pThickness     = "thickness"
	pBoreDiameter  = "boreDiameter"
)

// gearCases is the parameter sweep every spur proof runs against.
//
// It spans the two shapes the tooth profile can take — the ordinary case where
// the flank starts outside the root circle and needs the flank-to-root stubs,
// and the embedded case at high tooth count where it starts inside and needs
// none — and the two rotations the tooth generator's angle argument has to
// survive, because the spur base calls draw() with angle 0 while helical,
// herringbone and the bevel virtual tooth call it with a real angle through the
// same code.
func gearCases() []proofkit.Case {
	return []proofkit.Case{
		{Name: "default_M1_T17_PA20", Params: map[string]float64{
			pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9,
			pSteps: 15, pAngle: 0, pThickness: 10, pBoreDiameter: 5}},
		{Name: "coarse_M4_T12_PA20", Params: map[string]float64{
			pModule: 4, pToothNumber: 12, pPressureAngle: math.Pi / 9,
			pSteps: 15, pAngle: 0, pThickness: 25, pBoreDiameter: 12}},
		{Name: "fine_M0.5_T30_PA14.5", Params: map[string]float64{
			pModule: 0.5, pToothNumber: 30, pPressureAngle: 14.5 * math.Pi / 180,
			pSteps: 15, pAngle: 0, pThickness: 4, pBoreDiameter: 3}},
		{Name: "embedded_M2_T60_PA20", Params: map[string]float64{
			pModule: 2, pToothNumber: 60, pPressureAngle: math.Pi / 9,
			pSteps: 15, pAngle: 0, pThickness: 10, pBoreDiameter: 20}},
		{Name: "rotated_30deg_M1_T17", Params: map[string]float64{
			pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9,
			pSteps: 15, pAngle: math.Pi / 6, pThickness: 10, pBoreDiameter: 5}},
		{Name: "rotated_90deg_M1_T17", Params: map[string]float64{
			pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9,
			pSteps: 15, pAngle: math.Pi / 2, pThickness: 10, pBoreDiameter: 5}},
	}
}

func dimensionsOf(p map[string]float64) involute.Dimensions {
	return involute.Derive(p[pModule], p[pToothNumber], p[pPressureAngle])
}

// ---------------------------------------------------------------------------
// Step: Tools Sketch
// ---------------------------------------------------------------------------

// stepToolsSketch is the Tools sketch: it draws no geometry of its own and
// exists to own one projection of the user's anchor point, which every later
// sketch re-projects from.
//
// The projection is a reference point, which is what sketch.project gives
// Fusion — geometry whose position is handed in from outside and which the
// solver never moves. Modelling it as an ordinary point with fixed coordinates
// would prove a different sketch: a projection tracks its source, and this is
// the sketch that owns that link.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the anchor point into the Tools sketch")
	anchor := s.CreateReferencePoint(0, 0, "user anchor point")
	anchor.SetName("ctx.anchorPoint")
}

func TestToolsSketch(t *testing.T) {
	proofkit.Run(t, gearCases(), stepToolsSketch)
}

// ---------------------------------------------------------------------------
// Step: Gear Profile Sketch
// ---------------------------------------------------------------------------

// stepGearProfileSketch draws the whole Gear Profile sketch — the four gear
// circles, the involute tooth, the tooth-top arc, the spine and its angular
// pin, the rib chain, the flank-to-root stubs, and the anchoring that slides
// all of it onto the user's anchor.
//
// It is one step because it is one Fusion timeline entry. The order below is
// the order draw() performs: drawCircles, drawTooth, then the anchoring.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	dim := dimensionsOf(p)
	teeth := p[pToothNumber]
	samples := int(p[pSteps])
	angle := p[pAngle]
	ca, sa := math.Cos(angle), math.Sin(angle)

	// drawCircles. Every circle shares the local origin as its centre point,
	// rather than being created from its coordinates and then re-coincidented;
	// a driving diameter dimension fixes each radius.
	proofkit.Step(t, "local origin and the four gear circles")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")
	root := s.CreateCircle(origin, dim.Root)
	tip := s.CreateCircle(origin, dim.Tip)
	base := s.CreateCircle(origin, dim.Base)
	pitch := s.CreateCircle(origin, dim.Pitch)
	for _, c := range []*sketch.Circle{tip, base, pitch} {
		c.SetConstruction(true)
	}
	s.AddConstraint(
		sketch.NewDiameter(root, 2*dim.Root),
		sketch.NewDiameter(tip, 2*dim.Tip),
		sketch.NewDiameter(base, 2*dim.Base),
		sketch.NewDiameter(pitch, 2*dim.Pitch),
	)

	// drawTooth, part one: the two flank splines. involute.Flanks does the
	// sampling, the mirror, the pitch-crossing rotation and the requested
	// angle, in that order; the tooth arrives already at its final position.
	proofkit.Step(t, "involute flanks, %d samples each, drawn at angle %.4f rad", samples, angle)
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, teeth, samples, angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "%d usable involute samples cannot make a flank", len(left))
	}
	leftFit := make([]*sketch.Point, len(left))
	rightFit := make([]*sketch.Point, len(right))
	for i := range left {
		leftFit[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightFit[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftFit...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rightFit...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	// The tooth-top arc. Its centre is the shared local origin and its ends are
	// the two flank ends, shared as well, so it needs no diameter dimension —
	// and must not get one, because a free centre plus a radius leaves an
	// inward-bulging arc equally valid.
	proofkit.Step(t, "tooth-top point and the tooth-top arc")
	last := len(left) - 1
	toothTop := s.CreatePoint(dim.Tip*ca, dim.Tip*sa)
	toothTop.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	s.CreateArc(origin, rightFit[last], leftFit[last])

	// The spine and the +X reference line it is dimensioned against. The
	// reference end is pinned with two signed dimensions rather than a
	// coincidence to the tip circle, and the angular dimension is what says
	// which way the spine points — a plain horizontal would leave the tooth
	// free to settle 180 degrees around.
	proofkit.Step(t, "spine, +X reference line and the confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	refEnd := s.CreatePoint(dim.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, dim.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	reference.SetName("+X reference")
	// NewAngle reads the sketch's default angle unit, which is degrees.
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	// The ribs. One per fit-point index, endpoints included: the fit points
	// carry no other constraint, so a missing endpoint rib leaves the sketch
	// under-constrained. The last rib takes no perpendicular, because the
	// tooth-top arc already holds its two ends at equal radius either side of
	// the spine and a perpendicular there is redundant.
	proofkit.Step(t, "%d ribs, midpoints chained along the spine", len(left))
	acrossIsVertical := math.Abs(ca) >= math.Abs(sa)
	previous := origin
	for i := range left {
		rib := s.CreateLine(leftFit[i], rightFit[i])
		rib.SetConstruction(true)
		// A signed dimension across the spine. An aligned one gives only the
		// length, which the left and right flanks satisfy equally well swapped
		// over, and the tooth comes out mirrored.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftFit[i], rightFit[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftFit[i], rightFit[i], right[i].X-left[i].X))
		}
		// The midpoint is seeded at the foot of the left fit point on the
		// spine, not at the rib's own 2-D midpoint.
		foot := left[i].X*ca + left[i].Y*sa
		mid := s.CreatePoint(foot*ca, foot*sa)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		// The chain runs outward from the local origin, signed along the spine.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, mid.X()-previous.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, mid.Y()-previous.Y()))
		}
		previous = mid
	}

	// The flank-to-root stubs, drawn only when the flank starts outside the
	// root circle. The root end is placed by two signed offsets from the local
	// origin; "on the root circle plus origin on the line" would be satisfied
	// by the far intersection too, and the stub would run right across the gear.
	if dim.Embedded() {
		proofkit.Step(t, "embedded profile: base circle inside the root circle, no flank-to-root lines")
	} else {
		proofkit.Step(t, "flank-to-root lines on both flanks")
		for _, side := range []struct {
			fit   *sketch.Point
			start involute.Pt
		}{{leftFit[0], left[0]}, {rightFit[0], right[0]}} {
			theta := math.Atan2(side.start.Y, side.start.X)
			rx, ry := dim.Root*math.Cos(theta), dim.Root*math.Sin(theta)
			rootEnd := s.CreatePoint(rx, ry)
			s.CreateLine(rootEnd, side.fit)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, rx),
				sketch.NewVerticalDistance(origin, rootEnd, ry),
			)
		}
	}

	// The anchoring, which draw() performs after drawCircles and drawTooth.
	// Everything above is drawn relative to the local origin, so this one
	// coincidence drags the whole tooth profile onto the user's anchor.
	proofkit.Step(t, "anchor the local origin onto the re-projected Tools anchor")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	anchor.SetName("projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
}

func TestGearProfileSketch(t *testing.T) {
	proofkit.Run(t, gearCases(), stepGearProfileSketch)
}

// ---------------------------------------------------------------------------
// Step: Bore Profile Sketch
// ---------------------------------------------------------------------------

// stepBoreProfileSketch draws the Bore Profile sketch: the bore circle centred
// on a fresh projection of the Tools anchor, plus the stray local-origin point
// the tooth generator's constructor always adds, grounded on that same
// projection so this sketch fully constrains like every other.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	bore := p[pBoreDiameter]
	if bore <= 0 {
		proofkit.Unmodelled(t, "bore diameter %.3f is not positive, so no Bore Profile sketch is drawn", bore)
	}

	proofkit.Step(t, "re-project the Tools anchor into the Bore Profile sketch")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	anchor.SetName("projected anchor")

	proofkit.Step(t, "the tooth generator's constructor adds a local origin at (0,0)")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")

	proofkit.Step(t, "bore circle of diameter %.3f on the projection, with a driving diameter dimension", bore)
	circle := s.CreateCircle(anchor, bore/2)
	s.AddConstraint(sketch.NewDiameter(circle, bore))

	proofkit.Step(t, "ground the stray local origin on the projection, not on the sketch origin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
}

func TestBoreProfileSketch(t *testing.T) {
	proofkit.Run(t, gearCases(), stepBoreProfileSketch)
}
