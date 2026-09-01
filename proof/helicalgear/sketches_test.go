package helicalgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// twistedProfileCases is the regime the twisted profile's constraint scheme has
// to hold across.
//
// Spur's Sketch Discipline names that regime and helical adds a sign to it. The
// table therefore sweeps sizes, the whole signed range of the draw() angle
// argument — a negative angle is a left-hand helix, and the dialog accepts one —
// the rib count that Involute Steps sets, and both routes into the embedded
// shape. A quarter turn is in the table because that is where |sin| exceeds
// |cos| and the rib and chain dimensions swap axes ([SPUR-F-RIBS]); it is a
// twist no gear would be cut with, and it is here because the scheme, not the
// gear, is what is on trial.
var twistedProfileCases = []proofkit.Case{
	{Name: "default-m1-z17-plus14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(14.5), "involuteSteps": 15}},
	{Name: "left-hand-m1-z17-minus14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(-14.5), "involuteSteps": 15}},
	{Name: "untwisted-m1-z17-zero", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": 0, "involuteSteps": 15}},
	{Name: "coarse-m3-z15-plus14.5", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": rad(20), "helixAngle": rad(14.5), "involuteSteps": 15}},
	{Name: "coarse-m2-z20-minus35", Params: map[string]float64{
		"module": 2, "toothNumber": 20, "pressureAngle": rad(20), "helixAngle": rad(-35), "involuteSteps": 15}},
	{Name: "fine-m1-z12-plus35", Params: map[string]float64{
		"module": 1, "toothNumber": 12, "pressureAngle": rad(20), "helixAngle": rad(35), "involuteSteps": 15}},
	{Name: "quarter-turn-plus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(90), "involuteSteps": 15}},
	{Name: "quarter-turn-minus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(-90), "involuteSteps": 15}},
	{Name: "few-ribs-4-steps", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(14.5), "involuteSteps": 4}},
	{Name: "few-ribs-4-steps-negative", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "helixAngle": rad(-14.5), "involuteSteps": 4}},
	{Name: "embedded-by-tooth-count-z45", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": rad(20), "helixAngle": rad(14.5), "involuteSteps": 15}},
	{Name: "embedded-by-pressure-angle-z30-pa25", Params: map[string]float64{
		"module": 1, "toothNumber": 30, "pressureAngle": rad(25), "helixAngle": rad(14.5), "involuteSteps": 15}},
}

// stepTwistedGearProfile draws the "Twisted Gear Profile" sketch: the spur
// tooth generator run at angle=HelixAngle on the offset helix plane.
//
// Helical writes no geometry of its own here. It hands the helix angle to
// draw(anchorPoint, angle=...) and the spur generator draws the whole tooth
// already rotated by it, so what this step proves is spur's angle != 0 path
// ([SPUR-F-SPINE], [SPUR-F-ROTATE-CONFIRM]) at the angles helical passes it.
//
// The sketch is drawn the way Fusion draws it, not the way a stand-in would be:
// the anchor arrives as a projected reference point and the local origin is
// constrained onto it rather than fixed; the root circle is solid and the other
// three are construction; and the tooth loop's root arc is left to profile
// detection, which splits the root circle where the two flank-to-root lines
// land on it. That last point is why this step can assert the curve counts at
// all — they are counted on the loops the drawn sketch actually closed.
func stepTwistedGearProfile(t testing.TB, s *sketch.Sketch, raw map[string]float64) {
	p := params(raw)
	dim := p.dimensions()
	angle := p.helixAngle()

	proofkit.Step(t, "anchor the sketch on the projected Tools-sketch point")
	// [SPUR-F-ANCHOR-CHAIN] / [SPUR-F-LOCAL-ORIGIN]: the Tools-sketch anchor is
	// projected in (reference geometry, externally locked) and the sketch's own
	// movable local origin is constrained coincident with it. Fixing the local
	// origin's coordinates instead would ground the sketch without modelling the
	// projection chain the whole gear tracks the anchor through.
	projectedAnchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	localOrigin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(localOrigin, projectedAnchor))

	proofkit.Step(t, "four gear circles on the shared local origin")
	circle := func(radius float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(localOrigin, radius)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*radius))
		return c
	}
	// The root circle is solid; the tip, base and pitch circles are construction
	// (spur step 3). Only the solid one bounds a profile, and the tooth loop's
	// root arc is a piece of it.
	circle(dim.Root, false)
	tipCircle := circle(dim.Tip, true)
	circle(dim.Base, true)
	circle(dim.Pitch, true)

	proofkit.Step(t, "involute flanks, drawn already rotated by the helix angle")
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, p.toothNumber(), p.involuteSteps(), angle)
	leftPoints := make([]*sketch.Point, len(left))
	rightPoints := make([]*sketch.Point, len(right))
	for i := range left {
		leftPoints[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPoints[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPoints...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPoints...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	proofkit.Step(t, "tooth-top arc sharing the local origin as its centre")
	// [SPUR-F-TOOTHTOP-ARC]: centre shared with the local origin, ends shared
	// with the flanks, and no diameter dimension. The arc sweeps counter-
	// clockwise from the right flank's end to the left flank's, so it caps the
	// tooth rather than cutting back through it.
	toothTopArc := s.CreateArc(localOrigin, rightPoints[len(rightPoints)-1], leftPoints[len(leftPoints)-1])

	proofkit.Step(t, "spine, +X reference and the angular pin that carries the sign")
	// [SPUR-F-SPINE]: the tooth-top point is materialised at its rotated
	// position and pinned onto the tip circle; the spine shares it and the local
	// origin.
	topX, topY := involute.Rotate(dim.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))
	spine := s.CreateLine(localOrigin, toothTop)
	spine.SetConstruction(true)
	// The reference line is built for every angle including 0. Its far end is
	// pinned with two axis distances from the local origin rather than by
	// touching the tip circle, and the angular dimension runs from the reference
	// to the spine in that argument order. That dimension is what says which way
	// the spine points: it carries the sign, so a left-hand helix and a
	// right-hand one of the same size are different sketches here, not the same
	// sketch read two ways.
	referenceEnd := s.CreatePoint(dim.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(localOrigin, referenceEnd, dim.Tip),
		sketch.NewVerticalDistance(localOrigin, referenceEnd, 0),
	)
	referenceLine := s.CreateLine(localOrigin, referenceEnd)
	referenceLine.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(referenceLine, spine, deg(angle)))

	proofkit.Step(t, "one rib per involute sample, in [SPUR-F-RIBS] order")
	// The rib takes the axis ACROSS the spine and the midpoint chain the axis
	// ALONG it; which axis is which swaps once |sin| passes |cos|, which is why
	// the quarter-turn cases are in the table.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previousMid := localOrigin
	previousMidX, previousMidY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPoints[i], rightPoints[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPoints[i], rightPoints[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPoints[i], rightPoints[i], right[i].X-left[i].X))
		}
		// The midpoint is seeded at the foot of the left fit point on the spine,
		// not at the rib's own 2-D midpoint and not at (fitX, 0).
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			// The last rib carries no perpendicular: the tooth-top arc already
			// holds the two flank tips at equal radius either side of the spine,
			// so adding it is the redundant constraint Fusion rejects.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previousMid, mid, midX-previousMidX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previousMid, mid, midY-previousMidY))
		}
		previousMid, previousMidX, previousMidY = mid, midX, midY
	}

	embedded := dim.Embedded()
	if !embedded {
		proofkit.Step(t, "flank-to-root lines, two axis dimensions each")
		// [SPUR-F-FLANK-ROOT]: the root end is seeded at its computed position and
		// placed with exactly two axis dimensions from the local origin. The
		// engine's targets are signed, which is how the seed side crosses over
		// from Fusion, where the same side is carried by the seed and the value is
		// a magnitude ([PB-DIM-VALUE-SEMANTICS]).
		stub := func(flankStart *sketch.Point, seed involute.Pt) {
			end := footOnRoot(seed, dim.Root)
			rootEnd := s.CreatePoint(end.X, end.Y)
			s.CreateLine(rootEnd, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(localOrigin, rootEnd, end.X),
				sketch.NewVerticalDistance(localOrigin, rootEnd, end.Y),
			)
		}
		stub(leftPoints[0], left[0])
		stub(rightPoints[0], right[0])
	}

	proofkit.Step(t, "solve, then read the twist and the loop counts off the drawing")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}

	// The twist the sketch delivers is the helix angle itself. Nothing rescales
	// it — no lead angle, no dependence on Thickness — so the solved tooth-top
	// point sits at exactly the helix angle from +X, sign included. A scheme
	// that dropped the angular dimension's sign would still solve, and would
	// fail here.
	if got, want := polarAngle(toothTop.X(), toothTop.Y()), angleInPi(angle); math.Abs(got-want) > 1e-9 {
		t.Errorf("solved tooth-top sits at %.6f deg from +X, helix angle is %.6f deg", deg(got), deg(want))
	}

	// The curve counts of the two loops are a contract, not a description:
	// loftTooth finds both loft sections with find_profile_by_curve_counts at a
	// fixed nurbs=2, arcs=2, lines=2, and the inherited chamferTooth picks the
	// cap face by an edge count that follows from the same loop
	// ([HELI-F-CHAMFER-COUNT] asserts the one-curve-one-edge correspondence;
	// no harness measures a Fusion cap face, so the loop count is the provable
	// half and this is where it is proven).
	tooth, disc := toothAndDisc(t, s, toothTopArc)
	nurbs, arcs, lines := countCurves(tooth)
	wantLines := 2
	if embedded {
		// The embedded tooth has no flank-to-root stubs: the flanks themselves
		// cross the root circle. Its loop is the 4-curve one, which the fixed
		// lines=2 search does not match — helical's documented limitation, and
		// the reason an embedded helical gear at a non-zero chamfer raises.
		wantLines = 0
	}
	if nurbs != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("tooth loop has nurbs=%d arcs=%d lines=%d, want nurbs=2 arcs=2 lines=%d (embedded=%v)",
			nurbs, arcs, lines, wantLines, embedded)
	}

	// The disc inside the root circle is the second region the sketch closes.
	// Fusion reports its boundary as the two arcs the flank-to-root lines cut
	// the root circle into; this engine reports the same region bounded by the
	// whole circle. The area is the reading both agree on, so that is what is
	// asserted here rather than a curve count that is an artifact of which
	// engine is looking.
	if want := math.Pi * dim.Root * dim.Root; math.Abs(disc.Area-want) > 1e-6*want {
		t.Errorf("disc region area %.6f mm^2, want %.6f mm^2", disc.Area, want)
	}
}

// angleInPi folds an angle into (-pi, pi], the range Atan2 reports in, so a
// quarter-turn case compares against the same branch the reading comes back on.
func angleInPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// toothAndDisc splits the two regions the sketch closes: the tooth, which is
// the one bounded by the tooth-top arc, and the disc inside the root circle.
//
// Both come out of a single Profiles call. Each call rebuilds the arrangement
// and hands back fresh values, so two calls cannot be compared against each
// other.
func toothAndDisc(t testing.TB, s *sketch.Sketch, toothTopArc sketch.Entity) (tooth, disc *sketch.Profile) {
	t.Helper()
	all := s.Profiles()
	if len(all) != 2 {
		t.Fatalf("sketch closed %d region(s), want the tooth and the disc inside the root circle", len(all))
	}
	for i, p := range all {
		for _, e := range p.Entities {
			if e == toothTopArc {
				return all[i], all[1-i]
			}
		}
	}
	t.Fatal("neither detected region is bounded by the tooth-top arc")
	return nil, nil
}

// countCurves counts a region's boundary by the curve types Fusion's profile
// search matches on.
//
// It counts distinct entities rather than boundary edges. A closed curve's own
// seam can split one fragment into two edges when the seam falls inside it —
// the untwisted case does exactly that — and Fusion, which splits the root
// circle into curves at the crossings only, would count one arc there. Counting
// entities is the faithful reading; counting edges would make the untwisted
// case report a curve Fusion does not have.
func countCurves(p *sketch.Profile) (nurbs, arcs, lines int) {
	partial := map[sketch.Entity]bool{}
	for _, e := range p.Outer {
		if e.Partial {
			partial[e.Entity] = true
		}
	}
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.Line:
			lines++
		case *sketch.Arc:
			arcs++
		case *sketch.Circle:
			// A circle on a region's boundary reaches Fusion's profile search as
			// an arc: it is there as the piece the tooth cut out of it.
			if partial[e] {
				arcs++
			}
		case *sketch.FitSpline, *sketch.Spline, *sketch.NURBS:
			nurbs++
		}
	}
	return nurbs, arcs, lines
}
