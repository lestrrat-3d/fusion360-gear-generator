// Package helicalgear_test proves the helical gear's two deltas from the spur
// build: the twisted "Twisted Gear Profile" sketch that becomes the top loft
// section, and the loft that replaces spur's tooth extrude.
//
// Everything else in a helical gear is spur's code, inherited unchanged, and is
// proven by spur's own proof. Nothing here re-proves it.
//
// Eight of the compiled steps are [PROSE] and no case in this directory reaches
// them, because none of them puts geometry anywhere. H1 (module layout and
// imports), H2 (the Helix Angle dialog input), H3 (the two new context fields),
// H4 (newContext / prefixBase / generateName) and H5 (registering the HelixAngle
// user parameter) are all Fusion command-dialog and user-parameter surface: the
// sketch engine has no dialog, no parameter table and no class hierarchy, so
// there is no substitute geometry to draw for them. H6, the cos(HelixAngle)
// fillet factor, is an expression string spliced into a Fusion parameter and
// consumed by spur's inherited createFillets; the root fillet it scales is
// spur's feature, not helical's, so the proof that it lands on the right edges
// belongs to spur. H10 is the inherited spur pipeline helical must not
// re-implement, which is spur's proof to make and not this one's. H7, the offset
// construction plane, is the one [PROSE] step with a measurable consequence, and
// solids_test.go pins that consequence next to the plane it builds.
package helicalgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// sketchCases sweeps the regime the spur spec declares for the tooth
// generator's `angle` argument, which is the regime helical hands it.
//
// The helix angle is signed and the sign is the hand of the helix, so every
// magnitude that appears here appears with both signs: a scheme that drops or
// flips the confirming angular dimension still solves at +angle and comes out
// mirrored at -angle, and only the negative case sees it. The quarter turn is
// where |sin| exceeds |cos| and the rib and the midpoint chain swap which axis
// they are dimensioned on. Zero is the spur baseline, and it is also the
// BOTTOM loft section: helical draws that one through the same generator at
// angle 0, so proving it here proves both sections of the loft.
//
// The anchor cases drag the whole drawing off the sketch origin, which is what
// Fusion does when the user's anchor point is not at the origin. A spur gear
// usually hides a stranded tooth-top arc centre because its drag is zero.
//
// Both routes into the embedded profile are covered — a high tooth count at the
// ordinary pressure angle, and a moderate tooth count at a large one — because
// the two reach the same missing-stub shape through different terms. Helical
// cannot loft an embedded profile (see solids_test.go), but it can still draw
// one, and the curve count it draws is what tells you why the loft fails.
var sketchCases = []proofkit.Case{
	{Name: "default_m1_z17_helix_plus14_5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "left_hand_m1_z17_helix_minus14_5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "bottom_section_helix_zero", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": 0, "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "quarter_turn_plus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(90), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "quarter_turn_minus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-90), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "anchor_dragged_plus14_5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "anchorX": 20, "anchorY": 15,
	}},
	{Name: "anchor_dragged_minus14_5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "anchorX": 20, "anchorY": -15,
	}},
	{Name: "coarse_m3_z15", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "fine_m0_5_z40", Params: map[string]float64{
		"module": 0.5, "toothNumber": 40, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "few_ribs_steps4", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 4, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "embedded_by_tooth_count_z45", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "embedded_by_pressure_angle_z30_pa25", Params: map[string]float64{
		"module": 1, "toothNumber": 30, "pressureAngle": radians(25),
		"helixAngle": radians(14.5), "involuteSteps": 15, "anchorX": 0, "anchorY": 0,
	}},
}

// stepTwistedGearProfileSketch draws the Twisted Gear Profile sketch: the spur
// tooth generator run at angle = HelixAngle, on the offset helix plane.
//
// Helical adds no constraint of its own here. What it does is take the spur
// generator's angle != 0 path, which spur itself never runs, so this is the
// first place that path is held to DOF 0.
func stepTwistedGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	toothNumber := p["toothNumber"]
	angle := p["helixAngle"]
	steps := int(p["involuteSteps"])
	dim := involute.Derive(p["module"], toothNumber, p["pressureAngle"])

	// The HelixAngle user parameter (step H5) is registered in radians from a
	// degree dialog input, and the generator reads its raw .value before handing
	// it to draw(). That conversion is dialog plumbing the sketch engine has no
	// counterpart for; what arrives here is the radian value it produces.
	// cos(HelixAngle), the fillet factor of step H6, is likewise an expression
	// consumed by spur's createFillets and never reaches this sketch.
	t.Logf("module=%g toothNumber=%g helix=%.3f deg steps=%d base=%.4f root=%.4f tip=%.4f embedded=%v",
		p["module"], toothNumber, angle*180/math.Pi, steps, dim.Base, dim.Root, dim.Tip, dim.Embedded())

	proofkit.Step(t, "project the Tools-sketch anchor in and give this sketch its own local origin")
	// The anchor arrives as a projection of the Tools sketch's own projection of
	// the user's point, so it is reference geometry: externally located, and not
	// something this sketch's solver may move.
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "Tools sketch anchor projection")
	anchor.SetName("projectedAnchor")
	// A fresh movable point, never the sketch's own origin, because the whole
	// drawing is built about it and then dragged onto the anchor.
	local := s.CreatePoint(0, 0)
	local.SetName("localOrigin")

	proofkit.Step(t, "four circles, every one centred on the shared local origin")
	circle := func(radius float64, construction bool, name string) *sketch.Circle {
		c := s.CreateCircle(local, radius)
		c.SetConstruction(construction)
		c.SetName(name)
		s.AddConstraint(sketch.NewDiameter(c, 2*radius))
		return c
	}
	// Only the root circle is solid geometry. The other three bound no profile,
	// which is why the tooth loop below is closed by a fragment of this one.
	rootCircle := circle(dim.Root, false, "rootCircle")
	tipCircle := circle(dim.Tip, true, "tipCircle")
	circle(dim.Base, true, "baseCircle")
	circle(dim.Pitch, true, "pitchCircle")

	proofkit.Step(t, "involute flanks, drawn already rotated by the helix angle")
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, toothNumber, steps, angle)
	if len(left) < 2 {
		t.Fatalf("involute sampling left %d usable points", len(left))
	}
	leftPoints := make([]*sketch.Point, len(left))
	rightPoints := make([]*sketch.Point, len(right))
	for i := range left {
		leftPoints[i] = s.CreatePoint(left[i].X, left[i].Y)
		leftPoints[i].SetName(fmt.Sprintf("leftFit%d", i))
		rightPoints[i] = s.CreatePoint(right[i].X, right[i].Y)
		rightPoints[i].SetName(fmt.Sprintf("rightFit%d", i))
	}
	leftFlank, err := s.CreateFitSpline(leftPoints...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	rightFlank, err := s.CreateFitSpline(rightPoints...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	leftFlank.SetName("leftFlank")
	rightFlank.SetName("rightFlank")

	proofkit.Step(t, "tooth-top point and the arc that caps the tooth on the tip circle")
	topX, topY := involute.Rotate(dim.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	toothTop.SetName("toothTopPoint")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))

	// Fusion's addByCenterStartEnd shares the two ends and COPIES the centre, so
	// the arc arrives with a centre of its own that nothing holds. The fresh
	// point here is that copy, and the coincidence below is what ties it back.
	// Without it the centre is free, the sketch is 2 DOF short, and the arc's
	// radius becomes whatever the solver lands on once the drawing is dragged
	// onto the anchor. The anchor_dragged cases are the ones that would see it.
	arcCentre := s.CreatePoint(0, 0)
	arcCentre.SetName("toothTopArcCentre")
	toothTopArc := s.CreateArc(arcCentre, rightPoints[len(rightPoints)-1], leftPoints[len(leftPoints)-1])
	toothTopArc.SetName("toothTopArc")
	s.AddConstraint(sketch.NewCoincident(arcCentre, local))

	proofkit.Step(t, "spine, +X reference line and the angular dimension that carries the sign")
	spine := s.CreateLine(local, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	// The reference end is pinned by two axis distances rather than by putting it
	// on the tip circle: a point on a circle has two answers, and the tip radius
	// touches it where the numbers go unstable.
	referenceEnd := s.CreatePoint(dim.Tip, 0)
	referenceEnd.SetName("plusXReferenceEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(local, referenceEnd, dim.Tip),
		sketch.NewVerticalDistance(local, referenceEnd, 0),
	)
	referenceLine := s.CreateLine(local, referenceEnd)
	referenceLine.SetConstruction(true)
	referenceLine.SetName("plusXReference")
	// Reference first, spine second, exactly the argument order Fusion's
	// addAngularDimension takes, so the measured angle runs from +X to the spine
	// and its sign is the hand of the helix.
	s.AddConstraint(sketch.NewAngle(referenceLine, spine, angle*180/math.Pi))

	proofkit.Step(t, "one rib per involute sample, chained outward along the spine")
	// The rib takes the axis ACROSS the spine and the chain the one ALONG it.
	// At a quarter turn the two swap, which is what the quarter_turn cases are
	// for: a scheme that fixes vertical for the rib solves at 14.5 degrees and
	// fails at 90.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previousMid := local
	previousX, previousY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPoints[i], rightPoints[i])
		rib.SetConstruction(true)
		rib.SetName(fmt.Sprintf("rib%d", i))
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
		mid.SetName(fmt.Sprintf("ribMid%d", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			// The last rib carries no perpendicular. The tooth-top arc already
			// holds those two flank tips at equal radius either side of the
			// spine, so adding one over-constrains the sketch.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		// The chain starts at the local origin, not at the first rib. Without
		// that first link the whole chain slides along the spine as a unit.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previousMid, mid, midX-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previousMid, mid, midY-previousY))
		}
		previousMid, previousX, previousY = mid, midX, midY
	}

	embedded := dim.Embedded()
	if !embedded {
		proofkit.Step(t, "flank-to-root stubs close the tooth on the root circle")
		stub := func(flankStart *sketch.Point, seed involute.Pt, name string) {
			norm := math.Hypot(seed.X, seed.Y)
			footX, footY := dim.Root*seed.X/norm, dim.Root*seed.Y/norm
			rootEnd := s.CreatePoint(footX, footY)
			rootEnd.SetName(name)
			line := s.CreateLine(rootEnd, flankStart)
			line.SetName(name + "Stub")
			// Two axis dimensions and nothing else. "On the root circle" plus
			// "origin on the line" would be satisfied by the far intersection
			// too, and the stub would become a line straight across the gear.
			s.AddConstraint(
				sketch.NewHorizontalDistance(local, rootEnd, footX),
				sketch.NewVerticalDistance(local, rootEnd, footY),
			)
		}
		stub(leftPoints[0], left[0], "leftRootEnd")
		stub(rightPoints[0], right[0], "rightRootEnd")
	} else {
		proofkit.Step(t, "embedded profile: the flanks start inside the root circle, so no stub is drawn")
	}

	proofkit.Step(t, "drag the whole drawing onto the projected anchor")
	s.AddConstraint(sketch.NewCoincident(local, anchor))

	checkTwistedProfile(t, s, dim, angle, embedded, local, anchor, toothTop, toothTopArc, rootCircle)
}

// checkTwistedProfile holds the solved sketch to what the spec pins about it:
// where the tooth points, that the tooth-top arc really is a piece of the tip
// circle, and the curve counts of the two loops the sketch closes.
func checkTwistedProfile(t testing.TB, s *sketch.Sketch, dim involute.Dimensions, angle float64,
	embedded bool, local, anchor, toothTop *sketch.Point, toothTopArc *sketch.Arc, rootCircle *sketch.Circle) {
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve: %v", err)
	}

	// The drawing landed on the user's anchor, so the whole gear follows it.
	if d := local.DistanceTo(anchor); d > 1e-9 {
		t.Errorf("local origin sits %.3e mm off the projected anchor", d)
	}

	// The tooth points at the helix angle, sign included. This is the assertion
	// a dropped or flipped angular dimension fails: a mirrored answer solves
	// just as well and lands the tooth at -angle.
	pointing := math.Atan2(toothTop.Y()-local.Y(), toothTop.X()-local.X())
	if delta := math.Abs(wrapPi(pointing - angle)); delta > 1e-6 {
		t.Errorf("tooth points at %.6f rad, helix angle is %.6f rad (off by %.3e)",
			pointing, angle, delta)
	}

	// The tooth-top arc caps the tooth ON the tip circle. Its centre must have
	// followed the drag onto the anchor, and its radius must be the tip radius.
	if d := toothTopArc.Center.DistanceTo(local); d > 1e-9 {
		t.Errorf("tooth-top arc centre stranded %.4f mm from the local origin", d)
	}
	if got := toothTopArc.R(); math.Abs(got-dim.Tip) > 1e-6 {
		t.Errorf("tooth-top arc radius %.6f mm, tip radius %.6f mm", got, dim.Tip)
	}

	// The two loops and their curve counts are a contract: the loft finds each
	// section by counting them, and spur's body extrude finds the disc the same
	// way. Counting them on the sketch actually drawn is the only way to know
	// the search will match.
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("sketch closes %d region(s), expected the tooth and the disc", len(profiles))
	}
	tooth, disc := -1, -1
	for i, profile := range profiles {
		for _, edge := range profile.Outer {
			if edge.Entity == toothTopArc {
				tooth = i
			}
		}
	}
	if tooth < 0 {
		t.Fatal("no closed region is bounded by the tooth-top arc")
	}
	disc = 1 - tooth

	// The count is taken over the DISTINCT entities the loop walks, not over its
	// boundary edges, and that is the one place the two engines have to be
	// reconciled. Fusion splits the root circle where the stubs meet it and the
	// tooth loop then walks one Arc3DCurveType piece of it. The sketch engine
	// leaves the piece attributed to the circle it came from, and at helix angle
	// zero, where the tooth sits on the circle's own parameter seam at +X, it
	// reports that one piece as two boundary edges of the same circle. Counting
	// entities gives the number Fusion's search would see at every angle;
	// counting edges gives three arcs at zero and two everywhere else.
	splines, lines, curved := 0, 0, 0
	for _, entity := range profiles[tooth].Entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			splines++
		case *sketch.Line:
			lines++
		case *sketch.Arc:
			curved++
		case *sketch.Circle:
			if entity != sketch.Entity(rootCircle) {
				t.Errorf("the tooth loop closes on a circle that is not the root circle")
			}
			curved++
		default:
			t.Errorf("unexpected %T on the tooth loop", entity)
		}
	}
	// The loop takes a piece of the root circle, never the whole of it: a whole
	// circle here would mean the tooth never met the root and the two regions
	// were never cut apart.
	for _, edge := range profiles[tooth].Outer {
		if _, isCircle := edge.Entity.(*sketch.Circle); isCircle && !edge.Partial {
			t.Errorf("the tooth loop walks the whole root circle, not a fragment of it")
		}
	}
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	if splines != 2 || curved != 2 || lines != wantLines {
		t.Errorf("tooth loop has nurbs=%d arcs=%d lines=%d, expected nurbs=2 arcs=2 lines=%d",
			splines, curved, lines, wantLines)
	}
	if embedded && lines == 0 {
		t.Logf("embedded: this loop has lines=0, and loftTooth searches both sections at " +
			"a fixed lines=2, so helical cannot consume it — see stepLoftTooth")
	}

	// The disc inside the root circle is what spur's inherited body extrude
	// consumes. Fusion sees it as exactly 2 arcs, the two pieces the stubs cut
	// the root circle into; the sketch engine reports the same region with the
	// circle uncut, as one closed edge.
	if n := len(profiles[disc].Outer); n != 1 {
		t.Errorf("the disc region walks %d edge(s), expected the root circle whole", n)
	}
	for _, edge := range profiles[disc].Outer {
		if edge.Entity != rootCircle {
			t.Errorf("the disc region is bounded by %T, expected the root circle", edge.Entity)
		}
	}
	if want := math.Pi * dim.Root * dim.Root; math.Abs(profiles[disc].Area-want) > 1e-6*want {
		t.Errorf("disc area %.4f mm2, root circle area %.4f mm2", profiles[disc].Area, want)
	}
}

func radians(degrees float64) float64 { return degrees * math.Pi / 180 }

// wrapPi folds an angle difference into (-pi, pi] so a comparison across the
// seam does not read as a full turn of error.
func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
