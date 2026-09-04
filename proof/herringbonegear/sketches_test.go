// Package herringbonegear_test proves the herringbone gear's build, one
// function per step of spec/herringbonegear/steps.md.
//
// Herringbone is a thin specialization of helical: it moves the twisted
// profile's plane to mid-body and builds its tooth as a lofted half that is
// mirrored and combined. This file holds the one sketch step — the Twisted Gear
// Profile drawn on the mid-body plane — because that is the only sketch
// herringbone's own delta touches. The solid steps are in solids_test.go.
//
// The constraint scheme drawn here is the spur tooth generator's, run at
// angle = HelixAngle, since herringbone passes the helix angle straight to
// SpurGearInvoluteToothDesignGenerator.draw(). It is reproduced rather than
// cited because the two curve counts the later steps select on — the six-curve
// tooth loop the loft's profile finder keys on, and the two-arc disc the
// inherited body extrude keys on — are properties of the sketch this step
// actually draws, and a citation proves neither.
package herringbonegear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// twistedCases sweeps the regime the inherited spur sketch scheme has to hold
// across, at the twist angles herringbone delivers to it.
//
// Every case carries the whole parameter set the step reads, so a case can be
// read on its own line: module, toothNumber, pressureAngle (radians),
// helixAngle (radians), involuteSteps and thickness (mm).
//
// The angle is swept on both signs because the confirming angular dimension
// carries the sign — a scheme that dropped or flipped it still solves at
// +angle and comes out mirrored at -angle — and at a quarter turn on both
// signs, where |sin| > |cos| swaps which axis the rib and the midpoint chain
// take. The rib count is swept at its low end as well as the standard 15,
// because one rib carries one across-spine dimension and one chain dimension,
// so a missing or redundant one is a large fraction of a short system. Both
// routes into the embedded shape are covered: a high tooth count at the
// ordinary 20 degrees, and a moderate tooth count at a large pressure angle.
//
// Not covered, deliberately: the exact base-radius == root-radius transition,
// where the flank-to-root stub has zero length. That configuration is
// ill-conditioned rather than wrong — the two axis dimensions that place a
// zero-length stub's root end are near-dependent — and the conditioning gate
// rejects it for that reason, so it is left out here and named as the edge of
// what this file checks.
var twistedCases = []proofkit.Case{
	{Name: "default_right_hand_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "left_hand_negative_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "quarter_turn_positive", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(90), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "quarter_turn_negative", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-90), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "coarse_module_few_teeth", Params: map[string]float64{
		"module": 4, "toothNumber": 12, "pressureAngle": deg(20),
		"helixAngle": deg(30), "involuteSteps": 15, "thickness": 40,
	}},
	{Name: "fine_module_many_teeth", Params: map[string]float64{
		"module": 0.5, "toothNumber": 40, "pressureAngle": deg(20),
		"helixAngle": deg(-30), "involuteSteps": 15, "thickness": 3,
	}},
	{Name: "low_rib_count_right_hand", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 4, "thickness": 10,
	}},
	{Name: "low_rib_count_left_hand", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 4, "thickness": 10,
	}},
	{Name: "embedded_by_tooth_count", Params: map[string]float64{
		"module": 1, "toothNumber": 60, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "embedded_by_pressure_angle", Params: map[string]float64{
		"module": 2, "toothNumber": 30, "pressureAngle": deg(25),
		"helixAngle": deg(-14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "thin_body_mid_plane", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15, "thickness": 1,
	}},
}

func deg(d float64) float64 { return d * math.Pi / 180 }

// stepTwistedGearProfileSketch draws the Twisted Gear Profile sketch on the
// mid-body plane and proves the scheme it is drawn with.
//
// Two things belong to herringbone here. The plane the sketch is created on is
// offset by half the thickness rather than the whole of it, which is the whole
// of the helicalPlaneOffset override, so the step creates that plane and checks
// where it lands. The tooth itself is drawn already rotated by the helix angle,
// and the confirming angular dimension is set last, exactly as the tooth
// generator does it.
//
// The two curve counts the later steps select on are asserted on the regions
// this sketch actually closes, not on a stand-in: the tooth loop the loft finds
// with nurbs=2, arcs=2, lines=2, and the disc inside the root circle the
// inherited body extrude finds with arcs=2.
func stepTwistedGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	module := p["module"]
	teeth := p["toothNumber"]
	pressure := p["pressureAngle"]
	angle := p["helixAngle"]
	steps := int(p["involuteSteps"])
	thickness := p["thickness"]

	proofkit.Step(t, "mid-body helix plane offset by thickness/2 = %.4f mm", thickness/2)
	world := s.World()
	helixPlane, err := world.CreateOffsetPlane(world.XY(), thickness/2)
	if err != nil {
		t.Fatalf("mid-body helix plane: %v", err)
	}
	frame, err := helixPlane.Frame()
	if err != nil {
		t.Fatalf("mid-body helix plane frame: %v", err)
	}
	if got, want := frame.Origin().Z, thickness/2; math.Abs(got-want) > 1e-9 {
		t.Errorf("helix plane sits at z=%.6f, want half the thickness %.6f", got, want)
	}

	d := involute.Derive(module, teeth, pressure)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	if len(left) != steps || len(right) != steps {
		t.Fatalf("flank sample counts %d/%d, want %d each", len(left), len(right), steps)
	}

	proofkit.Step(t, "local origin on the projected anchor")
	// The Tools-sketch projection of the user's anchor. It is reference
	// geometry: its coordinates are locked, and the sketch is grounded by
	// constraining the movable local origin onto it, which is what the tooth
	// generator's draw() does with addCoincident.
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "four gear circles on the local origin")
	rootCircle := s.CreateCircle(origin, d.Root)
	rootCircle.SetName("root circle")
	tipCircle := s.CreateCircle(origin, d.Tip)
	tipCircle.SetName("tip circle")
	tipCircle.SetConstruction(true)
	baseCircle := s.CreateCircle(origin, d.Base)
	baseCircle.SetName("base circle")
	baseCircle.SetConstruction(true)
	pitchCircle := s.CreateCircle(origin, d.Pitch)
	pitchCircle.SetName("pitch circle")
	pitchCircle.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(rootCircle, 2*d.Root),
		sketch.NewDiameter(tipCircle, 2*d.Tip),
		sketch.NewDiameter(baseCircle, 2*d.Base),
		sketch.NewDiameter(pitchCircle, 2*d.Pitch),
	)

	proofkit.Step(t, "involute flanks at angle %.4f rad", angle)
	leftFit := make([]*sketch.Point, len(left))
	rightFit := make([]*sketch.Point, len(right))
	for i := range left {
		leftFit[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightFit[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	leftFlank, err := s.CreateFitSpline(leftFit...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(rightFit...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("right flank")

	proofkit.Step(t, "tooth-top point and arc centred on the local origin")
	toothTop := s.CreatePoint(d.Tip*math.Cos(angle), d.Tip*math.Sin(angle))
	toothTop.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))
	last := len(left) - 1
	topArc := s.CreateArc(origin, rightFit[last], leftFit[last])
	topArc.SetName("tooth top arc")

	proofkit.Step(t, "spine, +X reference line and the confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetName("spine")
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	refEnd.SetName("+X reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	refLine := s.CreateLine(origin, refEnd)
	refLine.SetName("+X reference")
	refLine.SetConstruction(true)
	// The angular dimension exists for every angle, including 0: it is what
	// says which way the spine points. Its value is set as the very last
	// action in Fusion; here the dimension carries the value it is set to.
	spineAngle := sketch.NewAngle(refLine, spine, angle*180/math.Pi)
	s.AddConstraint(spineAngle)

	proofkit.Step(t, "one rib per fit-point index, with the midpoint chain along the spine")
	// The rib takes the axis across the spine and the chain the one along it.
	ribIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previous := origin
	previousX, previousY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftFit[i], rightFit[i])
		rib.SetConstruction(true)
		if ribIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftFit[i], rightFit[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftFit[i], rightFit[i], right[i].X-left[i].X))
		}
		// The midpoint is seeded at the foot of the left fit point on the
		// spine, never at the rib's own 2-D midpoint.
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			// The last rib carries no perpendicular: the tooth-top arc
			// already holds those two tips at equal radius either side of
			// the spine, and Fusion rejects the pair as over-constrained.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if ribIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, midX-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, midY-previousY))
		}
		previous, previousX, previousY = mid, midX, midY
	}

	embedded := d.Embedded()
	if !embedded {
		proofkit.Step(t, "flank-to-root lines, each placed by two axis dimensions")
		// The stub runs radially from the root circle up to the flank's own
		// start point, which it shares. Its root end is placed by exactly two
		// axis dimensions from the local origin, whose captured directions are
		// what say which side of the gear centre it sits on; the bench engine
		// takes those directions as the sign of the target.
		stubs := []struct {
			side       string
			flankStart *sketch.Point
		}{{"left", leftFit[0]}, {"right", rightFit[0]}}
		for _, stubEnd := range stubs {
			side, flankStart := stubEnd.side, stubEnd.flankStart
			seed := involute.Pt{X: flankStart.X(), Y: flankStart.Y()}
			radius := math.Hypot(seed.X, seed.Y)
			rootX, rootY := seed.X*d.Root/radius, seed.Y*d.Root/radius
			rootEnd := s.CreatePoint(rootX, rootY)
			rootEnd.SetName(side + " root end")
			stub := s.CreateLine(rootEnd, flankStart)
			stub.SetName(side + " flank-to-root line")
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, rootX),
				sketch.NewVerticalDistance(origin, rootEnd, rootY),
			)
		}
	}

	proofkit.Step(t, "confirm the requested rotation as the last action")
	// Drawn already rotated AND confirmed: the pre-rotation puts the geometry
	// on the correct solver branch, and this value-set locks it there.
	if angle != 0 {
		if err := spineAngle.SetValue(units.Radians(angle)); err != nil {
			t.Fatalf("confirming angular dimension: %v", err)
		}
	}

	proofkit.Step(t, "solve and read the two regions the sketch closes")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
	assertProfileCounts(t, s, d, embedded)
	assertFlankNarrowsToTip(t, leftFlank, rightFlank, angle)
}

// assertProfileCounts checks the two curve counts the later steps select on.
//
// The loft finds its section with nurbs=2, arcs=2, lines=2 and the inherited
// body extrude finds the disc with arcs=2, and neither search matches on
// anything else, so a sketch that closes different loops is a broken sketch
// rather than a later step's problem. Both counts are read off the regions this
// sketch actually detected.
//
// A fragment of the root circle counts as an arc: Fusion's tooth profile is
// bounded by the trimmed root circle, which it reports as an Arc3DCurveType
// curve, and the bench engine reports the same boundary as a partial circle
// entity.
//
// Substituted, and what it costs: the disc's boundary. Fusion reports the disc
// as the two arcs the tooth's two flank-to-root ends cut the root circle into,
// which is the arcs=2 key its extrude searches on. The bench engine detects the
// same region but renders its boundary as the one undivided circle entity, so
// the count itself cannot be read here. What is read instead is the fact the
// count comes from — that the circle IS cut, which shows up as the tooth loop
// bounding on a PARTIAL fragment of the root circle — together with the disc's
// area, which is the whole root circle's. A pair of stub ends that failed to
// reach the root circle would break both.
func assertProfileCounts(t testing.TB, s *sketch.Sketch, d involute.Dimensions, embedded bool) {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("sketch closes %d region(s), want 2 — the tooth and the disc inside the root circle", len(profiles))
	}

	wantLines := 2
	if embedded {
		// The flanks themselves cross the root circle, so no stub is drawn
		// and the loop is four curves. The loft's fixed nurbs=2, arcs=2,
		// lines=2 key does not match this shape: an embedded herringbone gear
		// has no lofted tooth, which is the inherited limitation this case
		// pins rather than papers over.
		wantLines = 0
	}

	var tooth, disc *sketch.Profile
	for _, profile := range profiles {
		nurbs, arcs, lines := curveCounts(profile.Outer)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth = profile
		case nurbs == 0 && arcs == 0 && lines == 0 && len(profile.Outer) == 1:
			disc = profile
		default:
			t.Errorf("region with nurbs=%d arcs=%d lines=%d matches neither the tooth nor the disc",
				nurbs, arcs, lines)
		}
	}
	if tooth == nil {
		t.Errorf("no region with nurbs=2 arcs=2 lines=%d — the loft's profile finder would raise", wantLines)
	}
	if disc == nil {
		t.Errorf("no disc region inside the root circle — the inherited body extrude's profile finder would raise")
	}
	if tooth == nil || disc == nil {
		return
	}
	if !tooth.Valid || !disc.Valid {
		t.Errorf("regions are not both extrudable: tooth valid=%v disc valid=%v", tooth.Valid, disc.Valid)
	}
	if want := math.Pi * d.Root * d.Root; math.Abs(disc.Area-want) > 1e-6*want {
		t.Errorf("disc area %.6f mm^2, want the whole root circle %.6f mm^2", disc.Area, want)
	}
	if tooth.Area <= 0 {
		t.Errorf("tooth region area %.6f mm^2, want a positive area", tooth.Area)
	}
	cut := false
	for _, edge := range tooth.Outer {
		if circle, ok := edge.Entity.(*sketch.Circle); ok && circle.Name() == "root circle" && edge.Partial {
			cut = true
		}
	}
	if !cut {
		t.Errorf("the tooth loop does not bound on a fragment of the root circle, " +
			"so the circle is not cut in two and Fusion's arcs=2 disc would not exist")
	}
}

// curveCounts classifies a boundary loop the way find_profile_by_curve_counts
// does: fitted splines are nurbs, arcs and circle fragments are arcs, and
// straight segments are lines.
func curveCounts(loop []sketch.BoundaryEdge) (nurbs, arcs, lines int) {
	for _, edge := range loop {
		switch edge.Entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc:
			arcs++
		case *sketch.Circle:
			if edge.Partial {
				arcs++
			}
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}

// assertFlankNarrowsToTip checks the tooth narrows from root to tip.
//
// The standard parametric involute spirals the other way, and using it
// unmirrored gives a tooth wider at the tip than at the root that still solves
// and still closes a six-curve loop, so the curve counts alone would not catch
// it. The width is measured across the spine direction, which is where the rib
// dimensions take it, so the test holds at any twist.
func assertFlankNarrowsToTip(t testing.TB, leftFlank, rightFlank *sketch.FitSpline, angle float64) {
	t.Helper()
	across := func(x, y float64) float64 { return -x*math.Sin(angle) + y*math.Cos(angle) }
	lx0, ly0 := leftFlank.Eval(0)
	rx0, ry0 := rightFlank.Eval(0)
	lx1, ly1 := leftFlank.Eval(1)
	rx1, ry1 := rightFlank.Eval(1)
	root := math.Abs(across(lx0, ly0) - across(rx0, ry0))
	tip := math.Abs(across(lx1, ly1) - across(rx1, ry1))
	if tip >= root {
		t.Errorf("tooth is %.6f mm wide at the root and %.6f mm at the tip; a tooth must narrow outward", root, tip)
	}
}
