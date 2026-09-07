package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// profileCases is the regime the Gear Profile scheme has to hold across
// (spec, Sketch Discipline): several sizes, the whole signed range of the
// draw angle including a quarter turn and the bevel's half turn, the low end
// of the involute-step count, both routes into the embedded shape, and an
// anchor that does not sit at the sketch origin so the step-5 drag is real.
var profileCases = []proofkit.Case{
	{Name: "M1_N17_PA20_a0", Params: params(1, 17, 20, 0, 15, 0, 0)},
	{Name: "M1_N17_PA20_a0_anchor_offset", Params: params(1, 17, 20, 0, 15, 12.5, -7)},
	{Name: "M2_N12_PA20_a0", Params: params(2, 12, 20, 0, 15, 0, 0)},
	{Name: "M3_N8_PA20_a0", Params: params(3, 8, 20, 0, 15, 0, 0)},
	{Name: "M0.5_N30_PA20_a0", Params: params(0.5, 30, 20, 0, 15, 0, 0)},
	{Name: "M1_N17_PA14.5_a0", Params: params(1, 17, 14.5, 0, 15, 0, 0)},
	{Name: "M1_N17_PA20_a+25", Params: params(1, 17, 20, 25, 15, 0, 0)},
	{Name: "M1_N17_PA20_a-25", Params: params(1, 17, 20, -25, 15, 0, 0)},
	{Name: "M1_N17_PA20_a+25_anchor_offset", Params: params(1, 17, 20, 25, 15, -9, 4)},
	{Name: "M1_N17_PA20_a+90", Params: params(1, 17, 20, 90, 15, 0, 0)},
	{Name: "M1_N17_PA20_a-120", Params: params(1, 17, 20, -120, 15, 0, 0)},
	{Name: "M1_N17_PA20_a180", Params: params(1, 17, 20, 180, 15, 0, 0)},
	{Name: "M1_N17_PA20_a0_steps5", Params: params(1, 17, 20, 0, 5, 0, 0)},
	{Name: "M1_N17_PA20_a-40_steps3", Params: params(1, 17, 20, -40, 3, 0, 0)},
	// Embedded by tooth count: 45 > 41.5 at 20 degrees.
	{Name: "M1_N45_PA20_a0_embedded", Params: params(1, 45, 20, 0, 15, 0, 0)},
	{Name: "M1_N45_PA20_a-30_steps6_embedded", Params: params(1, 45, 20, -30, 6, 0, 0)},
	// Embedded by pressure angle: 30 > 26.7 at 25 degrees, while 30 < 41.5 at 20.
	{Name: "M1_N30_PA25_a0_embedded", Params: params(1, 30, 25, 0, 15, 0, 0)},
	{Name: "M1_N80_PA14.5_a0_embedded", Params: params(1, 80, 14.5, 0, 15, 0, 0)},
	// What a default 31/31 bevel pair draws through the borrowed generator.
	{Name: "M1_N43_PA20_a180_embedded", Params: params(1, 43, 20, 180, 15, 0, 0)},
	{Name: "M1_N43_PA20_a180_anchor_offset_embedded", Params: params(1, 43, 20, 180, 15, 20, 30)},
}

// params builds a sketch case. The solid-only keys are left at zero.
func params(module, teeth, pressureAngleDeg, angleDeg, steps, anchorX, anchorY float64) map[string]float64 {
	return map[string]float64{
		pModule:        module,
		pToothNumber:   teeth,
		pPressureAngle: pressureAngleDeg,
		pAngle:         angleDeg,
		pInvoluteSteps: steps,
		pAnchorX:       anchorX,
		pAnchorY:       anchorY,
	}
}

// stepToolsSketch is step 2's Tools sketch. It draws no geometry of its own:
// it projects the user's Anchor Point in and keeps that projection as the
// handle every later sketch re-projects from. The projection is reference
// geometry, locked to its source, which is all the sketch holds.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := derive(t, p)
	proofkit.Step(t, "project the Anchor Point into the Tools sketch")
	anchor := s.CreateReferencePoint(g.anchorX, g.anchorY, "anchorPoint")
	anchor.SetName("ctx.anchorPoint")
}

// gearProfile holds the handles the Gear Profile build leaves behind, so the
// solid steps and the assertions can reach the entities by role.
type gearProfile struct {
	origin                 *sketch.Point
	root, tip, base, pitch *sketch.Circle
	left, right            *sketch.FitSpline
	toothTop               *sketch.Point
	toothTopArc            *sketch.Arc
	spine, reference       *sketch.Line
	angleDim               *sketch.Angle
	ribs                   []*sketch.Line
	leftStub, rightStub    *sketch.Line
	embedded               bool
}

// stepGearProfile is step 3 (with the spec's 3, 4 and 5 inside it): the
// Gear Profile sketch, drawn by the tooth generator's draw(anchorPoint,
// angle). It asserts the two regions the sketch has to close and the curve
// counts the extrude steps match on, then hands the sketch to the harness
// gate, which requires DOF 0 with no redundant or conflicting constraint and
// no second configuration.
func stepGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := derive(t, p)
	gp := drawGearProfile(t, s, g)

	proofkit.Step(t, "solve and read the regions the sketch closes")
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e DOF %d", res.Residual, res.DOF)
	}
	if gp.embedded != g.Embedded() {
		t.Fatalf("embedded flag from the drawn flank start is %v, from the radii %v", gp.embedded, g.Embedded())
	}
	assertGearProfileRegions(t, s, g, gp)
}

// drawGearProfile draws the Gear Profile sketch the way the tooth generator's
// draw(anchorPoint, angle) does, in the same order: drawCircles, drawTooth,
// the step-5 anchoring, then the confirming angular dimension.
//
// Fusion to engine: addByCenterRadius(localOrigin, r) is CreateCircle(origin,
// r) with a NewDiameter; a fitted spline through SketchPoints is a FitSpline
// through the same points; addByCenterStartEnd(centre, start, end) shares the
// two ends and copies the centre, so the arc gets a fresh centre point that
// the explicit coincident then ties to the origin; addDistanceDimension with
// an axis orientation is NewHorizontalDistance / NewVerticalDistance, whose
// signed target carries the side Fusion captures from the seed; the projected
// anchor is a reference point and the step-5 addCoincident is a NewCoincident
// onto it. Along-path circle labels have no engine counterpart and are not
// drawn; they carry no constraint, so the scheme is unaffected.
func drawGearProfile(t testing.TB, s *sketch.Sketch, g gear) *gearProfile {
	t.Helper()
	gp := &gearProfile{}

	// The tooth generator's constructor: a fresh movable local origin at
	// (0, 0, 0), not the sketch's own origin point [SPUR-F-LOCAL-ORIGIN].
	gp.origin = s.CreatePoint(0, 0)
	gp.origin.SetName("localOrigin")

	proofkit.Step(t, "drawCircles: root (solid), tip, base, pitch (construction), all centred on the local origin")
	circle := func(name string, r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(gp.origin, r)
		c.SetName(name)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	gp.root = circle("Root Circle", g.Root, false)
	gp.tip = circle("Tip Circle", g.Tip, true)
	gp.base = circle("Base Circle", g.Base, true)
	gp.pitch = circle("Pitch Circle", g.Pitch, true)

	proofkit.Step(t, "drawTooth: sample the involute flanks and fit splines through them")
	left, right := g.flanks(g.angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "fewer than two involute samples survive, so no flank spline can be fitted")
	}
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].x, left[i].y)
		rightPts[i] = s.CreatePoint(right[i].x, right[i].y)
	}
	var err error
	gp.left, err = s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank: %v", err)
	}
	gp.right, err = s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank: %v", err)
	}
	gp.left.SetName("left flank")
	gp.right.SetName("right flank")
	last := len(left) - 1

	proofkit.Step(t, "tooth-top point on the tip circle, tooth-top arc centred on the local origin [SPUR-F-TOOTHTOP-ARC]")
	gp.toothTop = s.CreatePoint(g.Tip*math.Cos(g.angle), g.Tip*math.Sin(g.angle))
	gp.toothTop.SetName("toothTop")
	s.AddConstraint(sketch.NewPointOnCircle(gp.toothTop, gp.tip))
	arcCentre := s.CreatePoint(0, 0)
	arcCentre.SetName("toothTopArc.centre (copied, not shared)")
	gp.toothTopArc = s.CreateArc(arcCentre, rightPts[last], leftPts[last])
	gp.toothTopArc.SetName("tooth-top arc")
	s.AddConstraint(sketch.NewCoincident(arcCentre, gp.origin))

	proofkit.Step(t, "spine, +X reference and the angular pin [SPUR-F-SPINE]")
	gp.spine = s.CreateLine(gp.origin, gp.toothTop)
	gp.spine.SetName("spine")
	gp.spine.SetConstruction(true)
	refEnd := s.CreatePoint(g.Tip, 0)
	refEnd.SetName("reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(gp.origin, refEnd, g.Tip),
		sketch.NewVerticalDistance(gp.origin, refEnd, 0),
	)
	gp.reference = s.CreateLine(gp.origin, refEnd)
	gp.reference.SetName("+X reference")
	gp.reference.SetConstruction(true)
	// Created at the angle the geometry already sits at, reference first,
	// spine second. The value is confirmed again as the very last action.
	gp.angleDim = sketch.NewAngle(gp.reference, gp.spine, g.angle*180/math.Pi)
	s.AddConstraint(gp.angleDim)

	proofkit.Step(t, "ribs, one per fit-point index, in the six-step order [SPUR-F-RIBS]")
	across := math.Abs(math.Cos(g.angle)) >= math.Abs(math.Sin(g.angle))
	prev := gp.origin
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		gp.ribs = append(gp.ribs, rib)
		// 2. axis dimension across the spine, direction from the seed.
		if across {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].y-left[i].y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].x-left[i].x))
		}
		// 3. midpoint seeded at the foot of the left fit point on the spine.
		tt := left[i].x*math.Cos(g.angle) + left[i].y*math.Sin(g.angle)
		mid := s.CreatePoint(tt*math.Cos(g.angle), tt*math.Sin(g.angle))
		// 4. onto the spine first, 5. then the rib's midpoint, 6. then
		// perpendicular, except on the last rib.
		s.AddConstraint(sketch.NewPointOnLine(mid, gp.spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(gp.spine, rib))
		}
		// Chain dimension along the spine from the previous midpoint, the
		// local origin for the first rib.
		if across {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, mid.X()-prev.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, mid.Y()-prev.Y()))
		}
		prev = mid
	}

	proofkit.Step(t, "flank-to-root lines unless the flank starts inside the root circle [SPUR-F-FLANK-ROOT]")
	firstRadius := math.Hypot(left[0].x, left[0].y)
	gp.embedded = firstRadius < g.Root
	if !gp.embedded {
		stub := func(start pt, flankStart *sketch.Point, name string) *sketch.Line {
			re := g.rootEnd(start)
			rootEnd := s.CreatePoint(re.x, re.y)
			rootEnd.SetName(name + " root end")
			l := s.CreateLine(rootEnd, flankStart)
			l.SetName(name)
			s.AddConstraint(
				sketch.NewHorizontalDistance(gp.origin, rootEnd, re.x),
				sketch.NewVerticalDistance(gp.origin, rootEnd, re.y),
			)
			return l
		}
		gp.leftStub = stub(left[0], leftPts[0], "left flank-to-root")
		gp.rightStub = stub(right[0], rightPts[0], "right flank-to-root")
	}

	proofkit.Step(t, "step 5: project the Tools anchor in and drag the local origin onto it [SPUR-F-ANCHOR-CHAIN]")
	anchor := s.CreateReferencePoint(g.anchorX, g.anchorY, "Tools.anchorPoint")
	anchor.SetName("projected anchor")
	s.AddConstraint(sketch.NewCoincident(gp.origin, anchor))

	if g.angle != 0 {
		proofkit.Step(t, "very last action: confirm the angular dimension at angle [SPUR-F-ROTATE-CONFIRM]")
		if err := gp.angleDim.SetValue(units.Radians(g.angle)); err != nil {
			t.Fatalf("confirm angle: %v", err)
		}
	}
	return gp
}

// curveCounts is a loop's boundary by curve kind, the key the extrude steps
// match on through find_profile_by_curve_counts(sketch, nurbs, arcs, lines).
type curveCounts struct{ nurbs, arcs, lines int }

// countCurves classifies the distinct entities on a region's outer boundary.
// A fragment of the solid root circle counts as an arc, which is what Fusion
// reports for a circle split by the tooth (Arc3DCurveType).
func countCurves(t testing.TB, prof *sketch.Profile) curveCounts {
	t.Helper()
	var c curveCounts
	for _, e := range prof.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			c.nurbs++
		case *sketch.Arc, *sketch.Circle:
			c.arcs++
		case *sketch.Line:
			c.lines++
		default:
			t.Fatalf("unexpected %T on a profile boundary", e)
		}
	}
	return c
}

// assertGearProfileRegions checks the contract the spec puts on the sketch:
// exactly two regions, the tooth bounded by 2 splines + 2 arcs + 2 lines (or
// no lines when embedded), and the disc inside the root circle bounded by the
// root circle alone. Fusion reports that disc as 2 arcs, the two pieces the
// tooth splits the root circle into; the engine walks the same split circle
// but reports it as one whole edge of the one entity, so the count it can
// pin is that no other entity bounds the disc, and that the tooth's own root
// edge is a fragment of that circle rather than a curve drawn for it.
func assertGearProfileRegions(t testing.TB, s *sketch.Sketch, g gear, gp *gearProfile) {
	t.Helper()
	profs := s.Profiles()
	if len(profs) != 2 {
		t.Fatalf("Gear Profile closes %d regions, want exactly 2 (tooth and disc)", len(profs))
	}
	var tooth, disc *sketch.Profile
	for _, p := range profs {
		if !p.Valid {
			t.Fatalf("region with %d boundary entities is not a valid profile", len(p.Entities))
		}
		if len(p.Entities) == 1 {
			disc = p
		} else {
			tooth = p
		}
	}
	if tooth == nil || disc == nil {
		t.Fatalf("could not tell the tooth from the disc: %d and %d boundary entities", len(profs[0].Entities), len(profs[1].Entities))
	}

	want := curveCounts{nurbs: 2, arcs: 2, lines: 2}
	if gp.embedded {
		want.lines = 0
	}
	if got := countCurves(t, tooth); got != want {
		t.Fatalf("tooth loop curve counts %+v, want %+v (embedded=%v)", got, want, gp.embedded)
	}
	rootFragment := false
	for _, e := range tooth.Outer {
		if e.Entity == gp.root {
			rootFragment = true
			if !e.Partial {
				t.Fatalf("the tooth's root edge is the whole root circle, not the piece the tooth cuts from it")
			}
		}
	}
	if !rootFragment {
		t.Fatalf("the tooth loop does not run along the root circle")
	}
	if len(tooth.Entities) == 0 || tooth.Area <= 0 {
		t.Fatalf("tooth region has no area")
	}

	if disc.Entities[0] != gp.root {
		t.Fatalf("disc is bounded by %T, want the root circle", disc.Entities[0])
	}
	if wantArea := math.Pi * g.Root * g.Root; !near(disc.Area, wantArea, 1e-6) {
		t.Fatalf("disc area %.6f, want the root circle's %.6f", disc.Area, wantArea)
	}

	// The whole drawing followed the anchor: the local origin and every
	// circle centre sit on the projected anchor after the solve.
	if !near(gp.origin.X(), g.anchorX, 1e-9) || !near(gp.origin.Y(), g.anchorY, 1e-9) {
		t.Fatalf("local origin solved to (%.6f, %.6f), want the anchor (%.6f, %.6f)", gp.origin.X(), gp.origin.Y(), g.anchorX, g.anchorY)
	}
	if r := gp.toothTopArc.R(); !near(r, g.Tip, 1e-6) {
		t.Fatalf("tooth-top arc radius %.6f, want the tip radius %.6f", r, g.Tip)
	}
	if c := gp.toothTopArc.Center; !near(c.X(), g.anchorX, 1e-9) || !near(c.Y(), g.anchorY, 1e-9) {
		t.Fatalf("tooth-top arc centre stayed at (%.6f, %.6f) instead of following the anchor", c.X(), c.Y())
	}
	// The spine points along angle, not its supplement.
	dx, dy := gp.toothTop.X()-gp.origin.X(), gp.toothTop.Y()-gp.origin.Y()
	if got := math.Atan2(dy, dx); math.Abs(math.Remainder(got-g.angle, 2*math.Pi)) > 1e-9 {
		t.Fatalf("spine points at %.6f rad, want %.6f", got, g.angle)
	}
}
