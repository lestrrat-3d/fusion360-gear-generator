package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
)

// The spur Gear Profile bench, constraint by constraint.
//
// This is the sketch-first gate [PB-SKETCH-FIRST] asks for: the Gear Profile
// sketch of spec/spurgear/instructions.md steps 3-5, rebuilt in the sketch
// engine, solved, and held to the engine's own verdict before any Fusion code
// is written. The map below is the one a reader trusts, so a row that goes
// stale sends the next generation back to a recipe the spec rejected.
//
//   - four circles: root solid, tip/base/pitch construction, every centre the
//     SHARED local-origin point, each with a driving diameter dimension
//     ([SPUR-F-SHARED-ADJACENCY]).
//   - local origin: a fresh point, not the sketch's own origin, made
//     coincident with the projected anchor ([SPUR-F-LOCAL-ORIGIN]). That one
//     coincidence grounds the sketch, and it is what DRAGS every piece of
//     geometry onto the user's anchor as a unit.
//   - flanks: two fit splines through the involute samples, mirrored, rotated
//     so the pitch crossing lands at +pi/(2N), then rotated again by the
//     requested angle ([SPUR-F-ROTATE-CONFIRM]).
//   - tooth-top arc: created on a COPIED centre and tied back with a
//     coincident to the local origin, no diameter dimension
//     ([SPUR-F-TOOTHTOP-ARC]).
//   - spine + reference: a construction line from origin to the tooth-top
//     point, a +X reference line whose far end is pinned by two axis
//     dimensions, and one angular dimension from the reference to the spine
//     ([SPUR-F-SPINE]).
//   - ribs: one per fit-point index including both endpoints; across-spine
//     axis dimension, midpoint seeded on the spine, point-on-line, midpoint,
//     perpendicular (skipped on the last rib), and a chain dimension along the
//     spine starting at the local origin ([SPUR-F-RIBS]).
//   - flank-to-root lines: root endpoint pinned by signed axis dimensions from
//     the local origin — NewHorizontalDistance(origin, rootEnd, dx) and
//     NewVerticalDistance(origin, rootEnd, dy) — and by nothing else
//     ([SPUR-F-FLANK-ROOT]). Pinning it to the root circle and putting the
//     local origin on the stub instead also reaches DOF 0, and leaves the far
//     intersection of that line with the root circle equally valid, so the
//     stub becomes a line straight across the gear.
//
// Scope. Three things this bench does not reach:
//
//   - The exactly-tangent case, base radius == root radius, where the stub is
//     zero length. [SPUR-F-FLANK-ROOT] keeps the comparison strict there on
//     purpose, so the recipe draws a degenerate stub whose two axis dimensions
//     have no direction to capture. The system is genuinely ill-conditioned at
//     that point rather than wrong, so no case sits on it; the embedded cases
//     bracket it from one side and the standard ones from the other.
//   - Fusion's ORDER of operations. [SPUR-F-ROTATE-CONFIRM] requires the
//     angular dimension's value to be assigned as the very last action, after
//     the whole constraint network exists, because a value set earlier lets
//     the solver pick a branch 180 degrees away. The engine takes a
//     dimension's target at creation and solves once, so the bench proves the
//     network the rule produces and not the order it has to be built in. Only
//     a Fusion session tells the two apart.
//   - Sketch text. The four circle labels of step 3 carry their own position
//     along the curve and nothing pins it ([PB-TEXT-HOLDS-DOF]), so they are
//     not drawn here. Drawing them would report a sketch that is under-
//     constrained for a reason the geometry has nothing to do with.

// gearProfile is one drawn Gear Profile sketch and the handles the solid steps
// need back from it.
type gearProfile struct {
	s        *sketch.Sketch
	d        dims
	origin   *sketch.Point
	anchor   *sketch.Point
	root     *sketch.Circle
	tip      *sketch.Circle
	base     *sketch.Circle
	pitch    *sketch.Circle
	left     *sketch.FitSpline
	right    *sketch.FitSpline
	topArc   *sketch.Arc
	spine    *sketch.Line
	refLine  *sketch.Line
	embedded bool
}

// drawGearProfile reproduces SpurGearInvoluteToothDesignGenerator.draw: the
// four circles, the involute tooth, then the step-5 anchoring, in that order.
//
// Every seed coordinate is plane-local and origin-relative, exactly as the
// Fusion code computes them, and the anchoring at the end is what moves the
// whole drawing onto the anchor. Seeding at the final position instead would
// make the drag zero and hide the one defect a spur gear is least able to see
// ([SPUR-F-TOOTHTOP-ARC]).
func drawGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) *gearProfile {
	t.Helper()
	d := derive(p)
	mustSteps(t, d)
	g := &gearProfile{s: s, d: d, embedded: d.Embedded()}

	proofkit.Step(t, "project the Tools-sketch anchor into the Gear Profile sketch")
	// The anchor is reference geometry: a frozen snapshot of the Tools sketch's
	// own projection ([SPUR-F-ANCHOR-CHAIN]). The engine locks its coordinates
	// the way Fusion's projection tracks its source, so the sketch is grounded
	// through a constraint to it rather than by fixing a coordinate of its own.
	g.anchor = s.CreateReferencePoint(d.anchorX, d.anchorY, "tools-sketch anchor")

	proofkit.Step(t, "drawCircles: root solid, tip/base/pitch construction, all on the local origin")
	g.origin = s.CreatePoint(0, 0)
	g.origin.SetName("local origin")
	g.root = circleOn(s, g.origin, d.Root, false)
	g.tip = circleOn(s, g.origin, d.Tip, true)
	g.base = circleOn(s, g.origin, d.Base, true)
	g.pitch = circleOn(s, g.origin, d.Pitch, true)

	proofkit.Step(t, "drawTooth: flanks, tooth-top arc, spine, ribs, flank-to-root lines")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, d.toothNumber, d.steps, d.angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	var err error
	if g.left, err = s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if g.right, err = s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	// The tooth-top point sits on the tip circle at the requested angle, and
	// carries a point-on-circle and nothing else; the angular dimension below
	// says which way round the circle it is.
	topX, topY := involute.Rotate(d.Tip, 0, d.angle)
	top := s.CreatePoint(topX, topY)
	top.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(top, g.tip))

	// addByCenterStartEnd shares the start and end points and COPIES the
	// centre, so the centre is modelled as a fresh point seeded where the copy
	// lands — at the un-dragged local origin — and tied back with one
	// coincident ([PB-SHARE-XOR-COINCIDENT], [SPUR-F-TOOTHTOP-ARC] step 3).
	// The arc runs counter-clockwise from the right flank's end to the left
	// flank's end, which is the direction Fusion's call takes them in.
	arcCentre := s.CreatePoint(0, 0)
	arcCentre.SetName("tooth-top arc centre")
	g.topArc = s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	if p[pFreeArcCentre] == 0 {
		s.AddConstraint(sketch.NewCoincident(arcCentre, g.origin))
	}
	// No diameter dimension on the arc: the coincident centre and the two
	// shared flank ends already determine it, and a diameter over a free
	// centre would determine its size without saying which way it bulges.

	proofkit.Step(t, "spine, +X reference line and the confirming angular dimension")
	g.spine = s.CreateLine(g.origin, top)
	g.spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	refEnd.SetName("reference end")
	g.refLine = s.CreateLine(g.origin, refEnd)
	g.refLine.SetConstruction(true)
	// Two axis dimensions rather than a point-on-circle: a point on a circle
	// has two answers, and pinning x at the tip radius touches the circle at
	// its extreme where the numbers go unstable. The engine's targets are
	// SIGNED; Fusion's dimension value is a magnitude whose direction is
	// captured from the seed, so the sign crosses over as the seed side and
	// only abs() may be assigned there ([PB-DIM-VALUE-SEMANTICS]).
	s.AddConstraint(
		sketch.NewHorizontalDistance(g.origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(g.origin, refEnd, 0),
	)
	// From the reference to the spine, in that argument order, so the measured
	// angle is the counter-clockwise turn from +X and carries the sign of a
	// left-hand helix ([SPUR-F-SPINE] step 3, [SPUR-F-ROTATE-CONFIRM]).
	//
	// The value is set as a typed quantity rather than passed as a bare number:
	// a bare number is read in the sketch's default angle unit, which is
	// DEGREES, and the spec's angle is in radians. Passing 0.5236 for 30
	// degrees left the spine at half a degree, every rib chain dimension then
	// pulled the tooth in to a radius the tip circle does not have, and the
	// sketch still reported DOF 0 with no conflict.
	confirm := sketch.NewAngle(g.refLine, g.spine, 0)
	if err := confirm.SetValue(units.Radians(d.angle)); err != nil {
		t.Fatalf("confirming angular dimension: %v", err)
	}
	s.AddConstraint(confirm)

	proofkit.Step(t, "ribs: one per fit-point index, %d in all", len(leftPts))
	drawRibs(s, g, leftPts, rightPts, left, right)

	if !g.embedded {
		proofkit.Step(t, "flank-to-root lines: the flank starts outside the root circle")
		drawFlankToRoot(s, g, leftPts[0], left[0])
		drawFlankToRoot(s, g, rightPts[0], right[0])
	} else {
		proofkit.Step(t, "embedded profile: the flank starts inside the root circle, no stub drawn")
	}

	proofkit.Step(t, "step 5: anchor the local origin onto the projected anchor")
	s.AddConstraint(sketch.NewCoincident(g.origin, g.anchor))
	return g
}

// circleOn centres a circle on the shared local-origin point and gives it a
// driving diameter dimension. Passing the point itself is what makes all four
// circles share one centre; creating each from coordinates and re-coincidenting
// would pile redundant coincidents onto that point
// ([PB-SHARE-XOR-COINCIDENT], [PB-DRIVING-DIM]).
func circleOn(s *sketch.Sketch, origin *sketch.Point, radius float64, construction bool) *sketch.Circle {
	c := s.CreateCircle(origin, radius)
	c.SetConstruction(construction)
	s.AddConstraint(sketch.NewDiameter(c, 2*radius))
	return c
}

// drawRibs builds one rib per fit-point index in the exact order
// [SPUR-F-RIBS] gives. A different order over-constrains the sketch in Fusion,
// and the last rib carries no perpendicular because the tooth-top arc already
// holds the two flank tips at equal radius either side of the spine.
//
// Which axis each dimension takes is decided by the rotation: the rib takes the
// axis ACROSS the spine and the midpoint chain the one ALONG it, so vertical
// and horizontal respectively while |cos(angle)| >= |sin(angle)|, swapped
// otherwise. At angle 0 that reduces to the plain vertical rib and horizontal
// chain, and a tooth at 90 degrees fails without the swap. An aligned
// dimension would give only a length, which the two flanks satisfy equally
// well swapped over, so the tooth could come out mirrored.
func drawRibs(s *sketch.Sketch, g *gearProfile, leftPts, rightPts []*sketch.Point, left, right []involute.Pt) {
	acrossIsVertical := math.Abs(math.Cos(g.d.angle)) >= math.Abs(math.Sin(g.d.angle))
	previous := g.origin
	previousX, previousY := 0.0, 0.0
	for i := range leftPts {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}

		// The midpoint is created ALREADY ON the spine, at the foot of the left
		// fit point on it, never at the rib's true 2-D midpoint and never at
		// (fitX, 0) for a rotated tooth. The seed is not a constraint, but a
		// seed off the spine is how this solve fails to converge
		// ([PB-SEED-NEAR]).
		foot := left[i].X*math.Cos(g.d.angle) + left[i].Y*math.Sin(g.d.angle)
		midX, midY := involute.Rotate(foot, 0, g.d.angle)
		mid := s.CreatePoint(midX, midY)
		mid.SetConstruction(true)
		s.AddConstraint(sketch.NewPointOnLine(mid, g.spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(leftPts)-1 {
			s.AddConstraint(sketch.NewPerpendicular(g.spine, rib))
		}

		// The chain dimension runs outward along the spine, and starts at the
		// LOCAL ORIGIN rather than at the first rib: without that first link the
		// whole chain slides along the spine as a unit and the sketch never
		// fully constrains.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, midX-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, midY-previousY))
		}
		previous, previousX, previousY = mid, midX, midY
	}
}

// drawFlankToRoot draws one short radial stub from the root circle up to the
// flank's first fit point, and pins the root end with exactly the two axis
// dimensions [SPUR-F-FLANK-ROOT] names and no others.
//
// The stub SHARES the flank's start point rather than adding a coincident to
// it, and the root end is seeded at its exact computed position before the
// dimensions are created, so each captures its direction from that seed. The
// engine's targets are signed deltas; the Fusion transcription sets only
// abs(dx) / abs(dy) and realises the sign by seeding the point on the intended
// side, because a negative parameter.value flips the point to the other side
// of the origin ([PB-DIM-VALUE-SEMANTICS]).
func drawFlankToRoot(s *sketch.Sketch, g *gearProfile, flankStart *sketch.Point, seed involute.Pt) {
	origin := g.origin
	theta := math.Atan2(seed.Y, seed.X)
	rx := g.d.Root * math.Cos(theta)
	ry := g.d.Root * math.Sin(theta)
	re := s.CreatePoint(rx, ry)
	re.SetName("flank-to-root end")
	s.CreateLine(re, flankStart)
	s.AddConstraint(sketch.NewHorizontalDistance(origin, re, rx))
	s.AddConstraint(sketch.NewVerticalDistance(origin, re, ry))
}

// stepGearProfile is step 3: the whole Gear Profile sketch, which is one entry
// in the Fusion timeline however much geometry goes into it.
//
// Beyond the harness gate it verifies the one fact a later step SELECTS on:
// the sketch closes exactly two regions, and their curve counts are what
// find_profile_by_curve_counts matches. A count that drifts is a broken sketch
// rather than a later step's problem, so it is asserted here, on the sketch
// actually drawn.
func stepGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := drawGearProfile(t, s, p)
	if p[pFreeArcCentre] != 0 {
		// The negative control's whole point is that the sketch does not close.
		// Its verdict belongs to the harness, which holds it to the declared
		// status, DOF and sole reason.
		return
	}
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	assertProfileContract(t, g, report)
}

// assertProfileContract holds the sketch to the curve counts spec steps 7 and 9
// select on: the tooth loop is 2 splines + 2 arcs + 2 flank-to-root lines, or 2
// splines + 2 arcs when the profile is embedded, and the disc inside the root
// circle is bounded by the root circle and nothing else. Both loops exist only
// because the tooth meets the root circle and splits it in two.
//
// The counts are read off the regions the engine detected in the sketch that
// was actually drawn, never off a stand-in drawn for the purpose: the
// agreement between the drawing and the counts is the thing in question.
//
// They are counted per distinct boundary ENTITY rather than per boundary edge,
// because the two engines split a curve differently and Fusion's count is the
// per-entity one. Two places it shows:
//
//   - The root circle's own seam sits at +X, inside the tooth's root arc
//     whenever the tooth is drawn there, so the engine reports that one arc as
//     two fragments of the circle. Fusion has one arc.
//   - The disc's boundary walks the whole root circle, and the engine reports
//     it as the single closed curve rather than as the two pieces the stub feet
//     cut it into. Fusion reports those two pieces, which is why step 9 matches
//     on arcs=2. It is the same boundary either way, so what the bench can
//     prove about it is that the disc is bounded by the root circle ALONE and
//     that its area is the full pi*r^2 — an annulus or a region the tip circle
//     also bounded would fail both.
func assertProfileContract(t testing.TB, g *gearProfile, report *sketch.VerificationReport) {
	t.Helper()
	if len(report.Profiles) != 2 {
		t.Fatalf("gear profile closes %d region(s), want 2 (the tooth and the disc inside the root circle)",
			len(report.Profiles))
	}
	tooth, disc := splitRegions(t, g, report.Profiles)

	wantLines := 2
	if g.embedded {
		wantLines = 0
	}
	nurbs, arcs, lines, circles := curveCounts(tooth)
	if nurbs != 2 || arcs != 2 || lines != wantLines || circles != 0 {
		t.Errorf("tooth loop is %d nurbs + %d arcs + %d lines + %d whole circles, "+
			"want 2 + 2 + %d + 0 (embedded=%v)", nurbs, arcs, lines, circles, wantLines, g.embedded)
	}
	nurbs, arcs, lines, circles = curveCounts(disc)
	if nurbs != 0 || lines != 0 || arcs+circles != 1 {
		t.Errorf("gear body loop is %d nurbs + %d arcs + %d lines + %d whole circles, "+
			"want the root circle and nothing else", nurbs, arcs, lines, circles)
	}
	if len(disc.Entities) != 1 || disc.Entities[0] != g.root {
		t.Errorf("gear body loop is bounded by %d entities, want only the root circle",
			len(disc.Entities))
	}

	sketchtest.IsValidProfile(t, tooth)
	sketchtest.IsValidProfile(t, disc)
	sketchtest.IsCurrentProfile(t, tooth)
	sketchtest.IsCurrentProfile(t, disc)
	// The disc is the full circle inside the root radius, not an annulus: the
	// tip circle is construction geometry and bounds no profile. Its area is
	// therefore pi*r^2 exactly, and the slack is the rounding of that product.
	sketchtest.MeasuresProfileArea(t, disc, math.Pi*g.d.Root*g.d.Root, sketchtest.WithinRel(1e-9))

	// The tooth sits outside the root circle in the ordinary case, so the two
	// regions meet along the short root arc and nowhere else. A tooth whose
	// area exceeded the disc's would mean the loops overlap.
	if tooth.Area <= 0 || tooth.Area >= disc.Area {
		t.Errorf("tooth area %.6f mm^2 is not a proper tooth beside a %.6f mm^2 disc",
			tooth.Area, disc.Area)
	}

	// The tooth-top arc is what [SPUR-F-TOOTHTOP-ARC] is about: with its centre
	// stranded the radius becomes whatever the solver lands on, and the drag is
	// the distance it is stranded by. Read it on the solved geometry.
	sketchtest.Measures(t, "tooth-top arc radius", g.topArc.R(), g.d.Tip, sketchtest.WithinRel(1e-9))
	sketchtest.MeasuresPoint(t, g.topArc.Center, g.d.anchorX, g.d.anchorY, sketchtest.Within(1e-9))
}

// splitRegions tells the tooth from the disc by the tooth-top arc, which only
// the tooth loop can carry.
func splitRegions(t testing.TB, g *gearProfile, profiles []*sketch.Profile) (tooth, disc *sketch.Profile) {
	t.Helper()
	for _, profile := range profiles {
		if usesEntity(profile, g.topArc) {
			tooth = profile
		} else {
			disc = profile
		}
	}
	if tooth == nil || disc == nil {
		t.Fatalf("could not tell the tooth region from the disc: tooth=%v disc=%v", tooth != nil, disc != nil)
	}
	return tooth, disc
}

func usesEntity(profile *sketch.Profile, want sketch.Entity) bool {
	for _, e := range profile.Entities {
		if e == want {
			return true
		}
	}
	return false
}

// curveCounts maps a region's distinct outer-boundary entities onto the curve
// kinds find_profile_by_curve_counts matches on. A circle the boundary uses
// only part of is an arc, which is what Fusion reports for the piece the tooth
// cuts out of it; one the boundary uses whole is counted apart so it cannot
// pass for an arc.
func curveCounts(profile *sketch.Profile) (nurbs, arcs, lines, circles int) {
	partial := map[sketch.Entity]bool{}
	for _, edge := range profile.Outer {
		if edge.Partial {
			partial[edge.Entity] = true
		}
	}
	for _, entity := range profile.Entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			lines++
		case *sketch.Circle:
			if partial[entity] {
				arcs++
			} else {
				circles++
			}
		}
	}
	return nurbs, arcs, lines, circles
}

// profileFailures is the negative control [SPUR-F-TOOTHTOP-ARC] requires the
// bench to carry, and it must keep failing.
//
// Everything is drawn exactly as above except the one coincident that ties the
// tooth-top arc's copied centre back to the local origin. The centre is then a
// free point carrying only the arc's own equal-radius relation to the two
// flank ends, which is one equation against its two coordinates, so the sketch
// reports two remaining degrees of freedom and reads underconstrained.
//
// In Fusion the consequence is worse than a DOF count: the stranded centre does
// not follow when step 5 drags the sketch onto the anchor, so it stays behind
// by the drag distance and the arc's radius becomes whatever the solver lands
// on. Measured 2026-09-02 on a default 31/31 bevel pair, a 0.5743 mm radius on
// the pinion and 17.0204 mm on the driving gear where both should have been
// 22.5 mm, from two sketches with byte-identical constraint counts and
// dimension values. This control checks the unconstrained state, which is what
// the bench can see, not those historical Fusion measurements.
var profileFailures = []proofkit.ExpectedFailureCase{
	{
		Case: proofkit.Case{
			Name:   "tooth-top-arc-centre-left-free",
			Params: params(map[string]float64{pFreeArcCentre: 1}),
		},
		Expected: proofkit.ExpectedFailure{
			Status: sketch.Underconstrained,
			DOF:    2,
			Reason: sketch.ErrNotFullyConstrained,
		},
	},
}

// stepBoreProfile is step 12's Bore Profile sketch: a second sketch on the
// target plane holding one circle of the bore diameter, centred on the anchor
// projected into it, with a driving diameter dimension.
//
// It also carries the accepted side effect the spec pins: the tooth
// generator's constructor always adds its local-origin (0, 0) point, so this
// sketch holds one stray unused point. It is grounded on the SAME projected
// anchor the circle is centred on, never on the sketch's own origin point —
// which would pin it to the plane rather than to the gear, and which
// [PB-CIRCLE-CENTER] records a solver failure for. Without any grounding the
// point is free in two directions and the sketch never fully constrains.
func stepBoreProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	bore := p[pBoreDiameter]
	if bore <= 0 {
		proofkit.Unmodelled(t, "step 12 returns before drawing anything at a bore diameter of %g", bore)
	}

	proofkit.Step(t, "project the Tools-sketch anchor into the Bore Profile sketch")
	anchor := s.CreateReferencePoint(d.anchorX, d.anchorY, "tools-sketch anchor")

	proofkit.Step(t, "drawBore: the bore circle centred on the projection, driving diameter dimension")
	circle := s.CreateCircle(anchor, bore/2)
	s.AddConstraint(sketch.NewDiameter(circle, bore))

	proofkit.Step(t, "ground the tooth generator's stray local origin on the same projection")
	origin := s.CreatePoint(0, 0)
	origin.SetName("stray local origin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.IsCurrentProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*bore*bore/4, sketchtest.WithinRel(1e-9))
	sketchtest.MeasuresPoint(t, circle.Center, d.anchorX, d.anchorY, sketchtest.Within(1e-9))
}
