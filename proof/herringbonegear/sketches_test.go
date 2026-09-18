// Package herringbonegear_test proves the herringbone gear's own two deltas
// over helical, against the step list compiled from
// spec/herringbonegear/instructions.md and spec/herringbonegear/fusion.md.
//
// Herringbone is a thin specialization of helical, which is itself a thin
// specialization of spur. It adds no dialog input, no user parameter and no
// sketch of its own, and changes exactly two things: helicalPlaneOffset returns
// half the thickness instead of the full thickness, so the twisted profile's
// plane lands at mid-body, and buildTooth lofts one half, mirrors it across
// that mid-body plane and combines the two halves into one tooth body. This
// proof covers those and nothing else. The bottom Gear Profile sketch, the body
// extrude, the pattern, the fillets, the bore and the completed-gear chamfer
// are spur's and are proven there; the loft recipe itself is helical's and is
// proven there.
//
// The tooth math is imported from proof/involute rather than restated, because
// herringbone draws the same tooth spur draws, pre-rotated by the helix angle.
package herringbonegear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// profileCases sweeps the mid-body twisted profile across the regime the spur
// spec states the constraint scheme must hold over, read through herringbone's
// own inputs.
//
// Size, because the rib chain's dimensions scale with the tooth and the
// conditioning of the system does not. The whole SIGNED range of the helix
// angle, because the sign is the hand of the helix and a scheme that drops or
// flips the confirming angular dimension still solves at +angle and comes out
// mirrored at -angle. A quarter turn, where |sin| > |cos| swaps which axis the
// rib and the chain dimensions take. A low rib count, where one missing or
// redundant dimension is a large fraction of the system. And both routes into
// the embedded shape — a high tooth count at the ordinary pressure angle, and a
// moderate tooth count at a large one — which herringbone inherits no support
// for and which this proof measures rather than assumes.
//
// Thickness does not enter the sketch: the twist between the two loft sections
// IS the helix angle, and the plane the section sits on is the solid proof's
// subject. The thickness in each case is carried anyway so one params helper
// serves both tables.
var profileCases = []proofkit.Case{
	{Name: "default_M1_N17_helix14.5", Params: params(1, 17, 20, 14.5, 15, 10)},
	{Name: "M1_N12_helix14.5", Params: params(1, 12, 20, 14.5, 15, 10)},
	{Name: "coarse_M3_N15_helix14.5", Params: params(3, 15, 20, 14.5, 15, 10)},
	{Name: "fine_M0.5_N24_helix14.5", Params: params(0.5, 24, 20, 14.5, 15, 10)},
	{Name: "large_M2_N20_helix14.5", Params: params(2, 20, 20, 14.5, 15, 10)},

	{Name: "helix0_spur_baseline", Params: params(1, 17, 20, 0, 15, 10)},
	{Name: "helix_plus10", Params: params(1, 17, 20, 10, 15, 10)},
	{Name: "helix_minus14.5_left_hand", Params: params(1, 17, 20, -14.5, 15, 10)},
	{Name: "helix_plus35", Params: params(1, 17, 20, 35, 15, 10)},
	{Name: "helix_minus35_left_hand", Params: params(1, 17, 20, -35, 15, 10)},
	{Name: "helix_plus90_quarter_turn", Params: params(1, 17, 20, 90, 15, 10)},
	{Name: "helix_minus90_quarter_turn", Params: params(1, 17, 20, -90, 15, 10)},

	{Name: "ribs_low_count_5_helix14.5", Params: params(1, 17, 20, 14.5, 5, 10)},
	{Name: "ribs_low_count_3_helix_minus25", Params: params(1, 17, 20, -25, 3, 10)},

	{Name: "embedded_by_tooth_count_N60_PA20", Params: params(1, 60, 20, 14.5, 15, 10)},
	{Name: "embedded_by_pressure_angle_N30_PA30", Params: params(1, 30, 30, 14.5, 15, 10)},
}

// params builds one case's parameter set, for both tables in this package.
//
// Angles arrive in degrees, the unit the dialog uses, and are held in radians,
// the unit the HelixAngle and PressureAngle user parameters are registered in.
// Lengths are millimetres throughout this proof; the generated module works in
// Fusion's internal centimetres, and no step here depends on the scale.
func params(module, toothNumber, pressureAngleDeg, helixAngleDeg float64, steps int, thickness float64) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": pressureAngleDeg * math.Pi / 180,
		"helixAngle":    helixAngleDeg * math.Pi / 180,
		"involuteSteps": float64(steps),
		"thickness":     thickness,
	}
}

// dimensionsOf derives the four circle radii one case is built from.
func dimensionsOf(p map[string]float64) involute.Dimensions {
	return involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
}

// stepMidBodyTwistedProfile draws the Twisted Gear Profile sketch — the spur
// tooth generator run at angle = HelixAngle — which herringbone's inherited
// buildSketches puts on the mid-body plane.
//
// Herringbone writes none of this construction: every constraint here is spur's
// ([SPUR-F-...]) and the non-zero angle is helical's. What makes the section
// herringbone's is that it is drawn ONCE and consumed TWICE — it is the chevron
// apex, the section both lofted halves end on and the plane the mirror reflects
// across — so the two facts asserted below are the ones both halves depend on:
// the section closes the six-curve tooth loop the inherited loftTooth searches
// for with a fixed nurbs=2, arcs=2, lines=2, and its tooth-top point sits at
// exactly the helix angle, which is the twist the chevron gets its apex from.
//
// The sketch is proven on the world XY plane because the constraint scheme is
// plane-local. The mid-body plane's own offset is a length, and it is proven in
// the solid proof by stepMidBodyPlane, where an offset can be measured.
func stepMidBodyTwistedProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	toothNumber := p["toothNumber"]
	angle := p["helixAngle"]
	steps := int(p["involuteSteps"])
	dims := dimensionsOf(p)

	proofkit.Step(t, "mid-body twisted profile: module=%g teeth=%g pressureAngle=%.4frad helix=%.4frad steps=%d embedded=%v",
		p["module"], toothNumber, p["pressureAngle"], angle, steps, dims.Embedded())

	// [SPUR-F-ANCHOR-CHAIN] / [SPUR-F-LOCAL-ORIGIN]. The Tools-sketch anchor is
	// projected in, which the engine models as a reference point: its
	// coordinates are locked by the projection, exactly as Fusion's projected
	// point tracks its source. The sketch's own movable local origin is a fresh
	// point, and the step-5 anchoring is the coincidence between the two.
	projectedAnchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	localOrigin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(localOrigin, projectedAnchor))

	// drawCircles. The root circle is solid geometry; the other three are
	// construction, so only the root circle bounds a profile. Every circle is
	// centred by SHARING the local origin ([PB-SHARE-XOR-COINCIDENT]) and
	// carries a driving diameter dimension ([PB-DRIVING-DIM]).
	circle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(localOrigin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(dims.Root, false)
	tipCircle := circle(dims.Tip, true)
	circle(dims.Base, true)
	circle(dims.Pitch, true)

	// drawTooth(angle). The flanks are drawn already rotated by the helix angle
	// ([SPUR-F-ROTATE-CONFIRM]'s draw half); nothing is drawn flat and swung
	// into place afterwards.
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, toothNumber, steps, angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "only %d involute samples survive, which is not a flank", len(left))
	}
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	proofkit.Step(t, "spine, +X reference and the angular pin that carries the helix sign")
	// [SPUR-F-TOOTHTOP-ARC] step 1: the tooth-top point, rotated by the same
	// angle as the flanks, held on the tip circle. This point is the chevron's
	// apex line: the mirror reflects it onto itself, so the two halves meet
	// along it.
	topX, topY := involute.Rotate(dims.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))

	// [SPUR-F-SPINE]. The spine shares both endpoints. The +X reference line's
	// far end is pinned with two axis distances from the local origin rather
	// than onto the tip circle, and the angular dimension runs FROM the
	// reference TO the spine, which is what carries the sign of the helix.
	spine := s.CreateLine(localOrigin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(dims.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(localOrigin, refEnd, dims.Tip),
		sketch.NewVerticalDistance(localOrigin, refEnd, 0),
	)
	refLine := s.CreateLine(localOrigin, refEnd)
	refLine.SetConstruction(true)
	twistDimension := sketch.NewAngle(refLine, spine, angle*180/math.Pi)
	s.AddConstraint(twistDimension)

	// [SPUR-F-TOOTHTOP-ARC] steps 2-4: the arc is created about the local
	// origin and shares both flank ends, and carries no diameter dimension.
	// The engine shares the centre point handle, which is that rule's
	// addCoincident(arc.centerSketchPoint, localOrigin).
	s.CreateArc(localOrigin, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])

	proofkit.Step(t, "%d ribs, midpoint chain along the spine", len(left))
	// [SPUR-F-RIBS]. One rib per fit-point index, endpoints included. The rib
	// takes the axis ACROSS the spine and the chain the axis ALONG it.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prevMid := localOrigin
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
		midX, midY := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prevMid, mid, midX-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevMid, mid, midY-prevY))
		}
		prevMid, prevX, prevY = mid, midX, midY
	}

	// [SPUR-F-FLANK-ROOT]. Non-embedded only: a radial stub from the root
	// circle up to each flank's first fit point, placed by exactly two axis
	// distances from the local origin.
	if !dims.Embedded() {
		proofkit.Step(t, "flank-to-root stubs (non-embedded)")
		stub := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := dims.Root*seed.X/n, dims.Root*seed.Y/n
			rootEnd := s.CreatePoint(rx, ry)
			s.CreateLine(rootEnd, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(localOrigin, rootEnd, rx),
				sketch.NewVerticalDistance(localOrigin, rootEnd, ry),
			)
		}
		stub(leftPts[0], left[0])
		stub(rightPts[0], right[0])
	}

	proofkit.Step(t, "read the section back")
	// The harness gate solves and verifies this sketch again after the build
	// returns, and that verdict is what passes or fails the case. This solve and
	// verification are the step's own, because the regions and the solved
	// tooth-top position have to be read here to be asserted at all.
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)

	// The twist, read off the solved geometry rather than off the seed. The
	// tooth-top point is the apex of the chevron, so this is the angle the two
	// halves meet at. The formula is exact — the tip radius rotated by the helix
	// angle — and the slack is the solver's own rounding on a point it has to
	// hold on the tip circle; 1e-9 mm is orders of magnitude above the 0 mm
	// residual measured across this table and orders below any real defect.
	sketchtest.MeasuresPoint(t, toothTop, topX, topY, sketchtest.Within(1e-9))

	// [SPUR-F-ROTATE-CONFIRM]'s confirm half, checked on the committed
	// constraint rather than by recomputing the angle from the spine's
	// endpoints. Slack 1e-12 degrees: the residual measured over this table is
	// at most 1.2e-16.
	sketchtest.Satisfies(t, twistDimension, sketchtest.Within(1e-12))

	assertProfileContract(t, report, dims)
}

// assertProfileContract counts the curves on the two loops the Gear Profile
// sketch closes, and measures the disc.
//
// The counts are a contract, not a description: herringbone's inherited
// loftTooth finds BOTH of its sections with
// find_profile_by_curve_counts(nurbs=2, arcs=2, lines=2), and spur's body
// extrude finds the disc with arcs=2. A sketch that closes those regions with
// different counts is a broken sketch, and the numbers are asserted on the
// loops the proof actually drew.
func assertProfileContract(t testing.TB, report *sketch.VerificationReport, dims involute.Dimensions) {
	t.Helper()

	if len(report.Profiles) != 2 {
		t.Fatalf("the Gear Profile sketch must close exactly two regions, the tooth and the disc "+
			"inside the root circle; got %d", len(report.Profiles))
	}

	var tooth, disc *sketch.Profile
	for _, profile := range report.Profiles {
		sketchtest.IsValidProfile(t, profile)
		sketchtest.IsCurrentProfile(t, profile)
		nurbs, _, _ := loopCounts(profile)
		if nurbs > 0 {
			tooth = profile
			continue
		}
		disc = profile
	}
	if tooth == nil || disc == nil {
		t.Fatal("the two regions are not one tooth loop (which carries the flank splines) and one " +
			"disc loop (which carries none)")
	}

	// sketchtest.HasExactCuts is deliberately NOT asserted on either loop. The
	// tooth loop walks a FRAGMENT of the root circle, and the engine reports
	// that fragment's trim parameters as approximate: the cut is where a fitted
	// spline's endpoint meets a circle, which is solved numerically rather than
	// closed-form. Measured across this table, every partial edge reads
	// TExact == false, so the assertion would fail on every case and would be
	// measuring the engine's trim arithmetic rather than herringbone's scheme.
	wantTooth := "nurbs=2 arcs=2 lines=2"
	if dims.Embedded() {
		// The flanks cross the root circle themselves, so no stub is drawn and
		// the loop is four curves. The inherited loftTooth passes a fixed
		// lines=2 to both of its profile searches and never reads
		// ctx.toothProfileIsEmbedded, so this is the shape it cannot find: the
		// measured form of [HELI-F-LOFT]'s documented limitation, which
		// herringbone inherits unchanged.
		wantTooth = "nurbs=2 arcs=2 lines=0"
	}
	if got := loopShape(tooth); got != wantTooth {
		t.Errorf("tooth loop is %s, want %s", got, wantTooth)
	}

	// SUBSTITUTION, and what it costs. Spur's body extrude finds the disc with
	// find_profile_by_curve_counts(arcs=2), because in Fusion the two
	// flank-to-root stubs split the root circle and the disc's loop carries both
	// halves. This engine reports the same region as ONE whole circle edge: the
	// region's boundary covers the circle completely, so nothing there is a
	// fragment. What the proof can still pin is that the region is the whole
	// disc inside the root circle and is bounded by that circle alone — the
	// tooth loop above already proves the split happened, since it walks a
	// PARTIAL circle edge for its root arc. The count of 2 on the disc side is
	// the one number here that only a Fusion session can confirm.
	if got := loopShape(disc); got != "nurbs=0 arcs=1 lines=0" {
		t.Errorf("disc loop is %s, want the root circle alone (nurbs=0 arcs=1 lines=0 in this engine)", got)
	}
	// The disc is the full root circle. The formula is exact and the reading
	// matched it to the last digit on every case in this table; 1e-12 relative
	// is the rounding of the engine's own area integral over that circle.
	sketchtest.MeasuresProfileArea(t, disc, math.Pi*dims.Root*dims.Root, sketchtest.WithinRel(1e-12))

	if !walksRootFragment(report.Profiles) {
		t.Error("no region walks a fragment of the root circle, so the tooth did not split it and " +
			"the tooth loop is not closed at the root")
	}
}

// walksRootFragment reports whether some region's boundary walks only PART of a
// circle, which is the split the tooth's root arc is cut from.
func walksRootFragment(profiles []*sketch.Profile) bool {
	for _, profile := range profiles {
		for _, edge := range profile.Outer {
			if _, ok := edge.Entity.(*sketch.Circle); ok && edge.Partial {
				return true
			}
		}
	}
	return false
}

// loopShape renders one region's curve-type counts the way a step list writes
// them, so a failure reads as the contract it broke.
func loopShape(profile *sketch.Profile) string {
	nurbs, arcs, lines := loopCounts(profile)
	return fmt.Sprintf("nurbs=%d arcs=%d lines=%d", nurbs, arcs, lines)
}

// loopCounts counts one region's outer boundary the way
// find_profile_by_curve_counts counts a Fusion profile loop: by curve type.
//
// A fragment of the root circle is an arc, which is what Fusion sees after the
// stubs split that circle in two, so a Circle edge is counted as an arc.
//
// Two adjacent edges on the SAME circle count once. A circle's parameter runs
// from its +X seam, so an arc that spans that seam — which is exactly what the
// root arc does at helix angle 0, where the tooth sits on +X — is reported as
// two fragments meeting at t=0/t=1. That is one arc of one circle, and Fusion,
// whose profile curve carries no such seam, counts it as one.
func loopCounts(profile *sketch.Profile) (nurbs, arcs, lines int) {
	edges := profile.Outer
	for i, edge := range edges {
		if len(edges) > 1 {
			previous := edges[(i-1+len(edges))%len(edges)]
			circle, isCircle := edge.Entity.(*sketch.Circle)
			previousCircle, previousIsCircle := previous.Entity.(*sketch.Circle)
			if isCircle && previousIsCircle && circle == previousCircle {
				continue
			}
		}
		switch edge.Entity.(type) {
		case *sketch.FitSpline, *sketch.Spline, *sketch.NURBS:
			nurbs++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}

// WHAT THIS PROOF DOES NOT REACH.
//
// Every step the compile marked [PROSE], and every part of a [GO] step neither
// harness can hold, is recorded here rather than only in the step list, so the
// next reader of the proof finds the edge of what it checks.
//
// THE MODULE SURFACE. Three classes, two of them empty subclasses, and the
// identity overrides: newContext returning this gear's context, prefixBase
// returning 'HerringboneGear', and generateName's four .expression strings.
// Those are Python and Fusion document state. A sketch engine and a solid engine
// model geometry, not a class hierarchy or a component's name.
//
// THE DIALOG AND THE PARAMETER TABLE. Herringbone adds no input and no
// parameter: its dialog is helical's, its parameters are helical's and spur's,
// and every registration happens in inherited code. There is nothing here for
// either harness to build, and nothing herringbone-specific to check if there
// were.
//
// THE NAMED BODIES AND THE COMBINE'S DEFAULT OPERATION. The mirrored half is
// renamed 'Tooth Body (Mirrored)' and the combine looks its target up with
// bRepBodies.itemByName('Tooth Body'), leaving combineInput.operation at its API
// default of Join. decad has no body names and no feature-input object, so the
// solid proof joins two body handles directly: it proves what a Join produces —
// one solid spanning both halves — and not that the target was found by that
// name or that the operation was left unassigned. Only a Fusion session settles
// those two.
//
// VISIBILITY. The helix construction plane is left visible after generation and
// the Twisted Gear Profile sketch stays hidden its whole life
// ([HELI-F-TWIST-PLANE]). Both are deliberate, and neither harness carries a
// visibility flag to assert them on.
//
// SKETCH TEXT. Fusion's drawCircles labels each of the four circles with
// along-path sketch text, and text carries its own position along the curve that
// nothing pins ([PB-TEXT-HOLDS-DOF]). The sketch engine has no text at all, so
// the DOF-0 verdict above is about the tooth's geometry: the same sketch in
// Fusion may read isFullyConstrained == False purely because it is labelled.
// Herringbone registers no runtime full-constraint gate, so nothing in the
// generated module depends on that reading either way.
//
// THE INHERITED PIPELINE AFTER THE TOOTH. The body extrude across the full
// thickness, the circular pattern, the root fillets, the bore and the
// completed-gear chamfer are spur's code, unchanged, and are proven in
// proof/spurgear. Herringbone changes none of them, and re-proving them here
// would state that spur is still spur rather than anything about this gear.
