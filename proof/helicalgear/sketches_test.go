// Package helicalgear_test proves the helical gear's own two build deltas —
// the twisted top profile sketch and the loft that replaces spur's tooth
// extrude — against the step list compiled from spec/helicalgear/instructions.md
// and spec/helicalgear/fusion.md.
//
// Helical is a thin specialization of spur: it inherits the whole spur
// pipeline and changes three things, one input and two build steps. So this
// proof covers those and nothing else. The bottom Gear Profile sketch, the
// body extrude, the pattern, the fillets, the bore and the completed-gear
// chamfer are spur's and are proven there.
//
// The tooth math is imported from proof/involute rather than restated, because
// helical draws the SAME tooth spur draws, only pre-rotated by the helix angle.
package helicalgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// twistedCases sweeps the twisted Gear Profile across the regime the spec
// states the scheme must hold over.
//
// Size, because the rib chain's dimensions scale with the tooth and the
// conditioning of the system does not. The whole SIGNED range of the helix
// angle, because the sign is the hand of the helix: a scheme that drops or
// flips the confirming angular dimension still solves at +angle and comes out
// mirrored at -angle, so a table of positive angles proves nothing about the
// sign. A quarter turn, where |sin| > |cos| swaps which axis the rib and the
// chain dimensions take. A low rib count, where one missing or redundant
// dimension is a large fraction of the system. And both routes into the
// embedded shape, which helical does not support and which this proof measures
// rather than assumes.
var twistedCases = []proofkit.Case{
	{Name: "default_M1_N17_helix14.5", Params: params(1, 17, 20, 14.5, 15)},
	{Name: "coarse_M3_N15_helix14.5", Params: params(3, 15, 20, 14.5, 15)},
	{Name: "fine_M0.5_N24_helix14.5", Params: params(0.5, 24, 20, 14.5, 15)},
	{Name: "large_M2_N20_helix14.5", Params: params(2, 20, 20, 14.5, 15)},

	{Name: "helix0_spur_baseline", Params: params(1, 17, 20, 0, 15)},
	{Name: "helix_plus10", Params: params(1, 17, 20, 10, 15)},
	{Name: "helix_minus10_left_hand", Params: params(1, 17, 20, -10, 15)},
	{Name: "helix_plus35", Params: params(1, 17, 20, 35, 15)},
	{Name: "helix_minus35_left_hand", Params: params(1, 17, 20, -35, 15)},
	{Name: "helix_plus90_quarter_turn", Params: params(1, 17, 20, 90, 15)},
	{Name: "helix_minus90_quarter_turn", Params: params(1, 17, 20, -90, 15)},

	{Name: "ribs_low_count_5_helix14.5", Params: params(1, 17, 20, 14.5, 5)},
	{Name: "ribs_low_count_3_helix_minus25", Params: params(1, 17, 20, -25, 3)},

	{Name: "embedded_by_tooth_count_N60_PA20", Params: paramsPA(1, 60, 20, 14.5, 15)},
	{Name: "embedded_by_pressure_angle_N30_PA30", Params: paramsPA(1, 30, 30, 14.5, 15)},
}

// params builds one case's parameter set. Angles arrive in degrees, the unit
// the dialog uses, and are held in radians, the unit the HelixAngle user
// parameter is registered in.
func params(module, toothNumber, pressureAngleDeg, helixAngleDeg float64, steps int) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": pressureAngleDeg * math.Pi / 180,
		"helixAngle":    helixAngleDeg * math.Pi / 180,
		"involuteSteps": float64(steps),
		"thickness":     10,
	}
}

// paramsPA is params under a different name, kept so the embedded cases read as
// what they are: the two independent routes into the embedded shape.
func paramsPA(module, toothNumber, pressureAngleDeg, helixAngleDeg float64, steps int) map[string]float64 {
	return params(module, toothNumber, pressureAngleDeg, helixAngleDeg, steps)
}

// stepTwistedGearProfile draws the Twisted Gear Profile sketch: the spur tooth
// generator run at angle = HelixAngle on the offset helix plane.
//
// Everything here is spur's construction ([SPUR-F-…]); helical passes a
// non-zero angle into it and nothing else. The sketch is proven on the world XY
// plane because the constraint scheme is plane-local — the helix plane's own
// offset is proven by stepHelixPlane, in the solid proof, where an offset can
// be measured.
func stepTwistedGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	module := p["module"]
	toothNumber := p["toothNumber"]
	pressureAngle := p["pressureAngle"]
	angle := p["helixAngle"]
	steps := int(p["involuteSteps"])
	dims := involute.Derive(module, toothNumber, pressureAngle)

	proofkit.Step(t, "twisted profile: module=%g teeth=%g pressureAngle=%.4frad helix=%.4frad steps=%d embedded=%v",
		module, toothNumber, pressureAngle, angle, steps, dims.Embedded())

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
	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	proofkit.Step(t, "spine, +X reference and the angular pin")
	// [SPUR-F-TOOTHTOP-ARC] step 1: the tooth-top point, rotated by the same
	// angle as the flanks, held on the tip circle.
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
	s.AddConstraint(sketch.NewAngle(refLine, spine, angle*180/math.Pi))

	// [SPUR-F-TOOTHTOP-ARC] steps 2-4: the arc is created about the local
	// origin and shares both flank ends, and carries no diameter dimension.
	// The engine shares the centre point handle, which is that rule's
	// addCoincident(arc.centerSketchPoint, localOrigin) — the arc's centre is
	// tied to the origin, not left free behind the anchor drag.
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

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve: %v", err)
	}
	assertProfileContract(t, s, dims, leftFlank, rightFlank)
}

// assertProfileContract counts the curves on the two loops the Gear Profile
// sketch closes.
//
// The counts are a contract, not a description: the loft finds both of its
// sections with find_profile_by_curve_counts(nurbs=2, arcs=2, lines=2), and
// spur's body extrude finds the disc with arcs=2. A sketch that closes those
// regions with different counts is a broken sketch, and the numbers are
// asserted on the loops the proof actually drew.
func assertProfileContract(t testing.TB, s *sketch.Sketch, dims involute.Dimensions,
	leftFlank, rightFlank *sketch.FitSpline) {
	t.Helper()

	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("the Gear Profile sketch must close exactly two regions, the tooth and the disc "+
			"inside the root circle; got %d", len(profiles))
	}

	var tooth, disc string
	var discArea float64
	for _, prof := range profiles {
		nurbs, arcs, lines := loopCounts(prof)
		shape := fmt.Sprintf("nurbs=%d arcs=%d lines=%d", nurbs, arcs, lines)
		switch {
		case nurbs > 0:
			tooth = shape
		default:
			disc = shape
			discArea = prof.Area
		}
	}

	wantTooth := "nurbs=2 arcs=2 lines=2"
	if dims.Embedded() {
		// The flanks cross the root circle themselves, so no stub is drawn and
		// the loop is four curves. helicalgear.py's loftTooth passes a fixed
		// lines=2 to both of its profile searches and never reads
		// ctx.toothProfileIsEmbedded, so this is the shape it cannot find: the
		// measured form of [HELI-F-LOFT]'s documented limitation.
		wantTooth = "nurbs=2 arcs=2 lines=0"
	}
	if tooth != wantTooth {
		t.Errorf("tooth loop is %s, want %s", tooth, wantTooth)
	}

	// SUBSTITUTION, and what it costs. Spur's body extrude finds the disc with
	// find_profile_by_curve_counts(arcs=2), because in Fusion the two
	// flank-to-root stubs split the root circle and the disc's loop carries
	// both halves. This engine reports the same region as ONE whole circle
	// edge: the region's boundary covers the circle completely, so nothing
	// there is a fragment. What the proof can still pin is that the region is
	// the whole disc inside the root circle and is bounded by that circle
	// alone — the tooth loop above already proves the split happened, since it
	// walks a PARTIAL circle edge for its root arc. The count of 2 on the disc
	// side is the one number here that only a Fusion session can confirm.
	if disc != "nurbs=0 arcs=1 lines=0" {
		t.Errorf("disc loop is %s, want the root circle alone (nurbs=0 arcs=1 lines=0 in this engine)", disc)
	}
	wantArea := math.Pi * dims.Root * dims.Root
	if math.Abs(discArea-wantArea) > 1e-6*wantArea {
		t.Errorf("disc area is %.6f, want the full root disc %.6f", discArea, wantArea)
	}
	if !toothLoopWalksRootFragment(profiles) {
		t.Error("no region walks a fragment of the root circle, so the flank-to-root stubs did not " +
			"split it and the tooth is not closed at the root")
	}
}

// toothLoopWalksRootFragment reports whether some region's boundary walks only
// PART of a circle, which is the split the tooth's root arc is cut from.
func toothLoopWalksRootFragment(profiles []*sketch.Profile) bool {
	for _, prof := range profiles {
		for _, edge := range prof.Outer {
			if _, ok := edge.Entity.(*sketch.Circle); ok && edge.Partial {
				return true
			}
		}
	}
	return false
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
func loopCounts(prof *sketch.Profile) (nurbs, arcs, lines int) {
	edges := prof.Outer
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
// Every step below is one this compile marked [PROSE], and this is the record
// of why, kept next to the sketch they belong to rather than only in the step
// list.
//
// SKETCH TEXT. Fusion's drawCircles labels each of the four circles with
// along-path sketch text, and text carries its own position along the curve
// that nothing pins ([PB-TEXT-HOLDS-DOF]). The sketch engine has no text at
// all, so the DOF-0 verdict above is about the tooth's geometry: the same
// sketch in Fusion reads isFullyConstrained == False purely because it is
// labelled. Helical registers no runtime full-constraint gate, so nothing in
// the generated module depends on that reading either way.
//
// THE FILLET FACTOR. filletHelixFactorExpression returns the STRING
// 'cos(<prefix>_HelixAngle)', which registerDerivedParameters splices into the
// FilletRadius expression. It is a Fusion expression, evaluated by Fusion's own
// parameter engine, and neither harness holds a parameter table to evaluate it
// in. The spec states the factor without deriving it, so this proof would have
// nothing to check the number against even if it could evaluate it.
//
// THE DIALOG AND THE PARAMETER TABLE. The Helix Angle input's id, label, unit
// string and default, its position after Parent Component, the HelixAngle
// parameter's registration in radians from a degree input, and generateName's
// four .expression strings are all Fusion document state. A sketch engine and a
// solid engine model geometry, not a command dialog.
//
// VISIBILITY. The Twisted Gear Profile sketch stays hidden its whole life and
// the helix ConstructionPlane is left visible after generation
// ([HELI-F-TWIST-PLANE]). Both are display state on a Fusion document, and both
// are deliberate; neither harness carries a visibility flag to assert them on.
