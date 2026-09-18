// Package helicalgear_test proves the helical gear's build, one function per
// [GO] step of spec/helicalgear/steps.md.
//
// Helical is a thin specialization of the spur gear. It inherits spur's whole
// pipeline and changes three things: one extra dialog input, a second "Twisted
// Gear Profile" sketch drawn by the spur tooth generator at angle = HelixAngle,
// and a loft between the bottom and top tooth profiles in place of spur's tooth
// extrude. Only the last two build geometry, so only those two are proved here;
// everything else helical inherits is proved by proof/spurgear, and re-proving
// it here would prove it twice and build nothing new.
//
// This file holds the sketch step: the Twisted Gear Profile sketch, which is
// the spur tooth generator's angle != 0 path ([SPUR-F-SPINE],
// [SPUR-F-ROTATE-CONFIRM]) run at the user's Helix Angle. What it proves is the
// sketch-first gate [PB-SKETCH-FIRST] for that path — the scheme reaches DOF 0
// with no conflicting or redundant constraint and no discrete ambiguity, across
// the whole signed range of the angle the dialog accepts — plus the two facts
// the loft step downstream selects on: the twist the top section carries, and
// the curve counts of the loop loftTooth's fixed key matches.
//
// Three things here are outside what a sketch engine can reach, and each is
// recorded next to the thing it cannot reach.
//
// The four circle labels are sketch text. This engine has no sketch text, and
// in Fusion text carries its own position along the curve and is never pinned,
// so a labelled sketch does not reliably report isFullyConstrained even when
// its geometry is completely determined ([PB-TEXT-HOLDS-DOF]). Neither the text
// nor that reading is reproduced.
//
// [SPUR-F-ANCHOR-CHAIN]'s chain of projections is not reproduced either. The
// engine refuses a reference to another sketch's point as a foreign handle,
// exactly as Fusion does, so what this sketch carries is its own local endpoint
// of the chain: one reference point standing for the Tools-sketch projection
// the twisted sketch re-projects.
//
// The tooth loop is deliberately NOT held to sketchtest.HasExactCuts. Its root
// boundary is a fragment of the root circle, and exact boundary parameters are
// withdrawn from every partial edge in a scene that holds a free-form entity —
// which this sketch does, twice, in the two flank splines. That is the same
// engine rule that makes the solid step chord those flanks, and it is why the
// loop is held to its curve COUNTS here rather than to its trimmed parameters.
package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// twistedProfileCases sweep the regime the twisted sketch has to hold across.
//
// Helix Angle is the spur tooth generator's own angle argument, so spur's
// regime for that argument governs it verbatim, and the helical spec adds that
// the value is SIGNED — negative is a left-hand helix — and that no range is
// enforced anywhere, in the dialog or in the code. So:
//
//   - the dialog default, 14.5 degrees, positive and negative. A scheme that
//     dropped or flipped the confirming angular dimension still solves at
//     +angle and comes out mirrored at -angle, so the negative cases are the
//     point rather than decoration;
//   - zero, which is the angle == 0 branch of [SPUR-F-ROTATE-CONFIRM]: the
//     angular dimension exists at 0 and there is nothing to set afterwards.
//     It is also the bottom section's own case, so the assertion that this
//     sketch's tooth top sits at exactly the angle drawn is, at this case, the
//     baseline the twist of every other case is measured against;
//   - a quarter turn either way, where |sin| > |cos| swaps which axis the rib
//     and the midpoint chain take ([SPUR-F-RIBS]);
//   - sizes, coarse and fine, since the rib chain's dimensions scale with the
//     tooth and the conditioning of the system does not;
//   - the rib count at the low end as well as the standard 15, where one
//     missing or redundant dimension is a large fraction of the system;
//   - both routes into the embedded shape — a high tooth count at the ordinary
//     20 degree pressure angle, and a moderate tooth count at a large pressure
//     angle. Helical cannot BUILD an embedded tooth ([HELI-F-LOFT] passes a
//     fixed lines=2 key), and the two cases here are what proves that: the loop
//     they close is not the loop that key matches;
//   - the anchor on the sketch origin and off it, since nothing in the dialog
//     requires the user to put it on the origin and the whole sketch is dragged
//     onto wherever it is.
var twistedProfileCases = []proofkit.Case{
	{Name: "M1_N17_helix_zero", Params: twistedParams(1, 17, 20, 0, 15)},
	{Name: "M1_N17_helix_plus_default_14_5", Params: twistedParams(1, 17, 20, 14.5, 15)},
	{Name: "M1_N17_helix_minus_default_14_5", Params: twistedParams(1, 17, 20, -14.5, 15)},
	{Name: "M2_N20_helix_plus_30", Params: twistedParams(2, 20, 20, 30, 15)},
	{Name: "M3_N15_helix_minus_45", Params: twistedParams(3, 15, 20, -45, 15)},
	{Name: "M1_N12_helix_plus_quarter_turn", Params: twistedParams(1, 12, 20, 90, 15)},
	{Name: "M1_N12_helix_minus_quarter_turn", Params: twistedParams(1, 12, 20, -90, 15)},
	{Name: "M1_N17_helix_plus_14_5_four_samples", Params: twistedParams(1, 17, 20, 14.5, 4)},
	{Name: "M1_N17_helix_minus_14_5_two_samples", Params: twistedParams(1, 17, 20, -14.5, 2)},
	{Name: "embedded_high_count_M1_N43_helix_plus_14_5", Params: twistedParams(1, 43, 20, 14.5, 15)},
	{Name: "embedded_large_pressure_angle_PA30_N20_helix_minus_14_5", Params: twistedParams(1, 20, 30, -14.5, 15)},
	{Name: "M1_N17_helix_plus_14_5_anchor_off_origin", Params: anchoredAt(twistedParams(1, 17, 20, 14.5, 15), 8, -5)},
	{Name: "M2_N20_helix_minus_30_anchor_off_origin", Params: anchoredAt(twistedParams(2, 20, 20, -30, 15), -12, 7)},
}

// twistedParams names one case by the dialog values it comes from. Module and
// Tooth Number are the two size inputs, the pressure angle and the helix angle
// are given in degrees and carried in radians — the Helix Angle dialog input is
// a degree field whose HelixAngle user parameter is registered in radians — and
// involuteSteps is the derived Involute Steps parameter, 15 in the shipped gear.
func twistedParams(module, toothNumber, pressureAngleDeg, helixAngleDeg float64, steps int) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": rad(pressureAngleDeg),
		"helixAngle":    rad(helixAngleDeg),
		"involuteSteps": float64(steps),
		"anchorX":       0,
		"anchorY":       0,
	}
}

// anchoredAt moves a case's anchor off the sketch origin. The tooth is drawn
// about the sketch's own movable local origin and only then dragged onto the
// projected anchor ([SPUR-F-LOCAL-ORIGIN]), so the drag distance is a parameter
// of the scheme and zero only where the user's anchor happens to sit on the
// sketch origin.
func anchoredAt(p map[string]float64, x, y float64) map[string]float64 {
	p["anchorX"], p["anchorY"] = x, y
	return p
}

func rad(deg float64) float64 { return deg * math.Pi / 180 }

// toolsProjectionSource is the source id the twisted sketch's re-projection of
// the Tools-sketch anchor carries. In Fusion the Tools projection is the
// canonical handle and every later sketch re-projects THAT
// ([SPUR-F-ANCHOR-CHAIN]); the engine refuses another sketch's point outright,
// so this sketch carries its own reference point tagged with this id.
const toolsProjectionSource = "Tools sketch anchor projection"

// stepTwistedGearProfileSketch draws the Twisted Gear Profile sketch: the spur
// tooth generator run on a second sketch at angle = HelixAngle.
//
// Helical does nothing special to the geometry. It constructs
// SpurGearInvoluteToothDesignGenerator on the loft sketch and calls
// draw(ctx.anchorPoint, angle=HelixAngle), and the generator draws the whole
// tooth already rotated by that angle in its own point math, then confirms the
// rotation with the spine's angular dimension as its very last action
// ([SPUR-F-ROTATE-CONFIRM]). Both halves are reproduced here, because it is
// exactly their combination that the helical loft depends on: a tooth drawn
// flat and swung into place by the dimension alone can settle on the branch
// about 180 degrees away, and the loft then passes through the gear centre.
func stepTwistedGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	module := p["module"]
	toothNumber := p["toothNumber"]
	pressureAngle := p["pressureAngle"]
	angle := p["helixAngle"]
	steps := int(p["involuteSteps"])
	d := involute.Derive(module, toothNumber, pressureAngle)

	proofkit.Step(t, "local origin")
	origin := s.CreatePoint(0, 0)

	// The four circles share the local origin as their centre by being created
	// on the point object itself, never on its coordinates
	// ([PB-SHARE-XOR-COINCIDENT], applied in [SPUR-F-SHARED-ADJACENCY]). Only
	// the root circle is solid; the other three are construction and bound no
	// region, which is why the disc below is the root disc and not an annulus.
	proofkit.Step(t, "four circles, each with a driving diameter dimension")
	circle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(d.Root, false)
	tip := circle(d.Tip, true)
	circle(d.Base, true)
	circle(d.Pitch, true)

	proofkit.Step(t, "involute flanks, drawn already rotated by the helix angle")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothNumber, steps, angle)
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

	// The tooth-top point sits on the tip circle at the rotated angle, and the
	// arc's centre is tied back to the local origin because
	// addByCenterStartEnd shares the two ends and COPIES the centre
	// ([SPUR-F-TOOTHTOP-ARC]). Without that coincidence the centre is a free
	// point that stays behind when the sketch is dragged onto the anchor.
	proofkit.Step(t, "tooth-top point and the tooth-top arc")
	topX, topY := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))

	// The +X reference line is built for every angle, including 0, and the
	// angular dimension from it to the spine is what says which way the spine
	// points ([SPUR-F-SPINE]). Its far endpoint is pinned with two axis
	// dimensions from the local origin rather than onto the tip circle, where a
	// point has two answers and the numbers go unstable.
	proofkit.Step(t, "spine, +X reference and the confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	referenceEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, referenceEnd, d.Tip),
		sketch.NewVerticalDistance(origin, referenceEnd, 0),
	)
	reference := s.CreateLine(origin, referenceEnd)
	reference.SetConstruction(true)
	// The engine's angular dimension is signed and reads in the sketch's
	// default angle unit, degrees. Fusion's is a magnitude whose direction is
	// captured from the seeded geometry, so the sign crosses over as the seed
	// side and the drawn rotation, never as a negative parameter value
	// ([PB-DIM-VALUE-SEMANTICS]).
	spineAngle := sketch.NewAngle(reference, spine, angle*180/math.Pi)
	s.AddConstraint(spineAngle)

	// One rib per fit-point index, endpoints included: the fit points carry no
	// other constraint, so an omitted endpoint rib leaves one free. The order
	// is fixed ([SPUR-F-RIBS]) — rib, axis dimension, midpoint seeded ON the
	// spine, point-on-line, midpoint, perpendicular — and the last rib carries
	// no perpendicular, because the tooth-top arc already holds the two flank
	// tips at equal radius either side of the spine.
	proofkit.Step(t, "ribs and the midpoint chain along the spine")
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previous := origin
	previousX, previousY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		along := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := along*math.Cos(angle), along*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, midX-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, midY-previousY))
		}
		previous, previousX, previousY = mid, midX, midY
	}

	// The stubs exist only where the flank starts outside the root circle. Each
	// root end is placed by exactly two axis dimensions from the local origin,
	// whose captured directions say which side of the gear centre it sits on
	// ([SPUR-F-FLANK-ROOT]).
	proofkit.Step(t, "flank-to-root lines")
	if !d.Embedded() {
		stub := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			x, y := d.Root*seed.X/n, d.Root*seed.Y/n
			rootEnd := s.CreatePoint(x, y)
			s.CreateLine(rootEnd, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, x),
				sketch.NewVerticalDistance(origin, rootEnd, y),
			)
		}
		stub(leftPts[0], left[0])
		stub(rightPts[0], right[0])
	}

	// draw() anchors the sketch itself, as its second-to-last action, and
	// helical relies on that single call: if the anchoring moved up into
	// buildSketches the twisted sketch would be left unconstrained and the loft
	// would float off the anchor.
	proofkit.Step(t, "anchor the sketch onto the projected Anchor Point")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the twist the loft's top section carries")
	assertTwistDrawnAndConfirmed(t, s, spineAngle, origin, toothTop, d, p)

	proofkit.Step(t, "the loops loftTooth selects on")
	assertLoopContract(t, s, d)
}

// assertTwistDrawnAndConfirmed reads the solved sketch and holds it to the two
// halves of [SPUR-F-ROTATE-CONFIRM] at once.
//
// The tooth top is measured where the drawn rotation puts it — on the tip
// circle at the helix angle, offset by the anchor the whole sketch was dragged
// onto — and the confirming angular dimension is asked whether it is satisfied
// rather than recomputed from the geometry it constrains. A scheme that drew
// the tooth flat and left the dimension to swing it would pass neither at a
// negative angle.
//
// The polar angle is the twist the loft sees. The bottom section is the same
// generator called at angle 0, which is this table's own helix_zero case, so
// what these two cases together say is that the angle between the two sections
// IS the Helix Angle: nothing rescales it into a lead angle, and Thickness,
// which does not appear in this sketch at all, cannot enter it.
//
// The solve is this proof's own, ahead of the harness gate's, because every
// reading below is of solved geometry. The formulas are exact, so the slack is
// the solver's convergence and float noise only.
func assertTwistDrawnAndConfirmed(t testing.TB, s *sketch.Sketch, spineAngle *sketch.Angle,
	origin, toothTop *sketch.Point, d involute.Dimensions, p map[string]float64) {
	t.Helper()
	sketchtest.Solve(t, s)

	angle := p["helixAngle"]
	sketchtest.MeasuresPoint(t, toothTop,
		p["anchorX"]+d.Tip*math.Cos(angle), p["anchorY"]+d.Tip*math.Sin(angle),
		sketchtest.Within(1e-6))
	sketchtest.MeasuresPoint(t, origin, p["anchorX"], p["anchorY"], sketchtest.Within(1e-6))
	sketchtest.Measures(t, "twist of the top section from +X",
		math.Atan2(toothTop.Y()-origin.Y(), toothTop.X()-origin.X()), angle,
		sketchtest.Within(1e-9))
	sketchtest.Satisfies(t, spineAngle, sketchtest.Within(1e-9))
}

// assertLoopContract holds the drawn sketch to the loops the two features
// downstream select on, on the geometry it actually drew rather than on a
// stand-in outline.
//
// loftTooth finds its top section with a FIXED nurbs=2, arcs=2, lines=2 key
// ([HELI-F-LOFT]) — it never reads ctx.toothProfileIsEmbedded and has no
// embedded branch. So this is where that limitation is either satisfied or
// proved: where the flank starts outside the root circle the sketch closes
// exactly that loop, and where it starts inside, the loop it closes has no
// stubs and the fixed key matches nothing in the sketch. An embedded helical
// gear therefore fails to find its profile, which is faithful to the code and a
// documented limitation rather than a defect of this proof.
//
// The disc inside the root circle is the loop spur's inherited body extrude
// selects on. Where the two engines part company is its arc COUNT: Fusion
// splits the solid root circle where the tooth meets it and the body loop takes
// two arcs, while this engine splits a curve only where a region needs it and
// hands back the one whole circle entity of the root circle's own area. What is
// proved here is that the region exists, that it is the root disc and not an
// annulus — the tip circle is construction and bounds nothing — and that it is
// extrudable; the arc count on it is the part only a Fusion session settles.
func assertLoopContract(t testing.TB, s *sketch.Sketch, d involute.Dimensions) {
	t.Helper()
	report := sketchtest.Verify(t, s)
	if report == nil {
		return
	}

	wantLines := 2
	if d.Embedded() {
		wantLines = 0
	}
	tooth, disc, loftKey := 0, 0, 0
	for _, region := range report.Profiles {
		nurbs, arcs, lines := loopCounts(region.Entities)
		if nurbs == 2 && arcs == 2 && lines == 2 {
			loftKey++
		}
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth++
			sketchtest.IsValidProfile(t, region)
			sketchtest.IsCurrentProfile(t, region)
		case nurbs == 0 && arcs == 1 && lines == 0:
			disc++
			sketchtest.MeasuresProfileArea(t, region, math.Pi*d.Root*d.Root, sketchtest.WithinRel(1e-9))
			sketchtest.IsValidProfile(t, region)
			sketchtest.IsCurrentProfile(t, region)
		default:
			t.Errorf("unexpected region: %d NURBS, %d arcs, %d lines", nurbs, arcs, lines)
		}
	}
	if len(report.Profiles) != 2 {
		t.Errorf("closed regions: %d, want exactly 2 (the tooth and the disc)", len(report.Profiles))
	}
	if tooth != 1 {
		t.Errorf("tooth loops of 2 NURBS, 2 arcs, %d lines: %d, want 1", wantLines, tooth)
	}
	if disc != 1 {
		t.Errorf("disc loops inside the root circle: %d, want 1", disc)
	}
	if d.Embedded() && loftKey != 0 {
		t.Errorf("an embedded tooth closed %d loop(s) matching loftTooth's fixed nurbs=2, arcs=2, "+
			"lines=2 key; [HELI-F-LOFT] records that key as finding nothing here", loftKey)
	}
	if !d.Embedded() && loftKey != 1 {
		t.Errorf("loops matching loftTooth's fixed nurbs=2, arcs=2, lines=2 key: %d, want 1", loftKey)
	}
}

// loopCounts classifies a region's DISTINCT boundary entities the way
// find_profile_by_curve_counts classifies Fusion's profile curves: a fitted
// spline is a NURBS, the tooth-top arc and the root boundary are each an arc,
// and a flank-to-root stub is a line.
//
// Profile.Entities is the de-duplicated entity set, and the de-duplication is
// what makes the two engines comparable: a tooth drawn across this engine's
// own parameterisation seam reports one entity in two fragments, and counting
// entities reads that piece once, which is the count Fusion sees.
func loopCounts(entities []sketch.Entity) (nurbs, arcs, lines int) {
	for _, entity := range entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}
