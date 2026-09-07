package helicalgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// loftCases is the regime the loft has to hold across. The helix angle is
// swept per sign, because the spec leaves the bound to this proof and says it
// need not be symmetric about zero.
//
// The bound, measured at M=1, N=17, T=10 with 15 involute steps on the
// loft-ready sections: every left-hand twist tried builds a sound solid, down
// to −179°; a right-hand twist builds up to +90° and is refused from +120° on
// by decad's wall audit ("loft triangles 0 and 4 share no recorded vertex, but
// make contact"). The asymmetry is in the ruled walls: decad splits each
// ruled quad into two triangles along one fixed diagonal, so a right-hand
// twist folds the facets one way and a left-hand twist the other. It is a
// property of this model and of the solid engine, not of the gear, which is
// why the table stops at ±90° on the right and carries −120° on the left as
// the record of the asymmetry, and why nothing here is quoted in the spec.
//
// The embedded case is here so the loft step's one branch — the fixed
// lines=2 profile search finding nothing — has a case on its side too.
var loftCases = []proofkit3d.Case{
	{Name: "M1_N17_helix+14.5_T10", Params: caseParams(1, 17, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-14.5_T10_left_hand", Params: caseParams(1, 17, 20, -14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N12_helix+14.5_T5", Params: caseParams(1, 12, 20, 14.5, defaultInvoluteSteps, 5)},
	{Name: "M2_N20_helix-14.5_T20", Params: caseParams(2, 20, 20, -14.5, defaultInvoluteSteps, 20)},
	{Name: "M3_N15_helix+14.5_T10", Params: caseParams(3, 15, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N35_helix+14.5_T10_short_stub", Params: caseParams(1, 35, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+14.5_T10_steps4", Params: caseParams(1, 17, 20, 14.5, 4, defaultThickness)},
	{Name: "M1_N17_helix-14.5_T10_steps4", Params: caseParams(1, 17, 20, -14.5, 4, defaultThickness)},
	{Name: "M1_N17_helix+30_T10", Params: caseParams(1, 17, 20, 30, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-30_T10", Params: caseParams(1, 17, 20, -30, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+45_T10", Params: caseParams(1, 17, 20, 45, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-45_T10", Params: caseParams(1, 17, 20, -45, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+60_T10", Params: caseParams(1, 17, 20, 60, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-60_T10", Params: caseParams(1, 17, 20, -60, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+90_T10_right_hand_bound", Params: caseParams(1, 17, 20, 90, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-90_T10", Params: caseParams(1, 17, 20, -90, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-120_T10_left_hand_builds_where_right_is_refused", Params: caseParams(1, 17, 20, -120, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+90_T40", Params: caseParams(1, 17, 20, 90, defaultInvoluteSteps, 40)},
	{Name: "M1_N60_PA20_helix+14.5_embedded_unsupported", Params: caseParams(1, 60, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
}

// loftFrame turns both loft sections half a turn from Fusion's sketch frame.
//
// The sketch engine parametrises a circle from its local +X. A tooth centred
// on +X, which is where the bottom Gear Profile tooth sits, straddles that
// seam, and the root-circle piece under it is reported as two edges, one either
// side of the seam. Fusion has no seam and reports one arc. decad pairs loft
// segments one to one, so a bottom loop of 33 edges against a top loop of 32
// is refused before any geometry is built. Turning the frame puts both teeth
// on the far side of the circle from the seam for every helix angle in the
// table. The recipe is unchanged: the reference line, the seeds and the
// signed axis dimensions all turn with the frame, the angular dimension still
// measures the helix angle from the reference line, and the centroid reading
// is turned back before it is compared. What the turn costs is that the loft
// proof is not drawn on the same absolute bearing as the Fusion sketch; the
// sketch step, which pairs nothing, stays in Fusion's frame.
const loftFrame = math.Pi

// stepLoftTooth is buildTooth → loftTooth ([HELI-F-LOFT]): the bottom Gear
// Profile tooth loop (the spur tooth at angle 0, on the target plane) lofted to
// the Twisted Gear Profile tooth loop (the same tooth at angle HelixAngle, on
// the plane offset by Thickness — [HELI-F-TWIST-PLANE], helicalPlaneOffset()),
// bottom section added first, as a new body.
//
// Two substitutions, and what each costs:
//
//   - Both sections are drawn loft-ready (see drawToothProfile): the involute
//     flanks chorded through their fit points, the tooth-top arc and the root
//     piece each replaced by a chord on the same endpoints. decad's loft pairs
//     LineSeg, ArcSeg and CircleSeg only; a free-form entity anywhere in a
//     sketch withdraws the exact trim every fragment needs to be recorded; and
//     a circular pair is chorded by decad with a volume bound its verification
//     then refuses. The chorded loop keeps every constraint and every point, so
//     the twist it carries is the sketch's; the walls are faceted and the caps
//     flat, so the volume read here is the faceted body's, not Fusion's. The
//     6-curve count the module's profile search keys on is proven by
//     stepTwistedGearProfile on the splined sketch with the solid root circle;
//     here the same loop is found by its loft-ready count, 2·(steps−1) + 4 lines.
//
//   - Fusion's loftSections.add(profile) takes the whole profile and pairs the
//     curves itself. decad pairs recorded segments by index and takes the
//     rotation of the top loop that pairs with the bottom's first segment as
//     WithLoftAlignment; the proof computes that rotation by rotating the
//     bottom loop's first vertex by the helix angle and finding the top edge
//     that starts there. Fusion does not have this argument, so the step list
//     does not carry it.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	steps := int(p[keyInvoluteSteps])
	helix := p[keyHelixAngle]
	thickness := p[keyThickness]

	world := sketch.NewWorld()
	proofkit.Step(t, "bottom Gear Profile on the target plane, angle 0")
	bottomSketch, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("create bottom sketch: %v", err)
	}
	bottom := drawToothProfile(t, bottomSketch, p, loftFrame, 0, true)

	// [HELI-F-TWIST-PLANE]: constructionPlanes.createInput() + setByOffset(self.plane,
	// helicalPlaneOffset()) — the offset is the full Thickness for helical.
	proofkit.Step(t, "helix plane at Thickness and the Twisted Gear Profile on it")
	helixPlane, err := world.CreateOffsetPlane(world.XY(), thickness)
	if err != nil {
		t.Fatalf("create helix plane: %v", err)
	}
	helixPlane.SetName("helixPlane")
	topSketch, err := world.CreateSketch(helixPlane)
	if err != nil {
		t.Fatalf("create twisted sketch: %v", err)
	}
	top := drawToothProfile(t, topSketch, p, loftFrame, helix, true)

	if bottom.embedded != top.embedded {
		t.Fatalf("the two sections disagree on the embedded shape (bottom %v, top %v)", bottom.embedded, top.embedded)
	}

	// Both sections must solve soundly before their loops are read; the sketch
	// gate is the same one proofkit applies to the sketch step.
	proofkit.Step(t, "solve both sections")
	proofkit.RequireSound(t, bottomSketch)
	proofkit.RequireSound(t, topSketch)

	// find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=2) on each
	// sketch. In loft-ready form the same loop reads as 2·(steps−1) + 4 lines.
	proofkit.Step(t, "find the tooth loop in each section")
	want := loftLoopCounts(steps)
	bottomProfiles := bottomSketch.Profiles()
	topProfiles := topSketch.Profiles()
	bottomLoop := findProfile(t, bottomProfiles, want)
	topLoop := findProfile(t, topProfiles, want)
	if bottom.embedded {
		// The module's search is fixed at lines=2 and has no embedded branch; an
		// embedded gear finds no loop and raises. The proof pins that the 6-curve
		// loop is absent and stops, because decad has no "raise" to build.
		if bottomLoop != nil || topLoop != nil {
			t.Fatalf("embedded profile unexpectedly closed a loop with flank-to-root lines:%s", describeProfiles(bottomProfiles))
		}
		proofkit3d.Unmodelled(t, "embedded profile: find_profile_by_curve_counts(nurbs=2, arcs=2, lines=2) finds no loop, "+
			"so loftTooth raises rather than building — helical does not support the embedded shape ([HELI-F-LOFT])")
		return nil
	}
	if bottomLoop == nil {
		t.Fatalf("bottom section has no tooth loop with lines=%d:%s", want.lines, describeProfiles(bottomProfiles))
	}
	if topLoop == nil {
		t.Fatalf("twisted section has no tooth loop with lines=%d:%s", want.lines, describeProfiles(topProfiles))
	}
	if len(bottomProfiles) != 1 || len(topProfiles) != 1 {
		t.Fatalf("a loft-ready section closes exactly one region, got %d and %d:%s%s",
			len(bottomProfiles), len(topProfiles), describeProfiles(bottomProfiles), describeProfiles(topProfiles))
	}

	// loftInput.loftSections.add(bottomToothProfile) then .add(topToothProfile):
	// bottom is the FROM section, top the TO section, in that order.
	proofkit.Step(t, "loft bottom → top")
	offset := loftAlignment(t, bottomLoop, topLoop, helix)
	body, err := doc.LoftContext(context.Background(), bottomSketch, bottomLoop, topSketch, topLoop, decad.WithLoftAlignment(offset))
	if err != nil {
		t.Fatalf("loft: %v", err)
	}
	return []*decad.Body{body}
}

// loftAlignment finds the index of the top loop's edge that corresponds to the
// bottom loop's first edge. The twisted section is the bottom one rotated by
// the helix angle about the anchor, so the bottom's first edge start, rotated,
// lands on exactly one top edge start.
func loftAlignment(t *testing.T, bottom, top *sketch.Profile, helix float64) int {
	t.Helper()
	if len(bottom.Outer) != len(top.Outer) {
		t.Fatalf("section loops have %d and %d edges; the loft needs one-to-one pairs", len(bottom.Outer), len(top.Outer))
	}
	start := bottom.Outer[0].Polyline[0]
	rx := start[0]*math.Cos(helix) - start[1]*math.Sin(helix)
	ry := start[0]*math.Sin(helix) + start[1]*math.Cos(helix)
	best, bestDist := -1, math.Inf(1)
	for i, e := range top.Outer {
		d := math.Hypot(e.Polyline[0][0]-rx, e.Polyline[0][1]-ry)
		if d < bestDist {
			best, bestDist = i, d
		}
	}
	if best < 0 || bestDist > 1e-6 {
		t.Fatalf("no top edge starts where the bottom's first edge start lands after a %.4f rad twist (nearest %.3e away)", helix, bestDist)
	}
	return best
}

// assertLoftTooth reads the lofted tooth against what the spec pins:
//
//   - it spans exactly the plane offset, Thickness, along the plane normal;
//   - its centroid sits at half the offset and, in the plane, on the bisector
//     of the twist: the ruled body is symmetric under a reflection across the
//     line at HelixAngle/2 combined with z → Thickness − z, so the centroid's
//     polar angle is HelixAngle/2, sign included. That reading is what tells a
//     left-hand helix from a right-hand one, which no volume or extent can.
//   - the centroid's radius is bounded by the tooth's own: no farther out than
//     the tip circle, and no closer in than the root circle drawn in by
//     cos(HelixAngle/2), which is where a straight ruling between a root point
//     and its twisted copy passes at mid-height. A twist large enough to drag
//     the ruled walls through the gear centre — the [SPUR-F-ROTATE-CONFIRM]
//     failure — fails here.
//   - the volume is the tooth's, bracketed against the straight prism of the
//     bottom loop's area. The true ruled body is mirror-symmetric in the sign
//     of the twist, but decad's facets are not (see loftCases): measured at
//     M=1, N=17, T=10, a right-hand twist reads 96.2% of the prism at 14.5°
//     and 82.4% at 45°, a left-hand one 101.7% and 98.0%. So the bracket is
//     loose, [50%, 105%], and the reading is logged rather than pinned.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("loftTooth leaves exactly one Tooth Body, got %d", len(bodies))
	}
	body := bodies[0]
	helix := p[keyHelixAngle]
	thickness := p[keyThickness]
	dims := toothDimensions(p)

	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	slack := 1e-6 + box.Bound.Base()
	if math.Abs(box.Min.Z) > slack || math.Abs(box.Max.Z-thickness) > slack {
		t.Fatalf("tooth spans z ∈ [%.6f, %.6f], want [0, %.6f] (the helix plane offset)", box.Min.Z, box.Max.Z, thickness)
	}
	radial := math.Max(math.Hypot(box.Min.X, box.Min.Y), math.Hypot(box.Max.X, box.Max.Y))
	if radial > dims.Tip*math.Sqrt2+slack {
		t.Fatalf("tooth reaches %.4f mm from the axis, beyond the tip circle box of %.4f", radial, dims.Tip*math.Sqrt2)
	}

	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	c := centroid.Value
	cSlack := 1e-6 + centroid.Bound.Base()
	if math.Abs(c.Z-thickness/2) > cSlack {
		t.Fatalf("centroid z = %.6f, want %.6f (half the offset)", c.Z, thickness/2)
	}
	radius := math.Hypot(c.X, c.Y)
	innermost := dims.Root * math.Cos(helix/2)
	if radius < innermost-cSlack || radius > dims.Tip+cSlack {
		t.Fatalf("centroid radius %.4f is outside [%.4f, %.4f] (root·cos(HelixAngle/2) to tip): the ruled tooth does not sit where the profile does",
			radius, innermost, dims.Tip)
	}
	// Turn the reading back out of loftFrame, then compare on the circle.
	gotAngle := wrapAngle(math.Atan2(c.Y, c.X) - loftFrame)
	wantAngle := helix / 2
	angleSlack := math.Asin(math.Min(1, cSlack/radius)) + 1e-9
	if math.Abs(wrapAngle(gotAngle-wantAngle)) > angleSlack {
		t.Fatalf("centroid polar angle %.6f rad, want HelixAngle/2 = %.6f rad (twist sign %+.0f): the loft twists by the wrong amount or the wrong way",
			gotAngle, wantAngle, math.Copysign(1, helix))
	}
	if helix != 0 && math.Signbit(gotAngle) != math.Signbit(helix) {
		t.Fatalf("centroid polar angle %.6f has the opposite sign to the helix angle %.6f: the hand is flipped", gotAngle, helix)
	}

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	prism := prismVolume(t, doc, p, thickness)
	got := volume.Value.Base()
	lo, hi := 0.5*prism, 1.05*prism+volume.Bound.Base()
	if got < lo || got > hi {
		t.Fatalf("lofted tooth volume %.4f mm³ is outside [%.4f, %.4f], the bracket around the straight prism %.4f mm³ of the same loop",
			got, lo, hi, prism)
	}
	t.Logf("helix %+.1f°: volume %.4f mm³ (%.1f%% of the straight prism), centroid angle %.4f°",
		helix*180/math.Pi, got, 100*got/prism, gotAngle*180/math.Pi)
}

// prismVolume is the bottom tooth loop's area times the thickness: the volume
// the spur extrude would have made from the same loop, drawn again here so the
// assertion measures against a fresh solve rather than a number carried over
// from the build.
func prismVolume(t *testing.T, _ *decad.Document, p map[string]float64, thickness float64) float64 {
	t.Helper()
	steps := int(p[keyInvoluteSteps])
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("create reference sketch: %v", err)
	}
	drawToothProfile(t, s, p, loftFrame, 0, true)
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve reference sketch: %v", err)
	}
	loop := findProfile(t, s.Profiles(), loftLoopCounts(steps))
	if loop == nil {
		t.Fatalf("reference sketch has no tooth loop")
	}
	return loop.Area * thickness
}

// wrapAngle reduces an angle into (-π, π].
func wrapAngle(a float64) float64 {
	return math.Atan2(math.Sin(a), math.Cos(a))
}

// What this proof does not build, and why, recorded beside the nearest thing
// it does build.
//
// The helix construction plane (step H7 of the step list) is not a step of its
// own here: a plane leaves no body for proofkit3d to gate. It is built inside
// stepLoftTooth as the World offset plane the twisted section is drawn on, and
// assertLoftTooth reads the offset back as the lofted body's extent along the
// plane normal, which is exactly what the plane exists to fix.
//
// The body extrude, the circular pattern and combine, the root fillets, the
// bore and the completed-gear chamfer (step H10) run spur's inherited code
// with no helical override, so they are spur's proof to carry, and
// proof/spurgear carries them. They are also out of decad's reach for the
// helical body: the lofted tooth and the extruded disc share the root arc's
// edge and lie in the same two cap planes, which is the coplanar, grazing-edge
// contact decad's booleans refuse (BooleanUnsupportedContact), and the fillet
// and chamfer verbs take a straight prism receiver, which a loft is not. The
// one helical contribution to those steps, the cos(HelixAngle) factor in the
// FilletRadius expression, is a parameter expression Fusion evaluates, and no
// engine here evaluates Fusion expressions.
//
// The SketchOnly path (spur step 6) stops before buildTooth, so nothing in it
// is helical geometry; what helical adds to it — the twisted sketch drawn but
// left hidden, the plane left lit — is visibility state the sketch engine does
// not model.

// toothDimensions derives the circle radii for a case, in millimetres.
func toothDimensions(p map[string]float64) involute.Dimensions {
	return involute.Derive(p[keyModule], p[keyToothNumber], p[keyPressureAngle])
}
