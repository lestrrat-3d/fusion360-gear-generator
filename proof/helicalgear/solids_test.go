package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// The twist this proof can still verify, measured rather than assumed, and
// measured separately for each sign because the two do not behave alike.
//
// decad rules the wall between a bottom segment and its top partner as two
// triangles, and it chooses that pair relative to the FROM section. Past a
// large enough twist the choice makes two walls meet off their shared vertex
// and the loft is refused as degenerate. Swept a degree at a time on the
// default module 1, 17 tooth, 10 mm gear at both 15 and 4 involute samples, the
// last twist that still lofts is +96 degrees, while the left-hand side goes all
// the way to -179 degrees without complaint.
//
// This is a property of the harness and of the substitute sections below, not
// of the gear: nothing in the spec clamps the helix angle, and what Fusion does
// at a large one is unverified. Past the bound the case is skipped, named, and
// not silently passed.
const (
	loftBoundRightHandDeg = 96
	loftBoundLeftHandDeg  = -179
)

// loftCases sweeps both hands of the helix, the sizes, and the two edges of the
// bound above.
//
// Every case that is not skipped has to leave a sound solid, so the table is
// where the loft's own limits are recorded: the right-hand twist at 120 degrees
// is past the bound and the left-hand twist at the same magnitude is not, which
// is the asymmetry the bound exists to state. The embedded case is here too,
// because a caller reaching for an embedded helical gear meets the loft, not
// the sketch.
var loftCases = []proofkit3d.Case{
	{Name: "default_right_hand_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(14.5),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "left_hand_-14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-14.5),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "untwisted_0deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": 0,
		"thickness": 10, "involuteSteps": 15}},
	{Name: "coarse_module_3_15teeth", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20), "helixAngle": deg(14.5),
		"thickness": 20, "involuteSteps": 15}},
	{Name: "fine_module_0.5_30teeth_left_hand", Params: map[string]float64{
		"module": 0.5, "toothNumber": 30, "pressureAngle": deg(20), "helixAngle": deg(-14.5),
		"thickness": 4, "involuteSteps": 15}},
	{Name: "axis_swap_right_hand_60deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(60),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "axis_swap_left_hand_-60deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-60),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "right_hand_at_the_bound_96deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(96),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "right_hand_past_the_bound_120deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(120),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "left_hand_-120deg_still_within_the_bound", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-120),
		"thickness": 10, "involuteSteps": 15}},
	{Name: "four_samples_left_hand", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-14.5),
		"thickness": 10, "involuteSteps": 4}},
	{Name: "embedded_by_tooth_count_45teeth", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": deg(20), "helixAngle": deg(14.5),
		"thickness": 10, "involuteSteps": 15}},
}

// The offset construction plane has no proof of its own, and this is where the
// thing it exists for is checked. A bare ConstructionPlane closes no sketch and
// bounds no body, so neither harness has a gate to put it through; what can be
// checked is the distance it puts between the two sections, and that is read
// back off the finished solid in assertLoftTooth, which requires the tooth body
// to span exactly the base plane to the full Thickness. A plane offset by half
// the thickness, the way herringbone re-points the hook, would fail there.

// sections draws the two loft sections: the bottom Gear Profile the inherited
// spur pass leaves at angle 0, and the Twisted Gear Profile on a plane offset
// from it by the full Thickness — which is what helicalPlaneOffset() returns.
//
// Both are drawn by the same tooth generator with the same constraint scheme;
// only the angle differs. They are solved and no more: the substitute's own
// constraint scheme is proved in stepTwistedGearProfile, whose table carries it
// as two cases of its own, so repeating that gate on every loft case would only
// buy running time.
func sections(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()

	basePlane := world.XY()
	bottomSketch, err := world.CreateSketch(basePlane)
	if err != nil {
		t.Fatalf("Gear Profile sketch: %v", err)
	}
	bottom := g
	bottom.helixAngle = 0
	drawSection(t, bottomSketch, bottom, chordedProfile)
	solve(t, bottomSketch)

	helixPlane, err := world.CreateOffsetPlane(basePlane, g.thickness)
	if err != nil {
		t.Fatalf("helix plane offset by Thickness: %v", err)
	}
	topSketch, err := world.CreateSketch(helixPlane)
	if err != nil {
		t.Fatalf("Twisted Gear Profile sketch: %v", err)
	}
	drawSection(t, topSketch, g, chordedProfile)
	solve(t, topSketch)

	return bottomSketch, toothProfile(t, bottomSketch), topSketch, toothProfile(t, topSketch)
}

// stepLoftTooth lofts the bottom tooth loop to the twisted top one.
//
// This is buildTooth, which helical overrides to call loftTooth instead of
// extruding. The bottom section is added first and the top second; the order is
// pinned, and what it buys is checked in assertLoftTooth.
func stepLoftTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := gearOf(params)

	if g.dims().Embedded() {
		proofkit3d.Unmodelled(t, "the flank starts inside the root circle, so neither section "+
			"carries the flank-to-root lines [HELI-F-LOFT]'s fixed nurbs=2, arcs=2, lines=2 "+
			"search needs; helical has no embedded branch and cannot build this gear")
	}
	if inDegrees := g.helixAngle * 180 / math.Pi; inDegrees > loftBoundRightHandDeg ||
		inDegrees < loftBoundLeftHandDeg {
		proofkit3d.Unmodelled(t, "a %.1f degree twist is past this proof's measured loft bound "+
			"(%+d degrees right-hand, %+d degrees left-hand), where decad refuses the ruled walls "+
			"as degenerate; the gear itself is unclamped and unverified there",
			inDegrees, loftBoundRightHandDeg, loftBoundLeftHandDeg)
	}

	bottomSketch, bottomTooth, topSketch, topTooth := sections(t, g)
	if len(bottomTooth.Entities) != len(topTooth.Entities) {
		t.Fatalf("the two loft sections carry %d and %d curves; a loft pairs them one for one",
			len(bottomTooth.Entities), len(topTooth.Entities))
	}

	body, err := doc.Loft(bottomSketch, bottomTooth, topSketch, topTooth)
	if err != nil {
		t.Fatalf("loft the bottom tooth loop to the twisted top loop: %v", err)
	}
	return []*decad.Body{body}
}

// assertLoftTooth measures the Tooth Body the loft leaves behind.
//
// Three things are pinned. The body spans exactly the base plane to the helix
// plane, which is helicalPlaneOffset() returning the full Thickness rather than
// some fraction of it. Its centroid sits at half the helix angle, sign
// included: the solid is carried onto itself by a rotation of the helix angle
// together with a flip in z, so its centroid can lie nowhere else, and reading
// it back is how the twist and its hand are checked. And the loft section order
// is checked for what it actually buys.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := gearOf(params)
	body := bodies[0]

	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("tooth body bounds: %v", err)
	}
	if math.Abs(bounds.Min.Z) > 1e-9 || math.Abs(bounds.Max.Z-g.thickness) > 1e-9 {
		t.Errorf("the tooth body spans z %.9f to %.9f, not the base plane to the helix plane at "+
			"the full Thickness %.9f", bounds.Min.Z, bounds.Max.Z, g.thickness)
	}

	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("tooth body centroid: %v", err)
	}
	wantAngle := g.helixAngle / 2
	gotAngle := math.Atan2(centroid.Value.Y, centroid.Value.X)
	if math.Abs(gotAngle-wantAngle) > 1e-6 {
		t.Errorf("the tooth body's centroid sits at %.9f rad, not at half the helix angle "+
			"%.9f rad; the twist or its hand is wrong", gotAngle, wantAngle)
	}

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("tooth body volume: %v", err)
	}
	// Swapping the sections makes the top the FROM section, which puts the
	// comparison on the right-hand side of the bound whichever hand the case
	// itself is, so it can only be run inside the tighter of the two.
	if math.Abs(g.helixAngle*180/math.Pi) > loftBoundRightHandDeg {
		t.Logf("the section-order comparison is not run at %.1f degrees: swapping the sections "+
			"puts the loft past its right-hand bound of %+d degrees",
			g.helixAngle*180/math.Pi, loftBoundRightHandDeg)
		return
	}
	assertSectionOrder(t, doc, g, volume.Value.Base())
}

// assertSectionOrder checks what adding the bottom section first buys.
//
// Adding the top section first also lofts a valid solid, and it does not flip
// the hand: the same twist, with the same sign, comes back off the centroid
// either way. What changes is the ruled walls, which decad builds outward from
// the FROM section, and with them the volume. So the order is silent in the one
// reading a caller is most likely to check, which is why the spec pins it
// rather than leaving it to the implementation.
//
// The prism case is the control. At a zero helix angle the two sections are
// congruent and parallel, the walls are flat either way, and the volume is
// exactly the section area times the thickness whichever section leads.
func assertSectionOrder(t *testing.T, doc *decad.Document, g gear, volume float64) {
	t.Helper()
	bottomSketch, bottomTooth, topSketch, topTooth := sections(t, g)
	prism := bottomTooth.Area * g.thickness

	swapped, err := doc.Loft(topSketch, topTooth, bottomSketch, bottomTooth)
	if err != nil {
		t.Fatalf("loft with the sections in the other order: %v", err)
	}
	swappedVolume, err := swapped.Volume()
	if err != nil {
		t.Fatalf("swapped tooth body volume: %v", err)
	}
	swappedCentroid, err := swapped.Centroid()
	if err != nil {
		t.Fatalf("swapped tooth body centroid: %v", err)
	}
	gotAngle := math.Atan2(swappedCentroid.Value.Y, swappedCentroid.Value.X)
	if math.Abs(gotAngle-g.helixAngle/2) > 1e-6 {
		t.Errorf("adding the top section first moved the twist to %.9f rad; the hand is supposed "+
			"to be the same either way", gotAngle)
	}

	drift := math.Abs(swappedVolume.Value.Base()-volume) / volume
	if g.helixAngle == 0 {
		if math.Abs(volume-prism) > 1e-6 || drift > 1e-12 {
			t.Errorf("an untwisted loft is a prism: volume %.9f and swapped volume %.9f should "+
				"both be the section area times the thickness, %.9f",
				volume, swappedVolume.Value.Base(), prism)
		}
		return
	}
	if drift < 1e-6 {
		t.Errorf("swapping the loft sections left the volume unchanged at %.9f, so this case "+
			"cannot show why the order is pinned", volume)
	}
	if volume <= 0 || volume > prism*1.1 {
		t.Errorf("the lofted tooth measures %.6f against a prism of %.6f, which is not a twisted "+
			"tooth", volume, prism)
	}
	t.Logf("loft section order: bottom first %.6f, top first %.6f, %.2f%% apart on a prism of "+
		"%.6f", volume, swappedVolume.Value.Base(), 100*drift, prism)
}
