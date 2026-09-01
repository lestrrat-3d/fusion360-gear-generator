package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// What the solid steps model, and what that costs.
//
// The tooth's real cross-section is six curves: two involute splines, two
// flank-to-root lines and two arcs. decad lofts a pair of sections only when
// every corresponding segment is the same kind and neither is free-form, so a
// spline pair is refused outright, and a circular pair is chorded — which
// leaves the volume reading's bound outside the default tolerance, a
// diagnostic proofkit3d's solid gate does not admit for volume.
//
// So the sections here are the tooth's six corners joined by six straight
// segments: each flank is chorded, and so is each arc. What survives the
// substitution is what these two steps are for — the section's six-segment
// structure, its corners taken from the same involute samples the sketch draws
// its splines through, the axial extent the offset plane sets, and the twist
// the second section carries. What does not survive is the flank's shape: a
// chord is inside the involute, so these bodies are slightly thinner than the
// gear's, and no reading here says anything about flank form.
//
// The curve COUNT the loft matches on is not lost with it. It is proven in
// stepTwistedGearProfile, on the sketch that actually draws all six curves.

// helixPlaneCases sweeps the thickness the helix plane is offset by. The
// offset is helicalPlaneOffset()'s value, which helical returns as the full
// Thickness, so every case here is a different Thickness.
var helixPlaneCases = []proofkit3d.Case{
	{Name: "thin-1mm", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15, "thickness": 1}},
	{Name: "default-10mm", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15, "thickness": 10}},
	{Name: "coarse-m3-z15-20mm", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": rad(20), "involuteSteps": 15, "thickness": 20}},
	{Name: "deep-50mm", Params: map[string]float64{
		"module": 2, "toothNumber": 20, "pressureAngle": rad(20), "involuteSteps": 15, "thickness": 50}},
}

// loftToothCases sweeps the twist, on both sides of zero and out to the ends
// this proof can verify.
//
// The spec enforces no range on the Helix Angle and asserts none: what Fusion
// does at a large angle is unverified. The bound below is this proof's own, and
// it is measured, not chosen — see verifiedPositiveTwist.
var loftToothCases = []proofkit3d.Case{
	{Name: "default-right-hand-plus14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 10, "helixAngle": rad(14.5)}},
	{Name: "left-hand-minus14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 10, "helixAngle": rad(-14.5)}},
	{Name: "untwisted-zero", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 10, "helixAngle": 0}},
	{Name: "right-hand-plus30", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 5, "helixAngle": rad(30)}},
	{Name: "left-hand-minus30", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 5, "helixAngle": rad(-30)}},
	{Name: "coarse-m2-z20-plus70", Params: map[string]float64{
		"module": 2, "toothNumber": 20, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 10, "helixAngle": rad(70)}},
	{Name: "fine-m1-z12-minus120", Params: map[string]float64{
		"module": 1, "toothNumber": 12, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 5, "helixAngle": rad(-120)}},
	{Name: "coarse-m3-z15-minus170", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 20, "helixAngle": rad(-170)}},
	{Name: "beyond-positive-bound-plus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": rad(20), "involuteSteps": 15,
		"thickness": 10, "helixAngle": rad(90)}},
}

// verifiedPositiveTwist is where this proof stops verifying a right-hand twist,
// in degrees.
//
// It is a property of the modelling above and of decad's own loft audit, not of
// the gear, and it is not symmetric about zero. Sweeping one degree at a time
// with the sections in this file, the first refusal on the positive side is
// +74 deg at m2 z20, +76 at m1 z17, +77 at m3 z15 and +78 at m1 z12, every one
// of them "loft triangles ... share no recorded vertex, but make contact" —
// decad proving the ruled walls touch where the record says they do not. On the
// negative side there is no refusal until -180 deg, where the two sections face
// each other exactly. So the table's positive end is +70 and its negative end
// is -170, and the case past the bound is skipped rather than dropped, because
// where the proof stops is itself a thing to record.
const verifiedPositiveTwist = 73

// buildToothSection draws one loft section: the tooth's six corners joined by
// six straight segments, at the requested angle.
//
// Every point is grounded. The sections are stand-ins for the loft's sake and
// the constraint scheme that places these corners in Fusion is proven in
// stepTwistedGearProfile, so re-deriving it here would prove nothing twice and
// would tie the solid steps to the sketch engine's solver.
func buildToothSection(t *testing.T, s *sketch.Sketch, p params, angle float64) {
	corners := toothOutline(p, angle)
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c.X, c.Y)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve section at %.3f deg: %v", deg(angle), err)
	}
}

// loftSections draws the bottom section on the base plane and the top section
// on the helix plane, and lofts between them.
//
// bottomFirst is the order the two sections are added in. loftTooth adds the
// bottom section before the top one; the reversed order is built only by
// assertLoftTooth, to read what that order is worth.
func loftSections(t *testing.T, doc *decad.Document, p params, twist float64, bottomFirst bool) (*decad.Body, error) {
	t.Helper()
	world := sketch.NewWorld()

	bottom, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("gear profile sketch: %v", err)
	}
	buildToothSection(t, bottom, p, 0)

	// [HELI-F-TWIST-PLANE]: the twisted profile is drawn on a construction plane
	// offset from the gear's base plane by helicalPlaneOffset(), which helical
	// returns as the full Thickness.
	helixPlane, err := world.CreateOffsetPlane(world.XY(), p.thickness())
	if err != nil {
		t.Fatalf("helix plane at offset %.3f: %v", p.thickness(), err)
	}
	top, err := world.CreateSketch(helixPlane)
	if err != nil {
		t.Fatalf("twisted gear profile sketch: %v", err)
	}
	buildToothSection(t, top, p, twist)

	bottomProfile := soleProfile(t, bottom, "Gear Profile")
	topProfile := soleProfile(t, top, "Twisted Gear Profile")
	if bottomFirst {
		return doc.LoftContext(t.Context(), bottom, bottomProfile, top, topProfile)
	}
	return doc.LoftContext(t.Context(), top, topProfile, bottom, bottomProfile)
}

// soleProfile returns the one region a section sketch closes, after checking it
// carries the six segments the tooth loop has curves.
func soleProfile(t *testing.T, s *sketch.Sketch, name string) *sketch.Profile {
	t.Helper()
	all := s.Profiles()
	if len(all) != 1 {
		t.Fatalf("%s closed %d regions, want exactly the tooth", name, len(all))
	}
	if got := len(all[0].Outer); got != 6 {
		t.Fatalf("%s tooth loop has %d segments, want the six the tooth's curves stand in for", name, got)
	}
	return all[0]
}

// stepHelixPlane creates the helix construction plane and proves the offset it
// is placed at.
//
// The plane itself measures nothing, so the step is proven through what is
// built on it: an untwisted section on the base plane lofted to the same
// section on the helix plane, which is a straight prism exactly as deep as the
// offset. helicalPlaneOffset() returns the full Thickness for helical — this is
// the hook herringbone re-points at half the thickness — so the body's axial
// extent IS the offset under test.
func stepHelixPlane(t *testing.T, doc *decad.Document, raw map[string]float64) []*decad.Body {
	p := params(raw)
	body, err := loftSections(t, doc, p, 0, true)
	if err != nil {
		t.Fatalf("prism on the helix plane at offset %.3f mm: %v", p.thickness(), err)
	}
	return []*decad.Body{body}
}

// assertHelixPlane checks the body spans exactly the offset, and no more.
func assertHelixPlane(t *testing.T, _ *decad.Document, bodies []*decad.Body, raw map[string]float64) {
	p := params(raw)
	if len(bodies) != 1 {
		t.Fatalf("built %d bodies, want one", len(bodies))
	}
	body := bodies[0]

	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	requireClose(t, "base plane end", box.Min.Z, 0, 1e-9)
	requireClose(t, "helix plane end", box.Max.Z, p.thickness(), 1e-9)

	// The prism law is the independent check on the offset: a plane placed at
	// anything other than Thickness gives a volume this equality misses.
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	want := outlineArea(toothOutline(p, 0)) * p.thickness()
	requireClose(t, "prism volume", volume.Value.Mag(), want, 1e-9*want)

	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	requireClose(t, "untwisted centroid angle", deg(polarAngle(centroid.Value.X, centroid.Value.Y)), 0, 1e-6)
}

// stepLoftTooth lofts the bottom Gear Profile tooth loop to the top twisted
// tooth loop, which is what helical builds instead of spur's extrude.
//
// The bottom section is added first and the top second ([HELI-F-LOFT]).
func stepLoftTooth(t *testing.T, doc *decad.Document, raw map[string]float64) []*decad.Body {
	p := params(raw)
	twist := p.helixAngle()
	if deg(twist) > verifiedPositiveTwist {
		proofkit3d.Unmodelled(t, "a right-hand twist of %.1f deg is past the +%d deg this proof verifies: "+
			"decad's loft audit refuses the chorded sections above it, proving the ruled walls make contact "+
			"where the record has them apart. The negative side has no such bound short of -180 deg.",
			deg(twist), verifiedPositiveTwist)
	}
	body, err := loftSections(t, doc, p, twist, true)
	if err != nil {
		t.Fatalf("loft at %.3f deg over %.3f mm: %v", deg(twist), p.thickness(), err)
	}
	return []*decad.Body{body}
}

// assertLoftTooth reads the twist back off the solid, and reads what the
// section order is worth.
func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, raw map[string]float64) {
	p := params(raw)
	if len(bodies) != 1 {
		t.Fatalf("lofted %d bodies, want the one Tooth Body", len(bodies))
	}
	body := bodies[0]

	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	requireClose(t, "base plane end", box.Min.Z, 0, 1e-9)
	requireClose(t, "helix plane end", box.Max.Z, p.thickness(), 1e-9)

	// The twist between the two sections IS the Helix Angle: no lead angle is
	// derived from it and Thickness does not enter. A tooth twisted uniformly
	// between its two ends is symmetric about the half-twist plane, so its
	// centroid sits at exactly half the helix angle from +X — which reads back
	// both the size of the twist and its hand. A scheme that rescaled the angle
	// by the thickness, or dropped its sign, still builds a sound solid and
	// fails here.
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	got := deg(polarAngle(centroid.Value.X, centroid.Value.Y))
	requireClose(t, "centroid angle (half the twist)", got, deg(p.helixAngle())/2, 1e-4)

	// [HELI-F-LOFT] adds the bottom section before the top, and the contract
	// manifest guards that order on the grounds that the other order lofts the
	// mirror-hand gear. The order IS observable — decad's ruled walls are
	// triangulated from the FROM section outwards, so reversing it moves every
	// wall's diagonal and changes the volume — but the hand is not what changes:
	// the reversed body carries the same twist, read the same way, to the last
	// digit the centroid is measured in. So the guard is worth keeping and its
	// stated reason is not what this proof sees.
	//
	// Reversing the sections reaches decad's refusal at the size a right-hand
	// twist of the same magnitude does — the reversed build IS the opposite
	// hand's wall set — so beyond the verified bound there is no reversed body
	// to compare against and the reading is left unmade rather than guessed.
	if math.Abs(deg(p.helixAngle())) > verifiedPositiveTwist {
		t.Logf("section order not read at %.1f deg: the reversed build reaches the +%d deg refusal from the other side",
			deg(p.helixAngle()), verifiedPositiveTwist)
		return
	}
	reversed, err := loftSections(t, decad.New(), p, p.helixAngle(), false)
	if err != nil {
		t.Fatalf("loft with the sections reversed at %.3f deg: %v", deg(p.helixAngle()), err)
	}
	reversedCentroid, err := reversed.Centroid()
	if err != nil {
		t.Fatalf("reversed centroid: %v", err)
	}
	reversedAngle := deg(polarAngle(reversedCentroid.Value.X, reversedCentroid.Value.Y))
	requireClose(t, "centroid angle with the sections reversed", reversedAngle, got, 1e-4)

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	reversedVolume, err := reversed.Volume()
	if err != nil {
		t.Fatalf("reversed volume: %v", err)
	}
	t.Logf("section order: bottom-first volume %.6f mm^3, top-first %.6f mm^3, both twisted %.4f deg",
		volume.Value.Mag(), reversedVolume.Value.Mag(), 2*got)
}

// requireClose fails the test when got is not within tol of want.
func requireClose(t *testing.T, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s is %.9f, want %.9f (tolerance %.1e)", what, got, want, tol)
	}
}

// Steps this proof does not reach, recorded next to the geometry they belong
// to rather than only in the step list.
//
//   - The chamfer helical triggers at the end of buildTooth is inherited whole
//     from spur, and it selects the tooth's cap face by an edge count.
//     [HELI-F-CHAMFER-COUNT] settled that count at 6 in a Fusion session and
//     says plainly what a design-time proof can establish about it: no harness
//     measures a Fusion cap face, so the provable fact is the sketch loop's
//     curve count, and the cap's edge count follows from it through a
//     one-curve-one-edge correspondence that is a Fusion behaviour rather than
//     a proof result. stepTwistedGearProfile proves the loop count, 6
//     non-embedded and 4 embedded. Nothing here proves the correspondence.
//   - The root fillet's transverse correction, cos(HelixAngle), is an
//     expression spliced into the FilletRadius parameter at registration time.
//     decad fillets only the lateral edges of a straight prism, so the fillet
//     helical's factor scales cannot be built on this lofted tooth at all, and
//     there is no substitute that would still be that fillet. The factor's one
//     geometric consequence a proof could reach — that cos is even, so a
//     left-hand and a right-hand gear of the same helix angle get the same
//     fillet radius while their teeth differ — is a property of the expression,
//     not of a body, and asserting it here would be asserting arithmetic.
//   - The dialog input, the HelixAngle parameter registration, the class and
//     hook surface, and the visibility of the helix plane and the twisted
//     sketch are Fusion-API surface with no geometry in them. They are pinned
//     by the contract manifest and the step list, and no harness sees them.
