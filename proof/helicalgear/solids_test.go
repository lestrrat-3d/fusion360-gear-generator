package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// solidCases sweeps what the loft itself depends on: the twist it is asked to
// carry, and the section spacing.
//
// The helix angle appears at both signs at every magnitude, because a left-hand
// helix is the same build with the twist the other way and the proof has to see
// that the hand survives. Two thicknesses at one helix angle are here to hold
// the spec's statement that nothing rescales the twist: the angle between the
// two sections is the Helix Angle itself, not a lead angle, so doubling the
// Thickness must leave the measured twist alone.
//
// The spec enforces no range on the helix angle and asserts none, and says the
// proof's own bound belongs here, measured per sign. It is this, on the m1 z17
// tooth at 10 mm thickness: a right-hand twist builds and verifies out to +91
// degrees and is refused at +92; a left-hand one runs to -170 and is refused at
// -175. Both refusals are decad's crossing audit reporting that the ruled walls
// of a single tooth touch where no shared vertex says they should, which is the
// evaluator declining to return a solid it cannot stand behind. The bound is a
// property of that audit and of a one-tooth section lofted alone, not of the
// gear, and it is not symmetric — which is why it is quoted per sign.
//
// The extreme cases below stop short of the refusals for a second reason: past
// about 170 degrees the top cap straddles the angular seam at +/-pi and
// capBearing can no longer read a bearing from it. The two embedded cases are
// unmodelled for a reason of the implementation's own, stated in the build.
var solidCases = []proofkit3d.Case{
	{Name: "default_m1_z17_helix_plus14_5_t10", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "left_hand_m1_z17_helix_minus14_5_t10", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "twist_is_not_rescaled_by_thickness_t30", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 30,
	}},
	{Name: "thin_body_t2", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 2,
	}},
	{Name: "quarter_turn_plus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(90), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "quarter_turn_minus90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-90), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "right_hand_bound_plus91", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(91), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "left_hand_bound_minus150", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": radians(-150), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "helix_zero_is_a_straight_prism", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": radians(20),
		"helixAngle": 0, "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "coarse_m3_z15", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "fine_m0_5_z40", Params: map[string]float64{
		"module": 0.5, "toothNumber": 40, "pressureAngle": radians(20),
		"helixAngle": radians(-14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "embedded_by_tooth_count_z45", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": radians(20),
		"helixAngle": radians(14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
	{Name: "embedded_by_pressure_angle_z30_pa25", Params: map[string]float64{
		"module": 1, "toothNumber": 30, "pressureAngle": radians(25),
		"helixAngle": radians(14.5), "involuteSteps": 15, "arcChords": 6, "thickness": 10,
	}},
}

// stepLoftTooth lofts the bottom Gear Profile tooth loop to the twisted top one
// and returns the single body it leaves behind.
//
// TWO SUBSTITUTIONS, and what each costs.
//
// First, the flanks. decad pairs a loft's sections segment by segment and
// refuses any pairing that is not two lines, two arcs or two circles, so the
// fitted spline Fusion draws each flank with has nothing it can be paired
// against. The flank is therefore chorded: the same involute samples the spline
// is fitted through, joined by straight segments. What this costs is the
// curvature between samples; the samples themselves, and so the flank's width
// at every one of them, are the real involute.
//
// Second, the tooth-top and root arcs. decad accepts an arc pair, and an
// earlier draft used one, but the bound it certifies on the resulting volume is
// wider than the default relative tolerance at every gear size tried, so the
// document verifies as suspect and the gate refuses it. Chording those two arcs
// as well leaves every pair a line pair, the volume exact to about 1e-15 mm3,
// and the document sound. What it costs is the tip and root faces: they arrive
// faceted, and the volume runs about 0.4 percent under the arc-bounded tooth.
// What survives is everything this step is here to pin — the section spacing,
// the twist and its hand, and that the loft closes into one solid lump.
//
// Neither substitution touches what makes helical helical, which is that the
// two sections are the same tooth at two angles.
func stepLoftTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	toothNumber := params["toothNumber"]
	dimensions := involute.Derive(params["module"], toothNumber, params["pressureAngle"])
	if dimensions.Embedded() {
		proofkit3d.Unmodelled(t, "loftTooth searches both sections at a fixed nurbs=2, arcs=2, "+
			"lines=2 and never reads ctx.toothProfileIsEmbedded, so the 4-curve embedded loop "+
			"stepTwistedGearProfileSketch draws is a loop this feature cannot find at all")
	}
	steps := int(params["involuteSteps"])
	chords := int(params["arcChords"])
	thickness := params["thickness"]
	angle := params["helixAngle"]

	world := sketch.NewWorld()

	// The bottom section is spur's own Gear Profile tooth, drawn at angle 0 by
	// the inherited buildSketches, and it goes into the loft FIRST.
	bottom := loftSection(t, world, world.XY(), dimensions, toothNumber, steps, chords, 0)

	// Step H7's offset plane. helicalPlaneOffset() returns the FULL Thickness
	// for helical, where herringbone re-points the same hook at half of it so
	// its mirror plane lands mid-body; that is the whole reason the offset is
	// its own overridable method. What the plane is in Fusion — a
	// ConstructionPlane created on the gear's component, left with its light
	// bulb on after the build — has no counterpart here, and neither does the
	// fact that the offset is a numeric snapshot of Thickness rather than a live
	// reference. The one consequence that does reach the geometry is the
	// distance itself, and assertLoftTooth measures it back off the solid.
	plane, err := world.CreateOffsetPlane(world.XY(), thickness)
	if err != nil {
		t.Fatalf("offset plane at %g mm: %v", thickness, err)
	}
	top := loftSection(t, world, plane, dimensions, toothNumber, steps, chords, angle)

	bottomProfile, topProfile := onlyProfile(t, bottom, "bottom"), onlyProfile(t, top, "top")
	// Bottom section first, then top. The order is the spec's, and it is not
	// cosmetic: the ruled walls are built outward from the FROM section, so
	// swapping the two builds a different solid.
	body, err := doc.Loft(bottom, bottomProfile, top, topProfile)
	if err != nil {
		t.Fatalf("loft bottom section to top section: %v", err)
	}
	requireSectionOrderMatters(t, bottom, bottomProfile, top, topProfile, body, angle)
	return []*decad.Body{body}
}

// requireSectionOrderMatters lofts the same two sections the other way round, in
// a document of its own that no gate ever sees, and holds the result against the
// one the step returned.
//
// The spec says to add the bottom section first and then the top, and a reader
// has no way to tell from the finished body that it was done that way: the
// centroid of the reversed solid sits within about 0.01 mm of the right one, and
// the twist and its sign come back identical. The volume is where the two part
// company, so that is what this reads. It also says what the swap really does:
// at +14.5 degrees the reversed volume is the volume the correctly-ordered loft
// gives at -14.5, so building the sections in the wrong order is the same
// mistake as building the other hand of helix.
func requireSectionOrderMatters(t *testing.T, bottom *sketch.Sketch, bottomProfile *sketch.Profile,
	top *sketch.Sketch, topProfile *sketch.Profile, body *decad.Body, angle float64) {
	if angle == 0 {
		// With no twist the two sections are congruent and the order genuinely
		// does not matter, so there is nothing here to tell apart.
		return
	}
	reversed, err := decad.New().Loft(top, topProfile, bottom, bottomProfile)
	if err != nil {
		// The reversed loft is the opposite hand of twist, and the evaluator's
		// bound is not symmetric, so past about -170 degrees the control itself
		// is refused. The case still proves everything else; it just cannot show
		// the section order apart here, and saying so is better than passing
		// quietly.
		t.Logf("section order not checked at this angle: the reversed control is refused — %v", err)
		return
	}
	ordered, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	swapped, err := reversed.Volume()
	if err != nil {
		t.Fatalf("control volume: %v", err)
	}
	gap := math.Abs(swapped.Value.Mag()-ordered.Value.Mag()) / ordered.Value.Mag()
	if gap < 1e-3 {
		t.Errorf("the reversed section order gives the same volume to within %.3e, so this "+
			"proof cannot tell the two orders apart and the spec's ordering is unchecked", gap)
	}
	t.Logf("section order: bottom-first %.6f mm3, top-first %.6f mm3, %.2f%% apart",
		ordered.Value.Mag(), swapped.Value.Mag(), 100*gap)
}

// assertLoftTooth measures the twist and the spacing back off the solid rather
// than off the numbers that built it.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("loft left %d bodies, expected the one Tooth Body", len(bodies))
	}
	body := bodies[0]
	thickness := params["thickness"]
	angle := params["helixAngle"]

	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	// The sections sit a full Thickness apart. A half-thickness offset — which
	// is what herringbone's override of the same hook returns — would show up
	// here and nowhere else.
	if math.Abs(box.Min.Z) > 1e-9 {
		t.Errorf("the body starts at z=%.6f mm, not on the base plane", box.Min.Z)
	}
	if got := box.Max.Z - box.Min.Z; math.Abs(got-thickness) > 1e-9 {
		t.Errorf("the body spans %.6f mm along the axis, Thickness is %.6f mm", got, thickness)
	}

	// The twist, read off the built solid. Each cap's vertices span an angular
	// wedge about the gear axis and the tooth is symmetric within it, so the
	// wedge's bisector is where that section points. The difference between the
	// two bisectors is the twist the loft actually carries, and it carries the
	// sign with it.
	bottomAim := capBearing(t, body, box.Min.Z, "bottom")
	topAim := capBearing(t, body, box.Max.Z, "top")
	twist := wrapPi(topAim - bottomAim)
	if math.Abs(twist-angle) > 1e-6 {
		t.Errorf("the loft twists by %.6f rad, the helix angle is %.6f rad", twist, angle)
	}
	t.Logf("thickness=%g twist=%.6f rad (%.3f deg)", thickness, twist, twist*180/math.Pi)

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	if volume.Value.Mag() <= 0 {
		t.Errorf("the tooth body has no volume")
	}
}

// loftSection draws one tooth outline, all of it in straight segments, and
// solves it. rotation is the angle the whole tooth is drawn at.
func loftSection(t *testing.T, world *sketch.World, plane *sketch.Plane, dimensions involute.Dimensions,
	toothNumber float64, steps, chords int, rotation float64) *sketch.Sketch {
	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create section sketch: %v", err)
	}
	// The constraint scheme is stepTwistedGearProfileSketch's subject, not this
	// step's, so these sections are located outright.
	place := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	bearing := func(p involute.Pt) float64 { return math.Atan2(p.Y, p.X) }
	onCircle := func(radius, at float64) *sketch.Point {
		return place(radius*math.Cos(at), radius*math.Sin(at))
	}

	left, right := involute.Flanks(dimensions.Base, dimensions.Tip, dimensions.Pitch, toothNumber, steps, rotation)
	if len(left) < 2 {
		t.Fatalf("involute sampling left %d usable points", len(left))
	}
	leftFootAt, rightFootAt := bearing(left[0]), bearing(right[0])

	// One walk around the loop: up the right flank, across the chorded tooth
	// top, down the left flank, and back along the chorded root.
	loop := []*sketch.Point{onCircle(dimensions.Root, rightFootAt)}
	for _, p := range right {
		loop = append(loop, place(p.X, p.Y))
	}
	loop = append(loop, chordArc(onCircle, dimensions.Tip,
		bearing(right[len(right)-1]), bearing(left[len(left)-1]), chords)...)
	for i := len(left) - 1; i >= 0; i-- {
		loop = append(loop, place(left[i].X, left[i].Y))
	}
	loop = append(loop, onCircle(dimensions.Root, leftFootAt))
	loop = append(loop, chordArc(onCircle, dimensions.Root, leftFootAt, rightFootAt, chords)...)
	for i := range loop {
		s.CreateLine(loop[i], loop[(i+1)%len(loop)])
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	return s
}

// chordArc returns the interior points of an arc from bearing a to bearing b,
// so a caller that already holds both ends can chord the span between them.
func chordArc(onCircle func(radius, at float64) *sketch.Point, radius, a, b float64, chords int) []*sketch.Point {
	out := make([]*sketch.Point, 0, chords-1)
	for i := 1; i < chords; i++ {
		out = append(out, onCircle(radius, a+(b-a)*float64(i)/float64(chords)))
	}
	return out
}

// onlyProfile returns the single closed region a section sketch encloses.
//
// In Fusion the section is found instead by curve count, and this proof cannot
// run that search: the tooth loop there is 2 splines, 2 arcs and 2 lines, and
// this one is all lines by the substitution above. What the count is on the
// sketch Fusion actually draws is asserted in stepTwistedGearProfileSketch.
func onlyProfile(t *testing.T, s *sketch.Sketch, which string) *sketch.Profile {
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("%s section encloses %d region(s), expected the tooth alone", which, len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatalf("%s section's region is not extrudable", which)
	}
	return profiles[0]
}

// capBearing returns the bisector of the angular wedge the body's vertices at
// height z span about the gear axis.
func capBearing(t *testing.T, body *decad.Body, z float64, which string) float64 {
	low, high := math.Inf(1), math.Inf(-1)
	found := 0
	for _, vertex := range body.Vertices() {
		position := vertex.Position().Value
		if math.Abs(position.Z-z) > 1e-6 {
			continue
		}
		at := math.Atan2(position.Y, position.X)
		low = math.Min(low, at)
		high = math.Max(high, at)
		found++
	}
	if found == 0 {
		t.Fatalf("no vertex on the %s cap at z=%.6f", which, z)
	}
	if high-low > math.Pi {
		t.Fatalf("the %s cap wraps the angular seam, so its bearing cannot be read this way", which)
	}
	return (low + high) / 2
}
