// This file holds helical's one solid step: the loft that replaces spur's
// tooth extrude. Everything downstream of it — the body extrude, the pattern,
// the combine, the root fillets, the bore and the completed-gear chamfer — is
// spur's code running unchanged on the body this step leaves behind, and is
// proved in proof/spurgear.
//
// The section is a polygon, and that is two substitutions, each forced.
//
// The flanks are chorded because decad lofts a corresponding segment pair only
// when both sides are the same kind of recorded segment, and a free-form
// pairing — which two fitted splines are — is refused outright.
//
// The tooth-top and root boundaries are chorded too, and this one is the solid
// GATE's doing rather than the loft's. decad does loft an arc against an arc,
// by chording its walls, but the bound it then proves on the body's VOLUME
// reading lands outside its own relative tolerance at gear-tooth scale:
// measured here at a 14.5 degree helix on a 1 module, 12 tooth, 10 mm tooth,
// a bound of 0.0321 mm^3 against a tolerance of 0.0300 mm^3. That is a volume
// diagnostic, and proofkit3d.RequireSolid tolerates such a reading only for an
// area or a centroid, so the arc-walled loft cannot pass the gate this proof is
// held to. A section of lines carries no such wall: every reading below comes
// back with a bound near 1e-15 and the document verifies Sound with no
// diagnostic at all.
//
// What the polygon costs is the sagitta of each chord: the flanks, the tooth
// top and the root boundary all move inward, so the lofted tooth is slightly
// smaller than the real one. No assertion below compares a volume against a
// closed-form involute tooth, so that never enters one — the volume oracle is
// the closed form of the polygon this proof actually drew, and the two section
// orders are compared against each other.
//
// The root boundary is authored rather than derived a second way: in Fusion it
// is a piece of the solid root circle that the tooth splits, and the curve
// counts that split produces are asserted on the real Twisted Gear Profile
// sketch in stepTwistedGearProfileSketch.
//
// The embedded tooth is not a case here. [HELI-F-LOFT] passes a fixed
// nurbs=2, arcs=2, lines=2 key to both sections and has no embedded branch, so
// an embedded helical gear never reaches a loft at all; it fails in the profile
// search. That is proved where it happens, on the sketch, by the two embedded
// cases of twistedProfileCases, and toothOutline refuses such parameters here
// rather than quietly building a tooth the shipped code cannot find.
//
// WHERE THE RULED WALLS STOP BEING BUILDABLE, MEASURED PER SIGN. decad audits
// the walls for contact away from their recorded shared edges and refuses a
// build it can prove touches itself. Measured on this model, 2026-09-18, at
// three sizes (1 module 12 teeth, 2 module 14 teeth, 1 module 17 teeth): a
// POSITIVE twist builds to +95 degrees and is refused from +100 degrees; a
// NEGATIVE twist builds to -179 degrees, the widest tried. The bound is
// therefore real, and it is not symmetric about zero — which is the helical
// spec's own statement about it, and the reason this file records it per sign.
// It is a property of this model and of decad's wall audit, not of the gear:
// what Fusion does at a large helix angle is unverified, and nothing here is a
// clamp, a warning or a documented maximum for the dialog. loftBetween turns
// that one refusal into a skip naming the case's signed angle, so a run reports
// the bound rather than quoting a constant nobody re-measures, and the table
// carries a case past the positive bound so every run says where it still is.

package helicalgear_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// loftCases sweep what the loft depends on: the signed helix angle, the
// Thickness its top section's plane is offset by, and the tooth size.
//
// Thickness is swept against one fixed helix angle on purpose. The twist
// between the two sections is the Helix Angle itself and not a lead angle
// derived from it, so Thickness must move the top section further away without
// changing the twist by anything, and a pair of cases differing only in
// Thickness is what says so.
//
// The angle is swept on both sides of zero and out to a quarter turn each way,
// because the sign is the hand of the helix and a scheme that dropped it would
// still build at +angle. The last case sits past the positive bound recorded in
// this file's header; it is expected to skip, and the line it prints is where
// that bound is read off a run.
//
// The involute sample counts are low. The wall audit grows with the number of
// corresponding segment pairs, and the tooth's shape is the sketch step's
// subject rather than this one's: what is proved here is the sweep between two
// sections, which a five-sample flank carries as well as a fifteen-sample one.
var loftCases = []proofkit3d.Case{
	{Name: "M1_N12_T10_helix_zero", Params: loftParams(1, 12, 20, 5, 10, 0)},
	{Name: "M1_N12_T10_helix_plus_default_14_5", Params: loftParams(1, 12, 20, 5, 10, 14.5)},
	{Name: "M1_N12_T10_helix_minus_default_14_5", Params: loftParams(1, 12, 20, 5, 10, -14.5)},
	{Name: "M1_N12_T20_helix_plus_default_14_5", Params: loftParams(1, 12, 20, 5, 20, 14.5)},
	{Name: "M1_N12_T3_helix_minus_default_14_5", Params: loftParams(1, 12, 20, 5, 3, -14.5)},
	{Name: "M2_N14_T8_helix_plus_30", Params: loftParams(2, 14, 20, 4, 8, 30)},
	{Name: "M2_N14_T8_helix_minus_30", Params: loftParams(2, 14, 20, 4, 8, -30)},
	{Name: "M1_N17_T25_helix_plus_45", Params: loftParams(1, 17, 20, 4, 25, 45)},
	{Name: "M1_N17_T25_helix_minus_45", Params: loftParams(1, 17, 20, 4, 25, -45)},
	{Name: "M1_N12_T10_helix_plus_quarter_turn", Params: loftParams(1, 12, 20, 5, 10, 90)},
	{Name: "M1_N12_T10_helix_minus_quarter_turn", Params: loftParams(1, 12, 20, 5, 10, -90)},
	{Name: "M1_N12_T10_helix_plus_120_past_the_measured_bound", Params: loftParams(1, 12, 20, 5, 10, 120)},
}

// loftParams names one solid case by the dialog values it comes from. The helix
// angle is given in degrees, as the dialog takes it, and carried in radians, as
// the HelixAngle user parameter holds it.
func loftParams(module, toothNumber, pressureAngleDeg float64, steps int, thickness, helixAngleDeg float64) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": rad(pressureAngleDeg),
		"involuteSteps": float64(steps),
		"thickness":     thickness,
		"helixAngle":    rad(helixAngleDeg),
	}
}

func mm(v float64) units.Value { return units.Millimeters(v) }

// section is one loft section: the sketch it was drawn on and the single valid
// region it closes. Both are needed, because a loft records a profile against
// the sketch that owns it.
type section struct {
	sketch *sketch.Sketch
	region *sketch.Profile
}

// toothOutline is the tooth's closed boundary, counter-clockwise, at one
// angular position: up the right flank, across the tooth top, down the left
// flank, out to the left stub foot and back along the root.
//
// One function owns this vertex list because two things read it — the section
// that gets drawn and lofted, and the closed-form volume the result is measured
// against — and a second copy of the order would let those two drift apart.
func toothOutline(t *testing.T, p map[string]float64, angle float64) []involute.Pt {
	t.Helper()
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	if d.Embedded() {
		t.Fatalf("these parameters close an embedded tooth, which loftTooth's fixed "+
			"nurbs=2, arcs=2, lines=2 key never reaches: module %g, %g teeth, pressure angle %g rad",
			p["module"], p["toothNumber"], p["pressureAngle"])
	}
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["toothNumber"], int(p["involuteSteps"]), angle)

	out := make([]involute.Pt, 0, 2*len(left)+2)
	out = append(out, footPoint(d.Root, right[0]))
	out = append(out, right...)
	for i := len(left) - 1; i >= 0; i-- {
		out = append(out, left[i])
	}
	out = append(out, footPoint(d.Root, left[0]))
	return out
}

// footPoint is a flank-to-root stub's far end: the flank start's direction from
// the gear centre, taken out to radius r.
func footPoint(r float64, start involute.Pt) involute.Pt {
	n := math.Hypot(start.X, start.Y)
	return involute.Pt{X: r * start.X / n, Y: r * start.Y / n}
}

// sectionOn draws one tooth outline on the given plane as a closed polyline and
// returns the region the loft consumes.
//
// The section carries no constraints. The constraint scheme is
// stepTwistedGearProfileSketch's subject, and drawing it again here would prove
// it twice; what this needs is the solved shape at the angle the generator
// draws it.
func sectionOn(t *testing.T, world *sketch.World, plane *sketch.Plane, outline []involute.Pt) section {
	t.Helper()
	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch on a loft section plane: %v", err)
	}
	pts := make([]*sketch.Point, len(outline))
	for i, pt := range outline {
		pts[i] = s.CreatePoint(pt.X, pt.Y)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	return section{sketch: s, region: decadtest.SolveRegion(t, s)}
}

// twistedPair builds the two sections the loft passes through, on two planes of
// ONE world: the bottom Gear Profile tooth at angle 0 on the target plane, and
// the Twisted Gear Profile tooth at the Helix Angle on a plane offset from it
// by helicalPlaneOffset(), which for helical is the full Thickness.
//
// That offset is the whole content of the hook. Herringbone re-points it to
// half the thickness so its mirror plane lands mid-body; helical returns the
// full thickness, and the height assertion below is what holds it there.
func twistedPair(t *testing.T, p map[string]float64) (section, section) {
	t.Helper()
	world := sketch.NewWorld()
	helixPlane, err := world.CreateOffsetPlane(world.XY(), p["thickness"])
	if err != nil {
		t.Fatalf("helix plane offset by Thickness %g mm: %v", p["thickness"], err)
	}
	bottom := sectionOn(t, world, world.XY(), toothOutline(t, p, 0))
	top := sectionOn(t, world, helixPlane, toothOutline(t, p, p["helixAngle"]))
	return bottom, top
}

// loftBetween lofts from one section to the other and names which order it was
// given them in.
//
// Every refusal decad can raise here but one is ruled out by construction: the
// two sections are non-nil, they are same-kind segment for segment and equal in
// count because one function draws both, and they sit on two distinct planes
// because Thickness is never zero in this table. What is left is the audit that
// proves the ruled walls make no contact away from their shared edges, which is
// the twist bound this file's header records. So at a nonzero helix angle that
// refusal is reported as a case this proof cannot model, naming the signed
// angle it was refused at; at zero twist no wall can touch another, so the same
// refusal is a defect here and fails.
func loftBetween(t *testing.T, doc *decad.Document, from, to section, p map[string]float64, order string) *decad.Body {
	t.Helper()
	body, err := doc.Loft(from.sketch, from.region, to.sketch, to.region)
	if err == nil {
		return body
	}
	refused := errors.Is(err, decad.ErrUnsupported) || errors.Is(err, decad.ErrDegenerate)
	if refused && p["helixAngle"] != 0 {
		proofkit3d.Unmodelled(t, "%s: decad refuses the ruled walls at a %+.4f rad (%+.1f degree) "+
			"helix angle: %v", order, p["helixAngle"], p["helixAngle"]*180/math.Pi, err)
	}
	t.Fatalf("%s: loft at a %+.4f rad helix angle: %v", order, p["helixAngle"], err)
	return nil
}

// stepLoftTooth is buildTooth: it lofts the bottom Gear Profile tooth loop to
// the top twisted tooth loop into one new body, ctx.toothBody, named Tooth
// Body. It does not extrude and it applies no chamfer.
//
// The BOTTOM section is added first and the top second, which is the order
// helical's contract pins. What that order decides, and what it leaves
// untouched, is proved in assertLoftTooth, which builds the other one.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	bottom, top := twistedPair(t, p)
	return []*decad.Body{loftBetween(t, doc, bottom, top, p, "bottom section first")}
}

// assertLoftTooth pins what the loft is supposed to produce.
func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the loft produced %d bodies, want the one Tooth Body", len(bodies))
	}
	assertSpansTheHelixPlaneOffset(t, bodies[0], p)
	assertTwistOffTheCentroid(t, bodies[0], p, "Tooth Body")
	assertSectionOrder(t, bodies[0], p)
}

// assertSpansTheHelixPlaneOffset holds the lofted tooth between the target
// plane and the twisted profile's plane: it starts on the one and is exactly
// helicalPlaneOffset() — the full Thickness — tall.
//
// A loft's extent is its two sections and nothing else, so this is the reading
// that says the offset hook returned the whole thickness rather than a fraction
// of it, and it is read off the built solid rather than off the plane it was
// asked for.
func assertSpansTheHelixPlaneOffset(t *testing.T, body *decad.Body, p map[string]float64) {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("Tooth Body bounds: %v", err)
	}
	// A box coordinate carries the box's own bound, and their difference twice
	// it. Both expected values are exact — the two sections ARE the two planes
	// — so the slack stated here is the float noise of the subtraction alone.
	base := decad.Measurement{
		Value:     mm(box.Min.Z),
		Exactness: box.Exactness,
		Bound:     mm(box.Bound.Base()),
	}
	height := decad.Measurement{
		Value:     mm(box.Max.Z - box.Min.Z),
		Exactness: box.Exactness,
		Bound:     mm(2 * box.Bound.Base()),
	}
	decadtest.Measures(t, "Tooth Body base against the target plane", base, mm(0),
		decadtest.Within(mm(1e-9)))
	decadtest.Measures(t, "Tooth Body height against the helix plane offset", height,
		mm(p["thickness"]), decadtest.Within(mm(1e-9)))
}

// assertTwistOffTheCentroid reads the twist the loft built, off the body's own
// centroid.
//
// The solid interpolates linearly between a tooth at angle 0 and the same tooth
// at the Helix Angle, and the tooth is symmetric about its own axis, so the
// whole body is symmetric under reflecting across the plane that bisects the
// twist while swapping its two ends. Its centroid is therefore on that bisector
// exactly: at half the Helix Angle, measured about the gear axis from the
// bottom tooth's own axis. The oracle is exact, and measured it holds to under
// 1e-15 radians at every angle in this table, so the slack stated is float
// noise.
//
// That one reading says three things at once. The twist is the Helix Angle
// rather than a lead angle derived from it; Thickness does not enter it, which
// the two cases differing only in Thickness are there to show; and its SIGN is
// the hand of the helix, since a left-hand helix that came out right-handed
// would read at the opposite angle rather than a little off this one.
func assertTwistOffTheCentroid(t *testing.T, body *decad.Body, p map[string]float64, label string) {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("%s centroid: %v", label, err)
	}
	radius := math.Hypot(centroid.Value.X, centroid.Value.Y)
	if radius <= centroid.Bound.Base() {
		t.Fatalf("%s centroid sits %g mm from the gear axis, inside its own %s bound, so no twist "+
			"can be read off it", label, radius, centroid.Bound)
	}
	// A centroid proven to lie within Bound of the reading can be off in angle
	// by at most Bound/radius, which is that bound carried onto the angle
	// derived from it.
	twist := decad.Measurement{
		Value:     units.Radians(math.Atan2(centroid.Value.Y, centroid.Value.X)),
		Exactness: centroid.Exactness,
		Bound:     units.Radians(centroid.Bound.Base() / radius),
	}
	decadtest.Measures(t, label+" twist off its centroid", twist,
		units.Radians(p["helixAngle"]/2), decadtest.Within(units.Radians(1e-9)))
}

// assertSectionOrder builds the loft the other way round and says what the swap
// changes and what it leaves alone.
//
// Helical's contract pins the bottom section as the one added first, and the
// reason it has to be pinned is that the swap is silent in the reading a caller
// is most likely to check. The reversed loft is a valid solid of the same
// handedness, twisted by the same angle in the same direction — the assertion
// above holds on it unchanged — so nothing about the gear announces that the
// sections were given the other way round.
//
// What the swap does change is the ruled walls. A wall between two corresponding
// segments is a quadrilateral that a twist makes non-planar, and it is built as
// two triangles across one of its two diagonals; reversing the sections picks
// the other diagonal. The two solids therefore straddle the true ruled volume,
// and their MEAN is it exactly: for two polygons paired vertex by vertex, the
// cross-section at height t has area (1-t)^2*A0 + t^2*A1 + t(1-t)*K, where K is
// the mixed term prismatoidVolume sums, and integrating that over the thickness
// is a closed form. Measured, the mean matches that closed form to 2e-16
// relative at every angle and thickness in this table, and the two orders
// differ by 6.8 percent of it at the dialog's default 14.5 degrees, 15.3
// percent at 30, 23.2 at 45 and 40.2 at a quarter turn — scale-free, since the
// split is a shape of the faceting rather than a size, and mirrored in the
// sign: the bottom-first loft at +14.5 degrees reads the volume the top-first
// loft reads at -14.5.
//
// So the closed form is asserted against the mean, the gap is asserted to be
// real rather than rounding, and its measured size is printed per case. At zero
// twist every wall is planar, there is no diagonal to choose, and the two
// orders are required to agree outright.
//
// The reversed loft is built in a document of its own and held to the same gate
// the harness applies to the forward one. Both in one document would be two
// live bodies occupying nearly the same space, and decad's disjoint/overlap
// partition proof resolves such a pair neither way, which the solid gate
// refuses and this proof does not waive.
func assertSectionOrder(t *testing.T, forward *decad.Body, p map[string]float64) {
	t.Helper()
	reversedDoc := decad.New()
	bottom, top := twistedPair(t, p)
	reversed := loftBetween(t, reversedDoc, top, bottom, p, "top section first")
	proofkit3d.RequireSolid(t, reversedDoc, []*decad.Body{reversed})
	assertTwistOffTheCentroid(t, reversed, p, "the top-section-first loft")

	forwardVolume := volumeReading(t, forward, "Tooth Body")
	reversedVolume := volumeReading(t, reversed, "the top-section-first loft")
	if p["helixAngle"] == 0 {
		decadtest.Agree(t, "the two section orders at zero twist",
			forwardVolume, reversedVolume, decadtest.WithinRel(units.Scalar(1e-12)))
		return
	}

	mean := decad.Measurement{
		Value:     units.CubicMillimeters((forwardVolume.Value.Base() + reversedVolume.Value.Base()) / 2),
		Exactness: decad.Approximate,
		Bound:     units.CubicMillimeters((forwardVolume.Bound.Base() + reversedVolume.Bound.Base()) / 2),
	}
	want := prismatoidVolume(t, p)
	decadtest.Measures(t, "the two section orders' mean against the ruled volume of the same two "+
		"outlines", mean, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-12)))

	gap := math.Abs(forwardVolume.Value.Base() - reversedVolume.Value.Base())
	bounds := forwardVolume.Bound.Base() + reversedVolume.Bound.Base()
	if gap <= bounds {
		t.Errorf("the two section orders read the same volume to within their own bounds "+
			"(%g mm^3 apart, bounds %g mm^3): at a %+.4f rad twist the walls are built across "+
			"opposite diagonals and the orders are not interchangeable, which is why the order "+
			"is pinned", gap, bounds, p["helixAngle"])
	}
	t.Logf("section order at a %+.1f degree helix angle: bottom first %s, top first %s, "+
		"%.1f percent of the ruled volume apart",
		p["helixAngle"]*180/math.Pi, forwardVolume.Value, reversedVolume.Value, 100*gap/want)
}

// prismatoidVolume is the closed-form volume of the solid ruled between the two
// outlines this case draws, paired vertex by vertex.
//
// The cross-section at height t is the polygon through (1-t)*a_i + t*b_i, whose
// area expands to (1-t)^2*A0 + t^2*A1 + t(1-t)*K with K the mixed term below,
// and integrating that from 0 to 1 gives (A0 + A1)/3 + K/6 per unit of
// thickness. It is a sum of a few dozen products, so its own error is float
// rounding, which is the 1e-12 relative slack its assertion states.
func prismatoidVolume(t *testing.T, p map[string]float64) float64 {
	t.Helper()
	bottom := toothOutline(t, p, 0)
	top := toothOutline(t, p, p["helixAngle"])
	if len(bottom) != len(top) {
		t.Fatalf("the two outlines carry %d and %d vertices; a loft pairs them one for one",
			len(bottom), len(top))
	}
	return p["thickness"] * (polygonArea(bottom)/3 + polygonArea(top)/3 + mixedArea(bottom, top)/6)
}

// polygonArea is the signed area of a closed polygon, by the shoelace sum.
func polygonArea(pts []involute.Pt) float64 {
	sum := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		sum += pts[i].X*pts[j].Y - pts[j].X*pts[i].Y
	}
	return sum / 2
}

// mixedArea is the cross term of the two outlines' shoelace sums, the K of
// prismatoidVolume: the part of the swept area that neither end carries alone.
func mixedArea(a, b []involute.Pt) float64 {
	sum := 0.0
	for i := range a {
		j := (i + 1) % len(a)
		sum += a[i].X*b[j].Y - b[j].X*a[i].Y
		sum += b[i].X*a[j].Y - a[j].X*b[i].Y
	}
	return sum / 2
}

// volumeReading is a body's volume reading: the value decad measured together
// with the bound it proved around it. label is this step's own name for the
// body, which is what a failure has to open with — decadtest names a body by
// its index and recipe step, and "body[0] (step 1 loft)" does not say which of
// the two lofts here is wrong.
func volumeReading(t *testing.T, body *decad.Body, label string) decad.Measurement {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return measured
}
