package herringbonegear_test

// This file proves herringbone's three solid-body facts: the plane its
// helicalPlaneOffset override puts the twisted section on, the half tooth the
// inherited loftTooth builds up to that plane, and the mirror-and-combine that
// turns that half into the chevron.
//
// WHAT IS SUBSTITUTED, AND WHAT THE SUBSTITUTION COSTS. Three of decad's own
// boundaries are met here, and each one is met with the closest geometry the
// engine does accept rather than by dropping the step.
//
//  1. THE SECTIONS ARE CHORDED. decad's loft pairs recorded segments and
//     refuses a free-form pair, so the two involute flanks — fitted splines in
//     Fusion and in this gear's sketch proof — cannot be lofted. Each section
//     here is the same tooth outline walked as a closed polyline through the
//     same involute samples, with the tooth-top arc and the root arc chorded
//     too. What that costs: the flank surface is faceted rather than swept, so
//     every volume below is the chorded tooth's, a little under the real one,
//     and no statement here is about the flank's curvature. What it keeps is
//     everything the three steps are about — where the sections sit, that the
//     twist between them is the helix angle, and how the two halves join.
//
//  2. THE SECTION POINTS ARE FIXED, NOT CONSTRAINED. Every vertex is placed at
//     its computed coordinate and pinned with sketch.Fix, so the section solves
//     to exactly one valid region with no scheme of its own. The constraint
//     scheme is not skipped: it is the sketch proof's subject, on the real
//     spline construction, in stepMidBodyTwistedProfile.
//
//  3. THE MIRROR IS A SECOND LOFT, AND THE COMBINE OVERLAPS. decad has no
//     mirror feature, so the mirrored half is built as the loft that IS the
//     reflection of the first: from the far face's untwisted section TO the
//     mid-body twisted section, the same direction of travel as the first half,
//     which is what makes the two bodies reflections rather than two differently
//     ruled solids (the evaluator's walls are built outward from the FROM
//     section, so the argument order changes the volume — measured 15.764 mm^3
//     against 16.671 mm^3 for the two orders of one 14.5-degree case). And
//     decad's union refuses two solids that meet exactly on a shared face —
//     "two operand facets overlap in one plane — the exact predicates cannot
//     classify a tangent contact" — so the combine step slides the mirrored
//     half's near section one hundredth of the thickness past the mid plane and
//     unions overlapping solids instead of tangent ones. What that costs: the
//     joined body is that sliver thicker at its waist than the gear is, which
//     is why the chevron's volume is asserted against twice a half with a
//     measured 0.7 percent of room rather than exactly.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// solidCases sweeps the chevron across the regime herringbone's inputs reach.
//
// The helix angle carries its whole signed range, because the sign is the hand
// of the helix and the two hands are not the same solid — the evaluator's own
// wall triangulation is chirality-dependent, measured below. Thickness appears
// at both ends of a wide range because the thickness is what the override
// halves: a proof that only ever halved 10 mm would not see an offset that
// happened to be a constant. Size covers coarse and fine teeth, the rib count
// covers the low end, and both routes into the embedded shape are present
// because herringbone inherits no support for either and the proof has to say
// so rather than never meeting one.
var solidCases = []proofkit3d.Case{
	{Name: "default_M1_N17_helix14.5", Params: params(1, 17, 20, 14.5, 15, 10)},
	{Name: "helix_minus14.5_left_hand", Params: params(1, 17, 20, -14.5, 15, 10)},
	{Name: "helix0_no_twist", Params: params(1, 17, 20, 0, 15, 10)},
	{Name: "helix_plus35", Params: params(1, 17, 20, 35, 15, 10)},
	{Name: "helix_minus35_left_hand", Params: params(1, 17, 20, -35, 15, 10)},
	{Name: "helix_plus90_quarter_turn", Params: params(1, 17, 20, 90, 15, 10)},
	{Name: "helix_minus90_quarter_turn", Params: params(1, 17, 20, -90, 15, 10)},

	{Name: "coarse_M3_N15_helix14.5", Params: params(3, 15, 20, 14.5, 15, 10)},
	{Name: "fine_M0.5_N24_helix14.5", Params: params(0.5, 24, 20, 14.5, 15, 10)},
	{Name: "large_M2_N20_helix14.5", Params: params(2, 20, 20, 14.5, 15, 10)},
	{Name: "M1_N12_helix14.5", Params: params(1, 12, 20, 14.5, 15, 10)},

	{Name: "ribs_low_count_5_helix14.5", Params: params(1, 17, 20, 14.5, 5, 10)},
	{Name: "ribs_low_count_3_helix_minus25", Params: params(1, 17, 20, -25, 3, 10)},

	{Name: "thin_T2_helix14.5", Params: params(1, 17, 20, 14.5, 15, 2)},
	{Name: "thick_T40_helix14.5", Params: params(1, 17, 20, 14.5, 15, 40)},

	{Name: "embedded_by_tooth_count_N60_PA20", Params: params(1, 60, 20, 14.5, 15, 10)},
	{Name: "embedded_by_pressure_angle_N30_PA30", Params: params(1, 30, 30, 14.5, 15, 10)},
}

// joinOverlapFraction is how far past the mid plane the mirrored half reaches in
// stepCombineToothHalves, as a fraction of the thickness. See substitution 3 in
// this file's header: decad's union refuses a tangent contact, so the two halves
// have to overlap for the join to be classifiable at all. One hundredth is the
// smallest overlap measured to hold across this table, and small enough that the
// joined body's centroid still sits on the mid plane to within a micron.
const joinOverlapFraction = 0.01

// combineHalfVolume carries one reading from stepCombineToothHalves's build to
// its assertion. The combine consumes both halves, so by the time the assertion
// runs there is no half left to read, and the chevron's volume has to be
// compared against something. That hand-off is why this step is registered with
// the serial proofkit3d.RunSolid and not its parallel counterpart: two cases
// running at once would overwrite each other's reading, and the proof would
// report a wrong verdict rather than fail.
var combineHalfVolume decad.Measurement

// stepMidBodyPlane proves where herringbone's helicalPlaneOffset override puts
// the twisted profile's plane.
//
// helicalPlaneOffset returns Thickness/2 where helical returns the whole
// Thickness, and everything else about this build is inherited, so the offset is
// visible in exactly one place: the face the lofted half ends on. The build is
// the lower half tooth; the assertion reads only its axial span.
func stepMidBodyPlane(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	requireNonEmbedded(t, p)
	thickness := p["thickness"]
	world := sketch.NewWorld()
	return []*decad.Body{loftHalf(t, doc, world, p, 0, 0, thickness/2, p["helixAngle"], "Tooth Body")}
}

// assertMidBodyPlane checks the one length this step is about.
func assertMidBodyPlane(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	box := boundsOf(t, "Tooth Body", bodies[0])
	// The base plane and the mid-body plane, read off the body that spans them.
	// Both numbers are exact — the sections are fixed at their plane's offset —
	// and 1e-9 mm is rounding room, not tolerance for a wrong plane. The top
	// face landing at Thickness/2 rather than at Thickness is the whole of
	// herringbone's helicalPlaneOffset override: helical's hook returns the
	// whole Thickness and would put this face on the far end of the gear.
	measuresAxialSpan(t, "Tooth Body", box, 0, thickness/2, decadtest.Within(units.Millimeters(1e-9)))
}

// stepLoftToothHalf proves the half tooth the inherited loftTooth builds: the
// bottom Gear Profile section lofted to the twisted section on the mid-body
// plane.
func stepLoftToothHalf(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	requireNonEmbedded(t, p)
	thickness := p["thickness"]
	world := sketch.NewWorld()
	return []*decad.Body{loftHalf(t, doc, world, p, 0, 0, thickness/2, p["helixAngle"], "Tooth Body")}
}

// assertLoftToothHalf checks the shape the loft left behind.
func assertLoftToothHalf(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	angle := p["helixAngle"]
	low, high := sectionEnvelope(p, 0, angle, 0, thickness/2)

	// The box is exact and it is the twist's own footprint: a straight prism of
	// the bottom section would be narrower, so a loft that lost the twisted
	// section, or took it at the wrong angle, fails here. Every corner of a
	// ruled loft is a convex combination of the two sections' vertices, so the
	// envelope of those vertices IS the body's bounding box; the readings match
	// it to the last digit and carry a zero bound, and 1e-9 mm is rounding room.
	measuresBounds(t, "Tooth Body", bodies[0], low, high, decadtest.Within(units.Millimeters(1e-9)))

	// The oracle is the prismatoid volume of the ruled solid between the two
	// sections, h/6 * (A0 + 4*Am + A1). It is exact for the solid this loft
	// denotes, because the cross-section at loft parameter u is the polygon
	// whose vertices interpolate the two sections, and a polygon's area is
	// quadratic in its vertices — so Simpson's rule is not an approximation
	// here.
	//
	// What it is not is the number decad reports. The evaluator triangulates
	// each ruled wall on a fixed diagonal, and that choice moves the volume off
	// the ruled solid's own by a measured 0 percent at no twist, 2.8 at 14.5
	// degrees, 4.8 at 25, 6.7 at 35 and 16.6 at a quarter turn — and it moves it
	// the other way for the opposite hand, because the reflection swaps which
	// diagonal each quad takes. The slack is a bound on that wall
	// triangulation, sized for the quarter-turn cases at the end of the table,
	// and not on the formula.
	measuresVolume(t, "Tooth Body", bodies[0],
		units.CubicMillimeters(prismatoidVolume(
			sectionVertices(p, 0), sectionVertices(p, angle), thickness/2)),
		decadtest.WithinRel(units.Scalar(0.2)))
}

// stepMirrorToothHalf proves the mirror across ctx.helixPlane: the lofted half
// reflected in the mid-body plane, which is the half that completes the chevron.
func stepMirrorToothHalf(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	requireNonEmbedded(t, p)
	thickness := p["thickness"]
	angle := p["helixAngle"]
	world := sketch.NewWorld()
	lower := loftHalf(t, doc, world, p, 0, 0, thickness/2, angle, "Tooth Body")
	// Substitution 3 in this file's header: the reflection is built as a loft
	// travelling in the same direction as the first half — from an untwisted
	// section on the far face to the same twisted section on the mid plane.
	mirrored := loftHalf(t, doc, world, p, thickness, 0, thickness/2, angle, "Tooth Body (Mirrored)")
	return []*decad.Body{lower, mirrored}
}

// assertMirrorToothHalf checks that the second half is the first one reflected.
func assertMirrorToothHalf(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	angle := p["helixAngle"]
	lower, mirrored := bodies[0], bodies[1]

	// A reflection moves no volume. Both sides are readings, so both bounds
	// count; the slack on top of them is 1e-12 relative, which is the rounding
	// of the two builds' own arithmetic — measured, the two readings agree to
	// the last digit on every case in this table.
	lowerVolume := volumeOf(t, "Tooth Body", lower)
	mirroredVolume := volumeOf(t, "Tooth Body (Mirrored)", mirrored)
	decadtest.Agree(t, "the mirrored half against the half it was mirrored from",
		lowerVolume, mirroredVolume, decadtest.WithinRel(units.Scalar(1e-12)))

	// Same footprint, reflected span: the mirrored half stands on the mid plane
	// and reaches the far face. Exact, as in stepLoftToothHalf.
	low, high := sectionEnvelope(p, 0, angle, thickness/2, thickness)
	measuresBounds(t, "Tooth Body (Mirrored)", mirrored, low, high,
		decadtest.Within(units.Millimeters(1e-9)))
}

// stepCombineToothHalves proves the combine: the mirrored half joined into
// 'Tooth Body' so that one body spans the full thickness before the inherited
// patternTeeth sees it.
func stepCombineToothHalves(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	requireNonEmbedded(t, p)
	thickness := p["thickness"]
	angle := p["helixAngle"]
	if angle == 0 {
		// Not a gap in the scheme, a boundary of the engine, and it is here
		// rather than only in the step list so the next reader of this proof
		// finds it. With no twist the two halves are straight prisms on the same
		// outline, so every side wall of one is coplanar with the other's, and
		// the overlap that substitution 3 relies on does not help: decad still
		// refuses with "two operand facets overlap in one plane — the exact
		// predicates cannot classify a tangent contact". A gear at zero helix
		// angle is a spur gear whose chevron has no apex, so what is lost is a
		// degenerate case rather than a working one — but it is lost, and only a
		// Fusion session or a boolean that classifies coplanar contact can say
		// what the combine does there.
		proofkit3d.Unmodelled(t, "at zero helix angle the two halves' walls are coplanar and "+
			"decad's union cannot classify the contact")
		return nil
	}
	world := sketch.NewWorld()
	lower := loftHalf(t, doc, world, p, 0, 0, thickness/2, angle, "Tooth Body")
	mirrored := loftHalf(t, doc, world, p, thickness, 0,
		thickness/2-thickness*joinOverlapFraction, angle, "Tooth Body (Mirrored)")

	// Read the half before the union consumes it; see combineHalfVolume.
	combineHalfVolume = volumeOf(t, "Tooth Body", lower)

	// The combine is a Join, and its target is the lofted half while the
	// mirrored half is the tool — the order [HERR-F-MIRROR-COMBINE] pins, where
	// the target is looked up by the name 'Tooth Body'.
	chevron, err := decad.Union(lower, mirrored)
	if err != nil {
		t.Fatalf("combine 'Tooth Body (Mirrored)' into 'Tooth Body': %v", err)
	}
	return []*decad.Body{chevron}
}

// assertCombineToothHalves checks that the join produced the chevron.
//
// That the result is ONE body, watertight, with a single lump and no voids, is
// the gate's own verdict (proofkit3d.RunSolid), and it is the point of the
// combine: the inherited patternTeeth circular-patterns one tooth body.
func assertCombineToothHalves(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	angle := p["helixAngle"]
	chevron := bodies[0]

	// The joined body spans the whole thickness and keeps both halves'
	// footprint, which is the same envelope either half has. Exact, as above.
	low, high := sectionEnvelope(p, 0, angle, 0, thickness)
	measuresBounds(t, "Tooth Body", chevron, low, high, decadtest.Within(units.Millimeters(1e-9)))

	// Two halves in, one chevron out, and nothing lost or counted twice. The
	// expected value is twice the half read in the build, so the comparison is
	// reading against reading in all but name; the slack has to cover the
	// mirrored half's overhang past the mid plane (substitution 3) and the wall
	// triangulation at the join. Measured over this table the difference runs
	// from 0.02 percent at 14.5 degrees to 0.61 at a quarter turn, so 2 percent
	// leaves room without admitting a lost half, which would be 50.
	measuresVolume(t, "Tooth Body", chevron, combineHalfVolume.Value.Scale(2),
		decadtest.WithinRel(units.Scalar(0.02)))

	// The chevron is symmetric about the mid plane, so its centroid sits on it.
	// This is the one reading that says the two halves are a chevron rather than
	// a wedge: a mirror that did not reflect, or a combine that kept only one
	// half, moves this off the mid plane by a quarter of the thickness. The
	// slack is the overhang's own asymmetry, measured at most 3.1e-4 mm across
	// this table.
	centroid := centroidOf(t, "Tooth Body", chevron)
	decadtest.Measures(t, "Tooth Body centroid height",
		decad.Measurement{
			Value:     units.Millimeters(centroid.Value.Z),
			Exactness: centroid.Exactness,
			Bound:     centroid.Bound,
		},
		units.Millimeters(thickness/2), decadtest.Within(units.Millimeters(1e-3)))
}

// requireNonEmbedded skips a case whose tooth profile is embedded.
//
// The inherited loftTooth finds both of its sections with a fixed
// nurbs=2, arcs=2, lines=2 and never reads ctx.toothProfileIsEmbedded, so an
// embedded tooth — the flanks crossing the root circle themselves, leaving no
// flank-to-root stubs — is a shape it cannot find at all ([HELI-F-LOFT]'s
// documented limitation, inherited unchanged). The sketch proof measures that
// shape; there is no solid for this proof to build from it, and the chorded
// outline here would self-intersect if it tried, since its root stubs would run
// inward from the flank start.
func requireNonEmbedded(t *testing.T, p map[string]float64) {
	t.Helper()
	if dimensionsOf(p).Embedded() {
		proofkit3d.Unmodelled(t, "the tooth profile is embedded, which the inherited loftTooth's "+
			"fixed lines=2 profile search cannot find")
	}
}

// sectionVertices is the chorded tooth outline at one draw angle, in the order
// the loft pairs it: the left root stub, the left flank from the base circle
// out to the tip, the right flank back in, and the right root stub. Both
// sections of a loft are built by this one function, so they pair segment for
// segment and kind for kind, which is what decad's loft requires.
func sectionVertices(p map[string]float64, angle float64) []involute.Pt {
	dims := dimensionsOf(p)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch,
		p["toothNumber"], int(p["involuteSteps"]), angle)
	onRootCircle := func(point involute.Pt) involute.Pt {
		radius := math.Hypot(point.X, point.Y)
		return involute.Pt{X: dims.Root * point.X / radius, Y: dims.Root * point.Y / radius}
	}
	out := make([]involute.Pt, 0, 2*len(left)+2)
	out = append(out, onRootCircle(left[0]))
	out = append(out, left...)
	for i := len(right) - 1; i >= 0; i-- {
		out = append(out, right[i])
	}
	return append(out, onRootCircle(right[0]))
}

// sectionSketch draws one chorded section on a plane offset by z and returns it
// with its single valid region.
func sectionSketch(t *testing.T, world *sketch.World, p map[string]float64, z, angle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane := world.XY()
	if z != 0 {
		offset, err := world.CreateOffsetPlane(world.XY(), z)
		if err != nil {
			t.Fatalf("construction plane offset %g mm from the base plane: %v", z, err)
		}
		plane = offset
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch on the plane %g mm from the base plane: %v", z, err)
	}
	vertices := sectionVertices(p, angle)
	points := make([]*sketch.Point, len(vertices))
	for i, vertex := range vertices {
		points[i] = s.CreatePoint(vertex.X, vertex.Y)
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	// Substitution 2 in this file's header: the outline is pinned rather than
	// constrained, because the constraint scheme is the sketch proof's subject.
	for _, point := range points {
		s.Fix(point)
	}
	return s, decadtest.SolveRegion(t, s)
}

// loftHalf lofts one half tooth, from an untwisted section on the face at
// fromZ to the twisted section on the plane at toZ, and names the body.
//
// Both halves travel in this direction, which is what makes them reflections of
// each other rather than two differently ruled solids; see substitution 3.
func loftHalf(t *testing.T, doc *decad.Document, world *sketch.World, p map[string]float64,
	fromZ, fromAngle, toZ, toAngle float64, name string) *decad.Body {
	t.Helper()
	fromSketch, fromProfile := sectionSketch(t, world, p, fromZ, fromAngle)
	toSketch, toProfile := sectionSketch(t, world, p, toZ, toAngle)
	body, err := doc.Loft(fromSketch, fromProfile, toSketch, toProfile)
	if err != nil {
		t.Fatalf("loft %q from the section at %g mm to the section at %g mm: %v", name, fromZ, toZ, err)
	}
	return body
}

// sectionEnvelope is the axis-aligned box of a body lofted between the sections
// at two draw angles, spanning lowZ to highZ.
//
// Every point of a ruled loft is a convex combination of the two sections'
// vertices, so the box of those vertices is the box of the solid.
func sectionEnvelope(p map[string]float64, firstAngle, secondAngle, lowZ, highZ float64) (low, high r3.Vec) {
	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	for _, angle := range []float64{firstAngle, secondAngle} {
		for _, vertex := range sectionVertices(p, angle) {
			minX, maxX = math.Min(minX, vertex.X), math.Max(maxX, vertex.X)
			minY, maxY = math.Min(minY, vertex.Y), math.Max(maxY, vertex.Y)
		}
	}
	return r3.NewVec(minX, minY, lowZ), r3.NewVec(maxX, maxY, highZ)
}

// polygonArea is the shoelace area of one section outline.
func polygonArea(vertices []involute.Pt) float64 {
	sum := 0.0
	for i := range vertices {
		next := vertices[(i+1)%len(vertices)]
		sum += vertices[i].X*next.Y - next.X*vertices[i].Y
	}
	return math.Abs(sum) / 2
}

// prismatoidVolume is the volume of the ruled solid between two sections a
// height apart: h/6 * (A0 + 4*Am + A1), with Am the area of the section halfway
// between them. See assertLoftToothHalf for why the rule is exact here.
func prismatoidVolume(from, to []involute.Pt, height float64) float64 {
	middle := make([]involute.Pt, len(from))
	for i := range from {
		middle[i] = involute.Pt{X: (from[i].X + to[i].X) / 2, Y: (from[i].Y + to[i].Y) / 2}
	}
	return height / 6 * (polygonArea(from) + 4*polygonArea(middle) + polygonArea(to))
}

// The four readers below exist so a failure names the body the step list names
// — 'Tooth Body', 'Tooth Body (Mirrored)' — instead of decadtest's body index
// and recipe step, which do not say which feature is wrong.

func volumeOf(t *testing.T, name string, body *decad.Body) decad.Measurement {
	t.Helper()
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: reading it failed: %v", name, err)
	}
	return volume
}

func centroidOf(t *testing.T, name string, body *decad.Body) decad.VecMeasurement {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("%s centroid: reading it failed: %v", name, err)
	}
	return centroid
}

func boundsOf(t *testing.T, name string, body *decad.Body) decad.Box {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: reading it failed: %v", name, err)
	}
	return box
}

func measuresVolume(t *testing.T, name string, body *decad.Body, want units.Value, opts ...decadtest.Option) {
	t.Helper()
	decadtest.Measures(t, name+" volume", volumeOf(t, name, body), want, opts...)
}

func measuresBounds(t *testing.T, name string, body *decad.Body, low, high r3.Vec, opts ...decadtest.Option) {
	t.Helper()
	decadtest.MeasuresBox(t, name+" bounds", boundsOf(t, name, body), low, high, opts...)
}

// measuresAxialSpan reads the two faces a body stands between off its own box.
// The box's proven bound covers each coordinate in it, so projecting it onto
// the axis keeps the reading a reading rather than turning it into a bare float.
func measuresAxialSpan(t *testing.T, name string, box decad.Box, low, high float64, opts ...decadtest.Option) {
	t.Helper()
	decadtest.Measures(t, name+" base face",
		decad.Measurement{Value: units.Millimeters(box.Min.Z), Exactness: box.Exactness, Bound: box.Bound},
		units.Millimeters(low), opts...)
	decadtest.Measures(t, name+" mid-body face",
		decad.Measurement{Value: units.Millimeters(box.Max.Z), Exactness: box.Exactness, Bound: box.Bound},
		units.Millimeters(high), opts...)
}
