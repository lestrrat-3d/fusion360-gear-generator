package helicalgear_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// WHAT THIS PROOF SUBSTITUTES, AND WHAT THE SUBSTITUTION COSTS.
//
// Fusion lofts two profiles whose flanks are fitted splines and whose tooth-top
// and root boundaries are arcs. decad's Loft pairs the two sections segment by
// segment and refuses a free-form pair outright (ErrUnsupported), so a spline
// flank cannot cross this harness at all. Keeping the two arcs as arcs does
// build, but its volume reading then carries a chord error bound past decad's
// own relative tolerance and the document verdict comes back Suspect — measured
// here at every helix angle, 32.38 mm^3 with a 0.0407 mm^3 bound against a
// 0.0324 mm^3 requirement — and neither proofkit3d gate may waive a volume
// diagnostic.
//
// So each section is CHORDED: every flank runs through the same involute sample
// points the Fusion spline interpolates, joined by straight segments, and the
// tooth-top and root arcs are chorded too. The sections stay the same shape,
// the same point set, and the same pairing.
//
// What that costs is the wall surface: between two chords the loft rules a flat
// quad where Fusion rules the surface between two splines, so the volume here
// is the chorded body's, not the gear tooth's. What it still pins is everything
// the loft step is responsible for — that the two sections pair at all, the
// order they are added in, the extent between the two planes, the sign and the
// size of the twist, and that the result is one sound solid lump.
//
// The tooth's own 6-curve profile contract — 2 splines, 2 arcs, 2 lines, the
// count both of loftTooth's find_profile_by_curve_counts calls key on — is not
// reachable here, since the chorded section is all lines. It is asserted in the
// sketch proof, on the sketch that actually draws those curves.

// arcChordCount is how many straight segments each chorded arc becomes. Six is
// enough that the chorded section's area is within a part in a thousand of the
// arc's, and small enough to keep the loft's pair-test budget clear.
const arcChordCount = 6

// solidCases sweeps the loft across the regime the spec states.
//
// Both signs, because the sign is the hand of the helix and a scheme that drops
// it still lofts at +angle. Both ends of what this harness can build, per sign,
// because the two are not the same size: see loftTwistBound. Two thicknesses at
// one angle, because the spec says the twist between the sections IS the helix
// angle and that Thickness does not enter — a rule that only shows up as a
// disagreement between two thicknesses. And a coarse and a fine gear, because
// the section scales with the tooth.
var solidCases = []proofkit3d.Case{
	{Name: "default_M1_N17_helix14.5_t10", Params: params(1, 17, 20, 14.5, 15)},
	{Name: "default_helix14.5_t30_thickness_does_not_enter", Params: thickness(params(1, 17, 20, 14.5, 15), 30)},
	{Name: "default_helix14.5_t3_thickness_does_not_enter", Params: thickness(params(1, 17, 20, 14.5, 15), 3)},
	{Name: "helix0_straight_prism", Params: params(1, 17, 20, 0, 15)},
	{Name: "helix_minus14.5_left_hand", Params: params(1, 17, 20, -14.5, 15)},
	{Name: "helix_plus35", Params: params(1, 17, 20, 35, 15)},
	{Name: "helix_minus35_left_hand", Params: params(1, 17, 20, -35, 15)},
	{Name: "helix_plus90_upper_bound", Params: params(1, 17, 20, 90, 15)},
	{Name: "helix_minus90", Params: params(1, 17, 20, -90, 15)},
	{Name: "helix_minus150_left_hand_reaches_further", Params: params(1, 17, 20, -150, 15)},
	{Name: "coarse_M3_N15_helix14.5", Params: params(3, 15, 20, 14.5, 15)},
	{Name: "fine_M0.5_N24_helix_minus14.5", Params: params(0.5, 24, 20, -14.5, 15)},
	{Name: "ribs_low_count_5_helix14.5", Params: params(1, 17, 20, 14.5, 5)},
	{Name: "beyond_the_proof_bound_helix_plus150", Params: params(1, 17, 20, 150, 15)},
}

// planeCases sweeps the offset the helix plane is created at. The angle is held
// at the default and the thickness varies, because the offset is what these
// cases are about.
var planeCases = []proofkit3d.Case{
	{Name: "thickness10_default", Params: params(1, 17, 20, 14.5, 15)},
	{Name: "thickness3_thin", Params: thickness(params(1, 17, 20, 14.5, 15), 3)},
	{Name: "thickness30_thick", Params: thickness(params(1, 17, 20, 14.5, 15), 30)},
	{Name: "thickness10_left_hand", Params: params(1, 17, 20, -14.5, 15)},
	{Name: "thickness30_helix0", Params: thickness(params(1, 17, 20, 0, 15), 30)},
}

// thickness returns p with Thickness replaced.
func thickness(p map[string]float64, mm float64) map[string]float64 {
	out := make(map[string]float64, len(p))
	for k, v := range p {
		out[k] = v
	}
	out["thickness"] = mm
	return out
}

// loftTwistBound is the largest twist, per sign, that this harness builds and
// verifies for the default gear — measured here, not assumed, and stated per
// sign because the two are not equal.
//
// Measured 2026-09-03, in 5-degree steps, at thicknesses 5, 10 and 30 mm:
// a RIGHT-hand (positive) twist verifies through +90 degrees and is refused at
// +95 with "loft triangles 0 and 2 do not meet exactly at their recorded shared
// vertex"; a LEFT-hand (negative) twist verifies through -175 and is refused at
// -180. Every bound was identical at all three thicknesses, which is itself the
// measured form of the spec's claim that the twist is the helix angle and that
// Thickness does not enter.
//
// This is a property of the chorded section and of decad's loft audit, NOT of
// the gear: helicalgear.py clamps nothing, the dialog accepts whatever the user
// types, and what Fusion does at a large helix angle is unverified. A case past
// the bound is proven to be REFUSED rather than skipped, so the bound stays
// measured as the harness changes.
const (
	loftTwistBoundPositive = 90.0
	loftTwistBoundNegative = -175.0
)

// stepHelixPlane creates the offset construction plane the twisted profile is
// drawn on, at helicalPlaneOffset() = the full Thickness, and lofts across it.
//
// The plane is only observable through what is built on it, so the step builds
// the loft and the assertion measures the gap it spans. The build also proves
// why the plane exists at all: with the twisted profile drawn on the gear's own
// plane instead, the two sections are coplanar and there is no solid to build.
func stepHelixPlane(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	dims := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	steps := int(p["involuteSteps"])
	offset := p["thickness"] // helicalPlaneOffset(): the FULL thickness

	world := sketch.NewWorld()
	basePlane := world.XY()
	bottomSketch, bottomProfile := loftSection(t, world, basePlane, dims, p, steps, 0)

	// The zero-offset control. helicalPlaneOffset is a distinct overridable
	// hook and the offset is what it returns; an offset of zero leaves both
	// sections on the gear's own plane, where every wall vertex lies in one
	// plane and the body has zero volume by construction.
	coplanarSketch, coplanarProfile := loftSection(t, world, basePlane, dims, p, steps, p["helixAngle"])
	if _, err := doc.Loft(bottomSketch, bottomProfile, coplanarSketch, coplanarProfile); err == nil {
		t.Fatal("a loft between two coplanar sections built a body; the helix plane's offset is " +
			"then not load-bearing, and this control has stopped controlling anything")
	} else if !errors.Is(err, decad.ErrDegenerate) {
		t.Fatalf("a coplanar loft must be refused as degenerate, got %v", err)
	}

	helixPlane, err := world.CreateOffsetPlane(basePlane, offset)
	if err != nil {
		t.Fatalf("helix plane at offset %g: %v", offset, err)
	}
	topSketch, topProfile := loftSection(t, world, helixPlane, dims, p, steps, p["helixAngle"])

	body, err := doc.Loft(bottomSketch, bottomProfile, topSketch, topProfile)
	if err != nil {
		t.Fatalf("loft across the helix plane at offset %g: %v", offset, err)
	}
	return []*decad.Body{body}
}

// assertHelixPlane measures the gap the helix plane was created at.
//
// helicalPlaneOffset() returns the FULL Thickness for helical. Herringbone
// re-points the same hook at half the thickness so its mirror plane lands
// mid-body, so the number this asserts is the one thing that separates the two
// gears' planes, and half a thickness is the wrong answer this catches.
func assertHelixPlane(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the loft leaves exactly one body, got %d", len(bodies))
	}
	want := p["thickness"]
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	span := box.Max.Z - box.Min.Z
	tolerance := 1e-9 * want
	if math.Abs(box.Min.Z) > tolerance {
		t.Errorf("the body starts at z=%.12g, not on the gear's own plane", box.Min.Z)
	}
	if math.Abs(span-want) > tolerance {
		t.Errorf("the helix plane sits %.12g mm off the gear plane, want the full Thickness %.12g mm "+
			"(half of it, %.12g, is herringbone's offset, not helical's)", span, want, want/2)
	}
}

// stepLoftTooth lofts the bottom Gear Profile tooth loop to the top twisted
// tooth loop, bottom section first ([HELI-F-LOFT]).
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	dims := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	steps := int(p["involuteSteps"])
	helix := p["helixAngle"]
	helixDegrees := helix * 180 / math.Pi

	world := sketch.NewWorld()
	bottomSketch, bottomProfile := loftSection(t, world, world.XY(), dims, p, steps, 0)
	helixPlane, err := world.CreateOffsetPlane(world.XY(), p["thickness"])
	if err != nil {
		t.Fatalf("helix plane: %v", err)
	}
	topSketch, topProfile := loftSection(t, world, helixPlane, dims, p, steps, helix)

	// Both sections are the same tooth, so they carry the same segment count.
	// The loft pairs them positionally, and a count mismatch is exactly what
	// decad refuses as ErrUnsupported — the harness form of Fusion's rule that
	// both loft sections are the same non-embedded 6-curve tooth.
	wantSegments := 2*steps + 2*arcChordCount
	if got := len(bottomProfile.Outer); got != wantSegments {
		t.Fatalf("bottom section has %d segments, want %d", got, wantSegments)
	}
	if got := len(topProfile.Outer); got != wantSegments {
		t.Fatalf("top section has %d segments, want %d", got, wantSegments)
	}

	body, err := doc.Loft(bottomSketch, bottomProfile, topSketch, topProfile)
	if err != nil {
		// Past the measured bound the refusal is the finding, and it is proven
		// rather than skipped. Inside the bound a refusal is a real failure.
		if helixDegrees > loftTwistBoundPositive || helixDegrees < loftTwistBoundNegative {
			proofkit3d.Unmodelled(t, "a %+.1f degree twist is past this harness's measured bound "+
				"(%+.1f to %+.1f degrees for the chorded section); decad refused it with %v, which "+
				"is a limit of the proof and not of the gear — helicalgear.py clamps nothing",
				helixDegrees, loftTwistBoundNegative, loftTwistBoundPositive, err)
		}
		t.Fatalf("loft at %+.1f degrees: %v", helixDegrees, err)
	}
	if helixDegrees > loftTwistBoundPositive || helixDegrees < loftTwistBoundNegative {
		t.Fatalf("a %+.1f degree twist built, but the recorded bound says it cannot; re-measure "+
			"loftTwistBoundPositive/loftTwistBoundNegative", helixDegrees)
	}
	return []*decad.Body{body}
}

// assertLoftTooth measures what the loft is supposed to produce: one solid
// tooth body, spanning the two planes, twisted by exactly the helix angle.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("loftTooth leaves exactly one body in ctx.toothBody, got %d", len(bodies))
	}
	body := bodies[0]
	dims := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	steps := int(p["involuteSteps"])

	// The section order. loftSections.add takes the bottom profile first, so
	// the loft's START cap is the bottom Gear Profile, on the gear's own plane,
	// and its END cap is the twisted profile on the helix plane.
	startZ := capPlaneHeight(t, body, decad.CapStart(body))
	endZ := capPlaneHeight(t, body, decad.CapEnd(body))
	if math.Abs(startZ) > 1e-9*p["thickness"] {
		t.Errorf("the loft's start cap sits at z=%.12g, so the bottom Gear Profile was not the "+
			"first section added", startZ)
	}
	if math.Abs(endZ-p["thickness"]) > 1e-9*p["thickness"] {
		t.Errorf("the loft's end cap sits at z=%.12g, want the helix plane at %.12g", endZ, p["thickness"])
	}

	// The twist. Each section is symmetric about its own tooth axis, so the
	// centroid of a cap's vertices lies on that axis and the angle between the
	// two axes is the twist the loft carries.
	//
	// This is the assertion that catches [SPUR-F-ROTATE-CONFIRM]'s failure: a
	// top profile drawn flat and swung by the confirming dimension can settle
	// half a turn away, and the loft then passes through the gear centre. That
	// body reads here as a twist of helix+180, not helix.
	twist := capAxisAngle(t, body, decad.CapEnd(body)) - capAxisAngle(t, body, decad.CapStart(body))
	twist = normalizeAngle(twist)
	want := normalizeAngle(p["helixAngle"])
	if math.Abs(twist-want) > 1e-9 {
		t.Errorf("the loft twists by %.9f rad (%.4f deg), want the Helix Angle %.9f rad (%.4f deg) — "+
			"nothing rescales it, so Thickness %.4g must not enter",
			twist, twist*180/math.Pi, want, want*180/math.Pi, p["thickness"])
	}

	// The volume. At zero twist the loft is a straight prism on the section, so
	// its volume is the section area times the thickness exactly; that is the
	// reading that says the extent really is the plane gap and the section
	// really is this tooth.
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	prism := ringArea(sectionRing(dims, p["toothNumber"], steps, 0)) * p["thickness"]
	measured := volume.Value.Base()
	if measured <= 0 {
		t.Fatalf("the tooth body has volume %.6g", measured)
	}
	if p["helixAngle"] == 0 {
		if math.Abs(measured-prism) > 1e-9*prism {
			t.Errorf("at zero twist the loft is a straight prism of %.9g mm^3, measured %.9g", prism, measured)
		}
		return
	}
	if measured > prism*1.5 || measured < prism*0.4 {
		t.Errorf("the twisted tooth measures %.6g mm^3 against a %.6g mm^3 prism on the same "+
			"section, which is not a tooth swept between the two planes", measured, prism)
	}
}

// loftSection draws one loft section — a chorded tooth loop — on plane, at the
// given twist, and returns it with its detected profile.
//
// Every point is fixed. The constraint scheme that holds this tooth together is
// proven in the sketch proof, on the sketch that carries it; here the geometry
// is a fixed input to the solid, and fixing it is what makes the profile
// recordable without restating a scheme that is proven elsewhere.
func loftSection(t *testing.T, world *sketch.World, plane *sketch.Plane, dims involute.Dimensions,
	p map[string]float64, steps int, angle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()

	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	ring := sectionRing(dims, p["toothNumber"], steps, angle)
	points := make([]*sketch.Point, len(ring))
	for i, pt := range ring {
		points[i] = s.CreatePoint(pt.X, pt.Y)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("a loft section closes exactly one region, got %d", len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatal("the loft section's region is not an extrudable profile")
	}
	return s, profiles[0]
}

// sectionRing returns the chorded tooth loop, counter-clockwise, at the given
// twist: root foot, left flank, tooth-top arc, right flank back down, root
// foot, root arc home.
func sectionRing(dims involute.Dimensions, toothNumber float64, steps int, angle float64) []involute.Pt {
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, toothNumber, steps, angle)
	foot := func(p involute.Pt) involute.Pt {
		n := math.Hypot(p.X, p.Y)
		return involute.Pt{X: dims.Root * p.X / n, Y: dims.Root * p.Y / n}
	}
	leftFoot, rightFoot := foot(left[0]), foot(right[0])

	ring := []involute.Pt{leftFoot}
	ring = append(ring, left...)
	ring = append(ring, chordArc(left[len(left)-1], right[len(right)-1], dims.Tip)...)
	for i := len(right) - 1; i >= 0; i-- {
		ring = append(ring, right[i])
	}
	ring = append(ring, rightFoot)
	ring = append(ring, chordArc(rightFoot, leftFoot, dims.Root)...)
	return ring
}

// chordArc returns the interior points of the SHORT arc from a to b at radius
// r, as arcChordCount straight segments.
func chordArc(a, b involute.Pt, r float64) []involute.Pt {
	from := math.Atan2(a.Y, a.X)
	to := normalizeAngle(math.Atan2(b.Y, b.X)-from) + from
	out := make([]involute.Pt, 0, arcChordCount-1)
	for i := 1; i < arcChordCount; i++ {
		at := from + (to-from)*float64(i)/float64(arcChordCount)
		out = append(out, involute.Pt{X: r * math.Cos(at), Y: r * math.Sin(at)})
	}
	return out
}

// ringArea is the shoelace area of a closed polygon.
func ringArea(ring []involute.Pt) float64 {
	sum := 0.0
	for i := range ring {
		j := (i + 1) % len(ring)
		sum += ring[i].X*ring[j].Y - ring[j].X*ring[i].Y
	}
	return math.Abs(sum) / 2
}

// capVertices returns the distinct vertex positions of the one face the given
// cap role created.
func capVertices(t *testing.T, body *decad.Body, role decad.FeatureRef) []decad.VecMeasurement {
	t.Helper()
	faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
	if err != nil {
		t.Fatalf("select the %s cap face: %v", role.Role, err)
	}
	seen := map[*decad.Vertex]bool{}
	var out []decad.VecMeasurement
	for _, loop := range faces[0].Loops() {
		for _, edge := range loop.Edges() {
			for _, vertex := range []*decad.Vertex{edge.Start(), edge.End()} {
				if seen[vertex] {
					continue
				}
				seen[vertex] = true
				out = append(out, vertex.Position())
			}
		}
	}
	if len(out) == 0 {
		t.Fatalf("the %s cap face has no vertices", role.Role)
	}
	return out
}

// capPlaneHeight is the height of a cap face above the gear's own plane.
func capPlaneHeight(t *testing.T, body *decad.Body, role decad.FeatureRef) float64 {
	t.Helper()
	vertices := capVertices(t, body, role)
	height := vertices[0].Value.Z
	for _, vertex := range vertices[1:] {
		if math.Abs(vertex.Value.Z-height) > 1e-9 {
			t.Fatalf("the %s cap face is not planar in z: %.12g and %.12g", role.Role, height, vertex.Value.Z)
		}
	}
	return height
}

// capAxisAngle is the polar angle of a cap section's own axis of symmetry.
//
// The tooth section is symmetric about that axis — mirrored flanks, symmetric
// chords on both arcs — so the centroid of its vertices lies on it.
func capAxisAngle(t *testing.T, body *decad.Body, role decad.FeatureRef) float64 {
	t.Helper()
	vertices := capVertices(t, body, role)
	var x, y float64
	for _, vertex := range vertices {
		x += vertex.Value.X
		y += vertex.Value.Y
	}
	return math.Atan2(y/float64(len(vertices)), x/float64(len(vertices)))
}

// normalizeAngle folds an angle into (-pi, pi].
func normalizeAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
