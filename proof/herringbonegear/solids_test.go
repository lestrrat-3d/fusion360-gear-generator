// This file holds herringbone's three solid steps: the lofted bottom-half
// tooth, the mirrored top half, and the combine that leaves one tooth body
// spanning the full thickness.
//
// Two substitutions run through all three, and both are named where they are
// made rather than only here.
//
// The section is chorded. decad lofts a pair of sections segment by segment
// and pairs two LineSegs, two ArcSegs or two CircleSegs only, so the fitted
// splines Fusion lofts are refused by name; and a loft carrying arc pairs comes
// back with a volume whose error bound is wider than the verification
// tolerance, which the solid gate rejects. Each section is therefore drawn as
// one closed chain of chords: through the involute sample points along each
// flank, and through arcChordCount steps along the root and tooth-top arcs.
// Every sample and both flank-to-root stubs sit exactly where the real section
// has them; what is lost is the curvature between them, so the section is a
// hair smaller than the drawn one, and the readings taken from it are compared
// against that same chorded section rather than against the arc-bounded ideal.
// What the loft is being asked to prove is unaffected: where the two sections
// sit, that the body between them is a sound solid, and that the second half is
// the first one reflected.
//
// The mirror is a second loft, not a reflection. decad's Placed and PlacedCopy
// carry a body under a RIGID motion; a reflection is not one, so the mirror
// feature has no direct counterpart here. The reflection of a ruled loft across
// its own top plane is the same two sections ruled again, one thickness-half
// higher, so that is what the proof builds — and the SECTION ORDER matters, for
// a reason worth knowing: see mirrorSectionOrder.
package herringbonegear_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// solidCases sweeps the twists and thicknesses the tooth is lofted across.
//
// The helix angle is signed, and a left-hand gear is the same build mirrored,
// so every twist is swept on both signs. The thickness is what herringbone
// halves, so it is swept from a thin body — where the mid plane sits at 0.5 mm
// and the two halves are short — to a thick one.
//
// Embedded profiles are absent on purpose. The inherited loftTooth passes a
// fixed nurbs=2, arcs=2, lines=2 to the profile finder and has no embedded
// branch, so an embedded gear has no lofted tooth to prove; the sketch step
// carries both embedded routes and pins the four-curve loop that finder cannot
// match.
var solidCases = []proofkit3d.Case{
	{Name: "default_right_hand_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "left_hand_negative_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 15, "thickness": 10,
	}},
	{Name: "steep_right_hand_35deg", Params: map[string]float64{
		"module": 2, "toothNumber": 24, "pressureAngle": deg(20),
		"helixAngle": deg(35), "involuteSteps": 15, "thickness": 20,
	}},
	{Name: "steep_left_hand_35deg", Params: map[string]float64{
		"module": 2, "toothNumber": 24, "pressureAngle": deg(20),
		"helixAngle": deg(-35), "involuteSteps": 15, "thickness": 20,
	}},
	{Name: "thin_body", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15, "thickness": 1,
	}},
	{Name: "thick_body_coarse_module", Params: map[string]float64{
		"module": 4, "toothNumber": 12, "pressureAngle": deg(20),
		"helixAngle": deg(-20), "involuteSteps": 15, "thickness": 60,
	}},
}

// mirrorSectionOrder records why the mirrored half is lofted from its FAR
// section inward rather than from the mid plane outward.
//
// A twist makes every ruled wall panel non-planar, and decad builds those walls
// outward from the section given FIRST, so which section leads decides how each
// panel is split into triangles. Ruling the same two sections the other way
// round therefore returns a slightly different polyhedron: measured on the
// pinned decad, 15.818 mm^3 against 16.227 mm^3 at 14.5 degrees (2.5% apart)
// and 123.677 against 129.350 at 35 degrees (4.4%), each with a proven error
// bound near 1e-16, so these are two exact readings of two solids rather than
// one uncertain reading of one.
//
// A mirror has no such freedom — it returns the reflection and nothing else —
// so the proof has to pick the ruling that IS the reflection. Ruling the
// mirrored half from the untwisted far section into the twisted mid section,
// the same order the lofted half is ruled in, makes the two triangulate as
// mirror images and their volumes agree to the last digit. Ruling it from the
// mid section outward instead leaves a half that is the right shape to a few
// percent and the wrong solid exactly, which is why this is pinned here rather
// than absorbed into a loose tolerance.
const mirrorSectionOrder = "far section first, mid section second"

// mirrorVolumeTolerance is the band the two halves' volumes are compared in.
// A mirror changes no volume, and with the section order above the two halves
// come back bit-identical, so the band only absorbs float64 noise.
const mirrorVolumeTolerance = 1e-9

// stepLoftToothHalf lofts the bottom half of the tooth.
//
// The bottom section is the untwisted Gear Profile tooth on the target plane
// and the top section is the twisted one on the mid-body plane, added in that
// order, which is the loft the inherited loftTooth performs once
// helicalPlaneOffset has put its plane at half the thickness.
func stepLoftToothHalf(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	lower, _ := loftLowerHalf(t, doc, p)
	return []*decad.Body{lower}
}

// assertLoftToothHalf checks the half spans the bottom half of the body and
// keeps the drawn tooth section at both ends.
func assertLoftToothHalf(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	section := toothSectionArea(t, p)

	requireSpan(t, bodies[0], "lofted half", 0, thickness/2)
	requireCapArea(t, bodies[0], "lofted half", r3.NewVec(0, 0, -1), section)
	requireCapArea(t, bodies[0], "lofted half", r3.NewVec(0, 0, 1), section)
}

// stepMirrorToothHalf mirrors the lofted half across the mid-body plane.
//
// The mirror's target plane is ctx.helixPlane — the plane the twisted section
// was drawn on, at half the thickness — and not a fresh plane, so the mirrored
// half starts exactly where the lofted half ends. Both bodies are returned:
// the mirror leaves the original in place and adds the reflected copy, which is
// what the combine then joins.
func stepMirrorToothHalf(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	lower, twisted := loftLowerHalf(t, doc, p)
	upper := mirrorAcrossMidPlane(t, doc, twisted, p)
	return []*decad.Body{lower, upper}
}

// assertMirrorToothHalf checks the mirrored half is the reflection of the
// lofted one: the same footprint, the same section at both ends, the same
// volume, and the top half of the body rather than the bottom.
func assertMirrorToothHalf(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	section := toothSectionArea(t, p)
	lower, upper := bodies[0], bodies[1]

	requireSpan(t, lower, "lofted half", 0, thickness/2)
	requireSpan(t, upper, "mirrored half", thickness/2, thickness)
	requireCapArea(t, upper, "mirrored half", r3.NewVec(0, 0, -1), section)
	requireCapArea(t, upper, "mirrored half", r3.NewVec(0, 0, 1), section)

	// A reflection across a plane normal to the axis leaves the footprint
	// alone, so the two halves must occupy the same x-y box. This is the
	// sharp half of the mirror check; the volume comparison below is the
	// blunt half, and the tolerance says why.
	lowerBox, err := lower.Bounds()
	if err != nil {
		t.Fatalf("lofted half bounds: %v", err)
	}
	upperBox, err := upper.Bounds()
	if err != nil {
		t.Fatalf("mirrored half bounds: %v", err)
	}
	tol := 1e-6 * math.Max(1, thickness)
	for _, axis := range []struct {
		name           string
		lowMin, lowMax float64
		upMin, upMax   float64
	}{
		{"x", lowerBox.Min.X, lowerBox.Max.X, upperBox.Min.X, upperBox.Max.X},
		{"y", lowerBox.Min.Y, lowerBox.Max.Y, upperBox.Min.Y, upperBox.Max.Y},
	} {
		if math.Abs(axis.lowMin-axis.upMin) > tol || math.Abs(axis.lowMax-axis.upMax) > tol {
			t.Errorf("mirrored half's %s extent [%.6f, %.6f] differs from the lofted half's [%.6f, %.6f]",
				axis.name, axis.upMin, axis.upMax, axis.lowMin, axis.lowMax)
		}
	}

	lowerVolume := bodyVolume(t, lower, "lofted half")
	upperVolume := bodyVolume(t, upper, "mirrored half")
	if relativeGap(lowerVolume, upperVolume) > mirrorVolumeTolerance {
		t.Errorf("mirrored half is %.9f mm^3 against the lofted half's %.9f mm^3, "+
			"a %.3f%% difference; a mirror changes no volume (sections ruled %s)",
			upperVolume, lowerVolume, 100*relativeGap(lowerVolume, upperVolume), mirrorSectionOrder)
	}
}

// stepCombineToothHalves combines the mirrored half into the lofted one.
//
// In Fusion the combine's target is looked up by name — the body named
// 'Tooth Body' — and the mirrored half, named 'Tooth Body (Mirrored)', is the
// tool; the operation is left at the API default, Join. What has to come out of
// it is one body spanning the full thickness, which is what the inherited
// patternTeeth then circular-patterns.
//
// The boolean itself is beyond this evaluator, and the refusal is specific: two
// bodies that meet exactly on a shared face come within the chord tolerance
// without provably interpenetrating deeper than it, so decad refuses to decide
// whether their surfaces touch or cross rather than answer it wrong. Every
// stacking of these two halves hits it — measured on the exact face contact and
// on an overlapped pair, and on a pair of straight prisms stacked the same way,
// so it is the contact and not the twist. The step therefore attempts the union,
// keeps its result when one comes back, and otherwise proves what a Join of
// these two bodies would have to rest on: that the faces they meet on are the
// same region, in the same plane, with the same area and the same centroid.
func stepCombineToothHalves(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	lower, twisted := loftLowerHalf(t, doc, p)
	upper := mirrorAcrossMidPlane(t, doc, twisted, p)

	joined, err := decad.Union(lower, upper)
	if err == nil {
		t.Logf("union of the two halves built: proving the joined body directly")
		return []*decad.Body{joined}
	}
	if !errors.Is(err, decad.ErrUnsupported) {
		t.Fatalf("union of the two halves failed for a reason this proof does not expect: %v", err)
	}
	t.Logf("union refused, proving the mating faces instead: %v", err)
	return []*decad.Body{lower, upper}
}

// assertCombineToothHalves checks what the Join has to leave behind.
func assertCombineToothHalves(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	thickness := p["thickness"]
	section := toothSectionArea(t, p)

	if len(bodies) == 1 {
		requireSpan(t, bodies[0], "combined tooth body", 0, thickness)
		requireCapArea(t, bodies[0], "combined tooth body", r3.NewVec(0, 0, -1), section)
		requireCapArea(t, bodies[0], "combined tooth body", r3.NewVec(0, 0, 1), section)
		return
	}

	lower, upper := bodies[0], bodies[1]
	// Together the two halves span the full thickness: the tooth the pattern
	// receives is as tall as the gear body the inherited buildBody extrudes.
	requireSpan(t, lower, "lofted half", 0, thickness/2)
	requireSpan(t, upper, "mirrored half", thickness/2, thickness)

	lowerTop := capFace(t, lower, "lofted half", r3.NewVec(0, 0, 1))
	upperBottom := capFace(t, upper, "mirrored half", r3.NewVec(0, 0, -1))

	lowerArea := faceArea(t, lowerTop, "lofted half's top cap")
	upperArea := faceArea(t, upperBottom, "mirrored half's bottom cap")
	if relativeGap(lowerArea, upperArea) > 1e-9 {
		t.Errorf("the halves meet on faces of %.6f mm^2 and %.6f mm^2; a Join of two faces that "+
			"are not the same region leaves a step in the tooth", lowerArea, upperArea)
	}
	if relativeGap(lowerArea, section) > 1e-9 {
		t.Errorf("the halves meet on a %.6f mm^2 face, but the twisted section they share is %.6f mm^2",
			lowerArea, section)
	}
}

// loftLowerHalf draws the two sections and lofts the bottom half of the tooth.
//
// It returns the body and the twisted mid-plane section, which the mirror needs
// again: the mirrored half is the same section pair in the opposite order, so
// reusing this sketch is what makes the two halves meet on one shared region
// rather than on two independently drawn ones.
func loftLowerHalf(t *testing.T, doc *decad.Document, p map[string]float64) (*decad.Body, toothSection) {
	world := sketch.NewWorld()
	bottom := drawToothSection(t, world, 0, 0, p)
	twisted := drawToothSection(t, world, p["thickness"]/2, p["helixAngle"], p)

	// Bottom section first, then the top, which is the order loftTooth adds
	// them in.
	body, err := doc.Loft(bottom.sketch, bottom.profile, twisted.sketch, twisted.profile)
	if err != nil {
		t.Fatalf("loft the bottom half of the tooth: %v", err)
	}
	return body, twisted
}

// mirrorAcrossMidPlane builds the reflection of the lofted half.
//
// The reflection of a ruled loft across its own top plane is the same pair of
// sections ruled again half a thickness higher, so the mirrored half is lofted
// between an untwisted section on the far face and the twisted mid-plane
// section — the very sketch the lofted half ends on, which is what makes the
// two halves meet on one shared region rather than on two drawn separately.
func mirrorAcrossMidPlane(t *testing.T, doc *decad.Document, twisted toothSection, p map[string]float64) *decad.Body {
	top := drawToothSection(t, twisted.sketch.World(), p["thickness"], 0, p)
	// Sections in mirrorSectionOrder: the untwisted far one first, the twisted
	// mid one second, which is the order the lofted half was ruled in.
	body, err := doc.Loft(top.sketch, top.profile, twisted.sketch, twisted.profile)
	if err != nil {
		t.Fatalf("mirror the lofted half across the mid-body plane: %v", err)
	}
	return body
}

// toothSection is one loft section: the sketch, the region the loft consumes,
// and that region's area.
type toothSection struct {
	sketch  *sketch.Sketch
	profile *sketch.Profile
	area    float64
}

// drawToothSection draws the tooth cross-section on a plane offset by z,
// rotated by angle.
//
// The geometry is placed rather than constrained: every point is fixed at the
// position the tooth generator computes for it. The constraint scheme that
// reaches those positions in Fusion is what the sketch step proves, and
// repeating it here would prove it twice while telling the loft nothing.
//
// The flanks are chorded — see this file's header for why, and for what the
// substitution costs.
func drawToothSection(t *testing.T, world *sketch.World, z, angle float64, p map[string]float64) toothSection {
	plane := world.XY()
	if z != 0 {
		var err error
		plane, err = world.CreateOffsetPlane(world.XY(), z)
		if err != nil {
			t.Fatalf("section plane at z=%.4f: %v", z, err)
		}
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("section sketch at z=%.4f: %v", z, err)
	}

	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	if d.Embedded() {
		proofkit3d.Unmodelled(t, "the inherited loftTooth has no embedded branch, so this gear has no lofted tooth")
	}
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["toothNumber"], int(p["involuteSteps"]), angle)

	onRoot := func(sample involute.Pt) involute.Pt {
		radius := math.Hypot(sample.X, sample.Y)
		return involute.Pt{X: sample.X * d.Root / radius, Y: sample.Y * d.Root / radius}
	}
	leftRoot, rightRoot := onRoot(left[0]), onRoot(right[0])

	// The loop, walked counter-clockwise: the root arc under the tooth, the
	// left flank-to-root stub, the left flank, the tooth-top arc, and the
	// right flank back down to the right stub.
	loop := []involute.Pt{rightRoot}
	loop = append(loop, arcChords(rightRoot, leftRoot, d.Root)...)
	loop = append(loop, leftRoot)
	loop = append(loop, left...)
	top := arcChords(right[len(right)-1], left[len(left)-1], d.Tip)
	for i := len(top) - 1; i >= 0; i-- {
		loop = append(loop, top[i])
	}
	for i := len(right) - 1; i >= 0; i-- {
		loop = append(loop, right[i])
	}

	vertices := make([]*sketch.Point, len(loop))
	for i, vertex := range loop {
		vertices[i] = s.CreatePoint(vertex.X, vertex.Y)
		s.Fix(vertices[i])
	}
	for i := range vertices {
		s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
	}

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the section at z=%.4f: %v", z, err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("the section at z=%.4f closes %d region(s), want the one tooth loop", z, len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatalf("the section at z=%.4f is not an extrudable region", z)
	}
	return toothSection{sketch: s, profile: profiles[0], area: profiles[0].Area}
}

// arcChordCount is how many chords stand in for each of the section's two
// arcs. Eight keeps the chorded root and tooth-top arcs within a thousandth of
// a millimetre of the true arc on every case in the table, which is well below
// anything the assertions here read.
const arcChordCount = 8

// arcChords returns the interior vertices of the chord chain standing in for
// the arc that runs counter-clockwise from one point to the other at radius.
func arcChords(from, to involute.Pt, radius float64) []involute.Pt {
	start := math.Atan2(from.Y, from.X)
	end := math.Atan2(to.Y, to.X)
	for end < start {
		end += 2 * math.Pi
	}
	chords := make([]involute.Pt, 0, arcChordCount-1)
	for i := 1; i < arcChordCount; i++ {
		at := start + (end-start)*float64(i)/float64(arcChordCount)
		chords = append(chords, involute.Pt{X: radius * math.Cos(at), Y: radius * math.Sin(at)})
	}
	return chords
}

// toothSectionArea is the area of the drawn tooth section, read off a section
// drawn the same way the loft's own sections are.
func toothSectionArea(t *testing.T, p map[string]float64) float64 {
	return drawToothSection(t, sketch.NewWorld(), 0, 0, p).area
}

// requireSpan checks a body occupies exactly the axial run it should.
func requireSpan(t *testing.T, body *decad.Body, name string, low, high float64) {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", name, err)
	}
	tol := 1e-6 * math.Max(1, high)
	if math.Abs(box.Min.Z-low) > tol || math.Abs(box.Max.Z-high) > tol {
		t.Errorf("%s spans z [%.6f, %.6f], want [%.6f, %.6f]", name, box.Min.Z, box.Max.Z, low, high)
	}
}

// requireCapArea checks the planar cap facing dir carries the drawn section.
func requireCapArea(t *testing.T, body *decad.Body, name string, dir r3.Vec, want float64) {
	t.Helper()
	area := faceArea(t, capFace(t, body, name, dir), name+" cap")
	if relativeGap(area, want) > 1e-9 {
		t.Errorf("%s cap facing %v is %.6f mm^2, want the drawn tooth section's %.6f mm^2",
			name, dir, area, want)
	}
}

func capFace(t *testing.T, body *decad.Body, name string, dir r3.Vec) *decad.Face {
	t.Helper()
	faces, err := decad.Faces(decad.Planar(), decad.Facing(dir)).Exactly(1).SelectFaces(body)
	if err != nil {
		t.Fatalf("%s: select the planar cap facing %v: %v", name, dir, err)
	}
	return faces[0]
}

func faceArea(t *testing.T, face *decad.Face, name string) float64 {
	t.Helper()
	area, err := face.Area()
	if err != nil {
		t.Fatalf("%s area: %v", name, err)
	}
	return area.Value.Base()
}

func bodyVolume(t *testing.T, body *decad.Body, name string) float64 {
	t.Helper()
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", name, err)
	}
	return volume.Value.Base()
}

func relativeGap(a, b float64) float64 {
	scale := math.Max(math.Abs(a), math.Abs(b))
	if scale == 0 {
		return 0
	}
	return math.Abs(a-b) / scale
}
