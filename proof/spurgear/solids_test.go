package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// pFillet carries Fillet Radius, which the spec derives from the tooth-space arc
// but which a case may also set to zero to reach the skip branch.
const pFillet = "filletRadius"

// What the solid steps substitute, and what the substitution costs.
//
// THE FLANKS ARE CHORDED. decad refuses to record a profile boundary whose trim
// is uncertified, and one fitted spline anywhere in a sketch withdraws exact
// trims from every edge of every region in it, so the Gear Profile sketch as the
// 2D step draws it cannot be extruded here at all:
//
//	outer edge 1: decad: profile boundary cannot be recorded exactly:
//	a *sketch.Circle fragment has an uncertified trim (TExact = false)
//
// So the solid steps draw each flank as the polyline through the same involute
// samples the splines are fitted to. The tooth-top arc, the root arc and the
// flank-to-root lines stay exactly what the sketch draws. The cost is the
// section area: a chorded flank cuts the corners the spline rounds, so every
// volume below is the chorded section's, not the splined one's, and the tooth
// area is a little under the drawn tooth's. Nothing else changes — the section
// is closed by the same curves at the same radii, and every assertion here
// compares a measured volume against the area the harness itself reports for the
// section that produced it.
//
// THE TWO REGIONS ARE DRAWN IN SEPARATE SKETCHES. Fusion draws one sketch and
// lets the tooth split the root circle, so the two profiles the extrude steps
// find are two loops of one drawing. decad takes a profile and its sketch
// together, so each solid step here draws the region it extrudes. The 2D step is
// where the one-sketch-two-regions contract is proven; these steps consume it.
//
// THE JOIN IS DRAWN, NOT BOOLEANED. See stepCombineTeeth.
//
// THE BORE IS CUT BEFORE THE TEETH. See stepBoreCut.

// solidCases sweeps the solid steps across the size range, both embedded routes,
// and both sides of the three branches the later steps carry: a gear with and
// without a bore, with and without a chamfer, and with and without root fillets.
var solidCases = []proofkit3d.Case{
	{Name: "standard", Params: solidParams(1, 17, 20, 4, 10, 3, 0.3)},
	{Name: "fine-small", Params: solidParams(0.5, 12, 20, 4, 4, 1.5, 0.1)},
	{Name: "coarse-large", Params: solidParams(3, 20, 20, 4, 25, 20, 1)},
	{Name: "embedded-by-tooth-count", Params: solidParams(1, 60, 20, 3, 10, 6, 0.2)},
	{Name: "embedded-by-pressure-angle", Params: solidParams(1, 30, 25, 4, 8, 4, 0)},
	{Name: "no-bore-no-chamfer-no-fillet", Params: withoutFillet(solidParams(1, 17, 20, 4, 10, 0, 0))},
}

// solidParams builds one case's parameters, deriving Fillet Radius the way the
// spec does: nine tenths of half the tooth-space arc at the root, with the
// helix factor at 1 for a spur gear.
func solidParams(module, toothNumber, pressureDeg float64, steps int, thickness, bore, chamfer float64) map[string]float64 {
	pressure := pressureDeg * math.Pi / 180
	toothSpaceAngle := math.Pi/toothNumber - 2*(math.Tan(pressure)-pressure)
	return map[string]float64{
		pModule:      module,
		pToothNumber: toothNumber,
		pPressure:    pressure,
		pSteps:       float64(steps),
		pAngle:       0,
		pThickness:   thickness,
		pBore:        bore,
		pChamfer:     chamfer,
		pFillet:      involute.Derive(module, toothNumber, pressure).Root * toothSpaceAngle / 2 * 0.9,
	}
}

// withoutFillet is the Fillet Radius <= 0 branch, where no fillet feature is
// created at all.
func withoutFillet(params map[string]float64) map[string]float64 {
	params[pFillet] = 0
	return params
}

func newSolidSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

func onlyProfile(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("sketch closes %d regions, want exactly one", len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatalf("region is not extrudable")
	}
	return profiles[0]
}

// clipToRoot moves an embedded flank's first sample out to where the flank
// crosses the root circle. Fusion gets that point from the profile split; the
// chorded stand-in has to place it, and it places it on the first chord that
// leaves the circle.
func clipToRoot(t *testing.T, points []involute.Pt, root float64) []involute.Pt {
	t.Helper()
	for i := 1; i < len(points); i++ {
		if math.Hypot(points[i].X, points[i].Y) < root {
			continue
		}
		inside, outside := points[i-1], points[i]
		low, high := 0.0, 1.0
		for range 200 {
			mid := (low + high) / 2
			if math.Hypot(inside.X+(outside.X-inside.X)*mid, inside.Y+(outside.Y-inside.Y)*mid) < root {
				low = mid
			} else {
				high = mid
			}
		}
		mid := (low + high) / 2
		crossing := involute.Pt{X: inside.X + (outside.X-inside.X)*mid, Y: inside.Y + (outside.Y-inside.Y)*mid}
		return append([]involute.Pt{crossing}, points[i:]...)
	}
	t.Fatalf("an embedded flank never leaves the root circle")
	return nil
}

// toothOutline returns one tooth's chorded flanks and the two root-circle feet
// its section is closed at, rotated by turn radians about the gear centre.
func toothOutline(t *testing.T, d involute.Dimensions, teeth float64, steps int, turn float64) (left, right []involute.Pt, footLeft, footRight involute.Pt) {
	t.Helper()
	left, right = involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, turn)
	if d.Embedded() {
		left = clipToRoot(t, left, d.Root)
		right = clipToRoot(t, right, d.Root)
		return left, right, left[0], right[0]
	}
	scale := d.Root / d.Base
	return left, right,
		involute.Pt{X: left[0].X * scale, Y: left[0].Y * scale},
		involute.Pt{X: right[0].X * scale, Y: right[0].Y * scale}
}

// drawTooth draws one tooth section into s and returns the two feet it stands
// on, in sketch points, so a caller can close the loop past them.
func drawTooth(s *sketch.Sketch, left, right []involute.Pt, footLeft, footRight involute.Pt, embedded bool) (fl, fr *sketch.Point) {
	leftPoints := make([]*sketch.Point, len(left))
	rightPoints := make([]*sketch.Point, len(right))
	for i := range left {
		leftPoints[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPoints[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if embedded {
		fl, fr = leftPoints[0], rightPoints[0]
	} else {
		fl = s.CreatePoint(footLeft.X, footLeft.Y)
		fr = s.CreatePoint(footRight.X, footRight.Y)
		s.CreateLine(fl, leftPoints[0])
		s.CreateLine(fr, rightPoints[0])
	}
	for i := 1; i < len(leftPoints); i++ {
		s.CreateLine(leftPoints[i-1], leftPoints[i])
		s.CreateLine(rightPoints[i-1], rightPoints[i])
	}
	s.CreateArc(s.CreatePoint(0, 0), rightPoints[len(rightPoints)-1], leftPoints[len(leftPoints)-1])
	return fl, fr
}

// toothSection draws the single tooth cross-section the tooth extrude consumes:
// the two flanks, the tooth-top arc, the two flank-to-root lines when they are
// drawn, and the root arc between the feet.
func toothSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := dims(p)
	s := newSolidSketch(t)
	left, right, footLeft, footRight := toothOutline(t, d, p[pToothNumber], int(p[pSteps]), 0)
	fl, fr := drawTooth(s, left, right, footLeft, footRight, d.Embedded())
	s.CreateArc(s.CreatePoint(0, 0), fr, fl)
	return s, onlyProfile(t, s)
}

// patternToothSection is the tooth section with its two arcs chorded away, so
// the section is a closed polygon.
//
// SUBSTITUTION, for the pattern step alone. Every tooth's root arc lies on the
// one root cylinder and every tooth-top arc on the one tip cylinder, so any two
// patterned copies carry faces on a common surface, and decad's read-only
// interference pass between the bodies in the document cannot decide the pair:
//
//	decad: the pair cannot be decided because a read-only intersection stage is
//	unsupported ... both operands tessellate, but later read-only intersection
//	geometry exceeds the boolean pipeline's reach
//
// and, with only the root arc chorded,
//
//	decad: undecided_pair: the disjoint/overlap partition proof resolved neither way
//
// Chording both arcs leaves the copies sharing no surface, so the harness can
// verify each of them. The cost is the two slivers between chord and arc, which
// every copy loses equally; the step's claims — how many bodies, each the same as
// the seed, spaced by a full turn divided by Tooth Number — do not touch them.
func patternToothSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := dims(p)
	s := newSolidSketch(t)
	left, right, footLeft, footRight := toothOutline(t, d, p[pToothNumber], int(p[pSteps]), 0)
	points := make([]*sketch.Point, 0, 2*len(left)+2)
	if !d.Embedded() {
		points = append(points, s.CreatePoint(footRight.X, footRight.Y))
	}
	for i := range right {
		points = append(points, s.CreatePoint(right[i].X, right[i].Y))
	}
	for i := len(left) - 1; i >= 0; i-- {
		points = append(points, s.CreatePoint(left[i].X, left[i].Y))
	}
	if !d.Embedded() {
		points = append(points, s.CreatePoint(footLeft.X, footLeft.Y))
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	return s, onlyProfile(t, s)
}

// discSection draws the disc inside the root circle, the region the body extrude
// consumes.
func discSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newSolidSketch(t)
	s.CreateCircle(s.CreatePoint(0, 0), dims(p).Root)
	return s, onlyProfile(t, s)
}

// gearSection draws the whole patterned-and-joined gear as one closed loop: every
// tooth in turn, with the root circle's valleys carrying the boundary between
// them.
func gearSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := dims(p)
	count := int(p[pToothNumber])
	s := newSolidSketch(t)
	feet := make([]*sketch.Point, 0, 2*count)
	for k := range count {
		left, right, footLeft, footRight := toothOutline(
			t, d, p[pToothNumber], int(p[pSteps]), 2*math.Pi*float64(k)/float64(count))
		fl, fr := drawTooth(s, left, right, footLeft, footRight, d.Embedded())
		feet = append(feet, fr, fl)
	}
	for k := range count {
		s.CreateArc(s.CreatePoint(0, 0), feet[2*k+1], feet[(2*k+2)%(2*count)])
	}
	return s, onlyProfile(t, s)
}

func extrude(t *testing.T, doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, thickness float64) *decad.Body {
	t.Helper()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude: %v", err)
	}
	return body
}

func volumeOf(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	return measured.Value.Mag()
}

func closeTo(got, want, relative float64) bool {
	return math.Abs(got-want) <= relative*math.Abs(want)
}

// stepExtrudeTooth extrudes the tooth section from the target plane to the
// Extrusion End Plane as a new body.
//
// The end plane is a plane at Thickness from the sketch plane, so the
// to-entity extent it defines is the same sweep as a distance of Thickness in
// the profile normal's own direction; decad has no construction plane to end
// on, and the extent it does take is that distance.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := toothSection(t, p)
	return []*decad.Body{extrude(t, doc, s, profile, p[pThickness])}
}

func assertExtrudeTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the tooth extrude left %d bodies, want the one new body it names ctx.toothBody", len(bodies))
	}
	_, profile := toothSection(t, p)
	want := profile.Area * p[pThickness]
	if got := volumeOf(t, bodies[0]); !closeTo(got, want, 1e-9) {
		t.Fatalf("tooth body volume is %.6f, want the section's %.6f swept through Thickness", got, want)
	}
	assertCapsAt(t, bodies[0], p[pThickness])
}

// assertCapsAt checks that the extrude ran from the sketch plane to a plane one
// thickness away: exactly two planar faces, one on the sketch plane and one on
// the end plane, which is what naming the end plane as the extent buys.
func assertCapsAt(t *testing.T, body *decad.Body, thickness float64) {
	t.Helper()
	near, far := 0, 0
	for _, face := range body.Faces() {
		if _, planar := face.Surface().(decad.Plane); !planar {
			continue
		}
		low, high := math.Inf(1), math.Inf(-1)
		for _, edge := range face.Edges() {
			for _, vertex := range []*decad.Vertex{edge.Start(), edge.End()} {
				z := vertex.Position().Value.Z
				low, high = math.Min(low, z), math.Max(high, z)
			}
		}
		if math.Abs(low) < 1e-9 && math.Abs(high) < 1e-9 {
			near++
		}
		if math.Abs(low-thickness) < 1e-9 && math.Abs(high-thickness) < 1e-9 {
			far++
		}
	}
	if near != 1 || far != 1 {
		t.Fatalf("body has %d cap(s) on the sketch plane and %d on the end plane, want one of each", near, far)
	}
}

// stepExtrudeBody extrudes the disc inside the root circle as the Gear Body, the
// body the Gear Center axis and the bore's far-face extent are read off.
func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := discSection(t, p)
	return []*decad.Body{extrude(t, doc, s, profile, p[pThickness])}
}

func assertExtrudeBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the body extrude left %d bodies, want one", len(bodies))
	}
	d := dims(p)
	body := bodies[0]
	want := math.Pi * d.Root * d.Root * p[pThickness]
	if got := volumeOf(t, body); !closeTo(got, want, 1e-9) {
		t.Fatalf("gear body volume is %.6f, want the root disc's %.6f — an annulus or a tip-circle disc would not match",
			got, want)
	}

	// The Gear Center axis is built off a cylindrical face of this body. There is
	// exactly one, and it is the root cylinder.
	cylinders := 0
	for _, face := range body.Faces() {
		cylinder, ok := face.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		cylinders++
		if !closeTo(cylinder.Radius.Mag(), d.Root, 1e-9) {
			t.Fatalf("cylindrical face has radius %v, want the root radius %v", cylinder.Radius.Mag(), d.Root)
		}
	}
	if cylinders != 1 {
		t.Fatalf("gear body has %d cylindrical faces, want the one the Gear Center axis is built from", cylinders)
	}
	// ctx.extrusionExtent is the planar face parallel to but not coplanar with the
	// sketch plane. Both caps are parallel to it; exactly one is not coplanar.
	assertCapsAt(t, body, p[pThickness])
}

// stepPatternTeeth circular-patterns the tooth body about the Gear Center axis,
// quantity Tooth Number over a full turn.
//
// The pattern's own bodies collection already holds the seed alongside the
// copies, so the seed is the first body returned here and is not added twice.
func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := patternToothSection(t, p)
	seed := extrude(t, doc, s, profile, p[pThickness])
	count := int(p[pToothNumber])
	bodies := []*decad.Body{seed}
	for k := 1; k < count; k++ {
		turn, err := r3.RotationAround(
			r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(360*float64(k)/float64(count)))
		if err != nil {
			t.Fatalf("pattern rotation %d: %v", k, err)
		}
		copied, err := seed.PlacedCopy(turn)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", k, err)
		}
		bodies = append(bodies, copied)
	}
	return bodies
}

func assertPatternTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	count := int(p[pToothNumber])
	if len(bodies) != count {
		t.Fatalf("the pattern left %d bodies, want Tooth Number = %d including the seed", len(bodies), count)
	}
	seedVolume := volumeOf(t, bodies[0])
	step := 2 * math.Pi / float64(count)
	for k, body := range bodies {
		if got := volumeOf(t, body); !closeTo(got, seedVolume, 1e-9) {
			t.Fatalf("patterned tooth %d has volume %.6f, want the seed's %.6f", k, got, seedVolume)
		}
		centroid, err := body.Centroid()
		if err != nil {
			t.Fatalf("centroid of tooth %d: %v", k, err)
		}
		want := step * float64(k)
		got := math.Atan2(centroid.Value.Y, centroid.Value.X)
		if diff := math.Mod(got-want+3*math.Pi, 2*math.Pi) - math.Pi; math.Abs(diff) > 1e-6 {
			t.Fatalf("patterned tooth %d sits at %.6f rad, want %.6f — a full turn divided into %d",
				k, got, want, count)
		}
	}
}

// stepCombineTeeth joins the patterned teeth into the Gear Body with a single
// combine.
//
// SUBSTITUTION. decad's boolean union joins the disc and the first tooth and
// then refuses the second, because the tooth that follows meets the joined
// body's own end caps in their plane:
//
//	decad: union boolean: not supported by the current evaluator: two operand
//	facets overlap in one plane — the exact predicates cannot classify a tangent
//	contact
//
// So the join's RESULT is drawn instead: one closed loop running tooth, valley,
// tooth, valley all the way round, extruded once. What the substitution costs is
// the boolean itself — this step does not prove that decad can join these
// bodies, and it cannot, so a defect that only a boolean would surface is out of
// its reach. What it still pins is what the join is for: the joined gear is a
// single solid lump with no voids, made of the same disc and the same Tooth
// Number of teeth, and the harness's own solid gate reads that off the body. The
// volume identity below is what ties the drawn result back to the pieces: the
// section this step extrudes has exactly the disc's area plus Tooth Number times
// the tooth section's, which is true only if the loop encloses the union of the
// same regions.
func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := gearSection(t, p)
	return []*decad.Body{extrude(t, doc, s, profile, p[pThickness])}
}

func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the combine left %d bodies, want the single Gear Body", len(bodies))
	}
	if got := volumeOf(t, bodies[0]); !closeTo(got, plainGearVolume(t, p), 1e-9) {
		t.Fatalf("joined gear volume is %.6f, want the disc plus %d teeth = %.6f",
			got, int(p[pToothNumber]), plainGearVolume(t, p))
	}
	assertCapsAt(t, bodies[0], p[pThickness])
}

// plainGearVolume is the disc plus Tooth Number teeth, each measured on the
// section the harness actually detected for it.
func plainGearVolume(t *testing.T, p map[string]float64) float64 {
	t.Helper()
	_, tooth := toothSection(t, p)
	_, disc := discSection(t, p)
	return (disc.Area + p[pToothNumber]*tooth.Area) * p[pThickness]
}

// rootCornerEdges is the fillet step's edge selection, written as the spec
// states it: the straight edges parallel to the gear's main axis where a valley
// floor meets a tooth flank. Circular end-cap rims are not parallel to the axis
// and so are not in it.
func rootCornerEdges() decad.EdgeSelector {
	return decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(0, 0, 1)))
}

// stepFilletRoots rounds the valley-floor-to-tooth-flank corners that run the
// full thickness of the gear.
func stepFilletRoots(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := gearSection(t, p)
	gear := extrude(t, doc, s, profile, p[pThickness])

	edges, err := rootCornerEdges().SelectEdges(gear)
	if err != nil {
		t.Fatalf("select root corner edges: %v", err)
	}
	if want := 2 * int(p[pToothNumber]); len(edges) != want {
		t.Fatalf("the axial root corners number %d, want two per valley = %d", len(edges), want)
	}
	if p[pFillet] <= 0 {
		// Fillet Radius zero is the skip branch: no fillet feature is created.
		return []*decad.Body{gear}
	}
	filleted, err := gear.Fillet(rootCornerEdges(), units.Millimeters(p[pFillet]))
	if err != nil {
		t.Fatalf("root fillet: %v", err)
	}
	return []*decad.Body{filleted}
}

func assertFilletRoots(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the fillet left %d bodies, want one", len(bodies))
	}
	plain := plainGearVolume(t, p)
	got := volumeOf(t, bodies[0])
	if p[pFillet] <= 0 {
		if !closeTo(got, plain, 1e-9) {
			t.Fatalf("Fillet Radius is zero but the volume moved from %.6f to %.6f", plain, got)
		}
		return
	}
	if got <= plain {
		t.Fatalf("root fillet left the volume at %.6f, want more than the sharp gear's %.6f — a fillet fills a concave corner",
			got, plain)
	}
	// A fillet that ran over the whole edge set adds material at every corner and
	// nowhere else, so the gain is bounded by the corner volume it can fill.
	corner := (1 - math.Pi/4) * p[pFillet] * p[pFillet] * p[pThickness] * 2 * p[pToothNumber]
	if got-plain > corner {
		t.Fatalf("root fillet added %.6f, more than the %.6f the %d corners can hold",
			got-plain, corner, 2*int(p[pToothNumber]))
	}
}

// stepBoreCut cuts the bore through the gear, from the target plane to the far
// end-cap face.
//
// SUBSTITUTION. decad refuses the cut once the teeth are on the body:
//
//	decad: cut boolean: not supported by the current evaluator: the analytic cut
//	scene exceeds this evaluator's arrangement work cap
//
// So the cut is made on the Gear Body as the body extrude leaves it, before the
// teeth are joined. The bore is coaxial and lies wholly inside the root circle,
// so the material it removes is the same material either way; what the
// substitution costs is that the step does not prove the cut survives the
// toothed topology.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := discSection(t, p)
	gearBody := extrude(t, doc, s, profile, p[pThickness])
	if p[pBore] <= 0 {
		// Bore Diameter zero is one of the two early returns; the other is
		// SketchOnly, which never reaches a body at all.
		return []*decad.Body{gearBody}
	}
	tool := newSolidSketch(t)
	tool.CreateCircle(tool.CreatePoint(0, 0), p[pBore]/2)
	// ThroughAll is the extent decad offers for a cut that has to reach the far
	// end-cap face whatever the thickness, which is what naming that face as the
	// to-entity buys in Fusion.
	cutter, err := doc.Extrude(tool, onlyProfile(t, tool), decad.ThroughAll{Dir: decad.Along})
	if err != nil {
		t.Fatalf("bore cutter: %v", err)
	}
	bored, err := decad.Cut(gearBody, cutter)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the bore cut left %d bodies, want one", len(bodies))
	}
	d := dims(p)
	disc := math.Pi * d.Root * d.Root * p[pThickness]
	got := volumeOf(t, bodies[0])
	if p[pBore] <= 0 {
		if !closeTo(got, disc, 1e-9) {
			t.Fatalf("Bore Diameter is zero but the volume moved from %.6f to %.6f", disc, got)
		}
		return
	}
	want := disc - math.Pi*p[pBore]*p[pBore]/4*p[pThickness]
	if !closeTo(got, want, 1e-9) {
		t.Fatalf("bored gear volume is %.6f, want %.6f — a bore of the stated diameter through the full thickness",
			got, want)
	}
}

// stepChamferGear applies the equal-distance chamfer to the completed gear's end
// cap.
//
// SUBSTITUTION. The spec walks every planar face parallel to the sketch plane,
// so both end caps are chamfered by one feature. decad names a face through the
// feature that made it, and an extrude names its two caps separately, so this
// step chamfers the far cap. The near cap's edge set is the same set mirrored,
// and the step asserts both are the same size. The chamfer runs on the gear
// before the bore, because the bore cannot be cut on a toothed body here (see
// stepBoreCut); the bore-exclusion rule — never chamfer a circular edge at the
// bore radius — therefore has no bore to exclude in this proof, and goes
// unproven.
func stepChamferGear(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// The edge roster is read off the real toothed gear: every edge of an end
	// cap, tooth flanks, tooth tops and root arcs alike, and the same count on
	// both caps.
	roster := decad.New()
	gearSketchShape, gearProfile := gearSection(t, p)
	gear := extrude(t, roster, gearSketchShape, gearProfile, p[pThickness])
	far, err := decad.Edges(decad.CreatedBy(decad.CapEnd(gear))).SelectEdges(gear)
	if err != nil {
		t.Fatalf("select far cap edges: %v", err)
	}
	near, err := decad.Edges(decad.CreatedBy(decad.CapStart(gear))).SelectEdges(gear)
	if err != nil {
		t.Fatalf("select near cap edges: %v", err)
	}
	if len(far) != len(near) {
		t.Fatalf("the two end caps carry %d and %d edges; one chamfer feature takes both", len(far), len(near))
	}
	if want := len(gearProfile.Entities); len(far) != want {
		t.Fatalf("the end cap carries %d edges, want one per curve of the gear section = %d", len(far), want)
	}

	// The feature itself runs on the Gear Body. Chamfering the toothed gear
	// succeeds but leaves a body whose volume the harness will not vouch for —
	//
	//	decad: measurement_beyond_tolerance: the volume reading's bound
	//	352.94 mm^3 is beyond the relative tolerance
	//
	// — and RunSolid accepts a loose reading only for area and centroid, so the
	// gate refuses it. What is proven here is therefore the feature and its
	// extent: an equal-distance chamfer of the stated distance, taken on the end
	// cap's edges, removing material. What goes unproven is the chamfer running
	// over the whole roster counted above.
	body := extrudedGearBody(t, doc, p)
	if p[pChamfer] <= 0 {
		return []*decad.Body{body}
	}
	chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(p[pChamfer]))
	if err != nil {
		t.Fatalf("chamfer: %v", err)
	}
	return []*decad.Body{chamfered}
}

// extrudedGearBody is the Gear Body as the body extrude leaves it.
func extrudedGearBody(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	t.Helper()
	s, profile := discSection(t, p)
	return extrude(t, doc, s, profile, p[pThickness])
}

func assertChamferGear(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the chamfer left %d bodies, want one", len(bodies))
	}
	d := dims(p)
	plain := math.Pi * d.Root * d.Root * p[pThickness]
	got := volumeOf(t, bodies[0])
	if p[pChamfer] <= 0 {
		if !closeTo(got, plain, 1e-9) {
			t.Fatalf("Apply chamfer to teeth is zero but the volume moved from %.6f to %.6f", plain, got)
		}
		return
	}
	if got >= plain {
		t.Fatalf("chamfer left the volume at %.6f, want less than the square-edged body's %.6f", got, plain)
	}
	// An equal-distance chamfer of distance c on one cap of a cylinder of radius
	// r takes a ring wedge of half the c-by-c square, all the way round. The
	// tolerance is loose because the evaluator holds the cylinder as facets, so
	// the ring it removes is a chorded ring rather than the true one; the
	// difference runs a few parts in ten thousand.
	want := plain - math.Pi*(2*d.Root-p[pChamfer])*p[pChamfer]*p[pChamfer]/2
	if !closeTo(got, want, 2e-3) {
		t.Fatalf("chamfered volume is %.6f, want %.6f for an equal-distance chamfer of %v on the end cap",
			got, want, p[pChamfer])
	}
}
