// The solid steps of spec/spurgear/steps.md, proven in decad.
//
// Every build here extrudes the chorded scaffold sections of
// geometry_test.go; the two substitutions that forces (chords for the
// involute splines, an explicit root arc for the split-derived one) are
// recorded on chordedToothProfile, and what each step's substitute still
// pins is stated on that step.
//
// The Extrusion End Plane (step 8) has no decad counterpart; its one job —
// ending the tooth and body extrudes on the same face — is modelled by
// giving both extrudes the same distance, and the far-cap assertions below
// check both bodies end at Thickness.
package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func mm(v units.Value, t *testing.T) float64 {
	t.Helper()
	x, err := v.In(units.Millimeter)
	if err != nil {
		t.Fatalf("convert to mm: %v", err)
	}
	return x
}

func mm3(v units.Value, t *testing.T) float64 {
	t.Helper()
	x, err := v.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("convert to mm^3: %v", err)
	}
	return x
}

// capFaces returns the body's planar faces whose plane is parallel to the
// sketch plane and sits at the given z height — the same
// parallel-then-height classification step 13 uses on face.geometry.
func capFaces(body *decad.Body, z float64) []*decad.Face {
	var out []*decad.Face
	for _, f := range body.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		n := pl.Frame.N()
		if math.Abs(math.Abs(n.Z)-1) > 1e-9 {
			continue // not parallel to the sketch plane
		}
		if math.Abs(pl.Frame.Origin().Z-z) < 1e-9 {
			out = append(out, f)
		}
	}
	return out
}

// classifyEdges counts an edge list by curve kind.
func classifyEdges(edges []*decad.Edge) (lines, arcs, other int) {
	for _, e := range edges {
		switch e.Curve().(type) {
		case decad.Line3:
			lines++
		case decad.Arc3, decad.Circle3:
			arcs++
		default:
			other++
		}
	}
	return
}

// toothCases covers both tooth shapes and both build sizes.
var toothCases = []proofkit3d.Case{
	{Name: "default m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
	{Name: "coarse m2 z13 thin", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5}},
	{Name: "embedded z50", Params: map[string]float64{
		pModule: 1, pToothNumber: 50, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
}

// stepExtrudeTooth models step 11: the single tooth section, found by its
// curve counts, extruded from the sketch plane to the Extrusion End Plane as
// a new body.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := chordedToothProfile(t, s, g)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude tooth: %v", err)
	}
	return []*decad.Body{body}
}

// assertExtrudeTooth pins what step 12's front-face search will match on:
// the front cap is coplanar with the sketch plane and carries
// chamferWantEdges() == 6 edges for the stubbed tooth (4 when embedded — the
// documented shape that makes an embedded chamfer raise), and exactly one of
// them is an arc on the root circle radius, the one edge the chamfer must
// skip. The extent is checked against Thickness, the end-plane's job.
func assertExtrudeTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	body := bodies[0]
	thickness := params[pThickness]

	s := newXYSketch(t)
	area := chordedToothProfile(t, s, g).Area
	vol, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	if got := mm3(vol.Value, t); relDiff(got, area*thickness) > 1e-6 {
		t.Errorf("tooth volume %.6f, want section area x thickness = %.6f", got, area*thickness)
	}

	front := capFaces(body, 0)
	if len(front) != 1 {
		t.Fatalf("found %d faces coplanar with the sketch plane, want 1", len(front))
	}
	wantEdges, wantLines := 6, 4
	if g.embedded() {
		wantEdges, wantLines = 4, 2
	}
	edges := front[0].Edges()
	lines, arcs, other := classifyEdges(edges)
	if len(edges) != wantEdges || lines != wantLines || arcs != 2 || other != 0 {
		t.Errorf("front face: %d edges (%d lines, %d arcs, %d other), want %d (%d, 2, 0)",
			len(edges), lines, arcs, other, wantEdges, wantLines)
	}
	rootArcs := 0
	for _, e := range edges {
		switch c := e.Curve().(type) {
		case decad.Arc3:
			if math.Abs(mm(c.Radius, t)-g.dims.Root) < 0.01 {
				rootArcs++
			}
		case decad.Circle3:
			if math.Abs(mm(c.Radius, t)-g.dims.Root) < 0.01 {
				rootArcs++
			}
		}
	}
	if rootArcs != 1 {
		t.Errorf("front face has %d edges on the root radius, want exactly 1 (the edge the chamfer skips)", rootArcs)
	}
	if far := capFaces(body, thickness); len(far) != 1 {
		t.Errorf("found %d faces at the extrusion end distance, want 1", len(far))
	}
}

// chamferCases: the chamfer distances against the m1 z17 gear's root disc.
var chamferCases = []proofkit3d.Case{
	{Name: "0.3mm chamfer", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10, pChamferTooth: 0.3}},
	{Name: "0.6mm chamfer", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10, pChamferTooth: 0.6}},
}

// stepChamferTooth models step 12's chamfer feature on substitute geometry:
// an equal-distance chamfer applied to the complete front-cap edge loop of
// the root disc.
//
// Why the substitute: decad's cap chamfer accepts only COMPLETE cap loops,
// and the real step chamfers the tooth's front face minus its root arc — an
// incomplete loop it refuses (SX4). Chamfering the tooth's own complete cap
// loop builds, but the resulting cap-blend body carries a volume reading
// whose proven bound exceeds decad's relative tolerance on a body that
// small, and no gate this proof may use tolerates a volume diagnostic; the
// disc's circular cap loop is the one front-cap chamfer whose band decad
// evaluates exactly. What this still pins: the feature order (chamfer on
// the front cap immediately after the tooth extrude), the equal-distance
// band geometry, and the exact volume it removes. What it cannot pin: the
// edge SELECTION — that the set is the front face's edges minus the root
// arc. The selection's inputs are pinned by assertExtrudeTooth (edge count
// 6, exactly one root-radius arc); the selection itself, and the raise when
// no face matches, stay prose in steps.md step 12.
func stepChamferTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := discProfile(t, s, g.dims.Root)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude disc: %v", err)
	}
	chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapStart(body))),
		units.Millimeters(params[pChamferTooth]))
	if err != nil {
		t.Fatalf("cap chamfer: %v", err)
	}
	return []*decad.Body{chamfered}
}

// assertChamferTooth checks the band removed exactly the equal-distance
// ring: for a 45-degree bevel of depth d on a rim of radius R, the removed
// volume is 2*pi*(R*d^2/2 - d^3/6).
func assertChamferTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	d := params[pChamferTooth]
	R := g.dims.Root
	thickness := params[pThickness]
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	want := math.Pi*R*R*thickness - 2*math.Pi*(R*d*d/2-d*d*d/6)
	if got := mm3(vol.Value, t); relDiff(got, want) > 1e-9 {
		t.Errorf("chamfered volume %.9f, want %.9f", got, want)
	}
	cones := 0
	for _, f := range bodies[0].Faces() {
		if _, ok := f.Surface().(decad.Cone); ok {
			cones++
		}
	}
	if cones != 1 {
		t.Errorf("chamfer left %d conical band faces, want 1", cones)
	}
}

// bodyCases: the gear body disc at both sizes.
var bodyCases = []proofkit3d.Case{
	{Name: "default m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
	{Name: "coarse m2 z13 thin", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5}},
}

// stepExtrudeBody models step 13: the disc inside the root circle, extruded
// from the sketch plane to the Extrusion End Plane as a new body.
func stepExtrudeBody(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := discProfile(t, s, g.dims.Root)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude body: %v", err)
	}
	return []*decad.Body{body}
}

// assertExtrudeBody pins the body's shape and the face census step 13
// classifies: one cylindrical lateral face at the root radius, and exactly
// two planar caps of which one is coplanar with the sketch plane and one is
// parallel-but-not-coplanar at Thickness.
func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	thickness := params[pThickness]
	body := bodies[0]
	vol, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	want := math.Pi * g.dims.Root * g.dims.Root * thickness
	if got := mm3(vol.Value, t); relDiff(got, want) > 1e-9 {
		t.Errorf("body volume %.6f, want pi*root^2*thickness = %.6f", got, want)
	}
	planes, cylinders := 0, 0
	for _, f := range body.Faces() {
		switch sf := f.Surface().(type) {
		case decad.Plane:
			planes++
		case decad.Cylinder:
			cylinders++
			if math.Abs(mm(sf.Radius, t)-g.dims.Root) > 1e-9 {
				t.Errorf("lateral cylinder radius %.6f, want root radius %.6f", mm(sf.Radius, t), g.dims.Root)
			}
		}
	}
	if planes != 2 || cylinders != 1 {
		t.Errorf("face census: %d planes, %d cylinders; want 2 and 1", planes, cylinders)
	}
	if len(capFaces(body, 0)) != 1 {
		t.Errorf("no single cap coplanar with the sketch plane")
	}
	if len(capFaces(body, thickness)) != 1 {
		t.Errorf("no single far cap at Thickness (the parallel-not-coplanar pick)")
	}
}

// stepGearCenterAxis models step 14: while iterating the new body's faces,
// the cylindrical face yields the Gear Center axis and the far planar cap
// becomes ctx.extrusionExtent.
func stepGearCenterAxis(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := discProfile(t, s, g.dims.Root)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude body: %v", err)
	}
	return []*decad.Body{body}
}

// assertGearCenterAxis derives the axis the way setByCircularFace does —
// from the cylindrical face — and checks it is the gear centre: parallel to
// the sketch-plane normal and passing through the anchor.
func assertGearCenterAxis(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	var cyl *decad.Cylinder
	for _, f := range bodies[0].Faces() {
		if c, ok := f.Surface().(decad.Cylinder); ok {
			cyl = &c
		}
	}
	if cyl == nil {
		t.Fatalf("no cylindrical face to build the Gear Center axis from")
	}
	normal := r3.Vec{Z: 1}
	dot := cyl.Axis.X*normal.X + cyl.Axis.Y*normal.Y + cyl.Axis.Z*normal.Z
	if math.Abs(math.Abs(dot)-1) > 1e-9 {
		t.Errorf("Gear Center axis %v is not parallel to the sketch-plane normal", cyl.Axis)
	}
	if math.Hypot(cyl.Origin.X, cyl.Origin.Y) > 1e-9 {
		t.Errorf("Gear Center axis passes through (%.9f, %.9f), want the gear centre", cyl.Origin.X, cyl.Origin.Y)
	}
}

// patternCases: the tooth placements sampled from the full pattern.
var patternCases = []proofkit3d.Case{
	{Name: "default m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
	{Name: "coarse m2 z13", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5}},
}

// patternSample is the slot subset stepPatternTeeth places: the seed, its
// immediate neighbour (the closest spacing in the pattern), and two spread
// slots. decad's pairwise disjointness proof goes undecided when many
// near-tangent tooth prisms coexist in one document (around ten consecutive
// slots), so the full quantity cannot be placed here; the full-count fact is
// carried by stepCombineTeeth's exact tiling equality instead.
var patternSample = []int{1, 4, 8}

// stepPatternTeeth models step 15: the tooth body patterned around the Gear
// Center axis. Each sampled slot is the seed placed by a rotation of
// k * 360/N degrees about the axis, which is what quantity N and totalAngle
// 360 deg mean; the returned bodies are the seed plus the copies — the
// pattern's own bodies collection includes the original
// ([PB-PATTERN-BODIES]).
func stepPatternTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := chordedToothProfile(t, s, g)
	seed, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude seed tooth: %v", err)
	}
	bodies := []*decad.Body{seed}
	for _, k := range patternSample {
		rot, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(float64(k)*2*math.Pi/g.toothNumber))
		if err != nil {
			t.Fatalf("rotation: %v", err)
		}
		c, err := seed.PlacedCopy(rot)
		if err != nil {
			t.Fatalf("place copy %d: %v", k, err)
		}
		bodies = append(bodies, c)
	}
	return bodies
}

// assertPatternTeeth checks each copy is the seed rotated into its slot:
// same volume, centroid at exactly the slot rotation of the seed centroid,
// axial position unchanged.
func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	seedVol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("seed volume: %v", err)
	}
	seedC, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("seed centroid: %v", err)
	}
	for i, k := range patternSample {
		copyBody := bodies[i+1]
		v, err := copyBody.Volume()
		if err != nil {
			t.Fatalf("copy volume: %v", err)
		}
		if relDiff(mm3(v.Value, t), mm3(seedVol.Value, t)) > 1e-9 {
			t.Errorf("copy %d volume differs from the seed", k)
		}
		c, err := copyBody.Centroid()
		if err != nil {
			t.Fatalf("copy centroid: %v", err)
		}
		a := float64(k) * 2 * math.Pi / g.toothNumber
		wx, wy := rotate(seedC.Value.X, seedC.Value.Y, a)
		if math.Abs(c.Value.X-wx) > 1e-6 || math.Abs(c.Value.Y-wy) > 1e-6 || math.Abs(c.Value.Z-seedC.Value.Z) > 1e-6 {
			t.Errorf("copy %d centroid %v, want the seed centroid rotated by %d/N turns", k, c.Value, k)
		}
	}
}

// combineCases: the whole-gear result at both sizes.
var combineCases = []proofkit3d.Case{
	{Name: "default m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
	{Name: "coarse m2 z13", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5}},
}

// stepCombineTeeth models step 16, the combine-join of the patterned teeth
// into the gear body, as the one solid it must produce: the whole-gear
// prism, built from the single outline the pattern tiles.
//
// Why the substitute: the real combine joins bodies whose tooth faces lie
// exactly ON the body's root cylinder, and decad's boolean refuses that
// tangent face-on-face contact outright (and, once one union has faceted
// the boundary, reports the next tooth's contact undecidable). No sequence
// of Union calls over the pattern survives. What the one-profile prism
// still pins: that N teeth at 360/N spacing plus the root disc close into
// exactly ONE watertight solid (the RunSolid gate checks one lump, no
// voids), and — through assertCombineTeeth's exact area bookkeeping — that
// the combined volume is the disc plus exactly N teeth, which is the
// quantity and total angle of step 15 carried to the full count.
func stepCombineTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := wholeGearOutline(t, s, g)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude whole gear: %v", err)
	}
	return []*decad.Body{body}
}

// assertCombineTeeth checks the tiling equality: whole gear = root disc +
// N teeth, exactly, in section area and so in volume.
func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	thickness := params[pThickness]
	s := newXYSketch(t)
	toothArea := chordedToothProfile(t, s, g).Area
	discArea := math.Pi * g.dims.Root * g.dims.Root
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	want := (discArea + g.toothNumber*toothArea) * thickness
	if got := mm3(vol.Value, t); relDiff(got, want) > 1e-9 {
		t.Errorf("whole gear volume %.6f, want disc + N teeth = %.6f", got, want)
	}
}

// filletCases: the root fillet at both sizes, radius from the spec formula.
var filletCases = []proofkit3d.Case{
	{Name: "default m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10}},
	{Name: "coarse m2 z13", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5}},
}

// stepRootFillets models step 17: on the combined gear, round every axial
// edge where a root valley meets a tooth flank — the concave lateral edges
// parallel to the gear axis, two per valley — at FilletRadius. The
// circular front and back rim edges are exactly what the axial-direction
// filter drops.
func stepRootFillets(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := wholeGearOutline(t, s, g)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude whole gear: %v", err)
	}
	filleted, err := body.Fillet(
		decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(0, 0, 1))),
		units.Millimeters(g.filletRadius()))
	if err != nil {
		t.Fatalf("root fillet: %v", err)
	}
	return []*decad.Body{filleted}
}

// assertRootFillets checks the selection found both corners of every valley
// and nothing else — 2N blend faces, every blend a cylinder at FilletRadius
// — and that rounding the inside corners ADDED material.
func assertRootFillets(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	blends := 0
	for _, f := range bodies[0].Faces() {
		isBlend := false
		for _, o := range f.Origins() {
			if len(o.Role) >= 7 && o.Role[:7] == "fillet(" {
				isBlend = true
			}
		}
		if !isBlend {
			continue
		}
		blends++
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok {
			t.Errorf("fillet blend face is %T, want a cylinder", f.Surface())
			continue
		}
		if math.Abs(mm(cyl.Radius, t)-g.filletRadius()) > 1e-9 {
			t.Errorf("blend radius %.6f, want FilletRadius %.6f", mm(cyl.Radius, t), g.filletRadius())
		}
	}
	if want := 2 * int(g.toothNumber); blends != want {
		t.Errorf("fillet rounded %d corners, want two per valley = %d", blends, want)
	}
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	s := newXYSketch(t)
	toothArea := chordedToothProfile(t, s, g).Area
	unfilleted := (math.Pi*g.dims.Root*g.dims.Root + g.toothNumber*toothArea) * params[pThickness]
	if got := mm3(vol.Value, t); got <= unfilleted {
		t.Errorf("filleting the concave root corners must add material: %.6f <= %.6f", got, unfilleted)
	}
}

// boreCutCases: bores through both gear sizes.
var boreCutCases = []proofkit3d.Case{
	{Name: "5mm bore m1 z17", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 10, pBoreDiameter: 5}},
	{Name: "8mm bore m2 z13", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15,
		pThickness: 5, pBoreDiameter: 8}},
}

// stepBoreCut models step 19: the bore profile cut through the gear body to
// the far end cap.
//
// One substitution: the real cut's ToEntityExtentDefinition ends exactly ON
// the far cap, and Fusion accepts the flush faces; decad's boolean refuses
// face-on-face contact, so the tool is extruded longer and shifted so it
// pierces both caps cleanly. What survives: the cut goes all the way
// through regardless of Thickness — asserted as the removed volume being
// the full-thickness cylinder.
func stepBoreCut(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := geomOf(t, params)
	s := newXYSketch(t)
	p := wholeGearOutline(t, s, g)
	gear, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(params[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude gear: %v", err)
	}
	bs := newXYSketch(t)
	bp := discProfile(t, bs, params[pBoreDiameter]/2)
	tool, err := doc.Extrude(bs, bp, decad.Distance{D: units.Millimeters(params[pThickness] + 4), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude bore tool: %v", err)
	}
	shift, err := r3.Translation(r3.Vec{Z: -2})
	if err != nil {
		t.Fatalf("translation: %v", err)
	}
	tool, err = tool.Placed(shift)
	if err != nil {
		t.Fatalf("place bore tool: %v", err)
	}
	bored, err := decad.Cut(gear, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

// assertBoreCut checks the bore removed the full-thickness cylinder and
// nothing else.
func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := geomOf(t, params)
	thickness := params[pThickness]
	r := params[pBoreDiameter] / 2
	s := newXYSketch(t)
	toothArea := chordedToothProfile(t, s, g).Area
	gearVol := (math.Pi*g.dims.Root*g.dims.Root + g.toothNumber*toothArea) * thickness
	want := gearVol - math.Pi*r*r*thickness
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	// The cut is a faceted boolean; its volume is proven only to the mesh
	// chord tolerance, so the check is loose where every other one is exact.
	if got := mm3(vol.Value, t); relDiff(got, want) > 1e-3 {
		t.Errorf("bored volume %.4f, want gear minus through-bore = %.4f", got, want)
	}
}
