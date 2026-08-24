package spurgear_test

// The solid steps of the spur build, proved in decad.
//
// Each function here is one Fusion timeline entry: an extrude, a pattern, a
// combine, a fillet, a cut. Every one of them consumes the chorded tooth
// described in geometry_test.go rather than the spline-walled loop the Gear
// Profile sketch really closes, and each step says below what that costs it.
//
// STEP 8, CHAMFER TOOTH, IS [PROSE] AND HAS NO FUNCTION HERE. It belongs beside
// stepExtrudeTooth, which builds the body it would chamfer, so the reason it is
// missing is recorded here rather than only in the step list.
//
// Two walls stand between the step and a proof, and the second one is the
// blocking one:
//
//   - The selection the spec asks for is every edge of the tooth's front face
//     EXCEPT the arc it shares with the root valley. decad chamfers a prism's
//     cap only as one or more COMPLETE cap loops; a partial loop is refused
//     (ErrUnsupported). The substitute is the complete loop, which chamfers the
//     root arc too — the one edge the spec skips, because chamfering it would
//     eat into the neighbouring tooth.
//   - That substitute builds, but decad's verification of the resulting
//     cap-blend body reports its VOLUME reading as beyond the relative
//     tolerance, at every module, thickness, chamfer distance and involute
//     sample count tried (m1 to m10, chamfer 0.02 mm to 2 mm, 3 to 15 samples).
//     proofkit3d.RunSolid admits only an area or centroid reading past
//     tolerance, so the gate rejects it, and passing a weaker gate to get it
//     through is not on offer.
//
// So no substitute survives the gate, and the step stays prose. What is
// therefore unproven: that the front face is the one with chamferWantEdges()
// edges coplanar with the sketch plane, that the root arc is the edge whose
// radius equals Root Circle Radius, and that an equal-distance chamfer of the
// remaining five edges fits. The first two are face and edge searches, which
// decad's selector cannot express against a Fusion face-and-edge model anyway;
// the third is the geometric claim a working gate would settle.

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// solidCases sweeps the sizes the solid steps have to hold across, and both
// sides of the embedded branch, which changes how many curves bound the tooth
// and therefore what the extrude consumes.
var solidCases = []proofkit3d.Case{
	{Name: "m1_t17_thk10", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10}},
	{Name: "m1_t17_thk2_thin", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 2}},
	{Name: "m0.5_t12_thk5_fine", Params: map[string]float64{
		"module": 0.5, "toothNumber": 12, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 5}},
	{Name: "m5_t24_thk20_coarse", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 20, "thickness": 20}},
	{Name: "m1_t45_thk10_embedded", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10}},
	{Name: "m2_t30_pa25_thk8_embedded", Params: map[string]float64{
		"module": 2, "toothNumber": 30, "pressureAngle": 25 * math.Pi / 180, "angle": 0,
		"involuteSteps": 15, "thickness": 8}},
	{Name: "m1_t17_thk10_s3_coarse_samples", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 3, "thickness": 10}},
}

var patternCases = []proofkit3d.Case{
	{Name: "m1_t17_thk10", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10}},
	{Name: "m5_t24_thk20_coarse", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 20, "thickness": 20}},
	{Name: "m1_t45_thk10_embedded", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10}},
	{Name: "m0.5_t12_thk5_fine", Params: map[string]float64{
		"module": 0.5, "toothNumber": 12, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 5}},
}

var boreCutCases = []proofkit3d.Case{
	{Name: "bore0_guard", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10, "boreDiameter": 0}},
	{Name: "bore3", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10, "boreDiameter": 3}},
	{Name: "bore12_near_root", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10, "boreDiameter": 12}},
	{Name: "bore40_coarse", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 20, "thickness": 20, "boreDiameter": 40}},
	{Name: "bore6_thin", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 2, "boreDiameter": 6}},
}

// filletCases adds the branch the spec calls out by name: an edge collection
// that comes back empty is skipped silently rather than reaching the feature.
var filletCases = []proofkit3d.Case{
	{Name: "m1_t17_thk10", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10}},
	{Name: "m5_t24_thk20_coarse", Params: map[string]float64{
		"module": 5, "toothNumber": 24, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 20, "thickness": 20}},
	// A chorded flank near the root crossing has segments shorter than the
	// fillet's own setback once the sampling is fine, and decad refuses to
	// merge two corner rewrites that overlap. The real flank there is one
	// smooth spline with no corner at all, so the coarse sampling removes an
	// artefact of the substitution rather than weakening the case.
	{Name: "m1_t45_thk10_embedded_coarse_samples", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 6, "thickness": 10}},
	{Name: "no_axial_root_edge", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": math.Pi / 9, "angle": 0,
		"involuteSteps": 15, "thickness": 10, "noRootEdges": 1}},
}

// stepExtrudeTooth is step 7: extrude the tooth cross-section from the target
// plane to the Extrusion End Plane as a new body.
//
// The Extrusion End Plane is an offset plane at Thickness, and a to-entity
// extent onto it sweeps exactly Thickness — which is what the distance extent
// here models, since decad has no construction planes to end on.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	s, profile := toothOutline(t, g, g.angle, g.dims.Root, false)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude tooth: %v", err)
	}
	return []*decad.Body{body}
}

func assertExtrudeTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	body := only(t, bodies)
	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("tooth bounds: %v", err)
	}
	// The extrude runs from the sketch plane to the end plane, so the body
	// spans exactly Thickness and starts on the sketch plane.
	if math.Abs(bounds.Min.Z) > 1e-9 || math.Abs(bounds.Max.Z-g.thickness) > 1e-9 {
		t.Errorf("tooth spans z %.9f..%.9f, want 0..%.9f", bounds.Min.Z, bounds.Max.Z, g.thickness)
	}
	// The tooth reaches the tip circle and its foot lands on the root circle: it
	// is the section outside the disc, and it meets the disc exactly there. Both
	// radii are pinned two-sided, because a foot merely outside the root circle
	// is a tooth floating clear of the disc, and step 10's Combine-Join would
	// leave a notch or a disjoint body instead of one gear. toothOutline closes
	// both flanks on footRadius and joins them with an origin-centred arc, so
	// the foot sits on the root radius by construction; measured across the
	// case table the worst relative error is around 1e-16, ten orders inside
	// the 1e-6 the tip already uses.
	near, far := radialExtent(t, body)
	if math.Abs(far-g.dims.Tip) > 1e-6*g.dims.Tip {
		t.Errorf("tooth reaches radius %.6f, want the tip circle's %.6f", far, g.dims.Tip)
	}
	if math.Abs(near-g.dims.Root) > 1e-6*g.dims.Root {
		t.Errorf("tooth foot sits at radius %.6f, want the root circle's %.6f", near, g.dims.Root)
	}
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("tooth volume: %v", err)
	}
	if volume.Value.Base() <= 0 {
		t.Errorf("tooth volume %v is not positive", volume.Value)
	}
}

// stepExtrudeBody is step 9: extrude the disc inside the root circle from the
// target plane to the Extrusion End Plane as a new body, the one named Gear
// Body.
//
// The spec finds this region by matching a boundary of exactly two arcs — the
// two pieces the tooth cuts the root circle into. That count is asserted on the
// real Gear Profile sketch by stepGearProfileSketch; here the region is drawn
// as the whole root circle, since decad refuses to record a trim it cannot
// certify and the sketch engine withholds certification from every edge of a
// sketch that holds a spline. The two describe the same disc: same boundary
// curve, same area, same body.
func stepExtrudeBody(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	s, profile := circleProfile(t, g.dims.Root)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude body: %v", err)
	}
	return []*decad.Body{body}
}

func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	body := only(t, bodies)
	// It is the disc, not an annulus: the tip circle is construction geometry
	// and bounds no profile, so the body's volume is the full root-circle disc.
	want := g.discArea() * g.thickness
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("body volume: %v", err)
	}
	if got := volume.Value.Base(); math.Abs(got-want) > 1e-6*want {
		t.Errorf("gear body volume %.6f, want the solid disc's %.6f", got, want)
	}
	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("body bounds: %v", err)
	}
	if math.Abs(bounds.Min.Z) > 1e-9 || math.Abs(bounds.Max.Z-g.thickness) > 1e-9 {
		t.Errorf("gear body spans z %.9f..%.9f, want 0..%.9f", bounds.Min.Z, bounds.Max.Z, g.thickness)
	}
	// The far end cap the bore cut later ends on is the one plane face parallel
	// to, and not coplanar with, the sketch plane.
	if got := endCaps(body); got != 2 {
		t.Errorf("gear body has %d planar end cap(s), want 2 (the near one and the bore's target)", got)
	}
	// The cylindrical face the Gear Center axis is built from.
	cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
	if err != nil || len(cylinders) == 0 {
		t.Errorf("gear body offers no cylindrical face for the Gear Center axis (err=%v)", err)
	}
}

// patternPlaces are the pattern positions this step actually builds: the seed,
// its immediate successor, and the one place that comes back round to the seed
// from the other side. The last is what pins the full turn — place Tooth Number
// minus one sits exactly one step short of the seed only if the turn was
// divided into Tooth Number places and the pattern was not symmetric about the
// seed.
//
// The whole ring is not built. decad decides interference pairwise, and beyond
// about five chorded teeth in one document a pair comes back undecidable ("both operands
// tessellate, but later read-only intersection geometry exceeds the boolean
// pipeline's reach"), which the solid gate rejects — correctly, since an
// undecided pair is not a proven-disjoint one, and the reach shrinks as the
// teeth spread further apart round the ring. So this step proves the spacing
// rule, the quantity arithmetic and that neighbouring teeth stand clear of one
// another, and leaves the remaining places, which are the same operation at the
// same spacing, unbuilt.
func patternPlaces(toothNumber int) []int {
	places := []int{0, 1, toothNumber - 1}
	seen := map[int]bool{}
	out := []int{}
	for _, p := range places {
		if p < 0 || p >= toothNumber || seen[p] {
			continue
		}
		seen[p] = true
		out = append(out, p)
	}
	return out
}

// stepPatternTeeth is step 10's first half: circular-pattern the tooth body
// around the Gear Center axis, quantity Tooth Number, over a full turn and not
// symmetric.
//
// The pattern's bodies collection already holds the seed plus the copies, which
// is why the seed is in the returned set rather than added again.
func stepPatternTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	s, profile := toothOutline(t, g, g.angle, g.dims.Root, false)
	seed, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude seed tooth: %v", err)
	}
	bodies := []*decad.Body{seed}
	for _, place := range patternPlaces(int(g.toothNumber))[1:] {
		turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1),
			units.Radians(2*math.Pi*float64(place)/g.toothNumber))
		if err != nil {
			t.Fatalf("pattern rotation %d: %v", place, err)
		}
		copyOf, err := seed.PlacedCopy(turn)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", place, err)
		}
		bodies = append(bodies, copyOf)
	}
	return bodies
}

func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	places := patternPlaces(int(g.toothNumber))
	if len(bodies) != len(places) {
		t.Fatalf("pattern left %d bodies, want the %d places built", len(bodies), len(places))
	}
	seedVolume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("seed volume: %v", err)
	}
	step := 2 * math.Pi / g.toothNumber
	for n, body := range bodies {
		i := places[n]
		volume, err := body.Volume()
		if err != nil {
			t.Fatalf("tooth %d volume: %v", i, err)
		}
		if math.Abs(volume.Value.Base()-seedVolume.Value.Base()) > 1e-6*seedVolume.Value.Base() {
			t.Errorf("tooth %d volume %v differs from the seed's %v", i, volume.Value, seedVolume.Value)
		}
		centroid, err := body.Centroid()
		if err != nil {
			t.Fatalf("tooth %d centroid: %v", i, err)
		}
		// A full turn split into Tooth Number places, not symmetric about the
		// seed: tooth i sits exactly i steps round from it.
		got := math.Atan2(centroid.Value.Y, centroid.Value.X)
		if diff := wrapPi(got - float64(i)*step); math.Abs(diff) > 1e-6 {
			t.Errorf("tooth %d sits at %.6f rad, want %.6f", i, got, float64(i)*step)
		}
		if radius := math.Hypot(centroid.Value.X, centroid.Value.Y); radius < g.dims.Root {
			t.Errorf("tooth %d centroid sits at radius %.6f, inside the root circle", i, radius)
		}
	}
}

// stepCombineTeeth is step 10's second half: one Combine-Join that merges the
// patterned tooth bodies into Gear Body.
//
// Two substitutions, and the second one bounds what this step proves.
//
// The tooth is drawn SUNK — its flanks close on 0.97 of the root radius rather
// than on it — so the join is a crossing rather than a contact. decad's boolean
// refuses a pair whose facets meet in one plane without crossing, which is
// exactly what a tooth that only touches the root circle presents. Sinking
// costs nothing that is measured here: the material below the root circle is
// already inside the disc, so the join adds precisely the volume of the tooth
// that stands outside it, which is the unsunk tooth's own volume, and that is
// what the assertion checks.
//
// Only ONE tooth is joined, not all Tooth Number of them. A second join takes
// the first join's result as an operand, and decad refuses a boolean on a
// faceted result whose cap facets are coplanar with the tool's — both bodies
// run from the sketch plane to the end plane, so they always are. What that
// leaves unproven is that the remaining teeth join without interfering with one
// another; what it does prove is that the join itself is exact and leaves one
// sound lump.
func stepCombineTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	discSketch, discProfile := circleProfile(t, g.dims.Root)
	disc, err := doc.Extrude(discSketch, discProfile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude gear body: %v", err)
	}
	toothSketch, toothProfile := toothOutline(t, g, g.angle, g.dims.Root*0.97, false)
	tooth, err := doc.Extrude(toothSketch, toothProfile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude patterned tooth: %v", err)
	}
	joined, err := decad.Union(disc, tooth)
	if err != nil {
		t.Fatalf("combine join: %v", err)
	}
	return []*decad.Body{joined}
}

func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	joined := only(t, bodies)
	// The join adds the tooth standing outside the root circle and nothing
	// else, so the result is the disc plus one unsunk tooth exactly.
	reference := decad.New()
	toothSketch, toothProfile := toothOutline(t, g, g.angle, g.dims.Root, false)
	tooth, err := reference.Extrude(toothSketch, toothProfile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("reference tooth: %v", err)
	}
	toothVolume, err := tooth.Volume()
	if err != nil {
		t.Fatalf("reference tooth volume: %v", err)
	}
	want := g.discArea()*g.thickness + toothVolume.Value.Base()
	volume, err := joined.Volume()
	if err != nil {
		t.Fatalf("joined volume: %v", err)
	}
	if got := volume.Value.Base(); math.Abs(got-want) > 1e-6*want {
		t.Errorf("joined volume %.6f, want disc plus one tooth = %.6f", got, want)
	}
	if got := len(joined.Lumps()); got != 1 {
		t.Errorf("the join left %d lumps, want one joined body", got)
	}
	_, far := radialExtent(t, joined)
	if math.Abs(far-g.dims.Tip) > 1e-6*g.dims.Tip {
		t.Errorf("joined body reaches radius %.6f, want the tip circle's %.6f", far, g.dims.Tip)
	}
}

// stepRootFillets is step 11: round the corner where the root valley floor
// meets each tooth flank, the inside corner that runs the full thickness
// parallel to the gear's main axis.
//
// The body is the disc and one tooth as a single region, which is what the
// combine leaves behind and the only shape that HAS those corners — a tooth on
// its own has none, and the combine's own result is a boolean body decad will
// not fillet. The corners are selected the way the spec selects them, by taking
// the edges parallel to the target plane's normal, and Concave() is what
// separates the two valley corners from the tooth's own convex ones.
func stepRootFillets(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	if params["noRootEdges"] == 1 {
		// The spec's silently-skipped branch: no axial root edge matched, so
		// the feature is never created and the body is returned untouched. A
		// plain disc is the shape that has no such edge.
		s, profile := circleProfile(t, g.dims.Root)
		body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
		if err != nil {
			t.Fatalf("extrude disc: %v", err)
		}
		axial := decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(0, 0, 1)))
		if _, err := axial.SelectEdges(body); !errors.Is(err, decad.ErrNoMatch) {
			t.Fatalf("a bare disc offered axial root edges (err=%v); the empty-collection branch is unreachable", err)
		}
		return []*decad.Body{body}
	}
	s, profile := toothOutline(t, g, g.angle, g.dims.Root, true)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude disc and tooth: %v", err)
	}
	axial := decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(2)
	rounded, err := body.Fillet(axial, units.Millimeters(g.filletRadius()))
	if err != nil {
		t.Fatalf("root fillet at radius %.6f: %v", g.filletRadius(), err)
	}
	return []*decad.Body{rounded}
}

func assertRootFillets(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	body := only(t, bodies)
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("filleted volume: %v", err)
	}
	unrounded := g.discArea() * g.thickness
	if params["noRootEdges"] == 1 {
		// Skipped silently: the body is the one the previous step left.
		if got := volume.Value.Base(); math.Abs(got-unrounded) > 1e-6*unrounded {
			t.Errorf("the skipped fillet changed the body: volume %.6f, want %.6f", got, unrounded)
		}
		return
	}
	// A fillet at a concave corner fills material in, so the body grows, and it
	// grows by less than the corner's bounding wedge.
	reference := decad.New()
	s, profile := toothOutline(t, g, g.angle, g.dims.Root, true)
	plain, err := reference.Extrude(s, profile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("reference disc and tooth: %v", err)
	}
	before, err := plain.Volume()
	if err != nil {
		t.Fatalf("reference volume: %v", err)
	}
	added := volume.Value.Base() - before.Value.Base()
	if added <= 0 {
		t.Errorf("the root fillet removed %.9f mm3; a concave corner round adds material", -added)
	}
	r := g.filletRadius()
	if ceiling := 2 * r * r * g.thickness; added > ceiling {
		t.Errorf("the root fillet added %.9f mm3, more than the two corners can hold (%.9f)", added, ceiling)
	}
	// The rounding is local to the valley: it moves neither cap nor the tip.
	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("filleted bounds: %v", err)
	}
	if math.Abs(bounds.Min.Z) > 1e-9 || math.Abs(bounds.Max.Z-g.thickness) > 1e-9 {
		t.Errorf("the fillet moved the end caps: z %.9f..%.9f, want 0..%.9f", bounds.Min.Z, bounds.Max.Z, g.thickness)
	}
}

// stepBoreCut is step 12's second half: extrude-cut the bore profile from the
// target plane to the far end-cap face, affecting only the gear body.
//
// buildBore runs unconditionally, so the step has to return early twice — in
// sketch-only mode, and when the bore diameter is not positive. The
// non-positive case is a live case here: the step runs, cuts nothing, and hands
// the gear body back unchanged. Sketch-only cannot be reached at all, because
// buildMainGearBody short-circuits before the gear body exists and there is
// then no body for this proof to build.
func stepBoreCut(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := newGear(params)
	discSketch, discProfile := circleProfile(t, g.dims.Root)
	gearBody, err := doc.Extrude(discSketch, discProfile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude gear body: %v", err)
	}
	if g.bore <= 0 {
		return []*decad.Body{gearBody}
	}
	boreSketch, boreProfile := circleProfile(t, g.bore/2)
	// The to-entity extent onto the far end cap is what guarantees the bore
	// goes all the way through whatever the Thickness is; a tool that spans the
	// same range is the same cut.
	tool, err := doc.Extrude(boreSketch, boreProfile, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude bore tool: %v", err)
	}
	bored, err := decad.Cut(gearBody, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := newGear(params)
	body := only(t, bodies)
	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("bored volume: %v", err)
	}
	want := g.discArea() * g.thickness
	if g.bore > 0 {
		want -= math.Pi * g.bore * g.bore / 4 * g.thickness
	}
	if got := volume.Value.Base(); math.Abs(got-want) > 1e-6*want {
		t.Errorf("bored volume %.6f, want %.6f", got, want)
	}
	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("bored bounds: %v", err)
	}
	// The cut runs right through: the body still spans the full thickness and
	// the hole leaves no closed void behind it.
	if math.Abs(bounds.Min.Z) > 1e-9 || math.Abs(bounds.Max.Z-g.thickness) > 1e-9 {
		t.Errorf("bored body spans z %.9f..%.9f, want 0..%.9f", bounds.Min.Z, bounds.Max.Z, g.thickness)
	}
	if g.bore > 0 {
		near, _ := radialExtent(t, body)
		if math.Abs(near-g.bore/2) > 1e-6*g.bore {
			t.Errorf("the bore's wall sits at radius %.6f, want %.6f", near, g.bore/2)
		}
	}
}

// only returns the single body a step left behind.
func only(t *testing.T, bodies []*decad.Body) *decad.Body {
	t.Helper()
	if len(bodies) != 1 {
		t.Fatalf("step left %d bodies, want 1", len(bodies))
	}
	return bodies[0]
}

// radialExtent returns the nearest and farthest distance from the gear axis
// reached by the body's vertices.
func radialExtent(t *testing.T, body *decad.Body) (near, far float64) {
	t.Helper()
	near, far = math.Inf(1), 0
	for _, v := range body.Vertices() {
		p := v.Position()
		r := math.Hypot(p.Value.X, p.Value.Y)
		near = math.Min(near, r)
		far = math.Max(far, r)
	}
	if math.IsInf(near, 1) {
		t.Fatal("body reports no vertices")
	}
	return near, far
}

// endCaps counts the planar faces whose normal runs along the extrude
// direction — the near cap the sketch sits on and the far one the bore ends on.
func endCaps(body *decad.Body) int {
	found, err := decad.Faces(decad.Planar(), decad.NormalTo(r3.NewVec(0, 0, 1))).SelectFaces(body)
	if err != nil {
		return 0
	}
	return len(found)
}
