// This file holds the spur gear's solid steps, one per Fusion timeline entry:
// the tooth extrude, the body extrude, the circular pattern, the Combine-Join,
// the root fillets, the bore cut and the end-cap chamfer.
//
// Each step builds its own receiver from the same tooth math the sketch step
// uses, rather than continuing the previous step's body, so a failure names one
// step instead of the longest chain that still runs.
//
// The sections here are drawn at their solved coordinates and carry no
// constraints. The constraint scheme is stepGearProfileSketch's subject;
// restating it here would prove it twice and build nothing new.
//
// Several of these steps substitute geometry decad accepts for geometry the
// spec names, and every substitution is written next to the builder that makes
// it, with what it costs: the tooth's root boundary drawn as its own arc rather
// than derived by splitting the root circle (toothSection), chorded flanks in
// place of fitted splines wherever a boolean is involved (drawFlank), the
// one-piece gear prism as the fillet, bore and chamfer receiver in place of the
// patterned-and-combined body (gearSection), the patterned copies measured one
// document at a time in place of all of them at once (stepPatternTeeth), and
// the single Combine-Join in place of a chain of Tooth Number of them
// (stepCombineTeeth).

package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

var solidCases = []proofkit3d.Case{
	{Name: "M1_N12_T10_no_bore", Params: solidParams(1, 12, 20, 5, 10, 0, 0)},
	{Name: "M1_N12_T10_bore4", Params: solidParams(1, 12, 20, 5, 10, 4, 0.2)},
	{Name: "M2_N14_T3_bore8", Params: solidParams(2, 14, 20, 4, 3, 8, 0.5)},
	{Name: "M1_N17_T25_bore2", Params: solidParams(1, 17, 20, 4, 25, 2, 0.1)},
}

// solidParams names one solid case by the dialog values it comes from. Bore
// Diameter 0 is the shipped default and means no bore; Apply-chamfer-to-teeth 0
// means no chamfer.
func solidParams(module, toothNumber, pressureAngleDeg float64, steps int, thickness, bore, chamfer float64) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": rad(pressureAngleDeg),
		"angle":         0,
		"involuteSteps": float64(steps),
		"thickness":     thickness,
		"boreDiameter":  bore,
		"chamferTooth":  chamfer,
	}
}

func mm(v float64) units.Value { return units.Millimeters(v) }

// newSketch is a fresh plane to draw one section on. The sections below are
// drawn at their solved coordinates and carry no constraints: the constraint
// scheme is what stepGearProfileSketch proves, and repeating it here would
// prove it twice and build nothing new.
func newSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

// onlyRegion is the single closed region a section sketch holds.
func onlyRegion(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("section holds %d regions, want exactly 1", len(regions))
	}
	if !regions[0].Valid {
		t.Fatal("section region is not extrudable")
	}
	return regions[0]
}

// footPoint is a flank-to-root stub's far end: the flank start's direction from
// the gear centre, taken out to radius r.
func footPoint(s *sketch.Sketch, r float64, start involute.Pt) *sketch.Point {
	n := math.Hypot(start.X, start.Y)
	return s.CreatePoint(r*start.X/n, r*start.Y/n)
}

// drawFlank draws one involute flank through its sample points.
//
// A flank is a SketchFittedSpline in Fusion and a fit spline here. chorded
// replaces it with the polyline through the same sample points, which is the
// substitution the Combine-Join needs: decad's analytic prism boolean admits a
// section of lines, circles and arcs only, because one free-form entity
// anywhere in an arrangement withdraws exact bounds from every edge in it. A
// pair outside that class falls to the mesh boolean, and the mesh boolean
// refuses two prisms swept from the same plane, which a tooth and the gear body
// always are. What chording costs is the sagitta between each chord and the
// involute it spans: the flank moves inward by that much, so a chorded tooth is
// slightly thinner and every volume read off it slightly smaller. The step that
// chords measures chorded against chorded, so the sagitta cancels.
func drawFlank(t *testing.T, s *sketch.Sketch, pts []*sketch.Point, chorded bool, label string) {
	t.Helper()
	if !chorded {
		if _, err := s.CreateFitSpline(pts...); err != nil {
			t.Fatalf("%s flank spline: %v", label, err)
		}
		return
	}
	for i := 0; i+1 < len(pts); i++ {
		s.CreateLine(pts[i], pts[i+1])
	}
}

// toothSection draws one tooth outline and returns the region the tooth extrude
// consumes.
//
// The root boundary is drawn as its own arc between the two stub feet. In
// Fusion that arc is a piece of the solid root circle, which the tooth splits;
// it is authored here because a fragment of a split curve is a range this
// harness will not certify once a free-form entity is in the sketch
// (sketch.BoundaryEdge.TExact is a whole-scene gate), and decad refuses to
// record an uncertified fragment. It is the same circle at the same radius, and
// the curve COUNTS that split produces are asserted on the real Gear Profile
// sketch in stepGearProfileSketch instead.
//
// sink pulls that root boundary inward by that many millimetres, and turn puts
// the whole tooth at a pattern angle. Both are 0 for the tooth the extrude step
// builds.
func toothSection(t *testing.T, p map[string]float64, sink, turn float64, chorded bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["toothNumber"],
		int(p["involuteSteps"]), p["angle"]+turn)

	s := newSketch(t)
	centre := s.CreatePoint(0, 0)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	drawFlank(t, s, leftPts, chorded, "left")
	drawFlank(t, s, rightPts, chorded, "right")
	s.CreateArc(centre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])

	leftFoot := footPoint(s, d.Root-sink, left[0])
	rightFoot := footPoint(s, d.Root-sink, right[0])
	s.CreateLine(leftFoot, leftPts[0])
	s.CreateLine(rightFoot, rightPts[0])
	s.CreateArc(centre, rightFoot, leftFoot)
	return s, onlyRegion(t, s)
}

// discSection draws the solid root circle on its own — the region the gear-body
// extrude consumes.
func discSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	s := newSketch(t)
	s.CreateCircle(s.CreatePoint(0, 0), d.Root)
	return s, onlyRegion(t, s)
}

// boreSection draws the bore circle, centred on the anchor at the gear centre.
func boreSection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newSketch(t)
	s.CreateCircle(s.CreatePoint(0, 0), p["boreDiameter"]/2)
	return s, onlyRegion(t, s)
}

// gearSection draws the whole gear outline — every tooth and the root-circle
// valley between neighbours — as one closed loop.
//
// It is the receiver the fillet, bore and chamfer steps build on. Those three
// features take a straight prism, and the patterned-and-combined gear is a
// boolean result, which decad does not accept as a fillet or chamfer receiver.
// Extruding this outline in one go gives a prism of exactly the gear's shape, so
// the edges those steps select — the concave axial root corners, the end-cap
// loop — are the real ones. What the substitution costs is the boolean history:
// those steps prove the selection and the feature on the gear's geometry, not
// that Fusion's combine leaves that geometry behind. stepPatternTeeth is where
// the join itself is proved.
func gearSection(t *testing.T, p map[string]float64, chorded bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	teeth := int(p["toothNumber"])
	s := newSketch(t)
	centre := s.CreatePoint(0, 0)

	feet := make([][2]*sketch.Point, teeth)
	for k := range teeth {
		turn := 2 * math.Pi * float64(k) / float64(teeth)
		left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["toothNumber"],
			int(p["involuteSteps"]), p["angle"]+turn)
		leftPts := make([]*sketch.Point, len(left))
		rightPts := make([]*sketch.Point, len(right))
		for i := range left {
			leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
			rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
		}
		drawFlank(t, s, leftPts, chorded, "left")
		drawFlank(t, s, rightPts, chorded, "right")
		s.CreateArc(centre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
		leftFoot := footPoint(s, d.Root, left[0])
		rightFoot := footPoint(s, d.Root, right[0])
		s.CreateLine(leftFoot, leftPts[0])
		s.CreateLine(rightFoot, rightPts[0])
		feet[k] = [2]*sketch.Point{leftFoot, rightFoot}
	}
	for k := range teeth {
		s.CreateArc(centre, feet[k][0], feet[(k+1)%teeth][1])
	}
	return s, onlyRegion(t, s)
}

// extrudeSection sweeps one section by Thickness, the distance between the
// target plane and the Extrusion End Plane.
func extrudeSection(t *testing.T, doc *decad.Document, p map[string]float64,
	s *sketch.Sketch, region *sketch.Profile) *decad.Body {
	t.Helper()
	body, err := doc.Extrude(s, region, decad.Distance{D: mm(p["thickness"]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude section: %v", err)
	}
	return body
}

// gearPrism extrudes the one-piece gear section by Thickness. Its flanks are
// always chorded: Tooth Number pairs of fit splines in one section exceed
// decad's fixed free-form work budget for exact integration.
func gearPrism(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	t.Helper()
	s, region := gearSection(t, p, true)
	return extrudeSection(t, doc, p, s, region)
}

func volumeOf(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return measured.Value.Base()
}

// axis is the gear's main axis: the target plane's normal, which every extrude
// here sweeps along.
func axis() r3.Vec { return r3.NewVec(0, 0, 1) }

// ---------------------------------------------------------------- extrude tooth

func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, region := toothSection(t, p, 0, 0, false)
	return []*decad.Body{extrudeSection(t, doc, p, s, region)}
}

// assertExtrudeTooth pins what the tooth extrude is supposed to produce: one
// new body, swept from the target plane to the Extrusion End Plane, so exactly
// Thickness tall, and reaching the tip circle at its top.
func assertExtrudeTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("tooth extrude produced %d bodies, want the one new body", len(bodies))
	}
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("tooth bounds: %v", err)
	}
	if got, want := box.Max.Z-box.Min.Z, p["thickness"]; math.Abs(got-want) > 1e-9 {
		t.Errorf("tooth is %.9f mm tall, want Thickness %.9f mm", got, want)
	}
	if got := box.Max.X; math.Abs(got-d.Tip) > 1e-9 {
		t.Errorf("tooth reaches %.9f mm along +X, want the tip radius %.9f mm", got, d.Tip)
	}
}

// ---------------------------------------------------------------- extrude body

func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, region := discSection(t, p)
	return []*decad.Body{extrudeSection(t, doc, p, s, region)}
}

// assertExtrudeBody pins the gear body as the disc inside the root circle and
// not an annulus: its volume is the whole root disc's, which a ring bounded by
// the tip circle could not be.
func assertExtrudeBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("body extrude produced %d bodies, want the one Gear Body", len(bodies))
	}
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	want := math.Pi * d.Root * d.Root * p["thickness"]
	if got := volumeOf(t, bodies[0], "Gear Body"); math.Abs(got-want) > 1e-6*want {
		t.Errorf("Gear Body volume %.6f mm3, want the root disc's %.6f mm3", got, want)
	}
}

// ---------------------------------------------------------------- pattern teeth

// stepPatternTeeth builds the circular pattern: Tooth Number copies of
// ctx.toothBody about the Gear Center axis, over a total angle of 360 degrees,
// not symmetric — so copy k sits at 2*pi*k/N.
//
// The step returns the seed tooth, which is copy 0 and is one of the bodies the
// pattern hands back ([PB-PATTERN-BODIES] — the seed is in the collection, so
// the combine must not add it again). The other copies are measured in
// assertPatternTeeth, each in a document of its own, and that split is forced:
// decad verifies every PAIR of live bodies in a document, and for the tooth
// pairs of a real gear its disjoint/overlap partition proof resolves neither
// way — reported as an undecided or unsupported pair, which the solid gate
// refuses and this proof does not waive. Holding all Tooth Number copies at
// once is therefore out of reach here; what is lost is the proof that the
// copies are mutually disjoint, and the whole-gear volume identity in
// assertCombineTeeth is what covers that instead, since overlapping copies
// could not add up to it.
func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	ts, tregion := toothSection(t, p, 0, 0, false)
	return []*decad.Body{extrudeSection(t, doc, p, ts, tregion)}
}

// assertPatternTeeth pins the three inputs the spec makes the step set
// explicitly. Quantity is how many copies are placed. The total angle and the
// not-symmetric flag together are the placement: copy k's centroid sits at
// 2*pi*k/N around the gear centre, measured one way from the seed, which a
// symmetric pattern or a different total angle would not produce. Every copy
// has the seed tooth's own volume, so none is deformed by its placement.
//
// The solved section is reused for every copy in this case. Each copy is
// extruded in a fresh document and made with Body.Placed, so no document holds
// two teeth at once and no pair reaches the verifier.
func assertPatternTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("pattern step returned %d bodies, want the seed tooth", len(bodies))
	}
	teeth := int(p["toothNumber"])
	seedVolume := volumeOf(t, bodies[0], "seed tooth")
	if got := centroidAngle(t, bodies[0]); angleGap(got, 0) > 1e-9 {
		t.Errorf("the seed tooth sits at %.9f rad, want 0 — the pattern is not symmetric", got)
	}
	var ts *sketch.Sketch
	var tregion *sketch.Profile
	if teeth > 1 {
		ts, tregion = toothSection(t, p, 0, 0, false)
	}
	for k := 1; k < teeth; k++ {
		scratch := decad.New()
		copied, err := extrudeSection(t, scratch, p, ts, tregion).
			Placed(patternTurn(t, 2*math.Pi*float64(k)/float64(teeth)))
		if err != nil {
			t.Fatalf("pattern copy %d: %v", k, err)
		}
		if got := volumeOf(t, copied, "patterned tooth"); math.Abs(got-seedVolume) > 1e-9*seedVolume {
			t.Errorf("patterned tooth %d has volume %.9f mm3, want the seed's %.9f mm3",
				k, got, seedVolume)
		}
		if got, want := centroidAngle(t, copied), 2*math.Pi*float64(k)/float64(teeth); angleGap(got, want) > 1e-9 {
			t.Errorf("patterned tooth %d sits at %.9f rad, want %.9f rad", k, got, want)
		}
	}
}

func TestPatternSectionReusableAcrossDocuments(t *testing.T) {
	p := solidCases[0].Params
	s, region := toothSection(t, p, 0, 0, false)
	revision := s.Revision()
	stale := region.IsStale()
	if stale {
		t.Fatal("newly solved tooth section profile is stale")
	}

	first, err := extrudeSection(t, decad.New(), p, s, region).
		Placed(patternTurn(t, math.Pi/3))
	if err != nil {
		t.Fatalf("first reused pattern copy: %v", err)
	}
	second, err := extrudeSection(t, decad.New(), p, s, region).
		Placed(patternTurn(t, 2*math.Pi/3))
	if err != nil {
		t.Fatalf("second reused pattern copy: %v", err)
	}

	firstVolume := volumeOf(t, first, "first reused pattern copy")
	secondVolume := volumeOf(t, second, "second reused pattern copy")
	if math.Abs(firstVolume-secondVolume) > 1e-9*firstVolume {
		t.Errorf("reused pattern copies have volumes %.9f and %.9f", firstVolume, secondVolume)
	}
	for _, copy := range []struct {
		name string
		body *decad.Body
		want float64
	}{
		{name: "first", body: first, want: math.Pi / 3},
		{name: "second", body: second, want: 2 * math.Pi / 3},
	} {
		if got := centroidAngle(t, copy.body); angleGap(got, copy.want) > 1e-9 {
			t.Errorf("%s reused pattern copy sits at %.9f rad, want %.9f rad", copy.name, got, copy.want)
		}
	}
	if got := s.Revision(); got != revision {
		t.Errorf("reusing the section changed sketch revision from %d to %d", revision, got)
	}
	if got := region.IsStale(); got != stale {
		t.Errorf("reusing the section changed profile stale status from %t to %t", stale, got)
	}
}

// centroidAngle is where a tooth sits around the gear axis: the polar angle of
// its centroid in the sketch plane.
func centroidAngle(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("tooth centroid: %v", err)
	}
	return math.Atan2(centroid.Value.Y, centroid.Value.X)
}

// angleGap is the distance between two angles, taking the turn into account.
func angleGap(a, b float64) float64 {
	gap := math.Mod(math.Abs(a-b), 2*math.Pi)
	return math.Min(gap, 2*math.Pi-gap)
}

// patternTurn is one circular-pattern step about the Gear Center axis, built
// from a literal basis rather than from an axis and an angle.
//
// r3.RotationAround evaluates Rodrigues' formula, whose z row comes out a few
// float ulps off (0.99999999999999989 at 17 teeth) for most tooth counts, and
// decad's analytic prism reduction admits a pair only when the two composed
// world planes are the SAME plane by exact float equality. A basis whose z
// components are written as literal zeros keeps a rotated section exactly on
// the sketch plane, so a placed copy stays in the admitted class.
func patternTurn(t *testing.T, angle float64) r3.Transform {
	t.Helper()
	sin, cos := math.Sin(angle), math.Cos(angle)
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(cos, sin, 0),
		EY: r3.NewVec(-sin, cos, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 0, 0))
	if err != nil {
		t.Fatalf("pattern rotation of %.6f rad: %v", angle, err)
	}
	return turn
}

// ---------------------------------------------------------------- combine

// combineSink is how far the tooth is pushed into the gear body before the
// join. It is a fraction of Module, so it scales with the gear and stays far
// below the tooth's own dedendum. A tooth whose root arc sits exactly on the
// gear body's rim meets it face-on-face, and decad refuses a boolean whose
// operands touch without provably crossing.
func combineSink(p map[string]float64) float64 { return p["module"] / 50 }

// stepCombineTeeth joins the patterned teeth into the Gear Body.
//
// Only ONE join is built. decad admits a single analytic union of two prisms,
// but the result carries a section displacement that reroutes the NEXT union on
// the same lineage back to the mesh path, which refuses two prisms swept from
// one plane. So this step proves that a patterned tooth and the Gear Body
// combine into a single solid lump; that all Tooth Number of them together
// leave a gear of the right size, assertCombineTeeth proves by volume, which
// needs no boolean at all.
func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, region := discSection(t, p)
	gear := extrudeSection(t, doc, p, s, region)
	ts, tregion := toothSection(t, p, combineSink(p), 0, true)
	toothBody := extrudeSection(t, doc, p, ts, tregion)
	joined, err := decad.Union(gear, toothBody)
	if err != nil {
		t.Fatalf("combine tooth into Gear Body: %v", err)
	}
	return []*decad.Body{joined}
}

// assertCombineTeeth checks the join twice over.
//
// The join itself: the united body is the disc plus the part of the tooth
// outside it, which is the tooth as it would be drawn with no sink at all. A
// join that swallowed the tooth, or left it as a second body, misses that.
//
// The whole gear: extruded from one outline, it has the volume of the disc plus
// Tooth Number teeth, which a join that dropped or doubled a tooth cannot
// satisfy. Both sides are chorded, so the chord sagitta cancels; the whole
// gear's flanks have to be, because Tooth Number pairs of fit splines in one
// section exceed decad's fixed free-form work budget.
func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("combine left %d bodies, want the one Gear Body", len(bodies))
	}
	joined := volumeOf(t, bodies[0], "combined Gear Body")

	scratch := decad.New()
	ds, dregion := discSection(t, p)
	disc := volumeOf(t, extrudeSection(t, scratch, p, ds, dregion), "gear body disc")
	cs, cregion := toothSection(t, p, 0, 0, true)
	want := disc + volumeOf(t, extrudeSection(t, scratch, p, cs, cregion), "chorded tooth")
	if math.Abs(joined-want) > 1e-6*want {
		t.Errorf("combined Gear Body volume %.6f mm3, want disc + one tooth = %.6f mm3", joined, want)
	}

	whole := decad.New()
	gear := volumeOf(t, gearPrism(t, whole, p), "one-piece gear")
	ts, tregion := toothSection(t, p, 0, 0, true)
	tooth := volumeOf(t, extrudeSection(t, whole, p, ts, tregion), "tooth")
	expected := disc + p["toothNumber"]*tooth
	if math.Abs(gear-expected) > 1e-6*expected {
		t.Errorf("whole gear volume %.6f mm3, want disc + %.0f teeth = %.6f mm3",
			gear, p["toothNumber"], expected)
	}
}

// filletRadius is the derived Fillet Radius parameter, in millimetres:
// (Tooth Space Arc At Root / 2) * Fillet Clearance * 1, where the arc is Root
// Circle Radius * Tooth Space Angle At Root and the angle is
// pi / Tooth Number - 2 * (tan(Pressure Angle) - Pressure Angle). Fillet
// Clearance is 0.9; the trailing factor is 1 for a spur gear.
func filletRadius(p map[string]float64) float64 {
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	spaceAngle := math.Pi/p["toothNumber"] - 2*(math.Tan(p["pressureAngle"])-p["pressureAngle"])
	return (d.Root * spaceAngle / 2) * 0.9 * 1
}

// ---------------------------------------------------------------- root fillets

// stepFilletRoots rounds the corner where the root valley floor meets each
// tooth flank — the axial corner that runs the full thickness of the gear.
//
// The receiver is the one-piece gear prism, for the reason gearSection gives:
// decad takes a straight prism as a fillet receiver and not a boolean result.
// The selection is the spec's own, written in the predicates decad has: concave
// (the inside corner, not the convex tooth tip), parallel to the gear's main
// axis (which drops the circular end-cap rims the spec is explicit about not
// filleting), and exactly two per valley.
func stepFilletRoots(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	gear := gearPrism(t, doc, p)
	corners := decad.Edges(decad.Concave(), decad.ParallelTo(axis())).Exactly(int(2 * p["toothNumber"]))
	filleted, err := gear.Fillet(corners, mm(filletRadius(p)))
	if err != nil {
		t.Fatalf("fillet the root corners: %v", err)
	}
	return []*decad.Body{filleted}
}

// assertFilletRoots checks the selection the spec spends most of step 11 on.
//
// A root fillet sits in a concave corner, so it ADDS material: a filleted gear
// weighs more than the sharp one, and one that had rounded the convex tooth
// tips instead would weigh less. The end-cap rims are circular and so never
// parallel to the axis; asking for the axial edges among them returns nothing,
// which is the same discrimination the spec's dot-product test makes.
//
// The empty-edge-set case the spec requires a guard for is reachable and
// checked here: a selection that matches no edge is an error from decad, not an
// empty set that quietly reaches the feature.
func assertFilletRoots(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("fillet produced %d bodies, want 1", len(bodies))
	}
	sharp := gearPrism(t, decad.New(), p)
	before := volumeOf(t, sharp, "sharp gear")
	after := volumeOf(t, bodies[0], "filleted gear")
	if after <= before {
		t.Errorf("filleted gear volume %.6f mm3 is not above the sharp gear's %.6f mm3; a fillet in a concave corner adds material",
			after, before)
	}

	rims, err := decad.Edges(decad.Circular(), decad.ParallelTo(axis())).SelectEdges(sharp)
	if err == nil {
		t.Errorf("%d circular edge(s) read as parallel to the gear axis; the end-cap rims must not reach the fillet", len(rims))
	}
	if _, err := decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(sharp); err == nil {
		t.Error("a selection matching no root corner returned edges; the empty-collection guard would never be exercised")
	}
}

// ---------------------------------------------------------------- bore cut

// stepBoreCut cuts the optional bore through the gear body.
//
// The receiver is the Gear Body cylinder rather than the completed gear.
// decad's analytic prism cut charges 256 arrangement segments per arc and caps
// the scene at 1024, and a toothed gear section carries two arcs per tooth, so
// the cut on the completed gear is refused outright on capacity rather than
// falling back. The bore is entirely inside the root circle, so the material it
// removes is the same either way; what the substitution cannot show is that the
// cut's participant list holds the gear body alone.
//
// Bore Diameter 0 is the shipped default and means no bore at all: the step
// returns the gear body untouched, which is the early return buildBore makes,
// and the Bore Profile sketch of the preceding step is never drawn either.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	ds, dregion := discSection(t, p)
	gear := extrudeSection(t, doc, p, ds, dregion)
	if p["boreDiameter"] <= 0 {
		return []*decad.Body{gear}
	}
	bs, bregion := boreSection(t, p)
	tool := extrudeSection(t, doc, p, bs, bregion)
	bored, err := decad.Cut(gear, tool)
	if err != nil {
		t.Fatalf("cut the bore: %v", err)
	}
	return []*decad.Body{bored}
}

// assertBoreCut pins both sides of the Bore-Diameter branch: at 0 the gear body is
// returned whole, and above 0 exactly the bore cylinder's volume is gone and
// the hole runs the full Thickness, which is what extruding to the far end-cap
// face guarantees.
func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("bore step left %d bodies, want the one Gear Body", len(bodies))
	}
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	disc := math.Pi * d.Root * d.Root * p["thickness"]
	want := disc
	if p["boreDiameter"] > 0 {
		want = disc - math.Pi*p["boreDiameter"]*p["boreDiameter"]/4*p["thickness"]
	}
	if got := volumeOf(t, bodies[0], "bored Gear Body"); math.Abs(got-want) > 1e-6*want {
		t.Errorf("bored Gear Body volume %.6f mm3, want %.6f mm3", got, want)
	}
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("bored Gear Body bounds: %v", err)
	}
	if got, want := box.Max.Z-box.Min.Z, p["thickness"]; math.Abs(got-want) > 1e-6 {
		t.Errorf("bored Gear Body is %.9f mm tall, want Thickness %.9f mm", got, want)
	}
}

// ---------------------------------------------------------------- chamfer

// boreRimCutoff is the edge length that separates the bore's cap edge from the
// gear's own. The spec excludes an end-cap edge whose radius is the positive
// Bore Diameter over two; decad selects edges by length, so the same
// discrimination is written as a length above the bore rim's circumference and
// below the gear body's.
func boreRimCutoff(p map[string]float64) float64 { return 1.5 * math.Pi * p["boreDiameter"] }

// stepChamferTeeth chamfers the completed gear's end-cap loop.
//
// Two substitutions, both forced and both narrowing what is proved. The
// receiver is the Gear Body's own end cap rather than the toothed one: a
// cap-loop chamfer of the toothed section BUILDS, but decad then reports its
// volume reading beyond the default tolerance, and the solid gate refuses that
// rather than waive it. And only ONE cap is chamfered: decad does not compose a
// second modify op onto a cap-loop chamfer result, so the spec's walk over every
// end-cap face parallel to the sketch plane — both of them — cannot be built
// here.
//
// What survives is the rule the spec actually states about the selection: a
// bore never receives a chamfer. With a bore present the cap carries two
// circular edges and only the outer one is chamfered.
//
// Apply-chamfer-to-teeth 0 is the shipped default and means no chamfer: the
// step returns the body untouched, which is the early return chamferTeeth makes.
func stepChamferTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	body := stepBoreCut(t, doc, p)[0]
	if p["chamferTooth"] <= 0 {
		return []*decad.Body{body}
	}
	rim := decad.Edges(decad.CreatedBy(decad.CapEnd(body)), decad.Circular())
	if p["boreDiameter"] > 0 {
		rim = decad.Edges(decad.CreatedBy(decad.CapEnd(body)), decad.Circular(),
			decad.LongerThan(mm(boreRimCutoff(p))))
	}
	chamfered, err := body.Chamfer(rim.Exactly(1), mm(p["chamferTooth"]))
	if err != nil {
		t.Fatalf("chamfer the end cap: %v", err)
	}
	return []*decad.Body{chamfered}
}

// assertChamferTeeth measures the material an equal-distance chamfer of
// distance d takes off a circular rim of radius R: the ring between the
// cylinder and the frustum that replaces it,
// pi*d*(R^2 - (R^2 + R*(R-d) + (R-d)^2)/3). Getting that number back is what
// says the chamfer landed on the outer rim and on nothing else — a chamfer that
// had also caught the bore rim would remove more.
func assertChamferTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("chamfer step left %d bodies, want 1", len(bodies))
	}
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	disc := math.Pi * d.Root * d.Root * p["thickness"]
	want := disc
	if p["boreDiameter"] > 0 {
		want -= math.Pi * p["boreDiameter"] * p["boreDiameter"] / 4 * p["thickness"]
	}
	if c := p["chamferTooth"]; c > 0 {
		r := d.Root
		want -= math.Pi * c * (r*r - (r*r+r*(r-c)+(r-c)*(r-c))/3)
	}
	if got := volumeOf(t, bodies[0], "chamfered gear"); math.Abs(got-want) > 1e-6*want {
		t.Errorf("chamfered gear volume %.6f mm3, want %.6f mm3", got, want)
	}
}
