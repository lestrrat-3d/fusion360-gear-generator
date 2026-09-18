package spurgear_test

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

// The spur gear's solid steps, and the edge of what decad can build for them.
//
// Steps 7, 9, 10, 12 and 13 each turn a profile into a body. Three of them
// build here exactly as Fusion builds them; two need a substitute, and one
// step has none. Each substitution and its cost is stated where it is made,
// and the one step no substitute reaches is recorded below rather than only in
// the step list, because the proof is where the next reader looks for the
// missing check.
//
// Two substitutions run through every solid step here:
//
//   - The flank is sampled at [solidFlankSamples] points rather than the
//     spec's fifteen. decad admits a fit spline as a Tier A free-form segment
//     but holds the whole profile's free-form work to a fixed budget, and a
//     fifteen-point flank is over it: measured on the pinned engine, eight
//     points build and nine return ErrUnsupported. What the substitution costs
//     is the difference between the two flanks, and it is small and one-sided
//     — the chorded flank is inside the true one. On the default gear the
//     tooth cross-section comes out 3.23798 mm^2 against the fifteen-point
//     3.23902 mm^2, three parts in ten thousand, and every measurement below
//     is taken against the flank the proof actually drew rather than against
//     the fifteen-point one.
//   - The tooth's root arc is drawn explicitly instead of being derived by
//     splitting the root circle. Fusion derives it: the solid root circle is
//     cut in two where the tooth meets it, and the tooth profile takes one
//     piece. The bench draws the same derived boundary directly, which is the
//     model [PB-SKETCH-FIRST] already describes, and it has to: the engine
//     refuses to record a circle fragment whose trim it could not certify
//     exactly, which is what a crossing against a spline gives it. The
//     boundary is the same curve either way; what is not proven here is that
//     the SPLIT produces it, and that is proven in step 3 instead, where the
//     region the split produces is detected and its curve counts are asserted.
//
// The sketches in this file are geometry, not constraint schemes: their points
// are placed at computed coordinates and carry no constraints, because the
// constraint scheme is what step 3 proves and repeating it here would prove it
// twice and build nothing new.

// solidFlankSamples is the involute sample count the solid steps draw the
// flank at. The spec's InvoluteSteps is 15; see the free-form budget above.
const solidFlankSamples = 7

// solidTolerance is the slack an author's closed-form volume gets against a
// reading taken off an analytic prism. The prism's own bound is added by
// decadtest on top of it, so this covers only the rounding of the formula.
const solidTolerance = 1e-9

// ---------------------------------------------------------------------------
// The sketches the solid steps consume.

// toothBoundary draws the tooth cross-section as one closed loop: the two
// flanks, the tooth-top arc, and either the two flank-to-root lines plus the
// root arc between their feet, or — when the profile is embedded — the root arc
// between the flank starts themselves.
//
// The flank runs from max(base radius, root radius) out to the tip. That is
// what the tooth's boundary is: below the root radius the flank is inside the
// gear body and bounds nothing, which is exactly why the embedded profile
// closes with four curves instead of six.
func toothBoundary(t *testing.T, d dims) *sketch.Sketch {
	s := decadtest.NewSketch(t)
	origin := s.CreatePoint(0, 0)
	left, right := flankSamples(d, solidFlankSamples)
	leftPts, rightPts := placePoints(s, left), placePoints(s, right)
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank: %v", err)
	}
	// Counter-clockwise from the right flank's tip to the left flank's tip,
	// which is the direction addByCenterStartEnd takes them in.
	s.CreateArc(origin, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	if d.Embedded() {
		s.CreateArc(origin, rightPts[0], leftPts[0])
		return s
	}
	leftFoot := radialPoint(s, left[0], d.Root)
	rightFoot := radialPoint(s, right[0], d.Root)
	s.CreateLine(leftFoot, leftPts[0])
	s.CreateLine(rightFoot, rightPts[0])
	s.CreateArc(origin, rightFoot, leftFoot)
	return s
}

// discBoundary draws the gear body profile: the solid disc inside the root
// circle. It is not an annulus and the tip circle is no part of it — the tip
// circle is construction geometry and bounds no profile — so its area is the
// full pi*r^2, which the step below asserts.
func discBoundary(t *testing.T, d dims) *sketch.Sketch {
	s := decadtest.NewSketch(t)
	s.CreateCircle(s.CreatePoint(0, 0), d.Root)
	return s
}

// boreBoundary draws the bore circle of step 12.
func boreBoundary(t *testing.T, diameter float64) *sketch.Sketch {
	s := decadtest.NewSketch(t)
	s.CreateCircle(s.CreatePoint(0, 0), diameter/2)
	return s
}

// flankSamples is the spec's sampling loop at a chosen sample count, starting
// at max(base radius, root radius) rather than at the base circle: the part of
// the flank below the root radius is not on the tooth's boundary. The mirror,
// the rotation that lands the pitch crossing at +pi/(2N), and the requested
// angle are applied in that order, exactly as involute.Flanks applies them.
func flankSamples(d dims, samples int) (left, right []involute.Pt) {
	inner := math.Max(d.Base, d.Root)
	mirrored := make([]involute.Pt, 0, samples)
	for i := range samples {
		r := inner + (d.Tip-inner)*float64(i)/float64(samples-1)
		x, y, ok := involute.Point(d.Base, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, involute.Pt{X: x, Y: -y})
	}
	px, py, _ := involute.Point(d.Base, d.Pitch)
	rotate := math.Pi/(2*d.toothNumber) - math.Atan2(-py, px)
	for _, q := range mirrored {
		// The order is load-bearing and is involute.Flanks's own: rotate the
		// mirrored sample so its pitch crossing lands at +pi/(2N), mirror THAT
		// across +X to get the right flank, and only then turn both by the
		// requested angle. Mirroring before the rotation instead gives a right
		// flank that is not the left one reflected in the tooth's spine, and at
		// angle 0 it puts the two flank starts on top of each other — the root
		// arc between their feet then closes as a full circle and the tooth's
		// own region is never detected.
		lx, ly := involute.Rotate(q.X, q.Y, rotate)
		rx, ry := lx, -ly
		lx, ly = involute.Rotate(lx, ly, d.angle)
		rx, ry = involute.Rotate(rx, ry, d.angle)
		left = append(left, involute.Pt{X: lx, Y: ly})
		right = append(right, involute.Pt{X: rx, Y: ry})
	}
	return left, right
}

func placePoints(s *sketch.Sketch, in []involute.Pt) []*sketch.Point {
	out := make([]*sketch.Point, len(in))
	for i, q := range in {
		out[i] = s.CreatePoint(q.X, q.Y)
	}
	return out
}

// radialPoint is the point at radius r on the ray through seed, which is where
// a flank-to-root line's root end sits.
func radialPoint(s *sketch.Sketch, seed involute.Pt, r float64) *sketch.Point {
	theta := math.Atan2(seed.Y, seed.X)
	return s.CreatePoint(r*math.Cos(theta), r*math.Sin(theta))
}

// measuresVolume and measuresBounds label a reading with the name the step has
// for the body — `Extrude tooth`, `Gear Body` — rather than with decadtest's
// own index-and-recipe-step name, which does not say which feature is wrong.
func measuresVolume(t *testing.T, label string, body *decad.Body, want units.Value, opts ...decadtest.Option) {
	t.Helper()
	reading, err := body.Volume()
	if err != nil {
		t.Fatalf("%s: volume: %v", label, err)
	}
	decadtest.Measures(t, label+" volume", reading, want, opts...)
}

func measuresBounds(t *testing.T, label string, body *decad.Body, lo, hi r3.Vec, opts ...decadtest.Option) {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s: bounds: %v", label, err)
	}
	decadtest.MeasuresBox(t, label+" bounds", box, lo, hi, opts...)
}

// ---------------------------------------------------------------------------
// Step 7 — extrude the tooth.

// stepExtrudeTooth is the `Extrude tooth` feature: the single tooth
// cross-section, from the target plane to the Extrusion End Plane, as a new
// body. The to-entity extent is modelled as a distance of exactly Thickness,
// which is what that plane is: step 2 creates it as an offset of Thickness
// from the target plane and it exists only to give both extrudes one
// well-defined face to end on.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	s := toothBoundary(t, d)
	profile := decadtest.SolveRegion(t, s)
	toothArea = profile.Area
	body, err := doc.Extrude(s, profile, decad.Distance{
		D: units.Millimeters(p[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("Extrude tooth: %v", err)
	}
	return []*decad.Body{body}
}

// toothArea carries the drawn tooth's cross-section area from the build to the
// assertion. It is package-level state, so stepExtrudeTooth and
// stepPatternTeeth below must stay on the SERIAL runners: two cases running at
// once would overwrite each other's reading and the proof would report a wrong
// verdict rather than fail. The area cannot be taken again in the assertion,
// which sees the body and not the sketch it came from.
var toothArea float64

func assertExtrudeTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	tooth := bodies[0]
	thickness := p[pThickness]

	// A prism's volume is its cross-section times its height. The area comes
	// from the sketch engine and the volume from decad's own integration of the
	// swept boundary, so the two engines are answering the same question
	// independently here.
	measuresVolume(t, "Extrude tooth", tooth, units.CubicMillimeters(toothArea*thickness),
		decadtest.WithinRel(units.Scalar(solidTolerance)))

	// The extrude runs from the target plane to the Extrusion End Plane and no
	// further, and the tooth reaches the tip circle and stops there.
	box, err := tooth.Bounds()
	if err != nil {
		t.Fatalf("Extrude tooth: bounds: %v", err)
	}
	decadtest.Measures(t, "Extrude tooth near cap", scalarAt(box.Min.Z), units.Millimeters(0),
		decadtest.Within(units.Millimeters(solidTolerance)))
	decadtest.Measures(t, "Extrude tooth far cap", scalarAt(box.Max.Z), units.Millimeters(thickness),
		decadtest.WithinRel(units.Scalar(solidTolerance)))
	decadtest.Measures(t, "Extrude tooth outer reach", scalarAt(box.Max.X), units.Millimeters(d.Tip),
		decadtest.WithinRel(units.Scalar(1e-6)))
}

// scalarAt wraps a plain coordinate as an exact reading so it can go through
// the same comparison rule as everything else. The coordinate is read off a
// bounded box whose own bound decadtest adds separately; this carries no bound
// of its own because the number itself is what is being compared.
func scalarAt(v float64) decad.Measurement {
	return decad.Measurement{Value: units.Millimeters(v), Bound: units.Millimeters(0)}
}

// ---------------------------------------------------------------------------
// Step 9 — extrude the gear body.

// stepExtrudeBody is the `Extrude body` feature: the disc inside the root
// circle, from the target plane to the Extrusion End Plane, as a new body
// named `Gear Body`.
func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	s := discBoundary(t, d)
	profile := decadtest.SolveRegion(t, s)
	body, err := doc.Extrude(s, profile, decad.Distance{
		D: units.Millimeters(p[pThickness]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("Extrude body: %v", err)
	}
	return []*decad.Body{body}
}

func assertExtrudeBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	body := bodies[0]
	thickness := p[pThickness]

	// pi*r^2*h, not pi*(tip^2 - root^2)*h: the body profile is the full disc.
	// An annulus would fail here, and so would a disc taken at the tip radius.
	measuresVolume(t, "Gear Body", body,
		units.CubicMillimeters(math.Pi*d.Root*d.Root*thickness),
		decadtest.WithinRel(units.Scalar(solidTolerance)))
	measuresBounds(t, "Gear Body", body,
		r3.NewVec(-d.Root, -d.Root, 0), r3.NewVec(d.Root, d.Root, thickness),
		decadtest.WithinRel(units.Scalar(1e-9)))

	// The cylindrical face is the one step 9 builds the `Gear Center`
	// construction axis off, and the two planar caps are where it finds
	// ctx.extrusionExtent: the far one is parallel to the sketch plane and not
	// coplanar with it. A disc that came back with anything else would leave
	// both searches with nothing to find.
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindPlane:    2,
		decad.KindCylinder: 1,
	})
}

// ---------------------------------------------------------------------------
// Step 10 — pattern the teeth and join them.

// stepPatternTeeth is the circular pattern of step 10: the tooth body repeated
// `Tooth Number` times about the `Gear Center` axis over a full turn, with
// isSymmetric false, so copy k sits at k * 360/N.
//
// The JOIN is not built here, and no substitute reaches it. The combine's two
// operands are the seed tooth and the gear body, and they meet in the two ways
// this evaluator refuses to classify at once: they share both cap planes,
// since both extrudes run from the target plane by the same Thickness, and
// they touch along the root arc without interpenetrating. Sinking the tooth
// inside the root circle answers the second and leaves the first — measured on
// the pinned engine, the union returns ErrUnsupported either way, naming the
// tangent contact before the sink and the coplanar facets after it.
//
// So what this step proves is the pattern's own content, which is the part
// step 10 decides: the count, the spacing, and that each copy is a rigid
// repeat of the seed. The gear body's own solidity is proven by step 9 and the
// tooth's by step 7, so the only thing left unproven is that joining two
// proven solids yields a solid, which is a property of the operation rather
// than of this gear.
//
// The same refusal is why step 11 has no proof function at all. The root
// fillet rounds the corner where the valley floor meets a tooth flank, and
// that corner exists only on the joined body. Two substitutes were measured
// and neither survives: cutting one valley out of a tip-radius blank produces
// the corner but leaves a faceted body, and this evaluator fillets a straight
// prism only; extruding one tooth pitch of the gear's cross-section gives a
// straight prism whose concave axial edges the selector does find — both root
// corners, and nothing else — but the fillet then reports the two walls as
// meeting smoothly and refuses the corner. Step 11 is therefore [PROSE], and
// what is left unchecked is the fillet geometry itself; the radius it is given
// is arithmetic the step list carries and the edge set it collects is a Fusion
// topology search with no bench counterpart.
func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	thickness := units.Millimeters(p[pThickness])

	// The gear body the teeth are joined onto is deliberately NOT built into
	// this document, and neither is a second tooth. Two teeth extruded from the
	// same plane share both cap planes, and the verifier reads such a pair as a
	// contact it cannot classify — measured on the pinned engine, a document
	// holding the seed and its copies comes back Suspect with an
	// undecided_pair diagnostic per neighbour, and one holding the disc as well
	// adds an unsupported_pair_contact for every tooth. So the pattern is walked
	// one placement at a time: [decad.Body.Placed] retires the body it moves, so
	// exactly one tooth is live at any moment and every reading is taken as it
	// passes.
	toothSketch := toothBoundary(t, d)
	profile := decadtest.SolveRegion(t, toothSketch)
	toothArea = profile.Area
	seed, err := doc.Extrude(toothSketch, profile, decad.Distance{D: thickness, Dir: decad.Along})
	if err != nil {
		t.Fatalf("Extrude tooth: %v", err)
	}
	patternSeedCentroid, err = seed.Centroid()
	if err != nil {
		t.Fatalf("seed tooth centroid: %v", err)
	}
	patternSeedVolume, err = seed.Volume()
	if err != nil {
		t.Fatalf("seed tooth volume: %v", err)
	}

	count := int(d.toothNumber)
	patternCentroids = patternCentroids[:0]
	patternVolumes = patternVolumes[:0]
	// One step of a full turn divided by the quantity, taken `quantity` times.
	// That is what totalAngle = '360 deg' with isSymmetric false says: the
	// copies are spread over a whole turn rather than half of one either side
	// of the seed, and the last step lands back on the seed rather than on top
	// of a copy.
	step, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1),
		units.Radians(2*math.Pi/float64(count)))
	if err != nil {
		t.Fatalf("pattern step rotation: %v", err)
	}
	current := seed
	for k := 1; k <= count; k++ {
		current, err = current.Placed(step)
		if err != nil {
			t.Fatalf("pattern placement %d: %v", k, err)
		}
		centroid, err := current.Centroid()
		if err != nil {
			t.Fatalf("pattern placement %d centroid: %v", k, err)
		}
		volume, err := current.Volume()
		if err != nil {
			t.Fatalf("pattern placement %d volume: %v", k, err)
		}
		patternCentroids = append(patternCentroids, centroid)
		patternVolumes = append(patternVolumes, volume)
	}
	return []*decad.Body{current}
}

// The pattern's readings travel from the build to the assertion in these
// package-level variables, because a placement retires the body it moved and
// the assertion cannot take the reading again. That is why stepPatternTeeth
// stays on the SERIAL runner: two cases running at once would overwrite each
// other's readings and the proof would report a wrong verdict rather than fail.
var (
	patternSeedCentroid decad.VecMeasurement
	patternSeedVolume   decad.Measurement
	patternCentroids    []decad.VecMeasurement
	patternVolumes      []decad.Measurement
)

func assertPatternTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	count := int(d.toothNumber)
	if len(patternCentroids) != count {
		t.Fatalf("the pattern took %d placements, want %d", len(patternCentroids), count)
	}

	for k := 1; k <= count; k++ {
		// A patterned copy is the seed moved, and nothing about it may differ
		// but its position. Both sides are readings, so Agree counts both bounds.
		decadtest.Agree(t, "patterned tooth against its seed", patternVolumes[k-1], patternSeedVolume)

		// And it sits exactly k seventeenths — k quantity-ths — of a turn round
		// from the seed. This is where a pattern set to half a turn, or one that
		// counted the seed twice, comes apart, and the k == count row is the
		// closure: the last placement lands back on the seed.
		angle := 2 * math.Pi * float64(k) / float64(count)
		wantX, wantY := involute.Rotate(patternSeedCentroid.Value.X, patternSeedCentroid.Value.Y, angle)
		decadtest.MeasuresVec(t, "patterned tooth centroid", patternCentroids[k-1],
			r3.NewVec(wantX, wantY, patternSeedCentroid.Value.Z),
			decadtest.Within(units.Millimeters(1e-9)))
	}

	// The teeth do not run into each other, and that is a property of the tooth
	// rather than of the pattern: one tooth's angular half-width at the tip has
	// to stay inside half the angular pitch. The gear is unbuildable otherwise,
	// whatever the pattern does.
	left, _ := flankSamples(d, solidFlankSamples)
	tip := left[len(left)-1]
	halfWidth := math.Atan2(tip.Y, tip.X) - d.angle
	if halfWidth <= 0 || halfWidth >= math.Pi/d.toothNumber {
		t.Errorf("the tooth spans %.6f rad either side of its spine, and the pattern gives it %.6f",
			halfWidth, math.Pi/d.toothNumber)
	}

	// Step 11's own guard, checked here because this is the nearest thing the
	// proof builds to the fillet it cannot reach. The root fillet runs only
	// when FilletRadius is above zero, and FilletRadius is
	// (ToothSpaceArcAtRoot / 2) * FilletClearance, which is 0.45 of the valley
	// arc — under the half-arc at which fillets from adjacent flanks would meet
	// at the valley midpoint, by the 0.9 clearance and for every gear. The
	// valley arc itself is not always positive: pi/N - 2*(tan(a) - a) goes
	// negative at a high tooth count and a large pressure angle, at 60 teeth
	// and 25 degrees among others, and the guard is what keeps a negative
	// radius out of filletFeatures.add.
	arc := toothSpaceArc(d, p[pPressureAngle])
	radius := filletRadius(d, p[pPressureAngle])
	if radius <= 0 || radius >= arc/2 {
		t.Errorf("the root fillet radius is %.6f mm against a %.6f mm valley arc; step 11 runs it "+
			"only above zero and it has to stay under half the arc", radius, arc)
	}

	// The body left standing is the seed back at its own place, so the whole
	// turn closed on it.
	final, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("final tooth centroid: %v", err)
	}
	decadtest.MeasuresVec(t, "the tooth the full turn came back to", final,
		patternSeedCentroid.Value, decadtest.Within(units.Millimeters(1e-9)))
}

// ---------------------------------------------------------------------------
// Step 12 — cut the bore.

// stepBoreCut is the bore's extrude-cut: the Bore Profile circle, from the
// target plane to ctx.extrusionExtent — the gear body's far end-cap face — so
// the hole goes all the way through whatever Thickness is, affecting only the
// gear body.
//
// The cut is made against the gear body disc of step 9 rather than against the
// finished gear, because step 10's join cannot be built here. Nothing is lost:
// the bore is concentric with the anchor and its diameter is well under the
// root circle, so every face it creates lies inside the disc and the teeth,
// which sit outside the root circle, are not reached by it.
//
// The tool is swept symmetrically past both caps instead of stopping on the
// far face. A tool that stopped exactly on it would share that cap plane with
// the target, which this evaluator refuses to classify; overshooting removes
// the same material, which is the whole point of ending the cut on the far
// face in the first place ([PB-THROUGH-CUT] describes the same shape for the
// same reason).
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	bore := p[pBoreDiameter]
	if bore <= 0 {
		proofkit3d.Unmodelled(t, "step 12 returns before cutting anything at a bore diameter of %g", bore)
	}
	thickness := p[pThickness]

	disc := discBoundary(t, d)
	gearBody, err := doc.Extrude(disc, decadtest.SolveRegion(t, disc),
		decad.Distance{D: units.Millimeters(thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("Extrude body: %v", err)
	}
	toolSketch := boreBoundary(t, bore)
	tool, err := doc.Extrude(toolSketch, decadtest.SolveRegion(t, toolSketch),
		decad.Symmetric{D: units.Millimeters(2 * thickness)})
	if err != nil {
		t.Fatalf("bore tool: %v", err)
	}
	bored, err := decad.Cut(gearBody, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	bore := p[pBoreDiameter]
	thickness := p[pThickness]
	bored := bodies[0]

	// A through hole, not a blind one: the volume removed is the full
	// pi*(D/2)^2*Thickness, which is the fact the to-entity extent buys.
	measuresVolume(t, "bored Gear Body", bored,
		units.CubicMillimeters(math.Pi*(d.Root*d.Root-bore*bore/4)*thickness),
		decadtest.WithinRel(units.Scalar(solidTolerance)))
	measuresBounds(t, "bored Gear Body", bored,
		r3.NewVec(-d.Root, -d.Root, 0), r3.NewVec(d.Root, d.Root, thickness),
		decadtest.WithinRel(units.Scalar(1e-9)))
	// Two cylinders — the rim and the bore wall — and the two caps the hole
	// went through.
	decadtest.HasSurfaceKinds(t, bored, map[decad.SurfaceKind]int{
		decad.KindPlane:    2,
		decad.KindCylinder: 2,
	})
}

// ---------------------------------------------------------------------------
// Step 13 — chamfer the completed gear.

// stepChamferTeeth is the completed-gear chamfer: every edge of every planar
// face parallel to the Gear Profile sketch plane, equal-distance, with a bore
// edge excluded.
//
// It runs on the bored gear body rather than on the finished gear, for the
// reason step 10 gives, and what that costs is the tooth part of the edge set:
// the tooth flanks, tooth tops and root arcs are free-form or derived from a
// free-form neighbour, and this evaluator's corner rewrite does not support a
// free-form boundary segment — measured, chamfering the tooth prism's own cap
// edges returns ErrUnsupported. What survives is the rest of the rule, and it
// is the half that is easy to get wrong: the chamfer is equal-distance, it is
// applied to the end-cap edges, and a bore edge is left out of it.
//
// The bore edge is excluded by its LENGTH rather than by its radius, which is
// the same test in a selector that has no radius predicate: a full circle's
// length is 2*pi*r, so a threshold between the bore circumference and the root
// circumference separates them exactly as the spec's `Circle3DCurveType` edge
// whose radius is the bore diameter over two does.
func stepChamferTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	chamfer := p[pChamfer]
	if chamfer <= 0 {
		proofkit3d.Unmodelled(t, "step 13 returns before chamfering anything at a distance of %g", chamfer)
	}
	bored := stepBoreCut(t, doc, p)[0]

	all, err := decad.Edges(decad.Circular()).SelectEdges(bored)
	if err != nil {
		t.Fatalf("selecting every circular cap edge: %v", err)
	}
	if len(all) != 4 {
		t.Fatalf("the bored gear body carries %d circular edges, want 4 (two rim, two bore)", len(all))
	}

	kept, err := chamferEdges(p).SelectEdges(bored)
	if err != nil {
		t.Fatalf("selecting the chamfered edges: %v", err)
	}
	if len(kept) != 2 {
		t.Fatalf("the chamfer would take %d edges, want the 2 rim edges with both bore edges excluded",
			len(kept))
	}

	chamfered, err := bored.Chamfer(chamferEdges(p), units.Millimeters(chamfer))
	if err != nil {
		t.Fatalf("chamfer: %v", err)
	}
	return []*decad.Body{chamfered}
}

// chamferEdges is the edge set of step 13, minus the tooth edges no substitute
// reaches: the circular end-cap edges whose circumference puts them past the
// bore. The threshold sits halfway between the two circumferences in the log,
// so it is a genuine separation rather than a value tuned to one gear.
func chamferEdges(p map[string]float64) *decad.EdgeQuery {
	d := derive(p)
	threshold := 2 * math.Pi * math.Sqrt(d.Root*p[pBoreDiameter]/2)
	return decad.Edges(decad.Circular(), decad.LongerThan(units.Millimeters(threshold)))
}

func assertChamferTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	chamfer := p[pChamfer]
	bore := p[pBoreDiameter]
	thickness := p[pThickness]
	body := bodies[0]

	// An equal-distance chamfer of c on the rim of a cylinder of radius R takes
	// a cone frustum off each end: the slab pi*R^2*c less the frustum
	// (pi*c/3)*(R^2 + R*(R-c) + (R-c)^2). The slack is wide because the
	// chamfered wall is held as facets and its chords fall inside the cone; at
	// the radii swept here the shortfall is about two parts in a thousand.
	rim := math.Pi*d.Root*d.Root*chamfer -
		math.Pi*chamfer/3*(d.Root*d.Root+d.Root*(d.Root-chamfer)+(d.Root-chamfer)*(d.Root-chamfer))
	bored := math.Pi * (d.Root*d.Root - bore*bore/4) * thickness
	measuresVolume(t, "chamfered gear", body, units.CubicMillimeters(bored-2*rim),
		decadtest.WithinRel(units.Scalar(0.01)))

	// The bore is untouched: its wall is still a full-height cylinder, so the
	// body still reaches both cap planes at the bore radius. A chamfer that had
	// taken the bore edges would have pulled the material back from them.
	measuresBounds(t, "chamfered gear", body,
		r3.NewVec(-d.Root, -d.Root, 0), r3.NewVec(d.Root, d.Root, thickness),
		decadtest.WithinRel(units.Scalar(1e-6)))
	remaining, err := decad.Edges(decad.Circular()).SelectEdges(body)
	if err != nil {
		t.Fatalf("selecting the circular edges of the chamfered gear: %v", err)
	}
	// Two bore edges untouched, and each chamfered rim edge replaced by the two
	// edges of its chamfer face.
	if len(remaining) != 6 {
		t.Errorf("the chamfered gear carries %d circular edges, want 6: the 2 bore edges and "+
			"the 2 edges each chamfer face brought", len(remaining))
	}
}
