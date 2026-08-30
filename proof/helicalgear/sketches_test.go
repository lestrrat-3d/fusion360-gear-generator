package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

func deg(d float64) float64 { return d * math.Pi / 180 }

// twistedProfileCases is the regime the twisted profile has to hold across.
//
// It is spur's own Sketch Discipline regime, because the twisted profile IS the
// spur tooth generator run at a non-zero angle and nothing about it is helical's
// except the value of that angle. Each group is here because the scheme fails
// differently outside it:
//
//   - SIZE. Coarse and fine module and tooth-count pairs, because the rib
//     chain's dimensions scale with the tooth and the conditioning does not.
//   - THE WHOLE SIGNED RANGE OF THE ANGLE. A left-hand helix is a NEGATIVE
//     helix angle and the dialog accepts one, so a negative angle is swept
//     beside its positive twin: the confirming angular dimension carries the
//     sign, and a scheme that drops or flips it still solves at +angle and comes
//     out mirrored at -angle. Zero is the spur-equivalent baseline. A quarter
//     turn is swept both ways, where |sin| > |cos| swaps which axis the rib and
//     the midpoint chain take.
//   - THE RIB COUNT. One rib, one across-spine dimension and one chain
//     dimension exist per involute sample, so a four-sample case is where one
//     missing or redundant dimension is a large fraction of the system.
//   - BOTH ROUTES INTO THE EMBEDDED SHAPE. High tooth count at the ordinary 20
//     degree pressure angle, and a moderate tooth count at a large pressure
//     angle. Helical does not support an embedded tooth; the two cases are here
//     to prove WHY, by measuring the four-curve loop its fixed six-curve profile
//     search cannot match.
var twistedProfileCases = []proofkit.Case{
	{Name: "default_m1_n17_helix14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 15}},
	{Name: "lefthand_m1_n17_helix-14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 15}},
	{Name: "spurbaseline_m1_n17_helix0", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": 0, "involuteSteps": 15}},
	{Name: "quarterturn_m1_n17_helix90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(90), "involuteSteps": 5}},
	{Name: "quarterturn_m1_n17_helix-90", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-90), "involuteSteps": 5}},
	{Name: "coarse_m3_n15_helix35", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20),
		"helixAngle": deg(35), "involuteSteps": 5}},
	{Name: "fine_m2_n20_helix-25", Params: map[string]float64{
		"module": 2, "toothNumber": 20, "pressureAngle": deg(20),
		"helixAngle": deg(-25), "involuteSteps": 5}},
	{Name: "lowribcount_m1_n12_helix14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 12, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 4}},
	{Name: "embedded_by_toothcount_m1_n45", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 5}},
	{Name: "embedded_by_pressureangle_m1_n20_pa30", Params: map[string]float64{
		"module": 1, "toothNumber": 20, "pressureAngle": deg(30),
		"helixAngle": deg(14.5), "involuteSteps": 5}},
}

// stepTwistedGearProfileSketch draws the "Twisted Gear Profile" sketch: the spur
// tooth generator run at angle = HelixAngle, on the offset construction plane.
//
// Everything here is spur's constraint recipe taken verbatim — helical passes an
// angle and changes nothing else — so the recipe anchors are cited at the place
// each one is applied. What this proves that the spur sketch bench does not is
// the angle != 0 path: the tooth is drawn already rotated by the helix angle and
// the spine's angular dimension confirms that rotation, both required.
//
// The sketch is drawn on the world XY plane, not on the offset plane the Fusion
// code puts it on. proofkit.Run hands the build a sketch it made itself, and a
// sketch's constraint system is plane-local, so the offset changes nothing here;
// the offset is what the helix-plane step proves.
func stepTwistedGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d, teeth, steps, angle := dims(p)

	proofkit.Step(t, "local origin on the projected Tools anchor")
	// [SPUR-F-ANCHOR-CHAIN]: the Tools-sketch projection of the user's anchor is
	// the canonical handle, re-projected into this sketch. A projection is
	// externally located and the solver never moves it, which is what a reference
	// point is. [SPUR-F-LOCAL-ORIGIN]: the sketch's own movable local origin is a
	// fresh point at (0,0), never the sketch's origin point, and everything is
	// drawn relative to it. The anchoring coincidence is step 5, performed inside
	// draw() itself.
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch: projected anchorPoint")
	origin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "four gear circles, all sharing the local origin as centre")
	// Spur step 3. The root circle is solid and the other three are construction,
	// so the root circle is what bounds a region and the tip, base and pitch
	// circles bound none. Each carries a driving diameter dimension
	// ([PB-DRIVING-DIM]); the centre is SHARED, not re-coincided
	// ([PB-SHARE-XOR-COINCIDENT]).
	circle := func(radius float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, radius)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*radius))
		return c
	}
	circle(d.Root, false)
	tipCircle := circle(d.Tip, true)
	circle(d.Base, true)
	circle(d.Pitch, true)

	proofkit.Step(t, "involute flanks, drawn already rotated by the helix angle")
	// Spur step 4 and [SPUR-F-ROTATE-CONFIRM]'s first half: the geometry is drawn
	// at its rotated position in the point math, not swung there afterwards by
	// the dimension. proof/involute owns the sampling, the mirror across +X, the
	// pitch-crossing rotation and the final rotation by the requested angle.
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank: %v", err)
	}

	proofkit.Step(t, "tooth-top arc centred on the local origin")
	// [SPUR-F-TOOTHTOP-ARC]: the arc shares the local origin as its centre and
	// the two flanks' end points, and carries no diameter dimension. A free
	// centre plus a diameter would leave an arc of the same radius that bulges
	// inward through the tooth, which is a second discrete answer at DOF 0 and
	// exactly what the ambiguity probe in this gate refuses.
	topX, topY := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))
	s.CreateArc(origin, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])

	proofkit.Step(t, "spine, +X reference and the confirming angular dimension")
	// [SPUR-F-SPINE]: the spine shares both existing points. The +X reference is
	// built for EVERY angle including zero, its far end pinned by a signed
	// horizontal and a signed vertical distance from the origin rather than by a
	// point-on-circle, which has two answers. The angular dimension runs FROM the
	// reference TO the spine, and its value is the requested angle
	// ([SPUR-F-ROTATE-CONFIRM]'s second half). A plain horizontal on the spine at
	// angle 0 would fix the direction and not the sense, leaving the tooth free
	// to settle 180 degrees around.
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	proofkit.Step(t, "one rib per fit-point index, in [SPUR-F-RIBS] order")
	// The rib takes the axis ACROSS the spine and the midpoint chain takes the
	// one ALONG it; vertical rib and horizontal chain while |cos| >= |sin|, and
	// both swapped otherwise. That reduces to spur's pair at angle 0 and is why
	// the quarter-turn cases are in the table. The order is fixed: rib, axis
	// dimension, midpoint seeded on the spine, coincident to the spine, midpoint
	// of the rib, then perpendicular — and the LAST rib gets no perpendicular,
	// because the tooth-top arc already holds those two ends at equal radius
	// either side of the spine. The chain starts at the local origin, without
	// which the whole chain slides along the spine as one free unit.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previous := origin
	previousX, previousY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, midX-previousX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, midY-previousY))
		}
		previous, previousX, previousY = mid, midX, midY
	}

	if !d.Embedded() {
		proofkit.Step(t, "flank-to-root lines, two axis dimensions each")
		// [SPUR-F-FLANK-ROOT]: each stub shares the flank spline's start point
		// and its root end is placed by exactly two axis dimensions from the
		// local origin — never by "on the root circle" plus "origin on the line",
		// which the far intersection satisfies equally well. The engine's
		// distances are signed and the seed sits at the computed position, which
		// is how Fusion's magnitude-plus-captured-direction crosses over
		// ([PB-DIM-VALUE-SEMANTICS]).
		stub := func(flankStart *sketch.Point, seed involute.Pt) {
			x, y := onRoot(d, seed)
			end := s.CreatePoint(x, y)
			s.CreateLine(end, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, end, x),
				sketch.NewVerticalDistance(origin, end, y),
			)
		}
		stub(leftPts[0], left[0])
		stub(rightPts[0], right[0])
	}

	assertTwistedProfileLoops(t, s, d)
}

// assertTwistedProfileLoops holds the sketch to the curve counts the loft step
// searches on.
//
// [HELI-F-LOFT] finds BOTH loft sections with a fixed nurbs=2, arcs=2, lines=2 —
// the non-embedded six-curve tooth — and never reads ctx.toothProfileIsEmbedded.
// So the count is not decoration: it is the key the step matches on, and it is
// asserted here on the loop the sketch actually closed, with the root circle
// solid and split by the stubs exactly as Fusion splits it.
//
// The embedded cases are asserted on the other side of that branch: four curves,
// zero lines, which the fixed six-curve search cannot match. That is the
// documented limitation stated as a measurement rather than as prose.
//
// One harness difference is worth recording. Fusion reports the disc inside the
// root circle as two arcs — the two pieces the stubs cut the root circle into,
// which is spur's body-extrude key. The sketch engine reports that region's
// boundary as the whole circle, one edge, while still handing the tooth loop the
// fragment it needs. The tooth count, which is helical's key, agrees; the disc
// count does not, and no helical step reads it.
func assertTwistedProfileLoops(t testing.TB, s *sketch.Sketch, d involute.Dimensions) {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("gear profile closed %d regions, want 2 (the tooth and the root disc)", len(profiles))
	}
	tooth := smallestProfile(profiles)
	got := countCurves(tooth)
	want := curveCounts{nurbs: 2, arcs: 2, lines: 2}
	if d.Embedded() {
		want = curveCounts{nurbs: 2, arcs: 2, lines: 0}
	}
	if got != want {
		t.Errorf("tooth loop is nurbs=%d arcs=%d lines=%d, want nurbs=%d arcs=%d lines=%d",
			got.nurbs, got.arcs, got.lines, want.nurbs, want.arcs, want.lines)
	}
	if !tooth.Valid {
		t.Error("tooth loop is not an extrudable region, so no loft section can be found in it")
	}
	if d.Embedded() && got.lines == 2 {
		t.Error("an embedded tooth drew flank-to-root lines; the embedded test is strict <")
	}
	if d.Embedded() {
		t.Logf("embedded: the loop carries %d curves and no lines, so loftTooth's fixed "+
			"nurbs=2, arcs=2, lines=2 search finds nothing and an embedded helical gear "+
			"cannot be built at all", got.nurbs+got.arcs+got.lines)
	}
}
