package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// S05 — Normalize the Target Plane
// ---------------------------------------------------------------------------

// normalizePlaneCases covers both sides of step 1's branch: a selection that is
// already a ConstructionPlane, which is used as it stands, and a selection that
// is not — a planar face — for which a coplanar construction plane is built at
// offset zero.
var normalizePlaneCases = []proofkit.Case{
	{Name: "selection_is_construction_plane", Params: map[string]float64{
		"selectionIsPlane": 1, "thickness": 10,
	}},
	{Name: "selection_is_planar_face", Params: map[string]float64{
		"selectionIsPlane": 0, "thickness": 10,
	}},
}

// stepNormalizeTargetPlane proves that the plane every later sketch is built on
// is the plane the user selected: a selection that is already a
// ConstructionPlane is passed through, and any other selection is replaced by a
// construction plane offset from it by zero, which must be coplanar with it.
//
// SUBSTITUTION. proofkit hands the build a sketch already created on the world
// XY datum, so the normalized plane cannot be the plane that sketch is drawn
// on. The proof creates the normalized plane in the same world and asserts its
// frame against the selected plane's directly. What that leaves unproved is
// only that a sketch drawn on the normalized plane lands where a sketch on the
// selection would; the frames being coplanar is what decides that, and it is
// what is asserted.
func stepNormalizeTargetPlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "step 1: normalize the user's target-plane selection")
	selected := s.Plane()
	target := selected
	if p["selectionIsPlane"] != 1 {
		var err error
		// setByOffset(selectedPlane, ValueInput.createByReal(0)): the offset is
		// a value, not a bare number, and it is zero so the result is coplanar.
		target, err = s.World().CreateOffsetPlane(selected, 0)
		if err != nil {
			t.Fatalf("coplanar construction plane at offset 0: %v", err)
		}
	}

	selFrame, err := selected.Frame()
	if err != nil {
		t.Fatalf("selected plane frame: %v", err)
	}
	tgtFrame, err := target.Frame()
	if err != nil {
		t.Fatalf("normalized plane frame: %v", err)
	}
	sn, tn := selFrame.N(), tgtFrame.N()
	if math.Abs(math.Abs(sn.Dot(tn))-1) > tol {
		t.Errorf("normalized plane is not parallel to the selection: normals %v and %v", sn, tn)
	}
	gap := tgtFrame.Origin().Sub(selFrame.Origin()).Dot(sn)
	if math.Abs(gap) > tol {
		t.Errorf("normalized plane is offset from the selection by %g mm, want 0", gap)
	}

	proofkit.Step(t, "the Tools sketch is then created on that plane")
	s.CreateReferencePoint(0, 0, "anchor point projected into the Tools sketch")
}

// ---------------------------------------------------------------------------
// S06 — Tools sketch
// ---------------------------------------------------------------------------

// toolsSketchCases move the user's anchor away from the plane origin, because
// the whole point of the Tools sketch is that the gear follows the anchor
// rather than the plane.
var toolsSketchCases = []proofkit.Case{
	{Name: "anchor_at_plane_origin", Params: map[string]float64{"anchorX": 0, "anchorY": 0}},
	{Name: "anchor_off_origin", Params: map[string]float64{"anchorX": 37.5, "anchorY": -12.25}},
	{Name: "anchor_far_off_origin", Params: map[string]float64{"anchorX": -400, "anchorY": 250}},
}

// stepToolsSketch proves the Tools sketch: one projection of the user's anchor
// point and no geometry of its own. The projection is the canonical handle
// every later sketch re-projects from, so what has to hold is that it is fully
// determined by its source and carries no free motion of its own.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "step 2: project the Anchor Point into the Tools sketch")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "user Anchor Point")
	anchor.SetName("ctx.anchorPoint")

	if !anchor.IsReference() {
		t.Error("the Tools-sketch anchor must be a projection of the user's entity, not local geometry")
	}
	if !anchor.IsFullyConstrained() {
		t.Error("the projected anchor still carries free motion, so nothing chained to it is located")
	}
	if got := [2]float64{anchor.X(), anchor.Y()}; got != [2]float64{p["anchorX"], p["anchorY"]} {
		t.Errorf("projected anchor at %v, want the source position (%g, %g)", got, p["anchorX"], p["anchorY"])
	}
}

// ---------------------------------------------------------------------------
// S07 — Extrusion End Plane
// ---------------------------------------------------------------------------

// extrusionEndPlaneCases sweep Thickness across the range the dialog allows: a
// thin gear, the 10 mm default, and a thick one. Thickness is declared positive,
// so the ends of the range are both positive.
var extrusionEndPlaneCases = []proofkit.Case{
	{Name: "thin_0p2mm", Params: map[string]float64{"thickness": 0.2}},
	{Name: "default_10mm", Params: map[string]float64{"thickness": 10}},
	{Name: "thick_250mm", Params: map[string]float64{"thickness": 250}},
}

// stepExtrusionEndPlane proves the Extrusion End Plane: the offset construction
// plane both extrudes end on, so the tooth and the body finish flush on one
// well-defined face. What has to hold is that it is parallel to the target
// plane and exactly Thickness away from it, in the direction the extrudes run.
//
// SUBSTITUTION. As in step 1, the plane is asserted from the world rather than
// by drawing on it; proofkit's sketch is on the target plane, which is the
// plane the offset is measured from.
func stepExtrusionEndPlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	thickness := p["thickness"]
	proofkit.Step(t, "step 2: Extrusion End Plane at Thickness = %g mm from the target plane", thickness)

	target := s.Plane()
	// setByOffset(targetPlane, ValueInput.createByReal(thickness)).
	end, err := s.World().CreateOffsetPlane(target, thickness)
	if err != nil {
		t.Fatalf("Extrusion End Plane: %v", err)
	}
	targetFrame, err := target.Frame()
	if err != nil {
		t.Fatalf("target plane frame: %v", err)
	}
	endFrame, err := end.Frame()
	if err != nil {
		t.Fatalf("Extrusion End Plane frame: %v", err)
	}
	n := targetFrame.N()
	if math.Abs(math.Abs(n.Dot(endFrame.N()))-1) > tol {
		t.Errorf("the Extrusion End Plane is not parallel to the target plane")
	}
	if got := endFrame.Origin().Sub(targetFrame.Origin()).Dot(n); math.Abs(got-thickness) > tol {
		t.Errorf("the Extrusion End Plane sits %g mm from the target plane, want Thickness = %g", got, thickness)
	}

	proofkit.Step(t, "the Tools sketch on the target plane is what the offset is measured from")
	s.CreateReferencePoint(0, 0, "anchor point projected into the Tools sketch")
}

// ---------------------------------------------------------------------------
// S08 — Gear Profile sketch
// ---------------------------------------------------------------------------

// gearProfileCases are the regime the spec's Sketch Discipline section says the
// scheme has to hold across, one case per way of leaving it.
//
//   - Size: a coarse module with few teeth and a fine module with many, because
//     the rib chain's dimensions scale with the tooth and the conditioning of
//     the system does not.
//   - The whole signed range of draw()'s angle: zero for spur, a positive and a
//     NEGATIVE helix angle (a left-hand helix passes a negative value, and a
//     scheme that drops the sign still solves at +angle and comes out mirrored
//     at -angle), a quarter turn where |sin| > |cos| swaps which axis the rib
//     and chain dimensions take, and the half turn the bevel virtual tooth uses.
//     45 degrees is in the table as well: it is where |cos| and |sin| are equal
//     and the axis choice tips over, which the spec states as a rule without
//     naming the angle it turns at.
//   - The rib count: the standard 15 involute samples and the low end, where a
//     single missing or redundant dimension is a large fraction of the system.
//   - Both routes into the embedded shape: a high tooth count at the ordinary
//     20 degree pressure angle, and a moderate tooth count at a large one.
//   - The anchor off the plane origin, which is what step 5 has to drag the
//     whole drawing onto.
var gearProfileCases = []proofkit.Case{
	{Name: "default_m1_t17_angle0", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": 0,
	}},
	{Name: "coarse_m8_t9", Params: map[string]float64{
		"module": 8, "toothNumber": 9, "pressureAngle": deg(20), "involuteSteps": 15, "angle": 0,
	}},
	{Name: "fine_m0p3_t40", Params: map[string]float64{
		"module": 0.3, "toothNumber": 40, "pressureAngle": deg(20), "involuteSteps": 15, "angle": 0,
	}},
	{Name: "helix_positive_30deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(30),
	}},
	{Name: "helix_negative_30deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(-30),
	}},
	{Name: "axis_swap_boundary_45deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(45),
	}},
	{Name: "quarter_turn_positive_90deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(90),
	}},
	{Name: "quarter_turn_negative_90deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(-90),
	}},
	{Name: "half_turn_bevel_virtual_tooth", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": deg(180),
	}},
	{Name: "few_ribs_3_samples", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 3, "angle": 0,
	}},
	{Name: "few_ribs_3_samples_rotated", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 3, "angle": deg(-30),
	}},
	{Name: "embedded_by_high_tooth_count", Params: map[string]float64{
		"module": 1, "toothNumber": 60, "pressureAngle": deg(20), "involuteSteps": 15, "angle": 0,
	}},
	{Name: "embedded_by_large_pressure_angle", Params: map[string]float64{
		"module": 2, "toothNumber": 30, "pressureAngle": deg(25), "involuteSteps": 15, "angle": 0,
	}},
	{Name: "anchor_off_plane_origin", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "involuteSteps": 15, "angle": 0,
		"anchorX": 25, "anchorY": -18,
	}},
}

// stepGearProfileSketch draws the whole Gear Profile sketch — the four circles,
// the involute tooth, the ribs, the spine and its angular pin, the flank-to-root
// lines, and the anchoring that slides the drawing onto the user's anchor — and
// then checks the two regions it is contracted to close.
//
// It is one step because it is one entry in the Fusion timeline. The gate on it
// is proofkit's, which asks for more than DOF 0: no conflicting or redundant
// constraint, valid profiles, a system that is not near-singular, and no
// discrete ambiguity — so a scheme that reaches DOF 0 but still admits the
// mirrored or half-turned answer fails here rather than in Fusion.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := readGear(p)
	d := g.Dims

	proofkit.Step(t, "step 3: the local origin and the four circles centred on it")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "Tools-sketch anchor re-projected")
	anchor.SetName("projected anchor")
	// Every seed below is placed relative to the local origin, and the local
	// origin is seeded ON the projected anchor.
	//
	// SUBSTITUTION. Fusion seeds the drawing around the sketch's own origin and
	// lets the step-5 coincidence drag it onto an anchor that may be far away;
	// the engine's solver does not converge across a translation that large, so
	// the seeds carry the offset instead. Seeds pin nothing — every dimension
	// below is a delta from the local origin and is unchanged by the shift — so
	// what the case still proves is that the drawing is located by the step-5
	// coincidence alone and by nothing tied to the plane origin.
	ox, oy := p["anchorX"], p["anchorY"]
	origin := s.CreatePoint(ox, oy)
	origin.SetName("local origin")

	// addByCenterRadius(localOrigin, radius) for each: the local origin is
	// passed directly so all four share it, and each carries a driving diameter
	// dimension. Only the root circle is solid; the other three are
	// construction, which is why the tip circle bounds no profile in step 9.
	root := s.CreateCircle(origin, d.Root)
	root.SetName("root circle")
	s.AddConstraint(sketch.NewDiameter(root, 2*d.Root))
	tip := s.CreateCircle(origin, d.Tip)
	tip.SetName("tip circle")
	tip.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(tip, 2*d.Tip))
	base := s.CreateCircle(origin, d.Base)
	base.SetName("base circle")
	base.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(base, 2*d.Base))
	pitch := s.CreateCircle(origin, d.Pitch)
	pitch.SetName("pitch circle")
	pitch.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(pitch, 2*d.Pitch))

	proofkit.Step(t, "step 4.1-4.5: the two involute flanks as fitted splines")
	// involute.Flanks carries step 4's exact math: sample from the base circle
	// outward, endpoint-inclusive; mirror across +X so the flank narrows toward
	// the tip; rotate so the pitch crossing lands at +pi/(2N); then apply the
	// requested angle to both flanks together, so the tooth is drawn already at
	// its final angular position rather than swung there by the dimension.
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, g.ToothNumber, g.Steps, g.Angle)
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(ox+left[i].X, oy+left[i].Y)
		rp[i] = s.CreatePoint(ox+right[i].X, oy+right[i].Y)
	}
	leftFlank, err := s.CreateFitSpline(lp...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(rp...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("right flank")
	last := len(lp) - 1

	proofkit.Step(t, "step 4.6: the tooth-top point and the arc centred on the local origin")
	// The tooth-top point sits at the tip radius, rotated by angle, and is
	// constrained coincident to the tip circle. The arc shares the local origin
	// as its centre and the two flanks' end points as its ends, and carries no
	// diameter dimension: a free centre plus a diameter would leave an equally
	// valid inward-bulging answer, which the shared centre removes.
	top := s.CreatePoint(ox+d.Tip*math.Cos(g.Angle), oy+d.Tip*math.Sin(g.Angle))
	top.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))
	topArc := s.CreateArc(origin, rp[last], lp[last])
	topArc.SetName("tooth top arc")

	proofkit.Step(t, "step 4.7: spine, +X reference line, and the confirming angular dimension")
	spine := s.CreateLine(origin, top)
	spine.SetName("spine")
	spine.SetConstruction(true)
	// The reference end is pinned by two axis dimensions from the local origin
	// rather than by putting it on the tip circle: a point on a circle has two
	// answers, and the extreme of the circle is where the numbers go unstable.
	refEnd := s.CreatePoint(ox+d.Tip, oy)
	refEnd.SetName("+X reference end")
	s.AddConstraint(sketch.NewHorizontalDistance(origin, refEnd, d.Tip))
	s.AddConstraint(sketch.NewVerticalDistance(origin, refEnd, 0))
	reference := s.CreateLine(origin, refEnd)
	reference.SetName("+X reference")
	reference.SetConstruction(true)
	// The angular dimension runs FROM the reference TO the spine and exists for
	// every angle including zero, because it is what says which way the spine
	// points. A plain horizontal on the spine would fix its direction and not
	// its sense, and the tooth would be free to settle half a turn around.
	//
	// SUBSTITUTION: the engine reads a bare angle target in the sketch's default
	// angle unit, which is degrees, where Fusion's angular dimension parameter
	// holds radians. The value below is the same angle in the unit each side
	// reads; nothing else about the constraint differs.
	s.AddConstraint(sketch.NewAngle(reference, spine, g.Angle*180/math.Pi))

	proofkit.Step(t, "step 4.8: one rib per involute sample, chained along the spine")
	ca, sa := math.Cos(g.Angle), math.Sin(g.Angle)
	// The rib takes the axis ACROSS the spine and the chain the one ALONG it.
	// An aligned dimension would give only a length, which the left and right
	// flanks satisfy equally well swapped over; the axis dimension's direction,
	// captured from the seed, is what forbids the swap.
	acrossIsVertical := math.Abs(ca) >= math.Abs(sa)
	previous, previousAlong := origin, 0.0
	for i := range lp {
		rib := s.CreateLine(lp[i], rp[i]) // shares both fit points
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}
		// The midpoint is seeded at the foot of the left fit point ON the spine,
		// not at the rib's own 2-D midpoint and not at (fitX, 0) for a rotated
		// tooth.
		along := left[i].X*ca + left[i].Y*sa
		mid := s.CreatePoint(ox+along*ca, oy+along*sa)
		// Order is load-bearing: on the spine first, then the rib's midpoint,
		// then perpendicular — and no perpendicular on the last rib, which the
		// tooth-top arc already holds at equal radius either side of the spine.
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		// The chain starts at the local origin, so the ribs cannot slide along
		// the spine as a unit.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, (along-previousAlong)*ca))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, (along-previousAlong)*sa))
		}
		previous, previousAlong = mid, along
	}

	proofkit.Step(t, "step 4.9: close the tooth at the root (embedded = %v)", d.Embedded())
	if !d.Embedded() {
		for _, side := range []struct {
			name  string
			start *sketch.Point
			at    involute.Pt
		}{
			{"left", lp[0], left[0]},
			{"right", rp[0], right[0]},
		} {
			foot := rootFoot(d.Root, side.at)
			end := s.CreatePoint(ox+foot.X, oy+foot.Y)
			end.SetName(side.name + " flank-to-root end")
			// The line shares the flank spline's start point; the root end is
			// placed by exactly two axis dimensions from the local origin and
			// nothing else. "Root end on the root circle" plus "origin on the
			// line" would be satisfied by two points — the far side of the
			// circle as well — and the stub would become a line across the gear.
			s.CreateLine(end, side.start)
			s.AddConstraint(sketch.NewHorizontalDistance(origin, end, foot.X))
			s.AddConstraint(sketch.NewVerticalDistance(origin, end, foot.Y))
		}
	}

	proofkit.Step(t, "step 5: constrain the local origin onto the projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	checkGearProfileRegions(t, s, g, root)
}

// checkGearProfileRegions checks the contract the two extrude steps depend on:
// the sketch closes exactly two regions, with the curve counts step 7 and step 9
// search by, and the root circle is cut in exactly two places, which is what
// makes Fusion's counts what they are.
//
// The sketch is solved here so the regions can be read; proofkit solves again
// under its own gate, which is idempotent.
func checkGearProfileRegions(t testing.TB, s *sketch.Sketch, g gear, root *sketch.Circle) {
	t.Helper()
	proofkit.Step(t, "the sketch must close the tooth region and the disc region")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before reading regions: %v", err)
	}
	if got := len(s.Profiles()); got != 2 {
		t.Fatalf("the Gear Profile sketch closes %d regions, want exactly 2 (the tooth and the disc)", got)
	}

	// Step 7: find_profile_by_curve_counts(sketch, nurbs=2, arcs=2,
	// lines=0 if toothProfileIsEmbedded else 2). The engine's reading of
	// Fusion's second arc is the whole root circle — see curveCounts.
	want := curveCounts{Splines: 2, Arcs: 1, Circles: 1, Lines: 2}
	if g.Dims.Embedded() {
		want.Lines = 0
	}
	tooth := findProfileByCurveCounts(t, s, want)

	// Step 9: find_profile_by_curve_counts(sketch, arcs=2) — the solid disc
	// inside the root circle, with no line and no spline on its boundary. It is
	// not an annulus: the tip circle is construction geometry and bounds nothing.
	disc := findProfileByCurveCounts(t, s, curveCounts{Circles: 1})
	if wantArea := math.Pi * g.Dims.Root * g.Dims.Root; math.Abs(disc.Area-wantArea) > 1e-6*wantArea {
		t.Errorf("the disc region has area %g mm^2, want pi*RootCircleRadius^2 = %g", disc.Area, wantArea)
	}
	if tooth.Area <= 0 || tooth.Area >= disc.Area {
		t.Errorf("the tooth region has area %g mm^2, which is not a single tooth beside a disc of %g", tooth.Area, disc.Area)
	}
	if got := rootCircleCuts(tooth, root); got != 2 {
		t.Errorf("the tooth cuts the root circle in %d places, want 2 — Fusion's two root arcs come from that split", got)
	}
}

// ---------------------------------------------------------------------------
// S16 — Bore Profile sketch
// ---------------------------------------------------------------------------

// boreProfileCases sweep the bore across the range the dialog allows above
// zero — a bore diameter of zero or less means no Bore Profile sketch is drawn
// at all, which is step 12's early return and carries no geometry — and move
// the anchor off the plane origin, because the bore has to follow it.
var boreProfileCases = []proofkit.Case{
	{Name: "small_bore_at_origin", Params: map[string]float64{
		"boreDiameter": 0.5, "anchorX": 0, "anchorY": 0,
	}},
	{Name: "default_bore_off_origin", Params: map[string]float64{
		"boreDiameter": 5, "anchorX": 31, "anchorY": -17.5,
	}},
	{Name: "large_bore_off_origin", Params: map[string]float64{
		"boreDiameter": 120, "anchorX": -220, "anchorY": 90,
	}},
}

// stepBoreProfileSketch proves the Bore Profile sketch of step 12, whose whole
// difficulty is a point nothing uses: the tooth generator's constructor always
// adds its local-origin sketch point at (0, 0), so this sketch carries one stray
// point that has to be grounded on the projected anchor like every other
// sketch's local origin, or the sketch never reaches full constraint.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := readGear(p)
	proofkit.Step(t, "step 12: project the Tools-sketch anchor into the Bore Profile sketch")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "Tools-sketch anchor re-projected")
	anchor.SetName("projected anchor")

	proofkit.Step(t, "the tooth generator's constructor adds its local origin at (0, 0)")
	local := s.CreatePoint(0, 0)
	local.SetName("tooth generator local origin")

	proofkit.Step(t, "drawBore draws the bore circle on the projection with a driving diameter dimension")
	// The projection is passed directly as the centre, so the circle shares it.
	bore := s.CreateCircle(anchor, g.BoreDiameter/2)
	bore.SetName("bore circle")
	s.AddConstraint(sketch.NewDiameter(bore, g.BoreDiameter))

	proofkit.Step(t, "ground the stray local origin on that same projection")
	// Not on the sketch's own origin point: that pins the point to the plane
	// rather than to the gear, and constraining to the origin point has been
	// observed to fail the solver outright.
	s.AddConstraint(sketch.NewCoincident(local, anchor))

	if got := bore.R(); math.Abs(got-g.BoreDiameter/2) > tol {
		t.Errorf("the bore circle solved to radius %g mm, want BoreDiameter/2 = %g", got, g.BoreDiameter/2)
	}
}
