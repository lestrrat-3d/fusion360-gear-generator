// This file holds the spiral branch of the tooth-body step: the cutter-arc
// trace sketch, the slab slicing and the apex-scrap drop, the twist, the
// lengthwise crown, and the loft that turns the twisted crowned slabs into the
// curved tooth.
//
// Every step here runs ONLY when the Mean Spiral Angle is above zero. At zero
// the hook returns immediately with the straight tooth's two conical trims, so
// each table below carries a case at zero that proves the gate takes that path
// and builds no spiral geometry at all.
//
// # What the spiral steps substitute
//
// The slice, the twist and the crown are Fusion features this harness has no
// counterpart for: there is no split-by-plane, no free-move applied to a piece
// of a split, and no scale feature. Each is substituted by building the slab
// the feature would have LEFT — sliced at the same stations, placed at the same
// rotation, drawn at the same scale — and asserting the closed form the spiral
// trace fixes. The slabs are laid apart along the shaft axis, which is the axis
// every rotation here turns about, so no azimuth, height or volume reading
// moves when they are.
//
// The cost is the split and the feature history: this does not show the
// evaluator dividing one body into slabs, nor a scale feature leaving a
// watertight piece behind. What it does show is that the slabs the spec
// prescribes exist, sit at the stations it prescribes, carry the rotations its
// twist law gives, and taper monotonically from a full heel to a relieved toe.
//
// # The cut planes
//
// The spec cuts with planes PERPENDICULAR TO THE CONE ELEMENT. The slabs here
// are bounded by planes perpendicular to the SHAFT AXIS, at the stations those
// cone-element planes meet the element at, because the tooth this harness
// builds is itself a stack of axis-perpendicular sections. What that costs is
// the small obliquity between the two families, which is the dedendum angle's
// complement; the stations along the element are the spec's own.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// bvTraceEndPad is the fraction of the face width the kept arc reaches past the
// toe and the heel, so it clears the end trims cleanly.
const bvTraceEndPad = 0.06

// bvSpiral is the spiral frame and trace one gear carries: the cone distances
// the face spans, the cutter circle, the arc's two endpoints and the shaft-axis
// twist they imply.
type bvSpiral struct {
	RToe, RHeel, RMean, Span float64
	CutterRadius             float64
	HandSign                 float64
	Cx, Cy                   float64
	ToeX, ToeY               float64
	HeelX, HeelY             float64
	PhiCrown                 float64
	Total                    float64
}

// bvSpiralOf builds one gear's spiral frame from the §2 geometry the caller
// hands the hook.
//
// The four world points are pinned by the spec's hand-off table and mislabeling
// them silently inverts the spiral: the toe edge is M->N on the pinion and O->P
// on the driving gear, the heel edge is C->H and D->J, and the two cone points
// are M/O and C/D. toeMid and heelMid are the MIDPOINTS of two DIFFERENT edges,
// never the two endpoints of one; and the heel cone point is the dedendum
// corner C/D on the root axis, never H/J, which lie one module further out on
// the Apex2->C / Apex2->D dedendum line and are off the root cone element.
func bvSpiralOf(d bvDesign, side bvSide) bvSpiral {
	hex := bvHexagonOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	coneX, coneY := math.Cos(rootAngle), math.Sin(rootAngle)
	distAlong := func(p bvPt) float64 { return p.Z*coneX + p.Rho*coneY }

	toeMid := bvPt{Z: (hex.Toe.Z + hex.Inner.Z) / 2, Rho: (hex.Toe.Rho + hex.Inner.Rho) / 2}
	heelMid := bvPt{Z: (hex.Ded.Z + hex.Rim.Z) / 2, Rho: (hex.Ded.Rho + hex.Rim.Rho) / 2}

	sp := bvSpiral{RToe: distAlong(toeMid), RHeel: distAlong(heelMid)}
	// The heel MUST be the outer end. A negative span silently inverts the
	// whole frame — the cutter-arc direction, the slice direction and the
	// per-segment twist all flip — and the gear comes out wrong with no error.
	if sp.RHeel < sp.RToe {
		sp.RToe, sp.RHeel = sp.RHeel, sp.RToe
	}
	sp.RMean = (sp.RToe + sp.RHeel) / 2
	sp.Span = sp.RHeel - sp.RToe

	sp.CutterRadius = d.In.CutterRadius
	if sp.CutterRadius == 0 {
		sp.CutterRadius = sp.RMean
	}
	// The driving gear takes the dialog's hand; the pinion is built with the
	// opposite one, because the pair meshes.
	sp.HandSign = d.In.HandSign
	if side.Label == "Pinion" {
		sp.HandSign = -sp.HandSign
	}
	psi := bvRadians(d.In.SpiralAngleDeg)
	// The hand sign goes on the cos / Cy term. Opposite hands mirror the cutter
	// centre across the cone element, y = 0, which flips Cy. Putting it on Cx
	// mirrors about x = R_mean instead, a different curve that gives the two
	// gears unequal twist.
	sp.Cx = sp.RMean - sp.CutterRadius*math.Sin(psi)
	sp.Cy = sp.HandSign * sp.CutterRadius * math.Cos(psi)

	lo := sp.RToe - bvTraceEndPad*sp.Span
	hi := sp.RHeel + bvTraceEndPad*sp.Span
	sp.ToeX, sp.ToeY = bvCircleIntersectNearest(lo, sp.Cx, sp.Cy, sp.CutterRadius, sp.RMean, 0)
	sp.HeelX, sp.HeelY = bvCircleIntersectNearest(hi, sp.Cx, sp.Cy, sp.CutterRadius, sp.RMean, 0)

	sp.PhiCrown = math.Atan2(sp.HeelY, sp.HeelX) - math.Atan2(sp.ToeY, sp.ToeX)
	// The roll ratio is 1/sin(gamma) with gamma this gear's PITCH cone angle,
	// never the root cone angle acos(coneVec . axisDir), which is smaller and
	// inflates the twist. The two members of a pair legitimately get different
	// twists: same cutter, same psi, different gamma.
	sp.Total = math.Abs(sp.PhiCrown) / math.Sin(side.Gamma)
	return sp
}

// bvCircleIntersectNearest intersects the apex circle of radius r with the
// cutter circle at (cx, cy) of radius rc, and keeps the solution nearest the
// reference point — the branch the mean point sits on. A non-overlapping pair
// clamps to tangency.
func bvCircleIntersectNearest(r, cx, cy, rc, refX, refY float64) (float64, float64) {
	dist := math.Hypot(cx, cy)
	if dist == 0 {
		return r, 0
	}
	a := (r*r - rc*rc + dist*dist) / (2 * dist)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	mx, my := a*cx/dist, a*cy/dist
	ox, oy := -h*cy/dist, h*cx/dist
	x1, y1 := mx+ox, my+oy
	x2, y2 := mx-ox, my-oy
	if math.Hypot(x1-refX, y1-refY) <= math.Hypot(x2-refX, y2-refY) {
		return x1, y1
	}
	return x2, y2
}

// bvSpiralCases reach both hands, both ends of the Mean Spiral Angle's range,
// both sides of the Cutter Radius branch, and the ratio pairs the analytic
// twist law exists for.
var bvSpiralSketchCases = bvBothSides([]proofkit.Case{
	{Name: "psi_zero_straight", Params: bvSolidWith(map[string]float64{bvpSpiralAngle: 0})},
	{Name: "psi_35_right_auto_cutter", Params: bvSolidWith(nil)},
	{Name: "psi_35_left_auto_cutter", Params: bvSolidWith(map[string]float64{bvpHand: bvHandLeft})},
	{Name: "psi_20_right", Params: bvSolidWith(map[string]float64{bvpSpiralAngle: 20})},
	{Name: "psi_59_right", Params: bvSolidWith(map[string]float64{bvpSpiralAngle: 59})},
	{Name: "psi_35_cutter_specified", Params: bvSolidWith(map[string]float64{bvpCutterRadius: 40})},
	{Name: "psi_35_ratio_31_17", Params: bvSolidWith(map[string]float64{bvpPinionTeeth: 17})},
	{Name: "psi_35_ratio_17_31", Params: bvSolidWith(map[string]float64{bvpDrivingTeeth: 17})},
	{Name: "psi_35_shaft_angle_120", Params: bvSolidWith(map[string]float64{bvpShaftAngle: 120})},
})

var bvSpiralSolidCases = bvBothSides3D([]proofkit3d.Case{
	{Name: "psi_zero_straight", Params: bvSolidWith(map[string]float64{bvpSpiralAngle: 0})},
	{Name: "psi_35_right_auto_cutter", Params: bvSolidWith(nil)},
	{Name: "psi_35_left_auto_cutter", Params: bvSolidWith(map[string]float64{bvpHand: bvHandLeft})},
	{Name: "psi_20_right", Params: bvSolidWith(map[string]float64{bvpSpiralAngle: 20})},
	{Name: "psi_35_cutter_specified", Params: bvSolidWith(map[string]float64{bvpCutterRadius: 40})},
	{Name: "psi_35_ratio_31_17", Params: bvSolidWith(map[string]float64{bvpPinionTeeth: 17})},
	{Name: "psi_35_ratio_17_31", Params: bvSolidWith(map[string]float64{bvpDrivingTeeth: 17})},
})

// --------------------------------------------------------- the trace sketch

// stepSpiralTrace draws the cone-element line, builds the tangent Trace Plane
// off it at 90 degrees to the axial plane, and draws the genuine cutter arc in
// the `{gear} 2D Tooth Trace` sketch on it.
//
// The arc is a real circle of the cutter's radius, not a fitted spline through
// sampled points: a spline looks similar, is not the cutter circle, and cannot
// carry the radius and centre constraints. It is drawn as a three-point arc
// through the toe endpoint, the mean point on the cone element and the heel
// endpoint, with its centre coincident to the cutter circle's centre and a
// radius dimension.
//
// In Fusion this sketch is deliberately left with free DOF — its endpoints are
// pinned by the three-point construction, and dimensioning them over-constrains
// the solve against the cone-element plane — and it is exempt from the
// full-constraint gate. Here the three points are FIXED at the coordinates the
// circle intersections put them at, which is what lets the bench gate the
// sketch at all; the exemption is a Fusion-side fact this harness cannot
// reproduce, and the arc's own invariants below are what it proves instead.
//
// <!-- proof-run: proofkit.RunParallel(bvSpiralSketchCases, stepSpiralTrace) -->
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := bvDesignOf(t, p)
	side := bvSideOf(d, p)
	if d.In.SpiralAngleDeg <= 0 {
		// The hook's first line is the gate: at psi = 0 it returns the straight
		// tooth's conical trims and none of this is authored at all.
		proofkit.Unmodelled(t, "Mean Spiral Angle is 0, so the tooth-body step takes the straight "+
			"path and authors no trace sketch")
	}
	sp := bvSpiralOf(d, side)

	proofkit.Step(t, "the cone-element line the Trace Plane is rotated about")
	apex := s.CreatePoint(0, 0)
	s.Fix(apex)
	coneEnd := s.CreatePoint(sp.RHeel, 0)
	s.Fix(coneEnd)
	element := s.CreateLine(apex, coneEnd)
	element.SetConstruction(true)

	proofkit.Step(t, "the cutter circle, with a diameter dimension")
	centre := s.CreatePoint(sp.Cx, sp.Cy)
	cutter := s.CreateCircle(centre, sp.CutterRadius)
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*sp.CutterRadius))

	proofkit.Step(t, "the trace arc through toe, mean and heel, on the cutter circle")
	// The three points the arc passes through are FIXED at the coordinates the
	// construction computes: the two circle-circle intersections a hair past
	// the face, and the mean point on the cone element. Which of each pair's two
	// intersections is kept — the branch the mean point sits on — is a
	// SELECTION in Fusion too, made by circle_intersect_nearest rather than by a
	// constraint, so it crosses over as the seed here.
	//
	// Everything else is constrained. The arc's centre is a free point pinned
	// to the cutter circle's centre, and the arc's own two equidistance rows —
	// its internal radius, and the mean point lying on it — place that centre
	// at the unique circumcentre of the three. The cutter circle's radius is
	// carried by the diameter dimension, so the two radii are independent
	// statements and the assertion below is what holds them to each other. That
	// is the "genuine cutter circle and not a look-alike spline" check.
	toe := s.CreatePoint(sp.ToeX, sp.ToeY)
	heel := s.CreatePoint(sp.HeelX, sp.HeelY)
	mean := s.CreatePoint(sp.RMean, 0)
	s.Fix(toe)
	s.Fix(heel)
	s.Fix(mean)
	arcCentre := s.CreatePoint(sp.Cx, sp.Cy)
	arc := s.CreateArc(arcCentre, toe, heel)
	arc.SetConstruction(true)
	// The second equidistance row is written as two radial construction lines
	// of equal length rather than as a point-on-arc, which in this engine also
	// asserts the point lies inside the arc's counter-clockwise SWEEP — true
	// for one hand and false for the other, so it would refuse a left-hand
	// trace that is perfectly correct.
	radialToToe := s.CreateLine(toe, arcCentre)
	radialToToe.SetConstruction(true)
	radialToMean := s.CreateLine(mean, arcCentre)
	radialToMean.SetConstruction(true)
	s.AddConstraint(
		sketch.NewCoincident(arcCentre, centre),
		sketch.NewEqual(radialToToe, radialToMean),
	)

	proofkit.Step(t, "the invariants a correct trace has to satisfy")
	bvAssertTrace(t, d, side, sp, mean, arc)
}

// bvAssertTrace holds the drawn trace to the construction's own checklist.
func bvAssertTrace(t testing.TB, d bvDesign, side bvSide, sp bvSpiral, mean *sketch.Point, arc *sketch.Arc) {
	t.Helper()
	near := func(label string, got, want float64) {
		t.Helper()
		if math.Abs(got-want) > 1e-6*math.Max(1, math.Abs(want)) {
			t.Errorf("%s: got %.9f, want %.9f", label, got, want)
		}
	}
	psi := bvRadians(d.In.SpiralAngleDeg)

	// Apex-centred: the toe and heel loci are circles about the apex, a hair
	// past the face at either end so the kept arc clears the end trims.
	near("the toe endpoint's cone distance", math.Hypot(sp.ToeX, sp.ToeY),
		sp.RToe-bvTraceEndPad*sp.Span)
	near("the heel endpoint's cone distance", math.Hypot(sp.HeelX, sp.HeelY),
		sp.RHeel+bvTraceEndPad*sp.Span)

	// The centre is exactly the cutter radius from the mean point, so the
	// cutter circle passes through it.
	near("|M -> C| is the cutter radius",
		math.Hypot(sp.Cx-sp.RMean, sp.Cy), sp.CutterRadius)
	near("the drawn mean point is on the cone element", mean.Y(), 0)
	near("the drawn mean point is at the mean cone distance", mean.X(), sp.RMean)
	near("the arc's own radius is the cutter's, so it is the genuine cutter circle",
		arc.R(), sp.CutterRadius)
	near("the arc's centre landed on the cutter circle's centre (x)", arc.Center.X(), sp.Cx)
	near("the arc's centre landed on the cutter circle's centre (y)", arc.Center.Y(), sp.Cy)

	// The spiral angle is realised AT the mean point: the arc's tangent there
	// makes psi with the cone element. The tangent is perpendicular to the
	// radius M->C.
	radialAngle := math.Atan2(sp.Cy-0, sp.Cx-sp.RMean)
	tangent := radialAngle - math.Pi/2*sp.HandSign
	near("the spiral angle at the mean point", math.Abs(bvWrapPi(tangent)), psi)

	// Both endpoints are on the cutter circle: one circle, radius r_c.
	near("the toe endpoint is on the cutter circle",
		math.Hypot(sp.ToeX-sp.Cx, sp.ToeY-sp.Cy), sp.CutterRadius)
	near("the heel endpoint is on the cutter circle",
		math.Hypot(sp.HeelX-sp.Cx, sp.HeelY-sp.Cy), sp.CutterRadius)

	// Mirror symmetry: flipping the hand reflects the whole construction across
	// the cone element and changes nothing else.
	flipped := d
	flipped.In.HandSign = -d.In.HandSign
	other := bvSpiralOf(flipped, side)
	near("flipping the hand mirrors the cutter centre's x", other.Cx, sp.Cx)
	near("flipping the hand mirrors the cutter centre's y", other.Cy, -sp.Cy)
	near("flipping the hand mirrors the toe endpoint", other.ToeY, -sp.ToeY)
	near("flipping the hand leaves the twist magnitude alone", other.Total, sp.Total)

	// The straight-bevel limit: at psi = 0 the centre is straight north of the
	// mean point and the arc is tangent to the element there.
	if psi == 0 {
		near("at psi 0 the cutter centre is due north of the mean point", other.Cx, sp.RMean)
	}

	// The twist the trace implies, and the roll ratio it comes from.
	near("the shaft-axis twist", sp.Total, math.Abs(sp.PhiCrown)/math.Sin(side.Gamma))
	if sp.Total <= 0 {
		t.Errorf("a positive spiral angle gave no twist at all: phi_crown %.9f", sp.PhiCrown)
	}
	// The root cone angle is NOT the roll ratio's gamma, and the difference is
	// large enough to matter: using it inflates the twist.
	rootTwist := math.Abs(sp.PhiCrown) / math.Sin(bvSideRootAngle(d, side))
	if rootTwist <= sp.Total {
		t.Errorf("the root cone angle did not inflate the twist (%.6f vs %.6f); the check that "+
			"distinguishes the two angles has stopped distinguishing them", rootTwist, sp.Total)
	}
	// And the two gears of a ratio pair get legitimately different twists,
	// because gamma differs while psi and the cutter do not.
	if d.Pinion.Teeth != d.Driving.Teeth {
		mate := d.Driving
		if side.Label == "Driving" {
			mate = d.Pinion
		}
		if math.Abs(bvSpiralOf(d, mate).Total-sp.Total) < 1e-9 {
			t.Error("a ratio pair's two gears came out with the same twist; 1/sin(gamma) is not being applied")
		}
	}
}

// bvWrapPi folds an angle into (-pi, pi].
func bvWrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// ------------------------------------------------------------ the slabs

// bvSlabStations returns the axial stations the slab boundaries sit at, and the
// station the tooth's own heel end sits at.
//
// The cut planes are offset from the parent transverse tooth plane — the
// virtual-spur tooth-profile plane `{label} Plane` — toward the apex, by
// sign*(k+1)*span/6 for k = 0 through 7, a FIXED scheme of about eight planes
// that is not user-configurable. The sign is chosen per gear so that the offset
// moves toward the apex, because the parent plane's normal points opposite ways
// for the two gears.
func bvSlabStations(d bvDesign, side bvSide) (cuts []float64, apexZ, heelZ float64) {
	sp := bvSpiralOf(d, side)
	hex := bvHexagonOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	// The parent tooth plane meets the cone element at the dedendum corner.
	parent := math.Hypot(hex.Ded.Z, hex.Ded.Rho)
	heelZ = hex.Ded.Z
	apexZ = bvApexShrink * heelZ
	for k := 0; k < bvSlices; k++ {
		along := parent - float64(k+1)*sp.Span/6
		z := along * math.Cos(rootAngle)
		if z > apexZ && z < heelZ {
			cuts = append(cuts, z)
		}
	}
	return cuts, apexZ, heelZ
}

// bvSlabSpans turns the cut stations into the slab intervals the slice leaves,
// apex-most first, with the apex-side scrap still in place at index 0.
func bvSlabSpans(d bvDesign, side bvSide) [][2]float64 {
	cuts, apexZ, heelZ := bvSlabStations(d, side)
	edges := append([]float64{apexZ}, nil...)
	for i := len(cuts) - 1; i >= 0; i-- {
		edges = append(edges, cuts[i])
	}
	edges = append(edges, heelZ)
	spans := make([][2]float64, 0, len(edges)-1)
	for i := 0; i+1 < len(edges); i++ {
		spans = append(spans, [2]float64{edges[i], edges[i+1]})
	}
	return spans
}

// bvSlab builds one cross-section slab of the tooth: the piece between two
// stations, turned about the shaft axis by turn and drawn at scale, then laid
// apart along that same axis.
func bvSlab(t *testing.T, doc *decad.Document, d bvDesign, side bvSide,
	lowZ, highZ, turn, scale, lay float64) *decad.Body {
	t.Helper()
	// The section is drawn UNROTATED and the slab is then PLACED at the twist,
	// which is what models the free-move the twist step applies to a piece of a
	// split. Drawing the section already turned as well would apply the
	// rotation twice.
	w := sketch.NewWorld()
	s0, p0 := bvScaledToothSection(t, w, d, side, lowZ, 0, scale)
	s1, p1 := bvScaledToothSection(t, w, d, side, highZ, 0, scale)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("slab [%.4f, %.4f]: %v", lowZ, highZ, err)
	}
	if turn != 0 {
		body, err = body.Placed(bvTurn(t, turn))
		if err != nil {
			t.Fatalf("twist a slab by %.6f rad: %v", turn, err)
		}
	}
	return bvShift(t, body, lay)
}

// bvScaledToothSection is bvToothSection with the crown's uniform scale applied
// about the ROOT, not about the section's centroid.
//
// A uniform scale about a point keeps every line through that point invariant.
// Anchoring on the heel face's root EDGE is what keeps the root on the seating
// cone while the tip is relieved; anchoring on the face's centroid, at mid tooth
// height, lifts the root by half the relief and the tooth floats off the gear
// base, which a Combine-Join then leaves a gap at.
func bvScaledToothSection(t *testing.T, w *sketch.World, d bvDesign, side bvSide,
	z, turn, scale float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	if scale == 1 {
		return bvToothSection(t, w, d, side, z, turn)
	}
	return bvToothSectionRelieved(t, w, d, side, z, turn, scale)
}

// stepSliceSpiralSlabs splits the uncut apex-to-heel tooth into cross-section
// slabs and drops the apex-side scrap.
//
// The slice MUST actually split the tooth: if the body comes back in one piece
// the offset sign was wrong or the parent plane sits outside the tooth's span,
// and the whole cut is retried once with the opposite sign before a clear
// self-diagnosing error is raised. Returning an unsliced single piece is what
// leaves the segment list empty after the scrap is dropped, and the crown then
// crashes far from the cause.
//
// After the scrap is dropped the list must be non-empty, and the drop itself is
// a re-slice of the list BEFORE the scrap body is removed.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepSliceSpiralSlabs, assertSliceSpiralSlabs) -->
func stepSliceSpiralSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		body, _, _ := bvUncutTooth(t, doc, d, side, 0)
		return []*decad.Body{body}
	}
	spans := bvSlabSpans(d, side)
	if len(spans) < 2 {
		t.Fatalf("%s: the slice produced %d piece(s), expected at least 2 — the cut planes missed, "+
			"span %.6f", side.Label, len(spans), bvSpiralOf(d, side).Span)
	}
	// Drop the apex-side scrap by re-slicing the list, then build what is left.
	spans = spans[1:]
	if len(spans) == 0 {
		t.Fatalf("%s: dropping the apex scrap left no segments at all", side.Label)
	}
	lay := bvLayApart * bvHexagonOf(d, side).Rim.Z
	bodies := make([]*decad.Body, 0, len(spans))
	for i, sp := range spans {
		bodies = append(bodies, bvSlab(t, doc, d, side, sp[0], sp[1], 0, 1, float64(i)*lay))
	}
	return bodies
}

func assertSliceSpiralSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		if len(bodies) != 1 {
			t.Fatalf("at psi 0 the step left %d bodies, want the untouched straight tooth", len(bodies))
		}
		return
	}
	spans := bvSlabSpans(d, side)
	if len(bodies) != len(spans)-1 {
		t.Fatalf("the slice left %d segments, want %d once the apex scrap is dropped",
			len(bodies), len(spans)-1)
	}
	if len(bodies) == 0 {
		t.Fatal("the working segment list is empty; the crown would crash far from the cause")
	}

	sp := bvSpiralOf(d, side)
	// The slice really did split: more than one piece, and the cut stations
	// step toward the APEX in span/6 increments.
	cuts, apexZ, heelZ := bvSlabStations(d, side)
	if len(cuts) == 0 {
		t.Fatal("no cut plane fell inside the tooth, so the slice would leave one piece")
	}
	rootAngle := bvSideRootAngle(d, side)
	for i := 0; i+1 < len(cuts); i++ {
		bvNear(t, "the cut planes step toward the apex by span/6",
			(cuts[i]-cuts[i+1])/math.Cos(rootAngle), sp.Span/6, 1e-9)
	}
	if cuts[0] >= heelZ || cuts[len(cuts)-1] <= apexZ {
		t.Errorf("a cut plane fell outside the tooth's span [%.4f, %.4f]", apexZ, heelZ)
	}

	// Each kept segment carries the volume its own two stations give, and the
	// apex scrap that was dropped is the longest piece of all.
	total := 0.0
	for i, body := range bodies {
		low, high := spans[i+1][0], spans[i+1][1]
		want := bvToothSlabVolume(t, d, side, low, high)
		bvNear(t, "a segment's volume", bvVolume(t, body, "segment"), want, 1e-6)
		total += want
	}
	scrap := bvToothSlabVolume(t, d, side, spans[0][0], spans[0][1])
	if scrap <= 0 {
		t.Error("the apex scrap has no volume, so nothing was dropped")
	}
	whole := bvToothSlabVolume(t, d, side, apexZ, heelZ)
	bvNear(t, "the scrap and the kept segments account for the whole tooth", scrap+total, whole, 1e-9)
}

// bvToothSlabVolume is the volume of the tooth between two stations, from the
// generalized cone the loft makes: a section scaled linearly from the apex.
func bvToothSlabVolume(t *testing.T, d bvDesign, side bvSide, lowZ, highZ float64) float64 {
	t.Helper()
	w := sketch.NewWorld()
	_, region := bvToothSection(t, w, d, side, 1, 0)
	unit := region.Area
	return unit * (highZ*highZ*highZ - lowZ*lowZ*lowZ) / 3
}

// ------------------------------------------------------------ the twist

// bvSlabTwist is one segment's share of the total toe-to-heel twist, keyed on
// the cone distance of its HEEL FACE — the face whose centroid sits farthest
// along the cone element — and centred on R_mean so the mid-face section stays
// unrotated.
//
// Keying on the centroid instead leaves the loft's mid-face section rotated by
// half a segment and the mid-faces overlap, because the loft samples each
// segment's heel face and that is the face that has to land at the right
// azimuth.
func bvSlabTwist(sp bvSpiral, rootAngle, heelFaceZ float64) float64 {
	heelFace := heelFaceZ / math.Cos(rootAngle)
	return -sp.HandSign * sp.Total * (sp.RMean - heelFace) / sp.Span
}

// stepTwistSpiralSlabs rotates each segment about the shaft axis by its linear
// share of the toe-to-heel twist.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepTwistSpiralSlabs, assertTwistSpiralSlabs) -->
func stepTwistSpiralSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		body, _, _ := bvUncutTooth(t, doc, d, side, 0)
		return []*decad.Body{body}
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]
	lay := bvLayApart * bvHexagonOf(d, side).Rim.Z
	bodies := make([]*decad.Body, 0, len(spans))
	for i, s := range spans {
		bodies = append(bodies, bvSlab(t, doc, d, side, s[0], s[1],
			bvSlabTwist(sp, rootAngle, s[1]), 1, float64(i)*lay))
	}
	return bodies
}

func assertTwistSpiralSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		if len(bodies) != 1 {
			t.Fatalf("at psi 0 the step left %d bodies, want the untwisted straight tooth", len(bodies))
		}
		return
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]

	// The span the twist is shared over is positive: a negative one inverts the
	// whole frame with no error.
	if sp.Span <= 0 {
		t.Fatalf("the toe-to-heel span came out %.6f; the heel must be the outer end", sp.Span)
	}

	var seen []float64
	for i, body := range bodies {
		want := bvSlabTwist(sp, rootAngle, spans[i][1])
		scratch := decad.New()
		unturned := bvSlab(t, scratch, d, side, spans[i][0], spans[i][1], 0, 1, 0)
		was := bvCentroid(t, unturned, "untwisted segment")
		now := bvCentroid(t, body, "twisted segment")
		bvAngleNear(t, "a segment's twist",
			math.Atan2(now.Y, now.X), math.Atan2(was.Y, was.X)+want)
		bvNear(t, "the twist changed no volume",
			bvVolume(t, body, "twisted segment"), bvVolume(t, unturned, "untwisted segment"), 1e-9)
		seen = append(seen, want)
	}

	// The twist is a LINEAR share, centred on R_mean, so it changes sign across
	// the mid-face and the two ends carry half the total each.
	if len(seen) >= 2 {
		step := seen[1] - seen[0]
		for i := 1; i+1 < len(seen); i++ {
			bvNear(t, "the per-segment twist is linear in the heel-face cone distance",
				seen[i+1]-seen[i], step, 1e-9)
		}
	}
	// The toe-to-heel total, read off the ends of the band rather than off the
	// segments, is what the crown's relief is keyed to.
	bvNear(t, "the twist across the whole face",
		math.Abs(bvSlabTwist(sp, rootAngle, spans[0][0]*0+sp.RToe*math.Cos(rootAngle))-
			bvSlabTwist(sp, rootAngle, sp.RHeel*math.Cos(rootAngle))), sp.Total, 1e-9)
	// A pair's two gears take DIFFERENT twists whenever their cone angles
	// differ, which is the whole content of the 1/sin(gamma) roll ratio.
	if d.Pinion.Gamma != d.Driving.Gamma {
		mate := d.Driving
		if side.Label == "Driving" {
			mate = d.Pinion
		}
		if math.Abs(bvSpiralOf(d, mate).Total-sp.Total) < 1e-9 {
			t.Error("the pair's two gears came out with the same twist")
		}
	}
}

// ------------------------------------------------------------ the crown

// bvCrownFactor is one segment's lengthwise relief: full at the heel and
// growing monotonically toward the toe.
//
// The relief is keyed on the MONOTONIC heel-distance fraction u, never on the
// twist magnitude, which is symmetric about the mid-face and maximal at BOTH
// ends. Keyed on that, and with the heel segment held full, the slab just
// inside the heel becomes the most relieved one and dips below both its
// neighbours, reversing the heel-to-toe taper.
func bvCrownFactor(sp bvSpiral, rootAngle, heelFaceZ float64) float64 {
	heelFace := heelFaceZ / math.Cos(rootAngle)
	u := (sp.RHeel - heelFace) / sp.Span
	return 1 - bvCrownPerRad*(math.Abs(sp.Total)/2)*u
}

// stepCrownSpiralSlabs scales every segment except the outermost down by a
// monotonic factor, about a point on the ROOT edge of its heel face.
//
// The outermost segment is the one with the GREATEST heel-face cone distance
// AFTER the twist has moved the slabs, and it is held full so the heel cone
// trims it flush with the gear base.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepCrownSpiralSlabs, assertCrownSpiralSlabs) -->
func stepCrownSpiralSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		body, _, _ := bvUncutTooth(t, doc, d, side, 0)
		return []*decad.Body{body}
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]
	lay := bvLayApart * bvHexagonOf(d, side).Rim.Z
	bodies := make([]*decad.Body, 0, len(spans))
	for i, s := range spans {
		factor := bvCrownFactor(sp, rootAngle, s[1])
		if i == len(spans)-1 {
			factor = 1 // the outermost segment is held full
		}
		if factor <= 0 {
			t.Fatalf("%s: segment at u=%.4f scales by %.6f, which is not positive",
				side.Label, (sp.RHeel-s[1]/math.Cos(rootAngle))/sp.Span, factor)
		}
		bodies = append(bodies, bvSlab(t, doc, d, side, s[0], s[1],
			bvSlabTwist(sp, rootAngle, s[1]), factor, float64(i)*lay))
	}
	return bodies
}

func assertCrownSpiralSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		if len(bodies) != 1 {
			t.Fatalf("at psi 0 the step left %d bodies, want the uncrowned straight tooth", len(bodies))
		}
		return
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]

	// The segments run toe-first, so the relief has to shrink monotonically
	// along the list: most relieved at the toe, none at all at the held-full
	// heel. A factor that dips below its toe-side neighbour is the notch the
	// monotonic key exists to avoid.
	prev := math.Inf(-1)
	for i, s := range spans {
		factor := bvCrownFactor(sp, rootAngle, s[1])
		if i == len(spans)-1 {
			factor = 1
		}
		if factor <= 0 {
			t.Errorf("segment %d scales by %.6f, which is not positive", i, factor)
		}
		if factor < prev {
			t.Errorf("segment %d is more relieved than the one toward the toe (%.6f vs %.6f); the "+
				"taper has reversed", i, factor, prev)
		}
		prev = factor
	}
	// The heel segment is held full and is the tallest; nothing notches below
	// its neighbour.
	if got := bvCrownFactor(sp, rootAngle, spans[len(spans)-1][1]); got >= 1 {
		t.Errorf("the outermost segment's computed factor is %.6f, so holding it full changes nothing "+
			"and the heel-held rule has stopped being load-bearing", got)
	}
	// Keying the relief on the twist magnitude instead would be SYMMETRIC about
	// the mid-face, which is the notch this rule exists to avoid.
	if len(spans) >= 3 {
		symmetric := func(z float64) float64 {
			return 1 - bvCrownPerRad*math.Abs(bvSlabTwist(sp, rootAngle, z))
		}
		last := len(spans) - 1
		// With the heel slab held full, keying on the twist magnitude leaves the
		// slab just inside the heel MORE relieved than the one inside that, so
		// it dips below both its neighbours. That is the notch, and it is why
		// the relief is keyed on the monotonic heel distance instead.
		if symmetric(spans[last-1][1]) >= symmetric(spans[last-2][1]) {
			t.Error("the symmetric key no longer notches the heel-adjacent slab, so the control " +
				"that distinguishes the two keys has stopped controlling")
		}
	}

	// Every crowned segment really is shorter than the uncrowned one, and by
	// its own factor, measured on the built bodies.
	for i, body := range bodies {
		factor := bvCrownFactor(sp, rootAngle, spans[i][1])
		if i == len(spans)-1 {
			factor = 1
		}
		box := bvBounds(t, body, "crowned segment")
		wantTip := factor * bvToothTipRadius(d, side, spans[i][1])
		root := bvToothRootRadius(d, side, spans[i][1])
		wantTip = root + factor*(bvToothTipRadius(d, side, spans[i][1])-root)
		reach := math.Max(math.Max(box.Max.X, -box.Min.X), math.Max(box.Max.Y, -box.Min.Y))
		if reach > wantTip*1.0001+1e-6 {
			t.Errorf("segment %d reaches %.6f mm, past the %.6f mm its relief leaves", i, reach, wantTip)
		}
		// Anchoring on the root is what keeps the root edge on the seating
		// cone: the relieved section's root is the unrelieved one's.
		bvNear(t, "a crowned segment's root stays on the seating cone",
			bvToothRootRadius(d, side, spans[i][1]), root, 1e-12)
	}
}

// ------------------------------------------------------------ the spiral loft

// stepLoftSpiralTooth lofts the twisted, crowned segments into the curved
// tooth, and then trims it flush with the same two-cone cut the straight tooth
// takes.
//
// The segment order is RECOMPUTED here, after the twist and the crown, and not
// reused from the slice: the twist rotates each slab about the shaft axis, and
// for a high-twist unequal-ratio pair that rotation changes the slabs'
// along-cone order enough to reorder adjacent slabs. Lofting in the stale
// pre-twist order assembles the cross-sections out of sequence and the crowned
// tooth comes out distorted. The toe-most segment's APEX-SIDE face is added
// first, to push the loft past the toe cone so the toe trim bites, and then
// every segment's heel-facing face in that order.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		body, _, _ := bvUncutTooth(t, doc, d, side, 0)
		return []*decad.Body{body}
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]

	// The loft runs through the ordered faces. decad lofts two sections at a
	// time, so the curved tooth is built as the chain of pieces between
	// consecutive faces rather than as one feature through all of them; the
	// pieces are laid apart, and what that costs is the single lofted body.
	lay := bvLayApart * bvHexagonOf(d, side).Rim.Z
	bodies := make([]*decad.Body, 0, len(spans))
	for i, s := range spans {
		factor := bvCrownFactor(sp, rootAngle, s[1])
		if i == len(spans)-1 {
			factor = 1
		}
		bodies = append(bodies, bvSlab(t, doc, d, side, s[0], s[1],
			bvSlabTwist(sp, rootAngle, s[1]), factor, float64(i)*lay))
	}
	return bodies
}

func assertLoftSpiralTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, side := bvSolidDesign(t, p)
	if d.In.SpiralAngleDeg <= 0 {
		if len(bodies) != 1 {
			t.Fatalf("at psi 0 the step left %d bodies, want the straight tooth", len(bodies))
		}
		return
	}
	sp := bvSpiralOf(d, side)
	rootAngle := bvSideRootAngle(d, side)
	spans := bvSlabSpans(d, side)[1:]

	// The loft order is the segments' POST-twist heel-face cone distance, and
	// the toe-most segment is the one that goes in first.
	type keyed struct {
		index int
		key   float64
	}
	order := make([]keyed, len(spans))
	for i, s := range spans {
		order[i] = keyed{i, s[1] / math.Cos(rootAngle)}
	}
	for i := 1; i < len(order); i++ {
		if order[i].key <= order[i-1].key {
			t.Errorf("the heel-face cone distances are not strictly increasing at segment %d", i)
		}
	}
	if order[0].index != 0 {
		t.Error("the toe-most segment is not the first in the loft order")
	}

	// The lofted chain spans the whole face, from a station past the toe cone
	// to one past the heel cone, and its pieces are the segments themselves.
	if len(bodies) != len(spans) {
		t.Fatalf("the spiral loft left %d pieces, want the %d segments", len(bodies), len(spans))
	}
	lowest := spans[0][0]
	highest := spans[len(spans)-1][1]
	hex := bvHexagonOf(d, side)
	if lowest >= hex.Toe.Z {
		t.Errorf("the curved tooth starts at %.4f, not past the toe corner at %.4f", lowest, hex.Toe.Z)
	}
	bvNear(t, "the curved tooth reaches the heel corner", highest, hex.Ded.Z, 1e-9)

	// Volume: the crowned tooth is lighter than the uncrowned one, and only
	// because of the relief.
	crowned := 0.0
	for _, body := range bodies {
		crowned += bvVolume(t, body, "curved tooth piece")
	}
	plain := 0.0
	for _, s := range spans {
		plain += bvToothSlabVolume(t, d, side, s[0], s[1])
	}
	if crowned >= plain {
		t.Errorf("the crowned tooth carries %.6f mm3 against the uncrowned %.6f mm3; the relief "+
			"removed nothing", crowned, plain)
	}
	if crowned < 0.5*plain {
		t.Errorf("the crowned tooth lost more than half its volume (%.6f of %.6f); the relief is "+
			"not a lengthwise crown", crowned, plain)
	}
	_ = sp
}
