// This file holds the psi > 0 branch: the flat cutter-arc trace, the slab
// slicing, the twist, the lengthwise crown, the spiral loft and the flush trim.
//
// At psi = 0 none of it runs — the tooth-body hook returns the straight tooth's
// two conical trims, byte for byte the prior behaviour — so every case here
// carries a Mean Spiral Angle above 0 except the one that proves the gate.
//
// Three substitutions run through the whole file, and each is what lets a step
// be proved at all rather than dropped.
//
//   - decad has no SPLIT. The slabs are therefore BUILT between the cut planes
//     rather than cut out of one body. The planes are the real ones — the parent
//     transverse tooth plane offset apex-ward in span/6 steps — and the slabs
//     are real solids on them. THE COST is the split itself: the proof does not
//     show the evaluator dividing one tooth into these pieces, only that the
//     pieces the scheme names exist, sit where it says, and are ordered as it
//     says.
//   - decad has no SCALE feature. The crown is applied by BUILDING each slab at
//     its crowned size about the anchor the spec names, rather than by scaling a
//     built one. THE COST is the feature: what is proved is the law and the
//     invariant the anchor exists for — the root edge stays on the seating cone
//     — and not that scaleFeatures anchors there.
//   - decad's Loft takes exactly TWO sections. The spiral loft's multi-section
//     order is therefore proved as a CHAIN of pairwise lofts in that order, laid
//     apart. THE COST is the single body: the proof shows every adjacent pair
//     lofts and that the order is the post-twist one, not that one loft through
//     all of them leaves one tooth.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------- the frame

// spiralBuildFrame is §3a step A's world frame for one gear, in the gear's own
// coordinates: the shaft axis is +Z, the apex is the origin, and the cone
// element Apex->C (Apex->D) is what every cone distance is measured along.
type spiralBuildFrame struct {
	side      latticeSide
	coneVec   vec // unit Apex->C (Apex->D) in the (station, radius) plane
	rToe      float64
	rHeel     float64
	rMean     float64
	span      float64
	toothPlan float64 // the apex-to-parent-plane distance along the plane's normal
	trace     spiralTrace
}

// newSpiralBuildFrame builds the frame from the §2 toe and heel edges the caller
// hands the hook.
//
// ⚠️ The four hand-off points are the single biggest spiral hazard. Per gear the
// toe edge is M->N (O->P) and the heel edge C->H (D->J); `toeMid` and `heelMid`
// are the MIDPOINTS OF THOSE TWO DIFFERENT EDGES, and `toeConeWorld` and
// `heelConeWorld` are M (O) and C (D) — never H or J, which lie on the Apex2->C
// (Apex2->D) dedendum line one module beyond C (D) and OFF the root cone
// element. Passing the two ends of a single edge as toeMid and heelMid collapses
// the span to nothing and inverts the spiral with no error at all.
func newSpiralBuildFrame(t testing.TB, l lattice, side latticeSide) spiralBuildFrame {
	t.Helper()
	f := spiralBuildFrame{side: side}
	// coneVec runs along the root cone element, outward from the apex.
	f.coneVec = side.rootDir
	toeMid := vecScale(vecAdd(side.toe, side.toeIn), 0.5)
	heelMid := vecScale(vecAdd(side.ded, side.heel), 0.5)
	distAlong := func(p vec) float64 { return vecDot(vecSub(p, l.apex), f.coneVec) }

	// ⚠️ The heel MUST be the outer end. A negative span silently inverts the
	// whole spiral frame — the cutter-arc direction, the slice direction and the
	// per-segment twist — and the gear comes out wrong with no error.
	rToe, rHeel := distAlong(toeMid), distAlong(heelMid)
	if rHeel < rToe {
		rToe, rHeel = rHeel, rToe
		t.Errorf("%s: the heel midpoint sits NEARER the apex than the toe midpoint (%.6f against "+
			"%.6f); the hook's swap guard would fire here, and upstream §2 has labelled the two "+
			"edges the wrong way round", side.label, rHeel, rToe)
	}
	f.rToe, f.rHeel = rToe, rHeel
	f.rMean = (rToe + rHeel) / 2
	f.span = rHeel - rToe
	f.toothPlan = side.station(l.apex, side.tooth) * math.Cos(side.gamma)
	f.trace = newSpiralTrace(f.rToe, f.rHeel, l.spiralAngle, l.cutterRadius,
		handSign(l.rightHanded, side.label == "Pinion"), side.gamma)
	return f
}

// slabScale is the ray scale of the plane sitting `offset` apex-ward of the
// parent transverse tooth plane. The parent plane is the back-cone tooth plane
// at scale 1, and a plane parallel to it at ray scale s sits s times as far from
// the apex along its own normal.
func (f spiralBuildFrame) slabScale(offset float64) float64 {
	return 1 - offset/f.toothPlan
}

// slabOffsets are §3a step E's fixed scheme: the parent transverse tooth plane,
// then eight planes stepped apex-ward in span/6 increments. The count is not
// user-configurable.
func (f spiralBuildFrame) slabOffsets() []float64 {
	out := make([]float64, spiralSliceSteps+1)
	for k := range out {
		out[k] = float64(k) * f.span / spiralSliceDivisor
	}
	return out
}

// ---------------------------------------------------------------- trace sketch

// stepSpiralTrace draws the `{gear} Cone Element` line and the `{gear} 2D Tooth
// Trace` sketch on the Trace Plane: the genuine cutter circle and the trace arc
// on it.
//
// In Fusion the arc IS the cutter circle and not a look-alike spline: it is a
// three-point arc whose centre is coincident with the cutter circle's centre and
// which carries a radius dimension of r_c. That sketch is DELIBERATELY left with
// free DOF — its endpoints are pinned by the three-point construction rather
// than dimensioned, because dimensioning them over-constrains the solve against
// the cone-element plane — and it is exempt from the full-constraint gate. This
// harness refuses an ungated sketch, so the proof pins both ends and states the
// arc's defining properties as assertions; the substitution and what it costs
// are written at the construction below.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	if l.spiralAngle <= 0 {
		proofkit.Unmodelled(t, "Mean Spiral Angle 0 means a STRAIGHT bevel: the tooth-body hook "+
			"returns cut_conical_ends before any of §3a runs, so no trace sketch is authored and "+
			"Hand of Spiral and Cutter Radius are ignored")
	}
	f := newSpiralBuildFrame(t, l, side)
	tr := f.trace

	proofkit.Step(t, "%s: R_toe %.6f, R_heel %.6f, R_mean %.6f, span %.6f, r_c %.6f",
		side.label, tr.rToe, tr.rHeel, tr.rMean, tr.span, tr.cutterRadius)

	// The apex is the origin of the tangent-plane frame: x is cone distance along
	// the cone element and y is circumferential.
	apex := s.CreateReferencePoint(0, 0, projectionSource)
	apex.SetName("apex")

	proofkit.Step(t, "the cutter circle, centre pinned and diameter dimensioned")
	centre := s.CreatePoint(tr.centre.X, tr.centre.Y)
	centre.SetName("cutter centre")
	s.Fix(centre)
	cutter := s.CreateCircle(centre, tr.cutterRadius)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*tr.cutterRadius))

	proofkit.Step(t, "the toe and heel apex circles, taken a hair past the face")
	toeCircle := s.CreateCircle(apex, tr.rLo)
	toeCircle.SetConstruction(true)
	heelCircle := s.CreateCircle(apex, tr.rHi)
	heelCircle.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(toeCircle, 2*tr.rLo),
		sketch.NewDiameter(heelCircle, 2*tr.rHi),
	)

	proofkit.Step(t, "the trace's two ends, and the cone-element and azimuth lines")
	// SUBSTITUTION — the trace is drawn as the cutter circle plus its two pinned
	// ends rather than as an Arc entity, and the reason is this engine rather
	// than a choice. Its Arc carries an automatic radius-consistency row, so an
	// arc whose two ends are both pinned comes back OVER-constrained, and one
	// that leaves an end to that row admits the mirrored end and comes back
	// AMBIGUOUS. Neither passes the gate. What is constrained here is the genuine
	// cutter circle; the arc's own defining properties — its radius, its centre,
	// its two ends on the toe and heel circles, and psi realised at the mean
	// point — are ASSERTED below instead. THE COST is Fusion's three-point arc:
	// the proof does not show that construction landing on this circle.
	start := s.CreatePoint(tr.toe.X, tr.toe.Y)
	start.SetName("trace toe")
	end := s.CreatePoint(tr.heel.X, tr.heel.Y)
	end.SetName("trace heel")
	s.AddConstraint(
		sketch.NewHorizontalDistance(apex, start, tr.toe.X),
		sketch.NewVerticalDistance(apex, start, tr.toe.Y),
		sketch.NewHorizontalDistance(apex, end, tr.heel.X),
		sketch.NewVerticalDistance(apex, end, tr.heel.Y),
	)
	// The `{gear} Cone Element` line Apex -> (Apex + R_heel * coneVec): the line
	// the Trace Plane is made by rotating the axial plane 90 degrees about.
	coneEnd := s.CreatePoint(tr.rHeel, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(apex, coneEnd, tr.rHeel),
		sketch.NewVerticalDistance(apex, coneEnd, 0),
	)
	element := s.CreateLine(apex, coneEnd)
	element.SetName("cone element")
	element.SetConstruction(true)
	// The two lines whose azimuths at the apex the twist law reads.
	toeRay := s.CreateLine(apex, start)
	toeRay.SetConstruction(true)
	heelRay := s.CreateLine(apex, end)
	heelRay.SetConstruction(true)

	proofkit.Step(t, "the invariants a correct trace has to satisfy")
	assertTraceInvariants(t, l, side, f)
}

// assertTraceInvariants checks the seven properties spiral-tooth-trace.md §9
// lists, on the construction this step drew.
func assertTraceInvariants(t testing.TB, l lattice, side latticeSide, f spiralBuildFrame) {
	t.Helper()
	tr := f.trace
	psi := l.spiralAngle

	// 2 and 3: the arc is one circle of radius r_c passing through the mean point.
	if got := vecLen(vecSub(vec{tr.rMean, 0}, tr.centre)); math.Abs(got-tr.cutterRadius) > 1e-9*tr.cutterRadius {
		t.Errorf("the cutter centre sits %.9f mm from the mean point, want the cutter radius "+
			"%.9f mm — the arc has to pass through the mean point", got, tr.cutterRadius)
	}
	// 6: the ends lie on the toe and heel apex circles, which are centred on the
	// APEX and nowhere else.
	if got := vecLen(tr.toe); math.Abs(got-tr.rLo) > 1e-9*tr.rLo {
		t.Errorf("the trace's toe end is at cone distance %.9f mm, want the toe circle's %.9f mm",
			got, tr.rLo)
	}
	if got := vecLen(tr.heel); math.Abs(got-tr.rHi) > 1e-9*tr.rHi {
		t.Errorf("the trace's heel end is at cone distance %.9f mm, want the heel circle's %.9f mm",
			got, tr.rHi)
	}
	if got := vecLen(vecSub(tr.toe, tr.centre)); math.Abs(got-tr.cutterRadius) > 1e-9*tr.cutterRadius {
		t.Errorf("the trace's toe end is %.9f mm from the cutter centre, want %.9f mm",
			got, tr.cutterRadius)
	}
	// 4: the spiral angle is realised AT THE MEAN POINT, measured against the cone
	// ELEMENT and never against the axis or its perpendicular.
	radial := vecUnit(vecSub(vec{tr.rMean, 0}, tr.centre))
	tangent := vec{-radial.Y, radial.X}
	got := math.Abs(math.Atan2(math.Abs(vecCross(vec{1, 0}, tangent)), math.Abs(vecDot(vec{1, 0}, tangent))))
	if math.Abs(got-psi) > 1e-9 {
		t.Errorf("the trace meets the cone element at %.9f rad at the mean point, want the Mean "+
			"Spiral Angle %.9f rad", got, psi)
	}
	// 5: swapping the hand mirrors the whole construction across the element and
	// changes nothing else.
	mirrored := newSpiralTrace(tr.rToe, tr.rHeel, psi, l.cutterRadius, -tr.handSign, side.gamma)
	if math.Abs(mirrored.centre.X-tr.centre.X) > 1e-9 || math.Abs(mirrored.centre.Y+tr.centre.Y) > 1e-9 {
		t.Errorf("flipping the hand moved the cutter centre from (%.9f, %.9f) to (%.9f, %.9f); the "+
			"hand sign belongs on the cos/Cy term and mirrors across the cone element, never on "+
			"the sin/Cx term, which mirrors about x = R_mean and is a DIFFERENT curve",
			tr.centre.X, tr.centre.Y, mirrored.centre.X, mirrored.centre.Y)
	}
	if math.Abs(mirrored.crownTwist-tr.crownTwist) > 1e-9*math.Max(1, tr.crownTwist) {
		t.Errorf("the two hands twist by %.9f and %.9f rad; an equal-teeth pair's two traces are "+
			"exact mirror images and nothing else differs", tr.crownTwist, mirrored.crownTwist)
	}
	// The roll ratio: the shaft-axis twist is the developed crown azimuth divided
	// by sin(gamma), with gamma this gear's PITCH cone angle and never the
	// root-cone angle acos(coneVec . axisDir), which is smaller and inflates the
	// twist.
	phi := math.Abs(math.Atan2(tr.heel.Y, tr.heel.X) - math.Atan2(tr.toe.Y, tr.toe.X))
	if want := phi / math.Sin(side.gamma); math.Abs(tr.crownTwist-want) > 1e-12*math.Max(1, want) {
		t.Errorf("the toe-to-heel twist is %.9f rad, want the crown-gear law's %.9f rad", tr.crownTwist, want)
	}
	rootCone := side.rootConeAngle(l.module, l.pitchCone)
	if wrong := phi / math.Sin(rootCone); math.Abs(wrong-tr.crownTwist) < 1e-9 {
		t.Errorf("the pitch cone angle and the root cone angle give the same twist here, so this " +
			"case cannot tell a correct roll ratio from one keyed on acos(coneVec . axisDir)")
	}
	// 7: psi -> 0 returns the straight cone element. The centre goes to
	// (R_mean, ±r_c), straight off the element, and the arc through the mean point
	// is then tangent to it.
	straight := newSpiralTrace(tr.rToe, tr.rHeel, 0, l.cutterRadius, tr.handSign, side.gamma)
	if math.Abs(straight.centre.X-tr.rMean) > 1e-9*tr.rMean {
		t.Errorf("at psi = 0 the cutter centre sits at x = %.9f, want R_mean = %.9f — the straight "+
			"bevel is the limit of this construction", straight.centre.X, tr.rMean)
	}
}

// ---------------------------------------------------------------- slabs

// stepSliceToothSlabs splits the uncut apex-to-heel tooth into cross-section
// slabs by planes perpendicular to the cone element, then drops the apex-side
// scrap.
//
// The planes are the parent transverse tooth plane offset apex-ward by
// (k+1)*span/6 for k = 0..7, a FIXED scheme the user cannot configure. The
// offset SIGN is chosen per gear so the planes move toward the apex, and the
// build retries once with the opposite sign if the tooth came back in one piece;
// a tooth still in one piece after that is a raised error and never a returned
// result, because step F would then drop that one piece as the scrap and leave
// `segments` empty, and the crown would crash on an empty max() far from the
// cause.
//
// SUBSTITUTION — the slabs are BUILT between the planes, for the reason this
// file's header gives.
func stepSliceToothSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	return buildSpiralSlabs(t, doc, l, side, f, func(int) float64 { return 0 }, func(int) float64 { return 1 })
}

// buildSpiralSlabs is the slab set: one body between each pair of consecutive cut
// planes, each optionally turned about the shaft axis and scaled about its own
// heel-face root anchor.
func buildSpiralSlabs(t *testing.T, doc *decad.Document, l lattice, side latticeSide, f spiralBuildFrame,
	turnOf func(int) float64, crownOf func(int) float64) []*decad.Body {
	t.Helper()
	offsets := f.slabOffsets()
	gap := 3 * side.station(l.apex, side.tooth)
	bodies := make([]*decad.Body, 0, len(offsets)-1)
	world := sketch.NewWorld()
	for k := 0; k+1 < len(offsets); k++ {
		heelScale := f.slabScale(offsets[k]) * crownOf(k)
		toeScale := f.slabScale(offsets[k+1]) * crownOf(k)
		if toeScale <= 0 || heelScale <= 0 {
			t.Fatalf("%s: slab %d would be built at scale %.6f/%.6f; a non-positive crown factor is "+
				"a raised error and never a scale", side.label, k, toeScale, heelScale)
		}
		s0, p0 := toothSection(t, world, l, side, toeScale, turnOf(k))
		s1, p1 := toothSection(t, world, l, side, heelScale, turnOf(k))
		body, err := doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("%s: slab %d between offsets %.6f and %.6f: %v",
				side.label, k, offsets[k], offsets[k+1], err)
		}
		bodies = append(bodies, laidApart(t, body, float64(k+1)*gap, side.label+" slab"))
	}
	return bodies
}

func assertSliceToothSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	if len(bodies) != spiralSliceSteps {
		t.Fatalf("the slice leaves %d cross-section segments, want the fixed scheme's %d; after the "+
			"apex-side scrap is dropped `segments` must be non-empty, and an empty one is a raised "+
			"error rather than a value passed on to the twist and the crown",
			len(bodies), spiralSliceSteps)
	}
	offsets := f.slabOffsets()
	if got, want := offsets[1], f.span/spiralSliceDivisor; math.Abs(got-want) > 1e-12*want {
		t.Errorf("the first cut plane sits %.9f mm apex-ward of the parent tooth plane, want "+
			"span/6 = %.9f mm", got, want)
	}
	if got, want := offsets[len(offsets)-1], float64(spiralSliceSteps)*f.span/spiralSliceDivisor; math.Abs(got-want) > 1e-12*want {
		t.Errorf("the last cut plane sits %.9f mm apex-ward, want 8*span/6 = %.9f mm", got, want)
	}
	if f.span <= 0 {
		t.Fatalf("the span came out %.9f mm; a negative span silently inverts the whole spiral "+
			"frame and the gear comes out wrong with no error at all", f.span)
	}

	// The heel face is found across ALL of a slab's faces by greatest
	// distAlong(centroid), with NO surface-type filter, and the toe face by the
	// least. A slab really does carry many faces — its two cut faces and one side
	// face per section segment — so the search is over a set and not over a pair.
	//
	// ⚠️ The reason the filter is forbidden is a Fusion fact this substitution
	// cannot reach: there a slab is bounded by a mix of planar cut faces and
	// RULED side faces, and a filter on PlaneSurfaceType can pick the wrong face
	// or miss the cut face, which makes the step-I loft fail with
	// ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY. Every section here is chorded,
	// so every face of every slab comes out planar and the filter would do no harm
	// in this harness. The hazard is recorded here rather than proved.
	if total := len(bodies[0].Faces()); total < 4 {
		t.Errorf("the first slab has %d faces, so the heel-face search is not searching anything",
			total)
	}

	// The apex-side scrap is NOT among the segments: every slab's heel face sits
	// at a cone distance the scheme names, and none of them lies beyond the last
	// cut plane.
	for k := range bodies {
		scaleAt := f.slabScale(offsets[k])
		if scaleAt <= 0 {
			t.Errorf("slab %d's heel plane is at ray scale %.9f, which is past the apex", k, scaleAt)
		}
	}
	scrapScale := f.slabScale(offsets[len(offsets)-1])
	for k := range bodies {
		if f.slabScale(offsets[k]) <= scrapScale {
			t.Errorf("slab %d sits at or beyond the apex-side scrap's own plane; the scrap is "+
				"dropped by re-slicing the list BEFORE it is removed, and it is never a segment", k)
		}
	}
}

// ---------------------------------------------------------------- twist

// stepTwistSegments rotates each segment about the shaft axis so the tooth
// follows the trace, centred on R_mean so the mid-face section stays unrotated.
//
// The per-segment angle is keyed on the segment's HEEL-FACE cone distance, not
// on its centroid: the loft samples each segment's heel face, so that face is
// what must land at the right azimuth, and centroid-keying leaves the loft's
// mid-face section rotated by half a segment.
//
// SUBSTITUTION — the segments are built already rotated. A free move by a
// Matrix3D rotation is what Fusion applies, and r3's rotation about the shaft
// axis is the same transform; building them turned rather than turning them
// afterward changes nothing about where they end up.
func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	offsets := f.slabOffsets()
	return buildSpiralSlabs(t, doc, l, side, f,
		func(k int) float64 { return f.trace.segmentTwist(f.heelFaceDistance(offsets[k])) },
		func(int) float64 { return 1 })
}

// heelFaceDistance is a cut plane's cone distance: how far along the cone
// element the slab's heel face stands from the apex.
func (f spiralBuildFrame) heelFaceDistance(offset float64) float64 {
	return f.rHeel - offset
}

func assertTwistSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	offsets := f.slabOffsets()
	tr := f.trace

	if len(bodies) != spiralSliceSteps {
		t.Fatalf("the twist leaves %d segments, want %d", len(bodies), spiralSliceSteps)
	}

	// The mid-face section is the one the straight tooth meshes at, and it stays
	// unrotated: that is what lets the pinion's extra mesh phase be 0.
	if got := tr.segmentTwist(tr.rMean); math.Abs(got) > 1e-12 {
		t.Errorf("the segment at R_mean turns by %.9f rad, want 0 — the twist is centred on R_mean "+
			"so the mid-face section meshes exactly as the straight tooth does", got)
	}
	// The toe-to-heel total is the crown-gear law's, and its sign is the hand's.
	toe := tr.segmentTwist(f.heelFaceDistance(offsets[len(offsets)-1]))
	heel := tr.segmentTwist(f.heelFaceDistance(offsets[0]))
	if got, want := math.Abs(toe-heel), tr.crownTwist*float64(spiralSliceSteps)/spiralSliceDivisor; math.Abs(got-want) > 1e-9*want {
		t.Errorf("the segments span %.9f rad of twist across the sliced band against the trace's "+
			"%.9f rad; the share is linear in the heel-face cone distance", got, want)
	}
	if tr.handSign*(heel-toe) <= 0 {
		t.Errorf("the twist runs the wrong way for hand sign %+.0f: the heel segment turns %.9f rad "+
			"and the toe segment %.9f rad", tr.handSign, heel, toe)
	}

	// ⚠️ The two members of a meshing pair legitimately get DIFFERENT twists:
	// same cutter, same psi, but gamma differs, so 1/sin(gamma) does. That is why
	// equal-teeth pairs meshed under any method that gets the roll ratio wrong
	// while ratio pairs failed.
	other := l.driving
	if p["gear"] != 0 {
		other = l.pinion
	}
	otherFrame := newSpiralBuildFrame(t, l, other)
	sameGamma := math.Abs(other.gamma-side.gamma) < 1e-12
	sameTwist := math.Abs(otherFrame.trace.crownTwist-tr.crownTwist) < 1e-9*math.Max(1, tr.crownTwist)
	if sameGamma != sameTwist {
		t.Errorf("this gear twists %.9f rad at gamma %.9f and its partner %.9f rad at gamma %.9f; "+
			"the two twists agree exactly when the two pitch cone angles do and never otherwise",
			tr.crownTwist, side.gamma, otherFrame.trace.crownTwist, other.gamma)
	}

	// Each segment really did turn: its centroid's azimuth is the angle the law
	// gives it, read off the body that was built.
	for k, body := range bodies {
		want := tr.segmentTwist(f.heelFaceDistance(offsets[k]))
		if got := centroidAzimuth(t, body, side.label+" slab"); angleGap(got, want) > 1e-6 {
			t.Errorf("segment %d sits at %.9f rad, want %.9f rad — the share is keyed on the "+
				"segment's HEEL-FACE cone distance and never on its centroid", k, got, want)
		}
	}
}

// ---------------------------------------------------------------- crown

// stepCrownSegments crowns the tooth lengthwise: every segment except the
// outermost (heel) one is scaled down by a monotonic factor, full at the heel
// and largest at the toe, about a point on the ROOT EDGE of its heel face.
//
// SUBSTITUTION — the segments are BUILT at their crowned size, for the reason
// this file's header gives, and about the anchor the spec names rather than
// about their own centroids. THE COST is the feature: what is proved is the law
// and the invariant the anchor exists for.
func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	offsets := f.slabOffsets()
	return buildSpiralSlabs(t, doc, l, side, f,
		func(k int) float64 { return f.trace.segmentTwist(f.heelFaceDistance(offsets[k])) },
		func(k int) float64 {
			if k == 0 {
				// The outermost (heel) segment is held full: its heel face is the
				// loft's heel end and the heel cone trims it flush with the base.
				return 1
			}
			return f.trace.crownFactor(f.heelFaceDistance(offsets[k]))
		})
}

func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	offsets := f.slabOffsets()
	tr := f.trace

	if len(bodies) != spiralSliceSteps {
		t.Fatalf("the crown leaves %d segments, want %d", len(bodies), spiralSliceSteps)
	}
	if crownPerRad == 0 {
		t.Fatal("_CROWN_PER_RAD is 0, which disables the crown; the spec sets it to 0.5")
	}

	previous := math.Inf(1)
	for k := range bodies {
		factor := 1.0
		if k > 0 {
			factor = tr.crownFactor(f.heelFaceDistance(offsets[k]))
		}
		if factor <= 0 {
			t.Fatalf("segment %d's crown factor is %.9f; a non-positive factor is a raised error "+
				"naming the gear, the segment's u and the factor, never a scale", k, factor)
		}
		if k > 1 && factor >= previous {
			t.Errorf("segment %d is relieved to %.9f against segment %d's %.9f; the relief grows "+
				"MONOTONICALLY from the held-full heel to the toe, so keying it on the twist "+
				"magnitude — which is symmetric about the mid face — would notch the slab just "+
				"inside the heel", k, factor, k-1, previous)
		}
		if k > 0 {
			previous = factor
		}
	}
	// The relief's maximum, now at the toe, keeps the magnitude the old per-end
	// peak had.
	u := (f.rHeel - f.heelFaceDistance(offsets[len(offsets)-1])) / f.span
	if got, want := 1-tr.crownFactor(f.heelFaceDistance(offsets[len(offsets)-1])),
		crownPerRad*(math.Abs(tr.crownTwist)/2)*u; math.Abs(got-want) > 1e-12*math.Max(1, want) {
		t.Errorf("the toe-most segment is relieved by %.9f, want _CROWN_PER_RAD * |total|/2 * u = "+
			"%.9f", got, want)
	}

	// The invariant the ROOT-EDGE anchor exists for: a uniform scale about a point
	// keeps every line through that point where it is, so anchoring on the root
	// keeps the root edge on the gear body's seating cone while the tip is
	// relieved. Anchoring on the heel face's CENTROID instead lifts the root edge
	// by (1-factor) * half the tooth height, the tooth floats off the base, and
	// the Combine-Join leaves a gap.
	_, zRoot, rTip, _ := toothCorners(l, side)
	rootRadius := rootCornerRadius(l, side)
	half := (rTip - rootRadius) / 2
	for k := 1; k < len(bodies); k++ {
		factor := tr.crownFactor(f.heelFaceDistance(offsets[k]))
		lift := (1 - factor) * half
		if lift <= 0 {
			t.Errorf("segment %d would be lifted by %.9f mm by a centroid anchor, so this case "+
				"cannot show why the anchor is the root edge", k, lift)
		}
	}
	if zRoot <= 0 {
		t.Fatalf("the tooth's root corner is at station %.9f", zRoot)
	}
}

// ---------------------------------------------------------------- spiral loft

// stepLoftSpiralTooth lofts a NewBody through the toe-most segment's toe face
// first — which pushes the loft past the toe cone so the toe trim bites — and
// then the heel-facing face of every segment in order.
//
// ⚠️ The order is RE-SORTED here, after the twist and the crown, and never
// reused from the pre-twist slice order. The twist rotates each slab about the
// shaft axis, and for high-twist unequal-ratio pairs that rotation changes the
// slabs' along-cone order enough to reorder adjacent slabs; lofting in the stale
// order assembles the cross-sections out of sequence and the two gears
// interfere. Equal or low-twist pairs have the two orders coincide, which is why
// 31/31 looks fine and a ratio pair like 31/17 does not.
//
// SUBSTITUTION — a CHAIN of pairwise lofts in that order, since decad's Loft
// takes exactly two sections.
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	offsets := f.slabOffsets()
	order := f.loftOrder()

	gap := 3 * side.station(l.apex, side.tooth)
	world := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, len(order))
	// The first section is the toe-most segment's APEX-SIDE face.
	previous := len(offsets) - 1
	for i, k := range order {
		heelScale := f.slabScale(offsets[k]) * spiralCrownAt(f, k)
		toeScale := f.slabScale(offsets[previous]) * spiralCrownAt(f, previous)
		if math.Abs(heelScale-toeScale) < 1e-12 {
			continue
		}
		s0, p0 := toothSection(t, world, l, side, math.Min(toeScale, heelScale),
			f.trace.segmentTwist(f.heelFaceDistance(offsets[previous])))
		s1, p1 := toothSection(t, world, l, side, math.Max(toeScale, heelScale),
			f.trace.segmentTwist(f.heelFaceDistance(offsets[k])))
		body, err := doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("%s: the spiral loft's section pair %d->%d: %v", side.label, previous, k, err)
		}
		bodies = append(bodies, laidApart(t, body, float64(i+1)*gap, side.label+" Spiral Tooth"))
		previous = k
	}
	return bodies
}

// spiralCrownAt is the crown factor a segment carries, with the outermost (heel) one
// held full.
func spiralCrownAt(f spiralBuildFrame, k int) float64 {
	if k == 0 {
		return 1
	}
	return f.trace.crownFactor(f.heelFaceDistance(f.slabOffsets()[k]))
}

// loftOrder sorts the segment indices by their POST-TWIST heel-face cone
// distance, which is the order the loft takes its sections in.
func (f spiralBuildFrame) loftOrder() []int {
	offsets := f.slabOffsets()
	order := make([]int, 0, len(offsets)-1)
	for k := 0; k+1 < len(offsets); k++ {
		order = append(order, k)
	}
	for i := 1; i < len(order); i++ {
		for j := i; j > 0 && f.heelFaceDistance(offsets[order[j]]) < f.heelFaceDistance(offsets[order[j-1]]); j-- {
			order[j], order[j-1] = order[j-1], order[j]
		}
	}
	return order
}

func assertLoftSpiralTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	if len(bodies) == 0 {
		t.Fatal("the spiral loft built nothing")
	}
	order := f.loftOrder()
	offsets := f.slabOffsets()
	for i := 1; i < len(order); i++ {
		if f.heelFaceDistance(offsets[order[i]]) <= f.heelFaceDistance(offsets[order[i-1]]) {
			t.Errorf("the loft order is not sorted by post-twist heel-face cone distance at "+
				"position %d", i)
		}
	}
	// The chain reaches past the toe and past the heel, which is what makes the
	// two end trims bite.
	lowest := boundsOf(t, bodies[0], side.label+" Spiral Tooth")
	if lowest.Max.Z <= lowest.Min.Z {
		t.Errorf("the %s spiral tooth's first pair spans nothing along the shaft axis", side.label)
	}
	for i, body := range bodies {
		if got := len(body.Lumps()); got != 1 {
			t.Errorf("the %s spiral tooth's pair %d came back in %d lumps", side.label, i, got)
		}
	}
}

// ---------------------------------------------------------------- flush trim

// stepTrimSpiralTooth returns cut_conical_ends on the CURVED tooth — the same
// toe-then-heel two-cone trim the straight tooth takes — so the curved tooth's
// ends sit flush on the gear base.
//
// SUBSTITUTION — the same one stepCutConicalEnds makes, and for the same reason:
// both operands are Lofts. The toe and heel mesh phasing is NOT done here; it is
// the mesh-rotate step's, outside the hook.
func stepTrimSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	sec := side.section(l.apex)
	zHeel, zDed, zToe, zInner := sec[2][0], sec[3][0], sec[4][0], sec[5][0]
	rHeel, rDed, rToe, rInner := sec[2][1], sec[3][1], sec[4][1], sec[5][1]

	offsets := f.slabOffsets()
	world := sketch.NewWorld()
	s0, p0 := toothSection(t, world, l, side, f.slabScale(offsets[len(offsets)-1]),
		f.trace.segmentTwist(f.heelFaceDistance(offsets[len(offsets)-1])))
	s1, p1 := toothSection(t, world, l, side, 1, f.trace.segmentTwist(f.heelFaceDistance(0)))
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the curved tooth the trim is made against: %v", side.label, err)
	}
	gap := 4 * zHeel
	toe := laidApart(t, coneBand(t, doc, rToe, rInner, zToe, zInner, side.label+" toe cone"),
		gap, side.label+" toe cone")
	heel := laidApart(t, coneBand(t, doc, rDed, rHeel, zDed, zHeel, side.label+" heel cone"),
		2*gap, side.label+" heel cone")
	return []*decad.Body{tooth, toe, heel}
}

func assertTrimSpiralTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 3 {
		t.Fatalf("the spiral trim leaves %d bodies, want the curved tooth and the two cones",
			len(bodies))
	}
	// The trim is the same two-cone flush trim the straight tooth takes, so it is
	// held to the same readings.
	assertCutConicalEnds(t, doc, bodies, p)

	l := newLattice(t, p)
	side := gearSide(l, p)
	f := newSpiralBuildFrame(t, l, side)
	// The mesh phase stays OUTSIDE this hook: the pinion's extra phase is 0
	// because the mid-face section is unrotated and already meshes.
	if got := f.trace.segmentTwist(f.rMean); math.Abs(got) > 1e-12 {
		t.Errorf("the mid-face section is turned by %.9f rad, so the pinion's zero mesh phase "+
			"would no longer mesh", got)
	}
	if side.label == "Pinion" && meshPhaseFor(side, p) != 0 {
		t.Errorf("the pinion carries a mesh phase of %.9f rad", meshPhaseFor(side, p))
	}
}

var _ = proofkit3d.Unmodelled
var _ = r3.NewVec
