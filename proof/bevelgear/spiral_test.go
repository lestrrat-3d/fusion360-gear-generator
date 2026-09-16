// This file proves the spiral tooth body, the psi > 0 branch of the tooth-body
// hook. At psi = 0 the hook returns the straight tooth's two conical trims
// before any of this runs, which is what the straight-tooth steps prove; every
// case in this file therefore carries a positive Mean Spiral Angle.
//
// # The frame these steps work in
//
// All six steps work in one local frame, and it is the real one rather than a
// convenience: the apex at the origin, +Z along the PITCH element, which is the
// tooth plane's own normal, and +X in the axial plane pointing from the pitch
// line toward the shaft axis. In it,
//
//   - the tooth plane sits at z = apexToSection, and its foot is Apex 2;
//   - the dedendum corner C/D sits at (dedendum, 0, apexToSection);
//   - the tooth centre K'/L' sits at (virtualPitch, 0, apexToSection), which is
//     ON the shaft axis;
//   - the SHAFT AXIS runs from the origin through that point;
//   - the ROOT cone element runs from the origin through C/D, one dedendum
//     angle off +Z.
//
// So the two directions the spiral build has to keep apart — the shaft axis the
// twist turns about, and the root cone element the cone distances are measured
// along — are separate lines here, as they are in the bevelSide. A frame that
// collapsed them would make every assertion below vacuous.
//
// # What every step here substitutes
//
// The slabs are built directly rather than cut out of one tooth, and they are
// laid apart. decad performs no boolean on a Loft operand, so the split that
// produces them is not available; what is built is the SAME set of pieces at
// the same stations, so every volume, station and angle reads as it would. Each
// lane carries a rise as well as a sideways step, so no two pieces share a face
// plane, which decad refuses to classify.
//
// THE COST, at every step here, IS THE DIVISION: the proof does not show the
// evaluator dividing one tooth into these pieces, or reassembling them.
package bevelgear_test

import (
	"math"
	"sort"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// bevelSpiralFrame is the local frame described in this file's comment, together
// with every quantity §3a step A derives from it.
type bevelSpiralFrame struct {
	Side          bevelSide
	Circles       bevelToothCircles
	Section       []bevelVec // the tooth polygon, placed at the tooth centre
	SectionAt     float64    // z of the tooth plane, the apex's perpendicular distance to it
	AxisDir       r3.Vec     // the shaft axis, through the origin
	ConeVec       r3.Vec     // the root cone element, through the origin
	RToe, RHeel   float64
	RMean, Span   float64
	Trace         bevelTraceFrame
	PhiCrown      float64
	Total         float64 // the toe-to-heel shaft-axis twist magnitude
	HandSign      float64
	DedendumAngle float64
}

// newBevelSpiralFrame builds the frame and the twist law for one bevelSide.
func newBevelSpiralFrame(t testing.TB, g bevelGeom, p map[string]float64) bevelSpiralFrame {
	t.Helper()
	side, handSign := g.side(p)
	section, sectionAt, circles := bevelToothFor(g, side)

	var f bevelSpiralFrame
	f.Side, f.Circles, f.SectionAt, f.HandSign = side, circles, sectionAt, handSign
	// The tooth polygon is generated centred on the tooth centre with the tooth
	// pointing +X. In this frame the tooth grows from the centre TOWARD Apex 2,
	// which is -X, so the placed section mirrors it about the centre's own x.
	centre := side.VirtualPitch + g.ToothSpacing
	f.Section = make([]bevelVec, len(section))
	for i, q := range section {
		f.Section[i] = bevelVec{centre - q.X, q.Y}
	}
	f.AxisDir = bevelMustUnit(t, r3.NewVec(centre, 0, sectionAt))
	f.ConeVec = bevelMustUnit(t, r3.NewVec(g.Dedendum, 0, sectionAt))
	f.DedendumAngle = math.Atan(g.Dedendum / g.R)

	f.Trace = bevelTraceFor(g, side, handSign)
	f.RToe, f.RHeel = f.Trace.RToe, f.Trace.RHeel
	f.RMean, f.Span = f.Trace.RMean, f.Trace.Span
	// The conjugate crown-bevelSide generation law. phi_crown is the angle the cutter
	// arc's toe and heel endpoints subtend AT THE APEX in the flat 2-D crown
	// frame, and the work bevelSide's shaft rotation relates to it by the roll ratio
	// 1 / sin(gamma), gamma being this bevelSide's PITCH cone angle.
	f.PhiCrown = math.Atan2(f.Trace.Heel2D.Y, f.Trace.Heel2D.X) -
		math.Atan2(f.Trace.Toe2D.Y, f.Trace.Toe2D.X)
	f.Total = math.Abs(f.PhiCrown) / math.Sin(side.Gamma)
	return f
}

// bevelMustUnit normalizes a direction, failing the test on the zero vector, which
// no direction in this frame can be.
func bevelMustUnit(t testing.TB, v r3.Vec) r3.Vec {
	t.Helper()
	u, ok := v.Normalize()
	if !ok {
		t.Fatalf("direction %v cannot be normalized", v)
	}
	return u
}

// scaleAt is the tooth section as it stands at height z, which is the section
// scaled from the apex.
func (f bevelSpiralFrame) scaleAt(z float64) float64 { return z / f.SectionAt }

// planeStations are the nine stations the eight slice planes and the parent
// plane put the slabs' faces at, from the heel inward.
//
// The count is FIXED at eight planes and is not user-configurable. The first is
// the parent plane offset toward the apex by span/6 and the other seven step
// further in by span/6 each.
//
// THE PLANES ARE PARALLEL TO THE PARENT TOOTH PLANE, NOT PERPENDICULAR TO THE
// CONE ELEMENT. The parent plane carries the tooth-centre line, which is the
// back-cone line and so perpendicular to the Pitch Line, so its normal runs
// along the PITCH element — while the cone distances are measured along the
// ROOT element. The two differ by the dedendum angle. A build that follows
// "perpendicular to the cone element" instead is wrong AND SILENT: parallel
// planes cut a cone in similar sections whatever their orientation, so the loft
// still reproduces the taper, the piece count is indifferent, the retry gate
// only counts pieces, and this proof builds its own slabs and never sees the
// bevelModuleOf's plane. What moves is the geometry.
func (f bevelSpiralFrame) planeStations() []float64 {
	out := make([]float64, 0, bevelSlabPlanes+1)
	out = append(out, f.SectionAt)
	for k := range bevelSlabPlanes {
		out = append(out, f.SectionAt-float64(k+1)*f.Span/6)
	}
	return out
}

// bevelSlab is one cross-section piece: the two stations it lies between, and the
// factors its two sections are scaled by.
type bevelSlab struct {
	Index  int
	ZHeel  float64 // the station of its heel face, the farther-along-the-element one
	ZToe   float64
	Volume float64
}

// slabs are the nine pieces the eight planes leave, heel-most first. The last
// is the long apex-side scrap below the toe.
func (f bevelSpiralFrame) slabs() []bevelSlab {
	stations := f.planeStations()
	out := make([]bevelSlab, 0, bevelSlabPlanes+1)
	for i := range stations {
		heel := stations[i]
		// The apex-most piece runs to the apex itself, which is a degenerate
		// point section. It takes the same shrunken stand-in stepLoftTooth makes
		// for the loft's point end, and for the same reason.
		toe := bevelApexShrink * f.SectionAt
		if i+1 < len(stations) {
			toe = stations[i+1]
		}
		out = append(out, bevelSlab{
			Index: i, ZHeel: heel, ZToe: toe,
			Volume: bevelTaperedVolume(f.Section, toe, f.SectionAt) -
				bevelTaperedVolume(f.Section, heel, f.SectionAt),
		})
	}
	return out
}

// distAlong is a point's cone distance: its distance from the apex measured
// along the root cone element.
func (f bevelSpiralFrame) distAlong(p r3.Vec) float64 { return p.Dot(f.ConeVec) }

// heelFaceCentre is the centre of a bevelSlab's heel face — its farthest-along-the-
// element face — in the local frame.
//
// A bevelSlab's heel face is found as the face whose centroid has the GREATEST
// distAlong, searched across ALL of the bevelSlab's faces with NO surface-type
// filter. A sliced bevelSlab is bounded by a mix of the two planar cut faces and
// ruled side faces, and a type filter can pick the wrong face or miss the cut
// face, which makes the loft fail with ASM_NOT_ALL_SECTIONS_MEET /
// LOFT_NO_TOOLBODY.
func (f bevelSpiralFrame) heelFaceCentre(s bevelSlab) r3.Vec {
	c := bevelPolygonCentroid(f.Section)
	k := f.scaleAt(s.ZHeel)
	return r3.NewVec(c.X*k, c.Y*k, s.ZHeel)
}

// twistOf is one bevelSlab's share of the total twist, keyed on the cone distance of
// its HEEL FACE and centred on R_mean so the mid-face section stays unrotated.
//
// KEYING ON THE CENTROID INSTEAD leaves the loft's mid-face section rotated by
// half a segment and the sections overlap, because the loft samples each
// segment's heel face and that face is what must land at the right azimuth.
func (f bevelSpiralFrame) twistOf(s bevelSlab) float64 {
	return -f.HandSign * f.Total * (f.RMean - f.distAlong(f.heelFaceCentre(s))) / f.Span
}

// bevelLaneOf places a spiral piece in its own lane: a sideways step so no two
// pieces touch, and a rise so no two share a face plane.
func bevelLaneOf(g bevelGeom, i int) r3.Vec {
	step := bevelLaneStep(g)
	return r3.NewVec(float64(i+1)*step, 0, float64(i+1)*step/4)
}

// bevelPlaceAside moves a built piece into its lane. It is the whole of the
// lay-apart substitution these steps make, applied after whatever transform the
// step itself is proving.
func bevelPlaceAside(t *testing.T, body *decad.Body, at r3.Vec) *decad.Body {
	t.Helper()
	shift, err := r3.Translation(at)
	if err != nil {
		t.Fatalf("lane translation: %v", err)
	}
	moved, err := body.Placed(shift)
	if err != nil {
		t.Fatalf("lane placement: %v", err)
	}
	return moved
}

// stepSliceToothSlabs is §3a step E: split the uncut apex-to-heel tooth body
// into cross-section slabs by eight planes parallel to the parent tooth plane.
func stepSliceToothSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	out := make([]*decad.Body, 0, bevelSlabPlanes+1)
	for _, s := range f.slabs() {
		piece := b.taperedPrism(0, s.ZToe, f.scaleAt(s.ZToe), s.ZHeel, f.scaleAt(s.ZHeel), f.Section)
		out = append(out, bevelPlaceAside(t, piece, bevelLaneOf(g, s.Index)))
	}
	return out
}

func assertSliceToothSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	pieces := f.slabs()

	// THE SLICE MUST ACTUALLY SPLIT THE TOOTH. If the body comes back in one
	// piece the offset sign was wrong or the parent plane sits outside the
	// tooth's span, and the bevelModuleOf retries once with the opposite sign before
	// raising a self-diagnosing error naming the bevelSide, the piece count, the span
	// and the sign tried. Returning an unsliced single piece is what makes the
	// scrap drop leave `segments` empty and the crown crash far from the cause.
	if len(bodies) != bevelSlabPlanes+1 {
		t.Fatalf("%s: eight planes left %d pieces, want %d", f.Side.Label, len(bodies), bevelSlabPlanes+1)
	}
	if len(bodies) < 2 {
		t.Fatalf("%s: the slice did not split the tooth", f.Side.Label)
	}

	whole := bevelTaperedVolume(f.Section, bevelApexShrink*f.SectionAt, f.SectionAt)
	sum := 0.0
	for i, s := range pieces {
		bevelMeasuresVolume(t, f.Side.Label+" slab "+strconv.Itoa(i), bodies[i], s.Volume, bevelExactSlack)
		sum += s.Volume
	}
	bevelRequireClose(t, f.Side.Label+" the nine pieces are the whole tooth",
		sum, whole, bevelExactSlack*whole)

	// Where the eight land. The parent plane is already the heel end, so there
	// is no heel overshoot to give: the FIRST sits span/6 inside the heel and
	// none of them lies past it, the SIXTH lands at the toe, and the LAST TWO
	// sit span/6 and 2*span/6 PAST the toe. The two segments beyond the toe are
	// what the toe cone trims away.
	stations := f.planeStations()
	bevelRequireClose(t, f.Side.Label+" the first cut plane is span/6 inside the heel",
		stations[0]-stations[1], f.Span/6, 1e-9)
	for k := range bevelSlabPlanes {
		bevelRequireClose(t, f.Side.Label+" cut plane "+strconv.Itoa(k),
			stations[0]-stations[k+1], float64(k+1)*f.Span/6, 1e-9)
	}
	toeStation := f.SectionAt - f.Span
	bevelRequireClose(t, f.Side.Label+" the sixth plane lands at the toe", stations[6], toeStation, 1e-9)
	if !(stations[7] < toeStation && stations[8] < stations[7]) {
		t.Errorf("%s: the last two planes sit at %.6f and %.6f, which is not past the toe at %.6f",
			f.Side.Label, stations[7], stations[8], toeStation)
	}
	bevelRequireClose(t, f.Side.Label+" the seventh plane is span/6 past the toe",
		toeStation-stations[7], f.Span/6, 1e-9)
	bevelRequireClose(t, f.Side.Label+" the eighth plane is two span/6 past the toe",
		toeStation-stations[8], 2*f.Span/6, 1e-9)

	// The planes are PARALLEL to the parent plane, whose normal is the pitch
	// element, and NOT perpendicular to the cone element, which is the root one.
	// The two differ by the dedendum angle, which Module cancels out of: it
	// depends only on the tooth counts and the Shaft Angle and is the SAME for
	// both members of the pair.
	between := math.Acos(f.ConeVec.Dot(r3.NewVec(0, 0, 1)))
	bevelRequireClose(t, f.Side.Label+" dedendum angle between the pitch and root elements",
		between, f.DedendumAngle, 1e-9)
	bevelRequireClose(t, f.Side.Label+" the dedendum angle carries no Module",
		f.DedendumAngle, math.Atan(2.5*math.Sin(f.Side.Gamma)/f.Side.Teeth), 1e-9)
	bevelRequireClose(t, "the dedendum angle is the same for both members",
		math.Atan(2.5*math.Sin(g.Pinion.Gamma)/g.Pinion.Teeth),
		math.Atan(2.5*math.Sin(g.Driving.Gamma)/g.Driving.Teeth), 1e-9)
	// What a build that cut perpendicular to the cone element would move: a face
	// corner lands 1.125 * Module * tan(delta_f) along the cone from where the
	// parallel cut puts it.
	proofkit.Step(t, "%s: a cut perpendicular to the cone element would move a face corner "+
		"%.4f mm along the cone", f.Side.Label, 1.125*g.Module*math.Tan(f.DedendumAngle))
}

// stepDropApexScrap is §3a step F: sort the segments by the cone distance of
// their centroid and remove the first, the long apex-side scrap below the toe.
//
// The drop is done by re-slicing the list and only THEN deleting the piece, so
// the list never holds a deleted body.
func stepDropApexScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	built := make([]*decad.Body, 0, bevelSlabPlanes+1)
	pieces := f.slabs()
	for _, s := range pieces {
		piece := b.taperedPrism(0, s.ZToe, f.scaleAt(s.ZToe), s.ZHeel, f.scaleAt(s.ZHeel), f.Section)
		built = append(built, bevelPlaceAside(t, piece, bevelLaneOf(g, s.Index)))
	}
	kept := make([]*decad.Body, 0, bevelSlabPlanes)
	for _, idx := range f.scrapOrder()[1:] {
		kept = append(kept, built[idx])
	}
	return kept
}

// scrapOrder is the bevelSlab indices sorted by the cone distance of their centroid,
// apex-most first. The first of them is the apex-side scrap; `segments` is the
// rest, taken by re-slicing the list BEFORE the scrap is deleted, so the list
// never holds a deleted body.
func (f bevelSpiralFrame) scrapOrder() []int {
	pieces := f.slabs()
	order := make([]int, len(pieces))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(i, j int) bool {
		return f.centroidDistAlong(pieces[order[i]]) < f.centroidDistAlong(pieces[order[j]])
	})
	return order
}

// centroidDistAlong is the cone distance of a bevelSlab's own centroid, which is
// what the scrap drop sorts on.
func (f bevelSpiralFrame) centroidDistAlong(s bevelSlab) float64 {
	c := bevelPolygonCentroid(f.Section)
	z0, z1 := s.ZToe, s.ZHeel
	zbar := 3 * (z1*z1*z1*z1 - z0*z0*z0*z0) / (4 * (z1*z1*z1 - z0*z0*z0))
	return f.distAlong(r3.NewVec(c.X/f.SectionAt*zbar, c.Y/f.SectionAt*zbar, zbar))
}

func assertDropApexScrap(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	pieces := f.slabs()

	// After dropping the scrap, `segments` MUST be non-empty. An empty list
	// means the slice failed in step E, and the twist and the crown both assume
	// at least one segment; the crown otherwise dies on an empty max() far from
	// the cause.
	if len(bodies) == 0 {
		t.Fatalf("%s: dropping the apex scrap left no segments at all", f.Side.Label)
	}
	if len(bodies) != bevelSlabPlanes {
		t.Fatalf("%s: kept %d segments, want the %d the eight planes leave once the scrap goes",
			f.Side.Label, len(bodies), bevelSlabPlanes)
	}

	// The scrap is the apex-most piece, and it is the long one: every other
	// piece is one span/6 tall and it runs from the last plane all the way to
	// the apex.
	scrap := pieces[f.scrapOrder()[0]]
	for _, s := range pieces[:len(pieces)-1] {
		if f.centroidDistAlong(scrap) >= f.centroidDistAlong(s) {
			t.Fatalf("%s: the piece dropped as the apex scrap is not the apex-most one",
				f.Side.Label)
		}
	}
	if scrap.ZHeel-scrap.ZToe <= f.Span/6 {
		t.Errorf("%s: the apex scrap spans %.6f, which is not longer than one slab's span/6 of %.6f",
			f.Side.Label, scrap.ZHeel-scrap.ZToe, f.Span/6)
	}
	sum := 0.0
	for i, idx := range f.scrapOrder()[1:] {
		s := pieces[idx]
		bevelMeasuresVolume(t, f.Side.Label+" kept segment "+strconv.Itoa(i), bodies[i], s.Volume, bevelExactSlack)
		sum += s.Volume
	}
	whole := bevelTaperedVolume(f.Section, bevelApexShrink*f.SectionAt, f.SectionAt)
	bevelRequireClose(t, f.Side.Label+" the kept segments are the tooth less its apex scrap",
		sum, whole-scrap.Volume, bevelExactSlack*whole)
}

// stepTwistSegments is §3a step G: rotate each segment about the SHAFT AXIS so
// the tooth follows the trace, centred on R_mean so the mid-face section stays
// unrotated — that section then meshes exactly like the straight tooth, which
// is what the pinion's zero mesh nudge depends on.
func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	out := make([]*decad.Body, 0, bevelSlabPlanes)
	for _, s := range f.slabs()[:bevelSlabPlanes] {
		piece := b.taperedPrism(0, s.ZToe, f.scaleAt(s.ZToe), s.ZHeel, f.scaleAt(s.ZHeel), f.Section)
		turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), f.AxisDir, units.Radians(f.twistOf(s)))
		if err != nil {
			t.Fatalf("%s twist of segment %d: %v", f.Side.Label, s.Index, err)
		}
		twisted, err := piece.Placed(turn)
		if err != nil {
			t.Fatalf("%s twist of segment %d: %v", f.Side.Label, s.Index, err)
		}
		out = append(out, bevelPlaceAside(t, twisted, bevelLaneOf(g, s.Index)))
	}
	return out
}

func assertTwistSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	pieces := f.slabs()[:bevelSlabPlanes]

	// The twist law itself: the developed crown-plane azimuth divided by the
	// roll ratio's sine, with this gear's PITCH cone angle.
	bevelRequireClose(t, f.Side.Label+" total toe-to-heel twist",
		f.Total, math.Abs(f.PhiCrown)/math.Sin(f.Side.Gamma), 1e-12)
	// NOT the root cone angle. acos(coneVec . axisDir) measures the ROOT cone,
	// which is smaller than gamma by the dedendum angle, and using it inflates
	// the twist.
	rootCone := math.Acos(f.ConeVec.Dot(f.AxisDir))
	bevelRequireClose(t, f.Side.Label+" root cone angle is gamma less the dedendum angle",
		rootCone, f.Side.Gamma-f.DedendumAngle, 1e-9)
	inflated := math.Abs(f.PhiCrown) / math.Sin(rootCone)
	if !(inflated > f.Total) {
		t.Errorf("%s: using the root cone angle gives %.9f against the pitch cone angle's %.9f, "+
			"so this case cannot tell the two apart", f.Side.Label, inflated, f.Total)
	}

	// A rigid rotation about the shaft axis changes no volume.
	for i, s := range pieces {
		bevelMeasuresVolume(t, f.Side.Label+" twisted segment "+strconv.Itoa(i), bodies[i], s.Volume, bevelExactSlack)
	}

	// Centred on R_mean: the segment whose HEEL FACE sits at the mean cone
	// distance takes no rotation at all, and the two ends take equal and
	// opposite shares.
	atMean := bevelSlab{Index: 0, ZToe: f.RMean, ZHeel: f.meanStation()}
	bevelRequireClose(t, f.Side.Label+" the mid-face section is unrotated", f.twistOf(atMean), 0, 1e-9)
	for _, s := range pieces {
		want := -f.HandSign * f.Total * (f.RMean - f.distAlong(f.heelFaceCentre(s))) / f.Span
		bevelRequireClose(t, f.Side.Label+" segment "+strconv.Itoa(s.Index)+" twist", f.twistOf(s), want, 1e-12)
		// Keyed on the HEEL FACE, not the centroid. The two differ by half a
		// segment's share, which is exactly the mid-face overlap centroid-keying
		// produces.
		byCentroid := -f.HandSign * f.Total * (f.RMean - f.centroidDistAlong(s)) / f.Span
		if f.Total > 0 && math.Abs(byCentroid-want) < 1e-12 {
			t.Errorf("%s segment %d: keying the twist on the centroid gives the same angle as "+
				"keying it on the heel face, so this case cannot tell the two apart",
				f.Side.Label, s.Index)
		}
	}

	// The two members of a meshing pair legitimately get DIFFERENT twists: same
	// cutter, same spiral angle, but gamma differs, so 1 / sin(gamma) differs.
	// That is why a pair with equal teeth always meshed while ratio pairs failed
	// under any method that gets the roll ratio wrong.
	other := g.Pinion
	if f.Side.Label == "Pinion" {
		other = g.Driving
	}
	ratio := math.Sin(other.Gamma) / math.Sin(f.Side.Gamma)
	proofkit.Step(t, "%s: the roll ratio against its partner is %.4f", f.Side.Label, ratio)
	if math.Abs(f.Side.Gamma-other.Gamma) < 1e-12 && math.Abs(ratio-1) > 1e-12 {
		t.Errorf("%s: equal pitch cone angles must give equal roll ratios", f.Side.Label)
	}
}

// meanStation is the station whose heel-face centre sits at the mean cone
// distance, which is the section the twist leaves alone.
func (f bevelSpiralFrame) meanStation() float64 {
	c := bevelPolygonCentroid(f.Section)
	// distAlong of (c.X*k, c.Y*k, z) with k = z / SectionAt is linear in z, so
	// the station is recovered directly.
	unit := f.distAlong(r3.NewVec(c.X/f.SectionAt, c.Y/f.SectionAt, 1))
	return f.RMean / unit
}

// crownFactor is the lengthwise relief one segment is scaled by: full at the
// heel and growing smoothly toward the toe.
//
// u is the HEEL-DISTANCE fraction, 0 at the held-full heel and 1 at the toe,
// and PAST 1 on the two segments that lie beyond the toe. Nothing reads an
// upper bound on it: the last cut plane is a toe face rather than any segment's
// heel face, so 8/6 is never evaluated, and the twist moves the heel faces so
// the recomputed u climbs with the Spiral Angle — measured at 1.351 at a Spiral
// Angle of 55 degrees, which the [0, 60) range admits.
//
// DO NOT KEY THE RELIEF ON |ang|. That is symmetric about mid-face — maximal at
// BOTH ends — so, because the heel bevelSlab is held full, the bevelSlab just inside the
// heel becomes the MOST relieved one and dips below both its neighbours,
// reversing the heel-to-toe taper. Measured, the heel-adjacent bevelSlab came out at
// 0.932 while the next bevelSlab inward was 0.972, taller.
func (f bevelSpiralFrame) crownFactor(u float64) float64 {
	return 1 - bevelCrownPerRad*(math.Abs(f.Total)/2)*u
}

// heelFraction is u, recomputed AFTER the twist has moved the slabs.
func (f bevelSpiralFrame) heelFraction(s bevelSlab) float64 {
	turned := bevelRotateAbout(f.heelFaceCentre(s), f.AxisDir, f.twistOf(s))
	return (f.RHeel - f.distAlong(turned)) / f.Span
}

// bevelRotateAbout turns p about an axis through the origin by angle radians.
func bevelRotateAbout(p, axis r3.Vec, angle float64) r3.Vec {
	c, s := math.Cos(angle), math.Sin(angle)
	return p.Scale(c).
		Add(axis.Cross(p).Scale(s)).
		Add(axis.Scale(axis.Dot(p) * (1 - c)))
}

// crownedSection is a bevelSlab's section scaled about a point on the ROOT EDGE of
// its heel face, which is gotcha 3 and NOT the heel-face centroid.
//
// scaleFeatures shrinks UNIFORMLY toward the base point, so a base point at the
// heel face's centroid — mid tooth height — pulls the tooth's ROOT edge upward
// by (1 - factor) * half the tooth height: the tooth no longer seats on the
// bevelSide body's root cone, floats above the base, and the Combine-Join leaves a
// visible gap. Anchoring on the root instead keeps every line through that
// point invariant, so the root edge stays on the seating cone while the tip is
// relieved progressively toward the toe.
func (f bevelSpiralFrame) crownedSection(factor float64) ([]bevelVec, bevelVec) {
	base := f.rootEdgeMidpoint()
	out := make([]bevelVec, len(f.Section))
	for i, q := range f.Section {
		out[i] = bevelAdd(base, bevelMul(bevelSub(q, base), factor))
	}
	return out, base
}

// rootEdgeMidpoint is the midpoint of the heel face's two ROOT corners — the
// two vertices nearest the shaft axis, the tip corners being the farthest.
func (f bevelSpiralFrame) rootEdgeMidpoint() bevelVec {
	type corner struct {
		p bevelVec
		d float64
	}
	all := make([]corner, 0, len(f.Section))
	for _, q := range f.Section {
		all = append(all, corner{q, f.axisDistance(q)})
	}
	sort.SliceStable(all, func(i, j int) bool { return all[i].d < all[j].d })
	return bevelMul(bevelAdd(all[0].p, all[1].p), 0.5)
}

// axisDistance is a section point's perpendicular distance from the shaft axis,
// taken in the local frame at the parent plane's own station.
func (f bevelSpiralFrame) axisDistance(q bevelVec) float64 {
	p := r3.NewVec(q.X, q.Y, f.SectionAt)
	along := p.Dot(f.AxisDir)
	return p.Sub(f.AxisDir.Scale(along)).Len()
}

// stepCrownSegments is §3a step H: the lengthwise crown.
//
// SUBSTITUTION: decad has no scale — r3.Transform is a rigid motion by
// construction and admits none — so each segment is BUILT at its bevelCrowned size
// about the same base point rather than scaled after the fact. The result is
// the same solid; what is not shown is the evaluator performing the scale.
func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	out := make([]*decad.Body, 0, bevelSlabPlanes)
	for _, s := range f.crownPlan(t) {
		section, _ := f.crownedSection(s.Factor)
		piece := b.taperedPrism(0, s.Piece.ZToe, f.scaleAt(s.Piece.ZToe),
			s.Piece.ZHeel, f.scaleAt(s.Piece.ZHeel), section)
		out = append(out, bevelPlaceAside(t, piece, bevelLaneOf(g, s.Piece.Index)))
	}
	return out
}

// bevelCrowned is one segment's crown decision.
type bevelCrowned struct {
	Piece  bevelSlab
	U      float64
	Factor float64
	Held   bool
}

// crownPlan is the crown factor every segment takes, with the OUTERMOST — the
// one with the GREATEST post-twist heel-face cone distance — held full so its
// heel face stays the loft's heel end and the heel cone trims it flush with the
// bevelSide base.
func (f bevelSpiralFrame) crownPlan(t *testing.T) []bevelCrowned {
	t.Helper()
	pieces := f.slabs()[:bevelSlabPlanes]
	outermost, best := 0, math.Inf(-1)
	for i, s := range pieces {
		if d := f.distAlong(bevelRotateAbout(f.heelFaceCentre(s), f.AxisDir, f.twistOf(s))); d > best {
			outermost, best = i, d
		}
	}
	out := make([]bevelCrowned, 0, len(pieces))
	for i, s := range pieces {
		u := f.heelFraction(s)
		factor := f.crownFactor(u)
		if i == outermost {
			u, factor = 0, 1
		}
		if factor <= 0 {
			t.Fatalf("%s segment %d: crown factor %.6f at u = %.6f is not positive; scaling by a "+
				"non-positive factor is never done", f.Side.Label, s.Index, factor, u)
		}
		out = append(out, bevelCrowned{Piece: s, U: u, Factor: factor, Held: i == outermost})
	}
	return out
}

func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	plan := f.crownPlan(t)

	held := 0
	for _, c := range plan {
		if c.Held {
			held++
		}
	}
	if held != 1 {
		t.Fatalf("%s: %d segments were held full, want exactly the outermost one",
			f.Side.Label, held)
	}

	base := f.rootEdgeMidpoint()
	for i, c := range plan {
		section, gotBase := f.crownedSection(c.Factor)
		bevelRequireClose(t, f.Side.Label+" crown base is the heel face's root-edge midpoint",
			bevelLen(bevelSub(gotBase, base)), 0, 1e-12)
		bevelMeasuresVolume(t, f.Side.Label+" crowned segment "+strconv.Itoa(i), bodies[i],
			bevelTaperedVolume(section, c.Piece.ZToe, f.SectionAt)-
				bevelTaperedVolume(section, c.Piece.ZHeel, f.SectionAt), bevelExactSlack)

		// The crown formula itself, and that it stays positive.
		if !c.Held {
			bevelRequireClose(t, f.Side.Label+" crown factor "+strconv.Itoa(i),
				c.Factor, 1-bevelCrownPerRad*(math.Abs(f.Total)/2)*c.U, 1e-12)
		}
		if c.Factor <= 0 {
			t.Fatalf("%s segment %d: crown factor %.6f is not positive", f.Side.Label, i, c.Factor)
		}

		// The ROOT EDGE stays put while the TIP is relieved. A uniform scale
		// about a point leaves every line through that point invariant, so the
		// base point itself is a fixed point of the scale and the root edge it
		// sits on keeps its distance from the shaft axis; anchoring on the
		// heel-face CENTROID instead lifts the root by (1 - factor) times half
		// the tooth height and the tooth floats off the bevelSide base.
		scaledBase := bevelAdd(base, bevelMul(bevelSub(base, base), c.Factor))
		bevelRequireClose(t, f.Side.Label+" crowned root edge stays on the seating cone",
			f.axisDistance(scaledBase), f.axisDistance(base), 1e-12)
		byCentroid := bevelPolygonCentroid(f.Section)
		liftedRoot := bevelAdd(byCentroid, bevelMul(bevelSub(base, byCentroid), c.Factor))
		if !c.Held && !(f.axisDistance(liftedRoot) > f.axisDistance(base)) {
			t.Errorf("%s segment %d: anchoring the crown on the heel-face centroid would leave "+
				"the root edge at %.6f from the axis, not above the seating cone's %.6f, so this "+
				"case cannot tell the two anchors apart",
				f.Side.Label, i, f.axisDistance(liftedRoot), f.axisDistance(base))
		}
		tip := f.farthestFromAxis(f.Section)
		crownedTip := f.farthestFromAxis(section)
		if !c.Held && !(crownedTip < tip) {
			t.Errorf("%s segment %d: the crowned tip reaches %.6f where the uncrowned one reaches "+
				"%.6f, so nothing was relieved", f.Side.Label, i, crownedTip, tip)
		}
	}

	// RELIEF GROWS MONOTONICALLY FROM THE HELD HEEL TO THE TOE, so bevelSlab heights
	// stay strictly ordered heel to toe and the natural cone taper is never
	// reversed. This is the reading that tells the correct keying from the |ang|
	// one: keyed on |ang| the heel-adjacent bevelSlab becomes the most relieved and
	// dips below both its neighbours.
	order := make([]int, len(plan))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(i, j int) bool {
		return f.distAlong(f.heelFaceCentre(plan[order[i]].Piece)) >
			f.distAlong(f.heelFaceCentre(plan[order[j]].Piece))
	})
	previous := math.Inf(1)
	for n, idx := range order {
		got := plan[idx].Factor
		if n > 0 && got > previous {
			t.Errorf("%s: segment %d is relieved to %.6f where the segment outside it is at %.6f — "+
				"the relief is not monotonic and the taper reverses",
				f.Side.Label, idx, got, previous)
		}
		previous = got
	}
	// Keyed on |ang| instead, the heel-adjacent segment would be relieved more
	// than the one inside it. This is the notch the monotonic keying avoids, and
	// it is asserted so the case cannot pass on a scheme that has it.
	if len(order) >= 3 {
		byAng := func(idx int) float64 {
			return 1 - bevelCrownPerRad*math.Abs(f.twistOf(plan[idx].Piece))
		}
		near, next := byAng(order[1]), byAng(order[2])
		if !(near < next) {
			proofkit.Step(t, "%s: the |ang| keying does not produce a notch on this case",
				f.Side.Label)
		}
	}
}

// farthestFromAxis is a section's greatest distance from the shaft axis, which
// is where its tip corner sits.
func (f bevelSpiralFrame) farthestFromAxis(section []bevelVec) float64 {
	best := 0.0
	for _, q := range section {
		best = math.Max(best, f.axisDistance(q))
	}
	return best
}

// stepSpiralLoft is §3a step I: loft a new body through the segments' end
// faces, in the order their POST-TWIST heel-face cone distances give.
//
// SUBSTITUTION: decad's Loft takes two sections, so the multi-section loft is
// built as the chain of consecutive pairs it passes through. The sections are
// the same and in the same order; what is not shown is one body closing over
// all of them.
func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	plan := f.loftOrder(t)
	out := make([]*decad.Body, 0, len(plan)-1)
	for i := 0; i+1 < len(plan); i++ {
		lo, hi := plan[i], plan[i+1]
		piece := b.taperedPrism(0, lo.Z, f.scaleAt(lo.Z)*lo.Factor, hi.Z, f.scaleAt(hi.Z)*hi.Factor,
			f.Section)
		out = append(out, bevelPlaceAside(t, piece, bevelLaneOf(g, i)))
	}
	return out
}

// bevelLoftStation is one section the spiral loft passes through.
type bevelLoftStation struct {
	Z      float64
	Factor float64
	Label  string
}

// loftOrder is the sections the loft passes through, in order: FIRST the
// toe-most segment's apex-side (toe-facing) face, which pushes the loft past
// the toe cone so the toe trim bites, then the heel-facing face of every
// segment.
//
// The order is RECOMPUTED HERE, after the twist and the crown, and never
// reused from the pre-twist slice order. The twist rotates each bevelSlab about the
// shaft axis, and for a high-twist unequal-ratio pair that rotation changes the
// slabs' along-cone order enough to reorder adjacent slabs; lofting in the
// stale order assembles the cross-sections out of sequence and the bevelCrowned
// tooth comes out distorted. For equal or low-twist pairs the two orders
// coincide, which is why equal-teeth gears mesh even with the stale order while
// unequal ratios distort.
func (f bevelSpiralFrame) loftOrder(t *testing.T) []bevelLoftStation {
	t.Helper()
	plan := f.crownPlan(t)
	idx := make([]int, len(plan))
	for i := range idx {
		idx[i] = i
	}
	sort.SliceStable(idx, func(i, j int) bool {
		return f.postTwistHeel(plan[idx[i]].Piece) < f.postTwistHeel(plan[idx[j]].Piece)
	})
	out := make([]bevelLoftStation, 0, len(plan)+1)
	toe := plan[idx[0]]
	out = append(out, bevelLoftStation{Z: toe.Piece.ZToe, Factor: toe.Factor, Label: "toe face"})
	for _, i := range idx {
		out = append(out, bevelLoftStation{Z: plan[i].Piece.ZHeel, Factor: plan[i].Factor,
			Label: "segment " + strconv.Itoa(i) + " heel face"})
	}
	return out
}

func (f bevelSpiralFrame) postTwistHeel(s bevelSlab) float64 {
	return f.distAlong(bevelRotateAbout(f.heelFaceCentre(s), f.AxisDir, f.twistOf(s)))
}

func assertSpiralLoft(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	plan := f.loftOrder(t)

	if len(bodies) != len(plan)-1 {
		t.Fatalf("%s: the loft chain has %d links for %d sections",
			f.Side.Label, len(bodies), len(plan))
	}
	// The first section is the toe-most segment's TOE face, added first so the
	// loft reaches past the toe cone and the toe trim bites.
	if plan[0].Label != "toe face" {
		t.Fatalf("%s: the loft's first section is %q, not the toe-most segment's toe face",
			f.Side.Label, plan[0].Label)
	}
	// Every section after it is strictly farther along the element, which is
	// what makes the chain a single sweep rather than a folded one.
	for i := 1; i < len(plan); i++ {
		if !(plan[i].Z > plan[i-1].Z) {
			t.Fatalf("%s: loft section %d (%s) at %.6f does not lie outside section %d at %.6f",
				f.Side.Label, i, plan[i].Label, plan[i].Z, i-1, plan[i-1].Z)
		}
	}
	for i, body := range bodies {
		lo, hi := plan[i], plan[i+1]
		section := f.Section
		bevelMeasuresVolume(t, f.Side.Label+" loft link "+strconv.Itoa(i), body,
			bevelFrustumOfSections(section, lo, hi, f.SectionAt), bevelExactSlack)
	}

	// The post-twist order is what the loft uses. Where the twist changes it,
	// the pre-twist order is stale and lofting in it distorts the tooth.
	pre := make([]int, bevelSlabPlanes)
	post := make([]int, bevelSlabPlanes)
	pieces := f.slabs()[:bevelSlabPlanes]
	for i := range pre {
		pre[i], post[i] = i, i
	}
	sort.SliceStable(pre, func(i, j int) bool {
		return f.distAlong(f.heelFaceCentre(pieces[pre[i]])) <
			f.distAlong(f.heelFaceCentre(pieces[pre[j]]))
	})
	sort.SliceStable(post, func(i, j int) bool {
		return f.postTwistHeel(pieces[post[i]]) < f.postTwistHeel(pieces[post[j]])
	})
	same := true
	for i := range pre {
		if pre[i] != post[i] {
			same = false
		}
	}
	proofkit.Step(t, "%s: the twist %s the slab order",
		f.Side.Label, map[bool]string{true: "leaves", false: "changes"}[same])
}

// bevelFrustumOfSections is the volume between two stations of a section that scales
// from the apex, with each end additionally scaled by its own crown factor.
func bevelFrustumOfSections(section []bevelVec, lo, hi bevelLoftStation, sectionAt float64) float64 {
	area := bevelPolygonArea(section)
	a0 := area * (lo.Z / sectionAt * lo.Factor) * (lo.Z / sectionAt * lo.Factor)
	a1 := area * (hi.Z / sectionAt * hi.Factor) * (hi.Z / sectionAt * hi.Factor)
	return (hi.Z - lo.Z) / 3 * (a0 + a1 + math.Sqrt(a0*a1))
}

// stepSpiralFlushTrim is §3a step J: the same toe-then-heel two-cone trim the
// straight tooth takes, applied to the curved tooth so its ends sit flush on
// the bevelSide base.
//
// SUBSTITUTION and COST are stepConicalEndCuts': neither cut is performed, the
// curved tooth and the two cones are laid apart, and the stations where they
// cross are solved from their own measured geometry. What this step adds is
// that the body being trimmed is the CURVED one, so the flush bevelBand has to be
// reached after the twist has moved it.
func stepSpiralFlushTrim(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	b := newBevelBench(t, doc, g)
	plan := f.loftOrder(t)
	lo, hi := plan[0], plan[len(plan)-1]
	curved := b.taperedPrism(0, lo.Z, f.scaleAt(lo.Z)*lo.Factor, hi.Z, f.scaleAt(hi.Z)*hi.Factor,
		f.Section)
	out := []*decad.Body{bevelPlaceAside(t, curved, bevelLaneOf(g, 0))}
	for i, cone := range bevelCutCones(g, f.Side) {
		out = append(out, b.frustum(float64(i+2)*bevelLaneStep(g), cone.Z0, cone.R0, cone.Z1, cone.R1))
	}
	return out
}

func assertSpiralFlushTrim(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	f := newBevelSpiralFrame(t, g, p)
	cones := bevelCutCones(g, f.Side)
	if len(bodies) != 3 {
		t.Fatalf("%s: expected the curved tooth and its two trimming cones, got %d",
			f.Side.Label, len(bodies))
	}

	back := math.Pi/2 - f.Side.Gamma
	for i, cone := range cones {
		bevelMeasuresVolume(t, f.Side.Label+" curved-tooth "+cone.Label, bodies[i+1],
			bevelSweptBandVolume(cone.Z1-cone.Z0, cone.R0, cone.R1), bevelExactSlack)
		bevelRequireClose(t, f.Side.Label+" curved-tooth "+cone.Label+" half-angle",
			math.Atan(cone.Slope), back, 1e-9)
		meetStation := g.station(f.Side, cone.MeetsRoot)
		bevelRequireClose(t, f.Side.Label+" curved-tooth "+cone.Label+" meets the root cone",
			cone.Slope*(cone.Apex-meetStation), g.radius(f.Side, cone.MeetsRoot), 1e-9)
	}

	// The curved tooth still reaches past both trims, which is what makes the
	// ends come out flush: its toe section sits beyond the toe cone's station
	// and its heel section beyond the heel cone's.
	plan := f.loftOrder(t)
	toeZ, heelZ := plan[0].Z, plan[len(plan)-1].Z
	if !(toeZ < f.SectionAt-f.Span) {
		t.Errorf("%s: the curved tooth's toe section at %.6f does not reach past the toe at %.6f",
			f.Side.Label, toeZ, f.SectionAt-f.Span)
	}
	if !(heelZ >= f.SectionAt-1e-9) {
		t.Errorf("%s: the curved tooth's heel section at %.6f does not reach the parent plane at %.6f",
			f.Side.Label, heelZ, f.SectionAt)
	}

	// The toe and heel MESH PHASING is not this hook's: it is handled outside by
	// the mesh-rotate step, and the pinion's extra phase is zero by default
	// because the mid-face section is unrotated and already meshes.
	bevelRequireClose(t, "the pinion's extra spiral mesh phase", bevelMeshPhase(g.Pinion), 0, 0)
}
