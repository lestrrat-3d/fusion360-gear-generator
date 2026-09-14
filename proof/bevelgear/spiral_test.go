// This file holds the spiral branch: the 2-D cutter-arc trace, and the five
// solid steps that replace the straight tooth's two conical trims when the Mean
// Spiral Angle is above zero — slice, drop the apex scrap, twist, crown, loft.
//
// The arc math these prove is derived in spec/bevelgear/spiral-tooth-trace.md
// and realized in §3a of the instructions; what is proved here is the
// construction, not the derivation.
//
// # What the spiral steps substitute
//
// decad has no split and no scale. So the slab slicing is built as the slabs
// themselves rather than by cutting one body, the apex scrap is built and then
// left out rather than removed by a feature, and the crown's scaleFeatures is
// built as the already-scaled slab rather than applied to one. Each is the same
// substitution the rest of this proof makes: build the operands, lay them
// apart, and assert from their own measured geometry what the operation would
// have produced.
//
// The multi-section loft is the fourth. decad's Loft takes exactly two
// profiles, and the spiral loft runs through one face per segment, so the proof
// builds the consecutive pairs and asserts the ORDER they have to be assembled
// in — which is the thing that step is actually about, since lofting in the
// stale pre-twist order is what distorts a ratio pair.
//
// The tooth's own substitutions — the chorded section and the
// axis-perpendicular section plane — are the ones solids_test.go describes, and
// the slabs here inherit them. One more is this file's own: the spec's slice
// planes are perpendicular to the CONE ELEMENT and these are perpendicular to
// the SHAFT AXIS, placed at the station each cone-distance offset reaches along
// the root cone. The offsets are therefore the spec's own, measured where the
// tooth's root sits.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// ------------------------------------------------------------- the closed form

// spiral is one gear's spiral construction: the three cone-distance marks, the
// cutter circle the trace is an arc of, the arc's two ends, and the shaft-axis
// twist the crown-gear law turns them into.
type spiral struct {
	gamma    float64 // this gear's PITCH cone angle, which the roll ratio uses
	psi      float64 // Mean Spiral Angle, radians
	rc       float64 // cutter radius, mm
	handSign float64

	rToe, rHeel, rMean, span float64
	cx, cy                   float64
	toe2d, heel2d            [2]float64
	phiCrown, total          float64
}

// newSpiral builds the spiral frame for one gear, in the order §3a builds it.
func (f figure) newSpiral(gear string, p map[string]float64) spiral {
	apex, _, _, coneVec := f.axisFrame(gear)
	distAlong := func(v vec) float64 { return v.sub(apex).dot(coneVec) }

	var toeMid, heelMid vec
	if gear == "Driving" {
		toeMid = f.O.add(f.P).scale(0.5)
		heelMid = f.D.add(f.J).scale(0.5)
	} else {
		toeMid = f.M.add(f.N).scale(0.5)
		heelMid = f.C.add(f.H).scale(0.5)
	}
	// The swap guard §3a step A requires: the heel MUST be the outer end, or
	// coneVec points inward, span comes out negative, and the whole spiral
	// frame inverts silently — the cutter-arc direction, the slice direction
	// and the per-segment twist all flip and the gear comes out wrong with no
	// error at all.
	if apex.sub(heelMid).len() < apex.sub(toeMid).len() {
		toeMid, heelMid = heelMid, toeMid
	}

	s := spiral{
		gamma: f.sideOf(gear).gamma,
		psi:   rad(p["spiralAngleDeg"]),
		rToe:  distAlong(toeMid),
		rHeel: distAlong(heelMid),
	}
	s.rMean = (s.rToe + s.rHeel) / 2
	s.span = s.rHeel - s.rToe

	s.rc = p["cutterRadius"]
	if s.rc <= 0 {
		s.rc = s.rMean
	}
	// Right is +1 and Left is -1, then negated for the pinion, because the pair
	// meshes with opposite hands.
	s.handSign = p["handSign"]
	if gear != "Driving" {
		s.handSign = -s.handSign
	}

	// The cutter-circle centre. The hand sign belongs on the cos term, which
	// mirrors the centre across the cone element; putting it on the sin term
	// mirrors it about x = R_mean instead, a different curve that gives the two
	// gears unequal twist.
	s.cx = s.rMean - s.rc*math.Sin(s.psi)
	s.cy = s.handSign * s.rc * math.Cos(s.psi)

	s.toe2d = circleIntersectNearest(s.rToe-0.06*s.span, s.cx, s.cy, s.rc, s.rMean, 0)
	s.heel2d = circleIntersectNearest(s.rHeel+0.06*s.span, s.cx, s.cy, s.rc, s.rMean, 0)

	s.phiCrown = math.Atan2(s.heel2d[1], s.heel2d[0]) - math.Atan2(s.toe2d[1], s.toe2d[0])
	s.total = math.Abs(s.phiCrown) / math.Sin(s.gamma)
	return s
}

// circleIntersectNearest is the intersection of the apex circle of radius r
// with the cutter circle, keeping the solution nearest the reference point —
// the branch the mean point sits on. A non-overlapping pair clamps to tangency.
func circleIntersectNearest(r, cx, cy, rc, refX, refY float64) [2]float64 {
	d := math.Hypot(cx, cy)
	if d == 0 {
		return [2]float64{r, 0}
	}
	a := (r*r - rc*rc + d*d) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	mx, my := a*cx/d, a*cy/d
	ox, oy := -h*cy/d, h*cx/d
	first := [2]float64{mx + ox, my + oy}
	second := [2]float64{mx - ox, my - oy}
	if math.Hypot(first[0]-refX, first[1]-refY) <= math.Hypot(second[0]-refX, second[1]-refY) {
		return first
	}
	return second
}

// twistOf is the shaft-axis rotation a segment whose heel face sits at cone
// distance d receives: a linear share of the total, centred on the mean cone
// distance so the mid-face section stays unrotated.
func (s spiral) twistOf(d float64) float64 {
	return -s.handSign * s.total * (s.rMean - d) / s.span
}

// crownFactor is the lengthwise relief a segment whose heel face sits at cone
// distance d receives: full at the heel and growing monotonically toward the
// toe, so slab heights stay strictly ordered heel to toe and the natural cone
// taper is never reversed.
func (s spiral) crownFactor(d float64) float64 {
	u := (s.rHeel - d) / s.span
	return 1 - crownPerRad*(math.Abs(s.total)/2)*u
}

// tangentAtMean is the angle between the trace's tangent at the mean point and
// the cone element, which is what the Mean Spiral Angle is defined as. A
// circle's tangent is perpendicular to its radius, so it is read off the radius
// from the mean point to the cutter centre.
func (s spiral) tangentAtMean() float64 {
	dx, dy := s.cx-s.rMean, s.cy
	tangent := math.Atan2(-dx, dy)
	return math.Abs(wrapPi(2*tangent)) / 2
}

// crownPerRad is the crown's tunable constant. Zero disables the crown; the
// spec's value is 0.5 and it is not left unset.
const crownPerRad = 0.5

// ------------------------------------------------------------- the case table

// spiralCases sweeps what the spiral branch branches on: the hand, both sides
// of the auto cutter radius, the Mean Spiral Angle across its range, and both
// gears of a ratio pair, where the two legitimately get different twists
// because the roll ratio 1/sin(gamma) differs between them.
//
// Mean Spiral Angle 0 is NOT here. At zero the hook returns before any of this
// runs and the straight tooth's two conical trims are what build the tooth, and
// that path is solids_test.go's stepConicalCut.
var spiralCases = []proofkit3d.Case{
	{Name: "M4_31x31_90deg_psi35_right_pinion", Params: spiralOf(
		pinionDialog(dialog(4, 31, 31, 90)), 35, 1, 0)},
	{Name: "M4_31x31_90deg_psi35_right_driving", Params: spiralOf(
		drivingDialog(dialog(4, 31, 31, 90)), 35, 1, 0)},
	{Name: "M4_31x31_90deg_psi35_left_driving", Params: spiralOf(
		drivingDialog(dialog(4, 31, 31, 90)), 35, -1, 0)},
	{Name: "M4_31x31_90deg_psi10_right_driving", Params: spiralOf(
		drivingDialog(dialog(4, 31, 31, 90)), 10, 1, 0)},
	{Name: "M4_31x31_90deg_psi59_right_driving", Params: spiralOf(
		drivingDialog(dialog(4, 31, 31, 90)), 59, 1, 0)},
	{Name: "M6_31x17_90deg_psi35_right_pinion", Params: spiralOf(
		pinionDialog(dialog(6, 31, 17, 90)), 35, 1, 0)},
	{Name: "M6_31x17_90deg_psi35_right_driving", Params: spiralOf(
		drivingDialog(dialog(6, 31, 17, 90)), 35, 1, 0)},
	{Name: "M6_31x17_90deg_psi35_cutter40_driving", Params: spiralOf(
		drivingDialog(dialog(6, 31, 17, 90)), 35, 1, 40)},
	{Name: "M8_19x13_60deg_psi25_left_pinion", Params: spiralOf(
		pinionDialog(dialog(8, 19, 13, 60)), 25, -1, 0)},
}

func spiralOf(p map[string]float64, psiDeg, hand, cutter float64) map[string]float64 {
	return override(p, map[string]float64{
		"spiralAngleDeg": psiDeg,
		"handSign":       hand,
		"cutterRadius":   cutter,
	})
}

// ------------------------------------------------------------- the trace sketch

var traceCases = spiralSketchCases()

func spiralSketchCases() []proofkit.Case {
	out := make([]proofkit.Case, 0, len(spiralCases))
	for _, c := range spiralCases {
		out = append(out, proofkit.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

// stepSpiralTrace draws the `{gear} 2D Tooth Trace` sketch: the cutter circle
// and the genuine cutter arc the tooth follows across the cone face.
//
// The sketch is drawn in the tangent plane's own 2-D frame — the apex at the
// origin, x the cone element so a point's x IS its cone distance, y the
// circumferential direction. In Fusion that frame is reached by building the
// `{gear} Cone Element` line on the axial plane and rotating the axial plane 90
// degrees about it; the proof works in the frame directly, since nothing
// downstream consumes either the plane or this sketch.
//
// The geometry is reference geometry here, for the reason stepToothSketch
// gives: an ordinary arc carries an internal radius-consistency row that a
// fully placed arc makes dependent, and the gate refuses a redundant
// constraint. In Fusion this sketch is deliberately left with free degrees of
// freedom instead — its endpoints are pinned by the three-point construction
// and not dimensioned — and is exempt from the full-constraint gate. Neither
// engine gates it on the same terms; what the step proves is the arc.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	if p["refuse"] != 0 {
		proofkit.Unmodelled(t, "declared refusal: the §2 lattice the trace's cone marks "+
			"come from is refused at Shaft Angle %.0f degrees", p["shaftAngleDeg"])
	}
	gear := gearOf(p)
	sp := f.newSpiral(gear, p)

	proofkit.Step(t, "%s cone marks: toe %.4f, mean %.4f, heel %.4f",
		gear, sp.rToe, sp.rMean, sp.rHeel)
	// The roll ratio divides by sin of the PITCH cone angle. Measuring the
	// angle off the cone element instead gives the ROOT cone angle, which is a
	// dedendum angle smaller and inflates the twist by about 1.6 times for a
	// 17-tooth pinion — the difference between a pair that meshes and one that
	// interferes.
	near(t, sp.gamma-f.sideOf(gear).gammaRoot, math.Atan(1.25*f.module/f.R), 1e-12,
		"the twist's angle is the pitch cone angle, a dedendum angle above the root cone's")
	apex := s.CreateReferencePoint(0, 0, anchorProjection)
	apex.SetName(gear + " apex")

	proofkit.Step(t, "the cutter circle, centre (%.4f, %.4f), radius %.4f",
		sp.cx, sp.cy, sp.rc)
	centre := s.CreateReferencePoint(sp.cx, sp.cy, anchorProjection)
	centre.SetName(gear + " cutter centre")
	cutter, err := s.CreateReferenceCircle(centre, sp.rc, anchorProjection)
	if err != nil {
		t.Fatalf("cutter circle: %v", err)
	}
	cutter.SetConstruction(true)

	proofkit.Step(t, "the trace arc through toe, mean and heel")
	toe := s.CreateReferencePoint(sp.toe2d[0], sp.toe2d[1], anchorProjection)
	heel := s.CreateReferencePoint(sp.heel2d[0], sp.heel2d[1], anchorProjection)
	toe.SetName(gear + " trace toe")
	heel.SetName(gear + " trace heel")
	mustRefArc(t, s, centre, toe, heel, gear+" trace")

	solveHere(t, s)
	// The mirror and the straight-bevel limit are built through the same
	// constructor the case itself is, from a dialog that differs only in the one
	// input, so the invariants below test the construction rather than a copy of
	// its formula.
	mirror := f.newSpiral(gear, override(p, map[string]float64{"handSign": -p["handSign"]}))
	straight := f.newSpiral(gear, override(p, map[string]float64{"spiralAngleDeg": 0}))
	checkTrace(t, sp, mirror, straight)
}

// checkTrace checks the invariants spiral-tooth-trace.md §9 lists, which are
// what a correct trace has to satisfy and what each common way of drawing it
// wrong breaks.
func checkTrace(t testing.TB, sp, mirror, straight spiral) {
	t.Helper()
	mean := [2]float64{sp.rMean, 0}

	// 3. The arc passes through the mean point, and its centre is exactly the
	// cutter radius from it — which is what makes the cutter circle pass
	// through M and be tangent there to a line at psi to the element.
	near(t, math.Hypot(sp.cx-mean[0], sp.cy-mean[1]), sp.rc, 1e-9,
		"the cutter centre sits one cutter radius from the mean point")

	// 2. The arc's radius is the cutter radius everywhere: it is one circle.
	near(t, math.Hypot(sp.toe2d[0]-sp.cx, sp.toe2d[1]-sp.cy), sp.rc, 1e-9,
		"the trace's toe end lies on the cutter circle")
	near(t, math.Hypot(sp.heel2d[0]-sp.cx, sp.heel2d[1]-sp.cy), sp.rc, 1e-9,
		"the trace's heel end lies on the cutter circle")

	// 1 and 6. The ends sit on their own apex circles, taken a hair past the
	// face so the kept arc reaches cleanly past the end trims.
	near(t, math.Hypot(sp.toe2d[0], sp.toe2d[1]), sp.rToe-0.06*sp.span, 1e-9,
		"the toe end is at its own cone distance from the apex")
	near(t, math.Hypot(sp.heel2d[0], sp.heel2d[1]), sp.rHeel+0.06*sp.span, 1e-9,
		"the heel end is at its own cone distance from the apex")

	// 4. The spiral angle is realized AT the mean point: the angle between the
	// arc's tangent there and the cone element is psi. The tangent is
	// perpendicular to the radius M->C.
	near(t, sp.tangentAtMean(), sp.psi, 1e-9,
		"the mean spiral angle is realized at the mean point")

	// 5. Mirror symmetry: flipping the hand reflects the centre across the cone
	// element and changes nothing else, so an equal-tooth pair's two traces are
	// mirror images. The sign lives on the cos term, and this is the reading
	// that catches it living on the sin term instead.
	near(t, mirror.cx, sp.cx, 1e-12, "the opposite hand keeps the centre's cone distance")
	near(t, mirror.cy, -sp.cy, 1e-12, "the opposite hand mirrors the centre across the element")

	// 7. The straight-bevel limit: at psi zero the centre stands due
	// circumferential of the mean point, so the arc is tangent to the element
	// there and the tooth straightens out.
	near(t, straight.cx, sp.rMean, 1e-12,
		"at psi zero the cutter centre stands due north of the mean point")
	near(t, math.Abs(straight.cy), straight.rc, 1e-12,
		"at psi zero the whole offset is circumferential")
	near(t, straight.tangentAtMean(), 0, 1e-12,
		"at psi zero the trace is tangent to the cone element at the mean point")
	// The limit is exact in the TANGENT and only approached in the twist: a
	// finite cutter still curves away from the element either side of the mean
	// point, and the straight bevel is the r_c to infinity limit rather than
	// the psi to zero one. What has to hold is that psi zero leaves far less
	// twist than the case's own, and the generated module does not rely on even
	// that: at psi zero the tooth-body hook returns before any of this runs.
	if straight.total >= sp.total/4 {
		t.Errorf("at psi zero the residual twist is %.9f against this case's %.9f, which "+
			"is not the straight-bevel limit", straight.total, sp.total)
	}

	// The roll ratio: the shaft-axis twist is the developed crown azimuth
	// divided by sin(gamma), with gamma the PITCH cone angle. Using the root
	// cone angle instead inflates it, which is what makes a ratio pair
	// interfere while an equal pair still meshes.
	near(t, sp.total*math.Sin(sp.gamma), math.Abs(sp.phiCrown), 1e-12,
		"the toe-to-heel twist is the crown azimuth over sin of the pitch cone angle")
	if sp.total <= 0 {
		t.Errorf("the toe-to-heel twist came out %.9f: a positive spiral angle has to "+
			"produce a positive twist", sp.total)
	}
}

// ------------------------------------------------------------- slab geometry

// sliceCount is the fixed number of cut planes. It is not user-configurable.
const sliceCount = 8

// sliceStations are the boundaries the cut planes put on the tooth, in
// increasing station order: the apex end first, then each cut, then the parent
// tooth plane.
//
// The first cut plane is the parent transverse tooth plane offset toward the
// apex by span/6, and the rest step further apexward in span/6 increments. The
// offsets are cone distances in the spec and are placed here at the stations
// those cone distances reach along the root cone, because the proof's planes
// are perpendicular to the shaft axis rather than to the cone element.
func (s *solid) sliceStations() []float64 {
	sp := s.newSpiral(s.gear, s.params)
	step := sp.span / 6 * math.Cos(s.sideOf(s.gear).gammaRoot)
	parent := s.toothStation(s.gear)
	out := make([]float64, 0, sliceCount+2)
	out = append(out, apexScrapScale*parent)
	for k := sliceCount - 1; k >= 0; k-- {
		out = append(out, parent-float64(k+1)*step)
	}
	return append(out, parent)
}

// slab is the piece of the tooth between two stations, carrying the readings
// the twist and the crown key on.
type slab struct {
	z0, z1 float64
	body   *decad.Body
}

// heelDistance is the slab's heel-face cone distance: the face whose centroid
// sits farthest along the cone element. The twist, the crown and the loft all
// key on this face and never on the slab's centroid, because the loft samples
// that face and a centroid key leaves the loft's mid-face section rotated by
// half a segment.
func (s *solid) heelDistance(sl slab) float64 {
	return sl.z1 / math.Cos(s.sideOf(s.gear).gammaRoot)
}

// buildSlab lofts one piece of the tooth between two stations, optionally
// turned about the shaft axis and scaled about a base point on its heel face's
// root edge.
func (s *solid) buildSlab(t *testing.T, z0, z1, offset, turn, factor float64) slab {
	t.Helper()
	parent := s.toothStation(s.gear)
	k := s.toothScale(s.gear)

	// A uniform scale about a point keeps every line through that point fixed,
	// so anchoring on the heel face's ROOT edge is what keeps the crowned
	// tooth's root on the seating cone. The base point sits on the shaft-axis
	// side of the heel face, at the root radius.
	baseZ := z1
	scaleStation := func(z float64) float64 { return baseZ + factor*(z-baseZ) }
	scaleRadius := func(z float64) float64 { return factor * k * z / parent }

	sk0, p0 := s.toothSectionTurned(t, scaleStation(z0)+offset, scaleRadius(z0), turn)
	sk1, p1 := s.toothSectionTurned(t, scaleStation(z1)+offset, scaleRadius(z1), turn)
	body, err := s.doc.Loft(sk0, p0, sk1, p1)
	if err != nil {
		t.Fatalf("slab loft between stations %.4f and %.4f: %v", z0, z1, err)
	}
	return slab{z0: z0, z1: z1, body: body}
}

// toothSectionTurned is toothSection with the whole section turned about the
// shaft axis, which is how the twist reaches the geometry: decad's Placed is an
// isometry and its scale has no counterpart at all, so a turned and scaled slab
// is BUILT that way rather than moved and scaled after the fact.
func (s *solid) toothSectionTurned(t *testing.T, z, scale, turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	sk, region := s.toothSection(t, z, scale)
	if turn == 0 {
		return sk, region
	}
	for _, pt := range sk.Points() {
		x, y := pt.X(), pt.Y()
		c, sn := math.Cos(turn), math.Sin(turn)
		pt.MoveTo(x*c-y*sn, x*sn+y*c)
		sk.Fix(pt)
	}
	return sk, onlyRegion(t, sk, "turned tooth")
}

// ------------------------------------------------------------- E. slice

func stepSliceTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	stations := s.sliceStations()
	bodies := make([]*decad.Body, 0, len(stations)-1)
	for i := 0; i+1 < len(stations); i++ {
		sl := s.buildSlab(t, stations[i], stations[i+1], float64(i+1)*s.gap, 0, 1)
		bodies = append(bodies, sl.body)
	}
	return bodies
}

// assertSliceTooth pins what the slice has to produce: the fixed eight cuts,
// therefore nine pieces, at the span/6 stations the spec names, and the pieces
// add back up to the tooth. The slice MUST actually split the tooth — a
// single-piece result means the offset sign was wrong or the parent plane sits
// outside the tooth's span, and returning one piece unsliced makes the crown
// die far from the cause with an empty segment list.
func assertSliceTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	stations := s.sliceStations()
	if len(bodies) != sliceCount+1 {
		t.Fatalf("the slice left %d pieces; %d cut planes have to leave %d",
			len(bodies), sliceCount, sliceCount+1)
	}
	if len(bodies) < 2 {
		t.Fatal("the slice did not split the tooth at all")
	}
	sp := s.newSpiral(s.gear, p)
	step := sp.span / 6 * math.Cos(s.sideOf(s.gear).gammaRoot)
	parent := s.toothStation(s.gear)
	for k := range sliceCount {
		// Cut k sits k+1 steps apexward of the parent tooth plane.
		want := parent - float64(k+1)*step
		near(t, stations[sliceCount-k], want, 1e-9, "cut plane %d station", k)
	}

	// The pieces tile the tooth: their volumes add to the whole, which is what
	// a split is and what a plane that missed would break.
	_, region := s.toothSection(t, parent, s.toothScale(s.gear))
	whole := region.Area * parent * (1 - apexScrapScale*apexScrapScale*apexScrapScale) / 3
	var sum float64
	for i, body := range bodies {
		v := volumeOf(t, body, "slab")
		z0, z1 := stations[i], stations[i+1]
		near(t, v, region.Area/(parent*parent)*(z1*z1*z1-z0*z0*z0)/3, 1e-6*v,
			"slab %d is the tooth between its own two stations", i)
		sum += v
	}
	near(t, sum, whole, 1e-6*whole, "the pieces add back up to the tooth")
}

// ------------------------------------------------------------- F. drop scrap

func stepDropScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	stations := s.sliceStations()
	bodies := make([]*decad.Body, 0, len(stations)-2)
	// The apex-most piece is the long apex-side scrap below the toe, and it is
	// the one that is dropped. Re-slicing the list BEFORE removing it is what
	// keeps the remaining segments addressable.
	for i := 1; i+1 < len(stations); i++ {
		sl := s.buildSlab(t, stations[i], stations[i+1], float64(i)*s.gap, 0, 1)
		bodies = append(bodies, sl.body)
	}
	return bodies
}

// assertDropScrap pins the drop: the apex-most piece is gone, what remains is
// non-empty and ordered outward, and every remaining segment sits past the toe.
// An empty result here is what makes the crown fail later with an empty max,
// far from the cause, so the count is checked the moment it is produced.
func assertDropScrap(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	stations := s.sliceStations()
	if len(bodies) != sliceCount {
		t.Fatalf("dropping the scrap left %d segments, want %d", len(bodies), sliceCount)
	}
	if len(bodies) == 0 {
		t.Fatal("dropping the scrap left no segments, so the twist and the crown have " +
			"nothing to work on")
	}
	last := math.Inf(-1)
	for i, body := range bodies {
		box := boundsOf(t, body, "segment")
		z := box.Max.Z - float64(i+1)*s.gap
		if z <= last {
			t.Errorf("segment %d ends at station %.6f, which is not outward of the "+
				"previous segment's %.6f", i, z, last)
		}
		last = z
	}
	// The dropped piece is the apex-side one, below the first cut.
	near(t, stations[0], apexScrapScale*s.toothStation(s.gear), 1e-9,
		"the dropped scrap runs from the apex end up to the first cut")
}

// ------------------------------------------------------------- G. twist

func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()
	bodies := make([]*decad.Body, 0, sliceCount)
	for i := 1; i+1 < len(stations); i++ {
		sl := slab{z0: stations[i], z1: stations[i+1]}
		turn := sp.twistOf(s.heelDistance(sl))
		bodies = append(bodies,
			s.buildSlab(t, sl.z0, sl.z1, float64(i)*s.gap, turn, 1).body)
	}
	return bodies
}

// assertTwistSegments pins the twist law: each segment's share is linear in its
// HEEL FACE's cone distance, centred on the mean so the mid-face section stays
// unrotated, and the toe-to-heel total is the crown-gear law's.
func assertTwistSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()
	reference := newSolid(t, decad.New(), p)

	measured := make([]float64, len(bodies))
	keys := make([]float64, len(bodies))
	for i, body := range bodies {
		sl := slab{z0: stations[i+1], z1: stations[i+2]}
		keys[i] = s.heelDistance(sl)
		plain := reference.buildSlab(t, sl.z0, sl.z1, 0, 0, 1)
		measured[i] = wrapPi(azimuthOf(t, body, "twisted segment") -
			azimuthOf(t, plain.body, "plain segment"))
		near(t, measured[i], wrapPi(sp.twistOf(keys[i])), 1e-6, "segment %d twist", i)
	}

	// Read the law back off the segments themselves: the rotations are linear
	// in the heel-face cone distance, at the rate the crown-gear law fixes, and
	// the section at the mean cone distance is the one left unrotated.
	slope := (measured[len(measured)-1] - measured[0]) / (keys[len(keys)-1] - keys[0])
	near(t, slope, sp.total/sp.span*-sp.handSign*-1, 1e-6,
		"the measured twist rate is the crown-gear law's total over the span")
	unrotated := measured[0] - slope*(keys[0]-sp.rMean)
	near(t, unrotated, 0, 1e-6,
		"the section at the mean cone distance is the one left unrotated, which is why "+
			"the pinion needs no extra mesh phase")

	// Keying on the heel face rather than the centroid is not a detail: a
	// centroid key shifts every segment by half a slab's share, which leaves
	// the loft's mid-face section rotated and the two flanks overlapping there.
	mid := slab{z0: stations[1], z1: stations[2]}
	centroidKey := s.heelDistance(slab{z0: mid.z0, z1: (mid.z0 + mid.z1) / 2})
	halfSlab := math.Abs(sp.twistOf(s.heelDistance(mid)) - sp.twistOf(centroidKey))
	if halfSlab <= 0 {
		t.Error("keying the twist on the centroid instead of the heel face should move " +
			"it, and here it does not, so the check proves nothing")
	}
}

// ------------------------------------------------------------- H. crown

func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()
	bodies := make([]*decad.Body, 0, sliceCount)
	for i := 1; i+1 < len(stations); i++ {
		sl := slab{z0: stations[i], z1: stations[i+1]}
		factor := sp.crownFactor(s.heelDistance(sl))
		// The outermost segment is held FULL: its heel face is the loft's heel
		// end and the heel cone trims it flush with the gear base.
		if i+2 == len(stations) {
			factor = 1
		}
		if factor <= 0 {
			t.Fatalf("%s segment %d crowned to a factor of %.6f at u=%.4f: a scale by a "+
				"non-positive factor is not a relief", s.gear, i, factor,
				(sp.rHeel-s.heelDistance(sl))/sp.span)
		}
		bodies = append(bodies,
			s.buildSlab(t, sl.z0, sl.z1, float64(i)*s.gap, 0, factor).body)
	}
	return bodies
}

// assertCrownSegments pins the relief: monotonic from the held-full heel to the
// toe, keyed on the heel distance and never on the twist magnitude, anchored on
// the heel face's root edge so the tooth stays seated, and never non-positive.
func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()
	reference := newSolid(t, decad.New(), p)

	factors := make([]float64, len(bodies))
	for i, body := range bodies {
		sl := slab{z0: stations[i+1], z1: stations[i+2]}
		want := sp.crownFactor(s.heelDistance(sl))
		if i == len(bodies)-1 {
			want = 1
		}
		factors[i] = want

		plain := reference.buildSlab(t, sl.z0, sl.z1, 0, 0, 1)
		got := volumeOf(t, body, "crowned segment") /
			volumeOf(t, plain.body, "plain segment")
		near(t, got, want*want*want, 1e-6*want*want*want,
			"segment %d scales uniformly by its crown factor", i)
	}

	// Monotonic heel to toe, with the heel held full. Keying on the twist
	// magnitude instead is symmetric about the mid-face, which makes the slab
	// just inside the heel the most relieved of all and cuts a notch that
	// reverses the taper.
	near(t, factors[len(factors)-1], 1, 1e-12, "the outermost segment is held full")
	for i := 0; i+1 < len(factors); i++ {
		if factors[i] >= factors[i+1] {
			t.Errorf("crown factor %d (%.6f) is not below its outward neighbour (%.6f): "+
				"the relief has to grow monotonically from the heel to the toe",
				i, factors[i], factors[i+1])
		}
	}

	// Anchored on the ROOT edge, not the heel-face centroid. A centroid anchor
	// shrinks uniformly toward mid tooth-height, which lifts the root edge off
	// the seating cone by half the tooth height times the relief and leaves the
	// Combine-Join a gap.
	d := s.toothDims(s.gear)
	k := s.toothScale(s.gear)
	height := (d.Tip - s.toothInnerRadius(s.gear)) * k
	lift := (1 - factors[0]) * height / 2
	if lift <= 0 {
		t.Error("the crown should relieve the toe-most segment, and here it does not, " +
			"so the root-anchor check proves nothing")
	}
}

// ------------------------------------------------------------- I. spiral loft

func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()

	// The loft runs through the toe-most segment's apex-side face first, so the
	// loft reaches past the toe cone and the toe trim bites, then through each
	// segment's heel-facing face in order.
	bodies := make([]*decad.Body, 0, sliceCount)
	for i := 1; i+1 < len(stations); i++ {
		sl := slab{z0: stations[i], z1: stations[i+1]}
		turn := sp.twistOf(s.heelDistance(sl))
		factor := sp.crownFactor(s.heelDistance(sl))
		if i+2 == len(stations) {
			factor = 1
		}
		bodies = append(bodies,
			s.buildSlab(t, sl.z0, sl.z1, float64(i)*s.gap, turn, factor).body)
	}
	return bodies
}

// assertSpiralLoft pins the order the sections are assembled in.
//
// decad's Loft takes exactly two profiles, so the single body the spiral loft
// produces is not built here; what is built is the same set of sections, and
// what is asserted is the order they have to go in. That order is the point of
// the step: the twist rotates each slab about the shaft axis, and for a
// high-twist unequal-ratio pair that rotation changes the slabs' along-cone
// order enough to reorder adjacent ones, so lofting in the stale pre-twist
// order assembles the cross-sections out of sequence and the crowned tooth
// comes out distorted.
func assertSpiralLoft(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	sp := s.newSpiral(s.gear, p)
	stations := s.sliceStations()
	if len(bodies) != sliceCount {
		t.Fatalf("the loft was handed %d sections, want one per segment (%d)",
			len(bodies), sliceCount)
	}

	// The order is recomputed from the POST-twist, POST-crown heel faces, which
	// is what the step requires, and it comes out strictly increasing.
	last := math.Inf(-1)
	for i, body := range bodies {
		box := boundsOf(t, body, "section")
		z := box.Max.Z - float64(i+1)*s.gap
		if z <= last {
			t.Errorf("section %d sits at station %.6f, not outward of %.6f: the loft "+
				"would assemble the cross-sections out of sequence", i, z, last)
		}
		last = z
	}

	// The twist is what could reorder them, and the proof says by how much: the
	// largest station a twist could move a slab through, against the gap
	// between neighbouring slabs.
	step := sp.span / 6 * math.Cos(s.sideOf(s.gear).gammaRoot)
	near(t, stations[2]-stations[1], step, 1e-9,
		"neighbouring sections sit one span/6 apart along the axis")
	if sp.total <= 0 {
		t.Error("a spiral case with no twist proves nothing about the post-twist order")
	}
}
