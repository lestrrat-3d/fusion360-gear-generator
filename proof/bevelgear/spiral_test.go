package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// The spiral frame, and the cutter arc it is built in
// ---------------------------------------------------------------------------

// spiral is the frame and the cutter-arc geometry §3a steps A and B build, for one
// gear. Everything below reads it and nothing else.
type spiral struct {
	// member is the gear this frame belongs to.
	member member

	apex    r3.Vec
	axisDir r3.Vec // the SHAFT axis, from the profile edge A'->G / B'->I
	coneVec r3.Vec // the dedendum (ROOT) cone element Apex->C / Apex->D
	v       r3.Vec // circumferential: axisDir x coneVec
	// tpNormal completes the frame and NOTHING CONSUMES IT. Step D removed the
	// projection that once used it, so it is computed and left unread; it is kept
	// here so a reader does not go looking for the consumer.
	tpNormal r3.Vec

	toeMid, heelMid        r3.Vec
	toeCone, heelCone      r3.Vec
	rToe, rHeel, rMean     float64
	span                   float64
	cutterRadius           float64
	handSign               float64
	cx, cy                 float64
	toe2d, heel2d          pt
	phiCrown, total        float64
	parentDistance         float64 // the Apex's perpendicular distance to the parent tooth plane
	parentNormalTowardApex r3.Vec
}

// newSpiral builds §3a's frame from the geometry §2 and §3 already constructed.
//
// ⚠️ The caller hand-off is the single biggest spiral-regen hazard, so it is spelled
// out: the toe edge is M->N (pinion) / O->P (driving) and the heel edge C->H / D->J;
// toeMid and heelMid are those two edges' MIDPOINTS — two different edges, never the
// two endpoints of one — and toeConeWorld/heelConeWorld are M/O and C/D. C/D is the
// dedendum corner and NEVER H/J: H/J lie on the Apex2->C / Apex2->D dedendum line,
// one Module beyond C/D and OFF the root cone element, so using them skews coneVec.
func newSpiral(f figure, g member, p map[string]float64) spiral {
	world := func(name string) r3.Vec {
		q := f.pts[name]
		return vec(f.stationOf(g, q), f.radiusOf(g, q), 0)
	}
	toeA, toeB := "M", "N"
	heelA, heelB := "C", "H"
	if g.label == "Driving" {
		toeA, toeB = "O", "P"
		heelA, heelB = "D", "J"
	}
	s := spiral{member: g, apex: vec(0, 0, 0), axisDir: vec(1, 0, 0)}
	s.toeCone, s.heelCone = world(toeA), world(heelA)
	s.toeMid = world(toeA).Add(world(toeB)).Scale(0.5)
	s.heelMid = world(heelA).Add(world(heelB)).Scale(0.5)

	// ⚠️ The heel MUST be the outer end so coneVec points outward and span > 0. A
	// negative span silently inverts the whole spiral frame — the cutter-arc
	// direction, the slice direction and the per-segment twist all flip — and the gear
	// comes out completely wrong with no error.
	if s.apex.Sub(s.heelMid).Len() < s.apex.Sub(s.toeMid).Len() {
		s.toeMid, s.heelMid = s.heelMid, s.toeMid
		s.toeCone, s.heelCone = s.heelCone, s.toeCone
	}
	s.coneVec, _ = s.heelCone.Sub(s.apex).Normalize()
	s.v, _ = s.axisDir.Cross(s.coneVec).Normalize()
	s.tpNormal, _ = s.coneVec.Cross(s.v).Normalize()

	s.rToe = s.toeMid.Sub(s.apex).Dot(s.coneVec)
	s.rHeel = s.heelMid.Sub(s.apex).Dot(s.coneVec)
	s.rMean = (s.rToe + s.rHeel) / 2
	s.span = s.rHeel - s.rToe

	s.cutterRadius = f.cutterRadius
	if s.cutterRadius == 0 {
		s.cutterRadius = s.rMean // the auto default
	}
	// The hand sign is +1 for Right else -1, then NEGATED FOR THE PINION, because the
	// pair meshes with opposite hands.
	s.handSign = f.handSign
	if g.label == "Pinion" {
		s.handSign = -s.handSign
	}
	// ⚠️ The hand sign goes on the cos / Cy term, NOT the sin / Cx term. Opposite hands
	// mirror the cutter centre across the cone element (y = 0), which flips Cy. Putting
	// it on Cx mirrors about x = R_mean instead — a different curve that gives the two
	// gears unequal twist, where for equal teeth the two traces must be exact mirrors.
	s.cx = s.rMean - s.cutterRadius*math.Sin(f.spiralAngle)
	s.cy = s.handSign * s.cutterRadius * math.Cos(f.spiralAngle)

	lo := s.rToe - traceEndOvershoot*s.span
	hi := s.rHeel + traceEndOvershoot*s.span
	s.toe2d = circleIntersectNearest(lo, s.cx, s.cy, s.cutterRadius, s.rMean, 0)
	s.heel2d = circleIntersectNearest(hi, s.cx, s.cy, s.cutterRadius, s.rMean, 0)

	s.phiCrown = math.Atan2(s.heel2d.Y, s.heel2d.X) - math.Atan2(s.toe2d.Y, s.toe2d.X)
	// ⚠️ gamma here is this gear's PITCH cone angle, not acos(coneVec . axisDir),
	// which is the ROOT cone angle — smaller by the dedendum angle and worth a twist
	// about 1.15x too large. The two members of a pair legitimately get different
	// twists: same cutter, same psi, different gamma, so 1/sin(gamma) differs.
	s.total = math.Abs(s.phiCrown) / math.Sin(g.gamma)

	s.parentDistance = f.apexDistance(g)
	centre, radialOut, circum := f.toothFrame(g)
	n, _ := radialOut.Cross(circum).Normalize()
	if centre.Dot(n) > 0 {
		n = n.Scale(-1) // point it apex-ward, which is the sign the offsets take
	}
	s.parentNormalTowardApex = n
	return s
}

// circleIntersectNearest intersects the apex circle of radius r with the cutter
// circle (centre (cx, cy), radius rc) and keeps the solution nearest the reference
// point — the branch the mean point sits on. A non-overlapping pair clamps to
// tangency, which is the framework helper's own behaviour.
func circleIntersectNearest(r, cx, cy, rc, refX, refY float64) pt {
	d := math.Hypot(cx, cy)
	if d == 0 {
		return pt{r, 0}
	}
	a := (d*d - rc*rc + r*r) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0 // tangency
	}
	h := math.Sqrt(h2)
	mx, my := cx*a/d, cy*a/d
	ux, uy := -cy/d, cx/d
	p1 := pt{mx + ux*h, my + uy*h}
	p2 := pt{mx - ux*h, my - uy*h}
	if math.Hypot(p1.X-refX, p1.Y-refY) <= math.Hypot(p2.X-refX, p2.Y-refY) {
		return p1
	}
	return p2
}

// ---------------------------------------------------------------------------
// The 2-D tooth trace sketch
// ---------------------------------------------------------------------------

// spiralCases carries both hands, the auto and an explicit cutter radius, the ends
// of the Mean Spiral Angle range [0, 60), and the ratio pair that separates the two
// members' twists. psi = 0 is a STRAIGHT bevel: the hook returns before any of this
// construction runs, which is what the psi = 0 cases assert.
var spiralCases = []proofkit.Case{
	{Name: "straight_psi_0", Params: caseParams(map[string]float64{"spiralAngleDeg": 0})},
	{Name: "default_psi_35_right", Params: caseParams(map[string]float64{"spiralAngleDeg": 35})},
	{Name: "default_psi_35_left", Params: caseParams(map[string]float64{"spiralAngleDeg": 35, "handLeft": 1})},
	{Name: "psi_55", Params: caseParams(map[string]float64{"spiralAngleDeg": 55})},
	{Name: "psi_59", Params: caseParams(map[string]float64{"spiralAngleDeg": 59})},
	{Name: "cutter_radius_user", Params: caseParams(map[string]float64{
		"spiralAngleDeg": 35, "cutterRadius": 30})},
	{Name: "ratio_driving_31_pinion_17", Params: caseParams(map[string]float64{
		"spiralAngleDeg": 35, "pinionTeeth": 17})},
	{Name: "toe_extension_100", Params: caseParams(map[string]float64{
		"spiralAngleDeg": 35, "toeExtension": 100})},
}

// stepSpiralTrace draws the genuine cutter arc — a real circle of the cutter's
// radius, never a fitted spline through sampled points — and proves the invariants
// `spiral-tooth-trace.md` §9 lists.
//
// Substitution, and what it costs: the Fusion sketch is DELIBERATELY left with free
// DOF (its toe and heel endpoints are pinned by the three-point construction, not
// dimensioned, because dimensioning them over-constrains the solve against the
// cone-element plane) and is exempt from the full-constraint gate. This harness
// gates every sketch, so the proof pins the three through-points and holds the arc's
// centre with one SIGNED component beside the arc's own equidistance row, then
// asserts the centre and the radius rather than dimensioning both — a radius
// dimension beside a centre coincidence is a third row for two freedoms. The cost is
// that the free-DOF sketch Fusion authors is not the sketch proved here.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)
	for _, g := range []member{f.pinion, f.driving} {
		sp := newSpiral(f, g, p)
		if f.spiralAngle <= 0 {
			// The hook's psi = 0 gate returns the straight-tooth path before any of this
			// construction runs, so NO BUILT TOOTH carries the residual sweep a finite
			// cutter radius leaves even at psi = 0.
			proofkit.Step(t, "%s: psi = 0 is a STRAIGHT bevel — the tooth-body hook returns "+
				"cut_conical_ends and every spiral input is ignored", g.label)
			residual := math.Abs(sp.phiCrown)
			if residual == 0 {
				t.Errorf("%s: at psi = 0 the arc is tangent to the element at the mean point and "+
					"only there, so its ends still subtend a residual sweep at the apex", g.label)
			}
			s.CreateReferencePoint(sp.rMean, 0, g.label+" mean point").SetName(g.label + " mean point")
			continue
		}
		drawTrace(t, s, f, sp)
	}
	if f.spiralAngle <= 0 {
		return
	}

	// Mirror symmetry: swapping the hand reflects the whole construction across the
	// radial through the mean point and changes nothing else, so an equal-tooth pair's
	// two traces are exact mirror images. The two members here already carry opposite
	// hands, because the pinion negates the driving gear's.
	if f.pinion.teeth == f.driving.teeth {
		a := newSpiral(f, f.pinion, p)
		b := newSpiral(f, f.driving, p)
		if rel(a.cx, b.cx) > 1e-12 || rel(a.cy, -b.cy) > 1e-12 {
			t.Errorf("equal teeth: the two cutter centres are (%.6f, %.6f) and (%.6f, %.6f); "+
				"opposite hands mirror across the cone element, so only Cy may differ",
				a.cx, a.cy, b.cx, b.cy)
		}
		if rel(a.total, b.total) > 1e-12 {
			t.Errorf("equal teeth: the two twists are %.9f and %.9f rad; same cutter, same psi "+
				"and equal gamma must give equal twist", a.total, b.total)
		}
	}
}

func drawTrace(t testing.TB, s *sketch.Sketch, f figure, sp spiral) {
	g := sp.member
	proofkit.Step(t, "%s: cutter radius %.4f, centre (%.4f, %.4f), twist %.6f rad",
		g.label, sp.cutterRadius, sp.cx, sp.cy, sp.total)

	// The frame: origin at the apex, x = coneVec (so a point's x is its cone
	// distance) and y = v (circumferential). The proof lays the two gears' traces side
	// by side in one sketch, which changes no length and no angle.
	lay := 0.0
	if g.label == "Driving" {
		lay = 4 * sp.rHeel
	}
	at := func(q pt) pt { return pt{q.X + lay, q.Y} }

	origin := s.CreateReferencePoint(lay, 0, g.label+" apex")
	origin.SetName(g.label + " apex")

	centre := s.CreatePoint(at(pt{sp.cx, sp.cy}).X, sp.cy)
	s.Fix(centre)
	cutter := s.CreateCircle(centre, sp.cutterRadius)
	cutter.SetConstruction(true)
	cutter.SetName(g.label + " cutter circle")
	dim := sketch.NewDiameter(cutter, 2*sp.cutterRadius)
	s.AddConstraint(dim)
	s.SetConstraintName(dim, g.label+" cutter diameter")

	toe := s.CreatePoint(at(sp.toe2d).X, sp.toe2d.Y)
	mean := s.CreatePoint(at(pt{sp.rMean, 0}).X, 0)
	heel := s.CreatePoint(at(sp.heel2d).X, sp.heel2d.Y)
	for _, q := range []*sketch.Point{toe, mean, heel} {
		s.Fix(q)
	}
	toe.SetName(g.label + " trace toe")
	mean.SetName(g.label + " trace mean")
	heel.SetName(g.label + " trace heel")

	arcCentre := s.CreatePoint(at(pt{sp.cx, sp.cy}).X, sp.cy)
	arc := s.CreateArc(arcCentre, toe, heel)
	arc.SetName(g.label + " trace arc")
	// One signed component, beside the arc's own equidistance row, locates the centre
	// uniquely. Fusion instead makes the arc's centre coincident with the cutter
	// circle's centre and dimensions its radius; both are asserted below.
	pin := sketch.NewHorizontalDistance(origin, arcCentre, sp.cx)
	s.AddConstraint(pin)
	s.SetConstraintName(pin, g.label+" trace arc centre")

	// The arc is the genuine cutter circle: its centre is the cutter centre and its
	// radius is r_c.
	if d := math.Hypot(arcCentre.X()-centre.X(), arcCentre.Y()-centre.Y()); d > 1e-9 {
		t.Errorf("%s: the trace arc's centre sits %.9f mm from the cutter circle's", g.label, d)
	}
	if got := arc.R(); rel(got, sp.cutterRadius) > 1e-9 {
		t.Errorf("%s: the trace arc's radius is %.6f, want the cutter radius %.6f",
			g.label, got, sp.cutterRadius)
	}

	// The ends sit on the right apex circles, a hair past the face so the kept arc
	// reaches cleanly past the end trims.
	lo := sp.rToe - traceEndOvershoot*sp.span
	hi := sp.rHeel + traceEndOvershoot*sp.span
	if got := math.Hypot(sp.toe2d.X, sp.toe2d.Y); rel(got, lo) > 1e-9 {
		t.Errorf("%s: the toe end sits at cone distance %.6f, want %.6f — the wrong intersection "+
			"branch was kept", g.label, got, lo)
	}
	if got := math.Hypot(sp.heel2d.X, sp.heel2d.Y); rel(got, hi) > 1e-9 {
		t.Errorf("%s: the heel end sits at cone distance %.6f, want %.6f", g.label, got, hi)
	}

	// The mean spiral angle is realised AT the mean point: the arc's tangent there
	// makes angle psi with the cone element. It is only exactly psi at that point;
	// toe and heel see larger and smaller angles, which is the signature of a real
	// circular-cut spiral bevel rather than an error.
	tangent := pt{-(0 - sp.cy), sp.rMean - sp.cx} // perpendicular to the radius C->mean
	if sp.handSign < 0 {
		tangent = pt{-tangent.X, -tangent.Y}
	}
	psi := math.Abs(math.Atan2(tangent.Y, tangent.X))
	if psi > math.Pi/2 {
		psi = math.Pi - psi
	}
	if math.Abs(psi-f.spiralAngle) > 1e-9 {
		t.Errorf("%s: the trace makes %.9f rad with the element at the mean point, want psi = %.9f",
			g.label, psi, f.spiralAngle)
	}
	// The centre is exactly r_c from the mean point, so the cutter circle passes
	// through it.
	if got := math.Hypot(sp.rMean-sp.cx, sp.cy); rel(got, sp.cutterRadius) > 1e-12 {
		t.Errorf("%s: the cutter centre sits %.6f from the mean point, want r_c = %.6f",
			g.label, got, sp.cutterRadius)
	}
}

// ---------------------------------------------------------------------------
// The spiral tooth body
// ---------------------------------------------------------------------------

// spiralSolidCases runs the spiral body chain at Module 4 to 8, per gear, over both
// hands and the ends of the Mean Spiral Angle range.
var spiralSolidCases = solidTable([]struct {
	name string
	over map[string]float64
}{
	{"default_psi_35", map[string]float64{"module": 4, "spiralAngleDeg": 35}},
	{"psi_35_left", map[string]float64{"module": 4, "spiralAngleDeg": 35, "handLeft": 1}},
	{"psi_55", map[string]float64{"module": 4, "spiralAngleDeg": 55}},
	{"module_8_psi_35", map[string]float64{"module": 8, "spiralAngleDeg": 35}},
	{"ratio_driving_31_pinion_17", map[string]float64{
		"module": 4, "spiralAngleDeg": 35, "pinionTeeth": 17}},
	{"cutter_radius_user", map[string]float64{
		"module": 4, "spiralAngleDeg": 35, "cutterRadius": 60}},
})

// slabOffsets are the eight cut planes, all PARALLEL to the parent transverse tooth
// plane and offset toward the apex in span/6 increments.
//
// ⚠️ The planes are NOT perpendicular to the cone element. The parent plane carries
// the tooth-centre line C->K' / D->L', which is the back-cone line and so
// perpendicular to the Pitch Line, so its normal runs along the PITCH element while
// coneVec is the ROOT element; the two differ by the dedendum angle
// atan(1.25*Module/R), which depends only on the tooth counts and the Shaft Angle
// and is the same for both members. The parallel family is what the build REQUIRES
// rather than what it happens to use: the framework helper offsets the parent plane,
// which produces parallel planes; the sign test reads the parent plane's own normal,
// which is meaningful only for that plane's own offsets; and the tooth is lofted to
// the profile drawn in the parent plane, so the heel-most slab's heel face IS the
// parent plane and a consistent family has to contain it.
//
// ⚠️ A build that follows "perpendicular to the cone element" is WRONG AND SILENT:
// it tilts every cut face by the dedendum angle and nothing in the pipeline fails.
// What moves is the geometry. THE PROOF CANNOT CATCH IT — it builds its own slabs
// from the offsets this spec fixes and never reads the plane the generated module
// constructs, so the module's choice of family reaches Fusion untested. That is the
// same shape of gap as the §2 seed, and only a measurement on a loaded spiral gear
// closes it.
func slabOffsets(sp spiral) []float64 {
	out := make([]float64, 0, sliceCount)
	for k := range sliceCount {
		out = append(out, float64(k+1)*sp.span/6)
	}
	return out
}

// slab is one cross-section segment: the two scale factors that bound it, plus the
// twist and crown its own step applies.
type slab struct {
	heelScale, toeScale float64
	twist               float64
	crown               float64
	base                pt // the heel face's root-edge midpoint, in the section's own plane
}

// slabs returns the eight working segments, heel first. The apex-side scrap below
// the toe is NOT one of them: step F sorts the pieces by the distAlong of their
// centroid and removes the apex-most, which is the long scrap, re-slicing the list
// BEFORE deleting it so `segments` is never left empty.
func (f figure) slabs(sp spiral) []slab {
	d := sp.parentDistance
	offsets := append([]float64{0}, slabOffsets(sp)...)
	out := make([]slab, 0, sliceCount)
	for k := range sliceCount {
		out = append(out, slab{heelScale: (d - offsets[k]) / d, toeScale: (d - offsets[k+1]) / d})
	}
	return out
}

// scrapScale is where the apex-side scrap's own toe end is cut off. The real scrap
// runs to the degenerate apex point, which is not a section this evaluator lofts.
const scrapScale = 0.08

// sectionOf returns one slab face's world polygon: the tooth outline scaled about
// the Apex, then crowned about its own base point, then twisted about the shaft
// axis.
func (f figure) sectionOf(g member, outline []pt, scale, crown, twist float64, base pt) []r3.Vec {
	out := make([]r3.Vec, 0, len(outline))
	sin, cos := math.Sin(twist), math.Cos(twist)
	for _, q := range outline {
		// The crown is a uniform scale about a point ON THE ROOT EDGE, never about the
		// heel face's centroid: a uniform scale keeps every line through its base point
		// invariant, so anchoring on the root keeps the root edge on the seating cone
		// while the tip is relieved. A centroid base pulls the root upward by
		// (1-factor) * half the tooth height and the tooth floats off the gear body.
		c := pt{base.X + (q.X-base.X)*crown, base.Y + (q.Y-base.Y)*crown}
		w := f.place(g, c, scale)
		out = append(out, vec(w.X, w.Y*cos-w.Z*sin, w.Y*sin+w.Z*cos))
	}
	return out
}

// rootBase is the heel face's ROOT-EDGE midpoint in the section's own plane: of the
// face's vertices, the two with the smallest perpendicular distance to the shaft
// axis are the root corners, and the tip corners are the farthest from it.
func (f figure) rootBase(g member, outline []pt, scale float64) pt {
	type cand struct {
		q pt
		d float64
	}
	best := []cand{{d: math.Inf(1)}, {d: math.Inf(1)}}
	for _, q := range outline {
		w := f.place(g, q, scale)
		d := math.Hypot(w.Y, w.Z)
		switch {
		case d < best[0].d:
			best[1] = best[0]
			best[0] = cand{q, d}
		case d < best[1].d:
			best[1] = cand{q, d}
		}
	}
	return pt{(best[0].q.X + best[1].q.X) / 2, (best[0].q.Y + best[1].q.Y) / 2}
}

// distAlong is a point's cone distance: its distance from the apex measured along
// the cone element.
func (sp spiral) distAlong(p r3.Vec) float64 { return p.Sub(sp.apex).Dot(sp.coneVec) }

func centroid(poly []r3.Vec) r3.Vec {
	sum := vec(0, 0, 0)
	for _, q := range poly {
		sum = sum.Add(q)
	}
	return sum.Scale(1 / float64(len(poly)))
}

// buildSlab lofts one segment between its two faces and lays it clear of its
// neighbours along the shaft axis, since two segments that share a cut face report a
// pair contact decad refuses to classify.
func buildSlab(t *testing.T, doc *decad.Document, w *sketch.World, heel, toe []r3.Vec,
	offset float64) *decad.Body {
	s0, p0 := polygonSection(t, w, heel, offset)
	s1, p1 := polygonSection(t, w, toe, offset)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("slab loft: %v", err)
	}
	return body
}

// polygonSection puts a world polygon on its own plane, shifted along the shaft
// axis by offset.
func polygonSection(t *testing.T, w *sketch.World, poly []r3.Vec, offset float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	origin := centroid(poly).Add(vec(offset, 0, 0))
	normal, ok := poly[1].Sub(poly[0]).Cross(poly[2].Sub(poly[0])).Normalize()
	if !ok {
		t.Fatalf("slab face is degenerate")
	}
	u, ok := poly[0].Sub(origin).Normalize()
	if !ok {
		t.Fatalf("slab face has no in-plane axis")
	}
	u, _ = u.Sub(normal.Scale(u.Dot(normal))).Normalize()
	v := normal.Cross(u)
	s := planeSketch(t, w, origin, u, v)
	local := make([]pt, 0, len(poly))
	for _, q := range poly {
		d := q.Add(vec(offset, 0, 0)).Sub(origin)
		local = append(local, pt{d.Dot(u), d.Dot(v)})
	}
	return s, closedLoop(t, s, local)
}

// stepSpiralSlice builds the cross-section segments the eight cut planes leave and
// drops the apex-side scrap.
//
// Substitution, and what it costs: the slabs are built directly between the
// consecutive section planes rather than by splitting one lofted tooth, because this
// evaluator performs no split; and every slab is then laid apart along the shaft
// axis, because two slabs that share a cut face report unsupported_pair_contact. The
// readings are unchanged by either. THE COST IS THE SPLIT ITSELF: the proof does not
// show the evaluator dividing one tooth into nine pieces, so the runtime guard — if
// the body is still in one piece the offset sign was wrong, retry once with the
// opposite sign and raise if it is still one piece — reaches Fusion untested.
func stepSpiralSlice(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 2)
	base := pt{}

	pieces := make([]*decad.Body, 0, sliceCount+1)
	spacing := 3 * sp.span
	for i, sl := range f.slabs(sp) {
		heel := f.sectionOf(g, outline, sl.heelScale, 1, 0, base)
		toe := f.sectionOf(g, outline, sl.toeScale, 1, 0, base)
		pieces = append(pieces, buildSlab(t, doc, w, heel, toe, float64(i)*spacing))
	}
	// The apex-side scrap below the toe, which step F removes.
	last := f.slabs(sp)[sliceCount-1]
	scrapHeel := f.sectionOf(g, outline, last.toeScale, 1, 0, base)
	scrapToe := f.sectionOf(g, outline, scrapScale, 1, 0, base)
	pieces = append(pieces, buildSlab(t, doc, w, scrapHeel, scrapToe, float64(sliceCount)*spacing))
	return pieces
}

func assertSpiralSlice(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)

	if len(bodies) != sliceCount+1 {
		t.Fatalf("%s: the slice left %d piece(s); eight planes cut the tooth into nine",
			g.label, len(bodies))
	}
	// After dropping the scrap, segments MUST be non-empty. An empty list is what
	// makes the crown crash with "max() iterable argument is empty" far from the
	// cause, which is why the slice asserts its own piece count here rather than three
	// steps later.
	segments := f.slabs(sp)
	if len(segments) == 0 {
		t.Fatalf("%s: dropping the apex scrap left no segments", g.label)
	}

	// The apex-most piece is the scrap, and it is the one dropped. Every other piece
	// sits further out along the element than it does.
	scrapEnd := f.slabs(sp)[sliceCount-1].toeScale * sp.parentDistance
	for i, sl := range segments {
		if sl.toeScale*sp.parentDistance < scrapEnd-1e-9 {
			t.Errorf("%s: segment %d reaches further toward the apex than the scrap", g.label, i)
		}
	}

	// Where the eight land: the first sits span/6 inside the HEEL and none of them
	// lies past it, the sixth lands at the toe, and the last two sit span/6 and
	// 2*span/6 PAST the toe — the two segments beyond the toe are what the toe cone
	// trims away.
	offsets := slabOffsets(sp)
	if len(offsets) != sliceCount {
		t.Fatalf("%s: %d cut planes, want exactly %d — the count is not user-configurable",
			g.label, len(offsets), sliceCount)
	}
	if offsets[0] <= 0 {
		t.Errorf("%s: the first cut plane sits at offset %.6f; the parent plane is already the "+
			"heel end, so there is no heel overshoot to give", g.label, offsets[0])
	}
	toeOffset := sp.rHeel - sp.rToe
	if math.Abs(offsets[5]-toeOffset) > 1e-9 {
		t.Errorf("%s: the sixth plane sits at %.6f, want the toe at %.6f",
			g.label, offsets[5], toeOffset)
	}
	for i := 6; i < sliceCount; i++ {
		if offsets[i] <= toeOffset {
			t.Errorf("%s: plane %d at %.6f does not reach past the toe at %.6f",
				g.label, i, offsets[i], toeOffset)
		}
	}

	// Every piece is a body the gate has already verified; what this adds is that no
	// piece is empty and each spans a positive height along the element.
	for i, b := range bodies {
		vol, err := b.Volume()
		if err != nil {
			t.Fatalf("%s: piece %d volume: %v", g.label, i, err)
		}
		if vol.Value.Base() <= 0 {
			t.Errorf("%s: piece %d encloses no volume", g.label, i)
		}
	}
}

// stepSpiralTwist rotates each segment about the SHAFT AXIS so the tooth follows the
// trace, centred on R_mean so the mid-face section stays unrotated — that section
// then meshes exactly like the straight tooth, which is what the pinion's zero mesh
// nudge depends on.
//
// ⚠️ The twist is keyed on the segment's HEEL-FACE cone distance, never on its
// centroid: the loft samples each segment's heel face, so that face is what must
// land at the right azimuth, and centroid-keying leaves the loft's mid-face section
// rotated by half a segment.
//
// ⚠️ A slab's heel face is the face whose centroid has the GREATEST distAlong,
// searched across ALL of the slab's faces with NO surface-type filter. A sliced slab
// is bounded by a mix of the two planar cut faces and ruled side faces, and a type
// filter can pick the wrong face or miss the cut face, which makes the loft fail
// with ASM_NOT_ALL_SECTIONS_MEET.
func stepSpiralTwist(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 2)
	base := pt{}

	pieces := make([]*decad.Body, 0, sliceCount)
	spacing := 3 * sp.span
	for i, sl := range f.slabs(sp) {
		ang := sp.twistOf(f, g, outline, sl)
		heel := f.sectionOf(g, outline, sl.heelScale, 1, ang, base)
		toe := f.sectionOf(g, outline, sl.toeScale, 1, ang, base)
		pieces = append(pieces, buildSlab(t, doc, w, heel, toe, float64(i)*spacing))
	}
	return pieces
}

// twistOf is one segment's linear share of the total toe->heel twist, keyed to the
// cone distance of its heel face.
func (sp spiral) twistOf(f figure, g member, outline []pt, sl slab) float64 {
	heel := f.sectionOf(g, outline, sl.heelScale, 1, 0, pt{})
	return sp.twistAt(sp.distAlong(centroid(heel)))
}

// twistAt is the twist law itself: a linear share of the total, centred on R_mean so
// the mid-face section stays unrotated.
func (sp spiral) twistAt(r float64) float64 {
	return -sp.handSign * sp.total * (sp.rMean - r) / sp.span
}

func assertSpiralTwist(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	outline := f.toothOutline(g, 2)
	segments := f.slabs(sp)

	if len(bodies) != len(segments) {
		t.Fatalf("%s: %d twisted segment(s), want %d", g.label, len(bodies), len(segments))
	}
	spacing := 3 * sp.span
	for i, sl := range segments {
		ang := sp.twistOf(f, g, outline, sl)
		want := centroid(f.sectionOf(g, outline, (sl.heelScale+sl.toeScale)/2, 1, ang, pt{}))
		centre, err := bodies[i].Centroid()
		if err != nil {
			t.Fatalf("%s: segment %d centroid: %v", g.label, i, err)
		}
		got := centre.Value.Sub(vec(float64(i)*spacing, 0, 0))
		gotAz := math.Atan2(got.Z, got.Y)
		wantAz := math.Atan2(want.Z, want.Y)
		if d := math.Abs(math.Mod(gotAz-wantAz+3*math.Pi, 2*math.Pi) - math.Pi); d > 1e-6 {
			t.Errorf("%s: segment %d sits at azimuth %.9f rad, want %.9f (its share of the "+
				"%.9f rad total)", g.label, i, gotAz, wantAz, sp.total)
		}
	}

	// The twist comes from the conjugate crown-gear generation law, and the roll ratio
	// is 1/sin(gamma) on this gear's PITCH cone angle. The ROOT cone angle,
	// acos(coneVec . axisDir), is smaller by the dedendum angle and yields a twist
	// about 1.15x too large.
	rootAngle := math.Acos(sp.coneVec.Dot(sp.axisDir))
	if rootAngle >= g.gamma {
		t.Errorf("%s: the root cone angle %.6f must be below the pitch cone angle %.6f",
			g.label, rootAngle, g.gamma)
	}
	if want := math.Abs(sp.phiCrown) / math.Sin(g.gamma); rel(sp.total, want) > 1e-12 {
		t.Errorf("%s: the twist is %.9f rad, want |phi_crown| / sin(gamma) = %.9f",
			g.label, sp.total, want)
	}

	// The twist is centred on R_mean, so a section whose heel face sits there stays
	// unrotated and meshes exactly like the straight tooth — which is what lets the
	// pinion's mesh phase stay 0 by default.
	if got := sp.twistAt(sp.rMean); math.Abs(got) > 1e-12 {
		t.Errorf("%s: a face at R_mean twists by %.12f rad, want 0", g.label, got)
	}
	// The two ends each take half the total, in opposite senses.
	for _, end := range []struct {
		name string
		r    float64
	}{{"toe", sp.rToe}, {"heel", sp.rHeel}} {
		if got := math.Abs(sp.twistAt(end.r)); rel(got, sp.total/2) > 1e-12 {
			t.Errorf("%s: the %s face twists by %.9f rad, want half the %.9f rad total",
				g.label, end.name, got, sp.total)
		}
	}
	if sp.twistAt(sp.rToe)*sp.twistAt(sp.rHeel) >= 0 {
		t.Errorf("%s: the toe and heel must twist in opposite senses about the mid-face", g.label)
	}
}

// stepSpiralCrown relieves each segment except the outermost by a monotonic factor —
// full at the heel, growing smoothly toward the toe — about a point on the ROOT EDGE
// of its heel face.
//
// Substitution, and what it costs: this evaluator has no scale operation, so each
// segment is BUILT at its crowned size rather than scaled after the fact. A uniform
// scale about a base point is exactly that construction, so every reading below is
// the reading the scale would leave. THE COST IS THE OPERATION: the proof does not
// show scaleFeatures itself, nor the one activation the crown needs — scaleFeatures
// is the sole exception to never-activate, and it takes the Design OCCURRENCE's own
// activate() with Design.activateRootComponent() restoring the root in a finally, a
// Component having no activate() method at all.
func stepSpiralCrown(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 2)

	pieces := make([]*decad.Body, 0, sliceCount)
	spacing := 3 * sp.span
	for i, sl := range sp.crowned(f, g, outline) {
		heel := f.sectionOf(g, outline, sl.heelScale, sl.crown, sl.twist, sl.base)
		toe := f.sectionOf(g, outline, sl.toeScale, sl.crown, sl.twist, sl.base)
		pieces = append(pieces, buildSlab(t, doc, w, heel, toe, float64(i)*spacing))
	}
	return pieces
}

// crowned resolves each segment's twist, its heel-distance fraction and its crown
// factor. The fraction is RECOMPUTED AFTER the twist has moved the slabs, never
// reused from before it.
func (sp spiral) crowned(f figure, g member, outline []pt) []slab {
	segments := f.slabs(sp)
	for i := range segments {
		segments[i].twist = sp.twistOf(f, g, outline, segments[i])
		heel := f.sectionOf(g, outline, segments[i].heelScale, 1, segments[i].twist, pt{})
		r := sp.distAlong(centroid(heel))
		u := (sp.rHeel - r) / sp.span
		segments[i].crown = 1 - crownPerRad*(math.Abs(sp.total)/2)*u
		segments[i].base = f.rootBase(g, outline, segments[i].heelScale)
	}
	// The outermost (heel) segment is the one with the GREATEST post-twist heel-face
	// distAlong, and it is held FULL: its heel face is the loft's heel end and must
	// stay full so the heel cone trims it flush with the gear base.
	outer, best := 0, math.Inf(-1)
	for i := range segments {
		heel := f.sectionOf(g, outline, segments[i].heelScale, 1, segments[i].twist, pt{})
		if r := sp.distAlong(centroid(heel)); r > best {
			outer, best = i, r
		}
	}
	segments[outer].crown = 1
	return segments
}

func assertSpiralCrown(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	outline := f.toothOutline(g, 2)
	segments := sp.crowned(f, g, outline)

	// _CROWN_PER_RAD is a tunable class constant whose default is 0.5; 0 disables the
	// crown, and it must not be left unset.
	if crownPerRad != 0.5 {
		t.Errorf("_CROWN_PER_RAD is %.4f, want the default 0.5", crownPerRad)
	}

	heights := make([]float64, 0, len(segments))
	for i, sl := range segments {
		// A factor at or below zero is an extreme twist, and nothing may be scaled by a
		// non-positive factor.
		if sl.crown <= 0 {
			t.Fatalf("%s: segment %d's crown factor is %.6f", g.label, i, sl.crown)
		}
		bounds, err := bodies[i].Bounds()
		if err != nil {
			t.Fatalf("%s: segment %d bounds: %v", g.label, i, err)
		}
		heights = append(heights, bounds.Max.Sub(bounds.Min).Len())
	}

	// ⚠️ The relief is keyed on the monotonic heel-distance u, NEVER on |ang|, the
	// twist magnitude. |ang| is symmetric about the mid-face — maximal at BOTH ends —
	// so, because the heel slab is held full, the slab just inside the heel becomes the
	// most relieved one and dips below both its neighbours. The observed bug read
	// 0.932 on the heel-adjacent slab against 0.972 on the next slab inward. This case
	// computes both keys and requires the u-keyed one to be monotonic.
	for i := 1; i < len(segments); i++ {
		if segments[i].crown > segments[i-1].crown+1e-12 {
			t.Errorf("%s: the crown factor rises from %.6f to %.6f between segments %d and %d; "+
				"relief must grow monotonically from the held heel toward the toe",
				g.label, segments[i-1].crown, segments[i].crown, i-1, i)
		}
		if heights[i] > heights[i-1]+1e-9 {
			t.Errorf("%s: segment %d spans %.6f mm against its heelward neighbour's %.6f; the "+
				"natural cone taper must never be reversed", g.label, i, heights[i], heights[i-1])
		}
	}
	byMagnitude := make([]float64, len(segments))
	for i, sl := range segments {
		byMagnitude[i] = 1 - crownPerRad*math.Abs(sl.twist)
	}
	notch := false
	for i := 2; i < len(byMagnitude); i++ {
		if byMagnitude[i] > byMagnitude[i-1] {
			notch = true
		}
	}
	if !notch && sp.total > 1e-6 {
		t.Errorf("%s: keying the relief on |ang| produced no notch on this case, so the case "+
			"does not separate the two keys", g.label)
	}

	// Nothing reads an upper bound on u: the toe-most segment's heel face sits about
	// 7/6 of the way in before the twist and a little more after the recompute, and the
	// last cut plane's 8/6 is never a heel face at all. What the slab count rests on is
	// that structural fact, not a ceiling.
	last := segments[len(segments)-1]
	if last.crown >= 1 {
		t.Errorf("%s: the toe-most segment is not relieved (factor %.6f)", g.label, last.crown)
	}
}

// stepSpiralLoft lofts the crowned, twisted segments into the curved tooth.
//
// ⚠️ The segments are re-sorted by their heel-face cone distance HERE, AFTER the
// twist and the crown — never in the pre-twist slice order. The twist rotates each
// slab about the shaft axis, and for high-twist unequal-ratio pairs that rotation
// changes the slabs' along-cone order enough to REORDER adjacent slabs; lofting in
// the stale order assembles the cross-sections out of sequence and the crowned tooth
// comes out distorted. For equal or low-twist pairs the two orders coincide, which is
// why equal-teeth gears mesh even with the stale order while ratio pairs distort.
//
// Substitution, and what it costs: this evaluator lofts TWO sections at a time, so
// the chain is built as consecutive pairwise lofts, laid apart along the shaft axis,
// rather than as one multi-section loft. THE COST IS THE SINGLE BODY: the proof does
// not show one `{gear} Spiral Tooth` made from all the sections at once, nor the toe
// face added first to push the loft past the toe cone.
func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 2)
	order := sp.loftOrder(f, g, outline)
	segments := sp.crowned(f, g, outline)

	faces := make([][]r3.Vec, 0, len(order)+1)
	first := segments[order[0]]
	faces = append(faces, f.sectionOf(g, outline, first.toeScale, first.crown, first.twist, first.base))
	for _, i := range order {
		sl := segments[i]
		faces = append(faces, f.sectionOf(g, outline, sl.heelScale, sl.crown, sl.twist, sl.base))
	}

	pieces := make([]*decad.Body, 0, len(faces)-1)
	spacing := 3 * sp.span
	for i := 0; i+1 < len(faces); i++ {
		pieces = append(pieces, buildSlab(t, doc, w, faces[i+1], faces[i], float64(i)*spacing))
	}
	return pieces
}

// loftOrder sorts the segment indices by the post-twist cone distance of their heel
// faces, toe-most first.
func (sp spiral) loftOrder(f figure, g member, outline []pt) []int {
	segments := sp.crowned(f, g, outline)
	keys := make([]float64, len(segments))
	for i, sl := range segments {
		heel := f.sectionOf(g, outline, sl.heelScale, sl.crown, sl.twist, sl.base)
		keys[i] = sp.distAlong(centroid(heel))
	}
	order := make([]int, len(segments))
	for i := range order {
		order[i] = i
	}
	for i := 1; i < len(order); i++ {
		for j := i; j > 0 && keys[order[j]] < keys[order[j-1]]; j-- {
			order[j], order[j-1] = order[j-1], order[j]
		}
	}
	return order
}

func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	sp := newSpiral(f, g, p)
	outline := f.toothOutline(g, 2)
	order := sp.loftOrder(f, g, outline)

	if len(bodies) != len(order) {
		t.Fatalf("%s: the spiral loft left %d piece(s), want %d", g.label, len(bodies), len(order))
	}
	// The order is computed now, after the twist and the crown. Every consecutive pair
	// of sections in it runs outward along the element, so the chain is assembled in
	// sequence.
	segments := sp.crowned(f, g, outline)
	prev := math.Inf(-1)
	for _, i := range order {
		heel := f.sectionOf(g, outline, segments[i].heelScale, segments[i].crown, segments[i].twist,
			segments[i].base)
		r := sp.distAlong(centroid(heel))
		if r < prev {
			t.Fatalf("%s: the loft order is not sorted by post-twist cone distance", g.label)
		}
		prev = r
	}
	// The toe-most segment's own toe face is added FIRST, which is what pushes the
	// loft past the toe cone so the toe trim bites.
	first := segments[order[0]]
	if first.toeScale >= first.heelScale {
		t.Errorf("%s: the toe-most segment's toe face does not reach past its heel face", g.label)
	}
	for i, b := range bodies {
		vol, err := b.Volume()
		if err != nil {
			t.Fatalf("%s: loft piece %d volume: %v", g.label, i, err)
		}
		if vol.Value.Base() <= 0 {
			t.Errorf("%s: loft piece %d encloses no volume", g.label, i)
		}
	}
}
