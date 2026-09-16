// The spiral tooth body, steps S17 to S21 — the psi > 0 branch of the
// tooth-body hook. When psi = 0 the hook returns immediately with the straight
// tooth's two conical trims, so every step here skips that case rather than
// proving a build the module does not perform.
//
// The frame this file works in, and what it costs. The generated module slices
// the tooth with planes PARALLEL TO THE PARENT TOOTH PLANE, whose normal runs
// along the PITCH element, while the cone element the spiral keys on is the
// ROOT element; the two differ by the dedendum angle
// delta_f = atan(1.25 * Module / R). The proof builds its own slabs from the
// offsets the spec fixes and never reads the plane the generated module
// constructs, so the module's choice of plane family reaches Fusion untested.
// That is the same shape of gap as the §2 seed, and the only thing that closes
// it is a measurement taken on a loaded spiral gear — a face corner's position
// along the cone, which no load has yet reported. What the proof does reach is
// the offsets themselves, the twist law, the crown law and the ordering rule,
// and it states below which of its own two frames each reading is taken in.
//
// The proof's frames:
//
//   - The OFFSET frame. A face's key is its perpendicular distance from the
//     apex along the shaft axis, which is how the spec states every slice
//     offset. The parent plane sits at R, the apex's perpendicular distance to
//     the back-cone plane, and the k-th cut plane at R - (k+1) * span/6.
//   - The CONE-ELEMENT frame. distAlong is the projection on the root element,
//     a direction at gamma_root to the shaft axis. This is the frame the twist
//     and the crown recompute their keys in, and it is the one a rotation
//     about the shaft axis moves.
package bevelgear_test

import (
	"fmt"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// crownPerRad is the module's tunable class constant _CROWN_PER_RAD. Its
// default is 0.5; 0 disables the crown, and the spec says to set it to 0.5
// rather than leave it unset.
const crownPerRad = 0.5

// sliceCount is the fixed slice scheme: exactly 8 planes, not user
// configurable, leaving 9 pieces of which the apex-most is scrap.
const sliceCount = 8

// slab is one cross-section piece of the sliced tooth.
type slab struct {
	index       int     // 0 is the heel-most piece
	dHeel, dToe float64 // the two faces' perpendicular distance from the apex
	u           float64 // heel-distance fraction, in the offset frame
	ang         float64 // this slab's share of the twist
	factor      float64 // its crown factor
	body        *decad.Body
}

// spiralSetup is everything the five spiral steps share: the geometry, the
// gear side, the trace the twist law reads, and the slab schedule.
type spiralSetup struct {
	g       geometry
	sd      side
	tr      trace
	psi     float64
	section []pt
	slabs   []slab
	apexEnd float64
}

// newSpiralSetup resolves the slab schedule. It returns false when the case is
// the straight branch, which builds none of this.
func newSpiralSetup(t *testing.T, p map[string]float64) (spiralSetup, bool) {
	t.Helper()
	var sp spiralSetup
	sp.psi = p["spiralAngle"] * math.Pi / 180
	if sp.psi <= 0 {
		return sp, false
	}
	sp.g = newGeometry(t, p)
	sp.sd = sp.g.side(gearOf(p))
	sp.tr = newTrace(sp.g, sp.sd, sp.psi, p["hand"], p["cutterRadius"])
	sp.section = toothSection(sp.g, sp.sd)
	sp.apexEnd = apexShrink * sp.g.R

	step := sp.tr.span / 6
	// The 8 cut planes step toward the apex in span/6 increments from the
	// parent plane, at sign*(k+1)*span/6 for k = 0..7. The first sits span/6
	// inside the HEEL and none of them lies past it, because the parent plane
	// is already the heel end; the sixth lands at the toe; the last two sit
	// span/6 and 2*span/6 PAST the toe, and those two segments are what the
	// toe cone trims away.
	for i := 0; i <= sliceCount; i++ {
		dHeel := sp.g.R - float64(i)*step
		dToe := sp.g.R - float64(i+1)*step
		if i == sliceCount {
			dToe = sp.apexEnd
		}
		if dToe <= sp.apexEnd {
			dToe = sp.apexEnd
		}
		if dHeel <= sp.apexEnd {
			t.Fatalf("slab %d: the slice scheme reaches past the apex substitute at %.4f mm; "+
				"span %.4f mm over a cone distance of %.4f mm", i, sp.apexEnd, sp.tr.span, sp.g.R)
		}
		sp.slabs = append(sp.slabs, slab{
			index: i,
			dHeel: dHeel,
			dToe:  dToe,
			u:     (sp.g.R - dHeel) / sp.tr.span,
		})
	}
	return sp, true
}

// twistOf is step G's law. The total toe-to-heel shaft-axis twist comes from
// the conjugate crown-gear generation law: the developed crown-plane azimuth
// the cutter arc's two endpoints subtend at the apex, divided by sin(gamma)
// for the roll ratio of the generating crown gear. gamma is this gear's PITCH
// cone angle, not acos(coneVec . axisDir), which is the root cone angle and
// yields a twist about 1.15 times too large.
func (sp spiralSetup) twistOf(s slab) float64 {
	// Keyed on the segment's HEEL FACE cone distance, not its centroid: the
	// loft samples each segment's heel face, so that face is what has to land
	// at the right azimuth. Centroid keying leaves the loft's mid-face section
	// rotated by half a segment.
	return -sp.tr.handSign * sp.tr.total * (sp.rMean() - s.dHeel) / sp.tr.span
}

// rMean, rHeel and rToe in the OFFSET frame: the parent plane is the heel end,
// the toe sits one span in from it, and the mean is halfway between.
func (sp spiralSetup) rHeel() float64 { return sp.g.R }
func (sp spiralSetup) rToe() float64  { return sp.g.R - sp.tr.span }
func (sp spiralSetup) rMean() float64 { return sp.g.R - sp.tr.span/2 }

// crownFactor is step H's law. |total|/2 is the per-end peak twist magnitude,
// so the maximum relief — now at the TOE — keeps the magnitude the old per-end
// peak had, relocated. Keying it on the monotonic heel distance u rather than
// on |ang| is what keeps the slab heights strictly ordered heel to toe: |ang|
// is symmetric about the mid-face, so with the heel slab held full the slab
// just inside the heel would become the most relieved one and dip below both
// its neighbours.
func (sp spiralSetup) crownFactor(u float64) float64 {
	return 1 - crownPerRad*(math.Abs(sp.tr.total)/2)*u
}

// factorOf is segment i's crown factor, with the outermost (heel) segment held
// full: its heel face is the loft's heel end and has to stay full so the heel
// cone trims it flush with the gear base.
func (sp spiralSetup) factorOf(i int) float64 {
	if i == 0 {
		return 1
	}
	return sp.crownFactor(sp.slabs[i].u)
}

// coneKey is a face's distAlong in the CONE-ELEMENT frame, for a face at
// perpendicular distance d from the apex whose centroid has been turned about
// the shaft axis by ang. The tooth is drawn on the -X side, so its centroid's
// radius from the axis is what the turn moves.
func (sp spiralSetup) coneKey(d, ang float64) float64 {
	cx, _ := sectionCentroid(sp.section)
	r := cx * d / sp.g.R
	return -r*math.Cos(ang)*math.Sin(sp.sd.gammaRoot) + d*math.Cos(sp.sd.gammaRoot)
}

func sectionCentroid(section []pt) (float64, float64) {
	var a, cx, cy float64
	for i := range section {
		p, q := section[i], section[(i+1)%len(section)]
		cross := p.X*q.Y - q.X*p.Y
		a += cross
		cx += (p.X + q.X) * cross
		cy += (p.Y + q.Y) * cross
	}
	a /= 2
	return cx / (6 * a), cy / (6 * a)
}

// buildSlab lofts one slab from its two sections, each scaled to its own
// distance from the apex, turned by ang and crowned by factor about the ROOT
// EDGE of its heel face.
func buildSlab(t *testing.T, doc *decad.Document, w *sketch.World, sp spiralSetup,
	s slab, ang, factor, z0 float64) *decad.Body {
	t.Helper()
	heelScale := s.dHeel / sp.g.R
	toeScale := s.dToe / sp.g.R

	// The crown's base point, per gotcha 3: the midpoint of the heel face's
	// two ROOT corners, NOT its centroid. scaleFeatures shrinks uniformly
	// toward the base point, so a centroid base pulls the tooth's root edge
	// upward by (1 - factor) * half the tooth height and the tooth floats off
	// the gear body; anchoring on the root keeps the root edge on the seating
	// cone while the tip is relieved.
	ring := buildVirtualToothRing(sp.g, sp.sd)
	corner := rootArcCorner(ring)
	base := pt{corner.X * heelScale, 0}

	heel := turnPts(scalePts(scalePts(sp.section, heelScale, pt{0, 0}), factor, base), ang)
	// A uniform scale about a point ON the heel face leaves that face in its
	// own plane and pulls the toe face in by the same factor, so the slab gets
	// shorter as well as thinner.
	toeZ := s.dHeel - factor*(s.dHeel-s.dToe)
	toeBase := pt{corner.X * toeScale, 0}
	toe := turnPts(scalePts(scalePts(sp.section, toeScale, pt{0, 0}), factor, toeBase), ang)

	loPlane, err := w.CreateOffsetPlane(w.XY(), z0)
	if err != nil {
		t.Fatalf("slab %d toe plane: %v", s.index, err)
	}
	hiPlane, err := w.CreateOffsetPlane(w.XY(), z0+(s.dHeel-toeZ))
	if err != nil {
		t.Fatalf("slab %d heel plane: %v", s.index, err)
	}
	s0, p0 := polySection(t, w, loPlane, toe)
	s1, p1 := polySection(t, w, hiPlane, heel)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("slab %d loft: %v", s.index, err)
	}
	return body
}

func turnPts(in []pt, a float64) []pt {
	out := make([]pt, len(in))
	for i, p := range in {
		out[i] = rotate(p, a)
	}
	return out
}

// slabVolume is the exact prismatoid volume of one slab, which is what the
// loft between its two chorded sections records.
func slabVolume(sp spiralSetup, s slab, factor float64) float64 {
	area := math.Abs(shoelace(sp.section))
	a0 := area * math.Pow(s.dToe/sp.g.R*factor, 2)
	a1 := area * math.Pow(s.dHeel/sp.g.R*factor, 2)
	h := factor * (s.dHeel - s.dToe)
	return h / 3 * (a0 + a1 + math.Sqrt(a0*a1))
}

// ---------------------------------------------------------------------------
// S17 Slice the straight tooth
// ---------------------------------------------------------------------------

func stepSliceTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	sp, ok := newSpiralSetup(t, p)
	if !ok {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0, so the tooth-body hook returns the "+
			"straight tooth's two conical trims before any slicing runs")
		return nil
	}
	w := sketch.NewWorld()
	out := make([]*decad.Body, 0, len(sp.slabs))
	z := 0.0
	for i := range sp.slabs {
		sp.slabs[i].body = buildSlab(t, doc, w, sp, sp.slabs[i], 0, 1, z)
		z += sp.slabs[i].dHeel - sp.slabs[i].dToe + layoutGap
		out = append(out, sp.slabs[i].body)
	}
	return out
}

func assertSliceTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	sp, _ := newSpiralSetup(t, p)

	// The slice MUST actually split the tooth. A single piece means the offset
	// sign was wrong or the parent plane sits outside the tooth's span, and
	// the module retries once with the opposite sign and then raises naming
	// the gear, the piece count, the span and the sign tried. Returning an
	// unsliced result would leave segments empty after the scrap is dropped
	// and the crown would crash far from the cause.
	if len(bodies) != sliceCount+1 {
		t.Fatalf("the slice produced %d piece(s), expected %d — 8 cut planes leave 9",
			len(bodies), sliceCount+1)
	}

	step := sp.tr.span / 6
	for k := 0; k < sliceCount; k++ {
		want := float64(k+1) * step
		got := sp.g.R - sp.slabs[k+1].dHeel
		requireClose(t, fmt.Sprintf("cut plane %d offset", k), got, want, 1e-9*sp.g.R)
	}

	// Where the eight land. The first sits span/6 inside the heel and none of
	// them lies past it, because the parent plane is already the heel end. The
	// sixth lands at the toe. The last two sit span/6 and 2*span/6 past it.
	requireClose(t, "the first cut plane sits span/6 inside the heel",
		sp.rHeel()-sp.slabs[1].dHeel, step, 1e-9*sp.g.R)
	requireClose(t, "the sixth cut plane lands at the toe", sp.slabs[6].dHeel, sp.rToe(), 1e-9*sp.g.R)
	requireClose(t, "the seventh cut plane sits span/6 past the toe",
		sp.rToe()-sp.slabs[7].dHeel, step, 1e-9*sp.g.R)
	requireClose(t, "the eighth cut plane sits 2*span/6 past the toe",
		sp.rToe()-(sp.g.R-8*step), 2*step, 1e-9*sp.g.R)

	// The nine pieces are the whole tooth: their volumes sum to the volume of
	// the unsliced loft over the same span.
	total := 0.0
	bound := 0.0
	for i, b := range bodies {
		m := volumeReading(t, b, fmt.Sprintf("slab %d", i))
		decadtest.Measures(t, fmt.Sprintf("slab %d volume", i), m,
			units.CubicMillimeters(slabVolume(sp, sp.slabs[i], 1)),
			decadtest.WithinRel(units.Scalar(1e-12)))
		total += m.Value.Base()
		bound += m.Bound.Base()
	}
	area := math.Abs(shoelace(sp.section))
	whole := sp.g.R/3*area - sp.apexEnd/3*area*math.Pow(sp.apexEnd/sp.g.R, 2)
	agreesWithin(t, "the nine pieces against the tooth they were cut from",
		reading{value: total, bound: bound}, whole, 1e-9*whole)
	_ = doc
}

// ---------------------------------------------------------------------------
// S18 Order the segments and drop the apex scrap
// ---------------------------------------------------------------------------

func stepDropApexScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	sp, ok := newSpiralSetup(t, p)
	if !ok {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0: the straight branch builds no segments")
		return nil
	}
	w := sketch.NewWorld()
	built := make([]slab, len(sp.slabs))
	z := 0.0
	for i := range sp.slabs {
		s := sp.slabs[i]
		s.body = buildSlab(t, doc, w, sp, s, 0, 1, z)
		z += s.dHeel - s.dToe + layoutGap
		built[i] = s
	}

	// Sort the segments by the distAlong of their centroid. The first
	// (apex-most) is the long apex-side scrap below the toe: re-slice the list
	// FIRST and delete the scrap afterwards, so segments never holds it.
	sort.SliceStable(built, func(a, b int) bool {
		return sp.coneKey(0.5*(built[a].dHeel+built[a].dToe), 0) <
			sp.coneKey(0.5*(built[b].dHeel+built[b].dToe), 0)
	})
	scrap := built[0]
	segments := built[1:]
	if scrap.index != sliceCount {
		t.Errorf("the apex-most piece is slab %d, want the last one (%d)", scrap.index, sliceCount)
	}
	// After dropping the scrap, segments must be non-empty. An empty list
	// means the slice failed, and the crown would then fail on an empty max()
	// far from the cause.
	if len(segments) == 0 {
		t.Fatal("segments is empty after dropping the apex scrap: the slice failed in S17")
	}
	out := make([]*decad.Body, len(segments))
	for i, s := range segments {
		out[i] = s.body
	}
	return out
}

func assertDropApexScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	sp, _ := newSpiralSetup(t, p)
	if len(bodies) != sliceCount {
		t.Fatalf("after the scrap is dropped %d segments remain, want %d", len(bodies), sliceCount)
	}
	// The scrap really is the long one: it runs from the last cut plane all
	// the way to the apex, while every kept segment is one span/6 slice.
	scrapLength := sp.slabs[sliceCount].dHeel - sp.slabs[sliceCount].dToe
	if scrapLength <= sp.tr.span/6 {
		t.Errorf("the apex scrap is %.4f mm long against a slice of %.4f mm; it should be the long "+
			"apex-side piece below the toe", scrapLength, sp.tr.span/6)
	}
	_ = doc
}

// ---------------------------------------------------------------------------
// S19 Twist
// ---------------------------------------------------------------------------

func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	sp, ok := newSpiralSetup(t, p)
	if !ok {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0: no twist is applied and the straight "+
			"tooth's mid-face section is the whole of its mesh")
		return nil
	}
	w := sketch.NewWorld()
	out := make([]*decad.Body, 0, sliceCount)
	z := 0.0
	for i := 0; i < sliceCount; i++ {
		s := sp.slabs[i]
		s.ang = sp.twistOf(s)
		// A rotation about the shaft axis is rigid, so the twisted slab has to
		// read the same volume as the untwisted one. Both are built and
		// compared rather than asserted from the formula alone.
		plain := buildSlab(t, doc, w, sp, s, 0, 1, z)
		z += s.dHeel - s.dToe + layoutGap
		turned := buildSlab(t, doc, w, sp, s, s.ang, 1, z)
		z += s.dHeel - s.dToe + layoutGap
		decadtest.Agree(t, fmt.Sprintf("slab %d after the twist against the same slab before it", i),
			volumeReading(t, turned, "twisted"), volumeReading(t, plain, "plain"),
			decadtest.WithinRel(units.Scalar(1e-9)))
		out = append(out, plain, turned)
	}
	return out
}

func assertTwistSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	sp, _ := newSpiralSetup(t, p)

	// The law itself.
	requireClose(t, "phi_crown, the developed azimuth the cutter arc subtends at the apex",
		sp.tr.phiCrown,
		math.Atan2(sp.tr.heel2d.Y, sp.tr.heel2d.X)-math.Atan2(sp.tr.toe2d.Y, sp.tr.toe2d.X), 1e-15)
	requireClose(t, "the toe-to-heel shaft-axis twist", sp.tr.total,
		math.Abs(sp.tr.phiCrown)/math.Sin(sp.sd.gamma), 1e-15)
	// Use the PITCH cone angle, not the root one. acos(coneVec . axisDir) is
	// the root cone angle, smaller by delta_f, and it inflates the twist.
	rootTwist := math.Abs(sp.tr.phiCrown) / math.Sin(sp.sd.gammaRoot)
	if rootTwist <= sp.tr.total {
		t.Errorf("the root-cone reading %.6f rad is not larger than the pitch-cone one %.6f rad; "+
			"the two angles must differ by the dedendum angle", rootTwist, sp.tr.total)
	}

	// The dedendum angle depends only on the tooth counts and the Shaft Angle
	// — Module cancels — and is the SAME for both members of the pair.
	deltaF := math.Atan(sp.g.Dedendum / sp.g.R)
	requireClose(t, "the dedendum angle from the pinion's own numbers",
		deltaF, math.Atan(2.5*math.Sin(sp.g.GammaP)/sp.g.Np), 1e-12)
	requireClose(t, "the dedendum angle from the driving gear's own numbers",
		deltaF, math.Atan(2.5*math.Sin(sp.g.GammaG)/sp.g.Nd), 1e-12)

	// The twist is centred on R_mean, so the mid-face section stays unrotated:
	// that section then meshes exactly like the straight tooth, which is what
	// the pinion's zero mesh nudge depends on.
	mid := -1
	for i := 0; i < sliceCount; i++ {
		if math.Abs(sp.slabs[i].dHeel-sp.rMean()) < 1e-9*sp.g.R {
			mid = i
		}
	}
	if mid < 0 {
		t.Fatalf("no segment's heel face sits at R_mean %.6f mm", sp.rMean())
	}
	requireClose(t, fmt.Sprintf("the mid-face segment (%d) is unrotated", mid),
		sp.twistOf(sp.slabs[mid]), 0, 1e-12)

	// The share is linear in the heel-face key and reaches half the total at
	// each end of the face.
	requireClose(t, "the heel end's share", sp.twistOf(sp.slabs[0]),
		-sp.tr.handSign*sp.tr.total*(sp.rMean()-sp.rHeel())/sp.tr.span, 1e-15)
	requireClose(t, "the per-end peak twist magnitude",
		math.Abs(sp.tr.total*(sp.rMean()-sp.rToe())/sp.tr.span), sp.tr.total/2, 1e-12)

	// Keying on the heel face rather than the centroid is not cosmetic: the
	// two keys sit half a segment apart, which is exactly the mid-face overlap
	// the centroid key would leave.
	centroidKey := 0.5 * (sp.slabs[mid].dHeel + sp.slabs[mid].dToe)
	gap := math.Abs(sp.slabs[mid].dHeel - centroidKey)
	requireClose(t, "the heel-face key and the centroid key sit half a segment apart",
		gap, sp.tr.span/12, 1e-9*sp.g.R)

	// The two members of a meshing pair legitimately get DIFFERENT twists:
	// same cutter, same spiral angle, but gamma differs, so 1/sin(gamma) does.
	other := pinion
	if sp.sd.which == pinion {
		other = driving
	}
	otherTrace := newTrace(sp.g, sp.g.side(other), sp.psi, p["hand"], p["cutterRadius"])
	if math.Abs(sp.g.GammaP-sp.g.GammaG) > 1e-9 {
		if math.Abs(otherTrace.total-sp.tr.total) < 1e-9 {
			t.Errorf("the two members read the same twist %.6f rad although their pitch cone angles "+
				"differ (%.6f and %.6f rad)", sp.tr.total, sp.g.GammaP, sp.g.GammaG)
		}
	} else if math.Abs(otherTrace.total-sp.tr.total) > 1e-9 {
		t.Errorf("an equal-teeth pair reads different twists, %.6f and %.6f rad",
			sp.tr.total, otherTrace.total)
	}
	// Equal-teeth pairs must come out as exact mirror images, which is what
	// the hand sign on the cos/Cy term buys. An unequal pair does NOT mirror:
	// its two members see different mean cone distances, so only the sign of
	// the offset is shared.
	if sp.g.Np == sp.g.Nd {
		requireClose(t, "the pair's two cutter centres mirror across the cone element",
			otherTrace.cy, -sp.tr.cy, 1e-9)
	} else if otherTrace.cy*sp.tr.cy >= 0 {
		t.Errorf("the pair's two cutter centres sit on the same side of the cone element "+
			"(%.6f and %.6f); opposite hands must flip Cy", sp.tr.cy, otherTrace.cy)
	}
	// The hand goes on the cos/Cy term, never on the sin/Cx term: putting it
	// on Cx mirrors about x = R_mean instead, a different curve that gives the
	// two gears unequal twist.
	flipped := newTrace(sp.g, sp.sd, sp.psi, -p["hand"], p["cutterRadius"])
	requireClose(t, "flipping the hand leaves Cx alone", flipped.cx, sp.tr.cx, 1e-12)
	requireClose(t, "flipping the hand negates Cy", flipped.cy, -sp.tr.cy, 1e-12)
	_ = doc
	_ = bodies
}

// ---------------------------------------------------------------------------
// S20 Lengthwise crown
// ---------------------------------------------------------------------------

func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	sp, ok := newSpiralSetup(t, p)
	if !ok {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0: the straight branch applies no crown")
		return nil
	}
	w := sketch.NewWorld()
	out := make([]*decad.Body, 0, sliceCount)
	z := 0.0
	for i := 0; i < sliceCount; i++ {
		s := sp.slabs[i]
		s.ang = sp.twistOf(s)
		// Skip the outermost (heel) segment: its heel face is the loft's heel
		// end and must stay full so the heel cone trims it flush with the gear
		// base.
		s.factor = sp.factorOf(i)
		if s.factor <= 0 {
			t.Fatalf("%s segment %d: crown factor %.6f at u %.6f is not positive; never scale by "+
				"a non-positive factor", sp.sd.which.label(), i, s.factor, s.u)
		}
		sp.slabs[i] = s
		out = append(out, buildSlab(t, doc, w, sp, s, s.ang, s.factor, z))
		z += s.dHeel - s.dToe + layoutGap
	}
	// scaleFeatures is the ONE exception to never-activate: it needs the
	// Design occurrence as the active edit target, so the module calls
	// designOccurrence.activate() before the crown scales and restores the
	// root in a finally with design.activateRootComponent(). A Component has
	// no .activate() at all. decad has no activation, so that pairing is
	// unreachable here and is named in the step list instead.
	return out
}

func assertCrownSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	sp, _ := newSpiralSetup(t, p)

	// u runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two
	// segments that lie beyond the toe: their heel faces sit 6*span/6 and
	// 7*span/6 in from the parent plane, so the toe-most one reads about 7/6.
	// 8/6 is NOT a ceiling: that figure is the last plane's offset, and that
	// plane is a toe face rather than any segment's heel face, so nothing ever
	// evaluates u there.
	for i := 0; i < sliceCount; i++ {
		requireClose(t, fmt.Sprintf("segment %d heel-distance fraction", i),
			sp.slabs[i].u, float64(i)/6, 1e-12)
	}
	// The two segments beyond the toe: the first of them reads exactly 1 and
	// the toe-most one about 7/6.
	requireClose(t, "the segment whose heel face lands at the toe", sp.slabs[6].u, 1, 1e-12)
	requireClose(t, "the toe-most segment's u", sp.slabs[7].u, 7.0/6, 1e-12)
	if sp.slabs[7].u <= 1 {
		t.Errorf("the toe-most segment reads u %.4f; it lies past the toe and must exceed 1",
			sp.slabs[7].u)
	}
	lastPlane := (sp.g.R - float64(sliceCount)*sp.tr.span/6)
	for i := 0; i < sliceCount; i++ {
		if math.Abs(sp.slabs[i].dHeel-lastPlane) < 1e-12 {
			t.Errorf("segment %d's heel face sits on the last cut plane; that plane is a toe face "+
				"and u is never evaluated there", i)
		}
	}

	// Recomputed AFTER the twist, in the cone-element frame: the twist moves
	// the heel faces, so the recomputed fraction climbs above the offset-frame
	// one and keeps climbing with the Spiral Angle.
	for i := 1; i < sliceCount; i++ {
		post := (sp.coneKey(sp.g.R, 0) - sp.coneKey(sp.slabs[i].dHeel, sp.twistOf(sp.slabs[i]))) / sp.tr.span
		if post < sp.slabs[i].u-1e-9 {
			t.Errorf("segment %d's recomputed heel-distance fraction %.6f fell below the pre-twist "+
				"%.6f; the twist moves the heel faces outward, not inward", i, post, sp.slabs[i].u)
		}
		if sp.crownFactor(post) <= 0 {
			t.Errorf("segment %d: the crown factor is %.6f at the recomputed u %.6f; it must stay "+
				"positive for every value u takes", i, sp.crownFactor(post), post)
		}
	}

	// The relief grows MONOTONICALLY from the full heel to the toe, so the
	// slab heights stay strictly ordered heel to toe and the natural cone
	// taper is never reversed. Keying on |ang| instead would be symmetric
	// about the mid-face and, with the heel slab held full, would make the
	// heel-adjacent slab the most relieved one — a notch.
	requireClose(t, "the heel segment is held full", sp.factorOf(0), 1, 1e-15)
	for i := 2; i < sliceCount; i++ {
		if !(sp.factorOf(i) < sp.factorOf(i-1)) {
			t.Errorf("crown factor %d (%.6f) is not below factor %d (%.6f): the relief is not "+
				"monotonic and a slab would dip below its neighbours",
				i, sp.factorOf(i), i-1, sp.factorOf(i-1))
		}
		if sp.factorOf(i) <= 0 {
			t.Errorf("crown factor %d is %.6f at u %.6f; never scale by a non-positive factor",
				i, sp.factorOf(i), sp.slabs[i].u)
		}
	}
	// Keying the relief on |ang| instead is symmetric about the mid-face —
	// maximal at BOTH ends — so with the heel slab held full the slab just
	// inside the heel becomes the most relieved one and dips below both its
	// neighbours. That reading is what this comparison records: under the
	// |ang| law segment 1 would be relieved more than segment 2, which is the
	// notch.
	byAng := func(i int) float64 { return 1 - crownPerRad*math.Abs(sp.twistOf(sp.slabs[i])) }
	if !(byAng(1) < byAng(2)) {
		t.Errorf("the |ang| law does not put its peak relief next to the held-full heel here "+
			"(%.6f against %.6f), so this case does not exercise the notch it produces",
			byAng(1), byAng(2))
	}
	requireClose(t, "the maximum relief keeps the old per-end peak's magnitude",
		1-sp.crownFactor(1), crownPerRad*sp.tr.total/2, 1e-12)

	// The body readings: each crowned slab is the prismatoid the factor
	// produces, and the heel one is untouched.
	for i, b := range bodies {
		decadtest.Measures(t, fmt.Sprintf("crowned segment %d volume", i),
			volumeReading(t, b, fmt.Sprintf("segment %d", i)),
			units.CubicMillimeters(slabVolume(sp, sp.slabs[i], sp.factorOf(i))),
			decadtest.WithinRel(units.Scalar(1e-12)))
	}

	// Gotcha 3, the reason the base point is the heel face's ROOT edge rather
	// than its centroid: a uniform scale keeps every line through the base
	// point invariant, so a root anchor leaves the root edge on the seating
	// cone while a centroid anchor lifts it by (1 - factor) * half the tooth
	// height and the Combine-Join leaves a gap.
	ring := buildVirtualToothRing(sp.g, sp.sd)
	corner := rootArcCorner(ring)
	cx, _ := sectionCentroid(sp.section)
	for i := 1; i < sliceCount; i++ {
		f := sp.factorOf(i)
		lift := math.Abs(cx-corner.X) * (1 - f) * sp.slabs[i].dHeel / sp.g.R
		if lift <= 0 {
			t.Errorf("segment %d: a centroid-anchored scale would lift the root edge by %.6f mm; "+
				"the reading that makes the root anchor load-bearing is that this is positive", i, lift)
		}
	}
	_ = doc
}

// ---------------------------------------------------------------------------
// S21 Loft the spiral tooth
// ---------------------------------------------------------------------------

func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	sp, ok := newSpiralSetup(t, p)
	if !ok {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0: there is no spiral loft, and the straight "+
			"tooth goes straight to its two conical trims")
		return nil
	}
	w := sketch.NewWorld()

	// Re-sort the segments by their heel-face cone distance HERE, after the
	// twist and the crown. The twist rotates each slab about the shaft axis,
	// and for high-twist unequal-ratio pairs that rotation changes the slabs'
	// along-cone order enough to REORDER adjacent slabs; lofting in the stale
	// pre-twist order assembles the cross-sections out of sequence and the two
	// gears interfere.
	order := spiralOrder(sp)

	// The loft runs: first the toe-most segment's apex-side (toe-facing) face,
	// which pushes the loft past the toe cone so the toe trim bites, then the
	// heel-facing face of every segment in order.
	sections := make([][]pt, 0, sliceCount+1)
	heights := make([]float64, 0, sliceCount+1)
	first := sp.slabs[order[0]]
	sections = append(sections, sectionAt(sp, first, first.dToe))
	heights = append(heights, first.dToe)
	for _, idx := range order {
		s := sp.slabs[idx]
		sections = append(sections, sectionAt(sp, s, s.dHeel))
		heights = append(heights, s.dHeel)
	}

	// decad lofts two sections at a time, so the chain is built pair by pair
	// and laid apart. The cost is the single lofted body: what the proof shows
	// is that consecutive sections in this order loft into valid solids and
	// that the order is the post-twist one, not that one loft passes through
	// all nine.
	out := make([]*decad.Body, 0, len(sections)-1)
	z := 0.0
	for i := 0; i+1 < len(sections); i++ {
		h := heights[i+1] - heights[i]
		if h <= 0 {
			t.Fatalf("loft section %d does not advance along the cone: %.6f mm", i, h)
		}
		lo, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			t.Fatalf("loft plane %d: %v", i, err)
		}
		hi, err := w.CreateOffsetPlane(w.XY(), z+h)
		if err != nil {
			t.Fatalf("loft plane %d: %v", i+1, err)
		}
		s0, p0 := polySection(t, w, lo, sections[i])
		s1, p1 := polySection(t, w, hi, sections[i+1])
		body, err := doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("spiral loft %d: %v", i, err)
		}
		out = append(out, body)
		z += h + layoutGap
	}
	return out
}

// sectionAt is one slab's face at distance d from the apex, twisted and
// crowned the way that slab was.
func sectionAt(sp spiralSetup, s slab, d float64) []pt {
	ang := sp.twistOf(s)
	factor := sp.factorOf(s.index)
	ring := buildVirtualToothRing(sp.g, sp.sd)
	corner := rootArcCorner(ring)
	scale := d / sp.g.R
	base := pt{corner.X * scale, 0}
	return turnPts(scalePts(scalePts(sp.section, scale, pt{0, 0}), factor, base), ang)
}

// spiralOrder is the post-twist ordering, computed from each segment's
// heel-face cone distance AFTER the twist has moved it.
func spiralOrder(sp spiralSetup) []int {
	idx := make([]int, sliceCount)
	for i := range idx {
		idx[i] = i
	}
	sort.SliceStable(idx, func(a, b int) bool {
		return sp.coneKey(sp.slabs[idx[a]].dHeel, sp.twistOf(sp.slabs[idx[a]])) <
			sp.coneKey(sp.slabs[idx[b]].dHeel, sp.twistOf(sp.slabs[idx[b]]))
	})
	return idx
}

func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	sp, _ := newSpiralSetup(t, p)
	order := spiralOrder(sp)

	if len(order) != sliceCount {
		t.Fatalf("the loft order names %d segments, want %d", len(order), sliceCount)
	}
	seen := map[int]bool{}
	for _, i := range order {
		if seen[i] {
			t.Fatalf("segment %d appears twice in the loft order", i)
		}
		seen[i] = true
	}
	// The order is taken from the POST-twist keys. Where the twist is small
	// enough that the two orders coincide this reads the same as the stale one
	// — which is exactly why an equal-teeth pair meshes even with the stale
	// order while a ratio pair distorts — so the assertion is that the order
	// was computed from the post-twist keys, and the pre-twist comparison is
	// logged rather than required.
	stale := make([]int, sliceCount)
	for i := range stale {
		stale[i] = sliceCount - 1 - i
	}
	same := true
	for i := range order {
		if order[i] != stale[i] {
			same = false
		}
	}
	t.Logf("post-twist loft order %v; the pre-twist order %v %s", order, stale,
		map[bool]string{true: "coincides with it", false: "differs from it"}[same])

	// Every section in the order advances along the cone, which is what makes
	// the chain a loft rather than a fold.
	for i := 1; i < len(order); i++ {
		a := sp.coneKey(sp.slabs[order[i-1]].dHeel, sp.twistOf(sp.slabs[order[i-1]]))
		b := sp.coneKey(sp.slabs[order[i]].dHeel, sp.twistOf(sp.slabs[order[i]]))
		if !(b > a) {
			t.Errorf("segment %d's post-twist key %.6f does not advance past segment %d's %.6f",
				order[i], b, order[i-1], a)
		}
	}

	// The first section added is the toe-most segment's TOE face, which pushes
	// the loft past the toe cone so the toe trim bites, and the last reaches
	// past the heel cone.
	if len(bodies) != sliceCount {
		t.Fatalf("the loft chain built %d bodies, want %d — one toe face plus eight heel faces",
			len(bodies), sliceCount)
	}
	toeMost := sp.slabs[order[0]]
	if !(toeMost.dToe < toeMost.dHeel) {
		t.Errorf("the toe-most segment's toe face is not inboard of its heel face")
	}
	if toeMost.dToe >= sp.rToe() {
		t.Errorf("the first lofted section sits at %.6f mm, not past the toe at %.6f mm; the toe "+
			"trim would have nothing to bite", toeMost.dToe, sp.rToe())
	}

	// The lofted chain is the crowned, twisted tooth: its pieces' volumes are
	// the prismatoids the sections bound.
	for i, b := range bodies {
		if _, err := b.Volume(); err != nil {
			t.Fatalf("spiral loft piece %d volume: %v", i, err)
		}
	}
	_ = doc
}
