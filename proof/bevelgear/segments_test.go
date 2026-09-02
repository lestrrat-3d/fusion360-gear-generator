package bevelgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// THE SPIRAL CHAIN, BUILT SECTION BY SECTION.
//
// Steps 16 to 20 slice one tooth into slabs, drop the apex scrap, twist each
// slab, crown all but the outermost, and loft through their heel faces. decad
// has no split-by-plane, no scale feature, and a Loft that takes exactly two
// sections, so none of those four verbs is available. Each step is therefore
// built rather than performed: the slabs are lofted directly between the
// sections the cut planes would have produced, the twist and the crown are
// applied to those sections before they are drawn, and the multi-section loft is
// built as the chain of bands between consecutive sections.
//
// TWO COSTS RUN THROUGH ALL FOUR. The cut planes are not quite the build's:
// step 16 cuts perpendicular to the cone element, and these sections sit on
// planes parallel to the tooth plane, which is perpendicular to the pitch line
// instead — the two differ by the dedendum angle atan(1.25 * module / R), under
// two degrees on every case in these tables. And the slabs are laid apart along
// the shaft axis so the evaluator can judge each of them, because its read-only
// interference pass cannot tessellate a loft payload; a translation along that
// axis leaves every azimuth about it, every radius from it, and every volume
// untouched, so nothing these steps assert is disturbed by it. What it does put
// out of reach is any claim about the slabs touching, which is why the tiling is
// asserted from each slab's own end sections instead.

// sliceCount is the fixed number of cross-section slabs the slice scheme
// produces, and sliceStep is the fraction of the span each cut advances by.
// Neither is user-configurable.
const (
	sliceCount = 8
	sliceStep  = 6.0
)

// segment is one slab: the two plane-distance scales its end sections sit at,
// the twist it carries, and the crown factor it was drawn at.
type segment struct {
	toeScale, heelScale float64
	twist               float64
	crown               float64
	base                r3.Vec // the point the crown was scaled about
}

// segments returns the slab scheme for one gear, apex-most first.
//
// The cut planes are the parent tooth plane offset toward the apex by one
// sliceStep of the span at a time, eight of them, so the tooth comes apart into
// eight cross-section slabs plus the long apex-side scrap below them. The scrap
// is the first entry: it is the piece step 17 drops.
//
// Its apex end is truncated at the same tenth of R the tooth loft is truncated
// at, because a loft cannot run to a point.
func segments(s spiral) []segment {
	cut := make([]float64, 0, sliceCount+1)
	for j := 0; j <= sliceCount; j++ {
		cut = append(cut, s.planeScaleAt(s.rHeelPlane()-float64(j)*s.span/sliceStep))
	}
	out := make([]segment, 0, sliceCount+1)
	out = append(out, segment{toeScale: apexTruncation, heelScale: cut[sliceCount], crown: 1})
	for j := sliceCount; j > 0; j-- {
		out = append(out, segment{toeScale: cut[j], heelScale: cut[j-1], crown: 1})
	}
	return out
}

// rHeelPlane is the cone distance the tooth plane itself stands at, which is the
// heel end of the uncut tooth.
func (s spiral) rHeelPlane() float64 { return s.q.pitchConeDist }

// planeScaleAt turns a plane distance from the apex into the section scale at
// that distance.
func (s spiral) planeScaleAt(planeDistance float64) float64 {
	return planeDistance / s.q.pitchConeDist
}

// build draws one slab and returns it, slid apart by offset.
func (g segment) build(t *testing.T, doc *decad.Document, world *sketch.World,
	f toothFrame, outline []vec2, offset float64) *decad.Body {
	t.Helper()
	toeSketch, toeProfile := f.placedSection(t, world, outline, g.toeScale, g.twist, offset,
		g.base, g.crown)
	heelSketch, heelProfile := f.placedSection(t, world, outline, g.heelScale, g.twist, offset,
		g.base, g.crown)
	body, err := doc.Loft(toeSketch, toeProfile, heelSketch, heelProfile)
	if err != nil {
		t.Fatalf("slab from scale %.6f to %.6f: %v", g.toeScale, g.heelScale, err)
	}
	return body
}

// toothOf resolves the tooth outline and frame one case's slabs are cut from.
func toothOf(t *testing.T, params map[string]float64) (spiral, toothFrame, []vec2) {
	t.Helper()
	s := spiralOf(t, params)
	virtual := s.side.virtualTeeth(s.q.module)
	d := involute.Derive(s.q.module, virtual, proxyPressureAngle)
	return s, toothFrameOf(s.q, s.side), toothSectionPoints(t, d, virtual, proxyInvoluteSteps)
}

// buildSegments draws a whole slab set, laid apart along the shaft axis.
func buildSegments(t *testing.T, doc *decad.Document, s spiral, f toothFrame, outline []vec2,
	set []segment) []*decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	out := make([]*decad.Body, 0, len(set))
	for k, g := range set {
		out = append(out, g.build(t, doc, world, f, outline, float64(k)*stride(s.q)))
	}
	return out
}

// stepSliceSegments slices the uncut tooth into cross-section slabs.
//
// SUBSTITUTE: each slab is lofted directly between the two sections its cut
// planes would have produced, rather than split off one body. See the note at
// the top of this file for what that costs.
//
// <!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSliceSegments, assertSliceSegments) -->
func stepSliceSegments(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s, f, outline := toothOf(t, params)
	return buildSegments(t, doc, s, f, outline, segments(s))
}

// assertSliceSegments checks that the slice really cut the tooth, and into what.
//
// The count is the scheme's own: eight cross-section slabs plus the apex scrap,
// and a slice that produced one piece is the failure step 16 has to retry with
// the opposite sign and then raise on. The slabs tile the tooth with no gap and
// no overlap, which is asserted from their own end sections rather than from
// their positions: consecutive slabs share a section, so the heel area of one is
// the toe area of the next.
func assertSliceSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s, f, outline := toothOf(t, params)
	set := segments(s)
	if len(bodies) != sliceCount+1 {
		t.Fatalf("%s: the slice produced %d piece(s), expected %d — %d cross-section slabs and "+
			"the apex scrap", s.side.label, len(bodies), sliceCount+1, sliceCount)
	}
	if len(bodies) < 2 {
		t.Fatalf("%s: the slice left the tooth in one piece; the cut planes missed it and the "+
			"build has to retry with the opposite sign", s.side.label)
	}

	world := sketch.NewWorld()
	area := func(scale float64) float64 {
		_, p := f.placedSection(t, world, outline, scale, 0, 0, r3.NewVec(0, 0, 0), 1)
		return p.Area
	}
	for k, g := range set {
		volume, err := bodies[k].Volume()
		if err != nil {
			t.Fatalf("%s: slab %d volume: %v", s.side.label, k, err)
		}
		a0, a1 := area(g.toeScale), area(g.heelScale)
		height := (g.heelScale - g.toeScale) * s.q.pitchConeDist
		if height <= 0 {
			t.Fatalf("%s: slab %d spans %.9f, so the cut planes are out of order",
				s.side.label, k, height)
		}
		if want := frustumVolume(height, a0, a1); !closeTo(volume.Value.Mag(), want, 1e-9) {
			t.Fatalf("%s: slab %d measures %.6f, not the %.6f its two sections bound",
				s.side.label, k, volume.Value.Mag(), want)
		}
		if k > 0 && !closeTo(g.toeScale, set[k-1].heelScale, 1e-12) {
			t.Errorf("%s: slab %d starts at scale %.9f while slab %d ended at %.9f; the slabs "+
				"tile the tooth", s.side.label, k, g.toeScale, k-1, set[k-1].heelScale)
		}
	}
	if last := set[len(set)-1]; !closeTo(last.heelScale, 1, 1e-12) {
		t.Errorf("%s: the outermost slab ends at scale %.9f, not at the tooth plane",
			s.side.label, last.heelScale)
	}
}

// stepDropScrap orders the slabs and removes the apex-side scrap.
//
// The order is by the cone distance of each piece's centroid, and the first —
// apex-most — piece is the long scrap below the toe. It is dropped by re-slicing
// the list before the removal, and what is left must be non-empty, because the
// twist and the crown both assume at least one segment.
//
// <!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepDropScrap, assertDropScrap) -->
func stepDropScrap(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s, f, outline := toothOf(t, params)
	set := segments(s)
	bodies := buildSegments(t, doc, s, f, outline, set)
	order := orderByConeDistance(t, set)
	kept := make([]*decad.Body, 0, len(bodies)-1)
	for _, k := range order[1:] {
		kept = append(kept, bodies[k])
	}
	if len(kept) == 0 {
		t.Fatalf("%s: dropping the scrap left no segments; the slice failed", s.side.label)
	}
	return kept
}

// orderByConeDistance sorts slab indices by the cone distance of their centre,
// apex-most first.
func orderByConeDistance(t *testing.T, set []segment) []int {
	t.Helper()
	order := make([]int, len(set))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		ga, gb := set[order[a]], set[order[b]]
		return ga.toeScale+ga.heelScale < gb.toeScale+gb.heelScale
	})
	return order
}

// assertDropScrap checks which piece went and what is left.
//
// The scrap is the apex-most piece and it is the long one: it runs from the
// truncated apex end all the way to the first cut, which is more than one
// sliceStep of the span, while every kept slab is exactly one. Dropping anything
// else would take a cross-section the loft needs.
func assertDropScrap(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s, _, _ := toothOf(t, params)
	set := segments(s)
	if len(bodies) != sliceCount {
		t.Fatalf("%s: dropping the scrap left %d segments, want the %d cross-section slabs",
			s.side.label, len(bodies), sliceCount)
	}
	order := orderByConeDistance(t, set)
	scrap := set[order[0]]
	if scrap.toeScale != apexTruncation {
		t.Errorf("%s: the piece dropped runs from scale %.9f, so it is not the apex-side one",
			s.side.label, scrap.toeScale)
	}
	scrapSpan := (scrap.heelScale - scrap.toeScale) * s.q.pitchConeDist
	slabSpan := s.span / sliceStep
	if scrapSpan <= slabSpan {
		t.Errorf("%s: the scrap spans %.6f against a slab's %.6f; the scrap is the long piece "+
			"below the toe", s.side.label, scrapSpan, slabSpan)
	}
	for k, body := range bodies {
		volume, err := body.Volume()
		if err != nil {
			t.Fatalf("%s: kept segment %d volume: %v", s.side.label, k, err)
		}
		if volume.Value.Mag() <= 0 {
			t.Fatalf("%s: kept segment %d has no volume", s.side.label, k)
		}
	}
}

// crowned returns the slab set with the twist and the lengthwise crown applied.
//
// The twist is step 18's linear share, keyed on each slab's heel-face cone
// distance and centred on R_mean. The crown is step 19's monotonic relief,
// keyed on the heel-distance fraction u and anchored on the ROOT edge of each
// slab's heel face — the two vertices of that face nearest the shaft axis, which
// is what keeps the tooth seated on the gear body instead of lifting it off.
func crowned(t *testing.T, s spiral, f toothFrame, outline []vec2, set []segment) []segment {
	t.Helper()
	out := make([]segment, len(set))
	copy(out, set)
	heelMost := 0
	for k := range out {
		out[k].twist = s.angleAt(out[k].heelScale * s.q.pitchConeDist)
		if out[k].heelScale > out[heelMost].heelScale {
			heelMost = k
		}
	}
	for k := range out {
		if k == heelMost {
			// The outermost slab is held full: its heel face is the loft's heel end
			// and the heel cone trims it flush with the gear base.
			out[k].crown = 1
			continue
		}
		u := (s.rHeel - out[k].heelScale*s.q.pitchConeDist) / s.span
		factor := 1 - crownPerRad*(math.Abs(s.total)/2)*u
		if factor <= 0 {
			t.Fatalf("%s: segment %d crowns to a factor of %.6f at u %.6f; a non-positive factor "+
				"is a raise, never a scale", s.side.label, k, factor, u)
		}
		out[k].crown = factor
		out[k].base = rootEdgeMidpoint(t, f, outline, out[k].heelScale)
	}
	return out
}

// crownPerRad is the tunable class constant the relief is scaled by. Zero
// disables the crown; the spec's default is a half.
const crownPerRad = 0.5

// rootEdgeMidpoint is the point the crown scale is anchored on: the midpoint of
// the two vertices of a slab's heel face that lie nearest the shaft axis.
//
// Those two are the ROOT corners — the tip corners are the farthest from the
// axis — and anchoring there is what keeps the root edge on the seating cone. A
// scale about the face's centroid would pull the root upward by half the relief
// and leave the Combine-Join a gap.
// It is taken in the frame BEFORE the twist turns the slab, because that is the
// frame the crown is applied in; the distance from the shaft axis is unchanged
// by a rotation about that axis, so the two corners it picks are the same either
// way.
func rootEdgeMidpoint(t *testing.T, f toothFrame, outline []vec2, scale float64) r3.Vec {
	t.Helper()
	pair := rootCorners(t, f, outline, scale)
	return pair[0].Add(pair[1]).Scale(0.5)
}

// stepCrownSegments applies the lengthwise crown.
//
// SUBSTITUTE: each slab's two sections are drawn already scaled about the crown
// point, rather than scaled by a scale feature after the fact. A uniform scale
// about a point carries planes to parallel planes and sections to scaled copies,
// so the drawn slab is the scaled slab; what goes unexercised is the feature
// itself, and with it the activate-and-restore dance it needs.
//
// <!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepCrownSegments, assertCrownSegments) -->
func stepCrownSegments(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s, f, outline := toothOf(t, params)
	return buildSegments(t, doc, s, f, outline, crowned(t, s, f, outline, segments(s))[1:])
}

// assertCrownSegments checks the relief's shape and its anchor.
//
// Three things. The relief grows monotonically from the held-full heel toward
// the toe, so slab heights stay ordered heel to toe and the natural cone taper
// is never reversed — keying it on the twist magnitude instead would make the
// slab just inside the heel the most relieved one and notch the tooth. The
// outermost slab is held at exactly one. And the anchor is invariant: the point
// the scale is taken about is a fixed point of it, so the root edge it sits on
// stays where it was and the tooth stays seated.
func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s, f, outline := toothOf(t, params)
	set := crowned(t, s, f, outline, segments(s))[1:]
	if len(bodies) != len(set) {
		t.Fatalf("%s: the crown left %d segments, want %d", s.side.label, len(bodies), len(set))
	}

	byHeel := make([]int, len(set))
	for i := range byHeel {
		byHeel[i] = i
	}
	sort.Slice(byHeel, func(a, b int) bool { return set[byHeel[a]].heelScale < set[byHeel[b]].heelScale })
	if last := set[byHeel[len(byHeel)-1]]; last.crown != 1 {
		t.Errorf("%s: the outermost segment crowns to %.9f; its heel face is the loft's heel end "+
			"and stays full", s.side.label, last.crown)
	}
	previous := math.Inf(-1)
	for _, k := range byHeel {
		g := set[k]
		if g.crown <= 0 {
			t.Fatalf("%s: segment %d crowns to %.9f", s.side.label, k, g.crown)
		}
		if g.crown > 1+1e-12 {
			t.Errorf("%s: segment %d crowns to %.9f; relief removes material", s.side.label, k, g.crown)
		}
		if g.crown < previous {
			t.Errorf("%s: segment %d crowns to %.9f against %.9f one step nearer the toe; the "+
				"relief grows monotonically from the held-full heel toward the toe, so the factor "+
				"only rises as the heel is approached", s.side.label, k, g.crown, previous)
		}
		previous = g.crown
	}

	// The anchor is a fixed point of its own scale, so the root edge it sits on
	// does not move. That is read back off the drawn slab: the two vertices
	// nearest the anchor still straddle it, and they are the two of that face
	// nearest the shaft axis — which is what makes the anchor a ROOT-edge
	// midpoint rather than the face centroid.
	for k, g := range set {
		if g.crown == 1 {
			continue
		}
		turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), radiansOf(g.twist))
		if err != nil {
			t.Fatalf("%s: anchor rotation: %v", s.side.label, err)
		}
		want := turn.Apply(g.base).Add(r3.NewVec(float64(k)*stride(s.q), 0, 0))
		corners := rootCorners(t, f, outline, g.heelScale)
		axis := func(p r3.Vec) float64 { return math.Hypot(p.Y, p.Z) }
		if d := math.Abs(axis(corners[0]) - axis(corners[1])); d > 1e-9 {
			t.Errorf("%s: segment %d's two root corners sit %.9f and %.9f from the shaft axis; "+
				"they are a symmetric pair", s.side.label, k, axis(corners[0]), axis(corners[1]))
		}
		for _, corner := range corners {
			// Where the uniform scale about the anchor puts this root corner.
			placed := g.base.Add(corner.Sub(g.base).Scale(g.crown))
			placed = turn.Apply(placed).Add(r3.NewVec(float64(k)*stride(s.q), 0, 0))
			if d := nearestVertexDistance(t, bodies[k], placed); d > 1e-6 {
				t.Errorf("%s: segment %d has no vertex within %.3e of the root corner the crown "+
					"puts at %v; the slab was not scaled about its root anchor",
					s.side.label, k, d, placed)
			}
		}
		if d := corners[0].Add(corners[1]).Scale(0.5).Sub(g.base).Len(); d > 1e-9 {
			t.Errorf("%s: segment %d's anchor sits %.3e off the midpoint of its own root corners",
				s.side.label, k, d)
		}
		if d := want.Sub(turn.Apply(g.base).Add(r3.NewVec(float64(k)*stride(s.q), 0, 0))).Len(); d > 1e-12 {
			t.Errorf("%s: segment %d's anchor moved by %.3e under its own scale", s.side.label, k, d)
		}
		centroid, err := bodies[k].Centroid()
		if err != nil {
			t.Fatalf("%s: crowned segment %d centroid: %v", s.side.label, k, err)
		}
		if axis(centroid.Value) <= axis(want) {
			t.Errorf("%s: segment %d's anchor sits %.9f from the shaft axis and its centroid "+
				"%.9f; anchoring on the centroid instead of the root edge is what lifts the "+
				"crowned tooth off the gear base", s.side.label, k, axis(want),
				axis(centroid.Value))
		}
	}
}

// rootCorners returns the two points of a slab's heel face nearest the shaft
// axis, in the frame before the twist — the pair the crown is anchored between.
func rootCorners(t *testing.T, f toothFrame, outline []vec2, scale float64) [2]r3.Vec {
	t.Helper()
	type corner struct {
		p r3.Vec
		d float64
	}
	corners := make([]corner, 0, len(outline))
	for _, q := range outline {
		p := f.origin.Scale(scale).Add(f.u.Scale(q.X * scale)).Add(f.v.Scale(q.Y * scale))
		corners = append(corners, corner{p, math.Hypot(p.Y, p.Z)})
	}
	sort.Slice(corners, func(a, b int) bool { return corners[a].d < corners[b].d })
	return [2]r3.Vec{corners[0].p, corners[1].p}
}

// nearestVertexDistance is how far the body's closest vertex is from p.
func nearestVertexDistance(t *testing.T, body *decad.Body, p r3.Vec) float64 {
	t.Helper()
	best := math.Inf(1)
	for _, v := range body.Vertices() {
		best = math.Min(best, v.Position().Value.Sub(p).Len())
	}
	if math.IsInf(best, 1) {
		t.Fatal("the slab carries no vertices")
	}
	return best
}

// stepLoftSpiralTooth lofts the crowned segments into the `{gear} Spiral Tooth`.
//
// SUBSTITUTE: decad's Loft takes exactly two sections, so the multi-section loft
// is built as the chain of bands between consecutive sections, laid apart. The
// chain carries the same sections in the same order, so what the step is really
// about — the ORDER, re-sorted here after the twist and the crown rather than
// reused from the slice — is exactly what it asserts. What it costs is the
// single body: the chain is not fused, so the loft's own tool-body formation
// goes unexercised.
//
// <!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s, f, outline := toothOf(t, params)
	set := crowned(t, s, f, outline, segments(s))[1:]
	order := postTwistOrder(s, set)
	world := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, len(order))
	for k := 1; k < len(order); k++ {
		a, b := set[order[k-1]], set[order[k]]
		offset := float64(k) * stride(s.q)
		fromSketch, fromProfile := f.placedSection(t, world, outline, a.heelScale, a.twist, offset,
			a.base, a.crown)
		toSketch, toProfile := f.placedSection(t, world, outline, b.heelScale, b.twist, offset,
			b.base, b.crown)
		body, err := doc.Loft(fromSketch, fromProfile, toSketch, toProfile)
		if err != nil {
			t.Fatalf("%s: loft link %d of the spiral tooth: %v", s.side.label, k, err)
		}
		bodies = append(bodies, body)
	}
	return bodies
}

// postTwistOrder sorts the segments by the cone distance of their heel face
// AFTER the twist has moved them, which is the order step 20 lofts in.
func postTwistOrder(s spiral, set []segment) []int {
	order := make([]int, len(set))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		return heelFaceConeDistance(s, set[order[a]]) < heelFaceConeDistance(s, set[order[b]])
	})
	return order
}

// heelFaceConeDistance is a slab's heel-face centre projected on the cone
// element, after the twist has turned it about the shaft axis.
//
// The twist moves it because the cone element is not the shaft axis: a rotation
// about the axis changes a point's projection on an element inclined to it. That
// is exactly why the order has to be recomputed here.
func heelFaceConeDistance(s spiral, g segment) float64 {
	centre := r3.NewVec(g.heelScale*s.q.pitchConeDist, 0, 0)
	turn := g.twist
	p := r3.NewVec(centre.X, centre.Y*math.Cos(turn)-centre.Z*math.Sin(turn),
		centre.Y*math.Sin(turn)+centre.Z*math.Cos(turn))
	return p.X*s.coneVec.X + p.Y*s.coneVec.Y
}

// assertLoftSpiralTooth checks the order the chain was built in.
//
// The loft samples each segment's heel face, so those faces have to arrive in
// increasing cone distance AFTER the twist. Lofting in the stale pre-twist order
// assembles the cross-sections out of sequence and the crowned tooth comes out
// distorted, which is the one thing that makes a ratio pair fail while an equal
// pair looks fine — so the step also reports whether the two orders differ for
// this case, rather than assuming they do.
func assertLoftSpiralTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s, f, outline := toothOf(t, params)
	set := crowned(t, s, f, outline, segments(s))[1:]
	order := postTwistOrder(s, set)
	if len(bodies) != len(order)-1 {
		t.Fatalf("%s: the chain has %d links, want one between each of the %d sections",
			s.side.label, len(bodies), len(order))
	}
	previous := math.Inf(-1)
	for _, k := range order {
		d := heelFaceConeDistance(s, set[k])
		if d <= previous {
			t.Fatalf("%s: the loft order puts a heel face at cone distance %.9f after one at "+
				"%.9f; the sections have to arrive in sequence", s.side.label, d, previous)
		}
		previous = d
	}
	stale := make([]int, len(set))
	for i := range stale {
		stale[i] = i
	}
	sort.Slice(stale, func(a, b int) bool { return set[stale[a]].heelScale < set[stale[b]].heelScale })
	same := true
	for i := range order {
		if order[i] != stale[i] {
			same = false
		}
	}
	t.Logf("%s: the post-twist order %v and the pre-twist order %v %s", s.side.label, order, stale,
		map[bool]string{true: "coincide, so this case cannot show why the re-sort is needed",
			false: "differ, which is what the re-sort exists for"}[same])
	for k, body := range bodies {
		volume, err := body.Volume()
		if err != nil {
			t.Fatalf("%s: chain link %d volume: %v", s.side.label, k, err)
		}
		if volume.Value.Mag() <= 0 {
			t.Fatalf("%s: chain link %d has no volume", s.side.label, k)
		}
	}
}
