package bevelgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// Section 3a — the spiral tooth body.
//
// This is the psi > 0 branch of the tooth-body hook. At psi = 0 the hook
// returns immediately with the straight tooth's two conical trims, which is
// stepCutConicalEnds; everything here runs only when psi > 0.
//
// SUBSTITUTION, and what it costs, for every step in this file. The Fusion
// build slices the straight tooth into slabs with eight offset planes, drops
// the apex-side scrap, rotates each remaining slab about the shaft axis, scales
// each one down about a point on its heel face's root edge, and lofts through
// the resulting faces. decad has neither a split-by-plane nor a scale feature,
// and it refuses a union whose operands share a facet plane, so each slab is
// BUILT at its station rather than cut out of a body and then moved: a slab is
// the loft between its own two cross-sections, already carrying that slab's
// twist and crown. The set of solids is the same one the
// slice-drop-rotate-scale sequence produces, and every quantity the section
// pins is read off those solids or off the numbers that place them.
//
// What the substitution cannot check is that the cut planes really do split the
// body, which is the failure step E tells the generator to retry with the
// opposite sign and then raise on.
//
// The slabs are also laid apart by slabGapFraction rather than left touching.
// A split leaves coincident faces, and decad's document verification cannot
// decide whether two bodies that share a face are disjoint or overlapping — it
// reports the pair undecided and the gate fails. The gap is a fortieth of a
// slab; every station assertion below is written against the gapped value, so
// the gap is stated rather than absorbed.
// ---------------------------------------------------------------------------

// slicePlanes is the fixed slice count section 3a step E specifies: eight
// planes, stepped toward the Apex in span/6 increments from the parent
// transverse tooth plane. The count is not user-configurable.
const slicePlanes = 8

// sliceStep is the fraction of the span each slice plane advances by.
const sliceStep = 1.0 / 6.0

// slabGapFraction is how far each slab is pulled back from its cut plane at
// both ends, as a fraction of the slab's own extent. See the note above.
const slabGapFraction = 0.025

// slab is one cross-section segment of the tooth, with the twist and crown it
// carries after steps G and H.
type slab struct {
	KLo, KHi     float64 // cone-distance fractions of its toe and heel faces
	RToeFace     float64 // cone distance of the toe face
	RHeelFace    float64 // cone distance of the heel face
	Twist        float64 // shaft-axis rotation, radians
	Crown        float64 // lengthwise crown factor
	OutermostSeg bool    // the heel slab, held full
	Scrap        bool    // the apex-side piece step F drops
}

// sliceSegments realises step E: the eight cut planes and the nine pieces they
// leave, apex-most first. Piece 0 is the long apex-side scrap below the toe.
func sliceSegments(d design, g gear, f gearFrame, tf traceFrame) []slab {
	coneLen := f.Ded.sub(f.Apex).len() // cone distance of the heel station, k = 1
	kToe := tf.RToe / coneLen
	kHeel := tf.RHeel / coneLen
	spanK := kHeel - kToe

	// The cut planes, apex-ward of the parent plane in span/6 steps. The parent
	// plane is the tooth-profile plane at the heel, so the pieces run from the
	// heel inward.
	stations := make([]float64, 0, slicePlanes+2)
	stations = append(stations, apexStubFraction, kHeel)
	for k := range slicePlanes {
		stations = append(stations, kHeel-float64(k+1)*sliceStep*spanK)
	}
	sort.Float64s(stations)

	out := make([]slab, 0, slicePlanes+1)
	for i := 0; i+1 < len(stations); i++ {
		lo, hi := stations[i], stations[i+1]
		out = append(out, slab{
			KLo: lo, KHi: hi,
			RToeFace:  lo * coneLen,
			RHeelFace: hi * coneLen,
			Crown:     1,
			Scrap:     i == 0,
		})
	}
	return out
}

// spiralSlabs realises steps F, G and H on top of the slice: the apex scrap is
// dropped, each remaining slab takes its twist, and each but the outermost
// takes its crown.
func spiralSlabs(d design, g gear, f gearFrame, tf traceFrame) []slab {
	all := sliceSegments(d, g, f, tf)

	// Step F: sort by the cone distance of the centroid and drop the apex-most
	// piece. Re-slice the list FIRST, then delete the body it names.
	sort.Slice(all, func(a, b int) bool {
		return all[a].centroid() < all[b].centroid()
	})
	out := append([]slab(nil), all[1:]...)

	// Step G: each slab's rotation is a linear share of the total, keyed to the
	// cone distance of its HEEL FACE — the exact section the step-I loft
	// samples — and centred on R_mean so the mid-face section stays unrotated.
	for i := range out {
		out[i].Twist = segmentTwist(tf, out[i].RHeelFace)
	}
	// Step H: relief grows monotonically from the held heel to the toe, keyed
	// to the heel-distance fraction u and never to the twist magnitude, which
	// is symmetric about mid-face and would notch the slab just inside the
	// heel. The outermost slab by POST-TWIST heel-face cone distance is held
	// full, because its heel face is the loft's heel end.
	outermost := 0
	for i := range out {
		if out[i].RHeelFace > out[outermost].RHeelFace {
			outermost = i
		}
	}
	for i := range out {
		if i == outermost {
			out[i].Crown = 1
			out[i].OutermostSeg = true
			continue
		}
		out[i].Crown = crownFactor(tf, out[i].RHeelFace)
	}
	return out
}

// centroid is the slab's own mid cone distance, which is where its centre of
// mass sits for a section that varies linearly along it.
func (sl slab) centroid() float64 { return (sl.RToeFace + sl.RHeelFace) / 2 }

// gapped returns the slab's stations pulled back from its cut planes at both
// ends, which is where the proof actually builds it.
func (sl slab) gapped() (lo, hi float64) {
	gap := slabGapFraction * (sl.KHi - sl.KLo)
	return sl.KLo + gap, sl.KHi - gap
}

// slabBody builds one slab as the loft between its own two cross-sections,
// both carrying that slab's twist and crown.
func slabBody(t *testing.T, doc *decad.Document, w *sketch.World,
	f gearFrame, o toothOutline, sl slab) *decad.Body {
	t.Helper()
	lo, hi := sl.gapped()
	s0, p0 := drawSection(t, w, sectionPlane(t, w, f, lo), o, lo, sl.Twist, sl.Crown, true)
	s1, p1 := drawSection(t, w, sectionPlane(t, w, f, hi), o, hi, sl.Twist, sl.Crown, true)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("slab [%.4f, %.4f]: %v", sl.KLo, sl.KHi, err)
	}
	return body
}

// spiralContext is what every step in this file resolves first.
func spiralContext(t *testing.T, p map[string]float64, what string) (
	design, gear, gearFrame, toothOutline, traceFrame) {
	t.Helper()
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if d.Psi <= 0 {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0, so the tooth-body hook returns "+
			"cut_conical_ends and never %s: that branch is stepCutConicalEnds", what)
	}
	rToe, rHeel := coneDistances(f)
	return d, g, f, newToothOutline(d, g), newTraceFrame(d, g, rToe, rHeel)
}

// buildSlabs builds one body per slab.
func buildSlabs(t *testing.T, doc *decad.Document, f gearFrame, o toothOutline, slabs []slab) []*decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, len(slabs))
	for _, sl := range slabs {
		bodies = append(bodies, slabBody(t, doc, w, f, o, sl))
	}
	return bodies
}

// ---------------------------------------------------------------------------
// Step E — slice.
// ---------------------------------------------------------------------------

// stepSliceToothSlabs splits the uncut Apex-to-heel tooth body into
// cross-section slabs with eight planes perpendicular to the cone element.
//
// The first cut plane is the parent transverse tooth plane offset toward the
// Apex by span/6, and the rest step further apex-ward in span/6 increments. The
// sign is chosen per gear so the offsets move toward the Apex, and the slice
// MUST actually split: still one piece after the loop means the sign was wrong
// or the parent plane sits outside the tooth's span, and the generator retries
// once with the opposite sign and then raises.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepSliceToothSlabs, assertSliceToothSlabs) -->
func stepSliceToothSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, _, f, o, tf := spiralContext(t, p, "slices anything")
	_ = d
	return buildSlabs(t, doc, f, o, sliceSegments(d, gearOf(d, p), f, tf))
}

func assertSliceToothSlabs(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, g, f, _, tf := spiralContext(t, p, "slices anything")
	pieces := sliceSegments(d, g, f, tf)

	// Eight planes leave nine pieces, and the slice has to have HAPPENED: one
	// piece means nothing was cut.
	if len(pieces) != slicePlanes+1 {
		t.Fatalf("%s slice: %d pieces from %d planes, want %d",
			g.Label, len(pieces), slicePlanes, slicePlanes+1)
	}
	if len(bodies) != len(pieces) {
		t.Fatalf("%s slice: built %d bodies for %d pieces", g.Label, len(bodies), len(pieces))
	}
	if len(pieces) < 2 {
		t.Fatalf("%s slice produced %d piece(s): the cut planes missed the tooth",
			g.Label, len(pieces))
	}

	// The eight cut planes are span/6 apart, and the first is one span/6 in
	// from the parent plane at the heel.
	coneLen := f.Ded.sub(f.Apex).len()
	step := tf.Span * sliceStep
	requireClose(t, tf.RHeel-pieces[len(pieces)-1].RToeFace, step, 1e-6,
		"%s first cut plane is span/6 inside the parent plane", g.Label)
	for i := 1; i < len(pieces); i++ {
		requireClose(t, pieces[i].RHeelFace-pieces[i].RToeFace, step, 1e-6,
			"%s piece %d spans one slice step", g.Label, i)
	}
	// Every plane sits apex-ward of the parent plane, never past it.
	for i, piece := range pieces {
		if piece.RHeelFace > tf.RHeel+1e-9 {
			t.Fatalf("%s piece %d reaches cone distance %.6f, past the parent plane at %.6f",
				g.Label, i, piece.RHeelFace, tf.RHeel)
		}
	}
	// The apex-most piece is the long scrap: it runs from the substituted apex
	// stub out to the last cut plane and is longer than any cross-section slab.
	if !pieces[0].Scrap {
		t.Fatalf("%s the apex-most piece is not the one marked as scrap", g.Label)
	}
	for i := 1; i < len(pieces); i++ {
		if pieces[0].RHeelFace-pieces[0].RToeFace <= pieces[i].RHeelFace-pieces[i].RToeFace {
			t.Fatalf("%s apex scrap is not longer than slab %d", g.Label, i)
		}
	}

	// Each body spans its own gapped station pair, so the pieces stack in cone
	// order and none overlaps its neighbour.
	assertSlabStations(t, g, f, coneLen, bodies, pieces)
}

// assertSlabStations checks each body against the slab it was built for.
func assertSlabStations(t *testing.T, g gear, f gearFrame, coneLen float64,
	bodies []*decad.Body, slabs []slab) {
	t.Helper()
	for i, body := range bodies {
		bounds, err := body.Bounds()
		if err != nil {
			t.Fatalf("%s slab %d bounds: %v", g.Label, i, err)
		}
		lo, hi := slabs[i].gapped()
		requireClose(t, bounds.Min.Z, lo*f.Ded.X, 1e-4, "%s slab %d toe station", g.Label, i)
		requireClose(t, bounds.Max.Z, hi*f.Ded.X, 1e-4, "%s slab %d heel station", g.Label, i)
		if i > 0 && slabs[i].KLo < slabs[i-1].KHi-1e-9 {
			t.Fatalf("%s slab %d starts inside slab %d", g.Label, i, i-1)
		}
	}
}

// gearOf is sideOf's gear half, for the callers that already have the frame.
func gearOf(d design, p map[string]float64) gear {
	g, _ := sideOf(d, p)
	return g
}

// ---------------------------------------------------------------------------
// Step F — order the pieces and drop the apex scrap.
// ---------------------------------------------------------------------------

// stepDropApexScrap sorts the sliced pieces by the cone distance of their
// centroid and removes the apex-most one, the long scrap below the toe.
//
// The list is RE-SLICED before the body is deleted — segments = segments[1:]
// and only then removeFeatures.add(scrap) — and what remains must be non-empty.
// An empty list here is the slice having failed in step E, and it has to raise
// now rather than surfacing later as an empty max() inside the crown.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepDropApexScrap, assertDropApexScrap) -->
func stepDropApexScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, g, f, o, tf := spiralContext(t, p, "drops any scrap")
	// The whole set is built so the sort has something to sort, exactly as the
	// slice leaves it; the scrap is then left out of what this step returns.
	pieces := sliceSegments(d, g, f, tf)
	sort.Slice(pieces, func(a, b int) bool { return pieces[a].centroid() < pieces[b].centroid() })
	all := buildSlabs(t, doc, f, o, pieces)
	kept := all[1:]
	if len(kept) == 0 {
		t.Fatalf("%s: no segments remain after dropping the scrap", g.Label)
	}
	return kept
}

func assertDropApexScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, g, f, _, tf := spiralContext(t, p, "drops any scrap")
	pieces := sliceSegments(d, g, f, tf)
	sort.Slice(pieces, func(a, b int) bool { return pieces[a].centroid() < pieces[b].centroid() })

	// Sorting by centroid cone distance puts the scrap first: it is the piece
	// the slice left below the toe.
	if !pieces[0].Scrap {
		t.Fatalf("%s the apex-most piece by centroid is not the scrap", g.Label)
	}
	for i := 1; i < len(pieces); i++ {
		if pieces[i].Scrap {
			t.Fatalf("%s the scrap sorted to position %d, not first", g.Label, i)
		}
	}
	// What remains is non-empty and is every piece but the scrap.
	if len(bodies) != len(pieces)-1 {
		t.Fatalf("%s: kept %d segments, want %d", g.Label, len(bodies), len(pieces)-1)
	}
	if len(bodies) == 0 {
		t.Fatalf("%s: segments is empty after the drop, so the slice failed", g.Label)
	}
	coneLen := f.Ded.sub(f.Apex).len()
	assertSlabStations(t, g, f, coneLen, bodies, pieces[1:])

	// The kept segments start at the toe, not at the apex: the scrap is exactly
	// the material the toe trim would have had to remove.
	requireClose(t, pieces[1].RToeFace, tf.RHeel-float64(slicePlanes)*sliceStep*tf.Span, 1e-6,
		"%s the kept segments start at the last cut plane", g.Label)
}

// ---------------------------------------------------------------------------
// Step G — twist.
// ---------------------------------------------------------------------------

// stepTwistSlabs rotates each kept segment about the shaft axis so the tooth
// follows the trace, centred on R_mean so the mid-face section stays unrotated.
//
// The rotation is applied as a free move by a Matrix3D.setToRotation about the
// shaft axis through the Apex, and each segment's angle is a linear share of
// the total keyed to the cone distance of its HEEL FACE — the exact section the
// step-I loft samples. Keying on the centroid instead leaves the loft's
// mid-face section rotated by half a segment and overlapping.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepTwistSlabs, assertTwistSlabs) -->
func stepTwistSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, g, f, o, tf := spiralContext(t, p, "twists anything")
	slabs := spiralSlabs(d, g, f, tf)
	// The crown belongs to the next step, so every slab is still full here.
	for i := range slabs {
		slabs[i].Crown = 1
	}
	return buildSlabs(t, doc, f, o, slabs)
}

func assertTwistSlabs(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, g, f, o, tf := spiralContext(t, p, "twists anything")
	slabs := spiralSlabs(d, g, f, tf)
	if len(bodies) != len(slabs) {
		t.Fatalf("%s twist: %d bodies for %d segments", g.Label, len(bodies), len(slabs))
	}

	// The twist is centred on R_mean, so a section at the mean cone distance is
	// unrotated; the two ends carry equal and opposite shares; and the total is
	// the crown-gear roll ratio's.
	requireClose(t, segmentTwist(tf, tf.RMean), 0, tightTol,
		"%s mid-face section is unrotated", g.Label)
	requireClose(t, math.Abs(segmentTwist(tf, tf.RToe)-segmentTwist(tf, tf.RHeel)),
		tf.Total, slackTol, "%s toe-to-heel twist", g.Label)
	requireClose(t, tf.Total, math.Abs(tf.PhiCrown)/math.Sin(g.Gamma), tightTol,
		"%s twist from the PITCH cone angle, not the root cone angle", g.Label)
	// The root cone angle would give a materially different answer, which is
	// why the two must not be confused.
	rootAngle := math.Atan2(f.Ded.Y, f.Ded.X)
	if math.Abs(rootAngle-g.Gamma) < 1e-6 {
		t.Fatalf("%s root and pitch cone angles coincide, so this case cannot tell them apart",
			g.Label)
	}

	// Monotone in the heel-face cone distance: the tooth follows one arc rather
	// than folding back on itself.
	for i := 1; i < len(slabs); i++ {
		if (slabs[i].Twist-slabs[i-1].Twist)*(slabs[1].Twist-slabs[0].Twist) <= 0 {
			t.Fatalf("%s slab twists are not monotone at segment %d", g.Label, i)
		}
	}
	// And each is the linear share its heel face asks for, keyed there and not
	// at its centroid: for a slab off mid-face the two differ by half a step.
	for i, sl := range slabs {
		requireClose(t, sl.Twist, segmentTwist(tf, sl.RHeelFace), tightTol,
			"%s slab %d twist keyed to its heel face", g.Label, i)
		if math.Abs(sl.RHeelFace-tf.RMean) > tf.Span*sliceStep {
			if math.Abs(sl.Twist-segmentTwist(tf, sl.centroid())) < tightTol {
				t.Fatalf("%s slab %d: heel-face and centroid keying agree, so this case "+
					"cannot tell them apart", g.Label, i)
			}
		}
	}

	// The rotation is rigid: it moves each slab's cross-section round the shaft
	// axis without changing its size, so every slab still reaches its own
	// UNCROWNED tip and root radii. The crown is the next step's; a slab that
	// lost tip here would mean the twist had scaled it.
	full := append([]slab(nil), slabs...)
	for i := range full {
		full[i].Crown = 1
	}
	assertSlabRadii(t, g, f, o, bodies, full)
}

// assertSlabRadii checks each body's smallest and largest distance from the
// shaft axis against the section it was built from. It is what makes the crown
// observable in the SOLID: a crowned slab keeps its root radius and loses tip.
func assertSlabRadii(t *testing.T, g gear, f gearFrame, o toothOutline,
	bodies []*decad.Body, slabs []slab) {
	t.Helper()
	for i, body := range bodies {
		lo, hi := slabs[i].gapped()
		minR, maxR := math.Inf(1), math.Inf(-1)
		for _, v := range body.Vertices() {
			pos := v.Position().Value
			r := math.Hypot(pos.X, pos.Y)
			minR = math.Min(minR, r)
			maxR = math.Max(maxR, r)
		}
		// The root radius is the toe face's, and the crown never touches it.
		_, _, rootLo, _ := o.section(lo, slabs[i].Twist, slabs[i].Crown)
		// The tip radius is the heel face's, scaled by this slab's crown.
		_, _, _, tipHi := o.section(hi, slabs[i].Twist, slabs[i].Crown)
		requireClose(t, minR, rootLo, 1e-4, "%s slab %d root radius", g.Label, i)
		requireClose(t, maxR, tipHi, 1e-4, "%s slab %d tip radius", g.Label, i)
	}
}

// ---------------------------------------------------------------------------
// Step H — the lengthwise crown.
// ---------------------------------------------------------------------------

// stepCrownSlabs scales every segment except the outermost down by a monotonic
// factor, full at the heel and growing smoothly toward the toe, about a sketch
// point on the ROOT edge of that segment's heel face.
//
// The anchor is the whole point. scaleFeatures shrinks uniformly toward its
// base point, so a base point at the heel face's CENTROID, at mid tooth height,
// pulls the tooth's root edge upward by (1 - factor) times half the tooth
// height: the tooth stops seating on the gear body's root cone, floats above
// the base, and the Combine-Join leaves a gap. Anchored on the root instead,
// the root edge stays on the seating cone and only the tip is relieved. That is
// what the radius assertions below read off the solids.
//
// decad has no scale feature, so each crowned slab is built at its reduced
// size; the anchoring shows up as the root radius being untouched while the tip
// radius scales, which is exactly the invariant a uniform scale about a root
// point has.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepCrownSlabs, assertCrownSlabs) -->
func stepCrownSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, g, f, o, tf := spiralContext(t, p, "crowns anything")
	return buildSlabs(t, doc, f, o, spiralSlabs(d, g, f, tf))
}

func assertCrownSlabs(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, g, f, o, tf := spiralContext(t, p, "crowns anything")
	slabs := spiralSlabs(d, g, f, tf)
	if len(bodies) != len(slabs) {
		t.Fatalf("%s crown: %d bodies for %d segments", g.Label, len(bodies), len(slabs))
	}

	// The outermost slab by post-twist heel-face cone distance is held FULL: its
	// heel face is the loft's heel end and the heel cone trims it flush.
	heelIdx := 0
	for i := range slabs {
		if slabs[i].RHeelFace > slabs[heelIdx].RHeelFace {
			heelIdx = i
		}
	}
	if !slabs[heelIdx].OutermostSeg || slabs[heelIdx].Crown != 1 {
		t.Fatalf("%s outermost slab is not held full: crown %.6f",
			g.Label, slabs[heelIdx].Crown)
	}
	// Every factor is positive; a non-positive one raises rather than scaling.
	for i, sl := range slabs {
		if sl.Crown <= 0 {
			t.Fatalf("%s slab %d has a non-positive crown factor %.6f", g.Label, i, sl.Crown)
		}
	}
	// Sorted heel to toe the factors FALL, monotonically. Keying the relief on
	// the twist magnitude instead is symmetric about mid-face, so the slab just
	// inside the held heel would come out the most relieved and notch the
	// taper: measured on the case that exposed it, 0.932 against 0.972 for the
	// slab further in.
	order := make([]int, len(slabs))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		return slabs[order[a]].RHeelFace > slabs[order[b]].RHeelFace
	})
	for i := 1; i < len(order); i++ {
		if slabs[order[i]].Crown > slabs[order[i-1]].Crown+tightTol {
			t.Fatalf("%s crown is not monotone heel to toe: %.6f after %.6f",
				g.Label, slabs[order[i]].Crown, slabs[order[i-1]].Crown)
		}
	}
	// The maximum relief now sits at the TOE, with the magnitude the old
	// per-end peak had.
	toe := slabs[order[len(order)-1]]
	requireClose(t, toe.Crown,
		1-crownPerRad*(math.Abs(tf.Total)/2)*((tf.RHeel-toe.RHeelFace)/tf.Span),
		slackTol, "%s toe-most crown factor", g.Label)
	if !(toe.Crown < 1) {
		t.Fatalf("%s toe-most slab was not relieved at all: crown %.6f", g.Label, toe.Crown)
	}

	// Read off the solids: a crowned slab keeps its ROOT radius and loses tip.
	// A centroid-anchored scale would have moved both.
	assertSlabRadii(t, g, f, o, bodies, slabs)
	uncrowned := spiralSlabs(d, g, f, tf)
	for i := range uncrowned {
		uncrowned[i].Crown = 1
	}
	for i, sl := range slabs {
		if sl.OutermostSeg {
			continue
		}
		lo, hi := sl.gapped()
		_, _, rootFull, tipFull := uncrowned[i].sectionRadii(o, lo, hi)
		_, _, rootCrowned, tipCrowned := sl.sectionRadii(o, lo, hi)
		requireClose(t, rootCrowned, rootFull, tightTol,
			"%s slab %d root radius is untouched by the crown", g.Label, i)
		if !(tipCrowned < tipFull) {
			t.Fatalf("%s slab %d tip radius %.6f was not relieved below %.6f",
				g.Label, i, tipCrowned, tipFull)
		}
	}
}

// sectionRadii returns this slab's root and tip radii at its two gapped
// stations, under its own crown factor.
func (sl slab) sectionRadii(o toothOutline, lo, hi float64) (loRoot, loTip, rootAtLo, tipAtHi float64) {
	_, _, rootLo, tipLo := o.section(lo, sl.Twist, sl.Crown)
	_, _, _, tipHi := o.section(hi, sl.Twist, sl.Crown)
	return rootLo, tipLo, rootLo, tipHi
}

// ---------------------------------------------------------------------------
// Step I — loft the curved tooth.
// ---------------------------------------------------------------------------

// stepLoftSpiralTooth lofts the twisted, crowned slabs into the curved tooth.
//
// The order is recomputed HERE, after the twist and the crown, never reused
// from the pre-twist slice: the rotation changes the slabs' along-cone order
// for high-twist unequal-ratio pairs, and lofting in the stale order assembles
// the cross-sections out of sequence and distorts the tooth. The loft takes the
// toe-most segment's toe-facing face first, so it reaches past the toe cone,
// then each segment's heel-facing face in that order, the last reaching past
// the heel cone.
//
// SUBSTITUTION, and what it costs. Fusion builds ONE body from nine sections.
// decad's loft takes two, so the same sections are lofted pairwise, and the
// bands are returned SEPARATELY rather than joined: joining them means unioning
// a boolean result with the next band across a face the two share, which decad
// refuses — measured, the chain gets three or six bands in before the operands'
// facets graze within the chord tolerance without provably crossing. So the
// bands are laid apart, and what they would have shared is asserted from their
// own geometry: consecutive bands are built from the same station at the same
// twist and crown, so the face each would present to the next is the same face,
// and the assertion below checks that station by station against the order the
// step is required to loft in. What the substitution cannot check is that
// Fusion's nine-section loft closes into one lump.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, g, f, o, tf := spiralContext(t, p, "lofts through slab faces")
	slabs := spiralSlabs(d, g, f, tf)
	order := loftOrder(slabs)

	w := sketch.NewWorld()
	bands := make([]*decad.Body, 0, len(order))
	for i := range order {
		sl := slabs[order[i]]
		lo, hi := sl.gapped()
		s0, p0 := drawSection(t, w, sectionPlane(t, w, f, lo), o, lo, sl.Twist, sl.Crown, true)
		s1, p1 := drawSection(t, w, sectionPlane(t, w, f, hi), o, hi, sl.Twist, sl.Crown, true)
		band, err := doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("%s spiral loft band %d: %v", g.Label, i, err)
		}
		bands = append(bands, band)
	}
	return bands
}

// loftOrder sorts the segments by their POST-TWIST heel-face cone distance,
// which is the order section 3a step I requires and which can differ from the
// slice order for a high-twist ratio pair.
func loftOrder(slabs []slab) []int {
	order := make([]int, len(slabs))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		return slabs[order[a]].RHeelFace < slabs[order[b]].RHeelFace
	})
	return order
}

func assertLoftSpiralTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, g, f, _, tf := spiralContext(t, p, "lofts through slab faces")
	slabs := spiralSlabs(d, g, f, tf)
	order := loftOrder(slabs)

	if len(bodies) != len(order) {
		t.Fatalf("%s spiral tooth: got %d loft bands, want %d", g.Label, len(bodies), len(order))
	}
	// Every band is a sound single-lump solid, which the gate has checked. What
	// is left is the loft's own contract: the bands run in the POST-TWIST
	// order, and each starts where the previous one ended.
	for i, body := range bodies {
		bounds, err := body.Bounds()
		if err != nil {
			t.Fatalf("%s spiral band %d bounds: %v", g.Label, i, err)
		}
		sl := slabs[order[i]]
		lo, hi := sl.gapped()
		requireClose(t, bounds.Min.Z, lo*f.Ded.X, 1e-4, "%s spiral band %d toe station", g.Label, i)
		requireClose(t, bounds.Max.Z, hi*f.Ded.X, 1e-4, "%s spiral band %d heel station", g.Label, i)
		if i > 0 {
			requireClose(t, sl.KLo, slabs[order[i-1]].KHi, tightTol,
				"%s spiral band %d starts where band %d ended", g.Label, i, i-1)
		}
	}

	// The post-twist order is what step I requires, sorted by heel-face cone
	// distance and never by the pre-twist slice order.
	for i := 1; i < len(order); i++ {
		if slabs[order[i]].RHeelFace <= slabs[order[i-1]].RHeelFace {
			t.Fatalf("%s loft order is not sorted by post-twist heel-face cone distance at %d",
				g.Label, i)
		}
	}

	// The curved tooth is genuinely curved: its mid-face section is unrotated
	// while its ends are not, so the two ends sit at different azimuths.
	if math.Abs(slabs[order[0]].Twist-slabs[order[len(order)-1]].Twist) < 1e-6 {
		t.Fatalf("%s spiral tooth came out with no twist at all", g.Label)
	}
}
