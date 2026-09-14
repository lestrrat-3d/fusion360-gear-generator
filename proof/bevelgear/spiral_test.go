// This file holds the spiral branch's solid steps — the ones §3a puts in place
// of the straight tooth's two conical trims when the Mean Spiral Angle is above
// zero: the slab slice, the apex-scrap drop, the twist, the lengthwise crown and
// the spiral loft. At psi = 0 the hook returns immediately with the straight
// tooth's flush trim and none of this runs.
//
// FOUR SUBSTITUTIONS RUN THROUGH ALL OF THEM, and each is recorded again beside
// the step it bites.
//
//  1. THE SPLIT. decad cannot split a Loft, and every slab is one. So the slabs
//     are BUILT at the stations the cut planes would have left them at, rather
//     than cut out of one tooth. What that drops is the evaluator's own division
//     — including the spec's retry-with-the-opposite-sign guard, which only a
//     real cut can miss.
//  2. THE PLANES' TILT. §3a's cut planes stand perpendicular to the CONE
//     ELEMENT. A loft here takes two parallel sections, so the slabs are bounded
//     by planes perpendicular to the SHAFT AXIS, placed at the stations that
//     carry the cut planes' own cone distances. The spacing and every key the
//     twist and the crown read are therefore the real ones — an
//     element-perpendicular plane gives every point on it the same cone distance
//     — and what the substitution drops is the lean of the slab end faces.
//  3. THE SCALE FEATURE. decad has no scale, so a crowned slab is built with its
//     sections already scaled about the base point rather than scaled after the
//     fact. The base point is the same one: the midpoint of the heel face's two
//     ROOT corners, never the heel face's centroid.
//  4. THE MULTI-SECTION LOFT. decad's Loft takes exactly two profiles, so the
//     spiral loft through nine faces is built as the consecutive pairs and laid
//     apart. What that drops is the single body; what it keeps is the ORDER, and
//     the order is the whole subject of step I.
//
// AND ONE READING THE HARNESS DOES NOT PUBLISH. Steps G, H and I all key on a
// slab's heel face, defined as the face whose CENTROID has the greatest cone
// distance, searched across all of the slab's faces with no surface-type filter.
// decad publishes no per-face centroid — there is Body.Centroid and no
// Face.Centroid — so the proof computes each end face's centroid from the
// section polygon it built that face from. The all-faces-no-filter part of the
// rule is therefore NOT exercised here: a proof that hands itself the right face
// cannot catch a module that filters on PlaneSurfaceType and picks the wrong
// one, which is the defect that makes the step-I loft fail with
// ASM_NOT_ALL_SECTIONS_MEET.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// sliceCount is the fixed number of cut planes §3a step E uses. It is not user
// configurable, and the offsets are sign*(k+1)*span/6 for k = 0 to 7, so eight
// planes leave nine pieces.
const sliceCount = 8

// sliceStep is the offset between two cut planes, span/6, measured along the
// cone element.
func (t spiralTrace) sliceStep() float64 { return t.span / 6 }

// slabPlan is where one gear's slabs sit, in CONE DISTANCE — the distance from
// the apex measured along the root cone element — because that is the quantity
// §3a's cut planes are spaced in and the quantity the twist and the crown are
// keyed on.
//
// Working in cone distance is also what makes the slab end faces' readings
// faithful under this file's axis-perpendicular substitution. A plane
// PERPENDICULAR TO THE ELEMENT, which is what §3a cuts with, gives every point
// on it the same cone distance, so the face's centroid carries the plane's own
// value whatever else the substitution changes.
type slabPlan struct {
	coneDists []float64 // len sliceCount+2: the apex end, the eight cuts, the parent plane
	trace     spiralTrace
	rootGamma float64
	// parentCone is the parent transverse tooth plane's own cone distance. The
	// plane stands square to the PITCH line through the tooth centre K', and the
	// element meets it at exactly |Apex->Ded| — the dedendum corner's own cone
	// distance — because K' sits on the shaft axis at R / cos(gamma) and the
	// element makes the dedendum angle with the pitch line.
	parentCone   float64
	toothStation float64
}

// station maps a cone distance to the axial station this proof builds its
// sections at, so that the tooth's heel section lands where the apex loft put
// it. It is the one place the substituted frame and the real one are tied
// together.
func (p slabPlan) station(coneDist float64) float64 {
	return coneDist * p.toothStation / p.parentCone
}

// planSlabs works out the nine pieces the eight cut planes leave.
//
// The first cut plane is the parent transverse tooth plane offset toward the
// apex by span/6, and the sign that makes it point apex-ward is decided by
// testing (apex - planeOrigin) . normal — here, by the cone distance falling
// rather than rising. Then seven more at span/6 each.
func (d bevelDesign) planSlabs(which float64) slabPlan {
	tr := d.newSpiralTrace(which)
	side := d.side(which)
	plan := slabPlan{
		trace:        tr,
		rootGamma:    side.rootGamma,
		parentCone:   d.rootCone,
		toothStation: d.toothCentreStation(which),
	}
	step := tr.sliceStep()
	plan.coneDists = append(plan.coneDists, apexShrink*d.rootCone)
	for k := sliceCount - 1; k >= 0; k-- {
		plan.coneDists = append(plan.coneDists, d.rootCone-float64(k+1)*step)
	}
	plan.coneDists = append(plan.coneDists, d.rootCone)
	return plan
}

// heelFaceReading is what a slab's heel face publishes: the cone distance steps
// G, H and I all sort and scale on, and the radius of its root and tip corners,
// which is where the crown's base point goes and what the relief is read
// against.
type heelFaceReading struct {
	station    float64
	coneDist   float64
	rootRadius float64
	tipRadius  float64
	// rootSeat is where the crown's base point sits: on the heel face's ROOT
	// edge, at the tooth's own symmetry axis.
	//
	// In Fusion the base point is the midpoint of the heel face's two ROOT
	// corners — of its vertices, the two with the smallest perpendicular distance
	// to the shaft axis, the tip corners being the farthest. The proof's section
	// carries its root boundary as a symmetric run of chords rather than as one
	// straight edge between two corners, so the same point is named here as the
	// root boundary's own crossing of the symmetry axis, which is the chorded
	// face's midpoint of that edge.
	rootSeat  float64
	centroidR float64
}

// readEndFace computes one slab end face's reading. Its cone distance is the cut
// plane's own, which every point on an element-perpendicular plane shares; its
// root and tip radii come from the section polygon the slab was built from,
// because decad publishes no per-face centroid and no per-face extent. See the
// file comment for what that costs.
func (d bevelDesign) readEndFace(p slabPlan, which, coneDist float64) heelFaceReading {
	station := p.station(coneDist)
	outline := toothOutlinePoints(d, which, station/p.toothStation)
	// The polygon's own area centroid.
	area, cx, cy := 0.0, 0.0, 0.0
	for i := range outline {
		a, b := outline[i], outline[(i+1)%len(outline)]
		cross := a.x*b.y - b.x*a.y
		area += cross
		cx += (a.x + b.x) * cross
		cy += (a.y + b.y) * cross
	}
	area /= 2
	cx /= 6 * area
	cy /= 6 * area
	root, tip := math.Inf(1), 0.0
	for _, q := range outline {
		r := q.len()
		root = math.Min(root, r)
		tip = math.Max(tip, r)
	}
	return heelFaceReading{
		station:    station,
		coneDist:   coneDist,
		rootRadius: root,
		tipRadius:  tip,
		rootSeat:   root,
		centroidR:  math.Hypot(cx, cy),
	}
}

// spiralSlab is one cross-section slab: the two stations it spans, the crown
// factor it carries and the shaft-axis rotation it takes.
type spiralSlab struct {
	toeCone, heelCone       float64
	toeStation, heelStation float64
	heel                    heelFaceReading
	angle                   float64
	crown                   float64
}

// buildSlabs is the whole per-gear slab set, after the apex scrap has been
// dropped, with each slab's twist and crown already worked out.
func (d bevelDesign) buildSlabs(which float64) (slabPlan, []spiralSlab) {
	p := d.planSlabs(which)
	slabs := make([]spiralSlab, 0, len(p.coneDists)-1)
	for i := 0; i+1 < len(p.coneDists); i++ {
		slabs = append(slabs, spiralSlab{
			toeCone: p.coneDists[i], heelCone: p.coneDists[i+1],
			toeStation: p.station(p.coneDists[i]), heelStation: p.station(p.coneDists[i+1]),
		})
	}
	// Step F: sort by cone distance and drop the apex-most piece, the long scrap
	// below the toe. The slice list is re-sliced FIRST and the scrap deleted
	// after, which is what keeps the remaining segments reachable.
	slabs = slabs[1:]
	for i := range slabs {
		slabs[i].heel = d.readEndFace(p, which, slabs[i].heelCone)
		slabs[i].angle = p.trace.segmentAngle(slabs[i].heel.coneDist)
	}
	// Step H: the crown is keyed on the heel distance u, recomputed AFTER the
	// twist, and the outermost (heel) slab is held full.
	outermost := 0
	for i := range slabs {
		if slabs[i].heel.coneDist > slabs[outermost].heel.coneDist {
			outermost = i
		}
	}
	for i := range slabs {
		if i == outermost {
			slabs[i].crown = 1
			continue
		}
		slabs[i].crown = p.trace.crownFactor(slabs[i].heel.coneDist)
	}
	return p, slabs
}

// slabBody builds one slab as a loft between its two sections, with the twist
// and the crown already applied and the whole thing lifted clear of its
// neighbours.
//
// The crown is a uniform scale about a base point on the heel face's ROOT edge,
// so the heel face is untouched and every station below it is pulled toward it.
// Anchoring on the CENTROID instead lifts the tooth's root off the gear body's
// seating cone by (1 - factor) times half the tooth height, and the Combine-Join
// then leaves a visible gap.
func (d bevelDesign) slabBody(t *testing.T, doc *decad.Document, which float64,
	p slabPlan, s spiralSlab, lift float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(station float64) (*sketch.Sketch, *sketch.Profile) {
		// The crown's base point sits on the heel face, at the midpoint of its two
		// root corners. A uniform scale about it keeps every line through it fixed,
		// so the root edge stays on the seating cone while the tip is relieved.
		baseX := s.heel.rootSeat
		baseZ := s.heelStation
		k := station / p.toothStation
		outline := toothOutlinePoints(d, which, k)
		scaled := make([]xy, len(outline))
		for i, q := range outline {
			rotated := xy{
				q.x*math.Cos(s.angle) - q.y*math.Sin(s.angle),
				q.x*math.Sin(s.angle) + q.y*math.Cos(s.angle),
			}
			// The base point turns with the slab, so the scale is taken about the
			// rotated root-edge midpoint.
			bx := baseX * math.Cos(s.angle)
			by := baseX * math.Sin(s.angle)
			scaled[i] = xy{bx + s.crown*(rotated.x-bx), by + s.crown*(rotated.y-by)}
		}
		z := baseZ + s.crown*(station-baseZ) + lift
		plane := w.XY()
		if z != 0 {
			var err error
			if plane, err = w.CreateOffsetPlane(w.XY(), z); err != nil {
				t.Fatalf("slab section plane at %.6f: %v", z, err)
			}
		}
		sk, err := w.CreateSketch(plane)
		if err != nil {
			t.Fatalf("slab section sketch: %v", err)
		}
		pts := make([]*sketch.Point, len(scaled))
		for i, q := range scaled {
			pts[i] = sk.CreatePoint(q.x, q.y)
		}
		for i := range pts {
			sk.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		solveHere(t, sk)
		regions := sk.Profiles()
		if len(regions) != 1 {
			t.Fatalf("slab section at %.6f holds %d regions, want 1", z, len(regions))
		}
		return sk, regions[0]
	}
	s0, p0 := section(s.toeStation)
	s1, p1 := section(s.heelStation)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("build the slab from %.6f to %.6f: %v", s.toeStation, s.heelStation, err)
	}
	return body
}

// ---------------------------------------------------------------- S16 slice

// stepSliceTooth splits the uncut apex-to-heel tooth into cross-section slabs
// with planes perpendicular to the cone element, spanning a touch past toe and
// heel, on the fixed eight-plane scheme.
//
// ⚠️ THE SLICE MUST ACTUALLY SPLIT THE TOOTH. If the body is still in one piece
// after the cut loop the offset sign was wrong or the parent plane sits outside
// the tooth's span, and the module retries once with the opposite sign and then
// raises a self-diagnosing error naming the gear, the final piece count, the
// span and the sign tried. Returning one piece is what makes step F drop that
// piece as the apex scrap, leaving no segments at all, and the crown then dies
// with `max() iterable argument is empty` far from the cause.
//
// The proof BUILDS the pieces rather than cutting them, so it cannot miss with a
// wrong sign; what it does check is that the scheme the sign feeds produces
// pieces at all, in the right number, in a strictly rising order of station, and
// that the offsets really are span/6 apart and really run toward the apex.
func stepSliceTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan := d.planSlabs(which)
	sep := d.bandSeparation(which)
	bodies := make([]*decad.Body, 0, len(plan.coneDists)-1)
	for i := 0; i+1 < len(plan.coneDists); i++ {
		slab := spiralSlab{
			toeCone: plan.coneDists[i], heelCone: plan.coneDists[i+1],
			toeStation:  plan.station(plan.coneDists[i]),
			heelStation: plan.station(plan.coneDists[i+1]),
			crown:       1,
		}
		slab.heel = d.readEndFace(plan, which, slab.heelCone)
		bodies = append(bodies, d.slabBody(t, doc, which, plan, slab, float64(i)*sep))
	}
	return bodies
}

func assertSliceTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan := d.planSlabs(which)
	tr := plan.trace
	side := d.side(which)

	if len(bodies) < 2 {
		t.Fatalf("the slice left %d piece(s); a slice that does not split the tooth is the failure "+
			"the retry and then the raise exist for", len(bodies))
	}
	if want := sliceCount + 1; len(bodies) != want {
		t.Errorf("the slice left %d pieces, want %d — eight cut planes and the piece beyond each end",
			len(bodies), want)
	}
	// The cut stations are strictly rising toward the heel, and every step is the
	// same span/6, projected onto the shaft.
	step := tr.sliceStep()
	for i := 1; i+1 < len(plan.coneDists); i++ {
		if plan.coneDists[i] >= plan.coneDists[i+1] {
			t.Errorf("cut %d is at cone distance %.6f and cut %d at %.6f; the cuts must run toward the heel",
				i, plan.coneDists[i], i+1, plan.coneDists[i+1])
		}
		if i+2 < len(plan.coneDists) {
			if got := plan.coneDists[i+1] - plan.coneDists[i]; math.Abs(got-step) > 1e-6 {
				t.Errorf("cuts %d and %d are %.6f mm apart, want span/6 along the element, %.6f",
					i, i+1, got, step)
			}
		}
	}
	// The first cut plane is the parent tooth plane offset TOWARD THE APEX by one
	// step, which is the sign choice the module makes by testing the parent
	// plane's normal against the apex direction.
	first := plan.coneDists[len(plan.coneDists)-2]
	if got := plan.parentCone - first; math.Abs(got-step) > 1e-6 {
		t.Errorf("the first cut sits %.6f mm from the parent tooth plane, want one span/6 step "+
			"toward the apex, %.6f", got, step)
	}
	if first >= plan.parentCone {
		t.Error("the first cut plane was offset away from the apex; the sign is inverted")
	}
	// The parent transverse tooth plane meets the root element at |Apex->Ded|,
	// which is what ties the cut stations to the §2 lattice.
	if math.Abs(plan.parentCone-d.rootCone) > 1e-9 {
		t.Errorf("the parent tooth plane's cone distance is %.9f mm, want |Apex->Ded| %.9f",
			plan.parentCone, d.rootCone)
	}
	_ = side
	// The span is positive, which is the frame guard: a heel nearer the apex than
	// the toe silently inverts the whole spiral.
	if tr.span <= 0 {
		t.Errorf("the face span is %.6f mm; a non-positive span means the toe and heel are swapped "+
			"and the cutter-arc direction, the slice direction and the twist all invert", tr.span)
	}
}

// ---------------------------------------------------------------- S17 drop the apex scrap

// stepDropApexScrap sorts the pieces by the cone distance of their centroid and
// removes the first — the long apex-side scrap below the toe — keeping the rest
// as the working segments.
//
// The order matters: the list is re-sliced FIRST and the scrap deleted after,
// because a removed body cannot then be read. After the drop the segments must
// be non-empty; an empty list means the slice failed and the twist and the crown
// both assume at least one segment.
func stepDropApexScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	sep := d.bandSeparation(which)
	bodies := make([]*decad.Body, 0, len(slabs))
	for i, s := range slabs {
		s.angle, s.crown = 0, 1
		bodies = append(bodies, d.slabBody(t, doc, which, plan, s, float64(i)*sep))
	}
	return bodies
}

func assertDropApexScrap(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	if len(slabs) == 0 {
		t.Fatal("no segments survived the scrap drop; the slice failed and the twist and crown " +
			"both assume at least one segment")
	}
	if len(bodies) != len(slabs) {
		t.Fatalf("the step returned %d bodies for %d segments", len(bodies), len(slabs))
	}
	if want := sliceCount; len(slabs) != want {
		t.Errorf("%d segments survived the drop, want %d — the nine pieces less the apex scrap",
			len(slabs), want)
	}
	// The dropped piece really was the apex-most, and it really was the long one:
	// it runs from the tooth's apex end up to the first cut, which is further than
	// any single span/6 step.
	scrap := plan.coneDists[1] - plan.coneDists[0]
	shortest := math.Inf(1)
	for _, s := range slabs {
		shortest = math.Min(shortest, s.heelCone-s.toeCone)
	}
	if scrap <= shortest {
		t.Errorf("the dropped scrap spans %.6f mm and the shortest kept segment %.6f; the scrap is "+
			"the long apex-side piece", scrap, shortest)
	}
	if slabs[0].toeCone != plan.coneDists[1] {
		t.Errorf("the first kept segment starts at cone distance %.6f, want the first cut at %.6f; "+
			"the apex-most piece was not the one dropped", slabs[0].toeCone, plan.coneDists[1])
	}
}

// ---------------------------------------------------------------- S18 twist

// stepTwistSegments rotates each segment about the shaft axis so the tooth
// follows the trace, centred on R_mean so the mid-face section stays unrotated —
// which is what lets the pinion mesh with no extra phase.
//
// The rotation is keyed on the segment's HEEL-FACE cone distance and never on
// its centroid: the step-I loft samples the heel face, so that face is what has
// to land at the right azimuth, and centroid keying leaves the loft's mid-face
// section rotated by half a segment.
func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	sep := d.bandSeparation(which)
	bodies := make([]*decad.Body, 0, len(slabs))
	for i, s := range slabs {
		s.crown = 1
		bodies = append(bodies, d.slabBody(t, doc, which, plan, s, float64(i)*sep))
	}
	return bodies
}

func assertTwistSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	tr := plan.trace
	side := d.side(which)
	if len(bodies) != len(slabs) {
		t.Fatalf("the twist returned %d bodies for %d segments", len(bodies), len(slabs))
	}

	// Each segment turned by its own linear share, read off the body.
	for i, s := range slabs {
		scratch := decad.New()
		flat := spiralSlab{toeStation: s.toeStation, heelStation: s.heelStation, heel: s.heel, crown: 1}
		untwisted := d.slabBody(t, &*scratch, which, plan, flat, 0)
		want := s.angle
		got := azimuthOf(t, bodies[i]) - azimuthOf(t, untwisted)
		if angleGap(got, want) > 1e-7 {
			t.Errorf("segment %d turned %.9f rad, want its linear share %.9f", i, got, want)
		}
	}

	// The twist is centred on R_mean: the share is zero exactly where the heel
	// face sits at the mean cone distance, and the segment nearest it turns least.
	nearest := 0
	for i := range slabs {
		if math.Abs(slabs[i].heel.coneDist-tr.rMean) < math.Abs(slabs[nearest].heel.coneDist-tr.rMean) {
			nearest = i
		}
	}
	for i := range slabs {
		if math.Abs(slabs[i].angle) < math.Abs(slabs[nearest].angle)-1e-12 {
			t.Errorf("segment %d turns less than the mid-face segment %d; the twist is not centred "+
				"on R_mean", i, nearest)
		}
	}

	// The toe-to-heel twist is the crown-gear law's, |phi_crown| / sin(gamma),
	// with gamma the PITCH cone angle. The root cone angle is what
	// acos(coneVec . axisDir) measures and it inflates the twist; for a 17-tooth
	// pinion that is about 14 degrees against the pitch's 29.
	phiCrown := math.Abs(math.Atan2(tr.heelY, tr.heelX) - math.Atan2(tr.toeY, tr.toeX))
	if got, want := tr.twist, phiCrown/math.Sin(side.gamma); math.Abs(got-want) > 1e-12 {
		t.Errorf("the toe-to-heel twist is %.12f rad, want |phi_crown| / sin(pitch cone angle) %.12f",
			got, want)
	}
	if math.Abs(side.gamma-side.rootGamma) > 1e-9 {
		rootLaw := phiCrown / math.Sin(side.rootGamma)
		if math.Abs(tr.twist-rootLaw) < 1e-12 {
			t.Error("the twist was taken with the root cone angle; the roll ratio is 1 / sin of the " +
				"PITCH cone angle")
		}
	}

	// Flipping the hand mirrors every share and changes nothing else, which is
	// what makes an equal-teeth pair's two traces exact mirror images.
	mirror := d.with(func(c *bevelDesign) { c.hand = -c.hand })
	_, mirrored := mirror.buildSlabs(which)
	for i := range slabs {
		if math.Abs(slabs[i].angle+mirrored[i].angle) > 1e-9 {
			t.Errorf("segment %d turns %.9f rad and its opposite hand %.9f; the two hands must be "+
				"mirror images", i, slabs[i].angle, mirrored[i].angle)
		}
	}
}

// ---------------------------------------------------------------- S19 crown

// stepCrownSegments applies the lengthwise crown: every segment except the
// outermost is scaled down by a monotonic factor — full at the heel, growing
// toward the toe — about a sketch point on the ROOT edge of its heel face.
func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	sep := d.bandSeparation(which)
	bodies := make([]*decad.Body, 0, len(slabs))
	for i, s := range slabs {
		bodies = append(bodies, d.slabBody(t, doc, which, plan, s, float64(i)*sep))
	}
	return bodies
}

func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	tr := plan.trace
	if len(bodies) != len(slabs) {
		t.Fatalf("the crown returned %d bodies for %d segments", len(bodies), len(slabs))
	}

	// The outermost segment — the one with the greatest post-twist heel-face cone
	// distance — is held FULL, because its heel face is the loft's heel end and
	// the heel cone trims it flush with the gear base.
	outermost := 0
	for i := range slabs {
		if slabs[i].heel.coneDist > slabs[outermost].heel.coneDist {
			outermost = i
		}
	}
	if slabs[outermost].crown != 1 {
		t.Errorf("the outermost segment is scaled by %.9f, want it held full", slabs[outermost].crown)
	}

	// The relief is keyed on the MONOTONIC heel distance u and never on the twist
	// magnitude, which is symmetric about mid-face: keyed on |ang| the
	// heel-adjacent slab becomes the most relieved one and dips below both its
	// neighbours, reversing the heel-to-toe taper. Measured on the bug this rule
	// came from, that slab came out at 0.932 against 0.972 for the next one in.
	for i := range slabs {
		if i == outermost {
			continue
		}
		u := (tr.rHeel - slabs[i].heel.coneDist) / tr.span
		want := 1 - crownPerRad*(math.Abs(tr.twist)/2)*u
		if math.Abs(slabs[i].crown-want) > 1e-12 {
			t.Errorf("segment %d is scaled by %.12f, want 1 - %.2f * |total|/2 * u = %.12f",
				i, slabs[i].crown, crownPerRad, want)
		}
		if slabs[i].crown <= 0 {
			t.Errorf("segment %d is scaled by %.9f; a non-positive factor has to raise rather than scale",
				i, slabs[i].crown)
		}
	}
	// Monotonic from the held heel to the toe: sorted by heel distance, the
	// factors never rise going inward.
	order := slabOrder(slabs)
	for i := 1; i < len(order); i++ {
		outer, inner := slabs[order[i]], slabs[order[i-1]]
		if inner.crown > outer.crown+1e-12 {
			t.Errorf("segment %d is scaled by %.9f and the one outside it by %.9f; the relief has to "+
				"grow monotonically from the heel to the toe", order[i-1], inner.crown, outer.crown)
		}
	}

	// ⚠️ THE BASE POINT IS ON THE ROOT EDGE, NOT THE HEEL FACE'S CENTROID. A
	// uniform scale keeps every line through its base point fixed, so a root-edge
	// base keeps the root edge on the seating cone while the tip is relieved.
	// Anchored on the centroid instead, the root lifts by (1 - factor) times half
	// the tooth height and the Combine-Join leaves a gap.
	for i, s := range slabs {
		if i == outermost {
			continue
		}
		station := s.heelStation + float64(i)*d.bandSeparation(which)
		if !seatsAt(t, bodies[i], station, s.heel.rootSeat) {
			t.Errorf("segment %d's heel face carries no vertex at %.9f mm after the crown; the "+
				"scale base has lifted off the root edge", i, s.heel.rootSeat)
		}
		gotTip := reachOuterAt(t, bodies[i], station)
		if gotTip >= s.heel.tipRadius-1e-9 {
			t.Errorf("segment %d's heel face still reaches %.6f mm, want it relieved below the "+
				"uncrowned %.6f", i, gotTip, s.heel.tipRadius)
		}
		// The choice of base point is not academic: anchored on the heel face's
		// CENTROID instead, the root would lift by (1 - factor) times the centroid's
		// own height above the root edge, and the Combine-Join would leave a gap.
		if lift := (1 - s.crown) * (s.heel.centroidR - s.heel.rootSeat); lift <= 0 {
			t.Errorf("segment %d: a centroid-anchored scale would lift the root by %.9f mm; a "+
				"non-positive figure means the two base points cannot be told apart here", i, lift)
		}
	}
}

// slabOrder is the segments' indices sorted by their heel face's cone distance,
// toe first.
func slabOrder(slabs []spiralSlab) []int {
	order := make([]int, len(slabs))
	for i := range order {
		order[i] = i
	}
	for i := 1; i < len(order); i++ {
		for j := i; j > 0 && slabs[order[j]].heel.coneDist < slabs[order[j-1]].heel.coneDist; j-- {
			order[j], order[j-1] = order[j-1], order[j]
		}
	}
	return order
}

// seatsAt reports whether the face a body has at one station still carries a
// vertex at the given distance from the shaft axis. A uniform scale leaves its
// own base point exactly where it was, so this is how the proof reads that the
// root edge has not lifted off the seating cone.
func seatsAt(t *testing.T, body *decad.Body, station, radius float64) bool {
	t.Helper()
	found := false
	for _, v := range body.Vertices() {
		q := v.Position().Value
		if math.Abs(q.Z-station) > 1e-6 {
			continue
		}
		if math.Abs(math.Hypot(q.X, q.Y)-radius) < 1e-9 {
			found = true
		}
	}
	return found
}

func reachOuterAt(t *testing.T, body *decad.Body, station float64) float64 {
	t.Helper()
	best := 0.0
	for _, v := range body.Vertices() {
		q := v.Position().Value
		if math.Abs(q.Z-station) > 1e-6 {
			continue
		}
		best = math.Max(best, math.Hypot(q.X, q.Y))
	}
	return best
}

// ---------------------------------------------------------------- S20 spiral loft

// stepLoftSpiralTooth lofts the curved tooth through the segments' faces: first
// the toe-most segment's apex-side face, so the loft pushes past the toe cone and
// the toe trim bites, then each segment's heel-facing face in order.
//
// ⚠️ THE ORDER IS RECOMPUTED HERE, AFTER THE TWIST AND THE CROWN. The twist
// rotates each slab about the shaft axis, and for a high-twist unequal-ratio pair
// that rotation moves the slabs' cone distances enough to reorder adjacent ones;
// lofting in the stale pre-twist order assembles the cross-sections out of
// sequence and the crowned tooth comes out distorted, which is the single thing
// that makes a ratio pair fail while an equal-teeth pair looks fine.
//
// decad's Loft takes exactly two profiles, so the nine-section loft is built as
// its consecutive pairs and laid apart. The cost is the single body; what is kept
// is the section ORDER, which is the whole subject of this step.
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	plan, slabs := d.buildSlabs(which)
	order := slabOrder(slabs)
	sep := d.bandSeparation(which)
	bodies := make([]*decad.Body, 0, len(order))
	for i := 1; i < len(order); i++ {
		previous, current := slabs[order[i-1]], slabs[order[i]]
		joint := spiralSlab{
			toeStation:  previous.heelStation,
			heelStation: current.heelStation,
			heel:        current.heel,
			angle:       current.angle,
			crown:       current.crown,
		}
		bodies = append(bodies, d.slabBody(t, doc, which, plan, joint, float64(i)*sep))
	}
	return bodies
}

func assertLoftSpiralTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	_, slabs := d.buildSlabs(which)
	order := slabOrder(slabs)
	if len(bodies) != len(order)-1 {
		t.Fatalf("the spiral loft produced %d pieces for %d sections", len(bodies), len(order))
	}
	// The loft runs through the sections in strictly rising cone distance, which
	// is what the order is for.
	for i := 1; i < len(order); i++ {
		if slabs[order[i]].heel.coneDist <= slabs[order[i-1]].heel.coneDist {
			t.Errorf("section %d sits at cone distance %.6f and section %d at %.6f; the loft order "+
				"has to rise", i, slabs[order[i]].heel.coneDist, i-1, slabs[order[i-1]].heel.coneDist)
		}
	}
	// The first section is the TOE-most segment's apex-side face, which is what
	// pushes the loft past the toe cone so the toe trim bites.
	toe := order[0]
	for i := range slabs {
		if slabs[i].toeStation < slabs[toe].toeStation {
			t.Errorf("the loft starts at segment %d but segment %d reaches further toward the apex",
				toe, i)
		}
	}
	// WHAT THIS CANNOT REACH. Where the twist is small the post-twist order and
	// the pre-twist slice order coincide, which is why an equal-teeth pair meshes
	// even with the stale order. The proof builds the order from the post-twist
	// readings, so it shows the order it uses is the post-twist one and never that
	// a module using the stale one would be caught. That distinction needs a real
	// split, whose piece list carries the slice order with it.
	if len(bodies) == 0 {
		t.Error("the spiral loft built nothing")
	}
}
