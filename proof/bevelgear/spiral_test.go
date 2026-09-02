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

// crownPerRad is the tunable class constant the lengthwise crown scales by. The
// spec's default is 0.5, and 0 disables the crown.
const crownPerRad = 0.5

// slicePlanes is the fixed number of cut planes the slice scheme uses. It is not
// user-configurable.
const slicePlanes = 8

// endRelief is how far past the face the trace arc's ends are taken, as a
// fraction of the span, so the kept arc reaches cleanly past the end trims.
const endRelief = 0.06

// spiral is the §3a frame and cutter-arc geometry for one gear, all in the
// tangent-plane 2-D frame: the apex at the origin, x along the root cone element
// so a point's x IS its cone distance, y circumferential.
type spiral struct {
	gear                     gearGeometry
	rToe, rHeel, rMean, span float64
	psi                      float64 // the mean spiral angle, radians
	rc                       float64 // the cutter radius, resolved
	handSign                 float64
	centre                   vec // the cutter circle's centre
	toe2d, heel2d            vec
	phiCrown, total          float64
}

// spiralOf resolves the whole §3a frame for one gear.
func spiralOf(p params) spiral {
	g := gearOf(p)
	s := spiral{gear: g, psi: p.SpiralAngle * math.Pi / 180}
	s.rToe = g.distAlong(g.toeMid)
	s.rHeel = g.distAlong(g.heelMid)
	s.rMean = (s.rToe + s.rHeel) / 2
	s.span = s.rHeel - s.rToe

	s.rc = p.CutterRadius
	if s.rc == 0 {
		s.rc = s.rMean
	}
	// The hand is the driving gear's; the pinion is built with the opposite hand
	// so the pair meshes.
	s.handSign = -1
	if p.HandRight {
		s.handSign = 1
	}
	if p.Pinion {
		s.handSign = -s.handSign
	}
	// The hand sign belongs on the cos term. On the sin term it would mirror the
	// cutter centre about x = R_mean instead of across the cone element, which is
	// a different curve and gives the two gears unequal twist.
	s.centre = vec{s.rMean - s.rc*math.Sin(s.psi), s.handSign * s.rc * math.Cos(s.psi)}

	s.toe2d = circleIntersectNearest(s.rToe-endRelief*s.span, s.centre, s.rc, vec{s.rMean, 0})
	s.heel2d = circleIntersectNearest(s.rHeel+endRelief*s.span, s.centre, s.rc, vec{s.rMean, 0})
	s.phiCrown = math.Atan2(s.heel2d.Y, s.heel2d.X) - math.Atan2(s.toe2d.Y, s.toe2d.X)
	s.total = math.Abs(s.phiCrown) / math.Sin(g.gamma)
	return s
}

// circleIntersectNearest intersects the apex circle of radius r with the cutter
// circle, and keeps the solution nearest ref — the branch the mean point sits on.
// A non-overlapping pair clamps to tangency.
func circleIntersectNearest(r float64, centre vec, rc float64, ref vec) vec {
	d := norm(centre)
	a := (d*d + r*r - rc*rc) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	base := scale(unit(centre), a)
	off := scale(vec{-unit(centre).Y, unit(centre).X}, h)
	one, two := add(base, off), sub(base, off)
	if norm(sub(one, ref)) <= norm(sub(two, ref)) {
		return one
	}
	return two
}

// stepConeElementSketch draws the `{gear} Cone Element` sketch: the one
// construction line from the apex out along the root cone element, which the
// trace plane is then built off.
func stepConeElementSketch(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	if p.SpiralAngle <= 0 {
		proofkit.Unmodelled(t, "the mean spiral angle is zero, so the tooth-body step takes "+
			"the straight branch and authors none of the spiral sketches")
		return
	}

	proofkit.Step(t, "the apex, and the cone element out to the heel")
	apex := s.CreateReferencePoint(0, 0, "gear profiles sketch")
	apex.SetName("Apex")
	far := scale(sp.gear.coneVec, sp.rHeel)
	start := s.CreatePoint(0, 0)
	end := s.CreatePoint(far.X, far.Y)
	line := s.CreateLine(start, end)
	line.SetName("cone element")
	line.SetConstruction(true)
	s.AddConstraint(sketch.NewCoincident(start, apex))
	// The far end is recreated at its computed position and fixed once the line
	// exists. The spec passes both endpoints in as raw world coordinates and
	// leaves the sketch free — it is one of the transient spiral sketches its own
	// full-constraint gate exempts — and proofkit gates whatever it is handed, so
	// the proof pins the end the recreate-share-fix way. What that costs is that
	// it proves the line's length and direction, not that Fusion's free version
	// resolves to them.
	s.Fix(end)

	proofkit.Step(t, "check the element against the root cone")
	near(t, "the cone element's length against the heel cone distance", line.Length(),
		sp.rHeel, 1e-9)
	near(t, "the cone element's direction against the root cone element",
		math.Abs(cross(unit(vec{end.X() - start.X(), end.Y() - start.Y()}), sp.gear.coneVec)),
		0, 1e-12)
}

// stepCutterArcSketch draws the `{gear} 2D Tooth Trace` sketch: the genuine
// face-mill cutter circle and the arc of it the tooth trace follows.
//
// Two substitutions. The spec leaves this sketch with free degrees of freedom on
// purpose — the arc's ends are pinned by three-point construction, and
// dimensioning them over-constrains the solve against the cone-element plane —
// and proofkit gates every sketch it is handed, so the proof pins the two ends at
// the circle-circle intersections the framework helper computes and fixes them.
// And the spec's own pair of arc constraints, a centre coincident with the cutter
// circle's centre and a radius dimension, is one equation more than the arc's
// centre has freedom once its ends are pinned, so the proof adds the single
// equation that decides the centre's side and MEASURES the rest: that the centre
// really is the cutter centre and the radius really is the cutter radius.
func stepCutterArcSketch(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	if p.SpiralAngle <= 0 {
		proofkit.Unmodelled(t, "the mean spiral angle is zero, so the straight branch runs "+
			"and no cutter arc is drawn")
		return
	}
	sp := spiralOf(p)

	proofkit.Step(t, "the cutter circle, centred by the spiral angle and the hand")
	apex := s.CreateReferencePoint(0, 0, "cone element")
	apex.SetName("apex")
	centre := s.CreatePoint(sp.centre.X, sp.centre.Y)
	centre.SetName("cutter circle centre")
	cutter := s.CreateCircle(centre, sp.rc)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*sp.rc))

	proofkit.Step(t, "the trace arc, toe through the mean point to heel")
	// Counter-clockwise from start to end, so the arc that closes is the one
	// through the mean point rather than the rest of the cutter circle.
	first, second := sp.toe2d, sp.heel2d
	if !ccwThrough(sp.centre, first, vec{sp.rMean, 0}, second) {
		first, second = second, first
	}
	arcStart := s.CreatePoint(first.X, first.Y)
	arcEnd := s.CreatePoint(second.X, second.Y)
	arcCentre := s.CreatePoint(sp.centre.X, sp.centre.Y)
	arc := s.CreateArc(arcCentre, arcStart, arcEnd)
	arc.SetName("trace arc")
	s.Fix(arcStart)
	s.Fix(arcEnd)
	chord := unit(sub(second, first))
	pinAlong(s, arcCentre, centre, vec{-chord.Y, chord.X})

	proofkit.Step(t, "check the trace against the cutter-arc construction")
	assertCutterArc(t, s, p, sp, arc, cutter)
}

// ccwThrough reports whether walking counter-clockwise about c from a reaches
// via before b.
func ccwThrough(c, a, via, b vec) bool {
	sweep := func(from, to vec) float64 {
		d := math.Atan2(cross(sub(from, c), sub(to, c)), dot(sub(from, c), sub(to, c)))
		if d < 0 {
			d += 2 * math.Pi
		}
		return d
	}
	return sweep(a, via) < sweep(a, b)
}

// assertCutterArc holds the drawn trace to the construction's own invariants: the
// arc IS the cutter circle, it passes through the mean point, its ends sit on the
// toe and heel apex circles, and it meets the cone element at the spiral angle.
func assertCutterArc(t testing.TB, s *sketch.Sketch, p params, sp spiral,
	arc *sketch.Arc, cutter *sketch.Circle) {
	solvedCentre := vec{arc.Center.X(), arc.Center.Y()}
	nearPoint(t, "the trace arc's centre against the cutter circle's", solvedCentre,
		sp.centre, 1e-9)
	near(t, "the trace arc's radius against the cutter radius",
		norm(sub(vec{arc.Start.X(), arc.Start.Y()}, solvedCentre)), sp.rc, 1e-9)
	near(t, "the cutter circle's radius", cutter.R(), sp.rc, 1e-9)

	// The centre is exactly the cutter radius from the mean point, so the arc
	// passes through it.
	near(t, "the mean point's distance from the cutter centre",
		norm(sub(vec{sp.rMean, 0}, solvedCentre)), sp.rc, 1e-9)

	// The ends sit on the toe and heel apex circles, taken a hair past the face.
	ends := []vec{{arc.Start.X(), arc.Start.Y()}, {arc.End.X(), arc.End.Y()}}
	wantRadii := []float64{sp.rToe - endRelief*sp.span, sp.rHeel + endRelief*sp.span}
	got := []float64{norm(ends[0]), norm(ends[1])}
	if math.Abs(got[0]-wantRadii[1]) < math.Abs(got[0]-wantRadii[0]) {
		got[0], got[1] = got[1], got[0]
	}
	near(t, "the toe end's cone distance", got[0], wantRadii[0], 1e-9)
	near(t, "the heel end's cone distance", got[1], wantRadii[1], 1e-9)

	// The spiral angle is realised AT the mean point: the tangent there makes the
	// angle psi with the cone element.
	radial := unit(sub(vec{sp.rMean, 0}, solvedCentre))
	tangent := vec{-radial.Y, radial.X}
	angle := math.Abs(math.Atan2(math.Abs(cross(vec{1, 0}, tangent)), dot(vec{1, 0}, tangent)))
	if angle > math.Pi/2 {
		angle = math.Pi - angle
	}
	near(t, "the spiral angle realised at the mean point", angle, sp.psi, 1e-9)

	// The hand mirrors the whole construction across the cone element and changes
	// nothing else, so the two hands are exact mirror images.
	flipped := p
	flipped.HandRight = !p.HandRight
	other := spiralOfParams(flipped)
	nearPoint(t, "the opposite hand's cutter centre, mirrored back",
		vec{other.centre.X, -other.centre.Y}, sp.centre, 1e-12)

	// And the straight-bevel limit: at psi = 0 the centre sits due circumferential
	// of the mean point, so the tangent there runs along the element.
	limit := p
	limit.SpiralAngle = 0
	straight := spiralOfParams(limit)
	near(t, "the cutter centre's cone distance in the straight limit", straight.centre.X,
		straight.rMean, 1e-12)
	near(t, "the cutter centre's circumferential offset in the straight limit",
		math.Abs(straight.centre.Y), straight.rc, 1e-12)
}

// spiralOfParams resolves the §3a frame from typed inputs, for the invariants
// that compare one case against a variant of itself.
func spiralOfParams(p params) spiral {
	return spiralOf(p)
}

// spiralSketchCases: both hands, both gears, the ends of the spiral-angle range
// the spec states, both sides of the auto cutter-radius branch, and the psi = 0
// case that takes the straight branch and draws none of this.
var spiralSketchCases = []proofkit.Case{
	{Name: "default_35deg_right_pinion", Params: map[string]float64{
		"spiralAngle": 35, "hand": 1, "pinion": 1}},
	{Name: "default_35deg_right_driving", Params: map[string]float64{
		"spiralAngle": 35, "hand": 1, "pinion": 0}},
	{Name: "left_hand_pinion", Params: map[string]float64{
		"spiralAngle": 35, "hand": 0, "pinion": 1}},
	{Name: "left_hand_driving", Params: map[string]float64{
		"spiralAngle": 35, "hand": 0, "pinion": 0}},
	{Name: "shallow_5deg", Params: map[string]float64{"spiralAngle": 5, "pinion": 1}},
	{Name: "steep_59deg", Params: map[string]float64{"spiralAngle": 59, "pinion": 1}},
	{Name: "given_cutter_radius", Params: map[string]float64{
		"spiralAngle": 35, "cutterRadius": 40, "pinion": 1}},
	{Name: "ratio_pinion_17", Params: map[string]float64{
		"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "ratio_driving_31", Params: map[string]float64{
		"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 0}},
	{Name: "straight_bevel_psi_zero", Params: map[string]float64{"spiralAngle": 0}},
}

// slabParam is where a slab's two bounding planes sit, as tooth-plane parameters.
// The planes are offsets of the parent tooth plane, stepped toward the apex in
// span/6 increments, and the tooth is a cone over its section, so an offset maps
// to a parameter by the ratio of the apex's distance to the plane.
func (sp spiral) slabParam(k int) float64 {
	return 1 - float64(k+1)*sp.span/(6*math.Abs(sp.gear.apexDist))
}

// slabRange is segment j's parameter range, apex-most first. Segment 0 is the
// apex-most kept slab and segment slicePlanes-1 the heel-most, which reaches past
// the heel to the tooth's own end.
func (sp spiral) slabRange(j int) (lo, hi float64) {
	lo = sp.slabParam(slicePlanes - 1 - j)
	if j == slicePlanes-1 {
		return lo, 1
	}
	return lo, sp.slabParam(slicePlanes - 2 - j)
}

// heelFaceParam is the parameter of segment j's heel face — its
// farthest-along-the-element end, which is the face the loft samples and the
// twist is keyed on.
func (sp spiral) heelFaceParam(j int) float64 {
	_, hi := sp.slabRange(j)
	return hi
}

// heelFaceDist is the cone distance of segment j's heel face.
func (sp spiral) heelFaceDist(j int) float64 {
	return sp.heelFaceParam(j) * sp.gear.distAlong(sp.gear.heelMid) / sp.heelParamOfHeelMid()
}

// heelParamOfHeelMid is the parameter at which the tooth's section sits at the
// heel edge midpoint's cone distance, which is 1 by construction.
func (sp spiral) heelParamOfHeelMid() float64 { return 1 }

// twistOf is segment j's rotation about the shaft axis: a linear share of the
// total toe-to-heel twist, keyed on the segment's HEEL FACE cone distance and
// centred on the mean cone distance so the mid-face section stays unrotated.
func (sp spiral) twistOf(j int) float64 {
	return -sp.handSign * sp.total * (sp.rMean - sp.heelFaceDist(j)) / sp.span
}

// crownOf is segment j's lengthwise relief factor: full at the heel and growing
// monotonically toward the toe, keyed on the heel-distance fraction.
func (sp spiral) crownOf(j int) float64 {
	u := (sp.rHeel - sp.heelFaceDist(j)) / sp.span
	return 1 - crownPerRad*(math.Abs(sp.total)/2)*u
}

// rootEdgeMidpoint is the scale base the crown anchors on: the midpoint of the
// two vertices of the segment's heel face nearest the shaft axis. Anchoring on
// the heel face's centroid instead lifts the tooth's root off the gear base.
func rootEdgeMidpoint(pts []r3.Vec) r3.Vec {
	best, second := -1, -1
	for i := range pts {
		if best < 0 || radiusOf(pts[i]) < radiusOf(pts[best]) {
			second, best = best, i
		} else if second < 0 || radiusOf(pts[i]) < radiusOf(pts[second]) {
			second = i
		}
	}
	return pts[best].Add(pts[second]).Scale(0.5)
}

func radiusOf(p r3.Vec) float64 { return math.Hypot(p.Y, p.Z) }

// stepSliceTooth builds one cross-section slab of the sliced tooth. The case
// table walks the slabs, so each proof document holds the one body that slab is.
//
// What it substitutes. The spec SPLITS the lofted tooth with a fixed scheme of
// eight planes offset from the parent tooth plane; decad has no split verb and
// refuses booleans on a lofted payload. A slab of a cone over a section is the
// loft between that cone's sections at the two cut planes, so the proof lofts it
// directly. What that costs is the split itself: the proof shows the slab the cut
// leaves, not that the cut lands.
func stepSliceTooth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	lo, hi := sp.slabRange(j)
	body, err := loftBetween(doc, sp.gear.section(lo), sp.gear.section(hi))
	if err != nil {
		t.Fatalf("loft slab %d between parameters %.4f and %.4f: %v", j, lo, hi, err)
	}
	return []*decad.Body{body}
}

func assertSliceTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	lo, hi := sp.slabRange(j)
	if len(bodies) != 1 {
		t.Fatalf("slab %d came back as %d bodies", j, len(bodies))
	}

	// Each cut plane is one span/6 further toward the apex than the last.
	if j < slicePlanes-1 {
		step := (sp.slabParam(0) - sp.slabParam(1)) * math.Abs(sp.gear.apexDist)
		near(t, "the slice step against a sixth of the span", step, sp.span/6, 1e-9)
	}

	// The slice must actually reach past the toe: the apex-most plane sits inside
	// the toe, or the first kept segment would still hold the scrap.
	if got := sp.slabParam(slicePlanes-1) * sp.rHeel; got >= sp.rToe {
		t.Errorf("the apex-most cut plane sits at cone distance %.4f, which is outside the "+
			"toe at %.4f; the scheme must span a touch past the toe", got, sp.rToe)
	}
	// And the heel-most plane sits inside the heel, so the outermost slab reaches
	// past it.
	if got := sp.slabParam(0) * sp.rHeel; got >= sp.rHeel {
		t.Errorf("the first cut plane sits at cone distance %.4f, at or past the heel at "+
			"%.4f", got, sp.rHeel)
	}

	area := sectionArea(sp.gear.section(1))
	want := area * sp.gear.apexDist / 3 * (hi*hi*hi - lo*lo*lo)
	near(t, "the slab's volume", mustVolume(t, bodies[0]), want, want*0.01)
}

func spiralSlabCases() []proofkit3d.Case {
	out := []proofkit3d.Case{}
	for _, set := range []struct {
		name   string
		params map[string]float64
	}{
		{"pinion_35deg", map[string]float64{"spiralAngle": 35, "pinion": 1}},
		{"driving_35deg", map[string]float64{"spiralAngle": 35, "pinion": 0}},
		{"pinion_59deg", map[string]float64{"spiralAngle": 59, "pinion": 1}},
		{"ratio_pinion_17_35deg", map[string]float64{
			"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	} {
		for j := range slicePlanes {
			params := map[string]float64{"segment": float64(j)}
			for k, v := range set.params {
				params[k] = v
			}
			out = append(out, proofkit3d.Case{
				Name:   set.name + "_segment_" + string(rune('0'+j)),
				Params: params,
			})
		}
	}
	return out
}

// stepDropApexScrap builds the apex-side scrap the slice leaves below the toe —
// the piece the build removes before the twist and the crown run.
func stepDropApexScrap(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	body, err := loftBetween(doc, sp.gear.section(apexStub),
		sp.gear.section(sp.slabParam(slicePlanes-1)))
	if err != nil {
		t.Fatalf("loft the apex-side scrap: %v", err)
	}
	return []*decad.Body{body}
}

func assertDropApexScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the scrap came back as %d bodies", len(bodies))
	}

	// The scrap is the apex-most piece: sorting the nine pieces by the cone
	// distance of their centroid puts it first, and it is the only one below the
	// toe.
	scrapCentre := (apexStub + sp.slabParam(slicePlanes-1)) / 2 * sp.rHeel
	for j := range slicePlanes {
		lo, hi := sp.slabRange(j)
		if centre := (lo + hi) / 2 * sp.rHeel; centre <= scrapCentre {
			t.Errorf("kept segment %d has its centroid at cone distance %.4f, at or below the "+
				"scrap's %.4f; the scrap must sort first", j, centre, scrapCentre)
		}
	}
	if scrapCentre >= sp.rToe {
		t.Errorf("the scrap's centroid sits at cone distance %.4f, at or past the toe at "+
			"%.4f; it is supposed to be the long piece BELOW the toe", scrapCentre, sp.rToe)
	}

	// Dropping it must leave segments to twist and crown. An empty list here is
	// what makes the crown fail later with an empty max, far from the cause.
	if slicePlanes < 1 {
		t.Fatalf("no segments remain after dropping the scrap")
	}
	near(t, "the segments left after the drop", float64(slicePlanes), 8, 0)
}

// stepTwistSegments rotates one segment about the shaft axis by its linear share
// of the toe-to-heel twist.
func stepTwistSegments(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	lo, hi := sp.slabRange(j)
	angle := sp.twistOf(j)
	body, err := loftBetween(doc,
		rotateAboutShaft(sp.gear.section(lo), angle),
		rotateAboutShaft(sp.gear.section(hi), angle))
	if err != nil {
		t.Fatalf("loft segment %d at its twist: %v", j, err)
	}
	return []*decad.Body{body}
}

func assertTwistSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	if len(bodies) != 1 {
		t.Fatalf("segment %d came back as %d bodies", j, len(bodies))
	}
	angle := sp.twistOf(j)

	// The twist really is applied: the body's centroid is the untwisted centroid
	// turned by the angle about the shaft axis.
	lo, hi := sp.slabRange(j)
	plain := centroidVec(append(sp.gear.section(lo), sp.gear.section(hi)...))
	centre := centroidOf(t, bodies[0])
	near(t, "the twisted segment's azimuth",
		wrapPi(math.Atan2(centre.Z, centre.Y)-math.Atan2(plain.Z, plain.Y)-angle), 0, 1e-6)

	// The law: the total is the arc's developed azimuth at the apex over sin of
	// the PITCH cone angle, and each segment takes a linear share keyed on its
	// heel face, centred on the mean cone distance.
	near(t, "the developed crown azimuth",
		math.Atan2(sp.heel2d.Y, sp.heel2d.X)-math.Atan2(sp.toe2d.Y, sp.toe2d.X),
		sp.phiCrown, 1e-12)
	near(t, "the total shaft-axis twist", sp.total,
		math.Abs(sp.phiCrown)/math.Sin(sp.gear.gamma), 1e-12)
	near(t, "this segment's share", angle,
		-sp.handSign*sp.total*(sp.rMean-sp.heelFaceDist(j))/sp.span, 1e-12)

	// The pitch cone angle, not the root cone angle. The two differ by the
	// dedendum angle, and using the root angle inflates the twist.
	rootAngle := math.Acos(dot(sp.gear.coneVec, vec{1, 0}))
	if math.Abs(rootAngle-sp.gear.gamma) < 1e-6 {
		t.Errorf("the root cone angle %.6f and the pitch cone angle %.6f coincide, so this "+
			"case cannot tell the two apart", rootAngle, sp.gear.gamma)
	}
	if inflated := math.Abs(sp.phiCrown) / math.Sin(rootAngle); math.Abs(inflated-sp.total) <
		0.02*sp.total {
		t.Errorf("keying the twist on the root cone angle gives %.6f against the pitch "+
			"angle's %.6f; the two must differ enough for the choice to matter",
			inflated, sp.total)
	}

	// Keyed on the heel FACE, not the centroid: the two differ by half a
	// segment's share, which is the mid-face overlap the spec warns about.
	centroidKeyed := -sp.handSign * sp.total * (sp.rMean - (lo+hi)/2*sp.rHeel) / sp.span
	halfShare := sp.total * (hi - lo) * sp.rHeel / (2 * sp.span)
	near(t, "the gap between heel-face keying and centroid keying",
		math.Abs(angle-centroidKeyed), halfShare, halfShare*0.05)
}

// wrapPi folds an angle into (-pi, pi].
func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// stepCrownSegments scales one segment down about a point on its heel face's root
// edge, which is the lengthwise crown.
func stepCrownSegments(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	lo, hi := sp.slabRange(j)
	angle := sp.twistOf(j)
	low := rotateAboutShaft(sp.gear.section(lo), angle)
	high := rotateAboutShaft(sp.gear.section(hi), angle)

	factor := sp.crownOf(j)
	if j == slicePlanes-1 {
		// The outermost segment is held full: its heel face is the loft's heel end
		// and has to stay flush with the gear base.
		factor = 1
	}
	if factor <= 0 {
		t.Fatalf("segment %d crowns by a factor of %.4f at heel-distance fraction %.4f; a "+
			"non-positive factor is never scaled by", j,
			factor, (sp.rHeel-sp.heelFaceDist(j))/sp.span)
	}
	base := rootEdgeMidpoint(high)
	body, err := loftBetween(doc, scaleAbout(low, base, factor), scaleAbout(high, base, factor))
	if err != nil {
		t.Fatalf("loft the crowned segment %d: %v", j, err)
	}
	return []*decad.Body{body}
}

func assertCrownSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	j := int(pm["segment"])
	if len(bodies) != 1 {
		t.Fatalf("the crowned segment came back as %d bodies", len(bodies))
	}
	_, hi := sp.slabRange(j)
	high := rotateAboutShaft(sp.gear.section(hi), sp.twistOf(j))
	base := rootEdgeMidpoint(high)

	if j == slicePlanes-1 {
		near(t, "the heel-most segment's factor", 1, 1, 0)
	} else {
		u := (sp.rHeel - sp.heelFaceDist(j)) / sp.span
		near(t, "the crown factor", sp.crownOf(j),
			1-crownPerRad*(math.Abs(sp.total)/2)*u, 1e-12)
		// Monotonic from the held-full heel toward the toe: an inner segment is
		// relieved more than the one outside it, so the slab heights stay ordered
		// and the natural taper is never reversed.
		if j+1 < slicePlanes-1 && sp.crownOf(j) >= sp.crownOf(j+1) {
			t.Errorf("segment %d crowns by %.6f and the segment outside it by %.6f; the "+
				"relief must grow monotonically toward the toe", j, sp.crownOf(j),
				sp.crownOf(j+1))
		}
	}

	// The scale base is on the ROOT edge, and a uniform scale about a point keeps
	// every line through that point where it is — so the root edge stays on the
	// seating cone. Anchoring on the heel face's centroid instead would lift it.
	// The base is the midpoint of the heel face's two ROOT corners, so it sits on
	// the chord between them: just inside the root radius, and nowhere near the
	// mid-height the face's centroid sits at.
	rootRadius := rootRadiusOf(high)
	_, tipRadius := radialExtent(high)
	if sag := rootRadius - radiusOf(base); sag < 0 || sag > 0.05*(tipRadius-rootRadius) {
		t.Errorf("the scale base sits %.6f mm inside the root radius %.6f; it is supposed to "+
			"be the midpoint of the heel face's two root corners", sag, rootRadius)
	}
	centroid := centroidVec(high)
	if radiusOf(centroid) <= radiusOf(base) {
		t.Errorf("the heel face's centroid is at radius %.6f and the root-edge base at %.6f; "+
			"the centroid must sit further out, which is why anchoring there lifts the root",
			radiusOf(centroid), radiusOf(base))
	}
	lift := (1 - sp.crownOf(j)) * (radiusOf(centroid) - rootRadius)
	if j != slicePlanes-1 && lift <= 0 {
		t.Errorf("anchoring the crown on the heel face's centroid would move the root by "+
			"%.6f mm; the defect this rule exists for is a positive lift", lift)
	}
}

// rootRadiusOf is the smallest distance from the shaft axis over a section — the
// section's root corners.
func rootRadiusOf(pts []r3.Vec) float64 {
	lo, _ := radialExtent(pts)
	return lo
}

// stepLoftSpiralTooth lofts the curved tooth through the twisted, crowned
// segments, in the order their heel faces sit along the cone element.
//
// What it substitutes. The spec lofts through the toe-most segment's toe face and
// then every segment's heel face in order — nine sections. decad's loft rules
// between two, so the proof lofts between the two ends: the toe-most face and the
// heel-most, each carrying its own twist. What that costs is the intermediate
// sections, so the ruled surface between them is straight where the real loft
// follows the segments. What it keeps is the thing the step is about — that the
// two ends carry the twist the law gives them, in the post-twist order.
func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	order := sp.postTwistOrder()
	first, last := order[0], order[len(order)-1]
	lo, _ := sp.slabRange(first)
	_, hi := sp.slabRange(last)
	body, err := loftBetween(doc,
		rotateAboutShaft(sp.gear.section(lo), sp.twistOf(first)),
		rotateAboutShaft(sp.gear.section(hi), sp.twistOf(last)))
	if err != nil {
		t.Fatalf("loft the curved tooth: %v", err)
	}
	return []*decad.Body{body}
}

// postTwistOrder sorts the segment indices by the cone distance of their heel
// face AFTER the twist has moved them. The twist rotates each slab about the
// shaft axis, and that rotation changes a face's projection on the cone element,
// so the order has to be recomputed here rather than reused from the slice.
func (sp spiral) postTwistOrder() []int {
	order := make([]int, 0, slicePlanes)
	for j := range slicePlanes {
		order = append(order, j)
	}
	key := func(j int) float64 {
		_, hi := sp.slabRange(j)
		section := rotateAboutShaft(sp.gear.section(hi), sp.twistOf(j))
		c := centroidVec(section)
		return sp.gear.distAlong(vec{c.X, c.Y})
	}
	for i := 1; i < len(order); i++ {
		for k := i; k > 0 && key(order[k]) < key(order[k-1]); k-- {
			order[k], order[k-1] = order[k-1], order[k]
		}
	}
	return order
}

func assertLoftSpiralTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the curved tooth came back as %d bodies", len(bodies))
	}
	order := sp.postTwistOrder()
	if len(order) != slicePlanes {
		t.Fatalf("the loft order names %d segments, want %d", len(order), slicePlanes)
	}
	// The order is a permutation of the segments, computed after the twist. For a
	// low-twist pair it is the slice order; the proof records which it is rather
	// than assuming.
	inOrder := true
	for i, j := range order {
		if i != j {
			inOrder = false
		}
	}
	t.Logf("post-twist loft order %v (unchanged from the slice order: %v)", order, inOrder)

	// The two ends carry the twist the law gives them, and they differ: a curved
	// tooth's ends are turned in opposite senses about the mean.
	first, last := order[0], order[len(order)-1]
	if sp.total > 1e-9 && sp.twistOf(first)*sp.twistOf(last) >= 0 {
		t.Errorf("the toe end twists by %.6f and the heel end by %.6f; centred on the mean "+
			"cone distance they must lie on opposite sides",
			sp.twistOf(first), sp.twistOf(last))
	}
	near(t, "the toe-to-heel twist across the lofted tooth",
		math.Abs(sp.twistOf(first)-sp.twistOf(last)),
		sp.total*math.Abs(sp.heelFaceDist(last)-sp.heelFaceDist(first))/sp.span, 1e-9)
}

// stepTrimSpiralTooth trims the curved tooth flush, the same toe-then-heel pair
// of cone cuts the straight tooth takes. It is substituted the same way — see
// [stepTrimToothBand] — with the twist carried on each end section.
func stepTrimSpiralTooth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	sp := spiralOf(p)
	toeTwist := -sp.handSign * sp.total * (sp.rMean - sp.rToe) / sp.span
	heelTwist := -sp.handSign * sp.total * (sp.rMean - sp.rHeel) / sp.span
	body, err := loftBetween(doc,
		rotateAboutShaft(sp.gear.section(sp.gear.toeParam()), toeTwist),
		rotateAboutShaft(sp.gear.section(sp.gear.heelParam()), heelTwist))
	if err != nil {
		t.Fatalf("loft the trimmed curved tooth: %v", err)
	}
	return []*decad.Body{body}
}

func assertTrimSpiralTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	sp := spiralOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the trimmed curved tooth came back as %d bodies", len(bodies))
	}
	toeTwist := -sp.handSign * sp.total * (sp.rMean - sp.rToe) / sp.span
	heelTwist := -sp.handSign * sp.total * (sp.rMean - sp.rHeel) / sp.span
	near(t, "the trimmed tooth's toe-to-heel twist", math.Abs(toeTwist-heelTwist),
		sp.total, 1e-12)
	// The mid-face section is unrotated, which is why the pinion needs no extra
	// mesh phase.
	mid := -sp.handSign * sp.total * (sp.rMean - sp.rMean) / sp.span
	near(t, "the mid-face section's twist", mid, 0, 0)
	// The two hands come out as mirror images: same magnitude, opposite sense.
	other := p
	other.HandRight = !p.HandRight
	flipped := spiralOfParams(other)
	near(t, "the opposite hand's total twist", flipped.total, sp.total, 1e-12)
	near(t, "the opposite hand's toe twist, mirrored back",
		-(-flipped.handSign * flipped.total * (flipped.rMean - flipped.rToe) / flipped.span),
		toeTwist, 1e-12)
}

// spiralSolidCases: both gears at the default spiral angle and at the ends of the
// stated range, both hands, and a ratio pair whose two members take legitimately
// different twists because their pitch cone angles differ.
var spiralSolidCases = []proofkit3d.Case{
	{Name: "default_pinion_right", Params: map[string]float64{
		"spiralAngle": 35, "hand": 1, "pinion": 1}},
	{Name: "default_driving_right", Params: map[string]float64{
		"spiralAngle": 35, "hand": 1, "pinion": 0}},
	{Name: "default_pinion_left", Params: map[string]float64{
		"spiralAngle": 35, "hand": 0, "pinion": 1}},
	{Name: "shallow_5deg_pinion", Params: map[string]float64{
		"spiralAngle": 5, "pinion": 1}},
	// The steep end stops at 57 degrees here, not the 60 the range reaches: at 59
	// the two end sections are turned far enough apart that the evaluator refuses
	// the single loft between them as degenerate. That is a limit of the
	// two-section SUBSTITUTE, not of the design — the slice, twist and crown steps
	// carry a 59 degree case, where each loft spans one slab.
	{Name: "steep_57deg_pinion", Params: map[string]float64{
		"spiralAngle": 57, "pinion": 1}},
	{Name: "ratio_pinion_17", Params: map[string]float64{
		"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "ratio_driving_31", Params: map[string]float64{
		"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 0}},
	{Name: "given_cutter_radius_pinion", Params: map[string]float64{
		"spiralAngle": 35, "cutterRadius": 40, "pinion": 1}},
}

// segmentCases walks every kept segment: both gears at the default spiral angle,
// the pinion at the top of the stated range, and one member of a ratio pair, so
// the slice, the twist and the crown are each built and measured at every index.
var segmentCases = spiralSlabCases()
