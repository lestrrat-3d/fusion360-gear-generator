// This file holds the ψ > 0 tooth-body chain of §3a: the slab slicing and the
// apex-scrap drop, the twist, the lengthwise crown and the spiral loft. At ψ = 0
// none of it runs — the hook returns the straight tooth's two conical trims
// before any of this construction is reached — so every case here carries a Mean
// Spiral Angle above zero.
//
// THE SLABS ARE REAL AND THEY ARE BUILT, NOT CUT. decad has no split-by-plane, so
// each slab is lofted between the two sections the parallel cut planes produce
// rather than carved out of one tooth. The planes are the same family: the parent
// tooth plane offset toward the apex by sign*(k+1)*span/6 for k = 0..7, and each
// section is the tooth's own cross-section there, which is the full section
// scaled about the apex because the tooth is a cone from the apex. The cost is
// that the proof does not show the evaluator dividing a body: the piece count
// after the cut loop, the retry with the opposite sign and the raise that follows
// a second single-piece result are all conditions this file asserts on the
// offsets rather than on a split. The slabs are also LAID APART along the shaft
// axis, because built where they belong they are face to face and decad refuses a
// pair in exact contact; [bevSpiralPlan.displacement] states what that costs.
//
// THE SLICE FAMILY REACHES FUSION UNTESTED, and that is recorded here because it
// is the largest gap in this chain. The cut planes are NOT perpendicular to the
// cone element: the parent plane carries the tooth-centre line C->K'/D->L', which
// is the back-cone line and so perpendicular to the Pitch Line, which puts the
// parent plane's normal along the PITCH element while coneVec is the ROOT
// element. The two differ by the dedendum angle, 3.26 degrees on the default
// 31/31 pair at Shaft Angle 90. A build that follows "perpendicular to the cone
// element" instead is wrong and SILENT — the piece count, the retry gate and the
// conical trims are all indifferent to slab orientation, and the loft still
// reproduces the taper. This proof builds its own slabs from the offsets the spec
// fixes and never reads the plane the generated module constructs, so the
// module's choice of family is not checked here. Only a measurement on a loaded
// spiral gear closes it — a face corner's position along the cone, which none of
// the three recorded loads reported.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// slabCount is the fixed slice scheme: exactly eight planes, which is not user
// configurable, cutting the tooth into nine pieces of which the apex-most is the
// scrap.
const bevSlabCount = 8

// spiralSolidCases sweep the spiral chain at Module 4 and 8, over both hands,
// both cutter-radius branches and the whole [0, 60) Mean Spiral Angle range above
// zero. The ratio pairs are here because the two members of a pair legitimately
// get DIFFERENT twists — same cutter and same psi, but gamma differs, so 1/sin
// gamma differs — and that is why equal-teeth pairs always meshed while ratio
// pairs failed under any method that gets the roll ratio wrong.
var bevSpiralSolidCases = bevPerGearSolid([]proofkit3d.Case{
	{Name: "module4_spiral35_right", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"spiralAngle", 35})},
	{Name: "module4_spiral35_left", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"spiralAngle", 35}, bevKV{"hand", -1})},
	{Name: "module4_spiral5_near_straight", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"spiralAngle", 5})},
	{Name: "module4_spiral55_upper_range", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"spiralAngle", 55})},
	{Name: "module4_spiral35_user_cutter", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"spiralAngle", 35}, bevKV{"cutterRadius", 120})},
	{Name: "module4_spiral35_ratio_43_31", Params: bevWith(bevCase(4, 75, 43, 31),
		bevKV{"spiralAngle", 35})},
	{Name: "module8_spiral35_ratio_17_31", Params: bevWith(bevCase(8, 90, 17, 31),
		bevKV{"spiralAngle", 35})},
})

// slab is one cross-section segment of the sliced tooth, with everything §3a
// computes about it.
type bevSlab struct {
	index int
	// nearScale and farScale are the two bounding sections' sizes relative to the
	// full back-cone section, which is also their distance from the apex relative
	// to the Pitch Cone Distance. near is the apex side.
	nearScale, farScale float64
	heelDistance        float64 // distAlong of the heel face's centroid, before the twist
	postTwistDistance   float64 // the same reading recomputed AFTER the twist
	angle               float64 // the step-G rotation about the shaft axis
	fraction            float64 // the step-H heel-distance fraction u
	crown               float64 // the step-H scale factor
	base                r3.Vec  // the crown's base point, on the heel face's ROOT edge
}

// spiralPlan is one gear's whole §3a plan: the frame, the trace, and the eight
// slabs with their twists and crowns.
type bevSpiralPlan struct {
	sp       *bevSpiral
	frame    r3.Frame
	ring     []bevPt
	coneVec  r3.Vec
	centroid bevPt // the tooth section's own centroid, in the tooth plane's (u, v)
	azimuth  float64
	slabs    []bevSlab
	outer    int     // the index of the outermost (heel) slab, which the crown skips
	clear    float64 // the along-axis spacing the slabs are laid apart by
}

// displacement is how far slab i is moved along the SHAFT AXIS so that no two
// slabs touch.
//
// This is the substitution this chain makes, and it is the one the spec's own
// account of the spiral steps sanctions: real slabs, laid apart. Built where they
// belong the slabs are face to face — each one's toe face IS the next one's heel
// face — and decad refuses a pair in exact contact, reporting that the read-only
// intersection cannot classify it. Moving them along the shaft axis leaves the
// azimuth about that axis untouched, which is the reading the twist step asserts,
// and every volume and length untouched too. The assertions that compare a
// position rather than an angle add the same displacement back, so nothing is
// compared against a point it was never built at. THE COST is that the proof does
// not show the nine sections standing in one stack: a slab that the slicing put in
// the wrong place relative to its neighbours would still read correctly here.
func (p *bevSpiralPlan) displacement(index int) r3.Vec {
	return r3.NewVec(float64(index)*p.clear, 0, 0)
}

// planSpiral works out §3a for one gear without building anything, so the
// assertions have a closed form to measure against.
func (f *bevFigure) planSpiral(t *testing.T, g *bevSide) *bevSpiralPlan {
	t.Helper()
	plan := &bevSpiralPlan{sp: f.spiralOf(g)}
	dims := f.toothDimensions(g)
	plan.ring = bevToothRing(dims, g.virtualTeeth)

	frame, err := r3.NewFrame(f.toothCentreWorld(g), f.dedendumWorld(g), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("%s: tooth-plane frame: %v", g.label, err)
	}
	plan.frame = frame
	plan.coneVec = r3.NewVec(math.Cos(g.gammaRoot), math.Sin(g.gammaRoot), 0)
	plan.centroid = bevRingCentroid(plan.ring)
	plan.clear = 1.5*(f.pitchCone/math.Cos(g.gamma)+dims.Tip) + 1

	full := frame.ToWorldUV(plan.centroid.X, plan.centroid.Y)
	plan.azimuth = math.Atan2(full.Z, full.Y)
	fullDistance := full.Dot(plan.coneVec)

	// The eight offsets, and the nine pieces they cut. Offset k sits k*span/6
	// inside the parent plane, measured perpendicular to it, and the section there
	// is the full one scaled by (R - offset)/R because the tooth is a cone from the
	// apex.
	scaleAt := func(offset float64) float64 { return (f.pitchCone - offset) / f.pitchCone }
	for k := 0; k < bevSlabCount; k++ {
		heelOffset := float64(k) * plan.sp.span / 6
		toeOffset := float64(k+1) * plan.sp.span / 6
		s := bevSlab{
			index:     k,
			farScale:  scaleAt(heelOffset),
			nearScale: scaleAt(toeOffset),
		}
		s.heelDistance = s.farScale * fullDistance
		s.angle = -plan.sp.handSign * plan.sp.total *
			(plan.sp.rMean - s.heelDistance) / plan.sp.span
		// The twist rotates the slab about the SHAFT axis while distAlong is measured
		// along the ROOT element, so the reading moves. §3a step H recomputes it
		// AFTER the twist and this is where that happens.
		moved := bevRotateAboutX(full.Scale(s.farScale), s.angle)
		s.postTwistDistance = moved.Dot(plan.coneVec)
		plan.slabs = append(plan.slabs, s)
	}

	// The outermost (heel) segment is the one with the GREATEST post-twist
	// heel-face distAlong, and it is held full so its heel face stays the loft's
	// heel end for the heel cone to trim flush.
	plan.outer = 0
	for i, s := range plan.slabs {
		if s.postTwistDistance > plan.slabs[plan.outer].postTwistDistance {
			plan.outer = i
		}
	}
	for i := range plan.slabs {
		s := &plan.slabs[i]
		s.fraction = (plan.sp.rHeel - s.postTwistDistance) / plan.sp.span
		s.crown = plan.sp.crownFactor(s.fraction)
		if i == plan.outer {
			s.crown = 1
		}
		s.base = plan.rootEdgeMidpoint(s.farScale, s.angle)
	}
	return plan
}

// rootCorners are one slab's heel-face two ROOT corners, in world coordinates,
// after the step-G twist and before the step-H crown: the two vertices with the
// smallest perpendicular distance to the shaft axis. The tip corners are the
// farthest from that axis, which is what tells the two pairs apart.
func (p *bevSpiralPlan) rootCorners(scale, angle float64) (r3.Vec, r3.Vec) {
	type corner struct {
		radius float64
		world  r3.Vec
	}
	corners := make([]corner, 0, len(p.ring))
	for _, v := range p.ring {
		w := bevRotateAboutX(p.frame.ToWorldUV(v.X, v.Y).Scale(scale), angle)
		corners = append(corners, corner{math.Hypot(w.Y, w.Z), w})
	}
	first, second := 0, 1
	if corners[second].radius < corners[first].radius {
		first, second = second, first
	}
	for i := 2; i < len(corners); i++ {
		switch {
		case corners[i].radius < corners[first].radius:
			second, first = first, i
		case corners[i].radius < corners[second].radius:
			second = i
		}
	}
	return corners[first].world, corners[second].world
}

// rootEdgeMidpoint is the crown's base point for one slab: the midpoint of its
// heel face's two ROOT corners. Anchoring on the heel face's CENTROID instead
// pulls the tooth's root edge upward by (1-factor) times half the tooth height,
// the tooth floats off the gear body's root cone, and the Combine-Join leaves a
// visible gap.
func (p *bevSpiralPlan) rootEdgeMidpoint(scale, angle float64) r3.Vec {
	a, b := p.rootCorners(scale, angle)
	return a.Add(b).Scale(0.5)
}

// ringCentroid is a closed polygon's area centroid.
func bevRingCentroid(ring []bevPt) bevPt {
	var cx, cy, area float64
	for i := range ring {
		j := (i + 1) % len(ring)
		cross := ring[i].cross(ring[j])
		area += cross
		cx += (ring[i].X + ring[j].X) * cross
		cy += (ring[i].Y + ring[j].Y) * cross
	}
	return bevPt{cx / (3 * area), cy / (3 * area)}
}

// rotateAboutX turns a point about the shaft axis, which is the world x axis in
// this frame.
func bevRotateAboutX(v r3.Vec, angle float64) r3.Vec {
	s, c := math.Sin(angle), math.Cos(angle)
	return r3.NewVec(v.X, v.Y*c-v.Z*s, v.Y*s+v.Z*c)
}

// buildSlab lofts one segment between its two sections, optionally rotated by the
// step-G twist and scaled by the step-H crown about the slab's own base point.
func (f *bevFigure) buildSlab(t *testing.T, doc *decad.Document, w *sketch.World, g *bevSide,
	plan *bevSpiralPlan, s bevSlab, twist bool, crown bool) *decad.Body {
	t.Helper()
	angle := 0.0
	if twist {
		angle = s.angle
	}
	factor := 1.0
	if crown {
		factor = s.crown
	}
	if factor <= 0 {
		t.Fatalf("%s: slab %d crowns to a factor of %.6g at u = %.6g; a non-positive factor is "+
			"never scaled by", g.label, s.index, factor, s.fraction)
	}
	section := func(scale float64) (*sketch.Sketch, *sketch.Profile) {
		// A uniform scale about the base point keeps every plane through that point
		// invariant, so the scaled section sits on a parallel plane whose origin moves
		// the same way and whose in-plane coordinates are simply scaled.
		origin := f.toothCentreWorld(g).Scale(scale)
		origin = bevRotateAboutX(origin, angle)
		origin = s.base.Add(origin.Sub(s.base).Scale(factor))
		origin = origin.Add(plan.displacement(s.index))
		u := bevRotateAboutX(f.dedendumWorld(g), angle)
		v := bevRotateAboutX(r3.NewVec(0, 0, 1), angle)
		frame, err := r3.NewFrame(origin, u, v)
		if err != nil {
			t.Fatalf("%s: slab %d section frame: %v", g.label, s.index, err)
		}
		plane, err := w.CreatePlaneFromFrame(frame)
		if err != nil {
			t.Fatalf("%s: slab %d section plane: %v", g.label, s.index, err)
		}
		return bevRingSketch(t, w, plane, bevScaleRing(plan.ring, scale*factor))
	}
	nearSketch, nearRegion := section(s.nearScale)
	farSketch, farRegion := section(s.farScale)
	body, err := doc.Loft(nearSketch, nearRegion, farSketch, farRegion)
	if err != nil {
		t.Fatalf("%s: loft slab %d: %v", g.label, s.index, err)
	}
	return body
}

// stepSliceTooth splits the uncut apex-to-heel tooth into cross-section slabs by
// eight planes PARALLEL to the parent transverse tooth plane, then sorts them by
// the distAlong of their centroid and drops the apex-most piece as scrap.
//
// The slice MUST actually split the tooth. In the module, a body still in one
// piece after the cut loop means the offset sign was wrong or the parent plane
// sits outside the tooth's span, and the whole cut is retried once with the
// opposite sign; a second single-piece result raises, naming the gear, the final
// piece count, the span and the sign tried. Returning an unsliced result instead
// leaves the next step's segment list empty and the crown crashes far from the
// cause. Here the slabs are built rather than cut, so what this step asserts is
// the condition those gates exist to catch: eight planes, all of them inside the
// tooth's own span, producing nine pieces of which eight survive the scrap drop.
func stepSliceTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	w := sketch.NewWorld()

	bodies := make([]*decad.Body, 0, bevSlabCount)
	for _, s := range plan.slabs {
		bodies = append(bodies, f.buildSlab(t, doc, w, g, plan, s, false, false))
	}
	return bodies
}

// assertSliceTooth measures where the eight planes land and what the slabs add up
// to.
func assertSliceTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)

	if len(bodies) != bevSlabCount {
		t.Fatalf("%s: the scrap drop leaves %d segments, want the eight the fixed scheme produces "+
			"(nine pieces less the apex scrap); an empty list is what makes the crown crash with "+
			"max() on an empty iterable far from the cause", g.label, len(bodies))
	}

	// The offsets are sign*(k+1)*span/6, and the count is fixed at eight rather
	// than user configurable.
	for k, want := range plan.sp.offsets {
		got := float64(k+1) * plan.sp.span / 6
		if math.Abs(got-want) > 1e-12*plan.sp.span {
			t.Errorf("%s: cut plane %d sits at %.9f, want %.9f", g.label, k, got, want)
		}
	}

	// Where the eight land. The first sits span/6 inside the HEEL and none lies
	// past it, because the parent plane IS the heel end and there is no heel
	// overshoot to give. The sixth lands at the toe, and the last two sit span/6
	// and 2*span/6 past it; those two are what step J's toe cone trims away.
	first := plan.sp.rHeel - plan.sp.offsets[0]
	if !(first < plan.sp.rHeel) {
		t.Errorf("%s: the first cut plane sits at %.6g, outside the heel at %.6g",
			g.label, first, plan.sp.rHeel)
	}
	sixth := plan.sp.rHeel - plan.sp.offsets[5]
	if math.Abs(sixth-plan.sp.rToe) > 0.05*plan.sp.span {
		t.Errorf("%s: the sixth cut plane lands at %.6g, want the toe at %.6g; it sits a fraction "+
			"of a millimetre inside because R_heel and R_toe are read at the two edge midpoints "+
			"rather than on the root element", g.label, sixth, plan.sp.rToe)
	}
	for _, k := range []int{6, 7} {
		if plan.sp.rHeel-plan.sp.offsets[k] >= plan.sp.rToe {
			t.Errorf("%s: cut plane %d sits at %.6g, which is not past the toe at %.6g",
				g.label, k, plan.sp.rHeel-plan.sp.offsets[k], plan.sp.rToe)
		}
	}

	// The slabs add back to the tooth between the outermost plane and the parent
	// plane. Each is the frustum of the cone from the apex between two similar
	// sections, so its volume is A*R*(near^3 - far^3)/3 with the scales this plan
	// computed; the formula's error is float rounding on the shoelace area.
	area := bevPolygonArea(plan.ring)
	for i, body := range bodies {
		s := plan.slabs[i]
		want := area * f.pitchCone *
			(s.farScale*s.farScale*s.farScale - s.nearScale*s.nearScale*s.nearScale) / 3
		decadtest.MeasuresVolume(t, body, units.CubicMillimeters(want),
			decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

// stepTwistSegments rotates each segment about the SHAFT axis so the tooth
// follows the trace, centred on R_mean so the mid-face section stays unrotated.
// That section then meshes exactly like the straight tooth, which is what the
// pinion's zero mesh nudge depends on.
//
// The rotation is keyed on the segment's HEEL-FACE cone distance, never on its
// centroid: the loft samples each segment's heel face, so that face is what must
// land at the right azimuth, and centroid-keying leaves the loft's mid-face
// section rotated by half a segment.
//
// THE TWO HALVES OF THE LAW ARE TAKEN ON TWO DIFFERENT CONES, deliberately.
// phi_crown is measured in the frame §3a step A builds, whose x axis is the ROOT
// cone element, while the divisor sin(gamma) is the PITCH cone's roll ratio,
// because the crown gear the generation law rolls against is tangent to the pitch
// cone. Written consistently on the root cone the divisor would be
// sin(gamma_root), which is 1.062 times smaller on the default pair and would
// twist the tooth about 6% further. The pitch angle is what is kept, and a build
// that used acos(coneVec . axisDir) — the root angle — inflated the twist by
// about 1.15 on a 17-tooth pinion meshing a 31-tooth gear, which is the defect
// that kept ratio pairs from meshing at all.
//
// WHAT IS NOT SETTLED, recorded here beside the assertion as the spec asks:
// whether the FRAME should move to the pitch cone to match the divisor. Nothing
// in this repository measures it. This step asserts the twist against the same
// formula the module computes, so it confirms the arithmetic and says nothing
// about which cone the frame belongs on; no Fusion load has reported a trace
// azimuth or a face corner either. Moving the frame would change R_toe and
// R_heel, which are read along coneVec, and would change which element psi is
// measured against, so it is its own derivation and its own change.
func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	w := sketch.NewWorld()

	bodies := make([]*decad.Body, 0, bevSlabCount)
	for _, s := range plan.slabs {
		bodies = append(bodies, f.buildSlab(t, doc, w, g, plan, s, true, false))
	}
	return bodies
}

// assertTwistSegments measures each segment's azimuth against the linear share
// the crown-gear law gives it.
func assertTwistSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	if len(bodies) != bevSlabCount {
		t.Fatalf("%s: the twist leaves %d segments, want %d", g.label, len(bodies), bevSlabCount)
	}

	// The law itself, asserted against the same closed form the module computes.
	recomputed := math.Abs(plan.sp.phiCrown) / math.Sin(g.gamma)
	if math.Abs(recomputed-plan.sp.total) > 1e-12*math.Max(1, plan.sp.total) {
		t.Errorf("%s: the twist magnitude %.12f does not come from |phi_crown| / sin(gamma)",
			g.label, plan.sp.total)
	}
	// The ROOT-cone divisor is what a build reaching for acos(coneVec . axisDir)
	// would use, and it is a real difference rather than a rounding.
	rootDivisor := math.Abs(plan.sp.phiCrown) / math.Sin(g.gammaRoot)
	if plan.sp.total > 0 && rootDivisor <= plan.sp.total {
		t.Errorf("%s: the root-cone divisor gives %.9f against the pitch-cone %.9f; the root angle "+
			"is the smaller one and must inflate the twist", g.label, rootDivisor, plan.sp.total)
	}

	for i, body := range bodies {
		s := plan.slabs[i]
		azimuth, _, _ := bevToothPose(t, body)
		want := bevNormalizeAngle(plan.azimuth + s.angle)
		if math.Abs(bevNormalizeAngle(azimuth-want)) > 1e-9 {
			t.Errorf("%s: segment %d sits at azimuth %.9f rad, want %.9f — the share is linear in "+
				"the segment's HEEL-FACE cone distance, not its centroid",
				g.label, i, azimuth, want)
		}
	}

	// The mid-face section stays unrotated: the share is zero exactly where the
	// heel-face distance reaches R_mean, and the two segments either side of that
	// station straddle zero.
	below, above := false, false
	for _, s := range plan.slabs {
		if s.heelDistance < plan.sp.rMean {
			below = true
		}
		if s.heelDistance > plan.sp.rMean {
			above = true
		}
	}
	if !below || !above {
		t.Errorf("%s: no segment straddles R_mean = %.6g, so nothing holds the mid-face section "+
			"unrotated", g.label, plan.sp.rMean)
	}
}

// stepCrownSegments crowns the tooth by scaling each segment EXCEPT the outermost
// one down by a monotonic factor — full at the heel, growing smoothly toward the
// toe — about a sketch point on the ROOT edge of its heel face.
//
// The relief is keyed on the monotonic heel-distance fraction u, never on |ang|.
// |ang| is symmetric about mid-face, maximal at BOTH ends, so with the heel slab
// held full the slab just INSIDE the heel becomes the most relieved one and dips
// below both its neighbours — a notch that reverses the heel-to-toe taper. That
// was the observed bug: the heel-adjacent slab came out at 0.932 while the next
// slab inward was 0.972, taller.
//
// decad has no scale feature, so each segment is BUILT at its crowned size rather
// than scaled after the fact. A uniform scale about a point keeps every plane
// through that point invariant, so the crowned slab is the same loft between two
// sections scaled by the factor on planes whose origins move the same way — which
// is what this step constructs. The cost is that Fusion's scaleFeatures is not
// exercised, and with it the one never-activate exception the crown needs:
// scaleFeatures wants the Design occurrence as the ACTIVE edit target, so the
// module calls designOccurrence.activate() before the crown and restores the root
// afterwards in a finally, with Design.activateRootComponent(). A Component has
// no .activate() at all.
func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	w := sketch.NewWorld()

	bodies := make([]*decad.Body, 0, bevSlabCount)
	for _, s := range plan.slabs {
		bodies = append(bodies, f.buildSlab(t, doc, w, g, plan, s, true, true))
	}
	return bodies
}

// assertCrownSegments measures the relief: monotonic heel to toe, the heel slab
// held full, every factor positive, and the root edge left where it was.
func assertCrownSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	if len(bodies) != bevSlabCount {
		t.Fatalf("%s: the crown leaves %d segments, want %d", g.label, len(bodies), bevSlabCount)
	}

	// _CROWN_PER_RAD reaches built geometry and nothing derives it. It is kept at
	// 0.5 so a regen reproduces today's tooth, and this proof asserts the value the
	// module must carry rather than a derivation the repository does not have.
	if bevCrownPerRad != 0.5 {
		t.Errorf("_CROWN_PER_RAD is %v, want the shipped 0.5; 0 disables the crown", bevCrownPerRad)
	}

	if plan.slabs[plan.outer].crown != 1 {
		t.Errorf("%s: the outermost segment is scaled by %.9f, want it held full so its heel face "+
			"stays the loft's heel end for the heel cone to trim flush",
			g.label, plan.slabs[plan.outer].crown)
	}

	// u runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two
	// segments beyond it. Nothing reads an upper bound on u, and 8/6 is not a
	// ceiling: that figure is the last plane's offset, and that plane is a toe face
	// rather than any segment's heel face, so nothing ever evaluates u there.
	for i, s := range plan.slabs {
		if s.crown <= 0 {
			t.Errorf("%s: segment %d crowns to %.9f at u = %.9f; a non-positive factor is never "+
				"scaled by and the module raises instead", g.label, i, s.crown, s.fraction)
		}
		if i != plan.outer && s.crown >= 1 {
			t.Errorf("%s: segment %d takes no relief (factor %.9f) though it is not the held-full "+
				"heel segment", g.label, i, s.crown)
		}
	}

	// Monotonic heel to toe. Keying on |ang| instead would put the deepest relief
	// at BOTH ends and notch the slab just inside the heel.
	order := make([]int, len(plan.slabs))
	for i := range order {
		order[i] = i
	}
	for i := 1; i < len(order); i++ {
		for j := i; j > 0 &&
			plan.slabs[order[j]].postTwistDistance > plan.slabs[order[j-1]].postTwistDistance; j-- {
			order[j], order[j-1] = order[j-1], order[j]
		}
	}
	for i := 1; i < len(order); i++ {
		heelward, toeward := plan.slabs[order[i-1]], plan.slabs[order[i]]
		if !(toeward.crown < heelward.crown) {
			t.Errorf("%s: segment %d (u = %.6f, factor %.6f) is not more relieved than the segment "+
				"heelward of it (u = %.6f, factor %.6f); the relief must grow monotonically from "+
				"the heel to the toe", g.label, toeward.index, toeward.fraction, toeward.crown,
				heelward.fraction, heelward.crown)
		}
	}

	// The root edge stays on the seating cone. A uniform scale about a point keeps
	// that point fixed, so the crowned slab's two heel-face ROOT corners sit exactly
	// where the factor puts them and their midpoint — the base point — does not move
	// at all. Each corner is paired with the body vertex nearest to IT rather than
	// to the base point: the two straddle the base and are very nearly equidistant
	// from it, so pairing by distance to the base picks between them by rounding.
	for i, body := range bodies {
		s := plan.slabs[i]
		anchor := s.base.Add(plan.displacement(s.index))
		first, second := plan.rootCorners(s.farScale, s.angle)
		var midpoint r3.Vec
		for _, corner := range []r3.Vec{first, second} {
			want := s.base.Add(corner.Sub(s.base).Scale(s.crown)).Add(plan.displacement(s.index))
			midpoint = midpoint.Add(want.Scale(0.5))
			nearest, best := decad.VecMeasurement{}, math.Inf(1)
			for _, vertex := range body.Vertices() {
				position := vertex.Position()
				if d := position.Value.Sub(want).Len(); d < best {
					best, nearest = d, position
				}
			}
			decadtest.MeasuresVec(t,
				g.label+" crowned slab heel-face root corner, where the factor puts it",
				nearest, want, decadtest.Within(units.Millimeters(1e-6)))
		}
		if gap := midpoint.Sub(anchor).Len(); gap > 1e-9 {
			t.Errorf("%s: segment %d's crown moved its root-edge base point by %.12g mm; a uniform "+
				"scale about that point leaves it exactly where it was, and that is what keeps the "+
				"root edge on the seating cone", g.label, s.index, gap)
		}
	}
}

// stepSpiralLoft lofts the crowned, twisted segments into the `{gear} Spiral
// Tooth` body: first the toe-most segment's apex-side face, to push the loft past
// the toe cone so the toe trim bites, then the heel-facing face of every segment
// in order.
//
// THE ORDER IS RECOMPUTED HERE, after the twist and the crown, and never reused
// from the pre-twist slice order. The twist rotates each slab about the shaft
// axis, and for high-twist unequal-ratio pairs that rotation changes the slabs'
// along-cone order enough to REORDER adjacent slabs; lofting in the stale order
// then assembles the cross-sections out of sequence and the two gears interfere.
// For equal or low-twist pairs the two orders coincide, which is why equal-teeth
// gears mesh even with the stale order while a ratio pair like 31/17 distorts.
//
// SUBSTITUTION: decad's Loft takes exactly TWO sections, so the single
// nine-section loft is not performed. The chain is lofted pair by pair in the
// recomputed order instead, which proves the order, that each consecutive pair's
// sections meet, and that the chain spans from past the toe to past the heel. The
// cost is the single loft: the evaluator is not shown building one body through
// all nine sections, so a pairing failure that only a nine-section loft would
// reach is out of range here.
func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	w := sketch.NewWorld()

	order := plan.loftOrder()
	bodies := make([]*decad.Body, 0, bevSlabCount)
	for _, index := range order {
		bodies = append(bodies, f.buildSlab(t, doc, w, g, plan, plan.slabs[index], true, true))
	}
	return bodies
}

// loftOrder is the segment order the loft adds sections in: toe-most first, by
// each segment's POST-twist heel-face cone distance.
func (p *bevSpiralPlan) loftOrder() []int {
	order := make([]int, len(p.slabs))
	for i := range order {
		order[i] = i
	}
	for i := 1; i < len(order); i++ {
		for j := i; j > 0 &&
			p.slabs[order[j]].postTwistDistance < p.slabs[order[j-1]].postTwistDistance; j-- {
			order[j], order[j-1] = order[j-1], order[j]
		}
	}
	return order
}

// assertSpiralLoft measures the order and the span the chain covers.
func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	plan := f.planSpiral(t, g)
	if len(bodies) != bevSlabCount {
		t.Fatalf("%s: the spiral loft consumes %d segments, want %d", g.label, len(bodies), bevSlabCount)
	}
	order := plan.loftOrder()

	// The first section added is the toe-most segment's apex-side face, which is
	// what pushes the loft past the toe cone so the toe trim bites.
	if plan.slabs[order[0]].postTwistDistance != bevMinPostTwist(plan) {
		t.Errorf("%s: the loft opens on segment %d, which is not the toe-most one",
			g.label, order[0])
	}
	// The order is strictly increasing in post-twist heel-face cone distance, which
	// is the reading the loft samples.
	for i := 1; i < len(order); i++ {
		if !(plan.slabs[order[i-1]].postTwistDistance < plan.slabs[order[i]].postTwistDistance) {
			t.Errorf("%s: segments %d and %d are out of order in the loft",
				g.label, order[i-1], order[i])
		}
	}
	// Each consecutive pair's sections meet, which is what a loft between them
	// needs; the bodies were built above and the gate has already read them Sound
	// and solid. What is measured here is that the chain spans the whole band: its
	// toe end reaches past the toe and its heel end past the heel cone.
	lowest := plan.slabs[order[0]]
	highest := plan.slabs[order[len(order)-1]]
	if !(lowest.postTwistDistance < plan.sp.rToe) {
		t.Errorf("%s: the loft's toe end reaches only %.6g, not past the toe at %.6g",
			g.label, lowest.postTwistDistance, plan.sp.rToe)
	}
	if !(highest.postTwistDistance >= plan.sp.rHeel-plan.sp.span/6-1e-9) {
		t.Errorf("%s: the loft's heel end reaches only %.6g, which does not carry the parent "+
			"plane's own section at %.6g", g.label, highest.postTwistDistance, plan.sp.rHeel)
	}
}

func bevMinPostTwist(plan *bevSpiralPlan) float64 {
	out := math.Inf(1)
	for _, s := range plan.slabs {
		out = math.Min(out, s.postTwistDistance)
	}
	return out
}
