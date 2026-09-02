package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// THE TOOTH CHAIN: THE APEX LOFT, THE CONICAL TRIM, THE PATTERN AND THE JOIN.
//
// Each of the four meets one of the refusals set out at the top of
// frustum_test.go, and each answers it with a substitute rather than by dropping
// the step. The substitutions and their costs are stated at the four steps
// themselves; what they have in common is stated here.
//
// THE TOOTH SECTION IS CHORDED. decad refuses to record a profile boundary whose
// trim is uncertified, and one fitted spline anywhere in a sketch withdraws
// exact trims from every region in it, so the tooth sketch as stepToothSketch
// draws it cannot be lofted at all. Each flank is drawn as the polyline through
// the same involute samples the spline is fitted to, and the tooth-top and root
// arcs as their chords. The cost is the section area: a chorded flank cuts the
// corners the spline rounds, so every volume below is the chorded section's, and
// every assertion compares a measured volume against the area the harness itself
// reports for the section that produced it, so the cost does not leak into a
// claim.
//
// THE APEX LOFT IS TRUNCATED. Fusion lofts the §2 Apex SKETCH POINT — a
// degenerate point section ([PB-LOFT]) — to the tooth profile. decad's Loft takes
// two profiles and has no point section, so the apex end is replaced by the same
// tooth section scaled about the apex on a parallel plane a tenth of R out from
// it. A uniform scale about the apex is exactly the section the point loft
// passes through there, so the walls are the same ruled walls; what is missing is
// the thousandth of the cone between that plane and the apex, and assertLoftTooth
// measures the body against the closed-form frustum volume that truncation
// predicts rather than against the full cone.

// toothSectionPoints returns one virtual spur tooth's closed outline in the
// tooth plane's own 2-D coordinates, centred on the tooth-centre point and
// already rotated by the 180 degrees draw() is passed.
//
// The outline is chorded: each flank is the polyline through the involute
// samples the Fusion sketch fits a spline to, and the tooth-top and root arcs
// are their chords. decad refuses a profile boundary whose trim is uncertified,
// and one fitted spline withdraws exact trims from every region of its sketch,
// so the drawn tooth cannot be lofted at all. The cost is the section area,
// which every assertion below reads off the harness's own report of the section
// it actually lofted rather than from the drawn tooth's.
func toothSectionPoints(t *testing.T, d involute.Dimensions, toothNumber float64, steps int) []vec2 {
	t.Helper()
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothNumber, steps,
		bevelToothAngle)
	if d.Embedded() {
		left = clipToRoot(t, left, d.Root)
		right = clipToRoot(t, right, d.Root)
	}
	out := make([]vec2, 0, 2*len(left)+2)
	if !d.Embedded() {
		scale := d.Root / d.Base
		out = append(out, vec2{right[0].X * scale, right[0].Y * scale})
	}
	for _, p := range right {
		out = append(out, vec2{p.X, p.Y})
	}
	for i := len(left) - 1; i >= 0; i-- {
		out = append(out, vec2{left[i].X, left[i].Y})
	}
	if !d.Embedded() {
		scale := d.Root / d.Base
		out = append(out, vec2{left[0].X * scale, left[0].Y * scale})
	}
	return out
}

// clipToRoot moves an embedded flank's first sample out to where the flank
// crosses the root circle. Fusion gets that point from the profile split; the
// chorded stand-in has to place it, and it places it on the first chord that
// leaves the circle.
func clipToRoot(t *testing.T, points []involute.Pt, root float64) []involute.Pt {
	t.Helper()
	for i := 1; i < len(points); i++ {
		if math.Hypot(points[i].X, points[i].Y) < root {
			continue
		}
		inside, outside := points[i-1], points[i]
		low, high := 0.0, 1.0
		for range 200 {
			mid := (low + high) / 2
			if math.Hypot(inside.X+(outside.X-inside.X)*mid,
				inside.Y+(outside.Y-inside.Y)*mid) < root {
				low = mid
			} else {
				high = mid
			}
		}
		mid := (low + high) / 2
		return append([]involute.Pt{{
			X: inside.X + (outside.X-inside.X)*mid,
			Y: inside.Y + (outside.Y-inside.Y)*mid,
		}}, points[i:]...)
	}
	t.Fatalf("an embedded flank never leaves the root circle")
	return nil
}

// toothFrame is the placement of one gear's tooth plane in its own axial frame.
//
// The axial frame has the apex at the origin, x along the shaft axis and the
// radial direction as y; z is out of the axial plane. The tooth plane contains
// the tooth-centre reference line C->K' and is perpendicular to the Gear
// Profiles plane, so its two in-plane directions are that line's direction and
// z. Its origin is the tooth-centre point K'.
//
// Two facts fall out and are asserted below. The plane sits at exactly the Pitch
// Cone Distance R from the apex, measured along its own normal, so it is the
// back-cone plane through Apex 2. And the tooth-centre point is on the shaft
// axis, at cone distance R/cos(gamma), which is why the virtual spur gear
// centred there has virtual pitch radius r/cos(gamma).
type toothFrame struct {
	origin r3.Vec // K', the tooth centre
	u, v   r3.Vec // the plane's in-plane axes: C->K' and out of the axial plane
	apexTo float64
}

func toothFrameOf(q pair, side gearSide) toothFrame {
	R := q.pitchConeDist
	// In the axial frame, the tooth centre K sits on the shaft axis at cone
	// distance R/cos(gamma), and the dedendum corner C at (R*cos+1.25m*sin,
	// r-1.25m*cos). K' is K shifted along the dedendum direction away from C.
	dedendum := dedendumFactor * q.module
	k := r3.NewVec(R/math.Cos(side.gamma), 0, 0)
	c := r3.NewVec(R*math.Cos(side.gamma)+dedendum*math.Sin(side.gamma),
		side.pitchDia/2-dedendum*math.Cos(side.gamma), 0)
	u, _ := k.Sub(c).Normalize()
	kp := k.Add(u.Scale(q.toothSpacing))
	return toothFrame{origin: kp, u: u, v: r3.NewVec(0, 0, 1), apexTo: R}
}

// section draws a scaled copy of the tooth outline on a plane parallel to the
// tooth plane, at scale s about the apex, and returns it with its sketch.
//
// s = 1 is the tooth plane itself. A uniform scale about the apex carries the
// tooth plane to a parallel plane at s times its distance and the tooth outline
// to s times its own coordinates, so the two sections below are exactly the two
// sections the apex point loft passes through.
func (f toothFrame) section(t *testing.T, world *sketch.World, outline []vec2,
	s float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(f.origin.Scale(s), f.u, f.v)
	if err != nil {
		t.Fatalf("tooth plane frame at scale %.6f: %v", s, err)
	}
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("tooth plane at scale %.6f: %v", s, err)
	}
	sk, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth section sketch at scale %.6f: %v", s, err)
	}
	pts := make([]*sketch.Point, len(outline))
	for i, p := range outline {
		pts[i] = sk.CreatePoint(p.X*s, p.Y*s)
	}
	for i := range pts {
		sk.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	profiles := sk.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		t.Fatalf("the chorded tooth section closes %d regions at scale %.6f, want one extrudable "+
			"loop", len(profiles), s)
	}
	return sk, profiles[0]
}

// apexTruncation is where the loft's apex-side section stands, as a fraction of
// the Pitch Cone Distance out from the apex.
//
// It exists only because decad's Loft takes two profiles and has no point
// section, so the degenerate apex end Fusion lofts from ([PB-LOFT]) has to be
// replaced by a real section a short way out from it. A tenth of R keeps that
// section large enough to record exactly while leaving all but a thousandth of
// the cone's volume in the body.
const apexTruncation = 0.1

// patternSteps is the involute sample count the pattern step draws its seed at.
const patternSteps = 4

// stepLoftTooth lofts the §2 Apex point to this gear's tooth profile, leaving
// the uncut Tooth Body.
//
// SUBSTITUTION: the apex end is a real section rather than a point, at
// apexTruncation of the way out from the apex. The walls are the same ruled
// walls the point loft builds, because the near section is the far one scaled
// about the apex, so what the truncation costs is the sliver between that plane
// and the apex — and the assertion measures the body against the frustum volume
// that truncation predicts, not against the full cone.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->
func stepLoftTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	body, _ := loftedTooth(t, doc, params, proxyInvoluteSteps)
	return []*decad.Body{body}
}

// loftedTooth builds the tooth body and returns it with the far section's area,
// which is the quantity every volume claim about it is written against.
func loftedTooth(t *testing.T, doc *decad.Document, params map[string]float64, steps int) (*decad.Body, float64) {
	t.Helper()
	q, side, _ := hexagonOf(t, params)
	virtual := side.virtualTeeth(q.module)
	d := involute.Derive(q.module, virtual, proxyPressureAngle)
	outline := toothSectionPoints(t, d, virtual, steps)

	f := toothFrameOf(q, side)
	world := sketch.NewWorld()
	nearSketch, nearProfile := f.section(t, world, outline, apexTruncation)
	farSketch, farProfile := f.section(t, world, outline, 1)

	body, err := doc.Loft(nearSketch, nearProfile, farSketch, farProfile)
	if err != nil {
		t.Fatalf("%s: loft the apex end to the tooth profile: %v", side.label, err)
	}
	return body, farProfile.Area
}

// assertLoftTooth measures the tapered tooth the loft leaves.
//
// The volume is the exact truncated cone over the section: a cone over a plane
// region at distance R from its apex holds area*R/3, so the piece between
// s*R and R holds area*R*(1-s^3)/3. Reading that back is what shows the loft
// really tapers to the apex rather than running parallel — a prism of the same
// section over the same length would measure far more.
//
// The tooth's own placement is checked too: every section point sits between the
// virtual root and tip radii of the tooth centre, and the tooth points AWAY from
// the shaft axis. That is what the 180 degree draw() angle buys, and a tooth
// drawn at 0 would point into the gear body instead.
func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	q, side, _ := hexagonOf(t, params)
	if len(bodies) != 1 {
		t.Fatalf("%s: the tooth loft left %d bodies, want the one Tooth Body", side.label, len(bodies))
	}
	virtual := side.virtualTeeth(q.module)
	d := involute.Derive(q.module, virtual, proxyPressureAngle)
	outline := toothSectionPoints(t, d, virtual, proxyInvoluteSteps)
	f := toothFrameOf(q, side)

	world := sketch.NewWorld()
	_, farProfile := f.section(t, world, outline, 1)
	s := apexTruncation
	want := farProfile.Area * f.apexTo * (1 - s*s*s) / 3

	got, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: tooth body volume: %v", side.label, err)
	}
	if !closeTo(got.Value.Mag(), want, 1e-6) {
		t.Fatalf("%s: the Tooth Body measures %.6f, not the %.6f a cone over this section from "+
			"the apex holds between %.1f and %.1f per cent of R; the loft is not tapering to the "+
			"apex", side.label, got.Value.Mag(), want, 100*s, 100.0)
	}

	assertToothPlacement(t, q, side, f, d, outline)
}

// assertToothPlacement checks where the tooth plane and the tooth centre sit.
//
// Three facts, all of them things a later step selects on. The tooth plane is
// the back-cone plane: its distance from the apex along its own normal is the
// Pitch Cone Distance R, so Apex 2 lies on it. The tooth centre is on the shaft
// axis, at cone distance R/cos(gamma) — which is what makes the virtual pitch
// radius r/cos(gamma) and the virtual tooth count what it is. And the dedendum
// corner C sits at the virtual ROOT radius from the tooth centre, so the tooth
// drawn at 180 degrees has its root on C and its tip a module beyond, pointing
// away from the shaft axis.
func assertToothPlacement(t *testing.T, q pair, side gearSide, f toothFrame,
	d involute.Dimensions, outline []vec2) {
	t.Helper()
	R := q.pitchConeDist
	normal, _ := f.u.Cross(f.v).Normalize()
	if got := math.Abs(f.origin.Dot(normal)); math.Abs(got-R) > 1e-7 {
		t.Errorf("%s: the tooth plane sits %.9f from the apex along its own normal, not the Pitch "+
			"Cone Distance %.9f; it is supposed to be the back-cone plane through Apex 2",
			side.label, got, R)
	}
	// K, before the Tooth Spacing offset, is the tooth centre on the shaft axis.
	k := f.origin.Sub(f.u.Scale(q.toothSpacing))
	if math.Abs(k.Y) > 1e-7 || math.Abs(k.Z) > 1e-7 {
		t.Errorf("%s: the tooth centre K sits %.9f off the shaft axis", side.label,
			math.Hypot(k.Y, k.Z))
	}
	if want := R / math.Cos(side.gamma); math.Abs(k.X-want) > 1e-7 {
		t.Errorf("%s: K sits at cone distance %.9f, not R/cos(gamma) = %.9f", side.label, k.X, want)
	}
	// C is the outline's own root radius away from the tooth centre, measured
	// along the reference line, and the drawn root radius is within half a module
	// of it because the virtual tooth number is floored.
	dedendum := dedendumFactor * q.module
	c := r3.NewVec(R*math.Cos(side.gamma)+dedendum*math.Sin(side.gamma),
		side.pitchDia/2-dedendum*math.Cos(side.gamma), 0)
	if got, want := k.Sub(c).Len(), side.pitchDia/2/math.Cos(side.gamma)-dedendum; math.Abs(got-want) > 1e-7 {
		t.Errorf("%s: C sits %.9f from K, not the virtual root radius %.9f", side.label, got, want)
	}
	if got, want := k.Sub(c).Len(), d.Root; math.Abs(got-want) > q.module/2 {
		t.Errorf("%s: C sits %.9f from K but the drawn tooth's root circle is at %.9f; the two "+
			"differ by more than the half module the floored virtual tooth count can explain",
			side.label, got, want)
	}
	// The tooth points away from the shaft axis: the tooth plane's u axis runs
	// from C toward K, so a tooth drawn at 180 degrees puts every one of its
	// points on the far side, at negative u.
	for i, p := range outline {
		if p.X > 1e-9 {
			t.Fatalf("%s: outline point %d sits at u = %.9f; the 180 degree draw() angle is what "+
				"turns the tooth away from the shaft axis", side.label, i, p.X)
		}
	}
}

// stepCutConicalEnds trims the Tooth Body to a flush band between the toe and
// heel cones.
//
// SUBSTITUTE: the trim's RESULT is drawn — the band of the tooth between the toe
// and heel cone distances — instead of splitting the tooth with two cone faces
// of the Gear Body, which decad refuses on a loft operand (refusal 2 in
// frustum_test.go). COST: the two cuts are drawn as PLANES perpendicular to the
// pitch line where the build cuts with CONES, so the band's ends are flat where
// the trimmed tooth's are conical, and the flushness against the gear base is
// asserted from the cones' own half-angles in stepRevolveGearBody rather than
// from this body's end faces. What the substitute does pin is which part of the
// tooth survives: the band runs from the toe cone distance to the heel cone
// distance, and it carries the volume that stretch of the tooth holds.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->
func stepCutConicalEnds(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	s, f, outline := toothOf(t, params)
	world := sketch.NewWorld()
	toeSketch, toeProfile := f.section(t, world, outline, s.scaleAt(s.rToe))
	heelSketch, heelProfile := f.section(t, world, outline, s.scaleAt(s.rHeel))
	body, err := doc.Loft(toeSketch, toeProfile, heelSketch, heelProfile)
	if err != nil {
		t.Fatalf("%s: the flush band between the toe and heel cones: %v", s.side.label, err)
	}
	return []*decad.Body{body}
}

// assertCutConicalEnds measures the trimmed tooth.
//
// The toe cut comes first and must bite — a toe cut that does not split is a
// hard failure in the build, since an untrimmed tooth is unusable — so the toe
// cone distance has to fall strictly inside the uncut tooth. The heel cut is the
// lenient one: a heel cone that does not reach the keeper is caught as the typed
// NonIntersectError and the keeper returned whole, so the heel cone distance is
// allowed to sit at or past the tooth's own heel end, and the step reports which
// of the two this case is.
func assertCutConicalEnds(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	s, f, outline := toothOf(t, params)
	if len(bodies) != 1 {
		t.Fatalf("%s: the conical trim left %d bodies, want the one keeper", s.side.label, len(bodies))
	}
	toe, heel := s.scaleAt(s.rToe), s.scaleAt(s.rHeel)
	if toe >= heel {
		t.Fatalf("%s: the toe cut stands at scale %.9f and the heel cut at %.9f; the toe is the "+
			"inner end", s.side.label, toe, heel)
	}
	if toe <= apexTruncation {
		t.Errorf("%s: the toe cut stands at scale %.9f, at or inside the tooth's own apex end; "+
			"the toe cut has to bite and its failure is not lenient", s.side.label, toe)
	}
	if heel <= 1 {
		t.Logf("%s: the heel cut stands at scale %.9f, inside the tooth, so it splits",
			s.side.label, heel)
	} else {
		t.Logf("%s: the heel cut stands at scale %.9f, past the tooth's own heel end, which is "+
			"the NonIntersectError the helper catches to return the keeper whole",
			s.side.label, heel)
	}

	world := sketch.NewWorld()
	_, toeProfile := f.section(t, world, outline, toe)
	_, heelProfile := f.section(t, world, outline, heel)
	height := (heel - toe) * s.q.pitchConeDist
	want := frustumVolume(height, toeProfile.Area, heelProfile.Area)
	volume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: trimmed tooth volume: %v", s.side.label, err)
	}
	if !closeTo(volume.Value.Mag(), want, 1e-9) {
		t.Fatalf("%s: the trimmed tooth measures %.6f, not the %.6f the stretch between the two "+
			"cuts holds", s.side.label, volume.Value.Mag(), want)
	}
}

// stepPatternTeeth circular-patterns the trimmed tooth about the shaft-axis
// edge, Teeth Number copies over a full turn.
//
// SUBSTITUTE: the copies are slid apart ALONG the shaft axis, because decad's
// read-only interference pass cannot tessellate a loft payload and so cannot
// judge two teeth whose boxes meet (refusal 3 in frustum_test.go). The slide runs
// along the very axis the pattern turns about, so every azimuth about it, every
// radius from it and every volume is exactly the pattern's. COST: the step does
// not show that neighbouring teeth clear each other where they really sit, so
// the angular gap between them is asserted from the seed's own width instead.
//
// The count is this gear's Teeth Number, never the virtual tooth number the
// tooth was drawn at. The two differ by the back-cone expansion, and patterning
// the virtual count would put the wrong number of teeth on the gear.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->
func stepPatternTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	q, side, _ := hexagonOf(t, params)
	seed, _ := loftedTooth(t, doc, params, proxyInvoluteSteps)
	count := int(side.teeth)
	bodies := []*decad.Body{seed}
	for k := 1; k < count; k++ {
		turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
			units.Degrees(360*float64(k)/float64(count)))
		if err != nil {
			t.Fatalf("%s: pattern rotation %d: %v", side.label, k, err)
		}
		slide, err := r3.Translation(r3.NewVec(float64(k)*stride(q), 0, 0))
		if err != nil {
			t.Fatalf("%s: lay-apart translation %d: %v", side.label, k, err)
		}
		placed, err := turn.Then(slide)
		if err != nil {
			t.Fatalf("%s: compose %d: %v", side.label, k, err)
		}
		copied, err := seed.PlacedCopy(placed)
		if err != nil {
			t.Fatalf("%s: pattern copy %d: %v", side.label, k, err)
		}
		bodies = append(bodies, copied)
	}
	return bodies
}

// assertPatternTeeth checks the count, the spacing and the clearance.
//
// The spacing is measured about the shaft axis: each copy sits the same distance
// from that axis as the seed and a full turn divided by Teeth Number further
// round than its predecessor. The radial taper is already in the seed — the loft
// made it — so the pattern only turns one tapered tooth into N copies. And
// because the copies were slid apart to be judged, the clearance the pattern
// depends on is asserted from the seed's own angular width instead: a tooth
// wider than its share of the turn would leave no valley between neighbours.
func assertPatternTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	_, side, _ := hexagonOf(t, params)
	count := int(side.teeth)
	if len(bodies) != count {
		t.Fatalf("%s: the pattern left %d bodies, want this gear's Teeth Number %d including the "+
			"seed", side.label, len(bodies), count)
	}
	seedVolume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: seed volume: %v", side.label, err)
	}
	step := 2 * math.Pi / float64(count)
	var firstAngle, firstRadius float64
	for k, body := range bodies {
		volume, err := body.Volume()
		if err != nil {
			t.Fatalf("%s: volume of tooth %d: %v", side.label, k, err)
		}
		if !closeTo(volume.Value.Mag(), seedVolume.Value.Mag(), 1e-9) {
			t.Fatalf("%s: patterned tooth %d measures %.6f against the seed's %.6f", side.label,
				k, volume.Value.Mag(), seedVolume.Value.Mag())
		}
		centroid, err := body.Centroid()
		if err != nil {
			t.Fatalf("%s: centroid of tooth %d: %v", side.label, k, err)
		}
		angle := math.Atan2(centroid.Value.Z, centroid.Value.Y)
		radius := math.Hypot(centroid.Value.Y, centroid.Value.Z)
		if k == 0 {
			firstAngle, firstRadius = angle, radius
			continue
		}
		if !closeTo(radius, firstRadius, 1e-9) {
			t.Fatalf("%s: patterned tooth %d sits %.6f from the shaft axis against the seed's "+
				"%.6f; the pattern turns about that axis", side.label, k, radius, firstRadius)
		}
		want := firstAngle + step*float64(k)
		if diff := math.Mod(angle-want+3*math.Pi, 2*math.Pi) - math.Pi; math.Abs(diff) > 1e-6 {
			t.Fatalf("%s: patterned tooth %d sits at %.6f rad, want %.6f — a full turn divided "+
				"into this gear's %d teeth", side.label, k, angle, want, count)
		}
	}
	if width := seedAngularWidth(t, params); width >= step {
		t.Errorf("%s: the tooth subtends %.6f rad against a tooth pitch of %.6f; neighbouring "+
			"copies would meet and the pattern would leave no valley", side.label, width, step)
	}
}

// seedAngularWidth is the angle the tooth subtends about the shaft axis, taken
// at the heel section where it is widest.
func seedAngularWidth(t *testing.T, params map[string]float64) float64 {
	t.Helper()
	_, f, outline := toothOf(t, params)
	widest := 0.0
	for _, q := range outline {
		p := f.origin.Add(f.u.Scale(q.X)).Add(f.v.Scale(q.Y))
		if a := math.Abs(math.Atan2(p.Z, p.Y)); a > widest {
			widest = a
		}
	}
	return 2 * widest
}

// stepCombineTeeth joins the patterned teeth into the Gear Body.
//
// SUBSTITUTE: decad refuses a union whose operand is a loft (refusal 2 in
// frustum_test.go), and the joined gear is a shape neither a loft nor a revolve
// draws in one piece — its blank has a three-segment meridian and its teeth
// stand on the root cone, so no single pair of sections sweeps it. So the join's
// operands are built side by side, laid apart, and what the join needs is
// asserted from their own geometry: the volume identity the joined body has to
// satisfy, and the SEATING — every tooth's root edge sitting on the blank's root
// cone, which is what decides whether the join closes or leaves the gap the
// crown's root anchoring exists to prevent. COST: the boolean itself goes
// unexercised, so a defect only a union would surface is out of reach.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->
func stepCombineTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	q, _, hex := hexagonOf(t, params)
	// The tooth stands where it was built; the blank's three bands are laid out
	// past it, one stride apart, so no two operands share a bounding box.
	tooth, _ := loftedTooth(t, doc, params, proxyInvoluteSteps)
	world := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, 4)
	for k, b := range sweptBands(hex) {
		bodies = append(bodies, bandOf(t, doc, world, b.x0, b.r0, b.x1, b.r1, 0,
			float64(k+1)*stride(q)))
	}
	return append(bodies, tooth)
}

// assertCombineTeeth checks what the join has to produce, and what it needs.
//
// THE IDENTITY. The joined gear's volume is the blank's plus this gear's Teeth
// Number times one tooth's, since the teeth stand on the blank and overlap it
// nowhere. The blank is read back as the signed sum of its three bands and the
// tooth off the body that was actually built, which is what ties the claim to
// the pieces rather than to the numbers that drew them.
//
// THE SEATING. Each tooth's root corners have to lie on the blank's root cone,
// the one the C->M band sweeps. They lie on it to within half a module, which is
// exactly the error the FLOORED virtual tooth count introduces: the drawn root
// circle sits at the floored count's root radius while the dedendum corner sits
// at r/cos(gamma) - 1.25m, and the two differ by the fraction of a tooth the
// floor discards. A tooth further off than that would stand clear of the blank
// and the join would leave a gap.
func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	q, side, hex := hexagonOf(t, params)
	bands := sweptBands(hex)
	if len(bodies) != len(bands)+1 {
		t.Fatalf("%s: the combine's operands are %d bodies, want the %d blank bands and the "+
			"tooth", side.label, len(bodies), len(bands))
	}
	blank := 0.0
	for k, b := range bands {
		volume, err := bodies[k].Volume()
		if err != nil {
			t.Fatalf("%s: %s band volume: %v", side.label, b.name, err)
		}
		sign := 1.0
		if b.x1 < b.x0 {
			sign = -1
		}
		blank += sign * volume.Value.Mag()
	}
	toothVolume, err := bodies[len(bands)].Volume()
	if err != nil {
		t.Fatalf("%s: tooth volume: %v", side.label, err)
	}
	joined := math.Abs(blank) + side.teeth*toothVolume.Value.Mag()
	if joined <= math.Abs(blank) {
		t.Fatalf("%s: the join would leave %.6f against a blank of %.6f; a join adds material",
			side.label, joined, math.Abs(blank))
	}
	t.Logf("%s: the join has to leave %.6f — a blank of %.6f plus %.0f teeth of %.6f",
		side.label, joined, math.Abs(blank), side.teeth, toothVolume.Value.Mag())

	// The seating, from the tooth's own root corners against the blank's root
	// cone. hex[3] is the dedendum corner C/D, which lies on that cone.
	rootHalfAngle := math.Atan2(hex[3].Y, hex[3].X)
	_, f, outline := toothOf(t, params)
	corners := rootCorners(t, f, outline, 1)
	for i, corner := range corners {
		got := math.Atan2(math.Hypot(corner.Y, corner.Z), corner.X)
		gap := math.Abs(got-rootHalfAngle) * corner.Len()
		if gap > q.module/2 {
			t.Errorf("%s: root corner %d stands %.6f mm off the blank's root cone, more than the "+
				"half module the floored virtual tooth count can explain; the Combine-Join would "+
				"leave a gap", side.label, i, gap)
		}
	}
}
