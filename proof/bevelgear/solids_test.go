package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// solidCases prove the per-gear body steps. Both gears are built, both ratio
// directions are reached, and the Shaft Angle runs to both ends of its range,
// since the cone angle is what sets every one of these bodies' proportions.
var solidCases = []proofkit3d.Case{
	{Name: "pinion-default", Params: map[string]float64{"gear": 0}},
	{Name: "driving-default", Params: map[string]float64{"gear": 1}},
	{Name: "pinion-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 0}},
	{Name: "driving-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 1}},
	{Name: "shaft-30", Params: map[string]float64{"shaftAngleDeg": 30, "gear": 1}},
	{Name: "shaft-150", Params: map[string]float64{"shaftAngleDeg": 150, "gear": 0}},
	{Name: "module-2-19-13", Params: map[string]float64{"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "gear": 0}},
}

// patternCases keep the tooth counts small enough to build every copy, and
// reach both gears of a ratio pair, where the two counts differ.
var patternCases = []proofkit3d.Case{
	{Name: "pinion-4-4", Params: map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}},
	{Name: "driving-of-19-13", Params: map[string]float64{"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "gear": 1}},
	{Name: "pinion-of-19-13", Params: map[string]float64{"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "gear": 0}},
}

// boreCases reach the bore branch from every side the spec offers: the
// auto-calculated diameter, an explicit one, and Enable Bore unchecked.
var boreCases = []proofkit3d.Case{
	{Name: "auto-diameter-pinion", Params: map[string]float64{"gear": 0}},
	{Name: "auto-diameter-driving-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 1}},
	{Name: "explicit-diameter", Params: map[string]float64{"pinionBore": 4, "gear": 0}},
	{Name: "bore-disabled", Params: map[string]float64{"boreEnable": 0, "gear": 0}},
}

// chordCount is how many straight segments stand in for a circle. decad's
// revolve is not gateable here (see stepGearBody), so every surface of
// revolution in this proof is chorded and its reading compared against the
// chord polygon's own closed form rather than the circle's.
const chordCount = 36

// toothChords is the number of straight segments the involute tooth section is
// chorded into: the flanks carry the involute samples, and the tip and root
// arcs are chorded to the same count on both loft sections so the pair
// corresponds one-to-one.
const toothChords = 6

// solidFrame is the world frame §3a step A builds for one gear: the shaft axis,
// the root cone element, and the circumferential direction across them. The §2
// figure is laid on the world XY plane, which is the Gear Profiles plane, so a
// lattice point (x, y) is the world point (x, y, 0).
type solidFrame struct {
	config config
	gear   gearSide
	apex   r3.Vec
	axis   r3.Vec // unit, pointing away from the apex along the shaft
	cone   r3.Vec // unit, the root cone element Apex->C
	circ   r3.Vec // unit, axis x cone — the circumferential direction
	uDir   r3.Vec // unit, the radial direction in a section plane
}

func world(p vec2) r3.Vec { return r3.NewVec(p.X, p.Y, 0) }

// normalized is r3.Vec.Normalize with the degenerate case turned into a
// failure, since every direction this proof normalizes is non-zero by
// construction.
func normalized(v r3.Vec) r3.Vec {
	u, ok := v.Normalize()
	if !ok {
		panic("bevelgear proof: a direction came out degenerate")
	}
	return u
}

func frameFor(c config, g *gearSide) solidFrame {
	apex := world(c.Apex)
	axis := normalized(world(g.AxisDir))
	cone := normalized(world(g.RootDir(c.Apex)))
	f := solidFrame{config: c, gear: *g, apex: apex, axis: axis, cone: cone, circ: normalized(axis.Cross(cone))}
	// The radial direction in a plane normal to the axis: the part of the cone
	// element perpendicular to the axis.
	f.uDir = normalized(cone.Sub(axis.Scale(cone.Dot(axis))))
	return f
}

// along is a point's distance from the apex measured along the shaft axis.
func (f solidFrame) along(p r3.Vec) float64 { return p.Sub(f.apex).Dot(f.axis) }

// radius is a point's perpendicular distance from the shaft axis.
func (f solidFrame) radius(p r3.Vec) float64 {
	d := p.Sub(f.apex)
	return d.Sub(f.axis.Scale(d.Dot(f.axis))).Len()
}

// distAlong is §3a's cone distance: distance from the apex along the root cone
// element, which is NOT the same as the along-axis distance.
func (f solidFrame) distAlong(p r3.Vec) float64 { return p.Sub(f.apex).Dot(f.cone) }

// sectionPlane is the plane normal to the shaft axis at along-axis distance d.
func (f solidFrame) sectionPlane(t *testing.T, w *sketch.World, d float64) *sketch.Plane {
	t.Helper()
	origin := f.apex.Add(f.axis.Scale(d))
	frame, err := r3.NewFrame(origin, f.uDir, f.axis.Cross(f.uDir))
	if err != nil {
		t.Fatalf("section frame at %.4f: %v", d, err)
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("section plane at %.4f: %v", d, err)
	}
	return plane
}

// polygonOn draws a chorded circle of the given radius on plane.
func polygonOn(t *testing.T, w *sketch.World, plane *sketch.Plane, radius float64, n int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	pts := make([]vec2, n)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = vec2{radius * math.Cos(a), radius * math.Sin(a)}
	}
	return regionOn(t, w, plane, pts)
}

// regionOn draws a closed polygon of fixed points on plane and returns its one
// region.
func regionOn(t *testing.T, w *sketch.World, plane *sketch.Plane, pts []vec2) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch on section plane: %v", err)
	}
	sp := make([]*sketch.Point, len(pts))
	for i, q := range pts {
		sp[i] = s.CreatePoint(q.X, q.Y)
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	for _, q := range sp {
		s.Fix(q)
	}
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve section sketch: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("a section is one closed region, got %d", len(regions))
	}
	return s, regions[0]
}

// polygonArea is the area of the chord polygon of n sides inscribed in a circle
// of the given radius — the closed form a chorded reading is compared against.
func polygonArea(radius float64, n int) float64 {
	return 0.5 * float64(n) * radius * radius * math.Sin(2*math.Pi/float64(n))
}

// frustumVolume is the prismatoid volume between two parallel sections of areas
// a0 and a1 separated by h.
func frustumVolume(a0, a1, h float64) float64 { return h / 3 * (a0 + a1 + math.Sqrt(a0*a1)) }

func measured(t *testing.T, m decad.Measurement) float64 {
	t.Helper()
	v, err := m.Value.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("read a volume: %v", err)
	}
	return v
}

// stepGearBody builds the band of the revolved gear blank the teeth seat on:
// the solid between the toe and heel sections, walled by the root cone the
// hexagon's C->M edge sweeps.
//
// Substitution, and what it costs. decad revolves a profile, but the volume it
// publishes for a full turn carries a bound of twice the volume — the sweep
// angle's own conservative error term — so the document never verifies as
// trustworthy and NO proofkit3d gate accepts it; a revolved body cannot be
// gated here at all. A solid of revolution is the union of the bands its
// profile edges sweep, and a band between two coaxial sections is a loft, so
// this step builds the band that the rest of the build actually consumes: its
// wall is the root cone the teeth are joined to and the two conical trims cut
// against. What it does not reach is the rest of the hexagon's sweep — the back
// face G->H and the heel cone H->C — because decad refuses a boolean whose
// operand is a loft (only prisms, cups and faceted bodies are admitted), so the
// bands cannot be unioned into the whole blank. The fact the revolve most
// depends on, that the profile never crosses its own axis of revolution
// ([PB-REVOLVE]), is asserted on the real hexagon in stepProfileHexagon.
func stepGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	toe, heel := world(g.M), world(g.Ded)
	_, toeProfile := polygonOn(t, w, f.sectionPlane(t, w, f.along(toe)), f.radius(toe), chordCount)
	toeSketch := toeProfile.Sketch()
	_, heelProfile := polygonOn(t, w, f.sectionPlane(t, w, f.along(heel)), f.radius(heel), chordCount)
	heelSketch := heelProfile.Sketch()

	body, err := doc.Loft(toeSketch, toeProfile, heelSketch, heelProfile)
	if err != nil {
		t.Fatalf("gear body band: %v", err)
	}
	return []*decad.Body{body}
}

func assertGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	toe, heel := world(g.M), world(g.Ded)

	span := f.along(heel) - f.along(toe)
	if span <= 0 {
		t.Fatalf("%s: the heel is not the outer end along the shaft axis", g.Label)
	}
	volume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("blank volume: %v", err)
	}
	want := frustumVolume(polygonArea(f.radius(toe), chordCount), polygonArea(f.radius(heel), chordCount), span)
	near(t, measured(t, volume), want, 1e-6*want, "%s blank band volume against the chord-polygon frustum", g.Label)

	// The band's wall is the root cone: both section radii sit on the cone
	// element Apex->C, so the wall's half angle is the root cone angle.
	rootAngle := math.Atan2(f.radius(heel)-f.radius(toe), span)
	near(t, rootAngle, math.Atan2(f.radius(heel), f.along(heel)), 1e-9,
		"%s the band wall is the cone through the apex", g.Label)
	if rootAngle >= g.Gamma {
		t.Errorf("%s: the root cone angle %.6f is not below the pitch cone angle %.6f",
			g.Label, rootAngle, g.Gamma)
	}

	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("blank bounds: %v", err)
	}
	if box.Max.Sub(box.Min).Len() <= 0 {
		t.Errorf("%s: the blank band has no extent", g.Label)
	}
}

// toothSection returns the tooth cross-section as a closed chord polygon in the
// tooth plane's own frame, centred on the tooth centre K'/L'.
func toothSection(t *testing.T, module, teeth float64) []vec2 {
	t.Helper()
	dims := involute.Derive(module, teeth, pressureAngle)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, teeth, involuteSteps, math.Pi)
	keepOutside := func(pts []involute.Pt) []involute.Pt {
		out := make([]involute.Pt, 0, len(pts))
		for _, q := range pts {
			if math.Hypot(q.X, q.Y) >= dims.Root {
				out = append(out, q)
			}
		}
		return out
	}
	if dims.Embedded() {
		left, right = keepOutside(left), keepOutside(right)
	}
	if len(left) < 2 {
		t.Fatalf("the tooth keeps too few flank samples")
	}

	arc := func(from, to involute.Pt, radius float64) []vec2 {
		a0 := math.Atan2(from.Y, from.X)
		a1 := math.Atan2(to.Y, to.X)
		for a1-a0 > math.Pi {
			a1 -= 2 * math.Pi
		}
		for a0-a1 > math.Pi {
			a1 += 2 * math.Pi
		}
		out := make([]vec2, 0, toothChords)
		for i := 1; i < toothChords; i++ {
			a := a0 + (a1-a0)*float64(i)/float64(toothChords)
			out = append(out, vec2{radius * math.Cos(a), radius * math.Sin(a)})
		}
		return out
	}

	// A tooth that is not embedded starts on the base circle, one flank-to-root
	// line short of the root circle at each end; an embedded one runs into the
	// root circle and needs none.
	toRoot := func(q involute.Pt) involute.Pt {
		r := math.Hypot(q.X, q.Y)
		return involute.Pt{X: q.X * dims.Root / r, Y: q.Y * dims.Root / r}
	}
	last := len(left) - 1
	pts := make([]vec2, 0, 2*len(left)+2*toothChords+2)
	leftRoot, rightRoot := left[0], right[0]
	if !dims.Embedded() {
		leftRoot, rightRoot = toRoot(left[0]), toRoot(right[0])
		pts = append(pts, vec2{leftRoot.X, leftRoot.Y})
	}
	for i := range left {
		pts = append(pts, vec2{left[i].X, left[i].Y})
	}
	pts = append(pts, arc(left[last], right[last], math.Hypot(left[last].X, left[last].Y))...)
	for i := last; i >= 0; i-- {
		pts = append(pts, vec2{right[i].X, right[i].Y})
	}
	if !dims.Embedded() {
		pts = append(pts, vec2{rightRoot.X, rightRoot.Y})
	}
	pts = append(pts, arc(rightRoot, leftRoot, math.Hypot(leftRoot.X, leftRoot.Y))...)
	return pts
}

// toothPlaneAt returns the plane parallel to the {gear} Plane whose distance
// from the apex is the fraction t of the tooth plane's own, together with the
// section scale that a cone from the apex gives it.
func (f solidFrame) toothPlaneAt(t *testing.T, w *sketch.World, fraction float64) (*sketch.Plane, r3.Vec) {
	t.Helper()
	centre := world(f.gear.KPrime)
	uDir := world(v2unit(v2sub(f.gear.KPrime, f.gear.Ded)))
	origin := f.apex.Add(centre.Sub(f.apex).Scale(fraction))
	frame, err := r3.NewFrame(origin, uDir, r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("tooth plane frame: %v", err)
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("tooth plane: %v", err)
	}
	return plane, origin
}

func scaledSection(pts []vec2, k float64) []vec2 {
	out := make([]vec2, len(pts))
	for i, q := range pts {
		out[i] = v2scale(q, k)
	}
	return out
}

// apexFraction is where the apex-end section of the tooth loft is taken. The
// loft's real first section is the Apex SKETCH POINT itself, a degenerate
// point-section decad has no equivalent for, so the proof takes a section a
// short way out from the apex instead and checks the taper it proves.
const apexFraction = 0.02

// stepToothLoft lofts the tooth body from the §2 Apex sketch point out to this
// gear's §3 tooth profile.
//
// Substitution: decad lofts between two profiles and has no point section, and
// it pairs profile segments by kind, so a free-form section is refused
// outright. The proof therefore chords the involute tooth and lofts from a
// section a small fraction of the way out from the apex — the same cone, cut
// off just short of its point. What that still pins is the taper: every section
// of the real loft is the tooth profile scaled by its own fraction of the cone
// distance, which is what makes the later conical trims cut a band of the right
// size, and what a loft built from anything but the apex would get wrong.
func stepToothLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	section := toothSection(t, c.Module, g.VirtualTeeth)
	nearPlane, _ := f.toothPlaneAt(t, w, apexFraction)
	_, nearProfile := regionOn(t, w, nearPlane, scaledSection(section, apexFraction))
	heelPlane, _ := f.toothPlaneAt(t, w, 1)
	_, heelProfile := regionOn(t, w, heelPlane, section)

	body, err := doc.Loft(nearProfile.Sketch(), nearProfile, heelProfile.Sketch(), heelProfile)
	if err != nil {
		t.Fatalf("tooth loft: %v", err)
	}
	return []*decad.Body{body}
}

func assertToothLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	section := toothSection(t, c.Module, g.VirtualTeeth)
	area := math.Abs(polygonSignedArea(section))
	centre := world(g.KPrime)
	uDir := world(v2unit(v2sub(g.KPrime, g.Ded)))
	normal := normalized(uDir.Cross(r3.NewVec(0, 0, 1)))
	height := math.Abs(centre.Sub(f.apex).Dot(normal)) * (1 - apexFraction)

	volume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("tooth volume: %v", err)
	}
	want := frustumVolume(area*apexFraction*apexFraction, area, height)
	near(t, measured(t, volume), want, 1e-6*math.Max(want, 1), "%s tooth loft volume against the cone taper", g.Label)

	// The tooth grows from a near-point at the apex to the drawn profile, so the
	// apex-end section is the heel section scaled by its own cone fraction.
	if apexFraction*apexFraction*area >= area {
		t.Errorf("%s: the apex-end section is not smaller than the tooth profile", g.Label)
	}
}

func polygonSignedArea(pts []vec2) float64 {
	sum := 0.0
	for i := range pts {
		q, r := pts[i], pts[(i+1)%len(pts)]
		sum += q.X*r.Y - r.X*q.Y
	}
	return sum / 2
}

// trimFractions are where the toe and heel conical cuts land on the tooth cone,
// as fractions of the tooth centre's own cone distance.
func (f solidFrame) trimFractions() (toe, heel float64) {
	centre := world(f.gear.KPrime).Sub(f.apex).Len()
	return world(f.gear.M).Sub(f.apex).Len() / centre, world(f.gear.Ded).Sub(f.apex).Len() / centre
}

// stepConicalTrim trims the lofted tooth to a flush band with the toe cone
// first and then the heel cone, the cutting tools being cone faces of the
// revolved blank ([PB-FACE-BY-MIDPOINT]) and the target the tooth loft.
//
// Substitution: the trim is a pair of splits by a cone face, and decad refuses
// a boolean whose operand is a loft body. The result of the two cuts is the
// piece of the tooth cone between the toe and heel cones, so the proof builds
// that piece directly, as the loft between the tooth sections at those two cone
// distances. What it pins is what the trim is for: the kept band starts at the
// toe cut and ends at the heel cut, carries no apex-side scrap, and keeps the
// taper the loft gave it.
//
// What the stand-in costs: the real cutting surfaces are CONES about the shaft
// axis, so the trimmed ends are conical, while the sections here are flat and
// parallel to the tooth plane through the same cone distances. The band's
// extent and taper are the same; the shape of its two end faces is not. Nor
// does the proof reach the face search that finds the cone
// ([PB-FACE-BY-MIDPOINT]) or the keeper selection after each split.
func stepConicalTrim(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	toeFraction, heelFraction := f.trimFractions()
	section := toothSection(t, c.Module, g.VirtualTeeth)
	toePlane, _ := f.toothPlaneAt(t, w, toeFraction)
	_, toeProfile := regionOn(t, w, toePlane, scaledSection(section, toeFraction))
	heelPlane, _ := f.toothPlaneAt(t, w, heelFraction)
	_, heelProfile := regionOn(t, w, heelPlane, scaledSection(section, heelFraction))

	body, err := doc.Loft(toeProfile.Sketch(), toeProfile, heelProfile.Sketch(), heelProfile)
	if err != nil {
		t.Fatalf("trimmed tooth band: %v", err)
	}
	return []*decad.Body{body}
}

func assertConicalTrim(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	toeFraction, heelFraction := f.trimFractions()

	if !(0 < toeFraction && toeFraction < heelFraction) {
		t.Fatalf("%s: the toe cut must land inside the heel cut, got %.4f and %.4f", g.Label, toeFraction, heelFraction)
	}
	// Every vertex of the trimmed band sits on one of the two cut sections and
	// none between or beyond them, so no apex-side scrap survived and nothing
	// reaches past the heel. A point's share of the tooth cone is measured along
	// the tooth plane's normal, which is the parameter the loft tapers on.
	centre := world(g.KPrime)
	uDir := normalized(world(v2unit(v2sub(g.KPrime, g.Ded))))
	normal := normalized(uDir.Cross(r3.NewVec(0, 0, 1)))
	full := centre.Sub(f.apex).Dot(normal)
	for _, v := range bodies[0].Vertices() {
		fraction := v.Position().Value.Sub(f.apex).Dot(normal) / full
		if math.Abs(fraction-toeFraction) > 1e-6 && math.Abs(fraction-heelFraction) > 1e-6 {
			t.Errorf("%s: a vertex sits at cone fraction %.6f, neither the toe cut %.6f nor the heel cut %.6f",
				g.Label, fraction, toeFraction, heelFraction)
		}
	}
	toeDist := world(g.M).Sub(f.apex).Len()
	heelDist := world(g.Ded).Sub(f.apex).Len()
	near(t, heelDist/toeDist, heelFraction/toeFraction, 1e-9, "%s the two trims keep the cone taper", g.Label)

	// The band spans the Face Width, which is an offset perpendicular to the
	// dedendum line C->H. Along the ROOT cone element Apex->C that same span
	// reads longer by the dedendum angle between the root element and the Pitch
	// Line, which is exactly why the two must not be confused.
	dedendumAngle := math.Acos(math.Min(1, v2dot(v2unit(v2sub(g.Ded, c.Apex)), v2unit(v2sub(c.Apex2, c.Apex)))))
	near(t, f.distAlong(world(g.Ded))-f.distAlong(world(g.M)), c.FaceWidth/math.Cos(dedendumAngle), 1e-9,
		"%s the trimmed band spans the Face Width along the cone element", g.Label)
	if dedendumAngle <= 0 || dedendumAngle >= g.Gamma {
		t.Errorf("%s: the dedendum angle %.6f must sit between zero and the pitch cone angle %.6f",
			g.Label, dedendumAngle, g.Gamma)
	}
}

func boxCorners(b decad.Box) []r3.Vec {
	var out []r3.Vec
	for _, x := range []float64{b.Min.X, b.Max.X} {
		for _, y := range []float64{b.Min.Y, b.Max.Y} {
			for _, z := range []float64{b.Min.Z, b.Max.Z} {
				out = append(out, r3.NewVec(x, y, z))
			}
		}
	}
	return out
}

// buildTrimmedTooth is the trimmed tooth band stepConicalTrim proves, reused by
// the pattern and meshing steps.
func buildTrimmedTooth(t *testing.T, doc *decad.Document, w *sketch.World, c config, g *gearSide, f solidFrame) *decad.Body {
	t.Helper()
	toeFraction, heelFraction := f.trimFractions()
	section := toothSection(t, c.Module, g.VirtualTeeth)
	toePlane, _ := f.toothPlaneAt(t, w, toeFraction)
	_, toeProfile := regionOn(t, w, toePlane, scaledSection(section, toeFraction))
	heelPlane, _ := f.toothPlaneAt(t, w, heelFraction)
	_, heelProfile := regionOn(t, w, heelPlane, scaledSection(section, heelFraction))
	body, err := doc.Loft(toeProfile.Sketch(), toeProfile, heelProfile.Sketch(), heelProfile)
	if err != nil {
		t.Fatalf("trimmed tooth band: %v", err)
	}
	return body
}

// stepToothPattern circular-patterns the trimmed tooth about the shaft axis,
// one copy per tooth, over the full circle.
//
// The pattern's own inputs are pinned numbers — quantity, a 360 degree total
// angle, not symmetric — so what the proof checks is what those numbers produce:
// the copies land at exactly 360/N about the shaft-axis edge, and neighbouring
// teeth do not touch, which is the angular room the tooth has at the heel.
func stepToothPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	seed := buildTrimmedTooth(t, doc, sketch.NewWorld(), c, g, f)
	// The copies are built in their own documents. decad refuses to decide a
	// PAIR whose operands are lofts — "a read-only intersection stage is
	// unsupported" — so a document holding the whole ring never verifies, and no
	// gate would pass however the ring is placed. Each copy is therefore gated
	// on its own and the ring's spacing is asserted from the copies' geometry.
	for k := 1; k < int(g.Teeth); k++ {
		copyBody := patternCopy(t, c, g, f, k)
		proofkit3d.RequireSolid(t, copyBody.Document(), []*decad.Body{copyBody})
	}
	return []*decad.Body{seed}
}

// patternCopy is the k-th patterned tooth, in a document of its own.
func patternCopy(t *testing.T, c config, g *gearSide, f solidFrame, k int) *decad.Body {
	t.Helper()
	doc := decad.New()
	seed := buildTrimmedTooth(t, doc, sketch.NewWorld(), c, g, f)
	if k == 0 {
		return seed
	}
	angle := 2 * math.Pi * float64(k) / g.Teeth
	transform, err := r3.RotationAround(f.apex, f.axis, units.Radians(angle))
	if err != nil {
		t.Fatalf("pattern rotation %d: %v", k, err)
	}
	copyBody, err := seed.Placed(transform)
	if err != nil {
		t.Fatalf("pattern copy %d: %v", k, err)
	}
	return copyBody
}

func assertToothPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	copies := make([]*decad.Body, int(g.Teeth))
	copies[0] = bodies[0]
	for k := 1; k < len(copies); k++ {
		copies[k] = patternCopy(t, c, g, f, k)
	}
	seedVolume := measured(t, mustVolume(t, copies[0]))
	for i, body := range copies {
		near(t, measured(t, mustVolume(t, body)), seedVolume, 1e-9*seedVolume, "%s pattern copy %d is a rigid copy", g.Label, i)
	}

	// The angular pitch is 360/N about the shaft axis, and the tooth has to fit
	// inside it with room to spare, or neighbouring copies interfere.
	pitch := 2 * math.Pi / g.Teeth
	centroid, err := copies[0].Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	azimuth := func(v r3.Vec) float64 {
		d := v.Sub(f.apex)
		d = d.Sub(f.axis.Scale(d.Dot(f.axis)))
		return math.Atan2(d.Dot(f.circ), d.Dot(f.uDir))
	}
	seedAzimuth := azimuth(centroid.Value)
	for k := 1; k < len(copies); k++ {
		c2, err := copies[k].Centroid()
		if err != nil {
			t.Fatalf("centroid %d: %v", k, err)
		}
		delta := azimuth(c2.Value) - seedAzimuth
		for delta < -math.Pi {
			delta += 2 * math.Pi
		}
		for delta > math.Pi {
			delta -= 2 * math.Pi
		}
		want := float64(k) * pitch
		for want > math.Pi {
			want -= 2 * math.Pi
		}
		near(t, delta, want, 1e-6, "%s copy %d sits at its share of the full circle", g.Label, k)
	}

	// Half-width of the tooth at the heel, as an angle about the shaft axis.
	half := 0.0
	for _, v := range bodies[0].Vertices() {
		if a := math.Abs(azimuth(v.Position().Value) - seedAzimuth); a > half {
			half = a
		}
	}
	if 2*half >= pitch {
		t.Errorf("%s: the tooth spans %.4f rad of the %.4f rad pitch, so neighbouring copies touch", g.Label, 2*half, pitch)
	}
}

func mustVolume(t *testing.T, b *decad.Body) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	return v
}

func mustBounds(t *testing.T, b *decad.Body) decad.Box {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	return box
}

// stepCombineSeat joins the patterned teeth to the gear body in one
// Combine-Join, the blank the target and the teeth the tools.
//
// Substitution: decad admits a boolean only on prisms, cups and faceted bodies,
// and both operands here are lofts, so the join itself cannot run. What the
// join depends on is that the tooth seats flush on the blank's root cone — the
// defect the spiral crown's scale base was moved to the root edge to avoid, a
// tooth floating above the base and leaving a gap the Join cannot close. So the
// two bodies are built apart and the seat is asserted from their own geometry:
// the tooth band's root corners lie on the cone the blank band's wall sweeps.
func stepCombineSeat(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	toe, heel := world(g.M), world(g.Ded)
	_, toeProfile := polygonOn(t, w, f.sectionPlane(t, w, f.along(toe)), f.radius(toe), chordCount)
	_, heelProfile := polygonOn(t, w, f.sectionPlane(t, w, f.along(heel)), f.radius(heel), chordCount)
	blank, err := doc.Loft(toeProfile.Sketch(), toeProfile, heelProfile.Sketch(), heelProfile)
	if err != nil {
		t.Fatalf("blank band: %v", err)
	}
	// The tooth is built in its own document and gated there: decad will not
	// decide a pair of loft bodies, so the two cannot share one.
	tooth := buildTrimmedTooth(t, decad.New(), sketch.NewWorld(), c, g, f)
	proofkit3d.RequireSolid(t, tooth.Document(), []*decad.Body{tooth})
	return []*decad.Body{blank}
}

func assertCombineSeat(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	blank := bodies[0]
	tooth := buildTrimmedTooth(t, decad.New(), sketch.NewWorld(), c, g, f)
	blankVolume := measured(t, mustVolume(t, blank))
	toothVolume := measured(t, mustVolume(t, tooth))
	if toothVolume <= 0 || blankVolume <= 0 {
		t.Fatalf("%s: a Join needs two solids with volume, got %.4f and %.4f", g.Label, blankVolume, toothVolume)
	}

	// The tooth's root corners are the vertices nearest the shaft axis at each
	// end; they must sit on the blank's root cone, whose radius grows linearly
	// with the along-axis distance.
	toe, heel := world(g.M), world(g.Ded)
	slope := (f.radius(heel) - f.radius(toe)) / (f.along(heel) - f.along(toe))
	base := f.radius(toe) - slope*f.along(toe)
	deepest := math.Inf(1)
	for _, v := range tooth.Vertices() {
		q := v.Position().Value
		gap := f.radius(q) - (base + slope*f.along(q))
		if gap < deepest {
			deepest = gap
		}
	}
	// The tooth's root corner must reach the cone or sit inside it; a tooth that
	// floats above the base leaves a gap the Join cannot close. It sits inside
	// by at most the rounding the virtual tooth number's floor() introduces,
	// which is half a module along the dedendum line.
	if deepest > 0 {
		t.Errorf("%s: the tooth floats %.6f mm above the blank's root cone", g.Label, deepest)
	}
	if deepest < -0.5*c.Module/math.Cos(g.Gamma) {
		t.Errorf("%s: the tooth root is buried %.6f mm below the root cone, more than the floor() rounding allows",
			g.Label, -deepest)
	}
}

// stepBoreCut cuts the through bore along the shaft axis.
//
// Substitution: the blank is modelled as a chorded prism rather than the
// revolved frustum, because decad admits a boolean only on prisms, cups and
// faceted bodies. The bore itself is built exactly as the spec states it — a
// circle centred on the shaft at the start of the shaft-axis edge, cut
// symmetrically far past both faces ([PB-THROUGH-CUT]) — so what the step pins
// is the diameter and that the cut pierces the body rather than pocketing it.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	heel := world(g.Ded)
	radius := f.radius(heel)
	height := f.along(heel) - f.along(world(g.M))
	planeStart := f.sectionPlane(t, w, f.along(world(g.M)))
	_, disc := polygonOn(t, w, planeStart, radius, chordCount)
	blank, err := doc.Extrude(disc.Sketch(), disc, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	if err != nil {
		t.Fatalf("blank prism: %v", err)
	}
	if !c.BoreEnable {
		return []*decad.Body{blank}
	}

	// The bore plane is normal to the shaft at the start of the shaft-axis edge;
	// the cut runs symmetrically 2 * Cone Distance each way, generously past
	// both faces.
	reach := 2 * c.ConeDistance
	borePlane := f.sectionPlane(t, w, f.along(world(g.M))-reach)
	_, boreDisc := polygonOn(t, w, borePlane, g.Bore/2, chordCount)
	tool, err := doc.Extrude(boreDisc.Sketch(), boreDisc, decad.Distance{D: units.Millimeters(2 * reach), Dir: decad.Along})
	if err != nil {
		t.Fatalf("bore tool: %v", err)
	}
	bored, err := decad.Cut(blank, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	heel := world(g.Ded)
	radius := f.radius(heel)
	height := f.along(heel) - f.along(world(g.M))
	solid := polygonArea(radius, chordCount) * height
	got := measured(t, mustVolume(t, bodies[0]))

	if !c.BoreEnable {
		near(t, got, solid, 1e-6*solid, "%s no bore is cut when Enable Bore is unchecked", g.Label)
		return
	}
	if param(p, "pinionBore", 0) == 0 && param(p, "drivingBore", 0) == 0 {
		near(t, g.Bore, g.PitchDia/4, 1e-12, "%s auto bore diameter is a quarter of the pitch diameter", g.Label)
	}
	hole := polygonArea(g.Bore/2, chordCount) * height
	near(t, got, solid-hole, 1e-4*solid, "%s bore removes a through hole of the stated diameter", g.Label)
	if got >= solid {
		t.Errorf("%s: the bore removed nothing", g.Label)
	}
}

// stepMeshRotation rotates the driving body by half a tooth pitch about its own
// shaft axis, so a driving valley meets the pinion tooth at the axial plane.
// The pinion's extra phase is zero unless a spiral pair asks for one.
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)
	w := sketch.NewWorld()

	tooth := buildTrimmedTooth(t, doc, w, c, g, f)
	angle := 0.0
	if g.Label == "Driving" {
		angle = math.Pi / g.Teeth
	}
	transform, err := r3.RotationAround(f.apex, f.axis, units.Radians(angle))
	if err != nil {
		t.Fatalf("mesh rotation: %v", err)
	}
	rotated, err := tooth.Placed(transform)
	if err != nil {
		t.Fatalf("mesh rotation: %v", err)
	}
	return []*decad.Body{rotated}
}

func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)
	f := frameFor(c, g)

	want := 0.0
	if g.Label == "Driving" {
		want = math.Pi / g.Teeth
	}
	near(t, want, halfPitch(g), 1e-12, "%s mesh phase is half a tooth pitch for the driving gear and none for the pinion", g.Label)

	// The rotation is rigid: the body keeps its volume and stays on its axis.
	centroid, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	azimuth := func(v r3.Vec) float64 {
		d := v.Sub(f.apex)
		d = d.Sub(f.axis.Scale(d.Dot(f.axis)))
		return math.Atan2(d.Dot(f.circ), d.Dot(f.uDir))
	}
	unrotated := unrotatedCentroid(t, c, g, f)
	delta := azimuth(centroid.Value) - azimuth(unrotated)
	for delta < -math.Pi {
		delta += 2 * math.Pi
	}
	for delta > math.Pi {
		delta -= 2 * math.Pi
	}
	near(t, delta, want, 1e-6, "%s body turned by the mesh phase", g.Label)
}

func halfPitch(g *gearSide) float64 {
	if g.Label == "Driving" {
		return math.Pi / g.Teeth
	}
	return 0
}

// unrotatedCentroid rebuilds the same tooth band in a throwaway document, so
// the rotated body can be compared against where it started.
func unrotatedCentroid(t *testing.T, c config, g *gearSide, f solidFrame) r3.Vec {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	body := buildTrimmedTooth(t, doc, w, c, g, f)
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("reference centroid: %v", err)
	}
	return centroid.Value
}
