package bevelgear_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ---------------------------------------------------------------------------
// The solid case table
// ---------------------------------------------------------------------------

// solidCases runs at Module 4 to 8 and never at Module 1: decad's mesh bound has
// an absolute floor, so a figure small enough brings every measurement inside it
// and the gate reports Suspect on geometry that is in fact correct. Module is a
// pure scale on this figure, so a case at Module 4 through 8 proves the same shape.
//
// Every configuration is carried twice, once per gear, because the two members
// differ in everything the solid steps read: gamma, the pitch radius, the base
// height and the tooth count.
var solidCases = solidTable([]struct {
	name string
	over map[string]float64
}{
	{"default_m4", map[string]float64{"module": 4}},
	{"module_8", map[string]float64{"module": 8}},
	{"ratio_driving_31_pinion_17", map[string]float64{"module": 4, "pinionTeeth": 17}},
	{"ratio_driving_17_pinion_31", map[string]float64{"module": 4, "drivingTeeth": 17}},
	{"teeth_16_12", map[string]float64{"module": 4, "drivingTeeth": 16, "pinionTeeth": 12}},
	{"teeth_4_4", map[string]float64{"module": 4, "drivingTeeth": 4, "pinionTeeth": 4}},
	{"shaft_angle_35", map[string]float64{"module": 4, "shaftAngleDeg": 35}},
	{"shaft_angle_60", map[string]float64{"module": 4, "shaftAngleDeg": 60}},
	{"shaft_angle_142", map[string]float64{"module": 4, "shaftAngleDeg": 142}},
	{"tooth_spacing_positive", map[string]float64{
		"module": 4, "drivingTeeth": 43, "pinionTeeth": 31, "shaftAngleDeg": 75, "toothSpacing": 0.5}},
	{"bore_disabled", map[string]float64{"module": 4, "boreDisable": 1}},
})

func solidTable(rows []struct {
	name string
	over map[string]float64
}) []proofkit3d.Case {
	out := make([]proofkit3d.Case, 0, 2*len(rows))
	for _, row := range rows {
		for _, side := range []struct {
			label string
			flag  float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := caseParams(row.over)
			p["drivingSide"] = side.flag
			out = append(out, proofkit3d.Case{Name: row.name + "_" + side.label, Params: p})
		}
	}
	return out
}

// caseMember is the gear one solid case builds. Reading it from the case table rather
// than building both members in one document keeps each case's readings its own.
func caseMember(f figure, p map[string]float64) member {
	if p["drivingSide"] == 1 {
		return f.driving
	}
	return f.pinion
}

// ---------------------------------------------------------------------------
// Shared solid fixtures
// ---------------------------------------------------------------------------

// axisU is the revolve axis in a sketch plane's own (u, v): the u axis through the
// plane origin. Every gear body in this proof is written about it.
var axisU = decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

// planeSketch returns a sketch on the world plane through origin with in-plane axes
// u and v. It is how the proof builds the real back-cone plane, tilted out of the
// axis-perpendicular by gamma, rather than substituting an axis-perpendicular one.
func planeSketch(t *testing.T, w *sketch.World, origin, u, v r3.Vec) *sketch.Sketch {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		t.Fatalf("frame: %v", err)
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("plane: %v", err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch: %v", err)
	}
	return s
}

// closedLoop draws a closed polygon of fixed points and returns its one valid
// region. Fixing the points after the lines exist is the [PB-PROJECT-NOT-FIXED]
// order: a bare point fixed before it is consumed as a line endpoint does not
// leave the sketch fully constrained.
func closedLoop(t *testing.T, s *sketch.Sketch, v []pt) *sketch.Profile {
	t.Helper()
	pts := make([]*sketch.Point, 0, len(v))
	for _, q := range v {
		pts = append(pts, s.CreatePoint(q.X, q.Y))
	}
	lines := make([]*sketch.Line, 0, len(pts))
	for i := range pts {
		lines = append(lines, s.CreateLine(pts[i], pts[(i+1)%len(pts)]))
	}
	for _, line := range lines {
		s.Fix(line.Start)
		s.Fix(line.End)
	}
	return decadtest.SolveRegion(t, s)
}

// mm3, mm2 and mm name the three Kinds the readings below carry, so a comparison
// cannot silently hand a length slack to a volume.
func mm3(v float64) units.Value { return units.CubicMillimeters(v) }
func mm2(v float64) units.Value { return units.SquareMillimeters(v) }
func mm(v float64) units.Value  { return units.Millimeters(v) }

// layApart moves a body clear of the others in its document.
//
// It is the ONE substitution every site below shares: build the operands, lay them
// apart along the shaft axis, and assert from their own measured geometry what the
// operation would have produced. A translation leaves every volume, radius and cone
// angle unchanged, which is what makes the readings still mean something. What it
// removes is a pair contact decad refuses to classify — two bodies that touch along
// a shared cut surface, or a tool that grazes its target, report
// unsupported_pair_contact and the gate admits no such diagnostic.
func layApart(t *testing.T, b *decad.Body, offset r3.Vec) *decad.Body {
	t.Helper()
	tr, err := r3.Translation(offset)
	if err != nil {
		t.Fatalf("lay apart: %v", err)
	}
	moved, err := b.Placed(tr)
	if err != nil {
		t.Fatalf("lay apart: %v", err)
	}
	return moved
}

// relSlack states an oracle's own error as a fraction of the value it claims.
func relSlack(v float64) decadtest.Option { return decadtest.WithinRel(units.Scalar(v)) }

// coneFaces returns a body's cone faces with their published half-angles, in
// radians. A decad.Cone publishes its own angle, so the proof reads the angle off
// the face rather than deriving a tangent from two cap radii and a height.
func coneFaces(b *decad.Body) []struct {
	Face      *decad.Face
	HalfAngle float64
	Apex      r3.Vec
	Axis      r3.Vec
} {
	var out []struct {
		Face      *decad.Face
		HalfAngle float64
		Apex      r3.Vec
		Axis      r3.Vec
	}
	for _, f := range b.Faces() {
		c, ok := f.Surface().(decad.Cone)
		if !ok {
			continue
		}
		half := c.HalfAngle.Base() // base angle unit is the radian
		// The apex sits where the wall radius reaches zero, walking back along the
		// axis from Origin, where the radius is Cone.Radius.
		radius := c.Radius.Base() // base length unit is the millimetre
		back := 0.0
		if tan := math.Tan(half); tan != 0 {
			back = radius / tan
		}
		out = append(out, struct {
			Face      *decad.Face
			HalfAngle float64
			Apex      r3.Vec
			Axis      r3.Vec
		}{f, half, c.Origin.Sub(c.Axis.Scale(back)), c.Axis})
	}
	return out
}

// ---------------------------------------------------------------------------
// The gear-body revolve
// ---------------------------------------------------------------------------

// stepRevolveGearBody revolves the §2 hexagon a full turn about the shaft-axis
// edge. SUBSTITUTE NOTHING: decad's own Revolve returns the whole frustum as one
// body, and its volume agrees with Pappus on that same hexagon with no polygon
// correction and no decomposition into bands.
//
// The gear body is the ONE body no boolean in this proof consumes. Measured over
// this gear's own bodies at the pinned decad revision, every boolean whose operands
// were both Lofts returned a Sound body, and every boolean with a Revolve operand
// verified Suspect instead — all of them — so a Revolve operand puts its step out
// of reach whatever the geometry is. A revolved frustum still clears the gate on
// its OWN readings, which is what this step uses it for.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	s := decadtest.NewSketch(t)
	profile := closedLoop(t, s, f.hexagon(g))
	body, err := doc.Revolve(s, profile, axisU, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s: revolve the hexagon: %v", g.label, err)
	}
	return []*decad.Body{body}
}

func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	hex := f.hexagon(g)
	body := bodies[0]

	// Pappus on the hexagon itself. The bound decad publishes for a revolve reading
	// is Approximate rather than Exact, so the comparison states a relative slack —
	// decadtest.Exactly fails on every case.
	vol, err := body.Volume()
	if err != nil {
		t.Fatalf("%s: volume: %v", g.label, err)
	}
	decadtest.Measures(t, g.label+" gear body volume (Pappus on the §2 hexagon)",
		vol, mm3(revolvedVolume(hex)), relSlack(1e-9))

	// Five faces, matched by surface kind and by their own readings rather than by
	// the order the face selector hands them back.
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindPlane: 2,
		decad.KindCone:  3,
	})

	heelR, toeR := hex[2].Y, hex[5].Y // H (resp. J) and N (resp. P)
	wantDiscs := []float64{math.Pi * heelR * heelR, math.Pi * toeR * toeR}
	var discs []float64
	for _, face := range body.Faces() {
		if face.Surface().Kind() != decad.KindPlane {
			continue
		}
		area, err := face.Area()
		if err != nil {
			t.Fatalf("%s: flat face area: %v", g.label, err)
		}
		discs = append(discs, area.Value.Base())
	}
	matchAreas(t, g.label+" flat faces", discs, wantDiscs)

	// The three cone faces are the frusta the profile edges H->C, C->M and M->N
	// sweep. The heel cone and the toe dish read 90 - gamma; the root cone reads this
	// gear's root cone angle.
	wantCones := []struct {
		area, half float64
	}{
		{coneFrustumArea(hex[2], hex[3]), math.Pi/2 - g.gamma}, // heel cone H->C
		{coneFrustumArea(hex[3], hex[4]), g.gammaRoot},         // root cone C->M
		{coneFrustumArea(hex[4], hex[5]), math.Pi/2 - g.gamma}, // toe dish M->N
	}
	cones := coneFaces(body)
	if len(cones) != len(wantCones) {
		t.Fatalf("%s: %d cone face(s), want %d", g.label, len(cones), len(wantCones))
	}
	used := make([]bool, len(cones))
	for _, want := range wantCones {
		best, bestErr := -1, math.Inf(1)
		for i, c := range cones {
			if used[i] {
				continue
			}
			area, err := c.Face.Area()
			if err != nil {
				t.Fatalf("%s: cone face area: %v", g.label, err)
			}
			if e := rel(area.Value.Base(), want.area); e < bestErr {
				best, bestErr = i, e
			}
		}
		if best < 0 || bestErr > 1e-6 {
			t.Fatalf("%s: no cone face sweeps %.4f mm² (nearest is off by %.3e)",
				g.label, want.area, bestErr)
		}
		used[best] = true
		if d := math.Abs(cones[best].HalfAngle - want.half); d > 1e-9 {
			t.Errorf("%s: the cone of area %.4f mm² publishes a half-angle of %.9f rad, want %.9f",
				g.label, want.area, cones[best].HalfAngle, want.half)
		}
	}
}

// matchAreas pairs measured areas with expected ones, best-first, so neither side
// depends on the order the face selector returns.
func matchAreas(t *testing.T, what string, got, want []float64) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("%s: %d face(s), want %d", what, len(got), len(want))
		return
	}
	used := make([]bool, len(got))
	for _, w := range want {
		best, bestErr := -1, math.Inf(1)
		for i, v := range got {
			if used[i] {
				continue
			}
			if e := rel(v, w); e < bestErr {
				best, bestErr = i, e
			}
		}
		if best < 0 || bestErr > 1e-6 {
			t.Errorf("%s: no face measures %.4f mm² (nearest is off by %.3e)", what, w, bestErr)
			return
		}
		used[best] = true
	}
}

// ---------------------------------------------------------------------------
// The tooth, in the frame every solid step below shares
// ---------------------------------------------------------------------------

// The shared frame: the Apex at the origin, +X along this gear's shaft axis
// (station), +Y radially outward toward this gear's own dedendum corner, +Z
// circumferential. It is the frame the §2 hexagon already lives in.
func vec(station, radius, circum float64) r3.Vec { return r3.NewVec(station, radius, circum) }

// toothFrame returns the tooth centre K'/L' and the two in-plane axes of the
// BACK-CONE plane, which is tilted out of the axis-perpendicular by gamma.
//
// ⚠️ The tooth is centred at K'/L', which is OFF the shaft axis whenever Tooth
// Spacing is positive: K/L is where the back-cone dedendum line crosses the axis,
// and K'/L' is one Tooth Spacing further along that line, which carries it PAST
// the axis to station R/cos(gamma) + Tooth Spacing * sin(gamma) and radius
// Tooth Spacing * cos(gamma) on the opposite side. Taking the station alone and
// seating the tooth plane's origin on the axis leaves every tooth point one
// Tooth Spacing * cos(gamma) too far out, which is the whole of the clearance the
// input asks for.
func (f figure) toothFrame(g member) (centre, radialOut, circum r3.Vec) {
	// The dedendum direction in this frame: station rises at sin(gamma) and radius
	// falls at cos(gamma) walking from Apex 2 toward the dedendum corner.
	ded := vec(math.Sin(g.gamma), -math.Cos(g.gamma), 0)
	apex2 := vec(f.pitchCone*math.Cos(g.gamma), g.pitchRadius, 0)
	centre = apex2.Add(ded.Scale(g.virtualPitchRadius + f.toothSpacing))
	// The drawn tooth's own outward radial direction runs from the centre back toward
	// Apex 2, which is what the 180-degree draw() angle delivers in Fusion: the spur
	// drawer centres its tooth on its own +X, and the rotation lands that on this
	// direction. The proof places the outline along it directly.
	radialOut = ded.Scale(-1)
	circum = vec(0, 0, 1)
	return centre, radialOut, circum
}

// toothOutline is one tooth's closed cross-section in its own back-cone plane, in
// (radial, circumferential) millimetres about the tooth centre, walked once round.
// The tooth's centreline is +radial.
func (f figure) toothOutline(g member, samples int) []pt {
	m := f.module
	vpr := g.virtualPitchRadius
	tipR := vpr + addendumFactor*m
	rootR := vpr - dedendumFactor*m - f.rootSink

	left, right, embedded := virtualFlanks(m, vpr, f.rootSink, g.virtualTeeth)

	out := make([]pt, 0, 4*samples)
	arc := func(from, to pt, r float64) {
		a0 := math.Atan2(from.Y, from.X)
		a1 := math.Atan2(to.Y, to.X)
		for i := 1; i < samples; i++ {
			a := a0 + (a1-a0)*float64(i)/float64(samples)
			out = append(out, pt{r * math.Cos(a), r * math.Sin(a)})
		}
	}
	radial := func(q pt, r float64) pt {
		k := r / math.Hypot(q.X, q.Y)
		return pt{q.X * k, q.Y * k}
	}

	// left flank, base (or root) outward to the tip
	out = append(out, left...)
	tipL := out[len(out)-1]
	tipR2 := right[len(right)-1]
	arc(tipL, tipR2, tipR)
	// right flank, tip inward to the base
	for i := len(right) - 1; i >= 0; i-- {
		out = append(out, right[i])
	}
	rootRight := out[len(out)-1]
	rootLeft := left[0]
	if !embedded {
		// The flank starts outside the root circle, so the drawer adds one radial stub
		// per flank down to the root arc — the two connecting lines the non-embedded
		// tooth carries.
		rootRight = radial(rootRight, rootR)
		rootLeft = radial(rootLeft, rootR)
		out = append(out, rootRight)
	}
	arc(rootRight, rootLeft, rootR)
	if !embedded {
		out = append(out, rootLeft)
	}
	return out
}

// place lifts a point of the tooth's own plane into the shared frame, scaled about
// the Apex by k. k = 1 is the heel section; a smaller k is the section the loft's
// degenerate apex end is substituted by.
func (f figure) place(g member, q pt, k float64) r3.Vec {
	centre, radialOut, circum := f.toothFrame(g)
	p := centre.Add(radialOut.Scale(q.X)).Add(circum.Scale(q.Y))
	return p.Scale(k)
}

// toothSection returns the plane sketch and profile for one tooth cross-section,
// scaled about the Apex by k.
func (f figure) toothSection(t *testing.T, w *sketch.World, g member, outline []pt, k int64,
	scale float64) (*sketch.Sketch, *sketch.Profile) {
	centre, radialOut, circum := f.toothFrame(g)
	s := planeSketch(t, w, centre.Scale(scale), radialOut, circum)
	local := make([]pt, 0, len(outline))
	for _, q := range outline {
		local = append(local, pt{q.X * scale, q.Y * scale})
	}
	return s, closedLoop(t, s, local)
}

// apexDistance is the Apex's perpendicular distance to this gear's back-cone tooth
// plane — sK * cos(gamma) rather than sK, which is what makes it the Pitch Cone
// Distance at Tooth Spacing 0.
func (f figure) apexDistance(g member) float64 {
	centre, radialOut, circum := f.toothFrame(g)
	n := radialOut.Cross(circum)
	unit, ok := n.Normalize()
	if !ok {
		return math.NaN()
	}
	return math.Abs(centre.Dot(unit))
}

// ---------------------------------------------------------------------------
// The apex loft
// ---------------------------------------------------------------------------

// apexShrink is the section the loft's degenerate apex end is substituted by.
// patternShrink is the same substitution taken further out, for the one step whose
// neighbours would otherwise sit inside decad's near-contact band.
const (
	apexShrink    = 0.1
	patternShrink = 0.55
)

// stepLoftTooth lofts the Apex to this gear's tooth profile.
//
// Substitution, and what it costs: the degenerate apex POINT section is replaced by
// the same tooth section shrunken about the Apex, because a point section is not a
// profile this evaluator lofts. Nothing else is substituted — in particular the
// tooth plane is the real back-cone plane, tilted out of the axis-perpendicular by
// gamma, and the Apex's perpendicular distance to it is sK * cos(gamma). THE COST IS
// THE POINT SECTION: the loft's degenerate end is not built, and what the volume and
// the section radii prove is the taper it has to produce.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 8)
	s0, p0 := f.toothSection(t, w, g, outline, 0, apexShrink)
	s1, p1 := f.toothSection(t, w, g, outline, 1, 1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: apex loft: %v", g.label, err)
	}
	return []*decad.Body{body}
}

func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	outline := f.toothOutline(g, 8)
	area := polygonArea(outline)
	d := f.apexDistance(g)

	// A cone from the Apex through the heel section, its apex end cut off at
	// apexShrink: V = A * d * (1 - k^3) / 3.
	want := area * d * (1 - apexShrink*apexShrink*apexShrink) / 3
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: tooth volume: %v", g.label, err)
	}
	decadtest.Measures(t, g.label+" tooth volume (the apex cone, truncated at the shrunken section)",
		vol, mm3(want), relSlack(1e-9))

	// The built tooth reaches out to the virtual tip radius laid on the back cone.
	tip := pt{g.virtualPitchRadius + addendumFactor*f.module, 0}
	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s: tooth bounds: %v", g.label, err)
	}
	decadtest.Encloses(t, g.label+" tooth reaches the virtual tip radius on the back cone",
		bounds, f.place(g, tip, 1))

	// At Tooth Spacing 0 the Apex's perpendicular distance to the tooth plane is the
	// Pitch Cone Distance itself.
	if f.toothSpacing == 0 && rel(d, f.pitchCone) > 1e-12 {
		t.Errorf("%s: the Apex sits %.6f mm from the tooth plane, want the Pitch Cone Distance %.6f",
			g.label, d, f.pitchCone)
	}
}

// ---------------------------------------------------------------------------
// The conical end cut
// ---------------------------------------------------------------------------

// coneSolid builds the solid inside a cone about the shaft axis: apex on the axis
// at station apexStation, opening toward the apex end, wall slope cot(gamma). It is
// an n-gon loft, because the cut consumes it and a boolean operand has to be a
// body rather than a band spanning one profile edge.
func coneSolid(t *testing.T, doc *decad.Document, w *sketch.World, apexStation, slope, reach float64,
	sides int) *decad.Body {
	disc := func(station, radius float64) (*sketch.Sketch, *sketch.Profile) {
		s := planeSketch(t, w, vec(station, 0, 0), vec(0, 1, 0), vec(0, 0, 1))
		poly := make([]pt, 0, sides)
		for i := range sides {
			a := 2 * math.Pi * float64(i) / float64(sides)
			poly = append(poly, pt{radius * math.Cos(a), radius * math.Sin(a)})
		}
		return s, closedLoop(t, s, poly)
	}
	s0, p0 := disc(apexStation-1e-3, 1e-3*slope)
	s1, p1 := disc(apexStation-reach, reach*slope)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("cutting cone: %v", err)
	}
	return body
}

// stepConicalEndCut performs the TOE cut on the tooth and substitutes for the heel
// cut. Where each cut lands is read the same way for both: the gear body is built,
// each cone's apex and half-angle and each of the tooth's two surfaces are read off
// the bodies themselves, the stations where they cross are solved from those
// readings, and those stations are checked against the flush band.
//
// Perform no heel cut, because its cone is TANGENT to the tooth plane: the dedendum
// corner C/D and the tooth centre K'/L' both sit on this gear's back-cone dedendum
// line, so the tooth plane contains a generator of the heel cone and the two touch
// along the tooth's own centreline instead of crossing it. decad refuses exactly
// that contact, and rebuilding the cone as a Revolve replaces the refusal with a
// Suspect verdict the gate does not admit either. The heel cone is therefore laid
// apart from the tooth and only the three readings are kept. THE COST IS THE HEEL
// SPLIT: for that end the proof does not show the evaluator dividing the tooth,
// selecting the keeper, or leaving a watertight body.
//
// ⚠️ What the readings cannot tell apart: a cone and a TILTED PLANE read identically
// in everything this step measures. In the axial section a cone of half-angle
// 90 - gamma and a plane tilted by gamma through the same generator are the same
// line, so every station solved here comes out the same for either surface. What
// makes the face conical is that it is a surface of revolution about the shaft
// axis — its crossing with the tooth's tip surface sits at one station at every
// azimuth, while a tilted plane's crossing moves with azimuth. The half-angle read
// here comes off a decad.Cone face of the revolved gear body, which is a surface of
// revolution by its own construction and publishes its angle rather than having one
// derived. Nothing here measures the azimuthal crossing of the tool the toe cut
// consumes.
func stepConicalEndCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	w := sketch.NewWorld()

	// The gear body, for its published cone faces alone. It is the cone-face source
	// the ConeSurfaceType search finds at this step in Fusion, and no boolean below
	// consumes it.
	gearSketch := decadtest.NewSketch(t)
	gearProfile := closedLoop(t, gearSketch, f.hexagon(g))
	gearBody, err := doc.Revolve(gearSketch, gearProfile, axisU, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s: gear body: %v", g.label, err)
	}

	outline := f.toothOutline(g, 6)
	s0, p0 := f.toothSection(t, w, g, outline, 0, apexShrink)
	s1, p1 := f.toothSection(t, w, g, outline, 1, 1)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: tooth: %v", g.label, err)
	}
	toothCopy, err := tooth.Duplicate()
	if err != nil {
		t.Fatalf("%s: duplicate the tooth: %v", g.label, err)
	}

	slope := 1 / math.Tan(g.gamma) // the toe cone's wall slope, half-angle 90 - gamma
	apexStation := f.toeConeApexStation(g)
	cone := coneSolid(t, doc, w, apexStation, slope, apexStation, 48)
	coneCopy, err := cone.Duplicate()
	if err != nil {
		t.Fatalf("%s: duplicate the cutting cone: %v", g.label, err)
	}

	out := []*decad.Body{gearBody}
	clear := 4 * f.apexDed

	// ⚠️ Tolerate exactly one typed refusal here and fail on any other error. An empty
	// toe trim is the same condition the generated module raises as
	// solids.NonIntersectError: the toe cone takes nothing off that tooth, and the
	// piece outside the cone is the whole of it.
	offcut, err := decad.Intersect(tooth, cone)
	switch {
	case err == nil:
		out = append(out, layApart(t, offcut, vec(0, 0, clear)))
	case isBooleanEmpty(err):
		proofkit3d.Unmodelled(t, "%s: the toe cone takes nothing off this tooth (%v), which is "+
			"the condition the generated module raises as solids.NonIntersectError", g.label, err)
		return out
	default:
		t.Fatalf("%s: toe cut, the piece inside the cone: %v", g.label, err)
	}

	keeper, err := decad.Cut(toothCopy, coneCopy)
	if err != nil {
		t.Fatalf("%s: toe cut, the piece outside the cone: %v", g.label, err)
	}
	out = append(out, layApart(t, keeper, vec(0, 0, -clear)))
	return out
}

// isBooleanEmpty reports decad's typed BooleanEmpty refusal.
func isBooleanEmpty(err error) bool {
	var be *decad.BooleanError
	return errors.As(err, &be) && be.Code == decad.BooleanEmpty
}

// toeConeApexStation is where the toe cone's apex sits on the shaft axis: walking
// out the dedendum direction from the toe corner M/O, the radius reaches zero at
// station + radius * tan(gamma).
func (f figure) toeConeApexStation(g member) float64 {
	name := "M"
	if g.label == "Driving" {
		name = "O"
	}
	m := f.pts[name]
	return f.stationOf(g, m) + f.radiusOf(g, m)*math.Tan(g.gamma)
}

func assertConicalEndCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	gearBody := bodies[0]

	// Take BOTH half-angles off the revolved gear body's own cone faces. A decad.Cone
	// publishes its angle, so nothing is derived from two cap radii and a height.
	var heel, toe, root float64 = math.NaN(), math.NaN(), math.NaN()
	hex := f.hexagon(g)
	for _, c := range coneFaces(gearBody) {
		area, err := c.Face.Area()
		if err != nil {
			t.Fatalf("%s: cone face area: %v", g.label, err)
		}
		switch a := area.Value.Base(); {
		case rel(a, coneFrustumArea(hex[2], hex[3])) < 1e-6:
			heel = c.HalfAngle
		case rel(a, coneFrustumArea(hex[3], hex[4])) < 1e-6:
			root = c.HalfAngle
		case rel(a, coneFrustumArea(hex[4], hex[5])) < 1e-6:
			toe = c.HalfAngle
		}
	}
	back := math.Pi/2 - g.gamma
	for _, c := range []struct {
		name string
		got  float64
	}{{"heel", heel}, {"toe", toe}} {
		if math.Abs(c.got-back) > 1e-9 {
			t.Errorf("%s: the %s cutting cone's half-angle reads %.9f rad, want the back-cone "+
				"half-angle 90° - gamma = %.9f", g.label, c.name, c.got, back)
		}
	}

	// Each cut lands where the flush band requires: the toe cone meets the gear body's
	// own root cone at M/O, the heel cone at C/D. Both cones have their apex on the
	// shaft axis, so a cone of wall slope k and apex station a crosses a ray of slope
	// m at a*k/(m+k).
	// A cone whose apex sits on the shaft axis has wall slope tan(half-angle) in
	// radius per station, so it crosses a ray of slope m at a*k/(m+k).
	cross := func(apexStation, half, raySlope float64) float64 {
		k := math.Tan(half)
		return apexStation * k / (raySlope + k)
	}
	rootSlope := math.Tan(root)
	toeName, heelName := "M", "C"
	if g.label == "Driving" {
		toeName, heelName = "O", "D"
	}
	if got, want := cross(f.toeConeApexStation(g), toe, rootSlope), f.stationOf(g, f.pts[toeName]); rel(got, want) > 1e-9 {
		t.Errorf("%s: the toe cone meets the root cone at station %.6f, want %s at %.6f",
			g.label, got, toeName, want)
	}
	heelApex := f.pitchCone / math.Cos(g.gamma) // K/L, where the dedendum line meets the axis
	if got, want := cross(heelApex, heel, rootSlope), f.stationOf(g, f.pts[heelName]); rel(got, want) > 1e-9 {
		t.Errorf("%s: the heel cone meets the root cone at station %.6f, want %s at %.6f",
			g.label, got, heelName, want)
	}

	// Each cone crosses the tooth's TIP inboard of where it crosses the tooth's ROOT,
	// so the trimmed end is shorter at the tip than at the root. (Nothing asserts
	// merely that the two crossings differ: every cut with a finite tilt crosses both
	// surfaces at different stations, so such an assertion passes on any figure this
	// spec can build.)
	outline := f.toothOutline(g, 6)
	tipSlope, rootTooth := f.toothSurfaceSlopes(g, outline)
	for _, c := range []struct {
		name  string
		apexS float64
		half  float64
	}{{"toe", f.toeConeApexStation(g), toe}, {"heel", heelApex, heel}} {
		tipAt := cross(c.apexS, c.half, tipSlope)
		rootAt := cross(c.apexS, c.half, rootTooth)
		if tipAt >= rootAt {
			t.Errorf("%s: the %s cone crosses the tooth tip at station %.6f and its root at "+
				"%.6f; the trimmed end must be shorter at the tip",
				g.label, c.name, tipAt, rootAt)
		}
	}

	// The toe split itself. The keeper and the offcut add back to the whole tooth, and
	// the gate above has already required each piece to be one lump and solid.
	area := polygonArea(outline)
	d := f.apexDistance(g)
	whole := area * d * (1 - apexShrink*apexShrink*apexShrink) / 3
	sum := 0.0
	for _, b := range bodies[1:] {
		v, err := b.Volume()
		if err != nil {
			t.Fatalf("%s: toe piece volume: %v", g.label, err)
		}
		sum += v.Value.Base()
	}
	if rel(sum, whole) > 1e-6 {
		t.Errorf("%s: the two toe pieces measure %.6f mm³ against a whole tooth of %.6f",
			g.label, sum, whole)
	}
}

// toothSurfaceSlopes is the radius-per-station slope of the tooth's tip surface and
// of its root surface, measured from the built outline's own extremes: both
// surfaces run from the Apex, so each slope is the outline point's perpendicular
// distance from the shaft axis over its station.
func (f figure) toothSurfaceSlopes(g member, outline []pt) (tip, root float64) {
	tip, root = 0, math.Inf(1)
	for _, q := range outline {
		w := f.place(g, q, 1)
		slope := math.Hypot(w.Y, w.Z) / w.X
		tip = math.Max(tip, slope)
		root = math.Min(root, slope)
	}
	return tip, root
}

// ---------------------------------------------------------------------------
// The circular pattern
// ---------------------------------------------------------------------------

// patternSeed holds the readings stepCircularPattern takes on its seed tooth
// during the build.
//
// ⚠️ THIS STEP IS SERIAL, AND THESE VARIABLES ARE WHY. The pattern increment retires
// the seed, so the seed cannot be measured after the step runs: its azimuth, radius,
// height and volume have to be read during the build and handed to the assertion,
// and that hand-off leaves the case. Two cases running at once overwrite each
// other's readings, and the proof then reports a wrong verdict rather than failing
// loudly — the two gear sides differ enough in volume that the overwrite was caught
// when it happened, but a pair of cases whose seeds measured alike would have passed
// on each other's numbers. Every other bevel step carries nothing from its build to
// its assertion and runs on the parallel runner; a step that acquires a reading
// moves to the serial one in the same change.
var patternSeed struct {
	volume  float64
	azimuth float64
	radius  float64
	height  float64
}

// stepCircularPattern rotates one tooth into this gear's Teeth Number of copies
// about the shaft-axis edge. The pattern is pinned: quantity = Teeth Number,
// totalAngle = 360 degrees, isSymmetric = false. Although the pitch diameter shrinks
// from the heel toward the apex, the ANGULAR spacing stays 360/N for the whole face
// width — the radial taper is already produced by the loft from the Apex.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	w := sketch.NewWorld()
	// This step's loft stops at patternShrink rather than at apexShrink. Toward the
	// Apex every tooth's section shrinks with it, so the gap between two neighbours
	// shrinks in proportion: at apexShrink the neighbours sit a few tenths of a
	// millimetre apart and decad reports undecided_pair on the near-contact —
	// "the disjoint/overlap partition proof resolved neither way" — which the gate
	// admits no more than any other Suspect verdict. The angular spacing this step
	// measures is the same at every station, so a seed that starts further out proves
	// the same increment. THE COST IS THE APEX END: the pattern is not shown on the
	// part of the tooth nearest the Apex.
	outline := f.toothOutline(g, 3)
	s0, p0 := f.toothSection(t, w, g, outline, 0, patternShrink)
	s1, p1 := f.toothSection(t, w, g, outline, 1, 1)
	seed, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: seed tooth: %v", g.label, err)
	}

	vol, err := seed.Volume()
	if err != nil {
		t.Fatalf("%s: seed volume: %v", g.label, err)
	}
	centre, err := seed.Centroid()
	if err != nil {
		t.Fatalf("%s: seed centroid: %v", g.label, err)
	}
	bounds, err := seed.Bounds()
	if err != nil {
		t.Fatalf("%s: seed bounds: %v", g.label, err)
	}
	patternSeed.volume = vol.Value.Base()
	patternSeed.azimuth = math.Atan2(centre.Value.Z, centre.Value.Y)
	patternSeed.radius = math.Hypot(centre.Value.Y, centre.Value.Z)
	patternSeed.height = bounds.Max.X - bounds.Min.X

	// Every copy is laid apart ALONG THE SHAFT AXIS after its rotation. A translation
	// along that axis leaves the azimuth, the distance from the axis, the volume and
	// the span unchanged — which is every quantity this step measures — and it is what
	// keeps the N bodies out of each other's way. Left where the pattern puts them,
	// decad reports undecided_pair on the neighbours ("the disjoint/overlap partition
	// proof resolved neither way") from three teeth up, and the gate admits no Suspect
	// verdict. THE COST IS THE ARRANGEMENT: the proof does not show the N copies
	// standing clear of one another around one axis.
	n := int(g.teeth)
	axis := vec(1, 0, 0)
	origin := vec(0, 0, 0)
	spacing := 3 * patternSeed.height
	out := make([]*decad.Body, 0, n)
	for i := 1; i < n; i++ {
		tr, err := r3.RotationAround(origin, axis, units.Radians(2*math.Pi*float64(i)/float64(n)))
		if err != nil {
			t.Fatalf("%s: pattern increment %d: %v", g.label, i, err)
		}
		copyBody, err := seed.PlacedCopy(tr)
		if err != nil {
			t.Fatalf("%s: pattern increment %d: %v", g.label, i, err)
		}
		out = append(out, layApart(t, copyBody, vec(float64(i)*spacing, 0, 0)))
	}
	// The last increment consumes the seed, which is what retires it.
	last, err := seed.Placed(r3.Identity())
	if err != nil {
		t.Fatalf("%s: retire the seed: %v", g.label, err)
	}
	return append(out, last)
}

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	n := int(g.teeth)
	if len(bodies) != n {
		t.Fatalf("%s: the pattern left %d bodies, want quantity = %d teeth", g.label, len(bodies), n)
	}

	seen := make([]bool, n)
	for _, b := range bodies {
		vol, err := b.Volume()
		if err != nil {
			t.Fatalf("%s: copy volume: %v", g.label, err)
		}
		decadtest.Measures(t, g.label+" patterned tooth volume (against the seed's own reading)",
			vol, mm3(patternSeed.volume), relSlack(1e-9))

		centre, err := b.Centroid()
		if err != nil {
			t.Fatalf("%s: copy centroid: %v", g.label, err)
		}
		if got := math.Hypot(centre.Value.Y, centre.Value.Z); rel(got, patternSeed.radius) > 1e-9 {
			t.Errorf("%s: a copy's centroid sits %.6f mm from the shaft axis, want the seed's %.6f",
				g.label, got, patternSeed.radius)
		}
		bounds, err := b.Bounds()
		if err != nil {
			t.Fatalf("%s: copy bounds: %v", g.label, err)
		}
		if got := bounds.Max.X - bounds.Min.X; rel(got, patternSeed.height) > 1e-6 {
			t.Errorf("%s: a copy spans %.6f mm along the shaft, want the seed's %.6f",
				g.label, got, patternSeed.height)
		}

		// Every copy sits at an exact multiple of 360/N from the seed, and each multiple
		// is taken exactly once: the full circle, not symmetric about the seed.
		delta := math.Mod(math.Atan2(centre.Value.Z, centre.Value.Y)-patternSeed.azimuth+4*math.Pi, 2*math.Pi)
		k := int(math.Round(delta / (2 * math.Pi / float64(n))))
		if k == n {
			k = 0
		}
		if d := math.Abs(delta - 2*math.Pi*float64(k)/float64(n)); d > 1e-7 && math.Abs(d-2*math.Pi) > 1e-7 {
			t.Errorf("%s: a copy sits %.9f rad from the seed, which is not a multiple of 360/%d",
				g.label, delta, n)
			continue
		}
		if k < 0 || k >= n || seen[k] {
			t.Errorf("%s: two copies share the pattern position %d of %d", g.label, k, n)
			continue
		}
		seen[k] = true
	}
}

// ---------------------------------------------------------------------------
// The Combine-Join
// ---------------------------------------------------------------------------

// stepCombineJoin performs NO join, because the two operands are not in one frame.
// The proof builds the tooth on the back-cone section — the Tredgold mapping —
// while the gear body is written about the shaft axis, and the tooth's own seating
// on that body is derived nowhere here. Put in one document they do not meet, and
// neither sign of the rotation that relates the two planes seats them.
//
// The engine is not what blocks this join: given operands that do overlap it
// performs the union, returns one lump and publishes a volume bound of 8e-15 of the
// value. What is missing is the tooth's real back-cone placement, and deriving it is
// its own change.
//
// So the operands are laid apart and the join's TWO CONSEQUENCES are asserted from
// their own measured geometry: a join leaves one lump when the tooth's root is below
// the body's root cone — seated, not floating — and the joined body reaches further
// out than the frustum when the tooth's tip stands proud of it. THE COST IS THE
// STITCH: the proof cannot show the evaluator making one boundary out of two.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	gearSketch := decadtest.NewSketch(t)
	gearProfile := closedLoop(t, gearSketch, f.hexagon(g))
	gearBody, err := doc.Revolve(gearSketch, gearProfile, axisU, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s: gear body: %v", g.label, err)
	}

	w := sketch.NewWorld()
	outline := f.toothOutline(g, 6)
	s0, p0 := f.toothSection(t, w, g, outline, 0, apexShrink)
	s1, p1 := f.toothSection(t, w, g, outline, 1, 1)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: tooth: %v", g.label, err)
	}
	return []*decad.Body{gearBody, layApart(t, tooth, vec(0, 0, 4*f.apexDed))}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	gearBody := bodies[0]

	// The gear body's root cone, read off its own published face.
	hex := f.hexagon(g)
	rootHalf := math.NaN()
	for _, c := range coneFaces(gearBody) {
		area, err := c.Face.Area()
		if err != nil {
			t.Fatalf("%s: cone face area: %v", g.label, err)
		}
		if rel(area.Value.Base(), coneFrustumArea(hex[3], hex[4])) < 1e-6 {
			rootHalf = c.HalfAngle
		}
	}
	if math.IsNaN(rootHalf) {
		t.Fatalf("%s: the gear body publishes no root cone face", g.label)
	}
	rootSlope := math.Tan(rootHalf)

	// ⚠️ Read the root arc's OUTERMOST point, not the tooth's centreline. The
	// centreline sits inside both root corners, so a reading taken there passes a
	// tooth whose corners float outside the cone, which is exactly the defect the root
	// sink exists to remove.
	outline := f.toothOutline(g, 6)
	rootR := g.virtualPitchRadius - dedendumFactor*f.module - f.rootSink
	tipR := g.virtualPitchRadius + addendumFactor*f.module
	rootOuter, tipOuter := 0.0, 0.0
	for _, q := range outline {
		polar := math.Hypot(q.X, q.Y)
		w := f.place(g, q, 1)
		slope := math.Hypot(w.Y, w.Z) / w.X
		if math.Abs(polar-rootR) < 1e-9*rootR && slope > rootOuter {
			rootOuter = slope
		}
		if math.Abs(polar-tipR) < 1e-9*tipR && slope > tipOuter {
			tipOuter = slope
		}
	}
	if rootOuter == 0 || tipOuter == 0 {
		t.Fatalf("%s: the tooth outline carries no root or tip sample", g.label)
	}

	// Both readings are taken at the toe, the middle and the heel of the band the join
	// would cover. The tooth is a cone from the Apex and so is the gear body's root
	// cone, so the three stations differ in magnitude and agree in verdict — that is a
	// property of the figure, and it is recorded here rather than hidden by taking one
	// reading.
	toeS := f.stationOf(g, f.pts[toeCorner(g)])
	heelS := f.stationOf(g, f.pts[heelCorner(g)])
	for _, name := range []struct {
		where string
		s     float64
	}{{"toe", toeS}, {"middle", (toeS + heelS) / 2}, {"heel", heelS}} {
		body := name.s * rootSlope
		root := name.s * rootOuter
		tip := name.s * tipOuter
		if root > body {
			t.Errorf("%s: at the %s the root arc's outermost point stands %.6f mm from the axis "+
				"against the body's root cone at %.6f — the tooth floats and the join would leave "+
				"two lumps", g.label, name.where, root, body)
		}
		if tip <= body {
			t.Errorf("%s: at the %s the tooth's tip reaches %.6f mm against the body's %.6f — the "+
				"join would reach no further out than the frustum", g.label, name.where, tip, body)
		}
	}
	// The sink is what puts the root arc inside the root cone across its whole width
	// rather than along the centreline alone. It is one figure, not a proof-only
	// offset: the generated module draws the same circle.
	if f.rootSink <= 0 {
		t.Errorf("%s: the root sink is %.6f; the root arc then touches the root cone only where "+
			"it crosses the tooth's own centreline", g.label, f.rootSink)
	}
}

func toeCorner(g member) string {
	if g.label == "Pinion" {
		return "M"
	}
	return "O"
}

func heelCorner(g member) string {
	if g.label == "Pinion" {
		return "C"
	}
	return "D"
}

// ---------------------------------------------------------------------------
// The bore
// ---------------------------------------------------------------------------

// boreSides is the bore tool's polygon count. The tool is a real extrude, which a
// symmetric extent produces as a PRISM, so its cross-section is a polygon and the
// material it removes over a height is that polygon's own area times the height.
const boreSides = 64

// stepBoreCut builds the bore tool as a real extrude and performs the cut.
//
// The target is the HEEL CONE BAND, lofted for this step: it is the section of the
// gear body the bore passes through, and it is a Loft, which is the form a boolean
// operand takes here. The revolved gear body cannot be the target, because every
// boolean this gear's proof ran with a Revolve operand verified Suspect and the gate
// admits neither Suspect nor the refusal. WHAT THIS DOES NOT REACH IS THE REST OF
// THE BODY: the bore is pierced through the band that stands for the heel section,
// not through the whole frustum.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	if !f.boreEnabled {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so no bore is cut on either gear")
		return nil
	}
	w := sketch.NewWorld()
	hex := f.hexagon(g)
	heel, corner := hex[2], hex[3] // H (resp. J) and C (resp. D)

	band := ngonBand(t, doc, w, heel, corner, boreSides)
	tool := boreTool(t, doc, w, f, g)
	pierced, err := decad.Cut(band, tool)
	if err != nil {
		t.Fatalf("%s: bore cut: %v", g.label, err)
	}
	return []*decad.Body{pierced}
}

// ngonBand lofts the conical band between two stations as an n-gon frustum.
func ngonBand(t *testing.T, doc *decad.Document, w *sketch.World, a, b pt, sides int) *decad.Body {
	disc := func(q pt) (*sketch.Sketch, *sketch.Profile) {
		s := planeSketch(t, w, vec(q.X, 0, 0), vec(0, 1, 0), vec(0, 0, 1))
		return s, closedLoop(t, s, ngon(q.Y, sides))
	}
	s0, p0 := disc(a)
	s1, p1 := disc(b)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("heel cone band: %v", err)
	}
	return body
}

func ngon(radius float64, sides int) []pt {
	out := make([]pt, 0, sides)
	for i := range sides {
		a := 2 * math.Pi * float64(i) / float64(sides)
		out = append(out, pt{radius * math.Cos(a), radius * math.Sin(a)})
	}
	return out
}

// boreTool extrudes the bore polygon symmetrically about the plane normal to the
// shaft at its start, 2 * Cone Distance per side — generously past any face width,
// which is what makes it a THROUGH cut.
func boreTool(t *testing.T, doc *decad.Document, w *sketch.World, f figure, g member) *decad.Body {
	start := f.stationOf(g, f.pts[shaftEdgeStart(g)])
	s := planeSketch(t, w, vec(start, 0, 0), vec(0, 1, 0), vec(0, 0, 1))
	profile := closedLoop(t, s, ngon(g.boreDiameter/2, boreSides))
	body, err := doc.Extrude(s, profile, decad.Symmetric{D: mm(2 * f.coneDistance)})
	if err != nil {
		t.Fatalf("%s: bore tool: %v", g.label, err)
	}
	return body
}

func shaftEdgeStart(g member) string {
	if g.label == "Pinion" {
		return "A'"
	}
	return "B'"
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	hex := f.hexagon(g)
	heel, corner := hex[2], hex[3]

	// The tool first, from its own closed form: the two ends sit exactly
	// 2 * Cone Distance either side of the shaft edge's start, and both clear the
	// frustum, which is what makes the cut a through cut.
	start := f.stationOf(g, f.pts[shaftEdgeStart(g)])
	lo, hi := start-2*f.coneDistance, start+2*f.coneDistance
	if lo >= math.Min(heel.X, corner.X) || hi <= math.Max(heel.X, corner.X) {
		t.Errorf("%s: the bore tool spans stations [%.4f, %.4f] and the band spans [%.4f, %.4f]; "+
			"the tool must clear both ends", g.label, lo, hi, heel.X, corner.X)
	}

	// The pierced body: the band's own n-gon closed form less the prism the bore
	// removes over that height.
	area := polygonArea(ngon(1, boreSides))
	a1 := area * heel.Y * heel.Y
	a2 := area * corner.Y * corner.Y
	h := math.Abs(corner.X - heel.X)
	band := h * (a1 + a2 + math.Sqrt(a1*a2)) / 3
	boreArea := area * sq(g.boreDiameter/2)
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: pierced volume: %v", g.label, err)
	}
	decadtest.Measures(t, g.label+" bored heel band volume",
		vol, mm3(band-boreArea*h), relSlack(1e-9))

	// The bore never reaches either end face: a bore past the heel term takes the
	// whole flat back face, and past the toe term the whole toe dish.
	if g.boreDiameter > g.maxBore+1e-12 {
		t.Errorf("%s: the bore diameter reaching this step is not bounded (%.6f > %.6f)",
			g.label, g.boreDiameter, g.maxBore)
	}
}

// ---------------------------------------------------------------------------
// The meshing rotation
// ---------------------------------------------------------------------------

// stepMeshingRotation rotates the DRIVING body by half a tooth pitch about its own
// shaft axis, so a driving valley sits where the pinion tooth crosses the axial
// plane. Both gears are patterned from a starting tooth in that plane, so without
// the offset a driving tooth and a pinion tooth would both sit at the crossing.
//
// The pinion's extra phase is _PINION_MESH_PHASE_TEETH tooth-fractions, 0 by
// default, and a ZERO ANGLE IS A NO-OP RATHER THAN A MOVE: Fusion refuses to move a
// body by the identity with "invalid transform", so the framework helper returns
// early instead of making each call site guard it. The proof applies no transform on
// the pinion side for the same reason.
func stepMeshingRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newFigure(p)
	g := caseMember(f, p)
	w := sketch.NewWorld()
	outline := f.toothOutline(g, 6)
	s0, p0 := f.toothSection(t, w, g, outline, 0, apexShrink)
	s1, p1 := f.toothSection(t, w, g, outline, 1, 1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: tooth: %v", g.label, err)
	}
	angle := meshPhase(g)
	if angle == 0 {
		return []*decad.Body{body}
	}
	tr, err := r3.RotationAround(vec(0, 0, 0), vec(1, 0, 0), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s: mesh rotation: %v", g.label, err)
	}
	moved, err := body.Placed(tr)
	if err != nil {
		t.Fatalf("%s: mesh rotation: %v", g.label, err)
	}
	return []*decad.Body{moved}
}

// meshPhase is this gear's mesh rotation in radians: half a tooth pitch on the
// driving gear, and _PINION_MESH_PHASE_TEETH tooth-fractions on the pinion, which is
// 0 by default because the spiral build leaves the mid-face section unrotated and
// already meshing.
func meshPhase(g member) float64 {
	if g.label == "Driving" {
		return math.Pi / g.teeth
	}
	return 0
}

func assertMeshingRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newFigure(p)
	g := caseMember(f, p)
	outline := f.toothOutline(g, 6)
	seat := f.place(g, pt{polygonCentroidX(outline), polygonCentroidY(outline)}, 1)
	want := math.Atan2(seat.Z, seat.Y) + meshPhase(g)

	centre, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("%s: centroid: %v", g.label, err)
	}
	got := math.Atan2(centre.Value.Z, centre.Value.Y)
	if d := math.Abs(math.Mod(got-want+3*math.Pi, 2*math.Pi) - math.Pi); d > 1e-6 {
		t.Errorf("%s: the body sits at azimuth %.9f rad, want %.9f — half a tooth pitch is "+
			"180°/%0.f", g.label, got, want, g.teeth)
	}
	if g.label == "Pinion" && meshPhase(g) != 0 {
		t.Errorf("Pinion: the default mesh phase is 0 tooth-fractions, got %.9f", meshPhase(g))
	}
}
