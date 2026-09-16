package bevelgear_test

// The solid steps.
//
// ---------------------------------------------------------------------------
// NO BOOLEAN IS PERFORMED ANYWHERE IN THIS GEAR'S PROOF, and that is a limit of
// the pinned evaluator rather than a choice.
//
// At this decad revision a boolean accepts only prism, cup and faceted payloads
// and refuses an operand built by Loft. Every solid in this gear is conical, and
// a cone is a Loft here because Extrude refuses a nonzero taper. So the union
// that joins the frustum, the intersection and cut that trim the tooth, the
// bore's through-cut and the Combine-Join are all unavailable.
//
// The substitution is the same at every site: BUILD THE OPERANDS, LAY THEM
// APART ALONG THE SHAFT AXIS, AND ASSERT FROM THEIR OWN MEASURED GEOMETRY WHAT
// THE OPERATION WOULD HAVE PRODUCED. Laying them apart leaves every volume,
// radius and cone angle unchanged, which is what makes the readings still mean
// something. Each step says below what its own substitution costs.
//
// Every step here is [GO]. None of them is [PROSE]: a boundary the harness
// refuses is not permission to drop the step.
// ---------------------------------------------------------------------------
//
// The tables run at Module 4 to 8 and never at Module 1. decad's mesh bound has
// an absolute floor, so a figure small enough brings every measurement inside it
// and the gate reports Suspect on geometry that is in fact correct. Module is a
// pure scale on this figure, so a case at Module 4 through 8 proves the same
// shape and clears the floor.

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// THE REVOLVE IS SUBSTITUTED BY A POLYGONAL SWEEP, and the polygon is not a
// convenience: a CIRCULAR loft pair is chorded by the evaluator, and the
// proven bound it publishes on the resulting volume comes out past decad's own
// relative tolerance -- measured at 124.4 mm^3 against a required 104.0 mm^3 on
// the shipped pair at Module 4 -- so every such body is reported Suspect and
// the harness gate refuses it. A loft between two regular polygons pairs
// LineSegs with LineSegs, is ruled exactly, and reads back EXACT.
//
// sweepSides is how many sides that polygon has. It is a stand-in for the
// circle, so the readings below are compared against the polygon's own closed
// form rather than the circle's, and the one place the difference matters --
// the frustum volume against Pappus -- carries the conversion explicitly.
// It is a multiple of four so the polygon carries a vertex on each axis, which
// is what lets a bounding-box reading name the ring radius outright.
const sweepSides = 48

// polygonFactor is the polygon sweep's share of the true solid of revolution:
// a regular n-gon of circumradius r has area (n/2) r^2 sin(2 pi / n) where the
// circle has pi r^2, and both scale the same way along the axis.
var polygonFactor = float64(sweepSides) / 2 * math.Sin(2*math.Pi/sweepSides) / math.Pi

// chordSlack is what a CIRCULAR wall needs, and the bore tool is the one body
// here that has one: decad chords a circular extrude's wall, so its volume
// comes out a little under pi r^2 h. The helpers add decad's own proven bound
// on top of whatever is stated here, which is the whole reason a comparison
// goes through them rather than through a subtraction.
var chordSlack = decadtest.WithinRel(units.Scalar(4e-3))

// oracleSlack is the rounding of a closed-form float64 oracle across a handful
// of operations. A ruled body between two polygons reads back with a bound
// around 1e-12 of its own volume, so nothing here needs the chord allowance a
// circular pair would.
var oracleSlack = decadtest.WithinRel(units.Scalar(1e-9))

// ---------------------------------------------------------------------------
// Bench helpers: the conical band, which is what every body here is made of.
// ---------------------------------------------------------------------------

// bandRing is one end of a conical band: a station along the shaft axis and the
// radius the profile edge reaches there.
type bandRing struct{ Station, Radius float64 }

// band is one profile edge's swept cone, already laid apart from its
// neighbours. Lift is how far along the axis it was moved to get clear of them,
// which is the ONLY reading the laying-apart changes.
type band struct {
	Name     string
	Near     bandRing // the end nearer the apex
	Far      bandRing // the end further from the apex
	Lift     float64
	Reversed bool // true when the profile edge runs back toward the heel
	Body     *decad.Body
}

// Height is the band's axial length, unchanged by the lift.
func (b band) Height() float64 { return b.Far.Station - b.Near.Station }

// SignedVolume is this edge's contribution to the solid of revolution. A closed
// profile revolved about the axis has volume pi * integral(y^2 dx), and one
// straight edge contributes pi * (x2-x1) * (y1^2 + y1 y2 + y2^2) / 3 -- exactly
// the truncated cone between its two stations, signed by which way the edge
// runs. The toe dish runs back toward the heel, so it enters with the opposite
// sign and HOLLOWS the front face.
func (b band) SignedVolume() float64 {
	v := polygonFactor * math.Pi * b.Height() *
		(b.Near.Radius*b.Near.Radius + b.Near.Radius*b.Far.Radius + b.Far.Radius*b.Far.Radius) / 3
	if b.Reversed {
		return -v
	}
	return v
}

// Slope is the band's cone slope, dr/dx. Two bands on the same cone family read
// the same slope.
func (b band) Slope() float64 {
	return (b.Far.Radius - b.Near.Radius) / b.Height()
}

// liftedSketch puts a sketch on a plane offset from the world XY by z.
func liftedSketch(t *testing.T, w *sketch.World, z float64) *sketch.Sketch {
	t.Helper()
	plane := w.XY()
	if z != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			t.Fatalf("offset plane at %g: %v", z, err)
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch on the plane at %g: %v", z, err)
	}
	return s
}

// axisRing draws one ring of the polygonal sweep: a regular polygon centred on
// the shaft axis at the station z, with the ring radius as its circumradius.
func axisRing(t *testing.T, w *sketch.World, z, radius float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := liftedSketch(t, w, z)
	pts := make([]*sketch.Point, 0, sweepSides)
	for i := range sweepSides {
		th := 2 * math.Pi * float64(i) / sweepSides
		pts = append(pts, s.CreatePoint(radius*math.Cos(th), radius*math.Sin(th)))
	}
	lines := make([]*sketch.Line, 0, sweepSides)
	for i := range pts {
		lines = append(lines, s.CreateLine(pts[i], pts[(i+1)%len(pts)]))
	}
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}
	return s, decadtest.SolveRegion(t, s)
}

// ringArea is the polygon sweep's own section area at a ring radius.
func ringArea(radius float64) float64 {
	return float64(sweepSides) / 2 * radius * radius * math.Sin(2*math.Pi/sweepSides)
}

// buildBand lofts one profile edge's cone between its two rings. The loft is
// the only way to build a cone at this decad revision, because Extrude refuses
// a nonzero taper.
func buildBand(t *testing.T, doc *decad.Document, w *sketch.World, b band) band {
	t.Helper()
	if b.Height() <= 0 {
		t.Fatalf("%s: the band has no axial length (%g)", b.Name, b.Height())
	}
	s0, p0 := axisRing(t, w, b.Lift, b.Near.Radius)
	s1, p1 := axisRing(t, w, b.Lift+b.Height(), b.Far.Radius)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: lofting the band between r=%g and r=%g over %g failed: %v",
			b.Name, b.Near.Radius, b.Far.Radius, b.Height(), err)
	}
	b.Body = body
	return b
}

// measuresBandVolume compares one band's reading against the truncated cone its
// two rings name, under the band's own label rather than by index. decadtest
// otherwise names a body by index and recipe step, which does not say which
// feature is wrong.
func measuresBandVolume(t *testing.T, b band) {
	t.Helper()
	v, err := b.Body.Volume()
	if err != nil {
		t.Fatalf("%s: volume: %v", b.Name, err)
	}
	want := math.Abs(b.SignedVolume())
	decadtest.Measures(t, b.Name+" band volume", v,
		units.CubicMillimeters(want), oracleSlack)
}

// bandsOf is the three profile edges that sweep anything: the heel band, the
// root band and the toe dish. The other three hexagon edges sweep nothing --
// two lie at a constant station and one lies ON the axis -- so the frustum is
// these three and only these three.
func bandsOf(c bevelCase, m sideMember) []band {
	f := latticeOf(c, m)
	edge := func(name string, a, b planeVec, lift float64) band {
		near, far := bandRing{a.X, a.Y}, bandRing{b.X, b.Y}
		reversed := false
		if b.X < a.X {
			near, far = far, near
		} else {
			// The edge runs outward from the apex, which for the toe dish is
			// back toward the heel: it subtracts.
			reversed = true
		}
		return band{Name: name, Near: near, Far: far, Lift: lift, Reversed: reversed}
	}
	// The hexagon is walked A' -> G -> H -> C -> M -> N, so the heel band runs
	// H -> C, the root band C -> M and the toe dish M -> N.
	return []band{
		edge("heel", f.Heel, f.Ded, 0),
		edge("root", f.Ded, f.Toe, 1000),
		edge("toe dish", f.Toe, f.ToeIn, 2000),
	}
}

// ---------------------------------------------------------------------------
// S16 -- the gear body revolve.
// ---------------------------------------------------------------------------

// stepGearBody substitutes a polygonal sweep for the revolve.
//
// decad publishes a revolved body's volume with a proven bound equal to the
// volume itself, so a revolved body is Suspect at any tolerance and cannot pass
// the harness gate at all. What is built instead is the three bands the
// frustum's own profile edges sweep -- the heel cone out to the heel end, the
// root cone out to the dedendum corner, and the toe dish that hollows the front
// face -- laid apart along the shaft axis and never joined.
//
// THE COST IS THE UNION: this does not show the three bands closing into one
// watertight solid, only that each is separately watertight and that together
// they have the right volume, stations and angles.
func stepGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	w := sketch.NewWorld()

	bodies := make([]*decad.Body, 0, 3)
	for _, b := range bandsOf(c, m) {
		built := buildBand(t, doc, w, b)
		bodies = append(bodies, built.Body)
	}
	return bodies
}

// assertGearBody reads the frustum out of the three bands.
func assertGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	bands := bandsOf(c, m)
	if len(bodies) != 3 {
		t.Fatalf("%s: the revolve substitution built %d band(s), want 3", m.Name, len(bodies))
		return
	}
	for i := range bands {
		bands[i].Body = bodies[i]
	}

	// Band by band, against its own stations and ring radii.
	var signed float64
	for _, b := range bands {
		measuresBandVolume(t, b)
		signed += b.SignedVolume()

		box, err := b.Body.Bounds()
		if err != nil {
			t.Fatalf("%s: bounds: %v", b.Name, err)
		}
		// sweepSides is a multiple of four, so the polygon carries a vertex on
		// each axis and its box reaches the ring radius both ways. The station
		// range is the band's own, offset by the lift and by nothing else,
		// which is what laying the bands apart is allowed to change.
		wide := math.Max(b.Near.Radius, b.Far.Radius)
		decadtest.MeasuresBox(t, m.Name+" "+b.Name+" band extent", box,
			r3.NewVec(-wide, -wide, b.Lift),
			r3.NewVec(wide, wide, b.Lift+b.Height()), oracleSlack)
	}

	// The frustum as the SIGNED SUM, against Pappus on the section 2 hexagon:
	// a solid of revolution has volume 2 pi * (centroid distance from the axis)
	// * (profile area). The toe dish enters negative, which is what hollowing
	// the front face means.
	loop := f.profileLoop()
	pappus := polygonFactor * 2 * math.Pi * polygonCentroidY(loop) * polygonArea(loop)
	near(t, m.Name+" frustum volume as the bands' signed sum",
		math.Abs(signed), math.Abs(pappus), math.Abs(pappus)*1e-9)
	if bands[2].SignedVolume() >= 0 {
		t.Errorf("%s: the toe dish did not enter the sum negatively, so it is not "+
			"hollowing the front face", m.Name)
	}

	// Cone half-angle by cone half-angle. The heel band and the toe dish come
	// out PARALLEL, on the back-cone family, because the toe line M->N is C->H
	// offset toward the apex; the root band sits at the dedendum angle to them.
	near(t, m.Name+" heel band and toe dish are parallel",
		bands[0].Slope(), bands[2].Slope(), 1e-9)
	near(t, m.Name+" back-cone slope", bands[0].Slope(),
		-math.Cos(m.Gamma)/math.Sin(m.Gamma), 1e-9)
	gammaRoot := m.Gamma - math.Atan2(1.25*c.Module, c.R)
	near(t, m.Name+" root band sits at the root cone angle",
		math.Atan(bands[1].Slope()), gammaRoot, 1e-9)
	near(t, m.Name+" dedendum angle between the two families",
		math.Atan(bands[0].Slope())-math.Atan(bands[1].Slope()),
		-math.Pi/2+math.Atan2(1.25*c.Module, c.R), 1e-9)
}

// ---------------------------------------------------------------------------
// S17 -- the apex tooth loft.
// ---------------------------------------------------------------------------

// toothSections builds the two loft sections: a shrunken copy near the apex in
// place of the degenerate apex POINT, and the full section at the heel. Both
// are axis-perpendicular, which is the second substitution -- the real loft's
// far section lies on the back-cone tooth plane.
//
// The tooth outline is chorded into line segments. decad's loft pairs LineSegs
// with LineSegs and refuses a free-form pair outright, so an involute flank
// cannot be lofted as a spline here; chording it keeps the section's area and
// its reach, which is what the assertions read.
func toothSections(t *testing.T, w *sketch.World, c bevelCase, m sideMember,
	nearScale, heelScale, nearZ, heelZ float64) (*sketch.Sketch, *sketch.Profile,
	*sketch.Sketch, *sketch.Profile, float64, float64) {
	t.Helper()
	section := toothSectionOf(c, m)
	loop := toothChords(section)

	draw := func(z, scale float64) (*sketch.Sketch, *sketch.Profile, float64) {
		s := liftedSketch(t, w, z)
		pts := make([]*sketch.Point, 0, len(loop))
		scaled := make([]planeVec, 0, len(loop))
		for _, v := range loop {
			q := v.times(scale)
			scaled = append(scaled, q)
			pts = append(pts, s.CreatePoint(q.X, q.Y))
		}
		lines := make([]*sketch.Line, 0, len(pts))
		for i := range pts {
			lines = append(lines, s.CreateLine(pts[i], pts[(i+1)%len(pts)]))
		}
		for _, l := range lines {
			s.Fix(l.Start)
			s.Fix(l.End)
		}
		return s, decadtest.SolveRegion(t, s), polygonArea(scaled)
	}

	s0, p0, a0 := draw(nearZ, nearScale)
	s1, p1, a1 := draw(heelZ, heelScale)
	return s0, p0, s1, p1, a0, a1
}

// toothChords is the tooth cross-section as a closed polygon: both flanks
// sampled, the tip arc and the root arc chorded between them.
func toothChords(s toothPolar) []planeVec {
	const arcChords = 8
	loop := make([]planeVec, 0, len(s.Right)+len(s.Left)+2*arcChords)
	at := func(p polarPt) planeVec {
		return pv(p.Radius*math.Cos(p.Theta), p.Radius*math.Sin(p.Theta))
	}
	for _, p := range s.Right {
		loop = append(loop, at(p))
	}
	loop = append(loop, arcChordsBetween(s.TipRadius,
		s.Right[len(s.Right)-1].Theta, s.Left[len(s.Left)-1].Theta, arcChords)...)
	for i := len(s.Left) - 1; i >= 0; i-- {
		loop = append(loop, at(s.Left[i]))
	}
	loop = append(loop, arcChordsBetween(s.RootRadius,
		s.Left[0].Theta, s.Right[0].Theta, arcChords)...)
	return loop
}

// arcChordsBetween walks the interior of an arc, endpoints excluded, so the
// chorded loop has no repeated vertex.
func arcChordsBetween(radius, from, to float64, n int) []planeVec {
	out := make([]planeVec, 0, n)
	sweep := foldAngle(to - from)
	for i := 1; i < n; i++ {
		th := from + sweep*float64(i)/float64(n)
		out = append(out, pv(radius*math.Cos(th), radius*math.Sin(th)))
	}
	return out
}

// apexShrink is the scale of the section that stands in for the degenerate
// apex point. It cannot be zero -- a loft to a point is not a section pair the
// evaluator takes -- so it is small enough that what it leaves out is under a
// thousandth of the tooth and large enough to stay clear of the mesh floor.
const apexShrink = 0.02

// stepToothLoft builds the uncut apex-to-heel tooth.
//
// TWO SUBSTITUTIONS, and the step says what each costs. The degenerate apex
// POINT becomes a shrunken section, so the proof does not show a loft closing
// on a point; and the back-cone tooth plane becomes an axis-perpendicular one,
// so it does not show the section standing at the back-cone angle. What both
// keep is the thing the later steps read: the tooth's reach, its section area
// at each end, and the volume the two of them imply.
func stepToothLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	// The section scales linearly with the station, because the real loft runs
	// from the apex. The heel section sits at the dedendum corner's own AXIAL
	// station -- the substitution stands it square to the shaft -- and the
	// shrunken one at apexShrink of it.
	heelStation := f.Ded.X
	s0, p0, s1, p1, _, _ := toothSections(t, w, c, m,
		apexShrink, 1, apexShrink*heelStation, heelStation)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the apex tooth loft failed: %v", m.Name, err)
	}
	return []*decad.Body{body}
}

func assertToothLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	section := toothSectionOf(c, m)
	heelStation := f.Ded.X
	height := heelStation * (1 - apexShrink)

	// A ruled body between two SIMILAR sections is exactly the prismatoid its
	// two areas name, because a linear scale makes the area quadratic in the
	// station: V = h (A0 + A1 + sqrt(A0 A1)) / 3.
	full := polygonArea(toothChords(section))
	a0 := full * apexShrink * apexShrink
	want := height * (a0 + full + math.Sqrt(a0*full)) / 3
	v, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: tooth volume: %v", m.Name, err)
	}
	decadtest.Measures(t, m.Name+" tooth body volume", v,
		units.CubicMillimeters(want), oracleSlack)

	// THE TIP READING. The tooth reaches out to the virtual tip radius laid on
	// the back cone, and this is the one place the proof reads a tip radius off
	// a built body. It runs at Module 4 through 8 only, because no solid case
	// runs at Module 1, and no case reads a tip radius off a JOINED body
	// because no case joins.
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s: tooth bounds: %v", m.Name, err)
	}
	reach := math.Max(math.Max(math.Abs(box.Min.X), box.Max.X),
		math.Max(math.Abs(box.Min.Y), box.Max.Y))
	if reach > section.TipRadius+1e-6 {
		t.Errorf("%s: the tooth body reaches %.6f mm, past the virtual tip radius "+
			"%.6f mm it is drawn to", m.Name, reach, section.TipRadius)
	}
	if reach < 0.98*section.TipRadius {
		t.Errorf("%s: the tooth body reaches only %.6f mm of the virtual tip radius "+
			"%.6f mm", m.Name, reach, section.TipRadius)
	}
	near(t, m.Name+" tip radius is the sunk root plus the whole tooth height",
		section.TipRadius, rootAtHeel(c, m)+2.25*c.Module+c.RootSink, 1e-9)
}

// ---------------------------------------------------------------------------
// S18 -- the two conical end cuts.
// ---------------------------------------------------------------------------

// stepConicalTrims builds the tooth and the two cutting cones and lays them
// apart. NEITHER CUT IS PERFORMED: both operands are Lofts -- the tooth and
// each cone alike -- and no boolean takes a Loft here.
//
// THE COST IS THE SPLIT: this does not show the evaluator dividing the tooth,
// selecting the keeper, or leaving a watertight body. What it does show is that
// each cut lands where the flush band requires, and that the two ends land on
// DIFFERENT surfaces of the tooth -- the toe on its tip and the heel on its
// root -- which is the observable signature of a conical cut face rather than
// a planar one.
func stepConicalTrims(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	heelStation := f.Ded.X
	s0, p0, s1, p1, _, _ := toothSections(t, w, c, m,
		apexShrink, 1, apexShrink*heelStation, heelStation)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the tooth loft failed: %v", m.Name, err)
	}

	// The cutting TOOLS are cone faces of the revolved frustum, never of the
	// lofted tooth: the tooth has no cone face, so searching it finds none. The
	// toe cone is the one the toe edge swept and the heel cone the one the heel
	// edge swept, and they are the same two bands S16 built.
	bands := bandsOf(c, m)
	toe := buildBand(t, doc, w, band{Name: "toe cone",
		Near: bands[2].Near, Far: bands[2].Far, Lift: 3000})
	heel := buildBand(t, doc, w, band{Name: "heel cone",
		Near: bands[0].Near, Far: bands[0].Far, Lift: 4000})

	return []*decad.Body{tooth, toe.Body, heel.Body}
}

func assertConicalTrims(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	section := toothSectionOf(c, m)
	bands := bandsOf(c, m)
	if len(bodies) != 3 {
		t.Fatalf("%s: the trim substitution built %d body(ies), want the tooth and "+
			"two cones", m.Name, len(bodies))
		return
	}

	// The substitute tooth stands its heel section square to the shaft at the
	// dedendum corner's station, so both of its surfaces are cones through the
	// apex and a radius is a slope.
	heelStation := f.Ded.X
	tipSlope := section.TipRadius / heelStation
	rootSlope := section.RootRadius / heelStation

	// Each cutting cone's apex and slope, read off that band's own two rings
	// rather than restated: a cone through (x0, r0) and (x1, r1) has slope
	// dr/dx, and its apex is the station where the radius reaches zero.
	apexOf := func(b band) (apex, slope float64) {
		slope = b.Slope()
		return b.Near.Station - b.Near.Radius/slope, slope
	}
	toeApex, toeSlope := apexOf(bands[2])
	heelApex, heelSlope := apexOf(bands[0])

	// Solve the stations where a cut cone crosses a tooth surface. A cone
	// r = s (x - a) meets a surface r = t x at x = -s a / (t - s).
	crossing := func(coneSlope, coneApex, surfaceSlope float64) float64 {
		return -coneSlope * coneApex / (surfaceSlope - coneSlope)
	}
	toeOnTip := crossing(toeSlope, toeApex, tipSlope)
	toeOnRoot := crossing(toeSlope, toeApex, rootSlope)
	heelOnTip := crossing(heelSlope, heelApex, tipSlope)
	heelOnRoot := crossing(heelSlope, heelApex, rootSlope)

	// EACH CUT LANDS WHERE THE FLUSH BAND REQUIRES. The band's two ends on the
	// root element are the toe corner M and the dedendum corner C, and each cut
	// meets the tooth's root within one ROOT SINK of its own corner. The sink is
	// what the difference is: the tooth's root circle is drawn that far inside
	// the corner, and the axis-perpendicular section this substitution stands
	// the tooth on moves it no further than that again.
	near(t, m.Name+" toe cut lands at the flush band's toe end",
		toeOnRoot, f.Toe.X, c.RootSink)
	near(t, m.Name+" heel cut lands at the flush band's heel end",
		heelOnRoot, f.Ded.X, c.RootSink)
	near(t, m.Name+" the trimmed band is the flush band",
		heelOnRoot-toeOnRoot, f.Ded.X-f.Toe.X, c.RootSink)

	// THE TWO ENDS LAND ON DIFFERENT SURFACES, the toe on the tooth's tip and
	// the heel on its root, and that is the observable signature of a CONICAL
	// cut face: a planar cut would meet tip and root at the same station.
	if tipSlope <= rootSlope {
		t.Errorf("%s: the tooth's tip surface (%.6f) does not stand outside its root "+
			"surface (%.6f), so the two cuts cannot land on different ones",
			m.Name, tipSlope, rootSlope)
	}
	for _, cut := range []struct {
		name          string
		onTip, onRoot float64
	}{
		{"toe", toeOnTip, toeOnRoot},
		{"heel", heelOnTip, heelOnRoot},
	} {
		spread := cut.onRoot - cut.onTip
		if spread <= 0 {
			t.Errorf("%s: the %s cut meets the tooth's tip at %.6f and its root at "+
				"%.6f, so the cut face is not leaning the way a cone does",
				m.Name, cut.name, cut.onTip, cut.onRoot)
		}
		// A planar cut would give a spread of zero. The conical one spreads by
		// the tooth's own height divided by how fast the two surfaces diverge
		// from the cone, which is never small next to the sink.
		if spread <= c.RootSink {
			t.Errorf("%s: the %s cut spreads only %.6f mm between tip and root, "+
				"which does not distinguish it from a planar cut", m.Name, cut.name, spread)
		}
	}

	// The cut cones are EXTENDED to reach the tooth, which is what Fusion's
	// isSplittingToolExtended asks for: the toe cone meets the tooth's tip at a
	// station its own band does not reach.
	if toeOnTip >= math.Min(bands[2].Near.Station, bands[2].Far.Station) {
		t.Logf("%s: the toe cone reaches the tooth's tip inside its own band", m.Name)
	}

	// Both cones still read as the bands they were built from, which is what
	// makes laying them apart harmless.
	toeVol, err := bodies[1].Volume()
	if err != nil {
		t.Fatalf("%s: toe cone volume: %v", m.Name, err)
	}
	decadtest.Measures(t, m.Name+" toe cone volume", toeVol,
		units.CubicMillimeters(math.Abs(bands[2].SignedVolume())), oracleSlack)
	heelVol, err := bodies[2].Volume()
	if err != nil {
		t.Fatalf("%s: heel cone volume: %v", m.Name, err)
	}
	decadtest.Measures(t, m.Name+" heel cone volume", heelVol,
		units.CubicMillimeters(math.Abs(bands[0].SignedVolume())), oracleSlack)
}

// ---------------------------------------------------------------------------
// S29 -- the Combine-Join.
// ---------------------------------------------------------------------------

// stepCombineJoin builds the two operands the join would take -- the trimmed
// tooth and the gear body's root band -- and lays them apart. NO JOIN IS
// PERFORMED.
//
// THE COST IS THE STITCH: this cannot show the evaluator making one boundary
// out of two. What it asserts instead is the join's two consequences, from the
// operands' own measured geometry: a join leaves ONE lump when the tooth's root
// is BELOW the body's root cone -- seated, not floating -- and the joined body
// reaches further out than the frustum when the tooth's tip stands proud of it.
// Both readings are taken at the toe, the middle and the heel of the band the
// join would cover.
//
// The generated module draws its root circle one ROOT SINK inside the dedendum
// corner, and the proof applies that same sink: it is one figure, not a
// proof-only offset. Without it the root arc touches the root cone along one
// line only, at the tooth's own centreline, and the corners stand outside.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	// The stations are AXIAL, as everywhere else here: the tooth stands its
	// sections square to the shaft axis, so the flush band's two ends are the
	// toe corner's and the dedendum corner's own stations.
	heelStation := f.Ded.X
	toeStation := f.Toe.X
	s0, p0, s1, p1, _, _ := toothSections(t, w, c, m,
		toeStation/heelStation, 1, toeStation, heelStation)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the trimmed tooth loft failed: %v", m.Name, err)
	}

	bands := bandsOf(c, m)
	root := buildBand(t, doc, w, band{Name: "root band",
		Near: bands[1].Near, Far: bands[1].Far, Lift: 5000})
	return []*decad.Body{tooth, root.Body}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	section := toothSectionOf(c, m)
	bands := bandsOf(c, m)

	heelStation := f.Ded.X
	toeStation := f.Toe.X

	// The gear body's root cone, as a radius per station, read off the root
	// band's own two rings.
	rootSlope := bands[1].Slope()
	rootAt := func(x float64) float64 {
		return bands[1].Near.Radius + rootSlope*(x-bands[1].Near.Station)
	}

	// The tooth, as a radius per station: it was lofted from the apex, so every
	// radius scales with the station.
	//
	// THE OUTERMOST POINT OF THE ROOT ARC, not the tooth's centreline. The
	// centreline sits inside both root corners, so a reading taken there passes
	// a tooth whose corners float outside the cone -- which is the defect the
	// sink exists to remove.
	outerRoot := section.outerRootRadius()
	toothRootAt := func(x float64) float64 { return outerRoot * x / heelStation }
	toothTipAt := func(x float64) float64 { return section.TipRadius * x / heelStation }

	for _, where := range []struct {
		name    string
		station float64
	}{
		{"toe", toeStation},
		{"middle", (toeStation + heelStation) / 2},
		{"heel", heelStation},
	} {
		body := rootAt(where.station)
		seat := toothRootAt(where.station)
		proud := toothTipAt(where.station)

		// Seated, not floating: ONE lump.
		if seat > body {
			t.Errorf("%s: at the %s the tooth's outermost root point sits %.6f mm "+
				"OUTSIDE the gear body's root cone (%.6f against %.6f), so the join "+
				"would leave the tooth floating rather than one lump",
				m.Name, where.name, seat-body, seat, body)
		}
		// The joined body reaches further out than the frustum.
		if proud <= body {
			t.Errorf("%s: at the %s the tooth's tip does not stand proud of the gear "+
				"body (%.6f against %.6f), so the join would add nothing",
				m.Name, where.name, proud, body)
		}
	}

	// The sink is what buys the seating. Without it the root arc's corners
	// stand outside the root cone; the largest float of any pair the spec
	// admits is 0.027 module, on a 4/4 pair.
	unsunk := outerRoot + c.RootSink
	if unsunk <= rootAt(heelStation) {
		t.Logf("%s: this pair's root arc would have seated without the sink "+
			"(%.6f against %.6f at the heel)", m.Name, unsunk, rootAt(heelStation))
	}

	// Both operands are still separate bodies, which is the cost stated above.
	if len(bodies) != 2 {
		t.Fatalf("%s: the join substitution built %d operand(s), want 2", m.Name, len(bodies))
	}
}

// ---------------------------------------------------------------------------
// S28 -- the circular pattern.
// ---------------------------------------------------------------------------

// THIS STEP IS SERIAL, and the readings just below are why.
//
// The pattern increment RETIRES the seed tooth: decad's Placed consumes the
// body it moves, exactly as Fusion's pattern leaves the seed inside the feature
// rather than beside it. So the seed cannot be measured after the step runs,
// and its azimuth, radius, height and volume have to be read during the build
// and handed to the assertion through these package-level variables.
//
// That hand-off leaves the case. Two cases running at once overwrite each
// other's readings and the proof reports a wrong verdict rather than failing
// loudly -- it is not a hazard that announces itself, because a pair of cases
// whose seeds measured alike would pass on each other's numbers. So this step
// keeps proofkit3d.RunSolid where every other solid step here takes the
// parallel runner.
var (
	patternSeedVolume   decad.Measurement
	patternSeedCentroid decad.VecMeasurement
	patternSeedBounds   decad.Box
)

// stepCircularPattern rotates the trimmed tooth into its copies about the
// SHAFT-AXIS EDGE, the hexagon's first edge, and never about the section 2
// Apex->A / Apex->B construction line.
//
// The pattern's three inputs are pinned rather than left to a default: the
// quantity is this gear's Teeth Number, the total angle is a full 360 degrees,
// and it is not symmetric. Although the pitch diameter shrinks from the heel
// toward the apex, the ANGULAR spacing stays at 360 / N for the whole face
// width -- the radial taper is already in the loft from the apex, so the
// pattern only rotates one tapered tooth into N evenly spaced copies.
//
// Building all N copies would prove nothing the first and the last do not, so
// the proof builds the increment and the last copy and reads the spacing off
// them.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	heelStation := f.Ded.X
	toeStation := f.Toe.X
	s0, p0, s1, p1, _, _ := toothSections(t, w, c, m,
		toeStation/heelStation, 1, toeStation, heelStation)
	seed, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the seed tooth loft failed: %v", m.Name, err)
	}

	// Read the seed BEFORE the increment retires it.
	patternSeedVolume, err = seed.Volume()
	if err != nil {
		t.Fatalf("%s: seed volume: %v", m.Name, err)
	}
	patternSeedCentroid, err = seed.Centroid()
	if err != nil {
		t.Fatalf("%s: seed centroid: %v", m.Name, err)
	}
	patternSeedBounds, err = seed.Bounds()
	if err != nil {
		t.Fatalf("%s: seed bounds: %v", m.Name, err)
	}

	axis := r3.NewVec(0, 0, 1)
	increment := 360 / m.Teeth
	step, err := r3.Rotation(axis, units.Degrees(increment))
	if err != nil {
		t.Fatalf("%s: the pattern increment is not a rotation: %v", m.Name, err)
	}
	// Placed RETIRES the seed, which is the whole reason the readings above had
	// to be taken first.
	first, err := seed.Placed(step)
	if err != nil {
		t.Fatalf("%s: the pattern increment failed: %v", m.Name, err)
	}
	rest, err := r3.Rotation(axis, units.Degrees(increment*(m.Teeth-2)))
	if err != nil {
		t.Fatalf("%s: the last copy's rotation is not a rotation: %v", m.Name, err)
	}
	last, err := first.PlacedCopy(rest)
	if err != nil {
		t.Fatalf("%s: the last copy failed: %v", m.Name, err)
	}
	return []*decad.Body{first, last}
}

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	increment := 2 * math.Pi / m.Teeth

	seedAz := math.Atan2(patternSeedCentroid.Value.Y, patternSeedCentroid.Value.X)
	seedRadius := math.Hypot(patternSeedCentroid.Value.X, patternSeedCentroid.Value.Y)

	for i, want := range []float64{increment, increment * (m.Teeth - 1)} {
		copyVolume, err := bodies[i].Volume()
		if err != nil {
			t.Fatalf("%s: copy %d volume: %v", m.Name, i, err)
		}
		// Both sides are readings, so Agree counts both bounds rather than
		// dropping the seed's.
		decadtest.Agree(t, m.Name+" patterned copy keeps the seed's volume",
			patternSeedVolume, copyVolume, oracleSlack)

		copyCentroid, err := bodies[i].Centroid()
		if err != nil {
			t.Fatalf("%s: copy %d centroid: %v", m.Name, i, err)
		}
		gotAz := math.Atan2(copyCentroid.Value.Y, copyCentroid.Value.X)
		near(t, fmt.Sprintf("%s copy %d sits one increment on", m.Name, i),
			foldAngle(gotAz-seedAz-want), 0, 1e-9)
		near(t, fmt.Sprintf("%s copy %d keeps the seed's radius", m.Name, i),
			math.Hypot(copyCentroid.Value.X, copyCentroid.Value.Y), seedRadius, 1e-9)
		near(t, fmt.Sprintf("%s copy %d keeps the seed's station", m.Name, i),
			copyCentroid.Value.Z, patternSeedCentroid.Value.Z, 1e-9)

		copyBounds, err := bodies[i].Bounds()
		if err != nil {
			t.Fatalf("%s: copy %d bounds: %v", m.Name, i, err)
		}
		near(t, fmt.Sprintf("%s copy %d keeps the seed's axial extent", m.Name, i),
			copyBounds.Max.Z-copyBounds.Min.Z,
			patternSeedBounds.Max.Z-patternSeedBounds.Min.Z, 1e-9)
	}

	// The quantity closes the circle exactly: N increments is one full turn, so
	// the last copy is one increment short of the seed.
	near(t, m.Name+" the pattern closes a full circle",
		m.Teeth*increment, 2*math.Pi, 1e-12)
}

// ---------------------------------------------------------------------------
// S31 -- the bore cut.
// ---------------------------------------------------------------------------

// stepBoreCut builds the cutting tool as a REAL extrude, which a symmetric
// extent produces as a prism, and performs NO CUT: the target is the frustum,
// whose bands are Lofts, and no boolean here takes a Loft.
//
// THE COST IS THE PIERCED BODY: one lump with a hole and no enclosed void is
// not shown. What is shown is the tool's own measured geometry -- its diameter,
// that its two ends sit exactly 2 * Cone Distance either side of the shaft
// edge's start, and that both of them clear the frustum, which is what makes it
// a THROUGH cut -- and the material it would remove, computed from the
// frustum's own profile clipped to the bore radius.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	if m.BoreDiameter == 0 {
		proofkit3d.Unmodelled(t, "%s: Enable Bore is unchecked, so no tool is built "+
			"and no cut is made", m.Name)
	}

	// The bore plane is rooted at the shaft edge's START, which is A' / B'.
	// The extent is symmetric, half a length per side, and 2 * Cone Distance is
	// generously past any face width.
	half := 2 * c.ConeDistance
	s := liftedSketch(t, w, f.Front.X)
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	circle := s.CreateCircle(centre, m.BoreDiameter/2)
	s.AddConstraint(sketch.NewDiameter(circle, m.BoreDiameter))
	profile := decadtest.SolveRegion(t, s)

	tool, err := doc.Extrude(s, profile, decad.Symmetric{D: units.Millimeters(half)})
	if err != nil {
		t.Fatalf("%s: the bore tool extrude failed: %v", m.Name, err)
	}

	// The frustum's own bands, laid apart from the tool, so the readings below
	// compare two real bodies rather than one body and a number.
	bodies := []*decad.Body{tool}
	for _, b := range bandsOf(c, m) {
		bodies = append(bodies, buildBand(t, doc, w, band{Name: "bore/" + b.Name,
			Near: b.Near, Far: b.Far, Lift: b.Lift + 6000, Reversed: b.Reversed}).Body)
	}
	return bodies
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	half := 2 * c.ConeDistance
	start := f.Front.X

	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s: bore tool bounds: %v", m.Name, err)
	}
	radius := m.BoreDiameter / 2
	decadtest.MeasuresBox(t, m.Name+" bore tool extent", box,
		r3.NewVec(-radius, -radius, start-half),
		r3.NewVec(radius, radius, start+half), oracleSlack)

	volume, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: bore tool volume: %v", m.Name, err)
	}
	decadtest.Measures(t, m.Name+" bore tool volume", volume,
		units.CubicMillimeters(math.Pi*radius*radius*2*half), chordSlack)

	// BOTH ENDS CLEAR THE FRUSTUM, which is what makes it a through cut. The
	// frustum runs from the toe corner's station to the heel's.
	lo := math.Min(f.Toe.X, f.Front.X)
	hi := math.Max(f.Heel.X, f.Ded.X)
	if start-half >= lo {
		t.Errorf("%s: the bore tool starts at %.4f, inside the frustum's %.4f",
			m.Name, start-half, lo)
	}
	if start+half <= hi {
		t.Errorf("%s: the bore tool ends at %.4f, inside the frustum's %.4f",
			m.Name, start+half, hi)
	}

	// THE MATERIAL IT WOULD REMOVE, from the frustum's own profile clipped to
	// the bore radius the tool itself measures. The clip is the whole profile,
	// not a cylinder assumed to fit: on a low-tooth-count pinion the bore
	// radius runs past the Toe Radius and the removed region then follows the
	// toe dish rather than the front face.
	measured := (box.Max.X - box.Min.X) / 2
	clipped := clipProfileBelow(f.profileLoop(), measured)
	removed := 2 * math.Pi * polygonCentroidY(clipped) * polygonArea(clipped)
	if removed <= 0 {
		t.Errorf("%s: the bore would remove no material at all", m.Name)
	}
	if removed >= math.Pi*radius*radius*2*half {
		t.Errorf("%s: the bore would remove %.4f mm^3, more than the whole tool holds",
			m.Name, removed)
	}
	span := hi - lo
	if removed > math.Pi*measured*measured*span+1e-9 {
		t.Errorf("%s: the bore would remove %.4f mm^3, more than a full cylinder "+
			"through the frustum's %.4f mm of span", m.Name, removed, span)
	}
	// Below the Toe Radius the profile's whole width is solid, so the clipped
	// region is a plain cylinder between the front face and the back face --
	// which is the closed form the clip has to reproduce.
	if measured < m.ToeRadius && measured < f.Heel.Y {
		near(t, m.Name+" material the bore would remove",
			removed, math.Pi*measured*measured*(f.Base.X-f.Front.X), removed*1e-9)
	} else {
		t.Logf("%s: the bore radius %.4f runs past the Toe Radius %.4f, so the "+
			"removed region follows the toe dish", m.Name, measured, m.ToeRadius)
	}
}

// clipProfileBelow cuts a profile loop to the half plane y <= limit, which is
// the frustum's own material inside the bore. The loop stays a single region:
// the hexagon's only reentrant corner is at the toe, and the cut runs across
// the figure rather than through that corner.
func clipProfileBelow(loop []planeVec, limit float64) []planeVec {
	out := make([]planeVec, 0, len(loop)+2)
	for i := range loop {
		a, b := loop[i], loop[(i+1)%len(loop)]
		aIn, bIn := a.Y <= limit, b.Y <= limit
		if aIn {
			out = append(out, a)
		}
		if aIn != bIn {
			tFrac := (limit - a.Y) / (b.Y - a.Y)
			out = append(out, pv(a.X+(b.X-a.X)*tFrac, limit))
		}
	}
	return out
}

// ---------------------------------------------------------------------------
// S32 -- the meshing rotation.
// ---------------------------------------------------------------------------

// stepMeshingRotation turns the DRIVING gear by half a tooth pitch about its
// own shaft axis, so a driving valley sits where the pinion tooth crosses the
// axial plane. Both gears are patterned from a starting tooth in that plane, so
// without the offset a driving tooth and a pinion tooth would sit at the same
// crossing and visually collide.
//
// THE PINION'S PHASE IS ZERO by default, and a zero angle is a NO-OP rather
// than a move: Fusion refuses to move a body by the identity, with
// `RuntimeError: 3 : invalid transform`. The framework helper returns early for
// exactly that reason, and the pinion case here takes the same early return.
func stepMeshingRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	heelStation := f.Ded.X
	toeStation := f.Toe.X
	s0, p0, s1, p1, _, _ := toothSections(t, w, c, m,
		toeStation/heelStation, 1, toeStation, heelStation)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the tooth loft failed: %v", m.Name, err)
	}

	angle := meshPhaseOf(c, m)
	if angle == 0 {
		// The early return. Returning the body unturned is what the helper does
		// and what the proof has to show, because a rotation by the identity is
		// an error rather than a no-op.
		return []*decad.Body{body}
	}
	turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s: the meshing rotation is not a rotation: %v", m.Name, err)
	}
	// Laid apart along the shaft axis, for the reason this file opens with: the
	// turned copy would otherwise sit inside the body it was copied from, and a
	// contact the read-only intersection cannot classify is reported as Suspect.
	// A slide along the axis leaves every azimuth, radius and volume below
	// unchanged, and the assertion subtracts the lift where it reads a station.
	apart, err := r3.Translation(r3.NewVec(0, 0, meshLift))
	if err != nil {
		t.Fatalf("%s: laying the copy apart failed: %v", m.Name, err)
	}
	moved, err := turn.Then(apart)
	if err != nil {
		t.Fatalf("%s: composing the rotation with the lift failed: %v", m.Name, err)
	}
	turned, err := body.PlacedCopy(moved)
	if err != nil {
		t.Fatalf("%s: the meshing rotation failed: %v", m.Name, err)
	}
	return []*decad.Body{body, turned}
}

// meshPhaseOf is the extra rotation each gear takes about its own shaft axis:
// half a tooth pitch on the driving gear, and the pinion's own mesh phase --
// pinionMeshPhaseTeeth tooth-fractions, zero by default -- on the pinion.
func meshPhaseOf(c bevelCase, m sideMember) float64 {
	if m.Name == "Driving" {
		return math.Pi / m.Teeth
	}
	return pinionMeshPhaseTeeth * 2 * math.Pi / m.Teeth
}

// pinionMeshPhaseTeeth is the pinion's extra mesh rotation in whole teeth. It
// is 0 for a straight bevel and stays 0 for a spiral one, because the spiral's
// mid-face section is left unrotated and so already meshes.
const pinionMeshPhaseTeeth = 0.0

// meshLift is how far along the shaft axis the turned copy is laid apart from
// the body it was turned from.
const meshLift = 10000.0

func assertMeshingRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	angle := meshPhaseOf(c, m)

	if angle == 0 {
		if len(bodies) != 1 {
			t.Errorf("%s: a zero mesh phase produced %d body(ies); the rotation must "+
				"be skipped, not performed by the identity", m.Name, len(bodies))
		}
		near(t, m.Name+" mesh phase", angle, 0, 0)
		return
	}
	near(t, m.Name+" mesh phase is half a tooth pitch", angle, math.Pi/m.Teeth, 1e-12)
	if len(bodies) != 2 {
		t.Fatalf("%s: the meshing rotation produced %d body(ies), want the original "+
			"and the turned copy", m.Name, len(bodies))
		return
	}

	before, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("%s: centroid before: %v", m.Name, err)
	}
	after, err := bodies[1].Centroid()
	if err != nil {
		t.Fatalf("%s: centroid after: %v", m.Name, err)
	}
	near(t, m.Name+" the body turned by exactly the mesh phase",
		foldAngle(math.Atan2(after.Value.Y, after.Value.X)-
			math.Atan2(before.Value.Y, before.Value.X)-angle), 0, 1e-9)
	near(t, m.Name+" the rotation kept the body's radius",
		math.Hypot(after.Value.X, after.Value.Y),
		math.Hypot(before.Value.X, before.Value.Y), 1e-9)
	near(t, m.Name+" the rotation moved the body along its axis by the lift alone",
		after.Value.Z-before.Value.Z, meshLift, 1e-9)

	volBefore, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("%s: volume before: %v", m.Name, err)
	}
	volAfter, err := bodies[1].Volume()
	if err != nil {
		t.Fatalf("%s: volume after: %v", m.Name, err)
	}
	decadtest.Agree(t, m.Name+" the rotation is rigid", volBefore, volAfter, oracleSlack)
}
