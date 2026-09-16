// Solid steps.
//
// NO BOOLEAN IS PERFORMED ANYWHERE IN THIS PROOF. At the decad revision this
// repository pins a boolean accepts only prism, cup and faceted payloads and
// refuses an operand built by Loft. Every solid in this gear is conical, and a
// cone is a Loft here, because Extrude refuses a nonzero taper. So the union
// that joins the frustum, the intersection and cut that trim the tooth, the
// bore's through-cut and the Combine-Join are all unavailable.
//
// The substitution is the same at every site: build the operands, lay them
// apart along the shaft axis, and assert from their own measured geometry what
// the operation would have produced. Laying them apart leaves every volume,
// radius and cone angle unchanged, which is what keeps the readings meaning
// something. Each step below says what its own substitution costs.
//
// Two conventions the whole file rests on.
//
// The (station, radius) frame. Every body here is a surface of revolution
// about one gear's shaft axis, so it is written as a profile in the half-plane
// whose first coordinate is the distance from the apex along that axis and
// whose second is the perpendicular distance from it. geometry_test.go's
// side.station and side.radius map a §2 lattice point into that frame, and
// side.hexProfile gives the whole frustum profile in it.
//
// The n-gon sweep. decad publishes a revolved body's volume with a proven
// bound equal to the volume itself, so a revolved body is Suspect at any
// tolerance and cannot pass the harness gate. Every surface of revolution here
// is therefore built as a loft between two regular n-gons, which is a
// polygonal sweep of the same profile. The ratio between an n-gon of
// circumradius r and the circle of that radius is exact, so every volume read
// off one of these bodies is the true one times polygonAreaFactor(bandSides)
// and no accuracy is lost — only the round face.
//
// The solid tables run at Module 4 to 8. decad's mesh bound has an absolute
// floor, so a figure small enough brings every measurement inside it and the
// gate reports Suspect on geometry that is in fact correct. Module is a pure
// scale on this figure, so a case at Module 4 proves the same shape as one at
// Module 1 and clears the floor.
package bevelgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// bandSides is the n-gon every sweep is built from. It is high enough that a
// band reads as the cone it stands for and low enough that a case table of
// this size still runs in seconds.
const bandSides = 96

// apexShrink is the section the degenerate apex point is substituted by. The
// loft's pointed end is the one thing the tooth step cannot build, so it ends
// at a section this fraction of the heel one instead; the cost is recorded at
// the step.
const apexShrink = 0.02

// layoutGap keeps laid-apart bodies from touching, in millimetres. Nothing is
// ever joined, so the only thing a gap has to do is keep two bodies from
// sharing a face.
const layoutGap = 5.0

func mm(v float64) units.Value { return units.Millimeters(v) }

// ---------------------------------------------------------------------------
// Section and band construction
// ---------------------------------------------------------------------------

// ringSection draws a regular bandSides-gon of circumradius r on plane and
// returns its one solved region.
func ringSection(t *testing.T, w *sketch.World, plane *sketch.Plane, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	pts := make([]pt, bandSides)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / float64(bandSides)
		pts[i] = pt{r * math.Cos(a), r * math.Sin(a)}
	}
	return polySection(t, w, plane, pts)
}

// polySection draws a closed chorded contour and returns its one solved
// region. Every point is fixed, so the section is placed geometry: this is a
// loft input, not a constraint scheme, and the constraint schemes are proved
// in sketches_test.go.
func polySection(t *testing.T, w *sketch.World, plane *sketch.Plane, pts []pt) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create section sketch: %v", err)
	}
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p.X, p.Y)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	return s, decadtest.SolveRegion(t, s)
}

// band is one swept band of a profile edge: the solid a straight edge of the
// (station, radius) profile sweeps about the shaft axis, built as an n-gon
// loft and laid at its own place along the layout axis.
type band struct {
	name   string
	r0, r1 float64 // circumradius at the low and high end
	h      float64 // height
	z0     float64 // where it was laid
	body   *decad.Body
}

// buildBand lofts one band. r0 and r1 are the two ring radii and h the height;
// z0 is where the band is laid so nothing touches anything else.
func buildBand(t *testing.T, doc *decad.Document, w *sketch.World, name string, r0, r1, h, z0 float64) band {
	t.Helper()
	if h <= 0 {
		t.Fatalf("%s: band height must be positive, got %g", name, h)
	}
	lo, err := w.CreateOffsetPlane(w.XY(), z0)
	if err != nil {
		t.Fatalf("%s: low plane: %v", name, err)
	}
	hi, err := w.CreateOffsetPlane(w.XY(), z0+h)
	if err != nil {
		t.Fatalf("%s: high plane: %v", name, err)
	}
	s0, p0 := ringSection(t, w, lo, r0)
	s1, p1 := ringSection(t, w, hi, r1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: loft: %v", name, err)
	}
	return band{name: name, r0: r0, r1: r1, h: h, z0: z0, body: body}
}

// reading is one measured quantity with the bound decad proved for it, so a
// value derived from several readings can carry the sum of their bounds
// instead of a tolerance invented for the comparison.
type reading struct {
	value float64
	bound float64
}

func volumeReading(t *testing.T, b *decad.Body, label string) decad.Measurement {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return m
}

func boundsReading(t *testing.T, b *decad.Body, label string) decad.Box {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", label, err)
	}
	return box
}

// capAreaReading reads the area of the face the named loft section produced.
func capAreaReading(t *testing.T, b *decad.Body, ref decad.FeatureRef, label string) decad.Measurement {
	t.Helper()
	faces, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(b)
	if err != nil {
		t.Fatalf("%s cap face: %v", label, err)
	}
	m, err := faces[0].Area()
	if err != nil {
		t.Fatalf("%s cap area: %v", label, err)
	}
	return m
}

// capRadius turns a cap's area reading into the circumradius of the n-gon that
// produced it, carrying the reading's own bound through the conversion.
func capRadius(m decad.Measurement) reading {
	c := 0.5 * float64(bandSides) * math.Sin(2*math.Pi/float64(bandSides))
	a := m.Value.Base()
	r := math.Sqrt(a / c)
	return reading{value: r, bound: m.Bound.Base() / (2 * c * r)}
}

// wallSlope is a band's cone half-angle read off the body itself: the two cap
// radii and the height, each a bounded reading, combined into the tangent of
// the angle the swept edge makes with the axis. The bound it carries is the
// three readings' own bounds propagated, so a comparison against it never
// drops what decad proved.
func wallSlope(t *testing.T, b band, label string) reading {
	t.Helper()
	lo := capRadius(capAreaReading(t, b.body, decad.CapStart(b.body), label+" start"))
	hi := capRadius(capAreaReading(t, b.body, decad.CapEnd(b.body), label+" end"))
	box := boundsReading(t, b.body, label)
	h := box.Max.Z - box.Min.Z
	hb := 2 * box.Bound.Base()
	slope := (hi.value - lo.value) / h
	return reading{
		value: slope,
		bound: (hi.bound+lo.bound)/h + math.Abs(slope)*hb/h,
	}
}

// agreesWithin compares a value derived from several readings against a closed
// form, adding the propagated bound to the slack the formula itself carries.
// A single reading is compared with decadtest.Measures instead, which does the
// same thing with decad's own bound.
func agreesWithin(t *testing.T, what string, got reading, want, slack float64) {
	t.Helper()
	if d := math.Abs(got.value - want); d > got.bound+slack {
		t.Errorf("%s reads %.9f, want %.9f: off by %.3e against a proven bound of %.3e plus %.3e of formula slack",
			what, got.value, want, d, got.bound, slack)
	}
}

// ---------------------------------------------------------------------------
// S13 Revolve — the gear body
// ---------------------------------------------------------------------------

// frustumBands is the three-band decomposition of the revolved hexagon. The
// hexagon's profile in the (station, radius) frame is
//
//	A'(s0, 0) -> G(s1, 0) -> H(s1, rH) -> C(sC, rC) -> M(sM, rM) -> N(s0, rN)
//
// with sM < sC < s1 and s0 = sN, so the body over [sM, sC] is bounded outside
// by the root cone, over [sC, s1] by the heel (back) cone, and the toe dish is
// the wedge the toe line takes OUT of the root cone between sM and s0. The
// signed sum is therefore root + heel - toe, which is what "the toe-dish plug
// that hollows the front face" names.
func frustumBands(t *testing.T, doc *decad.Document, w *sketch.World, g geometry, sd side, base float64) []band {
	t.Helper()
	hex := sd.hexProfile(g)
	s0, s1 := hex[0].X, hex[1].X
	sC, rC := hex[3].X, hex[3].Y
	sM, rM := hex[4].X, hex[4].Y
	rH, rN := hex[2].Y, hex[5].Y

	z := base
	lay := func(h float64) float64 {
		at := z
		z += h + layoutGap
		return at
	}
	return []band{
		buildBand(t, doc, w, "root cone band", rM, rC, sC-sM, lay(sC-sM)),
		buildBand(t, doc, w, "heel cone band", rC, rH, s1-sC, lay(s1-sC)),
		buildBand(t, doc, w, "toe dish plug", rM, rN, s0-sM, lay(s0-sM)),
	}
}

func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()
	bands := frustumBands(t, doc, w, g, sd, 0)
	out := make([]*decad.Body, len(bands))
	for i, b := range bands {
		out[i] = b.body
	}
	return out
}

func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	hex := sd.hexProfile(g)
	k := polygonAreaFactor(bandSides)

	names := []string{"root cone band", "heel cone band", "toe dish plug"}
	s0, s1 := hex[0].X, hex[1].X
	sC, rC := hex[3].X, hex[3].Y
	sM, rM := hex[4].X, hex[4].Y
	rH, rN := hex[2].Y, hex[5].Y
	want := []struct{ r0, r1, h float64 }{
		{rM, rC, sC - sM},
		{rC, rH, s1 - sC},
		{rM, rN, s0 - sM},
	}

	// Each band against its own stations and ring radii.
	total := 0.0
	totalBound := 0.0
	for i, b := range bodies {
		vol := volumeReading(t, b, names[i])
		decadtest.Measures(t, names[i]+" volume", vol,
			units.CubicMillimeters(frustumVolume(want[i].r0, want[i].r1, want[i].h, bandSides)),
			decadtest.WithinRel(units.Scalar(1e-12)))
		sign := 1.0
		if i == 2 {
			sign = -1 // the toe dish is hollowed OUT of the root cone
		}
		total += sign * vol.Value.Base()
		totalBound += vol.Bound.Base()

		box := boundsReading(t, b, names[i])
		rMax := math.Max(want[i].r0, want[i].r1)
		decadtest.MeasuresBox(t, names[i]+" extent", box,
			r3.NewVec(-rMax, -rMax, bandZ(i, want)),
			r3.NewVec(rMax, rMax, bandZ(i, want)+want[i].h))
	}

	// The signed sum against Pappus on the §2 hexagon. pappusVolume keeps the
	// walk's sign, so an inverted figure would come back negative here rather
	// than agreeing by accident.
	pappus := pappusVolume(hex)
	agreesWithin(t, "the three bands' signed sum against Pappus on the §2 hexagon",
		reading{value: total, bound: totalBound}, k*math.Abs(pappus), 1e-9*math.Abs(k*pappus))

	// Cone half-angle by cone half-angle. The heel band and the toe plug come
	// out parallel, on the back-cone family at 90 - gamma to the shaft axis,
	// and the root band sits at the dedendum angle to them.
	root := wallSlope(t, band{body: bodies[0]}, "root cone band")
	heel := wallSlope(t, band{body: bodies[1]}, "heel cone band")
	toe := wallSlope(t, band{body: bodies[2]}, "toe dish plug")
	slack := 1e-9
	agreesWithin(t, "root band wall slope", root, math.Tan(sd.gammaRoot), slack)
	agreesWithin(t, "heel band wall slope", heel, -math.Tan(math.Pi/2-sd.gamma), slack)
	agreesWithin(t, "toe plug wall slope", toe, -math.Tan(math.Pi/2-sd.gamma), slack)
	agreesWithin(t, "the heel band and the toe plug are parallel",
		reading{value: heel.value - toe.value, bound: heel.bound + toe.bound}, 0, slack)
	deltaF := math.Atan(g.Dedendum / g.R)
	agreesWithin(t, "the root band stands at the dedendum angle to the back-cone family",
		reading{value: math.Atan(root.value) + math.Atan(-heel.value), bound: root.bound + heel.bound},
		math.Pi/2-deltaF, 1e-8)

	// What the substitution costs, recorded where it is paid: the proof does
	// not show the three bands closing into one watertight solid. Each is
	// separately watertight — RequireSolid reads that off every body — and
	// together they have the right volume, stations and angles. The union
	// itself is unavailable, because every operand is a Loft.
	_ = doc
}

// bandZ recovers where frustumBands laid the i-th band, so the extent
// assertion can name it.
func bandZ(i int, want []struct{ r0, r1, h float64 }) float64 {
	z := 0.0
	for j := 0; j < i; j++ {
		z += want[j].h + layoutGap
	}
	return z
}

// ---------------------------------------------------------------------------
// S14 Tooth loft
// ---------------------------------------------------------------------------

// toothSection is the tooth's cross-section in the back-cone plane, as a
// chorded closed contour centred on the tooth-centre point. decad refuses a
// loft whose corresponding segments are not the same kind, and it refuses a
// free-form pair outright, so the involute flanks and the two arcs are all
// chorded. The cost is the curvature: the section's area and its tip and root
// reach are the real ones, and the flank's own shape between samples is not.
func toothSection(g geometry, sd side) []pt {
	ring := buildVirtualToothRing(g, sd)
	out := make([]pt, 0, 2*len(ring.left)+2)
	// root arc -> left flank -> tip arc -> right flank, chorded.
	for i := range ring.left {
		out = append(out, pt{ring.left[i].X, ring.left[i].Y})
	}
	for i := len(ring.right) - 1; i >= 0; i-- {
		out = append(out, pt{ring.right[i].X, ring.right[i].Y})
	}
	if !ring.embedded {
		lx, ly := radialTo(ring.right[0], ring.root)
		rx, ry := radialTo(ring.left[0], ring.root)
		out = append(out, pt{lx, ly}, pt{rx, ry})
	}
	return out
}

func scalePts(in []pt, k float64, about pt) []pt {
	out := make([]pt, len(in))
	for i, p := range in {
		out[i] = about.add(p.sub(about).scale(k))
	}
	return out
}

func stepToothLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()

	// The apex's PERPENDICULAR distance to the tooth's section plane. The
	// tooth centre K'/L' sits at station R/cos(gamma) from the apex along the
	// shaft axis, and the section plane's normal runs along the pitch element,
	// so that perpendicular distance is R. The tooth plane itself is NOT
	// substituted: this is the real back-cone plane's distance, tilted out of
	// the axis-perpendicular by gamma, rather than the along-axis station.
	h := g.R
	section := toothSection(g, sd)

	heelPlane, err := w.CreateOffsetPlane(w.XY(), h)
	if err != nil {
		t.Fatalf("heel plane: %v", err)
	}
	// The degenerate apex point is the one thing this step cannot build: a
	// loft needs two non-degenerate sections. It ends at a shrunken section
	// instead, so what the volume and the section scaling prove is the taper
	// the real loft has to produce, not its point.
	s0, p0 := polySection(t, w, w.XY(), scalePts(section, apexShrink, pt{0, 0}))
	s1, p1 := polySection(t, w, heelPlane, section)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("apex loft: %v", err)
	}
	return []*decad.Body{body}
}

func assertToothLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	section := toothSection(g, sd)
	area := math.Abs(shoelace(section))
	h := g.R
	body := bodies[0]

	decadtest.Measures(t, "tooth body volume", volumeReading(t, body, "tooth"),
		units.CubicMillimeters(h/3*(area+apexShrink*apexShrink*area+apexShrink*area)),
		decadtest.WithinRel(units.Scalar(1e-12)))

	// The taper, read as the two cap areas: the section scales linearly from
	// the shrunken apex end to the heel one, which is what the loft from a
	// point has to produce.
	heelCap := capAreaReading(t, body, decad.CapEnd(body), "tooth heel section")
	apexCap := capAreaReading(t, body, decad.CapStart(body), "tooth apex section")
	decadtest.Measures(t, "tooth heel section area", heelCap,
		units.SquareMillimeters(area), decadtest.WithinRel(units.Scalar(1e-12)))
	decadtest.Measures(t, "tooth apex section area", apexCap,
		units.SquareMillimeters(apexShrink*apexShrink*area), decadtest.WithinRel(units.Scalar(1e-12)))

	// The heel section reaches the virtual tip radius laid on the back cone,
	// which is the tip circle §3 dimensions at virtualPitchRadius + Module.
	ring := buildVirtualToothRing(g, sd)
	box := boundsReading(t, body, "tooth")
	decadtest.MeasuresBox(t, "tooth extent", box,
		r3.NewVec(box.Min.X, box.Min.Y, 0), r3.NewVec(box.Max.X, box.Max.Y, h))
	reach := math.Max(math.Abs(box.Min.X), math.Abs(box.Max.X))
	reach = math.Max(reach, math.Max(math.Abs(box.Min.Y), math.Abs(box.Max.Y)))
	// The heel section's own extent, which the loft's bounding box has to
	// reproduce. The section is CHORDED, so its axis-aligned extent falls a
	// little short of the tip radius; that shortfall is the chording cost and
	// it is stated here rather than absorbed into a tolerance.
	sectionReach := 0.0
	sectionRadius := 0.0
	for _, q := range section {
		sectionReach = math.Max(sectionReach, math.Max(math.Abs(q.X), math.Abs(q.Y)))
		sectionRadius = math.Max(sectionRadius, q.len())
	}
	agreesWithin(t, "the tooth's heel end reaches its own section's extent",
		reading{value: reach, bound: box.Bound.Base()}, sectionReach, 1e-9*sectionReach)
	// And the section itself reaches the virtual tip radius, which is the tip
	// circle §3 dimensions at virtualPitchRadius + Module. This is the fact a
	// later step selects on, so it is asserted on the geometry actually built.
	requireClose(t, "the tooth section's outermost point", sectionRadius, ring.dims.Tip, 1e-9)
	_ = doc
}

// ---------------------------------------------------------------------------
// S22 Conical end cuts
// ---------------------------------------------------------------------------

// stepConicalEndCut performs NEITHER cut. Both operands are Lofts — the tooth
// and each cone alike — so the split is unavailable. It builds the tooth and
// the two cones and lays them apart; the assertion reads each cone's apex and
// half-angle off the cone and each of the tooth's two surfaces off the tooth,
// solves the stations where they cross from those readings, and checks them
// against the flush band.
func stepConicalEndCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()
	hex := sd.hexProfile(g)

	tooth := stepToothLoft(t, doc, p)[0]
	z := g.R + layoutGap
	toe := buildBand(t, doc, w, "toe cutting cone", hex[4].Y, hex[5].Y, hex[0].X-hex[4].X, z)
	z += hex[0].X - hex[4].X + layoutGap
	heel := buildBand(t, doc, w, "heel cutting cone", hex[3].Y, hex[2].Y, hex[1].X-hex[3].X, z)
	return []*decad.Body{tooth, toe.body, heel.body}
}

func assertConicalEndCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	hex := sd.hexProfile(g)
	ring := buildVirtualToothRing(g, sd)

	// The cutting TOOLS are cone faces of the revolved frustum; the TARGET is
	// the lofted tooth. The two are never conflated: the tooth has no cone
	// face, so searching it for one finds none.
	toe := wallSlope(t, band{body: bodies[1]}, "toe cutting cone")
	heel := wallSlope(t, band{body: bodies[2]}, "heel cutting cone")

	// Each cone's half-angle is this gear's back-cone half-angle, 90 - gamma,
	// at the same slope tolerance the revolve's bands use. The flush-band
	// reading below cannot see this: any band through M crosses the gear
	// body's root ray at M whatever slope it has.
	slack := 1e-9
	agreesWithin(t, "toe cone half-angle", toe, -math.Tan(math.Pi/2-sd.gamma), slack)
	agreesWithin(t, "heel cone half-angle", heel, -math.Tan(math.Pi/2-sd.gamma), slack)

	// Where each cut lands. Both cones have their apex on the shaft axis. In
	// the (station, radius) frame a cone of wall slope k and apex station a is
	// rho = (a - s) * k, and the gear body's own root ray is rho = s * tan
	// gamma_root, so the two meet at a*k/(tan gamma_root + k).
	rootSlope := math.Tan(sd.gammaRoot)
	for _, c := range []struct {
		name     string
		slope    reading
		throughS float64
		throughR float64
		want     float64
	}{
		{"toe", toe, hex[4].X, hex[4].Y, hex[4].X},   // meets the root cone at M/O
		{"heel", heel, hex[3].X, hex[3].Y, hex[3].X}, // meets the root cone at C/D
	} {
		k := -c.slope.value
		apexStation := c.throughS + c.throughR/k
		landing := apexStation * k / (rootSlope + k)
		agreesWithin(t, fmt.Sprintf("the %s cut lands on the gear body's root cone", c.name),
			reading{value: landing, bound: c.slope.bound * apexStation / k}, c.want, 1e-6*g.R)

		// Each cone crosses the tooth's tip inboard of where it crosses the
		// tooth's root, so the trimmed end is shorter at the tip than at the
		// root. This is NOT asserted as "the two crossings differ": every cut
		// with a finite tilt crosses both surfaces at different stations, so
		// such an assertion would pass on any figure this spec can build.
		mTip := ring.dims.Tip / g.R
		mRoot := ring.root / g.R
		tipAt := apexStation * k / (mTip + k)
		rootAt := apexStation * k / (mRoot + k)
		if !(tipAt < rootAt) {
			t.Errorf("the %s cut crosses the tooth's tip at station %.6f and its root at %.6f; "+
				"the tip crossing has to be the inboard one", c.name, tipAt, rootAt)
		}
	}

	// What this substitution cannot reach, recorded beside the assertions it
	// does make:
	//
	//   - The split itself. The proof does not show the evaluator dividing the
	//     tooth, selecting the keeper, or leaving a watertight body.
	//   - That the face is CONICAL rather than a tilted plane. In the axial
	//     section a cone of half-angle 90 - gamma and a plane tilted by gamma
	//     through the same generator are the same line, so every station
	//     solved above comes out the same for either surface. What makes the
	//     face conical is that it is a surface of revolution about the shaft
	//     axis: its crossing with the tooth's tip sits at one station at every
	//     azimuth, while a tilted plane's crossing moves with azimuth. The
	//     proof measures that nowhere. It has it BY CONSTRUCTION, because each
	//     cutting tool here is a band swept about that axis and a swept band
	//     cannot be anything else.
	_ = doc
}

// ---------------------------------------------------------------------------
// S23 Circular pattern — the one step that stays SERIAL
// ---------------------------------------------------------------------------

// The pattern increment retires the seed tooth, so the seed cannot be measured
// after the step runs: its volume, centroid and radius have to be read during
// the build and handed to the assertion. That hand-off leaves the case, and
// two cases sharing one set of seed readings overwrite each other. It is not a
// hazard that announces itself — two cases whose seeds measured alike would
// pass on each other's numbers — so THIS STEP IS REGISTERED SERIALLY, on
// proofkit3d.RunSolid, while every other step in this proof runs its cases in
// parallel. Nothing else in the bevel proof carries a reading from its build
// into its assertion; a step that acquires one moves to the serial runner in
// the same change.
var (
	patternSeedVolume   decad.Measurement
	patternSeedCentroid decad.VecMeasurement
	patternSeedBounds   decad.Box
)

func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()
	section := toothSection(g, sd)

	// The seed tooth. Its own section already sits at the gear's rim — the
	// tooth is drawn on the back-cone plane whose origin K'/L' is on the shaft
	// axis, so the tooth's polar radius in that plane IS its distance from the
	// axis — and the pattern turns it about that axis.
	heelPlane, err := w.CreateOffsetPlane(w.XY(), g.R)
	if err != nil {
		t.Fatalf("heel plane: %v", err)
	}
	s0, p0 := polySection(t, w, w.XY(), scalePts(section, apexShrink, pt{0, 0}))
	s1, p1 := polySection(t, w, heelPlane, section)
	seed, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("seed tooth loft: %v", err)
	}

	patternSeedVolume = volumeReading(t, seed, "pattern seed")
	patternSeedCentroid, err = seed.Centroid()
	if err != nil {
		t.Fatalf("pattern seed centroid: %v", err)
	}
	patternSeedBounds = boundsReading(t, seed, "pattern seed")

	// The pattern is a full circle of this gear's Teeth Number copies at
	// 360/N apart. Building every copy would prove nothing three copies do
	// not, so the proof takes the first increment, one near the half turn and
	// the last; the seed itself is retired by the increment, exactly as the
	// pattern feature retires it.
	n := int(sd.teeth)
	pitch := 2 * math.Pi / float64(n)
	out := []*decad.Body{}
	for _, k := range patternIndices(n) {
		turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(pitch*float64(k)))
		if err != nil {
			t.Fatalf("pattern rotation %d: %v", k, err)
		}
		// Lay each copy apart along the shaft axis. Patterned teeth at
		// adjacent azimuths reach a contact decad's read-only intersection
		// cannot classify, and nothing here is joined, so the lift costs
		// nothing and keeps every reading the pattern is about — volume,
		// radius and azimuth — exactly as it was.
		lift, err := r3.Translation(r3.NewVec(0, 0, patternLift(g, len(out)+1)))
		if err != nil {
			t.Fatalf("pattern lift %d: %v", k, err)
		}
		placement, err := turn.Then(lift)
		if err != nil {
			t.Fatalf("pattern placement %d: %v", k, err)
		}
		copyBody, err := seed.PlacedCopy(placement)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", k, err)
		}
		out = append(out, copyBody)
	}
	// Retire the seed, which is what the pattern increment does to it: after
	// this the seed cannot be measured again, and the readings taken above are
	// the only record of it.
	retire, err := r3.Translation(r3.NewVec(0, 0, patternLift(g, 0)))
	if err != nil {
		t.Fatalf("seed retirement: %v", err)
	}
	moved, err := seed.Placed(retire)
	if err != nil {
		t.Fatalf("seed retirement move: %v", err)
	}
	out = append(out, moved)
	return out
}

// patternLift is where the i-th patterned body is laid along the shaft axis.
func patternLift(g geometry, i int) float64 {
	return float64(i) * (g.R + layoutGap)
}

func patternIndices(n int) []int {
	idx := []int{1}
	if n/2 != 1 {
		idx = append(idx, n/2)
	}
	if n-1 != 1 && n-1 != n/2 {
		idx = append(idx, n-1)
	}
	return idx
}

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	n := int(sd.teeth)
	pitch := 2 * math.Pi / float64(n)
	idx := patternIndices(n)

	if len(bodies) < len(idx) {
		t.Fatalf("the pattern built %d bodies, want at least %d copies", len(bodies), len(idx))
	}
	for i, k := range idx {
		b := bodies[i]
		// Both sides are readings, so Agree counts both bounds.
		decadtest.Agree(t, fmt.Sprintf("pattern copy %d against the retired seed", k),
			volumeReading(t, b, fmt.Sprintf("copy %d", k)), patternSeedVolume,
			decadtest.WithinRel(units.Scalar(1e-9)))

		got, err := b.Centroid()
		if err != nil {
			t.Fatalf("copy %d centroid: %v", k, err)
		}
		a := pitch * float64(k)
		c := patternSeedCentroid.Value
		want := r3.NewVec(c.X*math.Cos(a)-c.Y*math.Sin(a), c.X*math.Sin(a)+c.Y*math.Cos(a),
			c.Z+patternLift(g, i+1))
		decadtest.MeasuresVec(t, fmt.Sprintf("pattern copy %d centroid", k), got, want,
			decadtest.Within(mm(patternSeedCentroid.Bound.Base()+1e-6)))
	}

	// The angular spacing stays constant at 360/N for the whole face width,
	// even though the pitch diameter shrinks from heel toward apex: the radial
	// taper is already produced by the loft from the apex, so the pattern just
	// rotates one tapered tooth into N evenly spaced copies. quantity = Teeth
	// Number, totalAngle = 360 deg, isSymmetric = False.
	requireClose(t, "pattern pitch angle", pitch*float64(n), 2*math.Pi, 1e-12)
	if patternSeedBounds.Max.Z-patternSeedBounds.Min.Z <= 0 {
		t.Fatal("the retired seed's own extent was never read")
	}
	_ = doc
}

// ---------------------------------------------------------------------------
// S24 Combine-Join
// ---------------------------------------------------------------------------

// stepCombineJoin performs NO join. It lays the tooth and the gear body's root
// cone apart and the assertion reads the join's two consequences off their own
// measured geometry.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()
	hex := sd.hexProfile(g)

	tooth := stepToothLoft(t, doc, p)[0]
	root := buildBand(t, doc, w, "gear body root cone", hex[4].Y, hex[3].Y,
		hex[3].X-hex[4].X, g.R+layoutGap)
	return []*decad.Body{tooth, root.body}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	ring := buildVirtualToothRing(g, sd)

	// The gear body's root cone, read off the band this step built rather than
	// restated: it is the cone the tooth has to seat on.
	rootCone := wallSlope(t, band{body: bodies[1]}, "gear body root cone")
	agreesWithin(t, "the gear body's root cone slope", rootCone, math.Tan(sd.gammaRoot), 1e-9)

	// A join leaves ONE lump when the tooth's root is below the body's root
	// cone — seated, not floating.
	//
	// READ THE ROOT ARC'S OUTERMOST POINT, NOT THE TOOTH'S CENTRELINE. The
	// tooth is drawn on the back-cone PLANE while the body's root surface is a
	// CONE, so only the point on the tooth's own centreline rides the cone its
	// polar radius names; a corner lying py off that centreline at polar
	// radius r stands outside the cone by py^2 / (2 * r * cos gamma). The
	// centreline sits inside both corners, so a reading taken there passes a
	// tooth whose corners float — exactly the defect the root sink exists to
	// remove.
	corner := rootArcCorner(ring)
	excess := corner.Y * corner.Y / (2 * ring.root * math.Cos(sd.gamma))
	if g.RootSink <= excess {
		t.Errorf("the root arc's corner stands %.6f mm outside the gear body's root cone (%.4f module) "+
			"against a root sink of %.6f mm: the arc would meet the cone along one line and the join "+
			"would not overlap across the root", excess, excess/g.Module, g.RootSink)
	}
	// The sink is one figure, not a proof-only offset: the generated module
	// draws its root circle the same distance inside the dedendum corner.
	requireClose(t, "root sink", g.RootSink, 0.05*2.25*g.Module, 1e-12)
	requireClose(t, "the root circle sits one sink inside the dedendum corner",
		ring.dims.Root-ring.root, g.RootSink, 1e-12)

	// And the joined body reaches further out than the frustum, because the
	// tooth's tip stands proud of the root cone by the working depth.
	proud := ring.dims.Tip - ring.dims.Root
	requireClose(t, "the tooth stands proud of the root cone", proud, 2.25*g.Module, 1e-9)

	// Both readings taken at the toe, the middle and the heel of the band the
	// join would cover. The tooth is lofted from the apex and the root cone
	// runs from the same apex, so both scale by the same fraction of the cone
	// distance and the margin scales with them; the three readings are what
	// show that, rather than one reading standing in for all of them.
	hex := sd.hexProfile(g)
	toe, heel := hex[4].X, hex[3].X
	for _, station := range []struct {
		name string
		s    float64
	}{
		{"toe", toe}, {"middle", 0.5 * (toe + heel)}, {"heel", heel},
	} {
		f := station.s / g.R
		if seated := f * (ring.dims.Root - ring.root - excess); seated <= 0 {
			t.Errorf("at the %s the tooth's root corner clears the body's root cone by %.6f mm; "+
				"a join there would leave two lumps", station.name, seated)
		}
		if f*proud <= 0 {
			t.Errorf("at the %s the tooth's tip stands proud by %.6f mm; the join would add nothing",
				station.name, f*proud)
		}
	}

	// The cost is the stitch: the proof cannot show the evaluator making one
	// boundary out of two. Fusion has made it, on the shipped default pair and
	// on a 16 driving / 12 pinion pair at Module 4, and it came back as one
	// solid per gear; that is the only place this has been seen.
	_ = doc
}

// rootArcCorner is one end of the tooth's root arc, in the tooth plane's own
// frame. The tooth is centred on the -X direction after the 180 degree draw
// angle and is symmetric about the X axis, so the corner's Y component is its
// offset from the tooth's centreline.
func rootArcCorner(ring virtualToothRing) pt {
	q := ring.left[0]
	if !ring.embedded {
		x, y := radialTo(q, ring.root)
		return pt{x, y}
	}
	return pt{q.X, q.Y}
}

// ---------------------------------------------------------------------------
// S27 Bore through-cut
// ---------------------------------------------------------------------------

// stepBoreCut builds the tool as a REAL extrude, which a symmetric extent
// produces as a prism, but performs no cut: the target is the frustum, whose
// bands are Lofts.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	if !g.BoreEnable {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so no bore is cut on either gear and "+
			"the per-gear bore diameter inputs are ignored")
		return nil
	}
	w := sketch.NewWorld()
	s, region := ringSection(t, w, w.XY(), sd.bore/2)
	// setSymmetricExtent(ValueInput.createByReal(2 * Cone Distance), False):
	// the second argument is isFullLength=False, so the value is the
	// half-length PER SIDE. Generously past any face width.
	tool, err := doc.Extrude(s, region, decad.Symmetric{D: mm(2 * g.ConeDist)})
	if err != nil {
		t.Fatalf("bore tool extrude: %v", err)
	}
	// The bore tool spans 2 * Cone Distance either side of the shaft edge's
	// start, so the bands are laid clear of it rather than through it: nothing
	// here is cut, and two bodies sharing a face would be a contact decad
	// cannot classify.
	bands := frustumBands(t, doc, w, g, sd, 2*g.ConeDist+layoutGap)
	out := []*decad.Body{tool}
	for _, b := range bands {
		out = append(out, b.body)
	}
	return out
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	hex := sd.hexProfile(g)
	tool := bodies[0]

	// The tool's own diameter, and that its two ends sit exactly 2 * Cone
	// Distance either side of the shaft edge's start, which is what makes it a
	// THROUGH cut.
	area := ringArea(sd.bore/2, bandSides)
	decadtest.Measures(t, "bore tool volume", volumeReading(t, tool, "bore tool"),
		units.CubicMillimeters(area*4*g.ConeDist), decadtest.WithinRel(units.Scalar(1e-12)))
	decadtest.MeasuresBox(t, "bore tool extent", boundsReading(t, tool, "bore tool"),
		r3.NewVec(-sd.bore/2, -sd.bore/2, -2*g.ConeDist),
		r3.NewVec(sd.bore/2, sd.bore/2, 2*g.ConeDist))

	// Both ends clear the frustum: the hexagon's own stations never reach
	// 2 * Cone Distance from the shaft edge's start.
	if hex[1].X >= 2*g.ConeDist {
		t.Errorf("the bore tool's half-length %.4f mm does not clear the gear body's heel station "+
			"%.4f mm, so the cut would not pierce it", 2*g.ConeDist, hex[1].X)
	}

	// The material the cut would remove, from the frustum's own profile
	// clipped to the bore radius. Every ring of material inside the bore
	// radius goes, so the removed volume is the clipped profile revolved.
	clipped := clipProfile(hex[:], sd.bore/2)
	removed := math.Abs(pappusVolume6(clipped)) * polygonAreaFactor(bandSides)
	if removed <= 0 {
		t.Errorf("the bore would remove no material at diameter %.4f mm", sd.bore)
	}

	// The bore is bounded before anything is revolved, which is what keeps it
	// from taking an end face off the body.
	if sd.bore > sd.maxBore+1e-9 {
		t.Errorf("%s resolved bore diameter %.6f mm exceeds its Maximum Bore Diameter %.6f mm; "+
			"past the heel term the cut takes the entire flat back face and past the toe term the "+
			"whole toe dish", sd.which.label(), sd.bore, sd.maxBore)
	}
	rHeel := sd.pitchRadius - sd.baseHeight/math.Tan(sd.gamma)
	rToe := (g.ApexDed - g.RootLen) * math.Sin(sd.gammaRoot)
	requireClose(t, sd.which.label()+" Maximum Bore Diameter",
		sd.maxBore, 2*0.95*math.Min(rHeel, rToe), 1e-9)

	// The cost is the pierced body: one lump with a hole and no enclosed void
	// is not shown.
	_ = doc
}

// clipProfile returns the part of a (station, radius) profile that lies inside
// a radius, as the closed contour the bore would take out.
func clipProfile(profile []pt, r float64) []pt {
	out := make([]pt, 0, len(profile)+4)
	for i := range profile {
		a := profile[i]
		out = append(out, pt{a.X, math.Min(a.Y, r)})
	}
	return out
}

// pappusVolume6 is pappusVolume over a slice rather than a fixed array.
func pappusVolume6(profile []pt) float64 {
	total := 0.0
	for i := range profile {
		a, b := profile[i], profile[(i+1)%len(profile)]
		total += (b.X - a.X) * (a.Y*a.Y + a.Y*b.Y + b.Y*b.Y) / 3
	}
	return math.Pi * total
}

// ---------------------------------------------------------------------------
// S28 Meshing rotation
// ---------------------------------------------------------------------------

func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	w := sketch.NewWorld()
	hex := sd.hexProfile(g)

	// One band of the gear body stands in for the whole: the rotation is a
	// free move about the shaft axis, and what it has to leave unchanged is
	// the body's own volume while moving its centroid by the phase.
	body := buildBand(t, doc, w, "gear body heel band", hex[3].Y, hex[2].Y, hex[1].X-hex[3].X, 0).body
	before := volumeReading(t, body, "gear body before the mesh rotation")

	angle := meshPhase(sd)
	if angle == 0 {
		// A zero angle is a no-op, not a move: setToRotation(0, axis, origin)
		// builds the identity and Fusion refuses it with "invalid transform".
		// rotate_body_about_edge absorbs that, which is why the pinion's
		// default phase of 0 reaches no call site.
		return []*decad.Body{body}
	}
	// Rotate about the shaft-axis EDGE's world endpoints, which for this body
	// is the z axis through the apex.
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(angle))
	if err != nil {
		t.Fatalf("mesh rotation: %v", err)
	}
	moved, err := body.Placed(turn)
	if err != nil {
		t.Fatalf("mesh rotation move: %v", err)
	}
	decadtest.Agree(t, "the mesh rotation leaves the body's volume alone",
		volumeReading(t, moved, "gear body after the mesh rotation"), before,
		decadtest.WithinRel(units.Scalar(1e-9)))
	return []*decad.Body{moved}
}

// meshPhase is the extra rotation this gear receives. The DRIVING gear turns
// by half a tooth pitch, 180 / Driving Gear Teeth Number, so a driving valley
// sits where the pinion tooth crosses the axial plane. The pinion's phase is
// _PINION_MESH_PHASE_TEETH tooth-fractions, default 0, because the spiral
// build leaves the mid-face section unrotated and it already meshes.
func meshPhase(sd side) float64 {
	if sd.which == driving {
		return math.Pi / sd.teeth
	}
	return 0
}

func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newGeometry(t, p)
	sd := g.side(gearOf(p))
	angle := meshPhase(sd)

	if sd.which == driving {
		requireClose(t, "driving mesh phase", angle, math.Pi/sd.teeth, 1e-15)
		requireClose(t, "driving mesh phase as a fraction of a tooth pitch",
			angle/(2*math.Pi/sd.teeth), 0.5, 1e-15)
	} else {
		requireClose(t, "pinion mesh phase", angle, 0, 1e-15)
	}

	// The rotation happens in the Design component BEFORE the body is moved
	// out, because a construction axis cannot be added in the moved-out gear
	// component, so it uses the profile edge's world geometry while still in
	// Design.
	hex := sd.hexProfile(g)
	box := boundsReading(t, bodies[0], "gear body")
	// A rotation about the shaft axis leaves the body's extent ALONG that axis
	// alone, which is the reading that says the move was a turn and not a
	// shift.
	agreesWithin(t, "the rotated body's extent along the shaft axis",
		reading{value: box.Max.Z - box.Min.Z, bound: 2 * box.Bound.Base()},
		hex[1].X-hex[3].X, 1e-9*(hex[1].X-hex[3].X))
	// Across the axis it stays between the sweep's inradius and its
	// circumradius. The n-gon's own axis-aligned extent turns with it, so the
	// band is what a rotation preserves and a fixed box is not: asserting the
	// box would be asserting the angle twice, once through a reading that
	// cannot see it.
	outer := math.Max(hex[3].Y, hex[2].Y)
	inner := outer * math.Cos(math.Pi/float64(bandSides))
	for _, v := range []struct {
		name string
		d    float64
	}{
		{"max X", box.Max.X}, {"max Y", box.Max.Y}, {"-min X", -box.Min.X}, {"-min Y", -box.Min.Y},
	} {
		if v.d < inner-1e-9 || v.d > outer+1e-9 {
			t.Errorf("the rotated body's %s reaches %.6f mm, outside the sweep's own band "+
				"[%.6f, %.6f]", v.name, v.d, inner, outer)
		}
	}
	_ = doc
}
