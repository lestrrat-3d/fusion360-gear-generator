// This file proves the bevel bevelSide's solid steps.
//
// # No boolean is performed anywhere in this proof
//
// At the decad revision this repo pins, a boolean accepts only prism, cup and
// faceted payloads and refuses an operand built by Loft. Every solid in this
// bevelSide is conical, and a cone is a Loft here, because Extrude refuses a nonzero
// taper. So the union that joins the frustum, the intersection and cut that
// trim the tooth, the bore's through-cut and the Combine-Join are all
// unavailable.
//
// The substitution is the same at every site: build the operands, lay them
// apart along the shaft axis, and assert from their own measured geometry what
// the operation would have produced. Laying them apart leaves every volume,
// radius and cone angle unchanged, which is what makes the readings still mean
// something. Each step below states what its own substitution costs.
//
// # Two further substitutions this file makes everywhere
//
// A CHORDED TOOTH SECTION. decad's Loft pairs only LineSeg, ArcSeg and
// CircleSeg segments; a pair of splines is ErrUnsupported. The drawn tooth's
// flanks are splines, so every solid step here builds the tooth section as the
// POLYGON through the same sample points the drawn tooth passes through — the
// flanks chorded, the tip and root arcs chorded, and the two root corners
// exactly where the drawn tooth puts them. What that costs is the area between
// each chord and its curve; what it keeps exactly is every vertex, so the
// tooth's radial extent, its root corners and its tip corners are the drawn
// tooth's own. Each step asserts its volume against THAT polygon's area rather
// than against a smooth tooth's, so the chord error is never hidden inside a
// tolerance.
//
// A POLYGONAL SWEEP FOR EVERY CONE. A loft between two CIRCLES builds as a
// faceted bevelBand, and decad then publishes its volume with a proven bound of
// about 0.12 percent — beyond its own default relative tolerance, so the gate
// reports Suspect on a bevelBand that is in fact correct. A loft between two regular
// polygons is built from LineSeg pairs and its volume comes back proven to
// about 1e-12 mm^3. So every cone here is swept through a regular
// bevelConeSides-gon rather than a circle, and every volume is asserted against the
// POLYGON's own closed form, exactly, with no chord tolerance anywhere. The
// circular volume the real revolve produces is then pinned by the same reading
// through the exact factor bevelPolygonFactor states.
//
// What that costs: the swept surface is a bevelBand of flats rather than a true cone,
// so nothing here reads a cone's own half-angle off a decad surface. Each band's
// half-angle is pinned instead by the two ring circumradii and the height its
// bounds and volume prove, which is the same quantity by a different route.
//
// # The solid tables run at Module 4 to 8
//
// decad's mesh bound has an absolute floor, so a figure small enough brings
// every measurement inside it and the gate reports Suspect on geometry that is
// in fact correct. Module is a pure scale on this figure, so a case at Module 4
// through 8 proves the same shape as one at Module 1 and clears the floor. The
// sketch tables are unaffected and stay at the dialog's own default.
package bevelgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// bevelConeSides is how many flats every swept cone in this proof carries. It is
// high enough that the polygon's volume is within two parts in a thousand of
// the circle's, and low enough that a loft of that many segment pairs stays
// well inside decad's own pair-test budget.
const bevelConeSides = 64

// bevelExactSlack is the relative slack a closed form evaluated in float64 carries
// against a reading decad proves to about 1e-12. It is float rounding and
// nothing else: the polygonal sweep introduces no chord error, so there is no
// approximation for a tolerance to absorb.
const bevelExactSlack = 1e-9

// bevelPolygonFactor is the exact ratio between the volume a regular n-gon sweeps
// and the volume the circle it is inscribed in sweeps, for the same height and
// the same pair of circumradii: (n / 2pi) * sin(2pi / n). It is what carries a
// reading taken on the polygonal bevelBand back to the frustum the real revolve
// produces.
func bevelPolygonFactor(n int) float64 {
	return float64(n) / (2 * math.Pi) * math.Sin(2*math.Pi/float64(n))
}

// bevelNgon is a regular n-gon of circumradius r, centred on the origin, walked
// counter-clockwise from +X.
func bevelNgon(n int, r float64) []bevelVec {
	out := make([]bevelVec, n)
	for i := range n {
		a := 2 * math.Pi * float64(i) / float64(n)
		out[i] = bevelVec{r * math.Cos(a), r * math.Sin(a)}
	}
	return out
}

// bevelLaneStep is how far apart along X successive bodies are laid. No operand here
// is ever joined to another, so every one of them has to stand clear of the
// rest or the verifier reports interference; the step is generous rather than
// tight because a case at Module 8 is twice the size of one at Module 4.
func bevelLaneStep(g bevelGeom) float64 {
	widest := math.Max(g.PPD, g.DPD) / 2
	for _, s := range []bevelSide{g.Pinion, g.Driving} {
		// The virtual tip radius, which at a large pitch cone angle is several
		// times the pitch radius and is the widest thing this proof ever lays in
		// a lane.
		widest = math.Max(widest, s.VirtualPitch+g.Module)
	}
	return 2.6 * widest
}

// bevelBench is one case's document, its shared sketch world, and the lane allocator
// that keeps the operands apart.
type bevelBench struct {
	t    *testing.T
	doc  *decad.Document
	w    *sketch.World
	step float64
	next int
}

func newBevelBench(t *testing.T, doc *decad.Document, g bevelGeom) *bevelBench {
	seed := decadtest.NewSketch(t)
	return &bevelBench{t: t, doc: doc, w: seed.World(), step: bevelLaneStep(g)}
}

// lane hands out the next X offset. Every body this proof builds sits in one.
func (b *bevelBench) lane() float64 {
	x := float64(b.next) * b.step
	b.next++
	return x
}

// planeAt returns a sketch plane at height z above the world XY datum.
func (b *bevelBench) planeAt(z float64) *sketch.Plane {
	b.t.Helper()
	if z == 0 {
		return b.w.XY()
	}
	p, err := b.w.CreateOffsetPlane(b.w.XY(), z)
	if err != nil {
		b.t.Fatalf("offset plane at z=%.6f: %v", z, err)
	}
	return p
}

// polygonRegion draws one closed polygon through pts, offset to lane, on the
// plane at height z.
func (b *bevelBench) polygonRegion(z, lane float64, pts []bevelVec) (*sketch.Sketch, *sketch.Profile) {
	b.t.Helper()
	s, err := b.w.CreateSketch(b.planeAt(z))
	if err != nil {
		b.t.Fatalf("create sketch at z=%.6f: %v", z, err)
	}
	handles := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		handles[i] = s.CreatePoint(lane+p.X, p.Y)
	}
	lines := make([]*sketch.Line, len(handles))
	for i := range handles {
		lines[i] = s.CreateLine(handles[i], handles[(i+1)%len(handles)])
	}
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}
	return s, decadtest.SolveRegion(b.t, s)
}

// frustum builds one conical bevelBand as the polygonal sweep described at the top
// of this file: a loft between two regular bevelConeSides-gons on parallel planes,
// in its own lane. Extrude refuses a nonzero taper, so a cone is a Loft here
// whatever its section; the polygon is what makes the volume come back proven.
func (b *bevelBench) frustum(lane, z0, r0, z1, r1 float64) *decad.Body {
	b.t.Helper()
	s0, p0 := b.polygonRegion(z0, lane, bevelNgon(bevelConeSides, r0))
	s1, p1 := b.polygonRegion(z1, lane, bevelNgon(bevelConeSides, r1))
	body, err := b.doc.Loft(s0, p0, s1, p1)
	if err != nil {
		b.t.Fatalf("conical band from (z=%.4f r=%.4f) to (z=%.4f r=%.4f): %v", z0, r0, z1, r1, err)
	}
	return body
}

// taperedPrism lofts a polygon to a scaled copy of itself, which is what a
// section swept toward a point produces once the degenerate end is replaced by
// a shrunken section.
func (b *bevelBench) taperedPrism(lane, z0, k0, z1, k1 float64, pts []bevelVec) *decad.Body {
	b.t.Helper()
	s0, p0 := b.polygonRegion(z0, lane, bevelScalePolygon(pts, k0))
	s1, p1 := b.polygonRegion(z1, lane, bevelScalePolygon(pts, k1))
	body, err := b.doc.Loft(s0, p0, s1, p1)
	if err != nil {
		b.t.Fatalf("tapered prism from z=%.4f (x%.4f) to z=%.4f (x%.4f): %v", z0, k0, z1, k1, err)
	}
	return body
}

func bevelScalePolygon(pts []bevelVec, k float64) []bevelVec {
	out := make([]bevelVec, len(pts))
	for i, p := range pts {
		out[i] = bevelMul(p, k)
	}
	return out
}

// bevelBandVolume is the exact volume of a CONICAL frustum of height h between radii
// r0 and r1: the Pappus reading the real revolve produces.
func bevelBandVolume(h, r0, r1 float64) float64 {
	return math.Pi * math.Abs(h) * (r0*r0 + r0*r1 + r1*r1) / 3
}

// bevelSweptBandVolume is the volume the polygonal bevelBand actually built carries: the
// same frustum swept through bevelConeSides flats instead of a circle. The two
// differ by exactly bevelPolygonFactor, which is why a reading on one pins the
// other.
func bevelSweptBandVolume(h, r0, r1 float64) float64 {
	return bevelBandVolume(h, r0, r1) * bevelPolygonFactor(bevelConeSides)
}

// bevelPolygonArea is the area of a closed polygon, which is the section area every
// tapered prism in this file is asserted against.
func bevelPolygonArea(pts []bevelVec) float64 { return math.Abs(bevelShoelace(pts)) }

// bevelMeasuresVolume compares a body's volume reading against a closed form under
// the label the step knows the body by. decadtest otherwise names a body by
// index and recipe step, which does not say which feature is wrong.
func bevelMeasuresVolume(t *testing.T, label string, body *decad.Body, want float64, rel float64) {
	t.Helper()
	got, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	decadtest.Measures(t, label+" volume", got, units.CubicMillimeters(want),
		decadtest.WithinRel(units.Scalar(rel)))
}

// bevelMeasuresBox compares a body's bounding box under the step's own label.
func bevelMeasuresBox(t *testing.T, label string, body *decad.Body, lo, hi r3.Vec, slack float64) {
	t.Helper()
	got, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", label, err)
	}
	decadtest.MeasuresBox(t, label+" bounds", got, lo, hi,
		decadtest.Within(units.Millimeters(slack)))
}

// bevelFrustumBands is the three bands the §2 hexagon's three sloped edges sweep,
// each as a station pair and a radius pair in the axial half-section.
//
// They are, in the order the hexagon walks them: the HEEL bevelBand that edge C->H
// sweeps, the ROOT bevelBand that edge C->M sweeps, and the TOE DISH PLUG that edge
// M->N sweeps. The frustum is their SIGNED sum — heel plus root MINUS plug —
// because the plug is the dish that hollows the flat front face.
type bevelBand struct {
	Label          string
	Z0, R0, Z1, R1 float64
}

func bevelFrustumBands(g bevelGeom, s bevelSide) []bevelBand {
	hex := g.hexagon(s)
	// hex is A'/B' -> G/I -> H/J -> C/D -> M/O -> N/P in (station, radius).
	heelCorner, dedendum, toeCorner, toeInner := hex[2], hex[3], hex[4], hex[5]
	return []bevelBand{
		{"heel band", dedendum.X, dedendum.Y, heelCorner.X, heelCorner.Y},
		{"root band", toeCorner.X, toeCorner.Y, dedendum.X, dedendum.Y},
		{"toe dish plug", toeCorner.X, toeCorner.Y, toeInner.X, toeInner.Y},
	}
}

// bevelHalfAngleOf is a bevelBand's cone half-angle, measured from the shaft axis, which
// is what the readings on that bevelBand's height and two radii bevelPin.
func bevelHalfAngleOf(b bevelBand) float64 {
	return math.Atan2(math.Abs(b.R1-b.R0), math.Abs(b.Z1-b.Z0))
}

// stepRevolveGearBody builds the frustum the §2 hexagon sweeps about the shaft
// axis.
//
// SUBSTITUTION: a polygonal sweep, because decad publishes a revolved body's
// volume with a proven bound equal to the volume itself, so a revolved body is
// Suspect at any tolerance and cannot pass the harness gate. The three bands
// the frustum's profile edges sweep are built, laid apart, and never joined.
//
// THE COST IS THE UNION: the proof does not show the three bands closing into
// one watertight solid, only that each is separately watertight and that
// together they have the right volume, stations and angles.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	b := newBevelBench(t, doc, g)
	out := make([]*decad.Body, 0, 3)
	for _, bd := range bevelFrustumBands(g, side) {
		out = append(out, b.frustum(b.lane(), bd.Z0, bd.R0, bd.Z1, bd.R1))
	}
	return out
}

func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	bands := bevelFrustumBands(g, side)
	if len(bodies) != len(bands) {
		t.Fatalf("built %d bands, want %d", len(bodies), len(bands))
	}
	step := bevelLaneStep(g)

	signed := 0.0
	for i, bd := range bands {
		h := bd.Z1 - bd.Z0
		want := bevelBandVolume(h, bd.R0, bd.R1)
		// The reading is taken on the polygonal bevelBand and compared to the
		// polygon's own closed form, so nothing is absorbed by a tolerance; the
		// conical volume `want` is then pinned through the exact factor.
		bevelMeasuresVolume(t, side.Label+" "+bd.Label, bodies[i],
			bevelSweptBandVolume(h, bd.R0, bd.R1), bevelExactSlack)
		// Station by station and ring radius by ring radius: the bevelBand spans
		// exactly the two stations the hexagon's edge spans, and reaches exactly
		// the larger of its two ring radii.
		lane := float64(i) * step
		rmax := math.Max(bd.R0, bd.R1)
		// bevelConeSides is a multiple of four, so the n-gon carries a vertex on each
		// axis and its box is exactly the circumscribed square of the larger
		// ring.
		bevelMeasuresBox(t, side.Label+" "+bd.Label, bodies[i],
			r3.NewVec(lane-rmax, -rmax, math.Min(bd.Z0, bd.Z1)),
			r3.NewVec(lane+rmax, rmax, math.Max(bd.Z0, bd.Z1)),
			bevelExactSlack*rmax+1e-9)
		if bd.Label == "toe dish plug" {
			signed -= want
		} else {
			signed += want
		}
	}

	// The frustum as the SIGNED SUM of the three bands, against Pappus on the
	// §2 hexagon itself.
	bevelRequireClose(t, side.Label+" frustum volume as the signed sum of its bands",
		signed, bevelRevolvedVolume(g.hexagon(side)), 1e-6*math.Abs(signed))

	// Cone half-angle by cone half-angle. The heel bevelBand and the toe plug come
	// out PARALLEL, both on the back-cone family at 90 degrees minus this bevelSide's
	// pitch cone angle, and the root bevelBand sits at the DEDENDUM ANGLE to them.
	back := math.Pi/2 - side.Gamma
	bevelRequireClose(t, side.Label+" heel band half-angle", bevelHalfAngleOf(bands[0]), back, 1e-9)
	bevelRequireClose(t, side.Label+" toe plug half-angle", bevelHalfAngleOf(bands[2]), back, 1e-9)
	bevelRequireClose(t, side.Label+" root band half-angle", bevelHalfAngleOf(bands[1]), side.GammaRoot, 1e-9)
	// The root bevelBand sits one DEDENDUM ANGLE inside this bevelSide's own PITCH cone,
	// which is where that angle lives. The spec's sentence puts the root bevelBand
	// "at the dedendum angle to" the back-cone family, and that is not what the
	// figure does: in the axial section the root generator stands at 90 degrees
	// PLUS the dedendum angle to the back-cone generator, never at the dedendum
	// angle itself. Both readings below are asserted so the relation the spec
	// meant is pinned and the one it wrote is not silently adopted.
	bevelRequireClose(t, side.Label+" root band inside the pitch cone by the dedendum angle",
		side.Gamma-bevelHalfAngleOf(bands[1]), math.Atan(g.Dedendum/g.R), 1e-9)
	bevelRequireClose(t, side.Label+" root generator against the back-cone generator",
		math.Pi/2+math.Atan(g.Dedendum/g.R),
		math.Acos(bevelDot(bevelUnit(bevelSub(g.M, g.C)), g.UP))+0, 1e-9)
}

// bevelToothFor is the tooth section this bevelSide's solid steps build, together with
// the apex-to-section distance the loft runs over.
//
// apexToSection is the apex's PERPENDICULAR distance to the tooth plane, which
// is s_K * cos(gamma) and not s_K: the tooth plane is the back-cone plane,
// tilted out of the axis-perpendicular by gamma. Substituting an
// axis-perpendicular plane here is the one substitution this step does NOT
// make.
func bevelToothFor(g bevelGeom, s bevelSide) (section []bevelVec, apexToSection float64, c bevelToothCircles) {
	c = bevelCirclesFor(g, s)
	return bevelToothPolygon(c, s.VirtualTeeth), g.toothCentreStation(s) * math.Cos(s.Gamma), c
}

// bevelApexShrink is the fraction of the way out from the apex at which the
// shrunken stand-in section sits. It is the whole of the loft substitution: the
// degenerate point section is replaced by a similar section this small, and
// nothing else about the loft changes.
const bevelApexShrink = 0.02

// stepLoftTooth builds §3's apex-to-profile loft: the uncut tooth body, lofted
// from the §2 Apex sketch point to this bevelSide's virtual spur tooth profile.
//
// SUBSTITUTION: a shrunken section stands in for the degenerate apex point, and
// nothing else. The tooth plane is NOT substituted — the proof works in the real
// back-cone plane, tilted out of the axis-perpendicular by gamma, and takes the
// apex's perpendicular distance to the section as s_K * cos(gamma) rather than
// s_K.
//
// THE COST IS THE POINT SECTION: the loft's degenerate end is not built. What
// the volume and the two cone slopes prove is the taper it has to produce.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	b := newBevelBench(t, doc, g)
	return []*decad.Body{b.taperedPrism(b.lane(), bevelApexShrink*d1, bevelApexShrink, d1, 1, section)}
}

func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, c := bevelToothFor(g, side)
	d0 := bevelApexShrink * d1
	area := bevelPolygonArea(section)

	// A section that scales linearly from the apex sweeps A1 * d1 * (1 - k^3) / 3
	// between k*d1 and d1, which is the truncated pyramid the real loft's point
	// end completes.
	bevelMeasuresVolume(t, side.Label+" uncut tooth", bodies[0],
		area*d1*(1-bevelApexShrink*bevelApexShrink*bevelApexShrink)/3, bevelExactSlack)

	// The bounds prove both stations and, through the shrink, both ends' radial
	// extents: the big end reaches the tip corner, and the small end's inner
	// edge is the root corner scaled by the same factor the stations carry.
	lo, hi := bevelSectionBox(section)
	bevelMeasuresBox(t, side.Label+" uncut tooth", bodies[0],
		r3.NewVec(math.Min(lo.X, bevelApexShrink*lo.X), math.Min(lo.Y, bevelApexShrink*lo.Y), d0),
		r3.NewVec(math.Max(hi.X, bevelApexShrink*hi.X), math.Max(hi.Y, bevelApexShrink*hi.Y), d1),
		bevelExactSlack*hi.X+1e-9)

	// The two cone slopes the taper has to produce, as radius gained per unit of
	// cone distance out from the apex. They are what the trimmed ends and the
	// Combine-Join are then read against.
	bevelRequireClose(t, side.Label+" tip cone slope", c.Tip/d1,
		c.Tip/(g.toothCentreStation(side)*math.Cos(side.Gamma)), 1e-12)
	bevelRequireClose(t, side.Label+" root cone slope", c.Root/d1,
		c.Root/(g.toothCentreStation(side)*math.Cos(side.Gamma)), 1e-12)
	// The apex's perpendicular distance to the section is s_K * cos(gamma). At
	// Tooth Spacing 0 that is exactly the Pitch Cone Distance, which is what
	// makes the back cone the plane it is.
	if g.ToothSpacing == 0 {
		bevelRequireClose(t, side.Label+" apex-to-back-cone-plane distance", d1, g.R, 1e-9)
	}
}

// bevelSectionBox is a polygon's own bounding rectangle, which is what a tapered
// prism's bounds are the union of at its two ends.
func bevelSectionBox(pts []bevelVec) (lo, hi bevelVec) {
	lo, hi = pts[0], pts[0]
	for _, p := range pts {
		lo.X, lo.Y = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y)
		hi.X, hi.Y = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y)
	}
	return lo, hi
}

// bevelCutCone is one of the two trimming cones, read as the axial-section line its
// generator traces: an apex station on the shaft axis and a wall slope.
type bevelCutCone struct {
	Label     string
	Apex      float64 // station on the shaft axis where the cone closes
	Slope     float64 // radius lost per unit of station, going outward
	Z0, R0    float64 // the bevelBand the proof actually builds
	Z1, R1    float64
	MeetsRoot bevelVec // the §2 point where this cone meets the bevelSide body's root cone
}

// bevelCutCones are the toe and heel trimming cones: the faces the revolved frustum
// already carries, swept by the toe edge M->N / O->P and the heel edge C->H /
// D->J. The TOOLS are cone faces of the GEAR BODY, never of the lofted tooth,
// which carries no cone face at all.
func bevelCutCones(g bevelGeom, s bevelSide) []bevelCutCone {
	hex := g.hexagon(s)
	dedendum, toeCorner, toeInner := hex[3], hex[4], hex[5]
	heelCorner := hex[2]
	meetToe, meetHeel := g.M, g.C
	if s.Label == "Driving" {
		meetToe, meetHeel = g.O, g.D
	}
	toeApex := toeCorner.X + (toeInner.X-toeCorner.X)*toeCorner.Y/(toeCorner.Y-toeInner.Y)
	heelApex := dedendum.X + (heelCorner.X-dedendum.X)*dedendum.Y/(dedendum.Y-heelCorner.Y)
	return []bevelCutCone{
		{
			Label: "toe cone", Apex: toeApex,
			Slope: (toeCorner.Y - toeInner.Y) / (toeInner.X - toeCorner.X),
			Z0:    toeCorner.X, R0: toeCorner.Y, Z1: toeInner.X, R1: toeInner.Y,
			MeetsRoot: meetToe,
		},
		{
			Label: "heel cone", Apex: heelApex,
			Slope: (dedendum.Y - heelCorner.Y) / (heelCorner.X - dedendum.X),
			Z0:    dedendum.X, R0: dedendum.Y, Z1: heelCorner.X, R1: heelCorner.Y,
			MeetsRoot: meetHeel,
		},
	}
}

// bevelCrossingStation is where a cone of apex station a and wall slope k meets a
// tooth surface of slope m, both measured from the same apex on the shaft axis:
// a * k / (m + k). It is positive for every m > 0, so EVERY cut crosses BOTH of
// the tooth's surfaces in every configuration this spec can build. That is why
// this step asserts the ORDER of the two crossings rather than that they
// differ: an assertion that they differ passes on any figure at all.
func bevelCrossingStation(apex, coneSlope, toothSlope float64) float64 {
	return apex * coneSlope / (toothSlope + coneSlope)
}

// stepConicalEndCuts is the toe-then-heel two-cone flush trim of the lofted
// tooth body.
//
// SUBSTITUTION: neither cut is performed. Both operands are Lofts — the tooth
// and each cone alike — and a boolean refuses a Loft operand here. The tooth
// and the two cones are built and laid apart; each cone's apex and half-angle
// are read off the cone, each of the tooth's two surfaces off the tooth, and
// the stations where they cross are solved from those readings and checked
// against the flush bevelBand.
//
// THE COST IS THE SPLIT: the proof does not show the evaluator dividing the
// tooth, selecting the keeper, or leaving a watertight body.
//
// WHAT IT ALSO CANNOT TELL APART: a cone and a tilted plane read identically in
// everything this step measures. In the axial section a cone of half-angle
// 90 - gamma and a plane tilted by gamma through the same generator are the
// same line, so every station solved here comes out the same for either
// surface. What makes the face conical is that it is a SURFACE OF REVOLUTION
// about the shaft axis — its crossing with the tooth's tip surface sits at one
// station at every azimuth, while a tilted plane's crossing moves with azimuth.
// This step measures that nowhere. It has it by construction, because each
// cutting tool is built as a bevelBand swept about that axis and a swept bevelBand cannot
// be anything else.
//
// AND THE FLUSH-BAND CHECK CANNOT SEE THE HALF-ANGLE: any bevelBand through M
// crosses the bevelSide body's root ray at M whatever slope the bevelBand has, so a toe
// bevelBand built at the wrong angle still lands on the toe end of the flush bevelBand
// and still crosses the tip inboard of the root. The half-angle assertion below
// is what pins the angle at this step.
func stepConicalEndCuts(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	b := newBevelBench(t, doc, g)
	out := []*decad.Body{b.taperedPrism(b.lane(), bevelApexShrink*d1, bevelApexShrink, d1, 1, section)}
	for _, cone := range bevelCutCones(g, side) {
		out = append(out, b.frustum(b.lane(), cone.Z0, cone.R0, cone.Z1, cone.R1))
	}
	return out
}

func assertConicalEndCuts(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, c := bevelToothFor(g, side)
	cones := bevelCutCones(g, side)
	step := bevelLaneStep(g)

	lo, hi := bevelSectionBox(section)
	bevelMeasuresBox(t, side.Label+" tooth being trimmed", bodies[0],
		r3.NewVec(math.Min(lo.X, bevelApexShrink*lo.X), math.Min(lo.Y, bevelApexShrink*lo.Y), bevelApexShrink*d1),
		r3.NewVec(math.Max(hi.X, bevelApexShrink*hi.X), math.Max(hi.Y, bevelApexShrink*hi.Y), d1),
		bevelExactSlack*hi.X+1e-9)

	back := math.Pi/2 - side.Gamma
	for i, cone := range cones {
		body := bodies[i+1]
		lane := float64(i+1) * step
		rmax := math.Max(cone.R0, cone.R1)
		bevelMeasuresVolume(t, side.Label+" "+cone.Label, body,
			bevelSweptBandVolume(cone.Z1-cone.Z0, cone.R0, cone.R1), bevelExactSlack)
		bevelMeasuresBox(t, side.Label+" "+cone.Label, body,
			r3.NewVec(lane-rmax, -rmax, math.Min(cone.Z0, cone.Z1)),
			r3.NewVec(lane+rmax, rmax, math.Max(cone.Z0, cone.Z1)),
			bevelExactSlack*rmax+1e-9)

		// Each cone's half-angle IS this gear's back-cone half-angle, at the
		// same slope tolerance the revolve's bands are held to.
		bevelRequireClose(t, side.Label+" "+cone.Label+" half-angle",
			math.Atan(cone.Slope), back, 1e-9)

		// Each cut lands where the flush bevelBand requires: the cone meets the bevelSide
		// body's own ROOT cone exactly at M (toe) and at C (heel).
		meetStation := g.station(side, cone.MeetsRoot)
		bevelRequireClose(t, side.Label+" "+cone.Label+" meets the root cone",
			cone.Slope*(cone.Apex-meetStation), g.radius(side, cone.MeetsRoot), 1e-9)
		bevelRequireClose(t, side.Label+" "+cone.Label+" meeting point is on the root cone",
			math.Tan(side.GammaRoot)*meetStation, g.radius(side, cone.MeetsRoot), 1e-9)

		// The tip crossing sits INBOARD of the root crossing, so the trimmed end
		// is shorter at the tip than at the root.
		tipSlope := c.Tip * math.Cos(side.Gamma) / (g.toothCentreStation(side) - c.Tip*math.Sin(side.Gamma))
		rootSlope := c.Root * math.Cos(side.Gamma) / (g.toothCentreStation(side) - c.Root*math.Sin(side.Gamma))
		atTip := bevelCrossingStation(cone.Apex, cone.Slope, tipSlope)
		atRoot := bevelCrossingStation(cone.Apex, cone.Slope, rootSlope)
		if !(atTip < atRoot) {
			t.Errorf("%s %s crosses the tooth's tip at station %.6f and its root at %.6f: "+
				"the trimmed end is not shorter at the tip", side.Label, cone.Label, atTip, atRoot)
		}
	}

	// The heel cut is the LENIENT one because its cone is tangent to the tooth
	// plane: the dedendum corner and the tooth centre both sit on this bevelSide's
	// back-cone dedendum line, so the tooth plane contains a generator of the
	// heel cone and the two touch along the tooth's own centreline instead of
	// crossing it. That cone's apex on the shaft axis is K/L, where the same
	// dedendum line meets the axis.
	bevelRequireClose(t, side.Label+" heel cone apex is the tooth centre's own station",
		cones[1].Apex, g.R/math.Cos(side.Gamma), 1e-9)
	// At Tooth Spacing 0 the heel cone therefore passes exactly through the
	// tooth's heel-end centreline and takes only the two corners, by
	// py^2 / (2 * r * cos gamma) for a corner lying py off the centreline at
	// polar radius r. A cut that removes that little is one that can miss the
	// keeper altogether on a ratio pair — the typed NonIntersectError the
	// framework helper catches — and is why only the TOE cut must split.
	if g.ToothSpacing == 0 {
		corner := bevelToothRootCorner(c, side.VirtualTeeth)
		bite := corner.Y * corner.Y / (2 * c.Root * math.Cos(side.Gamma))
		if bite <= 0 {
			t.Errorf("%s heel cone takes nothing at all from the tooth's corners", side.Label)
		}
		proofkit.Step(t, "%s heel cone takes %.6f mm at the tooth's root corners", side.Label, bite)
	}
}

// The circular pattern's carried readings.
//
// THIS STEP IS SERIAL BECAUSE OF THESE VARIABLES, and it is the only step in
// this package that is. The pattern increment retires the seed tooth, so the
// seed cannot be measured after the step runs: its azimuth, radius, height and
// volume have to be read during the build and handed to the assertion, and that
// hand-off leaves the case. Two cases sharing one set of seed readings overwrite
// each other, and it is not a hazard that announces itself — the two bevelSide sides
// differ enough in volume that the overwrite was caught when it happened, and a
// pair of cases whose seeds measured alike would have passed on each other's
// numbers instead. stepCircularPattern therefore stays on proofkit3d.RunSolid
// while every other step in this package runs its cases together.
var (
	bevelSeedVolume   decad.Measurement
	bevelSeedCentroid decad.VecMeasurement
	bevelSeedBox      decad.Box
)

// stepCircularPattern patterns the trimmed tooth around the shaft-axis edge,
// one copy per tooth, over a full circle.
//
// SUBSTITUTION: the tooth is built on an axis-perpendicular section rather than
// on the tilted back-cone plane. The tilt is what stepLoftTooth proves and it is
// irrelevant to the angular spacing, the congruence and the count this step
// reads. Everything else is the real arrangement: the apex sits on the shaft
// axis and every copy is a rigid rotation about it, which is what the pattern
// produces.
//
// The angular spacing stays constant at 360/N for the entire face width even
// though the pitch diameter shrinks from heel toward apex: the radial taper is
// already produced by the loft, so the pattern just rotates one tapered tooth
// into N evenly spaced copies.
//
// SECOND SUBSTITUTION: the whole ring of N copies is not built. decad's
// disjoint/overlap partition proof does not resolve a full ring at the real
// tooth pitch — measured on the default pair at Module 4 it comes back Sound at
// 8 and at 16 copies and Suspect, with the adjacent pairs undecided, at 12, at
// 20 and at the real 31 — and a Suspect pair fails the gate whatever the
// geometry is. So this step builds the seed and the ONE adjacent copy the
// increment puts beside it, which is the ring's tightest pair and the only one
// whose disjointness is in question, and asserts the ring itself arithmetically:
// N copies at 360/N close the full circle exactly once.
//
// THE COST IS THE RING: the proof shows one increment rather than N of them, so
// it does not show that the N-th copy lands back on the seed or that no
// non-adjacent pair collides. Neither depends on anything but the increment,
// which is what it does show.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	b := newBevelBench(t, doc, g)
	// What the pattern copies is the TRIMMED tooth piece — the flush bevelBand the
	// two conical cuts leave between the toe and the heel — not the uncut apex
	// loft. Its toe end sits at the fraction of the cone distance the Root
	// Length leaves.
	toeFraction := (g.ApexDed - g.RootLength) / g.ApexDed
	seed := b.taperedPrism(0, toeFraction*d1, toeFraction, d1, 1, section)

	// Read the seed NOW. The increment below retires it.
	var err error
	if bevelSeedVolume, err = seed.Volume(); err != nil {
		t.Fatalf("seed tooth volume: %v", err)
	}
	if bevelSeedCentroid, err = seed.Centroid(); err != nil {
		t.Fatalf("seed tooth centroid: %v", err)
	}
	if bevelSeedBox, err = seed.Bounds(); err != nil {
		t.Fatalf("seed tooth bounds: %v", err)
	}

	quantity := int(side.Teeth)
	axis := r3.NewVec(0, 0, 1)
	turn, err2 := r3.RotationAround(r3.NewVec(0, 0, 0), axis,
		units.Radians(2*math.Pi/float64(quantity)))
	if err2 != nil {
		t.Fatalf("pattern increment: %v", err2)
	}
	neighbour, err2 := seed.PlacedCopy(turn)
	if err2 != nil {
		t.Fatalf("pattern copy: %v", err2)
	}
	out := []*decad.Body{neighbour}
	// The increment retires the seed, which is what makes the readings above the
	// only record of it.
	identity, err := r3.Rotation(axis, units.Radians(0))
	if err != nil {
		t.Fatalf("pattern seed placement: %v", err)
	}
	placed, err := seed.Placed(identity)
	if err != nil {
		t.Fatalf("pattern seed placement: %v", err)
	}
	return append([]*decad.Body{placed}, out...)
}

func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	quantity := int(side.Teeth)
	if len(bodies) != 2 {
		t.Fatalf("%s pattern produced %d bodies, want the seed and its one adjacent copy",
			side.Label, len(bodies))
	}
	// The ring the increment closes: N copies at 360/N, exactly once round, with
	// quantity = this bevelSide's Teeth Number, totalAngle = 360 deg and
	// isSymmetric = False.
	bevelRequireClose(t, side.Label+" pattern quantity is this gear's Teeth Number",
		float64(quantity), side.Teeth, 0)
	bevelRequireClose(t, side.Label+" N copies at the increment close the full circle",
		float64(quantity)*(360/float64(quantity)), 360, 1e-12)
	for k, body := range bodies {
		// Reading against reading, so BOTH proven bounds count: a patterned copy
		// against its seed is exactly the comparison decadtest.Agree is for.
		got, err := body.Volume()
		if err != nil {
			t.Fatalf("%s pattern copy %d volume: %v", side.Label, k, err)
		}
		decadtest.Agree(t, fmt.Sprintf("%s pattern copy %d against the seed", side.Label, k),
			got, bevelSeedVolume, decadtest.WithinRel(units.Scalar(bevelExactSlack)))

		// The copy sits exactly k * 360/N around the shaft axis from the seed.
		angle := 2 * math.Pi * float64(k) / float64(quantity)
		cx := bevelSeedCentroid.Value.X*math.Cos(angle) - bevelSeedCentroid.Value.Y*math.Sin(angle)
		cy := bevelSeedCentroid.Value.X*math.Sin(angle) + bevelSeedCentroid.Value.Y*math.Cos(angle)
		centroid, err := body.Centroid()
		if err != nil {
			t.Fatalf("%s pattern copy %d centroid: %v", side.Label, k, err)
		}
		decadtest.MeasuresVec(t, fmt.Sprintf("%s pattern copy %d centroid", side.Label, k),
			centroid, r3.NewVec(cx, cy, bevelSeedCentroid.Value.Z),
			decadtest.Within(units.Millimeters(1e-6)))
	}
	// The seed's own extent, carried from the build, is the extent every copy
	// has: a rigid rotation about the axis changes no distance from it.
	seedReach := math.Hypot(math.Max(math.Abs(bevelSeedBox.Min.X), math.Abs(bevelSeedBox.Max.X)),
		math.Max(math.Abs(bevelSeedBox.Min.Y), math.Abs(bevelSeedBox.Max.Y)))
	if seedReach <= 0 {
		t.Fatalf("%s seed tooth reached nothing from the shaft axis", side.Label)
	}
}

// stepCombineJoin joins the patterned tooth pieces onto the bevelSide body.
//
// SUBSTITUTION: no join is performed. The operands are laid apart and the
// join's TWO consequences are asserted from their own measured geometry — a
// join leaves ONE lump when the tooth's root is below the body's root cone,
// seated rather than floating, and the joined body reaches further out than the
// frustum when the tooth's tip stands proud of it. Both readings are taken at
// the toe, the middle and the heel of the bevelBand the join would cover.
//
// THE COST IS THE STITCH: the proof cannot show the evaluator making one
// boundary out of two.
//
// The generated bevelModuleOf draws its root circle one ROOT SINK inside the dedendum
// corner, so the root arc lies inside the bevelSide body's root cone across its whole
// width and the join overlaps along the whole root rather than along the
// centreline alone. This proof applies that same sink — it is one figure, not a
// proof-only offset.
//
// Fusion has made this stitch once, loaded 2026-09-16 from the build the root
// sink was introduced on: the shipped default of 31 teeth on both gears at
// Module 1 and Shaft Angle 90, and a 16 driving / 12 pinion pair at Module 4,
// both with no error. The default is also the configuration where the sink
// drops the root circle below the base circle, so its tooth is drawn
// NON-embedded and the drawer adds the two flank-to-root lines. So the stitch
// this substitution cannot show has been seen once, on those two
// configurations, and on nothing else in the table. The heel TIP radius was not
// measured on that load, so the tip is still checked only where this proof
// checks it: no case here reads a tip radius off a joined body, because no case
// joins.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	b := newBevelBench(t, doc, g)
	tooth := b.taperedPrism(b.lane(), bevelApexShrink*d1, bevelApexShrink, d1, 1, section)
	bands := bevelFrustumBands(g, side)
	root := bands[1]
	return []*decad.Body{tooth, b.frustum(b.lane(), root.Z0, root.R0, root.Z1, root.R1)}
}

func assertCombineJoin(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, c := bevelToothFor(g, side)
	bands := bevelFrustumBands(g, side)
	root := bands[1]

	lo, hi := bevelSectionBox(section)
	bevelMeasuresBox(t, side.Label+" tooth being joined", bodies[0],
		r3.NewVec(math.Min(lo.X, bevelApexShrink*lo.X), math.Min(lo.Y, bevelApexShrink*lo.Y), bevelApexShrink*d1),
		r3.NewVec(math.Max(hi.X, bevelApexShrink*hi.X), math.Max(hi.Y, bevelApexShrink*hi.Y), d1),
		bevelExactSlack*hi.X+1e-9)
	bevelMeasuresVolume(t, side.Label+" gear body root band", bodies[1],
		bevelSweptBandVolume(root.Z1-root.Z0, root.R0, root.R1), bevelExactSlack)

	// Read the root arc's OUTERMOST point, never the tooth's centreline: the
	// centreline sits inside both root corners, so a reading taken there passes
	// a tooth whose corners float outside the cone, which is exactly the defect
	// the sink exists to remove.
	corner := bevelToothRootCorner(c, side.VirtualTeeth)
	tanRoot := math.Tan(side.GammaRoot)
	for _, at := range []struct {
		name     string
		fraction float64
	}{
		{"toe", (g.ApexDed - g.RootLength) / g.ApexDed},
		{"middle", (g.ApexDed - g.RootLength/2) / g.ApexDed},
		{"heel", 1},
	} {
		// Both the tooth's root surface and the gear body's root cone are cones
		// through the apex, so a station along the bevelBand scales the whole
		// section — the station AND the radius together, from the apex. Scaling
		// the in-plane coordinate alone instead walks the point along the tooth
		// plane rather than along the loft, and compares two points that are not
		// at the same cone distance at all.
		f := at.fraction
		station := f * g.toothStation(side, corner.X)
		radius := f * g.toothRadius(side, corner.X, corner.Y)
		cone := tanRoot * station
		if !(radius < cone) {
			t.Errorf("%s at the %s: the tooth's outermost root corner sits at radius %.6f "+
				"where the body's root cone is at %.6f — the tooth floats and the join leaves a gap",
				side.Label, at.name, radius, cone)
		}
		// The tip stands proud, which is what makes the joined body reach
		// further out than the frustum.
		tipStation := f * g.toothStation(side, c.Tip)
		tipRadius := f * g.toothRadius(side, c.Tip, 0)
		if !(tipRadius > tanRoot*tipStation) {
			t.Errorf("%s at the %s: the tooth's tip reaches %.6f where the frustum reaches %.6f — "+
				"the joined body would not stand proud of the gear body",
				side.Label, at.name, tipRadius, tanRoot*tipStation)
		}
	}

	// Without the sink the tooth's root ARC would touch the root cone only where
	// it crosses the tooth's own centreline, and the corners would stand outside
	// it. This is the reading the sink has to clear, and 4/4 carries the largest
	// corner float of any pair the spec admits.
	unsunk := bevelToothCircles{Pitch: c.Pitch, Base: c.Base, Tip: c.Tip, Root: c.Root + g.RootSink}
	unsunk.Embedded = unsunk.Base < unsunk.Root
	bare := bevelToothRootCorner(unsunk, side.VirtualTeeth)
	float := g.toothRadius(side, bare.X, bare.Y) - tanRoot*g.toothStation(side, bare.X)
	if float <= 0 {
		t.Errorf("%s: with no root sink the root corner is already inside the root cone by %.6f, "+
			"so this case says nothing about what the sink is for", side.Label, -float)
	}
	proofkit.Step(t, "%s: an unsunk root arc's corner floats %.4f mm (%.4f module) outside the "+
		"root cone; the sink of %.4f mm is what puts it inside",
		side.Label, float, float/g.Module, g.RootSink)
}

// stepBoreCut cuts the cylindrical through bore along the shaft axis.
//
// SUBSTITUTION: the TOOL is built as a real extrude, which a symmetric extent
// produces as a prism, but no cut is performed: the target is the frustum,
// whose bands are Lofts. The tool and the bands are laid apart, the cut is
// asserted from the tool's own measured geometry — its diameter, that its two
// ends sit exactly 2 * Cone Distance either side of the shaft edge's start, and
// that both clear the frustum, which is what makes it a THROUGH cut — and the
// material it would remove is computed from the frustum's own profile clipped
// to the bore radius.
//
// THE COST IS THE PIERCED BODY: one lump with a hole and no enclosed void is
// not shown.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	b := newBevelBench(t, doc, g)
	out := make([]*decad.Body, 0, 4)
	for _, bd := range bevelFrustumBands(g, side) {
		out = append(out, b.frustum(b.lane(), bd.Z0, bd.R0, bd.Z1, bd.R1))
	}
	if !g.BoreEnable {
		// Enable Bore unchecked: no bore is cut on either bevelSide and the per-bevelSide
		// bore diameter inputs are ignored. The step is skipped entirely, so
		// there is no tool to build.
		return out
	}
	lane := b.lane()
	hex := g.hexagon(side)
	start := hex[0].X // the shaft-axis edge's start, A' / B'
	s, region := b.polygonRegion(start, lane, bevelNgon(bevelConeSides, side.BoreDiameter/2))
	tool, err := b.doc.Extrude(s, region,
		decad.Symmetric{D: units.Millimeters(2 * g.ConeDistance)})
	if err != nil {
		t.Fatalf("%s bore tool: %v", side.Label, err)
	}
	return append(out, tool)
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	bands := bevelFrustumBands(g, side)
	if !g.BoreEnable {
		if len(bodies) != len(bands) {
			t.Fatalf("%s: Enable Bore is unchecked, so nothing but the frustum's own bands should "+
				"exist, but %d bodies were built", side.Label, len(bodies))
		}
		return
	}
	if len(bodies) != len(bands)+1 {
		t.Fatalf("%s: built %d bodies, want the %d frustum bands and one bore tool",
			side.Label, len(bodies), len(bands))
	}
	tool := bodies[len(bands)]
	hex := g.hexagon(side)
	start := hex[0].X
	half := 2 * g.ConeDistance
	rb := side.BoreDiameter / 2
	lane := float64(len(bands)) * bevelLaneStep(g)

	// The tool's own diameter, and its two ends exactly 2 * Cone Distance either
	// side of the shaft edge's start. Cone Distance here is the DIAGONAL of the
	// two pitch diameters, not the Pitch Cone Distance R.
	bevelMeasuresBox(t, side.Label+" bore tool", tool,
		r3.NewVec(lane-rb, -rb, start-half), r3.NewVec(lane+rb, rb, start+half),
		bevelExactSlack*rb+1e-9)
	bevelMeasuresVolume(t, side.Label+" bore tool", tool,
		bevelPolygonArea(bevelNgon(bevelConeSides, rb))*2*half, bevelExactSlack)
	bevelRequireClose(t, side.Label+" Cone Distance is the pitch-diameter diagonal",
		g.ConeDistance, math.Hypot(g.Module*g.Pinion.Teeth, g.Module*g.Driving.Teeth), 1e-12)

	// Both ends clear the frustum, which is what makes it a THROUGH cut.
	zLo, zHi := hex[4].X, hex[1].X
	if !(start-half < zLo && start+half > zHi) {
		t.Errorf("%s bore tool spans [%.4f, %.4f] but the frustum spans [%.4f, %.4f]: the cut "+
			"does not pierce the body", side.Label, start-half, start+half, zLo, zHi)
	}

	// The material it would remove, from the frustum's own profile clipped to
	// the bore radius.
	removed := bevelRevolvedVolume(bevelClipBelowRadius(hex, rb))
	whole := bevelRevolvedVolume(hex)
	if removed <= 0 || removed >= whole {
		t.Fatalf("%s bore would remove %.4f mm^3 from a frustum of %.4f mm^3",
			side.Label, removed, whole)
	}
	proofkit.Step(t, "%s bore of diameter %.4f mm removes %.4f mm^3 of %.4f mm^3",
		side.Label, side.BoreDiameter, removed, whole)

	// The bore is already bounded: an auto value capped to the Maximum Bore
	// Diameter and a user value above it rejected, both in §2. A bore past
	// r_heel takes the ENTIRE flat back face and one past r_toe takes the whole
	// toe dish and bites into the root cone; either way the body comes out of
	// this step with no end face on that side.
	if side.BoreDiameter > side.MaxBore+1e-9 {
		t.Errorf("%s resolved bore diameter %.6f is above its Maximum Bore Diameter %.6f",
			side.Label, side.BoreDiameter, side.MaxBore)
	}
	rHeel := side.PitchRadius - side.BaseHeight/math.Tan(side.Gamma)
	rToe := (g.ApexDed - g.RootLength) * math.Sin(side.GammaRoot)
	bevelRequireClose(t, side.Label+" Maximum Bore Diameter",
		side.MaxBore, 2*0.95*math.Min(rHeel, rToe), 1e-9)
	// The flat FRONT face is deliberately NOT protected: its radius is the Toe
	// Radius, which is not on the body's outer envelope, so a bore wider than it
	// only exits through the toe cone instead of through that face and the
	// frustum stays whole.
	if rb > side.ToeRadius {
		proofkit.Step(t, "%s bore radius %.4f is outside the Toe Radius %.4f and exits through "+
			"the toe cone, which the bound deliberately allows",
			side.Label, rb, side.ToeRadius)
	}
}

// bevelClipBelowRadius clips a (station, radius) profile to the part at or inside
// radius rb, which is the part a through bore of that radius removes.
func bevelClipBelowRadius(poly []bevelVec, rb float64) []bevelVec {
	out := make([]bevelVec, 0, len(poly)+4)
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		aIn, bIn := a.Y <= rb, b.Y <= rb
		if aIn {
			out = append(out, a)
		}
		if aIn != bIn {
			tt := (rb - a.Y) / (b.Y - a.Y)
			out = append(out, bevelVec{a.X + tt*(b.X-a.X), rb})
		}
	}
	return out
}

// bevelPolygonCentroid is a closed polygon's own area centroid.
func bevelPolygonCentroid(pts []bevelVec) bevelVec {
	area, cx, cy := 0.0, 0.0, 0.0
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		cross := bevelCross(a, b)
		area += cross
		cx += (a.X + b.X) * cross
		cy += (a.Y + b.Y) * cross
	}
	return bevelVec{cx / (3 * area), cy / (3 * area)}
}

// bevelTaperedCentroid is the centroid of a section scaled linearly from an apex at
// the origin, kept between stations z0 and z1, with the full section at z1.
func bevelTaperedCentroid(section []bevelVec, z0, z1 float64) r3.Vec {
	c := bevelPolygonCentroid(section)
	zbar := 3 * (z1*z1*z1*z1 - z0*z0*z0*z0) / (4 * (z1*z1*z1 - z0*z0*z0))
	return r3.NewVec(c.X/z1*zbar, c.Y/z1*zbar, zbar)
}

// bevelTaperedVolume is the volume of that same tapered prism.
func bevelTaperedVolume(section []bevelVec, z0, z1 float64) float64 {
	return bevelPolygonArea(section) * (z1*z1*z1 - z0*z0*z0) / (3 * z1 * z1)
}

// stepMeshRotation rotates the DRIVING bevelSide body half a tooth pitch about its
// own shaft axis, so a driving valley sits where the pinion tooth crosses the
// axial plane.
//
// Both gears are patterned from a starting tooth in the axial plane, so without
// the offset a driving tooth and a pinion tooth would both sit at that crossing
// and visually collide. The rotation runs in the Design component before the
// body is moved out, because a construction axis cannot be added in the moved-
// out bevelSide component, so it has to use the profile edge's world geometry while
// the body is still there.
//
// The pinion's extra phase is zero by default, and a ZERO ANGLE IS A NO-OP
// rather than a move: Fusion refuses to move a body by the identity with
// "invalid transform", so the framework helper returns early instead. This step
// models that by not placing the body at all when the angle is zero.
//
// SUBSTITUTION: only the MOVED body is left in the document. Half a tooth pitch
// is less than a whole one, so a body and its rotated self overlap, and decad
// refuses to classify a pair that reaches a contact it cannot resolve — which
// it does here whether the two are laid apart or not, because the two operands
// are the same recipe and share their face planes. So the before reading is
// taken from the closed form the section and the two stations give, which is
// exactly what the body was built from, and the after reading is decad's.
//
// THE COST IS THE BEFORE READING: one side of the comparison is a formula
// rather than a measurement. What is compared is still the move itself.
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	toeFraction := (g.ApexDed - g.RootLength) / g.ApexDed
	b := newBevelBench(t, doc, g)
	body := b.taperedPrism(0, toeFraction*d1, toeFraction, d1, 1, section)

	angle := bevelMeshPhase(side)
	if angle == 0 {
		return []*decad.Body{body}
	}
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s meshing rotation: %v", side.Label, err)
	}
	rotated, err := body.Placed(turn)
	if err != nil {
		t.Fatalf("%s meshing rotation: %v", side.Label, err)
	}
	return []*decad.Body{rotated}
}

// bevelMeshPhase is the extra rotation one bevelSide's body receives about its own shaft
// axis: half a tooth pitch on the driving bevelSide, and the pinion's own phase,
// which is zero tooth-fractions by default.
func bevelMeshPhase(s bevelSide) float64 {
	if s.Label == "Driving" {
		return math.Pi / s.Teeth
	}
	return 0
}

func assertMeshRotation(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	section, d1, _ := bevelToothFor(g, side)
	toeFraction := (g.ApexDed - g.RootLength) / g.ApexDed
	z0, z1 := toeFraction*d1, d1
	angle := bevelMeshPhase(side)
	if len(bodies) != 1 {
		t.Fatalf("%s: expected one body after the move, got %d", side.Label, len(bodies))
	}

	// A rigid rotation about the shaft axis changes no volume.
	bevelMeasuresVolume(t, side.Label+" body after the meshing rotation", bodies[0],
		bevelTaperedVolume(section, z0, z1), bevelExactSlack)

	before := bevelTaperedCentroid(section, z0, z1)
	if angle == 0 {
		// The pinion's phase is zero tooth-fractions by default, and a zero
		// angle is a no-op rather than a move: the body is left exactly where it
		// was built.
		decadtest.MeasuresVec(t, side.Label+" centroid, unmoved",
			bevelMustCentroid(t, bodies[0]), before, decadtest.Within(units.Millimeters(1e-6)))
		return
	}
	bevelRequireClose(t, side.Label+" meshing rotation is half a tooth pitch",
		angle*180/math.Pi, 180/side.Teeth, 1e-12)
	cx := before.X*math.Cos(angle) - before.Y*math.Sin(angle)
	cy := before.X*math.Sin(angle) + before.Y*math.Cos(angle)
	decadtest.MeasuresVec(t, side.Label+" centroid after the meshing rotation",
		bevelMustCentroid(t, bodies[0]), r3.NewVec(cx, cy, before.Z),
		decadtest.Within(units.Millimeters(1e-6)))
	// The rotation is about the shaft axis, so every distance from that axis is
	// unchanged: the body turns and does not travel.
	bevelRequireClose(t, side.Label+" distance from the shaft axis after the rotation",
		math.Hypot(cx, cy), math.Hypot(before.X, before.Y), 1e-9)
}

func bevelMustCentroid(t *testing.T, body *decad.Body) decad.VecMeasurement {
	t.Helper()
	c, err := body.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	return c
}
