// This file holds the cycloidal drive's solid steps, one function per Fusion
// timeline feature: the lobe sector extrude and the disc it patterns into, the
// output-hole cut and its pattern, the disc bore, the cam sections and their
// join, the housing base and the pinless casing, the output plate, its pin and
// socket and the pin pattern, and the rim chamfers.
//
// Three substitutions run through the file, each made because decad refuses
// the operation Fusion performs rather than because it would be convenient.
//
// Chorded free-form boundaries. Fusion draws the lobe and the casing contour
// as fitted splines. decad refuses to extrude a free-form span whose curvature
// sign it cannot certify — measured here as "a free-form span's curvature-sign
// certificate is still mixed at the fixed subdivision depth cap" — and both
// curves turn from concave to convex within one span, so every solid step
// extrudes the chord polyline through the same sampled points. What it costs
// is the sagitta of each span: under the spec's 5-degree turn limit the chord
// area sits within about a part in a thousand of the spline's, so an area or a
// volume proven here is proven to that, and the smoothness of the drawn wall
// is not proven at all.
//
// Patterned sectors are not joined by boolean. Fusion circular-patterns one
// sector and Joins the copies, which share a whole face. decad refuses a
// boolean whose operands meet face to face — "whether their true surfaces
// touch or cross is decided by where the chords fall" — so the patterned
// result is built from the whole boundary in one extrude and the tiling is
// proven the way the Join's outcome is judged: one lump, and a volume that is
// exactly the sector count times the sector. A tiling that left a gap between
// sectors is exactly what that pair of readings refuses, which is the reported
// "several unnamed bodies" bug.
//
// Abutting joins overlap by a sliver. Where Fusion Joins two bodies that meet
// on a plane — the cam's two sections, the casing onto the housing base — the
// proof overlaps them by overlapSliver and says so at the call. What that
// costs is the volume of the sliver, which each step subtracts explicitly; the
// claim being proven is connectivity, which the sliver does not create out of
// nothing, since two bodies that do not meet in plan stay two lumps however
// far they overlap in z.
package cycloidal_test

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

// overlapSliver is the axial overlap that stands in for an exact face-to-face
// abutment decad will not evaluate. It is small enough that its own volume is
// a rounding error against the bodies it joins, and large enough to sit far
// outside any facet chord.
const overlapSliver = 0.01

// toolOverhang is how far a cutting cylinder runs past both faces of the body
// it pierces. Fusion's cut is flush with the disc, which puts the tool's cap in
// the plane of the disc's, and a coplanar pair is the same question decad
// refuses; running the tool past both faces asks one it answers.
const toolOverhang = 1.0

// wallRelief is how far the housing base's outer wall is grown past the
// casing's so the two cross instead of coinciding. Fusion's Join has them
// flush, which puts one cylinder in both operands, and decad refuses a
// tangent contact it cannot classify.
const wallRelief = 0.01

// boxSlack is the millimetre slack for a bounding box read off a body whose
// boundary the evaluator states exactly — a chorded prism. Only float64
// rounding separates the reading from the polygon it was built from.
const boxSlack = 1e-6

// -- the rotor disc ----------------------------------------------------------

// stepExtrudeLobeSector extrudes the Rotor Lobe sketch's one closed profile —
// the pie sector bounded by spoke 1, the lobe and spoke 2 — by Disc Thickness,
// from disc d's own plane.
func stepExtrudeLobeSector(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{
		prismFromPolygon(t, doc, lobeSector(d), d.discBase(), d.T, "Cycloidal Disk sector"),
	}
}

func assertExtrudeLobeSector(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	sector := lobeSector(d)
	label := "Cycloidal Disk sector"
	requireOneLump(t, label, bodies[0])
	// The prism's volume is the polygon's area times the extent, which the
	// evaluator computes the same way, so only float64 rounding separates them.
	measuresVolume(t, label, bodies[0], polygonArea(sector)*d.T, exact())
	measuresPolygonBox(t, label, bodies[0], sector, d.discBase(), d.discBase()+d.T)
}

// lobeSector is the pie sector's boundary: the disc centre, then the lobe.
func lobeSector(d dims) []point {
	c := d.centre()
	return append([]point{c}, lobeSamples(d, c.X, c.Y, d.Phi)...)
}

// stepJoinDiscSectors Joins disc d's own L patterned sectors into one
// `Cycloidal Disk {d+1}` body.
//
// The Join's outcome is one geometric claim: that L sectors turned by 360/L
// about Od tile the rotor with no gap and no overlap. The proof builds the
// tiled boundary in one extrude, for the reason this file's header gives, and
// holds the claim as the pair of readings the assertion makes.
//
// The circular pattern that produced those sectors is the step before this
// one, and it is [PROSE]. decad has no pattern feature, and placing the L
// copies by hand leaves L bodies that meet face to face: the boolean that
// would join them is refused, and leaving them live makes decad's report
// unable to say whether a touching pair crosses. The tiling they have to
// satisfy is what this step reads instead.
func stepJoinDiscSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{
		prismFromPolygon(t, doc, discBoundary(d), d.discBase(), d.T, "Cycloidal Disk"),
	}
}

func assertJoinDiscSectors(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Cycloidal Disk"
	disc := bodies[0]
	// One lump is the Join's whole outcome: L sectors that did not meet would
	// arrive as L bodies.
	requireOneLump(t, label, disc)

	full := discBoundary(d)
	measuresVolume(t, label, disc, polygonArea(full)*d.T, exact())
	measuresPolygonBox(t, label, disc, full, d.discBase(), d.discBase()+d.T)

	// The tiling claim, read against a real sector rather than against the
	// same formula twice: the seed sector is built in a scratch document of
	// its own, because a second live body in this document would be judged
	// against the disc as an interfering pair.
	scratch := decad.New()
	seed := prismFromPolygon(t, scratch, lobeSector(d), d.discBase(), d.T, "Cycloidal Disk sector")
	decadtest.Measures(t, "one lobe sector against a disc share",
		volumeOf(t, "Cycloidal Disk sector", seed),
		units.CubicMillimeters(polygonArea(full)*d.T/float64(d.L)), exact())
}

// discBoundary is the whole rotor outline: L lobes tiled about the disc centre.
func discBoundary(d dims) []point {
	c := d.centre()
	return fullLobeSamples(d, c.X, c.Y, d.Phi)
}

// stepCutOutputHole cuts the Output Hole sketch's one solid circle through the
// disc by Disc Thickness, restricted to that disc.
func stepCutOutputHole(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	disc := prismFromPolygon(t, doc, discBoundary(d), d.discBase(), d.T, "Cycloidal Disk")
	return []*decad.Body{cutHoles(t, doc, disc, d, 1)}
}

func assertCutOutputHole(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	assertHoleCount(t, bodies[0], derive(p), 1)
}

// stepPatternOutputHoles is the circular pattern of the cut feature ×M about
// the Disk Axis, which leaves M holes orbiting Od.
//
// The M openings are stated as holes in the extruded profile rather than cut
// one after another: decad refuses a boolean whose tool is finer than the mesh
// the previous boolean left, which a second cut into this disc always is. The
// solid is the same solid, and stating it this way reads its volume exactly
// instead of to a facet chord. What it costs is that only the first cut is
// proven as a boolean, by stepCutOutputHole.
func stepPatternOutputHoles(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{
		prismWithHoles(t, doc, discBoundary(d), outputHoles(d), d.discBase(), d.T, "Cycloidal Disk"),
	}
}

func assertPatternOutputHoles(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Cycloidal Disk"
	requireOneLump(t, label, bodies[0])
	removed := float64(d.M) * math.Pi * d.DHole * d.DHole / 4 * d.T
	measuresVolume(t, label, bodies[0], polygonArea(discBoundary(d))*d.T-removed, exact())
	// M holes on the Rop circle only stay M holes while they do not run into
	// one another; the spec rejects the dialog on exactly this comparison.
	chord := 2 * d.Rop * math.Sin(math.Pi/float64(d.M))
	if chord <= d.DHole {
		t.Fatalf("the %d output holes overlap: a %.6f mm hole on a %.6f mm chord between neighbours",
			d.M, d.DHole, chord)
	}
	// And only while they stay inside the rotor. The disc's own smallest
	// radius is the valley circle Rv, so a hole whose outer edge passes it
	// breaks out through the lobe profile. The spec's validity table does not
	// check this, and the volume above is what catches it.
	if reach := d.Rop + d.DHole/2; reach >= d.Rv {
		t.Fatalf("the output holes reach %.6f mm from Od, at or past the valley circle at "+
			"%.6f mm, so they breach the rotor's rim", reach, d.Rv)
	}
}

// outputHoles is the M openings the disc carries, on the output-pin circle
// about the disc centre.
func outputHoles(d dims) []hole {
	c := d.centre()
	out := make([]hole, 0, d.M)
	for k := range d.M {
		a := 2 * math.Pi * float64(k) / float64(d.M)
		out = append(out, hole{c.X + d.Rop*math.Cos(a), c.Y + d.Rop*math.Sin(a), d.DHole / 2})
	}
	return out
}

// cutHoles pierces the disc with the first n of its M output holes, each tool
// run past both faces for the reason toolOverhang gives.
func cutHoles(t *testing.T, doc *decad.Document, disc *decad.Body, d dims, n int) *decad.Body {
	t.Helper()
	c := d.centre()
	body := disc
	for k := range n {
		a := 2 * math.Pi * float64(k) / float64(d.M)
		tool := cylinder(t, doc,
			c.X+d.Rop*math.Cos(a), c.Y+d.Rop*math.Sin(a), d.DHole/2,
			d.discBase()-toolOverhang, d.T+2*toolOverhang, "output hole tool")
		cut, err := decad.Cut(body, tool)
		if err != nil {
			t.Fatalf("cut output hole %d of %d: %v", k+1, n, err)
		}
		body = cut
	}
	return body
}

func assertHoleCount(t *testing.T, disc *decad.Body, d dims, n int) {
	t.Helper()
	label := "Cycloidal Disk"
	requireOneLump(t, label, disc)
	removed := float64(n) * math.Pi * d.DHole * d.DHole / 4 * d.T
	// decad tessellates the cylindrical tool before the boolean, so the hole
	// it removes is an inscribed prism slightly under pi*r^2*h; faceted()
	// states that chord error.
	measuresVolume(t, label, disc, polygonArea(discBoundary(d))*d.T-removed, faceted())
}

// stepCutDiscBore cuts the Disc Bore sketch's solid circle — the cam outer
// enlarged by Bearing Clearance — through the disc.
//
// The bore is cut through the plain disc rather than through the holed one:
// the bore and the output holes never meet, which stepDiscBoreSketch refuses
// the case for when they would, so the volume this step removes does not
// depend on whether the holes are already there.
func stepCutDiscBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	c := d.centre()
	disc := prismFromPolygon(t, doc, discBoundary(d), d.discBase(), d.T, "Cycloidal Disk")
	tool := cylinder(t, doc, c.X, c.Y, d.boreRadius(),
		d.discBase()-toolOverhang, d.T+2*toolOverhang, "disc bore tool")
	bored, err := decad.Cut(disc, tool)
	if err != nil {
		t.Fatalf("cut the disc centre bore: %v", err)
	}
	return []*decad.Body{bored}
}

func assertCutDiscBore(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Cycloidal Disk"
	requireOneLump(t, label, bodies[0])
	removed := math.Pi * d.boreRadius() * d.boreRadius() * d.T
	measuresVolume(t, label, bodies[0], polygonArea(discBoundary(d))*d.T-removed, faceted())
	// The running gap is the whole point of the enlarged bore: the disc's bore
	// stands off the cam outer by half the Bearing Clearance all the way round.
	if got := d.boreRadius() - d.CBD/2; math.Abs(got-d.Clr/2) > 1e-12 {
		t.Fatalf("the bore stands off the cam by %.9f mm, want half the Bearing Clearance %.9f mm",
			got, d.Clr/2)
	}
}

// -- the eccentric cam -------------------------------------------------------

// stepExtrudeCamSection extrudes the `Eccentric Cam {d+1}` cross-section: the
// cam outer on Od with the input-shaft bore on O as its one hole, or the plain
// cam disc when Input Shaft Diameter is 0. Section d runs T + g when another
// section follows it and T when it is the last.
func stepExtrudeCamSection(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{camSection(t, doc, p, int(math.Round(p[keyDisc])), 0)}
}

func assertExtrudeCamSection(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	c := d.centre()
	label := "Eccentric Cam section"
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], camSectionArea(d)*d.camSectionHeight(), exact())
	measuresBox(t, label, bodies[0],
		r3.NewVec(c.X-d.CBD/2, c.Y-d.CBD/2, d.discBase()),
		r3.NewVec(c.X+d.CBD/2, c.Y+d.CBD/2, d.discBase()+d.camSectionHeight()),
		decadtest.Within(units.Millimeters(1e-3)))
}

// stepJoinCamSections Joins the two eccentric sections into one `Eccentric Cam`.
//
// The claim is that the sections join into one continuous solid even though
// their centres are 2E apart: they overlap wherever both discs of radius
// CenterBearingDiameter/2 cover the same ground.
//
// Two substitutions. The axial abutment is replaced by an overlapSliver, for
// the reason this file's header gives. And the two sections are joined without
// their input bore: both sections put the same bore on the drive axis, so
// through the overlap their bore walls are one cylinder, and decad refuses a
// boolean whose operands it cannot classify a tangent contact in. Cutting the
// bore afterwards is refused too, since the joined body is faceted and holds a
// mesh coarser than a fresh cut's tolerance. So the two-loop cam section is
// proven by stepExtrudeCamSection, this step proves that the two sections meet
// in one lump, and the fact that ties them — the bore staying inside both
// sections, so it runs unbroken through the join — is checked arithmetically
// in the assertion.
func stepJoinCamSections(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	if d.D < 2 {
		proofkit3d.Unmodelled(t, "one disc leaves one cam section and nothing to join")
	}
	lower, upper := dimsFor(p, 0), dimsFor(p, 1)
	lowC, upC := lower.centre(), upper.centre()
	lowBody := cylinder(t, doc, lowC.X, lowC.Y, d.CBD/2, lower.discBase(),
		lower.camSectionHeight(), "Eccentric Cam section 1")
	upBody := cylinder(t, doc, upC.X, upC.Y, d.CBD/2, upper.discBase()-overlapSliver,
		upper.camSectionHeight()+overlapSliver, "Eccentric Cam section 2")
	joined, err := decad.Union(lowBody, upBody)
	if err != nil {
		t.Fatalf("join the two cam sections: %v", err)
	}
	return []*decad.Body{joined}
}

func assertJoinCamSections(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Eccentric Cam"
	// One lump is the claim. Two sections whose outer circles did not overlap
	// in plan would stay two lumps however far they overlapped in z.
	requireOneLump(t, label, bodies[0])

	lower, upper := dimsFor(p, 0), dimsFor(p, 1)
	top := upper.discBase() + upper.camSectionHeight()
	disc := math.Pi * d.CBD * d.CBD / 4
	// The sliver is swept by both extrudes and filled once by the solid, so it
	// is counted out at the lens the two outer circles share.
	total := disc*lower.camSectionHeight() + disc*(upper.camSectionHeight()+overlapSliver) -
		lensArea(d.CBD/2, 2*d.E)*overlapSliver
	measuresVolume(t, label, bodies[0], total, faceted())
	measuresBox(t, label, bodies[0],
		r3.NewVec(-d.E-d.CBD/2, -d.CBD/2, 0),
		r3.NewVec(d.E+d.CBD/2, d.CBD/2, top),
		decadtest.Within(units.Millimeters(1e-2)))
	if d.ISD <= 0 {
		return
	}
	// The bore is inside both sections all the way up, which is what
	// E + InputShaftDiameter/2 < CenterBearingDiameter/2 buys and what makes
	// the joined cam's bore one unbroken hole.
	if d.E+d.ISD/2 >= d.CBD/2 {
		t.Fatalf("the input bore reaches %.6f mm from a section centre, past the cam outer at "+
			"%.6f mm, so the join would break it open", d.E+d.ISD/2, d.CBD/2)
	}
	if d.ISD >= d.CBD {
		t.Fatalf("the input bore's %.6f mm diameter is not under the cam's %.6f mm", d.ISD, d.CBD)
	}
}

// camSectionArea is the cross-section the cam extrudes: the cam disc, less the
// input bore when the dialog asks for one.
func camSectionArea(d dims) float64 {
	area := math.Pi * d.CBD * d.CBD / 4
	if d.ISD > 0 {
		area -= math.Pi * d.ISD * d.ISD / 4
	}
	return area
}

// camSection builds cam section `disc`, dropped by sink so the section above
// overlaps the one below instead of abutting it.
func camSection(t *testing.T, doc *decad.Document, p map[string]float64, disc int, sink float64) *decad.Body {
	t.Helper()
	d := dimsFor(p, disc)
	c := d.centre()
	if d.ISD <= 0 {
		return cylinder(t, doc, c.X, c.Y, d.CBD/2, d.discBase()-sink,
			d.camSectionHeight()+sink, "Eccentric Cam section")
	}
	return annulus(t, doc, c.X, c.Y, d.CBD/2, 0, 0, d.ISD/2, d.discBase()-sink,
		d.camSectionHeight()+sink, "Eccentric Cam section")
}

// dimsFor resolves a case at a chosen disc index, for the steps that run over
// the whole stack and need every disc's frame rather than the case's own.
func dimsFor(p map[string]float64, disc int) dims {
	q := make(map[string]float64, len(p))
	for k, v := range p {
		q[k] = v
	}
	q[keyDisc] = float64(disc)
	return derive(q)
}

// lensArea is the area two circles of radius r whose centres are dist apart
// share, and 0 when they do not reach each other.
func lensArea(r, dist float64) float64 {
	if dist >= 2*r {
		return 0
	}
	return 2*r*r*math.Acos(dist/(2*r)) - dist/2*math.Sqrt(4*r*r-dist*dist)
}

// -- the housing -------------------------------------------------------------

// stepExtrudeHousingBase extrudes the `Housing Ring` annulus by Base Thickness
// away from the disc, from the construction plane 1 mm below it.
func stepExtrudeHousingBase(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{housingBase(t, doc, d, 0)}
}

func assertExtrudeHousingBase(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Housing Ring"
	ro, ri := d.housingOuterRadius(), d.housingInnerRadius()
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], math.Pi*(ro*ro-ri*ri)*d.BaseT, exact())
	// The base sits below the target plane, its top face exactly 1 mm under
	// it, which is what the casing's downward side has to reach to Join.
	measuresBox(t, label, bodies[0],
		r3.NewVec(-ro, -ro, -1-d.BaseT), r3.NewVec(ro, ro, -1),
		decadtest.Within(units.Millimeters(1e-3)))
}

func housingBase(t *testing.T, doc *decad.Document, d dims, relief float64) *decad.Body {
	t.Helper()
	return annulus(t, doc, 0, 0, d.housingOuterRadius()+relief, 0, 0, d.housingInnerRadius(),
		-1-d.BaseT, d.BaseT, "Housing Ring")
}

// stepExtrudeCasingSector extrudes the `Ring Casing` wedge two-sided: up to the
// stack top and 1 mm down to the housing base's top face.
func stepExtrudeCasingSector(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{casingSector(t, doc, derive(p), 0)}
}

func assertExtrudeCasingSector(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Ring Casing sector"
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], casingSectorArea(d)*(d.stackTop()+1), faceted())
	lo, hi := polygonBounds(casingSectorReach(d))
	measuresBox(t, label, bodies[0],
		r3.NewVec(lo.X, lo.Y, -1), r3.NewVec(hi.X, hi.Y, d.stackTop()),
		decadtest.Within(units.Millimeters(1e-3)))
}

// stepJoinCasingSectors Joins the N patterned casing sectors into one casing
// body. The pattern that produced them is the step before this one, and it is
// [PROSE] for the reason stepJoinDiscSectors gives.
//
// This is where the bin-edge rule earns its place. The sector's two spokes sit
// at exactly -pi/N and +pi/N, so a turn of 2*pi/N carries one onto the other
// and the N sectors tile the ring. Contour points emitted at bin centres would
// inset each end by half a bin, leaving an angular gap at every seam, and the
// readings below — one lump, and a volume that is exactly N sectors — are what
// refuses that.
func stepJoinCasingSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{casingRing(t, doc, derive(p), 0)}
}

func assertJoinCasingSectors(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Ring Casing"
	requireOneLump(t, label, bodies[0])
	height := d.stackTop() + 1
	measuresVolume(t, label, bodies[0], casingRingArea(d)*height, exact())

	// The same claim against a real sector, built in a scratch document so the
	// ring is not judged against an interfering neighbour. N sectors have to
	// come to the ring exactly, which is what a seam gap would break.
	scratch := decad.New()
	seed := casingSector(t, scratch, d, 0)
	decadtest.Measures(t, "one casing sector against a ring share",
		volumeOf(t, "Ring Casing sector", seed),
		units.CubicMillimeters(casingRingArea(d)*height/float64(d.N)), faceted())

	ro := d.housingOuterRadius()
	measuresBox(t, label, bodies[0],
		r3.NewVec(-ro, -ro, -1), r3.NewVec(ro, ro, d.stackTop()),
		decadtest.Within(units.Millimeters(1e-3)))
}

// stepCombineHousing Joins the casing into the base, leaving one `Housing`.
func stepCombineHousing(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	// The base's outer wall is grown by wallRelief so it crosses the casing's
	// rather than coinciding with it, for the reason wallRelief gives.
	base := housingBase(t, doc, d, wallRelief)
	casing := casingRing(t, doc, d, overlapSliver)
	joined, err := decad.Union(base, casing)
	if err != nil {
		t.Fatalf("join the casing into the housing base: %v", err)
	}
	return []*decad.Body{joined}
}

func assertCombineHousing(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Housing"
	// One printable part is the claim the Combine makes, and the reason the
	// casing's downward side is exactly the housing plane's 1 mm offset.
	requireOneLump(t, label, bodies[0])

	ro, ri := d.housingOuterRadius(), d.housingInnerRadius()
	ring := casingRingArea(d)
	// The casing's footprint lies wholly inside the base annulus, so the
	// sliver the two share is the ring's own area; check that rather than
	// assume it, since it is what makes the subtraction below correct.
	for i, q := range fullContour(d) {
		if r := math.Hypot(q.X, q.Y); r <= ri {
			t.Fatalf("contour point %d falls at %.6f mm, inside the base's inner lip at %.6f mm",
				i, r, ri)
		}
	}
	rb := ro + wallRelief
	base := math.Pi * (rb*rb - ri*ri) * d.BaseT
	casing := ring * (d.stackTop() + 1 + overlapSliver)
	measuresVolume(t, label, bodies[0], base+casing-ring*overlapSliver, faceted())
	measuresBox(t, label, bodies[0],
		r3.NewVec(-rb, -rb, -1-d.BaseT), r3.NewVec(rb, rb, d.stackTop()),
		decadtest.Within(units.Millimeters(1e-2)))
}

// casingRingArea is the joined casing's cross-section: the outer disc less the
// region the contour encloses.
func casingRingArea(d dims) float64 {
	ro := d.housingOuterRadius()
	return math.Pi*ro*ro - polygonArea(fullContour(d))
}

// casingSectorArea is the wedge's area: one pin pitch of the outer disc, less
// the pie the contour encloses over that pitch.
func casingSectorArea(d dims) float64 {
	ro := d.housingOuterRadius()
	return math.Pi*ro*ro/float64(d.N) - polygonArea(append([]point{{0, 0}}, contourPitch(d)...))
}

// casingSectorReach is the point set the wedge's bounding box is read from: the
// contour, the two outer corners, and the outer arc's own farthest point.
func casingSectorReach(d dims) []point {
	ro := d.housingOuterRadius()
	half := math.Pi / float64(d.N)
	out := append([]point{}, contourPitch(d)...)
	return append(out,
		point{ro * math.Cos(-half), ro * math.Sin(-half)},
		point{ro * math.Cos(half), ro * math.Sin(half)},
		point{ro, 0})
}

// casingSector builds one wedge — the contour, a spoke out to the outer circle
// at each end, and the outer arc between them — two-sided, dropped by sink so
// it can overlap the housing base instead of abutting it.
func casingSector(t *testing.T, doc *decad.Document, d dims, sink float64) *decad.Body {
	t.Helper()
	ro := d.housingOuterRadius()
	half := math.Pi / float64(d.N)
	s := decadtest.NewSketch(t)
	centre := s.CreatePoint(0, 0)
	pts := polyline(s, contourPitch(d), false)
	first := s.CreatePoint(ro*math.Cos(-half), ro*math.Sin(-half))
	last := s.CreatePoint(ro*math.Cos(half), ro*math.Sin(half))
	s.CreateLine(pts[0], first)
	s.CreateLine(pts[len(pts)-1], last)
	// The outer boundary is the real arc, not a chord of it, so that N sector
	// areas add up to the ring's without a rounding of their own.
	s.CreateArc(centre, first, last)
	s.Fix(centre)
	s.Fix(first)
	s.Fix(last)
	for _, q := range pts {
		s.Fix(q)
	}
	profile := decadtest.SolveRegion(t, s)
	body, err := doc.Extrude(s, profile, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(d.stackTop())},
		Two: decad.DistanceSide{D: units.Millimeters(1 + sink)},
	})
	if err != nil {
		t.Fatalf("extrude the ring casing sector: %v", err)
	}
	return body
}

// casingRing builds the joined casing in one extrude, two-sided, reaching sink
// further down so it can overlap the housing base.
func casingRing(t *testing.T, doc *decad.Document, d dims, sink float64) *decad.Body {
	t.Helper()
	ro := d.housingOuterRadius()
	s := decadtest.NewSketch(t)
	oc := s.CreatePoint(0, 0)
	s.Fix(oc)
	outer := s.CreateCircle(oc, ro)
	s.AddConstraint(sketch.NewDiameter(outer, 2*ro))
	for _, q := range polyline(s, fullContour(d), true) {
		s.Fix(q)
	}
	profile := holedProfile(t, s)
	body, err := doc.Extrude(s, profile, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(d.stackTop())},
		Two: decad.DistanceSide{D: units.Millimeters(1 + sink)},
	})
	if err != nil {
		t.Fatalf("extrude the joined ring casing: %v", err)
	}
	return body
}

// -- the output member -------------------------------------------------------

// stepExtrudeOutputPlate extrudes every profile of the `Output Plate` sketch —
// the plate with its pin bite and the pin disc, so the pin's footprint is
// solid — by Output Plate Thickness, away from the disc. The two profiles
// together are the plate's whole outer circle, which is what this builds.
func stepExtrudeOutputPlate(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{outputPlate(t, doc, d)}
}

func assertExtrudeOutputPlate(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Output Plate"
	rp := d.plateRadius()
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], math.Pi*rp*rp*d.PlateT, exact())
	// The plate sits 1 mm above the top disc and grows away from it, which is
	// the mirror of the housing's plane and direction.
	measuresBox(t, label, bodies[0],
		r3.NewVec(-rp, -rp, d.stackTop()+1), r3.NewVec(rp, rp, d.stackTop()+1+d.PlateT),
		decadtest.Within(units.Millimeters(1e-3)))
}

func outputPlate(t *testing.T, doc *decad.Document, d dims) *decad.Body {
	t.Helper()
	return cylinder(t, doc, 0, 0, d.plateRadius(), d.stackTop()+1, d.PlateT, "Output Plate")
}

// stepExtrudeOutputPin extrudes the pin disc two-sided: Output Plate Thickness
// into the plate, and the stack top plus 1 mm toward the disc, so the pin
// reaches disc 0's bottom face and threads every disc's output holes.
func stepExtrudeOutputPin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	return []*decad.Body{outputPin(t, doc, d, 0, 0)}
}

func assertExtrudeOutputPin(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Output Pin"
	r := d.DPin / 2
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], math.Pi*r*r*(d.PlateT+d.stackTop()+1), exact())
	// The lower end lands on z = 0, disc 0's bottom face. A pin that stopped
	// short would miss the lower disc of a two-disc stack.
	measuresBox(t, label, bodies[0],
		r3.NewVec(d.Rop-r, -r, 0), r3.NewVec(d.Rop+r, r, d.stackTop()+1+d.PlateT),
		decadtest.Within(units.Millimeters(1e-3)))
	// The pin passes through a hole oversized by exactly 2E, which is the
	// orbit clearance that lets the eccentric disc carry it.
	if got := d.DHole - d.DPin; math.Abs(got-2*d.E) > 1e-12 {
		t.Fatalf("the output hole is %.9f mm wider than its pin, want 2E = %.9f mm", got, 2*d.E)
	}
}

// stepCutPinSocket is the Combine-Cut that sinks the pin's own footprint out of
// the plate, leaving the pin seated in a matching hole.
//
// Fusion keeps the tool body, so the plate and the pin end up touching in one
// document. decad judges every pair of live bodies, and a touching pair leaves
// its report unable to say whether the two cross, so the proof cuts with a tool
// the boolean consumes and holds the pin's own proof in the step above. The
// tool runs overlapSliver past the plate's top face, for the reason this file's
// header gives about coplanar caps.
func stepCutPinSocket(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	plate := outputPlate(t, doc, d)
	// The tool spans the plate alone, run past both of its faces. Fusion cuts
	// with the pin body itself, which also reaches down to disc 0 and removes
	// nothing on the way; a tool that long leaves its cap inside decad's chord
	// tolerance of the plate's and the boolean is refused.
	tool := cylinder(t, doc, d.Rop, 0, d.DPin/2,
		d.stackTop()+1-toolOverhang, d.PlateT+2*toolOverhang, "Output Pin socket tool")
	socketed, err := decad.Cut(plate, tool)
	if err != nil {
		t.Fatalf("cut the output pin's socket: %v", err)
	}
	return []*decad.Body{socketed}
}

func assertCutPinSocket(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Output Plate"
	rp, r := d.plateRadius(), d.DPin/2
	requireOneLump(t, label, bodies[0])
	measuresVolume(t, label, bodies[0], math.Pi*(rp*rp-r*r)*d.PlateT, faceted())
	measuresBox(t, label, bodies[0],
		r3.NewVec(-rp, -rp, d.stackTop()+1), r3.NewVec(rp, rp, d.stackTop()+1+d.PlateT),
		decadtest.Within(units.Millimeters(1e-2)))
}

// stepPatternOutputPins is the circular pattern ×M about the Drive Axis of the
// pin extrude, its socket Combine and the pin-end chamfer, which leaves M pins
// orbiting O and named `Output Pin 1` through `Output Pin M`.
func stepPatternOutputPins(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	pins := make([]*decad.Body, 0, d.M)
	for k := range d.M {
		pins = append(pins, outputPin(t, doc, d, 2*math.Pi*float64(k)/float64(d.M), 0))
	}
	return pins
}

func assertPatternOutputPins(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	r := d.DPin / 2
	if len(bodies) != d.M {
		t.Fatalf("the pattern left %d pin(s), want Output Pin Count = %d", len(bodies), d.M)
	}
	want := math.Pi * r * r * (d.PlateT + d.stackTop() + 1)
	for k, body := range bodies {
		label := fmt.Sprintf("Output Pin %d", k+1)
		requireOneLump(t, label, body)
		measuresVolume(t, label, body, want, exact())
		a := 2 * math.Pi * float64(k) / float64(d.M)
		cx, cy := d.Rop*math.Cos(a), d.Rop*math.Sin(a)
		measuresBox(t, label, body,
			r3.NewVec(cx-r, cy-r, 0), r3.NewVec(cx+r, cy+r, d.stackTop()+1+d.PlateT),
			decadtest.Within(units.Millimeters(1e-3)))
	}
	// Neighbouring pins keep a real gap; that gap is what the output holes'
	// non-overlap bound guarantees.
	chord := 2 * d.Rop * math.Sin(math.Pi/float64(d.M))
	if chord <= d.DPin {
		t.Fatalf("neighbouring output pins overlap: a %.6f mm pin on a %.6f mm chord",
			d.DPin, chord)
	}
}

// outputPin builds one pin at angle a on the output-pin circle, reaching sink
// past the plate's top face.
func outputPin(t *testing.T, doc *decad.Document, d dims, a, sink float64) *decad.Body {
	t.Helper()
	return cylinder(t, doc, d.Rop*math.Cos(a), d.Rop*math.Sin(a), d.DPin/2,
		0, d.stackTop()+1+d.PlateT+sink, "Output Pin")
}

// -- chamfers ----------------------------------------------------------------

// stepChamferRims chamfers the outer rim of every disc-like body on both flat
// faces, at 45 degrees and equal distance, and does nothing when Chamfer Size
// is 0.
//
// What the proof chamfers is the Output Plate. The rotor disc's rim is its lobe
// profile and the Housing's is the scalloped casing contour, and decad refuses
// a cap-loop chamfer whose corner offset it cannot enclose, which a lobe valley
// and a contour seam both are; neither rim is reachable here. The spec's own
// resilient-chamfer rule exists for the same geometry, since Fusion raises on a
// lobe valley too once the chamfer grows past it, and only a Fusion session
// decides where that is.
func stepChamferRims(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	plate := outputPlate(t, doc, d)
	if d.Chamfer <= 0 {
		return []*decad.Body{plate}
	}
	chamfered, err := plate.Chamfer(decad.Edges(decad.Circular()), units.Millimeters(d.Chamfer))
	if err != nil {
		t.Fatalf("chamfer the Output Plate's cap rims at %.4f mm: %v", d.Chamfer, err)
	}
	return []*decad.Body{chamfered}
}

func assertChamferRims(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Output Plate"
	rp, s := d.plateRadius(), d.Chamfer
	requireOneLump(t, label, bodies[0])
	plain := math.Pi * rp * rp * d.PlateT
	if s <= 0 {
		// Chamfer Size 0 means the step does nothing, and the body has to come
		// through untouched rather than merely close to it.
		measuresVolume(t, label, bodies[0], plain, exact())
		return
	}
	// Each rim loses the solid of revolution of a right triangle with legs s,
	// whose centroid sits at rp - s/3: by Pappus, (s^2/2) * 2*pi*(rp - s/3).
	// The evaluator chords the chamfer's conical band, so the slack states that
	// chord error rather than the formula's, which is exact.
	ring := (s * s / 2) * 2 * math.Pi * (rp - s/3)
	measuresVolume(t, label, bodies[0], plain-2*ring, decadtest.WithinRel(units.Scalar(1e-5)))
	measuresBox(t, label, bodies[0],
		r3.NewVec(-rp, -rp, d.stackTop()+1), r3.NewVec(rp, rp, d.stackTop()+1+d.PlateT),
		decadtest.Within(units.Millimeters(1e-2)))
}

// stepChamferPinEnds chamfers the two ends of the `Output Pin` body, the same
// cap-rim helper the disc-like bodies use, before the pattern carries the
// chamfer onto every copy.
func stepChamferPinEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	pin := outputPin(t, doc, d, 0, 0)
	if d.Chamfer <= 0 {
		return []*decad.Body{pin}
	}
	chamfered, err := pin.Chamfer(decad.Edges(decad.Circular()), units.Millimeters(d.Chamfer))
	if err != nil {
		t.Fatalf("chamfer the Output Pin's ends at %.4f mm: %v", d.Chamfer, err)
	}
	return []*decad.Body{chamfered}
}

func assertChamferPinEnds(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	label := "Output Pin"
	r, s := d.DPin/2, d.Chamfer
	requireOneLump(t, label, bodies[0])
	plain := math.Pi * r * r * (d.PlateT + d.stackTop() + 1)
	if s <= 0 {
		measuresVolume(t, label, bodies[0], plain, exact())
		return
	}
	// A chamfer only fits while it stays inside the pin's own radius, which is
	// the pin-end case of the spec's resilient-chamfer rule.
	if s >= r {
		t.Fatalf("Chamfer Size %.4f mm is at or past the output pin's %.4f mm radius", s, r)
	}
	// The same Pappus ring as the plate's rim, at the pin's radius.
	ring := (s * s / 2) * 2 * math.Pi * (r - s/3)
	measuresVolume(t, label, bodies[0], plain-2*ring, decadtest.WithinRel(units.Scalar(1e-5)))
}
