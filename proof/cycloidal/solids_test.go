// Solid steps. Each function is one Fusion feature — an extrude, a pattern, a
// combine, a chamfer — rebuilt in decad and gated by proofkit3d on decad's own
// verdict plus the topology a solid has to have.
//
// Four substitutions run through this file. Each is forced by a boundary the
// solid engine refuses, and each is named again at the step it applies to.
//
//  1. Free-form sections cannot be boolean operands, and a section whose
//     curvature changes sign is refused outright. The rotor lobe inflects and
//     the casing contour is swept, so both are drawn here as polylines through
//     the same sampled points the Fusion sketch feeds to its fitted spline.
//     What that costs: the proof measures a chorded profile, so every area and
//     volume it checks carries the chord's shortfall against the spline, and it
//     is checked with a tolerance that says so. What it keeps: the sample set,
//     the closure, the tiling and every extent are the real ones.
//
//  2. A boolean refuses operands that merely touch — a face-on-face, coplanar
//     or tangent contact is rejected rather than merged. Fusion's Joins here all
//     join bodies that exactly touch: the L lobe sectors along their spokes, the
//     N casing sectors along theirs, the two cam sections at their shared plane,
//     the casing onto the base. Where the proof needs the join's result it
//     overlaps the operands deliberately and subtracts the overlap it introduced;
//     where it needs the tiling, it draws the tiled outline as one closed loop
//     and extrudes once, then checks the volume identity the pattern promises.
//
//  3. decad's Verify judges every pair of live bodies in a document, and a pair
//     it can resolve neither way makes the report Suspect, which fails the gate.
//     So a step that needs a comparison body measures it in its own scratch
//     document and returns only the bodies its own Fusion feature leaves behind.
//
//  4. decad's Cut retires both operands, so Fusion's isKeepToolBodies is
//     modelled by cutting with a duplicate of the tool.
package cycloidal_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// ---- solid case tables ------------------------------------------------

// discSolidCases exercises the rotor disc: both discs of a two-disc stack (so
// the signed eccentricity and the 180-degree clocking are both built), both
// resolutions of Pin Diameter, both ends of the eccentricity range, and the
// smallest pin count the spec allows alongside the default.
var discSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "disc2of2", Params: baseCase(map[string]float64{pDiscCount: 2, pDiscIndex: 1})},
	{Name: "eccentricityNearUndercutLimit", Params: baseCase(map[string]float64{pEccentricity: 2.45})},
	{Name: "pinDiameterOverride", Params: baseCase(map[string]float64{pPinDiameter: 9})},
	{Name: "minimumCounts", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
}

// camSolidCases covers the cam's two branches in both directions: the bore and
// no-bore cross-section, and the one-section and two-section stack.
var camSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "noInputBore", Params: baseCase(map[string]float64{pInputShaftDiameter: 0})},
	{Name: "twoDiscStack", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18,
	})},
	{Name: "twoDiscStackNoBore", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18, pInputShaftDiameter: 0,
	})},
}

// casingSolidCases keeps the pin count modest where the whole ring is built,
// because the contour ring charges one arrangement segment per chord and the
// engine's section budget is fixed.
var casingSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "minimumPinCount", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
	{Name: "twoDiscStack", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18,
	})},
}

// housingJoinCases is casingSolidCases without the two-disc entry. The join of a
// faceted casing ring onto a faceted base is by far the most expensive boolean
// in this proof — minutes at the default pin count — and what the two-disc case
// would add here is only the casing's extent, which S25 and S26 both build and
// measure at two discs. The pin count, which is what actually changes the ring's
// facet count and the contact the join has to resolve, is still swept.
var housingJoinCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "minimumPinCount", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
}

// outputSolidCases covers the output member, including the chamfer branch in
// both directions: a size that cuts, and the zero that means no chamfer at all.
var outputSolidCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "outputPinDiameterOverride", Params: baseCase(map[string]float64{pOutputPinDiameter: 8})},
	{Name: "noChamfer", Params: baseCase(map[string]float64{pChamferSize: 0})},
	{Name: "minimumOutputPinCount", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
	{Name: "twoDiscStack", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18,
	})},
}

// ---- building blocks --------------------------------------------------

func mm(v float64) units.Value { return units.Millimeters(v) }

// buildSketch returns an empty sketch on a plane parallel to XY at height z.
func buildSketch(t *testing.T, z float64) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	plane := w.XY()
	if z != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			t.Fatalf("offset plane at z=%.4f: %v", z, err)
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

// chordLoop draws a closed polyline through points, every vertex fixed. This is
// the chorded stand-in for a fitted spline: decad's boolean and its analytic
// prism path both take sections of lines, circles and arcs only.
func chordLoop(s *sketch.Sketch, points []pt) {
	handles := make([]*sketch.Point, len(points))
	for i, q := range points {
		handles[i] = s.CreatePoint(q.X, q.Y)
		s.Fix(handles[i])
	}
	for i := range handles {
		s.CreateLine(handles[i], handles[(i+1)%len(handles)])
	}
}

// chordChain draws an open polyline and returns its segments.
func chordChain(s *sketch.Sketch, points []pt) []*sketch.Line {
	handles := make([]*sketch.Point, len(points))
	for i, q := range points {
		handles[i] = s.CreatePoint(q.X, q.Y)
		s.Fix(handles[i])
	}
	lines := make([]*sketch.Line, 0, len(handles)-1)
	for i := 0; i < len(handles)-1; i++ {
		lines = append(lines, s.CreateLine(handles[i], handles[i+1]))
	}
	return lines
}

// fixedCircle draws a circle whose centre and radius are both pinned.
func fixedCircle(s *sketch.Sketch, c pt, r float64) *sketch.Circle {
	centre := s.CreatePoint(c.X, c.Y)
	s.Fix(centre)
	circle := s.CreateCircle(centre, r)
	s.AddConstraint(sketch.NewRadius(circle, r))
	return circle
}

// solveRegions solves the sketch and returns the regions it closes.
func solveRegions(t *testing.T, s *sketch.Sketch) []*sketch.Profile {
	t.Helper()
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve build sketch: %v", err)
	}
	regions := s.Profiles()
	if len(regions) == 0 {
		t.Fatal("build sketch closes no region")
	}
	return regions
}

// onlyRegion returns the single region a sketch closes.
func onlyRegion(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	regions := solveRegions(t, s)
	if len(regions) != 1 {
		t.Fatalf("build sketch closes %d regions, want 1", len(regions))
	}
	return regions[0]
}

// holedRegion returns the one region with the given number of hole loops.
func holedRegion(t *testing.T, s *sketch.Sketch, holes int) *sketch.Profile {
	t.Helper()
	var found *sketch.Profile
	for _, region := range solveRegions(t, s) {
		if len(region.Holes) == holes {
			if found != nil {
				t.Fatalf("more than one region has %d hole loop(s)", holes)
			}
			found = region
		}
	}
	if found == nil {
		t.Fatalf("no region has %d hole loop(s)", holes)
	}
	return found
}

// smallestRegionOn returns the smallest-area region whose boundary uses the
// given entity. The casing's solid outer circle makes its contour the shared
// edge of two closed regions — the thin wedge and the whole complement inside
// the circle — and the wedge is the smaller by a wide margin.
func smallestRegionOn(t *testing.T, s *sketch.Sketch, want sketch.Entity) *sketch.Profile {
	t.Helper()
	var found *sketch.Profile
	for _, region := range solveRegions(t, s) {
		for _, e := range region.Entities {
			if e != want {
				continue
			}
			if found == nil || region.Area < found.Area {
				found = region
			}
			break
		}
	}
	if found == nil {
		t.Fatal("no region uses the named entity")
	}
	return found
}

func extrudeUp(t *testing.T, doc *decad.Document, s *sketch.Sketch, region *sketch.Profile,
	depth float64) *decad.Body {
	t.Helper()
	body, err := doc.Extrude(s, region, decad.Distance{D: mm(depth), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude %.4f mm along the sketch normal: %v", depth, err)
	}
	return body
}

func extrudeDown(t *testing.T, doc *decad.Document, s *sketch.Sketch, region *sketch.Profile,
	depth float64) *decad.Body {
	t.Helper()
	body, err := doc.Extrude(s, region, decad.Distance{D: mm(depth), Dir: decad.Against})
	if err != nil {
		t.Fatalf("extrude %.4f mm against the sketch normal: %v", depth, err)
	}
	return body
}

func volumeOf(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("measure volume: %v", err)
	}
	return m.Value.Base()
}

func boundsOf(t *testing.T, b *decad.Body) decad.Box {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("measure bounds: %v", err)
	}
	return box
}

// requireSpan fails unless the body's z extent is exactly the span the step's
// extrude is supposed to produce.
func requireSpan(t *testing.T, b *decad.Body, lo, hi, tol float64) {
	t.Helper()
	box := boundsOf(t, b)
	if !nearly(box.Min.Z, lo, tol) || !nearly(box.Max.Z, hi, tol) {
		t.Errorf("body spans z [%.6f, %.6f], want [%.6f, %.6f]", box.Min.Z, box.Max.Z, lo, hi)
	}
}

// requireVolume fails unless the measured volume matches want within rel.
func requireVolume(t *testing.T, b *decad.Body, want, rel float64, what string) {
	t.Helper()
	got := volumeOf(t, b)
	if !nearly(got, want, math.Abs(want)*rel) {
		t.Errorf("%s volume %.4f mm^3, want %.4f mm^3 (within %.2f%%)", what, got, want, rel*100)
	}
}

// turnAbout is the pattern transform: a rotation by angle about the vertical
// axis through (cx, cy). The basis is written with literal zeros in the z row
// rather than evaluated from Rodrigues, because a z row a few ulps off drops the
// engine's analytic prism path.
func turnAbout(t *testing.T, cx, cy, angle float64) r3.Transform {
	t.Helper()
	sin, cos := math.Sin(angle), math.Cos(angle)
	basis := r3.Basis{
		EX: r3.NewVec(cos, sin, 0),
		EY: r3.NewVec(-sin, cos, 0),
		EZ: r3.NewVec(0, 0, 1),
	}
	tx := cx - (cx*cos - cy*sin)
	ty := cy - (cx*sin + cy*cos)
	turn, err := r3.FromBasis(basis, r3.NewVec(tx, ty, 0))
	if err != nil {
		t.Fatalf("build the pattern turn of %.6f rad: %v", angle, err)
	}
	return turn
}

func liftBy(t *testing.T, dz float64) r3.Transform {
	t.Helper()
	lift, err := r3.Translation(r3.NewVec(0, 0, dz))
	if err != nil {
		t.Fatalf("build a %.4f mm lift: %v", dz, err)
	}
	return lift
}

// sectorPoints is the lobe pie-sector outline: the disc centre, then the lobe's
// sampled points. Closing the loop returns along spoke 2 to the centre, which
// chordLoop does by joining the last point back to the first.
func sectorPoints(d dims) []pt {
	out := make([]pt, 0, 1+len(d.lobeSamples()))
	out = append(out, d.centre())
	return append(out, d.lobeSamples()...)
}

// casingSection draws the Ring Casing section: the solid outer circle, the
// chorded contour over one pin pitch, and the two radial spokes from the
// contour's ends out to that circle. It returns the region the extrude takes —
// the smallest-area one the contour bounds, which is the thin wedge and not the
// large complement the same contour also bounds.
func casingSection(t *testing.T, s *sketch.Sketch, d dims) *sketch.Profile {
	t.Helper()
	half := math.Pi / float64(d.N)
	fixedCircle(s, pt{}, d.outerWall())
	contour := d.contour()
	lines := chordChain(s, contour)
	for i, end := range []pt{contour[0], contour[len(contour)-1]} {
		angle := -half
		if i == 1 {
			angle = half
		}
		inner := s.CreatePoint(end.X, end.Y)
		outer := s.CreatePoint(d.outerWall()*math.Cos(angle), d.outerWall()*math.Sin(angle))
		s.Fix(inner)
		s.Fix(outer)
		s.CreateLine(inner, outer)
	}
	return smallestRegionOn(t, s, lines[0])
}

// ---- S08: extrude the lobe sector -------------------------------------

// stepExtrudeLobeSector extrudes the Rotor Lobe sketch's one closed profile —
// the pie sector bounded by spoke 1, the lobe and spoke 2 — by Disc Thickness,
// as a new body, in the sketch normal's positive direction from plane(d).
func stepExtrudeLobeSector(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.zBase())
	chordLoop(s, sectorPoints(d))
	return []*decad.Body{extrudeUp(t, doc, s, onlyRegion(t, s), d.T)}
}

func assertExtrudeLobeSector(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	body := bodies[0]
	// Disc d spans [z_d, z_d + T], every extrude in the positive direction from
	// its own plane.
	requireSpan(t, body, d.zBase(), d.zBase()+d.T, 1e-6)
	requireVolume(t, body, math.Abs(polygonArea(sectorPoints(d)))*d.T, 1e-6, "lobe sector")
	// The sector's apex is the disc centre and its far edge is the lobe, which
	// reaches the tip radius. Both spoke faces meet at that apex, which is why
	// the disc axis cannot be found by looking for a planar face near it.
	box := boundsOf(t, body)
	c := d.centre()
	reach := math.Max(
		math.Max(math.Abs(box.Max.X-c.X), math.Abs(box.Min.X-c.X)),
		math.Max(math.Abs(box.Max.Y-c.Y), math.Abs(box.Min.Y-c.Y)))
	if reach > d.tipRadius()+1e-6 {
		t.Errorf("sector reaches %.6f mm from the disc centre, past the lobe tip %.6f",
			reach, d.tipRadius())
	}
}

// ---- S10: circular-pattern the lobe sector x L ------------------------

// stepPatternLobeSectors is the pattern feature: the lobe-sector extrude
// repeated L times about the disc axis at Od over 360 degrees. decad expresses
// a pattern as placed copies, and its verification judges every pair of live
// bodies, so the seed sector is what this document holds; the assertion places
// each copy in a document of its own and checks where it lands.
func stepPatternLobeSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.zBase())
	chordLoop(s, sectorPoints(d))
	return []*decad.Body{extrudeUp(t, doc, s, onlyRegion(t, s), d.T)}
}

func assertPatternLobeSectors(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	seed := bodies[0]
	seedVolume := volumeOf(t, seed)
	seedCentroid, err := seed.Centroid()
	if err != nil {
		t.Fatalf("measure the seed sector's centroid: %v", err)
	}
	c := d.centre()
	pitch := 2 * math.Pi / float64(d.L)
	for k := 1; k < d.L; k++ {
		scratch := decad.New()
		s := buildSketch(t, d.zBase())
		chordLoop(s, sectorPoints(d))
		copyBody := extrudeUp(t, scratch, s, onlyRegion(t, s), d.T)
		placed, err := copyBody.Placed(turnAbout(t, c.X, c.Y, pitch*float64(k)))
		if err != nil {
			t.Fatalf("place pattern instance %d: %v", k, err)
		}
		if got := volumeOf(t, placed); !nearly(got, seedVolume, seedVolume*1e-9) {
			t.Errorf("pattern instance %d has volume %.6f, want the seed's %.6f", k, got, seedVolume)
		}
		got, err := placed.Centroid()
		if err != nil {
			t.Fatalf("measure pattern instance %d: %v", k, err)
		}
		sin, cos := math.Sin(pitch*float64(k)), math.Cos(pitch*float64(k))
		wx := c.X + (seedCentroid.Value.X-c.X)*cos - (seedCentroid.Value.Y-c.Y)*sin
		wy := c.Y + (seedCentroid.Value.X-c.X)*sin + (seedCentroid.Value.Y-c.Y)*cos
		if !nearly(got.Value.X, wx, 1e-6) || !nearly(got.Value.Y, wy, 1e-6) ||
			!nearly(got.Value.Z, seedCentroid.Value.Z, 1e-6) {
			t.Errorf("pattern instance %d centroid (%.6f, %.6f, %.6f), want (%.6f, %.6f, %.6f)",
				k, got.Value.X, got.Value.Y, got.Value.Z, wx, wy, seedCentroid.Value.Z)
		}
	}
	// L instances at 360/L degrees is exactly one turn: instance L would land
	// back on the seed, which is what tiles the disc without a gap or an overlap.
	if !nearly(pitch*float64(d.L), 2*math.Pi, 1e-12) {
		t.Errorf("L = %d instances of %.9f rad do not close the turn", d.L, pitch)
	}
}

// ---- S11: join the L sectors into one disc ----------------------------

// stepJoinDiskSectors is the combine that turns disc d's own L sectors into one
// body. The engine refuses a boolean whose operands merely touch, and adjacent
// sectors touch exactly along a spoke face, so the joined result is built here
// as the tiled outline extruded once — which is the same solid, and lets the
// assertion check both halves of what the Join has to deliver: one connected
// lump, and the L sectors' material neither lost nor counted twice.
func stepJoinDiskSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.zBase())
	chordLoop(s, d.lobeOutline())
	return []*decad.Body{extrudeUp(t, doc, s, onlyRegion(t, s), d.T)}
}

func assertJoinDiskSectors(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	disc := bodies[0]
	if got := len(disc.Lumps()); got != 1 {
		t.Errorf("joined disc has %d lumps, want 1 connected body", got)
	}
	requireSpan(t, disc, d.zBase(), d.zBase()+d.T, 1e-6)

	scratch := decad.New()
	s := buildSketch(t, d.zBase())
	chordLoop(s, sectorPoints(d))
	sector := extrudeUp(t, scratch, s, onlyRegion(t, s), d.T)
	want := volumeOf(t, sector) * float64(d.L)
	requireVolume(t, disc, want, 1e-9, "joined disc")

	// The spokes are gone: the joined disc's side is the lobe outline and
	// nothing else, so it has one lateral face per outline chord plus the two
	// caps. L separate sectors would each still carry their two spoke faces.
	if got, want := len(disc.Faces()), len(d.lobeOutline())+2; got != want {
		t.Errorf("joined disc has %d faces, want %d (one per outline chord, plus two caps)",
			got, want)
	}
}

// ---- S13: cut one output hole through the disc ------------------------

// stepCutOutputHole cuts the Output Hole sketch's solid hole through disc d,
// restricted to that disc's body, by Disc Thickness.
func stepCutOutputHole(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	disc := buildDisc(t, doc, d)
	// The tool is grown past both faces of the disc. Fusion's cut is exactly
	// Disc Thickness from the sketch plane and lands flush on both; decad
	// refuses a tool whose cap rests on the target's face, so the proof pierces.
	tool := buildCylinder(t, doc, d.zBase()-1, d.T+2,
		pt{X: d.centre().X + d.Rop, Y: 0}, d.DHole/2)
	cut, err := decad.CutContext(t.Context(), disc, tool)
	if err != nil {
		t.Fatalf("cut the output hole: %v", err)
	}
	return []*decad.Body{cut}
}

func assertCutOutputHole(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	cut := bodies[0]
	scratch := decad.New()
	plain := buildDisc(t, scratch, d)
	want := volumeOf(t, plain) - math.Pi*(d.DHole/2)*(d.DHole/2)*d.T
	requireVolume(t, cut, want, 1e-3, "disc with one output hole")
	if got := len(cut.Lumps()); got != 1 {
		t.Errorf("cut disc has %d lumps, want 1", got)
	}
	requireHolesInsideRim(t, d)
	requireSpan(t, cut, d.zBase(), d.zBase()+d.T, 1e-6)
}

// ---- S14: pattern the output-hole cut x M -----------------------------

// stepPatternOutputHoles is the M-times circular pattern of that cut about the
// disc axis. A chain of M booleans on one lineage is not something the engine
// supports, so the patterned result is built as the disc profile carrying all M
// hole loops and extruded once; the assertion checks the count, the placement
// and the volume the pattern is supposed to leave.
func stepPatternOutputHoles(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.zBase())
	chordLoop(s, d.lobeOutline())
	for _, c := range d.outputHoleCentres() {
		fixedCircle(s, c, d.DHole/2)
	}
	return []*decad.Body{extrudeUp(t, doc, s, holedRegion(t, s, d.M), d.T)}
}

func assertPatternOutputHoles(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	holed := bodies[0]
	scratch := decad.New()
	plain := buildDisc(t, scratch, d)
	want := volumeOf(t, plain) - float64(d.M)*math.Pi*(d.DHole/2)*(d.DHole/2)*d.T
	requireVolume(t, holed, want, 1e-3, "disc with M output holes")
	if got := len(holed.Lumps()); got != 1 {
		t.Errorf("patterned disc has %d lumps, want 1", got)
	}
	requireHolesInsideRim(t, d)
	// Every hole orbits the disc centre Od at the output-pin-circle radius, one
	// per M-th of a turn.
	for i, c := range d.outputHoleCentres() {
		if got := radiusOf(c, d.centre()); !nearly(got, d.Rop, 1e-9) {
			t.Errorf("output hole %d sits %.6f from Od, want Rop %.6f", i, got, d.Rop)
		}
	}
}

// ---- S16: cut the disc centre bore ------------------------------------

// stepCutDiscBore cuts the Disc Bore sketch's solid circle through disc d: the
// cam diameter widened by the running clearance, on the disc centre Od.
func stepCutDiscBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	disc := buildDisc(t, doc, d)
	tool := buildCylinder(t, doc, d.zBase()-1, d.T+2, d.centre(), (d.CBD+d.Clr)/2)
	cut, err := decad.CutContext(t.Context(), disc, tool)
	if err != nil {
		t.Fatalf("cut the disc centre bore: %v", err)
	}
	return []*decad.Body{cut}
}

func assertCutDiscBore(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	bored := bodies[0]
	radius := (d.CBD + d.Clr) / 2
	scratch := decad.New()
	plain := buildDisc(t, scratch, d)
	requireVolume(t, bored, volumeOf(t, plain)-math.Pi*radius*radius*d.T, 1e-3, "bored disc")
	if got := len(bored.Lumps()); got != 1 {
		t.Errorf("bored disc has %d lumps, want 1", got)
	}
	// The bore is concentric with the cam and wider by the clearance, so the
	// running gap is the same all the way round: a bore on O instead of Od
	// would foul the cam on one side.
	if got := radius - d.CBD/2; !nearly(got, d.Clr/2, 1e-12) {
		t.Errorf("radial running gap %.6f, want half the Bearing Clearance %.6f", got, d.Clr/2)
	}
}

// requireHolesInsideRim fails when an output hole reaches past the root circle.
//
// SPEC GAP, recorded here because this is where it bites: the validity table
// checks only Rop < Rv, which keeps the hole CENTRES inside the valley circle
// and says nothing about the holes themselves. A hole is D_hole/2 wide, so the
// real condition is Rop + D_hole/2 < Rv, and a parameter set that satisfies the
// stated check but not this one cuts the rim open — the patterned cut then
// leaves a disc with slots through its valleys instead of holes, which no gate
// in the dialog refuses. It was found by building it: the first minimum-count
// case here had Rop + D_hole/2 = 32.3 mm against Rv = 26.5 mm, and the cut
// removed material the disc did not have.
func requireHolesInsideRim(t *testing.T, d dims) {
	t.Helper()
	if reach := d.Rop + d.DHole/2; reach >= d.Rv {
		t.Errorf("output holes reach %.4f mm from Od, past the root circle at %.4f mm; the "+
			"validity table checks only Rop < Rv and lets this through", reach, d.Rv)
	}
}

// ---- disc helpers -----------------------------------------------------

// buildDisc extrudes the whole rotor disc of disc d: the tiled lobe outline,
// which is what the sector extrude, the L-times pattern and the Join leave.
func buildDisc(t *testing.T, doc *decad.Document, d dims) *decad.Body {
	t.Helper()
	s := buildSketch(t, d.zBase())
	chordLoop(s, d.lobeOutline())
	return extrudeUp(t, doc, s, onlyRegion(t, s), d.T)
}

// buildCylinder extrudes a plain cylinder, the tool a cut is made with.
func buildCylinder(t *testing.T, doc *decad.Document, z, height float64, c pt,
	radius float64) *decad.Body {
	t.Helper()
	s := buildSketch(t, z)
	fixedCircle(s, c, radius)
	return extrudeUp(t, doc, s, onlyRegion(t, s), height)
}

// ---- S18: extrude one eccentric cam section ---------------------------

// stepExtrudeCamSection extrudes the Eccentric Cam {d+1} cross-section from
// plane(d) toward the disk: T + g for every section but the last, so adjacent
// sections abut, and T for the last. With an input shaft the cross-section is
// the two-loop annulus — outer loop the cam outer on Od, hole loop the input
// bore on O — and without one it is the plain cam disc.
func stepExtrudeCamSection(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	return []*decad.Body{buildCamSection(t, doc, d, camSectionDepth(d))}
}

func assertExtrudeCamSection(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	body := bodies[0]
	depth := camSectionDepth(d)
	requireSpan(t, body, d.zBase(), d.zBase()+depth, 1e-6)
	area := math.Pi * (d.CBD / 2) * (d.CBD / 2)
	if d.ISD > 0 {
		area -= math.Pi * (d.ISD / 2) * (d.ISD / 2)
	}
	requireVolume(t, body, area*depth, 2e-3, "cam section")
	// The section that is not the last fills the inter-disc gap as well as its
	// own disc, so the next section starts exactly where it ends.
	if d.D0 < d.D-1 && !nearly(depth, d.T+d.G, 1e-12) {
		t.Errorf("cam section %d is %.4f deep, want T + g = %.4f", d.D0+1, depth, d.T+d.G)
	}
	if d.D0 == d.D-1 && !nearly(depth, d.T, 1e-12) {
		t.Errorf("last cam section is %.4f deep, want T = %.4f", depth, d.T)
	}
}

// ---- S19: join the cam sections into one Eccentric Cam ----------------

// stepJoinCamSections is the combine that makes the D sections one cam.
//
// SPEC DEFECT, recorded here because this is the step that would raise it: for
// D = 1 the spec still says to Join, target section 0 with "the rest" as tools,
// and the rest is empty. CombineFeatures.createInput takes an ObjectCollection
// of one or more tool bodies, so a single-disc build has no Join to make and
// must skip the combine and rename the one section instead. This proof takes
// the skip branch for D = 1 and the real join for D = 2.
//
// For D = 2 the two sections abut on the plane between the discs, and the engine
// refuses a boolean whose operands meet face to face, so the lower section is
// grown by a small overlap and the overlap's volume is taken back out of the
// expected total.
func stepJoinCamSections(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	if d.D == 1 {
		return []*decad.Body{buildCamSection(t, doc, d, camSectionDepth(d))}
	}
	lower := d
	lower.D0, lower.S, lower.Phi = 0, 1, 0
	upper := d
	upper.D0, upper.S, upper.Phi = 1, -1, math.Pi
	// Second substitution, on top of the overlap: the sections are joined here
	// without their bores. Both carry the same bore on the same axis, so inside
	// the overlap the two bore walls coincide exactly, and the engine refuses
	// that tangent contact by name. Cutting the bore through afterwards is
	// refused too — a union leaves a faceted body whose mesh bound is coarser
	// than the tolerance a following cut derives, and the engine will not chain
	// past it. NOT REACHED, therefore: the bore running through the joined cam.
	// What is reached is the bore in a single section, which S18 extrudes and
	// measures, plus the check below that its footprint sits inside both
	// sections, which is what makes the bore continuous once joined.
	first := buildCamSection(t, doc, boreless(lower), camSectionDepth(lower)+camJoinOverlap)
	second := buildCamSection(t, doc, boreless(upper), camSectionDepth(upper))
	joined, err := decad.UnionContext(t.Context(), first, second)
	if err != nil {
		t.Fatalf("join the cam sections: %v", err)
	}
	return []*decad.Body{joined}
}

func assertJoinCamSections(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	cam := bodies[0]
	if got := len(cam.Lumps()); got != 1 {
		t.Errorf("cam has %d lumps, want one continuous solid", got)
	}
	if d.D == 1 {
		// Nothing was joined: the single section is the cam, spanning its disc.
		requireSpan(t, cam, 0, d.T, 1e-6)
		return
	}
	requireSpan(t, cam, 0, d.stackTop(), 1e-6)
	disc := math.Pi * (d.CBD / 2) * (d.CBD / 2)
	want := disc*(d.T+d.G+camJoinOverlap) + disc*d.T - overlapArea(d)*camJoinOverlap
	requireVolume(t, cam, want, 3e-3, "joined cam")
	// The two sections' centres are only 2E apart against a radius of
	// CenterBearingDiameter/2, so they overlap over most of their area, which is
	// what makes the join one solid rather than two touching lumps.
	if overlapArea(d) <= 0.5*disc {
		t.Errorf("the two cam sections share %.2f mm^2 of a %.2f mm^2 section, too little to join",
			overlapArea(d), disc)
	}
	// The input bore is on the drive axis and both sections are on their own
	// eccentric centres, so the bore only runs through the joined cam if its
	// footprint sits inside each section: that is E + ISD/2 < CBD/2, the check
	// the dialog runs.
	if d.ISD > 0 && d.E+d.ISD/2 >= d.CBD/2 {
		t.Errorf("the input bore reaches %.4f mm from a section centre, past the cam radius %.4f",
			d.E+d.ISD/2, d.CBD/2)
	}
}

// camJoinOverlap is the sink the proof introduces so the abutting cam sections
// present a real overlap to the boolean rather than a face-on-face contact.
const camJoinOverlap = 0.1

// camSectionDepth is section d's extent: T + g for every section but the last.
func camSectionDepth(d dims) float64 {
	if d.D0 < d.D-1 {
		return d.T + d.G
	}
	return d.T
}

// buildCamSection extrudes one cam cross-section from plane(d).
func buildCamSection(t *testing.T, doc *decad.Document, d dims, depth float64) *decad.Body {
	t.Helper()
	s := buildSketch(t, d.zBase())
	fixedCircle(s, d.centre(), d.CBD/2)
	if d.ISD > 0 {
		fixedCircle(s, pt{}, d.ISD/2)
		return extrudeUp(t, doc, s, holedRegion(t, s, 1), depth)
	}
	return extrudeUp(t, doc, s, onlyRegion(t, s), depth)
}

// boreless is d with the input bore removed, the cross-section the cam join is
// built from before the bore is cut through the joined solid.
func boreless(d dims) dims {
	d.ISD = 0
	return d
}

// overlapArea is the cross-section the two cam sections share: the lens of two
// circles of the cam radius whose centres are 2E apart.
func overlapArea(d dims) float64 {
	r, gap := d.CBD/2, 2*d.E
	return 2*r*r*math.Acos(gap/(2*r)) - gap/2*math.Sqrt(4*r*r-gap*gap)
}

// ---- S22: extrude the housing base annulus ----------------------------

// stepExtrudeHousingBase extrudes the Housing Ring annulus by Base Thickness in
// the negative direction, away from the disc, from the housing plane 1 mm below
// the disc.
func stepExtrudeHousingBase(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	return []*decad.Body{buildHousingBase(t, doc, d, d.Base, 0)}
}

func assertExtrudeHousingBase(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	base := bodies[0]
	requireSpan(t, base, -1-d.Base, -1, 1e-6)
	area := math.Pi * (d.outerWall()*d.outerWall() - d.innerFloor()*d.innerFloor())
	requireVolume(t, base, area*d.Base, 2e-3, "housing base")
	if got := len(base.Lumps()); got != 1 {
		t.Errorf("housing base has %d lumps, want 1", got)
	}
}

// buildHousingBase extrudes the base annulus downward from the housing plane.
// grow widens its outer circle, which the join step needs and the extrude step
// leaves at zero.
func buildHousingBase(t *testing.T, doc *decad.Document, d dims, depth, grow float64) *decad.Body {
	t.Helper()
	s := buildSketch(t, -1)
	fixedCircle(s, pt{}, d.outerWall()+grow)
	fixedCircle(s, pt{}, d.innerFloor())
	return extrudeDown(t, doc, s, holedRegion(t, s, 1), depth)
}

// ---- S25: extrude the ring casing sector ------------------------------

// stepExtrudeCasingSector extrudes the casing section two-sided: up to the stack
// top in the positive direction, and 1 mm in the negative one so its bottom face
// lands on the housing base's top face and the two can be joined into one part.
func stepExtrudeCasingSector(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, 0)
	region := casingSection(t, s, d)
	body, err := doc.Extrude(s, region, decad.TwoSided{
		One: decad.DistanceSide{D: mm(d.stackTop())},
		Two: decad.DistanceSide{D: mm(1)},
	})
	if err != nil {
		t.Fatalf("extrude the casing sector two-sided: %v", err)
	}
	return []*decad.Body{body}
}

func assertExtrudeCasingSector(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	sector := bodies[0]
	// The negative side matches the housing plane's own -1 mm offset, so the
	// casing bottom is coincident with the base top rather than floating above
	// it, and the positive side reaches the top of the whole disc stack.
	requireSpan(t, sector, -1, d.stackTop(), 1e-6)
	want := (math.Pi*d.outerWall()*d.outerWall() - math.Abs(polygonArea(d.contourRing()))) /
		float64(d.N)
	requireVolume(t, sector, want*(d.stackTop()+1), 5e-3, "casing sector")
}

// ---- S26: pattern the casing sector x N -------------------------------

// stepPatternCasingSectors is the N-times pattern of that sector about the drive
// axis. It is built here as the whole ring in one extrude, for the reason the
// lobe pattern is: the sectors meet face to face, which the boolean refuses.
func stepPatternCasingSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	return []*decad.Body{buildCasingRing(t, doc, d, d.stackTop(), 1)}
}

func assertPatternCasingSectors(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	ring := bodies[0]
	requireSpan(t, ring, -1, d.stackTop(), 1e-6)

	scratch := decad.New()
	s := buildSketch(t, 0)
	region := casingSection(t, s, d)
	sector, err := scratch.Extrude(s, region, decad.TwoSided{
		One: decad.DistanceSide{D: mm(d.stackTop())},
		Two: decad.DistanceSide{D: mm(1)},
	})
	if err != nil {
		t.Fatalf("extrude the comparison sector: %v", err)
	}
	requireVolume(t, ring, volumeOf(t, sector)*float64(d.N), 1e-2, "patterned casing ring")
}

// ---- S27: join the N casing sectors into one casing -------------------

// stepJoinCasingSectors is the combine that makes the N sectors one casing body.
// The sectors share their spoke faces exactly — which is what the contour's ends
// at +/-pi/N buy — so what the join has to deliver is one connected lump whose
// inner wall runs unbroken all the way round, with no seam face left at any
// pitch boundary.
func stepJoinCasingSectors(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	return []*decad.Body{buildCasingRing(t, doc, d, d.stackTop(), 1)}
}

func assertJoinCasingSectors(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	casing := bodies[0]
	if got := len(casing.Lumps()); got != 1 {
		t.Errorf("casing has %d lumps; N sectors that do not touch never join into one", got)
	}
	// One face per inner chord, one outer cylinder, two caps. A spoke face left
	// over at a pitch boundary would show up here as extra faces.
	if got, want := len(casing.Faces()), len(d.contourRing())+3; got != want {
		t.Errorf("joined casing has %d faces, want %d (one per contour chord, the outer wall, "+
			"and two caps)", got, want)
	}
}

// buildCasingRing extrudes the whole casing: the outer circle with the N-fold
// contour ring as its hole loop.
func buildCasingRing(t *testing.T, doc *decad.Document, d dims, up, down float64) *decad.Body {
	t.Helper()
	s := buildSketch(t, 0)
	fixedCircle(s, pt{}, d.outerWall())
	chordLoop(s, d.contourRing())
	body, err := doc.Extrude(s, holedRegion(t, s, 1), decad.TwoSided{
		One: decad.DistanceSide{D: mm(up)},
		Two: decad.DistanceSide{D: mm(down)},
	})
	if err != nil {
		t.Fatalf("extrude the casing ring: %v", err)
	}
	return body
}

// ---- S28: combine the casing and the base into one Housing ------------

// stepJoinHousing is the final combine: the casing joined into the base so the
// housing is one printable part. Their faces meet at z = -1, which the boolean
// refuses, so the base is grown upward by a small overlap and that overlap's
// volume is taken back out of the expected total.
func stepJoinHousing(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	base := buildHousingBase(t, doc, d, d.Base, housingJoinGrow)
	lifted, err := base.Placed(liftBy(t, housingJoinOverlap))
	if err != nil {
		t.Fatalf("sink the base into the casing: %v", err)
	}
	casing := buildCasingRing(t, doc, d, d.stackTop(), 1)
	housing, err := decad.UnionContext(t.Context(), lifted, casing)
	if err != nil {
		t.Fatalf("join the casing into the housing base: %v", err)
	}
	return []*decad.Body{housing}
}

func assertJoinHousing(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := derive(p)
	housing := bodies[0]
	if got := len(housing.Lumps()); got != 1 {
		t.Errorf("housing has %d lumps; the casing and the base did not meet", got)
	}
	// The whole housing spans from the base floor to the stack top.
	requireSpan(t, housing, -1-d.Base+housingJoinOverlap, d.stackTop(), 1e-6)

	scratch := decad.New()
	base := buildHousingBase(t, scratch, d, d.Base, housingJoinGrow)
	baseVolume := volumeOf(t, base)
	other := decad.New()
	casing := buildCasingRing(t, other, d, d.stackTop(), 1)
	casingVolume := volumeOf(t, casing)
	// The casing ring's footprint lies wholly inside the base annulus, so the
	// overlap the proof introduced is that footprint times the sink.
	shared := casingVolume / (d.stackTop() + 1) * housingJoinOverlap
	requireVolume(t, housing, baseVolume+casingVolume-shared, 1e-2, "housing")
}

// housingJoinOverlap sinks the base into the casing so their meeting faces
// present a real overlap, and housingJoinGrow widens the base's outer circle so
// the two outer walls are not the same cylinder. Fusion's base and casing share
// that outer diameter exactly, which is what makes them Join flush; the engine
// reads two coincident cylinders as a tangent contact it cannot classify and
// refuses the union, so the proof separates them by a twentieth of a millimetre
// and carries the difference in the volume it expects.
const (
	housingJoinOverlap = 0.1
	housingJoinGrow    = 0.05
)

// ---- S31: extrude the output plate ------------------------------------

// stepExtrudeOutputPlate extrudes the plate by Output Plate Thickness, away from
// the disk, from the plate plane 1 mm above the stack top. Fusion extrudes every
// profile in the sketch — the plate with the pin's bite taken out of it, plus the
// pin disc itself — so the footprint under each pin is solid plate. decad takes
// one region per extrude, so the proof extrudes the whole plate disc, which is
// what those two regions add up to; the sketch step measures that identity.
func stepExtrudeOutputPlate(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{}, d.plateRadius())
	return []*decad.Body{extrudeUp(t, doc, s, onlyRegion(t, s), d.PlateT)}
}

func assertExtrudeOutputPlate(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	plate := bodies[0]
	// The plate plane sits 1 mm above the top disc, on the opposite side of the
	// stack from the housing, and the plate grows away from the disc from there.
	requireSpan(t, plate, d.stackTop()+1, d.stackTop()+1+d.PlateT, 1e-6)
	requireVolume(t, plate, math.Pi*d.plateRadius()*d.plateRadius()*d.PlateT, 2e-3, "output plate")
}

// ---- S32: extrude the output pin, two-sided ---------------------------

// stepExtrudeOutputPin extrudes the pin disc both ways from the plate plane:
// Output Plate Thickness away from the disk, into the plate, and stackTop + 1 mm
// toward it, which reaches disc 0's bottom face so the pin runs through every
// disc's output holes.
func stepExtrudeOutputPin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	return []*decad.Body{buildOutputPin(t, doc, d, 0)}
}

func assertExtrudeOutputPin(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	pin := bodies[0]
	// The far end lands exactly on z = 0, disc 0's bottom. Short of that and the
	// pin does not reach through the lowest disc's holes.
	requireSpan(t, pin, 0, d.stackTop()+1+d.PlateT, 1e-6)
	length := d.stackTop() + 1 + d.PlateT
	requireVolume(t, pin, math.Pi*(d.DPin/2)*(d.DPin/2)*length, 2e-3, "output pin")
	// The pin is the hole less the orbit clearance, which is what lets it sit in
	// a hole that orbits by E about it.
	if got := d.DHole - d.DPin; !nearly(got, 2*d.E, 1e-9) {
		t.Errorf("the hole is %.6f wider than the pin, want 2E = %.6f", got, 2*d.E)
	}
}

// ---- S33: cut the plate socket, keeping the pin -----------------------

// stepCutOutputSocket is the combine-cut that opens a matching socket in the
// plate with the pin as the tool, keeping the tool body. decad's Cut retires
// both operands, so the tool here is a duplicate of the pin, which is what
// isKeepToolBodies means; and the pin's top cap is coplanar with the plate's,
// which the boolean refuses, so the duplicate is grown past it.
func stepCutOutputSocket(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{}, d.plateRadius())
	plate := extrudeUp(t, doc, s, onlyRegion(t, s), d.PlateT)
	tool := buildOutputPin(t, doc, d, socketToolOversize)
	socketed, err := decad.CutContext(t.Context(), plate, tool)
	if err != nil {
		t.Fatalf("cut the output socket: %v", err)
	}
	return []*decad.Body{socketed}
}

func assertCutOutputSocket(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	socketed := bodies[0]
	plateVolume := math.Pi * d.plateRadius() * d.plateRadius() * d.PlateT
	socket := math.Pi * (d.DPin / 2) * (d.DPin / 2) * d.PlateT
	requireVolume(t, socketed, plateVolume-socket, 3e-3, "plate with one socket")
	if got := len(socketed.Lumps()); got != 1 {
		t.Errorf("socketed plate has %d lumps, want 1", got)
	}
	// The socket is the pin's own footprint, so the pin seats in it: any other
	// diameter would leave the pin loose or refuse to enter.
	scratch := decad.New()
	pin := buildOutputPin(t, scratch, d, 0)
	box := boundsOf(t, pin)
	if got := box.Max.X - box.Min.X; !nearly(got, d.DPin, 1e-3) {
		t.Errorf("pin is %.4f mm across, want the socket's %.4f mm", got, d.DPin)
	}
}

// socketToolOversize grows the cutting tool past the plate's top face. Fusion's
// pin ends flush with it, and a tool whose cap rests on the target's face is a
// contact the boolean refuses rather than cuts.
const socketToolOversize = 0.5

// buildOutputPin extrudes the output pin two-sided from the plate plane,
// optionally grown by extra on the far side for use as a cutting tool.
func buildOutputPin(t *testing.T, doc *decad.Document, d dims, extra float64) *decad.Body {
	t.Helper()
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{X: d.Rop, Y: 0}, d.DPin/2)
	body, err := doc.Extrude(s, onlyRegion(t, s), decad.TwoSided{
		One: decad.DistanceSide{D: mm(d.PlateT + extra)},
		Two: decad.DistanceSide{D: mm(d.stackTop() + 1)},
	})
	if err != nil {
		t.Fatalf("extrude the output pin two-sided: %v", err)
	}
	return body
}

// ---- S34: chamfer the output pin's ends -------------------------------

// stepChamferOutputPinEnds chamfers the pin's two end rims, an equal-distance
// chamfer of Chamfer Size, or leaves the pin alone when the size is zero.
func stepChamferOutputPinEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	pin := buildOutputPin(t, doc, d, 0)
	if d.Cham <= 0 {
		// Chamfer Size 0 means no chamfer at all: the helper returns before it
		// selects an edge, and the build carries on with the plain pin.
		return []*decad.Body{pin}
	}
	chamfered, err := pin.Chamfer(decad.Edges(decad.Circular()).Exactly(2), mm(d.Cham))
	if err != nil {
		t.Fatalf("chamfer the pin's two end rims: %v", err)
	}
	return []*decad.Body{chamfered}
}

func assertChamferOutputPinEnds(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	body := bodies[0]
	radius := d.DPin / 2
	length := d.stackTop() + 1 + d.PlateT
	plain := math.Pi * radius * radius * length
	if d.Cham <= 0 {
		requireVolume(t, body, plain, 2e-3, "unchamfered pin")
		return
	}
	// A 45-degree equal-distance chamfer of size c takes a ring of
	// pi*c^2*(r - c/3) off each end.
	ring := math.Pi * d.Cham * d.Cham * (radius - d.Cham/3)
	requireVolume(t, body, plain-2*ring, 5e-3, "chamfered pin")
	requireSpan(t, body, 0, length, 1e-6)
}

// ---- S35: pattern the pin, socket and chamfer x M ---------------------

// stepPatternOutputPins is the M-times pattern about the drive axis of the pin
// extrude, the socket cut and the pin-end chamfer together. The pattern's
// placement is what this step owns, and the proof takes it on the sockets: the
// plate carrying all M of them, extruded once, because a chain of M booleans on
// one body is not something the engine supports.
func stepPatternOutputPins(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{}, d.plateRadius())
	for _, c := range d.outputPinCentres() {
		fixedCircle(s, c, d.DPin/2)
	}
	return []*decad.Body{extrudeUp(t, doc, s, holedRegion(t, s, d.M), d.PlateT)}
}

func assertPatternOutputPins(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	plate := bodies[0]
	full := math.Pi * d.plateRadius() * d.plateRadius() * d.PlateT
	socket := math.Pi * (d.DPin / 2) * (d.DPin / 2) * d.PlateT
	requireVolume(t, plate, full-float64(d.M)*socket, 3e-3, "plate with M sockets")
	if got := len(plate.Lumps()); got != 1 {
		t.Errorf("plate has %d lumps, want 1", got)
	}
	// The pins are on the drive axis O while the disc's holes are on Od, so each
	// pin sits in its hole offset by the eccentricity, and the plate still
	// covers the outermost pin by Wall.
	for i, c := range d.outputPinCentres() {
		if got := radiusOf(c, pt{}); !nearly(got, d.Rop, 1e-9) {
			t.Errorf("output pin %d sits %.6f from O, want Rop %.6f", i, got, d.Rop)
		}
		if got := d.plateRadius() - (radiusOf(c, pt{}) + d.DPin/2); !nearly(got, d.Wall, 1e-9) {
			t.Errorf("plate covers pin %d by %.6f, want Wall %.6f", i, got, d.Wall)
		}
	}
}

// ---- S36: chamfer the outer rims --------------------------------------

// stepChamferOuterRims chamfers the outer rim of a disc-like body on both flat
// faces. The output plate is the one this proof builds: a uniform-thickness disc
// with exactly two cap faces, both of them axially extreme, which is the shape
// the helper's extreme-cap filter is a no-op for.
//
// NOT REACHED, and recorded here rather than only in the step list: the rotor
// disc's rim, whose outer loop is the lobe contour, and the combined Housing's,
// whose interior ledge at the base-casing junction is the case the extreme-cap
// filter exists for. The engine chamfers a straight prism only, so a Housing
// that is a boolean result cannot be chamfered here at all, and the lobe rim's
// self-intersection at the tight valleys — the failure the spec's resilient
// chamfer catches — is a Fusion behaviour with no counterpart in this engine.
func stepChamferOuterRims(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := derive(p)
	requireInRegime(t, d)
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{}, d.plateRadius())
	plate := extrudeUp(t, doc, s, onlyRegion(t, s), d.PlateT)
	if d.Cham <= 0 {
		return []*decad.Body{plate}
	}
	chamfered, err := plate.Chamfer(decad.Edges(decad.Circular()).Exactly(2), mm(d.Cham))
	if err != nil {
		t.Fatalf("chamfer the plate's two rim loops: %v", err)
	}
	return []*decad.Body{chamfered}
}

func assertChamferOuterRims(t *testing.T, _ *decad.Document, bodies []*decad.Body,
	p map[string]float64) {
	d := derive(p)
	body := bodies[0]
	radius := d.plateRadius()
	plain := math.Pi * radius * radius * d.PlateT
	if d.Cham <= 0 {
		requireVolume(t, body, plain, 2e-3, "unchamfered plate")
		return
	}
	ring := math.Pi * d.Cham * d.Cham * (radius - d.Cham/3)
	requireVolume(t, body, plain-2*ring, 5e-3, "chamfered plate")
	// Only the rim moves: the chamfer takes nothing off the plate's height.
	requireSpan(t, body, d.stackTop()+1, d.stackTop()+1+d.PlateT, 1e-6)
}
