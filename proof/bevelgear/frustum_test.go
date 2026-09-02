package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// HOW THE SOLID STEPS REACH A REVOLVE, A BOOLEAN AND A PATTERN THAT decad
// REFUSES, AND WHAT EACH SUBSTITUTE COSTS.
//
// Four refusals were measured on this evaluator, and each is answered by a
// substitute rather than by dropping the step. The refusals themselves are
// recorded here because they are what the substitutes are for.
//
//  1. A REVOLVE'S READINGS ARE UNPROVEN. doc.Verify returns a revolved body's
//     area and volume with a bound of 200 per cent of the reading — measured
//     with a plain rectangle swept into a solid cylinder, where 785.398 mm^3
//     comes back carrying a bound of 1570.796 — so the document is Suspect and
//     no gate accepts it. SUBSTITUTE: a solid of revolution is the union of the
//     bands its profile edges sweep, and a band between two coaxial sections is
//     a loft, which this evaluator does record exactly. The Gear Body is
//     therefore built as the three bands its three non-axial hexagon edges
//     sweep. COST: the bands are chorded — each circular section is drawn as a
//     regular polygon, because a circular one needs 257 chord cells against the
//     250 its share of the loft's 500-station cap allows — so every volume below
//     is the chorded solid's, and the closed form each is checked against is the
//     chorded one, with the circular value quoted beside it. And the union that
//     makes the three bands one body is not exercised, for the reason in 2.
//
//  2. A BOOLEAN REFUSES A LOFT OPERAND. decad: "tessellation does not support
//     payload decad.loftPayload; supported payload classes are prism, cup, and
//     faceted" — measured for Cut with a prism tool, Cut with a loft tool, and
//     Union of two abutting bands. SUBSTITUTE: draw the boolean's RESULT.
//     stepBoreCut draws the bored band as a loft between two ANNULAR sections
//     rather than cutting a cylinder out of a solid one, and stepCutConicalEnds
//     draws the trimmed tooth as the band between the toe and heel sections
//     rather than splitting the tooth with two cone faces. COST: the boolean
//     itself goes unexercised, so a defect only a boolean would surface — a
//     split that finds no face, a keeper selection that drops the wrong piece —
//     is out of reach; and the conical trim's ends are drawn planar, so the
//     flushness against the frustum's cones is asserted from the cones' own
//     half-angles rather than from the trimmed body's end faces.
//
//  3. THE READ-ONLY INTERFERENCE PASS CANNOT TESSELLATE A LOFT PAYLOAD, so two
//     lofted teeth whose bounding boxes overlap cannot be judged as a pair.
//     SUBSTITUTE: lay them apart. A translation ALONG the shaft axis is what the
//     pattern's own claim is invariant under — rotating about that axis and then
//     sliding along it leaves every azimuth about it unchanged — so the copies
//     are slid apart and every angular assertion still reads the pattern's own
//     geometry. Measured: four teeth rotated by their pattern angles and slid
//     apart verify Sound with no diagnostics. COST: the step does not show that
//     the copies clear each other where they really sit, which is why it asserts
//     the angular gap between neighbours from their own geometry instead.
//
//  4. Loft TAKES TWO SECTIONS AND THERE IS NO SCALE FEATURE. SUBSTITUTE: the
//     spiral chain is built section by section — stepSliceSegments lofts each
//     slab directly rather than splitting one body, stepCrownSegments draws each
//     slab's two sections already scaled about the point scaleFeatures would
//     have scaled them about, and stepLoftSpiralTooth builds the multi-section
//     loft as the chain of bands between consecutive sections. COST is stated at
//     each of those steps.

// bandFacets is how many sides a chorded circular section is drawn with.
//
// A circular section is refused by the loft outright — "loop 0 segment 0 needs
// 257 chord cells to meet the loft chord target, past the 250 its share of the
// 500-station cap allows" — and an annular one has two such loops. Forty-eight
// sides is inside the cap with room for the second loop and leaves the chorded
// area within a quarter of a per cent of the circle's.
const bandFacets = 48

// polygonArea is the area of a regular bandFacets-gon of circumradius r, which
// is the area a chorded section actually encloses.
func polygonArea(r float64) float64 {
	return 0.5 * bandFacets * r * r * math.Sin(2*math.Pi/bandFacets)
}

// frustumVolume is the exact volume between two parallel sections of areas a0
// and a1 whose boundary is ruled straight between them, a distance h apart.
func frustumVolume(h, a0, a1 float64) float64 {
	return math.Abs(h) * (a0 + a1 + math.Sqrt(a0*a1)) / 3
}

// ringSection draws one chorded circular section, with an optional concentric
// hole, on the plane normal to the shaft axis at x.
func ringSection(t *testing.T, world *sketch.World, x, radius, hole float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(x, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("section frame at x %.6f: %v", x, err)
	}
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("section plane at x %.6f: %v", x, err)
	}
	sk, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("section sketch at x %.6f: %v", x, err)
	}
	polygon := func(r float64) {
		pts := make([]*sketch.Point, bandFacets)
		for i := range pts {
			a := 2 * math.Pi * float64(i) / bandFacets
			pts[i] = sk.CreatePoint(r*math.Cos(a), r*math.Sin(a))
		}
		for i := range pts {
			sk.CreateLine(pts[i], pts[(i+1)%bandFacets])
			sk.Fix(pts[i])
		}
	}
	polygon(radius)
	if hole > 0 {
		polygon(hole)
	}
	solve(t, sk)
	// The bored section closes two regions, the ring and the disc inside the
	// bore, and which of the two is larger depends on the bore's size — so the
	// ring is picked by carrying the hole, never by area.
	var outer *sketch.Profile
	for _, p := range sk.Profiles() {
		if hole > 0 {
			if len(p.Holes) == 1 {
				outer = p
			}
			continue
		}
		if outer == nil || p.Area > outer.Area {
			outer = p
		}
	}
	if outer == nil || !outer.Valid {
		t.Fatalf("the section at x %.6f closes no extrudable region with the expected hole count "+
			"(%d regions drawn)", x, len(sk.Profiles()))
	}
	return sk, outer
}

// bandOf lofts one coaxial band between two chorded sections, slid along the
// shaft axis by offset so several of them can be judged in one document.
func bandOf(t *testing.T, doc *decad.Document, world *sketch.World,
	x0, r0, x1, r1, hole, offset float64) *decad.Body {
	t.Helper()
	s0, p0 := ringSection(t, world, x0+offset, r0, hole)
	s1, p1 := ringSection(t, world, x1+offset, r1, hole)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("band from x %.6f r %.6f to x %.6f r %.6f: %v", x0, r0, x1, r1, err)
	}
	return body
}

// sweptBand is one hexagon edge and the band it sweeps.
type sweptBand struct {
	name       string
	x0, r0     float64
	x1, r1     float64
	halfAngle  float64 // the cone half-angle the edge sweeps
	contribute float64 // its signed share of the solid of revolution's volume
}

// sweptBands returns the three bands the frustum profile's non-axial edges
// sweep, in the hexagon's own draw order.
//
// The two remaining edges bound nothing: A->G lies on the shaft axis and G->H
// stands at one x, so each sweeps a surface of zero thickness and contributes
// nothing to the volume. Every other edge sweeps a cone, and the solid of
// revolution is those three cones' signed sum.
func sweptBands(hex []vec2) []sweptBand {
	names := []string{"heel cone", "root cone", "toe cone"}
	out := make([]sweptBand, 0, 3)
	for k, pair := range [][2]vec2{{hex[2], hex[3]}, {hex[3], hex[4]}, {hex[4], hex[5]}} {
		p, q := pair[0], pair[1]
		b := sweptBand{name: names[k], x0: p.X, r0: p.Y, x1: q.X, r1: q.Y}
		b.halfAngle = math.Atan2(math.Abs(q.Y-p.Y), math.Abs(q.X-p.X))
		b.contribute = (q.X - p.X) * (math.Pi / 3) * (p.Y*p.Y + p.Y*q.Y + q.Y*q.Y)
		out = append(out, b)
	}
	return out
}

// stride keeps bodies laid apart far enough that no two bounding boxes meet.
func stride(q pair) float64 { return 100 * q.pitchConeDist }

// stepRevolveGearBody builds the Gear Body: the frustum the hexagon profile
// sweeps in a full turn about its own first edge.
//
// SUBSTITUTE: the three bands the profile's three non-axial edges sweep, laid
// apart along the shaft axis, instead of one revolved body. See refusals 1 and 2
// at the top of this file for why, and what it costs.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->
func stepRevolveGearBody(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	q, _, hex := hexagonOf(t, params)
	world := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, 3)
	for k, b := range sweptBands(hex) {
		bodies = append(bodies, bandOf(t, doc, world, b.x0, b.r0, b.x1, b.r1, 0,
			float64(k)*stride(q)))
	}
	return bodies
}

// assertRevolveGearBody measures the three bands and the solid they compose.
//
// Each band is checked against the exact volume its own two chorded sections
// bound, so a band drawn on the wrong corners misses it. Then the three signed
// contributions are summed and compared with the chorded analogue of the
// hexagon's Pappus volume — the identity that ties the pieces back to the
// revolve they stand in for, and the one thing a wrongly ordered or wrongly
// signed decomposition cannot satisfy.
//
// The cone half-angles are checked here too, on the bodies that carry them
// rather than on the section that produced them: the toe and heel bands are the
// two cone faces the conical trim's face search has to find
// ([PB-FACE-BY-MIDPOINT]), and a band that came out cylindrical or flat would
// leave that search nothing to match.
func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	_, side, hex := hexagonOf(t, params)
	bands := sweptBands(hex)
	if len(bodies) != len(bands) {
		t.Fatalf("%s: the Gear Body came out as %d bands, want the %d its profile's non-axial "+
			"edges sweep", side.label, len(bodies), len(bands))
	}

	chorded := 0.0
	for k, b := range bands {
		volume, err := bodies[k].Volume()
		if err != nil {
			t.Fatalf("%s: %s band volume: %v", side.label, b.name, err)
		}
		want := frustumVolume(b.x1-b.x0, polygonArea(b.r0), polygonArea(b.r1))
		if !closeTo(volume.Value.Mag(), want, 1e-9) {
			t.Fatalf("%s: the %s band measures %.6f, not the %.6f its two sections bound",
				side.label, b.name, volume.Value.Mag(), want)
		}
		if b.halfAngle <= 0 || b.halfAngle >= math.Pi/2 {
			t.Errorf("%s: the %s band's half-angle is %.9f rad, so that edge sweeps a cylinder or "+
				"a flat annulus and the conical cut has no cone face to find",
				side.label, b.name, b.halfAngle)
		}
		sign := 1.0
		if b.x1 < b.x0 {
			sign = -1
		}
		chorded += sign * frustumVolume(b.x1-b.x0, polygonArea(b.r0), polygonArea(b.r1))
	}

	// The chorded analogue of Pappus: the same signed sum with every circular
	// area replaced by its polygon's. The circular value is quoted so the
	// chording's cost is visible rather than hidden.
	circular := pappusVolume(hex)
	ratio := polygonArea(1) / math.Pi
	if !closeTo(math.Abs(chorded), circular*ratio, 1e-9) {
		t.Fatalf("%s: the three bands sum to %.6f, not the %.6f the hexagon's Pappus volume "+
			"%.6f comes to once every section is chorded to %d sides",
			side.label, math.Abs(chorded), circular*ratio, circular, bandFacets)
	}
	if d := math.Abs(math.Abs(chorded)-circular) / circular; d > 3e-3 {
		t.Errorf("%s: chording to %d sides moved the swept volume by %.2f per cent, which is more "+
			"than this substitution is supposed to cost", side.label, bandFacets, 100*d)
	}
}

// stepBoreCut cuts the cylindrical through bore along the shaft axis.
//
// SUBSTITUTE: the cut's RESULT is drawn — each band that the bore passes
// through is built as a loft between two ANNULAR sections, the hole being the
// bore — rather than extruding a cylinder and cutting with it, which decad
// refuses on a loft operand (refusal 2). A band whose own radius never exceeds
// the bore radius is consumed by the cut outright, and is asserted as consumed
// rather than drawn, because an annulus whose hole is wider than its rim closes
// no region.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepBoreCut, assertBoreCut) -->
func stepBoreCut(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	q, side, hex := hexagonOf(t, params)
	world := sketch.NewWorld()
	hole := 0.0
	if q.boreEnable {
		hole = side.bore / 2
	}
	bodies := make([]*decad.Body, 0, 3)
	for k, b := range sweptBands(hex) {
		if hole > 0 && math.Min(b.r0, b.r1) <= hole {
			continue
		}
		bodies = append(bodies, bandOf(t, doc, world, b.x0, b.r0, b.x1, b.r1, hole,
			float64(k)*stride(q)))
	}
	if len(bodies) == 0 {
		t.Fatalf("%s: a bore of radius %.6f consumes every band of the Gear Body", side.label, hole)
	}
	return bodies
}

// assertBoreCut measures the bored bands, on both sides of the Enable Bore
// branch.
//
// Each surviving band has to have lost exactly the cylinder the bore removes
// from it, and each band the bore consumes has to be one the bore really is
// wider than. The resolved diameter is checked against the rule it comes from:
// this gear's own input when non-zero, and its Pitch Diameter / 4 when not.
func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	q, side, hex := hexagonOf(t, params)
	hole := 0.0
	if q.boreEnable {
		hole = side.bore / 2
		given := q.pinionBore
		if side.label == "Driving" {
			given = q.drivingBore
		}
		auto := side.pitchDia / 4
		if !closeTo(side.bore, given, 1e-12) && !closeTo(side.bore, auto, 1e-12) {
			t.Errorf("%s: the bore diameter resolved to %.9f, which is neither the given value "+
				"nor the Pitch Diameter / 4 = %.9f", side.label, side.bore, auto)
		}
	}

	drawn := 0
	for _, b := range sweptBands(hex) {
		if hole > 0 && math.Min(b.r0, b.r1) <= hole {
			// Consumed. The blank tapers to nothing at its toe corner, which sits ON
			// the shaft axis, so a bore of any diameter takes the toe end with it;
			// what would leave nothing at all is a bore wider than the blank ever
			// gets, and NOTHING IN THE SPEC BOUNDS THE BORE AGAINST THE BLANK.
			t.Logf("%s: the bore of radius %.6f consumes the %s band, whose radius runs %.6f to "+
				"%.6f", side.label, hole, b.name, b.r0, b.r1)
			continue
		}
		if drawn >= len(bodies) {
			t.Fatalf("%s: the bore left %d bands, fewer than the bore passes through",
				side.label, len(bodies))
		}
		volume, err := bodies[drawn].Volume()
		if err != nil {
			t.Fatalf("%s: bored %s band volume: %v", side.label, b.name, err)
		}
		solid := frustumVolume(b.x1-b.x0, polygonArea(b.r0), polygonArea(b.r1))
		removed := math.Abs(b.x1-b.x0) * polygonArea(hole)
		if !closeTo(volume.Value.Mag(), solid-removed, 1e-9) {
			t.Fatalf("%s: the bored %s band measures %.6f, not the %.6f left once a bore of "+
				"radius %.6f is taken out of %.6f", side.label, b.name, volume.Value.Mag(),
				solid-removed, hole, solid)
		}
		if hole == 0 && !closeTo(volume.Value.Mag(), solid, 1e-9) {
			t.Fatalf("%s: Enable Bore is unchecked but the %s band lost material",
				side.label, b.name)
		}
		drawn++
	}
	if drawn != len(bodies) {
		t.Fatalf("%s: the bore left %d bands but %d were expected", side.label, len(bodies), drawn)
	}
	if q.boreEnable && hole >= maxRadius(hex) {
		t.Errorf("%s: the bore radius %.9f is at or beyond the blank's widest radius %.9f",
			side.label, hole, maxRadius(hex))
	}
}

// stepMeshRotate applies the driving gear's half-tooth-pitch meshing rotation.
//
// SUBSTITUTE: the rotation is applied to one tooth rather than to the finished,
// joined gear, because no gate here holds a joined gear (refusal 2). What the
// rotation is for survives the substitution exactly: it moves every tooth by
// half a pitch about the shaft axis, and one tooth is where that is visible —
// a symmetric blank could be turned by any angle with nothing to measure.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepMeshRotate, assertMeshRotate) -->
func stepMeshRotate(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	q, side, _ := hexagonOf(t, params)
	seed, _ := loftedTooth(t, doc, params, proxyInvoluteSteps)
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
		units.Degrees(180/side.teeth))
	if err != nil {
		t.Fatalf("%s: meshing rotation: %v", side.label, err)
	}
	slide, err := r3.Translation(r3.NewVec(stride(q), 0, 0))
	if err != nil {
		t.Fatalf("%s: lay-apart translation: %v", side.label, err)
	}
	both, err := turn.Then(slide)
	if err != nil {
		t.Fatalf("%s: compose: %v", side.label, err)
	}
	rotated, err := seed.PlacedCopy(both)
	if err != nil {
		t.Fatalf("%s: rotate the body about its shaft axis: %v", side.label, err)
	}
	return []*decad.Body{seed, rotated}
}

// assertMeshRotate checks the half-pitch.
//
// The rotation is 180 / Teeth Number degrees, which is exactly half of the
// 360 / Teeth Number the pattern steps by, so a driving valley lands where the
// pinion tooth crosses the axial plane. It is applied about the shaft axis, so
// the body's distance from that axis and its volume are both untouched, and the
// lay-apart slide runs along that same axis so it changes neither the azimuth
// nor either of those.
func assertMeshRotate(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	_, side, _ := hexagonOf(t, params)
	if len(bodies) != 2 {
		t.Fatalf("%s: want the body before and after the meshing rotation, got %d",
			side.label, len(bodies))
	}
	before, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("%s: centroid before: %v", side.label, err)
	}
	after, err := bodies[1].Centroid()
	if err != nil {
		t.Fatalf("%s: centroid after: %v", side.label, err)
	}
	want := math.Pi / side.teeth
	got := math.Atan2(after.Value.Z, after.Value.Y) - math.Atan2(before.Value.Z, before.Value.Y)
	if diff := math.Mod(got-want+3*math.Pi, 2*math.Pi) - math.Pi; math.Abs(diff) > 1e-9 {
		t.Errorf("%s: the meshing rotation moved the body by %.9f rad, not the half tooth pitch "+
			"%.9f", side.label, got, want)
	}
	if pitch := 2 * math.Pi / side.teeth; math.Abs(2*want-pitch) > 1e-12 {
		t.Errorf("%s: %.9f rad is not half the %.9f rad tooth pitch", side.label, want, pitch)
	}
	rBefore := math.Hypot(before.Value.Y, before.Value.Z)
	rAfter := math.Hypot(after.Value.Y, after.Value.Z)
	if !closeTo(rAfter, rBefore, 1e-9) {
		t.Errorf("%s: the rotation moved the body from %.9f to %.9f from the shaft axis; it turns "+
			"about that axis", side.label, rBefore, rAfter)
	}
	vBefore, _ := bodies[0].Volume()
	vAfter, _ := bodies[1].Volume()
	if !closeTo(vAfter.Value.Mag(), vBefore.Value.Mag(), 1e-9) {
		t.Errorf("%s: the rotation changed the body's volume from %.6f to %.6f",
			side.label, vBefore.Value.Mag(), vAfter.Value.Mag())
	}
}
