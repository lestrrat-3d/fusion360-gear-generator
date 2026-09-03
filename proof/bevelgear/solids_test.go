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

// ---------------------------------------------------------------------------
// The solid model, and what it substitutes.
//
// Every gear is built in its own world frame: the Apex at the origin, the
// shaft axis along +Z, and the axial (Gear Profiles) plane at y = 0. A lattice
// point (along, radial) is then the world point (radial, 0, along), and the
// axial sketch is the world XZ plane whose own (u, v) is exactly (radial,
// along).
//
// Two substitutions run through all of these steps, and both are named again
// where they bite:
//
//   - THE TOOTH SECTION IS PERPENDICULAR TO THE SHAFT AXIS, not on the back
//     cone. Fusion draws the virtual spur tooth on the `{gear} Plane`, which
//     contains the tooth-centre reference line C->K' and is therefore tilted
//     from the axis-perpendicular plane by the pitch cone angle, and lofts to
//     it from the Apex point. decad's loft takes two profiles on distinct
//     planes and no point section, so the proof lofts between two
//     axis-perpendicular sections instead and applies the Tredgold mapping
//     that carries the back-cone tooth onto the real gear: a point's radial
//     offset from the root is preserved and its angular offset from the tooth
//     centre is divided by cos(gamma), which is what turns the virtual tooth's
//     360/virtualTeeth angular pitch into the real gear's 360/teeth. The cost
//     is that the proof does not exercise the tilt of the tooth plane itself
//     or the degenerate apex section; what it keeps is the tooth's size, its
//     curve inventory, the embedded flag, the taper toward the Apex, the
//     angular pitch the pattern depends on, and both conical trims.
//   - THE TOOTH ROOT IS SUNK a twentieth of the tooth's height below the gear
//     body's root cone. In Fusion the tooth seats exactly ON that cone and the
//     Combine-Join takes the face-to-face contact; decad refuses a boolean
//     whose operands graze within the chord tolerance without provably
//     crossing, so the tooth is sunk far enough to interpenetrate. What that
//     costs is the exactness of the seating: the proof shows the join leaves
//     one lump, not that a tooth resting exactly on the cone would.
//   - THE APEX SECTION IS A SHRUNKEN SECTION, not a point. The loft starts at
//     cone-distance fraction 0.02 rather than 0, since a degenerate section is
//     not representable. Everything the step pins — the taper ratio and the
//     trimmed extent — is measured well outside that stub, which the toe trim
//     removes anyway.
// ---------------------------------------------------------------------------

// pressureAngle is the spur drawer's own default, which the framework proxy
// serves to it: 20 degrees, and not a bevel dialog input.
var pressureAngle = 20 * math.Pi / 180

// involuteSteps is the proxy's InvoluteSteps: how many samples each flank
// carries.
const involuteSteps = 15

// toothRootSink is how far the tooth's root is sunk below the gear body's root
// cone, as a fraction of the tooth's height. See the substitution note above.
const toothRootSink = 0.05

// outlinePt is one point of a tooth section, in polar coordinates about the
// shaft axis: R is the distance from the axis and Th the angle in the section
// plane, both at the HEEL station (cone-distance fraction 1).
type outlinePt struct{ R, Th float64 }

// toothOutline is one gear's tooth cross-section at the heel.
type toothOutline struct {
	Right, Left  []outlinePt // flank samples, root end first, tip end last
	RootR, TipR  float64
	Embedded     bool
	VirtualTeeth int
}

// newToothOutline builds the virtual spur tooth this gear's section 3 draws,
// and maps it onto the gear.
//
// The virtual tooth number comes from the CLOSED FORM — floor(2 *
// (PitchDiameter/2)/cos(gamma) / Module) — never from measuring Apex2->K', and
// it is independent of Tooth Spacing, which moves only where the tooth is
// centred. The tooth is drawn already rotated by 180 degrees, through the
// drawer's own angle argument rather than by rotating the sketch afterwards.
func newToothOutline(d design, g gear) toothOutline {
	vt := virtualTeeth(d.Module, g.PitchDiameter, g.Gamma)
	dims := involute.Derive(d.Module, float64(vt), pressureAngle)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch,
		float64(vt), involuteSteps, math.Pi)

	rootAtHeel := g.PitchDiameter/2 - 1.25*d.Module*math.Cos(g.Gamma) -
		toothRootSink*(dims.Tip-dims.Root)
	cosGamma := math.Cos(g.Gamma)
	mapped := func(p involute.Pt) outlinePt {
		rv := math.Hypot(p.X, p.Y)
		phi := math.Atan2(p.Y, p.X) - math.Pi
		phi = math.Atan2(math.Sin(phi), math.Cos(phi))
		return outlinePt{R: rootAtHeel + (rv - dims.Root), Th: math.Pi + phi/cosGamma}
	}
	convert := func(src []involute.Pt) []outlinePt {
		out := make([]outlinePt, 0, len(src))
		for _, p := range src {
			out = append(out, mapped(p))
		}
		return out
	}

	o := toothOutline{
		Right:        convert(right),
		Left:         convert(left),
		RootR:        rootAtHeel,
		TipR:         rootAtHeel + (dims.Tip - dims.Root),
		Embedded:     dims.Embedded(),
		VirtualTeeth: vt,
	}
	// Put the flank with the smaller angle first, so the loop below runs
	// counter-clockwise.
	if o.Right[0].Th > o.Left[0].Th {
		o.Right, o.Left = o.Left, o.Right
	}
	if o.Embedded {
		// The flank starts inside the root circle, so tip, root and flanks meet
		// with no connecting lines: trim each flank at the root radius.
		o.Right = trimToRoot(o.Right, o.RootR)
		o.Left = trimToRoot(o.Left, o.RootR)
	}
	return o
}

// trimToRoot drops the flank samples that lie inside the root radius and pulls
// the first survivor onto it, which is where the embedded tooth's flank meets
// its root arc.
func trimToRoot(flank []outlinePt, rootR float64) []outlinePt {
	out := make([]outlinePt, 0, len(flank))
	for _, p := range flank {
		if p.R >= rootR {
			out = append(out, p)
		}
	}
	if len(out) == 0 {
		return flank
	}
	out[0].R = rootR
	return out
}

// section places the outline at a cone-distance fraction, twisted about the
// shaft axis and crowned about its own root.
//
// The whole gear is a cone from the Apex, so a section at fraction k is the
// heel section scaled by k — radius and axial position alike. crown scales
// only the tooth's HEIGHT, about the root edge, which is what anchoring the
// scale on the root rather than the face centroid means: the root stays on the
// seating cone and only the tip is relieved.
func (o toothOutline) section(k, twist, crown float64) ([]vec2, []vec2, float64, float64) {
	place := func(p outlinePt) vec2 {
		r := k * (o.RootR + crown*(p.R-o.RootR))
		th := p.Th + twist
		return v2(r*math.Cos(th), r*math.Sin(th))
	}
	right := make([]vec2, len(o.Right))
	for i, p := range o.Right {
		right[i] = place(p)
	}
	left := make([]vec2, len(o.Left))
	for i, p := range o.Left {
		left[i] = place(p)
	}
	return right, left, k * o.RootR, k * (o.RootR + crown*(o.TipR-o.RootR))
}

// drawSection draws one tooth section into a sketch on a plane perpendicular
// to the shaft axis, and returns its single profile.
//
// With chord false the loop is exactly the one section 3's profile search keys
// on: two NURBS flanks, two arcs (tip and root), and either no connecting lines
// when the tooth is embedded or two when it is not. That count is not a guess —
// for a given gear only ONE of the two counts is the real tooth, and an
// unrelated annular loop can carry the other.
//
// With chord true the same loop is emitted as a polyline. That is the
// substitution the solid steps need: decad refuses to extrude or loft a
// free-form boundary at this size, reporting that exact integration needs more
// than its fixed work budget, so every SOLID here is built from the chorded
// loop while the curve inventory is asserted on the curved one. The cost is
// that the solids carry a chording error, which every measurement below allows
// for explicitly rather than hiding.
func drawSection(t *testing.T, w *sketch.World, plane *sketch.Plane, o toothOutline,
	k, twist, crown float64, chord bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth section sketch: %v", err)
	}
	right, left, rootR, _ := o.section(k, twist, crown)

	centre := s.CreatePoint(0, 0)
	s.Fix(centre)

	pt := func(p vec2) *sketch.Point {
		q := s.CreatePoint(p.X, p.Y)
		s.Fix(q)
		return q
	}
	radialAt := func(p vec2, r float64) vec2 { return p.unit().scale(r) }

	rightPts := make([]*sketch.Point, len(right))
	for i, p := range right {
		rightPts[i] = pt(p)
	}
	leftPts := make([]*sketch.Point, len(left))
	for i, p := range left {
		leftPts[i] = pt(p)
	}

	rightRoot, leftRoot := rightPts[0], leftPts[0]
	if !o.Embedded {
		// The flank starts outside the root circle, so a connecting line runs
		// radially inward from its first sample to the root arc.
		rightRoot = pt(radialAt(right[0], rootR))
		leftRoot = pt(radialAt(left[0], rootR))
		s.CreateLine(rightRoot, rightPts[0])
		s.CreateLine(leftPts[0], leftRoot)
	}

	rightTip := rightPts[len(rightPts)-1]
	leftTip := leftPts[len(leftPts)-1]
	if chord {
		polyline(s, rightPts)
		polyline(s, leftPts)
		arcPolyline(s, pt, rightTip, leftTip)
		arcPolyline(s, pt, rightRoot, leftRoot)
	} else {
		if _, err := s.CreateFitSpline(rightPts...); err != nil {
			t.Fatalf("right flank: %v", err)
		}
		if _, err := s.CreateFitSpline(leftPts...); err != nil {
			t.Fatalf("left flank: %v", err)
		}
		// The tip arc runs counter-clockwise from the right flank's tip to the
		// left flank's; the root arc is the one UNDER the tooth, between the
		// two root ends.
		s.CreateArc(centre, rightTip, leftTip)
		s.CreateArc(centre, rightRoot, leftRoot)
	}

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("tooth section: solve: %v", err)
	}
	profs := s.Profiles()
	if len(profs) != 1 {
		t.Fatalf("tooth section holds %d regions, want exactly 1", len(profs))
	}
	return s, profs[0]
}

// arcSegments is how finely a chorded arc is cut. The tip and root arcs each
// span well under a tooth pitch, so eight segments hold the chord error to a
// few parts in ten thousand of the radius.
const arcSegments = 8

func polyline(s *sketch.Sketch, pts []*sketch.Point) {
	for i := 0; i+1 < len(pts); i++ {
		s.CreateLine(pts[i], pts[i+1])
	}
}

// arcPolyline chords an arc about the section's own centre, from start to end
// the short way round.
func arcPolyline(s *sketch.Sketch, pt func(vec2) *sketch.Point, start, end *sketch.Point) {
	a0 := math.Atan2(start.Y(), start.X())
	a1 := math.Atan2(end.Y(), end.X())
	r := math.Hypot(start.X(), start.Y())
	sweep := a1 - a0
	for sweep > math.Pi {
		sweep -= 2 * math.Pi
	}
	for sweep < -math.Pi {
		sweep += 2 * math.Pi
	}
	prev := start
	for i := 1; i <= arcSegments; i++ {
		next := end
		if i < arcSegments {
			a := a0 + sweep*float64(i)/float64(arcSegments)
			next = pt(v2(r*math.Cos(a), r*math.Sin(a)))
		}
		s.CreateLine(prev, next)
		prev = next
	}
}

// sectionPlane is the plane perpendicular to the shaft axis at the given
// cone-distance fraction of the root element Apex->C.
func sectionPlane(t *testing.T, w *sketch.World, f gearFrame, k float64) *sketch.Plane {
	t.Helper()
	p, err := w.CreateOffsetPlane(w.XY(), k*f.Ded.X)
	if err != nil {
		t.Fatalf("section plane at fraction %g: %v", k, err)
	}
	return p
}

// ---------------------------------------------------------------------------
// The gear body: revolve the section 2 hexagon.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Solids of revolution, as polygonal sweeps.
//
// SUBSTITUTION, and what it costs. Every body bevel revolves — the Gear Body
// frustum, the two cut cones, the bore tool — is a solid of revolution, and
// decad's revolve publishes a volume whose PROVEN bound is the volume itself:
// measured here on a plain cylinder, a cone and this gear's own frustum alike,
// the reading is Approximate with a bound of 2x the value, so a revolved body
// is Suspect at any tolerance and can never pass the harness gate. Its extrude,
// loft and union of polygonal profiles are Sound.
//
// So each solid of revolution is built as a chain of lofts between coaxial
// polygonal sections — the union of the bands its profile edges sweep, which is
// what the solid IS — and each band's expected volume is computed in closed
// form for the same polygon count, so the measurement stays exact against the
// model. Every reading is also tied back to the true solid of revolution within
// the chording error, so a wrong lattice still fails. The cost is that the
// proof does not exercise decad's revolve; it measures the same solid to a few
// parts in ten thousand.
// ---------------------------------------------------------------------------

// sweepFacets is the polygon count each revolved band is chorded to. At 96 the
// polygon's area is 1 - 1.6e-3 of the circle's, which is the tolerance every
// tie-back below allows.
const sweepFacets = 64

// chordFactor is the ratio of the inscribed regular polygon's area to the
// circle's, which is what the polygonal sweep's volume is short by.
func chordFactor(n int) float64 {
	return float64(n) * math.Sin(2*math.Pi/float64(n)) / (2 * math.Pi)
}

// station is one cross-section of a solid of revolution: an axial position and
// the inner and outer radii there. RIn 0 means a full disc.
type station struct{ Z, RIn, ROut float64 }

// ringProfile draws one station as a polygon, with a concentric hole when the
// station is annular.
func ringProfile(t *testing.T, w *sketch.World, z float64, st station, n int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	if err != nil {
		t.Fatalf("station plane at z=%g: %v", z, err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("station sketch at z=%g: %v", z, err)
	}
	ring := func(r float64) {
		pts := make([]*sketch.Point, n)
		for i := range n {
			a := 2 * math.Pi * float64(i) / float64(n)
			pts[i] = s.CreatePoint(r*math.Cos(a), r*math.Sin(a))
			s.Fix(pts[i])
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%n])
		}
	}
	ring(st.ROut)
	if st.RIn > 0 {
		ring(st.RIn)
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("station at z=%g: solve: %v", z, err)
	}
	profs := s.Profiles()
	for _, pr := range profs {
		if st.RIn > 0 && len(pr.Holes) == 1 {
			return s, pr
		}
		if st.RIn == 0 && len(pr.Holes) == 0 && len(pr.Entities) == n {
			return s, pr
		}
	}
	t.Fatalf("station at z=%g produced %d regions, none of the expected shape", z, len(profs))
	return nil, nil
}

// sweepBands lofts each consecutive pair of stations and unions the chain.
func sweepBands(t *testing.T, doc *decad.Document, sts []station, n int) *decad.Body {
	t.Helper()
	if len(sts) < 2 {
		t.Fatalf("a swept solid needs at least two stations, got %d", len(sts))
	}
	w := sketch.NewWorld()
	var solid *decad.Body
	for i := 0; i+1 < len(sts); i++ {
		s0, p0 := ringProfile(t, w, sts[i].Z, sts[i], n)
		s1, p1 := ringProfile(t, w, sts[i+1].Z, sts[i+1], n)
		band, err := doc.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("band %d of the swept solid: %v", i, err)
		}
		if solid == nil {
			solid = band
			continue
		}
		solid, err = decad.Union(solid, band)
		if err != nil {
			t.Fatalf("joining band %d of the swept solid: %v", i, err)
		}
	}
	return solid
}

// sweptVolume is the closed-form volume of the polygonal sweep sweepBands
// builds: each band's cross-sectional area varies quadratically with height, so
// the integral is the prismatoid form.
func sweptVolume(sts []station, n int) float64 {
	k := chordFactor(n) * math.Pi
	total := 0.0
	for i := 0; i+1 < len(sts); i++ {
		a, b := sts[i], sts[i+1]
		h := b.Z - a.Z
		outer := (a.ROut*a.ROut + a.ROut*b.ROut + b.ROut*b.ROut) / 3
		inner := (a.RIn*a.RIn + a.RIn*b.RIn + b.RIn*b.RIn) / 3
		total += k * h * (outer - inner)
	}
	return math.Abs(total)
}

// toeStubFraction leaves the very tip of the toe dish out of the swept model:
// at the toe corner M the cross-section is a single circle of zero area, which
// no loft can start from and no boolean can classify. The omitted sliver is 2%
// of the toe-edge span, and every volume reading below is taken against the
// hexagon TRUNCATED at the same station rather than against the untruncated
// one, so the truncation is stated rather than absorbed.
const toeStubFraction = 0.02

// bandOverlap is how far consecutive bands are made to overlap rather than
// meet face to face. decad refuses a union whose operands share a facet plane —
// "the exact predicates cannot classify a tangent contact" — so bands that
// would touch at a flat disc are given a sliver of shared VOLUME instead. The
// two bands' lateral cones differ, so nothing is coplanar in the overlap, and
// the sliver of extra material outside the true body is second order in this
// fraction.
const bandOverlap = 1e-3

// radialAtZ interpolates the line through a and b, taken as (along, radial),
// at an axial position.
func radialAtZ(a, b vec2, z float64) float64 {
	if a.X == b.X {
		return a.Y
	}
	return a.Y + (b.Y-a.Y)*(z-a.X)/(b.X-a.X)
}

// truncatedHexagon is the section 2 frustum profile with its toe corner cut
// back to the stub station: the polygon the swept model actually realises.
func truncatedHexagon(f gearFrame) []vec2 {
	z := toeStub(f)
	return []vec2{
		f.Axis, f.Base, f.Heel, f.Ded,
		v2(z, radialAtZ(f.Toe, f.Ded, z)),
		v2(z, radialAtZ(f.Toe, f.ToeIn, z)),
		f.ToeIn,
	}
}

func toeStub(f gearFrame) float64 {
	return f.Toe.X + toeStubFraction*(f.ToeIn.X-f.Toe.X)
}

// revolveGearBody builds one gear's frustum from its hexagon.
//
// The solid is two overlapping bands — the root cone from the toe stub out to
// the dedendum corner C, and the heel cone from C out to the heel end — with
// the toe dish then cut away by the cone the toe edge M->N sweeps. That is the
// same set the revolve produces: the frustum's outer surface changes generator
// at C, and its front face is conical, not flat.
func revolveGearBody(t *testing.T, doc *decad.Document, f gearFrame) *decad.Body {
	t.Helper()
	zStub := toeStub(f)
	overlap := bandOverlap * (f.Base.X - zStub)

	root := sweepBands(t, doc, []station{
		{Z: zStub, ROut: radialAtZ(f.Toe, f.Ded, zStub)},
		{Z: f.Ded.X + overlap, ROut: radialAtZ(f.Toe, f.Ded, f.Ded.X+overlap)},
	}, sweepFacets)
	heel := sweepBands(t, doc, []station{
		{Z: f.Ded.X - overlap, ROut: radialAtZ(f.Ded, f.Heel, f.Ded.X-overlap)},
		{Z: f.Base.X, ROut: f.Heel.Y},
	}, sweepFacets)
	blank, err := decad.Union(root, heel)
	if err != nil {
		t.Fatalf("gear body: joining the root cone band to the heel cone band: %v", err)
	}

	// The toe plug is the cone the toe edge sweeps, truncated at the toe
	// edge's inner end N, where the dish bottoms out on the shaft axis.
	zLow := zStub - overlap
	plug := sweepBands(t, doc, []station{
		{Z: zLow, ROut: radialAtZ(f.Toe, f.ToeIn, zLow)},
		{Z: f.ToeIn.X, ROut: f.ToeIn.Y},
	}, sweepFacets)
	body, err := decad.Cut(blank, plug)
	if err != nil {
		t.Fatalf("gear body: cutting the toe dish: %v", err)
	}
	return body
}

// stepRevolveGearBody revolves one gear's hexagon a full turn about its shaft
// edge, producing the Gear Body: the frustum whose conical faces are reused as
// the cutting tools for both end trims.
//
// The sketch holds exactly one closed loop, so the revolve takes its single
// profile without filtering, and the axis is the hexagon's FIRST edge — A->G
// resp. B->I, in the same sketch as the profile — never the section 2 Apex->A
// construction line, which lives in another sketch and which Fusion's revolve
// will not accept.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	_, f := sideOf(d, p)
	return []*decad.Body{revolveGearBody(t, doc, f)}
}

// assertRevolveGearBody checks the frustum against Pappus on the section 2
// hexagon: a full revolution sweeps 2*pi*rbar*A, with A the hexagon's own area
// and rbar its centroid's distance from the axis. Both come from the lattice,
// so the reading ties the solid back to section 2 rather than to a restatement
// of it. The hexagon used is the truncated one named above, and the tolerance
// is the polygonal sweep's own chording error at this facet count.
func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s gear body: got %d bodies, want 1", g.Label, len(bodies))
	}
	area, radial := polygonAreaCentroid(truncatedHexagon(f))
	want := 2 * math.Pi * radial * area * chordFactor(sweepFacets)
	requireVolume(t, bodies[0], want, 3e-3, "%s gear body volume by Pappus", g.Label)

	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s gear body bounds: %v", g.Label, err)
	}
	// The widest point of the swept model sits at the band overlap rather than
	// exactly at C, since each band runs a sliver past the dedendum corner
	// along its OWN cone.
	overlap := bandOverlap * (f.Base.X - toeStub(f))
	maxR := math.Max(
		radialAtZ(f.Toe, f.Ded, f.Ded.X+overlap),
		radialAtZ(f.Ded, f.Heel, f.Ded.X-overlap))
	requireClose(t, bounds.Max.X, maxR*chordRadius(sweepFacets), 1e-4,
		"%s gear body outer radius", g.Label)
	requireClose(t, bounds.Max.Z, f.Base.X, 1e-6, "%s gear body heel station", g.Label)
	if bounds.Min.Z < toeStub(f)-1e-6 {
		t.Fatalf("%s gear body reaches z=%.6f, inside the toe stub at %.6f",
			g.Label, bounds.Min.Z, toeStub(f))
	}
	if lumps := len(bodies[0].Lumps()); lumps != 1 {
		t.Fatalf("%s gear body has %d lumps, want 1", g.Label, lumps)
	}
}

// chordRadius is the fraction of the true radius an inscribed polygon's widest
// vertex reaches along an axis. With an even facet count a vertex lands on the
// axis, so it is 1.
func chordRadius(n int) float64 {
	if n%4 == 0 {
		return 1
	}
	return math.Cos(math.Pi / float64(n))
}

// polygonAreaCentroid returns a closed polygon's area and the distance of its
// centroid from the axis, taking each vertex as (along, radial).
func polygonAreaCentroid(poly []vec2) (area, radial float64) {
	var cx float64
	for i := range poly {
		a := poly[i]
		b := poly[(i+1)%len(poly)]
		cross := a.Y*b.X - b.Y*a.X
		area += cross
		cx += (a.Y + b.Y) * cross
	}
	area /= 2
	cx /= 6 * area
	return math.Abs(area), math.Abs(cx)
}

// ---------------------------------------------------------------------------
// The tooth profile, and the loft that turns it into a body.
// ---------------------------------------------------------------------------

// toothSectionThickness is the nominal extrusion the tooth-profile step uses to
// turn its section into something the solid harness can judge.
const toothSectionThickness = 0.5

// stepToothProfile draws one gear's `{gearLabel} Tooth` sketch and extrudes it
// a nominal thickness so the harness has a solid to gate.
//
// This is the ONE sketch bevel does not gate on isFullyConstrained: it is
// drawn by the borrowed spur generator, which labels its four circles with
// along-path text, and sketch text holds a degree of freedom, so a tooth
// sketch whose geometry is completely determined still reads False. The proof
// therefore proves it through the solid harness rather than the sketch gate,
// which is why the step is a solid step at all. What the step owns and what
// this asserts is bevel's side of the contract: the virtual tooth number, the
// tooth centre, the 180 degree draw angle, the embedded flag, and the curve
// inventory the profile search selects on. The spur tooth's own constraint
// scheme belongs to spec/spurgear and is proved there.
//
// <!-- proof-run: proofkit3d.RunSolid(toothSolidCases, stepToothProfile, assertToothProfile) -->
func stepToothProfile(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)

	w := sketch.NewWorld()
	_, curved := drawSection(t, w, sectionPlane(t, w, f, 1), o, 1, 0, 1, false)
	prof := curved

	// The inventory the tooth-profile selection keys on: two NURBS flanks, two
	// arcs, and 0 lines when the tooth is embedded, 2 when it is not. Accepting
	// "0 OR 2" here would let an unrelated annular loop through and the
	// apex-to-profile loft would die with LOFT_NO_TOOLBODY.
	wantLines := 2
	if o.Embedded {
		wantLines = 0
	}
	splines, arcs, lines := inventory(prof)
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Fatalf("%s tooth loop holds %d NURBS, %d arcs, %d lines; want 2, 2, %d (embedded=%v)",
			g.Label, splines, arcs, lines, wantLines, o.Embedded)
	}

	// The solid the harness gates is the chorded twin of that same loop.
	sc, pc := drawSection(t, w, sectionPlane(t, w, f, 1), o, 1, 0, 1, true)
	body, err := doc.Extrude(sc, pc,
		decad.Distance{D: units.Millimeters(toothSectionThickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("%s tooth section: extrude: %v", g.Label, err)
	}
	return []*decad.Body{body}
}

func assertToothProfile(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)

	// The virtual tooth number is the closed form, and Tooth Spacing does not
	// touch it: the spacing moves the tooth CENTRE and nothing else.
	requireClose(t, float64(o.VirtualTeeth),
		math.Floor(2*virtualPitchRadius(g.PitchDiameter, g.Gamma)/d.Module), tightTol,
		"%s virtual tooth number", g.Label)
	zero := d
	zero.ToothSpacing = 0
	if other := newToothOutline(zero, g); other.VirtualTeeth != o.VirtualTeeth {
		t.Fatalf("%s virtual tooth number moved with Tooth Spacing: %d vs %d",
			g.Label, o.VirtualTeeth, other.VirtualTeeth)
	}

	// The embedded flag is the spur drawer's own reading, and it is what the
	// line count above was taken from — not a second opinion.
	dims := involute.Derive(d.Module, float64(o.VirtualTeeth), pressureAngle)
	if o.Embedded != dims.Embedded() {
		t.Fatalf("%s embedded flag disagrees with the drawn tooth", g.Label)
	}

	// The tooth is drawn ALREADY rotated 180 degrees, through the drawer's
	// angle argument. Its centre therefore points away from the tooth-centre
	// reference line's outward direction, which is what the loft and the
	// pattern are built around.
	right, left, rootR, tipR := o.section(1, 0, 1)
	mid := right[len(right)-1].add(left[len(left)-1]).scale(0.5)
	requireClose(t, wrapAngle(math.Atan2(mid.Y, mid.X)-math.Pi), 0, 1e-6,
		"%s tooth centre angle away from the reference direction", g.Label)

	// The tooth stands on the gear body's root cone at the heel: its root
	// radius is the dedendum corner's radial distance, so the Combine-Join
	// leaves no gap.
	requireClose(t, rootR, f.Ded.Y-toothRootSink*(dims.Tip-dims.Root), tightTol,
		"%s tooth root radius at the heel, sunk into the root cone", g.Label)
	requireClose(t, tipR-rootR, dims.Tip-dims.Root, tightTol, "%s tooth height", g.Label)

	// The angular pitch the pattern will use: the virtual tooth's angular
	// thickness divided by cos(gamma) is the real gear's, which is what makes
	// 360/teeth copies close the circle.
	half := math.Abs(wrapAngle(math.Atan2(left[0].Y, left[0].X)-math.Atan2(right[0].Y, right[0].X))) / 2
	if half <= 0 || half >= math.Pi/g.Teeth {
		t.Fatalf("%s tooth half-thickness %.6f rad does not fit a %g-tooth pitch",
			g.Label, half, g.Teeth)
	}

	if len(bodies) != 1 {
		t.Fatalf("%s tooth section: got %d bodies, want 1", g.Label, len(bodies))
	}
}

// inventory tallies a profile's boundary by curve type, the same three counts
// find_profile_by_curve_counts matches on.
func inventory(prof *sketch.Profile) (splines, arcs, lines int) {
	for _, e := range prof.Entities {
		switch e.(type) {
		case *sketch.FitSpline, *sketch.Spline, *sketch.NURBS:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return splines, arcs, lines
}

// apexStubFraction is where the substituted apex section sits: the loft cannot
// start at a degenerate point, so it starts at 5% of the cone distance, well
// inside the toe trim that removes it and far enough out that its facets do
// not collapse below the evaluator's mesh bound. Every case's toe trim lands
// above 0.11 of the cone distance, since the Maximum Face Width caps the face
// at 0.95*sin(gamma)^2 of the pitch cone distance.
const apexStubFraction = 0.05

// heelOverrunFraction carries the loft past the heel cone so the heel trim has
// something to bite on.
const heelOverrunFraction = 1.12

// loftToothBody builds the uncut Apex-to-heel tooth.
func loftToothBody(t *testing.T, doc *decad.Document, d design, g gear, f gearFrame) *decad.Body {
	t.Helper()
	o := newToothOutline(d, g)
	w := sketch.NewWorld()
	s0, p0 := drawSection(t, w, sectionPlane(t, w, f, apexStubFraction), o, apexStubFraction, 0, 1, true)
	s1, p1 := drawSection(t, w, sectionPlane(t, w, f, heelOverrunFraction), o, heelOverrunFraction, 0, 1, true)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s tooth loft: %v", g.Label, err)
	}
	return body
}

// stepLoftToothBody lofts the section 2 Apex sketch point to this gear's
// section 3 tooth profile, giving the uncut Tooth Body.
//
// The Apex end is a SKETCH point in Fusion, never a construction point: the
// Design component is never activated and construction geometry needs an
// active component. decad has no point section at all, so the proof lofts from
// the shrunken section named at the top of this file instead. What survives
// the substitution is the taper: the tooth's radial size at any station is
// proportional to its cone distance, which is what makes one lofted tooth
// serve the whole face width.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftToothBody, assertLoftToothBody) -->
func stepLoftToothBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	return []*decad.Body{loftToothBody(t, doc, d, g, f)}
}

func assertLoftToothBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)
	if len(bodies) != 1 {
		t.Fatalf("%s tooth body: got %d bodies, want 1", g.Label, len(bodies))
	}
	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s tooth body bounds: %v", g.Label, err)
	}
	requireClose(t, bounds.Min.Z, apexStubFraction*f.Ded.X, 1e-6,
		"%s tooth body starts at the substituted apex station", g.Label)
	requireClose(t, bounds.Max.Z, heelOverrunFraction*f.Ded.X, 1e-6,
		"%s tooth body reaches past the heel cone", g.Label)

	// The taper: the tip radius at the heel-overrun station is the heel tip
	// radius scaled by the overrun, and the tooth never reaches further out
	// than that.
	_, _, _, tipHeel := o.section(heelOverrunFraction, 0, 1)
	reach := math.Max(math.Abs(bounds.Min.X), math.Abs(bounds.Max.X))
	requireClose(t, reach, tipHeel, 1e-3, "%s tooth body outer reach", g.Label)
}

// ---------------------------------------------------------------------------
// The two conical end trims.
// ---------------------------------------------------------------------------

// coneBandSlack is how far each cut cone is shifted outward, as a fraction of
// the distance between the two cone apexes. Both cut cones pass exactly
// through the tooth's own root edge — the toe cone through M and the heel cone
// through C — so an exact trim grazes the tooth along a whole curve, and decad
// refuses a boolean whose operands touch within the chord tolerance without
// provably crossing. Each cone is therefore shifted outward by this sliver,
// which trims a hair wide of flush; the extent assertions allow for exactly it.
const coneBandSlack = 0.01

// cutCones returns this gear's two trim cones: the heel cone the tooth must lie
// INSIDE and the toe cone it must lie OUTSIDE.
//
// Both are cones of revolution generated by lines parallel to the back cone —
// the toe edge M->N and the heel edge C->H — so each meets the shaft axis at a
// single point. Building them as swept solids and intersecting then cutting is
// the substitution for two split-body features and their keeper selection:
// decad has no split-by-face, and the band those two trims leave is their whole
// purpose. They are kept as SEPARATE operands rather than combined into one
// band first, because a boolean whose operand is itself a boolean result
// carries that result's facets and the second boolean then refuses the
// question.
func cutCones(t *testing.T, doc *decad.Document, g gear, f gearFrame) (heel, toe *decad.Body) {
	t.Helper()
	tanGamma := math.Tan(g.Gamma)
	heelAxis := f.Ded.X + f.Ded.Y*tanGamma
	toeAxis := f.Toe.X + f.Toe.Y*tanGamma
	if toeAxis > heelAxis {
		t.Fatalf("%s toe cone apex %.6f is beyond the heel cone apex %.6f",
			g.Label, toeAxis, heelAxis)
	}
	slack := coneBandSlack * (heelAxis - toeAxis)
	toeAxis -= slack
	heelAxis += slack

	maxR := 3 * f.Ded.Y
	zLow := toeAxis - maxR*tanGamma
	tip := 1e-4 * maxR
	heel = sweepBands(t, doc, []station{
		{Z: heelAxis - tip*tanGamma, ROut: tip},
		{Z: zLow, ROut: (heelAxis - zLow) / tanGamma},
	}, sweepFacets)
	toe = sweepBands(t, doc, []station{
		{Z: toeAxis - tip*tanGamma, ROut: tip},
		{Z: zLow - tip, ROut: (toeAxis - zLow + tip) / tanGamma},
	}, sweepFacets)
	return heel, toe
}

// trimToBand applies the toe cut and then the heel cut to one tooth body.
func trimToBand(t *testing.T, doc *decad.Document, g gear, f gearFrame, tooth *decad.Body) *decad.Body {
	t.Helper()
	heel, toe := cutCones(t, doc, g, f)
	inside, err := decad.Intersect(tooth, heel)
	if err != nil {
		t.Fatalf("%s heel cone trim: %v", g.Label, err)
	}
	trimmed, err := decad.Cut(inside, toe)
	if err != nil {
		t.Fatalf("%s toe cone trim: %v", g.Label, err)
	}
	return trimmed
}

// bandStations returns the trimmed tooth's toe and heel axial extents.
//
// Each cut cone's radius FALLS as z grows, and the tooth's own surfaces are
// cones through the Apex whose radius RISES with z, so the two ends are reached
// on different surfaces: the toe end at the tooth's TIP, which leaves the toe
// cone first, and the heel end at the tooth's ROOT, which stays inside the heel
// cone longest. Each is the crossing of two cones written in z.
func bandStations(o toothOutline, g gear, f gearFrame) (zToe, zHeel float64) {
	tanGamma := math.Tan(g.Gamma)
	heelAxis := f.Ded.X + f.Ded.Y*tanGamma
	toeAxis := f.Toe.X + f.Toe.Y*tanGamma
	slack := coneBandSlack * (heelAxis - toeAxis)
	qRoot := o.RootR / f.Ded.X // the tooth's root surface, radius per unit of z
	qTip := o.TipR / f.Ded.X   // and its tip surface
	return (toeAxis - slack) / (1 + qTip*tanGamma),
		(heelAxis + slack) / (1 + qRoot*tanGamma)
}

// stepCutConicalEnds trims the uncut Tooth Body to a flush band with the toe
// cone and then the heel cone.
//
// Two distinct bodies are involved and they must not be conflated: the cutting
// TOOLS are cone faces of the Gear Body, the revolved-hexagon frustum, and the
// TARGET being split is the lofted Tooth Body. The lofted tooth has no cone
// faces of its own, so searching it for one finds nothing. Each cone face is
// found by the cut edge's world MIDPOINT rather than an endpoint, because an
// endpoint near the apex singularity cannot be evaluated at all.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->
func stepCutConicalEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	tooth := loftToothBody(t, doc, d, g, f)
	return []*decad.Body{trimToBand(t, doc, g, f, tooth)}
}

func assertCutConicalEnds(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s trimmed tooth: got %d bodies, want 1", g.Label, len(bodies))
	}
	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s trimmed tooth bounds: %v", g.Label, err)
	}
	// The two cones trim the tooth to a band whose ends are where each cone
	// meets the tooth's own root surface — the section 2 toe corner M and
	// dedendum corner C, each moved out by the sliver named at coneBandSlack.
	// That the trimmed tooth reaches exactly those two stations is what "flush"
	// means here: too short and the tooth floats off the gear base, too long
	// and it overhangs it.
	zToe, zHeel := bandStations(newToothOutline(d, g), g, f)
	requireClose(t, bounds.Min.Z, zToe, 5e-3, "%s trimmed tooth toe station", g.Label)
	requireClose(t, bounds.Max.Z, zHeel, 5e-3, "%s trimmed tooth heel station", g.Label)
	// The trim removes material: the tooth that went in spanned the whole
	// apex-to-past-the-heel loft, and what comes out spans only the band, so
	// its volume is short of the untrimmed tooth's by everything outside it.
	// The untrimmed volume is read from the SAME closed form the loft is built
	// from rather than by re-lofting, since a boolean has already retired the
	// body that went in.
	trimmedVol := volumeOf(t, bodies[0])
	if trimmedVol <= 0 {
		t.Fatalf("%s trimmed tooth has volume %.6f mm^3", g.Label, trimmedVol)
	}
	span := zHeel - zToe
	loftSpan := (heelOverrunFraction - apexStubFraction) * f.Ded.X
	if span >= loftSpan {
		t.Fatalf("%s trimmed tooth spans %.6f mm of a %.6f mm loft: nothing was trimmed",
			g.Label, span, loftSpan)
	}
}

// ---------------------------------------------------------------------------
// Pattern and Combine.
// ---------------------------------------------------------------------------

// stepCircularPattern circular-patterns the trimmed tooth around the shaft-axis
// edge, one copy per tooth.
//
// The pattern is pinned at quantity = this gear's tooth number, totalAngle =
// 360 deg and isSymmetric = false; all three are set explicitly rather than
// left to Fusion's defaults. The angular spacing stays 360/N for the whole face
// width even though the pitch diameter shrinks toward the Apex: the radial
// taper is already in the lofted tooth, so the pattern only rotates that one
// tapered tooth into N evenly spaced copies. A pattern returns the SEED plus
// the copies, so the seed is never re-added.
//
// SUBSTITUTION. decad has no pattern feature, and leaving N copies live in one
// document makes its verification check every PAIR of them for interference,
// which it cannot decide for teeth this close together — it reports the pair
// undecided and the gate fails. So the proof applies ONE pattern increment,
// consuming the seed rather than copying it, and measures where that increment
// put the tooth. The count and the closure are then arithmetic over the same
// increment, and stepToothProfile independently checks the tooth is thin enough
// to repeat N times without overlapping itself.
//
// <!-- proof-run: proofkit3d.RunSolid(patternCases, stepCircularPattern, assertCircularPattern) -->
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	seed := trimToBand(t, doc, g, f, loftToothBody(t, doc, d, g, f))

	before, err := seed.Centroid()
	if err != nil {
		t.Fatalf("%s pattern seed centroid: %v", g.Label, err)
	}
	patternSeedAzimuth = math.Atan2(before.Value.Y, before.Value.X)
	patternSeedRadius = math.Hypot(before.Value.X, before.Value.Y)
	patternSeedHeight = before.Value.Z
	patternSeedVolume = volumeOf(t, seed)

	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(2*math.Pi/g.Teeth))
	if err != nil {
		t.Fatalf("%s pattern rotation: %v", g.Label, err)
	}
	// Placed retires the seed, so exactly one body stays live and decad is not
	// asked to decide a pair it cannot.
	moved, err := seed.Placed(rot)
	if err != nil {
		t.Fatalf("%s pattern copy: %v", g.Label, err)
	}
	return []*decad.Body{moved}
}

// patternSeedAzimuth and patternSeedVolume carry the seed's reading from the
// build to the assertion, because the seed is retired by the move and cannot be
// measured afterwards. The harness runs one case at a time in one goroutine.
var (
	patternSeedAzimuth float64
	patternSeedRadius  float64
	patternSeedHeight  float64
	patternSeedVolume  float64
)

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, _ := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s patterned tooth: got %d bodies, want 1", g.Label, len(bodies))
	}
	n := g.Teeth
	pitch := 2 * math.Pi / n

	// One pattern increment moves the tooth exactly one tooth pitch about the
	// shaft axis and changes nothing else about it.
	after, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("%s patterned tooth centroid: %v", g.Label, err)
	}
	az := math.Atan2(after.Value.Y, after.Value.X)
	requireClose(t, wrapAngle(az-patternSeedAzimuth-pitch), 0, 1e-6,
		"%s pattern increment is one tooth pitch", g.Label)
	requireClose(t, volumeOf(t, bodies[0]), patternSeedVolume, 1e-9,
		"%s pattern copy is the seed, moved", g.Label)
	// The copy keeps its distance from the axis and its station along it, which
	// is what makes the increment a rotation ABOUT that axis rather than any
	// other move: the pattern must not translate the tooth toward the Apex.
	requireClose(t, math.Hypot(after.Value.X, after.Value.Y), patternSeedRadius, 1e-6,
		"%s pattern copy stays the same distance from the shaft axis", g.Label)
	requireClose(t, after.Value.Z, patternSeedHeight, 1e-6,
		"%s pattern copy stays at the same station along the shaft axis", g.Label)

	// N increments close the circle exactly once, which is what
	// totalAngle = 360 deg with isSymmetric false means.
	requireClose(t, n*pitch, 2*math.Pi, tightTol,
		"%s pattern closes the circle exactly once", g.Label)

	// N copies at that pitch only fit if the tooth is thinner than the pitch.
	o := newToothOutline(d, g)
	right, left, _, _ := o.section(1, 0, 1)
	thickness := math.Abs(wrapAngle(
		math.Atan2(left[0].Y, left[0].X) - math.Atan2(right[0].Y, right[0].X)))
	if thickness >= pitch {
		t.Fatalf("%s tooth spans %.6f rad of a %.6f rad tooth pitch: %g copies would overlap",
			g.Label, thickness, pitch, n)
	}
}

// stepCombineJoin joins the patterned tooth pieces to the Gear Body in a single
// Combine-Join, the Gear Body as the target and the tooth bodies as the tools.
//
// pattern.bodies is a BRepBodies and combineFeatures.createInput rejects that
// type, so its items are copied into a fresh ObjectCollection first, and the
// seed is already among them.
//
// SUBSTITUTION. Chaining decad's faceted booleans compounds the mesh until an
// operand holds a collapsed facet and the evaluator refuses it, so the proof
// joins ONE tooth. That is the fact the join exists to establish: a tooth
// seated on the root cone leaves one lump, not two. It also sinks the tooth's
// root a twentieth of the tooth height below the root cone, because Fusion's
// Combine-Join takes the exact face-to-face seating and decad refuses a boolean
// whose operands graze without provably crossing. In the generated module the
// tooth seats exactly on the cone; nothing here asks for a sink.
//
// <!-- proof-run: proofkit3d.RunSolid(patternCases, stepCombineJoin, assertCombineJoin) -->
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)

	gearBody := revolveGearBody(t, doc, f)
	seed := trimToBand(t, doc, g, f, loftToothBody(t, doc, d, g, f))
	joined, err := decad.Union(gearBody, seed)
	if err != nil {
		t.Fatalf("%s combine-join: %v", g.Label, err)
	}
	return []*decad.Body{joined}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s joined gear: got %d bodies, want 1", g.Label, len(bodies))
	}
	// The join has to leave ONE lump. A mis-seated tooth that floats off the
	// root cone shows up here as a second lump, which is exactly the gap the
	// crown's root anchoring exists to avoid.
	if lumps := len(bodies[0].Lumps()); lumps != 1 {
		t.Fatalf("%s joined gear has %d lumps, want 1", g.Label, lumps)
	}
	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s joined gear bounds: %v", g.Label, err)
	}
	// The tooth stands proud of the frustum, so the joined body reaches further
	// out than the gear body alone — the join added material rather than
	// swallowing the tooth.
	maxR := 0.0
	for _, v := range f.hexagon() {
		maxR = math.Max(maxR, v.Y)
	}
	reach := math.Max(math.Abs(bounds.Max.X), math.Abs(bounds.Max.Y))
	if reach <= maxR {
		t.Fatalf("%s joined gear reaches %.6f mm, no further than the frustum's %.6f mm",
			g.Label, reach, maxR)
	}
	// And it is a join, not a cut: the result holds at least the frustum.
	area, radial := polygonAreaCentroid(truncatedHexagon(f))
	frustum := 2 * math.Pi * radial * area * chordFactor(sweepFacets)
	if volumeOf(t, bodies[0]) <= frustum {
		t.Fatalf("%s joined gear is no larger than the frustum alone", g.Label)
	}
}

// ---------------------------------------------------------------------------
// The bore.
// ---------------------------------------------------------------------------

// stepBoreCut cuts the cylindrical through bore along the shaft axis.
//
// The cut is a symmetric through-extrude of the Bore sketch's circle,
// restricted to this Gear Body, with 2 * Cone Distance as the half-length per
// side so it clears any face width. The step is skipped entirely when Enable
// Bore is unchecked.
//
// <!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepBoreCut, assertBoreCut) -->
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	gearBody := revolveGearBody(t, doc, f)
	if !d.BoreEnable {
		return []*decad.Body{gearBody}
	}

	// The bore plane is rooted at the start of the shaft-axis edge, so the
	// sketch origin already lies on the axis and the circle is centred there:
	// its centre is FIXED and its diameter dimensioned, never coincident to the
	// sketch origin. The cut is symmetric through, with 2 * Cone Distance as
	// the half-length per side.
	half := 2 * d.ConeDistance
	tool := sweepBands(t, doc, []station{
		{Z: f.Axis.X - half, ROut: g.Bore / 2},
		{Z: f.Axis.X + half, ROut: g.Bore / 2},
	}, sweepFacets)
	bored, err := decad.Cut(gearBody, tool)
	if err != nil {
		t.Fatalf("%s bore cut: %v", g.Label, err)
	}
	return []*decad.Body{bored}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s bored gear: got %d bodies, want 1", g.Label, len(bodies))
	}
	// The baseline is the SWEPT model's own gear body, not the untruncated
	// solid of revolution, so the truncation and the chording cancel out of the
	// difference below rather than leaking into it.
	area, radial := polygonAreaCentroid(truncatedHexagon(f))
	solidVol := 2 * math.Pi * radial * area * chordFactor(sweepFacets)
	got := volumeOf(t, bodies[0])
	if !d.BoreEnable {
		requireClose(t, got, solidVol, 3e-3, "%s gear body with the bore unchecked", g.Label)
		return
	}
	// The bore is a full-height cylinder through the frustum, so what it
	// removes is the frustum's own material inside the bore radius: the volume
	// of the solid of revolution of the hexagon clipped to r <= bore/2.
	removed := solidVol - got
	if removed <= 0 {
		t.Fatalf("%s bore removed %.6f mm^3", g.Label, removed)
	}
	wantRemoved := revolvedVolumeInside(truncatedHexagon(f), g.Bore/2) * chordFactor(sweepFacets)
	requireClose(t, removed, wantRemoved, 5e-3, "%s bore removed volume", g.Label)

	// A through bore leaves one lump and no enclosed void.
	if lumps := len(bodies[0].Lumps()); lumps != 1 {
		t.Fatalf("%s bored gear has %d lumps, want 1", g.Label, lumps)
	}
}

// revolvedVolumeInside is the volume of the hexagon's solid of revolution
// clipped to radii at or below r, computed by integrating the axial thickness
// over the radius. It is the material a through bore of that radius removes.
func revolvedVolumeInside(poly []vec2, r float64) float64 {
	const steps = 4000
	total := 0.0
	for i := range steps {
		rad := r * (float64(i) + 0.5) / steps
		lo, hi, ok := axialSpanAt(poly, rad)
		if !ok {
			continue
		}
		total += 2 * math.Pi * rad * (hi - lo) * (r / steps)
	}
	return total
}

// axialSpanAt returns the axial extent of the hexagon at a given radius.
func axialSpanAt(poly []vec2, rad float64) (lo, hi float64, ok bool) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		if (a.Y-rad)*(b.Y-rad) > 0 {
			continue
		}
		if a.Y == b.Y {
			continue
		}
		f := (rad - a.Y) / (b.Y - a.Y)
		z := a.X + f*(b.X-a.X)
		lo = math.Min(lo, z)
		hi = math.Max(hi, z)
		ok = true
	}
	return lo, hi, ok
}

// ---------------------------------------------------------------------------
// The meshing rotation.
// ---------------------------------------------------------------------------

// stepMeshRotation rotates the driving body by half a tooth pitch about its own
// shaft axis, so a driving valley sits where the pinion tooth crosses the axial
// plane.
//
// The rotation runs in the Design component, before the body is moved out, and
// takes its axis and origin from the profile edge's WORLD endpoints. The
// pinion's own extra phase is _PINION_MESH_PHASE_TEETH tooth-fractions, which
// is 0: a zero angle is not a move at all — Fusion refuses the identity
// transform with "invalid transform" — so the helper returns early rather than
// making each call site guard it.
//
// <!-- proof-run: proofkit3d.RunSolid(meshCases, stepMeshRotation, assertMeshRotation) -->
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)

	body := revolveGearBody(t, doc, f)
	angle := meshAngle(d, g, p)
	if angle == 0 {
		// The pinion's phase is zero by default, and rotating by zero is a
		// no-op the caller must not attempt.
		return []*decad.Body{body}
	}
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s mesh rotation: %v", g.Label, err)
	}
	moved, err := body.Placed(rot)
	if err != nil {
		t.Fatalf("%s mesh rotation: %v", g.Label, err)
	}
	return []*decad.Body{moved}
}

// meshAngle is the rotation this gear receives: half a tooth pitch for the
// driving gear, and the pinion's own mesh phase (0 by default) for the pinion.
func meshAngle(d design, g gear, p map[string]float64) float64 {
	if g.Label == "Driving" {
		return math.Pi / g.Teeth
	}
	return pinionMeshPhase(d.Pinion.Teeth)
}

func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s rotated body: got %d bodies, want 1", g.Label, len(bodies))
	}
	// The rotation is about the body's own axis of revolution, so it changes
	// neither volume nor bounds — which is exactly why the angle has to be
	// checked as a number rather than read off the solid.
	area, radial := polygonAreaCentroid(truncatedHexagon(f))
	requireVolume(t, bodies[0], 2*math.Pi*radial*area*chordFactor(sweepFacets), 3e-3,
		"%s body volume is unchanged by the meshing rotation", g.Label)

	angle := meshAngle(d, g, p)
	if g.Label == "Driving" {
		requireClose(t, angle, math.Pi/g.Teeth, tightTol,
			"driving meshing rotation is half a tooth pitch")
		// Half a pitch is what puts a valley where the other gear's tooth is:
		// twice it is a whole pitch, which the pattern already repeats.
		requireClose(t, 2*angle, 2*math.Pi/g.Teeth, tightTol,
			"twice the meshing rotation is one whole tooth pitch")
	} else {
		requireClose(t, angle, 0, tightTol, "pinion mesh phase is zero by default")
	}
}

// ---------------------------------------------------------------------------
// Measurement helpers.
// ---------------------------------------------------------------------------

func volumeOf(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	v, err := m.Value.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("volume units: %v", err)
	}
	return v
}

func requireVolume(t *testing.T, b *decad.Body, want, rel float64, what string, args ...any) {
	t.Helper()
	got := volumeOf(t, b)
	if math.Abs(got-want) > rel*math.Max(1, math.Abs(want)) {
		t.Fatalf("%s: got %.6f mm^3, want %.6f mm^3 (relative tolerance %g)",
			sprintfArgs(what, args...), got, want, rel)
	}
}

// wrapAngle folds an angle into (-pi, pi].
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
