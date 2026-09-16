package bevelgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// NO BOOLEAN IS PERFORMED ANYWHERE IN THIS GEAR'S PROOF.
//
// At the decad revision proof/go.mod pins, a boolean accepts only prism, cup and
// faceted payloads and refuses an operand built by Loft. Every solid in this
// gear is conical, and a cone is a Loft here because Extrude refuses a nonzero
// taper. So the union that joins the frustum, the intersection and cut that trim
// the tooth, the bore's through-cut and the Combine-Join are all unavailable.
//
// The substitution is the same at every site: build the operands, lay them
// apart, and assert from their own measured geometry what the operation would
// have produced. Laying them apart leaves every volume, radius, station and cone
// angle unchanged, which is what makes the readings still mean something. Each
// site says below what its own substitution costs.
//
// THE FRAME. Every solid step works in the gear's own shaft frame: the shaft
// axis is world +Z with the apex at the origin, so a point's STATION is its z
// and its RADIUS is its distance from the axis. The §2 lattice's (station,
// radius) pairs map straight onto it. Bodies that a boolean would have consumed
// together are laid apart along +X, each about its own parallel axis, so no pair
// interferes and every reading is still taken about that body's own axis.
//
// THE TABLES RUN AT MODULE 4 TO 8, NEVER MODULE 1. decad's mesh bound has an
// absolute floor, so a figure small enough brings every measurement inside it
// and the gate reports Suspect on geometry that is in fact correct. Module is a
// pure scale on this figure, so a case at Module 4 through 8 proves the same
// shape as one at Module 1 and clears the floor.

// bgSpread is how far apart along +X operands are laid, in multiples of the
// figure's own Cone Distance.
const bgSpread = 3.0

// bgSolid is the per-case scaffolding: one decad document, one sketch World,
// and the lattice the case resolves to.
type bgSolid struct {
	t   *testing.T
	doc *decad.Document
	w   *sketch.World
	lat bgLattice
	g   bgMember
}

func bgNewSolid(t *testing.T, doc *decad.Document, p map[string]float64) *bgSolid {
	in := bgRead(p)
	if in.Module < 4 {
		t.Fatalf("solid case at Module %v: the solid tables run at Module 4 to 8, "+
			"because decad's mesh bound has an absolute floor a Module 1 figure sits inside", in.Module)
	}
	l := bgSolve(in)
	return &bgSolid{t: t, doc: doc, w: sketch.NewWorld(), lat: l, g: l.bgSide()}
}

// bgSweepFacets is how many sides the polygonal sweep is drawn with.
//
// A band is drawn as an explicit regular polygon rather than a circle because a
// lofted polygon is a polyhedron, whose volume decad proves exactly; a lofted
// circle is tessellated, and the proven bound on its volume is wide enough that
// the harness gate reports Suspect on a thin band that is in fact correct. The
// polygon's own area is smaller than the circle's by a known factor
// (bgFacetFactor), which every volume assertion below carries.
const bgSweepFacets = 48

// ring draws one regular polygon of circumradius r at station z, about the axis
// x = lateral — one cross-section of the polygonal sweep that stands in for the
// revolve.
func (b *bgSolid) ring(z, r, lateral float64) (*sketch.Sketch, *sketch.Profile) {
	b.t.Helper()
	plane, err := b.w.CreateOffsetPlane(b.w.XY(), z)
	if err != nil {
		b.t.Fatalf("station plane at z=%.4f: %v", z, err)
	}
	s, err := b.w.CreateSketch(plane)
	if err != nil {
		b.t.Fatalf("station sketch at z=%.4f: %v", z, err)
	}
	pts := make([]*sketch.Point, bgSweepFacets)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / bgSweepFacets
		pts[i] = s.CreatePoint(lateral+r*math.Cos(a), r*math.Sin(a))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, q := range pts {
		s.Fix(q)
	}
	return b.solve(s)
}

// polygon draws a closed loop through pts in the plane given by frame.
func (b *bgSolid) polygon(frame r3.Frame, pts []bgPt) (*sketch.Sketch, *sketch.Profile) {
	b.t.Helper()
	plane, err := b.w.CreatePlaneFromFrame(frame)
	if err != nil {
		b.t.Fatalf("section plane: %v", err)
	}
	s, err := b.w.CreateSketch(plane)
	if err != nil {
		b.t.Fatalf("section sketch: %v", err)
	}
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p.X, p.Y)
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	for _, q := range sp {
		s.Fix(q)
	}
	return b.solve(s)
}

func (b *bgSolid) solve(s *sketch.Sketch) (*sketch.Sketch, *sketch.Profile) {
	b.t.Helper()
	res, err := s.Solve(b.t.Context())
	if err != nil {
		b.t.Fatalf("solve section sketch: %v", err)
	}
	if !res.Converged {
		b.t.Fatalf("section sketch did not converge: residual %.3e", res.Residual)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		b.t.Fatalf("section sketch holds %d profiles, want exactly 1", len(profiles))
	}
	if !profiles[0].Valid {
		b.t.Fatalf("section sketch's loop is not an extrudable profile")
	}
	return s, profiles[0]
}

// band lofts the frustum one straight profile edge sweeps about the axis
// x = lateral, between two stations. It is the polygonal sweep that stands in
// for a revolve: decad publishes a revolved body's volume with a proven bound
// equal to the volume itself, so a revolved body is Suspect at any tolerance and
// cannot pass the harness gate at all.
func (b *bgSolid) band(z0, r0, z1, r1, lateral float64) *decad.Body {
	b.t.Helper()
	s0, p0 := b.ring(z0, r0, lateral)
	s1, p1 := b.ring(z1, r1, lateral)
	body, err := b.doc.Loft(s0, p0, s1, p1)
	if err != nil {
		b.t.Fatalf("band loft from (z=%.4f, r=%.4f) to (z=%.4f, r=%.4f): %v", z0, r0, z1, r1, err)
	}
	return body
}

// bgRing is one measured cross-section of a built body: every vertex the body
// carries at one station, and the radius they share.
type bgRing struct {
	Z, R  float64
	Count int
}

// bgReadRings measures a body's vertex rings about the axis x = lateral. Each
// ring's radius is the mean of its vertices' distances from that axis, and its
// count is how many facets the sketch engine's circle was drawn with — which is
// what turns an exact frustum volume into the volume this polygonal sweep
// actually has.
func bgReadRings(t *testing.T, body *decad.Body, lateral float64) []bgRing {
	t.Helper()
	type acc struct {
		sum   float64
		count int
	}
	byZ := map[int64]*acc{}
	for _, v := range body.Vertices() {
		p := v.Position().Value
		key := int64(math.Round(p.Z * 1e6))
		a := byZ[key]
		if a == nil {
			a = &acc{}
			byZ[key] = a
		}
		a.sum += math.Hypot(p.X-lateral, p.Y)
		a.count++
	}
	out := make([]bgRing, 0, len(byZ))
	for key, a := range byZ {
		out = append(out, bgRing{Z: float64(key) / 1e6, R: a.sum / float64(a.count), Count: a.count})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Z < out[j].Z })
	return out
}

// bgFacetFactor turns an exact solid-of-revolution volume into the volume a
// polygonal sweep of n facets actually encloses: the inscribed n-gon's area is
// n sin(2 pi / n) / (2 pi) of the circle's.
func bgFacetFactor(n int) float64 {
	if n < 3 {
		return 1
	}
	x := 2 * math.Pi / float64(n)
	return math.Sin(x) / x
}

// bgVolumeReading is a body's volume reading: the value decad measured
// together with the bound it proved around it.
func bgVolumeReading(t *testing.T, body *decad.Body) decad.Measurement {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	return v
}

// bgVolume is the reading split into its two floats, for the assertions that
// add several readings up before comparing anything.
func bgVolume(t *testing.T, body *decad.Body) (value, bound float64) {
	t.Helper()
	v := bgVolumeReading(t, body)
	return v.Value.Base(), v.Bound.Base()
}

// bgRequireVolume checks a body against the volume this proof's own formula
// gives, where rel is that formula's error — a facet factor, a Pappus figure
// on the §2 hexagon. decadtest adds the reading's own proven bound to it, so
// what is asserted is that decad's interval and this proof's claim overlap.
func bgRequireVolume(t *testing.T, body *decad.Body, what string, want, rel float64) {
	t.Helper()
	decadtest.Measures(t, what, bgVolumeReading(t, body),
		units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(rel)))
}

// bgToothPolygon returns the virtual spur tooth's closed outline, in the tooth
// plane's own 2-D frame with the tooth centre at the origin and the tooth drawn
// already rotated 180° — the angle `draw()` is given.
//
// The flanks are the real involute, from the shared involute package, at the
// same 15 samples the VirtualSpurProxy serves. Two chords stand in for curves
// the spur generator draws as arcs: the tooth-top arc and the root arc. That
// substitution is the tooth generator's geometry rather than bevel's, and it is
// proved in proof/spurgear/sketches_test.go; what it costs here is a few
// hundredths of a square millimetre of section area, which every assertion below
// takes from the polygon actually built rather than from a formula.
func bgToothPolygon(module, teeth, sink float64) []bgPt {
	dim := involute.Derive(module, teeth, bgPressureAngle)
	dim.Root -= sink
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, teeth, bgInvoluteSteps, math.Pi)
	keep := func(in []involute.Pt) []bgPt {
		out := make([]bgPt, 0, len(in))
		for _, p := range in {
			if math.Hypot(p.X, p.Y) < dim.Root-1e-12 {
				continue // embedded: the flank starts inside the root circle
			}
			out = append(out, bgPt{p.X, p.Y})
		}
		return out
	}
	l, r := keep(left), keep(right)
	if len(l) < 2 || len(r) < 2 {
		return nil
	}
	// Walk the outline: down the right flank from the tip to its root end,
	// across the root, up the left flank to its tip, and close across the tooth
	// top. When the tooth is NOT embedded the spur generator draws a radial
	// flank-to-root line on each side, so the walk steps in to the root circle
	// first; when it IS embedded the flanks already start inside the root circle
	// and the two ends meet with no connecting lines.
	poly := make([]bgPt, 0, len(l)+len(r)+2)
	for i := len(r) - 1; i >= 0; i-- {
		poly = append(poly, r[i])
	}
	if !dim.Embedded() {
		poly = append(poly, bgScale(bgUnit(r[0]), dim.Root), bgScale(bgUnit(l[0]), dim.Root))
	}
	poly = append(poly, l...)
	return poly
}

// bgRootArcPoints samples the root ARC the spur generator draws between the two
// flank feet, in the tooth plane's own frame.
//
// bgToothPolygon substitutes a chord for that arc, and a chord lies INSIDE the
// arc it spans, so a reading taken off the polygon alone understates how far out
// the drawn root reaches. The root is the surface the Combine-Join meets the gear
// body on, so it is read here from the circle the generator actually draws.
func bgRootArcPoints(module, teeth, sink float64, samples int) []bgPt {
	dim := involute.Derive(module, teeth, bgPressureAngle)
	dim.Root -= sink
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, teeth, bgInvoluteSteps, math.Pi)
	// Each flank's foot is where it meets the root circle: the first sample
	// outside the root circle, carried radially onto it. That is the same foot
	// the generator draws, whether it reaches the root through a flank-to-root
	// line or the flank already starts outside.
	foot := func(flank []involute.Pt) float64 {
		for _, p := range flank {
			if math.Hypot(p.X, p.Y) >= dim.Root {
				return math.Atan2(p.Y, p.X)
			}
		}
		return math.Atan2(flank[0].Y, flank[0].X)
	}
	a, c := foot(left), foot(right)
	// The tooth is drawn at 180 degrees, so its two feet straddle the -X axis and
	// their principal angles land on opposite branches. Unwrap onto the branch
	// that keeps the SHORT arc, which is the one the root is drawn on.
	if c-a > math.Pi {
		a += 2 * math.Pi
	} else if a-c > math.Pi {
		c += 2 * math.Pi
	}
	out := make([]bgPt, 0, samples+1)
	for i := 0; i <= samples; i++ {
		th := a + (c-a)*float64(i)/float64(samples)
		out = append(out, bgPt{dim.Root * math.Cos(th), dim.Root * math.Sin(th)})
	}
	return out
}

// bgPolygonArea is the outline's own area, read from the points actually drawn.
func bgPolygonArea(poly []bgPt) float64 {
	sum := 0.0
	for i := range poly {
		j := (i + 1) % len(poly)
		sum += poly[i].X*poly[j].Y - poly[j].X*poly[i].Y
	}
	return math.Abs(sum) / 2
}

// toothFrame is the tooth plane in the shaft frame: origin at the tooth centre
// K'/L' on the shaft axis, U pointing so that the tooth's own +X runs AWAY from
// the dedendum corner (so a tooth drawn at 180° faces the corner), V
// circumferential.
//
// This plane is the real back-cone tooth plane, tilted by gamma out of the
// axis-perpendicular. Building it tilted rather than flat is what puts the
// tooth's root and tip on the true root and tip cones, which is what every
// later cut reading depends on.
func (b *bgSolid) toothFrame(scale, lateral float64) r3.Frame {
	b.t.Helper()
	g := b.g
	sK := b.lat.bgStation(g, g.ToothCtr)
	u := r3.NewVec(-math.Cos(g.Gamma), 0, math.Sin(g.Gamma))
	v := r3.NewVec(0, 1, 0)
	origin := r3.NewVec(lateral, 0, sK*scale)
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		b.t.Fatalf("tooth frame: %v", err)
	}
	return frame
}

// coneSlopeAt is the wall slope, about the shaft axis, of the cone through the
// point that sits `d` from the tooth centre along the back cone. A point there
// has station sK - d sin gamma and radius d cos gamma, and the loft carries both
// straight back to the apex.
func (b *bgSolid) coneSlopeAt(d float64) float64 {
	sK := b.lat.bgStation(b.g, b.g.ToothCtr)
	return d * math.Cos(b.g.Gamma) / (sK - d*math.Sin(b.g.Gamma))
}

// expectedSlopes is what cones the drawn outline's own points ride, about the
// shaft axis, once the tooth plane places them.
//
// It is not the same as coneSlopeAt on the root and tip radii, and the
// difference is real geometry rather than error: the tooth plane is the BACK-CONE
// plane, so a point's radius about the shaft axis is hypot(px cos gamma, py) and
// only a point on the tooth's own centreline (py = 0) rides the cone its polar
// radius names. A tooth corner therefore sits a little inside the tip cone,
// exactly as the drawn tooth does in Fusion.
func (b *bgSolid) expectedSlopes(poly []bgPt) (low, high float64) {
	sK := b.lat.bgStation(b.g, b.g.ToothCtr)
	low, high = math.Inf(1), 0
	for _, p := range poly {
		slope := math.Hypot(p.X*math.Cos(b.g.Gamma), p.Y) / (sK + p.X*math.Sin(b.g.Gamma))
		low = math.Min(low, slope)
		high = math.Max(high, slope)
	}
	return low, high
}

// toothDims is the virtual spur gear's four circle radii, which are what the
// drawn tooth's root and tip actually ride.
//
// The pitch and tip circles sit exactly where the back cone puts them, because
// the virtual tooth number is the real 2 r_v / Module. The ROOT circle sits one
// root sink further in, and that inset is what seats the tooth's whole root arc
// inside the gear body rather than leaving its two corners proud of the root
// cone — see bgMember.Circles.
func (b *bgSolid) toothDims() involute.Dimensions {
	return b.g.Circles(b.lat.In.Module)
}

// toothBody lofts the uncut apex->heel tooth.
//
// SUBSTITUTION AND COST. Fusion lofts the §2 Apex SKETCH POINT — a degenerate
// point section — to the §3 tooth profile. decad's Loft takes two profiles, so
// the proof substitutes a SHRUNKEN SECTION for the apex point: the same outline
// scaled about the apex by `nose`. The cost is that the true point-section is
// not built; what is proved is the taper the loft has to produce, which the
// assertions read off the body. `lower` drops the WHOLE tooth along the back
// cone; it is not the root sink, which shortens the root circle alone and is
// already inside the polygon. Nothing uses `lower` today, and it is kept because
// the Combine-Join reading is what it exists for.
func (b *bgSolid) toothBody(nose, lower, lateral float64) (*decad.Body, []bgPt) {
	b.t.Helper()
	poly := bgToothPolygon(b.lat.In.Module, b.g.VirtualTeeth, b.g.RootSink)
	if lower != 0 {
		// The tooth's own +X runs away from the dedendum corner, so ADDING to it
		// walks every point back toward the tooth centre — down the back cone,
		// which is what lowers the tooth's radius.
		for i := range poly {
			poly[i].X += lower
		}
	}
	small := make([]bgPt, len(poly))
	for i, p := range poly {
		small[i] = bgScale(p, nose)
	}
	s0, p0 := b.polygon(b.toothFrame(nose, lateral), small)
	s1, p1 := b.polygon(b.toothFrame(1, lateral), poly)
	body, err := b.doc.Loft(s0, p0, s1, p1)
	if err != nil {
		b.t.Fatalf("%s tooth loft: %v", b.g.Label, err)
	}
	return body, poly
}

// bgHeelVertices returns only the tooth's HEEL-section vertices.
//
// The loft substitution adds a nose section that is the same outline scaled
// about the apex, so it sits at `nose` times the heel's stations — well inside
// half the body's own reach. A slope reading needs no such filter, because
// scaling about the apex leaves radius/station unchanged; a radius, a height or
// an azimuth does.
func bgHeelVertices(body *decad.Body) []r3.Vec {
	zmax := 0.0
	for _, v := range body.Vertices() {
		zmax = math.Max(zmax, v.Position().Value.Z)
	}
	out := make([]r3.Vec, 0, len(body.Vertices()))
	for _, v := range body.Vertices() {
		if q := v.Position().Value; q.Z > zmax/2 {
			out = append(out, q)
		}
	}
	return out
}

// bgSlopeRange reads the innermost and outermost cone a body's surfaces ride,
// as radius over station about the shaft axis. The nose section the loft
// substitution adds is a copy scaled about the apex, so it rides the same cones
// and needs no filtering out here.
func bgSlopeRange(body *decad.Body) (low, high float64) {
	low, high = math.Inf(1), 0
	for _, v := range body.Vertices() {
		q := v.Position().Value
		slope := math.Hypot(q.X, q.Y) / q.Z
		low = math.Min(low, slope)
		high = math.Max(high, slope)
	}
	return low, high
}

// bgSectionReading is what a tooth's heel section says about itself.
type bgSectionReading struct {
	Radius  float64 // the outermost reach from the shaft axis
	Height  float64 // outermost less innermost — the tooth's height at that section
	Azimuth float64 // where the section's centroid sits round the axis
}

func bgReadSection(body *decad.Body) bgSectionReading { return bgReadSectionAbout(body, 0) }

// bgReadSectionAbout reads a body that has been laid apart along +X, about its
// own parallel axis.
func bgReadSectionAbout(body *decad.Body, lateral float64) bgSectionReading {
	out := bgSectionReading{Height: 0, Azimuth: math.NaN()}
	lo := math.Inf(1)
	var sx, sy, n float64
	for _, q := range bgHeelVertices(body) {
		q.X -= lateral
		r := math.Hypot(q.X, q.Y)
		out.Radius = math.Max(out.Radius, r)
		lo = math.Min(lo, r)
		sx, sy, n = sx+q.X, sy+q.Y, n+1
	}
	out.Height = out.Radius - lo
	// The azimuth is the SECTION'S CENTROID, not its outermost vertex. A tooth
	// is symmetric about its own centreline, so two corners tie for outermost
	// and which one an argmax picks flips under a rotation; the centroid does
	// not, and it is the reading a pattern increment has to move by exactly one
	// pitch.
	if n > 0 {
		out.Azimuth = math.Atan2(sy/n, sx/n)
	}
	return out
}

// bgConeReading is what a built band says about the cone it lies on: where its
// wall meets the axis, and the slope of that wall.
type bgConeReading struct {
	ApexStation float64
	Slope       float64 // d(radius)/d(station)
	HalfAngle   float64 // radians, between the axis and the wall
	Base        float64 // how far apart in station the two rings it was read from sit
}

// bgSlopeTol is how tightly a wall slope read off two vertex rings can be
// trusted. The evaluator's coordinates carry about a micrometre of rounding, and
// a slope read over a short base divides by that base — the toe plug is a few
// hundredths of a millimetre thick at Toe Extension 100, where the toe face has
// nearly closed, so its wall is measured over the shortest base in the figure.
func bgSlopeTol(c bgConeReading) float64 { return math.Max(2e-6/c.Base, 1e-9) }

func bgReadCone(t *testing.T, body *decad.Body, lateral float64) bgConeReading {
	t.Helper()
	rings := bgReadRings(t, body, lateral)
	if len(rings) < 2 {
		t.Fatalf("band produced %d vertex ring(s), expected 2", len(rings))
	}
	lo, hi := rings[0], rings[len(rings)-1]
	slope := (hi.R - lo.R) / (hi.Z - lo.Z)
	return bgConeReading{
		ApexStation: lo.Z - lo.R/slope,
		Slope:       slope,
		HalfAngle:   math.Atan(math.Abs(slope)),
		Base:        math.Abs(hi.Z - lo.Z),
	}
}

// ----------------------------------------------------------------------------
// The solid case tables.

func bgSolidCases() []proofkit3d.Case {
	var out []proofkit3d.Case
	base := []struct {
		name string
		p    map[string]float64
	}{
		{"module_4_31_31_90", map[string]float64{idModule: 4, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90}},
		{"module_8_31_17_90", map[string]float64{idModule: 8, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90}},
		{"module_6_17_31_90", map[string]float64{idModule: 6, idDrivingTeeth: 17, idPinionTeeth: 31, idShaftAngle: 90}},
		{"module_5_31_31_35", map[string]float64{idModule: 5, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 35}},
		{"module_5_31_31_142", map[string]float64{idModule: 5, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 142}},
		{"module_4_19_13_60_toe_100", map[string]float64{idModule: 4, idDrivingTeeth: 19, idPinionTeeth: 13,
			idShaftAngle: 60, idToeExtension: 100}},
		{"module_4_43_31_75_spacing", map[string]float64{idModule: 4, idDrivingTeeth: 43, idPinionTeeth: 31,
			idShaftAngle: 75, idToothSpacing: 0.5, idToeExtension: 35}},
		{"module_6_31_31_90_user_toe_radius", map[string]float64{idModule: 6, idDrivingTeeth: 31, idPinionTeeth: 31,
			idShaftAngle: 90, idPinionToeRadius: 40, idDrivingToeRadius: 20, idToeExtension: 20}},
		// The two ends of the virtual tooth count's range, at a module the solid
		// tables can read. 16/12 at 90 degrees puts the pinion's virtual count at
		// exactly 15 and the driving gear's at 26.667, so the pair carries both an
		// integer count and a fractional one. 4/4 is the largest root-corner float
		// of any admitted pair, which is the case the root sink has to clear.
		{"module_4_16_12_90", map[string]float64{idModule: 4, idDrivingTeeth: 16, idPinionTeeth: 12,
			idShaftAngle: 90}},
		{"module_4_4_4_90", map[string]float64{idModule: 4, idDrivingTeeth: 4, idPinionTeeth: 4, idShaftAngle: 90}},
	}
	for _, bc := range base {
		for _, side := range []struct {
			name string
			v    float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := map[string]float64{idSide: side.v}
			for k, v := range bc.p {
				p[k] = v
			}
			out = append(out, proofkit3d.Case{Name: bc.name + "_" + side.name, Params: p})
		}
	}
	return out
}

var (
	revolveCases    = bgSolidCases()
	toothLoftCases  = bgSolidCases()
	conicalCutCases = bgSolidCases()
	patternCases    = bgSolidCases()
	combineCases    = bgSolidCases()
	boreCases       = bgBoreCases()
	meshRotateCases = bgSolidCases()
)

// bgBoreCases adds the Enable Bore branch and both bore-diameter branches to the
// shared table, because the bore step is the only one that reads them.
func bgBoreCases() []proofkit3d.Case {
	out := bgSolidCases()
	for _, extra := range []proofkit3d.Case{
		{Name: "bore_disabled_pinion", Params: map[string]float64{idModule: 4, idDrivingTeeth: 31,
			idPinionTeeth: 31, idShaftAngle: 90, idBoreEnable: 0, idSide: 0}},
		{Name: "bore_disabled_driving", Params: map[string]float64{idModule: 4, idDrivingTeeth: 31,
			idPinionTeeth: 31, idShaftAngle: 90, idBoreEnable: 0, idSide: 1}},
		{Name: "bore_user_diameter_pinion", Params: map[string]float64{idModule: 4, idDrivingTeeth: 31,
			idPinionTeeth: 31, idShaftAngle: 90, idPinionBore: 18, idDrivingBore: 22, idSide: 0}},
		{Name: "bore_user_diameter_driving", Params: map[string]float64{idModule: 4, idDrivingTeeth: 31,
			idPinionTeeth: 31, idShaftAngle: 90, idPinionBore: 18, idDrivingBore: 22, idSide: 1}},
	} {
		out = append(out, extra)
	}
	return out
}

// ----------------------------------------------------------------------------
// S12 — the gear-body revolve.

// stepRevolveGearBody builds the three bands the frustum's profile edges sweep,
// lays them apart and never joins them.
//
// THE COST IS THE UNION: the proof does not show the three bands closing into
// one watertight solid, only that each is separately watertight and that
// together they have the right volume, stations and cone angles.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	root := b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
		l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), 0)
	heel := b.band(l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded),
		l.bgStation(g, g.Heel), l.bgRadius(g, g.Heel), spread)
	toe := b.band(l.bgStation(g, g.ToeInner), l.bgRadius(g, g.ToeInner),
		l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe), 2*spread)
	return []*decad.Body{root, heel, toe}
}

func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	if len(bodies) != 3 {
		t.Fatalf("expected the root, heel and toe bands, got %d bodies", len(bodies))
	}
	lateral := []float64{0, spread, 2 * spread}
	ends := [][4]float64{
		{l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe), l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded)},
		{l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), l.bgStation(g, g.Heel), l.bgRadius(g, g.Heel)},
		{l.bgStation(g, g.ToeInner), l.bgRadius(g, g.ToeInner), l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe)},
	}
	names := []string{"root band", "heel band", "toe plug"}

	signed, facets := 0.0, 0
	for i, body := range bodies {
		rings := bgReadRings(t, body, lateral[i])
		if len(rings) != 2 {
			t.Fatalf("%s produced %d vertex ring(s), want 2", names[i], len(rings))
		}
		facets = rings[0].Count
		z0, r0, z1, r1 := ends[i][0], ends[i][1], ends[i][2], ends[i][3]
		lo, hi := z0, z1
		loR, hiR := r0, r1
		if lo > hi {
			lo, hi, loR, hiR = hi, lo, hiR, loR
		}
		bgClose(t, names[i]+" near station", rings[0].Z, lo, 1e-6)
		bgClose(t, names[i]+" far station", rings[1].Z, hi, 1e-6)
		bgClose(t, names[i]+" near ring radius", rings[0].R, loR, 1e-6)
		bgClose(t, names[i]+" far ring radius", rings[1].R, hiR, 1e-6)

		reading := bgVolumeReading(t, body)
		decadtest.Measures(t, names[i]+" volume", reading,
			units.CubicMillimeters(bgFacetFactor(rings[0].Count)*bgFrustum(z0, r0, z1, r1)),
			decadtest.WithinRel(units.Scalar(1e-9)))
		value := reading.Value.Base()
		if i == 2 {
			signed -= value
		} else {
			signed += value
		}
	}

	// The frustum as the SIGNED SUM of the three bands, against Pappus on the
	// §2 hexagon: root + heel - toe plug, the toe plug being the dish that
	// hollows the front face.
	wantHex := bgFacetFactor(facets) * l.bgRevolvedVolume(g)
	bgClose(t, "signed sum of the three bands against Pappus on the hexagon",
		signed, wantHex, 2e-4*wantHex)

	// Cone half-angle by cone half-angle: the heel band and the toe plug come
	// out parallel, on the back-cone family, and the root band at the dedendum
	// angle to them.
	rootCone := bgReadCone(t, bodies[0], lateral[0])
	heelCone := bgReadCone(t, bodies[1], lateral[1])
	toeCone := bgReadCone(t, bodies[2], lateral[2])
	backTol := math.Max(bgSlopeTol(heelCone), bgSlopeTol(toeCone))
	bgClose(t, "the heel band and the toe plug are parallel", heelCone.Slope, toeCone.Slope, backTol)
	bgClose(t, "the back-cone family's half-angle", heelCone.HalfAngle, math.Pi/2-g.Gamma,
		bgSlopeTol(heelCone))
	bgClose(t, "the root band's half-angle is the root cone angle", rootCone.HalfAngle, g.RootConeAngle,
		bgSlopeTol(rootCone))
	dedendumAngle := math.Atan(bgDedendumFactor * l.In.Module / l.R)
	bgClose(t, "root and back-cone walls stand perpendicular up to the dedendum angle",
		math.Abs(rootCone.HalfAngle+heelCone.HalfAngle), math.Pi/2-dedendumAngle,
		bgSlopeTol(rootCone)+bgSlopeTol(heelCone))
}

// ----------------------------------------------------------------------------
// S13 — the apex loft.

func stepLoftToothBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	body, _ := b.toothBody(bgNose, 0, 0)
	return []*decad.Body{body}
}

// bgNose is how far down the cone the shrunken section that stands in for the
// loft's degenerate apex point is placed, as a fraction of the tooth centre's
// station.
const bgNose = 0.02

func assertLoftToothBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	poly := bgToothPolygon(l.In.Module, g.VirtualTeeth, g.RootSink)
	sK := l.bgStation(g, g.ToothCtr)
	area := bgPolygonArea(poly)

	// A loft from a point to a profile is a cone on that profile: its volume is
	// a third of the section's area times the apex's perpendicular distance to
	// the section plane, less the nose the substitution cuts off. The tooth
	// plane is the BACK-CONE plane, tilted by gamma out of the
	// axis-perpendicular, so that distance is sK cos gamma and not sK.
	bgRequireVolume(t, bodies[0], "the lofted tooth's volume is the profile's cone",
		area*sK*math.Cos(g.Gamma)/3*(1-bgNose*bgNose*bgNose), 1e-6)

	// The taper is what makes it a bevel tooth: the root and the tip both ride
	// straight cones through the apex, so a reading anywhere along the body
	// gives the same slope. Those cones are the ones through the DRAWN root and
	// tip circles — see toothDims for why they are not the dedendum corner's.
	rootSlope, tipSlope := bgSlopeRange(bodies[0])
	wantRoot, wantTip := b.expectedSlopes(poly)
	bgClose(t, g.Label+" tooth root rides the cone the drawn outline puts it on", rootSlope, wantRoot, 1e-9)
	bgClose(t, g.Label+" tooth tip rides the cone the drawn outline puts it on", tipSlope, wantTip, 1e-9)
	if tipSlope <= rootSlope {
		t.Errorf("%s: the tooth has no height (root slope %.6f, tip slope %.6f)", g.Label, rootSlope, tipSlope)
	}

	// The tip stands exactly one module outside the back cone, so its centreline
	// rides the cone through a point one addendum beyond the tooth centre.
	dim := b.toothDims()
	bgClose(t, g.Label+" the tip's centreline rides the cone one module outside the back cone",
		b.coneSlopeAt(dim.Tip), b.coneSlopeAt(g.VirtualPitchRadius+bgAddendumFactor*l.In.Module), 1e-12)

	// And the drawn root sits inside the gear body's root cone ALL THE WAY ACROSS
	// the root arc, which is what seats the tooth rather than leaving it proud.
	//
	// The reading is the arc's MAXIMUM, not its centreline. The tooth plane is the
	// back-cone plane, so only a point on the tooth's own centreline rides the cone
	// its polar radius names; the arc's two corners stand further out. Put the root
	// circle at the dedendum corner exactly and the centreline touches the cone
	// while both corners float above it — by 0.002 module on the shipped 31/31
	// default and 0.027 on the 4/4 pair, the worst the table admits. A
	// centreline-only reading passes that tooth. The root sink is what buys the
	// margin asserted here.
	arc := bgRootArcPoints(l.In.Module, g.VirtualTeeth, g.RootSink, 64)
	_, rootArcSlope := b.expectedSlopes(arc)
	if rootArcSlope >= math.Tan(g.RootConeAngle) {
		t.Errorf("%s: the tooth's root arc stands proud of the gear body's root cone "+
			"(%.9f against %.9f)", g.Label, rootArcSlope, math.Tan(g.RootConeAngle))
	}
}

// ----------------------------------------------------------------------------
// S14 — the conical end cuts.

// stepConicalEndCuts performs NEITHER cut. Both operands are Lofts — the tooth
// and each cone alike — so the split is unavailable. The step builds the tooth
// and the two cones and lays them apart; the assertion reads each cone's apex
// and half-angle off the cone and each of the tooth's two surfaces off the
// tooth, solves the stations where they cross from those readings, and checks
// them against the flush band.
//
// THE COST IS THE SPLIT: the proof does not show the evaluator dividing the
// tooth, selecting the keeper, or leaving a watertight body.
func stepConicalEndCuts(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	tooth, _ := b.toothBody(bgNose, 0, 0)
	// The cutting TOOLS are cone faces of the GEAR BODY, never of the lofted
	// tooth, which has no cone face to find. These are those two cones.
	toeCone := b.band(l.bgStation(g, g.ToeInner), l.bgRadius(g, g.ToeInner),
		l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe), spread)
	heelCone := b.band(l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded),
		l.bgStation(g, g.Heel), l.bgRadius(g, g.Heel), 2*spread)
	// The gear body's own root cone, which is the surface the flush band is
	// measured on.
	rootBand := b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
		l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), 3*spread)
	return []*decad.Body{tooth, toeCone, heelCone, rootBand}
}

func assertConicalEndCuts(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	sK := l.bgStation(g, g.ToothCtr)

	rootSlope, tipSlope := bgSlopeRange(bodies[0])
	toe := bgReadCone(t, bodies[1], spread)
	heel := bgReadCone(t, bodies[2], 2*spread)
	bodyRoot := bgReadCone(t, bodies[3], 3*spread)

	// Where a cone of wall slope `k` through its own axis station `a` crosses a
	// straight surface of slope `m` through the apex: m*s = (a - s) * |k|.
	cross := func(c bgConeReading, m float64) float64 {
		return c.ApexStation * math.Abs(c.Slope) / (m + math.Abs(c.Slope))
	}
	toeRoot, heelRoot := cross(toe, rootSlope), cross(heel, rootSlope)
	toeTip, heelTip := cross(toe, tipSlope), cross(heel, tipSlope)

	// The two cuts land exactly on the flush band's own ends, read where each
	// cone meets the GEAR BODY's root cone — the surface the trimmed tooth has
	// to sit flush on.
	bgClose(t, "the toe cut lands on the toe end of the flush band",
		cross(toe, math.Abs(bodyRoot.Slope)), l.bgStation(g, g.Toe),
		1e-4*l.bgStation(g, g.Toe))
	bgClose(t, "the heel cut lands on the heel end of the flush band",
		cross(heel, math.Abs(bodyRoot.Slope)), l.bgStation(g, g.Ded),
		1e-4*l.bgStation(g, g.Ded))

	// And each cut meets the tooth's tip at a DIFFERENT station from its root.
	// That difference is the observable signature of a conical cut face: a
	// PLANE would cross both surfaces at one station, and the trimmed end would
	// not sit flush on the gear base.
	if math.Abs(toeTip-toeRoot) < 1e-6 {
		t.Errorf("the toe cut crosses the tooth's tip and root at the same station %.6f — "+
			"that is a planar cut, not a conical one", toeRoot)
	}
	if math.Abs(heelTip-heelRoot) < 1e-6 {
		t.Errorf("the heel cut crosses the tooth's tip and root at the same station %.6f — "+
			"that is a planar cut, not a conical one", heelRoot)
	}
	// Both cones lean the same way, so both tip crossings sit inboard of their
	// root crossings, and the trimmed tooth is shorter at the tip than at the
	// root — the flush band.
	if toeTip >= toeRoot || heelTip >= heelRoot {
		t.Errorf("a cut leans the wrong way: toe tip %.6f root %.6f, heel tip %.6f root %.6f",
			toeTip, toeRoot, heelTip, heelRoot)
	}
	// The heel cone is the one that legitimately misses on some ratio pairs, and
	// the helper raises that as the typed solids.NonIntersectError and returns
	// the keeper whole. Here it is a reading, not an error: the heel cut lands
	// beyond the tooth's own heel end when the cone never overshoots it.
	if heelRoot > sK {
		t.Logf("the heel cone does not reach the tooth (cut at %.4f, tooth ends at %.4f) — "+
			"this is the NonIntersectError case the helper catches", heelRoot, sK)
	}
}

// ----------------------------------------------------------------------------
// S15 — the circular pattern. THIS STEP IS SERIAL, and these package-level
// readings are why.
//
// The pattern increment retires the seed tooth, so the seed cannot be measured
// after the step runs: its azimuth, radius, height and volume have to be read
// during the build and handed to the assertion. That hand-off leaves the case,
// and two cases sharing one set of seed readings overwrite each other. It is
// not a hazard that announces itself — the two gear sides differ enough in
// volume that the overwrite was caught when it happened, and a pair of cases
// whose seeds measured alike would have passed on each other's numbers instead.
// So this step keeps proofkit3d.RunSolid while every other step in this package
// takes the parallel runner.
var (
	bgSeedAzimuth float64
	bgSeedRadius  float64
	bgSeedHeight  float64
	bgSeedVolume  decad.Measurement
)

func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	g := b.g
	seed, _ := b.toothBody(bgNose, 0, 0)

	// Read the seed while it still exists.
	bgSeedVolume = bgVolumeReading(t, seed)
	seedSection := bgReadSection(seed)
	bgSeedRadius, bgSeedHeight, bgSeedAzimuth = seedSection.Radius, seedSection.Height, seedSection.Azimuth

	// The pattern rotates that one tapered tooth into N evenly spaced copies
	// about the SHAFT-AXIS EDGE, quantity = this gear's Teeth Number,
	// totalAngle = '360 deg', isSymmetric = False. The angular spacing stays
	// 360/N for the entire face width even though the pitch diameter shrinks
	// toward the apex, because the radial taper is already in the loft.
	n := int(g.Teeth)
	step := 2 * math.Pi / float64(n)
	spread := bgSpread * b.lat.ConeDist
	out := make([]*decad.Body, 0, 2)
	for i, k := range []int{1, n - 1} {
		rot, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1),
			units.Radians(float64(k)*step))
		if err != nil {
			t.Fatalf("pattern rotation: %v", err)
		}
		// Adjacent teeth of one gear converge on the apex, so a copy left
		// coaxial with the seed sits close enough to it near the apex that the
		// evaluator cannot prove them disjoint. Each copy is laid apart along +X
		// after its rotation; the translation changes no azimuth, radius, height
		// or volume, and every reading below is taken about that copy's own axis.
		tr, err := r3.Translation(r3.NewVec(float64(i+1)*spread, 0, 0))
		if err != nil {
			t.Fatalf("pattern lay-apart: %v", err)
		}
		placed, err := rot.Then(tr)
		if err != nil {
			t.Fatalf("pattern placement: %v", err)
		}
		copyBody, err := seed.PlacedCopy(placed)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", k, err)
		}
		out = append(out, copyBody)
	}
	return out
}

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	g := b.g
	n := int(g.Teeth)
	step := 2 * math.Pi / float64(n)

	for i, k := range []int{1, n - 1} {
		// Two readings, not a reading against a formula: the pattern is
		// supposed to change nothing about the tooth, so the seed's own proven
		// bound counts towards the comparison as much as the copy's.
		decadtest.Agree(t, "patterned copy keeps the seed's volume",
			bgVolumeReading(t, bodies[i]), bgSeedVolume, decadtest.WithinRel(units.Scalar(1e-9)))
		section := bgReadSectionAbout(bodies[i], float64(i+1)*bgSpread*b.lat.ConeDist)
		bgClose(t, "patterned copy keeps the seed's radius", section.Radius, bgSeedRadius, 1e-6)
		bgClose(t, "patterned copy keeps the seed's height", section.Height, bgSeedHeight, 1e-6)
		turn := math.Mod(section.Azimuth-bgSeedAzimuth+4*math.Pi, 2*math.Pi)
		bgClose(t, "patterned copy sits one whole pitch increment round",
			turn, math.Mod(float64(k)*step, 2*math.Pi), 1e-9)
	}
}

// ----------------------------------------------------------------------------
// S16 — the Combine-Join.

// stepCombineJoin performs no join. It lays the operands apart and the assertion
// reads the join's two consequences off their own geometry.
//
// The tooth is built with the root sink the SPEC applies — the generated module
// draws the same sunk root circle — so "seated" is measurable here as a strict
// inequality without the proof inventing an offset of its own.
//
// THE COST IS THE STITCH: the proof cannot show the evaluator making one
// boundary out of two.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	tooth, _ := b.toothBody(bgNose, 0, 0)
	root := b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
		l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), spread)
	return []*decad.Body{tooth, root}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	spread := bgSpread * l.ConeDist
	sK := l.bgStation(g, g.ToothCtr)

	minSlope, tipSlope := bgSlopeRange(bodies[0])
	body := bgReadCone(t, bodies[1], spread)
	bodySlope := math.Abs(body.Slope)

	// The root reading is the root ARC's MAXIMUM, not the tooth's centreline and
	// not the body's minimum vertex. Both of those sit inside the arc's two
	// corners, so either one passes a tooth whose corners float outside the gear
	// body's root cone — which is the whole thing this step is here to catch.
	arc := bgRootArcPoints(l.In.Module, g.VirtualTeeth, g.RootSink, 64)
	_, rootSlope := b.expectedSlopes(arc)

	// The built body still has to be the drawn outline: its innermost reading is
	// the outline's own innermost point.
	poly := bgToothPolygon(l.In.Module, g.VirtualTeeth, g.RootSink)
	wantMin, _ := b.expectedSlopes(poly)
	bgClose(t, "the lofted tooth's innermost cone is the drawn outline's", minSlope, wantMin, 1e-9)

	// The readings are taken at the toe, the middle and the heel of the band the
	// join would cover.
	toe, heel := l.bgStation(g, g.Toe), l.bgStation(g, g.Ded)
	for _, at := range []struct {
		name string
		s    float64
	}{{"toe", toe}, {"middle", (toe + heel) / 2}, {"heel", heel}} {
		toothRoot := rootSlope * at.s
		toothTip := tipSlope * at.s
		bodyRoot := bodySlope * at.s
		// A join leaves ONE lump when the tooth's root is below the body's root
		// cone across its whole width — seated, not floating.
		if toothRoot >= bodyRoot {
			t.Errorf("at the %s the tooth's root arc reaches %.6f, on or above the gear body's root cone at %.6f — "+
				"the join would meet along a line rather than across the root", at.name, toothRoot, bodyRoot)
		}
		// And the joined body reaches further out than the frustum, because the
		// tooth's tip stands proud of it.
		if toothTip <= bodyRoot {
			t.Errorf("at the %s the tooth's tip reaches %.6f, no further than the frustum's %.6f — "+
				"the join would add nothing", at.name, toothTip, bodyRoot)
		}
	}
	// The margin the sink buys, stated as a number rather than left implicit: the
	// root arc's outermost point sits this far inside the gear body's root cone at
	// the heel, where the band is widest.
	gap := (bodySlope - rootSlope) * heel
	if gap <= 0 {
		t.Errorf("the root sink leaves no margin at the heel: %.9f mm", gap)
	}
	t.Logf("%s: the root sink clears the gear body's root cone by %.6f mm at the heel (%.4f module)",
		g.Label, gap, gap/l.In.Module)
	_ = sK
}

// ----------------------------------------------------------------------------
// S17 — the bore cut.

// stepBoreCut builds the tool as a REAL extrude, which a symmetric extent
// produces as a prism, but performs no cut: the target is the frustum, whose
// bands are Lofts.
//
// THE COST IS THE PIERCED BODY: one lump with a hole and no enclosed void is not
// shown.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	if !l.In.BoreEnable {
		// Enable Bore unchecked: the step is skipped entirely, and the per-gear
		// bore diameters are not consulted. There is nothing to build, so the
		// case proves the branch by asserting the resolved diameter is gone and
		// building only the frustum band the cut would have pierced.
		root := b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
			l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), 0)
		return []*decad.Body{root}
	}
	spread := bgSpread * l.ConeDist
	half := 2 * l.ConeDist
	start := l.bgStation(g, g.FrontFoot) // setByDistanceOnPath(shaft-axis edge, 0.0)
	plane, err := b.w.CreateOffsetPlane(b.w.XY(), start)
	if err != nil {
		t.Fatalf("bore plane: %v", err)
	}
	s, err := b.w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("bore sketch: %v", err)
	}
	// The plane is rooted at the shaft start, so the sketch origin is on the
	// axis: fix the circle's centre and give it a diameter dimension
	// ([PB-CIRCLE-CENTER]).
	centre := s.CreatePoint(0, 0)
	circle := s.CreateCircle(centre, g.BoreDia/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, g.BoreDia))
	sk, prof := b.solve(s)
	tool, err := doc.Extrude(sk, prof, decad.Symmetric{D: units.Millimeters(half)})
	if err != nil {
		t.Fatalf("bore tool: %v", err)
	}
	root := b.band(l.bgStation(g, g.Toe), l.bgRadius(g, g.Toe),
		l.bgStation(g, g.Ded), l.bgRadius(g, g.Ded), spread)
	return []*decad.Body{tool, root}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	if !l.In.BoreEnable {
		if g.BoreDia != 0 {
			t.Errorf("%s: Enable Bore is unchecked but a bore diameter of %.4f resolved", g.Label, g.BoreDia)
		}
		if len(bodies) != 1 {
			t.Errorf("Enable Bore unchecked: %d bodies built, want only the frustum band", len(bodies))
		}
		return
	}
	// The resolved diameter: this gear's Bore Diameter if specified, otherwise
	// this gear's Pitch Diameter / 4.
	want := g.PitchDia / 4
	if l.In.Driving && l.In.DrivingBore > 0 {
		want = l.In.DrivingBore
	}
	if !l.In.Driving && l.In.PinionBore > 0 {
		want = l.In.PinionBore
	}
	bgClose(t, g.Label+" resolved bore diameter", g.BoreDia, want, 1e-12)

	tool := bodies[0]
	box, err := tool.Bounds()
	if err != nil {
		t.Fatalf("bore tool bounds: %v", err)
	}
	half := 2 * l.ConeDist
	start := l.bgStation(g, g.FrontFoot)
	bgClose(t, "the bore tool's near end sits 2 * Cone Distance before the shaft edge's start",
		box.Min.Z, start-half, 1e-6)
	bgClose(t, "the bore tool's far end sits 2 * Cone Distance after the shaft edge's start",
		box.Max.Z, start+half, 1e-6)
	// A THROUGH cut: both ends clear the frustum, whatever the face width.
	loStation := math.Min(l.bgStation(g, g.ToeInner), l.bgStation(g, g.Toe))
	hiStation := l.bgStation(g, g.Heel)
	if box.Min.Z >= loStation || box.Max.Z <= hiStation {
		t.Errorf("the bore tool does not clear the frustum: tool [%.4f, %.4f], frustum [%.4f, %.4f]",
			box.Min.Z, box.Max.Z, loStation, hiStation)
	}
	// The tool's own diameter, read off the prism rather than off the input.
	rings := bgReadRings(t, tool, 0)
	for _, ring := range rings {
		bgClose(t, "the bore tool's radius", ring.R, g.BoreDia/2, 1e-6)
	}

	// What the cut would remove: the frustum's own profile clipped to the bore
	// radius, swept about the axis.
	removed := bgFacetFactor(rings[0].Count) * l.bgClippedVolume(g, g.BoreDia/2)
	if removed <= 0 {
		t.Errorf("%s: the bore would remove nothing", g.Label)
	}
	if full := l.bgRevolvedVolume(g); removed >= full {
		t.Errorf("%s: the bore would remove %.4f of a %.4f frustum, leaving nothing", g.Label, removed, full)
	}
}

// ----------------------------------------------------------------------------
// S18 — the meshing rotation.

func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	tooth, _ := b.toothBody(bgNose, 0, 0)
	angle := bgMeshAngle(l, g)
	if angle == 0 {
		// A zero angle is a no-op, not a move: setToRotation(0, axis, origin)
		// builds the identity and Fusion refuses to move a body by it with
		// `invalid transform`. rotate_body_about_edge absorbs that, which is why
		// the pinion's default phase of 0 does not crash the build.
		return []*decad.Body{tooth}
	}
	rot, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(angle))
	if err != nil {
		t.Fatalf("meshing rotation: %v", err)
	}
	// Half a tooth pitch leaves the turned body overlapping where it started, so
	// the two are laid apart along +X and each is read about its own axis.
	tr, err := r3.Translation(r3.NewVec(bgSpread*l.ConeDist, 0, 0))
	if err != nil {
		t.Fatalf("meshing lay-apart: %v", err)
	}
	placed, err := rot.Then(tr)
	if err != nil {
		t.Fatalf("meshing placement: %v", err)
	}
	moved, err := tooth.PlacedCopy(placed)
	if err != nil {
		t.Fatalf("meshing rotation: %v", err)
	}
	return []*decad.Body{tooth, moved}
}

// bgMeshAngle is this gear's extra rotation about its own shaft axis: half a
// tooth pitch for the driving gear, so a driving valley sits where the pinion
// tooth crosses the axial plane, and _PINION_MESH_PHASE_TEETH tooth-fractions
// for the pinion, which is 0 by default.
func bgMeshAngle(l bgLattice, g bgMember) float64 {
	if g.Label == "Driving" {
		return math.Pi / g.Teeth
	}
	return bgMeshPhaseTeeth * 2 * math.Pi / g.Teeth
}

func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	b := bgNewSolid(t, doc, p)
	l, g := b.lat, b.g
	angle := bgMeshAngle(l, g)
	if !l.In.Driving {
		bgClose(t, "the pinion's mesh phase", angle, 0, 0)
		if len(bodies) != 1 {
			t.Errorf("the pinion's phase is zero, so no move feature is emitted; got %d bodies", len(bodies))
		}
		return
	}
	bgClose(t, "the driving gear turns half a tooth pitch", angle, math.Pi/g.Teeth, 1e-12)
	if len(bodies) != 2 {
		t.Fatalf("expected the tooth before and after the rotation, got %d bodies", len(bodies))
	}
	decadtest.Agree(t, "the rotation moves the body and changes nothing about it",
		bgVolumeReading(t, bodies[1]), bgVolumeReading(t, bodies[0]),
		decadtest.WithinRel(units.Scalar(1e-9)))

	turn := math.Mod(bgReadSectionAbout(bodies[1], bgSpread*l.ConeDist).Azimuth-
		bgReadSection(bodies[0]).Azimuth+4*math.Pi, 2*math.Pi)
	bgClose(t, "the driving body's tooth moved by half a pitch", turn, math.Pi/g.Teeth, 1e-9)
}

// bgClippedVolume is the material a bore of the given radius removes from the
// frustum: the §2 hexagon clipped to radius <= r, swept about the shaft axis.
func (l bgLattice) bgClippedVolume(g bgMember, r float64) float64 {
	poly := l.bgHexagon(g)
	sr := make([]bgPt, 0, len(poly)+2)
	for i, p := range poly {
		q := poly[(i+1)%len(poly)]
		pi := bgPt{l.bgStation(g, p), l.bgRadius(g, p)}
		qi := bgPt{l.bgStation(g, q), l.bgRadius(g, q)}
		in, next := pi.Y <= r, qi.Y <= r
		if in {
			sr = append(sr, pi)
		}
		if in != next {
			t := (r - pi.Y) / (qi.Y - pi.Y)
			sr = append(sr, bgPt{pi.X + t*(qi.X-pi.X), r})
		}
	}
	if len(sr) < 3 {
		return 0
	}
	moment := 0.0
	for i := range sr {
		j := (i + 1) % len(sr)
		moment += (sr[i].X*sr[j].Y - sr[j].X*sr[i].Y) * (sr[i].Y + sr[j].Y)
	}
	return 2 * math.Pi * math.Abs(moment) / 6
}
