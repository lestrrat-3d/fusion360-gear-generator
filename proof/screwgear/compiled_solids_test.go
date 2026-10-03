package screwgear_test

// The solid steps of the compiled step list. decad builds what Fusion builds
// where it can, and where it cannot each step says what stands in and what
// the stand-in costs, beside the construction it replaces.

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

// Steps the proof does not build. S22's print note and S27's relocation and
// cleanup change no geometry. S26's count of one cage body, 16,602 mm³ at the
// defaults, needs the four bore cuts and both window cuts in one cage, which
// decad cannot chain (stepCutBore says why); the hand-written
// TestSleeveIsOnePiece holds the sleeve one piece on the implicit model.

func sgSolidCase(name string, p map[string]float64) proofkit3d.Case {
	return proofkit3d.Case{Name: name, Params: p}
}

// sgPolygonSketch draws a closed polygon of fixed points on the plane with the
// given frame and returns its one valid region.
func sgPolygonSketch(t *testing.T, w *sketch.World, f r3.Frame, poly [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("section plane: %v", err)
	}
	sk, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("section sketch: %v", err)
	}
	var pts []*sketch.Point
	for _, q := range poly {
		pts = append(pts, sk.CreatePoint(q[0], q[1]))
	}
	for i := range pts {
		sk.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, q := range pts {
		sk.Fix(q)
	}
	return sk, decadtest.SolveRegion(t, sk)
}

// sectionFrame is the plane square to gear g's axis at station s, with U
// along û_g and V along v̂_g, so its normal is +dir_g.
func (m *sgModel) sectionFrame(t *testing.T, g int, s float64) r3.Frame {
	f, err := r3.NewFrame(m.Origin[g].Add(m.Dir[g].Scale(s)), m.U[g], m.V[g])
	if err != nil {
		t.Fatalf("section frame: %v", err)
	}
	return f
}

// sgLoftChain builds the solid through the given section polygons, one per
// station in increasing order: a two-section ruled loft between each pair of
// neighbours as a sheet, the two end sections as caps, all stitched into one
// solid.
//
// What stands in, and what it costs. Fusion lofts a cell through all its
// sections at once, smooth between them; decad lofts two sections at a time,
// ruled, and walls each cell with two flat triangles, up to 0.125 mm off the
// ruled loft on the ribbon's faces at the defaults
// (TestLoftSectionCountHoldsTheHelicoid logs it). Two lofted solids that share
// a section are a face-on-face contact decad's booleans refuse, so the cells
// are built as sheets and stitched along their shared edges into one solid;
// the stitched solid is sound but no boolean may take it, which is why the
// joins of §3 and the cuts of §4 below stand in rather than run.
func sgLoftChain(t *testing.T, doc *decad.Document, w *sketch.World, m *sgModel, g int, stations []float64, polys [][][2]float64) *decad.Body {
	t.Helper()
	ctx := t.Context()
	var sks []*sketch.Sketch
	var prs []*sketch.Profile
	for i, st := range stations {
		sk, pr := sgPolygonSketch(t, w, m.sectionFrame(t, g, st), polys[i])
		sks = append(sks, sk)
		prs = append(prs, pr)
	}
	var parts []*decad.Body
	first, err := doc.Patch(ctx, sks[0], prs[0])
	if err != nil {
		t.Fatalf("first cap: %v", err)
	}
	parts = append(parts, first)
	for i := 0; i+1 < len(stations); i++ {
		wall, err := doc.Loft(ctx, sks[i], prs[i], sks[i+1], prs[i+1], decad.WithSurfaceResult())
		if err != nil {
			t.Fatalf("loft between stations %.4f and %.4f mm: %v", stations[i], stations[i+1], err)
		}
		parts = append(parts, wall)
	}
	last, err := doc.Patch(ctx, sks[len(sks)-1], prs[len(prs)-1])
	if err != nil {
		t.Fatalf("last cap: %v", err)
	}
	parts = append(parts, last)
	body, err := decad.Stitch(ctx, parts...)
	if err != nil {
		t.Fatalf("stitch %d sheets: %v", len(parts), err)
	}
	if !body.IsSolid() {
		free := 0
		for _, e := range body.Edges() {
			if e.IsFree() {
				free++
			}
		}
		t.Fatalf("the stitched sections leave a sheet with %d free edges, not a solid", free)
	}
	return body
}

// ribbonPiece is gear g's ribbon from station from over teeth whole pitches,
// through teeth*n + 1 sections.
func (m *sgModel) ribbonPiece(t *testing.T, doc *decad.Document, w *sketch.World, g int, from float64, teeth int) *decad.Body {
	var stations []float64
	var polys [][][2]float64
	straight := true
	for k := 0; k <= teeth*m.Steps; k++ {
		st := from + float64(k)*m.P/float64(m.Steps)
		poly := m.sectionPolygon(g, st)
		stations = append(stations, st)
		polys = append(polys, poly)
		straight = straight && sgStraightSide(poly[1:len(poly)-1])
	}
	if straight {
		// A straight ridge with no bow puts every toothed point of every
		// section on the chord between the two toothed corners. The engine's
		// profile then merges some of those collinear lines and not others,
		// and sections of unlike segment counts cannot be lofted in pairs; the
		// line between the corners is the same section, so the stand-in draws
		// that.
		for i, poly := range polys {
			polys[i] = [][2]float64{poly[0], poly[1], poly[len(poly)-2], poly[len(poly)-1]}
		}
	}
	return sgLoftChain(t, doc, w, m, g, stations, polys)
}

// sgStraightSide reports whether the points lie on the chord between the
// first and the last, to 1e-9 mm.
func sgStraightSide(pts [][2]float64) bool {
	a, b := pts[0], pts[len(pts)-1]
	ex, ey := b[0]-a[0], b[1]-a[1]
	l := math.Hypot(ex, ey)
	for _, q := range pts[1 : len(pts)-1] {
		if math.Abs((q[0]-a[0])*ey-(q[1]-a[1])*ex)/l > 1e-9 {
			return false
		}
	}
	return true
}

// helicoidVolume is the exact ribbon's volume over teeth whole pitches: the
// section's area does not change with the twist, and over whole pitches the
// cosine averages out, leaving T*(W - H/2) less the bow's T^3/12.
func (m *sgModel) helicoidVolume(teeth int) float64 {
	return float64(teeth) * m.P * (m.T*(m.W-m.H/2) - m.Bow*m.T*m.T*m.T/12)
}

// sgMeasureBelow holds a stand-in's volume within rel under the exact
// figure. The stand-in's chords all cut inside the curved surfaces they stand
// for, so it can only read low.
func sgMeasureBelow(t *testing.T, what string, body *decad.Body, exact, rel float64) {
	t.Helper()
	vol, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	// The window is [exact*(1 - rel), exact], plus the reading's own bound.
	decadtest.Measures(t, what, vol, units.CubicMillimeters(exact*(1-rel/2)), decadtest.Within(units.CubicMillimeters(exact*rel/2)))
}

// sgMeasureVolume holds a body's volume reading at want, within rel of it,
// under the step's own name for the body.
func sgMeasureVolume(t *testing.T, what string, body *decad.Body, want, rel float64) {
	t.Helper()
	vol, err := body.Volume()
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	decadtest.Measures(t, what, vol, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(rel)))
}

// sgMesh is a body's boundary as triangles, for point containment.
type sgMesh struct {
	verts []r3.Vec
	tris  [][3]int
}

// meshOf tessellates body. A lofted, stitched or boolean-built body restates
// its own triangles; a curved prism is chorded at tol.
func sgMeshOf(t *testing.T, body *decad.Body, tol float64) sgMesh {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(tol))
	if err != nil {
		t.Fatalf("tessellate: %v", err)
	}
	return sgMesh{verts: mesh.Vertices(), tris: mesh.Triangles()}
}

// contains is the generalized winding number of the mesh about q, rounded:
// 1 inside a closed, outward-oriented mesh and 0 outside. Every probe here
// stands at least 0.25 mm from the boundary it is read against, far beyond
// any mesh's chording.
func (s sgMesh) contains(q r3.Vec) bool {
	total := 0.0
	for _, tri := range s.tris {
		a, b, c := s.verts[tri[0]].Sub(q), s.verts[tri[1]].Sub(q), s.verts[tri[2]].Sub(q)
		la, lb, lc := a.Len(), b.Len(), c.Len()
		num := a.Dot(b.Cross(c))
		den := la*lb*lc + a.Dot(b)*lc + b.Dot(c)*la + c.Dot(a)*lb
		total += 2 * math.Atan2(num, den)
	}
	return math.Round(total/(4*math.Pi)) != 0
}

// --- Cell loft -----------------------------------------------------------------

var cellLoftCases = []proofkit3d.Case{
	sgSolidCase("gear A defaults", sgWith(map[string]float64{"gear": 0})),
	sgSolidCase("gear B defaults", sgWith(map[string]float64{"gear": 1})),
	sgSolidCase("negative slant", sgWith(map[string]float64{"gear": 1, "toothSlant": -25.8})),
	sgSolidCase("straight ridge", sgThirdPrint(map[string]float64{"gear": 0})),
	sgSolidCase("slow twist, floor of eight", sgWith(map[string]float64{"gear": 0, "twistLead": 400})),
	sgSolidCase("fast twist", sgWith(map[string]float64{"gear": 1, "twistLead": 20})),
	sgSolidCase("mounted at 30 degrees", sgWith(map[string]float64{"gear": 0, "mountAngleA": 30, "mountAngleB": 30})),
}

// stepCellLoft is a gear's cell loft of §2: c*n + 1 sections from s0 to
// s0 + c*P in station order, one body, then the slant's sign check. The loft
// stands in as sgLoftChain says.
func stepCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	w := sketch.NewWorld()
	return []*decad.Body{m.ribbonPiece(t, doc, w, g, m.cellStart(g), m.Cell)}
}

// sgSlantProbe is one of the four probes of the slant's sign check.
type sgSlantProbe struct {
	Name      string
	At        r3.Vec
	M, MWrong float64
}

// slantProbes are §2's four probes for gear g's cell: on and off the ridge
// through the first crest half a pitch into the cell, a quarter millimetre
// inside each face and under the crest.
func (m *sgModel) slantProbes(g int) []sgSlantProbe {
	s0 := m.cellStart(g)
	sc := m.Z0[g] + m.P*math.Ceil((s0+m.P/2-m.Z0[g])/m.P)
	var out []sgSlantProbe
	for _, face := range []float64{-1, 1} {
		vp := face * (m.T/2 - 0.25)
		up := m.W/2 - m.Bow*vp*vp - 0.25
		for _, on := range []bool{true, false} {
			st := sc + m.TanSlant*vp
			name := "off"
			if on {
				st = sc - m.TanSlant*vp
				name = "on"
			}
			out = append(out, sgSlantProbe{
				Name:   fmt.Sprintf("%s-ridge probe at the %+.0f face", name, face),
				At:     m.world(g, st, up, vp),
				M:      m.utooth(g, vp, st, 1) - up,
				MWrong: m.utooth(g, vp, st, -1) - up,
			})
		}
	}
	return out
}

func (p sgSlantProbe) used() bool {
	return math.Abs(p.M) >= 0.1 && math.Abs(p.MWrong) >= 0.1 && (p.M > 0) != (p.MWrong > 0)
}

func assertCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	cell := bodies[0]
	// The stand-in falls short of the helicoid by its chords: across the
	// thickness the toothed side is M - 1 lines, along the axis each cell is
	// ruled, and decad walls it with two flat triangles. Measured at the
	// defaults that is 2.1% of the volume; 4% bounds it at every case here.
	sgMeasureBelow(t, "cell volume", cell, m.helicoidVolume(m.Cell), 0.04)
	// Every point of the cell is within the crest rectangle's corner of the
	// axis and between the cell's end stations.
	reach := math.Hypot(m.W/2, m.T/2)
	for _, v := range cell.Vertices() {
		x, y, s := m.local(g, v.Position().Value)
		if math.Hypot(x, y) > reach+1e-9 || s < m.cellStart(g)-1e-9 || s > m.cellStart(g)+float64(m.Cell)*m.P+1e-9 {
			t.Fatalf("vertex at station %.4f mm, %.4f mm from the axis, is outside the cell", s, math.Hypot(x, y))
		}
	}
	// The slant's sign check: a used probe reads inside where the edge under
	// the input slant passes outside it, and outside where it does not.
	mesh := sgMeshOf(t, cell, 0.01)
	used := 0
	for _, probe := range m.slantProbes(g) {
		if !probe.used() {
			t.Logf("%s not used: m %.3f mm, under the negated slant %.3f mm", probe.Name, probe.M, probe.MWrong)
			continue
		}
		used++
		inside := mesh.contains(probe.At)
		if inside != (probe.M > 0) {
			t.Errorf("%s reads inside=%v, but the edge stands %.3f mm from it (%.3f mm under the negated slant)",
				probe.Name, inside, probe.M, probe.MWrong)
		}
	}
	switch {
	case sgIsDefaults(p):
		// At the defaults all four are used: 0.25 mm inside the tooth under the
		// right sign and 2.13 mm outside under the wrong one, and the reverse.
		if used != 4 {
			t.Errorf("%d of the four probes used at the defaults, want 4", used)
		}
		for i, probe := range m.slantProbes(g) {
			near, far := 0.25, 2.13
			if i%2 == 1 { // the off-ridge probe: outside, and inside under the wrong sign
				near, far = far, near
			}
			decadtestMeasure(t, probe.Name+" margin", math.Abs(probe.M), near, 0.01)
			decadtestMeasure(t, probe.Name+" margin under the wrong sign", math.Abs(probe.MWrong), far, 0.01)
		}
	case p["toothSlant"] == 0:
		// The straight ridge leans neither way, so no probe can tell the sign
		// and the build logs that it was not checked.
		if used != 0 {
			t.Errorf("%d probes used at a zero slant, want none", used)
		}
	}
}

// decadtestMeasure compares two plain figures of the model, which carry no
// decad bound, with a stated slack.
func decadtestMeasure(t *testing.T, what string, got, want, slack float64) {
	t.Helper()
	decadtest.Measures(t, what, decad.Measurement{Value: units.Millimeters(got), Exactness: decad.Exact},
		units.Millimeters(want), decadtest.Within(units.Millimeters(slack)))
}

// --- Doubling: copy, move, join ------------------------------------------------

// A note on documents. decad verifies every pair of live bodies in a
// document, and a body beside its exact copy, or beside a piece it meets on a
// shared section, is a contact its read-only intersection cannot classify:
// the harness's gate would read the pair Suspect. So where a step's Fusion
// result holds two bodies that coincide or touch, the step builds the pair in
// a scratch document of its own, makes its checks there, and hands the
// harness the one body the next step consumes.

var copyCases = []proofkit3d.Case{
	sgSolidCase("gear A cell", sgWith(map[string]float64{"gear": 0})),
	sgSolidCase("gear B cell", sgWith(map[string]float64{"gear": 1})),
}

// stepCopyBody is a round's copy (§3): copyPasteBodies.add on the current
// body, the copy taken from the feature's bodies. decad's Duplicate is the
// same operation: a new body with the source's geometry, the source left
// live. The source and its copy coincide, so they are compared in a scratch
// document, and the harness gates the copy's geometry, the cell, alone.
func stepCopyBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	scratch := decad.New()
	source := m.ribbonPiece(t, scratch, sketch.NewWorld(), g, m.cellStart(g), m.Cell)
	copied, err := source.Duplicate(t.Context())
	if err != nil {
		t.Fatalf("copy: %v", err)
	}
	if copied == source {
		t.Fatalf("the copy is the source body, not a body of its own")
	}
	a, err := source.Volume()
	if err != nil {
		t.Fatal(err)
	}
	b, err := copied.Volume()
	if err != nil {
		t.Fatal(err)
	}
	decadtest.Agree(t, "the copy against its source", a, b)
	va, vb := source.Vertices(), copied.Vertices()
	if len(va) != len(vb) {
		t.Fatalf("the copy has %d vertices, its source %d", len(vb), len(va))
	}
	for i := range va {
		decadtestMeasure(t, "a copied vertex against its source", va[i].Position().Value.Sub(vb[i].Position().Value).Len(), 0, 0)
	}
	return []*decad.Body{m.ribbonPiece(t, doc, sketch.NewWorld(), g, m.cellStart(g), m.Cell)}
}

func assertCopyBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	sgMeasureBelow(t, "copied cell volume", bodies[0], m.helicoidVolume(m.Cell), 0.04)
}

var moveCases = []proofkit3d.Case{
	sgSolidCase("gear A doubling a one-cell body", sgWith(map[string]float64{"gear": 0, "moveCells": 1})),
	sgSolidCase("gear B doubling a two-cell body", sgWith(map[string]float64{"gear": 1, "moveCells": 2})),
	sgSolidCase("gear A aside placed by 16 cells", sgWith(map[string]float64{"gear": 0, "moveCells": 16})),
	sgSolidCase("gear B aside placed by 16 cells", sgWith(map[string]float64{"gear": 1, "moveCells": 16})),
}

// stepScrewMove is a round's move (§3): the copy moved by Step(m*c), a turn of
// m*c*P/Lambda about the gear's axis and an advance of m*c*P along it, built
// as the rotation about the axis followed by the translation along it. The
// case moves a copy of the cell, which is the copy a doubling of a one-cell
// body or an aside of one cell moves; a body of more cells moves the same way.
func stepScrewMove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	copied := m.ribbonPiece(t, doc, sketch.NewWorld(), g, m.cellStart(g), m.Cell)
	step, err := m.screwStep(g, int(p["moveCells"])*m.Cell)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	moved, err := copied.Placed(t.Context(), step)
	if err != nil {
		t.Fatalf("move: %v", err)
	}
	return []*decad.Body{moved}
}

// offSections reports the largest distance from any vertex of body to the
// nearest point of the analytic section at its station, for gear g's
// sections at stations from first, every P/n, over sections of them.
func (m *sgModel) offSections(g int, body *decad.Body, first float64, sections int) float64 {
	worst := 0.0
	for _, v := range body.Vertices() {
		q := v.Position().Value
		_, _, s := m.local(g, q)
		k := int(math.Round((s - first) / (m.P / float64(m.Steps))))
		if k < 0 || k > sections {
			return math.Inf(1)
		}
		st := first + float64(k)*m.P/float64(m.Steps)
		best := math.Inf(1)
		b0, b1, f := m.sectionUV(g, st)
		for _, uv := range append([][2]float64{b0, b1}, f...) {
			best = math.Min(best, m.world(g, st, uv[0], uv[1]).Sub(q).Len())
		}
		worst = math.Max(worst, best)
	}
	return worst
}

func assertScrewMove(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	shift := float64(int(p["moveCells"])*m.Cell) * m.P
	// The ribbon is invariant under its screw step, so the moved copy is the
	// ribbon's own cell that much further on, vertex for vertex. The move
	// composes a rotation and a translation in float64, at stations up to
	// 180 mm from the axis's origin, so its vertices land within a few
	// nanometres of the analytic sections there; 1e-6 mm bounds that.
	off := m.offSections(g, bodies[0], m.cellStart(g)+shift, m.Cell*m.Steps)
	decadtestMeasure(t, "the moved copy against the cell it lands on", off, 0, 1e-6)
	sgMeasureBelow(t, "moved cell volume", bodies[0], m.helicoidVolume(m.Cell), 0.04)
}

var joinCases = []proofkit3d.Case{
	sgSolidCase("gear A one cell and its copy", sgWith(map[string]float64{"gear": 0})),
	sgSolidCase("gear B one cell and its copy", sgWith(map[string]float64{"gear": 1})),
	sgSolidCase("gear A mounted at 30 degrees", sgWith(map[string]float64{"gear": 0, "mountAngleA": 30, "mountAngleB": 30})),
}

// stepJoinBodies is a round's join (§3): the body and its moved copy joined
// into one body, here the one-cell body of a doubling's first round.
//
// What stands in, and what it costs. The two pieces are stitched solids,
// which no decad boolean takes, and they meet on a shared cross-section, a
// face-on-face contact decad's union refuses even between plain prisms. So the
// step builds the body and its moved copy in a scratch document, holds that
// the copy's first section is the body's last one and that the two lie on
// either side of it, and builds what the join leaves as the ribbon over both
// spans, holding its volume to the two pieces' sum: no sliver and no overlap.
// A stitched chain past about eight teeth exceeds decad's crossing audit, so
// the proof joins one cell and its copy; a longer body joins the same way.
// Fusion's combine and its one-body count are Fusion's.
func stepJoinBodies(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	return sgJoined(t, doc, m, g, m.cellStart(g), m.Cell, m.Cell)
}

// sgJoined builds, in a scratch document, a body of teethA teeth from station
// from and the next teethB teeth as the copy a screw step moves there, checks
// that the two meet on one section, and builds the joined ribbon over both in
// doc.
func sgJoined(t *testing.T, doc *decad.Document, m *sgModel, g int, from float64, teethA, teethB int) []*decad.Body {
	scratch := decad.New()
	w := sketch.NewWorld()
	body := m.ribbonPiece(t, scratch, w, g, from, teethA)
	step, err := m.screwStep(g, teethB)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	var moved *decad.Body
	if teethA == teethB {
		// A doubling: the copy is the body itself, moved by its own length.
		moved, err = body.PlacedCopy(t.Context(), step)
	} else {
		// The remainder: the stretch of teethB teeth that the screw step
		// carries onto the remainder's stations, which is the remainder.
		seed := m.ribbonPiece(t, scratch, w, g, from+float64(teethA-teethB)*m.P, teethB)
		moved, err = seed.Placed(t.Context(), step)
	}
	if err != nil {
		t.Fatalf("move: %v", err)
	}
	junction := from + float64(teethA)*m.P
	// The two pieces lie on either side of the junction's plane...
	for name, b := range map[string]*decad.Body{"body": body, "copy": moved} {
		for _, v := range b.Vertices() {
			_, _, s := m.local(g, v.Position().Value)
			if (name == "body" && s > junction+1e-6) || (name == "copy" && s < junction-1e-6) {
				t.Fatalf("the %s reaches station %.6f mm across the junction at %.6f mm", name, s, junction)
			}
		}
	}
	// ...and meet on it at the same section: every vertex of the copy on the
	// plane is a vertex of the body's last section, to the move's rounding.
	var last []r3.Vec
	for _, v := range body.Vertices() {
		if _, _, s := m.local(g, v.Position().Value); math.Abs(s-junction) < 1e-6 {
			last = append(last, v.Position().Value)
		}
	}
	matched := 0
	for _, v := range moved.Vertices() {
		q := v.Position().Value
		if _, _, s := m.local(g, q); math.Abs(s-junction) >= 1e-6 {
			continue
		}
		best := math.Inf(1)
		for _, l := range last {
			best = math.Min(best, l.Sub(q).Len())
		}
		decadtestMeasure(t, "the copy's first section against the body's last", best, 0, 1e-6)
		matched++
	}
	if matched == 0 || matched != len(last) {
		t.Fatalf("the junction holds %d vertices of the copy and %d of the body", matched, len(last))
	}
	va, err := body.Volume()
	if err != nil {
		t.Fatal(err)
	}
	vb, err := moved.Volume()
	if err != nil {
		t.Fatal(err)
	}
	joined := m.ribbonPiece(t, doc, w, g, from, teethA+teethB)
	vj, err := joined.Volume()
	if err != nil {
		t.Fatal(err)
	}
	sum := decad.Measurement{
		Value:     units.CubicMillimeters(sgMM3(t, va.Value) + sgMM3(t, vb.Value)),
		Exactness: decad.Approximate,
		Bound:     units.CubicMillimeters(sgMM3(t, va.Bound) + sgMM3(t, vb.Bound)),
	}
	// The pieces' sum and the joined ribbon are the same facets summed in a
	// different order, up to the move's rounding: they agree to 1e-6 mm³.
	decadtest.Agree(t, "the joined ribbon against its two pieces", vj, sum, decadtest.Within(units.CubicMillimeters(1e-6)))
	return []*decad.Body{joined}
}

func sgMM3(t *testing.T, v units.Value) float64 {
	t.Helper()
	x, err := v.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("volume unit: %v", err)
	}
	return x
}

func assertJoinBodies(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	sgMeasureBelow(t, "joined ribbon volume", bodies[0], m.helicoidVolume(2*m.Cell), 0.04)
}

// --- Remainder loft and join ---------------------------------------------------

var remainderCases = []proofkit3d.Case{
	sgSolidCase("gear A one tooth over", sgWith(map[string]float64{"gear": 0, "toothCount": 69})),
	sgSolidCase("gear B two teeth over", sgWith(map[string]float64{"gear": 1, "toothCount": 70})),
	sgSolidCase("gear A three teeth over", sgWith(map[string]float64{"gear": 0, "toothCount": 71})),
}

// stepRemainderLoft is the remainder cell of §3: r*n + 1 sections from
// s0 + q*c*P to s0 + N*P, built where the teeth belong. The loft stands in as
// sgLoftChain says.
func stepRemainderLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	if m.Remains == 0 {
		t.Fatalf("toothCount %d leaves no remainder", m.N)
	}
	from := m.cellStart(g) + float64(m.Whole*m.Cell)*m.P
	return []*decad.Body{m.ribbonPiece(t, doc, sketch.NewWorld(), g, from, m.Remains)}
}

func assertRemainderLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	sgMeasureBelow(t, "remainder volume", bodies[0], m.helicoidVolume(m.Remains), 0.04)
	// Its first section is the body's last: the cell's last section carried
	// by the screw step of the whole cells before it.
	from := m.cellStart(g) + float64(m.Whole*m.Cell)*m.P
	step, err := m.screwStep(g, (m.Whole-1)*m.Cell)
	if err != nil {
		t.Fatal(err)
	}
	cellEnd := m.cellStart(g) + float64(m.Cell)*m.P
	b0, b1, f := m.sectionUV(g, cellEnd)
	for _, uv := range append([][2]float64{b0, b1}, f...) {
		carried := step.Apply(m.world(g, cellEnd, uv[0], uv[1]))
		decadtestMeasure(t, "the body's last section against the remainder's first",
			carried.Sub(m.world(g, from, uv[0], uv[1])).Len(), 0, 1e-9)
	}
	off := m.offSections(g, bodies[0], from, m.Remains*m.Steps)
	decadtestMeasure(t, "the remainder against its sections", off, 0, 1e-9)
}

// stepRemainderJoin is the remainder's join (§3), standing in as
// stepJoinBodies does: the last whole cell and the remainder after it.
func stepRemainderJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	g := int(p["gear"])
	if m.Remains == 0 {
		t.Fatalf("toothCount %d leaves no remainder", m.N)
	}
	from := m.cellStart(g) + float64((m.Whole-1)*m.Cell)*m.P
	// The remainder is lofted where it belongs rather than moved there; the
	// copy sgJoined moves is the analytic remainder one cell back, which the
	// screw step carries onto the remainder's own stations.
	return sgJoined(t, doc, m, g, from, m.Cell, m.Remains)
}

func assertRemainderJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	sgMeasureBelow(t, "the last cell and the remainder", bodies[0], m.helicoidVolume(m.Cell+m.Remains), 0.04)
}

// --- Sleeve --------------------------------------------------------------------

// sgTube is the Sleeve sketch's ring extruded cageRise each way, in a fresh
// sketch on the selected plane (world XY through C).
func sgTube(t *testing.T, doc *decad.Document, w *sketch.World, m *sgModel) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(m.Centre, m.Ex, m.Kx)
	if err != nil {
		t.Fatalf("selected plane: %v", err)
	}
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatal(err)
	}
	sk, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatal(err)
	}
	for _, r := range []float64{m.Ri, m.Ro} {
		centre := sk.CreatePoint(0, 0)
		c := sk.CreateCircle(centre, r)
		sk.Fix(centre)
		sk.AddConstraint(sketch.NewDiameter(c, 2*r))
	}
	if _, err := sk.Solve(t.Context()); err != nil {
		t.Fatalf("sleeve sketch: %v", err)
	}
	var ring *sketch.Profile
	for _, pr := range sk.Profiles() {
		if pr.Valid && len(pr.Holes) == 1 {
			if ring != nil {
				t.Fatalf("two profiles with two loops")
			}
			ring = pr
		}
	}
	if ring == nil {
		t.Fatalf("no ring among the sleeve sketch's profiles")
	}
	tube, err := doc.Extrude(sk, ring, decad.Symmetric{D: units.Millimeters(m.Rise)})
	if err != nil {
		t.Fatalf("extrude the ring: %v", err)
	}
	return tube
}

var tubeCases = []proofkit3d.Case{
	sgSolidCase("defaults", sgDefaults()),
	sgSolidCase("tall", sgWith(map[string]float64{"cageRise": 25})),
	sgSolidCase("wide, thick wall", sgWith(map[string]float64{"cageRadius": 17, "collarHalf": 3.25})),
}

// stepExtrudeTube is the tube of §4: the Sleeve sketch's ring extruded as a
// new body, symmetric, cageRise each side.
func stepExtrudeTube(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	return []*decad.Body{sgTube(t, doc, sketch.NewWorld(), m)}
}

func assertExtrudeTube(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	// The ring's area times the height, exact for exact circles; decad's
	// reading carries its own bound.
	sgMeasureVolume(t, "tube volume", bodies[0], math.Pi*(m.Ro*m.Ro-m.Ri*m.Ri)*2*m.Rise, 1e-9)
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatal(err)
	}
	lo := m.Centre.Sub(r3.NewVec(m.Ro, m.Ro, m.Rise))
	hi := m.Centre.Add(r3.NewVec(m.Ro, m.Ro, m.Rise))
	decadtest.MeasuresBox(t, "tube", box, lo, hi, decadtest.Within(units.Millimeters(1e-9)))
}

// --- Bore cut ------------------------------------------------------------------

var boreCutCases = []proofkit3d.Case{
	sgSolidCase("gear A -R defaults", sgWith(map[string]float64{"bore": 0})),
	sgSolidCase("gear A +R defaults", sgWith(map[string]float64{"bore": 1})),
	sgSolidCase("gear B -R defaults", sgWith(map[string]float64{"bore": 2})),
	sgSolidCase("gear B +R defaults", sgWith(map[string]float64{"bore": 3})),
	sgSolidCase("gear A +R third print", sgThirdPrint(map[string]float64{"bore": 1})),
	sgSolidCase("gear B -R no allowance", sgWith(map[string]float64{"bore": 2, "roofAllowance": 0})),
	sgSolidCase("gear B +R at 110 degrees", sgWith(map[string]float64{"bore": 3, "crossAngle": 110})),
	sgSolidCase("gear A -R fine clearance", sgWith(map[string]float64{"bore": 0, "clearance": 0.05})),
}

// boreSections is the stand-in's section count for a bore: no two
// neighbouring sections more than 5 degrees of twist apart, nor more than the
// angle at which the facets between them take 4% of the clearance.
func (m *sgModel) boreSections(b sgBore) int {
	turn := (b.SpanHi - b.SpanLo) / m.Lambda
	step := math.Min(5*math.Pi/180, 2*math.Acos(1-0.04*m.Clear/m.Corner))
	return int(math.Ceil(turn/step-1e-12)) + 1
}

// boreChannel is the stand-in for a bore's sweep: the bore's rectangle at
// boreSections stations over its cut span, each turned by sense*(s - s0)/Lambda
// from the profile's angle at s0, lofted as sgLoftChain does.
func (m *sgModel) boreChannel(t *testing.T, doc *decad.Document, b sgBore, sense float64) *decad.Body {
	count := m.boreSections(b)
	th0 := m.theta(b.Gear, b.SpanLo)
	var stations []float64
	var polys [][][2]float64
	for i := 0; i < count; i++ {
		st := b.SpanLo + (b.SpanHi-b.SpanLo)*float64(i)/float64(count-1)
		th := th0 + sense*(st-b.SpanLo)/m.Lambda
		var poly [][2]float64
		for _, c := range [][2]float64{{-m.Hw, b.VLo}, {m.Hw, b.VLo}, {m.Hw, b.VHi}, {-m.Hw, b.VHi}} {
			x, y := sgTurn(c[0], c[1], th)
			poly = append(poly, [2]float64{x, y})
		}
		stations = append(stations, st)
		polys = append(polys, poly)
	}
	return sgLoftChain(t, doc, sketch.NewWorld(), m, b.Gear, stations, polys)
}

// stepCutBore stands in for a bore's twisted sweep cut (§4).
//
// What stands in, and what it costs. decad has no twisted sweep (a nonzero
// WithSweepTwist is ErrUnsupported), so the channel is a chain of
// two-section lofts through the sweep's own section turned by s/Lambda + Phi
// at boreSections stations, 18 at the defaults. The cut itself cannot run:
// the chain's cells cut from the tube one after another meet the previous
// cut's face on their shared section, a contact decad refuses; stitched into
// one solid, the channel is a body no boolean takes; and a second cut from
// the curved tube is refused anyway, because the first cut's held mesh bound
// exceeds the chord tolerance the next pair derives. A tube and a channel in
// one document overlap, which decad's verification cannot classify either, so
// the tube is built in a scratch document and the harness gates the channel.
// The step holds what the cut depends on: the profile starts and ends in air,
// the channel is the rectangle carried along the span, and the build's two
// probes at the crossing stand in the tube's wall and in the channel under
// the right twist sense, so the cut opens them, and outside it under the
// wrong sense, so the check tells the senses apart. That the cut leaves one
// body and the cage's volume after it are not read here: TestSleeveIsOnePiece
// holds the sleeve one piece, on the hand-written model.
func stepCutBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	b := m.bores()[int(p["bore"])]
	// The tube is built beside the channel in a scratch document: the two
	// overlap, a pair decad's verification cannot classify, and the tube's
	// own step is stepExtrudeTube.
	sgTube(t, decad.New(), sketch.NewWorld(), m)
	return []*decad.Body{m.boreChannel(t, doc, b, +1)}
}

// boreProbes are the build's two probes at the bore's crossing, on the
// toothed and back sides of the channel's middle.
func (m *sgModel) boreProbes(b sgBore) []r3.Vec {
	sc := m.crossing(b)
	th := m.theta(b.Gear, sc)
	uDir := m.U[b.Gear].Scale(math.Cos(th)).Add(m.V[b.Gear].Scale(math.Sin(th)))
	at := m.Origin[b.Gear].Add(m.Dir[b.Gear].Scale(sc))
	off := m.W/2 + m.Clear/2
	return []r3.Vec{at.Add(uDir.Scale(off)), at.Sub(uDir.Scale(off))}
}

// inWall reports whether q is in the tube's material.
func (m *sgModel) inWall(q r3.Vec) bool {
	rel := q.Sub(m.Centre)
	r := math.Hypot(rel.Dot(m.Ex), rel.Dot(m.Kx))
	return r > m.Ri && r < m.Ro && math.Abs(rel.Dot(m.Nx)) < m.Rise
}

func assertCutBore(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	b := m.bores()[int(p["bore"])]
	channel := bodies[0]
	// The profile starts and ends in air: every corner of the section at the
	// span's two ends stands inside Ri (in the hollow) or outside Ro (outside
	// the tube), never in the wall.
	for _, st := range []float64{b.SpanLo, b.SpanHi} {
		th := m.theta(b.Gear, st)
		for _, c := range [][2]float64{{-m.Hw, b.VLo}, {m.Hw, b.VLo}, {m.Hw, b.VHi}, {-m.Hw, b.VHi}} {
			_, y := sgTurn(c[0], c[1], th)
			if r := math.Hypot(st, y); r >= m.Ri && r <= m.Ro {
				t.Errorf("%s: a corner of the section at station %.3f mm stands %.3f mm from the frame's axis, in the wall", b.Name, st, r)
			}
		}
	}
	// The channel is the rectangle carried along the span, whose volume a
	// rigid turn does not change. decad walls each lofted cell with two flat
	// triangles, which fold inside the ruled wall by up to 0.32 mm on the long
	// faces at the defaults' 5 degrees a cell (TestSleeveBoreSubstituteKeepsItsClearance
	// logs it), so the stand-in reads 5.6% under the exact channel there and
	// 3% at the 32 sections of a 0.05 mm clearance; 8% bounds both.
	sgMeasureBelow(t, b.Name+" channel volume", channel, 2*m.Hw*(b.VHi-b.VLo)*(b.SpanHi-b.SpanLo), 0.08)
	right := sgMeshOf(t, channel, 0.01)
	wrongDoc := decad.New()
	wrong := sgMeshOf(t, m.boreChannel(t, wrongDoc, b, -1), 0.01)
	// A probe stands clearance/2 inside the channel's short wall. The
	// stand-in's flat triangles fold inside that wall by up to 0.09 mm at the
	// defaults, so the stand-in can be read at the probe only where the probe
	// stands at least 0.1 mm inside; the exact channel is read at every case.
	meshReadable := m.Clear/2 >= 0.1
	for i, q := range m.boreProbes(b) {
		name := fmt.Sprintf("%s probe %d", b.Name, i)
		if !m.inWall(q) {
			t.Errorf("%s stands outside the tube's wall, so the cut cannot be read there", name)
		}
		if !m.inChannel(b, q, +1) {
			t.Errorf("%s is outside the exact channel under the right twist sense", name)
		}
		if m.inChannel(b, q, -1) {
			t.Errorf("%s is inside the exact channel under the wrong twist sense, so the check cannot tell them apart", name)
		}
		if !meshReadable {
			t.Logf("%s stands %.3f mm inside the short wall, under the stand-in's facet departure; read on the exact channel only", name, m.Clear/2)
			continue
		}
		if !right.contains(q) {
			t.Errorf("%s is outside the stand-in channel under the right twist sense", name)
		}
		if wrong.contains(q) {
			t.Errorf("%s is inside the stand-in channel under the wrong twist sense", name)
		}
	}
}

// inChannel reports whether q lies in the bore's exact channel when the
// sweep turns the profile by sense*(s - s0)/Lambda from its angle at s0.
func (m *sgModel) inChannel(b sgBore, q r3.Vec, sense float64) bool {
	x, y, s := m.local(b.Gear, q)
	if s < b.SpanLo || s > b.SpanHi {
		return false
	}
	th := m.theta(b.Gear, b.SpanLo) + sense*(s-b.SpanLo)/m.Lambda
	u, v := sgTurn(x, y, -th)
	return math.Abs(u) <= m.Hw && v >= b.VLo && v <= b.VHi
}

// --- Window cut ----------------------------------------------------------------

var windowCutCases = []proofkit3d.Case{
	sgSolidCase("defaults, facing +k", sgWith(map[string]float64{"window": 0})),
	sgSolidCase("defaults, facing -k", sgWith(map[string]float64{"window": 1})),
	sgSolidCase("defaults, facing -k, plane the other way", sgWith(map[string]float64{"window": 1, "flipPlane": 1})),
	sgSolidCase("third print, facing +k", sgThirdPrint(map[string]float64{"window": 0})),
	sgSolidCase("110 degrees, facing +e", sgWith(map[string]float64{"window": 0, "crossAngle": 110})),
	sgSolidCase("110 degrees, facing -e, plane the other way", sgWith(map[string]float64{"window": 1, "crossAngle": 110, "flipPlane": 1})),
}

// windowFrame is the Window Plane's frame: through C, holding n̂, square to
// d. flip turns the frame's normal to -d, as Fusion's plane may face either
// way.
func (m *sgModel) windowFrame(t *testing.T, w sgWindow, flip bool) (r3.Frame, func(c [2]float64) [2]float64) {
	if flip {
		f, err := r3.NewFrame(m.Centre, m.Nx, w.Across)
		if err != nil {
			t.Fatal(err)
		}
		return f, func(c [2]float64) [2]float64 { return [2]float64{c[1], c[0]} }
	}
	f, err := r3.NewFrame(m.Centre, w.Across, m.Nx)
	if err != nil {
		t.Fatal(err)
	}
	return f, func(c [2]float64) [2]float64 { return c }
}

// stepCutWindow is a window's extrude cut of §4: the Window sketch's hexagon
// extruded one way, Ro + 1 mm along d, cut from the cage alone. The direction
// is the build's rule: positive when C + d maps to a positive sketch z.
//
// What stands in, and what it costs. decad refuses a second cut from the
// curved tube (stepCutBore says why), so each case cuts its one window from
// the tube alone, without the bores. The window search keeps every face of the
// cut collarWall from every bore's channel, and the assertion holds that at
// the probe, so the bores and the window do not meet and the cut is the same
// with them. That both windows come out of one cage is Fusion's.
func stepCutWindow(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := sgModelOf(p)
	win := m.windows()[int(p["window"])]
	if win.Corners == nil {
		t.Fatalf("no window facing %s: %s", win.Facing, win.Reason)
	}
	w := sketch.NewWorld()
	tube := sgTube(t, doc, w, m)
	f, toSketch := m.windowFrame(t, win, p["flipPlane"] != 0)
	var poly [][2]float64
	for _, c := range win.Corners {
		poly = append(poly, toSketch(c))
	}
	if p["flipPlane"] != 0 {
		// Swapping the axes reverses the walk; keep it counter-clockwise.
		for i, j := 0, len(poly)-1; i < j; i, j = i+1, j-1 {
			poly[i], poly[j] = poly[j], poly[i]
		}
	}
	sk, profile := sgPolygonSketch(t, w, f, poly)
	dir := decad.Against
	if f.ToLocal(m.Centre.Add(win.D)).Z > 0 {
		dir = decad.Along
	}
	tool, err := doc.Extrude(sk, profile, decad.Distance{D: units.Millimeters(m.Ro + 1), Dir: dir})
	if err != nil {
		t.Fatalf("window tool: %v", err)
	}
	cage, err := decad.Cut(t.Context(), tube, tool)
	if err != nil {
		t.Fatalf("cut the window facing %s: %v", win.Facing, err)
	}
	return []*decad.Body{cage}
}

// windowVolume is the wall the window takes away: over the hexagon on the
// plane, the wall's depth along d from a0(t) to a1(t).
func (m *sgModel) windowVolume(w sgWindow) float64 {
	tLo, tHi := math.Inf(1), math.Inf(-1)
	for _, c := range w.Corners {
		tLo, tHi = math.Min(tLo, c[0]), math.Max(tHi, c[0])
	}
	const n = 200000
	total := 0.0
	dt := (tHi - tLo) / n
	for i := 0; i < n; i++ {
		tt := tLo + (float64(i)+0.5)*dt
		zLo, zHi := math.Inf(1), math.Inf(-1)
		for j := range w.Corners {
			a, b := w.Corners[j], w.Corners[(j+1)%len(w.Corners)]
			if (a[0]-tt)*(b[0]-tt) > 0 || a[0] == b[0] {
				continue
			}
			z := a[1] + (b[1]-a[1])*(tt-a[0])/(b[0]-a[0])
			zLo, zHi = math.Min(zLo, z), math.Max(zHi, z)
		}
		if zHi <= zLo {
			continue
		}
		depth := math.Sqrt(m.Ro*m.Ro-tt*tt) - math.Sqrt(math.Max(0, m.Ri*m.Ri-tt*tt))
		total += (zHi - zLo) * depth * dt
	}
	return total
}

func assertCutWindow(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := sgModelOf(p)
	win := m.windows()[int(p["window"])]
	probe := m.windowProbe(win)
	// Before the cut the probe is in the wall, where the window goes...
	if !m.inWall(probe) {
		t.Errorf("the window probe stands outside the tube's wall before the cut")
	}
	// ...and keeps collarWall from every bore's channel.
	for _, b := range m.bores() {
		if gap := m.wallGap(m.stationTable(b), probe, m.CollarWall); gap < m.CollarWall {
			t.Errorf("the window probe stands %.3f mm from %s's channel, under collarWall", gap, b.Name)
		}
	}
	// After it the probe is outside the cage: the cut went the right way.
	if sgMeshOf(t, bodies[0], 0.01).contains(probe) {
		t.Errorf("the window probe is still inside the cage after the cut facing %s", win.Facing)
	}
	// The cage is the tube less the wall over the hexagon. The integral is a
	// midpoint sum over 200000 slices of a smooth integrand, good to well
	// under 1e-6 of it; decad's reading carries the facet bound of the cut.
	tube := math.Pi * (m.Ro*m.Ro - m.Ri*m.Ri) * 2 * m.Rise
	sgMeasureVolume(t, "cage volume after the window facing "+win.Facing, bodies[0], tube-m.windowVolume(win), 1e-6)
}
