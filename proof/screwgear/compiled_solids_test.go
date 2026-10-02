package screwgear_test

// The solid steps of spec/screwgear/steps.md, each proved in decad through
// proofkit3d.RunSolid: the document's verification has to come back Sound but
// for an area or centroid reading a faceted boolean left outside the default
// tolerance, and every returned body has to be a valid solid of one lump.
//
// Four kinds of step here are ones decad cannot build as Fusion does, and each
// takes a stand-in, named beside the function that builds it:
//
//   - A loft through more than two sections. decad lofts between exactly two
//     sections, ruled. The stand-in for the cell loft (and the remainder loft)
//     is a chain of two-section lofts, one per neighbouring pair of sections,
//     each built as a sheet (WithSurfaceResult), closed by a patch on each end
//     section and welded into one solid by Stitch. Fusion's loft through the
//     same sections is smooth between them (spec §2, "What the loft is").
//   - A bore's twisted sweep cut. decad has no twisted sweep: WithSweepTwist
//     accepts only zero at the pinned revision. The stand-in is the same
//     chain-of-lofts solid through the sweep's own section turned by
//     s/Lambda + Phi_g at each of the stations "What the proof's stand-in
//     costs" derives, 18 at the defaults.
//   - Copy, screw move and join. A copy coincides with its source and a moved
//     copy shares a face with the body it is joined to; decad's pairwise
//     verification reads both pairs Suspect (unsupported_pair_contact and
//     unsupported_pair_payload, measured at the pinned revision), and Union
//     refuses two bodies that meet face to face ("two operand facets overlap
//     in one plane"). So the real operations, Duplicate and Placed, run in a
//     document of their own and are held against the geometry the step is
//     meant to produce, which the gated document builds in place; the join is
//     proved as the weld of the two pieces that meet at the seam.
//   - A chain of cuts. A cut leaves the cage a faceted body whose held mesh
//     bound is coarser than the chord tolerance the next cut derives, which
//     decad refuses ("requested tolerance … is below the faceted body's
//     minimum mesh bound", measured on the second bore cut). The stand-in cuts
//     the tube once by the union of every tool cut so far, which is the same
//     solid the sequence of cuts leaves.
//
// The cost of the chain-of-lofts stand-in, measured here and not in the spec:
// decad builds a two-section loft between line segments as two flat triangles
// per wall cell, not as the ruled (bilinear) patch through its four corners.
// Between two rectangles turned dtheta apart the triangle pair departs from the
// ruled patch by |T|/4, T = vLo - vHi - wLo + wHi the cell's twist vector:
// about 0.12 mm on a 15 mm face at the cell's 1.91° step, and about 0.34 mm on a
// 15.4 mm bore face at the stand-in's 5° step. The spec's chord and facet
// figures (1.0 µm, 0.007 mm) are for the ruled loft, which is neither what
// Fusion builds nor what decad builds. The volume checks below carry the
// triangle pair's departure as slack, computed from the sections
// (sgTwistVolumeSlack).

import (
	"errors"
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

// ---------------------------------------------------------------------------
// Construction helpers.

// sgSection is one planar section: its plane's frame (origin on the gear's
// axis, x along û_g, y along v̂_g, normal +dir_g) and its corners on it.
type sgSection struct {
	origin, u, v r3.Vec
	xy           [][2]float64
}

func (q sgSection) world(i int) r3.Vec {
	return q.origin.Add(q.u.Scale(q.xy[i][0])).Add(q.v.Scale(q.xy[i][1]))
}

// sgSketchSection draws a section as fixed corners and lines on its own plane
// and returns the one valid profile.
func sgSketchSection(t *testing.T, w *sketch.World, q sgSection) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	f, err := r3.NewFrame(q.origin, q.u, q.v)
	if err != nil {
		t.Fatalf("section frame: %v", err)
	}
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("section plane: %v", err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("section sketch: %v", err)
	}
	pts := make([]*sketch.Point, len(q.xy))
	for i, c := range q.xy {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	return s, decadtest.SolveRegion(t, s)
}

// sgChainSolid is the chain-of-lofts stand-in: one sheet loft per pair of
// neighbouring sections, a patch on each end section, welded by Stitch.
func sgChainSolid(t *testing.T, doc *decad.Document, sections []sgSection, label string) *decad.Body {
	t.Helper()
	ctx := t.Context()
	w := sketch.NewWorld()
	sks := make([]*sketch.Sketch, len(sections))
	prs := make([]*sketch.Profile, len(sections))
	for i, q := range sections {
		sks[i], prs[i] = sgSketchSection(t, w, q)
	}
	var parts []*decad.Body
	for i := 0; i+1 < len(sections); i++ {
		b, err := doc.Loft(ctx, sks[i], prs[i], sks[i+1], prs[i+1], decad.WithSurfaceResult())
		if err != nil {
			t.Fatalf("%s: loft between sections %d and %d: %v", label, i, i+1, err)
		}
		parts = append(parts, b)
	}
	for _, i := range []int{0, len(sections) - 1} {
		b, err := doc.Patch(ctx, sks[i], prs[i])
		if err != nil {
			t.Fatalf("%s: patch on section %d: %v", label, i, err)
		}
		parts = append(parts, b)
	}
	body, err := decad.Stitch(ctx, parts...)
	if err != nil {
		t.Fatalf("%s: stitch: %v", label, err)
	}
	if !body.IsSolid() {
		t.Fatalf("%s: the stitched sections leave a sheet, not a solid", label)
	}
	return body
}

// sgCellSections is the sections of a ribbon piece of `teeth` teeth starting
// at station from, at the §2 spacing P/n.
func sgCellSections(m sgModel, g sgGear, from float64, teeth int) []sgSection {
	out := make([]sgSection, 0, teeth*m.n+1)
	for k := 0; k <= teeth*m.n; k++ {
		s, xy := m.cellCorners(g, from, k)
		out = append(out, sgSection{origin: g.origin.Add(g.dir.Scale(s)), u: g.u, v: g.v, xy: xy[:]})
	}
	return out
}

// sgTwistVolumeSlack bounds how far the volume of the triangle-pair walls
// decad builds can fall from the ruled walls through the same sections, plus
// how far the ruled walls fall inside the exact helicoid. A wall cell with
// side vectors a (along the axis) and b (across the face) and twist vector T
// differs from its bilinear patch by det(a, T, b)/12 of volume
// (decad docs/loft-design.md §5.2), so the sum of |det|/12 bounds the first.
// The second is the sag of a straight chord between two turned corners,
// rho*(1 - cos(dtheta/2)), over the cell's perimeter and length; the toothed
// edge's straight chords integrate exactly over whole pitches (the trapezoid
// rule is exact for a cosine sampled evenly over whole periods).
func sgTwistVolumeSlack(m sgModel, g sgGear, sections []sgSection) float64 {
	sum := 0.0
	for k := 0; k+1 < len(sections); k++ {
		a, b := sections[k], sections[k+1]
		for i := range len(a.xy) {
			j := (i + 1) % len(a.xy)
			vLo, vHi := a.world(i), a.world(j)
			wLo, wHi := b.world(i), b.world(j)
			ax := wLo.Sub(vLo)
			bx := vHi.Sub(vLo)
			tw := vLo.Sub(vHi).Sub(wLo).Add(wHi)
			sum += math.Abs(ax.Dot(tw.Cross(bx))) / 12
		}
	}
	length := float64(len(sections)-1) * m.P / float64(m.n)
	dtheta := m.P / float64(m.n) / m.lambda
	rho := math.Hypot(m.W/2, m.T/2)
	sag := 2 * (m.W + m.T) * length * rho * (1 - math.Cos(dtheta/2))
	return sum + sag
}

// sgHelicoidVolume is the exact twisted ribbon's volume over whole teeth: the
// cross-section's area integrated along the axis (Cavalieri), whose cosine
// term vanishes over whole pitches: T*(W - H/2)*teeth*P.
func sgHelicoidVolume(m sgModel, teeth int) float64 {
	return m.T * (m.W - m.H/2) * float64(teeth) * m.P
}

func mm(x float64) units.Value  { return units.Millimeters(x) }
func mm3(x float64) units.Value { return units.CubicMillimeters(x) }

// sgVolume reads a body's volume in mm³.
func sgVolume(t *testing.T, b *decad.Body) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	return v
}

func sgMM3(t *testing.T, v units.Value) float64 {
	t.Helper()
	x, err := v.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("volume unit: %v", err)
	}
	return x
}

func sgMM(t *testing.T, v units.Value) float64 {
	t.Helper()
	x, err := v.In(units.Millimeter)
	if err != nil {
		t.Fatalf("length unit: %v", err)
	}
	return x
}

// sgMeasuresVolume is decadtest.MeasuresVolume labelled with the piece's name.
func sgMeasuresVolume(t *testing.T, name string, b *decad.Body, want, slack float64) {
	t.Helper()
	decadtest.Measures(t, name+" volume", sgVolume(t, b), mm3(want), decadtest.Within(mm3(slack)))
}

// sgScrewStep is Step(k) of §3 for gear g: rotate by k*P/Lambda about the
// gear's axis, then translate by k*P along it.
func sgScrewStep(t *testing.T, m sgModel, g sgGear, teeth int) r3.Transform {
	t.Helper()
	k := float64(teeth)
	rot, err := r3.RotationAround(g.origin, g.dir, units.Radians(k*m.P/m.lambda))
	if err != nil {
		t.Fatalf("screw rotation: %v", err)
	}
	tr, err := r3.Translation(g.dir.Scale(k * m.P))
	if err != nil {
		t.Fatalf("screw translation: %v", err)
	}
	step, err := rot.Then(tr)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	return step
}

// ---------------------------------------------------------------------------
// Cell loft (§2) and remainder loft (§3).

var cellLoftCases = []proofkit3d.Case{
	{Name: "Gear A defaults", Params: sgWith(map[string]float64{"gear": 0})},
	{Name: "Gear B defaults", Params: sgWith(map[string]float64{"gear": 1})},
	{Name: "Gear A lead 400", Params: sgWith(map[string]float64{"gear": 0, "twistLead": 400})},
	{Name: "Gear A lead 20", Params: sgWith(map[string]float64{"gear": 0, "twistLead": 20})},
	{Name: "Gear A mount -25", Params: sgWith(map[string]float64{"gear": 0, "mountAngleA": -25})},
	{Name: "Gear B phase +2.6", Params: sgWith(map[string]float64{"gear": 1, "assemblyPhase": 2.6})},
	{Name: "Gear B phase -2.6", Params: sgWith(map[string]float64{"gear": 1, "assemblyPhase": -2.6})},
	{Name: "Gear A four teeth", Params: sgWith(map[string]float64{"gear": 0, "toothCount": 4})},
	{Name: "Gear B tooth 7.4", Params: sgWith(map[string]float64{"gear": 1, "toothHeight": 7.4})},
}

// stepCellLoft is the gear's tooth cell: c = min(CELL_TEETH, N) teeth from
// s0 = Z0 - L/2, lofted through c*n + 1 sections in station order. Stand-in:
// the chain of two-section lofts (see the file comment).
func stepCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	sections := sgCellSections(m, g, m.cellStart(g), m.c)
	if len(sections) != m.c*m.n+1 {
		t.Fatalf("%s cell: %d sections, want c*n + 1 = %d", g.label, len(sections), m.c*m.n+1)
	}
	return []*decad.Body{sgChainSolid(t, doc, sections, g.label+" Cell")}
}

func assertCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	sections := sgCellSections(m, g, m.cellStart(g), m.c)
	assertRibbonPiece(t, m, g, bodies[0], sections, m.c, g.label+" Cell")
}

// assertRibbonPiece holds a lofted ribbon piece to the exact helicoid's
// volume, within the triangle-pair and chord-sag slack, and to its station
// span: no vertex stands outside the piece's two end planes, so pieces built
// at whole teeth apart meet only on a shared section.
func assertRibbonPiece(t *testing.T, m sgModel, g sgGear, body *decad.Body, sections []sgSection, teeth int, name string) {
	t.Helper()
	slack := sgTwistVolumeSlack(m, g, sections)
	t.Logf("%s: volume %.4f mm³ against the helicoid's %.4f, slack %.4f", name,
		sgMM3(t, sgVolume(t, body).Value), sgHelicoidVolume(m, teeth), slack)
	sgMeasuresVolume(t, name, body, sgHelicoidVolume(m, teeth), slack+1e-9*sgHelicoidVolume(m, teeth))
	lo := sections[0].origin.Sub(g.origin).Dot(g.dir)
	hi := sections[len(sections)-1].origin.Sub(g.origin).Dot(g.dir)
	for _, v := range body.Vertices() {
		pos := v.Position()
		st := pos.Value.Sub(g.origin).Dot(g.dir)
		b := sgMM(t, pos.Bound)
		if st < lo-b-1e-9 || st > hi+b+1e-9 {
			t.Fatalf("%s: a vertex stands at station %.6f, outside the piece's span %.6f..%.6f", name, st, lo, hi)
		}
	}
}

var remainderLoftCases = []proofkit3d.Case{
	{Name: "Gear A 69 teeth", Params: sgWith(map[string]float64{"gear": 0, "toothCount": 69})},
	{Name: "Gear B 69 teeth", Params: sgWith(map[string]float64{"gear": 1, "toothCount": 69})},
	{Name: "Gear A 6 teeth", Params: sgWith(map[string]float64{"gear": 0, "toothCount": 6})},
	{Name: "Gear B 7 teeth phase 2.6", Params: sgWith(map[string]float64{"gear": 1, "toothCount": 7, "assemblyPhase": 2.6})},
}

// stepRemainderLoft is the remainder cell: r = N mod c teeth, lofted through
// r*n + 1 sections from s0 + q*c*P to s0 + N*P, where they belong.
func stepRemainderLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	if m.r == 0 {
		t.Fatalf("%d teeth leave no remainder", m.N)
	}
	from := m.cellStart(g) + float64(m.q*m.c)*m.P
	sections := sgCellSections(m, g, from, m.r)
	if len(sections) != m.r*m.n+1 {
		t.Fatalf("%s remainder: %d sections, want r*n + 1 = %d", g.label, len(sections), m.r*m.n+1)
	}
	return []*decad.Body{sgChainSolid(t, doc, sections, g.label+" Cell Remainder")}
}

func assertRemainderLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	from := m.cellStart(g) + float64(m.q*m.c)*m.P
	sections := sgCellSections(m, g, from, m.r)
	assertRibbonPiece(t, m, g, bodies[0], sections, m.r, g.label+" Cell Remainder")
	// The remainder's last section is the ribbon's positive end, s0 + N*P.
	end := sections[len(sections)-1].origin.Sub(g.origin).Dot(g.dir)
	if math.Abs(end-(m.cellStart(g)+m.L)) > 1e-9 {
		t.Fatalf("%s remainder ends at station %.6f, want s0 + N*P = %.6f", g.label, end, m.cellStart(g)+m.L)
	}
}

// ---------------------------------------------------------------------------
// Doubling (§3): copy, screw move, join.

// copyCases is the copy at the defaults on both gears; what a copy is does not
// depend on how many cells the body holds.
var copyCases = []proofkit3d.Case{
	{Name: "Gear A defaults", Params: sgWith(map[string]float64{"gear": 0})},
	{Name: "Gear B defaults", Params: sgWith(map[string]float64{"gear": 1})},
}

// stepCopyBody is copyPasteBodies.add(body) and the copy taken from the
// feature's bodies. STAND-IN: the copy coincides with its source, which
// decad's pairwise verification cannot classify, so the copy is made with
// Duplicate in a document of its own and held against its source there; the
// gated document holds the cell the copy is of.
func stepCopyBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	sections := sgCellSections(m, g, m.cellStart(g), m.c)

	side := decad.New()
	src := sgChainSolid(t, side, sections, g.label+" source")
	cp, err := src.Duplicate(t.Context())
	if err != nil {
		t.Fatalf("%s: copy: %v", g.label, err)
	}
	report := decadtest.Verify(t, side)
	decadtest.IsValid(t, report, cp)
	decadtest.Agree(t, g.label+" copy volume", sgVolume(t, cp), sgVolume(t, src), decadtest.Within(mm3(1e-9)))
	cs, err := src.Centroid()
	if err != nil {
		t.Fatal(err)
	}
	cc, err := cp.Centroid()
	if err != nil {
		t.Fatal(err)
	}
	decadtest.MeasuresVec(t, g.label+" copy centroid", cc, cs.Value, decadtest.Within(cs.Bound))
	if cp == src {
		t.Fatalf("%s: the copy is the source body itself ([SCREW-F-COPY-BODY])", g.label)
	}

	return []*decad.Body{sgChainSolid(t, doc, sections, g.label+" Cell")}
}

func assertCopyBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	sections := sgCellSections(m, g, m.cellStart(g), m.c)
	assertRibbonPiece(t, m, g, bodies[0], sections, m.c, g.label+" Cell")
}

// moveCases is every move the defaults' schedule makes, on both gears: the
// four doublings move the copy by 4, 8, 16 and 32 teeth and the aside moves
// by 64 (sgDoublingRounds(17, 4)).
var moveCases = func() []proofkit3d.Case {
	var out []proofkit3d.Case
	d := newModel(sgDefaults())
	for _, g := range []float64{0, 1} {
		for _, r := range sgDoublingRounds(d.q, d.c) {
			name := fmt.Sprintf("%s Step(%d)", map[float64]string{0: "Gear A", 1: "Gear B"}[g], r.moveTeeth)
			if r.aside {
				name += " aside"
			}
			out = append(out, proofkit3d.Case{Name: name, Params: sgWith(map[string]float64{"gear": g, "moveTeeth": float64(r.moveTeeth)})})
		}
	}
	out = append(out, proofkit3d.Case{Name: "Gear A lead 20 Step(4)", Params: sgWith(map[string]float64{"gear": 0, "moveTeeth": 4, "twistLead": 20})})
	out = append(out, proofkit3d.Case{Name: "Gear B mount -25 Step(8)", Params: sgWith(map[string]float64{"gear": 1, "moveTeeth": 8, "mountAngleB": -25})})
	return out
}()

// stepScrewMove is the move of a copy by Step(k) ([SCREW-F-SCREW-STEP]):
// rotate by k*P/Lambda about the gear's axis and translate k*P along it.
// STAND-IN: the moved copy shares its first section with the body it will be
// joined to, which decad's pairwise verification reads Suspect, so the copy
// is made and moved in a document of its own; the gated document holds the
// cell built in place k teeth further along, which is what the screw step
// must land the copy on. A body of m cells moves as each of its cells does,
// so one cell moved by each k the schedule uses stands for every body moved.
func stepScrewMove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	k := int(p["moveTeeth"])
	target := sgCellSections(m, g, m.cellStart(g)+float64(k)*m.P, m.c)
	return []*decad.Body{sgChainSolid(t, doc, target, fmt.Sprintf("%s Cell at +%d teeth", g.label, k))}
}

func assertScrewMove(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	k := int(p["moveTeeth"])
	inPlace := bodies[0]

	side := decad.New()
	src := sgChainSolid(t, side, sgCellSections(m, g, m.cellStart(g), m.c), g.label+" source")
	cp, err := src.Duplicate(t.Context())
	if err != nil {
		t.Fatalf("copy: %v", err)
	}
	moved, err := cp.Placed(t.Context(), sgScrewStep(t, m, g, k))
	if err != nil {
		t.Fatalf("%s: move by Step(%d): %v", g.label, k, err)
	}
	report := decadtest.Verify(t, side)
	decadtest.IsValid(t, report, moved)

	decadtest.Agree(t, fmt.Sprintf("%s copy moved by Step(%d) volume", g.label, k), sgVolume(t, moved), sgVolume(t, inPlace), decadtest.Within(mm3(1e-9)))
	cm, err := moved.Centroid()
	if err != nil {
		t.Fatal(err)
	}
	ci, err := inPlace.Centroid()
	if err != nil {
		t.Fatal(err)
	}
	// The in-place reading carries its own bound; the 1e-9 mm is the rounding
	// of a rotation of up to 64 teeth applied to coordinates of ~100 mm.
	decadtest.MeasuresVec(t, fmt.Sprintf("%s copy moved by Step(%d) centroid", g.label, k), cm, ci.Value, decadtest.Within(mm(sgMM(t, ci.Bound)+1e-9)))
	// Every vertex of the moved copy lands on a vertex of the cell built in
	// place: the body is invariant under its screw step, not only its twist.
	want := inPlace.Vertices()
	for _, v := range moved.Vertices() {
		pos := v.Position()
		best := math.Inf(1)
		for _, w := range want {
			best = math.Min(best, pos.Value.Sub(w.Position().Value).Len())
		}
		if best > sgMM(t, pos.Bound)+1e-8 {
			t.Fatalf("%s: a vertex of the copy moved by Step(%d) lands %.3g mm from every vertex of the cell built there", g.label, k, best)
		}
	}
}

// joinCases is every join of the defaults' schedule on both gears — the seam
// after 4, 8, 16 and 32 teeth for the doublings and after 64 for the aside —
// and the remainder joins: one tooth after 68 (69 teeth), two after 4 (six
// teeth, one cell and no round), three after 4 (seven teeth, on gear B at a
// shifted phase).
var joinCases = func() []proofkit3d.Case {
	var out []proofkit3d.Case
	d := newModel(sgDefaults())
	for _, g := range []float64{0, 1} {
		label := map[float64]string{0: "Gear A", 1: "Gear B"}[g]
		for _, r := range sgDoublingRounds(d.q, d.c) {
			out = append(out, proofkit3d.Case{
				Name:   fmt.Sprintf("%s seam after %d teeth", label, r.moveTeeth),
				Params: sgWith(map[string]float64{"gear": g, "seamTeeth": float64(r.moveTeeth), "rightTeeth": float64(d.c)}),
			})
		}
	}
	out = append(out,
		proofkit3d.Case{Name: "Gear A 69 teeth remainder", Params: sgWith(map[string]float64{"gear": 0, "toothCount": 69, "seamTeeth": 68, "rightTeeth": 1})},
		proofkit3d.Case{Name: "Gear B 69 teeth remainder", Params: sgWith(map[string]float64{"gear": 1, "toothCount": 69, "seamTeeth": 68, "rightTeeth": 1})},
		proofkit3d.Case{Name: "Gear A 6 teeth remainder", Params: sgWith(map[string]float64{"gear": 0, "toothCount": 6, "seamTeeth": 4, "rightTeeth": 2})},
		proofkit3d.Case{Name: "Gear B 7 teeth remainder phase 2.6", Params: sgWith(map[string]float64{"gear": 1, "toothCount": 7, "assemblyPhase": 2.6, "seamTeeth": 4, "rightTeeth": 3})},
	)
	return out
}()

// stepJoinBodies is the combine join of the body and the moved copy (or the
// remainder cell), which has to leave one body. STAND-IN: decad's Union
// refuses two bodies that meet face to face, so the join is proved at its
// seam: the last cell of the body (c teeth before the seam) and the first
// piece after it (a cell, or the remainder's r teeth), each lofted from
// sections drawn in sketches of its own, the seam section drawn twice, welded
// into one solid with no face left at the seam. decad's stitch audit refuses a
// whole 68-tooth ribbon ("the loft crossing audit's facet-pair count exceeds
// the fixed work ceiling", measured; 32 teeth stitch), so the seam is what the
// proof can reach; every piece stays inside its own station span
// (assertRibbonPiece), so pieces whole teeth apart meet nowhere else.
func stepJoinBodies(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	seam := m.cellStart(g) + p["seamTeeth"]*m.P
	right := int(p["rightTeeth"])
	left := sgCellSections(m, g, seam-float64(m.c)*m.P, m.c)
	after := sgCellSections(m, g, seam, right)
	return []*decad.Body{sgWeld(t, doc, left, after, fmt.Sprintf("%s join at %.4f", g.label, seam))}
}

// sgWeld lofts two pieces that share an end section, each from its own
// sketches, and welds every sheet but the two seam patches into one solid.
func sgWeld(t *testing.T, doc *decad.Document, left, right []sgSection, label string) *decad.Body {
	t.Helper()
	ctx := t.Context()
	var parts []*decad.Body
	for pi, piece := range [][]sgSection{left, right} {
		w := sketch.NewWorld() // each piece in sketches of its own
		sks := make([]*sketch.Sketch, len(piece))
		prs := make([]*sketch.Profile, len(piece))
		for i, q := range piece {
			sks[i], prs[i] = sgSketchSection(t, w, q)
		}
		for i := 0; i+1 < len(piece); i++ {
			b, err := doc.Loft(ctx, sks[i], prs[i], sks[i+1], prs[i+1], decad.WithSurfaceResult())
			if err != nil {
				t.Fatalf("%s: piece %d loft %d: %v", label, pi, i, err)
			}
			parts = append(parts, b)
		}
		// The outer end of each piece is capped; the seam is not.
		end := 0
		if pi == 1 {
			end = len(piece) - 1
		}
		b, err := doc.Patch(ctx, sks[end], prs[end])
		if err != nil {
			t.Fatalf("%s: piece %d cap: %v", label, pi, err)
		}
		parts = append(parts, b)
	}
	body, err := decad.Stitch(ctx, parts...)
	if err != nil {
		t.Fatalf("%s: weld: %v", label, err)
	}
	if !body.IsSolid() {
		t.Fatalf("%s: the two pieces do not close at the seam; the weld leaves a sheet", label)
	}
	return body
}

func assertJoinBodies(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	seam := m.cellStart(g) + p["seamTeeth"]*m.P
	right := int(p["rightTeeth"])
	sections := sgCellSections(m, g, seam-float64(m.c)*m.P, m.c+right)
	assertRibbonPiece(t, m, g, bodies[0], sections, m.c+right, g.label+" joined")
	// No face is left at the seam: the joined body keeps only its two end caps.
	caps := 0
	for _, f := range bodies[0].Faces() {
		for _, o := range f.Origins() {
			if o.Role == "patch" {
				caps++
			}
		}
	}
	if caps != 2 {
		t.Fatalf("%s joined: %d planar end faces, want 2 (none at the seam)", g.label, caps)
	}
}

// ---------------------------------------------------------------------------
// Sleeve tube (§4).

var sleeveTubeCases = []proofkit3d.Case{
	{Name: "defaults", Params: sgDefaults()},
	{Name: "rise 25", Params: sgWith(map[string]float64{"cageRise": 25})},
	{Name: "cage radius 25", Params: sgWith(map[string]float64{"cageRadius": 25, "collarHalf": 3.5})},
	{Name: "collar half 2", Params: sgWith(map[string]float64{"collarHalf": 2})},
}

// sgTube is the Sleeve sketch's ring extruded symmetrically by cageRise each
// side of the selected plane, which the proof puts at z = 0.
func sgTube(t *testing.T, doc *decad.Document, m sgModel) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	ci := s.CreatePoint(0, 0)
	co := s.CreatePoint(0, 0)
	inner := s.CreateCircle(ci, m.Ri)
	outer := s.CreateCircle(co, m.Ro)
	s.Fix(ci)
	s.Fix(co)
	s.AddConstraint(sketch.NewDiameter(inner, 2*m.Ri), sketch.NewDiameter(outer, 2*m.Ro))
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("Sleeve sketch: %v", err)
	}
	var ring *sketch.Profile
	for _, pr := range s.Profiles() {
		if pr.Valid && len(pr.Holes) == 1 {
			if ring != nil {
				t.Fatal("Sleeve: two profiles with two loops")
			}
			ring = pr
		}
	}
	if ring == nil {
		t.Fatal("Sleeve: no profile with two loops")
	}
	body, err := doc.Extrude(s, ring, decad.Symmetric{D: mm(m.cageRise)})
	if err != nil {
		t.Fatalf("Sleeve extrude: %v", err)
	}
	return body
}

// stepSleeveTube is the tube: the ring between Ri and Ro extruded as a new
// body with a symmetric extent of cageRise each side.
func stepSleeveTube(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{sgTube(t, doc, newModel(p))}
}

func assertSleeveTube(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	// pi*(Ro² - Ri²)*2*cageRise: 21,205.75 mm³ at the defaults, 37.5 mm tall.
	want := math.Pi * (m.Ro*m.Ro - m.Ri*m.Ri) * 2 * m.cageRise
	sgMeasuresVolume(t, "Cage tube", bodies[0], want, 1e-9*want)
	decadtest.MeasuresBounds(t, bodies[0], r3.NewVec(-m.Ro, -m.Ro, -m.cageRise), r3.NewVec(m.Ro, m.Ro, m.cageRise), decadtest.WithinRel(units.Scalar(1e-9)))
}

// ---------------------------------------------------------------------------
// Bore cuts (§4).

// boreCutCases is the four bores cut in the build's order at the defaults,
// each case the cage after that bore, and the cage after all four with no
// roof allowance and with gear A's +R bore the level one.
var boreCutCases = []proofkit3d.Case{
	{Name: "after Gear A Bore -R", Params: sgWith(map[string]float64{"bore": 0})},
	{Name: "after Gear A Bore +R", Params: sgWith(map[string]float64{"bore": 1})},
	{Name: "after Gear B Bore -R", Params: sgWith(map[string]float64{"bore": 2})},
	{Name: "after Gear B Bore +R", Params: sgWith(map[string]float64{"bore": 3})},
	{Name: "no roof allowance, after all four", Params: sgWith(map[string]float64{"bore": 3, "roofAllowance": 0})},
	{Name: "mount -20 and 30, after all four", Params: sgWith(map[string]float64{"bore": 3, "mountAngleA": -20, "mountAngleB": 30})},
}

// sgBoreSections is the stand-in channel's sections: the bore's rectangle,
// u from -hw to hw and v from vLo to vHi, turned by s/Lambda + Phi_g at
// boreSections() stations evenly spaced over the cut's span.
func sgBoreSections(m sgModel, b sgBore) []sgSection {
	g := m.gears[b.gear]
	n := m.boreSections()
	out := make([]sgSection, n)
	for k := range n {
		s := b.from + (b.to-b.from)*float64(k)/float64(n-1)
		th := g.theta(s)
		xy := make([][2]float64, 4)
		for i, c := range [4][2]float64{{-m.hw, b.vLo}, {m.hw, b.vLo}, {m.hw, b.vHi}, {-m.hw, b.vHi}} {
			xy[i][0], xy[i][1] = sgTurn(c[0], c[1], th)
		}
		out[k] = sgSection{origin: g.origin.Add(g.dir.Scale(s)), u: g.u, v: g.v, xy: xy}
	}
	return out
}

// sgUnion folds bodies into one by Union, in order.
func sgUnion(t *testing.T, bodies []*decad.Body, label string) *decad.Body {
	t.Helper()
	acc := bodies[0]
	for i, b := range bodies[1:] {
		u, err := decad.Union(t.Context(), acc, b)
		if err != nil {
			t.Fatalf("%s: union of tool %d: %v", label, i+1, err)
		}
		acc = u
	}
	return acc
}

// sgBoreTools builds the stand-in channels of bores 0..last.
func sgBoreTools(t *testing.T, doc *decad.Document, m sgModel, last int) []*decad.Body {
	t.Helper()
	var tools []*decad.Body
	for i, b := range m.bores() {
		if i > last {
			break
		}
		tools = append(tools, sgChainSolid(t, doc, sgBoreSections(m, b), b.name+" channel"))
	}
	return tools
}

// stepBoreCut is the cage after the twisted sweep cuts of bores 0..i. Each
// sweep cut has the cage as its only participant; the proof's document holds
// no ribbon, so the cut touches nothing else here, and that the ribbons are
// left whole is Fusion's, measured on 2026-09-28. STAND-IN: each sweep is the
// chain-of-lofts channel, and the cuts so far are one cut by their union.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	last := int(p["bore"])
	tube := sgTube(t, doc, m)
	tool := sgUnion(t, sgBoreTools(t, doc, m, last), "bore channels")
	cage, err := decad.Cut(t.Context(), tube, tool)
	if err != nil {
		t.Fatalf("cut through %s: %v", m.bores()[last].name, err)
	}
	return []*decad.Body{cage}
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	last := int(p["bore"])
	tube := math.Pi * (m.Ro*m.Ro - m.Ri*m.Ri) * 2 * m.cageRise
	if v := sgMM3(t, sgVolume(t, bodies[0]).Value); !(v < tube) {
		t.Fatalf("the cage after the cuts holds %.3f mm³, not less than the tube's %.3f", v, tube)
	}
	// [SCREW-F-SWEEP-CHECK]: the two probes at each crossing,
	// origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc), sc = sigma*cageRadius,
	// stand inside the wall (between Ri and Ro, within ±cageRise) and must be
	// outside the cage, which here means inside the channel cut there.
	for i, b := range m.bores() {
		if i > last {
			break
		}
		g := m.gears[b.gear]
		sc := b.sigma * m.cageRadius
		th := g.theta(sc)
		uHat := g.u.Scale(math.Cos(th)).Add(g.v.Scale(math.Sin(th)))
		for _, sgn := range []float64{1, -1} {
			pt := g.origin.Add(g.dir.Scale(sc)).Add(uHat.Scale(sgn * (m.W/2 + m.clearance/2)))
			rad := math.Hypot(pt.X, pt.Y)
			if rad <= m.Ri || rad >= m.Ro || math.Abs(pt.Z) >= m.cageRise {
				t.Fatalf("%s probe %+.0f at %v is not inside the wall (radius %.3f)", b.name, sgn, pt, rad)
			}
			ch := sgBoreSections(m, b)
			if !sgContains(t, func(d *decad.Document) *decad.Body { return sgChainSolid(t, d, ch, b.name+" probe channel") }, pt) {
				t.Fatalf("%s: the probe %+.0f at %v, %.3f mm from the frame's axis, is not in the channel", b.name, sgn, pt, rad)
			}
		}
	}
}

// sgContains reports whether a 0.02 mm cube about pt lies inside the body mk
// builds, by intersecting the two in a document of their own: an empty
// intersection is outside, the whole cube is inside, and anything else means
// the probe sits on the boundary, which fails. The cube is far smaller than the
// probes' 0.1 mm clearance from every wall the spec places them by.
func sgContains(t *testing.T, mk func(*decad.Document) *decad.Body, pt r3.Vec) bool {
	t.Helper()
	const h = 0.01
	doc := decad.New()
	body := mk(doc)
	w := sketch.NewWorld()
	f, err := r3.NewFrame(pt.Add(r3.NewVec(0, 0, -h)), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		t.Fatal(err)
	}
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatal(err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatal(err)
	}
	c := [4]*sketch.Point{s.CreatePoint(-h, -h), s.CreatePoint(h, -h), s.CreatePoint(h, h), s.CreatePoint(-h, h)}
	for i := range c {
		s.CreateLine(c[i], c[(i+1)%4])
	}
	for _, p := range c {
		s.Fix(p)
	}
	cube, err := doc.Extrude(s, decadtest.SolveRegion(t, s), decad.Distance{D: mm(2 * h), Dir: decad.Along})
	if err != nil {
		t.Fatal(err)
	}
	in, err := decad.Intersect(t.Context(), body, cube)
	var be *decad.BooleanError
	if errors.As(err, &be) && be.Code == decad.BooleanEmpty {
		return false
	}
	if err != nil {
		t.Fatalf("probe at %v: %v", pt, err)
	}
	v := sgMM3(t, sgVolume(t, in).Value)
	if math.Abs(v-8*h*h*h) > 1e-3*8*h*h*h {
		t.Fatalf("probe at %v straddles a face: %.3g of %.3g mm³ inside", pt, v, 8*h*h*h)
	}
	return true
}

// ---------------------------------------------------------------------------
// Window cuts (§4).

var windowCutCases = []proofkit3d.Case{
	{Name: "after Window +k", Params: sgWith(map[string]float64{"window": 0})},
	{Name: "after Window -k", Params: sgWith(map[string]float64{"window": 1})},
}

// sgWindowPrism is one window's cut: the hexagon on the Window Plane pushed
// out along d. STAND-IN for where it starts: Fusion extrudes from the Window
// Plane itself, through the frame's axis, by Ro + 1 mm; on that plane the two
// windows' hexagons overlap, and decad refuses a union of two prisms that
// meet face to face there. The stand-in starts 1 mm out along d and runs to
// the same far end, Ro + 1 mm. The wall begins at a0(t) = sqrt(Ri² - t²), more
// than 1 mm out at every t of the hexagon (asserted), so it removes the same
// wall; what it leaves is air in the hollow.
func sgWindowPrism(t *testing.T, doc *decad.Document, m sgModel, win sgWindow) *decad.Body {
	t.Helper()
	const start = 1.0
	q := win.corners(m.Ro)
	for _, c := range q {
		if math.Sqrt(math.Max(0, m.Ri*m.Ri-c[0]*c[0])) <= start {
			t.Fatalf("%s: the wall reaches within %.1f mm of the plane at t = %.3f; the stand-in would miss it", win.name, start, c[0])
		}
	}
	w := sketch.NewWorld()
	f, err := r3.NewFrame(win.d.Scale(start), win.across(), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatal(err)
	}
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatal(err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatal(err)
	}
	pts := make([]*sketch.Point, len(q))
	for i, c := range q {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, p := range pts {
		s.Fix(p)
	}
	// The plane's normal is across × n̂ = d, so Along runs toward d: the
	// build's PositiveExtentDirection when modelToSketchSpace(C + d).z > 0.
	body, err := doc.Extrude(s, decadtest.SolveRegion(t, s), decad.Distance{D: mm(m.Ro + 1 - start), Dir: decad.Along})
	if err != nil {
		t.Fatalf("%s extrude: %v", win.name, err)
	}
	return body
}

// sgWindowProbe is §4's probe for a window: C + tc*across + zc*n̂ +
// ((a0(tc) + a1(tc))/2)*d, (tc, zc) the average of the hexagon's corners.
func sgWindowProbe(m sgModel, win sgWindow) r3.Vec {
	q := win.corners(m.Ro)
	var tc, zc float64
	for _, c := range q {
		tc += c[0]
		zc += c[1]
	}
	tc /= float64(len(q))
	zc /= float64(len(q))
	a0 := math.Sqrt(math.Max(0, m.Ri*m.Ri-tc*tc))
	a1 := math.Sqrt(math.Max(0, m.Ro*m.Ro-tc*tc))
	return win.across().Scale(tc).Add(r3.NewVec(0, 0, zc)).Add(win.d.Scale((a0 + a1) / 2))
}

// stepWindowCut is the cage after the bores and the windows up to this one,
// each an extrude cut of the window's hexagon from the Window Plane toward d
// by Ro + 1 mm with the cage as the only participant. STAND-IN: one cut of
// the tube by the union of the bore channels and the window prisms so far.
func stepWindowCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := newModel(p)
	last := int(p["window"])
	tube := sgTube(t, doc, m)
	tools := sgBoreTools(t, doc, m, 3)
	for i, win := range sgDefaultWindows() {
		if i > last {
			break
		}
		tools = append(tools, sgWindowPrism(t, doc, m, win))
	}
	cage, err := decad.Cut(t.Context(), tube, sgUnion(t, tools, "bore channels and windows"))
	if err != nil {
		t.Fatalf("cut through %s: %v", sgDefaultWindows()[last].name, err)
	}
	return []*decad.Body{cage}
}

func assertWindowCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := newModel(p)
	last := int(p["window"])
	for i, win := range sgDefaultWindows() {
		if i > last {
			break
		}
		pt := sgWindowProbe(m, win)
		// Before the cut the probe is in the middle of the wall: in the tube
		// (between Ri and Ro, within ±cageRise) and in no bore's channel.
		rad := math.Hypot(pt.X, pt.Y)
		if rad <= m.Ri || rad >= m.Ro || math.Abs(pt.Z) >= m.cageRise {
			t.Fatalf("%s probe at %v is not in the tube (radius %.3f)", win.name, pt, rad)
		}
		for _, b := range m.bores() {
			ch := sgBoreSections(m, b)
			if sgContains(t, func(d *decad.Document) *decad.Body { return sgChainSolid(t, d, ch, b.name+" probe channel") }, pt) {
				t.Fatalf("%s probe at %v is in %s's channel before the cut", win.name, pt, b.name)
			}
		}
		// After the cut it is outside the cage: inside the window's cut.
		if !sgContains(t, func(d *decad.Document) *decad.Body { return sgWindowPrism(t, d, m, win) }, pt) {
			t.Fatalf("%s probe at %v is not in the window's cut", win.name, pt)
		}
	}
	if last == 1 {
		// TestSleeveIsOnePiece's flood fill reads 16,561 mm³ at the defaults on
		// a 0.25 mm grid, a count of cells and not an exact volume. The
		// stand-in's channels stand inside the swept ones by up to the triangle
		// pair's |T|/4 (file comment), so it removes a little less than Fusion's
		// sweeps do: 16,647 mm³ at the pinned revision, 0.5% over. 1% holds the
		// sleeve to the spec's number without pretending to the grid's precision.
		t.Logf("Cage: %.1f mm³ against the flood fill's 16,561", sgMM3(t, sgVolume(t, bodies[0]).Value))
		sgMeasuresVolume(t, "Cage", bodies[0], 16561, 0.01*16561)
	}
}
