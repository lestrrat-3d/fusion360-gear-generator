package screwgear_test

// The solid steps of spec/screwgear/steps.md, one function each, gated by
// proofkit3d.RunSolid: every body valid, one lump, no voids, and nothing in
// the document's report but the area or centroid reading a faceted boolean
// leaves outside the default tolerance.
//
// Three things decad cannot do as Fusion does shape what these build, and
// each is said again beside the function it touches:
//
//   - decad lofts between two sections only. A loft through many sections is
//     a sheet lofted between each neighbouring pair, the two end sections
//     patched, and all of them stitched (sectionSolid). Each wall between two
//     sections is two flat triangles, not Fusion's smooth surface and not the
//     ruled surface the spec's chord arithmetic describes; sectionVolume says
//     how far that can move a volume.
//   - decad has no twisted sweep (a nonzero WithSweepTwist is ErrUnsupported),
//     so each bore is that lofted stand-in through the sections boreSections
//     derives, as the spec's "What the proof's stand-in costs" prescribes.
//   - decad will not join two solids that meet on a shared planar face (a
//     face-on-face contact its exact predicates do not classify), will not
//     weld a moved copy's edges to a body's (the moved coordinates differ in
//     the last bits), and will not cut a second tool from a body a faceted
//     cut has already made (its held mesh bound is coarser than the next
//     cut's chord tolerance). So a join is proven by stitching the two pieces
//     from one shared station list and holding the result's volume to the sum
//     of the pieces built apart; and the four bore cuts and the two window
//     cuts are each proven as one cut of the union of their tools, which
//     removes the same material, since no two of the tools meet.
//
// A body's own name in a failure comes from the label each check passes.

import (
	"context"
	"math"
	"math/bits"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cellPieceRange is the global section range a cell case names: from the
// section of its first tooth to the section `teeth` teeth on.
func cellPieceRange(p Params, m map[string]float64) (int, int) {
	n := cellSteps(p)
	first, teeth := int(m["firstTooth"]), int(m["teeth"])
	return first * n, (first + teeth) * n
}

// caseAxisGear is the gear a case names, laid on the X axis (axisGear).
func caseAxisGear(t testing.TB, m map[string]float64) (Params, Gear) {
	t.Helper()
	p, g := caseGear(m)
	requireAccepted(t, p, m)
	return p, axisGear(g)
}

// stepCellLoft is a gear's cell loft: one solid through the cell's teeth*n + 1
// sections in station order, each the rectangle from the back edge to the
// toothed edge, T thick, turned to the section's angle. decad lofts it as
// sectionSolid does: ruled between each pair, walls split into triangles.
func stepCellLoft(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p, g := caseAxisGear(t, m)
	from, to := cellPieceRange(p, m)
	piece := buildRibbonPiece(t, doc, g, from, to)
	return []*decad.Body{piece.body}
}

func assertCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p, g := caseGear(m)
	g = axisGear(g)
	from, to := cellPieceRange(p, m)
	if len(bodies) != 1 {
		t.Fatalf("the loft leaves %d bodies, want 1", len(bodies))
	}
	piece := ribbonPiece{body: bodies[0]}
	for j := from; j <= to; j++ {
		s := ribbonStation(g, j)
		piece.stations = append(piece.stations, s)
		piece.sections = append(piece.sections, g.section(s))
	}
	if got, want := len(piece.sections), int(m["teeth"])*cellSteps(p)+1; got != want {
		t.Fatalf("the cell lofts through %d sections, want teeth*n + 1 = %d", got, want)
	}
	checkRibbonPiece(t, "cell", piece)
}

// stepCopyBody is a doubling round's copy: copyPasteBodies of the body, here
// Duplicate of the cell, which leaves the source live and gives an identical
// body of its own. The copy sits exactly on its source until the next step
// moves it, and a document holding two coincident solids is not one decad
// verifies as sound, so the proof hands the gate the copy alone: the source
// is built and copied in a document of its own, the copy is what this step
// returns, and the assertion holds it to the source.
func stepCopyBody(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p, g := caseAxisGear(t, m)
	from, to := cellRange(p, 0)
	cell := buildRibbonPiece(t, doc, g, from, to)
	dup, err := cell.body.Duplicate(context.Background())
	if err != nil {
		t.Fatalf("copy the cell: %v", err)
	}
	// decad retires a body only by consuming it, so the source is moved four
	// ribbon lengths down the axis, clear of the copy, rather than left where
	// it is.
	away, err := r3.Translation(g.Ez.Scale(-4 * p.Length()))
	if err != nil {
		t.Fatalf("translation: %v", err)
	}
	if _, err := cell.body.Placed(context.Background(), away); err != nil {
		t.Fatalf("set the source aside: %v", err)
	}
	return []*decad.Body{dup}
}

func assertCopyBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p, g := caseGear(m)
	g = axisGear(g)
	from, to := cellRange(p, 0)
	var copied *decad.Body
	for _, b := range bodies {
		copied = b
	}
	ref := decad.New()
	source := buildRibbonPiece(t, ref, g, from, to)
	got, err := copied.Volume()
	if err != nil {
		t.Fatalf("copy volume: %v", err)
	}
	want, err := source.body.Volume()
	if err != nil {
		t.Fatalf("source volume: %v", err)
	}
	decadtest.Agree(t, "copy volume against its source", got, want)
	box, err := copied.Bounds()
	if err != nil {
		t.Fatalf("copy bounds: %v", err)
	}
	lo, hi := cornerBox(source.sections)
	// An unmoved copy keeps the source's vertices exactly.
	decadtest.MeasuresBox(t, "copy bounds against the source's corners", box, lo, hi,
		decadtest.Within(units.Millimeters(1e-9)))
}

// stepScrewMove is a doubling round's move: the copy of the cell moved by the
// screw step Step(shift*c), a rotation of shift*c*P/Lambda about the gear's
// axis composed with a translation of shift*c*P along it. The proof moves the
// cell itself, which is the copy's geometry, so the document holds only the
// moved body.
func stepScrewMove(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p, g := caseAxisGear(t, m)
	c, _, _ := cellCount(p)
	from, to := cellRange(p, 0)
	cell := buildRibbonPiece(t, doc, g, from, to)
	moved, err := cell.body.Placed(context.Background(), screwStep(t, g, int(m["shift"])*c))
	if err != nil {
		t.Fatalf("screw-move the copy: %v", err)
	}
	return []*decad.Body{moved}
}

// assertScrewMove holds the moved copy to the cell built in place shift cells
// along, in a document of its own: the same volume, the same box to rounding,
// and the same centroid. That is the spec's claim that every placement is
// exact because the ribbon is invariant under its screw step, read on the
// solid; TestRibbonIsInvariantUnderItsScrewStep holds it on the model.
func assertScrewMove(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p, g := caseGear(m)
	g = axisGear(g)
	if len(bodies) != 1 {
		t.Fatalf("the move leaves %d bodies, want 1", len(bodies))
	}
	moved := bodies[0]
	from, to := cellRange(p, int(m["shift"]))
	ref := decad.New()
	there := buildRibbonPiece(t, ref, g, from, to)

	got, err := moved.Volume()
	if err != nil {
		t.Fatalf("moved volume: %v", err)
	}
	want, err := there.body.Volume()
	if err != nil {
		t.Fatalf("in-place volume: %v", err)
	}
	decadtest.Agree(t, "moved copy volume against the cell built there", got, want)

	// The vertices, rather than the box: decad bounds a placed body by its
	// placed source box, which a rotation loosens, so the box of a moved body
	// is not tight. Every vertex of the moved copy lands on a corner of the
	// cell built there, to the rounding of a rigid motion of up to 180 mm.
	checkVerticesOn(t, "moved copy", moved, there.sections, 1e-9)

	gc, err := moved.Centroid()
	if err != nil {
		t.Fatalf("moved centroid: %v", err)
	}
	wc, err := there.body.Centroid()
	if err != nil {
		t.Fatalf("in-place centroid: %v", err)
	}
	// Both are readings; the in-place one's own bound is added to the slack.
	decadtest.MeasuresVec(t, "moved copy centroid against the cell built there", gc, wc.Value,
		decadtest.Within(wc.Bound))
}

// checkVerticesOn holds a body's vertices to a run of section corners: as
// many vertices as corners, and each corner the position of one of them.
func checkVerticesOn(t *testing.T, name string, body *decad.Body, sections [][4]r3.Vec, slack float64) {
	t.Helper()
	vs := body.Vertices()
	if got, want := len(vs), 4*len(sections); got != want {
		t.Fatalf("%s has %d vertices, want %d, four per section", name, got, want)
	}
	for _, sec := range sections {
		for _, c := range sec {
			best := vs[0].Position()
			for _, v := range vs[1:] {
				if pos := v.Position(); pos.Value.Sub(c).Len() < best.Value.Sub(c).Len() {
					best = pos
				}
			}
			decadtest.MeasuresVec(t, name+" vertex at a section corner", best, c,
				decadtest.Within(units.Millimeters(slack)))
		}
	}
}

// joinSeam is one join of the build: the body's last cell and the first cell
// of the piece joined to it, by their global section ranges.
type joinSeam struct {
	name             string
	bodyFrom, bodyTo int
	copyFrom, copyTo int
}

// joinSeams lists every join the build makes for a case, in build order:
// each doubling and each placed aside, at the cell where the copy meets the
// body, and the remainder when there is one. It also checks the schedule
// itself: floor(log2 q) + popcount(q) - 1 rounds, each copy moved by exactly
// the cells the body holds, so each one starts where the body ends, and the
// last bringing the body to q cells.
func joinSeams(t testing.TB, p Params) []joinSeam {
	t.Helper()
	_, q, r := cellCount(p)
	ops := cellSchedule(q)
	rounds, m := 0, 1
	var seams []joinSeam
	for _, op := range ops {
		if op.kind == "aside" {
			continue
		}
		rounds++
		if op.cells != m || op.shift != m {
			t.Fatalf("a %s round moves its copy by %d cells onto a body of %d, want the body's %d",
				op.kind, op.shift, op.cells, m)
		}
		bf, bt := cellRange(p, m-1)
		cf, ct := cellRange(p, m)
		seams = append(seams, joinSeam{name: op.kind, bodyFrom: bf, bodyTo: bt, copyFrom: cf, copyTo: ct})
		m += op.piece
	}
	if want := bits.Len(uint(q)) - 1 + bits.OnesCount(uint(q)) - 1; rounds != want {
		t.Fatalf("the schedule for %d cells makes %d rounds, want floor(log2 q) + popcount(q) - 1 = %d", q, rounds, want)
	}
	if m != q {
		t.Fatalf("the schedule ends with %d cells, want %d", m, q)
	}
	if r > 0 {
		bf, bt := cellRange(p, q-1)
		rf, rt := remainderRange(p)
		seams = append(seams, joinSeam{name: "remainder", bodyFrom: bf, bodyTo: bt, copyFrom: rf, copyTo: rt})
	}
	return seams
}

// stepJoinCopy is a join: combineFeatures joining the moved copy, or the
// remainder cell, into the body, leaving one body. decad refuses to join two
// solids that meet on a face, and a moved copy's edges do not weld to the
// body's, so each join is proven on the two cells that meet at it: lofted from
// one station list, the shared section computed once, into one stitched body.
// The assertion holds that body's volume to the two cells' built apart, so the
// join adds nothing and loses nothing at the seam. Every join of the case's
// schedule is checked; all but the last are built and gated in documents of
// their own, and the last is what this step hands the gate. For one cell and
// no remainder there is no join, and the body the step leaves is the cell.
//
// The whole default ribbon, 68 teeth and 681 sections, is not built: decad's
// stitch refuses it (its crossing audit exceeds a fixed work ceiling between
// 48 and 60 teeth at the defaults), so a join is proven at its seam rather
// than on the body it makes. The seams of one schedule are the same two cells
// carried along by the screw step, which stepScrewMove holds exact.
func stepJoinCopy(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p, g := caseAxisGear(t, m)
	seams := joinSeams(t, p)
	if len(seams) == 0 {
		from, to := cellRange(p, 0)
		return []*decad.Body{buildRibbonPiece(t, doc, g, from, to).body}
	}
	for i, sm := range seams {
		target := doc
		if i < len(seams)-1 {
			target = decad.New()
		}
		joined := buildRibbonPiece(t, target, g, sm.bodyFrom, sm.copyTo)
		checkSeam(t, g, sm, joined)
		if i < len(seams)-1 {
			proofkit3d.RequireSolid(t, target, []*decad.Body{joined.body})
		} else {
			return []*decad.Body{joined.body}
		}
	}
	panic("unreachable")
}

// checkSeam holds a joined pair of cells to the two cells built apart.
func checkSeam(t *testing.T, g Gear, sm joinSeam, joined ribbonPiece) {
	t.Helper()
	checkRibbonPiece(t, sm.name+" join", joined)
	a := buildRibbonPiece(t, decad.New(), g, sm.bodyFrom, sm.bodyTo)
	b := buildRibbonPiece(t, decad.New(), g, sm.copyFrom, sm.copyTo)
	va, err := a.body.Volume()
	if err != nil {
		t.Fatalf("body cell volume: %v", err)
	}
	vb, err := b.body.Volume()
	if err != nil {
		t.Fatalf("copy cell volume: %v", err)
	}
	vj, err := joined.body.Volume()
	if err != nil {
		t.Fatalf("joined volume: %v", err)
	}
	sum, err := va.Value.Add(vb.Value)
	if err != nil {
		t.Fatalf("sum the cells: %v", err)
	}
	bound, err := va.Bound.Add(vb.Bound)
	if err != nil {
		t.Fatalf("sum the bounds: %v", err)
	}
	decadtest.Agree(t, sm.name+" join volume against its two cells", vj,
		decad.Measurement{Value: sum, Exactness: decad.Approximate, Bound: bound})
}

func assertJoinCopy(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p, g := caseGear(m)
	g = axisGear(g)
	if len(bodies) != 1 {
		t.Fatalf("the join leaves %d bodies, want 1", len(bodies))
	}
	seams := joinSeams(t, p)
	from, to := cellRange(p, 0)
	name := "cell with no join"
	if len(seams) > 0 {
		last := seams[len(seams)-1]
		from, to, name = last.bodyFrom, last.copyTo, last.name+" join"
	}
	piece := ribbonPiece{body: bodies[0]}
	for j := from; j <= to; j++ {
		s := ribbonStation(g, j)
		piece.stations = append(piece.stations, s)
		piece.sections = append(piece.sections, g.section(s))
	}
	checkRibbonPiece(t, name, piece)
	if m["toothCount"] == 0 {
		// The defaults the spec quotes: 17 cells, five rounds, no remainder.
		_, q, r := cellCount(p)
		if q != 17 || r != 0 || len(seams) != 5 {
			t.Errorf("the defaults make %d cells, a remainder of %d and %d joins, want 17, 0 and 5", q, r, len(seams))
		}
	}
}

// stepSleeveTube is the sleeve's tube: the Sleeve sketch's ring extruded both
// ways from the selected plane by cageRise each side.
func stepSleeveTube(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p := caseParams(m)
	requireAccepted(t, p, m)
	return []*decad.Body{tubeBody(t, doc, p)}
}

func assertSleeveTube(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p := caseParams(m)
	ri, ro, h := p.SleeveInner(), p.SleeveOuter(), p.CageRise
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("tube volume: %v", err)
	}
	// pi*(Ro^2 - Ri^2)*2*cageRise, exact but for float rounding.
	decadtest.Measures(t, "tube volume", vol, units.CubicMillimeters(math.Pi*(ro*ro-ri*ri)*2*h),
		decadtest.WithinRel(units.Scalar(1e-9)))
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("tube bounds: %v", err)
	}
	decadtest.MeasuresBox(t, "tube bounds", box, r3.NewVec(-ro, -ro, -h), r3.NewVec(ro, ro, h),
		decadtest.Within(units.Millimeters(1e-9)))
}

// boreTools builds the stand-in for each of the four bores' sweeps, in the
// build's order (gear A -R, gear A +R, gear B -R, gear B +R), and returns them
// with the volume each removes from the uncut tube and the slack on it.
func boreTools(t testing.TB, doc *decad.Document, f sleeve) ([]*decad.Body, float64, float64) {
	t.Helper()
	var tools []*decad.Body
	removal, slack := 0.0, 0.0
	for _, b := range f.bores() {
		tool, fold := boreTool(t, doc, f, b)
		tools = append(tools, tool)
		removal += boreRemoval(f, b)
		// The triangle fold of the stand-in's walls, the facets standing inside
		// the exact channel, and the trapezoid rule at 1 µm, which is under
		// 0.01 mm^3 on these spans.
		slack += fold + boreFacetSlack(f, b) + 0.01
	}
	return tools, removal, slack
}

// cutAll unions disjoint tools and cuts them from target in one cut.
func cutAll(t testing.TB, target *decad.Body, tools []*decad.Body) *decad.Body {
	t.Helper()
	ctx := context.Background()
	all := tools[0]
	for _, tool := range tools[1:] {
		var err error
		all, err = decad.Union(ctx, all, tool)
		if err != nil {
			t.Fatalf("gather the tools: %v", err)
		}
	}
	cut, err := decad.Cut(ctx, target, all)
	if err != nil {
		t.Fatalf("cut: %v", err)
	}
	return cut
}

// stepBoreSweepCut is the four bores' twisted sweep cuts, each from the cage
// alone, by the stand-in boreTool builds for each: the bore's rectangle turned
// to the ribbon's own angle at boreSections' stations over the cut's span,
// sIn to sOut or -sOut to -sIn. decad has no twisted sweep; the stand-in's
// cost against the swept channel is what TestSleeveBoreSubstituteKeepsItsClearance
// bounds for a ruled channel, and decad's walls fold that ruled channel into
// triangles as well (sectionSolid). The four cuts are one cut of the four
// tools' union (see the file comment); no two channels meet, which
// channelSeparation holds to collarWall.
func stepBoreSweepCut(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p, gs := caseGears(m)
	requireAccepted(t, p, m)
	f := plainSleeve(gs[0], gs[1])
	tube := tubeBody(t, doc, p)
	tools, _, _ := boreTools(t, doc, f)
	return []*decad.Body{cutAll(t, tube, tools)}
}

// assertBoreSweepCut holds what the cuts took to what the channels hold in
// the wall: the tube's volume less, for each bore, its section in the wall
// (sectionInWall) integrated along its span.
func assertBoreSweepCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	_, gs := caseGears(m)
	f := plainSleeve(gs[0], gs[1])
	p := f.p
	tubeVol := math.Pi * (f.ro*f.ro - f.ri*f.ri) * 2 * p.CageRise
	_, removal, slack := boreTools(t, decad.New(), f)
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("cage volume: %v", err)
	}
	decadtest.Measures(t, "cage after the four bores", vol, units.CubicMillimeters(tubeVol-removal),
		decadtest.Within(units.CubicMillimeters(slack)))
	got := vol.Value.Base()
	t.Logf("the four bores take %.1f mm^3 from the tube by the channels' sections, %.1f mm^3 by the cut, within %.1f mm^3",
		removal, tubeVol-got, slack)
}

// stepWindowExtrudeCut is the two windows' extrude cuts, each a hexagon on the
// plane through the frame's axis square to its facing direction, pushed one
// way, toward that direction, by Ro + 1 mm, from the cage alone.
//
// What the proof cuts them from is the uncut tube, not the bored cage the
// build cuts them from: decad will not make a second cut in a body a faceted
// cut has made, and the bores' tools cannot be gathered with the windows' into
// one cut, since in the hollow, where neither cuts anything, a bore's tool and
// a window's prism come within decad's chord tolerance of each other without
// provably crossing. So this is the windows alone, and that the windows keep
// collarWall from every channel, which is what lets the two kinds of cut be
// taken apart, is TestSleeveWindowsKeepTheirWalls's to hold.
//
// Each prism starts in the hollow, halfway to the inner face at the window's
// outermost end, rather than on the axis plane: the two windows' hexagons
// cross on the axis plane, and decad will not gather two prisms whose faces
// overlap in one plane. Between the axis plane and the inner face is the
// hollow, so the material each prism removes is the same.
func stepWindowExtrudeCut(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	p := caseParams(m)
	requireAccepted(t, p, m)
	f := caseSleeve(m)
	if len(f.windows) != 2 {
		t.Fatalf("the search finds room for %d windows, want 2", len(f.windows))
	}
	tube := tubeBody(t, doc, p)
	var tools []*decad.Body
	for _, w := range f.windows {
		tools = append(tools, windowPrism(t, doc, f, w))
	}
	return []*decad.Body{cutAll(t, tube, tools)}
}

// assertWindowExtrudeCut holds the tube after both windows to the tube less
// each window's prism where it meets the wall.
func assertWindowExtrudeCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	f := caseSleeve(m)
	p := f.p
	tubeVol := math.Pi * (f.ro*f.ro - f.ri*f.ri) * 2 * p.CageRise
	removal, slack := 0.0, 0.0
	for _, w := range f.windows {
		wr := windowRemoval(f, windowCorners(w))
		// The window's prism is exact; the slack is the trapezoid rule at 1 µm
		// over a chord whose slope stays finite inside the inner radius.
		removal += wr
		slack += 1e-4 * wr
		t.Logf("the window facing %s takes %.1f mm^3 from the wall", faceName(w.facing), wr)
	}
	vol, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("cage volume: %v", err)
	}
	decadtest.Measures(t, "tube after the windows", vol, units.CubicMillimeters(tubeVol-removal),
		decadtest.Within(units.CubicMillimeters(slack)))
	bores := 0.0
	for _, b := range f.bores() {
		bores += boreRemoval(f, b)
	}
	t.Logf("with the bores' %.1f mm^3 the sleeve holds %.1f mm^3", bores, tubeVol-removal-bores)
}
