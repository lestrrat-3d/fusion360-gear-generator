package screwgear_test

// The compiled step proof's solid steps, in decad. Each build returns the
// bodies its step leaves; each assertion checks what the spec pins on them.
//
// decad at the pinned revision cannot build four of this gear's features as
// Fusion builds them, and each step below names its substitute beside it:
//
//   - A loft through more than two sections. decad lofts two sections at a
//     time, and two lofts that share a section refuse to union ("two operand
//     facets overlap in one plane"). The stand-in is every two-section loft
//     built as a sheet, the two end caps patched, and the whole stitched into
//     one solid: the ruled loft through every section, which is what §2's
//     chord bounds describe. Fusion's own loft is smooth between sections
//     ([SCREW-F-CELL-LOFT]); how far that surface departs is Fusion's, measured
//     on 2026-09-28, and nothing here sees it.
//   - A join. A stitched solid takes no boolean ("a stitched body's mesh
//     carries no proof of the volume ... so no boolean may compose it"), and
//     two bodies that share a face read Suspect. The join's stand-in is the
//     joined body built directly as one ruled loft through all its sections,
//     held against the two pieces the build joins, each built in a document
//     of its own: equal volume to their sum, and the box that spans both.
//   - A copy. decad cannot verify a document holding two coincident bodies
//     ("unsupported_pair_contact"), which is what Fusion's copy leaves until
//     the move. The stand-in takes the copy with Placed under the identity,
//     which retires the source, and holds the copy against a cell built
//     afresh in a document of its own.
//   - A twisted sweep, and a cut by more than one tool. decad stages a nonzero
//     sweep twist as ErrUnsupported, and a tube cut by a chain of lofts or of
//     prisms refuses by the second to sixth cut ("the operands' held facets
//     come within the chord tolerance ..." and "requested tolerance ... is
//     below the faceted body's minimum mesh bound"). The bore's stand-in is the
//     channel itself, the ruled loft through the count of sections "What the
//     proof's stand-in costs" derives, held to that count, to its two ends in
//     the air, and to holding the build's two crossing probes; the cut of the
//     tube by it is not made here. That the sleeve stays one piece with all
//     four bores and both windows cut is the hand-written TestSleeveIsOnePiece.
//   - Two window cuts of one tube. The second window's cut of the tube the
//     first has cut refuses ("requested tolerance 0.00128 mm is below the
//     faceted body's minimum mesh bound 0.0052 mm"), so each window is cut
//     from a tube of its own, and the order of the two cuts is not exercised.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
)

// cpSection draws one rectangle section on the plane of frame f, solved, and
// returns its sketch and its one profile.
func cpSection(t *testing.T, w *sketch.World, f r3.Frame, q cpQuad) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("section plane at station %.4f: %v", q.S, err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("section sketch at station %.4f: %v", q.S, err)
	}
	cpDrawQuad(s, q)
	sketchtest.Solve(t, s)
	rep := sketchtest.IsTrustworthy(t, s)
	prof := sketchtest.SingleProfile(t, rep)
	sketchtest.IsValidProfile(t, prof)
	return s, prof
}

// cpRuledSolid is the stand-in for a loft through every section: each
// neighbouring pair lofted as a sheet, the end sections patched, all of it
// stitched into one solid. frame(q) places a section's plane.
func cpRuledSolid(t *testing.T, doc *decad.Document, secs []cpQuad, frame func(cpQuad) r3.Frame) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	var sheets []*decad.Body
	var first, prev *sketch.Sketch
	var firstP, prevP *sketch.Profile
	for k, q := range secs {
		s, p := cpSection(t, w, frame(q), q)
		if k == 0 {
			first, firstP = s, p
		} else {
			wall, err := doc.Loft(t.Context(), prev, prevP, s, p, decad.WithSurfaceResult())
			if err != nil {
				t.Fatalf("loft sections %d to %d: %v", k-1, k, err)
			}
			sheets = append(sheets, wall)
		}
		prev, prevP = s, p
	}
	capStart, err := doc.Patch(t.Context(), first, firstP)
	if err != nil {
		t.Fatalf("patch the first section: %v", err)
	}
	capEnd, err := doc.Patch(t.Context(), prev, prevP)
	if err != nil {
		t.Fatalf("patch the last section: %v", err)
	}
	body, err := decad.Stitch(t.Context(), append(sheets, capStart, capEnd)...)
	if err != nil {
		t.Fatalf("stitch %d walls and two caps: %v", len(sheets), err)
	}
	if !body.IsSolid() {
		t.Fatalf("the stitched loft through %d sections is not a solid", len(secs))
	}
	return body
}

// cpGearLocal places a ribbon section in the gear's own frame: Z along the
// axis from the ribbon's negative end, X along û, Y along v̂. Every ribbon
// solid of a case is built in this one frame, so a moved piece and a piece
// built in place are compared in the same coordinates.
func cpGearLocal(z0 float64) func(cpQuad) r3.Frame {
	return func(q cpQuad) r3.Frame {
		f, err := r3.NewFrame(r3.NewVec(0, 0, q.S-z0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		if err != nil {
			panic(err) // an axis-aligned frame is always valid
		}
		return f
	}
}

// cpRibbonSolid builds the teeth [from, from+teeth) of gear g's ribbon in
// place, in the gear-local frame.
func cpRibbonSolid(t *testing.T, doc *decad.Document, m cpModel, g cpGear, from, teeth int) *decad.Body {
	t.Helper()
	secs := m.ribbonSections(g, from, teeth)
	return cpRuledSolid(t, doc, secs, cpGearLocal(m.ribbonStart(g)))
}

// cpScrewStep is Step(k) of §3 in the gear-local frame: a turn of k*P/Lambda
// about the axis, then k*P along it.
func cpScrewStep(t *testing.T, m cpModel, k int) r3.Transform {
	t.Helper()
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(float64(k)*m.P/m.Lam))
	if err != nil {
		t.Fatalf("screw step rotation: %v", err)
	}
	tr, err := r3.Translation(r3.NewVec(0, 0, float64(k)*m.P))
	if err != nil {
		t.Fatalf("screw step translation: %v", err)
	}
	step, err := rot.Then(tr)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	return step
}

func cpMM3(v float64) units.Value { return units.CubicMillimeters(v) }

// cpVolumeOf reads a body's volume as value and bound, in mm³.
func cpVolumeOf(t *testing.T, what string, b *decad.Body) (float64, float64) {
	t.Helper()
	v, err := b.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	val, err := v.Value.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("%s volume value: %v", what, err)
	}
	bound, err := v.Bound.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("%s volume bound: %v", what, err)
	}
	return val, bound
}

// cpMeasuresRibbon holds a ribbon piece to the ruled volume through its
// sections and to the box of their corners, labelled with the piece's name.
// The volume's slack is the faceting allowance of cpFacetSlack plus a
// relative 1e-9 for the float64 Simpson sum; the box is exact up to 1e-9 mm,
// since a faceted ruled solid reaches its extremes at section corners.
func cpMeasuresRibbon(t *testing.T, name string, body *decad.Body, secs []cpQuad, z0 float64) {
	t.Helper()
	v, slack := cpRuledVolume(secs)
	got, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", name, err)
	}
	decadtest.Measures(t, name+" volume", got, cpMM3(v), decadtest.Within(cpMM3(slack+1e-9*v)))
	lo, hi := cpLocalBox(secs, z0)
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", name, err)
	}
	decadtest.MeasuresBox(t, name+" bounds", box, lo, hi, decadtest.Within(units.Millimeters(1e-9)))
}

// --- The tooth cell and the remainder ------------------------------------

var cpRibbonSolidCases = []proofkit3d.Case{
	{Name: "defaults, gear A", Params: cpGearParams(nil, 0)},
	{Name: "defaults, gear B", Params: cpGearParams(nil, 1)},
	{Name: "lead 400, eight-step floor, gear A", Params: cpGearParams(map[string]float64{"twistLead": 400}, 0)},
	{Name: "lead 20, twist bound, gear B", Params: cpGearParams(map[string]float64{"twistLead": 20}, 1)},
	{Name: "four teeth, gear A", Params: cpGearParams(map[string]float64{"toothCount": 4}, 0)},
	{Name: "tooth height near W/2, gear B", Params: cpGearParams(map[string]float64{"toothHeight": 7.4}, 1)},
	{Name: "mount -15, phase +2.6, gear B", Params: cpGearParams(map[string]float64{"mountAngleB": -15, "assemblyPhase": 2.6}, 1)},
	{Name: "mount 30, gear A", Params: cpGearParams(map[string]float64{"mountAngleA": 30}, 0)},
}

func cpGearParams(over map[string]float64, g int) map[string]float64 {
	p := cpWith(over)
	p["gear"] = float64(g)
	return p
}

var cpCellLoftCases = cpRibbonSolidCases

// stepCellLoft is the cell loft of §2: one loft through the c*n + 1 sections
// of the Cell Sections sketch, in station order, one body with six faces in
// Fusion. The stand-in is the ruled loft through the same sections (see the
// file comment).
func stepCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	return []*decad.Body{cpRibbonSolid(t, doc, m, g, 0, m.Cell)}
}

func assertCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if len(bodies) != 1 {
		t.Fatalf("%s cell loft left %d bodies, want 1", g.Label, len(bodies))
	}
	secs := m.ribbonSections(g, 0, m.Cell)
	cpMeasuresRibbon(t, g.Label+" cell", bodies[0], secs, m.ribbonStart(g))
}

var cpRemainderLoftCases = []proofkit3d.Case{
	{Name: "69 teeth, remainder 1, gear A", Params: cpGearParams(map[string]float64{"toothCount": 69}, 0)},
	{Name: "71 teeth, remainder 3, gear B", Params: cpGearParams(map[string]float64{"toothCount": 71}, 1)},
	{Name: "5 teeth, remainder 1, lead 400, gear B", Params: cpGearParams(map[string]float64{"toothCount": 5, "twistLead": 400}, 1)},
}

// stepCellRemainderLoft is the remainder's loft of §3, built where the last r
// teeth belong.
func stepCellRemainderLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if m.R == 0 {
		t.Fatalf("case leaves no remainder; the step does not run")
	}
	return []*decad.Body{cpRibbonSolid(t, doc, m, g, m.Q*m.Cell, m.R)}
}

func assertCellRemainderLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if len(bodies) != 1 {
		t.Fatalf("%s remainder loft left %d bodies, want 1", g.Label, len(bodies))
	}
	secs := m.ribbonSections(g, m.Q*m.Cell, m.R)
	cpMeasuresRibbon(t, g.Label+" remainder", bodies[0], secs, m.ribbonStart(g))
	// Its first section is the body's last: station s0 + q*c*P.
	sketchtest.Measures(t, "remainder's first station", secs[0].S, m.ribbonStart(g)+float64(m.Q*m.Cell)*m.P, sketchtest.Within(1e-9))
}

// --- Doubling: copy, move, join ------------------------------------------

var cpCopyCases = []proofkit3d.Case{
	{Name: "defaults, gear A", Params: cpGearParams(nil, 0)},
	{Name: "defaults, gear B", Params: cpGearParams(nil, 1)},
	{Name: "lead 400, gear B", Params: cpGearParams(map[string]float64{"twistLead": 400}, 1)},
}

// stepCopyBody is a copy of §3: the copy taken from the CopyPasteBody
// feature's bodies, identical to the body it copies. Stand-in: see the file
// comment.
func stepCopyBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	cell := cpRibbonSolid(t, doc, m, g, 0, m.Cell)
	cp, err := cell.Placed(t.Context(), r3.Identity())
	if err != nil {
		t.Fatalf("copy the cell: %v", err)
	}
	return []*decad.Body{cp}
}

func assertCopyBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if len(bodies) != 1 {
		t.Fatalf("%s copy left %d bodies, want 1", g.Label, len(bodies))
	}
	ref := cpRibbonSolid(t, decad.New(), m, g, 0, m.Cell)
	cpAgreeVolume(t, g.Label+" copy against its source", bodies[0], ref)
	cpMeasuresRibbon(t, g.Label+" copy", bodies[0], m.ribbonSections(g, 0, m.Cell), m.ribbonStart(g))
}

func cpAgreeVolume(t *testing.T, what string, a, b *decad.Body) {
	t.Helper()
	va, err := a.Volume()
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	vb, err := b.Volume()
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	// Both are readings of the same construction; the only slack beyond their
	// own bounds is float64 rounding in the placement, 1e-9 relative.
	decadtest.Agree(t, what, va, vb, decadtest.WithinRel(units.Scalar(1e-9)))
}

// cpMoveCases are the screw steps the doubling schedule makes, in teeth: at
// the defaults (17 cells of four teeth) the four doublings move by 4, 8, 16
// and 32 teeth and the aside by 64; at a one-tooth cell the aside moves by 64
// too. Gear B and a slow lead run the largest.
var cpMoveCases = []proofkit3d.Case{
	{Name: "Step(4), gear A", Params: cpMoveParams(nil, 0, 4)},
	{Name: "Step(8), gear A", Params: cpMoveParams(nil, 0, 8)},
	{Name: "Step(16), gear A", Params: cpMoveParams(nil, 0, 16)},
	{Name: "Step(32), gear A", Params: cpMoveParams(nil, 0, 32)},
	{Name: "Step(64), gear A", Params: cpMoveParams(nil, 0, 64)},
	{Name: "Step(64), gear B", Params: cpMoveParams(nil, 1, 64)},
	{Name: "Step(64), lead 400, gear B", Params: cpMoveParams(map[string]float64{"twistLead": 400}, 1, 64)},
	{Name: "Step(4), lead 20, mount -15, gear A", Params: cpMoveParams(map[string]float64{"twistLead": 20, "mountAngleA": -15}, 0, 4)},
}

func cpMoveParams(over map[string]float64, g, teeth int) map[string]float64 {
	p := cpGearParams(over, g)
	p["moveTeeth"] = float64(teeth)
	return p
}

// stepScrewMove is a move of §3: the free move by Step(k) of [SCREW-F-SCREW-STEP],
// a turn of k*P/Lambda about the gear's axis composed with k*P along it.
func stepScrewMove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	k := int(p["moveTeeth"])
	cell := cpRibbonSolid(t, doc, m, g, 0, m.Cell)
	moved, err := cell.Placed(t.Context(), cpScrewStep(t, m, k))
	if err != nil {
		t.Fatalf("move the cell by Step(%d): %v", k, err)
	}
	return []*decad.Body{moved}
}

// assertScrewMove holds the moved cell to the cell built in place k teeth
// further on: the ribbon is invariant under its own screw step, so the move
// lands the copy exactly where those teeth belong.
func assertScrewMove(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	k := int(p["moveTeeth"])
	if len(bodies) != 1 {
		t.Fatalf("%s move left %d bodies, want 1", g.Label, len(bodies))
	}
	ref := cpRibbonSolid(t, decad.New(), m, g, k, m.Cell)
	cpAgreeVolume(t, g.Label+" moved cell against the cell built in place", bodies[0], ref)
	cpAgreeCentroid(t, g.Label+" moved cell's centroid against the cell built in place", bodies[0], ref)
	// A placed body's box is the moved box of its source, axis-aligned again,
	// so only its extent along the turn's own axis stays tight: the moved cell
	// spans stations k*P to (k + c)*P from the ribbon's end.
	cpMeasuresAxialSpan(t, g.Label+" moved cell", bodies[0], float64(k)*m.P, float64(k+m.Cell)*m.P)
}

// cpAgreeCentroid holds two bodies' centroids together, each to its own bound.
func cpAgreeCentroid(t *testing.T, what string, a, b *decad.Body) {
	t.Helper()
	ca, err := a.Centroid()
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	cb, err := b.Centroid()
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	// The second reading's own bound is the slack on the first, plus 1e-9 mm
	// of float64 rounding in the placement.
	bound, err := cb.Bound.In(units.Millimeter)
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	decadtest.MeasuresVec(t, what, ca, cb.Value, decadtest.Within(units.Millimeters(bound+1e-9)))
}

// cpMeasuresAxialSpan holds a ribbon body's extent along the gear's axis.
func cpMeasuresAxialSpan(t *testing.T, what string, b *decad.Body, z0, z1 float64) {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", what, err)
	}
	bound, err := box.Bound.In(units.Millimeter)
	if err != nil {
		t.Fatalf("%s bounds: %v", what, err)
	}
	sketchtest.Measures(t, what+" first station", box.Min.Z, z0, sketchtest.Within(bound+1e-9))
	sketchtest.Measures(t, what+" last station", box.Max.Z, z1, sketchtest.Within(bound+1e-9))
}

// cpJoinCases are joins the schedule makes, in cells, plus the remainder's.
// At the defaults the joins are 1+1, 2+2, 4+4, 8+8 and then the aside 16+1;
// at 12 teeth (q = 3) they are 1+1 and the aside 2+1; at 20 teeth (q = 5)
// 1+1, 2+2 and the aside 4+1; at 28 teeth (q = 7) 1+1, 2+2, then the asides
// 4+2 and 6+1. A remainder joins the q cells at 21 and 5 teeth. The last two
// default joins and the remainder at 69 teeth are past what the stand-in can
// build (cpStitchCeiling) and are reported as unmodelled, not dropped.
var cpJoinCases = []proofkit3d.Case{
	{Name: "defaults, 1+1, gear A", Params: cpJoinParams(nil, 0, 1, 1, false)},
	{Name: "defaults, 2+2, gear A", Params: cpJoinParams(nil, 0, 2, 2, false)},
	{Name: "defaults, 4+4, gear B", Params: cpJoinParams(nil, 1, 4, 4, false)},
	{Name: "defaults, 8+8, gear A", Params: cpJoinParams(nil, 0, 8, 8, false)},
	{Name: "defaults, aside 16+1, gear B", Params: cpJoinParams(nil, 1, 16, 1, false)},
	{Name: "12 teeth, aside 2+1, gear A", Params: cpJoinParams(map[string]float64{"toothCount": 12}, 0, 2, 1, false)},
	{Name: "20 teeth, aside 4+1, gear B", Params: cpJoinParams(map[string]float64{"toothCount": 20}, 1, 4, 1, false)},
	{Name: "28 teeth, aside 4+2, lead 400, gear A", Params: cpJoinParams(map[string]float64{"toothCount": 28, "twistLead": 400}, 0, 4, 2, false)},
	{Name: "28 teeth, aside 6+1, mount -15, gear B", Params: cpJoinParams(map[string]float64{"toothCount": 28, "mountAngleB": -15, "assemblyPhase": 2.6}, 1, 6, 1, false)},
	{Name: "21 teeth, remainder 5 cells + 1 tooth, gear A", Params: cpJoinParams(map[string]float64{"toothCount": 21}, 0, 5, 1, true)},
	{Name: "5 teeth, remainder 1 cell + 1 tooth, lead 400, gear B", Params: cpJoinParams(map[string]float64{"toothCount": 5, "twistLead": 400}, 1, 1, 1, true)},
	{Name: "69 teeth, remainder 17 cells + 1 tooth, gear A", Params: cpJoinParams(map[string]float64{"toothCount": 69}, 0, 17, 1, true)},
}

// cpStitchCeiling is the most two-section walls the stand-in's stitch builds
// at the pinned decad: 480 built and 560 refused ("the loft crossing audit's
// facet-pair count exceeds the fixed work ceiling"), measured on the default
// ribbon. A join whose joined body needs more is a case decad cannot
// represent: the default ribbon's 8+8 and 16+1 joins (640 and 680 walls).
// What the stand-in proves at the smaller joins — no overlap, no gap, the
// piece starting where the body ends — is the same arithmetic at every size;
// the hand-written TestDoublingScheduleCoversTheRibbon runs the schedule to
// 512 teeth.
const cpStitchCeiling = 480

func cpJoinParams(over map[string]float64, g, body, piece int, remainder bool) map[string]float64 {
	p := cpGearParams(over, g)
	p["joinBody"], p["joinPiece"] = float64(body), float64(piece)
	if remainder {
		p["joinRemainder"] = 1
	}
	return p
}

// cpJoinTeeth is a join case's body and piece in teeth: a remainder's piece
// counts teeth already, every other piece counts cells.
func cpJoinTeeth(m cpModel, p map[string]float64) (int, int) {
	body := int(p["joinBody"]) * m.Cell
	if p["joinRemainder"] == 1 {
		return body, int(p["joinPiece"])
	}
	return body, int(p["joinPiece"]) * m.Cell
}

// stepJoinBodies is a join of §3, [SCREW-F-JOIN]: the body and the piece
// placed against it become one body. The stand-in builds that one body
// directly (see the file comment); the assertion holds it to the two pieces.
func stepJoinBodies(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	cpCheckJoinIsScheduled(t, m, p)
	body, piece := cpJoinTeeth(m, p)
	if walls := (body + piece) * m.Steps; walls > cpStitchCeiling {
		proofkit3d.Unmodelled(t, "the joined body needs %d two-section walls, past the %d the stand-in's stitch builds", walls, cpStitchCeiling)
	}
	return []*decad.Body{cpRibbonSolid(t, doc, m, g, 0, body+piece)}
}

// cpCheckJoinIsScheduled holds the case to a join the build really makes.
func cpCheckJoinIsScheduled(t *testing.T, m cpModel, p map[string]float64) {
	t.Helper()
	b, pc := int(p["joinBody"]), int(p["joinPiece"])
	if p["joinRemainder"] == 1 {
		if b != m.Q || pc != m.R {
			t.Fatalf("remainder join %d cells + %d teeth is not this build's (q = %d, r = %d)", b, pc, m.Q, m.R)
		}
		return
	}
	joins, _ := cpSchedule(m.Q)
	for _, j := range joins {
		if j.Body == b && j.Piece == pc {
			return
		}
	}
	t.Fatalf("join %d+%d is not in the schedule for q = %d: %v", b, pc, m.Q, joins)
}

func assertJoinBodies(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if len(bodies) != 1 {
		t.Fatalf("%s join left %d bodies, want 1", g.Label, len(bodies))
	}
	body, piece := cpJoinTeeth(m, p)
	secs := m.ribbonSections(g, 0, body+piece)
	cpMeasuresRibbon(t, g.Label+" joined body", bodies[0], secs, m.ribbonStart(g))

	// The two pieces, each in a document of its own, as the build makes them:
	// the body in place, and the piece either built in place (the remainder)
	// or copied from the ribbon's first teeth and moved by Step(body).
	a := cpRibbonSolid(t, decad.New(), m, g, 0, body)
	var b *decad.Body
	if p["joinRemainder"] == 1 {
		b = cpRibbonSolid(t, decad.New(), m, g, body, piece)
	} else {
		src := cpRibbonSolid(t, decad.New(), m, g, 0, piece)
		var err error
		b, err = src.Placed(t.Context(), cpScrewStep(t, m, body))
		if err != nil {
			t.Fatalf("move the piece by Step(%d): %v", body, err)
		}
	}
	va, ba := cpVolumeOf(t, "body", a)
	vb, bb := cpVolumeOf(t, "piece", b)
	got, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("joined volume: %v", err)
	}
	// The join neither overlaps nor leaves a gap: its volume is the pieces'
	// sum, to their own bounds and float64 rounding.
	decadtest.Measures(t, g.Label+" joined volume against body + piece", got, cpMM3(va+vb),
		decadtest.Within(cpMM3(ba+bb+1e-9*(va+vb))))
	// The piece starts exactly where the body ends, and the two together span
	// the joined body's stations.
	cpMeasuresAxialSpan(t, g.Label+" body", a, 0, float64(body)*m.P)
	cpMeasuresAxialSpan(t, g.Label+" piece", b, float64(body)*m.P, float64(body+piece)*m.P)
	cpMeasuresAxialSpan(t, g.Label+" joined body", bodies[0], 0, float64(body+piece)*m.P)
}

// --- Sleeve --------------------------------------------------------------

func cpSleeveSolidCases() []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range cpSleeveCases {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

var cpSleeveExtrudeCases = cpSleeveSolidCases()

// cpTube builds the Sleeve sketch's ring on the selected plane and extrudes it
// symmetrically by cageRise each side.
func cpTube(t *testing.T, doc *decad.Document, m cpModel) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	ci := s.CreatePoint(0, 0)
	inner := s.CreateCircle(ci, m.Ri)
	s.Fix(ci)
	co := s.CreatePoint(0, 0)
	outer := s.CreateCircle(co, m.Ro)
	s.Fix(co)
	s.AddConstraint(sketch.NewDiameter(inner, 2*m.Ri), sketch.NewDiameter(outer, 2*m.Ro))
	sketchtest.Solve(t, s)
	var ring *sketch.Profile
	for _, pr := range s.Profiles() {
		if pr.Valid && len(pr.Holes) == 1 {
			if ring != nil {
				t.Fatalf("Sleeve sketch has two ring profiles")
			}
			ring = pr
		}
	}
	if ring == nil {
		t.Fatalf("Sleeve sketch has no ring profile")
	}
	tube, err := doc.Extrude(s, ring, decad.Symmetric{D: units.Millimeters(m.Rise)})
	if err != nil {
		t.Fatalf("extrude the ring: %v", err)
	}
	return tube
}

// stepSleeveExtrude is the tube of §4: the ring extruded as a new body with a
// symmetric extent of cageRise each side.
func stepSleeveExtrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{cpTube(t, doc, cpModelOf(p))}
}

func assertSleeveExtrude(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	if len(bodies) != 1 {
		t.Fatalf("sleeve extrude left %d bodies, want 1", len(bodies))
	}
	want := math.Pi * (m.Ro*m.Ro - m.Ri*m.Ri) * 2 * m.Rise
	got, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("tube volume: %v", err)
	}
	// Closed form; its only error is float64 rounding.
	decadtest.Measures(t, "tube volume", got, cpMM3(want), decadtest.WithinRel(units.Scalar(1e-12)))
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("tube bounds: %v", err)
	}
	decadtest.MeasuresBox(t, "tube bounds", box, r3.NewVec(-m.Ro, -m.Ro, -m.Rise), r3.NewVec(m.Ro, m.Ro, m.Rise),
		decadtest.Within(units.Millimeters(1e-9)))
}

// --- Bores ---------------------------------------------------------------

func cpBoreSolidCases() []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range cpPerBore(append(append([]proofkit.Case{}, cpSleeveCases...),
		proofkit.Case{Name: "mount 0", Params: cpWith(map[string]float64{"mountAngleA": 0, "mountAngleB": 0})},
		proofkit.Case{Name: "mount -15", Params: cpWith(map[string]float64{"mountAngleA": -15, "mountAngleB": -15})},
		proofkit.Case{Name: "clearance 0.05", Params: cpWith(map[string]float64{"clearance": 0.05})},
	)) {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

var cpBoreCutCases = cpBoreSolidCases()

// cpBoreSections are the stand-in's sections: the bore's rectangle at
// boreSectionCount stations spread evenly over the cut's span, each turned to
// its station's angle.
func cpBoreSections(m cpModel, g cpGear, sign int) []cpQuad {
	a, b := m.boreSpan(sign)
	n := m.boreSectionCount()
	out := make([]cpQuad, 0, n)
	for i := range n {
		s := a + (b-a)*float64(i)/float64(n-1)
		out = append(out, m.quad(g, s, -m.Hw, m.Hw, m.Ht))
	}
	return out
}

// cpMechanismFrame places a section of gear g in the mechanism's own frame:
// its plane square to the axis at the station, x along û, y along v̂.
func cpMechanismFrame(g cpGear) func(cpQuad) r3.Frame {
	return func(q cpQuad) r3.Frame {
		f, err := r3.NewFrame(g.Origin.Add(g.Dir.Scale(q.S)), g.U, g.V)
		if err != nil {
			panic(err) // û and v̂ are orthonormal by construction
		}
		return f
	}
}

// stepBoreSweepCut is a bore's twisted sweep cut of §4. Its substitute is the
// channel the sweep cuts, not the cut (see the file comment): the ruled loft
// through the derived count of the bore's own sections over the cut's span.
func stepBoreSweepCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	secs := cpBoreSections(m, g, int(p["bore"]))
	return []*decad.Body{cpRuledSolid(t, doc, secs, cpMechanismFrame(g))}
}

func assertBoreSweepCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	sign := int(p["bore"])
	if len(bodies) != 1 {
		t.Fatalf("bore channel left %d bodies, want 1", len(bodies))
	}
	secs := cpBoreSections(m, g, sign)
	if p["clearance"] == cpDefaults["clearance"] && p["twistLead"] == cpDefaults["twistLead"] &&
		p["cageRadius"] == cpDefaults["cageRadius"] && p["collarHalf"] == cpDefaults["collarHalf"] &&
		p["ribbonWidth"] == cpDefaults["ribbonWidth"] && p["ribbonThickness"] == cpDefaults["ribbonThickness"] {
		sketchtest.Measures(t, "stand-in section count at the defaults", float64(len(secs)), 18, sketchtest.Within(0))
	}
	// The sweep's turn: +(sOut - sIn)/Lambda, 82.31° at the defaults.
	turn := (m.SOut - m.SIn) / m.Lam
	sketchtest.Measures(t, "twist from first to last section", m.theta(g, secs[len(secs)-1].S)-m.theta(g, secs[0].S), turn, sketchtest.Within(1e-12))
	v, slack := cpRuledVolume(secs)
	got, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("channel volume: %v", err)
	}
	decadtest.Measures(t, "channel volume", got, cpMM3(v), decadtest.Within(cpMM3(slack+1e-9*v)))

	// Both ends stand in air: a +R bore's profile in the hollow and its far end
	// outside the tube, a -R bore's profile outside and its far end in the
	// hollow.
	radius := func(q cpQuad, i int) float64 {
		w := g.Origin.Add(g.Dir.Scale(q.S)).Add(g.U.Scale(q.Corners[i][0])).Add(g.V.Scale(q.Corners[i][1]))
		return math.Hypot(w.X, w.Y)
	}
	inHollow, outside := secs[0], secs[len(secs)-1]
	if sign < 0 {
		inHollow, outside = outside, inHollow
	}
	for i := range 4 {
		if r := radius(inHollow, i); r >= m.Ri {
			t.Fatalf("hollow-side end corner %d at radius %.4f, not inside Ri = %.4f", i, r, m.Ri)
		}
		if r := radius(outside, i); r <= m.Ro {
			t.Fatalf("outer end corner %d at radius %.4f, not outside Ro = %.4f", i, r, m.Ro)
		}
	}

	// The build's probes at the crossing, origin + sc*dir ± (W/2 + clearance/2)
	// *û(sc), lie inside the stand-in's section there, clearance/2 from the
	// channel's wall less what the facets cost: the count keeps that under 4%
	// of the clearance ("What the proof's stand-in costs").
	sc := float64(sign) * m.CageRadius
	k := 0
	for k+1 < len(secs)-1 && secs[k+1].S < sc {
		k++
	}
	a, b := secs[k], secs[k+1]
	f := (sc - a.S) / (b.S - a.S)
	poly := make([][2]float64, 4)
	for i := range 4 {
		poly[i] = [2]float64{a.Corners[i][0] + f*(b.Corners[i][0]-a.Corners[i][0]), a.Corners[i][1] + f*(b.Corners[i][1]-a.Corners[i][1])}
	}
	th := m.theta(g, sc)
	for _, side := range []float64{1, -1} {
		r := side * (m.W/2 + m.Clearance/2)
		margin := cpInsideConvex(poly, r*math.Cos(th), r*math.Sin(th))
		if margin < m.Clearance/2-0.04*m.Clearance {
			t.Fatalf("probe %+.0f at the crossing stands %.5f mm inside the stand-in channel, want at least %.5f",
				side, margin, m.Clearance/2-0.04*m.Clearance)
		}
	}
}

// --- Windows -------------------------------------------------------------

var cpWindowCutCases = []proofkit3d.Case{
	{Name: "defaults, window +k", Params: cpWith(map[string]float64{"window": 0})},
	{Name: "defaults, window -k", Params: cpWith(map[string]float64{"window": 1})},
}

// cpWindowPrism is a Window sketch on the Window Plane (x along across, y
// along n̂, normal d) extruded one way along d by Ro + 1 mm.
func cpWindowPrism(t *testing.T, doc *decad.Document, m cpModel, win cpWindow) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(r3.NewVec(0, 0, 0), win.across(), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("window plane: %v", err)
	}
	if f.N().Sub(win.D).Len() > 1e-12 {
		t.Fatalf("window plane normal %v is not d %v", f.N(), win.D)
	}
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("window plane: %v", err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("window sketch: %v", err)
	}
	corners := win.corners(m.Ro)
	var pts []*sketch.Point
	for _, c := range corners {
		pts = append(pts, s.CreatePoint(c[0], c[1]))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	prof := decadtest.SolveRegion(t, s)
	prism, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(m.Ro + 1), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude window %s: %v", win.Name, err)
	}
	return prism
}

// cpWindowsOf is the window a case cuts.
func cpWindowsOf(p map[string]float64) []cpWindow {
	i := int(p["window"])
	return cpDefaultWindows()[i : i+1]
}

// stepWindowCut is a window's extrude cut of §4: the hexagon extruded one way,
// along d, by Ro + 1 mm, cut from the cage alone. The tube here carries no
// bore and no other window: the window keeps collarWall from every bore, so
// the bores do not reach the material it removes, and a second cut of one
// tube is refused at this decad (see the file comment).
func stepWindowCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	m := cpModelOf(p)
	cage := cpTube(t, doc, m)
	for _, win := range cpWindowsOf(p) {
		prism := cpWindowPrism(t, doc, m, win)
		var err error
		cage, err = decad.Cut(t.Context(), cage, prism)
		if err != nil {
			t.Fatalf("cut window %s: %v", win.Name, err)
		}
	}
	return []*decad.Body{cage}
}

// cpWindowVolume integrates the wall the hexagon's prism takes away: at each
// t the hexagon spans [zLow(t), zHigh(t)] and the wall runs from a0(t) to
// a1(t) along d.
func cpWindowVolume(m cpModel, win cpWindow) float64 {
	const n = 200000
	f := func(tt float64) float64 {
		zl := max(win.Lo-win.Lean*tt, win.Bottom+win.Lean*tt)
		zh := min(win.Hi-win.Lean*tt, win.Top+win.Lean*tt)
		if zh <= zl {
			return 0
		}
		a0 := math.Sqrt(max(0, m.Ri*m.Ri-tt*tt))
		a1 := math.Sqrt(max(0, m.Ro*m.Ro-tt*tt))
		return (zh - zl) * (a1 - a0)
	}
	h := (win.Right - win.Left) / n
	sum := f(win.Left) + f(win.Right)
	for i := 1; i < n; i++ {
		w := 2.0
		if i%2 == 1 {
			w = 4
		}
		sum += w * f(win.Left+float64(i)*h)
	}
	return sum * h / 3
}

func assertWindowCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	m := cpModelOf(p)
	if len(bodies) != 1 {
		t.Fatalf("window cut left %d bodies, want 1", len(bodies))
	}
	tube := math.Pi * (m.Ro*m.Ro - m.Ri*m.Ri) * 2 * m.Rise
	removed := 0.0
	for _, win := range cpWindowsOf(p) {
		removed += cpWindowVolume(m, win)
		// The build's probe: the hexagon's corner average on the plane, at the
		// middle of the wall along d. It lies inside the hexagon and inside the
		// wall, so it is material before the cut and inside the prism after.
		corners := win.corners(m.Ro)
		var tc, zc float64
		for _, c := range corners {
			tc += c[0]
			zc += c[1]
		}
		tc /= float64(len(corners))
		zc /= float64(len(corners))
		if cpInsideConvex(corners, tc, zc) <= 0 {
			t.Fatalf("window %s probe (%.3f, %.3f) is not inside its hexagon", win.Name, tc, zc)
		}
		a0 := math.Sqrt(m.Ri*m.Ri - tc*tc)
		a1 := math.Sqrt(m.Ro*m.Ro - tc*tc)
		if a := (a0 + a1) / 2; a <= a0 || a >= a1 || a >= m.Ro+1 || math.Abs(zc) >= m.Rise {
			t.Fatalf("window %s probe depth %.3f is not inside the wall [%.3f, %.3f]", win.Name, a, a0, a1)
		}
	}
	got, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("cage volume: %v", err)
	}
	// Composite Simpson over 200000 intervals of a piecewise-smooth integrand:
	// its error at the hexagon's kinks is under 1e-4 mm³.
	decadtest.Measures(t, "cage volume after the window cuts", got, cpMM3(tube-removed), decadtest.Within(cpMM3(1e-4)))
}
