package screwgear_test

import (
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ---- case tables -----------------------------------------------------------

func csSolidPerGear(vs []csVariant) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range csPerGear(vs) {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

// csRounds are the pieces of spec §3's doubling schedule for a case, from the
// hand-written doublingRounds that TestDoublingScheduleCoversTheRibbon runs:
// the first piece is the cell, each later one a round's tool, [from, to) in
// teeth, and a final remainder piece when N mod c is not zero.
func csRounds(in csIn) (rounds int, pieces [][2]int) {
	return doublingRounds(in.P.ToothCount, in.CellTeeth)
}

// csMoveCases is one case per screw move the schedule makes: every round's
// tool is moved by Step(from) teeth, the size of the body it joins.
func csMoveCases(vs []csVariant, gears []int) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, v := range vs {
		for _, g := range gears {
			m := csParamsOf(v, nil)
			in := csReadQuiet(m)
			c, q, _ := csCellCount(in)
			_, pieces := csRounds(in)
			for _, pc := range pieces[1:] {
				if pc[0] == q*c && in.P.ToothCount%c != 0 && pc[1] == in.P.ToothCount {
					continue // the remainder is built in place, not moved
				}
				out = append(out, proofkit3d.Case{
					Name:   fmt.Sprintf("%s/%s/Step(%d)", v.name, csGearLabel(g), pc[0]),
					Params: csParamsOf(v, map[string]float64{csGearKey: float64(g), csMoveKey: float64(pc[0])}),
				})
			}
		}
	}
	return out
}

// csJoinCases is one case per join of the schedule: a round's join, or the
// remainder's when remainder is set.
func csJoinCases(vs []csVariant, remainder bool) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, v := range vs {
		for g := range 2 {
			m := csParamsOf(v, nil)
			in := csReadQuiet(m)
			c, q, r := csCellCount(in)
			_, pieces := csRounds(in)
			for _, pc := range pieces[1:] {
				isRemainder := r != 0 && pc[0] == q*c && pc[1] == in.P.ToothCount
				if isRemainder != remainder {
					continue
				}
				moved := 1.0
				if isRemainder {
					moved = 0
				}
				out = append(out, proofkit3d.Case{
					Name: fmt.Sprintf("%s/%s/%d+%d", v.name, csGearLabel(g), pc[0], pc[1]-pc[0]),
					Params: csParamsOf(v, map[string]float64{csGearKey: float64(g),
						csBodyKey: float64(pc[0]), csToolKey: float64(pc[1] - pc[0]), csPlaceKey: moved}),
				})
			}
		}
	}
	return out
}

// csReadQuiet reads a table's own inputs while the table is being built, where
// no test is running yet; the step reads them again under its own test.
func csReadQuiet(m map[string]float64) csIn {
	return csIn{
		P: Params{
			ToothCount:  int(m[csToothCount]),
			ToothPitch:  m[csToothPitch],
			TwistLead:   m[csTwistLead],
			ToothHeight: m[csToothHeight],
			Width:       m[csRibbonWidth],
		},
		Phase:     m[csAssemblyPhase],
		CellTeeth: int(m[csCellTeeth]),
	}
}

var (
	csCellLoftCases      = csSolidPerGear(csRibbonVariants)
	csRemainderLoftCases = csSolidPerGear([]csVariant{csVTeeth15, csVTeeth69})
	csCopyCases          = csSolidPerGear([]csVariant{csVDefaults, csVCellOne, csVLead400, csVPhasePlus})
	csMoveCasesTable     = append(
		csMoveCases([]csVariant{csVDefaults, csVCellOne}, []int{0, 1}),
		csMoveCases([]csVariant{csVTeeth15, csVMountNeg, csVPhaseMinus, csVLead20}, []int{1})...)
	csJoinCasesTable     = csJoinCases([]csVariant{csVDefaults, csVTeeth15, csVCellOne15}, false)
	csRemainderJoinCases = csJoinCases([]csVariant{csVTeeth15, csVTeeth69}, true)
)

// csMaxStitchedSections is the longest ribbon the proof stitches into one
// solid. decad's loft crossing audit has a fixed work ceiling: a stitched
// ribbon of 321 sections (32 teeth at the defaults) builds and verifies Sound,
// and the whole 68-tooth ribbon of 681 sections was refused with "the loft
// crossing audit's facet-pair count exceeds the fixed work ceiling" when this
// proof was drafted. A join whose result passes this is Unmodelled rather than
// dropped, and the rounds below it carry the same move and the same shared
// cross-section.
const csMaxStitchedSections = 330

// ---- the cell and the remainder --------------------------------------------

// csRibbonSections are the corners of every section of a stretch of ribbon, in
// station order, with their frames: the input the loft is fed.
func csRibbonSections(t testing.TB, g Gear, from float64, teeth int) ([]r3.Frame, [][]r3.Vec) {
	t.Helper()
	stations := csStations(g.P, from, teeth)
	frames := make([]r3.Frame, len(stations))
	sections := make([][]r3.Vec, len(stations))
	for k, st := range stations {
		frames[k] = csSectionFrame(t, g, st)
		sections[k] = csSectionCorners(g, st)
	}
	return frames, sections
}

// csRibbonPiece lofts teeth teeth of gear g's ribbon from station from.
func csRibbonPiece(t testing.TB, doc *decad.Document, g Gear, from float64, teeth int, what string) *decad.Body {
	t.Helper()
	frames, sections := csRibbonSections(t, g, from, teeth)
	return csRuledLoft(t, doc, frames, sections, what)
}

// csCellPart is the stretch a cell step lofts: the cell at the ribbon's start,
// or the remainder after the q whole cells.
func csCellPart(t testing.TB, m map[string]float64, remainder bool) (csIn, Gear, float64, int) {
	t.Helper()
	in := csRead(t, m)
	csCheckInputs(t, in)
	g := csGears(in)[int(csNeed(t, m, csGearKey))]
	c, q, r := csCellCount(in)
	if !remainder {
		return in, g, csStartStation(g), c
	}
	if r == 0 {
		t.Fatalf("toothCount %d leaves no remainder in %d-tooth cells", in.P.ToothCount, c)
	}
	return in, g, csStartStation(g) + float64(q*c)*in.P.ToothPitch, r
}

// stepCellLoft is the cell's loft: one body through every section, in station
// order, fed one path of four lines per section.
//
// SUBSTITUTE: Fusion's loft through more than two sections is smooth between
// them ([SCREW-F-CELL-LOFT]); decad lofts two sections at a time, ruled, and
// its walls between sections are flat triangle pairs. The stand-in passes
// through the same sections, so it pins what the count guarantees — the exact
// rotated rectangle at each station — and that the sections close into one
// solid. It does not pin the surface between them: the smooth one was measured
// in Fusion within 0.04 mm of the helicoid at the 10 mm ribbon, and the flat
// triangle pairs here depart from the ruled (bilinear) surface the spec's chord
// figures describe by up to a quarter of a wall's warp, 0.13 mm on a long face
// at the defaults' 1.91-degree step. The volume assertion therefore holds the
// body to the triangulated loft's own bracket, not to the helicoid.
func stepCellLoft(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	_, g, from, teeth := csCellPart(t, m, false)
	return []*decad.Body{csRibbonPiece(t, doc, g, from, teeth, "the cell")}
}

func checkCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	csCheckRibbonPart(t, bodies, m, false)
}

// stepRemainderLoft is the Cell Remainder's loft, r*n + 1 sections from
// s0 + q*c*P to s0 + N*P, built where the teeth belong. The substitute is the
// cell's.
func stepRemainderLoft(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	_, g, from, teeth := csCellPart(t, m, true)
	return []*decad.Body{csRibbonPiece(t, doc, g, from, teeth, "the remainder cell")}
}

func checkRemainderLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	csCheckRibbonPart(t, bodies, m, true)
}

func csCheckRibbonPart(t *testing.T, bodies []*decad.Body, m map[string]float64, remainder bool) {
	in, g, from, teeth := csCellPart(t, m, remainder)
	if len(bodies) != 1 {
		t.Fatalf("the loft leaves %d bodies; the build requires exactly one", len(bodies))
	}
	body := bodies[0]
	if !body.IsSolid() {
		t.Fatalf("the lofted cell is not a solid")
	}
	_, sections := csRibbonSections(t, g, from, teeth)
	if want := teeth*csStepsPerTooth(in.P) + 1; len(sections) != want {
		t.Fatalf("%d sections; spec §2 lofts %d", len(sections), want)
	}
	if want, ok := m[csExpectSections]; ok && !remainder && float64(len(sections)) != want {
		t.Fatalf("%d sections; the spec quotes %v for this case", len(sections), want)
	}
	want, slack := csRuledVolume(sections)
	csMeasuresVolume(t, "the lofted cell", body, want, slack)
	// The two end sections are the exact rotated rectangles: the joins of §3
	// meet the next piece on them.
	for _, k := range []int{0, len(sections) - 1} {
		for i, corner := range sections[k] {
			csHasVertex(t, fmt.Sprintf("section %d corner %d", k, i), body, corner)
		}
	}
	helicoid := float64(teeth) * in.P.ToothPitch * (in.P.Width - in.P.ToothHeight/2) * in.P.Thickness
	v, _ := body.Volume()
	t.Logf("%d sections; volume %.4f mm³ against %.4f for the exact helicoid cell (%.3f%%)",
		len(sections), v.Value.Base(), helicoid, 100*(v.Value.Base()/helicoid-1))
}

// ---- §3 copy, move and join -------------------------------------------------

// stepCopyRibbon is a round's copy: copyPasteBodies.add(body), taking the copy
// from the feature's bodies.
//
// A copy coincides with its source, and two coincident bodies are a pair the
// document verification cannot decide, so the copy is taken in a scratch
// document, where Duplicate is held to its source vertex for vertex and by
// volume; the harness document holds the geometry the copy carries, lofted
// afresh, for the gate. The cell stands for the m-cell body: the copy is the
// same operation at every size.
func stepCopyRibbon(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	_, g, from, teeth := csCellPart(t, m, false)
	return []*decad.Body{csRibbonPiece(t, doc, g, from, teeth, "the copy")}
}

func checkCopyRibbon(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	_, g, from, teeth := csCellPart(t, m, false)
	scratch := decad.New()
	src := csRibbonPiece(t, scratch, g, from, teeth, "the source body")
	cp, err := src.Duplicate(context.Background())
	if err != nil {
		t.Fatalf("copy: %v", err)
	}
	if got, want := len(cp.Vertices()), len(src.Vertices()); got != want {
		t.Fatalf("the copy has %d vertices, its source %d", got, want)
	}
	for i, v := range src.Vertices() {
		csHasVertex(t, fmt.Sprintf("the copy at source vertex %d", i), cp, v.Position().Value)
	}
	vs, _ := src.Volume()
	vc, _ := cp.Volume()
	decadtest.Agree(t, "the copy's volume against its source's", vc, vs, decadtest.Within(units.CubicMillimeters(1e-9)))
	vh, _ := bodies[0].Volume()
	decadtest.Agree(t, "the gated body's volume against the copy's", vh, vc, decadtest.Within(units.CubicMillimeters(1e-9)))
}

// stepMoveRibbon is a round's move: the copy moved by the screw step Step(k),
// a turn of k*P/Lambda about the gear's axis composed with k*P along it,
// through moveFeatures.createInput2 → defineAsFreeMove(matrix) → add.
//
// The proof moves the cell rather than an m-cell body, by the same Step(k);
// what it pins is the matrix. The moved cell has to land vertex for vertex on
// the cell lofted k teeth further along, which is the invariance the whole
// doubling rests on (TestRibbonIsInvariantUnderItsScrewStep) and which fails
// under a rotation of the wrong sense, about the wrong line, or with the
// translation overwritten ([SCREW-F-SCREW-STEP]). decad's Placed retires what
// it moves, so the harness holds the moved body alone.
func stepMoveRibbon(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	_, g, from, teeth := csCellPart(t, m, false)
	k := int(csNeed(t, m, csMoveKey))
	cell := csRibbonPiece(t, doc, g, from, teeth, "the copy before its move")
	moved, err := cell.Placed(context.Background(), csScrewStep(t, g, k))
	if err != nil {
		t.Fatalf("move by Step(%d): %v", k, err)
	}
	return []*decad.Body{moved}
}

func checkMoveRibbon(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in, g, from, teeth := csCellPart(t, m, false)
	k := int(csNeed(t, m, csMoveKey))
	moved := bodies[0]
	scratch := decad.New()
	there := csRibbonPiece(t, scratch, g, from+float64(k)*in.P.ToothPitch, teeth, "the cell built where the move lands")
	if got, want := len(moved.Vertices()), len(there.Vertices()); got != want {
		t.Fatalf("the moved cell has %d vertices, the cell built in place %d", got, want)
	}
	for i, v := range there.Vertices() {
		csHasVertex(t, fmt.Sprintf("the moved cell at vertex %d of the cell built in place", i), moved, v.Position().Value)
	}
	vm, _ := moved.Volume()
	vt, _ := there.Volume()
	_, slack := csRuledVolume(func() [][]r3.Vec { _, s := csRibbonSections(t, g, from, teeth); return s }())
	// The two bodies' flat walls may split their quads along different
	// diagonals, which the loft's own bracket bounds.
	decadtest.Agree(t, "the moved cell's volume against the cell built in place", vm, vt,
		decadtest.Within(units.CubicMillimeters(2*slack)))
}

// stepJoinRibbon is a round's join: a combineFeatures join of the body and its
// moved copy, which must leave exactly one body.
//
// SUBSTITUTE: the two pieces meet face to face on one shared cross-section, and
// decad's boolean refuses exactly that ("two operand facets overlap in one
// plane"). The proof builds both pieces in a scratch document — the body as
// lofted, the tool lofted at the start and moved by Step(body) as the build
// moves it — and holds that the tool's first section lands on the body's last,
// corner for corner; the harness document then holds the joined ribbon as one
// loft through all of their sections, which the gate reads as one solid lump,
// and whose volume has to be the two pieces' together: no gap, no overlap.
func stepJoinRibbon(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	return csJoinBuild(t, doc, m)
}

func checkJoinRibbon(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	csJoinCheck(t, bodies, m)
}

// stepRemainderJoin joins the remainder cell, built in place, onto the body
// after the last round. The substitute is the round join's.
func stepRemainderJoin(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	return csJoinBuild(t, doc, m)
}

func checkRemainderJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	csJoinCheck(t, bodies, m)
}

func csJoinPieces(t *testing.T, m map[string]float64) (csIn, Gear, int, int, bool) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	g := csGears(in)[int(csNeed(t, m, csGearKey))]
	return in, g, int(csNeed(t, m, csBodyKey)), int(csNeed(t, m, csToolKey)), csNeed(t, m, csPlaceKey) == 1
}

func csJoinBuild(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in, g, body, tool, _ := csJoinPieces(t, m)
	if sections := (body+tool)*csStepsPerTooth(in.P) + 1; sections > csMaxStitchedSections {
		proofkit3d.Unmodelled(t, "the joined ribbon of %d teeth is %d sections, past the %d decad's loft "+
			"crossing audit stitches into one solid; the smaller rounds prove the same move and the same "+
			"shared cross-section", body+tool, sections, csMaxStitchedSections)
	}
	return []*decad.Body{csRibbonPiece(t, doc, g, csStartStation(g), body+tool, "the joined ribbon")}
}

func csJoinCheck(t *testing.T, bodies []*decad.Body, m map[string]float64) {
	in, g, body, tool, moved := csJoinPieces(t, m)
	p := in.P
	s0 := csStartStation(g)
	joined := bodies[0]

	scratch := decad.New()
	bodyPiece := csRibbonPiece(t, scratch, g, s0, body, "the body before the join")
	var toolPiece *decad.Body
	if moved {
		// Lofted in a document of its own before it moves: at its lofted
		// station it would coincide with the body's start.
		toolDoc := decad.New()
		lofted := csRibbonPiece(t, toolDoc, g, s0, tool, "the tool before its move")
		var err error
		toolPiece, err = lofted.Placed(context.Background(), csScrewStep(t, g, body))
		if err != nil {
			t.Fatalf("move by Step(%d): %v", body, err)
		}
	} else {
		toolPiece = csRibbonPiece(t, scratch, g, s0+float64(body)*p.ToothPitch, tool, "the remainder built in place")
	}

	// The shared cross-section: the body's last section is the tool's first.
	seam := csSectionCorners(g, s0+float64(body)*p.ToothPitch)
	for i, corner := range seam {
		csHasVertex(t, fmt.Sprintf("the body's last section, corner %d", i), bodyPiece, corner)
		csHasVertex(t, fmt.Sprintf("the tool's first section, corner %d", i), toolPiece, corner)
	}
	// The joined ribbon's ends are the body's start and the tool's end.
	for i, corner := range csSectionCorners(g, s0) {
		csHasVertex(t, fmt.Sprintf("the joined ribbon's first section, corner %d", i), joined, corner)
	}
	for i, corner := range csSectionCorners(g, s0+float64(body+tool)*p.ToothPitch) {
		csHasVertex(t, fmt.Sprintf("the joined ribbon's last section, corner %d", i), joined, corner)
	}

	_, all := csRibbonSections(t, g, s0, body+tool)
	want, slack := csRuledVolume(all)
	csMeasuresVolume(t, "the joined ribbon", joined, want, slack)
	vb, _ := bodyPiece.Volume()
	vt, _ := toolPiece.Volume()
	_, toolSections := csRibbonSections(t, g, s0, tool)
	_, toolSlack := csRuledVolume(toolSections)
	sum := decad.Measurement{
		Value:     units.CubicMillimeters(vb.Value.Base() + vt.Value.Base()),
		Exactness: decad.Approximate,
		Bound:     units.CubicMillimeters(vb.Bound.Base() + vt.Bound.Base()),
	}
	vj, _ := joined.Volume()
	// The moved tool's flat walls may split along other diagonals than the
	// joined loft's over the same stretch; its own bracket bounds that.
	decadtest.Agree(t, "the joined ribbon's volume against its two pieces'", vj, sum,
		decadtest.Within(units.CubicMillimeters(2*toolSlack)))
}
