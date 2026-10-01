package screwgear_test

// The solid steps of spec/screwgear/steps.md, each built in decad through
// proofkit3d.RunSolid and checked by its assertion after the gate.
//
// Four of the build's operations have no decad counterpart, and each is
// stood in for where it is built below (spec "What the proof checks"):
//
//   - the cell loft, smooth in Fusion, is the ruled loft through the same
//     sections (cgLoftWalls, compiled_model_test.go);
//   - a join of two bodies meeting on a shared cross-section is the stitch of
//     their wall sheets, since decad's Union refuses that contact;
//   - a bore's twisted sweep is a ruled loft through BoreSections rotated
//     rectangles, the first of them drawn by the rectangle scheme;
//   - a cut with a participant list is a decad Cut of the cage alone.
//
// Where a step's state holds two bodies that share face planes — a copy over
// its source, a moved copy against the body it will join — decad's verifier
// reads the pair Suspect (unsupported_pair_contact), so the proof sets the
// source aside by a translation that leaves the two bounding boxes apart and
// says so beside the step.
//
// Two steps of the list have nothing here to build. The component tree
// (Screw Gearing, Design, Gear A, Gear B, Cage) and the final relocation of
// the three bodies into their components with moveToComponent are Fusion's
// occurrence plumbing; decad has one document and no components, so neither
// is reachable, and the three bodies the relocation moves are the ones the
// steps below prove: the ribbon of each gear and the cut tube.

import (
	"errors"
	"fmt"
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

// ---------------------------------------------------------------------------
// Case tables.

func cgSolidCases(cases []proofkit.Case) []proofkit3d.Case {
	out := make([]proofkit3d.Case, len(cases))
	for i, c := range cases {
		out[i] = proofkit3d.Case{Name: c.Name, Params: c.Params}
	}
	return out
}

var cellLoftCases = cgSolidCases(cgGearCases(cgRibbonInputs(), nil))

// copyCases copy the smallest and the largest body the default schedule
// copies: the single cell set aside before the first doubling, and the eight
// cells the last doubling copies.
var copyCases = cgSolidCases(append(
	cgGearCases([]cgSize{{"defaults, one cell", cgDefaults()}}, map[string]float64{"cells": 1}),
	cgGearCases([]cgSize{{"defaults, eight cells", cgDefaults()}}, map[string]float64{"cells": 8})...))

// moveCases move a copy by every kind of screw step the schedule takes: the
// first doubling, the last doubling, and the aside placed after it, at the
// defaults; and the one-tooth cell's aside of four cells placed after 64.
var moveCases = func() []proofkit3d.Case {
	one := cgDefaults()
	one.CellTeeth = 1
	var out []proofkit.Case
	for _, c := range []struct {
		name      string
		in        cgIn
		cells, by float64
	}{
		{"first doubling", cgDefaults(), 1, 1},
		{"last doubling", cgDefaults(), 8, 8},
		{"aside placed after sixteen", cgDefaults(), 1, 16},
		{"one-tooth cell, aside of four placed after 64", one, 4, 64},
	} {
		out = append(out, cgGearCases([]cgSize{{c.name, c.in}}, map[string]float64{"cells": c.cells, "by": c.by})...)
	}
	return cgSolidCases(out)
}()

// joinCases run the whole schedule: the defaults' seventeen cells (one aside
// and four doublings), and cell counts of 2, 3, 7 and 12, which reach a lone
// doubling, an aside before the first doubling, two asides placed largest
// first, and an aside taken after two doublings; and the one-tooth cell at 13
// teeth, two asides of one and four cells. decad's crossing audit has a fixed
// work ceiling that a closed ribbon of more than about 48 teeth at ten
// sections a tooth exceeds (measured at the pinned revision: 48 closes, 56
// is refused), so the defaults' 68 teeth are recorded as a case the proof
// cannot represent; the schedule itself is held at every count from 4 to 512
// by the mechanism proof's TestDoublingScheduleCoversTheRibbon.
var joinCases = func() []proofkit3d.Case {
	one := cgWithTeeth(13)
	one.CellTeeth = 1
	return cgSolidCases(cgGearCases([]cgSize{
		{"defaults", cgDefaults()}, {"8 teeth", cgWithTeeth(8)}, {"12 teeth", cgWithTeeth(12)},
		{"28 teeth", cgWithTeeth(28)}, {"48 teeth", cgWithTeeth(48)}, {"one-tooth cell, 13 teeth", one},
	}, nil))
}()

var remainderLoftCases = cgSolidCases(remainderSketchCases)

// joinRemainderCases join every remainder a four-tooth cell leaves, one to
// three teeth, behind eleven whole cells and behind a single one. The
// spec's own example, 69 teeth, is past decad's audit ceiling (joinCases).
var joinRemainderCases = cgSolidCases(cgGearCases([]cgSize{
	{"45 teeth", cgWithTeeth(45)}, {"46 teeth", cgWithTeeth(46)}, {"47 teeth", cgWithTeeth(47)},
	{"5 teeth", cgWithTeeth(5)}, {"7 teeth", cgWithTeeth(7)}, {"69 teeth", cgWithTeeth(69)},
}, nil))

var sleeveExtrudeCases = cgSolidCases(sleeveSketchCases)

var boreCutCases = cgSolidCases(sleeveSketchCases)

var windowCutCases = cgSolidCases(windowSketchCases)

// ---------------------------------------------------------------------------
// Shared readings.

func cgMM3(v float64) units.Value { return units.CubicMillimeters(v) }

// cgAddLen is a reading's length bound plus a rounding slack in millimetres.
func cgAddLen(b units.Value, mm float64) units.Value {
	v, err := b.Add(units.Millimeters(mm))
	if err != nil {
		panic(err)
	}
	return v
}

// cgVolume compares a body's volume with the ruled-loft oracle, naming the
// body.
func cgVolume(t *testing.T, what string, b *decad.Body, want, slack float64) {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	decadtest.Measures(t, what+" volume", m, cgMM3(want), decadtest.Within(cgMM3(slack+1e-9*want)))
}

func cgVol(t *testing.T, what string, b *decad.Body) decad.Measurement {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	return m
}

// cgClosedCells is the closed body of cells cells of gear g's ribbon from
// cell first: the cell stand-in, at any length. decad's refusal to close it
// is recorded as a case the proof cannot represent.
func cgClosedCells(t *testing.T, doc *decad.Document, g cgGear, first, cells int) *decad.Body {
	t.Helper()
	per := g.in.Cell() * g.in.StepsPerTooth()
	b, err := cgClosedRange(t, doc, g, first*per, (first+cells)*per)
	cgUnlessRefused(t, err, "closing %d cells of %s", cells, g.label)
	return b
}

// cgUnlessRefused fails the test on err, except that a refusal of decad's —
// ErrUnsupported, a contact or an audit past its work ceiling — skips the
// case as one the proof cannot represent.
func cgUnlessRefused(t *testing.T, err error, format string, args ...any) {
	t.Helper()
	if err == nil {
		return
	}
	what := fmt.Sprintf(format, args...)
	if cgRefused(err) {
		proofkit3d.Unmodelled(t, "decad refuses %s: %v", what, err)
	}
	t.Fatalf("%s: %v", what, err)
}

// cgAside is the translation that sets a source body aside: far enough that
// the two bounding boxes are apart, along no face's own plane.
func cgAside(t *testing.T, in cgIn) r3.Transform {
	t.Helper()
	l := in.L() + 100
	tr, err := r3.Translation(r3.NewVec(3*l, 5*l, 7*l))
	if err != nil {
		t.Fatalf("aside translation: %v", err)
	}
	return tr
}

// ---------------------------------------------------------------------------
// The tooth cell (§2).

// stepCellLoft lofts a gear's cell, c teeth from s0, through c*n + 1
// sections. The stand-in is the ruled loft through the same sections; Fusion's
// loft is smooth between them and came within 0.23% of the ruled loft's volume
// when measured ([SCREW-F-CELL-LOFT]).
func stepCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	return []*decad.Body{cgClosedCells(t, doc, g, 0, 1)}
}

func assertCellLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	if len(bodies) != 1 {
		t.Fatalf("the cell loft left %d bodies, want 1", len(bodies))
	}
	per := in.Cell() * in.StepsPerTooth()
	want, slack := cgRuledVolume(cgRibbonCorners(g, 0, per))
	cgVolume(t, g.label+" cell", bodies[0], want, slack)
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("cell bounds: %v", err)
	}
	for _, q := range append(cgRibbonCorners(g, 0, 0), cgRibbonCorners(g, per, per)...) {
		for _, c := range q {
			decadtest.Encloses(t, g.label+" cell bounds", box, c)
		}
	}
}

// ---------------------------------------------------------------------------
// The doubling (§3): copy, move, join.

// stepCopyBody copies a body of the given number of cells, as the build's
// copyPasteBodies does. What is substituted: a copy coincides with its source
// face for face, and decad's verifier reads such a pair Suspect, so after the
// copy is taken the source is set aside by cgAside. Nothing the next step
// reads is lost: the copy carries the source's geometry, which the assertion
// holds.
func stepCopyBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	cells := int(p["cells"])
	src := cgClosedCells(t, doc, g, 0, cells)
	cp, err := src.Duplicate(t.Context())
	if err != nil {
		t.Fatalf("copying %d cells: %v", cells, err)
	}
	set, err := src.Placed(t.Context(), cgAside(t, in))
	if err != nil {
		t.Fatalf("setting the source aside: %v", err)
	}
	return []*decad.Body{cp, set}
}

func assertCopyBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	if len(bodies) != 2 {
		t.Fatalf("the copy step left %d bodies, want the copy and its source", len(bodies))
	}
	cp, src := bodies[0], bodies[1]
	decadtest.Agree(t, "the copy's volume against its source's", cgVol(t, "copy", cp), cgVol(t, "source", src))
	cc, err := cp.Centroid()
	if err != nil {
		t.Fatalf("copy centroid: %v", err)
	}
	sc, err := src.Centroid()
	if err != nil {
		t.Fatalf("source centroid: %v", err)
	}
	shift := cgAside(t, in).Translation()
	// The source's own bound plus rounding through the translation.
	decadtest.MeasuresVec(t, "the copy's centroid against its source's, set back", cc, sc.Value.Sub(shift),
		decadtest.Within(cgAddLen(sc.Bound, 1e-6)))
}

// stepScrewMove moves a copy of a body by the screw step Step(by*c), as the
// build's free move does. What is substituted: the moved copy meets the body
// it will be joined to on one shared cross-section, a contact decad's
// verifier cannot classify, so the source is set aside by cgAside. The
// assertion holds the moved copy against the same cells built where it lands.
func stepScrewMove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	cells, by := int(p["cells"]), int(p["by"])
	src := cgClosedCells(t, doc, g, 0, cells)
	cp, err := src.Duplicate(t.Context())
	if err != nil {
		t.Fatalf("copying %d cells: %v", cells, err)
	}
	moved, err := cp.Placed(t.Context(), g.screwStep(t, by*in.Cell()))
	if err != nil {
		t.Fatalf("moving the copy by Step(%d): %v", by*in.Cell(), err)
	}
	set, err := src.Placed(t.Context(), cgAside(t, in))
	if err != nil {
		t.Fatalf("setting the source aside: %v", err)
	}
	return []*decad.Body{moved, set}
}

func assertScrewMove(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	cells, by := int(p["cells"]), int(p["by"])
	if len(bodies) != 2 {
		t.Fatalf("the move step left %d bodies, want the moved copy and its source", len(bodies))
	}
	moved := bodies[0]
	// The same cells built in place, cells by to by + cells, in a document of
	// their own: the body is invariant under its screw step, so the moved copy
	// must be them. A placed body's bounding box is not tight (decad bounds
	// the placed box), so the comparison is the volume and the centroid, which
	// the helix's off-axis section makes sensitive to the turn as well as the
	// advance.
	ref := cgClosedCells(t, decad.New(), g, by, cells)
	decadtest.Agree(t, "the moved copy's volume against the cells built in place", cgVol(t, "moved", moved),
		cgVol(t, "in place", ref))
	mc, err := moved.Centroid()
	if err != nil {
		t.Fatalf("moved centroid: %v", err)
	}
	rc, err := ref.Centroid()
	if err != nil {
		t.Fatalf("reference centroid: %v", err)
	}
	// The reference's own bound plus rounding through the transform.
	decadtest.MeasuresVec(t, "the moved copy's centroid against the cells built in place", mc, rc.Value,
		decadtest.Within(cgAddLen(rc.Bound, 1e-6)))
}

// stepJoinBodies runs the whole doubling schedule and returns the ribbon of q
// cells it ends with. What is substituted: each join is the stitch of the two
// bodies' wall sheets, the stand-in for a combine join that cgLoftWalls
// describes, and each joined copy is lofted where its move puts it
// (cgRibbonWalls); the finished tube is closed by its two end sections.
func stepJoinBodies(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	q := in.N / in.Cell()
	walls, ops := cgRibbonWalls(t, doc, g, q)
	joins := 0
	for _, op := range ops {
		if op.kind != "aside" {
			joins++
		}
	}
	// The spec's round count, floor(log2 q) + popcount(q) - 1: 5 at the
	// defaults and 7 at a one-tooth cell of 68 teeth.
	if joins != cgRounds(q) || (in.N == 68 && in.Cell() == 4 && joins != 5) || (in.N == 68 && in.Cell() == 1 && joins != 7) {
		t.Fatalf("the schedule for %d cells joins %d times; the spec's count is %d", q, joins, cgRounds(q))
	}
	ribbon, err := cgCloseRibbon(t, doc, g, walls, 0, q*in.Cell()*in.StepsPerTooth())
	cgUnlessRefused(t, err, "closing the ribbon of %d cells", q)
	return []*decad.Body{ribbon}
}

func assertJoinBodies(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	c := in.Cell()
	q := in.N / c
	if len(bodies) != 1 {
		t.Fatalf("the joins left %d bodies, want 1", len(bodies))
	}
	want, slack := cgRuledVolume(cgRibbonCorners(g, 0, q*c*in.StepsPerTooth()))
	cgVolume(t, g.label+" ribbon of whole cells", bodies[0], want, slack)
}

// ---------------------------------------------------------------------------
// The remainder (§3).

// stepCellRemainderLoft lofts the remainder cell, r teeth from s0 + q*c*P, by
// the cell's recipe: ribbon sections q*c*n to N*n.
func stepCellRemainderLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	n, c := in.StepsPerTooth(), in.Cell()
	b, err := cgClosedRange(t, doc, g, (in.N/c)*c*n, in.N*n)
	cgUnlessRefused(t, err, "closing the remainder cell")
	return []*decad.Body{b}
}

func assertCellRemainderLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	n, c := in.StepsPerTooth(), in.Cell()
	if len(bodies) != 1 {
		t.Fatalf("the remainder loft left %d bodies, want 1", len(bodies))
	}
	want, slack := cgRuledVolume(cgRibbonCorners(g, (in.N/c)*c*n, in.N*n))
	cgVolume(t, g.label+" remainder cell", bodies[0], want, slack)
}

// stepJoinRemainder joins the remainder cell to the ribbon after the last
// round, by the same stand-in as stepJoinBodies.
func stepJoinRemainder(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	n, c := in.StepsPerTooth(), in.Cell()
	q := in.N / c
	walls, _ := cgRibbonWalls(t, doc, g, q)
	rem := cgRibbonPiece(t, doc, g, q*c*n, in.N*n)
	joined, err := decad.Stitch(t.Context(), walls, rem)
	if err != nil {
		t.Fatalf("joining the remainder: %v", err)
	}
	ribbon, err := cgCloseRibbon(t, doc, g, joined, 0, in.N*n)
	cgUnlessRefused(t, err, "closing the ribbon of %d teeth", in.N)
	return []*decad.Body{ribbon}
}

func assertJoinRemainder(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	if len(bodies) != 1 {
		t.Fatalf("the remainder join left %d bodies, want 1", len(bodies))
	}
	want, slack := cgRuledVolume(cgRibbonCorners(g, 0, in.N*in.StepsPerTooth()))
	cgVolume(t, g.label+" ribbon", bodies[0], want, slack)
}

// ---------------------------------------------------------------------------
// The sleeve (§4).

// cgTube is the tube: the Sleeve sketch's ring extruded cageRise each side of
// the selected plane.
func cgTube(t *testing.T, doc *decad.Document, in cgIn) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("sleeve sketch: %v", err)
	}
	for _, r := range []float64{in.Ri(), in.Ro()} {
		centre := s.CreatePoint(0, 0)
		circle := s.CreateCircle(centre, r)
		s.Fix(centre)
		s.AddConstraint(sketch.NewDiameter(circle, 2*r))
	}
	sketchtest.Solve(t, s)
	var ring *sketch.Profile
	for _, prof := range s.Profiles() {
		if len(prof.Holes) == 1 {
			if ring != nil {
				t.Fatalf("two profiles with two loops in the Sleeve sketch")
			}
			ring = prof
		}
	}
	if ring == nil {
		t.Fatalf("no profile with two loops in the Sleeve sketch")
	}
	tube, err := doc.Extrude(s, ring, decad.Symmetric{D: units.Millimeters(in.CageRise)})
	if err != nil {
		t.Fatalf("extruding the tube: %v", err)
	}
	return tube
}

// stepSleeveExtrude extrudes the Sleeve sketch's ring as a new body, cageRise
// each side of the selected plane.
func stepSleeveExtrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{cgTube(t, doc, cgRead(t, p))}
}

func assertSleeveExtrude(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	if len(bodies) != 1 {
		t.Fatalf("the tube extrude left %d bodies, want 1", len(bodies))
	}
	ri, ro, h := in.Ri(), in.Ro(), in.CageRise
	// pi*(Ro^2 - Ri^2)*2*cageRise exactly; the slack is rounding.
	want := math.Pi * (ro*ro - ri*ri) * 2 * h
	cgVolume(t, "the tube", bodies[0], want, 1e-9*want)
	decadtest.MeasuresBounds(t, bodies[0], r3.NewVec(-ro, -ro, -h), r3.NewVec(ro, ro, h),
		decadtest.Within(units.Millimeters(1e-9)))
}

// cgChannel is one bore's stand-in: a ruled loft through BoreSections
// sections over the cut's span, section j at lo + (hi - lo)*j/(count - 1),
// each the bore's rectangle turned to its station's angle. The first is the
// sweep's own profile, drawn by the rectangle scheme on its plane; the rest
// are fixed rectangles. TestSleeveBoreSubstituteKeepsItsClearance bounds what
// the ruled facets cost against the swept channel: 0.007 mm of the 0.45 mm
// clearance at the defaults.
func cgChannel(t *testing.T, doc *decad.Document, in cgIn, bore int) (*decad.Body, float64) {
	t.Helper()
	g := cgGears(in)[bore/2]
	lo, hi := in.SIn(), in.SOut()
	if bore%2 == 0 {
		lo, hi = -in.SOut(), -in.SIn()
	}
	w := sketch.NewWorld()
	count := in.BoreSections()
	secs := make([]cgSection, 0, count)
	for j := range count {
		st := lo + (hi-lo)*float64(j)/float64(count-1)
		f := g.sectionFrame(t, st)
		if j > 0 {
			secs = append(secs, cgNewSection(t, w, f, g.boreCorners(st)))
			continue
		}
		pl, err := w.CreatePlaneFromFrame(f)
		if err != nil {
			t.Fatalf("bore plane: %v", err)
		}
		s, err := w.CreateSketch(pl)
		if err != nil {
			t.Fatalf("bore sketch: %v", err)
		}
		lines, _, _ := cgBoreScheme(s, g, st, s.Fix)
		sketchtest.Solve(t, s)
		sec := cgSection{s: s, lines: lines[:], prof: cgOneProfile(t, s)}
		for i, q := range g.boreCorners(st) {
			sec.world[i] = f.ToWorldUV(q[0], q[1])
		}
		secs = append(secs, sec)
	}
	walls := cgLoftWalls(t, doc, secs)
	vol, _ := cgRuledVolume(cgWorldCorners(secs))
	ch, err := cgClose(t, doc, walls, secs[0], secs[len(secs)-1], cgBoreNames[bore]+" channel")
	if err != nil {
		t.Fatalf("closing the %s stand-in: %v", cgBoreNames[bore], err)
	}
	return ch, vol
}

// cgChannels is the union of the four bores' stand-ins and their summed
// volume. The four are disjoint, which is the wall between the bores.
func cgChannels(t *testing.T, doc *decad.Document, in cgIn) (*decad.Body, float64, error) {
	t.Helper()
	var tool *decad.Body
	total := 0.0
	for b := range 4 {
		ch, v := cgChannel(t, doc, in, b)
		total += v
		if tool == nil {
			tool = ch
			continue
		}
		var err error
		if tool, err = decad.Union(t.Context(), tool, ch); err != nil {
			return nil, 0, err
		}
	}
	return tool, total, nil
}

// cgRefused reports decad's refusal of a contact it cannot classify, which
// the proof records as a case it cannot represent rather than a failure.
func cgRefused(err error) bool {
	var be *decad.BooleanError
	return errors.Is(err, decad.ErrUnsupported) || (errors.As(err, &be) && be.Code == decad.BooleanUnsupportedContact)
}

// stepBoreSweepCut cuts the four bores from the tube. What is substituted:
// Fusion cuts each bore with its own sweep, cage alone as participant, in the
// order gear A -R, gear A +R, gear B -R, gear B +R. A decad cut leaves a
// faceted body whose held chord bound is coarser than the tolerance the next
// cut asks of it (measured at the pinned revision: 0.0022 mm held against
// 0.0013 mm asked), so a second cut on the result is refused; the proof cuts
// the union of the four stand-ins in one Cut. Where decad refuses even that
// one cut, as a contact its chords cannot classify, the case is recorded as
// one the proof cannot represent.
func stepBoreSweepCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	tube := cgTube(t, doc, in)
	tool, _, err := cgChannels(t, doc, in)
	if err != nil {
		if cgRefused(err) {
			proofkit3d.Unmodelled(t, "decad refuses the union of the four bore stand-ins: %v", err)
		}
		t.Fatalf("uniting the four bore stand-ins: %v", err)
	}
	cut, err := decad.Cut(t.Context(), tube, tool)
	if err != nil {
		if cgRefused(err) {
			proofkit3d.Unmodelled(t, "decad refuses the bores' cut from the tube: %v", err)
		}
		t.Fatalf("cutting the bores: %v", err)
	}
	return []*decad.Body{cut}
}

func assertBoreSweepCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	if len(bodies) != 1 {
		t.Fatalf("the bores' cut left %d bodies, want 1", len(bodies))
	}
	ri, ro := in.Ri(), in.Ro()
	tubeVol := math.Pi * (ro*ro - ri*ri) * 2 * in.CageRise
	// The cut took material and took no more than the four channels hold:
	// the tube's volume less between none and all of the channels' summed
	// ruled-loft volume. The tube's volume is exact; the slack is that range.
	_, channels, err := cgChannels(t, decad.New(), in)
	if err != nil {
		t.Fatalf("rebuilding the channels: %v", err)
	}
	got := cgVol(t, "the cut tube", bodies[0])
	decadtest.Measures(t, "the tube after the bores' cut", got, cgMM3(tubeVol-channels/2),
		decadtest.Within(cgMM3(channels/2)))
	// The build's sense check ([SCREW-F-SWEEP-CHECK]): the two probes at each
	// crossing, origin + sc*dir +- (W/2 + clearance/2)*u(sc), stand in the
	// tube's wall, clear of the ribbon and of the channel's wall by
	// clearance/2. At the defaults they stand 16.86 mm (-R) and 16.31 mm (+R)
	// from the frame's axis.
	for b, name := range cgBoreNames {
		g := cgGears(in)[b/2]
		sc := [2]float64{-in.CageRadius, in.CageRadius}[b%2]
		th := g.theta(sc)
		u := g.u.Scale(math.Cos(th)).Add(g.v.Scale(math.Sin(th)))
		for _, side := range []float64{1, -1} {
			probe := g.axisPoint(sc).Add(u.Scale(side * (in.W/2 + in.Clearance/2)))
			r := math.Hypot(probe.X, probe.Y)
			if r < ri || r > ro || math.Abs(probe.Z) > in.CageRise {
				t.Fatalf("%s probe at radius %.3f, height %.3f is not in the tube's wall", name, r, probe.Z)
			}
			if cgMapKey(in) == cgMapKey(cgDefaults()) {
				want := [2]float64{16.86, 16.31}[b%2]
				if math.Abs(r-want) > 0.005 {
					t.Fatalf("%s probe stands %.4f mm from the frame's axis, the spec %.2f", name, r, want)
				}
			}
		}
	}
	// The sweep's twist and the stand-in's count at the defaults: 82.31
	// degrees over the cut and 18 sections.
	if cgMapKey(in) == cgMapKey(cgDefaults()) {
		twist := (in.SOut() - in.SIn()) / in.Lambda() * 180 / math.Pi
		if math.Abs(twist-82.31) > 0.005 {
			t.Fatalf("the bore's sweep turns %.4f degrees at the defaults, the spec 82.31", twist)
		}
		if in.BoreSections() != 18 {
			t.Fatalf("the stand-in lofts %d sections at the defaults, the spec 18", in.BoreSections())
		}
	}
	// At a 0.05 mm clearance the facet bound governs and the count is 32.
	thin := in
	thin.Clearance = 0.05
	if cgMapKey(in) == cgMapKey(cgDefaults()) && thin.BoreSections() != 32 {
		t.Fatalf("the stand-in lofts %d sections at a 0.05 mm clearance, the spec 32", thin.BoreSections())
	}
}

// cgPrism is a window's cut: its hexagon on the Window Plane, extruded
// Ro + 1 mm toward d. The plane's frame is (C, across, n), whose normal
// across x n is d, so the build's direction rule — positive when C + d maps
// to a positive sketch z — is Along here.
func cgPrism(t *testing.T, doc *decad.Document, in cgIn, w sleeveWindow) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(cgC, w.across, cgN)
	if err != nil {
		t.Fatalf("window plane: %v", err)
	}
	if z := f.ToLocal(cgC.Add(w.facing)).Z; z <= 0 {
		t.Fatalf("C + d maps to sketch z %.3f; the extrude would run away from d", z)
	}
	wd := sketch.NewWorld()
	pl, err := wd.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("window plane: %v", err)
	}
	s, err := wd.CreateSketch(pl)
	if err != nil {
		t.Fatalf("window sketch: %v", err)
	}
	cgFixedLoop(s, cgHexagon(w))
	sketchtest.Solve(t, s)
	prism, err := doc.Extrude(s, cgOneProfile(t, s), decad.Distance{D: units.Millimeters(in.Ro() + 1), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extruding the window: %v", err)
	}
	return prism
}

// cgPrismBox is the world box of a window's prism of the given depth.
func cgPrismBox(w sleeveWindow, depth float64) (r3.Vec, r3.Vec) {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := lo.Scale(-1)
	for _, q := range cgHexagon(w) {
		for _, a := range []float64{0, depth} {
			pt := cgC.Add(w.across.Scale(q[0])).Add(cgN.Scale(q[1])).Add(w.facing.Scale(a))
			lo = r3.NewVec(math.Min(lo.X, pt.X), math.Min(lo.Y, pt.Y), math.Min(lo.Z, pt.Z))
			hi = r3.NewVec(math.Max(hi.X, pt.X), math.Max(hi.Y, pt.Y), math.Max(hi.Z, pt.Z))
		}
	}
	return lo, hi
}

// stepWindowCut cuts one window from the tube. What is substituted: Fusion
// cuts both windows after the four bores, the cage alone as participant. A
// second decad cut on a faceted result is refused (stepBoreSweepCut), and a
// window's prism runs through the hollow where the bores' stand-ins start, so
// the proof cuts each window from the uncut tube, one window per case. That
// the windows and the bores together leave one piece is the mechanism proof's
// TestSleeveIsOnePiece and TestSleeveWindowsFollowTheSize, which hold the
// whole sleeve on a 0.25 mm grid.
func stepWindowCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	in := cgRead(t, p)
	w, ok := cgWindow(in, int(p["window"]))
	if !ok {
		t.Fatalf("the window search found no room for this window")
	}
	tube := cgTube(t, doc, in)
	cut, err := decad.Cut(t.Context(), tube, cgPrism(t, doc, in, w))
	if err != nil {
		if cgRefused(err) {
			proofkit3d.Unmodelled(t, "decad refuses the window's cut from the tube: %v", err)
		}
		t.Fatalf("cutting the window: %v", err)
	}
	return []*decad.Body{cut}
}

func assertWindowCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	in := cgRead(t, p)
	w, _ := cgWindow(in, int(p["window"]))
	if len(bodies) != 1 {
		t.Fatalf("the window's cut left %d bodies, want 1", len(bodies))
	}
	// The removed material, measured on its own: the tube and the prism
	// intersected in a document of their own. The cut and the intersection
	// together are the tube.
	other := decad.New()
	removed, err := decad.Intersect(t.Context(), cgTube(t, other, in), cgPrism(t, other, in, w))
	if err != nil {
		t.Fatalf("intersecting the tube and the window: %v", err)
	}
	ri, ro := in.Ri(), in.Ro()
	tubeVol := math.Pi * (ro*ro - ri*ri) * 2 * in.CageRise
	got, rem := cgVol(t, "the cut tube", bodies[0]), cgVol(t, "the window's material", removed)
	sum := decad.Measurement{Value: got.Value, Exactness: decad.Approximate, Bound: got.Bound}
	sum.Value, _ = got.Value.Add(rem.Value)
	sum.Bound, _ = got.Bound.Add(rem.Bound)
	decadtest.Measures(t, "the cut tube and the window's material together", sum, cgMM3(tubeVol),
		decadtest.Within(cgMM3(1e-9*tubeVol)))
	// The cut ran toward d: the prism stands on d's side of the Window Plane,
	// from the plane to Ro + 1 mm along d, over the hexagon. Its box is exact,
	// since d and across lie along world axes here; the slack is rounding.
	prism := cgPrism(t, decad.New(), in, w)
	lo, hi := cgPrismBox(w, in.Ro()+1)
	decadtest.MeasuresBounds(t, prism, lo, hi, decadtest.Within(units.Millimeters(1e-9)))
	// The build's probe, the middle of the wall where the window goes, is in
	// the tube before the cut and inside the window's prism, so the cut takes
	// it.
	var tc, zc float64
	hex := cgHexagon(w)
	for _, q := range hex {
		tc += q[0] / float64(len(hex))
		zc += q[1] / float64(len(hex))
	}
	a0, a1 := math.Sqrt(math.Max(0, ri*ri-tc*tc)), math.Sqrt(math.Max(0, ro*ro-tc*tc))
	probe := cgC.Add(w.across.Scale(tc)).Add(cgN.Scale(zc)).Add(w.facing.Scale((a0 + a1) / 2))
	if r := math.Hypot(probe.X, probe.Y); r < ri || r > ro || math.Abs(probe.Z) > in.CageRise {
		t.Fatalf("the probe stands at radius %.3f, height %.3f: not in the tube's wall", r, probe.Z)
	}
	if !w.contains(probe) {
		t.Fatalf("the probe is not inside the window's prism")
	}
}
