package screwgear_test

// The compiled step proof for spec/screwgear/steps.md: one function per build
// step, in compiled_sketches_test.go and compiled_solids_test.go, and the case
// tables and shared geometry in this file. It sits beside the hand-written
// mechanism proof (geometry_test.go, pair_test.go, bore_play_test.go,
// sleeve_test.go) and borrows that proof's model of the part rather than
// deriving it again: Params, Gear, pair, Gear.section and Gear.angle for the
// ribbon, and plainSleeve, newSleeve, newWindow, sectionInWall and
// boreSections for the sleeve. The two searches the build runs in
// processInputs, the wall between the bores and the window search, are the
// hand-written proof's channelSeparation and newWindow; this proof takes their
// results as numbers.
//
// Steps of the step list this proof does not build, and why, next to the
// nearest thing it does build:
//
//   - The dialog, the reading and range checks of the inputs, the two searches
//     of processInputs, the component tree and the relocation of the bodies are
//     not geometry; neither engine has anything to build for them.
//     sleeveRefusal, channelSeparation and newWindow in sleeve_test.go are the
//     searches, and TestSleeveInputsAreChecked holds the checks.
//   - The two Gear Axis Planes, each Bore Plane and the Window Plane are
//     construction planes. The sketch engine places a sketch on a plane but
//     reads nothing back from how Fusion made one, which is what those steps
//     are about (the sign of n̂, where setByDistanceOnPath puts the origin). The
//     Paths, bore section and window sketch steps below draw on the plane each
//     of them stands for, in that plane's own coordinates.

import (
	"context"
	"math"
	"math/bits"
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

// caseParams is the spec's default table with a case's overrides applied. The
// keys are the dialog's input ids, lengths in millimetres and the three angles
// in degrees, as the dialog shows them.
func caseParams(m map[string]float64) Params {
	p := sleeveParams()
	set := map[string]*float64{
		"ribbonWidth":     &p.Width,
		"ribbonThickness": &p.Thickness,
		"toothPitch":      &p.ToothPitch,
		"toothHeight":     &p.ToothHeight,
		"twistLead":       &p.TwistLead,
		"engagement":      &p.Engagement,
		"cageRadius":      &p.CageRadius,
		"cageRise":        &p.CageRise,
		"clearance":       &p.Clearance,
		"collarHalf":      &p.CollarHalf,
		"collarWall":      &p.CollarWall,
	}
	for k, dst := range set {
		if v, ok := m[k]; ok {
			*dst = v
		}
	}
	for k, dst := range map[string]*float64{
		"crossAngle":  &p.CrossAngle,
		"mountAngleA": &p.MountAngleA,
		"mountAngleB": &p.MountAngleB,
	} {
		if v, ok := m[k]; ok {
			*dst = v * math.Pi / 180
		}
	}
	if v, ok := m["toothCount"]; ok {
		p.ToothCount = int(v)
	}
	return p
}

// casePhase is the assembly phase a case builds gear B at: the input
// assemblyPhase, or the default when the case does not set it.
func casePhase(m map[string]float64) float64 {
	if v, ok := m["assemblyPhase"]; ok {
		return v
	}
	return assemblyPhase
}

// caseGears is the pair a case describes, placed as the spec's §1 frame
// places it: C at the origin, ê, k̂ and n̂ on X, Y and Z, gear A's axis A/2
// below the middle plane and gear B's above it.
func caseGears(m map[string]float64) (Params, [2]Gear) {
	p := caseParams(m)
	ga, gb := pair(p, p.Sigma(), 0, casePhase(m))
	return p, [2]Gear{ga, gb}
}

// caseGear is the one gear a case names with "gear", 0 for gear A and 1 for B.
func caseGear(m map[string]float64) (Params, Gear) {
	p, gs := caseGears(m)
	return p, gs[int(m["gear"])]
}

// axisGear is a gear laid on the world X axis, û on Y and v̂ on Z, with the
// mounting angle and tooth phase of g. The ribbon's own steps (the cell, its
// copies and its joins) are rigid in the gear's frame, so their proof builds
// them there, where the axis is a coordinate axis and a bounding box along it
// reads the stations directly.
func axisGear(g Gear) Gear {
	return Gear{
		P:      g.P,
		Origin: r3.NewVec(0, 0, 0),
		Ex:     r3.NewVec(0, 1, 0),
		Ey:     r3.NewVec(0, 0, 1),
		Ez:     r3.NewVec(1, 0, 0),
		Hand:   g.Hand,
		Mount:  g.Mount,
		Phase:  g.Phase,
	}
}

// cellSteps is the spec's n, the steps to the tooth the cell is lofted
// through: the larger of the count that keeps the twist between neighbouring
// sections under 2 degrees and the floor of eight.
func cellSteps(p Params) int {
	twist := int(math.Ceil((p.ToothPitch / p.Lambda()) / (2 * math.Pi / 180)))
	return max(twist, 8)
}

// cellCount is the spec's c, q and r: the teeth a cell holds,
// min(CELL_TEETH, N), the whole cells and the remainder.
func cellCount(p Params) (c, q, r int) {
	c = min(cellTeeth, p.ToothCount)
	return c, p.ToothCount / c, p.ToothCount % c
}

// ribbonStation is the station of the ribbon's j-th section counted from its
// negative end, s0 + j*P/n with s0 = Z0 - L/2. Every section the proof builds
// takes its station from here by its global index, so two pieces that meet
// at one section compute it once and meet exactly.
func ribbonStation(g Gear, j int) float64 {
	n := cellSteps(g.P)
	return g.Phase - g.P.Length()/2 + float64(j)*g.P.ToothPitch/float64(n)
}

// planeSketch is a fresh sketch on a plane through origin spanned by u and v,
// whose normal is u x v.
func planeSketch(t testing.TB, w *sketch.World, origin, u, v r3.Vec) (*sketch.Sketch, r3.Frame) {
	t.Helper()
	fr, err := r3.NewFrame(origin, u, v)
	if err != nil {
		t.Fatalf("section frame: %v", err)
	}
	pl, err := w.CreatePlaneFromFrame(fr)
	if err != nil {
		t.Fatalf("section plane: %v", err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("section sketch: %v", err)
	}
	return s, fr
}

// polygonSketch draws a closed polygon of fixed points and lines on a sketch:
// the corners are world points, mapped into the plane, and each line shares
// its two corners. The points are fixed after the last line exists, which is
// the order the spec's reference points are fixed in.
func polygonSketch(t testing.TB, s *sketch.Sketch, fr r3.Frame, corners []r3.Vec) *sketch.Profile {
	t.Helper()
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		l := fr.ToLocal(c)
		pts[i] = s.CreatePoint(l.X, l.Y)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	return decadtest.SolveRegion(t, s)
}

// sectionSolid lofts a solid through a run of planar sections, each square to
// g's axis at its own station, the way the sketch engine and decad can:
// decad's Loft takes two sections, so each neighbouring pair is lofted as a
// sheet of its walls, the two end sections are patched, and the sheets are
// stitched into one closed body.
//
// What decad builds between two sections is not the smooth surface Fusion's
// loft through many sections builds, and it is not quite the ruled surface the
// spec's chord arithmetic is about either: decad's loft splits each wall
// between two sections into two flat triangles. A wall that twists between
// its two sections is a warped quadrilateral, and one diagonal folds it
// outward and the other inward, by a quarter of the warp at the middle and by
// a twelfth of the warp times the wall's area in volume. sectionVolume bounds
// the volume by exactly that, and the spec's figures for the ruled loft are
// not what this solid is held to.
func sectionSolid(t testing.TB, doc *decad.Document, g Gear, stations []float64, corners func(s float64) [4]r3.Vec) *decad.Body {
	t.Helper()
	ctx := context.Background()
	w := sketch.NewWorld()
	sks := make([]*sketch.Sketch, len(stations))
	prs := make([]*sketch.Profile, len(stations))
	for k, s := range stations {
		c := corners(s)
		sks[k], prs[k] = func() (*sketch.Sketch, *sketch.Profile) {
			sk, fr := planeSketch(t, w, g.Origin.Add(g.Ez.Scale(s)), g.Ex, g.Ey)
			return sk, polygonSketch(t, sk, fr, c[:])
		}()
	}
	var sheets []*decad.Body
	for k := 0; k+1 < len(stations); k++ {
		b, err := doc.Loft(ctx, sks[k], prs[k], sks[k+1], prs[k+1], decad.WithSurfaceResult())
		if err != nil {
			t.Fatalf("loft between stations %.4f and %.4f: %v", stations[k], stations[k+1], err)
		}
		sheets = append(sheets, b)
	}
	for _, k := range []int{0, len(stations) - 1} {
		b, err := doc.Patch(ctx, sks[k], prs[k])
		if err != nil {
			t.Fatalf("cap at station %.4f: %v", stations[k], err)
		}
		sheets = append(sheets, b)
	}
	body, err := decad.Stitch(ctx, sheets...)
	if err != nil {
		t.Fatalf("stitch %d sections into one body: %v", len(stations), err)
	}
	if body.Kind() != decad.BodySolid {
		t.Fatalf("the stitched sections are a %v, want a solid", body.Kind())
	}
	return body
}

// sectionVolume is the volume of the ruled loft through a run of sections,
// and how far a loft that splits each wall into two flat triangles can stand
// from it. Each wall quadrilateral contributes, by the divergence theorem, the
// average of its two triangulations; that average is the ruled (bilinear)
// patch's own contribution, and either triangulation stands half their
// difference from it. The caps are flat. Both are exact arithmetic on the
// section corners, so the only error is float rounding.
func sectionVolume(sections [][4]r3.Vec) (volume, slack float64) {
	tri := func(a, b, c r3.Vec) float64 { return a.Dot(b.Cross(c)) / 6 }
	first, last := sections[0], sections[len(sections)-1]
	// The corners run counter-clockwise about the axis, so the far cap faces
	// along it and the near cap against it.
	volume += tri(last[0], last[1], last[2]) + tri(last[0], last[2], last[3])
	volume -= tri(first[0], first[1], first[2]) + tri(first[0], first[2], first[3])
	for k := 0; k+1 < len(sections); k++ {
		a, b := sections[k], sections[k+1]
		for i := range 4 {
			j := (i + 1) % 4
			one := tri(a[i], a[j], b[j]) + tri(a[i], b[j], b[i])
			other := tri(a[i], a[j], b[i]) + tri(a[j], b[j], b[i])
			volume += (one + other) / 2
			slack += math.Abs(one-other) / 2
		}
	}
	return volume, slack
}

// cornerBox is the bounding box of every section corner: the box of the
// faceted solid through those sections, whose vertices they are.
func cornerBox(sections [][4]r3.Vec) (lo, hi r3.Vec) {
	lo = r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi = r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, sec := range sections {
		for _, c := range sec {
			lo = r3.NewVec(math.Min(lo.X, c.X), math.Min(lo.Y, c.Y), math.Min(lo.Z, c.Z))
			hi = r3.NewVec(math.Max(hi.X, c.X), math.Max(hi.Y, c.Y), math.Max(hi.Z, c.Z))
		}
	}
	return lo, hi
}

// ribbonPiece is the stretch of the ribbon from its section `from` to its
// section `to`, by global index: the stations, the four corners of each, and
// the solid lofted through them in doc.
type ribbonPiece struct {
	stations []float64
	sections [][4]r3.Vec
	body     *decad.Body
}

func buildRibbonPiece(t testing.TB, doc *decad.Document, g Gear, from, to int) ribbonPiece {
	t.Helper()
	var piece ribbonPiece
	for j := from; j <= to; j++ {
		s := ribbonStation(g, j)
		piece.stations = append(piece.stations, s)
		piece.sections = append(piece.sections, g.section(s))
	}
	piece.body = sectionSolid(t, doc, g, piece.stations, g.section)
	return piece
}

// cellRange is the global section range of cell k, counted in whole cells of
// c teeth from the ribbon's negative end, n sections to the tooth.
func cellRange(p Params, k int) (int, int) {
	c, _, _ := cellCount(p)
	n := cellSteps(p)
	return k * c * n, (k + 1) * c * n
}

// remainderRange is the global section range of the remainder cell.
func remainderRange(p Params) (int, int) {
	_, q, r := cellCount(p)
	c, _, _ := cellCount(p)
	n := cellSteps(p)
	return q * c * n, (q*c + r) * n
}

// checkRibbonPiece holds a lofted stretch of ribbon to its sections: one
// solid whose box is the box of its corners, so every station and every
// corner is where the spec puts it, whose volume is the ruled loft's up to
// the triangle fold, and whose faces are two caps and two triangles per wall
// per step, so it was lofted through exactly the sections asked for.
func checkRibbonPiece(t *testing.T, name string, piece ribbonPiece) {
	t.Helper()
	box, err := piece.body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", name, err)
	}
	lo, hi := cornerBox(piece.sections)
	// The vertices are the corners themselves; the slack is float rounding.
	decadtest.MeasuresBox(t, name+" bounds", box, lo, hi, decadtest.Within(units.Millimeters(1e-9)))
	vol, err := piece.body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", name, err)
	}
	want, slack := sectionVolume(piece.sections)
	// The ruled loft's volume, and the most a loft whose walls are split into
	// flat triangles can stand from it (sectionVolume).
	decadtest.Measures(t, name+" volume", vol, units.CubicMillimeters(want),
		decadtest.Within(units.CubicMillimeters(slack+1e-9*want)))
	if got, want := len(piece.body.Faces()), 2+8*(len(piece.sections)-1); got != want {
		t.Errorf("%s has %d faces, want %d: two caps and two triangles per wall for each of its %d steps",
			name, got, want, len(piece.sections)-1)
	}
}

// screwStep is the spec's Step(k): a translation of k*P along the gear's
// axis composed with a rotation of k*P/Lambda about it.
func screwStep(t testing.TB, g Gear, teeth int) r3.Transform {
	t.Helper()
	k := float64(teeth)
	rot, err := r3.RotationAround(g.Origin, g.Ez, units.Radians(k*g.P.ToothPitch/g.lambda()))
	if err != nil {
		t.Fatalf("screw step rotation: %v", err)
	}
	move, err := r3.Translation(g.Ez.Scale(k * g.P.ToothPitch))
	if err != nil {
		t.Fatalf("screw step translation: %v", err)
	}
	step, err := rot.Then(move)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	return step
}

// scheduleOp is one feature group of the doubling schedule of spec §3: an
// aside copy, a doubling round, or the placing of an aside.
type scheduleOp struct {
	kind  string // "aside", "double" or "place"
	cells int    // the cells the body holds before the op
	piece int    // the cells the copy holds
	shift int    // the screw step the copy is moved by, in cells; 0 for an aside
}

// cellSchedule is the doubling schedule for q whole cells, in build order:
// for each bit of q below its top bit, lowest first, an aside copy of the
// body when the bit is set and then a doubling; then each aside, largest
// first, moved by the cells built so far and joined.
func cellSchedule(q int) []scheduleOp {
	var ops []scheduleOp
	var asides []int
	m := 1
	top := bits.Len(uint(q)) - 1
	for bit := 0; bit < top; bit++ {
		if q&(1<<bit) != 0 {
			ops = append(ops, scheduleOp{kind: "aside", cells: m, piece: m})
			asides = append(asides, m)
		}
		ops = append(ops, scheduleOp{kind: "double", cells: m, piece: m, shift: m})
		m *= 2
	}
	for i := len(asides) - 1; i >= 0; i-- {
		ops = append(ops, scheduleOp{kind: "place", cells: m, piece: asides[i], shift: m})
		m += asides[i]
	}
	return ops
}

// The case tables. Each is a package-level table a registration names. The
// default case of each is the spec's default table; the others reach the
// branches the step takes and the ends of the ranges the spec states.

var anchorCases = []proofkit.Case{
	{Name: "centre at the sketch origin", Params: map[string]float64{}},
	{Name: "centre off the origin", Params: map[string]float64{"centreX": 37.2, "centreY": -12.5}},
	{Name: "centre at negative coordinates", Params: map[string]float64{"centreX": -120, "centreY": -80}},
}

var pathsCases = []proofkit.Case{
	{Name: "defaults gear A", Params: map[string]float64{"gear": 0, "quoted": 1}},
	{Name: "defaults gear B", Params: map[string]float64{"gear": 1}},
	{Name: "gear A off the origin", Params: map[string]float64{"gear": 0, "centreX": -40, "centreY": 25}},
	{Name: "gear B at a 70 degree crossing", Params: map[string]float64{"gear": 1, "crossAngle": 70}},
	{Name: "gear A at a 120 degree crossing", Params: map[string]float64{"gear": 0, "crossAngle": 120}},
	{Name: "gear B at a 25 mm cage radius", Params: map[string]float64{"gear": 1, "cageRadius": 25, "cageRise": 25}},
	{Name: "gear A at the least inner radius the mesh check accepts", Params: map[string]float64{"gear": 0, "cageRadius": 14.2}},
}

// cellCases name the run of sections a Cell Sections sketch or its loft holds
// with "teeth" (the cell's c, or the remainder's r) and "firstTooth" (the
// tooth the run starts at, q*c for the remainder).
var cellSketchCases = []proofkit.Case{
	{Name: "defaults gear A", Params: map[string]float64{"gear": 0, "teeth": 4, "quoted": 1}},
	{Name: "defaults gear B", Params: map[string]float64{"gear": 1, "teeth": 4}},
	{Name: "lead 400 mm takes the floor of eight", Params: map[string]float64{"gear": 0, "teeth": 4, "twistLead": 400}},
	{Name: "lead 20 mm takes the twist count", Params: map[string]float64{"gear": 1, "teeth": 4, "twistLead": 20}},
	{Name: "negative mounting angle", Params: map[string]float64{"gear": 0, "teeth": 4, "mountAngleA": -30}},
	{Name: "assembly phase near minus a pitch", Params: map[string]float64{"gear": 1, "teeth": 4, "assemblyPhase": -2.6}},
	{Name: "assembly phase near plus a pitch", Params: map[string]float64{"gear": 1, "teeth": 4, "assemblyPhase": 2.6}},
	{Name: "deep tooth just under half the width", Params: map[string]float64{"gear": 0, "teeth": 4, "toothHeight": 7.4}},
	{Name: "remainder of one tooth at 69 teeth", Params: map[string]float64{"gear": 0, "teeth": 1, "firstTooth": 68, "toothCount": 69}},
	{Name: "remainder of three teeth at 71 teeth", Params: map[string]float64{"gear": 1, "teeth": 3, "firstTooth": 68, "toothCount": 71}},
}

var cellLoftCases = []proofkit3d.Case{
	{Name: "defaults gear A", Params: map[string]float64{"gear": 0, "teeth": 4}},
	{Name: "defaults gear B", Params: map[string]float64{"gear": 1, "teeth": 4}},
	{Name: "lead 400 mm takes the floor of eight", Params: map[string]float64{"gear": 0, "teeth": 4, "twistLead": 400}},
	{Name: "negative mounting angle", Params: map[string]float64{"gear": 0, "teeth": 4, "mountAngleA": -30}},
	{Name: "remainder of one tooth at 69 teeth", Params: map[string]float64{"gear": 0, "teeth": 1, "firstTooth": 68, "toothCount": 69}},
}

// The copy and move cases name the cells the screw step moves by with
// "shift": every shift the default schedule makes, 1, 2, 4, 8 and 16 cells.
var copyCases = []proofkit3d.Case{
	{Name: "defaults gear A", Params: map[string]float64{"gear": 0}},
	{Name: "defaults gear B", Params: map[string]float64{"gear": 1}},
}

var moveCases = []proofkit3d.Case{
	{Name: "gear A by one cell", Params: map[string]float64{"gear": 0, "shift": 1}},
	{Name: "gear A by two cells", Params: map[string]float64{"gear": 0, "shift": 2}},
	{Name: "gear A by four cells", Params: map[string]float64{"gear": 0, "shift": 4}},
	{Name: "gear A by eight cells", Params: map[string]float64{"gear": 0, "shift": 8}},
	{Name: "gear A by sixteen cells", Params: map[string]float64{"gear": 0, "shift": 16}},
	{Name: "gear B by sixteen cells", Params: map[string]float64{"gear": 1, "shift": 16}},
	{Name: "negative mounting angle by one cell", Params: map[string]float64{"gear": 0, "shift": 1, "mountAngleA": -30}},
	{Name: "lead 400 mm by one cell", Params: map[string]float64{"gear": 0, "shift": 1, "twistLead": 400}},
}

var joinCases = []proofkit3d.Case{
	{Name: "defaults gear A, 17 cells", Params: map[string]float64{"gear": 0}},
	{Name: "defaults gear B, 17 cells", Params: map[string]float64{"gear": 1}},
	{Name: "one cell and no round", Params: map[string]float64{"gear": 0, "toothCount": 4}},
	{Name: "one cell and a remainder", Params: map[string]float64{"gear": 0, "toothCount": 5}},
	{Name: "two cells, one doubling", Params: map[string]float64{"gear": 0, "toothCount": 8}},
	{Name: "three cells, an aside", Params: map[string]float64{"gear": 1, "toothCount": 12}},
	{Name: "69 teeth, a remainder after five rounds", Params: map[string]float64{"gear": 0, "toothCount": 69}},
}

var sleeveSketchCases = []proofkit.Case{
	{Name: "defaults", Params: map[string]float64{}},
	{Name: "centre off the origin", Params: map[string]float64{"centreX": 12.5, "centreY": -40}},
	{Name: "cage radius 25 mm", Params: map[string]float64{"cageRadius": 25, "cageRise": 25}},
	{Name: "collar half 2 mm", Params: map[string]float64{"collarHalf": 2}},
}

var sleeveTubeCases = []proofkit3d.Case{
	{Name: "defaults", Params: map[string]float64{}},
	{Name: "cage radius 25 mm", Params: map[string]float64{"cageRadius": 25, "cageRise": 25}},
	{Name: "collar half 3.5 mm", Params: map[string]float64{"collarHalf": 3.5}},
}

// The bore section cases name the bore with "bore": 0 and 1 for gear A's -R
// and +R, 2 and 3 for gear B's. "offsetX" and "offsetY" put the axis point O
// away from the plane's own origin, as Fusion's plane origin need not be on
// it. The mounting angles of -60, 122 and 138 degrees turn the section to
// within 45 degrees of 0 or 180 at the bore's first station, which is where
// the angle is taken against the toothed side instead of the spine.
var boreSketchCases = []proofkit.Case{
	{Name: "defaults gear A -R", Params: map[string]float64{"bore": 0}},
	{Name: "defaults gear A +R", Params: map[string]float64{"bore": 1, "offsetX": 3, "offsetY": -2}},
	{Name: "defaults gear B -R", Params: map[string]float64{"bore": 2}},
	{Name: "defaults gear B +R", Params: map[string]float64{"bore": 3}},
	{Name: "toothed-side angle near 0 on a +R bore", Params: map[string]float64{"bore": 1, "mountAngleA": -60}},
	{Name: "toothed-side angle near 180 on a +R bore", Params: map[string]float64{"bore": 3, "mountAngleB": 122}},
	{Name: "toothed-side angle near 0 on a -R bore", Params: map[string]float64{"bore": 0, "mountAngleA": 138}},
	{Name: "clearance 0.05 mm", Params: map[string]float64{"bore": 1, "clearance": 0.05}},
	{Name: "clearance 0.9 mm", Params: map[string]float64{"bore": 2, "clearance": 0.9, "cageRise": 19}},
}

var boreCutCases = []proofkit3d.Case{
	{Name: "defaults", Params: map[string]float64{}},
	{Name: "clearance 0.05 mm", Params: map[string]float64{"clearance": 0.05}},
	{Name: "crossing 100 degrees", Params: map[string]float64{"crossAngle": 100}},
	{Name: "mounting angles 0 and 30 degrees", Params: map[string]float64{"mountAngleA": 0, "mountAngleB": 30}},
}

// The window cases name the window with "window": 0 for the one facing d and
// 1 for the one facing -d.
var windowSketchCases = []proofkit.Case{
	{Name: "defaults +k", Params: map[string]float64{"window": 0, "quoted": 1}},
	{Name: "defaults -k", Params: map[string]float64{"window": 1}},
	{Name: "crossing 100 degrees +e", Params: map[string]float64{"window": 0, "crossAngle": 100}},
	{Name: "crossing 100 degrees -e", Params: map[string]float64{"window": 1, "crossAngle": 100}},
	{Name: "mounting angles 0 and 30 degrees -k", Params: map[string]float64{"window": 1, "mountAngleA": 0, "mountAngleB": 30}},
}

var windowCutCases = []proofkit3d.Case{
	{Name: "defaults", Params: map[string]float64{}},
	{Name: "crossing 100 degrees", Params: map[string]float64{"crossAngle": 100}},
}

// caseSleeve is the sleeve a case describes, with its windows found by the
// hand-written proof's newWindow, which is the build's window search step for
// step. It is the same search the build runs in processInputs.
func caseSleeve(m map[string]float64) sleeve {
	_, gs := caseGears(m)
	return newSleeve(gs[0], gs[1])
}

// windowCorners is a window's hexagon as the build draws it: newWindow's
// corners with any corner within 0.001 mm of the one before it dropped, since
// a sketch line cannot have zero length.
func windowCorners(w sleeveWindow) [][2]float64 {
	var out [][2]float64
	for _, c := range w.corners {
		if n := len(out); n > 0 && math.Hypot(c[0]-out[n-1][0], c[1]-out[n-1][1]) < 0.001 {
			continue
		}
		out = append(out, c)
	}
	if n := len(out); n > 1 && math.Hypot(out[0][0]-out[n-1][0], out[0][1]-out[n-1][1]) < 0.001 {
		out = out[:n-1]
	}
	return out
}

// polygonArea is a polygon's signed area by the shoelace formula.
func polygonArea(q [][2]float64) float64 {
	a := 0.0
	for i := range q {
		p, r := q[i], q[(i+1)%len(q)]
		a += p[0]*r[1] - r[0]*p[1]
	}
	return a / 2
}

// boreRemoval is the volume a bore's channel takes out of the uncut tube: the
// area of the channel's section in the wall, sectionInWall's two pieces,
// integrated along the cut's span by the trapezoid rule at 1 µm. The section
// is square to the gear's axis, so that integral is the volume.
func boreRemoval(f sleeve, b bore) float64 {
	lo, hi := b.span(f)
	const ds = 0.001
	area := func(s float64) float64 {
		a := 0.0
		for _, piece := range f.sectionInWall(b.g, s) {
			if piece.n >= 3 {
				a += polygonArea(piece.v[:piece.n])
			}
		}
		return a
	}
	n := int(math.Ceil((hi - lo) / ds))
	h := (hi - lo) / float64(n)
	sum := (area(lo) + area(hi)) / 2
	for i := 1; i < n; i++ {
		sum += area(lo + float64(i)*h)
	}
	return sum * h
}

// windowRemoval is the volume a window's prism takes out of the uncut tube:
// at each t of the hexagon, its height times the wall's chord a1(t) - a0(t),
// integrated by the trapezoid rule at 1 µm.
func windowRemoval(f sleeve, corners [][2]float64) float64 {
	left, right := math.Inf(1), math.Inf(-1)
	for _, c := range corners {
		left, right = math.Min(left, c[0]), math.Max(right, c[0])
	}
	height := func(t float64) float64 {
		lo, hi := math.Inf(1), math.Inf(-1)
		for i := range corners {
			p, r := corners[i], corners[(i+1)%len(corners)]
			if (p[0]-t)*(r[0]-t) > 0 || p[0] == r[0] {
				if p[0] == t {
					lo, hi = math.Min(lo, p[1]), math.Max(hi, p[1])
				}
				continue
			}
			z := p[1] + (r[1]-p[1])*(t-p[0])/(r[0]-p[0])
			lo, hi = math.Min(lo, z), math.Max(hi, z)
		}
		if hi < lo {
			return 0
		}
		a0, a1 := f.chord(t)
		return (hi - lo) * (a1 - a0)
	}
	const dt = 0.001
	n := int(math.Ceil((right - left) / dt))
	h := (right - left) / float64(n)
	sum := (height(left) + height(right)) / 2
	for i := 1; i < n; i++ {
		sum += height(left + float64(i)*h)
	}
	return sum * h
}

// boreTool is the proof's stand-in for one bore's twisted sweep: the bore's
// rectangle, turned to the ribbon's angle, at the sections boreSections
// derives over the cut's span, lofted and stitched by sectionSolid. It returns
// the tool and the most its triangle-split walls can take from or add to the
// ruled channel's volume, as sectionVolume reckons it.
func boreTool(t testing.TB, doc *decad.Document, f sleeve, b bore) (*decad.Body, float64) {
	t.Helper()
	p := f.p
	lo, hi := b.span(f)
	n := boreSections(p, (hi-lo)/p.Lambda())
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	rect := func(s float64) [4]r3.Vec {
		var c [4]r3.Vec
		for i, uv := range [4][2]float64{{-hw, -ht}, {hw, -ht}, {hw, ht}, {-hw, ht}} {
			c[i] = b.g.world(uv[0], uv[1], s)
		}
		return c
	}
	stations := make([]float64, n)
	sections := make([][4]r3.Vec, n)
	for k := range n {
		stations[k] = lo + (hi-lo)*float64(k)/float64(n-1)
		sections[k] = rect(stations[k])
	}
	_, slack := sectionVolume(sections)
	return sectionSolid(t, doc, b.g, stations, rect), slack
}

// boreFacetSlack is how much the ruled channel through boreSections' sections
// can fall short of the exact swept channel in the volume it removes: each
// facet stands at most c*(1 - cos(step/2)) inside the channel, over a wall no
// larger than the rectangle's perimeter times the cut's span.
func boreFacetSlack(f sleeve, b bore) float64 {
	p := f.p
	lo, hi := b.span(f)
	turn := (hi - lo) / p.Lambda()
	n := boreSections(p, turn)
	step := turn / float64(n-1)
	depth := p.BoreCorner() * (1 - math.Cos(step/2))
	return depth * 4 * (p.BoreHalfWidth() + p.BoreHalfThickness()) * (hi - lo)
}

// tubeBody extrudes the sleeve's ring from a Sleeve sketch on the plane
// through C square to n̂, both ways by cageRise.
func tubeBody(t testing.TB, doc *decad.Document, p Params) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, _ := planeSketch(t, w, r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	ri, ro := p.SleeveInner(), p.SleeveOuter()
	inner := s.CreateCircle(s.CreatePoint(0, 0), ri)
	outer := s.CreateCircle(s.CreatePoint(0, 0), ro)
	s.Fix(inner.Center)
	s.Fix(outer.Center)
	s.AddConstraint(sketch.NewDiameter(inner, 2*ri), sketch.NewDiameter(outer, 2*ro))
	ring := ringProfile(t, s)
	body, err := doc.Extrude(s, ring, decad.Symmetric{D: units.Millimeters(p.CageRise)})
	if err != nil {
		t.Fatalf("extrude the sleeve's ring: %v", err)
	}
	return body
}

// ringProfile solves a Sleeve sketch and returns the one profile with a hole,
// the ring, failing unless there is exactly one.
func ringProfile(t testing.TB, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	sketchtest.Solve(t, s)
	var ring []*sketch.Profile
	for _, pr := range s.Profiles() {
		if len(pr.Holes) == 1 {
			ring = append(ring, pr)
		}
	}
	if len(ring) != 1 {
		t.Fatalf("the Sleeve sketch has %d profiles with one hole, want exactly 1", len(ring))
	}
	return ring[0]
}

// windowPrism extrudes a window's hexagon square to its facing direction, one
// way, toward that direction, to Ro + 1 mm from the frame's axis.
func windowPrism(t testing.TB, doc *decad.Document, f sleeve, w sleeveWindow) *decad.Body {
	t.Helper()
	ws := sketch.NewWorld()
	n := r3.NewVec(0, 0, 1)
	// The prism starts halfway to the inner face at the window's outermost
	// end (stepWindowExtrudeCut says why); its far face stays Ro + 1 mm out.
	start := math.Inf(1)
	for _, c := range windowCorners(w) {
		a0, _ := f.chord(c[0])
		start = math.Min(start, a0/2)
	}
	s, fr := planeSketch(t, ws, w.facing.Scale(start), w.across, n)
	if d := fr.N().Sub(w.facing).Len(); d > 1e-12 {
		t.Fatalf("the window plane's normal misses its facing direction by %.3e", d)
	}
	corners := windowCorners(w)
	world := make([]r3.Vec, len(corners))
	for i, c := range corners {
		world[i] = w.across.Scale(c[0]).Add(n.Scale(c[1])).Add(w.facing.Scale(start))
	}
	pr := polygonSketch(t, s, fr, world)
	body, err := doc.Extrude(s, pr, decad.Distance{D: units.Millimeters(f.ro + 1 - start), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude window facing %v: %v", w.facing, err)
	}
	return body
}

func faceName(d r3.Vec) string {
	switch {
	case d.Y > 0.5:
		return "+k"
	case d.Y < -0.5:
		return "-k"
	case d.X > 0.5:
		return "+e"
	default:
		return "-e"
	}
}

