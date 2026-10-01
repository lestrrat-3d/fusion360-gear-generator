package screwgear_test

// The compiled step proof's shared model: the inputs a case carries, the
// frame of spec/screwgear/instructions.md §1, the sections every sketch and
// loft is drawn from, the doubling schedule of §3, and the builders the solid
// steps share. Everything here is written from spec/screwgear/steps.md, and
// every name carries the cg prefix so it cannot collide with the hand-written
// mechanism proof that shares this package (geometry_test.go, pair_test.go,
// sleeve_test.go, cage_test.go, render_test.go).
//
// The hand-written proof is used in three places only, each named where it is
// called: sleeveRefusal decides which inputs the build accepts (spec
// "Variables", the sleeve's four checks), newSleeve runs the two searches of
// spec §4 the build runs before any feature (the wall between the bores and
// the window search), whose results the compiled proof takes as numbers, and
// windowSizes is the regime the spec names for the sleeve.

import (
	"fmt"
	"math"
	"math/bits"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
)

// cgCellTeeth is the module constant CELL_TEETH: the teeth the lofted cell
// holds. A case may override it through the "cellTeeth" key to reach the
// one-tooth fallback the spec states counts for.
const cgCellTeeth = 4

// cgIn is one case's dialog inputs, in millimetres and radians, plus the
// cell size.
type cgIn struct {
	W, T, H, P, Lead       float64
	Sigma, PhiA, PhiB, Eng float64
	N                      int
	CageRadius, CageRise   float64
	CollarHalf, CollarWall float64
	Clearance, Phase       float64
	CellTeeth              int
}

// cgDefaults is the spec's default table ("Variables").
func cgDefaults() cgIn {
	return cgIn{
		W: 15, T: 3.75, H: 2.625, P: 2.625, Lead: 49.5,
		Sigma: 80 * math.Pi / 180, PhiA: 15 * math.Pi / 180, PhiB: 15 * math.Pi / 180, Eng: 0.75,
		N:          68,
		CageRadius: 15, CageRise: 18.75, CollarHalf: 3, CollarWall: 3,
		Clearance: 0.45, Phase: -1.31,
		CellTeeth: cgCellTeeth,
	}
}

var cgKeys = []string{"width", "thickness", "toothHeight", "toothPitch", "twistLead", "crossAngleDeg",
	"mountAngleADeg", "mountAngleBDeg", "engagement", "toothCount", "cageRadius", "cageRise", "collarHalf",
	"collarWall", "clearance", "assemblyPhase", "cellTeeth"}

// cgMap writes the inputs as a case's parameter map, with extra keys added.
func cgMap(in cgIn, extra map[string]float64) map[string]float64 {
	m := map[string]float64{
		"width": in.W, "thickness": in.T, "toothHeight": in.H, "toothPitch": in.P, "twistLead": in.Lead,
		"crossAngleDeg": in.Sigma * 180 / math.Pi, "mountAngleADeg": in.PhiA * 180 / math.Pi,
		"mountAngleBDeg": in.PhiB * 180 / math.Pi, "engagement": in.Eng, "toothCount": float64(in.N),
		"cageRadius": in.CageRadius, "cageRise": in.CageRise, "collarHalf": in.CollarHalf,
		"collarWall": in.CollarWall, "clearance": in.Clearance, "assemblyPhase": in.Phase,
		"cellTeeth": float64(in.CellTeeth),
	}
	for k, v := range extra {
		m[k] = v
	}
	return m
}

// cgRead reads a case's parameter map back. A missing key is a table defect.
func cgRead(t testing.TB, m map[string]float64) cgIn {
	t.Helper()
	for _, k := range cgKeys {
		if _, ok := m[k]; !ok {
			t.Fatalf("case is missing the %q input", k)
		}
	}
	return cgIn{
		W: m["width"], T: m["thickness"], H: m["toothHeight"], P: m["toothPitch"], Lead: m["twistLead"],
		Sigma: m["crossAngleDeg"] * math.Pi / 180, PhiA: m["mountAngleADeg"] * math.Pi / 180,
		PhiB: m["mountAngleBDeg"] * math.Pi / 180, Eng: m["engagement"], N: int(math.Round(m["toothCount"])),
		CageRadius: m["cageRadius"], CageRise: m["cageRise"], CollarHalf: m["collarHalf"],
		CollarWall: m["collarWall"], Clearance: m["clearance"], Phase: m["assemblyPhase"],
		CellTeeth: int(math.Round(m["cellTeeth"])),
	}
}

// cgFromParams carries the hand-written proof's Params into a case, with the
// assembly phase the spec defaults it to.
func cgFromParams(p Params) cgIn {
	return cgIn{
		W: p.Width, T: p.Thickness, H: p.ToothHeight, P: p.ToothPitch, Lead: p.TwistLead,
		Sigma: p.CrossAngle, PhiA: p.MountAngleA, PhiB: p.MountAngleB, Eng: p.Engagement,
		N:          p.ToothCount,
		CageRadius: p.CageRadius, CageRise: p.CageRise, CollarHalf: p.CollarHalf, CollarWall: p.CollarWall,
		Clearance: p.Clearance, Phase: assemblyPhase, CellTeeth: cgCellTeeth,
	}
}

// cgParams is the hand-written proof's Params for these inputs, which is what
// its sleeve searches read. The video frame's ring, wire and rod inputs are
// not the sleeve's and keep their defaults.
func (in cgIn) cgParams() Params {
	p := sleeveParams()
	p.Width, p.Thickness, p.ToothHeight, p.ToothPitch, p.TwistLead = in.W, in.T, in.H, in.P, in.Lead
	p.CrossAngle, p.MountAngleA, p.MountAngleB, p.Engagement = in.Sigma, in.PhiA, in.PhiB, in.Eng
	p.ToothCount = in.N
	p.CageRadius, p.CageRise, p.CollarHalf, p.CollarWall, p.Clearance =
		in.CageRadius, in.CageRise, in.CollarHalf, in.CollarWall, in.Clearance
	return p
}

// The derived lengths of "Geometry" and §4.
func (in cgIn) Lambda() float64 { return in.Lead / (2 * math.Pi) }
func (in cgIn) A() float64      { return in.W - in.Eng }
func (in cgIn) L() float64      { return float64(in.N) * in.P }
func (in cgIn) Ri() float64     { return in.CageRadius - in.CollarHalf }
func (in cgIn) Ro() float64     { return in.CageRadius + in.CollarHalf }
func (in cgIn) Hw() float64     { return in.W/2 + in.Clearance }
func (in cgIn) Ht() float64     { return in.T/2 + in.Clearance }
func (in cgIn) Corner() float64 { return math.Hypot(in.Hw(), in.Ht()) }

// SIn and SOut are the bore cut's span on its gear's axis (§4): a millimetre
// before the channel's corner first touches the inner face, and a millimetre
// past the outer face.
func (in cgIn) SIn() float64 {
	ri, c := in.Ri(), in.Corner()
	return math.Sqrt(ri*ri-c*c) - 1
}
func (in cgIn) SOut() float64 { return in.Ro() + 1 }

// StepsPerTooth is n of §2: the larger of the twist count, which keeps
// neighbouring sections under 2 degrees apart, and the floor of eight.
func (in cgIn) StepsPerTooth() int {
	n := int(math.Ceil((in.P / in.Lambda()) / (2 * math.Pi / 180)))
	return max(n, 8)
}

// Cell is c of §2: the teeth one cell holds.
func (in cgIn) Cell() int { return min(in.CellTeeth, in.N) }

// BoreSections is the stand-in's section count for one bore ("What the
// proof's stand-in costs"): no two neighbouring sections more than 5 degrees
// of twist apart, nor further than the facet bound 2*acos(1 - 0.04*clearance/c).
func (in cgIn) BoreSections() int {
	turn := (in.SOut() - in.SIn()) / in.Lambda()
	step := math.Min(5*math.Pi/180, 2*math.Acos(1-0.04*in.Clearance/in.Corner()))
	return int(math.Ceil(turn/step)) + 1
}

// ---------------------------------------------------------------------------
// The frame of §1. The proof's selected plane is world XY, C is the origin,
// e is world X and n is world Z: Fusion reads n off the Gear A Axis Plane and
// signs it so that C lies +A/2 along it ([SCREW-F-NORMAL-SIGN]), which the
// proof has by construction.

var (
	cgC = r3.NewVec(0, 0, 0)
	cgE = r3.NewVec(1, 0, 0)
	cgN = r3.NewVec(0, 0, 1)
	cgK = cgN.Cross(cgE) // k = n x e
)

// cgGear is one gear placed by §1.
type cgGear struct {
	in      cgIn
	index   int    // 0 for gear A, 1 for gear B
	label   string // "Gear A" or "Gear B"
	origin  r3.Vec // origin_g
	dir     r3.Vec // dir_g, the axis
	u, v    r3.Vec // the unrotated section frame: u points at the other gear, v = dir x u
	phi, z0 float64
}

// cgGears places both gears: dirA = rotate(e, +Sigma/2 about n), originA =
// C - (A/2) n, uA = +n; dirB = rotate(e, -Sigma/2 about n), originB =
// C + (A/2) n, uB = -n; v = dir x u. Gear A's tooth phase is 0 and gear B's is
// the assembly phase.
func cgGears(in cgIn) [2]cgGear {
	h := in.Sigma / 2
	rot := func(a float64) r3.Vec { return cgE.Scale(math.Cos(a)).Add(cgK.Scale(math.Sin(a))) }
	ga := cgGear{in: in, index: 0, label: "Gear A", origin: cgC.Sub(cgN.Scale(in.A() / 2)), dir: rot(h),
		u: cgN, phi: in.PhiA, z0: 0}
	gb := cgGear{in: in, index: 1, label: "Gear B", origin: cgC.Add(cgN.Scale(in.A() / 2)), dir: rot(-h),
		u: cgN.Scale(-1), phi: in.PhiB, z0: in.Phase}
	ga.v = ga.dir.Cross(ga.u)
	gb.v = gb.dir.Cross(gb.u)
	return [2]cgGear{ga, gb}
}

func (g cgGear) theta(s float64) float64 { return s/g.in.Lambda() + g.phi }

// uTooth is the toothed edge at station s: crest at W/2, root at W/2 - H.
func (g cgGear) uTooth(s float64) float64 {
	in := g.in
	return in.W/2 - in.H/2 + in.H/2*math.Cos(2*math.Pi*(s-g.z0)/in.P)
}

// turn rotates section coordinates (u, v) by the section angle at s, giving
// the point's coordinates along the unrotated u and v directions: the plane
// coordinates of a sketch whose x axis is u and y axis is v.
func (g cgGear) turn(u, v, s float64) (float64, float64) {
	c, sn := math.Cos(g.theta(s)), math.Sin(g.theta(s))
	return u*c - v*sn, u*sn + v*c
}

func (g cgGear) axisPoint(s float64) r3.Vec { return g.origin.Add(g.dir.Scale(s)) }

// world is the point of §1: origin + s dir + (u cos - v sin) u + (u sin + v cos) v.
func (g cgGear) world(u, v, s float64) r3.Vec {
	x, y := g.turn(u, v, s)
	return g.axisPoint(s).Add(g.u.Scale(x)).Add(g.v.Scale(y))
}

// s0 is the ribbon's negative end, Z0 - L/2 (§2).
func (g cgGear) s0() float64 { return g.z0 - g.in.L()/2 }

// cellCorners are a ribbon section's corners at station s, (uB, -hv),
// (uF, -hv), (uF, hv), (uB, hv) with uB = -W/2, uF = Utooth(s), hv = T/2, in
// the section plane's coordinates.
func (g cgGear) cellCorners(s float64) [4][2]float64 {
	uB, uF, hv := -g.in.W/2, g.uTooth(s), g.in.T/2
	var out [4][2]float64
	for i, q := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		out[i][0], out[i][1] = g.turn(q[0], q[1], s)
	}
	return out
}

// boreCorners are a bore's rectangle at station s: uB = -hw, uF = hw, hv = ht.
func (g cgGear) boreCorners(s float64) [4][2]float64 {
	hw, ht := g.in.Hw(), g.in.Ht()
	var out [4][2]float64
	for i, q := range [4][2]float64{{-hw, -ht}, {hw, -ht}, {hw, ht}, {-hw, ht}} {
		out[i][0], out[i][1] = g.turn(q[0], q[1], s)
	}
	return out
}

// sectionFrame is the plane square to the axis at station s, its x axis the
// gear's unrotated u and its y axis v, so its normal runs along +dir.
func (g cgGear) sectionFrame(t testing.TB, s float64) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(g.axisPoint(s), g.u, g.v)
	if err != nil {
		t.Fatalf("%s section frame at %.4f: %v", g.label, s, err)
	}
	return f
}

// screwStep is Step(k) of §3: k teeth along the axis and k*P/Lambda about it,
// right-handed about +dir, which carries the section at s onto the one at
// s + k*P because theta grows with s.
func (g cgGear) screwStep(t testing.TB, k int) r3.Transform {
	t.Helper()
	in := g.in
	rot, err := r3.RotationAround(g.origin, g.dir, units.Radians(float64(k)*in.P/in.Lambda()))
	if err != nil {
		t.Fatalf("screw rotation: %v", err)
	}
	shift, err := r3.Translation(g.dir.Scale(float64(k) * in.P))
	if err != nil {
		t.Fatalf("screw translation: %v", err)
	}
	step, err := rot.Then(shift)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	return step
}

// ---------------------------------------------------------------------------
// The doubling schedule of §3.

// cgOp is one placement of the schedule. An "aside" is a copy kept unmoved,
// holding cells cells. A "double" copies the body of cells cells, moves the
// copy by Step(cells*c) and joins it. A "place" moves the aside of cells
// cells by Step(by*c), by being the cells the body holds, and joins it.
type cgOp struct {
	kind      string
	cells, by int
}

// cgSchedule is the schedule for q cells: q in binary; for each bit below
// the top one, lowest first, an aside of the body when the bit is set and
// then a doubling; then each aside, largest first, moved to the body's end
// and joined.
func cgSchedule(q int) []cgOp {
	var ops []cgOp
	var asides []int
	m := 1
	top := bits.Len(uint(q)) - 1
	for b := 0; b < top; b++ {
		if q&(1<<b) != 0 {
			ops = append(ops, cgOp{kind: "aside", cells: m})
			asides = append(asides, m)
		}
		ops = append(ops, cgOp{kind: "double", cells: m, by: m})
		m *= 2
	}
	for i := len(asides) - 1; i >= 0; i-- {
		ops = append(ops, cgOp{kind: "place", cells: asides[i], by: m})
		m += asides[i]
	}
	return ops
}

// cgRounds is the count §3 states: floor(log2 q) + popcount(q) - 1.
func cgRounds(q int) int {
	if q < 1 {
		return 0
	}
	return bits.Len(uint(q)) - 1 + bits.OnesCount(uint(q)) - 1
}

// ---------------------------------------------------------------------------
// Sketch helpers.

// cgFixedLoop draws a closed loop of solid lines through points created at
// the given plane coordinates, sharing every corner, and fixes the points
// after the last line exists: the reference-point recipe of
// [SCREW-F-REFERENCES], where the engine's Fix is Fusion's isFixed. It returns
// the lines in drawing order.
func cgFixedLoop(s *sketch.Sketch, corners [][2]float64) []*sketch.Line {
	pts := make([]*sketch.Point, len(corners))
	for i, q := range corners {
		pts[i] = s.CreatePoint(q[0], q[1])
	}
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, p := range pts {
		s.Fix(p)
	}
	return lines
}

// cgSection is one planar section on its own plane in a world: the sketch,
// its one profile, and its four lines in drawing order (L1..L4).
type cgSection struct {
	s     *sketch.Sketch
	prof  *sketch.Profile
	lines []*sketch.Line
	world [4]r3.Vec // the corners in world space, for the volume oracle
}

// cgNewSection draws corners on the plane at frame f, solves it and takes
// its single valid profile.
func cgNewSection(t testing.TB, w *sketch.World, f r3.Frame, corners [4][2]float64) cgSection {
	t.Helper()
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("section plane: %v", err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("section sketch: %v", err)
	}
	sec := cgSection{s: s, lines: cgFixedLoop(s, corners[:])}
	sketchtest.Solve(t, s)
	sec.prof = cgOneProfile(t, s)
	for i, q := range corners {
		sec.world[i] = f.ToWorldUV(q[0], q[1])
	}
	return sec
}

// cgOneProfile is the sketch's single profile, which must be valid.
func cgOneProfile(t testing.TB, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	ps := s.Profiles()
	if len(ps) != 1 {
		t.Fatalf("a section sketch has %d profiles, want 1", len(ps))
	}
	sketchtest.IsValidProfile(t, ps[0])
	return ps[0]
}

// cgLineIndex is where line i of the drawing order sits in the profile's
// outer loop, so two sections' loops can be paired line for line.
func cgLineIndex(t testing.TB, sec cgSection, i int) int {
	t.Helper()
	for j, e := range sec.prof.Outer {
		if e.Entity == sketch.Entity(sec.lines[i]) {
			return j
		}
	}
	t.Fatalf("line %d of a section is not on its profile's outer loop", i+1)
	return -1
}

// ---------------------------------------------------------------------------
// The ruled-loft stand-in.
//
// Fusion lofts one smooth surface through every section of a cell
// ([SCREW-F-CELL-LOFT]); decad lofts only between two sections, ruled. The
// stand-in is a ruled loft between each pair of neighbouring sections, built
// as an open wall sheet (no caps), the walls stitched into one open tube and
// closed by a planar patch on its first and last section. A union of the
// two-section solids would be the obvious build, but decad refuses it: two
// slabs meeting on their shared section are "two operand facets overlap in one
// plane", a contact its exact predicates cannot classify (measured at the
// pinned revision). Stitching the walls is that union exactly, since the
// shared section is interior to it, and Stitch audits the walls for crossing,
// so an overlap would be refused rather than built.

// cgLoftWalls lofts each neighbouring pair of sections as a wall sheet,
// pairing line i of one with line i of the next, and stitches them into one
// open sheet.
func cgLoftWalls(t testing.TB, doc *decad.Document, secs []cgSection) *decad.Body {
	t.Helper()
	ctx := t.Context()
	walls := make([]*decad.Body, 0, len(secs)-1)
	for k := 0; k+1 < len(secs); k++ {
		a, b := secs[k], secs[k+1]
		// Segment 0 of a's outer loop pairs with the segment of b's that is the
		// same line of the drawing order, so line i of a meets line i of b.
		off := cgLineIndex(t, b, cgLineAt(t, a, 0))
		w, err := doc.Loft(ctx, a.s, a.prof, b.s, b.prof, decad.WithSurfaceResult(), decad.WithLoftAlignment(off))
		if err != nil {
			t.Fatalf("ruled loft between sections %d and %d: %v", k, k+1, err)
		}
		walls = append(walls, w)
	}
	sheet, err := decad.Stitch(ctx, walls...)
	if err != nil {
		t.Fatalf("stitching %d wall sheets: %v", len(walls), err)
	}
	return sheet
}

// cgLineAt is the drawing-order index of the line at outer-loop position j.
func cgLineAt(t testing.TB, sec cgSection, j int) int {
	t.Helper()
	for i, l := range sec.lines {
		if sec.prof.Outer[j].Entity == sketch.Entity(l) {
			return i
		}
	}
	t.Fatalf("outer edge %d of a section is none of its four lines", j)
	return -1
}

// cgCap is the planar patch on a section, one end of a closed tube.
func cgCap(t testing.TB, doc *decad.Document, sec cgSection) *decad.Body {
	t.Helper()
	b, err := doc.Patch(t.Context(), sec.s, sec.prof)
	if err != nil {
		t.Fatalf("cap patch: %v", err)
	}
	return b
}

// cgClose stitches an open wall sheet and its two end caps into a solid. A
// refusal comes back as the error, for the caller to record; a stitch that
// leaves free edges fails the test, since every cap here is drawn from the
// very section the walls end on.
func cgClose(t testing.TB, doc *decad.Document, walls *decad.Body, first, last cgSection, what string) (*decad.Body, error) {
	t.Helper()
	b, err := decad.Stitch(t.Context(), walls, cgCap(t, doc, first), cgCap(t, doc, last))
	if err != nil {
		return nil, err
	}
	if b.Kind() != decad.BodySolid {
		t.Fatalf("closing %s left a sheet, not a solid", what)
	}
	return b, nil
}

// The ribbon's sections are numbered from its negative end: section K stands
// at s0 + K*P/n. Every piece of a ribbon reads its sections by that one
// number, so two pieces that meet share their meeting section to the bit and
// Stitch welds them; a station summed piece by piece would not.

// cgStation is the station of ribbon section K.
func (g cgGear) cgStation(k int) float64 {
	return g.s0() + float64(k)*g.in.P/float64(g.in.StepsPerTooth())
}

// cgRibbonSections are ribbon sections k0 to k1 of gear g, each on its own
// plane.
func cgRibbonSections(t testing.TB, w *sketch.World, g cgGear, k0, k1 int) []cgSection {
	t.Helper()
	secs := make([]cgSection, 0, k1-k0+1)
	for k := k0; k <= k1; k++ {
		s := g.cgStation(k)
		secs = append(secs, cgNewSection(t, w, g.sectionFrame(t, s), g.cellCorners(s)))
	}
	return secs
}

// cgRibbonPiece is the open wall sheet through ribbon sections k0 to k1.
func cgRibbonPiece(t testing.TB, doc *decad.Document, g cgGear, k0, k1 int) *decad.Body {
	t.Helper()
	return cgLoftWalls(t, doc, cgRibbonSections(t, sketch.NewWorld(), g, k0, k1))
}

// cgCloseRibbon closes an open ribbon sheet that runs from section k0 to k1
// with a cap on each.
func cgCloseRibbon(t testing.TB, doc *decad.Document, g cgGear, walls *decad.Body, k0, k1 int) (*decad.Body, error) {
	t.Helper()
	w := sketch.NewWorld()
	first := cgRibbonSections(t, w, g, k0, k0)[0]
	last := cgRibbonSections(t, w, g, k1, k1)[0]
	return cgClose(t, doc, walls, first, last, fmt.Sprintf("%s sections %d to %d", g.label, k0, k1))
}

// cgClosedRange is the closed body through ribbon sections k0 to k1.
func cgClosedRange(t testing.TB, doc *decad.Document, g cgGear, k0, k1 int) (*decad.Body, error) {
	t.Helper()
	return cgCloseRibbon(t, doc, g, cgRibbonPiece(t, doc, g, k0, k1), k0, k1)
}

// cgRibbonWalls runs the doubling schedule of §3 for q cells and returns the
// open sheet of the q cells and the schedule it ran.
//
// What is substituted. The build copies the body, moves the copy by the screw
// step and joins it. The proof's join is a stitch of wall sheets, and a sheet
// carried there by Body.Placed carries the transform's rounding on its edges,
// so its far end no longer meets a cap drawn from the exact section and the
// ribbon cannot be closed. So each copy is lofted where the move puts it, from
// the ribbon's own sections. stepScrewMove holds that a copy Placed by the
// screw step is those cells built in place; that is what licenses it here.
func cgRibbonWalls(t testing.TB, doc *decad.Document, g cgGear, q int) (*decad.Body, []cgOp) {
	t.Helper()
	per := g.in.Cell() * g.in.StepsPerTooth() // sections a cell spans
	body := cgRibbonPiece(t, doc, g, 0, per)
	ops := cgSchedule(q)
	for _, op := range ops {
		if op.kind == "aside" {
			continue // an unmoved copy of the first op.cells cells, placed below
		}
		from := op.by * per
		piece := cgRibbonPiece(t, doc, g, from, from+op.cells*per)
		var err error
		if body, err = decad.Stitch(t.Context(), body, piece); err != nil {
			t.Fatalf("joining %d cells after %d: %v", op.cells, op.by, err)
		}
	}
	return body, ops
}

// ---------------------------------------------------------------------------
// The volume oracle for a ruled loft.
//
// decad's ruled wall between two sections is two flat triangles per quad,
// split along one diagonal. The two splits of a skew quad differ by the
// tetrahedron its four corners span, and the ruled (bilinear) wall encloses
// the average of the two. So the solid's volume lies within half the sum of
// those tetrahedra of the volume the averaged split encloses, whichever
// diagonal each quad takes. That half-sum is the oracle's error, and it is
// the slack every ruled-loft volume is compared with.

// cgRuledVolume is that average volume and its half-sum, for a tube through
// the sections' world corners, closed by the first and last section.
func cgRuledVolume(secs [][4]r3.Vec) (float64, float64) {
	tri := func(a, b, c r3.Vec) float64 { return a.Dot(b.Cross(c)) / 6 }
	v1, v2, slack := 0.0, 0.0, 0.0
	first, last := secs[0], secs[len(secs)-1]
	// The first cap faces back along the axis, the last forward; the corners
	// run counter-clockwise about +dir.
	capFlux := tri(first[0], first[2], first[1]) + tri(first[0], first[3], first[2]) +
		tri(last[0], last[1], last[2]) + tri(last[0], last[2], last[3])
	v1, v2 = capFlux, capFlux
	for k := 0; k+1 < len(secs); k++ {
		for i := range 4 {
			a, b := secs[k][i], secs[k][(i+1)%4]
			c, d := secs[k+1][(i+1)%4], secs[k+1][i]
			s1 := tri(a, b, c) + tri(a, c, d)
			s2 := tri(a, b, d) + tri(b, c, d)
			v1 += s1
			v2 += s2
			slack += math.Abs(s1-s2) / 2
		}
	}
	return math.Abs(v1+v2) / 2, slack
}

// cgWorldCorners collects sections' world corners.
func cgWorldCorners(secs []cgSection) [][4]r3.Vec {
	out := make([][4]r3.Vec, len(secs))
	for i, s := range secs {
		out[i] = s.world
	}
	return out
}

// cgRibbonCorners are the world corners of ribbon sections k0 to k1, as the
// volume oracle reads them.
func cgRibbonCorners(g cgGear, k0, k1 int) [][4]r3.Vec {
	out := make([][4]r3.Vec, 0, k1-k0+1)
	for k := k0; k <= k1; k++ {
		s := g.cgStation(k)
		f, _ := r3.NewFrame(g.axisPoint(s), g.u, g.v)
		var q [4]r3.Vec
		for i, c := range g.cellCorners(s) {
			q[i] = f.ToWorldUV(c[0], c[1])
		}
		out = append(out, q)
	}
	return out
}
