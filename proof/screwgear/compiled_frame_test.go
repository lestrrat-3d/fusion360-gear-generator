package screwgear_test

// This file and the three compiled_*_test.go files beside it are the COMPILED
// step proof: one function per build step of spec/screwgear/steps.md, run
// through the sketch and decad engines by the registrations generated into
// zz_registrations_test.go. They are not the hand-written mechanism proof in
// geometry_test.go, pair_test.go and cage_test.go, which proves that the two
// parts drive each other 1:1 and move freely in the frame; this proves that
// every sketch the build draws closes fully constrained and unambiguous, and
// that every solid step leaves the body the next step consumes.
//
// The compiled proof reuses the hand-written model where the spec says the
// build does the same thing: Params and Gear for the ribbon's section function,
// newFrame and its rodShift for the rod search the build runs in processInputs,
// axialWindow for the engaged zone, boreSections for the stand-in section count
// and doublingRounds for the doubling schedule. Everything else is derived here
// from the spec's own formulas.
//
// Everything is in millimetres and radians; Fusion's internal units are
// centimetres, and the step list says where the build converts.

import (
	"context"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
	"github.com/lestrrat-3d/units"
)

// The dialog's input ids, which are also the case tables' parameter keys. The
// angles are in degrees in a case table, as the dialog shows them.
const (
	csRibbonWidth     = "ribbonWidth"
	csToothCount      = "toothCount"
	csTwistLead       = "twistLead"
	csRibbonThickness = "ribbonThickness"
	csToothPitch      = "toothPitch"
	csToothHeight     = "toothHeight"
	csRingRadius      = "ringRadius"
	csCageRadius      = "cageRadius"
	csCageRise        = "cageRise"
	csClearance       = "clearance"
	csCollarHalf      = "collarHalf"
	csCollarWall      = "collarWall"
	csRodDiameter     = "rodDiameter"
	csRingWire        = "ringWire"
	csCrossAngle      = "crossAngle"
	csEngagement      = "engagement"
	csMountAngleA     = "mountAngleA"
	csMountAngleB     = "mountAngleB"
	csAssemblyPhase   = "assemblyPhase"

	// csCellTeeth is the module constant CELL_TEETH, not a dialog input. The
	// spec states counts for 4, the constant's value, and for 1, the fallback
	// the first cell measurement was made at, so the tables run both.
	csCellTeeth = "cellTeeth"

	// Per-case selectors a step reads beside the inputs.
	csGearKey  = "gear"  // 0 for gear A, 1 for gear B
	csIndexKey = "index" // collar, rod, bar or ball index
	csMoveKey  = "teeth" // how many teeth a screw move carries the cell
	csBodyKey  = "body"  // teeth the ribbon holds before a join
	csToolKey  = "tool"  // teeth the join's tool holds
	csPlaceKey = "moved" // 1 when the join's tool was moved into place, 0 when built there

	// csExpectSections, when present, is the section count the spec quotes for
	// the case, which the step asserts beside the count it derives.
	csExpectSections = "expectSections"
)

// csDefaults is the spec's default table ("Defaults, and what they were
// measured to do" and the dialog table of "Variables").
func csDefaults() map[string]float64 {
	return map[string]float64{
		csRibbonWidth:     15,
		csToothCount:      68,
		csTwistLead:       49.5,
		csRibbonThickness: 3.75,
		csToothPitch:      2.625,
		csToothHeight:     2.625,
		csRingRadius:      16.875,
		csCageRadius:      15,
		csCageRise:        20.25,
		csClearance:       0.45,
		csCollarHalf:      3,
		csCollarWall:      3,
		csRodDiameter:     3,
		csRingWire:        3.75,
		csCrossAngle:      80,
		csEngagement:      0.75,
		csMountAngleA:     15,
		csMountAngleB:     15,
		csAssemblyPhase:   -1.31,
		csCellTeeth:       4,
	}
}

// csVariant is one input set: the defaults with some fields moved. Every
// variant is an input the build accepts — csCheckInputs holds each to the
// spec's range checks and the rod search — because a step proof of an input
// processInputs refuses proves nothing about a build that never runs.
type csVariant struct {
	name string
	set  map[string]float64
}

// The variants, and the edge each one stands on.
//
// The crossing angle's stated range is (0°, 180°), but at every other default
// the rod checks of processInputs refuse anything outside about 70°–110°: a rod
// that has to turn past the other gear's ribbon ends up outside its own
// collar's wall. cross70 and cross110 are the ends the build accepts. The
// engagement's range ends at the tooth height, which the default cage refuses
// (a collar would reach into the widened engaged zone), so engageFull moves the
// cage out with it. Negative values are exercised where the spec's inputs are
// signed: the assembly phase (its default is negative, and phasePlus and
// phaseMinus stand near +/- one pitch) and the mounting angles, which carry no
// range at all.
var (
	csVDefaults      = csVariant{"defaults", nil}
	csVCellOne       = csVariant{"cellTeeth1", map[string]float64{csCellTeeth: 1}}
	csVLead20        = csVariant{"lead20", map[string]float64{csTwistLead: 20}}
	csVLead400       = csVariant{"lead400", map[string]float64{csTwistLead: 400}}
	csVCross70       = csVariant{"cross70", map[string]float64{csCrossAngle: 70}}
	csVCross110      = csVariant{"cross110", map[string]float64{csCrossAngle: 110}}
	csVEngageFine    = csVariant{"engage0.05", map[string]float64{csEngagement: 0.05}}
	csVEngageFull    = csVariant{"engageFull", map[string]float64{csEngagement: 2.625, csCageRadius: 20, csRingRadius: 22}}
	csVPhasePlus     = csVariant{"phase+2.6", map[string]float64{csAssemblyPhase: 2.6}}
	csVPhaseMinus    = csVariant{"phase-2.6", map[string]float64{csAssemblyPhase: -2.6}}
	csVPhaseZero     = csVariant{"phase0", map[string]float64{csAssemblyPhase: 0}}
	csVMountNeg      = csVariant{"mount-30-15", map[string]float64{csMountAngleA: -30, csMountAngleB: -15}}
	csVMountUnequal  = csVariant{"mount0+30", map[string]float64{csMountAngleA: 0, csMountAngleB: 30}}
	csVMountSteep    = csVariant{"mount80", map[string]float64{csMountAngleA: 80, csMountAngleB: 80}}
	csVToothTall     = csVariant{"tooth7.4", map[string]float64{csToothHeight: 7.4}}
	csVToothShort    = csVariant{"tooth0.2", map[string]float64{csToothHeight: 0.2, csEngagement: 0.1}}
	csVTeeth15       = csVariant{"teeth15", map[string]float64{csToothCount: 15}}
	csVTeeth69       = csVariant{"teeth69", map[string]float64{csToothCount: 69}}
	csVCellOne15     = csVariant{"cellTeeth1-teeth15", map[string]float64{csCellTeeth: 1, csToothCount: 15}}
	csVClearanceFine = csVariant{"clearance0.05", map[string]float64{csClearance: 0.05}}
	csVClearanceWide = csVariant{"clearance0.9", map[string]float64{csClearance: 0.9}}
	csVThinRod       = csVariant{"rod1", map[string]float64{csRodDiameter: 1}}
)

// csParamsOf is a variant's full parameter map, with extra per-case keys.
func csParamsOf(v csVariant, extra map[string]float64) map[string]float64 {
	m := csDefaults()
	for k, x := range v.set {
		m[k] = x
	}
	for k, x := range extra {
		m[k] = x
	}
	return m
}

// csIn is one case's inputs, read the way processInputs reads the dialog.
type csIn struct {
	P         Params
	Phase     float64 // the assemblyPhase input: gear B's tooth phase Z0, mm
	CellTeeth int     // CELL_TEETH
}

func csNeed(t testing.TB, m map[string]float64, key string) float64 {
	t.Helper()
	v, ok := m[key]
	if !ok {
		t.Fatalf("case carries no %q", key)
	}
	return v
}

func csRead(t testing.TB, m map[string]float64) csIn {
	t.Helper()
	deg := func(key string) float64 { return csNeed(t, m, key) * math.Pi / 180 }
	count := csNeed(t, m, csToothCount)
	if count != math.Trunc(count) {
		t.Fatalf("toothCount %v is not a whole number", count)
	}
	cell := csNeed(t, m, csCellTeeth)
	return csIn{
		P: Params{
			Width:       csNeed(t, m, csRibbonWidth),
			Thickness:   csNeed(t, m, csRibbonThickness),
			ToothHeight: csNeed(t, m, csToothHeight),
			ToothPitch:  csNeed(t, m, csToothPitch),
			TwistLead:   csNeed(t, m, csTwistLead),
			CrossAngle:  deg(csCrossAngle),
			MountAngleA: deg(csMountAngleA),
			MountAngleB: deg(csMountAngleB),
			Engagement:  csNeed(t, m, csEngagement),
			ToothCount:  int(count),
			CageRadius:  csNeed(t, m, csCageRadius),
			RingRadius:  csNeed(t, m, csRingRadius),
			CageRise:    csNeed(t, m, csCageRise),
			RingWire:    csNeed(t, m, csRingWire),
			RodDiameter: csNeed(t, m, csRodDiameter),
			CollarHalf:  csNeed(t, m, csCollarHalf),
			CollarWall:  csNeed(t, m, csCollarWall),
			Clearance:   csNeed(t, m, csClearance),
		},
		Phase:     csNeed(t, m, csAssemblyPhase),
		CellTeeth: int(cell),
	}
}

// The world the proof builds in. The spec's frame is read from whatever plane
// and point the user selects, so the proof places it off every world axis: the
// selected plane's normal n̂ is tilted and the centre C is off the origin. A
// sign or a frame mix-up that the world axes would hide shows up here.
var (
	csCentre = r3.NewVec(12, -7, 30)
)

// csAxes is the frame the Anchor sketch reads: ê along the Anchor Line, n̂ the
// selected plane's normal (as the Gear A Axis Plane reads it), k̂ = n̂ × ê.
func csAxes() (e, n, k r3.Vec) {
	n, _ = r3.NewVec(0.2, -0.3, 1).Normalize()
	x := r3.NewVec(1, 0.4, 0)
	e, _ = x.Sub(n.Scale(x.Dot(n))).Normalize()
	return e, n, n.Cross(e)
}

// csRotateAbout turns v by angle about the unit axis a (right-handed).
func csRotateAbout(v, a r3.Vec, angle float64) r3.Vec {
	c, s := math.Cos(angle), math.Sin(angle)
	return v.Scale(c).Add(a.Cross(v).Scale(s)).Add(a.Scale(a.Dot(v) * (1 - c)))
}

// csGears places both ribbons by spec §1: dirA and dirB are ê turned by
// +Sigma/2 and -Sigma/2 about n̂, originA = C - (A/2)n̂ and originB = C + (A/2)n̂,
// û_A = +n̂, û_B = -n̂, v̂ = dir × û. The hand-written Gear carries the section
// function; its Phase is the tooth phase Z0, zero for gear A and the
// assemblyPhase input for gear B.
func csGears(in csIn) [2]Gear {
	p := in.P
	e, n, _ := csAxes()
	a := p.AxisOffset()
	dirA := csRotateAbout(e, n, p.Sigma()/2)
	dirB := csRotateAbout(e, n, -p.Sigma()/2)
	ga := Gear{P: p, Origin: csCentre.Sub(n.Scale(a / 2)), Ex: n, Ez: dirA, Hand: 1,
		Mount: p.MountAngleA, Phase: 0}
	ga.Ey = dirA.Cross(ga.Ex)
	uB := n.Scale(-1)
	gb := Gear{P: p, Origin: csCentre.Add(n.Scale(a / 2)), Ex: uB, Ez: dirB, Hand: 1,
		Mount: p.MountAngleB, Phase: in.Phase}
	gb.Ey = dirB.Cross(gb.Ex)
	return [2]Gear{ga, gb}
}

func csGearLabel(g int) string { return [2]string{"Gear A", "Gear B"}[g] }

// csCanonicalFrame is the hand-written cage model (cage_test.go) for these
// inputs, in its own frame: centre at the origin, n̂ = +Z, ê = +X. Azimuths are
// the same numbers in any frame, so the rod search it runs is the build's.
func csCanonicalFrame(in csIn) frame {
	ga, gb := pair(in.P, in.P.Sigma(), 0, in.Phase)
	return newFrame(ga, gb)
}

// csCheckInputs holds a case to every range check processInputs raises on
// (spec "Variables", "Range checks"), in the order the spec lists them. A
// table case that fails one is a defect in the table, not in the build.
func csCheckInputs(t testing.TB, in csIn) frame {
	t.Helper()
	p := in.P
	for name, v := range map[string]float64{
		csRibbonWidth: p.Width, csRibbonThickness: p.Thickness, csToothPitch: p.ToothPitch,
		csTwistLead: p.TwistLead, csRingWire: p.RingWire, csRodDiameter: p.RodDiameter,
		csCollarHalf: p.CollarHalf, csCollarWall: p.CollarWall, csClearance: p.Clearance,
	} {
		if !(v > 0) {
			t.Fatalf("%s must be > 0, got %v", name, v)
		}
	}
	if p.ToothCount < 4 {
		t.Fatalf("toothCount must be a whole number >= 4, got %d", p.ToothCount)
	}
	if !(p.ToothHeight > 0 && p.ToothHeight < p.Width/2) {
		t.Fatalf("toothHeight must be > 0 and < ribbonWidth/2 = %v, got %v", p.Width/2, p.ToothHeight)
	}
	if !(p.Engagement > 0 && p.Engagement <= p.ToothHeight) {
		t.Fatalf("engagement must be > 0 and <= toothHeight = %v, got %v", p.ToothHeight, p.Engagement)
	}
	if !(p.CrossAngle > 0 && p.CrossAngle < math.Pi) {
		t.Fatalf("crossAngle must lie strictly between 0 and 180 degrees")
	}
	if !(math.Abs(in.Phase) < p.ToothPitch) {
		t.Fatalf("assemblyPhase must lie strictly within +/-toothPitch = %v, got %v", p.ToothPitch, in.Phase)
	}
	if zone := axialWindow(p); !(p.CageRadius-p.CollarHalf > zone) {
		t.Fatalf("cageRadius - collarHalf = %v must exceed the engaged zone's half-length %v",
			p.CageRadius-p.CollarHalf, zone)
	}
	if !(p.CageRadius+p.CollarHalf+1 < p.Length()/2) {
		t.Fatalf("cageRadius + collarHalf + 1 mm = %v must be under toothCount*toothPitch/2 = %v",
			p.CageRadius+p.CollarHalf+1, p.Length()/2)
	}
	f := csCanonicalFrame(in)
	names := [4]string{"gear A -R", "gear A +R", "gear B -R", "gear B +R"}
	for i, c := range f.cross {
		if math.IsNaN(c.rod) {
			t.Fatalf("ringRadius: no rod on the ring's circle clears both ribbons for the collar %s", names[i])
		}
		depth, along := f.rodWallDepth(c)
		if !(depth > 0 && depth <= p.CollarWall) {
			t.Fatalf("ringRadius: the rod for the collar %s stands %v mm outside its bore, outside (0, collarWall = %v]",
				names[i], depth, p.CollarWall)
		}
		if math.Abs(along-c.station)+p.RodDiameter/2 > p.CollarHalf {
			t.Fatalf("ringRadius: the rod for the collar %s misses the collar's length", names[i])
		}
	}
	if d := f.nearestRods(); d < p.RodDiameter+p.Clearance {
		t.Fatalf("ringRadius: two rods stand %v mm apart, under rodDiameter + clearance", d)
	}
	a := p.AxisOffset()
	if left := p.CageRise - p.RingWire/2 - (a/2 + math.Hypot(p.Width/2, p.Thickness/2)); left < p.Clearance {
		t.Fatalf("cageRise leaves %v mm between the frame and the ribbons, under the clearance %v", left, p.Clearance)
	}
	if !(p.CollarWall >= p.RodDiameter) {
		t.Fatalf("collarWall %v must be at least rodDiameter %v", p.CollarWall, p.RodDiameter)
	}
	if !(in.CellTeeth == 4 || in.CellTeeth == 1) {
		t.Fatalf("cellTeeth %d is neither the module constant 4 nor its measured fallback 1", in.CellTeeth)
	}
	return f
}

// csStepsPerTooth is n of spec §2: the larger of the twist count, which keeps
// neighbouring sections under 2 degrees apart, and the floor of eight. It is
// the hand-written LoftSections less the closing section.
func csStepsPerTooth(p Params) int { return p.LoftSections() - 1 }

// csCellCount is c = min(cellTeeth, N), with q whole cells and r teeth left.
func csCellCount(in csIn) (c, q, r int) {
	c = min(in.CellTeeth, in.P.ToothCount)
	return c, in.P.ToothCount / c, in.P.ToothCount % c
}

// csStartStation is s0 = Z0 - L/2, the ribbon's negative end.
func csStartStation(g Gear) float64 { return g.Phase - g.P.Length()/2 }

// csSectionCorners is the cross-section at station s, spec §2: the corners
// (uB,-hv), (uF,-hv), (uF,hv), (uB,hv) with uB = -W/2, uF = Utooth(s), hv = T/2,
// turned by theta = s/Lambda + Phi and placed at origin + s*dir. It is wound
// counter-clockwise about +dir.
func csSectionCorners(g Gear, s float64) []r3.Vec {
	uB, uF, hv := -g.P.Width/2, g.edge(s), g.P.Thickness/2
	return []r3.Vec{g.world(uB, -hv, s), g.world(uF, -hv, s), g.world(uF, hv, s), g.world(uB, hv, s)}
}

// csStations are the stations of a cell of `teeth` teeth starting at `from`:
// teeth*n + 1 of them, P/n apart.
func csStations(p Params, from float64, teeth int) []float64 {
	n := csStepsPerTooth(p)
	out := make([]float64, teeth*n+1)
	for k := range out {
		out[k] = from + float64(k)*p.ToothPitch/float64(n)
	}
	return out
}

// csSectionFrame is the plane square to gear g's axis at station s, with its
// sketch axes along the gear's unrotated û and v̂, so its normal is +dir.
func csSectionFrame(t testing.TB, g Gear, s float64) r3.Frame {
	t.Helper()
	return csFrame(t, g.Origin.Add(g.Ez.Scale(s)), g.Ex, g.Ey)
}

func csFrame(t testing.TB, o, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(o, u, v)
	if err != nil {
		t.Fatalf("frame at %v: %v", o, err)
	}
	return f
}

// csLocal is modelToSketchSpace: a world point in a plane's own coordinates,
// with the height off the plane measured. Every point the spec means to lie on
// its plane must come back at zero height; the step list sets Fusion's z to 0
// on each ([PB-SKETCH-ZERO-Z]), and this is the check that the point it zeroes
// really was on the plane in the build's own arithmetic.
func csLocal(t testing.TB, f r3.Frame, w r3.Vec, what string) (float64, float64) {
	t.Helper()
	l := f.ToLocal(w)
	// The frame arithmetic rounds at about 1e-15 of the coordinates, which
	// stand within a few hundred millimetres of the origin.
	sketchtest.Measures(t, what+" height above its sketch plane", l.Z, 0, sketchtest.Within(1e-9))
	return l.X, l.Y
}

// csFootAzimuths are the rods' azimuths about +n̂ from ê, in the collar order
// of the rod search (gear A -R, gear A +R, gear B -R, gear B +R), from the same
// search the build runs.
func csFootAzimuths(f frame) [4]float64 {
	var out [4]float64
	for i, c := range f.cross {
		out[i] = c.rod
	}
	return out
}

// csFootOrder is the loop's foot order: foot 0 the rod with the smallest
// azimuth in [0, 2*pi), the rest increasing. It returns rod indices.
func csFootOrder(az [4]float64) [4]int {
	order := []int{0, 1, 2, 3}
	sort.Slice(order, func(a, b int) bool { return wrap(az[order[a]]) < wrap(az[order[b]]) })
	return [4]int{order[0], order[1], order[2], order[3]}
}

// csFoot is a rod's foot at height h along n̂: C + h*n̂ + R*(cos psi ê + sin psi k̂).
func csFoot(p Params, psi, h float64) r3.Vec {
	e, n, k := csAxes()
	return csCentre.Add(n.Scale(h)).Add(e.Scale(p.RingRadius * math.Cos(psi))).Add(k.Scale(p.RingRadius * math.Sin(psi)))
}

// csCollarStation is where collar i sits on its gear's axis, -R or +R, and
// the gear it belongs to, in the rod search's collar order.
func csCollarStation(p Params, i int) (gear int, sc float64) {
	return i / 2, [2]float64{-p.CageRadius, p.CageRadius}[i%2]
}

// ---- sketch-engine helpers -------------------------------------------------

// csSolve solves a step's sketch so the step can read its solved geometry; the
// harness solves and gates it again afterwards.
func csSolve(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	res := sketchtest.Solve(t, s)
	if !res.Converged {
		t.Fatalf("solve did not converge: residual %.3e", res.Residual)
	}
}

// csSatisfied holds every committed constraint of a solved sketch to zero
// residual. The solver's tolerance is far under a nanometre on these sketches.
func csSatisfied(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	for _, c := range s.Constraints() {
		sketchtest.Satisfies(t, c, sketchtest.Within(1e-7))
	}
}

// csProfileKinds counts the lines and arcs on a profile's outer boundary.
func csProfileKinds(p *sketch.Profile) (lines, arcs, other int) {
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.Line:
			lines++
		case *sketch.Arc:
			arcs++
		default:
			other++
		}
	}
	return lines, arcs, other
}

// csOneProfile verifies a solved sketch, takes its one profile and holds it
// valid, current and exactly cut, with the stated line and arc counts: the
// counts the build's find_profile_by_curve_counts or profiles.count check keys on.
func csOneProfile(t testing.TB, s *sketch.Sketch, lines, arcs int) *sketch.Profile {
	t.Helper()
	rep := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, rep)
	sketchtest.IsValidProfile(t, prof)
	sketchtest.IsCurrentProfile(t, prof)
	sketchtest.HasExactCuts(t, prof)
	l, a, o := csProfileKinds(prof)
	if l != lines || a != arcs || o != 0 {
		t.Fatalf("the profile has %d lines, %d arcs and %d other curves; the build keys on %d lines and %d arcs",
			l, a, o, lines, arcs)
	}
	return prof
}

// ---- decad helpers ---------------------------------------------------------

// csPlaneSketch is an empty sketch on a world plane with the given frame, for
// a solid step: decadtest.NewSketch draws only on XY, and every sketch of this
// build stands on a plane of the spec's own frame.
func csPlaneSketch(t testing.TB, f r3.Frame) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	if err != nil {
		t.Fatalf("create plane: %v", err)
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

// csPolygon draws one closed polygon of fixed points on a sketch and returns
// its lines in drawing order.
func csPolygon(t testing.TB, s *sketch.Sketch, f r3.Frame, pts []r3.Vec, what string) []*sketch.Line {
	t.Helper()
	ps := make([]*sketch.Point, len(pts))
	for i, w := range pts {
		x, y := csLocal(t, f, w, what)
		ps[i] = s.CreatePoint(x, y)
	}
	lines := make([]*sketch.Line, len(ps))
	for i := range ps {
		lines[i] = s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	for _, p := range ps {
		s.Fix(p)
	}
	return lines
}

// csPolygonRegion is a polygon on its own plane, solved to its one region.
func csPolygonRegion(t testing.TB, f r3.Frame, pts []r3.Vec, what string) (*sketch.Sketch, *sketch.Profile, []*sketch.Line) {
	t.Helper()
	s := csPlaneSketch(t, f)
	lines := csPolygon(t, s, f, pts, what)
	return s, decadtest.SolveRegion(t, s), lines
}

// csRuledLoft is the proof's stand-in for a loft through many sections: decad
// lofts two sections at a time, so each neighbouring pair is lofted as a sheet
// with flat walls, the two end sections are patched, and the whole is stitched
// into one solid. Section i+1's segment that pairs with section i's first one
// is found by drawing order rather than assumed, since the walk a profile
// starts from is the sketch engine's choice.
func csRuledLoft(t testing.TB, doc *decad.Document, frames []r3.Frame, sections [][]r3.Vec, what string) *decad.Body {
	t.Helper()
	ctx := context.Background()
	type sec struct {
		s     *sketch.Sketch
		p     *sketch.Profile
		lines []*sketch.Line
	}
	secs := make([]sec, len(sections))
	for i := range sections {
		s, p, lines := csPolygonRegion(t, frames[i], sections[i], what)
		secs[i] = sec{s, p, lines}
	}
	first := func(x sec) int {
		for j, l := range x.lines {
			if x.p.Outer[0].Entity == sketch.Entity(l) {
				return j
			}
		}
		t.Fatalf("%s: a section's boundary starts on no line it drew", what)
		return 0
	}
	var sheets []*decad.Body
	for i := 0; i+1 < len(secs); i++ {
		a, b := secs[i], secs[i+1]
		// The recorded segment index of b that pairs with a's segment 0.
		off := (first(a) - first(b) + len(b.lines)) % len(b.lines)
		sheet, err := doc.Loft(ctx, a.s, a.p, b.s, b.p, decad.WithSurfaceResult(), decad.WithLoftAlignment(off))
		if err != nil {
			t.Fatalf("%s: loft between sections %d and %d: %v", what, i, i+1, err)
		}
		sheets = append(sheets, sheet)
	}
	for _, i := range []int{0, len(secs) - 1} {
		cap, err := doc.Patch(ctx, secs[i].s, secs[i].p)
		if err != nil {
			t.Fatalf("%s: cap at section %d: %v", what, i, err)
		}
		sheets = append(sheets, cap)
	}
	body, err := decad.Stitch(ctx, sheets...)
	if err != nil {
		t.Fatalf("%s: stitching %d sections into one solid: %v", what, len(sections), err)
	}
	if body.Kind() != decad.BodySolid {
		t.Fatalf("%s: the stitched loft is not closed into a solid", what)
	}
	return body
}

// csRuledVolume brackets the volume of a loft whose walls between neighbouring
// sections are flat triangle pairs, which is what decad builds. Each wall quad
// is split along one of its two diagonals, and the two splits enclose different
// volumes; the loft's volume lies between the sum of the smaller and the sum of
// the larger. The caps are planar and exact. Sections must be wound
// counter-clockwise about the direction they advance in.
//
// The bracket is the proof's own oracle. Its width is the price of the flat
// walls: at the ribbon's 1.9-degree step a long wall's corners move 0.25 mm out
// of its plane, and the two splits of that quad differ by a sliver.
func csRuledVolume(sections [][]r3.Vec) (want, slack float64) {
	o := sections[0][0]
	tet := func(a, b, c r3.Vec) float64 {
		a, b, c = a.Sub(o), b.Sub(o), c.Sub(o)
		return a.Dot(b.Cross(c)) / 6
	}
	fan := func(ps []r3.Vec, reverse bool) float64 {
		v := 0.0
		for j := 1; j+1 < len(ps); j++ {
			if reverse {
				v += tet(ps[0], ps[j+1], ps[j])
			} else {
				v += tet(ps[0], ps[j], ps[j+1])
			}
		}
		return v
	}
	lo := fan(sections[0], true) + fan(sections[len(sections)-1], false)
	hi := lo
	for k := 0; k+1 < len(sections); k++ {
		pk, qk := sections[k], sections[k+1]
		m := len(pk)
		for i := range m {
			p0, p1 := pk[i], pk[(i+1)%m]
			q0, q1 := qk[i], qk[(i+1)%m]
			a := tet(p0, p1, q1) + tet(p0, q1, q0)
			b := tet(p0, p1, q0) + tet(p1, q1, q0)
			lo += math.Min(a, b)
			hi += math.Max(a, b)
		}
	}
	want = (lo + hi) / 2
	// Float64 summation over a few thousand tetrahedra of a few hundred mm³.
	return want, (hi-lo)/2 + 1e-9*math.Abs(want)
}

// csMeasuresVolume holds a named body's volume reading to the proof's oracle.
func csMeasuresVolume(t testing.TB, what string, body *decad.Body, want, slack float64) {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	decadtest.Measures(t, what+" volume", v, units.CubicMillimeters(want), decadtest.Within(units.CubicMillimeters(slack)))
}

// csHasVertex holds that some vertex of a body lies on a point: the build's
// end-face checks read body.vertices the same way. The vertex readings carry
// their own bound; the slack covers the section arithmetic.
func csHasVertex(t testing.TB, what string, body *decad.Body, at r3.Vec) {
	t.Helper()
	best, bestD := (*decad.Vertex)(nil), math.Inf(1)
	for _, v := range body.Vertices() {
		if d := v.Position().Value.Sub(at).Len(); d < bestD {
			best, bestD = v, d
		}
	}
	if best == nil {
		t.Fatalf("%s: the body has no vertices", what)
	}
	decadtest.MeasuresVec(t, what, best.Position(), at, decadtest.Within(units.Millimeters(1e-9)))
}

// csScrewStep is Step(k) of spec §3 about gear g's axis: a turn of k*P/Lambda
// about +dir through the axis, then k*P along +dir. The two commute because the
// translation runs along the rotation axis ([SCREW-F-SCREW-STEP]).
func csScrewStep(t testing.TB, g Gear, k int) r3.Transform {
	t.Helper()
	d := float64(k) * g.P.ToothPitch
	rot, err := r3.RotationAround(g.Origin, g.Ez, units.Radians(d/g.lambda()))
	if err != nil {
		t.Fatalf("screw step rotation: %v", err)
	}
	shift, err := r3.Translation(g.Ez.Scale(d))
	if err != nil {
		t.Fatalf("screw step translation: %v", err)
	}
	step, err := rot.Then(shift)
	if err != nil {
		t.Fatalf("screw step: %v", err)
	}
	return step
}
