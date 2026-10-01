package screwgear_test

// The compiled step proof's sketch steps. Each step function draws one Fusion
// sketch of the build into the case's sketch, as the step list states it, and
// proofkit gates it on the engine's full verification verdict.
//
// What these sketches cannot show is Fusion's own frame: the sketch engine
// draws on an exact plane, so the z = 0 rule ([PB-SKETCH-ZERO-Z]) has no
// counterpart here, and every sketch is drawn in its plane's own coordinates
// (the Anchor and Sleeve sketches on the selected plane with C at the origin,
// a Paths sketch on its axis plane with C's foot at the origin, a section on
// its station's plane with the axis point at the origin and x along û, y along
// v̂, a window on the Window Plane with x along `across` and y along n̂).
// Fusion's `isFixed` is the engine's Fix, applied after the curves that use a
// point, as the step list orders it; a projection is a reference point.

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// cpGeomTol is the slack for a reading of a fixed point or of a solved
// position the scheme pins exactly: the solver's own convergence, 1e-9 mm.
const cpGeomTol = 1e-9

// --- Anchor --------------------------------------------------------------

// cpAnchorCases places the selected point at the plane's origin and away from
// it: the scheme must close wherever the user's point sits on the plane.
var cpAnchorCases = []proofkit.Case{
	{Name: "centre at the plane origin", Params: map[string]float64{"anchorX": 0, "anchorY": 0}},
	{Name: "centre off the plane origin", Params: map[string]float64{"anchorX": 37.5, "anchorY": -12.25}},
	{Name: "centre at negative coordinates", Params: map[string]float64{"anchorX": -120, "anchorY": -45}},
}

// stepAnchorSketch is the Anchor sketch of §1: the projected centre point, the
// 10 mm Anchor Line seeded either side of it, midpoint (which in the engine
// carries the coincident row Fusion's addCoincident adds), horizontal, and a
// signed horizontal distance from start to end that fixes the line's sense.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	cx, cy := p["anchorX"], p["anchorY"]
	proofkit.Step(t, "project the selected point")
	centre := s.CreateReferencePoint(cx, cy, "selected point")
	proofkit.Step(t, "draw the Anchor Line from raw seeds 5 mm either side")
	start := s.CreatePoint(cx-5, cy)
	end := s.CreatePoint(cx+5, cy)
	line := s.CreateLine(start, end)
	proofkit.Step(t, "constrain it: midpoint, horizontal, horizontal distance 10 mm")
	mid := sketch.NewMidpoint(centre, line)
	hor := sketch.NewHorizontal(line)
	dist := sketch.NewHorizontalDistance(start, end, 10)
	s.AddConstraint(mid, hor, dist)

	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, start, cx-5, cy, sketchtest.Within(cpGeomTol))
	sketchtest.MeasuresPoint(t, end, cx+5, cy, sketchtest.Within(cpGeomTol))
	sketchtest.Measures(t, "Anchor Line length", line.Length(), 10, sketchtest.Within(cpGeomTol))
	for _, c := range s.Constraints() {
		sketchtest.Satisfies(t, c, sketchtest.Within(cpGeomTol))
	}
}

// --- Paths ---------------------------------------------------------------

// cpSleeveCases is the parameter regime the sleeve steps hold across: the
// defaults and inputs the spec's TestSleeveWindowsFollowTheSize accepts at
// the ends of the dialog's ranges, each of which passes the build's checks.
var cpSleeveCases = []proofkit.Case{
	{Name: "defaults", Params: cpWith(nil)},
	{Name: "cage radius 25", Params: cpWith(map[string]float64{"cageRadius": 25})},
	{Name: "cage radius 14", Params: cpWith(map[string]float64{"cageRadius": 14})},
	{Name: "collar half 2", Params: cpWith(map[string]float64{"collarHalf": 2})},
	{Name: "collar half 4", Params: cpWith(map[string]float64{"collarHalf": 4})},
	{Name: "clearance 0.2", Params: cpWith(map[string]float64{"clearance": 0.2})},
	{Name: "clearance 0.9", Params: cpWith(map[string]float64{"clearance": 0.9})},
	{Name: "crossing 70", Params: cpWith(map[string]float64{"crossAngle": 70})},
	{Name: "crossing 120", Params: cpWith(map[string]float64{"crossAngle": 120})},
	{Name: "width 10", Params: cpWith(map[string]float64{"ribbonWidth": 10})},
	{Name: "thickness 5", Params: cpWith(map[string]float64{"ribbonThickness": 5})},
	{Name: "lead 40", Params: cpWith(map[string]float64{"twistLead": 40})},
	{Name: "lead 60", Params: cpWith(map[string]float64{"twistLead": 60})},
	{Name: "rise 25", Params: cpWith(map[string]float64{"cageRise": 25})},
}

// cpPerGear doubles a case table, one row per gear.
func cpPerGear(cases []proofkit.Case) []proofkit.Case {
	var out []proofkit.Case
	for _, c := range cases {
		for g, label := range []string{"gear A", "gear B"} {
			p := cpWith(c.Params)
			p["gear"] = float64(g)
			out = append(out, proofkit.Case{Name: c.Name + ", " + label, Params: p})
		}
	}
	return out
}

var cpPathsCases = cpPerGear(cpSleeveCases)

// stepPathsSketch is a gear's Paths sketch of §1: four reference points on the
// gear's axis at stations -sOut, -sIn, sIn and sOut, the solid line bore-
// from -sOut to -sIn and bore+ from sIn to sOut, each drawn from its negative
// station to its positive one, then every point fixed. The axis plane is
// parallel to the selected plane, so its coordinates are the frame's ê and k̂
// with C's foot at the origin.
func stepPathsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	at := func(st float64) (float64, float64) {
		w := g.Origin.Add(g.Dir.Scale(st))
		return w.X, w.Y
	}
	proofkit.Step(t, "%s Paths: four reference points", g.Label)
	stations := []float64{-m.SOut, -m.SIn, m.SIn, m.SOut}
	var pts []*sketch.Point
	for _, st := range stations {
		x, y := at(st)
		pts = append(pts, s.CreatePoint(x, y))
	}
	proofkit.Step(t, "%s Paths: lines bore- and bore+", g.Label)
	boreMinus := s.CreateLine(pts[0], pts[1])
	borePlus := s.CreateLine(pts[2], pts[3])
	for _, pt := range pts {
		s.Fix(pt)
	}

	sketchtest.Solve(t, s)
	for i, pt := range pts {
		x, y := at(stations[i])
		sketchtest.MeasuresPoint(t, pt, x, y, sketchtest.Within(cpGeomTol))
	}
	span := m.SOut - m.SIn
	sketchtest.Measures(t, "bore- length", boreMinus.Length(), span, sketchtest.Within(cpGeomTol))
	sketchtest.Measures(t, "bore+ length", borePlus.Length(), span, sketchtest.Within(cpGeomTol))
	// Each line runs along +dir_g from its start: the start is its negative end.
	for name, l := range map[string]*sketch.Line{"bore-": boreMinus, "bore+": borePlus} {
		gl := l.Geometry()
		dx, dy := gl.End.X-gl.Start.X, gl.End.Y-gl.Start.Y
		sketchtest.Measures(t, name+" runs along +dir", (dx*g.Dir.X+dy*g.Dir.Y)/span, 1, sketchtest.Within(1e-12))
	}
	if n := len(s.Profiles()); n != 0 {
		t.Fatalf("%s Paths sketch holds %d profiles; two lines on one axis bound none", g.Label, n)
	}
}

// --- Cell Sections and Cell Remainder ------------------------------------

// cpDrawQuad draws one section as four solid lines sharing their corners, the
// corners fixed after the last line, and returns the corner points.
func cpDrawQuad(s *sketch.Sketch, q cpQuad) []*sketch.Point {
	var pts []*sketch.Point
	for _, c := range q.Corners {
		pts = append(pts, s.CreatePoint(c[0], c[1]))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%4])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	return pts
}

// cpRibbonSketchCases is the regime the ribbon's sketches and solids hold
// across: both gears, the leads that set the section count from either of its
// two bounds (a 400 mm lead takes the floor of eight steps, a 20 mm lead the
// 2° twist bound at 24 steps), a tooth as deep as the width allows, the
// smallest tooth count, and the signed inputs at both signs.
var cpRibbonSketchCases = cpPerGear([]proofkit.Case{
	{Name: "defaults", Params: cpWith(nil)},
	{Name: "lead 400, eight-step floor", Params: cpWith(map[string]float64{"twistLead": 400})},
	{Name: "lead 20, twist bound", Params: cpWith(map[string]float64{"twistLead": 20})},
	{Name: "tooth height near W/2", Params: cpWith(map[string]float64{"toothHeight": 7.4})},
	{Name: "four teeth", Params: cpWith(map[string]float64{"toothCount": 4})},
	{Name: "mount -15, phase +2.6", Params: cpWith(map[string]float64{"mountAngleA": -15, "mountAngleB": -15, "assemblyPhase": 2.6})},
	{Name: "mount 30, phase -2.6", Params: cpWith(map[string]float64{"mountAngleA": 30, "mountAngleB": 0, "assemblyPhase": -2.6})},
	{Name: "width 10, thickness 2.5", Params: cpWith(map[string]float64{"ribbonWidth": 10, "ribbonThickness": 2.5})},
})

var cpCellSectionCases = cpRibbonSketchCases

// cpRemainderCases are tooth counts that leave a remainder in four-tooth cells.
var cpRemainderCases = cpPerGear([]proofkit.Case{
	{Name: "69 teeth, remainder 1", Params: cpWith(map[string]float64{"toothCount": 69})},
	{Name: "70 teeth, remainder 2", Params: cpWith(map[string]float64{"toothCount": 70})},
	{Name: "71 teeth, remainder 3, lead 400", Params: cpWith(map[string]float64{"toothCount": 71, "twistLead": 400})},
	{Name: "5 teeth, remainder 1, lead 20", Params: cpWith(map[string]float64{"toothCount": 5, "twistLead": 20})},
})

// cpSectionsSketch is the stand-in for one 3D Cell Sections sketch. Fusion
// draws every section into one sketch on the gear's Axis Plane with each
// corner kept at its own height ([SCREW-F-CELL-LOFT]); the sketch engine is
// planar, so the stand-in draws each section on its own station's plane: the
// first in the case's sketch, whose plane stands for the first station's
// plane (x along û, y along v̂, the axis point at the origin), and each later
// one in a sketch of its own on the plane offset along the axis to its
// station, gated on the engine's verdict here. It pins every section's
// numbers; Fusion's verdict on the one 3D sketch — fully constrained, one
// profile per section — is the measurement of 2026-09-28 and is not
// reproduced.
func cpSectionsSketch(t testing.TB, s *sketch.Sketch, m cpModel, g cpGear, secs []cpQuad, label string) {
	if want := len(secs); want < 2 {
		t.Fatalf("%s: %d sections; a loft needs at least two", label, want)
	}
	s0 := secs[0].S
	w := s.World()
	for k, q := range secs {
		proofkit.Step(t, "%s section %d at station %.4f", label, k, q.S)
		sk := s
		if k > 0 {
			pl, err := w.CreateOffsetPlane(s.Plane(), q.S-s0)
			if err != nil {
				t.Fatalf("%s section %d plane: %v", label, k, err)
			}
			sk, err = w.CreateSketch(pl)
			if err != nil {
				t.Fatalf("%s section %d sketch: %v", label, k, err)
			}
		}
		pts := cpDrawQuad(sk, q)
		sketchtest.Solve(t, sk)
		for i, pt := range pts {
			want := r3.NewVec(q.Corners[i][0], q.Corners[i][1], q.S-s0)
			sketchtest.MeasuresWorldPoint(t, pt, want, sketchtest.Within(cpGeomTol))
		}
		if k == 0 {
			continue // the harness gates the case's own sketch
		}
		rep := sketchtest.IsTrustworthy(t, sk)
		prof := sketchtest.SingleProfile(t, rep)
		sketchtest.IsValidProfile(t, prof)
		uF := m.utooth(q.S, g.Z0)
		sketchtest.MeasuresProfileArea(t, prof, (uF+m.W/2)*m.T, sketchtest.WithinRel(1e-9))
	}
	rep := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, rep)
	sketchtest.IsValidProfile(t, prof)
	uF := m.utooth(secs[0].S, g.Z0)
	sketchtest.MeasuresProfileArea(t, prof, (uF+m.W/2)*m.T, sketchtest.WithinRel(1e-9))
}

// stepCellSectionsSketch is a gear's Cell Sections sketch of §2: c*n + 1
// rectangles at stations s0 + k*P/n, the toothed side at Utooth(s_k), turned
// by s_k/Lambda + Phi.
func stepCellSectionsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	secs := m.ribbonSections(g, 0, m.Cell)
	if want := m.Cell*m.Steps + 1; len(secs) != want {
		t.Fatalf("%s Cell Sections: %d sections, want c*n + 1 = %d", g.Label, len(secs), want)
	}
	cpCheckSectionCount(t, m, p)
	cpSectionsSketch(t, s, m, g, secs, g.Label+" Cell Sections")
}

// cpCheckSectionCount holds the section count of §2 to its two bounds: the
// twist between neighbours under 2°, and at least eight steps to the tooth.
// At the defaults it is 10 steps and 41 sections in the four-tooth cell.
func cpCheckSectionCount(t testing.TB, m cpModel, p map[string]float64) {
	dtheta := m.P / m.Lam / float64(m.Steps)
	if dtheta > 2*math.Pi/180+1e-12 {
		t.Fatalf("twist per step %.4f° exceeds 2°", dtheta*180/math.Pi)
	}
	if m.Steps < 8 {
		t.Fatalf("%d steps to the tooth, under the floor of eight", m.Steps)
	}
	if m.Steps > 8 && (m.P/m.Lam)/float64(m.Steps-1) <= 2*math.Pi/180 {
		t.Fatalf("%d steps is not the least that keeps the twist under 2°", m.Steps)
	}
	if p["twistLead"] == cpDefaults["twistLead"] && p["toothPitch"] == cpDefaults["toothPitch"] {
		sketchtest.Measures(t, "steps to the tooth at the default lead", float64(m.Steps), 10, sketchtest.Within(0))
	}
}

// stepCellRemainderSketch is a gear's Cell Remainder sketch of §3: the last r
// teeth, r*n + 1 sections at stations from s0 + q*c*P to s0 + N*P, by the
// recipe of §2 with c replaced by r.
func stepCellRemainderSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	if m.R == 0 {
		t.Fatalf("case %v leaves no remainder; the step does not run", p["toothCount"])
	}
	secs := m.ribbonSections(g, m.Q*m.Cell, m.R)
	if want := m.R*m.Steps + 1; len(secs) != want {
		t.Fatalf("%s Cell Remainder: %d sections, want r*n + 1 = %d", g.Label, len(secs), want)
	}
	end := m.ribbonStart(g) + float64(m.N)*m.P
	sketchtest.Measures(t, "remainder's last station", secs[len(secs)-1].S, end, sketchtest.Within(1e-9))
	cpSectionsSketch(t, s, m, g, secs, g.Label+" Cell Remainder")
}

// --- Sleeve --------------------------------------------------------------

var cpSleeveSketchCases = cpSleeveCases

// stepSleeveSketch is the Sleeve sketch of §4: two circles about C of radii
// Ri and Ro, each centre its own point fixed at C (two points at one place,
// no coincident between them) and each circle carrying a diameter dimension.
// Its two profiles are the inner disc and the ring; the ring is the one with
// a hole.
func stepSleeveSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	proofkit.Step(t, "inner circle, radius Ri = %.4f", m.Ri)
	ci := s.CreatePoint(0, 0)
	inner := s.CreateCircle(ci, m.Ri)
	s.Fix(ci)
	s.AddConstraint(sketch.NewDiameter(inner, 2*m.Ri))
	proofkit.Step(t, "outer circle, radius Ro = %.4f", m.Ro)
	co := s.CreatePoint(0, 0)
	outer := s.CreateCircle(co, m.Ro)
	s.Fix(co)
	s.AddConstraint(sketch.NewDiameter(outer, 2*m.Ro))

	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "inner radius", inner.R(), m.Ri, sketchtest.Within(cpGeomTol))
	sketchtest.Measures(t, "outer radius", outer.R(), m.Ro, sketchtest.Within(cpGeomTol))
	rep := sketchtest.Verify(t, s)
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("Sleeve sketch has %d profiles, want 2 (disc and ring)", len(profiles))
	}
	var ring *sketch.Profile
	for _, pr := range profiles {
		if len(pr.Holes) == 1 {
			if ring != nil {
				t.Fatalf("Sleeve sketch has more than one profile with two loops")
			}
			ring = pr
		}
	}
	if ring == nil {
		t.Fatalf("Sleeve sketch has no profile with two loops")
	}
	_ = rep
	sketchtest.IsValidProfile(t, ring)
	sketchtest.IsCurrentProfile(t, ring)
	sketchtest.MeasuresProfileArea(t, ring, math.Pi*(m.Ro*m.Ro-m.Ri*m.Ri), sketchtest.WithinRel(1e-9))
}

// --- Bore sections -------------------------------------------------------

// cpPerBore expands a case table to the four bores in the build's order.
func cpPerBore(cases []proofkit.Case) []proofkit.Case {
	var out []proofkit.Case
	for _, c := range cases {
		for _, b := range []struct {
			gear, sign float64
			label      string
		}{{0, -1, "Gear A Bore -R"}, {0, 1, "Gear A Bore +R"}, {1, -1, "Gear B Bore -R"}, {1, 1, "Gear B Bore +R"}} {
			p := cpWith(c.Params)
			p["gear"], p["bore"] = b.gear, b.sign
			out = append(out, proofkit.Case{Name: c.Name + ", " + b.label, Params: p})
		}
	}
	return out
}

// cpBoreSectionCases reaches both references of the angle: at the defaults
// all four bores take the spine K (|sin theta| >= sqrt(1/2)); a 0° mounting
// angle turns the -R bores to the toothed side L2, -15° the +R bores too, and
// a 20 mm lead turns the +R bore past 135°.
var cpBoreSectionCases = cpPerBore(append(append([]proofkit.Case{}, cpSleeveCases...),
	proofkit.Case{Name: "mount 0", Params: cpWith(map[string]float64{"mountAngleA": 0, "mountAngleB": 0})},
	proofkit.Case{Name: "mount -15", Params: cpWith(map[string]float64{"mountAngleA": -15, "mountAngleB": -15})},
	proofkit.Case{Name: "mount 30 and 0", Params: cpWith(map[string]float64{"mountAngleA": 30, "mountAngleB": 0})},
	proofkit.Case{Name: "lead 20", Params: cpWith(map[string]float64{"twistLead": 20})},
))

// cpBoreUsesSpine reports which reference the angle is taken against.
func cpBoreUsesSpine(theta float64) bool { return math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) }

// stepBoreSectionSketch is one bore section sketch of §4, the rectangle
// scheme: reference points O (the axis point) and Cp (A/2 along the unrotated
// û), the construction line Ru from O to Cp and the construction spine K from
// O to E; both references fixed; the rectangle L1..L4 sharing its corners;
// then the distance O–E of uF, the angle, L1 and L3 parallel to K at hv each
// side, E on L2, L2 perpendicular to K, and L4 parallel to L2 at uF - uB.
//
// Fusion's addParallel plus addOffsetDimension is the engine's signed
// NewOffset, which carries both rows and the side: Fusion takes the side from
// the seed ([PB-DIM-VALUE-SEMANTICS]), and the engine's probe would refuse an
// unsigned offset as ambiguous, since L4 could stand either side of L2. The
// angle is the engine's signed angle from Ru's direction to K's (or L2's),
// which is what Fusion's text point inside the wedge selects
// ([PB-ANGULAR-DIM]).
func stepBoreSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	g := m.gear(int(p["gear"]))
	sign := int(p["bore"])
	s0, _ := m.boreSpan(sign)
	theta := m.theta(g, s0)
	uB, uF, hv := -m.Hw, m.Hw, m.Ht
	q := m.quad(g, s0, uB, uF, hv)

	proofkit.Step(t, "references O and Cp, Ru and the spine K")
	o := s.CreatePoint(0, 0)
	cp := s.CreatePoint(m.A/2, 0)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	ex, ey := cpRot(uF, 0, theta)
	e := s.CreatePoint(ex, ey)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	s.Fix(o)
	s.Fix(cp)

	proofkit.Step(t, "the rectangle L1..L4 sharing its corners")
	var c [4]*sketch.Point
	for i := range c {
		c[i] = s.CreatePoint(q.Corners[i][0], q.Corners[i][1])
	}
	l1 := s.CreateLine(c[0], c[1])
	l2 := s.CreateLine(c[1], c[2])
	l3 := s.CreateLine(c[2], c[3])
	l4 := s.CreateLine(c[3], c[0])
	_ = l3

	proofkit.Step(t, "dimensions and constraints of the rectangle scheme")
	s.AddConstraint(sketch.NewDistance(o, e, uF))
	if cpBoreUsesSpine(theta) {
		s.AddConstraint(sketch.NewAngle(ru, k, theta*180/math.Pi))
	} else {
		s.AddConstraint(sketch.NewAngle(ru, l2, theta*180/math.Pi+90))
	}
	s.AddConstraint(
		sketch.NewOffset(k, l1, -hv),
		sketch.NewOffset(k, l3, hv),
		sketch.NewPointOnLine(e, l2),
		sketch.NewPerpendicular(l2, k),
		sketch.NewOffset(l2, l4, uF-uB),
	)

	sketchtest.Solve(t, s)
	for i := range c {
		sketchtest.MeasuresPoint(t, c[i], q.Corners[i][0], q.Corners[i][1], sketchtest.Within(1e-7))
	}
	sketchtest.MeasuresPoint(t, e, ex, ey, sketchtest.Within(1e-7))
	sketchtest.Measures(t, "L1 length (the bore's width)", l1.Length(), uF-uB, sketchtest.Within(1e-7))
	sketchtest.Measures(t, "L2 length (the bore's thickness)", l2.Length(), 2*hv, sketchtest.Within(1e-7))
	sketchtest.Measures(t, "L4 length", l4.Length(), 2*hv, sketchtest.Within(1e-7))
	for _, con := range s.Constraints() {
		sketchtest.Satisfies(t, con, sketchtest.Within(1e-7))
	}
	rep := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, rep)
	sketchtest.IsValidProfile(t, prof)
	sketchtest.HasExactCuts(t, prof)
	if len(prof.Entities) != 4 {
		t.Fatalf("bore profile has %d boundary entities, want the four lines", len(prof.Entities))
	}
	sketchtest.MeasuresProfileArea(t, prof, 4*m.Hw*m.Ht, sketchtest.WithinRel(1e-7))
}

// --- Windows -------------------------------------------------------------

// cpWindowCases are the two windows the search settles at the defaults. The
// window search runs in processInputs and the compiled proof takes its
// results as numbers; the spec quotes them for the defaults alone, so the
// windows facing ±ê past a 90° crossing, and the windows at any other input,
// have no case here (the hand-written TestSleeveWindowsFollowTheSize holds
// them).
var cpWindowCases = []proofkit.Case{
	{Name: "defaults, window +k", Params: cpWith(map[string]float64{"window": 0})},
	{Name: "defaults, window -k", Params: cpWith(map[string]float64{"window": 1})},
}

// stepWindowSketch is a Window sketch of §4: one reference point per hexagon
// corner, a solid line from each corner to the next, every point fixed.
func stepWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := cpModelOf(p)
	win := cpDefaultWindows()[int(p["window"])]
	corners := win.corners(m.Ro)
	if len(corners) != 6 {
		t.Fatalf("window %s has %d corners, want 6", win.Name, len(corners))
	}
	proofkit.Step(t, "Window %s: six corners and six lines", win.Name)
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
	sketchtest.Solve(t, s)
	rep := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, rep)
	sketchtest.IsValidProfile(t, prof)
	// The spec quotes the area as 208.1 mm² from the search's own numbers; the
	// six numbers here are quoted to 0.001 mm, which moves the area by at most
	// the perimeter times 0.0005 mm, about 0.04 mm², plus the 0.05 of the
	// quoted figure's rounding.
	sketchtest.MeasuresProfileArea(t, prof, 208.1, sketchtest.Within(0.1))
	sketchtest.Measures(t, fmt.Sprintf("window %s area against its corners", win.Name),
		prof.Area, math.Abs(cpPolygonArea(corners)), sketchtest.WithinRel(1e-9))
}
