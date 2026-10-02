package screwgear_test

// The sketch steps of spec/screwgear/steps.md, each proved in the sketch
// engine through proofkit.Run, which gates the result on the engine's own
// verification report: DOF 0, no conflicting or redundant constraint, valid
// profiles, a well-conditioned system and no discrete ambiguity.
//
// Fusion → engine mapping, as [PB-SKETCH-FIRST] gives it. A reference point
// the build adds with sketchPoints.add and pins with isFixed is a point the
// engine grounds with Fix; the one projected point, the Anchor sketch's centre,
// is a reference point (CreateReferencePoint), since a projection is locked to
// its source. A Fusion dimension is a magnitude whose side the seed decides
// ([PB-DIM-VALUE-SEMANTICS]); the engine's signed dimensions carry that side
// in their sign, so where the build seeds a point on one side the proof writes
// the signed value of that side.
//
// Every sketch is drawn in its own plane's coordinates. Fusion picks a
// sketch's x and y axes itself and the build maps world points in with
// modelToSketchSpace; the proof draws the same points in a frame of its own
// choosing on the same plane, named at each step. Nothing a step constrains
// depends on which in-plane frame it is drawn in, except a signed angle,
// which the build does not write: its angular dimension is unsigned and its
// text point picks the wedge ([PB-ANGULAR-DIM]).

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// ---------------------------------------------------------------------------
// Anchor sketch (§1).

// anchorCases puts the projected centre at the sketch origin and away from it
// on both sides, since the line is seeded from wherever the projection lands.
var anchorCases = []proofkit.Case{
	{Name: "centre at origin", Params: sgWith(map[string]float64{"cx": 0, "cy": 0})},
	{Name: "centre off origin", Params: sgWith(map[string]float64{"cx": 37.5, "cy": -12.25})},
	{Name: "centre negative", Params: sgWith(map[string]float64{"cx": -140, "cy": 63})},
}

// stepAnchorSketch is the Anchor sketch: the projected centre, and the Anchor
// Line through it, held by midpoint, horizontal and a horizontal start-to-end
// distance of 10 mm.
//
// The build writes addCoincident and addMidPoint together, as the bevel gear
// does; the engine's midpoint already carries the point-on-line row, so the
// proof writes the midpoint alone, as proof/bevelgear does. A Fusion
// horizontal distance from start to end is a magnitude whose side is the
// seed's (end to the right of start); the engine's horizontal distance is
// signed, +10 for that side, and a negative value would be the mirrored line
// the aligned-length form could not tell apart.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	cx, cy := p["cx"], p["cy"]
	proofkit.Step(t, "projected centre at (%.3f, %.3f)", cx, cy)
	centre := s.CreateReferencePoint(cx, cy, "selected point")
	proofkit.Step(t, "Anchor Line from seeds 5 mm either side along x")
	start := s.CreatePoint(cx-5, cy)
	end := s.CreatePoint(cx+5, cy)
	line := s.CreateLine(start, end)
	mid := sketch.NewMidpoint(centre, line)
	hor := sketch.NewHorizontal(line)
	dist := sketch.NewHorizontalDistance(start, end, 10)
	s.AddConstraint(mid, hor, dist)

	sketchtest.Solve(t, s)
	for _, c := range s.Constraints() {
		sketchtest.Satisfies(t, c, sketchtest.Within(1e-9))
	}
	// The frame the build reads from this sketch: C is the centre, ê runs from
	// the line's start to its end. Both are exact here, the seeds being the
	// solved positions ([PB-SEED-NEAR]); the 1e-9 mm is the solver's tolerance.
	sketchtest.MeasuresPoint(t, start, cx-5, cy, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, end, cx+5, cy, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "Anchor Line length", line.Length(), 10, sketchtest.Within(1e-9))
}

// ---------------------------------------------------------------------------
// Gear Paths sketch (§1).

// pathsCases covers both gears, whose axes turn opposite ways about n̂, the
// ends of the crossing angle's open range, the roof allowance at zero, and a
// cage radius just past the least the "channel starts in the hollow" check
// accepts, where sIn is barely positive.
var pathsCases = func() []proofkit.Case {
	var out []proofkit.Case
	for _, g := range []float64{0, 1} {
		name := map[float64]string{0: "Gear A", 1: "Gear B"}[g]
		out = append(out,
			proofkit.Case{Name: name + " defaults", Params: sgWith(map[string]float64{"gear": g})},
			proofkit.Case{Name: name + " crossing 5 deg", Params: sgWith(map[string]float64{"gear": g, "crossAngle": 5})},
			proofkit.Case{Name: name + " crossing 175 deg", Params: sgWith(map[string]float64{"gear": g, "crossAngle": 175})},
			proofkit.Case{Name: name + " no roof allowance", Params: sgWith(map[string]float64{"gear": g, "roofAllowance": 0})},
			// hypot(c, 1 mm) < Ri is the check; c = 8.058 at the defaults, so a
			// cage radius of 3 + hypot(8.058, 1) + 0.05 puts sIn at about 0.05 mm.
			proofkit.Case{Name: name + " sIn barely positive", Params: sgWith(map[string]float64{
				"gear": g, "cageRadius": 3 + math.Hypot(math.Hypot(7.7, 2.375), 1) + 0.05})},
		)
	}
	return out
}()

// stepPathsSketch is a gear's Paths sketch on its Axis Plane: four reference
// points on the gear's axis at stations -sOut, -sIn, sIn and sOut, and two
// solid lines, bore- from -sOut to -sIn and bore+ from sIn to sOut, each
// sharing its two points, then every point fixed.
//
// The proof draws the Axis Plane in coordinates x along ê and y along k̂,
// with the gear's axis point origin_g at the sketch origin; the plane holds
// the gear's axis, which runs at +Sigma/2 (gear A) or -Sigma/2 (gear B)
// from ê.
func stepPathsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	if !(m.sIn > 0) {
		t.Fatalf("%s: sIn = %.4f mm; the channel-starts-in-the-hollow check refuses this input", g.label, m.sIn)
	}
	ex, ey := g.dir.X, g.dir.Y
	stations := [4]float64{-m.sOut, -m.sIn, m.sIn, m.sOut}
	var pts [4]*sketch.Point
	proofkit.Step(t, "%s Paths: reference points at stations %v", g.label, stations)
	for i, st := range stations {
		pts[i] = s.CreatePoint(st*ex, st*ey)
	}
	boreMinus := s.CreateLine(pts[0], pts[1])
	borePlus := s.CreateLine(pts[2], pts[3])
	for _, pt := range pts {
		s.Fix(pt)
	}

	sketchtest.Solve(t, s)
	for i, st := range stations {
		sketchtest.MeasuresPoint(t, pts[i], st*ex, st*ey, sketchtest.Within(1e-9))
	}
	// Each line runs from its negative station to its positive one, along
	// +dir_g, so a plane at fraction 0 stands at its negative end.
	for _, l := range []*sketch.Line{boreMinus, borePlus} {
		sketchtest.Measures(t, g.label+" bore line length", l.Length(), m.sOut-m.sIn, sketchtest.WithinRel(1e-12))
	}
	if got := (borePlus.Start.Geometry().X)*ex + (borePlus.Start.Geometry().Y)*ey; math.Abs(got-m.sIn) > 1e-9 {
		t.Fatalf("%s bore+ starts at station %.6f, want sIn %.6f", g.label, got, m.sIn)
	}
	if got := (boreMinus.Start.Geometry().X)*ex + (boreMinus.Start.Geometry().Y)*ey; math.Abs(got+m.sOut) > 1e-9 {
		t.Fatalf("%s bore- starts at station %.6f, want -sOut %.6f", g.label, got, -m.sOut)
	}
}

// ---------------------------------------------------------------------------
// Cell Sections sketch (§2), and the Remainder sections (§3).
//
// STAND-IN. Fusion's Cell Sections sketch is ONE sketch on the gear's Axis
// Plane whose fixed points keep the z modelToSketchSpace gives them, so its
// sections stand off the plane at their own stations. The sketch engine is
// planar and cannot hold a point off its plane. The stand-in is one planar
// sketch per section, on that station's own plane (x along û_g, y along v̂_g,
// normal +dir_g), holding the section's four corners as fixed points and the
// four lines L1..L4 sharing them. It pins every section's numbers and that
// each closes one valid profile; it cannot see Fusion's verdict on the one 3D
// sketch — fully constrained, one profile per section, measured on 2026-09-28
// at 11, 41 and 81 sections — which is the step's [PROSE] part.

// cellSectionCases runs every section of the cell at each parameter set: the
// defaults on both gears; the slowest twist the floor of eight governs
// (400 mm lead, n = 8) and a fast one (20 mm, n = 24); a negative mounting
// angle; gear B at both ends of the assembly phase's open range; the thinnest
// cell (four teeth, one cell, no rounds); a tooth as tall as the check
// allows, just under W/2; and a narrow, thick ribbon.
var cellSectionCases = func() []proofkit.Case {
	sets := []struct {
		name string
		over map[string]float64
	}{
		{"Gear A defaults", map[string]float64{"gear": 0}},
		{"Gear B defaults", map[string]float64{"gear": 1}},
		{"Gear A lead 400", map[string]float64{"gear": 0, "twistLead": 400}},
		{"Gear A lead 20", map[string]float64{"gear": 0, "twistLead": 20}},
		{"Gear A mount -25", map[string]float64{"gear": 0, "mountAngleA": -25}},
		{"Gear B phase +2.6", map[string]float64{"gear": 1, "assemblyPhase": 2.6}},
		{"Gear B phase -2.6", map[string]float64{"gear": 1, "assemblyPhase": -2.6}},
		{"Gear A four teeth", map[string]float64{"gear": 0, "toothCount": 4}},
		{"Gear B tooth 7.4", map[string]float64{"gear": 1, "toothHeight": 7.4}},
		{"Gear A narrow thick", map[string]float64{"gear": 0, "ribbonWidth": 10, "ribbonThickness": 5}},
	}
	var out []proofkit.Case
	for _, set := range sets {
		m := newModel(sgWith(set.over))
		for k := 0; k <= m.c*m.n; k++ {
			over := map[string]float64{"section": float64(k)}
			for key, v := range set.over {
				over[key] = v
			}
			out = append(out, proofkit.Case{Name: fmt.Sprintf("%s section %d of %d", set.name, k, m.c*m.n+1), Params: sgWith(over)})
		}
	}
	return out
}()

// stepCellSectionSketch draws section k of the gear's tooth cell: stations
// s_k = s0 + k*P/n from s0 = Z0 - L/2, the rectangle u from -W/2 to
// Utooth(s_k), v from -T/2 to T/2, turned by theta_k = s_k/Lambda + Phi.
func stepCellSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	drawSection(t, s, m, g, m.cellStart(g), int(p["section"]))
}

// remainderSectionCases covers each remainder a four-tooth cell leaves, one,
// two and three teeth, on both gears, and gear B at a shifted phase.
var remainderSectionCases = func() []proofkit.Case {
	sets := []struct {
		name string
		over map[string]float64
	}{
		{"Gear A 69 teeth", map[string]float64{"gear": 0, "toothCount": 69}},
		{"Gear B 69 teeth", map[string]float64{"gear": 1, "toothCount": 69}},
		{"Gear A 6 teeth", map[string]float64{"gear": 0, "toothCount": 6}},
		{"Gear B 7 teeth phase 2.6", map[string]float64{"gear": 1, "toothCount": 7, "assemblyPhase": 2.6}},
	}
	var out []proofkit.Case
	for _, set := range sets {
		m := newModel(sgWith(set.over))
		for k := 0; k <= m.r*m.n; k++ {
			over := map[string]float64{"section": float64(k)}
			for key, v := range set.over {
				over[key] = v
			}
			out = append(out, proofkit.Case{Name: fmt.Sprintf("%s remainder section %d of %d", set.name, k, m.r*m.n+1), Params: sgWith(over)})
		}
	}
	return out
}()

// stepRemainderSectionSketch draws section k of the remainder cell: the
// recipe of the Cell Sections sketch with c replaced by r = N mod c, at
// stations from s0 + q*c*P to s0 + N*P.
func stepRemainderSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	g := m.gears[int(p["gear"])]
	if m.r == 0 {
		t.Fatalf("%d teeth leave no remainder in %d-tooth cells", m.N, m.c)
	}
	drawSection(t, s, m, g, m.cellStart(g)+float64(m.q*m.c)*m.P, int(p["section"]))
}

func drawSection(t testing.TB, s *sketch.Sketch, m sgModel, g sgGear, from float64, k int) {
	st, xy := m.cellCorners(g, from, k)
	proofkit.Step(t, "%s section %d at station %.4f, theta %.4f rad", g.label, k, st, g.theta(st))
	var pts [4]*sketch.Point
	for i, c := range xy {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range 4 {
		s.CreateLine(pts[i], pts[(i+1)%4]) // L1..L4, sharing the corners
	}
	for _, pt := range pts {
		s.Fix(pt) // after the last line, as the build sets isFixed
	}
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, prof)
	// The rectangle is (Utooth(s_k) + W/2) by T; the shoelace area of four
	// exact corners carries only rounding, 1e-12 relative.
	want := (g.toothed(st) + m.W/2) * m.T
	sketchtest.MeasuresProfileArea(t, prof, want, sketchtest.WithinRel(1e-9))
	// The toothed edge is a pure cosine: crest W/2, root W/2 - H.
	if u := g.toothed(st); u > m.W/2+1e-12 || u < m.W/2-m.H-1e-12 {
		t.Fatalf("Utooth(%.4f) = %.6f outside [W/2 - H, W/2]", st, u)
	}
}

// ---------------------------------------------------------------------------
// Sleeve sketch (§4).

// sleeveSketchCases covers the defaults, a thin wall and a thick one, and a
// centre away from the sketch origin.
var sleeveSketchCases = []proofkit.Case{
	{Name: "defaults", Params: sgWith(map[string]float64{"cx": 0, "cy": 0})},
	{Name: "centre off origin", Params: sgWith(map[string]float64{"cx": -40, "cy": 22.5})},
	{Name: "collar half 2", Params: sgWith(map[string]float64{"cx": 0, "cy": 0, "collarHalf": 2})},
	{Name: "cage radius 25", Params: sgWith(map[string]float64{"cx": 3, "cy": 4, "cageRadius": 25, "collarHalf": 3.5})},
}

// stepSleeveSketch is the Sleeve sketch on the selected plane: two circles
// about C of radii Ri and Ro, each centre a separate point fixed at C, each
// with a diameter dimension. The ring is the one profile with two loops.
func stepSleeveSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	cx, cy := p["cx"], p["cy"]
	proofkit.Step(t, "Sleeve: circles of radius %.3f and %.3f about (%.3f, %.3f)", m.Ri, m.Ro, cx, cy)
	ci := s.CreatePoint(cx, cy)
	co := s.CreatePoint(cx, cy)
	inner := s.CreateCircle(ci, m.Ri)
	outer := s.CreateCircle(co, m.Ro)
	s.Fix(ci)
	s.Fix(co)
	s.AddConstraint(sketch.NewDiameter(inner, 2*m.Ri), sketch.NewDiameter(outer, 2*m.Ro))
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	_ = report
	var ring *sketch.Profile
	count := 0
	profiles := s.Profiles()
	for _, pr := range profiles {
		if len(pr.Holes) == 1 {
			ring = pr
			count++
		}
	}
	if len(profiles) != 2 || count != 1 {
		t.Fatalf("Sleeve: %d profiles, %d with two loops; want 2 and 1", len(profiles), count)
	}
	sketchtest.IsValidProfile(t, ring)
	sketchtest.IsCurrentProfile(t, ring)
	// π(Ro² - Ri²); the engine's circle area is exact to rounding.
	sketchtest.MeasuresProfileArea(t, ring, math.Pi*(m.Ro*m.Ro-m.Ri*m.Ri), sketchtest.WithinRel(1e-9))
}

// ---------------------------------------------------------------------------
// Bore section sketch (§4, "The rectangle scheme").

// boreSketchCases runs all four bores at each parameter set. The defaults
// reach only the spine branch of the angle (|sin theta| >= sqrt(1/2) at every
// profile station); mounting angles of -57° and -40° put gear A's +R profile
// and gear B's -R profile on the toothed-side branch, -20° makes gear A's +R
// bore the level one with its roof on the -v face, and a zero roof allowance
// leaves every rectangle symmetric about its axis. A 0.05 mm clearance is the
// smallest the stand-in cost was derived for.
var boreSketchCases = func() []proofkit.Case {
	sets := []struct {
		name string
		over map[string]float64
	}{
		{"defaults", nil},
		{"mount -57 and -40", map[string]float64{"mountAngleA": -57, "mountAngleB": -40}},
		{"mount -20 and 30", map[string]float64{"mountAngleA": -20, "mountAngleB": 30}},
		{"mount 90 and 170", map[string]float64{"mountAngleA": 90, "mountAngleB": 170}},
		{"no roof allowance", map[string]float64{"roofAllowance": 0}},
		{"clearance 0.05", map[string]float64{"clearance": 0.05}},
		{"crossing 100", map[string]float64{"crossAngle": 100}},
	}
	var out []proofkit.Case
	for _, set := range sets {
		m := newModel(sgWith(set.over))
		bores := m.bores()
		for i, b := range bores {
			over := map[string]float64{"bore": float64(i)}
			for key, v := range set.over {
				over[key] = v
			}
			g := m.gears[b.gear]
			branch := "spine"
			if math.Abs(math.Sin(g.theta(b.from))) < math.Sqrt(0.5) {
				branch = "toothed side"
			}
			out = append(out, proofkit.Case{
				Name:   fmt.Sprintf("%s %s (%s angle)", set.name, b.name, branch),
				Params: sgWith(over),
			})
		}
	}
	return out
}()

// stepBoreSectionSketch draws one bore's section sketch by the rectangle
// scheme, on the bore's plane at the cut's first station s0 (-sOut for a -R
// bore, sIn for a +R bore), in coordinates x along û_g and y along v̂_g with
// the axis point O at the origin.
//
//   - References: O at (0, 0) and Cp at (A/2, 0), fixed after Ru and K exist;
//     Ru is the construction line O→Cp.
//   - Spine: construction line K from O to E, E seeded at (uF, 0) turned by
//     theta, with a distance O–E of uF.
//   - Angle: between Ru and K when |sin theta| >= sqrt(1/2), else between Ru
//     and the toothed side L2.
//   - Rectangle: L1 (uB, vLo)→(uF, vLo), L2 on to (uF, vHi), L3 on to
//     (uB, vHi), L4 back, sharing corners; L1 offset from K by -vLo (the right
//     of K), L3 by vHi (its left); E on L2; L2 perpendicular to K; L4 offset
//     from L2 by uF - uB.
//
// Mapping. Fusion's addParallel + addOffsetDimension is the engine's NewOffset,
// which carries both rows; its value is signed, positive on the left of the
// source line's start→end direction. Fusion's angular dimension is unsigned
// and its text point picks the wedge; the engine's NewAngle is signed,
// counter-clockwise from the first line's start→end direction to the
// second's, so the proof writes the signed angle the seeds already make, and
// the unsigned value the build writes is its magnitude, which lies in
// 45°..135° on either branch.
func stepBoreSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	b := m.bores()[int(p["bore"])]
	g := m.gears[b.gear]
	s0 := b.from
	th := g.theta(s0)
	uB, uF := -m.hw, m.hw
	proofkit.Step(t, "%s: plane at station %.4f, theta %.4f rad, v from %.4f to %.4f", b.name, s0, th, b.vLo, b.vHi)

	// References.
	o := s.CreatePoint(0, 0)
	cp := s.CreatePoint(m.A/2, 0)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)

	// Spine.
	ex, ey := sgTurn(uF, 0, th)
	e := s.CreatePoint(ex, ey)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	s.Fix(o)
	s.Fix(cp)

	// Rectangle.
	corner := func(u, v float64) *sketch.Point {
		x, y := sgTurn(u, v, th)
		return s.CreatePoint(x, y)
	}
	c1, c2, c3, c4 := corner(uB, b.vLo), corner(uF, b.vLo), corner(uF, b.vHi), corner(uB, b.vHi)
	l1 := s.CreateLine(c1, c2)
	l2 := s.CreateLine(c2, c3)
	l3 := s.CreateLine(c3, c4)
	l4 := s.CreateLine(c4, c1)

	length := sketch.NewDistance(o, e, uF)
	// The angle, by branch ([PB-ANGULAR-DIM]). sgSigned folds an angle into
	// (-180°, 180°].
	var angle *sketch.Angle
	var unsigned float64
	if math.Abs(math.Sin(th)) >= math.Sqrt(0.5) {
		a := sgSigned(th)
		angle = sketch.NewAngle(ru, k, a*180/math.Pi)
		unsigned = math.Abs(a)
		proofkit.Step(t, "%s: angle Ru→K, %.4f°", b.name, unsigned*180/math.Pi)
	} else {
		a := sgSigned(th + math.Pi/2)
		angle = sketch.NewAngle(ru, l2, a*180/math.Pi)
		unsigned = math.Abs(a)
		proofkit.Step(t, "%s: angle Ru→L2, %.4f°", b.name, unsigned*180/math.Pi)
	}
	if unsigned < sgRad(45)-1e-12 || unsigned > sgRad(135)+1e-12 {
		t.Fatalf("%s: the dimensioned angle %.4f° lies outside 45°..135°", b.name, unsigned*180/math.Pi)
	}
	s.AddConstraint(
		length, angle,
		sketch.NewOffset(k, l1, b.vLo), // -vLo to the right of K
		sketch.NewOffset(k, l3, b.vHi), // vHi to the left of K
		sketch.NewPointOnLine(e, l2),
		sketch.NewPerpendicular(l2, k),
		sketch.NewOffset(l2, l4, uF-uB), // L4 on the left of L2's start→end
	)

	sketchtest.Solve(t, s)
	for _, c := range s.Constraints() {
		sketchtest.Satisfies(t, c, sketchtest.Within(1e-9))
	}
	for i, want := range [4][2]float64{{uB, b.vLo}, {uF, b.vLo}, {uF, b.vHi}, {uB, b.vHi}} {
		x, y := sgTurn(want[0], want[1], th)
		sketchtest.MeasuresPoint(t, []*sketch.Point{c1, c2, c3, c4}[i], x, y, sketchtest.Within(1e-9))
	}
	report := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, prof)
	if len(prof.Entities) != 4 {
		t.Fatalf("%s: the profile is bounded by %d curves; find_profile_by_curve_counts(lines=4) needs 4", b.name, len(prof.Entities))
	}
	// 2*hw by (vHi - vLo): 15.4 by 4.15 mm, 4.45 mm on a level bore.
	sketchtest.MeasuresProfileArea(t, prof, 2*m.hw*(b.vHi-b.vLo), sketchtest.WithinRel(1e-9))
}

// sgSigned folds an angle into (-pi, pi].
func sgSigned(a float64) float64 {
	a = math.Mod(a, 2*math.Pi)
	if a > math.Pi {
		a -= 2 * math.Pi
	} else if a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// ---------------------------------------------------------------------------
// Window sketches (§4).

// windowSketchCases is the two default windows, the only ones whose numbers
// the spec states (see sgWindow).
var windowSketchCases = []proofkit.Case{
	{Name: "Window +k", Params: sgWith(map[string]float64{"window": 0})},
	{Name: "Window -k", Params: sgWith(map[string]float64{"window": 1})},
}

// stepWindowSketch is one Window sketch on the Window Plane: a reference
// point per hexagon corner at C + t*across + z*n̂, a solid line from each to
// the next sharing the points, every point fixed. The proof draws the plane
// in (t, z), whose normal across × n̂ is the facing direction d.
func stepWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := newModel(p)
	w := sgDefaultWindows()[int(p["window"])]
	q := w.corners(m.Ro)
	proofkit.Step(t, "%s: %d corners %v", w.name, len(q), q)
	if len(q) != 6 {
		t.Fatalf("%s: %d corners, the spec's default window has six", w.name, len(q))
	}
	pts := make([]*sketch.Point, len(q))
	for i, c := range q {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, prof)
	// The spec quotes the area to 0.1 mm² from search numbers it quotes to
	// 0.001 mm, so 0.05 mm² covers its rounding; the shoelace area of the
	// corners themselves is exact to rounding.
	sketchtest.MeasuresProfileArea(t, prof, w.area, sketchtest.Within(0.05))
	sketchtest.MeasuresProfileArea(t, prof, sgPolygonArea(q), sketchtest.WithinRel(1e-9))
	// Every edge is upright or at 45° on the plane (§4, "The hexagon").
	for i := range q {
		dt, dz := q[(i+1)%len(q)][0]-q[i][0], q[(i+1)%len(q)][1]-q[i][1]
		if !(math.Abs(dt) < 1e-9 || math.Abs(math.Abs(dt)-math.Abs(dz)) < 1e-9) {
			t.Fatalf("%s: edge %d (%.4f, %.4f) is neither upright nor at 45°", w.name, i, dt, dz)
		}
	}
}
