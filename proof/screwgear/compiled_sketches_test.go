package screwgear_test

// The sketch steps of the compiled step list. Each build draws one Fusion
// sketch's scheme into the harness's sketch, which stands on the world XY
// datum; the build maps the scheme's own plane coordinates onto it, so the
// constraint verdict is the scheme's and the world placement is the model's
// (compiled_model_test.go), asserted separately where a step pins it.

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// sgCase is a named parameter case.
func sgCase(name string, p map[string]float64) proofkit.Case {
	return proofkit.Case{Name: name, Params: p}
}

// --- Anchor sketch -----------------------------------------------------------

var anchorCases = []proofkit.Case{
	sgCase("defaults", sgDefaults()),
	sgCase("centre at the origin", sgWith(map[string]float64{"centreX": 0, "centreY": 0})),
	sgCase("centre far off", sgWith(map[string]float64{"centreX": -120.5, "centreY": 87.25})),
}

// stepAnchorSketch is the Anchor sketch of §1: the selected point projected
// in, and a 10 mm Anchor Line bisected by it, horizontal, with a horizontal
// distance from its start to its end.
//
// The projection is a reference point, which the engine never moves, as
// Fusion's projected point is driven by its source. Fusion's coincident and
// midpoint are both written in the step list; the engine's midpoint carries
// the coincident row itself, so the proof writes the midpoint alone. The
// horizontal distance is signed here, as the engine's is; in Fusion its sign
// is the seed's side, start left of end.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	proofkit.Step(t, "project the selected point")
	centre := s.CreateReferencePoint(m.Centre.X, m.Centre.Y, "selected point")
	proofkit.Step(t, "draw the Anchor Line from seeds 5 mm either side")
	start := s.CreatePoint(m.Centre.X-5, m.Centre.Y)
	end := s.CreatePoint(m.Centre.X+5, m.Centre.Y)
	line := s.CreateLine(start, end)
	proofkit.Step(t, "midpoint, horizontal, horizontal distance")
	s.AddConstraint(
		sketch.NewMidpoint(centre, line),
		sketch.NewHorizontal(line),
		sketch.NewHorizontalDistance(start, end, 10),
	)
	sketchtest.Solve(t, s)
	// The seeds are the solved positions, so nothing moves: the readings below
	// are exact up to the solver's tolerance.
	sketchtest.MeasuresPoint(t, start, m.Centre.X-5, m.Centre.Y, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, end, m.Centre.X+5, m.Centre.Y, sketchtest.Within(1e-9))
	// ê runs from the line's start to its end: +X, the frame's ê.
	sketchtest.Measures(t, "ê along X", (end.X()-start.X())/line.Length(), 1, sketchtest.Within(1e-12))
	sketchtest.Measures(t, "ê across Y", (end.Y()-start.Y())/line.Length(), 0, sketchtest.Within(1e-12))
	for _, c := range s.Constraints() {
		sketchtest.Satisfies(t, c, sketchtest.Within(1e-9))
	}
}

// --- Paths sketch ------------------------------------------------------------

var pathsCases = []proofkit.Case{
	sgCase("gear A defaults", sgWith(map[string]float64{"gear": 0})),
	sgCase("gear B defaults", sgWith(map[string]float64{"gear": 1})),
	sgCase("gear A at 110 degrees", sgWith(map[string]float64{"gear": 0, "crossAngle": 110})),
	sgCase("gear B thick wall", sgWith(map[string]float64{"gear": 1, "collarHalf": 3.25, "cageRadius": 17})),
	sgCase("gear A no roof allowance", sgWith(map[string]float64{"gear": 0, "roofAllowance": 0})),
}

// The Axis Planes (S06, S07) are not built: the harness hands each sketch
// step a sketch on the world XY datum, and a plane offset from the selected
// one carries nothing the engine can check but its offset, which this step
// checks as each point's height, and the sign of Fusion's normal, which is
// Fusion's and is checked at build time ([SCREW-F-NORMAL-SIGN]).
//
// stepPathsSketch is a gear's Paths sketch of §1 on its Axis Plane: four
// points on the gear's axis at -sOut, -sIn, sIn and sOut, the bore- and bore+
// lines between them, each from its negative station to its positive one, and
// then all four points fixed. The Axis Plane is parallel to the selected
// plane, so a point's plane coordinates are its world X and Y; its world Z,
// the plane's offset, is asserted to be the plane's.
func stepPathsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	stations := []float64{-m.SOut, -m.SIn, m.SIn, m.SOut}
	if sgIsDefaults(p) {
		// The defaults' stations: 7.892 mm and 19 mm ("The cage").
		sketchtest.Measures(t, "sIn", m.SIn, 7.892, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "sOut", m.SOut, 19, sketchtest.Within(1e-12))
	}
	if !(m.SIn > 0 && m.SIn < m.SOut) {
		t.Fatalf("sIn %.4f mm must lie in (0, sOut %.4f mm)", m.SIn, m.SOut)
	}
	proofkit.Step(t, "reference points on the gear's axis")
	var pts []*sketch.Point
	for _, st := range stations {
		w := m.Origin[g].Add(m.Dir[g].Scale(st))
		sketchtest.Measures(t, "point on the Axis Plane", w.Sub(m.Centre).Dot(m.Nx), m.Origin[g].Sub(m.Centre).Dot(m.Nx), sketchtest.Within(1e-12))
		pts = append(pts, s.CreatePoint(w.X, w.Y))
	}
	proofkit.Step(t, "bore- and bore+ lines")
	lines := []*sketch.Line{s.CreateLine(pts[0], pts[1]), s.CreateLine(pts[2], pts[3])}
	proofkit.Step(t, "fix the four points after the last line")
	for _, q := range pts {
		s.Fix(q)
	}
	sketchtest.Solve(t, s)
	for i, l := range lines {
		name := []string{"bore-", "bore+"}[i]
		sketchtest.Measures(t, name+" length", l.Length(), m.SOut-m.SIn, sketchtest.Within(1e-9))
		// Each line runs along +dir_g: its start is its negative end.
		dx, dy := l.End.X()-l.Start.X(), l.End.Y()-l.Start.Y()
		sketchtest.Measures(t, name+" along dir", (dx*m.Dir[g].X+dy*m.Dir[g].Y)/l.Length(), 1, sketchtest.Within(1e-12))
	}
	// The two lines leave the hollow's middle open between them, 2*sIn long.
	sketchtest.Measures(t, "gap between the lines", pts[1].DistanceTo(pts[2]), 2*m.SIn, sketchtest.Within(1e-9))
}

// --- Cell Sections sketch ----------------------------------------------------

var cellSectionCases = []proofkit.Case{
	sgCase("gear A first section", sgWith(map[string]float64{"gear": 0, "section": 0})),
	sgCase("gear A middle section", sgWith(map[string]float64{"gear": 0, "section": 17})),
	sgCase("gear A last section", sgWith(map[string]float64{"gear": 0, "section": 40})),
	sgCase("gear B first section", sgWith(map[string]float64{"gear": 1, "section": 0})),
	sgCase("gear B last section", sgWith(map[string]float64{"gear": 1, "section": 40})),
	sgCase("negative slant", sgWith(map[string]float64{"gear": 0, "section": 5, "toothSlant": -25.8})),
	sgCase("straight ridge", sgThirdPrint(map[string]float64{"gear": 1, "section": 7})),
	sgCase("steep slant, no bow", sgWith(map[string]float64{"gear": 0, "section": 3, "toothSlant": 60, "toothBow": 0})),
	sgCase("slow twist, floor of eight", sgWith(map[string]float64{"gear": 0, "section": 31, "twistLead": 400})),
	sgCase("fast twist", sgWith(map[string]float64{"gear": 1, "section": 50, "twistLead": 20})),
}

// stepCellSectionsSketch stands in for one section of a gear's Cell Sections
// sketch (§2).
//
// What is substituted, and what it costs. Fusion draws every section of the
// cell in one sketch on the Axis Plane, its points kept off the plane; the
// engine is planar, so each case draws one section k on that section's own
// plane, in the plane's coordinates x along û_g and y along v̂_g, turned to
// the station's angle. The section's M + 2 points are added and fixed after
// the last curve, the three lines L1, L3 and L4 share them, and the toothed
// side is the engine's fit spline through F_0 … F_(M-1): a natural cubic that
// interpolates every fit point, as Fusion's fitted spline does. Fusion's end
// conditions are not stated in its reference, so where between the points
// its spline runs is not this proof's to say (the step list's [PROSE] part,
// and TestToothSplineHoldsTheEdge for the natural cubic). Fusion's verdict on
// the one sketch of all the sections is Fusion's: a sketch of 41 rectangle
// sections read fully constrained with one profile each on 2026-09-28, and
// the sketch of spline sections has not been loaded.
func stepCellSectionsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	k := int(p["section"])
	if k < 0 || k > m.Cell*m.Steps {
		t.Fatalf("section %d is outside the cell's %d sections", k, m.Cell*m.Steps+1)
	}
	if sgIsDefaults(p) {
		// 10 steps a tooth and 41 sections in a four-tooth cell at the defaults.
		sketchtest.Measures(t, "sections in the cell", float64(m.Cell*m.Steps+1), 41, sketchtest.Within(0))
	}
	st := m.station(g, k)
	drawCellSection(t, s, m, g, st)
}

// drawCellSection draws section at station st and checks it.
func drawCellSection(t testing.TB, s *sketch.Sketch, m *sgModel, g int, st float64) {
	poly := m.sectionPolygon(g, st)
	proofkit.Step(t, "the section's %d points", len(poly))
	var pts []*sketch.Point
	for _, q := range poly {
		pts = append(pts, s.CreatePoint(q[0], q[1]))
	}
	b0, f, b1 := pts[0], pts[1:len(pts)-1], pts[len(pts)-1]
	if len(f) != sgSplinePoints {
		t.Fatalf("%d toothed points, want %d", len(f), sgSplinePoints)
	}
	proofkit.Step(t, "L1, S, L3, L4")
	s.CreateLine(b0, f[0])
	spline, err := s.CreateFitSpline(f...)
	if err != nil {
		t.Fatalf("fitted spline: %v", err)
	}
	if len(spline.Fit) != sgSplinePoints || spline.Fit[0] != f[0] || spline.Fit[sgSplinePoints-1] != f[sgSplinePoints-1] {
		t.Fatalf("the spline holds %d fit points, not the %d toothed points from F_0 to F_(M-1)", len(spline.Fit), sgSplinePoints)
	}
	s.CreateLine(f[sgSplinePoints-1], b1)
	s.CreateLine(b1, b0)
	proofkit.Step(t, "fix every point after the last curve")
	for _, q := range pts {
		s.Fix(q)
	}
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	// The section's area is the edge's integral across the thickness, which
	// the twist does not change. The fit spline departs from the cosine edge
	// by under 0.02 mm along u (TestToothSplineHoldsTheEdge), which over the
	// 3.75 mm thickness moves the area by under 0.08 mm² at the defaults; the
	// slack is that departure times the thickness, scaled with the tooth.
	want := 0.0
	const n = 4000
	for i := 0; i < n; i++ {
		v := -m.T/2 + (float64(i)+0.5)*m.T/n
		want += (m.utooth(g, v, st, 1) + m.W/2) * m.T / n
	}
	sketchtest.MeasuresProfileArea(t, profile, want, sketchtest.Within(0.02*m.T*m.H/2.625+1e-6))
	// The cell is one pitch-periodic piece of the ribbon: the section a pitch
	// further on is this one carried by Step(1), its toothed points included,
	// which is what lets §3 repeat the cell (TestRibbonIsInvariantUnderItsScrewStep).
	next := m.sectionPolygon(g, st+m.P)
	turn := m.P / m.Lambda
	for i, q := range poly {
		x, y := sgTurn(q[0], q[1], turn)
		sketchtest.Measures(t, fmt.Sprintf("point %d one pitch on, x", i), next[i][0], x, sketchtest.Within(1e-9))
		sketchtest.Measures(t, fmt.Sprintf("point %d one pitch on, y", i), next[i][1], y, sketchtest.Within(1e-9))
	}
}

// --- Remainder Sections sketch -----------------------------------------------

var remainderSectionCases = []proofkit.Case{
	sgCase("one tooth over, first section", sgWith(map[string]float64{"gear": 0, "toothCount": 69, "section": 0})),
	sgCase("two teeth over, last section", sgWith(map[string]float64{"gear": 1, "toothCount": 70, "section": 20})),
	sgCase("three teeth over, middle", sgWith(map[string]float64{"gear": 0, "toothCount": 71, "section": 13})),
}

// stepRemainderSectionsSketch stands in for one section of a gear's Cell
// Remainder sketch (§3): the recipe of the Cell Sections sketch with c
// replaced by r, at stations s0 + q*c*P + k*P/n. The substitution is the Cell
// Sections sketch's.
func stepRemainderSectionsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	g := int(p["gear"])
	k := int(p["section"])
	if m.Remains == 0 {
		t.Fatalf("toothCount %d leaves no remainder in %d-tooth cells", m.N, m.Cell)
	}
	if k < 0 || k > m.Remains*m.Steps {
		t.Fatalf("section %d is outside the remainder's %d sections", k, m.Remains*m.Steps+1)
	}
	first := m.cellStart(g) + float64(m.Whole*m.Cell)*m.P
	drawCellSection(t, s, m, g, first+float64(k)*m.P/float64(m.Steps))
}

// --- Sleeve sketch -----------------------------------------------------------

var sleeveSketchCases = []proofkit.Case{
	sgCase("defaults", sgDefaults()),
	sgCase("the third print", sgThirdPrint(nil)),
	sgCase("thick wall, wide cage", sgWith(map[string]float64{"cageRadius": 17, "collarHalf": 3.25})),
	sgCase("110 degree crossing", sgWith(map[string]float64{"crossAngle": 110})),
	sgCase("no roof allowance", sgWith(map[string]float64{"roofAllowance": 0})),
}

// stepSleeveSketch is the Sleeve sketch of §4 on the selected plane: two
// circles at C of radii Ri and Ro, each centre fixed, each with a diameter
// dimension. The two centre points are separate points at one place, as
// Fusion's are. It also runs the build's range checks on the case, so a case
// here is one the build accepts, and holds the fourth sleeve check, the
// separation between neighbouring bores, at the value the spec gives.
func stepSleeveSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	if why := m.refusal(p); why != "" {
		t.Fatalf("the build refuses this case, naming %s", why)
	}
	sep, where := m.channelSeparation()
	if sep < m.CollarWall {
		t.Fatalf("the wall across the %s gap is %.4f mm, under collarWall %.3f mm", where, sep, m.CollarWall)
	}
	if p["crossAngle"] == 80 && p["mountAngleA"] == 0 && p["mountAngleB"] == 0 && p["cageRadius"] == 15 &&
		p["roofAllowance"] == 0.3 && p["collarHalf"] == 3 {
		// The defaults: 5.067 mm across the -ê gap ("The wall between the bores").
		sketchtest.Measures(t, "separation at the defaults", sep, 5.067, sketchtest.Within(0.0005))
		if where != "-e" {
			t.Errorf("the least separation is across %s, want -e", where)
		}
	}
	proofkit.Step(t, "two circles at C")
	var circles []*sketch.Circle
	for _, r := range []float64{m.Ri, m.Ro} {
		centre := s.CreatePoint(m.Centre.X, m.Centre.Y)
		c := s.CreateCircle(centre, r)
		s.Fix(centre)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		circles = append(circles, c)
	}
	report := sketchtest.Verify(t, s)
	var ring *sketch.Profile
	for _, pr := range report.Profiles {
		if len(pr.Holes) == 1 { // two loops: the outer and one hole
			if ring != nil {
				t.Fatalf("two profiles with two loops")
			}
			ring = pr
		}
	}
	if ring == nil {
		t.Fatalf("no profile with two loops among %d", len(report.Profiles))
	}
	sketchtest.IsValidProfile(t, ring)
	// A ring of two exact circles: the area is exact up to the solver.
	sketchtest.MeasuresProfileArea(t, ring, math.Pi*(m.Ro*m.Ro-m.Ri*m.Ri), sketchtest.WithinRel(1e-9))
}

// --- Bore section sketch -----------------------------------------------------

var boreSectionCases = []proofkit.Case{
	sgCase("gear A -R defaults", sgWith(map[string]float64{"bore": 0})),
	sgCase("gear A +R defaults", sgWith(map[string]float64{"bore": 1})),
	sgCase("gear B -R defaults", sgWith(map[string]float64{"bore": 2})),
	sgCase("gear B +R defaults", sgWith(map[string]float64{"bore": 3})),
	sgCase("gear A -R third print", sgThirdPrint(map[string]float64{"bore": 0})),
	sgCase("gear B +R third print", sgThirdPrint(map[string]float64{"bore": 3})),
	sgCase("gear A +R no allowance", sgWith(map[string]float64{"bore": 1, "roofAllowance": 0})),
	sgCase("gear B -R no allowance", sgWith(map[string]float64{"bore": 2, "roofAllowance": 0})),
	sgCase("gear A +R level at 30 degrees", sgWith(map[string]float64{"bore": 1, "mountAngleA": 30, "mountAngleB": 30})),
	sgCase("gear B +R level at 30 degrees", sgWith(map[string]float64{"bore": 3, "mountAngleA": 30, "mountAngleB": 30})),
	sgCase("gear A -R unequal angles", sgWith(map[string]float64{"bore": 0, "mountAngleA": -40, "mountAngleB": 25})),
	sgCase("gear B -R 110 degrees", sgWith(map[string]float64{"bore": 2, "crossAngle": 110})),
	sgCase("gear A +R long lead", sgWith(map[string]float64{"bore": 1, "twistLead": 60})),
}

// sgAngleRef reports which reference the bore scheme dimensions its angle
// against at angle th, and the angle the dimension carries, in degrees in
// [45, 135]: the rays are +û from O along Ru, and either O→E along K or
// L2's start→end direction from where L2 meets Ru's line.
func sgAngleRef(th float64) (useK bool, deg float64) {
	r1 := [2]float64{1, 0}
	var r2 [2]float64
	if math.Abs(math.Sin(th)) >= math.Sqrt(0.5) {
		useK = true
		r2 = [2]float64{math.Cos(th), math.Sin(th)}
	} else {
		r2 = [2]float64{-math.Sin(th), math.Cos(th)}
	}
	return useK, math.Acos(r1[0]*r2[0]+r1[1]*r2[1]) * 180 / math.Pi
}

// The bore's plane (S19) is not built: setByDistanceOnPath at fraction 0 of
// the bore's path line stands square to the axis at the line's start, which
// is the plane this sketch is drawn on, in its coordinates; where Fusion puts
// that plane to a few nanometres is Fusion's ([PB-SKETCH-ZERO-Z]).
//
// stepBoreSectionSketch is a bore's section sketch by the rectangle scheme of
// §4, drawn on the bore's plane at the negative end of its cut, in the plane's
// coordinates x along û_g and y along v̂_g.
//
// Fusion's parallel plus offset dimension is the engine's signed offset, which
// carries both rows; Fusion takes the side from the seed, the engine from the
// sign, and the seeds are on the signed side. Fusion's angular dimension is
// unsigned and keeps the seeded wedge through its text point; the engine's is
// signed, from Ru's start→end to the other line's.
func stepBoreSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	b := m.bores()[int(p["bore"])]
	st := b.planeStation()
	th := m.theta(b.Gear, st)
	uB, uF := -m.Hw, m.Hw
	proofkit.Step(t, "reference points O and Cp, Ru and K")
	o := s.CreatePoint(0, 0)
	cp := s.CreatePoint(m.A/2, 0)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	ex, ey := sgTurn(uF, 0, th)
	e := s.CreatePoint(ex, ey)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	s.Fix(o)
	s.Fix(cp)
	proofkit.Step(t, "the rectangle's four lines")
	var c []*sketch.Point
	for _, q := range [][2]float64{{uB, b.VLo}, {uF, b.VLo}, {uF, b.VHi}, {uB, b.VHi}} {
		x, y := sgTurn(q[0], q[1], th)
		c = append(c, s.CreatePoint(x, y))
	}
	l1 := s.CreateLine(c[0], c[1])
	l2 := s.CreateLine(c[1], c[2])
	l3 := s.CreateLine(c[2], c[3])
	l4 := s.CreateLine(c[3], c[0])
	proofkit.Step(t, "length, angle, offsets, perpendicular, coincidence")
	useK, deg := sgAngleRef(th)
	if sgIsDefaults(p) && b.Gear == 0 {
		// Gear A's -R profile stands at -138.2 degrees and takes L2; its +R
		// profile at 57.4 degrees and takes K.
		want := map[float64]float64{-1: -138.18, 1: 57.40}[b.Sigma]
		sketchtest.Measures(t, b.Name+" profile angle", th*180/math.Pi, want, sketchtest.Within(0.005))
		if useK != (b.Sigma > 0) {
			t.Errorf("%s takes the other reference line", b.Name)
		}
	}
	if deg < 45-1e-9 || deg > 135+1e-9 {
		t.Fatalf("the angle dimension would read %.3f degrees, outside 45-135", deg)
	}
	var angle *sketch.Angle
	if useK {
		angle = sketch.NewAngle(ru, k, math.Mod(th*180/math.Pi+720, 360))
	} else {
		angle = sketch.NewAngle(ru, l2, math.Mod(th*180/math.Pi+90+720, 360))
	}
	s.AddConstraint(
		sketch.NewDistance(o, e, uF),
		angle,
		sketch.NewOffset(k, l1, b.VLo),
		sketch.NewOffset(k, l3, b.VHi),
		sketch.NewPointOnLine(e, l2),
		sketch.NewPerpendicular(l2, k),
		sketch.NewOffset(l2, l4, uF-uB),
	)
	sketchtest.Solve(t, s)
	// The corners solve where they were seeded: the rectangle turned by theta.
	for i, q := range [][2]float64{{uB, b.VLo}, {uF, b.VLo}, {uF, b.VHi}, {uB, b.VHi}} {
		x, y := sgTurn(q[0], q[1], th)
		proofkit.Step(t, "corner %d", i)
		sketchtest.MeasuresPoint(t, c[i], x, y, sketchtest.Within(1e-9))
	}
	sketchtest.Measures(t, "the unsigned angle Fusion dimensions", unsignedAngle(useK, ru, k, l2), deg, sketchtest.Within(1e-9))
	for _, cs := range s.Constraints() {
		sketchtest.Satisfies(t, cs, sketchtest.Within(1e-9))
	}
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, (uF-uB)*(b.VHi-b.VLo), sketchtest.WithinRel(1e-9))
	if len(profile.Entities) != 4 {
		t.Fatalf("the profile has %d curves, want the rectangle's four lines", len(profile.Entities))
	}
}

// unsignedAngle is the angle between the dimension's two rays as Fusion reads
// it.
func unsignedAngle(useK bool, ru, k, l2 *sketch.Line) float64 {
	other := l2
	if useK {
		other = k
	}
	ax, ay := ru.End.X()-ru.Start.X(), ru.End.Y()-ru.Start.Y()
	bx, by := other.End.X()-other.Start.X(), other.End.Y()-other.Start.Y()
	return math.Acos((ax*bx+ay*by)/(math.Hypot(ax, ay)*math.Hypot(bx, by))) * 180 / math.Pi
}

// --- Window sketch -----------------------------------------------------------

var windowSketchCases = []proofkit.Case{
	sgCase("defaults, facing +k", sgWith(map[string]float64{"window": 0})),
	sgCase("defaults, facing -k", sgWith(map[string]float64{"window": 1})),
	sgCase("third print, facing -k", sgThirdPrint(map[string]float64{"window": 1})),
	sgCase("110 degrees, facing +e", sgWith(map[string]float64{"window": 0, "crossAngle": 110})),
	sgCase("110 degrees, facing -e", sgWith(map[string]float64{"window": 1, "crossAngle": 110})),
	sgCase("thick wall, facing +k", sgRaised(sgWith(map[string]float64{"window": 0, "collarWall": 5}))),
	sgCase("scaled by 0.75, facing -k", sgScaled(0.75, map[string]float64{"window": 1})),
}

// sgScaled is the defaults with every length of the ribbon and the frame
// scaled by f, collarWall and the clearances held, and cageRise raised to the
// least the end-wall check accepts when the scale leaves it short.
func sgScaled(f float64, over map[string]float64) map[string]float64 {
	p := sgDefaults()
	for _, k := range []string{"ribbonWidth", "ribbonThickness", "toothPitch", "toothHeight", "twistLead",
		"cageRadius", "cageRise", "collarHalf", "engagement", "assemblyPhase"} {
		p[k] *= f
	}
	p["toothBow"] /= f
	for k, v := range over {
		p[k] = v
	}
	return sgRaised(p)
}

// sgRaised raises cageRise to the least the end-wall check accepts when the
// case leaves it short, as TestSleeveWindowsFollowTheSize does.
func sgRaised(p map[string]float64) map[string]float64 {
	m := sgModelOf(p)
	if least := m.A/2 + m.Corner + m.CollarWall; p["cageRise"] < least {
		p["cageRise"] = least
	}
	return p
}

// sgDefaultWindows are the default windows the spec states ("The hexagon").
var sgDefaultWindows = map[string]struct {
	lo, hi, bottom, top, left, right, area float64
	corners                                [][2]float64
}{
	"+k": {-5.180, 5.180, -22.151, 22.151, -11.570, 11.570, 220.7,
		[][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.57, 10.58}, {-11.57, 6.39}, {8.49, -13.67}, {11.57, -10.58}}},
	"-k": {-4.886, 5.180, -21.857, 22.151, -11.536, 11.570, 213.8,
		[][2]float64{{11.57, -6.39}, {-8.49, 13.67}, {-11.54, 10.61}, {-11.54, 6.65}, {8.49, -13.37}, {11.57, -10.29}}},
}

func sgIsDefaults(p map[string]float64) bool {
	for k, v := range sgDefaults() {
		if k == "centreX" || k == "centreY" {
			continue
		}
		if p[k] != v {
			return false
		}
	}
	return true
}

// The Window Plane (S23) is not built here; the window's cut step draws the
// hexagon on a plane through the frame's axis square to d, facing either way.
//
// stepWindowSketch is a Window sketch of §4 on the Window Plane: one point per
// corner of the hexagon the window search finds, a line from each corner to
// the next sharing them, and every point fixed. The plane coordinates are
// (t, z): t along n̂ × d and z along n̂. The search itself is the model's
// newWindow, the spec's algorithm transcribed; at the defaults it is held to
// the numbers the spec gives.
func stepWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	m := sgModelOf(p)
	if why := m.refusal(p); why != "" {
		t.Fatalf("the build refuses this case, naming %s", why)
	}
	w := m.windows()[int(p["window"])]
	if w.Corners == nil {
		t.Fatalf("no window facing %s: %s", w.Facing, w.Reason)
	}
	if sgIsDefaults(p) {
		want := sgDefaultWindows[w.Facing]
		for _, c := range []struct {
			name      string
			got, want float64
		}{{"lo", w.Lo, want.lo}, {"hi", w.Hi, want.hi}, {"bottom", w.Bottom, want.bottom},
			{"top", w.Top, want.top}, {"left", w.Left, want.left}, {"right", w.Right, want.right}} {
			// The spec quotes three decimals.
			sketchtest.Measures(t, w.Facing+" "+c.name, c.got, c.want, sketchtest.Within(0.0005))
		}
		if len(w.Corners) != len(want.corners) {
			t.Fatalf("%s window has %d corners %s, want %d", w.Facing, len(w.Corners), sgFormatCorners(w.Corners), len(want.corners))
		}
		for i, c := range want.corners {
			// The spec quotes two decimals; the corners come in the clip's order.
			sketchtest.Measures(t, fmt.Sprintf("%s corner %d t", w.Facing, i), w.Corners[i][0], c[0], sketchtest.Within(0.005))
			sketchtest.Measures(t, fmt.Sprintf("%s corner %d z", w.Facing, i), w.Corners[i][1], c[1], sketchtest.Within(0.005))
		}
		sketchtest.Measures(t, w.Facing+" area", sgArea(w.Corners), want.area, sketchtest.Within(0.05))
	}
	proofkit.Step(t, "%d corner points and the lines between them", len(w.Corners))
	var pts []*sketch.Point
	for _, c := range w.Corners {
		pts = append(pts, s.CreatePoint(c[0], c[1]))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, q := range pts {
		s.Fix(q)
	}
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, sgArea(w.Corners), sketchtest.WithinRel(1e-9))
	// Every edge is upright or at 45 degrees to the plane's axes.
	for i := range w.Corners {
		a, b := w.Corners[i], w.Corners[(i+1)%len(w.Corners)]
		dx, dy := math.Abs(b[0]-a[0]), math.Abs(b[1]-a[1])
		// A corner dropped within 0.001 mm of its neighbour moves an edge's end
		// by at most that much, so an edge is upright or at 45 degrees to 0.002 mm.
		if !(dx < 0.002 || math.Abs(dx-dy) < 0.002) {
			t.Errorf("edge %d runs (%.6f, %.6f), neither upright nor at 45 degrees", i, b[0]-a[0], b[1]-a[1])
		}
	}
}
