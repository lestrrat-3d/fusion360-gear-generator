package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// stepAnchorSketch draws §1's `Anchor` sketch on the user's target plane: the
// projected centre point and the reference line through it that every §2
// direction is taken relative to.
//
// One substitution. The spec adds BOTH a coincident between the projected centre
// and the anchor line and a midpoint constraint, and says to use both rather than
// the midpoint alone. In the engine the midpoint already puts the point on the
// line, so the pair is redundant by one equation and the gate fails on it. The
// proof adds the midpoint and asserts below that the coincident's own condition
// holds anyway — which is the measurement behind calling the pair redundant.
// The aligned distance dimension is written as the signed horizontal distance the
// seeded side implies, so the two endpoints cannot swap: an unsigned length plus
// a horizontal constraint reaches DOF 0 and still admits the mirrored line.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)

	proofkit.Step(t, "project the user-selected centre point onto the target plane")
	centre := s.CreateReferencePoint(p.CenterX, p.CenterY, "user centre point")
	centre.SetName("projected centre")

	proofkit.Step(t, "the anchor line, seeded at plus and minus 5 mm along sketch-local X")
	start := s.CreatePoint(p.CenterX-5, p.CenterY)
	end := s.CreatePoint(p.CenterX+5, p.CenterY)
	anchor := s.CreateLine(start, end)
	anchor.SetName("Anchor Line")

	proofkit.Step(t, "midpoint, length and direction")
	s.AddConstraint(sketch.NewMidpoint(centre, anchor))
	s.AddConstraint(sketch.NewHorizontal(anchor))
	s.AddConstraint(sketch.NewHorizontalDistance(start, end, 10))

	proofkit.Step(t, "check the anchor line against what §1 pins")
	near(t, "the seeded anchor line length", anchor.Length(), 10, 1e-9)
	nearPoint(t, "the anchor line's midpoint",
		scale(add(vec{start.X(), start.Y()}, vec{end.X(), end.Y()}), 0.5),
		vec{p.CenterX, p.CenterY}, 1e-9)
	// The coincident the proof did not add. Its condition holds anyway, which is
	// what makes the spec's pair redundant rather than merely unnecessary.
	near(t, "the projected centre's distance off the anchor line",
		math.Abs(cross(sub(vec{p.CenterX, p.CenterY}, vec{start.X(), start.Y()}),
			unit(sub(vec{end.X(), end.Y()}, vec{start.X(), start.Y()})))), 0, 1e-9)
}

// anchorCases: the anchor sketch takes no dialog value, so its regime is where
// the projected centre lands — the sketch origin, and away from it in each
// direction, since nothing in §1 assumes the projection is at the origin.
var anchorCases = []proofkit.Case{
	{Name: "centre_at_the_sketch_origin", Params: map[string]float64{}},
	{Name: "centre_off_origin_positive", Params: map[string]float64{
		"centerX": 37.5, "centerY": 21}},
	{Name: "centre_off_origin_negative", Params: map[string]float64{
		"centerX": -18.25, "centerY": -44.5}},
}

// hexagonVertices returns one gear's profile-sketch hexagon in draw order, with
// the shaft-axis edge first: A->G->H->C->M->N for the pinion, B->I->J->D->O->P for
// the driving gear.
func hexagonVertices(l lattice, pinion bool) []vec {
	if pinion {
		return []vec{l.A, l.G, l.H, l.C, l.M, l.N}
	}
	return []vec{l.B, l.I, l.J, l.D, l.O, l.P}
}

// stepGearHexagon draws one gear's `{label} Profile` sketch: a fresh sketch on the
// axial plane holding exactly one closed hexagon, built by the recreate-share-fix
// recipe — the six §2 vertices recreated as new points at their solved positions,
// the six lines drawn sharing those points, and the endpoints fixed only once the
// lines exist.
func stepGearHexagon(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	l := solveLattice(p)
	verts := hexagonVertices(l, p.Pinion)

	proofkit.Step(t, "recreate the six §2 vertices at their solved positions")
	handles := make([]*sketch.Point, 0, len(verts))
	names := []string{"A", "G", "H", "C", "M", "N"}
	if !p.Pinion {
		names = []string{"B", "I", "J", "D", "O", "P"}
	}
	for i, v := range verts {
		h := s.CreatePoint(v.X, v.Y)
		h.SetName(names[i])
		handles = append(handles, h)
	}

	proofkit.Step(t, "the closed hexagon, its first edge the gear's shaft axis")
	lines := make([]*sketch.Line, 0, len(handles))
	for i := range handles {
		line := s.CreateLine(handles[i], handles[(i+1)%len(handles)])
		line.SetName(names[i] + "->" + names[(i+1)%len(names)])
		lines = append(lines, line)
	}

	proofkit.Step(t, "fix the endpoints, now the lines exist")
	for _, line := range lines {
		s.Fix(line.Start)
		s.Fix(line.End)
	}

	proofkit.Step(t, "check the one loop the revolve consumes")
	assertHexagon(t, s, p, l, verts, lines)
}

// assertHexagon holds the profile sketch to what the revolve needs: one closed
// six-line loop, and every vertex on one side of the shaft axis it is revolved
// about.
func assertHexagon(t testing.TB, s *sketch.Sketch, p params, l lattice, verts []vec,
	lines []*sketch.Line) {
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("the profile sketch holds %d closed region(s); it must hold exactly one, "+
			"so the revolve takes profiles.item(0) without filtering", len(profiles))
	}
	loop := profiles[0]
	if !loop.Valid || loop.SelfIntersecting {
		t.Fatalf("the hexagon is not extrudable: valid=%v selfIntersecting=%v; a folded "+
			"profile is what an over-long face width or an inward heel edge produces",
			loop.Valid, loop.SelfIntersecting)
	}
	if len(loop.Outer) != 6 {
		t.Errorf("the hexagon loop walks %d edge(s), want 6", len(loop.Outer))
	}
	for _, edge := range loop.Outer {
		if _, ok := edge.Entity.(*sketch.Line); !ok {
			t.Errorf("the hexagon loop walks a %T; every edge must be a line", edge.Entity)
		}
	}

	// The first edge is the shaft axis, and the revolve spins the loop about it.
	// A profile that crosses that axis aborts the revolve, so every vertex must sit
	// on one side of it.
	axisFrom, axisTo := verts[0], verts[1]
	axisDir := unit(sub(axisTo, axisFrom))
	shaft := unit(sub(l.A, l.Apex))
	corner, hat, shaftPoint := l.C, l.CHat, l.A
	gamma := coneOf(p).GammaP
	pitchRadius := coneOf(p).PPD / 2
	if !p.Pinion {
		shaft = unit(sub(l.B, l.Apex))
		corner, hat, shaftPoint = l.D, l.DHat, l.B
		gamma = coneOf(p).GammaG
		pitchRadius = coneOf(p).DPD / 2
	}
	near(t, "the first hexagon edge against the gear's shaft direction",
		math.Abs(dot(axisDir, shaft)), 1, 1e-9)

	// The side is read off the loop rather than assumed: the two gears' hexagons
	// are drawn in opposite senses, so what matters is that every vertex shares one
	// sign, not which sign that is.
	side := func(v vec) float64 { return cross(sub(v, axisFrom), axisDir) }
	want := 0.0
	for _, v := range verts {
		if s := side(v); math.Abs(s) > 1e-9 {
			want = s
			break
		}
	}
	for i, v := range verts {
		if s := side(v); math.Abs(s) > 1e-9 && (s > 0) != (want > 0) {
			t.Errorf("hexagon vertex %d is %.6f mm on the far side of the shaft axis; the "+
				"revolve aborts on a profile that crosses its axis", i, math.Abs(s))
		}
	}

	// Two separate ways the loop reaches across that axis, and the spec bounds only
	// the first.
	//
	// The TOE end: the Maximum Face Width is exactly the offset at which the toe
	// end lands on the shaft corner, so a face width past it drives the toe across.
	// The proof pushes the offset 2 per cent past the reach and checks the toe end
	// really does change sides.
	reach := math.Abs(cross(sub(shaftPoint, corner), hat))
	_, over := toeEdge(corner, hat, l.Apex, shaftPoint, l.Apex2, reach*1.02)
	if s := side(over); math.Abs(s) > 1e-9 && (s > 0) == (want > 0) {
		t.Errorf("a face width 2%% past the reach %.4f mm leaves the toe end on the near "+
			"side of the shaft axis; the bound is supposed to be the crossing point", reach)
	}

	// The HEEL edge: C->H (D->J) runs from the dedendum corner toward the axis, and
	// crosses it once the base height carries H past the crossing. Nothing in the
	// spec bounds this. The condition is base height < (pitch radius - 1.25 * module
	// * cos gamma) * tan gamma, and it binds at small gamma — for the default 31/31
	// pair it is 3.83 mm against a resolved base height of 3.875 mm at the 30 degree
	// end of the stated shaft-angle range.
	drivingBase, pinionBase := resolvedBaseHeights(p)
	base := drivingBase
	if p.Pinion {
		base = pinionBase
	}
	limit := (pitchRadius - 1.25*p.Module*math.Cos(gamma)) * math.Tan(gamma)
	if base >= limit {
		t.Errorf("the resolved base height %.4f mm is at or past %.4f mm, the offset at which "+
			"the heel edge crosses the shaft axis; the Maximum Face Width bounds the toe end "+
			"only and does not see this", base, limit)
	}
	_ = lines
}

// hexagonCases: both gears, both ratio directions, both ends of the shaft-angle
// range the lattice holds across, and the face-width and base-height branches
// that move the hexagon's toe and heel edges.
var hexagonCases = []proofkit.Case{
	{Name: "default_pinion", Params: map[string]float64{"pinion": 1}},
	{Name: "default_driving", Params: map[string]float64{"pinion": 0}},
	{Name: "ratio_pinion_binding", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "ratio_driving_binding", Params: map[string]float64{
		"drivingTeeth": 17, "pinionTeeth": 31, "pinion": 0}},
	{Name: "shaft_angle_30_pinion", Params: map[string]float64{
		"shaftAngle": 30, "drivingBaseHeight": 2, "pinion": 1}},
	{Name: "shaft_angle_30_driving", Params: map[string]float64{
		"shaftAngle": 30, "drivingBaseHeight": 2, "pinion": 0}},
	{Name: "shaft_angle_135_driving", Params: map[string]float64{
		"shaftAngle": 135, "pinion": 0}},
	{Name: "face_width_given_pinion", Params: map[string]float64{
		"faceWidth": 4, "pinion": 1}},
	{Name: "base_heights_given_driving", Params: map[string]float64{
		"drivingBaseHeight": 6, "pinionBaseHeight": 2.5, "pinion": 0}},
	{Name: "module_2_pinion_13", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 1}},
	{Name: "low_tooth_count_8_8_pinion", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 1}},
}

// boreDiameter resolves one gear's bore: the dialog value when it is non-zero,
// otherwise this gear's pitch diameter over four.
func boreDiameter(p params) float64 {
	c := coneOf(p)
	given, pitch := p.DrivingBore, c.DPD
	if p.Pinion {
		given, pitch = p.PinionBore, c.PPD
	}
	if given != 0 {
		return given
	}
	return pitch / 4
}

// stepBoreSketch draws one gear's `{label} Bore` sketch on the plane normal to the
// shaft at its start. The circle is centred on the sketch origin, which sits on
// the axis because the plane is rooted at the shaft's start point.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	if !p.BoreEnable {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no bore sketch is created and "+
			"there is no sketch to gate")
		return
	}
	diameter := boreDiameter(p)

	proofkit.Step(t, "the bore circle, centred on the plane's own origin")
	centre := s.CreatePoint(0, 0)
	centre.SetName("bore centre")
	circle := s.CreateCircle(centre, diameter/2)
	circle.SetName("bore circle")
	// A circle's centre is free even when it is created at the origin, so it is
	// fixed rather than coincided to the sketch origin.
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, diameter))

	proofkit.Step(t, "check the resolved bore diameter")
	near(t, "the bore circle diameter", 2*circle.R(), diameter, 1e-9)
	c := coneOf(p)
	pitch := c.DPD
	given := p.DrivingBore
	if p.Pinion {
		pitch, given = c.PPD, p.PinionBore
	}
	if given == 0 {
		near(t, "the auto bore diameter against a quarter of the pitch diameter",
			diameter, pitch/4, 1e-9)
	} else {
		near(t, "the given bore diameter", diameter, given, 1e-9)
	}
	if diameter >= pitch {
		t.Errorf("the bore diameter %.4f mm is not smaller than the pitch diameter %.4f mm",
			diameter, pitch)
	}
}

// boreCases: both gears, both sides of the auto-diameter branch, and the
// unchecked Enable Bore that skips the step entirely.
var boreCases = []proofkit.Case{
	{Name: "auto_pinion", Params: map[string]float64{"pinion": 1}},
	{Name: "auto_driving", Params: map[string]float64{"pinion": 0}},
	{Name: "given_pinion", Params: map[string]float64{"pinion": 1, "pinionBore": 6}},
	{Name: "given_driving", Params: map[string]float64{"pinion": 0, "drivingBore": 9.5}},
	{Name: "auto_ratio_pinion_17", Params: map[string]float64{
		"pinion": 1, "drivingTeeth": 31, "pinionTeeth": 17}},
	{Name: "bore_disabled", Params: map[string]float64{"boreEnable": 0}},
}
