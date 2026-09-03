package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// pressureAngle and involuteSteps are what the framework's VirtualSpurProxy
// serves the borrowed spur tooth generator; neither is a bevel dialog input.
var pressureAngle = 20 * math.Pi / 180

const involuteSteps = 15

// toothCases prove the virtual spur tooth §3 draws per gear. The branch that
// matters is the borrowed generator's `embedded` flag, which decides the line
// count the profile search keys on, so the table reaches it from both sides:
// the flag turns on at high virtual tooth counts (41.5 teeth at a 20 degree
// pressure angle), and the virtual count is the real count divided by cos(gamma),
// so a modest gear crosses it while a small one does not.
var toothCases = []proofkit.Case{
	{Name: "pinion-31-31-90", Params: map[string]float64{"gear": 0}},
	{Name: "driving-31-31-90", Params: map[string]float64{"gear": 1}},
	{Name: "pinion-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 0}},
	{Name: "driving-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 1}},
	{Name: "small-pair-4-4", Params: map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}},
	{Name: "shallow-cone-at-30", Params: map[string]float64{"shaftAngleDeg": 30, "gear": 1}},
	{Name: "with-tooth-spacing", Params: map[string]float64{"toothSpacing": 0.4, "gear": 0}},
	{Name: "module-2-19-13", Params: map[string]float64{"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "gear": 1}},
}

// profileCases prove the per-gear Profile sketch. Both gears are drawn, since
// the hexagon and its first edge differ per gear, and the table reaches the
// Shaft Angle ends and both ratio directions because the Face Width cap that
// keeps the toe off the axis binds differently on each.
var profileCases = []proofkit.Case{
	{Name: "pinion-default", Params: map[string]float64{"gear": 0}},
	{Name: "driving-default", Params: map[string]float64{"gear": 1}},
	{Name: "pinion-of-31-17", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "gear": 0}},
	{Name: "driving-of-17-31", Params: map[string]float64{"drivingTeeth": 17, "pinionTeeth": 31, "gear": 1}},
	{Name: "pinion-at-150", Params: map[string]float64{"shaftAngleDeg": 150, "gear": 0}},
	{Name: "driving-at-30", Params: map[string]float64{"shaftAngleDeg": 30, "gear": 1}},
	{Name: "mirrored-grow-side", Params: map[string]float64{"growSign": -1, "gear": 1}},
	{Name: "teeth-floor-4", Params: map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4, "gear": 0}},
}

// toothProfile is one drawn virtual spur tooth, in the tooth plane's own frame
// with the tooth centre at the origin.
type toothProfile struct {
	dims     involute.Dimensions
	embedded bool
	teeth    float64
	rootEnd  float64 // radius the drawn flanks end at
	left     []involute.Pt
	right    []involute.Pt
}

// stepVirtualSpurTooth draws §3 step 3: the borrowed spur tooth, at the virtual
// tooth number, on the tooth plane, already rotated 180 degrees by the draw
// angle rather than by a later sketch rotation.
//
// What bevel owns here is the virtual tooth number, the module handed to the
// drawer, the tooth centre K'/L' and the 180 degree draw angle; the tooth's own
// constraint scheme belongs to spec/spurgear and its own bench proof. This step
// therefore draws the tooth from proof/involute on fixed points and proves the
// facts bevel depends on: the closed-form virtual count, the four circle radii
// the drawer derives from it, the 180 degree placement, and — the one a later
// step selects on — the curve-count key, 2 splines and 2 arcs with 0 connecting
// lines when the tooth is embedded and 2 when it is not.
//
// Not reached here: the four labelled circles the drawer also draws
// ([PB-SKETCH-TEXT]), and with them the impostor loop the spec warns about — an
// unrelated region between those circles carrying the same 2 splines and 2 arcs
// with the other line count. The proof shows the key is a function of the flag;
// it does not show what else is in the sketch to be confused with.
func stepVirtualSpurTooth(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)

	proofkit.Step(t, "%s: virtual tooth number from the back-cone closed form", g.Label)
	virtualPitchRadius := (g.PitchDia / 2) / math.Cos(g.Gamma)
	virtualTeeth := math.Floor(2 * virtualPitchRadius / c.Module)
	near(t, virtualTeeth, g.VirtualTeeth, 1e-12, "%s virtual tooth number", g.Label)
	if virtualTeeth < g.Teeth {
		t.Errorf("%s: the virtual tooth number %v must be at least the real count %v", g.Label, virtualTeeth, g.Teeth)
	}
	// The Units note: the stashed pitch diameters are internal centimetres while
	// Module is raw millimetres, so a virtual radius computed without the cm->mm
	// conversion comes out ten times small.
	unconverted := math.Floor(2 * (virtualPitchRadius / 10) / c.Module)
	if unconverted == virtualTeeth && virtualTeeth > 1 {
		t.Errorf("%s: the case cannot tell a converted virtual count from an unconverted one", g.Label)
	}

	tooth := drawTooth(t, s, c.Module, virtualTeeth)

	proofkit.Step(t, "%s: the four circle radii the drawer derives", g.Label)
	near(t, tooth.dims.Pitch, c.Module*virtualTeeth/2, 1e-12, "virtual pitch radius")
	near(t, tooth.dims.Base, tooth.dims.Pitch*math.Cos(pressureAngle), 1e-12, "base radius")
	near(t, tooth.dims.Tip, (c.Module*virtualTeeth+2*c.Module)/2, 1e-12, "tip radius (one module addendum)")
	near(t, tooth.dims.Root, (c.Module*virtualTeeth-2.5*c.Module)/2, 1e-12, "root radius (1.25 module dedendum)")

	proofkit.Step(t, "%s: the profile the loft consumes and the curve-count key", g.Label)
	wantLines := 2
	if tooth.embedded {
		wantLines = 0
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("%s: the tooth is one closed region, got %d", g.Label, len(profiles))
	}
	prof := profiles[0]
	if !prof.Valid {
		t.Errorf("%s: the tooth region is not extrudable", g.Label)
	}
	var arcs, splines, lines int
	for _, e := range prof.Entities {
		switch e.(type) {
		case *sketch.Arc:
			arcs++
		case *sketch.FitSpline:
			splines++
		case *sketch.Line:
			lines++
		}
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("%s: tooth loop is %d splines, %d arcs, %d lines; the profile search asks for 2, 2 and %d (embedded=%v)",
			g.Label, splines, arcs, lines, wantLines, tooth.embedded)
	}
	if tooth.embedded != (tooth.dims.Base < tooth.dims.Root) {
		t.Errorf("%s: the embedded flag must be base < root", g.Label)
	}

	proofkit.Step(t, "%s: the 180 degree draw angle places the tooth", g.Label)
	lastLeft, lastRight := tooth.left[len(tooth.left)-1], tooth.right[len(tooth.right)-1]
	mid := math.Atan2(lastLeft.Y+lastRight.Y, lastLeft.X+lastRight.X)
	near(t, math.Abs(mid), math.Pi, 1e-9, "%s tooth centreline sits at 180 degrees", g.Label)
	for i := range tooth.left {
		near(t, math.Hypot(tooth.left[i].X, tooth.left[i].Y), math.Hypot(tooth.right[i].X, tooth.right[i].Y), 1e-12,
			"%s flanks are mirror images at sample %d", g.Label, i)
	}
	// The Tooth Spacing moves the tooth CENTRE only; the drawn tooth keeps the
	// size the virtual count gives it.
	near(t, tooth.teeth, g.VirtualTeeth, 1e-12, "%s drawn tooth size ignores the Tooth Spacing", g.Label)
}

// drawTooth draws one involute tooth centred on the sketch origin and rotated
// by 180 degrees, on fixed points.
//
// Two substitutions, both of the kind the playbook's sketch-first note allows.
// Fusion derives the root boundary by splitting the drawn root circle; the
// bench draws that same derived arc directly. And an embedded tooth's flank
// starts inside the root circle, where Fusion trims it: the bench drops the
// samples inside the root radius, so the drawn flank starts at the first sample
// outside it rather than exactly on it, and the step checks that landing is
// within one sampling step of the root circle.
func drawTooth(t testing.TB, s *sketch.Sketch, module, teeth float64) toothProfile {
	t.Helper()
	dims := involute.Derive(module, teeth, pressureAngle)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, teeth, involuteSteps, math.Pi)
	embedded := dims.Embedded()

	keep := func(pts []involute.Pt) []involute.Pt {
		if !embedded {
			return pts
		}
		out := make([]involute.Pt, 0, len(pts))
		for _, q := range pts {
			if math.Hypot(q.X, q.Y) >= dims.Root {
				out = append(out, q)
			}
		}
		return out
	}
	left, right = keep(left), keep(right)
	if len(left) < 2 || len(right) < 2 {
		proofkit.Unmodelled(t, "the tooth keeps %d flank samples outside the root circle, too few to draw", len(left))
	}
	rootEnd := math.Hypot(left[0].X, left[0].Y)
	if embedded {
		step := (dims.Tip - dims.Base) / float64(involuteSteps-1)
		if rootEnd-dims.Root > step {
			t.Errorf("the drawn flank starts %.4f mm outside the root circle, more than one sampling step %.4f mm",
				rootEnd-dims.Root, step)
		}
	} else {
		near(t, rootEnd, dims.Base, 1e-9, "a non-embedded flank starts on the base circle")
	}

	centre := s.CreatePoint(0, 0)
	centre.SetName("tooth centre")
	s.Fix(centre)

	// An arc drawn on the tooth centre does not SHARE that point: Fusion's
	// addByCenterStartEnd copies the centre, which is how a tooth-top arc came
	// out at 0.5743 mm where 22.5 mm was meant, from a sketch that raised no
	// error ([PB-SHARE-XOR-COINCIDENT]). The bench models the copy the same way —
	// a fresh centre point — and pins it back to the tooth centre, then checks
	// the gap it closes. The pin is one row, not two: the arc's own endpoint rows
	// already leave its centre on the tooth centreline, so a full coincidence is
	// redundant there and the engine says so.
	arcCentre := func(name string) *sketch.Point {
		q := s.CreatePoint(0, 0)
		q.SetName(name)
		s.AddConstraint(sketch.NewHorizontalDistance(centre, q, 0))
		return q
	}

	pt := func(q involute.Pt, name string) *sketch.Point {
		sp := s.CreatePoint(q.X, q.Y)
		sp.SetName(name)
		return sp
	}
	leftPts := make([]*sketch.Point, 0, len(left))
	rightPts := make([]*sketch.Point, 0, len(right))
	for i, q := range left {
		leftPts = append(leftPts, pt(q, "left flank"))
		rightPts = append(rightPts, pt(right[i], "right flank"))
	}
	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank: %v", err)
	}
	rightFlank.SetName("right flank")

	last := len(left) - 1
	tipCentre := arcCentre("tooth top arc centre")
	tip := s.CreateArc(tipCentre, leftPts[last], rightPts[last])
	tip.SetName("tooth top arc")

	var rootStart, rootEndPt *sketch.Point
	if embedded {
		rootStart, rootEndPt = rightPts[0], leftPts[0]
	} else {
		// The flank-to-root connecting lines run radially inward to the root
		// circle; they are the two lines the profile search counts.
		radial := func(from *sketch.Point, name string) *sketch.Point {
			r := math.Hypot(from.X(), from.Y())
			to := s.CreatePoint(from.X()*dims.Root/r, from.Y()*dims.Root/r)
			to.SetName(name)
			l := s.CreateLine(from, to)
			l.SetName(name)
			return to
		}
		rootEndPt = radial(leftPts[0], "left flank-to-root")
		rootStart = radial(rightPts[0], "right flank-to-root")
	}
	rootCentre := arcCentre("root arc centre")
	root := s.CreateArc(rootCentre, rootStart, rootEndPt)
	root.SetName("root arc")

	for _, q := range append(append([]*sketch.Point{}, leftPts...), rightPts...) {
		s.Fix(q)
	}
	if !embedded {
		s.Fix(rootStart)
		s.Fix(rootEndPt)
	}
	solveHere(t, s)

	// Both arc centres must close onto the tooth centre exactly; a stranded
	// centre is the defect fusion.md records, and it deforms the tooth without
	// raising anything.
	near(t, math.Hypot(tipCentre.X()-centre.X(), tipCentre.Y()-centre.Y()), 0, 1e-9, "tooth top arc centre gap")
	near(t, math.Hypot(rootCentre.X()-centre.X(), rootCentre.Y()-centre.Y()), 0, 1e-9, "root arc centre gap")
	near(t, math.Hypot(leftPts[last].X()-tipCentre.X(), leftPts[last].Y()-tipCentre.Y()), dims.Tip, 1e-9,
		"tooth top arc radius is the tip radius")
	wantRoot := dims.Root
	if embedded {
		// The embedded tooth's root boundary is the arc through the first flank
		// samples outside the root circle, the substitution named above.
		wantRoot = rootEnd
	}
	near(t, math.Hypot(rootStart.X()-rootCentre.X(), rootStart.Y()-rootCentre.Y()), wantRoot, 1e-9,
		"root arc radius")
	return toothProfile{dims: dims, embedded: embedded, teeth: teeth, rootEnd: rootEnd, left: left, right: right}
}

// stepProfileHexagon draws the per-gear Profile sketch: the six §2 vertices
// recreated as new points at their world positions, the closed hexagon drawn
// sharing those points, and the points fixed AFTER the lines exist
// ([PB-PROJECT-NOT-FIXED]).
//
// The sketch holds exactly one loop, so the revolve takes its single profile
// ([PB-SINGLE-PROFILE]), and the loop's FIRST edge is the shaft axis every body
// operation below uses — the revolve, the pattern, the bore plane and the
// meshing rotation — so the step checks that edge is on the axis and that no
// other vertex crosses it, which is the ASM_WIRE_X_AXIS failure the Maximum
// Face Width exists to prevent ([PB-REVOLVE]).
func stepProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := mustResolve(t, p)
	g := c.gearOf(p)

	proofkit.Step(t, "%s Profile: recreate A, G, H, C, M, N and draw the hexagon", g.Label)
	verts := []vec2{g.Axis, g.G, g.H, g.Ded, g.M, g.N}
	names := []string{"A", "G", "H", "C", "M", "N"}
	if g.Label == "Driving" {
		names = []string{"B", "I", "J", "D", "O", "P"}
	}
	pts := make([]*sketch.Point, len(verts))
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.X, v.Y)
		pts[i].SetName(g.Label + " " + names[i])
	}
	edges := make([]*sketch.Line, len(pts))
	for i := range pts {
		edges[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		edges[i].SetName(g.Label + " " + names[i] + "->" + names[(i+1)%len(names)])
	}
	// The points are fixed only once the lines exist; fixing a bare point first
	// does not leave the sketch fully constrained ([PB-PROJECT-NOT-FIXED]).
	for _, e := range edges {
		s.Fix(e.Start)
		s.Fix(e.End)
	}
	solveHere(t, s)

	proofkit.Step(t, "%s Profile: the first edge is the shaft axis every body operation uses", g.Label)
	axisDir := v2unit(v2sub(g.G, g.Axis))
	near(t, math.Abs(v2dot(axisDir, v2unit(v2sub(g.Axis, c.Apex)))), 1, 1e-9,
		"%s first edge is collinear with the section-2 Apex->A shaft line", g.Label)
	near(t, v2len(v2sub(g.G, g.Axis)), g.BaseHeight, 1e-9, "%s first edge spans the resolved base height", g.Label)

	side := 0.0
	for i, v := range verts {
		d := v2cross(axisDir, v2sub(v, g.Axis))
		if math.Abs(d) < 1e-9 {
			continue // A and G lie on the axis of revolution
		}
		if side == 0 {
			side = math.Copysign(1, d)
		} else if math.Copysign(1, d) != side {
			t.Errorf("%s: vertex %s crosses the axis of revolution; the revolve fails with ASM_WIRE_X_AXIS",
				g.Label, names[i])
		}
	}

	proofkit.Step(t, "%s Profile: the toe and heel edges the trims and the spiral hand-off name", g.Label)
	near(t, v2distToLine(g.M, g.Ded, g.DedDir), c.FaceWidth, 1e-9,
		"%s toe edge sits one Face Width from the heel edge", g.Label)
	near(t, v2len(v2sub(g.H, g.Ded)), g.BaseHeight/math.Sin(g.Gamma)-1.25*c.Module, 1e-9,
		"%s heel edge length C->H", g.Label)
	rootDir := g.RootDir(c.Apex)
	if v2dot(v2sub(g.M, c.Apex), rootDir) >= v2dot(v2sub(g.Ded, c.Apex), rootDir) {
		t.Errorf("%s: the toe edge is not the inner one", g.Label)
	}

	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("the Profile sketch holds exactly one hexagon loop, got %d region(s)", len(profiles))
	}
	prof := profiles[0]
	if !prof.Valid || prof.Area <= 0 {
		t.Errorf("the hexagon region is not extrudable: valid=%v area=%.6f", prof.Valid, prof.Area)
	}
	if len(prof.Outer) != 6 {
		t.Errorf("the hexagon loop has %d edges, want 6", len(prof.Outer))
	}
}
