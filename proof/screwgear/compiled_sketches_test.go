package screwgear_test

// The sketch steps of spec/screwgear/steps.md, each built in the sketch engine
// through proofkit.Run, which gates it on sketch.VerificationReport.Check.
//
// Every sketch here is drawn in its own plane's coordinates on the harness's
// XY sketch. The Fusion sketch sits on a plane whose own x axis the build does
// not choose, and the build maps world points in with modelToSketchSpace; the
// proof draws the same points in the plane frame the step names, which is the
// same figure up to a rigid motion of the plane.
//
// The construction planes the sketches stand on — the two Gear Axis Planes,
// the four bore planes and the Window Plane — are neither a sketch nor a
// solid, so neither engine gates them. Each sketch below is drawn in the frame
// its plane gives it, and the frame is asserted where it is read: the Paths
// sketch holds the gear's axis on its axis plane, the bore section sketch holds
// O on the bore's line, and the window sketch holds the Window Plane through C
// and n. Where Fusion puts a plane, and which way its normal points, the proof
// cannot see ([SCREW-F-NORMAL-SIGN], [PB-SKETCH-ZERO-Z]); the build reads both.

import (
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// ---------------------------------------------------------------------------
// Case tables.

// cgSize is one accepted input of the sleeve's regime.
type cgSize struct {
	name string
	in   cgIn
}

// cgSizes is the regime the spec names for the sleeve: the defaults and every
// input TestSleeveWindowsFollowTheSize builds the sleeve at (windowSizes),
// kept where the build accepts it (sleeveRefusal is the build's four sleeve
// checks in the build's order). The spec states the build refuses five of the
// 33, each for the wall between two bores; TestSleeveWindowsFollowTheSize
// holds that, and the proof builds the 28 it accepts.
var cgSizes = sync.OnceValue(func() []cgSize {
	out := []cgSize{{"defaults", cgDefaults()}}
	for _, s := range windowSizes() {
		if sleeveRefusal(s.p) != "" {
			continue
		}
		out = append(out, cgSize{s.name, cgFromParams(s.p)})
	}
	return out
})

// cgRibbonInputs are the inputs the ribbon's steps are proved at: the
// defaults, the leads at the ends of the range TestLoftSectionCountHoldsTheHelicoid
// covers (20 mm, where the twist sets the section count, and 400 mm, where the
// floor of eight steps does), the one-tooth cell fallback, and the assembly
// phase near both ends of its open range, +-toothPitch.
func cgRibbonInputs() []cgSize {
	d := cgDefaults()
	lead20, lead400, oneTooth, phaseLo, phaseHi := d, d, d, d, d
	lead20.Lead = 20
	lead400.Lead = 400
	oneTooth.CellTeeth = 1
	phaseLo.Phase = -2.6
	phaseHi.Phase = 2.6
	return []cgSize{
		{"defaults", d}, {"lead 20", lead20}, {"lead 400", lead400},
		{"one-tooth cell", oneTooth}, {"assembly phase -2.6", phaseLo}, {"assembly phase +2.6", phaseHi},
	}
}

// cgWithTeeth is the defaults at another tooth count.
func cgWithTeeth(n int) cgIn {
	in := cgDefaults()
	in.N = n
	return in
}

func cgGearCases(sizes []cgSize, extra map[string]float64) []proofkit.Case {
	var out []proofkit.Case
	for _, s := range sizes {
		for gi, label := range []string{"gear A", "gear B"} {
			e := map[string]float64{"gear": float64(gi)}
			for k, v := range extra {
				e[k] = v
			}
			out = append(out, proofkit.Case{Name: s.name + ", " + label, Params: cgMap(s.in, e)})
		}
	}
	return out
}

// anchorCases puts the selected centre point at the plane's origin and away
// from it: the Anchor sketch is the same figure wherever the point is.
var anchorCases = []proofkit.Case{
	{Name: "centre at the plane's origin", Params: map[string]float64{"centreX": 0, "centreY": 0}},
	{Name: "centre off the origin", Params: map[string]float64{"centreX": 37.2, "centreY": -12.5}},
}

var pathsCases = func() []proofkit.Case { return cgGearCases(cgSizes(), nil) }()

// cellSectionsCases are the cell of §2 for both gears at every ribbon input.
var cellSectionsCases = cgGearCases(cgRibbonInputs(), nil)

// remainderSketchCases reach every remainder a four-tooth cell leaves, one to
// three teeth, behind seventeen whole cells, and behind a single cell, where
// no round runs first.
var remainderSketchCases = cgGearCases([]cgSize{
	{"69 teeth", cgWithTeeth(69)}, {"70 teeth", cgWithTeeth(70)}, {"71 teeth", cgWithTeeth(71)},
	{"5 teeth", cgWithTeeth(5)}, {"7 teeth", cgWithTeeth(7)},
}, nil)

var sleeveSketchCases = func() []proofkit.Case {
	var out []proofkit.Case
	for _, s := range cgSizes() {
		out = append(out, proofkit.Case{Name: s.name, Params: cgMap(s.in, nil)})
	}
	return out
}()

// cgBoreNames are the four bores in the order the build cuts them (§4,
// "Order"): bore index 0..3.
var cgBoreNames = [4]string{"gear A -R", "gear A +R", "gear B -R", "gear B +R"}

var boreSectionCases = func() []proofkit.Case {
	var out []proofkit.Case
	for _, s := range cgSizes() {
		for b, name := range cgBoreNames {
			out = append(out, proofkit.Case{Name: s.name + ", " + name,
				Params: cgMap(s.in, map[string]float64{"bore": float64(b)})})
		}
	}
	return out
}()

var windowSketchCases = func() []proofkit.Case {
	var out []proofkit.Case
	for _, s := range cgSizes() {
		for w, name := range []string{"window facing d", "window facing -d"} {
			out = append(out, proofkit.Case{Name: s.name + ", " + name,
				Params: cgMap(s.in, map[string]float64{"window": float64(w)})})
		}
	}
	return out
}()

// ---------------------------------------------------------------------------
// The Anchor sketch (§1).

// stepAnchorSketch builds the Anchor sketch: the selected point projected in
// (a reference point, since Fusion's projection tracks its source), and the
// Anchor Line from seeds 5 mm either side of it along the sketch's x axis,
// held by midpoint, horizontal and a horizontal distance of 10 mm from start
// to end. The engine's midpoint carries the coincident row Fusion's
// addCoincident adds, so the proof writes the midpoint alone, as
// proof/bevelgear does. The horizontal distance is signed in the engine and
// runs one way in Fusion, so only the seeded orientation, end to the right of
// start, satisfies it.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	cx, cy := p["centreX"], p["centreY"]
	proofkit.Step(t, "project the selected point")
	c := s.CreateReferencePoint(cx, cy, "selected centre point")
	proofkit.Step(t, "draw the Anchor Line")
	a := s.CreatePoint(cx-5, cy)
	b := s.CreatePoint(cx+5, cy)
	line := s.CreateLine(a, b)
	proofkit.Step(t, "constrain the Anchor Line")
	s.AddConstraint(sketch.NewMidpoint(c, line), sketch.NewHorizontal(line))
	s.AddConstraint(sketch.NewHorizontalDistance(a, b, 10))

	sketchtest.Solve(t, s)
	// e is read from start to end: it must come out along +x, 10 mm long,
	// centred on the projected point. Exact arithmetic on seeds that already
	// satisfy every row; the slack is rounding.
	sketchtest.MeasuresPoint(t, a, cx-5, cy, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, b, cx+5, cy, sketchtest.Within(1e-9))
	for _, con := range s.Constraints() {
		sketchtest.Satisfies(t, con, sketchtest.Within(1e-9))
	}
}

// ---------------------------------------------------------------------------
// The Paths sketch (§1), one per gear on its Axis Plane.

// stepPathsSketch builds a gear's Paths sketch on its Axis Plane, in that
// plane's coordinates: x along e and y along k, origin at the foot of C. The
// gear's axis lies in the plane, so the four reference points are
// s*(dir.e, dir.k) for s = -sOut, -sIn, sIn, sOut, and the two solid lines run
// from each span's negative station to its positive one, sharing the points,
// which are fixed after both lines exist.
func stepPathsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	sIn, sOut := in.SIn(), in.SOut()
	proofkit.Step(t, "check the frame of §1 for %s", g.label)
	cgCheckFrame(t, in)
	// The axis lies in the Axis Plane: its origin is A/2 below (gear A) or
	// above (gear B) C along n, and its direction has no n component.
	sketchtest.Measures(t, "the axis origin's height along n", g.origin.Sub(cgC).Dot(cgN),
		[2]float64{-in.A() / 2, in.A() / 2}[g.index], sketchtest.Within(1e-12))
	sketchtest.Measures(t, "the axis direction's n component", g.dir.Dot(cgN), 0, sketchtest.Within(1e-12))
	if !(sIn > 0) {
		t.Fatalf("sIn is %.4f mm; the build's first sleeve check refuses this input", sIn)
	}

	proofkit.Step(t, "draw the two bore lines")
	dx, dy := g.dir.Dot(cgE), g.dir.Dot(cgK)
	pt := func(st float64) *sketch.Point { return s.CreatePoint(st*dx, st*dy) }
	pm, pi, qi, qm := pt(-sOut), pt(-sIn), pt(sIn), pt(sOut)
	minus := s.CreateLine(pm, pi) // bore-
	plus := s.CreateLine(qi, qm)  // bore+
	for _, q := range []*sketch.Point{pm, pi, qi, qm} {
		s.Fix(q)
	}

	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "the bore- line's length", minus.Length(), sOut-sIn, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, "the bore+ line's length", plus.Length(), sOut-sIn, sketchtest.WithinRel(1e-12))
	// Each line starts at its negative end and runs along +dir.
	sketchtest.MeasuresPoint(t, minus.Start, -sOut*dx, -sOut*dy, sketchtest.Within(1e-12))
	sketchtest.MeasuresPoint(t, plus.End, sOut*dx, sOut*dy, sketchtest.Within(1e-12))
	if cgMapKey(in) == cgMapKey(cgDefaults()) {
		// The spec's numbers at the defaults: 7.683 and 19 mm.
		sketchtest.Measures(t, "sIn at the defaults", sIn, 7.683, sketchtest.Within(5e-4))
		sketchtest.Measures(t, "sOut at the defaults", sOut, 19, sketchtest.Within(1e-12))
	}
}

// cgCheckFrame holds the frame of §1 to the mechanism proof's own pair: both
// gears' origins, axes and section frames agree, so every number the
// mechanism proof measures is a number about the parts these steps build.
func cgCheckFrame(t testing.TB, in cgIn) {
	t.Helper()
	pa, pb := pair(in.cgParams(), in.Sigma, 0, in.Phase)
	for i, ref := range []Gear{pa, pb} {
		g := cgGears(in)[i]
		for _, pr := range []struct {
			what      string
			got, want r3.Vec
		}{{"origin", g.origin, ref.Origin}, {"axis", g.dir, ref.Ez}, {"u", g.u, ref.Ex}, {"v", g.v, ref.Ey}} {
			sketchtest.Measures(t, g.label+" "+pr.what+" against the mechanism proof's",
				pr.got.Sub(pr.want).Len(), 0, sketchtest.Within(1e-12))
		}
		// The section angle and the tooth phase are the mechanism proof's too.
		for _, st := range []float64{-40, 0, 13.7} {
			sketchtest.Measures(t, g.label+" section angle", g.theta(st), ref.angle(st), sketchtest.Within(1e-12))
			sketchtest.Measures(t, g.label+" toothed edge", g.uTooth(st), ref.edge(st), sketchtest.Within(1e-12))
		}
	}
}

// ---------------------------------------------------------------------------
// The Cell Sections sketch (§2) and the Cell Remainder sketch (§3).
//
// What is substituted. Fusion draws every section of the cell in one sketch,
// each section's corners at their own height off the sketch's plane, and
// reads it fully constrained with one profile per section
// ([SCREW-F-CELL-LOFT], measured 2026-09-28 at 11, 41 and 81 sections). The
// sketch engine is planar, so the proof lays the sections side by side in one
// plane, section k moved along x by k times a spacing wider than any section,
// each its four corners as fixed points and its four solid lines. That pins
// each section's numbers, the section count and the one-profile-per-section
// count; it cannot see Fusion's verdict on the off-plane points, which is the
// step's [PROSE] part.

// cgSideBySide draws sections of gear g at the given stations, each moved by
// its index times a spacing wider than any section, and checks the profile
// count and each section's area.
func cgSideBySide(t testing.TB, s *sketch.Sketch, g cgGear, stations []float64) {
	t.Helper()
	in := g.in
	gap := 2*math.Hypot(in.W/2, in.T/2) + 2
	for k, st := range stations {
		var corners [][2]float64
		for _, q := range g.cellCorners(st) {
			corners = append(corners, [2]float64{q[0] + float64(k)*gap, q[1]})
		}
		cgFixedLoop(s, corners)
	}
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	profiles := report.Profiles
	if len(profiles) != len(stations) {
		t.Fatalf("%s: %d profiles for %d sections; Fusion's count check would raise", g.label,
			len(profiles), len(stations))
	}
	for _, prof := range profiles {
		sketchtest.IsValidProfile(t, prof)
		k := int(math.Round(cgProfileX(prof) / gap))
		st := stations[k]
		// The section is the rectangle (Utooth + W/2) by T, exactly; the slack is
		// rounding.
		sketchtest.MeasuresProfileArea(t, prof, (g.uTooth(st)+in.W/2)*in.T, sketchtest.WithinRel(1e-9))
	}
}

// cgProfileX is the mean x of a profile's outer corners.
func cgProfileX(prof *sketch.Profile) float64 {
	sum, n := 0.0, 0
	for _, e := range prof.Outer {
		sum += e.Polyline[0][0]
		n++
	}
	return sum / float64(n)
}

// stepCellSectionsSketch builds the stand-in for a gear's Cell Sections
// sketch: c*n + 1 sections at s_k = s0 + k*P/n, s0 = Z0 - L/2.
func stepCellSectionsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	c, n := in.Cell(), in.StepsPerTooth()
	proofkit.Step(t, "%s: %d sections, %d steps to the tooth", g.label, c*n+1, n)
	stations := make([]float64, c*n+1)
	for k := range stations {
		stations[k] = g.cgStation(k)
	}
	cgSideBySide(t, s, g, stations)
	if in.Lead == 49.5 && in.P == 2.625 {
		// The spec's counts at the defaults: 10 steps to the tooth, 41 sections
		// in a four-tooth cell and 11 in a one-tooth cell.
		if n != 10 || (c == 4 && len(stations) != 41) || (c == 1 && len(stations) != 11) {
			t.Fatalf("%d steps and %d sections at the defaults' lead and pitch", n, len(stations))
		}
	}
	if in.Lead == 400 && n != 8 {
		t.Fatalf("at a 400 mm lead the floor of eight steps should set the count, got %d", n)
	}
}

// stepCellRemainderSketch builds the stand-in for a gear's Cell Remainder
// sketch: r*n + 1 sections from s0 + q*c*P to s0 + N*P, r = N mod c.
func stepCellRemainderSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	g := cgGears(in)[int(p["gear"])]
	c, n := in.Cell(), in.StepsPerTooth()
	q, r := in.N/c, in.N%c
	if r == 0 {
		t.Fatalf("%d teeth in %d-tooth cells leave no remainder; the case table is wrong", in.N, c)
	}
	proofkit.Step(t, "%s: remainder of %d teeth after %d cells", g.label, r, q)
	from := g.s0() + float64(q*c)*in.P
	stations := make([]float64, r*n+1)
	for k := range stations {
		stations[k] = from + float64(k)*in.P/float64(n)
	}
	cgSideBySide(t, s, g, stations)
	sketchtest.Measures(t, "the remainder's last station", stations[len(stations)-1], g.s0()+float64(in.N)*in.P,
		sketchtest.Within(1e-9))
}

// ---------------------------------------------------------------------------
// The Sleeve sketch (§4).

// stepSleeveSketch builds the Sleeve sketch on the selected plane: two
// circles at C of radii Ri and Ro, each centre a separate point fixed after
// its circle exists, each circle held by a diameter. The ring is the one
// profile with two loops.
func stepSleeveSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	proofkit.Step(t, "draw the inner and outer circles")
	for _, r := range []float64{in.Ri(), in.Ro()} {
		centre := s.CreatePoint(0, 0)
		circle := s.CreateCircle(centre, r)
		s.Fix(centre)
		s.AddConstraint(sketch.NewDiameter(circle, 2*r))
	}
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	var rings []*sketch.Profile
	for _, prof := range report.Profiles {
		sketchtest.IsValidProfile(t, prof)
		if len(prof.Holes) == 1 {
			rings = append(rings, prof)
		}
	}
	if len(report.Profiles) != 2 || len(rings) != 1 {
		t.Fatalf("%d profiles with %d of two loops; the build raises unless exactly one has two",
			len(report.Profiles), len(rings))
	}
	// pi*(Ro^2 - Ri^2) is the ring's area exactly; the slack is rounding. At
	// the defaults it is the spec's 565.5 mm^2.
	sketchtest.MeasuresProfileArea(t, rings[0], math.Pi*(in.Ro()*in.Ro()-in.Ri()*in.Ri()), sketchtest.WithinRel(1e-9))
}

// ---------------------------------------------------------------------------
// The bore section sketch (§4, "The rectangle scheme").

// cgWrap folds an angle into (-pi, pi].
func cgWrap(a float64) float64 {
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a <= 0 {
		a += 2 * math.Pi
	}
	return a - math.Pi
}

// cgBoreStation is the station a bore's section sketch stands at: the cut's
// first station, -sOut for a -R bore and sIn for a +R bore.
func cgBoreStation(in cgIn, bore int) float64 {
	if bore%2 == 0 {
		return -in.SOut()
	}
	return in.SIn()
}

// cgBoreScheme draws the rectangle scheme in the bore plane's coordinates (x
// along the gear's unrotated u, y along v, O at the origin) and returns the
// rectangle's lines and the dimensioned angle's branch: true when the angle is
// taken against the spine K, false when against the toothed side L2.
func cgBoreScheme(s *sketch.Sketch, g cgGear, st float64, fix func(*sketch.Point)) ([4]*sketch.Line, bool, *sketch.Point) {
	in := g.in
	uB, uF, hv := -in.Hw(), in.Hw(), in.Ht()
	o := s.CreatePoint(0, 0)
	cp := s.CreatePoint(in.A()/2, 0)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	ex, ey := g.turn(uF, 0, st)
	e := s.CreatePoint(ex, ey)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	fix(o)
	fix(cp)
	var pts [4]*sketch.Point
	for i, q := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		x, y := g.turn(q[0], q[1], st)
		pts[i] = s.CreatePoint(x, y)
	}
	var l [4]*sketch.Line
	for i := range 4 {
		l[i] = s.CreateLine(pts[i], pts[(i+1)%4])
	}
	psi := cgWrap(g.theta(st))
	onSpine := math.Abs(math.Sin(psi)) >= math.Sqrt(0.5)
	var angle *sketch.Angle
	if onSpine {
		angle = sketch.NewAngle(ru, k, psi*180/math.Pi)
	} else {
		angle = sketch.NewAngle(ru, l[1], cgWrap(psi+math.Pi/2)*180/math.Pi)
	}
	// The engine's Offset is Fusion's parallel plus offset dimension: it drives
	// the second line to the first's parallel at a signed distance, positive
	// on the left of the first's start-to-end direction. L1 is on K's right,
	// L3 on its left, and L4 on L2's left.
	s.AddConstraint(
		sketch.NewDistance(o, e, uF),
		angle,
		sketch.NewOffset(k, l[0], -hv),
		sketch.NewOffset(k, l[2], hv),
		sketch.NewPointOnLine(e, l[1]),
		sketch.NewPerpendicular(l[1], k),
		sketch.NewOffset(l[1], l[3], uF-uB),
	)
	return l, onSpine, e
}

// stepBoreSectionSketch builds one bore's section sketch, {gearLabel} Bore
// {-R|+R}, by the rectangle scheme, at the cut's first station.
func stepBoreSectionSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	bore := int(p["bore"])
	g := cgGears(in)[bore/2]
	st := cgBoreStation(in, bore)
	proofkit.Step(t, "%s at station %.4f", cgBoreNames[bore], st)
	// O, the point where the bore's line pierces the plane, is the line's own
	// start: on the gear's axis at the station.
	sketchtest.Measures(t, "O's distance from the gear's axis",
		g.axisPoint(st).Sub(g.origin).Cross(g.dir).Len(), 0, sketchtest.Within(1e-12))
	lines, onSpine, e := cgBoreScheme(s, g, st, s.Fix)
	proofkit.Step(t, "the angle is taken against %s", map[bool]string{true: "K", false: "L2"}[onSpine])

	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, prof)
	// The profile is the one loop of four lines the build finds with
	// find_profile_by_curve_counts(sketch, lines=4).
	if len(prof.Outer) != 4 || len(prof.Holes) != 0 {
		t.Fatalf("the bore's profile has %d outer edges and %d holes, want 4 and 0", len(prof.Outer), len(prof.Holes))
	}
	for _, e := range prof.Outer {
		if _, ok := e.Entity.(*sketch.Line); !ok || e.Entity.IsConstruction() {
			t.Fatalf("the bore's profile is bounded by something other than its four solid lines")
		}
	}
	// The rectangle is 2hw by 2ht exactly; the slack is rounding.
	sketchtest.MeasuresProfileArea(t, prof, 4*in.Hw()*in.Ht(), sketchtest.WithinRel(1e-9))
	for i, q := range g.boreCorners(st) {
		sketchtest.MeasuresPoint(t, lines[i].Start, q[0], q[1], sketchtest.Within(1e-7))
	}
	x, y := g.turn(in.Hw(), 0, st)
	sketchtest.MeasuresPoint(t, e, x, y, sketchtest.Within(1e-7))
	for _, con := range s.Constraints() {
		sketchtest.Satisfies(t, con, sketchtest.Within(1e-7))
	}
}

// ---------------------------------------------------------------------------
// The window sketches (§4).

// cgSleeveCache holds the sleeve searches per input, so a case table that
// reaches one input several times runs its searches once.
var cgSleeveCache sync.Map

// cgSleeve is the sleeve the build would make at these inputs: the window
// search of §4 is the mechanism proof's newSleeve, step for step, and the
// compiled proof takes its windows as numbers.
func cgSleeve(in cgIn) sleeve {
	key := cgMapKey(in)
	if f, ok := cgSleeveCache.Load(key); ok {
		return f.(sleeve)
	}
	p := in.cgParams()
	ga, gb := pair(p, p.Sigma(), 0, in.Phase)
	f := newSleeve(ga, gb)
	cgSleeveCache.Store(key, f)
	return f
}

func cgMapKey(in cgIn) cgIn { in.Phase = 0; in.N = 0; in.CellTeeth = 0; return in }

// cgWindow is the window facing the case's direction, the first or second of
// the two facings (d before -d), or ok false when the search found it no room.
func cgWindow(in cgIn, which int) (sleeveWindow, bool) {
	f := cgSleeve(in)
	facing := windowFacings(in.cgParams())[which]
	for _, w := range f.windows {
		if w.facing == facing {
			return w, true
		}
	}
	return sleeveWindow{}, false
}

// cgHexagon is the window's corners with the build's rule applied: walking
// the clipped corners in order, a corner within 0.001 mm of the last one kept
// is dropped, and so is the last one kept when it lies within 0.001 mm of the
// first, since a sketch line cannot have zero length.
func cgHexagon(w sleeveWindow) [][2]float64 {
	var out [][2]float64
	for _, q := range w.corners {
		if n := len(out); n > 0 && math.Hypot(q[0]-out[n-1][0], q[1]-out[n-1][1]) < 0.001 {
			continue
		}
		out = append(out, q)
	}
	for len(out) > 1 {
		a, b := out[len(out)-1], out[0]
		if math.Hypot(a[0]-b[0], a[1]-b[1]) >= 0.001 {
			break
		}
		out = out[:len(out)-1]
	}
	return out
}

// stepWindowSketch builds one window's sketch, Window {d}, on the Window
// Plane in the plane's coordinates (t along across = n x d, z along n, origin
// C): one fixed reference point per hexagon corner and a solid line from each
// to the next.
func stepWindowSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := cgRead(t, p)
	which := int(p["window"])
	w, ok := cgWindow(in, which)
	if !ok {
		t.Fatalf("the window search found no room for window %d at an input TestSleeveWindowsFollowTheSize says cuts both", which)
	}
	// The Window Plane holds C and n and stands square to d.
	sketchtest.Measures(t, "across against n x d", w.across.Sub(cgN.Cross(w.facing)).Len(), 0, sketchtest.Within(1e-12))
	sketchtest.Measures(t, "d's n component", w.facing.Dot(cgN), 0, sketchtest.Within(1e-12))
	hex := cgHexagon(w)
	proofkit.Step(t, "draw the window's %d corners", len(hex))
	cgFixedLoop(s, hex)

	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, prof)
	// The shoelace area of the clipped corners; the dropped corners are
	// within 0.001 mm of a kept one, so the area moves by less than 0.001 mm
	// times the hexagon's perimeter.
	perimeter := 0.0
	for i := range w.corners {
		a, b := w.corners[i], w.corners[(i+1)%len(w.corners)]
		perimeter += math.Hypot(b[0]-a[0], b[1]-a[1])
	}
	sketchtest.MeasuresProfileArea(t, prof, w.area(), sketchtest.Within(0.001*perimeter+1e-9))

	if cgMapKey(in) == cgMapKey(cgDefaults()) {
		cgDefaultWindow(t, w, which)
	}
}

// cgDefaultWindow holds the default windows to the spec's numbers (§4, "The
// hexagon"): the +k window's six lines to the spec's three decimals and its
// corners to its two, and the -k window to the +k one turned half a turn
// about e, which in its own plane coordinates is (t, z) -> (-t, -z).
func cgDefaultWindow(t testing.TB, w sleeveWindow, which int) {
	t.Helper()
	sign := []float64{1, -1}[which]
	if which == 0 {
		for _, c := range []struct {
			what      string
			got, want float64
		}{{"lo", w.lo, -3.335}, {"hi", w.hi, 6.495}, {"bottom", w.bottom, -20.306}, {"top", w.top, 23.466},
			{"left", w.left, -11.404}, {"right", w.right, 11.663}} {
			sketchtest.Measures(t, "the +k window's "+c.what, c.got, c.want, sketchtest.Within(5e-4))
		}
	}
	spec := [][2]float64{{11.66, -5.17}, {-8.49, 14.98}, {-11.40, 12.06}, {-11.40, 8.07}, {8.49, -11.82}, {11.66, -8.64}}
	hex := cgHexagon(w)
	if len(hex) != len(spec) {
		t.Fatalf("the default window has %d corners, the spec six", len(hex))
	}
	for _, q := range spec {
		best := math.Inf(1)
		for _, h := range hex {
			best = math.Min(best, math.Hypot(h[0]-sign*q[0], h[1]-sign*q[1]))
		}
		sketchtest.Measures(t, "the nearest corner to the spec's", best, 0, sketchtest.Within(0.0051))
	}
}
