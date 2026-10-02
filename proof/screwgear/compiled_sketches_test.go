package screwgear_test

// The sketch steps of spec/screwgear/steps.md, one function each. Every
// sketch here is gated by proofkit on the engine's own verification verdict;
// what each function adds is the numbers the step pins, measured on the
// sketch it built.
//
// Fusion's isFixed is the engine's Fix, and a projection is a reference point
// (spec/screwgear/fusion.md [SCREW-F-REFERENCES]). Fusion's addParallel plus
// addOffsetDimension on one pair of lines is the engine's NewOffset, whose
// signed distance carries the side Fusion takes from the seed. Fusion's
// unsigned angular dimension, whose wedge the text point picks, is the
// engine's signed NewAngle from the first line's direction to the second's.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// requireAccepted fails a case whose inputs the build would refuse before
// drawing anything, so that no case table proves a sketch for an input the
// dialog does not let through. sleeveRefusal is the build's own four sleeve
// checks in order.
func requireAccepted(t testing.TB, p Params, m map[string]float64) {
	t.Helper()
	if why := sleeveRefusal(p); why != "" {
		t.Fatalf("the case's inputs are refused by the build (%s); a case table holds only accepted inputs", why)
	}
	if ph := casePhase(m); math.Abs(ph) >= p.ToothPitch {
		t.Fatalf("the case's assembly phase %.3f mm is not strictly within one tooth pitch", ph)
	}
	if p.ToothHeight >= p.Width/2 || p.Engagement > p.ToothHeight {
		t.Fatalf("the case's tooth height or engagement is out of the dialog's range")
	}
}

// stepAnchorSketch is the Anchor sketch: the selected point projected in, and
// the Anchor Line bisected by it, horizontal, 10 mm from start to end along
// the sketch's x axis. The engine's midpoint carries the coincident row, so
// the proof writes the midpoint alone where Fusion is given both.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	cx, cy := m["centreX"], m["centreY"]
	proofkit.Step(t, "project the selected point")
	centre := s.CreateReferencePoint(cx, cy, "selected point")

	proofkit.Step(t, "draw the Anchor Line from seeds 5 mm either side")
	start := s.CreatePoint(cx-5, cy)
	end := s.CreatePoint(cx+5, cy)
	line := s.CreateLine(start, end)
	mid := sketch.NewMidpoint(centre, line)
	horizontal := sketch.NewHorizontal(line)
	length := sketch.NewHorizontalDistance(start, end, 10)
	s.AddConstraint(mid, horizontal, length)

	proofkit.Step(t, "read the frame")
	sketchtest.Solve(t, s)
	for _, c := range []sketch.Constraint{mid, horizontal, length} {
		sketchtest.Satisfies(t, c, sketchtest.Within(1e-9))
	}
	// ê runs from the start to the end, along +x: only the seeded orientation
	// satisfies a signed horizontal distance, which is why the dimension is
	// horizontal rather than aligned. The points are solved exactly.
	sketchtest.MeasuresPoint(t, start, cx-5, cy, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, end, cx+5, cy, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "Anchor Line length", line.Length(), 10, sketchtest.Within(1e-9))
}

// pathStations are the four stations of a gear's Paths sketch, -sOut, -sIn,
// sIn and sOut, with sIn = sqrt(Ri^2 - c^2) - 1 mm and sOut = Ro + 1 mm.
func pathStations(p Params) [4]float64 {
	ri, ro, c := p.SleeveInner(), p.SleeveOuter(), p.BoreCorner()
	sIn, sOut := math.Sqrt(ri*ri-c*c)-1, ro+1
	return [4]float64{-sOut, -sIn, sIn, sOut}
}

// stepGearPathsSketch is a gear's Paths sketch on its Axis Plane: four fixed
// reference points on the gear's axis and two solid lines between them, each
// drawn from its negative station to its positive one. The Axis Plane is
// parallel to the selected plane, so its coordinates here are the selected
// plane's: x along ê and y along k̂, with C at (centreX, centreY), where the
// gear's axis runs through C's foot at +Sigma/2 (gear A) or -Sigma/2 (gear B)
// from ê.
func stepGearPathsSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	p, g := caseGear(m)
	requireAccepted(t, p, m)
	cx, cy := m["centreX"], m["centreY"]
	dx, dy := g.Ez.X, g.Ez.Y // the axis is level, so its direction lies in the plane
	st := pathStations(p)
	if st[2] <= 0 {
		t.Fatalf("sIn is %.4f mm, at or before the middle: the two lines would overlap", st[2])
	}

	proofkit.Step(t, "four reference points on the axis")
	var pts [4]*sketch.Point
	for i, sv := range st {
		pts[i] = s.CreatePoint(cx+sv*dx, cy+sv*dy)
	}
	proofkit.Step(t, "the bore- and bore+ lines")
	minus := s.CreateLine(pts[0], pts[1])
	plus := s.CreateLine(pts[2], pts[3])
	for _, pt := range pts {
		s.Fix(pt)
	}

	sketchtest.Solve(t, s)
	span := st[3] - st[2]
	sketchtest.Measures(t, "bore- line length", minus.Length(), span, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "bore+ line length", plus.Length(), span, sketchtest.Within(1e-9))
	// Each line runs along +dir_g, from its start, which is where its bore's
	// plane and profile sit.
	for _, l := range []*sketch.Line{minus, plus} {
		along := (l.End.X()-l.Start.X())*dx + (l.End.Y()-l.Start.Y())*dy
		sketchtest.Measures(t, "line run along the gear's axis", along, span, sketchtest.Within(1e-9))
	}
	if m["quoted"] == 1 {
		// The defaults the spec quotes: 7.967 and 19 mm, to the digits quoted.
		sketchtest.Measures(t, "sIn at the defaults", st[2], 7.967, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "sOut at the defaults", st[3], 19, sketchtest.Within(1e-9))
	}
}

// stepCellSectionsSketch is the Cell Sections sketch, by its stand-in. Fusion
// holds every section of the cell in one sketch on the Gear Axis Plane, each
// at its true position off that plane; the sketch engine is planar, so this
// draws each section in its own station's plane coordinates (x along û_g, y
// along v̂_g, the axis at the origin) and lays them side by side along x, far
// enough apart that no two touch. Each is four fixed points and four lines,
// exactly the Fusion recipe's, so the section's numbers are pinned: its
// corners, its area, and that the sketch finds one profile per section and
// teeth*n + 1 in all. What the stand-in cannot show is Fusion's own verdict on
// a sketch whose points lie off its plane; that was measured on 2026-09-28 at
// 11, 41 and 81 sections (spec/screwgear/fusion.md [SCREW-F-CELL-LOFT]).
func stepCellSectionsSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	p, g := caseGear(m)
	requireAccepted(t, p, m)
	n := cellSteps(p)
	teeth, first := int(m["teeth"]), int(m["firstTooth"])
	count := teeth*n + 1
	spacing := 2*p.Width + 10

	proofkit.Step(t, "draw %d sections side by side", count)
	section := map[*sketch.Line]int{}
	var points []*sketch.Point
	want := make([]float64, count)
	for k := range count {
		st := ribbonStation(g, first*n+k)
		theta := g.angle(st)
		uB, uF, hv := -p.Width/2, g.edge(st), p.Thickness/2
		want[k] = (uF - uB) * 2 * hv
		var pts [4]*sketch.Point
		for i, uv := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
			x := uv[0]*math.Cos(theta) - uv[1]*math.Sin(theta)
			y := uv[0]*math.Sin(theta) + uv[1]*math.Cos(theta)
			pts[i] = s.CreatePoint(float64(k)*spacing+x, y)
		}
		for i := range 4 {
			section[s.CreateLine(pts[i], pts[(i+1)%4])] = k
		}
		points = append(points, pts[:]...)
	}
	proofkit.Step(t, "fix every point after the last line")
	for _, pt := range points {
		s.Fix(pt)
	}

	sketchtest.Solve(t, s)
	rep := sketchtest.Verify(t, s)
	if len(rep.Profiles) != count {
		t.Fatalf("the sections make %d profiles, want one per section, %d", len(rep.Profiles), count)
	}
	seen := make([]bool, count)
	for _, pr := range rep.Profiles {
		sketchtest.IsValidProfile(t, pr)
		k := -1
		for _, e := range pr.Entities {
			if l, ok := e.(*sketch.Line); ok {
				if j, ok := section[l]; ok && (k < 0 || k == j) {
					k = j
					continue
				}
			}
			t.Fatalf("a profile is bounded by more than one section's lines")
		}
		if k < 0 || seen[k] {
			t.Fatalf("a profile does not map to one unseen section")
		}
		seen[k] = true
		// T*(Utooth(s_k) + W/2): the rectangle's own area, exact.
		sketchtest.MeasuresProfileArea(t, pr, want[k], sketchtest.WithinRel(1e-9))
	}
	if m["quoted"] == 1 {
		// The defaults: ten steps to the tooth, 41 sections.
		sketchtest.Measures(t, "steps to the tooth at the defaults", float64(n), 10, sketchtest.Within(0))
	}
	if p.TwistLead >= 400 {
		sketchtest.Measures(t, "steps to the tooth at a slow twist", float64(n), 8, sketchtest.Within(0))
	}
}

// stepSleeveSketch is the Sleeve sketch on the selected plane: two circles
// centred on C, radii Ri and Ro, each centre fixed and each circle given its
// diameter. It has two profiles, the disc inside Ri and the ring; the ring is
// the one profile with a hole, which is how the build picks it.
func stepSleeveSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	p := caseParams(m)
	requireAccepted(t, p, m)
	cx, cy := m["centreX"], m["centreY"]
	ri, ro := p.SleeveInner(), p.SleeveOuter()

	proofkit.Step(t, "two circles at C")
	inner := s.CreateCircle(s.CreatePoint(cx, cy), ri)
	outer := s.CreateCircle(s.CreatePoint(cx, cy), ro)
	s.Fix(inner.Center)
	s.Fix(outer.Center)
	dIn, dOut := sketch.NewDiameter(inner, 2*ri), sketch.NewDiameter(outer, 2*ro)
	s.AddConstraint(dIn, dOut)

	sketchtest.Solve(t, s)
	rep := sketchtest.Verify(t, s)
	if len(rep.Profiles) != 2 {
		t.Fatalf("the Sleeve sketch has %d profiles, want 2: the disc and the ring", len(rep.Profiles))
	}
	rings := 0
	for _, pr := range rep.Profiles {
		sketchtest.IsValidProfile(t, pr)
		switch len(pr.Holes) {
		case 1:
			rings++
			sketchtest.MeasuresProfileArea(t, pr, math.Pi*(ro*ro-ri*ri), sketchtest.WithinRel(1e-9))
		case 0:
			sketchtest.MeasuresProfileArea(t, pr, math.Pi*ri*ri, sketchtest.WithinRel(1e-9))
		default:
			t.Errorf("a Sleeve profile has %d holes", len(pr.Holes))
		}
	}
	if rings != 1 {
		t.Fatalf("%d Sleeve profiles have one hole, want exactly 1", rings)
	}
}

// stepBoreSectionSketch is one bore's section sketch by the rectangle scheme,
// in its plane's coordinates: x along the gear's unrotated û, y along v̂, the
// axis point O at (offsetX, offsetY). O and Cp are fixed reference points
// and Ru the construction line between them; the spine K runs from O to E
// with its length uF; one angle turns it; the rectangle hangs off it by two
// offsets, a coincidence, a perpendicular and a third offset. Ten degrees of
// freedom, E and four corners, against ten rows.
func stepBoreSectionSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	p, gs := caseGears(m)
	requireAccepted(t, p, m)
	f := plainSleeve(gs[0], gs[1])
	b := f.bores()[int(m["bore"])]
	station, _ := b.span(f)
	theta := b.g.angle(station)
	ox, oy := m["offsetX"], m["offsetY"]
	uB, uF, hv := -p.BoreHalfWidth(), p.BoreHalfWidth(), p.BoreHalfThickness()
	at := func(u, v float64) (float64, float64) {
		return ox + u*math.Cos(theta) - v*math.Sin(theta), oy + u*math.Sin(theta) + v*math.Cos(theta)
	}

	proofkit.Step(t, "references O and Cp, Ru, and the spine K")
	o := s.CreatePoint(ox, oy)
	cp := s.CreatePoint(ox+p.AxisOffset()/2, oy)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	ex, ey := at(uF, 0)
	e := s.CreatePoint(ex, ey)
	k := s.CreateLine(o, e)
	k.SetConstruction(true)
	s.Fix(o)
	s.Fix(cp)

	proofkit.Step(t, "the rectangle")
	var c [4]*sketch.Point
	for i, uv := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		x, y := at(uv[0], uv[1])
		c[i] = s.CreatePoint(x, y)
	}
	l1 := s.CreateLine(c[0], c[1])
	l2 := s.CreateLine(c[1], c[2])
	l3 := s.CreateLine(c[2], c[3])
	l4 := s.CreateLine(c[3], c[0])

	proofkit.Step(t, "the dimensions and constraints")
	var angle sketch.Constraint
	deg := theta * 180 / math.Pi
	if math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) {
		angle = sketch.NewAngle(ru, k, deg)
	} else {
		angle = sketch.NewAngle(ru, l2, deg+90)
	}
	rows := []sketch.Constraint{
		sketch.NewDistance(o, e, uF),
		angle,
		sketch.NewOffset(k, l1, -hv),
		sketch.NewOffset(k, l3, hv),
		sketch.NewPointOnLine(e, l2),
		sketch.NewPerpendicular(l2, k),
		sketch.NewOffset(l2, l4, uF-uB),
	}
	s.AddConstraint(rows...)

	sketchtest.Solve(t, s)
	for _, r := range rows {
		sketchtest.Satisfies(t, r, sketchtest.Within(1e-9))
	}
	for i, uv := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		x, y := at(uv[0], uv[1])
		sketchtest.MeasuresPoint(t, c[i], x, y, sketchtest.Within(1e-7))
	}
	sketchtest.MeasuresPoint(t, e, ex, ey, sketchtest.Within(1e-7))
	pr := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, pr)
	sketchtest.MeasuresProfileArea(t, pr, (uF-uB)*2*hv, sketchtest.WithinRel(1e-9))

	// The value Fusion writes is the angle between the two named rays, which
	// the reference choice keeps between 45 and 135 degrees.
	var written float64
	if math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) {
		written = math.Acos(math.Cos(theta)) * 180 / math.Pi // ray O->Cp against ray O->E
	} else {
		written = math.Acos(-math.Sin(theta)) * 180 / math.Pi // ray along +û against ray along +v̂(theta)
	}
	if written < 45-1e-9 || written > 135+1e-9 {
		t.Errorf("the angular dimension's value is %.4f degrees, outside 45 to 135", written)
	}
}

// stepWindowSketch is one window's sketch on the Window Plane, in that plane's
// coordinates (t along across = n̂ x d, z along n̂, C at the origin): one fixed
// reference point per hexagon corner and one line from each to the next. Its
// one loop is its one profile.
func stepWindowSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	p := caseParams(m)
	requireAccepted(t, p, m)
	f := caseSleeve(m)
	facing := windowFacings(p)[int(m["window"])]
	var w *sleeveWindow
	for i := range f.windows {
		if f.windows[i].facing.Sub(facing).Len() < 1e-12 {
			w = &f.windows[i]
		}
	}
	if w == nil {
		t.Fatalf("the search finds no room for the window facing %s", faceName(facing))
	}
	corners := windowCorners(*w)

	proofkit.Step(t, "%d corners and the lines between them", len(corners))
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, pt := range pts {
		s.Fix(pt)
	}

	sketchtest.Solve(t, s)
	pr := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, pr)
	sketchtest.MeasuresProfileArea(t, pr, math.Abs(polygonArea(corners)), sketchtest.WithinRel(1e-9))

	if m["quoted"] == 1 {
		// The +k window the spec quotes at the defaults, to the digits quoted.
		sketchtest.Measures(t, "lo", w.lo, -3.780, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "hi", w.hi, 6.730, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "bottom", w.bottom, -20.751, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "top", w.top, 22.355, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "left", w.left, -11.523, sketchtest.Within(0.0005))
		sketchtest.Measures(t, "right", w.right, 11.703, sketchtest.Within(0.0005))
		quoted := [][2]float64{{11.70, -4.97}, {-7.81, 14.54}, {-11.52, 10.83}, {-11.52, 7.74}, {8.49, -12.27}, {11.70, -9.05}}
		if len(corners) != len(quoted) {
			t.Fatalf("the default +k window has %d corners, want %d", len(corners), len(quoted))
		}
		for _, q := range quoted {
			best := math.Inf(1)
			for _, c := range corners {
				best = math.Min(best, math.Hypot(c[0]-q[0], c[1]-q[1]))
			}
			// The spec quotes two decimals.
			sketchtest.Measures(t, "distance to a quoted corner", best, 0, sketchtest.Within(0.0071))
		}
		sketchtest.MeasuresProfileArea(t, pr, 220.0, sketchtest.Within(0.05))
	}
}
