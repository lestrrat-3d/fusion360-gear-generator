package screwgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// ---- case tables -----------------------------------------------------------

// The variants each group of steps runs, chosen for the inputs that step reads.
var (
	// Everything that moves a ribbon's section or its stations.
	csRibbonVariants = []csVariant{
		csVDefaults, csVCellOne, csVLead20, csVLead400, csVPhasePlus, csVPhaseMinus,
		csVPhaseZero, csVMountNeg, csVMountSteep, csVToothTall, csVToothShort,
		csVTeeth15, csVTeeth69,
	}
	// Everything that moves the frame: the crossing angle's accepted ends, the
	// twist, the mount, the clearance, the rod and a cage moved outward.
	csCageVariants = []csVariant{
		csVDefaults, csVCross70, csVCross110, csVLead20, csVLead400, csVMountNeg,
		csVMountUnequal, csVClearanceFine, csVClearanceWide, csVThinRod, csVEngageFull,
		csVPhasePlus,
	}
	// Every variant, for the steps cheap enough to run them all.
	csAllVariants = []csVariant{
		csVDefaults, csVCellOne, csVLead20, csVLead400, csVCross70, csVCross110,
		csVEngageFine, csVEngageFull, csVPhasePlus, csVPhaseMinus, csVPhaseZero,
		csVMountNeg, csVMountUnequal, csVMountSteep, csVToothTall, csVToothShort,
		csVTeeth15, csVTeeth69, csVCellOne15, csVClearanceFine, csVClearanceWide,
		csVThinRod,
	}
)

// csPerGear is one case per variant and gear.
func csPerGear(vs []csVariant) []proofkit.Case {
	var out []proofkit.Case
	for _, v := range vs {
		for g := range 2 {
			extra := map[string]float64{csGearKey: float64(g)}
			if v.name == csVDefaults.name {
				extra[csExpectSections] = 41 // spec §2: 41 sections in a four-tooth cell
			}
			if v.name == csVCellOne.name {
				extra[csExpectSections] = 11 // and 11 in a one-tooth cell
			}
			out = append(out, proofkit.Case{Name: v.name + "/" + csGearLabel(g), Params: csParamsOf(v, extra)})
		}
	}
	return out
}

// csPerIndex is one case per variant and index 0..count-1.
func csPerIndex(vs []csVariant, count int, label string) []proofkit.Case {
	var out []proofkit.Case
	for _, v := range vs {
		for i := range count {
			out = append(out, proofkit.Case{
				Name:   fmt.Sprintf("%s/%s%d", v.name, label, i),
				Params: csParamsOf(v, map[string]float64{csIndexKey: float64(i)}),
			})
		}
	}
	return out
}

func csPerVariant(vs []csVariant) []proofkit.Case {
	var out []proofkit.Case
	for _, v := range vs {
		out = append(out, proofkit.Case{Name: v.name, Params: csParamsOf(v, nil)})
	}
	return out
}

// The projected centre point, placed away from the sketch origin on every
// side, since the Anchor Line is drawn from seeds either side of it.
var csAnchorCases = []proofkit.Case{
	{Name: "at the sketch origin", Params: map[string]float64{"pointX": 0, "pointY": 0}},
	{Name: "positive quadrant", Params: map[string]float64{"pointX": 37.5, "pointY": 12.25}},
	{Name: "negative quadrant", Params: map[string]float64{"pointX": -120, "pointY": -64}},
	{Name: "mixed signs", Params: map[string]float64{"pointX": -8, "pointY": 250}},
}

var (
	csPathsCases     = csPerGear(csAllVariants)
	csSectionCases   = csPerGear(csRibbonVariants)
	csRemainderCases = csPerGear([]csVariant{csVTeeth15, csVTeeth69})
	csRingCases      = csPerVariant([]csVariant{csVDefaults, csVEngageFull})
	csRodsCases      = csPerVariant(csCageVariants)
	csLoopCases      = csPerIndex(csCageVariants, 4, "foot")
	csCollarCases    = csPerIndex(csCageVariants, 4, "collar")
)

// ---- §1 the Anchor sketch --------------------------------------------------

// stepAnchorSketch is the Anchor sketch on the selected plane. The selected
// point is projected in, which the sketch engine models as a reference point:
// it is locked where the projection puts it, as a projection is. Fusion's
// sketch carries addCoincident AND addMidPoint on it; the engine's midpoint
// already carries the coincident row, so the proof writes the midpoint alone,
// as proof/bevelgear does, and the two readings agree.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	cx, cy := csNeed(t, p, "pointX"), csNeed(t, p, "pointY")

	proofkit.Step(t, "project the centre point and draw the Anchor Line from seeds 5 mm either side")
	centre := s.CreateReferencePoint(cx, cy, "selected point")
	start := s.CreatePoint(cx-5, cy)
	end := s.CreatePoint(cx+5, cy)
	line := s.CreateLine(start, end)

	proofkit.Step(t, "midpoint, horizontal, and a horizontal distance of 10 mm from start to end")
	// NewHorizontalDistance is signed (end.x - start.x), which is Fusion's
	// horizontal dimension with its direction captured from the seed: the end
	// stays to the right of the start, and an aligned length would let the line
	// flip end for end, which the gate's probe refuses as ambiguous.
	s.AddConstraint(
		sketch.NewMidpoint(centre, line),
		sketch.NewHorizontal(line),
		sketch.NewHorizontalDistance(start, end, 10),
	)
	csSolve(t, s)
	csSatisfied(t, s)

	// The frame the build reads: C is the projection, ê runs from start to
	// end, 10 mm long, along the sketch's own +x.
	sketchtest.MeasuresPoint(t, start, cx-5, cy, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, end, cx+5, cy, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "Anchor Line length", line.Length(), 10, sketchtest.WithinRel(1e-12))
}

// ---- §1 the Paths sketches -------------------------------------------------

// stepPathsSketch is one gear's Paths sketch on its Axis Plane: eight fixed
// points on the gear's axis and four lines between them, collar-, bore-,
// collar+ and bore+, each drawn from its negative station to its positive one.
//
// SUBSTITUTE: the four lines overlap on one carrier, and the sketch engine
// refuses an arrangement of overlapping collinear segments as invalid profiles
// ("0 of 0 regions"). Fusion reads the same sketch fully constrained with no
// profile, because open lines bound nothing. The proof draws the four lines as
// construction, which the engine keeps out of its region arrangement; that
// costs nothing the step relies on, since no profile is taken from this sketch,
// and the points, the lines' ends and their directions are all still proved.
func stepPathsSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	p := in.P
	gi := int(csNeed(t, m, csGearKey))
	g := csGears(in)[gi]
	e, n, k := csAxes()

	proofkit.Step(t, "%s Axis Plane: offset %+.4f mm along n̂ from the selected plane", csGearLabel(gi), [2]float64{-1, 1}[gi]*p.AxisOffset()/2)
	offset := [2]float64{-1, 1}[gi] * p.AxisOffset() / 2
	planeOrigin := csCentre.Add(n.Scale(offset))
	f := csFrame(t, planeOrigin, e, k)
	// [SCREW-F-NORMAL-SIGN]: C stands A/2 from the axis plane, on +n̂ for gear A.
	sketchtest.Measures(t, "C along n̂ from the axis plane", csCentre.Sub(planeOrigin).Dot(n), -offset, sketchtest.Within(1e-9))

	type span struct {
		name   string
		lo, hi float64
	}
	var spans []span
	for _, sc := range [2]float64{-p.CageRadius, p.CageRadius} {
		sign := map[bool]string{true: "-", false: "+"}[sc < 0]
		spans = append(spans,
			span{"collar" + sign, sc - p.CollarHalf, sc + p.CollarHalf},
			span{"bore" + sign, sc - p.CollarHalf - 1, sc + p.CollarHalf + 1})
	}

	proofkit.Step(t, "eight reference points at origin + s*dir for the eight span ends")
	points := map[float64]*sketch.Point{}
	pointAt := func(st float64) *sketch.Point {
		if pt, ok := points[st]; ok {
			return pt
		}
		x, y := csLocal(t, f, g.Origin.Add(g.Ez.Scale(st)), fmt.Sprintf("the axis point at station %+.4f", st))
		pt := s.CreatePoint(x, y)
		points[st] = pt
		return pt
	}
	lines := make([]*sketch.Line, len(spans))
	for i, sp := range spans {
		lines[i] = s.CreateLine(pointAt(sp.lo), pointAt(sp.hi))
		lines[i].SetConstruction(true) // see SUBSTITUTE above
	}
	if len(points) != 8 {
		t.Fatalf("the four spans share %d distinct stations; the spec draws eight", len(points))
	}
	proofkit.Step(t, "fix all eight points after the last line")
	for _, pt := range points {
		s.Fix(pt)
	}
	csSolve(t, s)

	dx, dy := csLocal(t, csFrame(t, r3.NewVec(0, 0, 0), e, k), g.Ez, "the axis direction")
	for i, sp := range spans {
		l := lines[i]
		sx, sy := csLocal(t, f, g.Origin.Add(g.Ez.Scale(sp.lo)), sp.name+" start")
		sketchtest.MeasuresPoint(t, l.Start, sx, sy, sketchtest.Within(1e-9))
		sketchtest.Measures(t, sp.name+" length", l.Length(), sp.hi-sp.lo, sketchtest.WithinRel(1e-12))
		// The line runs along +dir, so the sweep's positive twist turns the
		// section the way s/Lambda + Phi grows ([SCREW-F-TWISTED-SLOT]).
		along := ((l.End.X()-l.Start.X())*dx + (l.End.Y()-l.Start.Y())*dy) / l.Length()
		sketchtest.Measures(t, sp.name+" direction against +dir", along, 1, sketchtest.Within(1e-12))
	}
}

// ---- §2 the Cell Sections sketch and §3 the remainder ---------------------

// stepCellSectionsSketch is the Cell Sections sketch of one gear.
//
// SUBSTITUTE: Fusion draws every section in one sketch on the Axis Plane with
// its corners off that plane, and the sketch engine is planar, so it cannot
// hold them. The proof draws each section on its own plane, square to the axis
// at that station: four fixed corners and four lines. That pins each section's
// numbers — its station, its turn, its toothed edge — and that each is one
// closed region of four lines. What it cannot pin is Fusion's verdict on the
// single 3-D sketch: fully constrained with one profile per section, measured
// in Fusion on 2026-09-28 at 11, 41 and 81 sections ([SCREW-F-CELL-LOFT]), which
// the build re-checks with isFullyConstrained and profiles.count at run time.
//
// The harness gates the first section in the sketch it hands the step; every
// other section gets a sketch of its own, gated the same way.
func stepCellSectionsSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	csSectionSketches(t, s, m, false)
}

// stepRemainderSketch is the Cell Remainder sketch, drawn only when N mod c is
// not zero: the same recipe as the cell with c replaced by r, at the stations
// from s0 + q*c*P to s0 + N*P. The substitute is the cell's.
func stepRemainderSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	csSectionSketches(t, s, m, true)
}

func csSectionSketches(t testing.TB, first *sketch.Sketch, m map[string]float64, remainder bool) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	p := in.P
	gi := int(csNeed(t, m, csGearKey))
	g := csGears(in)[gi]
	c, q, r := csCellCount(in)
	n := csStepsPerTooth(p)

	from, teeth := csStartStation(g), c
	if remainder {
		if r == 0 {
			t.Fatalf("toothCount %d is a whole number of %d-tooth cells, so the build draws no remainder", p.ToothCount, c)
		}
		from, teeth = csStartStation(g)+float64(q*c)*p.ToothPitch, r
	}
	stations := csStations(p, from, teeth)
	proofkit.Step(t, "%s: %d teeth, %d steps a tooth, %d sections from station %+.4f",
		csGearLabel(gi), teeth, n, len(stations), from)
	if want := teeth*n + 1; len(stations) != want {
		t.Fatalf("%d sections; spec §2 lofts c*n + 1 = %d", len(stations), want)
	}
	if want, ok := m[csExpectSections]; ok && !remainder {
		sketchtest.Measures(t, "section count the spec quotes", float64(len(stations)), want, sketchtest.Within(0))
	}

	for k, st := range stations {
		s := first
		if k > 0 {
			s = proofkit.NewSketch(t)
		}
		f := csSectionFrame(t, g, st)
		corners := csSectionCorners(g, st)
		ps := make([]*sketch.Point, 4)
		for i, w := range corners {
			x, y := csLocal(t, f, w, fmt.Sprintf("section %d corner %d", k, i))
			ps[i] = s.CreatePoint(x, y)
		}
		// L1..L4 share the corners in order; every point is fixed after the
		// last line ([PB-SHARE-XOR-COINCIDENT], [PB-PROJECT-NOT-FIXED] (b)).
		for i := range 4 {
			s.CreateLine(ps[i], ps[(i+1)%4])
		}
		for _, pt := range ps {
			s.Fix(pt)
		}
		csSolve(t, s)
		prof := csOneProfile(t, s, 4, 0)
		// Exact: the region is the rectangle (uF - uB) by T.
		sketchtest.MeasuresProfileArea(t, prof, (g.edge(st)+p.Width/2)*p.Thickness, sketchtest.WithinRel(1e-12))
		if k > 0 {
			proofkit.RequireSound(t, s)
		}
	}
}

// ---- §4 the Ring sketch ----------------------------------------------------

// stepRingSketch is the Ring sketch on the Ring Plane, the plane through the
// Anchor Line square to the selected plane, which holds C, ê and n̂. The proof
// draws it in that plane's (ê, n̂) coordinates.
func stepRingSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	p := in.P
	e, n, _ := csAxes()
	f := csFrame(t, csCentre, e, n)

	proofkit.Step(t, "revolve axis An: construction line from C to C + cageRise*n̂, both ends fixed")
	ax, ay := csLocal(t, f, csCentre, "C")
	bx, by := csLocal(t, f, csCentre.Add(n.Scale(p.CageRise)), "C + cageRise*n̂")
	a := s.CreatePoint(ax, ay)
	b := s.CreatePoint(bx, by)
	axis := s.CreateLine(a, b)
	axis.SetConstruction(true)
	s.Fix(a)
	s.Fix(b)

	proofkit.Step(t, "the wire: a circle at C + cageRise*n̂ + ringRadius*ê, centre fixed, diameter ringWire")
	cx, cy := csLocal(t, f, csCentre.Add(n.Scale(p.CageRise)).Add(e.Scale(p.RingRadius)), "the wire's centre")
	centre := s.CreatePoint(cx, cy)
	wire := s.CreateCircle(centre, p.RingWire/2)
	s.Fix(centre) // [PB-CIRCLE-CENTER]: the centre is fixed, never coincident
	s.AddConstraint(sketch.NewDiameter(wire, p.RingWire))
	csSolve(t, s)
	csSatisfied(t, s)

	rep := sketchtest.Verify(t, s)
	prof := sketchtest.SingleProfile(t, rep) // the build: profiles.count == 1, then item(0)
	sketchtest.IsValidProfile(t, prof)
	sketchtest.MeasuresProfileArea(t, prof, math.Pi*p.RingWire*p.RingWire/4, sketchtest.WithinRel(1e-9))
	// [PB-REVOLVE]: the profile must not reach the axis it revolves about.
	if gap := p.RingRadius - p.RingWire/2; !(gap > 0) {
		t.Fatalf("the ring's wire reaches its axis: ringRadius - ringWire/2 = %v", gap)
	}
}

// ---- §4 the Rods sketch ----------------------------------------------------

// stepRodsSketch is the Rods sketch on the selected plane: four circles on the
// ring's circle at the azimuths the rod search returns, in the search's collar
// order, each centre fixed and each diameter dimensioned.
func stepRodsSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	p := in.P
	e, n, k := csAxes()
	f := csFrame(t, csCentre, e, k)
	gears := csGears(in)
	az := csFootAzimuths(fr)

	// The hand-written frame runs the search in its own axes; the build runs it
	// in the frame the Anchor sketch reads. The crossings must stand at the same
	// azimuths in both, or the rods would be placed against another frame.
	for i := range 4 {
		gi, sc := csCollarStation(p, i)
		cross := gears[gi].Origin.Add(gears[gi].Ez.Scale(sc)).Sub(csCentre)
		got := math.Atan2(cross.Dot(k), cross.Dot(e))
		sketchtest.Measures(t, fmt.Sprintf("collar %d's crossing azimuth", i), math.Remainder(got-fr.cross[i].azimuth, 2*math.Pi), 0, sketchtest.Within(1e-9))
		sketchtest.Measures(t, fmt.Sprintf("collar %d's crossing height", i), cross.Dot(n), [2]float64{-1, 1}[gi]*p.AxisOffset()/2, sketchtest.Within(1e-9))
	}

	proofkit.Step(t, "four circles at C + ringRadius*(cos psi ê + sin psi k̂)")
	circles := make([]*sketch.Circle, 4)
	for i := range 4 {
		x, y := csLocal(t, f, csFoot(p, az[i], 0), fmt.Sprintf("rod %d's foot", i))
		c := s.CreatePoint(x, y)
		circles[i] = s.CreateCircle(c, p.RodDiameter/2)
		s.Fix(c)
		s.AddConstraint(sketch.NewDiameter(circles[i], p.RodDiameter))
	}
	csSolve(t, s)
	csSatisfied(t, s)

	rep := sketchtest.Verify(t, s)
	if len(rep.Profiles) != 4 {
		t.Fatalf("the Rods sketch has %d profiles; the build extrudes exactly 4", len(rep.Profiles))
	}
	for _, prof := range rep.Profiles {
		sketchtest.IsValidProfile(t, prof)
		sketchtest.MeasuresProfileArea(t, prof, math.Pi*p.RodDiameter*p.RodDiameter/4, sketchtest.WithinRel(1e-9))
	}
	if d := fr.nearestRods(); d < p.RodDiameter+p.Clearance {
		t.Fatalf("two rods stand %.4f mm apart, under rodDiameter + clearance", d)
	}

	if m[csRingRadius] == csDefaults()[csRingRadius] && len(csVariantOverrides(m)) == 0 {
		// The spec's logged azimuths at the defaults ("The loop"), and the foot
		// order they give: foot 0 is gear A's +R rod.
		want := [4]float64{254.61, 74.43, 174.61, 354.43}
		for i := range 4 {
			sketchtest.Measures(t, fmt.Sprintf("rod %d azimuth, degrees", i), wrap(az[i])*180/math.Pi, want[i], sketchtest.Within(0.005))
		}
		if order := csFootOrder(az); order != [4]int{1, 2, 0, 3} {
			t.Fatalf("foot order %v; at the defaults it is gear A +R, gear B -R, gear A -R, gear B +R", order)
		}
	}
}

// csVariantOverrides lists the keys of a case whose inputs differ from the
// defaults, so a step can tell the default case from a variant.
func csVariantOverrides(m map[string]float64) []string {
	var out []string
	for k, v := range csDefaults() {
		if m[k] != v {
			out = append(out, k)
		}
	}
	return out
}

// ---- §4 the loop's bar and ball sketches -----------------------------------

// csLoopFoot is foot i of the loop, in foot order, on the Loop Plane.
func csLoopFoot(t testing.TB, in csIn, fr frame, i int) r3.Vec {
	t.Helper()
	az := csFootAzimuths(fr)
	order := csFootOrder(az)
	return csFoot(in.P, az[order[i%4]], -in.P.CageRise)
}

// csBarOutward is m̂ for bar i: the unit vector in the Loop Plane square to the
// bar and pointing away from the loop's centre C - cageRise*n̂.
func csBarOutward(in csIn, a, b r3.Vec) r3.Vec {
	_, n, _ := csAxes()
	centre := csCentre.Sub(n.Scale(in.P.CageRise))
	d, _ := b.Sub(a).Normalize()
	mid := a.Add(b).Scale(0.5).Sub(centre)
	out, _ := mid.Sub(d.Scale(mid.Dot(d))).Normalize()
	return out
}

func csLoopFrame(t testing.TB, in csIn) r3.Frame {
	e, n, k := csAxes()
	return csFrame(t, csCentre.Sub(n.Scale(in.P.CageRise)), e, k)
}

// stepLoopBarSketch is Loop Bar i: one rectangle of four fixed points on the
// Loop Plane, foot_i, foot_j, foot_j + (ringWire/2)*m̂ and foot_i +
// (ringWire/2)*m̂, with B0 from foot_i to foot_j drawn first.
func stepLoopBarSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	f := csLoopFrame(t, in)
	a, b := csLoopFoot(t, in, fr, i), csLoopFoot(t, in, fr, i+1)
	out := csBarOutward(in, a, b)
	w := in.P.RingWire / 2
	_, n, _ := csAxes()
	sketchtest.Measures(t, "m̂ along n̂", out.Dot(n), 0, sketchtest.Within(1e-12))

	proofkit.Step(t, "bar %d: foot %d to foot %d, %.4f mm long", i, i, (i+1)%4, b.Sub(a).Len())
	lines := csPolygon(t, s, f, []r3.Vec{a, b, b.Add(out.Scale(w)), a.Add(out.Scale(w))}, fmt.Sprintf("bar %d", i))
	csSolve(t, s)
	prof := csOneProfile(t, s, 4, 0) // the build: profiles.count == 1, then item(0)
	sketchtest.MeasuresProfileArea(t, prof, b.Sub(a).Len()*w, sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "B0 length", lines[0].Length(), b.Sub(a).Len(), sketchtest.WithinRel(1e-12))
}

// stepLoopBallSketch is Loop Ball i: the chord Bl from foot_i - (ringWire/2)*ê
// to foot_i + (ringWire/2)*ê, a three-point arc from its start through foot_i +
// (ringWire/2)*k̂ to its end, both ends fixed, and the arc's centre coincident
// on Bl.
//
// The engine's arc runs counter-clockwise from its first end to its second
// about a centre it solves for, where Fusion's three-point arc runs through a
// seed. In the Loop Plane's (ê, k̂) coordinates the arc through +k̂ runs
// counter-clockwise from Bl's end to Bl's start, so the proof names the ends in
// that order; the arc is the same curve.
func stepLoopBallSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	f := csLoopFrame(t, in)
	e, _, k := csAxes()
	foot := csLoopFoot(t, in, fr, i)
	w := in.P.RingWire / 2

	proofkit.Step(t, "ball %d: chord Bl across foot %d along ê", i, i)
	sx, sy := csLocal(t, f, foot.Sub(e.Scale(w)), "Bl start")
	ex, ey := csLocal(t, f, foot.Add(e.Scale(w)), "Bl end")
	fx, fy := csLocal(t, f, foot, "the foot")
	tx, ty := csLocal(t, f, foot.Add(k.Scale(w)), "the arc's through point")
	start := s.CreatePoint(sx, sy)
	end := s.CreatePoint(ex, ey)
	chord := s.CreateLine(start, end)
	centre := s.CreatePoint(fx, fy)
	arc := s.CreateArc(centre, end, start)
	s.Fix(start)
	s.Fix(end)
	s.AddConstraint(sketch.NewPointOnLine(arc.Center, chord))
	csSolve(t, s)
	csSatisfied(t, s)

	sketchtest.MeasuresPoint(t, arc.Center, fx, fy, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "arc radius", arc.R(), w, sketchtest.WithinRel(1e-9))
	// The arc passes through the through point: its middle is there.
	mid := arc.StartAngle() + arc.Sweep()/2
	sketchtest.Measures(t, "arc middle x", fx+w*math.Cos(mid), tx, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "arc middle y", fy+w*math.Sin(mid), ty, sketchtest.Within(1e-9))
	prof := csOneProfile(t, s, 1, 1) // find_profile_by_curve_counts(sketch, lines=1, arcs=1)
	sketchtest.MeasuresProfileArea(t, prof, math.Pi*w*w/2, sketchtest.WithinRel(1e-9))
}

// ---- §4 the collar and bore section sketches -------------------------------

// stepCollarSketch is one collar's section by the rectangle scheme of §4, its
// rectangle as construction, with the rounded outline collarWall outside it.
func stepCollarSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	csRectangleScheme(t, s, m, true)
}

// stepBoreSketch is one bore's section by the same scheme, its rectangle solid.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, m map[string]float64) {
	csRectangleScheme(t, s, m, false)
}

// csSchemeStation is the station a collar's or bore's section plane stands at:
// the negative end of its span, sc - collarHalf, a millimetre further for a bore.
func csSchemeStation(p Params, sc float64, collar bool) float64 {
	if collar {
		return sc - p.CollarHalf
	}
	return sc - p.CollarHalf - 1
}

// csAngleBranch says which reference the scheme's angle is taken against, and
// the unsigned value Fusion's angular dimension carries: against the spine K
// when |sin theta| >= sqrt(1/2), where the value is the angle between the rays
// O→Cp and O→E; against the toothed side L2 otherwise, where it is the angle
// between Ru's direction and L2's start→end direction. Both land in 45°–135°.
func csAngleBranch(theta float64) (againstSpine bool, value float64) {
	if math.Abs(math.Sin(theta)) >= math.Sqrt(0.5) {
		return true, math.Acos(math.Cos(theta))
	}
	return false, math.Acos(-math.Sin(theta))
}

func csRectangleScheme(t testing.TB, s *sketch.Sketch, m map[string]float64, collar bool) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	p := in.P
	i := int(csNeed(t, m, csIndexKey))
	gi, sc := csCollarStation(p, i)
	g := csGears(in)[gi]
	st := csSchemeStation(p, sc, collar)
	f := csSectionFrame(t, g, st)
	theta := g.angle(st)
	uB, uF, hv := -(p.Width/2 + p.Clearance), p.Width/2+p.Clearance, p.Thickness/2+p.Clearance
	w := p.CollarWall

	proofkit.Step(t, "%s section at station %+.4f, theta %.4f degrees", csGearLabel(gi), st, theta*180/math.Pi)
	// The seeds are the solved points: (u, v) turned by theta in the plane's
	// (û, v̂) coordinates, which is the world point mapped in.
	at := func(u, v float64, what string) *sketch.Point {
		x, y := csLocal(t, f, g.world(u, v, st), what)
		return s.CreatePoint(x, y)
	}

	proofkit.Step(t, "references O and Cp, the construction line Ru, the spine K from O to E")
	ox, oy := csLocal(t, f, g.Origin.Add(g.Ez.Scale(st)), "O")
	cpx, cpy := csLocal(t, f, g.Origin.Add(g.Ez.Scale(st)).Add(g.Ex.Scale(p.AxisOffset()/2)), "Cp")
	o := s.CreatePoint(ox, oy)
	cp := s.CreatePoint(cpx, cpy)
	ru := s.CreateLine(o, cp)
	ru.SetConstruction(true)
	eP := at(uF, 0, "E")
	spine := s.CreateLine(o, eP)
	spine.SetConstruction(true)
	s.Fix(o)
	s.Fix(cp)

	proofkit.Step(t, "the rectangle L1..L4 sharing its corners")
	c0, c1, c2, c3 := at(uB, -hv, "corner (uB,-hv)"), at(uF, -hv, "corner (uF,-hv)"), at(uF, hv, "corner (uF,hv)"), at(uB, hv, "corner (uB,hv)")
	l1, l2, l3, l4 := s.CreateLine(c0, c1), s.CreateLine(c1, c2), s.CreateLine(c2, c3), s.CreateLine(c3, c0)
	rect := []*sketch.Line{l1, l2, l3, l4}
	if collar {
		for _, l := range rect {
			l.SetConstruction(true)
		}
	}

	proofkit.Step(t, "length of K, the angle, the offsets, the perpendicular and E on L2")
	// Each Fusion offset dimension comes with the parallel it needs: L1 and L3
	// get addParallel to K, L4 to L2. The engine's NewOffset holds both of a
	// line's ends at a signed distance, which is the parallel and the offset
	// together, two rows, as Fusion's pair is. The sign is Fusion's seed side
	// ([PB-DIM-VALUE-SEMANTICS]): L1 lies right of K, L3 left of it, L4 left of L2.
	spineAngle, value := csAngleBranch(theta)
	deg := theta * 180 / math.Pi
	var angle *sketch.Angle
	if spineAngle {
		angle = sketch.NewAngle(ru, spine, deg)
	} else {
		angle = sketch.NewAngle(ru, l2, deg+90)
	}
	s.AddConstraint(
		sketch.NewDistance(o, eP, uF),
		angle,
		sketch.NewOffset(spine, l1, -hv),
		sketch.NewOffset(spine, l3, hv),
		sketch.NewPointOnLine(eP, l2),
		sketch.NewPerpendicular(l2, spine),
		sketch.NewOffset(l2, l4, uF-uB),
	)

	var outline []*sketch.Line
	var arcs []*sketch.Arc
	if collar {
		proofkit.Step(t, "the outline: O1..O4 collarWall outside L1..L4, a tangent arc at each corner")
		// Oi runs the way Li does, from the neighbouring construction side's
		// line to the next one's; the arc from the end of Oi to the start of
		// O(i+1) turns about the construction corner.
		o1 := s.CreateLine(at(uB, -hv-w, "O1 start"), at(uF, -hv-w, "O1 end"))
		o2 := s.CreateLine(at(uF+w, -hv, "O2 start"), at(uF+w, hv, "O2 end"))
		o3 := s.CreateLine(at(uF, hv+w, "O3 start"), at(uB, hv+w, "O3 end"))
		o4 := s.CreateLine(at(uB-w, hv, "O4 start"), at(uB-w, -hv, "O4 end"))
		outline = []*sketch.Line{o1, o2, o3, o4}
		corners := [][2]float64{{uF, -hv}, {uF, hv}, {uB, hv}, {uB, -hv}}
		for j := range 4 {
			centre := at(corners[j][0], corners[j][1], fmt.Sprintf("arc %d centre seed", j))
			arcs = append(arcs, s.CreateArc(centre, outline[j].End, outline[(j+1)%4].Start))
		}
		for j := range 4 {
			s.AddConstraint(
				sketch.NewOffset(rect[j], outline[j], -w),
				sketch.NewPointOnLine(outline[j].Start, rect[(j+3)%4]),
				sketch.NewPointOnLine(outline[j].End, rect[(j+1)%4]),
				sketch.NewTangent(outline[j], arcs[j]),
			)
		}
	}
	csSolve(t, s)
	csSatisfied(t, s)

	// The angle Fusion writes, read back off the solved geometry.
	var got float64
	if spineAngle {
		got = math.Acos(((cp.X()-o.X())*(eP.X()-o.X()) + (cp.Y()-o.Y())*(eP.Y()-o.Y())) / (ru.Length() * spine.Length()))
	} else {
		got = math.Acos(((cp.X()-o.X())*(l2.End.X()-l2.Start.X()) + (cp.Y()-o.Y())*(l2.End.Y()-l2.Start.Y())) / (ru.Length() * l2.Length()))
	}
	proofkit.Step(t, "angle against %s: %.4f degrees", map[bool]string{true: "K", false: "L2"}[spineAngle], value*180/math.Pi)
	sketchtest.Measures(t, "the angular dimension's value", got, value, sketchtest.Within(1e-9))
	if value < math.Pi/4-1e-12 || value > 3*math.Pi/4+1e-12 {
		t.Fatalf("the angular dimension reads %.4f degrees, outside 45–135 ([PB-ANGULAR-DIM])", value*180/math.Pi)
	}
	for _, c := range []struct {
		pt   *sketch.Point
		u, v float64
		name string
	}{{c0, uB, -hv, "(uB,-hv)"}, {c1, uF, -hv, "(uF,-hv)"}, {c2, uF, hv, "(uF,hv)"}, {c3, uB, hv, "(uB,hv)"}, {eP, uF, 0, "E"}} {
		x, y := csLocal(t, f, g.world(c.u, c.v, st), c.name)
		sketchtest.MeasuresPoint(t, c.pt, x, y, sketchtest.Within(1e-7))
	}

	if collar {
		// find_profile_by_curve_counts(sketch, lines=4, arcs=4): the rounded
		// rectangle; the construction rectangle bounds nothing.
		prof := csOneProfile(t, s, 4, 4)
		area := (uF-uB+2*w)*(2*hv+2*w) - (4-math.Pi)*w*w
		sketchtest.MeasuresProfileArea(t, prof, area, sketchtest.WithinRel(1e-9))
		for j, a := range arcs {
			sketchtest.Measures(t, fmt.Sprintf("arc %d radius", j), a.R(), w, sketchtest.WithinRel(1e-9))
		}
		return
	}
	// find_profile_by_curve_counts(sketch, lines=4): the bore's rectangle.
	prof := csOneProfile(t, s, 4, 0)
	sketchtest.MeasuresProfileArea(t, prof, (uF-uB)*2*hv, sketchtest.WithinRel(1e-9))
}
