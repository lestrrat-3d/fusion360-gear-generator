package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// The borrowed spur tooth generator's fixed inputs. Neither is a bevel dialog
// input: the proxy's defaults are a 20 degree pressure angle and 15 involute
// samples, and bevel supplies only the module and the virtual tooth count.
const (
	proxyPressureAngle = math.Pi / 9 // 20 degrees
	proxyInvoluteSteps = 15
	// The rotation bevel passes as draw()'s angle argument. The whole tooth is
	// drawn already turned by it, in the point math, never rotated afterwards.
	bevelToothAngle = math.Pi
)

// toothCases sweeps both gears of several pairs, because the virtual tooth
// count — and with it the embedded branch the profile search keys on — is a
// function of that gear's own cone angle, so the two members of one pair land on
// opposite sides of it.
//
// The default 31/31 pair is already embedded on both sides: its virtual count is
// 43, above the 41.5 at which the base circle falls inside the root circle at a
// 20 degree pressure angle. The 31/17 pair reaches the other branch on its
// pinion, at 19 virtual teeth, while its driving gear is embedded at 64. So both
// values of the wantLines selector are reached from both gears, which is what
// the spec's warning about the impostor loop needs.
var toothCases = []proofkit.Case{
	{Name: "default_31_31_pinion_embedded", Params: defaultParams()},
	{Name: "default_31_31_driving_embedded", Params: with(defaultParams(), pGear, 1.0)},
	{Name: "ratio_31_17_pinion_not_embedded", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_31_17_driving_embedded", Params: with(defaultParams(),
		pPinionTeeth, 17.0, pGear, 1.0)},
	{Name: "minimum_teeth_4_4_pinion", Params: with(defaultParams(),
		pDrivingTeeth, 4.0, pPinionTeeth, 4.0)},
	{Name: "coarse_module_3_20_20_driving", Params: with(defaultParams(),
		pModule, 3.0, pDrivingTeeth, 20.0, pPinionTeeth, 20.0, pGear, 1.0)},
	{Name: "shaft_angle_140deg_pinion", Params: with(defaultParams(), pShaftAngle, deg(140))},
	{Name: "shaft_angle_35deg_driving", Params: with(defaultParams(),
		pShaftAngle, deg(35), pGear, 1.0)},
	{Name: "tooth_spacing_1mm_pinion", Params: with(defaultParams(),
		pPinionTeeth, 17.0, pToothSpacing, 1.0)},
}

// stepToothSketch draws the `{gearLabel} Tooth` sketch: the borrowed spur tooth,
// at this gear's virtual tooth number, already rotated by 180 degrees.
//
// WHAT THIS SKETCH SUBSTITUTES. The Fusion sketch carries four along-path
// labels, one per circle ([PB-SKETCH-TEXT]), and sketch text placed with
// setAsAlongPath carries a position nothing in these recipes pins, which is why
// [BEVEL-F-FULL-CONSTRAINT] exempts the two tooth sketches from the gate. The
// bench engine has no sketch text, so what is drawn here is the same sketch with
// the labels absent — which is exactly the state that exemption says must read
// fully constrained on its own. This step therefore gates it normally, and a
// pass here is the measurement the exemption asks for before it is renewed.
//
// <!-- proof-run: proofkit.Run(toothCases, stepToothSketch) -->
func stepToothSketch(t testing.TB, s *sketch.Sketch, params map[string]float64) {
	q := pairOf(t, params)
	side := q.gearOf(params)
	virtual := side.virtualTeeth(q.module)

	proofkit.Step(t, "%s: virtual pitch radius %.6f mm at cone angle %.6f rad gives %.0f virtual teeth",
		side.label, side.pitchDia/2/math.Cos(side.gamma), side.gamma, virtual)
	assertVirtualTeeth(t, q, side, virtual)

	d := involute.Derive(q.module, virtual, proxyPressureAngle)
	tooth := drawSpurTooth(t, s, d, virtual, bevelToothAngle)
	solve(t, s)

	proofkit.Step(t, "the tooth sits at the 180 degrees draw() was given")
	if got := math.Atan2(tooth.toothTop.Y(), tooth.toothTop.X()); math.Abs(math.Abs(got)-math.Pi) > 1e-7 {
		t.Errorf("the tooth top sits at %.9f rad, not at the %.9f rad draw() was passed",
			got, bevelToothAngle)
	}
	if got := math.Hypot(tooth.toothTop.X(), tooth.toothTop.Y()); math.Abs(got-d.Tip) > 1e-7 {
		t.Errorf("the tooth top sits at radius %.6f, not on the tip circle at %.6f", got, d.Tip)
	}

	proofkit.Step(t, "the loop the tooth-profile search selects, and the count it selects on")
	assertToothProfile(t, s, d, virtual)
}

// assertVirtualTeeth checks the back-cone tooth count against the closed form,
// and against the one wrong answer the spec names: measuring Apex2->K' instead.
//
// The two disagree by construction. K sits at cone distance R/cos(gamma) along
// the shaft axis, so Apex2->K is R*tan(gamma), while the virtual pitch radius is
// r/cos(gamma). The proof compares them so a build that measured the lattice
// instead of computing the closed form would be caught here rather than in a
// tooth of the wrong size.
func assertVirtualTeeth(t testing.TB, q pair, side gearSide, virtual float64) {
	t.Helper()
	radius := side.pitchDia / 2 / math.Cos(side.gamma)
	if want := math.Floor(2 * radius / q.module); virtual != want {
		t.Fatalf("%s: the virtual tooth number is %.0f, not floor(2 * %.6f / %.6f) = %.0f",
			side.label, virtual, radius, q.module, want)
	}
	if virtual < 3 {
		t.Fatalf("%s: a virtual tooth count of %.0f cannot carry an involute tooth",
			side.label, virtual)
	}
	// The lattice does carry this radius: Apex2->K is R*tan(gamma), which is
	// exactly r/cos(gamma). What it does not carry is a radius independent of the
	// Tooth Spacing — Apex2->K' is longer by the whole spacing — and the spec's
	// rule that the count comes from the closed form and never from measuring
	// Apex2->K' is what keeps the drawn tooth the same size at every spacing.
	if measured := q.pitchConeDist * math.Tan(side.gamma); math.Abs(measured-radius) > 1e-9 {
		t.Errorf("%s: Apex2->K measures %.9f against the virtual pitch radius %.9f; the two are "+
			"the same length", side.label, measured, radius)
	}
	if q.toothSpacing > 0 {
		spaced := math.Floor(2 * (radius + q.toothSpacing) / q.module)
		if spaced == virtual {
			t.Errorf("%s: measuring Apex2->K' would give the same %.0f virtual teeth at a Tooth "+
				"Spacing of %.6f, so this case cannot show why the spec forbids measuring it",
				side.label, virtual, q.toothSpacing)
		}
	}
}

// spurTooth is the drawn tooth and the handles its assertions read.
type spurTooth struct {
	origin   *sketch.Point
	spine    *sketch.Line
	toothTop *sketch.Point
	embedded bool
}

// drawSpurTooth draws one involute tooth the way the borrowed generator does.
//
// The construction is spur's, cited from spec/spurgear/instructions.md §3 and
// §4: four circles on the shared local origin with driving diameter dimensions,
// the two flanks as fitted splines through samples that run base circle to tip
// circle endpoint-inclusive, the tooth-top arc, the spine with its +X reference
// and confirming angular dimension, one rib per sample with the last carrying no
// perpendicular, and the flank-to-root lines only when the flank starts outside
// the root circle. Bevel adds nothing to it — it supplies the module and the
// virtual tooth count through the proxy and passes 180 degrees as the angle.
func drawSpurTooth(t testing.TB, s *sketch.Sketch, d involute.Dimensions,
	toothNumber, angle float64) *spurTooth {
	t.Helper()

	// The projected tooth-centre point, and the movable local origin the whole
	// tooth is built relative to. Fusion projects K'/L' in and coincides the
	// generator's own anchorPoint to it; the engine's reference point is the same
	// thing, a point this sketch does not own the coordinates of.
	anchor := s.CreateReferencePoint(0, 0, "toothCenterPoint")
	anchor.SetName("projected tooth centre")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	circle := func(r float64, construction bool, name string) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		c.SetName(name)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(d.Root, false, "root circle")
	tip := circle(d.Tip, true, "tip circle")
	circle(d.Base, true, "base circle")
	circle(d.Pitch, true, "pitch circle")

	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothNumber, proxyInvoluteSteps, angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(leftPts...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rightPts...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	// The tooth-top arc caps the tooth at the tip circle. Fusion's
	// addByCenterStartEnd COPIES the centre rather than sharing it, so the Fusion
	// build needs an explicit coincidence back to the origin — the fix that took
	// a 0.5743 mm arc back to the intended 22.5 mm tip radius. The engine shares
	// the point handle it is given, which is the same constraint by the other of
	// the two routes [PB-SHARE-XOR-COINCIDENT] allows.
	last := len(leftPts) - 1
	s.CreateArc(origin, rightPts[last], leftPts[last])

	// Spine, +X reference and the angular pin. The reference line's far end is
	// held by signed distances, and the angular dimension runs from it to the
	// spine, so the tooth's placement carries a direction and a sign.
	ttx, tty := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(ttx, tty)
	toothTop.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	reference.SetName("+X reference")
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	// Ribs, one per sample. The rib takes the axis across the spine and the
	// midpoint chain the axis along it, and which axis that is swaps once the
	// tooth passes 45 degrees.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prevMid := origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		mx, my := foot*math.Cos(angle), foot*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(
			sketch.NewPointOnLine(mid, spine),
			sketch.NewMidpoint(mid, rib),
		)
		// The last rib joins the two flank tips, which the tooth-top arc already
		// holds either side of the spine, so its perpendicular is redundant.
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prevMid, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevMid, mid, my-prevY))
		}
		prevMid, prevX, prevY = mid, mx, my
	}

	// The flank-to-root lines, and the strict embedded test. Above roughly 41.5
	// virtual teeth at a 20 degree pressure angle the base circle falls inside
	// the root circle, no stub is drawn, and the tooth loop loses its two lines —
	// which is the branch bevel's profile search selects with wantLines = 0.
	embedded := d.Embedded()
	if !embedded {
		stub := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := d.Root*seed.X/n, d.Root*seed.Y/n
			foot := s.CreatePoint(rx, ry)
			s.CreateLine(foot, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, foot, rx),
				sketch.NewVerticalDistance(origin, foot, ry),
			)
		}
		stub(leftPts[0], left[0])
		stub(rightPts[0], right[0])
	}
	return &spurTooth{origin: origin, spine: spine, toothTop: toothTop, embedded: embedded}
}

// curveCounts is one profile's boundary, counted by curve type.
type curveCounts struct {
	splines int
	lines   int
	arcs    int
	circles int
}

func countCurves(p *sketch.Profile) curveCounts {
	var c curveCounts
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			c.splines++
		case *sketch.Line:
			c.lines++
		case *sketch.Arc:
			c.arcs++
		case *sketch.Circle:
			c.circles++
		}
	}
	return c
}

// assertToothProfile checks the count the tooth-profile search selects on, and
// that exactly one loop carries it.
//
// The spec is emphatic that the line count is DETERMINED by the embedded flag
// and never accepted as "0 or 2": an unrelated loop between the drawCircles
// circles can carry the same two NURBS and two arcs with the other line count,
// and lofting that impostor dies with LOFT_NO_TOOLBODY. So the assertion is not
// only that the tooth loop has the right count — it is that no other loop in the
// sketch shares it.
//
// The engine reports a curve split by the arrangement against its parent entity,
// so the root arc the tooth borrows from the root circle comes back as that
// Circle rather than as an Arc. Fusion's nurbs=2, arcs=2, lines=N is therefore
// read here as splines=2, arcs=1, circles=1, lines=N.
func assertToothProfile(t testing.TB, s *sketch.Sketch, d involute.Dimensions, toothNumber float64) {
	t.Helper()
	wantLines := 2
	if d.Embedded() {
		wantLines = 0
	}
	want := curveCounts{splines: 2, arcs: 1, circles: 1, lines: wantLines}

	matches := make([]*sketch.Profile, 0, 1)
	for _, p := range s.Profiles() {
		if !p.Valid {
			t.Errorf("a detected region is not an extrudable profile: area %.6f", p.Area)
		}
		if countCurves(p) == want {
			matches = append(matches, p)
		}
	}
	if len(matches) != 1 {
		counts := make([]curveCounts, 0, len(s.Profiles()))
		for _, p := range s.Profiles() {
			counts = append(counts, countCurves(p))
		}
		t.Fatalf("%d of the sketch's %d regions carry the loop %+v the tooth-profile search "+
			"selects on; the search takes exactly one. The regions are %+v",
			len(matches), len(s.Profiles()), want, counts)
	}
	tooth := matches[0]
	if tooth.Area <= 0 {
		t.Fatalf("the tooth region encloses %.9f", tooth.Area)
	}
	// One tooth of a gear of this many teeth cannot be more than a whole turn's
	// worth of the annulus between root and tip.
	annulus := math.Pi * (d.Tip*d.Tip - d.Root*d.Root)
	if tooth.Area > annulus/toothNumber*2 {
		t.Errorf("the tooth region measures %.6f against %.6f for one tooth's share of the "+
			"root-to-tip annulus; the search has selected something larger than a tooth",
			tooth.Area, annulus/toothNumber)
	}
}
