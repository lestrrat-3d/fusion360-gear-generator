package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// pressureAngle is the virtual spur proxy's fixed 20 degrees. It is not a bevel
// dialog input; the proxy serves it and the borrowed drawer reads it.
const pressureAngle = 20 * math.Pi / 180

// involuteSteps is the proxy's fixed sample count per flank.
const involuteSteps = 15

// toothOf returns the gear-specific inputs §3 draws one virtual spur tooth from:
// this gear's pitch diameter, its pitch cone half angle, and the virtual tooth
// number the two give.
func toothOf(p params) (pitchDiameter, gamma float64, teeth int) {
	c := coneOf(p)
	pitchDiameter, gamma = c.DPD, c.GammaG
	if p.Pinion {
		pitchDiameter, gamma = c.PPD, c.GammaP
	}
	return pitchDiameter, gamma, virtualTeeth(p.Module, pitchDiameter, gamma)
}

// stepToothProfile draws one gear's `{label} Tooth` sketch: the four circles the
// borrowed spur drawer lays down and the single involute tooth it draws on them,
// at the virtual tooth number, already rotated 180 degrees, centred on the
// tooth-centre point.
//
// What it substitutes, and what that costs. The spur drawer holds its tooth
// together with a spine, a rib per flank sample and a tooth-top arc whose
// constraint set is spur's own; this proof recreates the tooth outline from the
// shared involute math and fixes it, the recreate-share-fix shape the playbook
// gives for turning another sketch's geometry into fully constrained local
// geometry. The cost is that it does not re-prove spur's rib and spine scheme —
// which is spur's proof to make, and which bevel's own full-constraint gate
// exempts this sketch from anyway. What it does prove is what bevel reads out of
// this sketch: the virtual tooth number, the 180 degree placement about the
// tooth-centre point, the embedded decision, and the curve counts the later
// profile search keys on.
func stepToothProfile(t testing.TB, s *sketch.Sketch, pm map[string]float64) {
	p := read(pm)
	pitchDiameter, gamma, teeth := toothOf(p)
	if teeth < 3 {
		proofkit.Unmodelled(t, "the virtual tooth number is %d, below the three teeth an "+
			"involute tooth needs", teeth)
		return
	}
	dims := involute.Derive(p.Module, float64(teeth), pressureAngle)

	proofkit.Step(t, "the virtual (back-cone) tooth number for this gear")
	near(t, "virtual pitch radius", (pitchDiameter/2)/math.Cos(gamma),
		float64(teeth)*p.Module/2, p.Module/2)
	// The spec pins a cm-to-mm conversion here because Fusion holds the pitch
	// diameters in internal centimetres while Module is a raw millimetre number.
	// This proof works in millimetres throughout, so the factor is absent — and
	// this is what dropping it would cost.
	if dropped := virtualTeeth(p.Module, pitchDiameter/10, gamma); dropped*10 > teeth+10 ||
		dropped*10 < teeth-10 {
		t.Errorf("skipping the cm-to-mm conversion gives %d virtual teeth against the correct "+
			"%d; the factor of ten the spec pins is exactly this", dropped, teeth)
	}

	proofkit.Step(t, "the four circles, all centred on the tooth-centre point")
	origin := s.CreatePoint(0, 0)
	origin.SetName("tooth centre")
	s.Fix(origin)
	root := s.CreateCircle(origin, dims.Root)
	root.SetName("root circle")
	s.AddConstraint(sketch.NewDiameter(root, 2*dims.Root))
	for _, circle := range []struct {
		name   string
		radius float64
	}{{"tip circle", dims.Tip}, {"base circle", dims.Base}, {"pitch circle", dims.Pitch}} {
		c := s.CreateCircle(origin, circle.radius)
		c.SetName(circle.name)
		c.SetConstruction(true)
		s.AddConstraint(sketch.NewDiameter(c, 2*circle.radius))
	}

	proofkit.Step(t, "the two involute flanks, drawn already rotated 180 degrees")
	leftPts, rightPts := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, float64(teeth),
		involuteSteps, math.Pi)
	if len(leftPts) != involuteSteps || len(rightPts) != involuteSteps {
		t.Fatalf("the flanks sampled %d and %d points, want %d each",
			len(leftPts), len(rightPts), involuteSteps)
	}
	leftHandles := samplePoints(s, "left flank", leftPts)
	rightHandles := samplePoints(s, "right flank", rightPts)
	leftFlank, err := s.CreateFitSpline(leftHandles...)
	if err != nil {
		t.Fatalf("draw the left flank: %v", err)
	}
	rightFlank, err := s.CreateFitSpline(rightHandles...)
	if err != nil {
		t.Fatalf("draw the right flank: %v", err)
	}
	// Fixed only now the curves exist: fixing a bare point before it is consumed
	// as a curve's point does not leave the sketch fully constrained.
	for _, handle := range append(append([]*sketch.Point{}, leftHandles...), rightHandles...) {
		s.Fix(handle)
	}

	proofkit.Step(t, "the tooth-top arc, its centre pinned onto the tooth centre")
	// The arc is created from a fresh centre point and pinned with one coincident
	// rather than sharing the tooth centre, which is the shape the fix to the
	// stranded-centre defect takes in Fusion: addByCenterStartEnd COPIES the centre
	// it is handed, so the arc needs the coincident to keep its centre on the tooth
	// centre when the anchoring drags the tooth onto K prime.
	arcCentre := s.CreatePoint(0, 0)
	arcCentre.SetName("tooth-top arc centre")
	// Start at the right flank's tip and end at the left flank's: the arc sweeps
	// counter-clockwise from start to end, and with the tooth drawn at 180 degrees
	// the right tip is the counter-clockwise-first of the two. Handing them over
	// the other way builds the MAJOR arc, and the region that closes is then the
	// whole annulus rather than the tooth.
	top := s.CreateArc(arcCentre, rightHandles[len(rightHandles)-1], leftHandles[len(leftHandles)-1])
	top.SetName("tooth-top arc")
	// One equation, not two: the arc's own internal construction already holds its
	// centre on the perpendicular bisector of its two ends, so a full coincident
	// to the fixed tooth centre would be redundant by exactly one. The remaining
	// freedom runs along the tooth's axis of symmetry, which for a tooth drawn at
	// 180 degrees is the sketch's X axis.
	pinAlong(s, arcCentre, origin, vec{1, 0})

	embedded := dims.Embedded()
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	if embedded {
		proofkit.Step(t, "embedded tooth: the flanks start inside the root circle, so no "+
			"flank-to-root lines are drawn")
	} else {
		proofkit.Step(t, "non-embedded tooth: one radial flank-to-root line per side")
		for _, side := range []struct {
			name string
			at   involute.Pt
		}{{"left", leftPts[0]}, {"right", rightPts[0]}} {
			r := math.Hypot(side.at.X, side.at.Y)
			foot := s.CreatePoint(side.at.X*dims.Root/r, side.at.Y*dims.Root/r)
			foot.SetName(side.name + " flank root foot")
			s.Fix(foot)
			line := s.CreateLine(pointNamed(s, side.name, leftHandles, rightHandles), foot)
			line.SetName(side.name + " flank-to-root line")
		}
	}

	proofkit.Step(t, "the tooth loop the later profile search keys on")
	assertToothLoop(t, s, p, dims, teeth, embedded, wantLines, leftFlank, rightFlank, top)
}

// pinAlong holds to at from's coordinate on the axis most aligned with dir. It is
// the one-equation half of a coincident, for a point whose other freedom another
// constraint already took.
func pinAlong(s *sketch.Sketch, from, to *sketch.Point, dir vec) {
	if math.Abs(dir.X) >= math.Abs(dir.Y) {
		s.AddConstraint(sketch.NewHorizontalDistance(from, to, 0))
		return
	}
	s.AddConstraint(sketch.NewVerticalDistance(from, to, 0))
}

// samplePoints adds one flank's sample points. They are fixed by the caller once
// the spline that shares them exists, which is the order the recreate-share-fix
// recipe requires.
func samplePoints(s *sketch.Sketch, name string, pts []involute.Pt) []*sketch.Point {
	out := make([]*sketch.Point, 0, len(pts))
	for _, pt := range pts {
		p := s.CreatePoint(pt.X, pt.Y)
		p.SetName(name)
		out = append(out, p)
	}
	return out
}

// pointNamed picks the base-circle end of the named flank.
func pointNamed(_ *sketch.Sketch, side string, left, right []*sketch.Point) *sketch.Point {
	if side == "left" {
		return left[0]
	}
	return right[0]
}

// assertToothLoop finds the tooth region among the sketch's profiles and holds it
// to what the spec pins: the curve mix the later profile search keys on, the tip
// and root radii, and the tooth's 180 degree placement.
func assertToothLoop(t testing.TB, s *sketch.Sketch, p params, dims involute.Dimensions,
	teeth int, embedded bool, wantLines int, leftFlank, rightFlank *sketch.FitSpline,
	top *sketch.Arc) {
	var tooth *sketch.Profile
	for _, profile := range s.Profiles() {
		if !usesAll(profile, leftFlank, rightFlank, top) {
			continue
		}
		if tooth != nil {
			t.Fatalf("two regions use both flanks and the tooth-top arc; the tooth loop must " +
				"be the only one")
		}
		tooth = profile
	}
	if tooth == nil {
		t.Fatalf("no region uses both flanks and the tooth-top arc; the sketch detected %d "+
			"region(s)", len(s.Profiles()))
	}
	if !tooth.Valid || tooth.SelfIntersecting {
		t.Fatalf("the tooth region is not extrudable: valid=%v selfIntersecting=%v",
			tooth.Valid, tooth.SelfIntersecting)
	}

	splines, arcs, lines := 0, 0, 0
	for _, edge := range tooth.Outer {
		switch edge.Entity.(type) {
		case *sketch.FitSpline:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Circle:
			// The root circle reaches the loop as a fragment, which is what Fusion
			// reports as a second arc of the tooth profile.
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("the tooth loop has %d spline(s), %d arc(s) and %d line(s); the profile "+
			"search asks for 2 NURBS, 2 arcs and %d line(s), and the line count is decided by "+
			"the embedded flag (embedded=%v), never accepted as either", splines, arcs, lines,
			wantLines, embedded)
	}

	// Embedded is decided by where the flank starts, which is the base circle, so
	// the two radii settle it. The proof measures both against the drawn circles.
	if got := dims.Base < dims.Root; got != embedded {
		t.Errorf("the embedded flag is %v but base radius %.4f against root radius %.4f says %v",
			embedded, dims.Base, dims.Root, got)
	}

	// The tooth sits at 180 degrees: its tooth-top point is on -X, at the tip radius.
	tip := vec{top.Start.X(), top.Start.Y()}
	near(t, "the tooth tip sample radius", norm(tip), dims.Tip, 1e-9)
	crest := scale(add(tip, vec{top.End.X(), top.End.Y()}), 0.5)
	if crest.X >= 0 {
		t.Errorf("the tooth crest is at (%.4f, %.4f); a tooth drawn at 180 degrees has its "+
			"crest on -X", crest.X, crest.Y)
	}
	near(t, "the tooth crest direction", math.Abs(math.Atan2(crest.Y, crest.X)), math.Pi, 1e-6)
	// The default pair's tooth cross-section was measured in Fusion at 3.5475 mm2
	// once the tooth-top arc's centre was pinned. The proof builds the same tooth
	// from the same involute math and lands on the same area, which is what ties
	// this bench tooth to the one Fusion draws.
	if p.Module == 1 && teeth == 43 {
		near(t, "the default pair's tooth cross-section area", tooth.Area, 3.5475, 0.02)
	}
}

// usesAll reports whether the region's outer boundary walks every one of the
// named entities.
func usesAll(profile *sketch.Profile, wanted ...sketch.Entity) bool {
	for _, want := range wanted {
		found := false
		for _, edge := range profile.Outer {
			if edge.Entity == want {
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

// toothCases covers both gears of the pair, both sides of the embedded branch,
// and the tooth counts and shaft angles that move the virtual tooth number across
// the embedded threshold — 41.5 teeth at a 20 degree pressure angle.
var toothCases = []proofkit.Case{
	{Name: "default_pinion_embedded", Params: map[string]float64{"pinion": 1}},
	{Name: "default_driving_embedded", Params: map[string]float64{"pinion": 0}},
	{Name: "ratio_pinion_17_not_embedded", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "ratio_driving_31_embedded", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 0}},
	{Name: "module_2_driving_19", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 0}},
	{Name: "module_2_pinion_13", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 1}},
	{Name: "shaft_angle_30_pinion", Params: map[string]float64{
		"shaftAngle": 30, "drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "shaft_angle_135_driving", Params: map[string]float64{
		"shaftAngle": 135, "pinion": 0}},
	{Name: "low_tooth_count_8_8_pinion", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 1}},
}
