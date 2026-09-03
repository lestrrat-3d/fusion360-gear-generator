package bevelgear_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Rejection codes a case carries in its "reject" parameter. 0 means the case
// must resolve; every other value names the bound the case must break, and the
// proof checks that the refusal message names that bound and its computed
// figure rather than merely failing.
const (
	rejectNone = iota
	rejectShaftAngle
	rejectMinTeeth
	rejectBaseTooHigh
	rejectBaseTooLow
	rejectFaceWidth
)

var rejectPhrase = map[int]string{
	rejectShaftAngle:  "Shaft Angle must be",
	rejectMinTeeth:    "Gear Teeth must be at least",
	rejectBaseTooHigh: "Base Height must be at most",
	rejectBaseTooLow:  "Base Height must be at least",
	rejectFaceWidth:   "Face Width must be at most",
}

// inputCases prove the input pass. Every bound the spec states is reached from
// both directions: a value inside it that must resolve, and a value outside it
// that must be refused with the computed figure named.
var inputCases = []proofkit.Case{
	{Name: "defaults", Params: map[string]float64{}},
	{Name: "shaft-30-floor", Params: map[string]float64{"shaftAngleDeg": 30}},
	{Name: "shaft-below-floor", Params: map[string]float64{"shaftAngleDeg": 29.9, "reject": rejectShaftAngle}},
	{Name: "shaft-150-ceiling", Params: map[string]float64{"shaftAngleDeg": 150}},
	{Name: "shaft-above-ceiling", Params: map[string]float64{"shaftAngleDeg": 150.1, "reject": rejectShaftAngle}},
	// A 31/17 pair's cone-angle limit is acos(-17/31), which is 123.256 deg —
	// the spec rounds it to 123.24 — and well under the flat 150 deg an earlier
	// revision promised. The limit is exclusive.
	{Name: "cone-limit-inside", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17, "shaftAngleDeg": 123}},
	{Name: "cone-limit-at", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17,
		"shaftAngleDeg": 123.3, "reject": rejectShaftAngle}},
	{Name: "teeth-floor-4", Params: map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4}},
	{Name: "teeth-below-floor-3", Params: map[string]float64{"drivingTeeth": 3, "pinionTeeth": 3, "reject": rejectMinTeeth}},
	{Name: "base-height-in-window", Params: map[string]float64{"drivingBaseHeight": 3, "pinionBaseHeight": 3}},
	{Name: "base-height-too-high", Params: map[string]float64{"drivingBaseHeight": 14, "reject": rejectBaseTooHigh}},
	{Name: "base-height-too-low", Params: map[string]float64{"drivingBaseHeight": 0.5, "reject": rejectBaseTooLow}},
	// The 30 deg worked case in the spec: the fallback 3.875 mm is capped to
	// 3.638 mm by the Maximum Base Height and the true crossing is 4.153 mm.
	{Name: "base-height-capped-at-30", Params: map[string]float64{"shaftAngleDeg": 30}},
	{Name: "face-width-user", Params: map[string]float64{"faceWidth": 2}},
	{Name: "face-width-too-wide", Params: map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17,
		"faceWidth": 7.3, "reject": rejectFaceWidth}},
	{Name: "bore-off-and-explicit", Params: map[string]float64{"boreEnable": 0, "drivingBore": 6, "pinionBore": 6}},
	{Name: "module-2-19-13", Params: map[string]float64{"module": 2, "drivingTeeth": 19, "pinionTeeth": 13}},
}

// stepInputBounds proves the input pass: the ranges, the two closed-form cone
// angles, the Minimum Teeth floor, the base-height window and the bore and
// face-width defaults, in the order the spec's Variables section fixes.
//
// The sketch it draws is the heel edge the base-height window is about — the
// gear's shaft axis, the drop to Apex 2, the dedendum walk Apex2->C->H — so the
// two bounds are checked as the geometry they are: the Minimum keeps H beyond
// C, and the Maximum keeps H clear of the shaft axis, which is where the
// revolve would fail with ASM_WIRE_X_AXIS ([PB-REVOLVE]).
func stepInputBounds(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	raw := resolve(p)
	want := int(param(p, "reject", 0))
	c := raw

	proofkit.Step(t, "check the resolved input pass")
	if want == rejectNone {
		if len(raw.Problems) != 0 {
			t.Fatalf("a case inside every bound was refused: %v", raw.Problems)
		}
	} else {
		if len(raw.Problems) == 0 {
			t.Fatalf("a case outside %q was accepted", rejectPhrase[want])
		}
		found := false
		for _, problem := range raw.Problems {
			if strings.Contains(problem, rejectPhrase[want]) {
				found = true
				if !strings.ContainsAny(problem, "0123456789") {
					t.Errorf("the refusal does not name the computed bound: %q", problem)
				}
			}
		}
		if !found {
			t.Errorf("wrong refusal: want one naming %q, got %v", rejectPhrase[want], raw.Problems)
		}
		// The bound a refusal names has to be reachable, or the message sends
		// the user somewhere that does not build either. Pull every input back
		// to its own bound and resolve again; the rest of this step then runs on
		// that admitted configuration.
		c = resolve(clampToBounds(p, raw))
		if len(c.Problems) != 0 {
			t.Fatalf("the bounds the refusal names do not themselves resolve: %v", c.Problems)
		}
	}

	proofkit.Step(t, "check the derived lengths and the two cone angles")
	near(t, c.Pinion.Gamma+c.Driving.Gamma, c.Sigma, 1e-12, "the two pitch cone angles sum to the Shaft Angle")
	near(t, math.Tan(c.Pinion.Gamma),
		math.Sin(c.Sigma)*c.Pinion.PitchDia/(c.Driving.PitchDia+c.Pinion.PitchDia*math.Cos(c.Sigma)), 1e-12,
		"tan(gamma_p) closed form")
	near(t, c.R, (c.Pinion.PitchDia/2)/math.Sin(c.Pinion.Gamma), 1e-12, "Pitch Cone Distance R")
	near(t, c.ConeDistance, math.Hypot(c.Pinion.PitchDia, c.Driving.PitchDia), 1e-12, "Cone Distance is the diagonal")
	if math.Abs(c.SigmaDeg-90) < 1e-9 {
		near(t, c.ConeDistance, 2*c.R, 1e-9, "Cone Distance is 2R exactly at Shaft Angle 90 deg")
	} else if math.Abs(c.ConeDistance-2*c.R) < 1e-6 {
		t.Errorf("Cone Distance and 2R must differ away from 90 deg, got %.6f and %.6f", c.ConeDistance, 2*c.R)
	}

	proofkit.Step(t, "check the Maximum Shaft Angle, the Minimum Teeth floor and the bores")
	limit, coneLimited := maxShaftAngleDeg(c.Pinion.PitchDia, c.Driving.PitchDia)
	near(t, c.MaxSigmaDeg, limit, 1e-12, "Maximum Shaft Angle")
	if c.Pinion.PitchDia == c.Driving.PitchDia && coneLimited {
		t.Errorf("equal pitch diameters give acos(-1) = 180 deg, which the 150 deg cap must take over")
	}
	for _, g := range []gearSide{c.Pinion, c.Driving} {
		near(t, g.MinTeeth, 5.27*math.Cos(g.Gamma), 1e-12, "%s Minimum Teeth floor", g.Label)
		near(t, g.MaxBase, 0.95*(g.PitchDia/2-1.25*c.Module*math.Cos(g.Gamma))*math.Tan(g.Gamma), 1e-12,
			"%s Maximum Base Height", g.Label)
		near(t, g.MinBase, 1.05*1.25*c.Module*math.Sin(g.Gamma), 1e-12, "%s Minimum Base Height", g.Label)
		if g.MaxBase <= g.MinBase && g.Teeth >= g.MinTeeth {
			t.Errorf("%s: the base-height window is empty above the Minimum Teeth floor", g.Label)
		}
		if g.MaxBase >= g.PitchDia/2*math.Tan(g.Gamma) {
			t.Errorf("%s: the Maximum Base Height is not below the true crossing r*tan(gamma)", g.Label)
		}
		if c.BoreEnable {
			near(t, g.Bore, g.PitchDia/4, 1e-12, "%s auto bore diameter", g.Label)
		}
	}
	if !c.BoreEnable {
		// Enable Bore off ignores both per-gear diameters; the resolved values
		// stay readable but no bore is cut.
		near(t, c.Pinion.Bore, param(p, "pinionBore", 0), 1e-12, "an explicit pinion bore is carried unchanged")
	}
	if param(p, "drivingBaseHeight", 0) == 0 && c.Driving.BaseHeight != 0 {
		fallback := c.Module * c.Driving.Teeth / 8
		wantBase := math.Min(math.Max(fallback, c.Driving.MinBase), c.Driving.MaxBase)
		near(t, c.Driving.BaseHeight, wantBase, 1e-12, "the driving fallback is raised to the Minimum and capped at the Maximum")
		if param(p, "pinionBaseHeight", 0) == 0 {
			scaled := c.Driving.BaseHeight * (c.Pinion.Teeth / c.Driving.Teeth)
			near(t, c.Pinion.BaseHeight, math.Min(math.Max(scaled, c.Pinion.MinBase), c.Pinion.MaxBase), 1e-12,
				"the pinion fallback scales the RESOLVED driving height, then takes the pinion's own window")
		}
	}

	proofkit.Step(t, "draw the heel edge the base-height window bounds")
	g := c.gearOf(p)
	// The figure is drawn in the gear's own frame: the shaft axis along +X
	// through the origin, Apex 2 one pitch radius below it, and the dedendum
	// walk from Apex 2 out to C and on to H.
	axisStart := s.CreateReferencePoint(0, 0, "shaft axis start")
	axisEnd := s.CreateReferencePoint(g.PitchDia/2/math.Cos(g.Gamma)+1, 0, "shaft axis end")
	axis, err := s.CreateReferenceLine(axisStart, axisEnd, "shaft axis")
	if err != nil {
		t.Fatalf("shaft axis: %v", err)
	}
	axis.SetName("shaft axis")
	apex2 := s.CreateReferencePoint(0, -g.PitchDia/2, "Apex 2")
	apex2.SetName("Apex 2")

	// Walking out along the dedendum from Apex 2 the along-shaft coordinate
	// rises at sin(gamma) and the distance to the axis falls at cos(gamma).
	dedDir := vec2{math.Sin(g.Gamma), math.Cos(g.Gamma)}
	cPoint := s.CreatePoint(1.25*c.Module*dedDir.X, -g.PitchDia/2+1.25*c.Module*dedDir.Y)
	cPoint.SetName("C")
	dedLine := s.CreateLine(apex2, cPoint)
	dedLine.SetName("Apex2->C")
	dedLine.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(axis, dedLine, math.Atan2(dedDir.Y, dedDir.X)*180/math.Pi))
	s.AddConstraint(sketch.NewDistance(apex2, cPoint, 1.25*c.Module))

	reach := g.BaseHeight / math.Sin(g.Gamma)
	hPoint := s.CreatePoint(reach*dedDir.X, -g.PitchDia/2+reach*dedDir.Y)
	hPoint.SetName("H")
	heel := s.CreateLine(cPoint, hPoint)
	heel.SetName("C->H")
	heel.SetConstruction(true)
	s.AddConstraint(sketch.NewPointOnLine(hPoint, dedLine))
	// The base height is the offset between the A->Apex2 drop and G->H, i.e.
	// the along-shaft distance from Apex 2 to H.
	s.AddConstraint(sketch.NewHorizontalDistance(apex2, hPoint, g.BaseHeight))

	solveHere(t, s)
	near(t, hPoint.X()-apex2.X(), g.BaseHeight, 1e-9, "%s base height is the along-shaft offset of H", g.Label)
	if hPoint.Y() >= 0 {
		t.Errorf("%s: H reached the shaft axis at base height %.4f (crossing at r*tan(gamma) = %.4f)",
			g.Label, g.BaseHeight, g.PitchDia/2*math.Tan(g.Gamma))
	}
	if hPoint.X() <= cPoint.X() {
		t.Errorf("%s: H landed behind C, so C->H runs back inward", g.Label)
	}
}

// clampToBounds pulls every input in p back to the bound the refusal named, so
// the step can go on to check that the admitted configuration builds. It is the
// proof's reading of "reject it with a message stating the maximum": the stated
// maximum has to be a value that resolves.
func clampToBounds(p map[string]float64, raw config) map[string]float64 {
	out := map[string]float64{}
	for k, v := range p {
		out[k] = v
	}
	delete(out, "reject")
	limit, coneLimited := maxShaftAngleDeg(raw.Pinion.PitchDia, raw.Driving.PitchDia)
	if coneLimited {
		limit -= 0.01
	}
	out["shaftAngleDeg"] = math.Min(math.Max(param(p, "shaftAngleDeg", 90), 30), limit)
	out["drivingTeeth"] = math.Max(math.Round(param(p, "drivingTeeth", 31)), math.Ceil(raw.Driving.MinTeeth))
	out["pinionTeeth"] = math.Max(math.Round(param(p, "pinionTeeth", 31)), math.Ceil(raw.Pinion.MinTeeth))
	if h := param(p, "drivingBaseHeight", 0); h != 0 {
		out["drivingBaseHeight"] = math.Min(math.Max(h, raw.Driving.MinBase), raw.Driving.MaxBase)
	}
	if h := param(p, "pinionBaseHeight", 0); h != 0 {
		out["pinionBaseHeight"] = math.Min(math.Max(h, raw.Pinion.MinBase), raw.Pinion.MaxBase)
	}
	if fw := param(p, "faceWidth", 0); fw != 0 {
		out["faceWidth"] = math.Min(fw, raw.MaxFaceWidth)
	}
	return out
}
