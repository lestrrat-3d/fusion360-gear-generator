package helicalgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
)

// twistedProfileCases is the regime the Twisted Gear Profile scheme has to hold
// across (spec/spurgear/instructions.md "Sketch Discipline", applied to the
// helix angle by spec/helicalgear/instructions.md "Variables"):
//
//   - several sizes at the default 14.5° helix;
//   - the signed range of the angle: the default, a left-hand (negative)
//     helix of the same size, larger twists of both signs, and a quarter turn
//     where |sin| > |cos| swaps the rib and chain axes;
//   - the low end of the involute sample count beside the standard 15;
//   - both routes into the embedded shape, a high tooth count at 20° and a
//     moderate one at 25°, which helical's loft does not support but the
//     sketch still has to close as the 4-curve loop the spur spec describes.
var twistedProfileCases = []proofkit.Case{
	{Name: "M1_N17_helix+14.5", Params: caseParams(1, 17, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-14.5_left_hand", Params: caseParams(1, 17, 20, -14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N12_helix+14.5", Params: caseParams(1, 12, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M2_N20_helix+14.5", Params: caseParams(2, 20, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M3_N15_helix-14.5", Params: caseParams(3, 15, 20, -14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N35_helix+14.5_short_stub", Params: caseParams(1, 35, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix0_spur_baseline", Params: caseParams(1, 17, 20, 0, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+35", Params: caseParams(1, 17, 20, 35, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-35", Params: caseParams(1, 17, 20, -35, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+60", Params: caseParams(1, 17, 20, 60, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-60", Params: caseParams(1, 17, 20, -60, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+90_quarter_turn", Params: caseParams(1, 17, 20, 90, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix-90_quarter_turn", Params: caseParams(1, 17, 20, -90, defaultInvoluteSteps, defaultThickness)},
	{Name: "M1_N17_helix+14.5_steps4", Params: caseParams(1, 17, 20, 14.5, 4, defaultThickness)},
	{Name: "M1_N17_helix-14.5_steps4", Params: caseParams(1, 17, 20, -14.5, 4, defaultThickness)},
	{Name: "M1_N60_PA20_helix+14.5_embedded_by_tooth_count", Params: caseParams(1, 60, 20, 14.5, defaultInvoluteSteps, defaultThickness)},
	{Name: "M2_N30_PA25_helix-14.5_embedded_by_pressure_angle", Params: caseParams(2, 30, 25, -14.5, defaultInvoluteSteps, defaultThickness)},
}

// stepTwistedGearProfile is the Twisted Gear Profile sketch: the spur tooth
// generator's angle != 0 path, drawn by
// SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint,
// angle=HelixAngle) on the helix plane ([HELI-F-TWIST-PLANE]).
//
// The plane itself is not modelled here: proofkit.Run hands every case a sketch
// on the world XY datum, and a sketch's constraint verdict does not depend on
// which plane it sits on. The offset is proven in stepLoftTooth, which draws
// this same profile on a plane offset by Thickness and reads the offset back
// off the lofted body's extent.
//
// After the recipe, the sketch is solved and its regions counted, because the
// counts are the key the loft step matches on: the tooth loop must close as
// 2 NURBS + 2 arcs + 2 lines (one of the "arcs" being the root-circle piece
// the flank-to-root lines split off), and the disc inside the root circle as
// exactly 2 arcs. In the embedded regime the tooth closes as 2 NURBS + 2 arcs
// and no 6-curve loop exists, which is the loop helical's loft searches for
// with a fixed lines=2 and never finds ([HELI-F-LOFT]); the proof records that
// as the documented limitation rather than a scheme defect.
func stepTwistedGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	tp := drawToothProfile(t, s, p, 0, p[keyHelixAngle], false)

	proofkit.Step(t, "solve and count the closed regions")
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge before the region count: residual %.3e DOF %d", res.Residual, res.DOF)
	}
	profiles := s.Profiles()
	assertToothRegions(t, profiles, tp)
}

// assertToothRegions pins the two regions the Gear Profile sketch closes and
// their curve counts, on the sketch that was actually drawn.
func assertToothRegions(t testing.TB, profiles []*sketch.Profile, tp *toothProfile) {
	t.Helper()
	valid := 0
	for _, p := range profiles {
		if p.Valid {
			valid++
		}
	}
	if valid != 2 {
		t.Fatalf("expected exactly two closed regions (the tooth and the disc), got %d valid of %d:%s",
			valid, len(profiles), describeProfiles(profiles))
	}

	wantLines := 2
	if tp.embedded {
		wantLines = 0
	}
	tooth := findProfile(t, profiles, edgeCounts{nurbs: 2, arcs: 2, lines: wantLines})
	if tooth == nil {
		t.Fatalf("no tooth loop with nurbs=2 arcs=2 lines=%d:%s", wantLines, describeProfiles(profiles))
	}
	toothEdges := countEdges(tooth.Outer)
	if toothEdges.circleFragments != 1 {
		t.Fatalf("the tooth loop should be closed by exactly one piece of the root circle, got %d", toothEdges.circleFragments)
	}
	if tp.embedded {
		if six := findProfile(t, profiles, edgeCounts{nurbs: 2, arcs: 2, lines: 2}); six != nil {
			t.Fatalf("an embedded profile must have no 6-curve tooth loop, but one was found")
		}
	}

	// The disc inside the root circle. Fusion reads its boundary as exactly 2
	// arcs, the two pieces the flank-to-root lines split the root circle into
	// (spur step 9). The sketch engine splits the circle the same way for the
	// tooth loop — its root edge above is a Partial piece of the circle — but
	// reports the disc, which the tooth only touches from outside at the two
	// line ends, as the whole circle in one edge. So the count asserted here is
	// "root-circle edges and nothing else", one whole or two pieces, together
	// with the full disc area; the split into two arcs is a fact about how
	// Fusion labels the same boundary, and only a Fusion session reads it.
	var disc *sketch.Profile
	for _, p := range profiles {
		if p == tooth || !p.Valid {
			continue
		}
		c := countEdges(p.Outer)
		if c.nurbs == 0 && c.lines == 0 && c.arcs == c.circleFragments && c.circleFragments >= 1 && c.circleFragments <= 2 {
			disc = p
		}
	}
	if disc == nil {
		t.Fatalf("no disc bounded by the root circle alone:%s", describeProfiles(profiles))
	}
	for _, e := range disc.Outer {
		if e.Entity != sketch.Entity(tp.root) {
			t.Fatalf("the disc is bounded by %q, not the Root Circle", e.Entity.Name())
		}
	}
	wantArea := math.Pi * tp.dims.Root * tp.dims.Root
	if math.Abs(disc.Area-wantArea) > 1e-3*wantArea {
		t.Fatalf("disc area %.4f, want the full root disc %.4f", disc.Area, wantArea)
	}
	if tooth.Area <= 0 || disc.Area <= tooth.Area {
		t.Fatalf("region areas are implausible: tooth %.4f, disc %.4f", tooth.Area, disc.Area)
	}
}
