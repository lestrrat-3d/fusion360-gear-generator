// The sketch steps of spec/spurgear/steps.md, proven in the sketch engine.
//
// Steps this file cannot reach, recorded next to the nearest thing it does
// build (their step-list entries are [PROSE]):
//
//   - Step 6 (Normalize the Target Plane) and step 8 (Extrusion End Plane)
//     are construction-plane timeline entries. The sketch engine has no 3D
//     construction planes; every bench sketch sits on the world XY plane, so
//     plane normalization and the offset end plane cannot be represented
//     here. The solid steps model the end plane's one job — bounding the
//     extrudes — as the extrude distance itself (solids_test.go).
//   - Step 10 (Sketch-Only Short-Circuit) and step 20 (Cleanup) are control
//     flow and visibility toggles, not geometry; neither harness has
//     visibility or a feature timeline to assert against.
package spurgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// toolsCases places the user's anchor at different spots; the Tools sketch
// only projects it, so any position must come through unchanged.
var toolsCases = []proofkit.Case{
	{Name: "anchor at plane origin", Params: map[string]float64{pAnchorX: 0, pAnchorY: 0}},
	{Name: "anchor off origin", Params: map[string]float64{pAnchorX: 25, pAnchorY: -12.5}},
}

// stepToolsSketch models step 7: the Tools sketch draws no geometry of its
// own; it exists to own the one projected copy of the user's anchor that
// every later sketch re-projects ([SPUR-F-ANCHOR-CHAIN]). The projection is a
// reference point: externally locked, tracking its source, never moved by
// the solver — which is what sketch.project brings in.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user's anchor into the Tools sketch")
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "userAnchorPoint")
	anchor.SetName("ctx.anchorPoint")
	if !anchor.IsReference() || anchor.Source() != "userAnchorPoint" {
		t.Errorf("projected anchor must be reference geometry tracking its source")
	}
	if anchor.X() != p[pAnchorX] || anchor.Y() != p[pAnchorY] {
		t.Errorf("projection moved the anchor: (%v,%v)", anchor.X(), anchor.Y())
	}
}

// profileCases is the regime the Gear Profile scheme must hold across
// (instructions.md Sketch Discipline): sizes coarse and fine, the whole
// signed range of the draw() angle including a quarter turn each way, the
// low end of the involute-step count, and the embedded shape reached through
// BOTH of its routes (high tooth count at 20 degrees, moderate count at 25
// degrees).
//
// Scope: the exact base==root boundary (ToothNumber = 2.5/(1-cos(PA))) is
// excluded. There the strict less-than comparison draws a zero-length
// flank-to-root stub, whose two coincident endpoints make the constraint
// system ill-conditioned; the spec keeps the strict comparison anyway
// ([SPUR-F-FLANK-ROOT]), so the boundary is a documented singular point, not
// a case this table can hold DOF 0 across.
var profileCases = []proofkit.Case{
	{Name: "default m1 z17 pa20", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "coarse m2 z13", Params: map[string]float64{
		pModule: 2, pToothNumber: 13, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "fine m0.5 z35", Params: map[string]float64{
		pModule: 0.5, pToothNumber: 35, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "positive helix 25deg", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: 25 * math.Pi / 180}},
	{Name: "negative helix -25deg", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: -25 * math.Pi / 180}},
	{Name: "quarter turn +90deg", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: math.Pi / 2}},
	{Name: "quarter turn -90deg", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: -math.Pi / 2}},
	{Name: "low involute steps", Params: map[string]float64{
		pModule: 1, pToothNumber: 17, pPressureAngle: math.Pi / 9, pInvoluteSteps: 3, pAngle: 0}},
	{Name: "embedded by tooth count z50 pa20", Params: map[string]float64{
		pModule: 1, pToothNumber: 50, pPressureAngle: math.Pi / 9, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "embedded by pressure angle z30 pa25", Params: map[string]float64{
		pModule: 1, pToothNumber: 30, pPressureAngle: 25 * math.Pi / 180, pInvoluteSteps: 15, pAngle: 0}},
}

// stepGearProfileSketch models step 9, the Gear Profile sketch: the four
// circles, the involute tooth, the tooth-top arc, spine, ribs and
// flank-to-root closing, and the step-5 anchoring — the full constraint
// scheme of instructions.md steps 3-5 realised through the recipes in
// fusion.md. proofkit gates the result on full constraint, no redundancy, no
// conflict, valid profiles and no discrete ambiguity, so a scheme that
// solves only on one branch fails here.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := geomOf(t, p)
	d := g.dims

	proofkit.Step(t, "local origin and projected anchor")
	// [SPUR-F-LOCAL-ORIGIN]: a fresh movable point at (0,0), never the
	// sketch's own origin. The Tools-sketch anchor arrives as a projection.
	origin := s.CreatePoint(0, 0)
	origin.SetName("localOrigin")
	anchor := s.CreateReferencePoint(0, 0, "toolsAnchor")
	anchor.SetName("projectedAnchor")

	proofkit.Step(t, "four circles centred on the shared local origin")
	// Instructions step 3: root solid, tip/base/pitch construction, each
	// with a driving diameter dimension, all sharing the origin point
	// ([PB-SHARE-XOR-COINCIDENT] — share, never re-coincident).
	rootC := s.CreateCircle(origin, d.Root)
	tipC := s.CreateCircle(origin, d.Tip)
	baseC := s.CreateCircle(origin, d.Base)
	pitchC := s.CreateCircle(origin, d.Pitch)
	for _, c := range []*sketch.Circle{tipC, baseC, pitchC} {
		c.SetConstruction(true)
	}
	s.AddConstraint(
		sketch.NewDiameter(rootC, 2*d.Root),
		sketch.NewDiameter(tipC, 2*d.Tip),
		sketch.NewDiameter(baseC, 2*d.Base),
		sketch.NewDiameter(pitchC, 2*d.Pitch),
	)

	proofkit.Step(t, "involute flanks as fitted splines")
	// Instructions step 4.1-4.5: endpoint-inclusive sampling base-to-tip,
	// below-base samples dropped, mirrored, centred so the pitch crossing
	// lands at +pi/(2N), rotated by the requested angle — all in
	// proof/involute — then drawn as fitted splines through shared points.
	left, right := g.left, g.right
	if len(left) < 2 {
		t.Fatalf("flank sampling produced %d points", len(left))
	}
	lpts := make([]*sketch.Point, len(left))
	rpts := make([]*sketch.Point, len(right))
	for i := range left {
		lpts[i] = s.CreatePoint(left[i].X, left[i].Y)
		rpts[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(lpts...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(rpts...); err != nil {
		t.Fatalf("right flank: %v", err)
	}

	// The spiral-direction pin (instructions step 4.2): after the mirror the
	// left flank's angular offset above the tooth axis must strictly
	// decrease as radius grows — the un-mirrored involute widens instead.
	for i := 1; i < len(left); i++ {
		prev := math.Atan2(left[i-1].Y, left[i-1].X) - g.angle
		cur := math.Atan2(left[i].Y, left[i].X) - g.angle
		if !(cur < prev+1e-12) {
			t.Errorf("left flank angular offset grew at sample %d: %.6f -> %.6f", i, prev, cur)
		}
	}
	// Tooth symmetry about the angle direction (step 4.4).
	for i := range left {
		la := math.Atan2(left[i].Y, left[i].X) - g.angle
		ra := math.Atan2(right[i].Y, right[i].X) - g.angle
		if math.Abs(la+ra) > 1e-9 {
			t.Errorf("flank pair %d not symmetric about the angle direction: %.9f vs %.9f", i, la, ra)
		}
	}
	// The centring rule (step 4.3): the flank crosses the pitch circle at
	// +pi/(2N) above the tooth axis. Interpolated between the two samples
	// bracketing the pitch radius, so the tolerance is the chord error.
	assertPitchCrossing(t, g)

	proofkit.Step(t, "tooth-top arc centred on the shared origin")
	// [SPUR-F-TOOTHTOP-ARC]: pass the origin and the two flank end points
	// directly; no diameter dimension — a free centre plus a diameter
	// admits the inward bulge.
	last := len(left) - 1
	s.CreateArc(origin, rpts[last], lpts[last])

	proofkit.Step(t, "tooth-top point, spine, +X reference, angular pin")
	// [SPUR-F-SPINE]: the tooth-top point sits at the tip radius rotated by
	// angle, on the tip circle; the spine shares origin and tooth-top; the
	// +X reference's far end is pinned by two axis dimensions (not
	// point-on-circle, whose extreme-x touch point is numerically unstable);
	// the angular dimension from reference to spine carries the SIGNED
	// angle, which is what forbids the mirrored answer — a plain horizontal
	// would leave the 180-degree branch open. Built for every angle,
	// including 0 ([SPUR-F-ROTATE-CONFIRM]).
	top := s.CreatePoint(d.Tip*math.Cos(g.angle), d.Tip*math.Sin(g.angle))
	top.SetName("toothTop")
	s.AddConstraint(sketch.NewPointOnCircle(top, tipC))
	spine := s.CreateLine(origin, top)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	refEnd.SetName("refEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	refLine := s.CreateLine(origin, refEnd)
	refLine.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(refLine, spine, g.angle*180/math.Pi))

	proofkit.Step(t, "one rib per fit-point pair, midpoint chained down the spine")
	// [SPUR-F-RIBS]: a rib per fit-point index, endpoints included; an axis
	// dimension across the spine and one along it, axes swapped past the
	// quarter turn; the midpoint seeded at the foot of the left fit point on
	// the spine; the chain starts at the local origin; the LAST rib carries
	// no perpendicular (the tooth-top arc's shared centre already implies
	// it, and keeping both over-constrains).
	//
	// Fusion's dimension values are unsigned magnitudes whose direction is
	// captured from the seed ([PB-DIM-VALUE-SEMANTICS]); the bench engine's
	// distance targets are SIGNED, so the seed side crosses over as the sign
	// here — never as a negative Fusion parameter value.
	ca, sa := math.Cos(g.angle), math.Sin(g.angle)
	acrossVertical := math.Abs(ca) >= math.Abs(sa)
	prev := origin
	prevT := 0.0
	for i := range lpts {
		rib := s.CreateLine(lpts[i], rpts[i])
		rib.SetConstruction(true)
		if acrossVertical {
			s.AddConstraint(sketch.NewVerticalDistance(lpts[i], rpts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lpts[i], rpts[i], right[i].X-left[i].X))
		}
		foot := left[i].X*ca + left[i].Y*sa
		mid := s.CreatePoint(foot*ca, foot*sa)
		mid.SetName(fmt.Sprintf("ribMid%d", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, (foot-prevT)*ca))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, (foot-prevT)*sa))
		}
		prev = mid
		prevT = foot
	}

	proofkit.Step(t, "close the tooth at the root")
	// [SPUR-F-FLANK-ROOT]: embedded is a strict less-than on the first
	// drawn sample's radius. Non-embedded: one radial stub per side, its
	// root end pinned by exactly two axis dimensions from the origin —
	// root-end-on-circle plus origin-on-line also reaches DOF 0 but admits
	// the far intersection, and the stub runs across the gear.
	firstRadius := math.Hypot(left[0].X, left[0].Y)
	embedded := firstRadius < d.Root
	if embedded != g.dims.Embedded() {
		t.Errorf("drawn embedded test (%v) disagrees with the radii (%v)", embedded, g.dims.Embedded())
	}
	if wantEmbedded := g.toothNumber*(1-math.Cos(g.pressureAngle)) > 2.5; embedded != wantEmbedded {
		t.Errorf("embedded (%v) disagrees with the closed form z(1-cos pa) > 2.5 (%v)", embedded, wantEmbedded)
	}
	if !embedded {
		for _, side := range []struct {
			start  *sketch.Point
			sx, sy float64
		}{{lpts[0], left[0].X, left[0].Y}, {rpts[0], right[0].X, right[0].Y}} {
			rex := d.Root * side.sx / firstRadius
			rey := d.Root * side.sy / firstRadius
			re := s.CreatePoint(rex, rey)
			s.CreateLine(re, side.start)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, re, rex),
				sketch.NewVerticalDistance(origin, re, rey),
			)
		}
	}

	proofkit.Step(t, "anchor the sketch (step 5)")
	// The coincidence between the local origin and the projected anchor is
	// what slides the whole drawing onto the user's anchor; everything above
	// hangs off the origin.
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the two regions and their curve counts")
	assertGearProfileRegions(t, s, g)
}

// assertPitchCrossing checks the left flank crosses the pitch circle at
// +pi/(2N) above the tooth axis, interpolating between the drawn samples.
func assertPitchCrossing(t testing.TB, g *gearGeom) {
	t.Helper()
	target := math.Pi / (2 * g.toothNumber)
	left := g.left
	for i := 1; i < len(left); i++ {
		r0 := math.Hypot(left[i-1].X, left[i-1].Y)
		r1 := math.Hypot(left[i].X, left[i].Y)
		if r0 <= g.dims.Pitch && g.dims.Pitch <= r1 {
			a0 := math.Atan2(left[i-1].Y, left[i-1].X) - g.angle
			a1 := math.Atan2(left[i].Y, left[i].X) - g.angle
			f := (g.dims.Pitch - r0) / (r1 - r0)
			got := a0 + f*(a1-a0)
			if math.Abs(got-target) > 0.1*target {
				t.Errorf("pitch crossing at %.6f rad above the axis, want %.6f", got, target)
			}
			return
		}
	}
	t.Errorf("no flank samples bracket the pitch radius")
}

// assertGearProfileRegions counts the curves on the two regions the sketch
// closes — the contract the extrude steps match on (instructions.md "The
// Gear Profile sketch closes exactly two regions"). The counts are asserted
// on distinct boundary entities: profile detection splits the solid root
// circle where the tooth loop meets it, and a split curve is still ONE
// curve. The disc's boundary reads here as that one root circle where
// Fusion's profile API reports the same boundary as its two split arcs; the
// fact carried is the same — the disc is bounded by the root circle and
// nothing else.
func assertGearProfileRegions(t testing.TB, s *sketch.Sketch, g *gearGeom) {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Errorf("gear profile sketch closed %d regions, want exactly 2", len(profiles))
		return
	}
	var tooth, disc *sketch.Profile
	for _, p := range profiles {
		splines := 0
		for _, e := range p.Entities {
			if _, ok := e.(*sketch.FitSpline); ok {
				splines++
			}
		}
		if splines == 2 {
			tooth = p
		} else {
			disc = p
		}
	}
	if tooth == nil || disc == nil {
		t.Errorf("could not tell the tooth region from the disc region")
		return
	}
	lines, arcs, splines := 0, 0, 0
	for _, e := range tooth.Entities {
		switch e.(type) {
		case *sketch.Line:
			lines++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.FitSpline:
			splines++
		}
	}
	wantLines := 2
	if g.embedded() {
		wantLines = 0
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("tooth loop has %d splines, %d arcs, %d lines; want 2, 2, %d",
			splines, arcs, lines, wantLines)
	}
	if len(disc.Entities) != 1 {
		t.Errorf("disc region bounded by %d entities, want the root circle alone", len(disc.Entities))
	} else if _, ok := disc.Entities[0].(*sketch.Circle); !ok {
		t.Errorf("disc region boundary is %T, want the root circle", disc.Entities[0])
	}
	if want := math.Pi * g.dims.Root * g.dims.Root; relDiff(disc.Area, want) > 1e-6 {
		t.Errorf("disc region area %.6f, want pi*root^2 = %.6f", disc.Area, want)
	}
}

// boreCases: bore diameters with the anchor on and off the plane origin. The
// zero-diameter dialog default never reaches this step — buildBore
// early-returns on BoreDiameter <= 0 (steps.md step 18).
var boreCases = []proofkit.Case{
	{Name: "2mm bore at origin", Params: map[string]float64{
		pBoreDiameter: 2, pAnchorX: 0, pAnchorY: 0}},
	{Name: "6mm bore off origin", Params: map[string]float64{
		pBoreDiameter: 6, pAnchorX: 30, pAnchorY: -20}},
}

// stepBoreProfileSketch models step 18's sketch: the Bore Profile sketch
// re-projects the Tools-sketch anchor, draws the bore circle centred on the
// projection (shared, per [PB-SHARE-XOR-COINCIDENT]) with a driving diameter
// dimension, and grounds the tooth generator constructor's stray local
// origin on that same projection — not on the sketch's own origin point,
// which pins it to the plane instead of the gear ([PB-CIRCLE-CENTER]).
// Without the grounding the stray point keeps two free DOF and the sketch
// never fully constrains, which is exactly what the gate checks.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "re-project the anchor")
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "toolsAnchor")
	anchor.SetName("projectedAnchor")

	proofkit.Step(t, "the constructor's stray local origin, grounded on the projection")
	origin := s.CreatePoint(0, 0)
	origin.SetName("strayLocalOrigin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "bore circle centred on the projected anchor")
	c := s.CreateCircle(anchor, p[pBoreDiameter]/2)
	s.AddConstraint(sketch.NewDiameter(c, p[pBoreDiameter]))

	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Errorf("bore sketch closed %d regions, want the bore disc alone", len(profiles))
	} else if want := math.Pi * p[pBoreDiameter] * p[pBoreDiameter] / 4; relDiff(profiles[0].Area, want) > 1e-6 {
		t.Errorf("bore area %.6f, want %.6f", profiles[0].Area, want)
	}
}
