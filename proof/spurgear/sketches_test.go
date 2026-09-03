// Package spurgear_test proves the spur gear's build, one function per step of
// spec/spurgear/steps.md.
//
// This file holds the sketch step. The spur gear has exactly one
// constraint-bearing sketch, the Gear Profile, and everything the spec's
// Sketch Discipline section pins about it is proved here: the scheme fully
// constrains across the whole regime, and the sketch closes the two regions the
// two extrude steps select on.
//
// This file holds the three sketch steps: the Tools sketch, the Gear Profile
// sketch and the Bore Profile sketch.
//
// Two things in them are outside what a sketch engine can hold, and each is
// recorded beside the thing it cannot reach.
//
// The four circle labels are sketch text. This engine has no sketch
// text, and in Fusion text carries its own position along the curve and is
// never pinned, so a labelled sketch never reports isFullyConstrained even when
// its geometry is completely determined ([PB-TEXT-HOLDS-DOF]). A proof written
// against a sketch engine cannot reproduce that, and a gear that labels a
// sketch cannot gate on isFullyConstrained in Fusion either.
//
// The negative control for the tooth-top arc's free centre
// ([SPUR-F-TOOTHTOP-ARC]) is not here. proofkit has no expect-failure mode: a
// case that must NOT fully constrain cannot be written as a proofkit run
// without inverting its gate. That control lives in the committed bench,
// spec/spurgear/sketch/main.go, which re-runs one case with the coincidence
// dropped and requires DOF 2.
package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// profileCases sweeps the regime the Sketch Discipline section declares the
// scheme has to hold across, because proving one gear proves nothing about the
// next one.
//
//   - size: several Module / Tooth Number pairs, coarse and fine, since the rib
//     chain's dimensions scale with the tooth and the conditioning does not;
//   - the whole SIGNED range of the angle argument: 0 for spur, a positive and a
//     negative helix angle, a quarter turn either way where |sin| > |cos| swaps
//     which axis the rib and chain dimensions take, and the half turn the bevel
//     virtual tooth draws. A scheme that dropped or flipped the confirming
//     angular dimension still solves at +angle and comes out mirrored at -angle,
//     so the negative cases are not decoration;
//   - the rib count, at the low end as well as the standard 15, where one missing
//     or redundant dimension is a large fraction of the system;
//   - BOTH routes into the embedded shape: a high tooth count at the ordinary 20
//     degree pressure angle, and a moderate tooth count at a large pressure
//     angle. The two arrive at the same missing-stub geometry through different
//     terms.
var profileCases = []proofkit.Case{
	{Name: "M1_N12_flat", Params: params(1, 12, 20, 0, 15)},
	{Name: "M1_N17_flat_default", Params: params(1, 17, 20, 0, 15)},
	{Name: "M2_N20_flat", Params: params(2, 20, 20, 0, 15)},
	{Name: "M3_N15_flat", Params: params(3, 15, 20, 0, 15)},
	{Name: "M2_N20_plus30", Params: params(2, 20, 20, 30, 15)},
	{Name: "M3_N15_minus60", Params: params(3, 15, 20, -60, 15)},
	{Name: "M1_N17_plus_quarter_turn", Params: params(1, 17, 20, 90, 15)},
	{Name: "M1_N17_minus_quarter_turn", Params: params(1, 17, 20, -90, 15)},
	{Name: "M1_N17_four_samples", Params: params(1, 17, 20, 0, 4)},
	{Name: "M1_N17_two_samples", Params: params(1, 17, 20, 0, 2)},
	{Name: "M2_N20_plus30_five_samples", Params: params(2, 20, 20, 30, 5)},
	{Name: "embedded_high_count_M1_N43_flat", Params: params(1, 43, 20, 0, 15)},
	{Name: "embedded_high_count_M1_N43_half_turn", Params: params(1, 43, 20, 180, 15)},
	{Name: "embedded_high_count_M1_N50_flat", Params: params(1, 50, 20, 0, 15)},
	{Name: "embedded_high_count_M2_N60_plus30", Params: params(2, 60, 20, 30, 15)},
	{Name: "embedded_large_pressure_angle_PA30_N20_flat", Params: params(1, 20, 30, 0, 15)},
	{Name: "embedded_large_pressure_angle_PA25_N30_minus45", Params: params(1, 30, 25, -45, 15)},
	{Name: "M1_N17_anchor_off_origin", Params: at(params(1, 17, 20, 0, 15), 8, -5)},
	{Name: "M2_N20_plus30_anchor_off_origin", Params: at(params(2, 20, 20, 30, 15), -12, 7)},
}

// params names one case by the dialog values it comes from. Module and Tooth
// Number are the two size inputs, the pressure angle and the tooth angle are
// given in degrees and carried in radians, and involuteSteps is the derived
// Involute Steps parameter, 15 in the shipped gear.
func params(module, toothNumber, pressureAngleDeg, angleDeg float64, steps int) map[string]float64 {
	return map[string]float64{
		"module":        module,
		"toothNumber":   toothNumber,
		"pressureAngle": rad(pressureAngleDeg),
		"angle":         rad(angleDeg),
		"involuteSteps": float64(steps),
		"anchorX":       0,
		"anchorY":       0,
	}
}

// at moves a case's anchor off the sketch origin. The tooth is drawn about the
// local origin and only then dragged onto the anchor, so the drag distance is a
// parameter of the scheme: it is what strands a copied arc centre, and it is
// zero in every case where the user's anchor happens to sit on the sketch
// origin. The spur dialog does not require that, so the sweep carries a case
// where it does not.
func at(p map[string]float64, x, y float64) map[string]float64 {
	p["anchorX"], p["anchorY"] = x, y
	return p
}

func rad(deg float64) float64 { return deg * math.Pi / 180 }

func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	module := p["module"]
	toothNumber := p["toothNumber"]
	pressureAngle := p["pressureAngle"]
	angle := p["angle"]
	steps := int(p["involuteSteps"])
	d := involute.Derive(module, toothNumber, pressureAngle)

	proofkit.Step(t, "local origin")
	origin := s.CreatePoint(0, 0)

	proofkit.Step(t, "four circles")
	mk := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	mk(d.Root, false)
	tip := mk(d.Tip, true)
	mk(d.Base, true)
	mk(d.Pitch, true)

	proofkit.Step(t, "involute flanks")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothNumber, steps, angle)
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

	proofkit.Step(t, "tooth-top arc")
	topX, topY := involute.Rotate(d.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))

	proofkit.Step(t, "spine and angular pin")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	proofkit.Step(t, "ribs")
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prev := origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}
		tt := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		mx, my := tt*math.Cos(angle), tt*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, my-prevY))
		}
		prev, prevX, prevY = mid, mx, my
	}

	proofkit.Step(t, "flank-to-root lines")
	if !d.Embedded() {
		foot := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := d.Root*seed.X/n, d.Root*seed.Y/n
			re := s.CreatePoint(rx, ry)
			s.CreateLine(re, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, re, rx),
				sketch.NewVerticalDistance(origin, re, ry),
			)
		}
		foot(leftPts[0], left[0])
		foot(rightPts[0], right[0])
	}

	proofkit.Step(t, "anchor the sketch")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "profile contract")
	assertProfileContract(t, s, d.Root, d.Embedded())
}

// entityCounts classifies a region's DISTINCT boundary entities the way
// find_profile_by_curve_counts classifies Fusion's profile curves: a fitted
// spline is a NURBS, the tooth-top arc and the root circle are each an arc, and
// a flank-to-root stub is a line.
//
// Profile.Entities is the de-duplicated entity set, and the de-duplication is
// what makes the two engines comparable. Fusion splits the solid root circle
// where the tooth meets it and the tooth loop takes ONE of the pieces, so the
// circle contributes one arc there. The sketch engine parameterises a circle
// from +X, so a tooth drawn across that seam reports the same single piece as
// two fragments of the one *Circle entity. Counting entities rather than
// boundary edges reads the piece once, which is the count Fusion sees.
func entityCounts(entities []sketch.Entity) (nurbs, arcs, lines int) {
	for _, entity := range entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}

// assertProfileContract holds the drawn sketch to the two regions the extrude
// steps select on, on the geometry the proof actually drew rather than on a
// stand-in outline.
//
// The tooth is the count step 7 keys on: 2 NURBS + 2 arcs + 2 lines, or 2 NURBS
// + 2 arcs where the flank starts inside the root circle and no stub is drawn.
//
// The gear-body disc is where the two engines part company, so the proof says
// what it can prove and no more. Step 9 keys on a boundary of exactly 2 arcs —
// the two pieces the tooth cuts the solid root circle into. The sketch engine
// splits a curve only where a region needs it, so the disc inside the root
// circle comes back bounded by the ONE whole *Circle entity, of the root
// circle's own area, and the two-arc decomposition of that same circle is a
// Fusion-side fact this harness cannot reach. What is proved here is that the
// region exists, that it is the disc inside the root circle and not an annulus
// (the tip circle is construction and bounds nothing), and that it is
// extrudable; the arc COUNT on it is the part only a Fusion session settles.
func assertProfileContract(t testing.TB, s *sketch.Sketch, root float64, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before reading profiles: %v", err)
	}
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	tooth, disc := 0, 0
	regions := s.Profiles()
	for _, profile := range regions {
		nurbs, arcs, lines := entityCounts(profile.Entities)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth++
		case nurbs == 0 && arcs == 1 && lines == 0:
			disc++
			if want := math.Pi * root * root; math.Abs(profile.Area-want) > 1e-6*want {
				t.Errorf("disc region area %.6f mm2, want the root circle's %.6f mm2", profile.Area, want)
			}
		default:
			t.Errorf("unexpected region: %d NURBS, %d arcs, %d lines", nurbs, arcs, lines)
		}
		if !profile.Valid {
			t.Errorf("region with %d NURBS, %d arcs, %d lines is not extrudable", nurbs, arcs, lines)
		}
	}
	if len(regions) != 2 {
		t.Errorf("closed regions: %d, want exactly 2 (the tooth and the disc)", len(regions))
	}
	if tooth != 1 {
		t.Errorf("tooth regions of 2 NURBS, 2 arcs, %d lines: %d, want 1", wantLines, tooth)
	}
	if disc != 1 {
		t.Errorf("disc regions inside the root circle: %d, want 1", disc)
	}
}

// toolsProjectionSource is the source id every projection of the user's anchor
// carries. In Fusion the Tools sketch's projection is the canonical handle and
// every later sketch re-projects THAT ([SPUR-F-ANCHOR-CHAIN]); the engine
// refuses a reference to another sketch's point outright, reporting it as a
// foreign handle, so each sketch below carries its own reference point tagged
// with this id. What that models is one link of the chain, not the chain.
const toolsProjectionSource = "Tools sketch anchor projection"

// toolsCases put the user's anchor on the sketch origin and off it. Nothing in
// the dialog requires the anchor to sit at the origin, and where it does not,
// every later sketch is dragged onto it.
var toolsCases = []proofkit.Case{
	{Name: "anchor_on_sketch_origin", Params: map[string]float64{"anchorX": 0, "anchorY": 0}},
	{Name: "anchor_off_sketch_origin", Params: map[string]float64{"anchorX": 8, "anchorY": -5}},
}

// stepToolsSketch builds the Tools sketch: one projection of the user's Anchor
// Point, and no geometry of its own.
//
// The sketch is nearly empty, so most of this step is about what the projection
// is FOR, which is checked on scratch sketches beside it: a later sketch's local
// origin made coincident to the projection reaches DOF 0, and the same local
// origin left unanchored keeps the two degrees of freedom it was born with. That
// pair is the whole content of the anchoring rule, and it is why the Gear
// Profile and Bore Profile sketches each ground their own local origin.
//
// Two things this step cannot reach, and they are the reason it proves what it
// proves rather than more.
//
// [PB-PROJECT-NOT-FIXED] says a Fusion projection arrives associatively and
// still carries free degrees of freedom, so a sketch hanging off one reports
// under-constrained until something pins it. The engine's counterpart,
// CreateReferencePoint, is coordinate-LOCKED: the solver never moves it. So the
// projection is modelled as already pinned, and the free-DOF half of that rule
// is not reproduced here — only its consequence, that the local origin hanging
// off it needs a constraint of its own.
//
// [SPUR-F-ANCHOR-CHAIN]'s chain is not reproduced either. A sketch may not
// reference another sketch's point in this engine any more than in Fusion, and
// the engine reports such a handle as foreign rather than resolving it, so what
// each sketch below carries is its own local endpoint of the chain.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Anchor Point into the Tools sketch")
	s.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)

	if got := len(s.Entities()); got != 0 {
		t.Errorf("Tools sketch holds %d drawn entities, want none: it exists to own the projection", got)
	}
	if got := len(s.Points()); got != 1 {
		t.Errorf("Tools sketch holds %d points, want the one projected anchor", got)
	}

	proofkit.Step(t, "what a later sketch does with the projection")
	assertAnchoringGrounds(t, p)
}

// assertAnchoringGrounds checks the two halves of the anchoring rule on scratch
// sketches: a local origin made coincident to the projection is fully
// constrained, and the same local origin without that constraint is not.
//
// The comparison the spec draws against grounding on the sketch's own origin
// point is NOT reachable. [PB-CIRCLE-CENTER] records a Fusion solver failure
// from constraining to originPoint, and the spec forbids grounding the Bore
// Profile's stray point that way because it pins the point to the plane rather
// than to the gear. Measured here, grounding on the engine's own origin reaches
// DOF 0 exactly as grounding on the projection does: the engine has no notion of
// a constraint that solves but tracks the wrong thing. Only a Fusion session
// tells those two apart.
func assertAnchoringGrounds(t testing.TB, p map[string]float64) {
	t.Helper()

	anchored := proofkit.NewSketch(t)
	anchoredProjection := anchored.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	anchoredOrigin := anchored.CreatePoint(0, 0)
	anchored.AddConstraint(sketch.NewCoincident(anchoredOrigin, anchoredProjection))
	if dof := solvedDOF(t, anchored); dof != 0 {
		t.Errorf("a local origin anchored to the projection has DOF %d, want 0", dof)
	}

	free := proofkit.NewSketch(t)
	free.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	free.CreatePoint(0, 0)
	if dof := solvedDOF(t, free); dof != 2 {
		t.Errorf("a local origin left unanchored has DOF %d, want the 2 it was born with", dof)
	}
}

// solvedDOF solves a scratch sketch and reports the degrees of freedom the
// engine finds left in it.
func solvedDOF(t testing.TB, s *sketch.Sketch) int {
	t.Helper()
	ctx := context.Background()
	if _, err := s.Solve(ctx); err != nil {
		t.Fatalf("solve scratch sketch: %v", err)
	}
	report := s.Verify(ctx)
	if !report.Analysed() {
		t.Fatal("scratch sketch was not analysed, so its DOF reading means nothing")
	}
	return report.DOF
}

// boreProfileCases sweep the bore sizes the dialog accepts above zero, with the
// anchor both on the sketch origin and off it. Bore Diameter 0 draws no sketch
// at all, so it is not a case here; that branch is the bore cut step's.
var boreProfileCases = []proofkit.Case{
	{Name: "bore4_anchor_on_origin", Params: map[string]float64{"boreDiameter": 4, "anchorX": 0, "anchorY": 0}},
	{Name: "bore4_anchor_off_origin", Params: map[string]float64{"boreDiameter": 4, "anchorX": 8, "anchorY": -5}},
	{Name: "bore0_5_anchor_off_origin", Params: map[string]float64{"boreDiameter": 0.5, "anchorX": -3, "anchorY": 11}},
	{Name: "bore20_anchor_on_origin", Params: map[string]float64{"boreDiameter": 20, "anchorX": 0, "anchorY": 0}},
}

// stepBoreProfileSketch builds the Bore Profile sketch: the projected anchor,
// the bore circle centred on it with a driving diameter dimension, and the
// stray local-origin point the tooth generator's constructor always adds.
//
// The stray point is the whole reason this sketch is worth proving. It is
// faithful behaviour that it exists, and it is free in two directions until it
// is grounded on the same projection the circle is centred on, which is what
// the check below measures both ways.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the anchor and draw the bore circle on it")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	bore := s.CreateCircle(anchor, p["boreDiameter"]/2)
	s.AddConstraint(sketch.NewDiameter(bore, p["boreDiameter"]))

	proofkit.Step(t, "ground the tooth generator's stray local origin")
	localOrigin := s.CreatePoint(0, 0)
	s.AddConstraint(sketch.NewCoincident(localOrigin, anchor))

	proofkit.Step(t, "the region the bore cut consumes")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve bore profile: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("Bore Profile holds %d regions, want the one bore disc", len(regions))
	}
	want := math.Pi * p["boreDiameter"] * p["boreDiameter"] / 4
	if got := regions[0].Area; math.Abs(got-want) > 1e-9*want {
		t.Errorf("bore region area %.9f mm2, want %.9f mm2", got, want)
	}
	if !regions[0].Valid {
		t.Error("bore region is not extrudable")
	}

	proofkit.Step(t, "the same sketch with the stray point left free")
	loose := proofkit.NewSketch(t)
	looseAnchor := loose.CreateReferencePoint(p["anchorX"], p["anchorY"], toolsProjectionSource)
	looseBore := loose.CreateCircle(looseAnchor, p["boreDiameter"]/2)
	loose.AddConstraint(sketch.NewDiameter(looseBore, p["boreDiameter"]))
	loose.CreatePoint(0, 0)
	if dof := solvedDOF(t, loose); dof != 2 {
		t.Errorf("the ungrounded stray point leaves DOF %d, want 2", dof)
	}
}
