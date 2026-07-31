// Package spurgear_test proves the spur gear's constrained sketches.
//
// Each [GO] step of spurgear.steps.md has one build function here, named
// step<Title> after the step's title, and every build function is named by a
// step. The 3D steps have no build function: the engine models 2D sketches
// only, so an extrude, a chamfer, a pattern or a fillet cannot be exercised
// here and is written as [PROSE] in the step list instead.
//
// What the proof models, and where it departs from Fusion:
//
//   - A projected point is reference geometry (CreateReferencePoint), because
//     Fusion's projection is positioned from outside the sketch. The tooth
//     generator's local origin is an ordinary point made coincident with it,
//     exactly as [SPUR-F-LOCAL-ORIGIN] and step 5 of the spec describe.
//   - Construction circles and construction lines are marked construction, so
//     region detection sees only what Fusion's profile search sees.
//   - No root arc is drawn by hand. The root circle is a whole solid circle and
//     region detection splits it where the tooth meets it, which is what Fusion
//     does when it finds the tooth and body profiles.
//   - The confirming angular dimension is created already carrying its value.
//     Fusion sets that value as a separate last action to move the solver onto
//     the intended branch ([SPUR-F-ROTATE-CONFIRM]); the other half of that
//     rule — the geometry being drawn pre-rotated — is what the proof models,
//     by seeding every point at its rotated position.
package spurgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys of a case. Lengths are millimetres and angles degrees, which
// is what the sketch engine's metric defaults use; the Fusion code works in
// internal centimetres and radians, a conversion that changes no constraint.
const (
	pModule      = "module"
	pTeeth       = "toothNumber"
	pPressureDeg = "pressureAngleDeg"
	pSteps       = "involuteSteps"
	pAngleDeg    = "angleDeg"
	pBoreDia     = "boreDiameter"
)

// cases sweeps the sizes the scheme has to hold across.
//
// The first four are the required sweep. The rest reach branches those do not:
// the embedded profile (base circle inside the root circle, no flank-to-root
// stubs), a rotated tooth in each of the two dimension orientations the rib
// recipe chooses between, a non-default pressure angle, a coarse involute
// sample count, and a gear with no bore.
func cases() []proofkit.Case {
	return []proofkit.Case{
		{Name: "m1_t12", Params: params(1, 12, 20, 15, 0, 4)},
		{Name: "m1_t17", Params: params(1, 17, 20, 15, 0, 5)},
		{Name: "m2_t20", Params: params(2, 20, 20, 15, 0, 8)},
		{Name: "m3_t15", Params: params(3, 15, 20, 15, 0, 12)},

		{Name: "m1_t60_embedded", Params: params(1, 60, 20, 15, 0, 20)},
		{Name: "m2_t100_embedded", Params: params(2, 100, 20, 15, 0, 40)},
		{Name: "m2_t20_rot30", Params: params(2, 20, 20, 15, 30, 8)},
		{Name: "m1_t17_rot90", Params: params(1, 17, 20, 15, 90, 5)},
		{Name: "m2_t20_rot145", Params: params(2, 20, 20, 15, 145, 8)},
		{Name: "m1_t17_pa14_5", Params: params(1, 17, 14.5, 15, 0, 5)},
		{Name: "m1_t17_pa25", Params: params(1, 17, 25, 15, 0, 5)},
		{Name: "m2_t20_steps4", Params: params(2, 20, 20, 4, 0, 8)},
		{Name: "m3_t15_nobore", Params: params(3, 15, 20, 15, 0, 0)},
	}
}

func params(module, teeth, pressureDeg, steps, angleDeg, boreDia float64) map[string]float64 {
	return map[string]float64{
		pModule:      module,
		pTeeth:       teeth,
		pPressureDeg: pressureDeg,
		pSteps:       steps,
		pAngleDeg:    angleDeg,
		pBoreDia:     boreDia,
	}
}

func TestToolsSketch(t *testing.T)       { proofkit.Run(t, cases(), stepToolsSketch) }
func TestGearProfileSketch(t *testing.T) { proofkit.Run(t, cases(), stepGearProfileSketch) }
func TestBoreProfileSketch(t *testing.T) { proofkit.Run(t, cases(), stepBoreProfileSketch) }

// stepToolsSketch realises step S3.
//
// The Tools sketch draws no geometry of its own. It exists to own one entity:
// the projection of the user's anchor point, which every later sketch projects
// in again. Modelling it is worth the line it takes, because it states what the
// canonical handle is and shows that a sketch holding only it is sound.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the anchor point into the Tools sketch")
	anchor := s.CreateReferencePoint(0, 0, "user anchor point")
	anchor.SetName("ctx.anchorPoint")
}

// stepGearProfileSketch realises step S5 — the whole Gear Profile sketch, which
// is one Fusion timeline entry however much goes into it: the four circles, the
// involute tooth, and the anchoring that slides the drawing onto the user's
// anchor.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	var (
		module   = p[pModule]
		teeth    = p[pTeeth]
		pressure = radians(p[pPressureDeg])
		steps    = int(p[pSteps])
		angle    = radians(p[pAngleDeg])
	)
	d := involute.Derive(module, teeth, pressure)

	// The tooth generator's constructor adds the local origin, and everything
	// below is drawn relative to it. It is an ordinary movable point, not the
	// sketch's own origin point, so that the anchoring at the end can drag the
	// whole drawing onto the user's anchor as a unit ([SPUR-F-LOCAL-ORIGIN]).
	proofkit.Step(t, "constructor: local origin at (0, 0)")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")

	proofkit.Step(t, "drawCircles: root solid, tip/base/pitch construction, all centred on the local origin")
	circle(s, origin, "root", d.Root, false)
	tip := circle(s, origin, "tip", d.Tip, true)
	circle(s, origin, "base", d.Base, true)
	circle(s, origin, "pitch", d.Pitch, true)

	proofkit.Step(t, "drawTooth: %d involute samples per flank at %.4g rad", steps, angle)
	leftPts, rightPts := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	if len(leftPts) < 2 {
		proofkit.Unmodelled(t, "only %d involute sample(s) survived, too few for a flank", len(leftPts))
	}
	left := make([]*sketch.Point, len(leftPts))
	right := make([]*sketch.Point, len(rightPts))
	for i := range leftPts {
		left[i] = named(s, leftPts[i], "left flank fit %d", i)
		right[i] = named(s, rightPts[i], "right flank fit %d", i)
	}
	leftFlank, err := s.CreateFitSpline(left...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(right...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("right flank")

	// The tooth-top arc caps the tooth at the tip circle, so it is part of that
	// circle and must bulge outward. Sharing the local origin as its centre is
	// what says so: a free centre with a diameter dimension reaches DOF 0 with
	// the inward-bulging arc equally valid ([SPUR-F-TOOTHTOP-ARC]).
	proofkit.Step(t, "tooth-top point on the tip circle, and the arc centred on the local origin")
	last := len(left) - 1
	toothTop := named(s, involute.Pt{X: d.Tip * math.Cos(angle), Y: d.Tip * math.Sin(angle)}, "tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arc := s.CreateArc(origin, right[last], left[last])
	arc.SetName("tooth top arc")

	// The spine says which way the tooth points. A horizontal constraint would
	// fix the direction but not the sense, leaving the tooth free to settle
	// 180 degrees around, so the angle is dimensioned against a reference line
	// that is itself pinned to +X by two signed offsets ([SPUR-F-SPINE]).
	proofkit.Step(t, "spine, +X reference line and the confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetName("spine")
	spine.SetConstruction(true)
	refEnd := named(s, involute.Pt{X: d.Tip, Y: 0}, "+X reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	refLine := s.CreateLine(origin, refEnd)
	refLine.SetName("+X reference")
	refLine.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(refLine, spine, p[pAngleDeg]))

	// One rib per fit-point index, endpoints included: the fit points carry no
	// other constraint, so a missing rib leaves one free ([SPUR-F-RIBS]). The
	// last rib takes no perpendicular — the tooth-top arc already holds its two
	// ends at equal radius either side of the spine, so a perpendicular there
	// would be redundant.
	proofkit.Step(t, "%d ribs, midpoints on the spine, last one without a perpendicular", len(left))
	alongSpine := signedAlong(angle)
	mids := make([]*sketch.Point, len(left))
	for i := range left {
		rib := s.CreateLine(left[i], right[i])
		rib.SetName(fmt.Sprintf("rib %d", i))
		rib.SetConstruction(true)

		// Signed, not aligned: an aligned dimension gives only the length,
		// which the left and right flanks satisfy equally well swapped over.
		s.AddConstraint(alongSpine.across(left[i], right[i], leftPts[i], rightPts[i]))

		// Seeded at the foot of the left fit point on the spine, which is
		// where the constraints put it — not at the rib's own 2-D midpoint.
		foot := spineFoot(leftPts[i], angle)
		mids[i] = named(s, foot, "rib %d midpoint", i)
		s.AddConstraint(sketch.NewPointOnLine(mids[i], spine))
		s.AddConstraint(sketch.NewMidpoint(mids[i], rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
	}

	// The chain of midpoint-to-midpoint dimensions runs outward along the
	// spine, and starts at the local origin. Without that first link the whole
	// chain slides along the spine as a unit and never fully constrains.
	proofkit.Step(t, "midpoint chain, starting from the local origin")
	prev, prevAt := origin, involute.Pt{}
	for i := range mids {
		at := spineFoot(leftPts[i], angle)
		s.AddConstraint(alongSpine.along(prev, mids[i], prevAt, at))
		prev, prevAt = mids[i], at
	}

	// Close the tooth at the root. When the flank starts outside the root
	// circle a short radial stub reaches down to it, placed by two signed
	// offsets from the local origin. Placing it instead with "on the root
	// circle" plus "origin on the line" admits the far intersection too, and
	// the stub becomes a line straight across the gear ([SPUR-F-FLANK-ROOT]).
	if d.Embedded() {
		proofkit.Step(t, "flank starts inside the root circle: embedded profile, no stubs")
	} else {
		proofkit.Step(t, "flank starts outside the root circle: two radial flank-to-root stubs")
		stub(s, origin, left[0], leftPts[0], d.Root, "left")
		stub(s, origin, right[0], rightPts[0], d.Root, "right")
	}

	// Step 5 of the spec, and the last thing draw() does apart from confirming
	// the rotation: project the Tools-sketch anchor in and make the local
	// origin coincident with it. Every point above is placed relative to the
	// local origin, so this one constraint grounds the whole sketch.
	proofkit.Step(t, "anchor: project ctx.anchorPoint in and make the local origin coincident with it")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor")
	anchor.SetName("projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	requireToothProfile(t, s, d.Embedded())
}

// stepBoreProfileSketch realises step S14.
//
// The sketch holds two points at the origin and one circle. One point is the
// projected anchor the circle is centred on; the other is the tooth
// generator's local origin, which its constructor always adds and drawBore
// never uses. That stray point is the spec's one exception to the
// full-constraint rule, and it is reference geometry here for the reason the
// exception is granted: it is placed by the constructor and never moved.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	bore := p[pBoreDia]
	if bore <= 0 {
		proofkit.Unmodelled(t, "bore diameter is %.3g mm, so buildBore returns before any sketch exists", bore)
	}

	proofkit.Step(t, "constructor: the unused local origin the tooth generator always adds")
	stray := s.CreateReferencePoint(0, 0, "tooth generator local origin, never moved")
	stray.SetName("unused local origin")

	proofkit.Step(t, "drawBore: project the anchor in, draw the bore circle on it, dimension the diameter")
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor")
	anchor.SetName("projected anchor")
	c := s.CreateCircle(anchor, bore/2)
	c.SetName("bore circle")
	s.AddConstraint(sketch.NewDiameter(c, bore))
}

// requireToothProfile checks that region detection recovers the loops the
// extrude steps look for: the tooth, whose curve count step S7 matches on, and
// the root disc, which step S9 matches on.
//
// The counts are the point of drawing no root arc by hand. Had the proof drawn
// one, the loop would close whether or not the splitting Fusion actually does
// would have produced it.
//
// Two representation differences from Fusion are absorbed here, and neither
// changes the boundary. The engine names the whole root circle as the tooth
// region's root-side entity, where Fusion names the arc it split out of it, so
// the tooth's entity count matches step S7's curve count exactly. The engine
// then reports the disc's boundary as that one circle, where Fusion reports the
// two arcs it split the circle into, which is the count step S9 matches on.
func requireToothProfile(t testing.TB, s *sketch.Sketch, embedded bool) {
	t.Helper()
	wantTooth := 6 // 2 flanks + 2 flank-to-root lines + tooth-top arc + root arc
	if embedded {
		wantTooth = 4 // 2 flanks + tooth-top arc + root arc
	}
	regions := s.Profiles()
	var tooth, disc int
	for _, r := range regions {
		switch len(r.Entities) {
		case wantTooth:
			tooth++
		case 1:
			disc++
		}
	}
	if len(regions) != 2 || tooth != 1 || disc != 1 {
		t.Errorf("region detection found %d region(s): %d bounded by the tooth's %d entities, %d by the root circle alone; want 2 regions, 1 and 1",
			len(regions), tooth, wantTooth, disc)
		for i, r := range regions {
			t.Logf("  region %d: %d entities, %d boundary edge(s), %d hole(s), area %.4g",
				i, len(r.Entities), len(r.Outer), len(r.Holes), r.Area)
		}
	}
}

func circle(s *sketch.Sketch, center *sketch.Point, name string, radius float64, construction bool) *sketch.Circle {
	c := s.CreateCircle(center, radius)
	c.SetName(name + " circle")
	c.SetConstruction(construction)
	s.AddConstraint(sketch.NewDiameter(c, 2*radius))
	return c
}

func named(s *sketch.Sketch, at involute.Pt, format string, args ...any) *sketch.Point {
	p := s.CreatePoint(at.X, at.Y)
	p.SetName(fmt.Sprintf(format, args...))
	return p
}

// stub draws one flank-to-root line and pins its root end with exactly two
// signed offsets from the local origin — no others.
func stub(s *sketch.Sketch, origin, flankStart *sketch.Point, at involute.Pt, rootR float64, side string) {
	dir := math.Atan2(at.Y, at.X)
	foot := involute.Pt{X: rootR * math.Cos(dir), Y: rootR * math.Sin(dir)}
	end := named(s, foot, "%s flank-to-root end", side)
	line := s.CreateLine(end, flankStart)
	line.SetName(side + " flank-to-root")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, end, foot.X),
		sketch.NewVerticalDistance(origin, end, foot.Y),
	)
}

// orientation picks between the horizontal and vertical signed dimension for
// each of the two jobs the rib recipe needs one for. Ribs run perpendicular to
// the spine and the chain runs along it, so one predicate settles both: when
// the spine is nearer horizontal the chain is dimensioned horizontally and the
// ribs vertically, and when it is nearer vertical, the other way round.
type orientation struct{ spineNearHorizontal bool }

func signedAlong(angle float64) orientation {
	return orientation{spineNearHorizontal: math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))}
}

// across dimensions a rib, from its left end to its right end.
func (o orientation) across(a, b *sketch.Point, atA, atB involute.Pt) sketch.Constraint {
	if o.spineNearHorizontal {
		return sketch.NewVerticalDistance(a, b, atB.Y-atA.Y)
	}
	return sketch.NewHorizontalDistance(a, b, atB.X-atA.X)
}

// along dimensions one link of the midpoint chain.
func (o orientation) along(a, b *sketch.Point, atA, atB involute.Pt) sketch.Constraint {
	if o.spineNearHorizontal {
		return sketch.NewHorizontalDistance(a, b, atB.X-atA.X)
	}
	return sketch.NewVerticalDistance(a, b, atB.Y-atA.Y)
}

// spineFoot is the foot of p on the spine — the line at angle through the
// local origin — which is where a rib midpoint ends up.
func spineFoot(p involute.Pt, angle float64) involute.Pt {
	sa, ca := math.Sin(angle), math.Cos(angle)
	t := p.X*ca + p.Y*sa
	return involute.Pt{X: t * ca, Y: t * sa}
}

func radians(deg float64) float64 { return deg * math.Pi / 180 }
