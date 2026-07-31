// Package spurgear_test proves the spur gear's sketches.
//
// The build is compiled from spec/spurgear/instructions.md, its Fusion sidecar
// spec/spurgear/fusion.md, and the shared PLAYBOOK.md, into the step list at
// .tmp/spurgear.steps.md. Every step… function here realises the step of the
// same name there, and every [GO] step there names a function here.
//
// The spur gear puts three sketches in the timeline — Tools, Gear Profile and
// Bore Profile — and all three are proved. Every other step is solid modelling
// (extrude, chamfer, pattern, combine, fillet, cut) or construction geometry,
// which this engine does not model, so those steps are prose and have no
// function here.
package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Case parameter keys. Lengths are millimetres and angles radians; the Fusion
// generator works in centimetres, which scales the sketch and changes none of
// its constraints.
const (
	pModule        = "module"
	pToothNumber   = "toothNumber"
	pPressureAngle = "pressureAngle"
	pSteps         = "involuteSteps"
	pAngle         = "angle"
	pAnchorX       = "anchorX"
	pAnchorY       = "anchorY"
	pBoreDiameter  = "boreDiameter"
)

func deg(d float64) float64 { return d * math.Pi / 180 }

// gearCases sweeps the Gear Profile constraint scheme.
//
// The first four are the required sizes. The rest reach branches those do not:
// the embedded profile, which drops the flank-to-root stubs; the near-degenerate
// size just outside it, where the stubs are 14 micrometres long; a draw angle in
// both signs, since the tooth generator takes one; an angle that puts the spine
// on the vertical, where the signed rib and chain dimensions have to swap axis;
// an anchor away from the plane origin; and a coarser involute sampling.
//
// Embedded needs a HIGH tooth count, not a low one. The base circle falls below
// the root circle when toothNumber > 2.5/(1-cos(pressureAngle)) — 41.5 teeth at
// 20 degrees, 26.7 at 25 — so 45 teeth is embedded and 41 is the near-miss.
var gearCases = []proofkit.Case{
	{Name: "m1_t12", Params: gear(1, 12, deg(20), 15, 0, 0, 0)},
	{Name: "m1_t17", Params: gear(1, 17, deg(20), 15, 0, 0, 0)},
	{Name: "m2_t20", Params: gear(2, 20, deg(20), 15, 0, 0, 0)},
	{Name: "m3_t15", Params: gear(3, 15, deg(20), 15, 0, 0, 0)},

	{Name: "m1_t45_embedded", Params: gear(1, 45, deg(20), 15, 0, 0, 0)},
	{Name: "m2_t30_pa25_embedded", Params: gear(2, 30, deg(25), 15, 0, 0, 0)},
	{Name: "m1_t41_near_zero_stub", Params: gear(1, 41, deg(20), 15, 0, 0, 0)},

	{Name: "m1_t17_rot30", Params: gear(1, 17, deg(20), 15, deg(30), 0, 0)},
	{Name: "m2_t20_rot_minus25", Params: gear(2, 20, deg(20), 15, deg(-25), 0, 0)},
	{Name: "m5_t12_rot90", Params: gear(5, 12, deg(20), 15, deg(90), 0, 0)},
	{Name: "m1_t45_rot120_embedded", Params: gear(1, 45, deg(20), 15, deg(120), 0, 0)},

	{Name: "m1_t17_offset_anchor", Params: gear(1, 17, deg(20), 15, 0, 37, -19)},
	{Name: "m1p5_t24_pa14p5_steps8", Params: gear(1.5, 24, deg(14.5), 8, 0, 0, 0)},
}

func gear(module, toothNumber, pressureAngle float64, steps int, angle, ax, ay float64) map[string]float64 {
	return map[string]float64{
		pModule:        module,
		pToothNumber:   toothNumber,
		pPressureAngle: pressureAngle,
		pSteps:         float64(steps),
		pAngle:         angle,
		pAnchorX:       ax,
		pAnchorY:       ay,
	}
}

func TestToolsSketch(t *testing.T) {
	proofkit.Run(t, []proofkit.Case{
		{Name: "anchor_at_plane_origin", Params: map[string]float64{pAnchorX: 0, pAnchorY: 0}},
		{Name: "anchor_off_plane_origin", Params: map[string]float64{pAnchorX: 37, pAnchorY: -19}},
	}, stepToolsSketch)
}

func TestGearProfileSketch(t *testing.T) {
	proofkit.Run(t, gearCases, stepGearProfileSketch)
}

func TestBoreProfileSketch(t *testing.T) {
	proofkit.Run(t, []proofkit.Case{
		{Name: "bore6_anchor_at_plane_origin", Params: map[string]float64{pBoreDiameter: 6, pAnchorX: 0, pAnchorY: 0}},
		{Name: "bore6_anchor_off_plane_origin", Params: map[string]float64{pBoreDiameter: 6, pAnchorX: 37, pAnchorY: -19}},
		{Name: "bore20", Params: map[string]float64{pBoreDiameter: 20, pAnchorX: 0, pAnchorY: 0}},
		{Name: "bore0_no_sketch", Params: map[string]float64{pBoreDiameter: 0, pAnchorX: 0, pAnchorY: 0}},
	}, stepBoreProfileSketch)
}

// stepToolsSketch proves the Tools sketch (step S3).
//
// The sketch draws nothing of its own. It exists to own one projection of the
// user's Anchor Point, the canonical handle every later sketch re-projects from,
// so what there is to prove is that a sketch holding only that projection is
// sound and has nothing loose in it.
//
// The projection is reference geometry, this engine's externally-locked import —
// the one thing here that genuinely comes from outside the sketch. Fusion's
// projection is weaker: [PB-PROJECT-NOT-FIXED] records that a projected point
// still carries free degrees of freedom until something constrains it. So this
// models the projection as the spec treats it, as an anchor, and the gap is
// covered where it matters: in the Gear Profile sketch the projection is what
// the local origin is constrained TO, which is the very fix that anchor names.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Anchor Point in as the canonical handle")
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "user Anchor Point")
	anchor.SetName("ctx.anchorPoint")
	// The Extrusion End Plane created alongside this sketch is 3D construction
	// geometry and carries no sketch constraint, so it is not modelled here.
}

// stepGearProfileSketch proves the Gear Profile sketch (step S5).
//
// This is the whole tooth construction: four gear circles on one shared centre,
// one involute tooth drawn at its final angular position, the rib chain tying
// both flanks to the spine, the spine's absolute angular pin, the flank-to-root
// stubs, and the anchoring that slides all of it onto the user's anchor.
//
// The tooth's root boundary is never drawn. The stubs end on the solid root
// circle — or, in the embedded case, the flanks cross it — and profile detection
// splits the circle there, which is how Fusion derives that arc too.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	var (
		module   = p[pModule]
		teeth    = p[pToothNumber]
		steps    = int(p[pSteps])
		angle    = p[pAngle]
		anchorX  = p[pAnchorX]
		anchorY  = p[pAnchorY]
		dims     = involute.Derive(module, teeth, p[pPressureAngle])
		embedded = dims.Embedded()
	)

	// Every seed is written relative to the local origin and then offset by the
	// anchor, because every dimension below is relative to the local origin too.
	// The anchor coincidence at the end is the only thing that says where the
	// gear sits, which is what lets the anchor move it afterwards.
	at := func(x, y float64) (float64, float64) { return anchorX + x, anchorY + y }

	proofkit.Step(t, "project the Tools anchor in and add the movable local origin")
	// [SPUR-F-ANCHOR-CHAIN]: the Tools projection, re-projected into this sketch.
	anchor := s.CreateReferencePoint(anchorX, anchorY, "Tools sketch anchor projection")
	anchor.SetName("projected anchor")
	// [SPUR-F-LOCAL-ORIGIN]: a fresh point rather than the sketch origin, so it
	// can be constrained to something brought in from elsewhere.
	origin := s.CreatePoint(at(0, 0))
	origin.SetName("local origin")

	proofkit.Step(t, "draw the four gear circles on the shared local origin")
	// All four take the local origin directly as their centre and share it;
	// re-coincidenting a fresh centre onto it is what breaks the solver
	// ([SPUR-F-SHARED-ADJACENCY], [PB-SHARE-XOR-COINCIDENT]). Only the root
	// circle is solid, so only it bounds a region.
	root := s.CreateCircle(origin, dims.Root)
	root.SetName("root circle")
	tip := s.CreateCircle(origin, dims.Tip)
	tip.SetName("tip circle")
	tip.SetConstruction(true)
	base := s.CreateCircle(origin, dims.Base)
	base.SetName("base circle")
	base.SetConstruction(true)
	pitch := s.CreateCircle(origin, dims.Pitch)
	pitch.SetName("pitch circle")
	pitch.SetConstruction(true)
	// One driving diameter dimension each ([PB-DRIVING-DIM]).
	s.AddConstraint(
		sketch.NewDiameter(root, 2*dims.Root),
		sketch.NewDiameter(tip, 2*dims.Tip),
		sketch.NewDiameter(base, 2*dims.Base),
		sketch.NewDiameter(pitch, 2*dims.Pitch),
	)
	// The along-path text label on each circle is not modelled. Sketch text
	// carries no constraint and bounds no region, so it cannot move this verdict
	// either way.

	proofkit.Step(t, "sample both involute flanks at draw angle %.1f deg", angle*180/math.Pi)
	// involute.Flanks owns the mirror, the pitch-crossing rotation that centres
	// the tooth on +X, and the requested draw angle, in that order.
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, teeth, steps, angle)
	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(at(left[i].X, left[i].Y))
		leftPts[i].SetName(nameIdx("left flank fit", i))
		rightPts[i] = s.CreatePoint(at(right[i].X, right[i].Y))
		rightPts[i].SetName(nameIdx("right flank fit", i))
	}
	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("left flank")
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("right flank")

	proofkit.Step(t, "cap the tooth with an arc centred on the local origin")
	// [SPUR-F-TOOTHTOP-ARC]: the tooth-top point sits on the tip circle, and the
	// arc shares the local origin as centre and the two flank ends as its ends.
	// No diameter dimension: the shared centre is what says the arc bulges
	// outward, where a free centre plus a diameter would reach the same DOF with
	// the inward-bulging answer still open.
	last := len(left) - 1
	toothTop := s.CreatePoint(at(dims.Tip*math.Cos(angle), dims.Tip*math.Sin(angle)))
	toothTop.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	topArc := s.CreateArc(origin, rightPts[last], leftPts[last])
	topArc.SetName("tooth top arc")

	proofkit.Step(t, "draw the spine and pin its absolute direction")
	// [SPUR-F-SPINE]: the spine and the +X reference share the local origin. The
	// reference's far end is placed by two signed dimensions rather than by
	// landing it on the tip circle, where a point has two answers and the numbers
	// go unstable at the extreme.
	spine := s.CreateLine(origin, toothTop)
	spine.SetName("spine")
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(at(dims.Tip, 0))
	refEnd.SetName("+X reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, dims.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetName("+X reference")
	reference.SetConstruction(true)
	// Signed, counter-clockwise from the reference to the spine, in that order.
	// Built for every angle including zero: a plain horizontal on the spine would
	// fix its slope and say nothing about which end the tooth top is on, which is
	// how a tooth comes out 180 degrees around.
	spineAngle := sketch.NewAngle(reference, spine, angle*180/math.Pi)
	s.AddConstraint(spineAngle)

	proofkit.Step(t, "build %d ribs and the midpoint chain along the spine", len(left))
	// [SPUR-F-RIBS], in the order the anchor fixes: rib, signed rib dimension,
	// midpoint seeded on the spine, point-on-line, midpoint-of, perpendicular.
	//
	// Both dimensions are signed. An aligned one gives only a length, which the
	// mirrored tooth satisfies just as well: swapping it in makes this sketch
	// reach DOF 0 with nine discrete answers instead of one.
	//
	// The spec pins the axis only for angle 0 — rib vertical, chain horizontal —
	// and otherwise says "whichever is better conditioned". The rib runs across
	// the spine and the chain along it, so they take opposite axes, chosen by
	// which component of the spine direction is larger.
	alongIsX := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previous := origin
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetName(nameIdx("rib", i))
		rib.SetConstruction(true)

		if alongIsX {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}

		// Seeded at the foot of the left fit point on the spine, so it starts on
		// the line it is about to be pinned to ([PB-SEED-NEAR]).
		foot := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		mid := s.CreatePoint(at(foot*math.Cos(angle), foot*math.Sin(angle)))
		mid.SetName(nameIdx("rib midpoint", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			// The last rib's perpendicular is what the tooth-top arc already
			// says. Adding it is one equation too many, and the engine reports it
			// as a redundant constraint exactly as Fusion rejects it.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}

		// The chain starts at the local origin. Without that first link the whole
		// chain slides along the spine as a unit and the sketch never closes.
		px, py := previous.X(), previous.Y()
		mx, my := mid.X(), mid.Y()
		if alongIsX {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, mx-px))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, my-py))
		}
		previous = mid
	}

	if embedded {
		proofkit.Step(t, "embedded profile: the base circle is inside the root circle, so no stubs are drawn")
	} else {
		proofkit.Step(t, "close the tooth at the root with two radial stubs")
		// [SPUR-F-FLANK-ROOT]: each stub shares the flank's first fit point and
		// has its root end placed by exactly two signed dimensions from the local
		// origin. "Root end on the root circle" plus "origin on the line" would be
		// satisfied on the far side of the gear as well, turning the stub into a
		// line straight across the centre.
		for _, side := range []struct {
			name string
			p    involute.Pt
			fit  *sketch.Point
		}{
			{"left", left[0], leftPts[0]},
			{"right", right[0], rightPts[0]},
		} {
			scale := dims.Root / math.Hypot(side.p.X, side.p.Y)
			rx, ry := side.p.X*scale, side.p.Y*scale
			rootEnd := s.CreatePoint(at(rx, ry))
			rootEnd.SetName(side.name + " flank root end")
			stub := s.CreateLine(rootEnd, side.fit)
			stub.SetName(side.name + " flank-to-root")
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, rx),
				sketch.NewVerticalDistance(origin, rootEnd, ry),
			)
		}
	}

	proofkit.Step(t, "anchor the sketch, then confirm the rotation")
	// Step 5: one coincidence drags the whole drawing onto the anchor, because
	// everything above is dimensioned relative to the local origin.
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	// [SPUR-F-ROTATE-CONFIRM]: the rotation is drawn AND confirmed. The geometry
	// is already at its rotated position; re-setting the angular dimension once
	// the whole network exists is the confirming half.
	if angle != 0 {
		spineAngle.Set(angle * 180 / math.Pi)
	}

	proofkit.Step(t, "check the regions the extrude steps will look for")
	requireExtrudableRegions(t, s, topArc, embedded)
}

// requireExtrudableRegions checks that this sketch offers the two regions the
// extrude steps search for, and that the tooth region carries the curve mix step
// S7 asks find_profile_by_curve_counts for.
//
// This is the seam between the sketch and the solid modelling that follows it.
// The tooth boundary is counted in DISTINCT entities, which is what Fusion
// counts: 2 splines, 2 arcs (the tooth top, plus the root circle standing in for
// the arc Fusion splits out of it), and 2 stub lines unless the profile is
// embedded. The engine's own edge count runs one higher whenever the tooth sits
// at angle 0, because it emits a closed curve's fragment as two edges when the
// fragment straddles that curve's seam at +X. That is how the arrangement
// represents the boundary, not a difference in the region.
func requireExtrudableRegions(t testing.TB, s *sketch.Sketch, topArc *sketch.Arc, embedded bool) {
	t.Helper()

	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("expected 2 regions (the tooth and the root disk), got %d", len(profiles))
	}

	var tooth, disk *sketch.Profile
	for _, p := range profiles {
		onTooth := false
		for _, e := range p.Entities {
			if e == sketch.Entity(topArc) {
				onTooth = true
			}
		}
		if onTooth {
			tooth = p
		} else {
			disk = p
		}
	}
	if tooth == nil || disk == nil {
		t.Fatalf("could not tell the tooth region from the root disk")
	}

	var splines, arcs, circles, lines int
	for _, e := range tooth.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Circle:
			circles++
		case *sketch.Line:
			lines++
		default:
			t.Errorf("unexpected %T on the tooth boundary", e)
		}
	}
	wantLines := 2
	if embedded {
		wantLines = 0
	}
	if splines != 2 || arcs+circles != 2 || lines != wantLines {
		t.Errorf("tooth region has %d spline(s), %d arc(s) + %d circle(s) and %d line(s); step S7 looks for nurbs=2, arcs=2, lines=%d",
			splines, arcs, circles, lines, wantLines)
	}

	// Step S9's body profile. The spec calls it "the annular loop bounded by
	// exactly 2 arcs (the root circle and the tip circle)", which does not
	// describe this sketch: the tip circle is construction and bounds nothing, so
	// what is really there is the full root DISK, whose boundary is the root
	// circle split in two by the tooth. Fusion counts those two fragments as two
	// arcs, which is why find_profile_by_curve_counts(sketch, arcs=2) finds it
	// anyway. This engine coalesces the same boundary back into one whole circle,
	// so the count cannot be checked here — the region's identity can.
	if len(disk.Entities) != 1 {
		t.Errorf("root disk boundary has %d entities, expected the root circle alone", len(disk.Entities))
	} else if _, ok := disk.Entities[0].(*sketch.Circle); !ok {
		t.Errorf("root disk is bounded by %T, expected the root circle", disk.Entities[0])
	}
}

// stepBoreProfileSketch proves the Bore Profile sketch (step S14).
//
// The bore circle is drawn by instantiating the tooth generator on a fresh
// sketch, so the sketch inherits that constructor's local-origin point with
// nothing using it. The spec keeps the stray point rather than suppressing it,
// and grounds it on the sketch's own origin; that one coincidence is the whole
// reason this sketch reaches zero degrees of freedom.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	if p[pBoreDiameter] <= 0 {
		proofkit.Unmodelled(t, "there is no Bore Profile sketch at Bore Diameter %g — buildBore returns before creating it", p[pBoreDiameter])
	}

	proofkit.Step(t, "project the Tools anchor in and draw the bore circle on it")
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "Tools sketch anchor projection")
	anchor.SetName("projected anchor")
	bore := s.CreateCircle(anchor, p[pBoreDiameter]/2)
	bore.SetName("bore circle")
	s.AddConstraint(sketch.NewDiameter(bore, p[pBoreDiameter]))

	proofkit.Step(t, "ground the tooth generator's unused local origin on the sketch origin")
	// This is the one place the spec grounds a point on the sketch's own origin
	// rather than on the projected anchor, so it is the one place the proof does.
	stray := s.CreatePoint(0, 0)
	stray.SetName("tooth generator local origin")
	s.AddConstraint(sketch.NewCoincident(stray, s.Origin()))

	if n := len(s.Profiles()); n != 1 {
		t.Errorf("expected the bore disk as the only region, got %d", n)
	}
}

func nameIdx(prefix string, i int) string {
	return prefix + " " + itoa(i)
}

func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	var b []byte
	for i > 0 {
		b = append([]byte{byte('0' + i%10)}, b...)
		i /= 10
	}
	return string(b)
}
