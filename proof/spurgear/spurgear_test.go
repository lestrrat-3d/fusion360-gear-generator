// Package spurgear_test proves the three sketches of the spur gear build.
//
// The step list this file answers to is .tmp/spurgear.steps.md. Every [GO] step
// there names one function here, and every function here is named by a step:
// stepToolsSketch (S4), stepGearProfileSketch (S6), stepBoreProfileSketch (S14).
// The remaining steps are 3D — extrudes, chamfer, pattern, combine, fillet, the
// bore cut and the cleanup — and the proof engine models 2D sketches only, so
// they are [PROSE] and have no function here.
//
// What is proved is the constraint scheme, not Fusion's entity list. Every case
// must reach the engine's full verdict with nothing waived: DOF 0, no redundant
// or conflicting constraints, valid profiles, a well-conditioned system, and a
// single discrete configuration. That last one is why the scheme leans on signed
// dimensions throughout — a mirrored or 180-degree-rotated tooth also reaches
// DOF 0, and only a constraint that carries a direction rules it out.
package spurgear_test

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys. These are the dialog inputs and the one derived count the
// sketches actually consume; lengths are in millimetres and angles in radians,
// converted to the engine's degree-valued angle dimensions at the call site.
const (
	pModule      = "module"
	pToothNumber = "toothNumber"
	pPressure    = "pressureAngle"
	pSteps       = "involuteSteps"
	pAngle       = "angle"
	pBore        = "boreDiameter"
	pAnchorX     = "anchorX"
	pAnchorY     = "anchorY"
)

func deg(d float64) float64 { return d * math.Pi / 180 }

// gear builds one case's parameter set. The four required sweep sizes and every
// size added below go through here, so a case differs from its neighbours only
// in what it overrides.
func gear(name string, module, teeth float64, opts ...func(map[string]float64)) proofkit.Case {
	p := map[string]float64{
		pModule:      module,
		pToothNumber: teeth,
		pPressure:    deg(20),
		pSteps:       15,
		pAngle:       0,
		pBore:        3,
		pAnchorX:     0,
		pAnchorY:     0,
	}
	for _, o := range opts {
		o(p)
	}
	return proofkit.Case{Name: name, Params: p}
}

func pressure(d float64) func(map[string]float64) {
	return func(p map[string]float64) { p[pPressure] = deg(d) }
}
func steps(n float64) func(map[string]float64) {
	return func(p map[string]float64) { p[pSteps] = n }
}
func rotated(d float64) func(map[string]float64) {
	return func(p map[string]float64) { p[pAngle] = deg(d) }
}
func bore(d float64) func(map[string]float64) {
	return func(p map[string]float64) { p[pBore] = d }
}
func anchorAt(x, y float64) func(map[string]float64) {
	return func(p map[string]float64) { p[pAnchorX], p[pAnchorY] = x, y }
}

// cases sweeps the sizes the scheme has to hold across.
//
// The first four are the required baseline. The rest reach branches the
// baseline does not: the embedded profile (the base circle inside the root
// circle, which drops the flank-to-root stubs and changes the tooth loop from
// six curves to four), the two pressure angles that move the embedding
// threshold, the rotation argument the tooth generator takes for helical,
// herringbone and the bevel virtual tooth, the sample counts at either end of
// the involute-steps range, and an anchor away from the plane origin, which is
// the only thing the projection chain exists for.
var cases = []proofkit.Case{
	// The required sweep: 20 degrees pressure angle, 15 involute steps.
	gear("m1_z12", 1, 12),
	gear("m1_z17", 1, 17),
	gear("m2_z20", 2, 20),
	gear("m3_z15", 3, 15),

	// Embedded profiles. At 20 degrees the base circle drops below the root
	// circle above 41.5 teeth; a larger pressure angle brings it on sooner
	// (26.7 teeth at 25 degrees), a smaller one holds it off (78.5 at 14.5).
	gear("m1_z50_embedded", 1, 50),
	gear("m2_z60_embedded", 2, 60),
	gear("m1_z30_pa25_embedded", 1, 30, pressure(25)),
	gear("m1_z60_pa14.5_notEmbedded", 1, 60, pressure(14.5)),

	// Just short of the embedding threshold, where the flank-to-root stub is
	// still drawn but is only a few hundredths of a millimetre long. This is
	// the ill-conditioned region the spec flags; it is here to say how close
	// the scheme survives.
	gear("m1_z35_shortStub", 1, 35),

	// Rotation. The generator draws the tooth already turned by the angle and
	// then confirms it with the spine dimension, and the rib and chain
	// dimensions swap axes once the spine passes 45 degrees.
	gear("m1_z17_rot15", 1, 17, rotated(15)),
	gear("m1_z17_rot45", 1, 17, rotated(45)),
	gear("m2_z20_rot90", 2, 20, rotated(90)),
	gear("m1_z17_rot135", 1, 17, rotated(135)),
	gear("m1_z17_rot180", 1, 17, rotated(180)),
	gear("m1_z17_rotMinus30", 1, 17, rotated(-30)),
	gear("m1_z50_rot30_embedded", 1, 50, rotated(30)),

	// Sample counts either side of the derived 15.
	gear("m1_z17_steps4", 1, 17, steps(4)),
	gear("m1_z17_steps2", 1, 17, steps(2)),
	gear("m1_z17_steps25", 1, 17, steps(25)),

	// The anchor away from the plane origin, with and without rotation.
	gear("m1_z17_offsetAnchor", 1, 17, anchorAt(37.5, -21.25)),
	gear("m2_z20_offsetAnchor_rot60", 2, 20, rotated(60), anchorAt(-14, 9)),

	// Bore variants. A zero bore is the branch where buildBore returns before
	// creating the sketch at all.
	gear("m3_z15_bigBore", 3, 15, bore(20)),
	gear("m1_z17_noBore", 1, 17, bore(0)),
}

func TestToolsSketch(t *testing.T)       { proofkit.Run(t, cases, stepToolsSketch) }
func TestGearProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepGearProfileSketch) }
func TestBoreProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepBoreProfileSketch) }

// stepToolsSketch proves S4, the Tools sketch.
//
// The sketch draws no geometry of its own. It exists to own one thing: the
// projection of the user's anchor point, which is the canonical handle every
// later sketch re-projects from. A projection is externally located, so it is
// reference geometry here, and the sketch is determinate with nothing else in
// it.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Anchor Point in and keep it as ctx.anchorPoint")
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "user/anchorPoint")
	anchor.SetName("ctx.anchorPoint")
}

// stepGearProfileSketch proves S6, the Gear Profile sketch.
//
// This is the whole tooth: the four gear circles, the two involute flanks, the
// tooth-top arc, the spine and its +X reference, one rib per flank fit-point,
// the flank-to-root stubs when the profile is not embedded, and the anchoring
// that slides the drawing onto the user's anchor. It is one Fusion timeline
// entry, so it is one step and one function.
//
// The order below is the order the tooth generator runs in: the constructor's
// local origin, then drawCircles, then drawTooth, then the step-5 anchoring,
// then the confirming angular dimension as the very last action.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	var (
		module = p[pModule]
		teeth  = p[pToothNumber]
		pa     = p[pPressure]
		n      = int(p[pSteps])
		angle  = p[pAngle]
		ax, ay = p[pAnchorX], p[pAnchorY]
	)
	d := involute.Derive(module, teeth, pa)

	// The tooth generator's constructor adds its movable local origin at
	// (0, 0). Everything below is drawn relative to it, and the anchoring at
	// the end drags the lot onto the user's anchor as a unit.
	// Seeds carry the anchor offset. Fusion draws at (0, 0) and lets the
	// step-5 coincidence slide the sketch onto the anchor; the engine's solver
	// will not travel that far in one go and reports no convergence, so the
	// proof seeds the drawing where it ends up. A seed is not a constraint —
	// the constraint set below is identical either way — but it is a real
	// difference from Fusion and is recorded here rather than hidden.
	pt := func(x, y float64) *sketch.Point { return s.CreatePoint(x+ax, y+ay) }

	proofkit.Step(t, "constructor: the movable local origin at (0, 0)")
	lo := pt(0, 0)
	lo.SetName("localOrigin")

	// --- drawCircles -------------------------------------------------------
	//
	// All four share the local origin as their centre — the point object
	// itself, not a coincident to a copy of it — and each carries a driving
	// diameter dimension. Only the root circle is solid; the other three are
	// construction, which is why the tip circle bounds no profile.
	proofkit.Step(t, "drawCircles: root solid, tip/base/pitch construction, all centred on the local origin")
	root := s.CreateCircle(lo, d.Root)
	root.SetName("rootCircle")
	tip := s.CreateCircle(lo, d.Tip)
	tip.SetName("tipCircle")
	tip.SetConstruction(true)
	base := s.CreateCircle(lo, d.Base)
	base.SetName("baseCircle")
	base.SetConstruction(true)
	pitch := s.CreateCircle(lo, d.Pitch)
	pitch.SetName("pitchCircle")
	pitch.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(root, 2*d.Root),
		sketch.NewDiameter(tip, 2*d.Tip),
		sketch.NewDiameter(base, 2*d.Base),
		sketch.NewDiameter(pitch, 2*d.Pitch),
	)

	// --- drawTooth: the flanks --------------------------------------------
	proofkit.Step(t, "drawTooth: %d involute samples per flank, drawn at angle %.1f degrees", n, angle*180/math.Pi)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, n, angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "%d involute sample(s) survive: a fitted spline needs at least two fit points", len(left))
	}
	last := len(left) - 1

	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = pt(left[i].X, left[i].Y)
		rightPts[i] = pt(right[i].X, right[i].Y)
	}
	leftFlank, err := s.CreateFitSpline(leftPts...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	leftFlank.SetName("leftFlank")
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	rightFlank.SetName("rightFlank")

	// --- the tooth-top arc -------------------------------------------------
	//
	// The arc caps the tooth at the tip circle, so it is part of that circle
	// and must bulge outward. That is said by sharing the local origin as its
	// centre and the two flank ends as its ends — and by nothing else. A free
	// centre plus a diameter dimension reaches DOF 0 with the arc free to
	// curve back through the tooth.
	proofkit.Step(t, "tooth-top arc: centred on the local origin, sharing both flank ends, no diameter dimension")
	toothTop := pt(d.Tip*math.Cos(angle), d.Tip*math.Sin(angle))
	toothTop.SetName("toothTopPoint")
	toothTop.SetConstruction(true)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))

	topArc := s.CreateArc(lo, rightPts[last], leftPts[last])
	topArc.SetName("toothTopArc")

	// --- the spine and its +X reference ------------------------------------
	//
	// The spine is the tooth's axis of symmetry. What pins it is not a
	// horizontal constraint — that fixes the direction but not which way it
	// points, and the tooth comes out 180 degrees around. It is a signed
	// angular dimension against a reference line that is itself pinned to +X
	// by two signed offsets from the local origin.
	proofkit.Step(t, "spine + X reference + signed angular pin")
	spine := s.CreateLine(lo, toothTop)
	spine.SetName("spine")
	spine.SetConstruction(true)

	refEnd := pt(d.Tip, 0)
	refEnd.SetName("xReferenceEnd")
	refEnd.SetConstruction(true)
	s.AddConstraint(
		sketch.NewHorizontalDistance(lo, refEnd, d.Tip),
		sketch.NewVerticalDistance(lo, refEnd, 0),
	)
	xRef := s.CreateLine(lo, refEnd)
	xRef.SetName("xReference")
	xRef.SetConstruction(true)

	spineAngle := sketch.NewAngle(xRef, spine, angle*180/math.Pi)
	s.AddConstraint(spineAngle)
	s.SetConstraintName(spineAngle, "spineAngle")

	// --- the ribs ----------------------------------------------------------
	//
	// One rib per fit-point index, endpoints included: the fit-points carry no
	// other constraint, so a missing endpoint rib leaves that point free. The
	// last rib carries no perpendicular — the tooth-top arc, sharing its centre
	// with the local origin, already holds the two tips at equal radius either
	// side of the spine, so the perpendicular would be redundant.
	//
	// The rib takes the axis across the spine and the midpoint chain the one
	// along it. Both are signed: an aligned dimension gives only a length,
	// which the left and right flanks satisfy equally well swapped over.
	ca, sa := math.Cos(angle), math.Sin(angle)
	acrossIsVertical := math.Abs(ca) >= math.Abs(sa)
	proofkit.Step(t, "ribs: %d of them, rib axis %s, chain axis %s",
		len(left), axisName(acrossIsVertical), axisName(!acrossIsVertical))

	mids := make([]*sketch.Point, len(left))
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)

		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].X-left[i].X))
		}

		// The midpoint is seeded on the spine at the foot of the left
		// fit-point, not at the rib's true two-dimensional midpoint.
		foot := left[i].X*ca + left[i].Y*sa
		mid := pt(foot*ca, foot*sa)
		mid.SetConstruction(true)
		mids[i] = mid

		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
	}

	// The midpoint chain runs outward from the local origin. Without the
	// origin-to-first dimension the whole chain slides along the spine as a
	// unit and the sketch never fully constrains.
	proofkit.Step(t, "midpoint chain, starting from the local origin")
	prev := lo
	for i := range mids {
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mids[i], mids[i].X()-prev.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mids[i], mids[i].Y()-prev.Y()))
		}
		prev = mids[i]
	}

	// --- closing the tooth at the root -------------------------------------
	embedded := d.Embedded()
	if embedded {
		proofkit.Step(t, "embedded profile: base radius %.4f is inside root radius %.4f, no flank-to-root stubs", d.Base, d.Root)
	} else {
		proofkit.Step(t, "flank-to-root stubs: base radius %.4f is outside root radius %.4f, stub length %.4f", d.Base, d.Root, d.Base-d.Root)
		for _, side := range []struct {
			name  string
			start *sketch.Point
			at    involute.Pt
		}{
			{"leftFlankToRoot", leftPts[0], left[0]},
			{"rightFlankToRoot", rightPts[0], right[0]},
		} {
			// The stub runs radially from the root circle out to the flank's
			// first fit point, which it shares. Its root end is placed by two
			// signed offsets from the local origin and nothing else: "on the
			// root circle" plus "the origin on the line" is satisfied by the
			// far intersection too, and the stub becomes a line straight
			// across the gear.
			th := math.Atan2(side.at.Y, side.at.X)
			rx, ry := d.Root*math.Cos(th), d.Root*math.Sin(th)
			rootEnd := pt(rx, ry)
			rootEnd.SetName(side.name + "/rootEnd")
			stub := s.CreateLine(rootEnd, side.start)
			stub.SetName(side.name)
			s.AddConstraint(
				sketch.NewHorizontalDistance(lo, rootEnd, rx),
				sketch.NewVerticalDistance(lo, rootEnd, ry),
			)
		}
	}

	// --- step 5: anchor the sketch -----------------------------------------
	//
	// Re-project the Tools-sketch anchor and tie the local origin to it. This
	// is the last thing draw() does apart from confirming the rotation, and it
	// is what chains this sketch to the user's original anchor entity.
	proofkit.Step(t, "anchoring: re-project the Tools anchor at (%.3f, %.3f) and tie the local origin to it", ax, ay)
	anchor := s.CreateReferencePoint(ax, ay, "Tools/anchorPoint")
	anchor.SetName("projectedAnchor")
	s.AddConstraint(sketch.NewCoincident(lo, anchor))

	// --- the confirming rotation, as the very last action -------------------
	//
	// The geometry above was already drawn rotated. Setting the dimension now
	// confirms and locks that rotation rather than swinging the tooth into
	// place from +X, which is what puts a helical loft on the wrong branch.
	if angle != 0 {
		proofkit.Step(t, "confirming the drawn rotation on the spine dimension")
		spineAngle.Set(angle * 180 / math.Pi)
	}

	requireToothLoop(t, s, embedded)
}

func axisName(vertical bool) string {
	if vertical {
		return "vertical"
	}
	return "horizontal"
}

// requireToothLoop checks that the sketch hands the extrude steps the two
// regions they look for, built from the curves they match on.
//
// This is the sketch's contract with S8 and S10. Neither boundary arc is drawn:
// the root circle is one solid circle, and the arrangement splits it where the
// tooth meets it, exactly as Fusion's profile detection does. So what is
// checked here is evidence that the split happened, not a restatement of what
// was drawn.
//
// One representational difference is worth naming. Fusion reports the body loop
// as two arcs — the two pieces the tooth cuts the root circle into — while the
// engine reports the region bounded by the whole circle entity, and reports the
// piece under the tooth as one or two fragments of that same entity depending
// on whether the tooth straddles the circle's parameter seam at +X. Counting
// distinct entities rather than boundary edges is stable under that, and it is
// the count Fusion's find_profile_by_curve_counts is really asking for: six
// curves for a stubbed tooth, four for an embedded one.
func requireToothLoop(t testing.TB, s *sketch.Sketch, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before profile check: %v", err)
	}

	wantTooth := []string{"leftFlank", "rightFlank", "toothTopArc", "rootCircle"}
	if !embedded {
		wantTooth = append(wantTooth, "leftFlankToRoot", "rightFlankToRoot")
	}
	wantBody := []string{"rootCircle"}

	profiles := s.Profiles()
	tooth := findRegion(profiles, wantTooth)
	body := findRegion(profiles, wantBody)
	if tooth == nil || body == nil {
		var got []string
		for _, pr := range profiles {
			got = append(got, fmt.Sprintf("{%s}", strings.Join(regionEntities(pr), " ")))
		}
		t.Fatalf("expected a tooth region on {%s} and a body region on {%s}; got %d region(s): %s",
			strings.Join(wantTooth, " "), strings.Join(wantBody, " "), len(profiles), strings.Join(got, " "))
	}
	if tooth.Area >= body.Area {
		t.Errorf("the tooth region (%.4f mm2) is not smaller than the body region (%.4f mm2), so they were matched the wrong way round",
			tooth.Area, body.Area)
	}
	proofkit.Step(t, "profiles: tooth loop on %d curves (%.4f mm2), body loop inside the root circle (%.4f mm2)",
		len(wantTooth), tooth.Area, body.Area)
}

func findRegion(profiles []*sketch.Profile, want []string) *sketch.Profile {
	sorted := append([]string(nil), want...)
	sort.Strings(sorted)
	for _, pr := range profiles {
		if slices.Equal(regionEntities(pr), sorted) {
			return pr
		}
	}
	return nil
}

// regionEntities names the distinct entities on a region's outer boundary, in
// sorted order. An unnamed entity would be a drawing mistake, so it is reported
// as such rather than skipped.
func regionEntities(pr *sketch.Profile) []string {
	names := make([]string, 0, len(pr.Entities))
	for _, e := range pr.Entities {
		n := e.Name()
		if n == "" {
			n = fmt.Sprintf("unnamed:%T", e)
		}
		names = append(names, n)
	}
	sort.Strings(names)
	return names
}

// stepBoreProfileSketch proves S14, the Bore Profile sketch.
//
// The bore circle is drawn by instantiating the tooth generator on a fresh
// sketch and calling only drawBore, so the sketch inherits the constructor's
// local origin at (0, 0) as a stray point nothing uses. That point is grounded
// on the sketch's own origin — one constraint, and the sketch is determinate
// like every other sketch here. Without it the point is free in two directions.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	diameter := p[pBore]
	if diameter <= 0 {
		proofkit.Unmodelled(t, "bore diameter %.3f: buildBore returns before creating the sketch, so there is no sketch to prove", diameter)
	}

	proofkit.Step(t, "constructor: the stray local origin the Bore Profile sketch never uses")
	stray := s.CreatePoint(0, 0)
	stray.SetName("strayLocalOrigin")
	s.AddConstraint(sketch.NewCoincident(stray, s.Origin()))

	proofkit.Step(t, "drawBore: project the anchor in and draw the bore circle on it, diameter %.3f", diameter)
	anchor := s.CreateReferencePoint(p[pAnchorX], p[pAnchorY], "Tools/anchorPoint")
	anchor.SetName("projectedAnchor")
	boreCircle := s.CreateCircle(anchor, diameter/2)
	boreCircle.SetName("boreCircle")
	s.AddConstraint(sketch.NewDiameter(boreCircle, diameter))
}
