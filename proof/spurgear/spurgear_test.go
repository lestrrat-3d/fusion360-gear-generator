// Package spurgear_test proves the spur gear's sketch scheme against the
// standalone sketch engine, before any Fusion code exists.
//
// It follows the compiled step list in .tmp/spurgear.steps.md: one Go function
// per [GO] step, named after the step's title. The spur build has three
// sketches; two of them are proved here.
//
//   - S3 Tools Sketch      -> stepToolsSketch
//   - S5 Gear Profile      -> stepGearProfileSketch
//
// The third sketch, Bore Profile (S14), is deliberately not proved. The tooth
// generator's constructor always adds a local-origin SketchPoint at (0, 0) and
// drawBore never anchors it, so that sketch carries a point nothing constrains
// and can never reach DOF 0. Removing the point to make the proof pass would be
// proving a build the generator does not make, so the step stays [PROSE] and the
// defect is reported instead.
//
// Everything else in the build is 3D — extrudes, chamfer, pattern, combine,
// fillet, bore cut — which this engine does not model at all.
package spurgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// gear is one case's parameter set, resolved the way processInputs resolves the
// dialog: the four primary inputs plus the circle radii derived from them.
//
// Lengths are millimetres. Fusion works in centimetres internally, but the
// scheme is scale-free and the engine's conditioning measure is scale-invariant,
// so the unit only has to be consistent.
type gear struct {
	Module        float64
	ToothNumber   float64
	PressureAngle float64 // radians, as the PressureAngle parameter stores it
	Steps         int     // InvoluteSteps
	Angle         float64 // radians; draw()'s angle argument, 0 on the spur base
	involute.Dimensions
}

func newGear(p map[string]float64) gear {
	g := gear{
		Module:        p["module"],
		ToothNumber:   p["toothNumber"],
		PressureAngle: p["pressureAngleDeg"] * math.Pi / 180,
		Steps:         int(p["involuteSteps"]),
		Angle:         p["toothAngleDeg"] * math.Pi / 180,
	}
	g.Dimensions = involute.Derive(g.Module, g.ToothNumber, g.PressureAngle)
	return g
}

// sweep is the parameter table every proof runs against.
//
// The four required sizes all leave the base circle outside the root circle, so
// on their own they never take step S5 down its embedded branch. Two more cases
// cover where that branch begins. At 20 degrees the two circles cross at just
// under 42 teeth: 41 teeth leaves a stub 0.014 mm long, the near-degenerate
// region the spec flags as ill-conditioned, and 60 teeth is comfortably
// embedded — no stubs, a four-curve tooth loop.
//
// The 41-tooth case passes the whole gate, conditioning included. The scheme
// does not become fragile there, whatever the spec expects of it.
func sweep() []proofkit.Case {
	mk := func(name string, module, teeth float64) proofkit.Case {
		return proofkit.Case{
			Name: name,
			Params: map[string]float64{
				"module":           module,
				"toothNumber":      teeth,
				"pressureAngleDeg": 20,
				"involuteSteps":    15,
				"toothAngleDeg":    0,
			},
		}
	}
	return []proofkit.Case{
		mk("module1_teeth12", 1, 12),
		mk("module1_teeth17", 1, 17),
		mk("module2_teeth20", 2, 20),
		mk("module3_teeth15", 3, 15),
		mk("module1_teeth41_stub_almost_zero", 1, 41),
		mk("module1_teeth60_embedded", 1, 60),
	}
}

func TestToolsSketch(t *testing.T) {
	proofkit.Run(t, sweep(), func(t testing.TB, s *sketch.Sketch, p map[string]float64) {
		stepToolsSketch(t, s, newGear(p))
	})
}

func TestGearProfileSketch(t *testing.T) {
	proofkit.Run(t, sweep(), func(t testing.TB, s *sketch.Sketch, p map[string]float64) {
		stepGearProfileSketch(t, s, newGear(p))
	})
}

// stepToolsSketch realises step S3.
//
// The Tools sketch holds exactly one thing: the projection of the user's anchor
// point. It draws no geometry of its own. The projection is modelled with
// CreateReferencePoint rather than a fixed point because that is what a Fusion
// projection is — geometry brought in from outside the sketch, whose position
// the sketch does not get to choose.
//
// The plane-local coordinates of the projection are (0, 0) here. The anchor can
// sit anywhere in the model, but a sketch's own frame is whatever the plane
// gives it, and every rule in the scheme is stated relative to the projection,
// so putting it at the local origin costs the proof nothing.
func stepToolsSketch(t testing.TB, s *sketch.Sketch, g gear) *sketch.Point {
	proofkit.Step(t, "Tools sketch: project the anchor point")
	anchor := s.CreateReferencePoint(0, 0, "anchorPoint selection")
	anchor.SetName("toolsAnchorProjection")
	return anchor
}

// profileSketch collects the handles step S5's parts pass between each other.
type profile struct {
	localOrigin *sketch.Point
	root        *sketch.Circle
	tip         *sketch.Circle
	base        *sketch.Circle
	pitch       *sketch.Circle
	left        *sketch.FitSpline
	right       *sketch.FitSpline
	spine       *sketch.Line
	toothTop    *sketch.Point
}

// stepGearProfileSketch realises step S5 — the whole Gear Profile sketch, which
// is one Fusion timeline entry however much goes into it.
//
// The parts run in the order the tooth generator's draw() runs them: the four
// circles, the tooth, then the anchoring that slides the drawing onto the user's
// anchor. Each part is announced with proofkit.Step.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, g gear) {
	pf := &profile{}

	// The movable local origin: a fresh point at (0, 0), not the sketch's own
	// origin point. Everything is drawn relative to it and it is anchored last.
	pf.localOrigin = s.CreatePoint(0, 0)
	pf.localOrigin.SetName("localOrigin")

	drawCircles(t, s, g, pf)
	drawTooth(t, s, g, pf)
	anchorSketch(t, s, g, pf)
}

// drawCircles is drawCircles: four circles sharing the local origin as their
// centre, each with a driving diameter dimension.
//
// The root circle is solid and the other three are construction. Only the root
// circle bounds real material, and it is the curve the tooth's root arc is cut
// out of by profile detection — which is why it is drawn whole rather than as
// an arc between the two flank feet.
func drawCircles(t testing.TB, s *sketch.Sketch, g gear, pf *profile) {
	proofkit.Step(t, "gear profile: four circles on the local origin (root %.4f, tip %.4f, base %.4f, pitch %.4f)",
		g.Root, g.Tip, g.Base, g.Pitch)

	add := func(name string, radius float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(pf.localOrigin, radius)
		c.SetName(name)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, radius*2))
		return c
	}
	pf.root = add("rootCircle", g.Root, false)
	pf.tip = add("tipCircle", g.Tip, true)
	pf.base = add("baseCircle", g.Base, true)
	pf.pitch = add("pitchCircle", g.Pitch, true)
}

// drawTooth is drawTooth: one involute tooth, drawn at its final angular
// position and then held there by the constraint network.
func drawTooth(t testing.TB, s *sketch.Sketch, g gear, pf *profile) {
	left, right := involute.Flanks(g.Base, g.Tip, g.Pitch, g.ToothNumber, g.Steps, g.Angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "the flank sampled to %d point(s); a spline needs at least two", len(left))
	}
	n := len(left)

	proofkit.Step(t, "gear profile: %d-point involute flanks as fitted splines", n)
	lp := flankPoints(s, "leftFlank", left)
	rp := flankPoints(s, "rightFlank", right)
	ls, err := s.CreateFitSpline(lp...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	rs, err := s.CreateFitSpline(rp...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	ls.SetName("leftFlank")
	rs.SetName("rightFlank")
	pf.left, pf.right = ls, rs

	drawToothTopArc(t, s, g, pf, lp[n-1], rp[n-1])
	drawSpine(t, s, g, pf)
	drawRibs(t, s, g, pf, lp, rp)
	drawFlankToRoot(t, s, g, pf, lp[0], rp[0])
}

func flankPoints(s *sketch.Sketch, name string, pts []involute.Pt) []*sketch.Point {
	out := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		out[i] = s.CreatePoint(p.X, p.Y)
		out[i].SetName(fmt.Sprintf("%s%d", name, i))
	}
	return out
}

// drawToothTopArc caps the tooth at the tip circle.
//
// The arc shares three points — the local origin as its centre and the two
// flank end points — and carries no dimension of its own. A free centre plus a
// diameter dimension would size the arc without saying which way it bulges, and
// the inward answer runs back through the tooth.
//
// The tooth-top point materialised here is not on the arc. It is the spine's far
// end, and it is put on the tip circle so the spine reaches exactly as far as
// the tooth does.
func drawToothTopArc(t testing.TB, s *sketch.Sketch, g gear, pf *profile, leftEnd, rightEnd *sketch.Point) {
	proofkit.Step(t, "gear profile: tooth-top point on the tip circle, arc centred on the local origin")

	x, y := involute.Rotate(g.Tip, 0, g.Angle)
	pf.toothTop = s.CreatePoint(x, y)
	pf.toothTop.SetName("toothTopPoint")
	s.AddConstraint(sketch.NewPointOnCircle(pf.toothTop, pf.tip))

	// Counter-clockwise from the right flank's end to the left flank's end, so
	// the arc sweeps across +X through the tooth top rather than the long way
	// round the gear.
	arc := s.CreateArc(pf.localOrigin, rightEnd, leftEnd)
	arc.SetName("toothTopArc")
}

// drawSpine draws the tooth's axis of symmetry and pins its absolute direction.
//
// The pin is an angular dimension against a reference line built for every
// angle, including zero. A plain horizontal on the spine would fix the line's
// direction but not which end of it the tooth top sits at, and the sketch would
// reach DOF 0 with the tooth 180 degrees round the gear.
//
// The reference line's far end is placed with two signed offsets from the local
// origin rather than by putting it on the tip circle, because a point on a
// circle has two answers and the one that matters here is at the circle's
// extreme in x, where those two answers are least distinguishable.
func drawSpine(t testing.TB, s *sketch.Sketch, g gear, pf *profile) {
	proofkit.Step(t, "gear profile: spine, +X reference line, and the angular pin at %.4f rad", g.Angle)

	pf.spine = s.CreateLine(pf.localOrigin, pf.toothTop)
	pf.spine.SetName("spine")
	pf.spine.SetConstruction(true)

	refEnd := s.CreatePoint(g.Tip, 0)
	refEnd.SetName("xReferenceEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(pf.localOrigin, refEnd, g.Tip),
		sketch.NewVerticalDistance(pf.localOrigin, refEnd, 0),
	)
	ref := s.CreateLine(pf.localOrigin, refEnd)
	ref.SetName("xReference")
	ref.SetConstruction(true)

	// Signed, counter-clockwise from the reference to the spine — the argument
	// order decides the sign, and the engine reads the value in degrees.
	s.AddConstraint(sketch.NewAngle(ref, pf.spine, g.Angle*180/math.Pi))
}

// drawRibs locks each pair of matching flank fit points to the spine.
//
// One rib per fit-point index, endpoints included: the fit points carry no other
// constraint, so an omitted rib leaves that pair of points free.
//
// Each rib is built in a fixed order — rib, signed width, midpoint seeded on the
// spine, point-on-line, midpoint-of, perpendicular — and the last rib gets no
// perpendicular, because the tooth-top arc already holds those two ends at equal
// radius either side of the spine and a second statement of that is redundant.
//
// The widths are signed vertical offsets rather than lengths. A length is
// satisfied equally well with the two flanks swapped, which comes out as a
// mirrored tooth. The same reasoning drives the chain of midpoint offsets along
// the spine, which starts at the local origin so the chain cannot slide outward
// as a unit.
func drawRibs(t testing.TB, s *sketch.Sketch, g gear, pf *profile, lp, rp []*sketch.Point) {
	proofkit.Step(t, "gear profile: %d ribs, midpoint chain from the local origin", len(lp))

	sa, ca := math.Sin(g.Angle), math.Cos(g.Angle)
	prev := pf.localOrigin
	for i := range lp {
		l, r := lp[i], rp[i]

		rib := s.CreateLine(l, r)
		rib.SetName(fmt.Sprintf("rib%d", i))
		rib.SetConstruction(true)
		s.AddConstraint(sketch.NewVerticalDistance(l, r, r.Y()-l.Y()))

		// Seeded at the foot of the left fit point on the spine, not at the
		// rib's own midpoint: the constraints below put it there, and the
		// solver picks its branch from where the geometry starts.
		foot := l.X()*ca + l.Y()*sa
		mid := s.CreatePoint(foot*ca, foot*sa)
		mid.SetName(fmt.Sprintf("ribMid%d", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, pf.spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(lp)-1 {
			s.AddConstraint(sketch.NewPerpendicular(pf.spine, rib))
		}

		s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, mid.X()-prev.X()))
		prev = mid
	}
}

// drawFlankToRoot closes the tooth at the root.
//
// The flank starts on the base circle. When that is outside the root circle a
// short radial stub runs from the root circle up to it on each side, and the
// tooth loop comes out with six curves. When the base circle has dropped inside
// the root circle — high tooth counts — the flank already starts below the root
// and no stub is drawn; the loop has four curves and the profile is embedded.
// The test is strict: an exactly equal pair counts as not embedded and draws a
// zero-length stub, which is the ill-conditioned case worth surfacing rather
// than smoothing away.
//
// Each stub's root end is placed with two signed offsets from the local origin.
// Putting it on the root circle and the local origin on the stub's line instead
// would be satisfied by two points — the line carries on through the centre and
// meets the root circle again on the far side — and the sketch would reach DOF 0
// with a stub that crosses the whole gear.
func drawFlankToRoot(t testing.TB, s *sketch.Sketch, g gear, pf *profile, leftStart, rightStart *sketch.Point) {
	if g.Embedded() {
		proofkit.Step(t, "gear profile: embedded profile (base %.4f < root %.4f), no flank-to-root stubs", g.Base, g.Root)
		return
	}
	proofkit.Step(t, "gear profile: radial flank-to-root stubs (base %.4f >= root %.4f)", g.Base, g.Root)

	stub := func(name string, flankStart *sketch.Point) {
		// Radial: the same direction as the flank start, at the root radius.
		scale := g.Root / g.Base
		x, y := flankStart.X()*scale, flankStart.Y()*scale

		end := s.CreatePoint(x, y)
		end.SetName(name + "End")
		line := s.CreateLine(end, flankStart)
		line.SetName(name)
		s.AddConstraint(
			sketch.NewHorizontalDistance(pf.localOrigin, end, x),
			sketch.NewVerticalDistance(pf.localOrigin, end, y),
		)
	}
	stub("leftFlankToRoot", leftStart)
	stub("rightFlankToRoot", rightStart)
}

// anchorSketch is the step-5 anchoring, which draw() performs itself.
//
// The Tools sketch's anchor projection is projected in again and the local
// origin is made coincident with it. Every piece of geometry above is placed
// relative to the local origin, so this one constraint drags the whole drawing
// onto the user's anchor and grounds the sketch at the same time.
func anchorSketch(t testing.TB, s *sketch.Sketch, g gear, pf *profile) {
	proofkit.Step(t, "gear profile: re-project the Tools anchor and make the local origin coincident with it")

	projected := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	projected.SetName("anchorProjection")
	s.AddConstraint(sketch.NewCoincident(projected, pf.localOrigin))
}
