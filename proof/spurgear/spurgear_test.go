// Package spurgear_test proves the spur gear's sketch construction.
//
// Two of the spur generator's steps are proved here. Tools holds nothing but the
// projected user anchor, and Gear Profile holds the four gear circles and one
// involute tooth. The extrudes, the chamfer, the circular pattern, the root
// fillets and the bore cut are solid modelling, which the sketch engine does not
// do, so those steps are proved by reading and are marked [PROSE].
//
// The Bore Profile sketch is a sketch and is still [PROSE]. As specified it
// cannot reach a verdict: the tooth generator's constructor leaves a stray
// unconstrained point at the origin, and nothing constrains the projected centre
// either, so the sketch is under-constrained by construction. Proving it would
// mean waiving the gate, and the step list records it as a defect instead.
//
// Each step function here is named by exactly one [GO] step of
// .tmp/spurgear.steps.md, and every [GO] step names one of these.
//
// The compiled spec is spec/spurgear/instructions.md plus spec/spurgear/fusion.md
// plus .claude/skills/generate-gear/PLAYBOOK.md; the step list records the blob
// hash of each.
package spurgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys of a case. Angles are in degrees, which is the sketch's own
// default angle unit, so a dimension value needs no conversion on the way in.
const (
	pModule        = "module"
	pToothNumber   = "toothNumber"
	pPressureAngle = "pressureAngle"
	pInvoluteSteps = "involuteSteps"
	pAngle         = "angle"
)

// spurCases sweeps the sizes the scheme has to hold across.
//
// The ninth part of S5 branches on where the flank starts. At 20 degrees the base circle
// drops below the root circle once the tooth count passes about 41.5, so the
// four required sizes all take the flank-to-root branch and none of them reaches
// the embedded one. z60 and z44 reach it; z41 sits one tooth on the other side
// of the boundary, where the stub is at its shortest. The two rotated cases
// exercise the angle argument the tooth generator takes — 0 for a spur gear, the
// helix angle for the gears that subclass it — in both directions, since the
// signed dimensions the scheme leans on would pass a positive angle and fail a
// negative one.
//
// The last case is exactly degenerate by construction: cos(pressure angle) =
// 1 - 2.5/z puts the base circle exactly on the root circle, the region
// [SPUR-F-FLANK-ROOT] flags. The proof cannot model it and says so.
var spurCases = []proofkit.Case{
	{Name: "m1_z12", Params: map[string]float64{pModule: 1, pToothNumber: 12, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m1_z17", Params: map[string]float64{pModule: 1, pToothNumber: 17, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m2_z20", Params: map[string]float64{pModule: 2, pToothNumber: 20, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m3_z15", Params: map[string]float64{pModule: 3, pToothNumber: 15, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m1_z41_shortest_stub", Params: map[string]float64{pModule: 1, pToothNumber: 41, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m1_z60_embedded", Params: map[string]float64{pModule: 1, pToothNumber: 60, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m2_z44_embedded", Params: map[string]float64{pModule: 2, pToothNumber: 44, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 0}},
	{Name: "m2_z20_rotated_30deg", Params: map[string]float64{pModule: 2, pToothNumber: 20, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: 30}},
	{Name: "m1_z17_rotated_minus20deg", Params: map[string]float64{pModule: 1, pToothNumber: 17, pPressureAngle: 20, pInvoluteSteps: 15, pAngle: -20}},
	{Name: "m2_z20_base_on_root", Params: map[string]float64{pModule: 2, pToothNumber: 20, pPressureAngle: degrees(math.Acos(1 - 2.5/20)), pInvoluteSteps: 15, pAngle: 0}},
}

func TestSpurGear(t *testing.T) {
	proofkit.Run(t, spurCases, buildSpurGear)
}

// buildSpurGear runs the sketch-bearing steps of the build in spec order.
//
// s is the Gear Profile sketch, the one proofkit gates. The Tools sketch is a
// second sketch of the same build, so S3 makes and gates its own.
func buildSpurGear(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	dims := involute.Derive(p[pModule], p[pToothNumber], radians(p[pPressureAngle]))

	// Which branch of that ninth part runs is decided by a strict comparison of two
	// radii the spec deliberately leaves untoleranced. When they are equal the
	// comparison is decided by rounding: called embedded, the tooth's root
	// boundary lies on the root circle and encloses nothing; called not
	// embedded, the flank-to-root stub is drawn with zero length and carries no
	// direction. Neither shape is a sketch this proof can put a verdict on.
	if math.Abs(dims.Base-dims.Root) < 1e-6 {
		proofkit.Unmodelled(t, "base circle sits on the root circle (both %.6f mm), the region "+
			"[SPUR-F-FLANK-ROOT]'s strict test resolves by rounding", dims.Base)
	}

	stepToolsSketch(t)
	stepGearProfileSketch(t, s, dims, p[pToothNumber], p[pAngle], int(p[pInvoluteSteps]))
}

// gear carries the handles the sketch steps hand to each other, standing in for
// the fields the spec pins on SpurGearInvoluteToothDesignGenerator.
type gear struct {
	dims        involute.Dimensions
	toothNumber float64
	angleDeg    float64 // the draw(anchorPoint, angle=…) argument
	angleRad    float64

	origin                 *sketch.Point // self.anchorPoint, the movable local origin
	root                   *sketch.Circle
	tip, base, pitch       *sketch.Circle
	spine                  *sketch.Line
	spineAngle             *sketch.Angle
	left, right            *sketch.FitSpline
	leftFit, rightFit      []*sketch.Point
	toothProfileIsEmbedded bool
}

// stepToolsSketch proves S3: the Tools sketch.
//
// The sketch owns one thing, the projection of the user's anchor point, and
// draws no geometry of its own. A projection is reference geometry — externally
// located, not solver-located — so the sketch is complete with that one point
// and nothing holding it.
func stepToolsSketch(t testing.TB) {
	t.Helper()
	proofkit.Step(t, "S3: Tools sketch — project the anchor point in and keep it as ctx.anchorPoint")

	tools := proofkit.NewSketch(t)
	anchor := tools.CreateReferencePoint(0, 0, "user anchor point")
	anchor.SetName("ctx.anchorPoint")

	proofkit.RequireSound(t, tools)
}

// stepGearProfileSketch proves S5: the Gear Profile sketch, the one sketch the
// whole gear is cut from.
//
// It is a single timeline entry, so it is a single step, and it runs the three
// parts of the tooth generator's draw() in that method's own order: the circles,
// the tooth, then the anchoring that slides the result onto the user's point.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, dims involute.Dimensions, toothNumber, angleDeg float64, steps int) {
	t.Helper()

	g := &gear{
		dims:        dims,
		toothNumber: toothNumber,
		angleDeg:    angleDeg,
		angleRad:    radians(angleDeg),
	}

	g.origin = s.CreatePoint(0, 0)
	g.origin.SetName("localOrigin")

	drawCircles(t, s, g)
	drawTooth(t, s, g, steps)
	anchorToProjectedPoint(t, s, g)
}

// drawCircles is the first part of S5: the four gear circles.
//
// Every circle is created on the local origin point itself rather than at its
// coordinates, so all four share it and no coincidence is needed or wanted. The
// root circle is the only solid one; it is what the body profile and the tooth's
// root boundary are cut from. The other three are construction and exist to be
// measured against.
//
// The along-path label each circle also carries is annotation. It holds no
// constraint, so it changes nothing here and is not drawn.
func drawCircles(t testing.TB, s *sketch.Sketch, g *gear) {
	t.Helper()
	proofkit.Step(t, "S5 drawCircles: the four gear circles, all centred on the shared local origin")

	circle := func(name string, r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(g.origin, r)
		c.SetName(name)
		c.SetConstruction(construction)
		d := sketch.NewDiameter(c, 2*r)
		s.AddConstraint(d)
		s.SetConstraintName(d, name+" diameter")
		return c
	}

	g.root = circle("rootCircle", g.dims.Root, false)
	g.tip = circle("tipCircle", g.dims.Tip, true)
	g.base = circle("baseCircle", g.dims.Base, true)
	g.pitch = circle("pitchCircle", g.dims.Pitch, true)
}

// drawTooth is the second part of S5: one involute tooth, drawn at its final
// angle.
//
// The flank points come from the shared involute package, which mirrors, rotates
// to centre the tooth on +X, and then applies the requested angle — the order the
// spec pins. Nothing here places a point by fixing a coordinate: the seeds are
// the drawn positions, and every one of them is then located by a constraint.
func drawTooth(t testing.TB, s *sketch.Sketch, g *gear, steps int) {
	t.Helper()

	proofkit.Step(t, "S5 drawTooth 1-5: sample both flanks and draw them as fitted splines")
	left, right := involute.Flanks(g.dims.Base, g.dims.Tip, g.dims.Pitch, g.toothNumber, steps, g.angleRad)
	if len(left) < 2 {
		t.Fatalf("involute sampling produced %d point(s), too few for a flank", len(left))
	}

	g.leftFit = make([]*sketch.Point, len(left))
	g.rightFit = make([]*sketch.Point, len(right))
	for i := range left {
		g.leftFit[i] = s.CreatePoint(left[i].X, left[i].Y)
		g.leftFit[i].SetName(fmt.Sprintf("leftFit%d", i))
		g.rightFit[i] = s.CreatePoint(right[i].X, right[i].Y)
		g.rightFit[i].SetName(fmt.Sprintf("rightFit%d", i))
	}

	var err error
	if g.left, err = s.CreateFitSpline(g.leftFit...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	g.left.SetName("leftFlank")
	if g.right, err = s.CreateFitSpline(g.rightFit...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	g.right.SetName("rightFlank")

	last := len(left) - 1

	proofkit.Step(t, "S5 drawTooth 6: tooth-top point on the tip circle, then the tooth-top arc centred on the local origin")
	top := s.CreatePoint(g.dims.Tip*math.Cos(g.angleRad), g.dims.Tip*math.Sin(g.angleRad))
	top.SetName("toothTop")
	onTip := sketch.NewPointOnCircle(top, g.tip)
	s.AddConstraint(onTip)
	s.SetConstraintName(onTip, "toothTop on tipCircle")

	// Centre, start and end are all shared points, so the arc needs no
	// coincidences of its own and carries no diameter dimension. Sweeping
	// counter-clockwise from the right flank's end to the left flank's end
	// makes it bulge outward over +X rather than back through the tooth.
	arc := s.CreateArc(g.origin, g.rightFit[last], g.leftFit[last])
	arc.SetName("toothTopArc")

	proofkit.Step(t, "S5 drawTooth 7: spine, +X reference line, and the angular dimension that says which way the tooth points")
	g.spine = s.CreateLine(g.origin, top)
	g.spine.SetName("spine")
	g.spine.SetConstruction(true)

	// The reference line's far end is pinned by two signed offsets rather than
	// by a coincidence to the tip circle: a point on a circle has two answers,
	// and the one the offsets name is unambiguous.
	xEnd := s.CreatePoint(g.dims.Tip, 0)
	xEnd.SetName("xReferenceEnd")
	xh := sketch.NewHorizontalDistance(g.origin, xEnd, g.dims.Tip)
	xv := sketch.NewVerticalDistance(g.origin, xEnd, 0)
	s.AddConstraint(xh, xv)
	s.SetConstraintName(xh, "xReference dx")
	s.SetConstraintName(xv, "xReference dy")

	ref := s.CreateLine(g.origin, xEnd)
	ref.SetName("xReference")
	ref.SetConstruction(true)

	// Measured from the reference to the spine, in that order, so the value is
	// the requested angle and not its supplement. A signed angle admits one
	// configuration, which is what keeps the tooth off the 180-degree branch.
	g.spineAngle = sketch.NewAngle(ref, g.spine, g.angleDeg)
	s.AddConstraint(g.spineAngle)
	s.SetConstraintName(g.spineAngle, "spine angle from +X")

	proofkit.Step(t, "S5 drawTooth 8: one rib per fit-point index, %d in all, each with its midpoint on the spine", len(left))
	// The spine points along the tooth angle and the ribs run across it, so the
	// well-conditioned signed direction for each swaps at 45 degrees.
	spineIsMostlyHorizontal := math.Abs(math.Cos(g.angleRad)) >= math.Abs(math.Sin(g.angleRad))

	prev := g.origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(g.leftFit[i], g.rightFit[i])
		rib.SetName(fmt.Sprintf("rib%d", i))
		rib.SetConstruction(true)

		// A signed width, not an aligned one. An aligned dimension gives only a
		// length, which the left and right flanks satisfy just as well swapped
		// over, and the tooth comes out mirrored.
		//
		// What the proof cannot reach: this engine's horizontal and vertical
		// distances carry a sign, so a negative value names the mirrored answer
		// and rules it out. Fusion's addDistanceDimension takes a magnitude plus
		// an orientation, which may leave the branch to the seed rather than to
		// the dimension. If a Fusion run ever produces a mirrored tooth from
		// this scheme, that difference is where to look, and the fix is a
		// constraint that carries a direction rather than a tighter seed.
		var width sketch.Constraint
		if spineIsMostlyHorizontal {
			width = sketch.NewVerticalDistance(g.leftFit[i], g.rightFit[i], right[i].Y-left[i].Y)
		} else {
			width = sketch.NewHorizontalDistance(g.leftFit[i], g.rightFit[i], right[i].X-left[i].X)
		}
		s.AddConstraint(width)
		s.SetConstraintName(width, fmt.Sprintf("rib%d width", i))

		// Seeded at the foot of the left fit point on the spine, so it starts on
		// the line it is about to be pinned to.
		foot := left[i].X*math.Cos(g.angleRad) + left[i].Y*math.Sin(g.angleRad)
		midX, midY := foot*math.Cos(g.angleRad), foot*math.Sin(g.angleRad)
		mid := s.CreatePoint(midX, midY)
		mid.SetName(fmt.Sprintf("ribMid%d", i))

		onSpine := sketch.NewPointOnLine(mid, g.spine)
		s.AddConstraint(onSpine)
		s.SetConstraintName(onSpine, fmt.Sprintf("ribMid%d on spine", i))

		isMid := sketch.NewMidpoint(mid, rib)
		s.AddConstraint(isMid)
		s.SetConstraintName(isMid, fmt.Sprintf("ribMid%d bisects rib%d", i, i))

		// The last rib gets no perpendicular. The tooth-top arc shares the local
		// origin as its centre, which already holds the two flank ends at equal
		// radius either side of the spine, so a perpendicular there says nothing
		// new and the solver reports it as redundant.
		if i != last {
			perp := sketch.NewPerpendicular(g.spine, rib)
			s.AddConstraint(perp)
			s.SetConstraintName(perp, fmt.Sprintf("rib%d perpendicular to spine", i))
		}

		// The chain of signed spacings along the spine, starting from the local
		// origin. Without the first link the whole chain slides along the spine
		// as one and the sketch never closes.
		var spacing sketch.Constraint
		if spineIsMostlyHorizontal {
			spacing = sketch.NewHorizontalDistance(prev, mid, midX-prevX)
		} else {
			spacing = sketch.NewVerticalDistance(prev, mid, midY-prevY)
		}
		s.AddConstraint(spacing)
		s.SetConstraintName(spacing, fmt.Sprintf("ribMid%d spacing from %s", i, prev.Name()))

		prev, prevX, prevY = mid, midX, midY
	}

	g.toothProfileIsEmbedded = g.dims.Embedded()
	wantCurves := 6
	if g.toothProfileIsEmbedded {
		proofkit.Step(t, "S5 drawTooth 9: base circle (r=%.4f mm) is inside the root circle (r=%.4f mm), so the "+
			"profile is embedded and no flank-to-root line is drawn", g.dims.Base, g.dims.Root)
		wantCurves = 4
	} else {
		proofkit.Step(t, "S5 drawTooth 9: base circle (r=%.4f mm) is outside the root circle (r=%.4f mm), so a "+
			"radial flank-to-root line closes each side", g.dims.Base, g.dims.Root)
		stub := func(name string, flank involute.Pt, flankStart *sketch.Point) {
			r := math.Hypot(flank.X, flank.Y)
			rootX, rootY := flank.X/r*g.dims.Root, flank.Y/r*g.dims.Root
			end := s.CreatePoint(rootX, rootY)
			end.SetName(name + "RootEnd")

			line := s.CreateLine(end, flankStart)
			line.SetName(name)

			// Exactly two signed offsets from the local origin, and nothing
			// else. "Root end on the root circle" plus "origin on the line"
			// would be satisfied by the far side of the gear too, and the stub
			// would become a line straight across the centre.
			dx := sketch.NewHorizontalDistance(g.origin, end, rootX)
			dy := sketch.NewVerticalDistance(g.origin, end, rootY)
			s.AddConstraint(dx, dy)
			s.SetConstraintName(dx, name+" root end dx")
			s.SetConstraintName(dy, name+" root end dy")
		}
		stub("leftFlankToRoot", left[0], g.leftFit[0])
		stub("rightFlankToRoot", right[0], g.rightFit[0])
	}

	// The root boundary of the tooth is never drawn. It is the piece of the root
	// circle the profile detector hands back once the loop is closed either way,
	// which is what makes the count 6 or 4 rather than 5 or 3. Steps S7 and S9
	// ask for the tooth and body profiles by exactly these counts, so the count
	// is checked where it is decided.
	assertToothLoop(t, s, arc, wantCurves)
}

// assertToothLoop checks that the tooth closes into one region with the curve
// count S5's ninth part says it has: 2 flanks, 2 arcs, plus the 2 flank-to-root
// lines when the profile is not embedded.
func assertToothLoop(t testing.TB, s *sketch.Sketch, top *sketch.Arc, want int) {
	t.Helper()
	for _, p := range s.Profiles() {
		for _, e := range p.Entities {
			a, ok := e.(*sketch.Arc)
			if !ok || a != top {
				continue
			}
			if !p.Valid {
				t.Errorf("tooth region is not extrudable")
			}
			if got := len(p.Entities); got != want {
				t.Errorf("tooth loop closes on %d curve(s), want %d", got, want)
			}
			return
		}
	}
	t.Errorf("no closed region uses the tooth-top arc, so the tooth loop did not close")
}

// anchorToProjectedPoint is the last part of S5: sliding the drawing onto the
// user's anchor.
//
// The Tools-sketch anchor is projected in again, which is what chains this sketch
// to the user's original entity, and the local origin is made coincident with it.
// Every point above is placed relative to that origin, so this one constraint
// carries the whole tooth.
//
// The confirming angle value-set is the last action of the whole draw, after the
// constraint network exists — the second half of the draw-and-confirm rule, whose
// first half was drawing the geometry already rotated.
func anchorToProjectedPoint(t testing.TB, s *sketch.Sketch, g *gear) {
	t.Helper()
	proofkit.Step(t, "S5 draw: project the Tools anchor in and make the local origin coincident with it")

	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	anchor.SetName("projectedAnchor")

	c := sketch.NewCoincident(g.origin, anchor)
	s.AddConstraint(c)
	s.SetConstraintName(c, "localOrigin on projectedAnchor")

	proofkit.Step(t, "S5 draw: confirm the drawn rotation by setting the spine's angular dimension to %.4g deg", g.angleDeg)
	g.spineAngle.Set(g.angleDeg)
}

func radians(deg float64) float64 { return deg * math.Pi / 180 }
func degrees(rad float64) float64 { return rad * 180 / math.Pi }
