// Package spurgear_test proves the three sketches of the spur gear build.
//
// The build has three sketches — Tools, Gear Profile and Bore Profile — and one
// step function each, named for the step it realises in the compiled step list
// (.tmp/spurgear.steps.md). Everything else in the build is 3D (extrude,
// chamfer, pattern, combine, fillet, cut) and is prose there, because the engine
// models 2D sketches only.
//
// The tooth math is not repeated here: proof/involute owns it, because helical
// and herringbone draw the same tooth rotated.
//
// # Two places the bench model differs from Fusion, deliberately
//
// A projected anchor is modelled as a locked reference point
// (Sketch.CreateReferencePoint). Fusion's sketch.project brings a point in
// associatively and it keeps free degrees of freedom ([PB-PROJECT-NOT-FIXED]),
// so the real Tools sketch is NOT fully constrained and neither is the anchor
// coincidence that grounds the other two. Locking it is the only way to model
// what the spec means when it says the local origin "rides on the projected
// anchor". This is the honest edge of the check: it proves every sketch is
// determinate GIVEN the anchor, not that Fusion reports isFullyConstrained.
//
// Profile detection splits a closed curve at its own +X seam as well as at
// crossings. A tooth drawn at angle 0 straddles that seam, so its root arc is
// reported as two circle fragments and the tooth loop reads 7 edges instead of
// the 6 that spec step 7 expects; the same tooth at any other angle reads 6.
// Fusion has no seam, so the count the extrude step matches on is the rotated
// one. The regions, their areas and their validity are the same either way.
package spurgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// cases sweeps the branches the build actually has: the two profile shapes
// (stub and embedded), the two rib/chain axis choices, an anchor away from the
// plane origin, and the boundary where the flank starts exactly on the root
// circle. Sizes are kept few and branches many — the ambiguity probe dominates
// the runtime, and a fifth similar module proves nothing a fourth did not.
var cases = []proofkit.Case{
	// The required sweep.
	{Name: "m1_t12_pa20", Params: params(1, 12, 20, 0, 0, 0, 3)},
	{Name: "m1_t17_pa20", Params: params(1, 17, 20, 0, 0, 0, 4)},
	{Name: "m2_t20_pa20", Params: params(2, 20, 20, 0, 0, 0, 6)},
	{Name: "m3_t15_pa20", Params: params(3, 15, 20, 0, 0, 0, 10)},

	// The anchor is a user-picked point anywhere on the plane, not the plane
	// origin; every sketch's local origin has to travel to it.
	{Name: "m2_t20_pa20_anchorOffPlaneOrigin", Params: params(2, 20, 20, 0, 37.5, -14.25, 6)},

	// Embedded profiles: the base circle sinks below the root circle above
	// 2.5/(1-cos(pressureAngle)) teeth, so no flank-to-root stubs are drawn.
	// Once at 20 degrees (41.5 teeth) and once at 25, where it starts at 26.7.
	{Name: "m1_t45_pa20_embedded", Params: params(1, 45, 20, 0, 0, 0, 8)},
	{Name: "m1_t28_pa25_embedded", Params: params(1, 28, 25, 0, 0, 0, 6)},

	// Either side of that threshold. The first leaves a stub one micrometre
	// long — the region [SPUR-F-FLANK-ROOT] calls ill-conditioned. The second
	// is the exact boundary, where the stub has zero length.
	{Name: "m1_t42_pa19.868_stub1um", Params: params(1, 42, 19.868311223067895, 0, 0, 0, 6)},
	{Name: "m1_t42_exactZeroLengthStub", Params: withBoundary(params(1, 42, 20, 0, 0, 0, 6))},

	// Rotated teeth. The generator takes an angle, and [SPUR-F-RIBS] picks the
	// rib and chain dimension axes from it: vertical rib / horizontal chain
	// while |cos| >= |sin|, swapped otherwise. 30 and 180 take the first
	// branch (180 with a negative cosine), 90 and 120 the second, and 45 sits
	// exactly on the tie.
	{Name: "m1_t17_pa20_rot30", Params: params(1, 17, 20, 30, 0, 0, 4)},
	{Name: "m1_t17_pa20_rot90", Params: params(1, 17, 20, 90, 0, 0, 4)},
	{Name: "m1_t17_pa20_rot120", Params: params(1, 17, 20, 120, 0, 0, 4)},
	{Name: "m1_t17_pa20_rot180", Params: params(1, 17, 20, 180, 0, 0, 4)},
	{Name: "m1_t45_pa20_embedded_rot45", Params: params(1, 45, 20, 45, 0, 0, 8)},
}

func params(module, teeth, pressureAngleDeg, toothAngleDeg, anchorX, anchorY, boreDiameter float64) map[string]float64 {
	return map[string]float64{
		"module":           module,
		"teeth":            teeth,
		"pressureAngleDeg": pressureAngleDeg,
		"involuteSteps":    15,
		"toothAngleDeg":    toothAngleDeg,
		"anchorX":          anchorX,
		"anchorY":          anchorY,
		"boreDiameter":     boreDiameter,
	}
}

// withBoundary drops the root circle onto the base circle, which is the exact
// configuration [SPUR-F-FLANK-ROOT]'s strict `<` test admits as non-embedded.
// No pressure angle reaches it in float64 — the nearest representable one lands
// a hair below and reads as embedded — so the case states it directly.
func withBoundary(p map[string]float64) map[string]float64 {
	p["rootOntoBase"] = 1
	return p
}

func TestToolsSketch(t *testing.T)       { proofkit.Run(t, cases, stepToolsSketch) }
func TestGearProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepGearProfileSketch) }
func TestBoreProfileSketch(t *testing.T) { proofkit.Run(t, cases, stepBoreProfileSketch) }

// stepToolsSketch realises step S5: the Tools sketch draws no geometry of its
// own and exists to own one reference, the projection of the user's Anchor
// Point, which every later sketch re-projects from ([SPUR-F-ANCHOR-CHAIN]).
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user's Anchor Point in and keep it as ctx.anchorPoint")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "user:anchorPoint")
	anchor.SetName("ctx.anchorPoint")
}

// stepBoreProfileSketch realises step S16. The tooth generator's constructor
// always adds a local origin at (0,0,0) ([SPUR-F-LOCAL-ORIGIN]), so the Bore
// Profile sketch carries one stray point that nothing else uses; drawBore's own
// projection of the anchor grounds it, exactly as step S7 grounds the Gear
// Profile's.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	ax, ay, dia := p["anchorX"], p["anchorY"], p["boreDiameter"]
	if dia <= 0 {
		proofkit.Unmodelled(t, "bore diameter %g: buildBore returns before creating the sketch", dia)
	}

	proofkit.Step(t, "the tooth generator constructor adds its local origin")
	origin := s.CreatePoint(ax, ay)
	origin.SetName("localOrigin")

	proofkit.Step(t, "drawBore projects ctx.anchorPoint into this sketch")
	anchor := s.CreateReferencePoint(ax, ay, "Tools:anchorPoint")
	anchor.SetName("projectedAnchor")

	proofkit.Step(t, "bore circle centred on the projection, driving diameter dimension")
	bore := s.CreateCircle(anchor, dia/2)
	bore.SetName("boreCircle")
	s.AddConstraint(sketch.NewDiameter(bore, dia))

	proofkit.Step(t, "ground the stray local origin on that same projection")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
}

// stepGearProfileSketch realises step S7 — the whole Gear Profile sketch, which
// is one timeline entry however much geometry goes into it: the four circles,
// the involute tooth, and the anchoring that slides the drawing onto the user's
// anchor.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := newGear(p)
	if g.zeroStub {
		// A zero-length stub has two distinct endpoints at identical
		// coordinates. The constraint system still closes at DOF 0, but the
		// profile arrangement collapses: the tooth stops being a region of its
		// own and the engine reports the only region it finds as not
		// extrudable. There is no zero-length edge to model this with, and
		// pretending the region is fine would hide what the spec already warns
		// about.
		proofkit.Unmodelled(t,
			"flank start lies exactly on the root circle: the flank-to-root stub has zero "+
				"length, so the tooth bounds no region (root=base=%.6f mm)", g.d.Root)
	}
	n := len(g.left)

	proofkit.Step(t, "project ctx.anchorPoint in and ground the local origin on it")
	anchor := s.CreateReferencePoint(g.ax, g.ay, "Tools:anchorPoint")
	anchor.SetName("projectedAnchor")
	origin := s.CreatePoint(g.ax, g.ay)
	origin.SetName("localOrigin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	// Every circle takes the local-origin point itself as its centre, so all
	// four share it and no centre coincidence is added on top
	// ([PB-SHARE-XOR-COINCIDENT]). Only the root circle is solid; the other
	// three are construction and bound no profile.
	proofkit.Step(t, "four gear circles on the local origin, each with a driving diameter")
	root := s.CreateCircle(origin, g.d.Root)
	root.SetName("rootCircle")
	tip := construction(s.CreateCircle(origin, g.d.Tip), "tipCircle")
	base := construction(s.CreateCircle(origin, g.d.Base), "baseCircle")
	pitch := construction(s.CreateCircle(origin, g.d.Pitch), "pitchCircle")
	s.AddConstraint(
		sketch.NewDiameter(root, 2*g.d.Root),
		sketch.NewDiameter(tip, 2*g.d.Tip),
		sketch.NewDiameter(base, 2*g.d.Base),
		sketch.NewDiameter(pitch, 2*g.d.Pitch),
	)

	proofkit.Step(t, "involute flanks: %d samples per side, drawn as fitted splines", n)
	left := make([]*sketch.Point, n)
	right := make([]*sketch.Point, n)
	for i := range n {
		left[i] = s.CreatePoint(g.ax+g.left[i].X, g.ay+g.left[i].Y)
		left[i].SetName(fmt.Sprintf("leftFit%d", i))
		right[i] = s.CreatePoint(g.ax+g.right[i].X, g.ay+g.right[i].Y)
		right[i].SetName(fmt.Sprintf("rightFit%d", i))
	}
	if _, err := s.CreateFitSpline(left...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(right...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}

	// The arc shares the local origin as its centre and both flank ends as its
	// ends, and carries no diameter dimension ([SPUR-F-TOOTHTOP-ARC]). A free
	// centre plus a diameter would reach DOF 0 with the arc free to bulge back
	// through the tooth.
	proofkit.Step(t, "tooth-top point on the tip circle, then the tooth-top arc")
	top := s.CreatePoint(g.ax+g.d.Tip*math.Cos(g.angle), g.ay+g.d.Tip*math.Sin(g.angle))
	top.SetName("toothTopPoint")
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))
	arc := s.CreateArc(origin, right[n-1], left[n-1])
	arc.SetName("toothTopArc")

	// The angular dimension runs from the reference to the spine, in that
	// order, and it exists for every angle including 0 — it is what says which
	// way the spine points. A plain horizontal would leave the tooth free to
	// settle 180 degrees around ([SPUR-F-SPINE]).
	proofkit.Step(t, "spine, +X reference line, and the angular dimension that pins the tooth")
	spine := construction(s.CreateLine(origin, top), "spine")
	refEnd := s.CreatePoint(g.ax+g.d.Tip, g.ay)
	refEnd.SetName("referenceEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, g.d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := construction(s.CreateLine(origin, refEnd), "plusXReference")
	s.AddConstraint(sketch.NewAngle(reference, spine, g.angleDeg))

	// One rib per fit-point index, endpoints included: the fit points carry no
	// other constraint, so a missing endpoint rib leaves one free. The last rib
	// takes no perpendicular — the tooth-top arc's shared centre already holds
	// its two ends at equal radius, and adding it over-constrains
	// ([SPUR-F-RIBS] step 6).
	proofkit.Step(t, "ribs: one per fit-point index, each with a midpoint on the spine")
	mids := make([]*sketch.Point, n)
	for i := range n {
		rib := construction(s.CreateLine(left[i], right[i]), fmt.Sprintf("rib%d", i))
		if g.ribVertical {
			s.AddConstraint(sketch.NewVerticalDistance(left[i], right[i], g.right[i].Y-g.left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(left[i], right[i], g.right[i].X-g.left[i].X))
		}
		// Seeded at the foot of the left fit point on the spine, not at the
		// rib's true 2-D midpoint and not at (fitX, 0).
		along := g.along(g.left[i])
		mid := s.CreatePoint(g.ax+along*math.Cos(g.angle), g.ay+along*math.Sin(g.angle))
		mid.SetName(fmt.Sprintf("ribMidpoint%d", i))
		mids[i] = mid
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != n-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
	}

	// Signed, so the chain runs outward, and started from the local origin —
	// without that first link the whole chain slides along the spine as a unit.
	proofkit.Step(t, "midpoint chain along the spine, starting at the local origin")
	prev, prevAlong := origin, 0.0
	for i := range n {
		along := g.along(g.left[i])
		if g.chainHorizontal {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mids[i], (along-prevAlong)*math.Cos(g.angle)))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mids[i], (along-prevAlong)*math.Sin(g.angle)))
		}
		prev, prevAlong = mids[i], along
	}

	if g.embedded {
		proofkit.Step(t, "embedded profile: flank starts inside the root circle, no stubs drawn")
	} else {
		// Two signed dimensions from the local origin, no others. "Root end on
		// the root circle" plus "origin on the line" is satisfied by the far
		// intersection too, and turns the stub into a line across the gear.
		proofkit.Step(t, "flank-to-root stubs, each placed by two signed dimensions")
		for _, side := range []struct {
			name  string
			start involute.Pt
			fit   *sketch.Point
		}{
			{"left", g.left[0], left[0]},
			{"right", g.right[0], right[0]},
		} {
			r := math.Hypot(side.start.X, side.start.Y)
			dx, dy := side.start.X*g.d.Root/r, side.start.Y*g.d.Root/r
			end := s.CreatePoint(g.ax+dx, g.ay+dy)
			end.SetName(side.name + "RootEnd")
			line := s.CreateLine(end, side.fit)
			line.SetName(side.name + "FlankToRoot")
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, end, dx),
				sketch.NewVerticalDistance(origin, end, dy),
			)
		}
	}

	confirmPose(t, s, g, origin, top, left[n-1], right[n-1])
}

// confirmPose checks that the scheme lands the tooth where it was drawn.
// Soundness says the answer is unique; this says it is the right one. The
// rotation is drawn AND confirmed ([SPUR-F-ROTATE-CONFIRM]), and the failure it
// guards against is a tooth that solves cleanly 180 degrees around.
func confirmPose(t testing.TB, s *sketch.Sketch, g gear, origin, top, leftTip, rightTip *sketch.Point) {
	proofkit.Step(t, "confirm the solved pose")
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve: %v", err)
	}
	dx, dy := top.X()-origin.X(), top.Y()-origin.Y()
	if r := math.Hypot(dx, dy); math.Abs(r-g.d.Tip) > 1e-6 {
		t.Errorf("tooth top at radius %.9f mm, want the tip radius %.9f", r, g.d.Tip)
	}
	if off := wrap(math.Atan2(dy, dx) - g.angle); math.Abs(off) > 1e-6 {
		t.Errorf("tooth top %.6f degrees off the requested %g", off*180/math.Pi, g.angleDeg)
	}
	// Positive cross product means the left flank tip really is counter-
	// clockwise of the right one, so the tooth is not mirrored.
	cross := (rightTip.X()-origin.X())*(leftTip.Y()-origin.Y()) -
		(rightTip.Y()-origin.Y())*(leftTip.X()-origin.X())
	if cross <= 0 {
		t.Errorf("flanks are swapped: left tip is clockwise of right (cross %.6f)", cross)
	}
}

// gear is one case's resolved geometry: the circle radii, the flank samples at
// their final angular position, and the two branch choices the drawing makes.
type gear struct {
	ax, ay      float64
	d           involute.Dimensions
	left, right []involute.Pt
	angle       float64
	angleDeg    float64

	// ribVertical and chainHorizontal are the axis choice of [SPUR-F-RIBS]
	// step 2: the rib takes the axis across the spine and the midpoint chain
	// the one along it. They move together.
	ribVertical     bool
	chainHorizontal bool

	embedded bool
	zeroStub bool
}

func newGear(p map[string]float64) gear {
	pressureAngle := p["pressureAngleDeg"] * math.Pi / 180
	d := involute.Derive(p["module"], p["teeth"], pressureAngle)
	if p["rootOntoBase"] != 0 {
		d.Root = d.Base
	}
	angle := p["toothAngleDeg"] * math.Pi / 180
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, p["teeth"], int(p["involuteSteps"]), angle)
	alongAxis := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	return gear{
		ax: p["anchorX"], ay: p["anchorY"],
		d: d, left: left, right: right,
		angle: angle, angleDeg: p["toothAngleDeg"],
		ribVertical: alongAxis, chainHorizontal: alongAxis,
		embedded: d.Embedded(),
		zeroStub: !d.Embedded() && d.Base == d.Root,
	}
}

// along projects a point onto the spine direction.
func (g gear) along(p involute.Pt) float64 {
	return p.X*math.Cos(g.angle) + p.Y*math.Sin(g.angle)
}

func construction[E sketch.Entity](e E, name string) E {
	e.SetConstruction(true)
	e.SetName(name)
	return e
}

func wrap(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
