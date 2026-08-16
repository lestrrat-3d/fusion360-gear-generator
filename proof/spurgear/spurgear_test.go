// Package spurgear_test proves the three sketches of the spur gear build.
//
// The spur generator puts three sketches in the Fusion timeline — Tools, Gear
// Profile and Bore Profile — and every one of them has to reach DOF 0 with a
// single admissible solution before any Fusion code is written. Everything else
// in the build is solid modelling (extrude, chamfer, pattern, combine, fillet,
// cut), which this engine does not model, so those steps stay prose in
// .tmp/spurgear.steps.md and are not proved here.
//
// One test function per provable step, each named for the step that names it:
//
//	S3  Tools sketch        -> stepToolsSketch
//	S5  Gear Profile sketch -> stepGearProfileSketch
//	S14 Bore Profile sketch -> stepBoreProfileSketch
//
// The tooth math is not repeated here: it lives in proof/involute, which the
// helical and herringbone proofs share.
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

// Parameter keys. The proof works in millimetres and degrees, which are the
// sketch engine's defaults; Fusion works in centimetres and radians internally,
// and that conversion is a generator concern, not a constraint-scheme one.
const (
	pModule        = "module"
	pToothNumber   = "toothNumber"
	pPressureAngle = "pressureAngle" // degrees
	pSteps         = "involuteSteps"
	pAngle         = "angle" // degrees, the draw(anchorPoint, angle=…) argument
	pBoreDiameter  = "boreDiameter"
)

// gear names one case. Params carries floats, so this keeps the tables readable
// and the defaults in one place.
type gear struct {
	name          string
	module        float64
	teeth         float64
	pressureAngle float64 // degrees; 20 when zero
	steps         float64 // involute samples; 15 when zero
	angle         float64 // degrees of rotation applied to the whole tooth
	bore          float64
}

func (g gear) Case() proofkit.Case {
	pa := g.pressureAngle
	if pa == 0 {
		pa = 20
	}
	steps := g.steps
	if steps == 0 {
		steps = 15
	}
	return proofkit.Case{
		Name: g.name,
		Params: map[string]float64{
			pModule:        g.module,
			pToothNumber:   g.teeth,
			pPressureAngle: pa,
			pSteps:         steps,
			pAngle:         g.angle,
			pBoreDiameter:  g.bore,
		},
	}
}

func cases(gears ...gear) []proofkit.Case {
	out := make([]proofkit.Case, 0, len(gears))
	for _, g := range gears {
		out = append(out, g.Case())
	}
	return out
}

// gearProfileCases sweeps the sizes the scheme has to hold across.
//
// The four required sizes come first. After them the table reaches for branches
// those cannot: the embedded profile (base circle inside the root circle, which
// drops the flank-to-root lines and takes the tooth loop from six curves to
// four), the two rotated-tooth branches — the rib/chain dimension axes swap when
// |cos angle| < |sin angle|, and a tooth at 90 degrees is exactly the case that
// fails without the swap — and a coarse involute sample count, which shortens
// the rib chain and moves the flank start.
var gearProfileCases = cases(
	gear{name: "m1_t12", module: 1, teeth: 12},
	gear{name: "m1_t17", module: 1, teeth: 17},
	gear{name: "m2_t20", module: 2, teeth: 20},
	gear{name: "m3_t15", module: 3, teeth: 15},

	// Embedded: 45 teeth at 20 degrees is past the 41.5 threshold, and 30 teeth
	// at 25 degrees is past the 26.7 one, so the same branch is reached from two
	// different directions.
	gear{name: "m1_t45_embedded", module: 1, teeth: 45},
	gear{name: "m2_t30_pa25_embedded", module: 2, teeth: 30, pressureAngle: 25},

	// Rotated. 30 degrees keeps |cos| >= |sin| (vertical rib, horizontal chain);
	// 90 and 120 degrees cross over to the swapped axes.
	gear{name: "m2_t20_rot30", module: 2, teeth: 20, angle: 30},
	gear{name: "m1_t17_rot90", module: 1, teeth: 17, angle: 90},
	gear{name: "m1_t45_rot120_embedded", module: 1, teeth: 45, angle: 120},

	// A negative angle, because the confirming angular dimension is signed and a
	// sign error would still solve at +30.
	gear{name: "m3_t15_rot_minus60", module: 3, teeth: 15, angle: -60},

	// Fewer samples: five ribs instead of fifteen.
	gear{name: "m1_t17_steps5", module: 1, teeth: 17, steps: 5},
)

func TestGearProfileSketch(t *testing.T) {
	proofkit.Run(t, gearProfileCases, stepGearProfileSketch)
}

func TestToolsSketch(t *testing.T) {
	proofkit.Run(t, cases(
		gear{name: "anchor_at_origin", module: 1, teeth: 17},
	), stepToolsSketch)
}

func TestBoreProfileSketch(t *testing.T) {
	proofkit.Run(t, cases(
		gear{name: "bore6", module: 1, teeth: 17, bore: 6},
		gear{name: "bore20_on_m3", module: 3, teeth: 15, bore: 20},
		gear{name: "bore0_no_sketch", module: 1, teeth: 17, bore: 0},
	), stepBoreProfileSketch)
}

// stepToolsSketch proves S3, the Tools sketch.
//
// The sketch draws no geometry of its own. Its whole content is the projection
// of the user's anchor point, which every later sketch re-projects from, so what
// there is to prove is that the projection alone leaves nothing free.
//
// The projection is modelled as a reference point: Fusion's projected geometry
// tracks its 3D source and the solver may not move it, which is what a reference
// point is. (Fusion's own projected point is NOT fixed — [PB-PROJECT-NOT-FIXED]
// — and the spec's answer is to make every consumer coincident to it, which is
// what the Gear Profile and Bore Profile sketches below do.)
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the Anchor Point into the Tools sketch")
	anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
	anchor.SetName("Tools/projected anchor")

	// The Extrusion End Plane the spec creates alongside this sketch is a
	// construction plane in 3D, with no sketch geometry and nothing to constrain.
	_ = p
}

// stepGearProfileSketch proves S5, the Gear Profile sketch — the whole of the
// spec's steps 3, 4 and 5, which are one Fusion timeline entry.
//
// The sketch is drawn relative to a movable local origin at (0,0) and only
// anchored at the very end, so the spec's claim that the anchoring "drags the
// entire tooth profile onto the anchor as a unit" is what the constraint order
// here reproduces.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	module := p[pModule]
	teeth := p[pToothNumber]
	steps := int(p[pSteps])
	angleDeg := p[pAngle]
	angle := angleDeg * math.Pi / 180
	d := involute.Derive(module, teeth, p[pPressureAngle]*math.Pi/180)

	// --- spec step 3: the four gear circles -------------------------------
	proofkit.Step(t, "drawCircles: root %.4f, tip %.4f, base %.4f, pitch %.4f (embedded=%v)",
		d.Root, d.Tip, d.Base, d.Pitch, d.Embedded())

	// The local origin is a fresh point at (0,0), never the sketch's own origin
	// point: [SPUR-F-LOCAL-ORIGIN] wants a point the solver can slide onto the
	// anchor, and s.Origin() is a point the solver never moves.
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin")

	// Every circle takes the local-origin point itself as its centre, so all four
	// share it and no centre coincidence is added ([PB-SHARE-XOR-COINCIDENT]).
	// Only the root circle is solid; the other three are construction and so bound
	// no profile, which is why the body profile is a disc and not an annulus.
	root := s.CreateCircle(origin, d.Root)
	tip := s.CreateCircle(origin, d.Tip)
	base := s.CreateCircle(origin, d.Base)
	pitch := s.CreateCircle(origin, d.Pitch)
	for _, c := range []*sketch.Circle{tip, base, pitch} {
		c.SetConstruction(true)
	}
	s.AddConstraint(
		sketch.NewDiameter(root, 2*d.Root),
		sketch.NewDiameter(tip, 2*d.Tip),
		sketch.NewDiameter(base, 2*d.Base),
		sketch.NewDiameter(pitch, 2*d.Pitch),
	)
	// The along-path label each circle carries is sketch text. It constrains
	// nothing and the engine has no analog, so it is not modelled; a wrong label
	// cannot make the sketch under-constrained.

	// --- spec step 4.1-4.5: the two involute flanks ------------------------
	proofkit.Step(t, "drawTooth: %d involute samples per flank, drawn at %g degrees", steps, angleDeg)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	if len(left) < 2 {
		proofkit.Unmodelled(t, "%d involute samples survive, too few for a fitted spline", len(left))
	}
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(left[i].X, left[i].Y)
		lp[i].SetName(fmt.Sprintf("left flank fit %d", i))
		rp[i] = s.CreatePoint(right[i].X, right[i].Y)
		rp[i].SetName(fmt.Sprintf("right flank fit %d", i))
	}
	if _, err := s.CreateFitSpline(lp...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rp...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	last := len(left) - 1

	// --- spec step 4.6: the tooth-top arc ---------------------------------
	// [SPUR-F-TOOTHTOP-ARC]: the arc shares the local origin as its centre and the
	// two flank end points as its ends, and carries no diameter dimension. The
	// shared centre is what says the arc bulges outward; a free centre plus a
	// diameter would reach DOF 0 with an inward-bulging answer available too.
	proofkit.Step(t, "tooth-top point on the tip circle, then the arc centred on the local origin")
	topX, topY := involute.Rotate(d.Tip, 0, angle)
	top := s.CreatePoint(topX, topY)
	top.SetName("tooth top")
	s.AddConstraint(sketch.NewPointOnCircle(top, tip))

	// Counter-clockwise from the right flank's end to the left flank's, so the arc
	// sweeps across the tip rather than back through the tooth. Creating it also
	// adds the engine's radius-consistency constraint, which is the same equation
	// the last rib's omitted perpendicular would have carried — which is exactly
	// why omitting that perpendicular is required rather than optional.
	s.CreateArc(origin, rp[last], lp[last])

	// --- spec step 4.7: the spine, the +X reference and the angular pin ----
	// [SPUR-F-SPINE]. The reference line's far end is pinned with two SIGNED
	// dimensions from the local origin rather than a coincidence to the tip
	// circle: a point on a circle has two answers, and the one this needs sits
	// where the numbers go unstable.
	proofkit.Step(t, "spine, +X reference line, and the angular dimension that pins the tooth to %g degrees", angleDeg)
	spine := s.CreateLine(origin, top)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(d.Tip, 0)
	refEnd.SetName("+X reference end")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, d.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	// Reference first, spine second — the argument order decides the sign, and the
	// dimension exists for every angle including 0, because it is the only thing
	// that says which way the spine points. A plain horizontal constraint would
	// leave the tooth free to settle 180 degrees around.
	spineAngle := sketch.NewAngle(reference, spine, angleDeg)
	s.AddConstraint(spineAngle)
	s.SetConstraintName(spineAngle, "spine angular pin")

	// --- spec step 4.8: the ribs ------------------------------------------
	// [SPUR-F-RIBS], in the order the anchor fixes: rib, signed dimension,
	// midpoint seeded on the spine, coincident to the spine, midpoint of the rib,
	// perpendicular to the spine — the last skipped on the final rib, whose
	// perpendicular the tooth-top arc already implies.
	//
	// The rib takes the axis ACROSS the spine and the midpoint chain the one
	// along it. At angle 0 that is vertical for the rib and horizontal for the
	// chain; past 45 degrees the two swap, and without the swap a tooth at 90
	// degrees fails.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	proofkit.Step(t, "%d ribs, rib dimension %s, chain dimension %s",
		len(left), axisName(acrossIsVertical), axisName(!acrossIsVertical))

	prev := origin
	prevX, prevY := 0.0, 0.0
	for i := range left {
		rib := s.CreateLine(lp[i], rp[i])
		rib.SetConstruction(true)

		// Signed, never an aligned length: a length alone is satisfied just as
		// well with the left and right flanks swapped, and the tooth comes out
		// mirrored.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(lp[i], rp[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(lp[i], rp[i], right[i].X-left[i].X))
		}

		// The midpoint is seeded at the foot of the left fit point on the spine,
		// not at the rib's true 2-D midpoint and not at (fitX, 0) for a rotated
		// tooth.
		tPar := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := tPar*math.Cos(angle), tPar*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		mid.SetName(fmt.Sprintf("rib %d midpoint", i))

		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != last {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}

		// The chain runs outward from the local origin, signed so it cannot run
		// the other way. Without the origin-to-first-rib dimension the whole chain
		// slides along the spine as a unit and the sketch never closes.
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, midX-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, midY-prevY))
		}
		prev, prevX, prevY = mid, midX, midY
	}

	// --- spec step 4.9: close the tooth at the root ------------------------
	// [SPUR-F-FLANK-ROOT]. The root end is placed with exactly two signed
	// dimensions from the local origin. "Root end on the root circle" plus "origin
	// on the line" would be satisfied by the far intersection too, and the stub
	// would become a line straight across the gear.
	if d.Embedded() {
		proofkit.Step(t, "embedded profile: base radius %.4f is inside root radius %.4f, no flank-to-root lines",
			d.Base, d.Root)
	} else {
		proofkit.Step(t, "flank-to-root stubs: base radius %.4f is outside root radius %.4f", d.Base, d.Root)
		for _, f := range []struct {
			name string
			at   involute.Pt
			end  *sketch.Point
		}{
			{"left", left[0], lp[0]},
			{"right", right[0], rp[0]},
		} {
			// Radial: the stub runs along the line from the gear centre to the
			// flank's first fit point.
			scale := d.Root / math.Hypot(f.at.X, f.at.Y)
			rx, ry := f.at.X*scale, f.at.Y*scale
			rootEnd := s.CreatePoint(rx, ry)
			rootEnd.SetName(f.name + " flank root end")
			stub := s.CreateLine(rootEnd, f.end) // solid: it is part of the tooth loop
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, rx),
				sketch.NewVerticalDistance(origin, rootEnd, ry),
			)
			checkRootEndConstraintRecipe(t, s, origin, rootEnd, stub)
		}
	}

	// --- spec step 5: anchor the sketch -----------------------------------
	// The re-projection of the Tools-sketch anchor, and the one coincidence that
	// slides the whole drawing onto it. This is the last constraint added, inside
	// draw(), exactly as the spec puts it.
	proofkit.Step(t, "anchor: re-project the Tools anchor and make the local origin coincident with it")
	anchor := s.CreateReferencePoint(0, 0, "Tools/anchorPoint")
	anchor.SetName("projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	// The confirming angular dimension is set last, after the whole constraint
	// network exists ([SPUR-F-ROTATE-CONFIRM]). Here the geometry was already
	// drawn rotated and the dimension already carries the angle, so re-setting it
	// is the same no-op Fusion performs at angle 0 — but the ordering is the
	// point, so it is written out.
	if angleDeg != 0 {
		spineAngle.Set(angleDeg)
	}

	// --- what the extrude steps will look for ------------------------------
	// S7 and S9 find their profiles by curve count, so which loops this sketch
	// yields is part of its contract and not just of the Python. Fusion derives
	// both loops by splitting the solid root circle where the tooth meets it; the
	// proof draws no boundary arc by hand and lets profile detection do the same
	// split.
	checkLoops(t, s, d.Embedded())
}

func axisName(vertical bool) string {
	if vertical {
		return "vertical"
	}
	return "horizontal"
}

func checkRootEndConstraintRecipe(t testing.TB, s *sketch.Sketch, origin, rootEnd *sketch.Point, stub *sketch.Line) {
	t.Helper()

	var horizontal, vertical int
	var rejected []string
	for _, c := range s.Constraints() {
		kind := sketch.ConstraintKind(c)
		pts, ents := sketch.ConstraintRefs(c)
		if kind == "hdistance" && len(pts) == 2 && pts[0] == origin && pts[1] == rootEnd {
			horizontal++
			continue
		}
		if kind == "vdistance" && len(pts) == 2 && pts[0] == origin && pts[1] == rootEnd {
			vertical++
			continue
		}
		if kind == "point_on_circle" && len(pts) == 1 && pts[0] == rootEnd {
			rejected = append(rejected, "root end on root circle")
			continue
		}
		if kind == "point_on_line" && len(pts) == 1 && pts[0] == origin && len(ents) == 1 && ents[0] == stub {
			rejected = append(rejected, "local origin on flank-to-root line")
		}
	}
	if horizontal != 1 || vertical != 1 || len(rejected) > 0 {
		t.Fatalf("root endpoint constraints must be exactly signed horizontal+vertical dimensions from the local origin; got horizontal=%d vertical=%d rejected=%v",
			horizontal, vertical, rejected)
	}
}

// stepBoreProfileSketch proves S14, the Bore Profile sketch.
//
// The sketch is drawn by instantiating the tooth generator on it and calling
// drawBore, so it inherits the constructor's local-origin point at (0,0) even
// though nothing draws from it. That stray point is the whole difficulty: left
// alone it is free in two directions and the sketch never fully constrains.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	bore := p[pBoreDiameter]
	if bore <= 0 {
		proofkit.Unmodelled(t, "bore diameter %g: buildBore returns before creating the Bore Profile sketch, so there is no sketch to prove", bore)
	}

	proofkit.Step(t, "tooth-generator constructor adds its local origin at (0,0)")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin (unused by the bore)")

	proofkit.Step(t, "drawBore: project the Tools anchor and draw the bore circle on it, diameter %g", bore)
	anchor := s.CreateReferencePoint(0, 0, "Tools/anchorPoint")
	anchor.SetName("projected anchor")
	circle := s.CreateCircle(anchor, bore/2)
	s.AddConstraint(sketch.NewDiameter(circle, bore))

	// Ground the stray local origin on the projected anchor, exactly as the Gear
	// Profile sketch does — not on the sketch's own origin point, which would pin
	// it to the plane instead of to the gear.
	proofkit.Step(t, "ground the local origin on the projected anchor")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
}

// checkLoops asserts the two closed regions the extrude steps consume: the
// tooth cross-section, and the disc inside the root circle.
//
// Both come out of splitting the solid root circle where the tooth meets it, so
// if that split does not happen the tooth loop never closes, and it says so here
// rather than in Fusion.
//
// The two engines agree on the loops but not on how they report them. Fusion
// splits the root circle into two SketchArcs at the points where the tooth meets
// it, so find_profile_by_curve_counts sees 2 arcs on the body loop and 2 arcs on
// the tooth loop (with the tooth-top arc). This engine keeps the circle as one
// entity and reports boundary FRAGMENTS of it: the body loop's two fragments
// come back merged into one whole-circle edge, and the tooth loop's single
// fragment comes back split in two by the circle's own seam at +X, which the
// tooth straddles. Counting raw edges here would therefore assert a fact about
// the engine rather than about the gear, so the check is by which entities bound
// each loop — the same statement about the geometry, in the form this engine can
// make it.
func checkLoops(t testing.TB, s *sketch.Sketch, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before profile detection: %v", err)
	}

	wantTooth := curveCounts{nurbs: 2, arcs: 1, circles: 1, lines: 2}
	if embedded {
		// No flank-to-root stubs: the flanks themselves cross the root circle, and
		// the loop is the two spline fragments, the tooth-top arc and the root arc.
		wantTooth.lines = 0
	}
	wantBody := curveCounts{circles: 1}

	var tooth, body int
	for _, prof := range s.Profiles() {
		switch countEntities(prof) {
		case wantTooth:
			tooth++
		case wantBody:
			body++
		}
	}
	if tooth != 1 || body != 1 {
		t.Errorf("want exactly one tooth loop %+v and one body loop %+v, found %d and %d — %s",
			wantTooth, wantBody, tooth, body, describeProfiles(s))
	}
}

type curveCounts struct{ nurbs, arcs, lines, circles int }

// countEntities counts the DISTINCT entities on a profile's outer boundary, so a
// curve a crossing split into several fragments counts once.
func countEntities(prof *sketch.Profile) curveCounts {
	var n curveCounts
	for _, e := range prof.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			n.nurbs++
		case *sketch.Arc:
			n.arcs++
		case *sketch.Line:
			n.lines++
		case *sketch.Circle:
			n.circles++
		}
	}
	return n
}

func describeProfiles(s *sketch.Sketch) string {
	profs := s.Profiles()
	out := fmt.Sprintf("%d region(s):", len(profs))
	for _, prof := range profs {
		out += fmt.Sprintf(" [%+v area=%.3f valid=%v]", countEntities(prof), prof.Area, prof.Valid)
	}
	return out
}
