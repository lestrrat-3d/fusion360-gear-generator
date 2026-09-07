// Package helicalgear_test proves the helical gear's two deltas over the spur
// build: the Twisted Gear Profile sketch, which is the spur involute tooth
// drawn at angle = HelixAngle, and the loft that joins the bottom (angle 0)
// tooth loop to that twisted top loop.
//
// This file holds the tooth recipe both steps draw from. The recipe is spur's
// (spec/spurgear/fusion.md [SPUR-F-SPINE], [SPUR-F-ROTATE-CONFIRM],
// [SPUR-F-TOOTHTOP-ARC], [SPUR-F-RIBS], [SPUR-F-FLANK-ROOT]) run at a non-zero
// angle, which is exactly what helical's second sketch does through
// SpurGearInvoluteToothDesignGenerator.draw(ctx.anchorPoint, angle=HelixAngle).
// The involute point math is imported from proof/involute, not derived again.
//
// Coordinates are millimetres in the sketch engine; the Fusion module works in
// centimetres. Only the unit of the number changes, never the recipe.
package helicalgear_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
)

// Parameter keys shared by every case table. Angles are radians and lengths
// are millimetres, so a case reads like the dialog with the units resolved.
const (
	keyModule        = "module"        // gear module, mm
	keyToothNumber   = "toothNumber"   // number of teeth
	keyPressureAngle = "pressureAngle" // rad
	keyHelixAngle    = "helixAngle"    // rad, signed: negative is a left-hand helix
	keyInvoluteSteps = "involuteSteps" // fit points per flank
	keyThickness     = "thickness"     // mm, the helix plane offset
)

// Exact spur parameter names the twisted sketch reads through the tooth
// generator. Written here so a case table and a failure message can quote them.
const (
	defaultPressureAngle = 20.0 * math.Pi / 180
	defaultHelixAngle    = 14.5 * math.Pi / 180
	defaultInvoluteSteps = 15
	defaultThickness     = 10.0
)

// caseParams builds one case's parameter map. steps is the involute sample
// count; helixDeg and pressureDeg are in degrees for readability, converted here.
func caseParams(module, toothNumber, pressureDeg, helixDeg float64, steps int, thickness float64) map[string]float64 {
	return map[string]float64{
		keyModule:        module,
		keyToothNumber:   toothNumber,
		keyPressureAngle: pressureDeg * math.Pi / 180,
		keyHelixAngle:    helixDeg * math.Pi / 180,
		keyInvoluteSteps: float64(steps),
		keyThickness:     thickness,
	}
}

// toothProfile is every handle the tooth recipe leaves behind, so an assertion
// can name the entity it measures rather than search for it.
type toothProfile struct {
	dims involute.Dimensions

	anchor *sketch.Point // the projected ctx.anchorPoint (reference geometry)
	origin *sketch.Point // the generator's movable local origin, self.anchorPoint

	root, tip, base, pitch *sketch.Circle

	left, right []*sketch.Point // flank fit points, base circle first
	leftFlank   sketch.Entity   // *sketch.FitSpline, or nil when chorded
	rightFlank  sketch.Entity
	leftChords  []*sketch.Line // the chorded substitute, or nil when splined
	rightChords []*sketch.Line
	topChord    *sketch.Line // loft-ready only: the chord under the tooth-top arc
	rootChord   *sketch.Line // loft-ready only: the chord under the root piece

	toothTop  *sketch.Point
	topArc    *sketch.Arc
	spine     *sketch.Line
	reference *sketch.Line
	angular   *sketch.Angle

	ribs      []*sketch.Line
	midpoints []*sketch.Point

	embedded             bool
	leftRootEnd          *sketch.Point // nil when embedded
	rightRootEnd         *sketch.Point
	leftFlankRoot        *sketch.Line
	rightFlankRoot       *sketch.Line
	angularSetAsLastStep bool
	acrossIsVerticalAxis bool
}

// drawToothProfile reproduces SpurGearInvoluteToothDesignGenerator.draw(anchor,
// angle) in the sketch engine: drawCircles, drawTooth(angle), the step-5
// anchoring, and then — as the very last action — the angular dimension's value
// set to angle when angle != 0 ([SPUR-F-ROTATE-CONFIRM]).
//
// frame turns the whole recipe rigidly in the sketch plane: the +X reference
// line is drawn along the frame direction, and every seed is rotated by
// frame + angle. Fusion's frame is 0. The loft proof draws in a frame turned
// half a turn (see loftFrame) because of how the engine parametrises a circle;
// nothing in the recipe changes, and the angular dimension still measures
// angle from the reference line.
//
// loftReady substitutes straight chords for every curved edge of the tooth
// loop, so that the loop reaches decad's loft, which pairs LineSeg, ArcSeg and
// CircleSeg only and chords a circular pair itself with a volume bound its
// verification then refuses (measured: 0.053 mm³ against a 0.032 mm³ allowance
// at M=1, N=17, T=10). Three substitutions, all made on the same constrained
// points, so no constraint, point or degree of freedom changes:
//
//   - each involute FitSpline becomes a polyline through its own fit points
//     (a free-form entity anywhere in a sketch also withdraws the exact trim,
//     BoundaryEdge.TExact, that a recorded fragment needs);
//   - the tooth-top arc is kept as construction geometry and a solid chord is
//     drawn between the two flank tips it joins;
//   - the root circle is kept as construction geometry and a solid chord is
//     drawn between the two root ends the flank-to-root lines stand on.
//
// What it costs: the lofted walls are faceted involutes, the tooth cap and the
// root are flat, and the disc inside the root circle is no longer a region of
// this sketch. Measured at M=1, N=17, T=10, 14.5°, the chorded body holds 0.6%
// less volume than the one with the two arcs kept. The sketch step draws the
// real splines and the solid root circle and proves the counts on them.
func drawToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64, frame, angle float64, loftReady bool) *toothProfile {
	t.Helper()
	chordFlanks := loftReady
	total := frame + angle // where the spine ends up in the sketch frame
	module := p[keyModule]
	toothNumber := p[keyToothNumber]
	pressureAngle := p[keyPressureAngle]
	steps := int(p[keyInvoluteSteps])
	if steps < 2 {
		t.Fatalf("involuteSteps must be at least 2 to place a flank, got %d", steps)
	}
	tp := &toothProfile{dims: involute.Derive(module, toothNumber, pressureAngle)}
	d := tp.dims

	// [SPUR-F-LOCAL-ORIGIN]: a fresh movable point at (0,0), NOT the sketch's
	// own origin. Everything below is drawn relative to it.
	proofkit.Step(t, "local origin")
	tp.origin = s.CreatePoint(0, 0)
	tp.origin.SetName("localOrigin")

	// drawCircles: root solid, the other three construction, every one centred
	// on the shared local origin with a driving diameter dimension
	// ([PB-CIRCLE-CENTER] realised by sharing the point, [PB-DRIVING-DIM]).
	proofkit.Step(t, "circles")
	circle := func(name string, r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(tp.origin, r)
		c.SetName(name)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	tp.root = circle("Root Circle", d.Root, false)
	tp.tip = circle("Tip Circle", d.Tip, true)
	tp.base = circle("Base Circle", d.Base, true)
	tp.pitch = circle("Pitch Circle", d.Pitch, true)

	// drawTooth(angle), step 4: sample, mirror, centre on +X, then rotate the
	// whole tooth by angle in the point math ([SPUR-F-ROTATE-CONFIRM] draw half).
	proofkit.Step(t, "flanks at angle %.3f rad (frame %.3f rad)", angle, frame)
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, toothNumber, steps, total)
	if len(left) != steps {
		t.Fatalf("involute.Flanks returned %d samples for %d steps; every sample from the base circle out is expected", len(left), steps)
	}
	tp.left = make([]*sketch.Point, steps)
	tp.right = make([]*sketch.Point, steps)
	for i := range steps {
		tp.left[i] = s.CreatePoint(left[i].X, left[i].Y)
		tp.right[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if chordFlanks {
		for i := 0; i+1 < steps; i++ {
			tp.leftChords = append(tp.leftChords, s.CreateLine(tp.left[i], tp.left[i+1]))
			tp.rightChords = append(tp.rightChords, s.CreateLine(tp.right[i], tp.right[i+1]))
		}
	} else {
		lf, err := s.CreateFitSpline(tp.left...)
		if err != nil {
			t.Fatalf("left flank spline: %v", err)
		}
		rf, err := s.CreateFitSpline(tp.right...)
		if err != nil {
			t.Fatalf("right flank spline: %v", err)
		}
		lf.SetName("leftFlank")
		rf.SetName("rightFlank")
		tp.leftFlank, tp.rightFlank = lf, rf
	}

	// Step 6, [SPUR-F-TOOTHTOP-ARC]: the tooth-top point at the tip radius,
	// rotated by angle, on the tip circle; the arc from the RIGHT flank end to
	// the LEFT flank end about the local origin. Fusion copies the centre and
	// needs addCoincident(arc.centerSketchPoint, localOrigin); the engine shares
	// the point, which is the same relation. No diameter dimension.
	proofkit.Step(t, "tooth-top arc")
	ttx, tty := involute.Rotate(d.Tip, 0, total)
	tp.toothTop = s.CreatePoint(ttx, tty)
	tp.toothTop.SetName("toothTop")
	s.AddConstraint(sketch.NewPointOnCircle(tp.toothTop, tp.tip))
	tp.topArc = s.CreateArc(tp.origin, tp.right[steps-1], tp.left[steps-1])
	tp.topArc.SetName("toothTopArc")

	// Step 7, [SPUR-F-SPINE]: the spine shares both endpoints; the +X reference
	// line's far end is pinned by two axis dimensions from the local origin
	// (seeded on +X, magnitudes tipR and 0 — [PB-DIM-VALUE-SEMANTICS]); the
	// angular dimension runs from the reference to the spine and is created at
	// the seeded angle for EVERY angle, including 0.
	proofkit.Step(t, "spine and +X reference")
	tp.spine = s.CreateLine(tp.origin, tp.toothTop)
	tp.spine.SetName("spine")
	tp.spine.SetConstruction(true)
	refX, refY := involute.Rotate(d.Tip, 0, frame) // (tipR, 0) in Fusion's frame
	refEnd := s.CreatePoint(refX, refY)
	refEnd.SetName("referenceEnd")
	s.AddConstraint(
		sketch.NewHorizontalDistance(tp.origin, refEnd, refX),
		sketch.NewVerticalDistance(tp.origin, refEnd, refY),
	)
	tp.reference = s.CreateLine(tp.origin, refEnd)
	tp.reference.SetName("xReference")
	tp.reference.SetConstruction(true)
	// NewAngle is signed, counter-clockwise from l1 to l2, in degrees. Fusion's
	// addAngularDimension(reference, spine, textPoint) measures the seeded
	// geometry, which sits at angle already; the text point on the bisector
	// selects that wedge ([PB-ANGULAR-DIM]).
	tp.angular = sketch.NewAngle(tp.reference, tp.spine, angle*180/math.Pi)
	s.AddConstraint(tp.angular)

	// Step 8, [SPUR-F-RIBS]: one rib per fit-point index, endpoints included,
	// in the exact six-step order. The rib takes the axis ACROSS the spine and
	// the midpoint chain the axis ALONG it; which axis is which swaps when
	// |sin| > |cos|. The last rib carries no perpendicular.
	proofkit.Step(t, "ribs")
	tp.acrossIsVerticalAxis = math.Abs(math.Cos(total)) >= math.Abs(math.Sin(total))
	prevMid := tp.origin
	prevX, prevY := 0.0, 0.0
	for i := range steps {
		rib := s.CreateLine(tp.left[i], tp.right[i]) // 1. share both fit points
		rib.SetConstruction(true)
		if tp.acrossIsVerticalAxis { // 2. axis dimension across the spine
			s.AddConstraint(sketch.NewVerticalDistance(tp.left[i], tp.right[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(tp.left[i], tp.right[i], right[i].X-left[i].X))
		}
		// 3. midpoint seeded at the foot of the left fit point on the spine
		foot := left[i].X*math.Cos(total) + left[i].Y*math.Sin(total)
		mx, my := foot*math.Cos(total), foot*math.Sin(total)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, tp.spine)) // 4. onto the spine first
		s.AddConstraint(sketch.NewMidpoint(mid, rib))         // 5. then the rib's midpoint
		if i != steps-1 {
			s.AddConstraint(sketch.NewPerpendicular(tp.spine, rib)) // 6. skipped for the last rib
		}
		// chain: previous midpoint (local origin for the first rib) to this one
		if tp.acrossIsVerticalAxis {
			s.AddConstraint(sketch.NewHorizontalDistance(prevMid, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevMid, mid, my-prevY))
		}
		tp.ribs = append(tp.ribs, rib)
		tp.midpoints = append(tp.midpoints, mid)
		prevMid, prevX, prevY = mid, mx, my
	}

	// Step 9, [SPUR-F-FLANK-ROOT]: strict `<` embedded test on the drawn first
	// fit point; when not embedded, a radial line from the root circle to each
	// flank start, its root end placed by exactly two axis dimensions from the
	// local origin, seeded at its exact position.
	proofkit.Step(t, "flank-to-root lines")
	firstRadius := math.Hypot(left[0].X, left[0].Y)
	tp.embedded = firstRadius < d.Root
	if tp.embedded != d.Embedded() {
		t.Fatalf("embedded by the drawn point (%v, first radius %.6f vs root %.6f) disagrees with involute.Dimensions.Embedded (%v)",
			tp.embedded, firstRadius, d.Root, d.Embedded())
	}
	if !tp.embedded {
		rootEnd := func(flankStart *sketch.Point, seed involute.Pt, name string) (*sketch.Point, *sketch.Line) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := d.Root*seed.X/n, d.Root*seed.Y/n
			re := s.CreatePoint(rx, ry)
			re.SetName(name)
			line := s.CreateLine(re, flankStart)
			line.SetName(name + "FlankRoot")
			s.AddConstraint(
				sketch.NewHorizontalDistance(tp.origin, re, rx),
				sketch.NewVerticalDistance(tp.origin, re, ry),
			)
			return re, line
		}
		tp.leftRootEnd, tp.leftFlankRoot = rootEnd(tp.left[0], left[0], "leftRootEnd")
		tp.rightRootEnd, tp.rightFlankRoot = rootEnd(tp.right[0], right[0], "rightRootEnd")
	}

	// Step 5 anchoring, inside draw(): project the Tools-sketch anchor in and
	// make the local origin coincident with it ([SPUR-F-ANCHOR-CHAIN]). The
	// projection is reference geometry, so it is a reference point here rather
	// than a fixed coordinate.
	proofkit.Step(t, "anchor the local origin")
	tp.anchor = s.CreateReferencePoint(0, 0, "ctx.anchorPoint")
	tp.anchor.SetName("projectedAnchor")
	s.AddConstraint(sketch.NewCoincident(tp.origin, tp.anchor))

	// Very last action: confirm the rotation ([SPUR-F-ROTATE-CONFIRM] confirm
	// half). At angle 0 there is nothing to set.
	if angle != 0 {
		tp.angular.Set(angle * 180 / math.Pi)
		tp.angularSetAsLastStep = true
	}

	// The loft-ready chords (see the function comment). They share existing
	// points and carry no constraint, so they are added after the recipe is
	// complete and change nothing about its verdict.
	if loftReady {
		tp.topArc.SetConstruction(true)
		tp.root.SetConstruction(true)
		tp.topChord = s.CreateLine(tp.right[steps-1], tp.left[steps-1])
		tp.topChord.SetName("toothTopChord")
		if !tp.embedded {
			tp.rootChord = s.CreateLine(tp.leftRootEnd, tp.rightRootEnd)
			tp.rootChord.SetName("rootChord")
		}
	}
	return tp
}

// loftLoopCounts is the tooth loop's edge composition in loft-ready form: the
// two chorded flanks, the two flank-to-root lines, the top chord and the root
// chord, all lines.
func loftLoopCounts(steps int) edgeCounts {
	return edgeCounts{lines: 2*(steps-1) + 4}
}

// edgeCounts is one profile's outer boundary sorted by the curve type Fusion's
// find_profile_by_curve_counts matches on. A fragment of the root circle is an
// Arc3DCurveType in Fusion and a Partial *sketch.Circle edge here, so it is
// counted with the arcs.
type edgeCounts struct {
	nurbs, arcs, lines int
	circleFragments    int // how many of the arcs are pieces of a full circle
}

// The engine parametrises a circle from its local +X, and a piece of the
// circle that runs across that seam comes back as two consecutive edges of
// the same entity, one either side of it. Fusion has no seam, so that piece
// is one arc there; consecutive edges on one circle are counted once here,
// wrapping around the end of the loop.
func countEdges(outer []sketch.BoundaryEdge) edgeCounts {
	var c edgeCounts
	n := len(outer)
	for i, e := range outer {
		switch e.Entity.(type) {
		case *sketch.FitSpline:
			c.nurbs++
		case *sketch.Line:
			c.lines++
		case *sketch.Arc:
			c.arcs++
		case *sketch.Circle:
			if n > 1 && outer[(i+n-1)%n].Entity == e.Entity {
				continue // the seam split: the same piece continues
			}
			c.arcs++
			c.circleFragments++
		}
	}
	return c
}

// findProfile returns the one valid profile whose outer loop has exactly the
// given curve counts, or nil when none does. Two matches fail the test: the
// framework helper raises on ambiguity rather than picking one.
func findProfile(t testing.TB, profiles []*sketch.Profile, want edgeCounts) *sketch.Profile {
	t.Helper()
	var found *sketch.Profile
	for _, p := range profiles {
		if !p.Valid {
			continue
		}
		got := countEdges(p.Outer)
		if got.nurbs == want.nurbs && got.arcs == want.arcs && got.lines == want.lines {
			if found != nil {
				t.Fatalf("two profiles match nurbs=%d arcs=%d lines=%d; find_profile_by_curve_counts would raise", want.nurbs, want.arcs, want.lines)
			}
			found = p
		}
	}
	return found
}

// describeProfiles renders every detected region for a failure message.
func describeProfiles(profiles []*sketch.Profile) string {
	var b strings.Builder
	for i, p := range profiles {
		c := countEdges(p.Outer)
		fmt.Fprintf(&b, "\n  profile %d: valid=%v nurbs=%d arcs=%d (circle fragments %d) lines=%d holes=%d area=%.4f",
			i, p.Valid, c.nurbs, c.arcs, c.circleFragments, c.lines, len(p.Holes), p.Area)
	}
	return b.String()
}
