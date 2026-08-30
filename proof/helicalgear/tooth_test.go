// Package helicalgear_test proves the helical gear's compiled step list.
//
// Helical is a thin specialization of spur: it inherits the whole spur build
// and changes three things. It draws a SECOND "Twisted Gear Profile" sketch —
// the spur tooth generator run at angle = HelixAngle — on a construction plane
// offset from the target plane by the full Thickness, and lofts the bottom
// (untwisted) profile to that top one instead of extruding. It also raises the
// front-face edge count its inherited chamfer step matches on.
//
// This file holds the geometry the step proofs share. The involute math itself
// is proof/involute, imported rather than derived again, because spur, helical
// and herringbone draw the same tooth.
//
// # What the harnesses could and could not build, and what was substituted
//
// Three substitutions are made here. Each one is named at the place it is made,
// and each one costs something the reader should know about.
//
//  1. THE LOFT'S FLANKS ARE CHORDED. decad's Loft pairs recorded segments and
//     admits only two LineSegs, two ArcSegs or two CircleSegs. Handing it the
//     real tooth — whose flanks are fitted splines — refuses with
//     "loop 0 segment 3 of the first profile and segment 3 of the second are not
//     the same admitted segment type; this evaluator pairs two LineSegs, two
//     ArcSegs or two CircleSegs only". So the loft sections here are polylines
//     ([polyToothSection]). Cost: the lofted walls are ruled between chords of
//     the involute rather than between the involute curves themselves, so the
//     loft proof pins the twist, the height, the section correspondence and the
//     solid topology, and does NOT pin the flank surface. The flank shape is
//     pinned instead by the sketch step, which draws the real fitted spline.
//
//  2. THE LOFT'S TIP AND ROOT ARCS ARE CHORDED TOO. decad does loft an ArcSeg
//     pair, by chording its walls internally, but the resulting body's volume
//     bound then exceeds the default relative tolerance on a module-1 tooth
//     (measured: volume 31.73 mm^3, bound 0.0526 mm^3 against a required
//     0.0317), which the RunSolid gate reports as measurement_beyond_tolerance
//     on the volume reading and refuses. It tolerates that diagnostic only on
//     an area or centroid reading. Chording the two arcs in the sketch instead
//     makes every wall an exact planar quad and the volume bound falls to ~1e-16
//     relative. Cost: the lofted body's tip and root are polygonal, so no radius
//     may be read off it.
//
//  3. THE CAP-FACE EDGE COUNT IS PROVEN ON AN EXTRUDE, NOT ON THE LOFT. The
//     chamfer step matches the tooth's front face by its edge count, which only
//     survives if each flank stays ONE edge — exactly what substitution 1 gives
//     up. decad's Extrude does accept the fitted-spline flank, so
//     [toothSection] builds the real 6-curve tooth and the chamfer step extrudes
//     it. Cost: the face counted is the cap of an untwisted extruded tooth
//     rather than of the lofted one. The count comes from the profile's own
//     curve count, which the sketch step proves is the same 6 either way.
//
// A fourth limit is not a substitution but a ceiling. decad refuses the real
// tooth profile past 8 involute samples with "free-form exact integration needs
// more than the fixed work budget of 1048576", so the chamfer step's cases run
// at 4 to 8 samples while the shipped InvoluteSteps is 15. The cap-face edge
// count does not depend on the sample count — 4, 5, 6 and 8 all give 6 edges —
// so the ceiling costs coverage of the production sample count, not of the fact
// being proven.
package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
)

// helicalChamferWantEdges is the value spec/helicalgear/instructions.md pins for
// chamferWantEdges() — spur's base returns 6, helical returns 4. The chamfer
// step measures what the tooth's front face really carries and holds it against
// this.
const helicalChamferWantEdges = 4

// dims reads the four circle radii and the derived facts one case needs.
func dims(p map[string]float64) (involute.Dimensions, float64, int, float64) {
	d := involute.Derive(p["module"], p["toothNumber"], p["pressureAngle"])
	return d, p["toothNumber"], int(p["involuteSteps"]), p["helixAngle"]
}

// curveCounts is one profile loop's curve mix, in the terms
// find_profile_by_curve_counts matches on.
type curveCounts struct{ nurbs, arcs, lines int }

// countCurves reads a detected region's boundary in the terms Fusion's profile
// search uses. A circle the arrangement split counts as an arc, which is what
// Fusion reports for the same split.
//
// The count is over the region's distinct boundary ENTITIES, not over its
// boundary edges, and the difference is not cosmetic. The sketch engine
// parameterizes a circle from +X and never lets one boundary edge wrap that
// seam, so a root arc lying under a tooth centred on +X — every tooth at helix
// angle 0 — arrives as two edges of one circle. Fusion has no such seam and
// reports one curve. Counting entities is what makes the two agree.
func countCurves(p *sketch.Profile) curveCounts {
	var c curveCounts
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.FitSpline, *sketch.Spline:
			c.nurbs++
		case *sketch.Arc, *sketch.Circle:
			c.arcs++
		case *sketch.Line:
			c.lines++
		}
	}
	return c
}

// smallestProfile is the tooth region: the gear profile sketch closes exactly
// two regions and the tooth is the smaller of them by a wide margin.
func smallestProfile(ps []*sketch.Profile) *sketch.Profile {
	best := ps[0]
	for _, p := range ps[1:] {
		if p.Area < best.Area {
			best = p
		}
	}
	return best
}

// toothSection draws the tooth outline as its six real curves — two fitted
// spline flanks, the tip arc, the root arc and the two flank-to-root lines —
// with every point fixed at its computed position.
//
// The root arc is drawn directly rather than derived by splitting a solid root
// circle, which is how Fusion gets it. That substitution is forced: decad
// refuses a circle fragment whose trim came from a spline crossing, with
// "a *sketch.Circle fragment has an uncertified trim (TExact = false)". The
// sketch step draws the solid circle and proves the split yields this same
// six-curve loop, so the two agree on the loop this builds.
//
// When the flank starts inside the root circle (the embedded case) there are no
// flank-to-root lines and the loop is four curves. The flank is then sampled
// from the root radius outward rather than from the base radius, so its first
// point lands on the root arc; that is the same restriction of the same
// involute, resampled.
func toothSection(t *testing.T, w *sketch.World, plane *sketch.Plane,
	d involute.Dimensions, teeth float64, steps int, angle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	fix := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}

	left, right := sectionFlanks(d, teeth, steps, angle)
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = fix(left[i].X, left[i].Y)
		rp[i] = fix(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(lp...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(rp...); err != nil {
		t.Fatalf("right flank: %v", err)
	}
	s.CreateArc(origin, rp[len(rp)-1], lp[len(lp)-1])

	if d.Embedded() {
		s.CreateArc(origin, rp[0], lp[0])
	} else {
		lf := fix(onRoot(d, left[0]))
		rf := fix(onRoot(d, right[0]))
		s.CreateLine(lf, lp[0])
		s.CreateLine(rp[0], rf)
		s.CreateArc(origin, rf, lf)
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	ps := s.Profiles()
	if len(ps) != 1 {
		t.Fatalf("tooth outline closed %d regions, want 1", len(ps))
	}
	return s, ps[0]
}

// polyToothSection draws the same tooth outline with every curve replaced by
// straight segments — see substitutions 1 and 2 in the package comment. arcSteps
// segments stand in for each of the tip and root arcs.
func polyToothSection(t *testing.T, w *sketch.World, plane *sketch.Plane,
	d involute.Dimensions, teeth float64, steps, arcSteps int, angle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	fix := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	chain := func(pts []*sketch.Point) {
		for i := 0; i+1 < len(pts); i++ {
			s.CreateLine(pts[i], pts[i+1])
		}
	}
	arc := func(radius, from, to float64) []*sketch.Point {
		out := make([]*sketch.Point, 0, arcSteps+1)
		for i := 0; i <= arcSteps; i++ {
			a := from + (to-from)*float64(i)/float64(arcSteps)
			out = append(out, fix(radius*math.Cos(a), radius*math.Sin(a)))
		}
		return out
	}

	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = fix(left[i].X, left[i].Y)
		rp[i] = fix(right[i].X, right[i].Y)
	}
	lf := fix(onRoot(d, left[0]))
	rf := fix(onRoot(d, right[0]))

	chain(append([]*sketch.Point{lf}, lp...))
	tip := arc(d.Tip, bearing(left[len(left)-1]), bearing(right[len(right)-1]))
	tip[0], tip[len(tip)-1] = lp[len(lp)-1], rp[len(rp)-1]
	chain(tip)
	down := make([]*sketch.Point, 0, len(rp)+1)
	for i := len(rp) - 1; i >= 0; i-- {
		down = append(down, rp[i])
	}
	chain(append(down, rf))
	root := arc(d.Root, bearing(right[0]), bearing(left[0]))
	root[0], root[len(root)-1] = rf, lf
	chain(root)

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
	ps := s.Profiles()
	if len(ps) != 1 {
		t.Fatalf("tooth outline closed %d regions, want 1", len(ps))
	}
	return s, ps[0]
}

// sectionFlanks samples both flanks for the outline builders above. It is spur's
// own base-to-tip sampling except in the embedded case, where the flank is
// sampled from the ROOT radius outward so its first point lands on the root arc
// this outline draws explicitly.
//
// That resampling is a substitution local to the extruded outline. Spur samples
// from the base circle even when the base circle sits inside the root circle,
// and the sketch step below does exactly that, letting the flank cross the solid
// root circle and letting profile detection split it. decad refuses to record a
// circle fragment trimmed by a spline crossing ("a *sketch.Circle fragment has
// an uncertified trim (TExact = false)"), so no body can be built from that
// loop; restricting the same involute to the part outside the root circle is
// what does build. Cost: the flank is the same curve over a shorter interval,
// re-fitted through different points, so its shape differs from the drawn one by
// the fitting error and no flank measurement may be read off the result. The
// edge COUNT, which is all the chamfer step reads, is unaffected.
func sectionFlanks(d involute.Dimensions, teeth float64, steps int, angle float64) (left, right []involute.Pt) {
	if !d.Embedded() {
		return involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	}
	mirrored := make([]involute.Pt, 0, steps)
	for i := range steps {
		r := d.Root + (d.Tip-d.Root)*float64(i)/float64(steps-1)
		x, y, ok := involute.Point(d.Base, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, involute.Pt{X: x, Y: -y})
	}
	px, py, _ := involute.Point(d.Base, d.Pitch)
	rotateAngle := math.Pi/(2*teeth) - math.Atan2(-py, px)
	for _, p := range mirrored {
		lx, ly := involute.Rotate(p.X, p.Y, rotateAngle)
		rx, ry := lx, -ly
		lx, ly = involute.Rotate(lx, ly, angle)
		rx, ry = involute.Rotate(rx, ry, angle)
		left = append(left, involute.Pt{X: lx, Y: ly})
		right = append(right, involute.Pt{X: rx, Y: ry})
	}
	return left, right
}

// onRoot is the radial foot of a flank start on the root circle.
func onRoot(d involute.Dimensions, p involute.Pt) (float64, float64) {
	n := math.Hypot(p.X, p.Y)
	return d.Root * p.X / n, d.Root * p.Y / n
}

// bearing is a point's polar angle about the gear centre.
func bearing(p involute.Pt) float64 { return math.Atan2(p.Y, p.X) }
