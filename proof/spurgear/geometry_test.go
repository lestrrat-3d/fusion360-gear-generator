// Package spurgear_test proves the spur gear build compiled in
// spec/spurgear/steps.md: the Gear Profile constraint scheme in the sketch
// engine, and the solid steps (extrude, chamfer, body, pattern, combine,
// fillet, bore) in decad.
//
// This file holds the shared geometry: parameter derivation, the involute
// flank placement (imported from proof/involute, never re-derived), and the
// scaffold sketches the solid steps extrude. All bench lengths are in
// millimetres, the sketch engine's base unit; Fusion's internal unit is cm,
// which scales every length by 10 and changes no count, ratio or angle this
// proof asserts.
package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys shared by every case table. Angles are radians, lengths mm.
const (
	pModule        = "Module"
	pToothNumber   = "ToothNumber"
	pPressureAngle = "PressureAngle"
	pInvoluteSteps = "InvoluteSteps"
	pAngle         = "Angle"
	pThickness     = "Thickness"
	pBoreDiameter  = "BoreDiameter"
	pChamferTooth  = "ChamferTooth"
	pAnchorX       = "AnchorX"
	pAnchorY       = "AnchorY"
)

// gearGeom bundles what every step derives from the three primary inputs.
type gearGeom struct {
	module, toothNumber, pressureAngle float64
	steps                              int
	angle                              float64
	dims                               involute.Dimensions
	left, right                        []involute.Pt
}

// geom derives the circle radii and flank samples for one case, using the
// shared involute package for the tooth math (spec instructions.md step 4;
// the exact involute parameterisation is pinned there and implemented once in
// proof/involute).
func geomOf(t testing.TB, p map[string]float64) *gearGeom {
	t.Helper()
	g := &gearGeom{
		module:        p[pModule],
		toothNumber:   p[pToothNumber],
		pressureAngle: p[pPressureAngle],
		steps:         int(p[pInvoluteSteps]),
		angle:         p[pAngle],
	}
	if g.module <= 0 || g.toothNumber <= 0 || g.steps < 2 {
		t.Fatalf("case is missing Module/ToothNumber/InvoluteSteps")
	}
	g.dims = involute.Derive(g.module, g.toothNumber, g.pressureAngle)
	g.left, g.right = involute.Flanks(g.dims.Base, g.dims.Tip, g.dims.Pitch,
		g.toothNumber, g.steps, g.angle)
	return g
}

// embedded reports the profile shape the way the generator decides it
// (fusion.md [SPUR-F-FLANK-ROOT]): the first drawn flank sample's radius
// against the root radius, strict less-than, no tolerance.
func (g *gearGeom) embedded() bool {
	first := math.Hypot(g.left[0].X, g.left[0].Y)
	return first < g.dims.Root
}

// filletRadius is the spec's derived FilletRadius: half the root valley arc
// scaled by the 0.9 clearance and the spur helix factor 1
// (instructions.md Variables, Tooth Space Angle At Root onward).
func (g *gearGeom) filletRadius() float64 {
	spaceAngle := math.Pi/g.toothNumber - 2*(math.Tan(g.pressureAngle)-g.pressureAngle)
	return (g.dims.Root * spaceAngle / 2) * 0.9 * 1
}

// rotate turns (x, y) counter-clockwise by a.
func rotate(x, y, a float64) (float64, float64) {
	c, s := math.Cos(a), math.Sin(a)
	return x*c - y*s, x*s + y*c
}

// rootCrossing returns the left flank's crossing of the root circle for an
// embedded tooth, run through the same mirror-then-centre transform as the
// flank samples (instructions.md step 4.2-4.4): mirror across +X, rotate so
// the pitch crossing lands at +pi/(2N), then apply the requested angle.
func (g *gearGeom) rootCrossing() (float64, float64) {
	cx, cy, ok := involute.Point(g.dims.Base, g.dims.Root)
	if !ok {
		panic("rootCrossing called on a non-embedded gear")
	}
	px, py, _ := involute.Point(g.dims.Base, g.dims.Pitch)
	rotateAngle := math.Pi/(2*g.toothNumber) - math.Atan2(-py, px)
	x, y := rotate(cx, -cy, rotateAngle)
	return rotate(x, y, g.angle)
}

// newXYSketch returns a fresh sketch on the world XY plane for the 3D
// scaffolds; the 2D constraint steps get theirs from proofkit.
func newXYSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

func mustSolve(t *testing.T, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve scaffold sketch: %v", err)
	}
}

// toothCorners are the chorded tooth section's corner points for one tooth,
// already rotated to its slot: stub feet on the root circle, flank starts,
// and flank tips. For an embedded gear the "feet" and "starts" coincide at
// the involute's root-circle crossing and no stub exists.
type toothCorners struct {
	rsf, rs, rt, lt, ls, lsf [2]float64
	stubbed                  bool
}

func (g *gearGeom) corners(slot float64) toothCorners {
	last := len(g.left) - 1
	var c toothCorners
	rot := func(p involute.Pt) [2]float64 {
		x, y := rotate(p.X, p.Y, slot)
		return [2]float64{x, y}
	}
	c.rt = rot(g.right[last])
	c.lt = rot(g.left[last])
	if g.embedded() {
		// The flanks cross the root circle; the chord stands in for the
		// involute from that crossing to the tip, and no stub exists. The
		// right crossing is the left one mirrored across the tooth's angle
		// direction.
		lx, ly := g.rootCrossing()
		mx, my := rotate(lx, ly, -g.angle)
		rx, ry := rotate(mx, -my, g.angle)
		lx, ly = rotate(lx, ly, slot)
		rx, ry = rotate(rx, ry, slot)
		c.ls = [2]float64{lx, ly}
		c.rs = [2]float64{rx, ry}
		c.lsf, c.rsf = c.ls, c.rs
		c.stubbed = false
		return c
	}
	c.rs = rot(g.right[0])
	c.ls = rot(g.left[0])
	first := math.Hypot(g.left[0].X, g.left[0].Y)
	c.rsf = [2]float64{g.dims.Root * c.rs[0] / first, g.dims.Root * c.rs[1] / first}
	c.lsf = [2]float64{g.dims.Root * c.ls[0] / first, g.dims.Root * c.ls[1] / first}
	c.stubbed = true
	return c
}

// chordedToothProfile draws the single-tooth section with straight chords in
// place of the involute splines and an explicit root arc, and returns its one
// region.
//
// Two substitutions against the real Gear Profile geometry, both forced by
// the solid harness and recorded here because the step list alone would lose
// them (steps.md steps 11-19 all build on this section):
//
//   - The flanks are straight chords between the flank start and tip, not
//     fitted splines. decad refuses to extrude the involute fit spline
//     outright ("a free-form wall edge's chain has curvature signs that
//     conflict across its spans or joints" — the natural-cubic interpolant
//     through the involute samples wobbles in curvature), so no spline-walled
//     prism survives the gate. What the chord keeps: the flank's endpoints,
//     the stub lines, both arcs, and every curve COUNT the profile search
//     matches on. What it costs: the flank's curvature is not carried into
//     3D, so nothing solid-side asserts the involute shape — that is pinned
//     by stepGearProfileSketch on the real splines.
//   - The root arc is drawn as an explicit arc between the stub feet rather
//     than derived by profile-splitting the solid root circle. decad refuses
//     a profile whose boundary is a circle fragment ("a *sketch.Circle
//     fragment has an uncertified trim"). The split-derived boundary itself
//     is proven in stepGearProfileSketch, which draws the full solid circle
//     and lets profile detection cut it.
func chordedToothProfile(t *testing.T, s *sketch.Sketch, g *gearGeom) *sketch.Profile {
	t.Helper()
	origin := s.CreatePoint(0, 0)
	c := g.corners(0)
	rs := s.CreatePoint(c.rs[0], c.rs[1])
	rt := s.CreatePoint(c.rt[0], c.rt[1])
	lt := s.CreatePoint(c.lt[0], c.lt[1])
	ls := s.CreatePoint(c.ls[0], c.ls[1])
	s.CreateLine(rs, rt)
	s.CreateArc(origin, rt, lt)
	s.CreateLine(lt, ls)
	if c.stubbed {
		rsf := s.CreatePoint(c.rsf[0], c.rsf[1])
		lsf := s.CreatePoint(c.lsf[0], c.lsf[1])
		s.CreateLine(rsf, rs)
		s.CreateLine(ls, lsf)
		s.CreateArc(origin, rsf, lsf)
	} else {
		s.CreateArc(origin, rs, ls)
	}
	mustSolve(t, s)
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("chorded tooth section: expected 1 region, got %d", len(profiles))
	}
	return profiles[0]
}

// discProfile draws the solid disc of the given radius and returns its region.
func discProfile(t *testing.T, s *sketch.Sketch, radius float64) *sketch.Profile {
	t.Helper()
	origin := s.CreatePoint(0, 0)
	s.CreateCircle(origin, radius)
	mustSolve(t, s)
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("disc: expected 1 region, got %d", len(profiles))
	}
	return profiles[0]
}

// wholeGearOutline draws the full N-tooth gear outline as one closed loop —
// per tooth: stub up, chord flank, tip arc, chord flank, stub down, root
// valley arc to the next tooth — and returns its single region. This is the
// exact shape the pattern-and-combine steps leave behind, built as one
// profile because decad's boolean refuses the tangent tooth-on-root-cylinder
// contact the real combine performs (see stepCombineTeeth).
func wholeGearOutline(t *testing.T, s *sketch.Sketch, g *gearGeom) *sketch.Profile {
	t.Helper()
	if g.embedded() {
		t.Fatalf("wholeGearOutline models the stubbed (non-embedded) shape")
	}
	origin := s.CreatePoint(0, 0)
	n := int(g.toothNumber)
	pitchAngle := 2 * math.Pi / g.toothNumber
	type pts struct{ rsf, rs, rt, lt, ls, lsf *sketch.Point }
	teeth := make([]pts, n)
	for k := 0; k < n; k++ {
		c := g.corners(float64(k) * pitchAngle)
		mk := func(xy [2]float64) *sketch.Point { return s.CreatePoint(xy[0], xy[1]) }
		teeth[k] = pts{mk(c.rsf), mk(c.rs), mk(c.rt), mk(c.lt), mk(c.ls), mk(c.lsf)}
	}
	for k := 0; k < n; k++ {
		tp := teeth[k]
		next := teeth[(k+1)%n]
		s.CreateLine(tp.rsf, tp.rs)
		s.CreateLine(tp.rs, tp.rt)
		s.CreateArc(origin, tp.rt, tp.lt)
		s.CreateLine(tp.lt, tp.ls)
		s.CreateLine(tp.ls, tp.lsf)
		s.CreateArc(origin, tp.lsf, next.rsf)
	}
	mustSolve(t, s)
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("whole-gear outline: expected 1 region, got %d", len(profiles))
	}
	return profiles[0]
}

// relDiff returns |a-b| / max(|a|,|b|,1).
func relDiff(a, b float64) float64 {
	scale := math.Max(math.Max(math.Abs(a), math.Abs(b)), 1)
	return math.Abs(a-b) / scale
}
