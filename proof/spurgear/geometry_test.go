// Package spurgear_test proves the spur gear build described by
// spec/spurgear/steps.md.
//
// Two harnesses carry it. proofkit rebuilds each constraint-bearing sketch in
// the sketch engine and gates it on the engine's own verdict; proofkit3d
// rebuilds each solid step in decad and gates it on decad's. Everything the
// spec pins about a step — a curve count a later profile search keys on, a face
// or edge count a selection keys on, an extent, a volume — is asserted against
// the geometry the step actually built.
//
// Where a harness refuses the real geometry the proof substitutes geometry it
// accepts and says, at the substitution, what the substitute stops proving.
// Every such note is on the step it belongs to, so a reader of the proof finds
// the limit next to the thing that has it.
package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// tol is the length comparison tolerance in millimetres. It is loose enough to
// absorb the solver's residual and decad's faceting, and far tighter than any
// difference the spec's rules turn on.
const tol = 1e-6

// gear is one proof case's parameter set, reduced to the four circle radii the
// spur family builds on. The names match the spec's user parameters.
type gear struct {
	Module        float64
	ToothNumber   float64
	PressureAngle float64 // radians
	Steps         int     // InvoluteSteps
	Angle         float64 // radians; draw()'s angle argument
	Thickness     float64
	BoreDiameter  float64
	ChamferTooth  float64
	FilletRadius  float64
	Dims          involute.Dimensions
}

func readGear(p map[string]float64) gear {
	g := gear{
		Module:        p["module"],
		ToothNumber:   p["toothNumber"],
		PressureAngle: p["pressureAngle"],
		Steps:         int(p["involuteSteps"]),
		Angle:         p["angle"],
		Thickness:     p["thickness"],
		BoreDiameter:  p["boreDiameter"],
		ChamferTooth:  p["chamferTooth"],
		FilletRadius:  p["filletRadius"],
	}
	g.Dims = involute.Derive(g.Module, g.ToothNumber, g.PressureAngle)
	return g
}

func deg(d float64) float64 { return d * math.Pi / 180 }

// zAxis is the gear's main axis. Every proof case sketches on the world XY
// datum, so the target plane's normal is +Z throughout.
func zAxis() r3.Vec { return r3.NewVec(0, 0, 1) }

// rootFoot is the point on the circle of radius r that lies on the ray from the
// gear centre through p — the foot of a flank-to-root line.
func rootFoot(r float64, p involute.Pt) involute.Pt {
	d := math.Hypot(p.X, p.Y)
	return involute.Pt{X: r * p.X / d, Y: r * p.Y / d}
}

// flankSamples is involute.Flanks with the first sample radius chosen by the
// caller rather than fixed at the base circle.
//
// The spline flank of the sketch steps starts on the base circle, exactly as
// step 4.1 prescribes, and involute.Flanks is used directly there. The solid
// steps need a flank whose first point is where the tooth actually meets the
// root circle, because decad records a boundary segment only when both of its
// bounds are the segment's own ends: a flank left running to the base circle
// and trimmed by the root circle becomes a fragment with an uncertified trim,
// which Extrude refuses outright (ErrUnrecordableProfile). Starting the
// sampling at the root radius in the embedded case puts the same involute
// through a start point the root arc can share. The involute itself is
// unchanged — the curve parameter is still taken from the base circle.
func flankSamples(d involute.Dimensions, toothNumber float64, steps int, angle, startR float64) (left, right []involute.Pt) {
	mirrored := make([]involute.Pt, 0, steps)
	for i := range steps {
		r := startR + (d.Tip-startR)*float64(i)/float64(steps-1)
		x, y, ok := involute.Point(d.Base, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, involute.Pt{X: x, Y: -y})
	}
	px, py, _ := involute.Point(d.Base, d.Pitch)
	rotateAngle := math.Pi/(2*toothNumber) - math.Atan2(-py, px)
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

// flankStartRadius is the radius the solid steps sample the flank from: the
// base circle when the tooth stands clear of the root circle, the root circle
// when the profile is embedded and the flank would otherwise start inside it.
func flankStartRadius(d involute.Dimensions) float64 {
	if d.Embedded() {
		return d.Root
	}
	return d.Base
}

// curveCounts is a tally of one detected region's DISTINCT boundary entities,
// by entity kind.
//
// This is the sketch-engine reading of the count Fusion's
// find_profile_by_curve_counts matches on, and the two differ in exactly one
// place. Fusion splits the solid root circle into two arcs where the tooth
// meets it, so Fusion's tooth loop counts a root ARC and Fusion's disc loop
// counts TWO arcs. The sketch engine leaves a closed curve whole and reports
// the same two regions with the root CIRCLE standing in for both: the tooth
// loop carries Circles 1 where Fusion carries the root arc, and the disc loop
// carries Circles 1 where Fusion carries its two arcs. Nothing else differs,
// and the number of places the tooth cuts the root circle — which is what makes
// Fusion's count two — is asserted separately by rootCircleCuts.
type curveCounts struct {
	Splines int
	Arcs    int
	Circles int
	Lines   int
}

func countCurves(p *sketch.Profile) curveCounts {
	var c curveCounts
	for _, e := range p.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			c.Splines++
		case *sketch.Arc:
			c.Arcs++
		case *sketch.Circle:
			c.Circles++
		case *sketch.Line:
			c.Lines++
		}
	}
	return c
}

// findProfileByCurveCounts is the proof's stand-in for the framework helper
// find_profile_by_curve_counts: it returns the one region whose distinct
// boundary entities match want exactly, and fails naming what it saw when none
// does or more than one does. Like the helper, it never falls back to a region
// with a different count.
func findProfileByCurveCounts(t testing.TB, s *sketch.Sketch, want curveCounts) *sketch.Profile {
	t.Helper()
	var found *sketch.Profile
	seen := make([]curveCounts, 0, len(s.Profiles()))
	for _, p := range s.Profiles() {
		got := countCurves(p)
		seen = append(seen, got)
		if got != want {
			continue
		}
		if found != nil {
			t.Fatalf("profile search: %+v matches more than one region; saw %+v", want, seen)
		}
		found = p
	}
	if found == nil {
		t.Fatalf("profile search: no region has curve counts %+v; saw %+v", want, seen)
	}
	return found
}

// rootCircleCuts counts the distinct parameters at which the tooth cuts the
// root circle, ignoring the circle's own seam.
//
// Fusion's two profiles exist only because the tooth splits the root circle in
// two, and both of the spec's curve counts follow from that split being exactly
// two-fold: the tooth loop takes one of the pieces and the disc loop takes both.
// The sketch engine reports a run that straddles the circle's seam as two
// fragments, so the fragment count is not the cut count; the distinct interior
// bounds are.
func rootCircleCuts(p *sketch.Profile, root *sketch.Circle) int {
	cuts := make(map[float64]struct{})
	for _, e := range p.Outer {
		c, ok := e.Entity.(*sketch.Circle)
		if !ok || c != root {
			continue
		}
		for _, v := range []float64{e.TStart, e.TEnd} {
			if v <= 1e-12 || v >= 1-1e-12 {
				continue // the circle's own seam, not a cut
			}
			cuts[v] = struct{}{}
		}
	}
	return len(cuts)
}

// planarFaceAt reports whether f is a planar face whose plane is the plane
// through height along +Z — the test the spec spells as
// sketchPlane.isCoPlanarTo(face.geometry) once the target plane is the world XY
// datum and the extrude runs along +Z.
func planarFaceAt(f *decad.Face, height float64) bool {
	pl, ok := f.Surface().(decad.Plane)
	if !ok {
		return false
	}
	n := pl.Frame.N()
	if math.Abs(math.Abs(n.Z)-1) > tol {
		return false
	}
	return math.Abs(pl.Frame.Origin().Z-height) < tol
}

// cylindricalFacesOfRadius returns the cylindrical faces of b whose radius is
// r — the spec's "collect EVERY cylindrical face whose radius equals Root
// Circle Radius", and the same radius match step 8 uses on the root arc.
func cylindricalFacesOfRadius(b *decad.Body, r float64) []*decad.Face {
	var out []*decad.Face
	for _, f := range b.Faces() {
		cy, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		if math.Abs(cy.Radius.Mag()-r) < 1e-4 {
			out = append(out, f)
		}
	}
	return out
}

// arcEdgesOfRadius returns the circular edges of f whose radius is r. The
// tolerance is the spec's own 0.001 cm, expressed in millimetres.
func arcEdgesOfRadius(f *decad.Face, r float64) []*decad.Edge {
	var out []*decad.Edge
	for _, e := range f.Edges() {
		var got float64
		switch c := e.Curve().(type) {
		case decad.Arc3:
			got = c.Radius.Mag()
		case decad.Circle3:
			got = c.Radius.Mag()
		default:
			continue
		}
		if math.Abs(got-r) < 0.01 {
			out = append(out, e)
		}
	}
	return out
}

func centroidAngle(t *testing.T, b *decad.Body) (angle, radius float64) {
	t.Helper()
	c, err := b.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	return math.Atan2(c.Value.Y, c.Value.X), math.Hypot(c.Value.X, c.Value.Y)
}

func volumeOf(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	v, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	return v.Value.Mag()
}
