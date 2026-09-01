package main

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
)

// gear is one rendered example. The fields are the generator's own dialog
// inputs, so a reader can set the same numbers in Fusion and get the same
// shape.
type gear struct {
	name          string
	module        float64
	teeth         int
	pressureAngle float64
	thickness     float64
	// helixAngle is the angle the top section is twisted by, which is what
	// the helical spec means by Helix Angle: it is delivered straight to the
	// tooth generator as its draw angle, not converted from a lead.
	helixAngle   float64
	herringbone  bool
	boreDiameter float64
}

// sectionSteps is how many involute samples each flank is drawn from, and
// arcSteps how many straight segments stand in for each tip and root arc.
// Both are set by how smooth the silhouette has to look at the output size,
// not by any tolerance the gear itself carries.
const (
	sectionSteps = 16
	arcSteps     = 12
)

// mesh returns the gear ready to render.
//
// The section is checked through the sketch engine first — it has to close
// exactly one valid region, which is what catches an outline that crosses
// itself — and is then swept. The sweep is not a decad build: decad assembles
// a loft but cannot tessellate one, and an untessellatable body can neither
// be rendered nor cut, so the ruled surface between sections is meshed
// directly in sweep(). The tooth geometry is untouched by that. It comes from
// proof/involute, the package the 3D proofs draw their teeth from.
func (g gear) mesh(ctx context.Context) (*solidlens.Mesh, error) {
	dims := involute.Derive(g.module, float64(g.teeth), g.pressureAngle)
	if dims.Embedded() {
		return nil, fmt.Errorf("%s: base circle sits inside the root circle; the outline builder draws the stubbed tooth only", g.name)
	}
	if err := g.checkSection(ctx, dims); err != nil {
		return nil, err
	}
	return sweep(dims, g.teeth, g.boreDiameter/2, g.stations())
}

// twistSteps is how many bands each twisted run is cut into. A band of the
// swept surface is warped, so its two triangles do not share a normal and a
// tall one reads as a stripe under flat shading; splitting the run hides
// that. It also keeps the bore round, because every station's bore ring
// turns with its outline rather than being chorded across the whole twist.
const twistSteps = 10

// stations are the cross-sections the gear is swept through. A spur gear has
// two at the same angle. Helical twists the far face by the helix angle.
// Herringbone puts the twisted section at mid-body and returns to zero at the
// far face, which is the chevron the real generator gets by lofting one half
// and mirroring the solid.
func (g gear) stations() []station {
	if g.helixAngle == 0 {
		return []station{{z: 0}, {z: g.thickness}}
	}
	if !g.herringbone {
		return bands(station{z: 0}, station{z: g.thickness, twist: g.helixAngle})
	}
	mid := station{z: g.thickness / 2, twist: g.helixAngle}
	lower := bands(station{z: 0}, mid)
	upper := bands(mid, station{z: g.thickness})
	return append(lower, upper[1:]...)
}

// bands cuts the sweep from one station to the next into twistSteps bands,
// including both ends.
func bands(from, to station) []station {
	out := make([]station, 0, twistSteps+1)
	for i := 0; i <= twistSteps; i++ {
		t := float64(i) / twistSteps
		out = append(out, station{
			z:     from.z + (to.z-from.z)*t,
			twist: from.twist + (to.twist-from.twist)*t,
		})
	}
	return out
}

// checkSection draws the gear outline in the sketch engine and requires it to
// close exactly one valid region. Nothing downstream uses the sketch; this is
// here so a bad outline fails against the repo's own profile detection rather
// than turning up as a strange picture.
func (g gear) checkSection(ctx context.Context, dims involute.Dimensions) error {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return fmt.Errorf("%s: create sketch: %w", g.name, err)
	}
	loop := outline(dims, float64(g.teeth), 0)
	points := make([]*sketch.Point, len(loop))
	for i, pt := range loop {
		points[i] = s.CreatePoint(pt.X, pt.Y)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return fmt.Errorf("%s: solve section: %w", g.name, err)
	}
	regions := 0
	for _, r := range s.Profiles() {
		if r.Valid {
			regions++
		}
	}
	if regions != 1 {
		return fmt.Errorf("%s: outline closed %d valid regions, want exactly 1", g.name, regions)
	}
	return nil
}

// outline returns the closed N-tooth gear boundary as one polyline, walking
// tooth by tooth: up the left flank, across the tip, down the right flank,
// then along the root valley to the next tooth.
//
// The flank samples come from proof/involute, the same package the 3D proofs
// draw their teeth from, so the rendered flank is the generator's involute
// and not an approximation of one. The walk runs clockwise because
// involute.Flanks puts the left flank at the higher polar angle, so each
// tooth's slot is a NEGATIVE multiple of the pitch angle. sweep() depends on
// that direction for its face normals.
func outline(dims involute.Dimensions, teeth float64, angle float64) []involute.Pt {
	pitchAngle := 2 * math.Pi / teeth
	n := int(teeth)
	loop := make([]involute.Pt, 0, n*(2*sectionSteps+2*arcSteps))
	for k := range n {
		slot := angle - float64(k)*pitchAngle
		left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, teeth, sectionSteps, slot)
		nextLeft, _ := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, teeth, sectionSteps, slot-pitchAngle)

		leftFoot := onRoot(dims, left[0])
		rightFoot := onRoot(dims, right[0])
		loop = append(loop, leftFoot)
		loop = append(loop, left...)
		loop = append(loop, arcPoints(dims.Tip, bearing(left[len(left)-1]), bearing(right[len(right)-1]))...)
		for i := len(right) - 1; i >= 0; i-- {
			loop = append(loop, right[i])
		}
		loop = append(loop, rightFoot)
		loop = append(loop, arcPoints(dims.Root, bearing(rightFoot), bearing(onRoot(dims, nextLeft[0])))...)
	}
	return loop
}

// arcPoints returns the INTERIOR samples of the arc of the given radius from
// bearing a to bearing b, taking the short way round. The endpoints are left
// out because the caller already holds them as flank samples or feet, and
// repeating a point would make a zero-length segment.
func arcPoints(radius, a, b float64) []involute.Pt {
	sweep := math.Mod(b-a, 2*math.Pi)
	if sweep > math.Pi {
		sweep -= 2 * math.Pi
	}
	if sweep < -math.Pi {
		sweep += 2 * math.Pi
	}
	out := make([]involute.Pt, 0, arcSteps-1)
	for i := 1; i < arcSteps; i++ {
		t := a + sweep*float64(i)/float64(arcSteps)
		out = append(out, involute.Pt{X: radius * math.Cos(t), Y: radius * math.Sin(t)})
	}
	return out
}

// onRoot is the radial foot of a flank start on the root circle.
func onRoot(dims involute.Dimensions, p involute.Pt) involute.Pt {
	n := math.Hypot(p.X, p.Y)
	return involute.Pt{X: dims.Root * p.X / n, Y: dims.Root * p.Y / n}
}

// bearing is a point's polar angle about the gear centre.
func bearing(p involute.Pt) float64 { return math.Atan2(p.Y, p.X) }
