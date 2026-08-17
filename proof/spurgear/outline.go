package spurgear

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/sketch"
)

// ErrEmbedded identifies the branch where involute flanks cross the root
// circle instead of meeting it at shared endpoints. The current decad seam
// rejects that crossing, so the 3D proof skips it explicitly.
var ErrEmbedded = errors.New("spurgear: embedded flank crosses the root circle")

// Outline is a closed, line-only approximation of a spur gear section.
//
// Fusion uses fitted involute splines and circular arcs. This outline keeps the
// same involute sample points but chords every curved boundary so decad's line
// profile loft and boolean paths can consume it. Its polygon area is therefore
// the reference for the solid proof, rather than an ideal involute area.
type Outline struct {
	Dimensions involute.Dimensions
	Teeth      int
	Steps      int
	RootChords int
	Points     []involute.Pt
	Area       float64
	MinSegment float64
}

// NewOutline creates a non-embedded chorded spur outline.
func NewOutline(module, teeth, pressureAngleDeg float64, steps, rootChords int) (Outline, error) {
	if !finitePositive(module) {
		return Outline{}, fmt.Errorf("module must be positive and finite")
	}
	if teeth < 3 || float64(int(teeth)) != teeth {
		return Outline{}, fmt.Errorf("tooth count must be an integer of at least 3, got %g", teeth)
	}
	if !finitePositive(pressureAngleDeg) || pressureAngleDeg >= 90 {
		return Outline{}, fmt.Errorf("pressure angle must be in (0, 90), got %g", pressureAngleDeg)
	}
	if steps < 2 {
		return Outline{}, fmt.Errorf("involute sample count must be at least 2, got %d", steps)
	}
	if rootChords < 1 {
		return Outline{}, fmt.Errorf("root chord count must be positive, got %d", rootChords)
	}

	n := int(teeth)
	dim := involute.Derive(module, teeth, pressureAngleDeg*math.Pi/180)
	if dim.Embedded() {
		return Outline{}, fmt.Errorf("%w: base radius %.6g is below root radius %.6g", ErrEmbedded, dim.Base, dim.Root)
	}
	left, right := make([][]involute.Pt, n), make([][]involute.Pt, n)
	for i := range n {
		angle := 2 * math.Pi * float64(i) / float64(n)
		left[i], right[i] = involute.Flanks(dim.Base, dim.Tip, dim.Pitch, teeth, steps, angle)
		if len(left[i]) < 2 || len(right[i]) < 2 {
			return Outline{}, fmt.Errorf("tooth %d has too few involute samples", i)
		}
	}

	points := make([]involute.Pt, 0, n*(2*steps+rootChords+3))
	appendPoint := func(p involute.Pt) {
		if len(points) > 0 && samePoint(points[len(points)-1], p) {
			return
		}
		points = append(points, p)
	}
	for i := range n {
		rightBase := right[i][0]
		leftBase := left[i][0]
		rightRoot := radialPoint(rightBase, dim.Root)
		leftRoot := radialPoint(leftBase, dim.Root)
		appendPoint(rightRoot)
		appendPoint(rightBase)
		for _, p := range right[i][1:] {
			appendPoint(p)
		}
		appendPoint(left[i][len(left[i])-1])
		for j := len(left[i]) - 2; j >= 0; j-- {
			appendPoint(left[i][j])
		}
		appendPoint(leftRoot)

		nextRightRoot := radialPoint(right[(i+1)%n][0], dim.Root)
		a0 := math.Atan2(leftRoot.Y, leftRoot.X)
		a1 := math.Atan2(nextRightRoot.Y, nextRightRoot.X)
		for a1 <= a0 {
			a1 += 2 * math.Pi
		}
		for j := 1; j < rootChords; j++ {
			a := a0 + (a1-a0)*float64(j)/float64(rootChords)
			appendPoint(involute.Pt{X: dim.Root * math.Cos(a), Y: dim.Root * math.Sin(a)})
		}
	}
	if len(points) < 3 || samePoint(points[0], points[len(points)-1]) {
		return Outline{}, fmt.Errorf("outline has fewer than three distinct points")
	}

	area := polygonArea(points)
	minSegment := math.Inf(1)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		length := math.Hypot(q.X-p.X, q.Y-p.Y)
		if length == 0 {
			return Outline{}, fmt.Errorf("outline contains a zero-length segment at %d", i)
		}
		if length < minSegment {
			minSegment = length
		}
	}
	return Outline{
		Dimensions: dim,
		Teeth:      n,
		Steps:      steps,
		RootChords: rootChords,
		Points:     points,
		Area:       area,
		MinSegment: minSegment,
	}, nil
}

// Sketch creates the line-only profile in a fresh XY sketch.
func (o Outline) Sketch() (*sketch.Sketch, *sketch.Profile, error) {
	if len(o.Points) < 3 {
		return nil, nil, fmt.Errorf("outline has no boundary")
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, nil, err
	}
	points := make([]*sketch.Point, len(o.Points))
	for i, p := range o.Points {
		points[i] = s.CreatePoint(p.X, p.Y)
	}
	for i, p := range points {
		s.CreateLine(p, points[(i+1)%len(points)])
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, nil, fmt.Errorf("outline produced %d profiles, want 1", len(profiles))
	}
	if !profiles[0].Valid {
		return nil, nil, fmt.Errorf("outline profile is invalid")
	}
	return s, profiles[0], nil
}

func radialPoint(p involute.Pt, radius float64) involute.Pt {
	length := math.Hypot(p.X, p.Y)
	return involute.Pt{X: p.X * radius / length, Y: p.Y * radius / length}
}

func polygonArea(points []involute.Pt) float64 {
	area := 0.0
	for i, p := range points {
		q := points[(i+1)%len(points)]
		area += p.X*q.Y - q.X*p.Y
	}
	return math.Abs(area) / 2
}

func samePoint(a, b involute.Pt) bool { return a.X == b.X && a.Y == b.Y }

func finitePositive(v float64) bool {
	return v > 0 && math.IsInf(v, 0) == false && math.IsNaN(v) == false
}
