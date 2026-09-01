package main

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/solidlens"
)

// station is one cross-section of a swept gear: the height it sits at and
// the angle its outline is twisted by. A spur gear has two stations at the
// same twist, a helical gear two at different twists, and a herringbone gear
// three, the middle one twisted and the two ends not.
type station struct {
	z     float64
	twist float64
}

// sweep meshes the ruled surface through the stations, closes it with an end
// cap at each extreme, and returns it ready to render.
//
// Why this exists rather than a decad loft: decad builds the loft but cannot
// tessellate one — the evaluator supports prism, cup and faceted payloads
// only — and an untessellatable body cannot be rendered or cut. The surface
// between two sections of a loft is ruled, so meshing it is a matter of
// joining the sections point for point, which is what this does. Nothing
// about the gear itself is approximated here: the outlines are the same
// involute sections decad extrudes for the spur render.
func sweep(dims involute.Dimensions, teeth int, boreRadius float64, stations []station) (*solidlens.Mesh, error) {
	if len(stations) < 2 {
		return nil, fmt.Errorf("sweep needs at least two stations, got %d", len(stations))
	}
	rings := make([][]involute.Pt, len(stations))
	bores := make([][]involute.Pt, len(stations))
	for i, st := range stations {
		rings[i] = outline(dims, float64(teeth), st.twist)
		if i > 0 && len(rings[i]) != len(rings[0]) {
			return nil, fmt.Errorf("station %d has %d outline points, station 0 has %d; the ruled surface needs them to correspond",
				i, len(rings[i]), len(rings[0]))
		}
		bores[i] = boreRing(rings[i], boreRadius)
	}

	n := len(rings[0])
	bored := boreRadius > 0
	perStation := n
	if bored {
		perStation = 2 * n
	}
	vertices := make([]solidlens.Vec, 0, len(stations)*perStation+2)
	for i, st := range stations {
		for _, p := range rings[i] {
			vertices = append(vertices, solidlens.Vec{X: p.X, Y: p.Y, Z: st.z})
		}
		for _, p := range bores[i] {
			vertices = append(vertices, solidlens.Vec{X: p.X, Y: p.Y, Z: st.z})
		}
	}
	outer := func(s, i int) int { return s*perStation + i%n }
	inner := func(s, i int) int { return s*perStation + n + i%n }

	var triangles [][3]int
	// Walls. The outline runs clockwise seen from +Z, so this winding puts
	// the outer wall's normal away from the axis and the bore wall's toward
	// it.
	for s := 0; s+1 < len(stations); s++ {
		for i := range n {
			lo, hi := outer(s, i), outer(s+1, i)
			loNext, hiNext := outer(s, i+1), outer(s+1, i+1)
			triangles = append(triangles, [3]int{lo, hi, loNext}, [3]int{loNext, hi, hiNext})
			if !bored {
				continue
			}
			blo, bhi := inner(s, i), inner(s+1, i)
			bloNext, bhiNext := inner(s, i+1), inner(s+1, i+1)
			triangles = append(triangles, [3]int{blo, bloNext, bhi}, [3]int{bloNext, bhiNext, bhi})
		}
	}

	last := len(stations) - 1
	if bored {
		triangles = append(triangles, capRing(outer, inner, 0, n, false)...)
		triangles = append(triangles, capRing(outer, inner, last, n, true)...)
	} else {
		bottomHub := len(vertices)
		vertices = append(vertices, solidlens.Vec{Z: stations[0].z})
		topHub := len(vertices)
		vertices = append(vertices, solidlens.Vec{Z: stations[last].z})
		for i := range n {
			triangles = append(triangles,
				[3]int{bottomHub, outer(0, i), outer(0, i+1)},
				[3]int{topHub, outer(last, i+1), outer(last, i)})
		}
	}
	return solidlens.NewMesh(vertices, triangles)
}

// capRing triangulates the flat face between the outline and the bore at one
// station. up selects the face's normal: false gives the -Z end cap, true the
// +Z one.
func capRing(outer, inner func(s, i int) int, s, n int, up bool) [][3]int {
	out := make([][3]int, 0, 2*n)
	for i := range n {
		o, oNext := outer(s, i), outer(s, i+1)
		b, bNext := inner(s, i), inner(s, i+1)
		if up {
			out = append(out, [3]int{o, bNext, oNext}, [3]int{o, b, bNext})
			continue
		}
		out = append(out, [3]int{o, oNext, bNext}, [3]int{o, bNext, b})
	}
	return out
}

// boreRing places one bore-circle point per outline point, at the same
// bearing, so the end cap between them is a plain strip of quads. The
// outline is star-shaped about the gear centre and the bore is well inside
// the root circle, so the strip never folds over itself.
func boreRing(loop []involute.Pt, radius float64) []involute.Pt {
	if radius <= 0 {
		return nil
	}
	ring := make([]involute.Pt, len(loop))
	for i, p := range loop {
		a := math.Atan2(p.Y, p.X)
		ring[i] = involute.Pt{X: radius * math.Cos(a), Y: radius * math.Sin(a)}
	}
	return ring
}
