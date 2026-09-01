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

// boreSegments is how many sides the bore is drawn with. Its ring turns with
// its station's outline, so bore point j always faces outline point 0 and
// both the bore wall and the end cap pair up without a seam. On a twisted
// gear that makes the bore a helicoid rather than a cylinder, by under a
// micrometre at the sizes rendered here.
const boreSegments = 144

// borePhase offsets the bore ring by half a segment.
//
// Where a tooth's stub foot sits directly below its flank start, the outline
// runs radially and the end cap has to cover that with a wedge of no width.
// The wedge is a real triangle only while its third corner, a bore point,
// lies off the ray — and the tooth pitch here is an exact multiple of the
// bore's own step, so an unshifted ring puts a bore point on every one of
// those rays. Half a step moves them clear. TestSweptMeshIsClosed is what
// says it stayed clear.
const borePhase = math.Pi / boreSegments

// sweep meshes the ruled surface through the stations, closes it with an end
// cap at each extreme, and returns it ready to render.
//
// Why this exists rather than a decad loft: decad builds the loft but cannot
// tessellate one — the evaluator supports prism, cup and faceted payloads
// only — and an untessellatable body cannot be rendered or cut. The surface
// between two sections of a loft is ruled, so meshing it is a matter of
// joining the sections point for point, which is what this does. Nothing
// about the gear itself is approximated here: the outlines are the involute
// sections proof/involute draws.
func sweep(dims involute.Dimensions, teeth int, boreRadius float64, stations []station) (*solidlens.Mesh, error) {
	if len(stations) < 2 {
		return nil, fmt.Errorf("sweep needs at least two stations, got %d", len(stations))
	}
	rings := make([][]involute.Pt, len(stations))
	for i, st := range stations {
		rings[i] = outline(dims, float64(teeth), st.twist)
		if i > 0 && len(rings[i]) != len(rings[0]) {
			return nil, fmt.Errorf("station %d has %d outline points, station 0 has %d; the ruled surface needs them to correspond",
				i, len(rings[i]), len(rings[0]))
		}
	}
	n := len(rings[0])
	bored := boreRadius > 0
	bores := make([][]involute.Pt, len(stations))
	boreCount := 0
	if bored {
		boreCount = boreSegments
		for i := range stations {
			bores[i] = boreRing(boreRadius, bearing(rings[i][0])-borePhase)
		}
	}
	perStation := n + boreCount

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
	inner := func(s, j int) int { return s*perStation + n + j%boreCount }

	var triangles [][3]int
	// Walls. The outline runs clockwise seen from +Z, so this winding puts
	// the outer wall's normal away from the axis and the bore wall's toward
	// it.
	for s := 0; s+1 < len(stations); s++ {
		for i := range n {
			lo, hi := outer(s, i), outer(s+1, i)
			loNext, hiNext := outer(s, i+1), outer(s+1, i+1)
			triangles = append(triangles, [3]int{lo, hi, loNext}, [3]int{loNext, hi, hiNext})
		}
		if !bored {
			continue
		}
		for j := range boreCount {
			lo, hi := inner(s, j), inner(s+1, j)
			loNext, hiNext := inner(s, j+1), inner(s+1, j+1)
			triangles = append(triangles, [3]int{lo, loNext, hi}, [3]int{loNext, hiNext, hi})
		}
	}

	last := len(stations) - 1
	if !bored {
		bottomHub := len(vertices)
		vertices = append(vertices, solidlens.Vec{Z: stations[0].z})
		topHub := len(vertices)
		vertices = append(vertices, solidlens.Vec{Z: stations[last].z})
		for i := range n {
			triangles = append(triangles,
				[3]int{bottomHub, outer(0, i), outer(0, i+1)},
				[3]int{topHub, outer(last, i+1), outer(last, i)})
		}
		return solidlens.NewMesh(vertices, triangles)
	}
	for _, cap := range []struct {
		s  int
		up bool
	}{{0, false}, {last, true}} {
		face, err := capFace(rings[cap.s], bores[cap.s],
			func(i int) int { return outer(cap.s, i) },
			func(j int) int { return inner(cap.s, j) }, cap.up)
		if err != nil {
			return nil, err
		}
		triangles = append(triangles, face...)
	}
	return solidlens.NewMesh(vertices, triangles)
}

// capFace triangulates the flat face between one station's outline and the
// bore.
//
// The two loops do not correspond point for point — the bore has its own even
// spacing — so the face is closed by merging them in bearing order, taking one
// triangle per point of either loop.
//
// Each triangle is then wound by its own signed area rather than by the order
// the merge produced it in. A tooth's stub runs radially, outward on one flank
// and inward on the other, and a merge step across the inward one comes out
// turning the opposite way. Left as it is, that triangle faces into the gear,
// and the renderer draws a line along all three of its edges because each
// neighbour disagrees with it by 180 degrees.
func capFace(loop, bore []involute.Pt, outer, inner func(int) int, up bool) ([][3]int, error) {
	n, m := len(loop), len(bore)
	if n == 0 || m == 0 {
		return nil, fmt.Errorf("cap face needs both loops, got %d outline and %d bore points", n, m)
	}
	outerBearings := descending(loop)
	boreBearings := descending(bore)
	triangles := make([][3]int, 0, n+m)
	// The bottom cap faces -Z, which a clockwise turn through the three
	// corners gives; the top cap faces +Z and wants the other turn.
	want := -1.0
	if up {
		want = 1
	}
	emit := func(ai, bi, ci int, a, b, c involute.Pt) {
		if math.Signbit(turn(a, b, c)) != math.Signbit(want) {
			bi, ci = ci, bi
		}
		triangles = append(triangles, [3]int{ai, bi, ci})
	}
	for i, j := 0, 0; i < n || j < m; {
		takeOuter := j >= m
		if i < n && j < m {
			takeOuter = nextBearing(outerBearings, i) > nextBearing(boreBearings, j)
		}
		if takeOuter {
			emit(outer(i), outer(i+1), inner(j), loop[i%n], loop[(i+1)%n], bore[j%m])
			i++
			continue
		}
		emit(outer(i), inner(j+1), inner(j), loop[i%n], bore[(j+1)%m], bore[j%m])
		j++
	}
	return triangles, nil
}

// turn is twice the signed area of the triangle, positive for a
// counter-clockwise turn through a, b, c.
func turn(a, b, c involute.Pt) float64 {
	return (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
}

// descending unwraps a clockwise loop's bearings so they fall monotonically
// from the first point instead of jumping at ±pi.
func descending(loop []involute.Pt) []float64 {
	out := make([]float64, len(loop))
	out[0] = bearing(loop[0])
	for i := 1; i < len(loop); i++ {
		// The step is taken as the shortest turn from the previous bearing,
		// then held at or below zero: the loop only ever goes clockwise, so a
		// positive step is rounding noise, and letting one through would drag
		// the whole tail of the loop back by a full turn.
		step := wrapPi(bearing(loop[i]) - out[i-1])
		if step > 0 {
			step = 0
		}
		out[i] = out[i-1] + step
	}
	return out
}

// wrapPi folds an angle into (-pi, pi].
func wrapPi(a float64) float64 {
	a = math.Mod(a, 2*math.Pi)
	if a > math.Pi {
		a -= 2 * math.Pi
	}
	if a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// nextBearing is the bearing of the point after k, continuing past the end of
// the loop into the next turn.
func nextBearing(bearings []float64, k int) float64 {
	if k+1 < len(bearings) {
		return bearings[k+1]
	}
	return bearings[0] - 2*math.Pi
}

// boreRing is the bore circle, walked clockwise from start so it runs the
// same way round as the outline.
func boreRing(radius, start float64) []involute.Pt {
	ring := make([]involute.Pt, boreSegments)
	for i := range ring {
		a := start - 2*math.Pi*float64(i)/boreSegments
		ring[i] = involute.Pt{X: radius * math.Cos(a), Y: radius * math.Sin(a)}
	}
	return ring
}
