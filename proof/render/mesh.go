package render

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
)

// Vec2 is a point of a plane figure: a profile point (Z, R) for [Revolve], or
// a section point (X, Y) for [Prism].
type Vec2 struct{ X, Y float64 }

// Revolve meshes the solid of revolution a closed profile loop sweeps about
// the Z axis. A profile point is (Z, R), R never negative, and the loop is
// closed implicitly from its last point back to its first.
//
// An edge with R = 0 at both ends lies ON the axis and sweeps nothing, so it
// contributes no triangles; an edge with R = 0 at ONE end sweeps a cone and is
// closed as a fan on that end. Everything else sweeps a band.
//
// The loop is oriented before it is meshed, so a caller may hand it over
// either way round and still get outward normals: taken counter-clockwise in
// the (Z, R) plane, the surface normal points away from the material.
func Revolve(profile []Vec2, segments int) (*solidlens.Mesh, error) {
	if len(profile) < 3 {
		return nil, fmt.Errorf("revolve needs at least three profile points, got %d", len(profile))
	}
	if segments < 3 {
		return nil, fmt.Errorf("revolve needs at least three segments, got %d", segments)
	}
	loop := make([]Vec2, len(profile))
	copy(loop, profile)
	for _, p := range loop {
		if p.Y < 0 {
			return nil, fmt.Errorf("revolve profile point (%g, %g) has a negative radius", p.X, p.Y)
		}
	}
	if signedArea(loop) < 0 {
		reverse(loop)
	}

	// One ring of vertices per profile point, plus one axis point per profile
	// point that sits on the axis. A ring of radius zero would collapse to a
	// repeated vertex and NewMesh refuses a triangle that repeats one, so the
	// axis points are held separately and shared by the fan.
	var vertices []solidlens.Vec
	rings := make([]int, len(loop)) // first vertex index of each point's ring, or -1 on the axis
	axis := make([]int, len(loop))
	for i, p := range loop {
		if p.Y == 0 {
			rings[i] = -1
			axis[i] = len(vertices)
			vertices = append(vertices, solidlens.Vec{Z: p.X})
			continue
		}
		rings[i] = len(vertices)
		for j := range segments {
			a := 2 * math.Pi * float64(j) / float64(segments)
			vertices = append(vertices, solidlens.Vec{X: p.Y * math.Cos(a), Y: p.Y * math.Sin(a), Z: p.X})
		}
	}

	var triangles [][3]int
	at := func(i, j int) int { return rings[i] + j%segments }
	for i := range loop {
		k := (i + 1) % len(loop)
		switch {
		case rings[i] < 0 && rings[k] < 0:
			// Both ends on the axis: the edge sweeps nothing.
		case rings[i] < 0:
			for j := range segments {
				triangles = append(triangles, [3]int{axis[i], at(k, j), at(k, j+1)})
			}
		case rings[k] < 0:
			for j := range segments {
				triangles = append(triangles, [3]int{at(i, j), axis[k], at(i, j+1)})
			}
		default:
			// Wound edge-first then round: the cross product of the step from
			// point i to point k with the step from angle j to j+1 is the
			// outward normal when the profile turns counter-clockwise.
			for j := range segments {
				triangles = append(triangles,
					[3]int{at(i, j), at(k, j), at(k, j+1)},
					[3]int{at(i, j), at(k, j+1), at(i, j+1)})
			}
		}
	}
	return solidlens.NewMesh(vertices, triangles)
}

// Prism meshes the solid between two corresponding rings of the same length,
// walked the same way round, and closes both ends with ends.
//
// The rings do not have to be planar — the bevel tooth's are on cones — but
// the near ring's points must all sit on the far ring's near side along the
// axis the end faces are wound about, which for these examples is Z. ends is a
// triangulation of the section the rings were built from, in the near ring's
// own index space and wound counter-clockwise seen from +Z, which is what
// [EarClip] returns for a counter-clockwise section.
func Prism(near, far []solidlens.Vec, ends [][3]int) (*solidlens.Mesh, error) {
	n := len(near)
	if n < 3 {
		return nil, fmt.Errorf("prism needs at least three ring points, got %d", n)
	}
	if len(far) != n {
		return nil, fmt.Errorf("prism rings differ: %d near points and %d far", n, len(far))
	}
	vertices := make([]solidlens.Vec, 0, 2*n)
	vertices = append(vertices, near...)
	vertices = append(vertices, far...)
	lo := func(i int) int { return i % n }
	hi := func(i int) int { return n + i%n }

	triangles := make([][3]int, 0, 2*n+2*len(ends))
	for i := range n {
		triangles = append(triangles,
			[3]int{lo(i), lo(i + 1), hi(i + 1)},
			[3]int{lo(i), hi(i + 1), hi(i)})
	}
	for _, t := range ends {
		triangles = append(triangles,
			[3]int{lo(t[0]), lo(t[2]), lo(t[1])},
			[3]int{hi(t[0]), hi(t[1]), hi(t[2])})
	}
	return solidlens.NewMesh(vertices, triangles)
}

// Placed returns the mesh moved by t. It is how a part is staged: a gear is
// turned so the face worth seeing meets the camera, and the parts of an
// exploded assembly are slid apart along their own axis.
//
// A reflection is refused. It would reverse every triangle's winding and leave
// the whole mesh lit from inside, and no staging move here needs one.
func Placed(m solidlens.TriangleSource, t r3.Transform) (*solidlens.Mesh, error) {
	if t.IsReflection() {
		return nil, fmt.Errorf("placing a mesh by a reflection would reverse its winding")
	}
	vertices := m.Vertices()
	moved := make([]solidlens.Vec, len(vertices))
	for i, v := range vertices {
		moved[i] = t.Apply(v)
	}
	return solidlens.NewMesh(moved, m.Triangles())
}

// Merge joins meshes into one, which is how a patterned feature is handed over
// as a single model rather than as one model per copy.
func Merge(meshes ...solidlens.TriangleSource) (*solidlens.Mesh, error) {
	if len(meshes) == 0 {
		return nil, fmt.Errorf("merge needs at least one mesh")
	}
	var vertices []solidlens.Vec
	var triangles [][3]int
	for _, m := range meshes {
		base := len(vertices)
		vertices = append(vertices, m.Vertices()...)
		for _, t := range m.Triangles() {
			triangles = append(triangles, [3]int{base + t[0], base + t[1], base + t[2]})
		}
	}
	return solidlens.NewMesh(vertices, triangles)
}

// EarClip triangulates a simple polygon, returning triangles as indices into
// it, wound counter-clockwise.
//
// It is here because a gear tooth's section is not convex — its root arc bulges
// into the section — so a fan from any single point of it lays triangles
// outside the tooth. Ear clipping is quadratic in the point count, which is
// what a section of a few dozen points wants.
func EarClip(poly []Vec2) ([][3]int, error) {
	n := len(poly)
	if n < 3 {
		return nil, fmt.Errorf("ear clip needs at least three points, got %d", n)
	}
	idx := make([]int, n)
	for i := range idx {
		idx[i] = i
	}
	if signedArea(poly) < 0 {
		reverse(idx)
	}

	triangles := make([][3]int, 0, n-2)
	// Each pass must remove at least one ear; a simple polygon always has one,
	// so a pass that removes none means the input was not simple.
	for len(idx) > 3 {
		removed := false
		for i := range idx {
			prev := idx[(i+len(idx)-1)%len(idx)]
			curr := idx[i]
			next := idx[(i+1)%len(idx)]
			if !isEar(poly, idx, prev, curr, next) {
				continue
			}
			triangles = append(triangles, [3]int{prev, curr, next})
			idx = append(idx[:i], idx[i+1:]...)
			removed = true
			break
		}
		if !removed {
			return nil, fmt.Errorf("ear clip found no ear with %d points left; the polygon is not simple", len(idx))
		}
	}
	return append(triangles, [3]int{idx[0], idx[1], idx[2]}), nil
}

// isEar reports whether the corner at curr can be clipped: it turns the same
// way as the polygon and no other vertex lies inside the triangle it cuts off.
func isEar(poly []Vec2, idx []int, prev, curr, next int) bool {
	a, b, c := poly[prev], poly[curr], poly[next]
	if turn(a, b, c) <= 0 {
		return false
	}
	for _, j := range idx {
		if j == prev || j == curr || j == next {
			continue
		}
		if inTriangle(poly[j], a, b, c) {
			return false
		}
	}
	return true
}

// ClipRadius clips a closed profile to the half-plane R >= radius, which is
// how a bore is taken out of a solid of revolution before it is swept.
//
// It is Sutherland-Hodgman against one half-plane, so it is exact for a
// profile the line crosses twice — the only shape asked of it here — and a
// profile it would cut into two pieces comes back as one loop joined along the
// clip line.
func ClipRadius(profile []Vec2, radius float64) []Vec2 {
	if radius <= 0 {
		return profile
	}
	out := make([]Vec2, 0, len(profile)+2)
	for i, curr := range profile {
		prev := profile[(i+len(profile)-1)%len(profile)]
		currIn, prevIn := curr.Y >= radius, prev.Y >= radius
		if currIn != prevIn {
			t := (radius - prev.Y) / (curr.Y - prev.Y)
			out = append(out, Vec2{X: prev.X + t*(curr.X-prev.X), Y: radius})
		}
		if currIn {
			out = append(out, curr)
		}
	}
	return out
}

// signedArea is twice the polygon's signed area, positive counter-clockwise.
func signedArea(poly []Vec2) float64 {
	total := 0.0
	for i, p := range poly {
		q := poly[(i+1)%len(poly)]
		total += p.X*q.Y - q.X*p.Y
	}
	return total
}

// turn is twice the signed area of the triangle a, b, c, positive for a
// counter-clockwise turn.
func turn(a, b, c Vec2) float64 {
	return (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
}

// inTriangle reports whether p lies inside or on the counter-clockwise
// triangle a, b, c.
func inTriangle(p, a, b, c Vec2) bool {
	return turn(a, b, p) >= 0 && turn(b, c, p) >= 0 && turn(c, a, p) >= 0
}

func reverse[T any](s []T) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}
