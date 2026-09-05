// Package render_test checks the mesh builders the example images are drawn
// with.
//
// A picture cannot fail loudly: a mesh whose triangles are wound the wrong way
// round renders as a dark, inside-out shape that still writes a PNG, and one
// with a hole in it renders as a shape with a hole nobody notices at 320
// pixels. So the two properties every builder here has to hold are checked as
// numbers instead — the mesh is CLOSED, meaning every edge is shared by exactly
// two triangles in opposite directions, and it encloses a POSITIVE volume,
// which it does only when its normals face outward.
package render_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/solidlens"
	"github.com/stretchr/testify/require"
)

func TestRevolve(t *testing.T) {
	const (
		radius = 4.0
		height = 3.0
	)
	// A rectangle in (Z, R) whose R = 0 edge lies on the axis: a cylinder.
	cylinder := []render.Vec2{{X: 0, Y: 0}, {X: height, Y: 0}, {X: height, Y: radius}, {X: 0, Y: radius}}
	// The same rectangle with the axis edge pulled out to an inner radius: a
	// tube, which has no vertex on the axis at all.
	const inner = 1.5
	tube := []render.Vec2{{X: 0, Y: inner}, {X: height, Y: inner}, {X: height, Y: radius}, {X: 0, Y: radius}}
	// A triangle with one vertex on the axis at each end: a cone.
	cone := []render.Vec2{{X: 0, Y: 0}, {X: height, Y: 0}, {X: 0, Y: radius}}

	for _, tc := range []struct {
		name    string
		profile []render.Vec2
		volume  float64
	}{
		{"cylinder", cylinder, math.Pi * radius * radius * height},
		{"cylinder reversed", reversed(cylinder), math.Pi * radius * radius * height},
		{"tube", tube, math.Pi * (radius*radius - inner*inner) * height},
		{"cone", cone, math.Pi * radius * radius * height / 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const segments = 720
			mesh, err := render.Revolve(tc.profile, segments)
			require.NoError(t, err)
			requireClosed(t, mesh)
			// The mesh is the inscribed polygon's solid, short of the circle's
			// by the polygon's own area ratio, which is a part in a million at
			// this segment count.
			require.InEpsilon(t, tc.volume, volumeOf(mesh), 1e-4)
		})
	}
}

func TestRevolveRejects(t *testing.T) {
	square := []render.Vec2{{X: 0, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 2}, {X: 0, Y: 2}}
	_, err := render.Revolve(square[:2], 8)
	require.Error(t, err)
	_, err = render.Revolve(square, 2)
	require.Error(t, err)
	_, err = render.Revolve([]render.Vec2{{X: 0, Y: -1}, {X: 1, Y: 1}, {X: 1, Y: 2}}, 8)
	require.Error(t, err, "a negative radius is not a profile point")
}

func TestPrism(t *testing.T) {
	// An L, which is the simplest section a fan from one point cannot
	// triangulate: its reflex corner puts the fan's own triangles outside it.
	section := []render.Vec2{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 2}, {X: 0, Y: 2}}
	cap, err := render.EarClip(section)
	require.NoError(t, err)
	require.Len(t, cap, len(section)-2)

	const height = 5.0
	near := make([]solidlens.Vec, len(section))
	far := make([]solidlens.Vec, len(section))
	for i, p := range section {
		near[i] = solidlens.Vec{X: p.X, Y: p.Y}
		far[i] = solidlens.Vec{X: p.X, Y: p.Y, Z: height}
	}
	mesh, err := render.Prism(near, far, cap)
	require.NoError(t, err)
	requireClosed(t, mesh)
	require.InEpsilon(t, 3*height, volumeOf(mesh), 1e-12, "the L covers three unit squares")
}

// TestPrismSlantedEnds is the shape the bevel tooth actually is: the two rings
// are not planar, because each section point is trimmed at its own station.
func TestPrismSlantedEnds(t *testing.T) {
	section := []render.Vec2{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1}, {X: 0, Y: 1}}
	cap, err := render.EarClip(section)
	require.NoError(t, err)

	// Both ends slope with X at the same rate, so the solid is the unit prism
	// sheared along Z: shearing moves no volume.
	near := make([]solidlens.Vec, len(section))
	far := make([]solidlens.Vec, len(section))
	for i, p := range section {
		near[i] = solidlens.Vec{X: p.X, Y: p.Y, Z: 0.25 * p.X}
		far[i] = solidlens.Vec{X: p.X, Y: p.Y, Z: 3 + 0.25*p.X}
	}
	mesh, err := render.Prism(near, far, cap)
	require.NoError(t, err)
	requireClosed(t, mesh)
	require.InEpsilon(t, 2*1*3.0, volumeOf(mesh), 1e-12)
}

func TestEarClipRejectsDegenerate(t *testing.T) {
	_, err := render.EarClip([]render.Vec2{{X: 0, Y: 0}, {X: 1, Y: 0}})
	require.Error(t, err)
}

func TestClipRadius(t *testing.T) {
	// A rectangle from R = 0 to R = 4, clipped to R >= 1: the clip closes the
	// profile along the clip line, which is the bore wall.
	profile := []render.Vec2{{X: 0, Y: 0}, {X: 3, Y: 0}, {X: 3, Y: 4}, {X: 0, Y: 4}}
	clipped := render.ClipRadius(profile, 1)
	require.Len(t, clipped, 4)
	for _, p := range clipped {
		require.GreaterOrEqual(t, p.Y, 1.0)
	}

	mesh, err := render.Revolve(clipped, 720)
	require.NoError(t, err)
	requireClosed(t, mesh)
	require.InEpsilon(t, math.Pi*(4*4-1*1)*3, volumeOf(mesh), 1e-4)

	require.Equal(t, profile, render.ClipRadius(profile, 0), "a zero radius is no bore at all")
}

func TestMerge(t *testing.T) {
	profile := []render.Vec2{{X: 0, Y: 1}, {X: 2, Y: 1}, {X: 2, Y: 3}, {X: 0, Y: 3}}
	one, err := render.Revolve(profile, 180)
	require.NoError(t, err)
	merged, err := render.Merge(one, one)
	require.NoError(t, err)
	requireClosed(t, merged)
	require.InEpsilon(t, 2*volumeOf(one), volumeOf(merged), 1e-12)

	_, err = render.Merge()
	require.Error(t, err)
}

// requireClosed fails unless every edge of the mesh is used exactly twice, once
// in each direction. A hole leaves an edge used once; a triangle wound against
// its neighbour leaves one used twice the same way.
func requireClosed(t *testing.T, mesh *solidlens.Mesh) {
	t.Helper()
	type edge struct{ a, b int }
	used := map[edge]int{}
	for _, tri := range mesh.Triangles() {
		for i := range 3 {
			a, b := tri[i], tri[(i+1)%3]
			used[edge{a, b}]++
		}
	}
	for e, n := range used {
		require.Equalf(t, 1, n, "edge %d->%d is used %d times", e.a, e.b, n)
		require.Equalf(t, 1, used[edge{e.b, e.a}],
			"edge %d->%d has no opposite; the mesh is open or wound inconsistently", e.a, e.b)
	}
}

// volumeOf is the signed volume the mesh encloses, by the divergence theorem.
// It is positive only when the triangles face outward.
func volumeOf(mesh *solidlens.Mesh) float64 {
	vertices := mesh.Vertices()
	total := 0.0
	for _, tri := range mesh.Triangles() {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		total += a.Dot(b.Cross(c)) / 6
	}
	return total
}

func reversed(profile []render.Vec2) []render.Vec2 {
	out := make([]render.Vec2, len(profile))
	for i, p := range profile {
		out[len(profile)-1-i] = p
	}
	return out
}
