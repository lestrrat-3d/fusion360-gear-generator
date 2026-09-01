package main

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/solidlens"
	"github.com/stretchr/testify/require"
)

// TestSweptMeshIsClosed requires every edge of a swept gear to be shared by
// exactly two triangles, and no triangle to be collapsed.
//
// The renderer draws a line along any edge that is not shared by two faces,
// so a hole in the mesh does not show up as a hole. It shows up as a stray
// line across a flat face, which is easy to mistake for a styling choice.
func TestSweptMeshIsClosed(t *testing.T) {
	for _, ex := range examples() {
		t.Run(ex.gear.name, func(t *testing.T) {
			dims := involute.Derive(ex.gear.module, float64(ex.gear.teeth), ex.gear.pressureAngle)
			mesh, err := sweep(dims, ex.gear.teeth, ex.gear.boreDiameter/2, ex.gear.stations())
			require.NoError(t, err)

			vertices := mesh.Vertices()
			collapsed := 0
			counts := map[[2][3]int64]int{}
			for _, tri := range mesh.Triangles() {
				corners := [3]solidlens.Vec{vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]}
				if _, ok := corners[1].Sub(corners[0]).Cross(corners[2].Sub(corners[0])).Normalize(); !ok {
					collapsed++
					continue
				}
				for i, a := range corners {
					counts[edgeKey(a, corners[(i+1)%3])]++
				}
			}
			require.Zero(t, collapsed, "collapsed triangles drop out of the mesh and open a hole around themselves")

			unshared := 0
			for _, n := range counts {
				if n != 2 {
					unshared++
				}
			}
			require.Zero(t, unshared, "edges not shared by exactly two triangles")

			require.Zero(t, inwardCaps(mesh, ex.gear.thickness),
				"end-cap triangles facing into the gear; the renderer lines every edge where two faces disagree")
		})
	}
}

// edgeKey identifies an edge by its endpoints, quantized and ordered the way
// the renderer's own edge collector does it.
func edgeKey(a, b solidlens.Vec) [2][3]int64 {
	lo, hi := quantize(a), quantize(b)
	if hi[0] < lo[0] ||
		(hi[0] == lo[0] && hi[1] < lo[1]) ||
		(hi[0] == lo[0] && hi[1] == lo[1] && hi[2] < lo[2]) {
		lo, hi = hi, lo
	}
	return [2][3]int64{lo, hi}
}

func quantize(v solidlens.Vec) [3]int64 {
	const scale = 1e5
	return [3]int64{
		int64(math.Round(v.X * scale)),
		int64(math.Round(v.Y * scale)),
		int64(math.Round(v.Z * scale)),
	}
}

// inwardCaps counts the flat end-cap triangles whose normal points into the
// gear instead of out of it.
func inwardCaps(mesh *solidlens.Mesh, thickness float64) int {
	vertices := mesh.Vertices()
	wrong := 0
	for _, tri := range mesh.Triangles() {
		corners := [3]solidlens.Vec{vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]}
		normal, ok := corners[1].Sub(corners[0]).Cross(corners[2].Sub(corners[0])).Normalize()
		if !ok || math.Abs(math.Abs(normal.Z)-1) > 1e-9 {
			continue
		}
		atTop := math.Abs(corners[0].Z-thickness) < 1e-9
		if atTop != (normal.Z > 0) {
			wrong++
		}
	}
	return wrong
}
