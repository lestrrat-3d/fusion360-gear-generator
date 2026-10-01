package screwgear_test

import (
	"fmt"
	"math"
	"slices"
	"sort"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// ---------------------------------------------------------------------------
// The sleeve's picture mesh.
//
// The pictures have no boolean to cut the sleeve with, so the sleeve is meshed
// from its own inside test, sleeve.inFrame, by dual contouring. The box round
// it is cut into cubes, and the surface's crossing of every cube edge whose
// ends differ is found by bisecting that edge against inFrame. Each cube the
// surface passes through gets one vertex, placed where the planes through its
// crossings meet, and the four cubes round each crossed edge are joined into
// a quad. Each crossing's plane is the face it lies on, taken from the pieces
// the sleeve is cut from: the tube's two cylinders and two end planes, each
// bore's four twisted walls, and each window's sides. Where two or three of
// those faces meet in a cube the vertex lands on the edge or the corner they
// make, which is what keeps the rims, the bore mouths and the window edges
// sharp. Marching cubes or tetrahedra can only put a vertex on a cube edge,
// and draw every edge of the part as a row of steps a cube wide.
//
// One vertex a cube cannot follow a wedge thinner than a cube, as where a
// bore's wall leaves a cylinder at a shallow angle, and the quads there can
// pinch or fold over. unpinch, flipSlivers and untangle tidy those places
// without moving any vertex off the surface.
//
// inFrame alone decides where the surface is. The faces only say which way it
// faces near a point inFrame has already put on it, and sleeveMeshFaults
// checks that every vertex they placed is on inFrame's boundary.
// ---------------------------------------------------------------------------

// sharpSolid is a solid with flat or smoothly curved faces meeting at sharp
// edges, as dualContour needs it.
type sharpSolid interface {
	// inside answers whether a point is in the solid. It decides where the
	// surface is.
	inside(pt r3.Vec) bool
	// face is the face a point on the surface lies on.
	face(pt r3.Vec) int
	// faceLevel is how far a point lies outside a face's surface, carried on
	// past the face's own edges, negative on the solid's side of it. It
	// changes at about one per mm near the surface.
	faceLevel(face int, pt r3.Vec) float64
	// level is the solid's own level: negative inside, positive outside,
	// and the distance to the nearest face when that is small.
	level(pt r3.Vec) float64
}

// faceNormal is the outward unit normal of a face's surface at a point.
func faceNormal(solid sharpSolid, face int, pt r3.Vec) r3.Vec {
	const eps = 1e-6
	diff := func(d r3.Vec) float64 { return solid.faceLevel(face, pt.Add(d)) - solid.faceLevel(face, pt.Sub(d)) }
	n, _ := r3.NewVec(diff(r3.NewVec(eps, 0, 0)), diff(r3.NewVec(0, eps, 0)), diff(r3.NewVec(0, 0, eps))).Normalize()
	return n
}

// sleeveSolid is the sleeve as dualContour sees it.
type sleeveSolid struct{ f sleeve }

func (s sleeveSolid) inside(pt r3.Vec) bool { return s.f.inFrame(pt) }

func (s sleeveSolid) face(pt r3.Vec) int {
	_, face := s.f.activeFace(pt)
	return face
}

// faceLevel turns a cut's face over, since the material is on its outside.
func (s sleeveSolid) faceLevel(face int, pt r3.Vec) float64 {
	if face < sleeveTubeFaces {
		return s.f.faceLevels(pt)[face]
	}
	return -s.f.faceLevels(pt)[face]
}

func (s sleeveSolid) level(pt r3.Vec) float64 {
	l, _ := s.f.activeFace(pt)
	return l
}

// sleeveFaceCount is the most faces faceLevels returns: four for the tube,
// four for each gear's pair of bores and seven for each of two windows.
const (
	sleeveTubeFaces = 4
	sleeveFaceCount = sleeveTubeFaces + 2*4 + 2*7
)

// faceLevels is, for every face the sleeve is cut from, how far a point lies
// on the outer side of that face's plane or surface, in the order: the tube's
// inner cylinder, outer cylinder, top and bottom; each gear's bore walls on u
// and on v and the cut's inner and outer ends, in the bore's own untwisted
// section; and each window's six sides and the plane through the frame's axis
// it starts from. A piece (the tube, one gear's bores, one window) holds a
// point when all its levels are at most zero, which is the test inShell,
// inSleeveChannel and sleeveWindow.contains make.
func (f sleeve) faceLevels(pt r3.Vec) [sleeveFaceCount]float64 {
	var out [sleeveFaceCount]float64
	r := math.Hypot(pt.X, pt.Y)
	out[0], out[1], out[2], out[3] = f.ri-r, r-f.ro, pt.Z-f.zb, -pt.Z-f.zb
	for gi, g := range f.gears {
		u, v, s := g.local(pt)
		k := sleeveTubeFaces + 4*gi
		out[k] = math.Abs(u) - g.P.BoreHalfWidth()
		out[k+1] = math.Abs(v) - g.P.BoreHalfThickness()
		out[k+2] = f.sIn - math.Abs(s)
		out[k+3] = math.Abs(s) - f.sOut
	}
	for wi, w := range f.windows {
		t, z, a := w.plane(pt)
		band, trim := z+w.lean*t, z-w.lean*t
		k := sleeveTubeFaces + 8 + 7*wi
		out[k] = (w.lo - band) / math.Sqrt2
		out[k+1] = (band - w.hi) / math.Sqrt2
		out[k+2] = (w.bottom - trim) / math.Sqrt2
		out[k+3] = (trim - w.top) / math.Sqrt2
		out[k+4] = w.left - t
		out[k+5] = t - w.right
		out[k+6] = -a
	}
	return out
}

// activeFace is the material's level at a point and the face that decides
// it. The material is the tube less the bores and the windows, so its level
// is the largest of the tube's level and each cut's level turned over, and a
// piece's level is its largest face's.
func (f sleeve) activeFace(pt r3.Vec) (float64, int) {
	levels := f.faceLevels(pt)
	best, face := math.Inf(-1), -1
	piece := func(from, to int, sign float64) {
		k := from
		for i := from + 1; i < to; i++ {
			if levels[i] > levels[k] {
				k = i
			}
		}
		if v := sign * levels[k]; v > best {
			best, face = v, k
		}
	}
	piece(0, sleeveTubeFaces, 1)
	for gi := range f.gears {
		from := sleeveTubeFaces + 4*gi
		piece(from, from+4, -1)
	}
	for wi := range f.windows {
		from := sleeveTubeFaces + 8 + 7*wi
		piece(from, from+7, -1)
	}
	return best, face
}

// sleeveSharpMesh meshes the sleeve on a grid of step h, in a box more than a
// cube clear of it all round. The odd 1.3 cubes keep the tube's end faces
// and the frame's axis off the grid's planes.
func sleeveSharpMesh(f sleeve, h float64) sharpMesh {
	pad := r3.NewVec(1.3*h, 1.3*h, 1.3*h)
	lo := r3.NewVec(-f.ro, -f.ro, -f.zb).Sub(pad)
	hi := r3.NewVec(f.ro, f.ro, f.zb).Add(pad)
	return dualContour(sleeveSolid{f}, lo, hi, h)
}

// sharpMesh is dualContour's result.
type sharpMesh struct {
	vertices  []r3.Vec
	triangles [][3]int
	// fallback counts the vertices whose cube's planes met too far outside
	// it, which were put on the surface near the crossings instead.
	fallback int
}

// qefCutoff is the smallest share of a cube's strongest plane direction that
// still counts as a direction of its own when the planes are solved. Below it
// the planes are taken as one face seen at slightly different angles, as a
// cylinder or a twisted wall is across one cube, rather than as an edge, and
// the vertex is left where the crossings' mean puts it along that direction.
// Two planes meeting at angle a give a share of tan^2(a/2), so 0.05 keeps
// every fold of 25 degrees or more.
const qefCutoff = 0.05

// dualContour meshes the surface of a solid inside the box from lo to hi on a
// grid of step h. The box's own faces must lie outside the solid.
func dualContour(solid sharpSolid, lo, hi r3.Vec, h float64) sharpMesh {
	nx := int(math.Ceil((hi.X-lo.X)/h)) + 1
	ny := int(math.Ceil((hi.Y-lo.Y)/h)) + 1
	nz := int(math.Ceil((hi.Z-lo.Z)/h)) + 1
	size := [3]int{nx, ny, nz}
	node := func(c [3]int) int { return (c[2]*ny+c[1])*nx + c[0] }
	coords := func(n int) [3]int { return [3]int{n % nx, (n / nx) % ny, n / (nx * ny)} }
	pos := func(c [3]int) r3.Vec {
		return r3.NewVec(lo.X+float64(c[0])*h, lo.Y+float64(c[1])*h, lo.Z+float64(c[2])*h)
	}

	in := make([]bool, nx*ny*nz)
	var wg sync.WaitGroup
	for k := range nz {
		wg.Go(func() {
			for j := range ny {
				for i := range nx {
					c := [3]int{i, j, k}
					in[node(c)] = solid.inside(pos(c))
				}
			}
		})
	}
	wg.Wait()

	// Every grid edge the surface crosses, keyed by its lower node times
	// three plus its axis, with where it crosses and the face it crosses.
	type crossing struct {
		key, face int
		p, n      r3.Vec
	}
	slabs := make([][]crossing, nz)
	for k := range nz {
		wg.Go(func() {
			for j := range ny {
				for i := range nx {
					a := [3]int{i, j, k}
					for axis := range 3 {
						b := a
						b[axis]++
						if b[axis] >= size[axis] || in[node(a)] == in[node(b)] {
							continue
						}
						pIn, pOut := pos(a), pos(b)
						if !in[node(a)] {
							pIn, pOut = pOut, pIn
						}
						for range 20 {
							mid := pIn.Add(pOut).Scale(0.5)
							if solid.inside(mid) {
								pIn = mid
							} else {
								pOut = mid
							}
						}
						p := pIn.Add(pOut).Scale(0.5)
						face := solid.face(p)
						slabs[k] = append(slabs[k], crossing{
							key: node(a)*3 + axis, face: face, p: p, n: faceNormal(solid, face, p),
						})
					}
				}
			}
		})
	}
	wg.Wait()
	var crossings []crossing
	for _, s := range slabs {
		crossings = append(crossings, s...)
	}
	edgeAt := make(map[int]int, len(crossings))
	for i, c := range crossings {
		edgeAt[c.key] = i
	}

	// The four cubes round a crossed edge, in turn about its axis so that
	// their vertices wind counterclockwise seen from the edge's upper end.
	around := func(key int) [4][3]int {
		c, axis := coords(key/3), key%3
		b, d := (axis+1)%3, (axis+2)%3
		var out [4][3]int
		for q, o := range [4][2]int{{-1, -1}, {0, -1}, {0, 0}, {-1, 0}} {
			out[q] = c
			out[q][b] += o[0]
			out[q][d] += o[1]
		}
		return out
	}

	// One vertex per cube the surface passes through.
	cellAt := map[int]int{}
	var cells [][3]int
	for _, c := range crossings {
		for _, cell := range around(c.key) {
			if _, ok := cellAt[node(cell)]; !ok {
				cellAt[node(cell)] = len(cells)
				cells = append(cells, cell)
			}
		}
	}
	vertices := make([]r3.Vec, len(cells))
	fellBack := make([]bool, len(cells))
	const chunk = 4096
	for start := 0; start < len(cells); start += chunk {
		wg.Go(func() {
			var ps, ns []r3.Vec
			var faces []int
			for ci := start; ci < min(start+chunk, len(cells)); ci++ {
				cell := cells[ci]
				ps, ns, faces = ps[:0], ns[:0], faces[:0]
				for axis := range 3 {
					b, d := (axis+1)%3, (axis+2)%3
					for _, o := range [4][2]int{{0, 0}, {1, 0}, {0, 1}, {1, 1}} {
						e := cell
						e[b] += o[0]
						e[d] += o[1]
						if at, ok := edgeAt[node(e)*3+axis]; ok {
							ps = append(ps, crossings[at].p)
							ns = append(ns, crossings[at].n)
							faces = append(faces, crossings[at].face)
						}
					}
				}
				var c r3.Vec
				for _, p := range ps {
					c = c.Add(p)
				}
				c = c.Scale(1 / float64(len(ps)))
				// The planes may meet outside the cube when an edge of the
				// part runs just past it, and that point is still on the part.
				const margin = 1
				low := pos(cell).Sub(r3.NewVec(margin*h, margin*h, margin*h))
				high := pos(cell).Add(r3.NewVec((1+margin)*h, (1+margin)*h, (1+margin)*h))
				fits := func(v r3.Vec) bool {
					return v.X >= low.X && v.Y >= low.Y && v.Z >= low.Z && v.X <= high.X && v.Y <= high.Y && v.Z <= high.Z
				}
				v, full := qefVertex(c, ps, ns, fits)
				if full {
					if w := refineVertex(solid, v, c, faces); fits(w) {
						vertices[ci] = w
						continue
					}
				}
				// Further out than one cube, the planes are solved again
				// pinning one direction fewer, down to the crossings' mean,
				// and the point is moved onto the surface along the
				// crossings' mean normal, or failing that, to the crossing
				// nearest it.
				fellBack[ci] = true
				var sum r3.Vec
				for _, n := range ns {
					sum = sum.Add(n)
				}
				n, ok := sum.Normalize()
				if ok {
					v, ok = settle(solid, v, n, h)
				}
				if !ok {
					near := ps[0]
					for _, p := range ps[1:] {
						if p.Sub(v).Len() < near.Sub(v).Len() {
							near = p
						}
					}
					v = near
				}
				vertices[ci] = v
			}
		})
	}
	wg.Wait()

	// One quad per crossed edge, joining its four cubes' vertices, wound so
	// that it faces from the edge's inside end toward its outside end, and
	// cut into two triangles across whichever diagonal lies nearer the
	// surface: the one that runs along a sharp edge rather than across it.
	triangles := make([][3]int, 0, 2*len(crossings))
	for _, c := range crossings {
		var q [4]int
		for i, cell := range around(c.key) {
			q[i] = cellAt[node(cell)]
		}
		if !in[c.key/3] {
			q[1], q[3] = q[3], q[1]
		}
		mid := func(a, b int) float64 { return math.Abs(solid.level(vertices[a].Add(vertices[b]).Scale(0.5))) }
		if mid(q[0], q[2]) <= mid(q[1], q[3]) {
			triangles = append(triangles, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		} else {
			triangles = append(triangles, [3]int{q[0], q[1], q[3]}, [3]int{q[1], q[2], q[3]})
		}
	}

	unpinch(solid, vertices, triangles)
	flipSlivers(solid, vertices, triangles)
	untangle(solid, vertices, triangles, h)
	m := sharpMesh{vertices: vertices, triangles: triangles}
	for _, b := range fellBack {
		if b {
			m.fallback++
		}
	}
	return m
}

// unpinch undoes the pinches dual contouring leaves. Where an edge of the
// part crosses a face of a grid cube corner to corner, all four of that
// face's grid edges are crossed, and the two cubes either side of it share
// four quads, so four triangles meet along the edge joining their vertices.
// Of those four, unpinch takes one running the edge
// each way and turns their shared edge to the other diagonal of the quad the
// two make, choosing the pair whose new edge lies nearest the surface. The
// edge is then shared by two triangles, as every other edge is, and the
// renderer no longer draws it as a tick across the part's edge.
func unpinch(solid sharpSolid, vertices []r3.Vec, triangles [][3]int) {
	type edge struct{ a, b int }
	uses := make(map[edge][]int, 3*len(triangles))
	add := func(ti int) {
		t := triangles[ti]
		for k := range 3 {
			e := edge{t[k], t[(k+1)%3]}
			uses[e] = append(uses[e], ti)
		}
	}
	drop := func(ti int) {
		t := triangles[ti]
		for k := range 3 {
			e := edge{t[k], t[(k+1)%3]}
			uses[e] = slices.DeleteFunc(uses[e], func(x int) bool { return x == ti })
		}
	}
	for ti := range triangles {
		add(ti)
	}
	// opposite is the corner of a triangle across its edge from a to c.
	opposite := func(t [3]int, a, c int) int {
		for k := range 3 {
			if t[k] == a && t[(k+1)%3] == c {
				return t[(k+2)%3]
			}
		}
		return -1
	}
	gap := func(a, b int) float64 { return math.Abs(solid.level(vertices[a].Add(vertices[b]).Scale(0.5))) }
	var pinched []edge
	for e, ts := range uses {
		if len(ts) > 1 && e.a < e.b {
			pinched = append(pinched, e)
		}
	}
	slices.SortFunc(pinched, func(x, y edge) int {
		if x.a != y.a {
			return x.a - y.a
		}
		return x.b - y.b
	})
	for _, e := range pinched {
		a, c := e.a, e.b
		bestI, bestJ, best := -1, -1, math.Inf(1)
		for _, ti := range uses[edge{a, c}] {
			for _, tj := range uses[edge{c, a}] {
				b, d := opposite(triangles[ti], a, c), opposite(triangles[tj], c, a)
				if b == d || len(uses[edge{b, d}]) > 0 || len(uses[edge{d, b}]) > 0 {
					continue
				}
				if g := gap(b, d); g < best {
					bestI, bestJ, best = ti, tj, g
				}
			}
		}
		if bestI < 0 {
			continue
		}
		b, d := opposite(triangles[bestI], a, c), opposite(triangles[bestJ], c, a)
		drop(bestI)
		drop(bestJ)
		triangles[bestI], triangles[bestJ] = [3]int{a, d, b}, [3]int{d, c, b}
		add(bestI)
		add(bestJ)
	}
}

// sliverShape is the shape below which flipSlivers counts a triangle a
// sliver: its area over its longest edge squared, which is 0.43 for an
// equilateral triangle and 0 for three points on a line.
const sliverShape = 0.05

// shape is a triangle's area over its longest edge squared.
func shape(a, b, c r3.Vec) float64 {
	longest := math.Max(b.Sub(a).Len(), math.Max(c.Sub(b).Len(), a.Sub(c).Len()))
	if longest == 0 {
		return 0
	}
	return b.Sub(a).Cross(c.Sub(a)).Len() / 2 / (longest * longest)
}

// flipSlivers turns the long edge of every sliver to the other diagonal of
// the quad it makes with its neighbour, where that gives two better shaped
// triangles facing the same way the pair did and the new edge lies no further
// from the surface than the old one, or than a curved face's chord would: a
// fiftieth of its length. An edge cut across a sharp edge of the part stands
// off it by a quarter of its length or more. It turns an edge of a triangle
// that faces against the part's face under it the same way, where both new
// triangles then face the right way.
//
// Along a sharp edge of the part, three cubes in a row can all put their
// vertex on that edge, and the triangle joining them is a sliver lying on the
// edge whose normal points anywhere. The renderer draws its outline as a
// crease, which shows as a tick off the edge. Its neighbour across its long
// edge reaches off the edge onto a face, and the other diagonal of the two
// lies on that face. A flip keeps every vertex, every other triangle and the
// mesh closed.
func flipSlivers(solid sharpSolid, vertices []r3.Vec, triangles [][3]int) {
	type edge struct{ a, b int }
	owner := make(map[edge]int, 3*len(triangles))
	for ti, t := range triangles {
		for i := range 3 {
			owner[edge{t[i], t[(i+1)%3]}] = ti
		}
	}
	at := func(t [3]int) (r3.Vec, r3.Vec, r3.Vec) { return vertices[t[0]], vertices[t[1]], vertices[t[2]] }
	quality := func(t [3]int) float64 { return shape(at(t)) }
	normal := func(t [3]int) r3.Vec {
		a, b, c := at(t)
		return b.Sub(a).Cross(c.Sub(a))
	}
	gap := func(a, b int) float64 { return math.Abs(solid.level(vertices[a].Add(vertices[b]).Scale(0.5))) }
	faces := func(t [3]int) float64 { return facing(solid, vertices[t[0]], vertices[t[1]], vertices[t[2]]) }
	// flip turns the edge from t's corner i to the next to the other
	// diagonal, when better says the new pair is better than the old.
	flip := func(ti, i int, better func(t, u, n1, n2 [3]int, a, b, c, d int) bool) bool {
		t := triangles[ti]
		a, c, b := t[i], t[(i+1)%3], t[(i+2)%3]
		tj, ok := owner[edge{c, a}]
		if !ok {
			return false
		}
		u := triangles[tj]
		d := u[0] + u[1] + u[2] - a - c
		if d == b {
			return false
		}
		if _, exists := owner[edge{b, d}]; exists {
			return false
		}
		n1, n2 := [3]int{a, d, b}, [3]int{d, c, b}
		if !better(t, u, n1, n2, a, b, c, d) {
			return false
		}
		delete(owner, edge{a, c})
		delete(owner, edge{c, a})
		triangles[ti], triangles[tj] = n1, n2
		for _, x := range []int{ti, tj} {
			for k := range 3 {
				owner[edge{triangles[x][k], triangles[x][(k+1)%3]}] = x
			}
		}
		return true
	}
	sliver := func(t, u, n1, n2 [3]int, a, b, c, d int) bool {
		before := normal(t).Add(normal(u))
		if normal(n1).Dot(before) <= 0 || normal(n2).Dot(before) <= 0 {
			return false
		}
		if math.Min(quality(n1), quality(n2)) <= math.Min(quality(t), quality(u)) {
			return false
		}
		return gap(b, d) <= math.Max(gap(a, c), 0.02*vertices[b].Sub(vertices[d]).Len())
	}
	// unfold is for a triangle turned over against the face under it: both
	// new triangles have to face the right way, and the new edge must not be
	// cut across a sharp edge of the part.
	unfold := func(t, u, n1, n2 [3]int, a, b, c, d int) bool {
		if gap(b, d) > math.Max(gap(a, c), 0.1*vertices[b].Sub(vertices[d]).Len()) {
			return false
		}
		return faces(n1) > 0 && faces(n2) > 0
	}
	for range 8 {
		flipped := false
		turned := facings(solid, vertices, triangles)
		for ti := range triangles {
			t := triangles[ti]
			if quality(t) < sliverShape {
				// The long edge runs from corner i to the next.
				i := 0
				for k := 1; k < 3; k++ {
					if vertices[t[k]].Sub(vertices[t[(k+1)%3]]).Len() > vertices[t[i]].Sub(vertices[t[(i+1)%3]]).Len() {
						i = k
					}
				}
				if flip(ti, i, sliver) {
					flipped = true
					continue
				}
			}
			if turned[ti] < 0 && faces(triangles[ti]) < 0 {
				for i := range 3 {
					if flip(ti, i, unfold) {
						flipped = true
						break
					}
				}
			}
		}
		if !flipped {
			return
		}
	}
}

// facing is how far a triangle faces the way the part's face under its
// middle does: 1 when it lies along that face, negative when it is turned
// over.
func facing(solid sharpSolid, a, b, c r3.Vec) float64 {
	n, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
	if !ok {
		return -1
	}
	mid := a.Add(b).Add(c).Scale(1.0 / 3)
	return n.Dot(faceNormal(solid, solid.face(mid), mid))
}

// facings is facing for every triangle.
func facings(solid sharpSolid, vertices []r3.Vec, triangles [][3]int) []float64 {
	out := make([]float64, len(triangles))
	var wg sync.WaitGroup
	const chunk = 4096
	for start := 0; start < len(triangles); start += chunk {
		wg.Go(func() {
			for ti := start; ti < min(start+chunk, len(triangles)); ti++ {
				t := triangles[ti]
				out[ti] = facing(solid, vertices[t[0]], vertices[t[1]], vertices[t[2]])
			}
		})
	}
	wg.Wait()
	return out
}

// snap moves a point near a solid's surface onto the face nearest it.
func snap(solid sharpSolid, v r3.Vec) r3.Vec {
	for range 4 {
		if math.Abs(solid.level(v)) < 1e-9 {
			break
		}
		face := solid.face(v)
		v = v.Sub(faceNormal(solid, face, v).Scale(solid.faceLevel(face, v)))
	}
	return v
}

// untangle moves the corners of triangles still turned over against the
// face under them, one at a time, to the middle of their neighbours and back
// onto the surface, keeping a move only where it leaves fewer triangles
// round that corner turned over than before.
//
// Where an edge of the part meets the grid at a shallow angle, or two faces
// meet at a narrow one, a vertex solved onto that edge can land past its
// neighbour's along it, and the triangles between them fold over onto each
// other. The renderer outlines the fold as a short tick. A vertex moved off
// the edge this way still lies on the surface: it is put back onto it by
// bisecting against the solid's inside test.
func untangle(solid sharpSolid, vertices []r3.Vec, triangles [][3]int, h float64) {
	fan := make([][]int, len(vertices))
	for ti, t := range triangles {
		for _, v := range t {
			fan[v] = append(fan[v], ti)
		}
	}
	turned := func(v int) int {
		n := 0
		for _, ti := range fan[v] {
			t := triangles[ti]
			if facing(solid, vertices[t[0]], vertices[t[1]], vertices[t[2]]) < 0 {
				n++
			}
		}
		return n
	}
	for range 4 {
		var suspects []int
		for ti, f := range facings(solid, vertices, triangles) {
			if f < 0 {
				suspects = append(suspects, triangles[ti][:]...)
			}
		}
		if len(suspects) == 0 {
			return
		}
		slices.Sort(suspects)
		improved := false
		for _, v := range slices.Compact(suspects) {
			before := turned(v)
			if before == 0 {
				continue
			}
			var mid, normal r3.Vec
			count := 0
			for _, ti := range fan[v] {
				t := triangles[ti]
				a, b, c := vertices[t[0]], vertices[t[1]], vertices[t[2]]
				if facing(solid, a, b, c) > 0 {
					normal = normal.Add(b.Sub(a).Cross(c.Sub(a)))
				}
				for _, w := range t {
					if w != v {
						mid = mid.Add(vertices[w])
						count++
					}
				}
			}
			n, ok := normal.Normalize()
			if !ok {
				continue
			}
			p, ok := settle(solid, mid.Scale(1/float64(count)), n, h)
			if !ok {
				continue
			}
			old := vertices[v]
			vertices[v] = p
			if turned(v) >= before {
				vertices[v] = old
				continue
			}
			improved = true
		}
		if !improved {
			return
		}
	}
}

// refineVertex solves a cube's planes again with one plane for each face
// the cube's crossings lie on, taken where that face passes nearest the
// vertex rather than at the crossings, a few times over. The crossings can be
// most of a cube from the edge or the corner the vertex sits on, and a
// cylinder or a twisted wall turns enough over that distance to move the
// point where their planes meet off the part. One plane a face also keeps a
// face crossed at many grid edges from outweighing one crossed at few, which
// would otherwise drop a corner's third direction as too weak to count.
func refineVertex(solid sharpSolid, v, c r3.Vec, faces []int) r3.Vec {
	var unique []int
	for _, face := range faces {
		if !slices.Contains(unique, face) {
			unique = append(unique, face)
		}
	}
	ps := make([]r3.Vec, len(unique))
	ns := make([]r3.Vec, len(unique))
	for range 4 {
		for i, face := range unique {
			n := faceNormal(solid, face, v)
			ps[i], ns[i] = v.Sub(n.Scale(solid.faceLevel(face, v))), n
		}
		w, full := qefVertex(c, ps, ns, func(r3.Vec) bool { return true })
		if !full {
			break
		}
		v = w
	}
	// Where the faces' planes do not all meet near the cube, as where three
	// faces cross it in pairs rather than at one corner, the point solved
	// for can be off the part. It is then moved onto the face nearest it.
	return snap(solid, v)
}

// qefVertex is the point nearest, in the least-squares sense, to every plane
// through a point of ps square to the matching normal, starting from c and
// moving only along the directions the planes pin down (see qefCutoff). When
// that point does not fit, it drops the least firmly pinned direction and
// tries again, down to c itself, and it reports whether it kept every
// direction.
func qefVertex(c r3.Vec, ps, ns []r3.Vec, fits func(r3.Vec) bool) (r3.Vec, bool) {
	var a [3][3]float64
	var b [3]float64
	for i, n := range ns {
		v := [3]float64{n.X, n.Y, n.Z}
		d := n.Dot(ps[i].Sub(c))
		for r := range 3 {
			b[r] += v[r] * d
			for s := range 3 {
				a[r][s] += v[r] * v[s]
			}
		}
	}
	values, vectors := symmetricEigen(a)
	order := []int{0, 1, 2}
	sort.Slice(order, func(i, j int) bool { return values[order[i]] > values[order[j]] })
	keep := 0
	for _, e := range order {
		if values[e] > qefCutoff*values[order[0]] {
			keep++
		}
	}
	for rank := keep; rank > 0; rank-- {
		var x [3]float64
		for _, e := range order[:rank] {
			proj := (vectors[0][e]*b[0] + vectors[1][e]*b[1] + vectors[2][e]*b[2]) / values[e]
			for r := range 3 {
				x[r] += proj * vectors[r][e]
			}
		}
		if v := c.Add(r3.NewVec(x[0], x[1], x[2])); fits(v) {
			return v, rank == keep
		}
	}
	return c, keep == 0
}

// settle moves a point onto a solid's surface along a direction, and
// reports whether the surface crosses that line within reach either side of
// the point.
func settle(solid sharpSolid, v, n r3.Vec, reach float64) (r3.Vec, bool) {
	pIn, pOut := v.Sub(n.Scale(reach)), v.Add(n.Scale(reach))
	if !solid.inside(pIn) || solid.inside(pOut) {
		return v, false
	}
	for range 30 {
		mid := pIn.Add(pOut).Scale(0.5)
		if solid.inside(mid) {
			pIn = mid
		} else {
			pOut = mid
		}
	}
	return pIn.Add(pOut).Scale(0.5), true
}

// symmetricEigen is the eigenvalues of a symmetric 3x3 matrix and its unit
// eigenvectors as the columns of the second result, by Jacobi rotations.
func symmetricEigen(a [3][3]float64) ([3]float64, [3][3]float64) {
	v := [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	for range 32 {
		off := a[0][1]*a[0][1] + a[0][2]*a[0][2] + a[1][2]*a[1][2]
		if off < 1e-30 {
			break
		}
		for _, pq := range [3][2]int{{0, 1}, {0, 2}, {1, 2}} {
			p, q := pq[0], pq[1]
			if math.Abs(a[p][q]) < 1e-300 {
				continue
			}
			theta := (a[q][q] - a[p][p]) / (2 * a[p][q])
			t := math.Copysign(1, theta) / (math.Abs(theta) + math.Sqrt(theta*theta+1))
			c := 1 / math.Sqrt(t*t+1)
			s := t * c
			for k := range 3 {
				akp, akq := a[k][p], a[k][q]
				a[k][p], a[k][q] = c*akp-s*akq, s*akp+c*akq
			}
			for k := range 3 {
				apk, aqk := a[p][k], a[q][k]
				a[p][k], a[q][k] = c*apk-s*aqk, s*apk+c*aqk
			}
			for k := range 3 {
				vkp, vkq := v[k][p], v[k][q]
				v[k][p], v[k][q] = c*vkp-s*vkq, s*vkp+c*vkq
			}
		}
	}
	return [3]float64{a[0][0], a[1][1], a[2][2]}, v
}

// sharpMeshCheck is what sleeveMeshFaults found.
type sharpMeshCheck struct {
	vertices   int
	offSurface int    // vertices with no point inside and no point outside within the tolerance
	worst      r3.Vec // one of them
	open       int    // edges run more times one way than the other
	pinched    int    // edges run twice each way
	flat       int    // triangles of no area
}

func (c sharpMeshCheck) String() string {
	return fmt.Sprintf("%d of %d vertices off the surface (one at %v), %d open edges, %d pinched edges, %d flat triangles",
		c.offSurface, c.vertices, c.worst, c.open, c.pinched, c.flat)
}

// sleeveMeshFaults checks that a mesh draws a solid's surface.
//
// Every vertex has to have a point inside the solid and a point outside it
// within delta, so that the surface passes within delta of it. The points
// tried are delta either way along the vertex's normal, along each of its
// triangles' normals, halfway between each two of those, and toward each
// corner, edge and face of a cube round the vertex. On a sharp edge the
// vertex's own normal leans toward one face, and the way into the solid is
// between the two faces' normals, which a vertex need not have triangles on
// both of.
//
// Every edge has to be run once each way, by two triangles, so that the mesh
// has no hole, is wound one way throughout and has no pinch, where four
// triangles meet along one edge (see unpinch). The renderer draws a pinch,
// like a hole's border, whatever the angle at it.
func sleeveMeshFaults(solid sharpSolid, m sharpMesh, delta float64) sharpMeshCheck {
	out := sharpMeshCheck{vertices: len(m.vertices)}
	normals := make([]r3.Vec, len(m.vertices))
	faces := make([][]r3.Vec, len(m.vertices))
	type edge struct{ a, b int }
	uses := map[edge]int{}
	for _, t := range m.triangles {
		a, b, c := m.vertices[t[0]], m.vertices[t[1]], m.vertices[t[2]]
		n := b.Sub(a).Cross(c.Sub(a))
		unit, ok := n.Normalize()
		if !ok || n.Len() < 1e-12 {
			out.flat++
		}
		for i := range 3 {
			normals[t[i]] = normals[t[i]].Add(n)
			if ok {
				faces[t[i]] = append(faces[t[i]], unit)
			}
			uses[edge{t[i], t[(i+1)%3]}]++
		}
	}
	for e, n := range uses {
		switch back := uses[edge{e.b, e.a}]; {
		case n != back:
			out.open++
		case n > 1 && e.a < e.b:
			out.pinched++
		}
	}
	var dirs []r3.Vec
	for _, d := range [][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {1, -1, 0}, {0, 1, 1}, {0, 1, -1},
		{1, 0, 1}, {1, 0, -1}, {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}} {
		n, _ := r3.NewVec(d[0], d[1], d[2]).Normalize()
		dirs = append(dirs, n)
	}
	var mu sync.Mutex
	var wg sync.WaitGroup
	const chunk = 4096
	for start := 0; start < len(m.vertices); start += chunk {
		wg.Go(func() {
			for i := start; i < min(start+chunk, len(m.vertices)); i++ {
				v := m.vertices[i]
				try := append(append([]r3.Vec{}, faces[i]...), dirs...)
				if n, ok := normals[i].Normalize(); ok {
					try = append(try, n)
				}
				for a, na := range faces[i] {
					for _, nb := range faces[i][a+1:] {
						if n, ok := na.Add(nb).Normalize(); ok {
							try = append(try, n)
						}
					}
				}
				var hasIn, hasOut bool
				for _, d := range try {
					for _, sign := range [2]float64{1, -1} {
						if solid.inside(v.Add(d.Scale(sign * delta))) {
							hasIn = true
						} else {
							hasOut = true
						}
					}
				}
				if hasIn && hasOut {
					continue
				}
				mu.Lock()
				out.offSurface++
				out.worst = v
				mu.Unlock()
			}
		})
	}
	wg.Wait()
	return out
}

// sleeveMeshTolerance is how near the surface sleeveMeshFaults asks every
// vertex of the sleeve's mesh to be, in mm.
const sleeveMeshTolerance = 0.001

// checkSleeveMesh fails the test unless the mesh draws the set inFrame
// describes and is closed.
func checkSleeveMesh(t *testing.T, f sleeve, m sharpMesh) {
	t.Helper()
	got := sleeveMeshFaults(sleeveSolid{f}, m, sleeveMeshTolerance)
	if got.offSurface != 0 || got.open != 0 || got.pinched != 0 || got.flat != 0 {
		t.Fatalf("the sleeve's mesh at %g mm tolerance: %v", sleeveMeshTolerance, got)
	}
	t.Logf("the sleeve's mesh: %d vertices, %d triangles, %d vertices placed by the fallback; %v",
		len(m.vertices), len(m.triangles), m.fallback, got)
}

// The mesh the sleeve's pictures are drawn from draws the set inFrame
// describes. This runs it on a coarse grid so that it runs with the proofs;
// TestRenderSleeve checks the pictures' own mesh the same way.
func TestSleeveMeshDrawsTheFrame(t *testing.T) {
	f := defaultSleeve()
	checkSleeveMesh(t, f, sleeveSharpMesh(f, 0.5))
}
