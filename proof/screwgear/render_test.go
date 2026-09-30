package screwgear_test

import (
	"flag"
	"fmt"
	"math"
	"path/filepath"
	"sync"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// Pictures of the screw gearing.
//
// These are not proofs and they are skipped unless -render.out names a
// directory. They live in the proof's own package because the alternative is a
// second description of the same part: every section drawn here is Gear.section,
// the same function the mesh proof samples, at the same defaults. A change that
// moves the proved geometry moves the pictures with it.
//
// The pair is drawn in the arrangement TestPairDrivesOneToOne passes at: gear B
// at the assembly phase, both gears mounted at the same angle, at the crossing
// angle the spec's default table gives.
// ---------------------------------------------------------------------------

var renderOut = flag.String("render.out", "",
	"directory the example images are written to; the render is skipped when it is empty")

var renderSettings = solidlens.Settings{Width: 1100, Height: 820}

// renderStations is how many cross-sections are meshed per tooth. The tooth is
// a cosine, so this is what decides whether a crest reads as a crest; sixteen
// puts a section every 22 degrees of the wave. It is a drawing resolution and
// has nothing to do with the sections the spec lofts a tooth from, which are
// ten steps to the tooth at the defaults, 41 sections in a four-tooth cell.
const renderStations = 16

var (
	gearAColor = solidlens.RGB(0.29, 0.66, 0.72)
	gearBColor = solidlens.RGB(0.86, 0.56, 0.24)
	cageColor  = solidlens.RGB(0.80, 0.80, 0.83)
)

// ribbonMesh meshes one gear over the station range given: every cross-section
// in one vertex list, with the four side faces walked across neighbouring
// sections and a cap at each end.
//
// The sections share their vertices rather than each band being its own prism.
// A merged chain of prisms leaves every band boundary a duplicated vertex pair,
// which the renderer cannot tell from a real corner, and the ribbon comes out
// hatched with one outline per band.
func ribbonMesh(g Gear, from, to float64) (*solidlens.Mesh, error) {
	step := g.P.ToothPitch / renderStations
	count := int(math.Round((to - from) / step))
	if count < 1 {
		return nil, fmt.Errorf("a ribbon from %g to %g holds no section", from, to)
	}

	const corners = 4
	vertices := make([]solidlens.Vec, 0, corners*(count+1))
	for i := 0; i <= count; i++ {
		for _, p := range g.section(from + float64(i)*step) {
			vertices = append(vertices, solidlens.Vec{X: p.X, Y: p.Y, Z: p.Z})
		}
	}
	at := func(station, corner int) int { return station*corners + corner%corners }

	triangles := make([][3]int, 0, 2*corners*count+4)
	for i := range count {
		for j := range corners {
			triangles = append(triangles,
				[3]int{at(i, j), at(i, j+1), at(i+1, j+1)},
				[3]int{at(i, j), at(i+1, j+1), at(i+1, j)})
		}
	}
	// The caps, wound so each faces out of its own end.
	triangles = append(triangles,
		[3]int{at(0, 0), at(0, 2), at(0, 1)},
		[3]int{at(0, 0), at(0, 3), at(0, 2)},
		[3]int{at(count, 0), at(count, 1), at(count, 2)},
		[3]int{at(count, 0), at(count, 2), at(count, 3)})

	return solidlens.NewMesh(vertices, triangles)
}

// roundSegments is how many facets a round part of the frame is drawn with.
const roundSegments = 48

// ringMesh draws the ring: a round wire bent into a circle, which is a torus
// about the frame's axis at the ring's height.
func ringMesh(p Params) (*solidlens.Mesh, error) {
	const around = 24
	profile := make([]render.Vec2, 0, around)
	for i := range around {
		a := 2 * math.Pi * float64(i) / around
		profile = append(profile, render.Vec2{
			X: p.CageRise + p.WireRadius()*math.Cos(a),
			Y: p.RingRadius + p.WireRadius()*math.Sin(a),
		})
	}
	return render.Revolve(profile, 3*roundSegments)
}

// barMesh draws a straight round bar from a to b as a cylinder, made on the
// frame's axis and moved into place.
func barMesh(a, b r3.Vec, radius float64) (*solidlens.Mesh, error) {
	length := b.Sub(a).Len()
	cylinder, err := render.Revolve([]render.Vec2{{X: 0, Y: 0}, {X: 0, Y: radius},
		{X: length, Y: radius}, {X: length, Y: 0}}, roundSegments)
	if err != nil {
		return nil, err
	}
	ez, ok := b.Sub(a).Normalize()
	if !ok {
		return nil, fmt.Errorf("a bar from %v to %v has no length", a, b)
	}
	// Any perpendicular will do for the other two axes; the bar is round.
	seed := r3.NewVec(0, 0, 1)
	if math.Abs(ez.Dot(seed)) > 0.9 {
		seed = r3.NewVec(1, 0, 0)
	}
	ex, _ := ez.Cross(seed).Normalize()
	ey := ez.Cross(ex)
	place, err := r3.FromBasis(r3.Basis{EX: ex, EY: ey, EZ: ez}, a)
	if err != nil {
		return nil, err
	}
	return render.Placed(cylinder, place)
}

// ballMesh draws a sphere, which is how a bar's end is rounded where it meets
// another.
func ballMesh(at r3.Vec, radius float64) (*solidlens.Mesh, error) {
	const around = 16
	profile := make([]render.Vec2, 0, around+1)
	for i := 0; i <= around; i++ {
		a := -math.Pi/2 + math.Pi*float64(i)/around
		profile = append(profile, render.Vec2{X: radius * math.Sin(a), Y: radius * math.Cos(a)})
	}
	sphere, err := render.Revolve(profile, roundSegments)
	if err != nil {
		return nil, err
	}
	move, err := r3.Translation(at)
	if err != nil {
		return nil, err
	}
	return render.Placed(sphere, move)
}

// roundedRect is the outline of a rectangle grown by a disc: straight sides
// joined by quarter circles. Only the arcs carry points, so two outlines of
// different sizes drawn with the same count pair up point for point.
func roundedRect(hw, ht, r float64, perCorner int) []render.Vec2 {
	out := make([]render.Vec2, 0, 4*(perCorner+1))
	for _, corner := range [][2]float64{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}} {
		cx, cy := corner[0]*hw, corner[1]*ht
		start := math.Atan2(corner[1], corner[0]) - math.Pi/4
		for i := 0; i <= perCorner; i++ {
			a := start + math.Pi/2*float64(i)/float64(perCorner)
			out = append(out, render.Vec2{X: cx + r*math.Cos(a), Y: cy + r*math.Sin(a)})
		}
	}
	return out
}

// collarMesh draws one collar: the bore grown by the wall, swept along the
// ribbon and turning with it, with the bore itself open through both ends.
//
// The bore's own outline is drawn with a hair of corner radius rather than
// none, so that its corners are distinct vertices the outer arcs can pair with.
func collarMesh(f frame, c crossing) (*solidlens.Mesh, error) {
	p := f.p
	const perCorner = 6
	const stations = 32
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	outer := roundedRect(hw, ht, p.CollarWall, perCorner)
	inner := roundedRect(hw, ht, 0.02, perCorner)
	n := len(outer)

	var vertices []solidlens.Vec
	index := func(k, i int, in bool) int {
		base := k * 2 * n
		if in {
			base += n
		}
		return base + i%n
	}
	for k := 0; k <= stations; k++ {
		s := c.station - p.CollarHalf + 2*p.CollarHalf*float64(k)/stations
		for _, q := range outer {
			vertices = append(vertices, c.g.world(q.X, q.Y, s))
		}
		for _, q := range inner {
			vertices = append(vertices, c.g.world(q.X, q.Y, s))
		}
	}

	var triangles [][3]int
	// quad adds a face, wound so its normal points along out.
	quad := func(a, b, cc, d int, out r3.Vec) {
		normal := vertices[b].Sub(vertices[a]).Cross(vertices[cc].Sub(vertices[a]))
		if normal.Dot(out) < 0 {
			a, b, cc, d = d, cc, b, a
		}
		triangles = append(triangles, [3]int{a, b, cc}, [3]int{a, cc, d})
	}
	for k := range stations {
		for i := range n {
			mid := vertices[index(k, i, false)].Add(vertices[index(k+1, i+1, false)]).Scale(0.5)
			axis := c.g.world(0, 0, c.station)
			out := mid.Sub(axis)
			out = out.Sub(c.g.Ez.Scale(out.Dot(c.g.Ez)))
			quad(index(k, i, false), index(k, i+1, false), index(k+1, i+1, false), index(k+1, i, false), out)
			quad(index(k, i, true), index(k, i+1, true), index(k+1, i+1, true), index(k+1, i, true), out.Scale(-1))
		}
	}
	for i := range n {
		quad(index(0, i, false), index(0, i+1, false), index(0, i+1, true), index(0, i, true), c.g.Ez.Scale(-1))
		quad(index(stations, i, false), index(stations, i+1, false), index(stations, i+1, true),
			index(stations, i, true), c.g.Ez)
	}
	return solidlens.NewMesh(vertices, triangles)
}

// cageMesh draws the whole frame: the ring, the loop, the four rods with a
// ball at each foot where the loop's bars meet, and the four collars.
func cageMesh(f frame) (*solidlens.Mesh, error) {
	p := f.p
	var pieces []solidlens.TriangleSource

	ring, err := ringMesh(p)
	if err != nil {
		return nil, fmt.Errorf("the ring: %w", err)
	}
	pieces = append(pieces, ring)
	for _, bar := range f.loopBars() {
		m, err := barMesh(bar[0], bar[1], p.WireRadius())
		if err != nil {
			return nil, fmt.Errorf("a bar of the loop: %w", err)
		}
		pieces = append(pieces, m)
	}
	for i, c := range f.cross {
		foot, top := f.rodPoint(c, -p.CageRise), f.rodPoint(c, p.CageRise)
		rod, err := barMesh(foot, top, p.RodRadius())
		if err != nil {
			return nil, fmt.Errorf("rod %d: %w", i, err)
		}
		ball, err := ballMesh(foot, p.WireRadius())
		if err != nil {
			return nil, fmt.Errorf("the foot of rod %d: %w", i, err)
		}
		collar, err := collarMesh(f, c)
		if err != nil {
			return nil, fmt.Errorf("collar %d: %w", i, err)
		}
		pieces = append(pieces, rod, ball, collar)
	}
	return render.Merge(pieces...)
}

// The whole mechanism, both ribbons full length in the cage rings.
func TestRenderPair(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example images are not being regenerated")
	}
	ga, gb := defaultPair()

	fromA, toA := ga.span()
	meshA, err := ribbonMesh(ga, fromA, toA)
	if err != nil {
		t.Fatalf("mesh gear A: %v", err)
	}
	fromB, toB := gb.span()
	meshB, err := ribbonMesh(gb, fromB, toB)
	if err != nil {
		t.Fatalf("mesh gear B: %v", err)
	}
	cage, err := cageMesh(newFrame(ga, gb))
	if err != nil {
		t.Fatalf("mesh the frame: %v", err)
	}

	parts := []render.Part{
		{Mesh: meshA, Color: gearAColor},
		{Mesh: meshB, Color: gearBColor},
		{Mesh: cage, Color: cageColor},
	}
	write(t, "pair.png", parts, 26, -58, 30, meshA, meshB, cage)

	// The same assembly from almost overhead, which is the only view that shows
	// the angle the two axes cross at. From the side the pair reads as two
	// ribbons lying near each other whatever that angle is.
	write(t, "plan.png", parts, 78, -90, 30, meshA, meshB, cage)

	// The frame with one gear left in it, from a little above, which is the view
	// that shows a collar with the ribbon's teeth running through it, the rod
	// beside it, and the loop's straight sides against the ring's round one.
	write(t, "cage.png", []render.Part{
		{Mesh: meshA, Color: gearAColor},
		{Mesh: cage, Color: cageColor},
	}, 20, -120, 30, cage)
}

// One gear alone, so the twisted rack reads: a flat toothed rack whose toothed
// edge spirals once every Twist Lead.
func TestRenderPart(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example images are not being regenerated")
	}
	ga, _ := defaultPair()

	from, to := ga.span()
	mesh, err := ribbonMesh(ga, from, to)
	if err != nil {
		t.Fatalf("mesh the gear: %v", err)
	}
	write(t, "part.png", []render.Part{{Mesh: mesh, Color: gearAColor}}, 14, -90, 26, mesh)
}

// The mesh itself, close in on the crossing. This is the picture that shows
// whether the teeth engage, which is the one thing about this gear that no
// number on a page settles for a reader.
func TestRenderMesh(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example images are not being regenerated")
	}
	ga, gb := defaultPair()
	reach := axialWindow(ga.P)

	meshA, err := ribbonMesh(ga, -reach, reach)
	if err != nil {
		t.Fatalf("mesh gear A: %v", err)
	}
	meshB, err := ribbonMesh(gb, -reach, reach)
	if err != nil {
		t.Fatalf("mesh gear B: %v", err)
	}
	parts := []render.Part{
		{Mesh: meshA, Color: gearAColor},
		{Mesh: meshB, Color: gearBColor},
	}
	write(t, "mesh.png", parts, 8, -90, 26, meshA, meshB)

	// And along the crossing from the other side, where the two tooth rows are
	// seen to run at an angle to each other rather than along one line. That is
	// what makes the contact a point rather than a line, and it is what the
	// mounting angle is there to work around.
	short := 0.6 * reach
	nearA, err := ribbonMesh(ga, -short, short)
	if err != nil {
		t.Fatalf("mesh gear A short: %v", err)
	}
	nearB, err := ribbonMesh(gb, -short, short)
	if err != nil {
		t.Fatalf("mesh gear B short: %v", err)
	}
	write(t, "mesh-across.png", []render.Part{
		{Mesh: nearA, Color: gearAColor},
		{Mesh: nearB, Color: gearBColor},
	}, 24, -150, 26, nearA, nearB)
}

// sleeveMeshStep is the grid the sleeve is meshed on for its pictures, in mm.
const sleeveMeshStep = 0.2

// TestRenderSleeve draws the printable sleeve that sleeve_test.go proves, with
// the same ribbons in it. The sleeve has four twisted holes cut through a
// tube, and the pieces the video frame is drawn from are laid over each other
// with no boolean between them, so the sleeve is meshed from its own inside
// test instead: sleeveMesh draws the surface of the set sleeve.inFrame
// describes, which is the frame the proof walks and nothing else.
func TestRenderSleeve(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example images are not being regenerated")
	}
	f := defaultSleeve()
	ga, gb := f.gears[0], f.gears[1]

	frame, err := sleeveMesh(f, sleeveMeshStep)
	if err != nil {
		t.Fatalf("mesh the sleeve: %v", err)
	}
	fromA, toA := ga.span()
	meshA, err := ribbonMesh(ga, fromA, toA)
	if err != nil {
		t.Fatalf("mesh gear A: %v", err)
	}
	fromB, toB := gb.span()
	meshB, err := ribbonMesh(gb, fromB, toB)
	if err != nil {
		t.Fatalf("mesh gear B: %v", err)
	}
	sleevePart := render.Part{Mesh: frame, Color: cageColor}
	both := []render.Part{{Mesh: meshA, Color: gearAColor}, {Mesh: meshB, Color: gearBColor}, sleevePart}

	// The sleeve alone, standing on the end it prints on, looking at the +X
	// side: gear A's +R hole low at 40 degrees beside gear B's +R hole high at
	// 320 degrees.
	write(t, "sleeve-frame.png", []render.Part{sleevePart}, 20, 0, 30, frame)
	// One gear through it, its teeth running through both its holes.
	write(t, "sleeve-cage.png", []render.Part{{Mesh: meshA, Color: gearAColor}, sleevePart}, 20, -50, 30, frame)
	// Both gears, from the side and from almost overhead, framed as the video
	// frame's pictures are.
	write(t, "sleeve-pair.png", both, 26, -58, 30, meshA, meshB, frame)
	write(t, "sleeve-plan.png", both, 78, -90, 30, meshA, meshB, frame)
	// Straight down the hollow from each end, framed on the sleeve: the mesh
	// seen from the top and from the bottom. The camera stops a degree short of
	// the axis because its up direction is the axis.
	write(t, "sleeve-top.png", both, 89, -90, 30, frame)
	write(t, "sleeve-bottom.png", both, -89, -90, 30, frame)
}

// sleeveMesh draws the sleeve's surface on a grid of step h.
func sleeveMesh(f sleeve, h float64) (*solidlens.Mesh, error) {
	pad := r3.NewVec(h, h, h)
	lo := r3.NewVec(-f.ro, -f.ro, -f.zb).Sub(pad)
	hi := r3.NewVec(f.ro, f.ro, f.zb).Add(pad)
	return implicitMesh(f.inFrame, lo, hi, h)
}

// cubeCorner is the offset of each corner of a grid cube, and cubeTets cuts
// the cube into six tetrahedra round its diagonal from corner 0 to corner 6.
// Every cube is cut the same way, so neighbouring cubes cut a shared face
// along the same diagonal, and every edge a tetrahedron uses runs from a grid
// node to one of the seven nodes above it in X, Y and Z.
var (
	cubeCorner = [8][3]int{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}
	cubeTets   = [6][4]int{{0, 5, 1, 6}, {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6}, {0, 7, 4, 6}, {0, 4, 5, 6}}
)

// implicitMesh draws the surface of the set inside describes, within the box
// from lo to hi, by marching tetrahedra: the box is cut into cubes of side h,
// each cube into six tetrahedra, and each tetrahedron whose corners are not
// all on one side of the surface gets one or two triangles across it. Each
// triangle corner sits where the surface crosses a tetrahedron's edge, found
// by bisecting that edge against inside, so every vertex lies on the true
// surface to well under a micron.
//
// Every crossing within a quarter of an edge of the same grid node is made
// one vertex, placed where the first of them crosses. Without that, a surface
// passing close to a node leaves slivers whose normals point anywhere, which
// shade and outline as noise; with it, those slivers collapse and are
// dropped, and the merged vertex still lies on the surface.
func implicitMesh(inside func(r3.Vec) bool, lo, hi r3.Vec, h float64) (*solidlens.Mesh, error) {
	nx := int(math.Ceil((hi.X-lo.X)/h)) + 1
	ny := int(math.Ceil((hi.Y-lo.Y)/h)) + 1
	nz := int(math.Ceil((hi.Z-lo.Z)/h)) + 1
	node := func(i, j, k int) int { return (k*ny+j)*nx + i }
	pos := func(i, j, k int) r3.Vec {
		return r3.NewVec(lo.X+float64(i)*h, lo.Y+float64(j)*h, lo.Z+float64(k)*h)
	}

	in := make([]bool, nx*ny*nz)
	var wg sync.WaitGroup
	for k := range nz {
		wg.Go(func() {
			for j := range ny {
				for i := range nx {
					in[node(i, j, k)] = inside(pos(i, j, k))
				}
			}
		})
	}
	wg.Wait()

	const snap = 0.25
	var vertices []solidlens.Vec
	index := map[int64]int{}
	vertexAt := func(key int64, p r3.Vec) int {
		if at, ok := index[key]; ok {
			return at
		}
		index[key] = len(vertices)
		vertices = append(vertices, solidlens.Vec{X: p.X, Y: p.Y, Z: p.Z})
		return len(vertices) - 1
	}
	// crossing is the vertex where the surface crosses the edge between two
	// nodes, one inside and one out, keyed by the edge's lower node and its
	// direction, or by the node itself when the crossing is moved onto it.
	crossing := func(a, b [3]int) int {
		low, high := a, b
		if b[0] < a[0] || b[1] < a[1] || b[2] < a[2] {
			low, high = b, a
		}
		pIn, pOut := pos(a[0], a[1], a[2]), pos(b[0], b[1], b[2])
		if !in[node(a[0], a[1], a[2])] {
			pIn, pOut = pOut, pIn
		}
		for range 16 {
			mid := pIn.Add(pOut).Scale(0.5)
			if inside(mid) {
				pIn = mid
			} else {
				pOut = mid
			}
		}
		at := pIn.Add(pOut).Scale(0.5)
		pLow, pHigh := pos(low[0], low[1], low[2]), pos(high[0], high[1], high[2])
		frac := at.Sub(pLow).Len() / pHigh.Sub(pLow).Len()
		lowNode := int64(node(low[0], low[1], low[2]))
		switch {
		case frac < snap:
			return vertexAt(lowNode*8, at)
		case frac > 1-snap:
			return vertexAt(int64(node(high[0], high[1], high[2]))*8, at)
		}
		code := int64((high[0] - low[0]) + 2*(high[1]-low[1]) + 4*(high[2]-low[2]))
		return vertexAt(lowNode*8+code, at)
	}

	var triangles [][3]int
	// face adds a triangle, wound so its normal points from the inside
	// corners of its tetrahedron toward the outside ones.
	face := func(a, b, c int, out r3.Vec) {
		if a == b || b == c || c == a {
			return
		}
		va, vb, vc := vertices[a], vertices[b], vertices[c]
		n := vb.Sub(va).Cross(vc.Sub(va))
		if n.X*out.X+n.Y*out.Y+n.Z*out.Z < 0 {
			b, c = c, b
		}
		triangles = append(triangles, [3]int{a, b, c})
	}
	for k := range nz - 1 {
		for j := range ny - 1 {
			for i := range nx - 1 {
				var corner [8][3]int
				var flag [8]bool
				all, none := true, true
				for c, o := range cubeCorner {
					corner[c] = [3]int{i + o[0], j + o[1], k + o[2]}
					flag[c] = in[node(corner[c][0], corner[c][1], corner[c][2])]
					all = all && flag[c]
					none = none && !flag[c]
				}
				if all || none {
					continue
				}
				for _, tet := range cubeTets {
					var ins, outs [][3]int
					for _, c := range tet {
						if flag[c] {
							ins = append(ins, corner[c])
						} else {
							outs = append(outs, corner[c])
						}
					}
					if len(ins) == 0 || len(outs) == 0 {
						continue
					}
					out := centroid(pos, outs).Sub(centroid(pos, ins))
					switch len(ins) {
					case 1:
						face(crossing(ins[0], outs[0]), crossing(ins[0], outs[1]), crossing(ins[0], outs[2]), out)
					case 3:
						face(crossing(ins[0], outs[0]), crossing(ins[1], outs[0]), crossing(ins[2], outs[0]), out)
					default:
						ac, ad := crossing(ins[0], outs[0]), crossing(ins[0], outs[1])
						bc, bd := crossing(ins[1], outs[0]), crossing(ins[1], outs[1])
						face(ac, ad, bd, out)
						face(ac, bd, bc, out)
					}
				}
			}
		}
	}
	return solidlens.NewMesh(vertices, triangles)
}

func centroid(pos func(i, j, k int) r3.Vec, nodes [][3]int) r3.Vec {
	var sum r3.Vec
	for _, n := range nodes {
		sum = sum.Add(pos(n[0], n[1], n[2]))
	}
	return sum.Scale(1 / float64(len(nodes)))
}

func write(t *testing.T, name string, parts []render.Part, elevation, azimuth, fov float64,
	meshes ...solidlens.TriangleSource) {
	t.Helper()
	camera, err := render.Fit{
		ElevationDeg: elevation,
		AzimuthDeg:   azimuth,
		FOV:          fov,
		Margin:       0.05,
		Settings:     renderSettings,
	}.Camera(meshes...)
	if err != nil {
		t.Fatalf("frame %s: %v", name, err)
	}
	path := filepath.Join(*renderOut, name)
	if err := render.WritePNG(t.Context(), path, scene(camera, parts), renderSettings); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s", path)
}

// creaseAngle is the smallest fold between neighbouring faces that still draws
// an edge here, in degrees.
//
// It is raised well above the 30 degrees render.Scene leaves in place, because
// a twisted ribbon's quads are not planar: across one band the far section is
// turned by 1.4 degrees, which moves a corner 0.12 mm across a band only
// 0.22 mm long, and the two triangles either side of that quad's diagonal meet
// at close to 30 degrees. At the default every one of those diagonals draws,
// and the ribbon comes out hatched along its whole length with lines that are
// an artefact of how it was cut into triangles rather than anything on the
// part. Sixty degrees keeps the real corners — the tooth flanks, the plate's
// own edges, the end caps — and drops the diagonals.
const creaseAngle = 60

// scene stages the parts under the shared light rig and background, which is
// what keeps these pictures reading like the other gears' examples.
func scene(camera solidlens.Camera, parts []render.Part) solidlens.Scene {
	models := make([]solidlens.Model, 0, len(parts))
	for _, p := range parts {
		models = append(models, solidlens.Model{
			Mesh:     p.Mesh,
			Material: solidlens.Matte(p.Color),
			Edges: solidlens.Edges{
				Enabled:     true,
				Color:       solidlens.RGB(p.Color.R*0.28, p.Color.G*0.28, p.Color.B*0.28),
				Width:       1.4,
				CreaseAngle: creaseAngle,
			},
		})
	}
	return solidlens.Scene{
		Camera:            camera,
		Models:            models,
		DirectionalLights: render.Lights(),
		Background:        render.Background,
	}
}
