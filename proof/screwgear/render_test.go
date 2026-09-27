package screwgear_test

import (
	"flag"
	"fmt"
	"math"
	"path/filepath"
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
// eleven at the defaults.
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
	p := ga.P
	half := p.Length() / 2

	meshA, err := ribbonMesh(ga, -half, half)
	if err != nil {
		t.Fatalf("mesh gear A: %v", err)
	}
	meshB, err := ribbonMesh(gb, -half, half)
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
	// that shows a collar with the boss in it, the rod beside it, and the loop's
	// straight sides against the ring's round one.
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
	half := ga.P.Length() / 2

	mesh, err := ribbonMesh(ga, -half, half)
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
