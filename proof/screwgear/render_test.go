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
// half a pitch out of step with gear A, both mounted at the same angle, at the
// crossing angle the crossed-helical rule gives.
// ---------------------------------------------------------------------------

var renderOut = flag.String("render.out", "",
	"directory the example images are written to; the render is skipped when it is empty")

var renderSettings = solidlens.Settings{Width: 1100, Height: 820}

// renderStations is how many cross-sections are meshed per tooth. The tooth is
// a cosine, so this is what decides whether a crest reads as a crest; sixteen
// puts a section every 22 degrees of the wave. It is a drawing resolution and
// has nothing to do with the nine sections the spec lofts a tooth from.
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

// collarMesh draws one collar: a disc standing across its gear's axis with the
// ribbon's own channel cut through it.
//
// It is meshed on a polar grid rather than cut by a boolean. A cell is dropped
// when it falls in the channel at either face or between them, which is the
// swept opening a real cut would leave, and the same local mapping the meshing
// proof uses. The cells' shared corners are then walked onto the true edge of
// the channel, because a cell either stands or goes whole and an opening that
// runs diagonally across the grid would otherwise come out as a staircase.
func collarMesh(g Gear) (*solidlens.Mesh, error) {
	const nAz, nR = 240, 90
	const rMin = 0.35

	centre := collarCentre(g)
	half := g.P.CollarDepth / 2
	point := func(face, i, j int) r3.Vec {
		a := 2 * math.Pi * float64(i%nAz) / nAz
		r := rMin + (g.P.CollarOuter-rMin)*float64(j)/nR
		along := float64(2*face-1) * half
		return centre.Add(g.Ez.Scale(along)).
			Add(g.Ex.Scale(r * math.Cos(a))).Add(g.Ey.Scale(r * math.Sin(a)))
	}
	// open is the channel test, taken across the collar's depth so the opening
	// is what the turning ribbon sweeps rather than its section at one face.
	open := func(pt r3.Vec) bool {
		for k := range 5 {
			along := -half + 2*half*float64(k)/4
			probe := pt.Add(g.Ez.Scale(along - pt.Sub(centre).Dot(g.Ez)))
			if inCollarOpening(g, probe) {
				return true
			}
		}
		return false
	}

	index := func(face, i, j int) int { return face*nAz*(nR+1) + (i%nAz)*(nR+1) + j }
	vertices := make([]solidlens.Vec, 2*nAz*(nR+1))
	for face := range 2 {
		for i := range nAz {
			for j := 0; j <= nR; j++ {
				pt := point(face, i, j)
				vertices[index(face, i, j)] = solidlens.Vec{X: pt.X, Y: pt.Y, Z: pt.Z}
			}
		}
	}

	keep := make([][]bool, nAz)
	for i := range nAz {
		keep[i] = make([]bool, nR)
		for j := range nR {
			mid := point(0, i, j).Add(point(1, i+1, j+1)).Scale(0.5)
			keep[i][j] = !open(mid)
		}
	}
	kept := func(i, j int) bool {
		if j < 0 || j >= nR {
			return false
		}
		return keep[(i+nAz)%nAz][j]
	}

	// Snap each corner that stands on the channel's edge onto the edge itself.
	for face := range 2 {
		for i := range nAz {
			for j := 0; j <= nR; j++ {
				standing := kept(i, j) || kept(i, j-1) || kept(i-1, j) || kept(i-1, j-1)
				gone := !kept(i, j) || !kept(i, j-1) || !kept(i-1, j) || !kept(i-1, j-1)
				if !standing || !gone {
					continue
				}
				here := point(face, i, j)
				inside := open(here)
				dir := 0
				for _, d := range []int{1, -1} {
					if open(point(face, i+d, j)) != inside {
						dir = d
						break
					}
				}
				if dir == 0 {
					continue
				}
				far := point(face, i+dir, j)
				if !inside {
					here, far = far, here
				}
				lo, hi := 0.0, 1.0
				for range 24 {
					m := (lo + hi) / 2
					if open(here.Add(far.Sub(here).Scale(m))) {
						lo = m
					} else {
						hi = m
					}
				}
				pt := here.Add(far.Sub(here).Scale((lo + hi) / 2))
				vertices[index(face, i, j)] = solidlens.Vec{X: pt.X, Y: pt.Y, Z: pt.Z}
			}
		}
	}

	const back, front = 0, 1
	var triangles [][3]int
	quad := func(a, b, c, d int) {
		triangles = append(triangles, [3]int{a, b, c}, [3]int{a, c, d})
	}
	for i := range nAz {
		for j := range nR {
			if !keep[i][j] {
				continue
			}
			quad(index(front, i, j), index(front, i+1, j), index(front, i+1, j+1), index(front, i, j+1))
			quad(index(back, i, j), index(back, i, j+1), index(back, i+1, j+1), index(back, i+1, j))
			if !kept(i, j+1) { // the rim, or the far side of the channel
				quad(index(back, i, j+1), index(front, i, j+1), index(front, i+1, j+1), index(back, i+1, j+1))
			}
			if !kept(i, j-1) {
				quad(index(back, i, j), index(back, i+1, j), index(front, i+1, j), index(front, i, j))
			}
			if !kept(i+1, j) {
				quad(index(back, i+1, j), index(back, i+1, j+1), index(front, i+1, j+1), index(front, i+1, j))
			}
			if !kept(i-1, j) {
				quad(index(back, i, j), index(front, i, j), index(front, i, j+1), index(back, i, j+1))
			}
		}
	}
	if len(triangles) == 0 {
		return nil, fmt.Errorf("the channel removed the whole collar")
	}
	return solidlens.NewMesh(vertices, triangles)
}

// cageMesh draws the frame: one collar per gear, meeting at their rims.
func cageMesh(ga, gb Gear) (*solidlens.Mesh, error) {
	a, err := collarMesh(ga)
	if err != nil {
		return nil, fmt.Errorf("gear A collar: %w", err)
	}
	b, err := collarMesh(gb)
	if err != nil {
		return nil, fmt.Errorf("gear B collar: %w", err)
	}
	return render.Merge(a, b)
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
	cage, err := cageMesh(ga, gb)
	if err != nil {
		t.Fatalf("mesh the cage: %v", err)
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

	// The frame on its own, looked at straight down gear A's axis, which is the
	// one view that shows an opening at its true shape. Neither gear is drawn:
	// a ribbon on this line of sight fills the frame, being exactly what the
	// opening is cut to pass.
	slotAzimuth := p.Sigma() / 2 * 180 / math.Pi
	write(t, "cage.png", []render.Part{{Mesh: cage, Color: cageColor}},
		-12, slotAzimuth, 30, cage)
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
