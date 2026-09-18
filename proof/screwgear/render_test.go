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

// shellMesh draws one piece of the frame's wall: a stretch of azimuth over a
// stretch of height, between the one inner and outer radius everything shares,
// with the bore of a gear cut out of it when one passes through.
//
// Every piece is drawn the same way, because every piece IS the same wall. That
// is what makes the outside read as one turned surface rather than an assembly
// of bars stuck onto plates.
func shellMesh(p Params, q piece) (*solidlens.Mesh, error) {
	g := q.g
	wide := q.maxHalfAngle()
	nA := int(math.Max(8, math.Round(wide*2*p.CageOuter()/0.15)))
	nZ := int(math.Max(8, math.Round((q.zHi-q.zLo)/0.15)))
	if nA > 900 {
		nA = 900
	}
	if nZ > 600 {
		nZ = 600
	}

	point := func(face, i, j int) r3.Vec {
		r := p.CageInner()
		if face == 1 {
			r = p.CageOuter()
		}
		a := q.azimuth - wide + 2*wide*float64(i)/float64(nA)
		z := q.zLo + (q.zHi-q.zLo)*float64(j)/float64(nZ)
		return r3.NewVec(r*math.Cos(a), r*math.Sin(a), z)
	}
	// A cell is gone where the bore passes, and where the post has narrowed
	// away from its bore. The second is what makes a post taper into the plates
	// rather than sit under a slab.
	open := func(pt r3.Vec) bool {
		if g != nil && inBore(*g, pt) {
			return true
		}
		d := math.Mod(math.Abs(math.Atan2(pt.Y, pt.X)-q.azimuth), 2*math.Pi)
		return math.Min(d, 2*math.Pi-d) > q.halfAngleAt(pt.Z)
	}

	index := func(face, i, j int) int { return face*(nA+1)*(nZ+1) + i*(nZ+1) + j }
	vertices := make([]solidlens.Vec, 2*(nA+1)*(nZ+1))
	for face := range 2 {
		for i := 0; i <= nA; i++ {
			for j := 0; j <= nZ; j++ {
				pt := point(face, i, j)
				vertices[index(face, i, j)] = solidlens.Vec{X: pt.X, Y: pt.Y, Z: pt.Z}
			}
		}
	}
	keep := make([][]bool, nA)
	for i := range nA {
		keep[i] = make([]bool, nZ)
		for j := range nZ {
			mid := point(0, i, j).Add(point(1, i+1, j+1)).Scale(0.5)
			keep[i][j] = !open(mid)
		}
	}
	kept := func(i, j int) bool {
		if j < 0 || j >= nZ {
			return false
		}
		if i < 0 || i >= nA {
			// A piece that goes the whole way round meets itself.
			if wide < math.Pi-1e-9 {
				return false
			}
			i = (i + nA) % nA
		}
		return keep[i][j]
	}

	// Snap each corner standing on an edge onto the edge itself. A cell is kept
	// or dropped whole, so a bore's edge and a post's taper both come out as a
	// staircase otherwise.
	for face := range 2 {
		for i := 0; i <= nA; i++ {
			for j := 0; j <= nZ; j++ {
				standing := kept(i, j) || kept(i, j-1) || kept(i-1, j) || kept(i-1, j-1)
				gone := !kept(i, j) || !kept(i, j-1) || !kept(i-1, j) || !kept(i-1, j-1)
				if !standing || !gone {
					continue
				}
				here := point(face, i, j)
				inside := open(here)
				// Walk round the cage first, then up it: a bore's edge runs
				// mostly one way and a post's taper mostly the other, and
				// neither is fixed by moving a corner along the wrong one.
				var far r3.Vec
				found := false
				for _, d := range []int{1, -1} {
					if i+d < 0 || i+d > nA {
						continue
					}
					if cand := point(face, i+d, j); open(cand) != inside {
						far, found = cand, true
						break
					}
				}
				if !found {
					for _, d := range []int{1, -1} {
						if j+d < 0 || j+d > nZ {
							continue
						}
						if cand := point(face, i, j+d); open(cand) != inside {
							far, found = cand, true
							break
						}
					}
				}
				if !found {
					continue
				}
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

	const in, out = 0, 1
	var triangles [][3]int
	quad := func(a, b, c, d int) {
		triangles = append(triangles, [3]int{a, b, c}, [3]int{a, c, d})
	}
	for i := range nA {
		for j := range nZ {
			if !keep[i][j] {
				continue
			}
			quad(index(out, i, j), index(out, i+1, j), index(out, i+1, j+1), index(out, i, j+1))
			quad(index(in, i, j), index(in, i, j+1), index(in, i+1, j+1), index(in, i+1, j))
			if !kept(i+1, j) {
				quad(index(in, i+1, j), index(in, i+1, j+1), index(out, i+1, j+1), index(out, i+1, j))
			}
			if !kept(i-1, j) {
				quad(index(in, i, j), index(out, i, j), index(out, i, j+1), index(in, i, j+1))
			}
			if !kept(i, j+1) {
				quad(index(in, i, j+1), index(out, i, j+1), index(out, i+1, j+1), index(in, i+1, j+1))
			}
			if !kept(i, j-1) {
				quad(index(in, i, j), index(in, i+1, j), index(out, i+1, j), index(out, i, j))
			}
		}
	}
	if len(triangles) == 0 {
		return nil, fmt.Errorf("a bore removed a whole piece of the wall")
	}
	return solidlens.NewMesh(vertices, triangles)
}

// cageMesh draws the whole frame from its pieces.
func cageMesh(ga, gb Gear) (*solidlens.Mesh, error) {
	p := ga.P
	var pieces []solidlens.TriangleSource
	for _, q := range cagePieces(ga, gb) {
		mesh, err := shellMesh(p, q)
		if err != nil {
			return nil, fmt.Errorf("wall piece at %.0f degrees: %w", q.azimuth*180/math.Pi, err)
		}
		pieces = append(pieces, mesh)
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

	// The frame with one gear left in it, from a little above, which is the view
	// that shows a post's block and the boss sitting in its bore.
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
