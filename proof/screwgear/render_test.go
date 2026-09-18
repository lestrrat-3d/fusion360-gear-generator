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

// cageMesh draws the frame: one tube, coaxial with the two gears' common
// perpendicular, with a slot cut through its wall for each gear.
//
// The slots are what make this a frame rather than a pair of bearings. A round
// hole would let its ribbon turn freely as it slid, and the mechanism would
// have three degrees of freedom instead of one; a slot the shape of the
// ribbon's own cross-section forces the ribbon to turn as it advances, exactly
// as a twisted-bar screwdriver does. Each slot is a twisted channel of the same
// lead as its gear, because the ribbon turns as it crosses the wall.
//
// No boolean builds it. A wall cell is dropped when its own midpoint falls
// inside a gear's clearance rectangle, which is the same local mapping the
// meshing proof uses, so the slot is the ribbon's cross-section by construction
// rather than by a second description of it.
func cageMesh(p Params, ga, gb Gear, inner, outer, halfHeight, clearance float64) (*solidlens.Mesh, error) {
	const nAz, nZ = 240, 400

	// inSlot answers whether a point of the wall has been cut away.
	inSlot := func(pt r3.Vec) bool {
		for _, g := range []Gear{ga, gb} {
			u, v, _ := g.local(pt)
			if math.Abs(u) <= p.Width/2+clearance && math.Abs(v) <= p.Thickness/2+clearance {
				return true
			}
		}
		return false
	}

	az := func(i int) float64 { return 2 * math.Pi * float64(i%nAz) / nAz }
	zAt := func(j int) float64 { return -halfHeight + 2*halfHeight*float64(j)/nZ }
	point := func(radius float64, i, j int) r3.Vec {
		a := az(i)
		return r3.NewVec(radius*math.Cos(a), radius*math.Sin(a), zAt(j))
	}

	// One vertex grid, shared by every cell, so neighbouring cells meet at the
	// same index and the renderer sees one surface rather than a field of
	// separate quads.
	index := func(ring, i, j int) int { return ring*nAz*(nZ+1) + (i%nAz)*(nZ+1) + j }
	vertices := make([]solidlens.Vec, 2*nAz*(nZ+1))
	for ring, radius := range []float64{inner, outer} {
		for i := range nAz {
			for j := 0; j <= nZ; j++ {
				pt := point(radius, i, j)
				vertices[index(ring, i, j)] = solidlens.Vec{X: pt.X, Y: pt.Y, Z: pt.Z}
			}
		}
	}

	keep := make([][]bool, nAz)
	mid := (inner + outer) / 2
	for i := range nAz {
		keep[i] = make([]bool, nZ)
		for j := range nZ {
			a, b := az(i), az(i+1)
			centre := r3.NewVec(mid*math.Cos((a+b)/2), mid*math.Sin((a+b)/2), (zAt(j)+zAt(j+1))/2)
			keep[i][j] = !inSlot(centre)
		}
	}
	kept := func(i, j int) bool {
		if j < 0 || j >= nZ {
			return false
		}
		return keep[(i+nAz)%nAz][j]
	}

	// Snap the boundary onto the real slot edge. A cell is kept or dropped
	// whole, so a slot edge that runs diagonally across the grid comes out as a
	// staircase. Every corner that is left standing inside a slot is therefore
	// walked around the tube, by bisection, to where it really crosses the
	// slot's edge. The cells stay the same; only the vertices they share move,
	// so the wall stays closed and its edge becomes the cut line.
	for ring, radius := range []float64{inner, outer} {
		for i := range nAz {
			for j := 0; j <= nZ; j++ {
				// Only a corner where the wall meets a slot moves; everywhere
				// else the grid is already on the tube.
				standing := kept(i, j) || kept(i, j-1) || kept(i-1, j) || kept(i-1, j-1)
				gone := !kept(i, j) || !kept(i, j-1) || !kept(i-1, j) || !kept(i-1, j-1)
				if !standing || !gone {
					continue
				}
				here := point(radius, i, j)
				inside := inSlot(here)
				dir := 0
				for _, d := range []int{1, -1} {
					if inSlot(point(radius, i+d, j)) != inside {
						dir = d
						break
					}
				}
				if dir == 0 {
					continue // the edge does not cross this row within a cell
				}
				far := point(radius, i+dir, j)
				if !inside {
					here, far = far, here
				}
				lo, hi := 0.0, 1.0
				for range 24 {
					m := (lo + hi) / 2
					if inSlot(here.Add(far.Sub(here).Scale(m))) {
						lo = m
					} else {
						hi = m
					}
				}
				pt := here.Add(far.Sub(here).Scale((lo + hi) / 2))
				vertices[index(ring, i, j)] = solidlens.Vec{X: pt.X, Y: pt.Y, Z: pt.Z}
			}
		}
	}

	const in, out = 0, 1
	var triangles [][3]int
	quad := func(a, b, c, d int) {
		triangles = append(triangles, [3]int{a, b, c}, [3]int{a, c, d})
	}
	for i := range nAz {
		for j := range nZ {
			if !keep[i][j] {
				continue
			}
			// The outer face, wound so its normal points away from the axis,
			// and the inner face wound the other way.
			quad(index(out, i, j), index(out, i+1, j), index(out, i+1, j+1), index(out, i, j+1))
			quad(index(in, i, j), index(in, i, j+1), index(in, i+1, j+1), index(in, i+1, j))
			// A wall face wherever the neighbour is gone: the rim of a slot, or
			// the tube's own two ends.
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
		return nil, fmt.Errorf("the slots removed the whole cage wall")
	}
	return solidlens.NewMesh(vertices, triangles)
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
	cage, err := cageMesh(p, ga, gb, p.CageInner(), p.CageOuter(), p.CageHalfHeight(), p.Clearance)
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
