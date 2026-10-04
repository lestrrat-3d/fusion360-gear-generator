package screwgear_test

import (
	"flag"
	"fmt"
	"math"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// Pictures of the screw gearing.
//
// These are not proofs and they are skipped unless -render.out names a
// directory. They live in the proof's own package because the alternative is a
// second description of the same part: every section drawn here is Gear.outline,
// whose toothed side is Gear.edgeAt, the same function the mesh proof samples,
// at the same defaults. A change that
// moves the proved geometry moves the pictures with it.
//
// The pair is drawn in the arrangement TestPairDrivesOneToOne passes at: gear B
// at the assembly phase, both gears mounted at the same angle, at the crossing
// angle the spec's default table gives, and the sleeve round them is the one
// sleeve_test.go proves.
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

// renderEdgePoints is how many points across the thickness the toothed side
// of each drawn section passes through. Thirteen put them 0.31 mm apart, so a
// ridge leaning at the default slant moves 0.15 mm of station from one to the
// next, under the 0.16 mm between drawn sections.
const renderEdgePoints = 13

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

	// The toothed side leans across the thickness, so it is drawn through
	// renderEdgePoints points of it rather than as one straight line.
	const corners = renderEdgePoints + 2
	vertices := make([]solidlens.Vec, 0, corners*(count+1))
	for i := 0; i <= count; i++ {
		for _, p := range g.outline(from+float64(i)*step, renderEdgePoints) {
			vertices = append(vertices, solidlens.Vec{X: p.X, Y: p.Y, Z: p.Z})
		}
	}
	at := func(station, corner int) int { return station*corners + corner%corners }

	// Each band between two sections is split along the diagonal from (i+1, j)
	// to (i, j+1), which runs the way a ridge leans: toward the lower station as
	// v grows. The other diagonal cuts across the ridges, and the folds it
	// leaves draw as a fringe of short edges along every tooth.
	triangles := make([][3]int, 0, 2*corners*count+2*corners)
	for i := range count {
		for j := range corners {
			triangles = append(triangles,
				[3]int{at(i, j), at(i, j+1), at(i+1, j)},
				[3]int{at(i, j+1), at(i+1, j+1), at(i+1, j)})
		}
	}
	// The caps, fanned from the back corner at -T/2, which sees every point of
	// the outline, and wound so each faces out of its own end.
	for j := 1; j+1 < corners; j++ {
		triangles = append(triangles,
			[3]int{at(0, 0), at(0, j+1), at(0, j)},
			[3]int{at(count, 0), at(count, j), at(count, j+1)})
	}

	return solidlens.NewMesh(vertices, triangles)
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

	// And along the crossing from the other side, where the two tooth rows'
	// ridges are seen leaning across each ribbon's thickness, so that where
	// they meet they lie along each other and touch along a line
	// (contact_test.go).
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
// The edges come out sharp at 0.2 mm too, but where a bore's wall leaves a
// cylinder at a shallow angle the wedge there is thinner than a cube, and
// the mesh leaves short ticks about a cube long, which are half as long at
// 0.1 mm.
const sleeveMeshStep = 0.1

// boreMarkMesh draws one raised bore sign from the same dimensions and centre
// rule as the compiled marker proof. Its lower face starts inside the sleeve.
func boreMarkMesh(f sleeve, gear int, sign float64) (*solidlens.Mesh, error) {
	angle := f.p.Sigma() / 2
	if gear == 1 {
		angle = -angle
	}
	x := sign * f.p.CageRadius * math.Cos(angle)
	y := sign * f.p.CageRadius * math.Sin(angle)
	halfSize := math.Min(1, f.p.CollarHalf/2)
	inset := math.Min(0.1, f.p.CollarWall/2)

	outline := make([][2]float64, 0, 48)
	if sign > 0 {
		for i := range 48 {
			a := 2 * math.Pi * float64(i) / 48
			outline = append(outline, [2]float64{x + halfSize*math.Cos(a), y + halfSize*math.Sin(a)})
		}
	} else {
		outline = append(outline,
			[2]float64{x - halfSize, y - halfSize},
			[2]float64{x + halfSize, y - halfSize},
			[2]float64{x + halfSize, y + halfSize},
			[2]float64{x - halfSize, y + halfSize})
	}

	n := len(outline)
	vertices := make([]solidlens.Vec, 0, 2*n)
	for _, z := range []float64{f.zb - inset, f.zb + 0.4} {
		for _, point := range outline {
			vertices = append(vertices, solidlens.Vec{X: point[0], Y: point[1], Z: z})
		}
	}
	triangles := make([][3]int, 0, 4*n-4)
	for i := range n {
		j := (i + 1) % n
		triangles = append(triangles, [3]int{i, j, n + j}, [3]int{i, n + j, n + i})
	}
	for j := 1; j+1 < n; j++ {
		triangles = append(triangles, [3]int{0, j + 1, j}, [3]int{n, n + j, n + j + 1})
	}
	return solidlens.NewMesh(vertices, triangles)
}

// TestRenderSleeve draws the printable sleeve that sleeve_test.go proves, with
// the same ribbons in it. The sleeve has four twisted holes and two windows
// cut through a tube, and these pictures have no boolean to cut them with, so
// the sleeve is meshed from its own inside test instead: sleeveSharpMesh
// draws the surface of the set sleeve.inFrame describes, which is the frame
// the proof walks and nothing else, and checkSleeveMesh checks that it does.
func TestRenderSleeve(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example images are not being regenerated")
	}
	f := defaultSleeve()
	ga, gb := f.gears[0], f.gears[1]

	m := sleeveSharpMesh(f, sleeveMeshStep)
	checkSleeveMesh(t, f, m)
	frame, err := solidlens.NewMesh(m.vertices, m.triangles)
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
	sleeveParts := []render.Part{sleevePart}
	for gear := range 2 {
		for _, sign := range []float64{-1, 1} {
			mark, err := boreMarkMesh(f, gear, sign)
			if err != nil {
				t.Fatalf("mesh gear %d bore %+.0f mark: %v", gear, sign, err)
			}
			sleeveParts = append(sleeveParts, render.Part{Mesh: mark, Color: cageColor})
		}
	}
	both := append([]render.Part{{Mesh: meshA, Color: gearAColor}, {Mesh: meshB, Color: gearBColor}}, sleeveParts...)

	// The sleeve alone, standing on the end it prints on, looking at the +X
	// side: gear A's +R hole low at 40 degrees beside gear B's +R hole high at
	// 320 degrees.
	write(t, "sleeve-frame.png", sleeveParts, 20, 0, 30, frame)
	// One gear through it, its teeth running through both its holes.
	write(t, "sleeve-cage.png", append([]render.Part{{Mesh: meshA, Color: gearAColor}}, sleeveParts...),
		20, -50, 30, frame)
	// Both gears, from the side and from almost overhead, which is the only
	// view that shows the angle the two axes cross at. From the side the pair
	// reads as two ribbons lying near each other whatever that angle is.
	write(t, "pair.png", both, 26, -58, 30, meshA, meshB, frame)
	write(t, "plan.png", both, 78, -90, 30, meshA, meshB, frame)
	// Straight down the hollow from each end, framed on the sleeve: the mesh
	// seen from the top and from the bottom. The camera stops a degree short of
	// the axis because its up direction is the axis.
	write(t, "sleeve-top.png", both, 89, -90, 30, frame)
	write(t, "sleeve-marks.png", sleeveParts, 89, -90, 30, frame)
	write(t, "sleeve-bottom.png", both, -89, -90, 30, frame)
	// The window facing +Y, square on and a little from above, alone and
	// with the ribbons meshing behind it.
	write(t, "sleeve-window.png", sleeveParts, 12, 90, 30, frame)
	write(t, "sleeve-side.png", both, 6, 90, 30, frame)
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
