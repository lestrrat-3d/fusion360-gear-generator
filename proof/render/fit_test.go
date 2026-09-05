package render_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/solidlens"
	"github.com/stretchr/testify/require"
)

// The frame a fitted camera produces is checked by projecting the mesh through
// the camera it returns, with the projection written out here rather than taken
// from the package under test. A fit that agrees only with its own arithmetic
// is a fit for a picture nobody renders, so the check has to be independent of
// it: what follows is solidlens's own mapping, and the whole claim is that the
// camera puts the mesh inside the frame, up against the margin, and centred.

func TestFitFillsTheFrame(t *testing.T) {
	settings := solidlens.Settings{Width: 960, Height: 720}
	for _, tc := range []struct {
		name   string
		mesh   *solidlens.Mesh
		margin float64
	}{
		{"a squat disc", disc(t, 40, 6), 0.05},
		{"a tall disc, which binds on the frame's height instead", disc(t, 6, 60), 0.05},
		{"no margin at all", disc(t, 40, 6), 0},
		{"a wide margin", disc(t, 40, 6), 0.4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			camera, err := render.Fit{
				ElevationDeg: 26,
				AzimuthDeg:   -112,
				FOV:          32,
				Margin:       tc.margin,
				Settings:     settings,
			}.Camera(tc.mesh)
			require.NoError(t, err)

			loX, hiX, loY, hiY := project(t, camera, settings, tc.mesh)
			limit := 1 - tc.margin
			require.LessOrEqual(t, math.Max(math.Abs(loX), math.Abs(hiX)), limit+1e-6,
				"the mesh reaches past the frame's width")
			require.LessOrEqual(t, math.Max(math.Abs(loY), math.Abs(hiY)), limit+1e-6,
				"the mesh reaches past the frame's height")
			// One side has to BIND, or the camera stopped further out than it
			// had to and the mesh is smaller in the picture than it should be.
			widest := math.Max((hiX-loX)/2, (hiY-loY)/2)
			require.InDelta(t, limit, widest, 1e-4, "the mesh does not fill the frame")
			require.InDelta(t, 0, loX+hiX, 1e-4, "the mesh is off centre across the frame")
			require.InDelta(t, 0, loY+hiY, 1e-4, "the mesh is off centre up the frame")
		})
	}
}

// TestFitIsIndependentOfScale is the property that makes the fit worth having:
// a gear twice the size is framed identically, so no caller carries a distance
// that a change of module invalidates.
func TestFitIsIndependentOfScale(t *testing.T) {
	settings := solidlens.Settings{Width: 640, Height: 640}
	fit := render.Fit{ElevationDeg: 30, AzimuthDeg: -58, FOV: 40, Margin: 0.1, Settings: settings}

	small, err := fit.Camera(disc(t, 10, 4))
	require.NoError(t, err)
	large, err := fit.Camera(disc(t, 20, 8))
	require.NoError(t, err)

	require.InEpsilon(t, 2*small.Position.Sub(small.Target).Len(), large.Position.Sub(large.Target).Len(), 1e-6)
}

func TestFitRejects(t *testing.T) {
	settings := solidlens.Settings{Width: 960, Height: 720}
	base := render.Fit{ElevationDeg: 26, AzimuthDeg: -112, FOV: 32, Margin: 0.05, Settings: settings}

	_, err := base.Camera()
	require.Error(t, err, "there is nothing to frame")

	bad := base
	bad.Margin = 1
	_, err = bad.Camera(disc(t, 10, 4))
	require.Error(t, err, "a margin of one leaves no frame")

	bad = base
	bad.Settings = solidlens.Settings{}
	_, err = bad.Camera(disc(t, 10, 4))
	require.Error(t, err, "an unsized raster has no aspect ratio")
}

// project returns the frame coordinates the mesh spans, in the units solidlens
// clips against: the frame runs from -1 to 1 across and up.
func project(t *testing.T, camera solidlens.Camera, settings solidlens.Settings, mesh *solidlens.Mesh) (loX, hiX, loY, hiY float64) {
	t.Helper()
	forward, ok := camera.Target.Sub(camera.Position).Normalize()
	require.True(t, ok)
	right, ok := forward.Cross(camera.Up).Normalize()
	require.True(t, ok)
	up, ok := right.Cross(forward).Normalize()
	require.True(t, ok)
	focal := 1 / math.Tan(camera.FOV*math.Pi/360)
	aspect := float64(settings.Width) / float64(settings.Height)

	loX, hiX = math.Inf(1), math.Inf(-1)
	loY, hiY = math.Inf(1), math.Inf(-1)
	for _, p := range mesh.Vertices() {
		relative := p.Sub(camera.Position)
		depth := relative.Dot(forward)
		require.Greater(t, depth, 0.0, "a vertex sits behind the camera")
		x := relative.Dot(right) * focal / (depth * aspect)
		y := relative.Dot(up) * focal / depth
		loX, hiX = math.Min(loX, x), math.Max(hiX, x)
		loY, hiY = math.Min(loY, y), math.Max(hiY, y)
	}
	return loX, hiX, loY, hiY
}

// disc is a plain cylinder of the given radius and height, standing on z = 0.
func disc(t *testing.T, radius, height float64) *solidlens.Mesh {
	t.Helper()
	mesh, err := render.Revolve([]render.Vec2{
		{X: 0, Y: 0}, {X: height, Y: 0}, {X: height, Y: radius}, {X: 0, Y: radius},
	}, 180)
	require.NoError(t, err)
	return mesh
}
