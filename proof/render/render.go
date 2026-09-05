// Package render stages the README's gear example images.
//
// It owns how a gear is LIT and FRAMED, and nothing about what a gear is: a
// caller hands over meshes it built from its own proved geometry and gets back
// a scene staged the same way as every other example, so the five pictures in
// the README read as one set rather than five separate renders.
//
// The mesh builders here are the shapes the proofs' own solids are made of —
// a solid of revolution, and a prism between two corresponding rings — meshed
// directly rather than through decad. A caller that already holds a decad body
// tessellates it instead, with [MeshOfBody].
package render

import (
	"context"
	"fmt"
	"math"
	"os"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// Part is one body in a staged scene: the mesh, and the colour it is shaded
// in. Two parts of the same colour read as one object, which is how a gear
// assembled from several meshes is presented.
type Part struct {
	Mesh  solidlens.TriangleSource
	Color solidlens.Color
}

// Scene stages parts under the standard light rig and background.
//
// Each part draws its own edges in a darkened version of its colour. That is
// what separates one tooth from the next where two flanks face the camera at
// nearly the same angle and flat shading gives them nearly the same value. The
// crease angle is left at its default, which picks up real corners and skips
// the seams between the bands a swept surface is cut into.
func Scene(camera solidlens.Camera, parts ...Part) solidlens.Scene {
	models := make([]solidlens.Model, 0, len(parts))
	for _, p := range parts {
		models = append(models, solidlens.Model{
			Mesh:     p.Mesh,
			Material: solidlens.Matte(p.Color),
			Edges:    outline(p.Color),
		})
	}
	return solidlens.Scene{
		Camera:            camera,
		Models:            models,
		DirectionalLights: Lights(),
		Background:        Background,
	}
}

// Background is the neutral grey every example is shot against.
var Background = solidlens.RGB(0.35, 0.37, 0.42)

// outlineDarken is how far a part's edge colour is taken below its surface
// colour.
const outlineDarken = 0.28

func outline(c solidlens.Color) solidlens.Edges {
	return solidlens.Edges{
		Enabled: true,
		Color:   solidlens.RGB(c.R*outlineDarken, c.G*outlineDarken, c.B*outlineDarken),
		Width:   1.4,
	}
}

// Lights are all directional on purpose. solidlens shades a triangle flat,
// from one of its own corners, so a point light makes a large flat face band
// triangle by triangle as the distance changes across it. A directional light
// depends on the normal alone, which leaves a gear's end faces even and still
// separates the three tooth surfaces by their angle.
func Lights() []solidlens.DirectionalLight {
	return []solidlens.DirectionalLight{
		{Direction: solidlens.Vec{X: 0.35, Y: 0.62, Z: -0.70}, Color: solidlens.RGB(1, 0.97, 0.92), Intensity: 0.95},
		{Direction: solidlens.Vec{X: -0.75, Y: 0.30, Z: -0.30}, Color: solidlens.RGB(0.62, 0.74, 0.92), Intensity: 0.45},
		{Direction: solidlens.Vec{X: 0.20, Y: -0.85, Z: 0.15}, Color: solidlens.RGB(0.45, 0.55, 0.75), Intensity: 0.30},
	}
}

// View is where the camera stands, in the spherical terms every example is
// aimed with: an elevation above the plane the part is centred in, an azimuth
// about the vertical axis, and a reach that is a multiple of the part's own
// size so a bigger gear is framed the same as a smaller one.
//
// Use it where several parts have to be framed by ONE rule, so that a spur, a
// helical and a herringbone gear of the same tooth count come out the same size
// beside each other however thick they are. Where a single part should simply
// be as large as its frame allows, [Fit] solves for the camera instead.
type View struct {
	// Target is the point the camera looks at.
	Target solidlens.Vec
	// Size is the part's own extent, which Distance multiplies.
	Size float64
	// Distance is how many Sizes the camera stands back.
	Distance float64
	// ElevationDeg and AzimuthDeg place the camera on that sphere.
	ElevationDeg, AzimuthDeg float64
	// FOV is the vertical field of view in degrees.
	FOV float64
}

// Fit places a camera by what it has to hold, rather than by a distance a
// caller keeps in step by hand.
//
// The elevation, azimuth and field of view say where the camera stands and how
// wide it sees. Everything else is solved: the camera walks in until the meshes
// fill the frame bar Margin, and aims so that what it holds sits in the middle
// of it. A gear whose teeth stand proud of every point its lattice names is
// then framed correctly without anyone working out how far proud.
type Fit struct {
	// ElevationDeg and AzimuthDeg place the camera on a sphere about what it
	// is looking at, the elevation measured up from the horizontal.
	ElevationDeg, AzimuthDeg float64
	// FOV is the vertical field of view in degrees.
	FOV float64
	// Margin is the fraction of the half-frame left clear on whichever side
	// binds, so 0.05 leaves a twentieth of the way from the middle to the edge
	// empty. Zero puts the outermost point on the edge itself.
	Margin float64
	// Settings is the raster the frame is measured against; its width and
	// height set the aspect ratio.
	Settings solidlens.Settings
}

// fitSteps is how many times the distance bracket is halved, and fitAimSteps
// how many times the aim is corrected at each distance. The aim correction is a
// fixed point that converges quickly, because a small move of the target moves
// the picture by very nearly the same amount at every depth; six passes take it
// below a pixel at these sizes.
const (
	fitSteps    = 40
	fitAimSteps = 6
)

// Camera solves for the camera that frames the meshes.
func (f Fit) Camera(meshes ...solidlens.TriangleSource) (solidlens.Camera, error) {
	points, err := vertices(meshes)
	if err != nil {
		return solidlens.Camera{}, err
	}
	if f.Margin < 0 || f.Margin >= 1 {
		return solidlens.Camera{}, fmt.Errorf("margin %g is not a fraction of the frame below one", f.Margin)
	}
	if f.Settings.Width <= 0 || f.Settings.Height <= 0 {
		return solidlens.Camera{}, fmt.Errorf("framing needs a raster size, got %dx%d", f.Settings.Width, f.Settings.Height)
	}
	centre, radius := boundingSphere(points)
	if radius == 0 {
		return solidlens.Camera{}, fmt.Errorf("the meshes occupy a single point, which no distance frames")
	}
	l := newLens(f)

	// The bracket. Inside the bounding sphere the camera sits among the meshes
	// and frames nothing; from there outward the picture only shrinks as the
	// camera retreats, so what is wanted is the smallest distance that fits.
	// The far end starts at what would frame the sphere with the frame's own
	// half angle and doubles until it really does fit, which is what makes the
	// bisection's invariant true before it starts.
	near := radius * 1.001
	far := radius * (1 + 1/math.Tan(l.halfAngle))
	target, ok := l.frame(points, centre, far)
	for range fitSteps {
		if ok {
			break
		}
		far *= 2
		target, ok = l.frame(points, centre, far)
	}
	if !ok {
		return solidlens.Camera{}, fmt.Errorf("no distance frames these meshes at %g degrees of elevation", f.ElevationDeg)
	}
	for range fitSteps {
		middle := (near + far) / 2
		if aim, ok := l.frame(points, centre, middle); ok {
			far, target = middle, aim
			continue
		}
		near = middle
	}
	return solidlens.Camera{
		Position: target.Add(l.offset.Scale(far)),
		Target:   target,
		Up:       cameraUp,
		FOV:      f.FOV,
	}, nil
}

// cameraUp is the camera's up hint, which solidlens defaults to and which the
// frame arithmetic here has to use as well for its answer to be a fit for the
// picture that actually renders.
var cameraUp = solidlens.Vec{Z: 1}

// lens is the fixed part of that arithmetic: where the camera stands relative
// to what it looks at, and how a point in front of it lands in the frame. It
// restates solidlens's own projection, because a fit computed by any other
// projection is a fit for a picture nobody is going to render.
type lens struct {
	offset        solidlens.Vec // unit, from the target toward the camera
	right, upward solidlens.Vec
	focal, aspect float64
	halfAngle     float64
	limit         float64 // the largest frame coordinate kept, after the margin
}

func newLens(f Fit) lens {
	elevation := f.ElevationDeg * math.Pi / 180
	azimuth := f.AzimuthDeg * math.Pi / 180
	offset := solidlens.Vec{
		X: math.Cos(elevation) * math.Cos(azimuth),
		Y: math.Cos(elevation) * math.Sin(azimuth),
		Z: math.Sin(elevation),
	}
	forward := offset.Scale(-1)
	right, _ := forward.Cross(cameraUp).Normalize()
	upward, _ := right.Cross(forward).Normalize()
	return lens{
		offset:    offset,
		right:     right,
		upward:    upward,
		focal:     1 / math.Tan(f.FOV*math.Pi/360),
		aspect:    float64(f.Settings.Width) / float64(f.Settings.Height),
		halfAngle: f.FOV * math.Pi / 360,
		limit:     1 - f.Margin,
	}
}

// frame returns where a camera at the given distance has to aim for the points
// to sit in the middle of its frame, and whether they fit there.
func (l lens) frame(points []solidlens.Vec, centre solidlens.Vec, distance float64) (solidlens.Vec, bool) {
	target := centre
	var loX, hiX, loY, hiY float64
	for range fitAimSteps {
		position := target.Add(l.offset.Scale(distance))
		loX, hiX = math.Inf(1), math.Inf(-1)
		loY, hiY = math.Inf(1), math.Inf(-1)
		for _, p := range points {
			relative := p.Sub(position)
			depth := -relative.Dot(l.offset)
			if depth <= 0 {
				return target, false
			}
			x := relative.Dot(l.right) * l.focal / (depth * l.aspect)
			y := relative.Dot(l.upward) * l.focal / depth
			loX, hiX = math.Min(loX, x), math.Max(hiX, x)
			loY, hiY = math.Min(loY, y), math.Max(hiY, y)
		}
		// Move the aim by however far off centre the picture came out,
		// converted back to the world at the target's own depth. That
		// conversion is exact only there, which is why this repeats rather
		// than correcting once.
		target = target.
			Add(l.right.Scale((loX + hiX) / 2 * distance * l.aspect / l.focal)).
			Add(l.upward.Scale((loY + hiY) / 2 * distance / l.focal))
	}
	return target, (hiX-loX)/2 <= l.limit && (hiY-loY)/2 <= l.limit
}

// vertices gathers every mesh vertex into one slice.
func vertices(meshes []solidlens.TriangleSource) ([]solidlens.Vec, error) {
	if len(meshes) == 0 {
		return nil, fmt.Errorf("framing needs at least one mesh")
	}
	var out []solidlens.Vec
	for _, m := range meshes {
		out = append(out, m.Vertices()...)
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("framing needs at least one vertex")
	}
	return out, nil
}

// boundingSphere is the bounding box's centre and the distance from it to the
// furthest point. It only has to bound, not to be the smallest such sphere: it
// is the bracket the distance search starts from.
func boundingSphere(points []solidlens.Vec) (solidlens.Vec, float64) {
	lo, hi := points[0], points[0]
	for _, p := range points {
		lo = solidlens.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = solidlens.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	centre := lo.Add(hi).Scale(0.5)
	radius := 0.0
	for _, p := range points {
		radius = math.Max(radius, p.Sub(centre).Len())
	}
	return centre, radius
}

// Camera is the solidlens camera the view describes.
func (v View) Camera() solidlens.Camera {
	reach := v.Size * v.Distance
	elevation := v.ElevationDeg * math.Pi / 180
	azimuth := v.AzimuthDeg * math.Pi / 180
	horizontal := reach * math.Cos(elevation)
	return solidlens.Camera{
		Position: solidlens.Vec{
			X: v.Target.X + horizontal*math.Cos(azimuth),
			Y: v.Target.Y + horizontal*math.Sin(azimuth),
			Z: v.Target.Z + reach*math.Sin(elevation),
		},
		Target: v.Target,
		Up:     solidlens.Vec{Z: 1},
		FOV:    v.FOV,
	}
}

// WritePNG renders the scene to path.
func WritePNG(ctx context.Context, path string, scene solidlens.Scene, settings solidlens.Settings) error {
	f, err := os.Create(path) //nolint:gosec // the path is a flag-controlled output directory
	if err != nil {
		return fmt.Errorf("create %s: %w", path, err)
	}
	renderErr := solidlens.RenderPNG(ctx, f, scene, settings)
	closeErr := f.Close()
	if renderErr != nil {
		return fmt.Errorf("render %s: %w", path, renderErr)
	}
	if closeErr != nil {
		return fmt.Errorf("close %s: %w", path, closeErr)
	}
	return nil
}

// MeshOfBody tessellates a decad body for rendering. The tolerance is a chord
// tolerance in millimetres: it binds the curved faces of a body decad built
// analytically, and is only a floor for one whose mesh decad already holds,
// such as a boolean result.
func MeshOfBody(ctx context.Context, body *decad.Body, toleranceMM float64) (*solidlens.Mesh, error) {
	mesh, err := body.TessellateContext(ctx, units.Millimeters(toleranceMM))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	out, err := solidlens.MeshFrom(mesh)
	if err != nil {
		return nil, fmt.Errorf("convert mesh: %w", err)
	}
	return out, nil
}
