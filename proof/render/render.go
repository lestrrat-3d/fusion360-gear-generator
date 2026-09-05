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

// FitTo fills in the view's Target and Size from what the meshes actually
// occupy: the target is the centre of their common bounding box and the size
// its half-diagonal.
//
// It is here so that framing survives a change to the thing being framed. A
// part whose reach a caller works out by hand — a bevel gear's, say, whose
// teeth stand proud of every point its lattice names — is framed by a number
// that a new tooth count quietly invalidates.
func (v View) FitTo(meshes ...solidlens.TriangleSource) (View, error) {
	if len(meshes) == 0 {
		return View{}, fmt.Errorf("fitting a view needs at least one mesh")
	}
	lo := solidlens.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := solidlens.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, m := range meshes {
		for _, p := range m.Vertices() {
			lo = solidlens.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
			hi = solidlens.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
		}
	}
	if !(lo.X <= hi.X) {
		return View{}, fmt.Errorf("fitting a view needs at least one vertex")
	}
	v.Target = solidlens.Vec{X: (lo.X + hi.X) / 2, Y: (lo.Y + hi.Y) / 2, Z: (lo.Z + hi.Z) / 2}
	v.Size = hi.Sub(lo).Len() / 2
	return v, nil
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
