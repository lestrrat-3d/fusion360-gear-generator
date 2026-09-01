// Command genexamples renders the README's gear example images.
//
// The geometry is not decorative. Every tooth flank is sampled from
// proof/involute — the package the 3D proofs draw their teeth from — so the
// pictures show the involute the generator actually cuts, at the parameters
// named beside each image. Each gear is swept the way its spec sweeps it:
// spur straight through, helical twisting to the far face, herringbone
// twisting to mid-body and back.
//
// Only the three gears whose tooth math lives in proof/involute are rendered.
// Bevel and cycloidal derive their profiles elsewhere and have no 3D proof to
// draw from, so putting them here would mean inventing geometry.
//
// Run it from the proof module with the sketch engine wired up:
//
//	./render_examples.sh
package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/lestrrat-3d/solidlens"
)

// example pairs a gear with how it is presented.
type example struct {
	file  string
	gear  gear
	color solidlens.Color
}

func examples() []example {
	const (
		module    = 2
		teeth     = 24
		pressure  = 20 * math.Pi / 180
		bore      = 10
		helixTilt = 20 * math.Pi / 180
	)
	return []example{
		{
			file:  "spur.png",
			color: solidlens.RGB(0.05, 0.42, 0.62),
			gear: gear{
				name: "spur", module: module, teeth: teeth, pressureAngle: pressure,
				thickness: 12, boreDiameter: bore,
			},
		},
		{
			file:  "helical.png",
			color: solidlens.RGB(0.30, 0.22, 0.68),
			gear: gear{
				name: "helical", module: module, teeth: teeth, pressureAngle: pressure,
				thickness: 16, helixAngle: helixTilt, boreDiameter: bore,
			},
		},
		{
			file:  "herringbone.png",
			color: solidlens.RGB(0.72, 0.32, 0.06),
			gear: gear{
				name: "herringbone", module: module, teeth: teeth, pressureAngle: pressure,
				thickness: 20, helixAngle: helixTilt, herringbone: true, boreDiameter: bore,
			},
		},
	}
}

func main() {
	out := flag.String("out", "../docs/images/gears", "directory the PNGs are written to")
	width := flag.Int("width", 960, "output width in pixels")
	height := flag.Int("height", 720, "output height in pixels")
	flag.Parse()

	if err := run(context.Background(), *out, solidlens.Settings{Width: *width, Height: *height}); err != nil {
		fmt.Fprintf(os.Stderr, "genexamples: %s\n", err)
		os.Exit(1)
	}
}

func run(ctx context.Context, out string, settings solidlens.Settings) error {
	if err := os.MkdirAll(out, 0o755); err != nil {
		return fmt.Errorf("create output directory: %w", err)
	}
	for _, ex := range examples() {
		scene, err := ex.scene(ctx)
		if err != nil {
			return err
		}
		path := filepath.Join(out, ex.file)
		if err := writePNG(ctx, path, scene, settings); err != nil {
			return err
		}
		fmt.Printf("wrote %s\n", path)
	}
	return nil
}

func writePNG(ctx context.Context, path string, scene solidlens.Scene, settings solidlens.Settings) error {
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

// scene builds the gear and stages it. The camera is derived from the gear's
// own size so every example is framed identically no matter how thick it is.
func (ex example) scene(ctx context.Context) (solidlens.Scene, error) {
	mesh, err := ex.gear.mesh(ctx)
	if err != nil {
		return solidlens.Scene{}, err
	}
	return solidlens.Scene{
		Camera: ex.camera(),
		Models: []solidlens.Model{{
			Mesh:     mesh,
			Material: solidlens.Matte(ex.color),
			Edges:    ex.outline(),
		}},
		DirectionalLights: lights(),
		Background:        solidlens.RGB(0.35, 0.37, 0.42),
	}, nil
}

// outline draws the gear's own edges in a darkened version of its colour. It
// is what separates one tooth from the next where two flanks face the camera
// at nearly the same angle and flat shading gives them nearly the same value.
// The crease angle is left at its default, which picks up the tooth corners
// and skips the seams between the bands the sweep is cut into.
func (ex example) outline() solidlens.Edges {
	const darken = 0.28
	return solidlens.Edges{
		Enabled: true,
		Color:   solidlens.RGB(ex.color.R*darken, ex.color.G*darken, ex.color.B*darken),
		Width:   1.4,
	}
}

// camera looks down on the gear from the front left at a fixed elevation,
// standing back far enough that the whole tip circle and the full thickness
// fit whatever the example's proportions are.
func (ex example) camera() solidlens.Camera {
	radius := ex.gear.tipRadius()
	centre := ex.gear.thickness / 2
	reach := math.Hypot(radius, ex.gear.thickness) * 3.4
	const elevation = 44 * math.Pi / 180
	const azimuth = -58 * math.Pi / 180
	horizontal := reach * math.Cos(elevation)
	return solidlens.Camera{
		Position: solidlens.Vec{
			X: horizontal * math.Cos(azimuth),
			Y: horizontal * math.Sin(azimuth),
			Z: centre + reach*math.Sin(elevation),
		},
		Target: solidlens.Vec{Z: centre},
		Up:     solidlens.Vec{Z: 1},
		FOV:    32,
	}
}

// lights are all directional on purpose. solidlens shades a triangle flat,
// from one of its own corners, so a point light makes a large flat face band
// triangle by triangle as the distance changes across it. A directional light
// depends on the normal alone, which leaves the gear's end faces even and
// still separates the three tooth surfaces by their angle.
func lights() []solidlens.DirectionalLight {
	return []solidlens.DirectionalLight{
		{Direction: solidlens.Vec{X: 0.35, Y: 0.62, Z: -0.70}, Color: solidlens.RGB(1, 0.97, 0.92), Intensity: 0.95},
		{Direction: solidlens.Vec{X: -0.75, Y: 0.30, Z: -0.30}, Color: solidlens.RGB(0.62, 0.74, 0.92), Intensity: 0.45},
		{Direction: solidlens.Vec{X: 0.20, Y: -0.85, Z: 0.15}, Color: solidlens.RGB(0.45, 0.55, 0.75), Intensity: 0.30},
	}
}

// tipRadius is the outermost radius the camera has to fit in frame.
func (g gear) tipRadius() float64 {
	return g.module * (float64(g.teeth)/2 + 1)
}
