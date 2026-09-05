// Command genexamples renders the README's gear example images.
//
// The geometry is not decorative. Every tooth flank is sampled from
// proof/involute — the package the 3D proofs draw their teeth from — so the
// pictures show the involute the generator actually cuts, at the parameters
// named beside each image. Each gear is swept the way its spec sweeps it:
// spur straight through, helical twisting to the far face, herringbone
// twisting to mid-body and back.
//
// Only the three gears whose tooth math lives in proof/involute are rendered
// here. The bevel gear and the cycloidal drive draw their shapes from their own
// proof packages instead, so each is rendered by an opt-in test inside the
// package that already holds its geometry; render_examples.sh runs both stages.
//
// Run it through that script rather than by hand:
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

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
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
		if err := render.WritePNG(ctx, path, scene, settings); err != nil {
			return err
		}
		fmt.Printf("wrote %s\n", path)
	}
	return nil
}

// scene builds the gear and stages it. The camera is derived from the gear's
// own size so every example is framed identically no matter how thick it is:
// it looks down on the gear from the front left at a fixed elevation, standing
// back far enough that the whole tip circle and the full thickness fit whatever
// the example's proportions are.
func (ex example) scene(ctx context.Context) (solidlens.Scene, error) {
	mesh, err := ex.gear.mesh(ctx)
	if err != nil {
		return solidlens.Scene{}, err
	}
	view := render.View{
		Target:       solidlens.Vec{Z: ex.gear.thickness / 2},
		Size:         math.Hypot(ex.gear.tipRadius(), ex.gear.thickness),
		Distance:     3.4,
		ElevationDeg: 44,
		AzimuthDeg:   -58,
		FOV:          32,
	}
	return render.Scene(view.Camera(), render.Part{Mesh: mesh, Color: ex.color}), nil
}

// tipRadius is the outermost radius the camera has to fit in frame.
func (g gear) tipRadius() float64 {
	return g.module * (float64(g.teeth)/2 + 1)
}
