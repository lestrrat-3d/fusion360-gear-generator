package bevelgear_test

import (
	"flag"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// One picture per step, for proof/bevelgear/README.md.
//
// This is not a proof and it is skipped unless -snapshot.out names a directory.
// Every picture is taken of geometry a registered step built: a sketch step's
// picture is the sketch that step drew and solved, and a solid step's picture is
// the bodies that step returned. Nothing here builds a gear of its own, so a
// change to a step moves its picture, and a step renamed out from under this
// file fails the build rather than going on showing the old shape.
//
// What a picture therefore shows is the PROOF's model, which is not Fusion's
// output. The substitutions are stated at the top of solids_test.go; three of
// them are visible in these images and the README repeats them:
//
//   - Each solid of revolution is a 32-gon sweep (sweepFacets), so a frustum
//     has visible flats rather than a round side.
//   - The tooth's sections are perpendicular to the shaft axis rather than on
//     the back cone, and the loft starts at an apex STUB rather than a point.
//   - No boolean is performed. A step that would union, trim or pierce builds
//     its operands and lays them apart along the shaft axis, which is why
//     [solidSnapshot.LaidAside] exists: a picture gathers those operands back
//     to where the step measured them from, so the image shows the operands in
//     place and overlapping rather than strung along the axis.
//
// FOUR registered steps get no picture, and in every case it is that step's own
// substitution that leaves nothing to photograph. The README carries them as
// rows without an image, naming the same reasons:
//
//   - stepCutConicalEnds returns the tooth and the two cut cones it would be
//     trimmed by. Each cone reaches 3 * (dedendum radius) * tan(gamma) back
//     from its own apex, so it is several times the size of the tooth and, with
//     no transparency in the renderer, an opaque cone is the whole picture.
//     The trim itself is not performed, so the tooth in it is the same uncut
//     loft S17 already shows.
//   - stepLoftSpiralTooth lofts the slabs pairwise in an order it recomputes
//     after the twist. The BANDS it returns are the slabs stepCrownSlabs
//     already built, from the same stations at the same twist and crown, so its
//     picture came out byte-identical to that step's.
//   - stepCircularPattern applies ONE pattern increment to the seed tooth
//     rather than making N copies, and the increment is a rotation of the only
//     body in frame.
//   - stepMeshRotation rotates the frustum's bands — a solid of revolution —
//     about their own axis, which leaves every pixel where it was.
// ---------------------------------------------------------------------------

var snapshotOut = flag.String("snapshot.out", "",
	"directory the per-step snapshot images are written to; the snapshots are skipped when it is empty")

// snapshotTolerance is the chord tolerance every body is tessellated at. It is
// chosen for how smooth a 30 mm cone has to look at the output size, and no
// measurement is taken off these meshes.
const snapshotTolerance = 0.04

// snapshotMargin is how much of the half-frame is left clear around a step's
// bodies. It is wider than the README picture's own margin because these images
// are read one at a time rather than as a row of table cells.
const snapshotMargin = 0.08

// snapshotElevationDeg and snapshotAzimuthDeg place the camera for every solid
// step, so the sequence reads as one model seen from one place. The elevation is
// above the toe end and the azimuth stands the camera off the axial plane, which
// is what opens both the tooth's flank and the frustum's end faces in the same
// frame.
const (
	snapshotElevationDeg = 22
	snapshotAzimuthDeg   = -58
	snapshotFOV          = 32
)

// snapshotPalette colours a step's bodies in the order the step returned them,
// so one body is told from the next where two of them meet. It is not a legend:
// body 0 of one step and body 0 of another are not the same part.
var snapshotPalette = []solidlens.Color{
	solidlens.RGB(0.13, 0.45, 0.35),
	solidlens.RGB(0.35, 0.64, 0.45),
	solidlens.RGB(0.20, 0.34, 0.52),
	solidlens.RGB(0.58, 0.44, 0.20),
	solidlens.RGB(0.48, 0.26, 0.40),
	solidlens.RGB(0.24, 0.52, 0.54),
	solidlens.RGB(0.62, 0.32, 0.26),
	solidlens.RGB(0.40, 0.48, 0.24),
	solidlens.RGB(0.30, 0.30, 0.46),
}

// sketchSnapshot is one picture of a sketch step. Case names the row of that
// step's own proof table the parameters come from, which is what the README
// prints beside the image.
type sketchSnapshot struct {
	Step   string
	File   string
	Case   string
	Params map[string]float64
	Draw   func(t *testing.T, p map[string]float64) *sketch.Sketch
	// Options are applied after the shared ones and therefore override them,
	// for a sketch whose own shape defeats a default. Each is given a reason
	// where it is set.
	Options []sketch.SVGOption
}

// solidSnapshot is one picture of a solid step. LaidAside says whether the step
// lays its operands apart along the shaft axis, which the picture undoes; see
// the note at the top of this file.
type solidSnapshot struct {
	Step      string
	File      string
	Case      string
	Params    map[string]float64
	Build     proofkit3d.Build
	LaidAside bool
}

// drawnBy adapts a registered sketch step to a snapshot's Draw. The step is
// handed the same empty sketch on the world XY plane that proofkit hands it, and
// it draws, solves and checks that sketch itself, so the picture is of solved
// geometry the step's own assertions have already passed.
func drawnBy(step proofkit.Build) func(*testing.T, map[string]float64) *sketch.Sketch {
	return func(t *testing.T, p map[string]float64) *sketch.Sketch {
		s := proofkit.NewSketch(t)
		step(t, s, p)
		return s
	}
}

// sketchSnapshots are the sketch steps, in step order.
//
// Every case is the shipped dialog default — module 1, an equal 31/31 pair at a
// 90 degree shaft angle, a 35 degree right-hand spiral — because a sequence of
// pictures is only a sequence if one gear runs through all of it. The per-gear
// steps are taken on the PINION, the member the generator builds first.
var sketchSnapshots = []sketchSnapshot{
	{
		Step: "S8", File: "s08-anchor-sketch", Case: "default",
		Params: params(nil), Draw: drawnBy(stepAnchorSketch),
	},
	{
		Step: "S10", File: "s10-gear-profiles", Case: "default_31_31_at_90",
		Params: params(nil), Draw: drawnBy(stepGearProfiles),
		// Thirteen dimensions inside one 30 mm figure. Drawn together their
		// labels overlap into a block of text with the lattice behind it.
		Options: []sketch.SVGOption{sketch.WithDimensions(false)},
	},
	{
		Step: "S12", File: "s12-tooth-section", Case: "default_31_31_embedded_pinion",
		Params: withSide(params(nil), pinionSide), Draw: drawToothSection,
		// The flanks are splines through sampled points, one marker each, and
		// the tooth is 2 mm wide in a 16 mm frame: the markers cover the curve
		// they are sampling.
		Options: []sketch.SVGOption{sketch.WithShowPoints(false)},
	},
	{
		Step: "S15", File: "s15-profile-hexagon", Case: "default_31_31_at_90_pinion",
		Params: withSide(params(nil), pinionSide), Draw: drawnBy(stepGearProfileHexagon),
	},
	{
		Step: "S20", File: "s20-cone-element", Case: "default_31_31_at_90_pinion",
		Params: withSide(params(nil), pinionSide), Draw: drawnBy(stepConeElementSketch),
	},
	{
		Step: "S22", File: "s22-tooth-trace", Case: "psi_35_right_auto_cutter",
		Params: params(nil), Draw: drawnBy(stepSpiralTrace),
		// The cutter diameter resolves to 36.897890490184324 mm and the label
		// carries every digit of it across the figure. The fill is dropped with
		// it: the only closed region here is the ring between the two apex
		// reference circles, which is no profile anything consumes.
		Options: []sketch.SVGOption{sketch.WithDimensions(false), sketch.WithProfileFill(false)},
	},
	{
		Step: "S30", File: "s30-bore-sketch", Case: "auto_from_pitch_diameter_pinion",
		Params: withSide(params(nil), pinionSide), Draw: drawnBy(stepBoreSketch),
	},
}

// solidSnapshots are the solid steps, in step order.
//
// The cases are the same default pair the sketch pictures use, with one
// exception that the step's own table already makes: the Combine-Join is one
// boolean per tooth, so patternCases keeps the count at 8 and this picture does
// too. A ring of 8 teeth is also what shows the join at all, since at 31 teeth
// on a module 1 gear one tooth is a few pixels wide.
var solidSnapshots = []solidSnapshot{
	{
		Step: "S12", File: "s12-tooth-extrude", Case: "default_31_31_embedded_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepToothProfile,
	},
	{
		Step: "S16", File: "s16-gear-body", Case: "default_31_31_at_90_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepRevolveGearBody, LaidAside: true,
	},
	{
		Step: "S17", File: "s17-tooth-loft", Case: "default_31_31_at_90_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepLoftToothBody,
	},
	{
		Step: "S23", File: "s23-slice-slabs", Case: "psi_35_right_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepSliceToothSlabs,
	},
	{
		Step: "S24", File: "s24-drop-apex-scrap", Case: "psi_35_right_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepDropApexScrap,
	},
	{
		Step: "S25", File: "s25-twist-slabs", Case: "psi_35_right_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepTwistSlabs,
	},
	{
		Step: "S26", File: "s26-crown-slabs", Case: "psi_35_right_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepCrownSlabs,
	},
	{
		Step: "S29", File: "s29-combine-join", Case: "small_teeth_8_8_pinion",
		Params: withSide(params(map[string]float64{keyDrivingTeeth: 8, keyPinionTeeth: 8}), pinionSide),
		Build:  stepCombineJoin, LaidAside: true,
	},
	{
		Step: "S31", File: "s31-bore-cut", Case: "auto_from_pitch_diameter_pinion",
		Params: withSide(params(nil), pinionSide), Build: stepBoreCut, LaidAside: true,
	},
}

func TestStepSnapshots(t *testing.T) {
	if *snapshotOut == "" {
		t.Skip("no -snapshot.out directory; the step snapshots are not being regenerated")
	}
	for _, sn := range sketchSnapshots {
		t.Run(sn.File, func(t *testing.T) { writeSketchSnapshot(t, sn) })
	}
	for _, sn := range solidSnapshots {
		t.Run(sn.File, func(t *testing.T) { writeSolidSnapshot(t, sn) })
	}
}

// writeSketchSnapshot draws one sketch step and writes the sketch engine's own
// drawing of it.
//
// The sketch pictures are SVG where the solid ones are PNG, and the reason is
// what each exporter draws rather than a preference. The sketch engine's PNG
// rasterizer draws entities and point markers and nothing else: the dimensions
// and the profile fill are written by the SVG path alone, and a dimension is
// most of what a sketch step is — the length the lattice computed, held where
// the step puts it. The fill is the closed region a revolve or an extrude then
// consumes, which is the other thing a sketch step has to get right.
//
// Constraint glyphs are left off. The anchor sketch alone carries a midpoint, a
// horizontal and a distance on a 10 mm line, and the badges cover the line they
// are anchored to.
func writeSketchSnapshot(t *testing.T, sn sketchSnapshot) {
	s := sn.Draw(t, sn.Params)
	options := append(sketchStyle(s),
		sketch.WithDimensions(true),
		sketch.WithProfileFill(true),
		sketch.WithPixelWidth(sketchPixelWidth),
	)
	svg, err := s.SVG(append(options, sn.Options...)...)
	if err != nil {
		t.Fatalf("%s render: %v", sn.Step, err)
	}
	path := filepath.Join(*snapshotOut, sn.File+".svg")
	if err := os.WriteFile(path, []byte(svg), 0o644); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s: %s, case %s", path, sn.Step, sn.Case)
}

// writeSolidSnapshot builds one solid step's bodies and renders them.
func writeSolidSnapshot(t *testing.T, sn solidSnapshot) {
	d := newDesign(t, sn.Params)
	g, f := sideOf(d, sn.Params)
	doc := decad.New()
	bodies := sn.Build(t, doc, sn.Params)

	parts := make([]render.Part, 0, len(bodies))
	meshes := make([]solidlens.TriangleSource, 0, len(bodies))
	for i, body := range bodies {
		mesh, err := render.MeshOfBody(t.Context(), body, snapshotTolerance)
		if err != nil {
			t.Fatalf("%s body %d: %v", sn.Step, i, err)
		}
		if sn.LaidAside {
			mesh = gathered(t, mesh, -asideOffset(g, f, i))
		}
		parts = append(parts, render.Part{Mesh: mesh, Color: snapshotPalette[i%len(snapshotPalette)]})
		meshes = append(meshes, mesh)
	}

	scene := render.Scene(snapshotView(t, meshes...), parts...)
	path := filepath.Join(*snapshotOut, sn.File+".png")
	if err := render.WritePNG(t.Context(), path, scene, renderSettings); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s: %s, case %s", path, sn.Step, sn.Case)
}

// gathered moves a laid-apart body back along the shaft axis by the offset the
// step laid it aside with, so the picture shows the operands where the step's
// own assertions measure them from. The move is a translation along the axis and
// changes nothing else, which is the same reason the lay-apart scheme is allowed
// to make it in the first place.
func gathered(t *testing.T, mesh *solidlens.Mesh, dz float64) *solidlens.Mesh {
	t.Helper()
	if dz == 0 {
		return mesh
	}
	tr, err := r3.Translation(r3.NewVec(0, 0, dz))
	if err != nil {
		t.Fatalf("gather by %.4f: %v", dz, err)
	}
	moved, err := render.Placed(mesh, tr)
	if err != nil {
		t.Fatalf("gather by %.4f: %v", dz, err)
	}
	return moved
}

// snapshotView frames a step's bodies on what they actually occupy. Each step is
// framed on its own extent, so a tooth and a whole gear body both fill their
// image and the scale differs between pictures.
func snapshotView(t *testing.T, meshes ...solidlens.TriangleSource) solidlens.Camera {
	t.Helper()
	camera, err := render.Fit{
		ElevationDeg: snapshotElevationDeg,
		AzimuthDeg:   snapshotAzimuthDeg,
		FOV:          snapshotFOV,
		Margin:       snapshotMargin,
		Settings:     renderSettings,
	}.Camera(meshes...)
	if err != nil {
		t.Fatalf("frame the step: %v", err)
	}
	return camera
}

// sketchStrokeFraction, sketchPointFraction and sketchMarginFraction are the
// stroke width, the point marker's radius and the blank border, each as a
// fraction of the drawing's own long side. They are fractions rather than
// lengths because the sketch engine takes all three in SKETCH UNITS while the
// raster it writes fits the drawing's long side to a fixed pixel width: a
// millimetre is 23 pixels on the 40 mm lattice and 250 on the 4 mm tooth
// section, so one fixed stroke is a hairline in one picture and covers the
// geometry in the other. A fraction lands on the same pixel width in both.
const (
	sketchStrokeFraction = 0.0035
	sketchPointFraction  = 0.005
	sketchMarginFraction = 0.15
)

// sketchPixelWidth is the width the SVG asks to be displayed at. Without it the
// drawing carries its own sketch units into the page, so a 4 mm tooth section
// would be laid out four millimetres wide.
const sketchPixelWidth = 900

// sketchStyle sizes a sketch's stroke, markers and margin from what the sketch
// occupies. A sketch that occupies a single point — nothing this file draws, but
// the arithmetic below would divide a drawing into it — keeps the engine's own
// defaults.
func sketchStyle(s *sketch.Sketch) []sketch.SVGOption {
	span := sketchSpan(s)
	if span <= 0 {
		return nil
	}
	return []sketch.SVGOption{
		sketch.WithStrokeWidth(sketchStrokeFraction * span),
		sketch.WithPointRadius(sketchPointFraction * span),
		sketch.WithMargin(sketchMarginFraction * span),
	}
}

// sketchSpan is the long side of what the sketch occupies: every authored and
// reference point, with each circle taken out to its own radius. An arc bulges
// past its endpoints and a spline past its control points, by less than the
// margin the style then adds, so neither is walked.
func sketchSpan(s *sketch.Sketch) float64 {
	lo, hi := v2(math.Inf(1), math.Inf(1)), v2(math.Inf(-1), math.Inf(-1))
	grow := func(x, y float64) {
		lo = v2(math.Min(lo.X, x), math.Min(lo.Y, y))
		hi = v2(math.Max(hi.X, x), math.Max(hi.Y, y))
	}
	for _, p := range s.Points() {
		grow(p.X(), p.Y())
	}
	for _, e := range s.Entities() {
		c, ok := e.(*sketch.Circle)
		if !ok {
			continue
		}
		grow(c.Center.X()-c.R(), c.Center.Y()-c.R())
		grow(c.Center.X()+c.R(), c.Center.Y()+c.R())
	}
	if math.IsInf(lo.X, 1) {
		return 0
	}
	return math.Max(hi.X-lo.X, hi.Y-lo.Y)
}

// drawToothSection draws the `{gearLabel} Tooth` sketch S12 draws, through the
// same drawSection the step itself calls. The curved loop is drawn rather than
// the chorded twin, because the picture is of the sketch Fusion is asked for and
// the chording exists only so decad can integrate the solid.
func drawToothSection(t *testing.T, p map[string]float64) *sketch.Sketch {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	w := sketch.NewWorld()
	s, _ := drawSection(t, w, sectionPlane(t, w, f, 1), newToothOutline(d, g), 1, 0, 1, false)
	return s
}
