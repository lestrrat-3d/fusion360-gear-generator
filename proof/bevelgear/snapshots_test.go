package bevelgear_test

import (
	"flag"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// One picture per step, for proof/bevelgear/README.md.
//
// This is not a proof and it is skipped unless -snapshot.out names a directory.
//
// The SOLID pictures are the gear the generator builds, not the bodies the
// proof builds. They are meshed by render_test.go's own model — the same one the
// README's bevel picture is drawn with — because that model performs what the
// proof substitutes for: the Gear Body is a real solid of revolution rather than
// a 32-gon sweep, the tooth is trimmed exactly on the two cones section 2's toe
// and heel edges lie on, the ring carries every tooth, and the bore is taken out
// of the revolve profile. Nothing is laid apart, because nothing here asks the
// evaluator for a boolean it would refuse.
//
// What the model still simplifies is the TOOTH'S SECTION PLANE. Fusion draws the
// virtual spur tooth on the `{gearLabel} Plane`, which is tilted from the
// axis-perpendicular plane by the pitch cone angle, and lofts to it from the
// Apex point. These pictures put that same tooth outline on axis-perpendicular
// sections scaled about the Apex, which is the Tredgold mapping newToothOutline
// already applies. The tooth's size, its curve inventory, its taper and both
// conical trims are the real ones; the tilt of the plane it is drawn on is not.
//
// The one SKETCH picture is drawn by the proof's own step, since that step draws
// the sketch the generator draws.
//
// The gear is the shipped dialog default with the Mean Spiral Angle at 0, which
// is a STRAIGHT bevel: module 1, an equal 31/31 pair at a 90 degree shaft angle.
// On that branch the generator never runs S19 to S27 — the tooth-body hook
// returns the conical end trims and no spiral geometry is built at all — so
// those steps have no picture here because the command does not take them.
// ---------------------------------------------------------------------------

var snapshotOut = flag.String("snapshot.out", "",
	"directory the per-step snapshot images are written to; the snapshots are skipped when it is empty")

// snapshotMargin is how much of the half-frame is left clear around a step's
// bodies. It is wider than the README picture's own margin because these images
// are read one at a time rather than as a row of table cells.
const snapshotMargin = 0.08

// snapshotElevationDeg and snapshotAzimuthDeg place the camera for most of the
// single-gear pictures, so the sequence reads as one model seen from one place.
// The elevation stands the camera above the toe and the azimuth takes it off the
// axial plane, which opens a tooth's flank and the heel face in one frame. Two
// steps move it, each for a reason given where it is moved, and the pair at the
// end is framed by renderView instead, which is the view that shows two gears
// meshing.
const (
	snapshotElevationDeg = 22
	snapshotAzimuthDeg   = -58
	snapshotFOV          = 32
)

// eye is where the camera stands for one picture.
type eye struct{ ElevationDeg, AzimuthDeg float64 }

// standingEye is the viewpoint the sequence is shot from.
func standingEye() eye { return eye{snapshotElevationDeg, snapshotAzimuthDeg} }

// The colours. A gear is one part, so its body and its teeth share a hue: the
// teeth take the lighter shade while they are still separate bodies, and the
// whole gear takes the darker one from the Combine-Join onward.
var (
	snapshotBodyColor  = solidlens.RGB(0.13, 0.45, 0.35)
	snapshotToothColor = solidlens.RGB(0.35, 0.64, 0.45)
)

// straightParams is the pair every picture is taken of: the dialog's own
// defaults with the Mean Spiral Angle at 0.
func straightParams() map[string]float64 {
	return params(map[string]float64{keySpiralAngle: 0})
}

// straightCase is that pair's pinion, the member the generator builds first.
func straightCase() map[string]float64 {
	return withSide(straightParams(), pinionSide)
}

// sketchSnapshot is one picture of a sketch step.
type sketchSnapshot struct {
	Step string
	File string
	Draw func(t *testing.T, p map[string]float64) *sketch.Sketch
	// Options are applied after the shared ones and therefore override them,
	// for a sketch whose own shape defeats a default. Each is given a reason
	// where it is set.
	Options []sketch.SVGOption
}

// solidSnapshot is one picture of the gear as a step leaves it.
type solidSnapshot struct {
	Step  string
	File  string
	Scene func(t *testing.T) solidlens.Scene
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

// sketchSnapshots is the one sketch worth a picture of its own.
//
// The other sketches a straight bevel draws are each a figure whose caption
// would be longer than the figure: the Anchor Line is a line, the frustum
// hexagon is the outline the very next picture revolves, the bore is a circle,
// and the tooth section is a 2 mm tooth in a 16 mm frame. The lattice is the
// one that carries something no solid picture shows, because every length the
// gear is built from is in it.
var sketchSnapshots = []sketchSnapshot{
	{
		Step: "S10", File: "s10-gear-profiles", Draw: drawGearProfiles,
		// The vertex names are drawn and the thirteen dimensions are not.
		// Thirteen dimension labels inside one 30 mm figure overlap into a block
		// of text with the lattice behind it, and the names are what a reader
		// holding instructions.md needs: that document argues about point C and
		// point D by those letters.
		Options: []sketch.SVGOption{sketch.WithDimensions(false), sketch.WithLabels(true)},
	},
}

// drawGearProfiles draws the lattice S10 draws and then clears every name but
// the section 2 vertices.
//
// The step names all of its geometry: a construction line called `Apex->B` and
// its endpoints called `Apex->B.start` and `Apex->B.end`. Labelled all together
// that is a hundred names over a 30 mm figure, and the lattice disappears under
// its own text.
// Clearing the rest happens on the picture's own copy of the sketch, after the
// step has drawn and checked it, and nothing the proof asserts reads a name.
func drawGearProfiles(t *testing.T, p map[string]float64) *sketch.Sketch {
	s := proofkit.NewSketch(t)
	stepGearProfiles(t, s, p)
	for _, pt := range s.Points() {
		if !latticeVertex.MatchString(pt.Name()) {
			pt.SetName("")
		}
	}
	for _, e := range s.Entities() {
		e.SetName("")
	}
	return s
}

// latticeVertex matches the names section 2's own vertices carry: one capital
// letter, or either apex. Everything else the step names is derived from a line.
var latticeVertex = regexp.MustCompile(`^([A-Z]|Apex|Apex 2)$`)

// solidSnapshots are the body steps a straight bevel runs, in step order. Each
// scene is the gear as that step leaves it, so the sequence is cumulative.
var solidSnapshots = []solidSnapshot{
	{Step: "S16", File: "s16-gear-body", Scene: sceneGearBody},
	{Step: "S17", File: "s17-tooth-loft", Scene: sceneToothLoft},
	{Step: "S18", File: "s18-conical-trims", Scene: sceneConicalTrims},
	{Step: "S28", File: "s28-circular-pattern", Scene: sceneCircularPattern},
	{Step: "S29", File: "s29-combine-join", Scene: sceneCombineJoin},
	{Step: "S31", File: "s31-bore-cut", Scene: sceneBoreCut},
	{Step: "S32", File: "s32-meshing-rotation", Scene: sceneMeshingRotation},
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

// ---------------------------------------------------------------------------
// The scenes, one per body step.
// ---------------------------------------------------------------------------

// sceneGearBody is the Gear Body as the revolve leaves it: the hexagon spun a
// full turn, with no bore in it yet.
func sceneGearBody(t *testing.T) solidlens.Scene {
	_, _, f := snapshotGear(t)
	return staged(t, standingEye(),
		render.Part{Mesh: revolved(t, frustumPolygon(f)), Color: snapshotBodyColor})
}

// sceneToothLoft is the Tooth Body the loft produces: one tooth from the Apex
// out to the tooth profile at the heel, with neither end trimmed. It runs past
// the Gear Body at both ends, which is what S18 is there to cut back.
func sceneToothLoft(t *testing.T) solidlens.Scene {
	d, g, f := snapshotGear(t)
	section, _ := toothEnds(t, d, g)
	return staged(t, toothEye(section),
		render.Part{Mesh: revolved(t, frustumPolygon(f)), Color: snapshotBodyColor},
		render.Part{Mesh: uncutTooth(t, d, g, f), Color: snapshotToothColor})
}

// sceneConicalTrims is that tooth cut flush with the Gear Body's own toe and
// heel faces, by the two cones those faces lie on.
func sceneConicalTrims(t *testing.T) solidlens.Scene {
	d, g, f := snapshotGear(t)
	section, _ := toothEnds(t, d, g)
	return staged(t, toothEye(section),
		render.Part{Mesh: revolved(t, frustumPolygon(f)), Color: snapshotBodyColor},
		render.Part{Mesh: oneTooth(t, d, g, f), Color: snapshotToothColor})
}

// sceneCircularPattern is that tooth patterned into all 31 of them.
func sceneCircularPattern(t *testing.T) solidlens.Scene {
	d, g, f := snapshotGear(t)
	return staged(t, standingEye(),
		render.Part{Mesh: revolved(t, frustumPolygon(f)), Color: snapshotBodyColor},
		render.Part{Mesh: ring(t, d, g, f), Color: snapshotToothColor})
}

// sceneCombineJoin is the same geometry in one colour, which is what the join
// makes of it: the body and its 31 teeth stop being separate bodies.
func sceneCombineJoin(t *testing.T) solidlens.Scene {
	d, g, f := snapshotGear(t)
	return staged(t, standingEye(),
		render.Part{Mesh: revolved(t, frustumPolygon(f)), Color: snapshotBodyColor},
		render.Part{Mesh: ring(t, d, g, f), Color: snapshotBodyColor})
}

// sceneBoreCut is the finished single gear, with the bore taken out along the
// shaft axis through the whole body.
func sceneBoreCut(t *testing.T) solidlens.Scene {
	d, g, f := snapshotGear(t)
	return staged(t, boreEye(),
		render.Part{Mesh: revolved(t, frustumProfile(g, f)), Color: snapshotBodyColor},
		render.Part{Mesh: ring(t, d, g, f), Color: snapshotBodyColor})
}

// sceneMeshingRotation is the pair, which is the only thing the meshing rotation
// shows: the driving gear turned half a tooth pitch about its own shaft, so its
// valley meets the pinion's tooth.
//
// Both members come out of the one case, exactly as the command builds them, and
// are placed by the same placement the README picture uses.
func sceneMeshingRotation(t *testing.T) solidlens.Scene {
	base := straightParams()
	d := newDesign(t, base)

	var parts []render.Part
	var meshes []solidlens.TriangleSource
	for _, member := range renderSides {
		p := withSide(base, member.side)
		g, f := sideOf(d, p)
		place := placement(t, d, g, p)
		for _, mesh := range gearMeshes(t, d, g, f) {
			moved, err := render.Placed(mesh, place)
			if err != nil {
				t.Fatalf("place the %s gear: %v", g.Label, err)
			}
			parts = append(parts, render.Part{Mesh: moved, Color: member.color})
			meshes = append(meshes, moved)
		}
	}
	return render.Scene(renderView(t, meshes...), parts...)
}

// ---------------------------------------------------------------------------
// The meshes the scenes are built from.
// ---------------------------------------------------------------------------

// snapshotGear resolves the gear every single-gear picture is taken of.
func snapshotGear(t *testing.T) (design, gear, gearFrame) {
	t.Helper()
	p := straightCase()
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	return d, g, f
}

// revolved meshes a profile spun about the shaft axis, at the segment count the
// README picture uses.
func revolved(t *testing.T, profile []render.Vec2) *solidlens.Mesh {
	t.Helper()
	mesh, err := render.Revolve(profile, renderSegments)
	if err != nil {
		t.Fatalf("revolve the profile: %v", err)
	}
	return mesh
}

// oneTooth meshes a single tooth, trimmed flush at both ends by the cones the
// Gear Body's toe and heel faces lie on. It is the tooth toothRing repeats.
func oneTooth(t *testing.T, d design, g gear, f gearFrame) *solidlens.Mesh {
	t.Helper()
	section, ends := toothEnds(t, d, g)
	mesh, err := tooth(g, f, section, ends)
	if err != nil {
		t.Fatalf("%s tooth: %v", g.Label, err)
	}
	return mesh
}

// ring meshes the gear's whole ring of teeth.
func ring(t *testing.T, d design, g gear, f gearFrame) *solidlens.Mesh {
	t.Helper()
	mesh, err := toothRing(t, d, g, f)
	if err != nil {
		t.Fatalf("%s teeth: %v", g.Label, err)
	}
	return mesh
}

// uncutTooth meshes the Tooth Body the loft produces before either trim: the
// Apex point at one end, the tooth profile at the heel at the other.
//
// The Apex end is a genuine point here, so the tooth is a fan of triangles from
// that one vertex out to the heel section's ring rather than a prism between two
// rings. A section at cone-distance fraction k is the heel section scaled by k,
// which is what makes the straight ray from the Apex through a section point the
// tooth's own edge.
func uncutTooth(t *testing.T, d design, g gear, f gearFrame) *solidlens.Mesh {
	t.Helper()
	section, ends := toothEnds(t, d, g)

	vertices := make([]solidlens.Vec, 0, len(section)+1)
	vertices = append(vertices, solidlens.Vec{})
	for _, q := range section {
		vertices = append(vertices, solidlens.Vec{X: q.X, Y: q.Y, Z: f.Ded.X})
	}

	n := len(section)
	triangles := make([][3]int, 0, n+len(ends))
	for i := range n {
		triangles = append(triangles, [3]int{0, 1 + (i+1)%n, 1 + i})
	}
	for _, e := range ends {
		triangles = append(triangles, [3]int{1 + e[0], 1 + e[1], 1 + e[2]})
	}
	mesh, err := solidlens.NewMesh(vertices, triangles)
	if err != nil {
		t.Fatalf("%s uncut tooth: %v", g.Label, err)
	}
	return mesh
}

// toothEnds is the tooth's heel section and its triangulation, which every tooth
// mesh here is built from.
func toothEnds(t *testing.T, d design, g gear) ([]render.Vec2, [][3]int) {
	t.Helper()
	section := toothSection(newToothOutline(d, g))
	ends, err := render.EarClip(section)
	if err != nil {
		t.Fatalf("%s tooth section: %v", g.Label, err)
	}
	return section, ends
}

// staged frames one gear's parts and lights them, after a half turn about X that
// leaves the toe end — the end the teeth taper to — meeting a camera standing
// above it. That turn is the one the README picture applies, for the same
// reason, and it moves no part relative to another.
func staged(t *testing.T, from eye, parts ...render.Part) solidlens.Scene {
	t.Helper()
	flip := turn(t, r3.NewVec(1, 0, 0), math.Pi)
	placed := make([]render.Part, 0, len(parts))
	meshes := make([]solidlens.TriangleSource, 0, len(parts))
	for _, p := range parts {
		moved, err := render.Placed(p.Mesh, flip)
		if err != nil {
			t.Fatalf("stage a part: %v", err)
		}
		placed = append(placed, render.Part{Mesh: moved, Color: p.Color})
		meshes = append(meshes, moved)
	}
	return render.Scene(snapshotView(t, from, meshes...), placed...)
}

// toothEye is where a camera has to stand to face the single tooth the loft and
// the trims work on.
//
// A gear with one tooth on it has to be looked at from that tooth's own side,
// and one tooth is where the spur drawer left it rather than anywhere this file
// chooses, so the bearing is read off the section. The half turn [staged]
// applies mirrors it, and the picture is then taken a further 25 degrees round
// so the flank is seen at an angle rather than edge on.
func toothEye(section []render.Vec2) eye {
	var sx, sy float64
	for _, q := range section {
		sx, sy = sx+q.X, sy+q.Y
	}
	bearing := math.Atan2(sy, sx) * 180 / math.Pi
	return eye{snapshotElevationDeg, -bearing - toothViewOffsetDeg}
}

// boreEye is the one steeper viewpoint in the sequence. The bore runs along the
// shaft axis and comes out in the floor of the toe dish, which the dish's own
// rim hides from the standing viewpoint: at that elevation the hole covers a
// patch 27 pixels across. From boreElevationDeg the camera looks far enough down
// the axis to see through it.
func boreEye() eye { return eye{boreElevationDeg, snapshotAzimuthDeg} }

// boreElevationDeg is that steeper elevation.
const boreElevationDeg = 62

// toothViewOffsetDeg is how far round from the tooth's own bearing the camera
// is taken, so that a flank and the tooth's end face are both open to it.
const toothViewOffsetDeg = 25

// snapshotView frames a step's bodies on what they actually occupy. Each step is
// framed on its own extent, so one tooth and a whole gear both fill their image
// and the scale differs between pictures.
func snapshotView(t *testing.T, from eye, meshes ...solidlens.TriangleSource) solidlens.Camera {
	t.Helper()
	camera, err := render.Fit{
		ElevationDeg: from.ElevationDeg,
		AzimuthDeg:   from.AzimuthDeg,
		FOV:          snapshotFOV,
		Margin:       snapshotMargin,
		Settings:     renderSettings,
	}.Camera(meshes...)
	if err != nil {
		t.Fatalf("frame the step: %v", err)
	}
	return camera
}

// ---------------------------------------------------------------------------
// Writing the files.
// ---------------------------------------------------------------------------

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
	s := sn.Draw(t, straightCase())
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
	t.Logf("wrote %s: %s", path, sn.Step)
}

// writeSolidSnapshot renders one step's scene.
func writeSolidSnapshot(t *testing.T, sn solidSnapshot) {
	scene := sn.Scene(t)
	path := filepath.Join(*snapshotOut, sn.File+".png")
	if err := render.WritePNG(t.Context(), path, scene, renderSettings); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s: %s", path, sn.Step)
}

// ---------------------------------------------------------------------------
// Sketch drawing style.
// ---------------------------------------------------------------------------

// sketchStrokeFraction, sketchPointFraction and sketchMarginFraction are the
// stroke width, the point marker's radius and the blank border, each as a
// fraction of the drawing's own long side. They are fractions rather than
// lengths because the sketch engine takes all three in SKETCH UNITS while the
// drawing is displayed at a fixed pixel width, so a stroke set in millimetres is
// a hairline on a large figure and covers a small one. A fraction lands on the
// same pixel width whatever the figure measures.
const (
	sketchStrokeFraction = 0.0035
	sketchPointFraction  = 0.005
	sketchMarginFraction = 0.15
)

// sketchPixelWidth is the width the SVG asks to be displayed at. Without it the
// drawing carries its own sketch units into the page, so a 30 mm lattice would
// be laid out thirty millimetres wide.
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
