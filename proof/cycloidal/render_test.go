package cycloidal_test

import (
	"flag"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// The README's cycloidal drive picture.
//
// This is not a proof and it is skipped unless -render.out names a directory.
// Every body in it is built from drawing_geometry_test.go, which transcribes
// the same spec the proof compiles from; see that file's header for why the
// drawing keeps its own arithmetic instead of reaching into the generated
// proof.
//
// No boolean is performed. Every join and cut in this drive is one the proof
// itself builds as a profile loop or as two flush bodies, for reasons its own
// steps record, and a picture of bodies that meet flush and share a colour is
// the picture the join would give.
//
// The drive is drawn part-exploded: the housing, the discs and the cam stand
// where they are assembled, and the output plate is lifted clear on its own
// pins. Assembled, the casing wall stands as high as the disc stack and the
// plate closes the top, so a picture of it is a picture of a plain ring. The
// lift is a rigid move along the drive axis and changes nothing else.
// ---------------------------------------------------------------------------

var renderOut = flag.String("render.out", "",
	"directory the README example image is written to; the render is skipped when it is empty")

// renderCase is the drive the README shows: the dialog's own defaults with the
// second disc turned on, which is the configuration whose eccentric loads
// cancel and the one worth a picture.
func renderCase() map[string]float64 {
	return drawWith(map[string]float64{"discCount": 2})
}

// renderTolerance is the chord tolerance every body is tessellated at. It is
// chosen for how round a 90 mm ring has to look at the output size, and no
// measurement is taken off these meshes.
const renderTolerance = 0.08

// renderLift is how far the output plate and its pins are raised above the
// stack, in millimetres. It clears the pins' own length, so a pin hangs above
// the hole it seats into rather than reaching into it.
const renderLift = 34

// renderMargin is how much of the half-frame is left clear around the drive.
// The image is a table cell 320 pixels wide, so the parts have to be as large
// in it as they go.
const renderMargin = 0.05

// renderSettings is the raster the image is written at, which the framing has
// to know as well, because the aspect ratio is what decides whether the frame's
// width or its height binds.
var renderSettings = solidlens.Settings{Width: 960, Height: 720}

// The parts' colours. The drive is an assembly rather than a single gear, so
// each sub-component the generator creates gets its own.
var (
	housingColor = solidlens.RGB(0.20, 0.27, 0.42)
	discColor    = solidlens.RGB(0.62, 0.24, 0.29)
	camColor     = solidlens.RGB(0.74, 0.53, 0.13)
	plateColor   = solidlens.RGB(0.14, 0.42, 0.45)
)

func TestRenderExample(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example image is not being regenerated")
	}
	p := renderCase()
	doc := decad.New()

	var parts []render.Part
	add := func(color solidlens.Color, bodies ...*decad.Body) {
		for _, body := range bodies {
			parts = append(parts, render.Part{Mesh: renderMesh(t, body), Color: color})
		}
	}
	add(housingColor, renderHousing(t, doc, newDrawDims(p, 0))...)
	for disc := range int(p["discCount"]) {
		d := newDrawDims(p, disc)
		add(discColor, renderDisc(t, doc, d))
		add(camColor, renderCam(t, doc, d))
	}
	add(plateColor, renderLiftedOutput(t, doc, newDrawDims(p, 0))...)

	scene := render.Scene(renderView(t, parts), parts...)
	path := filepath.Join(*renderOut, "cycloidal.png")
	if err := render.WritePNG(t.Context(), path, scene, renderSettings); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s", path)
}

// renderHousing is the Housing, as its two extrudes: the base annulus below the
// stack and the casing ring around it, which meet flush at the housing plane.
//
// The generator joins them into the one printable part, and to do that it has
// to widen the base and sink it into the casing, since the engine refuses a
// union whose operands share a face and an outer cylinder. Drawn in one colour
// and left flush, the two bodies are the picture that join would produce,
// without the sliver the boolean needs.
func renderHousing(t *testing.T, doc *decad.Document, d drawDims) []*decad.Body {
	t.Helper()
	return []*decad.Body{
		drawAnnulusBody(t, doc, drawPt{}, d.drawHousingOuter(), drawPt{}, d.drawHousingInner(),
			-1-d.BaseT, d.BaseT, "Housing Ring"),
		drawRingAround(t, doc, d.drawHousingOuter(), drawCasingContour(d),
			0, d.drawStackTop(), "Ring Casing"),
	}
}

// renderDisc is the finished rotor disc: the lobe outline carrying both cuts
// the spec makes through it, the M output holes and the enlarged centre bore.
//
// The generator builds each cut as its own feature, and each has its own reason
// for being a profile loop rather than a boolean. Taking every loop into one
// extrude is the same solid as those features in sequence.
func renderDisc(t *testing.T, doc *decad.Document, d drawDims) *decad.Body {
	t.Helper()
	holes := drawHoles(drawHoleCentres(d), d.DHole/2)
	holes = append(holes, drawHole{C: d.drawCentre(), R: d.drawBoreRadius()})
	return drawHoledPrism(t, doc, drawDiscLoop(d), holes, d.drawDiscBase(), d.T, "Rotor Disc")
}

// renderCam is this disc's eccentric cam section: the cam outer on the disc
// centre, carrying the input-shaft bore on the drive axis when the dialog asks
// for one.
func renderCam(t *testing.T, doc *decad.Document, d drawDims) *decad.Body {
	t.Helper()
	c := d.drawCentre()
	if d.ISD <= 0 {
		return drawCylinderBody(t, doc, c, d.CBD/2, d.drawDiscBase(), d.drawCamHeight(),
			"Eccentric Cam section")
	}
	return drawAnnulusBody(t, doc, c, d.CBD/2, drawPt{}, d.ISD/2,
		d.drawDiscBase(), d.drawCamHeight(), "Eccentric Cam section")
}

// renderLiftedOutput is the output plate and its M pins, raised clear of the
// stack. The plate carries a socket per pin, which is the combine-cut the
// generator makes against it, and each pin runs the whole height it would seat
// through, so the lift shows a pin hanging over the hole it belongs in.
func renderLiftedOutput(t *testing.T, doc *decad.Document, d drawDims) []*decad.Body {
	t.Helper()
	centres := drawPinCentres(d)
	out := []*decad.Body{
		drawHoledDisc(t, doc, d.drawPlateRadius(), drawHoles(centres, d.DPin/2),
			d.drawStackTop()+1, d.PlateT, "Output Plate"),
	}
	for _, c := range centres {
		out = append(out, drawCylinderBody(t, doc, c, d.DPin/2, 0,
			d.drawStackTop()+1+d.PlateT, "Output Pin"))
	}

	lift, err := r3.Translation(r3.NewVec(0, 0, renderLift))
	if err != nil {
		t.Fatalf("lift translation: %v", err)
	}
	for i, body := range out {
		moved, err := body.Placed(lift)
		if err != nil {
			t.Fatalf("lift output body %d clear of the stack: %v", i, err)
		}
		out[i] = moved
	}
	return out
}

func renderMesh(t *testing.T, body *decad.Body) *solidlens.Mesh {
	t.Helper()
	mesh, err := render.MeshOfBody(t.Context(), body, renderTolerance)
	if err != nil {
		t.Fatalf("mesh a part: %v", err)
	}
	return mesh
}

// renderView frames the drive on what its parts actually occupy, the lift
// included. The elevation is high enough to see down into the ring, which is
// where the top disc sits.
func renderView(t *testing.T, parts []render.Part) solidlens.Camera {
	t.Helper()
	meshes := make([]solidlens.TriangleSource, 0, len(parts))
	for _, part := range parts {
		meshes = append(meshes, part.Mesh)
	}
	camera, err := render.Fit{
		ElevationDeg: 30,
		AzimuthDeg:   -58,
		FOV:          32,
		Margin:       renderMargin,
		Settings:     renderSettings,
	}.Camera(meshes...)
	if err != nil {
		t.Fatalf("frame the drive: %v", err)
	}
	return camera
}
