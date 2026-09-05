package cycloidal_test

import (
	"flag"
	"math"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/solidlens"
)

// ---------------------------------------------------------------------------
// The README's cycloidal drive picture.
//
// This is not a proof and it is skipped unless -render.out names a directory.
// It is here, in the proof's own package, because every body in the picture is
// one the proof already builds: the housing is stepExtrudeHousingBase's
// annulus under stepPatternCasingSectors' ring, the discs are
// stepPatternOutputHoles' profile carrying stepCutDiscBore's bore, the cam is
// the sections stepExtrudeCamSection extrudes, and the plate and its pins are
// stepPatternOutputPins' plate and stepExtrudeOutputPin's pin. Nothing is
// modelled twice, so a change to the proved geometry moves the picture with
// it.
//
// No boolean is performed. Every join and cut in this drive is one the proof
// itself builds as a profile loop or as two flush bodies, for reasons its own
// steps record, and a picture of bodies that meet flush and share a colour is
// the picture the join would give.
//
// The drive is drawn part-exploded: the housing, the two discs and the cam
// stand where they are assembled, and the output plate is lifted clear on its
// own pins. Assembled, the casing wall stands as high as the disc stack and the
// plate closes the top, so a picture of it is a picture of a plain ring. The
// lift is a rigid move along the drive axis and changes nothing else.
// ---------------------------------------------------------------------------

var renderOut = flag.String("render.out", "",
	"directory the README example image is written to; the render is skipped when it is empty")

// renderCase is the drive the README shows: the dialog's own defaults with the
// second disc turned on, which is the configuration whose eccentric loads
// cancel and the one worth a picture.
func renderCase() map[string]float64 {
	return baseCase(map[string]float64{pDiscCount: 2})
}

// renderTolerance is the chord tolerance every body is tessellated at. It is
// chosen for how round a 90 mm ring has to look at the output size, and no
// measurement is taken off these meshes.
const renderTolerance = 0.08

// renderLift is how far the output plate and its pins are raised above the
// stack, in millimetres. It clears the pins' own length, so a pin hangs above
// the hole it seats into rather than reaching into it.
const renderLift = 34

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
	d := derive(p)
	requireInRegime(t, d)
	doc := decad.New()

	var parts []render.Part
	add := func(color solidlens.Color, bodies ...*decad.Body) {
		for _, body := range bodies {
			parts = append(parts, render.Part{Mesh: renderMesh(t, body), Color: color})
		}
	}
	add(housingColor, housing(t, doc, d)...)
	for _, disc := range discsOf(p) {
		add(discColor, renderDisc(t, doc, disc))
		add(camColor, buildCamSection(t, doc, disc, camSectionDepth(disc)))
	}
	add(plateColor, liftedOutput(t, doc, d)...)

	scene := render.Scene(renderView(t, parts), parts...)
	path := filepath.Join(*renderOut, "cycloidal.png")
	if err := render.WritePNG(t.Context(), path, scene, solidlens.Settings{Width: 960, Height: 720}); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s", path)
}

// discsOf resolves the case once per disc of the stack. A case names the one
// disc a per-disc step is run for, through pDiscIndex, and derive turns that
// index into the disc's own eccentric sign and clocking; a picture of the whole
// drive needs every index, so it asks derive for each in turn rather than
// restating that rule.
func discsOf(p map[string]float64) []dims {
	count := int(p[pDiscCount])
	out := make([]dims, 0, count)
	for i := range count {
		out = append(out, derive(overriddenWith(p, pDiscIndex, float64(i))))
	}
	return out
}

// overriddenWith is p with one key replaced, so a per-disc case can be spelled
// without disturbing the one the test holds.
func overriddenWith(p map[string]float64, key string, value float64) map[string]float64 {
	out := make(map[string]float64, len(p)+1)
	for k, v := range p {
		out[k] = v
	}
	out[key] = value
	return out
}

// housing is the Housing, as its two extrudes: the base annulus and the casing
// ring, which meet flush at the housing plane.
//
// stepJoinHousing joins them into the one printable part the generator makes,
// and to do that it has to widen the base and sink it into the casing, since
// the engine refuses a union whose operands share a face and an outer cylinder.
// Drawn in one colour and left flush, the two bodies are the picture that join
// would produce, without the sliver the boolean needs.
func housing(t *testing.T, doc *decad.Document, d dims) []*decad.Body {
	t.Helper()
	return []*decad.Body{
		buildHousingBase(t, doc, d, d.Base, 0),
		buildCasingRing(t, doc, d, d.stackTop(), 1),
	}
}

// renderDisc is the finished rotor disc: the lobe outline carrying both cuts
// the spec makes through it, the M output holes and the centre bore.
//
// stepPatternOutputHoles and stepCutDiscBore each build one of those cuts, and
// each has its own reason for building it as a profile loop rather than as a
// boolean. Taking both loops into one extrude is the same solid as the two
// steps in sequence, which is what the generator's disc is.
func renderDisc(t *testing.T, doc *decad.Document, d dims) *decad.Body {
	t.Helper()
	s := buildSketch(t, d.zBase())
	chordLoop(s, d.lobeOutline())
	for _, c := range d.outputHoleCentres() {
		fixedCircle(s, c, d.DHole/2)
	}
	fixedCircle(s, d.centre(), (d.CBD+d.Clr)/2)
	return extrudeUp(t, doc, s, holedRegion(t, s, d.M+1), d.T)
}

// liftedOutput is the output plate and its M pins, raised clear of the stack.
//
// The pins are the one pin stepExtrudeOutputPin builds, copied round the drive
// axis: that copy IS the M-times circular pattern the spec makes of it, and
// building them as one extrude the way the plate's sockets are built would put
// M separate regions in one sketch, which decad's extrude does not take.
func liftedOutput(t *testing.T, doc *decad.Document, d dims) []*decad.Body {
	t.Helper()
	s := buildSketch(t, d.stackTop()+1)
	fixedCircle(s, pt{}, d.plateRadius())
	for _, c := range d.outputPinCentres() {
		fixedCircle(s, c, d.DPin/2)
	}
	out := []*decad.Body{extrudeUp(t, doc, s, holedRegion(t, s, d.M), d.PlateT)}

	seed := buildOutputPin(t, doc, d, 0)
	for i := range d.M {
		pin := seed
		if i > 0 {
			copied, err := seed.PlacedCopy(turnAbout(t, 0, 0, 2*math.Pi*float64(i)/float64(d.M)))
			if err != nil {
				t.Fatalf("pattern output pin %d: %v", i, err)
			}
			pin = copied
		}
		out = append(out, pin)
	}

	lift := liftBy(t, renderLift)
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
	view, err := render.View{
		Distance:     2.7,
		ElevationDeg: 30,
		AzimuthDeg:   -58,
		FOV:          32,
	}.FitTo(meshes...)
	if err != nil {
		t.Fatalf("frame the drive: %v", err)
	}
	return view.Camera()
}
