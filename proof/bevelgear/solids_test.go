// This file holds the bevel pair's straight-bevel solid steps: the gear-body
// revolve, the apex-to-tooth loft, the two conical end cuts, the circular
// pattern, the Combine-Join, the bore cut and the meshing rotation.
//
// # No boolean is performed anywhere in this gear's proof
//
// At the decad revision this repo pins, a boolean accepts only prism, cup and
// faceted payloads and refuses an operand built by Loft. Every solid in this
// gear is conical, and a cone is a Loft here because Extrude refuses a nonzero
// taper. So the union that joins the frustum, the intersection and cut that
// trim the tooth, the bore's through-cut and the Combine-Join are all out of
// reach.
//
// The substitution is the same at every site: build the operands, lay them
// apart along the shaft axis, and assert from their own measured geometry what
// the operation would have produced. Laying them apart leaves every volume,
// radius and cone angle unchanged, which is what makes the readings still mean
// something. What each site gives up is written at that site.
//
// # Two substitutions of shape, not of operation
//
// Every section here is drawn as a closed polyline, arcs and rings included. A
// lofted body whose sections carry a circle or an arc publishes a volume whose
// proven bound runs past decad's relative tolerance — about 1.4 times it for a
// cone band, and near two percent for a tooth — and the solid gate reports that
// as a measurement beyond tolerance. Lofted between polygons the same body
// publishes an EXACT volume. What it costs is the chord sagitta, and it costs
// it in a form that can be written down rather than tolerated: a band between
// inscribed regular polygons is exactly ringFactor times the cone it stands
// for, at every station, so the frustum's volume identity carries that factor
// instead of an error bar. On the tooth, where the boundary is not a full turn,
// every reading is chorded against chorded and the sagitta cancels.
//
// The tooth's cross-section plane is the second. In Fusion the tooth profile
// sits on the BACK CONE plane through the tooth-centre point, which is tilted
// off the shaft axis by the pitch cone angle; here it sits on an
// axis-perpendicular plane, at the station where the gear's own root cone has
// reached the virtual tooth's root radius. That keeps the two facts the later
// steps read — the tooth's root surface IS the root cone, and its tip surface
// stands proud of it — and gives up the tilt.
//
// # The frame
//
// Each gear is built in its own frame: the apex at the origin and the shaft
// axis along +Z, so a §2 point becomes its station along the axis and its
// radius from it. The two gears are proved one case at a time rather than as a
// meshing pair; what makes them a pair is the shared §2 lattice, which the
// sketch steps prove.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// solidCases runs at Module 4 to 8 and never at Module 1.
//
// decad's mesh bound has an absolute floor, so a figure small enough brings
// every measurement inside it and the gate reports Suspect on geometry that is
// in fact correct. Module is a pure scale on this figure, so a case at Module 4
// through 8 proves the same shape as one at Module 1 and clears the floor.
//
// Each gear of a pair is its own case, because the two are not mirror images
// once the tooth counts differ and either can be the binding side.
var solidCases = []proofkit3d.Case{
	{Name: "M4_31x31_90deg_pinion", Params: pinionDialog(dialog(4, 31, 31, 90))},
	{Name: "M4_31x31_90deg_driving", Params: drivingDialog(dialog(4, 31, 31, 90))},
	{Name: "M6_31x17_90deg_pinion", Params: pinionDialog(dialog(6, 31, 17, 90))},
	{Name: "M6_31x17_90deg_driving", Params: drivingDialog(dialog(6, 31, 17, 90))},
	{Name: "M8_19x13_60deg_pinion", Params: pinionDialog(dialog(8, 19, 13, 60))},
	{Name: "M4_31x31_120deg_driving", Params: drivingDialog(dialog(4, 31, 31, 120))},
	{Name: "M4_31x31_90deg_toe_ext_100_pinion", Params: pinionDialog(
		override(dialog(4, 31, 31, 90), map[string]float64{"toeExtension": 100}))},
	{Name: "M4_31x31_90deg_no_bore_driving", Params: drivingDialog(
		override(dialog(4, 31, 31, 90), map[string]float64{"boreEnable": 0}))},
}

func mm(v float64) units.Value { return units.Millimeters(v) }

// ---------------------------------------------------------------- the frame

// solid is one gear's figure in its own shaft frame: the apex at the origin,
// the shaft axis along +Z, and every §2 point reduced to the station and radius
// that the revolve turns into a solid.
type solid struct {
	figure
	gear   string
	params map[string]float64
	doc    *decad.Document
	world  *sketch.World
	gap    float64 // how far apart operands are laid along the axis
}

func newSolid(t *testing.T, doc *decad.Document, p map[string]float64) *solid {
	t.Helper()
	f := newFigure(p)
	s := &solid{figure: f, gear: gearOf(p), params: p, doc: doc, world: sketch.NewWorld()}
	span := 0.0
	for _, v := range f.hexagon(s.gear) {
		span = math.Max(span, f.station(s.gear, v))
	}
	s.gap = 4 * (span + f.toothStation(s.gear))
	return s
}

// zr is a §2 point's station along the shaft axis and its radius from it.
func (s *solid) zr(p vec) (float64, float64) {
	return s.station(s.gear, p), s.radius(s.gear, p)
}

// toothDims is the virtual spur tooth this gear's §3 step draws: the four
// circle radii the borrowed generator derives from Module and the virtual tooth
// number, at the proxy's own 20-degree pressure angle.
func (f figure) toothDims(gear string) involute.Dimensions {
	return involute.Derive(f.module, float64(f.virtualTeeth(gear)), rad(20))
}

// toothStation and toothScale place the tooth's cross-section.
//
// In Fusion the section sits on the BACK CONE plane through the tooth-centre
// point K prime (L prime on the driving side), which is tilted off the shaft
// axis by this gear's pitch cone angle. The proof puts it on an
// axis-perpendicular plane instead, and the two numbers here are what that
// substitution is worth keeping: the tooth-centre point sits ON the shaft axis,
// so a back-cone radius maps to a distance from the axis by cos(gamma), and the
// profile's root corner sits back from the centre along the tilted plane by its
// own root radius times sin(gamma).
//
// Placing the section at the root corner's station and scaling it by cos(gamma)
// therefore lands the tooth's root edge where the real one runs, just inside
// this gear's root cone, and leaves its tip standing proud. What is given up is
// the tilt: in the real tooth the tip corner sits at a slightly different
// station from the root corner, and here they share one.
func (f figure) toothStation(gear string) float64 {
	centre := f.K
	if gear == "Driving" {
		centre = f.L
	}
	s := f.sideOf(gear)
	return f.station(gear, centre) - f.toothDims(gear).Root*math.Sin(s.gamma)
}

func (f figure) toothScale(gear string) float64 {
	return math.Cos(f.sideOf(gear).gamma)
}

// toothInnerRadius is how far in the drawn tooth actually reaches. For an
// ordinary tooth that is its root radius, where the two flank-to-root lines
// seat it. For an EMBEDDED tooth — one whose flanks start inside the root
// circle, which happens at HIGH virtual tooth counts and is the common case
// here — the flanks meet at the base radius instead and the tooth reaches
// below its own root circle, with no connecting lines at all. That is the same
// fact the profile search's line count keys on.
func (f figure) toothInnerRadius(gear string) float64 {
	d := f.toothDims(gear)
	if d.Embedded() {
		return d.Base
	}
	return d.Root
}

// ---------------------------------------------------------------- sections

func (s *solid) planeAt(t *testing.T, z float64) *sketch.Plane {
	t.Helper()
	pl, err := s.world.CreateOffsetPlane(s.world.XY(), z)
	if err != nil {
		t.Fatalf("construction plane at station %.4f: %v", z, err)
	}
	return pl
}

func (s *solid) sketchAt(t *testing.T, z float64) *sketch.Sketch {
	t.Helper()
	sk, err := s.world.CreateSketch(s.planeAt(t, z))
	if err != nil {
		t.Fatalf("sketch at station %.4f: %v", z, err)
	}
	return sk
}

func onlyRegion(t *testing.T, sk *sketch.Sketch, label string) *sketch.Profile {
	t.Helper()
	if _, err := sk.Solve(context.Background()); err != nil {
		t.Fatalf("%s section solve: %v", label, err)
	}
	regions := sk.Profiles()
	if len(regions) != 1 {
		t.Fatalf("%s section holds %d regions, want exactly 1", label, len(regions))
	}
	if !regions[0].Valid {
		t.Fatalf("%s section region is not extrudable", label)
	}
	return regions[0]
}

// ringChords is how finely a cone band's circular cross-section is chorded,
// and ringFactor is exactly what that costs.
//
// A band lofted between two real circles publishes a volume whose proven bound
// runs about 1.4 times decad's relative tolerance, so the gate reports it as a
// measurement beyond tolerance and the step cannot pass. Lofted between two
// inscribed regular polygons it publishes an EXACT volume instead, because
// every cross-section of that loft is the inscribed polygon of the interpolated
// radius. So the band is exactly ringFactor times the cone it stands for, at
// every station, and the frustum's volume identity below carries that factor
// rather than a tolerance.
const ringChords = 64

var ringFactor = float64(ringChords) * math.Sin(2*math.Pi/ringChords) / (2 * math.Pi)

// ringSection is one cross-section of a cone band: the regular polygon
// inscribed in that station's ring.
func (s *solid) ringSection(t *testing.T, z, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	sk := s.sketchAt(t, z)
	pts := make([]*sketch.Point, ringChords)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / ringChords
		pts[i] = sk.CreatePoint(r*math.Cos(a), r*math.Sin(a))
	}
	for i := range pts {
		sk.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	return sk, onlyRegion(t, sk, "ring")
}

// coneBand is the solid a straight profile edge sweeps about the shaft axis,
// closed to the axis at both ends: a truncated cone between two stations.
//
// The three of these the revolve step builds are what the frustum's volume,
// stations and cone angles are read from. Each is laid at its own offset along
// the axis so no two share any space, which is what lets them live in one
// document without a boolean between them.
func (s *solid) coneBand(t *testing.T, z0, r0, z1, r1, offset float64, label string) *decad.Body {
	t.Helper()
	if r0 <= 0 || r1 <= 0 {
		t.Fatalf("%s band has a zero ring radius (%.6f, %.6f)", label, r0, r1)
	}
	sk0, p0 := s.ringSection(t, z0+offset, r0)
	sk1, p1 := s.ringSection(t, z1+offset, r1)
	body, err := s.doc.Loft(sk0, p0, sk1, p1)
	if err != nil {
		t.Fatalf("%s band loft: %v", label, err)
	}
	return body
}

// toothSection draws one cross-section of the tooth, scaled about the shaft
// axis, as a closed polyline. Every boundary is chorded, including the tip and
// root arcs, for the reason the file header gives.
func (s *solid) toothSection(t *testing.T, z, scale float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := s.toothDims(s.gear)
	virtual := float64(s.virtualTeeth(s.gear))
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, virtual, involuteSteps, math.Pi)

	sk := s.sketchAt(t, z)
	at := func(p involute.Pt) *sketch.Point {
		return sk.CreatePoint(p.X*scale, p.Y*scale)
	}
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = at(left[i])
		rp[i] = at(right[i])
	}
	for i := 0; i+1 < len(lp); i++ {
		sk.CreateLine(lp[i], lp[i+1])
		sk.CreateLine(rp[i], rp[i+1])
	}
	chordArc(sk, d.Tip*scale, rp[len(rp)-1], lp[len(lp)-1], arcChords)
	if d.Embedded() {
		chordArc(sk, d.Base*scale, rp[0], lp[0], arcChords)
	} else {
		lf := radialFoot(sk, d.Root*scale, left[0])
		rf := radialFoot(sk, d.Root*scale, right[0])
		sk.CreateLine(lf, lp[0])
		sk.CreateLine(rf, rp[0])
		chordArc(sk, d.Root*scale, rf, lf, arcChords)
	}
	return sk, onlyRegion(t, sk, "tooth")
}

// arcChords is how finely the tip and root boundaries are chorded.
const arcChords = 8

func chordArc(sk *sketch.Sketch, r float64, from, to *sketch.Point, n int) {
	a0 := math.Atan2(from.Y(), from.X())
	a1 := math.Atan2(to.Y(), to.X())
	for a1 < a0 {
		a1 += 2 * math.Pi
	}
	prev := from
	for i := 1; i < n; i++ {
		a := a0 + (a1-a0)*float64(i)/float64(n)
		next := sk.CreatePoint(r*math.Cos(a), r*math.Sin(a))
		sk.CreateLine(prev, next)
		prev = next
	}
	sk.CreateLine(prev, to)
}

func radialFoot(sk *sketch.Sketch, r float64, p involute.Pt) *sketch.Point {
	n := math.Hypot(p.X, p.Y)
	return sk.CreatePoint(r*p.X/n, r*p.Y/n)
}

// apexScrapScale is the section the degenerate apex point is substituted by.
// decad's Loft has no point section, so the apex end is a shrunken copy of the
// heel section instead, at the same fraction of the station. The taper is
// unchanged; what is given up is the single degenerate vertex at the apex.
const apexScrapScale = 0.05

// toothBody is the uncut apex-to-heel tooth: the loft from the shrunken apex
// section to the full section at the tooth station, offset along the axis by
// the amount the caller is laying operands apart by.
func (s *solid) toothBody(t *testing.T, offset, stationScale float64) *decad.Body {
	t.Helper()
	z1 := s.toothStation(s.gear) * stationScale
	k := s.toothScale(s.gear)
	sk0, p0 := s.toothSection(t, apexScrapScale*z1+offset, apexScrapScale*k)
	sk1, p1 := s.toothSection(t, z1+offset, k)
	body, err := s.doc.Loft(sk0, p0, sk1, p1)
	if err != nil {
		t.Fatalf("apex-to-tooth loft: %v", err)
	}
	return body
}

// ---------------------------------------------------------------- readings

func volumeOf(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return measured.Value.Base()
}

func boundsOf(t *testing.T, body *decad.Body, label string) decad.Box {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", label, err)
	}
	return box
}

// bandReading is a cone band read back off the body that was built, rather than
// off the numbers it was built from: the two cap radii come from the planar cap
// faces' own areas and the stations from the vertices those faces carry.
type bandReading struct {
	z0, r0, z1, r1 float64
}

// halfAngle is the angle between the band's element and the shaft axis, which
// is the cone's half-angle. It is signed, so a band that narrows as it climbs
// reads negative.
func (b bandReading) halfAngle() float64 {
	return math.Atan2(b.r1-b.r0, b.z1-b.z0)
}

// apexStation is where the band's element, extended, meets the axis.
func (b bandReading) apexStation() float64 {
	return b.z0 - b.r0*(b.z1-b.z0)/(b.r1-b.r0)
}

// radiusAt evaluates the band's element at a station.
func (b bandReading) radiusAt(z float64) float64 {
	return b.r0 + (b.r1-b.r0)*(z-b.z0)/(b.z1-b.z0)
}

func readBand(t *testing.T, body *decad.Body, offset float64, label string) bandReading {
	t.Helper()
	stations, radii := stationRadii(t, body, offset, label)
	return bandReading{stations[0], radii[0].hi, stations[1], radii[1].hi}
}

type radialSpan struct{ lo, hi float64 }

// stationRadii groups a lofted body's vertices by the station they sit at and
// returns the radial span at each. A loft between two sections has vertices at
// exactly two stations, so this is how the body's own geometry is read back
// without trusting the numbers it was built from.
func stationRadii(t *testing.T, body *decad.Body, offset float64, label string) ([2]float64, [2]radialSpan) {
	t.Helper()
	byZ := map[float64]*radialSpan{}
	order := []float64{}
	for _, v := range body.Vertices() {
		p := v.Position().Value
		z := math.Round((p.Z-offset)*1e6) / 1e6
		r := math.Hypot(p.X, p.Y)
		span, ok := byZ[z]
		if !ok {
			span = &radialSpan{lo: r, hi: r}
			byZ[z] = span
			order = append(order, z)
		}
		span.lo = math.Min(span.lo, r)
		span.hi = math.Max(span.hi, r)
	}
	if len(order) != 2 {
		t.Fatalf("%s has vertices at %d stations, want the loft's 2", label, len(order))
	}
	z0, z1 := order[0], order[1]
	if z0 > z1 {
		z0, z1 = z1, z0
	}
	return [2]float64{z0, z1}, [2]radialSpan{*byZ[z0], *byZ[z1]}
}

// coneLine is a cone through the apex, read off a lofted tooth: the station and
// radius of one of its surfaces at the two ends the loft was built between.
type coneLine struct {
	z0, r0, z1, r1 float64
}

func (c coneLine) radiusAt(z float64) float64 {
	return c.r0 + (c.r1-c.r0)*(z-c.z0)/(c.z1-c.z0)
}

func (c coneLine) halfAngle() float64 {
	return math.Atan2(c.r1-c.r0, c.z1-c.z0)
}

// readTooth returns the tooth's tip and root surfaces, measured from the
// vertices of the body that was built. The tooth is a loft between two
// sections, so its vertices sit at exactly two stations and the extreme radius
// at each is what the two surfaces pass through.
func readTooth(t *testing.T, body *decad.Body, offset float64) (tip, root coneLine) {
	t.Helper()
	stations, radii := stationRadii(t, body, offset, "tooth body")
	return coneLine{stations[0], radii[0].hi, stations[1], radii[1].hi},
		coneLine{stations[0], radii[0].lo, stations[1], radii[1].lo}
}

// crossStation solves for the station where a band's element meets a cone
// through the apex.
func crossStation(band bandReading, line coneLine) float64 {
	bandSlope := (band.r1 - band.r0) / (band.z1 - band.z0)
	lineSlope := (line.r1 - line.r0) / (line.z1 - line.z0)
	lineAt0 := line.r0 - lineSlope*line.z0
	bandAt0 := band.r0 - bandSlope*band.z0
	return (lineAt0 - bandAt0) / (bandSlope - lineSlope)
}

// azimuthOf is the angle about the shaft axis of the body's own centroid,
// which is the marker the pattern and the meshing rotation are read by. A
// tooth's centroid sits off the axis and turns with it exactly; a vertex would
// do as well until two vertices tie at the same radius, which the tooth's two
// tip corners do.
func azimuthOf(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	c, err := body.Centroid()
	if err != nil {
		t.Fatalf("%s centroid: %v", label, err)
	}
	return math.Atan2(c.Value.Y, c.Value.X)
}

// maxRadiusOf is how far the body reaches from the shaft axis. Unlike a
// bounding box it does not change when the body turns about that axis, which is
// what a pattern copy has to be compared on.
func maxRadiusOf(body *decad.Body) float64 {
	best := 0.0
	for _, v := range body.Vertices() {
		p := v.Position().Value
		best = math.Max(best, math.Hypot(p.X, p.Y))
	}
	return best
}

func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// ---------------------------------------------------------------- the revolve

// stepRevolveFrustum builds the Gear Body: the hexagon of §2 revolved a full
// turn about the hexagon's first edge.
//
// decad publishes a revolved body's volume with a proven bound equal to the
// volume itself, so a revolved body is Suspect at any tolerance and cannot pass
// the gate. The substitute is the three bands the frustum's three off-axis
// profile edges sweep — the heel cone out to the heel end, the root cone out to
// the dedendum corner, and the toe-dish plug that hollows the front face — laid
// apart and never joined. The other three edges sweep nothing: two lie in a
// plane perpendicular to the axis and one lies on the axis itself.
//
// The cost is the union: the proof does not show the three bands closing into
// one watertight solid, only that each is separately watertight and that
// together they have the right volume, stations and angles.
func stepRevolveFrustum(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	return s.frustumBands(t)
}

// bandEdges are the hexagon edges that sweep a band, by index in draw order:
// heel end to dedendum corner, dedendum corner to toe root, toe root to inner
// toe corner.
var bandEdges = [3]int{2, 3, 4}

func (s *solid) frustumBands(t *testing.T) []*decad.Body {
	t.Helper()
	verts := s.hexagon(s.gear)
	bodies := make([]*decad.Body, 0, 3)
	labels := [3]string{"heel", "root", "toe plug"}
	for k, i := range bandEdges {
		z0, r0 := s.zr(verts[i])
		z1, r1 := s.zr(verts[(i+1)%len(verts)])
		bodies = append(bodies, s.coneBand(t, z0, r0, z1, r1,
			float64(k+1)*s.gap, labels[k]))
	}
	return bodies
}

// assertRevolveFrustum reads the three bands back and checks the frustum they
// stand for: its volume as their signed sum against Pappus on the §2 hexagon,
// each band's own stations and ring radii, and the three cone half-angles.
func assertRevolveFrustum(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 3 {
		t.Fatalf("the frustum profile swept %d bands, want the 3 its off-axis edges make",
			len(bodies))
	}
	verts := s.hexagon(s.gear)

	var signedSum float64
	readings := make([]bandReading, 3)
	for k, i := range bandEdges {
		z0, r0 := s.zr(verts[i])
		z1, r1 := s.zr(verts[(i+1)%len(verts)])
		offset := float64(k+1) * s.gap
		band := readBand(t, bodies[k], offset, "band")
		readings[k] = band

		near(t, band.z0, math.Min(z0, z1), 1e-6, "band %d lower station", k)
		near(t, band.z1, math.Max(z0, z1), 1e-6, "band %d upper station", k)
		lo, hi := r0, r1
		if z0 > z1 {
			lo, hi = r1, r0
		}
		near(t, band.r0, lo, 1e-4, "band %d lower ring radius", k)
		near(t, band.r1, hi, 1e-4, "band %d upper ring radius", k)

		// The signed sum: each directed edge contributes minus the cone
		// frustum its two rings bound, and the six of them add to the solid of
		// revolution. Three contribute nothing and are not built.
		h := z1 - z0
		signedSum -= ringFactor * math.Pi / 3 * (r0*r0 + r0*r1 + r1*r1) * h
		fromRings := ringFactor * math.Pi / 3 *
			(band.r0*band.r0 + band.r0*band.r1 + band.r1*band.r1) * (band.z1 - band.z0)
		near(t, volumeOf(t, bodies[k], "band"), fromRings, 1e-5*fromRings,
			"band %d volume against its own measured rings", k)
	}

	want := ringFactor * s.pappusVolume(s.gear)
	near(t, signedSum, want, 1e-6*want,
		"the three bands' signed sum is the revolved hexagon's volume, chorded")

	// The heel band and the toe plug are the back-cone family and come out
	// parallel; the root band stands off them by the dedendum angle's
	// complement, which is what makes the toe a dish rather than a step.
	heel, root, toe := readings[0], readings[1], readings[2]
	near(t, math.Abs(wrapPi(heel.halfAngle()-toe.halfAngle())), 0, 1e-4,
		"the heel band and the toe plug are parallel")
	dedendumAngle := math.Atan(1.25 * s.module / s.R)
	near(t, math.Abs(wrapPi(heel.halfAngle()-root.halfAngle())), math.Pi/2-dedendumAngle,
		1e-4, "the root band stands off the back cone by the dedendum angle's complement")
}

// ---------------------------------------------------------------- apex loft

// stepApexLoft builds the uncut Tooth Body: the §2 Apex sketch point lofted to
// this gear's §3 tooth profile.
//
// Two substitutions, both of shape rather than of operation. decad's Loft has
// no point section, so the degenerate apex end is a shrunken copy of the tooth
// section at the same fraction of the station — which is the section the real
// loft passes through there anyway. And the section sits on an
// axis-perpendicular plane rather than the tilted back-cone plane.
func stepApexLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	return []*decad.Body{s.toothBody(t, 0, 1)}
}

// assertApexLoft pins what the loft produces: one body, tapering from the apex
// to the tooth section, whose volume is the section's own area times the
// station it sits at, over three — the volume a linear taper to a point has.
func assertApexLoft(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 1 {
		t.Fatalf("the apex loft produced %d bodies, want the one Tooth Body", len(bodies))
	}
	body := bodies[0]
	z1 := s.toothStation(s.gear)
	_, region := s.toothSection(t, z1, s.toothScale(s.gear))

	want := region.Area * z1 * (1 - apexScrapScale*apexScrapScale*apexScrapScale) / 3
	near(t, volumeOf(t, body, "Tooth Body"), want, 1e-6*want,
		"the tooth tapers linearly from the apex to its section")

	box := boundsOf(t, body, "Tooth Body")
	near(t, box.Min.Z, apexScrapScale*z1, 1e-6, "the tooth's apex end")
	near(t, box.Max.Z, z1, 1e-6, "the tooth's section end")

	// The tooth seats on the root cone and stands proud of it, which is what
	// the Combine-Join later needs and what the conical cuts trim.
	tip, inner := readTooth(t, body, 0)
	d := s.toothDims(s.gear)
	gammaRoot := s.sideOf(s.gear).gammaRoot
	k := s.toothScale(s.gear)
	near(t, tip.r1, d.Tip*k, 1e-4, "the tooth reaches the virtual tip radius")
	near(t, inner.r1, s.toothInnerRadius(s.gear)*k, 1e-4,
		"the tooth reaches in to its inner radius (embedded=%v)", d.Embedded())

	// The tooth is SEATED: its root surface lies at or inside this gear's root
	// cone at every station, which is what makes the Combine-Join leave one
	// lump rather than two. An embedded tooth reaches further in still, by
	// exactly the ratio of its base radius to its root radius.
	rootHalf := math.Atan2(d.Root*k, z1)
	if math.Tan(rootHalf) > math.Tan(gammaRoot) {
		t.Errorf("the tooth's root surface stands at %.6f rad, outside this gear's root "+
			"cone at %.6f rad: the tooth would float off the gear body",
			rootHalf, gammaRoot)
	}
	near(t, math.Tan(inner.halfAngle())/math.Tan(rootHalf),
		s.toothInnerRadius(s.gear)/d.Root, 1e-6,
		"how far the tooth reaches past its own root circle")
	if tip.halfAngle() <= inner.halfAngle() {
		t.Errorf("the tooth's tip surface (%.6f rad) does not stand proud of its inner "+
			"surface (%.6f rad)", tip.halfAngle(), inner.halfAngle())
	}
}

// ---------------------------------------------------------------- conical cuts

// stepConicalCut builds the two cutting cones and the tooth they trim, and
// performs neither cut.
//
// Both operands are Lofts — the tooth and each cone alike — so the split is out
// of reach. The three are laid apart along the axis and each cut is solved from
// the operands' own measured geometry instead.
//
// The cost is the split: the proof does not show the evaluator dividing the
// tooth, selecting the keeper, or leaving a watertight body. What it does show
// is that each cut lands where the flush band requires, and that the two ends
// land on DIFFERENT surfaces of the tooth — the toe on its tip and the heel on
// its root — which is the observable signature of a conical cut face rather
// than a planar one.
func stepConicalCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	verts := s.hexagon(s.gear)
	tz0, tr0 := s.zr(verts[4]) // toe root
	tz1, tr1 := s.zr(verts[5]) // inner toe corner
	hz0, hr0 := s.zr(verts[3]) // dedendum corner
	hz1, hr1 := s.zr(verts[2]) // heel end
	return []*decad.Body{
		s.toothBody(t, 0, 1),
		s.coneBand(t, tz0, tr0, tz1, tr1, s.gap, "toe cone"),
		s.coneBand(t, hz1, hr1, hz0, hr0, 2*s.gap, "heel cone"),
	}
}

func assertConicalCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 3 {
		t.Fatalf("the conical cut laid out %d bodies, want the tooth and its two cones",
			len(bodies))
	}
	tip, inner := readTooth(t, bodies[0], 0)
	toe := readBand(t, bodies[1], s.gap, "toe cone")
	heel := readBand(t, bodies[2], 2*s.gap, "heel cone")

	// Read off each cone: it passes through both ends of the §2 edge it was
	// swept from, so it is that gear's real toe or heel cone and not a
	// look-alike, and the two are parallel — both on the back-cone family.
	verts := s.hexagon(s.gear)
	for _, want := range []struct {
		name string
		band bandReading
		a, b vec
	}{
		{"toe", toe, verts[4], verts[5]},
		{"heel", heel, verts[3], verts[2]},
	} {
		az, ar := s.zr(want.a)
		bz, br := s.zr(want.b)
		near(t, want.band.radiusAt(az), ar, 1e-4,
			"the %s cone passes through its dedendum-side endpoint", want.name)
		near(t, want.band.radiusAt(bz), br, 1e-4,
			"the %s cone passes through its far endpoint", want.name)
	}
	near(t, math.Abs(wrapPi(toe.halfAngle()-heel.halfAngle())), 0, 1e-4,
		"the toe cone and the heel cone are parallel, both on the back-cone family")

	// The flush band: the two cones meet the gear's own root cone at the toe
	// root corner and the dedendum corner, a Root Length apart along it.
	mz, _ := s.zr(verts[4])
	cz, _ := s.zr(verts[3])
	near(t, cz-mz, s.rootLen*math.Cos(s.sideOf(s.gear).gammaRoot), 1e-4,
		"the flush band along the root cone is the resolved Root Length")

	// Where each cut lands ON THE TOOTH. Both cones lean back toward the apex,
	// so each meets the tooth's tip surface nearer the apex than its inner one,
	// and the toe cut's pair lies wholly below the heel cut's. That ordering is
	// what makes the trimmed tooth a band with length rather than a point, and
	// the two ends landing on different surfaces is the observable signature of
	// a conical cut face rather than a planar one.
	toeTip, toeInner := crossStation(toe, tip), crossStation(toe, inner)
	heelTip, heelInner := crossStation(heel, tip), crossStation(heel, inner)
	for _, c := range []struct {
		name     string
		at, next float64
	}{
		{"the toe cut reaches the tooth's tip nearer the apex than its inner surface",
			toeTip, toeInner},
		{"the heel cut reaches the tooth's tip nearer the apex than its inner surface",
			heelTip, heelInner},
		{"the toe cut lands below the heel cut on the tip surface", toeTip, heelTip},
		{"the toe cut lands below the heel cut on the inner surface", toeInner, heelInner},
	} {
		if !(c.at < c.next) {
			t.Errorf("%s: %.6f is not below %.6f", c.name, c.at, c.next)
		}
	}
}

// ---------------------------------------------------------------- pattern

// The circular pattern is the one bevel step that carries a reading from its
// build into its assertion, and it is therefore the one step registered through
// the SERIAL runner rather than the parallel one. The pattern increment retires
// the seed tooth — Placed consumes its operand, as Fusion's pattern retires the
// body it copies — so the seed cannot be measured after the step runs, and its
// azimuth, radius, height and volume have to be read during the build. Two
// cases running at once would overwrite each other's readings here and the
// proof would report a wrong verdict rather than failing loudly.
var (
	patternSeedAzimuth float64
	patternSeedVolume  float64
	patternSeedRadius  float64
	patternSeedTop     float64
)

// stepCircularPattern builds the seed tooth and one patterned copy of it.
//
// The pattern's inputs are pinned: quantity is this gear's Teeth Number,
// totalAngle is a full 360 degrees and it is not symmetric, so copy k sits at
// 2*pi*k/N about the shaft axis. The angular spacing is constant for the whole
// face width even though the pitch diameter shrinks toward the apex, because
// the taper is already in the loft.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	seed := s.toothBody(t, 0, 1)
	patternSeedAzimuth = azimuthOf(t, seed, "seed tooth")
	patternSeedVolume = volumeOf(t, seed, "seed tooth")
	patternSeedRadius = maxRadiusOf(seed)
	patternSeedTop = boundsOf(t, seed, "seed tooth").Max.Z

	teeth := s.sideOf(s.gear).teeth
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1),
		units.Degrees(360/teeth))
	if err != nil {
		t.Fatalf("pattern increment: %v", err)
	}
	copyOne, err := seed.Placed(turn)
	if err != nil {
		t.Fatalf("pattern copy: %v", err)
	}
	return []*decad.Body{copyOne}
}

func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 1 {
		t.Fatalf("the pattern increment left %d bodies, want the one copy", len(bodies))
	}
	teeth := s.sideOf(s.gear).teeth
	step := 2 * math.Pi / teeth

	near(t, wrapPi(azimuthOf(t, bodies[0], "copy")-patternSeedAzimuth), step, 1e-6,
		"copy 1 sits one tooth pitch round from the seed")
	near(t, volumeOf(t, bodies[0], "copy"), patternSeedVolume, 1e-6*patternSeedVolume,
		"a pattern copy is the seed moved, not reshaped")
	near(t, boundsOf(t, bodies[0], "copy").Max.Z, patternSeedTop, 1e-6,
		"the copy keeps the seed's station")
	near(t, maxRadiusOf(bodies[0]), patternSeedRadius, 1e-6*patternSeedRadius,
		"the copy keeps the seed's reach from the axis")

	// A full turn of these increments is what quantity and totalAngle ask for.
	near(t, step*teeth, 2*math.Pi, 1e-12,
		"%.0f copies at 360/N degrees close the full circle", teeth)
}

// ---------------------------------------------------------------- combine

// stepCombineJoin lays the Gear Body's seating surface and one tooth apart and
// performs no join.
//
// Both operands are Lofts. The substitute asserts the join's two consequences
// from the operands' own measured geometry: a join leaves ONE lump when the
// tooth's root is at or below the body's root cone — seated, not floating — and
// the joined body reaches further out than the frustum when the tooth's tip
// stands proud of it.
//
// The cost is the stitch: the proof cannot show the evaluator making one
// boundary out of two.
//
// The tooth here is sunk a twentieth of the tooth height below the gear body's
// root cone, which is what makes "seated" measurable as a strict inequality.
// The generated module seats the tooth exactly ON the cone and must not sink
// it: the sink belongs to the proof alone.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	verts := s.hexagon(s.gear)
	mz, mr := s.zr(verts[4])
	cz, cr := s.zr(verts[3])
	rootBand := s.coneBand(t, mz, mr, cz, cr, 0, "root cone")
	tooth := s.toothBody(t, s.gap, combineSinkStation(s))
	return []*decad.Body{rootBand, tooth}
}

// combineSinkStation moves the tooth's section out along the axis until the
// root cone there stands a twentieth of the tooth height above the tooth's own
// root, which sinks the tooth by that much.
func combineSinkStation(s *solid) float64 {
	d := s.toothDims(s.gear)
	sink := (d.Tip - d.Root) / 20
	return (d.Root + sink) / d.Root
}

func assertCombineJoin(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 2 {
		t.Fatalf("the join laid out %d bodies, want the gear body and the tooth",
			len(bodies))
	}
	band := readBand(t, bodies[0], 0, "root cone")
	tip, root := readTooth(t, bodies[1], s.gap)

	verts := s.hexagon(s.gear)
	toeZ, _ := s.zr(verts[4])
	heelZ, _ := s.zr(verts[3])
	for _, probe := range []struct {
		name string
		z    float64
	}{
		{"toe", toeZ},
		{"middle", (toeZ + heelZ) / 2},
		{"heel", heelZ},
	} {
		cone := band.radiusAt(probe.z)
		if got := root.radiusAt(probe.z); got >= cone {
			t.Errorf("at the %s the tooth's root sits at %.6f and the gear body's root "+
				"cone at %.6f: the tooth is not seated, so the join would leave two lumps",
				probe.name, got, cone)
		}
		if got := tip.radiusAt(probe.z); got <= cone {
			t.Errorf("at the %s the tooth's tip reaches %.6f against the gear body's "+
				"%.6f: the join would add no material", probe.name, got, cone)
		}
	}
}

// ---------------------------------------------------------------- bore

// stepBoreCut builds the bore tool and performs no cut.
//
// The tool is a real extrude: a symmetric extent produces a prism, which decad
// builds exactly. The target is the frustum, whose bands are Lofts, so the cut
// itself is out of reach. Tool and band are laid apart and the cut is asserted
// from the tool's own measured geometry.
//
// The cost is the pierced body: one lump with a hole and no enclosed void is
// not shown.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	if p["boreEnable"] == 0 {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so no bore is cut on either gear")
	}
	verts := s.hexagon(s.gear)
	startZ, _ := s.zr(verts[0]) // the shaft-axis edge's start: the front-face foot
	dia := s.sideOf(s.gear).boreDia

	sk, region := s.ringSection(t, startZ, dia/2)
	tool, err := s.doc.Extrude(sk, region,
		decad.Symmetric{D: mm(2 * s.coneDistance)})
	if err != nil {
		t.Fatalf("bore tool extrude: %v", err)
	}

	mz, mr := s.zr(verts[4])
	cz, cr := s.zr(verts[3])
	return []*decad.Body{tool, s.coneBand(t, mz, mr, cz, cr, s.gap, "root cone")}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 2 {
		t.Fatalf("the bore step laid out %d bodies, want the tool and the gear body",
			len(bodies))
	}
	dia := s.sideOf(s.gear).boreDia
	verts := s.hexagon(s.gear)
	startZ, _ := s.zr(verts[0])
	half := 2 * s.coneDistance

	box := boundsOf(t, bodies[0], "bore tool")
	near(t, box.Min.Z, startZ-half, 1e-6,
		"the bore tool starts 2 x Cone Distance below the shaft edge's start")
	near(t, box.Max.Z, startZ+half, 1e-6,
		"the bore tool ends 2 x Cone Distance above the shaft edge's start")
	near(t, box.Max.X, dia/2, 1e-3*dia, "the bore tool's radius is half the Bore Diameter")
	near(t, volumeOf(t, bodies[0], "bore tool"), ringFactor*math.Pi*dia*dia/4*2*half,
		1e-5*math.Pi*dia*dia/4*2*half, "the bore tool is a prism of that section")

	// A THROUGH cut: the tool clears the whole figure at both ends.
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, v := range verts {
		z, _ := s.zr(v)
		lo, hi = math.Min(lo, z), math.Max(hi, z)
	}
	if !(box.Min.Z < lo && box.Max.Z > hi) {
		t.Errorf("the bore tool spans [%.4f, %.4f] and the frustum [%.4f, %.4f]: the cut "+
			"would not pierce it", box.Min.Z, box.Max.Z, lo, hi)
	}

	// What the cut would remove: the frustum's own profile clipped to the bore
	// radius, swept about the axis.
	removed := s.pappusOf(clipToRadius(s.profilePoints(), dia/2))
	whole := s.pappusVolume(s.gear)
	if removed <= 0 || removed >= whole {
		t.Errorf("the bore would remove %.6f mm3 of a %.6f mm3 frustum, which is not a "+
			"hole through it", removed, whole)
	}
}

// profilePoints is the hexagon in (station, radius) coordinates.
func (s *solid) profilePoints() [][2]float64 {
	verts := s.hexagon(s.gear)
	out := make([][2]float64, len(verts))
	for i, v := range verts {
		z, r := s.zr(v)
		out[i] = [2]float64{z, r}
	}
	return out
}

// pappusOf is the volume a closed (station, radius) polygon sweeps about the
// axis.
func (s *solid) pappusOf(pts [][2]float64) float64 {
	if len(pts) < 3 {
		return 0
	}
	var area2, moment float64
	for i := range pts {
		j := (i + 1) % len(pts)
		cross := pts[i][0]*pts[j][1] - pts[j][0]*pts[i][1]
		area2 += cross
		moment += (pts[i][1] + pts[j][1]) * cross
	}
	area := area2 / 2
	if area == 0 {
		return 0
	}
	return math.Abs(2 * math.Pi * area * (moment / (6 * area)))
}

// clipToRadius keeps the part of a profile polygon inside a radius.
func clipToRadius(pts [][2]float64, r float64) [][2]float64 {
	out := make([][2]float64, 0, len(pts)+2)
	for i := range pts {
		cur, next := pts[i], pts[(i+1)%len(pts)]
		curIn, nextIn := cur[1] <= r, next[1] <= r
		if curIn {
			out = append(out, cur)
		}
		if curIn != nextIn {
			f := (r - cur[1]) / (next[1] - cur[1])
			out = append(out, [2]float64{cur[0] + f*(next[0]-cur[0]), r})
		}
	}
	return out
}

// ---------------------------------------------------------------- mesh phase

// stepMeshRotate applies the meshing rotation: half a tooth pitch about the
// gear's own shaft axis, on the driving gear only.
//
// Both gears are patterned from a starting tooth in the axial plane, so without
// the offset a driving tooth and a pinion tooth would both sit at the
// axial-plane crossing and collide. The pinion's extra phase is zero by
// default, and a zero angle is not a move: Fusion refuses the identity
// transform outright with "invalid transform", so the generator must return
// before building one rather than guard at each call site.
func stepMeshRotate(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s := newSolid(t, doc, p)
	body := s.toothBody(t, 0, 1)
	angle := s.meshPhase()
	if angle == 0 {
		return []*decad.Body{body}
	}
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1),
		units.Radians(angle))
	if err != nil {
		t.Fatalf("meshing rotation: %v", err)
	}
	moved, err := body.Placed(turn)
	if err != nil {
		t.Fatalf("meshing rotation move: %v", err)
	}
	return []*decad.Body{moved}
}

// meshPhase is this gear's extra rotation about its own shaft axis: half a
// tooth pitch for the driving gear, and the pinion's zero mesh phase otherwise.
func (s *solid) meshPhase() float64 {
	if s.gear == "Driving" {
		return math.Pi / s.driving.teeth
	}
	return pinionMeshPhaseTeeth * 2 * math.Pi / s.pinion.teeth
}

// pinionMeshPhaseTeeth is the pinion's extra mesh rotation in tooth fractions.
// It is zero for a straight bevel and stays zero for the spiral, because the
// spiral twist is centred on the mean cone distance and leaves the mid-face
// section unrotated, so that section already meshes like the straight tooth.
const pinionMeshPhaseTeeth = 0

func assertMeshRotate(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	s := newSolid(t, decad.New(), p)
	if len(bodies) != 1 {
		t.Fatalf("the meshing rotation left %d bodies, want the one gear body", len(bodies))
	}
	reference := newSolid(t, decad.New(), p)
	unmoved := reference.toothBody(t, 0, 1)
	near(t, wrapPi(azimuthOf(t, bodies[0], "rotated body")-
		azimuthOf(t, unmoved, "reference body")), s.meshPhase(), 1e-6,
		"%s mesh phase", s.gear)
	near(t, volumeOf(t, bodies[0], "rotated body"), volumeOf(t, unmoved, "reference body"),
		1e-6*volumeOf(t, unmoved, "reference body"),
		"the meshing rotation moves the body and does not reshape it")
	if s.gear == "Driving" {
		near(t, s.meshPhase(), math.Pi/s.driving.teeth, 1e-12,
			"the driving gear turns half a tooth pitch")
	} else if s.meshPhase() != 0 {
		t.Errorf("the pinion's mesh phase is %.9f, and a nonzero phase needs a move "+
			"Fusion would refuse at zero", s.meshPhase())
	}
}
