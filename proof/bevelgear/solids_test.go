// This file holds the bevel gear's straight-bevel solid steps, one per Fusion
// timeline entry: the gear-body revolve, the apex-to-tooth loft, the conical end
// cuts, the circular pattern, the Combine-Join, the bore cut and the driving
// gear's meshing rotation. The spiral branch's steps are in spiral_test.go.
//
// NO BOOLEAN IS PERFORMED ANYWHERE IN THIS GEAR'S PROOF, and that is forced
// rather than chosen. At the decad revision proof/go.mod pins, a boolean accepts
// only prism, cup and faceted payloads and refuses an operand built by Loft.
// Every solid in this gear is conical, and a cone is a Loft here because Extrude
// refuses a nonzero taper — WithTaper is ErrUnsupported before a step is even
// recorded. So the union that joins the frustum, the intersection and cut that
// trim the tooth, the bore's through-cut and the Combine-Join are all out of
// reach.
//
// THE SUBSTITUTION IS THE SAME AT EVERY SITE: build the operands, lay them apart
// along the shaft axis, and assert from their own measured geometry what the
// operation would have produced. Laying them apart leaves every volume, radius,
// station and cone half-angle unchanged, which is what makes the readings still
// mean something; what it drops is the evaluator's own work — the stitch, the
// split, the pierced body — and each step below says which.
//
// THE REVOLVE ITSELF IS THE OTHER SUBSTITUTION. decad has a Revolve, and it is
// not usable here: measured on this repository's pinned revision, a revolved
// trapezoid publishes volume 8210.03 mm3 with a proven bound of 16420.06 mm3 —
// a bound equal to twice the reading — so a revolved body is Suspect at any
// tolerance and cannot pass the harness gate. A Loft between two coaxial circles
// is the polygonal sweep of the same figure and publishes the same volume with a
// bound four orders of magnitude smaller.
//
// THE TABLES RUN AT MODULE 4 TO 8 AND NEVER AT MODULE 1, because decad's mesh
// bound has an absolute floor and a small enough figure brings every measurement
// inside it. Module is a pure scale on this figure, so a case at Module 4 proves
// the same shape.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func millimetres(v float64) units.Value { return units.Millimeters(v) }

// sweepSides is how many sides the polygonal sweep uses for one full turn.
//
// The spec calls the revolve's substitute a POLYGONAL sweep, and that is what
// keeps the readings usable: a loft between two coaxial CIRCLES is chorded by
// the evaluator at a tolerance it chooses, and measured here that left the
// frustum's volume bound just past the gate's relative tolerance on every case.
// A loft between two regular polygons is a polyhedron, and its volume is the
// same cone formula with pi replaced by the polygon's own (n/2)*sin(2*pi/n), so
// the closed form the assertion compares against is exact for the body that was
// actually built. ringArea below is that substitution, written once.
const sweepSides = 64

// ringArea is the area a ring of radius r encloses in the polygonal sweep: the
// regular sweepSides-gon's, not the circle's.
func ringArea(r float64) float64 {
	return float64(sweepSides) / 2 * math.Sin(2*math.Pi/float64(sweepSides)) * r * r
}

// ringSweep builds one band of the revolved frustum: the solid swept by one
// straight profile edge turning about the shaft axis, as a loft between the two
// rings its endpoints sweep.
//
// lift slides the whole band along the shaft axis. The bands are laid apart
// because no boolean can join them here and decad verifies every PAIR of live
// bodies in a document; overlapping operands report as an interference or an
// undecided pair, which the gate refuses. A translation along the axis changes
// no radius, no cone half-angle and no volume, so every reading below still
// means what it says — only the stations move, by a known amount the assertion
// subtracts.
func ringSweep(t *testing.T, doc *decad.Document, z0, r0, z1, r1, lift float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	build := func(z, r float64) (*sketch.Sketch, *sketch.Profile) {
		plane := w.XY()
		if z+lift != 0 {
			var err error
			if plane, err = w.CreateOffsetPlane(w.XY(), z+lift); err != nil {
				t.Fatalf("station plane at %.6f mm: %v", z+lift, err)
			}
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			t.Fatalf("station sketch at %.6f mm: %v", z+lift, err)
		}
		if _, err := s.CreatePolygon(0, 0, sweepSides, r); err != nil {
			t.Fatalf("ring polygon at station %.6f: %v", z+lift, err)
		}
		solveHere(t, s)
		regions := s.Profiles()
		if len(regions) != 1 {
			t.Fatalf("ring at station %.6f holds %d regions, want 1", z+lift, len(regions))
		}
		return s, regions[0]
	}
	s0, p0 := build(z0, r0)
	s1, p1 := build(z1, r1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("sweep the band from (%.4f, %.4f) to (%.4f, %.4f): %v", z0, r0, z1, r1, err)
	}
	return body
}

// bandReading is what a built band publishes about itself: the two stations its
// rings sit at and the radius of each, read off the body's own vertices rather
// than off the numbers it was built from.
type bandReading struct {
	lowStation, lowRadius   float64
	highStation, highRadius float64
	volume, volumeBound     float64
}

// readBand measures one band. lift is subtracted so the stations come back in
// the gear's own frame.
func readBand(t *testing.T, body *decad.Body, lift float64, label string) bandReading {
	t.Helper()
	type ring struct{ z, r float64 }
	rings := make([]ring, 0, 2)
	for _, v := range body.Vertices() {
		p := v.Position().Value
		z := p.Z - lift
		r := math.Hypot(p.X, p.Y)
		found := false
		for i := range rings {
			if math.Abs(rings[i].z-z) < 1e-6 {
				if math.Abs(rings[i].r-r) > 1e-6 {
					t.Errorf("%s: station %.6f carries radii %.6f and %.6f; a ring is one radius",
						label, z, rings[i].r, r)
				}
				found = true
			}
		}
		if !found {
			rings = append(rings, ring{z, r})
		}
	}
	if len(rings) != 2 {
		t.Fatalf("%s: the band reports %d distinct stations, want the two rings it was swept between",
			label, len(rings))
	}
	if rings[0].z > rings[1].z {
		rings[0], rings[1] = rings[1], rings[0]
	}
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return bandReading{
		lowStation: rings[0].z, lowRadius: rings[0].r,
		highStation: rings[1].z, highRadius: rings[1].r,
		volume: measured.Value.Base(), volumeBound: measured.Bound.Base(),
	}
}

// halfAngle is the cone half-angle the band's own two rings give: the angle
// between its swept element and the shaft axis.
func (b bandReading) halfAngle() float64 {
	return math.Atan2(math.Abs(b.highRadius-b.lowRadius), math.Abs(b.highStation-b.lowStation))
}

// element is the band's swept element as a line in the axial half-plane, for
// solving where two of them cross.
func (b bandReading) element() (slope, intercept float64) {
	slope = (b.highRadius - b.lowRadius) / (b.highStation - b.lowStation)
	return slope, b.lowRadius - slope*b.lowStation
}

// crossStation is where two elements meet, measured along the shaft axis.
func crossStation(a, b bandReading) (float64, bool) {
	sa, ia := a.element()
	sb, ib := b.element()
	if math.Abs(sa-sb) < 1e-12 {
		return 0, false
	}
	return (ib - ia) / (sa - sb), true
}

// bandSeparation is how far apart the laid-out bodies are set along the shaft
// axis. It is several times the whole figure's reach, so two bodies' bounding
// boxes never meet and decad's pair analysis resolves as disjoint rather than
// reporting an undecided pair.
func (d bevelDesign) bandSeparation(which float64) float64 {
	return 4 * (d.axialProfile(which).heelAxis.station + d.rootCone)
}

// ---------------------------------------------------------------- S11 revolve

// stepRevolveGearBody builds the Gear Body: the hexagon of §2 revolved a full
// turn about its own first edge.
//
// THE SUBSTITUTION. The revolve is replaced by the three bands the hexagon's
// edges sweep — the root cone out to the dedendum corner, the heel cone out to
// the heel end, and the toe-dish plug that hollows the front face — each built
// as its own loft and laid apart along the shaft axis. The hexagon's other three
// edges sweep nothing: two of them lie ON the axis, and the front face's annulus
// is the plug's own end ring.
//
// THE COST IS THE UNION: the proof does not show the three bands closing into
// one watertight solid, only that each is separately watertight and that
// together, taken as a SIGNED sum with the plug subtracted, they have the
// frustum's volume, its stations and its cone half-angles.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	return []*decad.Body{
		ringSweep(t, doc, h.toeCone.station, h.toeCone.radius, h.ded.station, h.ded.radius, 0),
		ringSweep(t, doc, h.ded.station, h.ded.radius, h.heelAxis.station, h.heelEnd.radius, sep),
		ringSweep(t, doc, h.toeCone.station, h.toeCone.radius, h.toeIn.station, h.toeIn.radius, 2*sep),
	}
}

func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	if len(bodies) != 3 {
		t.Fatalf("the revolve substitution produced %d bands, want the root band, the heel band and the toe plug",
			len(bodies))
	}
	root := readBand(t, bodies[0], 0, "root band")
	heel := readBand(t, bodies[1], sep, "heel band")
	plug := readBand(t, bodies[2], 2*sep, "toe plug")

	// Each band against its own stations and ring radii.
	ring := func(label string, gotStation, gotRadius float64, want axialPt) {
		if math.Abs(gotStation-want.station) > 1e-6 || math.Abs(gotRadius-want.radius) > 1e-6 {
			t.Errorf("%s ring is at station %.6f radius %.6f, want station %.6f radius %.6f",
				label, gotStation, gotRadius, want.station, want.radius)
		}
	}
	ring("the root band's toe", root.lowStation, root.lowRadius, h.toeCone)
	ring("the root band's heel", root.highStation, root.highRadius, h.ded)
	ring("the heel band's inner", heel.lowStation, heel.lowRadius, h.ded)
	ring("the heel band's outer", heel.highStation, heel.highRadius,
		axialPt{station: h.heelAxis.station, radius: h.heelEnd.radius})
	ring("the toe plug's outer", plug.lowStation, plug.lowRadius, h.toeCone)
	ring("the toe plug's inner", plug.highStation, plug.highRadius, h.toeIn)

	// Cone half-angle by cone half-angle. The heel band and the toe plug come out
	// PARALLEL, both on the back-cone family at 90 degrees minus the pitch cone
	// angle, and the root band stands at the dedendum angle to them.
	back := side.backConeHalfAngle()
	if got := heel.halfAngle(); math.Abs(got-back) > 1e-6 {
		t.Errorf("the heel band's cone half-angle is %.9f rad, want the back cone's %.9f", got, back)
	}
	if got := plug.halfAngle(); math.Abs(got-back) > 1e-6 {
		t.Errorf("the toe plug's cone half-angle is %.9f rad, want the back cone's %.9f", got, back)
	}
	if got := root.halfAngle(); math.Abs(got-side.rootGamma) > 1e-6 {
		t.Errorf("the root band's cone half-angle is %.9f rad, want the root cone angle %.9f",
			got, side.rootGamma)
	}
	// The back cone stands square to the pitch line and the root element sits one
	// dedendum angle off that line, so the two swept elements stand at 90 degrees
	// minus the dedendum angle to each other. Both directions are read off the
	// bands the proof actually built.
	direction := func(b bandReading) xy {
		return xy{b.highStation - b.lowStation, b.highRadius - b.lowRadius}
	}
	dedendumAngle := side.gamma - side.rootGamma
	if got := angleBetweenLines(direction(root), direction(heel)); math.Abs(got-(math.Pi/2-dedendumAngle)) > 1e-6 {
		t.Errorf("the root band stands at %.9f rad to the heel band, want 90 degrees less the "+
			"dedendum angle, %.9f", got, math.Pi/2-dedendumAngle)
	}
	if got := angleBetweenLines(direction(heel), direction(plug)); got > 1e-6 {
		t.Errorf("the heel band and the toe plug stand at %.9f rad to each other; both are on the "+
			"back-cone family and must come out parallel", got)
	}

	// The signed sum, against the solid-of-revolution integral taken edge by edge
	// around the §2 hexagon. The plug is SUBTRACTED: it is the dish the front
	// face hollows out of the root band.
	got := root.volume + heel.volume - plug.volume
	want := h.revolvedVolume(ringArea(1))
	// The polygonal sweep publishes each band's volume EXACTLY — measured here,
	// every bound comes back zero — so the only slack the comparison needs is the
	// floating-point noise of summing three of them.
	slack := root.volumeBound + heel.volumeBound + plug.volumeBound + 1e-9*want
	if math.Abs(got-want) > slack {
		t.Errorf("the three bands sum to %.6f mm3, want the hexagon's revolved volume %.6f "+
			"(bands' own bounds allow %.6f)", got, want, slack)
	}
	if want <= 0 {
		t.Errorf("the hexagon revolves to %.6f mm3; a non-positive volume is a folded profile", want)
	}
}

// ---------------------------------------------------------------- S12 apex loft

// apexShrink is the fraction of the tooth section the apex end is built at.
//
// Fusion lofts a degenerate POINT section — the §2 Apex sketch point — to the
// tooth profile, and decad has no point section, so the apex end is a shrunken
// copy of the same outline at the matching station. Because a cone's sections
// ARE its end section scaled linearly in station, the loft between them is
// exactly the piece of the real cone above that station, and the volume it drops
// is the apexShrink-cubed fraction the assertion accounts for.
const apexShrink = 0.02

// toothArcSamples is how many chords each of the tooth outline's two arcs is
// drawn with. Every curve in a section here is a straight chord, which is what
// makes a lofted body's volume come back EXACT rather than carrying a chord
// bound the gate then refuses; the assertions compare chorded against chorded,
// so the sagitta cancels.
const toothArcSamples = 12

// toothOutlinePoints is one gear's virtual spur tooth outline, closed, in the
// plane's own coordinates and scaled by k.
//
// The flanks are the involute samples the borrowed spur drawer lays down. Where
// the tooth is EMBEDDED — the flank starts inside the root circle, which happens
// at HIGH tooth counts and so is the normal case for a back-cone tooth number —
// the spur drawer omits the flank-to-root stubs and the flank itself runs to the
// root circle, so the samples inside it are dropped and the first kept sample is
// moved out onto the root circle. That is the same 4-curve loop the tooth
// profile selection keys on with zero lines.
func toothOutlinePoints(d bevelDesign, which, k float64) []xy {
	side := d.side(which)
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, side.virtualTeeth, spurInvoluteSteps, 0)

	clip := func(pts []involute.Pt) []xy {
		out := make([]xy, 0, len(pts))
		for _, q := range pts {
			r := math.Hypot(q.X, q.Y)
			if r < dims.Root {
				continue
			}
			out = append(out, xy{q.X, q.Y})
		}
		if len(out) == 0 || !dims.Embedded() {
			return out
		}
		// Where the tooth is embedded the flank itself runs down to the root
		// circle, so the first kept sample is pulled back onto it rather than
		// starting a fraction of a sample above.
		first := out[0]
		out[0] = first.mul(dims.Root / first.len())
		return out
	}
	leftPts, rightPts := clip(left), clip(right)
	if !dims.Embedded() {
		// The stubs: the flank start's own direction from the centre, taken in to
		// the root circle.
		foot := func(p xy) xy { return p.mul(dims.Root / p.len()) }
		leftPts = append([]xy{foot(leftPts[0])}, leftPts...)
		rightPts = append([]xy{foot(rightPts[0])}, rightPts...)
	}

	arc := func(from, to xy, radius float64) []xy {
		a0 := math.Atan2(from.y, from.x)
		a1 := math.Atan2(to.y, to.x)
		for a1-a0 > math.Pi {
			a1 -= 2 * math.Pi
		}
		for a1-a0 < -math.Pi {
			a1 += 2 * math.Pi
		}
		out := make([]xy, 0, toothArcSamples-1)
		for i := 1; i < toothArcSamples; i++ {
			a := a0 + (a1-a0)*float64(i)/float64(toothArcSamples)
			out = append(out, xy{radius * math.Cos(a), radius * math.Sin(a)})
		}
		return out
	}

	outline := make([]xy, 0, 2*len(leftPts)+2*toothArcSamples)
	outline = append(outline, leftPts...)
	outline = append(outline, arc(leftPts[len(leftPts)-1], rightPts[len(rightPts)-1], dims.Tip)...)
	for i := len(rightPts) - 1; i >= 0; i-- {
		outline = append(outline, rightPts[i])
	}
	outline = append(outline, arc(rightPts[0], leftPts[0], dims.Root)...)

	for i := range outline {
		outline[i] = outline[i].mul(k)
	}
	return outline
}

// toothSection draws that outline as a closed region in a plane perpendicular to
// the shaft axis at the given station.
func toothSection(t *testing.T, w *sketch.World, d bevelDesign, which, station, k float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane := w.XY()
	if station != 0 {
		var err error
		if plane, err = w.CreateOffsetPlane(w.XY(), station); err != nil {
			t.Fatalf("tooth section plane at %.6f mm: %v", station, err)
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth section sketch: %v", err)
	}
	outline := toothOutlinePoints(d, which, k)
	pts := make([]*sketch.Point, len(outline))
	for i, q := range outline {
		pts[i] = s.CreatePoint(q.x, q.y)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	solveHere(t, s)
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("the tooth section holds %d regions, want the one tooth outline", len(regions))
	}
	if !regions[0].Valid {
		t.Fatal("the tooth section's region is not extrudable")
	}
	return s, regions[0]
}

// stepLoftTooth builds the Tooth Body: the §2 Apex sketch point lofted to this
// gear's §3 tooth profile, which produces the tapered uncut tooth.
//
// TWO SUBSTITUTIONS, both forced. The degenerate apex point becomes a shrunken
// section, because decad has no point section. And the tooth profile's own plane
// — the back cone plane through K', built setByAngle off the Gear Profiles plane
// — becomes a plane PERPENDICULAR to the shaft axis at the same station,
// because a loft here takes two parallel sections. The cost of the second is the
// back cone's tilt: the real tooth's heel face leans by the dedendum angle, and
// what the loft proves is the taper from the apex rather than the lean of the
// face it ends on.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	station := d.toothCentreStation(which)
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
	s1, p1 := toothSection(t, w, d, which, station, 1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the apex section to the tooth profile: %v", err)
	}
	return []*decad.Body{body}
}

func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	if len(bodies) != 1 {
		t.Fatalf("the apex loft produced %d bodies, want the one Tooth Body", len(bodies))
	}
	station := d.toothCentreStation(which)
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)

	// The tooth reaches the virtual TIP radius at its heel end and nowhere
	// further, which is the size the back-cone tooth number and the Module fix.
	// The reach is read off the body's own vertices; an axis-aligned bounding box
	// would answer with its corner instead.
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("tooth bounds: %v", err)
	}
	reach := 0.0
	for _, v := range bodies[0].Vertices() {
		q := v.Position().Value
		reach = math.Max(reach, math.Hypot(q.X, q.Y))
	}
	if math.Abs(reach-dims.Tip) > 1e-6 {
		t.Errorf("the tooth reaches %.6f mm from the shaft axis, want the virtual tip radius %.6f",
			reach, dims.Tip)
	}
	if got := box.Max.Z - box.Min.Z; math.Abs(got-station*(1-apexShrink)) > 1e-6 {
		t.Errorf("the tooth spans %.6f mm along the shaft, want the apex-to-heel run %.6f",
			got, station*(1-apexShrink))
	}
	if math.Abs(box.Max.Z-station) > 1e-6 {
		t.Errorf("the tooth's heel face sits at station %.6f, want the tooth centre's %.6f",
			box.Max.Z, station)
	}

	// The taper is the cone's own: a section at station z is the heel section
	// scaled by z / station, so the volume is the heel area times the station,
	// over three, less the apex fraction the shrunken end drops.
	w := sketch.NewWorld()
	_, region := toothSection(t, w, d, which, station, 1)
	want := region.Area * station * (1 - apexShrink*apexShrink*apexShrink) / 3
	measured, err := bodies[0].Volume()
	if err != nil {
		t.Fatalf("tooth volume: %v", err)
	}
	if got := measured.Value.Base(); math.Abs(got-want) > measured.Bound.Base()+1e-6*want {
		t.Errorf("the lofted tooth is %.6f mm3, want the tapered cone's %.6f (bound %.6f)",
			got, want, measured.Bound.Base())
	}
}

// ---------------------------------------------------------------- S13 conical end cuts

// stepConicalEndCuts builds the three bodies the flush trim is made of and lays
// them apart: the uncut tooth, and the two cone faces of the Gear Body the cut
// uses as tools — the toe cone the toe edge M->N sweeps and the heel cone the
// heel edge C->H sweeps.
//
// PERFORM NEITHER CUT. Both operands are Lofts — the tooth and each cone alike —
// so the split is unavailable. The two cutting TOOLS are cone faces of the GEAR
// BODY, never of the tooth, which has no cone face at all; that is the
// conflation the spec is explicit about, and it is why searching the tooth for
// the cone finds none.
//
// THE COST IS THE SPLIT: the proof does not show the evaluator dividing the
// tooth, selecting the keeper by dropping the apex-containing piece and keeping
// the largest, or leaving a watertight body behind. What it does show is where
// each cut lands, read off the cones and the tooth as built.
func stepConicalEndCuts(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	h := d.axialProfile(which)
	station := d.toothCentreStation(which)
	sep := d.bandSeparation(which)

	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
	s1, p1 := toothSection(t, w, d, which, station, 1)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the uncut tooth: %v", err)
	}
	return []*decad.Body{
		tooth,
		ringSweep(t, doc, h.toeCone.station, h.toeCone.radius, h.toeIn.station, h.toeIn.radius, sep),
		ringSweep(t, doc, h.ded.station, h.ded.radius, h.heelAxis.station, h.heelEnd.radius, 2*sep),
	}
}

func assertConicalEndCuts(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	if len(bodies) != 3 {
		t.Fatalf("the end-cut substitution produced %d bodies, want the tooth and its two cones", len(bodies))
	}
	toeCone := readBand(t, bodies[1], sep, "toe cone")
	heelCone := readBand(t, bodies[2], 2*sep, "heel cone")

	// Each cone's half-angle and apex, read off the cone itself. Both stand on
	// the back-cone family, so they are PARALLEL and differ only in where their
	// apexes sit on the shaft.
	back := side.backConeHalfAngle()
	if got := toeCone.halfAngle(); math.Abs(got-back) > 1e-6 {
		t.Errorf("the toe cone's half-angle is %.9f rad, want the back cone's %.9f", got, back)
	}
	if got := heelCone.halfAngle(); math.Abs(got-back) > 1e-6 {
		t.Errorf("the heel cone's half-angle is %.9f rad, want the back cone's %.9f", got, back)
	}
	apexOf := func(b bandReading) float64 {
		slope, intercept := b.element()
		return -intercept / slope
	}
	toeApex, heelApex := apexOf(toeCone), apexOf(heelCone)
	if heelApex <= toeApex {
		t.Errorf("the heel cone's apex is at station %.6f and the toe cone's at %.6f; the heel cone "+
			"must sit further out or the trim has inverted", heelApex, toeApex)
	}

	// Each cone passes through the §2 points its edge was drawn between, read off
	// the body rather than off the numbers it was built from.
	ring := func(label string, gotStation, gotRadius float64, want axialPt) {
		if math.Abs(gotStation-want.station) > 1e-6 || math.Abs(gotRadius-want.radius) > 1e-6 {
			t.Errorf("%s is at station %.6f radius %.6f, want station %.6f radius %.6f",
				label, gotStation, gotRadius, want.station, want.radius)
		}
	}
	ring("the toe cone's outer ring", toeCone.lowStation, toeCone.lowRadius, h.toeCone)
	ring("the toe cone's inner ring", toeCone.highStation, toeCone.highRadius, h.toeIn)
	ring("the heel cone's inner ring", heelCone.lowStation, heelCone.lowRadius, h.ded)
	ring("the heel cone's outer ring", heelCone.highStation, heelCone.highRadius,
		axialPt{station: h.heelAxis.station, radius: h.heelEnd.radius})

	// The tooth's own two surfaces, read off the tooth as built: the tip cone it
	// reaches at its widest and the root cone it seats on.
	station := d.toothCentreStation(which)
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)
	tipReach, rootReach := 0.0, math.Inf(1)
	for _, v := range bodies[0].Vertices() {
		q := v.Position().Value
		if math.Abs(q.Z-station) > 1e-6 {
			continue
		}
		r := math.Hypot(q.X, q.Y)
		tipReach = math.Max(tipReach, r)
		rootReach = math.Min(rootReach, r)
	}
	if math.Abs(tipReach-dims.Tip) > 1e-6 || math.Abs(rootReach-dims.Root) > 1e-6 {
		t.Errorf("the tooth's heel section runs from %.6f to %.6f mm, want the virtual root and tip "+
			"radii %.6f and %.6f", rootReach, tipReach, dims.Root, dims.Tip)
	}
	tip := bandReading{lowStation: 0, lowRadius: 0, highStation: station, highRadius: tipReach}
	root := bandReading{lowStation: 0, lowRadius: 0, highStation: station, highRadius: rootReach}

	// Where each cut lands. BOTH cones cross BOTH of the tooth's surfaces, because
	// a cut passes through the whole body; what the flush band needs is that every
	// toe crossing sits strictly nearer the apex than every heel crossing, and
	// that each crossing falls inside the tooth's own span so the cut meets the
	// tooth at all. A toe cut that does not split is the failure the spec says
	// propagates and crashes the build, and an inverted frame is what makes the
	// toe cone miss.
	cuts := map[string]float64{}
	for name, pair := range map[string][2]bandReading{
		"toe on the tip":   {toeCone, tip},
		"toe on the root":  {toeCone, root},
		"heel on the tip":  {heelCone, tip},
		"heel on the root": {heelCone, root},
	} {
		where, ok := crossStation(pair[0], pair[1])
		if !ok {
			t.Errorf("the %s cut never meets its surface; the cone and the tooth surface are parallel", name)
			continue
		}
		if where <= 0 || where > station {
			t.Errorf("the %s cut falls at station %.6f, outside the tooth's own span (0, %.6f]; "+
				"the cone misses the tooth", name, where, station)
		}
		cuts[name] = where
	}
	toeCut := math.Max(cuts["toe on the tip"], cuts["toe on the root"])
	heelCut := math.Min(cuts["heel on the tip"], cuts["heel on the root"])
	if toeCut >= heelCut {
		t.Errorf("the toe cut reaches station %.6f and the heel cut %.6f; the flush band is empty "+
			"and the trimmed tooth would be inverted", toeCut, heelCut)
	}

	// WHAT IS NOT REPRODUCED. The spec reads the cut's signature as the two ends
	// landing on DIFFERENT surfaces of the tooth, the toe on its tip and the heel
	// on its root. Measured here, each cone crosses BOTH of the tooth's surfaces,
	// which is what a cut through a solid does; which surface carries the new face
	// is decided by the keeper selection, and the keeper needs the split. So the
	// proof checks where the four crossings are and that they leave a non-empty
	// band, and leaves the surface the cut face ends up on to a Fusion session.
	//
	// The band's LENGTH is not checked against the hexagon's toe-to-heel run
	// either, and that is the apex loft's substitution showing through rather than
	// a gap here: the tooth was lofted to a section PERPENDICULAR to the shaft
	// where the real one ends on the tilted back cone, so its root surface is not
	// the gear body's root cone and the two cones meet it at their own stations.
	// What survives the substitution is the ORDER of those stations and that each
	// falls inside the tooth, which is what the cut needs to split at all.
}

// ---------------------------------------------------------------- S22 circular pattern

// THIS STEP IS SERIAL, and these four package-level readings are why.
//
// The pattern increment retires the seed tooth — decad's Placed consumes the
// body it moves — so the seed cannot be measured after the step runs, and its
// azimuth, radius, height and volume have to be read during the build and handed
// to the assertion. That hand-off leaves the case, and two cases sharing one set
// of readings overwrite each other. It is not a hazard that announces itself:
// the two gear sides differ enough in volume that an overwrite was caught when
// it happened, and a pair of cases whose seeds measured alike would have passed
// on each other's numbers instead. So stepCircularPattern keeps
// proofkit3d.RunSolid while every other bevel step runs its cases in parallel.
var (
	patternSeedAzimuth float64
	patternSeedRadius  float64
	patternSeedHeight  float64
	patternSeedVolume  float64
)

// stepCircularPattern places this gear's teeth around the shaft axis: Teeth
// Number copies over a total angle of 360 degrees, not symmetric, so copy k
// sits at 2*pi*k/N.
//
// The step builds the seed tooth, reads what the pattern must preserve, and
// returns copy 1 — the first increment — which is what retires the seed. The
// remaining copies are measured one document at a time in the assertion, because
// decad verifies every PAIR of live bodies and a real gear's teeth resolve
// neither as disjoint nor as overlapping, which the gate refuses.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	station := d.toothCentreStation(which)
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
	s1, p1 := toothSection(t, w, d, which, station, 1)
	seed, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the seed tooth: %v", err)
	}
	patternSeedAzimuth = azimuthOf(t, seed)
	patternSeedRadius = reachOf(t, seed)
	box, err := seed.Bounds()
	if err != nil {
		t.Fatalf("seed tooth bounds: %v", err)
	}
	patternSeedHeight = box.Max.Z - box.Min.Z
	patternSeedVolume = volumeOf(t, seed, "seed tooth")

	copied, err := seed.Placed(turnAbout(t, 2*math.Pi/d.side(which).teeth))
	if err != nil {
		t.Fatalf("place the first pattern copy: %v", err)
	}
	return []*decad.Body{copied}
}

func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	teeth := int(d.side(which).teeth)
	if len(bodies) != 1 {
		t.Fatalf("the pattern step returned %d bodies, want the first copy", len(bodies))
	}
	// The three inputs the spec makes the step set explicitly are readable from
	// where the copies land. Quantity is how many there are; the total angle and
	// the not-symmetric flag together say copy k sits at 2*pi*k/N measured ONE
	// way from the seed, which a symmetric pattern or a different total angle
	// would not produce.
	check := func(k int, body *decad.Body) {
		want := patternSeedAzimuth + 2*math.Pi*float64(k)/float64(teeth)
		if got := azimuthOf(t, body); angleGap(got, want) > 1e-9 {
			t.Errorf("patterned tooth %d sits at %.9f rad, want %.9f", k, got, want)
		}
		if got := volumeOf(t, body, "patterned tooth"); math.Abs(got-patternSeedVolume) > 1e-9*patternSeedVolume {
			t.Errorf("patterned tooth %d is %.9f mm3, want the seed's %.9f", k, got, patternSeedVolume)
		}
		if got := reachOf(t, body); math.Abs(got-patternSeedRadius) > 1e-9 {
			t.Errorf("patterned tooth %d reaches %.9f mm, want the seed's %.9f", k, got, patternSeedRadius)
		}
		box, err := body.Bounds()
		if err != nil {
			t.Fatalf("patterned tooth %d bounds: %v", k, err)
		}
		if got := box.Max.Z - box.Min.Z; math.Abs(got-patternSeedHeight) > 1e-9 {
			t.Errorf("patterned tooth %d spans %.9f mm, want the seed's %.9f", k, got, patternSeedHeight)
		}
	}
	check(1, bodies[0])

	// The angular spacing stays 360/N for the WHOLE face width even though the
	// pitch diameter shrinks toward the apex: the radial taper is already in the
	// loft, so the pattern rotates one tapered tooth rather than scaling it.
	station := d.toothCentreStation(which)
	for k := 2; k < teeth; k++ {
		scratch := decad.New()
		w := sketch.NewWorld()
		s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
		s1, p1 := toothSection(t, w, d, which, station, 1)
		body, err := scratch.Loft(s0, p0, s1, p1)
		if err != nil {
			t.Fatalf("loft pattern copy %d: %v", k, err)
		}
		placed, err := body.Placed(turnAbout(t, 2*math.Pi*float64(k)/float64(teeth)))
		if err != nil {
			t.Fatalf("place pattern copy %d: %v", k, err)
		}
		check(k, placed)
	}
}

// turnAbout is one rotation about the shaft axis, built from a literal basis
// rather than from an axis and an angle: r3.RotationAround evaluates Rodrigues'
// formula, whose z row comes out a few float ulps off for most angles, and that
// is enough to move a section off the plane decad's exact reductions require it
// to stay on.
func turnAbout(t *testing.T, angle float64) r3.Transform {
	t.Helper()
	sin, cos := math.Sin(angle), math.Cos(angle)
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(cos, sin, 0),
		EY: r3.NewVec(-sin, cos, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 0, 0))
	if err != nil {
		t.Fatalf("rotation of %.9f rad about the shaft axis: %v", angle, err)
	}
	return turn
}

func azimuthOf(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	return math.Atan2(centroid.Value.Y, centroid.Value.X)
}

func reachOf(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	reach := 0.0
	for _, v := range body.Vertices() {
		q := v.Position().Value
		reach = math.Max(reach, math.Hypot(q.X, q.Y))
	}
	return reach
}

func volumeOf(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return measured.Value.Base()
}

func angleGap(a, b float64) float64 {
	gap := math.Mod(math.Abs(a-b), 2*math.Pi)
	return math.Min(gap, 2*math.Pi-gap)
}

// ---------------------------------------------------------------- S23 combine

// combineSink is how far the proof pushes the tooth's root below the gear body's
// root cone before reading the two bodies against each other.
//
// ⚠️ THE SINK BELONGS TO THE PROOF ALONE. The generated module seats the tooth
// EXACTLY on the cone and must not sink it. The sink is here so that "seated,
// not floating" is measurable as a strict inequality rather than as an equality
// inside a tolerance, and it is a twentieth of the tooth's height, so it scales
// with the gear and stays far below the dedendum.
func combineSink(d bevelDesign, which float64) float64 {
	side := d.side(which)
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)
	return (dims.Tip - dims.Root) / 20
}

// seatedToothCone is the tooth seated on the gear body's root cone: the radius
// its root and its tip ride at, at one station.
//
// In Fusion the tooth is lofted to a profile on the TILTED back-cone plane, so
// its root lands on the gear's root cone by construction. The proof's sections
// are perpendicular to the shaft, so the seating is applied as the uniform scale
// that puts the tooth's root radius on that cone — the same tooth outline, at
// the radius the tilted plane would have given it.
func seatedToothCone(d bevelDesign, which, station float64) (root, tip float64) {
	side := d.side(which)
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)
	gearRoot := station * math.Tan(side.rootGamma)
	seat := (gearRoot - combineSink(d, which)) / dims.Root
	return seat * dims.Root, seat * dims.Tip
}

// stepCombineTeeth joins the patterned teeth into the Gear Body in a single
// Combine-Join, the Gear Body as the target and the patterned teeth as the
// tools.
//
// PERFORM NO JOIN. Both operands are Lofts, so the boolean is unavailable. The
// two are laid apart and the join's two consequences are read off their own
// geometry instead: a join leaves ONE lump when the tooth's root is at or below
// the body's root cone — seated, not floating — and the joined body reaches
// further out than the frustum when the tooth's tip stands proud of it.
//
// THE COST IS THE STITCH: the proof cannot show the evaluator making one
// boundary out of two.
func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	station := d.toothCentreStation(which)

	gearRootBand := ringSweep(t, doc, h.toeCone.station, h.toeCone.radius, h.ded.station, h.ded.radius, 0)

	// The seated tooth, as the apex loft with the seating scale applied.
	seatRoot, _ := seatedToothCone(d, which, station)
	dims := involute.Derive(d.module, d.side(which).virtualTeeth, involutePressureAngle)
	seat := seatRoot / dims.Root
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station+sep, apexShrink*seat)
	s1, p1 := toothSection(t, w, d, which, station+sep, seat)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the seated tooth: %v", err)
	}
	return []*decad.Body{gearRootBand, tooth}
}

func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	if len(bodies) != 2 {
		t.Fatalf("the combine substitution produced %d bodies, want the Gear Body band and the tooth", len(bodies))
	}
	band := readBand(t, bodies[0], 0, "gear body root band")
	toothReach := 0.0
	toothSeat := math.Inf(1)
	for _, v := range bodies[1].Vertices() {
		q := v.Position().Value
		if math.Abs(q.Z-sep-d.toothCentreStation(which)) > 1e-6 {
			continue
		}
		r := math.Hypot(q.X, q.Y)
		toothReach = math.Max(toothReach, r)
		toothSeat = math.Min(toothSeat, r)
	}

	// Both readings are taken at the toe, the middle and the heel of the band the
	// join would cover.
	slope, intercept := band.element()
	for _, where := range []struct {
		label   string
		station float64
	}{
		{"toe", h.toeCone.station},
		{"middle", (h.toeCone.station + h.ded.station) / 2},
		{"heel", h.ded.station},
	} {
		gearRadius := slope*where.station + intercept
		toothRoot, toothTip := seatedToothCone(d, which, where.station)
		if toothRoot >= gearRadius {
			t.Errorf("at the %s the tooth's root rides at %.6f mm and the gear body's root cone at "+
				"%.6f; a tooth that does not sit at or below the cone floats and the join leaves a gap",
				where.label, toothRoot, gearRadius)
		}
		if toothTip <= gearRadius {
			t.Errorf("at the %s the tooth's tip reaches %.6f mm and the gear body's root cone %.6f; "+
				"a tooth that does not stand proud adds nothing to the joined body",
				where.label, toothTip, gearRadius)
		}
	}

	// The seated tooth really is the tooth this gear's virtual tooth number and
	// Module draw, scaled onto the cone rather than reshaped.
	dims := involute.Derive(d.module, side.virtualTeeth, involutePressureAngle)
	if toothSeat <= 0 || math.Abs(toothReach/toothSeat-dims.Tip/dims.Root) > 1e-9 {
		t.Errorf("the seated tooth's tip-to-root ratio is %.9f, want the virtual tooth's %.9f",
			toothReach/toothSeat, dims.Tip/dims.Root)
	}
	if got := combineSink(d, which); got <= 0 {
		t.Errorf("the proof's sink is %.9f mm; it has to be positive for `seated` to be a strict reading", got)
	}
}

// ---------------------------------------------------------------- S25 bore cut

// stepBoreCut cuts the optional cylindrical through bore along the shaft axis.
//
// THE TOOL IS A REAL EXTRUDE, which a symmetric extent produces as a prism, and
// the extent is the spec's own: 2 * Cone Distance per side, generously past any
// face width. NO CUT IS PERFORMED — the target is the frustum, whose bands are
// Lofts — so the tool and the frustum's root band are laid apart and the cut is
// read off the tool's own geometry.
//
// THE COST IS THE PIERCED BODY: one lump with a hole and no enclosed void is not
// shown.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	h := d.axialProfile(which)
	if side.boreDiameter <= 0 {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so the bore step is skipped entirely")
	}
	sep := d.bandSeparation(which)
	band := ringSweep(t, doc, h.toeCone.station, h.toeCone.radius, h.ded.station, h.ded.radius, 0)

	// The bore plane is rooted at the shaft-axis edge's START — the hexagon's
	// first edge begins at the front face's foot — so the symmetric extent is
	// centred there.
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), h.toeFoot.station+sep)
	if err != nil {
		t.Fatalf("bore plane: %v", err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("bore sketch: %v", err)
	}
	if _, err := s.CreatePolygon(0, 0, sweepSides, side.boreDiameter/2); err != nil {
		t.Fatalf("bore circle: %v", err)
	}
	solveHere(t, s)
	tool, err := doc.Extrude(s, s.Profiles()[0],
		decad.Symmetric{D: millimetres(2 * d.coneDistance), FullLength: false})
	if err != nil {
		t.Fatalf("extrude the bore tool: %v", err)
	}
	return []*decad.Body{band, tool}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	side := d.side(which)
	h := d.axialProfile(which)
	sep := d.bandSeparation(which)
	if len(bodies) != 2 {
		t.Fatalf("the bore substitution produced %d bodies, want the frustum band and the tool", len(bodies))
	}
	tool := bodies[1]
	box, err := tool.Bounds()
	if err != nil {
		t.Fatalf("bore tool bounds: %v", err)
	}
	start := h.toeFoot.station + sep
	if got := box.Min.Z; math.Abs(got-(start-2*d.coneDistance)) > 1e-6 {
		t.Errorf("the bore tool starts at station %.6f, want 2 * Cone Distance below the shaft "+
			"edge's start, %.6f", got-sep, start-2*d.coneDistance-sep)
	}
	if got := box.Max.Z; math.Abs(got-(start+2*d.coneDistance)) > 1e-6 {
		t.Errorf("the bore tool ends at station %.6f, want 2 * Cone Distance above the shaft "+
			"edge's start, %.6f", got-sep, start+2*d.coneDistance-sep)
	}
	// A THROUGH cut: both ends of the tool clear the frustum, which spans from
	// the toe corner to the heel end.
	if box.Min.Z-sep >= h.toeCone.station || box.Max.Z-sep <= h.heelAxis.station {
		t.Errorf("the bore tool spans stations %.6f to %.6f and the frustum %.6f to %.6f; a tool "+
			"that does not clear both ends is not a through cut",
			box.Min.Z-sep, box.Max.Z-sep, h.toeCone.station, h.heelAxis.station)
	}
	if got := reachOf(t, tool); math.Abs(got-side.boreDiameter/2) > 1e-6 {
		t.Errorf("the bore tool's radius is %.6f mm, want the bore diameter's half %.6f",
			got, side.boreDiameter/2)
	}

	// What the cut would remove: the frustum's own profile clipped to the bore
	// radius, revolved. It has to be a real bite — positive, and short of the
	// whole frustum.
	removed := h.revolvedVolumeClipped(ringArea(1), side.boreDiameter/2)
	whole := h.revolvedVolume(ringArea(1))
	if removed <= 0 {
		t.Errorf("the bore would remove %.6f mm3; a through bore of %.6f mm has to take material",
			removed, side.boreDiameter)
	}
	if removed >= whole {
		t.Errorf("the bore would remove %.6f mm3 of a %.6f mm3 frustum; it would take the whole body",
			removed, whole)
	}
}

// ---------------------------------------------------------------- S26 meshing rotation

// stepMeshRotation applies the meshing rotation: the DRIVING gear turns half a
// tooth pitch, 180 degrees over its own Teeth Number, about its shaft axis, so a
// driving valley sits where the pinion tooth crosses the axial plane. The pinion
// takes its own mesh phase, which is 0 for a straight bevel and 0 by default for
// a spiral one because the mid-face section is left unrotated.
//
// ⚠️ A ZERO ANGLE IS A NO-OP, NOT A MOVE. Fusion refuses to move a body by the
// identity with `RuntimeError: 3 : invalid transform`, measured on the bevel
// pinion whose mesh phase is 0 by default, and the framework's
// rotate_body_about_edge returns early for exactly that reason. r3 refuses the
// zero angle too — Rotation rejects a zero units.Value — so the proof takes the
// same early return.
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newBevelDesign(p)
	which := p["gearSide"]
	station := d.toothCentreStation(which)
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
	s1, p1 := toothSection(t, w, d, which, station, 1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the tooth to rotate: %v", err)
	}
	angle := meshRotation(d, which)
	if angle == 0 {
		return []*decad.Body{body}
	}
	turned, err := body.Placed(turnAbout(t, angle))
	if err != nil {
		t.Fatalf("rotate the gear body by %.9f rad: %v", angle, err)
	}
	return []*decad.Body{turned}
}

// meshRotation is this gear's own mesh offset in radians: half a tooth pitch for
// the driving gear, and the pinion's _PINION_MESH_PHASE_TEETH tooth-fractions —
// default 0 — for the pinion.
func meshRotation(d bevelDesign, which float64) float64 {
	if which == sideDriving {
		return math.Pi / d.driving.teeth
	}
	return pinionMeshPhaseTeeth * 2 * math.Pi / d.pinion.teeth
}

// pinionMeshPhaseTeeth is _PINION_MESH_PHASE_TEETH, the pinion's extra mesh
// rotation in tooth-fractions. It is 0: the spiral build leaves the mid-face
// section unrotated, so the pinion already meshes.
const pinionMeshPhaseTeeth = 0.0

func assertMeshRotation(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newBevelDesign(p)
	which := p["gearSide"]
	if len(bodies) != 1 {
		t.Fatalf("the mesh rotation produced %d bodies, want the one gear body", len(bodies))
	}
	station := d.toothCentreStation(which)
	scratch := decad.New()
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, d, which, apexShrink*station, apexShrink)
	s1, p1 := toothSection(t, w, d, which, station, 1)
	unturned, err := scratch.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the unrotated tooth: %v", err)
	}
	want := meshRotation(d, which)
	got := azimuthOf(t, bodies[0]) - azimuthOf(t, unturned)
	if angleGap(got, want) > 1e-9 {
		t.Errorf("the %s gear turned %.9f rad, want %.9f", d.side(which).label, got, want)
	}
	if which == sideDriving {
		half := math.Pi / d.driving.teeth
		if math.Abs(want-half) > 1e-12 {
			t.Errorf("the driving mesh rotation is %.9f rad, want half a tooth pitch %.9f", want, half)
		}
	} else if want != 0 {
		t.Errorf("the pinion mesh phase is %.9f rad, want 0 unless a spiral pair asks for one", want)
	}
	// The rotation moves the body and changes nothing about it.
	if before, after := volumeOf(t, unturned, "unrotated"), volumeOf(t, bodies[0], "rotated"); math.Abs(after-before) > 1e-9*before {
		t.Errorf("the rotated body is %.9f mm3 against %.9f before; a rotation is not a reshaping",
			after, before)
	}
}
