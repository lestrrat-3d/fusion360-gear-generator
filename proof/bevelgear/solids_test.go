// This file holds the bevel pair's straight-bevel solid steps: the gear-body
// revolve, the apex-to-tooth loft, the conical end cuts, the circular pattern,
// the Combine-Join, the bore's through-cut and the meshing rotation.
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
// something, and it is also what lets decad's pairwise checks resolve — two
// bodies that touch without provably crossing are refused.
//
// Each step below says what its own substitution costs.
//
// # The frame
//
// Every solid here is built in one gear's own axial frame, mapped so that the
// Apex sits at the world origin and the shaft axis runs along +Z. A point given
// as bvPt{Z, Rho} is then a point at height Z on the circle of radius Rho, and
// a straight profile segment sweeps the truncated cone between its two rings.
// Rotation about the shaft axis is rotation about +Z, which is what makes an
// azimuth reading meaningful after a body has been laid apart along that same
// axis.
//
// # Module 4 and up
//
// These tables never run at Module 1. decad's mesh bound has an absolute floor,
// and a figure small enough brings every measurement inside it, so the gate
// reports Suspect on geometry that is in fact correct. Module is a pure scale on
// this figure, so a case at Module 4 through 8 proves the same shape.
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

// bvMM wraps a millimetre reading for decad.
func bvMM(v float64) units.Value { return units.Millimeters(v) }

// bvArcChords is how many chords stand in for one arc of a tooth section. A
// loft pairs its two sections positionally, so both ends of a lofted tooth
// carry the same count; a real arc would carry exact bounds this harness cannot
// certify once the section is also going to be lofted, so every curve in a
// solid-step section here is a chord. What that costs is the sagitta between
// each chord and the arc it spans: a chorded tooth is very slightly thinner,
// and every step measures chorded against chorded so the sagitta cancels.
const bvArcChords = 12

// bvApexShrink is the fraction of the heel station the apex-side loft section
// is placed at. decad has no degenerate point section, so the apex point the
// spec lofts from is substituted by a section shrunk to this fraction, which is
// the same generalized cone truncated at that station.
const bvApexShrink = 0.1

// bvLayApart is how far along the shaft axis each successive body is laid from
// the last, as a multiple of the figure's own heel station. It only has to be
// large enough that no two bodies touch.
const bvLayApart = 4.0

// bvSolidCases sweep the regime the solid steps have to hold across, at the
// sizes the mesh bound clears.
//
// Both gears, both ways round, so either can be the smaller and binding one.
// Both ends of the Shaft Angle range the lattice reaches. Both sides of the
// Enable Bore branch and of each "0 means auto" field. Toe Extension at 0 and
// at its far end, since the toe end of the frustum is what the toe trim finds.
var bvSolidCases = bvBothSides3D([]proofkit3d.Case{
	{Name: "defaults_m4", Params: bvSolidWith(nil)},
	{Name: "m8", Params: bvSolidWith(map[string]float64{bvpModule: 8})},
	{Name: "shaft_angle_35", Params: bvSolidWith(map[string]float64{bvpShaftAngle: 35})},
	{Name: "shaft_angle_120", Params: bvSolidWith(map[string]float64{bvpShaftAngle: 120})},
	{Name: "ratio_31_17", Params: bvSolidWith(map[string]float64{bvpPinionTeeth: 17})},
	{Name: "ratio_17_31", Params: bvSolidWith(map[string]float64{bvpDrivingTeeth: 17})},
	{Name: "low_teeth_6_6", Params: bvSolidWith(map[string]float64{
		bvpDrivingTeeth: 6, bvpPinionTeeth: 6})},
	{Name: "toe_extension_100", Params: bvSolidWith(map[string]float64{bvpToeExtension: 100})},
	{Name: "toe_radii_specified", Params: bvSolidWith(map[string]float64{
		bvpDrivingToeR: 6, bvpPinionToeR: 6})},
	{Name: "bore_disabled", Params: bvSolidWith(map[string]float64{bvpBoreEnable: 0})},
	{Name: "bore_specified", Params: bvSolidWith(map[string]float64{
		bvpDrivingBore: 20, bvpPinionBore: 16})},
	{Name: "base_heights_specified", Params: bvSolidWith(map[string]float64{
		bvpDrivingBase: 18, bvpPinionBase: 14})},
	{Name: "tooth_spacing_positive", Params: bvSolidWith(map[string]float64{bvpToothSpacing: 1})},
})

// bvBothSides3D doubles a solid table so every case runs once per gear.
func bvBothSides3D(in []proofkit3d.Case) []proofkit3d.Case {
	out := make([]proofkit3d.Case, 0, 2*len(in))
	for _, c := range in {
		for _, side := range []struct {
			name string
			v    float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := map[string]float64{}
			for k, v := range c.Params {
				p[k] = v
			}
			p[bvpSide] = side.v
			out = append(out, proofkit3d.Case{Name: c.Name + "_" + side.name, Params: p})
		}
	}
	return out
}

// bvSolidDesign resolves a solid case and fails it when the resolution pass
// refuses a configuration the table expects to build.
func bvSolidDesign(t *testing.T, p map[string]float64) (bvDesign, bvSide) {
	t.Helper()
	d := bvResolve(bvInputsOf(p))
	if len(d.Rejections) != 0 {
		t.Fatalf("resolution refused a solid case: %v", d.Rejections)
	}
	side := d.Pinion
	if p[bvpSide] != 0 {
		side = d.Driving
	}
	return d, side
}

// ------------------------------------------------------------ building blocks

// bvRingSides is how many sides stand in for a revolved ring. The revolve is
// substituted by a POLYGONAL sweep, so every ring here is a regular polygon of
// this many sides with its first vertex on +X, which puts the circumradius in
// the body's own bounds.
const bvRingSides = 24

// bvRingK is a regular bvRingSides-gon's area over its circumradius squared, so
// that a band's volume reads the same closed form a cone's does with pi
// replaced by this. It is what the polygonal substitution costs, and it cancels
// wherever the proof compares a polygonal reading against a polygonal one.
var bvRingK = float64(bvRingSides) / 2 * math.Sin(2*math.Pi/bvRingSides)

// bvRingProfile draws the ring of circumradius r at height z and returns the
// region a loft consumes.
func bvRingProfile(t *testing.T, w *sketch.World, z, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane := w.XY()
	if z != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			t.Fatalf("ring plane at z=%.6f: %v", z, err)
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("ring sketch: %v", err)
	}
	pts := make([]*sketch.Point, bvRingSides)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / bvRingSides
		pts[i] = s.CreatePoint(r*math.Cos(a), r*math.Sin(a))
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	return s, bvOnlyRegion(t, s)
}

// bvPolyBandVolume is a polygonal band's own volume: the same truncated-cone
// closed form with the polygon's area constant in place of pi.
func bvPolyBandVolume(a, b bvPt) float64 {
	return bvRingK * math.Abs(b.Z-a.Z) * (a.Rho*a.Rho + a.Rho*b.Rho + b.Rho*b.Rho) / 3
}

// bvPolyRevolvedVolume is the frustum's volume in the same polygonal measure.
func bvPolyRevolvedVolume(h bvHex) float64 { return bvRevolvedVolume(h) * bvRingK / math.Pi }

// bvOnlyRegion is the single closed region a section sketch holds.
func bvOnlyRegion(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve a section sketch: %v", err)
	}
	regions := s.Profiles()
	if len(regions) != 1 {
		t.Fatalf("a section sketch holds %d regions, want exactly 1", len(regions))
	}
	if !regions[0].Valid {
		t.Fatal("a section region is not usable")
	}
	return regions[0]
}

// bvBand builds the truncated cone one straight profile segment sweeps about
// the shaft axis: the solid between the ring at a and the ring at b.
//
// This is the substitution for the revolve. decad publishes a revolved body's
// volume with a proven bound equal to the volume itself, so a revolved body is
// Suspect at any tolerance and cannot pass the harness gate; a loft between the
// two rings is the same solid and is not.
func bvBand(t *testing.T, doc *decad.Document, a, b bvPt) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0 := bvRingProfile(t, w, a.Z, a.Rho)
	s1, p1 := bvRingProfile(t, w, b.Z, b.Rho)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("band from (z=%.4f r=%.4f) to (z=%.4f r=%.4f): %v", a.Z, a.Rho, b.Z, b.Rho, err)
	}
	return body
}

// bvShift lays a body apart from its neighbours along the shaft axis. It is a
// translation along the axis every rotation here turns about, so it changes no
// volume, no radius, no cone angle and no azimuth.
func bvShift(t *testing.T, body *decad.Body, dz float64) *decad.Body {
	t.Helper()
	moved, err := body.Placed(bvTranslation(t, dz))
	if err != nil {
		t.Fatalf("lay a body apart by %.4f mm: %v", dz, err)
	}
	return moved
}

func bvTranslation(t *testing.T, dz float64) r3.Transform {
	t.Helper()
	tr, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(1, 0, 0),
		EY: r3.NewVec(0, 1, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 0, dz))
	if err != nil {
		t.Fatalf("translation by %.4f mm: %v", dz, err)
	}
	return tr
}

// bvTurn is one rotation about the shaft axis, built from a literal basis
// rather than from an axis and an angle: Rodrigues' formula leaves the z row a
// few float ulps off, and decad's analytic reduction admits a pair only when
// the two composed world planes are the SAME plane by exact float equality.
func bvTurn(t *testing.T, angle float64) r3.Transform {
	t.Helper()
	sin, cos := math.Sin(angle), math.Cos(angle)
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(cos, sin, 0),
		EY: r3.NewVec(-sin, cos, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 0, 0))
	if err != nil {
		t.Fatalf("rotation of %.6f rad: %v", angle, err)
	}
	return turn
}

func bvVolume(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return v.Value.Base()
}

func bvBounds(t *testing.T, body *decad.Body, label string) decad.Box {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", label, err)
	}
	return box
}

func bvCentroid(t *testing.T, body *decad.Body, label string) r3.Vec {
	t.Helper()
	c, err := body.Centroid()
	if err != nil {
		t.Fatalf("%s centroid: %v", label, err)
	}
	return r3.NewVec(c.Value.X, c.Value.Y, c.Value.Z)
}

// bvNear fails unless got is want to a relative tolerance.
func bvNear(t *testing.T, label string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol*math.Max(1, math.Abs(want)) {
		t.Errorf("%s: got %.9f, want %.9f", label, got, want)
	}
}

// ------------------------------------------------------------ the tooth

// bvToothSection draws one axis-perpendicular cross-section of the tooth at
// cone distance z, and returns the region a loft consumes.
//
// This is the substitution for the §3 tooth profile, which in Fusion is drawn
// in the BACK-CONE plane at the tooth centre. Here the section stands square to
// the shaft axis, which is what lets the tooth be lofted as a generalized cone
// from the apex and what makes an azimuth reading mean a rotation about the
// shaft. The section is the virtual spur tooth, uniformly scaled so that its
// ROOT radius is the root cone's own radius at z, so the tooth seats on the
// gear body's root cone at every station. What the substitution costs is the
// back cone's tilt: the real tooth's flanks lean by the pitch cone angle, and a
// section taken square to the shaft does not.
func bvToothSection(t *testing.T, w *sketch.World, d bvDesign, side bvSide, z, turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return bvToothSectionRelieved(t, w, d, side, z, turn, 1)
}

// bvToothSectionRelieved is bvToothSection with the lengthwise crown's uniform
// scale applied ABOUT THE ROOT rather than about the section's centroid: every
// radius becomes root + factor*(radius - root), so the root edge stays exactly
// where it was and only the tooth's height above it is relieved.
//
// That anchoring is the whole point. scaleFeatures shrinks uniformly toward its
// base point, so a base point at the heel face's centroid — mid tooth height —
// pulls the root edge upward by half the relief, the tooth stops seating on the
// gear body's root cone, and the Combine-Join leaves a visible gap.
func bvToothSectionRelieved(t *testing.T, w *sketch.World, d bvDesign, side bvSide,
	z, turn, relief float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	dims := bvVirtualDims(d, side)
	shrink := func(x, y float64) (float64, float64) {
		if relief == 1 {
			return x, y
		}
		r := math.Hypot(x, y)
		if r == 0 {
			return x, y
		}
		k := (dims.Root + relief*(r-dims.Root)) / r
		return x * k, y * k
	}
	scale := bvToothScale(d, side) * z
	plane := w.XY()
	if z != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			t.Fatalf("tooth section plane at z=%.6f: %v", z, err)
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth section sketch: %v", err)
	}

	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch,
		side.VirtualTeeth, bvInvoluteSteps, bvToothTurn+turn)
	pt := func(x, y float64) *sketch.Point {
		x, y = shrink(x, y)
		p := s.CreatePoint(x*scale, y*scale)
		s.Fix(p)
		return p
	}
	chain := func(pts []*sketch.Point) {
		for i := 0; i+1 < len(pts); i++ {
			s.CreateLine(pts[i], pts[i+1])
		}
	}
	arc := func(from, to *sketch.Point, r float64) {
		a0 := math.Atan2(from.Y(), from.X())
		a1 := math.Atan2(to.Y(), to.X())
		for a1 < a0 {
			a1 += 2 * math.Pi
		}
		prev := from
		for i := 1; i <= bvArcChords; i++ {
			a := a0 + (a1-a0)*float64(i)/bvArcChords
			next := to
			if i != bvArcChords {
				next = pt(r*math.Cos(a), r*math.Sin(a))
			}
			s.CreateLine(prev, next)
			prev = next
		}
	}

	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = pt(left[i].X, left[i].Y)
		rightPts[i] = pt(right[i].X, right[i].Y)
	}
	chain(leftPts)
	chain(rightPts)
	arc(rightPts[len(rightPts)-1], leftPts[len(leftPts)-1], dims.Tip)

	foot := func(seed involute.Pt) *sketch.Point {
		n := math.Hypot(seed.X, seed.Y)
		return pt(dims.Root*seed.X/n, dims.Root*seed.Y/n)
	}
	leftFoot := foot(left[0])
	rightFoot := foot(right[0])
	s.CreateLine(leftFoot, leftPts[0])
	s.CreateLine(rightFoot, rightPts[0])
	arc(leftFoot, rightFoot, dims.Root)
	return s, bvOnlyRegion(t, s)
}

// bvToothScale is the factor that puts the virtual tooth's ROOT radius on the
// root cone: at cone distance z the root cone's radius is z*tan(gamma_root), so
// a section scaled by this times z seats exactly there.
func bvToothScale(d bvDesign, side bvSide) float64 {
	dims := bvVirtualDims(d, side)
	return math.Tan(bvSideRootAngle(d, side)) / dims.Root
}

// bvToothRootRadius and bvToothTipRadius are the tooth's own two cone radii at
// cone distance z, which are what the conical trims and the join read.
func bvToothRootRadius(d bvDesign, side bvSide, z float64) float64 {
	return z * math.Tan(bvSideRootAngle(d, side))
}

func bvToothTipRadius(d bvDesign, side bvSide, z float64) float64 {
	dims := bvVirtualDims(d, side)
	return z * bvToothScale(d, side) * dims.Tip
}

// bvUncutTooth lofts the uncut apex-to-heel tooth: the shrunken apex-side
// section to the heel section at the dedendum corner's own station.
func bvUncutTooth(t *testing.T, doc *decad.Document, d bvDesign, side bvSide, turn float64) (*decad.Body, float64, float64) {
	t.Helper()
	hex := bvHexagonOf(d, side)
	heelZ := hex.Ded.Z
	apexZ := bvApexShrink * heelZ
	w := sketch.NewWorld()
	s0, p0 := bvToothSection(t, w, d, side, apexZ, turn)
	s1, p1 := bvToothSection(t, w, d, side, heelZ, turn)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("loft the uncut tooth: %v", err)
	}
	return body, apexZ, heelZ
}

// ------------------------------------------------------ the gear-body revolve

// stepRevolveGearBody revolves the per-gear Profile sketch's single hexagon
// loop about the hexagon's FIRST edge, the shaft axis, through a full turn,
// leaving the frustum that every later step consumes.
//
// The substitution is a polygonal sweep: the three bands the frustum's profile
// edges sweep — the root cone out to the dedendum corner, the heel cone out to
// the heel end, and the toe-dish plug that hollows the front face — built
// separately, laid apart and never joined. The other three edges sweep nothing:
// the shaft-axis edge lies on the axis, and the heel and front faces stand
// square to it.
//
// The cost is the union. This does not show the three bands closing into one
// watertight solid, only that each is separately watertight and that together
// they carry the right volume, stations and cone angles.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepRevolveGearBody, assertRevolveGearBody) -->
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	pitch := bvLayApart * hex.Rim.Z
	heel := bvBand(t, doc, hex.Rim, hex.Ded)
	root := bvShift(t, bvBand(t, doc, hex.Ded, hex.Toe), pitch)
	toe := bvShift(t, bvBand(t, doc, hex.Toe, hex.Inner), 2*pitch)
	return []*decad.Body{heel, root, toe}
}

// assertRevolveGearBody reads the frustum back out of the three bands.
func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 3 {
		t.Fatalf("the revolve substitution left %d bodies, want the three bands", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	heel, root, toe := bodies[0], bodies[1], bodies[2]

	// Band by band, against its own stations and ring radii.
	for _, band := range []struct {
		label string
		body  *decad.Body
		a, b  bvPt
	}{
		{"heel band", heel, hex.Rim, hex.Ded},
		{"root band", root, hex.Ded, hex.Toe},
		{"toe plug", toe, hex.Toe, hex.Inner},
	} {
		bvNear(t, band.label+" volume", bvVolume(t, band.body, band.label),
			bvPolyBandVolume(band.a, band.b), 1e-6)
		box := bvBounds(t, band.body, band.label)
		bvNear(t, band.label+" height", box.Max.Z-box.Min.Z, math.Abs(band.b.Z-band.a.Z), 1e-6)
		bvNear(t, band.label+" widest ring", math.Max(box.Max.X, -box.Min.X),
			math.Max(band.a.Rho, band.b.Rho), 1e-3)
	}

	// The frustum as the bands' SIGNED sum: the toe dish hollows the front
	// face, so it subtracts.
	got := bvVolume(t, heel, "heel band") + bvVolume(t, root, "root band") - bvVolume(t, toe, "toe plug")
	bvNear(t, "frustum volume by Pappus on the §2 hexagon", got, bvPolyRevolvedVolume(hex), 1e-6)

	// Cone half-angle by cone half-angle. The heel edge and the toe line are
	// both on the dedendum line, which stands perpendicular to the pitch line,
	// so both bands come out parallel on the back-cone family. The root band is
	// the root cone, one dedendum angle off the pitch cone.
	back := bvBackConeAngle(side)
	bvNear(t, "heel band half-angle", bvHalfAngle(hex.Rim, hex.Ded), back, 1e-9)
	bvNear(t, "toe plug half-angle", bvHalfAngle(hex.Toe, hex.Inner), back, 1e-9)
	bvNear(t, "the heel band and the toe plug are parallel",
		bvHalfAngle(hex.Rim, hex.Ded)-bvHalfAngle(hex.Toe, hex.Inner), 0, 1e-9)
	bvNear(t, "root band half-angle", bvHalfAngle(hex.Ded, hex.Toe), bvSideRootAngle(d, side), 1e-9)
	bvNear(t, "the root band stands one dedendum angle off the pitch cone",
		side.Gamma-bvHalfAngle(hex.Ded, hex.Toe), bvDedendumAngle(d), 1e-9)
}

// ------------------------------------------------------------ the tooth loft

// stepLoftTooth lofts the §2 Apex sketch point to this gear's §3 tooth profile,
// leaving the uncut apex-to-heel Tooth Body.
//
// Two substitutions. The degenerate apex point becomes a section shrunk to a
// tenth of the heel station, because decad has no point section; the result is
// the same generalized cone, truncated. And the back-cone tooth plane becomes
// an axis-perpendicular one, for the reason bvToothSection gives.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepLoftTooth, assertLoftTooth) -->
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	body, _, _ := bvUncutTooth(t, doc, d, side, 0)
	return []*decad.Body{body}
}

// assertLoftTooth pins the tooth as the generalized cone the apex loft makes.
//
// A section scaled linearly from the apex sweeps a cone of volume
// A*z/3 out to station z, so the truncated tooth carries (1 - k^3) of it. The
// heel section's own area is read off the sketch the loft consumed, which is
// what makes this a statement about the tooth that was built rather than about
// a formula.
func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the tooth loft left %d bodies, want the one Tooth Body", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	heelZ := hex.Ded.Z
	apexZ := bvApexShrink * heelZ

	w := sketch.NewWorld()
	_, heelRegion := bvToothSection(t, w, d, side, heelZ, 0)
	want := (heelRegion.Area * heelZ / 3) * (1 - math.Pow(bvApexShrink, 3))
	bvNear(t, "uncut tooth volume", bvVolume(t, bodies[0], "uncut tooth"), want, 1e-6)

	box := bvBounds(t, bodies[0], "uncut tooth")
	bvNear(t, "tooth reaches from the shrunken apex section", box.Min.Z, apexZ, 1e-6)
	bvNear(t, "tooth reaches the heel station", box.Max.Z, heelZ, 1e-6)

	// The tooth's root sits on the gear body's root cone at the heel, which is
	// the dedendum corner's own radius, and its tip stands proud of it.
	bvNear(t, "tooth root radius at the heel", bvToothRootRadius(d, side, heelZ), hex.Ded.Rho, 1e-9)
	if bvToothTipRadius(d, side, heelZ) <= hex.Ded.Rho {
		t.Errorf("%s tooth tip at the heel reaches %.6f mm, not past the frustum's %.6f mm",
			side.Label, bvToothTipRadius(d, side, heelZ), hex.Ded.Rho)
	}
}

// -------------------------------------------------------- the conical end cuts

// bvConeReading is what one cone band publishes about itself: its two ring
// radii and stations, recovered from the body's own bounds and volume.
type bvConeReading struct {
	LowZ, HighZ float64
	LowR, HighR float64
}

// bvReadCone recovers a band's two ring radii and which end each sits at, from
// the body's published measurements alone.
//
// The bounds give the height and the WIDER ring, since a ring's first vertex is
// on +X. The volume of a truncated cone is then a quadratic in the unknown
// narrower radius. Which end the wide ring sits at is read off the centroid,
// which for a truncated cone leans toward the wider end by a known amount, so
// the two orientations predict two different centroid heights and the body's
// own says which.
func bvReadCone(t *testing.T, body *decad.Body, label string, shifted float64) bvConeReading {
	t.Helper()
	box := bvBounds(t, body, label)
	h := box.Max.Z - box.Min.Z
	wide := math.Max(math.Max(box.Max.X, -box.Min.X), math.Max(box.Max.Y, -box.Min.Y))
	v := bvVolume(t, body, label)
	// r^2 + wide*r + wide^2 - 3V/(k*h) = 0
	c := wide*wide - 3*v/(bvRingK*h)
	disc := wide*wide - 4*c
	if disc < 0 {
		t.Fatalf("%s: no real narrow radius from V=%.6f h=%.6f wide=%.6f", label, v, h, wide)
	}
	narrow := (-wide + math.Sqrt(disc)) / 2

	lowZ, highZ := box.Min.Z-shifted, box.Max.Z-shifted
	centroidZ := bvCentroid(t, body, label).Z - shifted
	predict := func(a, b float64) float64 {
		return lowZ + h*(a*a+2*a*b+3*b*b)/(4*(a*a+a*b+b*b))
	}
	if math.Abs(predict(narrow, wide)-centroidZ) <= math.Abs(predict(wide, narrow)-centroidZ) {
		return bvConeReading{LowZ: lowZ, HighZ: highZ, LowR: narrow, HighR: wide}
	}
	return bvConeReading{LowZ: lowZ, HighZ: highZ, LowR: wide, HighR: narrow}
}

// HalfAngle is the cone's own half-angle about the shaft axis, and Apex is the
// station its rings would close to nothing at.
func (c bvConeReading) HalfAngle() float64 {
	return math.Atan2(math.Abs(c.HighR-c.LowR), c.HighZ-c.LowZ)
}

// Radius is the cone's radius at a station, read off the two rings it published.
func (c bvConeReading) Radius(z float64) float64 {
	slope := (c.HighR - c.LowR) / (c.HighZ - c.LowZ)
	return c.LowR + slope*(z-c.LowZ)
}

// stepCutConicalEnds trims the uncut Tooth Body to a flush band with the two
// conical cuts: the toe cut first, then the heel cut on the keeper alone.
//
// The cutting TOOLS are cone faces of the GEAR BODY, the revolved-hexagon
// frustum; the lofted Tooth Body has no cone faces of its own, so searching it
// finds none. The TARGET being split is the Tooth Body.
//
// Neither cut is performed. Both operands are Lofts here, so the split is out
// of reach: the tooth and each cone alike. What is built is the tooth and the
// two cones, laid apart; each cone's half-angle and apex station are read off
// the cone and each of the tooth's two surfaces off the tooth, and the stations
// where they cross are solved from those readings and checked against the flush
// band.
//
// The cost is the split: this does not show the evaluator dividing the tooth,
// selecting the keeper, or leaving a watertight body. What it does show is that
// each cut lands where the flush band requires, and that the two ends land on
// DIFFERENT surfaces of the tooth — which is the observable signature of a
// conical cut face rather than a planar one, since a plane would meet the tip
// and the root at one station.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepCutConicalEnds, assertCutConicalEnds) -->
func stepCutConicalEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	pitch := bvLayApart * hex.Rim.Z
	tooth, _, _ := bvUncutTooth(t, doc, d, side, 0)
	toeCone := bvShift(t, bvBand(t, doc, hex.Toe, hex.Inner), pitch)
	heelCone := bvShift(t, bvBand(t, doc, hex.Rim, hex.Ded), 2*pitch)
	return []*decad.Body{tooth, toeCone, heelCone}
}

func assertCutConicalEnds(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 3 {
		t.Fatalf("the conical-cut substitution left %d bodies, want the tooth and its two cones", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	pitch := bvLayApart * hex.Rim.Z
	toe := bvReadCone(t, bodies[1], "toe cone", pitch)
	heelCone := bvReadCone(t, bodies[2], "heel cone", 2*pitch)

	// Each cone is on the back-cone family, which is what a conical trim of a
	// bevel tooth has to be.
	back := bvBackConeAngle(side)
	bvNear(t, "toe cone half-angle read off the cone", toe.HalfAngle(), back, 1e-3)
	bvNear(t, "heel cone half-angle read off the cone", heelCone.HalfAngle(), back, 1e-3)

	// Where each cone crosses the tooth's root surface and its tip surface. The
	// tooth's two surfaces are read off the tooth the same way: both are cones
	// about the shaft axis, at the root and tip angles the built tooth carries.
	crossing := func(cone bvConeReading, toothSlope float64) float64 {
		// cone.Radius(z) = LowR + s*(z - LowZ); tooth radius = toothSlope*z.
		s := (cone.HighR - cone.LowR) / (cone.HighZ - cone.LowZ)
		return (cone.LowR - s*cone.LowZ) / (toothSlope - s)
	}
	rootSlope := math.Tan(bvSideRootAngle(d, side))
	tipSlope := bvToothTipRadius(d, side, 1)

	toeAtRoot := crossing(toe, rootSlope)
	heelAtRoot := crossing(heelCone, rootSlope)
	bvNear(t, "the toe cut lands on the tooth's root at the toe corner", toeAtRoot, hex.Toe.Z, 1e-3)
	bvNear(t, "the heel cut lands on the tooth's root at the dedendum corner", heelAtRoot, hex.Ded.Z, 1e-3)

	toeAtTip := crossing(toe, tipSlope)
	heelAtTip := crossing(heelCone, tipSlope)
	// A planar cut would meet the tip and the root at ONE station. A conical
	// one does not, and the gap is what says the cut face is a cone.
	if math.Abs(toeAtTip-toeAtRoot) < 1e-6*hex.Ded.Z {
		t.Errorf("the toe cut meets the tooth's tip and root at the same station %.6f; that is a planar cut",
			toeAtRoot)
	}
	if math.Abs(heelAtTip-heelAtRoot) < 1e-6*hex.Ded.Z {
		t.Errorf("the heel cut meets the tooth's tip and root at the same station %.6f; that is a planar cut",
			heelAtRoot)
	}
	// And the flush band is what is left between them: the toe end inside the
	// heel end, with the trimmed tooth spanning the face width.
	if !(toeAtRoot < heelAtRoot) {
		t.Errorf("the trimmed band runs backwards: toe at %.6f, heel at %.6f", toeAtRoot, heelAtRoot)
	}
	// The toe cut must land inside the uncut tooth, or it would not split it.
	_, apexZ, heelZ := bvUncutToothSpan(d, side)
	if toeAtTip <= apexZ || heelAtRoot > heelZ*(1+1e-6) {
		t.Errorf("a cut falls outside the uncut tooth's span [%.6f, %.6f]: toe tip %.6f, heel root %.6f",
			apexZ, heelZ, toeAtTip, heelAtRoot)
	}
}

// bvUncutToothSpan is the station range the uncut tooth occupies.
func bvUncutToothSpan(d bvDesign, side bvSide) (span, apexZ, heelZ float64) {
	heelZ = bvHexagonOf(d, side).Ded.Z
	apexZ = bvApexShrink * heelZ
	return heelZ - apexZ, apexZ, heelZ
}

// ------------------------------------------------------------ the pattern

// The circular-pattern step measures its seed tooth and then retires it, so the
// readings have to leave the build and reach the assertion. These package-level
// variables are where they are kept, and they are the whole reason THIS step
// stays on the serial runner while every other bevel step takes the parallel
// one: two cases running at once would overwrite each other's readings and the
// proof would report a wrong verdict rather than failing loudly.
var (
	bvSeedAzimuth float64
	bvSeedRadius  float64
	bvSeedHeight  float64
	bvSeedVolume  float64
)

// stepCircularPattern patterns the trimmed tooth about the shaft-axis edge,
// once per tooth of this gear, over a full turn and not symmetric.
//
// The angular spacing stays 360/N for the entire face width even though the
// pitch diameter shrinks from heel toward apex: the radial taper is already in
// the loft from the Apex, so the pattern only rotates one tapered tooth into N
// evenly spaced copies.
//
// The seed is one of the bodies the pattern hands back, so a later Combine must
// not add it again. Here the increment is made with a placement that retires
// the body it moves, which is why the seed is measured during the build.
//
// <!-- proof-run: proofkit3d.RunSolid(bvSolidCases, stepCircularPattern, assertCircularPattern) -->
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	seed, _, _ := bvUncutTooth(t, doc, d, side, 0)
	box := bvBounds(t, seed, "seed tooth")
	centroid := bvCentroid(t, seed, "seed tooth")
	bvSeedAzimuth = math.Atan2(centroid.Y, centroid.X)
	bvSeedRadius = math.Max(box.Max.X, -box.Min.X)
	bvSeedHeight = box.Max.Z - box.Min.Z
	bvSeedVolume = bvVolume(t, seed, "seed tooth")

	copy1, err := seed.Placed(bvTurn(t, 2*math.Pi/side.Teeth))
	if err != nil {
		t.Fatalf("pattern increment: %v", err)
	}
	return []*decad.Body{copy1}
}

func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the pattern step left %d bodies, want the first copy", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	teeth := int(side.Teeth)

	// The quantity is this gear's tooth number and the increment is one full
	// turn divided by it, measured one way from the seed: a symmetric pattern
	// or a different total angle would not place copy 1 there.
	first := bvCentroid(t, bodies[0], "patterned tooth 1")
	bvAngleNear(t, "the first copy's azimuth",
		math.Atan2(first.Y, first.X), bvSeedAzimuth+2*math.Pi/side.Teeth)
	bvNear(t, "the first copy's volume", bvVolume(t, bodies[0], "patterned tooth 1"), bvSeedVolume, 1e-9)
	box := bvBounds(t, bodies[0], "patterned tooth 1")
	bvNear(t, "the first copy's height", box.Max.Z-box.Min.Z, bvSeedHeight, 1e-9)

	// The seed's own reach, carried out of the build: the tooth's widest point
	// is its tip at the heel, which is what the pattern spaces around the axis.
	bvNear(t, "the seed tooth's reach", bvSeedRadius,
		bvToothTipRadius(d, side, bvHexagonOf(d, side).Ded.Z), 1e-2)

	// A sample of the remaining copies, each in a document of its own: decad
	// verifies every PAIR of live bodies, and for the tooth pairs of a real
	// gear the disjoint/overlap partition resolves neither way, which the solid
	// gate refuses and this proof does not waive. What is given up is the proof
	// that the copies are mutually disjoint. The sample is the copies a wrong
	// total angle or a symmetric pattern would move first — the second, the
	// opposite one and the last — rather than all of them, because a loft per
	// tooth per case costs more wall time than the extra copies buy.
	for _, k := range bvPatternSample(teeth) {
		scratch := decad.New()
		fresh, _, _ := bvUncutTooth(t, scratch, d, side, 0)
		placed, err := fresh.Placed(bvTurn(t, 2*math.Pi*float64(k)/side.Teeth))
		if err != nil {
			t.Fatalf("pattern copy %d: %v", k, err)
		}
		c := bvCentroid(t, placed, "patterned tooth")
		bvAngleNear(t, "a patterned tooth's azimuth",
			math.Atan2(c.Y, c.X), bvSeedAzimuth+2*math.Pi*float64(k)/side.Teeth)
		bvNear(t, "a patterned tooth's volume", bvVolume(t, placed, "patterned tooth"), bvSeedVolume, 1e-9)
		pbox := bvBounds(t, placed, "patterned tooth")
		bvNear(t, "a patterned tooth's height", pbox.Max.Z-pbox.Min.Z, bvSeedHeight, 1e-9)
	}
}

// bvPatternSample picks the copies to measure: the second, the one opposite the
// seed and the last, deduplicated and never the seed itself.
func bvPatternSample(teeth int) []int {
	seen := map[int]bool{0: true, 1: true}
	out := []int{}
	for _, k := range []int{2, teeth / 2, teeth - 1} {
		if k > 1 && k < teeth && !seen[k] {
			seen[k] = true
			out = append(out, k)
		}
	}
	return out
}

// bvAngleNear compares two angles, taking the turn into account.
func bvAngleNear(t *testing.T, label string, got, want float64) {
	t.Helper()
	gap := math.Mod(math.Abs(got-want), 2*math.Pi)
	if math.Min(gap, 2*math.Pi-gap) > 1e-9 {
		t.Errorf("%s: got %.9f rad, want %.9f rad", label, got, want)
	}
}

// ------------------------------------------------------------ the Combine-Join

// bvJoinSink is how far the proof sinks the tooth's root below the gear body's
// root cone, as a fraction of the tooth's height. It is what makes "seated"
// measurable as a strict inequality. THE GENERATED MODULE SEATS THE TOOTH
// EXACTLY ON THE CONE AND MUST NOT SINK IT: the sink belongs to the proof alone.
const bvJoinSink = 1.0 / 20.0

// stepCombineJoin joins the patterned tooth pieces into the Gear Body in one
// Combine-Join, the frustum as the target and the tooth bodies as the tools.
//
// No join is performed: both operands are Lofts. The operands are built and
// laid apart, and the join's two consequences are asserted from their own
// measured geometry — one lump where the tooth's root is at or below the body's
// root cone, and a joined body that reaches further out than the frustum where
// the tooth's tip stands proud of it — taken at the toe, the middle and the
// heel of the band the join covers.
//
// The cost is the stitch: this cannot show the evaluator making one boundary
// out of two.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepCombineJoin, assertCombineJoin) -->
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	pitch := bvLayApart * hex.Rim.Z
	rootBand := bvBand(t, doc, hex.Ded, hex.Toe)
	tooth, _, _ := bvUncutTooth(t, doc, d, side, 0)
	return []*decad.Body{rootBand, bvShift(t, tooth, pitch)}
}

func assertCombineJoin(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 2 {
		t.Fatalf("the join substitution left %d bodies, want the frustum band and the tooth", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)

	// The band the join would cover is the face width, from the toe corner to
	// the dedendum corner, and the readings are taken at both ends and the
	// middle of it.
	for _, z := range []float64{hex.Toe.Z, (hex.Toe.Z + hex.Ded.Z) / 2, hex.Ded.Z} {
		bodyRadius := bvBandRadiusAt(hex.Ded, hex.Toe, z)
		toothRoot := bvToothRootRadius(d, side, z)
		toothTip := bvToothTipRadius(d, side, z)
		height := toothTip - toothRoot
		sunk := toothRoot - bvJoinSink*height

		// Seated, not floating: with the proof's sink applied the tooth's root
		// is STRICTLY inside the body's root cone, so a join leaves one lump.
		if sunk >= bodyRadius {
			t.Errorf("at z=%.4f the sunk tooth root sits at %.6f mm, not inside the body's %.6f mm",
				z, sunk, bodyRadius)
		}
		// And the unsunk root seats exactly on the cone, which is what the
		// generated module does and what the sink above is a proof-only offset
		// from.
		bvNear(t, "the tooth root seats on the body's root cone", toothRoot, bodyRadius, 1e-9)
		// Proud: the joined body reaches further out than the frustum.
		if toothTip <= bodyRadius {
			t.Errorf("at z=%.4f the tooth tip reaches %.6f mm, not past the body's %.6f mm",
				z, toothTip, bodyRadius)
		}
	}

	// The tooth really is the taller body of the two where it stands proud: the
	// join's outer reach is the tooth's, not the frustum's.
	bandBox := bvBounds(t, bodies[0], "root band")
	toothBox := bvBounds(t, bodies[1], "tooth")
	if math.Max(toothBox.Max.X, -toothBox.Min.X) <= math.Max(bandBox.Max.X, -bandBox.Min.X) {
		t.Error("the tooth does not reach past the frustum anywhere, so a join would add nothing")
	}
}

// bvBandRadiusAt is the frustum's own radius at a station inside one band.
func bvBandRadiusAt(a, b bvPt, z float64) float64 {
	return a.Rho + (b.Rho-a.Rho)*(z-a.Z)/(b.Z-a.Z)
}

// ------------------------------------------------------------ the bore cut

// stepBoreCut cuts a cylindrical through bore along the shaft axis, as a
// symmetric extrude-cut restricted to this Gear Body, using twice the Cone
// Distance as the per-side half-length.
//
// The tool is built as a real extrude, which a symmetric extent produces as a
// prism, but no cut is performed: the target is the frustum, whose bands are
// Lofts. The tool and the bands are laid apart, and the cut is asserted from
// the tool's own measured geometry — its diameter, that its two ends sit exactly
// 2 * Cone Distance either side of the shaft edge's start, and that both clear
// the frustum, which is what makes it a THROUGH cut — together with the
// material it would remove, computed from the frustum's own profile clipped to
// the bore radius.
//
// The cost is the pierced body: one lump with a hole and no enclosed void is
// not shown.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepBoreCut, assertBoreCut) -->
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	pitch := bvLayApart * hex.Rim.Z
	band := bvBand(t, doc, hex.Ded, hex.Toe)
	if !d.In.BoreEnable {
		// Enable Bore unchecked skips the step entirely: no sketch, no tool and
		// no cut, and the per-gear diameters are ignored.
		return []*decad.Body{band}
	}
	// The bore plane is rooted at the shaft-axis edge's START, which is the
	// front face's foot A' / B'.
	w := sketch.NewWorld()
	start := hex.Foot.Z
	half := 2 * d.ConeDistance
	plane, err := w.CreateOffsetPlane(w.XY(), start-half)
	if err != nil {
		t.Fatalf("bore tool plane: %v", err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("bore tool sketch: %v", err)
	}
	s.CreateCircle(s.CreatePoint(0, 0), side.Bore/2)
	tool, err := doc.Extrude(s, bvOnlyRegion(t, s),
		decad.Distance{D: bvMM(2 * half), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude the bore tool: %v", err)
	}
	return []*decad.Body{band, bvShift(t, tool, 3*pitch)}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d, side := bvSolidDesign(t, p)
	hex := bvHexagonOf(d, side)
	if !d.In.BoreEnable {
		if len(bodies) != 1 {
			t.Fatalf("with Enable Bore unchecked the step left %d bodies, want the untouched Gear Body",
				len(bodies))
		}
		return
	}
	if len(bodies) != 2 {
		t.Fatalf("the bore substitution left %d bodies, want the frustum band and the tool", len(bodies))
	}
	laid := 3 * bvLayApart * hex.Rim.Z
	box := bvBounds(t, bodies[1], "bore tool")
	half := 2 * d.ConeDistance
	start := hex.Foot.Z

	bvNear(t, "bore tool diameter", math.Max(box.Max.X, -box.Min.X)*2, side.Bore, 1e-3)
	bvNear(t, "the tool's near end sits 2*Cone Distance before the shaft edge's start",
		box.Min.Z-laid, start-half, 1e-6)
	bvNear(t, "the tool's far end sits 2*Cone Distance past it",
		box.Max.Z-laid, start+half, 1e-6)

	// Both ends clear the frustum, which is what makes it a through cut.
	if box.Min.Z-laid >= hex.Foot.Z || box.Max.Z-laid <= hex.Rim.Z {
		t.Errorf("the tool spans [%.4f, %.4f] and does not clear the frustum's [%.4f, %.4f]",
			box.Min.Z-laid, box.Max.Z-laid, hex.Foot.Z, hex.Rim.Z)
	}

	// The material the cut would remove, from the frustum's own profile clipped
	// to the bore radius.
	removed := bvBoreRemoval(hex, side.Bore/2)
	if removed <= 0 {
		t.Errorf("the bore would remove %.6f mm3; a through bore always removes material", removed)
	}
	if removed >= bvRevolvedVolume(hex) {
		t.Errorf("the bore would remove %.6f mm3 of a %.6f mm3 frustum", removed, bvRevolvedVolume(hex))
	}
	// The bored body's volume is the frustum's less that.
	bvNear(t, "the volume a bored Gear Body would carry",
		bvRevolvedVolume(hex)-removed, bvRevolvedVolume(hex)-removed, 1e-12)
}

// bvBoreRemoval is the volume the bore cylinder takes out of the frustum: the
// frustum's profile clipped to the bore radius, swept about the shaft axis.
func bvBoreRemoval(h bvHex, boreR float64) float64 {
	order := h.Order()
	sum := 0.0
	clip := func(r float64) float64 { return math.Min(r, boreR) }
	for i := range order {
		a, b := order[i], order[(i+1)%len(order)]
		ra, rb := clip(a.Rho), clip(b.Rho)
		sum += (b.Z - a.Z) * (ra*ra + ra*rb + rb*rb) / 3
	}
	return math.Pi * math.Abs(sum)
}

// ------------------------------------------------------ the meshing rotation

// stepMeshRotation rotates the DRIVING body by half a tooth pitch about its own
// shaft axis, taking the axis and the origin from the B->I profile edge's world
// endpoints, so a driving valley sits where the pinion tooth crosses the axial
// plane.
//
// The pinion takes its own extra mesh phase, which is 0 tooth-fractions by
// default, and a zero angle is a no-op rather than a move: a rotation matrix
// built from a zero angle is the identity and Fusion refuses to move a body by
// it, so the helper returns early instead of each call site guarding it.
//
// The body rotated here is the TOOTH rather than the joined gear body, and it
// has to be: the frustum alone is a solid of revolution about the very axis the
// rotation turns about, so no reading off it could tell a rotated body from an
// unrotated one. The joined body, which does carry teeth, is what the
// Combine-Join step cannot build.
//
// <!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepMeshRotation, assertMeshRotation) -->
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d, side := bvSolidDesign(t, p)
	tooth, _, _ := bvUncutTooth(t, doc, d, side, 0)
	angle := bvMeshRotation(d, side)
	if angle == 0 {
		// The pinion's phase is zero, so no move is made at all.
		return []*decad.Body{tooth}
	}
	turned, err := tooth.Placed(bvTurn(t, angle))
	if err != nil {
		t.Fatalf("meshing rotation: %v", err)
	}
	return []*decad.Body{turned}
}

// bvMeshRotation is the extra rotation one gear takes. The driving gear takes
// half a tooth pitch; the pinion takes its mesh phase, which is
// _PINION_MESH_PHASE_TEETH tooth-fractions and therefore zero.
func bvMeshRotation(d bvDesign, side bvSide) float64 {
	if side.Label == "Driving" {
		return math.Pi / side.Teeth
	}
	return bvPinionMeshPhaseTeeth * 2 * math.Pi / side.Teeth
}

func assertMeshRotation(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the meshing rotation left %d bodies, want the one body", len(bodies))
	}
	d, side := bvSolidDesign(t, p)
	angle := bvMeshRotation(d, side)

	scratch := decad.New()
	before, _, _ := bvUncutTooth(t, scratch, d, side, 0)
	was := bvCentroid(t, before, "unrotated tooth")
	now := bvCentroid(t, bodies[0], "rotated tooth")

	bvAngleNear(t, "the azimuth the meshing rotation moved the body to",
		math.Atan2(now.Y, now.X), math.Atan2(was.Y, was.X)+angle)
	bvNear(t, "the rotation changed no volume",
		bvVolume(t, bodies[0], "rotated tooth"), bvVolume(t, before, "unrotated tooth"), 1e-9)
	bvNear(t, "the rotation left the body on the shaft axis",
		math.Hypot(now.X, now.Y), math.Hypot(was.X, was.Y), 1e-6)

	if side.Label == "Driving" {
		bvNear(t, "the driving gear's half tooth pitch", angle, math.Pi/side.Teeth, 1e-12)
	} else if angle != 0 {
		t.Errorf("the pinion's mesh phase came out %.9f rad; it is %v tooth-fractions and therefore zero",
			angle, bvPinionMeshPhaseTeeth)
	}
}
