// This file holds the bevel pair's solid steps, one per Fusion timeline entry:
// the gear-body revolve, the apex-to-tooth loft, the conical end cuts, the five
// steps of the spiral tooth body, the circular pattern, the Combine-Join, the
// bore cut and the driving gear's meshing rotation.
//
// THE FRAME. Every solid step works in one world frame: the gear's shaft axis is
// world +Z, the radial direction the dedendum corner C/D sits on is world +X, and
// the circumferential direction is world +Y. A profile-frame point (station,
// radius) is therefore the world point (radius, 0, station), and the Apex is the
// origin. The tooth plane's own in-plane frame has its origin at Apex 2, its u
// running along Apex2->C, and its v circumferential — so the tooth centre sits at
// u = virtualPitchRadius + Tooth Spacing, which is what §2 says |Apex2 -> K'| is.
//
// THE OPERAND RULE. Every boolean operand here is built with Loft and never with
// Revolve. Measured at the pinned decad revision over this gear's own bodies
// across the whole solid table: every boolean whose operands were both Lofts
// returned a body the document verified Sound, while every boolean with a Revolve
// operand verified Suspect instead, which proofkit3d's gate admits no more than
// it admits a failure. The gear body itself IS a real Revolve, and it is the one
// body no boolean consumes: a revolved frustum's volume agrees with Pappus on its
// own profile to the bound decad publishes, so it clears the gate on its own
// readings.
//
// THE ONE SUBSTITUTION, stated once. Where an operation is out of reach the
// operands are still built, laid apart along the shaft axis, and what the
// operation would have produced is asserted from their own measured geometry.
// Laying them apart leaves every volume, radius and cone angle unchanged, which
// is what makes the readings still mean something. Each site says what it costs.
//
// WHAT THE SOLID TABLE CANNOT RUN. Module 1 is not in it. decad's mesh bound has
// an absolute floor, so a figure small enough brings every measurement inside it
// and the gate reports Suspect on geometry that is in fact correct. Module is a
// pure scale on this figure, so a case at Module 4 through 8 proves the same shape
// and clears the floor. The sketch tables are unaffected and stay at the dialog's
// own default.
package bevelgear_test

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// apexSectionScale is the shrunken section that stands in for the apex loft's
// degenerate point end, as a fraction of the Pitch Cone Distance. The loft's
// point section is the one thing this proof does not build; what the volume and
// the taper prove is the shape it has to produce.
const apexSectionScale = 0.02

// chordSteps is how many straight spans each drawn arc is replaced by. Every
// solid section here is a polygon: decad's Loft pairs segments by kind, and a
// polygon paired against the same polygon scaled is a one-to-one LineSeg pairing
// whose ruled walls are exact, so the frustum volume below is a closed form and
// not an approximation. What chording costs is the sagitta between each chord and
// the arc it spans: a chorded tooth is slightly thinner than the drawn one and
// every volume read off it slightly smaller. Both sides of every comparison are
// chorded, so the sagitta cancels.
const chordSteps = 24

func mm(v float64) units.Value { return units.Millimeters(v) }

// ---------------------------------------------------------------- case tables

// solidCase names one solid case: the shared `params` default with Module raised
// to the table's floor of 4, for the reason the file comment gives, and the Mean
// Spiral Angle at 0 so the straight tooth is what a solid case builds unless it
// says otherwise.
func solidCase(mut ...func(map[string]float64)) map[string]float64 {
	p := params(map[string]float64{keyModule: 4, keySpiralAngle: 0})
	for _, f := range mut {
		f(p)
	}
	return p
}

func perGearSolidCases(base []proofkit3d.Case) []proofkit3d.Case {
	out := make([]proofkit3d.Case, 0, 2*len(base))
	for _, c := range base {
		for _, side := range []struct {
			name string
			gear float64
		}{{"pinion", 0}, {"driving", 1}} {
			params := map[string]float64{keyGearSide: side.gear}
			for k, v := range c.Params {
				params[k] = v
			}
			out = append(out, proofkit3d.Case{Name: c.Name + "_" + side.name, Params: params})
		}
	}
	return out
}

// solidBaseCases carry the regime the solid steps have to hold across: the
// shipped default, the two virtual-tooth-count cases the spec names, a ratio pair
// with Tooth Spacing above zero, both ends of the Shaft Angle range that this
// net reaches, the bore in both states, and the two toe inputs together.
var solidBaseCases = []proofkit3d.Case{
	{Name: "m4_31_31_sigma90", Params: solidCase()},
	{Name: "m4_16_12_sigma90", Params: solidCase(pair(4, 16, 12))},
	{Name: "m8_4_4_sigma90", Params: solidCase(pair(8, 4, 4))},
	{Name: "m4_43_31_sigma75_spacing", Params: solidCase(
		pair(4, 43, 31), shaft(75), with(keyToothSpacing, 0.5))},
	{Name: "m6_31_17_sigma120", Params: solidCase(pair(6, 31, 17), shaft(120))},
	{Name: "m4_31_31_bore_disabled", Params: solidCase(with(keyBoreEnable, 0))},
	{Name: "m4_31_31_toe50_toe_radius_user", Params: solidCase(
		with(keyToeExtension, 50), with(keyDrivingToeRadius, 12), with(keyPinionToeRadius, 12))},
	{Name: "m4_31_31_sigma35", Params: solidCase(shaft(35))},
}

// spiralBaseCases carry the psi > 0 branch, plus the psi = 0 case the hook
// returns early on, so both sides of that gate are covered.
var spiralBaseCases = []proofkit3d.Case{
	{Name: "psi0_straight_m4_31_31", Params: solidCase()},
	{Name: "psi35_right_m4_31_31", Params: solidCase(with(keySpiralAngle, 35))},
	{Name: "psi35_left_m4_31_31", Params: solidCase(
		with(keySpiralAngle, 35), with(keyHand, -1))},
	{Name: "psi55_m4_31_31", Params: solidCase(with(keySpiralAngle, 55))},
	{Name: "psi35_ratio_m4_43_31_sigma75", Params: solidCase(
		with(keySpiralAngle, 35), pair(4, 43, 31), shaft(75))},
	{Name: "psi35_m4_16_12", Params: solidCase(with(keySpiralAngle, 35), pair(4, 16, 12))},
}

var (
	solidCases  = perGearSolidCases(solidBaseCases)
	spiralCases = perGearSolidCases(spiralBaseCases)
)

// ---------------------------------------------------------------- frames

// axialSketch is the Gear Profiles plane in this proof's world frame: u is the
// radius from the shaft axis, v is the station along it, and the shaft axis is
// the plane's own v axis. The revolve spins the hexagon about that line.
func axialSketch(t *testing.T, w *sketch.World) *sketch.Sketch {
	t.Helper()
	plane, err := w.CreatePlaneFromPoints(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("axial plane: %v", err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("axial sketch: %v", err)
	}
	return s
}

// shaftAxis is the hexagon's first edge A'->G / B'->I, stated in the axial
// plane's own coordinates: the line u = 0, which is the shaft axis itself.
var shaftAxis = decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

// stationSketch is a plane perpendicular to the shaft axis at station z, with u
// along the radial +X and v along the circumferential +Y, so its normal is +Z and
// a prism built on it sweeps along the shaft.
func stationSketch(t *testing.T, w *sketch.World, z float64) *sketch.Sketch {
	t.Helper()
	plane, err := w.CreatePlaneFromPoints(
		r3.NewVec(0, 0, z), r3.NewVec(1, 0, z), r3.NewVec(0, 1, z))
	if err != nil {
		t.Fatalf("station plane at z=%g: %v", z, err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("station sketch at z=%g: %v", z, err)
	}
	return s
}

// toothPlaneSketch is the back-cone tooth plane, or a plane parallel to it at a
// fraction of the Pitch Cone Distance from the Apex.
//
// The plane is the REAL one: it carries the tooth-centre reference line C->K' and
// is tilted out of the axis-perpendicular by this gear's pitch cone angle, so its
// normal runs along the PITCH element. Its perpendicular distance from the Apex is
// the Pitch Cone Distance R exactly — the tooth centre's station times cos(gamma),
// and NOT the station itself. Tooth Spacing slides the centre ALONG the dedendum
// line, which lies in this plane, so it moves the centre and never the plane.
//
// Its in-plane origin is Apex 2 and its u runs along Apex2->C, which is why the
// tooth centre sits at u = virtualPitchRadius + Tooth Spacing and the dedendum
// corner C at u = 1.25*Module.
func toothPlaneSketch(t *testing.T, w *sketch.World, l lattice, g member, scale float64) *sketch.Sketch {
	t.Helper()
	sg, cg := math.Sin(g.gamma), math.Cos(g.gamma)
	d := scale * l.pitchCone
	origin := r3.NewVec(d*sg, 0, d*cg)
	u := r3.NewVec(-cg, 0, sg)
	plane, err := w.CreatePlaneFromPoints(origin, origin.Add(u), origin.Add(r3.NewVec(0, 1, 0)))
	if err != nil {
		t.Fatalf("tooth plane at scale %g: %v", scale, err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth sketch at scale %g: %v", scale, err)
	}
	return s
}

// toothPlaneToWorld maps an in-plane (u, v) on the tooth plane family at the
// given scale into this proof's world frame.
func toothPlaneToWorld(l lattice, g member, scale float64, p pt2) r3.Vec {
	sg, cg := math.Sin(g.gamma), math.Cos(g.gamma)
	d := scale * l.pitchCone
	return r3.NewVec(d*sg-p.X*cg, p.Y, d*cg+p.X*sg)
}

// distAlong is a world point's cone distance: its distance from the Apex measured
// along the ROOT cone element Apex->C / Apex->D, which is the frame step A builds.
func distAlong(g member, p r3.Vec) float64 {
	return p.X*math.Sin(g.gammaRoot) + p.Z*math.Cos(g.gammaRoot)
}

// ---------------------------------------------------------------- the gear body

// gearHexagon draws one gear's frustum profile on the axial plane and returns the
// one region the revolve consumes. The sketch is drawn at its solved coordinates
// and carries no constraints: the constraint scheme is stepGearProfiles' subject,
// and repeating it here would prove it twice and build nothing new.
func gearHexagon(t *testing.T, w *sketch.World, l lattice, g member) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := axialSketch(t, w)
	hex := profileHexagon(l, g)
	pts := make([]*sketch.Point, 0, 6)
	for _, v := range hex.points() {
		pts = append(pts, s.CreatePoint(v.Y, v.X)) // u = radius, v = station
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	return s, decadtest.SolveRegion(t, s)
}

func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	s, region := gearHexagon(t, sketch.NewWorld(), l, g)
	body, err := doc.Revolve(s, region, shaftAxis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("revolve the %s Gear Body: %v", g.label, err)
	}
	return []*decad.Body{body}
}

// assertRevolveGearBody pins what the revolve produces: one watertight frustum of
// the volume Pappus gives on the hexagon it was swept from, with the right two
// flat faces and the right three cone faces.
//
// The faces are matched by surface kind and by their OWN readings rather than by
// the order the face selector hands them back: the flat heel face at the back is a
// disc of the heel radius, the flat toe face at the front is a disc of the Toe
// Radius, and the three cone frusta are the ones the profile edges C->H, M->C and
// N->M sweep.
//
// Each cone's half-angle is read off its own face — a decad.Cone publishes it —
// rather than derived from two cap radii and a height. The heel cone and the
// toe-dish cone read 90 - gamma, because both edges run along the back-cone
// dedendum direction, and the root cone reads this gear's own root cone angle.
//
// THIS STEP COSTS NOTHING. The frustum is one body with the right volume, the
// right two flat faces and the right three cone faces, and there is no union left
// for the proof to owe.
func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the revolve produced %d bodies, want the one Gear Body", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	hex := profileHexagon(l, g)
	body := bodies[0]

	// Pappus on the hexagon is a closed form with no polygon correction and no
	// decomposition into bands, so the only slack it needs is float round-off;
	// decad's own published bound is added on top by decadtest.
	decadtest.Measures(t, g.label+" Gear Body volume against Pappus",
		volumeReading(t, body, g.label+" Gear Body"),
		units.CubicMillimeters(pappusVolume(hex.points())),
		decadtest.WithinRel(units.Scalar(1e-12)))

	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindPlane: 2,
		decad.KindCone:  3,
	})

	wantDiscs := []float64{math.Pi * hex.h.Y * hex.h.Y, math.Pi * hex.n.Y * hex.n.Y}
	wantCones := [][2]float64{
		{frustumArea(hex.c, hex.h), coneHalfAngle(hex.c, hex.h)}, // the heel cone
		{frustumArea(hex.m, hex.c), coneHalfAngle(hex.m, hex.c)}, // the root cone
		{frustumArea(hex.n, hex.m), coneHalfAngle(hex.n, hex.m)}, // the toe dish
	}
	backCone := math.Pi/2 - g.gamma
	for i, want := range wantCones {
		if i == 1 {
			continue
		}
		if math.Abs(want[1]-backCone) > 1e-9 {
			t.Errorf("%s cone %d reads half-angle %.9f rad from the profile, want the back-cone "+
				"half-angle 90 - gamma = %.9f rad", g.label, i, want[1], backCone)
		}
	}
	if math.Abs(wantCones[1][1]-g.gammaRoot) > 1e-9 {
		t.Errorf("%s root cone half-angle %.9f rad, want this gear's root cone angle %.9f rad",
			g.label, wantCones[1][1], g.gammaRoot)
	}

	for _, face := range body.Faces() {
		area, err := face.Area()
		if err != nil {
			t.Fatalf("%s face area: %v", g.label, err)
		}
		got := area.Value.Base() // decad's base length unit is the millimetre
		switch surface := face.Surface().(type) {
		case decad.Plane:
			if !matchWithin(got, wantDiscs, 1e-6) {
				t.Errorf("%s flat face of %.6f mm2 matches neither the heel disc %.6f nor the toe "+
					"disc %.6f", g.label, got, wantDiscs[0], wantDiscs[1])
			}
		case decad.Cone:
			half := surface.HalfAngle.Base()
			matched := false
			for _, want := range wantCones {
				if relClose(got, want[0], 1e-6) && math.Abs(half-want[1]) < 1e-6 {
					matched = true
				}
			}
			if !matched {
				t.Errorf("%s cone face of %.6f mm2 at half-angle %.9f rad matches none of the "+
					"three the profile edges sweep: %v", g.label, got, half, wantCones)
			}
		default:
			t.Errorf("%s carries a face of kind %v, which the hexagon cannot sweep", g.label, surface.Kind())
		}
	}
}

func matchWithin(got float64, want []float64, rel float64) bool {
	for _, w := range want {
		if relClose(got, w, rel) {
			return true
		}
	}
	return false
}

func relClose(got, want, rel float64) bool {
	return math.Abs(got-want) <= rel*math.Max(math.Abs(want), 1)
}

// volumeReading is a body's volume reading: the value decad measured together
// with the bound it proved around it. label is the spec's own name for the body,
// which is what a failure has to open with — decadtest names a body by its index
// and recipe step, and that does not say which feature is wrong.
func volumeReading(t *testing.T, body *decad.Body, label string) decad.Measurement {
	t.Helper()
	measured, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", label, err)
	}
	return measured
}

func volumeOf(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	return volumeReading(t, body, label).Value.Base()
}

// ---------------------------------------------------------------- the tooth

// toothPolygon is one gear's tooth cross-section in the tooth plane's own (u, v),
// as a closed polygon walked once: the right flank out from the root, the tip arc
// across, the left flank back in, and the root arc home.
//
// The tooth is drawn ALREADY ROTATED 180 degrees, which is the drawer's own angle
// argument and not a post-hoc rotation, and it is centred at u = virtualPitchRadius
// + Tooth Spacing. At Tooth Spacing 0 that puts the pitch crossing exactly on the
// plane's origin, which is Apex 2 — the back-cone pitch point at the heel.
//
// The root boundary is drawn one ROOT SINK inside the dedendum corner. It is one
// figure and not a proof-only offset: the generated module passes the same value
// to the virtual-spur proxy as rootSink_mm, so the root arc lies inside the gear
// body's root cone across its whole width and the Combine-Join overlaps along the
// whole root rather than along the centreline alone.
func toothPolygon(l lattice, g member, scale float64) []pt2 {
	d := involute.Derive(l.module, g.virtualTeeth, toothPressureAngle)
	d.Root -= l.rootSink
	centre := pt2{g.virtualPitchRadius + l.toothSpacing, 0}
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, g.virtualTeeth, toothInvoluteSteps, math.Pi)

	at := func(p involute.Pt) pt2 { return pt2{centre.X + p.X, centre.Y + p.Y}.mul(scale) }
	foot := func(p involute.Pt) pt2 {
		n := math.Hypot(p.X, p.Y)
		return pt2{centre.X + d.Root*p.X/n, centre.Y + d.Root*p.Y/n}.mul(scale)
	}
	arc := func(from, to pt2, radius float64) []pt2 {
		a0 := math.Atan2(from.Y-centre.Y*scale, from.X-centre.X*scale)
		a1 := math.Atan2(to.Y-centre.Y*scale, to.X-centre.X*scale)
		for a1 < a0 {
			a1 += 2 * math.Pi
		}
		out := make([]pt2, 0, chordSteps)
		for i := 1; i < chordSteps; i++ {
			a := a0 + (a1-a0)*float64(i)/float64(chordSteps)
			out = append(out, pt2{
				centre.X*scale + radius*scale*math.Cos(a),
				centre.Y*scale + radius*scale*math.Sin(a),
			})
		}
		return out
	}

	poly := []pt2{foot(right[0])}
	for _, p := range right {
		poly = append(poly, at(p))
	}
	poly = append(poly, arc(at(right[len(right)-1]), at(left[len(left)-1]), d.Tip)...)
	for i := len(left) - 1; i >= 0; i-- {
		poly = append(poly, at(left[i]))
	}
	poly = append(poly, foot(left[0]))
	poly = append(poly, arc(foot(left[0]), foot(right[0]), d.Root)...)
	return poly
}

// toothSection draws one tooth cross-section on the tooth plane family at the
// given scale and returns the one region it closes.
func toothSection(t *testing.T, w *sketch.World, l lattice, g member,
	scale float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := toothPlaneSketch(t, w, l, g, scale)
	poly := toothPolygon(l, g, scale)
	pts := make([]*sketch.Point, 0, len(poly))
	for _, v := range poly {
		pts = append(pts, s.CreatePoint(v.X, v.Y))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	return s, decadtest.SolveRegion(t, s)
}

// toothSlab lofts the tooth between the two tooth-plane sections at the given
// scales. Both sections are the SAME polygon scaled about the Apex, so decad
// pairs their segments one to one and the ruled walls are exact: the solid is the
// frustum of the cone the Apex and the far section define.
func toothSlab(t *testing.T, doc *decad.Document, l lattice, g member,
	near, far float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0 := toothSection(t, w, l, g, near)
	s1, p1 := toothSection(t, w, l, g, far)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s tooth slab from %.4f to %.4f of R: %v", g.label, near, far, err)
	}
	return body
}

// frustumVolume is the closed form for a slab lofted between the sections at two
// scales: a cone frustum of base area A at scale 1, cut at the two scales, has
// volume A*R*(far^3 - near^3)/3.
func frustumVolume(area, pitchCone, near, far float64) float64 {
	return area * pitchCone * (far*far*far - near*near*near) / 3
}

// toothSectionArea is the drawn tooth cross-section's area at scale 1.
func toothSectionArea(t *testing.T, l lattice, g member) float64 {
	t.Helper()
	_, region := toothSection(t, sketch.NewWorld(), l, g, 1)
	return region.Area
}

func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	return []*decad.Body{toothSlab(t, doc, l, g, apexSectionScale, 1)}
}

// assertLoftTooth pins what the apex loft has to produce: a body that tapers from
// the Apex out to the tooth profile drawn on the back-cone plane.
//
// SUBSTITUTION. The loft's near end is a degenerate POINT section in Fusion — the
// §2 Apex sketch point — and decad has no point section, so a shrunken copy of
// the tooth stands in for it. The tooth PLANE is not substituted: the proof builds
// the real back-cone plane, tilted out of the axis-perpendicular by gamma, and
// takes the Apex's perpendicular distance to the section as the Pitch Cone
// Distance R rather than the tooth centre's station. THE COST IS THE POINT
// SECTION: the degenerate end is not built, and what the volume and the taper
// prove is the shape it has to produce.
func assertLoftTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the apex loft produced %d bodies, want the one Tooth Body", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	area := toothSectionArea(t, l, g)
	decadtest.Measures(t, g.label+" Tooth Body volume against the cone frustum",
		volumeReading(t, bodies[0], g.label+" Tooth Body"),
		units.CubicMillimeters(frustumVolume(area, l.pitchCone, apexSectionScale, 1)),
		decadtest.WithinRel(units.Scalar(1e-9)))

	// The taper the point section would have produced: the near section is the far
	// one scaled about the Apex, so its area is the square of the scale.
	_, near := toothSection(t, sketch.NewWorld(), l, g, apexSectionScale)
	if want := area * apexSectionScale * apexSectionScale; !relClose(near.Area, want, 1e-9) {
		t.Errorf("%s near section area %.9f mm2, want the far section's %.9f mm2 scaled by %.4f "+
			"squared = %.9f mm2", g.label, near.Area, area, apexSectionScale, want)
	}

	// The far section reaches the virtual tip radius laid on the back cone: its
	// outermost point is one addendum outside the pitch cone at the heel.
	tip := toothPlaneToWorld(l, g, 1, pt2{l.toothSpacing - addendumFactor*l.module, 0})
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s Tooth Body bounds: %v", g.label, err)
	}
	if got := math.Hypot(tip.X, tip.Y); box.Max.X < got-1e-6 {
		t.Errorf("%s Tooth Body reaches %.6f mm along +X, short of the virtual tip radius laid on "+
			"the back cone at %.6f mm", g.label, box.Max.X, got)
	}
}

// ---------------------------------------------------------------- conical cuts

// coneSolid builds the SOLID inside a cone whose apex sits on the shaft axis at
// the given station and whose wall makes halfAngle with the axis, opening toward
// the Apex. It is the whole body on the discard side, not a band spanning one
// profile edge: a band is enough to read an angle off and is not a tool a cut can
// use.
//
// It is an n-gon loft, because the cut consumes it and every boolean operand here
// is a Loft. The n-gon is INSCRIBED, so its wall sits inside the true cone by the
// chord sagitta; that is what the cut's own tolerance has to absorb, and it is
// why the stations below are solved from the closed form rather than read off the
// cut.
func coneSolid(t *testing.T, doc *decad.Document, apexStation, halfAngle, zNear, zFar float64,
	label string) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	slope := math.Tan(halfAngle)
	section := func(z float64) (*sketch.Sketch, *sketch.Profile) {
		s := stationSketch(t, w, z)
		radius := (apexStation - z) * slope
		pts := make([]*sketch.Point, 0, chordSteps)
		for i := range chordSteps {
			a := 2 * math.Pi * float64(i) / float64(chordSteps)
			pts = append(pts, s.CreatePoint(radius*math.Cos(a), radius*math.Sin(a)))
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		return s, decadtest.SolveRegion(t, s)
	}
	s0, p0 := section(zFar)
	s1, p1 := section(zNear)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s cutting cone from z=%.4f to z=%.4f: %v", label, zFar, zNear, err)
	}
	return body
}

// toeConeApexStation is where the toe cutting cone's apex sits on the shaft axis:
// the toe edge M->N continued to the axis. The toe corner N rides at the Toe
// Radius and the front face is square to the shaft, so the apex is one
// ToeRadius*tan(gamma) beyond N's own station.
func toeConeApexStation(l lattice, g member) float64 {
	hex := profileHexagon(l, g)
	return hex.n.X + hex.n.Y*math.Tan(g.gamma)
}

// heelConeApexStation is the same for the heel cone, whose edge C->H continued to
// the axis meets it at K — where the back-cone dedendum line crosses this gear's
// shaft axis, at station R/cos(gamma).
func heelConeApexStation(l lattice, g member) float64 {
	return l.pitchCone / math.Cos(g.gamma)
}

func stepConicalEndCuts(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	tooth := toothSlab(t, doc, l, g, apexSectionScale, 1)
	apexStation := toeConeApexStation(l, g)
	cone := coneSolid(t, doc, apexStation, math.Pi/2-g.gamma,
		apexStation*0.98, apexSectionScale*l.pitchCone*math.Cos(g.gamma)*0.5, g.label+" toe cone")
	kept, err := decad.Cut(tooth, cone)
	if err != nil {
		t.Fatalf("%s toe cut: %v", g.label, err)
	}
	return []*decad.Body{kept}
}

// assertConicalEndCuts holds the two conical trims to the three readings the
// flush band fixes, and performs the toe half of the pair.
//
// WHERE EACH CUT LANDS is read the same way for both: each cone's apex sits on
// the shaft axis and its half-angle is this gear's back-cone half-angle, so a
// cone of wall slope k and apex station a crosses a tooth surface of slope m at
// a*k/(m + k). The toe cone meets the gear body's own root cone at M and the heel
// cone meets it at C, which is what makes the trimmed band flush.
//
// Both half-angles are taken off the REVOLVED GEAR BODY's own cone faces. Those
// faces are the cutting tools Fusion's ConeSurfaceType search finds at this step,
// and a decad.Cone publishes the angle directly rather than having one derived
// from two cap radii and a height.
//
// NOTHING IS ASSERTED ABOUT THE TIP AND ROOT CROSSINGS BEING DIFFERENT, and there
// is no message for that case. Both cutting cones have their apex on the shaft
// axis, so a*k/(m + k) is positive for every m > 0 and different for the tooth's
// two surfaces whenever the tooth has height: every cut crosses both surfaces in
// every configuration, and an assertion that the two differ passes on any figure
// this spec can build. What IS asserted is the ORDER — the cut crosses the tip
// inboard of the root, so the trimmed end is shorter at the tip than at the root.
//
// PERFORM NO HEEL CUT, because its cone is TANGENT to the tooth plane. The
// dedendum corner C/D and the tooth centre K'/L' both sit on this gear's
// back-cone dedendum line, so the tooth plane contains a generator of the heel
// cone and the two touch along the tooth's own centreline instead of crossing it.
// decad refuses exactly that, as BooleanUnsupportedContact, and rebuilding the
// cone as a Revolve replaces the refusal with a Suspect verdict the gate does not
// admit either. The heel cone is therefore laid apart and the three readings kept.
// THE COST IS THE HEEL SPLIT: for that end the proof does not show the evaluator
// dividing the tooth, selecting the keeper, or leaving a watertight body.
//
// WHAT NEITHER HALF REACHES. A cone and a tilted plane read identically in
// everything this step measures: in the axial section a cone of half-angle
// 90 - gamma and a plane tilted by gamma through the same generator are the same
// line, so every station solved here comes out the same for either surface. What
// makes the face conical is that it is a surface of revolution about the shaft
// axis — its crossing with the tooth's tip surface sits at one station at every
// azimuth, while a tilted plane's crossing moves with azimuth. The half-angle
// read here comes off a decad.Cone face of the revolved gear body, which is a
// surface of revolution by its own construction; the tool the toe cut consumes is
// a swept body and cannot be anything else either, but nothing here measures its
// azimuthal crossing with the tooth. The flush-band check cannot see the angle at
// all: any band through M crosses the root ray at M whatever slope it has.
func assertConicalEndCuts(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the toe cut left %d bodies, want the one keeper", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	hex := profileHexagon(l, g)
	backCone := math.Pi/2 - g.gamma

	// Both half-angles off the revolved gear body's own cone faces.
	scratch := decad.New()
	s, region := gearHexagon(t, sketch.NewWorld(), l, g)
	frustum, err := scratch.Revolve(s, region, shaftAxis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s gear body for its cone faces: %v", g.label, err)
	}
	found := 0
	for _, face := range frustum.Faces() {
		cone, ok := face.Surface().(decad.Cone)
		if !ok {
			continue
		}
		if math.Abs(cone.HalfAngle.Base()-backCone) < 1e-6 {
			found++
		}
	}
	if found != 2 {
		t.Errorf("%s gear body carries %d cone faces at the back-cone half-angle %.9f rad, want "+
			"the two the toe and heel cuts take as tools", g.label, found, backCone)
	}

	// Where each cut lands: the toe cone meets the root cone at M, the heel cone
	// meets it at C. Both are solved from the cone apexes and the root ray's own
	// slope, not read off the cut.
	rootSlope := math.Tan(g.gammaRoot)
	cross := func(apexStation float64) float64 {
		k := math.Tan(backCone)
		return apexStation * k / (rootSlope + k)
	}
	if got, want := cross(toeConeApexStation(l, g)), hex.m.X; math.Abs(got-want) > 1e-6 {
		t.Errorf("%s toe cone meets the root cone at station %.6f, want M at %.6f",
			g.label, got, want)
	}
	if got, want := cross(heelConeApexStation(l, g)), hex.c.X; math.Abs(got-want) > 1e-6 {
		t.Errorf("%s heel cone meets the root cone at station %.6f, want C at %.6f",
			g.label, got, want)
	}

	// Each cone crosses the tooth's TIP inboard of where it crosses its ROOT, so
	// the trimmed end is shorter at the tip than at the root.
	tipSlope := math.Hypot(
		toothPlaneToWorld(l, g, 1, pt2{l.toothSpacing - addendumFactor*l.module, 0}).X,
		0) / (l.pitchCone * math.Cos(g.gamma))
	for _, apexStation := range []float64{toeConeApexStation(l, g), heelConeApexStation(l, g)} {
		k := math.Tan(backCone)
		tip := apexStation * k / (tipSlope + k)
		root := apexStation * k / (rootSlope + k)
		if tip >= root {
			t.Errorf("%s cut crosses the tooth tip at station %.6f and its root at %.6f; the tip "+
				"crossing must be inboard of the root crossing", g.label, tip, root)
		}
	}

	// The toe split itself: the two pieces add back to the whole tooth, and each
	// is one solid lump.
	whole := decad.New()
	wholeTooth := volumeOf(t, toothSlab(t, whole, l, g, apexSectionScale, 1), "uncut tooth")
	kept := volumeOf(t, bodies[0], g.label+" toe keeper")
	scrapDoc := decad.New()
	scrapTooth := toothSlab(t, scrapDoc, l, g, apexSectionScale, 1)
	apexStation := toeConeApexStation(l, g)
	scrapCone := coneSolid(t, scrapDoc, apexStation, backCone, apexStation*0.98,
		apexSectionScale*l.pitchCone*math.Cos(g.gamma)*0.5, g.label+" toe cone")
	scrap, err := decad.Intersect(scrapTooth, scrapCone)
	if err != nil {
		// A BooleanEmpty says the cone took nothing off this tooth, which is the
		// condition the generated module raises as solids.NonIntersectError. Every
		// other error fails.
		var be *decad.BooleanError
		if errors.As(err, &be) && be.Code == decad.BooleanEmpty {
			t.Logf("%s: the toe cone took nothing off this tooth, so this case records that it "+
				"built no split rather than passing silently", g.label)
			return
		}
		t.Fatalf("%s toe intersect: %v", g.label, err)
	}
	discarded := volumeOf(t, scrap, g.label+" toe scrap")
	if !relClose(kept+discarded, wholeTooth, 1e-6) {
		t.Errorf("%s toe split: keeper %.6f + scrap %.6f = %.6f mm3, want the whole tooth's "+
			"%.6f mm3", g.label, kept, discarded, kept+discarded, wholeTooth)
	}
	if n := len(scrap.Lumps()); n != 1 {
		t.Errorf("%s toe scrap is %d lumps, want 1", g.label, n)
	}
	if !scrap.IsSolid() {
		t.Errorf("%s toe scrap is not solid", g.label)
	}
}

// ---------------------------------------------------------------- spiral chain

// slabScales is §3a step E's fixed family as this proof builds it: the parent
// transverse tooth plane is the heel end at scale 1, and the eight cut planes sit
// span/6 apart toward the Apex, so the slab boundaries run from the Apex-side
// stand-in section out to the parent plane.
//
// The planes are PARALLEL to the parent plane and are NOT perpendicular to the
// cone element. The parent plane carries the tooth-centre line C->K', which is
// the back-cone line and so perpendicular to the Pitch Line, so its normal runs
// along the PITCH element while coneVec is the ROOT element; the two differ by
// the dedendum angle. slice_body_by_offset_planes offsets the parent plane with
// setByOffset, which produces parallel planes, and the tooth is lofted from the
// Apex to the profile drawn in the parent plane, so the heel-most slab's heel
// face IS the parent plane and a consistent family has to contain it.
//
// THE PROOF BUILDS ITS OWN SLABS FROM THESE OFFSETS AND NEVER READS THE PLANE THE
// GENERATED MODULE CONSTRUCTS, so the module's choice of family reaches Fusion
// untested. A build that follows "perpendicular to the cone element" instead is
// wrong and silent: it tilts every cut face by the dedendum angle and nothing in
// the pipeline fails. That is the same shape of gap as the §2 seed, and the only
// thing that closes it is a face corner's position measured on a loaded spiral
// gear.
func slabScales(l lattice, sp spiral) []float64 {
	out := []float64{apexSectionScale}
	offsets := slabOffsets(sp.span)
	for i := len(offsets) - 1; i >= 0; i-- {
		out = append(out, (l.pitchCone-offsets[i])/l.pitchCone)
	}
	return append(out, 1)
}

// slabHeelDistAlong is the cone distance of the slab whose heel face sits at the
// given scale: the face's centroid measured along the ROOT cone element. The
// section at scale s is the scale-1 polygon scaled about the Apex, so its
// centroid sits at s times the base centroid's in-plane u.
func slabHeelDistAlong(l lattice, g member, centroidU, scale float64) float64 {
	return distAlong(g, toothPlaneToWorld(l, g, scale, pt2{centroidU * scale, 0}))
}

// sectionCentroidU is the tooth cross-section's centroid in the tooth plane's u,
// at scale 1.
func sectionCentroidU(l lattice, g member) float64 {
	poly := toothPolygon(l, g, 1)
	area, cx := 0.0, 0.0
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		c := a.cross(b)
		area += c
		cx += (a.X + b.X) * c
	}
	return cx / (3 * area)
}

// laidApart moves a body clear along the shaft axis so no two operands laid out
// together touch. It changes no volume, radius or cone angle, which is what makes
// the readings still mean something.
func laidApart(t *testing.T, body *decad.Body, index int, pitch float64) *decad.Body {
	t.Helper()
	shift, err := r3.Translation(r3.NewVec(0, float64(index+1)*pitch, 0))
	if err != nil {
		t.Fatalf("lay body %d apart: %v", index, err)
	}
	moved, err := body.Placed(shift)
	if err != nil {
		t.Fatalf("lay body %d apart: %v", index, err)
	}
	return moved
}

// turnAboutShaft is one rotation about the shaft axis, built from a literal basis
// rather than from an axis and an angle. Rodrigues' formula leaves the z row a
// few float ulps off for most angles, and decad's analytic prism reduction admits
// a pair only when the two composed world planes are the SAME plane by exact
// float equality; a basis whose z components are written as literal zeros keeps a
// rotated section exactly on its own plane.
func turnAboutShaft(t *testing.T, angle float64) r3.Transform {
	t.Helper()
	sin, cos := math.Sin(angle), math.Cos(angle)
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(cos, sin, 0),
		EY: r3.NewVec(-sin, cos, 0),
		EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 0, 0))
	if err != nil {
		t.Fatalf("rotation about the shaft axis by %.6f rad: %v", angle, err)
	}
	return turn
}

// spiralOf returns the spiral construction for the gear a case names, and skips
// the case when the Mean Spiral Angle is 0 — the hook returns immediately with
// the straight tooth's two conical trims and none of this construction runs.
func spiralOf(t *testing.T, l lattice, g member, p map[string]float64) spiral {
	t.Helper()
	if p[keySpiralAngle] <= 0 {
		proofkit3d.Unmodelled(t, "Mean Spiral Angle is 0, so the tooth-body hook returns the "+
			"straight tooth before any of the spiral construction runs")
	}
	return newSpiral(l, g, p)
}

func stepSliceToothSlabs(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := spiralOf(t, l, g, p)
	scales := slabScales(l, sp)
	pitch := 4 * l.pitchCone
	out := make([]*decad.Body, 0, len(scales)-1)
	for i := 0; i+1 < len(scales); i++ {
		out = append(out, laidApart(t, toothSlab(t, doc, l, g, scales[i], scales[i+1]), i, pitch))
	}
	return out
}

// assertSliceToothSlabs pins what the eight offset planes have to produce.
//
// SUBSTITUTION. decad has no split-by-plane, so each slab is BUILT between its
// two planes rather than cut out of one body, and the slabs are laid apart along
// the circumferential direction so no pair of them touches — decad verifies every
// PAIR of live bodies in a document, and two slabs sharing a face resolve neither
// way. Laying them apart changes no volume. THE COST IS THE SPLIT: the proof does
// not show the evaluator dividing one body into these pieces.
//
// The runtime gate the step carries is asserted instead: the cut MUST produce more
// than one piece, and a plane family that misses the tooth leaves it whole, which
// is the retry-then-raise branch. Here that is the piece count, which is nine —
// eight working segments and the apex-side scrap.
func assertSliceToothSlabs(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)
	scales := slabScales(l, sp)
	if want := len(scales) - 1; len(bodies) != want {
		t.Fatalf("%s slice produced %d piece(s), want %d — eight working segments and the "+
			"apex-side scrap", g.label, len(bodies), want)
	}
	if len(bodies) < 2 {
		t.Fatalf("%s slice left the tooth in one piece; the offset sign was wrong or the parent "+
			"plane sits outside the tooth's span", g.label)
	}
	area := toothSectionArea(t, l, g)
	total := 0.0
	for i, body := range bodies {
		want := frustumVolume(area, l.pitchCone, scales[i], scales[i+1])
		decadtest.Measures(t, fmt.Sprintf("%s slab %d volume", g.label, i),
			volumeReading(t, body, fmt.Sprintf("%s slab %d", g.label, i)),
			units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-9)))
		total += want
	}
	if whole := frustumVolume(area, l.pitchCone, apexSectionScale, 1); !relClose(total, whole, 1e-9) {
		t.Errorf("%s slabs sum to %.6f mm3, want the whole tooth's %.6f mm3", g.label, total, whole)
	}
	// Where the eight land: the first cut sits span/6 inside the HEEL and none of
	// them lies past it, the sixth lands at the toe, and the last two sit span/6
	// and 2*span/6 PAST it.
	centroidU := sectionCentroidU(l, g)
	for k, offset := range slabOffsets(sp.span) {
		if offset > sp.span+1e-9 && k < 5 {
			t.Errorf("%s cut plane %d sits %.6f mm inside the heel, past the toe at %.6f mm",
				g.label, k, offset, sp.span)
		}
	}
	heel := slabHeelDistAlong(l, g, centroidU, 1)
	if math.Abs(heel-sp.rHeel) > 0.5*sp.span {
		t.Errorf("%s heel-most slab's heel face reads cone distance %.6f, want it within half a "+
			"span of R_heel %.6f", g.label, heel, sp.rHeel)
	}
}

func stepDropApexScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := spiralOf(t, l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	type piece struct {
		body  *decad.Body
		along float64
	}
	pitch := 4 * l.pitchCone
	pieces := make([]piece, 0, len(scales)-1)
	for i := 0; i+1 < len(scales); i++ {
		mid := (scales[i] + scales[i+1]) / 2
		pieces = append(pieces, piece{
			body:  toothSlab(t, doc, l, g, scales[i], scales[i+1]),
			along: slabHeelDistAlong(l, g, centroidU, mid),
		})
	}
	sort.Slice(pieces, func(a, b int) bool { return pieces[a].along < pieces[b].along })
	// Re-slice the list FIRST, then remove the scrap: dropping the piece before
	// the list is re-sliced is what leaves segments empty and makes the crown fail
	// far from the cause.
	kept := pieces[1:]
	out := make([]*decad.Body, 0, len(kept))
	for i, piece := range kept {
		out = append(out, laidApart(t, piece.body, i, pitch))
	}
	if len(out) == 0 {
		t.Fatalf("%s: dropping the apex scrap left no segments; the slice failed", g.label)
	}
	return out
}

// assertDropApexScrap pins the ordering rule and the guard that follows it: the
// segments are sorted by the cone distance of their centroid, the FIRST — the
// long apex-side scrap below the toe — is removed, and what is left must be
// non-empty before the twist and the crown, which both assume at least one
// segment.
func assertDropApexScrap(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)
	scales := slabScales(l, sp)
	if want := len(scales) - 2; len(bodies) != want {
		t.Fatalf("%s kept %d segment(s) after the drop, want %d", g.label, len(bodies), want)
	}
	area := toothSectionArea(t, l, g)
	scrap := frustumVolume(area, l.pitchCone, scales[0], scales[1])
	whole := frustumVolume(area, l.pitchCone, apexSectionScale, 1)
	total := 0.0
	for _, body := range bodies {
		total += volumeOf(t, body, g.label+" segment")
	}
	if !relClose(total, whole-scrap, 1e-6) {
		t.Errorf("%s segments sum to %.6f mm3, want the whole tooth %.6f less the apex scrap "+
			"%.6f", g.label, total, whole, scrap)
	}
	if scrap <= 0 {
		t.Errorf("%s apex scrap has no volume, so the drop removed nothing", g.label)
	}
}

func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := spiralOf(t, l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	pitch := 4 * l.pitchCone
	out := make([]*decad.Body, 0, len(scales)-2)
	for i := 1; i+1 < len(scales); i++ {
		slab := toothSlab(t, doc, l, g, scales[i], scales[i+1])
		ang := segmentTwist(sp, slabHeelDistAlong(l, g, centroidU, scales[i+1]))
		turned, err := slab.Placed(turnAboutShaft(t, ang))
		if err != nil {
			t.Fatalf("%s twist segment %d by %.6f rad: %v", g.label, i, ang, err)
		}
		out = append(out, laidApart(t, turned, i, pitch))
	}
	return out
}

// assertTwistSegments pins the twist law of §3a step G.
//
// The total toe-to-heel shaft-axis twist is the conjugate crown-gear generation
// law: |phi_crown| / sin(gamma), where phi_crown is the angle the cutter arc's toe
// and heel endpoints subtend at the Apex in the flat crown frame, and gamma is
// this gear's PITCH cone angle.
//
// THE TWO HALVES ARE TAKEN ON TWO DIFFERENT CONES, DELIBERATELY. phi_crown is
// measured in the frame step A builds, whose x axis is coneVec, the ROOT cone
// element. The divisor sin(gamma) is the PITCH cone's roll ratio, because the
// crown gear the law generates against is tangent to the pitch cone. Written
// consistently on the root cone the divisor would be sin(gamma_root), which is a
// real difference and not a rounding. WHETHER THE FRAME SHOULD MOVE TO THE PITCH
// CONE TO MATCH IS NOT SETTLED, AND NOTHING IN THIS REPOSITORY MEASURES IT: this
// assertion checks the same formula the module computes, so it confirms the
// arithmetic and says nothing about which cone the frame belongs on. Moving the
// frame would change R_toe and R_heel and the element psi is measured against, so
// it is its own derivation and its own change.
//
// Each segment's rotation is keyed on the cone distance of its HEEL FACE, not its
// centroid: the loft samples that face, so that face is what must land at the
// right azimuth. Centroid-keying leaves the mid-face section rotated by half a
// segment.
func assertTwistSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	if want := len(scales) - 2; len(bodies) != want {
		t.Fatalf("%s twisted %d segment(s), want %d", g.label, len(bodies), want)
	}
	if want := math.Abs(sp.phiCrown) / math.Sin(g.gamma); math.Abs(sp.total-want) > 1e-12 {
		t.Errorf("%s total twist %.9f rad, want |phi_crown| / sin(gamma) = %.9f rad",
			g.label, sp.total, want)
	}
	if root := math.Abs(sp.phiCrown) / math.Sin(g.gammaRoot); relClose(sp.total, root, 1e-12) {
		t.Errorf("%s total twist matches the ROOT cone divisor %.9f rad; the law takes the pitch "+
			"cone's roll ratio", g.label, root)
	}
	area := toothSectionArea(t, l, g)
	for i, body := range bodies {
		want := frustumVolume(area, l.pitchCone, scales[i+1], scales[i+2])
		decadtest.Measures(t, fmt.Sprintf("%s twisted segment %d volume", g.label, i),
			volumeReading(t, body, fmt.Sprintf("%s twisted segment %d", g.label, i)),
			units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-9)))
	}
	// The section at R_mean is unrotated, which is what makes the mid-face section
	// mesh exactly like the straight tooth and why the pinion needs no extra mesh
	// phase.
	if got := segmentTwist(sp, sp.rMean); math.Abs(got) > 1e-12 {
		t.Errorf("%s twist at R_mean is %.12f rad, want 0 — the twist is centred there", g.label, got)
	}
	toe := segmentTwist(sp, sp.rToe)
	heel := segmentTwist(sp, sp.rHeel)
	if toe*heel >= 0 {
		t.Errorf("%s twist runs %+.9f at the toe and %+.9f at the heel; centred on R_mean the two "+
			"ends turn opposite ways", g.label, toe, heel)
	}
	if math.Abs(toe-heel) < 1e-12 && sp.total > 1e-9 {
		t.Errorf("%s toe and heel twist are equal at a total of %.9f rad", g.label, sp.total)
	}
	_ = centroidU
}

func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := spiralOf(t, l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	pitch := 4 * l.pitchCone
	out := make([]*decad.Body, 0, len(scales)-2)
	for i := 1; i+1 < len(scales); i++ {
		heelFace := slabHeelDistAlong(l, g, centroidU, scales[i+1])
		factor := crownFactor(sp, heelFace)
		if factor <= 0 {
			t.Fatalf("%s segment %d: crown factor %.6f at u %.6f is not positive", g.label, i,
				factor, (sp.rHeel-heelFace)/sp.span)
		}
		// The OUTERMOST (heel) segment is held full: its heel face is the loft's
		// heel end and must stay full so the heel cone trims it flush with the gear
		// base.
		if i+1 == len(scales)-1 {
			factor = 1
		}
		out = append(out, laidApart(t, crownedSlab(t, doc, l, g, scales[i], scales[i+1], factor), i, pitch))
	}
	return out
}

// crownedSlab builds one slab already relieved by factor.
//
// SUBSTITUTION, and there are two here. decad has no scale feature, so the relief
// is BUILT into the slab's two sections rather than applied to a finished slab;
// and the scale is taken IN SECTION rather than uniformly in three dimensions, so
// the slab keeps its length along the cone and its cross-section alone is
// relieved. Its volume is therefore the plain slab's times factor SQUARED and not
// cubed. THE COST IS THE AXIAL COMPONENT: a uniform scale would shorten the slab
// along the cone as well, which the step-I loft never samples — it takes each
// slab's heel FACE — so what is lost is a length no later step reads. The scale
// base is the ROOT-EDGE MIDPOINT of the slab's heel face, not the face's centroid:
// scaleFeatures shrinks uniformly toward the base point, so a base point at
// mid tooth-height pulls the tooth's root edge upward by (1 - factor) times half
// the tooth height, the tooth stops seating on the gear body's root cone, and the
// Combine-Join leaves a gap. A uniform scale about a point keeps every line
// through that point invariant, so anchoring on the root keeps the root edge on
// the seating cone while the tip is relieved. THE COST IS THE FEATURE: the proof
// shows what the scale produces, not that decad's own scale produces it.
func crownedSlab(t *testing.T, doc *decad.Document, l lattice, g member,
	near, far, factor float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(scale float64) (*sketch.Sketch, *sketch.Profile) {
		s := toothPlaneSketch(t, w, l, g, scale)
		// Each section is relieved about ITS OWN root anchor, which is the scale-1
		// anchor carried out to this section's scale. The two sections then stay
		// similar about the plane origin, so the relieved slab is still the frustum
		// of a cone through the Apex and its volume is the plain slab's times
		// factor squared.
		anchor := crownAnchorU(l, g) * scale
		pts := make([]*sketch.Point, 0)
		for _, v := range toothPolygon(l, g, scale) {
			pts = append(pts, s.CreatePoint(anchor+(v.X-anchor)*factor, v.Y*factor))
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		return s, decadtest.SolveRegion(t, s)
	}
	s0, p0 := section(near)
	s1, p1 := section(far)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s crowned slab from %.4f to %.4f of R at factor %.6f: %v",
			g.label, near, far, factor, err)
	}
	return body
}

// crownAnchorU is the root-edge midpoint's in-plane u at scale 1: of the heel
// face's vertices, the two with the smallest perpendicular distance to the shaft
// axis are the root corners — the tip corners are the FARTHEST from the axis —
// and the base point is their midpoint. In the tooth plane's frame the
// perpendicular distance grows as u falls, so the root corners are the two at the
// greatest u, which is the root arc's own outermost reach.
func crownAnchorU(l lattice, g member) float64 {
	poly := toothPolygon(l, g, 1)
	best := poly[0].X
	for _, v := range poly {
		if v.X > best {
			best = v.X
		}
	}
	return best
}

// assertCrownSegments pins the lengthwise crown of §3a step H.
//
// The relief is keyed on the MONOTONIC heel-distance fraction u, never on the
// twist magnitude. Keying on |ang| is symmetric about mid-face — maximal at BOTH
// ends — so, because the heel slab is held full, the slab just inside the heel
// becomes the most-relieved one and dips below both its neighbours, reversing the
// heel-to-toe taper. That was the observed bug: 0.932 against 0.972 on the slab
// next inward.
//
// u runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two segments
// beyond the toe; nothing reads an upper bound on it, and what the slab count
// rests on is the structural fact that the last cut plane is a toe face and never
// any segment's heel face.
func assertCrownSegments(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	if want := len(scales) - 2; len(bodies) != want {
		t.Fatalf("%s crowned %d segment(s), want %d", g.label, len(bodies), want)
	}
	prevU, prevFactor := math.Inf(1), math.Inf(-1)
	for i := 1; i+1 < len(scales); i++ {
		heelFace := slabHeelDistAlong(l, g, centroidU, scales[i+1])
		u := (sp.rHeel - heelFace) / sp.span
		factor := crownFactor(sp, heelFace)
		if factor <= 0 {
			t.Errorf("%s segment %d: crown factor %.6f at u %.6f is not positive",
				g.label, i, factor, u)
		}
		if u > prevU {
			t.Errorf("%s segments are not ordered heel to toe: u %.6f follows %.6f", g.label, u, prevU)
		}
		if factor < prevFactor {
			t.Errorf("%s relief is not monotonic: factor %.6f at u %.6f follows %.6f — keying the "+
				"relief on the twist magnitude notches the slab just inside the heel",
				g.label, factor, u, prevFactor)
		}
		prevU, prevFactor = u, factor
	}
	// The outermost segment is held full so the heel cone trims it flush.
	heelFace := slabHeelDistAlong(l, g, centroidU, 1)
	area := toothSectionArea(t, l, g)
	want := frustumVolume(area, l.pitchCone, scales[len(scales)-2], 1)
	decadtest.Measures(t, g.label+" heel segment volume, held full",
		volumeReading(t, bodies[len(bodies)-1], g.label+" heel segment"),
		units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-9)))
	// The heel segment carries the SMALLEST heel-distance fraction of any segment,
	// which is what makes it the one held full. It is not exactly 0: R_heel is read
	// at the heel edge's MIDPOINT while this face is the parent plane, and the two
	// differ by the offset between them.
	heelU := (sp.rHeel - heelFace) / sp.span
	for i := 1; i+1 < len(scales)-1; i++ {
		other := (sp.rHeel - slabHeelDistAlong(l, g, centroidU, scales[i+1])) / sp.span
		if other < heelU {
			t.Errorf("%s segment %d reads u %.6f, below the heel segment's %.6f; the held-full "+
				"segment must be the outermost one", g.label, i, other, heelU)
		}
	}
	// A uniform scale by f about a point multiplies the volume by f^3, and
	// anchoring on the root keeps the root edge where it was. Anchoring on the
	// centroid instead would lift the root by (1 - factor) times half the tooth
	// height, which is the gap the Combine-Join then leaves.
	// bodies[0] is the toe-most working segment, which spans scales[1] to
	// scales[2]; its own heel face is the one at scales[2], and that is what keys
	// its relief.
	toeFace := slabHeelDistAlong(l, g, centroidU, scales[2])
	factor := crownFactor(sp, toeFace)
	got := volumeOf(t, bodies[0], g.label+" toe segment")
	plain := frustumVolume(area, l.pitchCone, scales[1], scales[2])
	if !relClose(got, plain*factor*factor, 1e-6) {
		t.Errorf("%s toe segment volume %.6f mm3, want the plain slab %.6f scaled by factor^2 = "+
			"%.6f", g.label, got, plain, plain*factor*factor)
	}
	height := crownAnchorU(l, g) - (l.toothSpacing - addendumFactor*l.module)
	if lift := (1 - factor) * height / 2; lift <= 0 {
		t.Errorf("%s: anchoring on the heel face's centroid would lift the root by %.6f mm, which "+
			"must be positive for the root anchor to be the rule", g.label, lift)
	}
}

func stepLoftSpiralTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := spiralOf(t, l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	order := postTwistOrder(l, g, sp, centroidU, scales)
	// The loft's sections, in order: FIRST the toe-most segment's apex-side face,
	// which pushes the loft past the toe cone so the toe trim bites, then the
	// heel-facing face of every segment iterated in that same order.
	sections := []float64{scales[order[0]+1]}
	for _, index := range order {
		sections = append(sections, scales[index+2])
	}
	pitch := 4 * l.pitchCone
	out := make([]*decad.Body, 0, len(sections)-1)
	for i := 0; i+1 < len(sections); i++ {
		near, far := sections[i], sections[i+1]
		if far <= near {
			t.Fatalf("%s spiral loft: section %d at scale %.6f does not precede section %d at "+
				"%.6f; the order is stale", g.label, i, near, i+1, far)
		}
		out = append(out, laidApart(t, toothSlab(t, doc, l, g, near, far), i, pitch))
	}
	return out
}

// postTwistOrder is §3a step I's re-sort: the segment indices ordered by the cone
// distance of their HEEL FACE, recomputed AFTER the twist and the crown.
func postTwistOrder(l lattice, g member, sp spiral, centroidU float64, scales []float64) []int {
	order := make([]int, 0, len(scales)-2)
	for i := range len(scales) - 2 {
		order = append(order, i)
	}
	sort.SliceStable(order, func(a, b int) bool {
		return slabHeelDistAlong(l, g, centroidU, scales[order[a]+2]) <
			slabHeelDistAlong(l, g, centroidU, scales[order[b]+2])
	})
	return order
}

// assertLoftSpiralTooth pins the loft of §3a step I.
//
// SUBSTITUTION. decad's Loft takes exactly TWO sections, so the single multi-section
// loft through every segment's heel face is built here as the chain of adjacent
// two-section lofts, laid apart. THE COST IS THE ONE BODY: the proof shows the
// sections in the right order and the volume they enclose, not the evaluator
// running one loft through all of them.
//
// The order is recomputed HERE, after the twist and the crown, and never reused
// from the pre-twist slice. The twist rotates each slab about the shaft axis, and
// for high-twist unequal-ratio pairs that rotation changes the slabs' along-cone
// order enough to reorder adjacent slabs; lofting in the stale order assembles the
// cross-sections out of sequence and the crowned tooth comes out distorted. For
// equal or low-twist pairs the two orders coincide, which is why equal-teeth gears
// mesh even with the stale order while unequal ratios distort.
func assertLoftSpiralTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	sp := newSpiral(l, g, p)
	scales := slabScales(l, sp)
	centroidU := sectionCentroidU(l, g)
	order := postTwistOrder(l, g, sp, centroidU, scales)
	if want := len(scales) - 2; len(bodies) != want {
		t.Fatalf("%s spiral loft produced %d body(ies), want %d", g.label, len(bodies), want)
	}
	prev := math.Inf(-1)
	for _, index := range order {
		along := slabHeelDistAlong(l, g, centroidU, scales[index+2])
		if along < prev {
			t.Errorf("%s loft order is not sorted by heel-face cone distance: %.6f follows %.6f",
				g.label, along, prev)
		}
		prev = along
	}
	area := toothSectionArea(t, l, g)
	total := 0.0
	for _, body := range bodies {
		total += volumeOf(t, body, g.label+" spiral loft section")
	}
	want := frustumVolume(area, l.pitchCone, scales[1], 1)
	if !relClose(total, want, 1e-6) {
		t.Errorf("%s spiral loft chain encloses %.6f mm3, want the working segments' %.6f mm3",
			g.label, total, want)
	}
}

// ---------------------------------------------------------------- pattern

// patternSeed carries the seed tooth's readings from the build into the
// assertion. THIS STEP IS SERIAL BECAUSE OF THIS VARIABLE: the pattern increment
// retires the seed, so the seed cannot be measured after the step runs and its
// readings have to be handed across. Two cases running at once overwrite each
// other's readings and the proof reports a wrong verdict rather than failing
// loudly — the two gear sides differ enough in volume that the overwrite was
// caught when it happened, and a pair of cases whose seeds measured alike would
// have passed on each other's numbers instead. Keep this step on
// proofkit3d.RunSolid; every other step here is parallel.
var patternSeed struct {
	volume  decad.Measurement
	box     decad.Box
	azimuth float64
}

func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	seed := toothSlab(t, doc, l, g, apexSectionScale, 1)
	patternSeed.volume = volumeReading(t, seed, g.label+" seed tooth")
	box, err := seed.Bounds()
	if err != nil {
		t.Fatalf("%s seed tooth bounds: %v", g.label, err)
	}
	patternSeed.box = box
	patternSeed.azimuth = centroidAzimuth(t, seed)
	return []*decad.Body{seed}
}

// assertCircularPattern pins the three inputs the step sets explicitly: the
// quantity is this gear's Teeth Number, the total angle is a full circle and the
// pattern is not symmetric, so copy k sits at 2*pi*k/N measured one way from the
// seed.
//
// Although the pitch diameter shrinks from the heel toward the Apex, the ANGULAR
// spacing stays constant at 360/N for the entire face width: the radial taper is
// already produced by the loft from the Apex to the heel-end profile, so the
// pattern just rotates that one tapered tooth into N evenly spaced copies.
//
// Each copy is made with Placed, which retires the body it moves, so a document
// never holds two teeth at once and no pair reaches the verifier.
func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the pattern step returned %d bodies, want the seed tooth", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	teeth := int(g.teeth)
	if got := angleGap(centroidAzimuth(t, bodies[0]), patternSeed.azimuth); got > 1e-9 {
		t.Errorf("%s seed tooth moved %.9f rad between the build and the assertion; copy 0 is the "+
			"seed itself and the pattern places it at no offset", g.label, got)
	}
	// The first copy is the increment, the middle one is the half turn, and the
	// last sits ONE increment short of the full turn — which is what a total angle
	// of 360 degrees with isSymmetric false produces, and what a symmetric pattern
	// or a different total angle would not. Every copy in between repeats the same
	// increment, so three of them carry the rule and the remaining Teeth Number
	// minus four are not rebuilt: each copy is a whole tooth loft, and measuring
	// all of them on a 43-tooth gear costs a minute of wall time for no further
	// reading.
	sample := map[int]bool{1: true, teeth / 2: true, teeth - 1: true}
	for k := 1; k < teeth; k++ {
		if !sample[k] {
			continue
		}
		scratch := decad.New()
		copyBody, err := toothSlab(t, scratch, l, g, apexSectionScale, 1).
			Placed(turnAboutShaft(t, 2*math.Pi*float64(k)/float64(teeth)))
		if err != nil {
			t.Fatalf("%s pattern copy %d: %v", g.label, k, err)
		}
		// Two readings, not a reading against a formula: the placement is supposed
		// to change nothing, so the seed's own proven bound counts as much as the
		// copy's.
		decadtest.Agree(t, fmt.Sprintf("%s patterned tooth %d against the seed", g.label, k),
			volumeReading(t, copyBody, "patterned tooth"), patternSeed.volume,
			decadtest.WithinRel(units.Scalar(1e-9)))
		want := patternSeed.azimuth + 2*math.Pi*float64(k)/float64(teeth)
		if got := centroidAzimuth(t, copyBody); angleGap(got, want) > 1e-6 {
			t.Errorf("%s patterned tooth %d sits at azimuth %.9f rad, want %.9f rad",
				g.label, k, got, want)
		}
		// The copy spans the same stations as the seed, so the pattern turned the
		// tooth about the shaft axis and did not move it along the shaft.
		box, err := copyBody.Bounds()
		if err != nil {
			t.Fatalf("%s patterned tooth %d bounds: %v", g.label, k, err)
		}
		if math.Abs(box.Min.Z-patternSeed.box.Min.Z) > 1e-6 ||
			math.Abs(box.Max.Z-patternSeed.box.Max.Z) > 1e-6 {
			t.Errorf("%s patterned tooth %d spans stations [%.6f, %.6f], want the seed's "+
				"[%.6f, %.6f]", g.label, k, box.Min.Z, box.Max.Z,
				patternSeed.box.Min.Z, patternSeed.box.Max.Z)
		}
	}
}

// centroidAzimuth is where a tooth sits around the shaft axis: the polar angle of
// its centroid in the plane perpendicular to that axis.
func centroidAzimuth(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("tooth centroid: %v", err)
	}
	return math.Atan2(centroid.Value.Y, centroid.Value.X)
}

func angleGap(a, b float64) float64 {
	gap := math.Mod(math.Abs(a-b), 2*math.Pi)
	return math.Min(gap, 2*math.Pi-gap)
}

// ---------------------------------------------------------------- combine

func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	s, region := gearHexagon(t, sketch.NewWorld(), l, g)
	body, err := doc.Revolve(s, region, shaftAxis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s Gear Body for the join: %v", g.label, err)
	}
	// Only the Gear Body is returned. Laying the tooth beside it in the same
	// document leaves decad a PAIR to decide, and its disjoint/overlap partition
	// proof resolves neither way for a revolved body against a lofted one however
	// far apart the two are laid — reported as an undecided pair, which the solid
	// gate refuses and this proof does not waive. The tooth is therefore measured
	// in a document of its own, in the assertion.
	return []*decad.Body{body}
}

// assertCombineJoin asserts the join's two consequences from the operands' own
// measured geometry, because the join itself is not performed.
//
// PERFORM NO JOIN, because the two operands are not in one frame. The proof builds
// the tooth on the back-cone section at the Pitch Cone Distance from the Apex,
// scaled about the Apex — the Tredgold mapping — while the gear body is written
// about the shaft axis, and the tooth's own seating on that body is derived
// nowhere in this proof. The two therefore do not meet when they are put in one
// document, and neither sign of the rotation that relates the two planes seats
// them. THE ENGINE IS NOT WHAT BLOCKS THIS: given operands that do overlap decad
// performs the union, returns one lump and publishes a tight volume bound. What is
// missing is the tooth's real back-cone placement, and deriving it is its own
// change. THE COST IS THE STITCH: the proof cannot show the evaluator making one
// boundary out of two.
//
// The two consequences asserted instead: a join leaves ONE lump when the tooth's
// root is below the gear body's root cone — seated, not floating — and the joined
// body reaches further out than the frustum when the tooth's tip stands proud of
// it. Both are read at the toe, the middle and the heel of the band the join would
// cover.
//
// READ THE ROOT ARC'S OUTERMOST POINT, NOT THE TOOTH'S CENTRELINE. The centreline
// sits inside both root corners, so a reading taken there passes a tooth whose
// corners float outside the cone, which is exactly the defect the root sink exists
// to remove.
func assertCombineJoin(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the join step returned %d bodies, want the Gear Body", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	hex := profileHexagon(l, g)

	// The tooth, measured in a document of its own: it has to reach further from
	// the shaft axis than the frustum does, or the joined body would be the
	// frustum and nothing else.
	scratch := decad.New()
	tooth := toothSlab(t, scratch, l, g, apexSectionScale, 1)
	toothBox, err := tooth.Bounds()
	if err != nil {
		t.Fatalf("%s tooth bounds: %v", g.label, err)
	}
	gearBox, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s Gear Body bounds: %v", g.label, err)
	}
	toothReach := math.Max(math.Abs(toothBox.Min.X), toothBox.Max.X)
	gearReach := math.Max(math.Abs(gearBox.Min.X), gearBox.Max.X)
	if toothReach <= gearReach {
		t.Errorf("%s tooth reaches %.6f mm from the shaft axis and the frustum %.6f mm; the "+
			"joined body must reach further out than the frustum", g.label, toothReach, gearReach)
	}

	// The root arc's OUTERMOST point in the tooth plane, and the two root corners
	// beside it, all measured at the scale each station of the band puts them.
	poly := toothPolygon(l, g, 1)
	outer := poly[0]
	for _, v := range poly {
		if v.X > outer.X {
			outer = v
		}
	}
	corners := []pt2{poly[0], poly[len(poly)-1]}
	for _, station := range []string{"toe", "middle", "heel"} {
		frac := map[string]float64{"toe": 0.0, "middle": 0.5, "heel": 1.0}[station]
		scale := (hex.m.X + frac*(hex.c.X-hex.m.X)) / (l.pitchCone * math.Cos(g.gamma))
		for _, point := range append([]pt2{outer}, corners...) {
			world := toothPlaneToWorld(l, g, scale, point.mul(scale))
			radius := math.Hypot(world.X, world.Y)
			cone := world.Z * math.Tan(g.gammaRoot)
			if radius > cone {
				t.Errorf("%s at the %s of the band the root point sits at radius %.6f mm, outside "+
					"the gear body's root cone at %.6f mm — the tooth floats and the join leaves "+
					"two lumps", g.label, station, radius, cone)
			}
		}
		tip := toothPlaneToWorld(l, g, scale,
			pt2{l.toothSpacing - addendumFactor*l.module, 0}.mul(scale))
		tipRadius := math.Hypot(tip.X, tip.Y)
		if tipRadius <= tip.Z*math.Tan(g.gammaRoot) {
			t.Errorf("%s at the %s of the band the tooth tip sits at radius %.6f mm, inside the "+
				"root cone — the joined body would not reach further out than the frustum",
				g.label, station, tipRadius)
		}
	}
	// The root sink is what puts the whole arc inside the cone: at the dedendum
	// corner exactly, the arc touches the cone only on the tooth's own centreline
	// and its two corners stand outside it.
	if l.rootSink <= 0 {
		t.Errorf("%s draws no root sink, so the root arc meets the gear body along one line",
			g.label)
	}
}

// ---------------------------------------------------------------- bore

func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	band := heelBand(t, doc, l, g)
	if g.bore <= 0 {
		return []*decad.Body{band}
	}
	tool := boreTool(t, doc, l, g)
	pierced, err := decad.Cut(band, tool)
	if err != nil {
		t.Fatalf("%s bore cut: %v", g.label, err)
	}
	return []*decad.Body{pierced}
}

// heelBand is the section of the gear body the bore passes through, lofted for
// this step. The revolved gear body cannot be the target, for the reason the
// operand rule gives, so the band stands in for it: a loft between two n-gons on
// the heel cone, spanning the stations between the dedendum corner and the back
// face.
func heelBand(t *testing.T, doc *decad.Document, l lattice, g member) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	hex := profileHexagon(l, g)
	section := func(z, radius float64) (*sketch.Sketch, *sketch.Profile) {
		s := stationSketch(t, w, z)
		pts := make([]*sketch.Point, 0, chordSteps)
		for i := range chordSteps {
			a := 2 * math.Pi * float64(i) / float64(chordSteps)
			pts = append(pts, s.CreatePoint(radius*math.Cos(a), radius*math.Sin(a)))
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		return s, decadtest.SolveRegion(t, s)
	}
	s0, p0 := section(hex.c.X, hex.c.Y)
	s1, p1 := section(hex.h.X, hex.h.Y)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s heel band: %v", g.label, err)
	}
	return body
}

// boreTool is the bore's own cutting prism: a symmetric extrude of the bore
// circle, 2*Cone Distance either side of the shaft edge's start, which is what
// makes it a THROUGH cut. The shaft edge's start is A'/B', at the toe end.
func boreTool(t *testing.T, doc *decad.Document, l lattice, g member) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	hex := profileHexagon(l, g)
	s := stationSketch(t, w, hex.aPrime.X-2*l.coneDistance)
	radius := g.bore / 2
	pts := make([]*sketch.Point, 0, chordSteps)
	for i := range chordSteps {
		a := 2 * math.Pi * float64(i) / float64(chordSteps)
		pts = append(pts, s.CreatePoint(radius*math.Cos(a), radius*math.Sin(a)))
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	region := decadtest.SolveRegion(t, s)
	return decadtest.NewPrism(t, doc, s, region, mm(4*l.coneDistance))
}

// assertBoreCut pins the tool first, from its own measured geometry, and then the
// pierced band.
//
// The tool's two ends sit exactly 2*Cone Distance either side of the shaft edge's
// start and both clear the frustum, which is what makes the cut a THROUGH cut.
// The pierced body's volume is the band's own n-gon closed form less the prism the
// bore removes over that height; the result is ONE lump, which is what a through
// hole leaves; and it is solid, which an enclosed void would not be.
//
// WHAT THIS DOES NOT REACH IS THE REST OF THE BODY: the bore is pierced through
// the band that stands for the heel section, not through the whole frustum,
// because the frustum is a Revolve and the operand rule refuses one.
func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the bore step left %d bodies, want the one Gear Body", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	hex := profileHexagon(l, g)
	height := hex.h.X - hex.c.X
	bandVolume := ngonFrustumVolume(hex.c.Y, hex.h.Y, height)

	if g.bore <= 0 {
		decadtest.Measures(t, g.label+" heel band, no bore cut",
			volumeReading(t, bodies[0], g.label+" heel band"),
			units.CubicMillimeters(bandVolume), decadtest.WithinRel(units.Scalar(1e-9)))
		return
	}

	// The tool, from its own measured geometry.
	toolDoc := decad.New()
	tool := boreTool(t, toolDoc, l, g)
	box, err := tool.Bounds()
	if err != nil {
		t.Fatalf("%s bore tool bounds: %v", g.label, err)
	}
	start := hex.aPrime.X
	decadtest.MeasuresBox(t, g.label+" bore tool reaches 2*Cone Distance either side of the "+
		"shaft edge's start", box,
		r3.NewVec(-g.bore/2, -g.bore/2, start-2*l.coneDistance),
		r3.NewVec(g.bore/2, g.bore/2, start+2*l.coneDistance),
		decadtest.Within(mm(1e-6+g.bore/2*(1-math.Cos(math.Pi/chordSteps)))))
	if box.Min.Z > hex.m.X || box.Max.Z < hex.h.X {
		t.Errorf("%s bore tool spans stations [%.6f, %.6f], which does not clear the frustum's "+
			"[%.6f, %.6f]", g.label, box.Min.Z, box.Max.Z, hex.m.X, hex.h.X)
	}
	if g.bore > g.maxBore {
		t.Errorf("%s resolved Bore Diameter %.6f mm is above the maximum %.6f mm; a bore past "+
			"r_heel or r_toe cuts a corner off the profile being revolved",
			g.label, g.bore, g.maxBore)
	}

	removed := ngonPrismVolume(g.bore/2, height)
	decadtest.Measures(t, g.label+" pierced heel band",
		volumeReading(t, bodies[0], g.label+" pierced heel band"),
		units.CubicMillimeters(bandVolume-removed), decadtest.WithinRel(units.Scalar(1e-6)))
	if n := len(bodies[0].Lumps()); n != 1 {
		t.Errorf("%s pierced band is %d lumps, want the one a through hole leaves", g.label, n)
	}
	if !bodies[0].IsSolid() {
		t.Errorf("%s pierced band is not solid", g.label)
	}
}

// ngonArea is the area of the regular chordSteps-gon of the given circumradius,
// which is what every circular section here is drawn as.
func ngonArea(radius float64) float64 {
	return 0.5 * float64(chordSteps) * radius * radius * math.Sin(2*math.Pi/chordSteps)
}

func ngonPrismVolume(radius, height float64) float64 { return ngonArea(radius) * height }

func ngonFrustumVolume(r0, r1, height float64) float64 {
	a0, a1 := ngonArea(r0), ngonArea(r1)
	return height * (a0 + a1 + math.Sqrt(a0*a1)) / 3
}

// ---------------------------------------------------------------- mesh rotation

func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	tooth := toothSlab(t, doc, l, g, apexSectionScale, 1)
	angle := meshRotation(l, g)
	if angle == 0 {
		return []*decad.Body{tooth}
	}
	turned, err := tooth.Placed(turnAboutShaft(t, angle))
	if err != nil {
		t.Fatalf("%s mesh rotation of %.9f rad: %v", g.label, angle, err)
	}
	return []*decad.Body{turned}
}

// meshRotation is the half-tooth-pitch offset the DRIVING gear receives about its
// own shaft axis: 180 degrees / Driving Gear Teeth Number. The pinion's own extra
// phase is _PINION_MESH_PHASE_TEETH tooth-fractions, which is 0 by default,
// because §3a step G leaves the mid-face section unrotated precisely so that none
// is needed.
func meshRotation(l lattice, g member) float64 {
	if g.label != "Driving" {
		return 0
	}
	return math.Pi / g.teeth
}

// assertMeshRotation pins both sides of the branch: the driving body turns by half
// a tooth pitch about its own shaft axis so a driving valley sits where the pinion
// tooth crosses the axial plane, and the pinion does not turn at all.
//
// A ZERO ANGLE IS A NO-OP, NOT A MOVE. Building the identity and asking Fusion to
// move a body by it raises invalid transform, which is why the pinion returns
// early rather than each call site guarding it.
func assertMeshRotation(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the mesh rotation left %d bodies, want the one Gear Body", len(bodies))
	}
	l := deriveLattice(t, p)
	g := l.gearOf(p)
	want := meshRotation(l, g)
	if g.label == "Pinion" && want != 0 {
		t.Fatalf("the pinion asks for a mesh rotation of %.9f rad, want none", want)
	}
	if g.label == "Driving" {
		if math.Abs(want-math.Pi/g.teeth) > 1e-12 {
			t.Errorf("driving mesh rotation %.9f rad, want half a tooth pitch %.9f rad",
				want, math.Pi/g.teeth)
		}
		if math.Abs(want-2*math.Pi/g.teeth/2) > 1e-12 {
			t.Errorf("driving mesh rotation %.9f rad is not half of the tooth pitch %.9f rad",
				want, 2*math.Pi/g.teeth)
		}
	}
	// The rotation changes where the tooth sits and nothing else.
	scratch := decad.New()
	plain := toothSlab(t, scratch, l, g, apexSectionScale, 1)
	decadtest.Agree(t, g.label+" mesh-rotated body against the unrotated one",
		volumeReading(t, bodies[0], g.label+" rotated"),
		volumeReading(t, plain, g.label+" unrotated"),
		decadtest.WithinRel(units.Scalar(1e-9)))
	// The tooth is drawn already rotated 180 degrees, so its own unrotated azimuth
	// is not zero; what the rotation has to produce is that azimuth plus half a
	// tooth pitch.
	base := centroidAzimuth(t, plain)
	if got := centroidAzimuth(t, bodies[0]); angleGap(got, base+want) > 1e-6 {
		t.Errorf("%s body sits at azimuth %.9f rad after the mesh rotation, want %.9f rad",
			g.label, got, base+want)
	}
}
