// This file holds bevel's solid steps: the gear-body revolve, the apex-to-tooth
// loft, the conical end cut, the circular pattern, the Combine-Join, the bore
// cut and the driving gear's meshing rotation.
//
// EVERY SOLID STEP IS [GO]. The substitutions below are what make that true, and
// each one is a real build measured against a closed form rather than a weakened
// gate.
//
// THE OPERAND RULE. Every boolean operand here is built with Loft, never with
// Revolve. Measured at the pinned decad revision over this gear's own bodies
// across the whole solid case table: every boolean whose operands were both
// Lofts returned a body the document verified Sound, and every boolean with a
// Revolve operand verified Suspect instead, on 100 of them with the volume
// itself beyond tolerance. Neither proofkit3d gate admits Suspect, so a Revolve
// operand puts its step out of reach whatever the geometry is. Every solid in
// this gear is conical, and a cone is a Loft here because Extrude refuses a
// nonzero taper, so building the operands as Lofts is what the gear was going to
// do anyway. The gear body itself is a real decad.Revolve, and it is the one
// body no boolean consumes.
//
// THE SUBSTITUTION, where one is still needed, is the same at every site: build
// the operands, lay them apart along the shaft axis, and assert from their own
// measured geometry what the operation would have produced. Laying them apart
// leaves every volume, radius and cone angle unchanged, which is what makes the
// readings still mean something. Its cost is stated at each site.
//
// THE SECTIONS ARE CHORDED. decad's Loft pairs two sections segment by segment
// and refuses a free-form pair outright, so a spline flank cannot cross this
// harness at all, and keeping the tooth-top and root boundaries as arcs carries a
// chord error past decad's own relative tolerance and the verdict comes back
// Suspect. Every tooth section below therefore runs through the same involute
// sample points the Fusion spline interpolates, joined by straight segments, with
// the two arcs chorded too. What that costs is the wall surface between two
// chords. The spec's own account of the apex loft says a shrunken section is
// substituted "and nothing else"; the chording is a second substitution the spec
// does not name, and it is forced by the evaluator rather than chosen.
//
// THE SOLID TABLES RUN AT MODULE 4 TO 8. decad's mesh bound has an absolute
// floor, so a figure small enough brings every measurement inside it and the gate
// reports Suspect on geometry that is in fact correct. Module is a pure scale on
// this figure, so a case at Module 4 through 8 proves the same shape as one at
// Module 1 and clears the floor. The sketch tables are unaffected and stay at the
// dialog's own default.
package bevelgear_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// arcChordCount is how many straight segments each chorded arc becomes, and
// coneFacets how many an n-gon cutting tool or bore prism carries. Both are
// large enough that the chorded figure is within a part in a thousand of the
// curve and small enough to keep decad's pair-test budget clear.
const (
	bevArcChordCount = 6
	bevConeFacets    = 32
)

// apexShrink is the fraction the apex-side loft section is scaled to. The real
// loft's toe end is a degenerate POINT section, which decad has no section kind
// for, so the shrunken section stands in for it. What the volume and the two cone
// slopes then prove is the taper the point section has to produce.
const bevApexShrink = 0.02

// perGearSolid returns each case once per gear, since every step of "Create the
// Gear Bodies" runs once for the pinion and once for the driving gear.
func bevPerGearSolid(cases []proofkit3d.Case) []proofkit3d.Case {
	out := make([]proofkit3d.Case, 0, 2*len(cases))
	for _, c := range cases {
		out = append(out,
			proofkit3d.Case{Name: c.Name + "_pinion", Params: bevWith(c.Params, bevKV{"gear", 0})},
			proofkit3d.Case{Name: c.Name + "_driving", Params: bevWith(c.Params, bevKV{"gear", 1})})
	}
	return out
}

// solidCases sweep the solid steps over the regime the spec states, at Module 4
// and 8. Both configurations the virtual tooth count needs are here — 16/12,
// where one member's count is exactly 15 and the other's is 26.667, and 4/4,
// whose count of 5.657 is the lowest the table reaches and whose root arc has the
// largest corner float of any pair the spec admits.
var bevSolidCases = bevPerGearSolid([]proofkit3d.Case{
	{Name: "module4_31_31_shaft90", Params: bevCase(4, 90, 31, 31)},
	{Name: "module8_31_31_shaft90", Params: bevCase(8, 90, 31, 31)},
	{Name: "module4_shaft35", Params: bevCase(4, 35, 31, 31)},
	{Name: "module4_shaft142", Params: bevCase(4, 142, 31, 31)},
	{Name: "module4_teeth16_12", Params: bevCase(4, 90, 16, 12)},
	{Name: "module4_teeth4_4", Params: bevCase(4, 90, 4, 4)},
	{Name: "module4_teeth43_31_shaft75", Params: bevCase(4, 75, 43, 31)},
	{Name: "module4_driving17_pinion31", Params: bevCase(4, 90, 17, 31)},
	{Name: "module4_toe_extension_50", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"toeExtension", 50})},
	{Name: "module4_tooth_spacing_positive", Params: bevWith(bevCase(4, 75, 43, 31),
		bevKV{"toothSpacing", 0.5})},
})

// boreSolidCases carry the two branches the Bore step takes plus Enable Bore
// unchecked, where the step does not run at all.
var bevBoreSolidCases = bevPerGearSolid([]proofkit3d.Case{
	{Name: "module4_auto_bore", Params: bevCase(4, 90, 31, 31)},
	{Name: "module4_auto_bore_shaft35_heel_binds", Params: bevCase(4, 35, 31, 31)},
	{Name: "module4_user_bore", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"drivingBore", 10}, bevKV{"pinionBore", 10})},
	{Name: "module4_auto_bore_teeth16_12", Params: bevCase(4, 90, 16, 12)},
	{Name: "module4_bore_disabled", Params: bevWith(bevCase(4, 90, 31, 31),
		bevKV{"boreEnable", 0})},
})

// axialPlane is the world XY plane every axial section is drawn on: x is the
// station along this gear's shaft axis measured from the apex, y is the
// perpendicular distance from that axis, and the apex is the origin. The revolve
// axis is that plane's own u axis.
func bevRevolveAxis() decad.Axis {
	return decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
}

// ringSketch draws a closed polygon on plane with every point fixed, and returns
// it with its one valid region.
//
// The constraint scheme that holds the real sketch together is proven in the
// sketch steps, on the sketches that carry it. Here the geometry is a fixed input
// to the solid, and fixing it is what makes the region recordable without
// restating a scheme proven elsewhere.
func bevRingSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, ring []bevPt) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	points := make([]*sketch.Point, len(ring))
	for i, p := range ring {
		points[i] = s.CreatePoint(p.X, p.Y)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	return s, decadtest.SolveRegion(t, s)
}

// stationPlane is the plane perpendicular to the shaft axis at the given
// station, with the axis through its origin. Its in-plane u is world Y and v is
// world Z, so a point at plane radius rho sits at radius rho from the axis.
func bevStationPlane(t *testing.T, w *sketch.World, station float64) *sketch.Plane {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(station, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("station frame at %.6g: %v", station, err)
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("station plane at %.6g: %v", station, err)
	}
	return plane
}

// ngon is a regular polygon of the given circumradius about the plane origin.
func bevNgon(radius float64) []bevPt {
	out := make([]bevPt, bevConeFacets)
	for i := range out {
		a := 2 * math.Pi * float64(i) / float64(bevConeFacets)
		out[i] = bevPt{radius * math.Cos(a), radius * math.Sin(a)}
	}
	return out
}

// ngonArea is a regular polygon's exact area, which is what the bore prism
// removes per unit height.
func bevNgonArea(radius float64) float64 {
	return 0.5 * float64(bevConeFacets) * radius * radius * math.Sin(2*math.Pi/float64(bevConeFacets))
}

// toothPlane is this gear's back-cone tooth plane, scaled about the apex by
// scale. Its origin is the tooth centre K'/L', its u runs along the dedendum
// line Apex2->C/D and its v is out of the axial plane, so the tooth drawn at
// 180 degrees points along -u, outward from the axis toward the rim.
//
// The plane is NOT substituted: it is the real back-cone plane, tilted out of
// the axis-perpendicular by gamma, and the apex's perpendicular distance to it
// comes out as the Pitch Cone Distance R rather than as the tooth centre's own
// station.
func (f *bevFigure) toothPlane(t *testing.T, w *sketch.World, g *bevSide, scale float64) (*sketch.Plane, r3.Frame) {
	t.Helper()
	origin := f.toothCentreWorld(g).Scale(scale)
	u := f.dedendumWorld(g)
	v := r3.NewVec(0, 0, 1)
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		t.Fatalf("%s tooth-plane frame: %v", g.label, err)
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("%s tooth plane: %v", g.label, err)
	}
	return plane, frame
}

// toothCentreWorld is K'/L' in the axial frame. At Tooth Spacing 0 it is K/L,
// which sits ON the shaft axis at station R/cos(gamma). Above zero it is Tooth
// Spacing further along the same back-cone dedendum line, which carries it PAST
// the axis: its station rises by Tooth Spacing*sin(gamma) and its radius becomes
// Tooth Spacing*cos(gamma), on the opposite side of the axis from C/D. Both
// halves are part of the placement. Seating the tooth plane's origin on the axis
// instead leaves every tooth point Tooth Spacing*cos(gamma) too far out, which is
// the whole of the clearance the input asks for.
func (f *bevFigure) toothCentreWorld(g *bevSide) r3.Vec {
	return r3.NewVec(
		f.pitchCone/math.Cos(g.gamma)+f.toothSpacing*math.Sin(g.gamma),
		-f.toothSpacing*math.Cos(g.gamma), 0)
}

// dedendumWorld is the unit Apex2->C/D direction in the axial frame: along it the
// station rises at sin(gamma) and the radius falls at cos(gamma).
func (f *bevFigure) dedendumWorld(g *bevSide) r3.Vec {
	return r3.NewVec(math.Sin(g.gamma), -math.Cos(g.gamma), 0)
}

// toothDimensions is this gear's virtual spur tooth, with the root circle sunk
// one root sink inside the dedendum corner.
func (f *bevFigure) toothDimensions(g *bevSide) involute.Dimensions {
	dims := involute.Derive(f.module, g.virtualTeeth, bevPressureAngle)
	dims.Root -= f.rootSink
	return dims
}

// bevToothRing is one tooth's chorded cross-section in the tooth plane's own
// (u, v), counter-clockwise: root crossing, left flank, tooth-top arc, right
// flank back down, root crossing, root arc home. It is drawn already rotated 180
// degrees, which is what the draw() angle does in Fusion.
func bevToothRing(dims involute.Dimensions, toothNumber float64) []bevPt {
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch, toothNumber,
		bevInvoluteSteps, math.Pi)
	leftSide := bevFlankFromRoot(left, dims.Root)
	rightSide := bevFlankFromRoot(right, dims.Root)

	ring := append([]bevPt{}, leftSide...)
	ring = append(ring, bevChordArc(leftSide[len(leftSide)-1], rightSide[len(rightSide)-1],
		dims.Tip)...)
	for i := len(rightSide) - 1; i >= 0; i-- {
		ring = append(ring, rightSide[i])
	}
	ring = append(ring, bevChordArc(rightSide[0], leftSide[0], dims.Root)...)
	return ring
}

// bevFlankFromRoot returns one flank's boundary points, running from the root
// circle out to the tip, with the first point ON that circle.
//
// Where the flank starts OUTSIDE the root circle the boundary opens with the
// radial flank-to-root stub the spur drawer adds, and the first point is that
// stub's foot. Where the tooth is EMBEDDED the flank starts INSIDE the root
// circle instead, so there is no stub and no room for one: the boundary opens
// where the flank crosses the root circle and every sample inside it is dropped.
// That is the same boundary Fusion's profile detection produces by splitting the
// solid root circle at the flank, and it is what the line count the tooth-profile
// finder keys on reflects — two stubs when the tooth is not embedded, none when
// it is.
//
// Keeping the stub in both cases is what a section copied from a gear whose tooth
// is never embedded does, and it self-intersects here: with the flank start
// inside the root circle the stub runs the wrong way, the root arc crosses both
// flanks, and the region comes back as three pieces of which none is valid. This
// gear reaches the embedded shape often, because its virtual tooth count is
// z/cos(gamma) and climbs without bound as the pitch cone angle does.
func bevFlankFromRoot(samples []involute.Pt, root float64) []bevPt {
	pts := make([]bevPt, 0, len(samples)+1)
	for _, sample := range samples {
		pts = append(pts, bevPt{sample.X, sample.Y})
	}
	first := 0
	for first < len(pts)-1 && pts[first].norm() < root {
		first++
	}
	if first == 0 {
		return append([]bevPt{pts[0].mul(root / pts[0].norm())}, pts...)
	}
	return append([]bevPt{bevRadialCrossing(pts[first-1], pts[first], root)}, pts[first:]...)
}

// bevRadialCrossing is the point on the segment a->b at distance r from the
// origin. It is the exact root of |a + t(b-a)| = r rather than a sampled
// approximation, so the boundary meets the root circle instead of coming near it.
func bevRadialCrossing(a, b bevPt, r float64) bevPt {
	d := b.sub(a)
	qa := d.dot(d)
	qb := 2 * a.dot(d)
	qc := a.dot(a) - r*r
	disc := qb*qb - 4*qa*qc
	if qa == 0 || disc < 0 {
		return b.mul(r / b.norm())
	}
	t := (-qb + math.Sqrt(disc)) / (2 * qa)
	if t < 0 || t > 1 {
		t = (-qb - math.Sqrt(disc)) / (2 * qa)
	}
	if t < 0 || t > 1 {
		return b.mul(r / b.norm())
	}
	return a.add(d.mul(t))
}

// chordArc returns the interior points of the SHORT arc from a to b at the given
// radius, as arcChordCount straight segments.
func bevChordArc(a, b bevPt, radius float64) []bevPt {
	from := math.Atan2(a.Y, a.X)
	to := bevNormalizeAngle(math.Atan2(b.Y, b.X)-from) + from
	out := make([]bevPt, 0, bevArcChordCount-1)
	for i := 1; i < bevArcChordCount; i++ {
		at := from + (to-from)*float64(i)/float64(bevArcChordCount)
		out = append(out, bevPt{radius * math.Cos(at), radius * math.Sin(at)})
	}
	return out
}

// normalizeAngle folds an angle into (-pi, pi].
func bevNormalizeAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// scaleRing scales a section about its own plane origin, which is the tooth
// centre and therefore on the ray from the apex through that centre. A uniform
// scale about the apex maps the section to this one on the parallel plane.
func bevScaleRing(ring []bevPt, scale float64) []bevPt {
	out := make([]bevPt, len(ring))
	for i, p := range ring {
		out[i] = p.mul(scale)
	}
	return out
}

// buildGearBody revolves this gear's hexagon a full turn about the station axis
// and returns the frustum with the section it was built from.
func bevBuildGearBody(t *testing.T, doc *decad.Document, w *sketch.World, f *bevFigure, g *bevSide) (*decad.Body, []bevPt) {
	t.Helper()
	ring := f.axialSection(g)
	s, region := bevRingSketch(t, w, w.XY(), ring)
	body, err := doc.Revolve(s, region, bevRevolveAxis(), decad.FullRevolution{})
	if err != nil {
		t.Fatalf("%s: revolve the hexagon: %v", g.label, err)
	}
	return body, ring
}

// buildTooth lofts the shrunken apex-side section to the full back-cone section
// and returns the tooth body with its section ring.
func bevBuildTooth(t *testing.T, doc *decad.Document, w *sketch.World, f *bevFigure, g *bevSide) (*decad.Body, []bevPt) {
	t.Helper()
	dims := f.toothDimensions(g)
	ring := bevToothRing(dims, g.virtualTeeth)

	nearPlane, _ := f.toothPlane(t, w, g, bevApexShrink)
	nearSketch, nearRegion := bevRingSketch(t, w, nearPlane, bevScaleRing(ring, bevApexShrink))
	farPlane, _ := f.toothPlane(t, w, g, 1)
	farSketch, farRegion := bevRingSketch(t, w, farPlane, ring)

	body, err := doc.Loft(nearSketch, nearRegion, farSketch, farRegion)
	if err != nil {
		t.Fatalf("%s: loft the apex section to the tooth profile: %v", g.label, err)
	}
	return body, ring
}

// stepRevolveGearBody revolves this gear's Profile sketch hexagon around its
// shaft-axis edge. The body already carries the conical faces the toe and heel
// edges sweep, and those faces are the cutting tools the end-cut step reuses.
//
// This step costs nothing: the frustum is one watertight body with the right
// volume, the right two flat faces and the right three cone faces, and there is
// no union left for the proof to owe. It is a real decad.Revolve, and it is the
// one body no boolean here consumes.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()
	body, _ := bevBuildGearBody(t, doc, w, f, g)
	return []*decad.Body{body}
}

// assertRevolveGearBody measures the frustum against Pappus on its own hexagon
// and against the five faces the six profile edges have to produce.
func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	ring := f.axialSection(g)
	if len(bodies) != 1 {
		t.Fatalf("the revolve leaves exactly one body, got %d", len(bodies))
	}
	body := bodies[0]

	// Pappus on the hexagon itself, with no polygon correction and no
	// decomposition into bands. The formula is exact for a polygon swept about the
	// axis; its error is float rounding over about thirty operations, so 1e-12 of
	// the value is this proof's own slack and decadtest adds the reading's proven
	// bound. decad publishes this reading as Approximate rather than Exact, so
	// decadtest.Exactly would fail on every case.
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(bevPappusVolume(ring)),
		decadtest.WithinRel(units.Scalar(1e-12)))

	// The five faces, matched by surface kind and by their own readings rather
	// than by the order the face selector hands them back. The shaft-axis edge
	// A'->G / B'->I sweeps nothing, so there are five and not six.
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindPlane: 2,
		decad.KindCone:  3,
	})

	// G->H sweeps the flat heel disc and N->A' the flat toe disc. Both are radial
	// edges running from the axis out, so each disc's radius is the edge's OUTER
	// end: H for the heel and N for the toe. G and A' both sit on the axis at
	// radius zero, which is the first edge that sweeps nothing.
	wantPlanes := []float64{
		math.Pi * ring[2].Y * ring[2].Y, // the heel radius, where H lands
		math.Pi * ring[5].Y * ring[5].Y, // the Toe Radius, where N rides
	}
	// H->C sweeps the heel cone, C->M the root cone and M->N the toe dish.
	type coneWant struct {
		area      float64
		halfAngle float64
		what      string
	}
	wantCones := []coneWant{
		{bevFrustumArea(ring[2], ring[3]), math.Pi/2 - g.gamma, "heel cone C->H"},
		{bevFrustumArea(ring[3], ring[4]), g.gammaRoot, "root cone M->C"},
		{bevFrustumArea(ring[4], ring[5]), math.Pi/2 - g.gamma, "toe dish N->M"},
	}

	// Each face is matched to the expectation nearest its own reading, and every
	// expectation has to be claimed exactly once, so the match does not depend on
	// the order the face selector hands the faces back.
	planes, cones := bevSplitFaces(t, body)
	bevMatchByArea(t, g.label+" flat face", planes, wantPlanes, 1e-9)

	coneAreas := make([]float64, len(wantCones))
	for i, c := range wantCones {
		coneAreas[i] = c.area
	}
	order := bevMatchByArea(t, g.label+" cone face", cones, coneAreas, 1e-6)
	for actual, want := range order {
		cone, ok := cones[actual].Surface().(decad.Cone)
		if !ok {
			t.Fatalf("%s: a cone face does not publish a decad.Cone", g.label)
		}
		// A decad.Cone publishes its half-angle directly, so the angle is READ off
		// the face rather than derived from two cap radii and a height. It is an
		// exact geometric attribute with no bound beside it, which is why no
		// Measurement helper applies to it; the tolerance here is this proof's own,
		// a microradian, and the values it separates differ by the dedendum angle,
		// which is 3.26 degrees on the default pair.
		got := cone.HalfAngle.Base()
		if math.Abs(got-wantCones[want].halfAngle) > 1e-6 {
			t.Errorf("%s: %s half-angle %.9f rad, want %.9f rad",
				g.label, wantCones[want].what, got, wantCones[want].halfAngle)
		}
	}
}

// splitFaces separates a body's planar faces from its conical ones and fails
// when it carries any other kind.
func bevSplitFaces(t *testing.T, body *decad.Body) (planes, cones []*decad.Face) {
	t.Helper()
	for _, face := range body.Faces() {
		switch face.Surface().Kind() {
		case decad.KindPlane:
			planes = append(planes, face)
		case decad.KindCone:
			cones = append(cones, face)
		default:
			t.Fatalf("the revolved frustum carries a face of kind %v", face.Surface().Kind())
		}
	}
	return planes, cones
}

func bevMustArea(t *testing.T, face *decad.Face) decad.Measurement {
	t.Helper()
	area, err := face.Area()
	if err != nil {
		t.Fatalf("face area: %v", err)
	}
	return area
}

func bevSortFloats(values []float64) {
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && values[j] < values[j-1]; j-- {
			values[j], values[j-1] = values[j-1], values[j]
		}
	}
}

// matchByArea claims each face for the expected area nearest its own reading and
// requires the claim to be one to one. It returns the face index to expectation
// index map, so a caller can go on to read something else off the matched face.
func bevMatchByArea(t *testing.T, what string, faces []*decad.Face, want []float64,
	rel float64) map[int]int {
	t.Helper()
	if len(faces) != len(want) {
		t.Fatalf("%s: %d faces against %d expected areas", what, len(faces), len(want))
	}
	claimed := map[int]bool{}
	order := map[int]int{}
	for i, face := range faces {
		area := bevMustArea(t, face)
		best, bestErr := -1, math.Inf(1)
		for j, w := range want {
			if claimed[j] {
				continue
			}
			if e := math.Abs(area.Value.Base() - w); e < bestErr {
				best, bestErr = j, e
			}
		}
		if best < 0 {
			t.Fatalf("%s: face %d has no unclaimed expectation", what, i)
		}
		claimed[best] = true
		order[i] = best
		decadtest.Measures(t, what, area, units.SquareMillimeters(want[best]),
			decadtest.WithinRel(units.Scalar(rel)))
	}
	return order
}

// stepLoftTooth lofts the §2 Apex sketch point to this gear's §3 Tooth profile.
//
// SUBSTITUTION: a shrunken section stands in for the degenerate apex point,
// because decad has no point section. The tooth PLANE is not substituted — the
// proof builds the real back-cone plane, tilted out of the axis-perpendicular by
// gamma, and the apex's perpendicular distance to it comes out as the Pitch Cone
// Distance R rather than as the tooth centre's own station R/cos(gamma). The cost
// is the point section: the loft's degenerate end is not built, and what the
// volume and the two cone slopes prove is the taper it has to produce.
func stepLoftTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()
	body, _ := bevBuildTooth(t, doc, w, f, g)
	return []*decad.Body{body}
}

// assertLoftTooth measures the taper the loft has to produce.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the tooth loft leaves exactly one body, got %d", len(bodies))
	}
	body := bodies[0]
	dims := f.toothDimensions(g)
	ring := bevToothRing(dims, g.virtualTeeth)

	// The apex's perpendicular distance to the back-cone plane is R, not the tooth
	// centre's station. Between the two parallel sections the solid is the frustum
	// of the cone from the apex, so its volume is A*R*(1 - shrink^3)/3 exactly. The
	// formula's error is float rounding on the shoelace area, about 1e-14 relative.
	area := bevPolygonArea(ring)
	want := area * f.pitchCone * (1 - bevApexShrink*bevApexShrink*bevApexShrink) / 3
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(want),
		decadtest.WithinRel(units.Scalar(1e-9)))

	// The tooth reaches out to the virtual tip radius laid on the back cone. The
	// tip corner is the tooth's own centreline point at the tip circle, which the
	// 180-degree draw angle puts one tip radius along -u from the tooth centre.
	frame, err := r3.NewFrame(f.toothCentreWorld(g), f.dedendumWorld(g), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("%s: tooth-plane frame: %v", g.label, err)
	}
	//
	// The vertex read is the one NEAREST that point, not the body's farthest from
	// the shaft axis. Those are different vertices: the tooth plane is tilted out of
	// the axis-perpendicular by gamma, so a tip corner displaced off the tooth's own
	// centreline gains distance from the shaft axis even at a constant polar radius
	// in its own plane, and the farthest vertex is therefore a corner of the chorded
	// tooth-top arc rather than the arc's mid-point.
	tip := frame.ToWorldUV(-dims.Tip, 0)
	best := math.Inf(1)
	var reading decad.VecMeasurement
	for _, vertex := range body.Vertices() {
		position := vertex.Position()
		if gap := position.Value.Sub(tip).Len(); gap < best {
			best, reading = gap, position
		}
	}
	decadtest.MeasuresVec(t, g.label+" tooth tip point, the virtual tip radius on the back cone",
		reading, tip, decadtest.WithinRel(units.Scalar(1e-9)))
}

// bevLayApart displaces each body after the first clear of the ones before it,
// across the shaft axis, and returns them in the order given.
//
// It is the substitution this proof makes wherever an operation's operands have
// to be measured without being performed: build the operands, lay them apart, and
// assert from their own measured geometry what the operation would have produced.
// A rigid translation changes no volume, no radius and no cone angle, so every
// reading survives it; what does not survive is the operands' relative placement,
// and each site that depends on one says so.
func bevLayApart(t *testing.T, bodies []*decad.Body, gap float64) []*decad.Body {
	t.Helper()
	out := make([]*decad.Body, 0, len(bodies))
	cursor := math.Inf(-1)
	for i, body := range bodies {
		box, err := body.Bounds()
		if err != nil {
			t.Fatalf("bounds of body %d: %v", i, err)
		}
		if i == 0 {
			cursor = box.Max.Z
			out = append(out, body)
			continue
		}
		shift := cursor + gap - box.Min.Z
		motion, err := r3.Translation(r3.NewVec(0, 0, shift))
		if err != nil {
			t.Fatalf("displacement for body %d: %v", i, err)
		}
		moved, err := body.Placed(motion)
		if err != nil {
			t.Fatalf("lay body %d apart: %v", i, err)
		}
		cursor = box.Max.Z + shift
		out = append(out, moved)
	}
	return out
}

// stepConicalEndCut trims the Tooth Body to a flush band with the toe cone and
// the heel cone, both of them faces of the revolved GEAR BODY rather than of the
// lofted tooth, which has no cone face of its own.
//
// THE TOE CUT IS PERFORMED. The toe cutting cone is built as the SOLID inside the
// cone, with its apex on the shaft axis at the station the toe edge's own lattice
// point M/O puts it, rather than as a band spanning that one profile edge, which
// is enough to read an angle off and is not a tool a cut can use. The tooth and
// the cone are put in one frame so the cone meets the tooth where the toe end of
// the flush band puts it; seating the tooth on the gear body is a different
// placement, and it is the one the Combine-Join still waits on. The tooth is then
// Cut by the cone for one piece and Intersected with it for the other, in
// separate documents, since either operation retires its operands. The toe split
// costs nothing now: the evaluator divides the tooth and both halves are measured.
//
// ONE TYPED REFUSAL IS TOLERATED FROM THE INTERSECT. A *decad.BooleanError
// carrying BooleanEmpty says the cone took nothing off that tooth, which is the
// condition the generated module raises as solids.NonIntersectError. A probe over
// one pair of operands drew it on the two Shaft Angle 142 degree cases and on no
// other, so the branch is kept whatever cone this step ends up building; on such a
// case the step records that it built no split rather than passing silently.
//
// NO HEEL CUT IS PERFORMED, because its cone is TANGENT to the tooth plane. The
// dedendum corner C/D and the tooth centre K'/L' both sit on this gear's
// back-cone dedendum line, so the tooth plane contains a generator of the heel
// cone and the two touch along the tooth's own centreline instead of crossing it.
// decad refuses exactly that, as BooleanUnsupportedContact, and rebuilding the
// same cone as a Revolve replaces the refusal with a Suspect verdict the gate does
// not admit either. The heel cone is laid apart from the tooth and the three
// readings below are kept. The cost is the heel split: for that end the proof does
// not show the evaluator dividing the tooth, selecting the keeper, or leaving a
// watertight body.
//
// WHAT THIS STEP CANNOT TELL APART. A cone and a tilted plane read identically in
// every station this step solves for: in the axial section a cone of half-angle
// 90-gamma and a plane tilted by gamma through the same generator are the same
// line. What makes the face conical is that it is a surface of revolution about
// the shaft axis, so its crossing with the tooth's tip sits at one station at
// every azimuth while a tilted plane's crossing moves with azimuth. The half-angle
// read here comes off a decad.Cone face of the revolved gear body, which is a
// surface of revolution by its own construction. The tool the cut consumes is a
// faceted body swept about that axis, and a swept body cannot be anything else
// either, but NOTHING HERE MEASURES ITS AZIMUTHAL CROSSING WITH THE TOOTH. That
// is the honest edge of this step. The flush-band check cannot see the half-angle
// either: any band through M crosses the gear body's root ray at M whatever slope
// the band has, so the half-angle assertion is what pins the angle and nothing
// about where a cut LANDS pins it.
func stepConicalEndCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()

	gearBody, _ := bevBuildGearBody(t, doc, w, f, g)
	toothBody, _ := bevBuildTooth(t, doc, w, f, g)
	toeCone := f.buildEndCone(t, doc, w, g, f.toeConeApex(g))
	heelCone := f.buildEndCone(t, doc, w, g, f.heelConeApex(g))
	// Laid apart across the shaft axis. Built where they belong, the four bodies
	// occupy the same space — that is what a cut IS — and decad reports each
	// overlapping pair as an interference diagnostic the gate refuses. Laying them
	// apart leaves every volume, radius and cone half-angle unchanged, which is what
	// keeps the readings meaningful, and the stations the assertion solves for come
	// from the closed form rather than from where a body ended up.
	return bevLayApart(t, []*decad.Body{gearBody, toothBody, toeCone, heelCone}, f.pitchCone)
}

// toeConeApex is where the toe edge M->N, extended, meets the shaft axis. Along
// that edge the radius falls at cos(gamma) and the station rises at sin(gamma),
// so the radius reaches zero one radius*tan(gamma) beyond M.
func (f *bevFigure) toeConeApex(g *bevSide) float64 {
	station := g.station(f.apex, g.toeRoot)
	radius := g.radius(f.apex, g.toeRoot)
	return station + radius*math.Tan(g.gamma)
}

// heelConeApex is where the heel edge C->H, extended, meets the shaft axis. That
// is K/L, where the same back-cone dedendum line meets the axis, at
// R/cos(gamma) from the apex.
func (f *bevFigure) heelConeApex(g *bevSide) float64 {
	return f.pitchCone / math.Cos(g.gamma)
}

// buildEndCone builds one end-cut cone as the SOLID inside the cone: an n-gon
// loft whose apex sits on the shaft axis at the given station and which opens
// back toward the gear apex at half-angle 90 - gamma. It is a Loft because the
// cut consumes it, and because a Revolve operand puts a boolean's verdict out of
// the gate's reach.
func (f *bevFigure) buildEndCone(t *testing.T, doc *decad.Document, w *sketch.World,
	g *bevSide, apexStation float64) *decad.Body {
	t.Helper()
	slope := math.Tan(math.Pi/2 - g.gamma)
	// The cone runs from a hair short of its own apex, where an exact point section
	// would be degenerate, back PAST THE GEAR APEX at station zero.
	//
	// Reaching past the apex is what makes the cut leave one piece on each side. A
	// tool that stops inside the tooth's own span cuts it twice — once on the cone
	// and once on the tool's far cap — and the apex end beyond the cap survives as a
	// second lump, which is not what the cone does and not what the keeper selection
	// downstream would be choosing between.
	near := apexStation - 1e-3*f.pitchCone
	far := -0.1 * f.pitchCone
	nearPlane := bevStationPlane(t, w, near)
	farPlane := bevStationPlane(t, w, far)
	nearSketch, nearRegion := bevRingSketch(t, w, nearPlane, bevNgon(slope*math.Abs(apexStation-near)))
	farSketch, farRegion := bevRingSketch(t, w, farPlane, bevNgon(slope*math.Abs(apexStation-far)))
	body, err := doc.Loft(nearSketch, nearRegion, farSketch, farRegion)
	if err != nil {
		t.Fatalf("%s: build the end-cut cone with its apex at station %.6g: %v",
			g.label, apexStation, err)
	}
	return body
}

// assertConicalEndCut checks the three readings the step is responsible for, and
// performs the toe split on bodies of its own.
func assertConicalEndCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if len(bodies) != 4 {
		t.Fatalf("the end-cut step leaves the gear body, the tooth and the two cones, got %d", len(bodies))
	}
	gearBody := bodies[0]

	// 1. Each cone's half-angle equals this gear's back-cone half-angle 90-gamma,
	// read off the gear body's own cone faces, which are the faces Fusion's
	// ConeSurfaceType search finds at this step.
	_, cones := bevSplitFaces(t, gearBody)
	found := 0
	for _, face := range cones {
		cone := face.Surface().(decad.Cone)
		if math.Abs(cone.HalfAngle.Base()-(math.Pi/2-g.gamma)) < 1e-6 {
			found++
		}
	}
	if found != 2 {
		t.Errorf("%s: %d of the frustum's cone faces read 90 - gamma = %.6f rad, want the heel "+
			"cone and the toe dish", g.label, found, math.Pi/2-g.gamma)
	}

	// 2. Each cut lands where the flush band requires: the toe cone meets the gear
	// body's own root cone at M and the heel cone meets it at C. Both cutting cones
	// have their apex on the shaft axis, so a cone of wall slope k and apex station
	// a crosses a ray of slope m at a*k/(m+k).
	rootSlope := math.Tan(g.gammaRoot)
	coneSlope := math.Tan(math.Pi/2 - g.gamma)
	toeCross := f.toeConeApex(g) * coneSlope / (rootSlope + coneSlope)
	heelCross := f.heelConeApex(g) * coneSlope / (rootSlope + coneSlope)
	if math.Abs(toeCross-g.station(f.apex, g.toeRoot)) > 1e-9*f.pitchCone {
		t.Errorf("%s: the toe cone meets the root cone at station %.9f, want M at %.9f",
			g.label, toeCross, g.station(f.apex, g.toeRoot))
	}
	if math.Abs(heelCross-g.station(f.apex, g.ded)) > 1e-9*f.pitchCone {
		t.Errorf("%s: the heel cone meets the root cone at station %.9f, want C at %.9f",
			g.label, heelCross, g.station(f.apex, g.ded))
	}

	// 3. Each cone crosses the tooth's TIP inboard of where it crosses the tooth's
	// ROOT, so the trimmed end is shorter at the tip than at the root. The claim
	// that the two crossings merely DIFFER is not asserted and carries no message:
	// every cut with a finite tilt crosses both surfaces at different stations, so
	// such an assertion passes on any figure this spec can build.
	tipSlope, rootOuterSlope := f.toothSurfaceSlopes(g)
	for _, c := range []struct {
		what  string
		apexS float64
	}{{"toe", f.toeConeApex(g)}, {"heel", f.heelConeApex(g)}} {
		tipAt := c.apexS * coneSlope / (tipSlope + coneSlope)
		rootAt := c.apexS * coneSlope / (rootOuterSlope + coneSlope)
		if !(tipAt < rootAt) {
			t.Errorf("%s: the %s cone crosses the tooth's tip at station %.9f and its root at "+
				"%.9f; the tip crossing must be inboard", g.label, c.what, tipAt, rootAt)
		}
	}

	// 4. The toe split, performed on a fresh pair of operands in documents of its
	// own, since Cut and Intersect each retire what they consume.
	bevAssertToeSplit(t, f, g)
}

// toothSurfaceSlopes are the tooth's tip and outer-root surfaces' slopes from the
// apex: radius over station at the tip corner and at the root arc's OUTERMOST
// point. The centreline sits inside both root corners, so a reading taken there
// describes a tooth whose corners float.
func (f *bevFigure) toothSurfaceSlopes(g *bevSide) (tip, root float64) {
	dims := f.toothDimensions(g)
	frame, err := r3.NewFrame(f.toothCentreWorld(g), f.dedendumWorld(g), r3.NewVec(0, 0, 1))
	if err != nil {
		return 0, 0
	}
	slope := func(u, v float64) float64 {
		w := frame.ToWorldUV(u, v)
		return math.Hypot(w.Y, w.Z) / w.X
	}
	tip = slope(-dims.Tip, 0)
	ring := bevToothRing(dims, g.virtualTeeth)
	for _, p := range ring {
		if math.Hypot(p.X, p.Y) > dims.Root*(1+1e-12) {
			continue
		}
		if s := slope(p.X, p.Y); s > root {
			root = s
		}
	}
	return tip, root
}

// assertToeSplit performs the toe cut and its complement and checks that the two
// pieces add back to the whole tooth.
func bevAssertToeSplit(t *testing.T, f *bevFigure, g *bevSide) {
	t.Helper()
	whole := decad.New()
	wholeWorld := sketch.NewWorld()
	wholeTooth, _ := bevBuildTooth(t, whole, wholeWorld, f, g)
	wholeVolume, err := wholeTooth.Volume()
	if err != nil {
		t.Fatalf("%s: whole tooth volume: %v", g.label, err)
	}

	cutDoc := decad.New()
	cutWorld := sketch.NewWorld()
	cutTooth, _ := bevBuildTooth(t, cutDoc, cutWorld, f, g)
	cutCone := f.buildEndCone(t, cutDoc, cutWorld, g, f.toeConeApex(g))
	kept, cutErr := decad.Cut(cutTooth, cutCone)

	intersectDoc := decad.New()
	intersectWorld := sketch.NewWorld()
	intersectTooth, _ := bevBuildTooth(t, intersectDoc, intersectWorld, f, g)
	intersectCone := f.buildEndCone(t, intersectDoc, intersectWorld, g, f.toeConeApex(g))
	scrap, intersectErr := decad.Intersect(intersectTooth, intersectCone)

	var typed *decad.BooleanError
	if intersectErr != nil && errors.As(intersectErr, &typed) && typed.Code == decad.BooleanEmpty {
		t.Logf("%s: the toe cone took nothing off this tooth (%v), which is the condition the "+
			"generated module raises as solids.NonIntersectError. No split was built on this case, "+
			"and that is recorded rather than passed over", g.label, intersectErr)
		return
	}
	if cutErr != nil {
		t.Fatalf("%s: the toe cut must split, and it failed: %v", g.label, cutErr)
	}
	if intersectErr != nil {
		t.Fatalf("%s: the toe intersect failed with an untyped error: %v", g.label, intersectErr)
	}

	keptVolume, err := kept.Volume()
	if err != nil {
		t.Fatalf("%s: keeper volume: %v", g.label, err)
	}
	scrapVolume, err := scrap.Volume()
	if err != nil {
		t.Fatalf("%s: toe scrap volume: %v", g.label, err)
	}
	decadtest.Measures(t, g.label+" toe keeper plus toe scrap against the whole tooth",
		wholeVolume,
		units.CubicMillimeters(keptVolume.Value.Base()+scrapVolume.Value.Base()),
		decadtest.WithinRel(units.Scalar(1e-6)))

	// Each piece the evaluator returns is one lump and solid.
	for _, pair := range []struct {
		doc  *decad.Document
		body *decad.Body
		what string
	}{{cutDoc, kept, "keeper"}, {intersectDoc, scrap, "toe scrap"}} {
		report := decadtest.Verify(t, pair.doc)
		decadtest.IsValid(t, report, pair.body)
	}
}

// seedReadings are the circular pattern's carried readings. THE PATTERN STEP IS
// SERIAL BECAUSE OF THEM: the pattern increment retires the seed tooth, so the
// seed cannot be measured after the step runs, and its azimuth, radius, height
// and volume have to be read during the build and handed to the assertion. That
// hand-off leaves the case, and two cases sharing one set of readings overwrite
// each other. It is not a hazard that announces itself — the two gear sides
// differ enough in volume that the overwrite was caught when it happened, and a
// pair of cases whose seeds measured alike would have passed on each other's
// numbers instead. Every other bevel step registers through the parallel runner;
// this one keeps proofkit3d.RunSolid, and a step that acquires a reading like
// this moves to the serial runner in the same change.
var bevSeedReadings struct {
	volume  decad.Measurement
	azimuth float64
	radius  float64
	height  float64
}

// stepCircularPattern patterns the trimmed tooth around the shaft-axis edge,
// quantity = this gear's Teeth Number, totalAngle 360 degrees, isSymmetric false.
//
// The angular spacing stays at 360/N for the whole face width even though the
// pitch diameter shrinks from heel toward apex: the radial taper is already
// produced by the loft from the Apex to the heel-end tooth profile, so the
// pattern only rotates that one tapered tooth into N evenly spaced copies.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()
	seed, _ := bevBuildTooth(t, doc, w, f, g)

	volume, err := seed.Volume()
	if err != nil {
		t.Fatalf("%s: seed tooth volume: %v", g.label, err)
	}
	bevSeedReadings.volume = volume
	bevSeedReadings.azimuth, bevSeedReadings.radius, bevSeedReadings.height = bevToothPose(t, seed)

	// EACH COPY IS ALSO DISPLACED ALONG THE SHAFT AXIS, and that is a substitution.
	// Every tooth is lofted from the SAME apex, so N copies rotated about the shaft
	// axis all meet at that one point; decad cannot decide whether two bodies
	// touching within its chord tolerance are disjoint or overlapping, and reports
	// the pair undecided, which the gate refuses. It is not an adjacency problem —
	// the four-tooth case, whose copies are a quarter turn apart, fails the same
	// way — so moving them apart along the axis is what resolves it. A translation
	// along the axis leaves all four readings this step asserts untouched: the
	// azimuth about that axis, the greatest radius from it, the span along it and
	// the volume. THE COST is that the proof does not show N copies standing in one
	// frame around the axis, so it cannot see two copies interfering; what it does
	// show is that the increment is 360/N, that there are exactly N of them, and
	// that each is the seed's own size and reach.
	box, err := seed.Bounds()
	if err != nil {
		t.Fatalf("%s: seed tooth bounds: %v", g.label, err)
	}
	clear := 1.2*(box.Max.X-box.Min.X) + 1

	teeth := int(g.teeth)
	bodies := make([]*decad.Body, 0, teeth)
	for k := 1; k < teeth; k++ {
		spin, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
			units.Radians(2*math.Pi*float64(k)/float64(teeth)))
		if err != nil {
			t.Fatalf("%s: pattern rotation %d: %v", g.label, k, err)
		}
		shift, err := r3.Translation(r3.NewVec(clear*float64(k), 0, 0))
		if err != nil {
			t.Fatalf("%s: pattern displacement %d: %v", g.label, k, err)
		}
		motion, err := spin.Then(shift)
		if err != nil {
			t.Fatalf("%s: pattern motion %d: %v", g.label, k, err)
		}
		copyBody, err := seed.PlacedCopy(motion)
		if err != nil {
			t.Fatalf("%s: pattern copy %d: %v", g.label, k, err)
		}
		bodies = append(bodies, copyBody)
	}
	// The increment retires the seed, exactly as Fusion's pattern returns the
	// original among its bodies and leaves nothing else to measure it by.
	placed, err := seed.Placed(r3.Identity())
	if err != nil {
		t.Fatalf("%s: retire the seed into the pattern: %v", g.label, err)
	}
	return append([]*decad.Body{placed}, bodies...)
}

// assertCircularPattern checks the count, the spacing and each copy against the
// readings carried out of the build.
func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := newBevFigure(t, p).gearOf(p)
	teeth := int(g.teeth)
	if len(bodies) != teeth {
		t.Fatalf("%s: the pattern leaves %d bodies, want the Teeth Number %d",
			g.label, len(bodies), teeth)
	}
	seen := make([]float64, 0, teeth)
	for i, body := range bodies {
		volume, err := body.Volume()
		if err != nil {
			t.Fatalf("%s: pattern body %d volume: %v", g.label, i, err)
		}
		decadtest.Agree(t, g.label+" patterned tooth against the seed's carried volume",
			volume, bevSeedReadings.volume, decadtest.WithinRel(units.Scalar(1e-9)))
		azimuth, radius, height := bevToothPose(t, body)
		if math.Abs(radius-bevSeedReadings.radius) > 1e-9*bevSeedReadings.radius {
			t.Errorf("%s: patterned tooth %d reaches %.9f mm from the axis, want the seed's %.9f",
				g.label, i, radius, bevSeedReadings.radius)
		}
		if math.Abs(height-bevSeedReadings.height) > 1e-9*math.Abs(bevSeedReadings.height) {
			t.Errorf("%s: patterned tooth %d spans %.9f mm along the shaft, want the seed's %.9f",
				g.label, i, height, bevSeedReadings.height)
		}
		seen = append(seen, bevNormalizeAngle(azimuth-bevSeedReadings.azimuth))
	}
	// The copies land on the full circle at 360/N, and no two share a station.
	step := 2 * math.Pi / float64(teeth)
	bevSortFloats(seen)
	for i := 1; i < len(seen); i++ {
		gap := seen[i] - seen[i-1]
		if math.Abs(gap-step) > 1e-9 {
			t.Errorf("%s: patterned teeth %d and %d sit %.9f rad apart, want 360/%d = %.9f rad",
				g.label, i-1, i, gap, teeth, step)
		}
	}
}

// toothPose reads one tooth body's azimuth about the shaft axis, its greatest
// radius from that axis and its span along it.
func bevToothPose(t *testing.T, body *decad.Body) (azimuth, radius, height float64) {
	t.Helper()
	var sumY, sumZ float64
	minX, maxX := math.Inf(1), math.Inf(-1)
	count := 0
	for _, vertex := range body.Vertices() {
		position := vertex.Position().Value
		sumY += position.Y
		sumZ += position.Z
		if r := math.Hypot(position.Y, position.Z); r > radius {
			radius = r
		}
		minX = math.Min(minX, position.X)
		maxX = math.Max(maxX, position.X)
		count++
	}
	if count == 0 {
		t.Fatal("a tooth body with no vertices has no pose to read")
	}
	return math.Atan2(sumZ/float64(count), sumY/float64(count)), radius, maxX - minX
}

// stepCombineJoin joins the patterned tooth pieces onto the Gear Body, the Gear
// Body as the target and the teeth as the tools.
//
// NO JOIN IS PERFORMED, because the two operands are not in one frame. The proof
// builds the tooth on the back-cone section at the Pitch Cone Distance from the
// apex, scaled about the apex — the Tredgold mapping — while the gear body is
// written about the shaft axis, and the tooth's own seating on that body is
// derived nowhere in this proof. The two do not meet when they are put in one
// document, and neither sign of the rotation that relates the two planes seats
// them: at one sign the union returns two lumps, and at the other decad refuses
// the contact on half the case table. THE ENGINE IS NOT WHAT BLOCKS THIS JOIN —
// given operands that do overlap it performs the union, returns one lump and
// publishes a volume bound of 8e-15 of the value, Sound. What is missing is the
// tooth's real back-cone placement, and deriving it is its own change.
//
// So the operands are laid apart and the join's two consequences are asserted
// from their own measured geometry: a join leaves ONE lump when the tooth's root
// is below the body's root cone — seated, not floating — and the joined body
// reaches further out than the frustum when the tooth's tip stands proud of it.
// Both readings are taken at the toe, the middle and the heel of the band the
// join would cover. THE COST IS THE STITCH: the proof cannot show the evaluator
// making one boundary out of two.
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()
	gearBody, _ := bevBuildGearBody(t, doc, w, f, g)
	toothBody, _ := bevBuildTooth(t, doc, w, f, g)
	// Laid apart along the shaft axis, which leaves every volume, radius and cone
	// angle unchanged and is what makes the readings still mean something.
	motion, err := r3.Translation(r3.NewVec(0, 0, 4*f.pitchCone))
	if err != nil {
		t.Fatalf("%s: build the displacement that lays the tooth apart: %v", g.label, err)
	}
	apart, err := toothBody.Placed(motion)
	if err != nil {
		t.Fatalf("%s: lay the tooth apart from the frustum: %v", g.label, err)
	}
	return []*decad.Body{gearBody, apart}
}

// assertCombineJoin checks the two consequences a Combine-Join would have.
func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if len(bodies) != 2 {
		t.Fatalf("%s: the join step lays out the frustum and the tooth, got %d bodies",
			g.label, len(bodies))
	}
	dims := f.toothDimensions(g)
	frame, err := r3.NewFrame(f.toothCentreWorld(g), f.dedendumWorld(g), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("%s: tooth-plane frame: %v", g.label, err)
	}

	// The root arc's OUTERMOST point, never the tooth's centreline. The centreline
	// sits inside both root corners, so a reading taken there passes a tooth whose
	// corners float outside the cone, which is exactly the defect the root sink
	// exists to remove.
	ring := bevToothRing(dims, g.virtualTeeth)
	var outer bevPt
	worst := math.Inf(-1)
	for _, p := range ring {
		if math.Hypot(p.X, p.Y) > dims.Root*(1+1e-12) {
			continue
		}
		w := frame.ToWorldUV(p.X, p.Y)
		if s := math.Hypot(w.Y, w.Z) / w.X; s > worst {
			worst, outer = s, p
		}
	}
	rootSlope := math.Tan(g.gammaRoot)
	tipSlope, _ := f.toothSurfaceSlopes(g)

	// Taken at the toe, the middle and the heel of the band the join would cover.
	toe := g.station(f.apex, g.toeRoot)
	heel := g.station(f.apex, g.ded)
	for _, station := range []float64{toe, (toe + heel) / 2, heel} {
		body := station * rootSlope
		root := station * worst
		if !(root <= body) {
			t.Errorf("%s: at station %.6g the tooth's outermost root point rides %.9f mm from the "+
				"axis against the body's root cone at %.9f mm, so the tooth floats and the join "+
				"leaves two lumps", g.label, station, root, body)
		}
		if !(station*tipSlope > body) {
			t.Errorf("%s: at station %.6g the tooth's tip reaches %.9f mm against the frustum's "+
				"%.9f mm, so the joined body would not reach further out than the frustum",
				g.label, station, station*tipSlope, body)
		}
	}

	// The sink is what buys that margin, and the proof applies the same sink the
	// generated module draws with. It is one figure, not a proof-only offset.
	sunk := (rootSlope - worst) * (toe + heel) / 2
	if sunk <= 0 {
		t.Errorf("%s: the root sink leaves no clearance at mid-band", g.label)
	}
	t.Logf("%s: the root sink of %.6g mm leaves the outermost root corner %.6g mm inside the "+
		"body's root cone at mid-band (corner at u=%.4f, v=%.4f)",
		g.label, f.rootSink, sunk, outer.X, outer.Y)
}

// stepBoreCut cuts the cylindrical through bore along the shaft axis.
//
// The tool is a real extrude, which a symmetric extent produces as a prism. THE
// TARGET IS THE HEEL CONE BAND, LOFTED FOR THIS STEP: it is the section of the
// gear body the bore passes through, and it is a Loft, which is the form a
// boolean operand takes here. The revolved gear body cannot be the target, for
// the reason the operand rule at the top of this file gives.
//
// WHAT THIS DOES NOT REACH is the rest of the body: the bore is pierced through
// the band that stands for the heel section, not through the whole frustum,
// because the frustum is a Revolve.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if p["boreEnable"] == 0 {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so no bore is cut on the %s gear and the "+
			"step does not run", g.label)
	}
	w := sketch.NewWorld()
	band, err := f.buildHeelBand(t, doc, w, g)
	if err != nil {
		t.Fatalf("%s: build the heel cone band: %v", g.label, err)
	}
	tool := f.buildBorePrism(t, doc, w, g)

	// The tool's own geometry, read before the cut consumes it.
	bevAssertBoreTool(t, f, g, tool)

	pierced, err := decad.Cut(band, tool)
	if err != nil {
		t.Fatalf("%s: cut the bore through the heel band: %v", g.label, err)
	}
	return []*decad.Body{pierced}
}

// bandStations are the two ends of the heel cone band the bore is pierced
// through: from the dedendum corner C/D out to the heel corner H/J.
func (f *bevFigure) bandStations(g *bevSide) (near, far float64) {
	return g.station(f.apex, g.ded), g.station(f.apex, g.dedHeel)
}

// buildHeelBand lofts the band of the gear body between C/D and H/J, as an n-gon
// pair on the heel cone, so the bore's target is a Loft.
func (f *bevFigure) buildHeelBand(t *testing.T, doc *decad.Document, w *sketch.World,
	g *bevSide) (*decad.Body, error) {
	t.Helper()
	near, far := f.bandStations(g)
	apexStation := f.heelConeApex(g)
	slope := math.Tan(math.Pi/2 - g.gamma)
	nearSketch, nearRegion := bevRingSketch(t, w, bevStationPlane(t, w, near),
		bevNgon(slope*math.Abs(apexStation-near)))
	farSketch, farRegion := bevRingSketch(t, w, bevStationPlane(t, w, far),
		bevNgon(slope*math.Abs(apexStation-far)))
	return doc.Loft(nearSketch, nearRegion, farSketch, farRegion)
}

// buildBorePrism extrudes the bore circle symmetrically about the bore plane,
// which sits normal to the shaft at the shaft-axis edge's START, A'/B'. The
// half-length is 2 * Cone Distance per side, generously past any face width,
// which is what makes it a THROUGH cut.
func (f *bevFigure) buildBorePrism(t *testing.T, doc *decad.Document, w *sketch.World,
	g *bevSide) *decad.Body {
	t.Helper()
	start := g.station(f.apex, g.axisToe)
	s, region := bevRingSketch(t, w, bevStationPlane(t, w, start), bevNgon(g.boreDiameter/2))
	body, err := doc.Extrude(s, region, decad.Symmetric{
		D: units.Millimeters(2 * f.coneDistance),
	})
	if err != nil {
		t.Fatalf("%s: extrude the bore tool: %v", g.label, err)
	}
	return body
}

// assertBoreTool measures the tool before the cut consumes it: its diameter, that
// its two ends sit exactly 2 * Cone Distance either side of the shaft edge's
// start, and that both clear the frustum.
func bevAssertBoreTool(t *testing.T, f *bevFigure, g *bevSide, tool *decad.Body) {
	t.Helper()
	start := g.station(f.apex, g.axisToe)
	box, err := tool.Bounds()
	if err != nil {
		t.Fatalf("%s: bore tool bounds: %v", g.label, err)
	}
	decadtest.MeasuresBox(t, g.label+" bore tool extent",
		box,
		r3.NewVec(start-2*f.coneDistance, -g.boreDiameter/2, -g.boreDiameter/2),
		r3.NewVec(start+2*f.coneDistance, g.boreDiameter/2, g.boreDiameter/2),
		decadtest.Within(units.Millimeters(1e-6)))

	heel := g.station(f.apex, g.dedHeel)
	if start-2*f.coneDistance >= 0 || start+2*f.coneDistance <= heel {
		t.Errorf("%s: the bore tool spans stations %.6g to %.6g, which does not clear the frustum "+
			"between 0 and %.6g, so the cut is not a through cut",
			g.label, start-2*f.coneDistance, start+2*f.coneDistance, heel)
	}
	if g.boreDiameter > g.maxBore {
		t.Errorf("%s: the resolved bore diameter %.6g mm is above the Maximum Bore Diameter %.6g mm",
			g.label, g.boreDiameter, g.maxBore)
	}
}

// assertBoreCut measures the pierced band against its own closed form.
func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("%s: the bore cut leaves one body, got %d", g.label, len(bodies))
	}
	near, far := f.bandStations(g)
	apexStation := f.heelConeApex(g)
	slope := math.Tan(math.Pi/2 - g.gamma)
	nearRadius := slope * math.Abs(apexStation-near)
	farRadius := slope * math.Abs(apexStation-far)
	height := math.Abs(far - near)

	// The band is an n-gon frustum, so its volume is the frustum-of-pyramid form
	// on the two n-gon areas exactly; the bore removes its own n-gon prism over the
	// same height, since both polygons carry the same phase and the same axis. The
	// formula's error is float rounding, about 1e-14 relative.
	a1, a2 := bevNgonArea(nearRadius), bevNgonArea(farRadius)
	band := height / 3 * (a1 + a2 + math.Sqrt(a1*a2))
	removed := bevNgonArea(g.boreDiameter/2) * height
	decadtest.MeasuresVolume(t, bodies[0], units.CubicMillimeters(band-removed),
		decadtest.WithinRel(units.Scalar(1e-9)))

	// One lump, which is what a through hole leaves, and solid, which an enclosed
	// void would not be. RequireSolid has already read both off the document
	// report; this is the reading restated at the step that produces it.
	if lumps := len(bodies[0].Lumps()); lumps != 1 {
		t.Errorf("%s: the pierced band has %d lumps, want the one a through hole leaves",
			g.label, lumps)
	}
	if !bodies[0].IsSolid() {
		t.Errorf("%s: the pierced band is not solid", g.label)
	}
}

// stepMeshRotation rotates the DRIVING body by 180/N about its shaft axis, half a
// tooth pitch, so a driving valley sits where the pinion tooth crosses the axial
// plane. Both gears are patterned from a starting tooth in that plane, so without
// the offset a driving tooth and a pinion tooth would both sit at the crossing.
//
// The pinion takes _pinionMeshPhase instead, which is 0 at the shipped
// _PINION_MESH_PHASE_TEETH of 0.0, and a zero angle is a NO-OP rather than a
// move: Fusion refuses to move a body by the identity, with "invalid transform",
// which is why solids.rotate_body_about_edge absorbs it. The pinion case
// therefore builds the unrotated body and the assertion holds it there.
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	w := sketch.NewWorld()
	tooth, _ := bevBuildTooth(t, doc, w, f, g)

	angle := bevMeshAngle(g)
	if angle == 0 {
		return []*decad.Body{tooth}
	}
	motion, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s: mesh rotation: %v", g.label, err)
	}
	rotated, err := tooth.Placed(motion)
	if err != nil {
		t.Fatalf("%s: apply the mesh rotation: %v", g.label, err)
	}
	return []*decad.Body{rotated}
}

// meshAngle is this gear's meshing rotation: half a tooth pitch on the driving
// gear, and _pinionMeshPhase on the pinion, which is 0 tooth-fractions by
// default.
func bevMeshAngle(g *bevSide) float64 {
	if g.label == "Driving" {
		return math.Pi / g.teeth
	}
	return 0
}

// toothSeedAzimuth is the azimuth an untwisted, unrotated tooth section sits at
// about the shaft axis, from this case's own closed form: the section's area
// centroid mapped through the back-cone tooth plane.
func (f *bevFigure) toothSeedAzimuth(t *testing.T, g *bevSide) float64 {
	t.Helper()
	frame, err := r3.NewFrame(f.toothCentreWorld(g), f.dedendumWorld(g), r3.NewVec(0, 0, 1))
	if err != nil {
		t.Fatalf("%s: tooth-plane frame: %v", g.label, err)
	}
	centroid := bevRingCentroid(bevToothRing(f.toothDimensions(g), g.virtualTeeth))
	world := frame.ToWorldUV(centroid.X, centroid.Y)
	return math.Atan2(world.Z, world.Y)
}

// assertMeshRotation measures the azimuth the body moved through.
func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("%s: the mesh rotation leaves one body, got %d", g.label, len(bodies))
	}
	// The azimuth the body started at is recomputed from this case's own closed
	// form rather than read during the build and carried out of it. That keeps the
	// step safe on the parallel runner: nothing outside a case travels from its
	// build to its assertion, which is the one thing the Pattern step cannot say.
	azimuth, _, _ := bevToothPose(t, bodies[0])
	moved := bevNormalizeAngle(azimuth - f.toothSeedAzimuth(t, g))
	want := bevMeshAngle(g)
	if math.Abs(moved-want) > 1e-9 {
		t.Errorf("%s: the body turned %.9f rad about its shaft axis, want %.9f rad "+
			"(180/%0.f degrees on the driving gear, and 0 on the pinion, where the mid-face "+
			"section is already unrotated)", g.label, moved, want, g.teeth)
	}
}
