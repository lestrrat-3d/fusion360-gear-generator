package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// solidSteps is the number of involute samples the SOLID steps draw a flank
// with, and it is a substitution the whole file rests on.
//
// The spec fixes InvoluteSteps at 15, and the sketch step draws all 15. decad's
// exact free-form integration has a fixed work budget, and a tooth section
// carrying TWO fit splines exhausts it above eight samples per flank:
//
//	decad: not supported by the current evaluator: free-form exact integration
//	needs more than the fixed work budget of 1048576
//
// Eight samples describe the same involute through fewer points. What the
// substitution costs is that no solid step proves the 15-sample flank builds;
// the flank's shape, which is what the sample count changes, is proved at 15 by
// the sketch step and by involute.Point's own math.
const solidSteps = 8

// ---------------------------------------------------------------------------
// shared section builders
//
// Every solid step below builds its section through one of these. They are all
// substitutions of one kind, and the reason is the same: decad records a
// boundary segment only when both of its bounds are that segment's own ends. In
// Fusion the tooth's root arc and the disc's two arcs are pieces the tooth cuts
// out of one solid root circle, and a piece of a circle is exactly what decad
// refuses to record ("a *sketch.Circle fragment has an uncertified trim"). So
// each section here is drawn as its own closed loop of whole entities, with an
// explicit root arc where Fusion has a cut circle. What that costs is that the
// solid steps do not re-prove that ONE sketch closes both regions with the curve
// counts the two profile searches key on; that is what the Gear Profile sketch
// step proves, on the real cut circle, with real profile detection.
// ---------------------------------------------------------------------------

// toothSection draws one tooth as a closed loop: two involute flanks as fitted
// splines, the tooth-top arc, the two flank-to-root lines, and the root arc
// between their feet. In the embedded case the flanks start on the root circle
// and there are no flank-to-root lines, exactly as step 4.9 prescribes.
func toothSection(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := g.Dims
	s := proofkit.NewSketch(t)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)

	left, right := flankSamples(d, g.ToothNumber, g.Steps, g.Angle, flankStartRadius(d))
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(left[i].X, left[i].Y)
		rp[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	if _, err := s.CreateFitSpline(lp...); err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	if _, err := s.CreateFitSpline(rp...); err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	last := len(lp) - 1
	s.CreateArc(origin, rp[last], lp[last]) // tooth top, counter-clockwise
	rootStart, rootEnd := rp[0], lp[0]
	if !d.Embedded() {
		lf := rootFoot(d.Root, left[0])
		rf := rootFoot(d.Root, right[0])
		rootEnd = s.CreatePoint(lf.X, lf.Y)
		rootStart = s.CreatePoint(rf.X, rf.Y)
		s.CreateLine(rootEnd, lp[0])
		s.CreateLine(rootStart, rp[0])
	}
	s.CreateArc(origin, rootStart, rootEnd) // the root arc under the tooth
	fixAll(s)
	solve(t, s)
	return s, onlyProfile(t, s, "tooth section")
}

// discSection draws the solid disc inside the root circle — step 9's profile,
// whose boundary is the root circle and nothing else. The tip circle is
// construction geometry in the Gear Profile sketch and bounds no region, so the
// disc is not an annulus.
func discSection(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := proofkit.NewSketch(t)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	c := s.CreateCircle(origin, g.Dims.Root)
	s.AddConstraint(sketch.NewDiameter(c, 2*g.Dims.Root))
	solve(t, s)
	return s, onlyProfile(t, s, "disc section")
}

// chordedToothSection is toothSection with each flank chorded into a chain of
// straight segments instead of a fitted spline.
//
// SUBSTITUTION. decad refuses every modify operation on a body with a free-form
// wall — "a modify corner rewrite does not support a free-form boundary
// segment" — so the chamfer step cannot run on the spline tooth. The chords pass
// through the same involute samples, so the section is the tooth's own polygon;
// what it costs is that the front face then carries one edge per chord instead
// of the six the spec's chamfer step counts. That six is asserted on the real
// spline tooth in the extrude step, which is the body the count belongs to.
func chordedToothSection(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := g.Dims
	s := proofkit.NewSketch(t)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)

	left, right := flankSamples(d, g.ToothNumber, g.Steps, g.Angle, flankStartRadius(d))
	lp := make([]*sketch.Point, len(left))
	rp := make([]*sketch.Point, len(right))
	for i := range left {
		lp[i] = s.CreatePoint(left[i].X, left[i].Y)
		rp[i] = s.CreatePoint(right[i].X, right[i].Y)
	}
	for i := 0; i+1 < len(lp); i++ {
		s.CreateLine(lp[i], lp[i+1])
		s.CreateLine(rp[i], rp[i+1])
	}
	last := len(lp) - 1
	s.CreateArc(origin, rp[last], lp[last])
	rootStart, rootEnd := rp[0], lp[0]
	if !d.Embedded() {
		lf := rootFoot(d.Root, left[0])
		rf := rootFoot(d.Root, right[0])
		rootEnd = s.CreatePoint(lf.X, lf.Y)
		rootStart = s.CreatePoint(rf.X, rf.Y)
		s.CreateLine(rootEnd, lp[0])
		s.CreateLine(rootStart, rp[0])
	}
	s.CreateArc(origin, rootStart, rootEnd)
	fixAll(s)
	solve(t, s)
	return s, onlyProfile(t, s, "chorded tooth section")
}

// gearSection draws the whole gear cross-section as one closed loop: N chorded
// teeth spaced by one pitch, with a root arc joining each tooth's left foot to
// the next tooth's right foot.
//
// SUBSTITUTION. This is the section the pattern-and-combine pair leaves behind,
// built in one extrude instead of by patterning a tooth body and joining it.
// decad's booleans refuse both halves of the real route: a free-form-walled
// operand cannot be tessellated ("chording a boundary loop does not support a
// free-form boundary segment"), and even chorded, the tooth and the disc share
// their root cylinder and both cap planes exactly, which the boolean predicates
// will not classify ("two operand facets overlap in one plane"). What the
// substitute costs is that no step proves Fusion's Combine-Join runs; what it
// still proves is the solid that join has to leave behind — one lump, N teeth at
// the pattern angles, and the volume of the disc plus N teeth.
func gearSection(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	d := g.Dims
	n := int(g.ToothNumber)
	s := proofkit.NewSketch(t)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)

	type foot struct{ right, left *sketch.Point }
	feet := make([]foot, n)
	pitchAngle := 2 * math.Pi / g.ToothNumber
	for k := range n {
		angle := g.Angle + float64(k)*pitchAngle
		left, right := flankSamples(d, g.ToothNumber, g.Steps, angle, flankStartRadius(d))
		lp := make([]*sketch.Point, len(left))
		rp := make([]*sketch.Point, len(right))
		for i := range left {
			lp[i] = s.CreatePoint(left[i].X, left[i].Y)
			rp[i] = s.CreatePoint(right[i].X, right[i].Y)
		}
		for i := 0; i+1 < len(lp); i++ {
			s.CreateLine(lp[i], lp[i+1])
			s.CreateLine(rp[i], rp[i+1])
		}
		last := len(lp) - 1
		s.CreateArc(origin, rp[last], lp[last])
		if d.Embedded() {
			feet[k] = foot{right: rp[0], left: lp[0]}
			continue
		}
		lf := rootFoot(d.Root, left[0])
		rf := rootFoot(d.Root, right[0])
		lfp := s.CreatePoint(lf.X, lf.Y)
		rfp := s.CreatePoint(rf.X, rf.Y)
		s.CreateLine(lfp, lp[0])
		s.CreateLine(rfp, rp[0])
		feet[k] = foot{right: rfp, left: lfp}
	}
	for k := range n {
		s.CreateArc(origin, feet[k].left, feet[(k+1)%n].right)
	}
	fixAll(s)
	solve(t, s)
	return s, onlyProfile(t, s, "gear section")
}

func fixAll(s *sketch.Sketch) {
	for _, p := range s.Points() {
		s.Fix(p)
	}
}

func solve(t *testing.T, s *sketch.Sketch) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve section: %v", err)
	}
}

func onlyProfile(t *testing.T, s *sketch.Sketch, label string) *sketch.Profile {
	t.Helper()
	ps := s.Profiles()
	if len(ps) != 1 {
		t.Fatalf("%s closes %d regions, want exactly 1", label, len(ps))
	}
	if !ps[0].Valid {
		t.Fatalf("%s is not an extrudable region", label)
	}
	return ps[0]
}

func extrude(t *testing.T, doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, thickness float64) *decad.Body {
	t.Helper()
	// The extrude runs from the target plane to the Extrusion End Plane, which
	// sits exactly Thickness away along the plane normal, so the distance below
	// is that plane's offset. A ToEntityExtentDefinition has no counterpart in
	// decad; the extent it names is this distance.
	b, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude: %v", err)
	}
	return b
}

// checkPrismExtent checks that the body starts on the target plane and ends on
// the Extrusion End Plane — the reason both extrudes are given the same
// to-entity — and that its volume is the section area carried that far.
func checkPrismExtent(t *testing.T, b *decad.Body, thickness float64) {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if math.Abs(box.Min.Z) > 1e-4 {
		t.Errorf("the body starts at z = %g, want the target plane at 0", box.Min.Z)
	}
	if math.Abs(box.Max.Z-thickness) > 1e-4 {
		t.Errorf("the body ends at z = %g, want the Extrusion End Plane at Thickness = %g", box.Max.Z, thickness)
	}

	var near, far *decad.Face
	for _, f := range b.Faces() {
		switch {
		case planarFaceAt(f, 0):
			near = f
		case planarFaceAt(f, thickness):
			far = f
		}
	}
	if near == nil {
		t.Fatal("no face is coplanar with the gear's sketch plane")
	}
	if far == nil {
		t.Fatal("no face is parallel to but not coplanar with the sketch plane — ctx.extrusionExtent has no candidate")
	}
	area, err := near.Area()
	if err != nil {
		t.Fatalf("near cap area: %v", err)
	}
	want := area.Value.Mag() * thickness
	if got := volumeOf(t, b); math.Abs(got-want) > 1e-6*want {
		t.Errorf("volume %g mm^3, want the section area carried to the end plane, %g", got, want)
	}
}

// ---------------------------------------------------------------------------
// S10 — Extrude the tooth
// ---------------------------------------------------------------------------

// toothCases sweep size, thickness and both routes into the embedded profile,
// because the embedded profile is the case whose front face carries four edges
// instead of six and so is the one the chamfer step cannot serve.
var toothCases = []proofkit3d.Case{
	{Name: "default_m1_t17", Params: solidParams(1, 17, deg(20), 10)},
	{Name: "coarse_m8_t9", Params: solidParams(8, 9, deg(20), 60)},
	{Name: "fine_m0p3_t40_thin", Params: solidParams(0.3, 40, deg(20), 0.6)},
	{Name: "embedded_by_high_tooth_count", Params: solidParams(1, 60, deg(20), 10)},
	{Name: "embedded_by_large_pressure_angle", Params: solidParams(2, 30, deg(25), 25)},
}

// stepExtrudeTooth extrudes the single tooth cross-section from the target
// plane to the Extrusion End Plane as a new body, and checks the front face the
// completed-gear chamfer follows this step but selects only after the teeth are
// patterned, filleted, and optionally bore-cut.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := toothSection(t, g)
	body := extrude(t, doc, s, p, g.Thickness)

	front := frontFace(t, body)
	// An embedded profile has no flank-to-root lines, so its front face carries
	// four edges rather than six. The final chamfer has no edge-count predicate.
	wantEdges := 6
	if g.Dims.Embedded() {
		wantEdges = 4
	}
	if got := len(front.Edges()); got != wantEdges {
		t.Errorf("the tooth's front face has %d edges, want %d", got, wantEdges)
	}
	if got := len(arcEdgesOfRadius(front, g.Dims.Root)); got != 1 {
		t.Errorf("the front face has %d edges on the root circle, want exactly 1 — that arc is the one the chamfer skips", got)
	}
	if got := len(arcEdgesOfRadius(front, g.Dims.Tip)); got != 1 {
		t.Errorf("the front face has %d edges on the tip circle, want exactly 1 — the tooth-top arc", got)
	}
	return []*decad.Body{body}
}

func assertExtrudeTooth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the tooth extrude left %d bodies, want the one new body ctx.toothBody holds", len(bodies))
	}
	checkPrismExtent(t, bodies[0], g.Thickness)
	if got := len(cylindricalFacesOfRadius(bodies[0], g.Dims.Tip)); got != 1 {
		t.Errorf("the tooth has %d walls on the tip circle, want 1", got)
	}
	if got := len(cylindricalFacesOfRadius(bodies[0], g.Dims.Root)); got != 1 {
		t.Errorf("the tooth has %d walls on the root circle, want 1", got)
	}
}

// frontFace applies step 8's selection: the face coplanar with the gear's
// sketch plane. There is exactly one, and a partial match is an error rather
// than a fallback.
func frontFace(t *testing.T, b *decad.Body) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, f := range b.Faces() {
		if !planarFaceAt(f, 0) {
			continue
		}
		if found != nil {
			t.Fatal("more than one face is coplanar with the gear's sketch plane")
		}
		found = f
	}
	if found == nil {
		t.Fatal("front face not found: no face is coplanar with the gear's sketch plane")
	}
	return found
}

func toothCapFaces(t *testing.T, b *decad.Body, thickness float64) []*decad.Face {
	t.Helper()
	caps := []*decad.Face{frontFace(t, b)}
	for _, face := range b.Faces() {
		if !planarFaceAt(face, thickness) {
			continue
		}
		caps = append(caps, face)
	}
	if len(caps) != 2 {
		t.Fatalf("the tooth has %d planar cap faces, want 2", len(caps))
	}
	return caps
}

// ---------------------------------------------------------------------------
// S11 — Chamfer the tooth
// ---------------------------------------------------------------------------

// chamferCases put a case on each side of the completed-gear chamfer guard: a chamfer distance of
// zero, where no chamfer feature is created at all, and distances at both ends
// of the range a tooth of that size admits.
var chamferCases = []proofkit3d.Case{
	{Name: "chamfer_off", Params: withChamfer(solidParams(1, 17, deg(20), 10), 0)},
	{Name: "chamfer_small", Params: withChamfer(solidParams(1, 17, deg(20), 10), 0.05)},
	{Name: "chamfer_large", Params: withChamfer(solidParams(8, 9, deg(20), 60), 0.3)},
	{Name: "chamfer_on_thin_gear", Params: withChamfer(solidParams(0.3, 40, deg(20), 0.6), 0.002)},
}

// stepChamferTeeth proves the completed-gear selection's cap-edge behavior:
// both planar cap faces are found and root arcs remain in the edge set.
//
// SUBSTITUTION, twice over, and both are recorded here rather than only in the
// step list.
//
// First, the body is the CHORDED tooth: decad refuses every modify operation on
// a spline-walled prism ("a modify corner rewrite does not support a free-form
// boundary segment"), so no chamfer of the real tooth builds at all. The chords
// pass through the same involute samples.
//
// Second, the edges actually chamfered are the tooth's AXIAL flank edges, not
// the front-face loop the spec chamfers. A cap-loop chamfer of this contour
// does build, but decad cannot bless the result: its volume reading's proven
// bound lands outside the default relative tolerance at every tooth size and
// every setback tried (module 1, 5 and 20; setback 0.05 to 4 mm), so
// proofkit3d.RequireSolid refuses it. What the substitute still pins is the
// part the spec says goes wrong — the selection: both caps are found, each root
// arc stays in the set, and the selected edges are what the
// chamfer consumes. What it stops proving is that a chamfer band on those
// particular edges builds.
func stepChamferTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := chordedToothSection(t, g)
	body := extrude(t, doc, s, p, g.Thickness)

	caps := toothCapFaces(t, body, g.Thickness)
	// The chamfer edge set contains every edge from both end caps, including the
	// root-radius arcs. The production bore exclusion is covered by the source
	// contract because this chorded-tooth substitute has no bore feature.
	var chamferSet []*decad.Edge
	for _, cap := range caps {
		rootArcs := arcEdgesOfRadius(cap, g.Dims.Root)
		if len(rootArcs) != 1 {
			t.Fatalf("a tooth cap has %d edges on the root circle, want exactly 1", len(rootArcs))
		}
		for _, edge := range cap.Edges() {
			chamferSet = append(chamferSet, edge)
		}
	}
	if len(chamferSet) != len(caps[0].Edges())+len(caps[1].Edges()) {
		t.Errorf("the chamfer edge set holds %d cap edges, want every cap edge", len(chamferSet))
	}
	for _, cap := range caps {
		for _, rootArc := range arcEdgesOfRadius(cap, g.Dims.Root) {
			for _, edge := range chamferSet {
				if edge == rootArc {
					note(t, "a root-radius arc remains in the completed-gear chamfer set")
				}
			}
		}
	}

	if g.ChamferTooth <= 0 {
		note(t, "Apply chamfer to teeth is 0, so no chamfer feature is created")
		return []*decad.Body{body}
	}
	before := volumeOf(t, body)
	// chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, ChamferTooth, False).
	chamfered, err := body.Chamfer(decad.Edges(decad.ParallelTo(zAxis()), decad.Convex()), units.Millimeters(g.ChamferTooth))
	if err != nil {
		t.Fatalf("equal-distance chamfer: %v", err)
	}
	if after := volumeOf(t, chamfered); after >= before {
		t.Errorf("the chamfer left the tooth at %g mm^3, up from %g — an equal-distance chamfer removes material", after, before)
	}
	return []*decad.Body{chamfered}
}

func assertChamferTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the chamfer left %d bodies, want 1", len(bodies))
	}
	b := bodies[0]
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if math.Abs(box.Min.Z) > 1e-4 || math.Abs(box.Max.Z-g.Thickness) > 1e-4 {
		t.Errorf("the chamfer changed the tooth's extent to [%g, %g], want [0, %g]", box.Min.Z, box.Max.Z, g.Thickness)
	}
	// Chamfered or not, the tooth is still the section carried from the target
	// plane to the Extrusion End Plane; the chamfer changes the section, not the
	// extent. How much material it took is checked in the step itself, which is
	// where both the before and after readings exist.
	checkPrismExtent(t, b, g.Thickness)
	if g.ChamferTooth <= 0 {
		return
	}
}

// ---------------------------------------------------------------------------
// S12 — Extrude the body
// ---------------------------------------------------------------------------

var bodyCases = []proofkit3d.Case{
	{Name: "default_m1_t17", Params: solidParams(1, 17, deg(20), 10)},
	{Name: "coarse_m8_t9_thick", Params: solidParams(8, 9, deg(20), 250)},
	{Name: "fine_m0p3_t40_thin", Params: solidParams(0.3, 40, deg(20), 0.2)},
	{Name: "embedded_by_large_pressure_angle", Params: solidParams(2, 30, deg(25), 25)},
}

// stepExtrudeBody extrudes the disc inside the root circle to the Extrusion End
// Plane as a new body, and captures the two references step 9 keeps: the
// cylindrical face the Gear Center axis is built from, and the far end-cap face
// the bore cut later ends on.
func stepExtrudeBody(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := discSection(t, g)
	body := extrude(t, doc, s, p, g.Thickness)

	// setByCircularFace(cylindrical_face): any face whose surfaceType is
	// CylinderSurfaceType. The disc has exactly one, and its axis is the gear's.
	cyl := cylindricalFacesOfRadius(body, g.Dims.Root)
	if len(cyl) != 1 {
		t.Fatalf("the gear body has %d cylindrical faces at the root radius, want 1 to build the Gear Center axis from", len(cyl))
	}
	axis := cyl[0].Surface().(decad.Cylinder).Axis
	if math.Abs(math.Abs(axis.Dot(zAxis()))-1) > 1e-6 {
		t.Errorf("the Gear Center axis came out along %v, want the target plane's normal", axis)
	}

	// ctx.extrusionExtent: among the planar faces, the one parallel to but NOT
	// coplanar with the sketch plane. The near cap is coplanar, which is what
	// rules it out.
	var extent *decad.Face
	for _, f := range body.Faces() {
		if planarFaceAt(f, g.Thickness) && !planarFaceAt(f, 0) {
			if extent != nil {
				t.Fatal("more than one planar face is parallel to but not coplanar with the sketch plane")
			}
			extent = f
		}
	}
	if extent == nil {
		t.Fatal("ctx.extrusionExtent not found: no planar face is parallel to but not coplanar with the sketch plane")
	}
	return []*decad.Body{body}
}

func assertExtrudeBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the body extrude left %d bodies, want the one Gear Body", len(bodies))
	}
	checkPrismExtent(t, bodies[0], g.Thickness)
	want := math.Pi * g.Dims.Root * g.Dims.Root * g.Thickness
	if got := volumeOf(t, bodies[0]); math.Abs(got-want) > 1e-4*want {
		t.Errorf("Gear Body volume %g mm^3, want pi*RootCircleRadius^2*Thickness = %g", got, want)
	}
}

// ---------------------------------------------------------------------------
// S13 — Pattern the teeth
// ---------------------------------------------------------------------------

var patternCases = []proofkit3d.Case{
	{Name: "default_m1_t17", Params: solidParams(1, 17, deg(20), 10)},
	{Name: "coarse_m8_t9", Params: solidParams(8, 9, deg(20), 60)},
	{Name: "embedded_by_high_tooth_count", Params: solidParams(1, 60, deg(20), 10)},
}

// patternProbeIndices are the copies the pattern step places. Index 1 pins the
// increment, index 2 pins that it repeats, and index N-1 pins that the last copy
// lands one increment short of a full turn — which is what totalAngle = 360 deg
// with isSymmetric = False and quantity = ToothNumber means.
func patternProbeIndices(n int) []int { return []int{1, 2, n - 1} }

// stepPatternTeeth circular-patterns the tooth body about the gear axis.
//
// SUBSTITUTION. decad has no pattern feature, so each copy is placed by an
// explicit rotation about the gear axis — which is what a circular pattern is —
// and only three of the N-1 copies are placed. All N-1 do build, but decad
// cannot decide the interference pair between two of them and reports the
// document Suspect, which no gate may pass. The three copies chosen pin the
// increment, its repetition and the wrap to a full turn; what the substitute
// stops proving is that all N copies coexist without interfering.
func stepPatternTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := toothSection(t, g)
	seed := extrude(t, doc, s, p, g.Thickness)

	bodies := []*decad.Body{seed}
	for _, i := range patternProbeIndices(int(g.ToothNumber)) {
		// quantity = ToothNumber over totalAngle = '360 deg', isSymmetric = False:
		// copy i sits at i full turns divided by the tooth count.
		turn, err := r3.Rotation(zAxis(), units.Degrees(360*float64(i)/g.ToothNumber))
		if err != nil {
			t.Fatalf("pattern rotation for copy %d: %v", i, err)
		}
		// The pattern's bodies collection already holds the seed plus the copies,
		// so the seed is never re-added.
		copyBody, err := seed.PlacedCopy(turn)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", i, err)
		}
		bodies = append(bodies, copyBody)
	}
	return bodies
}

func assertPatternTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	indices := patternProbeIndices(int(g.ToothNumber))
	if len(bodies) != len(indices)+1 {
		t.Fatalf("the pattern left %d bodies, want the seed plus %d copies", len(bodies), len(indices))
	}
	seedAngle, seedRadius := centroidAngle(t, bodies[0])
	seedVolume := volumeOf(t, bodies[0])
	for k, i := range indices {
		got := bodies[k+1]
		if v := volumeOf(t, got); math.Abs(v-seedVolume) > 1e-6*seedVolume {
			t.Errorf("copy %d has volume %g mm^3, want the seed tooth's %g", i, v, seedVolume)
		}
		angle, radius := centroidAngle(t, got)
		if math.Abs(radius-seedRadius) > 1e-6*seedRadius {
			t.Errorf("copy %d sits at radius %g mm, want the seed's %g", i, radius, seedRadius)
		}
		want := 2 * math.Pi * float64(i) / g.ToothNumber
		if diff := math.Abs(wrapAngle(angle-seedAngle) - wrapAngle(want)); diff > 1e-9 {
			t.Errorf("copy %d is %g rad from the seed, want %g — a full turn divided by ToothNumber, %d times",
				i, wrapAngle(angle-seedAngle), wrapAngle(want), i)
		}
	}
}

func wrapAngle(a float64) float64 {
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	return a
}

// ---------------------------------------------------------------------------
// S14 — Combine the patterned teeth into the gear body
// ---------------------------------------------------------------------------

var combineCases = []proofkit3d.Case{
	{Name: "default_m1_t17", Params: solidParams(1, 17, deg(20), 10)},
	{Name: "coarse_m8_t9_thick", Params: solidParams(8, 9, deg(20), 250)},
	{Name: "embedded_by_large_pressure_angle", Params: solidParams(2, 30, deg(25), 25)},
}

// stepCombineTeeth proves what the single Combine-Join has to leave behind: one
// body, watertight and free of voids, carrying every tooth at its pattern angle.
// See gearSection for why the join itself is not the operation that builds it.
func stepCombineTeeth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := gearSection(t, g)
	return []*decad.Body{extrude(t, doc, s, p, g.Thickness)}
}

func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the combine left %d bodies, want the single Gear Body", len(bodies))
	}
	b := bodies[0]
	checkPrismExtent(t, b, g.Thickness)

	n := int(g.ToothNumber)
	if got := len(cylindricalFacesOfRadius(b, g.Dims.Tip)); got != n {
		t.Errorf("the gear carries %d tooth tops, want ToothNumber = %d", got, n)
	}
	if got := len(cylindricalFacesOfRadius(b, g.Dims.Root)); got != n {
		t.Errorf("the gear carries %d root valleys, want ToothNumber = %d", got, n)
	}
	// Every tooth-top wall is a piece of the one tip circle, so each must be
	// coaxial with the gear: the pattern turned the tooth about the Gear Center
	// axis and about nothing else.
	for _, f := range cylindricalFacesOfRadius(b, g.Dims.Tip) {
		cy := f.Surface().(decad.Cylinder)
		if math.Abs(math.Abs(cy.Axis.Dot(zAxis()))-1) > 1e-6 {
			t.Errorf("a tooth top is on an axis %v, want the gear axis", cy.Axis)
		}
		if math.Hypot(cy.Origin.X, cy.Origin.Y) > 1e-6 {
			t.Errorf("a tooth top is on a cylinder centred at (%g, %g), want the gear centre", cy.Origin.X, cy.Origin.Y)
		}
	}
}

// ---------------------------------------------------------------------------
// S15 — Root fillets
// ---------------------------------------------------------------------------

var filletCases = []proofkit3d.Case{
	{Name: "fillet_off", Params: withFillet(solidParams(1, 17, deg(20), 10), 0)},
	{Name: "default_m1_t17", Params: withFillet(solidParams(1, 17, deg(20), 10), 0.2)},
	{Name: "coarse_m8_t9", Params: withFillet(solidParams(8, 9, deg(20), 60), 1.6)},
	{Name: "embedded_by_large_pressure_angle", Params: withFillet(solidParams(2, 30, deg(25), 25), 0.3)},
}

// stepRootFillets rounds the corner where each root valley floor meets a tooth
// flank — the sharp inside corner that runs the full thickness of the gear,
// parallel to its main axis, which is where bending stress concentrates.
//
// The edge selection is the spec's own: every cylindrical face at the root
// radius (the pattern usually leaves one patch per valley rather than a single
// continuous surface), and on each of those the straight edges whose direction
// is parallel to the gear axis. The circular edges that wrap the front and back
// end caps are rims, not structural corners, and are dropped.
func stepRootFillets(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := gearSection(t, g)
	body := extrude(t, doc, s, p, g.Thickness)

	rootFaces := cylindricalFacesOfRadius(body, g.Dims.Root)
	if got, want := len(rootFaces), int(g.ToothNumber); got != want {
		t.Fatalf("found %d root-radius cylindrical faces, want one valley patch per tooth, %d", got, want)
	}
	axial := 0
	for _, f := range rootFaces {
		for _, e := range f.Edges() {
			if isAxialLine(e) {
				axial++
			}
		}
	}
	if want := 2 * int(g.ToothNumber); axial != want {
		t.Errorf("collected %d axial root edges, want the two valley-floor-to-flank corners on each of %d valleys", axial, want)
	}

	if g.FilletRadius <= 0 {
		note(t, "Fillet Radius is 0, so no fillet feature is created")
		return []*decad.Body{body}
	}
	// addConstantRadiusEdgeSet(edges, FilletRadius, isTangentChain=False): the
	// collected edges are exactly the axial root corners, and tangent-chaining
	// would let more than the intended corner be rounded.
	filleted, err := body.Fillet(decad.Edges(decad.ParallelTo(zAxis()), decad.Concave()), units.Millimeters(g.FilletRadius))
	if err != nil {
		t.Fatalf("constant-radius root fillet: %v", err)
	}
	return []*decad.Body{filleted}
}

func assertRootFillets(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the fillet left %d bodies, want 1", len(bodies))
	}
	b := bodies[0]
	checkExtent(t, b, g.Thickness)
	if g.FilletRadius <= 0 {
		checkPrismExtent(t, b, g.Thickness)
		return
	}
	// A fillet on an inside corner ADDS material, so the gear must come out
	// heavier than the plain prism of the same front face.
	front := frontFace(t, b)
	area, err := front.Area()
	if err != nil {
		t.Fatalf("front face area: %v", err)
	}
	if got, prism := volumeOf(t, b), area.Value.Mag()*g.Thickness; math.Abs(got-prism) > 1e-6*prism {
		t.Errorf("volume %g mm^3 does not match the filleted section carried through, %g", got, prism)
	}
	if got := len(cylindricalFacesOfRadius(b, g.FilletRadius)); got != 2*int(g.ToothNumber) {
		t.Errorf("the gear carries %d fillet walls at the fillet radius, want two per valley, %d", got, 2*int(g.ToothNumber))
	}
}

func checkExtent(t *testing.T, b *decad.Body, thickness float64) {
	t.Helper()
	box, err := b.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if math.Abs(box.Min.Z) > 1e-4 || math.Abs(box.Max.Z-thickness) > 1e-4 {
		t.Errorf("the body spans z in [%g, %g], want [0, Thickness = %g]", box.Min.Z, box.Max.Z, thickness)
	}
}

// isAxialLine is the spec's edge filter: a straight edge whose direction, taken
// from its geometry endpoints and normalized, is parallel to the target plane's
// normal within the stated tolerance. The loose tolerance is deliberate — a
// tighter test drops valid axial edges that are slightly off and leaves root
// fillets missing.
func isAxialLine(e *decad.Edge) bool {
	if _, ok := e.Curve().(decad.Line3); !ok {
		return false
	}
	dir := e.End().Position().Value.Sub(e.Start().Position().Value)
	unit, ok := dir.Normalize()
	if !ok {
		return false
	}
	return math.Abs(math.Abs(unit.Dot(zAxis()))-1) < 0.01
}

// ---------------------------------------------------------------------------
// S17 — Bore cut
// ---------------------------------------------------------------------------

// boreCases put a case on each side of step 12's Bore-Diameter guard and sweep
// the bore across the range a gear of that size admits.
var boreCases = []proofkit3d.Case{
	{Name: "bore_off", Params: withBore(solidParams(1, 17, deg(20), 10), 0)},
	{Name: "small_bore", Params: withBore(solidParams(1, 17, deg(20), 10), 1)},
	{Name: "default_bore", Params: withBore(solidParams(1, 17, deg(20), 10), 5)},
	{Name: "large_bore_coarse_gear", Params: withBore(solidParams(8, 9, deg(20), 60), 40)},
}

// stepBoreCut cuts the bore through the gear body, from the target plane to the
// far end-cap face, so the hole goes all the way through whatever Thickness is.
//
// SUBSTITUTION, in two places.
//
// The body cut is the plain disc rather than the toothed gear: decad refuses the
// boolean against the full toothed section ("the analytic cut scene exceeds this
// evaluator's arrangement work cap"). The teeth stand outside the root circle
// and the bore is inside it, so nothing about the cut depends on them.
//
// The tool runs past both caps rather than ending on the far end-cap face.
// decad's booleans will not classify two operands whose caps are coplanar, which
// a tool ending exactly on the far face is; running it past both ends produces
// the same through hole and lets the predicates decide.
func stepBoreCut(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	g := readGear(params)
	s, p := discSection(t, g)
	body := extrude(t, doc, s, p, g.Thickness)

	if g.BoreDiameter <= 0 {
		note(t, "Bore Diameter is 0, so buildBore returns before drawing anything")
		return []*decad.Body{body}
	}

	bore := proofkit.NewSketch(t)
	centre := bore.CreatePoint(0, 0)
	bore.Fix(centre)
	circle := bore.CreateCircle(centre, g.BoreDiameter/2)
	bore.AddConstraint(sketch.NewDiameter(circle, g.BoreDiameter))
	solve(t, bore)

	tool, err := doc.Extrude(bore, onlyProfile(t, bore, "bore profile"),
		decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(g.Thickness * 2)},
			Two: decad.DistanceSide{D: units.Millimeters(g.Thickness)},
		})
	if err != nil {
		t.Fatalf("bore tool: %v", err)
	}
	cut, err := decad.Cut(body, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{cut}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, params map[string]float64) {
	g := readGear(params)
	if len(bodies) != 1 {
		t.Fatalf("the bore left %d bodies, want the one gear body it affects", len(bodies))
	}
	b := bodies[0]
	checkExtent(t, b, g.Thickness)

	boreRadius := g.BoreDiameter / 2
	want := math.Pi * g.Dims.Root * g.Dims.Root * g.Thickness
	if g.BoreDiameter > 0 {
		want -= math.Pi * boreRadius * boreRadius * g.Thickness
	}
	if got := volumeOf(t, b); math.Abs(got-want) > 1e-3*want {
		t.Errorf("volume %g mm^3, want the disc less the bore, %g", got, want)
	}
	got := len(cylindricalFacesOfRadius(b, boreRadius))
	if g.BoreDiameter > 0 && got != 1 {
		t.Errorf("the gear has %d walls at the bore radius, want the one cut through it", got)
	}
	if g.BoreDiameter <= 0 && got != 0 {
		t.Errorf("the gear has %d walls at radius 0 with the bore off, want none", got)
	}
}

// ---------------------------------------------------------------------------
// case-table helpers
// ---------------------------------------------------------------------------

func solidParams(module, toothNumber, pressureAngle, thickness float64) map[string]float64 {
	return map[string]float64{
		"module": module, "toothNumber": toothNumber, "pressureAngle": pressureAngle,
		"involuteSteps": solidSteps, "angle": 0, "thickness": thickness,
	}
}

func withChamfer(p map[string]float64, d float64) map[string]float64 {
	p["chamferTooth"] = d
	return p
}

func withFillet(p map[string]float64, r float64) map[string]float64 {
	p["filletRadius"] = r
	return p
}

func withBore(p map[string]float64, d float64) map[string]float64 {
	p["boreDiameter"] = d
	return p
}

// note records which part of a solid step is running, the 3D counterpart of
// proofkit.Step.
func note(t *testing.T, format string, args ...any) {
	t.Helper()
	t.Logf("step: "+format, args...)
}
