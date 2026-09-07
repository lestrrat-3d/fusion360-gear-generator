package spurgear_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// filletCases cover the root fillet: the default gear, a coarse gear off the
// sketch origin, the tightest fit the formula allows at 20 degrees (N=25,
// where the fillet's setback nearly equals the stub's length), and an
// embedded gear.
var filletCases = []proofkit3d.Case{
	{Name: "M1_N17_T10", Params: solidParams(1, 17, 20, 0, 0, 10, 0, 0)},
	{Name: "M2_N12_T6_anchor_offset", Params: solidParams(2, 12, 20, 30, -15, 6, 0, 0)},
	{Name: "M1_N25_T5", Params: solidParams(1, 25, 20, 0, 0, 5, 0, 0)},
	{Name: "M1_N45_T4_embedded", Params: solidParams(1, 45, 20, 0, 0, 4, 0, 0)},
}

// boreCases reach both sides of the bore guard: no bore, and bores on the
// default, the off-origin and the embedded gear.
var boreCases = []proofkit3d.Case{
	{Name: "M1_N17_T10_no_bore", Params: solidParams(1, 17, 20, 0, 0, 10, 0, 0)},
	{Name: "M1_N17_T10_bore4", Params: solidParams(1, 17, 20, 0, 0, 10, 4, 0)},
	{Name: "M2_N12_T6_anchor_offset_bore6", Params: solidParams(2, 12, 20, 30, -15, 6, 6, 0)},
	{Name: "M1_N45_T4_embedded_bore10", Params: solidParams(1, 45, 20, 0, 0, 4, 10, 0)},
}

// chamferCases reach both sides of the chamfer guard and both sides of the
// bore exclusion.
var chamferCases = []proofkit3d.Case{
	{Name: "M1_N17_T10_no_chamfer", Params: solidParams(1, 17, 20, 0, 0, 10, 0, 0)},
	{Name: "M1_N17_T10_chamfer0.3", Params: solidParams(1, 17, 20, 0, 0, 10, 0, 0.3)},
	{Name: "M1_N17_T10_bore4_chamfer0.3", Params: solidParams(1, 17, 20, 0, 0, 10, 4, 0.3)},
	{Name: "M2_N12_T6_anchor_offset_bore6_chamfer0.4", Params: solidParams(2, 12, 20, 30, -15, 6, 6, 0.4)},
	{Name: "M1_N45_T4_embedded_chamfer0.2", Params: solidParams(1, 45, 20, 0, 0, 4, 0, 0.2)},
}

// joinedGear builds the substitute joined gear (step 10b) for the steps that
// come after it, returning the body and its section area.
func joinedGear(t *testing.T, doc *decad.Document, g gear) (*decad.Body, float64) {
	t.Helper()
	s, prof := gearSketch(t, g)
	return extrudeTo(t, doc, s, prof, g.thickness), recordedArea(t, s, prof)
}

// gearFaces is the face count of the joined gear before any later step.
func (g gear) gearFaces() int {
	return 2 + int(g.toothNumber)*(g.outlineAt(0).lateralCurves()+1)
}

// rootCorners selects the axial edges where a valley floor meets a tooth
// flank: parallel to the gear axis and concave, which on the joined gear is
// exactly the root corners. The spec picks them as the line edges of the
// root-radius cylindrical faces whose direction is parallel to the axis.
func rootCorners() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(zAxis), decad.Concave())
}

// ---------------------------------------------------------------- step 11

// stepRootFillets rounds every root corner of the joined gear with Fillet
// Radius. The joined gear is the step-10b substitute.
func stepRootFillets(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	gear, _ := joinedGear(t, doc, g)
	if g.filletRadius <= 0 {
		t.Fatalf("Fillet Radius %.6f is not positive", g.filletRadius)
	}
	edges, err := rootCorners().SelectEdges(gear)
	if err != nil {
		t.Fatalf("select root corners: %v", err)
	}
	if want := 2 * int(g.toothNumber); len(edges) != want {
		// The spec skips the fillet when nothing matched. The construction
		// always yields two axial root corners per tooth, so an empty or
		// short selection is a defect here, not a case to skip.
		t.Fatalf("selected %d root corners, want %d", len(edges), want)
	}
	for _, e := range edges {
		if _, ok := e.Curve().(decad.Line3); !ok {
			t.Fatalf("root corner edge is %T, want a straight axial edge", e.Curve())
		}
	}
	filleted, err := gear.Fillet(rootCorners(), units.Millimeters(g.filletRadius))
	if err != nil {
		if g.Embedded() {
			// The embedded corner is spline against root circle in Fusion, one
			// face each side. The chorded substitute puts a very short first
			// chord there, shorter than the fillet's setback, and decad refuses
			// an over-large setback rather than run the fillet onto the next
			// chord. Nothing decad accepts reproduces a single flank face, so
			// the embedded fillet stays out of reach here.
			proofkit3d.Unmodelled(t, "embedded root fillet on chorded flanks: %v", err)
		}
		t.Fatalf("fillet: %v", err)
	}
	return []*decad.Body{filleted}
}

// filletAddedArea is the section area one root fillet adds: the corner
// between the radial stub and the root circle of radius R, filled by an arc
// of radius r tangent to both. As R grows it tends to r^2 (1 - pi/4), the
// square corner's figure.
func filletAddedArea(r, R float64) float64 {
	beta := math.Asin(r / (R + r))
	return 0.5*r*math.Sqrt((R+r)*(R+r)-r*r) - 0.5*R*R*beta - 0.5*r*r*(math.Pi/2-beta)
}

// assertRootFillets checks the fillet added exactly the material 2N tangent
// corner fills add, kept the gear's extent, left one more face per corner,
// and left no axial concave edge that is not the seam of a fillet.
func assertRootFillets(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	if len(bodies) != 1 {
		t.Fatalf("fillet left %d bodies, want 1", len(bodies))
	}
	gear := bodies[0]
	s, prof := gearSketch(t, g)
	area := recordedArea(t, s, prof)
	n := int(g.toothNumber)
	added := 2 * float64(n) * filletAddedArea(g.filletRadius, g.Root)
	if v := volume(t, gear, "filleted gear"); !near(v, (area+added)*g.thickness, 1e-6) {
		t.Fatalf("filleted gear volume %.9f, want %.9f (section %.9f plus %d fillets adding %.9f)", v, (area+added)*g.thickness, area, 2*n, added)
	}
	assertSpansThickness(t, gear, g.thickness, "filleted gear")
	if want, got := g.gearFaces()+2*n, len(gear.Faces()); got != want {
		t.Fatalf("filleted gear has %d faces, want %d", got, want)
	}
	// Whatever still reads as an axial concave edge is a seam where a fillet
	// meets its neighbour tangentially, never a sharp corner.
	left, err := rootCorners().SelectEdges(gear)
	if err != nil && !errors.Is(err, decad.ErrNoMatch) {
		t.Fatalf("select remaining axial concave edges: %v", err)
	}
	for _, e := range left {
		onFillet := false
		for _, f := range e.Faces() {
			if cyl, ok := f.Surface().(decad.Cylinder); ok && near(cyl.Radius.Base(), g.filletRadius, 1e-9) {
				onFillet = true
			}
		}
		if !onFillet {
			t.Fatalf("an axial concave edge remains that is not a fillet seam")
		}
	}
}

// ---------------------------------------------------------------- step 12

// boreSketch draws the Bore Profile: a circle of Bore Diameter centred on the
// projected anchor, plus the tooth generator's free local origin and its
// coincidence to that anchor. Reference geometry is an externally locked
// snapshot in this abstract engine. It can prove the resulting constraint net
// and reference refresh, but only the retained native probe can determine
// Fusion's isFixed/isLinked/isReference/isFullyConstrained flags.
func boreSketch(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newXYSketch(t)
	localOrigin := s.CreatePoint(0, 0)
	localOrigin.SetName("local origin")
	anchor := s.CreateReferencePoint(g.anchorX, g.anchorY, "Tools anchor projection")
	anchor.SetName("projected anchor")
	circle := s.CreateCircle(anchor, g.bore/2)
	circle.SetName("Bore Diameter")
	s.AddConstraint(sketch.NewDiameter(circle, g.bore))

	before, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve Bore Profile before anchoring: %v", err)
	}
	if !before.Converged || before.DOF != 2 || before.Redundant != 0 {
		t.Fatalf("Bore Profile before anchoring: converged=%v DOF=%d redundant=%d, want true/2/0",
			before.Converged, before.DOF, before.Redundant)
	}

	s.AddConstraint(sketch.NewCoincident(localOrigin, anchor))
	requireBoreAnchorSound(t, s, localOrigin, anchor, circle, g.anchorX, g.anchorY)

	// RefreshReference is the abstract engine's public 3D re-feed path. It is
	// not evidence of Fusion association, which the native source-move stage
	// supplies, but it proves the compiled constraint net follows a new anchor.
	const dx, dy = 5.0, 2.5
	if err := s.RefreshReference(anchor, g.anchorX+dx, g.anchorY+dy); err != nil {
		t.Fatalf("refresh projected anchor: %v", err)
	}
	requireBoreAnchorSound(t, s, localOrigin, anchor, circle, g.anchorX+dx, g.anchorY+dy)
	if err := s.RefreshReference(anchor, g.anchorX, g.anchorY); err != nil {
		t.Fatalf("restore projected anchor: %v", err)
	}
	requireBoreAnchorSound(t, s, localOrigin, anchor, circle, g.anchorX, g.anchorY)
	return s, singleProfile(t, s, "Bore Profile")
}

func requireBoreAnchorSound(t *testing.T, s *sketch.Sketch, localOrigin, anchor *sketch.Point,
	circle *sketch.Circle, wantX, wantY float64) {
	t.Helper()
	result, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve Bore Profile after anchoring: %v", err)
	}
	if !result.Converged || result.DOF != 0 || result.Redundant != 0 {
		t.Fatalf("Bore Profile after anchoring: converged=%v DOF=%d redundant=%d, want true/0/0",
			result.Converged, result.DOF, result.Redundant)
	}
	for name, point := range map[string]*sketch.Point{
		"local origin":     localOrigin,
		"projected anchor": anchor,
		"circle centre":    circle.Center,
	} {
		if !near(point.X(), wantX, 1e-9) || !near(point.Y(), wantY, 1e-9) {
			t.Fatalf("%s solved to (%.6f, %.6f), want (%.6f, %.6f)",
				name, point.X(), point.Y(), wantX, wantY)
		}
	}
}

// boreTool extrudes the Bore Profile from the target plane to the body's far
// end cap, the to-entity extent onto ctx.extrusionExtent.
func boreTool(t *testing.T, doc *decad.Document, g gear, body *decad.Body) *decad.Body {
	t.Helper()
	s, prof := boreSketch(t, g)
	tool, err := doc.Extrude(s, prof, decad.ToFace{Body: body, Face: farCap(body)})
	if err != nil {
		t.Fatalf("extrude bore to the far cap: %v", err)
	}
	return tool
}

// boredBody extrudes the Gear Body's disc and cuts the bore through it with
// the to-entity tool. It is the real cut on the body the spec names, minus
// the teeth.
func boredBody(t *testing.T, doc *decad.Document, g gear) *decad.Body {
	t.Helper()
	ds, dp := discSketch(t, g)
	body := extrudeTo(t, doc, ds, dp, g.thickness)
	bored, err := decad.Cut(body, boreTool(t, doc, g, body))
	if err != nil {
		t.Fatalf("cut the bore from the Gear Body: %v", err)
	}
	return bored
}

// stepBore cuts the bore through the gear when Bore Diameter is positive,
// and leaves the gear alone when it is not.
//
// Fusion cuts the bore with a to-entity extrude onto the far end cap of the
// joined gear. decad's analytic cut refuses the joined gear's section (the
// 17-tooth scene charges 4323 arranger segments against its cap of 4096),
// its mesh cut refuses a tool ending exactly on the caps as a tangent
// contact, and the sketch engine does not nest the bore circle as a hole of
// the 544-entity outline, so no bored toothed gear can be built here. The
// step therefore first extrudes the bore tool onto the far cap of the joined
// gear in a scratch document and checks the extent reached that cap, and
// requires decad's refusal of that cut so a later engine that takes it is
// noticed; then it runs the same cut on the Gear Body alone, the root
// cylinder, which is the body the spec names minus teeth the bore never
// touches. That body is what the assertion pins.
func stepBore(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	if g.bore <= 0 {
		gear, _ := joinedGear(t, doc, g)
		return []*decad.Body{gear}
	}
	scratch := decad.New()
	gear0, _ := joinedGear(t, scratch, g)
	tool := boreTool(t, scratch, g, gear0)
	assertPrism(t, tool, math.Pi*g.bore*g.bore/4, g.thickness, "bore tool to the far cap")
	if _, err := decad.Cut(gear0, tool); err == nil {
		t.Fatalf("decad cut the joined gear's bore directly: the Gear Body stand-in below is no longer needed, cut the real one")
	} else if !errors.Is(err, decad.ErrUnsupported) {
		t.Fatalf("cut joined gear: %v", err)
	}
	return []*decad.Body{boredBody(t, doc, g)}
}

// assertBoredBody checks a body of the given unbored section area and face
// count lost exactly the bore cylinder through the whole Thickness on the
// anchor's axis.
func assertBoredBody(t *testing.T, body *decad.Body, area float64, faces int, g gear, what string) {
	t.Helper()
	hole := math.Pi * g.bore * g.bore / 4
	if v := volume(t, body, what); !near(v, (area-hole)*g.thickness, 1e-6) {
		t.Fatalf("%s volume %.9f, want %.9f", what, v, (area-hole)*g.thickness)
	}
	assertSpansThickness(t, body, g.thickness, what)
	if got := len(body.Faces()); got != faces+1 {
		t.Fatalf("%s has %d faces, want %d", what, got, faces+1)
	}
	var bores int
	for _, f := range body.Faces() {
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok || !near(cyl.Radius.Base(), g.bore/2, 1e-9) {
			continue
		}
		bores++
		if off := cyl.Origin.Sub(r3.NewVec(g.anchorX, g.anchorY, cyl.Origin.Z)); off.Len() > 1e-9 {
			t.Fatalf("%s: bore axis misses the anchor by %v", what, off)
		}
	}
	if bores != 1 {
		t.Fatalf("%s has %d cylindrical faces of the bore radius, want 1", what, bores)
	}
}

// assertBore checks the bore removed exactly a cylinder of Bore Diameter
// through the whole Thickness on the anchor's axis, or removed nothing when
// there is no bore.
func assertBore(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	if len(bodies) != 1 {
		t.Fatalf("bore left %d bodies, want 1", len(bodies))
	}
	if g.bore <= 0 {
		s, prof := gearSketch(t, g)
		assertPrism(t, bodies[0], recordedArea(t, s, prof), g.thickness, "unbored gear")
		if got := len(bodies[0].Faces()); got != g.gearFaces() {
			t.Fatalf("unbored gear has %d faces, want %d", got, g.gearFaces())
		}
		return
	}
	assertBoredBody(t, bodies[0], math.Pi*g.Root*g.Root, 3, g, "bored Gear Body")
}

// ---------------------------------------------------------------- step 13

// capEdges collects, once each, every edge of the body's planar faces
// parallel to the sketch plane, minus a circular edge whose radius is the
// positive Bore Diameter over two within 0.01 mm (the spec's 0.001 cm). It
// is the spec's selection rule, run on the built body.
func capEdges(t *testing.T, body *decad.Body, g gear) (kept []*decad.Edge, excluded int) {
	t.Helper()
	seen := map[*decad.Edge]struct{}{}
	for _, f := range body.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(pl.Frame.N().Dot(zAxis))-1) > 1e-9 {
			continue
		}
		for _, e := range f.Edges() {
			if _, dup := seen[e]; dup {
				continue
			}
			seen[e] = struct{}{}
			if c, ok := e.Curve().(decad.Circle3); ok && g.bore > 0 && math.Abs(c.Radius.Base()-g.bore/2) <= 0.01 {
				excluded++
				continue
			}
			kept = append(kept, e)
		}
	}
	return kept, excluded
}

// chamferOffset is where the chamfer's stand-in body sits: beside the gear,
// clear of it, so both can be verified in one document.
func (g gear) chamferOffset() r3.Vec { return r3.NewVec(3*g.Tip, 0, 0) }

// stepChamferTeeth applies the equal-distance chamfer to the completed
// gear's end-cap edges, bore edges excluded, when Apply Chamfer To Teeth is
// positive, after the optional bore.
//
// decad builds a cap-loop chamfer on the toothed gear, but the volume bound
// it certifies for one is far coarser than the material removed (measured: a
// bound of 86 mm^3 against 5 mm^3 removed on the 17-tooth gear), so the
// document cannot verify and the harness gate refuses it. It also chamfers
// one body's cap loops per call and takes no further operation on the
// result, and no bored toothed gear can be built here at all (see stepBore).
// So this step returns two bodies. The first is the joined gear, unbored and
// unchamfered, on which the spec's edge-selection rule is run and its counts
// asserted. The second is the Gear Body, bored when the case has a bore,
// moved beside the gear, with the selection rule run on it too (its bore rims
// excluded) and the chamfer applied to its start cap's outer rim: that rim is
// the cap's one convex edge, which is what excluding the bore circle by
// radius comes to on this body. The assertion pins the removed material
// exactly.
func stepChamferTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	gear, _ := joinedGear(t, doc, g)
	if g.chamfer <= 0 {
		return []*decad.Body{gear}
	}
	kept, excluded := capEdges(t, gear, g)
	if perCap := int(g.toothNumber) * (g.outlineAt(0).lateralCurves() + 1); len(kept) != 2*perCap || excluded != 0 {
		t.Fatalf("chamfer selection on the gear kept %d cap edges and excluded %d, want %d and 0", len(kept), excluded, 2*perCap)
	}

	var body *decad.Body
	if g.bore > 0 {
		body = boredBody(t, doc, g)
		kept, excluded = capEdges(t, body, g)
		if len(kept) != 2 || excluded != 2 {
			t.Fatalf("chamfer selection on the bored Gear Body kept %d edges and excluded %d, want the 2 outer rims kept and the 2 bore rims excluded", len(kept), excluded)
		}
	} else {
		ds, dp := discSketch(t, g)
		body = extrudeTo(t, doc, ds, dp, g.thickness)
		kept, excluded = capEdges(t, body, g)
		if len(kept) != 2 || excluded != 0 {
			t.Fatalf("chamfer selection on the Gear Body kept %d edges and excluded %d, want 2 and 0", len(kept), excluded)
		}
	}
	shift, err := r3.Translation(g.chamferOffset())
	if err != nil {
		t.Fatalf("translation: %v", err)
	}
	body, err = body.Placed(shift)
	if err != nil {
		t.Fatalf("move the Gear Body beside the gear: %v", err)
	}
	outerRim := decad.Edges(decad.CreatedBy(decad.CapStart(body)), decad.Convex()).Exactly(1)
	chamfered, err := body.Chamfer(outerRim, units.Millimeters(g.chamfer))
	if err != nil {
		t.Fatalf("chamfer: %v", err)
	}
	return []*decad.Body{gear, chamfered}
}

// assertChamferTeeth checks the gear is untouched, and that the stand-in's
// chamfer removed exactly the ring a distance-d chamfer takes off a cylinder
// rim, pi d^2 (R - d/3), added one conical face, and left the bore alone.
func assertChamferTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	s, prof := gearSketch(t, g)
	area := recordedArea(t, s, prof)
	if g.chamfer <= 0 {
		if len(bodies) != 1 {
			t.Fatalf("no chamfer left %d bodies, want the gear alone", len(bodies))
		}
		assertPrism(t, bodies[0], area, g.thickness, "gear with no chamfer")
		return
	}
	if len(bodies) != 2 {
		t.Fatalf("chamfer left %d bodies, want the gear and the chamfered Gear Body", len(bodies))
	}
	gear, body := bodies[0], bodies[1]
	assertPrism(t, gear, area, g.thickness, "gear")

	R, d := g.Root, g.chamfer
	hole := 0.0
	if g.bore > 0 {
		hole = math.Pi * g.bore * g.bore / 4
	}
	removed := math.Pi * d * d * (R - d/3)
	if v := volume(t, body, "chamfered Gear Body"); !near(v, (math.Pi*R*R-hole)*g.thickness-removed, 1e-6) {
		t.Fatalf("chamfered Gear Body volume %.9f, want %.9f (%.9f removed)", v, (math.Pi*R*R-hole)*g.thickness-removed, removed)
	}
	assertSpansThickness(t, body, g.thickness, "chamfered Gear Body")
	wantFaces := 4 // cylinder, two caps, the chamfer cone
	if g.bore > 0 {
		wantFaces++
	}
	if got := len(body.Faces()); got != wantFaces {
		t.Fatalf("chamfered Gear Body has %d faces, want %d", got, wantFaces)
	}
	var cones int
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cone); ok {
			cones++
		}
	}
	if cones != 1 {
		t.Fatalf("chamfered Gear Body has %d conical faces, want 1", cones)
	}
	if g.bore > 0 {
		// The bore's two rim circles are still there: the chamfer took the
		// outer rim only.
		var rims int
		for _, e := range body.Edges() {
			if c, ok := e.Curve().(decad.Circle3); ok && near(c.Radius.Base(), g.bore/2, 1e-9) {
				rims++
			}
		}
		if rims != 2 {
			t.Fatalf("bore rim circles after the chamfer: %d, want 2", rims)
		}
	}
	gb, err := gear.Bounds()
	if err != nil {
		t.Fatalf("gear bounds: %v", err)
	}
	bb, err := body.Bounds()
	if err != nil {
		t.Fatalf("Gear Body bounds: %v", err)
	}
	if bb.Min.X <= gb.Max.X {
		t.Fatalf("the stand-in body overlaps the gear in x: %.3f <= %.3f", bb.Min.X, gb.Max.X)
	}
}
