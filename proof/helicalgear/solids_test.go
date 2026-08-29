package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// witnessHeight is how far the helix-plane step extrudes its section. decad has
// no standalone construction plane to gate, and a body is the only thing a
// proofkit3d run can return, so the plane is proven by building on it and
// reading where the body starts. The height is arbitrary and nothing reads it
// but the far-face assertion.
const witnessHeight = 1.0

// capTolerance is the slack allowed on a coordinate read back off a body built
// from exactly-placed points. Every wall here is a planar quad, so the readings
// come back at machine precision and this is generous.
const capTolerance = 1e-9

// helixPlaneCases sweep the Thickness the offset is taken from, because
// helicalPlaneOffset() returns the FULL Thickness and herringbone re-points the
// same hook at half of it. A single thickness cannot tell those apart when the
// number happens to agree, so the sweep runs values whose halves are distinct
// and includes the dialog default of 10 mm. The helix angle is swept with it
// because the section drawn on this plane is the twisted one.
var helixPlaneCases = []proofkit3d.Case{
	{Name: "default_thickness10_helix14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 8, "arcSteps": 4, "thickness": 10}},
	{Name: "thin_thickness2_helix14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 8, "arcSteps": 4, "thickness": 2}},
	{Name: "thick_thickness35_helix-14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 8, "arcSteps": 4, "thickness": 35}},
	{Name: "coarse_m3_n15_thickness25_helix30", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20),
		"helixAngle": deg(30), "involuteSteps": 8, "arcSteps": 4, "thickness": 25}},
}

// loftCases sweep what the loft itself decides: the sign and size of the twist,
// the section size, and the height. Zero is included because a helical gear with
// no helix is the degenerate member of the family and the loft must still close
// on two sections that differ only by their plane. The negative cases are the
// left-hand helix, which is a distinct build and not the mirror image of the
// positive one as far as section correspondence is concerned.
var loftCases = []proofkit3d.Case{
	{Name: "default_m1_n17_helix14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 8, "arcSteps": 4, "thickness": 10}},
	{Name: "lefthand_m1_n17_helix-14.5", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 8, "arcSteps": 4, "thickness": 10}},
	{Name: "zerohelix_m1_n17_helix0", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": 0, "involuteSteps": 8, "arcSteps": 4, "thickness": 10}},
	{Name: "steep_m1_n17_helix35", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(35), "involuteSteps": 8, "arcSteps": 4, "thickness": 10}},
	{Name: "steep_lefthand_m1_n12_helix-35_thin", Params: map[string]float64{
		"module": 1, "toothNumber": 12, "pressureAngle": deg(20),
		"helixAngle": deg(-35), "involuteSteps": 8, "arcSteps": 4, "thickness": 3}},
	{Name: "coarse_m3_n15_helix20", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20),
		"helixAngle": deg(20), "involuteSteps": 8, "arcSteps": 4, "thickness": 25}},
	{Name: "fine_m2_n20_helix-25", Params: map[string]float64{
		"module": 2, "toothNumber": 20, "pressureAngle": deg(20),
		"helixAngle": deg(-25), "involuteSteps": 8, "arcSteps": 4, "thickness": 12}},
}

// chamferCases put the front-face edge count on both sides of the branch that
// decides it. The non-embedded cases are what a helical gear actually builds and
// carry six curves; the embedded cases carry four and are where a wanted count
// of 4 would match. Sizes and sample counts vary to show the count is a property
// of the tooth's shape and not of how finely the flank was sampled.
//
// wantCapEdges is the count the profile's own curves imply, and the assertion
// holds the built face to it.
var chamferCases = []proofkit3d.Case{
	{Name: "nonembedded_m1_n17_samples8", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 8, "thickness": 10, "wantCapEdges": 6}},
	{Name: "nonembedded_m1_n17_samples4", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 4, "thickness": 10, "wantCapEdges": 6}},
	{Name: "nonembedded_m3_n15_samples5", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20),
		"helixAngle": deg(-14.5), "involuteSteps": 5, "thickness": 25, "wantCapEdges": 6}},
	{Name: "embedded_by_toothcount_m1_n45", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": deg(20),
		"helixAngle": deg(14.5), "involuteSteps": 5, "thickness": 10, "wantCapEdges": 4}},
	{Name: "embedded_by_pressureangle_m1_n20_pa30", Params: map[string]float64{
		"module": 1, "toothNumber": 20, "pressureAngle": deg(30),
		"helixAngle": deg(14.5), "involuteSteps": 8, "thickness": 10, "wantCapEdges": 4}},
}

// stepHelixPlane builds the offset construction plane the twisted profile is
// drawn on, and the twisted section on it.
//
// [HELI-F-TWIST-PLANE] creates the plane with setByOffset from self.plane at
// self.helicalPlaneOffset(), and helical's hook returns the full Thickness. The
// offset is a numeric snapshot: getParameterAsValueInput hands back
// ValueInput.createByReal(param.value), the Thickness as it stood at generation
// time, not a live reference to the parameter.
//
// decad has no construction plane of its own to hand a gate, so the plane is
// proven by what is built on it: the twisted section is extruded a fixed
// witness height and the body's near face reports where the plane sits. That is
// the substitution, and it costs the fact that Fusion's plane is a timeline
// entity with a light bulb — this proves its position, not its lifecycle. The
// two visibility facts [HELI-F-TWIST-PLANE] pins, that the plane is left lit and
// the twisted sketch never shown, have no counterpart in decad at all.
func stepHelixPlane(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	d, teeth, steps, angle := dims(params)
	thickness := params["thickness"]
	if d.Embedded() {
		proofkit3d.Unmodelled(t, "helical does not build an embedded tooth")
	}

	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), thickness)
	if err != nil {
		t.Fatalf("offset plane at the full Thickness: %v", err)
	}
	s, profile := polyToothSection(t, w, plane, d, teeth, steps, int(params["arcSteps"]), angle)
	body, err := doc.Extrude(s, profile,
		decad.Distance{D: units.Millimeters(witnessHeight), Dir: decad.Along})
	if err != nil {
		t.Fatalf("witness extrude off the helix plane: %v", err)
	}
	return []*decad.Body{body}
}

// assertHelixPlane holds the plane to the full Thickness.
func assertHelixPlane(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	thickness := params["thickness"]
	box, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if math.Abs(box.Min.Z-thickness) > capTolerance {
		t.Errorf("the twisted profile's plane sits at %.12f mm, want the full Thickness %.12f mm",
			box.Min.Z, thickness)
	}
	// Named explicitly rather than left to the reader of the number above:
	// helicalPlaneOffset() is a distinct overridable hook and herringbone
	// re-points it at half the thickness so its mirror plane lands mid-body.
	// Helical's must not be that.
	if math.Abs(box.Min.Z-thickness/2) <= capTolerance && thickness != 0 {
		t.Errorf("the plane sits at half the Thickness (%.12f mm); that is herringbone's offset, not helical's",
			box.Min.Z)
	}
	if math.Abs(box.Max.Z-(thickness+witnessHeight)) > capTolerance {
		t.Errorf("the witness body spans to %.12f mm, want %.12f mm", box.Max.Z, thickness+witnessHeight)
	}
}

// stepLoftTooth lofts the bottom Gear Profile tooth loop to the top twisted
// tooth loop, into ctx.toothBody.
//
// [HELI-F-LOFT]: both sections are found with the same fixed curve counts, the
// BOTTOM section is added first and the top second, and the operation is a new
// body. Section order is not cosmetic — it is what decides which end of the
// solid carries the twist, and reversing it turns a right-hand helix into a
// left-hand one. The assertion reads the twist off the built body with its sign,
// so a reversed order fails here.
//
// Both sections are polylines. See substitutions 1 and 2 in the package comment:
// decad's Loft admits only same-kind LineSeg, ArcSeg or CircleSeg pairs, so the
// fitted-spline flanks cannot cross it at all, and an ArcSeg pair's chorded
// walls leave a volume bound the RunSolid gate refuses. What survives is a
// section whose every segment is a straight line, and it survives with exact
// walls.
func stepLoftTooth(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	d, teeth, steps, angle := dims(params)
	thickness := params["thickness"]
	arcSteps := int(params["arcSteps"])
	if d.Embedded() {
		proofkit3d.Unmodelled(t, "helical does not build an embedded tooth")
	}

	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), thickness)
	if err != nil {
		t.Fatalf("offset plane: %v", err)
	}
	// The bottom section is the Gear Profile sketch's tooth, drawn at angle 0 by
	// the inherited super().buildSketches(ctx). The top is the Twisted Gear
	// Profile's, drawn at the helix angle.
	bottomSketch, bottomProfile := polyToothSection(t, w, w.XY(), d, teeth, steps, arcSteps, 0)
	topSketch, topProfile := polyToothSection(t, w, top, d, teeth, steps, arcSteps, angle)
	body, err := doc.Loft(bottomSketch, bottomProfile, topSketch, topProfile)
	if err != nil {
		t.Fatalf("loft bottom section to top section: %v", err)
	}
	return []*decad.Body{body}
}

// assertLoftTooth measures the three things the loft is supposed to produce: a
// body that spans the target plane to the helix plane, a top section turned by
// exactly the helix angle relative to the bottom one, and two caps that
// correspond edge for edge.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	thickness := params["thickness"]
	angle := params["helixAngle"]
	body := bodies[0]

	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if math.Abs(box.Min.Z) > capTolerance {
		t.Errorf("the loft starts at z=%.12f mm, want the target plane at 0", box.Min.Z)
	}
	if math.Abs(box.Max.Z-thickness) > capTolerance {
		t.Errorf("the loft ends at z=%.12f mm, want the helix plane at the full Thickness %.12f mm",
			box.Max.Z, thickness)
	}

	bottom, top := caps(t, body, thickness)
	if bottom.edges != top.edges {
		t.Errorf("the two loft sections carry %d and %d edges; a loft pairs segment for segment",
			bottom.edges, top.edges)
	}
	got := math.Atan2(top.y, top.x) - math.Atan2(bottom.y, bottom.x)
	if math.Abs(got-angle) > 1e-9 {
		t.Errorf("the lofted tooth twists by %.9f rad, want the helix angle %.9f rad",
			got, angle)
	}

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	if volume.Value.Mag() <= 0 {
		t.Errorf("the lofted tooth encloses %v", volume.Value)
	}
}

// stepChamferFrontFace builds the tooth the inherited chamferTooth step has to
// find a front face on, and is where [HELI-F-CHAMFER-COUNT] is settled.
//
// chamferTooth picks the tooth's front face by a single conjunction: the face is
// coplanar with the gear's sketch plane AND face.edges.count == chamferWantEdges().
// Helical raises that wanted count from spur's 6 to 4, and its own fusion.md
// flags the value as asserted and unverified, since the non-embedded tooth it
// lofts carries six curves. This step measures the count on a real body.
//
// The body is an EXTRUDE of the real six-curve tooth, not the loft. That is
// substitution 3 in the package comment: the count only means anything while
// each flank is one edge, which the loft's chorded sections give up. The face
// counted is therefore the cap of an untwisted extruded tooth. What carries over
// is that the cap loop's edge count is the profile's curve count, which the
// sketch step proves is 6 for the loop the loft consumes.
//
// One thing no harness here can reach: the spec's concrete failure mode is that
// chamferTooth RAISES when no face matches and the command's execute handler
// then calls deleteComponent(), rolling the whole new component back. That is
// Fusion control flow around an add-in exception. decad has no chamfer-by-face-
// search and no command handler, so the rollback is unproven here; what is
// proven is the premise it rests on, that the counts do not match.
func stepChamferFrontFace(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body {
	d, teeth, steps, angle := dims(params)
	w := sketch.NewWorld()
	s, profile := toothSection(t, w, w.XY(), d, teeth, steps, angle)
	body, err := doc.Extrude(s, profile,
		decad.Distance{D: units.Millimeters(params["thickness"]), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude the tooth whose front face the chamfer searches for: %v", err)
	}
	return []*decad.Body{body}
}

// assertChamferFrontFace measures the front face's edge count and holds
// helical's wanted count against it.
func assertChamferFrontFace(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64) {
	d, _, _, _ := dims(params)
	want := int(params["wantCapEdges"])
	bottom, top := caps(t, bodies[0], params["thickness"])
	if bottom.edges != want || top.edges != want {
		t.Errorf("the tooth's front and back faces carry %d and %d edges, want %d",
			bottom.edges, top.edges, want)
	}
	if d.Embedded() {
		if want != helicalChamferWantEdges {
			t.Errorf("an embedded tooth's front face carries %d edges; helical wants %d",
				want, helicalChamferWantEdges)
		}
		t.Logf("an embedded tooth's front face carries %d edges, which is exactly "+
			"chamferWantEdges()'s %d — the shape the flagged value does fit, and the one "+
			"helical can never build, since loftTooth's fixed six-curve section search "+
			"finds nothing in it", bottom.edges, helicalChamferWantEdges)
		return
	}
	if bottom.edges == helicalChamferWantEdges {
		t.Errorf("a non-embedded tooth's front face carries %d edges, which would make "+
			"chamferWantEdges()'s %d correct; [HELI-F-CHAMFER-COUNT] says it does not",
			bottom.edges, helicalChamferWantEdges)
	}
	t.Logf("the non-embedded tooth helical lofts has a %d-edge front face while "+
		"chamferWantEdges() returns %d, so the coplanar-and-edge-count conjunction "+
		"matches no face and chamferTooth raises for any chamfer > 0 "+
		"([HELI-F-CHAMFER-COUNT], reproduced not fixed)", bottom.edges, helicalChamferWantEdges)
}

// capReading is one end face of a tooth body: how many edges bound it, and where
// its boundary sits about the gear axis.
type capReading struct {
	edges int
	x, y  float64
}

// caps returns the near (z = 0) and far (z = height) end faces of a tooth body.
// They are the only planar faces whose normal runs along the gear axis.
func caps(t *testing.T, body *decad.Body, height float64) (near, far capReading) {
	t.Helper()
	found := 0
	for _, face := range body.Faces() {
		plane, ok := face.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(plane.Frame.N().Z)-1) > 1e-9 {
			continue
		}
		var sx, sy, sz float64
		edges := face.Edges()
		for _, e := range edges {
			p := e.Start().Position().Value
			sx, sy, sz = sx+p.X, sy+p.Y, sz+p.Z
		}
		n := float64(len(edges))
		reading := capReading{edges: len(edges), x: sx / n, y: sy / n}
		if sz/n > height/2 {
			far = reading
		} else {
			near = reading
		}
		found++
	}
	if found != 2 {
		t.Fatalf("the tooth body has %d axis-normal planar faces, want the two end caps", found)
	}
	return near, far
}
