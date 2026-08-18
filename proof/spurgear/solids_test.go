package spurgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// gearBodySection rebuilds the part of the Gear Profile sketch the body extrude
// consumes: the root circle, in the state Fusion's profile detection leaves it
// after the tooth's flank-to-root lines have split it — two arcs meeting at the
// two points where the tooth meets the root circle.
//
// Fusion draws one whole root circle and derives those two arcs by splitting
// it. The sketch step (stepGearProfileSketch) models that faithfully and lets
// profile detection do the splitting. This function draws the two arcs
// outright, because decad refuses to record a boundary edge that is a fragment
// of a curve whose trim it cannot certify, and the sketch engine withholds that
// certificate from every edge of any sketch that holds a free-form curve. The
// region is the same region either way; only who splits the circle differs.
func gearBodySection(t *testing.T, p map[string]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	dim := dimensionsOf(p)
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, p[pToothNumber], int(p[pSteps]), p[pAngle])

	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	fixed := func(x, y float64) *sketch.Point {
		point := s.CreatePoint(x, y)
		s.Fix(point)
		return point
	}
	origin := fixed(0, 0)
	onRoot := func(pt involute.Pt) *sketch.Point {
		theta := math.Atan2(pt.Y, pt.X)
		return fixed(dim.Root*math.Cos(theta), dim.Root*math.Sin(theta))
	}
	leftSplit := onRoot(left[0])
	rightSplit := onRoot(right[0])
	// Counter-clockwise from the right split to the left split is the arc under
	// the tooth; the other arc closes the circle.
	s.CreateArc(origin, rightSplit, leftSplit)
	s.CreateArc(origin, leftSplit, rightSplit)
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve gear body section: %v", err)
	}

	for _, profile := range s.Profiles() {
		arcs := 0
		others := 0
		for _, entity := range profile.Entities {
			if _, ok := entity.(*sketch.Arc); ok {
				arcs++
				continue
			}
			others++
		}
		// find_profile_by_curve_counts(sketch, arcs=2): exactly two arcs and
		// nothing else.
		if arcs == 2 && others == 0 {
			return s, profile
		}
	}
	t.Fatal("no two-arc gear body profile in the section sketch")
	return nil, nil
}

func mm(v units.Value) float64 { return v.Base() }

func closeTo(got, want, rel float64) bool {
	scale := math.Max(math.Abs(want), 1)
	return math.Abs(got-want) <= rel*scale
}

// ---------------------------------------------------------------------------
// Step: Extrude the Gear Body
// ---------------------------------------------------------------------------

// stepExtrudeGearBody extrudes the two-arc disc inside the root circle from the
// target plane to the Extrusion End Plane, as a new body.
//
// The tip circle is construction geometry and bounds nothing, so the profile is
// the solid disc rather than an annulus — that is the claim the volume
// assertion settles.
func stepExtrudeGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	s, profile := gearBodySection(t, p)
	body, err := doc.Extrude(s, profile, decad.Distance{
		D:   units.Millimeters(p[pThickness]),
		Dir: decad.Along,
	})
	if err != nil {
		t.Fatalf("extrude gear body: %v", err)
	}
	return []*decad.Body{body}
}

// assertGearBody checks what step 9 needs to be true of the body it just made:
// the disc's volume, its axial extent, and the face classification the step
// walks to find the Gear Center axis and the far end cap.
func assertGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("gear body extrude produced %d bodies, want 1", len(bodies))
	}
	body := bodies[0]
	dim := dimensionsOf(p)
	thickness := p[pThickness]

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	want := math.Pi * dim.Root * dim.Root * thickness
	if got := mm(volume.Value); !closeTo(got, want, 1e-9) {
		t.Errorf("gear body volume %.6f mm^3, want the full root disc %.6f mm^3", got, want)
	}

	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if !closeTo(bounds.Min.Z, 0, 1e-9) || !closeTo(bounds.Max.Z, thickness, 1e-9) {
		t.Errorf("gear body spans z %.6f..%.6f, want 0..%.6f", bounds.Min.Z, bounds.Max.Z, thickness)
	}
	if !closeTo(bounds.Max.X, dim.Root, 1e-9) || !closeTo(bounds.Max.Y, dim.Root, 1e-9) {
		t.Errorf("gear body reaches x %.6f y %.6f, want the root radius %.6f", bounds.Max.X, bounds.Max.Y, dim.Root)
	}

	// Step 9 classifies faces by surface type: the one cylindrical face gives
	// the Gear Center axis, and among the planar faces the one that is parallel
	// to but not coplanar with the sketch plane is the far end cap the bore cut
	// later targets.
	var cylindrical, planar int
	var farCaps int
	for _, face := range body.Faces() {
		switch face.Surface().Kind() {
		case decad.KindCylinder:
			cylindrical++
		case decad.KindPlane:
			planar++
		}
	}
	if cylindrical < 1 {
		t.Errorf("gear body has no cylindrical face, so step 9 has nothing to build the Gear Center axis from")
	}
	if planar != 2 {
		t.Errorf("gear body has %d planar faces, want the two end caps", planar)
	}
	for _, face := range body.Faces() {
		if face.Surface().Kind() != decad.KindPlane {
			continue
		}
		area, err := face.Area()
		if err != nil {
			t.Fatalf("face area: %v", err)
		}
		if !closeTo(mm(area.Value), math.Pi*dim.Root*dim.Root, 1e-6) {
			t.Errorf("end cap area %.6f mm^2, want the root disc %.6f mm^2",
				mm(area.Value), math.Pi*dim.Root*dim.Root)
		}
		farCaps++
	}
	if farCaps != 2 {
		t.Errorf("counted %d end caps, want 2", farCaps)
	}
}

func TestExtrudeGearBody(t *testing.T) {
	proofkit3d.RunSolid(t, gear3DCases(), stepExtrudeGearBody, assertGearBody)
}

// ---------------------------------------------------------------------------
// Step: Bore Extrude-Cut
// ---------------------------------------------------------------------------

// stepBoreCut cuts the bore through the gear body, from the target plane to the
// far end cap, affecting only that body.
//
// The receiver here is the plain root disc rather than the disc with its teeth
// joined on: the bore is concentric and well inside the root circle, so the
// teeth are untouched by it, and the tooth ring is the part decad cannot build
// (see the step list's note on the tooth extrude).
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	dim := dimensionsOf(p)
	bore := p[pBoreDiameter]
	if bore <= 0 {
		proofkit3d.Unmodelled(t, "bore diameter %.3f is not positive, so buildBore returns early", bore)
	}
	if bore/2 >= dim.Root {
		proofkit3d.Unmodelled(t, "bore radius %.3f is not inside the root radius %.3f", bore/2, dim.Root)
	}

	s, profile := gearBodySection(t, p)
	gearBody, err := doc.Extrude(s, profile, decad.Distance{
		D:   units.Millimeters(p[pThickness]),
		Dir: decad.Along,
	})
	if err != nil {
		t.Fatalf("extrude gear body: %v", err)
	}

	// The Bore Profile sketch, and its extrude-cut. Running the tool past the
	// far cap is how the ToEntityExtentDefinition to that face behaves: the
	// bore goes all the way through whatever the thickness is.
	world := sketch.NewWorld()
	boreSketch, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatalf("create bore sketch: %v", err)
	}
	centre := boreSketch.CreatePoint(0, 0)
	boreSketch.Fix(centre)
	circle := boreSketch.CreateCircle(centre, bore/2)
	boreSketch.AddConstraint(sketch.NewDiameter(circle, bore))
	if _, err := boreSketch.Solve(context.Background()); err != nil {
		t.Fatalf("solve bore sketch: %v", err)
	}
	tool, err := doc.Extrude(boreSketch, boreSketch.Profiles()[0], decad.Symmetric{
		D: units.Millimeters(2 * p[pThickness]),
	})
	if err != nil {
		t.Fatalf("extrude bore tool: %v", err)
	}

	bored, err := decad.Cut(gearBody, tool)
	if err != nil {
		t.Fatalf("bore cut: %v", err)
	}
	return []*decad.Body{bored}
}

// assertBoredBody checks the bore went all the way through and took exactly the
// cylinder it was supposed to.
func assertBoredBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("bore cut produced %d bodies, want 1", len(bodies))
	}
	body := bodies[0]
	dim := dimensionsOf(p)
	boreR := p[pBoreDiameter] / 2
	thickness := p[pThickness]

	volume, err := body.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	// The cut is evaluated on a faceted mesh, so the readings converge on the
	// analytic answer from below rather than matching it exactly. A tenth of a
	// percent is far tighter than the difference a bore of the wrong diameter,
	// or one that stopped short, would make.
	const facetedTolerance = 1e-3
	want := math.Pi * (dim.Root*dim.Root - boreR*boreR) * thickness
	if got := mm(volume.Value); !closeTo(got, want, facetedTolerance) {
		t.Errorf("bored body volume %.6f mm^3, want %.6f mm^3", got, want)
	}

	bounds, err := body.Bounds()
	if err != nil {
		t.Fatalf("bounds: %v", err)
	}
	if !closeTo(bounds.Min.Z, 0, facetedTolerance) || !closeTo(bounds.Max.Z, thickness, facetedTolerance) {
		t.Errorf("bored body spans z %.6f..%.6f, want the full 0..%.6f — a bore that stopped short "+
			"would leave the far cap whole", bounds.Min.Z, bounds.Max.Z, thickness)
	}
	if !closeTo(bounds.Max.X, dim.Root, facetedTolerance) {
		t.Errorf("bored body reaches x %.6f, want the root radius %.6f", bounds.Max.X, dim.Root)
	}
}

func TestBoreCut(t *testing.T) {
	proofkit3d.RunSolid(t, gear3DCases(), stepBoreCut, assertBoredBody)
}

// gear3DCases mirrors gearCases for the solid steps.
func gear3DCases() []proofkit3d.Case {
	cases := make([]proofkit3d.Case, 0, len(gearCases()))
	for _, c := range gearCases() {
		cases = append(cases, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return cases
}
