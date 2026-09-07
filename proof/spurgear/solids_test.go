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

// solidParams builds a solid case: a spur gear drawn at angle 0 with the
// spec's 15 involute steps.
func solidParams(module, teeth, pressureAngleDeg, anchorX, anchorY, thickness, bore, chamfer float64) map[string]float64 {
	p := params(module, teeth, pressureAngleDeg, 0, 15, anchorX, anchorY)
	p[pThickness] = thickness
	p[pBoreDiameter] = bore
	p[pChamfer] = chamfer
	return p
}

// solidCases feed the extrude, pattern and combine steps: the default gear,
// a coarse one whose anchor is off the sketch origin, and an embedded one.
var solidCases = []proofkit3d.Case{
	{Name: "M1_N17_T10", Params: solidParams(1, 17, 20, 0, 0, 10, 0, 0)},
	{Name: "M2_N12_T6_anchor_offset", Params: solidParams(2, 12, 20, 30, -15, 6, 0, 0)},
	{Name: "M1_N45_T4_embedded", Params: solidParams(1, 45, 20, 0, 0, 4, 0, 0)},
}

// zAxis is the sketch plane's normal: every solid step extrudes along it.
var zAxis = r3.NewVec(0, 0, 1)

// mm reads a measurement as a float in base units (mm, mm^2, mm^3).
func mm(m decad.Measurement) float64 { return m.Value.Base() }

// newXYSketch returns an empty sketch on a fresh world's XY plane, which
// stands for the target plane every sketch of this gear sits on.
func newXYSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("create sketch: %v", err)
	}
	return s
}

// solve runs the solver and fails on anything but convergence.
func solve(t *testing.T, s *sketch.Sketch) {
	t.Helper()
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e", res.Residual)
	}
}

// What the solid steps substitute, and what it costs.
//
// decad extrudes a region only from a boundary it can record exactly, and it
// refuses two things the real Gear Profile sketch does. The tooth loop's
// root edge is a fragment of the solid root circle split by the stubs (or by
// the flanks, when embedded); in a sketch that holds a fitted spline no
// fragment's trim is certified exact, so the region is unrecordable. And a
// fitted spline through the spec's 15 samples exceeds decad's exact
// free-form integration budget (measured against the pinned engine: a flank
// spline through 8 samples extrudes, one through 10 or more is refused). So
// the substitute sketches below draw each flank as the
// chords between the same 15 sample points, close the tooth with an explicit
// arc on the root circle between the same two root points, and in the
// embedded shape start the flank at its analytic crossing of the root circle
// instead of where the fitted spline's split would put it.
//
// What survives the substitution and is asserted: the swept extent, the
// volume as area times Thickness, the face-per-boundary-curve topology, the
// placement of every pattern instance, the join's volume as disc plus N
// teeth, the fillet's added material, the bore's removed material and the
// cap-loop chamfer. What does not survive: the flank's exact curve (chords
// bow inside the involute by a few microns), the 6-curve loop count, which
// the Gear Profile step asserts on the real sketch instead, and Fusion's
// boolean join itself, which decad refuses for operands that share the root
// cylinder face.

// toothOutline is one tooth's boundary points at a given angle, as the
// substitute sketches draw it: the flank points from where the flank leaves
// the root outward to the tip, and the root ends of the two stubs (the flank
// crossings themselves when embedded).
type toothOutline struct {
	left, right         []pt // flank points, root end of the flank first
	leftRoot, rightRoot pt   // where the loop meets the root circle
	embedded            bool
}

// outlineAt returns the tooth boundary at angle. In the non-embedded shape
// the flank starts on the base circle and a radial stub reaches the root
// circle. In the embedded shape the samples inside the root circle are
// replaced by the analytic flank point on the root circle.
func (g gear) outlineAt(angle float64) toothOutline {
	left, right := g.flanks(angle)
	o := toothOutline{embedded: g.Embedded()}
	if !o.embedded {
		o.left, o.right = left, right
		o.leftRoot, o.rightRoot = g.rootEnd(left[0]), g.rootEnd(right[0])
		return o
	}
	lc, rc := g.flankPointAt(g.Root, angle)
	o.left, o.right = []pt{lc}, []pt{rc}
	for i := range left {
		if math.Hypot(left[i].x, left[i].y) > g.Root {
			o.left = append(o.left, left[i])
			o.right = append(o.right, right[i])
		}
	}
	o.leftRoot, o.rightRoot = lc, rc
	return o
}

// lateralCurves is how many boundary curves one substitute tooth contributes
// above the root circle: two chorded flanks, the tip arc, and the two stubs
// unless embedded.
func (o toothOutline) lateralCurves() int {
	n := 2*(len(o.left)-1) + 1
	if !o.embedded {
		n += 2
	}
	return n
}

// drawTooth adds one tooth's chorded flanks, tip arc and stubs to s, offset
// to the anchor, and returns the two points where the loop meets the root
// circle so the caller can close it with a root arc.
func drawTooth(t *testing.T, s *sketch.Sketch, g gear, centre *sketch.Point, o toothOutline) (leftRoot, rightRoot *sketch.Point) {
	t.Helper()
	at := func(p pt) *sketch.Point { return s.CreatePoint(p.x+g.anchorX, p.y+g.anchorY) }
	leftPts := make([]*sketch.Point, len(o.left))
	rightPts := make([]*sketch.Point, len(o.right))
	for i := range o.left {
		leftPts[i] = at(o.left[i])
		rightPts[i] = at(o.right[i])
	}
	for i := 0; i+1 < len(o.left); i++ {
		s.CreateLine(leftPts[i], leftPts[i+1])
		s.CreateLine(rightPts[i], rightPts[i+1])
	}
	last := len(o.left) - 1
	s.CreateArc(centre, rightPts[last], leftPts[last])
	if o.embedded {
		return leftPts[0], rightPts[0]
	}
	leftRoot, rightRoot = at(o.leftRoot), at(o.rightRoot)
	s.CreateLine(leftRoot, leftPts[0])
	s.CreateLine(rightRoot, rightPts[0])
	return leftRoot, rightRoot
}

// singleProfile returns the one region a substitute sketch closes.
func singleProfile(t *testing.T, s *sketch.Sketch, what string) *sketch.Profile {
	t.Helper()
	solve(t, s)
	profs := s.Profiles()
	if len(profs) != 1 {
		t.Fatalf("%s sketch closes %d regions, want 1", what, len(profs))
	}
	if !profs[0].Valid {
		t.Fatalf("%s region is not a valid profile", what)
	}
	return profs[0]
}

// toothSketch draws the substitute single-tooth sketch: the tooth loop with
// an explicit root arc where the Gear Profile sketch has the split root
// circle.
func toothSketch(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newXYSketch(t)
	centre := s.CreatePoint(g.anchorX, g.anchorY)
	leftRoot, rightRoot := drawTooth(t, s, g, centre, g.outlineAt(0))
	s.CreateArc(centre, rightRoot, leftRoot)
	return s, singleProfile(t, s, "substitute tooth")
}

// discSketch draws the Gear Body's section on its own: the solid root circle
// on the anchor, which is the region the real Gear Profile sketch closes
// inside the root circle.
func discSketch(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newXYSketch(t)
	centre := s.CreatePoint(g.anchorX, g.anchorY)
	s.CreateCircle(centre, g.Root)
	return s, singleProfile(t, s, "Gear Body disc")
}

// gearSketch draws the whole gear outline as one loop: every tooth at its
// patterned angle, joined by the root-circle valley arcs between them. It is
// the substitute for pattern-and-combine.
func gearSketch(t *testing.T, g gear) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s := newXYSketch(t)
	centre := s.CreatePoint(g.anchorX, g.anchorY)
	n := int(g.toothNumber)
	lefts := make([]*sketch.Point, n)
	rights := make([]*sketch.Point, n)
	for k := range n {
		angle := 2 * math.Pi * float64(k) / g.toothNumber
		lefts[k], rights[k] = drawTooth(t, s, g, centre, g.outlineAt(angle))
	}
	for k := range n {
		s.CreateArc(centre, lefts[k], rights[(k+1)%n])
	}
	return s, singleProfile(t, s, "gear outline")
}

// extrudeTo sweeps a profile from the sketch plane by thickness, the
// counterpart of the to-entity extent onto the Extrusion End Plane.
//
// Steps 1 and 2b of the step list, the coplanar construction plane and the
// Extrusion End Plane, have no counterpart here: decad has no construction
// planes and no plane extent, so the plane at Thickness is what the Distance
// extent below means, and every prism assertion checks that the body ends at
// z = Thickness.
func extrudeTo(t *testing.T, doc *decad.Document, s *sketch.Sketch, prof *sketch.Profile, thickness float64) *decad.Body {
	t.Helper()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(thickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("extrude: %v", err)
	}
	return body
}

// recordedArea is the exact area decad records for a profile, mm^2.
func recordedArea(t *testing.T, s *sketch.Sketch, prof *sketch.Profile) float64 {
	t.Helper()
	rec, _, err := decad.RecordProfile(s, prof)
	if err != nil {
		t.Fatalf("record profile: %v", err)
	}
	a, err := rec.Area()
	if err != nil {
		t.Fatalf("profile area: %v", err)
	}
	return mm(a)
}

// volume reads a body's volume in mm^3.
func volume(t *testing.T, body *decad.Body, what string) float64 {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	return mm(v)
}

// assertPrism checks a body is the given area swept from z=0 to z=thickness.
func assertPrism(t *testing.T, body *decad.Body, area, thickness float64, what string) {
	t.Helper()
	if v := volume(t, body, what); !near(v, area*thickness, 1e-9) {
		t.Fatalf("%s volume %.9f, want area %.9f x thickness %.3f = %.9f", what, v, area, thickness, area*thickness)
	}
	assertSpansThickness(t, body, thickness, what)
}

// assertSpansThickness checks a body runs from the sketch plane to the
// Extrusion End Plane and no further.
func assertSpansThickness(t *testing.T, body *decad.Body, thickness float64, what string) {
	t.Helper()
	b, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", what, err)
	}
	if !near(b.Min.Z, 0, 1e-9) || !near(b.Max.Z, thickness, 1e-9) {
		t.Fatalf("%s spans z in [%.6f, %.6f], want [0, %.3f]", what, b.Min.Z, b.Max.Z, thickness)
	}
}

// ---------------------------------------------------------------- step 7

// stepExtrudeTooth extrudes the single tooth profile from the target plane
// to the Extrusion End Plane as a new body.
//
// It first offers decad the tooth region of the real Gear Profile sketch and
// requires the refusal described above, so a future engine that records the
// split root circle is noticed here; then it extrudes the substitute.
func stepExtrudeTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	real := newXYSketch(t)
	drawGearProfile(t, real, g)
	solve(t, real)
	var toothProf *sketch.Profile
	for _, prof := range real.Profiles() {
		if len(prof.Entities) > 1 {
			toothProf = prof
		}
	}
	if toothProf == nil {
		t.Fatalf("Gear Profile sketch closes no tooth region")
	}
	_, err := doc.Extrude(real, toothProf, decad.Distance{D: units.Millimeters(g.thickness), Dir: decad.Along})
	if err == nil {
		t.Fatalf("decad extruded the real tooth profile: the substitute below is no longer needed, extrude the real one")
	}
	if !errors.Is(err, decad.ErrUnrecordableProfile) {
		t.Fatalf("extrude real tooth profile: %v", err)
	}
	s, prof := toothSketch(t, g)
	return []*decad.Body{extrudeTo(t, doc, s, prof, g.thickness)}
}

// assertExtrudeTooth checks the tooth prism: one body, the tooth area swept
// through Thickness, with a lateral face per boundary curve plus two caps.
func assertExtrudeTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	if len(bodies) != 1 {
		t.Fatalf("tooth extrude left %d bodies, want 1", len(bodies))
	}
	s, prof := toothSketch(t, g)
	assertPrism(t, bodies[0], recordedArea(t, s, prof), g.thickness, "tooth")
	wantFaces := 2 + g.outlineAt(0).lateralCurves() + 1 // caps, flanks/tip/stubs, root arc
	if n := len(bodies[0].Faces()); n != wantFaces {
		t.Fatalf("tooth prism has %d faces, want %d", n, wantFaces)
	}
}

// ---------------------------------------------------------------- step 9

// stepExtrudeBody extrudes the disc inside the root circle from the target
// plane to the Extrusion End Plane as a new body, from the real Gear Profile
// sketch: that region is bounded by the whole root circle, which decad can
// record.
func stepExtrudeBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	s := newXYSketch(t)
	drawGearProfile(t, s, g)
	solve(t, s)
	var disc *sketch.Profile
	for _, prof := range s.Profiles() {
		if len(prof.Entities) == 1 {
			disc = prof
		}
	}
	if disc == nil {
		t.Fatalf("Gear Profile sketch closes no disc region")
	}
	return []*decad.Body{extrudeTo(t, doc, s, disc, g.thickness)}
}

// assertExtrudeBody checks the Gear Body: a cylinder of Root Circle Radius on
// the anchor's axis, whose two planar faces are the near cap on the sketch
// plane and the far cap at Thickness. The far cap is ctx.extrusionExtent and
// the cylinder is what the Gear Center axis is built from.
func assertExtrudeBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	if len(bodies) != 1 {
		t.Fatalf("body extrude left %d bodies, want 1", len(bodies))
	}
	body := bodies[0]
	assertPrism(t, body, math.Pi*g.Root*g.Root, g.thickness, "gear body")
	var cylinders, nearCaps, farCaps int
	for _, f := range body.Faces() {
		switch surf := f.Surface().(type) {
		case decad.Cylinder:
			cylinders++
			if !near(surf.Radius.Base(), g.Root, 1e-9) {
				t.Fatalf("cylinder radius %.6f, want Root Circle Radius %.6f", surf.Radius.Base(), g.Root)
			}
			if math.Abs(math.Abs(surf.Axis.Dot(zAxis))-1) > 1e-9 {
				t.Fatalf("cylinder axis %v is not the plane normal", surf.Axis)
			}
			// Gear Center: the axis passes through the anchor.
			off := surf.Origin.Sub(r3.NewVec(g.anchorX, g.anchorY, surf.Origin.Z))
			if off.Len() > 1e-9 {
				t.Fatalf("cylinder axis misses the anchor by %v", off)
			}
		case decad.Plane:
			n := surf.Frame.N()
			if math.Abs(math.Abs(n.Dot(zAxis))-1) > 1e-9 {
				t.Fatalf("planar face normal %v is not parallel to the sketch plane", n)
			}
			switch z := surf.Frame.Origin().Z; {
			case near(z, 0, 1e-9):
				nearCaps++
			case near(z, g.thickness, 1e-9):
				farCaps++
			default:
				t.Fatalf("planar face at z=%.6f is neither cap", z)
			}
		default:
			t.Fatalf("unexpected %T face on the gear body", surf)
		}
	}
	if cylinders != 1 || nearCaps != 1 || farCaps != 1 {
		t.Fatalf("gear body faces: %d cylinder, %d coplanar cap, %d far cap; want 1/1/1", cylinders, nearCaps, farCaps)
	}
	// The far cap is the one planar face whose outward normal leaves the
	// sketch plane, which is how the bore step names it.
	if faces, err := farCap(body).SelectFaces(body); err != nil || len(faces) != 1 {
		t.Fatalf("far cap selection: %d faces, err %v", len(faces), err)
	}
}

// farCap selects a body's end cap away from the sketch plane: parallel to
// it and facing along its normal. It stands for ctx.extrusionExtent.
func farCap(body *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.Planar(), decad.Facing(zAxis)).Exactly(1)
}

// --------------------------------------------------------------- step 10a

// patternStep is the rotation between two neighbouring pattern instances.
func (g gear) patternStep(t *testing.T) r3.Transform {
	t.Helper()
	rot, err := r3.RotationAround(r3.NewVec(g.anchorX, g.anchorY, 0), zAxis, units.Degrees(360/g.toothNumber))
	if err != nil {
		t.Fatalf("pattern step rotation: %v", err)
	}
	return rot
}

// stepPatternTeeth patterns the tooth body about the Gear Center axis: Tooth
// Number instances over a full turn, 360/N apart.
//
// decad has no pattern feature, and its verification cannot decide whether
// two neighbouring tooth prisms are disjoint (measured on the pinned engine:
// an undecided_pair diagnostic on instances 6 and 7 of a 17-tooth pattern),
// so Tooth Number instances cannot stand in one document. The substitute
// walks the one tooth through the N-1 placements instead: each Placed step
// rotates the body by 360/N about the anchor's axis and retires the body it
// moved, so the recipe records every instance's motion and one body is live
// at the end. What it pins: the count, the angular step, the axis, and that
// each instance is the seed moved rigidly. What it cannot pin: N bodies
// existing at once, which the combine substitute in step 10b covers.
func stepPatternTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	s, prof := toothSketch(t, g)
	body := extrudeTo(t, doc, s, prof, g.thickness)
	step := g.patternStep(t)
	for k := 1; k < int(g.toothNumber); k++ {
		moved, err := body.Placed(step)
		if err != nil {
			t.Fatalf("pattern instance %d: %v", k, err)
		}
		body = moved
	}
	return []*decad.Body{body}
}

// assertPatternTeeth reads the pattern back from the recipe: one extrude then
// Tooth Number minus one placements, each the same 360/N rotation about the
// anchor's axis and each moving the body the previous one produced, and the
// live body at the end sits where N-1 steps put the seed with its volume
// unchanged.
func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	n := int(g.toothNumber)
	if len(bodies) != 1 {
		t.Fatalf("pattern left %d live bodies, want 1", len(bodies))
	}
	steps := doc.Recipe().Steps
	if len(steps) != n {
		t.Fatalf("recipe holds %d steps, want the extrude plus %d placements", len(steps), n-1)
	}
	if steps[0].Op != decad.OpExtrude {
		t.Fatalf("recipe step 0 is %v, want the tooth extrude", steps[0].Op)
	}
	want := g.patternStep(t)
	centre := r3.NewVec(g.anchorX, g.anchorY, 0)
	probe := r3.NewVec(g.anchorX+g.Tip, g.anchorY, g.thickness)
	for k := 1; k < n; k++ {
		st := steps[k]
		if st.Op != decad.OpPlaced {
			t.Fatalf("recipe step %d is %v, want a placement", k, st.Op)
		}
		if len(st.Inputs) != 1 || int(st.Inputs[0]) != k-1 {
			t.Fatalf("placement %d moves step %v, want the body step %d produced", k, st.Inputs, k-1)
		}
		got, err := st.Placement.Transform()
		if err != nil {
			t.Fatalf("placement %d: %v", k, err)
		}
		if !got.Apply(centre).Equal(centre, 1e-9) {
			t.Fatalf("placement %d moves the anchor to %v: not a rotation about the Gear Center", k, got.Apply(centre))
		}
		if !got.Apply(probe).Equal(want.Apply(probe), 1e-9) {
			t.Fatalf("placement %d turns a tip point to %v, want %v (360/%d about the axis)", k, got.Apply(probe), want.Apply(probe), n)
		}
	}
	ts, tp := toothSketch(t, g)
	toothArea := recordedArea(t, ts, tp)
	last := bodies[0]
	if v := volume(t, last, "last instance"); !near(v, toothArea*g.thickness, 1e-9) {
		t.Fatalf("last instance volume %.9f, want the seed's %.9f", v, toothArea*g.thickness)
	}
	assertSpansThickness(t, last, g.thickness, "last instance")
	// Where N-1 steps put the seed: the seed's own centroid is read from a
	// fresh tooth prism in a scratch document, since the seed here is retired.
	scratch := decad.New()
	seed := extrudeTo(t, scratch, ts, tp, g.thickness)
	c0, err := seed.Centroid()
	if err != nil {
		t.Fatalf("seed centroid: %v", err)
	}
	c, err := last.Centroid()
	if err != nil {
		t.Fatalf("last instance centroid: %v", err)
	}
	total, err := r3.RotationAround(centre, zAxis, units.Degrees(360*float64(n-1)/g.toothNumber))
	if err != nil {
		t.Fatalf("total rotation: %v", err)
	}
	if !c.Value.Equal(total.Apply(c0.Value), 1e-6) {
		t.Fatalf("last instance centroid %v, want %v", c.Value, total.Apply(c0.Value))
	}
}

// --------------------------------------------------------------- step 10b

// stepCombineTeeth joins the patterned teeth into the Gear Body. decad's
// boolean refuses operands that share a face (the tooth's root arc lies on
// the body's root cylinder), so the joined gear is built as one prism from
// the substitute whole-gear outline, and the assertion pins what the join
// has to produce.
func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	g := derive(t, p)
	s, prof := gearSketch(t, g)
	return []*decad.Body{extrudeTo(t, doc, s, prof, g.thickness)}
}

// assertCombineTeeth checks the joined gear against the parts it joins: its
// volume is the disc's plus Tooth Number tooth prisms, its faces are the two
// caps plus a lateral face per tooth curve and per valley arc, in one lump.
func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	g := derive(t, p)
	if len(bodies) != 1 {
		t.Fatalf("combine left %d bodies, want 1", len(bodies))
	}
	gear := bodies[0]
	ts, tp := toothSketch(t, g)
	toothArea := recordedArea(t, ts, tp)
	discArea := math.Pi * g.Root * g.Root
	n := int(g.toothNumber)
	assertPrism(t, gear, discArea+float64(n)*toothArea, g.thickness, "joined gear")
	if want, got := 2+n*(g.outlineAt(0).lateralCurves()+1), len(gear.Faces()); got != want {
		t.Fatalf("joined gear has %d faces, want %d", got, want)
	}
	if lumps := gear.Lumps(); len(lumps) != 1 {
		t.Fatalf("joined gear is %d lumps, want 1", len(lumps))
	}
}
