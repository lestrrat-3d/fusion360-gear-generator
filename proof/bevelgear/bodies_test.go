package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
)

// apexStub is how close to the apex the tooth loft's degenerate end is taken.
//
// The spec lofts the §2 Apex SKETCH POINT to the tooth profile — a point section.
// decad's loft rules between two profiles and takes no point section, so the proof
// lofts from the tooth section scaled to this fraction of the way out from the
// apex. What it costs is a 2 per cent stub at the tip; the conical trim in the
// next step removes the whole apex side well outside it, so nothing downstream
// sees the difference.
const apexStub = 0.02

// stepLoftTooth lofts the uncut apex-to-heel tooth body: the §2 Apex to this
// gear's §3 tooth profile.
func stepLoftTooth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	body, err := loftBetween(doc, g.section(apexStub), g.section(1))
	if err != nil {
		t.Fatalf("loft the tooth from the apex to the tooth profile: %v", err)
	}
	return []*decad.Body{body}
}

// assertLoftTooth checks the taper: a body lofted from a point to a section is a
// cone over that section, so its volume is a third of the section area times the
// apex's perpendicular distance to the section plane, less the stub.
func assertLoftTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the tooth loft returned %d bodies, want 1", len(bodies))
	}
	area := sectionArea(g.section(1))
	want := area * g.apexDist / 3 * (1 - apexStub*apexStub*apexStub)
	near(t, "the lofted tooth's volume against the cone over its section",
		mustVolume(t, bodies[0]), want, want*0.005)

	// The tooth profile sits at the heel: it is drawn on the back-cone plane, so
	// every point of it lies within a tooth height of the heel's cone distance.
	// Its root end falls a little SHORT of the dedendum corner, and by a bounded
	// amount: the virtual tooth number is a floor, so the virtual pitch radius the
	// tooth is drawn at is up to half a module under the radius the lattice
	// measures, and the tooth's root sits proud of the root cone by that much.
	lo, hi := math.Inf(1), 0.0
	for _, pt := range g.section(1) {
		d := g.distAlong(vec{pt.X, pt.Y})
		lo, hi = math.Min(lo, d), math.Max(hi, d)
	}
	if offset := math.Abs(g.distAlong(g.heel) - hi); offset >= p.Module/2 {
		t.Errorf("the tooth profile's root end misses the dedendum corner by %.4f mm along "+
			"the cone element; the virtual tooth number is a floor, which bounds that by half "+
			"a module, %.4f mm", offset, p.Module/2)
	}
	if lo > g.distAlong(g.heel) {
		t.Errorf("the whole tooth profile sits past the heel corner at cone distance %.4f", lo)
	}
}

// sectionArea is the planar area of a closed loop of world points, by the
// shoelace formula in the loop's own plane.
func sectionArea(pts []r3.Vec) float64 {
	origin := pts[0]
	u := pts[1].Sub(origin)
	var normal r3.Vec
	for _, p := range pts[2:] {
		if n := u.Cross(p.Sub(origin)); n.Len() > 1e-12 {
			normal = n
			break
		}
	}
	unitNormal, ok := normal.Normalize()
	if !ok {
		return 0
	}
	total := r3.NewVec(0, 0, 0)
	for i := range pts {
		total = total.Add(pts[i].Cross(pts[(i+1)%len(pts)]))
	}
	return math.Abs(total.Dot(unitNormal)) / 2
}

// solidCases: both gears, both ratio directions, the shaft-angle ends the lattice
// holds across, and the module and tooth-count changes that move the virtual
// tooth number across the embedded threshold.
var solidCases = []proofkit3d.Case{
	{Name: "default_pinion", Params: map[string]float64{"pinion": 1}},
	{Name: "default_driving", Params: map[string]float64{"pinion": 0}},
	{Name: "ratio_pinion_17", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
	{Name: "ratio_driving_31", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 0}},
	{Name: "shaft_angle_30_pinion", Params: map[string]float64{
		"shaftAngle": 30, "drivingBaseHeight": 2, "pinion": 1}},
	{Name: "shaft_angle_135_driving", Params: map[string]float64{
		"shaftAngle": 135, "pinion": 0}},
	{Name: "module_2_pinion_13", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 1}},
	{Name: "face_width_given_pinion", Params: map[string]float64{
		"faceWidth": 4, "pinion": 1}},
}

// stepRevolveGearBody builds the gear body the hexagon profile is revolved into.
//
// What it substitutes, and why. decad's revolve is the one feature verb whose
// bodies neither proofkit3d gate accepts: every revolve it builds reports a
// volume reading beyond the default tolerance — decad's own revolve tests assert
// exactly that, Suspect and not trustworthy — and RunSolid admits only an area or
// centroid reading. Its booleans refuse a revolve payload too. So the proof lofts
// the frustum's ROOT-CONE BAND between two coaxial circles: the same surface the
// hexagon's C->M edge sweeps, over the same toe-to-heel span, built the way the
// harness accepts. What that costs is the other four faces — the back face, the
// front annulus, and the heel and toe cones — so this step proves the seating
// cone and its taper, and the profile's own one-loop, one-side-of-the-axis
// shape is proved where it is drawn, in the profile sketch step.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	toe, heel := g.toe, g.heel
	body, err := loftBetween(doc,
		diskSection(toe.X, math.Abs(toe.Y), 96),
		diskSection(heel.X, math.Abs(heel.Y), 96))
	if err != nil {
		t.Fatalf("loft the gear body's root-cone band: %v", err)
	}
	return []*decad.Body{body}
}

func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the revolve returned %d bodies, want the one gear body", len(bodies))
	}
	toe, heel := g.toe, g.heel
	// Both radii are the perpendicular distances of the toe and heel dedendum
	// corners from the shaft axis, which is what the hexagon edge sweeps.
	rToe, rHeel := math.Abs(toe.Y), math.Abs(heel.Y)
	height := math.Abs(heel.X - toe.X)
	want := math.Pi * height * (rToe*rToe + rToe*rHeel + rHeel*rHeel) / 3
	near(t, "the gear body's root-cone band volume", mustVolume(t, bodies[0]), want, want*0.005)
	if rHeel <= rToe {
		t.Errorf("the root cone reaches %.4f mm at the heel and %.4f mm at the toe; it must "+
			"widen from the apex outward", rHeel, rToe)
	}
	// The band's lateral surface is the cone through both corners, so the two
	// corners lie on one line through the apex — that line is the root cone
	// element the whole spiral frame is built on.
	near(t, "the toe corner's distance off the root cone element",
		math.Abs(cross(toe, g.coneVec)), 0, 1e-9)
	near(t, "the heel corner's distance off the root cone element",
		math.Abs(cross(heel, g.coneVec)), 0, 1e-9)
}

// stepTrimToothBand builds the flush band the two conical trims leave: the tooth
// between the toe and the heel.
//
// What it substitutes. The spec trims by SPLITTING the tooth with two
// ConeSurfaceType faces of the revolved gear body and keeping the middle piece.
// decad has no split verb, and its booleans refuse both the payloads involved:
// the tooth is a loft and the cone would be a revolve, and its own refusal names
// prism, cup and faceted as the payloads it tessellates. So the proof lofts the
// band directly between the tooth's cross-sections AT the toe and heel cone
// distances. What that costs is the shape of the two end faces: the trim leaves
// conical ends that seat flush on the gear body, and the substitute leaves the
// planar sections through the same cone distances. The band's extent, its
// cross-sections and the volume between them are the same.
func stepTrimToothBand(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	body, err := loftBetween(doc, g.section(g.toeParam()), g.section(g.heelParam()))
	if err != nil {
		t.Fatalf("loft the trimmed band between the toe and heel sections: %v", err)
	}
	return []*decad.Body{body}
}

// toeParam and heelParam are the tooth-plane parameters at which the tooth's
// section sits at the toe and heel cone distances. The tooth is a cone over its
// section, so the parameter is the ratio of cone distances.
func (g gearGeometry) toeParam() float64 {
	return g.distAlong(g.toeMid) / g.distAlong(g.heelMid) * g.heelParam()
}

func (g gearGeometry) heelParam() float64 { return 1 }

func assertTrimToothBand(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the trim returned %d bodies, want the one keeper", len(bodies))
	}
	toe, heel := g.toeParam(), g.heelParam()
	area := sectionArea(g.section(1))
	// The band is the difference of two cones over the same section.
	want := area * g.apexDist / 3 * (heel*heel*heel - toe*toe*toe)
	near(t, "the trimmed band's volume", mustVolume(t, bodies[0]), want, want*0.005)

	// The heel MUST be the outer end. A negative span silently inverts the whole
	// spiral frame downstream, so it is checked here where the band is built.
	span := g.distAlong(g.heelMid) - g.distAlong(g.toeMid)
	if span <= 0 {
		t.Fatalf("the heel edge midpoint is at cone distance %.4f and the toe edge midpoint "+
			"at %.4f; the heel must be the OUTER end", g.distAlong(g.heelMid),
			g.distAlong(g.toeMid))
	}

	// And the band's extent is the face width, read along the cone element. The
	// two dedendum corners lie ON the element, so the distance between them is the
	// dimensioned face width divided by the sine of the angle the dedendum line
	// makes with the element — exactly, not approximately.
	l := solveLattice(p)
	sine := math.Abs(cross(g.hat, g.coneVec))
	near(t, "the corner-to-corner extent along the cone element",
		g.distAlong(g.heel)-g.distAlong(g.toe), l.FaceWidth/sine, 1e-6)
}

// stepPatternTeeth places this gear's teeth around the shaft axis: the tooth,
// plus one copy per remaining tooth, at 360 degrees over the tooth number.
//
// Every body it returns is a prism over the tooth's transverse footprint rather
// than the tapered tooth — see [gearGeometry.transverse] for why and for what
// that costs. The gate is what makes the step worth proving: decad's verification
// reports any proven overlap between two bodies as a diagnostic, so a pattern
// whose teeth collided would fail here rather than in the viewport.
func stepPatternTeeth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	seed, err := extrudeAlongShaft(doc, g.transverse(), g.faceWidth(p))
	if err != nil {
		t.Fatalf("extrude the seed tooth: %v", err)
	}
	bodies := []*decad.Body{seed}
	quantity := int(g.teeth)
	for i := 1; i < quantity; i++ {
		xf, err := rotationAboutShaft(2 * math.Pi * float64(i) / float64(quantity))
		if err != nil {
			t.Fatalf("pattern rotation %d: %v", i, err)
		}
		copyBody, err := seed.PlacedCopy(xf)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", i, err)
		}
		bodies = append(bodies, copyBody)
	}
	return bodies
}

// faceWidth is the resolved face width for this gear's case.
func (g gearGeometry) faceWidth(p params) float64 { return solveLattice(p).FaceWidth }

func assertPatternTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	quantity := int(g.teeth)
	if len(bodies) != quantity {
		t.Fatalf("the pattern produced %d bodies for %d teeth; the pattern's own bodies "+
			"already include the seed", len(bodies), quantity)
	}
	seedVolume := mustVolume(t, bodies[0])
	for i, body := range bodies {
		near(t, "patterned tooth volume", mustVolume(t, body), seedVolume, seedVolume*1e-6)
		centre := centroidOf(t, body)
		want := math.Atan2(centroidOf(t, bodies[0]).Z, centroidOf(t, bodies[0]).Y) +
			2*math.Pi*float64(i)/float64(quantity)
		got := math.Atan2(centre.Z, centre.Y)
		near(t, "patterned tooth azimuth", math.Mod(got-want+3*math.Pi, 2*math.Pi)-math.Pi,
			0, 1e-9)
	}

	// The teeth clear one another because a tooth's angular half-width about the
	// shaft is under half the angular pitch. That is what makes the whole circle
	// hold N copies, and it is measured off the section the proof actually drew.
	half := angularExtent(g.transverse())
	pitch := 2 * math.Pi / float64(quantity)
	if 2*half >= pitch {
		t.Errorf("one tooth spans %.6f rad about the shaft and the angular pitch is %.6f rad; "+
			"the patterned teeth would collide", 2*half, pitch)
	}
}

// patternCases keeps the tooth count small enough that every pair of patterned
// bodies is checked in reasonable time, and covers both gears and both ratio
// directions at the counts the input validation admits.
var patternCases = []proofkit3d.Case{
	{Name: "eight_teeth_pinion", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 1}},
	{Name: "eight_teeth_driving", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 0}},
	{Name: "ratio_13_pinion", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 1}},
	{Name: "ratio_19_driving", Params: map[string]float64{
		"module": 2, "drivingTeeth": 19, "pinionTeeth": 13, "pinion": 0}},
}

// stepMeshRotate applies the driving gear's meshing offset: half a tooth pitch
// about its own shaft axis, so a driving valley sits where the pinion tooth
// crosses the axial plane. The pinion's own extra phase is zero unless a spiral
// pair asks for one.
func stepMeshRotate(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	body, err := extrudeAlongShaft(doc, g.transverse(), g.faceWidth(p))
	if err != nil {
		t.Fatalf("extrude the body to rotate: %v", err)
	}
	xf, err := rotationAboutShaft(meshPhase(p))
	if err != nil {
		t.Fatalf("meshing rotation: %v", err)
	}
	rotated, err := body.Placed(xf)
	if err != nil {
		t.Fatalf("rotate the gear body about its shaft edge: %v", err)
	}
	return []*decad.Body{rotated}
}

// meshPhase is the extra rotation this gear receives: half a tooth pitch for the
// driving gear, and the pinion's spiral phase, which is zero teeth by default,
// for the pinion.
func meshPhase(p params) float64 {
	if p.Pinion {
		return pinionMeshPhaseTeeth * 2 * math.Pi / p.PinionTeeth
	}
	return math.Pi / p.DrivingTeeth
}

// pinionMeshPhaseTeeth is the pinion's extra mesh rotation in tooth fractions.
// It is zero: the spiral build leaves the mid-face section unrotated, so the
// pinion already meshes where the straight tooth did.
const pinionMeshPhaseTeeth = 0.0

func assertMeshRotate(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the meshing rotation returned %d bodies, want 1", len(bodies))
	}
	// The rotation is about the shaft axis, so every radius is unchanged and the
	// azimuth moves by exactly the phase.
	before := g.transverse()
	centre := centroidOf(t, bodies[0])
	wantAzimuth := azimuthOf(before) + meshPhase(p)
	near(t, "the rotated body's azimuth", math.Atan2(centre.Z, centre.Y), wantAzimuth, 1e-6)
	near(t, "the rotated body's radius", math.Hypot(centre.Y, centre.Z),
		math.Hypot(centroidVec(before).Y, centroidVec(before).Z), 1e-6)
	if p.Pinion {
		near(t, "the pinion's extra mesh phase", meshPhase(p), 0, 0)
	} else {
		near(t, "the driving gear's meshing offset against half a tooth pitch",
			meshPhase(p), math.Pi/p.DrivingTeeth, 0)
	}
}

// centroidVec is the AREA centroid of a transverse loop, in the plane
// perpendicular to the shaft. A prism's volume centroid projects onto it, so it
// is the reference the meshing rotation is measured against.
func centroidVec(pts []r3.Vec) r3.Vec {
	a, cy, cz := 0.0, 0.0, 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		c := pts[i].Y*pts[j].Z - pts[j].Y*pts[i].Z
		a += c
		cy += (pts[i].Y + pts[j].Y) * c
		cz += (pts[i].Z + pts[j].Z) * c
	}
	a /= 2
	return r3.NewVec(pts[0].X, cy/(6*a), cz/(6*a))
}

func azimuthOf(pts []r3.Vec) float64 {
	c := centroidVec(pts)
	return math.Atan2(c.Z, c.Y)
}

// meshCases covers both gears — the driving gear takes the half-pitch offset and
// the pinion takes none — over both ratio directions.
var meshCases = []proofkit3d.Case{
	{Name: "driving_half_pitch", Params: map[string]float64{"pinion": 0}},
	{Name: "pinion_no_phase", Params: map[string]float64{"pinion": 1}},
	{Name: "ratio_driving_31", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 0}},
	{Name: "ratio_pinion_17", Params: map[string]float64{
		"drivingTeeth": 31, "pinionTeeth": 17, "pinion": 1}},
}

// hubOverlap is how far the stand-in gear body reaches past the tooth's root
// radius, as a fraction of it. In the gear the tooth's root edge lies ON the
// root cone the body already carries, and the Combine-Join meets it there; two
// bodies that only touch are not a boolean the evaluator will join, so the
// stand-in body reaches this much further out.
const hubOverlap = 0.02

// stepCombineTeeth joins every patterned tooth to the gear body in one
// Combine-Join, the gear body as the target and the patterned teeth as the tools.
//
// It builds on the same transverse prisms the pattern step does, and stands the
// gear body up as a cylinder reaching the tooth roots. What it costs is stated
// at [gearGeometry.transverse]; what it keeps is the step's own content, that
// one body comes out of the join with every tooth attached.
func stepCombineTeeth(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	section := g.transverse()
	rootRadius, _ := radialExtent(section)
	length := g.faceWidth(p)

	// The stand-in body runs past the teeth at both ends, so no pair of operand
	// faces is coplanar: the evaluator refuses a boolean whose facets meet in one
	// plane, and the gear's own body genuinely does run past its teeth.
	target, err := extrudeAlongShaft(doc,
		diskSection(section[0].X-0.1*length, rootRadius*(1+hubOverlap), 64), 1.2*length)
	if err != nil {
		t.Fatalf("stand the gear body up as the join target: %v", err)
	}
	seed, err := extrudeAlongShaft(doc, section, length)
	if err != nil {
		t.Fatalf("extrude the seed tooth: %v", err)
	}
	// Every patterned copy is made before the first join: a boolean consumes its
	// operands, so the seed has to be copied while it is still live.
	quantity := int(g.teeth)
	tools := []*decad.Body{seed}
	for i := 1; i < quantity; i++ {
		xf, err := rotationAboutShaft(2 * math.Pi * float64(i) / float64(quantity))
		if err != nil {
			t.Fatalf("pattern rotation %d: %v", i, err)
		}
		tool, err := seed.PlacedCopy(xf)
		if err != nil {
			t.Fatalf("pattern copy %d: %v", i, err)
		}
		tools = append(tools, tool)
	}
	joined := target
	for i, tool := range tools {
		if joined, err = decad.Union(joined, tool); err != nil {
			t.Fatalf("join tooth %d to the gear body: %v", i, err)
		}
	}
	return []*decad.Body{joined}
}

func assertCombineTeeth(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the Combine-Join left %d bodies; a single join leaves one", len(bodies))
	}
	if lumps := len(bodies[0].Lumps()); lumps != 1 {
		t.Fatalf("the joined body has %d lumps; a tooth that did not reach the body would "+
			"leave its own", lumps)
	}
	// Every tooth is attached, so the joined volume is the body plus the teeth
	// less the overlap each one buries in it — bounded below by the body alone and
	// above by the body plus every tooth.
	section := g.transverse()
	rootRadius, _ := radialExtent(section)
	length := g.faceWidth(p)
	hub := math.Pi * math.Pow(rootRadius*(1+hubOverlap), 2) * 1.2 * length
	tooth := sectionArea(section) * length
	got := mustVolume(t, bodies[0])
	if got <= hub || got >= hub+float64(int(g.teeth))*tooth {
		t.Errorf("the joined body measures %.4f mm3; the gear body alone is %.4f and the body "+
			"plus every tooth whole is %.4f", got, hub, hub+float64(int(g.teeth))*tooth)
	}
}

// stepCutBore cuts the through bore: a cylinder of the resolved bore diameter,
// along the shaft axis, restricted to this gear's body.
func stepCutBore(t *testing.T, doc *decad.Document, pm map[string]float64) []*decad.Body {
	p := read(pm)
	g := gearOf(p)
	if !p.BoreEnable {
		proofkit3d.Unmodelled(t, "Enable Bore is unchecked, so no bore is cut and there is "+
			"no body for this step to produce")
		return nil
	}
	section := g.transverse()
	rootRadius, _ := radialExtent(section)
	length := g.faceWidth(p)
	target, err := extrudeAlongShaft(doc, diskSection(section[0].X, rootRadius, 64), length)
	if err != nil {
		t.Fatalf("stand the gear body up as the cut target: %v", err)
	}
	// The cut is symmetric about the bore plane and generously longer than the
	// body on each side, so it pierces whatever the face width turns out to be.
	reach := 2 * coneDistanceVariable(p)
	tool, err := extrudeAlongShaft(doc,
		diskSection(section[0].X-reach, boreDiameter(p)/2, 64), 2*reach)
	if err != nil {
		t.Fatalf("extrude the bore tool: %v", err)
	}
	bored, err := decad.Cut(target, tool)
	if err != nil {
		t.Fatalf("cut the bore through the gear body: %v", err)
	}
	return []*decad.Body{bored}
}

func assertCutBore(t *testing.T, doc *decad.Document, bodies []*decad.Body,
	pm map[string]float64) {
	p := read(pm)
	g := gearOf(p)
	if len(bodies) != 1 {
		t.Fatalf("the bore cut left %d bodies, want 1", len(bodies))
	}
	if voids := len(bodies[0].Lumps()); voids != 1 {
		t.Fatalf("the bored body has %d lumps; a through bore leaves one", voids)
	}
	section := g.transverse()
	rootRadius, _ := radialExtent(section)
	length := g.faceWidth(p)
	diameter := boreDiameter(p)
	want := math.Pi * (rootRadius*rootRadius - diameter*diameter/4) * length
	near(t, "the bored body's volume", mustVolume(t, bodies[0]), want, want*0.01)
	if diameter/2 >= rootRadius {
		t.Errorf("the bore radius %.4f mm is not inside the gear body's %.4f mm root radius",
			diameter/2, rootRadius)
	}
}

// boreSolidCases: both gears, both sides of the auto-diameter branch, and the
// unchecked Enable Bore that produces no body at all.
var boreSolidCases = []proofkit3d.Case{
	{Name: "auto_pinion", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 1}},
	{Name: "auto_driving", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 0}},
	{Name: "given_pinion", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "pinion": 1, "pinionBore": 3}},
	{Name: "bore_disabled", Params: map[string]float64{
		"drivingTeeth": 8, "pinionTeeth": 8, "boreEnable": 0}},
}
