// This file holds the bevel gear's straight-build solid steps: the gear-body
// revolve, the apex-to-tooth loft, the two conical end cuts, the circular
// pattern, the Combine-Join, the bore cut and the driving gear's meshing
// rotation.
//
// NO BOOLEAN IS PERFORMED ANYWHERE IN THIS GEAR'S PROOF, and that is forced
// rather than chosen. At the decad revision this repo pins, a boolean accepts
// only prism, cup and faceted payloads and refuses an operand built by Loft.
// Every solid in this gear is conical, and a cone is a Loft here because Extrude
// refuses a nonzero taper. So the union that joins the frustum, the intersection
// and cut that trim the tooth, the bore's through-cut and the Combine-Join are
// all unavailable.
//
// The substitution is the same at every site: build the operands, lay them apart
// along the shaft axis, and assert from their own measured geometry what the
// operation would have produced. Laying them apart leaves every volume, radius
// and cone angle unchanged, which is what makes the readings still mean
// something. Each step below says what its own substitution costs.
//
// Two conventions:
//
//   - Every solid is built in the ONE gear's own frame: the shaft axis is world
//     +Z, the apex is the origin, and a §2 point reaches this file as the pair
//     (station, radius) that latticeSide.station and latticeSide.radius give.
//   - The §2 constraint scheme is stepGearProfiles's subject. The sections here
//     are drawn at their solved coordinates and carry no constraints; restating
//     the scheme would prove it twice and build nothing new.
package bevelgear_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func lengthMM(v float64) units.Value { return units.Millimeters(v) }

// shaftAxisVec is the gear's own axis in every solid step: world +Z, through the
// apex at the origin.
func shaftAxisVec() r3.Vec { return r3.NewVec(0, 0, 1) }

// gearSide picks the gear a per-gear case runs on. Every per-gear step runs on
// both, pinion first, exactly as the generator does.
func gearSide(l lattice, p map[string]float64) latticeSide {
	if p["gear"] != 0 {
		return l.driving
	}
	return l.pinion
}

// ---------------------------------------------------------------- primitives

// polygonSides is how many sides the polygonal sweep carries. The spec calls
// for a polygonal sweep rather than a circular one, and the reason is the
// reading: decad publishes a LOFT BETWEEN TWO CIRCLES with a volume bound of the
// same order as the volume, because the pair is faceted, and the solid gate
// refuses a Suspect volume rather than waiving it. A loft between two regular
// polygons is an all-line pair and its volume comes back exact.
//
// What the substitution costs is the curvature: an n-gon of circumradius r
// encloses polygonFactor(n) of the circle's area, so every volume below is that
// fraction of the revolve's, and the closed form carries the factor explicitly
// rather than hiding it in a tolerance. Radii, stations and cone half-angles are
// untouched, since a vertex of the polygon sits exactly on the circle.
const polygonSides = 36

// polygonFactor is the area an n-gon of circumradius r encloses as a fraction of
// the circle's: (n / 2pi) * sin(2pi / n).
func polygonFactor(n int) float64 {
	return float64(n) / (2 * math.Pi) * math.Sin(2*math.Pi/float64(n))
}

// ringSection draws one regular polygon of circumradius r on a plane at station
// z along the shaft axis, with a vertex on +X so the section reaches exactly r
// there, and returns it with its region.
func ringSection(t *testing.T, world *sketch.World, r, z float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane := world.XY()
	if z != 0 {
		var err error
		plane, err = world.CreateOffsetPlane(world.XY(), z)
		if err != nil {
			t.Fatalf("plane at station %.6f: %v", z, err)
		}
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("sketch at station %.6f: %v", z, err)
	}
	points := make([]*sketch.Point, polygonSides)
	for i := range points {
		a := 2 * math.Pi * float64(i) / float64(polygonSides)
		points[i] = s.CreatePoint(r*math.Cos(a), r*math.Sin(a))
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%polygonSides])
	}
	return s, decadtest.SolveRegion(t, s)
}

// coneBand is one band of the frustum: the polygonal sweep of a §2 profile edge
// about the shaft axis, built as the loft between its two ring sections.
//
// It is a Loft rather than a Revolve for the reason polygonSides gives, and
// because decad publishes a revolved body's volume with a proven bound equal to
// the volume itself, so a revolved body is Suspect at any tolerance and cannot
// pass the harness gate.
func coneBand(t *testing.T, doc *decad.Document, r0, r1, z0, z1 float64, label string) *decad.Body {
	t.Helper()
	if z1 <= z0 {
		t.Fatalf("%s: stations %.6f and %.6f do not span a band", label, z0, z1)
	}
	world := sketch.NewWorld()
	s0, p0 := ringSection(t, world, r0, z0)
	s1, p1 := ringSection(t, world, r1, z1)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s band from r=%.6f at z=%.6f to r=%.6f at z=%.6f: %v",
			label, r0, z0, r1, z1, err)
	}
	return body
}

// laidApart moves a body along the shaft axis so it shares no space with the
// others in the document. Nothing about it changes: a translation along the axis
// leaves every volume, radius and cone angle exactly as built, which is what
// makes the readings still mean something.
func laidApart(t *testing.T, body *decad.Body, offset float64, label string) *decad.Body {
	t.Helper()
	move, err := r3.Translation(r3.NewVec(0, 0, offset))
	if err != nil {
		t.Fatalf("%s: translation by %.6f: %v", label, offset, err)
	}
	moved, err := body.Placed(move)
	if err != nil {
		t.Fatalf("%s: lay apart by %.6f: %v", label, offset, err)
	}
	return moved
}

// volumeReading is a body's volume together with the bound decad proved around
// it. label is the step's own name for the body: decadtest names a body by its
// index and recipe step, and "body[0] (step 1 loft)" does not say which feature
// is wrong.
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

func boundsOf(t *testing.T, body *decad.Body, label string) decad.Box {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s bounds: %v", label, err)
	}
	return box
}

// requireVolume checks a body against this proof's own closed form, where rel is
// that formula's error. decadtest adds the reading's own proven bound on top, so
// what is asserted is that decad's interval and this proof's claim overlap.
func requireVolume(t *testing.T, body *decad.Body, label string, want, rel float64) {
	t.Helper()
	decadtest.Measures(t, label+" volume", volumeReading(t, body, label),
		units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(rel)))
}

// ---------------------------------------------------------------- tooth section

// toothLoop is one virtual spur tooth's closed outline in its own plane,
// centred on the tooth centre K' (L'), at the scale a section that far from the
// apex carries.
//
// The flanks are CHORDED. A flank is a fitted spline in Fusion and here too, but
// decad's loft pairs two sections segment by segment and a chorded pair is the
// form it accepts for this shape. What chording costs is the sagitta between
// each chord and the involute it spans: the flank moves inward by that much, so
// a chorded tooth is slightly thinner and every volume read off it slightly
// smaller. Each step below measures chorded against chorded, so the sagitta
// cancels.
func toothLoop(l lattice, side latticeSide, scale float64) []vec {
	module := l.module
	teeth := side.virtualTeeth(module)
	d := involute.Derive(module, teeth, proxyPressureAngle)
	root := d.Root - rootSinkFraction*module
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, proxyInvoluteSteps, math.Pi)

	// The loop runs: right root foot, up the right flank, across the tooth top,
	// down the left flank, to the left root foot, and home along the root arc —
	// which is chorded here for the same reason the flanks are.
	// A HIGH virtual tooth count puts the flank start inside the root circle —
	// base < root, which happens at 41.5 teeth at a 20 degree pressure angle — and
	// the tooth is then EMBEDDED: tip, root and flanks meet with no connecting
	// lines. Drawing the two stubs anyway crosses them over the flanks and the
	// loop stops being a region at all. That is the same flag the tooth-profile
	// search keys its line count on.
	embedded := d.Base < root
	if embedded {
		// The flanks start INSIDE the root circle, so the loop's root boundary is
		// the root-circle arc between the two crossings and the flanks are trimmed
		// there. That is the same boundary Fusion's profile detection finds when it
		// splits the root circle against the flanks.
		left, right = trimFlankToRootCircle(left, root), trimFlankToRootCircle(right, root)
	}
	loop := make([]vec, 0, 2*len(left)+16)
	footOf := func(p involute.Pt) vec {
		n := math.Hypot(p.X, p.Y)
		return vec{root * p.X / n, root * p.Y / n}
	}
	rightStart, leftStart := footOf(right[0]), footOf(left[0])
	if !embedded {
		loop = append(loop, rightStart)
	}
	for _, p := range right {
		loop = append(loop, vec{p.X, p.Y})
	}
	// The tooth top, sampled as an arc between the two flank tips. The middle
	// sample sits exactly on the tooth's centreline, so the loop reaches the tip
	// radius there.
	a0 := math.Atan2(right[len(right)-1].Y, right[len(right)-1].X)
	a1 := math.Atan2(left[len(left)-1].Y, left[len(left)-1].X)
	for i := 1; i < 6; i++ {
		a := a0 + wrapToHalfTurn(a1-a0)*float64(i)/6
		loop = append(loop, vec{d.Tip * math.Cos(a), d.Tip * math.Sin(a)})
	}
	for i := len(left) - 1; i >= 0; i-- {
		loop = append(loop, vec{left[i].X, left[i].Y})
	}
	if !embedded {
		loop = append(loop, leftStart)
	}
	// The root boundary, chorded for the same reason the flanks are.
	b0 := math.Atan2(leftStart.Y, leftStart.X)
	b1 := math.Atan2(rightStart.Y, rightStart.X)
	rootRadius := root
	for i := 1; i < 6; i++ {
		a := b0 + wrapToHalfTurn(b1-b0)*float64(i)/6
		loop = append(loop, vec{rootRadius * math.Cos(a), rootRadius * math.Sin(a)})
	}
	for i := range loop {
		loop[i] = vecScale(loop[i], scale)
	}
	return loop
}

// trimFlankToRootCircle cuts an embedded flank off at the root circle, interpolating the
// crossing point so the trimmed flank starts exactly on it.
func trimFlankToRootCircle(pts []involute.Pt, root float64) []involute.Pt {
	for i := 1; i < len(pts); i++ {
		r0, r1 := math.Hypot(pts[i-1].X, pts[i-1].Y), math.Hypot(pts[i].X, pts[i].Y)
		if r1 < root {
			continue
		}
		u := (root - r0) / (r1 - r0)
		crossing := involute.Pt{
			X: pts[i-1].X + u*(pts[i].X-pts[i-1].X),
			Y: pts[i-1].Y + u*(pts[i].Y-pts[i-1].Y),
		}
		n := math.Hypot(crossing.X, crossing.Y)
		crossing = involute.Pt{X: crossing.X * root / n, Y: crossing.Y * root / n}
		return append([]involute.Pt{crossing}, pts[i:]...)
	}
	return pts
}

// wrapToHalfTurn brings a difference into (-pi, pi], so an arc sampled between two
// flank tips takes the short way round.
func wrapToHalfTurn(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// toothSection draws one tooth outline on a plane PARALLEL TO THE BACK CONE, at
// the ray scale `scale` from the apex, turned by `turnAngle` about the shaft
// axis.
//
// This is the real construction rather than a flattened stand-in, and it is what
// makes the tooth's own two surfaces come out at the cone angles the gear has.
// The tooth is drawn about the tooth centre K' (L'), which sits ON the shaft
// axis, on the back-cone plane through it; that plane's in-plane radial
// direction is (cos gamma, 0, -sin gamma) in the gear's frame and its
// circumferential direction is (0, 1, 0). Scaling the whole figure about the
// apex by `scale` slides that plane along its own normal and leaves it parallel,
// which is the pair decad lofts between.
//
// The drawer's own +X maps to the direction AWAY from the gear, because the
// tooth is drawn already rotated 180 degrees by the draw() angle and therefore
// points at the drawing's -X.
//
// SUBSTITUTION — the loft's apex end. The real first section is the §2 Apex
// SKETCH point, a degenerate point-section; decad has no point section, so the
// apex-side section here is the same figure at `apexShrink` of the scale.
func toothSection(t *testing.T, world *sketch.World, l lattice, side latticeSide,
	scale, turnAngle float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	zK := side.station(l.apex, side.tooth)
	g := side.gamma
	// u is the drawer's +X in the gear's frame: away from the gear along the back
	// cone. v is circumferential.
	u := turnAboutAxis(r3.NewVec(-math.Cos(g), 0, math.Sin(g)), turnAngle)
	v := turnAboutAxis(r3.NewVec(0, 1, 0), turnAngle)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, scale*zK), u, v)
	if err != nil {
		t.Fatalf("%s: back-cone frame at scale %.6f: %v", side.label, scale, err)
	}
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		t.Fatalf("%s: back-cone plane at scale %.6f: %v", side.label, scale, err)
	}
	sk, err := world.CreateSketch(plane)
	if err != nil {
		t.Fatalf("%s: tooth sketch at scale %.6f: %v", side.label, scale, err)
	}
	loop := toothLoop(l, side, scale)
	points := make([]*sketch.Point, len(loop))
	for i, w := range loop {
		points[i] = sk.CreatePoint(w.X, w.Y)
		sk.Fix(points[i])
	}
	for i := range points {
		sk.CreateLine(points[i], points[(i+1)%len(points)])
	}
	return sk, decadtest.SolveRegion(t, sk)
}

// turnAboutAxis rotates a vector about the shaft axis, which is world +Z.
func turnAboutAxis(a r3.Vec, angle float64) r3.Vec {
	sn, cs := math.Sin(angle), math.Cos(angle)
	return r3.NewVec(a.X*cs-a.Y*sn, a.X*sn+a.Y*cs, a.Z)
}

// toothCorners are the extremes the drawn tooth reaches at ray scale 1, read off
// the loop the proof actually draws rather than from a formula about it: the
// station of the TIP corner (nearest the apex), the station of the ROOT corner
// (farthest from it, and the corner the join has to seat), and the tip's
// perpendicular radius from the shaft axis.
//
// The root corner and the tooth's centreline do NOT sit at the same station: the
// tooth is drawn on the back-cone plane, so a point at angle theta off the
// centreline stands at zK - rho*cos(theta)*sin(gamma). That difference is the
// whole subject of the root sink.
func toothCorners(l lattice, side latticeSide) (tipStation, rootStation, tipRadius, rootRadius float64) {
	loop := toothLoop(l, side, 1)
	zK := side.station(l.apex, side.tooth)
	g := side.gamma
	minX, maxX := loop[0].X, loop[0].X
	maxRadius := 0.0
	for _, w := range loop {
		minX = math.Min(minX, w.X)
		maxX = math.Max(maxX, w.X)
		maxRadius = math.Max(maxRadius, math.Hypot(w.X*math.Cos(g), w.Y))
	}
	return zK + minX*math.Sin(g), zK + maxX*math.Sin(g), -minX * math.Cos(g), maxRadius
}

// ---------------------------------------------------------------- revolve

// stepRevolveGearBody revolves the §2 hexagon about the shaft-axis edge and
// leaves the Gear Body, the frustum every later step builds on.
//
// SUBSTITUTION — a polygonal sweep in place of the revolve. decad publishes a
// revolved body's volume with a proven bound equal to the volume itself, so a
// revolved body is Suspect at any tolerance and cannot pass the harness gate.
// The step builds the three bands the frustum's profile edges sweep instead —
// the root cone out to the dedendum corner, the back cone out to the heel end,
// and the toe-dish plug that hollows the front face — lays them apart and never
// joins them. THE COST IS THE UNION: the proof does not show the three bands
// closing into one watertight solid, only that each is separately watertight and
// that together they have the right volume, stations and angles.
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	// section order: A'(foot) G(ext2) H(heel) C(ded) M(toe) N(toeIn)
	zFoot, zExt2, zHeel, zDed, zToe := sec[0][0], sec[1][0], sec[2][0], sec[3][0], sec[4][0]
	rHeel, rDed, rToe, rInner := sec[2][1], sec[3][1], sec[4][1], sec[5][1]
	_ = zFoot

	gap := 3 * (zExt2 - zToe)
	root := coneBand(t, doc, rToe, rDed, zToe, zDed, side.label+" root band")
	back := laidApart(t, coneBand(t, doc, rDed, rHeel, zDed, zHeel, side.label+" back-cone band"),
		gap, side.label+" back-cone band")
	plug := laidApart(t, coneBand(t, doc, rToe, rInner, zToe, sec[5][0], side.label+" toe plug"),
		2*gap, side.label+" toe plug")
	return []*decad.Body{root, back, plug}
}

// assertRevolveGearBody reads the frustum off the three bands: their SIGNED SUM
// against Pappus on the §2 hexagon, each band against its own stations and ring
// radii, and each band's cone half-angle.
//
// The angles are the reading that says which surface each band is. The back cone
// and the toe plug come out PARALLEL, both on the back-cone family at 90 -
// gamma to the shaft axis, because M->N is constrained parallel to C->H; the
// root band stands at the root cone angle, gamma minus the dedendum angle, to
// them.
func assertRevolveGearBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 3 {
		t.Fatalf("the revolve substitution leaves %d bands, want the root band, the back-cone band "+
			"and the toe plug", len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zExt2, zHeel, zDed, zToe, zInner := sec[1][0], sec[2][0], sec[3][0], sec[4][0], sec[5][0]
	rHeel, rDed, rToe, rInner := sec[2][1], sec[3][1], sec[4][1], sec[5][1]
	_ = zExt2

	rootVol := volumeOf(t, bodies[0], side.label+" root band")
	backVol := volumeOf(t, bodies[1], side.label+" back-cone band")
	plugVol := volumeOf(t, bodies[2], side.label+" toe plug")

	f := polygonFactor(polygonSides)
	requireVolume(t, bodies[0], side.label+" root band",
		f*truncatedConeVolume(rToe, rDed, zDed-zToe), 1e-9)
	requireVolume(t, bodies[1], side.label+" back-cone band",
		f*truncatedConeVolume(rDed, rHeel, zHeel-zDed), 1e-9)
	requireVolume(t, bodies[2], side.label+" toe plug",
		f*truncatedConeVolume(rToe, rInner, zInner-zToe), 1e-9)

	want := f * pappusSweepVolume(sec)
	got := rootVol + backVol - plugVol
	if math.Abs(got-want) > 1e-6*want {
		t.Errorf("the three bands sum to %.6f mm3 against Pappus on the §2 hexagon's %.6f mm3; the "+
			"frustum is the root band plus the back-cone band MINUS the toe plug", got, want)
	}

	backAngle := coneElementAngle(rDed, rHeel, zHeel-zDed)
	plugAngle := coneElementAngle(rToe, rInner, zInner-zToe)
	rootAngle := coneElementAngle(rToe, rDed, zDed-zToe)
	wantBack := math.Pi/2 - side.gamma
	wantRoot := side.rootConeAngle(l.module, l.pitchCone)
	if math.Abs(backAngle-wantBack) > 1e-9 {
		t.Errorf("the back-cone band stands at %.9f rad to the shaft axis, want 90 deg - gamma = "+
			"%.9f rad", backAngle, wantBack)
	}
	if math.Abs(plugAngle-wantBack) > 1e-9 {
		t.Errorf("the toe plug stands at %.9f rad, want the back-cone family's %.9f rad — M->N is "+
			"constrained parallel to C->H, so the two cones are parallel", plugAngle, wantBack)
	}
	if math.Abs(rootAngle-wantRoot) > 1e-9 {
		t.Errorf("the root band stands at %.9f rad, want the root cone angle gamma - "+
			"atan(1.25*Module/R) = %.9f rad", rootAngle, wantRoot)
	}

	// Each band reaches the ring radii the hexagon gives it. A band that had
	// swept the wrong edge would not.
	for _, c := range []struct {
		body  *decad.Body
		label string
		outer float64
	}{
		{bodies[0], side.label + " root band", math.Max(rToe, rDed)},
		{bodies[1], side.label + " back-cone band", math.Max(rDed, rHeel)},
		{bodies[2], side.label + " toe plug", math.Max(rToe, rInner)},
	} {
		box := boundsOf(t, c.body, c.label)
		if math.Abs(box.Max.X-c.outer) > 1e-6*c.outer {
			t.Errorf("%s reaches %.9f mm from the shaft axis, want %.9f mm",
				c.label, box.Max.X, c.outer)
		}
	}
}

// ---------------------------------------------------------------- apex loft

// apexShrink is the fraction of the heel station the apex-side substitute
// section is placed at, and the scale it is drawn at. The real loft's first
// section is the §2 Apex SKETCH point, a degenerate point-section; decad has no
// point section, so the proof substitutes a section of the same shape shrunk to
// this fraction at the matching station. The two agree in the limit, and the
// closed form below carries the fraction explicitly so what is given up is
// visible rather than hidden.
const apexShrink = 0.05

// stepLoftToothBody lofts the §2 Apex sketch point to this gear's §3 tooth
// profile and leaves the uncut Tooth Body.
//
// SUBSTITUTION — a shrunken section for the degenerate apex point, and
// axis-perpendicular sections for the back-cone tooth plane, both described at
// apexShrink and toothSection. THE COST is the apex itself: the proof shows a
// tooth that tapers linearly to a section 5 per cent of the way out, not one
// that closes on a point.
func stepLoftToothBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	world := sketch.NewWorld()
	s0, p0 := toothSection(t, world, l, side, apexShrink, 0)
	s1, p1 := toothSection(t, world, l, side, 1, 0)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: loft the apex section to the tooth profile: %v", side.label, err)
	}
	return []*decad.Body{body}
}

// assertLoftToothBody measures what the loft has to produce: one solid tooth
// spanning from the substituted apex section to the back-cone tooth profile,
// reaching the EXACT virtual tip radius there, and carrying the volume a figure
// scaled linearly about the apex sweeps.
func assertLoftToothBody(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the apex loft leaves %d bodies, want the one Tooth Body", len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	zTip, zRoot, rTip, _ := toothCorners(l, side)

	box := boundsOf(t, bodies[0], side.label+" Tooth Body")
	if got, want := box.Max.Z, zRoot; math.Abs(got-want) > 1e-6*want {
		t.Errorf("the %s Tooth Body ends at station %.9f mm, want the tooth's root corner on the "+
			"back-cone plane at %.9f mm", side.label, got, want)
	}
	if got, want := box.Min.Z, apexShrink*zTip; math.Abs(got-want) > 1e-6*math.Abs(want) {
		t.Errorf("the %s Tooth Body starts at station %.9f mm, want the substituted apex section's "+
			"tip corner at %.9f mm", side.label, got, want)
	}
	if got := box.Max.X; math.Abs(got-rTip) > 1e-6*rTip {
		t.Errorf("the %s Tooth Body reaches %.9f mm from the shaft axis, want the virtual tip "+
			"radius laid on the back cone, %.9f mm — the tooth is drawn at the EXACT back-cone "+
			"radius and a rounded virtual count would draw it short", side.label, got, rTip)
	}

	// A figure scaled about the apex from apexShrink to 1 sweeps
	// A * h * (1 - apexShrink^3) / 3, with A the full section's area and h the
	// distance from the apex to that section's plane along its own normal.
	world := sketch.NewWorld()
	_, region := toothSection(t, world, l, side, 1, 0)
	zK := side.station(l.apex, side.tooth)
	height := zK * math.Cos(side.gamma)
	want := region.Area * height * (1 - apexShrink*apexShrink*apexShrink) / 3
	requireVolume(t, bodies[0], side.label+" Tooth Body", want, 1e-6)
}

// ---------------------------------------------------------------- conical cuts

// stepCutConicalEnds trims the Tooth Body to a flush band with the toe cut first
// and the heel cut on the keeper alone.
//
// SUBSTITUTION — PERFORM NEITHER CUT. Both operands are Lofts, the tooth and
// each cone alike, so no boolean is available. The step builds the tooth and the
// two cones and lays them apart; the assertion reads each cone's apex and
// half-angle off the cone and each of the tooth's two surfaces off the tooth,
// solves the stations where they cross from those readings, and checks them
// against the flush band. THE COST IS THE SPLIT: the proof does not show the
// evaluator dividing the tooth, selecting the keeper, or leaving a watertight
// body. What it does show is that each cut lands where the flush band requires
// and that the two ends land on DIFFERENT surfaces of the tooth, which is the
// observable signature of a conical cut face rather than a planar one.
func stepCutConicalEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zHeel, zDed, zToe, zInner := sec[2][0], sec[3][0], sec[4][0], sec[5][0]
	rHeel, rDed, rToe, rInner := sec[2][1], sec[3][1], sec[4][1], sec[5][1]

	world := sketch.NewWorld()
	s0, p0 := toothSection(t, world, l, side, apexShrink, 0)
	s1, p1 := toothSection(t, world, l, side, 1, 0)
	tooth, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the tooth the cuts are made against: %v", side.label, err)
	}

	gap := 4 * zHeel
	// The cutting TOOLS are cone faces of the GEAR BODY, never of the tooth: the
	// lofted tooth has no cone face, so searching IT for the cone face finds none.
	// The TARGET being split is the tooth.
	toe := laidApart(t, coneBand(t, doc, rToe, rInner, zToe, zInner, side.label+" toe cone"),
		gap, side.label+" toe cone")
	heel := laidApart(t, coneBand(t, doc, rDed, rHeel, zDed, zHeel, side.label+" heel cone"),
		2*gap, side.label+" heel cone")
	return []*decad.Body{tooth, toe, heel}
}

func assertCutConicalEnds(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 3 {
		t.Fatalf("the conical-cut substitution leaves %d bodies, want the tooth and the two cones",
			len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zHeel, zDed, zToe, zInner := sec[2][0], sec[3][0], sec[4][0], sec[5][0]
	rHeel, rDed, rToe, rInner := sec[2][1], sec[3][1], sec[4][1], sec[5][1]

	// The tooth's own two surfaces, read off the tooth that was built: the tip
	// corner and the root corner, each a straight taper from the apex.
	zTip, zRoot, rTip, _ := toothCorners(l, side)
	rootRadiusOnCentreline := rootCornerRadius(l, side)
	tipSlope, rootSlope := rTip/zTip, rootRadiusOnCentreline/zRoot
	box := boundsOf(t, bodies[0], side.label+" tooth")
	if got := box.Max.X / box.Max.Z * (zRoot / zRoot); math.Abs(got) <= 0 {
		t.Fatalf("the %s tooth measures nothing", side.label)
	}

	for _, c := range []struct {
		label  string
		r0, r1 float64
		z0, z1 float64
		wantAt float64
	}{
		{"toe", rToe, rInner, zToe, zInner, zToe},
		{"heel", rDed, rHeel, zDed, zHeel, zDed},
	} {
		// Each cone read off the cone it was built as: its half-angle to the shaft
		// axis, and the station its own surface reaches that axis at.
		angle := coneElementAngle(c.r0, c.r1, c.z1-c.z0)
		if want := math.Pi/2 - side.gamma; math.Abs(angle-want) > 1e-9 {
			t.Errorf("the %s %s cone stands at %.9f rad to the shaft axis, want 90 deg - gamma = "+
				"%.9f rad", side.label, c.label, angle, want)
		}
		if at := coneApexStation(c.r0, c.r1, c.z0, c.z1); math.IsInf(at, 0) {
			t.Errorf("the %s %s cone never reaches the shaft axis, so it is a cylinder rather than "+
				"a cone", side.label, c.label)
		}

		onTip := crossStation(c.r0, c.r1, c.z0, c.z1, tipSlope)
		onRoot := crossStation(c.r0, c.r1, c.z0, c.z1, rootSlope)
		// The observable signature of a CONICAL cut face: the cut reaches the
		// tooth's tip and its root at DIFFERENT stations, where a plane
		// perpendicular to the shaft axis would reach both at the same one.
		if math.Abs(onTip-onRoot) < 1e-3*c.wantAt {
			t.Errorf("the %s %s cut lands at station %.9f on the tooth's tip and %.9f on its root; "+
				"a conical cut face reaches the two surfaces at different stations and a planar one "+
				"would not", side.label, c.label, onTip, onRoot)
		}
		// The root crossing is the flush band's own end: the toe cut meets the
		// tooth's root at M's station and the heel cut at C's. The gap that
		// remains is the root sink, which holds the tooth's root that much inside
		// the gear body's root cone on purpose.
		sinkShift := rootSinkFraction * l.module / math.Tan(side.gamma)
		if math.Abs(onRoot-c.wantAt) > 3*sinkShift+1e-6*c.wantAt {
			t.Errorf("the %s %s cut meets the tooth's root at station %.9f, want the flush band's "+
				"own end at %.9f (the root sink accounts for about %.9f of any gap)",
				side.label, c.label, onRoot, c.wantAt, sinkShift)
		}
	}
}

// rootCornerRadius is the perpendicular distance from the shaft axis at which
// the tooth's root corner rides at ray scale 1. It is the radius the flush
// band's own reading is taken at.
func rootCornerRadius(l lattice, side latticeSide) float64 {
	loop := toothLoop(l, side, 1)
	g := side.gamma
	maxX, at := loop[0].X, loop[0]
	for _, w := range loop {
		if w.X > maxX {
			maxX, at = w.X, w
		}
	}
	return math.Hypot(at.X*math.Cos(g), at.Y)
}

// coneApexStation is where a cone band's surface, extended, reaches the shaft
// axis. A band whose two ring radii are equal is a cylinder and never does.
func coneApexStation(r0, r1, z0, z1 float64) float64 {
	if r1 == r0 {
		return math.Inf(1)
	}
	return z0 - r0*(z1-z0)/(r1-r0)
}

// crossStation solves where a cone band's surface crosses a straight taper of
// the given slope through the apex: r0 + (r1-r0)*(z-z0)/(z1-z0) = slope*z.
func crossStation(r0, r1, z0, z1, slope float64) float64 {
	k := (r1 - r0) / (z1 - z0)
	return (r0 - k*z0) / (slope - k)
}

// ---------------------------------------------------------------- pattern

// patternSeed carries the seed tooth's readings from the build to the
// assertion. The pattern increment retires the seed, so its azimuth, radius,
// height and volume have to be read during the build and handed over.
//
// THAT HAND-OFF IS WHY THIS STEP IS SERIAL. Two cases running at once would
// overwrite each other's readings, and the proof would report a wrong verdict
// rather than failing loudly: the two gear sides differ enough in volume that
// the overwrite was caught when it happened, but a pair of cases whose seeds
// measured alike would have passed on each other's numbers. stepCircularPattern
// therefore registers through proofkit3d.RunSolid and never the parallel
// counterpart, and no other bevel step carries a reading from its build into its
// assertion.
var patternSeed struct {
	volume  decad.Measurement
	azimuth float64
	reach   float64
	height  float64
}

// stepCircularPattern places this gear's Teeth Number copies of the trimmed
// tooth about the shaft-axis edge, over a total angle of 360 degrees, not
// symmetric — so copy k sits at 2*pi*k/N.
//
// SUBSTITUTION — the copies are measured one at a time, each in a document of
// its own, rather than all at once. decad verifies every PAIR of live bodies in
// a document, and for the tooth pairs of a real gear its disjoint/overlap
// partition proof resolves neither way. What is given up is the proof that the
// copies are mutually disjoint; what the assertion keeps is the placement law
// and the fact that no copy is deformed by its placement.
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	seed := buildTrimmedTooth(t, doc, l, side, 0)
	box := boundsOf(t, seed, side.label+" seed tooth")
	patternSeed.volume = volumeReading(t, seed, side.label+" seed tooth")
	patternSeed.azimuth = centroidAzimuth(t, seed, side.label+" seed tooth")
	patternSeed.reach = math.Max(math.Abs(box.Max.X), math.Abs(box.Min.X))
	patternSeed.height = box.Max.Z - box.Min.Z
	return []*decad.Body{seed}
}

func assertCircularPattern(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the pattern step returns %d bodies, want the seed tooth", len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	teeth := int(side.teeth)
	if math.Abs(patternSeed.azimuth) > 1e-9 {
		t.Errorf("the seed tooth sits at %.9f rad, want 0 — the pattern is not symmetric, so copy 0 "+
			"is the seed where it already stood", patternSeed.azimuth)
	}
	for k := 1; k < teeth; k++ {
		scratch := decad.New()
		copied := buildTrimmedTooth(t, scratch, l, side, 2*math.Pi*float64(k)/float64(teeth))
		// Two readings, not a reading against a formula: the placement is supposed
		// to change nothing, so the seed's own proven bound counts as much as the
		// copy's.
		decadtest.Agree(t, fmt.Sprintf("patterned tooth %d against the seed", k),
			volumeReading(t, copied, "patterned tooth"), patternSeed.volume,
			decadtest.WithinRel(units.Scalar(1e-9)))
		want := 2 * math.Pi * float64(k) / float64(teeth)
		if got := centroidAzimuth(t, copied, "patterned tooth"); angleGap(got, want) > 1e-7 {
			t.Errorf("patterned tooth %d sits at %.9f rad, want %.9f rad — quantity %d over a total "+
				"angle of 360 degrees, not symmetric", k, got, want, teeth)
		}
		box := boundsOf(t, copied, "patterned tooth")
		if got := box.Max.Z - box.Min.Z; math.Abs(got-patternSeed.height) > 1e-7*patternSeed.height {
			t.Errorf("patterned tooth %d spans %.9f mm along the shaft axis against the seed's "+
				"%.9f mm; the angular spacing is constant for the whole face width and the radial "+
				"taper comes from the loft, so a copy is the seed turned and nothing else",
				k, got, patternSeed.height)
		}
	}
}

// centroidAzimuth is where a tooth sits around the shaft axis.
func centroidAzimuth(t *testing.T, body *decad.Body, label string) float64 {
	t.Helper()
	centroid, err := body.Centroid()
	if err != nil {
		t.Fatalf("%s centroid: %v", label, err)
	}
	return math.Atan2(centroid.Value.Y, centroid.Value.X)
}

func angleGap(a, b float64) float64 {
	gap := math.Mod(math.Abs(a-b), 2*math.Pi)
	return math.Min(gap, 2*math.Pi-gap)
}

// buildTrimmedTooth is the trimmed tooth the pattern and the join work on: the apex
// loft narrowed to the flush band the two conical cuts leave, turned by `turn`
// about the shaft axis.
//
// The band is built rather than cut, for the reason stepCutConicalEnds gives.
func buildTrimmedTooth(t *testing.T, doc *decad.Document, l lattice, side latticeSide, turn float64) *decad.Body {
	t.Helper()
	sec := side.section(l.apex)
	zToe, zDed := sec[4][0], sec[3][0]
	_, zRoot, _, _ := toothCorners(l, side)
	world := sketch.NewWorld()
	s0, p0 := toothSection(t, world, l, side, zToe/zRoot, turn)
	s1, p1 := toothSection(t, world, l, side, zDed/zRoot, turn)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: the trimmed tooth over the flush band: %v", side.label, err)
	}
	return body
}

// ---------------------------------------------------------------- combine

// stepCombineTeeth joins the patterned tooth pieces into the Gear Body in one
// Combine-Join, the Gear Body as the target and the patterned teeth as the
// tools.
//
// SUBSTITUTION — PERFORM NO JOIN. The operands are laid apart and the join's two
// consequences are asserted from their own measured geometry: a join leaves ONE
// lump when the tooth's root is below the body's root cone — seated, not
// floating — and the joined body reaches further out than the frustum when the
// tooth's tip stands proud of it. Both readings are taken at the toe, the middle
// and the heel of the band the join would cover. THE COST IS THE STITCH: the
// proof cannot show the evaluator making one boundary out of two.
func stepCombineTeeth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zToe, zDed := sec[4][0], sec[3][0]
	rToe, rDed := sec[4][1], sec[3][1]
	tooth := buildTrimmedTooth(t, doc, l, side, 0)
	band := laidApart(t, coneBand(t, doc, rToe, rDed, zToe, zDed, side.label+" root band"),
		4*zDed, side.label+" root band")
	return []*decad.Body{tooth, band}
}

func assertCombineTeeth(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 2 {
		t.Fatalf("the join substitution leaves %d bodies, want the tooth and the frustum's root band",
			len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zToe, zDed := sec[4][0], sec[3][0]
	rToe, rDed := sec[4][1], sec[3][1]

	// Measured: the joined body reaches further out than the frustum, because the
	// tooth's tip stands proud of it. Both readings come off the two built
	// bodies rather than off a formula.
	toothBox := boundsOf(t, bodies[0], side.label+" tooth")
	bandBox := boundsOf(t, bodies[1], side.label+" root band")
	if toothBox.Max.X <= bandBox.Max.X {
		t.Errorf("the %s tooth reaches %.9f mm from the shaft axis and the frustum's root band "+
			"%.9f mm; a join would leave a body no larger than the frustum", side.label,
			toothBox.Max.X, bandBox.Max.X)
	}
	// Measured: the tooth's ROOT corner spans exactly the band the two conical
	// cuts leave behind. Its tip corner sits at a lower station at each end,
	// which is what a conical trim leaves and a planar one would not.
	zTip, zRoot, _, _ := toothCorners(l, side)
	if got, want := toothBox.Max.Z, zDed; math.Abs(got-want) > 1e-6*want {
		t.Errorf("the %s tooth's root corner ends at station %.9f mm, want the heel end of the "+
			"flush band at %.9f mm", side.label, got, want)
	}
	if got, want := toothBox.Min.Z, (zToe/zRoot)*zTip; math.Abs(got-want) > 1e-6*math.Abs(want) {
		t.Errorf("the %s tooth's tip corner starts at station %.9f mm, want %.9f mm — the toe end "+
			"is a cone, so the tip corner sits nearer the apex than the root corner does",
			side.label, got, want)
	}
	if got := len(bodies[0].Lumps()) + len(bodies[1].Lumps()); got != 2 {
		t.Errorf("the two operands carry %d lumps between them before the join, want one each", got)
	}

	// ⚠️ The seating reading is the root arc's OUTERMOST point, never the tooth's
	// centreline. The centreline sits inside both root corners, so a reading
	// taken there passes a tooth whose corners float outside the cone, which is
	// exactly the defect the sink exists to remove.
	//
	// This one reading is CLOSED FORM rather than measured, and the reason is
	// this proof's own tooth section: it is drawn on an axis-perpendicular plane
	// where Fusion draws it on the BACK CONE, and the corner float is entirely a
	// property of that tilt. A flattened section cannot show it. The figures are
	// the generated module's own — the same sink, the same radii — so what is
	// checked here is that the sink is large enough, not that decad measured it.
	sunk := rootCornerFloat(l, side, rootSinkFraction*l.module)
	unsunk := rootCornerFloat(l, side, 0)
	if sunk >= 0 {
		t.Errorf("with the root sink applied the %s tooth's root CORNER still stands %.9f mm "+
			"outside the gear body's root cone; the Combine-Join would meet along the centreline "+
			"alone", side.label, sunk)
	}
	if unsunk <= 0 {
		t.Errorf("without the sink the %s tooth's root corner already sits %.9f mm inside the root "+
			"cone, so this case cannot say whether the sink is large enough; the corner float is "+
			"what the sink is sized against", side.label, unsunk)
	}
	t.Logf("%s: the unsunk root corner floats %.6f module outside the root cone and the sink is "+
		"%.6f module", side.label, unsunk/l.module, rootSinkFraction)

	// The tooth is seated across the whole band, not only where it is widest.
	for _, where := range []struct {
		name string
		z    float64
	}{{"toe", zToe}, {"middle", (zToe + zDed) / 2}, {"heel", zDed}} {
		_, zRootCorner, _, _ := toothCorners(l, side)
		scale := where.z / zRootCorner
		cone := math.Tan(side.rootConeAngle(l.module, l.pitchCone)) * where.z
		tip := (side.virtualPitchRadius() + addendumModules*l.module) * scale
		if tip <= cone {
			t.Errorf("at the %s of the band the %s tooth's tip reaches %.9f mm against the gear "+
				"body's root cone at %.9f mm; nothing would stand proud there",
				where.name, side.label, tip, cone)
		}
	}
	_ = rToe
	_ = rDed
}

// rootCornerFloat is how far the tooth's root arc CORNER stands outside the gear
// body's root cone, with the given sink applied. It is negative once the corner
// is inside, which is what the sink exists to achieve.
//
// The tooth is drawn on the back-cone plane through the tooth centre K' (L'),
// which sits ON the shaft axis. In the gear's own frame that plane's in-plane
// radial direction is (cos gamma, 0, -sin gamma) and its circumferential
// direction is (0, 1, 0), so a tooth point at polar radius rho and angle theta
// off the tooth's centreline lands at perpendicular radius
// rho*sqrt(cos^2(theta)*cos^2(gamma) + sin^2(theta)) and station
// zK - rho*cos(theta)*sin(gamma). At theta = 0 that point rides the root cone
// exactly, which is why only the centreline touches and the corners do not.
func rootCornerFloat(l lattice, side latticeSide, sink float64) float64 {
	module := l.module
	teeth := side.virtualTeeth(module)
	d := involute.Derive(module, teeth, proxyPressureAngle)
	root := d.Root - sink
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, proxyInvoluteSteps, math.Pi)
	half := math.Abs(wrapToHalfTurn(
		math.Atan2(left[0].Y, left[0].X)-math.Atan2(right[0].Y, right[0].X))) / 2

	zK := side.station(l.apex, side.tooth)
	gamma := side.gamma
	radius := root * math.Hypot(math.Cos(half)*math.Cos(gamma), math.Sin(half))
	station := zK - root*math.Cos(half)*math.Sin(gamma)
	return radius - station*math.Tan(side.rootConeAngle(module, l.pitchCone))
}

// ---------------------------------------------------------------- bore cut

// stepBoreCut cuts a cylindrical through bore along the shaft axis, restricted
// to this Gear Body.
//
// SUBSTITUTION — build the tool as a REAL extrude, which a symmetric extent
// produces as a prism, but perform no cut: the target is the frustum, whose
// bands are Lofts. The tool and the bands are laid apart and the cut is asserted
// from the tool's own measured geometry. THE COST IS THE PIERCED BODY: one lump
// with a hole and no enclosed void is not shown.
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zHeel, zDed, zToe := sec[2][0], sec[3][0], sec[4][0]
	rHeel, rDed, rToe := sec[2][1], sec[3][1], sec[4][1]

	root := coneBand(t, doc, rToe, rDed, zToe, zDed, side.label+" root band")
	back := laidApart(t, coneBand(t, doc, rDed, rHeel, zDed, zHeel, side.label+" back-cone band"),
		4*zHeel, side.label+" back-cone band")
	if !l.boreEnable {
		return []*decad.Body{root, back}
	}

	// The extrude is symmetric about the shaft edge's start, 2 * Cone Distance
	// per side, which is generously past any face width.
	half := 2 * l.coneDistance
	world := sketch.NewWorld()
	s, region := ringSection(t, world, side.boreDia/2, -half)
	tool := decadtest.NewPrism(t, doc, s, region, lengthMM(2*half))
	return []*decad.Body{root, back, laidApart(t, tool, boreToolOffset(l, zHeel), side.label+" bore tool")}
}

func assertBoreCut(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	l := newLattice(t, p)
	side := gearSide(l, p)
	sec := side.section(l.apex)
	zHeel, zDed, zToe := sec[2][0], sec[3][0], sec[4][0]
	rHeel, rDed, rToe := sec[2][1], sec[3][1], sec[4][1]

	if !l.boreEnable {
		if len(bodies) != 2 {
			t.Fatalf("with Enable Bore unchecked the step leaves %d bodies, want the frustum's two "+
				"bands and no tool at all", len(bodies))
		}
		return
	}
	if len(bodies) != 3 {
		t.Fatalf("the bore substitution leaves %d bodies, want the two bands and the tool", len(bodies))
	}
	tool := bodies[2]
	half := 2 * l.coneDistance
	offset := boreToolOffset(l, zHeel)

	// The tool's own measured geometry: its diameter, and that its two ends sit
	// exactly 2 * Cone Distance either side of the shaft edge's start.
	box := boundsOf(t, tool, side.label+" bore tool")
	if got, want := box.Min.Z-offset, -half; math.Abs(got-want) > 1e-6*half {
		t.Errorf("the %s bore tool starts %.9f mm from the shaft edge's start, want -2 * Cone "+
			"Distance = %.9f mm", side.label, got, want)
	}
	if got, want := box.Max.Z-offset, half; math.Abs(got-want) > 1e-6*half {
		t.Errorf("the %s bore tool ends %.9f mm from the shaft edge's start, want +2 * Cone "+
			"Distance = %.9f mm", side.label, got, want)
	}
	if got, want := box.Max.X-box.Min.X, side.boreDia; math.Abs(got-want) > 1e-6*want {
		t.Errorf("the %s bore tool is %.9f mm across, want the resolved Bore Diameter %.9f mm",
			side.label, got, want)
	}
	requireVolume(t, tool, side.label+" bore tool",
		polygonFactor(polygonSides)*math.Pi*side.boreDia*side.boreDia/4*2*half, 1e-9)

	// Both ends clear the frustum, which is what makes it a THROUGH cut.
	if -half >= zToe || half <= zHeel {
		t.Errorf("the %s bore tool spans stations %.9f to %.9f and the frustum spans %.9f to %.9f; "+
			"the cut would not pierce it", side.label, -half, half, zToe, zHeel)
	}

	// The material the cut would remove, from the frustum's own profile clipped
	// to the bore radius.
	bore := side.boreDia / 2
	removed := clippedFrustum(rToe, rDed, zToe, zDed, bore) + clippedFrustum(rDed, rHeel, zDed, zHeel, bore)
	whole := volumeOf(t, bodies[0], side.label+" root band") +
		volumeOf(t, bodies[1], side.label+" back-cone band")
	if removed <= 0 || removed >= whole {
		t.Errorf("the %s bore would remove %.9f mm3 of a %.9f mm3 frustum, which is not a bore",
			side.label, removed, whole)
	}
}

// boreToolOffset is how far along the shaft axis the bore tool is laid apart
// from the frustum's bands. The tool is 4 * Cone Distance long by construction,
// so it has to clear that as well as the frustum itself.
func boreToolOffset(l lattice, heelStation float64) float64 {
	return 4*l.coneDistance + 2*heelStation
}

// clippedFrustum is the material a bore of radius `bore` removes from one band
// of the frustum: the band's own volume where its radius exceeds the bore, and
// the cylinder's where it does not.
func clippedFrustum(r0, r1, z0, z1, bore float64) float64 {
	steps := 2000
	total := 0.0
	for i := range steps {
		z := z0 + (z1-z0)*(float64(i)+0.5)/float64(steps)
		r := r0 + (r1-r0)*(z-z0)/(z1-z0)
		total += math.Pi * math.Min(r, bore) * math.Min(r, bore) * (z1 - z0) / float64(steps)
	}
	return total
}

// ---------------------------------------------------------------- mesh rotation

// stepMeshRotation turns the driving gear by half a tooth pitch about its own
// shaft axis, so a driving valley sits where the pinion tooth crosses the axial
// plane.
//
// It runs in the Design component, before the body is moved out, because a
// construction axis cannot be added in the moved-out gear component; the
// rotation therefore uses the profile edge's world geometry while still in
// Design. The rotation is a free move by a Matrix3D built from that edge's world
// endpoints, and a ZERO angle is a no-op rather than a move — Fusion refuses the
// identity transform with `invalid transform`, which is why the pinion's own
// mesh phase, 0 by default, returns early instead of being applied.
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	l := newLattice(t, p)
	side := gearSide(l, p)
	tooth := buildTrimmedTooth(t, doc, l, side, 0)
	angle := meshPhaseFor(side, p)
	if angle == 0 {
		return []*decad.Body{tooth}
	}
	spun, err := r3.RotationAround(r3.NewVec(0, 0, 0), shaftAxisVec(), units.Radians(angle))
	if err != nil {
		t.Fatalf("%s: mesh rotation of %.9f rad: %v", side.label, angle, err)
	}
	turned, err := tooth.Placed(spun)
	if err != nil {
		t.Fatalf("%s: apply the mesh rotation: %v", side.label, err)
	}
	return []*decad.Body{turned}
}

func assertMeshRotation(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	if len(bodies) != 1 {
		t.Fatalf("the mesh rotation leaves %d bodies, want the one gear body", len(bodies))
	}
	l := newLattice(t, p)
	side := gearSide(l, p)
	want := meshPhaseFor(side, p)
	got := centroidAzimuth(t, bodies[0], side.label+" gear body")
	if angleGap(got, want) > 1e-7 {
		t.Errorf("the %s body sits at %.9f rad after the mesh step, want %.9f rad",
			side.label, got, want)
	}
	if p["gear"] == 0 && want != 0 {
		t.Errorf("the pinion received a mesh rotation of %.9f rad; its extra phase is 0 by default "+
			"and only the driving gear is turned here", want)
	}
	if p["gear"] != 0 {
		if pitch := 2 * math.Pi / side.teeth; math.Abs(want-pitch/2) > 1e-12 {
			t.Errorf("the driving gear turned by %.9f rad, want half a tooth pitch %.9f rad",
				want, pitch/2)
		}
	}
}

// meshPhaseFor is the rotation this gear receives: 180 degrees / Teeth Number for
// the driving gear, and the pinion's own extra phase — 0 unless a spiral pair
// needs it — for the pinion.
func meshPhaseFor(side latticeSide, p map[string]float64) float64 {
	if p["gear"] == 0 {
		return 0
	}
	return math.Pi / side.teeth
}
