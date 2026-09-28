package screwgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The frame's joins and cuts, and why they take stand-ins.
//
// The frame is round: a torus, cylinders, spheres, and rounded collars. decad
// builds each piece on its own — the ring's revolve, the rods' extrusion, a
// bar's and a ball's revolve are all proved below at their exact volumes — but
// it cannot combine them at this size. A union of the torus and one rod was
// refused before any contact was examined ("this chord tolerance asks for more
// than 65536 facets in one revolve mesh"), and a union chain whose first
// operand had a curved face refused its second step ("requested tolerance
// 0.0013 mm is below the faceted body's minimum mesh bound 0.0029 mm"). A chain
// of all-planar stand-ins got further and then refused an operand holding a
// collapsed facet its own earlier union had produced.
//
// So every join and cut step takes planar stand-ins that lie INSIDE the real
// pieces — polygons inscribed in every round section — and is proved one
// joining pair at a time: a case per tool, the tool's stand-in united with the
// stand-in of the piece it attaches to, in a document of their own. Inside is
// what makes that sound: if the inscribed pieces overlap, the real ones do, so
// the real join takes the tool in. Pair by pair, the cases connect every tool
// to the ring — rods to the ring, loop pieces to the rods, collars to the rods
// — which is what "each join leaves one body" needs. What the pairs do not see
// is the Fusion combine of a whole group at once, which the build checks at
// run time with the combine feature's bodies.count.

// The polygon counts and phases of the stand-ins. The phases are offset from
// every other stand-in's so that no two faces are coplanar and no vertex lands
// on another piece's face: exact coincidences are what decad's booleans refuse.
const (
	csRingSides = 64
	csRodSides  = 16
	csRodPhase  = 0.13
	csBarSides  = 8
	csBarPhase  = math.Pi/8 + 0.07
	csBallPhase = 0.21
)

// ---- case tables -----------------------------------------------------------

func csSolidPerVariant(vs []csVariant) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range csPerVariant(vs) {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

func csSolidPerIndex(vs []csVariant, count int, label string) []proofkit3d.Case {
	var out []proofkit3d.Case
	for _, c := range csPerIndex(vs, count, label) {
		out = append(out, proofkit3d.Case{Name: c.Name, Params: c.Params})
	}
	return out
}

var (
	csRingRevolveCases = csSolidPerVariant([]csVariant{csVDefaults, csVEngageFull})
	csRodsExtrudeCases = csSolidPerVariant(csCageVariants)
	csRodsJoinCases    = csSolidPerIndex(csCageVariants, 4, "rod")
	csLoopPieceCases   = csSolidPerIndex(csCageVariants, 4, "foot")
	csLoopJoinCases    = csSolidPerIndex(csCageVariants, 8, "piece")
	csCollarBodyCases  = csSolidPerIndex(csCageVariants, 4, "collar")
)

// ---- plane geometry for the oracles ----------------------------------------

// csClip is the part of convex polygon a inside convex polygon b, both wound
// counter-clockwise (Sutherland–Hodgman).
func csClip(a, b [][2]float64) [][2]float64 {
	out := a
	for i := range b {
		p, q := b[i], b[(i+1)%len(b)]
		side := func(x [2]float64) float64 { return (q[0]-p[0])*(x[1]-p[1]) - (q[1]-p[1])*(x[0]-p[0]) }
		in := out
		out = nil
		for j := range in {
			c, d := in[j], in[(j+1)%len(in)]
			sc, sd := side(c), side(d)
			if sc >= 0 {
				out = append(out, c)
			}
			if (sc >= 0) != (sd >= 0) {
				t := sc / (sc - sd)
				out = append(out, [2]float64{c[0] + t*(d[0]-c[0]), c[1] + t*(d[1]-c[1])})
			}
		}
		if len(out) == 0 {
			return nil
		}
	}
	return out
}

func csArea(a [][2]float64) float64 {
	s := 0.0
	for i := range a {
		p, q := a[i], a[(i+1)%len(a)]
		s += p[0]*q[1] - q[0]*p[1]
	}
	return s / 2
}

// csFlat is a world polygon seen along n̂, in the selected plane's (ê, k̂).
func csFlat(pts []r3.Vec) [][2]float64 {
	e, _, k := csAxes()
	out := make([][2]float64, len(pts))
	for i, p := range pts {
		d := p.Sub(csCentre)
		out[i] = [2]float64{d.Dot(e), d.Dot(k)}
	}
	return out
}

// csRegular is a regular polygon of circumradius r about c in the plane of u
// and v, counter-clockwise about u × v.
func csRegular(c, u, v r3.Vec, r float64, sides int, phase float64) []r3.Vec {
	out := make([]r3.Vec, sides)
	for i := range sides {
		a := phase + 2*math.Pi*float64(i)/float64(sides)
		out[i] = c.Add(u.Scale(r * math.Cos(a))).Add(v.Scale(r * math.Sin(a)))
	}
	return out
}

// ---- the true pieces -------------------------------------------------------

// csRingSketch is the Ring sketch in decad: the construction axis An and the
// wire's circle, on the Ring Plane's (ê, n̂) coordinates.
func csRingSketch(t testing.TB, in csIn) (*sketch.Sketch, *sketch.Profile, decad.SketchLine) {
	t.Helper()
	p := in.P
	e, n, _ := csAxes()
	f := csFrame(t, csCentre, e, n)
	s := csPlaneSketch(t, f)
	ax, ay := csLocal(t, f, csCentre, "C")
	bx, by := csLocal(t, f, csCentre.Add(n.Scale(p.CageRise)), "C + cageRise*n̂")
	a, b := s.CreatePoint(ax, ay), s.CreatePoint(bx, by)
	s.CreateLine(a, b).SetConstruction(true)
	s.Fix(a)
	s.Fix(b)
	cx, cy := csLocal(t, f, csCentre.Add(n.Scale(p.CageRise)).Add(e.Scale(p.RingRadius)), "the wire's centre")
	centre := s.CreatePoint(cx, cy)
	wire := s.CreateCircle(centre, p.RingWire/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(wire, p.RingWire))
	return s, decadtest.SolveRegion(t, s), decad.SketchLine{Start: decad.Point2{U: ax, V: ay}, End: decad.Point2{U: bx, V: by}}
}

// stepRingRevolve is the ring: the wire's circle revolved a full turn about An,
// revolveFeatures.createInput(profile, axis, NewBodyFeatureOperation) with a
// 360-degree extent. The ring is the cage's first body.
//
// SUBSTITUTE: decad builds the torus, and the assertion holds that real
// revolve to its exact volume, but its verification reports the torus's area
// reading as having no tolerance reference, which the solid gate refuses. The
// gated body is the revolve of the wire's inscribed 32-gon, drawn in the same
// plane about the same axis: a solid of cones and cylinders inside the torus.
func stepRingRevolve(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	csCheckInputs(t, in)
	s, prof, axis, _ := csRingPolygonSketch(t, in)
	ring, err := doc.Revolve(s, prof, axis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("ring stand-in revolve: %v", err)
	}
	return []*decad.Body{ring}
}

// csRoundSides is how many sides the gated stand-in of a revolved circle has.
const csRoundSides = 32

// csRingPolygonSketch is the Ring sketch with the wire's circle replaced by its
// inscribed polygon.
func csRingPolygonSketch(t testing.TB, in csIn) (*sketch.Sketch, *sketch.Profile, decad.SketchLine, [][2]float64) {
	t.Helper()
	p := in.P
	e, n, _ := csAxes()
	f := csFrame(t, csCentre, e, n)
	s := csPlaneSketch(t, f)
	c := csCentre.Add(n.Scale(p.CageRise)).Add(e.Scale(p.RingRadius))
	poly := csRegular(c, e, n, p.RingWire/2, csRoundSides, 0.1)
	csPolygon(t, s, f, poly, "the wire's polygon")
	ax, ay := csLocal(t, f, csCentre, "C")
	bx, by := csLocal(t, f, csCentre.Add(n.Scale(p.CageRise)), "C + cageRise*n̂")
	local := make([][2]float64, len(poly))
	for i, w := range poly {
		local[i][0], local[i][1] = csLocal(t, f, w, "the wire's polygon")
	}
	return s, decadtest.SolveRegion(t, s), decad.SketchLine{Start: decad.Point2{U: ax, V: ay}, End: decad.Point2{U: bx, V: by}}, local
}

// csPappus is the volume a plane polygon sweeps revolving a full turn about the
// line x = 0 of its own plane: 2*pi times its centroid's distance times its area.
func csPappus(poly [][2]float64) float64 {
	a, cx := 0.0, 0.0
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		cross := p[0]*q[1] - q[0]*p[1]
		a += cross
		cx += (p[0] + q[0]) * cross
	}
	a /= 2
	cx /= 6 * a
	return 2 * math.Pi * math.Abs(cx) * math.Abs(a)
}

func checkRingRevolve(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	p := in.P
	r := p.RingWire / 2
	// The real revolve, of the circle, in a document of its own: Pappus is exact.
	s, prof, axis := csRingSketch(t, in)
	torus, err := decad.New().Revolve(s, prof, axis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("ring revolve: %v", err)
	}
	csMeasuresVolume(t, "the ring", torus, 2*math.Pi*math.Pi*p.RingRadius*r*r, 1e-9)
	// The gated stand-in, by Pappus on its polygon; the axis is the sketch's
	// x = 0 line, which the Ring Plane's frame puts through C along n̂.
	_, _, _, local := csRingPolygonSketch(t, in)
	csMeasuresVolume(t, "the ring's polygon stand-in", bodies[0], csPappus(local), 1e-9)
}

// stepRodsExtrude is the four rods: the Rods sketch's four profiles extruded
// in one feature as new bodies, symmetrically cageRise either side of the
// selected plane. decad extrudes one profile a call, so the proof makes four
// calls with the one extent; the build requires the feature's bodies.count to
// be 4, which is the four bodies returned here.
func stepRodsExtrude(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	p := in.P
	e, _, k := csAxes()
	f := csFrame(t, csCentre, e, k)
	s := csPlaneSketch(t, f)
	for i, psi := range csFootAzimuths(fr) {
		x, y := csLocal(t, f, csFoot(p, psi, 0), fmt.Sprintf("rod %d's foot", i))
		c := s.CreatePoint(x, y)
		circle := s.CreateCircle(c, p.RodDiameter/2)
		s.Fix(c)
		s.AddConstraint(sketch.NewDiameter(circle, p.RodDiameter))
	}
	sketchtestSolve(t, s)
	profiles := s.Profiles()
	if len(profiles) != 4 {
		t.Fatalf("the Rods sketch has %d profiles; the build requires 4", len(profiles))
	}
	var rods []*decad.Body
	for i, prof := range profiles {
		rod, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(p.CageRise)})
		if err != nil {
			t.Fatalf("rod %d extrude: %v", i, err)
		}
		rods = append(rods, rod)
	}
	return rods
}

func checkRodsExtrude(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	p := csRead(t, m).P
	if len(bodies) != 4 {
		t.Fatalf("%d rods; the build requires 4", len(bodies))
	}
	for i, rod := range bodies {
		// A cylinder of diameter rodDiameter from -cageRise to +cageRise.
		csMeasuresVolume(t, fmt.Sprintf("rod %d", i), rod, math.Pi*p.RodDiameter*p.RodDiameter/4*2*p.CageRise, 1e-9)
	}
}

// csBarGeometry is bar i of the loop: from foot_i to foot_(i+1), the in-plane
// outward normal m̂, and the unit axis.
func csBarGeometry(t testing.TB, in csIn, fr frame, i int) (a, b, out, axis r3.Vec) {
	a, b = csLoopFoot(t, in, fr, i), csLoopFoot(t, in, fr, i+1)
	axis, _ = b.Sub(a).Normalize()
	return a, b, csBarOutward(in, a, b), axis
}

// stepLoopBarRevolve is Loop Bar i revolved a full turn about its own side B0,
// foot to foot: a cylinder of diameter ringWire between the two feet.
func stepLoopBarRevolve(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	f := csLoopFrame(t, in)
	a, b, out, _ := csBarGeometry(t, in, fr, i)
	w := in.P.RingWire / 2
	s := csPlaneSketch(t, f)
	csPolygon(t, s, f, []r3.Vec{a, b, b.Add(out.Scale(w)), a.Add(out.Scale(w))}, fmt.Sprintf("bar %d", i))
	prof := decadtest.SolveRegion(t, s)
	ax, ay := csLocal(t, f, a, "B0 start")
	bx, by := csLocal(t, f, b, "B0 end")
	bar, err := doc.Revolve(s, prof, decad.SketchLine{Start: decad.Point2{U: ax, V: ay}, End: decad.Point2{U: bx, V: by}}, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("bar %d revolve: %v", i, err)
	}
	return []*decad.Body{bar}
}

func checkLoopBarRevolve(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	a, b, _, _ := csBarGeometry(t, in, fr, i)
	w := in.P.RingWire / 2
	csMeasuresVolume(t, fmt.Sprintf("bar %d", i), bodies[0], math.Pi*w*w*b.Sub(a).Len(), 1e-9)
}

// stepLoopBallRevolve is Loop Ball i: the half-disc revolved a full turn about
// its chord Bl, a sphere of diameter ringWire centred on foot i.
//
// SUBSTITUTE: as for the ring, the real sphere is built and held to its exact
// volume, but decad's verification reports its area reading as having no
// tolerance reference, so the gated body is the revolve of the half-disc's
// inscribed half-polygon about the same chord.
func stepLoopBallRevolve(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	s, prof, axis, _ := csBallPolygonSketch(t, in, fr, i)
	ball, err := doc.Revolve(s, prof, axis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("ball %d stand-in revolve: %v", i, err)
	}
	return []*decad.Body{ball}
}

// csBallSketch is Loop Ball i's sketch as the build draws it.
func csBallSketch(t testing.TB, in csIn, fr frame, i int) (*sketch.Sketch, *sketch.Profile, decad.SketchLine) {
	t.Helper()
	f := csLoopFrame(t, in)
	e, _, _ := csAxes()
	foot := csLoopFoot(t, in, fr, i)
	w := in.P.RingWire / 2
	s := csPlaneSketch(t, f)
	sx, sy := csLocal(t, f, foot.Sub(e.Scale(w)), "Bl start")
	ex, ey := csLocal(t, f, foot.Add(e.Scale(w)), "Bl end")
	fx, fy := csLocal(t, f, foot, "the foot")
	start, end := s.CreatePoint(sx, sy), s.CreatePoint(ex, ey)
	chord := s.CreateLine(start, end)
	arc := s.CreateArc(s.CreatePoint(fx, fy), end, start)
	s.Fix(start)
	s.Fix(end)
	s.AddConstraint(sketch.NewPointOnLine(arc.Center, chord))
	return s, decadtest.SolveRegion(t, s), decad.SketchLine{Start: decad.Point2{U: sx, V: sy}, End: decad.Point2{U: ex, V: ey}}
}

// csBallPolygonSketch is the half-disc with its arc replaced by the inscribed
// half-polygon, and the polygon in coordinates whose x axis is the chord's
// perpendicular, for Pappus.
func csBallPolygonSketch(t testing.TB, in csIn, fr frame, i int) (*sketch.Sketch, *sketch.Profile, decad.SketchLine, [][2]float64) {
	t.Helper()
	f := csLoopFrame(t, in)
	e, _, k := csAxes()
	foot := csLoopFoot(t, in, fr, i)
	w := in.P.RingWire / 2
	var poly []r3.Vec
	var about [][2]float64 // (distance from the chord, along the chord)
	for j := 0; j <= csRoundSides/2; j++ {
		a := math.Pi * float64(j) / float64(csRoundSides/2)
		poly = append(poly, foot.Add(e.Scale(w*math.Cos(a))).Add(k.Scale(w*math.Sin(a))))
		about = append(about, [2]float64{w * math.Sin(a), w * math.Cos(a)})
	}
	s := csPlaneSketch(t, f)
	csPolygon(t, s, f, poly, "the ball's half-polygon")
	sx, sy := csLocal(t, f, foot.Add(e.Scale(w)), "Bl end")
	ex, ey := csLocal(t, f, foot.Sub(e.Scale(w)), "Bl start")
	return s, decadtest.SolveRegion(t, s), decad.SketchLine{Start: decad.Point2{U: sx, V: sy}, End: decad.Point2{U: ex, V: ey}}, about
}

func checkLoopBallRevolve(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	w := in.P.RingWire / 2
	s, prof, axis := csBallSketch(t, in, fr, i)
	sphere, err := decad.New().Revolve(s, prof, axis, decad.FullRevolution{})
	if err != nil {
		t.Fatalf("ball %d revolve: %v", i, err)
	}
	csMeasuresVolume(t, fmt.Sprintf("ball %d", i), sphere, 4*math.Pi*w*w*w/3, 1e-9)
	_, _, _, about := csBallPolygonSketch(t, in, fr, i)
	csMeasuresVolume(t, fmt.Sprintf("ball %d's polygon stand-in", i), bodies[0], csPappus(about), 1e-9)
}

// ---- the stand-ins ---------------------------------------------------------

// csRingInscribed is the ring's stand-in: an annulus prism whose section, a
// square of half-side a about the wire's centre, lies inside the wire's circle
// (a*sqrt(2) = 0.976 of its radius), and whose inner and outer edges are
// 64-gons that stay inside the torus. It returns the body and its section.
func csRingInscribed(t testing.TB, doc *decad.Document, in csIn) (*decad.Body, float64, [2][]r3.Vec) {
	t.Helper()
	p := in.P
	e, n, k := csAxes()
	a := 0.69 * p.RingWire / 2
	c := csCentre.Add(n.Scale(p.CageRise))
	outer := csRegular(c, e, k, p.RingRadius+a, csRingSides, 0)
	inner := csRegular(c, e, k, (p.RingRadius-a)/math.Cos(math.Pi/csRingSides), csRingSides, 0)
	f := csFrame(t, c, e, k)
	s := csPlaneSketch(t, f)
	csPolygon(t, s, f, outer, "the ring stand-in's outer edge")
	csPolygon(t, s, f, inner, "the ring stand-in's inner edge")
	sketchtestSolve(t, s)
	var prof *sketch.Profile
	for _, pr := range s.Profiles() {
		if pr.Valid && len(pr.Holes) == 1 {
			prof = pr
		}
	}
	if prof == nil {
		t.Fatalf("the ring stand-in's annulus has no region with one hole")
	}
	body, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(a)})
	if err != nil {
		t.Fatalf("ring stand-in: %v", err)
	}
	return body, a, [2][]r3.Vec{outer, inner}
}

// csRodInscribed is a rod's stand-in: a 16-gon prism inscribed in the rod,
// scaled by scale, from -cageRise to +cageRise.
func csRodInscribed(t testing.TB, doc *decad.Document, in csIn, psi, scale float64) (*decad.Body, []r3.Vec) {
	t.Helper()
	return csRodPolygon(t, doc, in, psi, scale, csRodPhase)
}

func csRodPolygon(t testing.TB, doc *decad.Document, in csIn, psi, scale, phase float64) (*decad.Body, []r3.Vec) {
	t.Helper()
	e, _, k := csAxes()
	foot := csFoot(in.P, psi, 0)
	poly := csRegular(foot, e, k, scale*in.P.RodDiameter/2, csRodSides, phase)
	f := csFrame(t, foot, e, k)
	s, prof, _ := csPolygonRegion(t, f, poly, "the rod stand-in")
	body, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(in.P.CageRise)})
	if err != nil {
		t.Fatalf("rod stand-in: %v", err)
	}
	return body, poly
}

// csBarInscribed is bar i's stand-in: an octagon prism inscribed in the bar's
// cylinder, from foot i to foot i+1.
func csBarInscribed(t testing.TB, doc *decad.Document, in csIn, fr frame, i int) *decad.Body {
	t.Helper()
	_, n, _ := csAxes()
	a, b, _, axis := csBarGeometry(t, in, fr, i)
	side := n.Cross(axis)
	f := csFrame(t, a, side, n) // normal side × n̂ = +axis
	poly := csRegular(a, side, n, in.P.RingWire/2, csBarSides, csBarPhase)
	s, prof, _ := csPolygonRegion(t, f, poly, "the bar stand-in")
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(b.Sub(a).Len()), Dir: decad.Along})
	if err != nil {
		t.Fatalf("bar stand-in: %v", err)
	}
	return body
}

// csBallInscribed is ball i's stand-in: a cube inscribed in the ball, half-side
// ringWire/2/sqrt(3), its faces square to n̂. It returns the body, its
// half-height and its square.
func csBallInscribed(t testing.TB, doc *decad.Document, in csIn, fr frame, i int) (*decad.Body, float64, []r3.Vec) {
	t.Helper()
	e, _, k := csAxes()
	foot := csLoopFoot(t, in, fr, i)
	h := in.P.RingWire / 2 / math.Sqrt(3)
	sq := csRegular(foot, e, k, h*math.Sqrt2, 4, csBallPhase)
	s, prof, _ := csPolygonRegion(t, csFrame(t, foot, e, k), sq, "the ball stand-in")
	body, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(h)})
	if err != nil {
		t.Fatalf("ball stand-in: %v", err)
	}
	return body, h, sq
}

// csCollarOutline is the collar's rounded outline in the section's (u, v):
// every quarter-circle corner of radius collarWall replaced by its inscribed
// chords, counter-clockwise from the end of O1. The eight side ends are points
// of it.
func csCollarOutline(p Params) [][2]float64 {
	return csRoundedOutline(p.Width/2+p.Clearance, p.Thickness/2+p.Clearance, p.CollarWall)
}

// csRoundedOutline is the rectangle of half-sides hu by hv grown by w, its
// corners chorded.
func csRoundedOutline(hu, hv, w float64) [][2]float64 {
	corners := [][2]float64{{hu, -hv}, {hu, hv}, {-hu, hv}, {-hu, -hv}}
	var out [][2]float64
	for j, c := range corners {
		a0 := -math.Pi/2 + float64(j)*math.Pi/2
		for i := 0; i <= csArcChords; i++ {
			a := a0 + math.Pi/2*float64(i)/float64(csArcChords)
			out = append(out, [2]float64{c[0] + w*math.Cos(a), c[1] + w*math.Sin(a)})
		}
	}
	return out
}

// csSweptStandIn is the stand-in for a twisted sweep of a section outline over
// [lo, hi] on gear g's axis: a loft through the section turned by its station's
// own angle s/Lambda + Phi at each of boreSections(turn) stations, the count
// "What the proof's stand-in costs" derives from the sweep's own turn.
func csSweptStandIn(t testing.TB, doc *decad.Document, g Gear, lo, hi float64, outline [][2]float64, what string) (*decad.Body, [][]r3.Vec) {
	t.Helper()
	count := boreSections(g.P, (hi-lo)/g.P.Lambda())
	frames := make([]r3.Frame, count)
	sections := make([][]r3.Vec, count)
	for k := range count {
		st := lo + (hi-lo)*float64(k)/float64(count-1)
		frames[k] = csSectionFrame(t, g, st)
		for _, uv := range outline {
			sections[k] = append(sections[k], g.world(uv[0], uv[1], st))
		}
	}
	return csRuledLoft(t, doc, frames, sections, what), sections
}

func csBoreOutline(p Params) [][2]float64 {
	uB, uF, hv := -(p.Width/2 + p.Clearance), p.Width/2+p.Clearance, p.Thickness/2+p.Clearance
	return [][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}}
}

// csCollarAt is collar i's stand-in, and the gear and station it stands at.
func csCollarAt(t testing.TB, doc *decad.Document, in csIn, i int) (*decad.Body, [][]r3.Vec, Gear, float64) {
	t.Helper()
	p := in.P
	gi, sc := csCollarStation(p, i)
	g := csGears(in)[gi]
	body, sections := csSweptStandIn(t, doc, g, sc-p.CollarHalf, sc+p.CollarHalf, csCollarOutline(p),
		fmt.Sprintf("collar %d", i))
	return body, sections, g, sc
}

// csUnder holds a reading strictly under a limit, its own bound included: the
// one comparison here that is an inequality, since an overlap of two pieces
// whose shapes cross obliquely has no closed form to measure against.
func csUnder(t testing.TB, what string, got decad.Measurement, limit float64) {
	t.Helper()
	if hi := got.Value.Base() + got.Bound.Base(); !(hi < limit) {
		t.Fatalf("%s: %.6f (bound %.2e) is not under %.6f", what, got.Value.Base(), got.Bound.Base(), limit)
	}
}

func csVolume(t testing.TB, what string, b *decad.Body) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	if err != nil {
		t.Fatalf("%s volume: %v", what, err)
	}
	return v
}

func csUnion(t testing.TB, a, b *decad.Body, what string) *decad.Body {
	t.Helper()
	u, err := decad.Union(context.Background(), a, b)
	if err != nil {
		t.Fatalf("%s: %v", what, err)
	}
	return u
}

// ---- the joins -------------------------------------------------------------

// stepRodsJoin is the rods' join into the ring: one combineFeatures join, the
// ring as target and the four rods as tools, leaving exactly one body. One case
// per rod: the rod's stand-in united with the ring's, which the gate reads as
// one solid lump, their overlap measured against its closed form.
func stepRodsJoin(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	ring, _, _ := csRingInscribed(t, doc, in)
	rod, _ := csRodInscribed(t, doc, in, csFootAzimuths(fr)[i], 1)
	return []*decad.Body{csUnion(t, ring, rod, fmt.Sprintf("ring and rod %d", i))}
}

func checkRodsJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	scratch := decad.New()
	ring, a, edges := csRingInscribed(t, scratch, in)
	rod, poly := csRodInscribed(t, scratch, in, csFootAzimuths(fr)[i], 1)
	// Both are prisms along n̂. The rod's top, at +cageRise, stands a above the
	// stand-in ring's bottom face, so their overlap is the rod's section inside
	// the annulus times a.
	rodFlat := csFlat(poly)
	overlapArea := csArea(csClip(rodFlat, csFlat(edges[0]))) - csArea(csClip(rodFlat, csFlat(edges[1])))
	if !(overlapArea > 0) {
		t.Fatalf("rod %d's section does not reach the ring's", i)
	}
	vRing, vRod := csVolume(t, "the ring stand-in", ring), csVolume(t, "the rod stand-in", rod)
	want := vRing.Value.Base() + vRod.Value.Base() - overlapArea*a
	// The oracle is closed-form polygon arithmetic on coordinates of order 100 mm.
	csMeasuresVolume(t, fmt.Sprintf("ring joined with rod %d", i), bodies[0], want,
		vRing.Bound.Base()+vRod.Bound.Base()+1e-9*want)
}

// stepLoopJoin is the loop's join: its four bars and four balls, made as new
// bodies, joined into the cage in one combine of eight tools. One case per
// tool: the tool's stand-in united with the stand-in of the rod that stands on
// its foot — bar i's first foot, or ball i's.
func stepLoopJoin(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	j := int(csNeed(t, m, csIndexKey))
	rod, _ := csRodInscribed(t, doc, in, csLoopRod(fr, j%4), 1)
	var piece *decad.Body
	if j < 4 {
		piece = csBarInscribed(t, doc, in, fr, j)
	} else {
		piece, _, _ = csBallInscribed(t, doc, in, fr, j-4)
	}
	return []*decad.Body{csUnion(t, rod, piece, fmt.Sprintf("loop piece %d and its rod", j))}
}

// csLoopRod is the azimuth of the rod standing on foot i of the loop.
func csLoopRod(fr frame, i int) float64 {
	az := csFootAzimuths(fr)
	return az[csFootOrder(az)[i]]
}

func checkLoopJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	j := int(csNeed(t, m, csIndexKey))
	scratch := decad.New()
	rod, poly := csRodInscribed(t, scratch, in, csLoopRod(fr, j%4), 1)
	vRod := csVolume(t, "the rod stand-in", rod)
	joined := csVolume(t, "the joined pair", bodies[0])
	if j < 4 {
		bar := csBarInscribed(t, scratch, in, fr, j)
		vBar := csVolume(t, "the bar stand-in", bar)
		// The bar's end cap stands across the rod's axis at the rod's own foot,
		// so the two overlap; the overlap of an octagon prism and a 16-gon prism
		// crossing at right angles has no closed form this oracle writes down,
		// so the proof holds the joined volume under the pieces' sum and over
		// the larger piece.
		csUnder(t, "the joined bar and rod against their sum", joined,
			vBar.Value.Base()+vRod.Value.Base()-vBar.Bound.Base()-vRod.Bound.Base())
		if lo := math.Max(vBar.Value.Base(), vRod.Value.Base()); joined.Value.Base()+joined.Bound.Base() < lo {
			t.Fatalf("the joined bar and rod read %.6f, under the larger piece's %.6f", joined.Value.Base(), lo)
		}
		return
	}
	ball, h, sq := csBallInscribed(t, scratch, in, fr, j-4)
	vBall := csVolume(t, "the ball stand-in", ball)
	// The cube and the rod are both prisms along n̂; the rod ends at the foot,
	// the cube's middle, so they share the rod's section inside the square over
	// the cube's upper half-height.
	overlap := csArea(csClip(csFlat(poly), csFlat(sq))) * h
	want := vBall.Value.Base() + vRod.Value.Base() - overlap
	csMeasuresVolume(t, "the joined ball and rod", bodies[0], want, vBall.Bound.Base()+vRod.Bound.Base()+1e-9*want)
}

// ---- the collars -----------------------------------------------------------

// stepCollarSweep is one collar: one sweep of its section along its collar
// line, twisted by +2*collarHalf/Lambda, as a new body of ten faces.
//
// SUBSTITUTE: decad has no twisted sweep (WithSweepTwist of anything but zero
// is ErrUnsupported). The stand-in is a loft through the collar's section,
// turned by s/Lambda + Phi, at boreSections(2*collarHalf/Lambda) stations, its
// corner arcs chorded because a loft pairs a chorded arc only within a station
// cap. Each chord lies inside its arc, so the stand-in lies inside the collar.
// What the proof holds is the stand-in's own shape: every section's eight side
// ends are vertices of it at both ends — the points [SCREW-F-SWEEP-CHECK] reads
// — and its volume is the triangulated loft's. The twist's sense and linearity
// are Fusion's, measured on 2026-09-28 and re-checked by the build's end-face
// check on every run; nothing here can see them.
func stepCollarSweep(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	csCheckInputs(t, in)
	body, _, _, _ := csCollarAt(t, doc, in, int(csNeed(t, m, csIndexKey)))
	return []*decad.Body{body}
}

func checkCollarSweep(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	p := in.P
	i := int(csNeed(t, m, csIndexKey))
	gi, sc := csCollarStation(p, i)
	g := csGears(in)[gi]
	body := bodies[0]
	count := boreSections(p, 2*p.CollarHalf/p.Lambda())
	var sections [][]r3.Vec
	for k := range count {
		st := sc - p.CollarHalf + 2*p.CollarHalf*float64(k)/float64(count-1)
		var sec []r3.Vec
		for _, uv := range csCollarOutline(p) {
			sec = append(sec, g.world(uv[0], uv[1], st))
		}
		sections = append(sections, sec)
	}
	want, slack := csRuledVolume(sections)
	csMeasuresVolume(t, fmt.Sprintf("collar %d stand-in", i), body, want, slack)

	uB, uF, hv, w := -(p.Width/2 + p.Clearance), p.Width/2+p.Clearance, p.Thickness/2+p.Clearance, p.CollarWall
	ends := [][2]float64{{uB, -hv - w}, {uF, -hv - w}, {uF + w, -hv}, {uF + w, hv}, {uF, hv + w}, {uB, hv + w}, {uB - w, hv}, {uB - w, -hv}}
	for _, st := range []float64{sc - p.CollarHalf, sc + p.CollarHalf} {
		for j, uv := range ends {
			csHasVertex(t, fmt.Sprintf("side end %d at station %+.3f", j, st), body, g.world(uv[0], uv[1], st))
		}
	}
	area := (uF-uB+2*w)*(2*hv+2*w) - (4-math.Pi)*w*w
	v := csVolume(t, "the collar stand-in", body)
	t.Logf("collar %d: %d sections over %.2f degrees; stand-in %.4f mm³ against the swept collar's exact %.4f",
		i, count, 2*p.CollarHalf/p.Lambda()*180/math.Pi, v.Value.Base(), area*2*p.CollarHalf)
}

// csSlabHalf is half the thickness of the collar slab the collars' join is
// proved on. Over it the section turns by 2*csSlabHalf/Lambda, and the slab's
// outline is pulled in and its hole let out by more than that turn moves any
// point of them, so the slab lies inside the twisted collar.
const csSlabHalf = 0.05

// csNotch is how far the slab's outline is notched in where the rod crosses it.
const csNotch = 0.01

// csCollarSlab is the part of collar i its rod runs through: the collar's
// section at the rod's own station sp, the chorded outline pulled in and the
// bore's rectangle let out by more than the section turns over +/-csSlabHalf,
// extruded csSlabHalf either side along the gear's axis. Its faces are planar
// and straight, which decad holds exactly.
func csCollarSlab(t testing.TB, doc *decad.Document, in csIn, fr frame, i int) (*decad.Body, float64) {
	t.Helper()
	p := in.P
	gi, _ := csCollarStation(p, i)
	g := csGears(in)[gi]
	_, sp := fr.rodWallDepth(fr.cross[i])
	turn := csSlabHalf / p.Lambda()
	hu, hv := p.Width/2+p.Clearance, p.Thickness/2+p.Clearance
	// Over +/-csSlabHalf a point at radius rho moves at most rho*turn, so the
	// hole is let out and the outline pulled in by a little more than their
	// farthest points move: the hole's rectangle grows that much on every side,
	// and the outline's corner radius shrinks by it.
	outDelta := 1.05 * math.Hypot(hu+p.CollarWall, hv+p.CollarWall) * turn
	inDelta := 1.05 * math.Hypot(hu, hv) * turn
	var outer, hole []r3.Vec
	for _, uv := range csRoundedOutline(hu, hv, p.CollarWall-outDelta) {
		outer = append(outer, g.world(uv[0], uv[1], sp))
	}
	for _, uv := range [][2]float64{{-hu - inDelta, -hv - inDelta}, {hu + inDelta, -hv - inDelta}, {hu + inDelta, hv + inDelta}, {-hu - inDelta, hv + inDelta}} {
		hole = append(hole, g.world(uv[0], uv[1], sp))
	}
	// Notch the outline where the rod's axis crosses it. decad admits a union
	// of two crossing faces only on a witness — a sample point of one proven
	// deep inside the other — and a rod face crossing a long outline face
	// between that face's corners offers none. So each crossing point becomes a
	// vertex, pushed csNotch into the wall so that decad does not merge it back
	// into a straight edge: it stands on the rod's axis to within csNotch, deep
	// inside the rod, and the notch only takes material away from the slab.
	o := g.Origin.Add(g.Ez.Scale(sp))
	yF := csFoot(p, fr.cross[i].rod, 0).Sub(o).Dot(g.Ey)
	var split []r3.Vec
	for j := range outer {
		a, b := outer[j], outer[(j+1)%len(outer)]
		split = append(split, a)
		ya, yb := a.Sub(o).Dot(g.Ey)-yF, b.Sub(o).Dot(g.Ey)-yF
		if (ya < 0) != (yb < 0) {
			at := a.Add(b.Sub(a).Scale(ya / (ya - yb)))
			// Inward is toward the axis point o, the outline being convex about it.
			inward, _ := o.Sub(at).Normalize()
			split = append(split, at.Add(inward.Scale(csNotch)))
		}
	}
	if len(split) != len(outer)+2 {
		t.Fatalf("the rod's axis crosses collar %d's outline %d times; it runs through the wall and crosses it twice",
			i, len(split)-len(outer))
	}
	outer = split
	f := csSectionFrame(t, g, sp)
	s := csPlaneSketch(t, f)
	csPolygon(t, s, f, outer, "the slab's outline")
	csPolygon(t, s, f, hole, "the slab's hole")
	sketchtestSolve(t, s)
	var prof *sketch.Profile
	for _, pr := range s.Profiles() {
		if pr.Valid && len(pr.Holes) == 1 {
			prof = pr
		}
	}
	if prof == nil {
		t.Fatalf("collar %d's slab has no region with one hole", i)
	}
	body, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(csSlabHalf)})
	if err != nil {
		t.Fatalf("collar %d's slab: %v", i, err)
	}
	return body, sp
}

// stepCollarsJoin is the collars' join: the four collars, made as new bodies,
// joined into the cage in one combine of four tools. What joins a collar to the
// cage is its rod, which runs through the collar's wall at one station, so one
// case per collar unites the rod's stand-in with the slab of the collar it
// crosses.
//
// SUBSTITUTE: decad refuses to unite the whole collar stand-in with a rod. It
// admits two crossing faces only when it can sample a point of one proven deep
// inside the other, and a rod face crossing the middle of one of the loft's long
// wall facets offers none ("the operands' held facets come within the chord
// tolerance without provably interpenetrating deeper than it"); about half the
// cases tried were refused, whatever the rod's polygon, phase or size. The slab
// is planar, lies inside the collar, and is notched where the rod's axis
// crosses its outline, which puts a vertex deep inside the rod; a rod that
// overlaps the slab overlaps the collar.
func stepCollarsJoin(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	slab, _ := csCollarSlab(t, doc, in, fr, i)
	rod, _ := csRodInscribed(t, doc, in, csFootAzimuths(fr)[i], 1)
	return []*decad.Body{csUnion(t, slab, rod, fmt.Sprintf("collar %d's slab and its rod", i))}
}

func checkCollarsJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	fr := csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	slab, sp := csCollarSlab(t, decad.New(), in, fr, i)
	rod, _ := csRodInscribed(t, decad.New(), in, csFootAzimuths(fr)[i], 1)
	vs, vr := csVolume(t, "the slab", slab), csVolume(t, "the rod stand-in", rod)
	joined := csVolume(t, "the joined pair", bodies[0])
	// The rod crosses the slab obliquely, so the overlap has no closed form
	// here: the joined volume is held under the sum and over the larger piece.
	csUnder(t, fmt.Sprintf("collar %d's slab joined with its rod, against their sum", i), joined,
		vs.Value.Base()+vr.Value.Base()-vs.Bound.Base()-vr.Bound.Base())
	if lo := math.Max(vs.Value.Base(), vr.Value.Base()); joined.Value.Base()+joined.Bound.Base() < lo {
		t.Fatalf("collar %d's slab joined with its rod reads under the larger piece", i)
	}
	depth, _ := fr.rodWallDepth(fr.cross[i])
	t.Logf("collar %d: the rod crosses at station %+.3f, its axis %.3f mm outside the bore", i, sp, depth)
}

// ---- the bores -------------------------------------------------------------

// stepBoreCut is one bore: a sweep of the bore's rectangle along its bore line,
// twisted by +2*(collarHalf + 1 mm)/Lambda, as a cut whose participant list
// holds the cage alone, leaving exactly one body.
//
// SUBSTITUTE: the twisted sweep is the collar's stand-in again, a loft of the
// rectangle at boreSections(turn) stations over the bore's span, and the cut is
// decad's Cut of the collar's stand-in, which touches its target alone as a
// participant list does; that the ribbons are left whole is Fusion's, measured
// on 2026-09-28. The rod is not in the cut: the stand-ins cannot be united with
// the whole collar (see the collars' join), and the real rod reaches into the
// bore by at most rodDiameter/2 - depth, 0.01 mm at the defaults, which the
// build's single cut takes out with the channel. The cut must leave one lump,
// and what it removes must be the channel's intersection with the collar.
func stepBoreCut(t *testing.T, doc *decad.Document, m map[string]float64) []*decad.Body {
	in := csRead(t, m)
	csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	collar, bore := csCollarAndBore(t, doc, in, i)
	cut, err := decad.Cut(context.Background(), collar, bore)
	if err != nil {
		t.Fatalf("bore %d cut: %v", i, err)
	}
	return []*decad.Body{cut}
}

func csCollarAndBore(t testing.TB, doc *decad.Document, in csIn, i int) (*decad.Body, *decad.Body) {
	t.Helper()
	collar, _, g, sc := csCollarAt(t, doc, in, i)
	lo, hi := boreSpan(in.P, sc)
	bore, _ := csSweptStandIn(t, doc, g, lo, hi, csBoreOutline(in.P), fmt.Sprintf("bore %d", i))
	return collar, bore
}

func checkBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, m map[string]float64) {
	in := csRead(t, m)
	csCheckInputs(t, in)
	i := int(csNeed(t, m, csIndexKey))
	before, _ := csCollarAndBore(t, decad.New(), in, i)
	collar, bore := csCollarAndBore(t, decad.New(), in, i)
	removed, err := decad.Intersect(context.Background(), collar, bore)
	if err != nil {
		t.Fatalf("the channel's intersection with collar %d: %v", i, err)
	}
	vb, vr, va := csVolume(t, "before", before), csVolume(t, "removed", removed), csVolume(t, "after", bodies[0])
	if !(vr.Value.Base() > 0) {
		t.Fatalf("bore %d removes nothing from its collar", i)
	}
	sum := decad.Measurement{
		Value:     units.CubicMillimeters(va.Value.Base() + vr.Value.Base()),
		Exactness: decad.Approximate,
		Bound:     units.CubicMillimeters(va.Bound.Base() + vr.Bound.Base()),
	}
	// Both sides are readings of exact-arithmetic booleans on the same facets;
	// the slack is float summation over them.
	decadtest.Agree(t, fmt.Sprintf("collar %d after the cut plus the channel, against before", i), sum, vb,
		decadtest.Within(units.CubicMillimeters(1e-6)))
	p := in.P
	t.Logf("bore %d removes %.4f mm³; the channel's rectangle over the collar's length is %.4f mm³",
		i, vr.Value.Base(), (p.Width+2*p.Clearance)*(p.Thickness+2*p.Clearance)*2*p.CollarHalf)
}

// sketchtestSolve solves a solid step's own sketch before its profiles are read.
func sketchtestSolve(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	csSolve(t, s)
}

// csArcChords is how many chords replace each quarter-circle corner of a
// collar's outline in its stand-in.
const csArcChords = 6
