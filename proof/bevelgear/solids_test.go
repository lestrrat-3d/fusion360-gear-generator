package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The solid proofs work in ONE frame per gear: the apex at the world origin, +X
// along that gear's shaft axis, +Y the in-axial-plane perpendicular on the side
// the gear's material lies, +Z out of the axial plane. Every §2 point maps into
// it, the shaft-axis rotations are rotations about +X, and a point's cone
// distance is its projection on the root cone element. It is the same figure §2
// draws, read in the frame the body operations use.
type frame struct {
	apex, u, v vec
}

// gearFrame builds one gear's shaft frame from the solved lattice.
func gearFrame(l lattice, pinion bool) frame {
	shaft := unit(sub(l.A, l.Apex))
	off := l.C
	if !pinion {
		shaft = unit(sub(l.B, l.Apex))
		off = l.D
	}
	perp := left(shaft)
	if dot(perp, sub(off, l.Apex)) < 0 {
		perp = scale(perp, -1)
	}
	return frame{apex: l.Apex, u: shaft, v: unit(perp)}
}

// at maps an axial-plane point into the shaft frame: x is its station along the
// shaft, y its perpendicular offset.
func (f frame) at(p vec) vec {
	d := sub(p, f.apex)
	return vec{dot(d, f.u), dot(d, f.v)}
}

// dir maps an axial-plane direction into the shaft frame.
func (f frame) dir(d vec) vec { return vec{dot(d, f.u), dot(d, f.v)} }

// gearGeometry is everything the solid steps read off the lattice for one gear,
// already in that gear's shaft frame.
type gearGeometry struct {
	frame    frame
	teeth    float64 // the real tooth number this gear is patterned to
	dims     involute.Dimensions
	virtual  int
	centre   vec // the tooth-centre point K' or L'
	hat      vec // the dedendum direction, C->K' or D->L'
	coneVec  vec // the root cone element, unit apex->C or apex->D
	toe      vec // M or O
	heel     vec // C or D
	toeMid   vec // midpoint of the toe edge M->N or O->P
	heelMid  vec // midpoint of the heel edge C->H or D->J
	gamma    float64
	rTip     float64 // the section's tip radius, straight from the virtual gear
	apexDist float64 // perpendicular distance from the apex to the tooth plane
}

// gearOf reads one gear's geometry out of the solved lattice.
func gearOf(p params) gearGeometry {
	l := solveLattice(p)
	f := gearFrame(l, p.Pinion)
	_, gamma, teeth := toothOf(p)
	g := gearGeometry{
		frame:   f,
		teeth:   p.DrivingTeeth,
		virtual: teeth,
		dims:    involute.Derive(p.Module, float64(teeth), pressureAngle),
		centre:  f.at(l.LPrime),
		hat:     f.dir(l.DHat),
		coneVec: unit(f.at(l.D)),
		toe:     f.at(l.O),
		heel:    f.at(l.D),
		toeMid:  f.at(scale(add(l.O, l.P), 0.5)),
		heelMid: f.at(scale(add(l.D, l.J), 0.5)),
		gamma:   gamma,
	}
	if p.Pinion {
		g.teeth = p.PinionTeeth
		g.centre = f.at(l.KPrime)
		g.hat = f.dir(l.CHat)
		g.coneVec = unit(f.at(l.C))
		g.toe = f.at(l.M)
		g.heel = f.at(l.C)
		g.toeMid = f.at(scale(add(l.M, l.N), 0.5))
		g.heelMid = f.at(scale(add(l.C, l.H), 0.5))
	}
	g.rTip = g.dims.Tip
	normal := vec{g.hat.Y, -g.hat.X}
	g.apexDist = dot(scale(g.centre, -1), normal)
	return g
}

// distAlong is a shaft-frame point's cone distance: its projection on the root
// cone element measured from the apex.
func (g gearGeometry) distAlong(p vec) float64 { return dot(p, g.coneVec) }

// section returns one cross-section of the straight tooth, in world 3-D, at the
// tooth-plane parameter t. t = 1 is the tooth profile itself and t = 0 the apex,
// so the section is the drawn tooth scaled by t about the apex.
//
// The outline is the polygon through the same involute samples the tooth sketch
// draws its two fitted splines from, closed at the root by a chord. A loft ruled
// between two spline sections and one ruled between the polygons through their
// samples differ only by the sampling, and the polygon keeps every section a
// single closed loop the harness will loft and extrude.
func (g gearGeometry) section(t float64) []r3.Vec {
	leftPts, rightPts := involute.Flanks(g.dims.Base, g.dims.Tip, g.dims.Pitch,
		float64(g.virtual), involuteSteps, math.Pi)
	outline := make([]involute.Pt, 0, 2*involuteSteps)
	for i := len(leftPts) - 1; i >= 0; i-- {
		outline = append(outline, leftPts[i])
	}
	outline = append(outline, rightPts...)

	origin := scale(g.centre, t)
	out := make([]r3.Vec, 0, len(outline))
	for _, pt := range outline {
		plane := add(origin, scale(g.hat, t*pt.X))
		out = append(out, r3.NewVec(plane.X, plane.Y, t*pt.Y))
	}
	return out
}

// rotateAboutShaft turns a section about the shaft axis, which is +X.
func rotateAboutShaft(pts []r3.Vec, angle float64) []r3.Vec {
	s, c := math.Sin(angle), math.Cos(angle)
	out := make([]r3.Vec, 0, len(pts))
	for _, p := range pts {
		out = append(out, r3.NewVec(p.X, p.Y*c-p.Z*s, p.Y*s+p.Z*c))
	}
	return out
}

// scaleAbout shrinks a section uniformly toward base, which is the crown's
// uniform scale about a point on the heel face's root edge.
func scaleAbout(pts []r3.Vec, base r3.Vec, factor float64) []r3.Vec {
	out := make([]r3.Vec, 0, len(pts))
	for _, p := range pts {
		out = append(out, r3.NewVec(
			base.X+(p.X-base.X)*factor,
			base.Y+(p.Y-base.Y)*factor,
			base.Z+(p.Z-base.Z)*factor))
	}
	return out
}

// planarSection commits one planar loop of world points as a sketch on its own
// plane, as a closed polygon with every vertex fixed once the lines exist.
func planarSection(w *sketch.World, pts []r3.Vec) (*sketch.Sketch, error) {
	origin := pts[0]
	u := pts[1].Sub(origin)
	var v r3.Vec
	for _, p := range pts[2:] {
		cand := p.Sub(origin)
		if u.Cross(cand).Len() > 1e-9*u.Len()*cand.Len() {
			v = cand
			break
		}
	}
	plane, err := w.CreatePlaneFromPoints(origin, origin.Add(u), origin.Add(v))
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	f, err := plane.Frame()
	if err != nil {
		return nil, err
	}
	handles := make([]*sketch.Point, 0, len(pts))
	for _, p := range pts {
		local := f.ToLocal(p)
		handles = append(handles, s.CreatePoint(local.X, local.Y))
	}
	for i := range handles {
		s.CreateLine(handles[i], handles[(i+1)%len(handles)])
	}
	for _, h := range handles {
		s.Fix(h)
	}
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, err
	}
	return s, nil
}

// loftBetween lofts a solid between two planar sections.
func loftBetween(doc *decad.Document, a, b []r3.Vec) (*decad.Body, error) {
	w := sketch.NewWorld()
	s0, err := planarSection(w, a)
	if err != nil {
		return nil, err
	}
	s1, err := planarSection(w, b)
	if err != nil {
		return nil, err
	}
	return doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
}

// mustVolume reads a body's volume in cubic millimetres.
func mustVolume(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	v, err := m.Value.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("volume units: %v", err)
	}
	return v
}

// centroidOf reads a body's centroid in the shaft frame.
func centroidOf(t *testing.T, b *decad.Body) r3.Vec {
	t.Helper()
	c, err := b.Centroid()
	if err != nil {
		t.Fatalf("centroid: %v", err)
	}
	return c.Value
}

// transverse is the tooth's outline seen ALONG the shaft axis: the heel section's
// points projected onto the plane through their mean station, perpendicular to
// the shaft.
//
// It is what the pattern, the combine and the bore stand on. Those three steps
// are decided by the tooth's angular footprint about the shaft, the tooth count
// and the bore diameter, none of which the taper changes, and decad's booleans
// take a prism where they refuse a loft — its own message names prism, cup and
// faceted as the payloads it tessellates. So the proof extrudes this footprint
// where the gear extrudes nothing, and what that costs is the taper: the prism
// stands where the tapered tooth stands and reaches the same radii, but its
// section does not shrink toward the apex.
func (g gearGeometry) transverse() []r3.Vec {
	pts := g.section(1)
	mean := 0.0
	for _, p := range pts {
		mean += p.X
	}
	mean /= float64(len(pts))
	out := make([]r3.Vec, 0, len(pts))
	for _, p := range pts {
		out = append(out, r3.NewVec(mean, p.Y, p.Z))
	}
	return out
}

// radialExtent returns the smallest and largest distance from the shaft axis over
// a set of world points.
func radialExtent(pts []r3.Vec) (lo, hi float64) {
	lo, hi = math.Inf(1), 0
	for _, p := range pts {
		r := math.Hypot(p.Y, p.Z)
		lo, hi = math.Min(lo, r), math.Max(hi, r)
	}
	return lo, hi
}

// angularExtent returns the half-width of a set of world points about the shaft
// axis, in radians.
func angularExtent(pts []r3.Vec) float64 {
	widest := 0.0
	for _, p := range pts {
		widest = math.Max(widest, math.Abs(math.Atan2(p.Z, p.Y)))
	}
	return widest
}

// extrudeAlongShaft extrudes a section that lies in a plane perpendicular to the
// shaft, by length along the shaft.
func extrudeAlongShaft(doc *decad.Document, pts []r3.Vec, length float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := planarSection(w, pts)
	if err != nil {
		return nil, err
	}
	return doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(math.Abs(length)), Dir: decad.Along})
}

// diskSection returns a circle of the given radius, centred on the shaft axis at
// the given station, as a closed polygon fine enough to carry the radius.
func diskSection(station, radius float64, sides int) []r3.Vec {
	out := make([]r3.Vec, 0, sides)
	for i := range sides {
		a := 2 * math.Pi * float64(i) / float64(sides)
		out = append(out, r3.NewVec(station, radius*math.Cos(a), radius*math.Sin(a)))
	}
	return out
}

// rotationAboutShaft is the free-move matrix the meshing rotations and the
// circular pattern are built on: a rotation about the shaft axis through the
// apex.
func rotationAboutShaft(angle float64) (r3.Transform, error) {
	return r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), units.Radians(angle))
}
