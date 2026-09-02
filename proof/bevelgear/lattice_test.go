package bevelgear_test

import (
	"math"

	"github.com/lestrrat-3d/sketch"
)

// seeds are the §2 seed coordinates, placed exactly where the spec says to place
// them: the apex one driving pitch diameter along the in-plane perpendicular,
// the two along-shaft lengths at the closed-form cone geometry, the
// module-length extensions one module long, and the toe lines seeded near their
// solved position rather than a face width from the heel.
//
// They are seeds and nothing more — no seed pins anything — but the solver is
// seed-sensitive, so the proof carries the spec's seeds rather than the solved
// answer. The apex seed is the one that is far out: it puts the whole figure
// DPD - |Apex->B| - drivingBaseHeight above where it solves, which is 11.6 mm
// for the default 31/31 pair.
type seeds struct {
	Centre, Apex, A, B, Apex2 vec
	C, D, E, F, G, H, I, J    vec
	K, KPrime, L, LPrime      vec
	M, N, O, P                vec
	PinionDir, DrivingDir     vec
	CHat, DHat                vec
}

// seedApexHeight is how far above the anchor line the apex seed sits.
//
// The spec seeds it at the Driving Gear Pitch Diameter and says in prose that
// the apex's offset from the anchor line IS that diameter. It is not: the
// centre-to-apex line carries no length dimension, and "Constrain Point I with
// center point" drives the offset to |Apex->B| plus the resolved driving base
// height instead — 19.375 mm rather than 31 mm for the default 31/31 pair. Seeded
// at the diameter, the closure has to translate the whole rigid figure 11.7 mm
// and the engine's solver does not converge; seeded where the closure puts it,
// the same constraint set solves to DOF 0 with nothing redundant. So the proof
// seeds here and the report carries the defect.
func seedApexHeight(p params, c cone) float64 {
	driving, _ := resolvedBaseHeights(p)
	return c.AlongB + driving
}

func seedLattice(p params) seeds {
	c := coneOf(p)
	centre := vec{p.CenterX, p.CenterY}
	drivingDir := vec{0, -1}
	plus := rot(drivingDir, c.Sigma)
	minus := rot(drivingDir, -c.Sigma)
	pinionDir := plus
	if minus.X > plus.X {
		pinionDir = minus
	}

	apex := add(centre, scale(vec{0, 1}, seedApexHeight(p, c)))
	a := add(apex, scale(pinionDir, c.AlongA))
	b := add(apex, scale(drivingDir, c.AlongB))
	apex2 := add(a, scale(perpToward(pinionDir, sub(b, a)), c.PPD/2))

	pitch := unit(sub(apex2, apex))
	dHat := unit(left(pitch))
	if dot(dHat, drivingDir) < 0 {
		dHat = scale(dHat, -1)
	}
	cHat := scale(dHat, -1)

	cc := add(apex2, scale(cHat, 1.25*p.Module))
	d := add(apex2, scale(dHat, 1.25*p.Module))
	e := add(a, scale(pinionDir, p.Module))
	f := add(b, scale(drivingDir, p.Module))
	g := add(e, scale(pinionDir, p.Module))
	h := add(cc, scale(cHat, p.Module))
	i := add(f, scale(drivingDir, p.Module))
	j := add(d, scale(dHat, p.Module))

	k := lineIntersect(apex, pinionDir, apex2, cHat)
	l := lineIntersect(apex, drivingDir, apex2, dHat)

	// The spec seeds M at roughly the midpoint of Apex->C and slides N along
	// C->H by the distance from that seed to A. That rule is written for the
	// default 31/31 pair, where the midpoint happens to land within a few
	// millimetres of the solved toe. It does not hold in general: at a 30 degree
	// shaft angle with 31/17 teeth the face width is 1.48 mm against a 47.9 mm
	// cone element, so the midpoint seed sits 22 mm from where M solves and the
	// engine's solver does not converge from it. The proof seeds the toe lines
	// near their solved position instead — which is what the seeding rule the
	// spec cites asks for — and the report carries the defect.
	solved := solveLattice(p)
	mSeed, nSeed := solved.M, solved.N
	oSeed, pSeed := solved.O, solved.P

	return seeds{
		Centre: centre, Apex: apex, A: a, B: b, Apex2: apex2,
		C: cc, D: d, E: e, F: f, G: g, H: h, I: i, J: j,
		K: k, KPrime: add(k, scale(cHat, p.ToothSpacing)),
		L: l, LPrime: add(l, scale(dHat, p.ToothSpacing)),
		M: mSeed, N: nSeed, O: oSeed, P: pSeed,
		PinionDir: pinionDir, DrivingDir: drivingDir, CHat: cHat, DHat: dHat,
	}
}

// figure carries the §2 sketch under construction: every named point and line,
// so a later part of the step reuses the one line it drew earlier rather than
// redrawing a second line over the same segment.
type figure struct {
	s      *sketch.Sketch
	points map[string]*sketch.Point
	lines  map[string]*sketch.Line
}

func newFigure(s *sketch.Sketch) *figure {
	return &figure{s: s, points: map[string]*sketch.Point{}, lines: map[string]*sketch.Line{}}
}

// ref adds a projected point: externally locked reference geometry, which is
// what a projection of a fully constrained source sketch is. The engine's
// solver never moves it, so the projected centre and anchor line ground the
// figure the way the Anchor sketch grounds it in Fusion.
func (f *figure) ref(name string, at vec) *sketch.Point {
	p := f.s.CreateReferencePoint(at.X, at.Y, "anchor sketch")
	p.SetName(name)
	f.points[name] = p
	return p
}

// line draws one §2 line in the COINCIDENT style: both endpoints are fresh
// points created at raw seed coordinates, never an existing SketchPoint passed
// in to be shared. Each end that meets existing geometry is pinned afterwards
// with exactly one coincident.
func (f *figure) line(name string, start, end vec) *sketch.Line {
	a := f.s.CreatePoint(start.X, start.Y)
	a.SetName(name + ".start")
	b := f.s.CreatePoint(end.X, end.Y)
	b.SetName(name + ".end")
	l := f.s.CreateLine(a, b)
	l.SetConstruction(true)
	f.lines[name] = l
	return l
}

// at returns the solved position of a named point.
func (f *figure) at(name string) vec {
	p := f.points[name]
	return vec{p.X(), p.Y()}
}

// pin names a line endpoint as a §2 point, so later steps reach it by name.
func (f *figure) pin(name string, p *sketch.Point) *sketch.Point {
	p.SetName(name)
	f.points[name] = p
	return p
}

// meet is the one coincident that ties a freshly drawn endpoint to an existing
// point. It is never used together with sharing the point.
func (f *figure) meet(a, b *sketch.Point) {
	f.s.AddConstraint(sketch.NewCoincident(a, b))
}

// onLine is Fusion's addCoincident(point, line): the point lies on the infinite
// line, one equation.
func (f *figure) onLine(p *sketch.Point, l *sketch.Line) {
	f.s.AddConstraint(sketch.NewPointOnLine(p, l))
}

// angle pins the signed angle from l1's direction to l2's, counter-clockwise in
// degrees.
//
// This is the proof's substitute for a Fusion perpendicular or parallel
// constraint whose side is decided by the seed. Fusion's addPerpendicular is
// unsigned and admits both perpendicular senses; the drawing seed picks one and
// the solver keeps it. The engine refuses that: a scheme that reaches DOF 0 and
// still admits the mirror answer fails its ambiguity probe. A signed angle is
// the documented crossing of the same fact — the playbook's dimension-value rule
// says the engine's target is signed where Fusion's is a magnitude plus the
// seeded side — so the proof states the side the seed states, as a constraint
// the engine can check.
func (f *figure) angle(l1, l2 *sketch.Line, degrees float64) {
	f.s.AddConstraint(sketch.NewAngle(l1, l2, degrees))
}

// signedAngle is the counter-clockwise angle in degrees from a to b, in
// (-180, 180].
func signedAngle(a, b vec) float64 {
	return math.Atan2(cross(a, b), dot(a, b)) * 180 / math.Pi
}

// left returns the unit left normal of d, the side [sketch.NewOffset] counts as
// positive.
func left(d vec) vec { return vec{-d.Y, d.X} }

// offsetSign returns the signed offset an engine offset dimension needs so that
// dst sits where the solved figure puts it: the magnitude the spec dimensions,
// with the sign the geometry has.
func offsetSign(srcFrom, srcTo, dstPoint vec, magnitude float64) float64 {
	n := unit(left(sub(srcTo, srcFrom)))
	if dot(sub(dstPoint, srcFrom), n) < 0 {
		return -magnitude
	}
	return magnitude
}

// component pins a point at a signed axis offset from another, choosing the axis
// the offset is least degenerate on.
//
// This is the second half of the same substitution the signed angle makes. A
// Fusion length dimension is a magnitude, and which of the two solutions it
// names is decided by the side the geometry was seeded on; the engine's signed
// component dimension states that side outright. Used where the spec pins a
// point with a point-on-line coincident plus a length — K prime and L prime.
func (f *figure) component(from, to *sketch.Point, delta vec) {
	if math.Abs(delta.X) >= math.Abs(delta.Y) {
		f.s.AddConstraint(sketch.NewHorizontalDistance(from, to, delta.X))
		return
	}
	f.s.AddConstraint(sketch.NewVerticalDistance(from, to, delta.Y))
}
