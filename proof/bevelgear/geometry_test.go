// Package bevelgear_test proves the bevel gear pair's build, one function per
// step of spec/bevelgear/steps.md.
//
// This file holds the closed form the whole proof measures against: the two
// pitch cone angles, the pitch cone distance, the resolved base heights, face
// width, root length and toe radii, and the solved position of every named
// point of the §2 lattice. Nothing here draws anything. The sketch steps build
// the lattice from constraints and check the solve against these numbers, and
// the solid steps build their sections from them.
//
// Lengths are millimetres throughout, which is the sketch engine's and decad's
// base unit. The generated module works in Fusion's internal centimetres and
// converts, and that conversion is a transcription concern rather than a
// geometric one, so the proof stays in one unit.
package bevelgear_test

import (
	"math"
	"testing"
)

// vec is a point or direction in the Gear Profiles sketch's own 2-D frame.
type vec struct{ X, Y float64 }

func (v vec) add(o vec) vec       { return vec{v.X + o.X, v.Y + o.Y} }
func (v vec) sub(o vec) vec       { return vec{v.X - o.X, v.Y - o.Y} }
func (v vec) scale(k float64) vec { return vec{v.X * k, v.Y * k} }
func (v vec) dot(o vec) float64   { return v.X*o.X + v.Y*o.Y }
func (v vec) cross(o vec) float64 { return v.X*o.Y - v.Y*o.X }
func (v vec) len() float64        { return math.Hypot(v.X, v.Y) }
func (v vec) unit() vec           { n := v.len(); return vec{v.X / n, v.Y / n} }
func (v vec) rot(a float64) vec {
	s, c := math.Sin(a), math.Cos(a)
	return vec{v.X*c - v.Y*s, v.X*s + v.Y*c}
}

// rad turns a degree figure from the dialog into the radians every formula here
// uses. The dialog's deg inputs come back from Fusion in radians already
// ([PB-EVAL-EXPRESSION]); this is the proof's own spelling of that.
func rad(deg float64) float64 { return deg * math.Pi / 180 }

// side holds everything one gear of the pair contributes. The two gears differ
// only in their tooth count and their pitch cone angle, so every per-gear
// quantity is derived the same way from those two.
type side struct {
	label     string
	teeth     float64
	pitchDia  float64 // mm
	gamma     float64 // pitch cone angle, radians
	baseHgt   float64 // resolved base height, mm
	minBase   float64
	maxBase   float64
	minTeeth  float64
	toeRadius float64 // resolved, mm
	toeCeil   float64
	toeLimit  float64
	gammaRoot float64
	boreDia   float64
}

func (s side) pitchRadius() float64 { return s.pitchDia / 2 }

// figure is the whole solved §2 lattice plus the resolved dialog values behind
// it. Every field is the closed form spec/bevelgear/instructions.md states, in
// the order the spec resolves them: cone angles, then the two base heights,
// then the lattice, then the Maximum Face Width the lattice's own solved points
// fix, then Face Width and the toe end.
type figure struct {
	module       float64
	sigma        float64 // shaft angle, radians
	pinion       side
	driving      side
	R            float64 // Pitch Cone Distance, mm
	coneDistance float64 // the diagonal of the two pitch diameters, mm
	distApexDed  float64 // |Apex->Ded| = sqrt(R^2 + (1.25*Module)^2)
	maxShaft     float64 // Maximum Shaft Angle, radians
	maxFaceWidth float64
	faceWidth    float64
	rootLen0     float64 // |Ded->Toe| at Toe Extension 0
	rootLen      float64 // |Ded->Toe| after the Toe Extension
	rootLenPerp  float64 // the same length measured perpendicular to the pitch line
	toothSpacing float64

	// The sketch-local frame the figure is placed in.
	c    vec // projected centre
	d    vec // projected anchor-line direction
	perp vec // the grow direction, chosen by the target-plane normal

	// The §2 named points, solved.
	apex, apex2      vec
	A, B, C, D       vec
	E, F, G, H, I, J vec
	K, KPrime        vec
	L, LPrime        vec
	M, N, APrime     vec
	O, P, BPrime     vec

	// Directions the rest of the proof reads off the lattice.
	up, ug   vec // Apex->A and Apex->B unit directions
	w        vec // Apex->Apex2, the pitch line
	eC, eD   vec // Apex2->C and Apex2->D, the two dedendum directions
	coneP    vec // Apex->C, the pinion root cone element
	coneG    vec // Apex->D, the driving root cone element
	senseSgn float64
}

// maxShaftAngle is the exclusive cone-angle limit capped at an inclusive 150
// degrees. A pitch cone angle reaching 90 degrees turns that gear's cone inside
// out, and cos(Shaft Angle) > -smaller/larger is exactly the condition that
// keeps both below it.
func maxShaftAngle(ppd, dpd float64) float64 {
	smaller, larger := math.Min(ppd, dpd), math.Max(ppd, dpd)
	return math.Min(math.Acos(-smaller/larger), rad(150))
}

// newFigure resolves the dialog values and solves the lattice.
//
// It takes the parameter map a case carries, so the proof's cases are written
// in the dialog's own terms. A zero for a length input means the dialog's
// "unspecified" and takes that input's fallback, exactly as the spec says.
func newFigure(p map[string]float64) figure {
	m := p["module"]
	sigma := rad(p["shaftAngleDeg"])
	zp, zg := p["pinionTeeth"], p["drivingTeeth"]
	ppd, dpd := m*zp, m*zg

	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	R := (ppd / 2) / math.Sin(gammaP)

	f := figure{
		module:       m,
		sigma:        sigma,
		R:            R,
		coneDistance: math.Hypot(ppd, dpd),
		distApexDed:  math.Hypot(R, 1.25*m),
		maxShaft:     maxShaftAngle(ppd, dpd),
		toothSpacing: p["toothSpacing"],
	}
	f.pinion = newSide("Pinion", zp, ppd, gammaP, m, R)
	f.driving = newSide("Driving", zg, dpd, gammaG, m, R)

	// Base heights, driving first: the pinion's fallback is a share of the
	// RESOLVED driving height, so the driving side's own cap has to be applied
	// before the pinion's fallback is formed.
	f.driving.baseHgt = resolveBase(p["drivingBaseHeight"], m*zg/8, f.driving)
	f.pinion.baseHgt = resolveBase(p["pinionBaseHeight"],
		f.driving.baseHgt*(zp/zg), f.pinion)

	f.driving.boreDia = boreOf(p["drivingBore"], dpd)
	f.pinion.boreDia = boreOf(p["pinionBore"], ppd)

	// The sketch-local frame. The anchor direction and the centre are swept by
	// the case table because nothing in the dialog fixes either.
	f.c = vec{p["anchorX"], p["anchorY"]}
	f.d = vec{1, 0}.rot(rad(p["anchorDirDeg"]))
	growSign := 1.0
	if p["growSign"] < 0 {
		growSign = -1
	}
	f.perp = vec{-f.d.Y, f.d.X}.scale(growSign)

	f.solveLattice()

	// The Maximum Face Width reads the SOLVED A, B, C, D, H and J
	// ([PB-SOLVED-GEOMETRY]); it cannot be formed before the lattice above.
	f.maxFaceWidth = 0.95 * math.Min(
		math.Abs(f.A.sub(f.C).cross(f.eC)),
		math.Abs(f.B.sub(f.D).cross(f.eD)))
	if w := p["faceWidth"]; w > 0 {
		f.faceWidth = w
	} else {
		f.faceWidth = math.Min(f.coneDistance/6, f.maxFaceWidth)
	}

	f.rootLen0 = f.faceWidth * f.distApexDed / R
	f.pinion.resolveToe(p["pinionToeRadius"], f.faceWidth, m, R, f.distApexDed)
	f.driving.resolveToe(p["drivingToeRadius"], f.faceWidth, m, R, f.distApexDed)

	limit := math.Min(f.pinion.toeLimit, f.driving.toeLimit)
	f.rootLen = f.rootLen0 + (p["toeExtension"]/100)*0.99*(limit-f.rootLen0)
	f.rootLenPerp = f.rootLen * R / f.distApexDed

	f.solveToeEnd()
	return f
}

func newSide(label string, teeth, pitchDia, gamma, m, R float64) side {
	r := pitchDia / 2
	return side{
		label:    label,
		teeth:    teeth,
		pitchDia: pitchDia,
		gamma:    gamma,
		minBase:  1.05 * 1.25 * m * math.Sin(gamma),
		maxBase:  0.95 * (r - 1.25*m*math.Cos(gamma)) * math.Tan(gamma),
		minTeeth: 5.27 * math.Cos(gamma),
	}
}

// resolveBase applies the gear's own Minimum and Maximum Base Height to its
// input or its fallback. A user value outside either end is a rejection in the
// generated module; the proof's table carries only values the module accepts,
// so clamping here is the same answer for every case in it.
func resolveBase(input, fallback float64, s side) float64 {
	v := input
	if v <= 0 {
		v = fallback
	}
	return math.Min(math.Max(v, s.minBase), s.maxBase)
}

func boreOf(input, pitchDia float64) float64 {
	if input > 0 {
		return input
	}
	return pitchDia / 4
}

// resolveToe fills in the toe radius and the two quantities the Toe Extension
// window is measured with.
func (s *side) resolveToe(input, faceWidth, m, R, distApexDed float64) {
	s.gammaRoot = s.gamma - math.Atan(1.25*m/R)
	s.toeCeil = (s.pitchRadius() - 1.25*m*math.Cos(s.gamma)) * (1 - faceWidth/R)
	if input > 0 {
		s.toeRadius = input
	} else {
		s.toeRadius = s.pitchRadius() - faceWidth/math.Sin(s.gamma)
	}
	s.toeLimit = distApexDed - s.toeRadius/math.Sin(s.gammaRoot)
}

// solveLattice places every §2 point from the closed form, in the order §2
// builds them.
func (f *figure) solveLattice() {
	m := f.module
	R := f.R
	f.apex = f.c.add(f.perp.scale(R*math.Cos(f.driving.gamma) + f.driving.baseHgt))
	f.ug = f.perp.scale(-1)
	f.B = f.apex.add(f.ug.scale(R * math.Cos(f.driving.gamma)))

	// The pinion shaft's sense: form both candidate point-A positions and keep
	// the one whose X is greater in the Gear Profiles sketch.
	plus := f.apex.add(f.ug.rot(f.sigma).scale(R * math.Cos(f.pinion.gamma)))
	minus := f.apex.add(f.ug.rot(-f.sigma).scale(R * math.Cos(f.pinion.gamma)))
	f.senseSgn = 1
	f.A = plus
	if minus.X > plus.X {
		f.senseSgn = -1
		f.A = minus
	}
	f.up = f.ug.rot(f.senseSgn * f.sigma)

	f.w = f.ug.rot(f.senseSgn * f.driving.gamma)
	f.apex2 = f.apex.add(f.w.scale(R))

	// The dedendum pair. D is the one drawn toward the anchor line, so its
	// direction is the one that opposes the grow direction.
	n := vec{-f.w.Y, f.w.X}
	if n.dot(f.perp) > 0 {
		n = n.scale(-1)
	}
	f.eD, f.eC = n, n.scale(-1)
	f.D = f.apex2.add(f.eD.scale(1.25 * m))
	f.C = f.apex2.add(f.eC.scale(1.25 * m))
	f.coneP = f.C.sub(f.apex).unit()
	f.coneG = f.D.sub(f.apex).unit()

	// E and F are the feet of the perpendiculars from C and D onto the two
	// shaft axes; their module-length seeds carry no dimension.
	f.E = f.apex.add(f.up.scale(f.C.sub(f.apex).dot(f.up)))
	f.F = f.apex.add(f.ug.scale(f.D.sub(f.apex).dot(f.ug)))

	f.G = f.apex.add(f.up.scale(R*math.Cos(f.pinion.gamma) + f.pinion.baseHgt))
	f.H = alongTo(f.apex2, f.eC, f.apex, f.up, f.G.sub(f.apex).dot(f.up))
	f.I = f.apex.add(f.ug.scale(R*math.Cos(f.driving.gamma) + f.driving.baseHgt))
	f.J = alongTo(f.apex2, f.eD, f.apex, f.ug, f.I.sub(f.apex).dot(f.ug))

	// K and L are where each shaft axis crosses that gear's dedendum line: the
	// back-cone centres the virtual spur teeth are drawn on.
	f.K = lineCross(f.apex, f.up, f.apex2, f.eC)
	f.L = lineCross(f.apex, f.ug, f.apex2, f.eD)
	f.KPrime = f.K.add(f.eC.scale(f.toothSpacing))
	f.LPrime = f.L.add(f.eD.scale(f.toothSpacing))
}

// solveToeEnd places the toe line and the front face of each gear. It is
// separate from solveLattice because the Face Width the toe line is offset by
// is itself read off the solved lattice.
func (f *figure) solveToeEnd() {
	f.M = f.apex.add(f.coneP.scale(f.distApexDed - f.rootLen))
	f.N = toeCorner(f.M, f.eC, f.apex, f.up, f.pinion.toeRadius)
	f.APrime = f.apex.add(f.up.scale(f.N.sub(f.apex).dot(f.up)))

	f.O = f.apex.add(f.coneG.scale(f.distApexDed - f.rootLen))
	f.P = toeCorner(f.O, f.eD, f.apex, f.ug, f.driving.toeRadius)
	f.BPrime = f.apex.add(f.ug.scale(f.P.sub(f.apex).dot(f.ug)))
}

// alongTo walks the line through base in direction dir until its station along
// axisDir, measured from axisOrigin, reaches station.
func alongTo(base, dir, axisOrigin, axisDir vec, station float64) vec {
	have := base.sub(axisOrigin).dot(axisDir)
	return base.add(dir.scale((station - have) / dir.dot(axisDir)))
}

// lineCross is the intersection of two lines given as point plus direction.
func lineCross(p1, d1, p2, d2 vec) vec {
	t := p2.sub(p1).cross(d2) / d1.cross(d2)
	return p1.add(d1.scale(t))
}

// toeCorner slides from the root point along the dedendum direction until the
// perpendicular distance to the shaft axis has fallen to the toe radius. The
// sign is kept so the corner stays on the side of the axis the root point is
// on: crossing it would put the corner on the axis of revolution, which the
// spec forbids outright.
func toeCorner(root, dedDir, apex, axisDir vec, toeRadius float64) vec {
	g := root.sub(apex).cross(axisDir)
	k := dedDir.cross(axisDir)
	sign := 1.0
	if g < 0 {
		sign = -1
	}
	return root.add(dedDir.scale((sign*toeRadius - g) / k))
}

// hexagon returns one gear's revolve profile in draw order: the front-face
// foot, the two heel vertices, the dedendum corner and the two toe vertices.
func (f figure) hexagon(gear string) []vec {
	if gear == "Driving" {
		return []vec{f.BPrime, f.I, f.J, f.D, f.O, f.P}
	}
	return []vec{f.APrime, f.G, f.H, f.C, f.M, f.N}
}

// axisFrame is the gear's own shaft frame: the apex, the outward shaft
// direction, and the dedendum direction the back cone runs along.
func (f figure) axisFrame(gear string) (apex, axisDir, dedDir, coneDir vec) {
	if gear == "Driving" {
		return f.apex, f.ug, f.eD, f.coneG
	}
	return f.apex, f.up, f.eC, f.coneP
}

func (f figure) sideOf(gear string) side {
	if gear == "Driving" {
		return f.driving
	}
	return f.pinion
}

// station is a point's distance along the gear's shaft axis from the apex, and
// radius its perpendicular distance from that axis. Together they are the
// (z, r) coordinates the revolve turns into a solid.
func (f figure) station(gear string, p vec) float64 {
	apex, axisDir, _, _ := f.axisFrame(gear)
	return p.sub(apex).dot(axisDir)
}

func (f figure) radius(gear string, p vec) float64 {
	apex, axisDir, _, _ := f.axisFrame(gear)
	return math.Abs(p.sub(apex).cross(axisDir))
}

// virtualTeeth is the back-cone (Tredgold) tooth count the borrowed spur tooth
// generator is handed: the virtual pitch radius in modules, floored.
func (f figure) virtualTeeth(gear string) int {
	s := f.sideOf(gear)
	virtualPitchRadius := s.pitchRadius() / math.Cos(s.gamma)
	return int(math.Floor(2 * virtualPitchRadius / f.module))
}

// pappusVolume is the volume the hexagon sweeps about the shaft axis, from the
// polygon's own area and centroid radius. It is what the revolve step's three
// bands have to add up to.
func (f figure) pappusVolume(gear string) float64 {
	pts := f.hexagon(gear)
	var area2, moment float64
	for i := range pts {
		j := (i + 1) % len(pts)
		z0, r0 := f.station(gear, pts[i]), f.radius(gear, pts[i])
		z1, r1 := f.station(gear, pts[j]), f.radius(gear, pts[j])
		cross := z0*r1 - z1*r0
		area2 += cross
		moment += (r0 + r1) * cross
	}
	area := area2 / 2
	centroidR := moment / (6 * area)
	return math.Abs(2 * math.Pi * area * centroidR)
}

// near fails unless got is want to within tol.
func near(t testing.TB, got, want, tol float64, format string, args ...any) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf(format+": got %.9f, want %.9f (tolerance %.3g)",
			append(args, got, want, tol)...)
	}
}
