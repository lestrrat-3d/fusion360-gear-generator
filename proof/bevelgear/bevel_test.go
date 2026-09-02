// Package bevelgear_test proves the bevel gear pair's modelling workflow.
//
// The bevel spec builds one pair from one lattice: a fully constrained axial
// sketch (§2) whose points every later feature reads, a borrowed spur tooth per
// gear (§3), and a body per gear revolved, lofted, trimmed, patterned, joined
// and bored from those points. This file holds what every step needs — the
// parameter cases, and the closed-form lattice the sketch proofs are checked
// against.
//
// The closed forms here are the spec's own, restated in Go, and they are used
// only as the expected answer. Every step builds the real geometry and asserts
// against it; nothing in the proof draws a simplified stand-in and measures that
// instead.
package bevelgear_test

import (
	"math"
	"testing"
)

// vec is a point or direction in the Gear Profiles sketch's own 2-D frame,
// millimetres. The proof works in millimetres throughout: the sketch engine's
// default unit system is metric (mm, deg), and the spec's Module is a raw
// millimetre number, so working in mm keeps the two in the same frame and drops
// the cm conversion that only exists because Fusion's internal unit is the
// centimetre.
type vec struct{ X, Y float64 }

func add(a, b vec) vec           { return vec{a.X + b.X, a.Y + b.Y} }
func sub(a, b vec) vec           { return vec{a.X - b.X, a.Y - b.Y} }
func scale(a vec, k float64) vec { return vec{a.X * k, a.Y * k} }
func dot(a, b vec) float64       { return a.X*b.X + a.Y*b.Y }
func cross(a, b vec) float64     { return a.X*b.Y - a.Y*b.X }
func norm(a vec) float64         { return math.Hypot(a.X, a.Y) }

func unit(a vec) vec {
	n := norm(a)
	if n == 0 {
		return vec{}
	}
	return vec{a.X / n, a.Y / n}
}

// rot turns a by angle radians counter-clockwise.
func rot(a vec, angle float64) vec {
	s, c := math.Sin(angle), math.Cos(angle)
	return vec{a.X*c - a.Y*s, a.X*s + a.Y*c}
}

// params is one case's dialog inputs, in the units the dialog shows: lengths in
// millimetres, angles in degrees, Module the raw millimetre number.
type params struct {
	Module       float64
	DrivingTeeth float64
	PinionTeeth  float64
	ShaftAngle   float64 // degrees, 30..150
	DrivingBase  float64 // mm, 0 = auto
	PinionBase   float64 // mm, 0 = auto
	BoreEnable   bool
	DrivingBore  float64 // mm, 0 = auto
	PinionBore   float64 // mm, 0 = auto
	FaceWidth    float64 // mm, 0 = auto
	ToothSpacing float64 // mm
	SpiralAngle  float64 // degrees, [0, 60)
	HandRight    bool
	CutterRadius float64 // mm, 0 = auto
	CenterX      float64 // where the projected centre lands in the sketch
	CenterY      float64
	Pinion       bool // which gear of the pair this case builds
}

// read turns a proofkit case table's map into the typed inputs. A key the case
// omits takes the dialog default, so a case states only what it varies.
func read(p map[string]float64) params {
	get := func(key string, fallback float64) float64 {
		if v, ok := p[key]; ok {
			return v
		}
		return fallback
	}
	return params{
		Module:       get("module", 1),
		DrivingTeeth: get("drivingTeeth", 31),
		PinionTeeth:  get("pinionTeeth", 31),
		ShaftAngle:   get("shaftAngle", 90),
		DrivingBase:  get("drivingBaseHeight", 0),
		PinionBase:   get("pinionBaseHeight", 0),
		BoreEnable:   get("boreEnable", 1) != 0,
		DrivingBore:  get("drivingBore", 0),
		PinionBore:   get("pinionBore", 0),
		FaceWidth:    get("faceWidth", 0),
		ToothSpacing: get("toothSpacing", 0),
		SpiralAngle:  get("spiralAngle", 0),
		HandRight:    get("hand", 1) != 0,
		CutterRadius: get("cutterRadius", 0),
		CenterX:      get("centerX", 0),
		CenterY:      get("centerY", 0),
		Pinion:       get("pinion", 1) != 0,
	}
}

// cone holds the closed-form cone geometry §2 seeds the lattice from and §3
// reads the virtual tooth numbers out of.
type cone struct {
	DPD, PPD float64 // pitch diameters, mm
	GammaP   float64 // pinion pitch-cone half angle, radians
	GammaG   float64 // driving pitch-cone half angle, radians
	R        float64 // cone distance (PPD/2)/sin(gamma_p), mm
	AlongA   float64 // |Apex->A| = R cos gamma_p
	AlongB   float64 // |Apex->B| = R cos gamma_g
	Sigma    float64 // shaft angle, radians
}

func coneOf(p params) cone {
	sigma := p.ShaftAngle * math.Pi / 180
	dpd := p.Module * p.DrivingTeeth
	ppd := p.Module * p.PinionTeeth
	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	r := (ppd / 2) / math.Sin(gammaP)
	return cone{
		DPD: dpd, PPD: ppd, GammaP: gammaP, GammaG: gammaG, R: r,
		AlongA: r * math.Cos(gammaP), AlongB: r * math.Cos(gammaG), Sigma: sigma,
	}
}

// coneDistanceVariable is the Variables section's Cone Distance,
// sqrt(DPD^2 + PPD^2). It is NOT the cone distance R the §2 seeds use: at a 90
// degree shaft angle it comes out exactly 2R, and away from 90 degrees it is not
// a cone distance at all. Both names appear in the spec; this proof keeps them
// apart, and the report records the collision.
func coneDistanceVariable(p params) float64 {
	return math.Hypot(p.Module*p.DrivingTeeth, p.Module*p.PinionTeeth)
}

// resolvedBaseHeights applies the two fallbacks: the driving offset falls back to
// module * driving teeth / 8, and the pinion offset falls back to the RESOLVED
// driving offset scaled by the tooth ratio — the resolved value, not the raw
// input.
func resolvedBaseHeights(p params) (driving, pinion float64) {
	driving = p.DrivingBase
	if driving == 0 {
		driving = p.Module * p.DrivingTeeth / 8
	}
	pinion = p.PinionBase
	if pinion == 0 {
		pinion = driving * (p.PinionTeeth / p.DrivingTeeth)
	}
	return driving, pinion
}

// virtualTeeth is the back-cone (Tredgold) tooth number for one gear: the
// virtual pitch radius is the real pitch radius divided by cos(gamma), and the
// tooth number is the floor of twice that over the module.
func virtualTeeth(module, pitchDiameter, gamma float64) int {
	return int(math.Floor(2 * ((pitchDiameter / 2) / math.Cos(gamma)) / module))
}

// lattice is every §2 point, solved in closed form in the Gear Profiles sketch's
// own frame. The sketch proof builds the same points from constraints and is
// checked against these.
type lattice struct {
	Centre     vec
	Apex       vec
	A, B       vec
	Apex2      vec
	C, D       vec
	E, F       vec
	G, H       vec
	I, J       vec
	K, L       vec
	KPrime     vec
	LPrime     vec
	M, N       vec
	O, P       vec
	PinionDir  vec // unit Apex->A
	DrivingDir vec // unit Apex->B
	CHat       vec // unit Apex2->C, the pinion dedendum direction
	DHat       vec // unit Apex2->D, the driving dedendum direction
	MaxFace    float64
	FaceWidth  float64
}

// solveLattice places the §2 figure. The anchor line runs along the sketch's
// local X through the projected centre, and the grow side is +Y, so perp is
// (0, 1) — the target-plane normal's side, chosen as one bit exactly as
// the spec's grow-side rule does.
func solveLattice(p params) lattice {
	c := coneOf(p)
	centre := vec{p.CenterX, p.CenterY}
	drivingDir := vec{0, -1} // Apex->B, back toward the anchor line

	// The pinion shaft is the driving shaft rotated about the apex by the shaft
	// angle, in the sense whose point A has the greater sketch X. Both candidates
	// are formed and compared, which is the spec's rule; with the anchor line on
	// X and the apex above it, +sigma always wins, and the proof asserts that
	// rather than assuming it.
	plus := rot(drivingDir, c.Sigma)
	minus := rot(drivingDir, -c.Sigma)
	pinionDir := plus
	if minus.X > plus.X {
		pinionDir = minus
	}

	drivingBase, pinionBase := resolvedBaseHeights(p)

	// Point I closes the figure onto the projected centre, which is what fixes
	// the apex's height above the anchor line. I sits one driving base height
	// beyond B along the driving shaft, so B — and from B the apex — follow.
	b := add(centre, scale(drivingDir, -drivingBase))
	apex := add(b, scale(drivingDir, -c.AlongB))
	a := add(apex, scale(pinionDir, c.AlongA))

	// Apex 2 is the interior-wedge point at pitch radius from each shaft.
	apex2 := apex2Of(a, b, pinionDir, c)

	pitch := unit(sub(apex2, apex))
	dHat := unit(left(pitch))
	if dot(dHat, drivingDir) < 0 {
		dHat = scale(dHat, -1)
	}
	cHat := scale(dHat, -1)

	d := add(apex2, scale(dHat, 1.25*p.Module))
	cc := add(apex2, scale(cHat, 1.25*p.Module))

	// E and F are the feet of the perpendiculars from C and D onto their own
	// shaft axes: the module-length seed is a seed, and the perpendicular
	// constraint is what places them.
	e := add(apex, scale(pinionDir, dot(sub(cc, apex), pinionDir)))
	f := add(apex, scale(drivingDir, dot(sub(d, apex), drivingDir)))

	// G and I sit one base height beyond A and B; H and J are the points of the
	// two dedendum lines at the same station along their shafts.
	g := add(a, scale(pinionDir, pinionBase))
	i := add(b, scale(drivingDir, drivingBase))
	h := alongLineToStation(cc, cHat, apex, pinionDir, dot(sub(g, apex), pinionDir))
	j := alongLineToStation(d, dHat, apex, drivingDir, dot(sub(i, apex), drivingDir))

	// K and L are the intersections of each shaft axis with the other side's
	// dedendum line, which is what the two point-on-line pins locate.
	k := lineIntersect(apex, pinionDir, apex2, cHat)
	l := lineIntersect(apex, drivingDir, apex2, dHat)
	kPrime := add(k, scale(cHat, p.ToothSpacing))
	lPrime := add(l, scale(dHat, p.ToothSpacing))

	// The Maximum Face Width is 0.95 times the smaller of the two perpendicular
	// distances from A to C->H and from B to D->J, read from the solved points.
	maxFace := 0.95 * math.Min(
		math.Abs(cross(sub(a, cc), cHat)),
		math.Abs(cross(sub(b, d), dHat)))
	face := p.FaceWidth
	if face == 0 {
		face = math.Min(coneDistanceVariable(p)/6, maxFace)
	}

	// The toe lines are the two dedendum lines offset toward the apex by the face
	// width, with N pinned onto the A->Apex2 drop and P onto B->Apex2.
	m, n := toeEdge(cc, cHat, apex, a, apex2, face)
	o, pp := toeEdge(d, dHat, apex, b, apex2, face)

	return lattice{
		Centre: centre, Apex: apex, A: a, B: b, Apex2: apex2,
		C: cc, D: d, E: e, F: f, G: g, H: h, I: i, J: j,
		K: k, L: l, KPrime: kPrime, LPrime: lPrime,
		M: m, N: n, O: o, P: pp,
		PinionDir: pinionDir, DrivingDir: drivingDir, CHat: cHat, DHat: dHat,
		MaxFace: maxFace, FaceWidth: face,
	}
}

// apex2Of is the interior-wedge point: pinion pitch radius from the pinion
// shaft, driving pitch radius from the driving shaft, on the side of each shaft
// that faces the other. Solving it in closed form rather than intersecting two
// perpendicular drops is what makes it the answer the proof holds the sketch to.
func apex2Of(a, b, pinionDir vec, c cone) vec {
	dropA := perpToward(pinionDir, sub(b, a))
	return add(a, scale(dropA, c.PPD/2))
}

// perpToward returns the unit perpendicular of dir that points to the same side
// as toward.
func perpToward(dir, toward vec) vec {
	perp := vec{-dir.Y, dir.X}
	if dot(perp, toward) < 0 {
		perp = scale(perp, -1)
	}
	return unit(perp)
}

// alongLineToStation walks from base along dir until the point's projection onto
// axisDir through axisOrigin reaches station.
func alongLineToStation(base, dir, axisOrigin, axisDir vec, station float64) vec {
	have := dot(sub(base, axisOrigin), axisDir)
	step := dot(dir, axisDir)
	return add(base, scale(dir, (station-have)/step))
}

// lineIntersect returns where the line through p0 along d0 meets the line
// through p1 along d1.
func lineIntersect(p0, d0, p1, d1 vec) vec {
	den := cross(d0, d1)
	t := cross(sub(p1, p0), d1) / den
	return add(p0, scale(d0, t))
}

// toeEdge places one gear's toe line: the dedendum line through corner offset
// toward the apex by face, with the far end on the drop from shaft through
// apex2, and the near end on the root axis apex->corner.
func toeEdge(corner, hat, apex, shaft, apex2 vec, face float64) (m, n vec) {
	toward := unit(sub(apex, corner))
	normal := vec{-hat.Y, hat.X}
	if dot(normal, toward) < 0 {
		normal = scale(normal, -1)
	}
	base := add(corner, scale(normal, face))
	m = lineIntersect(base, hat, apex, unit(sub(corner, apex)))
	n = lineIntersect(base, hat, shaft, unit(sub(apex2, shaft)))
	return m, n
}

// near fails the test when got is not within tol of want.
func near(t testing.TB, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %.6f, want %.6f (tolerance %.6f)", what, got, want, tol)
	}
}

// nearPoint fails the test when the two points are further apart than tol.
func nearPoint(t testing.TB, what string, got, want vec, tol float64) {
	t.Helper()
	if norm(sub(got, want)) > tol {
		t.Errorf("%s = (%.6f, %.6f), want (%.6f, %.6f) (tolerance %.6f)",
			what, got.X, got.Y, want.X, want.Y, tol)
	}
}
