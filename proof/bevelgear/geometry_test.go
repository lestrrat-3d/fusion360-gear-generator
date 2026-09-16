// Package bevelgear_test proves the bevel gear pair's build, one function per
// step of spec/bevelgear/steps.md.
//
// Units. The generated module works in Fusion-internal centimetres; this proof
// works throughout in MILLIMETRES, which is the unit the spec writes every
// formula in, and Module is a raw millimetre number in both. Angles are
// radians inside the proof and degrees in the case tables, because the dialog
// states them in degrees.
//
// This file holds the closed-form bevel geometry every step reads, and the
// case tables the steps run over. It declares no step function of its own.
//
// What this proof cannot reach, recorded once here and again beside the
// lattice assertion in sketches_test.go:
//
//   - The generated module is never executed. Every range check in S03, every
//     rejection message, and the module's own choice of seed for a §2 point is
//     outside this proof. The lattice below seeds at the closed form, so what
//     it proves is that the constraints solve from a CORRECT seed, never that
//     the module's seed is correct. A seed defect therefore reaches Fusion
//     untested, which is how the M/N toe-line seed defect the spec records got
//     there.
//   - The dialog (S01, S02), the occurrence tree (S04) and the cleanup walk
//     (S30) are not geometry and have no substitute either harness accepts.
//   - The Gear Profiles Plane (S06), the per-gear tooth planes (S08), the tooth
//     axes (S10) and the spiral Trace Plane (S15) are frames rather than
//     geometry. The sketch engine is planar and has no second plane to tilt
//     against. What those planes are FOR is built into the figures below: the
//     §2 lattice is laid out in the plane the Gear Profiles Plane defines, and
//     the tooth section is laid on the back-cone plane at its own tilt.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// pt is a point in the Gear Profiles sketch's own 2-D frame, millimetres.
type pt struct{ X, Y float64 }

func (p pt) add(q pt) pt           { return pt{p.X + q.X, p.Y + q.Y} }
func (p pt) sub(q pt) pt           { return pt{p.X - q.X, p.Y - q.Y} }
func (p pt) scale(k float64) pt    { return pt{p.X * k, p.Y * k} }
func (p pt) dot(q pt) float64      { return p.X*q.X + p.Y*q.Y }
func (p pt) cross(q pt) float64    { return p.X*q.Y - p.Y*q.X }
func (p pt) len() float64          { return math.Hypot(p.X, p.Y) }
func (p pt) unit() pt              { l := p.len(); return pt{p.X / l, p.Y / l} }
func (p pt) distance(q pt) float64 { return p.sub(q).len() }

// rotate turns p counter-clockwise by a radians about the origin.
func rotate(p pt, a float64) pt {
	s, c := math.Sin(a), math.Cos(a)
	return pt{p.X*c - p.Y*s, p.X*s + p.Y*c}
}

// distanceToLine is the perpendicular distance from p to the infinite line
// through a along direction d (d need not be a unit vector).
func distanceToLine(p, a, d pt) float64 {
	u := d.unit()
	return math.Abs(p.sub(a).cross(u))
}

// gearSide names which member of the pair a per-gear quantity belongs to. The
// spec builds the pinion first and the driving gear second, and the case
// tables carry the choice as the "gear" parameter.
type gearSide int

const (
	pinion gearSide = iota
	driving
)

func (g gearSide) label() string {
	if g == driving {
		return "Driving"
	}
	return "Pinion"
}

// geometry is the whole of §2's closed form for one parameter case: the
// resolved inputs, the derived cone geometry, and every named lattice point at
// the position §2 seeds it at. Every step reads it; nothing else derives the
// figure a second time.
type geometry struct {
	Module     float64 // raw millimetres, as the dialog returns it
	Sigma      float64 // Shaft Angle, radians
	Nd, Np     float64 // tooth counts
	DPD, PPD   float64 // pitch diameters, mm
	GammaP     float64 // pinion pitch cone angle, radians
	GammaG     float64 // driving pitch cone angle, radians
	R          float64 // Pitch Cone Distance
	ConeDist   float64 // Cone Distance, the diagonal of the two pitch diameters
	BHd, BHp   float64 // resolved base heights
	Dedendum   float64 // 1.25 * Module
	ApexDed    float64 // |Apex->Ded| = sqrt(R^2 + dedendum^2)
	GammaRootP float64
	GammaRootG float64

	MaxFaceWidth float64
	FaceWidth    float64 // resolved
	RootLen      float64 // resolved |Ded->Toe|
	ToeExtension float64 // percent

	ToeRadiusP, ToeRadiusG         float64 // resolved
	ToeCeilingP, ToeCeilingG       float64
	ToeLimitP, ToeLimitG           float64
	VirtualRadiusP, VirtualRadiusG float64
	VirtualTeethP, VirtualTeethG   float64
	RootSink                       float64
	ToothSpacing                   float64

	BoreEnable         bool
	MaxBoreP, MaxBoreG float64
	BoreP, BoreG       float64 // resolved bore diameters

	// The §2 figure, in creation order. C0 is the projected centre.
	C0, Apex, A, B, Apex2, C, D, E, F, G, H, I, J, K, Kp, M, N, Ap, L, Lp, O, P, Bp pt

	// Unit directions the figure is built from.
	Perp       pt // in-plane perpendicular to the anchor line, grow side
	PinionDir  pt // unit Apex->A
	DrivingDir pt // unit Apex->B
	PitchDir   pt // unit Apex->Apex2
	DedP       pt // unit Apex2->C, the pinion dedendum direction
	DedG       pt // unit Apex2->D, the driving dedendum direction
}

// minBaseHeight and maxBaseHeight are the two closed-form bounds S03 resolves
// every base height against. r is that gear's own pitch radius.
func minBaseHeight(module, gamma float64) float64 {
	return 1.05 * 1.25 * module * math.Sin(gamma)
}

func maxBaseHeight(module, r, gamma float64) float64 {
	return 0.95 * (r - 1.25*module*math.Cos(gamma)) * math.Tan(gamma)
}

// minTeeth is the computed per-gear tooth floor, 5.27 * cos(gamma). The
// constant is 2*(1.05*1.25/0.95 + 1.25) = 5.2632 rounded UP, so the published
// floor stays at or above the exact crossing of the two base-height bounds.
func minTeeth(gamma float64) float64 { return 5.27 * math.Cos(gamma) }

// maxShaftAngleDeg is the Maximum Shaft Angle: the cone-angle singularity
// capped at 150 degrees. The cone-angle half is EXCLUSIVE, the 150 half
// inclusive.
func maxShaftAngleDeg(dpd, ppd float64) float64 {
	lo, hi := math.Min(dpd, ppd), math.Max(dpd, ppd)
	limit := 180 / math.Pi * math.Acos(-lo/hi)
	return math.Min(limit, 150)
}

// newGeometry resolves one case into the full §2 figure. It fails the case
// rather than returning a figure for inputs the spec's own range checks
// reject, so a case table row that breaks a bound is reported as a table
// defect instead of silently proving a gear the module would refuse to build.
func newGeometry(t testing.TB, p map[string]float64) geometry {
	t.Helper()

	var g geometry
	g.Module = p["module"]
	g.Sigma = p["shaftAngle"] * math.Pi / 180
	g.Nd = p["drivingTeeth"]
	g.Np = p["pinionTeeth"]
	g.ToothSpacing = p["toothSpacing"]
	g.ToeExtension = p["toeExtension"]
	g.BoreEnable = p["boreEnable"] != 0

	if g.Module <= 0 {
		t.Fatalf("module must be positive, got %g", g.Module)
	}
	if g.Nd < 3 || g.Np < 3 {
		t.Fatalf("both tooth counts must be at least 3, got driving %g pinion %g", g.Nd, g.Np)
	}

	g.DPD = g.Module * g.Nd
	g.PPD = g.Module * g.Np
	g.Dedendum = 1.25 * g.Module

	shaftDeg := p["shaftAngle"]
	maxShaft := maxShaftAngleDeg(g.DPD, g.PPD)
	if shaftDeg < 30 {
		t.Fatalf("shaft angle %g deg is below the documented floor of 30 deg", shaftDeg)
	}
	if shaftDeg > maxShaft || (shaftDeg >= maxShaft && maxShaft < 150) {
		t.Fatalf("shaft angle %g deg is at or above the Maximum Shaft Angle %g deg", shaftDeg, maxShaft)
	}

	// The closed form §2 states. sin(gamma_g)/sin(gamma_p) = DPD/PPD follows
	// from it, which is what makes the two PPD/2 and DPD/2 drops meet.
	g.GammaP = math.Atan2(math.Sin(g.Sigma)*g.PPD, g.DPD+g.PPD*math.Cos(g.Sigma))
	g.GammaG = g.Sigma - g.GammaP
	g.R = (g.PPD / 2) / math.Sin(g.GammaP)
	g.ConeDist = math.Hypot(g.DPD, g.PPD)
	g.ApexDed = math.Hypot(g.R, g.Dedendum)
	g.GammaRootP = g.GammaP - math.Atan(g.Dedendum/g.R)
	g.GammaRootG = g.GammaG - math.Atan(g.Dedendum/g.R)

	rp, rg := g.PPD/2, g.DPD/2

	// Minimum Teeth, per gear, with that gear's own gamma, on top of teeth >= 3.
	if floor := minTeeth(g.GammaP); g.Np < floor {
		t.Fatalf("pinion tooth count %g is below the computed floor %.3f", g.Np, floor)
	}
	if floor := minTeeth(g.GammaG); g.Nd < floor {
		t.Fatalf("driving tooth count %g is below the computed floor %.3f", g.Nd, floor)
	}

	// Base heights. Driving first, then the pinion scaled off the RESOLVED
	// driving value and passed through the pinion's own bounds.
	g.BHd = resolveBaseHeight(t, "driving", p["drivingBaseHeight"], g.Module*g.Nd/8, g.Module, rg, g.GammaG)
	g.BHp = resolveBaseHeight(t, "pinion", p["pinionBaseHeight"], g.BHd*(g.Np/g.Nd), g.Module, rp, g.GammaP)

	// Frame. The projected centre sits at the origin, the projected anchor
	// line runs along +X, and the grow side is +Y. In Fusion the grow side is
	// chosen by the target-plane normal rather than by the sketch's local +Y
	// ([BEVEL-F-GROW-SIDE]); the proof has no target plane, so it fixes the
	// side and records here that the one-bit normal comparison is not reached.
	g.C0 = pt{0, 0}
	g.Perp = pt{0, 1}

	g.Apex = g.C0.add(g.Perp.scale(g.R*math.Cos(g.GammaG) + g.BHd))
	g.DrivingDir = g.Perp.scale(-1)
	g.B = g.Apex.add(g.DrivingDir.scale(g.R * math.Cos(g.GammaG)))

	// The pinion shaft is the driving direction rotated about the apex by
	// +/- Sigma. Form BOTH candidates and keep the one whose endpoint has the
	// greater X: rotating one fixed sense and flipping only on a negative X
	// keeps the wrong candidate whenever both come out positive.
	sense := 1.0
	plus := g.Apex.add(rotate(g.DrivingDir, g.Sigma).scale(g.R * math.Cos(g.GammaP)))
	minus := g.Apex.add(rotate(g.DrivingDir, -g.Sigma).scale(g.R * math.Cos(g.GammaP)))
	g.A = plus
	if minus.X > plus.X {
		g.A, sense = minus, -1
	}
	g.PinionDir = rotate(g.DrivingDir, sense*g.Sigma)

	// The Pitch Line turns the same way, by gamma_g, so Apex 2 lands in the
	// interior wedge between the two shafts and both perpendicular drops reach
	// it from the correct side.
	g.PitchDir = rotate(g.DrivingDir, sense*g.GammaG)
	g.Apex2 = g.Apex.add(g.PitchDir.scale(g.R))

	// Seed the two dedendum ends by dot product against the shaft axes: the
	// pinion direction is the perpendicular u with u . (unit Apex->A) > 0,
	// which is sin(gamma_p) > 0, and the driving direction is its negation.
	u := pt{-g.PitchDir.Y, g.PitchDir.X}
	if u.dot(g.PinionDir) < 0 {
		u = u.scale(-1)
	}
	g.DedP = u
	g.DedG = u.scale(-1)
	g.C = g.Apex2.add(g.DedP.scale(g.Dedendum))
	g.D = g.Apex2.add(g.DedG.scale(g.Dedendum))

	g.E = g.A.add(g.PinionDir.scale(g.Dedendum * math.Sin(g.GammaP)))
	g.F = g.B.add(g.DrivingDir.scale(g.Dedendum * math.Sin(g.GammaG)))
	g.G = g.A.add(g.PinionDir.scale(g.BHp))
	g.H = g.Apex2.add(g.DedP.scale(g.BHp / math.Sin(g.GammaP)))
	g.I = g.B.add(g.DrivingDir.scale(g.BHd))
	g.J = g.Apex2.add(g.DedG.scale(g.BHd / math.Sin(g.GammaG)))

	// The virtual (back-cone / Tredgold) radii. These are exact and are NEVER
	// rounded: |Apex2 -> K| is r / cos(gamma), which is where the dedendum
	// line meets the shaft axis.
	g.VirtualRadiusP = rp / math.Cos(g.GammaP)
	g.VirtualRadiusG = rg / math.Cos(g.GammaG)
	g.VirtualTeethP = 2 * g.VirtualRadiusP / g.Module
	g.VirtualTeethG = 2 * g.VirtualRadiusG / g.Module
	g.RootSink = 0.05 * 2.25 * g.Module

	g.K = g.Apex2.add(g.DedP.scale(g.VirtualRadiusP))
	g.L = g.Apex2.add(g.DedG.scale(g.VirtualRadiusG))
	g.Kp = g.Apex2.add(g.DedP.scale(g.VirtualRadiusP + g.ToothSpacing))
	g.Lp = g.Apex2.add(g.DedG.scale(g.VirtualRadiusG + g.ToothSpacing))

	// Maximum Face Width, from the SOLVED positions of A, B, C, D, H, J. The
	// closed form it must equal is 0.95 * min(R sin^2 gamma_p, R sin^2 gamma_g),
	// which the sketch step asserts separately against this measurement.
	dp := distanceToLine(g.A, g.C, g.H.sub(g.C))
	dg := distanceToLine(g.B, g.D, g.J.sub(g.D))
	g.MaxFaceWidth = 0.95 * math.Min(dp, dg)

	if fw := p["faceWidth"]; fw > 0 {
		if fw > g.MaxFaceWidth {
			t.Fatalf("face width %g mm exceeds the Maximum Face Width %.4f mm", fw, g.MaxFaceWidth)
		}
		g.FaceWidth = fw
	} else {
		g.FaceWidth = math.Min(g.ConeDist/6, g.MaxFaceWidth)
	}

	rootLen0 := g.FaceWidth * g.ApexDed / g.R

	// Toe radii. 0 means auto: that gear's own inner toe corner radius at Toe
	// Extension 0, which is what makes Toe Extension 0 reproduce the profile
	// the gear had before the input existed.
	g.ToeCeilingP = (rp - g.Dedendum*math.Cos(g.GammaP)) * (1 - g.FaceWidth/g.R)
	g.ToeCeilingG = (rg - g.Dedendum*math.Cos(g.GammaG)) * (1 - g.FaceWidth/g.R)
	g.ToeRadiusP = resolveToeRadius(t, "pinion", p["pinionToeRadius"], rp-g.FaceWidth/math.Sin(g.GammaP), g.ToeCeilingP)
	g.ToeRadiusG = resolveToeRadius(t, "driving", p["drivingToeRadius"], rg-g.FaceWidth/math.Sin(g.GammaG), g.ToeCeilingG)

	g.ToeLimitP = g.ApexDed - g.ToeRadiusP/math.Sin(g.GammaRootP)
	g.ToeLimitG = g.ApexDed - g.ToeRadiusG/math.Sin(g.GammaRootG)
	limit := math.Min(g.ToeLimitP, g.ToeLimitG)

	if g.ToeExtension < 0 || g.ToeExtension > 100 {
		t.Fatalf("toe extension %g is outside [0, 100]", g.ToeExtension)
	}
	if g.ToeExtension > 0 && limit <= rootLen0 {
		t.Fatalf("toe extension %g rejected: the smaller Toe Limit %.4f mm is already at or below "+
			"the Toe Extension 0 root length %.4f mm; the Toe Radius must come below its ceiling "+
			"(pinion %.4f mm, driving %.4f mm)", g.ToeExtension, limit, rootLen0, g.ToeCeilingP, g.ToeCeilingG)
	}
	g.RootLen = rootLen0 + (g.ToeExtension/100)*0.99*(limit-rootLen0)

	// The toe lattice. M rides the root axis; N slides in from M along the
	// C->H direction until it reaches the Toe Radius. The slide length is
	// (M's perpendicular distance from the shaft axis - Toe Radius) / cos gamma,
	// and it is the whole of what keeps N on the correct side of that axis.
	g.M = g.Apex.add(g.C.sub(g.Apex).scale(1 - g.RootLen/g.C.distance(g.Apex)))
	mRadius := distanceToLine(g.M, g.Apex, g.PinionDir)
	g.N = g.M.add(g.DedP.scale((mRadius - g.ToeRadiusP) / math.Cos(g.GammaP)))
	g.Ap = g.Apex.add(g.PinionDir.scale(g.N.sub(g.Apex).dot(g.PinionDir)))

	g.O = g.Apex.add(g.D.sub(g.Apex).scale(1 - g.RootLen/g.D.distance(g.Apex)))
	oRadius := distanceToLine(g.O, g.Apex, g.DrivingDir)
	g.P = g.O.add(g.DedG.scale((oRadius - g.ToeRadiusG) / math.Cos(g.GammaG)))
	g.Bp = g.Apex.add(g.DrivingDir.scale(g.P.sub(g.Apex).dot(g.DrivingDir)))

	// Maximum Bore Diameter, per gear. Its heel term is closed-form from the
	// resolved base height; its toe term needs the Root Length, which is why
	// the whole bound resolves here and not in the input-reading pass.
	g.MaxBoreP = maxBore(rp, g.BHp, g.GammaP, g.ApexDed, g.RootLen, g.GammaRootP)
	g.MaxBoreG = maxBore(rg, g.BHd, g.GammaG, g.ApexDed, g.RootLen, g.GammaRootG)
	if g.BoreEnable {
		g.BoreP = resolveBore(t, "pinion", p["pinionBore"], g.PPD/4, g.MaxBoreP)
		g.BoreG = resolveBore(t, "driving", p["drivingBore"], g.DPD/4, g.MaxBoreG)
	}

	return g
}

func resolveBaseHeight(t testing.TB, who string, user, fallback, module, r, gamma float64) float64 {
	t.Helper()
	lo, hi := minBaseHeight(module, gamma), maxBaseHeight(module, r, gamma)
	if lo > hi {
		t.Fatalf("%s base-height window is empty: minimum %.4f mm above maximum %.4f mm", who, lo, hi)
	}
	if user > 0 {
		if user < lo {
			t.Fatalf("%s base height %g mm is below the minimum %.4f mm", who, user, lo)
		}
		if user > hi {
			t.Fatalf("%s base height %g mm is above the maximum %.4f mm", who, user, hi)
		}
		return user
	}
	return math.Max(lo, math.Min(fallback, hi))
}

func resolveToeRadius(t testing.TB, who string, user, auto, ceiling float64) float64 {
	t.Helper()
	if user > 0 {
		if user >= ceiling {
			t.Fatalf("%s toe radius %g mm must be strictly below its ceiling %.4f mm", who, user, ceiling)
		}
		return user
	}
	return auto
}

func maxBore(r, baseHeight, gamma, apexDed, rootLen, gammaRoot float64) float64 {
	rHeel := r - baseHeight/math.Tan(gamma)
	rToe := (apexDed - rootLen) * math.Sin(gammaRoot)
	return 2 * 0.95 * math.Min(rHeel, rToe)
}

func resolveBore(t testing.TB, who string, user, auto, max float64) float64 {
	t.Helper()
	if user > 0 {
		if user > max {
			t.Fatalf("%s bore diameter %g mm exceeds the Maximum Bore Diameter %.4f mm", who, user, max)
		}
		return user
	}
	return math.Min(auto, max)
}

// side collects the per-gear halves of the figure so a step can read one
// member without repeating the pinion/driving branch at every line.
type side struct {
	which         gearSide
	teeth         float64
	gamma         float64
	gammaRoot     float64
	pitchRadius   float64
	axisDir       pt // unit Apex->A or Apex->B
	dedDir        pt // unit Apex2->C or Apex2->D
	toeRadius     float64
	bore          float64
	maxBore       float64
	baseHeight    float64
	virtualRadius float64
	virtualTeeth  float64
	// hexagon vertices in the Fusion draw order A'->G->H->C->M->N.
	hex      [6]pt
	toeEdge  [2]pt // M, N  / O, P
	heelEdge [2]pt // C, H  / D, J
}

func (g geometry) side(which gearSide) side {
	if which == driving {
		return side{
			which: driving, teeth: g.Nd, gamma: g.GammaG, gammaRoot: g.GammaRootG,
			pitchRadius: g.DPD / 2, axisDir: g.DrivingDir, dedDir: g.DedG,
			toeRadius: g.ToeRadiusG, bore: g.BoreG, maxBore: g.MaxBoreG, baseHeight: g.BHd,
			virtualRadius: g.VirtualRadiusG, virtualTeeth: g.VirtualTeethG,
			hex:      [6]pt{g.Bp, g.I, g.J, g.D, g.O, g.P},
			toeEdge:  [2]pt{g.O, g.P},
			heelEdge: [2]pt{g.D, g.J},
		}
	}
	return side{
		which: pinion, teeth: g.Np, gamma: g.GammaP, gammaRoot: g.GammaRootP,
		pitchRadius: g.PPD / 2, axisDir: g.PinionDir, dedDir: g.DedP,
		toeRadius: g.ToeRadiusP, bore: g.BoreP, maxBore: g.MaxBoreP, baseHeight: g.BHp,
		virtualRadius: g.VirtualRadiusP, virtualTeeth: g.VirtualTeethP,
		hex:      [6]pt{g.Ap, g.G, g.H, g.C, g.M, g.N},
		toeEdge:  [2]pt{g.M, g.N},
		heelEdge: [2]pt{g.C, g.H},
	}
}

// station is a point's distance from the apex measured ALONG this gear's
// shaft axis, and radius is its perpendicular distance from that axis. Every
// solid step works in this (station, radius) frame, because the bodies are
// surfaces of revolution about the shaft axis.
func (s side) station(g geometry, p pt) float64 { return p.sub(g.Apex).dot(s.axisDir) }
func (s side) radius(g geometry, p pt) float64 {
	d := p.sub(g.Apex)
	return math.Abs(d.cross(s.axisDir))
}

// distAlong is the cone distance of a point: its distance from the apex
// measured along the ROOT cone element Apex->C / Apex->D. It is what the
// spiral build keys every slab on.
func (s side) distAlong(g geometry, p pt) float64 {
	cone := s.heelEdge[0].sub(g.Apex).unit()
	return p.sub(g.Apex).dot(cone)
}

// hexProfile returns the frustum profile in the (station, radius) half-plane,
// in the Fusion draw order. Revolving it about the shaft axis is the gear body.
func (s side) hexProfile(g geometry) [6]pt {
	var out [6]pt
	for i, v := range s.hex {
		out[i] = pt{s.station(g, v), s.radius(g, v)}
	}
	return out
}

// pappusVolume is the exact volume the hexProfile sweeps when it is revolved a
// full turn about the station axis: pi * sum over edges of the linear radius
// squared integrated along the station. The walk's sign is kept, so an
// inverted figure (toe outside heel) comes back negative rather than right.
func pappusVolume(profile [6]pt) float64 {
	total := 0.0
	for i := range profile {
		a, b := profile[i], profile[(i+1)%len(profile)]
		total += (b.X - a.X) * (a.Y*a.Y + a.Y*b.Y + b.Y*b.Y) / 3
	}
	return math.Pi * total
}

// polygonAreaFactor is the exact ratio between a regular n-gon of circumradius
// r and the circle of that radius. The proof builds every surface of
// revolution as an n-gon sweep, because decad has no revolve the gate accepts
// for these bodies, so every volume it reads is the true one times this
// factor. It is exact, not an approximation, which is what keeps the readings
// meaningful.
func polygonAreaFactor(n int) float64 {
	return float64(n) * math.Sin(2*math.Pi/float64(n)) / (2 * math.Pi)
}

// ringArea is the area of the regular n-gon of circumradius r.
func ringArea(r float64, n int) float64 {
	return 0.5 * float64(n) * r * r * math.Sin(2*math.Pi/float64(n))
}

// frustumVolume is the exact volume of a loft between two parallel regular
// n-gons of circumradius r0 and r1 separated by h: the prismatoid formula
// h/3 * (A0 + A1 + sqrt(A0*A1)), which is what decad's loft records.
func frustumVolume(r0, r1, h float64, n int) float64 {
	a0, a1 := ringArea(r0, n), ringArea(r1, n)
	return h / 3 * (a0 + a1 + math.Sqrt(a0*a1))
}

// bevelParams fills the dialog's own defaults and applies the overrides a case
// names. Shaft and spiral angles are stated in DEGREES here, as the dialog
// states them, and converted at the geometry boundary.
func bevelParams(overrides map[string]float64) map[string]float64 {
	p := map[string]float64{
		"module":            1,
		"shaftAngle":        90,
		"drivingTeeth":      31,
		"pinionTeeth":       31,
		"drivingBaseHeight": 0,
		"pinionBaseHeight":  0,
		"boreEnable":        1,
		"drivingBore":       0,
		"pinionBore":        0,
		"faceWidth":         0,
		"toothSpacing":      0,
		"spiralAngle":       0,
		"hand":              1, // +1 Right, -1 Left; read only when spiralAngle > 0
		"cutterRadius":      0,
		"toeExtension":      0,
		"drivingToeRadius":  0,
		"pinionToeRadius":   0,
		"gear":              0, // 0 Pinion, 1 Driving
		"declaredRefusal":   0,
	}
	for k, v := range overrides {
		if _, ok := p[k]; !ok {
			panic("bevelgear proof: unknown case parameter " + k)
		}
		p[k] = v
	}
	return p
}

func gearOf(p map[string]float64) gearSide {
	if p["gear"] != 0 {
		return driving
	}
	return pinion
}

// declaredRefusal reports a configuration the SPEC admits and this particular
// §2 lattice cannot reach. The spec's rule is that such a case stays in the
// table and is marked, rather than the advertised range being narrowed on one
// net's evidence: a conditioning refusal is a fact about the constraint net,
// not about the geometry, and three independently written lattices do not even
// agree on which end of the Shaft Angle range is reachable.
func declaredRefusal(p map[string]float64) bool { return p["declaredRefusal"] != 0 }

// ---------------------------------------------------------------------------
// Case tables.
//
// The sketch tables run at the dialog's own default Module, because a sketch
// carries no mesh bound. The solid tables run at Module 4 to 8: decad's mesh
// bound has an absolute floor, so a figure small enough brings every
// measurement inside it and the gate reports Suspect on geometry that is in
// fact correct. Module is a pure scale on this figure, so a case at Module 4
// proves the same shape as one at Module 1 and clears the floor.
// ---------------------------------------------------------------------------

// anchorCases covers the §1 reference line. Its geometry does not depend on
// any gear parameter — the line is 10 mm long whatever the gear is — so the
// table varies only what could plausibly move it.
var anchorCases = sketchTable([]proofCase{
	{"default", bevelParams(nil)},
	{"module_8", bevelParams(map[string]float64{"module": 8})},
})

// gearProfileCases is the §2 regime: both ends of the Shaft Angle range, both
// directions of the gear ratio, the two virtual-tooth-count cases, the lowest
// admissible tooth count, both ends of Toe Extension, a positive Tooth
// Spacing, and user-supplied base heights and toe radii.
var gearProfileCases = sketchTable([]proofCase{
	{"default_31_31_at_90", bevelParams(nil)},
	// The documented Shaft Angle floor, and a DECLARED REFUSAL for this net.
	// Measured here: the lattice solves at DOF 0 with nothing conflicting or
	// redundant and every point within the seed tolerance, and the engine then
	// refuses it as near-singular at conditioning 2.8308e-05, against its
	// 4e-05 trust floor. That is one of the two readings the spec records for
	// the three independently written lattices — two refuse 30 degrees at
	// 2.83e-05 and 2.94e-05 and first clear at 35, the third passes 30 and
	// refuses the top of the range instead. The spec's rule is that the case
	// stays in the table and is marked, because a conditioning refusal is a
	// fact about the net rather than about the geometry, and the advertised
	// Shaft Angle range is never narrowed on one net's evidence. The remedy,
	// if one is wanted, is to change how the lattice is built; never to loosen
	// the gate.
	{"shaft_angle_30", bevelParams(map[string]float64{"shaftAngle": 30, "declaredRefusal": 1})},
	{"shaft_angle_35", bevelParams(map[string]float64{"shaftAngle": 35})},
	{"shaft_angle_60", bevelParams(map[string]float64{"shaftAngle": 60})},
	{"shaft_angle_120", bevelParams(map[string]float64{"shaftAngle": 120})},
	{"shaft_angle_142", bevelParams(map[string]float64{"shaftAngle": 142})},
	{"shaft_angle_150", bevelParams(map[string]float64{"shaftAngle": 150})},
	// Both directions of the ratio. The Maximum Face Width binds on whichever
	// gear carries the smaller tooth count, so a table with only one direction
	// proves nothing about the other.
	{"ratio_driving_31_pinion_17", bevelParams(map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17})},
	{"ratio_driving_17_pinion_31", bevelParams(map[string]float64{"drivingTeeth": 17, "pinionTeeth": 31})},
	{"ratio_43_31_at_75", bevelParams(map[string]float64{"drivingTeeth": 43, "pinionTeeth": 31, "shaftAngle": 75})},
	// The two the virtual tooth count needs: 16/12 has one member at an exact
	// count of 15 and the other at 26.667, and 4/4 is the lowest count the
	// computed floor admits and carries the largest root-arc corner float.
	{"virtual_16_12_at_90", bevelParams(map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12})},
	{"virtual_4_4_at_90", bevelParams(map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4})},
	// Toe Extension at both ends of its range, and one in the middle.
	{"toe_extension_50", bevelParams(map[string]float64{"toeExtension": 50})},
	{"toe_extension_100", bevelParams(map[string]float64{"toeExtension": 100})},
	// Tooth Spacing above zero builds K' and L', which exist at no other row.
	{"tooth_spacing_positive", bevelParams(map[string]float64{
		"drivingTeeth": 43, "pinionTeeth": 31, "shaftAngle": 75, "toothSpacing": 0.4})},
	{"tooth_spacing_with_toe_extension", bevelParams(map[string]float64{
		"toothSpacing": 0.5, "toeExtension": 40})},
	// A user value on every input that has a bound, held inside it.
	{"user_base_heights", bevelParams(map[string]float64{
		"drivingBaseHeight": 3.0, "pinionBaseHeight": 3.2})},
	{"user_toe_radii", bevelParams(map[string]float64{
		"drivingToeRadius": 6.0, "pinionToeRadius": 5.0})},
	{"user_face_width", bevelParams(map[string]float64{"faceWidth": 5.0})},
	{"bore_disabled", bevelParams(map[string]float64{"boreEnable": 0})},
	{"module_4", bevelParams(map[string]float64{"module": 4})},
})

// perGearCases runs the per-gear sketch steps over both members of each pair,
// because §3's tooth, S12's hexagon and S26's bore are built once per gear
// with that gear's own gamma and virtual tooth count.
var perGearCases = sketchTable(expandPerGear([]proofCase{
	{"default_31_31_at_90", bevelParams(nil)},
	{"virtual_16_12_at_90", bevelParams(map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12})},
	{"virtual_4_4_at_90", bevelParams(map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4})},
	{"ratio_driving_31_pinion_17", bevelParams(map[string]float64{"drivingTeeth": 31, "pinionTeeth": 17})},
	{"shaft_angle_35", bevelParams(map[string]float64{"shaftAngle": 35})},
	{"shaft_angle_142", bevelParams(map[string]float64{"shaftAngle": 142})},
	{"tooth_spacing_positive", bevelParams(map[string]float64{
		"drivingTeeth": 43, "pinionTeeth": 31, "shaftAngle": 75, "toothSpacing": 0.4})},
	{"toe_extension_100", bevelParams(map[string]float64{"toeExtension": 100})},
}))

// solidCases is the solid regime, at Module 4 to 8 for the mesh-bound reason
// above, over both members of each pair.
var solidCases = solidTable(expandPerGear([]proofCase{
	{"m4_31_31_at_90", bevelParams(map[string]float64{"module": 4})},
	{"m8_31_31_at_90", bevelParams(map[string]float64{"module": 8})},
	{"m4_16_12_at_90", bevelParams(map[string]float64{"module": 4, "drivingTeeth": 16, "pinionTeeth": 12})},
	{"m4_4_4_at_90", bevelParams(map[string]float64{"module": 4, "drivingTeeth": 4, "pinionTeeth": 4})},
	{"m4_31_17_at_90", bevelParams(map[string]float64{"module": 4, "drivingTeeth": 31, "pinionTeeth": 17})},
	{"m4_17_31_at_90", bevelParams(map[string]float64{"module": 4, "drivingTeeth": 17, "pinionTeeth": 31})},
	{"m4_43_31_at_75_spaced", bevelParams(map[string]float64{
		"module": 4, "drivingTeeth": 43, "pinionTeeth": 31, "shaftAngle": 75, "toothSpacing": 0.5})},
	{"m6_31_31_at_35", bevelParams(map[string]float64{"module": 6, "shaftAngle": 35})},
	{"m6_31_31_at_142", bevelParams(map[string]float64{"module": 6, "shaftAngle": 142})},
	{"m4_toe_extension_100", bevelParams(map[string]float64{"module": 4, "toeExtension": 100})},
}))

// traceCases is the §3a trace sketch's own table: the straight branch, which
// builds no trace at all, both hands at the default spiral angle, the top of
// the [0, 60) range, an explicit cutter radius, and a ratio pair, whose two
// members get legitimately different twists from the same cutter and angle.
var traceCases = sketchTable(expandPerGear([]proofCase{
	{"psi_0_straight", bevelParams(map[string]float64{"spiralAngle": 0})},
	{"psi_35_right", bevelParams(map[string]float64{"spiralAngle": 35, "hand": 1})},
	{"psi_35_left", bevelParams(map[string]float64{"spiralAngle": 35, "hand": -1})},
	{"psi_55_right", bevelParams(map[string]float64{"spiralAngle": 55, "hand": 1})},
	{"psi_35_cutter_60", bevelParams(map[string]float64{"spiralAngle": 35, "cutterRadius": 60})},
	{"psi_35_ratio_31_17", bevelParams(map[string]float64{
		"spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17})},
	{"psi_35_at_shaft_angle_60", bevelParams(map[string]float64{
		"spiralAngle": 35, "shaftAngle": 60})},
}))

// spiralCases carries the spiral branch's own regime on top of the solid one:
// psi = 0 is the straight path the hook returns on, both hands at the default
// psi, the top of the [0, 60) range, and a ratio pair, which is the shape that
// gets a legitimately different twist on each member.
var spiralCases = solidTable(expandPerGear([]proofCase{
	{"m4_psi_0_straight", bevelParams(map[string]float64{"module": 4, "spiralAngle": 0})},
	{"m4_psi_35_right", bevelParams(map[string]float64{"module": 4, "spiralAngle": 35, "hand": 1})},
	{"m4_psi_35_left", bevelParams(map[string]float64{"module": 4, "spiralAngle": 35, "hand": -1})},
	{"m4_psi_55_right", bevelParams(map[string]float64{"module": 4, "spiralAngle": 55, "hand": 1})},
	{"m4_psi_35_ratio_31_17", bevelParams(map[string]float64{
		"module": 4, "spiralAngle": 35, "drivingTeeth": 31, "pinionTeeth": 17})},
	{"m4_psi_35_cutter_60", bevelParams(map[string]float64{
		"module": 4, "spiralAngle": 35, "cutterRadius": 60})},
}))

// proofCase is the shape both harnesses' Case types share, so one table can
// feed either. The two conversions below are what the registrations call.
type proofCase struct {
	Name   string
	Params map[string]float64
}

func expandPerGear(in []proofCase) []proofCase {
	out := make([]proofCase, 0, 2*len(in))
	for _, c := range in {
		for _, which := range []gearSide{pinion, driving} {
			p := make(map[string]float64, len(c.Params))
			for k, v := range c.Params {
				p[k] = v
			}
			p["gear"] = float64(which)
			out = append(out, proofCase{c.Name + "_" + which.label(), p})
		}
	}
	return out
}

// sketchTable and solidTable are the two conversions that let one table shape
// feed both harnesses. The registrations name the converted variables, which
// is why the conversion happens here rather than at the call site.
func sketchTable(in []proofCase) []proofkit.Case {
	out := make([]proofkit.Case, len(in))
	for i, c := range in {
		out[i] = proofkit.Case{Name: c.Name, Params: c.Params}
	}
	return out
}

func solidTable(in []proofCase) []proofkit3d.Case {
	out := make([]proofkit3d.Case, len(in))
	for i, c := range in {
		out[i] = proofkit3d.Case{Name: c.Name, Params: c.Params}
	}
	return out
}
