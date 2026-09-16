// Package bevelgear_test proves the bevel bevelSide pair's modelling workflow.
//
// This file holds the closed forms every step shares: the two pitch cone
// angles, the resolved bounds, and the plane-local position each bevelNamed §2 point
// is seeded at. Nothing here reads a built figure; it is the oracle the steps
// assert their own construction against.
//
// # The frame this proof works in
//
// The Gear Profiles sketch is drawn on a plane built perpendicular to the
// user's target plane through the Anchor Line, so inside that sketch the
// direction perpendicular to the projected anchor line IS the target-plane
// normal ([BEVEL-F-APEX-LOCAL]). The bevelBench has no target plane and no
// projection, so this proof lays the projected anchor line along the sketch's
// own +X and takes the grow direction as +Y. Every §2 position below is
// therefore plane-local, exactly as the bevelModuleOf computes it, and no world
// round-trip appears anywhere.
//
// What that costs: the one-bit grow-side decision [BEVEL-F-GROW-SIDE] makes —
// picking perp's sign by the target-plane normal rather than by the sketch's
// local +Y — cannot be reached here, because the bevelBench sketch has no host plane
// whose normal could disagree with its +Y. The proof fixes the sign and proves
// everything downstream of it; the rule that chooses it is checked only in
// Fusion.
//
// # What the whole §2 proof cannot reach
//
// This proof seeds every §2 point at the closed form stated below, which is the
// rule the bevelModuleOf is held to. So it proves that the constraint net solves to
// that figure FROM a correct seed, and never that the bevelModuleOf's own seed is
// correct. A seed defect therefore reaches Fusion untested — which is how the
// toe-line seed defect recorded at stepGearProfiles got there — and the
// [BEVEL-F-SEED-HELD] gate has to live inside the generated bevelModuleOf rather than
// only here.
package bevelgear_test

import (
	"math"
	"testing"
)

// bevelVec is a plane-local point or direction in the Gear Profiles sketch frame,
// in millimetres. The proof works in millimetres throughout; the generated
// bevelModuleOf works in Fusion internal centimetres, which is a pure scale on every
// formula here and changes none of them.
type bevelVec struct{ X, Y float64 }

func bevelAdd(a, b bevelVec) bevelVec         { return bevelVec{a.X + b.X, a.Y + b.Y} }
func bevelSub(a, b bevelVec) bevelVec         { return bevelVec{a.X - b.X, a.Y - b.Y} }
func bevelMul(a bevelVec, k float64) bevelVec { return bevelVec{a.X * k, a.Y * k} }
func bevelDot(a, b bevelVec) float64          { return a.X*b.X + a.Y*b.Y }
func bevelCross(a, b bevelVec) float64        { return a.X*b.Y - a.Y*b.X }
func bevelLen(a bevelVec) float64             { return math.Hypot(a.X, a.Y) }
func bevelUnit(a bevelVec) bevelVec           { return bevelMul(a, 1/bevelLen(a)) }

// bevelLeft is the unit normal 90 degrees counter-clockwise from a, which is the
// side the sketch engine's signed offset constraint calls positive.
func bevelLeft(a bevelVec) bevelVec { u := bevelUnit(a); return bevelVec{-u.Y, u.X} }

// bevelDistPointLine is the perpendicular distance from p to the infinite line
// through base with direction dir.
func bevelDistPointLine(p, base, dir bevelVec) float64 {
	return math.Abs(bevelCross(bevelSub(p, base), bevelUnit(dir)))
}

// bevelSignedAngleDeg is the angle in degrees turned counter-clockwise from
// direction a to direction b, which is the sense the sketch engine's signed
// angle dimension measures from one line's start->end direction to another's.
//
// Every §2 direction this proof pins is stated as this angle between two
// closed-form seeds, so the value written into the constraint is a closed form
// and not a reading of anything the solver produced.
func bevelSignedAngleDeg(a, b bevelVec) float64 {
	return math.Atan2(bevelCross(a, b), bevelDot(a, b)) * 180 / math.Pi
}

// bevelSide carries the half of the pair one per-bevelSide step builds. Both members
// share every input except the tooth count, the pitch cone angle and the two
// per-bevelSide inputs, so a per-bevelSide step reads this rather than branching on a
// label at every line.
type bevelSide struct {
	Label        string  // "Pinion" or "Driving"
	Teeth        float64 // this bevelSide's tooth count
	Gamma        float64 // this bevelSide's pitch cone angle, radians
	PitchRadius  float64 // mm
	BaseHeight   float64 // resolved, mm
	ToeRadius    float64 // resolved, mm
	ToeCeiling   float64 // Toe Radius Ceiling, mm
	ToeLimit     float64 // |Ded->X|, mm
	GammaRoot    float64 // root cone angle, radians
	BoreDiameter float64 // resolved and bounded, mm
	MaxBore      float64 // Maximum Bore Diameter, mm
	MinBaseH     float64 // Minimum Base Height, mm
	MaxBaseH     float64 // Maximum Base Height, mm
	MinTeeth     float64 // computed Minimum Teeth floor, 5.27 * cos gamma
	VirtualTeeth float64 // exact Tredgold count, NEVER rounded
	VirtualPitch float64 // back-cone pitch radius, mm
}

// bevelGeom is one resolved configuration: every derived number and every
// bevelNamed §2 point, in the plane-local frame described in the package comment.
type bevelGeom struct {
	Module       float64
	Sigma        float64 // shaft angle, radians
	MaxSigma     float64 // Maximum Shaft Angle, radians
	PPD, DPD     float64 // pitch diameters, mm
	ConeDistance float64 // the DIAGONAL sqrt(PPD^2 + DPD^2), never R
	R            float64 // Pitch Cone Distance, mm
	Dedendum     float64 // 1.25 * Module, mm
	ApexDed      float64 // |Apex->Ded| = sqrt(R^2 + dedendum^2), mm
	FaceWidth    float64 // resolved, mm
	MaxFaceWidth float64 // from SOLVED-equivalent closed-form positions
	RootLength   float64 // resolved |Ded->Toe|, mm
	RootSink     float64 // 0.05 * 2.25 * Module, mm
	ToothSpacing float64
	ToeExtension float64 // percent
	SpiralAngle  float64 // psi, radians
	HandSign     float64 // +1 Right, -1 Left, on the DRIVING bevelSide
	CutterRadius float64 // resolved r_c for the bevelSide the step names, mm
	BoreEnable   bool

	Pinion, Driving bevelSide

	// The §2 figure, in creation order. Every one of these carries the closed
	// form §2 states for it, which is what [BEVEL-F-SEED-HELD] compares the
	// solve against.
	Apex, B, A, Apex2, C, D, E, F, G, H, I, J bevelVec
	K, Kp, M, N, Ap, L, Lp, O, P, Bp          bevelVec

	// UA, UB are the unit Apex->A and Apex->B shaft directions; UP, UG are the
	// unit Apex2->C and Apex2->D dedendum directions.
	UA, UB, UP, UG bevelVec
	// Perp is the in-plane grow direction, c -> Apex.
	Perp bevelVec
}

const (
	// bevelPressureAngle is not a bevel dialog input: VirtualSpurProxy's own default
	// is what the borrowed spur drawer reads, and bevel never overrides it.
	bevelPressureAngle = 20 * math.Pi / 180
	// bevelInvoluteSteps is likewise the proxy's default.
	bevelInvoluteSteps = 15
	// bevelCrownPerRad is _CROWN_PER_RAD, the tunable class constant the spiral
	// crown scales by. The spec pins its default at 0.5 and says not to leave
	// it unset.
	bevelCrownPerRad = 0.5
	// bevelSlabPlanes is the fixed slice scheme of §3a step E. The count is not
	// user-configurable.
	bevelSlabPlanes = 8
	// bevelToeExtensionReach is the 0.99 factor Toe Extension 100 stops at, so the
	// toe face never closes to nothing and the toe trim keeps a cone face to
	// find.
	bevelToeExtensionReach = 0.99
	// bevelSeedTolerance is [BEVEL-F-SEED-HELD]'s tolerance, 0.001 mm.
	bevelSeedTolerance = 1e-3
)

// bevelMaxShaftAngle is the Maximum Shaft Angle: the cone-angle singularity
// acos(-smaller/larger), which is exclusive, capped at the inclusive practical
// ceiling of 150 degrees. Equal tooth counts give acos(-1) = 180 degrees, which
// is no constraint at all, so the cap is what binds there.
func bevelMaxShaftAngle(ppd, dpd float64) float64 {
	smaller, larger := math.Min(ppd, dpd), math.Max(ppd, dpd)
	return math.Min(math.Acos(-smaller/larger), 150*math.Pi/180)
}

// resolveBevelBaseHeight applies the two closed-form base-height bounds in the
// direction the spec gives them: a fallback below the minimum is raised, a
// fallback above the maximum is capped, and a user value outside either end is
// a rejection the case table never carries.
func resolveBevelBaseHeight(user, fallback, bevelModuleOf, pitchRadius, gamma float64) (resolved, lo, hi float64) {
	lo = 1.05 * 1.25 * bevelModuleOf * math.Sin(gamma)
	hi = 0.95 * (pitchRadius - 1.25*bevelModuleOf*math.Cos(gamma)) * math.Tan(gamma)
	if user > 0 {
		return user, lo, hi
	}
	return math.Min(math.Max(fallback, lo), hi), lo, hi
}

// resolveBevel turns one case's parameters into the whole figure.
//
// The order is the spec's resolution order and is load-bearing: the cone angles
// come first, then the base heights (closed form, resolvable during input
// validation), then the §2 lattice up to H and J, then the Maximum Face Width
// read off those positions, then the Root Length, the toe lattice, and last the
// Maximum Bore Diameter, whose toe term needs the Root Length.
func resolveBevel(p map[string]float64) bevelGeom {
	var g bevelGeom
	g.Module = p["module"]
	zp, zg := p["pinionTeeth"], p["drivingTeeth"]
	g.Sigma = p["shaftAngle"] * math.Pi / 180
	g.PPD, g.DPD = g.Module*zp, g.Module*zg
	g.MaxSigma = bevelMaxShaftAngle(g.PPD, g.DPD)
	g.ConeDistance = math.Hypot(g.PPD, g.DPD)

	gammaP := math.Atan2(math.Sin(g.Sigma)*g.PPD, g.DPD+g.PPD*math.Cos(g.Sigma))
	gammaG := g.Sigma - gammaP
	g.R = (g.PPD / 2) / math.Sin(gammaP)
	g.Dedendum = 1.25 * g.Module
	g.ApexDed = math.Hypot(g.R, g.Dedendum)
	g.RootSink = 0.05 * 2.25 * g.Module
	g.ToothSpacing = p["toothSpacing"]
	g.ToeExtension = p["toeExtension"]
	g.SpiralAngle = p["spiralAngle"] * math.Pi / 180
	g.HandSign = 1
	if p["handLeft"] == 1 {
		g.HandSign = -1
	}
	g.BoreEnable = p["boreEnable"] != 0

	bhG, loG, hiG := resolveBevelBaseHeight(p["drivingBaseHeight"], g.Module*zg/8, g.Module, g.DPD/2, gammaG)
	bhP, loP, hiP := resolveBevelBaseHeight(p["pinionBaseHeight"], bhG*(zp/zg), g.Module, g.PPD/2, gammaP)

	g.Pinion = bevelSide{
		Label: "Pinion", Teeth: zp, Gamma: gammaP, PitchRadius: g.PPD / 2,
		BaseHeight: bhP, MinBaseH: loP, MaxBaseH: hiP,
		MinTeeth:  5.27 * math.Cos(gammaP),
		GammaRoot: gammaP - math.Atan(g.Dedendum/g.R),
	}
	g.Driving = bevelSide{
		Label: "Driving", Teeth: zg, Gamma: gammaG, PitchRadius: g.DPD / 2,
		BaseHeight: bhG, MinBaseH: loG, MaxBaseH: hiG,
		MinTeeth:  5.27 * math.Cos(gammaG),
		GammaRoot: gammaG - math.Atan(g.Dedendum/g.R),
	}
	for _, s := range []*bevelSide{&g.Pinion, &g.Driving} {
		s.VirtualPitch = s.PitchRadius / math.Cos(s.Gamma)
		s.VirtualTeeth = 2 * s.VirtualPitch / g.Module
	}

	// The §2 lattice, in the plane-local frame: the projected centre at the
	// origin, the projected anchor line along +X, and the grow direction +Y.
	g.Perp = bevelVec{0, 1}
	hApex := g.R*math.Cos(gammaG) + bhG
	g.Apex = bevelVec{0, hApex}
	g.UB = bevelVec{0, -1}
	g.B = bevelAdd(g.Apex, bevelMul(g.UB, g.R*math.Cos(gammaG)))
	// The pinion shaft is the driving direction rotated about the apex by the
	// Shaft Angle. Of the two senses, §2 keeps the candidate whose endpoint has
	// the greater X; rotating (0,-1) by +Sigma gives (sin, -cos), whose X is
	// positive for every admitted angle, and by -Sigma gives the mirror.
	g.UA = bevelVec{math.Sin(g.Sigma), -math.Cos(g.Sigma)}
	g.A = bevelAdd(g.Apex, bevelMul(g.UA, g.R*math.Cos(gammaP)))
	pitchDir := bevelVec{math.Sin(gammaG), -math.Cos(gammaG)}
	g.Apex2 = bevelAdd(g.Apex, bevelMul(pitchDir, g.R))
	// The dedendum sides are seeded by dot product against the shaft axes, never
	// by "towards / away from the anchor line": u_p . unit(Apex->A) = sin gamma_p
	// and (-u_p) . unit(Apex->B) = sin gamma_g, both strictly positive.
	g.UP = bevelVec{math.Cos(gammaG), math.Sin(gammaG)}
	g.UG = bevelMul(g.UP, -1)
	g.C = bevelAdd(g.Apex2, bevelMul(g.UP, g.Dedendum))
	g.D = bevelAdd(g.Apex2, bevelMul(g.UG, g.Dedendum))
	g.E = bevelAdd(g.A, bevelMul(g.UA, g.Dedendum*math.Sin(gammaP)))
	g.F = bevelAdd(g.B, bevelMul(g.UB, g.Dedendum*math.Sin(gammaG)))
	g.G = bevelAdd(g.A, bevelMul(g.UA, bhP))
	g.H = bevelAdd(g.Apex2, bevelMul(g.UP, bhP/math.Sin(gammaP)))
	g.I = bevelAdd(g.B, bevelMul(g.UB, bhG))
	g.J = bevelAdd(g.Apex2, bevelMul(g.UG, bhG/math.Sin(gammaG)))
	g.K = bevelAdd(g.Apex2, bevelMul(g.UP, g.Pinion.VirtualPitch))
	g.L = bevelAdd(g.Apex2, bevelMul(g.UG, g.Driving.VirtualPitch))
	g.Kp = bevelAdd(g.Apex2, bevelMul(g.UP, g.Pinion.VirtualPitch+g.ToothSpacing))
	g.Lp = bevelAdd(g.Apex2, bevelMul(g.UG, g.Driving.VirtualPitch+g.ToothSpacing))

	// The Maximum Face Width, read off the positions the constraint net puts A,
	// B, C, D, H and J at rather than off any looser stand-in. Either bevelSide can
	// be the binding side, so both distances are taken and the smaller wins.
	distA := bevelDistPointLine(g.A, g.C, bevelSub(g.H, g.C))
	distB := bevelDistPointLine(g.B, g.D, bevelSub(g.J, g.D))
	g.MaxFaceWidth = 0.95 * math.Min(distA, distB)
	if fw := p["faceWidth"]; fw > 0 {
		g.FaceWidth = fw
	} else {
		g.FaceWidth = math.Min(g.ConeDistance/6, g.MaxFaceWidth)
	}

	// The toe lattice. The Toe Radius default is that bevelSide's inner toe corner
	// radius at Toe Extension 0, which is what makes Toe Extension 0 reproduce
	// the pre-Toe-Radius profile exactly.
	rootLen0 := g.FaceWidth * g.ApexDed / g.R
	for _, s := range []*bevelSide{&g.Pinion, &g.Driving} {
		auto := s.PitchRadius - g.FaceWidth/math.Sin(s.Gamma)
		s.ToeCeiling = (s.PitchRadius - g.Dedendum*math.Cos(s.Gamma)) * (1 - g.FaceWidth/g.R)
		s.ToeRadius = auto
		if s.Label == "Pinion" && p["pinionToeRadius"] > 0 {
			s.ToeRadius = p["pinionToeRadius"]
		}
		if s.Label == "Driving" && p["drivingToeRadius"] > 0 {
			s.ToeRadius = p["drivingToeRadius"]
		}
		s.ToeLimit = g.ApexDed - s.ToeRadius/math.Sin(s.GammaRoot)
	}
	// The pair shares one root length, so the SMALLER of the two Toe Limits
	// wins and the other bevelSide stops short of its own X.
	limit := math.Min(g.Pinion.ToeLimit, g.Driving.ToeLimit)
	g.RootLength = rootLen0 + (g.ToeExtension/100)*bevelToeExtensionReach*(limit-rootLen0)

	g.M = bevelAdd(g.Apex, bevelMul(bevelUnit(bevelSub(g.C, g.Apex)), g.ApexDed-g.RootLength))
	g.N = bevelAdd(g.M, bevelMul(g.UP, (bevelDistPointLine(g.M, g.Apex, g.UA)-g.Pinion.ToeRadius)/math.Cos(gammaP)))
	g.Ap = bevelAdd(g.Apex, bevelMul(g.UA, bevelDot(bevelSub(g.N, g.Apex), g.UA)))
	g.O = bevelAdd(g.Apex, bevelMul(bevelUnit(bevelSub(g.D, g.Apex)), g.ApexDed-g.RootLength))
	g.P = bevelAdd(g.O, bevelMul(g.UG, (bevelDistPointLine(g.O, g.Apex, g.UB)-g.Driving.ToeRadius)/math.Cos(gammaG)))
	g.Bp = bevelAdd(g.Apex, bevelMul(g.UB, bevelDot(bevelSub(g.P, g.Apex), g.UB)))

	// The Maximum Bore Diameter resolves last: its heel term is closed form as
	// soon as the base heights are, but its toe term needs the Root Length, so
	// the whole bound belongs at the step that applies the Maximum Face Width.
	for _, s := range []*bevelSide{&g.Pinion, &g.Driving} {
		rHeel := s.PitchRadius - s.BaseHeight/math.Tan(s.Gamma)
		rToe := (g.ApexDed - g.RootLength) * math.Sin(s.GammaRoot)
		s.MaxBore = 2 * 0.95 * math.Min(rHeel, rToe)
		user := p["pinionBore"]
		pitchDia := g.PPD
		if s.Label == "Driving" {
			user, pitchDia = p["drivingBore"], g.DPD
		}
		if user > 0 {
			s.BoreDiameter = user
		} else {
			s.BoreDiameter = math.Min(pitchDia/4, s.MaxBore)
		}
	}

	rc := p["cutterRadius"]
	g.CutterRadius = rc
	return g
}

// side returns the member of the pair a per-bevelSide case names, and the hand sign
// that member's spiral is built with: the driving gear uses the dialog's hand
// and the meshing pinion is built with the opposite one.
func (g bevelGeom) side(p map[string]float64) (bevelSide, float64) {
	if p["gear"] == 1 {
		return g.Driving, g.HandSign
	}
	return g.Pinion, -g.HandSign
}

// station is a point's distance from the apex measured along the gear's own
// shaft axis, and radius is its perpendicular distance from that axis. Together
// they are the axial half-section every solid step is built in.
func (g bevelGeom) station(s bevelSide, p bevelVec) float64 {
	u := g.UA
	if s.Label == "Driving" {
		u = g.UB
	}
	return bevelDot(bevelSub(p, g.Apex), u)
}

func (g bevelGeom) radius(s bevelSide, p bevelVec) float64 {
	u := g.UA
	if s.Label == "Driving" {
		u = g.UB
	}
	return bevelDistPointLine(p, g.Apex, u)
}

// hexagon is the six profile vertices in the draw order the Profile sketch
// uses: A' -> G -> H -> C -> M -> N on the pinion and B' -> I -> J -> D -> O ->
// P on the driving bevelSide, each as (station, radius) in the axial half-section.
func (g bevelGeom) hexagon(s bevelSide) []bevelVec {
	pts := []bevelVec{g.Ap, g.G, g.H, g.C, g.M, g.N}
	if s.Label == "Driving" {
		pts = []bevelVec{g.Bp, g.I, g.J, g.D, g.O, g.P}
	}
	out := make([]bevelVec, len(pts))
	for i, p := range pts {
		out[i] = bevelVec{g.station(s, p), g.radius(s, p)}
	}
	return out
}

// bevelRevolvedVolume is the volume the hexagon sweeps about the shaft axis, by the
// solid-of-revolution form of the bevelShoelace sum: every edge contributes
// pi/3 * dz * (r0^2 + r0*r1 + r1^2), and the three edges with a nonzero dz are
// exactly the heel bevelBand, the root bevelBand and the toe plug the proof builds.
func bevelRevolvedVolume(hex []bevelVec) float64 {
	total := 0.0
	for i := range hex {
		a, b := hex[i], hex[(i+1)%len(hex)]
		total += math.Pi / 3 * (b.X - a.X) * (a.Y*a.Y + a.Y*b.Y + b.Y*b.Y)
	}
	return math.Abs(total)
}

// bevelRequireClose fails the test unless got is within tol of want. It is used only
// for the proof's own closed-form cross-checks and for sketch readings, never
// for a decad reading: a decad reading carries a proven bound and is compared
// through decadtest so that bound is added to the slack rather than dropped.
func bevelRequireClose(t testing.TB, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %.9g, want %.9g (tolerance %.3g, off by %.3g)", what, got, want, tol, got-want)
	}
}

// bevelToothFlankSamples and bevelToothArcChords set how finely the solid steps chord the
// drawn tooth. They are the resolution of the substitution the solids file
// declares, not a tuning knob: every vertex they produce is a point the drawn
// tooth passes through exactly, and each step asserts its volume against the
// polygon those vertices close rather than against a smooth tooth.
const (
	bevelToothFlankSamples = 8
	bevelToothArcChords    = 6
)

// bevelToothPolygon is the drawn tooth's boundary chorded into a closed polygon, in
// the tooth plane's own 2-D frame with the tooth pointing along +X.
//
// The drawer draws it already rotated 180 degrees, by the angle argument of
// draw(anchorPoint, angle). That rotation is a turn within the tooth plane and
// changes no radius, area or volume any solid step reads, so the polygon here
// is built at angle 0 and every reading taken on it is the drawn tooth's own.
func bevelToothPolygon(c bevelToothCircles, virtualTeeth float64) []bevelVec {
	startR := c.Base
	if c.Embedded {
		startR = c.Root
	}
	left := make([]bevelVec, 0, bevelToothFlankSamples)
	right := make([]bevelVec, 0, bevelToothFlankSamples)
	for i := range bevelToothFlankSamples {
		at := startR + (c.Tip-startR)*float64(i)/float64(bevelToothFlankSamples-1)
		l, r := bevelFlankSample(c.Base, at, c.Pitch, virtualTeeth, 0)
		left = append(left, l)
		right = append(right, r)
	}

	loop := make([]bevelVec, 0, 4*bevelToothFlankSamples)
	rootRight, rootLeft := right[0], left[0]
	if !c.Embedded {
		// The two flank-to-root connecting lines, radial, one per flank.
		rootRight = bevelMul(bevelUnit(right[0]), c.Root)
		rootLeft = bevelMul(bevelUnit(left[0]), c.Root)
		loop = append(loop, rootRight)
	}
	loop = append(loop, right...)
	loop = append(loop, bevelArcChords(c.Tip, math.Atan2(right[len(right)-1].Y, right[len(right)-1].X),
		math.Atan2(left[len(left)-1].Y, left[len(left)-1].X))...)
	for i := len(left) - 1; i >= 0; i-- {
		loop = append(loop, left[i])
	}
	if !c.Embedded {
		loop = append(loop, rootLeft)
	}
	loop = append(loop, bevelArcChords(c.Root, math.Atan2(rootLeft.Y, rootLeft.X),
		math.Atan2(rootRight.Y, rootRight.X))...)
	return loop
}

// bevelArcChords is the interior chord points of an arc of radius r swept from
// angle a to angle b the short way, excluding both ends, which the caller
// already holds.
func bevelArcChords(r, a, b float64) []bevelVec {
	out := make([]bevelVec, 0, bevelToothArcChords)
	for i := 1; i <= bevelToothArcChords; i++ {
		t := a + (b-a)*float64(i)/float64(bevelToothArcChords+1)
		out = append(out, bevelVec{r * math.Cos(t), r * math.Sin(t)})
	}
	return out
}

// bevelToothRootCorner is the outermost point of the drawn tooth's root boundary —
// the root arc's own end, never the tooth's centreline. The centreline sits
// inside both root corners, so a seating reading taken there passes a tooth
// whose corners float outside the bevelSide body's root cone, which is exactly the
// defect the root sink exists to remove.
func bevelToothRootCorner(c bevelToothCircles, virtualTeeth float64) bevelVec {
	loop := bevelToothPolygon(c, virtualTeeth)
	best := loop[0]
	for _, p := range loop {
		if math.Abs(bevelLen(p)-c.Root) > 1e-9 {
			continue
		}
		if math.Abs(p.Y) > math.Abs(best.Y) {
			best = p
		}
	}
	return best
}

// toothStation maps a tooth-plane point to the axial half-section: a point at
// distance x out along the tooth's centreline and y circumferentially sits at
// this station along the shaft axis and this distance from it.
//
// The tooth plane is the BACK-CONE plane, tilted out of the axis-perpendicular
// by this bevelSide's pitch cone angle, so a step along the centreline costs
// sin(gamma) of station and gains cos(gamma) of radius. Taking the tooth plane
// as axis-perpendicular instead is the substitution this proof never makes.
func (g bevelGeom) toothStation(s bevelSide, x float64) float64 {
	return g.toothCentreStation(s) - x*math.Sin(s.Gamma)
}

func (g bevelGeom) toothRadius(s bevelSide, x, y float64) float64 {
	return math.Hypot(x*math.Cos(s.Gamma), y)
}

// toothCentreStation is s_K, the tooth centre's own station: R / cos(gamma)
// from the apex, plus the Tooth Spacing's share along the dedendum line.
func (g bevelGeom) toothCentreStation(s bevelSide) float64 {
	return g.R/math.Cos(s.Gamma) + g.ToothSpacing*math.Sin(s.Gamma)
}
