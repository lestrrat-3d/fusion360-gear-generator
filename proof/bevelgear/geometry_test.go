// Package bevelgear_test proves the bevel pair's build: the four permanent
// sketches, the per-gear solid chain, and the spiral branch's cutter arc and
// twist law.
//
// Everything here is derived from spec/bevelgear/instructions.md, its fusion
// sidecar and spiral-tooth-trace.md. The involute tooth itself is imported from
// proof/involute rather than derived again, because bevel borrows the spur
// tooth generator whole and only feeds it a virtual tooth count.
//
// Lengths are millimetres throughout, which is the unit the spec states its
// geometry in. The generated module works in Fusion internal centimetres; that
// conversion is a transcription concern with no geometry in it, and no harness
// here can see it.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
)

// Parameter keys. The harness case tables are plain float maps, so every value
// a step reads is named here once.
const (
	pModule        = "module"
	pDrivingTeeth  = "drivingTeeth"
	pPinionTeeth   = "pinionTeeth"
	pShaftAngle    = "shaftAngle"        // radians
	pDrivingHeight = "drivingBaseHeight" // 0 means unspecified
	pPinionHeight  = "pinionBaseHeight"  // 0 means unspecified
	pFaceWidth     = "faceWidth"         // 0 means unspecified
	pToothSpacing  = "toothSpacing"
	pDrivingBore   = "drivingBore" // 0 means auto
	pPinionBore    = "pinionBore"
	pBoreEnable    = "boreEnable"   // 1 or 0
	pSpiralAngle   = "spiralAngle"  // radians, 0 means straight
	pCutterRadius  = "cutterRadius" // 0 means auto
	pHandSign      = "handSign"     // +1 Right, -1 Left, driving gear's hand
	pGear          = "gear"         // 0 pinion, 1 driving
)

// deg reads better than radians in a case table.
func deg(d float64) float64 { return d * math.Pi / 180 }

// dedendumFactor is the 1.25 modules every dedendum in this spec is built from.
const dedendumFactor = 1.25

// pair is one bevel pair's resolved parameters: the dialog values plus every
// derived quantity the spec's Variables section defines.
//
// Resolution happens in the order the spec pins under "Reading the raw
// numbers": cone angles first, then the Minimum Teeth floor, then each gear's
// base height against its own two bounds, and only then the Face Width against
// the Maximum Face Width. The order is load-bearing — the Minimum Teeth check
// is exactly the statement that the base-height window is non-empty, so the
// step that resolves a base height never has to describe an empty window.
type pair struct {
	module       float64
	drivingTeeth float64
	pinionTeeth  float64
	shaftAngle   float64
	toothSpacing float64
	spiralAngle  float64
	cutterRadius float64
	handSign     float64
	boreEnable   bool

	// Derived.
	drivingPitchDia float64
	pinionPitchDia  float64
	coneDistance    float64 // the diagonal of the two pitch diameters
	gammaP          float64
	gammaG          float64
	pitchConeDist   float64 // R, apex to heel along the pitch cone

	drivingHeight float64 // resolved
	pinionHeight  float64 // resolved
	faceWidth     float64 // resolved
	drivingBore   float64
	pinionBore    float64
}

// pairOf resolves one case's parameters the way _readInputs does.
func pairOf(t testing.TB, p map[string]float64) pair {
	t.Helper()
	q := pair{
		module:       p[pModule],
		drivingTeeth: p[pDrivingTeeth],
		pinionTeeth:  p[pPinionTeeth],
		shaftAngle:   p[pShaftAngle],
		toothSpacing: p[pToothSpacing],
		spiralAngle:  p[pSpiralAngle],
		cutterRadius: p[pCutterRadius],
		handSign:     p[pHandSign],
		boreEnable:   p[pBoreEnable] != 0,
	}
	if q.handSign == 0 {
		q.handSign = 1
	}
	q.drivingPitchDia = q.module * q.drivingTeeth
	q.pinionPitchDia = q.module * q.pinionTeeth
	q.coneDistance = math.Hypot(q.drivingPitchDia, q.pinionPitchDia)

	// The closed form of §2: the pinion cone angle from the shaft angle and the
	// two pitch diameters, the driving one as the remainder, and the Pitch Cone
	// Distance R from the pinion's own half-pitch-diameter.
	q.gammaP = math.Atan2(math.Sin(q.shaftAngle)*q.pinionPitchDia,
		q.drivingPitchDia+q.pinionPitchDia*math.Cos(q.shaftAngle))
	q.gammaG = q.shaftAngle - q.gammaP
	q.pitchConeDist = (q.pinionPitchDia / 2) / math.Sin(q.gammaP)

	q.drivingHeight = resolveBaseHeight(t, "Driving", p[pDrivingHeight],
		q.module*q.drivingTeeth/8, q.drivingPitchDia/2, q.gammaG, q.module)
	// The pinion fallback is the RESOLVED driving height scaled by the tooth
	// ratio, then held to the pinion's OWN bounds — the two gears share no cone
	// angle once the tooth counts differ.
	q.pinionHeight = resolveBaseHeight(t, "Pinion", p[pPinionHeight],
		q.drivingHeight*q.pinionTeeth/q.drivingTeeth, q.pinionPitchDia/2, q.gammaP, q.module)

	q.faceWidth = p[pFaceWidth]
	if q.faceWidth == 0 {
		q.faceWidth = q.coneDistance / 6
	}
	if max := q.maxFaceWidth(); q.faceWidth > max {
		q.faceWidth = max
	}

	q.drivingBore = p[pDrivingBore]
	if q.drivingBore == 0 {
		q.drivingBore = q.drivingPitchDia / 4
	}
	q.pinionBore = p[pPinionBore]
	if q.pinionBore == 0 {
		q.pinionBore = q.pinionPitchDia / 4
	}
	return q
}

// maxShaftAngle is the cone-angle singularity, capped at 150 degrees.
//
// A pitch cone angle reaching 90 degrees turns that gear's cone inside out:
// R*cos(gamma) passes through zero and changes sign. Both stay below 90 exactly
// while cos(Sigma) > -smaller/larger, so acos of that ratio is a hard limit the
// range check must reject at or above.
func maxShaftAngle(drivingPitchDia, pinionPitchDia float64) float64 {
	smaller := math.Min(drivingPitchDia, pinionPitchDia)
	larger := math.Max(drivingPitchDia, pinionPitchDia)
	return math.Min(math.Acos(-smaller/larger), deg(150))
}

// minTeeth is the count below which the two base-height bounds cross and no
// base height satisfies both.
func minTeeth(gamma float64) float64 {
	return 2 * (1.05*dedendumFactor/0.95 + dedendumFactor) * math.Cos(gamma)
}

// minBaseHeight keeps the heel point H (resp. J) past the dedendum point's own
// along-shaft projection, so the heel edge runs outward rather than back inward.
func minBaseHeight(module, gamma float64) float64 {
	return 1.05 * dedendumFactor * module * math.Sin(gamma)
}

// maxBaseHeight keeps the heel point off the shaft axis.
//
// It is deliberately conservative: it starts from the dedendum point at
// perpendicular distance r - 1.25*module*cos(gamma) rather than from the pitch
// point, so it sits 1.25*module*sin(gamma) below the true crossing at
// r*tan(gamma). The proof asserts both — the bound and the crossing it is meant
// to stay under — in stepGearProfiles.
func maxBaseHeight(pitchRadius, gamma, module float64) float64 {
	return 0.95 * (pitchRadius - dedendumFactor*module*math.Cos(gamma)) * math.Tan(gamma)
}

// trueHeelCrossing is the base height at which the heel point reaches the shaft
// axis and the revolved profile would cross its own axis of revolution.
func trueHeelCrossing(pitchRadius, gamma float64) float64 {
	return pitchRadius * math.Tan(gamma)
}

// resolveBaseHeight applies both bounds the way the spec's step 2 does: raise a
// fallback below the minimum, cap one above the maximum, and reject a user value
// outside either end.
func resolveBaseHeight(t testing.TB, label string, given, fallback, pitchRadius, gamma, module float64) float64 {
	t.Helper()
	low := minBaseHeight(module, gamma)
	high := maxBaseHeight(pitchRadius, gamma, module)
	if low > high {
		t.Fatalf("%s: the base-height window is empty (min %.6f > max %.6f); the Minimum Teeth "+
			"floor is supposed to have refused this case first", label, low, high)
	}
	if given != 0 {
		if given < low || given > high {
			t.Fatalf("%s: a user base height of %.6f is outside [%.6f, %.6f] and is supposed to "+
				"be rejected, not clamped", label, given, low, high)
		}
		return given
	}
	return math.Min(math.Max(fallback, low), high)
}

// maxFaceWidth is 0.95 times the smaller of the two perpendicular distances from
// each gear's shaft-axis point to that gear's own dedendum line.
//
// The closed form is R*sin(gamma)^2 for each gear, which stepGearProfiles checks
// against the distance measured on the solved lattice rather than trusting it
// here. At Shaft Angle 90 it reduces to PPD^2/(2*ConeDistance), which is the
// value the spec names.
func (q pair) maxFaceWidth() float64 {
	pinion := q.pitchConeDist * math.Sin(q.gammaP) * math.Sin(q.gammaP)
	driving := q.pitchConeDist * math.Sin(q.gammaG) * math.Sin(q.gammaG)
	return 0.95 * math.Min(pinion, driving)
}

// gearSide is one member of the pair, with the substitutions the spec's "Create
// the Gear Bodies" table makes.
type gearSide struct {
	label      string
	teeth      float64
	pitchDia   float64
	gamma      float64
	baseHeight float64
	bore       float64
	handSign   float64 // this gear's own hand: the driving gear's, negated for the pinion
}

func (q pair) pinion() gearSide {
	return gearSide{label: "Pinion", teeth: q.pinionTeeth, pitchDia: q.pinionPitchDia,
		gamma: q.gammaP, baseHeight: q.pinionHeight, bore: q.pinionBore, handSign: -q.handSign}
}

func (q pair) driving() gearSide {
	return gearSide{label: "Driving", teeth: q.drivingTeeth, pitchDia: q.drivingPitchDia,
		gamma: q.gammaG, baseHeight: q.drivingHeight, bore: q.drivingBore, handSign: q.handSign}
}

// gearOf picks the side a case names.
func (q pair) gearOf(p map[string]float64) gearSide {
	if p[pGear] == 1 {
		return q.driving()
	}
	return q.pinion()
}

// virtualTeeth is the back-cone (Tredgold) tooth number: the virtual pitch
// radius is this gear's pitch radius divided by cos(gamma), and the count is
// twice that over the module, floored.
func (g gearSide) virtualTeeth(module float64) float64 {
	return math.Floor(2 * (g.pitchDia / 2 / math.Cos(g.gamma)) / module)
}

// vec2 is a point in the Gear Profiles sketch's own plane: x along the projected
// anchor line, y along the in-plane perpendicular toward the apex.
type vec2 struct{ X, Y float64 }

func (a vec2) add(b vec2) vec2      { return vec2{a.X + b.X, a.Y + b.Y} }
func (a vec2) sub(b vec2) vec2      { return vec2{a.X - b.X, a.Y - b.Y} }
func (a vec2) scale(s float64) vec2 { return vec2{a.X * s, a.Y * s} }
func (a vec2) dot(b vec2) float64   { return a.X*b.X + a.Y*b.Y }
func (a vec2) norm() float64        { return math.Hypot(a.X, a.Y) }

// dir returns the unit vector at angle theta measured from the driving shaft
// direction (0, -1), turning toward the pinion side.
//
// The whole §2 figure is written in this one angular measure, which is what
// makes its closed form short: the driving shaft is dir(0), the pitch line
// dir(gammaG), the pinion shaft dir(shaftAngle).
func dir(theta float64) vec2 { return vec2{math.Sin(theta), -math.Cos(theta)} }

// lattice is the §2 figure's solved positions, in the sketch's own 2-D frame
// with the projected centre at the origin and the projected anchor line along
// +X.
//
// These are the closed-form positions the constraint net has to reproduce. The
// sketch step seeds from them and then asserts the solved geometry against them,
// so the seeds are never what is being proved.
type lattice struct {
	center vec2
	apex   vec2
	a, b   vec2 // the two shaft-axis ends
	apex2  vec2
	c, d   vec2 // the two dedendum points
	e, f   vec2
	g, i   vec2
	h, j   vec2
	k, l   vec2 // the two tooth centres before the Tooth Spacing offset
	kp, lp vec2 // K' and L', after it
	m, n   vec2 // the pinion toe edge
	o, p   vec2 // the driving toe edge
}

// build lays out the §2 lattice from the closed form.
//
// Every position here is derived in the frame the spec fixes: the apex sits at
// c + perp*(R*cos(gammaG) + drivingBaseHeight), and every other point follows
// from the cone angles. Point I lands exactly on the projected centre, which is
// the closure the spec states as "Constrain Point I with center point" — the one
// constraint that pins the figure's height above the anchor line.
func (q pair) build() lattice {
	var lat lattice
	dedendum := dedendumFactor * q.module
	R := q.pitchConeDist

	drivingDir := dir(0)
	pinionDir := dir(q.shaftAngle)
	pitchDir := dir(q.gammaG)
	// The two dedendum offsets from the pitch line: toward the anchor line is
	// the driving side, away from it the pinion side.
	towardAnchor := dir(q.gammaG - math.Pi/2)
	awayFromAnchor := dir(q.gammaG + math.Pi/2)

	lat.center = vec2{0, 0}
	lat.apex = vec2{0, R*math.Cos(q.gammaG) + q.drivingHeight}
	lat.b = lat.apex.add(drivingDir.scale(R * math.Cos(q.gammaG)))
	lat.a = lat.apex.add(pinionDir.scale(R * math.Cos(q.gammaP)))
	lat.apex2 = lat.apex.add(pitchDir.scale(R))
	lat.d = lat.apex2.add(towardAnchor.scale(dedendum))
	lat.c = lat.apex2.add(awayFromAnchor.scale(dedendum))

	// E and F are the module-length extensions, undimensioned: each solves to
	// the foot of the perpendicular from its dedendum point onto its own shaft
	// axis, which is 1.25*module*sin(gamma) beyond A resp. B.
	lat.e = lat.a.add(pinionDir.scale(dedendum * math.Sin(q.gammaP)))
	lat.f = lat.b.add(drivingDir.scale(dedendum * math.Sin(q.gammaG)))

	// G and I are driven by the base-height offset dimensions: each sits one
	// base height along its own shaft axis past the perpendicular drop.
	lat.g = lat.apex.add(pinionDir.scale(R*math.Cos(q.gammaP) + q.pinionHeight))
	lat.i = lat.apex.add(drivingDir.scale(R*math.Cos(q.gammaG) + q.drivingHeight))

	// H and J close the heel edges: each is where the dedendum line, extended
	// past its dedendum point, meets the line through G resp. I perpendicular to
	// that gear's shaft axis.
	lat.h = lat.c.add(awayFromAnchor.scale(q.pinionHeight/math.Sin(q.gammaP) - dedendum))
	lat.j = lat.d.add(towardAnchor.scale(q.drivingHeight/math.Sin(q.gammaG) - dedendum))

	// K and L are where each dedendum line crosses its own shaft axis, at cone
	// distance R/cos(gamma) — the back-cone distance. K' and L' shift outward
	// along the dedendum line by the Tooth Spacing.
	lat.k = lat.apex.add(pinionDir.scale(R / math.Cos(q.gammaP)))
	lat.l = lat.apex.add(drivingDir.scale(R / math.Cos(q.gammaG)))
	lat.kp = lat.k.add(awayFromAnchor.scale(q.toothSpacing))
	lat.lp = lat.l.add(towardAnchor.scale(q.toothSpacing))

	// The toe edges are the heel edges offset toward the apex by the Face Width.
	// Sliding along the root cone element by faceWidth*|apex->C|/R moves the
	// perpendicular distance to the heel line by exactly the face width, so M is
	// the dedendum point scaled toward the apex by 1 - faceWidth/R.
	lat.m = lat.apex.add(lat.c.sub(lat.apex).scale(1 - q.faceWidth/R))
	lat.o = lat.apex.add(lat.d.sub(lat.apex).scale(1 - q.faceWidth/R))
	lat.n = slideToDrop(lat.m, awayFromAnchor, lat.a, pinionDir)
	lat.p = slideToDrop(lat.o, towardAnchor, lat.b, drivingDir)
	return lat
}

// slideToDrop slides from the toe's root-axis point along the dedendum direction
// until it meets the perpendicular drop through the shaft-axis point, which is
// where the toe edge's far end is pinned.
func slideToDrop(from, along, axisPoint, axisDir vec2) vec2 {
	travel := axisPoint.sub(from).dot(axisDir) / along.dot(axisDir)
	return from.add(along.scale(travel))
}

// axial converts a lattice point into the per-gear revolve frame: x is the cone
// distance along that gear's shaft axis from the apex, y the perpendicular
// distance from that axis. The hexagon profile is drawn in this frame, with the
// shaft axis as the sketch's own x axis.
func (lat lattice) axial(p vec2, apexDir vec2) vec2 {
	rel := p.sub(lat.apex)
	perp := vec2{-apexDir.Y, apexDir.X}
	x := rel.dot(apexDir)
	y := rel.dot(perp)
	if y < 0 {
		y = -y
	}
	return vec2{x, y}
}

// hexagon returns one gear's frustum profile in its own axial frame, in the draw
// order the spec's table fixes: A, G, H, C, M, N for the pinion and B, I, J, D,
// O, P for the driving gear. The first edge is the shaft axis.
func (q pair) hexagon(lat lattice, side gearSide) []vec2 {
	if side.label == "Driving" {
		axis := dir(0)
		return []vec2{lat.axial(lat.b, axis), lat.axial(lat.i, axis), lat.axial(lat.j, axis),
			lat.axial(lat.d, axis), lat.axial(lat.o, axis), lat.axial(lat.p, axis)}
	}
	axis := dir(q.shaftAngle)
	return []vec2{lat.axial(lat.a, axis), lat.axial(lat.g, axis), lat.axial(lat.h, axis),
		lat.axial(lat.c, axis), lat.axial(lat.m, axis), lat.axial(lat.n, axis)}
}

// toeHeel returns the four world points §3a's caller hand-off passes into the
// tooth-body hook, in the axial frame: the toe edge's midpoint and inner
// endpoint, and the heel edge's midpoint and dedendum corner.
//
// The pairing is the one thing §3a calls the single biggest spiral hazard: toeMid
// and heelMid are the midpoints of two DIFFERENT edges, and heelConeWorld is the
// dedendum corner C/D on the root cone element, never H/J.
func (q pair) toeHeel(lat lattice, side gearSide) (toeMid, heelMid, toeCone, heelCone vec2) {
	hex := q.hexagon(lat, side)
	// hex is A/B, G/I, H/J, C/D, M/O, N/P.
	heelCone, toeCone = hex[3], hex[4]
	heelMid = hex[3].add(hex[2]).scale(0.5)
	toeMid = hex[4].add(hex[5]).scale(0.5)
	return toeMid, heelMid, toeCone, heelCone
}

// solve runs the solver so every reading is taken off solved geometry rather
// than off the seed coordinates ([PB-SOLVED-GEOMETRY]).
func solve(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	res, err := s.Solve(context.Background())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e, DOF %d", res.Residual, res.DOF)
	}
}

// at reads a solved sketch point back as a plane vector.
func at(p *sketch.Point) vec2 { return vec2{p.X(), p.Y()} }

// near fails when two solved positions disagree by more than tol.
func near(t testing.TB, what string, got, want vec2, tol float64) {
	t.Helper()
	if d := got.sub(want).norm(); d > tol {
		t.Errorf("%s solved to (%.9f, %.9f), not the closed form's (%.9f, %.9f); %.3e apart",
			what, got.X, got.Y, want.X, want.Y, d)
	}
}

// closeTo is the relative comparison the solid assertions use.
func closeTo(got, want, relative float64) bool {
	return math.Abs(got-want) <= relative*math.Abs(want)
}
