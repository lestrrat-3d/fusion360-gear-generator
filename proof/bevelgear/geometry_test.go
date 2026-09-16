// Package bevelgear_test proves the bevel gear pair's build, one function per
// step of spec/bevelgear/steps.md.
//
// This file holds no step. It holds the closed form every step is measured
// against: the §2 lattice solved on paper, the parameter bounds the dialog
// resolves, and the per-gear anchors §3 and the body steps read. A step builds
// geometry and compares it with what is derived here; nothing here builds
// anything.
//
// Lengths are millimetres throughout, which is what the dialog shows. The
// generated module works in Fusion's internal centimetres and the spec's Units
// note owns that conversion; a proof that repeated it would prove arithmetic
// rather than geometry.
//
// Two conventions the rest of the package rests on:
//
//   - The §2 figure is plane geometry in the Gear Profiles sketch's own frame.
//     `c` is the projected centre, `d` the projected anchor line's unit
//     direction, and `perp` = (-d.Y, d.X) with the sign the target-plane normal
//     chooses ([BEVEL-F-APEX-LOCAL], [BEVEL-F-GROW-SIDE]). The proof carries
//     that sign as a case parameter rather than reading a normal it has no
//     3-D document for.
//   - A solid step works in the one gear's own frame: the shaft axis is world
//     +Z and the apex is the origin, so a §2 point reaches a solid as the pair
//     (station, radius) — its distance from the apex along the shaft axis, and
//     its perpendicular distance from that axis. latticeSide.station and
//     latticeSide.radius are that map, and they are the only place the two
//     frames meet.
package bevelgear_test

import (
	"math"
	"testing"
)

// ---------------------------------------------------------------- plane vectors

// vec is a point or a direction in the Gear Profiles sketch's own 2-D frame.
type vec struct{ X, Y float64 }

func vecAdd(a, b vec) vec           { return vec{a.X + b.X, a.Y + b.Y} }
func vecSub(a, b vec) vec           { return vec{a.X - b.X, a.Y - b.Y} }
func vecScale(a vec, s float64) vec { return vec{a.X * s, a.Y * s} }
func vecDot(a, b vec) float64       { return a.X*b.X + a.Y*b.Y }
func vecCross(a, b vec) float64     { return a.X*b.Y - a.Y*b.X }
func vecLen(a vec) float64          { return math.Hypot(a.X, a.Y) }

func vecUnit(a vec) vec {
	n := vecLen(a)
	return vec{a.X / n, a.Y / n}
}

// turn rotates a by angle radians counter-clockwise.
func vecTurn(a vec, angle float64) vec {
	s, c := math.Sin(angle), math.Cos(angle)
	return vec{a.X*c - a.Y*s, a.X*s + a.Y*c}
}

// perpDistance is the unsigned distance from p to the infinite line through o
// along unit direction dir.
func perpDistance(p, o, dir vec) float64 { return math.Abs(vecCross(dir, vecSub(p, o))) }

// signedOffset is the offset dimension's own reading: the perpendicular distance
// from p to the infinite line through src along dir, positive on the left of
// dir. sketch.NewOffset is signed this way, where Fusion's addOffsetDimension
// takes a magnitude and captures the side from the seeded geometry
// ([PB-DIM-VALUE-SEMANTICS]). The proof therefore computes the side here and
// hands the signed value to the constraint.
func signedOffset(p, src, dir vec) float64 { return vecCross(dir, vecSub(p, src)) }

// footOnLine is the foot of the perpendicular from p onto the line through o
// along unit direction dir.
func footOnLine(p, o, dir vec) vec { return vecAdd(o, vecScale(dir, vecDot(vecSub(p, o), dir))) }

// ---------------------------------------------------------------- dialog values

const (
	// proxyPressureAngle is the spur drawer's own default, served by VirtualSpurProxy.
	// It is not a bevel dialog input.
	proxyPressureAngle = 20 * math.Pi / 180
	// proxyInvoluteSteps is the proxy's InvoluteSteps default.
	proxyInvoluteSteps = 15
	// addendum and dedendum are the standard proportions, in modules.
	addendumModules = 1.0
	dedendumModules = 1.25
	// rootSinkFraction is §3 step 1's root sink, `0.05 * 2.25 * Module`.
	rootSinkFraction = 0.05 * 2.25
	// toeExtensionReach is the 0.99 of §3's "Toe Extension 100 stops at 0.99 of
	// the way". At the limit itself the toe face has zero length and the toe
	// cone the conical end-cut has to find does not exist.
	toeExtensionReach = 0.99
	// minimumTeethFactor is the published floor's constant, rounded UP from
	// 2*(1.05*1.25/0.95 + 1.25) = 5.2632.
	minimumTeethFactor = 5.27
	// faceWidthMargin and baseHeightMargin are the 0.95 and 1.05 the bounds carry.
	faceWidthMargin  = 0.95
	baseHeightMargin = 1.05
	// shaftAngleCeiling is the practical 150-degree cap on the Maximum Shaft Angle.
	shaftAngleCeiling = 150.0
	// crownPerRad is `_CROWN_PER_RAD`, the spiral crown's tunable class constant.
	crownPerRad = 0.5
	// spiralSliceSteps is §3a step E's fixed slice count, and spiralSliceDivisor
	// the `span/6` step it moves by.
	spiralSliceSteps   = 8
	spiralSliceDivisor = 6.0
	// spiralArcOvershoot is §3a step B's 0.06 of span, the hair past the face the
	// trace's ends are taken at.
	spiralArcOvershoot = 0.06
)

// handSign maps the Hand of Spiral dropdown to `handSign` in §3a step B. The two
// list-item strings are `Right` and `Left`; the proof carries the choice as +1
// for Right and -1 for Left, and negates it for the pinion.
func handSign(rightHanded bool, pinion bool) float64 {
	s := 1.0
	if !rightHanded {
		s = -1
	}
	if pinion {
		s = -s
	}
	return s
}

// ---------------------------------------------------------------- the lattice

// latticeSide carries one gear's share of the §2 figure and the values §3 and
// the body steps read off it. The pinion's names are given first and the
// driving gear's in parentheses.
type latticeSide struct {
	label      string
	teeth      float64
	pitchDia   float64
	gamma      float64 // pitch cone half-angle, radians
	baseHeight float64 // resolved, after both bounds
	toeRadius  float64 // resolved
	boreDia    float64 // resolved

	axisDir vec // unit Apex->A (Apex->B)
	axis    vec // A (B)
	drop    vec // the far end of the A->Apex2 (B->Apex2) drop; always Apex 2
	dedDir  vec // unit Apex2->C (Apex2->D)
	ded     vec // C (D)
	rootDir vec // unit Apex->C (Apex->D), the root cone element
	ext     vec // E (F)
	ext2    vec // G (I)
	heel    vec // H (J)
	centre  vec // K (L)
	tooth   vec // K' (L'), the tooth centre after Tooth Spacing
	toe     vec // M (O)
	toeIn   vec // N (P)
	foot    vec // A' (B')
}

// lattice is the whole §2 figure plus every parameter the dialog resolved to
// draw it.
type lattice struct {
	module       float64
	shaftAngle   float64 // radians
	centre       vec     // the projected centre c
	anchorDir    vec     // the projected anchor line's unit direction d
	perp         vec     // the in-plane perpendicular, already carrying the grow sign
	apex         vec
	apex2        vec
	pitchDir     vec // unit Apex->Apex2
	turnSign     float64
	coneDistance float64 // the DIAGONAL of the two pitch diameters
	pitchCone    float64 // R, the Pitch Cone Distance
	apexToDed    float64 // |Apex->Ded| = sqrt(R^2 + (1.25*Module)^2)
	faceWidth    float64 // resolved, after the Maximum Face Width cap
	maxFaceWidth float64
	rootLength   float64 // resolved |Ded->Toe|
	toothSpacing float64
	spiralAngle  float64 // psi, radians
	cutterRadius float64 // resolved r_c is per gear; this is the raw input
	rightHanded  bool
	boreEnable   bool
	pinion       latticeSide
	driving      latticeSide
}

// station is a point's cone distance along this gear's shaft axis: how far it
// sits from the apex measured along Apex->A (Apex->B).
func (s latticeSide) station(apex, p vec) float64 { return vecDot(vecSub(p, apex), s.axisDir) }

// radius is a point's perpendicular distance from this gear's shaft axis, which
// is the radius the revolve sweeps it to.
func (s latticeSide) radius(apex, p vec) float64 { return perpDistance(p, apex, s.axisDir) }

// hexagon is the frustum profile in draw order, A' -> G -> H -> C -> M -> N
// (B' -> I -> J -> D -> O -> P). Its first edge is the shaft axis edge.
func (s latticeSide) hexagon() []vec {
	return []vec{s.foot, s.ext2, s.heel, s.ded, s.toe, s.toeIn}
}

// section is the hexagon as (station, radius) pairs in the gear's own frame,
// which is what every solid step builds from.
func (s latticeSide) section(apex vec) [][2]float64 {
	out := make([][2]float64, 0, 6)
	for _, p := range s.hexagon() {
		out = append(out, [2]float64{s.station(apex, p), s.radius(apex, p)})
	}
	return out
}

// virtualPitchRadius is §3 step 1's exact back-cone radius, `r / cos gamma`. It
// is never rounded, and nothing downstream rebuilds it from a rounded count.
func (s latticeSide) virtualPitchRadius() float64 {
	return (s.pitchDia / 2) / math.Cos(s.gamma)
}

// virtualTeeth is the Tredgold count `2 * virtualPitchRadius / Module`,
// equivalently `teeth / cos gamma`. It is a REAL number.
func (s latticeSide) virtualTeeth(module float64) float64 {
	return 2 * s.virtualPitchRadius() / module
}

// rootConeAngle is `gamma - atan(1.25 * Module / R)`, the dedendum element's
// angle to the shaft axis.
func (s latticeSide) rootConeAngle(module, pitchCone float64) float64 {
	return s.gamma - math.Atan(dedendumModules*module/pitchCone)
}

// toeLimit is |Ded->X|, X being the point on Apex->Ded at this gear's Toe
// Radius. It is where the toe face has closed to nothing.
func (s latticeSide) toeLimit(module, pitchCone, apexToDed float64) float64 {
	return apexToDed - s.toeRadius/math.Sin(s.rootConeAngle(module, pitchCone))
}

// toeRadiusCeiling is this gear's OUTER toe corner radius at Toe Extension 0. A
// user Toe Radius must be strictly below it.
func (s latticeSide) toeRadiusCeiling(module, faceWidth, pitchCone float64) float64 {
	return (s.pitchDia/2 - dedendumModules*module*math.Cos(s.gamma)) * (1 - faceWidth/pitchCone)
}

// ---------------------------------------------------------------- bounds

// maximumShaftAngle is the Maximum Shaft Angle in degrees: the cone-angle limit
// `degrees(acos(-smaller/larger))`, which is exclusive, capped at the inclusive
// practical ceiling of 150 degrees.
func maximumShaftAngle(drivingPitchDia, pinionPitchDia float64) float64 {
	smaller := math.Min(drivingPitchDia, pinionPitchDia)
	larger := math.Max(drivingPitchDia, pinionPitchDia)
	return math.Min(shaftAngleCeiling, math.Acos(-smaller/larger)*180/math.Pi)
}

// minimumTeeth is `5.27 * cos gamma`, the count below which the two base-height
// bounds have crossed and no base height satisfies both.
func minimumTeeth(gamma float64) float64 { return minimumTeethFactor * math.Cos(gamma) }

// maximumBaseHeight is `0.95 * (r - 1.25 * Module * cos gamma) * tan gamma`,
// measured from Apex 2's plane. It is deliberately conservative: it sits
// `1.25 * Module * sin gamma` below the true crossing `r * tan gamma`.
func maximumBaseHeight(pitchRadius, module, gamma float64) float64 {
	return faceWidthMargin * (pitchRadius - dedendumModules*module*math.Cos(gamma)) * math.Tan(gamma)
}

// trueBaseHeightCrossing is `r * tan gamma`, where the heel point H (J) reaches
// the shaft axis. The bound above refuses a band below it that would still build.
func trueBaseHeightCrossing(pitchRadius, gamma float64) float64 {
	return pitchRadius * math.Tan(gamma)
}

// minimumBaseHeight is `1.05 * 1.25 * Module * sin gamma`, the dedendum's own
// along-shaft projection with the margin the other bounds carry.
func minimumBaseHeight(module, gamma float64) float64 {
	return baseHeightMargin * dedendumModules * module * math.Sin(gamma)
}

// resolveBaseHeight applies both bounds the way the spec's input validation
// does: a fallback is raised to the minimum and capped to the maximum, and a
// user value outside either end is a rejection the caller reports.
func resolveBaseHeight(want, module, pitchRadius, gamma float64) float64 {
	lo := minimumBaseHeight(module, gamma)
	hi := maximumBaseHeight(pitchRadius, module, gamma)
	return math.Max(lo, math.Min(want, hi))
}

// ---------------------------------------------------------------- resolution

// newLattice solves the whole §2 figure from one case's dialog values.
//
// The order is the spec's: the closed-form cone angles first, then the base
// heights, then the figure, then the Maximum Face Width — which the spec makes
// a function of the SOLVED points A, B, C, D, H and J, so it cannot be resolved
// until those exist — and only then the toe end.
func newLattice(t testing.TB, p map[string]float64) lattice {
	t.Helper()
	l := lattice{
		module:       p["module"],
		shaftAngle:   p["shaftAngle"],
		centre:       vec{p["centerX"], p["centerY"]},
		toothSpacing: p["toothSpacing"],
		spiralAngle:  p["spiralAngle"],
		cutterRadius: p["cutterRadius"],
		rightHanded:  p["leftHanded"] == 0,
		boreEnable:   p["boreEnable"] != 0,
	}
	drivingTeeth, pinionTeeth := p["drivingTeeth"], p["pinionTeeth"]
	dpd, ppd := l.module*drivingTeeth, l.module*pinionTeeth
	sigma := l.shaftAngle

	// The closed form of §2: tan gamma_p = sin S * PPD / (DPD + PPD cos S),
	// gamma_g = S - gamma_p, R = (PPD/2) / sin gamma_p.
	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	l.pitchCone = (ppd / 2) / math.Sin(gammaP)
	l.coneDistance = math.Hypot(dpd, ppd)
	l.apexToDed = math.Hypot(l.pitchCone, dedendumModules*l.module)

	l.pinion = latticeSide{label: "Pinion", teeth: pinionTeeth, pitchDia: ppd, gamma: gammaP}
	l.driving = latticeSide{label: "Driving", teeth: drivingTeeth, pitchDia: dpd, gamma: gammaG}

	// Base heights. The driving fallback is `Module * Driving Gear Teeth / 8`;
	// the pinion's is the RESOLVED driving height scaled by the tooth ratio, and
	// then the pinion's OWN bounds are applied to it.
	drivingWant := p["drivingBaseHeight"]
	if drivingWant <= 0 {
		drivingWant = l.module * drivingTeeth / 8
	}
	l.driving.baseHeight = resolveBaseHeight(drivingWant, l.module, dpd/2, gammaG)
	pinionWant := p["pinionBaseHeight"]
	if pinionWant <= 0 {
		pinionWant = l.driving.baseHeight * pinionTeeth / drivingTeeth
	}
	l.pinion.baseHeight = resolveBaseHeight(pinionWant, l.module, ppd/2, gammaP)

	// The figure. The anchor line's direction is arbitrary — nothing downstream
	// depends on it — so a case may tilt it, and the grow side is the one bit the
	// target-plane normal decides.
	l.anchorDir = vec{math.Cos(p["anchorAngle"]), math.Sin(p["anchorAngle"])}
	growSign := 1.0
	if p["growSign"] < 0 {
		growSign = -1
	}
	l.perp = vecScale(vec{-l.anchorDir.Y, l.anchorDir.X}, growSign)

	l.apex = vecAdd(l.centre, vecScale(l.perp, l.pitchCone*math.Cos(gammaG)+l.driving.baseHeight))
	drivingDir := vecScale(l.perp, -1)
	l.driving.axisDir = drivingDir
	l.driving.axis = vecAdd(l.apex, vecScale(drivingDir, l.pitchCone*math.Cos(gammaG)))

	// The pinion shaft direction is the driving one rotated about the apex by
	// the Shaft Angle, and the sense is chosen by the candidate whose point A has
	// the greater X in the Gear Profiles sketch.
	plus := vecTurn(drivingDir, sigma)
	minus := vecTurn(drivingDir, -sigma)
	l.turnSign = 1
	pinionDir := plus
	if l.apex.X+minus.X*l.pitchCone*math.Cos(gammaP) > l.apex.X+plus.X*l.pitchCone*math.Cos(gammaP) {
		l.turnSign = -1
		pinionDir = minus
	}
	l.pinion.axisDir = pinionDir
	l.pinion.axis = vecAdd(l.apex, vecScale(pinionDir, l.pitchCone*math.Cos(gammaP)))

	l.pitchDir = vecTurn(drivingDir, l.turnSign*gammaG)
	l.apex2 = vecAdd(l.apex, vecScale(l.pitchDir, l.pitchCone))
	l.pinion.drop, l.driving.drop = l.apex2, l.apex2

	// The two dedendum lines leave Apex 2 perpendicular to the Pitch Line. The
	// driving one points toward the anchor line and the pinion one away from it.
	toward := vec{-l.pitchDir.Y, l.pitchDir.X}
	if vecDot(toward, vecScale(l.perp, -1)) < 0 {
		toward = vecScale(toward, -1)
	}
	l.driving.ded = vecAdd(l.apex2, vecScale(toward, dedendumModules*l.module))
	l.pinion.ded = vecSub(l.apex2, vecScale(toward, dedendumModules*l.module))
	l.driving.dedDir = vecUnit(vecSub(l.driving.ded, l.apex2))
	l.pinion.dedDir = vecUnit(vecSub(l.pinion.ded, l.apex2))
	l.driving.rootDir = vecUnit(vecSub(l.driving.ded, l.apex))
	l.pinion.rootDir = vecUnit(vecSub(l.pinion.ded, l.apex))

	resolveRim(&l.pinion, l.apex)
	resolveRim(&l.driving, l.apex)

	// The Maximum Face Width, from the SOLVED A, B, C, D, H, J ([PB-SOLVED-GEOMETRY]).
	// The pinion is only usually the binding side, so both are measured and the
	// smaller wins.
	l.maxFaceWidth = faceWidthMargin * math.Min(
		perpDistance(l.pinion.axis, l.pinion.ded, l.pinion.dedDir),
		perpDistance(l.driving.axis, l.driving.ded, l.driving.dedDir))
	if want := p["faceWidth"]; want > 0 {
		l.faceWidth = want
	} else {
		l.faceWidth = math.Min(l.coneDistance/6, l.maxFaceWidth)
	}

	// Toe radii. 0 means auto: this gear's inner toe corner radius at Toe
	// Extension 0, which is what makes Toe Extension 0 today's profile exactly.
	l.pinion.toeRadius = p["pinionToeRadius"]
	if l.pinion.toeRadius <= 0 {
		l.pinion.toeRadius = ppd/2 - l.faceWidth/math.Sin(gammaP)
	}
	l.driving.toeRadius = p["drivingToeRadius"]
	if l.driving.toeRadius <= 0 {
		l.driving.toeRadius = dpd/2 - l.faceWidth/math.Sin(gammaG)
	}

	// The Root Length: the face width re-measured along the root element, plus
	// the Toe Extension's share of the window to the smaller of the two Toe
	// Limits, stopping 0.99 of the way there.
	base := l.faceWidth * l.apexToDed / l.pitchCone
	limit := math.Min(
		l.pinion.toeLimit(l.module, l.pitchCone, l.apexToDed),
		l.driving.toeLimit(l.module, l.pitchCone, l.apexToDed))
	l.rootLength = base + (p["toeExtension"]/100)*toeExtensionReach*(limit-base)

	resolveToe(&l.pinion, l.apex, l.rootLength, l.apexToDed)
	resolveToe(&l.driving, l.apex, l.rootLength, l.apexToDed)

	// Bores. 0 means auto: this gear's Pitch Diameter / 4.
	l.pinion.boreDia = p["pinionBore"]
	if l.pinion.boreDia <= 0 {
		l.pinion.boreDia = ppd / 4
	}
	l.driving.boreDia = p["drivingBore"]
	if l.driving.boreDia <= 0 {
		l.driving.boreDia = dpd / 4
	}

	// Tooth Spacing shifts the tooth centre outward along the dedendum line,
	// away from the lower corner C (D). At 0 the centre is K (L) exactly.
	l.pinion.tooth = vecAdd(l.pinion.centre, vecScale(l.pinion.dedDir, l.toothSpacing))
	l.driving.tooth = vecAdd(l.driving.centre, vecScale(l.driving.dedDir, l.toothSpacing))
	return l
}

// resolveRim solves E, F, G, H, I, J and K, L — the part of the figure the base
// height drives.
//
// E (F) is where C (D) drops perpendicularly onto the shaft axis, since C->E is
// constrained perpendicular to A->E and E sits on the axis. G (I) is the point
// on that axis at the base height's offset from the A->Apex2 (B->Apex2) drop,
// and H (J) the point on the dedendum line at the same offset. K (L) is the
// intersection of the shaft axis with the dedendum line, which is where the §3
// tooth is centred.
func resolveRim(s *latticeSide, apex vec) {
	s.ext = footOnLine(s.ded, apex, s.axisDir)

	// The drop runs axis -> Apex 2, and G (I) sits one resolved base height from
	// it, on the side away from the apex.
	dropDir := vecUnit(vecSub(s.drop, s.axis))
	along := vecDot(vecSub(s.ded, s.axis), s.axisDir)
	side := 1.0
	if along < 0 {
		side = -1
	}
	// A point at parameter u along the axis has signed offset
	// vecCross(dropDir, apex + u*axisDir - axis), which is affine in u.
	o0 := vecCross(dropDir, vecSub(apex, s.axis))
	o1 := vecCross(dropDir, s.axisDir)
	want := side * math.Abs(s.baseHeight)
	if o1*side < 0 {
		want = -want
	}
	s.ext2 = vecAdd(apex, vecScale(s.axisDir, (want-o0)/o1))
	offset := vecCross(dropDir, vecSub(s.ext2, s.axis))

	// H (J) carries the same signed offset, on the dedendum line through Apex 2.
	h0 := vecCross(dropDir, vecSub(s.drop, s.axis))
	h1 := vecCross(dropDir, s.dedDir)
	s.heel = vecAdd(s.drop, vecScale(s.dedDir, (offset-h0)/h1))

	// K (L) is the shaft axis crossed with the dedendum line.
	k1 := vecCross(s.dedDir, s.axisDir)
	k0 := vecCross(s.dedDir, vecSub(apex, s.drop))
	s.centre = vecAdd(apex, vecScale(s.axisDir, -k0/k1))
}

// resolveToe solves M, N and A' (O, P and B') from the resolved Root Length and
// Toe Radius.
//
// M (O) sits on the root element Apex->C (Apex->D) at |Ded->Toe| = Root Length
// from the dedendum corner. N (P) then slides from M along the C->H (D->J)
// direction until its perpendicular distance from the shaft axis is the Toe
// Radius, which is what the front face's length dimension holds it at. A' (B')
// is N's foot on the shaft axis.
func resolveToe(s *latticeSide, apex vec, rootLength, apexToDed float64) {
	s.toe = vecAdd(apex, vecScale(s.rootDir, apexToDed-rootLength))
	slide := (s.radius(apex, s.toe) - s.toeRadius) / math.Cos(s.gamma)
	s.toeIn = vecAdd(s.toe, vecScale(s.dedDir, slide))
	s.foot = footOnLine(s.toeIn, apex, s.axisDir)
}

// ---------------------------------------------------------------- readings

// pappusSweepVolume is the volume the §2 hexagon sweeps about the shaft axis:
// 2*pi*A*rbar for a plane region of area A whose centroid sits rbar from the
// axis. It is the closed form the three lofted bands are measured against.
func pappusSweepVolume(section [][2]float64) float64 {
	twiceArea, moment := 0.0, 0.0
	n := len(section)
	for i := range section {
		z1, r1 := section[i][0], section[i][1]
		z2, r2 := section[(i+1)%n][0], section[(i+1)%n][1]
		c := z1*r2 - z2*r1
		twiceArea += c
		moment += (r1 + r2) * c
	}
	return math.Abs(math.Pi * moment / 3)
}

// hexagonArea is the §2 hexagon's own plane area, which the profile sketch's
// single region has to report.
func hexagonArea(section [][2]float64) float64 {
	twiceArea := 0.0
	n := len(section)
	for i := range section {
		z1, r1 := section[i][0], section[i][1]
		z2, r2 := section[(i+1)%n][0], section[(i+1)%n][1]
		twiceArea += z1*r2 - z2*r1
	}
	return math.Abs(twiceArea / 2)
}

// truncatedConeVolume is a truncated cone of height h between radii r1 and r2.
func truncatedConeVolume(r1, r2, h float64) float64 {
	return math.Pi * h * (r1*r1 + r1*r2 + r2*r2) / 3
}

// coneElementAngle is the angle between a cone's surface element and its axis, from
// the two ring radii and the height between them.
func coneElementAngle(r1, r2, h float64) float64 { return math.Atan2(math.Abs(r2-r1), math.Abs(h)) }

// ---------------------------------------------------------------- spiral trace

// spiralTrace is the flat cutter-arc construction of spiral-tooth-trace.md, in
// the tangent plane's own frame: x is cone distance along the root cone element
// and y is circumferential.
type spiralTrace struct {
	rToe, rHeel, rMean, span float64
	cutterRadius             float64
	centre                   vec // C, the cutter-circle centre
	toe, heel                vec // the trace arc's two ends
	rLo, rHi                 float64
	handSign                 float64
	crownTwist               float64 // total, the toe->heel shaft-axis twist magnitude
}

// newSpiralTrace builds the trace for one gear from its toe and heel cone
// distances.
func newSpiralTrace(rToe, rHeel, psi, cutter, sign, gamma float64) spiralTrace {
	s := spiralTrace{rToe: rToe, rHeel: rHeel, handSign: sign}
	s.rMean = (rToe + rHeel) / 2
	s.span = rHeel - rToe
	s.cutterRadius = cutter
	if s.cutterRadius <= 0 {
		s.cutterRadius = s.rMean
	}
	// §3a step B: the hand sign goes on the cos/Cy term, never on sin/Cx.
	s.centre = vec{
		s.rMean - s.cutterRadius*math.Sin(psi),
		sign * s.cutterRadius * math.Cos(psi),
	}
	s.rLo = rToe - spiralArcOvershoot*s.span
	s.rHi = rHeel + spiralArcOvershoot*s.span
	s.toe = circleIntersectNearest(s.rLo, s.centre, s.cutterRadius, vec{s.rMean, 0})
	s.heel = circleIntersectNearest(s.rHi, s.centre, s.cutterRadius, vec{s.rMean, 0})
	phi := math.Atan2(s.heel.Y, s.heel.X) - math.Atan2(s.toe.Y, s.toe.X)
	s.crownTwist = math.Abs(phi) / math.Sin(gamma)
	return s
}

// circleIntersectNearest is solids.circle_intersect_nearest: the apex circle of
// radius r crossed with the cutter circle, keeping the solution nearest ref —
// the branch the mean point sits on. A non-overlapping pair clamps to tangency.
func circleIntersectNearest(r float64, centre vec, cutter float64, ref vec) vec {
	d := vecLen(centre)
	if d == 0 {
		return vec{r, 0}
	}
	a := (d*d + r*r - cutter*cutter) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	base := vecScale(centre, a/d)
	off := vecScale(vec{-centre.Y / d, centre.X / d}, h)
	first, second := vecAdd(base, off), vecSub(base, off)
	if vecLen(vecSub(first, ref)) <= vecLen(vecSub(second, ref)) {
		return first
	}
	return second
}

// segmentTwist is §3a step G's per-segment share: the linear key on the
// segment's HEEL-FACE cone distance, centred on R_mean so the mid-face section
// stays unrotated.
func (s spiralTrace) segmentTwist(heelFace float64) float64 {
	return -s.handSign * s.crownTwist * (s.rMean - heelFace) / s.span
}

// crownFactor is §3a step H's relief: monotonic in the heel distance u, full at
// the heel and largest at the toe.
func (s spiralTrace) crownFactor(heelFace float64) float64 {
	u := (s.rHeel - heelFace) / s.span
	return 1 - crownPerRad*(math.Abs(s.crownTwist)/2)*u
}
