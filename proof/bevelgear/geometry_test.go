// Package bevelgear_test proves the bevel gear pair's compiled step list.
//
// Everything here works in millimetres. The generated Fusion module works in
// Fusion's internal centimetres and converts every Module-derived length with
// to_cm; that conversion is a property of the module, not of the geometry, and
// no case in this proof can reach it. The step list carries the conversion rule.
//
// What this proof cannot reach, recorded once here and again beside each site:
//
//   - The generated module's SEEDS. Every sketch case below seeds its geometry at
//     the closed form the spec states, so what the solve proves is that the
//     constraints close from a correct seed — never that the module seeds
//     correctly. A seed defect therefore reaches Fusion untested, which is how
//     the M/N toe-line seed defect got there. See stepGearProfiles.
//   - Fusion's own solver. The [BEVEL-F-SEED-HELD] gate compares Fusion's solved
//     geometry against the same closed forms; whether Fusion leaves a point inside
//     0.001 mm, and whether the gate false-fails on a correct solve, are
//     properties of Fusion's solver. Nothing in this repository runs it.
//   - The generated module's construction planes. The proof builds its own frames
//     from the closed form and never reads a plane the module constructs, so the
//     module's choice of plane family (§3a step E) reaches Fusion untested.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Constants the spec fixes. Each is written once here and read everywhere, so a
// case table cannot disagree with a step about what the spec says.
const (
	dedendumFactor = 1.25 // dedendum = 1.25 * Module
	addendumFactor = 1.0  // addendum = 1 * Module

	bevelPressureAngle = 20 * math.Pi / 180 // VirtualSpurProxy's default, not a bevel input
	bevelInvoluteSteps = 15                 // VirtualSpurProxy's default

	// Root sink: 0.05 * 2.25 * Module, drawn INSIDE the dedendum corner so the
	// Combine-Join meets the gear body across the root rather than along one line.
	rootSinkFactor = 0.05 * 2.25

	faceWidthMargin  = 0.95 // Maximum Face Width
	baseHeightMargin = 0.95 // Maximum Base Height
	baseHeightFloor  = 1.05 // Minimum Base Height
	boreMargin       = 0.95 // Maximum Bore Diameter
	minTeethConstant = 5.27 // Minimum Teeth = 5.27 * cos gamma
	toeExtensionCap  = 0.99 // Toe Extension 100 stops at 0.99 of the way to the Toe Limit

	sliceCount        = 8    // §3a step E: a fixed eight offset planes
	crownPerRad       = 0.5  // §3a step H: _CROWN_PER_RAD
	traceEndOvershoot = 0.06 // §3a step B: R_lo/R_hi sit 0.06*span past the face
)

// pt is a plane-local point in millimetres. The §2 lattice lives in the Gear
// Profiles sketch's own 2-D frame ([BEVEL-F-APEX-LOCAL]); the per-gear hexagons
// live in a (station, radius) frame about that gear's shaft axis.
type pt struct{ X, Y float64 }

func (p pt) add(q pt) pt        { return pt{p.X + q.X, p.Y + q.Y} }
func (p pt) sub(q pt) pt        { return pt{p.X - q.X, p.Y - q.Y} }
func (p pt) scale(k float64) pt { return pt{p.X * k, p.Y * k} }
func (p pt) dot(q pt) float64   { return p.X*q.X + p.Y*q.Y }
func (p pt) norm() float64      { return math.Hypot(p.X, p.Y) }

// member carries one member of the gear pair. Both carry the same fields; which one
// is the pinion and which the driving gear is the label alone.
//
// The name is `member` rather than `gear` because this package's rendering helpers
// already declare a `gear` of their own, and two declarations of one name in one
// package is a build failure rather than a shadow.
type member struct {
	label string

	teeth       float64
	gamma       float64 // pitch cone angle
	gammaRoot   float64 // root cone angle, gamma - atan(1.25*Module/R)
	pitchRadius float64

	baseHeight   float64 // resolved: fallback raised/capped, or the user's value
	toeRadius    float64 // resolved: 0 means auto, the inner toe corner at Toe Extension 0
	toeCeiling   float64 // Toe Radius Ceiling, the OUTER toe corner at Toe Extension 0
	toeLimit     float64 // |Ded->X|
	boreDiameter float64 // resolved AND bounded by the Maximum Bore Diameter
	maxBore      float64
	minBaseH     float64
	maxBaseH     float64
	minTeeth     float64

	// Directions in the Gear Profiles sketch frame.
	axis     pt // unit Apex -> this gear's shaft point (A or B)
	radial   pt // unit, perpendicular to axis, pointing toward this gear's dedendum corner
	dedendum pt // unit Apex2 -> this gear's dedendum corner (C or D)

	virtualPitchRadius float64 // back-cone (Tredgold) radius, r / cos gamma
	virtualTeeth       float64 // 2 * virtualPitchRadius / Module — a REAL number, never rounded
}

// figure is one configuration's whole closed-form geometry: the §2 lattice, both
// gears, and the values the later steps read off it.
type figure struct {
	module       float64
	sigma        float64 // Shaft Angle
	coneDistance float64 // the DIAGONAL sqrt(PPD^2 + DPD^2), not R
	pitchCone    float64 // R, the Pitch Cone Distance
	apexDed      float64 // |Apex->Ded| = sqrt(R^2 + (1.25*Module)^2)

	maxShaftAngle float64
	maxFaceWidth  float64
	faceWidth     float64
	rootLength    float64
	toothSpacing  float64
	rootSink      float64

	spiralAngle  float64 // psi
	handSign     float64 // +1 Right, -1 Left, on the DRIVING gear
	cutterRadius float64 // 0 means auto: R_mean
	toeExtension float64 // percent
	boreEnabled  bool

	apex    pt
	pinion  member
	driving member

	pts map[string]pt // every named §2 point, by the spec's own name
}

func sq(x float64) float64 { return x * x }

// newFigure resolves one parameter case the way "Reading the raw numbers" and §2
// resolve it: cone angles first, then the base heights, then the Face Width and
// the Root Length, and only then the bore — whose toe term needs the Root Length,
// which is why it cannot resolve during input validation.
func newFigure(p map[string]float64) figure {
	m := p["module"]
	nd := p["drivingTeeth"]
	np := p["pinionTeeth"]
	sigma := p["shaftAngleDeg"] * math.Pi / 180

	ppd := m * np
	dpd := m * nd

	// tan gamma_p = sin S * PPD / (DPD + PPD * cos S); gamma_g = S - gamma_p.
	gp := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gg := sigma - gp
	r := (ppd / 2) / math.Sin(gp) // Pitch Cone Distance R
	ded := dedendumFactor * m
	apexDed := math.Hypot(r, ded)

	f := figure{
		module:       m,
		sigma:        sigma,
		coneDistance: math.Hypot(ppd, dpd),
		pitchCone:    r,
		apexDed:      apexDed,
		toothSpacing: p["toothSpacing"],
		rootSink:     rootSinkFactor * m,
		spiralAngle:  p["spiralAngleDeg"] * math.Pi / 180,
		cutterRadius: p["cutterRadius"],
		toeExtension: p["toeExtension"],
		boreEnabled:  p["boreDisable"] == 0,
		pts:          map[string]pt{},
	}
	f.handSign = 1
	if p["handLeft"] == 1 {
		f.handSign = -1
	}
	f.maxShaftAngle = maxShaftAngle(ppd, dpd)

	f.pinion = member{label: "Pinion", teeth: np, gamma: gp, pitchRadius: ppd / 2}
	f.driving = member{label: "Driving", teeth: nd, gamma: gg, pitchRadius: dpd / 2}

	for _, g := range []*member{&f.pinion, &f.driving} {
		g.gammaRoot = g.gamma - math.Atan(ded/r)
		g.minTeeth = minTeethConstant * math.Cos(g.gamma)
		g.minBaseH = baseHeightFloor * ded * math.Sin(g.gamma)
		g.maxBaseH = baseHeightMargin * (g.pitchRadius - ded*math.Cos(g.gamma)) * math.Tan(g.gamma)
		g.virtualPitchRadius = g.pitchRadius / math.Cos(g.gamma)
		g.virtualTeeth = 2 * g.virtualPitchRadius / m
	}

	// Base heights. The driving fallback is Module * Driving Teeth / 8; the pinion
	// fallback scales the RESOLVED driving height by the tooth ratio and then takes
	// the pinion's OWN Maximum Base Height, since the two gammas differ.
	f.driving.baseHeight = clampBaseHeight(&f.driving, p["drivingBaseHeight"], m*nd/8)
	f.pinion.baseHeight = clampBaseHeight(&f.pinion, p["pinionBaseHeight"], f.driving.baseHeight*np/nd)

	// Maximum Face Width: 0.95 * the smaller of the two perpendicular distances,
	// A to line C-H and B to line D-J. In closed form each is R * sin^2(that gear's
	// gamma), so at Shaft Angle 90 the bound is 0.95 * min(PPD,DPD)^2 / (2*ConeDistance)
	// — the SMALLER pitch diameter, never the pinion's by name.
	f.maxFaceWidth = faceWidthMargin * r * math.Min(sq(math.Sin(gp)), sq(math.Sin(gg)))
	f.faceWidth = p["faceWidth"]
	if f.faceWidth == 0 {
		f.faceWidth = f.coneDistance / 6
	}
	f.faceWidth = math.Min(f.faceWidth, f.maxFaceWidth)

	// Toe radii. 0 means auto: this gear's inner toe corner at Toe Extension 0,
	// which is what makes Toe Extension 0 reproduce today's profile exactly.
	f.pinion.toeRadius = resolveToeRadius(&f.pinion, f.faceWidth, p["pinionToeRadius"])
	f.driving.toeRadius = resolveToeRadius(&f.driving, f.faceWidth, p["drivingToeRadius"])
	for _, g := range []*member{&f.pinion, &f.driving} {
		g.toeCeiling = (g.pitchRadius - ded*math.Cos(g.gamma)) * (1 - f.faceWidth/r)
		g.toeLimit = apexDed - g.toeRadius/math.Sin(g.gammaRoot)
	}

	// Root Length: the Face Width re-measured along the root element, plus the
	// extension's share of the window. Face Width still resolves as it always did.
	rl0 := f.faceWidth * apexDed / r
	f.rootLength = rl0
	if f.toeExtension > 0 {
		reach := math.Min(f.pinion.toeLimit, f.driving.toeLimit)
		if reach > rl0 {
			f.rootLength = rl0 + (f.toeExtension/100)*toeExtensionCap*(reach-rl0)
		}
	}

	f.buildLattice()

	// Maximum Bore Diameter resolves HERE, in §2, at the step that applies the
	// Maximum Face Width: its heel term is closed-form but its toe term needs the
	// Root Length, so the input-reading pass deliberately leaves the bores unbounded.
	for _, side := range []struct {
		g   *member
		raw float64
	}{{&f.pinion, p["pinionBore"]}, {&f.driving, p["drivingBore"]}} {
		g := side.g
		rHeel := g.pitchRadius - g.baseHeight/math.Tan(g.gamma)
		rToe := (apexDed - f.rootLength) * math.Sin(g.gammaRoot)
		g.maxBore = 2 * boreMargin * math.Min(rHeel, rToe)
		bore := side.raw
		if bore == 0 {
			bore = 2 * g.pitchRadius / 4 // this gear's Pitch Diameter / 4
		}
		g.boreDiameter = math.Min(bore, g.maxBore)
	}
	return f
}

// maxShaftAngle is the cone-angle singularity capped at 150 degrees. A pitch cone
// angle reaching 90 degrees turns that gear's cone inside out, and both stay below
// 90 exactly while cos(Shaft Angle) > -smaller/larger, so acos of that ratio is a
// hard singularity the range check must reject AT as well as above. The 150 half is
// a practical ceiling and is inclusive.
func maxShaftAngle(ppd, dpd float64) float64 {
	return math.Min(math.Acos(-math.Min(ppd, dpd)/math.Max(ppd, dpd)), 150*math.Pi/180)
}

// clampBaseHeight raises a fallback below the Minimum Base Height and caps one
// above the Maximum. A USER value outside either end is rejected by the generated
// module rather than clamped; the geometry step asserts the window, and no case in
// this proof runs the module, so the refusal itself is out of reach.
func clampBaseHeight(g *member, user, fallback float64) float64 {
	h := user
	if h == 0 {
		h = fallback
	}
	return math.Max(g.minBaseH, math.Min(h, g.maxBaseH))
}

func resolveToeRadius(g *member, faceWidth, user float64) float64 {
	if user > 0 {
		return user
	}
	return g.pitchRadius - faceWidth/math.Sin(g.gamma)
}

// buildLattice writes every named §2 point at the closed-form position §2 seeds it
// at. The frame is the Gear Profiles sketch's own: the projected centre c at the
// origin, the projected anchor line along +X, and perp = (-d.y, d.x) = +Y. The
// grow side is +Y here because the proof fixes the anchor line; in Fusion the sign
// comes from the target-plane normal ([BEVEL-F-GROW-SIDE]).
func (f *figure) buildLattice() {
	m, r, sigma := f.module, f.pitchCone, f.sigma
	ded := dedendumFactor * m
	gp, gg := f.pinion.gamma, f.driving.gamma

	perp := pt{0, 1}
	c := pt{0, 0}
	apex := c.add(perp.scale(r*math.Cos(gg) + f.driving.baseHeight))

	drivingAxis := pt{0, -1}
	pinionAxis := pt{math.Sin(sigma), -math.Cos(sigma)} // the +X-most of the two senses
	pitchDir := pt{math.Sin(gg), -math.Cos(gg)}
	dedP := pt{math.Cos(gg), math.Sin(gg)} // Apex2 -> C: the u with u . unit(Apex->A) = sin gamma_p > 0
	dedD := dedP.scale(-1)                 // Apex2 -> D: (-u) . unit(Apex->B) = sin gamma_g > 0
	radP := pt{-math.Cos(sigma), -math.Sin(sigma)}
	radD := pt{1, 0}

	f.apex = apex
	f.pinion.axis, f.pinion.radial, f.pinion.dedendum = pinionAxis, radP, dedP
	f.driving.axis, f.driving.radial, f.driving.dedendum = drivingAxis, radD, dedD

	b := apex.add(drivingAxis.scale(r * math.Cos(gg)))
	a := apex.add(pinionAxis.scale(r * math.Cos(gp)))
	apex2 := apex.add(pitchDir.scale(r))
	cc := apex2.add(dedP.scale(ded))
	dd := apex2.add(dedD.scale(ded))

	hp, hg := f.pinion.baseHeight, f.driving.baseHeight
	e := a.add(pinionAxis.scale(ded * math.Sin(gp)))
	ff := b.add(drivingAxis.scale(ded * math.Sin(gg)))
	g := a.add(pinionAxis.scale(hp))
	h := apex2.add(dedP.scale(hp / math.Sin(gp)))
	i := b.add(drivingAxis.scale(hg))
	j := apex2.add(dedD.scale(hg / math.Sin(gg)))

	k := apex2.add(dedP.scale(f.pinion.virtualPitchRadius))
	l := apex2.add(dedD.scale(f.driving.virtualPitchRadius))
	kp := apex2.add(dedP.scale(f.pinion.virtualPitchRadius + f.toothSpacing))
	lp := apex2.add(dedD.scale(f.driving.virtualPitchRadius + f.toothSpacing))

	// The toe lattice. M rides the root axis one Root Length back from the dedendum
	// corner; N slides from M along the dedendum direction until it reaches this
	// gear's Toe Radius, and A' is N's foot on the shaft axis.
	toe := func(g member, corner pt) (mm, nn, prime pt) {
		mm = apex.add(corner.sub(apex).scale(1 - f.rootLength/corner.sub(apex).norm()))
		nn = mm.add(g.dedendum.scale((f.radiusOf(g, mm) - g.toeRadius) / math.Cos(g.gamma)))
		prime = apex.add(g.axis.scale(f.stationOf(g, nn)))
		return mm, nn, prime
	}
	mPin, nPin, aPrime := toe(f.pinion, cc)
	oDrv, pDrv, bPrime := toe(f.driving, dd)

	f.pts = map[string]pt{
		"c": c, "Apex": apex, "B": b, "A": a, "Apex2": apex2, "C": cc, "D": dd,
		"E": e, "F": ff, "G": g, "H": h, "I": i, "J": j,
		"K": k, "K'": kp, "L": l, "L'": lp,
		"M": mPin, "N": nPin, "A'": aPrime, "O": oDrv, "P": pDrv, "B'": bPrime,
	}
}

// stationOf is a point's distance from the Apex measured ALONG this gear's shaft
// axis; radiusOf is its perpendicular distance from that axis, signed positive
// toward this gear's own dedendum corner.
func (f figure) stationOf(g member, p pt) float64 { return p.sub(f.apex).dot(g.axis) }
func (f figure) radiusOf(g member, p pt) float64  { return p.sub(f.apex).dot(g.radial) }

// seedOrder is the order §2 creates the points in, which is the order the
// [BEVEL-F-SEED-HELD] gate checks them so the message names the earliest site that
// flipped. K' and L' are built only when Tooth Spacing is above zero.
func (f figure) seedOrder() []string {
	names := []string{"Apex", "B", "A", "Apex2", "C", "D", "E", "F", "G", "H", "I", "J", "K"}
	if f.toothSpacing > 0 {
		names = append(names, "K'")
	}
	names = append(names, "M", "N", "A'", "L")
	if f.toothSpacing > 0 {
		names = append(names, "L'")
	}
	return append(names, "O", "P", "B'")
}

// hexagon is this gear's frustum profile in (station, radius), in the draw order
// the Profile sketch uses: A'->G->H->C->M->N for the pinion, B'->I->J->D->O->P for
// the driving gear. The first edge is the shaft axis, which every body operation
// below revolves, patterns and bores about.
func (f figure) hexagon(g member) []pt {
	var names []string
	if g.label == "Pinion" {
		names = []string{"A'", "G", "H", "C", "M", "N"}
	} else {
		names = []string{"B'", "I", "J", "D", "O", "P"}
	}
	out := make([]pt, 0, len(names))
	for _, n := range names {
		p := f.pts[n]
		out = append(out, pt{f.stationOf(g, p), f.radiusOf(g, p)})
	}
	return out
}

// polygonArea and polygonCentroidY are Pappus's two inputs: revolving the hexagon a
// full turn about the station axis sweeps 2*pi*Ybar*Area.
func polygonArea(v []pt) float64 {
	sum := 0.0
	for i := range v {
		j := (i + 1) % len(v)
		sum += v[i].X*v[j].Y - v[j].X*v[i].Y
	}
	return math.Abs(sum) / 2
}

func polygonCentroidY(v []pt) float64 {
	cross, sum := 0.0, 0.0
	for i := range v {
		j := (i + 1) % len(v)
		c := v[i].X*v[j].Y - v[j].X*v[i].Y
		cross += c
		sum += (v[i].Y + v[j].Y) * c
	}
	return sum / (3 * cross)
}

func revolvedVolume(v []pt) float64 {
	return 2 * math.Pi * math.Abs(polygonCentroidY(v)*polygonArea(v))
}

// coneFrustumArea is the lateral area a profile edge sweeps about the station axis.
func coneFrustumArea(p, q pt) float64 {
	return math.Pi * (p.Y + q.Y) * math.Hypot(q.X-p.X, q.Y-p.Y)
}

// caseParams builds one case's parameter map over the shipped defaults.
//
// The sketch tables stay at the dialog's own default Module. The solid tables run
// at Module 4 to 8 instead: decad's mesh bound has an absolute floor, so a figure
// small enough brings every measurement inside it and the gate reports Suspect on
// geometry that is in fact correct. Module is a pure scale on this figure.
func caseParams(over map[string]float64) map[string]float64 {
	p := map[string]float64{
		"module": 1, "drivingTeeth": 31, "pinionTeeth": 31, "shaftAngleDeg": 90,
		"spiralAngleDeg": 0,
	}
	for k, v := range over {
		p[k] = v
	}
	return p
}

// sketchCases is the §2 regime: the ends of the Shaft Angle range, both directions
// of the gear ratio (either member can be the smaller, binding side), the tooth
// counts the virtual count needs, and each input that moves the lattice.
//
// shaft_angle_30 carries declaredRefusal. The spec's Shaft Angle floor is 30
// degrees and the spec says so; this particular lattice does not reach it, because
// its conditioning falls below the sketch engine's trust floor there. That is a
// property of THIS net — of three independently written lattices two refuse 30 and
// one passes — so the case stays in the table and is marked, rather than the range
// being narrowed on one net's evidence.
var sketchCases = []proofkit.Case{
	{Name: "default_31_31_90", Params: caseParams(nil)},
	{Name: "shaft_angle_30", Params: caseParams(map[string]float64{"shaftAngleDeg": 30, "declaredRefusal": 1})},
	{Name: "shaft_angle_35", Params: caseParams(map[string]float64{"shaftAngleDeg": 35})},
	{Name: "shaft_angle_60", Params: caseParams(map[string]float64{"shaftAngleDeg": 60})},
	{Name: "shaft_angle_142", Params: caseParams(map[string]float64{"shaftAngleDeg": 142})},
	{Name: "shaft_angle_150", Params: caseParams(map[string]float64{"shaftAngleDeg": 150})},
	{Name: "ratio_driving_31_pinion_17", Params: caseParams(map[string]float64{"pinionTeeth": 17})},
	{Name: "ratio_driving_17_pinion_31", Params: caseParams(map[string]float64{"drivingTeeth": 17})},
	{Name: "teeth_16_12", Params: caseParams(map[string]float64{"drivingTeeth": 16, "pinionTeeth": 12})},
	{Name: "teeth_4_4", Params: caseParams(map[string]float64{"drivingTeeth": 4, "pinionTeeth": 4})},
	{Name: "tooth_spacing_positive", Params: caseParams(map[string]float64{
		"drivingTeeth": 43, "pinionTeeth": 31, "shaftAngleDeg": 75, "toothSpacing": 0.4})},
	{Name: "toe_extension_100", Params: caseParams(map[string]float64{"toeExtension": 100})},
	{Name: "toe_radius_user", Params: caseParams(map[string]float64{
		"drivingToeRadius": 3, "pinionToeRadius": 3})},
	{Name: "base_height_user", Params: caseParams(map[string]float64{
		"drivingBaseHeight": 3, "pinionBaseHeight": 3})},
	{Name: "bore_disabled", Params: caseParams(map[string]float64{"boreDisable": 1})},
	{Name: "face_width_user", Params: caseParams(map[string]float64{"faceWidth": 3})},
}

// declaredRefusal reports a case the spec admits and this lattice cannot reach.
func declaredRefusal(t testing.TB, p map[string]float64) bool {
	if p["declaredRefusal"] != 1 {
		return false
	}
	proofkit.Unmodelled(t, "declared refusal: this §2 lattice's conditioning falls below the "+
		"sketch engine's trust floor at Shaft Angle %.0f°, which is a property of this net and "+
		"not of the Shaft Angle range the spec states", p["shaftAngleDeg"])
	return true
}

// stepDerivedValues proves the closed-form bounds the input pass and §2 resolve,
// against the lattice they are read off. It authors the figure's named points as
// reference geometry so the case has geometry of its own to solve, and every
// assertion below is taken on those solved positions rather than on the arithmetic
// alone.
// A declared refusal is a property of the §2 constraint NET, not of the closed
// form, so this step runs every case in the table including that one.
func stepDerivedValues(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newFigure(p)

	proofkit.Step(t, "the named §2 points, as reference geometry")
	for _, name := range f.seedOrder() {
		q := f.pts[name]
		s.CreateReferencePoint(q.X, q.Y, name).SetName(name)
	}

	proofkit.Step(t, "the Shaft Angle range")
	if f.sigma < 30*math.Pi/180-1e-12 {
		t.Errorf("Shaft Angle %.3f rad is below the documented 30° floor", f.sigma)
	}
	if f.sigma >= f.maxShaftAngle && f.maxShaftAngle < 150*math.Pi/180 {
		t.Errorf("Shaft Angle %.4f rad reaches the cone-angle singularity %.4f rad",
			f.sigma, f.maxShaftAngle)
	}
	// Both cone angles stay strictly below 90°, which is exactly what the Maximum
	// Shaft Angle exists to guarantee: at 90° R*cos(gamma) changes sign and the
	// along-shaft seed points backwards.
	for _, g := range []member{f.pinion, f.driving} {
		if g.gamma <= 0 || g.gamma >= math.Pi/2 {
			t.Errorf("%s pitch cone angle %.4f rad is outside (0, pi/2)", g.label, g.gamma)
		}
		if math.Abs(g.gamma+otherGamma(f, g)-f.sigma) > 1e-12 {
			t.Errorf("%s: gamma_p + gamma_g does not close on the Shaft Angle", g.label)
		}
	}

	proofkit.Step(t, "the two-lengths rule: Cone Distance is the diagonal, R the pitch cone distance")
	if math.Abs(f.sigma-math.Pi/2) < 1e-12 && math.Abs(f.coneDistance-2*f.pitchCone) > 1e-9 {
		t.Errorf("at Shaft Angle 90° Cone Distance %.6f must equal 2R %.6f",
			f.coneDistance, 2*f.pitchCone)
	}

	proofkit.Step(t, "the base-height window and the Minimum Teeth floor that keeps it open")
	for _, g := range []member{f.pinion, f.driving} {
		if g.teeth < 3 {
			t.Errorf("%s: %0.f teeth is below the blanket floor of 3", g.label, g.teeth)
		}
		if g.teeth < g.minTeeth {
			t.Errorf("%s: %0.f teeth is below the computed floor %.3f", g.label, g.teeth, g.minTeeth)
		}
		if g.minBaseH > g.maxBaseH {
			t.Errorf("%s: base-height window is empty (%.4f > %.4f), which the Minimum Teeth "+
				"check exists to rule out before the heights resolve",
				g.label, g.minBaseH, g.maxBaseH)
		}
		if g.baseHeight < g.minBaseH-1e-12 || g.baseHeight > g.maxBaseH+1e-12 {
			t.Errorf("%s: resolved base height %.4f is outside [%.4f, %.4f]",
				g.label, g.baseHeight, g.minBaseH, g.maxBaseH)
		}
		// The Maximum Base Height is deliberately conservative: it sits
		// 1.25*Module*sin(gamma) below the true crossing r*tan(gamma), because it
		// starts from the dedendum corner rather than from the pitch point.
		trueCrossing := g.pitchRadius * math.Tan(g.gamma)
		if g.maxBaseH >= trueCrossing {
			t.Errorf("%s: the bound %.4f must stay under the true crossing %.4f",
				g.label, g.maxBaseH, trueCrossing)
		}
	}

	proofkit.Step(t, "the Maximum Face Width, on the SOLVED points and from both sides")
	// The bound is 0.95 * the smaller of the perpendicular distance from A to line
	// C-H and from B to line D-J. Reading it off the solved lattice is what the
	// generated module does; the closed form is what this case compares against.
	dp := pointLineDistance(f.pts["A"], f.pts["C"], f.pts["H"])
	dd := pointLineDistance(f.pts["B"], f.pts["D"], f.pts["J"])
	measured := faceWidthMargin * math.Min(dp, dd)
	if rel(measured, f.maxFaceWidth) > 1e-9 {
		t.Errorf("Maximum Face Width from the solved points %.6f disagrees with the closed form %.6f",
			measured, f.maxFaceWidth)
	}
	if f.faceWidth > f.maxFaceWidth+1e-12 {
		t.Errorf("resolved Face Width %.6f exceeds its maximum %.6f", f.faceWidth, f.maxFaceWidth)
	}
	if math.Abs(f.sigma-math.Pi/2) < 1e-12 {
		smaller := math.Min(2*f.pinion.pitchRadius, 2*f.driving.pitchRadius)
		want := faceWidthMargin * sq(smaller) / (2 * f.coneDistance)
		if rel(want, f.maxFaceWidth) > 1e-9 {
			t.Errorf("at 90° the bound must read off the SMALLER pitch diameter: %.6f vs %.6f",
				want, f.maxFaceWidth)
		}
	}

	proofkit.Step(t, "the toe lattice")
	for _, g := range []member{f.pinion, f.driving} {
		if g.toeRadius <= 0 {
			t.Errorf("%s: Toe Radius %.6f must be strictly positive — only the front face's "+
				"foot A'/B' touches the shaft axis, never N/P", g.label, g.toeRadius)
		}
		if g.toeRadius >= g.toeCeiling && f.toeExtension > 0 {
			t.Errorf("%s: Toe Extension %.0f%% needs a Toe Radius below the ceiling %.6f, got %.6f",
				g.label, f.toeExtension, g.toeCeiling, g.toeRadius)
		}
	}
	// At Toe Extension 0 the outer toe corner sits exactly at the Toe Radius Ceiling.
	if f.toeExtension == 0 {
		for _, g := range []member{f.pinion, f.driving} {
			corner := "M"
			if g.label == "Driving" {
				corner = "O"
			}
			if rel(f.radiusOf(g, f.pts[corner]), g.toeCeiling) > 1e-9 {
				t.Errorf("%s: outer toe corner %.6f is not the Toe Radius Ceiling %.6f",
					g.label, f.radiusOf(g, f.pts[corner]), g.toeCeiling)
			}
		}
	}

	proofkit.Step(t, "the Maximum Bore Diameter, whose toe term needs the Root Length")
	for _, g := range []member{f.pinion, f.driving} {
		heelPoint, toePoint := "H", "M"
		if g.label == "Driving" {
			heelPoint, toePoint = "J", "O"
		}
		rHeel := f.radiusOf(g, f.pts[heelPoint])
		rToe := f.radiusOf(g, f.pts[toePoint])
		want := 2 * boreMargin * math.Min(rHeel, rToe)
		if rel(want, g.maxBore) > 1e-9 {
			t.Errorf("%s: Maximum Bore Diameter from the solved heel/toe corners %.6f "+
				"disagrees with the closed form %.6f", g.label, want, g.maxBore)
		}
		if f.boreEnabled && g.boreDiameter > g.maxBore+1e-12 {
			t.Errorf("%s: resolved bore diameter %.6f exceeds its maximum %.6f — a bore past "+
				"the heel term takes the whole flat back face", g.label, g.boreDiameter, g.maxBore)
		}
	}

	proofkit.Step(t, "the virtual tooth count is a real number and is never rounded")
	for _, g := range []member{f.pinion, f.driving} {
		if rel(g.virtualTeeth, g.teeth/math.Cos(g.gamma)) > 1e-12 {
			t.Errorf("%s: virtual tooth count %.6f is not z / cos gamma", g.label, g.virtualTeeth)
		}
		if rel(g.virtualPitchRadius*2/f.module, g.virtualTeeth) > 1e-12 {
			t.Errorf("%s: the virtual count and the back-cone radius have drifted apart", g.label)
		}
	}
}

func otherGamma(f figure, g member) float64 {
	if g.label == "Pinion" {
		return f.driving.gamma
	}
	return f.pinion.gamma
}

// pointLineDistance is the perpendicular distance from p to the infinite line
// through a and b.
func pointLineDistance(p, a, b pt) float64 {
	d := b.sub(a)
	n := d.norm()
	if n == 0 {
		return math.NaN()
	}
	return math.Abs((p.X-a.X)*d.Y-(p.Y-a.Y)*d.X) / n
}

// rel is the relative difference, falling back to the absolute one at zero.
func rel(got, want float64) float64 {
	if want == 0 {
		return math.Abs(got)
	}
	return math.Abs(got-want) / math.Abs(want)
}

// polygonCentroidX is the other half of the polygon centroid; the Y half above is
// what Pappus reads.
func polygonCentroidX(v []pt) float64 {
	cross, sum := 0.0, 0.0
	for i := range v {
		j := (i + 1) % len(v)
		c := v[i].X*v[j].Y - v[j].X*v[i].Y
		cross += c
		sum += (v[i].X + v[j].X) * c
	}
	return sum / (3 * cross)
}

// virtualFlanks returns one gear's two involute flank sample sets and its embedded
// flag, with the flanks trimmed at the root circle when the tooth is embedded.
//
// The involute math is imported rather than derived again: involute.Flanks lays the
// samples down, mirrors them across +X, rotates the pitch crossing to pi/(2*z_v) and
// applies the requested angle, in that order. Where the root circle sits OUTSIDE the
// base circle the flank starts inside it, and the drawer has no room for the
// flank-to-root lines; the sample at the root radius is then the flank's own end,
// and it is recovered by applying the same transform to involute.Point at that
// radius. The transform is read back off the first sample, which sits at exactly the
// base radius by construction.
//
// The tooth is drawn already rotated 180 degrees in Fusion, through the draw()
// angle rather than a sketch rotation. The proof lays the tooth's centreline down
// the drawing's own +X and records the equivalence here, because an angle applied to
// both flanks together moves the whole tooth and changes no radius.
func virtualFlanks(module, virtualPitchRadius, rootSink, virtualTeeth float64) (left, right []pt, embedded bool) {
	baseR := virtualPitchRadius * math.Cos(bevelPressureAngle)
	tipR := virtualPitchRadius + addendumFactor*module
	rootR := virtualPitchRadius - dedendumFactor*module - rootSink
	embedded = baseR < rootR

	l, r := involute.Flanks(baseR, tipR, virtualPitchRadius, virtualTeeth, bevelInvoluteSteps, 0)
	if len(l) == 0 {
		return nil, nil, embedded
	}
	signedTurn := math.Atan2(l[0].Y, l[0].X)
	add := func(q involute.Pt) {
		left = append(left, pt{q.X, q.Y})
		right = append(right, pt{q.X, -q.Y})
	}
	if embedded {
		x, y, ok := involute.Point(baseR, rootR)
		if !ok {
			return nil, nil, embedded
		}
		rx, ry := involute.Rotate(x, -y, signedTurn)
		add(involute.Pt{X: rx, Y: ry})
	}
	for _, q := range l {
		if embedded && math.Hypot(q.X, q.Y) <= rootR {
			continue
		}
		add(q)
	}
	_ = r
	return left, right, embedded
}
