// Package bevelgear_test proves the bevel gear pair's build, one function per
// step of spec/bevelgear/steps.md.
//
// This file holds the closed-form bevel geometry every step is measured
// against, and the parameter-case tables the steps run over. Nothing here draws
// anything; it is the arithmetic the spec's Variables section states, written
// once so a step function and its assertion cannot drift apart by each deriving
// it.
//
// UNITS. Every length here is in millimetres, which is the sketch engine's base
// unit and decad's. The generated module works in Fusion's internal centimetres
// and reads Module as a raw millimetre number; that conversion is the module's
// and is outside what this proof can observe. The parameter map keys are the
// dialog's own input ids, so a case reads as the dialog values it comes from.
//
// FRAME. The §2 figure is planar, and every solid step revolves or patterns it
// about one gear's shaft axis. So the solid steps work in that gear's own
// (station, radius) frame: +Z is the shaft axis pointing away from the Apex,
// the Apex sits at the origin, and a point's radius is its distance from +Z.
// That frame is a rigid motion of the real one, which is what makes every
// volume, radius, station and cone half-angle read here the same number the
// gear has in Fusion. What it does not carry is the target plane's orientation,
// which is a one-bit grow-side choice the sketch steps sweep instead.
package bevelgear_test

import (
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// The dialog input ids, as the parameter-map keys. The hand-written
// drawing_geometry_test.go declares the key* constants that hold these same
// strings and calls stepGearProfiles with a map built from them, so the two
// sides agree on the spelling by both taking it from the dialog's input table.
//
// gearSide is the one key with no dialog row behind it: it selects which of the
// pair a per-gear step builds, which the module decides by running the section
// twice rather than by reading an input.
const (
	sidePinion  = 0.0
	sideDriving = 1.0
)

// handRight and handLeft are the two Hand of Spiral list items, carried as
// numbers because a case table is a map of float64. The module reads the
// dropdown's selectedItem.name and compares it against the module constants
// _HAND_RIGHT = 'Right' and _HAND_LEFT = 'Left'.
const (
	handRight = 1.0
	handLeft  = -1.0
)

// bevelDesign is one resolved configuration of the pair: every value the
// Variables section derives, with the bounds already applied, in millimetres
// and radians.
type bevelDesign struct {
	module       float64
	shaftAngle   float64 // radians
	growSide     float64 // +1 or -1: which side of the anchor line the figure grows
	toothSpacing float64
	toeExtension float64 // percent, [0, 100]
	spiralAngle  float64 // radians
	hand         float64 // handRight or handLeft, the DRIVING gear's hand
	cutterRadius float64 // 0 means auto

	coneDistance float64 // sqrt(DPD^2 + PPD^2) — the diagonal, NOT R
	pitchCone    float64 // R, the Pitch Cone Distance
	rootCone     float64 // |Apex->Ded| = hypot(R, 1.25*module), shared by both gears

	faceWidth    float64 // resolved, after the Maximum Face Width cap
	maxFaceWidth float64
	rootLength   float64 // resolved |Ded->Toe|

	pinion  bevelSide
	driving bevelSide
}

// bevelSide is one gear of the pair.
type bevelSide struct {
	label     string
	teeth     float64
	pitchDia  float64
	gamma     float64 // pitch cone angle
	rootGamma float64 // root cone angle, gamma - atan(1.25*module/R)

	baseHeight float64 // resolved
	minBase    float64
	maxBase    float64
	minTeeth   float64

	toeRadius        float64 // resolved
	toeRadiusDefault float64
	toeRadiusCeiling float64
	toeLimit         float64

	virtualTeeth float64
	boreDiameter float64 // resolved; 0 when Enable Bore is off
}

func (s bevelSide) pitchRadius() float64 { return s.pitchDia / 2 }

// newBevelDesign resolves one parameter case the way _readInputs does, in the
// order the spec's Variables section fixes: the cone angles first, then the
// tooth-count floor, then each gear's base height between its own two bounds,
// then the Face Width against its cap, then the toe radii and the root length.
func newBevelDesign(p map[string]float64) bevelDesign {
	module := p["module"]
	sigma := p["shaftAngle"]
	drivingTeeth := p["drivingTeeth"]
	pinionTeeth := p["pinionTeeth"]

	dpd := module * drivingTeeth
	ppd := module * pinionTeeth
	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	pitchCone := (ppd / 2) / math.Sin(gammaP)
	rootCone := math.Hypot(pitchCone, 1.25*module)

	grow := p["growSide"]
	if grow == 0 {
		grow = 1
	}

	d := bevelDesign{
		module:       module,
		shaftAngle:   sigma,
		growSide:     grow,
		toothSpacing: p["toothSpacing"],
		toeExtension: p["toeExtension"],
		spiralAngle:  p["spiralAngle"],
		hand:         handRight,
		cutterRadius: p["cutterRadius"],
		coneDistance: math.Hypot(dpd, ppd),
		pitchCone:    pitchCone,
		rootCone:     rootCone,
	}
	if p["spiralHand"] != 0 {
		d.hand = p["spiralHand"]
	}
	d.pinion = newBevelSide("Pinion", module, pinionTeeth, ppd, gammaP, pitchCone, rootCone)
	d.driving = newBevelSide("Driving", module, drivingTeeth, dpd, gammaG, pitchCone, rootCone)

	// Base heights. The driving fallback is module * Driving Gear Teeth / 8; the
	// pinion's is the RESOLVED driving height scaled by the tooth ratio, then held
	// to the PINION's own bounds, which differ whenever the cone angles do.
	d.driving.baseHeight = resolveBaseHeight(p["drivingBaseHeight"], module*drivingTeeth/8, d.driving)
	pinionFallback := d.driving.baseHeight * pinionTeeth / drivingTeeth
	d.pinion.baseHeight = resolveBaseHeight(p["pinionBaseHeight"], pinionFallback, d.pinion)

	// Maximum Face Width: 0.95 times the smaller of the perpendicular distance
	// from A to the Pinion Dedendum line and from B to the Driving Dedendum line.
	// Both come out as R*sin(gamma)^2 = pitchRadius^2 / R, so the binding side is
	// the SMALLER pitch radius and never the pinion's by name.
	d.maxFaceWidth = 0.95 * math.Min(
		d.pinion.pitchRadius()*d.pinion.pitchRadius(),
		d.driving.pitchRadius()*d.driving.pitchRadius()) / pitchCone
	d.faceWidth = p["faceWidth"]
	if d.faceWidth <= 0 {
		d.faceWidth = math.Min(d.coneDistance/6, d.maxFaceWidth)
	}

	d.pinion.resolveToe(module, pitchCone, rootCone, d.faceWidth, p["pinionToeRadius"])
	d.driving.resolveToe(module, pitchCone, rootCone, d.faceWidth, p["drivingToeRadius"])

	// Root Length. At Toe Extension 0 it is the Face Width re-measured along the
	// root element. Toe Extension 100 stops at 0.99 of the way to the SMALLER of
	// the two gears' Toe Limits, because the pair shares one root length and the
	// last percent is what leaves a toe cone for the end-cut to find.
	base := d.faceWidth * rootCone / pitchCone
	limit := math.Min(d.pinion.toeLimit, d.driving.toeLimit)
	d.rootLength = base + (d.toeExtension/100)*0.99*(limit-base)

	d.pinion.virtualTeeth = math.Floor(2 * (d.pinion.pitchRadius() / math.Cos(d.pinion.gamma)) / module)
	d.driving.virtualTeeth = math.Floor(2 * (d.driving.pitchRadius() / math.Cos(d.driving.gamma)) / module)

	if p["boreEnable"] != 0 {
		d.pinion.boreDiameter = boreDiameter(p["pinionBore"], ppd)
		d.driving.boreDiameter = boreDiameter(p["drivingBore"], dpd)
	}
	return d
}

func newBevelSide(label string, module, teeth, pitchDia, gamma, pitchCone, rootCone float64) bevelSide {
	r := pitchDia / 2
	return bevelSide{
		label:     label,
		teeth:     teeth,
		pitchDia:  pitchDia,
		gamma:     gamma,
		rootGamma: gamma - math.Atan(1.25*module/pitchCone),
		minBase:   1.05 * 1.25 * module * math.Sin(gamma),
		maxBase:   0.95 * (r - 1.25*module*math.Cos(gamma)) * math.Tan(gamma),
		minTeeth:  5.27 * math.Cos(gamma),
	}
}

// resolveToe fixes this gear's toe radius and the two figures the Toe Extension
// window is measured between. A Toe Radius of 0 means auto, and the auto value
// is the inner toe corner's radius at Toe Extension 0, which is what makes Toe
// Extension 0 reproduce the profile built before the input existed.
func (s *bevelSide) resolveToe(module, pitchCone, rootCone, faceWidth, given float64) {
	r := s.pitchRadius()
	s.toeRadiusDefault = r - faceWidth/math.Sin(s.gamma)
	s.toeRadiusCeiling = (r - 1.25*module*math.Cos(s.gamma)) * (1 - faceWidth/pitchCone)
	s.toeRadius = given
	if s.toeRadius <= 0 {
		s.toeRadius = s.toeRadiusDefault
	}
	s.toeLimit = rootCone - s.toeRadius/math.Sin(s.rootGamma)
}

// resolveBaseHeight applies one gear's own two base-height bounds: a fallback
// is raised to the minimum and capped at the maximum, and a user value outside
// either end is a rejection the module raises rather than a value to clamp.
// The proof's tables carry only configurations the module accepts, so the
// clamp is what a case reaches.
func resolveBaseHeight(given, fallback float64, s bevelSide) float64 {
	value := given
	if value <= 0 {
		value = fallback
	}
	return math.Min(math.Max(value, s.minBase), s.maxBase)
}

func boreDiameter(given, pitchDia float64) float64 {
	if given > 0 {
		return given
	}
	return pitchDia / 4
}

func (d bevelDesign) side(which float64) bevelSide {
	if which == sideDriving {
		return d.driving
	}
	return d.pinion
}

// ---------------------------------------------------------------- the axial profile

// axialPt is one §2 point in the gear's own axial half-plane: station is its
// distance from the Apex along the shaft axis, radius its perpendicular
// distance from that axis. The §2 sketch's two-dimensional frame carries the
// same figure rotated and offset; this pair is what survives the revolve.
type axialPt struct{ station, radius float64 }

// hexagon is the six-vertex profile the gear body is revolved from, in draw
// order: A'->G->H->C->M->N for the pinion, B'->I->J->D->O->P for the driving
// gear. The first edge is the shaft axis and is what every later body
// operation takes its axis from.
type hexagon struct {
	toeFoot  axialPt // A' / B'  — the front face's foot, on the shaft axis
	heelAxis axialPt // G / I   — the heel end, on the shaft axis
	heelEnd  axialPt // H / J    — the heel edge's far end
	ded      axialPt // C / D    — the dedendum corner
	toeCone  axialPt // M / O    — the toe edge's inner endpoint, on the root element
	toeIn    axialPt // N / P    — the inner toe corner, at the Toe Radius
}

// axialProfile solves one gear's hexagon from the closed form.
//
// Every station and radius below follows from the §2 constraint net rather than
// from a seed: the along-shaft lengths |Apex->A| and |Apex->B| are driven by the
// Apex 2 closure, the module-length extensions by their perpendiculars, and the
// heel edge by the base-height offset dimension.
func (d bevelDesign) axialProfile(which float64) hexagon {
	s := d.side(which)
	r := s.pitchRadius()
	ded := 1.25 * d.module
	// C / D: the dedendum corner, one dedendum off the pitch point Apex2 along
	// the back cone.
	dedPt := axialPt{
		station: d.pitchCone*math.Cos(s.gamma) + ded*math.Sin(s.gamma),
		radius:  r - ded*math.Cos(s.gamma),
	}
	// G / I: the base-height offset drives the heel edge to one base height
	// beyond A / B along the shaft.
	heelStation := d.pitchCone*math.Cos(s.gamma) + s.baseHeight
	// H / J: |Ded->Heel| follows from that same offset, measured along the
	// dedendum line rather than along the shaft.
	dedToHeel := s.baseHeight/math.Sin(s.gamma) - ded
	heelEnd := axialPt{
		station: dedPt.station + dedToHeel*math.Sin(s.gamma),
		radius:  dedPt.radius - dedToHeel*math.Cos(s.gamma),
	}
	// M / O: the root length back along the root element from the dedendum corner.
	toeCone := axialPt{
		station: (d.rootCone - d.rootLength) * math.Cos(s.rootGamma),
		radius:  (d.rootCone - d.rootLength) * math.Sin(s.rootGamma),
	}
	// N / P: slid from M along the toe line, which is parallel to the heel edge,
	// until it rides at the Toe Radius.
	slide := (toeCone.radius - s.toeRadius) / math.Cos(s.gamma)
	toeIn := axialPt{
		station: toeCone.station + slide*math.Sin(s.gamma),
		radius:  s.toeRadius,
	}
	return hexagon{
		toeFoot:  axialPt{station: toeIn.station, radius: 0},
		heelAxis: axialPt{station: heelStation, radius: 0},
		heelEnd:  heelEnd,
		ded:      dedPt,
		toeCone:  toeCone,
		toeIn:    toeIn,
	}
}

// vertices returns the hexagon in its draw order.
func (h hexagon) vertices() []axialPt {
	return []axialPt{h.toeFoot, h.heelAxis, h.heelEnd, h.ded, h.toeCone, h.toeIn}
}

// revolvedVolume is the volume the hexagon sweeps about the shaft axis, by the
// solid-of-revolution integral taken edge by edge around the closed loop. It is
// the figure the three separately built bands have to add up to.
//
// unitArea is the area one unit of radius encloses in whatever sweep the caller
// built: math.Pi for a true revolve, and the regular polygon's own figure for
// the polygonal sweep the solid steps substitute.
func (h hexagon) revolvedVolume(unitArea float64) float64 {
	pts := h.vertices()
	total := 0.0
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		total += (b.station - a.station) * (a.radius*a.radius + a.radius*b.radius + b.radius*b.radius)
	}
	return math.Abs(unitArea * total / 3)
}

// revolvedVolumeClipped is the part of that swept volume that lies inside a
// cylinder of the given radius about the shaft axis — what a through bore of
// that radius takes out of the frustum. The profile is integrated station by
// station with its outer and inner radii each clipped to the bore.
func (h hexagon) revolvedVolumeClipped(unitArea, bore float64) float64 {
	pts := h.vertices()
	const samples = 20000
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, p := range pts {
		lo, hi = math.Min(lo, p.station), math.Max(hi, p.station)
	}
	step := (hi - lo) / samples
	total := 0.0
	for i := 0; i < samples; i++ {
		z := lo + (float64(i)+0.5)*step
		outer, inner := h.radiiAt(z)
		o := math.Min(outer, bore)
		n := math.Min(inner, bore)
		total += (o*o - n*n) * step
	}
	return unitArea * total
}

// radiiAt is the hexagon's outer and inner radius at one station: the crossings
// its closed boundary makes with the line at that station, taken as the largest
// and the smallest.
func (h hexagon) radiiAt(station float64) (outer, inner float64) {
	pts := h.vertices()
	outer, inner = 0, math.Inf(1)
	crossed := false
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		if a.station == b.station {
			continue
		}
		lo, hi := a, b
		if lo.station > hi.station {
			lo, hi = hi, lo
		}
		if station < lo.station || station > hi.station {
			continue
		}
		f := (station - lo.station) / (hi.station - lo.station)
		r := lo.radius + f*(hi.radius-lo.radius)
		outer = math.Max(outer, r)
		inner = math.Min(inner, r)
		crossed = true
	}
	if !crossed {
		return 0, 0
	}
	return outer, inner
}

// backConeHalfAngle is the half-angle of the cone the heel edge and the toe
// line each sweep. Both run along the back cone, which stands perpendicular to
// the pitch line, so each makes 90 degrees minus the pitch cone angle with the
// shaft axis.
func (s bevelSide) backConeHalfAngle() float64 { return math.Pi/2 - s.gamma }

// virtualPitchRadius is the back-cone (Tredgold) pitch radius the §3 spur tooth
// is drawn at.
func (s bevelSide) virtualPitchRadius() float64 { return s.pitchRadius() / math.Cos(s.gamma) }

// toothCentreStation is where K / L sits on the shaft axis: the back cone's own
// apex, which is the centre of the virtual spur gear.
func (d bevelDesign) toothCentreStation(which float64) float64 {
	return d.pitchCone / math.Cos(d.side(which).gamma)
}

// ---------------------------------------------------------------- spiral trace

// spiralTrace is the flat cutter-arc construction of spiral-tooth-trace.md, in
// the cone's tangent plane with the apex at the origin, x along the cone
// element and y circumferential.
type spiralTrace struct {
	rToe, rHeel, rMean, span float64
	cutterRadius             float64
	centreX, centreY         float64
	toeX, toeY               float64
	heelX, heelY             float64
	handSign                 float64
	twist                    float64 // the toe-to-heel shaft-axis twist magnitude
}

// newSpiralTrace builds the trace for one gear. The hand is the driving gear's,
// negated for the pinion so the pair meshes.
func (d bevelDesign) newSpiralTrace(which float64) spiralTrace {
	s := d.side(which)
	h := d.axialProfile(which)
	// The toe and heel cone distances are the root edges' midpoints measured
	// along the root cone element Apex->Ded.
	rToe := alongCone(midpoint(h.toeCone, h.toeIn), s.rootGamma)
	rHeel := alongCone(midpoint(h.ded, h.heelEnd), s.rootGamma)
	t := spiralTrace{rToe: rToe, rHeel: rHeel, rMean: (rToe + rHeel) / 2, span: rHeel - rToe}
	t.cutterRadius = d.cutterRadius
	if t.cutterRadius == 0 {
		t.cutterRadius = t.rMean
	}
	t.handSign = d.hand
	if which == sidePinion {
		t.handSign = -t.handSign
	}
	t.centreX = t.rMean - t.cutterRadius*math.Sin(d.spiralAngle)
	t.centreY = t.handSign * t.cutterRadius * math.Cos(d.spiralAngle)
	t.toeX, t.toeY = circleIntersectNearest(t.rToe-0.06*t.span, t.centreX, t.centreY, t.cutterRadius, t.rMean, 0)
	t.heelX, t.heelY = circleIntersectNearest(t.rHeel+0.06*t.span, t.centreX, t.centreY, t.cutterRadius, t.rMean, 0)
	phiCrown := math.Atan2(t.heelY, t.heelX) - math.Atan2(t.toeY, t.toeX)
	t.twist = math.Abs(phiCrown) / math.Sin(s.gamma)
	return t
}

// segmentAngle is the shaft-axis rotation a slab whose heel face sits at cone
// distance rHeelFace takes, centred on R_mean so the mid-face section stays
// unrotated.
func (t spiralTrace) segmentAngle(rHeelFace float64) float64 {
	return -t.handSign * t.twist * (t.rMean - rHeelFace) / t.span
}

// crownFactor is the lengthwise relief a slab takes, keyed on its monotonic
// heel distance so the taper never reverses.
func (t spiralTrace) crownFactor(rHeelFace float64) float64 {
	u := (t.rHeel - rHeelFace) / t.span
	return 1 - crownPerRad*(math.Abs(t.twist)/2)*u
}

// crownPerRad is the class constant the crown scales by, per radian of per-end
// twist. 0 disables the crown; the shipped gear sets it to 0.5.
const crownPerRad = 0.5

// circleIntersectNearest is the framework helper's arithmetic: the intersection
// of the apex circle of radius r with the cutter circle, taking the solution
// nearest the reference point, and clamping to tangency when the two circles do
// not reach each other.
func circleIntersectNearest(r, cx, cy, rc, refX, refY float64) (float64, float64) {
	dist := math.Hypot(cx, cy)
	if dist == 0 {
		return r, 0
	}
	a := (dist*dist + r*r - rc*rc) / (2 * dist)
	h2 := r*r - a*a
	ux, uy := cx/dist, cy/dist
	baseX, baseY := a*ux, a*uy
	if h2 <= 0 {
		return baseX, baseY
	}
	h := math.Sqrt(h2)
	x1, y1 := baseX-h*uy, baseY+h*ux
	x2, y2 := baseX+h*uy, baseY-h*ux
	if math.Hypot(x1-refX, y1-refY) <= math.Hypot(x2-refX, y2-refY) {
		return x1, y1
	}
	return x2, y2
}

func midpoint(a, b axialPt) axialPt {
	return axialPt{station: (a.station + b.station) / 2, radius: (a.radius + b.radius) / 2}
}

// alongCone is a point's cone distance: its distance from the apex measured
// along the root cone element, which is the projection onto that element.
func alongCone(p axialPt, rootGamma float64) float64 {
	return p.station*math.Cos(rootGamma) + p.radius*math.Sin(rootGamma)
}

// ---------------------------------------------------------------- case tables

// caseParams names one case by the dialog values it comes from. Every length is
// in millimetres and every angle in degrees on the way in; the shaft and spiral
// angles are carried in radians, which is what evaluateExpression hands the
// module back for a `deg` input.
type caseParams struct {
	module                    float64
	drivingTeeth, pinionTeeth float64
	shaftAngleDeg             float64
	drivingBase, pinionBase   float64
	faceWidth                 float64
	toothSpacing              float64
	spiralAngleDeg            float64
	hand                      float64
	cutterRadius              float64
	toeExtension              float64
	drivingToeRadius          float64
	pinionToeRadius           float64
	boreEnable                float64
	drivingBore, pinionBore   float64
	gearSide                  float64
	growSide                  float64
	declaredRefusal           float64
}

func (c caseParams) params() map[string]float64 {
	grow := c.growSide
	if grow == 0 {
		grow = 1
	}
	hand := c.hand
	if hand == 0 {
		hand = handRight
	}
	return map[string]float64{
		"module":            c.module,
		"drivingTeeth":      c.drivingTeeth,
		"pinionTeeth":       c.pinionTeeth,
		"shaftAngle":        inRadians(c.shaftAngleDeg),
		"drivingBaseHeight": c.drivingBase,
		"pinionBaseHeight":  c.pinionBase,
		"faceWidth":         c.faceWidth,
		"toothSpacing":      c.toothSpacing,
		"spiralAngle":       inRadians(c.spiralAngleDeg),
		"spiralHand":        hand,
		"cutterRadius":      c.cutterRadius,
		"toeExtension":      c.toeExtension,
		"drivingToeRadius":  c.drivingToeRadius,
		"pinionToeRadius":   c.pinionToeRadius,
		"boreEnable":        c.boreEnable,
		"drivingBore":       c.drivingBore,
		"pinionBore":        c.pinionBore,
		"gearSide":          c.gearSide,
		"growSide":          grow,
		"declaredRefusal":   c.declaredRefusal,
	}
}

func inRadians(deg float64) float64 { return deg * math.Pi / 180 }

// base is the shipped dialog defaults, at the Module the table in hand wants.
func defaultCase(module float64) caseParams {
	return caseParams{
		module:         module,
		drivingTeeth:   31,
		pinionTeeth:    31,
		shaftAngleDeg:  90,
		spiralAngleDeg: 35,
		hand:           handRight,
		boreEnable:     1,
	}
}

func (c caseParams) with(f func(*caseParams)) caseParams {
	f(&c)
	return c
}

// latticeCases is the regime the §2 constraint scheme has to hold across.
//
// The Shaft Angle sweep runs from the spec's documented 30 degree floor to well
// above the perpendicular default, because the lattice's conditioning
// approaches the engine's trust floor at BOTH ends and the spec is explicit
// that where this particular net cannot reach a configuration the spec admits,
// the case stays in the table as a declared refusal rather than the range being
// narrowed. Ratio pairs run both ways round, since either gear can carry the
// smaller tooth count and the Maximum Face Width binds on whichever does. The
// grow side is swept because the sign of the in-plane perpendicular is chosen
// by the target plane's normal, and a scheme that only holds on one side is a
// scheme that mirrors on half the target planes.
var latticeCases = []proofkit.Case{
	{Name: "default_31_31_90deg", Params: defaultCase(1).params()},
	{Name: "default_grow_side_reversed", Params: defaultCase(1).with(func(c *caseParams) { c.growSide = -1 }).params()},
	{Name: "shaft_35deg", Params: defaultCase(1).with(func(c *caseParams) { c.shaftAngleDeg = 35 }).params()},
	{Name: "shaft_60deg", Params: defaultCase(1).with(func(c *caseParams) { c.shaftAngleDeg = 60 }).params()},
	{Name: "shaft_120deg", Params: defaultCase(1).with(func(c *caseParams) { c.shaftAngleDeg = 120 }).params()},
	{Name: "shaft_142deg", Params: defaultCase(1).with(func(c *caseParams) { c.shaftAngleDeg = 142 }).params()},
	{Name: "shaft_150deg", Params: defaultCase(1).with(func(c *caseParams) { c.shaftAngleDeg = 150 }).params()},
	{Name: "ratio_31_17", Params: defaultCase(1).with(func(c *caseParams) { c.pinionTeeth = 17 }).params()},
	{Name: "ratio_17_31", Params: defaultCase(1).with(func(c *caseParams) { c.drivingTeeth = 17 }).params()},
	{Name: "ratio_19_13_module2", Params: defaultCase(2).with(func(c *caseParams) {
		c.drivingTeeth, c.pinionTeeth = 19, 13
	}).params()},
	{Name: "low_tooth_count_4_4", Params: defaultCase(1).with(func(c *caseParams) {
		c.drivingTeeth, c.pinionTeeth = 4, 4
	}).params()},
	{Name: "toe_extension_50", Params: defaultCase(1).with(func(c *caseParams) { c.toeExtension = 50 }).params()},
	{Name: "toe_extension_100", Params: defaultCase(1).with(func(c *caseParams) { c.toeExtension = 100 }).params()},
	{Name: "toe_extension_100_ratio_31_17", Params: defaultCase(1).with(func(c *caseParams) {
		c.pinionTeeth, c.toeExtension = 17, 100
	}).params()},
	{Name: "toe_radius_user_set", Params: defaultCase(1).with(func(c *caseParams) {
		c.drivingToeRadius, c.pinionToeRadius = 3, 2
	}).params()},
	{Name: "toe_radius_user_set_with_extension", Params: defaultCase(1).with(func(c *caseParams) {
		c.drivingToeRadius, c.pinionToeRadius, c.toeExtension = 3, 2, 60
	}).params()},
	{Name: "tooth_spacing_positive", Params: defaultCase(1).with(func(c *caseParams) { c.toothSpacing = 0.3 }).params()},
	{Name: "base_heights_user_set", Params: defaultCase(1).with(func(c *caseParams) {
		c.drivingBase, c.pinionBase = 6, 2
	}).params()},
	{Name: "face_width_user_set", Params: defaultCase(1).with(func(c *caseParams) { c.faceWidth = 4 }).params()},
	{Name: "shaft_30deg_declared_refusal", Params: defaultCase(1).with(func(c *caseParams) {
		c.shaftAngleDeg, c.declaredRefusal = 30, 1
	}).params()},
}

// solidCases run at Module 4 to 8, never at Module 1: decad's mesh bound has an
// absolute floor, so a figure small enough brings every measurement inside it
// and the gate reports Suspect on geometry that is in fact correct. Module is a
// pure scale on this figure, so a case at Module 4 proves the same shape as one
// at Module 1 and clears the floor. Both sides of the pair are built, because
// the two differ in pitch cone angle wherever the tooth counts do and every
// per-gear step is run twice by the module.
var solidCases = []proofkit3d.Case{
	{Name: "pinion_M4_31_31", Params: defaultCase(4).params()},
	{Name: "driving_M4_31_31", Params: defaultCase(4).with(func(c *caseParams) { c.gearSide = sideDriving }).params()},
	{Name: "pinion_M4_31_17", Params: defaultCase(4).with(func(c *caseParams) { c.pinionTeeth = 17 }).params()},
	{Name: "driving_M4_31_17", Params: defaultCase(4).with(func(c *caseParams) {
		c.pinionTeeth, c.gearSide = 17, sideDriving
	}).params()},
	{Name: "pinion_M8_17_31_shaft_60deg", Params: defaultCase(8).with(func(c *caseParams) {
		c.drivingTeeth, c.shaftAngleDeg = 17, 60
	}).params()},
	{Name: "driving_M8_17_31_shaft_120deg", Params: defaultCase(8).with(func(c *caseParams) {
		c.drivingTeeth, c.shaftAngleDeg, c.gearSide = 17, 120, sideDriving
	}).params()},
	{Name: "pinion_M6_toe_extension_100", Params: defaultCase(6).with(func(c *caseParams) { c.toeExtension = 100 }).params()},
	{Name: "driving_M6_no_bore", Params: defaultCase(6).with(func(c *caseParams) {
		c.boreEnable, c.gearSide = 0, sideDriving
	}).params()},
}

// spiralCases add the Mean Spiral Angle and the Hand of Spiral to the solid
// regime. Spiral Angle 0 is the straight bevel, which takes the other branch
// entirely and is proved by the straight steps; every case here is above it,
// at both hands, because opposite hands must be exact mirror images for an
// equal-teeth pair and the two members of a ratio pair legitimately get
// different twists.
var spiralCases = []proofkit3d.Case{
	{Name: "pinion_M4_31_31_right_35deg", Params: defaultCase(4).params()},
	{Name: "driving_M4_31_31_right_35deg", Params: defaultCase(4).with(func(c *caseParams) { c.gearSide = sideDriving }).params()},
	{Name: "pinion_M4_31_31_left_35deg", Params: defaultCase(4).with(func(c *caseParams) { c.hand = handLeft }).params()},
	{Name: "driving_M4_31_31_left_35deg", Params: defaultCase(4).with(func(c *caseParams) {
		c.hand, c.gearSide = handLeft, sideDriving
	}).params()},
	{Name: "pinion_M4_31_17_right_20deg", Params: defaultCase(4).with(func(c *caseParams) {
		c.pinionTeeth, c.spiralAngleDeg = 17, 20
	}).params()},
	{Name: "driving_M4_31_17_right_20deg", Params: defaultCase(4).with(func(c *caseParams) {
		c.pinionTeeth, c.spiralAngleDeg, c.gearSide = 17, 20, sideDriving
	}).params()},
	{Name: "pinion_M6_55deg_cutter_set", Params: defaultCase(6).with(func(c *caseParams) {
		c.spiralAngleDeg, c.cutterRadius = 55, 40
	}).params()},
	{Name: "driving_M6_55deg_cutter_set", Params: defaultCase(6).with(func(c *caseParams) {
		c.spiralAngleDeg, c.cutterRadius, c.gearSide = 55, 40, sideDriving
	}).params()},
}
