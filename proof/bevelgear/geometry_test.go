// Package bevelgear_test proves the bevel gear pair described by
// spec/bevelgear/instructions.md, its Fusion sidecar spec/bevelgear/fusion.md
// and the trace derivation spec/bevelgear/spiral-tooth-trace.md.
//
// Units. The Fusion module works in Fusion internal units (centimetres) and
// reads Module as a raw millimetre number. Both engines used here work in
// millimetres, so the proof works in millimetres throughout and the cm
// conversions the generator has to make are carried in the step list rather
// than modelled here. Angles are radians unless a name says otherwise.
//
// Frames. Every gear is proved in its own frame: the origin is the shared
// Apex, the first coordinate ("along") runs down that gear's shaft axis away
// from the Apex, and the second ("radial") is the perpendicular distance from
// that axis. The two gears then have the same closed form with their own
// (gamma, pitch diameter, base height) substituted, which is why one lattice
// function serves both. The Gear Profiles sketch holds both gears at once, so
// the lattice proof works in the shared sketch frame instead and converts.
package bevelgear_test

import (
	"fmt"
	"math"
	"testing"
)

// ---------------------------------------------------------------------------
// Case parameter keys. A case table is a []proofkit.Case / []proofkit3d.Case,
// and both carry map[string]float64, so every input is a float64 here — the
// two booleans included.
// ---------------------------------------------------------------------------

const (
	keyModule       = "module"        // raw mm, the dialog's unitless Module
	keyDrivingTeeth = "drivingTeeth"  // Driving Gear Teeth
	keyPinionTeeth  = "pinionTeeth"   // Pinion Gear Teeth
	keyShaftAngle   = "shaftAngleDeg" // Shaft Angle, degrees
	keyDrivingBase  = "drivingBaseHeight"
	keyPinionBase   = "pinionBaseHeight"
	keyFaceWidth    = "faceWidth"
	keyToothSpacing = "toothSpacing"
	keySpiralAngle  = "spiralAngleDeg" // Mean Spiral Angle psi, degrees
	keyHand         = "hand"           // +1 Right, -1 Left (the DRIVING gear's hand)
	keyCutterRadius = "cutterRadius"
	keyBoreEnable   = "boreEnable" // 1 enabled, 0 disabled
	keyDrivingBore  = "drivingBore"
	keyPinionBore   = "pinionBore"

	// The keyExpect* keys carry the values instructions.md publishes, so a
	// worked case in the table states the number the spec states and the proof
	// checks the resolution against it rather than against itself.
	keyExpectMaxBaseHeight   = "expectMaxBaseHeight"
	keyExpectTrueCrossing    = "expectTrueCrossing"
	keyExpectDrivingFallback = "expectDrivingFallback"
	keyExpectResolvedBase    = "expectResolvedBase"
	keyExpectMaxShaftAngle   = "expectMaxShaftAngle"
	keyExpectShaftExclusive  = "expectShaftExclusive"
	keyExpectMinTeeth        = "expectMinTeeth"

	// keyNetLimited marks a case this particular section 2 net cannot reach
	// within the sketch engine's conditioning floor. It is a property of the
	// net, never a bound on the input — see the note on latticeCases.
	keyNetLimited = "netLimited"
)

// tightTol is the tolerance for a quantity two closed forms should agree on
// exactly; slackTol is for one that has been through the constraint solver,
// whose convergence tolerance is 1e-10 on the residual.
const (
	tightTol = 1e-9
	slackTol = 1e-7
)

// ---------------------------------------------------------------------------
// Plane vectors. The lattice is a plane figure, so the proof carries its own
// two-component vector rather than borrowing r3.Vec and leaving a zero z to be
// read as meaningful.
// ---------------------------------------------------------------------------

type vec2 struct{ X, Y float64 }

func v2(x, y float64) vec2            { return vec2{x, y} }
func (a vec2) add(b vec2) vec2        { return vec2{a.X + b.X, a.Y + b.Y} }
func (a vec2) sub(b vec2) vec2        { return vec2{a.X - b.X, a.Y - b.Y} }
func (a vec2) scale(k float64) vec2   { return vec2{a.X * k, a.Y * k} }
func (a vec2) dot(b vec2) float64     { return a.X*b.X + a.Y*b.Y }
func (a vec2) cross(b vec2) float64   { return a.X*b.Y - a.Y*b.X }
func (a vec2) len() float64           { return math.Hypot(a.X, a.Y) }
func (a vec2) distanceTo(b vec2) float64 { return a.sub(b).len() }

func (a vec2) unit() vec2 {
	n := a.len()
	if n == 0 {
		return vec2{}
	}
	return a.scale(1 / n)
}

// leftNormal is the +90 degree rotation of a. The sketch engine's Offset
// dimension is signed with positive on the LEFT of the source line's
// start-to-end direction, so this is the vector the proof's offset signs are
// read against.
func (a vec2) leftNormal() vec2 { return vec2{-a.Y, a.X} }

func rotate2(a vec2, ang float64) vec2 {
	s, c := math.Sin(ang), math.Cos(ang)
	return vec2{a.X*c - a.Y*s, a.X*s + a.Y*c}
}

// signedDistanceFromLine is the signed perpendicular distance of p from the
// infinite line through from -> to, positive on the left of that direction.
// It is the quantity a signed offset dimension drives.
func signedDistanceFromLine(from, to, p vec2) float64 {
	d := to.sub(from).unit()
	return p.sub(from).dot(d.leftNormal())
}

// ---------------------------------------------------------------------------
// Input resolution — instructions.md "Variables".
//
// Everything here is closed form: the Fusion generator resolves all of it in
// _readInputs before any geometry exists, except the Maximum Face Width, which
// instructions.md pins to the SOLVED sketch geometry of A, B, C, D, H and J
// ([PB-SOLVED-GEOMETRY]). The proof computes the face-width cap from the same
// closed form AND checks it against the solved lattice, which is what makes
// the two readings comparable.
// ---------------------------------------------------------------------------

// gear is one member of the pair: everything the build needs that differs
// between pinion and driving.
type gear struct {
	Label         string  // "Pinion" or "Driving"
	Teeth         float64 // this gear's tooth count
	PitchDiameter float64 // Module * Teeth, mm
	Gamma         float64 // pitch cone half angle, radians
	BaseHeight    float64 // RESOLVED base height, mm
	Bore          float64 // RESOLVED bore diameter, mm (0 when Enable Bore is off)
}

// design holds every resolved value of one case.
type design struct {
	Module       float64
	Sigma        float64 // Shaft Angle, radians
	R            float64 // Pitch Cone Distance: (PPD/2) / sin gamma_p
	ConeDistance float64 // sqrt(DPD^2 + PPD^2) — the OTHER length, the diagonal
	FaceWidth    float64 // resolved, already capped by MaximumFaceWidth
	ToothSpacing float64
	Psi          float64 // Mean Spiral Angle, radians
	HandSign     float64 // +1 Right, -1 Left, as read for the DRIVING gear
	CutterRadius float64 // raw input, 0 meaning auto
	BoreEnable   bool

	// Bounds is every closed-form limit this design was resolved against.
	Bounds bounds

	Pinion  gear
	Driving gear
}

// coneAngles is the closed form instructions.md section 2 seeds the lattice
// with: tan gamma_p = sin Sigma * PPD / (DPD + PPD cos Sigma), gamma_g =
// Sigma - gamma_p.
func coneAngles(sigma, drivingPitchDia, pinionPitchDia float64) (gammaP, gammaG float64) {
	gammaP = math.Atan2(math.Sin(sigma)*pinionPitchDia,
		drivingPitchDia+pinionPitchDia*math.Cos(sigma))
	return gammaP, sigma - gammaP
}

// maximumShaftAngleDeg is the cone-angle singularity limit capped at 150.
// The cone-angle half is EXCLUSIVE (acos is a hard singularity there) and the
// 150 half is inclusive, so the caller compares with < against the first and
// <= against the second; maximumShaftAngleExclusive reports which half won.
func maximumShaftAngleDeg(drivingPitchDia, pinionPitchDia float64) (limit float64, exclusive bool) {
	smaller := math.Min(drivingPitchDia, pinionPitchDia)
	larger := math.Max(drivingPitchDia, pinionPitchDia)
	cone := math.Acos(-smaller/larger) * 180 / math.Pi
	if cone <= 150 {
		return cone, true
	}
	return 150, false
}

// minimumBaseHeight and maximumBaseHeight are the two ends of one gear's heel
// edge window. Both are closed form in r, gamma and Module, so both resolve
// during input validation.
func minimumBaseHeight(module, gamma float64) float64 {
	return 1.05 * 1.25 * module * math.Sin(gamma)
}

func maximumBaseHeight(module, pitchRadius, gamma float64) float64 {
	return 0.95 * (pitchRadius - 1.25*module*math.Cos(gamma)) * math.Tan(gamma)
}

// trueHeelCrossing is the base height at which the heel point H (resp. J)
// reaches the shaft axis and the revolved profile folds onto its own axis of
// revolution. maximumBaseHeight sits 1.25*Module*sin(gamma) below it, which is
// the conservatism instructions.md declares.
func trueHeelCrossing(pitchRadius, gamma float64) float64 {
	return pitchRadius * math.Tan(gamma)
}

// minimumTeeth is the count below which the base-height window is empty:
// 5.27 * cos(gamma), with 5.27 = 2 * (1.05*1.25/0.95 + 1.25).
func minimumTeeth(gamma float64) float64 { return 5.27 * math.Cos(gamma) }

// resolveBaseHeight applies one gear's window to one base height. raw is the
// dialog value (0 meaning unspecified) and fallback is the value to use in its
// place. It returns the resolved height and, when the user's own value broke a
// bound, the side it broke — the generator rejects there rather than clamping.
func resolveBaseHeight(raw, fallback, minimum, maximum float64) (resolved float64, rejected string) {
	if raw > 0 {
		switch {
		case raw < minimum:
			return raw, "below minimum"
		case raw > maximum:
			return raw, "above maximum"
		}
		return raw, ""
	}
	return math.Min(math.Max(fallback, minimum), maximum), ""
}

// maximumFaceWidth is 0.95 times the smaller of the perpendicular distance
// from A to line C-H and from B to line D-J. Both distances reduce to
// R * sin(gamma)^2 for the gear in question, so the binding side is the gear
// with the smaller pitch diameter. Kept as its own function so the lattice
// proof can compare it against the same quantity measured off solved points.
func maximumFaceWidth(r, gammaP, gammaG float64) float64 {
	sp := math.Sin(gammaP)
	sg := math.Sin(gammaG)
	return 0.95 * r * math.Min(sp*sp, sg*sg)
}

// virtualTeeth is the back-cone (Tredgold) tooth number this gear's spur tooth
// is drawn with: floor(2 * virtualPitchRadius / Module) with virtualPitchRadius
// = (PitchDiameter/2) / cos(gamma). It is independent of Tooth Spacing.
func virtualTeeth(module, pitchDiameter, gamma float64) int {
	return int(math.Floor(2 * virtualPitchRadius(pitchDiameter, gamma) / module))
}

func virtualPitchRadius(pitchDiameter, gamma float64) float64 {
	return (pitchDiameter / 2) / math.Cos(gamma)
}

// bounds is every closed-form limit the Variables section defines, kept beside
// the design that was resolved against it so a proof can read the bound as well
// as the value it produced.
type bounds struct {
	MaxShaftAngleDeg    float64 // the cone-angle singularity, capped at 150
	ShaftAngleExclusive bool    // true when the cone-angle half won, so the limit itself is refused

	MinTeethPinion  float64 // 5.27 * cos(gamma), per gear, on top of the blanket 3
	MinTeethDriving float64

	MinBaseHeightPinion  float64
	MaxBaseHeightPinion  float64
	TrueCrossingPinion   float64 // r * tan(gamma): where the heel point reaches the axis
	MinBaseHeightDriving float64
	MaxBaseHeightDriving float64
	TrueCrossingDriving  float64

	DrivingFallback float64 // Module * Driving Gear Teeth Number / 8
	PinionFallback  float64 // the RESOLVED driving height * pinion teeth / driving teeth

	MaxFaceWidth      float64
	FaceWidthFallback float64 // Cone Distance / 6
}

// resolveDesign resolves one case the way _readInputs plus the section 2
// face-width step do, in the order instructions.md fixes: cone angles, then the
// Minimum Teeth floor, then each gear's base-height window, then the face
// width.
//
// It RETURNS its rejections rather than raising, because a rejection is a
// result the bounds step has to be able to assert. Every problem names the
// offending input and the numeric bound it broke, which is what the generator's
// message has to do. Resolution continues past a rejection so the caller still
// gets the bounds that were computed.
func resolveDesign(p map[string]float64) (design, []string) {
	var problems []string
	reject := func(format string, args ...any) {
		problems = append(problems, fmt.Sprintf(format, args...))
	}

	module := p[keyModule]
	nd := p[keyDrivingTeeth]
	np := p[keyPinionTeeth]
	dpd := module * nd
	ppd := module * np
	sigmaDeg := p[keyShaftAngle]

	if module <= 0 {
		reject("Module must be positive, got %g", module)
	}
	if nd < 3 {
		reject("Driving Gear Teeth must be at least 3, got %g", nd)
	}
	if np < 3 {
		reject("Pinion Gear Teeth must be at least 3, got %g", np)
	}

	limit, exclusive := maximumShaftAngleDeg(dpd, ppd)
	if sigmaDeg < 30 {
		reject("Shaft Angle %g deg is below the 30 deg floor", sigmaDeg)
	}
	if (exclusive && sigmaDeg >= limit) || (!exclusive && sigmaDeg > limit) {
		reject("Shaft Angle %g deg is not below the Maximum Shaft Angle %.4f deg", sigmaDeg, limit)
	}

	sigma := sigmaDeg * math.Pi / 180
	gammaP, gammaG := coneAngles(sigma, dpd, ppd)
	r := (ppd / 2) / math.Sin(gammaP)

	b := bounds{
		MaxShaftAngleDeg:    limit,
		ShaftAngleExclusive: exclusive,
		MinTeethPinion:      minimumTeeth(gammaP),
		MinTeethDriving:     minimumTeeth(gammaG),
		MinBaseHeightPinion: minimumBaseHeight(module, gammaP),
		MaxBaseHeightPinion: maximumBaseHeight(module, ppd/2, gammaP),
		TrueCrossingPinion:  trueHeelCrossing(ppd/2, gammaP),
		MinBaseHeightDriving: minimumBaseHeight(module, gammaG),
		MaxBaseHeightDriving: maximumBaseHeight(module, dpd/2, gammaG),
		TrueCrossingDriving:  trueHeelCrossing(dpd/2, gammaG),
		DrivingFallback:      module * nd / 8,
	}
	if np < b.MinTeethPinion {
		reject("Pinion Gear Teeth %g is below the computed floor %.3f", np, b.MinTeethPinion)
	}
	if nd < b.MinTeethDriving {
		reject("Driving Gear Teeth %g is below the computed floor %.3f", nd, b.MinTeethDriving)
	}

	drivingBase, rejected := resolveBaseHeight(
		p[keyDrivingBase], b.DrivingFallback, b.MinBaseHeightDriving, b.MaxBaseHeightDriving)
	switch rejected {
	case "below minimum":
		reject("Driving Gear Base Height %g mm is below the Minimum Base Height %.4f mm",
			p[keyDrivingBase], b.MinBaseHeightDriving)
	case "above maximum":
		reject("Driving Gear Base Height %g mm is above the Maximum Base Height %.4f mm",
			p[keyDrivingBase], b.MaxBaseHeightDriving)
	}
	// The pinion fallback scales the RESOLVED driving height by the tooth
	// ratio, then takes the pinion's OWN window: the two gears share no cone
	// angle unless the counts are equal.
	b.PinionFallback = drivingBase * np / nd
	pinionBase, rejected := resolveBaseHeight(
		p[keyPinionBase], b.PinionFallback, b.MinBaseHeightPinion, b.MaxBaseHeightPinion)
	switch rejected {
	case "below minimum":
		reject("Pinion Gear Base Height %g mm is below the Minimum Base Height %.4f mm",
			p[keyPinionBase], b.MinBaseHeightPinion)
	case "above maximum":
		reject("Pinion Gear Base Height %g mm is above the Maximum Base Height %.4f mm",
			p[keyPinionBase], b.MaxBaseHeightPinion)
	}

	coneDistance := math.Hypot(dpd, ppd)
	b.MaxFaceWidth = maximumFaceWidth(r, gammaP, gammaG)
	b.FaceWidthFallback = coneDistance / 6
	faceWidth := p[keyFaceWidth]
	if faceWidth <= 0 {
		faceWidth = math.Min(b.FaceWidthFallback, b.MaxFaceWidth)
	} else if faceWidth > b.MaxFaceWidth {
		reject("Face Width %g mm exceeds the Maximum Face Width %.4f mm", faceWidth, b.MaxFaceWidth)
	}

	psiDeg := p[keySpiralAngle]
	if psiDeg < 0 || psiDeg >= 60 {
		reject("Mean Spiral Angle %g deg is outside [0, 60)", psiDeg)
	}
	if p[keyCutterRadius] < 0 {
		reject("Cutter Radius %g mm is negative", p[keyCutterRadius])
	}
	if p[keyToothSpacing] < 0 {
		reject("Tooth Spacing %g mm is negative", p[keyToothSpacing])
	}

	hand := p[keyHand]
	if hand == 0 {
		hand = 1
	}
	boreEnable := p[keyBoreEnable] != 0

	return design{
		Module:       module,
		Sigma:        sigma,
		R:            r,
		ConeDistance: coneDistance,
		FaceWidth:    faceWidth,
		ToothSpacing: p[keyToothSpacing],
		Psi:          psiDeg * math.Pi / 180,
		HandSign:     hand,
		CutterRadius: p[keyCutterRadius],
		BoreEnable:   boreEnable,
		Bounds:       b,
		Pinion: gear{
			Label: "Pinion", Teeth: np, PitchDiameter: ppd,
			Gamma: gammaP, BaseHeight: pinionBase,
			Bore: resolveBore(boreEnable, p[keyPinionBore], ppd),
		},
		Driving: gear{
			Label: "Driving", Teeth: nd, PitchDiameter: dpd,
			Gamma: gammaG, BaseHeight: drivingBase,
			Bore: resolveBore(boreEnable, p[keyDrivingBore], dpd),
		},
	}, problems
}

// newDesign is resolveDesign for the steps that need a case the generator would
// accept: a rejection here is a case table defect, so it fails the case.
func newDesign(t testing.TB, p map[string]float64) design {
	t.Helper()
	d, problems := resolveDesign(p)
	for _, problem := range problems {
		t.Errorf("input validation rejected this case: %s", problem)
	}
	if len(problems) > 0 {
		t.Fatalf("%d input problem(s); this case is not one the generator would build", len(problems))
	}
	return d
}

// resolveBore is the "0 means auto" rule: a bore diameter of 0 becomes this
// gear's Pitch Diameter / 4, and no bore at all when Enable Bore is off.
func resolveBore(enabled bool, raw, pitchDiameter float64) float64 {
	if !enabled {
		return 0
	}
	if raw > 0 {
		return raw
	}
	return pitchDiameter / 4
}

// pinionMeshPhase is the pinion's extra rotation about its own shaft axis, in
// radians: _PINION_MESH_PHASE_TEETH tooth-fractions, and that constant is 0.
// The driving gear's own meshing rotation is half a tooth pitch and lives with
// its step.
const pinionMeshPhaseTeeth = 0.0

func pinionMeshPhase(pinionTeeth float64) float64 {
	return pinionMeshPhaseTeeth * 2 * math.Pi / pinionTeeth
}

// crownPerRad is _CROWN_PER_RAD, the tunable class constant section 3a step H
// fixes at 0.5 (0 would disable the crown).
const crownPerRad = 0.5

// ---------------------------------------------------------------------------
// The section 2 lattice, in closed form.
//
// gearFrame is one gear's half of the figure in that gear's own frame: Along
// runs from the Apex down the shaft axis, Radial is the perpendicular distance
// from that axis. Both gears take the same form, which is why the two halves
// of section 2 read as mirror text.
// ---------------------------------------------------------------------------

type gearFrame struct {
	Apex  vec2 // (0, 0)
	Axis  vec2 // A (pinion) / B (driving): the pitch point on the shaft axis
	Base  vec2 // G (pinion) / I (driving): the heel end of the shaft edge
	Apex2 vec2 // shared with the other gear, in THIS gear's frame
	Ded   vec2 // C (pinion) / D (driving): the dedendum corner
	Heel  vec2 // H (pinion) / J (driving): the heel outer corner
	Toe   vec2 // M (pinion) / O (driving): the toe corner on the root element
	ToeIn vec2 // N (pinion) / P (driving): the toe corner on the drop line
	Foot  vec2 // E (pinion) / F (driving): the foot of the dedendum perpendicular
	Back  vec2 // K (pinion) / L (driving): the back-cone apex on the shaft axis
	Tooth vec2 // K' (pinion) / L' (driving): the tooth centre after Tooth Spacing
	// DedDir is the unit dedendum direction Apex2 -> Ded -> Heel in this frame.
	DedDir vec2
}

// gearLattice is the closed form of one gear's half of section 2, derived from
// the constraint net rather than from the seeds: A/B sit at R*cos(gamma) along
// the axis, Apex2 at pitch radius above that station, the dedendum line leaves
// Apex2 perpendicular to the pitch line, G/I sit one resolved base height
// beyond A/B, and the toe line is the dedendum line offset by the Face Width
// toward the Apex.
func gearLattice(g gear, module, r, faceWidth, toothSpacing float64) gearFrame {
	sin, cos, tan := math.Sin(g.Gamma), math.Cos(g.Gamma), math.Tan(g.Gamma)
	pitchRadius := g.PitchDiameter / 2
	along := r * cos

	// The dedendum direction runs outward along the back cone: further from
	// the Apex along the shaft, closer to the shaft axis.
	ded := v2(sin, -cos)

	apex := v2(0, 0)
	axis := v2(along, 0)
	base := v2(along+g.BaseHeight, 0)
	apex2 := v2(along, pitchRadius)
	dedendum := apex2.add(ded.scale(1.25 * module))
	heel := apex2.add(ded.scale(g.BaseHeight / sin))
	foot := v2(along+1.25*module*sin, 0)
	back := apex2.add(ded.scale(pitchRadius / cos))

	// M/O lie on the root element Apex->Ded at the fraction that puts them one
	// Face Width from the dedendum line, measured perpendicular to it. The
	// Apex is exactly R from that line, so the fraction is 1 - FaceWidth/R.
	toe := dedendum.scale(1 - faceWidth/r)
	// N/P lie on the Apex2 drop line (the radial line through Axis and Apex2)
	// at the same perpendicular distance.
	toeIn := v2(along, pitchRadius-faceWidth/sin)

	tooth := back
	if toothSpacing > 0 {
		tooth = back.add(ded.scale(toothSpacing))
	}
	_ = tan
	return gearFrame{
		Apex: apex, Axis: axis, Base: base, Apex2: apex2,
		Ded: dedendum, Heel: heel, Toe: toe, ToeIn: toeIn,
		Foot: foot, Back: back, Tooth: tooth, DedDir: ded,
	}
}

// hexagon is the frustum profile the gear body is revolved from, in the draw
// order instructions.md fixes: A -> G -> H -> C -> M -> N -> A for the pinion
// and B -> I -> J -> D -> O -> P -> B for the driving gear. Its FIRST edge is
// the shaft axis every later body operation uses.
func (f gearFrame) hexagon() []vec2 {
	return []vec2{f.Axis, f.Base, f.Heel, f.Ded, f.Toe, f.ToeIn}
}

// sharedFrame maps this gear's own frame into the shared Gear Profiles sketch
// frame, where the projected anchor centre is the origin, +X is the projected
// anchor line direction and +Y is the grow side chosen from the target-plane
// normal ([BEVEL-F-GROW-SIDE]).
type sharedFrame struct {
	Apex   vec2
	AxisIn vec2 // unit Apex -> A (pinion) / Apex -> B (driving)
	RadIn  vec2 // unit direction of increasing radial distance
}

// ---------------------------------------------------------------------------
// The spiral trace, section 3a steps A and B and spiral-tooth-trace.md.
// ---------------------------------------------------------------------------

// traceFrame holds the cone-distance marks step A derives and the cutter-circle
// geometry step B places, all in the tangent-plane 2-D frame whose origin is
// the Apex, whose x is the cone element and whose y is circumferential.
type traceFrame struct {
	RToe, RHeel, RMean, Span float64
	CutterRadius             float64 // r_c after the auto default
	HandSign                 float64 // this gear's hand, pinion already negated
	Centre                   vec2    // the cutter-circle centre (Cx, Cy)
	Toe2D, Heel2D            vec2    // the kept arc's toe and heel endpoints
	PhiCrown                 float64 // developed azimuth subtended at the Apex
	Total                    float64 // toe-to-heel shaft-axis twist magnitude
}

// newTraceFrame realises steps A and B for one gear. rToe and rHeel are the
// cone distances of the toe and heel edge MIDpoints; the swap guard section 3a
// step A demands is applied here, so a caller that hands them over in the wrong
// order still gets a positive span.
func newTraceFrame(d design, g gear, rToe, rHeel float64) traceFrame {
	if rHeel < rToe {
		rToe, rHeel = rHeel, rToe
	}
	mean := (rToe + rHeel) / 2
	span := rHeel - rToe

	rc := d.CutterRadius
	if rc == 0 {
		rc = mean
	}
	hand := d.HandSign
	if g.Label == "Pinion" {
		hand = -hand
	}

	// The hand sign belongs on the cos (y) term. Putting it on the sin (x)
	// term mirrors the centre about x = R_mean instead of about the cone
	// element, which is a different curve and gives the pair unequal twist.
	centre := v2(mean-rc*math.Sin(d.Psi), hand*rc*math.Cos(d.Psi))

	lo := rToe - 0.06*span
	hi := rHeel + 0.06*span
	toe2D := circleIntersectNearest(lo, centre, rc, v2(mean, 0))
	heel2D := circleIntersectNearest(hi, centre, rc, v2(mean, 0))

	phi := math.Atan2(heel2D.Y, heel2D.X) - math.Atan2(toe2D.Y, toe2D.X)
	return traceFrame{
		RToe: rToe, RHeel: rHeel, RMean: mean, Span: span,
		CutterRadius: rc, HandSign: hand, Centre: centre,
		Toe2D: toe2D, Heel2D: heel2D,
		PhiCrown: phi,
		Total:    math.Abs(phi) / math.Sin(g.Gamma),
	}
}

// circleIntersectNearest is solids.circle_intersect_nearest: the intersection
// of the apex circle of radius r with the cutter circle, kept on the branch the
// mean point sits on. A non-overlapping pair clamps to tangency.
func circleIntersectNearest(r float64, centre vec2, cutter float64, ref vec2) vec2 {
	dist := centre.len()
	if dist == 0 {
		return v2(r, 0)
	}
	// Standard circle-circle: a is the distance from the apex to the radical
	// line along the centre direction, h the half-chord.
	a := (dist*dist + r*r - cutter*cutter) / (2 * dist)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0 // tangency clamp
	}
	h := math.Sqrt(h2)
	dir := centre.unit()
	perp := dir.leftNormal()
	base := dir.scale(a)
	c1 := base.add(perp.scale(h))
	c2 := base.sub(perp.scale(h))
	if c1.distanceTo(ref) <= c2.distanceTo(ref) {
		return c1
	}
	return c2
}

// segmentTwist is section 3a step G's per-segment rotation: a linear share of
// the total keyed to the cone distance of the segment's HEEL FACE, centred on
// R_mean so the mid-face section stays unrotated.
func segmentTwist(tf traceFrame, heelFaceDistance float64) float64 {
	return -tf.HandSign * tf.Total * (tf.RMean - heelFaceDistance) / tf.Span
}

// crownFactor is section 3a step H: relief growing monotonically from the held
// heel to the toe, keyed to the heel-distance fraction u, never on |ang|.
func crownFactor(tf traceFrame, heelFaceDistance float64) float64 {
	u := (tf.RHeel - heelFaceDistance) / tf.Span
	return 1 - crownPerRad*(math.Abs(tf.Total)/2)*u
}

// ---------------------------------------------------------------------------
// Small assertion helpers.
// ---------------------------------------------------------------------------

// requireClose compares two scalars with a tolerance that scales with the
// expected magnitude, for the same reason requireCloseVec does.
func requireClose(t testing.TB, got, want, tol float64, what string, args ...any) {
	t.Helper()
	if scale := math.Abs(want); scale > 1 {
		tol *= scale
	}
	if math.Abs(got-want) > tol {
		t.Fatalf("%s: got %.12g, want %.12g (tolerance %g)",
			fmt.Sprintf(what, args...), got, want, tol)
	}
}

// requireCloseVec compares two plane points with a tolerance that scales with
// the point's own magnitude: the lattice reaches a few hundred millimetres from
// the Apex at the top of the Shaft Angle range, where an absolute 1e-7 mm is
// below the solver's own convergence noise.
// sprintfArgs formats an assertion label, tolerating one with no verbs.
func sprintfArgs(format string, args ...any) string {
	if len(args) == 0 {
		return format
	}
	return fmt.Sprintf(format, args...)
}

func requireCloseVec(t testing.TB, got, want vec2, tol float64, what string, args ...any) {
	t.Helper()
	if scale := want.len(); scale > 1 {
		tol *= scale
	}
	if got.distanceTo(want) > tol {
		t.Fatalf("%s: got (%.12g, %.12g), want (%.12g, %.12g) (tolerance %g)",
			fmt.Sprintf(what, args...), got.X, got.Y, want.X, want.Y, tol)
	}
}
