package bevelgear_test

// The closed form spec/bevelgear/instructions.md fixes, in one place.
//
// Every step below builds real geometry and then measures it against what this
// file computes. Nothing here is a proof on its own: it is the oracle the steps
// are checked against, so a formula written wrong here is a formula written
// wrong in the step that trusts it, and the two never agree by accident because
// the step's numbers come out of a solved sketch or a built body.
//
// Lengths are millimetres throughout. Fusion's own internal unit is the
// centimetre, and the step list carries that conversion; the proof has no
// reason to work in a unit nothing here reads back.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
)

// ---------------------------------------------------------------------------
// Plane vectors.
// ---------------------------------------------------------------------------

// planeVec is a point or direction in one of the two plane frames this proof
// works in: the shared section 2 sketch frame, and the per-gear frame that
// latticeOf builds.
type planeVec struct{ X, Y float64 }

func pv(x, y float64) planeVec { return planeVec{x, y} }

func (a planeVec) plus(b planeVec) planeVec    { return planeVec{a.X + b.X, a.Y + b.Y} }
func (a planeVec) minus(b planeVec) planeVec   { return planeVec{a.X - b.X, a.Y - b.Y} }
func (a planeVec) times(k float64) planeVec    { return planeVec{a.X * k, a.Y * k} }
func (a planeVec) norm() float64               { return math.Hypot(a.X, a.Y) }
func (a planeVec) dot(b planeVec) float64      { return a.X*b.X + a.Y*b.Y }
func (a planeVec) cross(b planeVec) float64    { return a.X*b.Y - a.Y*b.X }
func (a planeVec) distance(b planeVec) float64 { return a.minus(b).norm() }

func (a planeVec) direction() planeVec {
	n := a.norm()
	if n == 0 {
		return planeVec{}
	}
	return a.times(1 / n)
}

// perpOf is the in-plane perpendicular (-y, x) the section 2 figure grows
// along, before the target normal picks its sign ([BEVEL-F-APEX-LOCAL]).
func perpOf(d planeVec) planeVec { return planeVec{-d.Y, d.X} }

// distancePointLine is the perpendicular distance from p to the infinite line
// through a and b. The Maximum Face Width is two of these, and the proof reads
// them off solved geometry exactly as the generated module does
// ([PB-SOLVED-GEOMETRY]).
func distancePointLine(p, a, b planeVec) float64 {
	d := b.minus(a).direction()
	if d.norm() == 0 {
		return p.distance(a)
	}
	return math.Abs(p.minus(a).cross(d))
}

// foldAngle folds an angle into (-pi, pi].
func foldAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// ---------------------------------------------------------------------------
// The resolved case.
// ---------------------------------------------------------------------------

// sideMember is one gear of the pair, with every per-gear value already
// resolved through its own bounds.
type sideMember struct {
	Name          string  // "Pinion" or "Driving"
	Teeth         float64 // this gear's tooth count
	PitchDiameter float64 // Module * Teeth
	Gamma         float64 // this gear's pitch cone half angle, radians
	BaseHeight    float64 // resolved, inside this gear's own window
	BoreDiameter  float64 // resolved; 0 when Enable Bore is off
	ToeRadius     float64 // resolved inner toe corner radius
}

// PitchRadius is this gear's heel pitch radius, the perpendicular distance from
// Apex 2 to its own shaft axis.
func (m sideMember) PitchRadius() float64 { return m.PitchDiameter / 2 }

// VirtualPitchRadius is the back-cone (Tredgold) pitch radius, r / cos(gamma).
// This is the radius the drawn tooth's pitch circle has to reach, which is why
// the virtual tooth count below is never rounded.
func (m sideMember) VirtualPitchRadius() float64 {
	return m.PitchRadius() / math.Cos(m.Gamma)
}

// VirtualTeeth is the back-cone tooth number, 2 * VirtualPitchRadius / Module.
//
// It is a REAL number and is never rounded. Rounding it rebuilds every drawn
// circle from the rounded count, which draws the tooth smaller than the back
// cone places it: at 31 teeth, Module 1 and gamma 45 degrees the exact radius is
// 21.9203 mm and a floored count of 43 draws 21.5 mm. The spur drawer reads the
// count only as the angular half thickness pi / (2 * z), so a real count leaves
// the tooth at the standard pi * Module / 2 of thickness at the pitch circle.
func (m sideMember) VirtualTeeth(module float64) float64 {
	return 2 * m.VirtualPitchRadius() / module
}

// MinBaseHeight is the heel edge's inner end: below it H lands behind C and the
// edge C->H runs back inward instead of outward.
func (m sideMember) MinBaseHeight(module float64) float64 {
	return 1.05 * 1.25 * module * math.Sin(m.Gamma)
}

// MaxBaseHeight is the heel edge's outer end, measured from Apex 2's plane and
// starting from the dedendum corner rather than the pitch point, which is what
// makes it deliberately conservative. Past the true crossing r * tan(gamma) the
// hexagon has crossed its own axis of revolution and the revolve fails with
// ASM_WIRE_X_AXIS ([PB-REVOLVE]).
func (m sideMember) MaxBaseHeight(module float64) float64 {
	return 0.95 * (m.PitchRadius() - 1.25*module*math.Cos(m.Gamma)) * math.Tan(m.Gamma)
}

// MinTeeth is the computed tooth floor, 5.27 * cos(gamma). It is exactly the
// statement that the base-height window above is non-empty. The constant is
// 2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632 rounded UP, so the published floor
// stays at or above the exact crossing.
func (m sideMember) MinTeeth() float64 { return 5.27 * math.Cos(m.Gamma) }

// bevelCase is one resolved dialog case: the values the generated module works
// out before it draws anything, plus the two members of the pair.
type bevelCase struct {
	Module       float64
	Sigma        float64 // Shaft Angle, radians
	ConeDistance float64 // hypot(DPD, PPD) -- the diagonal, NOT R
	R            float64 // Pitch Cone Distance, (PPD/2) / sin(gamma_p)
	RootDistance float64 // |Apex->Ded|, hypot(R, 1.25 * Module)
	MaxFaceWidth float64
	FaceWidth    float64 // resolved, already capped
	RootLength   float64 // resolved |Ded->Toe|
	RootLength0  float64 // the Toe Extension 0 root length
	ToothSpacing float64
	SpiralAngle  float64 // psi, radians; 0 is a straight bevel
	HandSign     float64 // +1 Right, -1 Left, as read on the DRIVING gear
	CutterRadius float64 // 0 means auto, R_mean per gear
	RootSink     float64 // 0.05 * 2.25 * Module

	Pinion  sideMember
	Driving sideMember
}

// spurPressureAngle is the 20 degrees the borrowed spur drawer uses. It is not a
// bevel dialog input.
const spurPressureAngle = 20 * math.Pi / 180

// spurInvoluteSteps is the InvoluteSteps the virtual spur proxy serves.
const spurInvoluteSteps = 15

// rootSinkShare is the share of the tooth height the root circle is sunk by,
// spec section 3 step 1: the sink is rootSinkShare * 2.25 * Module. At the
// dedendum corner exactly, only the point on the tooth's own centreline rides
// the root cone and the arc's two corners stand outside it -- 0.027 module on a
// 4/4 pair, the largest of any pair the spec admits. The sink pushes the whole
// arc inside so the Combine-Join meets the body across the root.
const rootSinkShare = 0.05

// toeReachCeiling is where a Toe Extension of 100 stops: 0.99 of the way from
// the Toe Extension 0 root length to the smaller of the two Toe Limits. At the
// limit itself the toe face has zero length, the revolved body carries no cone
// at its toe end, and the toe conical trim has no ConeSurfaceType face to find.
const toeReachCeiling = 0.99

// spiralEndOvershoot is how far past the face the cutter arc's kept ends are
// taken, as a share of the span, so the arc reaches cleanly past the end trims.
const spiralEndOvershoot = 0.06

// crownPerRad is _CROWN_PER_RAD, the tunable class constant the lengthwise
// crown scales its relief by. The spec fixes the default at 0.5.
const crownPerRad = 0.5

// sliceCount is the fixed number of cut planes the spiral slice steps through.
// It is not user-configurable.
const sliceCount = 8

// maxShaftAngleOf is the Shaft Angle ceiling: the cone-angle limit
// acos(-smaller/larger), which is exclusive because a pitch cone angle reaching
// 90 degrees turns that gear's cone inside out, capped at an inclusive 150
// degrees. Equal tooth counts give acos(-1) = 180, which is no constraint.
func maxShaftAngleOf(drivingTeeth, pinionTeeth float64) float64 {
	small, large := math.Min(drivingTeeth, pinionTeeth), math.Max(drivingTeeth, pinionTeeth)
	return math.Min(math.Acos(-small/large), 150*math.Pi/180)
}

// resolveCase resolves one parameter map the way the spec's Variables section
// does, in the order it fixes: cone angles, then the per-gear tooth floor and
// base-height window, then the face width against its cap, then the toe end.
//
// The map keys are the dialog input ids the spec's table fixes, so a recompile
// cannot rename them; keyGearSide is the one key that is not a dialog input and
// only says which member of the pair a per-gear step is building.
func resolveCase(t testing.TB, p map[string]float64) bevelCase {
	t.Helper()

	module := p[keyModule]
	sigma := p[keyShaftAngle] * math.Pi / 180
	pinionTeeth, drivingTeeth := p[keyPinionTeeth], p[keyDrivingTeeth]
	ppd, dpd := module*pinionTeeth, module*drivingTeeth

	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	r := (ppd / 2) / math.Sin(gammaP)
	coneDistance := math.Hypot(dpd, ppd)
	rootDistance := math.Hypot(r, 1.25*module)

	pinion := sideMember{Name: "Pinion", Teeth: pinionTeeth, PitchDiameter: ppd, Gamma: gammaP}
	driving := sideMember{Name: "Driving", Teeth: drivingTeeth, PitchDiameter: dpd, Gamma: gammaG}

	// The driving side resolves first because the pinion's own fallback is a
	// share of the RESOLVED driving height -- after the driving fallback and
	// after the driving cap, never the raw input -- and the pinion's own window
	// is then applied on top, since the two gammas differ whenever the tooth
	// counts do.
	driving.BaseHeight = resolveBaseHeight(p[keyDrivingBase], module*drivingTeeth/8, module, driving)
	pinion.BaseHeight = resolveBaseHeight(p[keyPinionBase],
		driving.BaseHeight*pinionTeeth/drivingTeeth, module, pinion)

	// The Maximum Face Width is the smaller of the two perpendicular distances
	// from A to line C->H and from B to line D->J. Either gear can be the
	// binding one, so both are computed and the smaller kept; written with the
	// pinion's diameter by name it is wrong whenever the driving gear carries
	// the smaller tooth count.
	maxFaceWidth := 0.95 * r * math.Min(
		math.Sin(gammaP)*math.Sin(gammaP), math.Sin(gammaG)*math.Sin(gammaG))
	faceWidth := p[keyFaceWidth]
	if faceWidth <= 0 {
		faceWidth = math.Min(coneDistance/6, maxFaceWidth)
	}

	rootLength0 := faceWidth * rootDistance / r
	pinion.ToeRadius = resolveToeRadius(p[keyPinionToeRadius], faceWidth, pinion)
	driving.ToeRadius = resolveToeRadius(p[keyDrivingToeRadius], faceWidth, driving)

	// The pair shares one root length, so the SMALLER of the two Toe Limits
	// wins and the other gear stops short of its own X.
	limit := math.Min(
		toeLimitOf(pinion, module, r, rootDistance),
		toeLimitOf(driving, module, r, rootDistance))
	rootLength := rootLength0
	if pct := p[keyToeExtension]; pct > 0 && limit > rootLength0 {
		rootLength += (pct / 100) * toeReachCeiling * (limit - rootLength0)
	}

	pinion.BoreDiameter = resolveBore(p[keyPinionBore], p[keyBoreEnable], ppd)
	driving.BoreDiameter = resolveBore(p[keyDrivingBore], p[keyBoreEnable], dpd)

	hand := 1.0
	if p[keyHand] == 0 {
		hand = -1
	}

	return bevelCase{
		Module:       module,
		Sigma:        sigma,
		ConeDistance: coneDistance,
		R:            r,
		RootDistance: rootDistance,
		MaxFaceWidth: maxFaceWidth,
		FaceWidth:    faceWidth,
		RootLength:   rootLength,
		RootLength0:  rootLength0,
		ToothSpacing: p[keyToothSpacing],
		SpiralAngle:  p[keySpiralAngle] * math.Pi / 180,
		HandSign:     hand,
		CutterRadius: p[keyCutterRadius],
		RootSink:     rootSinkShare * 2.25 * module,
		Pinion:       pinion,
		Driving:      driving,
	}
}

// resolveBaseHeight applies one gear's own window in both directions: a
// fallback below the minimum is raised, one above the maximum is capped, and a
// user value inside the window is taken as given. A user value outside either
// end is a rejection the dialog makes before anything is drawn, so the proof's
// tables carry only values the dialog admits.
func resolveBaseHeight(raw, fallback, module float64, m sideMember) float64 {
	if raw > 0 {
		return raw
	}
	lo, hi := m.MinBaseHeight(module), m.MaxBaseHeight(module)
	return math.Min(math.Max(fallback, lo), hi)
}

// resolveToeRadius defaults to this gear's own inner toe corner radius at Toe
// Extension 0, which is the value that makes Toe Extension 0 today's profile
// exactly.
func resolveToeRadius(raw, faceWidth float64, m sideMember) float64 {
	if raw > 0 {
		return raw
	}
	return m.PitchRadius() - faceWidth/math.Sin(m.Gamma)
}

// toeRadiusCeilingOf is this gear's OUTER toe corner radius at Toe Extension 0.
// At or above it the point X falls behind the toe corner and the Toe Extension
// has nowhere to go.
func toeRadiusCeilingOf(m sideMember, module, r, faceWidth float64) float64 {
	return (m.PitchRadius() - 1.25*module*math.Cos(m.Gamma)) * (1 - faceWidth/r)
}

// toeLimitOf is |Ded->X|, with X the point on the root element Apex->Ded at this
// gear's Toe Radius. It is where the toe end is heading: at X the toe face has
// closed to nothing.
func toeLimitOf(m sideMember, module, r, rootDistance float64) float64 {
	gammaRoot := m.Gamma - math.Atan2(1.25*module, r)
	return rootDistance - m.ToeRadius/math.Sin(gammaRoot)
}

func resolveBore(raw, enable, pitchDiameter float64) float64 {
	if enable == 0 {
		return 0
	}
	if raw > 0 {
		return raw
	}
	return pitchDiameter / 4
}

// memberFor picks the member of the pair a per-gear step is building. One
// resolved case holds both gears; keyGearSide is all that separates them.
func memberFor(c bevelCase, p map[string]float64) sideMember {
	if p[keyGearSide] != 0 {
		return c.Driving
	}
	return c.Pinion
}

// ---------------------------------------------------------------------------
// The per-gear lattice.
// ---------------------------------------------------------------------------

// latticeFrame is one gear's half of the section 2 figure, in THAT GEAR'S own
// frame: X runs from the Apex along its shaft axis and Y is the perpendicular
// distance from that axis. A cone is a statement about a distance from one
// axis, so this frame is the one every cone reading is taken in; the shared
// sketch frame would make each of them a projection.
type latticeFrame struct {
	Apex   planeVec // (0, 0)
	Axis   planeVec // A / B, the pitch station on the shaft axis
	Base   planeVec // G / I, the heel end of the shaft edge
	Foot   planeVec // E / F, one module along the shaft past A / B
	Apex2  planeVec
	Ded    planeVec // C / D, the dedendum corner
	Heel   planeVec // H / J, the heel outer corner
	Back   planeVec // K / L, the back-cone point
	Tooth  planeVec // K' / L', the tooth centre after Tooth Spacing
	Toe    planeVec // M / O, the toe corner on the root element
	ToeIn  planeVec // N / P, the toe corner on the toe-radius line
	Front  planeVec // A' / B', the front face's foot on the shaft axis
	DedDir planeVec // unit Apex2 -> Ded -> Heel
}

// latticeOf is the closed form section 2 solves to, for one gear.
func latticeOf(c bevelCase, m sideMember) latticeFrame {
	sin, cos := math.Sin(m.Gamma), math.Cos(m.Gamma)
	pitchRadius := m.PitchRadius()
	along := c.R * cos

	// Outward along the back cone: further from the Apex along the shaft,
	// closer to the shaft axis.
	ded := pv(sin, -cos)

	apex2 := pv(along, pitchRadius)
	dedendum := apex2.plus(ded.times(1.25 * c.Module))
	heel := apex2.plus(ded.times(m.BaseHeight / sin))
	back := apex2.plus(ded.times(pitchRadius / cos))

	toe := dedendum.times(1 - c.RootLength/dedendum.norm())

	// The front face stands square to the shaft at the Toe Radius, and the toe
	// line M->N is C->H offset toward the Apex by the root length measured
	// perpendicular to the pitch line.
	faceOffset := c.RootLength * c.R / c.RootDistance
	toeIn := pv(along+(-faceOffset-(m.ToeRadius-pitchRadius)*sin)/cos, m.ToeRadius)

	tooth := back
	if c.ToothSpacing > 0 {
		tooth = back.plus(ded.times(c.ToothSpacing))
	}

	return latticeFrame{
		Apex:   pv(0, 0),
		Axis:   pv(along, 0),
		Base:   pv(along+m.BaseHeight, 0),
		Foot:   pv(along+1.25*c.Module*sin, 0),
		Apex2:  apex2,
		Ded:    dedendum,
		Heel:   heel,
		Back:   back,
		Tooth:  tooth,
		Toe:    toe,
		ToeIn:  toeIn,
		Front:  pv(toeIn.X, 0),
		DedDir: ded,
	}
}

// profileLoop is the revolved hexagon in draw order, A' -> G -> H -> C -> M -> N
// (pinion) / B' -> I -> J -> D -> O -> P (driving). Its FIRST edge is the gear's
// shaft axis, which every body operation below uses.
func (f latticeFrame) profileLoop() []planeVec {
	return []planeVec{f.Front, f.Base, f.Heel, f.Ded, f.Toe, f.ToeIn}
}

// coneDistanceAlong is a point's cone distance: its distance from the apex
// measured along the root cone element Apex->Ded.
func (f latticeFrame) coneDistanceAlong(p planeVec) float64 {
	return p.minus(f.Apex).dot(f.Ded.direction())
}

// toeMid and heelMid are the toe and heel edge MIDPOINTS. They are two
// DIFFERENT edges: passing the two endpoints of one edge collapses the span to
// zero or negative and inverts the whole spiral frame.
func (f latticeFrame) toeMid() planeVec  { return f.Toe.plus(f.ToeIn).times(0.5) }
func (f latticeFrame) heelMid() planeVec { return f.Ded.plus(f.Heel).times(0.5) }

// signedProfileArea is twice the signed area of the hexagon, used to check the
// revolve substitution against Pappus.
func polygonArea(loop []planeVec) float64 {
	sum := 0.0
	for i := range loop {
		a, b := loop[i], loop[(i+1)%len(loop)]
		sum += a.cross(b)
	}
	return math.Abs(sum) / 2
}

// polygonCentroidY is the hexagon's centroid distance from the shaft axis, the
// other half of Pappus.
func polygonCentroidY(loop []planeVec) float64 {
	var cross, cy float64
	for i := range loop {
		a, b := loop[i], loop[(i+1)%len(loop)]
		w := a.cross(b)
		cross += w
		cy += (a.Y + b.Y) * w
	}
	if cross == 0 {
		return 0
	}
	return cy / (3 * cross)
}

// ---------------------------------------------------------------------------
// The virtual spur tooth.
// ---------------------------------------------------------------------------

// toothCircles are the four radii the virtual spur proxy is asked for. Pitch,
// base and tip are the plain spur formulas at the virtual tooth count; the root
// is the only one the sink moves.
type toothCircles struct {
	Pitch, Base, Tip, Root float64
	Sink                   float64
	VirtualTeeth           float64
	Embedded               bool
}

// circlesFor builds the four circles for one gear. rootSink shortens the ROOT
// circle only, by the sink itself on radius, and leaves pitch, base and tip
// where the standard spur formulas put them.
func circlesFor(c bevelCase, m sideMember) toothCircles {
	vt := m.VirtualTeeth(c.Module)
	dims := involute.Derive(c.Module, vt, spurPressureAngle)
	sunk := dims
	sunk.Root = dims.Root - c.RootSink
	return toothCircles{
		Pitch:        dims.Pitch,
		Base:         dims.Base,
		Tip:          dims.Tip,
		Root:         sunk.Root,
		Sink:         c.RootSink,
		VirtualTeeth: vt,
		// The embedded flag is the spur drawer's own answer, taken from the
		// SUNK root radius: base < root is what leaves no room for the two
		// flank-to-root lines. Sinking the root can drop it below the base
		// circle and turn an embedded tooth into a non-embedded one, which is
		// exactly what the shipped 31/31 default at Module 1 does.
		Embedded: sunk.Embedded(),
	}
}

// wantLines is the line count the tooth loop carries, which is DETERMINED by
// the embedded flag and never guessed: 0 when embedded, 2 otherwise. An
// unrelated loop between the drawn circles can carry the same two NURBS and two
// arcs with the other line count, and selecting it makes the apex loft fail with
// LOFT_NO_TOOLBODY.
func (tc toothCircles) wantLines() int {
	if tc.Embedded {
		return 0
	}
	return 2
}

// involutePt is the flank sample type, aliased so the steps do not each name
// the shared package.
type involutePt = involute.Pt

// flankSamples are the two flanks of the tooth, already drawn rotated 180
// degrees the way the spur drawer's own angle argument draws them.
func (tc toothCircles) flankSamples() (left, right []involute.Pt) {
	return involute.Flanks(tc.Base, tc.Tip, tc.Pitch, tc.VirtualTeeth, spurInvoluteSteps, math.Pi)
}

// rootAtHeel is where the drawn root arc lands on the gear body: the heel root
// radius, one dedendum in from the pitch radius along the back cone, less the
// sink. The whole point of the sink is that this sits INSIDE the gear body's
// root cone across the arc's whole width rather than touching it along one line.
func rootAtHeel(c bevelCase, m sideMember) float64 {
	return m.PitchRadius() - 1.25*c.Module*math.Cos(m.Gamma) - c.RootSink
}

// toothPolar is one tooth cross-section at the heel, in polar coordinates about
// the gear's shaft axis: the drawn virtual spur tooth mapped onto the back cone.
type toothPolar struct {
	Right, Left []polarPt
	RootRadius  float64
	TipRadius   float64
	Embedded    bool
}

type polarPt struct{ Radius, Theta float64 }

// toothSectionOf maps the drawn tooth onto the gear. The drawn tooth lies on
// the back-cone plane, so a drawn point's radial distance from the tooth centre
// becomes a radial distance from the gear's own axis, and its azimuth about the
// tooth centre opens by 1/cos(gamma) as the back cone is rolled onto the gear.
func toothSectionOf(c bevelCase, m sideMember) toothPolar {
	tc := circlesFor(c, m)
	left, right := tc.flankSamples()
	root := rootAtHeel(c, m)
	cosGamma := math.Cos(m.Gamma)

	convert := func(src []involute.Pt) []polarPt {
		out := make([]polarPt, 0, len(src))
		for _, p := range src {
			rv := math.Hypot(p.X, p.Y)
			phi := foldAngle(math.Atan2(p.Y, p.X) - math.Pi)
			out = append(out, polarPt{Radius: root + (rv - tc.Root), Theta: math.Pi + phi/cosGamma})
		}
		return out
	}

	section := toothPolar{
		Right:      convert(right),
		Left:       convert(left),
		RootRadius: root,
		TipRadius:  root + (tc.Tip - tc.Root),
		Embedded:   tc.Embedded,
	}
	if section.Right[0].Theta > section.Left[0].Theta {
		section.Right, section.Left = section.Left, section.Right
	}
	if section.Embedded {
		section.Right = clipToRoot(section.Right, section.RootRadius)
		section.Left = clipToRoot(section.Left, section.RootRadius)
	}
	return section
}

func clipToRoot(flank []polarPt, root float64) []polarPt {
	out := make([]polarPt, 0, len(flank))
	for _, p := range flank {
		if p.Radius >= root {
			out = append(out, p)
		}
	}
	if len(out) == 0 {
		return flank
	}
	out[0].Radius = root
	return out
}

// outerRootRadius is the root arc's OUTERMOST point, the reading the join check
// has to take. The centreline sits inside both root corners, so a reading taken
// there passes a tooth whose corners float outside the root cone -- which is
// exactly the defect the sink exists to remove.
func (s toothPolar) outerRootRadius() float64 {
	widest := s.RootRadius
	for _, flank := range [][]polarPt{s.Right, s.Left} {
		if len(flank) == 0 {
			continue
		}
		p := flank[0]
		if r := p.Radius / math.Cos(0); r > widest {
			widest = r
		}
	}
	// The root arc runs between the two flanks' root ends at the root radius,
	// so its outermost point in a plane through the axis is the root radius
	// itself scaled by the cosine of half the arc's own subtended angle away
	// from the section plane. Half that angle is what separates the two root
	// ends; the arc bulges no further than its own radius.
	half := 0.0
	if len(s.Right) > 0 && len(s.Left) > 0 {
		half = math.Abs(foldAngle(s.Left[0].Theta-s.Right[0].Theta)) / 2
	}
	return s.RootRadius / math.Cos(half)
}

// toothSectionArea is the drawn tooth's cross-sectional area, closed by the
// root arc and the tip arc.
func (s toothPolar) area() float64 {
	loop := make([]planeVec, 0, len(s.Right)+len(s.Left))
	for _, p := range s.Right {
		loop = append(loop, pv(p.Radius*math.Cos(p.Theta), p.Radius*math.Sin(p.Theta)))
	}
	for i := len(s.Left) - 1; i >= 0; i-- {
		p := s.Left[i]
		loop = append(loop, pv(p.Radius*math.Cos(p.Theta), p.Radius*math.Sin(p.Theta)))
	}
	return polygonArea(loop)
}

// ---------------------------------------------------------------------------
// The spiral trace.
// ---------------------------------------------------------------------------

// spiralTrace is the cutter-arc construction of spec/bevelgear/spiral-tooth-trace.md,
// in the tangent plane's own 2-D frame: origin at the apex, x = coneVec (so a
// point's x is its cone distance) and y = v (circumferential).
type spiralTrace struct {
	RToe, RHeel, RMean, Span float64
	CutterRadius             float64
	Centre                   planeVec
	Toe2D, Heel2D            planeVec
	HandSign                 float64
	PhiCrown                 float64
	Total                    float64 // toe->heel shaft-axis twist magnitude
}

// traceFor builds the trace for one gear. The hand sign is read on the DRIVING
// gear and NEGATED for the pinion, because the pair meshes with opposite hands.
func traceFor(c bevelCase, m sideMember) spiralTrace {
	f := latticeOf(c, m)
	rToe := f.coneDistanceAlong(f.toeMid())
	rHeel := f.coneDistanceAlong(f.heelMid())
	rMean := (rToe + rHeel) / 2
	span := rHeel - rToe

	rc := c.CutterRadius
	if rc == 0 {
		rc = rMean
	}
	hand := c.HandSign
	if m.Name == "Pinion" {
		hand = -hand
	}

	// The hand sign goes on the cos / Cy term and NEVER on the sin / Cx term.
	// Opposite hand mirrors the cutter centre across the cone element y = 0,
	// which flips Cy; putting it on Cx mirrors about x = R_mean instead, a
	// different curve that gives the two gears unequal twist.
	centre := pv(rMean-rc*math.Sin(c.SpiralAngle), hand*rc*math.Cos(c.SpiralAngle))

	toe2d := circleIntersectNearest(rToe-spiralEndOvershoot*span, centre, rc, pv(rMean, 0))
	heel2d := circleIntersectNearest(rHeel+spiralEndOvershoot*span, centre, rc, pv(rMean, 0))

	phi := math.Atan2(heel2d.Y, heel2d.X) - math.Atan2(toe2d.Y, toe2d.X)
	total := math.Abs(phi) / math.Sin(m.Gamma)

	return spiralTrace{
		RToe: rToe, RHeel: rHeel, RMean: rMean, Span: span,
		CutterRadius: rc, Centre: centre,
		Toe2D: toe2d, Heel2D: heel2d,
		HandSign: hand, PhiCrown: phi, Total: total,
	}
}

// circleIntersectNearest intersects the apex circle of radius rr with the cutter
// circle (centre, rc) and keeps the solution nearest ref, the branch the mean
// point sits on. A non-overlap clamps to tangency, which is what the framework
// helper of the same shape does.
func circleIntersectNearest(rr float64, centre planeVec, rc float64, ref planeVec) planeVec {
	d := centre.norm()
	if d == 0 {
		return pv(rr, 0)
	}
	a := (rr*rr - rc*rc + d*d) / (2 * d)
	h2 := rr*rr - a*a
	u := centre.direction()
	base := u.times(a)
	if h2 <= 0 {
		return base
	}
	h := math.Sqrt(h2)
	n := pv(-u.Y, u.X)
	first, second := base.plus(n.times(h)), base.minus(n.times(h))
	if first.distance(ref) <= second.distance(ref) {
		return first
	}
	return second
}

// twistAt is one slab's rotation about the shaft axis, keyed on the cone
// distance of its HEEL FACE and centred on R_mean so the mid-face section stays
// unrotated. Keying on the centroid instead leaves the loft's mid-face section
// rotated by half a segment.
func (tr spiralTrace) twistAt(rHeelFace float64) float64 {
	return -tr.HandSign * tr.Total * (tr.RMean - rHeelFace) / tr.Span
}

// crownAt is one slab's lengthwise relief, keyed on the MONOTONIC heel-distance
// fraction u and never on |ang|. Keying on the twist magnitude is symmetric
// about mid-face, so with the heel slab held full the slab just inside it
// becomes the most relieved one and dips below both neighbours.
func (tr spiralTrace) crownAt(rHeelFace float64) float64 {
	u := (tr.RHeel - rHeelFace) / tr.Span
	return 1 - crownPerRad*(math.Abs(tr.Total)/2)*u
}

// sliceOffsets are the cut-plane offsets, sign * (k+1) * span/6 for k = 0..7,
// all stepping from the parent transverse tooth plane toward the apex.
func (tr spiralTrace) sliceOffsets(sign float64) []float64 {
	offsets := make([]float64, 0, sliceCount)
	for k := range sliceCount {
		offsets = append(offsets, sign*float64(k+1)*tr.Span/6)
	}
	return offsets
}
