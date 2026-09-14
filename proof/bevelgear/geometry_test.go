// Package bevelgear_test proves the bevel gear pair's build, step by step,
// against the closed forms spec/bevelgear/instructions.md pins.
//
// This file holds the closed form itself: the §2 lattice solved analytically in
// the Gear Profiles sketch's own 2-D frame, the resolved input bounds, and the
// per-gear anchors the later steps measure against. Nothing here draws or
// builds; the step functions in sketches_test.go, solids_test.go and
// spiral_test.go do that and check themselves against these numbers.
//
// Frame. Every 2-D quantity lives in the Gear Profiles sketch's local frame,
// with the projected centre at the origin, the projected Anchor Line along +X
// and the grow direction `perp` = +Y. The generated module picks perp's sign
// from the target plane's normal ([BEVEL-F-GROW-SIDE]); that one-bit choice is
// proved in stepGearProfilesPlane and is fixed to +Y everywhere else, so the
// lattice numbers below are the same figure either way.
//
// Units. Lengths are millimetres, angles radians. The dialog's parameter keys
// carry lengths in mm and angles in degrees; bgRead converts.
package bevelgear_test

import (
	"math"
	"testing"
)

// The proof's parameter keys are the dialog's own input ids, so a case table
// reads as the dialog does. They are spelled here rather than shared with the
// hand-written drawing files' key constants so this file stands alone.
const (
	idModule            = "module"
	idShaftAngle        = "shaftAngle"
	idDrivingTeeth      = "drivingTeeth"
	idPinionTeeth       = "pinionTeeth"
	idDrivingBaseHeight = "drivingBaseHeight"
	idPinionBaseHeight  = "pinionBaseHeight"
	idBoreEnable        = "boreEnable"
	idDrivingBore       = "drivingBore"
	idPinionBore        = "pinionBore"
	idFaceWidth         = "faceWidth"
	idToothSpacing      = "toothSpacing"
	idSpiralAngle       = "spiralAngle"
	idHand              = "spiralHand"
	idCutterRadius      = "cutterRadius"
	idToeExtension      = "toeExtension"
	idDrivingToeRadius  = "drivingToeRadius"
	idPinionToeRadius   = "pinionToeRadius"
	idSide              = "gearSide"
	// idRefused marks a configuration the spec admits that this particular §2
	// net cannot reach — see "A refusal the case table records rather than
	// avoids". The case stays in the table; the step reads the flag.
	idRefused = "declaredRefusal"
)

// Fixed values the generated module carries as module-level constants.
const (
	bgPressureAngle     = 20.0 * math.Pi / 180 // VirtualSpurProxy's default, not a bevel dialog input
	bgInvoluteSteps     = 15                   // VirtualSpurProxy's default
	bgDedendumFactor    = 1.25
	bgAddendumFactor    = 1.0
	bgCrownPerRad       = 0.5 // _CROWN_PER_RAD
	bgMeshPhaseTeeth    = 0.0 // _PINION_MESH_PHASE_TEETH
	bgHandRight         = 1.0 // _HAND_RIGHT = 'Right'
	bgHandLeft          = -1.0
	bgSlicePlanes       = 8 // the fixed slice scheme of §3a step E
	bgTraceOvershoot    = 0.06
	bgToeExtCap         = 0.99
	bgShaftAngleCeiling = 150.0 // the practical ceiling the cone-angle limit is capped at
	bgShaftAngleFloor   = 30.0
	bgMinTeethFactor    = 5.27
	bgBoundFactor       = 0.95
	bgMinBHFactor       = 1.05
)

// bgPt is a point or vector in the Gear Profiles sketch's 2-D frame, mm.
type bgPt struct{ X, Y float64 }

func bgAdd(a, b bgPt) bgPt           { return bgPt{a.X + b.X, a.Y + b.Y} }
func bgSub(a, b bgPt) bgPt           { return bgPt{a.X - b.X, a.Y - b.Y} }
func bgScale(a bgPt, k float64) bgPt { return bgPt{a.X * k, a.Y * k} }
func bgDot(a, b bgPt) float64        { return a.X*b.X + a.Y*b.Y }
func bgCross(a, b bgPt) float64      { return a.X*b.Y - a.Y*b.X }
func bgLen(a bgPt) float64           { return math.Hypot(a.X, a.Y) }
func bgUnit(a bgPt) bgPt             { return bgScale(a, 1/bgLen(a)) }
func bgRot(a bgPt, t float64) bgPt {
	s, c := math.Sin(t), math.Cos(t)
	return bgPt{a.X*c - a.Y*s, a.X*s + a.Y*c}
}

// bgMix walks from a toward b by t of the way.
func bgMix(a, b bgPt, t float64) bgPt { return bgAdd(a, bgScale(bgSub(b, a), t)) }

// bgIn is one resolved dialog reading.
type bgIn struct {
	Module            float64
	ShaftAngle        float64 // radians
	DrivingTeeth      float64
	PinionTeeth       float64
	DrivingBaseHeight float64 // mm, 0 means "unspecified"
	PinionBaseHeight  float64
	BoreEnable        bool
	DrivingBore       float64
	PinionBore        float64
	FaceWidth         float64 // mm, 0 means "unspecified"
	ToothSpacing      float64
	SpiralAngle       float64 // radians
	Hand              float64 // +1 Right, -1 Left
	CutterRadius      float64
	ToeExtension      float64 // percent, [0, 100]
	DrivingToeRadius  float64 // mm, 0 means "auto-calculate"
	PinionToeRadius   float64
	Driving           bool // gearSide: 0 pinion, 1 driving
	DeclaredRefusal   bool
}

func bgGet(p map[string]float64, key string, fallback float64) float64 {
	if v, ok := p[key]; ok {
		return v
	}
	return fallback
}

// bgRead turns a case's parameter map into a resolved reading.
//
// Angles arrive in degrees. The Shaft Angle reader also accepts radians,
// because the spec's own range floor of 30° puts the two spellings in disjoint
// ranges: no legal degree reading is below 6.5 and no legal radian reading is
// above it. That tolerance is the proof's alone — the generated module reads
// one unit, Fusion's internal radians ([PB-EVAL-EXPRESSION]).
func bgRead(p map[string]float64) bgIn {
	shaft := bgGet(p, idShaftAngle, 90)
	if shaft > 6.5 {
		shaft = shaft * math.Pi / 180
	}
	return bgIn{
		Module:            bgGet(p, idModule, 1),
		ShaftAngle:        shaft,
		DrivingTeeth:      bgGet(p, idDrivingTeeth, 31),
		PinionTeeth:       bgGet(p, idPinionTeeth, 31),
		DrivingBaseHeight: bgGet(p, idDrivingBaseHeight, 0),
		PinionBaseHeight:  bgGet(p, idPinionBaseHeight, 0),
		BoreEnable:        bgGet(p, idBoreEnable, 1) != 0,
		DrivingBore:       bgGet(p, idDrivingBore, 0),
		PinionBore:        bgGet(p, idPinionBore, 0),
		FaceWidth:         bgGet(p, idFaceWidth, 0),
		ToothSpacing:      bgGet(p, idToothSpacing, 0),
		SpiralAngle:       bgGet(p, idSpiralAngle, 0) * math.Pi / 180,
		Hand:              bgGet(p, idHand, bgHandRight),
		CutterRadius:      bgGet(p, idCutterRadius, 0),
		ToeExtension:      bgGet(p, idToeExtension, 0),
		DrivingToeRadius:  bgGet(p, idDrivingToeRadius, 0),
		PinionToeRadius:   bgGet(p, idPinionToeRadius, 0),
		Driving:           bgGet(p, idSide, 0) != 0,
		DeclaredRefusal:   bgGet(p, idRefused, 0) != 0,
	}
}

// bgMember is one gear of the pair, with every §2 anchor that belongs to it.
//
// Pinion and Driving carry the same field names over the mirrored points, so a
// per-gear step reads one structure whatever side it is building: the pinion's
// Ded is C and the driving's is D, the pinion's Heel is H and the driving's J,
// and so on down the "Create the Gear Bodies" substitution table.
type bgMember struct {
	Label    string  // "Pinion" / "Driving" — the {gearLabel} of every sketch name
	Teeth    float64 //
	PitchDia float64 // this gear's Pitch Diameter, mm
	Gamma    float64 // this gear's pitch cone angle, radians

	AxisDir bgPt // unit Apex->A (pinion) / Apex->B (driving)
	DedDir  bgPt // unit Apex2->C (pinion) / Apex2->D (driving), the back-cone direction
	RootDir bgPt // unit Apex->C / Apex->D, the root cone element

	Shaft     bgPt // A / B
	Ext       bgPt // E / F, the module-length extension's end
	Ded       bgPt // C / D, the dedendum corner
	Heel      bgPt // H / J
	Foot      bgPt // G / I
	Toe       bgPt // M / O
	ToeInner  bgPt // N / P
	FrontFoot bgPt // A' / B'
	Center    bgPt // K / L
	ToothCtr  bgPt // K' / L'

	BaseHeight       float64 // resolved
	MinBaseHeight    float64
	MaxBaseHeight    float64
	MinTeeth         float64
	ToeRadius        float64 // resolved
	ToeRadiusCeiling float64
	ToeLimit         float64
	BoreDia          float64 // resolved; 0 when Enable Bore is unchecked

	VirtualPitchRadius float64
	VirtualTeeth       int
	Embedded           bool
	RootConeAngle      float64 // gamma_root
}

// bgLattice is the whole solved §2 figure plus the values §3 and the body
// steps read off it.
type bgLattice struct {
	In       bgIn
	DPD, PPD float64
	GammaP   float64
	GammaG   float64
	R        float64 // Pitch Cone Distance — never the Cone Distance below
	ConeDist float64 // Cone Distance, the diagonal of the two pitch diameters

	MaxShaftAngle float64 // degrees

	Center bgPt // the projected centre, the frame's origin
	Perp   bgPt // in-plane unit perpendicular to the projected anchor line
	Apex   bgPt
	Apex2  bgPt

	FaceWidth    float64
	MaxFaceWidth float64
	RootLength   float64
	RootLength0  float64
	ApexToDed    float64 // |Apex->Ded| = sqrt(R^2 + (1.25 m)^2)

	Pinion  bgMember
	Driving bgMember
}

// bgMaxShaftAngle is the cone-angle singularity, capped at 150°, in degrees.
// The cone-angle half is exclusive and the 150° half inclusive.
func bgMaxShaftAngle(dpd, ppd float64) float64 {
	smaller, larger := math.Min(dpd, ppd), math.Max(dpd, ppd)
	limit := math.Acos(-smaller/larger) * 180 / math.Pi
	return math.Min(limit, bgShaftAngleCeiling)
}

// bgSolve resolves one reading into the solved §2 lattice.
//
// Every position here is the position the constraint net closes on, computed
// from the closed form the spec gives, so a step can seed its geometry at the
// solved position ([PB-SEED-NEAR]) and then assert the solve against the same
// numbers.
func bgSolve(in bgIn) bgLattice {
	m := in.Module
	l := bgLattice{In: in}
	l.DPD = m * in.DrivingTeeth
	l.PPD = m * in.PinionTeeth
	l.ConeDist = math.Hypot(l.DPD, l.PPD)
	l.MaxShaftAngle = bgMaxShaftAngle(l.DPD, l.PPD)

	sig := in.ShaftAngle
	l.GammaP = math.Atan2(math.Sin(sig)*l.PPD, l.DPD+l.PPD*math.Cos(sig))
	l.GammaG = sig - l.GammaP
	l.R = (l.PPD / 2) / math.Sin(l.GammaP)
	l.ApexToDed = math.Hypot(l.R, bgDedendumFactor*m)

	l.Center = bgPt{0, 0}
	l.Perp = bgPt{0, 1}

	drivingDir := bgScale(l.Perp, -1)                    // Apex->B
	pinionDir := bgRot(drivingDir, sig)                  // Apex->A: the +X-most of the two senses
	pitchDir := bgRot(drivingDir, l.GammaG)              // Apex->Apex2
	dedC := bgPt{math.Cos(l.GammaG), math.Sin(l.GammaG)} // Apex2->C, away from the anchor line
	dedD := bgScale(dedC, -1)                            // Apex2->D, toward the anchor line

	l.Pinion = bgMember{Label: "Pinion", Teeth: in.PinionTeeth, PitchDia: l.PPD, Gamma: l.GammaP,
		AxisDir: pinionDir, DedDir: dedC}
	l.Driving = bgMember{Label: "Driving", Teeth: in.DrivingTeeth, PitchDia: l.DPD, Gamma: l.GammaG,
		AxisDir: drivingDir, DedDir: dedD}

	for _, g := range []*bgMember{&l.Pinion, &l.Driving} {
		r := g.PitchDia / 2
		g.MinBaseHeight = bgMinBHFactor * bgDedendumFactor * m * math.Sin(g.Gamma)
		g.MaxBaseHeight = bgBoundFactor * (r - bgDedendumFactor*m*math.Cos(g.Gamma)) * math.Tan(g.Gamma)
		g.MinTeeth = bgMinTeethFactor * math.Cos(g.Gamma)
		g.RootConeAngle = g.Gamma - math.Atan(bgDedendumFactor*m/l.R)
		g.VirtualPitchRadius = r / math.Cos(g.Gamma)
		g.VirtualTeeth = int(math.Floor(2 * g.VirtualPitchRadius / m))
		g.Embedded = bgEmbedded(m, float64(g.VirtualTeeth))
	}

	// Base heights, driving first: the pinion's fallback is a share of the
	// RESOLVED driving height, and then carries the pinion's own bounds.
	l.Driving.BaseHeight = bgResolveBase(in.DrivingBaseHeight, m*in.DrivingTeeth/8, l.Driving)
	l.Pinion.BaseHeight = bgResolveBase(in.PinionBaseHeight,
		l.Driving.BaseHeight*(in.PinionTeeth/in.DrivingTeeth), l.Pinion)

	// The lattice, in closure order.
	hApex := l.R*math.Cos(l.GammaG) + l.Driving.BaseHeight
	l.Apex = bgAdd(l.Center, bgScale(l.Perp, hApex))
	l.Apex2 = bgAdd(l.Apex, bgScale(pitchDir, l.R))
	l.Pinion.Shaft = bgAdd(l.Apex, bgScale(pinionDir, l.R*math.Cos(l.GammaP)))
	l.Driving.Shaft = bgAdd(l.Apex, bgScale(drivingDir, l.R*math.Cos(l.GammaG)))
	l.Pinion.Ded = bgAdd(l.Apex2, bgScale(dedC, bgDedendumFactor*m))
	l.Driving.Ded = bgAdd(l.Apex2, bgScale(dedD, bgDedendumFactor*m))

	for _, g := range []*bgMember{&l.Pinion, &l.Driving} {
		g.RootDir = bgUnit(bgSub(g.Ded, l.Apex))
		// E / F: the foot of the perpendicular from the dedendum corner onto the
		// shaft axis, which is where "C->E perpendicular to A->E" puts it.
		g.Ext = bgAdd(l.Apex, bgScale(g.AxisDir, bgDot(bgSub(g.Ded, l.Apex), g.AxisDir)))
		// G / I: one base height beyond the shaft point, along the axis.
		g.Foot = bgAdd(l.Apex, bgScale(g.AxisDir,
			l.R*math.Cos(g.Gamma)+g.BaseHeight))
		// H / J: the same offset, taken along the back-cone dedendum line.
		g.Heel = bgAdd(l.Apex2, bgScale(g.DedDir, g.BaseHeight/math.Sin(g.Gamma)))
		// K / L: where the back-cone line crosses the shaft axis.
		g.Center = bgAdd(l.Apex2, bgScale(g.DedDir, g.VirtualPitchRadius))
		g.ToothCtr = bgAdd(g.Center, bgScale(g.DedDir, in.ToothSpacing))
	}

	// Maximum Face Width, from the solved A, B, C, D, H, J.
	dp := bgPointLine(l.Pinion.Shaft, l.Pinion.Ded, l.Pinion.Heel)
	dg := bgPointLine(l.Driving.Shaft, l.Driving.Ded, l.Driving.Heel)
	l.MaxFaceWidth = bgBoundFactor * math.Min(dp, dg)
	if in.FaceWidth > 0 {
		l.FaceWidth = in.FaceWidth
	} else {
		l.FaceWidth = math.Min(l.ConeDist/6, l.MaxFaceWidth)
	}

	// Toe radii and the toe window.
	for _, g := range []*bgMember{&l.Pinion, &l.Driving} {
		r := g.PitchDia / 2
		g.ToeRadiusCeiling = (r - bgDedendumFactor*m*math.Cos(g.Gamma)) * (1 - l.FaceWidth/l.R)
		auto := r - l.FaceWidth/math.Sin(g.Gamma)
		switch {
		case g == &l.Pinion && in.PinionToeRadius > 0:
			g.ToeRadius = in.PinionToeRadius
		case g == &l.Driving && in.DrivingToeRadius > 0:
			g.ToeRadius = in.DrivingToeRadius
		default:
			g.ToeRadius = auto
		}
		g.ToeLimit = l.ApexToDed - g.ToeRadius/math.Sin(g.RootConeAngle)
	}
	l.RootLength0 = l.FaceWidth * l.ApexToDed / l.R
	window := math.Min(l.Pinion.ToeLimit, l.Driving.ToeLimit) - l.RootLength0
	l.RootLength = l.RootLength0 + (in.ToeExtension/100)*bgToeExtCap*window

	for _, g := range []*bgMember{&l.Pinion, &l.Driving} {
		g.Toe = bgAdd(l.Apex, bgScale(g.RootDir, l.ApexToDed-l.RootLength))
		slide := (bgPointLine(g.Toe, l.Apex, bgAdd(l.Apex, g.AxisDir)) - g.ToeRadius) / math.Cos(g.Gamma)
		g.ToeInner = bgAdd(g.Toe, bgScale(g.DedDir, slide))
		g.FrontFoot = bgAdd(l.Apex, bgScale(g.AxisDir, bgDot(bgSub(g.ToeInner, l.Apex), g.AxisDir)))
		if !in.BoreEnable {
			g.BoreDia = 0
			continue
		}
		user := in.PinionBore
		if g == &l.Driving {
			user = in.DrivingBore
		}
		if user > 0 {
			g.BoreDia = user
		} else {
			g.BoreDia = g.PitchDia / 4
		}
	}
	return l
}

// bgResolveBase applies a gear's own two base-height bounds to a value, the way
// input validation does: a fallback below the minimum is raised, one above the
// maximum is capped.
func bgResolveBase(user, fallback float64, g bgMember) float64 {
	v := fallback
	if user > 0 {
		v = user
	}
	return math.Min(math.Max(v, g.MinBaseHeight), g.MaxBaseHeight)
}

// bgEmbedded reports the spur drawer's embedded verdict for a virtual tooth
// count: the flank starts inside the root circle, so no flank-to-root lines are
// drawn and the tooth loop holds 4 curves rather than 6.
func bgEmbedded(module, teeth float64) bool {
	pitch := module * teeth / 2
	base := pitch * math.Cos(bgPressureAngle)
	root := (module*teeth - 2*bgDedendumFactor*module) / 2
	return base < root
}

// bgWantLines is the line count find_profile_by_curve_counts is asked for:
// 0 when the tooth is embedded, 2 when it is not. Never "0 or 2".
func bgWantLines(embedded bool) int {
	if embedded {
		return 0
	}
	return 2
}

// bgPointLine is the perpendicular distance from p to the line through a and b.
func bgPointLine(p, a, b bgPt) float64 {
	d := bgUnit(bgSub(b, a))
	return math.Abs(bgCross(d, bgSub(p, a)))
}

// bgStation is a point's distance from the apex along this gear's shaft axis.
func (l bgLattice) bgStation(g bgMember, p bgPt) float64 {
	return bgDot(bgSub(p, l.Apex), g.AxisDir)
}

// bgRadius is a point's perpendicular distance from this gear's shaft axis —
// the radius the revolve sweeps it to.
func (l bgLattice) bgRadius(g bgMember, p bgPt) float64 {
	return math.Abs(bgCross(g.AxisDir, bgSub(p, l.Apex)))
}

// bgConeDistance is a point's distance from the apex along this gear's ROOT
// cone element Apex->C / Apex->D — §3a's distAlong.
func (l bgLattice) bgConeDistance(g bgMember, p bgPt) float64 {
	return bgDot(bgSub(p, l.Apex), g.RootDir)
}

// bgHexagon returns this gear's profile-sketch hexagon in draw order,
// A' -> G -> H -> C -> M -> N (B' -> I -> J -> D -> O -> P).
func (l bgLattice) bgHexagon(g bgMember) []bgPt {
	return []bgPt{g.FrontFoot, g.Foot, g.Heel, g.Ded, g.Toe, g.ToeInner}
}

// bgSide picks the member a case's gearSide names.
func (l bgLattice) bgSide() bgMember {
	if l.In.Driving {
		return l.Driving
	}
	return l.Pinion
}

// bgRevolvedVolume is Pappus on the hexagon: the volume the profile sweeps
// about this gear's shaft axis, from the polygon's first moment about it.
func (l bgLattice) bgRevolvedVolume(g bgMember) float64 {
	poly := l.bgHexagon(g)
	moment := 0.0
	for i := range poly {
		j := (i + 1) % len(poly)
		xi, ri := l.bgStation(g, poly[i]), l.bgRadius(g, poly[i])
		xj, rj := l.bgStation(g, poly[j]), l.bgRadius(g, poly[j])
		moment += (xi*rj - xj*ri) * (ri + rj)
	}
	return 2 * math.Pi * math.Abs(moment) / 6
}

// bgFrustum is the volume a straight profile edge sweeps about the axis,
// between two stations — the band the revolve substitution builds.
func bgFrustum(x0, r0, x1, r1 float64) float64 {
	return math.Pi / 3 * math.Abs(x1-x0) * (r0*r0 + r0*r1 + r1*r1)
}

// bgClose fails when two readings differ by more than tol.
func bgClose(t *testing.T, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %.9g, want %.9g (tolerance %.2e)", what, got, want, tol)
	}
}

func bgCloseTB(t testing.TB, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %.9g, want %.9g (tolerance %.2e)", what, got, want, tol)
	}
}
