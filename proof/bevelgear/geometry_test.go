// Package bevelgear_test proves the bevel gear pair's build, one function per
// step of spec/bevelgear/steps.md.
//
// This file holds the geometry every step shares: the dialog values a case
// carries, the resolution and validation pass the generator runs before it
// draws anything, and the closed-form §2 lattice the sketch and solid steps are
// both measured against.
//
// # Units
//
// Everything here is in millimetres and radians. The generated module works in
// Fusion internal units (cm, radians) and converts every Module-derived length
// with to_cm; that conversion is a transcription concern the step list carries,
// and it changes no shape, so the proof stays in the units the spec states its
// formulas in.
//
// # The two lengths called "cone distance"
//
// coneDistance is the diagonal of the two pitch diameters, which the Face Width
// default and the bore's through-cut extent are written against. pitchConeR is
// the real apex-to-heel length along the pitch cone, R, which seeds the
// along-shaft lengths and the back-cone virtual radii. They coincide as
// coneDistance = 2*R only at Shaft Angle 90 degrees, so nothing here uses one
// where the spec names the other.
//
// # The per-gear axial frame
//
// Each gear is described in its own two-dimensional axial frame: z is the
// distance from the Apex measured along that gear's shaft axis, and rho is the
// perpendicular distance from that axis. The Apex is the origin of both gears'
// frames. Every station and radius below, and every hexagon vertex, is written
// in that frame, because it is the frame the revolve, the pattern and the
// conical trims all measure in.
package bevelgear_test

import (
	"fmt"
	"math"
	"sort"
)

// bvPressureAngle is the pressure angle the borrowed spur tooth drawer uses.
// It is not a bevel dialog input: VirtualSpurProxy defaults it to 20 degrees
// and bevel never overrides it.
const bvPressureAngle = 20 * math.Pi / 180

// bvInvoluteSteps is the proxy's InvoluteSteps default, the number of samples
// each involute flank is drawn from.
const bvInvoluteSteps = 15

// bvHandRight and bvHandLeft are the two list-item strings of the Hand of
// Spiral dropdown, carried here as the signs the cutter-centre formula uses.
const (
	bvHandRight = +1.0
	bvHandLeft  = -1.0
)

// bvCrownPerRad is _CROWN_PER_RAD, the class constant that scales the
// lengthwise crown relief by the per-end peak twist. 0 disables the crown.
const bvCrownPerRad = 0.5

// bvPinionMeshPhaseTeeth is _PINION_MESH_PHASE_TEETH, the pinion's extra mesh
// rotation expressed in tooth fractions. It is 0, so the pinion takes no mesh
// rotation at all and rotate_body_about_edge absorbs the zero.
const bvPinionMeshPhaseTeeth = 0.0

// bvToeExtensionReach is the 0.99 the Toe Extension stops short at. At the Toe
// Limit itself the toe face has zero length, so the revolved body carries no
// cone at its toe end and the conical toe trim has no face to find.
const bvToeExtensionReach = 0.99

// bvSlices is the fixed number of cross-section cut planes the spiral slice
// step uses. It is not user-configurable.
const bvSlices = 8

// bvInputs is one dialog's worth of values, in the units the dialog presents
// them in: lengths in millimetres, angles in degrees, Toe Extension as a plain
// percentage.
type bvInputs struct {
	Module            float64
	ShaftAngleDeg     float64
	DrivingTeeth      float64
	PinionTeeth       float64
	DrivingBaseHeight float64 // 0 means unspecified
	PinionBaseHeight  float64 // 0 means unspecified
	BoreEnable        bool
	DrivingBore       float64 // 0 means auto
	PinionBore        float64 // 0 means auto
	FaceWidth         float64 // 0 means auto
	ToothSpacing      float64
	SpiralAngleDeg    float64
	HandSign          float64 // +1 Right, -1 Left
	CutterRadius      float64 // 0 means auto
	ToeExtension      float64 // percent, [0, 100]
	DrivingToeRadius  float64 // 0 means auto
	PinionToeRadius   float64 // 0 means auto
}

// bvSide is one gear of the pair, resolved.
type bvSide struct {
	Label            string
	Teeth            float64
	PitchDia         float64
	Gamma            float64 // pitch cone half-angle, radians
	MinTeeth         float64
	MinBaseHeight    float64
	MaxBaseHeight    float64
	BaseHeight       float64
	Bore             float64
	ToeRadius        float64
	ToeRadiusCeiling float64
	ToeLimit         float64
	VirtualTeeth     float64
}

// bvDesign is a fully resolved pair: every derived value the build needs, and
// every rejection the resolution pass raised.
type bvDesign struct {
	In               bvInputs
	PPD              float64 // Pinion Gear Pitch Diameter
	DPD              float64 // Driving Gear Pitch Diameter
	ConeDistance     float64 // the diagonal of the two pitch diameters
	PitchConeR       float64 // R, the apex-to-heel length along the pitch cone
	MaxShaftAngleDeg float64
	Pinion           bvSide
	Driving          bvSide
	MaxFaceWidth     float64
	FaceWidth        float64
	RootLength0      float64 // |Ded->Toe| at Toe Extension 0
	RootLength       float64 // |Ded->Toe| at the resolved Toe Extension
	Rejections       []string
}

// bvRadians converts the dialog's degrees to the radians every formula uses.
func bvRadians(deg float64) float64 { return deg * math.Pi / 180 }

// bvDegrees is its inverse, used where the spec states a bound in degrees.
func bvDegrees(rad float64) float64 { return rad * 180 / math.Pi }

// bvDedendum is 1.25 modules, the dedendum every bevel formula uses.
func bvDedendum(module float64) float64 { return 1.25 * module }

// bvResolve runs the resolution and validation pass in the order the spec
// fixes, and returns what the build would use. It never stops at the first
// rejection: a case table wants every problem a configuration has, and the
// order below is the spec's only because the later bounds need the earlier
// values, not because a caller should read only the first.
//
// The Maximum Face Width is the one bound that cannot be resolved here in
// Fusion, because it is read from the §2 sketch's solved geometry. Its closed
// form is carried here and stepGearProfiles is where the two are held to each
// other.
func bvResolve(in bvInputs) bvDesign {
	d := bvDesign{In: in}
	reject := func(format string, args ...any) {
		d.Rejections = append(d.Rejections, fmt.Sprintf(format, args...))
	}

	if in.Module <= 0 {
		reject("Module must be above 0")
	}
	if in.DrivingTeeth < 3 {
		reject("Driving Gear Teeth must be at least 3")
	}
	if in.PinionTeeth < 3 {
		reject("Pinion Gear Teeth must be at least 3")
	}

	d.PPD = in.Module * in.PinionTeeth
	d.DPD = in.Module * in.DrivingTeeth
	d.ConeDistance = math.Hypot(d.DPD, d.PPD)

	// The cone-angle limit is a hard singularity: at it, one gear's pitch cone
	// angle reaches 90 degrees, R*cos(gamma) passes through zero and changes
	// sign, and the along-shaft seed points backwards. The 150 degree cap on
	// top of it is a practical ceiling on the figure, not a measured one, and
	// it is the inclusive half of the bound.
	coneLimitDeg := bvDegrees(math.Acos(-math.Min(d.DPD, d.PPD) / math.Max(d.DPD, d.PPD)))
	d.MaxShaftAngleDeg = math.Min(coneLimitDeg, 150)
	switch {
	case in.ShaftAngleDeg < 30:
		reject("Shaft Angle must be at least 30 deg")
	case coneLimitDeg <= 150 && in.ShaftAngleDeg >= coneLimitDeg:
		reject("Shaft Angle must be below %.4f deg for these tooth counts", coneLimitDeg)
	case coneLimitDeg > 150 && in.ShaftAngleDeg > 150:
		reject("Shaft Angle must be at most 150 deg")
	}

	sigma := bvRadians(in.ShaftAngleDeg)
	// atan2 rather than atan so an obtuse Shaft Angle, where the denominator
	// goes negative, still lands in the right quadrant.
	gammaP := math.Atan2(math.Sin(sigma)*d.PPD, d.DPD+d.PPD*math.Cos(sigma))
	gammaG := sigma - gammaP
	d.PitchConeR = (d.PPD / 2) / math.Sin(gammaP)

	d.Pinion = bvSide{Label: "Pinion", Teeth: in.PinionTeeth, PitchDia: d.PPD, Gamma: gammaP}
	d.Driving = bvSide{Label: "Driving", Teeth: in.DrivingTeeth, PitchDia: d.DPD, Gamma: gammaG}

	// 1. The Minimum Teeth floor, per gear with that gear's own gamma. It is
	// exactly the statement that the base-height window below is non-empty, so
	// it runs first and step 2 never has to describe an empty window.
	for _, s := range []*bvSide{&d.Pinion, &d.Driving} {
		s.MinTeeth = 5.27 * math.Cos(s.Gamma)
		if s.Teeth < s.MinTeeth {
			reject("%s Gear Teeth must be at least %.4f at this Shaft Angle", s.Label, s.MinTeeth)
		}
	}

	// 2. The two base-height bounds, per gear, both closed form.
	for _, s := range []*bvSide{&d.Pinion, &d.Driving} {
		r := s.PitchDia / 2
		s.MinBaseHeight = 1.05 * bvDedendum(in.Module) * math.Sin(s.Gamma)
		s.MaxBaseHeight = 0.95 * (r - bvDedendum(in.Module)*math.Cos(s.Gamma)) * math.Tan(s.Gamma)
	}

	// The driving side resolves first, because the pinion's fallback scales the
	// driving side's RESOLVED value — after its own fallback and after its own
	// cap — and not the raw driving input.
	d.Driving.BaseHeight = bvApplyBaseHeight(&d.Driving, in.DrivingBaseHeight,
		in.Module*in.DrivingTeeth/8, reject)
	d.Pinion.BaseHeight = bvApplyBaseHeight(&d.Pinion, in.PinionBaseHeight,
		d.Driving.BaseHeight*(in.PinionTeeth/in.DrivingTeeth), reject)

	// The Maximum Face Width is 0.95 times the smaller of the perpendicular
	// distance from A to line C->H and from B to line D->J. Both reduce to
	// R*sin(gamma)^2 for that gear, so the bound is 0.95*R*min of the two —
	// equivalently 0.95*min(PPD,DPD)^2/(4R), which at Shaft Angle 90 is the
	// 0.95*min^2/(2*Cone Distance) the spec states. The SMALLER pitch diameter
	// binds, and it is not always the pinion's.
	pinionReach := d.PitchConeR * math.Sin(gammaP) * math.Sin(gammaP)
	drivingReach := d.PitchConeR * math.Sin(gammaG) * math.Sin(gammaG)
	d.MaxFaceWidth = 0.95 * math.Min(pinionReach, drivingReach)
	if in.FaceWidth > 0 {
		if in.FaceWidth > d.MaxFaceWidth {
			reject("Face Width must be at most %.4f mm", d.MaxFaceWidth)
		}
		d.FaceWidth = in.FaceWidth
	} else {
		d.FaceWidth = math.Min(d.ConeDistance/6, d.MaxFaceWidth)
	}

	// The root length at Toe Extension 0 is the resolved Face Width re-measured
	// along the root element rather than perpendicular to the pitch line.
	apexToDed := math.Hypot(d.PitchConeR, bvDedendum(in.Module))
	d.RootLength0 = d.FaceWidth * apexToDed / d.PitchConeR

	// Toe radii, their ceilings and the toe limits, per gear.
	gammaRoot := bvRootConeAngle(d)
	for _, s := range []*bvSide{&d.Pinion, &d.Driving} {
		r := s.PitchDia / 2
		s.ToeRadiusCeiling = (r - bvDedendum(in.Module)*math.Cos(s.Gamma)) *
			(1 - d.FaceWidth/d.PitchConeR)
		user := in.PinionToeRadius
		if s.Label == "Driving" {
			user = in.DrivingToeRadius
		}
		switch {
		case user < 0:
			reject("%s Gear Toe Radius must not be negative", s.Label)
			s.ToeRadius = r - d.FaceWidth/math.Sin(s.Gamma)
		case user == 0:
			s.ToeRadius = r - d.FaceWidth/math.Sin(s.Gamma)
		default:
			if user >= s.ToeRadiusCeiling {
				reject("%s Gear Toe Radius must be below %.4f mm", s.Label, s.ToeRadiusCeiling)
			}
			s.ToeRadius = user
		}
		s.ToeLimit = apexToDed - s.ToeRadius/math.Sin(gammaRoot)
		s.VirtualTeeth = math.Floor(2 * (r / math.Cos(s.Gamma)) / in.Module)
	}

	// The Toe Extension reaches 0.99 of the way from the Toe Extension 0 root
	// length to the SMALLER of the two gears' Toe Limits: the pair shares one
	// root length and the other gear simply stops short of its own X.
	switch {
	case in.ToeExtension < 0 || in.ToeExtension > 100:
		reject("Toe Extension must be between 0 and 100")
		d.RootLength = d.RootLength0
	case in.ToeExtension == 0:
		d.RootLength = d.RootLength0
	default:
		blocked := false
		for _, s := range []bvSide{d.Pinion, d.Driving} {
			if s.ToeLimit <= d.RootLength0 {
				reject("%s Gear Toe Radius must come below %.4f mm before Toe Extension can rise above 0",
					s.Label, s.ToeRadiusCeiling)
				blocked = true
			}
		}
		if blocked {
			d.RootLength = d.RootLength0
			break
		}
		limit := math.Min(d.Pinion.ToeLimit, d.Driving.ToeLimit)
		d.RootLength = d.RootLength0 + (in.ToeExtension/100)*bvToeExtensionReach*(limit-d.RootLength0)
	}

	// Bores. A zero means auto-calculate from that gear's own pitch diameter,
	// and the whole branch is skipped when Enable Bore is unchecked.
	d.Pinion.Bore = bvBoreOf(in.BoreEnable, in.PinionBore, d.PPD)
	d.Driving.Bore = bvBoreOf(in.BoreEnable, in.DrivingBore, d.DPD)

	if in.ToothSpacing < 0 {
		reject("Tooth Spacing must not be negative")
	}
	if in.SpiralAngleDeg < 0 || in.SpiralAngleDeg >= 60 {
		reject("Mean Spiral Angle must be at least 0 deg and below 60 deg")
	}
	if in.CutterRadius < 0 {
		reject("Cutter Radius must not be negative")
	}
	sort.Strings(d.Rejections)
	return d
}

// bvApplyBaseHeight applies one gear's two base-height bounds in both
// directions: a fallback below the minimum is raised, a fallback above the
// maximum is capped, and a user value outside either end is rejected naming the
// bound it broke.
func bvApplyBaseHeight(s *bvSide, user, fallback float64, reject func(string, ...any)) float64 {
	if user > 0 {
		switch {
		case user < s.MinBaseHeight:
			reject("%s Gear Base Height must be at least %.4f mm", s.Label, s.MinBaseHeight)
		case user > s.MaxBaseHeight:
			reject("%s Gear Base Height must be at most %.4f mm", s.Label, s.MaxBaseHeight)
		}
		return user
	}
	if user < 0 {
		reject("%s Gear Base Height must not be negative", s.Label)
	}
	return math.Max(s.MinBaseHeight, math.Min(fallback, s.MaxBaseHeight))
}

// bvBoreOf resolves one gear's bore diameter. Zero means auto: a quarter of
// that gear's own pitch diameter. Enable Bore unchecked means no bore at all,
// reported as a zero diameter, and the per-gear inputs are then ignored.
func bvBoreOf(enabled bool, user, pitchDia float64) float64 {
	if !enabled {
		return 0
	}
	if user > 0 {
		return user
	}
	return pitchDia / 4
}

// bvRootConeAngle is the root cone angle's offset from the pitch cone angle:
// atan(1.25*Module / R), the dedendum angle. Both gears share it, because both
// R and the dedendum are shared.
func bvDedendumAngle(d bvDesign) float64 {
	return math.Atan2(bvDedendum(d.In.Module), d.PitchConeR)
}

// bvRootConeAngle is the pinion's root cone angle. gamma_root = gamma - the
// dedendum angle, and it is the angle the Toe Limit's closed form measures the
// root element at. The two gears have different gammas, so bvSideRootAngle is
// what a per-gear caller wants; this one is kept for the pinion because the
// pinion is the gear the Toe Limit formula was written against.
func bvRootConeAngle(d bvDesign) float64 { return d.Pinion.Gamma - bvDedendumAngle(d) }

// bvSideRootAngle is one gear's own root cone angle.
func bvSideRootAngle(d bvDesign, s bvSide) float64 { return s.Gamma - bvDedendumAngle(d) }

// bvPt is a point in one gear's axial frame: Z is the distance from the Apex
// along that gear's shaft axis, Rho the perpendicular distance from it.
type bvPt struct{ Z, Rho float64 }

// bvHex holds the six hexagon vertices the profile sketch draws and the
// revolve sweeps, in the draw order the spec's table fixes:
// A' -> G -> H -> C -> M -> N -> A' for the pinion, and the driving gear's
// B' -> I -> J -> D -> O -> P -> B' with the same roles in the same order.
type bvHex struct {
	Foot  bvPt // A' / B' — the front face's foot, on the shaft axis
	Heel  bvPt // G  / I  — the heel end of the shaft-axis edge
	Rim   bvPt // H  / J  — the heel edge's outer end
	Ded   bvPt // C  / D  — the dedendum corner
	Toe   bvPt // M  / O  — the toe edge's outer end, on the root element
	Inner bvPt // N  / P  — the inner toe corner, at the Toe Radius
}

// bvHexagonOf places one gear's six hexagon vertices from the resolved design.
//
// Every vertex below is the closed form of where the §2 constraint net puts
// it, and stepGearProfiles is where the drawn lattice is held to these numbers.
func bvHexagonOf(d bvDesign, s bvSide) bvHex {
	R, ded := d.PitchConeR, bvDedendum(d.In.Module)
	sinG, cosG := math.Sin(s.Gamma), math.Cos(s.Gamma)

	// Apex 2 sits at R along the pitch line; the dedendum line through it runs
	// perpendicular to that line, in the direction (sin gamma, -cos gamma).
	apex2 := bvPt{Z: R * cosG, Rho: R * sinG}
	dedCorner := bvPt{Z: apex2.Z + ded*sinG, Rho: apex2.Rho - ded*cosG}

	// H is on that same dedendum line, at the station the base-height offset
	// dimension drives it to: the offset is measured from Apex 2's plane, the
	// plane through Apex 2 perpendicular to this gear's shaft axis.
	rim := bvPt{Z: apex2.Z + s.BaseHeight, Rho: apex2.Rho - s.BaseHeight*cosG/sinG}
	heel := bvPt{Z: rim.Z, Rho: 0}

	// M is on the root element Apex->C, one root length short of C.
	rootAngle := bvSideRootAngle(d, s)
	apexToDed := math.Hypot(R, ded)
	toeAlong := apexToDed - d.RootLength
	toe := bvPt{Z: toeAlong * math.Cos(rootAngle), Rho: toeAlong * math.Sin(rootAngle)}

	// N rides the Toe Radius: it is where the toe line, parallel to C->H
	// through M, reaches that perpendicular distance from the shaft axis.
	slide := (toe.Rho - s.ToeRadius) / cosG
	inner := bvPt{Z: toe.Z + slide*sinG, Rho: s.ToeRadius}
	foot := bvPt{Z: inner.Z, Rho: 0}

	return bvHex{Foot: foot, Heel: heel, Rim: rim, Ded: dedCorner, Toe: toe, Inner: inner}
}

// Order returns the six vertices in the table's draw order, so a caller can
// walk the closed hexagon without restating the order.
func (h bvHex) Order() [6]bvPt {
	return [6]bvPt{h.Foot, h.Heel, h.Rim, h.Ded, h.Toe, h.Inner}
}

// bvBandVolume is the volume a straight profile segment sweeps about the shaft
// axis: the truncated cone between the two rings, pi*|dz|*(r1^2+r1*r2+r2^2)/3.
func bvBandVolume(a, b bvPt) float64 {
	return math.Pi * math.Abs(b.Z-a.Z) * (a.Rho*a.Rho + a.Rho*b.Rho + b.Rho*b.Rho) / 3
}

// bvRevolvedVolume is the frustum's volume read straight off the hexagon, by
// Pappus: pi times the loop integral of rho^2 dz. The two shaft-axis-parallel
// faces contribute nothing because dz is zero across them, and the shaft-axis
// edge contributes nothing because rho is zero along it, which leaves the three
// bands the heel, root and toe edges sweep — signed, so the toe dish subtracts.
func bvRevolvedVolume(h bvHex) float64 {
	order := h.Order()
	sum := 0.0
	for i := range order {
		a, b := order[i], order[(i+1)%len(order)]
		sum += (b.Z - a.Z) * (a.Rho*a.Rho + a.Rho*b.Rho + b.Rho*b.Rho) / 3
	}
	return math.Pi * math.Abs(sum)
}

// bvHalfAngle is a band's cone half-angle: the angle its swept surface makes
// with the shaft axis, as a magnitude, so it does not depend on which end of
// the edge the caller names first.
func bvHalfAngle(a, b bvPt) float64 {
	return math.Atan2(math.Abs(b.Rho-a.Rho), math.Abs(b.Z-a.Z))
}

// bvBackConeAngle is the half-angle of the back-cone family, the cones the heel
// edge and the toe edge both sweep. The dedendum line is perpendicular to the
// pitch line, so its cones stand at 90 degrees minus the pitch cone angle.
func bvBackConeAngle(s bvSide) float64 { return math.Pi/2 - s.Gamma }

// bvVirtualPitchRadius is one gear's back-cone (Tredgold) pitch radius,
// (PitchDiameter/2)/cos(gamma). The virtual tooth number floors twice this over
// Module, and neither depends on Tooth Spacing.
func bvVirtualPitchRadius(s bvSide) float64 { return (s.PitchDia / 2) / math.Cos(s.Gamma) }

// bvToothCentre is K (pinion) / L (driving): the point where the dedendum line
// Apex2->C extended meets the shaft axis, which is the back cone's own apex, at
// R/cos(gamma) along the axis.
func bvToothCentre(d bvDesign, s bvSide) bvPt {
	return bvPt{Z: d.PitchConeR / math.Cos(s.Gamma), Rho: 0}
}

// bvSpacedToothCentre is K' / L': K shifted along the dedendum line by Tooth
// Spacing, away from the lower corner C / D. At Tooth Spacing 0 it is K itself,
// which is why the spec builds no geometry for it there and reuses C->K.
func bvSpacedToothCentre(d bvDesign, s bvSide) bvPt {
	k := bvToothCentre(d, s)
	sinG, cosG := math.Sin(s.Gamma), math.Cos(s.Gamma)
	return bvPt{Z: k.Z + d.In.ToothSpacing*sinG, Rho: k.Rho - d.In.ToothSpacing*cosG}
}
