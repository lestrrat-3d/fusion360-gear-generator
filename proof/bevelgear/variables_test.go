// This file holds the dialog's values as a proof case carries them, the case
// tables every step draws from, and the resolution step that turns a dialog
// into the numbers the rest of the build uses.
//
// The tables are split in two for one reason the spec gives: a solid case may
// not run at Module 1, because decad's mesh bound has an absolute floor and a
// figure that small brings every measurement inside it, so the gate reports
// Suspect on geometry that is correct. Module is a pure scale on this figure,
// so the solid tables run at Module 4 through 8 and prove the same shape. The
// sketch tables are unaffected and stay at the dialog's own default.
package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// The keys a case carries. Angles are in degrees and lengths in millimetres,
// exactly as the dialog presents them. The spellings are the ones the package's
// hand-written drawing helpers already use, so a figure built from a case here
// and one built from those helpers read the same map.
const (
	bvpModule        = "module"
	bvpShaftAngle    = "shaftAngleDeg"
	bvpDrivingTeeth  = "drivingTeeth"
	bvpPinionTeeth   = "pinionTeeth"
	bvpDrivingBase   = "drivingBaseHeight"
	bvpPinionBase    = "pinionBaseHeight"
	bvpBoreEnable    = "boreEnable"
	bvpDrivingBore   = "drivingBore"
	bvpPinionBore    = "pinionBore"
	bvpFaceWidth     = "faceWidth"
	bvpToothSpacing  = "toothSpacing"
	bvpSpiralAngle   = "spiralAngleDeg"
	bvpHand          = "hand"
	bvpCutterRadius  = "cutterRadius"
	bvpToeExtension  = "toeExtension"
	bvpDrivingToeR   = "drivingToeRadius"
	bvpPinionToeR    = "pinionToeRadius"
	bvpSide          = "gearSide"
	bvpRejected      = "rejected"
	bvpLatticeRefuse = "latticeRefused"
)

// bvDialog is the shipped dialog: every default the input table declares, with
// Enable Bore checked and every "0 means auto" field left at 0.
func bvDialog() map[string]float64 {
	return map[string]float64{
		bvpModule:       1,
		bvpShaftAngle:   90,
		bvpDrivingTeeth: 31,
		bvpPinionTeeth:  31,
		bvpDrivingBase:  0,
		bvpPinionBase:   0,
		bvpBoreEnable:   1,
		bvpDrivingBore:  0,
		bvpPinionBore:   0,
		bvpFaceWidth:    0,
		bvpToothSpacing: 0,
		bvpSpiralAngle:  35,
		bvpHand:         bvHandRight,
		bvpCutterRadius: 0,
		bvpToeExtension: 0,
		bvpDrivingToeR:  0,
		bvpPinionToeR:   0,
		bvpSide:         0,
		bvpRejected:     0,
	}
}

// bvWith is the shipped dialog with named fields overridden.
func bvWith(over map[string]float64) map[string]float64 {
	p := bvDialog()
	for k, v := range over {
		p[k] = v
	}
	return p
}

// bvSolidWith is bvWith at Module 4, the smallest size the solid gate's mesh
// bound clears, unless the override names a Module of its own.
func bvSolidWith(over map[string]float64) map[string]float64 {
	p := bvDialog()
	p[bvpModule] = 4
	for k, v := range over {
		p[k] = v
	}
	return p
}

// bvInputsOf reads a case back into the dialog's own shape.
func bvInputsOf(p map[string]float64) bvInputs {
	return bvInputs{
		Module:            p[bvpModule],
		ShaftAngleDeg:     p[bvpShaftAngle],
		DrivingTeeth:      p[bvpDrivingTeeth],
		PinionTeeth:       p[bvpPinionTeeth],
		DrivingBaseHeight: p[bvpDrivingBase],
		PinionBaseHeight:  p[bvpPinionBase],
		BoreEnable:        p[bvpBoreEnable] != 0,
		DrivingBore:       p[bvpDrivingBore],
		PinionBore:        p[bvpPinionBore],
		FaceWidth:         p[bvpFaceWidth],
		ToothSpacing:      p[bvpToothSpacing],
		SpiralAngleDeg:    p[bvpSpiralAngle],
		HandSign:          p[bvpHand],
		CutterRadius:      p[bvpCutterRadius],
		ToeExtension:      p[bvpToeExtension],
		DrivingToeRadius:  p[bvpDrivingToeR],
		PinionToeRadius:   p[bvpPinionToeR],
	}
}

// bvDesignOf resolves a case and fails it when the resolution pass disagrees
// with the case's own expectation, so a case can never quietly stop exercising
// the branch it was written for.
func bvDesignOf(t testing.TB, p map[string]float64) bvDesign {
	t.Helper()
	d := bvResolve(bvInputsOf(p))
	if p[bvpRejected] != 0 {
		if len(d.Rejections) == 0 {
			t.Fatalf("this case is marked rejected and the resolution pass accepted it")
		}
		return d
	}
	if len(d.Rejections) != 0 {
		t.Fatalf("resolution refused an accepted case: %v", d.Rejections)
	}
	return d
}

// bvSideOf is the gear a case names: the pinion, which is built first, or the
// driving gear, which is built second and is the one that takes the meshing
// rotation.
func bvSideOf(d bvDesign, p map[string]float64) bvSide {
	if p[bvpSide] != 0 {
		return d.Driving
	}
	return d.Pinion
}

// ------------------------------------------------------------------ tables

// bvResolveCases is the whole validation surface: each bound at both ends, each
// "0 means auto" field on both sides of its branch, and each rejection reached
// by the route that produces it.
//
// The signed inputs are covered where the spec says a value is signed. Every
// length input the dialog declares is documented as non-negative, so a negative
// case is a rejection case rather than a geometry case, and each one is here.
var bvResolveCases = []proofkit.Case{
	{Name: "defaults", Params: bvWith(nil)},
	{Name: "shaft_angle_floor_30", Params: bvWith(map[string]float64{bvpShaftAngle: 30})},
	{Name: "shaft_angle_35", Params: bvWith(map[string]float64{bvpShaftAngle: 35})},
	{Name: "shaft_angle_60", Params: bvWith(map[string]float64{bvpShaftAngle: 60})},
	{Name: "shaft_angle_120", Params: bvWith(map[string]float64{bvpShaftAngle: 120})},
	{Name: "shaft_angle_142", Params: bvWith(map[string]float64{bvpShaftAngle: 142})},
	{Name: "shaft_angle_ceiling_150_equal_teeth", Params: bvWith(map[string]float64{bvpShaftAngle: 150})},
	{Name: "ratio_31_17", Params: bvWith(map[string]float64{bvpPinionTeeth: 17})},
	{Name: "ratio_17_31", Params: bvWith(map[string]float64{bvpDrivingTeeth: 17})},
	{Name: "ratio_31_43", Params: bvWith(map[string]float64{bvpPinionTeeth: 43})},
	{Name: "low_teeth_4_4", Params: bvWith(map[string]float64{bvpDrivingTeeth: 4, bvpPinionTeeth: 4})},
	{Name: "module_4", Params: bvWith(map[string]float64{bvpModule: 4})},
	{Name: "module_8_ratio", Params: bvWith(map[string]float64{bvpModule: 8, bvpPinionTeeth: 19})},
	{Name: "base_heights_specified", Params: bvWith(map[string]float64{bvpDrivingBase: 5, bvpPinionBase: 4})},
	{Name: "face_width_specified", Params: bvWith(map[string]float64{bvpFaceWidth: 6})},
	{Name: "tooth_spacing_positive", Params: bvWith(map[string]float64{bvpToothSpacing: 0.2})},
	{Name: "toe_extension_50", Params: bvWith(map[string]float64{bvpToeExtension: 50})},
	{Name: "toe_extension_100", Params: bvWith(map[string]float64{bvpToeExtension: 100})},
	{Name: "toe_radii_specified", Params: bvWith(map[string]float64{bvpDrivingToeR: 4, bvpPinionToeR: 4})},
	{Name: "bore_disabled", Params: bvWith(map[string]float64{bvpBoreEnable: 0})},
	{Name: "bore_specified", Params: bvWith(map[string]float64{bvpDrivingBore: 6, bvpPinionBore: 5})},
	{Name: "spiral_straight", Params: bvWith(map[string]float64{bvpSpiralAngle: 0})},
	{Name: "spiral_left_hand", Params: bvWith(map[string]float64{bvpHand: bvHandLeft})},
	{Name: "spiral_59_with_cutter", Params: bvWith(map[string]float64{bvpSpiralAngle: 59, bvpCutterRadius: 30})},

	{Name: "reject_shaft_angle_below_floor", Params: bvWith(map[string]float64{
		bvpShaftAngle: 29, bvpRejected: 1})},
	{Name: "reject_shaft_angle_at_cone_limit", Params: bvWith(map[string]float64{
		bvpPinionTeeth: 17, bvpShaftAngle: 124, bvpRejected: 1})},
	{Name: "reject_shaft_angle_above_150", Params: bvWith(map[string]float64{
		bvpShaftAngle: 151, bvpRejected: 1})},
	{Name: "reject_teeth_below_blanket_floor", Params: bvWith(map[string]float64{
		bvpPinionTeeth: 2, bvpRejected: 1})},
	{Name: "reject_teeth_below_computed_floor", Params: bvWith(map[string]float64{
		bvpDrivingTeeth: 3, bvpPinionTeeth: 3, bvpRejected: 1})},
	{Name: "reject_base_height_above_max", Params: bvWith(map[string]float64{
		bvpDrivingBase: 40, bvpRejected: 1})},
	{Name: "reject_base_height_below_min", Params: bvWith(map[string]float64{
		bvpPinionBase: 0.01, bvpRejected: 1})},
	{Name: "reject_face_width_above_max", Params: bvWith(map[string]float64{
		bvpFaceWidth: 40, bvpRejected: 1})},
	{Name: "reject_toe_radius_at_ceiling", Params: bvWith(map[string]float64{
		bvpPinionToeR: 40, bvpRejected: 1})},
	{Name: "reject_negative_tooth_spacing", Params: bvWith(map[string]float64{
		bvpToothSpacing: -1, bvpRejected: 1})},
	{Name: "reject_negative_cutter_radius", Params: bvWith(map[string]float64{
		bvpCutterRadius: -1, bvpRejected: 1})},
	{Name: "reject_spiral_angle_at_60", Params: bvWith(map[string]float64{
		bvpSpiralAngle: 60, bvpRejected: 1})},
	{Name: "reject_toe_extension_above_100", Params: bvWith(map[string]float64{
		bvpToeExtension: 101, bvpRejected: 1})},
	{Name: "reject_toe_extension_with_no_room", Params: bvWith(map[string]float64{
		bvpDrivingTeeth: 45, bvpPinionTeeth: 13, bvpShaftAngle: 90,
		bvpToeExtension: 50, bvpRejected: 1})},
}

// ------------------------------------------------------- the resolution step

// stepResolveInputs proves the resolution pass every later step depends on.
//
// Two things are proved together, and they have to be, because the closed forms
// only mean anything against the figure they describe. The drawn sketch is the
// heel chain of the §2 lattice — the shaft axis, the A->Apex2 drop, the pitch
// line, the dedendum line and the base-height offset that drives G->H — which
// is the sub-lattice every base-height bound is a statement about. The
// assertions then hold each bound to what that figure actually does.
//
// The figure is drawn in one gear's axial frame rather than in the §2 sketch's
// own frame: the bounds are per-gear and say nothing about the other gear, so
// the second shaft would add geometry without adding a check. stepGearProfiles
// draws the whole lattice, both shafts and all.
func stepResolveInputs(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := bvDesignOf(t, p)
	if p[bvpRejected] != 0 {
		// A refused dialog never reaches geometry, so the figure below would be
		// drawn from values the generator would not have used.
		proofkit.Unmodelled(t, "the resolution pass refuses this dialog: %v", d.Rejections)
	}
	side := bvSideOf(d, p)
	assertResolutionClosedForms(t, d, side)

	R, ded := d.PitchConeR, bvDedendum(d.In.Module)
	sinG, cosG := math.Sin(side.Gamma), math.Cos(side.Gamma)
	bh := side.BaseHeight

	proofkit.Step(t, "apex and shaft axis")
	apex := s.CreatePoint(0, 0)
	apex.SetName("Apex")
	s.Fix(apex)
	a := s.CreatePoint(R*cosG, 0)
	a.SetName("A")
	shaftAxis := s.CreateLine(apex, a)
	shaftAxis.SetConstruction(true)
	// In §2 this length is DRIVEN by the Apex 2 closure with the other shaft
	// and carries no dimension ([BEVEL-F-DRIVEN-DIMS]). This figure holds one
	// gear only, so there is no second shaft to close it and the station has to
	// be dimensioned here. That is the one deviation this step makes from the
	// lattice it is a piece of, and stepGearProfiles is where the undimensioned
	// form is proved.
	s.AddConstraint(
		sketch.NewHorizontalDistance(apex, a, R*cosG),
		sketch.NewVerticalDistance(apex, a, 0),
	)

	proofkit.Step(t, "the A->Apex2 drop, dimensioned at the pitch radius")
	apex2 := s.CreatePoint(R*cosG, R*sinG)
	apex2.SetName("Apex2")
	drop := s.CreateLine(a, apex2)
	drop.SetConstruction(true)
	s.AddConstraint(
		bvRightAngle(shaftAxis, drop),
		sketch.NewDistance(a, apex2, side.PitchDia/2),
	)

	proofkit.Step(t, "the pitch line, whose cone angle is driven")
	pitchLine := s.CreateLine(apex, apex2)
	pitchLine.SetConstruction(true)

	proofkit.Step(t, "the dedendum line, perpendicular to the pitch line")
	dedC := s.CreatePoint(apex2.X()+ded*sinG, apex2.Y()-ded*cosG)
	dedC.SetName("C")
	dedLine := s.CreateLine(apex2, dedC)
	dedLine.SetConstruction(true)
	s.AddConstraint(
		bvRightAngle(pitchLine, dedLine),
		sketch.NewDistance(apex2, dedC, ded),
	)

	proofkit.Step(t, "G->H, held off the drop by the resolved base height")
	g := s.CreatePoint(R*cosG+bh, 0)
	g.SetName("G")
	h := s.CreatePoint(R*cosG+bh, R*sinG-bh*cosG/sinG)
	h.SetName("H")
	gh := s.CreateLine(g, h)
	gh.SetConstruction(true)
	s.AddConstraint(
		sketch.NewPointOnLine(g, shaftAxis),
		sketch.NewPointOnLine(h, dedLine),
		sketch.NewOffset(drop, gh, bvOffsetSign(drop, gh)*bh),
	)

	proofkit.Step(t, "what the drawn heel chain says about the bounds")
	assertHeelChain(t, s, d, side, apex2, dedC, h, pitchLine)
}

// bvRightAngle is the bench rendering of a Fusion perpendicular constraint.
//
// The two are not the same shape, and the difference is load-bearing. Fusion's
// addPerpendicular carries no sense: it admits both right angles, and which one
// the sketch lands on is decided by where the geometry was seeded. The engine's
// signed angle dimension admits one, so the seeded sense crosses over as the
// SIGN of the target and nothing else changes ([PB-DIM-VALUE-SEMANTICS] states
// that cross-over for a distance; an angle carries its direction the same way).
// Both are one residual row, so the arity of every count in the spec is
// preserved.
//
// This matters because proofkit's gate refuses a discrete ambiguity, and every
// perpendicular in this gear's figure sits on a two-fold branch: the whole
// lattice is defined relative to the projected centre and to directions, so
// nothing in it forbids the 180-degree-rotated answer. In Fusion the grow side
// is a seed ([BEVEL-F-GROW-SIDE] reads it off the target-plane normal, a
// one-bit direction), and on the bench the same one bit is the sign here.
func bvRightAngle(l1, l2 *sketch.Line) sketch.Constraint {
	return sketch.NewAngle(l1, l2, bvSeedSense(l1, l2)*90)
}

// bvSeedSense reports which way l2 turns from l1 as the two are seeded: +1 for
// counter-clockwise, -1 for clockwise.
func bvSeedSense(l1, l2 *sketch.Line) float64 {
	d1x, d1y := l1.End.X()-l1.Start.X(), l1.End.Y()-l1.Start.Y()
	d2x, d2y := l2.End.X()-l2.Start.X(), l2.End.Y()-l2.Start.Y()
	if d1x*d2y-d1y*d2x < 0 {
		return -1
	}
	return 1
}

// bvOffsetSign reads the side a freshly seeded destination line sits on, so the
// engine's SIGNED offset target carries the direction the Fusion dimension's
// seeded geometry carries. [PB-DIM-VALUE-SEMANTICS] is the cross-over rule: the
// sign is realized by the seed side, and only the magnitude is the dimension.
func bvOffsetSign(src, dst *sketch.Line) float64 {
	ax, ay := src.Start.X(), src.Start.Y()
	bx, by := src.End.X()-ax, src.End.Y()-ay
	n := math.Hypot(bx, by)
	side := (bx*(dst.Start.Y()-ay) - by*(dst.Start.X()-ax)) / n
	if side < 0 {
		return -1
	}
	return 1
}

// assertResolutionClosedForms holds every bound the resolution pass computes to
// the identity the spec derives it from, rather than to a second copy of the
// same arithmetic.
func assertResolutionClosedForms(t testing.TB, d bvDesign, side bvSide) {
	t.Helper()
	near := func(label string, got, want float64) {
		t.Helper()
		if math.Abs(got-want) > 1e-9*math.Max(1, math.Abs(want)) {
			t.Errorf("%s: got %.9f, want %.9f", label, got, want)
		}
	}

	// The two cone angles add to the Shaft Angle, and each gear's pitch radius
	// over its own sine is the one shared Pitch Cone Distance.
	near("gamma_p + gamma_g", d.Pinion.Gamma+d.Driving.Gamma, bvRadians(d.In.ShaftAngleDeg))
	near("R from the driving side", (d.DPD/2)/math.Sin(d.Driving.Gamma), d.PitchConeR)

	// Cone Distance is the diagonal of the two pitch diameters and never
	// depends on the Shaft Angle; it equals 2R only at Shaft Angle 90.
	near("Cone Distance", d.ConeDistance, math.Hypot(d.PPD, d.DPD))
	if math.Abs(d.In.ShaftAngleDeg-90) < 1e-12 {
		near("Cone Distance at 90 deg", d.ConeDistance, 2*d.PitchConeR)
	}

	// The Maximum Shaft Angle is where a cone angle reaches 90 degrees, so at
	// the cone limit the larger gear's tangent denominator vanishes.
	limit := bvRadians(bvDegrees(math.Acos(-math.Min(d.DPD, d.PPD) / math.Max(d.DPD, d.PPD))))
	near("cone limit turns a pitch cone inside out",
		math.Min(d.DPD+d.PPD*math.Cos(limit), d.PPD+d.DPD*math.Cos(limit)), 0)

	for _, s := range []bvSide{d.Pinion, d.Driving} {
		r := s.PitchDia / 2
		// The true crossing, where the heel edge reaches the shaft axis, and
		// the conservative bound's distance below it.
		crossing := r * math.Tan(s.Gamma)
		near(s.Label+" exact bound sits one dedendum-projection below the crossing",
			crossing-(r-bvDedendum(d.In.Module)*math.Cos(s.Gamma))*math.Tan(s.Gamma),
			bvDedendum(d.In.Module)*math.Sin(s.Gamma))
		if s.MaxBaseHeight >= crossing {
			t.Errorf("%s Maximum Base Height %.6f is not below the true crossing %.6f",
				s.Label, s.MaxBaseHeight, crossing)
		}
		// The Minimum Base Height carries H past C's own along-shaft
		// projection, by the 1.05 margin and no more.
		near(s.Label+" minimum base height",
			s.MinBaseHeight, 1.05*bvDedendum(d.In.Module)*math.Sin(s.Gamma))
		// The two bounds cross exactly at the published tooth floor's
		// constant, and 5.27 is that constant rounded UP.
		exact := 2 * (1.05*1.25/0.95 + 1.25)
		if exact > 5.27 {
			t.Errorf("the published Minimum Teeth constant 5.27 is below the exact crossing %.6f", exact)
		}
		near(s.Label+" tooth floor", s.MinTeeth, 5.27*math.Cos(s.Gamma))
	}

	// The Maximum Face Width's closed form, both spellings.
	near("Maximum Face Width",
		d.MaxFaceWidth, 0.95*math.Min(d.PPD*d.PPD, d.DPD*d.DPD)/(4*d.PitchConeR))
	if math.Abs(d.In.ShaftAngleDeg-90) < 1e-12 {
		near("Maximum Face Width at 90 deg",
			d.MaxFaceWidth, 0.95*math.Min(d.PPD*d.PPD, d.DPD*d.DPD)/(2*d.ConeDistance))
	}
	if d.FaceWidth > d.MaxFaceWidth+1e-12 {
		t.Errorf("resolved Face Width %.6f exceeds the cap %.6f", d.FaceWidth, d.MaxFaceWidth)
	}

	// The root length at Toe Extension 0 is the Face Width re-measured along
	// the root element, so it is longer by the dedendum angle's cosine.
	apexToDed := math.Hypot(d.PitchConeR, bvDedendum(d.In.Module))
	near("Root Length at Toe Extension 0", d.RootLength0, d.FaceWidth*apexToDed/d.PitchConeR)
	if d.RootLength0 < d.FaceWidth {
		t.Errorf("Root Length %.6f is below the Face Width %.6f it is measured from",
			d.RootLength0, d.FaceWidth)
	}
	near("the dedendum angle", bvDedendumAngle(d),
		math.Atan(bvDedendum(d.In.Module)/d.PitchConeR))

	// A defaulted Toe Radius is what makes Toe Extension 0 reproduce the
	// profile the gear had before the input existed: the inner toe corner then
	// lands exactly at that gear's A / B station on the shaft axis.
	hex := bvHexagonOf(d, side)
	userToe := d.In.PinionToeRadius
	if side.Label == "Driving" {
		userToe = d.In.DrivingToeRadius
	}
	if userToe == 0 && d.In.ToeExtension == 0 {
		near(side.Label+" defaulted toe corner sits at A's station",
			hex.Inner.Z, d.PitchConeR*math.Cos(side.Gamma))
	}
	// The Toe Radius Ceiling is the OUTER toe corner's radius at Toe
	// Extension 0, which is M / O's own radius there.
	if d.In.ToeExtension == 0 {
		near(side.Label+" toe radius ceiling is the outer toe corner",
			side.ToeRadiusCeiling, hex.Toe.Rho)
	}
	// X is the point on the root element at this gear's Toe Radius, so the Toe
	// Limit is what is left of Apex->Ded once that much is walked back.
	near(side.Label+" toe limit",
		apexToDed-side.ToeLimit, side.ToeRadius/math.Sin(bvRootConeAngle(d)))
	if side.ToeRadius <= 0 {
		t.Errorf("%s Toe Radius resolved to %.6f; it must be strictly positive so N never sits on the axis",
			side.Label, side.ToeRadius)
	}

	// The virtual tooth number comes from the back-cone radius and is
	// independent of Tooth Spacing.
	near(side.Label+" virtual tooth number", side.VirtualTeeth,
		math.Floor(2*bvVirtualPitchRadius(side)/d.In.Module))

	// A bore of 0 is auto, which is a quarter of that gear's pitch diameter;
	// unchecking Enable Bore ignores both per-gear inputs.
	if d.In.BoreEnable {
		want := side.PitchDia / 4
		if user := map[string]float64{"Pinion": d.In.PinionBore, "Driving": d.In.DrivingBore}[side.Label]; user > 0 {
			want = user
		}
		near(side.Label+" bore diameter", side.Bore, want)
	} else if side.Bore != 0 {
		t.Errorf("%s bore resolved to %.6f with Enable Bore unchecked", side.Label, side.Bore)
	}
}

// assertHeelChain reads the drawn figure back and holds the base-height bounds
// to it. Everything here is measured off solved geometry, never off the seed
// coordinates the points were created at ([PB-SOLVED-GEOMETRY]).
func assertHeelChain(t testing.TB, s *sketch.Sketch, d bvDesign, side bvSide,
	apex2, dedC, h *sketch.Point, pitchLine *sketch.Line) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve the heel chain: %v", err)
	}
	near := func(label string, got, want float64) {
		t.Helper()
		if math.Abs(got-want) > 1e-6*math.Max(1, math.Abs(want)) {
			t.Errorf("%s: got %.9f, want %.9f", label, got, want)
		}
	}

	// The pitch line's length is driven, so reading R off it checks the closed
	// form that seeds it rather than restating it.
	near("driven |Apex->Apex2|", pitchLine.Length(), d.PitchConeR)
	near("Apex2 stands at the pitch radius", apex2.Y(), side.PitchDia/2)
	// The cone angle carries no dimension in this figure either: the drop's
	// right angle and its pitch-radius length are what place Apex 2, so the
	// angle the pitch line comes out at is a reading, not a setting.
	near("driven pitch cone angle",
		math.Atan2(apex2.Y()-0, apex2.X()-0), side.Gamma)

	// The base height is the offset from Apex 2's plane, not from the dedendum
	// point, and this is the reading that says so.
	near("base height measured from Apex 2's plane", h.X()-apex2.X(), side.BaseHeight)
	near("heel radius falls at cos(gamma) per unit of base height",
		h.Y(), apex2.Y()-side.BaseHeight/math.Tan(side.Gamma))

	// Where the dedendum line meets the shaft axis is the true crossing: the
	// base height at which the heel corner would reach the axis of revolution.
	dx, dy := dedC.X()-apex2.X(), dedC.Y()-apex2.Y()
	if math.Abs(dy) < 1e-12 {
		t.Fatal("the dedendum line came out parallel to the shaft axis")
	}
	crossZ := apex2.X() + dx*(-apex2.Y()/dy)
	near("the true crossing", crossZ-apex2.X(), (side.PitchDia/2)*math.Tan(side.Gamma))
	if side.MaxBaseHeight >= crossZ-apex2.X() {
		t.Errorf("%s Maximum Base Height %.6f reaches the measured crossing %.6f",
			side.Label, side.MaxBaseHeight, crossZ-apex2.X())
	}

	// H lands beyond C along the dedendum line, which is what the Minimum Base
	// Height is for: below it the heel edge runs back inward instead of out.
	if h.X() <= dedC.X() {
		t.Errorf("%s heel corner H at z=%.6f did not clear the dedendum corner C at z=%.6f",
			side.Label, h.X(), dedC.X())
	}
	near("the margin the Minimum Base Height leaves at its own floor",
		side.MinBaseHeight-bvDedendum(d.In.Module)*math.Sin(side.Gamma),
		0.05*bvDedendum(d.In.Module)*math.Sin(side.Gamma))
}
