// Sketch steps. Each function is one Fusion sketch, rebuilt in the sketch
// engine with the constraint scheme the spec prescribes, and gated by proofkit
// on the engine's own verdict: DOF 0, no conflict, no redundancy, valid
// profiles, well conditioned, and no discrete ambiguity.
//
// Three substitutions run through every sketch here, each because the engine
// judges the constraint system alone while Fusion also gets to keep the pose it
// seeded:
//
//  1. The sketch's +X axis is drawn as a short fixed construction line, and
//     every place the spec says addHorizontal on a spoke is pinned instead by a
//     signed zero angle to that line. addHorizontal — in Fusion and in the
//     engine alike — says only "parallel to X", so it leaves the mirrored
//     answer standing: a valley at -Rv satisfies it as happily as one at +Rv.
//     Fusion picks the intended one from the seed; the gate refuses to. The
//     substituted angle carries the direction the seed carried, and costs the
//     proof nothing except that it cannot observe Fusion's seed sensitivity.
//  2. A distance dimension is a magnitude in Fusion, whose side is captured
//     from the seeded geometry ([PB-DIM-VALUE-SEMANTICS]). The eccentric offset
//     is therefore NewDistance(origin, centre, E) — the magnitude — with the
//     side supplied by the same signed angle, so disc 1's -E is the angle at
//     180 degrees and never a negative dimension value.
//  3. Sketch text is not modelled. Fusion's along-path labels add their own
//     letter profiles, which is why every extrude step selects its profile by
//     identity rather than by index, and they hold DOF, which is why a labelled
//     sketch cannot be gated on isFullyConstrained ([PB-TEXT-HOLDS-DOF]). The
//     engine has no text, so the proof shows the geometry fully constrained and
//     says here what the labels would add.
package cycloidal_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
)

// ---- case tables ------------------------------------------------------

// baseCase is the dialog's own defaults, in display units.
func baseCase(overrides map[string]float64) map[string]float64 {
	p := map[string]float64{
		pPinCount:              16,
		pPinCircleDiameter:     90,
		pPinDiameter:           0,
		pEccentricity:          1.5,
		pDiskClearance:         0.3,
		pDiscThickness:         8,
		pDiscGap:               0.5,
		pCenterBearingDiameter: 30,
		pInputShaftDiameter:    8,
		pBearingClearance:      0.2,
		pOutputPinCircleDiam:   50,
		pOutputPinCount:        6,
		pOutputPinDiameter:     0,
		pWall:                  3,
		pBaseThickness:         5,
		pOutputPlateThickness:  5,
		pChamferSize:           0.5,
		pDiscCount:             1,
		pDiscIndex:             0,
	}
	for k, v := range overrides {
		p[k] = v
	}
	return p
}

// discSketchCases covers every branch the per-disc sketches take, from both
// directions the spec offers: the disc index (and with it the signed
// eccentricity and the 180-degree clocking), the disc count, the auto and
// override resolutions of Pin Diameter and Output Pin Diameter, the bore and
// no-bore cam, the two ends of the eccentricity range the undercut guard
// allows, and the two ends of the pin-count range the spec states.
var discSketchCases = []proofkit.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "disc2of2", Params: baseCase(map[string]float64{pDiscCount: 2, pDiscIndex: 1})},
	{Name: "disc1of2", Params: baseCase(map[string]float64{pDiscCount: 2, pDiscIndex: 0})},
	{Name: "eccentricityNearUndercutLimit", Params: baseCase(map[string]float64{pEccentricity: 2.45})},
	{Name: "eccentricitySmall", Params: baseCase(map[string]float64{pEccentricity: 0.5})},
	{Name: "pinDiameterOverride", Params: baseCase(map[string]float64{pPinDiameter: 9})},
	{Name: "outputPinDiameterOverride", Params: baseCase(map[string]float64{pOutputPinDiameter: 8})},
	{Name: "noInputBore", Params: baseCase(map[string]float64{pInputShaftDiameter: 0})},
	{Name: "minimumCounts", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
	{Name: "manyPins", Params: baseCase(map[string]float64{
		pPinCount: 30, pOutputPinCount: 8, pEccentricity: 1.0,
	})},
	{Name: "twoDiscsEvenCountsDisc2", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pDiscIndex: 1,
		pOutputPinCircleDiam: 42, pCenterBearingDiameter: 18,
	})},
	{Name: "smallDrive", Params: baseCase(map[string]float64{
		pPinCircleDiameter: 40, pOutputPinCircleDiam: 22, pCenterBearingDiameter: 12,
		pInputShaftDiameter: 4, pEccentricity: 0.8, pOutputPinCount: 4, pPinCount: 10,
	})},
}

// casingSketchCases sweeps the pin count, since the casing section's angular
// width is one pin pitch and its tiling is what the ends at +/-pi/N buy, and
// the eccentricity, since the contour's depth is set by the swept envelope.
var casingSketchCases = []proofkit.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "minimumPinCount", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
	{Name: "manyPins", Params: baseCase(map[string]float64{
		pPinCount: 30, pOutputPinCount: 8, pEccentricity: 1.0,
	})},
	{Name: "eccentricityNearUndercutLimit", Params: baseCase(map[string]float64{pEccentricity: 2.45})},
	{Name: "twoDiscStack", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18,
	})},
}

// outputSketchCases covers the output member: the pin's auto and override
// resolution, the pin count at both ends, and the two-disc stack whose stack
// top the plate plane is offset from.
var outputSketchCases = []proofkit.Case{
	{Name: "defaults", Params: baseCase(nil)},
	{Name: "outputPinDiameterOverride", Params: baseCase(map[string]float64{pOutputPinDiameter: 8})},
	{Name: "minimumOutputPinCount", Params: baseCase(map[string]float64{
		pPinCount: 4, pOutputPinCount: 3, pOutputPinCircleDiam: 34,
		pCenterBearingDiameter: 14, pInputShaftDiameter: 5,
	})},
	{Name: "manyOutputPins", Params: baseCase(map[string]float64{
		pPinCount: 30, pOutputPinCount: 8, pEccentricity: 1.0,
	})},
	{Name: "twoDiscStack", Params: baseCase(map[string]float64{
		pPinCount: 6, pOutputPinCount: 4, pDiscCount: 2, pOutputPinCircleDiam: 42,
		pCenterBearingDiameter: 18,
	})},
}

// ---- shared sketch frame ----------------------------------------------

// frame is the anchored local frame every sketch in this gear starts from.
type frame struct {
	origin *sketch.Point // the sketch's own local origin, on the drive axis O
	axis   *sketch.Line  // the sketch's +X direction, fixed, construction
	centre *sketch.Point // the disc centre Od_d, nil in a sketch built on O only
	ecc    *sketch.Line  // the O -> Od construction line, nil likewise
}

// anchoredFrame builds the anchor chain: a projected Anchor, a fresh local
// origin coincident to it, and the fixed +X reference the signed angles use.
func anchoredFrame(t testing.TB, s *sketch.Sketch, d dims) frame {
	anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
	anchor.SetName("projected Anchor")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin O")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	axisLen := math.Max(10, d.R)
	tip := s.CreatePoint(axisLen, 0)
	tip.SetName("+X reference")
	s.Fix(tip)
	axis := s.CreateLine(origin, tip)
	axis.SetName("+X axis")
	axis.SetConstruction(true)
	return frame{origin: origin, axis: axis}
}

// eccentricFrame adds the eccentric disc centre Od_d = O + s_d*E*Xhat: a point
// on a construction line from O, with a driving distance dimension of E and the
// side carried by the signed angle to +X.
func eccentricFrame(t testing.TB, s *sketch.Sketch, d dims) frame {
	f := anchoredFrame(t, s, d)
	c := d.centre()
	f.centre = s.CreatePoint(c.X, c.Y)
	f.centre.SetName("disc centre Od")
	f.ecc = s.CreateLine(f.origin, f.centre)
	f.ecc.SetName("eccentric offset")
	f.ecc.SetConstruction(true)
	s.AddConstraint(
		sketch.NewAngle(f.axis, f.ecc, signAngle(d.S)),
		sketch.NewDistance(f.origin, f.centre, d.E),
	)
	return f
}

// signAngle is the signed angle, in degrees, that carries an eccentric sign:
// disc 0 offsets along +X, disc 1 along -X.
func signAngle(s float64) float64 {
	if s < 0 {
		return 180
	}
	return 0
}

// circleOn draws a circle whose centre is a fresh point made coincident to an
// existing one — Fusion's addByCenterRadius(Point3D, r) plus addCoincident,
// which is share xor coincident done the coincident way ([PB-SHARE-XOR-COINCIDENT]).
func circleOn(s *sketch.Sketch, at *sketch.Point, seed pt, radius float64, name string,
	construction bool) *sketch.Circle {
	c := s.CreatePoint(seed.X, seed.Y)
	circle := s.CreateCircle(c, radius)
	circle.SetName(name)
	circle.SetConstruction(construction)
	s.AddConstraint(
		sketch.NewCoincident(c, at),
		sketch.NewDiameter(circle, 2*radius),
	)
	return circle
}

// ---- S07: the rotor lobe sketch ---------------------------------------

// stepRotorLobeSketch builds Rotor Lobe {d+1}: the reference circles, the open
// adaptively sampled lobe spline, and the two spokes that close the pie sector.
func stepRotorLobeSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Rotor Lobe %d: N=%d L=%d E=%.3f Rr=%.3f Rv=%.3f",
		d.D0+1, d.N, d.L, d.E, d.Rr, d.Rv)

	f := eccentricFrame(t, s, d)
	centre := d.centre()

	// 1. Pin circle, on O, the fixed ring. Drawn for disc 0 only.
	if d.D0 == 0 {
		circleOn(s, f.origin, pt{}, d.R, "Pin Circle", true)
	}
	// 2. Output-pin circle and 3. root circle, both on the disc centre Od.
	circleOn(s, f.centre, centre, d.Rop, "Output Pin Circle", true)
	root := circleOn(s, f.centre, centre, d.Rv, "Root Circle", true)

	// 4. The lobe: one open fitted spline, adaptively sampled. Never closed,
	// and no closing arc — the disc is closed later by the pattern, not here.
	proofkit.Step(t, "lobe spline: %d adaptive fit points", len(d.lobeSamples()))
	spline := fitSpline(t, s, d.lobeSamples(), "Lobe")

	// 5. Lock the spline: fix every interior fit point, and coincide each end
	// onto the root circle so the valley radii are pinned by Rv rather than by
	// the fixed coordinates. Fixing the whole spline instead would make the
	// lobe-pitch angle dimension redundant.
	fit := spline.Fit
	for i := 1; i < len(fit)-1; i++ {
		s.Fix(fit[i])
	}
	s.AddConstraint(
		sketch.NewPointOnCircle(fit[0], root),
		sketch.NewPointOnCircle(fit[len(fit)-1], root),
	)

	// 6. Spoke 1: disc centre to the lobe's first point, on the +X ray from Od.
	end1 := s.CreatePoint(fit[0].X(), fit[0].Y())
	spoke1 := s.CreateLine(f.centre, end1)
	spoke1.SetName("spoke 1")
	s.AddConstraint(
		sketch.NewCoincident(end1, fit[0]),
		sketch.NewAngle(f.ecc, spoke1, 0),
	)

	// 7. Spoke 2: disc centre to the lobe's last point.
	last := fit[len(fit)-1]
	end2 := s.CreatePoint(last.X(), last.Y())
	spoke2 := s.CreateLine(f.centre, end2)
	spoke2.SetName("spoke 2")
	s.AddConstraint(sketch.NewCoincident(end2, last))

	// 8. The lobe-pitch angle, one turn of the L-fold symmetry. Fusion's
	// angular dimension is the positive magnitude 360 deg / Lobes, with the
	// minor wedge chosen by where the text point sits; the engine's angle is
	// signed counter-clockwise from spoke 1 to spoke 2, and the lobe runs
	// clockwise, so the same wedge is the negative value here.
	s.AddConstraint(sketch.NewAngle(spoke1, spoke2, -360/float64(d.L)))

	solveHere(t, s)

	// What the spec pins about this sketch, measured on what was built.
	if got := radiusOf(pointAt(fit[0]), centre); !nearly(got, d.Rv, 1e-6) {
		t.Errorf("start valley radius %.6f, want Rv %.6f", got, d.Rv)
	}
	if got := radiusOf(pointAt(last), centre); !nearly(got, d.Rv, 1e-6) {
		t.Errorf("end valley radius %.6f, want Rv %.6f", got, d.Rv)
	}
	wantEnd := pt{
		X: centre.X + d.Rv*math.Cos(d.Phi-2*math.Pi/float64(d.L)),
		Y: centre.Y + d.Rv*math.Sin(d.Phi-2*math.Pi/float64(d.L)),
	}
	if got := pointAt(last); !nearly(got.X, wantEnd.X, 1e-6) || !nearly(got.Y, wantEnd.Y, 1e-6) {
		t.Errorf("end valley at (%.6f, %.6f), want (%.6f, %.6f)", got.X, got.Y, wantEnd.X, wantEnd.Y)
	}
	// The lobe tip reaches R - Rr_eff + E from the disc centre. This is the
	// number the casing's outer wall is sized from, one E further out.
	if got := d.tracedTipRadius(); !nearly(got, d.tipRadius(), 1e-9) {
		t.Errorf("traced lobe tip radius %.9f, want R - Rr_eff + E = %.9f", got, d.tipRadius())
	}
	// The kept fit points fall short of that tip by the sampler's chordal
	// shortfall, which the 5-degree turn threshold is what bounds.
	sampled := 0.0
	for _, q := range d.lobeSamples() {
		sampled = math.Max(sampled, radiusOf(q, centre))
	}
	if short := d.tipRadius() - sampled; short < 0 || short > 0.05 {
		t.Errorf("sampled lobe tip falls %.6f mm short of the traced tip, want within 0.05 mm", short)
	}
	// The sector is one closed region: two lines and one spline.
	requireProfileCount(t, s, 1)

	// The undercut guard is the binding eccentricity limit, tighter than the
	// base-cycloid cusp limit R/N the spec calls loose, and this case sits
	// under it — above it the equidistant self-intersects and Fusion fails on
	// the spline rather than on a dimension.
	ceiling := undercutCeiling(p)
	if ceiling <= 0 || ceiling >= d.R/float64(d.N) {
		t.Errorf("undercut ceiling %.4f mm does not bind below the cusp limit R/N = %.4f mm",
			ceiling, d.R/float64(d.N))
	}
	if d.E > ceiling {
		t.Errorf("case eccentricity %.4f mm exceeds the undercut ceiling %.4f mm", d.E, ceiling)
	}
	// The spec states the number for the dialog defaults: about 2.50 mm, well
	// under the 2.81 mm the cusp limit would allow.
	if d.N == 16 && d.R == 45 && d.C == 0.3 && p[pPinDiameter] == 0 && !nearly(ceiling, 2.4978, 0.005) {
		t.Errorf("undercut ceiling for the default pin geometry is %.4f mm, want about 2.50 mm", ceiling)
	}
}

// ---- S12: the output hole sketch --------------------------------------

// stepOutputHoleSketch builds Output Hole {d+1}: the output-pin circle about
// the disc centre and one solid hole seated on it.
func stepOutputHoleSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Output Hole %d: Rop=%.3f D_hole=%.3f (= D_pin %.3f + 2E)",
		d.D0+1, d.Rop, d.DHole, d.DPin)

	f := eccentricFrame(t, s, d)
	centre := d.centre()
	holeCircle := circleOn(s, f.centre, centre, d.Rop, "Output Hole Circle", true)

	// The hole spoke is horizontal in the sketch's own +X sense and is not
	// rotated with the disc's clocking: the spec substitutes only the signed E
	// for a second disc, and the M-fold pattern maps the half-turned hole set
	// onto itself whenever M is even, which two discs require anyway.
	seat := pt{X: centre.X + d.Rop, Y: 0}
	hole := s.CreateCircle(s.CreatePoint(seat.X, seat.Y), d.DHole/2)
	hole.SetName("output hole")
	spoke := s.CreateLine(f.centre, hole.Center)
	spoke.SetName("hole spoke")
	spoke.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(hole, d.DHole),
		sketch.NewPointOnCircle(hole.Center, holeCircle),
		sketch.NewAngle(f.axis, spoke, 0),
	)

	solveHere(t, s)

	if got := radiusOf(pointAt(hole.Center), centre); !nearly(got, d.Rop, 1e-6) {
		t.Errorf("hole centre at radius %.6f from Od, want Rop %.6f", got, d.Rop)
	}
	if got := pointAt(hole.Center); !nearly(got.X, centre.X+d.Rop, 1e-6) || !nearly(got.Y, 0, 1e-6) {
		t.Errorf("hole centre at (%.6f, %.6f), want the +X ray from Od at (%.6f, 0)",
			got.X, got.Y, centre.X+d.Rop)
	}
	// The hole is the pin plus the orbit clearance 2E, on every branch of the
	// pin-size resolution.
	if !nearly(d.DHole-d.DPin, 2*d.E, 1e-9) {
		t.Errorf("hole oversize %.6f, want 2E = %.6f", d.DHole-d.DPin, 2*d.E)
	}
	requireProfileCount(t, s, 1)
}

// ---- S15: the disc bore sketch ----------------------------------------

// stepDiscBoreSketch builds Disc Bore {d+1}: one solid circle on the disc
// centre, the cam diameter widened by the running clearance.
func stepDiscBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Disc Bore %d: bore diameter %.3f = CBD %.3f + clearance %.3f",
		d.D0+1, d.CBD+d.Clr, d.CBD, d.Clr)

	f := eccentricFrame(t, s, d)
	bore := circleOn(s, f.centre, d.centre(), (d.CBD+d.Clr)/2, "disc centre bore", false)

	solveHere(t, s)

	if got := 2 * bore.R(); !nearly(got, d.CBD+d.Clr, 1e-6) {
		t.Errorf("bore diameter %.6f, want CenterBearingDiameter + BearingClearance %.6f",
			got, d.CBD+d.Clr)
	}
	// The bore is concentric with the cam and larger by the clearance, so the
	// gap is the same all the way round rather than an eccentric one.
	if got := radiusOf(pointAt(bore.Center), d.centre()); !nearly(got, 0, 1e-6) {
		t.Errorf("bore centre %.6f from Od, want concentric", got)
	}
	requireProfileCount(t, s, 1)
}

// ---- S17: the eccentric cam section sketch ----------------------------

// stepEccentricCamSketch builds Eccentric Cam {d+1}: the cam outer on the disc
// centre and, when the input shaft has a diameter, the input bore on the drive
// axis. The E offset between the two is the eccentricity.
func stepEccentricCamSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Eccentric Cam %d: outer %.3f on Od, bore %.3f on O", d.D0+1, d.CBD, d.ISD)

	f := eccentricFrame(t, s, d)
	outer := circleOn(s, f.centre, d.centre(), d.CBD/2, "cam outer", false)

	wantProfiles := 1
	if d.ISD > 0 {
		circleOn(s, f.origin, pt{}, d.ISD/2, "input bore", false)
		// The bore lies wholly inside the cam outer but off its centre, so the
		// arrangement splits the disc into a bore disc and a cam annulus.
		wantProfiles = 2
	}

	solveHere(t, s)

	if got := 2 * outer.R(); !nearly(got, d.CBD, 1e-6) {
		t.Errorf("cam outer diameter %.6f, want CenterBearingDiameter %.6f", got, d.CBD)
	}
	if got := radiusOf(pointAt(outer.Center), pt{}); !nearly(got, d.E, 1e-6) {
		t.Errorf("cam outer centre %.6f from O, want the eccentricity %.6f", got, d.E)
	}
	requireProfileCount(t, s, wantProfiles)
	if d.ISD > 0 {
		// The cam cross-section is the two-loop annulus: an outer loop and one
		// hole. That count, not a curve-type count, is how the extrude finds it
		// — a full circle is a Circle3DCurveType, not an arc, and the annulus's
		// two circles sit in separate loops.
		if got := annulusProfiles(s); got != 1 {
			t.Errorf("%d two-loop profiles in the cam sketch, want exactly 1", got)
		}
	}
}

// ---- S21: the housing ring sketch -------------------------------------

// stepHousingRingSketch builds Housing Ring: the base annulus on the drive
// axis, one millimetre below the disc.
func stepHousingRingSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Housing Ring: outer %.3f inner %.3f (wall %.3f)",
		2*d.outerWall(), 2*d.innerFloor(), d.Wall)

	f := anchoredFrame(t, s, d)
	outer := circleOn(s, f.origin, pt{}, d.outerWall(), "housing outer", false)
	inner := circleOn(s, f.origin, pt{}, d.innerFloor(), "housing inner", false)

	solveHere(t, s)

	// The outer wall clears the contour peak at R - PinRadius + 2E by exactly
	// Wall, which is what makes Wall the minimum wall thickness; the inner floor
	// lip sits Wall inside the contour valley at R - PinRadius.
	if got := outer.R() - (d.R - d.Rr + 2*d.E); !nearly(got, d.Wall, 1e-9) {
		t.Errorf("outer wall clears the contour peak by %.6f, want Wall %.6f", got, d.Wall)
	}
	if got := (d.R - d.Rr) - inner.R(); !nearly(got, d.Wall, 1e-9) {
		t.Errorf("inner floor lip is %.6f inside the contour valley, want Wall %.6f", got, d.Wall)
	}
	requireProfileCount(t, s, 2)
	if got := annulusProfiles(s); got != 1 {
		t.Errorf("%d two-loop profiles in the housing sketch, want exactly 1", got)
	}
}

// ---- S24: the ring casing section sketch ------------------------------

// stepRingCasingSketch builds Ring Casing: one pin-pitch of the swept-envelope
// contour, a solid outer circle, and the two radial spokes that close the
// wedge.
//
// Substituted: the spec leaves this sketch deliberately under-constrained — the
// contour's fit points are numeric snapshots that are not fixed, and the spokes'
// outer ends are only seeded on the outer circle — because the sketch is
// consumed immediately by the sector extrude and never re-solved. The engine's
// gate has no way to accept free geometry, so the proof pins what Fusion leaves
// loose: every contour point is fixed, and each spoke's outer end is put on the
// circle at its own radial angle. The cost is that this proof says nothing about
// the exempted sketch's free degrees of freedom; what it still proves is the
// thing the exemption does not touch — that the contour ends land exactly on
// +/-pi/N and that the wedge is a real closed profile, distinct from the
// complement the same spline also bounds.
func stepRingCasingSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	half := math.Pi / float64(d.N)
	points := d.contour()
	proofkit.Step(t, "Ring Casing: %d contour points over one pin pitch of %.4f rad",
		len(points), 2*half)

	f := anchoredFrame(t, s, d)
	outer := circleOn(s, f.origin, pt{}, d.outerWall(), "casing outer", false)

	contour := fitSpline(t, s, points, "contour")
	for _, q := range contour.Fit {
		s.Fix(q)
	}
	ends := []*sketch.Point{contour.Fit[0], contour.Fit[len(contour.Fit)-1]}
	for i, end := range ends {
		angle := -half
		if i == 1 {
			angle = half
		}
		tip := s.CreatePoint(d.outerWall()*math.Cos(angle), d.outerWall()*math.Sin(angle))
		spoke := s.CreateLine(end, tip)
		spoke.SetName("casing spoke")
		s.AddConstraint(
			sketch.NewPointOnCircle(tip, outer),
			sketch.NewAngle(f.axis, spoke, angle*180/math.Pi),
		)
	}

	solveHere(t, s)

	// The load-bearing fact: the first and last contour points sit exactly on
	// the pin-pitch boundaries. Bin centres would inset both by half a bin, and
	// the N patterned sectors would then never touch.
	for i, want := range []float64{-half, half} {
		q := points[i*(len(points)-1)]
		if got := math.Atan2(q.Y, q.X); !nearly(got, want, 1e-12) {
			t.Errorf("contour end %d at angle %.15f rad, want %.15f", i, got, want)
		}
	}
	// The contour is the swept envelope plus the clearance: its deepest point
	// is the pin seat at R - Rr and its ends are the mid-gap peaks at
	// R - Rr + 2E, which is what the outer wall is sized from.
	lo, hi := math.Inf(1), 0.0
	for _, q := range points {
		r := math.Hypot(q.X, q.Y)
		lo, hi = math.Min(lo, r), math.Max(hi, r)
	}
	// The sweep is a fixed 240 x 240 sample binned 80 ways, so these radii carry
	// that resolution's error. The spec accepts it by design: the disc clears
	// the contour by the clearance c, which absorbs a sampling error far larger
	// than this one.
	const sweepTol = 0.05
	if !nearly(lo, d.R-d.Rr, sweepTol) {
		t.Errorf("contour valley radius %.6f, want R - Rr = %.6f", lo, d.R-d.Rr)
	}
	if !nearly(hi, d.R-d.Rr+2*d.E, sweepTol) {
		t.Errorf("contour peak radius %.6f, want R - Rr + 2E = %.6f", hi, d.R-d.Rr+2*d.E)
	}
	if got := math.Hypot(points[0].X, points[0].Y); !nearly(got, hi, sweepTol) {
		t.Errorf("contour end radius %.6f is not the peak %.6f, so the ends are not the mid-gap "+
			"peaks the tiling joins on", got, hi)
	}

	// The solid outer circle makes the open contour a shared edge of two closed
	// regions: the thin wedge, and the whole complement inside the circle. Both
	// contain the contour, so a first-match search can take the complement and
	// extrude a near-full disc. The wedge is the smaller by area, by a wide
	// margin, which is what the extrude step selects on.
	holding := profilesContaining(s, contour)
	if len(holding) != 2 {
		t.Fatalf("%d profiles contain the contour spline, want 2 (the wedge and its complement)",
			len(holding))
	}
	small, large := holding[0].Area, holding[1].Area
	if small > large {
		small, large = large, small
	}
	if small >= large {
		t.Fatalf("the two profiles containing the contour have areas %.4f and %.4f", small, large)
	}
	wedge := (math.Pi*d.outerWall()*d.outerWall() - math.Abs(polygonArea(d.contourRing()))) /
		float64(d.N)
	if !nearly(small, wedge, 0.05*wedge) {
		t.Errorf("smallest containing profile has area %.4f mm^2, want the one-pitch wedge %.4f",
			small, wedge)
	}
}

// ---- S30: the output plate sketch -------------------------------------

// stepOutputPlateSketch builds Output Plate: the plate outer, the output-pin
// circle, and one solid pin seated on it. All three are on the drive axis O,
// not the disc centre — the pins are the fixed output member, the holes they
// pass through orbit with the disc.
func stepOutputPlateSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)
	requireInRegime(t, d)
	proofkit.Step(t, "Output Plate: plate diameter %.3f, pin diameter %.3f on Rop %.3f",
		2*d.plateRadius(), d.DPin, d.Rop)

	f := anchoredFrame(t, s, d)
	plate := circleOn(s, f.origin, pt{}, d.plateRadius(), "plate outer", false)
	pinCircle := circleOn(s, f.origin, pt{}, d.Rop, "Output Pin Circle", true)

	pin := s.CreateCircle(s.CreatePoint(d.Rop, 0), d.DPin/2)
	pin.SetName("output pin")
	spoke := s.CreateLine(f.origin, pin.Center)
	spoke.SetName("pin spoke")
	spoke.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(pin, d.DPin),
		sketch.NewPointOnCircle(pin.Center, pinCircle),
		sketch.NewAngle(f.axis, spoke, 0),
	)

	solveHere(t, s)

	// The plate covers the outermost pin by Wall.
	if got := plate.R() - (d.Rop + d.DPin/2); !nearly(got, d.Wall, 1e-9) {
		t.Errorf("plate overhangs the pin by %.6f, want Wall %.6f", got, d.Wall)
	}
	if got := radiusOf(pointAt(pin.Center), pt{}); !nearly(got, d.Rop, 1e-6) {
		t.Errorf("pin centre at radius %.6f from O, want Rop %.6f", got, d.Rop)
	}
	// The pin sits wholly inside the plate and splits it: the plate-with-bite
	// and the pin disc are two regions, and the extrude takes both so the pin
	// footprint is solid plate.
	requireProfileCount(t, s, 2)
	if got := annulusProfiles(s); got != 1 {
		t.Errorf("%d two-loop profiles in the plate sketch, want exactly 1 (the plate with its bite)", got)
	}
}

// ---- small helpers ----------------------------------------------------

// fitSpline adds an open fitted spline through the given points. It is never
// closed and never given a closing arc.
func fitSpline(t testing.TB, s *sketch.Sketch, points []pt, name string) *sketch.FitSpline {
	t.Helper()
	handles := make([]*sketch.Point, len(points))
	for i, q := range points {
		handles[i] = s.CreatePoint(q.X, q.Y)
	}
	spline, err := s.CreateFitSpline(handles...)
	if err != nil {
		t.Fatalf("create %s spline through %d points: %v", name, len(points), err)
	}
	spline.SetName(name)
	return spline
}

// solveHere solves so the assertions below it read solved positions rather than
// the seeds they were drawn from ([PB-SOLVED-GEOMETRY]). proofkit solves and
// verifies again afterwards; this one is only so the step can measure.
func solveHere(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	res, err := s.Solve(t.Context())
	if err != nil {
		t.Fatalf("solve: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e, DOF %d", res.Residual, res.DOF)
	}
}

func pointAt(p *sketch.Point) pt { return pt{X: p.X(), Y: p.Y()} }

// requireProfileCount fails unless the sketch closes exactly the regions the
// step expects. Fusion's along-path text labels add letter profiles on top of
// these, which is why every extrude selects by identity, never by index.
func requireProfileCount(t testing.TB, s *sketch.Sketch, want int) {
	t.Helper()
	if got := len(s.Profiles()); got != want {
		t.Errorf("sketch closes %d profile(s), want %d", got, want)
	}
}

// annulusProfiles counts the regions with exactly one hole loop — the count the
// annulus extrudes select on.
func annulusProfiles(s *sketch.Sketch) int {
	n := 0
	for _, prof := range s.Profiles() {
		if len(prof.Holes) == 1 {
			n++
		}
	}
	return n
}

// profilesContaining returns the profiles whose boundary uses the given entity.
func profilesContaining(s *sketch.Sketch, want sketch.Entity) []*sketch.Profile {
	var out []*sketch.Profile
	for _, prof := range s.Profiles() {
		for _, e := range prof.Entities {
			if e == want {
				out = append(out, prof)
				break
			}
		}
	}
	return out
}
