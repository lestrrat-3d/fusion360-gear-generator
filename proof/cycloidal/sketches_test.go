// This file holds the cycloidal drive's sketch steps, one function per Fusion
// timeline sketch: the rotor lobe, the output hole, the disc bore, the
// eccentric cam section, the housing base annulus, the ring-casing section and
// the output plate.
//
// Two substitutions run through all of them, and both are recorded at the
// place they are made rather than only here.
//
// The first is the signed dimension. Fusion's addDistanceDimension carries a
// magnitude and takes its side from the seeded geometry, while the engine's
// horizontal and vertical distances carry a signed target
// ([PB-DIM-VALUE-SEMANTICS]). Where the spec pins a point on a ray from a
// centre — the eccentric offset Od, the output hole's spoke, the output pin's
// spoke — the unsigned pair (a point-on-circle and a horizontal) reaches DOF 0
// and still admits the mirrored answer, which proofkit's gate refuses by
// design. The proof states the signed distance and then measures the
// coincidence the spec's constraint would have asserted.
//
// The second is the lobe spoke. Fusion pins spoke 1 with addHorizontal, which
// leaves the same mirrored pair; the proof states a zero angle to the
// eccentric construction line instead, which is the same direction with a
// sense attached, and keeps the spec's own lobe-pitch angular dimension
// between the two spokes unchanged.
package cycloidal_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// residualSlack is the absolute slack a committed constraint's residual is
// checked to. The solver converges to a residual near 1e-11 on these sketches,
// so this leaves four orders of magnitude and still refuses a constraint that
// did not take.
const residualSlack = 1e-7

// coordSlack is the absolute millimetre slack for a solved coordinate compared
// against the closed-form position the spec states for it. The formulas are
// exact, so the only error is the solver's own convergence.
const coordSlack = 1e-9

// stepRotorLobeSketch draws `Rotor Lobe {d+1}`: the anchor chain, the three
// reference circles, one open adaptively-sampled lobe spline about the disc
// centre, and the two spokes that pin its ends.
//
// The scheme is what the step exists to prove. The spline's shape comes from
// fixing every interior fit point, and its two ends are pinned by radius (each
// coincident on the root circle) and by angle (spoke 1's direction and the
// lobe-pitch dimension between the spokes). Nothing else is free.
func stepRotorLobeSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	// The no-undercut guard is the binding eccentricity limit, and it decides
	// whether the curve below is drawable at all, so it is checked before the
	// curve is drawn rather than inferred from the solve afterwards.
	proofkit.Step(t, "check the no-undercut guard Rr_eff < rho_min^O")
	rho := rhoMinTowardO(d)
	if d.RrEff >= rho {
		t.Fatalf("Rr_eff %.6f mm has reached the base trochoid's smallest inward radius of "+
			"curvature %.6f mm, so the drawn profile undercuts itself", d.RrEff, rho)
	}
	// E* is the number the rejection message carries, so the bisection that
	// finds it is proven here too, against a bracket wide enough to hold it.
	// The bracket is the loose base-cycloid cusp limit R/N, which the guard has
	// always already failed by: that the undercut bound is the tighter of the
	// two is the whole reason it exists.
	cusp := d.R / float64(d.N)
	if outer := atEccentricity(p, cusp); outer.RrEff < rhoMinTowardO(outer) {
		t.Fatalf("the undercut guard still holds at the cusp limit R/N = %.6f mm, so it is not "+
			"the binding bound the spec says it is", cusp)
	}
	limit := undercutLimit(p, cusp)
	if limit <= d.E {
		t.Fatalf("the bisected undercut bound E* is %.6f mm, at or below this case's "+
			"Eccentricity %.6f mm, so the bound and the guard disagree", limit, d.E)
	}
	if d.N == 16 && d.R == 45 && d.C == 0.3 && p[keyPinDiameter] == 0 {
		// epitrochoid-trace.md states the worked answer for the dialog's own
		// defaults: max safe E is about 2.50 mm, against the loose R/N of 2.81.
		sketchtest.Measures(t, "undercut bound E* at the dialog defaults", limit, 2.50,
			sketchtest.Within(0.01))
		sketchtest.Measures(t, "the loose base-cycloid cusp limit R/N", cusp, 2.8125,
			sketchtest.Within(1e-9))
	}

	proofkit.Step(t, "anchor a local origin and build the eccentric disc centre Od")
	origin := groundedSketch(t, s)
	centre, ecc := eccentricCentre(t, s, origin, d)

	proofkit.Step(t, "draw the pin, output-pin and root reference circles")
	pin := circleOn(t, s, origin, d.R, "Pin Circle", true)
	outputPins := circleOn(t, s, centre, d.Rop, "Output Pin Circle", true)
	root := circleOn(t, s, centre, d.Rv, "Root Circle", true)

	c := d.centre()
	samples := lobeSamples(d, c.X, c.Y, d.Phi)
	proofkit.Step(t, "fit the open lobe spline through %d adaptively sampled points", len(samples))
	fit := make([]*sketch.Point, len(samples))
	for i, q := range samples {
		fit[i] = s.CreatePoint(q.X, q.Y)
	}
	spline, err := s.CreateFitSpline(fit...)
	if err != nil {
		t.Fatalf("fit the lobe spline through %d points: %v", len(fit), err)
	}
	spline.SetName("lobe")
	start, end := fit[0], fit[len(fit)-1]
	start.SetName("start valley")
	end.SetName("end valley")

	proofkit.Step(t, "lock the spline: fix the interior fit points, put both ends on the root circle")
	for i := 1; i < len(fit)-1; i++ {
		s.Fix(fit[i])
	}
	startOnRoot := sketch.NewPointOnCircle(start, root)
	endOnRoot := sketch.NewPointOnCircle(end, root)
	s.AddConstraint(startOnRoot, endOnRoot)
	s.SetConstraintName(startOnRoot, "start valley on the root circle")
	s.SetConstraintName(endOnRoot, "end valley on the root circle")

	proofkit.Step(t, "draw the two spokes and the lobe-pitch angular dimension")
	pitch := 2 * math.Pi / float64(d.L)
	tip1 := s.CreatePoint(c.X+d.Rv*math.Cos(d.Phi), c.Y+d.Rv*math.Sin(d.Phi))
	spoke1 := s.CreateLine(centre, tip1)
	tip2 := s.CreatePoint(c.X+d.Rv*math.Cos(d.Phi-pitch), c.Y+d.Rv*math.Sin(d.Phi-pitch))
	// The engine reads an angle target in the sketch's default angle unit,
	// which is degrees, and the spec's dimension is `360 deg / Lobes`, so the
	// target is written in degrees here too. The lobe runs clockwise from its
	// first valley, so the turn from spoke 1 to spoke 2 is negative.
	spoke2 := s.CreateLine(centre, tip2)
	// Spoke 1's direction stands in for Fusion's addHorizontal on it: a zero
	// angle to the eccentric line is the same direction with a sense, and the
	// sense is what keeps the mirrored configuration out. Disc 1's eccentric
	// line points along -X and so does its first valley, so the target is zero
	// for both discs.
	spoke1Dir := sketch.NewAngle(ecc, spoke1, 0)
	lobePitch := sketch.NewAngle(spoke1, spoke2, -360/float64(d.L))
	s.AddConstraint(
		sketch.NewCoincident(tip1, start),
		sketch.NewCoincident(tip2, end),
		spoke1Dir, lobePitch,
	)
	s.SetConstraintName(spoke1Dir, "spoke 1 along the eccentric line")
	s.SetConstraintName(lobePitch, "lobe pitch 360 deg / Lobes")

	proofkit.Step(t, "read the solved frame back")
	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, centre, d.Sign*d.E, 0, sketchtest.Within(coordSlack))
	sketchtest.Measures(t, "pin circle radius", pin.R(), d.R, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, "output-pin circle radius", outputPins.R(), d.Rop, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, "root circle radius", root.R(), d.Rv, sketchtest.WithinRel(1e-12))
	// Both valleys sit at Rv from Od: that is the fact the root circle exists
	// to pin, and the step that extrudes the sector selects on the closed loop
	// it completes.
	sketchtest.Measures(t, "start valley radius from Od", start.DistanceTo(centre), d.Rv,
		sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "end valley radius from Od", end.DistanceTo(centre), d.Rv,
		sketchtest.WithinRel(1e-9))
	for _, name := range []string{
		"start valley on the root circle", "end valley on the root circle",
		"spoke 1 along the eccentric line", "lobe pitch 360 deg / Lobes",
	} {
		sketchtest.Satisfies(t, s.ConstraintByName(name), sketchtest.Within(residualSlack))
	}

	proofkit.Step(t, "check the one closed region the sector extrude selects")
	report := sketchtest.Verify(t, s)
	profile := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, profile)
	sketchtest.IsCurrentProfile(t, profile)
	sketchtest.HasExactCuts(t, profile)
	// The spline's area and its chord polygon's differ only by the sagitta of
	// each span, which the 5-degree turn limit holds under a part in a
	// thousand; the slack states that difference and nothing else.
	sector := append([]point{c}, samples...)
	sketchtest.MeasuresProfileArea(t, profile, polygonArea(sector), sketchtest.WithinRel(2e-3))
}

// stepOutputHoleSketch draws `Output Hole {d+1}`: its own anchor chain and disc
// centre, the construction output-pin circle, and the one solid hole seated on
// it whose profile the cut step selects by identity.
func stepOutputHoleSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch and rebuild Od")
	origin := groundedSketch(t, s)
	centre, _ := eccentricCentre(t, s, origin, d)

	proofkit.Step(t, "draw the construction output-hole circle on Od")
	circle := circleOn(t, s, centre, d.Rop, "Output Hole Circle", true)

	proofkit.Step(t, "draw the one solid hole on the +X ray from Od")
	c := d.centre()
	holeCentre := s.CreatePoint(c.X+d.Rop, c.Y)
	holeCentre.SetName("output hole centre")
	hole := s.CreateCircle(holeCentre, d.DHole/2)
	hole.SetName("output hole")
	spoke := s.CreateLine(centre, holeCentre)
	spoke.SetConstruction(true)
	// The spec pins this centre with a point-on-circle and a horizontal spoke.
	// That pair admits the hole at -Rop as well, so the radius is stated as a
	// signed horizontal distance and the seating on the circle is measured
	// below. Both discs place their first hole on +X: M is even whenever two
	// discs are asked for, so disc 1's half-turn maps its hole set onto itself.
	seatRadius := sketch.NewHorizontalDistance(centre, holeCentre, d.Rop)
	s.AddConstraint(
		sketch.NewHorizontal(spoke),
		seatRadius,
		sketch.NewDiameter(hole, d.DHole),
	)
	s.SetConstraintName(seatRadius, "output hole on the output-pin circle radius")

	proofkit.Step(t, "read the solved hole back")
	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, holeCentre, c.X+d.Rop, c.Y, sketchtest.Within(coordSlack))
	sketchtest.Measures(t, "output hole seating radius", holeCentre.DistanceTo(centre), circle.R(),
		sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "output hole radius", hole.R(), d.DHole/2, sketchtest.WithinRel(1e-12))
	sketchtest.Satisfies(t, s.ConstraintByName("output hole on the output-pin circle radius"),
		sketchtest.Within(residualSlack))

	proofkit.Step(t, "check the one solid region the cut selects")
	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*d.DHole*d.DHole/4, sketchtest.WithinRel(1e-9))
}

// stepDiscBoreSketch draws `Disc Bore {d+1}`: the enlarged centre bore on Od,
// whose diameter is CenterBearingDiameter + BearingClearance so the cam turns
// in it with the running gap.
func stepDiscBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch and rebuild Od")
	origin := groundedSketch(t, s)
	centre, _ := eccentricCentre(t, s, origin, d)

	proofkit.Step(t, "draw the solid bore circle on Od")
	bore := circleOn(t, s, centre, d.boreRadius(), "Disc Bore", false)

	proofkit.Step(t, "read the solved bore back")
	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "disc bore radius", bore.R(), d.boreRadius(), sketchtest.WithinRel(1e-12))
	// The bore has to clear the output holes, or the cut opens into them: the
	// spec rejects the dialog on exactly this comparison.
	if d.boreRadius() >= d.Rop-d.DHole/2 {
		t.Fatalf("the disc centre bore reaches %.6f mm, into the output holes whose inner edge "+
			"is at %.6f mm", d.boreRadius(), d.Rop-d.DHole/2)
	}

	profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
	sketchtest.IsValidProfile(t, profile)
	sketchtest.MeasuresProfileArea(t, profile, math.Pi*d.boreRadius()*d.boreRadius(),
		sketchtest.WithinRel(1e-9))
}

// stepEccentricCamSketch draws `Eccentric Cam {d+1}`: the cam outer on Od and,
// when the dialog asks for one, the input-shaft bore on O. The two circles are
// concentric with nothing — the E offset between them is the eccentricity — so
// the cross-section is an eccentric annulus with the bore as its one hole.
func stepEccentricCamSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch and rebuild Od")
	origin := groundedSketch(t, s)
	centre, _ := eccentricCentre(t, s, origin, d)

	proofkit.Step(t, "draw the cam outer on Od")
	outer := circleOn(t, s, centre, d.CBD/2, "Cam Outer", false)

	if d.ISD <= 0 {
		proofkit.Step(t, "Input Shaft Diameter is 0, so the section is the solid cam disc")
		sketchtest.Solve(t, s)
		sketchtest.Measures(t, "cam outer radius", outer.R(), d.CBD/2, sketchtest.WithinRel(1e-12))
		profile := sketchtest.SingleProfile(t, sketchtest.Verify(t, s))
		sketchtest.IsValidProfile(t, profile)
		sketchtest.MeasuresProfileArea(t, profile, math.Pi*d.CBD*d.CBD/4, sketchtest.WithinRel(1e-9))
		return
	}

	proofkit.Step(t, "draw the input-shaft bore on the drive axis O")
	bore := circleOn(t, s, origin, d.ISD/2, "Input Bore", false)
	// The bore has to fit inside the cam once the E offset is taken into
	// account, or the cam is not a closed annulus at all.
	if d.E+d.ISD/2 >= d.CBD/2 {
		t.Fatalf("the input bore reaches %.6f mm from Od, past the cam outer at %.6f mm",
			d.E+d.ISD/2, d.CBD/2)
	}

	proofkit.Step(t, "check the two regions the cam section detects as")
	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "cam outer radius", outer.R(), d.CBD/2, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, "input bore radius", bore.R(), d.ISD/2, sketchtest.WithinRel(1e-12))
	report := sketchtest.Verify(t, s)
	// Two nested circles detect as two regions: the bore disc and the cam
	// annulus. The extrude wants the annulus, which is the one carrying a hole
	// — the engine's reading of the spec's profileLoops.count == 2 rule, and
	// the reason find_profile_by_curve_counts cannot be used here.
	annulusProfile := oneHoledProfile(t, report.Profiles)
	sketchtest.IsValidProfile(t, annulusProfile)
	sketchtest.MeasuresProfileArea(t, annulusProfile,
		math.Pi*(d.CBD*d.CBD-d.ISD*d.ISD)/4, sketchtest.WithinRel(1e-9))
}

// stepHousingRingSketch draws `Housing Ring`: the plain base annulus on the
// drive axis, outer at the contour peak plus Wall and inner Wall inside the
// contour valley.
func stepHousingRingSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch on the drive axis O")
	origin := groundedSketch(t, s)

	proofkit.Step(t, "draw the annulus")
	outer := circleOn(t, s, origin, d.housingOuterRadius(), "Housing Outer", false)
	inner := circleOn(t, s, origin, d.housingInnerRadius(), "Housing Inner", false)

	proofkit.Step(t, "read the solved annulus back")
	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "housing outer radius", outer.R(), d.housingOuterRadius(),
		sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, "housing inner radius", inner.R(), d.housingInnerRadius(),
		sketchtest.WithinRel(1e-12))
	// The wall thickness at the contour peak is exactly Wall: that is the whole
	// claim the pinless outer diameter makes.
	sketchtest.Measures(t, "minimum wall at the contour peak",
		d.housingOuterRadius()-(d.R-d.Rr+2*d.E), d.Wall, sketchtest.WithinRel(1e-12))

	annulusProfile := oneHoledProfile(t, sketchtest.Verify(t, s).Profiles)
	sketchtest.IsValidProfile(t, annulusProfile)
	sketchtest.MeasuresProfileArea(t, annulusProfile,
		math.Pi*(d.housingOuterRadius()*d.housingOuterRadius()-
			d.housingInnerRadius()*d.housingInnerRadius()), sketchtest.WithinRel(1e-9))
}

// stepRingCasingSketch draws `Ring Casing`: the solid outer circle, the swept
// envelope contour over one pin pitch, and the two radial spokes at its ends.
//
// Two substitutions are made here and both are deliberate.
//
// The contour is drawn as the chord polyline through its points rather than as
// a fitted spline. decad refuses to extrude a free-form span whose curvature
// sign it cannot certify, and this contour turns from the pin bump into the
// mid-gap peak, so the solid steps downstream need a chorded boundary; drawing
// the same boundary here keeps the two artifacts describing one build. What it
// costs is the spline's own smoothness, which no step measures.
//
// The sketch is also fully constrained here, where the spec leaves it
// deliberately under-constrained: the contour's fit points and the spokes'
// outer endpoints are numeric snapshots that the sector extrude consumes
// immediately and nothing re-solves. proofkit's gate is DOF 0 with nothing
// waived, so the proof grounds exactly the points the spec calls snapshots and
// leaves every relation the spec does constrain — the outer circle's centre on
// the anchored origin and its diameter — as a constraint.
func stepRingCasingSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch on the drive axis O")
	origin := groundedSketch(t, s)

	proofkit.Step(t, "draw the SOLID outer circle that closes the wedge")
	outer := circleOn(t, s, origin, d.housingOuterRadius(), "Casing Outer", false)

	proofkit.Step(t, "draw the swept-envelope contour over one pin pitch")
	contour := contourPitch(d)
	pts := polyline(s, contour, false)

	proofkit.Step(t, "draw the two radial spokes out to the outer circle")
	half := math.Pi / float64(d.N)
	ro := d.housingOuterRadius()
	outFirst := s.CreatePoint(ro*math.Cos(-half), ro*math.Sin(-half))
	outLast := s.CreatePoint(ro*math.Cos(half), ro*math.Sin(half))
	s.CreateLine(pts[0], outFirst)
	s.CreateLine(pts[len(pts)-1], outLast)
	for _, q := range append(pts, outFirst, outLast) {
		s.Fix(q)
	}

	proofkit.Step(t, "check the contour lands exactly on the pin-pitch boundaries")
	sketchtest.Solve(t, s)
	sketchtest.Measures(t, "contour first point angle",
		math.Atan2(pts[0].Y(), pts[0].X()), -half, sketchtest.Within(1e-12))
	sketchtest.Measures(t, "contour last point angle",
		math.Atan2(pts[len(pts)-1].Y(), pts[len(pts)-1].X()), half, sketchtest.Within(1e-12))
	sketchtest.Measures(t, "casing outer radius", outer.R(), ro, sketchtest.WithinRel(1e-12))
	// The contour never reaches the outer wall and never falls inside the
	// valley floor: the wedge is a wall of real thickness everywhere.
	for i, q := range contour {
		r := math.Hypot(q.X, q.Y)
		if r >= ro {
			t.Fatalf("contour point %d reaches %.6f mm, at or past the outer wall %.6f mm", i, r, ro)
		}
	}

	proofkit.Step(t, "check the wedge is the smaller of the two regions the contour bounds")
	report := sketchtest.Verify(t, s)
	// The solid outer circle makes the open contour a shared edge of two closed
	// regions: the thin annular wedge, and the whole complement inside the
	// circle. Both contain the contour, which is why "the profile containing
	// the spline" is ambiguous in Fusion and the smaller area is the rule.
	if n := len(report.Profiles); n != 2 {
		t.Fatalf("the casing section detects as %d region(s), want the wedge and its complement", n)
	}
	wedge, complement := report.Profiles[0], report.Profiles[1]
	if wedge.Area > complement.Area {
		wedge, complement = complement, wedge
	}
	sketchtest.IsValidProfile(t, wedge)
	sketchtest.IsValidProfile(t, complement)
	want := polygonArea(append([]point{{0, 0}}, contour...)) // the contour's own pie
	sector := math.Pi*ro*ro/float64(d.N) - want
	sketchtest.MeasuresProfileArea(t, wedge, sector, sketchtest.WithinRel(2e-3))
	if complement.Area <= wedge.Area {
		t.Fatalf("the complement region measures %.6f mm^2, not larger than the wedge's %.6f mm^2",
			complement.Area, wedge.Area)
	}
}

// stepOutputPlateSketch draws `Output Plate`: the solid plate outer on O, the
// construction output-pin circle, and one solid output pin seated on it.
func stepOutputPlateSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := derive(p)

	proofkit.Step(t, "anchor the sketch on the drive axis O")
	origin := groundedSketch(t, s)

	proofkit.Step(t, "draw the plate outer and the construction output-pin circle")
	plate := circleOn(t, s, origin, d.plateRadius(), "Output Plate Outer", false)
	pinCircle := circleOn(t, s, origin, d.Rop, "Output Pin Circle", true)

	proofkit.Step(t, "draw the one solid output pin on the +X ray from O")
	pinCentre := s.CreatePoint(d.Rop, 0)
	pinCentre.SetName("output pin centre")
	pin := s.CreateCircle(pinCentre, d.DPin/2)
	pin.SetName("output pin")
	spoke := s.CreateLine(origin, pinCentre)
	spoke.SetConstruction(true)
	seatRadius := sketch.NewHorizontalDistance(origin, pinCentre, d.Rop)
	s.AddConstraint(
		sketch.NewHorizontal(spoke),
		seatRadius,
		sketch.NewDiameter(pin, d.DPin),
	)
	s.SetConstraintName(seatRadius, "output pin on the output-pin circle radius")

	proofkit.Step(t, "read the solved plate back")
	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, pinCentre, d.Rop, 0, sketchtest.Within(coordSlack))
	sketchtest.Measures(t, "output pin seating radius", pinCentre.DistanceTo(origin), pinCircle.R(),
		sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "output pin radius", pin.R(), d.DPin/2, sketchtest.WithinRel(1e-12))
	// The plate covers the pin circle by Wall: that is what OutputPlateDiameter
	// is for, and a plate that did not would leave the pins proud of its rim.
	sketchtest.Measures(t, "plate cover past the outermost pin",
		plate.R()-(d.Rop+d.DPin/2), d.Wall, sketchtest.WithinRel(1e-12))
	sketchtest.Satisfies(t, s.ConstraintByName("output pin on the output-pin circle radius"),
		sketchtest.Within(residualSlack))

	proofkit.Step(t, "check the two regions the plate and the pin detect as")
	report := sketchtest.Verify(t, s)
	if n := len(report.Profiles); n != 2 {
		t.Fatalf("the output plate sketch detects as %d region(s), want the pin disc and the "+
			"plate with its pin bite", n)
	}
	// The pin splits the plate disc: the extrude takes every profile for the
	// plate body, and the pin extrude takes the one whose single loop is the
	// pin itself.
	pinDisc := report.Profiles[0]
	bitten := report.Profiles[1]
	if len(pinDisc.Holes) == 1 {
		pinDisc, bitten = bitten, pinDisc
	}
	sketchtest.IsValidProfile(t, pinDisc)
	sketchtest.IsValidProfile(t, bitten)
	if len(pinDisc.Holes) != 0 || len(bitten.Holes) != 1 {
		t.Fatalf("regions detected with %d and %d hole(s), want 0 for the pin disc and 1 for the plate",
			len(pinDisc.Holes), len(bitten.Holes))
	}
	sketchtest.MeasuresProfileArea(t, pinDisc, math.Pi*d.DPin*d.DPin/4, sketchtest.WithinRel(1e-9))
	sketchtest.MeasuresProfileArea(t, bitten,
		math.Pi*(d.plateRadius()*d.plateRadius()-d.DPin*d.DPin/4), sketchtest.WithinRel(1e-9))
}

// oneHoledProfile returns the single region carrying exactly one hole, which is
// how every annular cross-section in this gear is selected.
func oneHoledProfile(t testing.TB, profiles []*sketch.Profile) *sketch.Profile {
	t.Helper()
	var found *sketch.Profile
	for _, p := range profiles {
		if len(p.Holes) != 1 {
			continue
		}
		if found != nil {
			t.Fatalf("the sketch holds more than one region with a hole")
		}
		found = p
	}
	if found == nil {
		t.Fatalf("the sketch holds no region with a hole; %d region(s) detected", len(profiles))
	}
	return found
}
