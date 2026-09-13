package bevelgear_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// The input bounds — instructions.md's Variables section, resolved.
//
// Everything this step owns is closed form over the dialog inputs, so it needs
// no sketch and no solid to compute. It is proved through the sketch harness
// anyway, and the sketch is not a formality: the frustum hexagon drawn at the
// RESOLVED values is the witness that what validation admits is a profile that
// stays on one side of its own axis of revolution, which is the whole reason
// the Maximum Base Height and Maximum Face Width exist.
// ---------------------------------------------------------------------------

// requireRejected asserts that one perturbed input is refused, and that the
// refusal names the input and the bound. It is what separates a bound that is
// merely computed from one that is enforced.
func requireRejected(t testing.TB, base map[string]float64, key string, value float64, want string) {
	t.Helper()
	p := make(map[string]float64, len(base)+1)
	for k, v := range base {
		p[k] = v
	}
	p[key] = value
	_, problems := resolveDesign(p)
	for _, problem := range problems {
		if strings.Contains(problem, want) {
			return
		}
	}
	t.Fatalf("%s = %g was accepted; wanted a rejection naming %q, got %v", key, value, want, problems)
}

// requireAccepted is its counterpart: the value one step INSIDE a bound is
// admitted, so the rejections above are pinning the bound rather than refusing
// everything near it.
func requireAccepted(t testing.TB, base map[string]float64, key string, value float64) {
	t.Helper()
	p := make(map[string]float64, len(base)+1)
	for k, v := range base {
		p[k] = v
	}
	p[key] = value
	if _, problems := resolveDesign(p); len(problems) > 0 {
		t.Fatalf("%s = %g was rejected but sits inside its bound: %v", key, value, problems)
	}
}

// stepResolveInputBounds resolves the Maximum Shaft Angle, the Minimum Teeth
// floor, each gear's base-height window and the Maximum Face Width, applies
// each of them to its own input, and draws the frustum the resolved values
// produce.
//
// It checks four things the Variables section states and nothing else does:
//
//   - the published worked values, named by the case that carries them, so the
//     spec's own arithmetic is compared against the resolution rather than
//     against itself;
//   - the identities the section reasons from — that the unfactored base-height
//     cap sits exactly 1.25*Module*sin(gamma) below the true heel crossing,
//     that the two base-height bounds meet at the Minimum Teeth count, that the
//     Maximum Face Width is PitchDiameter^2/(2*ConeDistance) at 90 degrees, and
//     that the Cone Distance is 2R there and only there;
//   - that a fallback below a minimum is RAISED and one above a maximum is
//     CAPPED, while a user value outside either end is REJECTED rather than
//     clamped — three different behaviours the section assigns to three
//     different cases;
//   - that the resolved values build: the frustum hexagon at those values is a
//     single closed all-line loop that never crosses the shaft axis.
//
// <!-- proof-run: proofkit.RunParallel(boundsCases, stepResolveInputBounds) -->
func stepResolveInputBounds(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	b := d.Bounds

	proofkit.Step(t, "the published worked values this case names")
	if want := p[keyExpectMaxBaseHeight]; want != 0 {
		requireClose(t, b.MaxBaseHeightDriving, want, 5e-4, "published Maximum Base Height")
	}
	if want := p[keyExpectTrueCrossing]; want != 0 {
		requireClose(t, b.TrueCrossingDriving, want, 5e-4, "published true heel crossing")
	}
	if want := p[keyExpectDrivingFallback]; want != 0 {
		requireClose(t, b.DrivingFallback, want, tightTol, "published driving fallback")
	}
	if want := p[keyExpectResolvedBase]; want != 0 {
		requireClose(t, d.Driving.BaseHeight, want, 5e-4, "published resolved driving base height")
	}
	if want := p[keyExpectMaxShaftAngle]; want != 0 {
		requireClose(t, b.MaxShaftAngleDeg, want, 5e-4, "published Maximum Shaft Angle")
		if got := 0.0; b.ShaftAngleExclusive {
			got = 1
		} else if p[keyExpectShaftExclusive] != got {
			t.Fatalf("Maximum Shaft Angle exclusivity: got %v, want %g",
				b.ShaftAngleExclusive, p[keyExpectShaftExclusive])
		}
	}
	if want := p[keyExpectMinTeeth]; want != 0 {
		requireClose(t, b.MinTeethDriving, want, 5e-4, "published Minimum Teeth floor")
	}

	proofkit.Step(t, "the identities the Variables section reasons from")
	for _, side := range []struct {
		g          gear
		minBH, max float64
		crossing   float64
	}{
		{d.Pinion, b.MinBaseHeightPinion, b.MaxBaseHeightPinion, b.TrueCrossingPinion},
		{d.Driving, b.MinBaseHeightDriving, b.MaxBaseHeightDriving, b.TrueCrossingDriving},
	} {
		r := side.g.PitchDiameter / 2
		sin, cos, tan := math.Sin(side.g.Gamma), math.Cos(side.g.Gamma), math.Tan(side.g.Gamma)
		// Read the origin carefully: the base height is measured from Apex 2's
		// plane, not from the dedendum point, so the heel point reaches the
		// axis at r*tan(gamma) and the cap starts one dedendum inside that.
		requireClose(t, (r-1.25*d.Module*cos)*tan, side.crossing-1.25*d.Module*sin, tightTol,
			"%s unfactored cap sits 1.25*Module*sin(gamma) below the true crossing", side.g.Label)
		// The 0.95 factor then makes it deliberately conservative: it refuses a
		// band of base heights that would in fact still build.
		if !(side.max < side.crossing) {
			t.Fatalf("%s Maximum Base Height %.6f is not below the true crossing %.6f",
				side.g.Label, side.max, side.crossing)
		}
		// The window is non-empty, which is exactly what the Minimum Teeth
		// check upstream guarantees.
		if !(side.minBH <= side.max) {
			t.Fatalf("%s base-height window is empty: minimum %.6f exceeds maximum %.6f",
				side.g.Label, side.minBH, side.max)
		}
		// The two bounds cross at teeth = 2*(1.05*1.25/0.95 + 1.25)*cos(gamma).
		// instructions.md rounds that constant UP to 5.27, which keeps the
		// published floor conservative; the exact crossing is below it.
		exact := 2 * (1.05*1.25/0.95 + 1.25) * cos
		crossM := exact * d.Module
		requireClose(t,
			minimumBaseHeight(d.Module, side.g.Gamma),
			maximumBaseHeight(d.Module, crossM/2, side.g.Gamma), slackTol,
			"%s base-height bounds meet at the exact tooth-count crossing", side.g.Label)
		if !(exact <= minimumTeeth(side.g.Gamma)+slackTol) {
			t.Fatalf("%s published Minimum Teeth floor %.6f is below the exact crossing %.6f",
				side.g.Label, minimumTeeth(side.g.Gamma), exact)
		}
		// The fallback clears the raw dedendum projection exactly while the
		// tooth count exceeds 10*sin(gamma).
		if side.g.Label == "Driving" {
			clears := b.DrivingFallback > 1.25*d.Module*sin
			if clears != (side.g.Teeth > 10*sin) {
				t.Fatalf("the driving fallback's clearance disagrees with teeth > 10 sin(gamma): "+
					"fallback %.6f, projection %.6f, teeth %g, bound %.6f",
					b.DrivingFallback, 1.25*d.Module*sin, side.g.Teeth, 10*sin)
			}
		}
	}
	requireClose(t, d.Pinion.Gamma+d.Driving.Gamma, d.Sigma, tightTol,
		"gamma_p + gamma_g = Shaft Angle")
	if math.Abs(p[keyShaftAngle]-90) < tightTol {
		// The two lengths coincide as Cone Distance = 2R exactly at 90 degrees,
		// for any pair of tooth counts, and diverge everywhere else.
		requireClose(t, d.ConeDistance, 2*d.R, slackTol, "Cone Distance is 2R at 90 degrees")
		requireClose(t, b.MaxFaceWidth,
			0.95*math.Min(
				d.Pinion.PitchDiameter*d.Pinion.PitchDiameter,
				d.Driving.PitchDiameter*d.Driving.PitchDiameter)/(2*d.ConeDistance),
			slackTol, "Maximum Face Width is PitchDiameter^2 / (2 Cone Distance) at 90 degrees")
	} else if math.Abs(d.ConeDistance-2*d.R) < slackTol {
		t.Fatalf("Cone Distance equals 2R at a %g degree Shaft Angle, where they should diverge",
			p[keyShaftAngle])
	}

	proofkit.Step(t, "raise a low fallback, cap a high one, reject a user value outside either")
	// The base heights this case resolved sit inside their own windows, and
	// each is the fallback clamped rather than the raw fallback.
	if p[keyDrivingBase] == 0 {
		requireClose(t, d.Driving.BaseHeight,
			math.Min(math.Max(b.DrivingFallback, b.MinBaseHeightDriving), b.MaxBaseHeightDriving),
			tightTol, "driving fallback clamped into its window")
	} else {
		// A user value inside the window is used AS GIVEN. Clamping is what a
		// fallback gets; a user value gets accepted or refused.
		requireClose(t, d.Driving.BaseHeight, p[keyDrivingBase], tightTol,
			"a user driving base height inside its window is used unchanged")
	}
	if p[keyPinionBase] != 0 {
		requireClose(t, d.Pinion.BaseHeight, p[keyPinionBase], tightTol,
			"a user pinion base height inside its window is used unchanged")
	}
	if p[keyPinionBase] == 0 {
		requireClose(t, d.Pinion.BaseHeight,
			math.Min(math.Max(b.PinionFallback, b.MinBaseHeightPinion), b.MaxBaseHeightPinion),
			tightTol, "pinion fallback clamped into its own window")
		// The pinion fallback scales the RESOLVED driving height, never the raw
		// input, and then takes the pinion's own window.
		requireClose(t, b.PinionFallback, d.Driving.BaseHeight*d.Pinion.Teeth/d.Driving.Teeth,
			tightTol, "pinion fallback scales the resolved driving height")
	}
	// A user value one step outside either end is refused, not clamped, and the
	// refusal names the bound.
	base := map[string]float64{}
	for k, v := range p {
		base[k] = v
	}
	delete(base, keyDrivingBase)
	delete(base, keyPinionBase)
	delete(base, keyFaceWidth)
	requireRejected(t, base, keyDrivingBase, b.MaxBaseHeightDriving*1.001,
		"is above the Maximum Base Height")
	requireRejected(t, base, keyDrivingBase, b.MinBaseHeightDriving*0.999,
		"is below the Minimum Base Height")
	requireRejected(t, base, keyPinionBase, b.MaxBaseHeightPinion*1.001,
		"is above the Maximum Base Height")
	requireRejected(t, base, keyPinionBase, b.MinBaseHeightPinion*0.999,
		"is below the Minimum Base Height")
	requireRejected(t, base, keyFaceWidth, b.MaxFaceWidth*1.001,
		"exceeds the Maximum Face Width")
	requireAccepted(t, base, keyDrivingBase, b.MaxBaseHeightDriving*0.999)
	requireAccepted(t, base, keyDrivingBase, b.MinBaseHeightDriving*1.001)
	requireAccepted(t, base, keyFaceWidth, b.MaxFaceWidth*0.999)

	// The Shaft Angle is refused AT the cone-angle limit, which is a hard
	// singularity, and ABOVE the 150 degree practical ceiling, which is not.
	if b.ShaftAngleExclusive {
		requireRejected(t, base, keyShaftAngle, b.MaxShaftAngleDeg,
			"is not below the Maximum Shaft Angle")
	} else {
		requireAccepted(t, base, keyShaftAngle, b.MaxShaftAngleDeg)
		requireRejected(t, base, keyShaftAngle, b.MaxShaftAngleDeg+0.01,
			"is not below the Maximum Shaft Angle")
	}
	requireRejected(t, base, keyShaftAngle, 29.99, "is below the 30 deg floor")
	// The computed tooth floor sits on top of the blanket floor of 3, so one
	// tooth below whichever binds is refused.
	requireRejected(t, base, keyDrivingTeeth,
		math.Ceil(math.Max(3, b.MinTeethDriving))-1, "Driving Gear Teeth")
	requireRejected(t, base, keySpiralAngle, 60, "is outside [0, 60)")
	requireAccepted(t, base, keySpiralAngle, 59.99)
	requireRejected(t, base, keyCutterRadius, -1, "is negative")
	requireRejected(t, base, keyToothSpacing, -1, "is negative")
	requireRejected(t, base, keyModule, 0, "Module must be positive")

	proofkit.Step(t, "resolve the toe end: the Toe Radius, X, and what the extension reaches")
	// Toe Extension 0 puts the root length at the resolved Face Width measured
	// along the root element instead of perpendicular to the pitch line, and
	// leaves each toe corner where it has always been. That equality is what
	// makes 0 a no-op for every gear built before the extension existed.
	requireClose(t, b.RootLengthBase, d.FaceWidth*rootConeDistance(d.Module, d.R)/d.R,
		tightTol, "root length at Toe Extension 0 is the Face Width along the root element")
	if p[keyToeExtension] == 0 {
		requireClose(t, d.RootLength, b.RootLengthBase, tightTol,
			"Toe Extension 0 resolves to the base root length")
		for _, side := range []struct {
			g     gear
			gamma float64
		}{{d.Pinion, d.Pinion.Gamma}, {d.Driving, d.Driving.Gamma}} {
			f := gearLattice(d, side.g)
			requireCloseVec(t, f.Front, f.Axis, slackTol,
				"%s front foot is the drop foot at Toe Extension 0", side.g.Label)
			requireClose(t, f.ToeIn.Y,
				side.g.PitchDiameter/2-d.FaceWidth/math.Sin(side.gamma), slackTol,
				"%s toe corner is still on the Apex 2 drop at Toe Extension 0", side.g.Label)
		}
	}
	// A defaulted Toe Radius IS that corner's own radius, which is the same
	// statement read the other way round.
	if p[keyPinionToeRadius] == 0 {
		requireClose(t, d.Pinion.ToeRadius, b.ToeRadiusDefaultPinion, tightTol,
			"the pinion Toe Radius defaults to its own toe corner")
	}
	if p[keyDrivingToeRadius] == 0 {
		requireClose(t, d.Driving.ToeRadius, b.ToeRadiusDefaultDriving, tightTol,
			"the driving Toe Radius defaults to its own toe corner")
	}
	// X is the point on Apex->Ded at the Toe Radius, so |Ded->X| is the toe
	// limit and the extension's 100% lands the ceiling's fraction short of it.
	for _, side := range []struct {
		label          string
		g              gear
		limit, ceiling float64
	}{
		{"pinion", d.Pinion, b.ToeLimitPinion, b.ToeRadiusCeilingPinion},
		{"driving", d.Driving, b.ToeLimitDriving, b.ToeRadiusCeilingDriving},
	} {
		rhoX := side.limit
		requireClose(t, rootConeDistance(d.Module, d.R)-rhoX,
			side.g.ToeRadius/math.Sin(rootConeAngle(d.Module, d.R, side.g.Gamma)),
			slackTol, "%s X sits at the Toe Radius on Apex->Ded", side.label)
		// The ceiling is the OUTER toe corner's radius. Below it the extension
		// has room; at or above it X falls behind the toe corner.
		hasRoom := side.limit > b.RootLengthBase
		if hasRoom != (side.g.ToeRadius < side.ceiling) {
			t.Fatalf("%s toe room disagrees with the Toe Radius ceiling: "+
				"limit %.6f, base %.6f, radius %.6f, ceiling %.6f",
				side.label, side.limit, b.RootLengthBase, side.g.ToeRadius, side.ceiling)
		}
	}
	smallestLimit := math.Min(b.ToeLimitPinion, b.ToeLimitDriving)
	if smallestLimit > b.RootLengthBase {
		requireClose(t, b.MaxRootLength,
			b.RootLengthBase+toeExtensionCeiling*(smallestLimit-b.RootLengthBase),
			tightTol, "Toe Extension 100 stops the ceiling's fraction short of the smaller X")
		requireClose(t, d.RootLength,
			b.RootLengthBase+d.ToeExtension*toeExtensionCeiling*(smallestLimit-b.RootLengthBase),
			tightTol, "the resolved root length is linear in the Toe Extension")
		// The pair shares one root length, so the SMALLER toe limit decides and
		// the other gear simply stops short of its own X.
		if d.RootLength > smallestLimit {
			t.Fatalf("the resolved root length %.6f passes the smaller toe limit %.6f",
				d.RootLength, smallestLimit)
		}
	}

	toeBase := map[string]float64{}
	for k, v := range p {
		toeBase[k] = v
	}
	delete(toeBase, keyToeExtension)
	delete(toeBase, keyPinionToeRadius)
	delete(toeBase, keyDrivingToeRadius)
	requireRejected(t, toeBase, keyToeExtension, 100.01, "is outside [0, 100]")
	requireRejected(t, toeBase, keyToeExtension, -0.01, "is outside [0, 100]")
	if smallestLimit > b.RootLengthBase {
		requireAccepted(t, toeBase, keyToeExtension, 100)
	} else {
		// A pair whose defaulted Toe Radius leaves no room refuses the
		// extension itself and names the radius that would buy some. Extension
		// 0 still resolves, which is what keeps the gear buildable.
		requireRejected(t, toeBase, keyToeExtension, 50, "which leaves no room")
		requireAccepted(t, toeBase, keyToeExtension, 0)
	}
	requireRejected(t, toeBase, keyPinionToeRadius, b.ToeRadiusCeilingPinion,
		"is at or above the toe corner")
	requireRejected(t, toeBase, keyDrivingToeRadius, b.ToeRadiusCeilingDriving,
		"is at or above the toe corner")
	requireAccepted(t, toeBase, keyPinionToeRadius, b.ToeRadiusCeilingPinion*0.999)

	proofkit.Step(t, "the witness: the frustum these resolved values produce")
	// The bounds exist so that this profile stays on one side of the axis it is
	// revolved about. Drawing it at the resolved values is what makes the
	// arithmetic above a statement about geometry.
	pinionFrame := gearLattice(d, d.Pinion)
	pts, lines, names := drawFrustumHexagon(t, s, d.Pinion, pinionFrame)
	requireFrustumProfile(t, s, d.Pinion, pinionFrame, pts, lines, names)
}
