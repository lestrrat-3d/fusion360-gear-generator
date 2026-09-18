// Package bevelgear_test proves the bevel pair's build, one function per step of
// spec/bevelgear/steps.md.
//
// Units. The generator works in Fusion-internal centimetres; this proof works in
// millimetres throughout, because both engines are millimetre-native. The figure
// is scale-free in Module, so the choice changes no shape, only the number a
// dimension carries. Every case table is written in the dialog's own display
// units.
//
// This file holds the geometry the steps share: the closed-form §2 lattice, the
// per-gear frustum profile, the §3 virtual-spur dimensions and the §3a spiral
// trace. Nothing here touches either engine.
//
// Two frames are used and they must not be mixed.
//
//   - The ANCHOR frame is the Gear Profiles sketch's own 2-D frame: the projected
//     anchor line runs along +u, the projected centre sits at the origin, and the
//     grow direction perp is (0, 1). Every §2 point is stated here, because
//     [BEVEL-F-APEX-LOCAL] places the whole figure in sketch-local coordinates
//     and [BEVEL-F-SEED-HELD] compares it there with no world round-trip.
//   - The PROFILE frame is per gear: x is the station along that gear's shaft
//     axis measured from the Apex, y is the perpendicular distance from that
//     axis. The hexagon the revolve consumes, the tooth plane and every solid
//     step are stated here, because the shaft axis is the axis of revolution and
//     a frame whose x is that axis makes every reading a radius or a station.
//
// The two agree by construction: a profile-frame point (x, y) is the anchor-frame
// point Apex + x*axisDir + y*radialDir for that gear's own axis, and
// latticePoints builds the anchor-frame figure out of the profile-frame one.
package bevelgear_test

import (
	"math"
	"testing"
)

// Case-table parameter keys. Every dialog input's key, and the gear-side key a
// per-gear case carries, are the ones `drawing_geometry_test.go` already
// declares for this package — `keyModule`, `keyShaftAngle`, `keyGearSide` and
// the rest — so one case table feeds a step here, the README drawing and the
// per-step snapshots alike. Two of them are DEGREES, `shaftAngleDeg` and
// `spiralAngleDeg`, and are converted where they are read.
//
// Only one key is this file's own, because no drawing has a use for it.
const keyLatticeRefused = "latticeRefused" // 1 = this net cannot reach the case

// The constants the spec fixes by number, each at its own name so a step reads
// the rule rather than the digits.
const (
	dedendumFactor     = 1.25 // dedendum = 1.25 * Module
	addendumFactor     = 1.0  // addendum = 1 * Module
	toothPressureAngle = 20 * math.Pi / 180
	toothInvoluteSteps = 15
	rootSinkFactor     = 0.05 * 2.25 // root sink = 0.05 * 2.25 * Module
	guardFactor        = 0.95        // the Maximum Face Width / Base Height / Bore factor
	baseHeightMargin   = 1.05        // the Minimum Base Height margin
	minTeethConstant   = 5.27        // 2 * (1.05*1.25/0.95 + 1.25), rounded UP
	shaftAngleCeiling  = 150.0       // degrees, the practical cap on the cone-angle limit
	shaftAngleFloor    = 30.0        // degrees, the documented floor
	spiralSlabs        = 8           // §3a step E: exactly 8 offset planes, not configurable
	crownPerRad        = 0.5         // _CROWN_PER_RAD
	traceOvershoot     = 0.06        // R_lo = R_toe - 0.06*span, R_hi = R_heel + 0.06*span
)

// pt2 is a plane-local point in millimetres. It carries no frame of its own; the
// caller knows which of the two frames it is in.
//
// It is deliberately NOT `drawing_geometry_test.go`'s `pt2`, even though the two
// hold the same pair of floats. That file says in its own comment that borrowing
// its vector broke those drawings twice, so it carries its own and this one
// carries its own; the drawings and the proof share the parameter keys and
// nothing else.
type pt2 struct{ X, Y float64 }

func (a pt2) add(b pt2) pt2       { return pt2{a.X + b.X, a.Y + b.Y} }
func (a pt2) sub(b pt2) pt2       { return pt2{a.X - b.X, a.Y - b.Y} }
func (a pt2) mul(k float64) pt2   { return pt2{a.X * k, a.Y * k} }
func (a pt2) cross(b pt2) float64 { return a.X*b.Y - a.Y*b.X }
func (a pt2) dot(b pt2) float64   { return a.X*b.X + a.Y*b.Y }
func (a pt2) length() float64     { return math.Hypot(a.X, a.Y) }
func (a pt2) unit() pt2           { return a.mul(1 / a.length()) }

func degrees(rad float64) float64 { return rad * 180 / math.Pi }
func radians(deg float64) float64 { return deg * math.Pi / 180 }

// member is one gear of the pair: everything that is stated per gear.
type member struct {
	label     string  // "Pinion" or "Driving", the {gearLabel} every name is built from
	teeth     float64 // this gear's Teeth Number
	gamma     float64 // this gear's pitch cone angle, radians
	gammaRoot float64 // gamma - atan(1.25*Module/R), the root cone angle
	pitchDia  float64 // this gear's Pitch Diameter, mm
	radius    float64 // pitchDia / 2

	minBaseHeight float64 // 1.05 * 1.25 * Module * sin(gamma)
	maxBaseHeight float64 // 0.95 * (r - 1.25*Module*cos(gamma)) * tan(gamma)
	minTeeth      float64 // 5.27 * cos(gamma)
	baseHeight    float64 // RESOLVED, mm

	toeRadius        float64 // RESOLVED, mm
	toeRadiusCeiling float64 // (r - 1.25*Module*cos(gamma)) * (1 - FaceWidth/R)
	toeLimit         float64 // |Ded->X|

	maxBore float64 // 2 * 0.95 * min(r_heel, r_toe)
	bore    float64 // RESOLVED and bounded, mm

	virtualPitchRadius float64 // r / cos(gamma), the exact back-cone radius
	virtualTeeth       float64 // 2 * virtualPitchRadius / Module — a REAL number
}

// lattice is the whole resolved figure: the two members plus everything shared.
type lattice struct {
	module   float64
	sigma    float64 // Shaft Angle, radians
	dedendum float64 // 1.25 * Module
	rootSink float64 // 0.05 * 2.25 * Module

	coneDistance   float64 // sqrt(PPD^2 + DPD^2) — the DIAGONAL, not R
	pitchCone      float64 // R = (PPD/2)/sin(gamma_p) — the Pitch Cone Distance
	apexToDedendum float64 // |Apex->Ded| = sqrt(R^2 + (1.25*Module)^2)
	maxShaftAngle  float64 // degrees

	maxFaceWidth float64 // 0.95 * min(r_p*sin(gamma_p), r_g*sin(gamma_g))
	faceWidth    float64 // RESOLVED
	rootLength0  float64 // |Ded->Toe| at Toe Extension 0
	rootLength   float64 // RESOLVED |Ded->Toe|

	toothSpacing float64
	pinion       member
	driving      member
}

// gearOf picks the member a per-gear case names. 0 is the Pinion, which is built
// first; 1 is the Driving gear.
func (l lattice) gearOf(p map[string]float64) member {
	if p[keyGearSide] >= 0.5 {
		return l.driving
	}
	return l.pinion
}

// maximumShaftAngle is the cone-angle limit capped at 150 degrees. The
// cone-angle half is where a pitch cone angle reaches 90 degrees, at which
// R*cos(gamma) passes through zero and the along-shaft seeds change sign; it is
// a hard singularity, so the range check rejects a Shaft Angle at or ABOVE it.
// The 150 degree half is a practical ceiling on the figure and is inclusive.
func maximumShaftAngle(ppd, dpd float64) float64 {
	return math.Min(shaftAngleCeiling,
		degrees(math.Acos(-math.Min(ppd, dpd)/math.Max(ppd, dpd))))
}

// newMember fills in everything about one gear that needs no resolved length.
func newMember(label string, teeth, gamma, pitchDia float64, l lattice) member {
	r := pitchDia / 2
	return member{
		label:              label,
		teeth:              teeth,
		gamma:              gamma,
		gammaRoot:          gamma - math.Atan(l.dedendum/l.pitchCone),
		pitchDia:           pitchDia,
		radius:             r,
		minBaseHeight:      baseHeightMargin * l.dedendum * math.Sin(gamma),
		maxBaseHeight:      guardFactor * (r - l.dedendum*math.Cos(gamma)) * math.Tan(gamma),
		minTeeth:           minTeethConstant * math.Cos(gamma),
		virtualPitchRadius: r / math.Cos(gamma),
		virtualTeeth:       2 * (r / math.Cos(gamma)) / l.module,
	}
}

// resolveBaseHeight applies the two base-height bounds in both directions: it
// raises a fallback below the minimum, caps one above the maximum, and rejects a
// user value outside either end naming the bound it broke.
func resolveBaseHeight(t testing.TB, g member, user, fallback float64) float64 {
	t.Helper()
	if user > 0 {
		if user < g.minBaseHeight {
			t.Fatalf("%s Gear Base Height %g mm is below the minimum %.4f mm",
				g.label, user, g.minBaseHeight)
		}
		if user > g.maxBaseHeight {
			t.Fatalf("%s Gear Base Height %g mm is above the maximum %.4f mm",
				g.label, user, g.maxBaseHeight)
		}
		return user
	}
	return math.Min(math.Max(fallback, g.minBaseHeight), g.maxBaseHeight)
}

// resolveToe fills the three toe quantities that need the resolved Face Width.
// The auto Toe Radius is the gear's own inner toe corner radius at Toe Extension
// 0, which is the one value that reproduces today's profile exactly.
func resolveToe(t testing.TB, g member, l lattice, user float64) member {
	t.Helper()
	g.toeRadiusCeiling = (g.radius - l.dedendum*math.Cos(g.gamma)) * (1 - l.faceWidth/l.pitchCone)
	if user > 0 {
		if user >= g.toeRadiusCeiling {
			t.Fatalf("%s Gear Toe Radius %g mm is at or above the ceiling %.4f mm",
				g.label, user, g.toeRadiusCeiling)
		}
		g.toeRadius = user
	} else {
		g.toeRadius = g.radius - l.faceWidth/math.Sin(g.gamma)
	}
	g.toeLimit = l.apexToDedendum - g.toeRadius/math.Sin(g.gammaRoot)
	return g
}

// resolveBore applies the Maximum Bore Diameter. Its heel term is where H lands
// and its toe term is where M lands, so the bound cannot resolve before the Root
// Length does — which is why §2, not the input pass, owns it.
func resolveBore(t testing.TB, g member, l lattice, user float64, enabled bool) member {
	t.Helper()
	rHeel := g.radius - g.baseHeight/math.Tan(g.gamma)
	rToe := (l.apexToDedendum - l.rootLength) * math.Sin(g.gammaRoot)
	g.maxBore = 2 * guardFactor * math.Min(rHeel, rToe)
	if !enabled {
		g.bore = 0
		return g
	}
	if user > 0 {
		if user > g.maxBore {
			t.Fatalf("%s Gear Bore Diameter %g mm is above the maximum %.4f mm",
				g.label, user, g.maxBore)
		}
		g.bore = user
		return g
	}
	g.bore = math.Min(g.pitchDia/4, g.maxBore)
	return g
}

// deriveLattice resolves the whole figure from one case's dialog values, in the
// order "Reading the raw numbers" fixes: the two cone angles, then the Minimum
// Teeth floor, then the two base heights, then — in §2, where the solved figure
// exists — the Maximum Face Width, the Face Width, the Root Length and the two
// Maximum Bore Diameters.
//
// Every bound is stated in closed form here because that is what the generated
// module computes; the sketch step then checks the same numbers against the
// SOLVED lattice, which is the half a closed form cannot give itself.
func deriveLattice(t testing.TB, p map[string]float64) lattice {
	t.Helper()
	m := p[keyModule]
	// The two angle keys are DEGREES, which is what the shared parameter set holds
	// and what the dialog shows; every angle below this line is radians.
	sigma := radians(p[keyShaftAngle])
	nd, np := p[keyDrivingTeeth], p[keyPinionTeeth]
	ppd, dpd := m*np, m*nd

	l := lattice{
		module:       m,
		sigma:        sigma,
		dedendum:     dedendumFactor * m,
		rootSink:     rootSinkFactor * m,
		coneDistance: math.Hypot(ppd, dpd),
		toothSpacing: p[keyToothSpacing],
	}
	l.maxShaftAngle = maximumShaftAngle(ppd, dpd)
	// The floor is inclusive ("at least 30 degrees") and so is the 150 degree half
	// of the ceiling; the epsilon is float slack on the degree round-trip, not a
	// widening of either bound.
	if d := degrees(sigma); d < shaftAngleFloor-1e-9 || d > l.maxShaftAngle+1e-9 {
		t.Fatalf("Shaft Angle %.4f deg is outside [%g, %g]", d, shaftAngleFloor, l.maxShaftAngle)
	}

	// tan(gamma_p) = sin(Sigma) * PPD / (DPD + PPD*cos(Sigma)); gamma_g = Sigma - gamma_p.
	gammaP := math.Atan2(math.Sin(sigma)*ppd, dpd+ppd*math.Cos(sigma))
	gammaG := sigma - gammaP
	l.pitchCone = (ppd / 2) / math.Sin(gammaP)
	l.apexToDedendum = math.Hypot(l.pitchCone, l.dedendum)

	l.pinion = newMember("Pinion", np, gammaP, ppd, l)
	l.driving = newMember("Driving", nd, gammaG, dpd, l)

	// 1. The Minimum Teeth floor, per gear with that gear's own gamma, on top of
	//    the blanket teeth >= 3. It is exactly the statement that the base-height
	//    window below is non-empty, which is why it runs first.
	for _, g := range []member{l.pinion, l.driving} {
		if g.teeth < 3 || g.teeth < g.minTeeth {
			t.Fatalf("%s Gear Teeth %g is below the floor %.4f", g.label, g.teeth, g.minTeeth)
		}
	}

	// 2. The base heights. The driving gear resolves first because the pinion's
	//    fallback is the RESOLVED driving height scaled by the tooth ratio, and
	//    the pinion's own bounds are then applied on top of that.
	l.driving.baseHeight = resolveBaseHeight(t, l.driving, p[keyDrivingBase], m*nd/8)
	l.pinion.baseHeight = resolveBaseHeight(t, l.pinion, p[keyPinionBase],
		l.driving.baseHeight*(np/nd))

	// 3. The Maximum Face Width. The perpendicular distance from A to the Pinion
	//    Dedendum line C->H is exactly r_p*sin(gamma_p), and from B to D->J is
	//    r_g*sin(gamma_g); take the smaller, since EITHER gear can be the binding
	//    one. At Shaft Angle 90 this is min(PitchDiameter)^2 / (2*ConeDistance).
	l.maxFaceWidth = guardFactor * math.Min(
		l.pinion.radius*math.Sin(l.pinion.gamma),
		l.driving.radius*math.Sin(l.driving.gamma))
	if user := p[keyFaceWidth]; user > 0 {
		if user > l.maxFaceWidth {
			t.Fatalf("Face Width %g mm is above the maximum %.4f mm", user, l.maxFaceWidth)
		}
		l.faceWidth = user
	} else {
		l.faceWidth = math.Min(l.coneDistance/6, l.maxFaceWidth)
	}

	// 4. The Root Length. At Toe Extension 0 it is the Face Width re-measured
	//    along the root element, longer by the dedendum angle's cosine.
	l.rootLength0 = l.faceWidth * l.apexToDedendum / l.pitchCone
	l.pinion = resolveToe(t, l.pinion, l, p[keyPinionToeRadius])
	l.driving = resolveToe(t, l.driving, l, p[keyDrivingToeRadius])
	l.rootLength = l.rootLength0
	if pct := p[keyToeExtension]; pct > 0 {
		smaller := math.Min(l.pinion.toeLimit, l.driving.toeLimit)
		if smaller <= l.rootLength0 {
			t.Fatalf("Toe Extension %g leaves no window: the smaller Toe Limit %.4f mm is at or "+
				"below the Toe Extension 0 root length %.4f mm", pct, smaller, l.rootLength0)
		}
		l.rootLength = l.rootLength0 + (pct/100)*toeExtensionCeiling*(smaller-l.rootLength0)
	}

	// 5. The Maximum Bore Diameter, whose toe term needs the Root Length — which
	//    is why the whole bound resolves here and not in the input pass.
	enabled := p[keyBoreEnable] >= 0.5
	l.pinion = resolveBore(t, l.pinion, l, p[keyPinionBore], enabled)
	l.driving = resolveBore(t, l.driving, l, p[keyDrivingBore], enabled)
	return l
}

// ---------------------------------------------------------------- profile frame

// hexagon is one gear's frustum profile in the PROFILE frame, in the draw order
// the Profile sketch uses: A' -> G -> H -> C -> M -> N (B' -> I -> J -> D -> O ->
// P on the driving side). Its FIRST edge A'->G lies along the shaft axis, which
// is what the revolve, the pattern, the bore plane and the mesh rotation all use.
type hexagon struct {
	aPrime, g, h, c, m, n pt2
}

func (x hexagon) points() []pt2 { return []pt2{x.aPrime, x.g, x.h, x.c, x.m, x.n} }

// profileHexagon states the six vertices in closed form.
//
// Walking out along the dedendum line from Apex 2, the perpendicular distance
// from the shaft axis FALLS at cos(gamma) per unit while the along-shaft
// coordinate RISES at sin(gamma). That one fact places H, C and N; reading it as
// a rise is the sign error §2 records against the N seed.
func profileHexagon(l lattice, g member) hexagon {
	rc, rs := math.Cos(g.gamma), math.Sin(g.gamma)
	heelStation := l.pitchCone*rc + g.baseHeight
	toM := l.apexToDedendum - l.rootLength
	mPt := pt2{toM * math.Cos(g.gammaRoot), toM * math.Sin(g.gammaRoot)}
	slide := (mPt.Y - g.toeRadius) / rc
	nPt := pt2{mPt.X + slide*rs, g.toeRadius}
	return hexagon{
		aPrime: pt2{nPt.X, 0},
		g:      pt2{heelStation, 0},
		h:      pt2{heelStation, g.radius - g.baseHeight/math.Tan(g.gamma)},
		c:      pt2{l.pitchCone*rc + l.dedendum*rs, g.radius - l.dedendum*rc},
		m:      mPt,
		n:      nPt,
	}
}

// polygonArea is twice the signed area of a closed polygon, halved: positive for
// a counter-clockwise walk.
func polygonArea(pts []pt2) float64 {
	total := 0.0
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		total += a.cross(b)
	}
	return total / 2
}

// pappusVolume is the volume the polygon sweeps in a full turn about y = 0:
// 2*pi*ybar*Area, written as the one sum so the centroid is never divided out
// and multiplied back in.
func pappusVolume(pts []pt2) float64 {
	total := 0.0
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		total += (a.Y + b.Y) * a.cross(b)
	}
	return math.Pi * total / 3
}

// frustumArea is the lateral area the segment a->b sweeps in a full turn about
// y = 0: pi*(y_a + y_b)*slant.
func frustumArea(a, b pt2) float64 {
	return math.Pi * (a.Y + b.Y) * b.sub(a).length()
}

// coneHalfAngle is the angle between the shaft axis and the wall the segment
// a->b sweeps — the half-angle a decad.Cone publishes for that face.
func coneHalfAngle(a, b pt2) float64 {
	d := b.sub(a)
	return math.Atan2(math.Abs(d.Y), math.Abs(d.X))
}

// ---------------------------------------------------------------- anchor frame

// latticePoints is the whole §2 figure in the ANCHOR frame, at the closed-form
// positions §2 seeds each point at. The names are §2's own, and the map is what
// the [BEVEL-F-SEED-HELD] assertion compares the solved sketch against.
//
// The frame: the projected centre c is the origin, the projected anchor line runs
// along +u, and perp = (0, 1) is the grow side. Point I lands on c, which is the
// closure "Constrain Point I with center point" states.
func latticePoints(l lattice) map[string]pt2 {
	gp, gg := l.pinion.gamma, l.driving.gamma
	R := l.pitchCone

	apex := pt2{0, R*math.Cos(gg) + l.driving.baseHeight}
	drivingDir := pt2{0, -1}
	pinionDir := pt2{math.Sin(l.sigma), -math.Cos(l.sigma)}
	pitchDir := pt2{math.Sin(gg), -math.Cos(gg)}
	// The two dedendum directions, chosen by dot product against the shaft axes:
	// those dots are exactly sin(gamma_p) and sin(gamma_g), both strictly
	// positive for every configuration the range checks admit.
	dedPinion := pt2{math.Cos(gg), math.Sin(gg)}
	dedDriving := dedPinion.mul(-1)

	b := apex.add(drivingDir.mul(R * math.Cos(gg)))
	a := apex.add(pinionDir.mul(R * math.Cos(gp)))
	apex2 := apex.add(pitchDir.mul(R))
	c := apex2.add(dedPinion.mul(l.dedendum))
	d := apex2.add(dedDriving.mul(l.dedendum))

	pts := map[string]pt2{
		"Apex":  apex,
		"B":     b,
		"A":     a,
		"Apex2": apex2,
		"C":     c,
		"D":     d,
		"E":     a.add(pinionDir.mul(l.dedendum * math.Sin(gp))),
		"F":     b.add(drivingDir.mul(l.dedendum * math.Sin(gg))),
		"G":     a.add(pinionDir.mul(l.pinion.baseHeight)),
		"H":     apex2.add(dedPinion.mul(l.pinion.baseHeight / math.Sin(gp))),
		"I":     b.add(drivingDir.mul(l.driving.baseHeight)),
		"J":     apex2.add(dedDriving.mul(l.driving.baseHeight / math.Sin(gg))),
		"K":     apex2.add(dedPinion.mul(l.pinion.virtualPitchRadius)),
		"L":     apex2.add(dedDriving.mul(l.driving.virtualPitchRadius)),
	}
	if l.toothSpacing > 0 {
		pts["K'"] = apex2.add(dedPinion.mul(l.pinion.virtualPitchRadius + l.toothSpacing))
		pts["L'"] = apex2.add(dedDriving.mul(l.driving.virtualPitchRadius + l.toothSpacing))
	}
	// The toe lattice comes from each gear's own profile frame, lifted back here
	// through that gear's axis. Stating it once, in the frame where the falling
	// perpendicular distance is a coordinate, is what keeps the slide's sign right.
	lift := func(axisDir, dedDir pt2, x hexagon) (m, n, foot pt2) {
		radial := radialDirOf(axisDir, dedDir)
		at := func(v pt2) pt2 { return apex.add(axisDir.mul(v.X)).add(radial.mul(v.Y)) }
		return at(x.m), at(x.n), at(x.aPrime)
	}
	pts["M"], pts["N"], pts["A'"] = lift(pinionDir, dedPinion, profileHexagon(l, l.pinion))
	pts["O"], pts["P"], pts["B'"] = lift(drivingDir, dedDriving, profileHexagon(l, l.driving))
	return pts
}

// radialDirOf is the unit vector a gear's profile-frame y runs along: perpendicular
// to its shaft axis, on the side its dedendum corner sits. The dedendum direction
// decomposes as sin(gamma)*axisDir - cos(gamma)*radialDir, so the radial direction
// is what is left of the axis component after the dedendum direction is removed.
func radialDirOf(axisDir, dedDir pt2) pt2 {
	return axisDir.mul(dedDir.dot(axisDir)).sub(dedDir).unit()
}

// latticeOrder is the order §2 creates the points in, which is the order
// [BEVEL-F-SEED-HELD] compares them in so the message names the EARLIEST site
// that flipped rather than a downstream symptom. At Tooth Spacing 0 the two
// primed points are not built at all and the list is 20 rather than 22.
func latticeOrder(l lattice) []string {
	out := []string{"Apex", "B", "A", "Apex2", "C", "D", "E", "F", "G", "H", "I", "J", "K"}
	if l.toothSpacing > 0 {
		out = append(out, "K'")
	}
	out = append(out, "M", "N", "A'", "L")
	if l.toothSpacing > 0 {
		out = append(out, "L'")
	}
	return append(out, "O", "P", "B'")
}

// ---------------------------------------------------------------- §3 tooth

// toothCircles is the four circles the virtual-spur proxy is asked for, in mm.
// The virtual tooth number is a REAL number and is never rounded: the Tredgold
// construction puts the equivalent spur gear's pitch radius exactly at the
// back-cone distance r/cos(gamma), and a rounded count rebuilds every circle
// from the rounded value and shortens the working addendum.
type toothCircles struct {
	pitch, base, tip, root float64
	embedded               bool // the flank starts inside the root circle
}

func newToothCircles(l lattice, g member) toothCircles {
	rv := g.virtualPitchRadius
	c := toothCircles{
		pitch: rv,
		base:  rv * math.Cos(toothPressureAngle),
		tip:   rv + addendumFactor*l.module,
		root:  rv - dedendumFactor*l.module - l.rootSink,
	}
	c.embedded = c.base < c.root
	return c
}

// toothCentre is where the tooth is centred in the PROFILE frame: K/L at Tooth
// Spacing 0, and K'/L' above it. K/L is where the back-cone dedendum line crosses
// the shaft axis, so its radius is 0 and its station is R/cos(gamma); K'/L' is
// Tooth Spacing further along that same line, which carries it PAST the axis, so
// both halves of the placement matter. Seating the tooth plane's origin on the
// axis instead leaves every tooth point Tooth Spacing*cos(gamma) too far out.
func toothCentre(l lattice, g member) pt2 {
	return pt2{
		X: l.pitchCone/math.Cos(g.gamma) + l.toothSpacing*math.Sin(g.gamma),
		Y: -l.toothSpacing * math.Cos(g.gamma),
	}
}

// apexToToothPlane is the apex's PERPENDICULAR distance to the back-cone tooth
// plane. The plane is tilted out of the axis-perpendicular by gamma, so it is the
// tooth centre's station times cos(gamma) and not the station itself. At Tooth
// Spacing 0 it is exactly the Pitch Cone Distance R.
func apexToToothPlane(l lattice, g member) float64 {
	return toothCentre(l, g).X * math.Cos(g.gamma)
}

// ---------------------------------------------------------------- §3a spiral

// spiral is the tangent-plane construction of §3a steps A and B, in the flat 2-D
// crown frame: x is the cone distance along the ROOT cone element coneVec, y is
// the circumferential direction v = axisDir x coneVec.
type spiral struct {
	rToe, rHeel, rMean, span float64
	cutterRadius             float64 // r_c, the Cutter Radius or R_mean when it is 0
	handSign                 float64 // +1 Right, -1 Left, NEGATED for the pinion
	centre                   pt2     // (Cx, Cy)
	toe2d, heel2d            pt2
	phiCrown                 float64 // the developed azimuth the arc subtends at the apex
	total                    float64 // |phi_crown| / sin(gamma) — the shaft-axis twist
}

// newSpiral builds the cutter-arc geometry for one gear.
//
// R_toe and R_heel are the cone distances of the toe and heel EDGE MIDPOINTS
// measured along coneVec, which is what step A reads them as — not the root
// element's own endpoints. The hand sign goes on the cos/Cy term and never on the
// sin/Cx term: opposite hand mirrors the cutter centre across the cone element
// (y = 0), and putting the sign on Cx mirrors about x = R_mean instead, a
// different curve that gives the two gears unequal twist.
func newSpiral(l lattice, g member, p map[string]float64) spiral {
	x := profileHexagon(l, g)
	// distAlong of a profile-frame point, measured along the ROOT cone element:
	// the element makes gammaRoot with the shaft axis, so the projection is
	// x*cos(gammaRoot) + y*sin(gammaRoot).
	along := func(v pt2) float64 {
		return v.X*math.Cos(g.gammaRoot) + v.Y*math.Sin(g.gammaRoot)
	}
	toeMid := x.m.add(x.n).mul(0.5)
	heelMid := x.c.add(x.h).mul(0.5)

	s := spiral{rToe: along(toeMid), rHeel: along(heelMid)}
	s.rMean = (s.rToe + s.rHeel) / 2
	s.span = s.rHeel - s.rToe
	s.cutterRadius = p[keyCutterRadius]
	if s.cutterRadius <= 0 {
		s.cutterRadius = s.rMean
	}
	s.handSign = p[keyHand]
	if g.label == "Pinion" {
		s.handSign = -s.handSign
	}
	psi := radians(p[keySpiralAngle]) // the shared key holds degrees
	s.centre = pt2{
		X: s.rMean - s.cutterRadius*math.Sin(psi),
		Y: s.handSign * s.cutterRadius * math.Cos(psi),
	}
	s.toe2d = circleIntersectNearest(s.rToe-traceOvershoot*s.span, s.centre, s.cutterRadius,
		pt2{s.rMean, 0})
	s.heel2d = circleIntersectNearest(s.rHeel+traceOvershoot*s.span, s.centre, s.cutterRadius,
		pt2{s.rMean, 0})
	s.phiCrown = math.Atan2(s.heel2d.Y, s.heel2d.X) - math.Atan2(s.toe2d.Y, s.toe2d.X)
	// The divisor is the PITCH cone's roll ratio while phiCrown is measured on the
	// ROOT cone frame. That split is deliberate: the crown gear the generation law
	// rolls against is tangent to the pitch cone. acos(coneVec . axisDir) is the
	// ROOT angle and inflates the twist by sin(gamma)/sin(gammaRoot).
	s.total = math.Abs(s.phiCrown) / math.Sin(g.gamma)
	return s
}

// circleIntersectNearest is the framework helper's own rule: intersect the apex
// circle of radius r with the cutter circle and keep the solution nearest the
// reference point, which is the branch the mean point sits on. A non-overlap
// clamps to tangency.
func circleIntersectNearest(r float64, centre pt2, rc float64, ref pt2) pt2 {
	d := centre.length()
	if d == 0 {
		return pt2{r, 0}
	}
	a := (r*r - rc*rc + d*d) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	base := centre.unit().mul(a)
	perp := pt2{-centre.Y, centre.X}.unit().mul(h)
	one, two := base.add(perp), base.sub(perp)
	if one.sub(ref).length() <= two.sub(ref).length() {
		return one
	}
	return two
}

// slabOffsets is §3a step E's fixed family: eight planes parallel to the parent
// transverse tooth plane, the first span/6 inside the heel and the last two past
// the toe. They are NOT perpendicular to the cone element — the parent plane
// carries the back-cone tooth-centre line, so its normal runs along the PITCH
// element while coneVec is the ROOT element, and the two differ by the dedendum
// angle.
func slabOffsets(span float64) []float64 {
	out := make([]float64, spiralSlabs)
	for k := range out {
		out[k] = float64(k+1) * span / 6
	}
	return out
}

// segmentTwist is the rotation §3a step G gives one slab, keyed on the cone
// distance of its HEEL FACE and centred on R_mean so the mid-face section stays
// unrotated. Keying on the centroid instead leaves the loft's mid-face section
// rotated by half a segment.
func segmentTwist(s spiral, heelFace float64) float64 {
	return -s.handSign * s.total * (s.rMean - heelFace) / s.span
}

// crownFactor is §3a step H's relief: monotonic in the heel-distance fraction u,
// full at the heel and growing toward the toe. Keying it on |ang| instead is
// symmetric about mid-face and notches the slab just inside the held-full heel.
func crownFactor(s spiral, heelFace float64) float64 {
	u := (s.rHeel - heelFace) / s.span
	return 1 - crownPerRad*(math.Abs(s.total)/2)*u
}
