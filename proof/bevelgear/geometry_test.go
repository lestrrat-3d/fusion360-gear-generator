// Package bevelgear_test proves the bevel pair's build, one function per step of
// spec/bevelgear/steps.md.
//
// This file holds no step. It holds the closed form every step is measured
// against: the two pitch cone angles, the Pitch Cone Distance, the resolved base
// heights, face width, root length and toe radii, and the twenty-two named §2
// points the lattice has to solve onto. Each step seeds its geometry from this
// figure and then asserts the built result against it, so the number a step
// checks is never the number the step itself produced.
//
// WHAT THE CLOSED FORM CANNOT REACH, recorded here because it applies to every
// step below rather than to one of them.
//
// The figure is computed from the spec's own formulas, and the proof seeds its
// sketches at those positions. So a lattice case proves that the constraint net
// solves FROM A CORRECT SEED, and never that the seed the generated module
// computes is correct. §2's own seeding paragraphs say the same thing, and the
// toe-line seeding in sketches_test.go repeats it at the site where a wrong seed
// has actually reached Fusion. Nothing in this package runs lib/geargen/
// bevelgear.py, so the module's seeds, its range-check refusals and its
// [BEVEL-F-SEED-HELD] gate reach Fusion untested by this stage.
//
// Lengths here are millimetres and angles radians, which is the frame the sketch
// engine and decad both read. The generated module works in Fusion internal cm;
// the conversion is the module's, and a factor dropped there is invisible to
// this proof.
package bevelgear_test

import (
	"math"
	"testing"
)

// pt is a point or a direction in the Gear Profiles sketch's own 2-D frame.
type bevPt struct{ X, Y float64 }

func (a bevPt) add(b bevPt) bevPt     { return bevPt{a.X + b.X, a.Y + b.Y} }
func (a bevPt) sub(b bevPt) bevPt     { return bevPt{a.X - b.X, a.Y - b.Y} }
func (a bevPt) mul(s float64) bevPt   { return bevPt{a.X * s, a.Y * s} }
func (a bevPt) dot(b bevPt) float64   { return a.X*b.X + a.Y*b.Y }
func (a bevPt) cross(b bevPt) float64 { return a.X*b.Y - a.Y*b.X }
func (a bevPt) norm() float64         { return math.Hypot(a.X, a.Y) }

func (a bevPt) unit() bevPt {
	n := a.norm()
	return bevPt{a.X / n, a.Y / n}
}

// signedAngleDeg is the counter-clockwise angle from d1 to d2, in degrees, which
// is what sketch.NewAngle takes. The engine's angle dimension is signed where
// Fusion's addAngularDimension is a magnitude plus a text point, so every place
// the proof writes an angle it is writing the SIGNED form of what Fusion pins
// with an unsigned dimension and a seed ([BEVEL-F-MIRROR-FIGURE]).
func bevSignedAngleDeg(d1, d2 bevPt) float64 {
	return math.Atan2(d1.cross(d2), d1.dot(d2)) * 180 / math.Pi
}

// signedOffset is the signed perpendicular distance of target from the infinite
// line through src0 in the direction src0->src1, positive on the left. It is
// what sketch.NewOffset takes, and it is computed from the closed-form positions
// rather than read back off the solve.
func bevSignedOffset(src0, src1, target bevPt) float64 {
	d := src1.sub(src0).unit()
	return d.cross(target.sub(src0))
}

// side is one gear of the pair: its own inputs, its own cone angles, and its own
// eleven named §2 points. The pinion is built first and the driving gear second,
// exactly as the spec orders them.
type bevSide struct {
	label         string
	teeth         float64
	gamma         float64 // pitch cone angle, radians
	gammaRoot     float64 // root cone angle, radians
	pitchDiameter float64
	pitchRadius   float64

	minTeeth   float64 // 5.27 * cos gamma
	minBase    float64
	maxBase    float64
	baseHeight float64 // resolved

	toeRadius        float64 // resolved
	toeRadiusCeiling float64
	toeLimit         float64

	heelBoreRadius float64 // r_heel
	toeBoreRadius  float64 // r_toe
	maxBore        float64
	boreDiameter   float64 // resolved, already bounded

	virtualPitchRadius float64
	virtualTeeth       float64
	embedded           bool

	shaftDir bevPt // unit Apex->A  / Apex->B
	dedDir   bevPt // unit Apex2->C / Apex2->D

	shaftEnd  bevPt // A  / B
	ded       bevPt // C  / D
	ext       bevPt // E  / F
	axisHeel  bevPt // G  / I
	dedHeel   bevPt // H  / J
	center    bevPt // K  / L
	centerOff bevPt // K' / L'
	toeRoot   bevPt // M  / O
	toeInner  bevPt // N  / P
	axisToe   bevPt // A' / B'
}

// hex is the six profile vertices in the draw order the Profile sketch uses:
// A'->G->H->C->M->N for the pinion, B'->I->J->D->O->P for the driving gear.
func (s *bevSide) hex() []bevPt {
	return []bevPt{s.axisToe, s.axisHeel, s.dedHeel, s.ded, s.toeRoot, s.toeInner}
}

// station is a point's distance from the apex measured ALONG this gear's shaft
// axis, and radius is its perpendicular distance from that axis. The pair is the
// axial section every solid step is built in.
func (s *bevSide) station(apex, p bevPt) float64 { return p.sub(apex).dot(s.shaftDir) }

func (s *bevSide) radius(apex, p bevPt) float64 {
	return math.Abs(p.sub(apex).cross(s.shaftDir))
}

// figure is one case's whole closed form.
type bevFigure struct {
	module       float64
	shaftAngle   float64 // radians
	coneDistance float64 // the DIAGONAL of the two pitch diameters, never R
	pitchCone    float64 // R, the Pitch Cone Distance
	dedendum     float64 // 1.25 * module
	apexToDed    float64 // |Apex->Ded| = sqrt(R^2 + dedendum^2)
	rootSink     float64 // 0.05 * 2.25 * module

	maxShaftAngle float64 // radians, the cone-angle limit capped at 150 degrees
	maxFaceWidth  float64
	faceWidth     float64
	rootLength0   float64 // the Toe Extension 0 root length
	rootLength    float64 // resolved at this case's Toe Extension
	toeExtension  float64
	toothSpacing  float64

	toeExtensionRejected bool

	spiralAngle  float64 // radians
	hand         float64 // +1 Right, -1 Left, as the DRIVING gear's hand
	cutterRadius float64 // 0 means auto

	center   bevPt // the projected centre point c
	apex     bevPt
	apex2    bevPt
	perp     bevPt // the in-plane unit perpendicular to the anchor line, grow side
	anchor   bevPt // the anchor line's unit direction
	pitchDir bevPt // unit Apex->Apex2

	pinion  bevSide
	driving bevSide
}

// gearOf returns the side a case's "gear" parameter names: 0 is the pinion, 1 is
// the driving gear. Every per-gear step reads it, since §3 and the whole of
// "Create the Gear Bodies" run once per gear.
func (f *bevFigure) gearOf(p map[string]float64) *bevSide {
	if p["gear"] != 0 {
		return &f.driving
	}
	return &f.pinion
}

// newFigure resolves one case, in the order the spec resolves it: the cone
// angles first, then the base heights, then the §2 lattice, then the Maximum
// Face Width off that lattice, then the toe lattice, and the Maximum Bore
// Diameter last because its toe term needs the Root Length.
func newBevFigure(t testing.TB, p map[string]float64) *bevFigure {
	t.Helper()

	f := &bevFigure{
		module:       p["module"],
		shaftAngle:   p["shaftAngle"] * math.Pi / 180,
		toothSpacing: p["toothSpacing"],
		toeExtension: p["toeExtension"],
		spiralAngle:  p["spiralAngle"] * math.Pi / 180,
		hand:         1,
		cutterRadius: p["cutterRadius"],
	}
	if p["hand"] < 0 {
		f.hand = -1
	}
	f.dedendum = 1.25 * f.module
	f.rootSink = 0.05 * 2.25 * f.module

	f.pinion.label = "Pinion"
	f.pinion.teeth = p["pinionTeeth"]
	f.driving.label = "Driving"
	f.driving.teeth = p["drivingTeeth"]
	ppd := f.module * f.pinion.teeth
	dpd := f.module * f.driving.teeth
	f.pinion.pitchDiameter, f.driving.pitchDiameter = ppd, dpd
	f.pinion.pitchRadius, f.driving.pitchRadius = ppd/2, dpd/2
	f.coneDistance = math.Hypot(ppd, dpd)

	// The Maximum Shaft Angle: acos of minus the smaller pitch diameter over the
	// larger, capped at 150 degrees. The cone-angle half is where a pitch cone
	// angle would reach 90 degrees and R*cos(gamma) change sign.
	small, large := math.Min(ppd, dpd), math.Max(ppd, dpd)
	f.maxShaftAngle = math.Min(math.Acos(-small/large), 150*math.Pi/180)

	f.pinion.gamma = math.Atan2(math.Sin(f.shaftAngle)*ppd, dpd+ppd*math.Cos(f.shaftAngle))
	f.driving.gamma = f.shaftAngle - f.pinion.gamma
	f.pitchCone = (ppd / 2) / math.Sin(f.pinion.gamma)
	f.apexToDed = math.Hypot(f.pitchCone, f.dedendum)
	dedendumAngle := math.Atan2(f.dedendum, f.pitchCone)
	f.pinion.gammaRoot = f.pinion.gamma - dedendumAngle
	f.driving.gammaRoot = f.driving.gamma - dedendumAngle

	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		s.minTeeth = 5.27 * math.Cos(s.gamma)
		s.minBase = 1.05 * f.dedendum * math.Sin(s.gamma)
		s.maxBase = 0.95 * (s.pitchRadius - f.dedendum*math.Cos(s.gamma)) * math.Tan(s.gamma)
	}

	// Base heights. The driving gear resolves first because the pinion's fallback
	// is the driving gear's RESOLVED value scaled by the tooth ratio, never the
	// raw driving input.
	f.driving.baseHeight = bevClampBase(t, &f.driving,
		bevPick(p["drivingBaseHeight"], f.module*f.driving.teeth/8))
	f.pinion.baseHeight = bevClampBase(t, &f.pinion,
		bevPick(p["pinionBaseHeight"], f.driving.baseHeight*f.pinion.teeth/f.driving.teeth))

	f.buildFrame(p)
	f.buildLattice()

	// The Maximum Face Width is the perpendicular distance from A to the pinion
	// dedendum line and from B to the driving one, whichever is smaller, times
	// 0.95. Both reduce to R*sin^2(gamma) on the solved figure, which is the form
	// asserted against the solve in stepGearProfiles.
	f.maxFaceWidth = 0.95 * f.pitchCone * math.Min(
		bevSq(math.Sin(f.pinion.gamma)), bevSq(math.Sin(f.driving.gamma)))
	f.faceWidth = bevPick(p["faceWidth"], f.coneDistance/6)
	if f.faceWidth > f.maxFaceWidth {
		if p["faceWidth"] == 0 {
			f.faceWidth = f.maxFaceWidth
		} else {
			t.Fatalf("case asks for Face Width %.6g mm above the Maximum Face Width %.6g mm, which "+
				"the generated module rejects rather than building", f.faceWidth, f.maxFaceWidth)
		}
	}

	f.buildToe(t, p)
	f.buildBore(t, p)
	f.buildVirtualTeeth()
	return f
}

func bevSq(x float64) float64 { return x * x }

// pick returns the user's value when it is non-zero and the fallback otherwise.
// Zero is the dialog's "unspecified" for every length input bevel declares.
func bevPick(user, fallback float64) float64 {
	if user != 0 {
		return user
	}
	return fallback
}

// clampBase applies one gear's own Minimum and Maximum Base Height. A fallback
// below the minimum is raised and one above the maximum is capped; a case that
// asks for a user value outside either end is a case the generated module
// refuses, so the proof refuses to build it rather than pretending it would.
func bevClampBase(t testing.TB, s *bevSide, value float64) float64 {
	t.Helper()
	if s.minBase > s.maxBase {
		t.Fatalf("%s: Minimum Base Height %.6g mm exceeds Maximum Base Height %.6g mm, so no base "+
			"height builds this gear; the Minimum Teeth floor of %.4g is what refuses it first",
			s.label, s.minBase, s.maxBase, s.minTeeth)
	}
	if value < s.minBase {
		return s.minBase
	}
	if value > s.maxBase {
		return s.maxBase
	}
	return value
}

// buildFrame places the projected centre, the grow direction and the apex in the
// sketch's own 2-D coordinates ([BEVEL-F-APEX-LOCAL]). The anchor line runs
// along local +X and the grow side is +Y; in the module the sign comes from the
// target plane's normal ([BEVEL-F-GROW-SIDE]), which is a one-bit direction the
// proof has no plane to read.
func (f *bevFigure) buildFrame(p map[string]float64) {
	f.center = bevPt{p["centerX"], p["centerY"]}
	f.anchor = bevPt{1, 0}
	f.perp = bevPt{-f.anchor.Y, f.anchor.X}
	f.apex = f.center.add(f.perp.mul(f.pitchCone*math.Cos(f.driving.gamma) + f.driving.baseHeight))
}

// buildLattice places every §2 point that does not depend on the resolved Face
// Width, in the order §2 creates them.
func (f *bevFigure) buildLattice() {
	drivingDir := f.perp.mul(-1)
	// The pinion shaft direction is the driving one rotated about the apex by the
	// Shaft Angle, in the sense whose endpoint has the greater X. With the anchor
	// line along +X that is the +Shaft Angle sense for every angle the range
	// admits, but the proof forms both and compares, since comparing is the rule.
	plus := bevRotate(drivingDir, f.shaftAngle)
	minus := bevRotate(drivingDir, -f.shaftAngle)
	pinionDir := plus
	if minus.mul(f.pitchCone*math.Cos(f.pinion.gamma)).X >
		plus.mul(f.pitchCone*math.Cos(f.pinion.gamma)).X {
		pinionDir = minus
	}
	f.pinion.shaftDir = pinionDir
	f.driving.shaftDir = drivingDir

	f.pinion.shaftEnd = f.apex.add(pinionDir.mul(f.pitchCone * math.Cos(f.pinion.gamma)))
	f.driving.shaftEnd = f.apex.add(drivingDir.mul(f.pitchCone * math.Cos(f.driving.gamma)))

	// The pitch line sits at gamma_g from the driving shaft axis and gamma_p from
	// the pinion's, so the two perpendicular drops of PPD/2 and DPD/2 close on one
	// point at the Pitch Cone Distance.
	f.pitchDir = bevRotate(drivingDir, f.driving.gamma*bevSign(drivingDir.cross(pinionDir)))
	f.apex2 = f.apex.add(f.pitchDir.mul(f.pitchCone))

	// The dedendum directions are picked by dot product against the shaft axes,
	// never by "towards the anchor line": the two dots are sin(gamma_p) and
	// sin(gamma_g), strictly positive for every admitted configuration.
	u := bevPt{-f.pitchDir.Y, f.pitchDir.X}
	if u.dot(pinionDir) < 0 {
		u = u.mul(-1)
	}
	f.pinion.dedDir = u
	f.driving.dedDir = u.mul(-1)

	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		s.ded = f.apex2.add(s.dedDir.mul(f.dedendum))
		s.ext = s.shaftEnd.add(s.shaftDir.mul(f.dedendum * math.Sin(s.gamma)))
		s.axisHeel = s.shaftEnd.add(s.shaftDir.mul(s.baseHeight))
		s.dedHeel = f.apex2.add(s.dedDir.mul(s.baseHeight / math.Sin(s.gamma)))
		s.virtualPitchRadius = s.pitchRadius / math.Cos(s.gamma)
		s.center = f.apex2.add(s.dedDir.mul(s.virtualPitchRadius))
		s.centerOff = f.apex2.add(s.dedDir.mul(s.virtualPitchRadius + f.toothSpacing))
	}
}

func bevSign(x float64) float64 {
	if x < 0 {
		return -1
	}
	return 1
}

func bevRotate(d bevPt, a float64) bevPt {
	s, c := math.Sin(a), math.Cos(a)
	return bevPt{d.X*c - d.Y*s, d.X*s + d.Y*c}
}

// buildToe resolves the Root Length, both Toe Radii and the toe lattice: M, N
// and A' on the pinion, O, P and B' on the driving gear.
func (f *bevFigure) buildToe(t testing.TB, p map[string]float64) {
	t.Helper()
	f.rootLength0 = f.faceWidth * f.apexToDed / f.pitchCone

	user := map[string]float64{"Pinion": p["pinionToeRadius"], "Driving": p["drivingToeRadius"]}
	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		s.toeRadiusCeiling = (s.pitchRadius - f.dedendum*math.Cos(s.gamma)) *
			(1 - f.faceWidth/f.pitchCone)
		s.toeRadius = bevPick(user[s.label], s.pitchRadius-f.faceWidth/math.Sin(s.gamma))
		if s.toeRadius >= s.toeRadiusCeiling {
			t.Fatalf("%s: Toe Radius %.6g mm is at or above the Toe Radius Ceiling %.6g mm, which "+
				"the generated module rejects", s.label, s.toeRadius, s.toeRadiusCeiling)
		}
		s.toeLimit = f.apexToDed - s.toeRadius/math.Sin(s.gammaRoot)
	}

	limit := math.Min(f.pinion.toeLimit, f.driving.toeLimit)
	f.toeExtensionRejected = limit <= f.rootLength0
	f.rootLength = f.rootLength0
	if f.toeExtension > 0 {
		if f.toeExtensionRejected {
			t.Fatalf("the Toe Limit %.6g mm has fallen below the Toe Extension 0 root length %.6g mm, "+
				"so the module rejects a Toe Extension above 0 on this pair", limit, f.rootLength0)
		}
		f.rootLength = f.rootLength0 + (f.toeExtension/100)*0.99*(limit-f.rootLength0)
	}

	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		fraction := 1 - f.rootLength/f.apexToDed
		s.toeRoot = f.apex.add(s.ded.sub(f.apex).mul(fraction))
		// Along dedDir the perpendicular distance from this gear's shaft axis FALLS
		// at cos(gamma) per unit, so the slide gives back exactly the difference
		// between the M seed's own radius and the Toe Radius. Read as a rise it is
		// the wrong sign, and a compile round measured 20 of 21 lattice cases
		// failing to converge on that reading.
		slide := (s.radius(f.apex, s.toeRoot) - s.toeRadius) / math.Cos(s.gamma)
		s.toeInner = s.toeRoot.add(s.dedDir.mul(slide))
		s.axisToe = f.apex.add(s.shaftDir.mul(s.station(f.apex, s.toeInner)))
	}
}

// buildBore resolves the Maximum Bore Diameter and each gear's bore. Its heel
// term is closed form from the base height, its toe term needs the Root Length,
// and the bound is the smaller of the two, which is why the whole of it resolves
// here and not in the input pass.
func (f *bevFigure) buildBore(t testing.TB, p map[string]float64) {
	t.Helper()
	user := map[string]float64{"Pinion": p["pinionBore"], "Driving": p["drivingBore"]}
	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		s.heelBoreRadius = s.pitchRadius - s.baseHeight/math.Tan(s.gamma)
		s.toeBoreRadius = (f.apexToDed - f.rootLength) * math.Sin(s.gammaRoot)
		s.maxBore = 2 * 0.95 * math.Min(s.heelBoreRadius, s.toeBoreRadius)
		if p["boreEnable"] == 0 {
			s.boreDiameter = 0
			continue
		}
		if user[s.label] == 0 {
			s.boreDiameter = math.Min(s.pitchDiameter/4, s.maxBore)
			continue
		}
		if user[s.label] > s.maxBore {
			t.Fatalf("%s: Bore Diameter %.6g mm is above the Maximum Bore Diameter %.6g mm, which "+
				"the generated module rejects rather than deleting the back face",
				s.label, user[s.label], s.maxBore)
		}
		s.boreDiameter = user[s.label]
	}
}

// buildVirtualTeeth computes each gear's back-cone tooth count. It is a REAL
// number and is never rounded: the Tredgold construction puts the equivalent
// spur gear's pitch radius exactly at r/cos(gamma), and a rounded count redraws
// every circle from the rounded radius.
func (f *bevFigure) buildVirtualTeeth() {
	for _, s := range []*bevSide{&f.pinion, &f.driving} {
		s.virtualTeeth = 2 * s.virtualPitchRadius / f.module
		s.embedded = f.baseRadius(s) < f.rootRadius(s)
	}
}

// rootRadius is the virtual tooth's root circle, drawn one root sink INSIDE the
// dedendum corner so the root arc lies inside the gear body's root cone across
// its whole width rather than touching it along the tooth's own centreline.
func (f *bevFigure) rootRadius(s *bevSide) float64 {
	return s.virtualPitchRadius - f.dedendum - f.rootSink
}

func (f *bevFigure) tipRadius(s *bevSide) float64 { return s.virtualPitchRadius + f.module }

func (f *bevFigure) baseRadius(s *bevSide) float64 {
	return s.virtualPitchRadius * math.Cos(20*math.Pi/180)
}

// polygonArea is the shoelace area of a closed polygon.
func bevPolygonArea(ring []bevPt) float64 {
	sum := 0.0
	for i := range ring {
		j := (i + 1) % len(ring)
		sum += ring[i].cross(ring[j])
	}
	return math.Abs(sum) / 2
}

// pappusVolume is the volume the ring sweeps in a full turn about the u axis,
// exactly: 2*pi times the ring's first moment of area about that axis. The ring
// is the axial section, x the station along the shaft axis and y the radius.
func bevPappusVolume(ring []bevPt) float64 {
	moment := 0.0
	for i := range ring {
		j := (i + 1) % len(ring)
		moment += ring[i].cross(ring[j]) * (ring[i].Y + ring[j].Y)
	}
	return 2 * math.Pi * math.Abs(moment) / 6
}

// frustumArea is the lateral area the edge a->b sweeps about the u axis: a cone
// frustum of slant length |b-a| between radii a.Y and b.Y.
func bevFrustumArea(a, b bevPt) float64 { return math.Pi * (a.Y + b.Y) * b.sub(a).norm() }

// axialSection is one gear's frustum profile in (station, radius), in the draw
// order the Profile sketch uses. The first edge, A'->G or B'->I, lies on the
// shaft axis and sweeps nothing; the two radial edges sweep the flat toe and
// heel faces, and the three between them sweep the toe dish, the root cone and
// the heel cone.
func (f *bevFigure) axialSection(s *bevSide) []bevPt {
	out := make([]bevPt, 0, 6)
	for _, p := range s.hex() {
		out = append(out, bevPt{f.stationOf(s, p), s.radius(f.apex, p)})
	}
	return out
}

func (f *bevFigure) stationOf(s *bevSide, p bevPt) float64 { return s.station(f.apex, p) }

// circleIntersectNearest is the 2-D circle-circle intersection the framework's
// solids.circle_intersect_nearest performs: the apex circle of radius r meets
// the cutter circle of centre (cx, cy) and radius rc, and the solution nearest
// the reference point is kept. A non-overlapping pair clamps to tangency.
func bevCircleIntersectNearest(r, cx, cy, rc, refX, refY float64) (float64, float64) {
	d := math.Hypot(cx, cy)
	a := (r*r - rc*rc + d*d) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	bx, by := cx*a/d, cy*a/d
	ox, oy := -cy*h/d, cx*h/d
	p1 := bevPt{bx + ox, by + oy}
	p2 := bevPt{bx - ox, by - oy}
	if math.Hypot(p1.X-refX, p1.Y-refY) <= math.Hypot(p2.X-refX, p2.Y-refY) {
		return p1.X, p1.Y
	}
	return p2.X, p2.Y
}

// spiral is the §3a frame and trace for one gear: the cone distances read at the
// toe and heel edge midpoints, the cutter circle, the two arc endpoints, and the
// shaft-axis twist the conjugate crown-gear law gives.
type bevSpiral struct {
	rToe, rHeel, rMean, span float64
	cutter                   float64
	handSign                 float64
	cx, cy                   float64
	toe2d, heel2d            bevPt
	phiCrown                 float64
	total                    float64
	offsets                  []float64
}

// spiralOf builds the §3a frame for one gear. R_toe and R_heel are read at the
// TOE and HEEL edge midpoints, which are two different edges: M->N and C->H on
// the pinion, O->P and D->J on the driving gear. Passing the two endpoints of
// one edge collapses the span and inverts the whole spiral.
func (f *bevFigure) spiralOf(s *bevSide) *bevSpiral {
	coneVec := s.ded.sub(f.apex).unit()
	distAlong := func(p bevPt) float64 { return p.sub(f.apex).dot(coneVec) }

	toeMid := s.toeRoot.add(s.toeInner).mul(0.5)
	heelMid := s.ded.add(s.dedHeel).mul(0.5)

	sp := &bevSpiral{}
	sp.rToe = distAlong(toeMid)
	sp.rHeel = distAlong(heelMid)
	sp.rMean = (sp.rToe + sp.rHeel) / 2
	sp.span = sp.rHeel - sp.rToe

	sp.cutter = f.cutterRadius
	if sp.cutter == 0 {
		sp.cutter = sp.rMean
	}
	sp.handSign = f.hand
	if s.label == "Pinion" {
		sp.handSign = -sp.handSign
	}
	// The hand sign belongs on the cos term. Opposite hands mirror the cutter
	// centre across the cone element, which flips Cy; putting the sign on Cx
	// mirrors about x = R_mean instead, a different curve that gives the two gears
	// unequal twist.
	sp.cx = sp.rMean - sp.cutter*math.Sin(f.spiralAngle)
	sp.cy = sp.handSign * sp.cutter * math.Cos(f.spiralAngle)

	lo := sp.rToe - 0.06*sp.span
	hi := sp.rHeel + 0.06*sp.span
	tx, ty := bevCircleIntersectNearest(lo, sp.cx, sp.cy, sp.cutter, sp.rMean, 0)
	hx, hy := bevCircleIntersectNearest(hi, sp.cx, sp.cy, sp.cutter, sp.rMean, 0)
	sp.toe2d = bevPt{tx, ty}
	sp.heel2d = bevPt{hx, hy}

	sp.phiCrown = math.Atan2(sp.heel2d.Y, sp.heel2d.X) - math.Atan2(sp.toe2d.Y, sp.toe2d.X)
	// The divisor is the PITCH cone's roll ratio while phiCrown is measured in a
	// frame whose x axis is the ROOT cone element. That split is deliberate: the
	// crown gear the generation law rolls against is tangent to the pitch cone.
	// Written on the root cone the divisor would be sin(gamma_root), which is 1.062
	// times smaller on the default pair and twists the tooth about 6% further.
	sp.total = math.Abs(sp.phiCrown) / math.Sin(s.gamma)

	sp.offsets = make([]float64, 8)
	for k := range sp.offsets {
		sp.offsets[k] = float64(k+1) * sp.span / 6
	}
	return sp
}

// crownPerRad is the class constant _CROWN_PER_RAD. It reaches built geometry
// and nothing derives it: 0.5 was hand-entered in 2026-06-07 on the hand-coded
// generator this spec replaced, and no measurement, published source or Fusion
// load stands behind it since. It is reproduced here so the proof builds today's
// tooth, not because the value is known to be right.
const bevCrownPerRad = 0.5

// crownFactor is the relief one slab takes, keyed on the monotonic heel-distance
// fraction u rather than on the twist magnitude. Keying on |ang| is symmetric
// about mid-face, and with the heel slab held full it makes the slab just inside
// the heel the most relieved one, which notches the taper.
func (sp *bevSpiral) crownFactor(u float64) float64 {
	return 1 - bevCrownPerRad*(math.Abs(sp.total)/2)*u
}
