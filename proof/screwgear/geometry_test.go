// Package screwgear_test proves the geometry of the screw/screw gearing that
// spec/screwgear describes: two racks twisted into helices, each moving by a
// screw motion in a cage, driving each other 1:1.
//
// The model here is IMPLICIT rather than a solid. A ribbon is the set of points
// whose cross-section coordinates satisfy four inequalities, which makes "is
// this point inside the other gear" one line of arithmetic. That is what the
// mesh proof needs a few million times, and it is exact where a boolean between
// two lofted solids would be a tangency decad's exact predicates refuse to
// classify. Nothing here goes through decad or sketch for that reason.
//
// The part this proves is the ideal ribbon: an exact cosine edge on an exact
// helicoid. The part Fusion builds lofts eleven rectangles per tooth at the
// defaults, and TestLoftSectionCountHoldsTheHelicoid bounds the difference.
package screwgear_test

import (
	"math"
	"math/bits"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Params is the whole mechanism, in millimetres and radians. The field names
// are the spec's dialog labels; defaultParams is the spec's default table.
type Params struct {
	Width       float64 // W, across the plate
	Thickness   float64 // T, through the plate
	ToothHeight float64 // H, crest to root
	ToothPitch  float64 // P, along the axis
	TwistLead   float64 // length of one full turn
	CrossAngle  float64 // Sigma, the angle between the two axes
	MountAngleA float64 // Phi for gear A, its cross-section angle where the axes cross
	MountAngleB float64 // Phi for gear B, which the meshing search does not make equal
	Engagement  float64 // how deep the crests overlap
	ToothCount  int

	CageRadius  float64 // where each ribbon crosses the frame, on its own axis from the middle
	RingRadius  float64 // the ring's radius to the centre of its wire, and where the rods stand
	CageRise    float64 // half the frame's height, to the centre of the ring's wire and of the loop's bars
	RingWire    float64 // diameter of the ring's wire and of the loop's bars
	RodDiameter float64 // diameter of the four rods
	CollarHalf  float64 // half a collar's length along its ribbon
	CollarWall  float64 // material a collar leaves round its bore
	Clearance   float64 // added all round a bore
}

func defaultParams() Params {
	return Params{
		Width:       10,
		Thickness:   2.5,
		ToothHeight: 1.2,
		ToothPitch:  1.75,
		TwistLead:   33,
		CrossAngle:  80 * math.Pi / 180,
		MountAngleA: 15 * math.Pi / 180,
		MountAngleB: 15 * math.Pi / 180,
		Engagement:  0.36,
		ToothCount:  80,

		CageRadius:  10,
		RingRadius:  11.25,
		CageRise:    13.5,
		RingWire:    2.5,
		RodDiameter: 2,
		CollarHalf:  2,
		CollarWall:  2,
		Clearance:   0.3,
	}
}

// BoreHalfWidth and BoreHalfThickness are the bore's opening: the rectangle
// the ribbon's crests and faces lie on, plus a clearance all round. The crest
// of a cosine rack is the ribbon's outer edge, u = Width/2, so that rectangle
// holds the whole ribbon, teeth included, and nothing in the cage has to be cut
// to the shape of a tooth. The crests are what bear on the bore's toothed side.
func (p Params) BoreHalfWidth() float64     { return p.Width/2 + p.Clearance }
func (p Params) BoreHalfThickness() float64 { return p.Thickness/2 + p.Clearance }

// RingOuter is the ring's outer radius, and FrameHeight is the frame's whole
// height from the bottom of the loop's bars to the top of the ring's wire. They
// are what the video's frame is measured by: the ring's outer diameter is the
// unit its other proportions were read in.
func (p Params) RingOuter() float64   { return p.RingRadius + p.RingWire/2 }
func (p Params) FrameHeight() float64 { return 2*p.CageRise + p.RingWire }

// WireRadius and RodRadius are the round sections the frame is built from.
func (p Params) WireRadius() float64 { return p.RingWire / 2 }
func (p Params) RodRadius() float64  { return p.RodDiameter / 2 }

// Travel is how far the mechanism runs, end to end. Nothing on the ribbon
// limits it: the whole ribbon is the same twisted rack, so any stretch of it
// fits a collar. What limits it is the ribbon's own length: a gear has to keep
// both its collars full, and its end reaches the far face of a collar once it
// has advanced Length/2 - CageRadius - CollarHalf from the middle. The engaged
// zone is nearer the middle than the collars, so the teeth are still meshing
// there. TestTravelIsTheRibbonBetweenItsCollars walks both limits.
func (p Params) Travel() float64 { return p.Length() - 2*(p.CageRadius+p.CollarHalf) }

// boreStations are where a gear's two collars sit on its own axis, measured
// from the ribbon's own middle: the two places it crosses the frame.
func boreStations(p Params) [2]float64 { return [2]float64{-p.CageRadius, p.CageRadius} }

// Lambda is the screw parameter: millimetres of advance per radian of turn.
func (p Params) Lambda() float64 { return p.TwistLead / (2 * math.Pi) }

// Beta is the helix angle of the toothed edge, which sits at radius Width/2.
func (p Params) Beta() float64 { return math.Atan(math.Pi * p.Width / p.TwistLead) }

// Sigma is the angle between the two axes.
//
// It is an input, not a derivation. The crossed-helical rule makes 2*Beta the
// angle at which the two crest helices run parallel, and that is where the
// search starts, but at 2*Beta (87 degrees) the pair departs from the 1:1 line
// by more than TestPairDrivesOneToOne allows: 0.187 mm at 90 degrees against
// its 0.10 mm bound, where 80 degrees departs by 0.067 mm.
// TestCrossedHelicalRuleMakesTheCrestHelicesParallel still holds the rule;
// this is the angle the pair is actually built at.
func (p Params) Sigma() float64 { return p.CrossAngle }

// AxisOffset is the distance between the two axes.
func (p Params) AxisOffset() float64 { return p.Width - p.Engagement }

// LoftSections is how many cross-sections a tooth cell is lofted from: one
// more than the number of steps between them, and the steps are the larger of
// two counts.
//
// The twist decides one: a surface ruled straight between two sections cuts
// the corner of the helicoid by (Width/2)*(1 - cos(step/2)), and the step is
// the twist per tooth divided by the number of steps, so the count keeps that
// twist under two degrees. A faster twist needs more sections for the same
// departure, and this gear's twist is fast.
//
// The tooth decides the other: between two sections a straight chord of the
// cosine falls (ToothHeight/2)*(1 - cos(pi/steps)) short of it at worst, and
// that depends on the step count alone. Eight steps hold the chord under four
// percent of the tooth height whatever the twist, and that is the floor;
// without it a slow twist would loft a tooth from two or three sections and
// lose the tooth.
//
// Both are bounds on a RULED loft through the sections. The cell is one Fusion
// loft through all of them, which is smooth between sections rather than ruled
// (spec/screwgear/fusion.md [SCREW-F-CELL-LOFT]), so the count fixes how closely
// the exact sections are spaced and the ruled figures are the one bound this
// package has; the built surface's own departure between sections is not
// measured here. TestLoftSectionCountHoldsTheHelicoid says the same.
func (p Params) LoftSections() int {
	const maxStep = 2 * math.Pi / 180
	steps := int(math.Ceil((p.ToothPitch / p.Lambda()) / maxStep))
	if steps < minCellSteps {
		steps = minCellSteps
	}
	return steps + 1
}

const minCellSteps = 8

// EdgeChord is how far the lofted toothed edge falls short of the cosine at
// worst, between two neighbouring sections.
func (p Params) EdgeChord() float64 {
	return p.ToothHeight / 2 * (1 - math.Cos(math.Pi/float64(p.LoftSections()-1)))
}

// Length is the finished ribbon's length.
func (p Params) Length() float64 { return float64(p.ToothCount) * p.ToothPitch }

// Gear is a ribbon placed in the world. Ez is its axis and Ex points at the
// mating gear, so MountAngle means the same thing for both members and the two
// gears are the same part rather than mirror images.
type Gear struct {
	P          Params
	Origin     r3.Vec
	Ex, Ey, Ez r3.Vec
	Hand       float64 // +1 or -1, multiplying Lambda
	Mount      float64 // this gear's own cross-section angle where the axes cross
	Phase      float64 // the tooth phase, and the only thing the motion moves
}

func (g Gear) lambda() float64 { return g.Hand * g.P.Lambda() }

// angle is the cross-section's rotation about the axis at station s.
func (g Gear) angle(s float64) float64 { return s/g.lambda() + g.Mount }

// edge is the toothed edge's u coordinate at station s: a pure cosine, crest at
// Width/2 and root at Width/2 - ToothHeight.
func (g Gear) edge(s float64) float64 {
	h := g.P.ToothHeight
	return g.P.Width/2 - h/2 + h/2*math.Cos(2*math.Pi*(s-g.Phase)/g.P.ToothPitch)
}

// envelope is everything the ribbon's cross-section can reach at any tooth
// phase: the crest rectangle. It is the same at every station, because some
// phase of the travel puts a crest at every station, and it is what the frame
// has to clear, since a gear is somewhere in its travel whenever it is in the
// frame at all.
func (g Gear) envelope() (uHi, uLo, vHalf float64) {
	p := g.P
	return p.Width / 2, -p.Width / 2, p.Thickness / 2
}

// profile is the ribbon's cross-section at station s: how far it reaches on the
// toothed side, on the back side, and either side of its own mid plane. The
// ribbon is the same twisted rack along its whole length, so only the toothed
// side varies, and it does so with the tooth phase alone.
func (g Gear) profile(s float64) (uHi, uLo, vHalf float64) {
	p := g.P
	return g.edge(s), -p.Width / 2, p.Thickness / 2
}

// span is the stretch of the axis the ribbon occupies at its current tooth
// phase. Advancing a gear translates it along its axis by the same amount it
// shifts the phase, so the ends move with the phase while the twisted blank
// between them does not.
func (g Gear) span() (float64, float64) {
	half := g.P.Length() / 2
	return g.Phase - half, g.Phase + half
}

// envelopeMargin is margin taken against everything the ribbon reaches at any
// tooth phase rather than against one phase of it.
func (g Gear) envelopeMargin(pt r3.Vec) float64 {
	u, v, _ := g.local(pt)
	uHi, uLo, vHalf := g.envelope()
	m := uHi - u
	if b := u - uLo; b < m {
		m = b
	}
	if b := vHalf - math.Abs(v); b < m {
		m = b
	}
	return m
}

// local maps a world point to (u, v, s) with the twist undone.
func (g Gear) local(pt r3.Vec) (u, v, s float64) {
	d := pt.Sub(g.Origin)
	x, y := d.Dot(g.Ex), d.Dot(g.Ey)
	s = d.Dot(g.Ez)
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	return x*c + y*sn, -x*sn + y*c, s
}

// world is local's inverse.
func (g Gear) world(u, v, s float64) r3.Vec {
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	x, y := u*c-v*sn, u*sn+v*c
	return g.Origin.Add(g.Ex.Scale(x)).Add(g.Ey.Scale(y)).Add(g.Ez.Scale(s))
}

// margin is positive inside the body, and is the smallest slack to the toothed
// edge, the back edge or either face. It is not a true distance — the slack to
// the toothed edge is measured along u rather than along the surface normal —
// but its SIGN is exact, and only the sign decides contact.
//
// The body is unbounded along its axis here. Callers sample only the axial
// window that can reach the other gear, which is far shorter than the ribbon.
func (g Gear) margin(pt r3.Vec) float64 {
	u, v, s := g.local(pt)
	uHi, uLo, vHalf := g.profile(s)
	m := uHi - u
	if b := u - uLo; b < m {
		m = b
	}
	if b := vHalf - math.Abs(v); b < m {
		m = b
	}
	return m
}

// section is the cross-section at station s, as its four corners in world
// space, wound counter-clockwise about +Ez.
func (g Gear) section(s float64) [4]r3.Vec {
	uHi, uLo, t := g.profile(s)
	return [4]r3.Vec{
		g.world(uLo, -t, s),
		g.world(uHi, -t, s),
		g.world(uHi, t, s),
		g.world(uLo, t, s),
	}
}

// crestTangent is the direction of the helix the tooth crests lie on, at
// station s. The crest sits at radius Width/2, so the helix is the axis point
// plus (Width/2) times the turning u direction, and its derivative is the axis
// direction plus (Width/2)/lambda times the v direction.
func (g Gear) crestTangent(s float64) r3.Vec {
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	vHat := g.Ex.Scale(-sn).Add(g.Ey.Scale(c))
	tangent, ok := g.Ez.Add(vHat.Scale(g.P.Width / 2 / g.lambda())).Normalize()
	if !ok {
		panic("the crest tangent collapsed, which no finite lead allows")
	}
	return tangent
}

// pair places both gears for a given crossing angle. The common perpendicular
// is world Z, the two axes sit half the offset above and below the origin, and
// they open by sigma about Z from world X.
func pair(p Params, sigma, phaseA, phaseB float64) (Gear, Gear) {
	a := p.AxisOffset()
	half := sigma / 2
	az := r3.NewVec(math.Cos(half), math.Sin(half), 0)
	bz := r3.NewVec(math.Cos(half), -math.Sin(half), 0)
	ax := r3.NewVec(0, 0, 1)  // gear A looks up at gear B
	bx := r3.NewVec(0, 0, -1) // gear B looks down at gear A

	ga := Gear{P: p, Origin: r3.NewVec(0, 0, -a/2), Ex: ax, Ez: az, Hand: 1,
		Mount: p.MountAngleA, Phase: phaseA}
	ga.Ey = ga.Ez.Cross(ga.Ex)
	gb := Gear{P: p, Origin: r3.NewVec(0, 0, a/2), Ex: bx, Ez: bz, Hand: 1,
		Mount: p.MountAngleB, Phase: phaseB}
	gb.Ey = gb.Ez.Cross(gb.Ex)
	return ga, gb
}

// assemblyPhase is the tooth phase gear B is built at, with gear A at zero. It
// is not half a pitch: the two gears are mounted at different cross-section
// angles, so the phase that puts a crest against a root is its own number.
// TestAssemblyPhaseSitsInTheFreeWindow holds it to the middle of the play.
const assemblyPhase = -0.90

// defaultPair is the arrangement the spec's default table describes.
func defaultPair() (Gear, Gear) {
	p := defaultParams()
	return pair(p, p.Sigma(), 0, assemblyPhase)
}

func TestToothProfileIsACosineOfTheStatedHeight(t *testing.T) {
	g, _ := defaultPair()
	p := g.P

	if got, want := g.edge(0), p.Width/2; math.Abs(got-want) > 1e-12 {
		t.Errorf("crest at the tooth phase is %.6f mm from the axis, want %.6f", got, want)
	}
	if got, want := g.edge(p.ToothPitch/2), p.Width/2-p.ToothHeight; math.Abs(got-want) > 1e-12 {
		t.Errorf("root half a pitch on is %.6f mm from the axis, want %.6f", got, want)
	}
	for _, s := range []float64{0, 0.7, 1.9, 3.2} {
		if got, want := g.edge(s+p.ToothPitch), g.edge(s); math.Abs(got-want) > 1e-12 {
			t.Errorf("edge at s=%.2f repeats as %.6f one pitch on, want %.6f", s, got, want)
		}
	}
	// Nothing on the edge ever passes the crest or falls below the root.
	for i := range 400 {
		s := float64(i) * p.ToothPitch / 400
		if e := g.edge(s); e > p.Width/2+1e-12 || e < p.Width/2-p.ToothHeight-1e-12 {
			t.Fatalf("edge at s=%.4f is %.6f, outside [%.6f, %.6f]",
				s, e, p.Width/2-p.ToothHeight, p.Width/2)
		}
	}
}

// tangencyStation is the station at which a gear's cross-section angle is zero,
// so its toothed edge points straight at the mating gear. With a mounting angle
// of Phi that station is Phi*Lambda back from the axes' closest approach, and
// with no mounting angle it is the closest approach itself.
func tangencyStation(g Gear) float64 { return -g.Mount * g.lambda() }

// The crossed-helical rule is what fixes the crossing angle, and the spec
// derives it rather than quoting a search: the two crest helices run parallel
// exactly when the shaft angle is twice the helix angle.
//
// They run parallel at ONE station on each ribbon — the one whose cross-section
// angle is zero — and not along the whole engagement. Everywhere else the two
// crest helices cross, which is why this pair carries a point contact like a
// crossed-helical pair rather than the line contact a spur pair has, and why
// the mounting angle does not spoil the rule: it moves that station along the
// ribbon instead of destroying it.
func TestCrossedHelicalRuleMakesTheCrestHelicesParallel(t *testing.T) {
	p := defaultParams()
	p.MountAngleA, p.MountAngleB = 0, 0
	// The rule is about 2*Beta, which is not the angle this pair is built at.
	p.CrossAngle = 2 * p.Beta()

	ga, gb := pair(p, p.Sigma(), 0, 0)
	sa, sb := tangencyStation(ga), tangencyStation(gb)
	if cross := ga.crestTangent(sa).Cross(gb.crestTangent(sb)).Len(); cross > 1e-12 {
		t.Errorf("at Sigma = 2*Beta the crest tangents are %.3e off parallel, want 0", cross)
	}

	// And the rule bites: a crossing angle five degrees either side does not.
	for _, off := range []float64{-5, 5} {
		sigma := 2*p.Beta() + off*math.Pi/180
		ga, gb := pair(p, sigma, 0, 0)
		cross := ga.crestTangent(tangencyStation(ga)).Cross(gb.crestTangent(tangencyStation(gb))).Len()
		if cross < 1e-3 {
			t.Errorf("at Sigma = 2*Beta%+.0f deg the crest tangents are %.3e off parallel, want them apart",
				off, cross)
		}
	}

	// The mounting angle moves that station and nothing else about the rule.
	mounted := defaultParams()
	mounted.CrossAngle = 2 * mounted.Beta()
	mounted.MountAngleA = 15 * math.Pi / 180
	mounted.MountAngleB = 15 * math.Pi / 180
	ma, mb := pair(mounted, mounted.Sigma(), 0, 0)
	if cross := ma.crestTangent(tangencyStation(ma)).Cross(mb.crestTangent(tangencyStation(mb))).Len(); cross > 1e-12 {
		t.Errorf("with a mounting angle the crest tangents are %.3e off parallel at their own station", cross)
	}
	if got, want := tangencyStation(ma), -mounted.MountAngleA*mounted.Lambda(); math.Abs(got-want) > 1e-12 {
		t.Errorf("the parallel station sits at %.4f mm, want %.4f", got, want)
	}

	// Beta is the angle the crest helix makes with the axis, which is what
	// "helix angle" means and what the rule is stated in.
	if got := math.Acos(ga.crestTangent(sa).Dot(ga.Ez)); math.Abs(got-p.Beta()) > 1e-12 {
		t.Errorf("the crest helix stands %.6f rad off the axis, want Beta = %.6f", got, p.Beta())
	}
}

// The whole build rests on this: the ribbon is invariant under the screw step,
// so it is one tooth cell repeated, and the gear's own motion is nothing but a
// shift of the tooth phase.
func TestRibbonIsInvariantUnderItsScrewStep(t *testing.T) {
	g, _ := defaultPair()
	p := g.P

	rot, err := r3.RotationAround(g.Origin, g.Ez, units.Radians(p.ToothPitch/g.lambda()))
	if err != nil {
		t.Fatalf("rotation: %v", err)
	}
	move, err := r3.Translation(g.Ez.Scale(p.ToothPitch))
	if err != nil {
		t.Fatalf("translation: %v", err)
	}
	step, err := rot.Then(move)
	if err != nil {
		t.Fatalf("compose the screw step: %v", err)
	}

	worst := 0.0
	for i := range 240 {
		s := float64(i) * p.ToothPitch / 24
		for _, c := range [][2]float64{{-p.Width / 2, -p.Thickness / 2}, {0, 0}, {2, 1}} {
			here := g.world(c[0], c[1], s)
			next := g.world(c[0], c[1], s+p.ToothPitch)
			if d := step.Apply(here).Sub(next).Len(); d > worst {
				worst = d
			}
		}
	}
	if worst > 1e-9 {
		t.Errorf("a point carried by one screw step misses its own image by %.3e mm, want 0", worst)
	}

	// The blank is invariant, and so is the body cut from it: the cross-section
	// one pitch on is the same rectangle, teeth included, so the four corners of
	// a section carried by the step land on the corners of the next cell's
	// section. This is the claim the build rests on when it copies one cell and
	// screw-moves the copy: every placement is exact because the body is the
	// same at every cell, not just the twist.
	worst = 0
	for i := range 240 {
		s := float64(i) * p.ToothPitch / 24
		uHi, uLo, vHalf := g.profile(s)
		nHi, nLo, nHalf := g.profile(s + p.ToothPitch)
		if d := math.Max(math.Abs(uHi-nHi), math.Max(math.Abs(uLo-nLo), math.Abs(vHalf-nHalf))); d > 1e-12 {
			t.Fatalf("the cross-section at s=%.3f is (%.6f, %.6f, %.6f) and one pitch on it is "+
				"(%.6f, %.6f, %.6f): the body is not one cell repeated", s, uHi, uLo, vHalf, nHi, nLo, nHalf)
		}
		here, next := g.section(s), g.section(s+p.ToothPitch)
		for k := range 4 {
			if d := step.Apply(here[k]).Sub(next[k]).Len(); d > worst {
				worst = d
			}
		}
	}
	if worst > 1e-9 {
		t.Errorf("a section corner carried by one screw step misses the next cell's corner by %.3e mm, want 0", worst)
	}

	// The same statement seen from the motion: advancing the gear by one pitch
	// leaves the body where it was, with the tooth phase back in step.
	moved := g
	moved.Phase = g.Phase + p.ToothPitch
	for i := range 100 {
		s := float64(i) * p.ToothPitch / 10
		if got, want := moved.edge(s), g.edge(s); math.Abs(got-want) > 1e-12 {
			t.Fatalf("one pitch of advance moves the edge at s=%.3f from %.6f to %.6f", s, want, got)
		}
	}
}

// The spec derives the section count from the twist per tooth with a floor of
// eight steps, eleven sections at the defaults. This is the arithmetic that
// count is bought with, and it is arithmetic about a RULED loft through those
// sections: a ruled surface cuts the corner of the true helicoid, and the
// spec's claim is that the shortfall is three orders below the backlash; a
// ruled loft's toothed edge is a chord of the cosine between sections, and the
// spec's claim is that the chord stays under four percent of the tooth height
// at any twist and under a tenth of the backlash at the defaults. The floor is
// what holds the chord where the twist is slow, so the leads swept here reach
// well past the one the count stops growing at.
//
// Fusion builds the cell as one loft through all the sections, which is smooth
// between them rather than ruled, and LoftFeatureInput has no ruled option. So
// these are bounds on a body Fusion does not build: the built cell passes
// through the same sections, and how far its surface departs from the helicoid
// between them, on either side, is measured by nothing in this package. This is
// the honest edge of what the section count proves, and a Fusion load is what
// sees the built surface.
func TestLoftSectionCountHoldsTheHelicoid(t *testing.T) {
	p := defaultParams()
	sections := p.LoftSections()

	dtheta := (p.ToothPitch / p.Lambda()) / float64(sections-1)
	departure := p.Width / 2 * (1 - math.Cos(dtheta/2))

	if got := dtheta * 180 / math.Pi; got > 2.01 {
		t.Errorf("%d sections put %.3f deg between neighbours, which is more than the count is "+
			"derived to allow", sections, got)
	}
	t.Logf("%d sections per tooth put %.3f deg between neighbours; a ruled loft through them would "+
		"fall %.6f mm short of the helicoid at the crest and its toothed edge's chord %.4f mm short of "+
		"the cosine", sections, dtheta*180/math.Pi, departure, p.EdgeChord())
	if departure > 1e-3 {
		t.Errorf("the ruled surface falls %.6f mm short of the helicoid, want under 0.001", departure)
	}
	if departure > measuredBacklash/100 {
		t.Errorf("the shortfall %.6f mm is not small against the %.3f mm backlash",
			departure, measuredBacklash)
	}
	if chord := p.EdgeChord(); chord > measuredBacklash/10 {
		t.Errorf("the edge chord falls %.4f mm short of the cosine, which is not small against the "+
			"%.3f mm backlash", chord, measuredBacklash)
	}

	for _, lead := range []float64{20, 33, 66, 200, 400} {
		q := defaultParams()
		q.TwistLead = lead
		n := q.LoftSections()
		if n < minCellSteps+1 {
			t.Errorf("at a %.0f mm lead the cell lofts through %d sections, under the floor of %d",
				lead, n, minCellSteps+1)
		}
		if step := (q.ToothPitch / q.Lambda()) / float64(n-1) * 180 / math.Pi; step > 2.01 {
			t.Errorf("at a %.0f mm lead %d sections put %.3f deg between neighbours", lead, n, step)
		}
		if chord := q.EdgeChord(); chord > 0.04*q.ToothHeight {
			t.Errorf("at a %.0f mm lead the edge chord falls %.4f mm short of the cosine, over four "+
				"percent of the %.2f mm tooth", lead, chord, q.ToothHeight)
		}
		t.Logf("at a %.0f mm lead the cell lofts through %d sections and the edge chord is %.4f mm",
			lead, n, q.EdgeChord())
	}
}

// doublingRounds is the schedule the build repeats the cell by, and returns
// the copy-move-join rounds it takes and the tooth ranges every piece lands
// on. The body doubles while it can; whenever the tooth count has a set bit
// below its top one, an unmoved copy of the body at that size is put aside,
// and after the last doubling the asides are moved into place, largest first,
// each by the screw step of the teeth built so far. Every move is by a whole
// number of teeth already built, so every join meets at a shared cross-section
// and nothing overlaps.
func doublingRounds(n int) (rounds int, pieces [][2]int) {
	m := 1
	pieces = [][2]int{{0, 1}}
	var asides []int
	for bit := 0; 1<<(bit+1) <= n; bit++ {
		if n&(1<<bit) != 0 {
			asides = append(asides, m)
		}
		pieces = append(pieces, [2]int{m, 2 * m})
		m *= 2
		rounds++
	}
	for i := len(asides) - 1; i >= 0; i-- {
		pieces = append(pieces, [2]int{m, m + asides[i]})
		m += asides[i]
		rounds++
	}
	if m != n {
		panic("the doubling schedule does not reach the tooth count")
	}
	return rounds, pieces
}

// The build repeats the cell by doubling, and the spec quotes the round count
// that costs. This runs the schedule over every tooth count the dialog can
// reasonably take and holds three things: the pieces tile the ribbon exactly,
// no two overlap, and the count is floor(log2 N) + popcount(N) - 1, which is
// seven at the default eighty teeth.
func TestDoublingScheduleCoversTheRibbon(t *testing.T) {
	for n := 4; n <= 512; n++ {
		rounds, pieces := doublingRounds(n)
		covered := make([]int, n)
		for _, piece := range pieces {
			for tooth := piece[0]; tooth < piece[1]; tooth++ {
				covered[tooth]++
			}
		}
		for tooth, times := range covered {
			if times != 1 {
				t.Fatalf("with %d teeth the schedule lands %d pieces on tooth %d", n, times, tooth)
			}
		}
		want := bits.Len(uint(n)) - 1 + bits.OnesCount(uint(n)) - 1
		if rounds != want {
			t.Errorf("with %d teeth the schedule takes %d rounds, want %d", n, rounds, want)
		}
	}
	rounds, _ := doublingRounds(defaultParams().ToothCount)
	t.Logf("the default %d teeth take %d copy-move-join rounds", defaultParams().ToothCount, rounds)
	if rounds != 7 {
		t.Errorf("the spec quotes seven rounds at the defaults, the schedule takes %d", rounds)
	}
}

// The spec follows Segerman's model on some proportions and departs from it on
// others, and its "What the video shows" table says which is which. This holds
// the defaults inside the video's ranges for the ratios the spec follows, and
// logs the ones it departs from, so that a later change to the defaults that
// stops the part looking like the video's is caught here rather than noticed
// in a picture.
//
// The readings are hand readings of 1280x720 frames and carry about +/-20%: the
// teeth on one face-on stretch of an arm (half a turn) at 0:09, 6:12 and 6:14
// give 20-26 per turn; the edge-on stretches at 6:14 give a thickness of 0.2-0.3
// widths; both ends of a ribbon against the ring at 6:10 give a length of about
// 12 widths; the ring's outer diameter against the face-on width of an arm at
// 0:09 and 6:10 gives 2.2-2.8 widths, and the frame's height against the ring's
// width at 5:26 and 5:34 gives about one. Each bound below is the reading's
// edge moved out by that 20%, so the 18.9 teeth per turn of the defaults, just
// under the reading, pass. Nothing here is finer than that, and a finer reading
// needs the model, not the video.
func TestProportionsFollowTheVideo(t *testing.T) {
	p := defaultParams()
	ga, gb := defaultPair()

	if got := p.TwistLead / p.ToothPitch; got < 16 || got > 31 {
		t.Errorf("the ribbon carries %.1f teeth per turn; the video's carries 20-26", got)
	}
	if got := p.Thickness / p.Width; got < 0.16 || got > 0.36 {
		t.Errorf("the ribbon is %.2f widths thick; the video's reads 0.2-0.3", got)
	}
	if got := p.Length() / p.Width; got < 9.6 || got > 14.4 {
		t.Errorf("the ribbon is %.1f widths long; the video's reads about 12", got)
	}
	if got := 2 * p.RingOuter() / p.Width; got < 1.76 || got > 3.36 {
		t.Errorf("the ring is %.2f widths across; the video's reads 2.2-2.8", got)
	}
	if got := p.FrameHeight() / (2 * p.RingOuter()); got < 0.64 || got > 1.44 {
		t.Errorf("the frame is %.2f ring widths tall; the video's reads about one", got)
	}
	if ga.Hand != gb.Hand {
		t.Errorf("the two gears are of opposite hand; the video's twist the same way")
	}
	if ga.Hand != 1 {
		t.Errorf("the gears are left-handed; the video's read as right-handed")
	}
	t.Logf("ring %.2f widths and %.2f leads across, frame %.2f ring widths tall",
		2*p.RingOuter()/p.Width, 2*p.RingOuter()/p.TwistLead, p.FrameHeight()/(2*p.RingOuter()))

	// What the spec departs from, for the record of a run.
	t.Logf("tooth depth %.2f widths and %.2f pitches, against the video's 0.15-0.2 widths and about one pitch",
		p.ToothHeight/p.Width, p.ToothHeight/p.ToothPitch)
	t.Logf("crossing angle %.0f degrees, against the video's 85-100", p.Sigma()*180/math.Pi)
}
