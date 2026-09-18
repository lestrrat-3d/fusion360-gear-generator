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
// helicoid. The part Fusion builds lofts nine rectangles per tooth, and
// TestLoftSectionCountHoldsTheHelicoid bounds the difference.
package screwgear_test

import (
	"math"
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

	BossHalf  float64 // half the smooth boss's length along the ribbon
	BossTaper float64 // how far the boss takes to run out into the teeth
	BossGrow  float64 // how far the boss stands proud of the plain ribbon

	CageRadius float64 // where each bore's centre sits, measured from the cage axis
	CageRise   float64 // half the cage's height, to the outer face of a plate
	ShellThick float64 // how thick the frame's wall is, everywhere
	PlateThick float64 // how tall an end plate's band is
	PostWidth  float64 // how wide a post is round the cage
	BlockWall  float64 // material left round a bore
	Clearance  float64 // added all round a bore
}

func defaultParams() Params {
	return Params{
		Width:       10,
		Thickness:   2.5,
		ToothHeight: 1.2,
		ToothPitch:  1.75,
		TwistLead:   30,
		CrossAngle:  80 * math.Pi / 180,
		MountAngleA: 15 * math.Pi / 180,
		MountAngleB: 15 * math.Pi / 180,
		Engagement:  0.36,
		ToothCount:  40,

		BossHalf:  5,
		BossTaper: 0.9,
		BossGrow:  0.6,

		CageRadius: 15,
		CageRise:   18,
		ShellThick: 3,
		PlateThick: 2,
		PostWidth:  3,
		BlockWall:  3.0,
		Clearance:  0.3,
	}
}

// BoreHalfWidth and BoreHalfThickness are the bore's opening, which is the
// ribbon's boss plus a clearance. The teeth never enter a bore: the boss is
// what passes through, which is why nothing in the cage is cut to the shape of
// a tooth.
func (p Params) BoreHalfWidth() float64 { return p.Width/2 + p.BossGrow + p.Clearance }
func (p Params) BoreHalfThickness() float64 {
	return p.Thickness/2 + p.BossGrow + p.Clearance
}

// CageOuter and CageInner are the frame's one outer surface and the depth
// everything reaches in to.
//
// Every outside face of the frame lies on ONE cylinder. The plates, the posts
// and the blocks are all pieces of the same wall, differing only in how far
// round and how far up they run, so nothing stands proud of anything else and
// the whole outside is a single turned surface. A block reaching outward, or a
// square post on a round plate, leaves corners sticking out of that surface.
func (p Params) CageOuter() float64 { return p.CageRadius + p.ShellThick/2 }
func (p Params) CageInner() float64 { return p.CageOuter() - p.ShellThick }

// boreStations are where a gear's two bores sit on its own axis: the two places
// it crosses the cage.
func boreStations(p Params) [2]float64 { return [2]float64{-p.CageRadius, p.CageRadius} }

// Lambda is the screw parameter: millimetres of advance per radian of turn.
func (p Params) Lambda() float64 { return p.TwistLead / (2 * math.Pi) }

// Beta is the helix angle of the toothed edge, which sits at radius Width/2.
func (p Params) Beta() float64 { return math.Atan(math.Pi * p.Width / p.TwistLead) }

// Sigma is the angle between the two axes.
//
// It is an input, not a derivation. The crossed-helical rule makes 2*Beta the
// angle at which the two crest helices run parallel, and that is where the
// search starts, but the arrangement also has to carry a printable frame: the
// bores' angle is set by the mounting angles and the cage radius together, and
// at 2*Beta nothing drives with the mounting angles close enough to put the
// bores near upright. TestCrossedHelicalRuleMakesTheCrestHelicesParallel still
// holds the rule; this is the angle the pair is actually built at.
func (p Params) Sigma() float64 { return p.CrossAngle }

// AxisOffset is the distance between the two axes.
func (p Params) AxisOffset() float64 { return p.Width - p.Engagement }

// LoftSections is how many cross-sections a tooth cell is lofted from.
//
// It is derived rather than pinned, because the twist per tooth is what decides
// it: the loft's ruled surface cuts the corner of the helicoid by
// (Width/2)*(1 - cos(step/2)), and the step is the twist per tooth divided by
// one less than the count. A faster twist needs more sections for the same
// departure, and this gear's twist is fast.
func (p Params) LoftSections() int {
	const maxStep = 2 * math.Pi / 180
	steps := int(math.Ceil((p.ToothPitch / p.Lambda()) / maxStep))
	if steps < 8 {
		steps = 8
	}
	return steps + 1
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

// boss is how far the ribbon stands proud of its plain section at station s.
//
// The ribbon carries a smooth swelling at each of the two places it passes
// through the cage, and that swelling is the only part of it the frame ever
// touches. It is what lets a bore be a plain hole: the teeth never pass through
// anything, and nothing bears on a crest.
//
// The boss is flat-topped over its middle and runs out into the teeth over
// BossTaper at each end. It travels with the gear, so its length is the stroke.
func (g Gear) boss(s float64) float64 {
	p := g.P
	best := 0.0
	for _, centre := range boreStations(p) {
		off := math.Abs(s-g.Phase-centre) - (p.BossHalf - p.BossTaper)
		switch {
		case off <= 0:
			best = math.Max(best, p.BossGrow)
		case off < p.BossTaper:
			best = math.Max(best, p.BossGrow*0.5*(1+math.Cos(math.Pi*off/p.BossTaper)))
		}
	}
	return best
}

// profile is the ribbon's cross-section at station s: how far it reaches on the
// toothed side, on the back side, and either side of its own mid plane.
//
// The boss both swells the section and fades the teeth out of it, so the two
// meet with no step.
func (g Gear) profile(s float64) (uHi, uLo, vHalf float64) {
	p := g.P
	b := g.boss(s)
	tooth := 1.0
	if p.BossGrow > 0 {
		tooth = 1 - b/p.BossGrow
	}
	cut := p.ToothHeight / 2 * (1 - math.Cos(2*math.Pi*(s-g.Phase)/p.ToothPitch))
	return p.Width/2 + b - tooth*cut, -p.Width/2 - b, p.Thickness/2 + b
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

// The spec pins nine sections per tooth. This is the arithmetic that count is
// bought with: the loft's ruled surface cuts the corner of the true helicoid,
// and the spec's claim is that the shortfall is three orders below the backlash.
func TestLoftSectionCountHoldsTheHelicoid(t *testing.T) {
	p := defaultParams()
	sections := p.LoftSections()

	dtheta := (p.ToothPitch / p.Lambda()) / float64(sections-1)
	departure := p.Width / 2 * (1 - math.Cos(dtheta/2))

	if got := dtheta * 180 / math.Pi; got > 2.01 {
		t.Errorf("%d sections put %.3f deg between neighbours, which is more than the count is "+
			"derived to allow", sections, got)
	}
	t.Logf("%d sections per tooth put %.3f deg between neighbours and fall %.6f mm short of the "+
		"helicoid", sections, dtheta*180/math.Pi, departure)
	if departure > 1e-3 {
		t.Errorf("the ruled surface falls %.6f mm short of the helicoid, want under 0.001", departure)
	}
	if departure > measuredBacklash/100 {
		t.Errorf("the shortfall %.6f mm is not small against the %.3f mm backlash",
			departure, measuredBacklash)
	}
}
