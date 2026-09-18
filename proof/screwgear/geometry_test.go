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
	MountAngleA float64 // Phi for gear A, its cross-section angle where the axes cross
	MountAngleB float64 // Phi for gear B, which the meshing search does not make equal
	Engagement  float64 // how deep the crests overlap
	ToothCount  int

	CollarOuter float64 // radius of a collar's rim
	CollarDepth float64 // how far a collar runs along its own gear's axis
	CollarAt    float64 // the station each collar sits at on its gear's axis
	Clearance   float64 // added all round a collar's opening
}

func defaultParams() Params {
	return Params{
		Width:       10,
		Thickness:   2.5,
		ToothHeight: 1.2,
		ToothPitch:  1.75,
		TwistLead:   40,
		MountAngleA: 30 * math.Pi / 180,
		MountAngleB: 0,
		Engagement:  0.60,
		ToothCount:  48,

		CollarOuter: 8,
		CollarDepth: 1.5,
		CollarAt:    8.4,
		Clearance:   0.3,
	}
}

// collarCentre is where a gear's collar sits, on that gear's own axis.
func collarCentre(g Gear) r3.Vec { return g.Origin.Add(g.Ez.Scale(g.P.CollarAt)) }

// Lambda is the screw parameter: millimetres of advance per radian of turn.
func (p Params) Lambda() float64 { return p.TwistLead / (2 * math.Pi) }

// Beta is the helix angle of the toothed edge, which sits at radius Width/2.
func (p Params) Beta() float64 { return math.Atan(math.Pi * p.Width / p.TwistLead) }

// Sigma is the angle between the two axes: the crossed-helical rule, twice the
// helix angle because both gears have the same helix angle and the same hand.
func (p Params) Sigma() float64 { return 2 * p.Beta() }

// AxisOffset is the distance between the two axes.
func (p Params) AxisOffset() float64 { return p.Width - p.Engagement }

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
	m := g.edge(s) - u
	if b := u + g.P.Width/2; b < m {
		m = b
	}
	if b := g.P.Thickness/2 - math.Abs(v); b < m {
		m = b
	}
	return m
}

// section is the cross-section at station s, as its four corners in world
// space, wound counter-clockwise about +Ez.
func (g Gear) section(s float64) [4]r3.Vec {
	w, t := g.P.Width/2, g.P.Thickness/2
	e := g.edge(s)
	return [4]r3.Vec{
		g.world(-w, -t, s),
		g.world(e, -t, s),
		g.world(e, t, s),
		g.world(-w, t, s),
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
const assemblyPhase = 0.315

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

	ga, gb := pair(p, p.Sigma(), 0, 0)
	sa, sb := tangencyStation(ga), tangencyStation(gb)
	if cross := ga.crestTangent(sa).Cross(gb.crestTangent(sb)).Len(); cross > 1e-12 {
		t.Errorf("at Sigma = 2*Beta the crest tangents are %.3e off parallel, want 0", cross)
	}

	// And the rule bites: a crossing angle five degrees either side does not.
	for _, off := range []float64{-5, 5} {
		sigma := p.Sigma() + off*math.Pi/180
		ga, gb := pair(p, sigma, 0, 0)
		cross := ga.crestTangent(tangencyStation(ga)).Cross(gb.crestTangent(tangencyStation(gb))).Len()
		if cross < 1e-3 {
			t.Errorf("at Sigma = 2*Beta%+.0f deg the crest tangents are %.3e off parallel, want them apart",
				off, cross)
		}
	}

	// The mounting angle moves that station and nothing else about the rule.
	mounted := defaultParams()
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
	const sections = 9
	p := defaultParams()

	dtheta := (p.ToothPitch / p.Lambda()) / (sections - 1)
	departure := p.Width / 2 * (1 - math.Cos(dtheta/2))

	if got := dtheta * 180 / math.Pi; got > 3 {
		t.Errorf("nine sections put %.3f deg between neighbours, which is more than the loft's "+
			"vertex pairing is worth trusting", got)
	}
	t.Logf("nine sections put %.3f deg between neighbours and fall %.6f mm short of the helicoid",
		dtheta*180/math.Pi, departure)
	if departure > 1e-3 {
		t.Errorf("the ruled surface falls %.6f mm short of the helicoid, want under 0.001", departure)
	}
	if departure > measuredBacklash/100 {
		t.Errorf("the shortfall %.6f mm is not small against the %.3f mm backlash",
			departure, measuredBacklash)
	}
}
