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
// The part this proves is the ideal ribbon: an exact leaned cosine edge on an
// exact helicoid. The part Fusion builds lofts a four-tooth cell through 41
// rotated sections at the defaults, each a rectangle whose toothed side is a
// fitted spline through eleven points of the edge across the thickness, and
// repeats it by a screw step. TestLoftSectionCountHoldsTheHelicoid bounds what a
// ruled loft through those sections would lose and TestToothSplineHoldsTheEdge
// what a spline through those eleven points would; a Fusion measurement on
// 2026-09-28 (spec/screwgear/fusion.md [SCREW-F-CELL-LOFT]) put the built smooth
// surface within 0.04 mm of the helicoid between sections, at the straight
// tooth of that day.
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

	// ToothSlant leans the tooth ridges across the thickness: the cosine's
	// phase moves by tan(ToothSlant) per millimetre of v, so a ridge runs from
	// (v, s) to (v + dv, s - tan(ToothSlant)*dv) in the section's own (u, v, s)
	// chart. Zero is the straight ridge every print before 2026-10-03 carried.
	ToothSlant float64
	// ToothBow lowers the toothed edge by ToothBow*v^2, in mm per mm^2, so a
	// ridge that leans along the axis runs straight in the world instead of
	// bowing with the twist (TestToothRidgesRunStraight).
	ToothBow float64

	CageRadius float64 // where each ribbon crosses the frame, on its own axis from the middle
	CageRise   float64 // half the sleeve's height, to its flat end faces
	CollarHalf float64 // half the sleeve's wall, which each bore runs through
	CollarWall float64 // the least material the sleeve leaves round a bore or window
	Clearance  float64 // added all round a bore
	// RoofAllowance is added to the clearance on one face of one bore of each
	// gear: the upper long face of the bore whose channel lies level in the
	// wall, which the printer bridges (sleeve_test.go, levelBore).
	RoofAllowance float64
}

// defaultParams is the spec's default table: the ribbons, and the sleeve that
// holds them.
//
// CageRise is half the sleeve's height, to its flat end faces. The video
// frame's 20.25 mm, measured to the centre of its ring's wire, would cost 3 mm
// of print height for 1.5 mm more end wall and nothing else; 18.75 mm, 1.25
// widths, leaves the 4.94 mm end wall TestSleeveIsOnePiece measures, against
// the 16.81 mm the channels need for a CollarWall of end wall and the 18.03 mm
// the build's closed form asks for.
//
// Clearance, Engagement and the two mounting angles changed on 2026-10-02,
// from 0.45 mm, 0.75 mm and 15 degrees: the printed pair did not mesh over the
// play the 0.45 mm bores allowed (bore_play_test.go,
// spec/screwgear/fusion.md [SCREW-F-PRINT-MESH]). The ribbon did not change.
//
// RoofAllowance came in on 2026-10-02 with the second sleeve's print: the two
// -R bores, whose roofs the printer bridges 15.4 mm across, came out too tight
// to pass the ribbons, and the +R bores, at the same 0.20 mm, did not
// (spec/screwgear/fusion.md [SCREW-F-PRINT-2]). The later marked sleeve stopped
// both -R ribbons at their mouths, so the next trial increases the default to
// 0.60 mm, giving 0.80 mm of drawn room under each bridged roof.
//
// ToothSlant and ToothBow came in on 2026-10-03, with the mounting angles
// moved from 14 degrees to 0 and the engagement from 0.90 mm to 1.05 mm: the
// third sleeve's bores fitted and its teeth still slipped, because the
// straight ridges of the two ribbons crossed at 74 to 80 degrees where they
// touched and met at a point (contact_test.go, spec/screwgear/fusion.md
// [SCREW-F-PRINT-3]). thirdPrintParams is the table that print was made at.
func defaultParams() Params {
	return Params{
		Width:       15,
		Thickness:   3.75,
		ToothHeight: 2.625,
		ToothPitch:  2.625,
		TwistLead:   49.5,
		CrossAngle:  80 * math.Pi / 180,
		MountAngleA: 0,
		MountAngleB: 0,
		Engagement:  1.15,
		ToothCount:  68,
		ToothSlant:  25.8 * math.Pi / 180,
		ToothBow:    0.048,

		CageRadius: 15.5,
		CageRise:   18.75,
		CollarHalf: 3,
		CollarWall: 3,
		Clearance:  0.20,

		RoofAllowance: 0.60,
	}
}

// thirdPrintParams is the table the third sleeve and the ribbons in it were
// printed at, the defaults until 2026-10-03: the straight-ridge tooth, 14
// degrees on both mounting angles, a 0.90 mm engagement, and the 0.30 mm roof
// allowance used for that print, with gear B built at thirdPrintPhase.
func thirdPrintParams() Params {
	p := defaultParams()
	p.ToothSlant, p.ToothBow = 0, 0
	p.MountAngleA = 14 * math.Pi / 180
	p.MountAngleB = 14 * math.Pi / 180
	p.Engagement = 0.90
	p.RoofAllowance = 0.30
	return p
}

// thirdPrintPhase is the assembly phase of thirdPrintParams.
const thirdPrintPhase = -1.30

// sleeveParams is defaultParams under the name the compiled step proof calls.
func sleeveParams() Params { return defaultParams() }

// BoreHalfWidth and BoreHalfThickness are the bore's opening: the rectangle
// the ribbon's crests and faces lie on, plus a clearance all round. The crest
// of a cosine rack is the ribbon's outer edge, u = Width/2, so that rectangle
// holds the whole ribbon, teeth included, and nothing in the sleeve has to be
// cut to the shape of a tooth. The crests are what bear on the bore's toothed
// side.
func (p Params) BoreHalfWidth() float64     { return p.Width/2 + p.Clearance }
func (p Params) BoreHalfThickness() float64 { return p.Thickness/2 + p.Clearance }

// Travel is how far the mechanism runs, end to end. Nothing on the ribbon
// limits it: the whole ribbon is the same twisted rack, so any stretch of it
// fits a bore. What limits it is the ribbon's own length: a gear has to keep
// both its bores full, and on the bore's centre line the sleeve's wall runs
// CollarHalf either side of CageRadius, so the gear's end reaches a bore's far
// face once it has advanced Length/2 - CageRadius - CollarHalf from the
// middle. The engaged zone is nearer the middle than the bores, so the teeth
// are still meshing there. TestTravelIsTheRibbonBetweenItsBores walks both
// limits.
func (p Params) Travel() float64 { return p.Length() - 2*(p.CageRadius+p.CollarHalf) }

// boreStations are where a gear's two bores sit on its own axis, measured
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
// search starts. The search chose 80 degrees at the arrangement before
// 2026-10-02, where 90 degrees departed from the 1:1 line by 7.4% of the pitch,
// past TestPairDrivesOneToOne's bound of 6%, and 80 degrees by 2.4%. The
// leaned tooth of 2026-10-03 was fitted to 80 degrees, and there it departs by
// 0.018 mm, 0.7%; with nothing else moved, 2*Beta (87.2 degrees) departs by
// 0.037 mm, 1.4%, and 90 degrees by 0.048 mm, 1.8%.
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
// package has. The built surface's own departure between sections is not
// measured here; Fusion measured it on 2026-09-28 at 0.04 mm or less at the
// defaults. TestLoftSectionCountHoldsTheHelicoid says the same.
func (p Params) LoftSections() int {
	const maxStep = 2 * math.Pi / 180
	steps := int(math.Ceil((p.ToothPitch / p.Lambda()) / maxStep))
	if steps < minCellSteps {
		steps = minCellSteps
	}
	return steps + 1
}

const minCellSteps = 8

// cellTeeth is how many teeth the build's lofted cell holds, the spec's
// CELL_TEETH: the ribbon is that cell repeated by the screw step (spec §3). Four
// is what a Fusion measurement of one-, four- and eight-tooth cells settled on
// 2026-09-28, and one is the fallback the spec states counts for beside it.
const cellTeeth = 4

// CellSections is how many sections a cell of c teeth is lofted through: the
// per-tooth steps of LoftSections, c times over, plus the closing section.
func (p Params) CellSections(c int) int { return c*(p.LoftSections()-1) + 1 }

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
	Blunt      float64 // how far a printed tip falls short of the crest; zero is the model's tooth
	Slant      float64 // tan(ToothSlant): the cosine's phase moves this much per mm of v
	Bow        float64 // ToothBow: the edge falls Bow*v^2 below the cosine
}

func (g Gear) lambda() float64 { return g.Hand * g.P.Lambda() }

// angle is the cross-section's rotation about the axis at station s.
func (g Gear) angle(s float64) float64 { return s/g.lambda() + g.Mount }

// edgeAt is the toothed edge's u coordinate at (v, s): a cosine whose phase
// moves by Slant per millimetre of v, so its ridges lean across the thickness,
// lowered by Bow*v^2. Crest at Width/2 on the mid plane, root at
// Width/2 - ToothHeight, and both Bow*(Thickness/2)^2 lower at the faces. A
// blunted gear is the same edge cut flat Blunt under the crest, which is how
// bore_play_test.go stands in for a printed tip that came out rounded or short;
// every other gear has Blunt 0 and is the exact edge.
func (g Gear) edgeAt(v, s float64) float64 {
	h := g.P.ToothHeight
	e := g.P.Width/2 - h/2 + h/2*math.Cos(2*math.Pi*(s+g.Slant*v-g.Phase)/g.P.ToothPitch) - g.Bow*v*v
	return math.Min(e, g.P.Width/2-g.Blunt)
}

// envelope is everything the ribbon's cross-section can reach at any tooth
// phase: the crest rectangle. It is the same at every station, because some
// phase of the travel puts a crest at every station, and it is what the frame
// has to clear, since a gear is somewhere in its travel whenever it is in the
// frame at all. The edge reaches Width/2 on the mid plane and falls short of it
// by Bow*v^2 elsewhere, so the rectangle holds it with no room to spare at v = 0.
func (g Gear) envelope() (uHi, uLo, vHalf float64) {
	p := g.P
	return p.Width / 2, -p.Width / 2, p.Thickness / 2
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
	uHi, uLo, vHalf := g.edgeAt(v, s), -g.P.Width/2, g.P.Thickness/2
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
// space, wound counter-clockwise about +Ez. The toothed side between the two
// toothed corners is not a straight line; outline draws it.
func (g Gear) section(s float64) [4]r3.Vec {
	uLo, t := -g.P.Width/2, g.P.Thickness/2
	return [4]r3.Vec{
		g.world(uLo, -t, s),
		g.world(g.edgeAt(-t, s), -t, s),
		g.world(g.edgeAt(t, s), t, s),
		g.world(uLo, t, s),
	}
}

// toothSplinePoints is how many points the build fits each section's toothed
// side through, evenly across the thickness, ends included (spec §2).
const toothSplinePoints = 11

// outline is the cross-section at station s as a polygon wound like section:
// the back corner at -T/2, the toothed side at n points from v = -T/2 to +T/2,
// and the back corner at +T/2.
func (g Gear) outline(s float64, n int) []r3.Vec {
	uLo, t := -g.P.Width/2, g.P.Thickness/2
	out := make([]r3.Vec, 0, n+2)
	out = append(out, g.world(uLo, -t, s))
	for i := range n {
		v := -t + 2*t*float64(i)/float64(n-1)
		out = append(out, g.world(g.edgeAt(v, s), v, s))
	}
	return append(out, g.world(uLo, t, s))
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

	slant := math.Tan(p.ToothSlant)
	ga := Gear{P: p, Origin: r3.NewVec(0, 0, -a/2), Ex: ax, Ez: az, Hand: 1,
		Mount: p.MountAngleA, Phase: phaseA, Slant: slant, Bow: p.ToothBow}
	ga.Ey = ga.Ez.Cross(ga.Ex)
	gb := Gear{P: p, Origin: r3.NewVec(0, 0, a/2), Ex: bx, Ez: bz, Hand: 1,
		Mount: p.MountAngleB, Phase: phaseB, Slant: slant, Bow: p.ToothBow}
	gb.Ey = gb.Ez.Cross(gb.Ex)
	return ga, gb
}

// assemblyPhase is the tooth phase gear B is built at, with gear A at zero. It
// is not half a pitch: the two ribbons cross at an angle and their ridges lean,
// so the phase that puts a crest against a root is its own number. At the
// defaults the free window at A's zero runs from -1.850 to -0.775 mm by
// bisection, whose middle is -1.3125; TestAssemblyPhaseSitsInTheFreeWindow
// holds it to the middle of the play. It was -1.30 mm at the straight tooth
// until 2026-10-03.
const assemblyPhase = -1.31

// defaultPair is the arrangement the spec's default table describes.
func defaultPair() (Gear, Gear) {
	p := defaultParams()
	return pair(p, p.Sigma(), 0, assemblyPhase)
}

func TestToothProfileIsACosineOfTheStatedHeight(t *testing.T) {
	g, _ := defaultPair()
	p := g.P
	th := p.Thickness / 2

	if got, want := g.edgeAt(0, 0), p.Width/2; math.Abs(got-want) > 1e-12 {
		t.Errorf("crest at the tooth phase is %.6f mm from the axis, want %.6f", got, want)
	}
	if got, want := g.edgeAt(0, p.ToothPitch/2), p.Width/2-p.ToothHeight; math.Abs(got-want) > 1e-12 {
		t.Errorf("root half a pitch on is %.6f mm from the axis, want %.6f", got, want)
	}
	for _, v := range []float64{-th, -0.6, 0, 1.1, th} {
		for _, s := range []float64{0, 0.7, 1.9, 3.2} {
			if got, want := g.edgeAt(v, s+p.ToothPitch), g.edgeAt(v, s); math.Abs(got-want) > 1e-12 {
				t.Errorf("edge at v=%.2f s=%.2f repeats as %.6f one pitch on, want %.6f", v, s, got, want)
			}
			// The ridge leans: the edge at v is the mid-plane edge Slant*v further
			// along s, lowered by Bow*v^2.
			if got, want := g.edgeAt(v, s), g.edgeAt(0, s+g.Slant*v)-g.Bow*v*v; math.Abs(got-want) > 1e-12 {
				t.Errorf("edge at v=%.2f s=%.2f is %.6f, want the mid plane's %.6f", v, s, got, want)
			}
		}
	}
	// Nothing on the edge ever passes the crest or falls below the root less
	// the bow at the faces.
	floor := p.Width/2 - p.ToothHeight - p.ToothBow*th*th
	for j := range 9 {
		v := -th + 2*th*float64(j)/8
		for i := range 400 {
			s := float64(i) * p.ToothPitch / 400
			if e := g.edgeAt(v, s); e > p.Width/2+1e-12 || e < floor-1e-12 {
				t.Fatalf("edge at v=%.3f s=%.4f is %.6f, outside [%.6f, %.6f]", v, s, e, floor, p.Width/2)
			}
		}
	}
	if got := math.Atan(g.Slant) * 180 / math.Pi; math.Abs(got-25.8) > 1e-9 {
		t.Errorf("the ridges lean %.4f degrees in the chart, the spec's default is 25.8", got)
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
// crest helices cross, and the mounting angle does not spoil the rule: it
// moves that station along the ribbon instead of destroying it. The crest
// helices are not the ridges, though: a ridge runs across the thickness, and
// the straight ridge every print before 2026-10-03 carried crossed the other
// ribbon's at 74 to 80 degrees and touched it at a point. The leaned ridge of
// ToothSlant lies along the other ribbon's where they touch, which is what
// makes the contact a line (contact_test.go).
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

	// The blank is invariant, and so is the body cut from it: the toothed edge
	// one pitch on is the same at every v, so the cross-section one pitch on is
	// the same outline, leaned teeth included, and the corners of a section
	// carried by the step land on the corners of the next cell's section. This
	// is the claim the build rests on when it copies one cell and screw-moves
	// the copy: every placement is exact because the body is the same at every
	// cell, not just the twist.
	worst = 0
	for i := range 240 {
		s := float64(i) * p.ToothPitch / 24
		for j := range toothSplinePoints {
			v := -p.Thickness/2 + p.Thickness*float64(j)/float64(toothSplinePoints-1)
			if here, next := g.edgeAt(v, s), g.edgeAt(v, s+p.ToothPitch); math.Abs(here-next) > 1e-12 {
				t.Fatalf("the toothed edge at v=%.3f s=%.3f is %.6f and one pitch on it is %.6f: the body "+
					"is not one cell repeated", v, s, here, next)
			}
		}
		here, next := g.outline(s, toothSplinePoints), g.outline(s+p.ToothPitch, toothSplinePoints)
		for k := range here {
			if d := step.Apply(here[k]).Sub(next[k]).Len(); d > worst {
				worst = d
			}
		}
	}
	if worst > 1e-9 {
		t.Errorf("a section point carried by one screw step misses the next cell's by %.3e mm, want 0", worst)
	}

	// The same statement seen from the motion: advancing the gear by one pitch
	// leaves the body where it was, with the tooth phase back in step.
	moved := g
	moved.Phase = g.Phase + p.ToothPitch
	for i := range 100 {
		s := float64(i) * p.ToothPitch / 10
		for _, v := range []float64{-p.Thickness / 2, 0, p.Thickness / 2} {
			if got, want := moved.edgeAt(v, s), g.edgeAt(v, s); math.Abs(got-want) > 1e-12 {
				t.Fatalf("one pitch of advance moves the edge at v=%.2f s=%.3f from %.6f to %.6f", v, s, want, got)
			}
		}
	}
}

// The spec derives the section count from the twist per tooth with a floor of
// eight steps, eleven sections a tooth at the defaults. This is the arithmetic that
// count is bought with, and it is arithmetic about a RULED loft through those
// sections: a ruled surface cuts the corner of the true helicoid, and the
// spec's claim is that the shortfall is three orders below the backlash; a
// ruled loft's toothed edge is a chord of the cosine between sections, and the
// spec's claim is that the chord stays under four percent of the tooth height
// at any twist and under a fifth of the backlash at the defaults. It was under
// a tenth until the looser fit of 2026-10-02 narrowed the backlash from 0.82 to
// 0.46 mm without changing the printed ribbon, and so without changing the
// sections (spec/screwgear/fusion.md [SCREW-F-PRINT-MESH]); the leaned tooth of
// 2026-10-03 widened it to 1.08 mm, where the chord is 6% of it. The floor is
// what holds the chord where the twist is slow, so the leads swept here reach
// well past the one the count stops growing at.
//
// Fusion builds the cell as one loft through all the sections, which is smooth
// between them rather than ruled, and LoftFeatureInput has no ruled option. So
// these are bounds on a body Fusion does not build: the built cell passes
// through the same sections, and how far its surface departs from the helicoid
// between them, on either side, is measured by nothing in this package. This is
// the honest edge of what the section count proves. Fusion is what sees the
// built surface, and on 2026-09-28 it put every probe 0.04 mm either side of
// the helicoid, at every midpoint between sections, on the right side
// (spec/screwgear/fusion.md [SCREW-F-DIAGNOSTIC]).
//
// Nor are they bounds on the stand-in the compiled step proof builds in decad
// (TestCellLoft). decad lofts two sections at a time and walls each cell with
// two flat triangles, which depart from the ruled patch through the cell's
// corners by up to a quarter of its twist vector, (w/2)*sin(dtheta/2) on a
// face w wide: about 0.12 mm on the 15 mm faces at the defaults, against the
// ruled loft's 1.0 µm. This logs that figure and holds nothing to it;
// TestCellLoft holds the stand-in's volume, 2.2% under the helicoid's at the
// defaults, to a slack computed from the same triangles.
func TestLoftSectionCountHoldsTheHelicoid(t *testing.T) {
	p := defaultParams()
	sections := p.LoftSections()

	if got := p.CellSections(cellTeeth); got != 41 {
		t.Errorf("a %d-tooth cell lofts through %d sections at the defaults, the spec quotes 41",
			cellTeeth, got)
	}
	if got := p.CellSections(1); got != 11 {
		t.Errorf("a one-tooth cell lofts through %d sections at the defaults, the spec quotes 11", got)
	}
	t.Logf("the build's %d-tooth cell lofts through %d sections; a one-tooth cell would loft through %d",
		cellTeeth, p.CellSections(cellTeeth), p.CellSections(1))

	dtheta := (p.ToothPitch / p.Lambda()) / float64(sections-1)
	departure := p.Width / 2 * (1 - math.Cos(dtheta/2))

	if got := dtheta * 180 / math.Pi; got > 2.01 {
		t.Errorf("%d sections put %.3f deg between neighbours, which is more than the count is "+
			"derived to allow", sections, got)
	}
	t.Logf("%d sections per tooth put %.3f deg between neighbours; a ruled loft through them would "+
		"fall %.6f mm short of the helicoid at the crest and its toothed edge's chord %.4f mm short of "+
		"the cosine", sections, dtheta*180/math.Pi, departure, p.EdgeChord())
	t.Logf("decad's stand-in walls each cell with two flat triangles, which depart from that ruled loft by "+
		"up to %.3f mm on the %.0f mm faces and %.3f mm on the %.2f mm edge faces",
		p.Width/2*math.Sin(dtheta/2), p.Width, p.Thickness/2*math.Sin(dtheta/2), p.Thickness)
	// The shortfall grows with the width and the backlash with the pitch, so
	// the bound is against the backlash rather than a fixed number of microns.
	if departure > measuredBacklash/100 {
		t.Errorf("the shortfall %.6f mm is not small against the %.3f mm backlash",
			departure, measuredBacklash)
	}
	if chord := p.EdgeChord(); chord > measuredBacklash/5 {
		t.Errorf("the edge chord falls %.4f mm short of the cosine, which is not small against the "+
			"%.3f mm backlash", chord, measuredBacklash)
	}

	for _, lead := range []float64{20, 49.5, 99, 200, 400} {
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

// doublingRounds is the schedule the build repeats a cell of c teeth by, for
// n teeth in all, and returns the copy-move-join rounds it takes and the tooth
// ranges every piece lands on. The cell holds min(c, n) teeth and the ribbon
// is q = n/c whole cells and a remainder of n mod c teeth. The body doubles in
// cells while it can; whenever q has a set bit below its top one, an unmoved
// copy of the body at that size is put aside, and after the last doubling the
// asides are moved into place, largest first, each by the screw step of the
// teeth built so far. Every move is by a whole number of teeth already built,
// so every join meets at a shared cross-section and nothing overlaps. A
// remainder is a second, shorter cell lofted where it belongs and joined last;
// it is a piece but not a round (spec §3).
func doublingRounds(n, c int) (int, [][2]int) {
	if c > n {
		c = n
	}
	q, r := n/c, n%c
	m := 1
	rounds := 0
	pieces := [][2]int{{0, c}}
	var asides []int
	for bit := 0; 1<<(bit+1) <= q; bit++ {
		if q&(1<<bit) != 0 {
			asides = append(asides, m)
		}
		pieces = append(pieces, [2]int{m * c, 2 * m * c})
		m *= 2
		rounds++
	}
	for i := len(asides) - 1; i >= 0; i-- {
		pieces = append(pieces, [2]int{m * c, (m + asides[i]) * c})
		m += asides[i]
		rounds++
	}
	if m != q {
		panic("the doubling schedule does not reach the cell count")
	}
	if r > 0 {
		pieces = append(pieces, [2]int{q * c, n})
	}
	return rounds, pieces
}

// The build repeats the cell by doubling, and the spec quotes the round count
// that costs. This runs the schedule over every tooth count the dialog can
// reasonably take, with cells of one, three and four teeth so that a remainder
// is exercised at the defaults' own count and away from it, and holds three
// things: the pieces tile the ribbon exactly, no two overlap, and the count is
// floor(log2 q) + popcount(q) - 1 rounds for q whole cells plus one remainder
// piece when the count is not a multiple of the cell: five rounds at the
// default sixty-eight teeth in four-tooth cells, seven in one-tooth cells.
func TestDoublingScheduleCoversTheRibbon(t *testing.T) {
	for _, c := range []int{1, 3, 4} {
		for n := 4; n <= 512; n++ {
			rounds, pieces := doublingRounds(n, c)
			covered := make([]int, n)
			for _, piece := range pieces {
				for tooth := piece[0]; tooth < piece[1]; tooth++ {
					covered[tooth]++
				}
			}
			for tooth, times := range covered {
				if times != 1 {
					t.Fatalf("with %d teeth in %d-tooth cells the schedule lands %d pieces on tooth %d",
						n, c, times, tooth)
				}
			}
			q := n / c
			want := bits.Len(uint(q)) - 1 + bits.OnesCount(uint(q)) - 1
			if rounds != want {
				t.Errorf("with %d teeth in %d-tooth cells the schedule takes %d rounds, want %d",
					n, c, rounds, want)
			}
			wantPieces := rounds + 1
			if n%c != 0 {
				wantPieces++
			}
			if len(pieces) != wantPieces {
				t.Errorf("with %d teeth in %d-tooth cells the schedule has %d pieces, want %d",
					n, c, len(pieces), wantPieces)
			}
		}
	}
	n := defaultParams().ToothCount
	rounds, _ := doublingRounds(n, cellTeeth)
	single, _ := doublingRounds(n, 1)
	t.Logf("the default %d teeth take %d copy-move-join rounds in %d-tooth cells and %d in one-tooth cells",
		n, rounds, cellTeeth, single)
	if rounds != 5 {
		t.Errorf("the spec quotes five rounds at the defaults, the schedule takes %d", rounds)
	}
	if single != 7 {
		t.Errorf("the spec quotes seven rounds at a one-tooth cell, the schedule takes %d", single)
	}
}

// The spec follows Segerman's model on some proportions and departs from it on
// others, and its "What the video shows" table says which is which. This holds
// the defaults inside the video's ranges for the ratios the spec follows, and
// logs the ones it departs from, so that a later change to the defaults that
// stops the part looking like the video's is caught here rather than noticed
// in a picture.
//
// The video's frame no longer binds the design, since the user dropped its
// look as a requirement when the sleeve replaced it, but the sleeve still lands
// inside the widened ranges read for that frame: its outer diameter is read as
// the ring's, and its own height as the frame's.
//
// The readings are hand readings of 1280x720 frames and carry about +/-20%: the
// teeth on one face-on stretch of an arm (half a turn) at 0:09, 6:12 and 6:14
// give 20-26 per turn; the edge-on stretches at 6:14 give a thickness of 0.2-0.3
// widths; the tooth depth at 0:09 and 6:14 reads 0.15-0.2 widths and about one
// pitch; both ends of a ribbon against the ring at 6:10 give a length of about
// 12 widths; the ring's outer diameter against the face-on width of an arm at
// 0:09 and 6:10 gives 2.2-2.8 widths, and the frame's height against the ring's
// width at 5:26 and 5:34 gives about one. Each bound below is the reading's
// edge moved out by that 20%, so the 18.9 teeth per turn of the defaults, just
// under the reading, pass. Nothing here is finer than that, and a finer reading
// needs the model, not the video. The tooth depth joined the ratios the spec
// follows on 2026-09-28, when a print at the earlier 0.12-width depth showed
// teeth far too small (spec/screwgear/instructions.md, "What the print
// showed").
func TestProportionsFollowTheVideo(t *testing.T) {
	p := defaultParams()
	ga, gb := defaultPair()

	if got := p.TwistLead / p.ToothPitch; got < 16 || got > 31 {
		t.Errorf("the ribbon carries %.1f teeth per turn; the video's carries 20-26", got)
	}
	if got := p.Thickness / p.Width; got < 0.16 || got > 0.36 {
		t.Errorf("the ribbon is %.2f widths thick; the video's reads 0.2-0.3", got)
	}
	if got := p.ToothHeight / p.Width; got < 0.12 || got > 0.24 {
		t.Errorf("the teeth are %.3f widths deep; the video's read 0.15-0.2", got)
	}
	if got := p.ToothHeight / p.ToothPitch; got < 0.8 || got > 1.2 {
		t.Errorf("the teeth are %.2f pitches deep; the video's read about one", got)
	}
	if got := p.Length() / p.Width; got < 9.6 || got > 14.4 {
		t.Errorf("the ribbon is %.1f widths long; the video's reads about 12", got)
	}
	across := 2 * p.SleeveOuter() / p.Width
	tall := 2 * p.CageRise / (2 * p.SleeveOuter())
	if across < 1.76 || across > 3.36 {
		t.Errorf("the sleeve is %.2f widths across; the video's ring reads 2.2-2.8", across)
	}
	if tall < 0.64 || tall > 1.44 {
		t.Errorf("the sleeve is %.2f of its own diameter tall; the video's frame reads about one", tall)
	}
	if ga.Hand != gb.Hand {
		t.Errorf("the two gears are of opposite hand; the video's twist the same way")
	}
	if ga.Hand != 1 {
		t.Errorf("the gears are left-handed; the video's read as right-handed")
	}
	t.Logf("the sleeve is %.2f ribbon widths across and %.2f of its diameter tall", across, tall)

	t.Logf("tooth depth %.3f widths and %.2f pitches, against the video's 0.15-0.2 widths and about one pitch",
		p.ToothHeight/p.Width, p.ToothHeight/p.ToothPitch)
	// What the spec departs from, for the record of a run.
	t.Logf("crossing angle %.0f degrees, against the video's 85-100", p.Sigma()*180/math.Pi)
}
