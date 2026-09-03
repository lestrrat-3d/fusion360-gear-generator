package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
)

// THE PROFILE SKETCH IS DRAWN IN ITS OWN GEAR'S AXIAL FRAME. Fusion draws both
// hexagons on the one Gear Profiles plane, at the world positions §2 solved, and
// revolves each about its own first edge. The frame used below puts that first
// edge on the sketch's x axis, which is a rigid motion of the same plane — every
// length, angle and area is the one §2 solved, and the revolve axis is then the
// sketch's own x axis. What it costs: nothing measurable, but the proof does not
// show that the two hexagons coexist in one plane, which is why the §2 step
// checks both of them there.
//
// The substitutions the SOLID steps make, and what each costs, are set out at
// the top of frustum_test.go, bodies_test.go and segments_test.go, next to the
// steps that make them.

// solidCases sweeps the pairs and both gears through the solid chain.
//
// The two ratio directions are both here because the frustum's binding
// dimension — the Maximum Face Width — is set by whichever gear is smaller, and
// because the conical trims and the spiral both behave differently once the two
// cone angles diverge. The bore branch is swept on both sides: enabled with the
// auto diameter, enabled with a given one, and disabled.
var solidCases = []proofkit3d.Case{
	{Name: "default_31_31_pinion", Params: defaultParams()},
	{Name: "default_31_31_driving", Params: with(defaultParams(), pGear, 1.0)},
	{Name: "ratio_31_17_pinion", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_31_17_driving", Params: with(defaultParams(), pPinionTeeth, 17.0, pGear, 1.0)},
	{Name: "ratio_17_31_pinion", Params: with(defaultParams(), pDrivingTeeth, 17.0)},
	{Name: "module_2_19_13_driving", Params: with(defaultParams(),
		pModule, 2.0, pDrivingTeeth, 19.0, pPinionTeeth, 13.0, pGear, 1.0)},
	{Name: "shaft_angle_35deg_pinion", Params: with(defaultParams(), pShaftAngle, deg(35))},
	{Name: "shaft_angle_140deg_driving", Params: with(defaultParams(),
		pShaftAngle, deg(140), pGear, 1.0)},
	{Name: "minimum_teeth_4_4_pinion", Params: with(defaultParams(),
		pDrivingTeeth, 4.0, pPinionTeeth, 4.0)},
	{Name: "given_bore_diameter", Params: with(defaultParams(), pPinionBore, 4.0)},
	{Name: "bore_disabled", Params: with(defaultParams(), pBoreEnable, 0.0)},
}

// profileCases is solidCases as a sketch table: the hexagon has to close and be
// fully constrained for every pair either gear can be built from.
var profileCases = []proofkit.Case{
	{Name: "default_31_31_pinion", Params: defaultParams()},
	{Name: "default_31_31_driving", Params: with(defaultParams(), pGear, 1.0)},
	{Name: "ratio_31_17_pinion", Params: with(defaultParams(), pPinionTeeth, 17.0)},
	{Name: "ratio_31_17_driving", Params: with(defaultParams(), pPinionTeeth, 17.0, pGear, 1.0)},
	{Name: "shaft_angle_35deg_driving", Params: with(defaultParams(),
		pShaftAngle, deg(35), pGear, 1.0)},
	{Name: "shaft_angle_140deg_pinion", Params: with(defaultParams(), pShaftAngle, deg(140))},
	{Name: "minimum_teeth_4_4_driving", Params: with(defaultParams(),
		pDrivingTeeth, 4.0, pPinionTeeth, 4.0, pGear, 1.0)},
	{Name: "face_width_at_the_maximum_pinion", Params: with(defaultParams(),
		pFaceWidth, atMaxFaceWidth)},
	{Name: "base_heights_at_their_minima_driving", Params: with(defaultParams(),
		pDrivingHeight, atMinBaseHeight, pPinionHeight, atMinBaseHeight, pGear, 1.0)},
}

// hexagonOf resolves one case into the six frustum corners in the gear's own
// axial frame, in the spec's draw order.
func hexagonOf(t testing.TB, params map[string]float64) (pair, gearSide, []vec2) {
	t.Helper()
	p := resolveSentinels(t, params)
	q := pairOf(t, p)
	side := q.gearOf(p)
	return q, side, q.hexagon(q.build(), side)
}

// stepProfileSketch draws one gear's `{gearLabel} Profile` sketch: the six §2
// vertices recreated as new points and the closed hexagon drawn on them.
//
// The recipe is [PB-PROJECT-NOT-FIXED]'s recreate-share-fix: recreate each
// vertex at its exact world-mapped position, draw the six lines SHARING those
// points, and fix the lines' endpoints only AFTER the lines exist. Fixing a bare
// point before it is consumed as an endpoint does not leave the sketch fully
// constrained, so the order is the constraint. What the fixing buys is a first
// edge with a trustworthy worldGeometry ([PB-WORLDGEO-CONSTRAINED]) — the edge
// the revolve, the pattern, the bore plane and the meshing rotation all take as
// the shaft axis.
//
// <!-- proof-run: proofkit.Run(profileCases, stepProfileSketch) -->
func stepProfileSketch(t testing.TB, s *sketch.Sketch, params map[string]float64) {
	q, side, hex := hexagonOf(t, params)

	proofkit.Step(t, "%s: recreate the six §2 vertices as new points", side.label)
	pts := make([]*sketch.Point, len(hex))
	names := []string{"A/B", "G/I", "H/J", "C/D", "M/O", "N/P"}
	for i, v := range hex {
		pts[i] = s.CreatePoint(v.X, v.Y)
		pts[i].SetName(names[i])
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points, in the spec's draw order")
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}

	proofkit.Step(t, "fix the endpoints, after the lines exist and not before")
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}

	solve(t, s)

	proofkit.Step(t, "the one loop the revolve takes [PB-SINGLE-PROFILE]")
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("%s: the Profile sketch closes %d regions; one profile sketch per gear means it "+
			"holds exactly this one hexagon loop", side.label, len(profiles))
	}
	if !profiles[0].Valid {
		t.Fatalf("%s: the hexagon is not an extrudable profile", side.label)
	}
	if got := countCurves(profiles[0]); got != (curveCounts{lines: 6}) {
		t.Errorf("%s: the hexagon loop is %+v, not the six lines it is drawn from", side.label, got)
	}
	if got, want := profiles[0].Area, math.Abs(shoelace(hex)); math.Abs(got-want) > 1e-7 {
		t.Errorf("%s: the loop encloses %.9f against the §2 hexagon's %.9f", side.label, got, want)
	}

	proofkit.Step(t, "the first edge is the shaft axis, and the profile lies on one side of it")
	if math.Abs(pts[0].Y()) > 1e-9 || math.Abs(pts[1].Y()) > 1e-9 {
		t.Errorf("%s: the first edge runs off the shaft axis, from y %.9f to y %.9f",
			side.label, pts[0].Y(), pts[1].Y())
	}
	for i := 2; i < len(pts); i++ {
		if pts[i].Y() <= 0 {
			t.Errorf("%s: corner %s sits at y %.9f; the revolved profile must not cross its axis "+
				"[PB-REVOLVE]", side.label, names[i], pts[i].Y())
		}
	}

	proofkit.Step(t, "what the revolve and the bore that follow it are for")
	solved := make([]vec2, len(pts))
	for i, p := range pts {
		solved[i] = at(p)
	}
	assertRevolveAndBore(t, q, side, solved)
}

// pappusVolume is the exact volume a closed section sweeps in a full turn about
// the sketch's x axis: 2*pi times the section's first moment about that axis.
//
// It is exact for a polygon, so the Gear Body the revolve step builds has a
// closed-form volume even though no solid gate here can measure one.
func pappusVolume(pts []vec2) float64 {
	moment := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		cross := pts[i].X*pts[j].Y - pts[j].X*pts[i].Y
		moment += cross * (pts[i].Y + pts[j].Y)
	}
	return math.Abs(2 * math.Pi * moment / 6)
}

// minRadius and maxRadius are the frustum's smallest and largest distances from
// the shaft axis over the corners that are off it. The first two corners sit on
// the axis by construction and are not part of either.
func minRadius(hex []vec2) float64 {
	best := math.Inf(1)
	for i := 2; i < len(hex); i++ {
		best = math.Min(best, hex[i].Y)
	}
	return best
}

func maxRadius(hex []vec2) float64 {
	best := 0.0
	for i := 2; i < len(hex); i++ {
		best = math.Max(best, hex[i].Y)
	}
	return best
}

// assertRevolveAndBore reads off the section the two quantities the steps below
// it consume. Both of those steps have gates of their own — stepRevolveGearBody
// builds the blank as the bands its edges sweep and stepBoreCut draws the bore's
// result — so this is not standing in for either; it catches a hexagon drawn on
// the wrong corners here, on the sketch, as well as there, on the solid.
//
// THE REVOLVE. The Gear Body's volume is the section's Pappus volume, and the
// two cone faces the conical trim later searches for are the ones the toe and
// heel edges sweep. Both are read off the hexagon this step gated, so a hexagon
// drawn on the wrong corners misses them.
//
// THE BORE. The bore diameter resolves to this gear's own input when it is
// non-zero and to its Pitch Diameter / 4 when it is not, and the resulting
// cylinder has to stay inside the frustum's smallest radius, or the through-cut
// would break the body into pieces rather than bore it.
func assertRevolveAndBore(t testing.TB, q pair, side gearSide, hex []vec2) {
	t.Helper()
	if v := pappusVolume(hex); v <= 0 {
		t.Errorf("%s: the section sweeps a volume of %.9f", side.label, v)
	}
	assertSweptCone(t, side.label, "toe", hex[4], hex[5])
	assertSweptCone(t, side.label, "heel", hex[2], hex[3])

	if !q.boreEnable {
		return
	}
	if got, want := side.bore, side.pitchDia/4; !closeTo(got, want, 1e-12) {
		// A given diameter is used as given; only a zero one auto-resolves.
		if side.label == "Pinion" && q.pinionBore == got {
			return
		}
		if side.label == "Driving" && q.drivingBore == got {
			return
		}
		t.Errorf("%s: the bore diameter resolved to %.9f, neither the given value nor the "+
			"Pitch Diameter / 4 = %.9f", side.label, got, want)
	}
	// The blank tapers to zero radius at its toe corner A/B, which sits ON the
	// shaft axis, so a bore of any diameter necessarily truncates the toe end;
	// that much is ordinary for a bevel blank. What would leave nothing at all is
	// a bore wider than the blank ever gets, and NOTHING IN THE SPEC BOUNDS THE
	// BORE DIAMETER AGAINST THE BLANK. The default Pitch Diameter / 4 clears the
	// widest radius comfortably on every case in this table, but it eats a real
	// share of the face: the numbers are logged so the margin is visible rather
	// than assumed.
	r := side.bore / 2
	if r >= maxRadius(hex) {
		t.Errorf("%s: the bore radius %.9f is at or beyond the blank's widest radius %.9f, so the "+
			"through-cut would consume the whole Gear Body", side.label, r, maxRadius(hex))
	}
	t.Logf("%s: a bore radius of %.6f against a blank running %.6f to %.6f", side.label, r,
		minRadius(hex), maxRadius(hex))
}

// assertSweptCone checks that one hexagon edge really sweeps a cone, and names
// its half-angle — the angle the conical-cut step's face search has to match.
func assertSweptCone(t testing.TB, label, what string, from, to vec2) {
	t.Helper()
	rise := math.Abs(to.Y - from.Y)
	run := math.Abs(to.X - from.X)
	if rise < 1e-12 {
		t.Errorf("%s: the %s edge is parallel to the shaft axis, so it sweeps a cylinder and the "+
			"conical cut has no cone face to find [PB-FACE-BY-MIDPOINT]", label, what)
		return
	}
	if run < 1e-12 {
		t.Errorf("%s: the %s edge is perpendicular to the shaft axis, so it sweeps a flat annulus "+
			"rather than a cone", label, what)
		return
	}
	half := math.Atan2(rise, run)
	if half <= 0 || half >= math.Pi/2 {
		t.Errorf("%s: the %s cone's half-angle is %.9f rad", label, what, half)
	}
}
