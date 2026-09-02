package helicalgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// twistedProfileCases sweeps the regime the twisted profile has to hold across.
//
// The helix angle is signed and nothing clamps it, so both signs are swept at
// every magnitude that matters: the 14.5 degree default, the 60 degree case
// where |sin| passes |cos| and the rib and chain dimensions swap axes, and the
// quarter turn. Zero is swept too, because that is the bottom loft section the
// inherited spur pass draws. Sizes run coarse to fine, the sample count is
// taken down to four so that one missing rib is a large fraction of the system,
// and both routes into the embedded shape are reached — a high tooth count at
// the ordinary pressure angle, and a moderate tooth count at a large one.
var twistedProfileCases = []proofkit.Case{
	{Name: "default_right_hand_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(14.5), "involuteSteps": 15}},
	{Name: "left_hand_-14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-14.5), "involuteSteps": 15}},
	{Name: "untwisted_bottom_section_0deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": 0, "involuteSteps": 15}},
	{Name: "coarse_module_3_15teeth", Params: map[string]float64{
		"module": 3, "toothNumber": 15, "pressureAngle": deg(20), "helixAngle": deg(14.5), "involuteSteps": 15}},
	{Name: "fine_module_0.5_30teeth_left_hand", Params: map[string]float64{
		"module": 0.5, "toothNumber": 30, "pressureAngle": deg(20), "helixAngle": deg(-14.5), "involuteSteps": 15}},
	{Name: "pressure_angle_25deg", Params: map[string]float64{
		"module": 1, "toothNumber": 20, "pressureAngle": deg(25), "helixAngle": deg(14.5), "involuteSteps": 15}},
	{Name: "axis_swap_right_hand_60deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(60), "involuteSteps": 15}},
	{Name: "axis_swap_left_hand_-60deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-60), "involuteSteps": 15}},
	{Name: "quarter_turn_right_hand_90deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(90), "involuteSteps": 15}},
	{Name: "quarter_turn_left_hand_-90deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-90), "involuteSteps": 15}},
	{Name: "four_samples_right_hand", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(14.5), "involuteSteps": 4}},
	{Name: "four_samples_left_hand", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-14.5), "involuteSteps": 4}},
	{Name: "chorded_substitute_right_hand_14.5deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(14.5),
		"involuteSteps": 15, "chorded": 1}},
	{Name: "chorded_substitute_left_hand_-60deg", Params: map[string]float64{
		"module": 1, "toothNumber": 17, "pressureAngle": deg(20), "helixAngle": deg(-60),
		"involuteSteps": 15, "chorded": 1}},
	{Name: "embedded_by_tooth_count_45teeth", Params: map[string]float64{
		"module": 1, "toothNumber": 45, "pressureAngle": deg(20), "helixAngle": deg(14.5), "involuteSteps": 15}},
	{Name: "embedded_by_pressure_angle_30teeth_25deg", Params: map[string]float64{
		"module": 1, "toothNumber": 30, "pressureAngle": deg(25), "helixAngle": deg(-14.5), "involuteSteps": 15}},
}

// stepTwistedGearProfile draws the Twisted Gear Profile sketch and checks what
// the loft below it selects on.
//
// The harness gate is the whole verdict: DOF 0, no redundant or conflicting
// constraint, valid profiles, a system that is not near-singular, and no
// discrete ambiguity. Nothing here is waived, so a scheme that reached DOF 0
// but still admitted the mirrored or half-turn answer would fail.
//
// The two chorded cases in the table put the substitute section stepLoftTooth
// has to loft through this same gate, so the solid step can draw it and only
// solve. Their sketch is the same construction with the two flank splines and
// the two arcs replaced by chords, and the one perpendicular the tooth-top arc
// used to supply put back on the last rib.
func stepTwistedGearProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := gearOf(p)
	d := g.dims()
	style := faithfulProfile
	if p["chorded"] == 1 {
		style = chordedProfile
	}

	proofkit.Step(t, "draw the tooth at helix %.4f rad on module %.2f, %.0f teeth, %d samples",
		g.helixAngle, g.module, g.toothNumber, g.involuteSteps)
	sec := drawSection(t, s, g, style)
	solve(t, s)

	if sec.embedded {
		assertEmbeddedIsUnreachable(t, s, d)
		return
	}

	proofkit.Step(t, "the regions the sketch closes, and their curve counts")
	if style == chordedProfile {
		assertChordedContract(t, s, g)
	} else {
		assertProfileContract(t, s)
	}

	proofkit.Step(t, "the twist the spine carries, and its sign")
	assertTwist(t, sec, g.helixAngle)
}

// assertChordedContract checks the substitute section the solid step lofts.
//
// It closes one region, the tooth, since its root circle is construction
// geometry and the root boundary is drawn as an explicit chord. Every curve on
// that region is a line, which is what decad's loft admits: two chords per
// flank sample gap, the two flank-to-root lines, the tooth-top chord and the
// root chord.
func assertChordedContract(t testing.TB, s *sketch.Sketch, g gear) {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("the chorded section closes exactly one region, the tooth; got %d", len(profiles))
	}
	if !profiles[0].Valid {
		t.Error("the chorded tooth region is not an extrudable profile")
	}
	want := curveCounts{lines: 2*(g.involuteSteps-1) + 4}
	if got := countCurves(profiles[0]); got != want {
		t.Errorf("the chorded tooth loop is %+v, not the all-line loop %+v a loft can pair "+
			"segment for segment", got, want)
	}
}

// assertProfileContract checks the curve counts the two later extrudes match
// on. [HELI-F-LOFT] finds each loft section with a fixed nurbs=2, arcs=2,
// lines=2, and spur's body extrude finds the disc with arcs=2; both counts are
// a contract of this sketch, so they are read off the sketch that was actually
// drawn.
//
// The engine reports a curve split by the arrangement against its parent
// entity, so the root arc the tooth borrows from the root circle appears as
// that Circle rather than as an Arc. The tooth loop is therefore two splines,
// two lines, the tooth-top arc and the root circle: six curves, which is
// Fusion's nurbs=2, arcs=2, lines=2 with its two arcs being the tooth-top arc
// and the root arc. The disc is that same circle, split at the two stub feet
// into the two arcs Fusion counts.
func assertProfileContract(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	profiles := s.Profiles()
	if len(profiles) != 2 {
		t.Fatalf("the Gear Profile sketch closes exactly two regions, the tooth and the disc "+
			"inside the root circle; got %d", len(profiles))
	}
	var tooth, disc *sketch.Profile
	for _, p := range profiles {
		if !p.Valid {
			t.Errorf("a detected region is not an extrudable profile: area %.4f", p.Area)
		}
		if countCurves(p).splines == 2 {
			tooth = p
		} else {
			disc = p
		}
	}
	if tooth == nil || disc == nil {
		t.Fatal("the sketch does not close one tooth region and one disc region")
	}
	got := countCurves(tooth)
	want := curveCounts{splines: 2, lines: 2, arcs: 1, circles: 1}
	if got != want {
		t.Errorf("the tooth loop is %+v, not the six-curve loop %+v that [HELI-F-LOFT] searches "+
			"for as nurbs=2, arcs=2, lines=2", got, want)
	}
	if got := countCurves(disc); got != (curveCounts{circles: 1}) {
		t.Errorf("the disc loop is %+v, not the root circle split into the two arcs spur's body "+
			"extrude searches for", got)
	}
	if tooth.Area <= 0 || disc.Area <= tooth.Area {
		t.Errorf("tooth area %.4f and disc area %.4f are not one tooth inside one gear body",
			tooth.Area, disc.Area)
	}
}

// assertTwist reads the twist back off the solved sketch.
//
// The angle from the +X reference to the spine is the quantity the confirming
// angular dimension drives ([SPUR-F-ROTATE-CONFIRM]), so reading it back is the
// check that the sketch proves its own twist rather than merely looking
// twisted. A scheme that dropped or flipped the sign would still solve at a
// positive helix angle and come out mirrored at a negative one, so the sign is
// checked, not just the magnitude.
//
// The tooth also has to be the spur tooth, rigidly rotated: every left flank
// sample sits as far from the spine as its right partner, and the tooth top
// sits on the tip circle at exactly the helix angle.
func assertTwist(t testing.TB, sec *section, want float64) {
	t.Helper()
	const tol = 1e-9
	if got := sec.reference.AngleTo(sec.spine); math.Abs(got-want) > tol {
		t.Errorf("the angle from the +X reference to the spine is %.9f rad, not the helix angle "+
			"%.9f rad", got, want)
	}
	if got := math.Atan2(sec.toothTop.Y(), sec.toothTop.X()); math.Abs(got-want) > 1e-7 {
		t.Errorf("the tooth top sits at %.9f rad, not at the helix angle %.9f rad", got, want)
	}
	if got := math.Hypot(sec.toothTop.X(), sec.toothTop.Y()); math.Abs(got-sec.dims.Tip) > 1e-7 {
		t.Errorf("the tooth top sits at radius %.6f, not on the tip circle at %.6f",
			got, sec.dims.Tip)
	}
	for i := range sec.leftPts {
		l := sec.leftPts[i].DistanceToLine(sec.spine)
		r := sec.rightPts[i].DistanceToLine(sec.spine)
		if math.Abs(l-r) > 1e-7 {
			t.Errorf("flank sample %d sits %.6f from the spine on the left and %.6f on the right, "+
				"so the tooth is not symmetric about the twist direction", i, l, r)
		}
	}
}

// assertEmbeddedIsUnreachable records the limitation [HELI-F-LOFT] declares.
//
// When the base circle falls inside the root circle no flank-to-root line is
// drawn, so the tooth loop carries no line at all. helical's loft passes a
// fixed nurbs=2, arcs=2, lines=2 to find each section and never reads
// ctx.toothProfileIsEmbedded, so there is no count it could match here and an
// embedded helical gear cannot be built. That is faithful to the spec, which
// declares embedded helical unsupported, and it is checked rather than
// asserted in prose: the sketch is drawn and its loops counted.
func assertEmbeddedIsUnreachable(t testing.TB, s *sketch.Sketch, d involute.Dimensions) {
	t.Helper()
	if !d.Embedded() {
		t.Fatalf("this case was meant to reach the embedded shape, but base %.4f is outside "+
			"root %.4f", d.Base, d.Root)
	}
	for _, p := range s.Profiles() {
		if c := countCurves(p); c.lines != 0 {
			t.Errorf("an embedded profile still carries %d line(s); the flank-to-root stubs are "+
				"supposed to be absent", c.lines)
		}
	}
	proofkit.Unmodelled(t, "an embedded profile has no flank-to-root lines, so [HELI-F-LOFT]'s "+
		"fixed nurbs=2, arcs=2, lines=2 search matches nothing and helical has no embedded branch")
}
