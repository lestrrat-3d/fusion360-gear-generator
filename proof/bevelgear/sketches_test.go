package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// What the sketch harness models, and what it does not.
//
// Every sketch step below draws into the harness's sketch, which sits on the
// world XY plane. The generated module draws the same figure on a plane derived
// from the user's target plane, which the bench has no way to be handed. The
// substitution is exact for everything these steps assert, because §2 is built
// in the sketch's OWN 2-D frame ([BEVEL-F-APEX-LOCAL]) and every direction in it
// is taken relative to the projected anchor line — nothing in the lattice reads
// a world coordinate. The one thing that does, the grow side, is the one-bit
// normal comparison of [BEVEL-F-GROW-SIDE], and stepGearProfilesPlane proves it
// on real world planes rather than assuming it.
//
// Three of Fusion's constraint calls carry a different number of independent
// rows here than they do in Fusion, and each substitution is named where it is
// made: the G->H and J->I perpendiculars (omitted — instructions.md §2 says so
// in full), the toe lines' addParallel (omitted — NewOffset already holds both
// endpoints, so it carries the parallelism), and point-to-point coincidences
// whose second row this net already implies (substituted by the single
// point-on-line row, as [PB-COLLINEAR-CHAIN] describes for addCollinear).
// Nothing weakens the gate: proofkit.RequireSound still demands DOF 0 with no
// redundant or conflicting constraint.

// bgDraw is the small bit of bookkeeping every sketch step shares: it names
// each point and line it creates, so a failed gate reports the lattice's own
// letters rather than point#41.
type bgDraw struct {
	t testing.TB
	s *sketch.Sketch
}

func (d *bgDraw) pt(p bgPt, name string) *sketch.Point {
	q := d.s.CreatePoint(p.X, p.Y)
	q.SetName(name)
	return q
}

// line draws a §2 construction line from raw coordinates. Every §2 line is a
// construction line, and every one of them is built this way and then pinned
// with one addCoincident per end ([BEVEL-F-COINCIDENT-STYLE]); no §2 line
// shares an existing point.
func (d *bgDraw) line(a, b *sketch.Point, name string) *sketch.Line {
	l := d.s.CreateLine(a, b)
	l.SetConstruction(true)
	return l
}

func (d *bgDraw) add(name string, c sketch.Constraint) sketch.Constraint {
	d.s.AddConstraint(c)
	d.s.SetConstraintName(c, name)
	return c
}

// bgOffsetValue is the signed value NewOffset needs. The engine's offset is
// positive to the LEFT of the source line's own direction, where Fusion's
// addOffsetDimension takes an unsigned distance and reads its side off the
// seeded geometry ([PB-DIM-VALUE-SEMANTICS]). The side is the same either way;
// this turns the magnitude the step list carries into the engine's spelling.
func bgOffsetValue(srcStart, srcEnd, target bgPt, d float64) float64 {
	if bgCross(bgSub(srcEnd, srcStart), bgSub(target, srcStart)) < 0 {
		return -d
	}
	return d
}

// bgTurnDeg is the signed counter-clockwise turn, in degrees, from direction d1
// to direction d2 — what NewAngle measures.
//
// It exists because Fusion pins several of §2's directions with an UNSIGNED
// addPerpendicular / addParallel and takes the side from the seed, while the
// bench engine's probe reports the other side as a second discrete
// configuration and RequireSound refuses it. Substituting the signed angle is
// the same one row with the seed's bit written into it, exactly the crossover
// [PB-DIM-VALUE-SEMANTICS] describes for dimension values. The cost is real and
// named at each site: a module that seeds one of these the wrong way round
// still builds the mirrored figure, and this proof cannot see that.
func bgTurnDeg(d1, d2 bgPt) float64 {
	return math.Atan2(bgCross(d1, d2), bgDot(d1, d2)) * 180 / math.Pi
}

// bgQuarterTurn rounds bgTurnDeg to the exact right angle it is meant to be, so
// the constraint carries 90 or -90 rather than 89.999999999.
func bgQuarterTurn(d1, d2 bgPt) float64 {
	if bgTurnDeg(d1, d2) >= 0 {
		return 90
	}
	return -90
}

// ----------------------------------------------------------------------------
// S04 — the Anchor sketch.

var anchorCases = []proofkit.Case{
	{Name: "default_pair", Params: map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90}},
	{Name: "ratio_pair", Params: map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90}},
	{Name: "module_8", Params: map[string]float64{idModule: 8, idDrivingTeeth: 19, idPinionTeeth: 13, idShaftAngle: 60}},
}

// stepAnchorSketch draws the Anchor sketch: the projected centre point and the
// Anchor Line through it.
//
// The line's length is arbitrary — nothing downstream reads it — so the step
// proves the thing that is NOT arbitrary: that midpoint, length and a
// sketch-local direction lock leave the line with zero freedom, which is what
// [BEVEL-F-FULL-CONSTRAINT] demands of this sketch.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := &bgDraw{t: t, s: s}
	proofkit.Step(t, "project the user centre point into the Anchor sketch")
	center := s.CreateReferencePoint(0, 0, "user centre point")
	center.SetName("projected centre")

	proofkit.Step(t, "draw the Anchor Line, seeded at +-0.5 cm from the projected centre")
	a := d.pt(bgPt{-5, 0}, "anchor line start")
	b := d.pt(bgPt{5, 0}, "anchor line end")
	line := d.s.CreateLine(a, b)

	// Fusion takes BOTH addCoincident(projectedCentre, anchorLine) and
	// addMidPoint(projectedCentre, anchorLine), and instructions.md says to use
	// both rather than the midpoint alone. The engine's NewMidpoint already
	// carries both rows, so adding the point-on-line row here would be the
	// third row over two freedoms and the gate would report it redundant. The
	// proof keeps the midpoint, which is the stronger of the two.
	d.add("centre bisects the anchor line", sketch.NewMidpoint(center, line))
	// Fusion's aligned distance dimension is a magnitude whose direction is
	// captured from the seed ([PB-DIM-VALUE-SEMANTICS]); the engine's
	// horizontal-distance target is signed, and the sign is how that seed side
	// crosses over. Without it the line satisfies every constraint end-for-end
	// as well, and the gate reports two discrete configurations.
	d.add("anchor line length", sketch.NewHorizontalDistance(a, b, 10))
	d.add("anchor line direction", sketch.NewHorizontal(line))

	if got := line.Length(); math.Abs(got-10) > 1e-9 {
		t.Errorf("anchor line seeded length: got %.6f mm, want 10 mm", got)
	}
}

// ----------------------------------------------------------------------------
// S05 — the Gear Profiles plane.

var gearProfilesPlaneCases = []proofkit.Case{
	{Name: "target_xy", Params: map[string]float64{bgTiltKey: 0, idShaftAngle: 90}},
	{Name: "target_tilted_30", Params: map[string]float64{bgTiltKey: 30, idShaftAngle: 90}},
	{Name: "target_tilted_90", Params: map[string]float64{bgTiltKey: 90, idShaftAngle: 60}},
	{Name: "target_tilted_negative", Params: map[string]float64{bgTiltKey: -55, idShaftAngle: 120}},
}

// bgTiltKey is the proof's own case knob: how far the user's target plane is
// tilted out of world XY. It is not a dialog input — the target plane is a
// selection — so it is spelled apart from the id* keys.
const bgTiltKey = "targetPlaneTiltDegrees"

// stepGearProfilesPlane builds the Gear Profiles plane and proves the two
// properties the build depends on.
//
// The plane is `setByAngle(anchorLine, '90 deg', targetPlane)` — through the
// Anchor Line, at 90° to the target plane, built off the ORIGINAL target plane
// and never a re-derived copy ([PB-USE-SELECTED-PLANE]). What matters
// downstream is that inside its sketch the in-plane perpendicular to the
// projected anchor line IS the target-plane normal, which is exactly why the
// apex can be placed in sketch-local coordinates ([BEVEL-F-APEX-LOCAL]) and why
// the grow side is a one-bit comparison against that normal
// ([BEVEL-F-GROW-SIDE]).
//
// Substitution: a Fusion construction plane has no counterpart on the bench, so
// the step builds the same two planes in the sketch engine's World and reads
// their frames. The cost is that Fusion's own setByAngle is not exercised; what
// is proved is the geometry that call has to produce, over target planes tilted
// every way including flat and past vertical.
func stepGearProfilesPlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	tilt := bgGet(p, bgTiltKey, 0) * math.Pi / 180
	w := s.World()

	proofkit.Step(t, "the user's target plane, tilted %.1f deg out of world XY", bgGet(p, bgTiltKey, 0))
	// Rotating XY about its own +X axis by `tilt` keeps the anchor line on +X
	// and swings the normal, which is the only freedom that matters here.
	tx := r3.NewVec(1, 0, 0)
	ty := r3.NewVec(0, math.Cos(tilt), math.Sin(tilt))
	target, err := w.CreatePlaneFromPoints(r3.NewVec(0, 0, 0), tx, ty)
	if err != nil {
		t.Fatalf("target plane: %v", err)
	}
	targetFrame, err := target.Frame()
	if err != nil {
		t.Fatalf("target frame: %v", err)
	}
	normal := targetFrame.N()

	proofkit.Step(t, "the Gear Profiles plane: through the Anchor Line, 90 deg off the target plane")
	profiles, err := w.CreatePlaneFromPoints(r3.NewVec(0, 0, 0), tx, normal)
	if err != nil {
		t.Fatalf("gear profiles plane: %v", err)
	}
	profilesFrame, err := profiles.Frame()
	if err != nil {
		t.Fatalf("gear profiles frame: %v", err)
	}

	// It contains the Anchor Line ...
	bgCloseTB(t, "anchor line lies in the Gear Profiles plane",
		profilesFrame.N().Dot(tx), 0, 1e-12)
	// ... and stands square to the target plane.
	bgCloseTB(t, "Gear Profiles plane is perpendicular to the target plane",
		profilesFrame.N().Dot(normal), 0, 1e-12)
	// ... so inside its sketch, the in-plane perpendicular to the projected
	// anchor line is the target normal itself. perp = (-d.y, d.x) for the
	// projected anchor direction d = +U.
	perpWorld := profilesFrame.ToWorldUV(0, 1).Sub(profilesFrame.Origin())
	grow := 1.0
	if perpWorld.Dot(normal) < 0 {
		grow = -1
	}
	bgCloseTB(t, "the grow direction is the target normal",
		perpWorld.Scale(grow).Sub(normal).Len(), 0, 1e-9)

	proofkit.Step(t, "the sketch-local consequence: the apex sits on the grow side")
	d := &bgDraw{t: t, s: s}
	center := s.CreateReferencePoint(0, 0, "projected centre")
	a := s.CreateReferencePoint(-5, 0, "projected anchor start")
	b := s.CreateReferencePoint(5, 0, "projected anchor end")
	anchor, err := s.CreateReferenceLine(a, b, "projected Anchor Line")
	if err != nil {
		t.Fatalf("projected anchor line: %v", err)
	}
	start := d.pt(bgPt{0, 0}, "centre->apex start")
	apex := d.pt(bgPt{0, grow * 20}, "Apex")
	toApex := d.line(start, apex, "centre->apex")
	d.add("centre->apex starts at the projected centre", sketch.NewCoincident(start, center))
	d.add("centre->apex is perpendicular to the anchor line", sketch.NewPerpendicular(toApex, anchor))
	// The §2 net leaves this line's length to the Apex 2 closure; here there is
	// no closure to leave it to, so the step dimensions it. That dimension is
	// the proof's alone — [BEVEL-F-DRIVEN-DIMS] forbids it in §2, where the
	// closure drives the length. Its SIGN is the grow-side bit: it is how the
	// bench spells the seed side Fusion picks from the target normal.
	d.add("apex height (proof-only: §2 leaves this driven)",
		sketch.NewVerticalDistance(start, apex, grow*20))
	if grow*apex.Y() <= 0 {
		t.Errorf("apex grew onto the wrong side: grow=%+.0f, apex Y=%.4f", grow, apex.Y())
	}
}

// ----------------------------------------------------------------------------
// S06 — the Gear Profiles sketch, §2.

// bgLattice2D is what stepGearProfiles hands back to the steps that measure it.
type bgLattice2D struct {
	lat   bgLattice
	point map[string]*sketch.Point
	line  map[string]*sketch.Line
}

var gearProfilesCases = []proofkit.Case{
	{Name: "default_31_31_90", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90}},
	{Name: "ratio_31_17_90", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90}},
	{Name: "ratio_17_31_90_driving_is_smaller", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 17, idPinionTeeth: 31, idShaftAngle: 90}},
	{Name: "shaft_angle_30_declared_refusal", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 30, idRefused: 1}},
	{Name: "shaft_angle_35", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 35}},
	{Name: "shaft_angle_120", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 120}},
	{Name: "shaft_angle_142", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 142}},
	{Name: "shaft_angle_150_ceiling", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 150}},
	{Name: "minimum_teeth_4_4_90", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 4, idPinionTeeth: 4, idShaftAngle: 90}},
	{Name: "toe_extension_0", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idToeExtension: 0}},
	{Name: "toe_extension_50", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idToeExtension: 50}},
	{Name: "toe_extension_100", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idToeExtension: 100}},
	{Name: "toe_radius_user_below_ceiling", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90,
		idPinionToeRadius: 9, idDrivingToeRadius: 2, idToeExtension: 25}},
	{Name: "tooth_spacing_0", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idToothSpacing: 0}},
	{Name: "tooth_spacing_positive", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idToothSpacing: 0.4}},
	{Name: "base_heights_user_specified", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90,
		idDrivingBaseHeight: 6, idPinionBaseHeight: 2}},
	{Name: "face_width_user_at_cap", Params: map[string]float64{
		idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90, idFaceWidth: 10.41}},
	{Name: "module_8_ratio_19_13_60", Params: map[string]float64{
		idModule: 8, idDrivingTeeth: 19, idPinionTeeth: 13, idShaftAngle: 60, idToeExtension: 40}},
	{Name: "module_4_ratio_43_31_75", Params: map[string]float64{
		idModule: 4, idDrivingTeeth: 43, idPinionTeeth: 31, idShaftAngle: 75, idToothSpacing: 0.2}},
}

// stepGearProfiles draws the whole §2 lattice and proves it closes.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	bgAssertLattice(t, bgBuildGearProfiles(t, s, p))
}

// bgBuildGearProfiles is stepGearProfiles' body, returning the drawn figure so
// the per-gear steps can measure the same net rather than a re-derived one.
func bgBuildGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) *bgLattice2D {
	in := bgRead(p)
	l := bgSolve(in)
	if in.DeclaredRefusal {
		proofkit.Unmodelled(t, "declared refusal: the spec admits Shaft Angle %.0f deg, this §2 net "+
			"reads below the sketch engine's conditioning floor there — a property of the net, "+
			"recorded rather than removed from the range", in.ShaftAngle*180/math.Pi)
	}
	d := &bgDraw{t: t, s: s}
	out := &bgLattice2D{lat: l, point: map[string]*sketch.Point{}, line: map[string]*sketch.Line{}}
	keep := func(name string, q *sketch.Point) *sketch.Point { out.point[name] = q; return q }
	keepLine := func(name string, ln *sketch.Line) *sketch.Line { out.line[name] = ln; return ln }

	proofkit.Step(t, "project the Anchor sketch's centre point and anchor line")
	center := s.CreateReferencePoint(0, 0, "projected Anchor centre")
	ra := s.CreateReferencePoint(-5, 0, "projected anchor start")
	rb := s.CreateReferencePoint(5, 0, "projected anchor end")
	anchor, err := s.CreateReferenceLine(ra, rb, "projected Anchor Line")
	if err != nil {
		t.Fatalf("projected anchor line: %v", err)
	}
	keep("c", center)

	proofkit.Step(t, "centre->apex, perpendicular to the anchor line, apex seeded at c + perp*(R cos gamma_g + driving base height)")
	capS := d.pt(bgPt{0, 0}, "centre->apex start")
	apex := keep("Apex", d.pt(l.Apex, "Apex"))
	centerToApex := keepLine("centerToApex", d.line(capS, apex, "centre->apex"))
	d.add("centre->apex starts at the projected centre", sketch.NewCoincident(capS, center))
	// Fusion applies addPerpendicular here and takes the side from the seed,
	// which the target normal chose ([BEVEL-F-GROW-SIDE]). Perpendicular alone
	// is unsigned, so on the bench the whole lattice also satisfies every
	// constraint rotated 180° about the projected centre, and the gate reports
	// the two configurations rather than picking the seeded one. The signed
	// angle is the same one row with that seed bit written down.
	d.add("centre->apex perpendicular to the anchor line (signed: the grow side)",
		sketch.NewAngle(anchor, centerToApex, 90))

	proofkit.Step(t, "the Driving Gear Shaft Axis, Apex->B, parallel to centre->apex")
	dAxS := d.pt(l.Apex, "Apex->B start")
	pB := keep("B", d.pt(l.Driving.Shaft, "B"))
	drivingAxis := keepLine("drivingAxis", d.line(dAxS, pB, "Apex->B"))
	d.add("Apex->B starts at the Apex", sketch.NewCoincident(dAxS, apex))
	// addParallel in Fusion; the signed half-turn here, so B cannot solve onto
	// the apex's other side (see bgTurnDeg).
	d.add("Apex->B parallel to centre->apex, pointing back toward the anchor line",
		sketch.NewAngle(centerToApex, drivingAxis, 180))

	proofkit.Step(t, "the Pinion Gear Shaft Axis, Apex->A, at the Shaft Angle")
	pAxS := d.pt(l.Apex, "Apex->A start")
	pA := keep("A", d.pt(l.Pinion.Shaft, "A"))
	pinionAxis := keepLine("pinionAxis", d.line(pAxS, pA, "Apex->A"))
	d.add("Apex->A starts at the Apex", sketch.NewCoincident(pAxS, apex))
	// Fusion's angular dimension is unsigned and picks its wedge from the text
	// point ([PB-ANGULAR-DIM]); the engine's angle is signed, counter-clockwise
	// from the first line to the second. The signed form pins the same side the
	// +X-most seed picks, and it is what keeps the probe from reporting the
	// mirrored configuration as a second discrete solution.
	// NewAngle reads its target in the sketch's default angle unit, degrees.
	d.add("Shaft Angle", sketch.NewAngle(drivingAxis, pinionAxis, in.ShaftAngle*180/math.Pi))

	proofkit.Step(t, "the two perpendicular drops to Apex 2, aimed into the interior wedge")
	aDropS := d.pt(l.Pinion.Shaft, "A->Apex2 start")
	apex2a := keep("Apex2", d.pt(l.Apex2, "Apex 2"))
	aDrop := keepLine("aDrop", d.line(aDropS, apex2a, "A->Apex2"))
	d.add("A->Apex2 starts at A", sketch.NewCoincident(aDropS, pA))
	// addPerpendicular in Fusion, with the drop aimed into the interior wedge —
	// toward the OTHER shaft axis, by the sign of its dot with A->B, never
	// "toward the anchor line". The signed turn is that aim written down.
	d.add("A->Apex2 perpendicular to Apex->A, into the interior wedge",
		sketch.NewAngle(pinionAxis, aDrop,
			bgQuarterTurn(l.Pinion.AxisDir, bgSub(l.Apex2, l.Pinion.Shaft))))
	d.add("A->Apex2 length = Pinion Gear Pitch Diameter / 2", sketch.NewDistance(aDropS, apex2a, l.PPD/2))

	bDropS := d.pt(l.Driving.Shaft, "B->Apex2 start")
	apex2b := d.pt(l.Apex2, "Apex 2 (driving drop)")
	bDrop := keepLine("bDrop", d.line(bDropS, apex2b, "B->Apex2"))
	d.add("B->Apex2 starts at B", sketch.NewCoincident(bDropS, pB))
	// The critical one. If this drop aims at the wrong side of the driving shaft
	// while A's aims correctly, the coincidence that closes them at Apex 2 flips
	// the whole frame to the mirror solution: A jumps sides, C collapses onto D,
	// the revolved frustum is degenerate and the conical end-cut reports
	// `face dist = inf` at the toe. Picking the sense by a "toward the anchor
	// line" reference is what does it — the driving shaft IS parallel to that
	// direction, so the test is degenerate.
	d.add("B->Apex2 perpendicular to Apex->B, into the interior wedge",
		sketch.NewAngle(drivingAxis, bDrop,
			bgQuarterTurn(l.Driving.AxisDir, bgSub(l.Apex2, l.Driving.Shaft))))
	d.add("B->Apex2 length = Driving Gear Pitch Diameter / 2", sketch.NewDistance(bDropS, apex2b, l.DPD/2))
	d.add("the two drops close at Apex 2", sketch.NewCoincident(apex2a, apex2b))

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	plS := d.pt(l.Apex, "Pitch Line start")
	plE := d.pt(l.Apex2, "Pitch Line end")
	pitchLine := keepLine("pitchLine", d.line(plS, plE, "Pitch Line"))
	d.add("Pitch Line starts at the Apex", sketch.NewCoincident(plS, apex))
	d.add("Pitch Line ends at Apex 2", sketch.NewCoincident(plE, apex2a))

	type sideDraw struct {
		g       bgMember
		ded     *sketch.Line
		dedEnd  *sketch.Point
		root    *sketch.Line
		axis    *sketch.Line
		shaft   *sketch.Point
		drop    *sketch.Line
		heel    *sketch.Point
		foot    *sketch.Point
		baseH   float64
		prefix  string
		dedName string
	}
	sides := []*sideDraw{
		{g: l.Pinion, axis: pinionAxis, shaft: pA, drop: aDrop, baseH: l.Pinion.BaseHeight, prefix: "Pinion", dedName: "C"},
		{g: l.Driving, axis: drivingAxis, shaft: pB, drop: bDrop, baseH: l.Driving.BaseHeight, prefix: "Driving", dedName: "D"},
	}

	for _, sd := range sides {
		dedS := d.pt(l.Apex2, sd.prefix+" Dedendum start")
		dedE := keep(sd.dedName, d.pt(sd.g.Ded, sd.dedName))
		ded := keepLine(sd.prefix+"Dedendum", d.line(dedS, dedE, sd.prefix+" Dedendum"))
		d.add(sd.prefix+" Dedendum starts at Apex 2", sketch.NewCoincident(dedS, apex2a))
		d.add(sd.prefix+" Dedendum perpendicular to the Pitch Line",
			sketch.NewAngle(pitchLine, ded, bgQuarterTurn(bgSub(l.Apex2, l.Apex), sd.g.DedDir)))
		d.add(sd.prefix+" Dedendum length = Module * 1.25",
			sketch.NewDistance(dedS, dedE, bgDedendumFactor*in.Module))
		sd.ded, sd.dedEnd = ded, dedE

		rootS := d.pt(l.Apex, sd.prefix+" Root Axis start")
		rootE := d.pt(sd.g.Ded, sd.prefix+" Root Axis end")
		root := keepLine(sd.prefix+"RootAxis", d.line(rootS, rootE, sd.prefix+" Root Axis"))
		d.add(sd.prefix+" Root Axis starts at the Apex", sketch.NewCoincident(rootS, apex))
		d.add(sd.prefix+" Root Axis ends at "+sd.dedName, sketch.NewCoincident(rootE, dedE))
		sd.root = root
	}

	proofkit.Step(t, "the module-length extensions and the base-height offsets")
	for i, sd := range sides {
		extName, heelName, footName := "E", "H", "G"
		if i == 1 {
			extName, heelName, footName = "F", "J", "I"
		}
		// A->E / B->F: collinear with the shaft axis. The engine's collinear
		// carries the same two point-on-line rows Fusion's does, and the row
		// pinning the start is already implied by the coincidence at A/B, so the
		// proof adds only the independent row ([PB-COLLINEAR-CHAIN] — the
		// substitution that rule says a proof cannot tell apart from the real
		// thing).
		aeS := d.pt(sd.g.Shaft, extName+" chain start")
		ext := keep(extName, d.pt(sd.g.Ext, extName))
		lineAE := keepLine("line"+extName, d.line(aeS, ext, sd.prefix+" module extension"))
		d.add(sd.prefix+" module extension starts at the shaft point", sketch.NewCoincident(aeS, sd.shaft))
		d.add(extName+" lies on the shaft axis", sketch.NewPointOnLine(ext, sd.axis))

		ceS := d.pt(sd.g.Ded, "dedendum corner -> "+extName+" start")
		ceE := d.pt(sd.g.Ext, "dedendum corner -> "+extName+" end")
		lineCE := d.line(ceS, ceE, sd.prefix+" dedendum -> "+extName)
		d.add(sd.prefix+" dedendum->"+extName+" starts at the dedendum corner", sketch.NewCoincident(ceS, sd.dedEnd))
		d.add(sd.prefix+" dedendum->"+extName+" ends at "+extName, sketch.NewCoincident(ceE, ext))
		d.add(sd.prefix+" dedendum->"+extName+" perpendicular to the module extension",
			sketch.NewPerpendicular(lineAE, lineCE))

		egS := d.pt(sd.g.Ext, footName+" chain start")
		foot := keep(footName, d.pt(sd.g.Foot, footName))
		keepLine("line"+footName, d.line(egS, foot, sd.prefix+" shaft extension"))
		d.add(sd.prefix+" shaft extension starts at "+extName, sketch.NewCoincident(egS, ext))
		d.add(footName+" lies on the module extension", sketch.NewPointOnLine(foot, lineAE))

		chS := d.pt(sd.g.Ded, heelName+" chain start")
		heel := keep(heelName, d.pt(sd.g.Heel, heelName))
		keepLine("line"+heelName, d.line(chS, heel, sd.prefix+" heel edge"))
		d.add(sd.prefix+" heel edge starts at the dedendum corner", sketch.NewCoincident(chS, sd.dedEnd))
		d.add(heelName+" lies on the "+sd.prefix+" Dedendum line", sketch.NewPointOnLine(heel, sd.ded))

		ghS := d.pt(sd.g.Foot, footName+"->"+heelName+" start")
		ghE := d.pt(sd.g.Heel, footName+"->"+heelName+" end")
		lineGH := keepLine("line"+footName+heelName, d.line(ghS, ghE, footName+"->"+heelName))
		d.add(footName+"->"+heelName+" starts at "+footName, sketch.NewCoincident(ghS, foot))
		d.add(footName+"->"+heelName+" ends at "+heelName, sketch.NewCoincident(ghE, heel))
		// Fusion needs a perpendicular here, because addOffsetDimension requires
		// the two lines to be parallel already; the engine's offset holds both
		// endpoints at the same signed distance and so carries the parallelism
		// itself. Adding the perpendicular as well is a third row over two
		// freedoms, and the engine reports the two base-height offsets as
		// redundant. instructions.md §2 states this in full and requires it be
		// left out here, never that the gate be weakened ([PB-NO-OVERCONSTRAIN]).
		d.add(sd.prefix+" base height offset", sketch.NewOffset(sd.drop, lineGH,
			bgOffsetValue(sd.g.Shaft, l.Apex2, sd.g.Foot, sd.baseH)))
		sd.heel, sd.foot = heel, foot
	}

	proofkit.Step(t, "close the figure: point I sits on the projected centre")
	// Fusion's addCoincident(I, projected centre) carries two rows, and this net
	// already implies one of them: I rides the driving shaft axis, which is the
	// same infinite line as centre->apex and therefore passes through the centre
	// whatever the apex height. The proof adds the single independent row.
	d.add("I lies on the projected anchor line", sketch.NewPointOnLine(out.point["I"], anchor))

	proofkit.Step(t, "the tooth-centre points K / L and the Tooth Spacing offset")
	for i, sd := range sides {
		ctrName, footName := "K", "G"
		if i == 1 {
			ctrName, footName = "L", "I"
		}
		gkS := d.pt(sd.g.Foot, footName+"->"+ctrName+" start")
		ctr := keep(ctrName, d.pt(sd.g.Center, ctrName))
		d.line(gkS, ctr, footName+"->"+ctrName)
		d.add(footName+"->"+ctrName+" starts at "+footName, sketch.NewCoincident(gkS, sd.foot))
		// K and L are the case where both ends are already fixed, so they take
		// two point-on-line coincidents and no collinear at all
		// ([BEVEL-F-COLLINEAR-CHAIN]).
		d.add(ctrName+" lies on the shaft axis", sketch.NewPointOnLine(ctr, sd.axis))
		d.add(ctrName+" lies on the "+sd.prefix+" Dedendum line", sketch.NewPointOnLine(ctr, sd.ded))

		toothCtr := ctr
		if in.ToothSpacing > 0 {
			ksS := d.pt(sd.g.Center, ctrName+"' start")
			toothCtr = d.pt(sd.g.ToothCtr, ctrName+"'")
			d.line(ksS, toothCtr, ctrName+"->"+ctrName+"'")
			d.add(ctrName+"->"+ctrName+"' starts at "+ctrName, sketch.NewCoincident(ksS, ctr))
			d.add(ctrName+"' lies on the "+sd.prefix+" Dedendum line", sketch.NewPointOnLine(toothCtr, sd.ded))
			d.add("Tooth Spacing on the "+sd.prefix+" side", sketch.NewDistance(ksS, toothCtr, in.ToothSpacing))
		}
		keep(ctrName+"'", toothCtr)

		// The tooth-centre reference line the tooth plane is built through:
		// dedendum corner -> K' / L'. At Tooth Spacing 0 this IS the C->K line,
		// drawn once ([BEVEL-F-LINE-ONCE]).
		ckS := d.pt(sd.g.Ded, sd.dedName+"->"+ctrName+"' start")
		ckE := d.pt(sd.g.ToothCtr, sd.dedName+"->"+ctrName+"' end")
		lineCK := keepLine(sd.prefix+"ToothCentre", d.line(ckS, ckE, sd.dedName+"->"+ctrName+"'"))
		d.add(sd.dedName+"->"+ctrName+"' starts at the dedendum corner", sketch.NewCoincident(ckS, sd.dedEnd))
		d.add(sd.dedName+"->"+ctrName+"' ends at the tooth centre", sketch.NewCoincident(ckE, toothCtr))
		_ = lineCK
	}

	proofkit.Step(t, "the toe lines M->N / O->P and the two front faces")
	for i, sd := range sides {
		toeName, innerName, frontName, footName := "M", "N", "A'", "G"
		if i == 1 {
			toeName, innerName, frontName, footName = "O", "P", "B'", "I"
		}
		// Seeded at the closed-form solved positions ([PB-SEED-NEAR]): M on
		// Apex->Ded at the fraction 1 - RootLength/|Apex->Ded|, then N slid from
		// that M seed along the heel-edge direction by
		// (perpendicular distance of the M seed from this gear's shaft axis
		//  - this gear's Toe Radius) / cos gamma.
		//
		// ⚠ THE PROOF CANNOT CATCH A WRONG SEED HERE. It seeds M and N at the
		// closed form, which IS the rule the module must follow, so what it
		// proves is that the constraints solve from a correct seed — never that
		// the module's seed is correct. The front face's length dimension is
		// unsigned, so the toe line meets the Toe Radius on BOTH sides of the
		// shaft axis and the solver takes whichever side the seed starts on; a
		// seed below the axis converges happily onto the mirror, and the revolve
		// several steps later fails with ASM_WIRE_X_AXIS naming itself rather
		// than the seed. That defect reaches Fusion untested. This comment is
		// the honest edge of what this stage checks.
		toeS := keep(toeName, d.pt(sd.g.Toe, toeName))
		toeE := keep(innerName, d.pt(sd.g.ToeInner, innerName))
		toeLine := keepLine("line"+toeName+innerName, d.line(toeS, toeE, toeName+"->"+innerName))
		d.add(toeName+" lies on the "+sd.prefix+" Root Axis", sketch.NewPointOnLine(toeS, sd.root))
		// Fusion draws this line at an arbitrary angle and needs
		// addParallel(toe line, heel edge) before addOffsetDimension will take
		// it ([PB-OFFSET-DIM]). The engine's offset holds both endpoints and
		// therefore already carries the parallelism, so the parallel is left out
		// for the same reason the G->H perpendicular is.
		heelEdge := out.line["line"+map[bool]string{true: "J", false: "H"}[i == 1]]
		d.add(sd.prefix+" root length, perpendicular to the pitch line",
			sketch.NewOffset(heelEdge, toeLine,
				bgOffsetValue(sd.g.Ded, sd.g.Heel, sd.g.Toe, l.RootLength*l.R/l.ApexToDed)))

		// The front face is what holds N off the shaft axis. N is NEVER pinned
		// to the shaft axis: that would put it ON the axis of revolution and the
		// later conical split fails with ASM_API_FAILED for asymmetric tooth
		// counts. Only the foot A'/B' touches the axis.
		faceS := d.pt(sd.g.ToeInner, frontName+" face start")
		front := keep(frontName, d.pt(sd.g.FrontFoot, frontName))
		faceLine := keepLine("front"+frontName, d.line(faceS, front, innerName+"->"+frontName))
		d.add(innerName+"->"+frontName+" starts at "+innerName, sketch.NewCoincident(faceS, toeE))
		d.add(frontName+" lies on the shaft axis", sketch.NewPointOnLine(front, sd.axis))
		d.add(innerName+"->"+frontName+" stands square to the shaft",
			sketch.NewAngle(sd.axis, faceLine,
				bgQuarterTurn(sd.g.AxisDir, bgSub(sd.g.FrontFoot, sd.g.ToeInner))))
		d.add(sd.prefix+" Gear Toe Radius", sketch.NewDistance(faceS, front, sd.g.ToeRadius))

		// The two short reference lines: toe -> dedendum corner, and the
		// hexagon's shaft-axis edge front foot -> G/I.
		mcS := d.pt(sd.g.Toe, toeName+"->"+sd.dedName+" start")
		mcE := d.pt(sd.g.Ded, toeName+"->"+sd.dedName+" end")
		d.line(mcS, mcE, toeName+"->"+sd.dedName)
		d.add(toeName+"->"+sd.dedName+" starts at "+toeName, sketch.NewCoincident(mcS, toeS))
		d.add(toeName+"->"+sd.dedName+" ends at the dedendum corner", sketch.NewCoincident(mcE, sd.dedEnd))

		agS := d.pt(sd.g.FrontFoot, frontName+"->"+footName+" start")
		agE := d.pt(sd.g.Foot, frontName+"->"+footName+" end")
		d.line(agS, agE, frontName+"->"+footName)
		d.add(frontName+"->"+footName+" starts at "+frontName, sketch.NewCoincident(agS, front))
		d.add(frontName+"->"+footName+" ends at "+footName, sketch.NewCoincident(agE, sd.foot))
	}
	return out
}

// assertGearProfiles is not a proofkit hook — proofkit.Run gates a sketch and
// takes no assertion — so stepGearProfiles calls its checks inline through
// bgAssertLattice, which the per-gear steps reuse.
func bgAssertLattice(t testing.TB, out *bgLattice2D) {
	l := out.lat
	at := func(name string) bgPt {
		q := out.point[name]
		return bgPt{q.X(), q.Y()}
	}
	const tol = 1e-6
	for name, want := range map[string]bgPt{
		"Apex": l.Apex, "Apex2": l.Apex2,
		"A": l.Pinion.Shaft, "B": l.Driving.Shaft,
		"C": l.Pinion.Ded, "D": l.Driving.Ded,
		"E": l.Pinion.Ext, "F": l.Driving.Ext,
		"G": l.Pinion.Foot, "I": l.Driving.Foot,
		"H": l.Pinion.Heel, "J": l.Driving.Heel,
		"K": l.Pinion.Center, "L": l.Driving.Center,
		"K'": l.Pinion.ToothCtr, "L'": l.Driving.ToothCtr,
		"M": l.Pinion.Toe, "O": l.Driving.Toe,
		"N": l.Pinion.ToeInner, "P": l.Driving.ToeInner,
		"A'": l.Pinion.FrontFoot, "B'": l.Driving.FrontFoot,
	} {
		got := at(name)
		if math.Hypot(got.X-want.X, got.Y-want.Y) > tol {
			t.Errorf("%s solved to (%.6f, %.6f), closed form gives (%.6f, %.6f)",
				name, got.X, got.Y, want.X, want.Y)
		}
	}

	// The cone angles the whole build reads come out of this net, so read them
	// back off the solved figure rather than trusting the seed.
	apex, apex2 := at("Apex"), at("Apex2")
	pitch := bgSub(apex2, apex)
	bgCloseTB(t, "Pitch Cone Distance R", bgLen(pitch), l.R, 1e-6)
	bgCloseTB(t, "gamma_p from the solved figure",
		math.Abs(math.Atan2(bgCross(pitch, bgSub(at("A"), apex)), bgDot(pitch, bgSub(at("A"), apex)))),
		l.GammaP, 1e-9)
	bgCloseTB(t, "gamma_g from the solved figure",
		math.Abs(math.Atan2(bgCross(pitch, bgSub(at("B"), apex)), bgDot(pitch, bgSub(at("B"), apex)))),
		l.GammaG, 1e-9)

	// Point I closes on the projected centre, which is what fixes the apex
	// height at R cos gamma_g + the resolved Driving Gear Base Height.
	bgCloseTB(t, "I closes on the projected centre", bgLen(bgSub(at("I"), l.Center)), 0, 1e-6)
	bgCloseTB(t, "apex height above the anchor line",
		bgDot(bgSub(apex, l.Center), l.Perp), l.R*math.Cos(l.GammaG)+l.Driving.BaseHeight, 1e-6)

	// The Maximum Face Width is read off SOLVED geometry, never the seeds
	// ([PB-SOLVED-GEOMETRY]), and takes the smaller of the two sides — which is
	// not always the pinion's.
	dp := bgPointLine(at("A"), at("C"), at("H"))
	dg := bgPointLine(at("B"), at("D"), at("J"))
	bgCloseTB(t, "Maximum Face Width", bgBoundFactor*math.Min(dp, dg), l.MaxFaceWidth, 1e-6)
	if l.FaceWidth > l.MaxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.6f exceeds the Maximum Face Width %.6f", l.FaceWidth, l.MaxFaceWidth)
	}

	for _, g := range []bgMember{l.Pinion, l.Driving} {
		toe, inner, front := g.Toe, g.ToeInner, g.FrontFoot
		if g.Label == "Pinion" {
			toe, inner, front = at("M"), at("N"), at("A'")
		} else {
			toe, inner, front = at("O"), at("P"), at("B'")
		}
		// The inner toe corner rides the Toe Radius, and only the front face's
		// foot touches the axis.
		bgCloseTB(t, g.Label+" inner toe corner radius", l.bgRadius(g, inner), g.ToeRadius, 1e-6)
		bgCloseTB(t, g.Label+" front face foot is on the axis", l.bgRadius(g, front), 0, 1e-6)
		if l.bgRadius(g, inner) <= 0 {
			t.Errorf("%s: the inner toe corner reached the shaft axis", g.Label)
		}
		// The toe end is nearer the apex than the heel end, which is what the
		// spiral frame's span guard later depends on.
		if l.bgConeDistance(g, toe) >= l.bgConeDistance(g, g.Ded) {
			t.Errorf("%s: the toe is not inside the heel (toe %.4f, heel %.4f)",
				g.Label, l.bgConeDistance(g, toe), l.bgConeDistance(g, g.Ded))
		}
		bgCloseTB(t, g.Label+" root length |Ded->Toe|", bgLen(bgSub(g.Ded, toe)), l.RootLength, 1e-6)
		// The toe radius stays strictly below this gear's ceiling, or the
		// extension has nowhere to go.
		if l.In.ToeExtension > 0 && g.ToeRadius >= g.ToeRadiusCeiling {
			t.Errorf("%s: Toe Radius %.6f is at or above the Toe Radius Ceiling %.6f, "+
				"so a Toe Extension above 0 must be rejected", g.Label, g.ToeRadius, g.ToeRadiusCeiling)
		}
		// Both base-height bounds hold, and the window they leave is non-empty,
		// which is exactly the Minimum Teeth check.
		if g.BaseHeight < g.MinBaseHeight-1e-9 || g.BaseHeight > g.MaxBaseHeight+1e-9 {
			t.Errorf("%s: resolved base height %.6f outside [%.6f, %.6f]",
				g.Label, g.BaseHeight, g.MinBaseHeight, g.MaxBaseHeight)
		}
		if g.Teeth < g.MinTeeth {
			t.Errorf("%s: %v teeth is below the computed floor %.4f", g.Label, g.Teeth, g.MinTeeth)
		}
		// The heel edge runs outward from the dedendum corner rather than back
		// inward, which is the low-tooth-count failure the Minimum Base Height
		// exists to stop.
		heel := at("H")
		if g.Label == "Driving" {
			heel = at("J")
		}
		if bgDot(bgSub(heel, g.Ded), g.DedDir) <= 0 {
			t.Errorf("%s: the heel point landed behind the dedendum corner", g.Label)
		}
	}
}

// ----------------------------------------------------------------------------
// S07 / S09 / S10 — the per-gear construction planes and the tooth axis.
//
// A Fusion construction plane or axis has no counterpart on the bench, so these
// three steps build the same frames in the sketch engine's World and read them
// back, with the in-plane consequence drawn in the harness's own sketch so the
// gate has something to judge. The cost is that Fusion's setByAngle,
// setByDistanceOnPath and setByTwoPlanes are not themselves exercised; what is
// proved is the geometry each of those calls has to produce.

var toothPlaneCases = bgPerGearCases()
var toothAxisHelperPlaneCases = bgPerGearCases()
var toothAxisCases = bgPerGearCases()

// bgPerGearCases is the table every per-gear step shares: both sides of the
// pair, both ways round the ratio, both ends of the spacing and toe windows,
// and target planes tilted out of world XY.
func bgPerGearCases() []proofkit.Case {
	var out []proofkit.Case
	base := []struct {
		name string
		p    map[string]float64
	}{
		{"default_31_31_90", map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 90}},
		{"ratio_31_17_90", map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 17, idShaftAngle: 90}},
		{"ratio_17_31_90", map[string]float64{idModule: 1, idDrivingTeeth: 17, idPinionTeeth: 31, idShaftAngle: 90}},
		{"low_teeth_4_4_90", map[string]float64{idModule: 1, idDrivingTeeth: 4, idPinionTeeth: 4, idShaftAngle: 90}},
		// Both ends of the virtual tooth count's own range. 16/12 at 90 degrees
		// gives the PINION a virtual count of exactly 15, so a floored count and
		// an exact one agree there and only the driving side's 26.667 shows the
		// difference — which is what makes the pair reject a rounded count rather
		// than merely disagree with one. 4/4 carries the largest root-corner
		// float of any admitted pair, 0.027 module, so it is what the root sink
		// has to clear.
		{"integer_zv_16_12_90", map[string]float64{idModule: 4, idDrivingTeeth: 16, idPinionTeeth: 12, idShaftAngle: 90}},
		{"shaft_35", map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 35}},
		{"shaft_142", map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 31, idShaftAngle: 142}},
		{"spacing_and_toe_extension", map[string]float64{idModule: 1, idDrivingTeeth: 31, idPinionTeeth: 17,
			idShaftAngle: 75, idToothSpacing: 0.4, idToeExtension: 60}},
		{"module_8_19_13_60", map[string]float64{idModule: 8, idDrivingTeeth: 19, idPinionTeeth: 13,
			idShaftAngle: 60, idToeExtension: 100}},
	}
	for _, b := range base {
		for _, side := range []struct {
			name string
			v    float64
		}{{"pinion", 0}, {"driving", 1}} {
			p := map[string]float64{idSide: side.v}
			for k, v := range b.p {
				p[k] = v
			}
			if side.v == 1 {
				p[bgTiltKey] = 40
			}
			out = append(out, proofkit.Case{Name: b.name + "_" + side.name, Params: p})
		}
	}
	return out
}

// bgPlaneWorld rebuilds the Gear Profiles plane in the World, tilted per the
// case, and maps a lattice 2-D point into world space on it. The lattice's own
// frame is the plane's (U, V), which is what [BEVEL-F-APEX-LOCAL] guarantees.
func bgPlaneWorld(t testing.TB, s *sketch.Sketch, tiltDeg float64) (r3.Frame, func(bgPt) r3.Vec) {
	t.Helper()
	tilt := tiltDeg * math.Pi / 180
	u := r3.NewVec(1, 0, 0)
	v := r3.NewVec(0, math.Cos(tilt), math.Sin(tilt))
	plane, err := s.World().CreatePlaneFromPoints(r3.NewVec(0, 0, 0), u, v)
	if err != nil {
		t.Fatalf("gear profiles plane: %v", err)
	}
	frame, err := plane.Frame()
	if err != nil {
		t.Fatalf("gear profiles frame: %v", err)
	}
	return frame, func(p bgPt) r3.Vec { return frame.ToWorldUV(p.X, p.Y) }
}

// bgToothCentreLine draws the tooth-centre reference line C->K' / D->L' in the
// harness sketch, on recreated fixed endpoints, and hands back the line.
func bgToothCentreLine(t testing.TB, s *sketch.Sketch, g bgMember) (*sketch.Point, *sketch.Point) {
	t.Helper()
	d := &bgDraw{t: t, s: s}
	ded := s.CreateReferencePoint(g.Ded.X, g.Ded.Y, "§2 dedendum corner")
	ctr := s.CreateReferencePoint(g.ToothCtr.X, g.ToothCtr.Y, "§2 tooth centre")
	a := d.pt(g.Ded, g.Label+" tooth-centre line start")
	b := d.pt(g.ToothCtr, g.Label+" tooth centre")
	d.line(a, b, g.Label+" tooth-centre reference line")
	d.add(g.Label+" tooth-centre line starts at the dedendum corner", sketch.NewCoincident(a, ded))
	d.add(g.Label+" tooth-centre line ends at the tooth centre", sketch.NewCoincident(b, ctr))
	return a, b
}

// stepToothPlane builds `{gearLabel} Plane`: the plane through the tooth-centre
// reference line, made perpendicular to the Gear Profiles sketch plane with
// setByAngle. The sketch line is passed to setByAngle DIRECTLY, never wrapped
// in Path.create first ([PB-CONSTRUCTION-PLANES]).
func stepToothPlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	frame, toWorld := bgPlaneWorld(t, s, bgGet(p, bgTiltKey, 0))

	proofkit.Step(t, "%s Plane: through %s->tooth centre, 90 deg off the Gear Profiles plane", g.Label, g.Label)
	dedW, ctrW := toWorld(g.Ded), toWorld(g.ToothCtr)
	toothPlane, err := s.World().CreatePlaneFromPoints(dedW, ctrW, dedW.Add(frame.N()))
	if err != nil {
		t.Fatalf("%s Plane: %v", g.Label, err)
	}
	tf, err := toothPlane.Frame()
	if err != nil {
		t.Fatalf("%s Plane frame: %v", g.Label, err)
	}
	bgCloseTB(t, g.Label+" Plane contains the tooth-centre reference line",
		tf.N().Dot(ctrW.Sub(dedW)), 0, 1e-9)
	bgCloseTB(t, g.Label+" Plane is perpendicular to the Gear Profiles plane",
		tf.N().Dot(frame.N()), 0, 1e-12)
	bgCloseTB(t, g.Label+" tooth centre lies in the "+g.Label+" Plane",
		tf.N().Dot(ctrW.Sub(tf.Origin())), 0, 1e-9)

	from, to := bgToothCentreLine(t, s, g)
	// The reference line runs along the back-cone direction, which is what puts
	// the tooth on the back cone at all (the Tredgold construction).
	got := bgUnit(bgPt{to.X() - from.X(), to.Y() - from.Y()})
	bgCloseTB(t, g.Label+" tooth-centre line runs along the dedendum direction",
		bgCross(got, g.DedDir), 0, 1e-9)
}

// stepToothAxisHelperPlane builds the helper plane
// `setByDistanceOnPath(<tooth-centre reference line>, 1.0)` — perpendicular to
// that line at its FAR end, the tooth-centre point. It is one half of the pair
// the tooth axis is the intersection of.
func stepToothAxisHelperPlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	_, toWorld := bgPlaneWorld(t, s, bgGet(p, bgTiltKey, 0))

	proofkit.Step(t, "helper plane, normal to %s->tooth centre at distance 1.0 along it", g.Label)
	dedW, ctrW := toWorld(g.Ded), toWorld(g.ToothCtr)
	along, ok := ctrW.Sub(dedW).Normalize()
	if !ok {
		t.Fatalf("%s: the tooth-centre reference line is degenerate", g.Label)
	}
	// Any two independent directions perpendicular to `along` span the plane.
	seed := r3.NewVec(0, 0, 1)
	if math.Abs(seed.Dot(along)) > 0.9 {
		seed = r3.NewVec(1, 0, 0)
	}
	e1, _ := seed.Sub(along.Scale(seed.Dot(along))).Normalize()
	e2 := along.Cross(e1)
	helper, err := s.World().CreatePlaneFromPoints(ctrW, ctrW.Add(e1), ctrW.Add(e2))
	if err != nil {
		t.Fatalf("helper plane: %v", err)
	}
	hf, err := helper.Frame()
	if err != nil {
		t.Fatalf("helper plane frame: %v", err)
	}
	bgCloseTB(t, "helper plane is normal to the tooth-centre reference line",
		math.Abs(hf.N().Dot(along))-1, 0, 1e-12)
	bgCloseTB(t, "helper plane passes through the tooth centre at distance 1.0",
		hf.N().Dot(ctrW.Sub(hf.Origin())), 0, 1e-9)

	bgToothCentreLine(t, s, g)
}

// stepToothAxis builds `{gearLabel} Tooth Axis` as the intersection of the Gear
// Profiles plane and that helper plane ([PB-CONSTRUCTION-AXES] —
// setByPerpendicularAtPoint would need a BRepFace this build does not have).
// The intersection is the line through the tooth centre normal to the tooth
// plane, which is what the tooth profile is drawn on.
func stepToothAxis(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	frame, toWorld := bgPlaneWorld(t, s, bgGet(p, bgTiltKey, 0))

	dedW, ctrW := toWorld(g.Ded), toWorld(g.ToothCtr)
	along, _ := ctrW.Sub(dedW).Normalize()

	proofkit.Step(t, "%s Tooth Axis = Gear Profiles plane x helper plane", g.Label)
	axis, ok := frame.N().Cross(along).Normalize()
	if !ok {
		t.Fatalf("%s Tooth Axis: the two planes are parallel", g.Label)
	}
	// It runs through the tooth centre, lies in the Gear Profiles plane, and
	// stands normal to the tooth plane — whose own normal is
	// along x (gear profiles normal).
	toothNormal, _ := along.Cross(frame.N()).Normalize()
	bgCloseTB(t, g.Label+" Tooth Axis lies in the Gear Profiles plane", axis.Dot(frame.N()), 0, 1e-12)
	bgCloseTB(t, g.Label+" Tooth Axis is normal to the tooth plane",
		math.Abs(axis.Dot(toothNormal))-1, 0, 1e-12)
	bgCloseTB(t, g.Label+" Tooth Axis is perpendicular to the tooth-centre line",
		axis.Dot(along), 0, 1e-12)

	bgToothCentreLine(t, s, g)
}

// ----------------------------------------------------------------------------
// S08 — the virtual spur tooth profile.

var toothProfileCases = bgPerGearCases()

// stepToothProfile draws the `{gearLabel} Tooth` sketch.
//
// Substitution, and what it costs. The tooth itself is drawn by the BORROWED
// spur generator — `SpurGearInvoluteToothDesignGenerator(sketch, proxy).draw(
// anchorPoint, angle=math.radians(180))` — and its involute flanks, ribs, spine
// and tooth-top arc are spur's geometry, proved in
// proof/spurgear/sketches_test.go. What BEVEL supplies is the four things this
// step builds and checks: the virtual (back-cone, Tredgold) tooth number, the
// module, the tooth centre K'/L', and the 180° draw angle that turns the tooth
// to face the dedendum corner. So the step builds the four circles that carry
// those, centred on the tooth centre, plus the spine at 180°, and leaves the
// flanks to spur's own proof. The cost is that a wrong involute would not be
// seen here; a wrong virtual tooth count, a wrong centre or a tooth facing the
// wrong way would.
func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	d := &bgDraw{t: t, s: s}

	// The virtual tooth number comes from the closed form, never from measuring
	// Apex2->K'. Units: the module is the raw mm number, the pitch diameter is
	// the same length, so nothing is converted twice here — the ×10 the
	// generated module needs is the cm->mm step that this proof does not have.
	//
	// It is a REAL number and is never rounded. Rounding it would rebuild every
	// drawn circle from the rounded count, which is what drew the tooth smaller
	// than the back cone places it (issue #155).
	vr := (g.PitchDia / 2) / math.Cos(g.Gamma)
	vt := 2 * vr / in.Module
	bgCloseTB(t, g.Label+" virtual pitch radius", vr, g.VirtualPitchRadius, 1e-9)
	bgCloseTB(t, g.Label+" virtual tooth number", vt, g.VirtualTeeth, 1e-12)
	if vt < 3 {
		proofkit.Unmodelled(t, "%s: virtual tooth number %v leaves no spur tooth to draw", g.Label, vt)
	}

	sink := g.RootSink
	pitchR := in.Module * vt / 2
	baseR := pitchR * math.Cos(bgPressureAngle)
	rootR := (in.Module*vt-2*bgDedendumFactor*in.Module)/2 - sink
	tipR := (in.Module*vt + 2*bgAddendumFactor*in.Module) / 2

	proofkit.Step(t, "%s Tooth: the four circles, centred on the tooth centre", g.Label)
	anchor := s.CreateReferencePoint(0, 0, "projected tooth centre")
	centre := d.pt(bgPt{0, 0}, g.Label+" tooth centre")
	d.add("the tooth centre is the projected K'/L'", sketch.NewCoincident(centre, anchor))
	circle := func(name string, r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(centre, r)
		c.SetConstruction(construction)
		d.add(name+" diameter", sketch.NewDiameter(c, 2*r))
		return c
	}
	circle("Root Circle", rootR, false)
	tip := circle("Tip Circle", tipR, true)
	circle("Base Circle", baseR, true)
	circle("Pitch Circle", pitchR, true)

	proofkit.Step(t, "%s Tooth: the spine, drawn already rotated 180 deg", g.Label)
	// The tooth plane's own +X runs from the dedendum corner toward the tooth
	// centre, so a tooth drawn at 180° points back at the dedendum corner. That
	// rotation is delivered through draw()'s angle argument, never by rotating
	// the sketch afterwards.
	ra := s.CreateReferencePoint(0, 0, "tooth plane origin")
	rb := s.CreateReferencePoint(10, 0, "tooth plane +X")
	refLine, err := s.CreateReferenceLine(ra, rb, "projected tooth-centre reference line")
	if err != nil {
		t.Fatalf("tooth plane reference line: %v", err)
	}
	spineStart := d.pt(bgPt{0, 0}, "spine start")
	top := d.pt(bgPt{-tipR, 0}, "tooth-top point")
	spine := d.line(spineStart, top, g.Label+" tooth spine")
	d.add("the spine starts at the tooth centre", sketch.NewCoincident(spineStart, centre))
	d.add("the tooth top sits on the Tip Circle", sketch.NewPointOnCircle(top, tip))
	d.add("the tooth is drawn at 180 deg", sketch.NewAngle(refLine, spine, 180))

	bgCloseTB(t, g.Label+" tooth-top radius", math.Hypot(top.X(), top.Y()), tipR, 1e-9)
	if top.X() >= 0 {
		t.Errorf("%s: the tooth faces away from the dedendum corner (tooth top at x=%.4f)", g.Label, top.X())
	}

	// The Tredgold construction, circle by circle. The tooth is drawn AT the back
	// cone, so the drawn pitch circle has to reach exactly as far as the back-cone
	// point K/L the §2 lattice put down — that is the reading the floored count
	// used to miss by up to half a module. Tooth Spacing moves the centre to K'/L'
	// and leaves the tooth its size, so the comparison is against K/L.
	bgCloseTB(t, g.Label+" the drawn pitch circle reaches the back-cone point",
		pitchR, bgLen(bgSub(g.Center, l.Apex2)), 1e-9)
	bgCloseTB(t, g.Label+" dedendum corner is one dedendum inside the virtual pitch radius",
		bgLen(bgSub(g.Center, g.Ded)), vr-bgDedendumFactor*in.Module, 1e-9)

	// The root circle is one ROOT SINK inside the dedendum corner: not at it,
	// which leaves the root arc's corners outside the gear body's root cone, and
	// not further, which would cut into the body for no reason. Both sides.
	bgCloseTB(t, g.Label+" the drawn root circle sits one root sink inside the dedendum corner",
		rootR, vr-bgDedendumFactor*in.Module-sink, 1e-9)
	bgCloseTB(t, g.Label+" the drawn tip circle stands one module outside the back cone",
		tipR, vr+bgAddendumFactor*in.Module, 1e-9)

	// And the tooth the drawer places on those circles is the NOMINAL tooth: its
	// thickness at the drawn pitch circle is pi * Module / 2. The angle comes
	// from the drawer's own placement — it rotates the flank so the pitch
	// crossing lands at pi/(2 z_v) — read here through the same involute the
	// drawer samples, never restated as the identity it is meant to check. This
	// is the reading that rejects an INTEGER virtual tooth count drawn at the
	// exact radius: that variant makes the thickness pi * r_v / round(z_v), which
	// misses nominal by a different amount on each member of an unequal pair.
	px, py, ok := involute.Point(baseR, pitchR)
	if !ok {
		t.Fatalf("%s: the pitch circle falls inside the base circle", g.Label)
	}
	lx, ly := involute.Rotate(px, -py, math.Pi/(2*vt)-math.Atan2(-py, px))
	bgCloseTB(t, g.Label+" tooth thickness at the drawn pitch circle",
		2*math.Atan2(ly, lx)*pitchR, math.Pi*in.Module/2, 1e-9)

	// The embedded flag decides the tooth loop's line count, and the selection
	// is `wantLines = 0 if embedded else 2` — never "0 or 2", which grabs an
	// unrelated loop and kills the apex->tooth loft with LOFT_NO_TOOLBODY.
	embedded := baseR < rootR
	if embedded != g.Embedded {
		t.Errorf("%s embedded flag: got %v, want %v", g.Label, embedded, g.Embedded)
	}
	want := 2
	if embedded {
		want = 0
	}
	if want != bgWantLines(g.Embedded) {
		t.Errorf("%s: find_profile_by_curve_counts line count: got %d, want %d",
			g.Label, bgWantLines(g.Embedded), want)
	}
}

// ----------------------------------------------------------------------------
// S11 — the per-gear Profile sketch.

var gearProfileSketchCases = bgPerGearCases()

// stepGearProfileSketch draws `{gearLabel} Profile`: one fresh sketch on the
// axial plane holding exactly this gear's hexagon, so sketch.profiles holds
// exactly one loop ([PB-SINGLE-PROFILE]).
//
// The six §2 vertices are RECREATED as new points at their exact positions and
// the lines are drawn SHARING them, then the endpoints are fixed AFTER the
// lines exist — the [PB-PROJECT-NOT-FIXED] recreate-share-fix recipe, in that
// order. Fixing a bare point before it is consumed as a line endpoint does not
// leave the sketch fully constrained, and projecting the §2 points instead
// leaves them associative and free.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()

	proofkit.Step(t, "%s Profile: recreate the six §2 vertices, draw the hexagon, then fix", g.Label)
	names := []string{"A'", "G", "H", "C", "M", "N"}
	if g.Label == "Driving" {
		names = []string{"B'", "I", "J", "D", "O", "P"}
	}
	verts := l.bgHexagon(g)
	pts := make([]*sketch.Point, len(verts))
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.X, v.Y)
		pts[i].SetName(g.Label + " " + names[i])
	}
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	// The order is the recipe's: the lines exist first, and only then are their
	// endpoints fixed.
	_ = lines
	for _, q := range pts {
		s.Fix(q)
	}

	proofkit.Step(t, "%s Profile: the hexagon and its shaft-axis edge", g.Label)
	for i, v := range verts {
		bgCloseTB(t, g.Label+" "+names[i]+" x", pts[i].X(), v.X, 1e-9)
		bgCloseTB(t, g.Label+" "+names[i]+" y", pts[i].Y(), v.Y, 1e-9)
	}
	// The hexagon's FIRST edge is the shaft axis for the revolve, the pattern,
	// the bore plane and the meshing rotation — not the §2 Apex->A / Apex->B
	// construction line, which lives in a different sketch.
	bgCloseTB(t, g.Label+" shaft-axis edge start is on the axis", l.bgRadius(g, verts[0]), 0, 1e-9)
	bgCloseTB(t, g.Label+" shaft-axis edge end is on the axis", l.bgRadius(g, verts[1]), 0, 1e-9)
	// Nothing else may touch the axis, or the revolve aborts with
	// ASM_WIRE_X_AXIS ([PB-REVOLVE]).
	for i := 2; i < len(verts); i++ {
		if l.bgRadius(g, verts[i]) <= 1e-9 {
			t.Errorf("%s: hexagon vertex %s reached the axis of revolution", g.Label, names[i])
		}
	}
	if n := len(s.Profiles()); n != 1 {
		t.Errorf("%s Profile holds %d profiles, want exactly 1", g.Label, n)
	}
	for _, prof := range s.Profiles() {
		if !prof.Valid || prof.SelfIntersecting {
			t.Errorf("%s Profile is not an extrudable loop: valid=%v selfIntersecting=%v",
				g.Label, prof.Valid, prof.SelfIntersecting)
		}
	}
}

// ----------------------------------------------------------------------------
// S28 / S29 — the bore plane and the Bore sketch.

var borePlaneCases = bgPerGearCases()
var boreSketchCases = bgPerGearCases()

// stepBorePlane builds the bore plane: normal to the shaft at its start, through
// `setByDistanceOnPath(<shaft-axis edge>, 0.0)` — the IN-SKETCH profile edge
// A'->G / B'->I, never the §2 Apex->A / Apex->B construction line, which lives
// in a different sketch and fails or misbuilds.
//
// Substitution: as with the other planes, the frame is built in the sketch
// engine's World and read back; Fusion's setByDistanceOnPath is not itself
// exercised.
func stepBorePlane(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	frame, toWorld := bgPlaneWorld(t, s, bgGet(p, bgTiltKey, 0))

	proofkit.Step(t, "%s bore plane: normal to the shaft-axis edge at distance 0.0 along it", g.Label)
	startW, endW := toWorld(g.FrontFoot), toWorld(g.Foot)
	along, ok := endW.Sub(startW).Normalize()
	if !ok {
		t.Fatalf("%s: the shaft-axis edge is degenerate", g.Label)
	}
	seed := frame.N()
	e1, _ := seed.Sub(along.Scale(seed.Dot(along))).Normalize()
	e2 := along.Cross(e1)
	plane, err := s.World().CreatePlaneFromPoints(startW, startW.Add(e1), startW.Add(e2))
	if err != nil {
		t.Fatalf("%s bore plane: %v", g.Label, err)
	}
	bf, err := plane.Frame()
	if err != nil {
		t.Fatalf("%s bore plane frame: %v", g.Label, err)
	}
	bgCloseTB(t, "the bore plane is normal to the shaft axis", math.Abs(bf.N().Dot(along))-1, 0, 1e-12)
	// Distance 0.0 along the path is the edge's START, so the plane is rooted at
	// the shaft start and the sketch origin therefore sits ON the axis — which is
	// what lets the bore circle be centred on the origin.
	bgCloseTB(t, "the bore plane passes through the shaft edge's start",
		bf.N().Dot(startW.Sub(bf.Origin())), 0, 1e-9)
	bgCloseTB(t, "the shaft edge's start is the hexagon's front foot, on the axis",
		l.bgRadius(g, g.FrontFoot), 0, 1e-9)

	// The in-plane consequence, drawn so the gate has something to judge: the
	// shaft-axis edge itself, on recreated fixed endpoints.
	d := &bgDraw{t: t, s: s}
	front := s.CreateReferencePoint(g.FrontFoot.X, g.FrontFoot.Y, "§2 front foot")
	foot := s.CreateReferencePoint(g.Foot.X, g.Foot.Y, "§2 G / I")
	a := d.pt(g.FrontFoot, g.Label+" shaft-axis edge start")
	b := d.pt(g.Foot, g.Label+" shaft-axis edge end")
	d.line(a, b, g.Label+" shaft-axis edge")
	d.add("the shaft-axis edge starts at the front foot", sketch.NewCoincident(a, front))
	d.add("the shaft-axis edge ends at G / I", sketch.NewCoincident(b, foot))
}

// stepBoreSketch draws `{gearLabel} Bore`: the bore circle centred at the sketch
// origin, its centre FIXED and a diameter dimension set to the bore diameter
// ([PB-CIRCLE-CENTER] — a circle's centre is free even when created at the
// origin, and coincidenting it to sketch.originPoint throws
// VCS_SKETCH_SOLVING_FAILED on a setByDistanceOnPath plane).
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	in := bgRead(p)
	l := bgSolve(in)
	g := l.bgSide()
	if !in.BoreEnable {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so no Bore sketch is authored at all")
	}
	proofkit.Step(t, "%s Bore: the bore circle on the shaft-start plane", g.Label)
	centre := s.CreatePoint(0, 0)
	centre.SetName(g.Label + " bore centre")
	circle := s.CreateCircle(centre, g.BoreDia/2)
	s.Fix(centre)
	s.AddConstraint(sketch.NewDiameter(circle, g.BoreDia))

	want := g.PitchDia / 4
	if in.Driving && in.DrivingBore > 0 {
		want = in.DrivingBore
	}
	if !in.Driving && in.PinionBore > 0 {
		want = in.PinionBore
	}
	bgCloseTB(t, g.Label+" bore diameter", 2*circle.R(), want, 1e-9)
	// ⚠ SPEC GAP, RECORDED RATHER THAN ASSERTED. Nothing in the spec bounds the
	// bore diameter against the gear body it pierces, and the auto value
	// `this gear's Pitch Diameter / 4` can exceed the body's own heel radius: at
	// Module 1, 31/31 teeth and Shaft Angle 35 deg the auto bore radius is
	// 3.8750 mm against a heel radius of 3.2101 mm, so the through-cut would
	// take the whole blank away. The spec admits that configuration, so this
	// step records the reading instead of failing on it.
	if g.BoreDia/2 >= l.bgRadius(g, g.Heel) {
		t.Logf("%s: the auto bore radius %.4f reaches past the heel radius %.4f — "+
			"no bound in the spec stops it", g.Label, g.BoreDia/2, l.bgRadius(g, g.Heel))
	}
}
