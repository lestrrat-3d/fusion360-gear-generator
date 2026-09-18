// This file holds bevel's sketch steps: the Anchor sketch, the §2 Gear Profiles
// lattice, the per-gear virtual-spur Tooth sketch, the per-gear Profile sketch,
// the Bore sketch, and the spiral build's 2-D cutter-arc trace.
//
// Four of those are permanent sketches the generated module gates on
// isFullyConstrained ([BEVEL-F-FULL-CONSTRAINT]); proofkit's own gate is
// stricter, so every one of them is held to DOF 0 with nothing redundant,
// nothing conflicting, no invalid profile and no discrete ambiguity.
//
// THE THREE PLACES THIS PROOF DEPARTS FROM FUSION'S ARITY, each recorded at its
// site as well as here, because in every one of them writing Fusion's own pair
// of constraints leaves the engine redundant at DOF 0 and the gate refuses it:
//
//   - the Anchor sketch's addCoincident(projectedCentre, anchorLine) beside
//     addMidPoint: the engine's midpoint already carries the point-on-line row;
//   - the two base-height offsets' companion perpendiculars, E->G / H->G and
//     F->I / J->I: the engine's Offset emits a row per endpoint and so carries
//     the parallelism Fusion's addOffsetDimension demands in advance. §2 states
//     this one itself;
//   - the same thing at the two toe lines, where §2's addParallel(M->N, C->H)
//     and addParallel(O->P, D->J) exist for that same Fusion precondition.
//
// In each case the omitted constraint is REQUIRED in Fusion and the module must
// write it. What the proof loses is that it does not exercise the Fusion pairing
// itself, only the freedoms the pairing removes.
package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

// pressureAngle is the virtual spur proxy's own default, 20 degrees. It is not a
// bevel dialog input.
const bevPressureAngle = 20 * math.Pi / 180

// involuteSteps is the proxy's InvoluteSteps, 15 in the shipped gear.
const bevInvoluteSteps = 15

// seedHeld is the [BEVEL-F-SEED-HELD] tolerance, 0.001 mm, which is what the
// generated module's own end-of-§2 gate compares against. The proof's lattice
// assertion is that gate's copy.
const bevSeedHeld = 1e-3

// anchorProjection and toothCentreProjection are the source ids a projection
// carries. The engine's reference geometry is coordinate-LOCKED where Fusion's
// sketch.project arrives associatively and still holds free degrees of freedom
// ([PB-PROJECT-NOT-FIXED]), so what is modelled here is the projection's
// position, never its freedom.
const (
	bevAnchorProjection      = "Anchor sketch centre projection"
	bevToothCentreProjection = "tooth-centre point on the {gear} Plane"
)

// kv is one parameter override.
type bevKV struct {
	key   string
	value float64
}

// bevelCase is one dialog's worth of inputs at their defaults, with the four
// values a case always states written in.
func bevCase(module, shaftAngleDeg, drivingTeeth, pinionTeeth float64) map[string]float64 {
	return map[string]float64{
		"module":            module,
		"shaftAngle":        shaftAngleDeg,
		"drivingTeeth":      drivingTeeth,
		"pinionTeeth":       pinionTeeth,
		"drivingBaseHeight": 0,
		"pinionBaseHeight":  0,
		"boreEnable":        1,
		"drivingBore":       0,
		"pinionBore":        0,
		"faceWidth":         0,
		"toothSpacing":      0,
		"spiralAngle":       0,
		"hand":              1,
		"cutterRadius":      0,
		"toeExtension":      0,
		"drivingToeRadius":  0,
		"pinionToeRadius":   0,
		"centerX":           0,
		"centerY":           0,
		"gear":              0,
		"latticeRefused":    0,
	}
}

// set copies a case and overrides the named values.
func bevWith(p map[string]float64, entries ...bevKV) map[string]float64 {
	out := make(map[string]float64, len(p))
	for k, v := range p {
		out[k] = v
	}
	for _, e := range entries {
		out[e.key] = e.value
	}
	return out
}

// latticeCases sweep the §2 lattice across the regime the spec states it holds
// over: the whole admitted Shaft Angle range, both directions of asymmetry
// (either gear can carry the smaller tooth count, and the smaller one is the
// binding side of the Maximum Face Width), the lowest tooth count the Minimum
// Teeth floor admits, and every input that moves the toe lattice.
//
// Shaft Angle 30 is carried as a DECLARED REFUSAL rather than dropped. It is a
// configuration the spec admits and this particular constraint net cannot reach:
// the net's conditioning falls below the sketch engine's 4e-5 trust floor there.
// Three independently written lattices disagree about which end of the range
// they reach, so that is a property of the net rather than of the geometry, and
// narrowing the advertised range on one net's evidence is what the spec forbids.
var bevLatticeCases = []proofkit.Case{
	{Name: "default_31_31_shaft90", Params: bevCase(1, 90, 31, 31)},
	{Name: "shaft30_declared_refusal", Params: bevWith(bevCase(1, 30, 31, 31),
		bevKV{"latticeRefused", 1})},
	{Name: "shaft35", Params: bevCase(1, 35, 31, 31)},
	{Name: "shaft60", Params: bevCase(1, 60, 31, 31)},
	{Name: "shaft120", Params: bevCase(1, 120, 31, 31)},
	{Name: "shaft142", Params: bevCase(1, 142, 31, 31)},
	{Name: "shaft150_upper_cap", Params: bevCase(1, 150, 31, 31)},
	{Name: "teeth16_12_shaft90", Params: bevCase(1, 90, 16, 12)},
	{Name: "teeth4_4_shaft90_lowest_count", Params: bevCase(1, 90, 4, 4)},
	{Name: "teeth43_31_shaft75", Params: bevCase(1, 75, 43, 31)},
	{Name: "driving17_pinion31_driving_binds", Params: bevCase(1, 90, 17, 31)},
	{Name: "driving31_pinion17_pinion_binds", Params: bevCase(1, 90, 31, 17)},
	{Name: "module4_default_pair", Params: bevCase(4, 90, 31, 31)},
	{Name: "tooth_spacing_positive", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"toothSpacing", 0.4})},
	{Name: "toe_extension_50", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"toeExtension", 50})},
	{Name: "toe_radius_user", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"drivingToeRadius", 3}, bevKV{"pinionToeRadius", 3})},
	{Name: "toe_extension_50_toe_radius_user", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"toeExtension", 50}, bevKV{"drivingToeRadius", 3}, bevKV{"pinionToeRadius", 3})},
	{Name: "base_heights_user", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"drivingBaseHeight", 5}, bevKV{"pinionBaseHeight", 6})},
	{Name: "centre_off_sketch_origin", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"centerX", 7}, bevKV{"centerY", -4})},
}

// anchorCases move the user's centre point off the sketch origin, since nothing
// in the dialog requires it to sit there.
var bevAnchorCases = []proofkit.Case{
	{Name: "centre_on_sketch_origin", Params: bevCase(1, 90, 31, 31)},
	{Name: "centre_off_sketch_origin", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"centerX", 7}, bevKV{"centerY", -4})},
	{Name: "centre_far_off_sketch_origin", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"centerX", -31}, bevKV{"centerY", 22})},
}

// perGear takes a case list and returns it once per gear, since §3 and every
// step of "Create the Gear Bodies" run once for the pinion and once for the
// driving gear with that gear's own parameters.
func bevPerGear(cases []proofkit.Case) []proofkit.Case {
	out := make([]proofkit.Case, 0, 2*len(cases))
	for _, c := range cases {
		out = append(out,
			proofkit.Case{Name: c.Name + "_pinion", Params: bevWith(c.Params, bevKV{"gear", 0})},
			proofkit.Case{Name: c.Name + "_driving", Params: bevWith(c.Params, bevKV{"gear", 1})})
	}
	return out
}

// toothBaseCases are the per-gear sketch table's configurations. The two the
// virtual tooth count needs are both here: 16/12 puts an exact count of 15 on
// one member and 26.667 on the other, so a rounded count is wrong by a different
// amount on each; and 4/4 carries the lowest virtual count the table reaches,
// 5.657, whose root arc has the largest corner float of any pair the spec
// admits and is therefore what fixes whether the root sink is large enough.
var bevToothBaseCases = []proofkit.Case{
	{Name: "default_31_31_shaft90", Params: bevCase(1, 90, 31, 31)},
	{Name: "shaft35", Params: bevCase(1, 35, 31, 31)},
	{Name: "shaft142", Params: bevCase(1, 142, 31, 31)},
	{Name: "teeth16_12_shaft90", Params: bevCase(1, 90, 16, 12)},
	{Name: "teeth4_4_shaft90", Params: bevCase(1, 90, 4, 4)},
	{Name: "teeth43_31_shaft75", Params: bevCase(1, 75, 43, 31)},
	{Name: "driving17_pinion31", Params: bevCase(1, 90, 17, 31)},
	{Name: "module4_default_pair", Params: bevCase(4, 90, 31, 31)},
	{Name: "tooth_spacing_positive", Params: bevWith(bevCase(4, 75, 43, 31),
		bevKV{"toothSpacing", 0.5})},
	{Name: "toe_extension_50", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"toeExtension", 50})},
	{Name: "toe_radius_user", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"drivingToeRadius", 3}, bevKV{"pinionToeRadius", 3})},
	{Name: "toe_extension_50_toe_radius_user", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"toeExtension", 50}, bevKV{"drivingToeRadius", 3}, bevKV{"pinionToeRadius", 3})},
}

var bevToothCases = bevPerGear(bevToothBaseCases)

var bevProfileCases = bevPerGear(bevToothBaseCases)

// boreCases add the two branches the Bore step takes: a bore the bound admits,
// an auto bore, and Enable Bore unchecked, where no sketch is drawn at all.
var bevBoreCases = bevPerGear([]proofkit.Case{
	{Name: "auto_bore_default_pair", Params: bevCase(1, 90, 31, 31)},
	{Name: "auto_bore_shaft35_heel_term_binds", Params: bevCase(1, 35, 31, 31)},
	{Name: "user_bore_below_maximum", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"drivingBore", 4}, bevKV{"pinionBore", 4})},
	{Name: "auto_bore_teeth16_12", Params: bevCase(1, 90, 16, 12)},
	{Name: "bore_disabled", Params: bevWith(bevCase(1, 90, 31, 31), bevKV{"boreEnable", 0})},
})

// spiralSketchCases sweep the cutter-arc trace over the whole [0, 60) Mean
// Spiral Angle range above zero, both hands, and both cutter-radius branches. A
// Mean Spiral Angle of 0 is not here: the tooth-body hook returns the straight
// path before any of §3a runs, so no trace sketch exists to prove.
var bevSpiralSketchCases = bevPerGear([]proofkit.Case{
	{Name: "spiral35_right_auto_cutter", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"spiralAngle", 35})},
	{Name: "spiral35_left_auto_cutter", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"spiralAngle", 35}, bevKV{"hand", -1})},
	{Name: "spiral5_right_near_the_straight_limit", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"spiralAngle", 5})},
	{Name: "spiral55_right_near_the_upper_bound", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"spiralAngle", 55})},
	{Name: "spiral35_user_cutter", Params: bevWith(bevCase(1, 90, 31, 31),
		bevKV{"spiralAngle", 35}, bevKV{"cutterRadius", 30})},
	{Name: "spiral35_ratio_pair_43_31", Params: bevWith(bevCase(1, 75, 43, 31),
		bevKV{"spiralAngle", 35})},
	{Name: "spiral35_ratio_pair_17_31", Params: bevWith(bevCase(1, 90, 17, 31),
		bevKV{"spiralAngle", 35})},
})

// stepAnchorSketch builds the Anchor sketch: the user's centre point projected
// in, and one reference line through it.
//
// The line's absolute direction is arbitrary — §2 derives every direction
// relative to it — but it must not be a free degree of freedom, so the sketch
// pins it with a sketch-local Horizontal ([PB-REFLINE-DIRECTION]). A world-axis
// lock would mis-orient the figure on a tilted target plane.
//
// Fusion applies BOTH addCoincident(projectedCentre, anchorLine) and
// addMidPoint(projectedCentre, anchorLine), and the module must write both. The
// engine's midpoint already emits the row the coincident would add, so writing
// both here leaves the sketch redundant at DOF 0 and the gate refuses it. The
// midpoint alone is what is written, and what that costs is that the proof does
// not exercise Fusion's own pairing of the two.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)

	proofkit.Step(t, "project the user's centre point onto the target plane")
	centre := s.CreateReferencePoint(f.center.X, f.center.Y, bevAnchorProjection)

	// The endpoints are seeded at exactly +/- 0.5 cm from the projected centre
	// along the sketch-local X, so the seeded length is the 10 mm the aligned
	// dimension then locks.
	proofkit.Step(t, "the anchor line, seeded 10 mm long about the centre")
	start := s.CreatePoint(f.center.X-5, f.center.Y)
	end := s.CreatePoint(f.center.X+5, f.center.Y)
	anchorLine := s.CreateLine(start, end)
	anchorLine.SetConstruction(true)

	// The length dimension is written in the engine's SIGNED form. Fusion's aligned
	// distance dimension is a magnitude whose direction is captured from the seeded
	// geometry, so the reversed line — end for end, which satisfies the midpoint,
	// the length and the Horizontal equally — is a second configuration the engine's
	// probe reports and the gate refuses. The signed target carries the seed side
	// across instead, which is the documented mapping ([PB-DIM-VALUE-SEMANTICS]):
	// only abs(target) may go into Fusion's parameter.value, and the side is the
	// seed's. It costs nothing here, because §2 derives every direction relative to
	// this line and the grow side is a one-bit read of the target plane's normal
	// that this engine has no plane to take.
	s.AddConstraint(
		sketch.NewMidpoint(centre, anchorLine),
		sketch.NewHorizontal(anchorLine),
		sketch.NewHorizontalDistance(start, end, 10),
	)

	proofkit.Step(t, "the anchor line is centred on the projection and 10 mm long")
	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, centre, f.center.X, f.center.Y, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "anchor line length",
		end.X()-start.X(), 10, sketchtest.Within(1e-9))
	sketchtest.Measures(t, "anchor line midpoint offset from the projected centre",
		math.Hypot((start.X()+end.X())/2-f.center.X, (start.Y()+end.Y())/2-f.center.Y),
		0, sketchtest.Within(1e-9))
	for _, constraint := range s.Constraints() {
		sketchtest.Satisfies(t, constraint, sketchtest.Within(1e-9))
	}
}

// lattice is the §2 sketch under construction: every line is built from raw
// coordinates and pinned with one addCoincident per connecting endpoint, which
// is the COINCIDENT style [BEVEL-F-COINCIDENT-STYLE] allows and the only style
// it allows. Sharing a SketchPoint instead leaves the Gear Profiles sketch
// under-constrained in Fusion; sharing AND coinciding fails the solve outright.
type bevLattice struct {
	t testing.TB
	s *sketch.Sketch
}

// line draws one §2 construction line from raw coordinates and returns it with
// its two fresh endpoints.
func (l *bevLattice) line(a, b bevPt) (*sketch.Line, *sketch.Point, *sketch.Point) {
	pa := l.s.CreatePoint(a.X, a.Y)
	pb := l.s.CreatePoint(b.X, b.Y)
	ln := l.s.CreateLine(pa, pb)
	// Every line drawn in §2 is a construction line: the lattice lines, the toe
	// lines, the front faces and the short reference lines alike. The solid
	// features consume the per-gear Profile sketches, never a §2 curve.
	ln.SetConstruction(true)
	return ln, pa, pb
}

// pin is one addCoincident: the fresh endpoint onto the point that already
// exists.
func (l *bevLattice) pin(fresh, existing *sketch.Point) {
	l.s.AddConstraint(sketch.NewCoincident(fresh, existing))
}

// connect draws a line whose BOTH endpoints already exist — the short reference
// and connector lines C->K', M->C, N->A', B'->I and their twins. Each end takes
// one coincident. A regen that shares these already-pinned points instead tips
// the sketch to under-constrained; one that shared only these came out about
// fourteen coincidents short.
func (l *bevLattice) connect(a, b bevPt, pa, pb *sketch.Point) *sketch.Line {
	ln, sa, sb := l.line(a, b)
	l.pin(sa, pa)
	l.pin(sb, pb)
	return ln
}

// stepGearProfiles builds the §2 Gear Profiles sketch: the whole lattice for
// both gears, in one sketch, in the order §2 creates it.
//
// WHERE A SIGNED CONSTRAINT STANDS IN FOR AN UNSIGNED FUSION ONE.
// [BEVEL-F-MIRROR-FIGURE] counts fifteen sites whose side no Fusion constraint
// pins — every geometric constraint Fusion offers is unsigned or undirected, and
// the seed is the only thing that picks the figure. This engine signs two
// things Fusion does not: NewAngle and NewOffset. So wherever §2 writes an
// addPerpendicular or an addParallel whose side the seed decides, the proof
// writes the SIGNED angle of the same arity, computed from the closed form. That
// is what the "Proving the §2 figure" section asks for: pin every one of the
// fifteen with a constraint the engine signs, rather than reading a clean
// ambiguity probe as evidence that no twin figure exists. The probe reports a
// LOWER bound on the number of solutions, so its silence is never the evidence.
//
// WHAT THIS STEP CANNOT REACH. It seeds at the closed form, so it proves the net
// solves from a correct seed and never that the generated module's seed is
// correct. That is the same limit the toe-line seeding records below, and it is
// why [BEVEL-F-SEED-HELD] has to run inside the module and not only here.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)
	if p["latticeRefused"] != 0 {
		proofkit.Unmodelled(t, "this lattice refuses Shaft Angle %.0f degrees on conditioning, below "+
			"the sketch engine's 4e-5 trust floor. The spec admits the configuration and two of three "+
			"independently written lattices refuse it, so the refusal belongs to this net and the "+
			"advertised range is not narrowed for it", p["shaftAngle"])
	}
	l := &bevLattice{t: t, s: s}

	proofkit.Step(t, "project the Anchor sketch's centre point and its anchor line")
	centre := s.CreateReferencePoint(f.center.X, f.center.Y, bevAnchorProjection)
	left := s.CreateReferencePoint(f.center.X-5, f.center.Y, bevAnchorProjection)
	right := s.CreateReferencePoint(f.center.X+5, f.center.Y, bevAnchorProjection)
	anchorLine, err := s.CreateReferenceLine(left, right, bevAnchorProjection)
	if err != nil {
		t.Fatalf("project the anchor line: %v", err)
	}

	proofkit.Step(t, "centre -> Apex, perpendicular to the anchor line, undimensioned")
	centerToApex, ctaStart, apex := l.line(f.center, f.apex)
	l.pin(ctaStart, centre)
	s.AddConstraint(sketch.NewAngle(anchorLine, centerToApex, bevSignedAngleDeg(f.anchor, f.perp)))

	proofkit.Step(t, "Apex -> B, the Driving Gear Shaft Axis, parallel to centre -> Apex")
	drivingAxis, dsStart, pointB := l.line(f.apex, f.driving.shaftEnd)
	l.pin(dsStart, apex)
	s.AddConstraint(sketch.NewAngle(centerToApex, drivingAxis,
		bevSignedAngleDeg(f.perp, f.driving.shaftDir)))

	proofkit.Step(t, "Apex -> A, the Pinion Gear Shaft Axis, at the Shaft Angle")
	pinionAxis, psStart, pointA := l.line(f.apex, f.pinion.shaftEnd)
	l.pin(psStart, apex)
	s.AddConstraint(sketch.NewAngle(drivingAxis, pinionAxis,
		bevSignedAngleDeg(f.driving.shaftDir, f.pinion.shaftDir)))

	// The two perpendicular drops close on Apex 2, which sits in the interior
	// wedge BETWEEN the two shaft axes. Each drop must aim at the other shaft
	// axis: a drop seeded on the far side makes the solver flip the whole figure
	// to its mirror, which nothing downstream refuses.
	proofkit.Step(t, "A -> Apex 2, the PPD/2 drop, and B -> Apex 2, the DPD/2 drop")
	dropA, dropAStart, apex2 := l.line(f.pinion.shaftEnd, f.apex2)
	l.pin(dropAStart, pointA)
	s.AddConstraint(
		sketch.NewAngle(pinionAxis, dropA,
			bevSignedAngleDeg(f.pinion.shaftDir, f.apex2.sub(f.pinion.shaftEnd).unit())),
		sketch.NewDistance(dropAStart, apex2, f.pinion.pitchRadius),
	)
	dropB, dropBStart, dropBEnd := l.line(f.driving.shaftEnd, f.apex2)
	l.pin(dropBStart, pointB)
	l.pin(dropBEnd, apex2)
	s.AddConstraint(
		sketch.NewAngle(drivingAxis, dropB,
			bevSignedAngleDeg(f.driving.shaftDir, f.apex2.sub(f.driving.shaftEnd).unit())),
		sketch.NewDistance(dropBStart, dropBEnd, f.driving.pitchRadius),
	)

	proofkit.Step(t, "the Pitch Line, Apex -> Apex 2")
	pitchLine := l.connect(f.apex, f.apex2, apex, apex2)
	top := map[string]*sketch.Point{"Apex": apex, "B": pointB, "A": pointA, "Apex 2": apex2}

	// The two dedendum directions are picked by dot product against the shaft
	// axes. Those dots are sin(gamma_p) and sin(gamma_g), strictly positive for
	// every admitted configuration, unlike the "towards the anchor line" test,
	// which reads about zero by construction. Flip the pinion seed and C solves
	// exactly onto D.
	proofkit.Step(t, "Apex 2 -> C and Apex 2 -> D, the two dedendum lines")
	named := map[string]*sketch.Point{}
	dedLine := map[string]*sketch.Line{}
	rootAxis := map[string]*sketch.Line{}
	shaftAxis := map[string]*sketch.Line{"Pinion": pinionAxis, "Driving": drivingAxis}
	drop := map[string]*sketch.Line{"Pinion": dropA, "Driving": dropB}
	shaftStart := map[string]*sketch.Point{"Pinion": pointA, "Driving": pointB}

	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		ded, dedStart, dedEnd := l.line(f.apex2, g.ded)
		l.pin(dedStart, apex2)
		s.AddConstraint(
			sketch.NewAngle(pitchLine, ded, bevSignedAngleDeg(f.pitchDir, g.dedDir)),
			sketch.NewDistance(dedStart, dedEnd, f.dedendum),
		)
		dedLine[g.label] = ded
		named[g.label+" ded"] = dedEnd
		rootAxis[g.label] = l.connect(f.apex, g.ded, apex, dedEnd)
	}

	proofkit.Step(t, "the A -> E / B -> F extensions and the C -> E / D -> F perpendiculars")
	extLine := map[string]*sketch.Line{}
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		// A collinear names the line the new line's START sits on, and it carries
		// two point-on-line rows. Here one of those rows is already satisfied,
		// because the extension starts at the shaft axis's own endpoint, so the
		// proof writes the single row that is not implied ([PB-COLLINEAR-CHAIN]).
		// Only a Fusion session tells the two readings apart.
		ext, extStart, extEnd := l.line(g.shaftEnd, g.ext)
		l.pin(extStart, shaftStart[g.label])
		s.AddConstraint(sketch.NewPointOnLine(extEnd, shaftAxis[g.label]))
		foot := l.connect(g.ded, g.ext, named[g.label+" ded"], extEnd)
		s.AddConstraint(sketch.NewPerpendicular(ext, foot))
		extLine[g.label] = ext
		named[g.label+" ext"] = extEnd
	}

	proofkit.Step(t, "the base-height offsets: E -> G with G -> H, F -> I with I -> J")
	heelLine := map[string]*sketch.Line{}
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		// E -> G is collinear with A -> E, never with the Apex -> A shaft axis
		// further up the chain, even though both describe the same infinite line.
		_, heelStart, heelEnd := l.line(g.ext, g.axisHeel)
		l.pin(heelStart, named[g.label+" ext"])
		s.AddConstraint(sketch.NewPointOnLine(heelEnd, extLine[g.label]))

		dedHeel, dedHeelStart, dedHeelEnd := l.line(g.ded, g.dedHeel)
		l.pin(dedHeelStart, named[g.label+" ded"])
		s.AddConstraint(sketch.NewPointOnLine(dedHeelEnd, dedLine[g.label]))

		back := l.connect(g.axisHeel, g.dedHeel, heelEnd, dedHeelEnd)
		// Fusion needs a perpendicular here before addOffsetDimension will take the
		// pair, because its offset dimension requires the second line to be parallel
		// to the first already. The engine's Offset emits a row per endpoint and so
		// carries that parallelism itself; adding the perpendicular is a third row
		// for the same two freedoms, and the engine reports the two base-height
		// offsets as a redundant pair. The module must still write it.
		s.AddConstraint(sketch.NewOffset(drop[g.label], back,
			bevSignedOffset(g.shaftEnd, f.apex2, g.axisHeel)))

		heelLine[g.label] = dedHeel
		named[g.label+" axisHeel"] = heelEnd
		named[g.label+" dedHeel"] = dedHeelEnd
	}

	// Constraining point I with the centre point is what hangs the whole lattice
	// off the projection. Fusion writes addCoincident(I, projectedCentre); one of
	// that coincident's two rows is already implied, because the chain starts at
	// the projected centre and the perpendicular above fixes its direction, so
	// only the component across the anchor line is independent. Written in full it
	// leaves the net redundant at DOF 0, which is the one dependent row the spec
	// records the lattice as carrying. The proof keeps the independent equation.
	proofkit.Step(t, "constrain point I with the centre point")
	s.AddConstraint(sketch.NewPointOnLine(named["Driving axisHeel"], anchorLine))

	proofkit.Step(t, "A' -> G, the pinion hexagon's shaft-axis edge, which CREATES A'")
	_, pinionAxisToe, agEnd := l.line(f.pinion.axisToe, f.pinion.axisHeel)
	l.pin(agEnd, named["Pinion axisHeel"])
	named["Pinion axisToe"] = pinionAxisToe

	proofkit.Step(t, "K and L, and the Tooth Spacing centres K' and L'")
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		// K is the case where both ends are already fixed, so it takes two
		// point-on-line coincidents and no collinear at all: by the time it is
		// added G and C are fixed, and an addCollinear here over-constrains.
		_, kStart, k := l.line(g.axisHeel, g.center)
		l.pin(kStart, named[g.label+" axisHeel"])
		s.AddConstraint(
			sketch.NewPointOnLine(k, shaftAxis[g.label]),
			sketch.NewPointOnLine(k, dedLine[g.label]),
		)
		named[g.label+" center"] = k

		if f.toothSpacing == 0 {
			// At Tooth Spacing 0 nothing is built here: K' is K, and the existing
			// C -> K reference line is reused. A zero-length dimensioned line would be
			// degenerate, and one segment gets one line.
			l.connect(g.ded, g.center, named[g.label+" ded"], k)
			named[g.label+" centerOff"] = k
			continue
		}
		spacing, kkStart, kPrime := l.line(g.center, g.centerOff)
		l.pin(kkStart, k)
		// The length dimension on K -> K' is unsigned in Fusion, and the twin it
		// admits sits one Tooth Spacing on the C side of K, two spacings away. A
		// clean ambiguity probe is not evidence against that twin: the engine
		// documents its probe as a LOWER bound on the number of solutions. So the
		// side is pinned with a constraint the engine signs — the zero angle from
		// the dedendum line to K -> K' — which stands in for Fusion's second
		// point-on-line coincident and carries the same single row.
		s.AddConstraint(
			sketch.NewAngle(dedLine[g.label], spacing,
				bevSignedAngleDeg(g.dedDir, g.centerOff.sub(g.center).unit())),
			sketch.NewDistance(kkStart, kPrime, f.toothSpacing),
		)
		l.connect(g.ded, g.centerOff, named[g.label+" ded"], kPrime)
		named[g.label+" centerOff"] = kPrime
	}

	proofkit.Step(t, "the toe lattice: M -> N and O -> P, then the two front faces")
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		// SEEDING, AND WHAT THE PROOF CANNOT CATCH HERE. M and N are seeded at their
		// closed-form solved positions: M on Apex->C at the fraction the Root Length
		// leaves, and N slid from M along C->H by (M's own radius - Toe Radius) /
		// cos(gamma). Along C->H the perpendicular distance from the shaft axis FALLS
		// at cos(gamma) per unit; read as a rise it is the wrong sign, and the compile
		// round that first wrote the rule reported 20 of its 21 lattice cases failing
		// to converge until it corrected it.
		//
		// Because the proof seeds at the closed form, it proves the constraints solve
		// from a correct seed and NEVER that the generated module's seed is correct. A
		// seed defect therefore reaches Fusion untested, which is how a wrong N seed
		// got there: the earlier rule slid by the Root Length instead, which puts the
		// N seed 0.27 mm PAST the shaft axis on the shipped default pair at a 50% Toe
		// Extension against a solved N at +5.17 mm, and Fusion aborts the revolve with
		// ASM_WIRE_X_AXIS rather than naming the seed.
		toe, m, n := l.line(g.toeRoot, g.toeInner)
		s.AddConstraint(sketch.NewPointOnLine(m, rootAxis[g.label]))
		// §2 also writes addParallel(M->N, C->H), for the same Fusion precondition
		// the base-height perpendiculars serve. The engine's Offset carries it, so
		// adding it here is a redundant row; the module must still write it.
		s.AddConstraint(sketch.NewOffset(heelLine[g.label], toe,
			bevSignedOffset(g.ded, g.dedHeel, g.toeRoot)))
		l.connect(g.toeRoot, g.ded, m, named[g.label+" ded"])

		// N is never pinned to the shaft axis: that would put it ON the axis of
		// revolution and the later conical split fails with ASM_API_FAILED for
		// asymmetric tooth counts. A' sits on the axis, and it is a FOOT, not a
		// corner; N never does, because the Toe Radius is strictly positive.
		var front *sketch.Line
		var frontN, axisToe *sketch.Point
		if existing := named[g.label+" axisToe"]; existing != nil {
			// The pinion: A' already exists, created by the A' -> G line above, so
			// N -> A' connects two points that both exist and takes one coincident
			// per end.
			front = l.connect(g.toeInner, g.axisToe, n, existing)
			frontN, axisToe = front.Start, existing
		} else {
			// The driving gear: P -> B' is the line that CREATES B', so it takes one
			// coincident rather than two, and B' -> I is drawn after it.
			var bPrime *sketch.Point
			front, frontN, bPrime = l.line(g.toeInner, g.axisToe)
			l.pin(frontN, n)
			axisToe = bPrime
			named[g.label+" axisToe"] = bPrime
		}
		s.AddConstraint(
			sketch.NewPointOnLine(axisToe, shaftAxis[g.label]),
			sketch.NewAngle(shaftAxis[g.label], front,
				bevSignedAngleDeg(g.shaftDir, g.axisToe.sub(g.toeInner).unit())),
			sketch.NewDistance(frontN, front.End, g.toeRadius),
		)
		if g.label == "Driving" {
			l.connect(g.axisToe, g.axisHeel, axisToe, named[g.label+" axisHeel"])
		}
		named[g.label+" toeRoot"] = m
		named[g.label+" toeInner"] = n
	}

	proofkit.Step(t, "the solved figure against its own seeds, and the two bounds §2 resolves")
	sketchtest.Solve(t, s)
	bevAssertSeedHeld(t, f, top, named)
	bevAssertFaceWidthBound(t, f, named)
	bevAssertBoreBound(t, f)
}

// assertSeedHeld is the proof's copy of the [BEVEL-F-SEED-HELD] gate: every
// named §2 point's solved position against the closed form §2 seeded it at, at
// the same 0.001 mm tolerance, in the order §2 creates them so the first point
// that moved is the one reported.
//
// The list is twenty-two points only when Tooth Spacing is above zero. At Tooth
// Spacing 0 — the default — K' is K and L' is L and neither is built, so twenty
// are compared; comparing a K' that was never created is the one way this gate
// raises on a correct figure.
func bevAssertSeedHeld(t testing.TB, f *bevFigure, top, named map[string]*sketch.Point) {
	t.Helper()
	type check struct {
		name string
		want bevPt
		got  *sketch.Point
	}
	order := []check{}
	add := func(name string, want bevPt, got *sketch.Point) {
		order = append(order, check{name, want, got})
	}
	// Apex, B, A and Apex 2 come first, in the order §2 creates them, so the
	// message names the earliest site that flipped rather than a downstream
	// symptom.
	add("Apex", f.apex, top["Apex"])
	add("B", f.driving.shaftEnd, top["B"])
	add("A", f.pinion.shaftEnd, top["A"])
	add("Apex 2", f.apex2, top["Apex 2"])
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		add(g.label+" C/D", g.ded, named[g.label+" ded"])
		add(g.label+" E/F", g.ext, named[g.label+" ext"])
		add(g.label+" G/I", g.axisHeel, named[g.label+" axisHeel"])
		add(g.label+" H/J", g.dedHeel, named[g.label+" dedHeel"])
		add(g.label+" K/L", g.center, named[g.label+" center"])
		if f.toothSpacing > 0 {
			add(g.label+" K'/L'", g.centerOff, named[g.label+" centerOff"])
		}
		add(g.label+" M/O", g.toeRoot, named[g.label+" toeRoot"])
		add(g.label+" N/P", g.toeInner, named[g.label+" toeInner"])
		add(g.label+" A'/B'", g.axisToe, named[g.label+" axisToe"])
	}
	for _, c := range order {
		if c.got == nil {
			t.Fatalf("%s was never created, so the lattice cannot be gated against its seed", c.name)
		}
		sketchtest.MeasuresPoint(t, c.got, c.want.X, c.want.Y, sketchtest.Within(bevSeedHeld))
	}
}

// assertFaceWidthBound reads the Maximum Face Width off the SOLVED geometry, the
// way §2 requires it to be read, and holds it to the closed form. Seeds diverge
// from the solve for asymmetric tooth counts and non-90-degree shaft angles, so a
// seed-based bound is too loose on the binding side and the toe still crosses the
// axis.
func bevAssertFaceWidthBound(t testing.TB, f *bevFigure, named map[string]*sketch.Point) {
	t.Helper()
	distance := func(g *bevSide, from bevPt) float64 {
		c := named[g.label+" ded"]
		h := named[g.label+" dedHeel"]
		dir := bevPt{h.X() - c.X(), h.Y() - c.Y()}.unit()
		return math.Abs(dir.cross(from.sub(bevPt{c.X(), c.Y()})))
	}
	// A to the pinion dedendum line and B to the driving one. On the solved figure
	// both reduce to R*sin^2(gamma), the smaller of which binds.
	pinion := distance(&f.pinion, f.pinion.shaftEnd)
	driving := distance(&f.driving, f.driving.shaftEnd)
	sketchtest.Measures(t, "perpendicular distance from A to the Pinion Dedendum line",
		pinion, f.pitchCone*bevSq(math.Sin(f.pinion.gamma)), sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "perpendicular distance from B to the Driving Dedendum line",
		driving, f.pitchCone*bevSq(math.Sin(f.driving.gamma)), sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, "Maximum Face Width",
		0.95*math.Min(pinion, driving), f.maxFaceWidth, sketchtest.WithinRel(1e-9))
	if f.faceWidth > f.maxFaceWidth {
		t.Errorf("the resolved Face Width %.6g mm exceeds the Maximum Face Width %.6g mm, which is "+
			"the profile crossing its own axis of revolution", f.faceWidth, f.maxFaceWidth)
	}
}

// assertBoreBound holds the Maximum Bore Diameter to its two terms and refuses a
// resolved bore above it.
//
// WHAT THIS CANNOT REACH: the generated module raising on the USER's value. The
// proof never runs that module, so the refusal itself is only ever seen in
// Fusion, where it has been seen once. There is no case to add for it.
func bevAssertBoreBound(t testing.TB, f *bevFigure) {
	t.Helper()
	for _, g := range []*bevSide{&f.pinion, &f.driving} {
		heel := g.pitchRadius - g.baseHeight/math.Tan(g.gamma)
		toe := (f.apexToDed - f.rootLength) * math.Sin(g.gammaRoot)
		sketchtest.Measures(t, g.label+" r_heel, where H lands",
			g.heelBoreRadius, heel, sketchtest.WithinRel(1e-12))
		sketchtest.Measures(t, g.label+" r_toe, where M lands",
			g.toeBoreRadius, toe, sketchtest.WithinRel(1e-12))
		sketchtest.Measures(t, g.label+" Maximum Bore Diameter",
			g.maxBore, 2*0.95*math.Min(heel, toe), sketchtest.WithinRel(1e-12))
		if g.boreDiameter > g.maxBore {
			t.Errorf("%s: the resolved bore diameter %.6g mm is above the Maximum Bore Diameter "+
				"%.6g mm, which takes the whole back face off the revolved frustum",
				g.label, g.boreDiameter, g.maxBore)
		}
		if f.toeExtension == 0 {
			sketchtest.Measures(t, g.label+" r_toe against the Toe Radius Ceiling at Toe Extension 0",
				g.toeBoreRadius, g.toeRadiusCeiling, sketchtest.WithinRel(1e-9))
		}
	}
}

// stepToothProfile builds one gear's `{gearLabel} Tooth` sketch: the virtual
// spur tooth the borrowed drawer draws on the back-cone plane, already rotated
// 180 degrees by the draw() angle rather than by a later sketch rotation.
//
// THE VIRTUAL TOOTH COUNT IS A REAL NUMBER AND IS NEVER ROUNDED. The Tredgold
// construction puts the equivalent spur gear's pitch radius exactly at
// r/cos(gamma), and the drawer reads the count in one place, the angular half
// thickness pi/(2*z_v), which at z_v = 2*r_v/Module gives the standard tooth
// thickness pi*Module/2 at the pitch circle. A rounded count rebuilds every
// circle from the rounded radius and shortens the working addendum.
//
// THE TOOTH-TOP ARC IS A CENTRE-POINT ARC WITH A PINNED CENTRE AND NO DIMENSION.
// The drawer creates it with addByCenterStartEnd and then pins the copied centre
// with addCoincident(arc.centerSketchPoint, localOrigin), carrying no radius and
// no diameter dimension. It is NOT a three-point arc with a radius dimension:
// that sentence sat in a proof comment once and sent an investigation chasing a
// reflected centre this construction has no room for. The §3a trace arc below is
// the genuine three-point-with-radius case, and the two are not written from one
// template.
//
// The faithful coincident is what is written here, and it solves: the cost is
// nil and this comment is the record the spec asks for.
//
// This sketch is EXEMPT from the module's full-constraint gate, because the
// drawer labels its four circles and sketch text holds a DOF
// ([PB-TEXT-HOLDS-DOF]). The engine has no sketch text, so what the exemption
// covers is not reproduced here; the geometry itself must still reach DOF 0, and
// that is what this step proves.
func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)

	dims := involute.Derive(f.module, g.virtualTeeth, bevPressureAngle)
	// The root circle is drawn one root sink INSIDE the dedendum corner. At the
	// corner exactly, only the point on the tooth's own centreline rides the root
	// cone and the arc's two corners stand outside it, so the Combine-Join meets
	// the gear body along one line instead of across the root.
	dims.Root -= f.rootSink

	proofkit.Step(t, "the four circles the proxy serves, at the exact back-cone radii")
	origin := s.CreatePoint(0, 0)
	circle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	circle(dims.Root, false)
	tip := circle(dims.Tip, true)
	circle(dims.Base, true)
	circle(dims.Pitch, true)

	proofkit.Step(t, "the two involute flanks, drawn already rotated 180 degrees")
	angle := math.Pi
	leftPts, rightPts := involute.Flanks(dims.Base, dims.Tip, dims.Pitch,
		g.virtualTeeth, bevInvoluteSteps, angle)
	left := make([]*sketch.Point, len(leftPts))
	right := make([]*sketch.Point, len(rightPts))
	for i := range leftPts {
		left[i] = s.CreatePoint(leftPts[i].X, leftPts[i].Y)
		right[i] = s.CreatePoint(rightPts[i].X, rightPts[i].Y)
	}
	if _, err := s.CreateFitSpline(left...); err != nil {
		t.Fatalf("left flank: %v", err)
	}
	if _, err := s.CreateFitSpline(right...); err != nil {
		t.Fatalf("right flank: %v", err)
	}

	proofkit.Step(t, "the tooth-top arc, its copied centre pinned to the local origin")
	topX, topY := involute.Rotate(dims.Tip, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, right[len(right)-1], left[len(left)-1])
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))

	proofkit.Step(t, "the spine and its confirming angular dimension")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(dims.Tip, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, dims.Tip),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	proofkit.Step(t, "the rib chain that pins the two flanks to the spine")
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prev := origin
	prevX, prevY := 0.0, 0.0
	for i := range leftPts {
		rib := s.CreateLine(left[i], right[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(left[i], right[i],
				rightPts[i].Y-leftPts[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(left[i], right[i],
				rightPts[i].X-leftPts[i].X))
		}
		along := leftPts[i].X*math.Cos(angle) + leftPts[i].Y*math.Sin(angle)
		mx, my := along*math.Cos(angle), along*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(
			sketch.NewPointOnLine(mid, spine),
			sketch.NewMidpoint(mid, rib),
		)
		if i != len(leftPts)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prev, mid, mx-prevX))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prev, mid, my-prevY))
		}
		prev, prevX, prevY = mid, mx, my
	}

	proofkit.Step(t, "the flank-to-root lines, present only when the tooth is not embedded")
	if !dims.Embedded() {
		foot := func(flankStart *sketch.Point, seed involute.Pt) {
			n := math.Hypot(seed.X, seed.Y)
			rx, ry := dims.Root*seed.X/n, dims.Root*seed.Y/n
			re := s.CreatePoint(rx, ry)
			s.CreateLine(re, flankStart)
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, re, rx),
				sketch.NewVerticalDistance(origin, re, ry),
			)
		}
		foot(left[0], leftPts[0])
		foot(right[0], rightPts[0])
	}

	proofkit.Step(t, "drag the tooth onto the tooth-centre point K'/L'")
	anchor := s.CreateReferencePoint(0, 0, bevToothCentreProjection)
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the radii the profile finder and the loft key on")
	bevAssertToothContract(t, f, g, dims, s, arcCentre, origin)
}

// assertToothContract holds the drawn tooth to the four radii §3 fixes and to
// the curve counts the tooth-profile selection keys on.
func bevAssertToothContract(t testing.TB, f *bevFigure, g *bevSide, dims involute.Dimensions,
	s *sketch.Sketch, arcCentre, origin *sketch.Point) {
	t.Helper()
	sketchtest.Solve(t, s)

	sketchtest.Measures(t, g.label+" virtual pitch radius",
		dims.Pitch, g.pitchRadius/math.Cos(g.gamma), sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, g.label+" virtual tooth number",
		g.virtualTeeth, g.teeth/math.Cos(g.gamma), sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, g.label+" tip radius",
		dims.Tip, g.virtualPitchRadius+f.module, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, g.label+" base radius",
		dims.Base, g.virtualPitchRadius*math.Cos(bevPressureAngle), sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, g.label+" root radius, one root sink inside the dedendum corner",
		dims.Root, g.virtualPitchRadius-f.dedendum-f.rootSink, sketchtest.WithinRel(1e-12))
	sketchtest.Measures(t, g.label+" root sink", f.rootSink, 0.05*2.25*f.module,
		sketchtest.WithinRel(1e-12))

	// The count a rounded virtual tooth number would draw instead. It is asserted
	// as a DIFFERENCE rather than restated in a comment, so a regen that rounds
	// fails here rather than in the GUI.
	rounded := math.Floor(g.virtualTeeth)
	if rounded != g.virtualTeeth {
		if math.Abs(rounded*f.module/2-dims.Pitch) < 1e-9 {
			t.Errorf("%s: the drawn pitch radius %.6f mm is the FLOORED count's, not the exact "+
				"back-cone radius %.6f mm", g.label, dims.Pitch, g.virtualPitchRadius)
		}
	}

	// The tooth-top arc's centre is pinned, not stranded. A copied centre left
	// free stays behind by the drag distance and deforms the arc silently.
	sketchtest.Measures(t, g.label+" tooth-top arc centre gap from the local origin",
		math.Hypot(arcCentre.X()-origin.X(), arcCentre.Y()-origin.Y()), 0,
		sketchtest.Within(1e-9))

	// The profile finder takes the loop with 2 NURBS, 2 arcs and a line count the
	// embedded flag DETERMINES: 0 when embedded, 2 when not. Accepting either
	// count grabs an unrelated loop and the apex loft dies with LOFT_NO_TOOLBODY.
	wantLines := 2
	if dims.Embedded() {
		wantLines = 0
	}
	if g.embedded != dims.Embedded() {
		t.Errorf("%s: the embedded flag disagrees with the sunk root radius", g.label)
	}
	tooth, disc := 0, 0
	regions := s.Profiles()
	for _, region := range regions {
		nurbs, arcs, lines := bevToothCurveCounts(region.Entities)
		switch {
		case nurbs == 2 && arcs == 2 && lines == wantLines:
			tooth++
		case nurbs == 0 && arcs == 1 && lines == 0:
			disc++
			sketchtest.MeasuresProfileArea(t, region, math.Pi*dims.Root*dims.Root,
				sketchtest.WithinRel(1e-6))
		default:
			t.Errorf("%s: unexpected region: %d NURBS, %d arcs, %d lines",
				g.label, nurbs, arcs, lines)
		}
		sketchtest.IsCurrentProfile(t, region)
		sketchtest.IsValidProfile(t, region)
	}
	if tooth != 1 {
		t.Errorf("%s: tooth regions of 2 NURBS, 2 arcs and %d lines: %d, want exactly 1",
			g.label, wantLines, tooth)
	}
	if disc != 1 {
		t.Errorf("%s: disc regions inside the root circle: %d, want exactly 1", g.label, disc)
	}
}

// toothCurveCounts classifies a region's distinct boundary entities the way
// find_profile_by_curve_counts classifies Fusion's profile curves: a fitted
// spline is a NURBS, the tooth-top arc and the root circle are each an arc, and
// a flank-to-root stub is a line.
func bevToothCurveCounts(entities []sketch.Entity) (nurbs, arcs, lines int) {
	for _, entity := range entities {
		switch entity.(type) {
		case *sketch.FitSpline:
			nurbs++
		case *sketch.Arc, *sketch.Circle:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return nurbs, arcs, lines
}

// stepProfileSketch builds one gear's `{gearLabel} Profile` sketch: the hexagon
// recreated on fixed points, on the axial plane, one profile sketch per gear so
// sketch.profiles holds exactly this one loop.
//
// The vertices are recreated as new points at the §2 points' positions and the
// lines are drawn SHARING them; the points are fixed only AFTER the lines exist,
// which is the [PB-PROJECT-NOT-FIXED] recipe. Fixing a bare point before it is
// consumed as a line endpoint does not leave the sketch fully constrained, and a
// projection would not fix it at all.
//
// The hexagon's FIRST edge is the gear's shaft axis, and every body operation
// below uses that edge rather than the §2 Apex->A construction line. A free edge
// resolves against a default world frame and silently moves the body onto world
// XY, which was observed on the driving gear.
func stepProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	hex := g.hex()

	proofkit.Step(t, "recreate the six §2 vertices in draw order")
	points := make([]*sketch.Point, len(hex))
	for i, v := range hex {
		points[i] = s.CreatePoint(v.X, v.Y)
	}
	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, len(points))
	for i := range points {
		lines[i] = s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	proofkit.Step(t, "fix the endpoints, after the lines exist")
	for _, line := range lines {
		s.Fix(line.Start)
		s.Fix(line.End)
	}

	proofkit.Step(t, "the one loop the revolve consumes, and its first edge")
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s, sketch.WithProbe())
	region := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, region)
	sketchtest.IsCurrentProfile(t, region)
	sketchtest.HasExactCuts(t, region)
	sketchtest.MeasuresProfileArea(t, region, bevPolygonArea(hex), sketchtest.WithinRel(1e-9))

	// The first edge is A'->G on the pinion and B'->I on the driving gear, and it
	// lies ON that gear's shaft axis: both its endpoints are at radius zero.
	first := lines[0]
	sketchtest.Measures(t, g.label+" shaft-axis edge start radius",
		g.radius(f.apex, bevPt{first.Start.X(), first.Start.Y()}), 0, sketchtest.Within(1e-9))
	sketchtest.Measures(t, g.label+" shaft-axis edge end radius",
		g.radius(f.apex, bevPt{first.End.X(), first.End.Y()}), 0, sketchtest.Within(1e-9))
	sketchtest.Measures(t, g.label+" shaft-axis edge length",
		bevPt{first.End.X() - first.Start.X(), first.End.Y() - first.Start.Y()}.norm(),
		math.Abs(g.station(f.apex, g.axisHeel)-g.station(f.apex, g.axisToe)),
		sketchtest.WithinRel(1e-9))
}

// stepBoreSketch builds one gear's `{gearLabel} Bore` sketch: the bore circle on
// the plane rooted at the shaft start, centred on the sketch origin because that
// plane's origin is on the axis.
//
// The centre is FIXED rather than made coincident to the sketch's origin point.
// A circle created at (0,0,0) does not reuse the sketch's originPoint — its
// centre is a free point that happens to sit there — and coincidenting it to the
// origin has been observed to throw VCS_SKETCH_SOLVING_FAILED on a
// setByDistanceOnPath plane ([PB-CIRCLE-CENTER]).
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	if p["boreEnable"] == 0 {
		proofkit.Unmodelled(t, "Enable Bore is unchecked, so the %s Bore sketch is not drawn at all "+
			"and there is no sketch to prove", g.label)
	}

	proofkit.Step(t, "the bore circle, centre fixed on the axis, with a diameter dimension")
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	bore := s.CreateCircle(centre, g.boreDiameter/2)
	s.AddConstraint(sketch.NewDiameter(bore, g.boreDiameter))

	proofkit.Step(t, "the region the through-cut consumes")
	sketchtest.Solve(t, s)
	report := sketchtest.Verify(t, s, sketch.WithProbe())
	region := sketchtest.SingleProfile(t, report)
	sketchtest.IsValidProfile(t, region)
	sketchtest.MeasuresProfileArea(t, region,
		math.Pi*g.boreDiameter*g.boreDiameter/4, sketchtest.WithinRel(1e-9))
	// The diameter the circle is dimensioned at is the RESOLVED one, and which of
	// the two branches resolved it is what this checks. A dialog value of 0 means
	// auto, and the auto value is the pitch diameter quarter capped by the Maximum
	// Bore Diameter; any other value is the user's own, carried through unchanged
	// and already known to be at or below that maximum. Re-deriving it at this step
	// instead would lose the cap and the bore would take the body's back face.
	user := map[string]float64{"Pinion": p["pinionBore"], "Driving": p["drivingBore"]}[g.label]
	want := math.Min(g.pitchDiameter/4, g.maxBore)
	if user != 0 {
		want = user
	}
	sketchtest.Measures(t, g.label+" resolved bore diameter",
		g.boreDiameter, want, sketchtest.WithinRel(1e-12))
	if g.boreDiameter > g.maxBore {
		t.Errorf("%s: the bore sketch is dimensioned at %.6g mm, above the Maximum Bore Diameter "+
			"%.6g mm", g.label, g.boreDiameter, g.maxBore)
	}
}

// stepSpiralTrace builds the `{gear} 2D Tooth Trace` sketch: the cutter circle
// and the genuine cutter arc, in the tangent-plane 2-D frame whose origin is the
// apex, whose x axis is the cone element and whose y axis is circumferential.
//
// This is the genuine three-point arc with a radius dimension — the case the
// tooth-top arc above is NOT, and the two must not be written from one template.
//
// WHERE THIS DEPARTS FROM FUSION. The trace sketch is deliberately left with free
// degrees of freedom in Fusion: its toe and heel endpoints are pinned by the
// three-point construction, not by endpoint dimensions, because dimensioning
// them over-constrains the solve against the cone-element plane. It is exempt
// from the module's gate for that reason. proofkit's gate waives nothing, so the
// proof pins each endpoint onto its own apex circle, which is what §6 of
// spiral-tooth-trace.md says the endpoints ARE: the toe point on the toe circle
// and the heel point on the heel circle. What that costs is that the free-DOF
// sketch Fusion actually authors is not the sketch solved here.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	f := newBevFigure(t, p)
	g := f.gearOf(p)
	sp := f.spiralOf(g)

	proofkit.Step(t, "the apex, the frame's origin")
	apex := s.CreatePoint(0, 0)
	s.Fix(apex)

	proofkit.Step(t, "the cutter circle, centre pinned, with a diameter dimension")
	cutterCentre := s.CreatePoint(sp.cx, sp.cy)
	s.Fix(cutterCentre)
	cutter := s.CreateCircle(cutterCentre, sp.cutter)
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*sp.cutter))

	proofkit.Step(t, "the cone element, the frame's own x axis, as a fixed reference direction")
	coneEnd := s.CreatePoint(sp.rMean, 0)
	s.Fix(coneEnd)
	coneElement := s.CreateLine(apex, coneEnd)
	coneElement.SetConstruction(true)

	proofkit.Step(t, "the trace arc, its centre coincident to the cutter's and its radius r_c")
	arcCentre := s.CreatePoint(sp.cx, sp.cy)
	toe := s.CreatePoint(sp.toe2d.X, sp.toe2d.Y)
	heel := s.CreatePoint(sp.heel2d.X, sp.heel2d.Y)
	arc := s.CreateArc(arcCentre, toe, heel)
	lo := sp.rToe - 0.06*sp.span
	hi := sp.rHeel + 0.06*sp.span

	// HOW THE TWO ENDPOINTS ARE PINNED, AND WHY NOT BY THEIR CONE DISTANCE.
	// Each end is a circle-circle intersection, and a circle pair meets in TWO
	// points, so pinning an end by its distance from the apex leaves the far branch
	// standing beside the near one: the engine's probe reports four configurations
	// for the pair and the gate refuses them. §6 of spiral-tooth-trace.md says to
	// keep the branch the mean point sits on, and that is a SIDE, which only a
	// signed constraint states. So each end is pinned by its signed angular
	// position ON THE CUTTER CIRCLE, measured from the cone element, which names one
	// point of that circle and no other. That the point so named really is the
	// circle-circle intersection is then asserted rather than assumed: the checks
	// below read each end's cone distance back and hold it to R_lo and R_hi.
	//
	// The arc entity already carries a structural row equalising its two endpoint
	// radii, so the pair takes six rows for six freedoms: the centre's coincident,
	// the radius dimension, that structural row, and one angle per end. A
	// point-on-circle on the far end is a seventh, and the engine reports it as
	// redundant.
	toeArm := s.CreateLine(arcCentre, toe)
	toeArm.SetConstruction(true)
	heelArm := s.CreateLine(arcCentre, heel)
	heelArm.SetConstruction(true)
	s.AddConstraint(
		sketch.NewCoincident(arcCentre, cutterCentre),
		sketch.NewRadius(arc, sp.cutter),
		sketch.NewAngle(coneElement, toeArm,
			bevSignedAngleDeg(bevPt{1, 0}, sp.toe2d.sub(bevPt{sp.cx, sp.cy}))),
		sketch.NewAngle(coneElement, heelArm,
			bevSignedAngleDeg(bevPt{1, 0}, sp.heel2d.sub(bevPt{sp.cx, sp.cy}))),
	)

	proofkit.Step(t, "the trace's own invariants")
	sketchtest.Solve(t, s)
	sketchtest.MeasuresPoint(t, arcCentre, sp.cx, sp.cy, sketchtest.Within(1e-9))
	sketchtest.MeasuresPoint(t, toe, sp.toe2d.X, sp.toe2d.Y, sketchtest.Within(1e-6))
	sketchtest.MeasuresPoint(t, heel, sp.heel2d.X, sp.heel2d.Y, sketchtest.Within(1e-6))
	for _, constraint := range s.Constraints() {
		sketchtest.Satisfies(t, constraint, sketchtest.Within(1e-6))
	}

	// The centre is r_c from the mean point, so the arc passes through it and is
	// tangent there to a line at psi to the cone element. That is the entire
	// dependence on psi, the hand and the cutter radius.
	sketchtest.Measures(t, g.label+" cutter centre distance from the mean point",
		math.Hypot(arcCentre.X()-sp.rMean, arcCentre.Y()), sp.cutter, sketchtest.WithinRel(1e-9))
	// The hand sign belongs on the cos term. Mirroring the hand must flip Cy and
	// change nothing else; putting the sign on Cx mirrors about x = R_mean, a
	// different curve that gives the two gears unequal twist.
	sketchtest.Measures(t, g.label+" cutter centre x, which the hand must NOT move",
		arcCentre.X(), sp.rMean-sp.cutter*math.Sin(f.spiralAngle), sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, g.label+" cutter centre y, which the hand flips",
		arcCentre.Y(), sp.handSign*sp.cutter*math.Cos(f.spiralAngle), sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, g.label+" toe end cone distance",
		math.Hypot(toe.X(), toe.Y()), lo, sketchtest.WithinRel(1e-9))
	sketchtest.Measures(t, g.label+" heel end cone distance",
		math.Hypot(heel.X(), heel.Y()), hi, sketchtest.WithinRel(1e-9))
}
