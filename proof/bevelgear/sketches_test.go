package bevelgear_test

// The sketch steps.
//
// Each one builds the sketch its step draws, in the order the step draws it,
// and then measures the SOLVED geometry against geometry_test.go's closed form.
// proofkit gates every case on the engine's own verdict: DOF 0 is necessary but
// not sufficient, and a scheme that still admits a mirrored answer fails there
// even though the seed avoided it.
//
// ---------------------------------------------------------------------------
// Two bench substitutions, stated once here and cited where they are used.
//
// 1. A DIRECTION-CARRYING RIGHT ANGLE REPLACES addPerpendicular / addParallel.
//    Fusion's addPerpendicular is unsigned: it admits the mirror image and the
//    SEED decides which one the solver lands on. The section 2 figure leans on
//    that in five places -- the grow side, the driving shaft's sense, the two
//    Apex 2 drops, the two dedendum lines, and the front faces -- and the spec
//    carries a warning at each. The bench engine refuses an ambiguous scheme
//    rather than seeding past it, so every one of those becomes a SIGNED angle
//    of the same arity: one row, the same freedom removed, with the side named
//    instead of seeded. The Fusion call stays addPerpendicular / addParallel,
//    and the step list names it; what the sign records is which of the two
//    answers the spec's seeding rule is there to reach.
//
//    This substitution costs the one thing the spec says the proof cannot show
//    anyway: it proves the constraints solve from a correct seed and never that
//    the module's seed is correct. A seed defect therefore reaches Fusion
//    untested, which is how the M->N seeding defect the spec records got there.
//
// 2. A COLLINEAR, AND THE CENTRE COINCIDENCE, LOSE THE ROW FUSION ABSORBS.
//    addCollinear carries two point-on-line rows, and where the new line's
//    start is already pinned to the reference line's own endpoint Fusion
//    absorbs one of them ([PB-COLLINEAR-CHAIN]). The bench engine counts both
//    and reports the pair as redundant, so the proof asserts the single row
//    that is not already implied. The same holds for the coincidence that pins
//    point I to the projected centre: the driving shaft is parallel to
//    Centre->Apex and shares the Apex with it, so it IS the line through the
//    centre, and one of the coincidence's two rows is already true. The proof
//    puts the centre on line I->J instead, which is the row that is not.
//
//    A proof cannot catch the collinear-chain defect for exactly this reason:
//    the substitution makes the right reading and the wrong one identical, and
//    only a Fusion session tells them apart.
// ---------------------------------------------------------------------------

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// ---------------------------------------------------------------------------
// Bench helpers.
// ---------------------------------------------------------------------------

// figure is one sketch under construction, with the section 2 naming the
// snapshots read back: each line carries its own name and its two endpoints
// carry that name with `.start` / `.end`, and the lattice's own vertices are
// renamed to the single letters instructions.md argues about.
type figure struct {
	s *sketch.Sketch
}

// seg draws one construction line from RAW COORDINATES, which is the coincident
// style section 2 requires: a line that must meet an existing point is built
// from its own fresh endpoints and pinned with exactly one coincident per end,
// never by sharing the existing SketchPoint ([BEVEL-F-COINCIDENT-STYLE]).
// Sharing leaves the sketch under-constrained; sharing AND coinciding fails the
// solve outright.
func (f figure) seg(name string, a, b planeVec) *sketch.Line {
	p1 := f.s.CreatePoint(a.X, a.Y)
	p1.SetName(name + ".start")
	p2 := f.s.CreatePoint(b.X, b.Y)
	p2.SetName(name + ".end")
	l := f.s.CreateLine(p1, p2)
	l.SetName(name)
	l.SetConstruction(true)
	return l
}

// pin is the one coincident per end the coincident style allows.
func (f figure) pin(p, to *sketch.Point) {
	f.s.AddConstraint(sketch.NewCoincident(p, to))
}

// vertex names a point as one of the lattice's own vertices. The snapshot
// drawing keeps exactly these names and clears the rest.
func vertex(p *sketch.Point, name string) *sketch.Point {
	p.SetName(name)
	return p
}

// rightAngle is substitution 1: a signed right angle in place of
// addPerpendicular. The sign comes from the closed form's own two directions,
// so it names the side the spec's seeding rule reaches.
func (f figure) rightAngle(l1, l2 *sketch.Line, d1, d2 planeVec) {
	a := 90.0
	if d1.cross(d2) < 0 {
		a = -90.0
	}
	f.s.AddConstraint(sketch.NewAngle(l1, l2, a))
}

// turnOf is substitution 1 for a parallel: the signed angle between the two
// directions, rounded to the multiple of 180 degrees they actually hold.
func (f figure) turnOf(l1, l2 *sketch.Line, d1, d2 planeVec) {
	a := 0.0
	if d1.dot(d2) < 0 {
		a = 180.0
	}
	f.s.AddConstraint(sketch.NewAngle(l1, l2, a))
}

// offsetTo is a signed parallel offset. The engine's offset is signed positive
// on the left of src's direction, and Fusion's addOffsetDimension is an
// unsigned perpendicular distance whose side the geometry already holds, so the
// sign here is read off the closed form rather than stated by the spec.
func (f figure) offsetTo(src, dst *sketch.Line, srcDir, srcStart, target planeVec, want float64) {
	d := srcDir.direction().cross(target.minus(srcStart))
	if d < 0 {
		want = -want
	}
	f.s.AddConstraint(sketch.NewOffset(src, dst, want))
}

// near fails the case when got is further from want than tol.
func near(t testing.TB, what string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %.9f, want %.9f (tolerance %.1e)", what, got, want, tol)
	}
}

// nearPoint fails the case when a solved point is further than tol from where
// the closed form puts it.
func nearPoint(t testing.TB, what string, got *sketch.Point, want planeVec, tol float64) {
	t.Helper()
	if d := math.Hypot(got.X()-want.X, got.Y()-want.Y); d > tol {
		t.Errorf("%s: solved at (%.6f, %.6f), closed form (%.6f, %.6f), off by %.3e",
			what, got.X(), got.Y(), want.X, want.Y, d)
	}
}

// solved reads a solved point back as a plane vector.
func solved(p *sketch.Point) planeVec { return pv(p.X(), p.Y()) }

// refuseDeclared skips a case this net cannot reach, naming it as the declared
// refusal the case table carries rather than a gap in the table.
func refuseDeclared(t testing.TB, p map[string]float64) {
	t.Helper()
	if p[keyDeclaredRefusal] != 0 {
		proofkit.Unmodelled(t, "declared refusal: this lattice's conditioning reads "+
			"%.4g, below the sketch engine's %g trust floor, at Shaft Angle %.0f "+
			"degrees with %g/%g teeth. The spec admits the configuration, so the "+
			"case stays in the table carrying this flag rather than the advertised "+
			"range being narrowed on one net's evidence",
			p[keyRefusalConditioning], 4e-5, p[keyShaftAngle],
			p[keyDrivingTeeth], p[keyPinionTeeth])
	}
}

// ---------------------------------------------------------------------------
// The per-gear frame inside the shared section 2 sketch.
// ---------------------------------------------------------------------------

// gearAxes places one gear's own lattice frame inside the shared sketch: Apex
// is the origin of both, U runs Apex -> that gear's shaft axis, and N runs from
// the axis toward Apex 2.
type gearAxes struct {
	Apex planeVec
	U, N planeVec
}

// at maps a point of latticeFrame into the shared sketch frame.
func (a gearAxes) at(p planeVec) planeVec {
	return a.Apex.plus(a.U.times(p.X)).plus(a.N.times(p.Y))
}

// dir maps a direction of latticeFrame into the shared sketch frame.
func (a gearAxes) dir(d planeVec) planeVec {
	return a.U.times(d.X).plus(a.N.times(d.Y))
}

// sketchFrame is the whole section 2 figure's placement: where the Apex sits
// above the projected centre and which way each shaft runs.
type sketchFrame struct {
	Centre  planeVec
	Anchor  planeVec // the projected anchor line's unit direction
	Perp    planeVec // the in-plane perpendicular, on the grow side
	Apex    planeVec
	Pinion  gearAxes
	Driving gearAxes
}

// frameOf places the figure the way section 2 does: the Apex at
// c + perp * (R cos(gamma_g) + resolved Driving Gear Base Height), the driving
// shaft running back down that same perpendicular, and the pinion shaft the
// driving direction rotated about the Apex by the Shaft Angle.
//
// The apex POSITION is sketch-local throughout ([BEVEL-F-APEX-LOCAL]); the only
// world reading section 2 is allowed is the target normal, as a one-bit choice
// of perp's sign ([BEVEL-F-GROW-SIDE]), which this frame takes as +perp.
func frameOf(c bevelCase) sketchFrame {
	centre := pv(0, 0)
	anchor := pv(1, 0)
	perp := perpOf(anchor)

	apexHeight := c.R*math.Cos(c.Driving.Gamma) + c.Driving.BaseHeight
	apex := centre.plus(perp.times(apexHeight))

	drivingU := perp.times(-1)

	// Both candidate pinion directions, kept apart by the rule the spec states:
	// form both, and keep the one whose point A has the greater X. Rotating one
	// fixed sense and flipping it only when its X comes out negative keeps the
	// wrong one whenever BOTH candidates have a positive X, which is exactly
	// the side flip to avoid.
	plus := rotate(drivingU, c.Sigma)
	minus := rotate(drivingU, -c.Sigma)
	along := c.R * math.Cos(c.Pinion.Gamma)
	pinionU := plus
	if apex.plus(minus.times(along)).X > apex.plus(plus.times(along)).X {
		pinionU = minus
	}

	a := apex.plus(pinionU.times(along))
	b := apex.plus(drivingU.times(c.R * math.Cos(c.Driving.Gamma)))

	return sketchFrame{
		Centre: centre, Anchor: anchor, Perp: perp, Apex: apex,
		Pinion:  gearAxes{Apex: apex, U: pinionU, N: towardSide(pinionU, b.minus(a))},
		Driving: gearAxes{Apex: apex, U: drivingU, N: towardSide(drivingU, a.minus(b))},
	}
}

// towardSide is the unit perpendicular to u that points to the same side as
// want. The two Apex 2 drops MUST aim at the same interior-wedge point: if one
// seeds Apex 2 on the wrong side, the coincidence that closes them makes the
// solver flip the whole frame to the mirror, the pinion dedendum collapses onto
// the driving one, and the conical end cut later finds no cone face at the toe.
func towardSide(u, want planeVec) planeVec {
	n := pv(-u.Y, u.X)
	if n.dot(want) < 0 {
		return n.times(-1)
	}
	return n
}

func rotate(v planeVec, a float64) planeVec {
	s, c := math.Sin(a), math.Cos(a)
	return pv(v.X*c-v.Y*s, v.X*s+v.Y*c)
}

// axesFor picks the frame of the gear a per-gear step is building.
func (f sketchFrame) axesFor(m sideMember) gearAxes {
	if m.Name == "Driving" {
		return f.Driving
	}
	return f.Pinion
}

// anchorReference is the projected anchor line: reference geometry, whose
// coordinates the solver never moves, which is what a projection is once
// section 2 reads a direction off it.
func anchorReference(s *sketch.Sketch, fr sketchFrame) *sketch.Line {
	a := fr.Centre.minus(fr.Anchor.times(5))
	b := fr.Centre.plus(fr.Anchor.times(5))
	p1 := s.CreateReferencePoint(a.X, a.Y, "Anchor Line")
	p2 := s.CreateReferencePoint(b.X, b.Y, "Anchor Line")
	l, err := s.CreateReferenceLine(p1, p2, "Anchor Line")
	if err != nil {
		panic("proof: the projected anchor line is not reference geometry: " + err.Error())
	}
	l.SetName("Anchor Line")
	return l
}

// shaftAngleTurn is the Shaft Angle as the signed turn from the driving shaft
// to the pinion shaft, in degrees.
func shaftAngleTurn(fr sketchFrame) float64 {
	return math.Atan2(fr.Driving.U.cross(fr.Pinion.U), fr.Driving.U.dot(fr.Pinion.U)) * 180 / math.Pi
}

// ---------------------------------------------------------------------------
// S4 -- the cone angles, the pitch cone distance and the per-gear bounds.
// ---------------------------------------------------------------------------

// stepConeGeometry proves the closed form the whole build is seeded from,
// before anything else is drawn: the two pitch cone angles, the Pitch Cone
// Distance R, and the per-gear windows the dialog resolves against.
//
// It draws the quadrilateral Apex / A / Apex 2 / B that section 2 closes --
// the two shaft axes at the Shaft Angle, each with the perpendicular drop of
// its own pitch radius, the two drops meeting at Apex 2 -- and then reads the
// cone angles OFF THE SOLVED FIGURE rather than restating them. That is the
// whole point: the closed form is what the module seeds the along-shaft lengths
// with, and a seed that disagrees with its own closure is a seed waiting to
// pick the wrong branch.
func stepConeGeometry(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	c := resolveCase(t, p)
	f := figure{s: s}
	fr := frameOf(c)

	proofkit.Step(t, "seed the two shaft axes at R cos(gamma), the closed form")
	apexPt := s.CreatePoint(fr.Apex.X, fr.Apex.Y)
	vertex(apexPt, "Apex")
	s.Fix(apexPt)

	alongP := c.R * math.Cos(c.Pinion.Gamma)
	alongG := c.R * math.Cos(c.Driving.Gamma)
	aSeed := fr.Pinion.at(pv(alongP, 0))
	bSeed := fr.Driving.at(pv(alongG, 0))

	pinionShaft := f.seg("Apex->A", fr.Apex, aSeed)
	f.pin(pinionShaft.Start, apexPt)
	drivingShaft := f.seg("Apex->B", fr.Apex, bSeed)
	f.pin(drivingShaft.Start, apexPt)
	pointA := vertex(pinionShaft.End, "A")
	pointB := vertex(drivingShaft.End, "B")

	proofkit.Step(t, "pin the driving shaft's direction and the Shaft Angle")
	// The driving shaft's direction is held against the projected anchor line,
	// exactly as section 2 holds it: an absolute Vertical leaves the figure free
	// to turn 180 degrees about the Apex, and the engine reports that as two
	// discrete configurations rather than seeding past it.
	anchorRef := anchorReference(s, fr)
	f.rightAngle(anchorRef, drivingShaft, fr.Anchor, fr.Driving.U)
	// The Shaft Angle is the traditional angle BETWEEN the two shaft axes, and
	// it is signed here for the reason at the top of this file: Fusion places
	// the text point inside the Sigma wedge so the dimension measures Sigma and
	// not its supplement, and the sign is what the bench has instead.
	s.AddConstraint(sketch.NewAngle(drivingShaft, pinionShaft, shaftAngleTurn(fr)))

	proofkit.Step(t, "drop each pitch radius perpendicular to its own shaft")
	apex2 := fr.Driving.at(pv(alongG, c.Driving.PitchRadius()))
	dropA := f.seg("A->Apex2", aSeed, apex2)
	f.pin(dropA.Start, pointA)
	f.rightAngle(pinionShaft, dropA, fr.Pinion.U, apex2.minus(aSeed))
	s.AddConstraint(sketch.NewDistance(dropA.Start, dropA.End, c.Pinion.PitchRadius()))

	dropB := f.seg("B->Apex2", bSeed, apex2)
	f.pin(dropB.Start, pointB)
	f.rightAngle(drivingShaft, dropB, fr.Driving.U, apex2.minus(bSeed))
	s.AddConstraint(sketch.NewDistance(dropB.Start, dropB.End, c.Driving.PitchRadius()))

	proofkit.Step(t, "close the two drops at Apex 2")
	f.pin(dropA.End, dropB.End)
	vertex(dropB.End, "Apex 2")

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("section 2's closing quadrilateral did not solve: %v", err)
	}

	proofkit.Step(t, "read the cone angles off the solved figure")
	apexV, a2 := solved(apexPt), solved(dropB.End)
	pitchDir := a2.minus(apexV).direction()
	gotGammaP := math.Acos(math.Max(-1, math.Min(1, pitchDir.dot(fr.Pinion.U))))
	gotGammaG := math.Acos(math.Max(-1, math.Min(1, pitchDir.dot(fr.Driving.U))))

	// The angle the pitch line makes with each shaft IS that gear's pitch cone
	// half angle, which is what tan(gamma_p) = sin(Sigma) PPD / (DPD + PPD
	// cos(Sigma)) and gamma_g = Sigma - gamma_p name.
	near(t, "pinion pitch cone angle", gotGammaP, c.Pinion.Gamma, 1e-9)
	near(t, "driving pitch cone angle", gotGammaG, c.Driving.Gamma, 1e-9)
	near(t, "the two cone angles sum to the Shaft Angle",
		gotGammaP+gotGammaG, c.Sigma, 1e-9)

	// R is the apex-to-heel length along the pitch cone, and it is NOT the Cone
	// Distance parameter: the two coincide exactly at Shaft Angle 90 for any
	// pair of tooth counts and diverge everywhere else.
	near(t, "pitch cone distance R", apexV.distance(a2), c.R, 1e-9)
	if math.Abs(c.Sigma-math.Pi/2) < 1e-12 {
		near(t, "Cone Distance is 2R at Shaft Angle 90", c.ConeDistance, 2*c.R, 1e-9)
	}

	proofkit.Step(t, "check the along-shaft seeds against the closure")
	near(t, "|Apex->A| closes at R cos(gamma_p)", apexV.distance(solved(pointA)), alongP, 1e-9)
	near(t, "|Apex->B| closes at R cos(gamma_g)", apexV.distance(solved(pointB)), alongG, 1e-9)

	proofkit.Step(t, "check each gear's admitted windows")
	for _, m := range []sideMember{c.Pinion, c.Driving} {
		// The base-height window is non-empty exactly when the tooth count
		// clears its own floor, which is why the tooth check runs first.
		if m.Teeth < m.MinTeeth() {
			t.Errorf("%s: the case table carries %g teeth, below this gear's own "+
				"computed floor of %.4f -- no base height satisfies both bounds",
				m.Name, m.Teeth, m.MinTeeth())
		}
		lo, hi := m.MinBaseHeight(c.Module), m.MaxBaseHeight(c.Module)
		if lo > hi {
			t.Errorf("%s: base-height window is empty, [%.4f, %.4f]", m.Name, lo, hi)
		}
		if m.BaseHeight < lo-1e-9 || m.BaseHeight > hi+1e-9 {
			t.Errorf("%s: resolved base height %.4f is outside its window [%.4f, %.4f]",
				m.Name, m.BaseHeight, lo, hi)
		}
		// The TRUE crossing is r tan(gamma): walking out along the dedendum
		// line from Apex 2's plane, the perpendicular distance to the shaft
		// axis falls at cos(gamma) per unit and the along-shaft coordinate
		// rises at sin(gamma). The bound sits 1.25 Module sin(gamma) below it,
		// because it starts from the dedendum corner instead of the pitch
		// point, and is therefore deliberately conservative rather than exact.
		crossing := m.PitchRadius() * math.Tan(m.Gamma)
		if hi >= crossing {
			t.Errorf("%s: the Maximum Base Height %.4f reaches the true crossing %.4f",
				m.Name, hi, crossing)
		}
	}

	proofkit.Step(t, "check the Shaft Angle against its computed ceiling")
	ceiling := maxShaftAngleOf(c.Driving.Teeth, c.Pinion.Teeth)
	if c.Sigma > ceiling+1e-12 {
		t.Errorf("Shaft Angle %.4f rad is at or above the computed ceiling %.4f rad",
			c.Sigma, ceiling)
	}
	// Both cosines must be positive for every angle the check admits, which is
	// what the ceiling is there to guarantee: a pitch cone angle reaching 90
	// degrees turns that gear's cone inside out, R cos(gamma) passes through
	// zero and changes sign, and the back-cone virtual radius is unbounded.
	if math.Cos(c.Pinion.Gamma) <= 0 || math.Cos(c.Driving.Gamma) <= 0 {
		t.Errorf("a cone angle reached 90 degrees: gamma_p %.4f, gamma_g %.4f",
			c.Pinion.Gamma, c.Driving.Gamma)
	}
}

// ---------------------------------------------------------------------------
// S8 -- the Anchor sketch.
// ---------------------------------------------------------------------------

// stepAnchorSketch proves the Anchor Line ends with zero free degrees of
// freedom.
//
// The line's absolute direction is arbitrary -- nothing downstream reads it,
// because section 2 derives every direction RELATIVE to the projected anchor
// line -- but it must not be a free rotation. Midpoint plus length plus a
// sketch-local horizontal is what takes it to zero; a world-axis lock would
// mis-orient the line on a tilted target plane ([PB-REFLINE-DIRECTION]).
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	f := figure{s: s}

	proofkit.Step(t, "project the user's centre point into the sketch")
	// A projected point is brought in associatively and is NOT fixed by the
	// projection ([PB-PROJECT-NOT-FIXED]); the bench models it as reference
	// geometry, whose coordinates the solver never moves.
	centre := s.CreateReferencePoint(0, 0, "Center Point")
	vertex(centre, "C")

	proofkit.Step(t, "draw the anchor line seeded at plus and minus 5 mm")
	// The seeded half length is 0.5 cm either side of the projected centre, so
	// the seeded length is 10 mm. The value is arbitrary: the dimension only
	// stops the length floating, and nothing downstream reads it.
	line := f.seg("Anchor Line", pv(-5, 0), pv(5, 0))
	line.SetConstruction(false)

	proofkit.Step(t, "pin the centre onto the line and at its midpoint")
	// Fusion takes BOTH addCoincident (the intersection, which puts the centre
	// ON the line) and addMidPoint (which makes it bisect), and absorbs the row
	// they share. The bench engine counts both and reports the pair as
	// redundant, so the proof keeps the midpoint alone -- substitution 2 at the
	// top of this file. The midpoint carries the point-on-line row inside it,
	// so the freedom removed is the same.
	s.AddConstraint(sketch.NewMidpoint(centre, line))

	proofkit.Step(t, "lock the seeded length and the sketch-local direction")
	// The horizontal is sketch-LOCAL, so it survives a tilted target plane; a
	// world-axis lock would mis-orient the line. Horizontal plus an unsigned
	// aligned length still leaves the line free to swap its two ends, which the
	// engine reports as two configurations, so the bench states the length as
	// the SIGNED horizontal distance the engine's own dimension takes. Fusion
	// gets the same direction from the seed and only abs(target) may reach a
	// parameter value there ([PB-DIM-VALUE-SEMANTICS]).
	s.AddConstraint(sketch.NewHorizontal(line))
	s.AddConstraint(sketch.NewHorizontalDistance(line.Start, line.End, 10))

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("the anchor sketch did not solve: %v", err)
	}
	near(t, "anchor line length", line.Length(), 10, 1e-9)
	near(t, "the projected centre bisects the line",
		solved(line.Start).distance(solved(centre)),
		solved(line.End).distance(solved(centre)), 1e-9)
	if s.DOF() != 0 {
		t.Errorf("the anchor sketch left %d free degree(s) of freedom", s.DOF())
	}
}

// ---------------------------------------------------------------------------
// S10 -- the Gear Profiles sketch, the section 2 lattice.
// ---------------------------------------------------------------------------

// latticeSide is the per-gear half of the lattice, kept so the driving half can
// be built by the same code as the pinion half and the two can then be
// measured against each other.
type latticeSide struct {
	member sideMember
	axes   gearAxes
	want   latticeFrame

	shaft, drop, ded, rootAxis     *sketch.Line
	lineAE, lineEG, lineCH, lineGH *sketch.Line
	lineMN, lineNFront             *sketch.Line

	ptAxis, ptDed, ptHeel           *sketch.Point
	ptFoot, ptBase, ptBack, ptTooth *sketch.Point
	ptToe, ptToeIn, ptFront         *sketch.Point
}

// stepGearProfiles builds the whole section 2 lattice: the two shaft axes off
// the Apex, the two perpendicular drops closing at Apex 2, the pitch line, both
// dedendum lines and root axes, each gear's module-length chain out to its heel
// edge, the two back-cone points and their Tooth Spacing offsets, and both toe
// ends with their front faces.
//
// Every line is drawn from RAW COORDINATES and pinned with one coincident per
// end -- the coincident style, which section 2 allows and sharing does not --
// and every one of them is construction geometry. The solid features later
// consume only the per-gear Profile sketches, never a section 2 curve.
//
// The two perpendiculars Fusion needs on G->H and I->J are deliberately ABSENT
// here, and that is a difference between the two engines rather than a choice.
// addOffsetDimension in Fusion is a distance dimension whose second entity must
// already be parallel to the first, and those two perpendiculars are what
// supply that parallelism. The bench engine's offset emits TWO rows, holding
// both endpoints of the target at the same signed perpendicular distance, so it
// carries the parallelism itself; adding the perpendicular here is a third row
// for the same two freedoms and the lattice comes back overconstrained at DOF 0
// with the two base-height offsets named as the redundant pair. Leaving it out
// is the right response, and weakening the gate is not.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	c := resolveCase(t, p)
	f := figure{s: s}
	fr := frameOf(c)

	proofkit.Step(t, "project the anchor sketch's centre point and its line")
	// The ANCHOR SKETCH's centre point is what section 2 projects, never the raw
	// user-selected point: projecting the anchor-sketch point keeps the chain
	// inside the Design component, where a cross-component reference can
	// resolve inconsistently.
	centre := s.CreateReferencePoint(fr.Centre.X, fr.Centre.Y, "Anchor centre")
	anchorRef := anchorReference(s, fr)

	proofkit.Step(t, "draw Centre->Apex perpendicular to the projected anchor line")
	// The apex POSITION is sketch-local: c + perp * (R cos(gamma_g) + the
	// resolved Driving Gear Base Height). Seeding it at the Driving Gear Pitch
	// Diameter instead, which an earlier revision of the spec said, puts the
	// seed 11.6 mm past where the solve lands it on the default pair, and a
	// seed that disagrees with its own closure by that margin is a seed waiting
	// to pick the wrong branch.
	centerToApex := f.seg("Center->Apex", fr.Centre, fr.Apex)
	f.pin(centerToApex.Start, centre)
	f.rightAngle(anchorRef, centerToApex, fr.Anchor, fr.Perp)
	apexPt := vertex(centerToApex.End, "Apex")
	// No length constraint on this line: it is pinned later, by putting the
	// projected centre back on the driving gear's own I->J edge.

	proofkit.Step(t, "draw both shaft axes off the Apex at the Shaft Angle")
	alongP := c.R * math.Cos(c.Pinion.Gamma)
	alongG := c.R * math.Cos(c.Driving.Gamma)
	aSeed := fr.Pinion.at(pv(alongP, 0))
	bSeed := fr.Driving.at(pv(alongG, 0))

	drivingShaft := f.seg("Apex->B", fr.Apex, bSeed)
	f.pin(drivingShaft.Start, apexPt)
	// Parallel to Centre->Apex and NEVER addVertical: a vertical forces the
	// line to the sketch's world vertical, which is wrong on a tilted target
	// plane and mis-orients the figure.
	f.turnOf(centerToApex, drivingShaft, fr.Perp, fr.Driving.U)

	pinionShaft := f.seg("Apex->A", fr.Apex, aSeed)
	f.pin(pinionShaft.Start, apexPt)
	s.AddConstraint(sketch.NewAngle(drivingShaft, pinionShaft, shaftAngleTurn(fr)))

	pointA := vertex(pinionShaft.End, "A")
	pointB := vertex(drivingShaft.End, "B")

	proofkit.Step(t, "drop each pitch radius to Apex 2, both aimed into the wedge")
	// BOTH drops must aim at the same interior-wedge point. If the B->Apex 2
	// drop seeds Apex 2 on the wrong side while A's seeds it on the right one,
	// the coincidence that closes them flips the WHOLE frame to the mirror: A
	// jumps across, the pinion dedendum collapses onto the driving one, and the
	// conical end cut later finds no cone face at the toe midpoint.
	apex2Seed := fr.Driving.at(pv(alongG, c.Driving.PitchRadius()))
	dropA := f.seg("A->Apex2", aSeed, apex2Seed)
	f.pin(dropA.Start, pointA)
	f.rightAngle(pinionShaft, dropA, fr.Pinion.U, apex2Seed.minus(aSeed))
	s.AddConstraint(sketch.NewDistance(dropA.Start, dropA.End, c.Pinion.PitchRadius()))

	dropB := f.seg("B->Apex2", bSeed, apex2Seed)
	f.pin(dropB.Start, pointB)
	f.rightAngle(drivingShaft, dropB, fr.Driving.U, apex2Seed.minus(bSeed))
	s.AddConstraint(sketch.NewDistance(dropB.Start, dropB.End, c.Driving.PitchRadius()))

	f.pin(dropA.End, dropB.End)
	apex2Pt := vertex(dropB.End, "Apex 2")

	proofkit.Step(t, "draw the pitch line and both dedendum lines")
	pitchLine := f.seg("Apex->Apex 2", fr.Apex, apex2Seed)
	f.pin(pitchLine.Start, apexPt)
	f.pin(pitchLine.End, apex2Pt)
	pitchDir := apex2Seed.minus(fr.Apex)

	wantPinion := latticeOf(c, c.Pinion)
	wantDriving := latticeOf(c, c.Driving)

	pinion := latticeSide{member: c.Pinion, axes: fr.Pinion, want: wantPinion,
		shaft: pinionShaft, drop: dropA}
	driving := latticeSide{member: c.Driving, axes: fr.Driving, want: wantDriving,
		shaft: drivingShaft, drop: dropB}
	pinion.ptAxis, driving.ptAxis = pointA, pointB

	// The line drawn AWAY from the anchor line is the Pinion Gear Dedendum and
	// its end is C; the one drawn TOWARD it is the Driving Gear Dedendum and its
	// end is D. The two are mirror images about Apex 2 at equal tooth counts,
	// so the bench states the side with a signed angle rather than leaving the
	// seed to pick -- substitution 1.
	for _, side := range []*latticeSide{&pinion, &driving} {
		name, endName := "Apex2->C", "C"
		if side.member.Name == "Driving" {
			name, endName = "Apex2->D", "D"
		}
		dedSeed := side.axes.at(side.want.Ded)
		side.ded = f.seg(name, apex2Seed, dedSeed)
		f.pin(side.ded.Start, apex2Pt)
		f.rightAngle(pitchLine, side.ded, pitchDir, dedSeed.minus(apex2Seed))
		s.AddConstraint(sketch.NewDistance(side.ded.Start, side.ded.End, 1.25*c.Module))
		side.ptDed = vertex(side.ded.End, endName)
	}

	proofkit.Step(t, "draw both root axes, Apex->C and Apex->D")
	for _, side := range []*latticeSide{&pinion, &driving} {
		name := "Apex->C"
		if side.member.Name == "Driving" {
			name = "Apex->D"
		}
		side.rootAxis = f.seg(name, fr.Apex, side.axes.at(side.want.Ded))
		f.pin(side.rootAxis.Start, apexPt)
		f.pin(side.rootAxis.End, side.ptDed)
	}

	proofkit.Step(t, "build each gear's module chain out to its heel edge")
	for _, side := range []*latticeSide{&pinion, &driving} {
		buildModuleChain(t, f, c, side)
	}

	proofkit.Step(t, "pin the figure by putting the projected centre on I->J")
	// Fusion's addCoincident(I, projected centre) is two rows and it absorbs the
	// one the driving shaft's own parallel already implies -- the shaft passes
	// through the centre, because it is parallel to Centre->Apex and shares the
	// Apex with it. The bench counts both, so the proof states only the row that
	// is not implied. This is the one constraint that fixes the Apex's height.
	s.AddConstraint(sketch.NewPointOnLine(centre, driving.lineIJ()))

	proofkit.Step(t, "pin each back-cone point K / L and its Tooth Spacing offset")
	for _, side := range []*latticeSide{&pinion, &driving} {
		buildToothCentre(f, c, side)
	}

	proofkit.Step(t, "draw both toe lines and the front faces that hold them")
	for _, side := range []*latticeSide{&pinion, &driving} {
		buildToeEnd(f, c, side)
	}

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("the section 2 lattice did not solve: %v", err)
	}

	assertLattice(t, c, fr, pinion, driving, apexPt, apex2Pt, centre)
}

// lineIJ is the driving side's heel edge, the one the projected centre is put
// back onto.
func (s latticeSide) lineIJ() *sketch.Line { return s.lineGH }

// buildModuleChain draws A->E, C->E, E->G, C->H and G->H for one gear (B->F,
// D->F, F->I, D->J and I->J on the driving side), and sets the base-height
// offset that drives the heel edge.
func buildModuleChain(t testing.TB, f figure, c bevelCase, side *latticeSide) {
	names := [5]string{"A->E", "C->E", "E->G", "C->H", "G->H"}
	ends := [3]string{"E", "G", "H"}
	if side.member.Name == "Driving" {
		names = [5]string{"B->F", "D->F", "F->I", "D->J", "I->J"}
		ends = [3]string{"F", "I", "J"}
	}
	at := side.axes.at
	footSeed, baseSeed, heelSeed := at(side.want.Foot), at(side.want.Base), at(side.want.Heel)

	// The module length is a SEED and is never dimensioned: E is DRIVEN to the
	// foot of the perpendicular from C onto the shaft axis, which is one
	// dedendum's along-shaft projection past A rather than one module past it
	// ([BEVEL-F-DRIVEN-DIMS]).
	side.lineAE = f.seg(names[0], at(side.want.Axis), footSeed)
	f.pin(side.lineAE.Start, side.ptAxis)
	// A collinear here would assert a row the start's own coincident already
	// holds, so the proof states the single point-on-line row that is not
	// implied -- substitution 2. The collinear NAMES the shaft axis Apex->A,
	// because that is the line A->E's start actually sits on.
	f.s.AddConstraint(sketch.NewPointOnLine(side.lineAE.End, side.shaft))
	side.ptFoot = vertex(side.lineAE.End, ends[0])

	lineCE := f.seg(names[1], at(side.want.Ded), footSeed)
	f.pin(lineCE.Start, side.ptDed)
	f.pin(lineCE.End, side.ptFoot)
	f.rightAngle(side.lineAE, lineCE, footSeed.minus(at(side.want.Axis)),
		footSeed.minus(at(side.want.Ded)))

	// E->G is collinear with A->E and NEVER with the shaft axis further up the
	// chain, even though both describe the same infinite line: naming the axis
	// raises VCS_SKETCH_OVER_CONSTRAINTS in Fusion.
	side.lineEG = f.seg(names[2], footSeed, baseSeed)
	f.pin(side.lineEG.Start, side.ptFoot)
	f.s.AddConstraint(sketch.NewPointOnLine(side.lineEG.End, side.lineAE))
	side.ptBase = vertex(side.lineEG.End, ends[1])

	// C->H is collinear with the dedendum line Apex2->C that C is the endpoint
	// of, for the same reason.
	side.lineCH = f.seg(names[3], at(side.want.Ded), heelSeed)
	f.pin(side.lineCH.Start, side.ptDed)
	f.s.AddConstraint(sketch.NewPointOnLine(side.lineCH.End, side.ded))
	side.ptHeel = vertex(side.lineCH.End, ends[2])

	side.lineGH = f.seg(names[4], baseSeed, heelSeed)
	f.pin(side.lineGH.Start, side.ptBase)
	f.pin(side.lineGH.End, side.ptHeel)

	// The base-height offset is what drives the heel edge toward the shaft
	// axis, and it is the value AFTER this gear's own Maximum Base Height has
	// been applied. The two gears have different pitch cone angles whenever
	// their tooth counts differ, so the driving cap does not imply the pinion's.
	f.offsetTo(side.drop, side.lineGH,
		at(side.want.Apex2).minus(at(side.want.Axis)), at(side.want.Axis),
		baseSeed, side.member.BaseHeight)
	if side.member.BaseHeight <= 0 {
		t.Fatalf("%s: resolved base height is %g", side.member.Name, side.member.BaseHeight)
	}
}

// buildToothCentre draws G->K and C->K (I->L and D->L), pins K on both lines it
// is the intersection of, and applies the Tooth Spacing offset.
func buildToothCentre(f figure, c bevelCase, side *latticeSide) {
	names := [4]string{"G->K", "C->K", "K->K'", "C->K'"}
	endName := "K"
	if side.member.Name == "Driving" {
		names = [4]string{"I->L", "D->L", "L->L'", "D->L'"}
		endName = "L"
	}
	at := side.axes.at
	backSeed := at(side.want.Back)

	lineGK := f.seg(names[0], at(side.want.Base), backSeed)
	f.pin(lineGK.Start, side.ptBase)
	// By the time K is added, G and C are already fixed, so a collinear here
	// over-constrains and Fusion errors. The two point-on-line coincidents
	// locate K exactly, as the intersection of the shaft axis with the dedendum
	// line, without over-constraining.
	f.s.AddConstraint(sketch.NewPointOnLine(lineGK.End, side.shaft))
	f.s.AddConstraint(sketch.NewPointOnLine(lineGK.End, side.ded))
	side.ptBack = vertex(lineGK.End, endName)

	lineCK := f.seg(names[1], at(side.want.Ded), backSeed)
	f.pin(lineCK.Start, side.ptDed)
	f.pin(lineCK.End, side.ptBack)

	if c.ToothSpacing <= 0 {
		// At Tooth Spacing 0 nothing is built here: K' IS K and the existing
		// C->K reference line is reused. A zero-length dimensioned line would
		// be degenerate, and one segment gets one line.
		side.ptTooth = side.ptBack
		return
	}
	toothSeed := at(side.want.Tooth)
	lineKKp := f.seg(names[2], backSeed, toothSeed)
	f.pin(lineKKp.Start, side.ptBack)
	f.s.AddConstraint(sketch.NewPointOnLine(lineKKp.End, side.ded))
	f.s.AddConstraint(sketch.NewDistance(lineKKp.Start, lineKKp.End, c.ToothSpacing))
	side.ptTooth = lineKKp.End
	side.ptTooth.SetName(endName + "'")

	lineCKp := f.seg(names[3], at(side.want.Ded), toothSeed)
	f.pin(lineCKp.Start, side.ptDed)
	f.pin(lineCKp.End, side.ptTooth)
}

// buildToeEnd draws M->N, M->C, the front face N->A' and the shaft edge A'->G
// (O->P, O->D, P->B' and B'->I on the driving side).
func buildToeEnd(f figure, c bevelCase, side *latticeSide) {
	names := [4]string{"M->N", "M->C", "N->A'", "A'->G"}
	ends := [3]string{"M", "N", "A'"}
	if side.member.Name == "Driving" {
		names = [4]string{"O->P", "O->D", "P->B'", "B'->I"}
		ends = [3]string{"O", "P", "B'"}
	}
	at := side.axes.at
	toeSeed, toeInSeed, frontSeed := at(side.want.Toe), at(side.want.ToeIn), at(side.want.Front)

	// BOTH ends are seeded at their closed-form solved positions. N is slid
	// from the M seed along C->H by (M's perpendicular distance from the shaft
	// axis minus this gear's Toe Radius) / cos(gamma). Sliding by the Root
	// Length instead, or by the distance from M to A, puts the N seed PAST the
	// shaft axis -- measured at -0.27 mm against a solved +5.17 mm on the
	// shipped default at a 50% Toe Extension -- and because the front face's
	// length dimension is unsigned the solver then converges happily onto the
	// mirror and Fusion refuses the revolve with ASM_WIRE_X_AXIS.
	//
	// THE PROOF CANNOT CATCH A WRONG SEED HERE. It seeds M and N at the closed
	// form, which is the rule, so it proves the constraints solve from a
	// correct seed and never that the module's seed is correct. A seed defect
	// therefore reaches Fusion untested, which is how the one above got there.
	// This is the honest edge of what this stage checks.
	side.lineMN = f.seg(names[0], toeSeed, toeInSeed)
	f.s.AddConstraint(sketch.NewPointOnLine(side.lineMN.Start, side.rootAxis))
	// Fusion adds addParallel(M->N, C->H) and then the offset dimension; the
	// bench offset carries the parallelism in its own two rows, so the parallel
	// is left out here for the reason the file header gives.
	f.offsetTo(side.lineCH, side.lineMN,
		at(side.want.Heel).minus(at(side.want.Ded)), at(side.want.Ded),
		toeSeed, c.RootLength*c.R/c.RootDistance)
	side.ptToe = vertex(side.lineMN.Start, ends[0])
	side.ptToeIn = vertex(side.lineMN.End, ends[1])

	lineMC := f.seg(names[1], toeSeed, at(side.want.Ded))
	f.pin(lineMC.Start, side.ptToe)
	f.pin(lineMC.End, side.ptDed)

	// N is NOT pinned to the A->Apex2 drop, and it is never pinned to the shaft
	// axis either: that would put it ON the axis of revolution and the conical
	// split fails with ASM_API_FAILED for asymmetric tooth counts. A' sits on
	// the axis and is a FOOT, not a corner; N never does, because the Toe
	// Radius is strictly positive.
	side.lineNFront = f.seg(names[2], toeInSeed, frontSeed)
	f.pin(side.lineNFront.Start, side.ptToeIn)
	f.s.AddConstraint(sketch.NewPointOnLine(side.lineNFront.End, side.shaft))
	f.rightAngle(side.shaft, side.lineNFront, side.axes.U, frontSeed.minus(toeInSeed))
	f.s.AddConstraint(sketch.NewDistance(side.lineNFront.Start, side.lineNFront.End,
		side.member.ToeRadius))
	side.ptFront = vertex(side.lineNFront.End, ends[2])

	// A' replaces A as the hexagon's first vertex. At Toe Extension 0 with a
	// defaulted Toe Radius the two coincide exactly, so nothing moves.
	shaftEdge := f.seg(names[3], frontSeed, at(side.want.Base))
	f.pin(shaftEdge.Start, side.ptFront)
	f.pin(shaftEdge.End, side.ptBase)
}

// assertLattice measures the solved lattice against the closed form, and then
// against the spec's own bounds: the Maximum Face Width, which cannot be
// evaluated until A, B, C, D, H and J exist and are SOLVED, and the toe window.
func assertLattice(t testing.TB, c bevelCase, fr sketchFrame,
	pinion, driving latticeSide, apexPt, apex2Pt, centre *sketch.Point) {
	t.Helper()

	// The lattice is tens of millimetres across and the solver converges to its
	// own residual, so a micron is the scale a genuine placement error shows up
	// at and rounding never does.
	const tol = 1e-6

	nearPoint(t, "Apex", apexPt, fr.Apex, tol)
	near(t, "the projected centre stays where the anchor put it",
		solved(centre).distance(fr.Centre), 0, tol)

	// Apex 2 is the one point both halves close on, so reading it in each
	// gear's own frame is what says the two drops aimed at the SAME interior
	// wedge point rather than at mirror images of it.
	for _, side := range []latticeSide{pinion, driving} {
		nearPoint(t, side.member.Name+" Apex 2", apex2Pt, side.axes.at(side.want.Apex2), tol)
		nearPoint(t, side.member.Name+" axis point", side.ptAxis, side.axes.at(side.want.Axis), tol)
		nearPoint(t, side.member.Name+" dedendum corner", side.ptDed, side.axes.at(side.want.Ded), tol)
		nearPoint(t, side.member.Name+" heel corner", side.ptHeel, side.axes.at(side.want.Heel), tol)
		nearPoint(t, side.member.Name+" module foot", side.ptFoot, side.axes.at(side.want.Foot), tol)
		nearPoint(t, side.member.Name+" shaft-edge heel", side.ptBase, side.axes.at(side.want.Base), tol)
		nearPoint(t, side.member.Name+" back-cone point", side.ptBack, side.axes.at(side.want.Back), tol)
		nearPoint(t, side.member.Name+" tooth centre", side.ptTooth, side.axes.at(side.want.Tooth), tol)
		nearPoint(t, side.member.Name+" toe corner", side.ptToe, side.axes.at(side.want.Toe), tol)
		nearPoint(t, side.member.Name+" inner toe corner", side.ptToeIn, side.axes.at(side.want.ToeIn), tol)
		nearPoint(t, side.member.Name+" front-face foot", side.ptFront, side.axes.at(side.want.Front), tol)
	}

	apex := solved(apexPt)
	for _, side := range []latticeSide{pinion, driving} {
		m := side.member
		axis := solved(side.ptAxis)
		ded, heel := solved(side.ptDed), solved(side.ptHeel)
		toe, toeIn, front := solved(side.ptToe), solved(side.ptToeIn), solved(side.ptFront)

		near(t, m.Name+" pitch radius at the heel",
			distancePointLine(solved(apex2Pt), apex, axis), m.PitchRadius(), tol)
		near(t, m.Name+" dedendum length", solved(apex2Pt).distance(ded), 1.25*c.Module, tol)

		// The base height is the offset between the A->Apex2 drop and G->H, so
		// it is measured from APEX 2's plane and not from the dedendum point.
		// Reading it one dedendum out is the mistake this assertion exists to
		// catch.
		near(t, m.Name+" base height offset",
			distancePointLine(heel, axis, solved(apex2Pt)), m.BaseHeight, tol)

		// The back-cone point K / L sits at the virtual pitch radius from Apex
		// 2, which is what makes the tooth drawn there reach the back cone.
		// The virtual tooth NUMBER is still taken from the closed form and
		// never measured here.
		near(t, m.Name+" back-cone distance Apex2->K",
			solved(apex2Pt).distance(solved(side.ptBack)), m.VirtualPitchRadius(), tol)
		near(t, m.Name+" Tooth Spacing offset",
			solved(side.ptBack).distance(solved(side.ptTooth)), c.ToothSpacing, tol)

		// The toe end. |Ded->Toe| IS the resolved Root Length, and the toe must
		// sit nearer the Apex than the heel: the toe-versus-heel order follows
		// from the frame being built correctly, and a negative span inverts
		// everything downstream with no error.
		near(t, m.Name+" root length |Ded->Toe|", ded.distance(toe), c.RootLength, tol)
		if apex.distance(toe) >= apex.distance(heel) {
			t.Errorf("%s: the toe is not nearer the Apex than the heel (%.4f vs %.4f)",
				m.Name, apex.distance(toe), apex.distance(heel))
		}

		// The front face stands square to the shaft at the Toe Radius, and N is
		// strictly OFF the axis.
		near(t, m.Name+" toe radius", toeIn.distance(front), m.ToeRadius, tol)
		near(t, m.Name+" inner toe corner rides the toe-radius line",
			distancePointLine(toeIn, apex, axis), m.ToeRadius, tol)
		near(t, m.Name+" front-face foot sits ON the shaft axis",
			distancePointLine(front, apex, axis), 0, tol)
		if m.ToeRadius <= 0 {
			t.Errorf("%s: resolved Toe Radius %.6f is not strictly positive, so N "+
				"lands on the axis of revolution", m.Name, m.ToeRadius)
		}

		// Every hexagon vertex must stay on one side of the axis of revolution,
		// which is what [PB-REVOLVE] refuses a profile for crossing.
		for i, v := range []planeVec{front, solved(side.ptBase), heel, ded, toe, toeIn} {
			if side.axes.N.dot(v.minus(apex)) < -tol {
				t.Errorf("%s: hexagon vertex %d crossed the shaft axis", m.Name, i)
			}
		}
	}

	// The Maximum Face Width, from SOLVED geometry: the smaller of the
	// perpendicular distance from A to line C->H and from B to line D->J. Both
	// are computed and the smaller kept, because either gear can be the binding
	// side -- written with the pinion's diameter by name it is wrong whenever
	// the driving gear carries the smaller tooth count.
	fromA := distancePointLine(solved(pinion.ptAxis), solved(pinion.ptDed), solved(pinion.ptHeel))
	fromB := distancePointLine(solved(driving.ptAxis), solved(driving.ptDed), solved(driving.ptHeel))
	gotMax := 0.95 * math.Min(fromA, fromB)
	near(t, "Maximum Face Width from solved geometry", gotMax, c.MaxFaceWidth, 1e-6)
	if c.FaceWidth > c.MaxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.6f exceeds its cap %.6f", c.FaceWidth, c.MaxFaceWidth)
	}

	// The toe window. A Toe Radius at or above this gear's own ceiling leaves
	// the Toe Extension nowhere to go, and the spec rejects a positive Toe
	// Extension on such a pair rather than silently substituting a smaller
	// radius.
	for _, m := range []sideMember{c.Pinion, c.Driving} {
		ceiling := toeRadiusCeilingOf(m, c.Module, c.R, c.FaceWidth)
		if m.ToeRadius >= ceiling && c.RootLength > c.RootLength0+1e-12 {
			t.Errorf("%s: Toe Radius %.4f is at or above its ceiling %.4f, so a "+
				"positive Toe Extension should have been rejected", m.Name, m.ToeRadius, ceiling)
		}
	}
	if c.RootLength < c.RootLength0-1e-12 {
		t.Errorf("the Toe Extension shortened the root length, %.6f below %.6f",
			c.RootLength, c.RootLength0)
	}
	limit := math.Min(
		toeLimitOf(c.Pinion, c.Module, c.R, c.RootDistance),
		toeLimitOf(c.Driving, c.Module, c.R, c.RootDistance))
	if c.RootLength > limit-1e-12 && limit > c.RootLength0 {
		t.Errorf("the root length %.6f reached the smaller Toe Limit %.6f, where the "+
			"toe face has closed to nothing and the toe trim has no cone to find",
			c.RootLength, limit)
	}
}

// ---------------------------------------------------------------------------
// S12 -- the virtual spur tooth profile.
// ---------------------------------------------------------------------------

// stepToothProfile draws the tooth the borrowed spur generator draws on this
// gear's own plane, at the virtual tooth number section 3 computes and with the
// root sink applied.
//
// Three things this step exists to hold:
//
//   - THE VIRTUAL TOOTH NUMBER IS A REAL NUMBER AND IS NEVER ROUNDED. Every
//     drawn circle is rebuilt from it, so a floored count draws the tooth
//     smaller than the back cone places it: on the shipped default the exact
//     virtual pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm,
//     and the addendum the tooth works over falls to 0.5797 mm against a
//     nominal 1.0 module. The count reaches the drawer only as the angular half
//     thickness pi / (2 z), which at the real count leaves the standard
//     pi * Module / 2 of tooth at the pitch circle -- the same thickness on
//     both members of an unequal pair, which an integer count does not give.
//
//   - THE ROOT SINK MOVES THE ROOT CIRCLE AND NOTHING ELSE, and it is the
//     generated module's own figure rather than a proof-only offset.
//
//   - THE EMBEDDED FLAG IS TAKEN FROM THE SUNK ROOT RADIUS, and the loop's line
//     count follows from it deterministically. wantLines is 0 when embedded and
//     2 otherwise, never "0 or 2": an unrelated loop between the drawn circles
//     can carry the same two NURBS and two arcs with the other line count, and
//     selecting it makes the apex loft fail with LOFT_NO_TOOLBODY.
//
// The sketch is gated normally here even though Fusion exempts it. The
// exemption covers the four circle LABELS, which hold a degree of freedom of
// their own, and nothing else: the geometry reads fully constrained on its own,
// and reading the exemption as licence for loose geometry is what let a
// deformed tooth ship once already.
func stepToothProfile(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	c := resolveCase(t, p)
	m := memberFor(c, p)
	tc := circlesFor(c, m)

	proofkit.Step(t, "%s: virtual tooth number %.6f, never rounded", m.Name, tc.VirtualTeeth)
	centre := s.CreatePoint(0, 0)
	centre.SetName(m.Name + " tooth centre")
	s.Fix(centre)

	proofkit.Step(t, "draw the four circles the proxy is asked for")
	// Pitch, base and tip are the plain spur formulas at the virtual count; the
	// root is the only one the sink moves, and it moves by the sink on radius.
	for _, circle := range []struct {
		name   string
		radius float64
	}{
		{"pitch", tc.Pitch},
		{"base", tc.Base},
		{"tip", tc.Tip},
		{"root", tc.Root},
	} {
		drawn := s.CreateCircle(centre, circle.radius)
		drawn.SetName(m.Name + " " + circle.name)
		drawn.SetConstruction(true)
		s.AddConstraint(sketch.NewDiameter(drawn, 2*circle.radius))
		near(t, m.Name+" "+circle.name+" circle radius", drawn.R(), circle.radius, 1e-9)
	}
	// The tip circle is dimensioned at virtualPitchRadius + Module, which is the
	// only place this proof reads a tip radius off a drawn figure. No case reads
	// one off a JOINED body, because no case joins.
	near(t, m.Name+" tip circle is virtualPitchRadius + Module",
		tc.Tip, m.VirtualPitchRadius()+c.Module, 1e-9)
	near(t, m.Name+" root circle is one dedendum in, less the sink",
		tc.Root, m.VirtualPitchRadius()-1.25*c.Module-c.RootSink, 1e-9)
	near(t, m.Name+" root sink", tc.Sink, rootSinkShare*2.25*c.Module, 1e-12)

	proofkit.Step(t, "draw both flanks, already rotated 180 degrees by the drawer")
	left, right := tc.flankSamples()
	if tc.Embedded {
		// The flank starts inside the root circle, so tip, root and flanks meet
		// with no connecting lines and each flank is trimmed at the root.
		left, right = trimFlank(left, tc.Root), trimFlank(right, tc.Root)
	}
	// Each flank sample is a point the generator COMPUTES rather than one the
	// solver places, so the bench equivalent of placing it is a fixed point --
	// with one exception per arc, below.
	leftSpline := drawFlank(t, s, m.Name+" left flank", left)
	rightSpline := drawFlank(t, s, m.Name+" right flank", right)

	// Put the flank at the smaller bearing first so the arcs below sweep the
	// short way, through the tooth's own centreline, rather than the long way
	// round the gear.
	lowFlank, highFlank := right, left
	lowSpline, highSpline := rightSpline, leftSpline
	if bearing(right[0]) > bearing(left[0]) {
		lowFlank, highFlank = left, right
		lowSpline, highSpline = leftSpline, rightSpline
	}

	proofkit.Step(t, "close the loop with the tip arc, the root arc and the stubs")
	// Each arc gets its OWN centre point, which is what Fusion's
	// addByCenterStartEnd does: it shares the start and end and COPIES the
	// centre, so the arc's centre is a fresh free point with nothing tying it to
	// the one that was passed. Leaving it that way is the defect fusion.md
	// records -- the bevel tooth-top arc's centre stranded 22.9 mm behind its
	// origin and gave a 0.5743 mm arc where 22.5 mm was intended, on a sketch
	// that raised no error -- and the fix is an explicit coincident.
	//
	// That coincident is two rows and the arc's own equal-radius row already
	// holds one of them: the two ends are mirror images about the tooth's
	// centreline, so any point equidistant from both lies ON that centreline.
	// Fusion absorbs the repeat; the bench counts it, so the proof states only
	// the row that is not implied -- substitution 2.
	tipArc := s.CreateArc(arcCentre(s, m.Name+" tip arc centre", centre),
		splineEnd(lowSpline, len(lowFlank)-1), splineEnd(highSpline, len(highFlank)-1))
	tipArc.SetName(m.Name + " tip arc")

	var lines int
	var rootStart, rootEnd *sketch.Point
	if tc.Embedded {
		rootStart = splineEnd(highSpline, 0)
		rootEnd = splineEnd(lowSpline, 0)
	} else {
		// The two flank-to-root lines are RADIAL, which is what lets the whole
		// tooth rotate with the drawer's angle argument. On the shipped default
		// pair at Module 1 they are 0.0405 mm each: the sink drops the root
		// circle below the BASE circle there, so the tooth that would otherwise
		// be embedded is drawn NON-embedded, with these two lines.
		lines = 2
		highStub := radialStub(s, m.Name+" left stub", splineEnd(highSpline, 0), tc.Root)
		lowStub := radialStub(s, m.Name+" right stub", splineEnd(lowSpline, 0), tc.Root)
		rootStart, rootEnd = highStub, lowStub
	}
	// Fusion derives the root arc by profile-splitting the solid root circle;
	// the bench draws that same derived boundary explicitly and keeps the root
	// circle as construction.
	rootArc := s.CreateArc(arcCentre(s, m.Name+" root arc centre", centre), rootEnd, rootStart)
	rootArc.SetName(m.Name + " root arc")

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: the tooth profile did not solve: %v", m.Name, err)
	}

	proofkit.Step(t, "check the loop the profile finder selects on")
	near(t, m.Name+" tip arc radius", tipArc.R(), tc.Tip, 1e-6)
	near(t, m.Name+" root arc radius", rootArc.R(), tc.Root, 1e-6)
	// The centre gap is the reading that catches a stranded arc centre. With
	// the coincident in place both arcs read 0.000000 mm of gap; without it the
	// tooth-top arc came out at a fortieth of its intended radius.
	near(t, m.Name+" tip arc centre gap", solved(tipArc.Center).distance(solved(centre)), 0, 1e-9)
	near(t, m.Name+" root arc centre gap", solved(rootArc.Center).distance(solved(centre)), 0, 1e-9)
	if got := tc.wantLines(); got != lines {
		t.Errorf("%s: embedded=%v asks for %d connecting line(s), the loop carries %d",
			m.Name, tc.Embedded, got, lines)
	}
	assertToothLoop(t, s, m.Name, lines)

	// The tooth thickness at the pitch circle is what the real virtual count
	// buys. The drawer rotates each flank so its pitch crossing lands at
	// pi / (2 z), so the two crossings are 2 * pi / (2 z) apart in angle, and
	// at the pitch radius r_v = z * Module / 2 that arc is exactly
	// pi * Module / 2 -- the standard tooth thickness, the same one a spur gear
	// of this module carries. An INTEGER count gives pi * r_v / round(z)
	// instead, which misses nominal by a different amount on each member of an
	// unequal pair.
	near(t, m.Name+" tooth thickness at the pitch circle",
		tc.Pitch*math.Pi/tc.VirtualTeeth, math.Pi*c.Module/2, 1e-9)
}

// trimFlank drops the samples inside the root circle and pulls the first onto
// it, which is what an embedded tooth's flank does.
func trimFlank(flank []involutePt, root float64) []involutePt {
	out := make([]involutePt, 0, len(flank))
	for _, p := range flank {
		if math.Hypot(p.X, p.Y) >= root {
			out = append(out, p)
		}
	}
	if len(out) == 0 {
		return flank
	}
	scale := root / math.Hypot(out[0].X, out[0].Y)
	out[0] = involutePt{X: out[0].X * scale, Y: out[0].Y * scale}
	return out
}

// drawFlank creates one flank as a spline through its computed samples. Each
// sample is a point the generator COMPUTES rather than one the solver places,
// so the bench equivalent is a fixed point.
func drawFlank(t testing.TB, s *sketch.Sketch, name string, flank []involutePt) *sketch.Spline {
	t.Helper()
	pts := make([]*sketch.Point, 0, len(flank))
	for _, p := range flank {
		q := s.CreatePoint(p.X, p.Y)
		q.SetName(name)
		s.Fix(q)
		pts = append(pts, q)
	}
	sp, err := s.CreateSpline(pts...)
	if err != nil {
		t.Fatalf("%s: %v", name, err)
	}
	sp.SetName(name)
	return sp
}

// splineEnd returns the spline's i-th control point, which a clamped spline
// passes through at its two ends.
func splineEnd(sp *sketch.Spline, i int) *sketch.Point { return sp.Control[i] }

// radialStub draws one flank-to-root line, radially inward from the flank's
// root end to the root circle.
func radialStub(s *sketch.Sketch, name string, from *sketch.Point, root float64) *sketch.Point {
	th := math.Atan2(from.Y(), from.X())
	to := s.CreatePoint(root*math.Cos(th), root*math.Sin(th))
	to.SetName(name)
	s.Fix(to)
	l := s.CreateLine(from, to)
	l.SetName(name)
	return to
}

func bearing(p involutePt) float64 { return math.Atan2(p.Y, p.X) }

// arcCentre is the fresh centre point Fusion's addByCenterStartEnd copies,
// pinned by the one row the arc's own equal-radius row does not already hold.
// The tooth is drawn along the sketch's negative X, so its centreline is the
// horizontal through the centre; the equal-radius row puts the arc's centre on
// that centreline and this row puts it on the vertical through the tooth
// centre, which meet at exactly one point.
func arcCentre(s *sketch.Sketch, name string, toothCentre *sketch.Point) *sketch.Point {
	p := s.CreatePoint(toothCentre.X(), toothCentre.Y())
	p.SetName(name)
	s.AddConstraint(sketch.NewVerticalPoints(toothCentre, p))
	return p
}

// assertToothLoop checks the curve counts the profile finder selects on, on the
// loop the sketch actually formed. Restating the numbers in a comment is not
// proving them.
func assertToothLoop(t testing.TB, s *sketch.Sketch, label string, wantLines int) {
	t.Helper()
	var best *sketch.Profile
	for _, prof := range s.Profiles() {
		if !prof.Valid {
			continue
		}
		if best == nil || prof.Area > best.Area {
			best = prof
		}
	}
	if best == nil {
		t.Fatalf("%s: the tooth sketch formed no valid region", label)
		return
	}
	var splines, arcs, lines int
	for _, e := range best.Entities {
		switch e.(type) {
		case *sketch.Spline:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Errorf("%s: the tooth loop carries %d spline(s), %d arc(s) and %d line(s); "+
			"the profile finder asks for exactly 2, 2 and %d",
			label, splines, arcs, lines, wantLines)
	}
}

// ---------------------------------------------------------------------------
// S15 -- the per-gear Profile sketch, the revolved hexagon.
// ---------------------------------------------------------------------------

// stepProfileHexagon builds ONE gear's frustum profile on the axial plane: the
// six section 2 vertices recreated as new points at their exact positions, the
// closed hexagon drawn sharing those points, and the points fixed AFTER the
// lines exist.
//
// The order is load-bearing. Projecting section 2's points instead would leave
// this sketch under-constrained, because a projection is a reference and not a
// fix; and fixing a bare point BEFORE it is consumed as a line endpoint does
// not leave the sketch fully constrained either. Recreate, share, then fix.
//
// One profile sketch per gear, so sketch.profiles holds exactly this one
// hexagon loop: drawing both gears' hexagons in the shared sketch would leave
// two identically shaped loops to disambiguate.
//
// The hexagon's FIRST edge is the gear's shaft axis, and every body operation
// below uses that edge -- the revolve, the pattern, the bore plane and the
// meshing rotation -- never the section 2 Apex->A / Apex->B construction line,
// which is collinear with it but lives in a different sketch. The edge has to
// be fixed well enough to carry a trustworthy world position: a free edge
// resolves against a default frame and silently moves the body onto world XY.
func stepProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	c := resolveCase(t, p)
	m := memberFor(c, p)
	fr := frameOf(c)
	axes := fr.axesFor(m)
	want := latticeOf(c, m)

	names := [6]string{"A'", "G", "H", "C", "M", "N"}
	if m.Name == "Driving" {
		names = [6]string{"B'", "I", "J", "D", "O", "P"}
	}

	proofkit.Step(t, "%s: recreate the six section 2 vertices at their exact positions",
		m.Name)
	loop := want.profileLoop()
	pts := make([]*sketch.Point, 0, 6)
	for i, v := range loop {
		w := axes.at(v)
		q := s.CreatePoint(w.X, w.Y)
		q.SetName(names[i])
		pts = append(pts, q)
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points")
	lines := make([]*sketch.Line, 0, 6)
	for i := range pts {
		l := s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		l.SetName(names[i] + "->" + names[(i+1)%len(names)])
		lines = append(lines, l)
	}

	proofkit.Step(t, "fix the endpoints now that the lines exist")
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: the profile sketch did not solve: %v", m.Name, err)
	}

	proofkit.Step(t, "check the loop the revolve takes")
	var valid []*sketch.Profile
	for _, prof := range s.Profiles() {
		if prof.Valid {
			valid = append(valid, prof)
		}
	}
	if len(valid) != 1 {
		t.Fatalf("%s: the profile sketch holds %d valid region(s), want exactly 1 -- "+
			"the revolve takes the single profile without filtering", m.Name, len(valid))
		return
	}
	if n := len(valid[0].Outer); n != 6 {
		t.Errorf("%s: the hexagon's outer loop carries %d edge(s), want 6", m.Name, n)
	}
	near(t, m.Name+" hexagon area", valid[0].Area, polygonArea(loop), 1e-6)

	// The FIRST edge is the shaft axis: both its ends sit on the line through
	// the Apex and this gear's axis point, and it is the edge every body
	// operation reads a world position off.
	apex, axisPt := axes.at(want.Apex), axes.at(want.Axis)
	near(t, m.Name+" shaft edge start lies on the shaft axis",
		distancePointLine(solved(lines[0].Start), apex, axisPt), 0, 1e-9)
	near(t, m.Name+" shaft edge end lies on the shaft axis",
		distancePointLine(solved(lines[0].End), apex, axisPt), 0, 1e-9)

	// The profile must stay on ONE side of the axis of revolution. A profile
	// that crosses it self-intersects the axis and Fusion aborts the revolve
	// with ASM_WIRE_X_AXIS, which is what the Maximum Face Width, the Maximum
	// Base Height and the strictly positive Toe Radius are all there to stop.
	for i, q := range pts {
		if axes.N.dot(solved(q).minus(apex)) < -1e-9 {
			t.Errorf("%s: hexagon vertex %s crossed the axis of revolution",
				m.Name, names[i])
		}
	}
	if !s.EntityIsFullyConstrained(lines[0]) {
		t.Errorf("%s: the shaft-axis edge is not fully constrained, so its world "+
			"position is not trustworthy", m.Name)
	}
}

// ---------------------------------------------------------------------------
// S30 -- the bore sketch.
// ---------------------------------------------------------------------------

// stepBoreSketch draws the bore circle on the plane rooted at the shaft edge's
// start, where the sketch origin is already on the axis.
//
// A circle's centre is FREE even when it is created at the origin: the creation
// call does not reuse the sketch's own origin point. So the centre is fixed and
// a diameter dimension is added, which is 2 plus 1 freedoms removed; a
// coincident to the origin point instead has been observed to fail the solve
// outright on a plane built along a path.
func stepBoreSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	refuseDeclared(t, p)
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if p[keyBoreEnable] == 0 {
		proofkit.Unmodelled(t, "%s: Enable Bore is unchecked, so this step is skipped "+
			"entirely and authors no sketch at all", m.Name)
	}

	proofkit.Step(t, "%s: bore diameter %.4f mm", m.Name, m.BoreDiameter)
	centre := s.CreatePoint(0, 0)
	centre.SetName(m.Name + " bore centre")
	s.Fix(centre)
	circle := s.CreateCircle(centre, m.BoreDiameter/2)
	circle.SetName(m.Name + " bore")
	s.AddConstraint(sketch.NewDiameter(circle, m.BoreDiameter))

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: the bore sketch did not solve: %v", m.Name, err)
	}
	near(t, m.Name+" bore diameter", 2*circle.R(), m.BoreDiameter, 1e-9)
	// A bore diameter of 0 means auto-calculate, and auto is this gear's own
	// Pitch Diameter / 4 -- never the other gear's.
	if p[keyPinionBore] == 0 && p[keyDrivingBore] == 0 {
		near(t, m.Name+" auto bore is this gear's Pitch Diameter / 4",
			m.BoreDiameter, m.PitchDiameter/4, 1e-12)
	}
	if len(s.Profiles()) != 1 {
		t.Errorf("%s: the bore sketch holds %d region(s), want 1", m.Name, len(s.Profiles()))
	}
}
