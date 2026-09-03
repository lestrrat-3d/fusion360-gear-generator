package bevelgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ---------------------------------------------------------------------------
// The shared Gear Profiles frame.
//
// The sketch is drawn on the Gear Profiles plane, which contains the projected
// anchor line and is perpendicular to the target plane. Inside it the proof
// puts the projected anchor centre at the origin, the projected anchor line
// along +X and the grow side ([BEVEL-F-GROW-SIDE], chosen in Python from the
// target-plane normal) along +Y. That is a choice of coordinates, not a
// constraint: the anchor line's absolute direction is arbitrary and section 2
// derives every direction relative to it.
// ---------------------------------------------------------------------------

// sideGeometry is one gear's half of the lattice in the shared frame.
type sideGeometry struct {
	AxisDir vec2 // unit Apex -> A (pinion) / Apex -> B (driving)
	RadDir  vec2 // unit direction of increasing distance from that shaft axis
	DedDir  vec2 // unit Apex2 -> dedendum corner -> heel corner

	Axis  vec2 // A / B
	Base  vec2 // G / I
	Apex2 vec2 // the shared Apex 2
	Ded   vec2 // C / D
	Heel  vec2 // H / J
	Toe   vec2 // M / O
	ToeIn vec2 // N / P
	Foot  vec2 // E / F
	Back  vec2 // K / L
	Tooth vec2 // K' / L'
}

// sharedGeometry is the whole section 2 figure in closed form.
type sharedGeometry struct {
	Centre  vec2
	Apex    vec2
	Pinion  sideGeometry
	Driving sideGeometry
}

// sharedLattice places both gears in the shared frame. The Apex sits at
// c + perp * (R cos gamma_g + resolved Driving Gear Base Height), which is
// where the Apex 2 closure and the "constrain point I with the centre point"
// step drive it — the proof asserts that closure rather than assuming it.
func sharedLattice(d design) sharedGeometry {
	sigma := d.Sigma
	perp := v2(0, 1)
	centre := v2(0, 0)
	apex := centre.add(perp.scale(d.R*math.Cos(d.Driving.Gamma) + d.Driving.BaseHeight))

	// The driving shaft axis runs from the Apex back toward the anchor line.
	drivingDir := perp.scale(-1)
	// The pinion shaft axis is the driving direction rotated about the Apex by
	// the Shaft Angle. Both senses are formed and the one whose endpoint has
	// the greater X in this sketch is kept, exactly as section 2 says.
	plus := rotate2(drivingDir, sigma)
	minus := rotate2(drivingDir, -sigma)
	pinionDir := plus
	if minus.X > plus.X {
		pinionDir = minus
	}

	// Apex 2 lies in the interior wedge between the two shaft axes, so each
	// gear's radial direction is the one that points at it.
	pinionRad := pinionDir.leftNormal().scale(-1)
	drivingRad := drivingDir.leftNormal()

	return sharedGeometry{
		Centre:  centre,
		Apex:    apex,
		Pinion:  placeSide(apex, pinionDir, pinionRad, gearLattice(d.Pinion, d.Module, d.R, d.FaceWidth, d.ToothSpacing)),
		Driving: placeSide(apex, drivingDir, drivingRad, gearLattice(d.Driving, d.Module, d.R, d.FaceWidth, d.ToothSpacing)),
	}
}

func placeSide(apex, axisDir, radDir vec2, f gearFrame) sideGeometry {
	at := func(p vec2) vec2 {
		return apex.add(axisDir.scale(p.X)).add(radDir.scale(p.Y))
	}
	return sideGeometry{
		AxisDir: axisDir,
		RadDir:  radDir,
		DedDir:  axisDir.scale(f.DedDir.X).add(radDir.scale(f.DedDir.Y)),
		Axis:    at(f.Axis), Base: at(f.Base), Apex2: at(f.Apex2),
		Ded: at(f.Ded), Heel: at(f.Heel), Toe: at(f.Toe), ToeIn: at(f.ToeIn),
		Foot: at(f.Foot), Back: at(f.Back), Tooth: at(f.Tooth),
	}
}

// ---------------------------------------------------------------------------
// Building the lattice in the sketch engine.
// ---------------------------------------------------------------------------

// latticeBuilder carries the sketch and the named handles section 2's later
// steps reach back for. Each named line is created ONCE and reused
// ([BEVEL-F-LINE-ONCE]); the builder is what makes that structural rather than
// a matter of discipline.
type latticeBuilder struct {
	t testing.TB
	s *sketch.Sketch
	g sharedGeometry
	d design

	anchorLine *sketch.Line
	centre     *sketch.Point

	apex, apex2 *sketch.Point
	pin, drv    *sideHandles

	centreToApex, pitchLine *sketch.Line
}

// sideHandles are one gear's named points and lines.
type sideHandles struct {
	ShaftAxis *sketch.Line // Apex -> A / Apex -> B
	Drop      *sketch.Line // A -> Apex2 / B -> Apex2, the PPD/2 resp. DPD/2 drop
	Dedendum  *sketch.Line // Apex2 -> C / Apex2 -> D
	RootAxis  *sketch.Line // Apex -> C / Apex -> D
	AxisToDed *sketch.Line // A -> E / B -> F
	DedToFoot *sketch.Line // C -> E / D -> F
	FootToEnd *sketch.Line // E -> G / F -> I
	DedToHeel *sketch.Line // C -> H / D -> J
	EndToHeel *sketch.Line // G -> H / I -> J
	ToeLine   *sketch.Line // M -> N / O -> P
	ToothRef  *sketch.Line // C -> K' / D -> L'

	Axis, Base, Ded, Heel, Foot, Toe, ToeIn, Back, Tooth *sketch.Point
}

// seg is the section 2 line constructor: every line is created from RAW
// coordinates and each endpoint that meets an existing point is pinned with
// exactly one coincident ([BEVEL-F-COINCIDENT-STYLE]). Sharing a SketchPoint
// instead leaves the sketch under-constrained in Fusion; sharing AND
// coinciding fails the solve outright. The short reference and connector lines
// are built the same way — no section 2 line is exempt.
func (b *latticeBuilder) seg(name string, startSeed, endSeed vec2, pinStart, pinEnd *sketch.Point) *sketch.Line {
	p1 := b.s.CreatePoint(startSeed.X, startSeed.Y)
	p2 := b.s.CreatePoint(endSeed.X, endSeed.Y)
	p1.SetName(name + ".start")
	p2.SetName(name + ".end")
	l := b.s.CreateLine(p1, p2)
	l.SetName(name)
	// Every line drawn in the Gear Profiles sketch is a construction line: the
	// solid features consume only the per-gear Profile sketches.
	l.SetConstruction(true)
	if pinStart != nil {
		b.add(name+" start coincident", sketch.NewCoincident(p1, pinStart))
	}
	if pinEnd != nil {
		b.add(name+" end coincident", sketch.NewCoincident(p2, pinEnd))
	}
	return l
}

func (b *latticeBuilder) add(name string, c sketch.Constraint) {
	b.s.AddConstraint(c)
	b.s.SetConstraintName(c, name)
}

// length applies an aligned distance dimension to a whole line. Every section 2
// length dimension is AlignedDimensionOrientation: the figure has no
// axis-aligned line in it, so a horizontal or vertical orientation would
// dimension a projection instead of the length.
func (b *latticeBuilder) length(name string, l *sketch.Line, mm float64) {
	dim := sketch.NewDistance(l.Start, l.End, mm)
	if err := dim.SetValue(units.Millimeters(mm)); err != nil {
		b.t.Fatalf("%s: %v", name, err)
	}
	b.add(name, dim)
}

// angle applies a signed angular dimension, measured counter-clockwise from
// l1's start-to-end direction to l2's. The value is read off the closed-form
// directions rather than written by hand, so the sign records the side the
// spec's own seed rule picks.
//
// Fusion has no signed angular dimension: addAngularDimension is unsigned and
// picks its wedge from the text point ([PB-ANGULAR-DIM]), and which side the
// geometry lands on is held by the seed. The engine's angle is signed and so
// admits a single configuration. That substitution is what lets this proof
// gate on the ambiguity probe: measured, the unsigned reading leaves the
// section 2 net with a second configuration in which the driving dedendum
// corner D collapses onto the pinion's C — the exact frame inversion the
// spec's warnings describe — and the proof would have to waive the probe to
// pass. The cost is that the proof cannot check the text-point placement
// Fusion needs to select the same side; only a Fusion session can.
func (b *latticeBuilder) angle(name string, l1, l2 *sketch.Line, dir1, dir2 vec2) {
	target := math.Atan2(dir1.cross(dir2), dir1.dot(dir2))
	dim := sketch.NewAngle(l1, l2, 0)
	if err := dim.SetValue(units.Radians(target)); err != nil {
		b.t.Fatalf("%s: %v", name, err)
	}
	b.add(name, dim)
}

// offset applies a parallel-offset dimension. The engine's Offset is SIGNED
// (positive on the left of src's start-to-end direction) where Fusion's
// addOffsetDimension takes a magnitude and captures the side from the seeded
// geometry ([PB-DIM-VALUE-SEMANTICS]), so the sign is read off the closed-form
// position of the point the offset drives rather than written by hand.
func (b *latticeBuilder) offset(name string, src, dst *sketch.Line, magnitude float64, driven vec2) {
	signed := magnitude
	if signedDistanceFromLine(b.pointAt(src.Start), b.pointAt(src.End), driven) < 0 {
		signed = -magnitude
	}
	dim := sketch.NewOffset(src, dst, signed)
	if err := dim.SetValue(units.Millimeters(signed)); err != nil {
		b.t.Fatalf("%s: %v", name, err)
	}
	b.add(name, dim)
}

func (b *latticeBuilder) pointAt(p *sketch.Point) vec2 { return v2(p.X(), p.Y()) }

func solved(p *sketch.Point) vec2 { return v2(p.X(), p.Y()) }

// buildLattice draws the whole Gear Profiles sketch. It follows section 2 in
// order, so a reader can walk the two side by side.
//
// Two constraints section 2 names are deliberately NOT emitted here, and both
// omissions are arity differences between the two engines rather than choices:
//
//   - "Constrain line E->G and H->G with a perpendicular constraint" (and its
//     driving twin F->I / J->I). Fusion needs it because addOffsetDimension
//     requires the second entity to be a line already parallel to the first,
//     and it controls only the perpendicular distance — one equation. The
//     engine's Offset emits TWO rows, holding both endpoints of the target at
//     the same signed distance, so it carries the parallelism itself. Adding
//     the perpendicular as well is a third row for two freedoms: measured, the
//     lattice comes back at DOF 0 with 2 redundant constraints and the engine
//     names the two base-height offsets as the redundant pair. Section 2 says
//     in as many words to leave it out here and never to weaken the gate.
//   - "addParallel(M->N, C->H)" (and O->P / D->J). Same arity difference: the
//     offset already forces the two lines parallel, so the explicit parallel
//     is one redundant row. In Fusion the toe line is freshly drawn at an
//     arbitrary angle and the parallel is what makes addOffsetDimension legal
//     at all ([PB-OFFSET-DIM]).
//
// Three further substitutions replace a Fusion constraint with the single
// independent row it carries, each noted at its call site: the two collinear
// chains, the Apex 2 angular dimension, and the "constrain point I with the
// centre point" closure.
func buildLattice(t testing.TB, s *sketch.Sketch, d design) *latticeBuilder {
	t.Helper()
	g := sharedLattice(d)
	b := &latticeBuilder{t: t, s: s, g: g, d: d}

	// The projected anchor centre and the projected anchor line. Section 1
	// leaves the Anchor sketch fully constrained, so its projection into this
	// sketch is reference geometry here: a projected curve tracks its source
	// and carries no freedom of its own once that source is rigid. Fusion's
	// own projection does keep free DOF ([PB-PROJECT-NOT-FIXED]), which is why
	// the per-gear Profile sketches recreate their vertices instead; section 2
	// reaches full constraint through the net below rather than through the
	// projection, so modelling it as locked is faithful to what the net has to
	// do and costs the proof nothing it could otherwise check.
	b.centre = s.CreateReferencePoint(0, 0, "Anchor sketch centre point")
	b.centre.SetName("projected centre")
	half := 5.0 // the Anchor Line is seeded at +-0.5 cm about its centre
	as := s.CreateReferencePoint(-half, 0, "Anchor Line start")
	ae := s.CreateReferencePoint(half, 0, "Anchor Line end")
	as.SetName("anchor line start")
	ae.SetName("anchor line end")
	anchor, err := s.CreateReferenceLine(as, ae, "Anchor Line")
	if err != nil {
		t.Fatalf("project the anchor line: %v", err)
	}
	anchor.SetName("projected anchor line")
	anchor.SetConstruction(true)
	b.anchorLine = anchor

	proofkit.Step(t, "centre -> Apex, perpendicular to the projected anchor line")
	b.centreToApex = b.seg("centre->Apex", g.Centre, g.Apex, b.centre, nil)
	b.apex = b.centreToApex.End
	b.apex.SetName("Apex")
	b.add("centre->Apex perpendicular to anchor", sketch.NewPerpendicular(b.centreToApex, anchor))

	proofkit.Step(t, "the two shaft axes and the Shaft Angle")
	b.pin = &sideHandles{}
	b.drv = &sideHandles{}

	// Driving Gear Shaft Axis: from the Apex back toward the anchor line,
	// seeded at apex - perp*(R cos gamma_g) which is one resolved driving base
	// height above the projected centre. It is PARALLEL to centre->Apex, never
	// addVertical: a world-vertical lock mis-orients the figure on a tilted
	// target plane.
	b.drv.ShaftAxis = b.seg("Apex->B", g.Apex, g.Driving.Axis, b.apex, nil)
	b.drv.Axis = b.drv.ShaftAxis.End
	b.drv.Axis.SetName("B")
	b.add("Apex->B parallel to centre->Apex", sketch.NewParallel(b.drv.ShaftAxis, b.centreToApex))

	b.pin.ShaftAxis = b.seg("Apex->A", g.Apex, g.Pinion.Axis, b.apex, nil)
	b.pin.Axis = b.pin.ShaftAxis.End
	b.pin.Axis.SetName("A")
	// Fusion's addAngularDimension is unsigned and picks its wedge from the
	// text point ([PB-ANGULAR-DIM]); which SIDE the pinion lands on is held by
	// the seed rule above plus the Apex 2 closure. The engine's angle is
	// signed and so admits a single configuration, which is the substitution:
	// it does in one row what Fusion does with an unsigned dimension and a
	// seeded branch, and it is why this proof can gate on the ambiguity probe
	// at all. Measured counter-clockwise from Apex->B to Apex->A.
	b.angle("Shaft Angle", b.drv.ShaftAxis, b.pin.ShaftAxis,
		g.Driving.AxisDir, g.Pinion.AxisDir)

	proofkit.Step(t, "the two perpendicular drops closing at Apex 2")
	// Each drop leaves its own shaft axis toward the OTHER shaft axis, so both
	// aim at the same interior-wedge point. Choosing a drop's sense against
	// the grow direction instead is degenerate on the driving side, whose
	// shaft axis IS that direction, and seeds Apex 2 on the wrong side; the
	// closing coincident then flips the whole frame to its mirror.
	b.pin.Drop = b.seg("A->Apex2", g.Pinion.Axis, g.Pinion.Apex2, b.pin.Axis, nil)
	b.angle("A->Apex2 perpendicular to Apex->A", b.pin.ShaftAxis, b.pin.Drop,
		g.Pinion.AxisDir, g.Pinion.RadDir)
	b.length("A->Apex2 = PPD/2", b.pin.Drop, d.Pinion.PitchDiameter/2)

	b.drv.Drop = b.seg("B->Apex2", g.Driving.Axis, g.Driving.Apex2, b.drv.Axis, nil)
	b.angle("B->Apex2 perpendicular to Apex->B", b.drv.ShaftAxis, b.drv.Drop,
		g.Driving.AxisDir, g.Driving.RadDir)
	b.length("B->Apex2 = DPD/2", b.drv.Drop, d.Driving.PitchDiameter/2)

	b.add("Apex 2 closure", sketch.NewCoincident(b.pin.Drop.End, b.drv.Drop.End))
	b.apex2 = b.pin.Drop.End
	b.apex2.SetName("Apex 2")

	proofkit.Step(t, "pitch line and the two dedendum lines")
	b.pitchLine = b.seg("Apex->Apex2", g.Apex, g.Pinion.Apex2, b.apex, b.apex2)

	b.drv.Dedendum = b.seg("Apex2->D", g.Driving.Apex2, g.Driving.Ded, b.apex2, nil)
	b.drv.Ded = b.drv.Dedendum.End
	b.drv.Ded.SetName("D")
	b.angle("Apex2->D perpendicular to the Pitch Line", b.pitchLine, b.drv.Dedendum,
		g.Pinion.Apex2.sub(g.Apex), g.Driving.DedDir)
	b.length("Apex2->D = 1.25*Module", b.drv.Dedendum, 1.25*d.Module)

	b.pin.Dedendum = b.seg("Apex2->C", g.Pinion.Apex2, g.Pinion.Ded, b.apex2, nil)
	b.pin.Ded = b.pin.Dedendum.End
	b.pin.Ded.SetName("C")
	b.angle("Apex2->C perpendicular to the Pitch Line", b.pitchLine, b.pin.Dedendum,
		g.Pinion.Apex2.sub(g.Apex), g.Pinion.DedDir)
	b.length("Apex2->C = 1.25*Module", b.pin.Dedendum, 1.25*d.Module)

	b.drv.RootAxis = b.seg("Apex->D", g.Apex, g.Driving.Ded, b.apex, b.drv.Ded)
	b.pin.RootAxis = b.seg("Apex->C", g.Apex, g.Pinion.Ded, b.apex, b.pin.Ded)

	proofkit.Step(t, "the module extensions, heel edges and base-height offsets")
	b.buildSide(b.pin, g.Pinion, d.Pinion, "pinion", "E", "G", "H", "A")
	b.buildSide(b.drv, g.Driving, d.Driving, "driving", "F", "I", "J", "B")

	// "Constrain Point I with center point." Fusion's addCoincident carries
	// two rows, and the chain already implies one of them: I sits on the
	// driving shaft axis, which is the perpendicular to the anchor line
	// through the projected centre, so I's coordinate ACROSS the anchor line
	// is fixed before this constraint is added. The engine counts both rows
	// and reports one redundant, so the proof keeps the single independent
	// row — I lies on the projected anchor line — and then ASSERTS that the
	// solved I lands on the projected centre, which is the closure the
	// coincident is really there to state.
	b.add("point I on the projected anchor line", sketch.NewPointOnLine(b.drv.Base, anchor))

	proofkit.Step(t, "back-cone points K and L and the tooth centres")
	b.buildToothCentre(b.pin, g.Pinion, "K")
	b.buildToothCentre(b.drv, g.Driving, "L")

	proofkit.Step(t, "the toe lines M->N and O->P at the resolved Face Width")
	b.buildToe(b.pin, g.Pinion, d, "M", "N")
	b.buildToe(b.drv, g.Driving, d, "O", "P")

	// The hexagon's first edge, drawn last on each side: A->G and B->I.
	b.seg("A->G", g.Pinion.Axis, g.Pinion.Base, b.pin.Axis, b.pin.Base)
	b.seg("B->I", g.Driving.Axis, g.Driving.Base, b.drv.Axis, b.drv.Base)

	return b
}

// buildSide draws one gear's module extensions (A->E / B->F, E->G / F->I), its
// heel edge (C->H / D->J, G->H / I->J) and its base-height offset.
func (b *latticeBuilder) buildSide(h *sideHandles, g sideGeometry, gr gear, label, foot, base, heel, axis string) {
	// The module-length extension is a SEED length only; the driven length is
	// 1.25*Module*sin(gamma), fixed by the perpendicular from the dedendum
	// corner. Do not dimension it ([BEVEL-F-DRIVEN-DIMS]).
	footSeed := g.Axis.add(g.AxisDir.scale(b.d.Module))
	h.AxisToDed = b.seg(axis+"->"+foot, g.Axis, footSeed, h.Axis, nil)
	h.Foot = h.AxisToDed.End
	h.Foot.SetName(foot)
	// Fusion writes addCollinear against the shaft axis here. The engine
	// counts a collinear as two point-on-line rows and the shared endpoint
	// already satisfies one, so the proof emits the single independent row.
	// That substitution also makes the two readings [PB-COLLINEAR-CHAIN]
	// separates identical, so only a Fusion session can tell them apart.
	b.add(label+" "+foot+" on the shaft axis", sketch.NewPointOnLine(h.Foot, h.ShaftAxis))

	h.DedToFoot = b.seg(gearDedLabel(label)+"->"+foot, g.Ded, footSeed, h.Ded, h.Foot)
	b.add(label+" dedendum perpendicular at "+foot,
		sketch.NewPerpendicular(h.AxisToDed, h.DedToFoot))

	baseSeed := footSeed.add(g.AxisDir.scale(b.d.Module))
	h.FootToEnd = b.seg(foot+"->"+base, footSeed, baseSeed, h.Foot, nil)
	h.Base = h.FootToEnd.End
	h.Base.SetName(base)
	// The collinear names the line the new line's start sits on — A->E, never
	// the Apex->A shaft axis further up the chain ([BEVEL-F-COLLINEAR-CHAIN]).
	b.add(label+" "+base+" on "+axis+"->"+foot, sketch.NewPointOnLine(h.Base, h.AxisToDed))

	heelSeed := g.Ded.add(g.DedDir.scale(b.d.Module))
	h.DedToHeel = b.seg(gearDedLabel(label)+"->"+heel, g.Ded, heelSeed, h.Ded, nil)
	h.Heel = h.DedToHeel.End
	h.Heel.SetName(heel)
	b.add(label+" "+heel+" on the dedendum line", sketch.NewPointOnLine(h.Heel, h.Dedendum))

	h.EndToHeel = b.seg(base+"->"+heel, baseSeed, heelSeed, h.Base, h.Heel)
	b.offset(label+" base height", h.Drop, h.EndToHeel, gr.BaseHeight, g.Base)
}

func gearDedLabel(label string) string {
	if label == "pinion" {
		return "C"
	}
	return "D"
}

// buildToothCentre places the back-cone point K/L and, when Tooth Spacing is
// positive, the tooth centre K'/L' one spacing further out along the dedendum
// line. K and L are the case where BOTH ends are already fixed, so they take
// two point-on-line coincidents and no collinear at all.
func (b *latticeBuilder) buildToothCentre(h *sideHandles, g sideGeometry, name string) {
	line := b.seg(pointName(h, name)+" extension", g.Base, g.Back, h.Base, nil)
	h.Back = line.End
	h.Back.SetName(name)
	b.add(name+" on the shaft axis", sketch.NewPointOnLine(h.Back, h.ShaftAxis))
	b.add(name+" on the dedendum line", sketch.NewPointOnLine(h.Back, h.Dedendum))

	dedName := "C"
	if name == "L" {
		dedName = "D"
	}
	ref := b.seg(dedName+"->"+name, g.Ded, g.Back, h.Ded, h.Back)

	if b.d.ToothSpacing <= 0 {
		// A zero-length dimensioned line would be degenerate, and one segment
		// gets one line ([BEVEL-F-LINE-ONCE]): the tooth centre IS K/L and the
		// reference line C->K / D->L is reused as C->K' / D->L'.
		h.Tooth = h.Back
		h.ToothRef = ref
		return
	}
	spacing := b.seg(name+"->"+name+"'", g.Back, g.Tooth, h.Back, nil)
	h.Tooth = spacing.End
	h.Tooth.SetName(name + "'")
	// The far end is seeded on the far side of K/L from the dedendum corner and
	// the length dimension then fixes how far. An unsigned length plus a
	// point-on-line leaves the OTHER side solvable too — measured, the probe
	// finds that second configuration — so the proof pins the direction with a
	// signed zero angle against the dedendum line, which is the same statement
	// the seed side makes in Fusion.
	b.angle(name+"' along the dedendum line", h.Dedendum, spacing, g.DedDir, g.DedDir)
	b.length(name+"->"+name+"' = Tooth Spacing", spacing, b.d.ToothSpacing)
	h.ToothRef = b.seg(dedName+"->"+name+"'", g.Ded, g.Tooth, h.Ded, h.Tooth)
}

func pointName(h *sideHandles, name string) string {
	if name == "K" {
		return "G->K"
	}
	return "I->L"
}

// buildToe draws the toe line M->N / O->P and its two connector lines. The two
// coincident pins are what make the Maximum Face Width guarantee real: N
// reaches A exactly at the cap, so a capped Face Width keeps N between Apex 2
// and A and the revolved profile never crosses its own axis.
func (b *latticeBuilder) buildToe(h *sideHandles, g sideGeometry, d design, toeName, toeInName string) {
	// Seed M near the midpoint of Apex->Ded and N by sliding from that seed
	// along the dedendum direction far enough to reach the drop line. Seeding
	// them a Face Width from the dedendum corner instead starts N near the
	// heel, far from its constraint target, and the solve does not converge.
	// M is seeded on the root element Apex->C at the fraction 1 - FaceWidth/R,
	// and N by sliding from that seed along the C->H direction until it reaches
	// line A->Apex2. Both are the solved positions ([PB-SEED-NEAR]).
	//
	// Section 2 states the M seed as "roughly the midpoint of Apex->C", which
	// is what that fraction comes to at the DEFAULT Face Width and only there:
	// at a user Face Width well below the default the solved fraction runs past
	// 0.9 and the literal midpoint seed does not converge in this engine. The
	// fraction is the same rule stated so it holds across the range, and it is
	// still emphatically not "a Face Width away from C", which is the seed
	// section 2 forbids because it starts N near H.
	mSeed := g.Toe
	nSeed := mSeed.add(g.DedDir.scale(mSeed.distanceTo(g.ToeIn)))

	h.ToeLine = b.seg(toeName+"->"+toeInName, mSeed, nSeed, nil, nil)
	h.Toe = h.ToeLine.Start
	h.ToeIn = h.ToeLine.End
	h.Toe.SetName(toeName)
	h.ToeIn.SetName(toeInName)

	b.add(toeName+" on the root axis", sketch.NewPointOnLine(h.Toe, h.RootAxis))
	// N is pinned to the A->Apex2 perpendicular DROP, never to the Apex->A
	// shaft axis: pinning it to the axis puts it ON the axis of revolution and
	// the later conical split fails for asymmetric tooth counts.
	b.add(toeInName+" on the drop line", sketch.NewPointOnLine(h.ToeIn, h.Drop))
	b.offset("face width "+toeName+"->"+toeInName, h.DedToHeel, h.ToeLine, d.FaceWidth, g.Toe)

	dedName := "C"
	axisName := "A"
	if toeName == "O" {
		dedName, axisName = "D", "B"
	}
	b.seg(toeName+"->"+dedName, mSeed, g.Ded, h.Toe, h.Ded)
	b.seg(toeInName+"->"+axisName, nSeed, g.Axis, h.ToeIn, h.Axis)
}

// ---------------------------------------------------------------------------
// The step.
// ---------------------------------------------------------------------------

// stepGearProfiles draws the Gear Profiles sketch: the whole section 2 lattice
// for both gears, in one sketch, ending fully constrained.
//
// Beyond the harness gate it asserts what section 2 pins and every later step
// selects on:
//
//   - every solved point against the closed form, to 1e-7 mm;
//   - that point I lands on the projected anchor centre, which is the closure
//     the Apex seed is derived from — get the seed wrong and this fails;
//   - the two cone angles against tan gamma_p = sin S * PPD / (DPD + PPD cos S)
//     and gamma_g = S - gamma_p;
//   - the Maximum Face Width measured from the SOLVED A, B, C, D, H and J
//     against its closed form, which is the reading section 2 requires and the
//     one the seeds are too loose to give ([PB-SOLVED-GEOMETRY]);
//   - that N stays between Apex 2 and A (and P between Apex 2 and B), which is
//     what keeps the revolved hexagon off its own axis;
//   - that the toe is nearer the Apex than the heel, the ordering the spiral
//     frame's swap guard exists to catch;
//   - that the back-cone point K sits at the virtual pitch radius from Apex 2,
//     since section 3 computes the virtual tooth number from that radius
//     rather than by measuring it.
//
// <!-- proof-run: proofkit.Run(latticeCases, stepGearProfiles) -->
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := newDesign(t, p)
	if p[keyNetLimited] != 0 {
		// Measured on THIS net, 2026-09-03: the default 31/31 pair at a 30
		// degree Shaft Angle solves to DOF 0 with nothing redundant and its
		// cone angles agree with the closed form, but its conditioning reads
		// 2.8308e-05 against the engine's 4e-05 trust floor, and it first
		// clears at 35 degrees. That is the reading instructions.md predicts
		// for two of the three lattices written from this spec, and it is a
		// fact about how THIS lattice is built, not about the Shaft Angle: the
		// third lattice passes 30 degrees and refuses 142 and 150, which this
		// one passes. So the advertised range is not narrowed, the gate is not
		// loosened, and the measurement is recorded here where it belongs.
		proofkit.Unmodelled(t, "this section 2 net reads conditioning 2.8308e-05 at a 30 degree "+
			"Shaft Angle, below the engine's 4e-05 trust floor; it first clears at 35 degrees")
	}
	b := buildLattice(t, s, d)

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("gear profiles: solve: %v", err)
	}

	g := sharedLattice(d)
	requireCloseVec(t, solved(b.apex), g.Apex, slackTol, "Apex")
	requireCloseVec(t, solved(b.apex2), g.Pinion.Apex2, slackTol, "Apex 2")
	assertSide(t, b.pin, g.Pinion, "pinion")
	assertSide(t, b.drv, g.Driving, "driving")

	// The Apex 2 closure drives point I onto the projected centre. Nothing in
	// the net says so directly — the proof pins I only to the anchor line — so
	// this reading is the closure itself.
	requireCloseVec(t, solved(b.drv.Base), g.Centre, slackTol, "point I at the projected centre")

	// The solved cone angles, read off the geometry rather than recomputed.
	gammaP := math.Atan2(
		solved(b.apex2).distanceTo(solved(b.pin.Axis)),
		solved(b.apex).distanceTo(solved(b.pin.Axis)))
	gammaG := math.Atan2(
		solved(b.apex2).distanceTo(solved(b.drv.Axis)),
		solved(b.apex).distanceTo(solved(b.drv.Axis)))
	requireClose(t, gammaP, d.Pinion.Gamma, slackTol, "solved pinion pitch cone angle")
	requireClose(t, gammaG, d.Driving.Gamma, slackTol, "solved driving pitch cone angle")
	requireClose(t, gammaP+gammaG, d.Sigma, slackTol, "gamma_p + gamma_g = Shaft Angle")

	// The Maximum Face Width, measured the way section 2 requires: the
	// perpendicular distance from A to line C-H and from B to line D-J, both
	// off SOLVED geometry.
	pinDist := math.Abs(signedDistanceFromLine(
		solved(b.pin.Ded), solved(b.pin.Heel), solved(b.pin.Axis)))
	drvDist := math.Abs(signedDistanceFromLine(
		solved(b.drv.Ded), solved(b.drv.Heel), solved(b.drv.Axis)))
	measured := 0.95 * math.Min(pinDist, drvDist)
	requireClose(t, measured, maximumFaceWidth(d.R, d.Pinion.Gamma, d.Driving.Gamma), slackTol,
		"Maximum Face Width from solved geometry")
	if d.FaceWidth > measured+slackTol {
		t.Fatalf("resolved Face Width %.6f mm exceeds the solved Maximum Face Width %.6f mm",
			d.FaceWidth, measured)
	}

	// N between Apex 2 and A, P between Apex 2 and B: the revolve fails with
	// ASM_WIRE_X_AXIS the moment the toe corner crosses the shaft axis.
	assertToeInside(t, solved(b.apex2), solved(b.pin.Axis), solved(b.pin.ToeIn), "N")
	assertToeInside(t, solved(b.apex2), solved(b.drv.Axis), solved(b.drv.ToeIn), "P")

	// Toe nearer the Apex than heel, per gear.
	for _, side := range []struct {
		name string
		h    *sideHandles
	}{{"pinion", b.pin}, {"driving", b.drv}} {
		apex := solved(b.apex)
		toeMid := solved(side.h.Toe).add(solved(side.h.ToeIn)).scale(0.5)
		heelMid := solved(side.h.Ded).add(solved(side.h.Heel)).scale(0.5)
		if apex.distanceTo(toeMid) >= apex.distanceTo(heelMid) {
			t.Fatalf("%s toe is not inside the heel: toe %.6f mm, heel %.6f mm from the Apex",
				side.name, apex.distanceTo(toeMid), apex.distanceTo(heelMid))
		}
	}

	// The back-cone point sits at the virtual pitch radius from Apex 2, which
	// is what makes section 3's closed-form virtual tooth number the same
	// number as the drawn geometry.
	requireClose(t, solved(b.apex2).distanceTo(solved(b.pin.Back)),
		virtualPitchRadius(d.Pinion.PitchDiameter, d.Pinion.Gamma), slackTol,
		"pinion virtual pitch radius Apex2->K")
	requireClose(t, solved(b.apex2).distanceTo(solved(b.drv.Back)),
		virtualPitchRadius(d.Driving.PitchDiameter, d.Driving.Gamma), slackTol,
		"driving virtual pitch radius Apex2->L")

	// Tooth Spacing moves only the centre, never the tooth size.
	requireClose(t, solved(b.pin.Back).distanceTo(solved(b.pin.Tooth)), d.ToothSpacing, slackTol,
		"pinion Tooth Spacing K->K'")
	requireClose(t, solved(b.drv.Back).distanceTo(solved(b.drv.Tooth)), d.ToothSpacing, slackTol,
		"driving Tooth Spacing L->L'")
}

func assertSide(t testing.TB, h *sideHandles, g sideGeometry, label string) {
	t.Helper()
	for _, c := range []struct {
		name string
		got  *sketch.Point
		want vec2
	}{
		{"shaft pitch point", h.Axis, g.Axis},
		{"shaft heel point", h.Base, g.Base},
		{"dedendum corner", h.Ded, g.Ded},
		{"heel corner", h.Heel, g.Heel},
		{"dedendum foot", h.Foot, g.Foot},
		{"toe root corner", h.Toe, g.Toe},
		{"toe drop corner", h.ToeIn, g.ToeIn},
		{"back-cone point", h.Back, g.Back},
		{"tooth centre", h.Tooth, g.Tooth},
	} {
		requireCloseVec(t, solved(c.got), c.want, slackTol, "%s %s", label, c.name)
	}
}

// assertToeInside checks that the toe corner sits strictly between Apex 2 and
// the shaft pitch point on the drop line, which is the Maximum Face Width
// guarantee expressed on the solved geometry.
func assertToeInside(t testing.TB, apex2, axisPoint, toeIn vec2, name string) {
	t.Helper()
	span := axisPoint.sub(apex2)
	u := toeIn.sub(apex2).dot(span) / span.dot(span)
	if u < 0 || u > 1 {
		t.Fatalf("%s is not between Apex 2 and the shaft axis: parameter %.6f", name, u)
	}
}

var _ = fmt.Sprintf
