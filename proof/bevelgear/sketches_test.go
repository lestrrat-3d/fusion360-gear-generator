package bevelgear_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// bevelSeg creates one §2 line the way [BEVEL-F-COINCIDENT-STYLE] requires: from raw
// coordinates for BOTH endpoints, never by sharing an existing point into the
// creation call. Every connection is then made with exactly one coincident per
// end. The rule covers the short reference and connector lines too — a regen
// that shared only those came out about fourteen coincidents short and the
// sketch gated as under-constrained — so no line in this file is built any
// other way.
//
// Each line is created ONCE and reused ([BEVEL-F-LINE-ONCE]); a second line
// over the same segment carries its own constraints over that segment and
// over-determines the coupled net.
func bevelSeg(s *sketch.Sketch, name string, a, b bevelVec) *sketch.Line {
	p0 := s.CreatePoint(a.X, a.Y)
	p1 := s.CreatePoint(b.X, b.Y)
	p0.SetName(name + ".start")
	p1.SetName(name + ".end")
	l := s.CreateLine(p0, p1)
	l.SetName(name)
	// Every line drawn in the §2 sketch is a construction line: the solid
	// features consume only the per-bevelSide Profile sketches, never a §2 curve.
	l.SetConstruction(true)
	return l
}

func bevelPin(s *sketch.Sketch, name string, a, b *sketch.Point) {
	c := sketch.NewCoincident(a, b)
	s.AddConstraint(c)
	s.SetConstraintName(c, name)
}

func bevelNamed(s *sketch.Sketch, name string, c sketch.Constraint) {
	s.AddConstraint(c)
	s.SetConstraintName(c, name)
}

// stepAnchorSketch builds §1: the Anchor Sketch on the user's selected target
// plane, carrying the projected centre point and the Anchor Line through it.
//
// The bevelBench substitutes a reference point for the projection, which is what
// sketch's reference geometry is for: a projected point is brought in from
// outside and is externally locked, and `CreateReferencePoint` is the engine's
// name for exactly that. Fusion's `sketch.project` does NOT fix what it brings
// in ([PB-PROJECT-NOT-FIXED]), so the bevelBench point is firmer than Fusion's; what
// the bevelModuleOf does about that is recreate-and-fix in the per-bevelSide Profile
// sketches, which stepGearProfileHexagon proves.
func stepAnchorSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user-selected centre point into the Anchor Sketch")
	cx, cy := p["centreX"], p["centreY"]
	centre := s.CreateReferencePoint(cx, cy, "user centre point")
	centre.SetName("projected centre")

	// Seeded at exactly +/- 0.5 cm from the projected centre along the sketch's
	// own X, so the seeded length is the 10 mm the dimension then locks.
	proofkit.Step(t, "draw the Anchor Line through the projected centre")
	line := bevelSeg(s, "Anchor Line", bevelVec{cx - 5, cy}, bevelVec{cx + 5, cy})
	line.SetConstruction(false)

	// Fusion applies BOTH addCoincident(projectedCentre, anchorLine) — the
	// point-on-line "intersection" — AND addMidPoint(projectedCentre,
	// anchorLine), and the spec says to use both rather than the midpoint
	// alone. The bevelBench's NewMidpoint carries the point-on-line row itself, so
	// adding the engine's NewPointOnLine beside it is a second statement of a
	// row the midpoint already holds and the sketch comes back with a redundant
	// constraint. This is the same shape of difference as the G->H
	// perpendicular below: Fusion's two calls are two rows, the bench's one call
	// is two rows, and the proof writes the bevelBench's arity rather than Fusion's.
	bevelNamed(s, "centre bisects Anchor Line", sketch.NewMidpoint(centre, line))
	// Fusion locks the seeded 10 mm with an aligned distance dimension, which is
	// a magnitude: the line may still solve end-for-end, and the bevelBench's
	// ambiguity probe reports exactly that pair. The engine's horizontal
	// distance is SIGNED, so it states the same 10 mm AND which end is which.
	// Nothing downstream depends on the line's absolute direction — §2 derives
	// every direction relative to it — so signing it here changes no geometry;
	// it only stops a figure the gate is right to refuse.
	bevelNamed(s, "Anchor Line length", sketch.NewHorizontalDistance(line.Start, line.End, 10))
	// Sketch-local, per [PB-REFLINE-DIRECTION]: the line's absolute direction is
	// arbitrary because §2 derives every direction relative to it, but it must
	// not be a free degree of freedom.
	bevelNamed(s, "Anchor Line direction", sketch.NewHorizontal(line))
}

// bevelLatticePoint is one bevelNamed §2 point: the solved handle and the closed-form
// position §2 seeded it at.
type bevelLatticePoint struct {
	name   string
	point  *sketch.Point
	seeded bevelVec
}

// stepGearProfiles builds §2, the Gear Profiles sketch: the whole two-bevelSide
// lattice from the projected centre out to both front faces.
//
// Read the constraint arities here against Fusion's, because two of them
// differ and the difference is the engine's, not a choice:
//
//   - The G->H and I->J base-height offsets. Fusion's addOffsetDimension is a
//     distance dimension whose documentation requires the second entity to be a
//     line parallel to the first, so §2 supplies that parallelism with a
//     perpendicular (E->G perpendicular to H->G, F->I to J->I) and the offset
//     then controls one freedom. The bevelBench's NewOffset emits TWO residual rows,
//     holding both endpoints of the destination line at the same SIGNED
//     perpendicular distance from the source, so it carries the parallelism
//     itself. Adding the perpendicular here is a third row for the same two
//     freedoms and the lattice comes back over-constrained at DOF 0 with the
//     two base-height offsets bevelNamed as the redundant pair. The perpendicular is
//     therefore omitted here and required in Fusion.
//
//   - The M->N and O->P toe lines. §2 requires addParallel(M->N, C->H) beside
//     the offset for the same reason, and the same two-row NewOffset makes it
//     redundant here. The spec spells the substitution out for the base-height
//     offsets and not for these, but it follows from the identical arity, so the
//     parallel is omitted here too.
//
// Both omissions are the proof declining to model Fusion's arity where the
// engine's differs. Neither weakens the gate: the count of freedoms each group
// pins is unchanged.
//
// Every perpendicular and the one parallel §2 states are written here as the
// SIGNED angle they are. That is this proof's obligation rather than a liberty:
// [BEVEL-F-MIRROR-FIGURE] counts fifteen §2 sites whose side no Fusion
// constraint can bevelPin — every geometric constraint Fusion offers is unsigned or
// undirected — and §2 therefore picks each side by seed and catches a flip with
// the [BEVEL-F-SEED-HELD] gate. The bevelBench DOES offer a signed angle, and a
// proof that used the unsigned perpendicular instead would leave those sites
// unsigned here too, which is the claim the gate rests on. Each angle below is
// the closed-form angle between the two seeded directions, so the value
// written is a closed form, not a reading; at a right angle it is the
// perpendicular Fusion adds, with the side stated.
func stepGearProfiles(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := resolveBevel(p)

	proofkit.Step(t, "project the Anchor Sketch centre point and the Anchor Line")
	centre := s.CreateReferencePoint(0, 0, "Anchor Sketch centre point")
	centre.SetName("projected centre")
	anchorA := s.CreateReferencePoint(-5, 0, "Anchor Line start")
	anchorB := s.CreateReferencePoint(5, 0, "Anchor Line end")
	anchor, err := s.CreateReferenceLine(anchorA, anchorB, "Anchor Line")
	if err != nil {
		t.Fatalf("reference anchor line: %v", err)
	}
	anchor.SetName("projected Anchor Line")

	proofkit.Step(t, "centre -> Apex, perpendicular to the projected Anchor Line")
	centreToApex := bevelSeg(s, "centre->Apex", bevelVec{0, 0}, g.Apex)
	bevelPin(s, "centre->Apex starts at the projected centre", centreToApex.Start, centre)
	// The grow side. In Fusion perp's sign is a one-bit decision taken against
	// the target-plane normal ([BEVEL-F-GROW-SIDE]); the bevelBench has no target
	// plane, so this proof fixes the side and proves everything downstream of
	// it, and the rule that chooses it is checked only in Fusion.
	bevelNamed(s, "centre->Apex square to the Anchor Line, on the grow side",
		sketch.NewAngle(anchor, centreToApex,
			bevelSignedAngleDeg(bevelVec{1, 0}, bevelSub(g.Apex, bevelVec{0, 0}))))
	apex := centreToApex.End
	apex.SetName("Apex")

	proofkit.Step(t, "Apex -> B, the Driving Gear Shaft Axis")
	axB := bevelSeg(s, "Apex->B", g.Apex, g.B)
	bevelPin(s, "Apex->B starts at the Apex", axB.Start, apex)
	// addParallel to the centre->Apex line, never addVertical: the bevelSide profiles
	// sketch is not world-aligned and a world-vertical lock mis-orients the
	// figure on any tilted target plane.
	bevelNamed(s, "Apex->B parallel to centre->Apex, pointing back at the anchor line",
		sketch.NewAngle(centreToApex, axB,
			bevelSignedAngleDeg(bevelSub(g.Apex, bevelVec{0, 0}), g.UB)))
	pointB := axB.End
	pointB.SetName("B")

	proofkit.Step(t, "Apex -> A, the Pinion Gear Shaft Axis at the Shaft Angle")
	axA := bevelSeg(s, "Apex->A", g.Apex, g.A)
	bevelPin(s, "Apex->A starts at the Apex", axA.Start, apex)
	// The bevelBench's NewAngle is SIGNED, measured counter-clockwise from the first
	// line's start->end direction to the second's, so it pins the sense §2
	// chooses by seed in Fusion, where addAngularDimension is unsigned and the
	// text point only picks the wedge that is MEASURED. The value is positive
	// because the pinion direction is the driving direction rotated by +Sigma,
	// which is the candidate whose endpoint carries the greater X.
	bevelNamed(s, "Shaft Angle", sketch.NewAngle(axB, axA, p["shaftAngle"]))
	pointA := axA.End
	pointA.SetName("A")

	proofkit.Step(t, "the two perpendicular drops that close at Apex 2")
	dropA := bevelSeg(s, "A->Apex2", g.A, g.Apex2)
	bevelPin(s, "A->Apex2 starts at A", dropA.Start, pointA)
	// The drop aims into the interior wedge BETWEEN the two shaft axes, toward
	// the driving shaft and point B — never "toward the anchor line".
	bevelNamed(s, "A->Apex2 square to Apex->A, into the wedge",
		sketch.NewAngle(axA, dropA, bevelSignedAngleDeg(g.UA, bevelSub(g.Apex2, g.A))))
	bevelNamed(s, "A->Apex2 length is the pinion pitch radius",
		sketch.NewDistance(dropA.Start, dropA.End, g.PPD/2))
	apex2 := dropA.End
	apex2.SetName("Apex 2")

	dropB := bevelSeg(s, "B->Apex2", g.B, g.Apex2)
	bevelPin(s, "B->Apex2 starts at B", dropB.Start, pointB)
	// Aimed at the pinion shaft axis and point A. A "toward the anchor line"
	// test is degenerate here: the Driving Gear Shaft Axis is itself parallel to
	// that grow direction, so the perpendicular's dot with it reads about zero
	// and silently selects an arbitrary side. If this drop seeds Apex 2 on the
	// wrong side of the driving shaft while the pinion's drop seeds it on the
	// correct side, the coincidence that closes them flips the WHOLE figure to
	// the mirror solution and nothing in the build refuses it.
	bevelNamed(s, "B->Apex2 square to Apex->B, into the wedge",
		sketch.NewAngle(axB, dropB, bevelSignedAngleDeg(g.UB, bevelSub(g.Apex2, g.B))))
	bevelNamed(s, "B->Apex2 length is the driving pitch radius",
		sketch.NewDistance(dropB.Start, dropB.End, g.DPD/2))
	bevelPin(s, "the two drops close at Apex 2", dropB.End, apex2)

	proofkit.Step(t, "the Pitch Line and the two dedendum lines")
	pitch := bevelSeg(s, "Apex->Apex2 (Pitch Line)", g.Apex, g.Apex2)
	bevelPin(s, "Pitch Line starts at the Apex", pitch.Start, apex)
	bevelPin(s, "Pitch Line ends at Apex 2", pitch.End, apex2)

	dedP := bevelSeg(s, "Apex2->C (Pinion Dedendum)", g.Apex2, g.C)
	bevelPin(s, "Pinion Dedendum starts at Apex 2", dedP.Start, apex2)
	// This site and its driving twin are where the "C collapses onto D" symptom
	// lives, and in Fusion each is held by its seed alone: the perpendicular
	// constrains the direction and the dimension the magnitude, and neither
	// picks a side, so flipping the pinion seed solves C exactly onto D. The
	// seed is stated by dot product against the shaft axes — u_p . unit(Apex->A)
	// is sin(gamma_p) and is strictly positive for every admitted
	// configuration — never as "towards or away from the anchor line".
	bevelNamed(s, "Pinion Dedendum square to the Pitch Line, on the Apex->A side",
		sketch.NewAngle(pitch, dedP, bevelSignedAngleDeg(bevelSub(g.Apex2, g.Apex), g.UP)))
	bevelNamed(s, "Pinion Dedendum length", sketch.NewDistance(dedP.Start, dedP.End, g.Dedendum))
	pointC := dedP.End
	pointC.SetName("C")

	dedG := bevelSeg(s, "Apex2->D (Driving Dedendum)", g.Apex2, g.D)
	bevelPin(s, "Driving Dedendum starts at Apex 2", dedG.Start, apex2)
	bevelNamed(s, "Driving Dedendum square to the Pitch Line, on the Apex->B side",
		sketch.NewAngle(pitch, dedG, bevelSignedAngleDeg(bevelSub(g.Apex2, g.Apex), g.UG)))
	bevelNamed(s, "Driving Dedendum length", sketch.NewDistance(dedG.Start, dedG.End, g.Dedendum))
	pointD := dedG.End
	pointD.SetName("D")

	proofkit.Step(t, "the two Root Axes, Apex->C and Apex->D")
	rootP := bevelSeg(s, "Apex->C (Pinion Root Axis)", g.Apex, g.C)
	bevelPin(s, "Pinion Root Axis starts at the Apex", rootP.Start, apex)
	bevelPin(s, "Pinion Root Axis ends at C", rootP.End, pointC)

	rootG := bevelSeg(s, "Apex->D (Driving Root Axis)", g.Apex, g.D)
	bevelPin(s, "Driving Root Axis starts at the Apex", rootG.Start, apex)
	bevelPin(s, "Driving Root Axis ends at D", rootG.End, pointD)

	proofkit.Step(t, "E and F, the feet of the dedendum perpendiculars on the shaft axes")
	// The collinear names the line the new line's start point sits on, never a
	// farther line up the same chain ([PB-COLLINEAR-CHAIN]). The bevelBench counts a
	// collinear as the same two point-on-line rows Fusion does, so a proof
	// written against it substitutes the single row that is not already implied
	// by the coincident at the shared end — which makes both readings of the
	// chain identical here. Only a Fusion session tells them apart, so this
	// substitution is also the reason the proof cannot catch a collinear bevelNamed
	// against the wrong line.
	lineAE := bevelSeg(s, "A->E", g.A, g.E)
	bevelPin(s, "A->E starts at A", lineAE.Start, pointA)
	bevelNamed(s, "E lies on Apex->A", sketch.NewPointOnLine(lineAE.End, axA))
	pointE := lineAE.End
	pointE.SetName("E")

	lineCE := bevelSeg(s, "C->E", g.C, g.E)
	bevelPin(s, "C->E starts at C", lineCE.Start, pointC)
	bevelPin(s, "C->E ends at E", lineCE.End, pointE)
	bevelNamed(s, "C->E square to A->E", sketch.NewAngle(lineAE, lineCE, bevelSignedAngleDeg(g.UA, bevelSub(g.E, g.C))))

	lineBF := bevelSeg(s, "B->F", g.B, g.F)
	bevelPin(s, "B->F starts at B", lineBF.Start, pointB)
	bevelNamed(s, "F lies on Apex->B", sketch.NewPointOnLine(lineBF.End, axB))
	pointF := lineBF.End
	pointF.SetName("F")

	lineDF := bevelSeg(s, "D->F", g.D, g.F)
	bevelPin(s, "D->F starts at D", lineDF.Start, pointD)
	bevelPin(s, "D->F ends at F", lineDF.End, pointF)
	bevelNamed(s, "D->F square to B->F", sketch.NewAngle(lineBF, lineDF, bevelSignedAngleDeg(g.UB, bevelSub(g.F, g.D))))

	proofkit.Step(t, "the pinion heel: E->G, C->H, G->H and the base-height offset")
	lineEG := bevelSeg(s, "E->G", g.E, g.G)
	bevelPin(s, "E->G starts at E", lineEG.Start, pointE)
	// Collinear with A->E, NEVER with the Apex->A shaft axis further up the
	// chain, even though both describe the same infinite line.
	bevelNamed(s, "G lies on A->E", sketch.NewPointOnLine(lineEG.End, lineAE))
	pointG := lineEG.End
	pointG.SetName("G")

	lineCH := bevelSeg(s, "C->H", g.C, g.H)
	bevelPin(s, "C->H starts at C", lineCH.Start, pointC)
	bevelNamed(s, "H lies on Apex2->C", sketch.NewPointOnLine(lineCH.End, dedP))
	pointH := lineCH.End
	pointH.SetName("H")

	lineGH := bevelSeg(s, "G->H", g.G, g.H)
	bevelPin(s, "G->H starts at G", lineGH.Start, pointG)
	bevelPin(s, "G->H ends at H", lineGH.End, pointH)
	// The offset is between the A->Apex2 PERPENDICULAR DROP — the PPD/2 drop,
	// not the Apex->A shaft axis — and G->H. Positive is to the left of the
	// drop's start->end direction, which is the +unit(Apex->A) side, and G sits
	// exactly one resolved pinion base height along that direction from A.
	bevelNamed(s, "pinion base height offset",
		sketch.NewOffset(dropA, lineGH, g.Pinion.BaseHeight))

	proofkit.Step(t, "the driving heel: F->I, D->J, I->J and the base-height offset")
	lineFI := bevelSeg(s, "F->I", g.F, g.I)
	bevelPin(s, "F->I starts at F", lineFI.Start, pointF)
	bevelNamed(s, "I lies on B->F", sketch.NewPointOnLine(lineFI.End, lineBF))
	pointI := lineFI.End
	pointI.SetName("I")

	lineDJ := bevelSeg(s, "D->J", g.D, g.J)
	bevelPin(s, "D->J starts at D", lineDJ.Start, pointD)
	bevelNamed(s, "J lies on Apex2->D", sketch.NewPointOnLine(lineDJ.End, dedG))
	pointJ := lineDJ.End
	pointJ.SetName("J")

	lineIJ := bevelSeg(s, "I->J", g.I, g.J)
	bevelPin(s, "I->J starts at I", lineIJ.Start, pointI)
	bevelPin(s, "I->J ends at J", lineIJ.End, pointJ)
	// Left of the B->Apex2 drop's start->end direction is -unit(Apex->B), and I
	// sits one resolved driving base height along +unit(Apex->B) from B, so the
	// signed value is negative. Fusion's addOffsetDimension is unsigned and the
	// I and J seeds are the only thing that picks the side there.
	bevelNamed(s, "driving base height offset",
		sketch.NewOffset(dropB, lineIJ, -g.Driving.BaseHeight))

	proofkit.Step(t, "close the figure: I coincides with the projected centre")
	// This is what hangs the whole lattice off the projected centre. In Fusion
	// it is one addCoincident; the bevelBench counts its two rows and one of them is
	// already implied, because I sits on the driving shaft axis and that axis
	// runs through the Apex, which the perpendicular already holds at the
	// centre's own X. The bevelBench therefore takes the row that is not implied —
	// the position ALONG the axis — as a signed vertical distance of zero, the
	// grow direction being +Y in this frame.
	bevelNamed(s, "I closes on the projected centre",
		sketch.NewVerticalDistance(centre, pointI, 0))

	proofkit.Step(t, "A', the front face's foot on the pinion shaft axis")
	// This line is what CREATES A'. It is drawn here, before the front face
	// below pins A' to the axis, so the hexagon's edges are created in the walk
	// order A' -> G -> H -> C -> M -> N the Profile sketch's first-edge rule
	// depends on.
	lineApG := bevelSeg(s, "A'->G", g.Ap, g.G)
	bevelPin(s, "A'->G ends at G", lineApG.End, pointG)
	pointAp := lineApG.Start
	pointAp.SetName("A'")

	proofkit.Step(t, "K and L, where the dedendum lines cross the shaft axes")
	// By the time K is added, G and C are already fixed, so a collinear on the
	// connecting lines over-constrains; two point-on-line coincidents locate K
	// exactly as the intersection of the two lines without over-constraining.
	lineGK := bevelSeg(s, "G->K", g.G, g.K)
	bevelPin(s, "G->K starts at G", lineGK.Start, pointG)
	bevelNamed(s, "K lies on Apex->A", sketch.NewPointOnLine(lineGK.End, axA))
	bevelNamed(s, "K lies on Apex2->C", sketch.NewPointOnLine(lineGK.End, dedP))
	pointK := lineGK.End
	pointK.SetName("K")

	lineIL := bevelSeg(s, "I->L", g.I, g.L)
	bevelPin(s, "I->L starts at I", lineIL.Start, pointI)
	bevelNamed(s, "L lies on Apex->B", sketch.NewPointOnLine(lineIL.End, axB))
	bevelNamed(s, "L lies on Apex2->D", sketch.NewPointOnLine(lineIL.End, dedG))
	pointL := lineIL.End
	pointL.SetName("L")

	// The tooth centre. At Tooth Spacing 0 — the default — K' is K and L' is L,
	// nothing is built, and the existing C->K and D->L reference lines are
	// reused: a zero-length dimensioned line would be degenerate, and one
	// segment gets ONE line ([BEVEL-F-LINE-ONCE]).
	pointKp, pointLp := pointK, pointL
	if g.ToothSpacing > 0 {
		proofkit.Step(t, "K' and L', the Tooth Spacing tooth centres")
		pointKp = bevelToothCentre(t, s, "K", g.Kp, pointK, dedP, g.UP, g.ToothSpacing)
		pointKp.SetName("K'")
		pointLp = bevelToothCentre(t, s, "L", g.Lp, pointL, dedG, g.UG, g.ToothSpacing)
		pointLp.SetName("L'")
	}

	lineCK := bevelSeg(s, "C->K' (tooth centre reference)", g.C, bevelSeedOf(g.ToothSpacing, g.K, g.Kp))
	bevelPin(s, "C->K' starts at C", lineCK.Start, pointC)
	bevelPin(s, "C->K' ends at the tooth centre", lineCK.End, pointKp)

	lineDL := bevelSeg(s, "D->L' (tooth centre reference)", g.D, bevelSeedOf(g.ToothSpacing, g.L, g.Lp))
	bevelPin(s, "D->L' starts at D", lineDL.Start, pointD)
	bevelPin(s, "D->L' ends at the tooth centre", lineDL.End, pointLp)

	proofkit.Step(t, "the pinion toe: M->N, M->C and the front face N->A'")
	// Both ends are seeded at their closed-form SOLVED positions, not merely
	// somewhere plausible. Two earlier seeding rules — sliding from the M seed
	// by the Root Length, and sliding by the distance from the M seed to A —
	// put the N seed PAST the shaft axis on the shipped default pair at a 50%
	// Toe Extension, at -0.27 mm against a solved +5.17 mm, and Fusion then
	// refuses the revolve with ASM_WIRE_X_AXIS, pointing at the revolve rather
	// than at the seed.
	//
	// THE PROOF CANNOT CATCH A WRONG SEED HERE. It seeds M and N at the closed
	// form, which is the rule, so what it proves is that the constraints solve
	// FROM a correct seed and never that the bevelModuleOf's seed is correct. A seed
	// defect reaches Fusion untested, which is how the one described above got
	// there. That is the honest edge of what this stage checks.
	lineMN := bevelSeg(s, "M->N", g.M, g.N)
	bevelNamed(s, "M lies on the Pinion Root Axis", sketch.NewPointOnLine(lineMN.Start, rootP))
	// Positive is to the left of C->H's start->end direction, which is
	// +unit(Apex2->C); M sits on that side of C->H by the root length
	// re-measured perpendicular to the pitch line.
	bevelNamed(s, "pinion root length offset",
		sketch.NewOffset(lineCH, lineMN, bevelOffsetSign(g.M, g.C, g.UP)*g.RootLength*g.R/g.ApexDed))
	pointM := lineMN.Start
	pointM.SetName("M")
	pointN := lineMN.End
	pointN.SetName("N")

	lineMC := bevelSeg(s, "M->C", g.M, g.C)
	bevelPin(s, "M->C starts at M", lineMC.Start, pointM)
	bevelPin(s, "M->C ends at C", lineMC.End, pointC)

	// N is NOT pinned to the A->Apex2 drop, and never to the Apex->A shaft axis:
	// a point ON the axis of revolution makes the later conical split fail with
	// ASM_API_FAILED for asymmetric tooth counts. A' sits on the axis and is a
	// foot rather than a corner; N rides the Toe Radius, which is strictly
	// positive.
	lineNAp := bevelSeg(s, "N->A'", g.N, g.Ap)
	bevelPin(s, "N->A' starts at N", lineNAp.Start, pointN)
	bevelPin(s, "N->A' ends at A'", lineNAp.End, pointAp)
	bevelNamed(s, "A' lies on Apex->A", sketch.NewPointOnLine(pointAp, axA))
	// The front face's own length dimension is a magnitude and the toe line
	// meets the Toe Radius on BOTH sides of the shaft axis, so an unsigned
	// perpendicular here leaves N free to solve onto the mirror, where the
	// revolved hexagon crosses its own axis of revolution and Fusion aborts with
	// ASM_WIRE_X_AXIS at the revolve rather than at the site that caused it. The
	// signed angle states which side N sits on.
	bevelNamed(s, "the pinion front face stands square to the shaft, on N's side",
		sketch.NewAngle(axA, lineNAp, bevelSignedAngleDeg(g.UA, bevelSub(g.Ap, g.N))))
	bevelNamed(s, "Pinion Gear Toe Radius",
		sketch.NewDistance(lineNAp.Start, lineNAp.End, g.Pinion.ToeRadius))

	proofkit.Step(t, "the driving toe: O->P, O->D and the front face P->B'")
	lineOP := bevelSeg(s, "O->P", g.O, g.P)
	bevelNamed(s, "O lies on the Driving Root Axis", sketch.NewPointOnLine(lineOP.Start, rootG))
	bevelNamed(s, "driving root length offset",
		sketch.NewOffset(lineDJ, lineOP, bevelOffsetSign(g.O, g.D, g.UG)*g.RootLength*g.R/g.ApexDed))
	pointO := lineOP.Start
	pointO.SetName("O")
	pointP := lineOP.End
	pointP.SetName("P")

	lineOD := bevelSeg(s, "O->D", g.O, g.D)
	bevelPin(s, "O->D starts at O", lineOD.Start, pointO)
	bevelPin(s, "O->D ends at D", lineOD.End, pointD)

	linePBp := bevelSeg(s, "P->B'", g.P, g.Bp)
	bevelPin(s, "P->B' starts at P", linePBp.Start, pointP)
	pointBp := linePBp.End
	pointBp.SetName("B'")
	bevelNamed(s, "B' lies on Apex->B", sketch.NewPointOnLine(pointBp, axB))
	bevelNamed(s, "the driving front face stands square to the shaft, on P's side",
		sketch.NewAngle(axB, linePBp, bevelSignedAngleDeg(g.UB, bevelSub(g.Bp, g.P))))
	bevelNamed(s, "Driving Gear Toe Radius",
		sketch.NewDistance(linePBp.Start, linePBp.End, g.Driving.ToeRadius))

	lineBpI := bevelSeg(s, "B'->I", g.Bp, g.I)
	bevelPin(s, "B'->I starts at B'", lineBpI.Start, pointBp)
	bevelPin(s, "B'->I ends at I", lineBpI.End, pointI)

	// A configuration this spec admits that this particular net cannot reach is
	// a property of the net, not of the spec, so the case stays in the table and
	// is declared here rather than being dropped or having the Shaft Angle range
	// narrowed around it.
	if p["declaredRefusal"] == 1 {
		proofkit.Unmodelled(t, "declared refusal: at Shaft Angle %.0f degrees this lattice's "+
			"conditioning reads 2.83e-05 against the sketch engine's 4e-05 trust floor, so the "+
			"engine refuses it as near-singular. That is a fact about this constraint net, not "+
			"about the geometry: of three independently written lattices built from this spec, "+
			"two refuse the default pair here and the third passes. The Shaft Angle floor the "+
			"spec states is geometric and stays where it is, the gate is not loosened, and the "+
			"remedy is to change how the lattice is built",
			p["shaftAngle"])
	}

	proofkit.Step(t, "gate the solved figure against its own seeds")
	assertLatticeHeldItsSeeds(t, s, g, []bevelLatticePoint{
		{"Apex", apex, g.Apex}, {"B", pointB, g.B}, {"A", pointA, g.A},
		{"Apex 2", apex2, g.Apex2}, {"C", pointC, g.C}, {"D", pointD, g.D},
		{"E", pointE, g.E}, {"F", pointF, g.F}, {"G", pointG, g.G}, {"H", pointH, g.H},
		{"I", pointI, g.I}, {"J", pointJ, g.J}, {"K", pointK, g.K},
		{"K'", pointKp, g.Kp}, {"M", pointM, g.M}, {"N", pointN, g.N},
		{"A'", pointAp, g.Ap}, {"L", pointL, g.L}, {"L'", pointLp, g.Lp},
		{"O", pointO, g.O}, {"P", pointP, g.P}, {"B'", pointBp, g.Bp},
	})
}

func bevelSeedOf(toothSpacing float64, plain, shifted bevelVec) bevelVec {
	if toothSpacing > 0 {
		return shifted
	}
	return plain
}

// bevelOffsetSign reads which side of the dedendum line the toe line's closed-form
// seed sits on, in the sense the bevelBench's signed offset calls positive. It is a
// reading of the closed form rather than of any built geometry, so it states
// the sign rather than discovering it: the pinion's toe line lies to the left
// of C->H and the driving bevelSide's to the right of D->J, for every configuration
// this spec admits.
func bevelOffsetSign(toeSeed, corner, dedendumDir bevelVec) float64 {
	if bevelDot(bevelSub(toeSeed, corner), bevelLeft(dedendumDir)) >= 0 {
		return 1
	}
	return -1
}

// bevelToothCentre builds one Tooth Spacing offset line, K->K' or L->L'.
//
// The site is one of the fifteen [BEVEL-F-MIRROR-FIGURE] lists, and it is the
// one the spec singles out: pinned with the bevelBench's unsigned NewDistance the
// twin figure — K' one Tooth Spacing on the C side of K, two spacings away —
// satisfies every constraint, and the ambiguity probe reports a LOWER BOUND on
// the solution count, so its silence is not evidence. The signed constraint is.
// The bevelBench signs a horizontal or a vertical distance, so this takes whichever
// axis the dedendum direction leans further along, which is never below 1/root2
// of a unit and so never ill-conditioned.
func bevelToothCentre(t testing.TB, s *sketch.Sketch, label string, seed bevelVec,
	from *sketch.Point, dedendum *sketch.Line, dir bevelVec, spacing float64) *sketch.Point {
	t.Helper()
	line := bevelSeg(s, label+"->"+label+"'", bevelVec{from.X(), from.Y()}, seed)
	bevelPin(s, label+"->"+label+"' starts at "+label, line.Start, from)
	bevelNamed(s, label+"' lies on the dedendum line", sketch.NewPointOnLine(line.End, dedendum))
	if math.Abs(dir.X) >= math.Abs(dir.Y) {
		bevelNamed(s, "Tooth Spacing at "+label,
			sketch.NewHorizontalDistance(line.Start, line.End, spacing*dir.X))
	} else {
		bevelNamed(s, "Tooth Spacing at "+label,
			sketch.NewVerticalDistance(line.Start, line.End, spacing*dir.Y))
	}
	return line.End
}

// assertLatticeHeldItsSeeds is the proof's copy of the [BEVEL-F-SEED-HELD] gate
// the generated bevelModuleOf carries: solve, then compare every bevelNamed §2 point's
// solved position against the closed form §2 seeded it at, in §2's own creation
// order, so the first point bevelNamed is the earliest site that moved rather than a
// downstream symptom. The tolerance is the gate's own 0.001 mm.
//
// The list is 22 points only when Tooth Spacing is above zero. At Tooth Spacing
// 0 — the default — K' is K and L' is L and neither is built, so those two are
// dropped and 20 are compared; comparing a K' that was never created is the one
// way this gate can raise on a correct figure.
//
// The step also resolves the two bounds that cannot be resolved before this
// sketch exists, and asserts them on the SOLVED figure: the Maximum Face Width,
// whose two candidate distances are read from A, B, C, D, H and J, and the
// Maximum Bore Diameter, whose toe term needs the Root Length that follows from
// the resolved Face Width.
func assertLatticeHeldItsSeeds(t testing.TB, s *sketch.Sketch, g bevelGeom, pts []bevelLatticePoint) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("§2 lattice did not solve: %v", err)
	}
	for _, lp := range pts {
		if lp.name == "K'" && g.ToothSpacing == 0 {
			continue
		}
		if lp.name == "L'" && g.ToothSpacing == 0 {
			continue
		}
		moved := math.Hypot(lp.point.X()-lp.seeded.X, lp.point.Y()-lp.seeded.Y)
		if moved > bevelSeedTolerance {
			t.Fatalf("§2 point %s moved %.6f mm off its seed: solved (%.6f, %.6f), seeded (%.6f, %.6f)",
				lp.name, moved, lp.point.X(), lp.point.Y(), lp.seeded.X, lp.seeded.Y)
		}
	}

	// The Tooth Spacing sign, asserted directly because the bevelBench's signed
	// distance pins one axis of it and this is the statement that whole site
	// stands for: K' - K projected on Apex2->C is +Tooth Spacing, never
	// -Tooth Spacing, and the two candidates sit two spacings apart.
	if g.ToothSpacing > 0 {
		along := bevelDot(bevelSub(g.Kp, g.K), g.UP)
		bevelRequireClose(t, "K' - K projected on Apex2->C", along, g.ToothSpacing, bevelSeedTolerance)
		along = bevelDot(bevelSub(g.Lp, g.L), g.UG)
		bevelRequireClose(t, "L' - L projected on Apex2->D", along, g.ToothSpacing, bevelSeedTolerance)
	}

	// The Maximum Face Width, from the solved positions rather than from seeds.
	// Either bevelSide can be the binding side, so both distances are taken: written
	// with the pinion's diameter by name the bound is wrong whenever the driving
	// bevelSide carries the smaller tooth count.
	distA := bevelDistPointLine(bevelVec{pts[2].point.X(), pts[2].point.Y()},
		bevelVec{pts[4].point.X(), pts[4].point.Y()},
		bevelSub(bevelVec{pts[9].point.X(), pts[9].point.Y()}, bevelVec{pts[4].point.X(), pts[4].point.Y()}))
	distB := bevelDistPointLine(bevelVec{pts[1].point.X(), pts[1].point.Y()},
		bevelVec{pts[5].point.X(), pts[5].point.Y()},
		bevelSub(bevelVec{pts[11].point.X(), pts[11].point.Y()}, bevelVec{pts[5].point.X(), pts[5].point.Y()}))
	bevelRequireClose(t, "Maximum Face Width from the solved figure",
		0.95*math.Min(distA, distB), g.MaxFaceWidth, 1e-6)
	if g.FaceWidth > g.MaxFaceWidth+1e-9 {
		t.Errorf("resolved Face Width %.6f exceeds the Maximum Face Width %.6f",
			g.FaceWidth, g.MaxFaceWidth)
	}
	// At Shaft Angle 90 the bound has a closed form in the SMALLER pitch
	// diameter, and never the pinion's by name.
	if math.Abs(g.Sigma-math.Pi/2) < 1e-12 {
		smaller := math.Min(g.PPD, g.DPD)
		bevelRequireClose(t, "Maximum Face Width at Shaft Angle 90",
			g.MaxFaceWidth, 0.95*smaller*smaller/(2*g.ConeDistance), 1e-6)
	}

	// The Maximum Bore Diameter's heel term is where H (resp. J) lands, which
	// the solved figure carries directly.
	for _, pair := range []struct {
		side  bevelSide
		heelP *sketch.Point
		axis  bevelVec
	}{
		{g.Pinion, pts[9].point, g.UA},
		{g.Driving, pts[11].point, g.UB},
	} {
		heel := bevelDistPointLine(bevelVec{pair.heelP.X(), pair.heelP.Y()}, g.Apex, pair.axis)
		bevelRequireClose(t, pair.side.Label+" heel radius r_heel",
			heel, pair.side.PitchRadius-pair.side.BaseHeight/math.Tan(pair.side.Gamma), 1e-6)
		if g.BoreEnable && pair.side.BoreDiameter > pair.side.MaxBore+1e-9 {
			t.Errorf("%s resolved bore diameter %.6f exceeds its Maximum Bore Diameter %.6f",
				pair.side.Label, pair.side.BoreDiameter, pair.side.MaxBore)
		}
	}
}

// bevelToothCircles are the four radii the borrowed spur drawer is asked for, all in
// millimetres and all derived from the EXACT virtual pitch radius.
type bevelToothCircles struct {
	Pitch, Base, Tip, Root float64
	Embedded               bool
}

// bevelCirclesFor derives the four radii §3 step 1 tabulates for one bevelSide.
//
// The virtual tooth number is a REAL number and is never rounded — not floored,
// not ceiled, not cast to an int. The Tredgold construction puts the equivalent
// spur bevelSide's pitch radius exactly at the back-cone distance r / cos gamma with
// this bevelSide's own bevelModuleOf, and z_v = z / cos gamma is a real number in every
// published form of it. A rounded count rebuilds every circle from the rounded
// value, which draws the tooth smaller than the back cone places it: on the
// shipped default the exact virtual pitch radius is 21.9203 mm, a floored count
// of 43 draws 21.5 mm, and the addendum the tooth works over falls to 0.5797 mm
// against a nominal 1.0 bevelModuleOf.
//
// The root circle is drawn one ROOT SINK inside the dedendum corner. At the
// corner exactly the tooth's root arc touches the gear body's root cone only
// where the arc crosses the tooth's own centreline, and the arc's two corners
// stand outside it — by 0.002 bevelModuleOf on the default pair and 0.027 bevelModuleOf on a
// 4/4 pair, the largest of any pair the spec admits. The sink pushes the whole
// arc inside so the Combine-Join meets the body across the root.
func bevelCirclesFor(g bevelGeom, s bevelSide) bevelToothCircles {
	c := bevelToothCircles{
		Pitch: s.VirtualPitch,
		Base:  s.VirtualPitch * math.Cos(bevelPressureAngle),
		Tip:   s.VirtualPitch + g.Module,
		Root:  s.VirtualPitch - 1.25*g.Module - g.RootSink,
	}
	// The flank starts inside the root circle, leaving no room for the
	// flank-to-root lines. This is the spur family's own test — base < root — and
	// the root sink is what can flip it: on the shipped default pair the sink
	// drops the root 0.0405 mm BELOW the base, so the tooth is drawn
	// non-embedded and the drawer adds the two connecting lines.
	c.Embedded = c.Base < c.Root
	return c
}

// bevelFlankSample is one point on the involute flank, already mirrored, centred and
// rotated the way the shared involute package places a tooth.
//
// The three transforms and their order are the involute package's own and are
// load-bearing; this function only re-applies them at a radius the package's
// Flanks sweep does not stop on, which is what the embedded branch needs: there
// the flank must begin exactly on the root circle rather than on the base
// circle.
func bevelFlankSample(baseR, atR, pitchR, toothNumber, angle float64) (left, right bevelVec) {
	x, y, ok := involute.Point(baseR, atR)
	if !ok {
		return bevelVec{}, bevelVec{}
	}
	px, py, _ := involute.Point(baseR, pitchR)
	rotateAngle := math.Pi/(2*toothNumber) - math.Atan2(-py, px)
	lx, ly := involute.Rotate(x, -y, rotateAngle)
	rx, ry := lx, -ly
	lx, ly = involute.Rotate(lx, ly, angle)
	rx, ry = involute.Rotate(rx, ry, angle)
	return bevelVec{lx, ly}, bevelVec{rx, ry}
}

// stepVirtualSpurTooth builds §3 step 3: the `{gearLabel} Tooth` sketch, the
// virtual spur tooth the borrowed drawer draws on the back-cone tooth plane.
//
// What this step owns is what BEVEL supplies: the exact virtual tooth count,
// the four circle radii that follow from it, the root sink, and the loop the
// tooth-profile finder then selects by curve count. The tooth's own involute
// constraint scheme belongs to the spur family and is proved in
// proof/spurgear; this step places the flank samples from the shared involute
// package and fixes them, so the region it detects is the real drawn tooth
// rather than a stand-in, without restating a scheme another proof owns.
//
// The step gates fully constrained, which the Fusion sketch does not: the four
// circles are labelled with along-path sketch text there, text holds a DOF, and
// a labelled sketch never reliably reads fully constrained
// ([PB-TEXT-HOLDS-DOF]). The bevelModuleOf therefore logs that sketch's state and
// never raises on it. This proof draws no labels, so the geometry gates
// normally — which is the check the exemption's earlier wording wrongly
// excused, and under which a deformed tooth shipped.
func stepVirtualSpurTooth(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	c := bevelCirclesFor(g, side)

	proofkit.Step(t, "%s: virtual pitch radius %.4f mm, virtual tooth count %.4f",
		side.Label, side.VirtualPitch, side.VirtualTeeth)
	bevelRequireClose(t, side.Label+" virtual pitch radius",
		side.VirtualPitch, (side.PitchRadius*2*10/2/10)/math.Cos(side.Gamma), 1e-12)
	bevelRequireClose(t, side.Label+" virtual tooth count",
		side.VirtualTeeth, side.Teeth/math.Cos(side.Gamma), 1e-12)

	// The tooth centre K' / L' is the sketch origin of the tooth plane. Where
	// that point sits relative to the shaft axis is part of the placement: K/L
	// is where the back-cone dedendum line crosses the axis, so its radius is 0
	// and its station is R / cos gamma from the apex, and K'/L' is one Tooth
	// Spacing further along that same line, which carries it PAST the axis.
	// Taking the station alone and seating the tooth plane's origin on the axis
	// leaves every tooth point Tooth Spacing * cos gamma too far out, which is
	// the whole of the clearance the input asks for.
	centreOfTooth := g.Kp
	plainCentre := g.K
	if side.Label == "Driving" {
		centreOfTooth, plainCentre = g.Lp, g.L
	}
	bevelRequireClose(t, side.Label+" tooth centre station",
		g.station(side, centreOfTooth), g.R/math.Cos(side.Gamma)+g.ToothSpacing*math.Sin(side.Gamma), 1e-9)
	bevelRequireClose(t, side.Label+" tooth centre radius",
		g.radius(side, centreOfTooth), g.ToothSpacing*math.Cos(side.Gamma), 1e-9)
	bevelRequireClose(t, side.Label+" tooth centre station at Tooth Spacing 0",
		g.station(side, plainCentre), g.R/math.Cos(side.Gamma), 1e-9)

	proofkit.Step(t, "draw the four circles the proxy serves")
	centre := s.CreatePoint(0, 0)
	centre.SetName(side.Label + " tooth centre")
	s.Fix(centre)
	for _, circle := range []struct {
		name string
		r    float64
	}{
		{"pitch", c.Pitch}, {"base", c.Base}, {"tip", c.Tip}, {"root", c.Root},
	} {
		if circle.r <= 0 {
			t.Fatalf("%s %s circle radius is %.6f mm, which cannot be drawn",
				side.Label, circle.name, circle.r)
		}
		drawn := s.CreateCircle(centre, circle.r)
		drawn.SetName(side.Label + " " + circle.name + " circle")
		drawn.SetConstruction(true)
		bevelNamed(s, side.Label+" "+circle.name+" circle radius", sketch.NewRadius(drawn, circle.r))
	}
	bevelRequireClose(t, side.Label+" tip circle radius", c.Tip, side.VirtualPitch+g.Module, 1e-12)
	bevelRequireClose(t, side.Label+" root circle radius",
		c.Root, side.VirtualPitch-1.25*g.Module-g.RootSink, 1e-12)
	bevelRequireClose(t, side.Label+" base circle radius",
		c.Base, side.VirtualPitch*math.Cos(bevelPressureAngle), 1e-12)

	// The tooth is drawn ALREADY ROTATED 180 degrees, by the angle argument the
	// drawer's draw(anchorPoint, angle) takes — never drawn flat and rotated
	// afterwards, which would leave the radially pinned flank-to-root stubs
	// behind.
	const drawAngle = math.Pi
	proofkit.Step(t, "draw the tooth, already rotated 180 degrees, embedded=%v", c.Embedded)

	startR := c.Base
	if c.Embedded {
		// The flank begins exactly on the root circle, which is where the drawn
		// tooth's boundary actually starts when the root sits outside the base.
		startR = c.Root
	}
	leftPts := make([]*sketch.Point, 0, bevelInvoluteSteps)
	rightPts := make([]*sketch.Point, 0, bevelInvoluteSteps)
	for i := range bevelInvoluteSteps {
		at := startR + (c.Tip-startR)*float64(i)/float64(bevelInvoluteSteps-1)
		l, r := bevelFlankSample(c.Base, at, c.Pitch, side.VirtualTeeth, drawAngle)
		lp := s.CreatePoint(l.X, l.Y)
		rp := s.CreatePoint(r.X, r.Y)
		leftPts = append(leftPts, lp)
		rightPts = append(rightPts, rp)
	}

	// The two flanks, tip-first on the left so the loop walks in one direction.
	rightFlank, err := s.CreateFitSpline(rightPts...)
	if err != nil {
		t.Fatalf("%s right flank: %v", side.Label, err)
	}
	rightFlank.SetName(side.Label + " right flank")
	reversedLeft := make([]*sketch.Point, len(leftPts))
	for i := range leftPts {
		reversedLeft[i] = leftPts[len(leftPts)-1-i]
	}
	leftFlank, err := s.CreateFitSpline(reversedLeft...)
	if err != nil {
		t.Fatalf("%s left flank: %v", side.Label, err)
	}
	leftFlank.SetName(side.Label + " left flank")

	tipArc, tipCentre := bevelArcAbout(s, centre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1],
		side.Label+" tooth top arc")

	var lines int
	rootStart, rootEnd := rightPts[0], leftPts[0]
	if !c.Embedded {
		// The two flank-to-root connecting lines, radial, one per flank. Each is
		// DOF-neutral in the spur scheme: it adds a free root end and the two
		// dimensions that bevelPin it.
		lines = 2
		rootStart = bevelRadialFoot(s, rightPts[0], c.Root)
		rootEnd = bevelRadialFoot(s, leftPts[0], c.Root)
		stubR := s.CreateLine(rootStart, rightPts[0])
		stubR.SetName(side.Label + " right flank-to-root line")
		stubL := s.CreateLine(leftPts[0], rootEnd)
		stubL.SetName(side.Label + " left flank-to-root line")
		gap := c.Base - c.Root
		proofkit.Step(t, "%s: the sunk root leaves a %.4f mm flank-to-root stub on each flank",
			side.Label, gap)
	}
	rootArc, rootCentre := bevelArcAbout(s, centre, rootStart, rootEnd, side.Label+" root arc")

	for _, pt := range append(append([]*sketch.Point{rootStart, rootEnd}, leftPts...), rightPts...) {
		s.Fix(pt)
	}

	proofkit.Step(t, "the tooth loop the profile finder selects")
	assertToothLoop(t, s, side, c, lines)

	// The tooth-top arc's own centre. addByCenterStartEnd shares the start and
	// end points but COPIES the centre: the Fusion arc gets a fresh SketchPoint
	// with nothing tying it to the point that was passed
	// ([PB-SHARE-XOR-COINCIDENT]), which is why this one arc needs an explicit
	// coincident where the rest of the recipe would call that redundant. Without
	// it, step 5's anchor coincidence drags everything else onto K'/L' and
	// leaves the stranded centre behind: measured in Fusion on a default 31/31
	// pair, the pinion's tooth-top arc came out at 0.5743 mm and the driving
	// bevelSide's at 17.0204 mm where both should have been the 22.5 mm tip radius,
	// from two sketches with byte-identical constraint counts and dimension
	// values. Both readings below are what the fix restored.
	bevelRequireClose(t, side.Label+" tooth top arc centre gap",
		math.Hypot(tipCentre.X()-centre.X(), tipCentre.Y()-centre.Y()), 0, 1e-9)
	bevelRequireClose(t, side.Label+" tooth top arc radius", tipArc.R(), c.Tip, 1e-6)
	bevelRequireClose(t, side.Label+" root arc centre gap",
		math.Hypot(rootCentre.X()-centre.X(), rootCentre.Y()-centre.Y()), 0, 1e-9)
	bevelRequireClose(t, side.Label+" root arc radius", rootArc.R(), c.Root, 1e-6)
}

// bevelArcAbout builds one of the tooth's two arcs with a centre point of its own,
// pinned back onto the tooth centre.
//
// The separate centre is faithful rather than incidental: Fusion's
// addByCenterStartEnd copies the centre it is handed, so the arc really does
// carry a centre point of its own there, and the recipe's explicit coincident
// is what ties it back. Fusion's coincident is two rows; the bench's arc
// already carries one of them through the internal constraint that keeps its
// start and end equidistant from its centre, which puts that centre on the
// chord's perpendicular bisector — and the tooth centre is on that bisector.
// So the bevelBench states the row that is NOT implied, as a signed distance of zero
// along whichever axis the chord leans further along, which is the axis the
// bisector crosses squarely.
func bevelArcAbout(s *sketch.Sketch, bevelToothCentre, start, end *sketch.Point, name string) (*sketch.Arc, *sketch.Point) {
	centre := s.CreatePoint(bevelToothCentre.X(), bevelToothCentre.Y())
	centre.SetName(name + " centre")
	arc := s.CreateArc(centre, start, end)
	arc.SetName(name)
	if math.Abs(end.Y()-start.Y()) >= math.Abs(end.X()-start.X()) {
		bevelNamed(s, name+" centre on the tooth centre",
			sketch.NewHorizontalDistance(bevelToothCentre, centre, 0))
	} else {
		bevelNamed(s, name+" centre on the tooth centre",
			sketch.NewVerticalDistance(bevelToothCentre, centre, 0))
	}
	return arc, centre
}

// bevelRadialFoot is the root end of one flank-to-root line: the flank's first
// sample projected radially onto the root circle, which is the radial pinning
// the spur family's own scheme uses.
func bevelRadialFoot(s *sketch.Sketch, from *sketch.Point, rootR float64) *sketch.Point {
	d := bevelUnit(bevelVec{from.X(), from.Y()})
	return s.CreatePoint(d.X*rootR, d.Y*rootR)
}

// bevelModuleOf recovers the Module the four circles were derived from, which is what
// the standard tooth thickness is stated in: the tip circle stands one bevelModuleOf
// outside the pitch circle.
func bevelModuleOf(c bevelToothCircles) float64 { return c.Tip - c.Pitch }

// assertToothLoop checks the fact the NEXT step selects on. The tooth
// cross-section is found with find_profile_by_curve_counts(nurbs=2, arcs=2,
// lines=wantLines), and the line count is DETERMINED by the embedded flag,
// never guessed and never accepted either way: an unrelated loop — an
// inter-tooth or annular region between the drawn circles — can carry 2 NURBS
// and 2 arcs with the OTHER line count, and selecting it makes the apex-to-
// profile loft fail with ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY, because the
// impostor loop cannot form a loft tool body.
func assertToothLoop(t testing.TB, s *sketch.Sketch, side bevelSide, c bevelToothCircles, wantLines int) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s tooth sketch did not solve: %v", side.Label, err)
	}
	wantEmbedded := wantLines == 0
	if c.Embedded != wantEmbedded {
		t.Fatalf("%s embedded flag %v does not match the %d connecting lines drawn",
			side.Label, c.Embedded, wantLines)
	}
	var best *sketch.Profile
	for _, prof := range s.Profiles() {
		splines, arcs, lines := 0, 0, 0
		for _, e := range prof.Entities {
			switch e.(type) {
			case *sketch.FitSpline:
				splines++
			case *sketch.Arc:
				arcs++
			case *sketch.Line:
				lines++
			}
		}
		if splines == 2 && arcs == 2 && lines == wantLines {
			best = prof
			break
		}
	}
	if best == nil {
		t.Fatalf("%s: no region carries 2 splines, 2 arcs and %d lines; the tooth-profile "+
			"finder would have nothing to select", side.Label, wantLines)
	}
	if !best.Valid {
		t.Fatalf("%s tooth loop is not an extrudable region", side.Label)
	}
	if best.Area <= 0 {
		t.Fatalf("%s tooth loop has area %.6f", side.Label, best.Area)
	}
	// The drawn tooth reaches the tip circle and no further, and the standard
	// tooth thickness follows from the REAL virtual count: the drawer uses it in
	// one place, the half-thickness angle pi / (2 * z_v), and with
	// z_v = 2 * r_v / Module that angle gives pi * Module / 2 of tooth at the
	// pitch circle — the same thickness the spur bevelSide of this bevelModuleOf carries. An
	// INTEGER count at the exact radius gives pi * r_v / round(z_v) instead,
	// which misses nominal by a different amount on each member of an unequal
	// pair.
	half := math.Pi / (2 * side.VirtualTeeth)
	bevelRequireClose(t, side.Label+" tooth thickness at the pitch circle",
		2*half*c.Pitch, math.Pi*bevelModuleOf(c)/2, 1e-9)
}

// bevelHexVerticesOf is the six §2 points the per-bevelSide Profile sketch recreates, in
// the draw order the table under "Create the Gear Bodies" fixes:
// A' -> G -> H -> C -> M -> N on the pinion and B' -> I -> J -> D -> O -> P on
// the driving bevelSide. The order is what makes the hexagon's FIRST edge the shaft
// axis every body operation then uses.
func bevelHexVerticesOf(g bevelGeom, s bevelSide) ([]bevelVec, []string) {
	if s.Label == "Driving" {
		return []bevelVec{g.Bp, g.I, g.J, g.D, g.O, g.P}, []string{"B'", "I", "J", "D", "O", "P"}
	}
	return []bevelVec{g.Ap, g.G, g.H, g.C, g.M, g.N}, []string{"A'", "G", "H", "C", "M", "N"}
}

// stepGearProfileHexagon builds the per-bevelSide `{gearLabel} Profile` sketch: a
// FRESH sketch on the axial Gear Profiles plane holding exactly this bevelSide's one
// hexagon loop.
//
// One profile sketch per bevelSide is the point. Drawing both gears' hexagons in the
// shared §2 sketch would leave two identically shaped loops to disambiguate,
// where one loop lets the revolve take sketch.profiles.item(0) directly
// ([PB-SINGLE-PROFILE]).
//
// The vertices are built by the recreate-share-fix recipe ([PB-PROJECT-NOT-
// FIXED]): projecting the §2 points would bring them in associatively and leave
// the sketch under-constrained, because a projected point carries free DOF even
// though its position is already correct — the defect is silent until the
// property is read. So each vertex is recreated as a new point at the §2 point's
// exact position, the six lines are drawn SHARING those points, and only then
// are the endpoints fixed. The order is load-bearing: fixing a bare point
// before it is consumed as a line endpoint does not leave the sketch fully
// constrained.
//
// The hexagon's first edge is this gear's shaft axis for the revolve, the
// pattern, the bore plane AND the meshing rotation, so it has to carry a
// trustworthy world position: fixed endpoints give that edge a well-defined
// worldGeometry ([PB-WORLDGEO-CONSTRAINED]), while a free edge resolves against
// a default frame and silently moves the body onto world XY — observed on the
// driving bevelSide, the pinion looking fine only because it never read the edge.
func stepGearProfileHexagon(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := resolveBevel(p)
	side, _ := g.side(p)
	verts, names := bevelHexVerticesOf(g, side)

	proofkit.Step(t, "%s Profile: recreate the six §2 vertices at their exact positions", side.Label)
	pts := make([]*sketch.Point, len(verts))
	for i, v := range verts {
		pts[i] = s.CreatePoint(v.X, v.Y)
		pts[i].SetName(side.Label + " " + names[i])
	}

	proofkit.Step(t, "draw the closed hexagon sharing those points, in draw order")
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		lines[i].SetName(side.Label + " " + names[i] + "->" + names[(i+1)%len(names)])
	}

	proofkit.Step(t, "fix the lines' endpoints, AFTER the lines exist")
	for _, l := range lines {
		s.Fix(l.Start)
		s.Fix(l.End)
	}

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s Profile sketch did not solve: %v", side.Label, err)
	}

	// Exactly one loop, six line edges, and no other curve type: this is what
	// lets the revolve take the sketch's single profile rather than filtering,
	// and a curve-type filter has spuriously rejected a valid all-line loop and
	// made the revolve fail with "could not find profile".
	profiles := s.Profiles()
	if len(profiles) != 1 {
		t.Fatalf("%s Profile sketch holds %d regions, want exactly 1", side.Label, len(profiles))
	}
	prof := profiles[0]
	if !prof.Valid || prof.SelfIntersecting {
		t.Fatalf("%s hexagon is not a valid region (valid=%v selfIntersecting=%v): the profile "+
			"has crossed its own axis of revolution, which Fusion aborts with ASM_WIRE_X_AXIS",
			side.Label, prof.Valid, prof.SelfIntersecting)
	}
	if len(prof.Outer) != 6 {
		t.Fatalf("%s hexagon boundary carries %d edges, want 6", side.Label, len(prof.Outer))
	}
	for _, e := range prof.Entities {
		if _, ok := e.(*sketch.Line); !ok {
			t.Fatalf("%s hexagon boundary carries a %T; every edge must be a line", side.Label, e)
		}
	}

	// The first edge IS the shaft axis: both its endpoints sit on the axis, at
	// radius zero, and the revolve, the pattern, the bore plane and the meshing
	// rotation all take it. The §2 Apex->A / Apex->B construction line is NOT
	// usable for any of them — it lives in a different sketch.
	bevelRequireClose(t, side.Label+" first edge start radius", g.radius(side, verts[0]), 0, 1e-9)
	bevelRequireClose(t, side.Label+" first edge end radius", g.radius(side, verts[1]), 0, 1e-9)
	// The toe corner never sits on the axis. Pinning it there puts a corner ON
	// the axis of revolution and the later conical split fails with
	// ASM_API_FAILED for asymmetric tooth counts, even though the symmetric case
	// happens to survive; the Toe Radius is strictly positive and is what holds
	// it off.
	toe := g.radius(side, verts[5])
	if toe <= 0 {
		t.Fatalf("%s toe corner sits at radius %.6f, on the axis of revolution", side.Label, toe)
	}
	bevelRequireClose(t, side.Label+" toe corner radius is the resolved Toe Radius",
		toe, side.ToeRadius, 1e-9)

	// The area the region reports is the area the closed form gives for the same
	// six vertices, which is the reading the revolve's Pappus check then rests
	// on.
	bevelRequireClose(t, side.Label+" hexagon area", prof.Area, math.Abs(bevelShoelace(verts)), 1e-6)
}

// bevelShoelace is the signed area of a closed polygon walked in the given order.
func bevelShoelace(pts []bevelVec) float64 {
	total := 0.0
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		total += bevelCross(a, b)
	}
	return total / 2
}

// bevelTraceFrame is the flat 2-D crown-plane frame the cutter arc is drawn in:
// origin at the apex, x along the cone element so a point's x IS its cone
// distance, y circumferential.
type bevelTraceFrame struct {
	RToe, RHeel, RMean, Span float64
	Cutter                   float64 // r_c
	Cx, Cy                   float64 // the cutter-circle centre
	Toe2D, Heel2D            bevelVec
	HandSign                 float64
	Psi                      float64
}

// bevelTraceFor builds the cutter-arc geometry §3a steps A and B derive.
func bevelTraceFor(g bevelGeom, s bevelSide, handSign float64) bevelTraceFrame {
	var toeEdge, heelEdge [2]bevelVec
	if s.Label == "Driving" {
		toeEdge, heelEdge = [2]bevelVec{g.O, g.P}, [2]bevelVec{g.D, g.J}
	} else {
		toeEdge, heelEdge = [2]bevelVec{g.M, g.N}, [2]bevelVec{g.C, g.H}
	}
	// toeMid is the midpoint of the TOE edge and heelMid the midpoint of the
	// HEEL edge — two DIFFERENT edges. Passing the two endpoints of a single
	// edge collapses the span to about zero or negative and the spiral inverts
	// with no error anywhere.
	toeMid := bevelMul(bevelAdd(toeEdge[0], toeEdge[1]), 0.5)
	heelMid := bevelMul(bevelAdd(heelEdge[0], heelEdge[1]), 0.5)
	// coneVec runs along the ROOT cone element Apex->C / Apex->D, whose heel end
	// is the dedendum corner C/D and NEVER H/J: H and J lie on the Apex2->C /
	// Apex2->D dedendum line, one bevelModuleOf beyond C/D and off the root element, so
	// using them skews coneVec.
	coneVec := bevelUnit(bevelSub(heelEdge[0], g.Apex))
	distAlong := func(p bevelVec) float64 { return bevelDot(bevelSub(p, g.Apex), coneVec) }

	var f bevelTraceFrame
	f.HandSign = handSign
	f.Psi = g.SpiralAngle
	f.RToe, f.RHeel = distAlong(toeMid), distAlong(heelMid)
	f.RMean = 0.5 * (f.RToe + f.RHeel)
	f.Span = f.RHeel - f.RToe
	f.Cutter = g.CutterRadius
	if f.Cutter == 0 {
		f.Cutter = f.RMean
	}
	// The hand sign goes on the cos / Cy term, NEVER on the sin / Cx term.
	// Opposite hands mirror the cutter centre across the cone element (y = 0),
	// which flips Cy; putting the sign on Cx mirrors about x = R_mean instead,
	// which is a different curve and gives the two gears unequal twist.
	f.Cx = f.RMean - f.Cutter*math.Sin(f.Psi)
	f.Cy = handSign * f.Cutter * math.Cos(f.Psi)
	// The ends are taken a hair PAST the face so the kept arc reaches cleanly
	// past the end trims.
	f.Toe2D = bevelCircleIntersectNearest(f.RToe-0.06*f.Span, f.Cx, f.Cy, f.Cutter, f.RMean, 0)
	f.Heel2D = bevelCircleIntersectNearest(f.RHeel+0.06*f.Span, f.Cx, f.Cy, f.Cutter, f.RMean, 0)
	return f
}

// bevelCircleIntersectNearest intersects the apex circle of radius r with the cutter
// circle and keeps the solution nearest the reference point, which is the
// branch the mean point sits on. A non-overlapping pair clamps to tangency.
func bevelCircleIntersectNearest(r, cx, cy, rc, refX, refY float64) bevelVec {
	d := math.Hypot(cx, cy)
	if d == 0 {
		return bevelVec{r, 0}
	}
	a := (d*d + r*r - rc*rc) / (2 * d)
	h2 := r*r - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	base := bevelVec{cx * a / d, cy * a / d}
	off := bevelVec{-cy * h / d, cx * h / d}
	p1, p2 := bevelAdd(base, off), bevelSub(base, off)
	if bevelLen(bevelSub(p1, bevelVec{refX, refY})) <= bevelLen(bevelSub(p2, bevelVec{refX, refY})) {
		return p1
	}
	return p2
}

// stepSpiralTrace builds §3a step C: the `{gear} 2D Tooth Trace` sketch, the
// genuine cutter arc a face-mill of radius r_c sweeps.
//
// The Fusion sketch is DELIBERATELY left with free DOF — the arc's endpoints
// are pinned by three-point construction rather than dimensioned, because
// dimensioning them over-constrains the solve against the cone-element plane —
// and it is exempt from the full-constraint gate. The bevelBench gate is not
// optional, so this proof pins the two endpoints at the closed-form circle-
// circle intersections instead and lets the arc's radius follow. What that
// costs: the radius Fusion writes as a dimension is asserted here rather than
// driven. The assertion is the same claim — that the trace is the genuine
// cutter circle and not a look-alike spline.
//
// The proof also drops the trace sketch's placement entirely. Fusion builds it
// on a Trace Plane made by rotating the axial plane 90 degrees about a cone-
// element line, and passes world Point3Ds straight into the sketch calls with
// no modelToSketchSpace conversion — which places that plane somewhere other
// than the true tangent plane. That is harmless only because NO downstream
// feature consumes the trace sketch or the Trace Plane: the twist is computed
// analytically from the 2-D endpoints. This proof works in the flat frame those
// endpoints live in, which is the frame the twist law reads, so the placement
// is outside what it can check. If a later revision ever makes a feature
// consume either, the shortcut stops being safe and both sketches need
// modelToSketchSpace on every point.
func stepSpiralTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	g := resolveBevel(p)
	side, handSign := g.side(p)
	f := bevelTraceFor(g, side, handSign)

	proofkit.Step(t, "%s: R_toe %.4f R_heel %.4f R_mean %.4f span %.4f r_c %.4f handSign %+.0f",
		side.Label, f.RToe, f.RHeel, f.RMean, f.Span, f.Cutter, f.HandSign)
	// The heel MUST be the outer end so coneVec points outward and span is
	// positive. A negative span silently inverts the whole spiral frame — the
	// cutter-arc direction, the slice direction and the per-segment twist — and
	// the bevelSide comes out completely wrong with no error at all.
	if f.Span <= 0 {
		t.Fatalf("%s span is %.6f: the heel is not the outer end and the spiral frame is inverted",
			side.Label, f.Span)
	}

	proofkit.Step(t, "the cutter circle and the trace arc that is an arc OF it")
	apex := s.CreatePoint(0, 0)
	apex.SetName("apex")
	s.Fix(apex)
	toe := s.CreatePoint(f.Toe2D.X, f.Toe2D.Y)
	toe.SetName("trace toe end")
	s.Fix(toe)
	heel := s.CreatePoint(f.Heel2D.X, f.Heel2D.Y)
	heel.SetName("trace heel end")
	s.Fix(heel)

	// The arc's centre is free and is pinned by the arc's own internal
	// equidistance — which puts it on the chord's perpendicular bisector — plus
	// one signed row along the axis that bisector crosses squarely.
	centre := s.CreatePoint(f.Cx, f.Cy)
	centre.SetName("cutter circle centre")
	arc := s.CreateArc(centre, toe, heel)
	arc.SetName("trace arc")
	if math.Abs(f.Heel2D.Y-f.Toe2D.Y) >= math.Abs(f.Heel2D.X-f.Toe2D.X) {
		bevelNamed(s, "cutter centre station", sketch.NewHorizontalDistance(apex, centre, f.Cx))
	} else {
		bevelNamed(s, "cutter centre offset", sketch.NewVerticalDistance(apex, centre, f.Cy))
	}

	// The cutter circle SHARES the arc's centre, which is how the bevelBench says
	// "centre coincident to the cutter circle's centre" ([PB-SHARE-XOR-
	// COINCIDENT]); its own diameter dimension is the only thing it needs.
	cutter := s.CreateCircle(centre, f.Cutter)
	cutter.SetName("cutter circle")
	cutter.SetConstruction(true)
	bevelNamed(s, "cutter circle diameter", sketch.NewDiameter(cutter, 2*f.Cutter))

	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("%s trace sketch did not solve: %v", side.Label, err)
	}

	proofkit.Step(t, "the trace invariants")
	bevelRequireClose(t, side.Label+" cutter centre x", centre.X(), f.Cx, 1e-9)
	bevelRequireClose(t, side.Label+" cutter centre y", centre.Y(), f.Cy, 1e-9)
	// Invariant 2: the arc's radius is r_c everywhere, because it is one circle.
	bevelRequireClose(t, side.Label+" trace arc radius", arc.R(), f.Cutter, 1e-6)
	// Invariant 3: it passes through the mean point, and the centre is r_c from
	// it, which is what makes the centre construction correct rather than merely
	// plausible.
	bevelRequireClose(t, side.Label+" cutter centre distance from the mean point",
		bevelLen(bevelSub(bevelVec{f.Cx, f.Cy}, bevelVec{f.RMean, 0})), f.Cutter, 1e-9)
	// Invariant 6: the ends sit on the apex circles they were intersected with,
	// taken a hair past the face at each end.
	bevelRequireClose(t, side.Label+" trace toe end cone distance",
		bevelLen(f.Toe2D), f.RToe-0.06*f.Span, 1e-6)
	bevelRequireClose(t, side.Label+" trace heel end cone distance",
		bevelLen(f.Heel2D), f.RHeel+0.06*f.Span, 1e-6)
	// Invariant 4: the angle between the arc's tangent at the mean point and the
	// cone element IS psi. The tangent is perpendicular to the radius, so the
	// tangent direction at the mean point is the radius turned a quarter turn.
	radial := bevelUnit(bevelSub(bevelVec{f.RMean, 0}, bevelVec{f.Cx, f.Cy}))
	tangent := bevelVec{-radial.Y, radial.X}
	measured := math.Abs(math.Atan2(math.Abs(tangent.Y), math.Abs(tangent.X)))
	bevelRequireClose(t, side.Label+" spiral angle realised at the mean point",
		measured*180/math.Pi, math.Abs(f.Psi)*180/math.Pi, 1e-6)
	// Invariant 5: flipping the hand reflects the whole construction across the
	// cone element and changes nothing else. The mirrored construction is the
	// same case with handSign negated, so this reads it directly rather than
	// building a second sketch.
	mirror := bevelTraceFor(g, side, -f.HandSign)
	bevelRequireClose(t, side.Label+" mirrored cutter centre station", mirror.Cx, f.Cx, 1e-9)
	bevelRequireClose(t, side.Label+" mirrored cutter centre offset", mirror.Cy, -f.Cy, 1e-9)
	bevelRequireClose(t, side.Label+" mirrored toe end x", mirror.Toe2D.X, f.Toe2D.X, 1e-6)
	bevelRequireClose(t, side.Label+" mirrored toe end y", mirror.Toe2D.Y, -f.Toe2D.Y, 1e-6)
	// Invariant 7: at psi = 0 the centre sits straight north of the mean point,
	// so the arc is TANGENT to the cone element there — and only there. It still
	// curves away toward toe and heel, so its ends still subtend an angle at the
	// apex. No built tooth carries that residual: the hook's psi = 0 gate
	// returns the straight-tooth path before any of this runs, which is what the
	// straight-tooth steps prove.
	if f.Psi == 0 {
		bevelRequireClose(t, "psi = 0 puts the cutter centre straight north of the mean point",
			f.Cx, f.RMean, 1e-12)
		bevelRequireClose(t, "psi = 0 puts the cutter centre one cutter radius out",
			math.Abs(f.Cy), f.Cutter, 1e-12)
	}
}
