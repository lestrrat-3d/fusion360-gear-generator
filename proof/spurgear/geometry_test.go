package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
)

// This file holds what the spur proof's sketch and solid halves share: the
// parameter keys, the derived circle radii, and the case tables the regime in
// spec/spurgear/instructions.md "Sketch Discipline" asks for. The drawing
// itself is in sketches_test.go and the bodies are in solids_test.go.
//
// Every length here is a millimetre. Fusion works in internal centimetres and
// the spec quotes millimetres; the bench picks one and states it, because a
// mixed unit is how a radius ends up ten times too big.

// Parameter keys. A case table is a map, so a misspelled key reads as a zero
// value rather than as a failure; naming them once is what stops that.
const (
	pModule        = "module"
	pToothNumber   = "toothNumber"
	pPressureAngle = "pressureAngle"
	pSteps         = "involuteSteps"
	pAngle         = "angle"
	pAnchorX       = "anchorX"
	pAnchorY       = "anchorY"
	pThickness     = "thickness"
	pBoreDiameter  = "boreDiameter"
	pChamfer       = "chamfer"

	// pFreeArcCentre drives the negative control of [SPUR-F-TOOTHTOP-ARC]: when
	// it is set the tooth-top arc's centre is left where addByCenterStartEnd
	// copied it instead of being tied back to the local origin.
	pFreeArcCentre = "freeArcCentre"
)

// dims are the four circle radii plus the sample count and rotation one case
// draws at, in millimetres and radians.
type dims struct {
	involute.Dimensions
	module      float64
	toothNumber float64
	steps       int
	angle       float64
	anchorX     float64
	anchorY     float64
}

// derive reads one case's parameters and applies the formulas in
// spec/spurgear/instructions.md "Variables": pitch = module * toothNumber,
// base = pitch * cos(pressureAngle), root = pitch - 2.5 * module of diameter,
// tip = pitch + 2 * module of diameter. involute.Derive owns them so spur,
// helical and herringbone cannot drift apart.
func derive(p map[string]float64) dims {
	return dims{
		Dimensions:  involute.Derive(p[pModule], p[pToothNumber], p[pPressureAngle]),
		module:      p[pModule],
		toothNumber: p[pToothNumber],
		steps:       int(p[pSteps]),
		angle:       p[pAngle],
		anchorX:     p[pAnchorX],
		anchorY:     p[pAnchorY],
	}
}

// toothSpaceArc is the arc length of one valley at the root circle:
// rootRadius * (pi/N - 2*(tan(alpha) - alpha)), the ToothSpaceAngleAtRoot and
// ToothSpaceArcAtRoot parameters of the spec's Variables section, evaluated in
// Python there because Fusion's expression engine refuses the mixed-unit
// subtraction.
func toothSpaceArc(d dims, pressureAngle float64) float64 {
	angle := math.Pi/d.toothNumber - 2*(math.Tan(pressureAngle)-pressureAngle)
	return d.Root * angle
}

// filletRadius is (ToothSpaceArcAtRoot / 2) * FilletClearance * 1, the spur
// base's FilletRadius. The trailing factor is the helix hook, 1 for spur.
func filletRadius(d dims, pressureAngle float64) float64 {
	return toothSpaceArc(d, pressureAngle) / 2 * filletClearance * 1
}

// filletClearance is the spec's fixed 0.9: the fraction of the half-valley arc
// the root fillet takes, leaving a small flat strip rather than meeting its
// neighbour at the valley midpoint.
const filletClearance = 0.9

// radians is a spelling convenience for the pressure and rotation angles the
// tables quote in degrees, the way the dialog does.
func radians(deg float64) float64 { return deg * math.Pi / 180 }

// sketchCase and solidCase build one case's parameter map. A case names only
// what it varies; the rest comes from the standard 17-tooth module-1 gear the
// dialog defaults to.
func params(overrides map[string]float64) map[string]float64 {
	p := map[string]float64{
		pModule:        1,
		pToothNumber:   17,
		pPressureAngle: radians(20),
		pSteps:         15,
		pAngle:         0,
		pAnchorX:       0,
		pAnchorY:       0,
		pThickness:     10,
		pBoreDiameter:  0,
		pChamfer:       0,
		pFreeArcCentre: 0,
	}
	for k, v := range overrides {
		p[k] = v
	}
	return p
}

// profileCases is the regime spec/spurgear/instructions.md "Sketch Discipline"
// states the Gear Profile scheme has to hold across, one case per reason it
// gives:
//
//   - Size. Coarse and fine module/tooth pairs, because the rib chain's
//     dimensions scale with the tooth and the conditioning does not.
//   - The whole signed range of the angle argument. Zero for spur, a positive
//     and a NEGATIVE helix angle (a left-hand helix passes a negative value),
//     a quarter turn where |sin| > |cos| swaps which axis the rib and the
//     chain dimensions take, and 180 degrees for the bevel virtual tooth.
//   - The rib count. One rib, one across-spine dimension and one chain
//     dimension exist per involute sample, so a handful of samples is the case
//     where one missing or redundant dimension is a large fraction of the
//     system.
//   - Both routes into the embedded shape, which is reached when
//     toothNumber * (1 - cos(pressureAngle)) > 2.5: a high tooth count at the
//     ordinary 20 degrees, and a moderate tooth count at a large pressure
//     angle. The two arrive at the same missing-stub geometry through
//     different terms.
//
// Several cases also drag the sketch onto an anchor away from the sketch
// origin. That is not decoration: [SPUR-F-TOOTHTOP-ARC] records that a spur
// gear HIDES a stranded arc centre precisely because its anchor usually sits
// at the origin, where the drag is zero, and the same scheme collapsed on the
// bevel gear at an 8-36 mm drag. A table of un-dragged cases proves the rule
// the spur gear is least able to see.
var profileCases = []proofkit.Case{
	{Name: "standard-m1-t17", Params: params(nil)},
	{Name: "standard-m1-t17-dragged", Params: params(map[string]float64{
		pAnchorX: 23.5, pAnchorY: -11.25})},
	{Name: "coarse-m5-t12", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12})},
	{Name: "coarse-m5-t12-dragged", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12, pAnchorX: -36, pAnchorY: 18})},
	{Name: "fine-m0.5-t40", Params: params(map[string]float64{
		pModule: 0.5, pToothNumber: 40})},
	{Name: "angle-positive-30deg", Params: params(map[string]float64{
		pAngle: radians(30)})},
	{Name: "angle-negative-30deg", Params: params(map[string]float64{
		pAngle: radians(-30)})},
	{Name: "angle-negative-30deg-dragged", Params: params(map[string]float64{
		pAngle: radians(-30), pAnchorX: 14, pAnchorY: 9})},
	{Name: "angle-quarter-turn", Params: params(map[string]float64{
		pAngle: radians(90)})},
	{Name: "angle-negative-quarter-turn", Params: params(map[string]float64{
		pAngle: radians(-90)})},
	{Name: "angle-half-turn", Params: params(map[string]float64{
		pAngle: radians(180)})},
	{Name: "few-samples-3", Params: params(map[string]float64{
		pSteps: 3})},
	{Name: "few-samples-3-rotated", Params: params(map[string]float64{
		pSteps: 3, pAngle: radians(45)})},
	{Name: "embedded-by-tooth-count-t60-pa20", Params: params(map[string]float64{
		pToothNumber: 60})},
	{Name: "embedded-by-pressure-angle-t30-pa25", Params: params(map[string]float64{
		pToothNumber: 30, pPressureAngle: radians(25)})},
	{Name: "embedded-dragged", Params: params(map[string]float64{
		pToothNumber: 60, pAnchorX: -21, pAnchorY: -7})},
}

// boreCases sweep the Bore Profile sketch of step 12 across the bore diameters
// the dialog accepts above zero, at the gear sizes that bound them, and both
// on and off the sketch origin. A bore at the origin is the case that cannot
// tell a point grounded on the projected anchor from one grounded on the
// sketch's own originPoint, which is the substitution [PB-CIRCLE-CENTER]
// records a solver failure for.
var boreCases = []proofkit.Case{
	{Name: "bore-3mm", Params: params(map[string]float64{pBoreDiameter: 3})},
	{Name: "bore-3mm-dragged", Params: params(map[string]float64{
		pBoreDiameter: 3, pAnchorX: 17.5, pAnchorY: -6.25})},
	{Name: "bore-small-0.4mm", Params: params(map[string]float64{
		pBoreDiameter: 0.4, pModule: 0.5, pToothNumber: 40})},
	{Name: "bore-large-40mm", Params: params(map[string]float64{
		pBoreDiameter: 40, pModule: 5, pToothNumber: 12, pAnchorX: -8, pAnchorY: 12})},
}

// solidCases are the sizes the solid steps build at. They are a subset of
// profileCases: every solid step re-draws the same Gear Profile sketch and
// then extrudes it, so the sketch regime is already proven above and what is
// added here is the thickness sweep and one case per profile SHAPE — the
// six-curve tooth with its flank-to-root stubs, and the four-curve embedded
// tooth whose flanks cross the root circle instead.
var solidCases = []proofkit3d.Case{
	{Name: "standard-m1-t17-th10", Params: params(nil)},
	{Name: "standard-m1-t17-thin", Params: params(map[string]float64{pThickness: 1.5})},
	{Name: "coarse-m5-t12-th25", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12, pThickness: 25})},
	{Name: "embedded-t60-th10", Params: params(map[string]float64{pToothNumber: 60})},
}

// patternCases are the tooth counts step 10 patterns at: the dialog default,
// a coarse twelve-tooth gear and a fine forty-tooth one. The table sweeps the
// COUNT, because that is what the step decides — the quantity, the full turn
// it is spread over, and where each copy lands. It does not sweep the tooth's
// shape: the pattern moves whatever body it is given, and the two shapes the
// profile can take are built and proven by steps 7 and 9.
var patternCases = []proofkit3d.Case{
	{Name: "teeth-17-m1", Params: params(nil)},
	{Name: "teeth-12-m5", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12, pThickness: 25})},
	{Name: "teeth-9-m2", Params: params(map[string]float64{
		pModule: 2, pToothNumber: 9, pThickness: 6})},
}

// boreSolidCases add the bore diameter to the solid sweep. Step 12 returns
// early at a diameter of zero, so every case here is above it.
var boreSolidCases = []proofkit3d.Case{
	{Name: "bore-3mm", Params: params(map[string]float64{pBoreDiameter: 3})},
	{Name: "bore-6mm-thin", Params: params(map[string]float64{
		pBoreDiameter: 6, pThickness: 1.5})},
	{Name: "bore-20mm-coarse", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12, pThickness: 25, pBoreDiameter: 20})},
}

// chamferCases add the chamfer distance. Step 13 returns early at zero, and
// every case here carries a bore as well, because the rule the step's edge set
// turns on is that a bore edge is excluded from it.
var chamferCases = []proofkit3d.Case{
	{Name: "chamfer-0.4mm-bore-3mm", Params: params(map[string]float64{
		pBoreDiameter: 3, pChamfer: 0.4})},
	{Name: "chamfer-0.2mm-thin", Params: params(map[string]float64{
		pBoreDiameter: 3, pChamfer: 0.2, pThickness: 1.5})},
	{Name: "chamfer-2mm-coarse", Params: params(map[string]float64{
		pModule: 5, pToothNumber: 12, pThickness: 25, pBoreDiameter: 20, pChamfer: 2})},
}

// mustSteps guards the one parameter the tables can get wrong in a way the
// geometry would absorb: the sampling loop divides by steps-1, so a table
// entry of 1 or 0 produces an infinity rather than a failure.
func mustSteps(t testing.TB, d dims) {
	t.Helper()
	if d.steps < 2 {
		t.Fatalf("involuteSteps is %d; the sampling loop divides by steps-1", d.steps)
	}
}
