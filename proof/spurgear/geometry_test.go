// Package spurgear_test proves the spur gear's modelling workflow: the Gear
// Profile constraint scheme in the sketch engine, and the extrude, pattern,
// combine, fillet, bore and chamfer steps in decad.
//
// Every length here is a millimetre. Fusion works in centimetres internally;
// the step list carries that conversion, the proof does not.
package spurgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
)

// Parameter keys shared by every case table. Angles are degrees so a case
// reads the way the dialog does; the builders convert.
const (
	pModule        = "module"           // mm
	pToothNumber   = "toothNumber"      // count
	pPressureAngle = "pressureAngleDeg" // degrees
	pAngle         = "angleDeg"         // the draw(anchorPoint, angle) argument, degrees, signed
	pInvoluteSteps = "involuteSteps"    // samples per flank
	pAnchorX       = "anchorX"          // mm, where the projected Tools anchor lands in the sketch
	pAnchorY       = "anchorY"          // mm
	pThickness     = "thickness"        // mm
	pBoreDiameter  = "boreDiameter"     // mm, 0 means no bore
	pChamfer       = "chamferTooth"     // mm, 0 means no chamfer
)

// gear is one case's derived geometry: the spec's Variables section in mm.
type gear struct {
	module, toothNumber, pressureAngle, angle float64
	steps                                     int
	anchorX, anchorY                          float64
	thickness, bore, chamfer                  float64
	involute.Dimensions
	// toothSpaceAngle is Tooth Space Angle At Root, radians:
	// pi/N - 2*(tan(PA) - PA). toothSpaceArc is Root Circle Radius times it.
	// filletRadius is (toothSpaceArc / 2) * FilletClearance * 1, the spur
	// base's helix factor being 1.
	toothSpaceAngle, toothSpaceArc, filletRadius float64
}

// filletClearance is the spec's Fillet Clearance constant.
const filletClearance = 0.9

// derive turns a case's parameters into the gear's circles and root-fillet
// figures, exactly as the spec's Variables section computes them.
func derive(t testing.TB, p map[string]float64) gear {
	t.Helper()
	g := gear{
		module:        p[pModule],
		toothNumber:   p[pToothNumber],
		pressureAngle: p[pPressureAngle] * math.Pi / 180,
		angle:         p[pAngle] * math.Pi / 180,
		steps:         int(p[pInvoluteSteps]),
		anchorX:       p[pAnchorX],
		anchorY:       p[pAnchorY],
		thickness:     p[pThickness],
		bore:          p[pBoreDiameter],
		chamfer:       p[pChamfer],
	}
	if g.module <= 0 || g.toothNumber < 3 {
		t.Fatalf("case needs a positive module and at least 3 teeth, got module=%v teeth=%v", g.module, g.toothNumber)
	}
	if g.steps < 2 {
		t.Fatalf("case needs at least 2 involute steps, got %d", g.steps)
	}
	g.Dimensions = involute.Derive(g.module, g.toothNumber, g.pressureAngle)
	g.toothSpaceAngle = math.Pi/g.toothNumber - 2*(math.Tan(g.pressureAngle)-g.pressureAngle)
	g.toothSpaceArc = g.Root * g.toothSpaceAngle
	g.filletRadius = (g.toothSpaceArc / 2) * filletClearance * 1
	return g
}

// pt is a plane-local point in mm.
type pt struct{ x, y float64 }

// rotated returns p turned counter-clockwise by a radians about the origin.
func rotated(p pt, a float64) pt {
	x, y := involute.Rotate(p.x, p.y, a)
	return pt{x, y}
}

// flanks returns the left and right flank samples of the tooth at angle,
// base circle to tip circle, exactly as step 4 samples them: endpoint-
// inclusive from Base Circle Radius to Tip Circle Radius, samples inside
// the base circle dropped, mirrored, rotated onto +X, then turned by angle.
func (g gear) flanks(angle float64) (left, right []pt) {
	l, r := involute.Flanks(g.Base, g.Tip, g.Pitch, g.toothNumber, g.steps, angle)
	for i := range l {
		left = append(left, pt{l[i].X, l[i].Y})
		right = append(right, pt{r[i].X, r[i].Y})
	}
	return left, right
}

// flankPointAt returns the left and right flank points at radius r for the
// tooth at angle, under the same mirror and rotation the flank samples get.
// It is the analytic position of a flank where it crosses a circle of
// radius r, which the solid steps need for the embedded case.
func (g gear) flankPointAt(r, angle float64) (left, right pt) {
	px, py, _ := involute.Point(g.Base, g.Pitch)
	rotateAngle := math.Pi/(2*g.toothNumber) - math.Atan2(-py, px)
	x, y, _ := involute.Point(g.Base, r)
	lx, ly := involute.Rotate(x, -y, rotateAngle)
	rx, ry := lx, -ly
	lx, ly = involute.Rotate(lx, ly, angle)
	rx, ry = involute.Rotate(rx, ry, angle)
	return pt{lx, ly}, pt{rx, ry}
}

// rootEnd is where the flank-to-root line meets the root circle: the flank
// start pulled radially onto Root Circle Radius.
func (g gear) rootEnd(flankStart pt) pt {
	d := math.Hypot(flankStart.x, flankStart.y)
	return pt{flankStart.x * g.Root / d, flankStart.y * g.Root / d}
}

// near reports whether a and b agree within rel of |b| (or abs when b is 0).
func near(a, b, rel float64) bool {
	if b == 0 {
		return math.Abs(a) <= rel
	}
	return math.Abs(a-b) <= rel*math.Abs(b)
}
