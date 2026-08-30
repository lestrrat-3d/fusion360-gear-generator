// Package helicalgear_test proves the helical gear's own build steps.
//
// Helical is a thin specialization of spur: it inherits the whole spur
// pipeline and changes three things — one extra input, a second "Twisted Gear
// Profile" sketch drawn by the spur tooth generator at angle=HelixAngle, and a
// loft between the two profiles instead of an extrude. The proof covers those
// three, and nothing spur already owns.
//
// This file holds the tooth geometry the three step functions share. The
// involute math itself comes from the involute package, so the flank the proof
// draws is the flank the spur tooth generator draws, not a second derivation of
// it.
package helicalgear_test

import (
	"math"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
)

// params reads one case's parameters. Every angle is in radians, matching the
// HelixAngle user parameter, which the spec registers in 'rad' even though the
// dialog input is in degrees.
type params map[string]float64

func (p params) module() float64        { return p["module"] }
func (p params) toothNumber() float64   { return p["toothNumber"] }
func (p params) pressureAngle() float64 { return p["pressureAngle"] }
func (p params) helixAngle() float64    { return p["helixAngle"] }
func (p params) thickness() float64     { return p["thickness"] }
func (p params) involuteSteps() int     { return int(p["involuteSteps"]) }

func (p params) dimensions() involute.Dimensions {
	return involute.Derive(p.module(), p.toothNumber(), p.pressureAngle())
}

// rad converts a dialog-facing degree figure to the radians the parameter
// holds. The case tables are written in degrees because that is how the spec
// states the Helix Angle default, 14.5 deg.
func rad(deg float64) float64 { return deg * math.Pi / 180 }

func deg(rad float64) float64 { return rad * 180 / math.Pi }

// toothOutline returns the six corners of one tooth's cross-section, in
// counter-clockwise loop order, at the requested angle.
//
// The corners are the real tooth's: the two flank-to-root feet on the root
// circle, the two flank starts on the base circle, and the two flank ends on
// the tip circle, all taken from the same involute samples the tooth generator
// draws its splines through. Only what runs BETWEEN the corners differs between
// the sketch step and the solid steps — see sectionCorners' callers.
func toothOutline(p params, angle float64) []involute.Pt {
	dim := p.dimensions()
	left, right := involute.Flanks(dim.Base, dim.Tip, dim.Pitch, p.toothNumber(), p.involuteSteps(), angle)
	return []involute.Pt{
		footOnRoot(right[0], dim.Root),
		right[0],
		right[len(right)-1],
		left[len(left)-1],
		left[0],
		footOnRoot(left[0], dim.Root),
	}
}

// footOnRoot is the flank-to-root line's root end: the flank start pulled
// radially in to the root circle, which is what [SPUR-F-FLANK-ROOT]'s two axis
// dimensions place it at.
func footOnRoot(flankStart involute.Pt, rootRadius float64) involute.Pt {
	n := math.Hypot(flankStart.X, flankStart.Y)
	return involute.Pt{X: rootRadius * flankStart.X / n, Y: rootRadius * flankStart.Y / n}
}

// outlineArea is the shoelace area of a corner loop, in mm^2. The solid steps
// check their bodies against it rather than against a volume the same evaluator
// reported.
func outlineArea(loop []involute.Pt) float64 {
	sum := 0.0
	for i, a := range loop {
		b := loop[(i+1)%len(loop)]
		sum += a.X*b.Y - b.X*a.Y
	}
	return math.Abs(sum) / 2
}

// polarAngle is the counter-clockwise angle of (x, y) from +X, in radians.
func polarAngle(x, y float64) float64 { return math.Atan2(y, x) }
