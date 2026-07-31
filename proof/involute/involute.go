// Package involute holds the tooth math the spur family shares.
//
// Spur, helical and herringbone all draw the same involute tooth; helical and
// herringbone simply draw it rotated. This package exists so that math lives in
// one place without pushing gear geometry into the harness, which has to stay
// gear-agnostic.
package involute

import "math"

// Pt is a plane-local point in millimetres.
type Pt struct{ X, Y float64 }

// Point returns the point on the involute of baseRadius at the radius where the
// unrolled string reaches intersectionRadius.
//
// ok is false when intersectionRadius is inside the base circle, where the curve
// does not exist; callers drop those samples.
//
// The curve parameter is tan(alpha), not inv(alpha) = tan(alpha) - alpha. Using
// the involute function here is a common substitution and produces a flank of
// the wrong shape.
func Point(baseRadius, intersectionRadius float64) (x, y float64, ok bool) {
	if intersectionRadius < baseRadius {
		return 0, 0, false
	}
	alpha := math.Acos(baseRadius / intersectionRadius)
	t := math.Tan(alpha)
	x = baseRadius * (math.Cos(t) + t*math.Sin(t))
	y = baseRadius * (math.Sin(t) - t*math.Cos(t))
	return x, y, true
}

// Rotate turns (x, y) counter-clockwise by a radians.
func Rotate(x, y, a float64) (float64, float64) {
	sa, ca := math.Sin(a), math.Cos(a)
	return x*ca - y*sa, x*sa + y*ca
}

// Flanks returns the left and right flank sample points of one tooth, already
// placed at its final angular position.
//
// The samples run from the base circle out to the tip circle in equal radial
// steps, endpoint-inclusive, so the first sits exactly on the base circle and
// the last exactly on the tip circle. Samples inside the base circle are dropped.
//
// Three things happen to them, in this order, and the order is load-bearing.
// They are mirrored across +X first, because the standard parametric involute
// spirals so its angular position grows with radius, which as a left flank gives
// a tooth wider at the tip than at the root. They are then rotated so the pitch
// crossing lands at +pi/(2N), which centres the tooth on +X. Only then is the
// requested angle applied, to both flanks together, so the tooth stays
// symmetric about the angle direction rather than about +X.
func Flanks(baseR, tipR, pitchR, toothNumber float64, steps int, angle float64) (left, right []Pt) {
	mirrored := make([]Pt, 0, steps)
	for i := range steps {
		r := baseR + (tipR-baseR)*float64(i)/float64(steps-1)
		x, y, ok := Point(baseR, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, Pt{x, -y})
	}

	px, py, _ := Point(baseR, pitchR)
	rotateAngle := math.Pi/(2*toothNumber) - math.Atan2(-py, px)

	left = make([]Pt, 0, len(mirrored))
	right = make([]Pt, 0, len(mirrored))
	for _, p := range mirrored {
		lx, ly := Rotate(p.X, p.Y, rotateAngle)
		rx, ry := lx, -ly
		lx, ly = Rotate(lx, ly, angle)
		rx, ry = Rotate(rx, ry, angle)
		left = append(left, Pt{lx, ly})
		right = append(right, Pt{rx, ry})
	}
	return left, right
}

// Dimensions are the circle radii a spur-family gear is built from.
type Dimensions struct {
	Pitch float64
	Base  float64
	Root  float64
	Tip   float64
}

// Dimensions derives the four circle radii from module, tooth count and pressure
// angle. Dedendum is 1.25 modules and addendum is 1.
func Derive(module, toothNumber, pressureAngle float64) Dimensions {
	pitch := module * toothNumber / 2
	return Dimensions{
		Pitch: pitch,
		Base:  pitch * math.Cos(pressureAngle),
		Root:  (module*toothNumber - 2.5*module) / 2,
		Tip:   (module*toothNumber + 2*module) / 2,
	}
}

// Embedded reports whether the flank starts inside the root circle, which
// happens at low tooth counts and leaves no room for the flank-to-root lines.
//
// The first flank sample sits at exactly the base radius by construction, so
// comparing the two radii is the same test as measuring the drawn point. The
// comparison is strict: equality counts as not embedded and draws a zero-length
// stub, which is the ill-conditioned case the proof is meant to surface rather
// than paper over.
func (d Dimensions) Embedded() bool { return d.Base < d.Root }
