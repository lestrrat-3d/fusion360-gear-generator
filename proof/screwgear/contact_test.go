package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// ---------------------------------------------------------------------------
// Where the teeth touch, as a set rather than a yes or no.
//
// TestPairDrivesOneToOne asks whether the two ribbons clear each other, and a
// pair can clear, box B in and drive 1:1 while its teeth touch at a single
// point. The third sleeve, printed with the roof allowance and reported on
// 2026-10-03, did: its bores fitted, and the teeth slipped. The user
// saw that "they meet at a single point rather than mating at the tooth
// surface" (spec/screwgear/fusion.md [SCREW-F-PRINT-3]). Each ribbon's ridges ran
// straight across its thickness, square to its own axis, and at the 80 degree
// crossing the two ribbons' ridges stood 74 to 80 degrees apart where they
// touched: a corner of one tooth dug into the other's flank, less than 0.13 mm
// of each 3.75 mm ridge came near the other ribbon, and the flank normals stood
// 107 to 118 degrees apart instead of facing each other.
//
// For the teeth to touch along a line, the two ridges have to lie along each
// other where they meet. Each ridge leans by ToothSlant in its own (v, s) chart
// and is straightened by ToothBow, and at the defaults both run along the
// bisector of the obtuse angle between the two axes, 50 degrees from each
// ribbon's own axis. touchAlong measures what that buys.
//
// Could the proof have caught the third print? Yes. Every quantity here was in
// the model the day the straight tooth was printed; nothing measured the
// contact's extent, only its existence. TestStraightRidgesTouchAtAPoint holds
// that this check fails that tooth.
// ---------------------------------------------------------------------------

// The bounds the contact is held to. At the defaults' nominal pose at least
// contactShare of the touching ridge has to lie within contactNear of the
// other ribbon's flank, the two ridges within contactRidgeAngle of parallel,
// and the two flank normals within contactNormalAngle of opposite. Under the
// bore play, with the tips short, at least contactPlayLength of the ridge has
// to lie within contactPlayNear.
const (
	contactNear        = 0.05
	contactShare       = 0.80
	contactRidgeAngle  = 5.0
	contactNormalAngle = 10.0
	contactPlayNear    = 0.10
	contactPlayLength  = 1.2
)

// The sampling touchAlong runs at: the station step and the steps across the
// thickness the touching sample is searched at, the steps along the ridge, and
// the station step and reach each ridge step slides over.
const (
	touchStationStep = 0.01
	touchEdgeSteps   = 12
	ridgeSteps       = 30
	ridgeSlideStep   = 0.002
	ridgeSlideReach  = 1.4
)

// frame is the section's own unit vectors at station s: u, v and the axis.
func (g Gear) frame(s float64) (uHat, vHat, ez r3.Vec) {
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	return g.Ex.Scale(c).Add(g.Ey.Scale(sn)), g.Ex.Scale(-sn).Add(g.Ey.Scale(c)), g.Ez
}

// surf is the toothed flank at chart (v, s), in the world.
func (g Gear) surf(v, s float64) r3.Vec { return g.world(g.edgeAt(v, s), v, s) }

// flankNormal is the toothed surface's outward unit normal at (v, s), by
// central differences.
func (g Gear) flankNormal(v, s float64) r3.Vec {
	const h = 1e-4
	dv := g.surf(v+h, s).Sub(g.surf(v-h, s))
	ds := g.surf(v, s+h).Sub(g.surf(v, s-h))
	n, _ := dv.Cross(ds).Normalize()
	return n
}

// ridgeDir is the unit direction along a tooth ridge, a line of constant
// s + Slant*v in the chart, at (v, s).
func (g Gear) ridgeDir(v, s float64) r3.Vec {
	const h = 1e-4
	d, _ := g.surf(v+h, s-g.Slant*h).Sub(g.surf(v-h, s+g.Slant*h)).Normalize()
	return d
}

// ridgeStretch is the world length of a ridge per millimetre of v at (v, s).
func (g Gear) ridgeStretch(v, s float64) float64 {
	const h = 1e-4
	return g.surf(v+h, s-g.Slant*h).Sub(g.surf(v-h, s+g.Slant*h)).Len() / (2 * h)
}

// flankDepth is how far a world point lies inside g's toothed flank, measured
// along the flank's normal to first order: positive inside, negative clear. A
// point past either face is not against the flank, and is taken as clear by
// any distance. It also returns the point's (v, s) in g.
func (g Gear) flankDepth(pt r3.Vec) (float64, float64, float64) {
	u, v, s := g.local(pt)
	if math.Abs(v) > g.P.Thickness/2 {
		return math.Inf(-1), v, s
	}
	uHat, _, _ := g.frame(s)
	return (g.edgeAt(v, s) - u) * g.flankNormal(v, s).Dot(uHat), v, s
}

// flankSample is one sample of a toothed flank: its chart place and the world
// point, and how deep it lies in the other ribbon's flank.
type flankSample struct {
	depth, v, s float64
	pt          r3.Vec
}

// deepestFlank finds the sample of from's toothed flank deepest in into's
// flank, over the window in which the two can reach each other.
func deepestFlank(from, into Gear) flankSample {
	t := from.P.Thickness / 2
	window := axialWindow(from.P)
	best := flankSample{depth: math.Inf(-1)}
	for s := -window; s <= window; s += touchStationStep {
		for i := 0; i <= touchEdgeSteps; i++ {
			v := -t + 2*t*float64(i)/touchEdgeSteps
			pt := from.surf(v, s)
			if d, _, _ := into.flankDepth(pt); d > best.depth {
				best = flankSample{d, v, s, pt}
			}
		}
	}
	return best
}

// ridgeContact is how one tooth touches the other ribbon at one pose.
type ridgeContact struct {
	onA         bool    // the touching ridge is gear A's
	ridge       float64 // its world length across the thickness
	near        float64 // the stretch of it within contactNear of the other flank
	playNear    float64 // the stretch of it within contactPlayNear
	ridgeAngle  float64 // degrees between the two ridges where they touch
	normalAngle float64 // degrees between the two flank normals there; 180 is face to face
}

// touchAlong measures where the teeth touch with A at phase za and B at zb, a
// pose a hair past the free window. It takes the flank sample of either gear
// deepest in the other, which is on the touching tooth, and the foot of that
// sample on the other flank. It reads the angle between the two ridges and
// between the two flank normals there. Then it walks the touching tooth's ridge
// across the thickness, ridgeSteps steps, and at each step slides along the
// station within ridgeSlideReach of the ridge for the least gap to the other
// flank; the stretch of ridge whose gap stays under a bound from one step to
// the next is the contact's length at that bound.
func touchAlong(ga, gb Gear, za, zb float64) ridgeContact {
	a, b := withPhase(ga, za), withPhase(gb, zb)
	from, into, h, onA := a, b, deepestFlank(a, b), true
	if hb := deepestFlank(b, a); hb.depth > h.depth {
		from, into, h, onA = b, a, hb, false
	}
	_, vi, si := into.flankDepth(h.pt)
	c := ridgeContact{onA: onA}
	c.ridgeAngle = math.Acos(math.Min(1, math.Abs(from.ridgeDir(h.v, h.s).Dot(into.ridgeDir(vi, si))))) * 180 / math.Pi
	c.normalAngle = math.Acos(math.Max(-1, math.Min(1,
		from.flankNormal(h.v, h.s).Dot(into.flankNormal(vi, si))))) * 180 / math.Pi

	// The touching ridge is the line s + Slant*v = s0 through the sample.
	t := from.P.Thickness / 2
	s0 := h.s + from.Slant*h.v
	stretch := from.ridgeStretch(0, s0)
	dv := 2 * t / ridgeSteps
	c.ridge = 2 * t * stretch
	var prev float64
	for i := 0; i <= ridgeSteps; i++ {
		v := -t + float64(i)*dv
		gap := math.Inf(1)
		for ds := -ridgeSlideReach; ds <= ridgeSlideReach; ds += ridgeSlideStep {
			if d, _, _ := into.flankDepth(from.surf(v, s0-from.Slant*v+ds)); -d < gap {
				gap = -d
			}
		}
		if i > 0 {
			if gap < contactNear && prev < contactNear {
				c.near += dv * stretch
			}
			if gap < contactPlayNear && prev < contactPlayNear {
				c.playNear += dv * stretch
			}
		}
		prev = gap
	}
	return c
}

// lineContact reports why a contact is not a line contact by the nominal
// bounds, or "" when it is.
func (c ridgeContact) lineContact() string {
	switch {
	case c.near < contactShare*c.ridge:
		return "too little of the ridge is near the other flank"
	case c.ridgeAngle > contactRidgeAngle:
		return "the ridges cross"
	case c.normalAngle < 180-contactNormalAngle:
		return "the flanks do not face each other"
	}
	return ""
}

// The teeth touch along a line, not at a point. At each of the twelve phases
// of A that TestPairDrivesOneToOne tracks the free window at, B is put a hair
// past either end of the window, against either flank of A, and touchAlong
// measures the contact: at least 80% of the touching ridge within 0.05 mm of
// the other flank, the two ridges within 5 degrees of parallel and the flank
// normals within 10 degrees of opposite, at all 24 poses.
func TestTeethTouchAlongALine(t *testing.T) {
	t.Parallel()
	ga, gb := defaultPair()
	step := ga.P.ToothPitch / phaseStep

	track := defaultTrack()
	if track.failure != "" {
		t.Fatal(track.failure)
	}
	least, leastShare := math.Inf(1), math.Inf(1)
	worstRidge, worstNormal := 0.0, 180.0
	for _, w := range track.windows[1:] {
		for _, zb := range []float64{w.lo - step, w.hi + step} {
			c := touchAlong(ga, gb, w.za, zb)
			least, leastShare = math.Min(least, c.near), math.Min(leastShare, c.near/c.ridge)
			worstRidge, worstNormal = math.Max(worstRidge, c.ridgeAngle), math.Min(worstNormal, c.normalAngle)
			if why := c.lineContact(); why != "" {
				t.Errorf("A at %.3f, B at %.3f: %s: %.2f of %.2f mm of ridge within %.2f mm, ridges %.1f "+
					"degrees apart, normals %.1f degrees apart", w.za, zb, why, c.near, c.ridge, contactNear,
					c.ridgeAngle, c.normalAngle)
			}
		}
	}
	t.Logf("at %d poses the touching ridge lies within %.2f mm of the other flank over %.2f mm at the "+
		"least, %.0f%% of it; the ridges stand %.1f degrees apart at the most and the normals %.1f at the least",
		2*phaseSamples, contactNear, least, 100*leastShare, worstRidge, worstNormal)
}

// The check above has to fail the tooth the third sleeve was printed with, or
// it is not the check that print asked for. At thirdPrintParams, the straight
// ridge at 14 degrees on both gears, the pair drives at A's phase 0, and at
// both of its driving poses the teeth touch at a point.
func TestStraightRidgesTouchAtAPoint(t *testing.T) {
	t.Parallel()
	p := thirdPrintParams()
	ga, gb := pair(p, p.Sigma(), 0, thirdPrintPhase)
	lo, hi, ok := freeWindow(ga, gb, 0, gb.Phase)
	if !ok {
		t.Fatal("the third print's pair jams at A's phase 0, so it has no driving pose to measure")
	}
	step := p.ToothPitch / phaseStep
	for _, zb := range []float64{lo - step, hi + step} {
		c := touchAlong(ga, gb, 0, zb)
		t.Logf("B at %.3f: %.2f of %.2f mm of ridge within %.2f mm, %.2f within %.2f mm, ridges %.1f degrees "+
			"apart, normals %.1f degrees apart", zb, c.near, c.ridge, contactNear, c.playNear, contactPlayNear,
			c.ridgeAngle, c.normalAngle)
		if c.lineContact() == "" {
			t.Errorf("B at %.3f: the straight ridges pass the line-contact check", zb)
		}
		if c.playNear >= contactPlayLength {
			t.Errorf("B at %.3f: the straight ridges pass the bore play's looser bound, %.2f mm within %.2f mm",
				zb, c.playNear, contactPlayNear)
		}
	}
}
