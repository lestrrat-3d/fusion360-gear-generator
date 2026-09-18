package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// measuredBacklash is the free play the default arrangement leaves, in mm. It
// is what a printed pair is judged by, and geometry_test.go measures the loft's
// section count against it.
const measuredBacklash = 0.28

// The sampling the contact search runs at. The station step has to resolve the
// tooth, whose flank rises 1.2 mm over less than a millimetre of station; 0.02 mm
// puts more than forty samples on a flank, far finer than the backlash the answer
// is quoted to.
const (
	stationStep  = 0.02
	edgeSamples  = 4
	faceSamples  = 6
	phaseSamples = 12
	phaseStep    = 200 // steps per pitch when the free window is scanned
)

// axialWindow is how far along its own axis a gear can still reach the other.
// A point of gear A at station s stands sqrt(offset^2 + (s*sin Sigma)^2) from
// gear B's axis, and nothing further than Width from that axis can touch a
// ribbon of that width, so the window closes where that distance reaches Width.
// The half added on is slack, not geometry.
func axialWindow(p Params) float64 {
	a := p.AxisOffset()
	if a >= p.Width {
		return p.Width
	}
	return 1.5 * math.Sqrt(p.Width*p.Width-a*a) / math.Sin(p.Sigma())
}

func withPhase(g Gear, z float64) Gear { g.Phase = z; return g }

// eachBoundarySample walks the ribbon's surface over the reaching window,
// handing each sample its world point and the cross-section place it came from.
func eachBoundarySample(g Gear, window float64, fn func(p r3.Vec, u, s float64)) {
	w, t := g.P.Width/2, g.P.Thickness/2
	for s := -window; s <= window; s += stationStep {
		e := g.edge(s)
		for i := 0; i <= edgeSamples; i++ {
			v := -t + 2*t*float64(i)/float64(edgeSamples)
			fn(g.world(e, v, s), e, s)
			fn(g.world(-w, v, s), -w, s)
		}
		for i := 0; i <= faceSamples; i++ {
			u := -w + (e+w)*float64(i)/float64(faceSamples)
			fn(g.world(u, t, s), u, s)
			fn(g.world(u, -t, s), u, s)
		}
	}
}

// hit is the deepest point one gear puts into the other: how deep, on which
// gear, and where on that gear's own ribbon.
type hit struct {
	depth float64 // >0 inside the other gear, <0 clear of it
	onA   bool
	u, s  float64
}

// deepest finds the worst point of either gear against the other at the given
// tooth phases.
func deepest(ga, gb Gear, za, zb float64) hit {
	a, b := withPhase(ga, za), withPhase(gb, zb)
	window := axialWindow(a.P)
	worst := hit{depth: math.Inf(-1)}
	eachBoundarySample(a, window, func(p r3.Vec, u, s float64) {
		if m := b.margin(p); m > worst.depth {
			worst = hit{m, true, u, s}
		}
	})
	eachBoundarySample(b, window, func(p r3.Vec, u, s float64) {
		if m := a.margin(p); m > worst.depth {
			worst = hit{m, false, u, s}
		}
	})
	return worst
}

// penetration is how deep the two gears lie in each other at the given tooth
// phases. It is negative when they are clear, so zero is contact.
func penetration(ga, gb Gear, za, zb float64) float64 { return deepest(ga, gb, za, zb).depth }

// freeWindow returns the interval of gear B's tooth phase that clears gear A,
// taken as the one containing seed or the nearest one to it.
func freeWindow(ga, gb Gear, za, seed float64) (lo, hi float64, ok bool) {
	p := ga.P.ToothPitch
	step := p / phaseStep
	clear := func(zb float64) bool { return penetration(ga, gb, za, zb) <= 0 }

	if !clear(seed) {
		found := false
		for k := 1; k <= phaseStep/2 && !found; k++ {
			for _, cand := range []float64{seed + float64(k)*step, seed - float64(k)*step} {
				if clear(cand) {
					seed, found = cand, true
					break
				}
			}
		}
		if !found {
			return 0, 0, false
		}
	}
	lo, hi = seed, seed
	for lo > seed-p && clear(lo-step) {
		lo -= step
	}
	for hi < seed+p && clear(hi+step) {
		hi += step
	}
	return lo, hi, true
}

// This is the proof the whole gear rests on. Three things have to hold across a
// full tooth cycle for the pair to be a gear rather than two parts that touch:
// the free window is never empty (no jam), it is narrower than a pitch (the
// teeth box B in, so something is driven), and it advances by exactly one pitch
// as A advances one pitch (the 1:1 ratio).
//
// A pair can satisfy the first two and still not be a gear: a window that
// widens and narrows but returns to where it started leaves B free to rattle
// and follow nothing. The winding is what separates those cases, and an earlier
// arrangement that passed the first two failed it.
func TestPairDrivesOneToOne(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	pitch := p.ToothPitch

	lo, hi, ok := freeWindow(ga, gb, 0, gb.Phase)
	if !ok {
		t.Fatal("the pair jams at the assembly phase: no phase of B clears A")
	}
	start := (lo + hi) / 2
	seed := start

	widest, tightest := hi-lo, hi-lo
	centres := make([]float64, 0, phaseSamples)
	for i := 1; i <= phaseSamples; i++ {
		za := float64(i) * pitch / phaseSamples
		lo, hi, ok := freeWindow(ga, gb, za, seed)
		if !ok {
			t.Fatalf("the pair jams: at tooth phase %.3f mm of A no phase of B clears it", za)
		}
		width := hi - lo
		if width >= pitch {
			t.Fatalf("at tooth phase %.3f mm of A every phase of B clears it over %.3f mm: "+
				"the teeth never box B in, so nothing is driven", za, width)
		}
		widest, tightest = math.Max(widest, width), math.Min(tightest, width)
		seed = (lo + hi) / 2
		centres = append(centres, seed)

		// Where the teeth actually touch, taken a hair outside the free window
		// so the pair is in contact rather than clear.
		touch := deepest(ga, gb, za, lo-pitch/phaseStep)
		side := "B"
		if touch.onA {
			side = "A"
		}
		t.Logf("phase %.3f: window [%.3f, %.3f] wide %.3f, contact on %s at u=%+.2f s=%+.2f",
			za, lo, hi, width, side, touch.u, touch.s)
	}

	winding := (seed - start) / pitch
	if math.Abs(winding-1) > 0.02 {
		t.Errorf("gear B advances %.4f pitches while A advances one: the ratio is not 1:1", winding)
	}

	var departure float64
	for i, c := range centres {
		want := start + (seed-start)*float64(i+1)/phaseSamples
		departure = math.Max(departure, math.Abs(c-want))
	}
	if departure > 0.08 {
		t.Errorf("B's phase departs from the 1:1 line by %.4f mm, want under 0.08", departure)
	}

	if widest > measuredBacklash+0.05 || tightest < measuredBacklash-0.15 {
		t.Errorf("the free window runs %.4f to %.4f mm wide, the spec quotes %.2f",
			tightest, widest, measuredBacklash)
	}
	t.Logf("winding %.4f pitches, window %.3f-%.3f mm wide, departure from 1:1 %.4f mm",
		winding, tightest, widest, departure)
}

// The phase gear B is built at has to sit in the play, not against a flank.
// A pair built at a phase outside the free window is a pair that has to be
// forced together, and the interference proof would be measuring a state the
// mechanism never reaches.
func TestAssemblyPhaseSitsInTheFreeWindow(t *testing.T) {
	ga, gb := defaultPair()
	lo, hi, ok := freeWindow(ga, gb, 0, assemblyPhase)
	if !ok {
		t.Fatal("no phase of B clears A at gear A's zero, so there is nothing to assemble at")
	}
	if assemblyPhase < lo || assemblyPhase > hi {
		t.Fatalf("the assembly phase %.3f is outside the free window [%.3f, %.3f]",
			assemblyPhase, lo, hi)
	}
	middle := (lo + hi) / 2
	if off := math.Abs(assemblyPhase - middle); off > (hi-lo)/4 {
		t.Errorf("the assembly phase %.3f sits %.3f mm off the middle %.3f of a %.3f mm window",
			assemblyPhase, off, middle, hi-lo)
	}
	t.Logf("assembly phase %.3f in a free window [%.3f, %.3f]", assemblyPhase, lo, hi)
}

// The arrangement that looks right jams, and this records it. Pointing both
// toothed edges straight at each other where the axes cross puts two or three
// tooth pairs in the engaged zone at once, and their ridges cross at an angle,
// so they cannot all interdigitate. The mounting angle exists for this reason,
// and a future simplification that drops it would be caught here.
func TestSymmetricMountJams(t *testing.T) {
	p := defaultParams()
	p.MountAngleA, p.MountAngleB = 0, 0
	ga, gb := pair(p, p.Sigma(), 0, p.ToothPitch/2)

	jammed := false
	for i := range phaseSamples {
		za := float64(i) * p.ToothPitch / phaseSamples
		free := 0
		for j := range phaseStep {
			zb := -p.ToothPitch/2 + p.ToothPitch*float64(j)/phaseStep
			if penetration(ga, gb, za, zb) <= 0 {
				free++
			}
		}
		if free == 0 {
			jammed = true
			t.Logf("at tooth phase %.3f mm of A, no phase of B clears it", za)
			break
		}
	}
	if !jammed {
		t.Error("the symmetric mounting cleared at every phase; the spec says it jams, " +
			"so either the spec's reason for the mounting angle is wrong or this model is")
	}
}

// The contact search only ever samples the window in which the two ribbons can
// reach each other, and everything the proof says rests on that window being
// the whole story. This walks BOTH ribbons end to end at their assembly phases
// and confirms nothing touches outside it — which is also what makes the
// pictures honest, since they draw the full 84 mm of each part.
func TestFullRibbonsClearOutsideTheEngagement(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	half := p.Length() / 2
	window := axialWindow(p)

	// The bound the window comes from: a point of one gear at station s stands
	// sqrt(offset^2 + (s sin Sigma)^2) from the other gear's axis, and material
	// further than Width from that axis cannot reach a ribbon of that width.
	limit := math.Sqrt(p.Width*p.Width-p.AxisOffset()*p.AxisOffset()) / math.Sin(p.Sigma())
	if window < limit {
		t.Fatalf("the sampled window is +/-%.2f mm but the two ribbons can reach to +/-%.2f mm",
			window, limit)
	}

	worst, worstAt := math.Inf(-1), 0.0
	check := func(from, into Gear) {
		for s := -half; s <= half; s += stationStep {
			e, w, th := from.edge(s), from.P.Width/2, from.P.Thickness/2
			for i := 0; i <= edgeSamples; i++ {
				v := -th + 2*th*float64(i)/float64(edgeSamples)
				for _, u := range []float64{e, -w} {
					if m := into.margin(from.world(u, v, s)); m > worst {
						worst, worstAt = m, s
					}
				}
			}
		}
	}
	check(ga, gb)
	check(gb, ga)

	if worst > 0 {
		t.Errorf("the two ribbons overlap by %.4f mm at station %.2f mm, where they should clear",
			worst, worstAt)
	}
	if math.Abs(worstAt) > window {
		t.Errorf("the closest approach is at station %.2f mm, outside the +/-%.2f mm the contact "+
			"search samples", worstAt, window)
	}
	t.Logf("closest approach %.4f mm at station %.2f mm, window +/-%.2f mm", -worst, worstAt, window)
}

// The engaged zone is what the mounting angle is working around, and its size
// is what decides how many tooth pairs have to interdigitate at once. This
// measures it rather than reasoning about it.
func TestEngagedZoneSpansSeveralTeeth(t *testing.T) {
	p := defaultParams()
	window := axialWindow(p)
	pairs := 2 * window / 1.5 / p.ToothPitch // undo the slack the window carries

	if pairs < 2 || pairs > 6 {
		t.Errorf("the engaged zone holds %.1f tooth pairs; the mounting angle is what lets several "+
			"interdigitate at once, and past six that is not worth trusting", pairs)
	}
	t.Logf("reaching window +/-%.2f mm, about %.1f tooth pairs engaged", window, pairs)
}
