package screwgear_test

import (
	"fmt"
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// measuredBacklash is the free play the default arrangement leaves, in mm. It
// is what a printed pair is judged by, and geometry_test.go measures the loft's
// section count against it. It was 0.46 mm at the straight tooth, 14 degrees
// and a 0.90 mm engagement until 2026-10-03.
const measuredBacklash = 1.00

// The bounds the mesh is held to are fractions of the pitch, not lengths. The
// model is an exact cosine on an exact helicoid, so a pair scaled by k has its
// free window and its departure from the 1:1 line scaled by k too, and the
// search of spec/screwgear/mesh-search.md found the departure tracking the
// pitch at every pitch it tried. A bound in millimetres would pass or fail a
// scaled gear on its size alone. maxDeparture is the largest gap between the
// window's middle and the 1:1 line, as a fraction of the pitch: 0.10 mm at the
// earlier 1.75 mm pitch, 0.1575 mm at the 2.625 mm default. backlashTolerance
// is how far the window may run from measuredBacklash before the spec's
// quoted number is stale, also as a fraction of the pitch.
const (
	maxDeparture      = 0.06
	backlashTolerance = 0.07
)

// The sampling the contact search runs at, which the spec quotes beside every
// number taken from it (spec/screwgear/instructions.md "Defaults"). The station
// step has to resolve the tooth, whose flank rises 2.6 mm over about a
// millimetre of station; 0.01 mm puts more than a hundred samples on a flank,
// far finer than the backlash the answer is quoted to. edgeSamples and faceSamples
// are steps, so an edge carries edgeSamples+1 points across the thickness and a
// face faceSamples+1 across the width; phaseSamples is the number of phases of A
// per pitch, and phaseStep the number of steps of B's phase per pitch when the
// free window is scanned, so a window's ends are quoted to that step.
const (
	stationStep  = 0.01
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
		for i := 0; i <= edgeSamples; i++ {
			v := -t + 2*t*float64(i)/float64(edgeSamples)
			e := g.edgeAt(v, s)
			fn(g.world(e, v, s), e, s)
			fn(g.world(-w, v, s), -w, s)
		}
		eHi, eLo := g.edgeAt(t, s), g.edgeAt(-t, s)
		for i := 0; i <= faceSamples; i++ {
			k := float64(i) / float64(faceSamples)
			uHi, uLo := -w+(eHi+w)*k, -w+(eLo+w)*k
			fn(g.world(uHi, t, s), uHi, s)
			fn(g.world(uLo, -t, s), uLo, s)
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

// clearAt answers penetration(ga, gb, za, zb) <= 0 over the same samples, and
// stops at the first sample inside the other gear. The free-window scans ask
// only this, and most of the phases they try off the window are blocked within
// a few stations.
func clearAt(ga, gb Gear, za, zb float64) bool {
	a, b := withPhase(ga, za), withPhase(gb, zb)
	window := axialWindow(a.P)
	return !reachesInto(a, b, window) && !reachesInto(b, a, window)
}

// reachesInto answers whether any of eachBoundarySample's samples of from lies
// inside into, in the same order eachBoundarySample walks them.
func reachesInto(from, into Gear, window float64) bool {
	w, t := from.P.Width/2, from.P.Thickness/2
	for s := -window; s <= window; s += stationStep {
		for i := 0; i <= edgeSamples; i++ {
			v := -t + 2*t*float64(i)/float64(edgeSamples)
			if into.margin(from.world(from.edgeAt(v, s), v, s)) > 0 || into.margin(from.world(-w, v, s)) > 0 {
				return true
			}
		}
		eHi, eLo := from.edgeAt(t, s), from.edgeAt(-t, s)
		for i := 0; i <= faceSamples; i++ {
			k := float64(i) / float64(faceSamples)
			if into.margin(from.world(-w+(eHi+w)*k, t, s)) > 0 || into.margin(from.world(-w+(eLo+w)*k, -t, s)) > 0 {
				return true
			}
		}
	}
	return false
}

// freeWindow returns the interval of gear B's tooth phase that clears gear A,
// taken as the one containing seed or the nearest one to it, with its ends on
// the grid of P/phaseStep steps from the seed and at most a pitch from it.
//
// Each end is found by doubling the step count outward from the seed while
// the pair stays clear and then bisecting between the last clear count and
// the first blocked one. That is the end a walk outward one step at a time
// finds whenever the phases that clear form one interval, which they do: a
// blocked phase between two clear ones would be a jam B passes through. It
// asks clearAt about 14 times an end rather than once a step, and the windows
// here run to 80 steps and more.
func freeWindow(ga, gb Gear, za, seed float64) (lo, hi float64, ok bool) {
	p := ga.P.ToothPitch
	step := p / phaseStep
	clear := func(zb float64) bool { return clearAt(ga, gb, za, zb) }

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
	return lastClear(clear, seed, -step), lastClear(clear, seed, step), true
}

// lastClear is the last of seed + k*step, for k from 0 to phaseStep, before
// the first that is not clear, given that seed is clear.
func lastClear(clear func(float64) bool, seed, step float64) float64 {
	good, bad := 0, -1
	for k := 1; ; k *= 2 {
		k = min(k, phaseStep)
		if !clear(seed + float64(k)*step) {
			bad = k
			break
		}
		good = k
		if k == phaseStep {
			return seed + float64(good)*step
		}
	}
	for bad-good > 1 {
		mid := (good + bad) / 2
		if clear(seed + float64(mid)*step) {
			good = mid
		} else {
			bad = mid
		}
	}
	return seed + float64(good)*step
}

// phaseWindow is the free window of B's tooth phase, [lo, hi], at A's tooth
// phase za.
type phaseWindow struct{ za, lo, hi float64 }

// windowTrack is the free window followed through one pitch of A: the window
// at A's phase 0, found from B's own phase, and then one at each of
// phaseSamples equal steps of A, each found from the middle of the one before.
// failure is why the walk stopped, or "" when it went the whole pitch.
type windowTrack struct {
	windows []phaseWindow
	failure string
}

func trackWindows(ga, gb Gear) windowTrack {
	pitch := ga.P.ToothPitch
	lo, hi, ok := freeWindow(ga, gb, 0, gb.Phase)
	if !ok {
		return windowTrack{failure: "the pair jams at the assembly phase: no phase of B clears A"}
	}
	r := windowTrack{windows: []phaseWindow{{0, lo, hi}}}
	seed := (lo + hi) / 2
	for i := 1; i <= phaseSamples; i++ {
		za := float64(i) * pitch / phaseSamples
		lo, hi, ok := freeWindow(ga, gb, za, seed)
		if !ok {
			r.failure = fmt.Sprintf("the pair jams: at tooth phase %.3f mm of A no phase of B clears it", za)
			return r
		}
		r.windows = append(r.windows, phaseWindow{za, lo, hi})
		if hi-lo >= pitch {
			r.failure = fmt.Sprintf("at tooth phase %.3f mm of A every phase of B clears it over %.3f mm: "+
				"the teeth never box B in, so nothing is driven", za, hi-lo)
			return r
		}
		seed = (lo + hi) / 2
	}
	return r
}

// defaultTrack is trackWindows at the defaults, found once: TestPairDrivesOneToOne
// judges the windows and TestTeethTouchAlongALine judges the contact at them.
var defaultTrack = sync.OnceValue(func() windowTrack { return trackWindows(defaultPair()) })

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
	t.Parallel()
	ga, gb := defaultPair()
	pitch := ga.P.ToothPitch

	track := defaultTrack()
	if track.failure != "" {
		t.Fatal(track.failure)
	}
	first := track.windows[0]
	start := (first.lo + first.hi) / 2
	widest, tightest := first.hi-first.lo, first.hi-first.lo
	centres := make([]float64, 0, phaseSamples)
	for _, w := range track.windows[1:] {
		width := w.hi - w.lo
		widest, tightest = math.Max(widest, width), math.Min(tightest, width)
		centres = append(centres, (w.lo+w.hi)/2)

		// Where the teeth actually touch, taken a hair outside the free window
		// so the pair is in contact rather than clear.
		touch := deepest(ga, gb, w.za, w.lo-pitch/phaseStep)
		side := "B"
		if touch.onA {
			side = "A"
		}
		t.Logf("phase %.3f: window [%.3f, %.3f] wide %.3f, contact on %s at u=%+.2f s=%+.2f",
			w.za, w.lo, w.hi, width, side, touch.u, touch.s)
	}
	end := centres[len(centres)-1]

	winding := (end - start) / pitch
	if math.Abs(winding-1) > 0.02 {
		t.Errorf("gear B advances %.4f pitches while A advances one: the ratio is not 1:1", winding)
	}

	var departure float64
	for i, c := range centres {
		want := start + (end-start)*float64(i+1)/phaseSamples
		departure = math.Max(departure, math.Abs(c-want))
	}
	if departure > maxDeparture*pitch {
		t.Errorf("B's phase departs from the 1:1 line by %.4f mm, %.1f%% of the pitch, want under %.0f%%",
			departure, 100*departure/pitch, 100*maxDeparture)
	}

	if tol := backlashTolerance * pitch; widest > measuredBacklash+tol || tightest < measuredBacklash-tol {
		t.Errorf("the free window runs %.4f to %.4f mm wide, the spec quotes %.2f",
			tightest, widest, measuredBacklash)
	}
	t.Logf("winding %.4f pitches, window %.3f-%.3f mm wide, departure from 1:1 %.4f mm, %.1f%% of the pitch",
		winding, tightest, widest, departure, 100*departure/pitch)
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

// The defaults point both toothed edges straight at each other where the axes
// cross, both mounting angles zero, and that arrangement drives only because
// the ridges lean. With the straight ridge every print before 2026-10-03
// carried it jams: the engaged zone holds two or three tooth pairs at once,
// and ridges that cross at an angle cannot all interdigitate. The mounting
// angles of 14 degrees existed to move the contact off the crossing for that
// tooth (mesh-search.md). With the ridges leaned to lie along each other where
// they meet, the tooth pairs in the engaged zone interdigitate at zero, which
// TestPairDrivesOneToOne measures at the defaults; this holds that the
// defaults are that arrangement and that the straight tooth still jams there,
// so a change that drops the lean while keeping the angles is caught.
func TestSymmetricMountNeedsTheLeanedRidge(t *testing.T) {
	t.Parallel()
	d := defaultParams()
	if d.MountAngleA != 0 || d.MountAngleB != 0 {
		t.Errorf("the default mounting angles are %.1f and %.1f degrees, want 0 and 0",
			d.MountAngleA*180/math.Pi, d.MountAngleB*180/math.Pi)
	}
	if track := defaultTrack(); track.failure != "" {
		t.Errorf("with the leaned ridge at zero mounting angles: %s", track.failure)
	}

	p := thirdPrintParams()
	p.MountAngleA, p.MountAngleB = 0, 0
	ga, gb := pair(p, p.Sigma(), 0, p.ToothPitch/2)
	jammed := false
	for i := range phaseSamples {
		za := float64(i) * p.ToothPitch / phaseSamples
		free := 0
		for j := range phaseStep {
			zb := -p.ToothPitch/2 + p.ToothPitch*float64(j)/phaseStep
			if clearAt(ga, gb, za, zb) {
				free++
			}
		}
		if free == 0 {
			jammed = true
			t.Logf("with the straight ridge at zero mounting angles, at tooth phase %.3f mm of A no phase of B "+
				"clears it", za)
			break
		}
	}
	if !jammed {
		t.Error("the straight ridge at zero mounting angles cleared at every phase; the spec says it jams, " +
			"so either the spec's reason for the lean is wrong or this model is")
	}
}

// The contact search only ever samples the window in which the two ribbons can
// reach each other, and everything the proof says rests on that window being
// the whole story. This walks BOTH ribbons end to end at their assembly phases
// and confirms nothing touches outside it — which is also what makes the
// pictures honest, since they draw the full 178.5 mm of each part.
//
// The closest approach it logs is the spec's slack at the assembly phases: the least slack of any
// sample of one ribbon's toothed or back edge against the other ribbon, at the
// assembly phases with nothing moved. The slack is Gear.margin, taken in the
// other ribbon's own section along u or v, so it is a slack rather than a
// Euclidean distance. At the defaults it lands in the mesh, since gear B sits
// in the middle of its free window and has about half of it each way.
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
			w, th := from.P.Width/2, from.P.Thickness/2
			for i := 0; i <= edgeSamples; i++ {
				v := -th + 2*th*float64(i)/float64(edgeSamples)
				for _, u := range []float64{from.edgeAt(v, s), -w} {
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
