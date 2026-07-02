// Sketch-first proof for the helical gear's TWISTED Gear Profile — the second
// (top) loft profile that helical draws with the spur tooth generator at
// angle=helixAngle. It reproduces the SPUR-F constraint scheme (from
// spec/spurgear/fusion.md) with the [SPUR-F-SPINE] *angle != 0* path — the
// horizontal reference line + far-end pin + angular dimension — which the spur
// bench (angle == 0) never exercised, and proves it FULLY CONSTRAINS (DOF==0, no
// redundant/conflicting constraints, well-conditioned) BEFORE any Fusion code is
// generated ([PB-SKETCH-FIRST]). It runs across sizes AND helix angles.
//
//	go run .
package main

import (
	"fmt"
	"math"
	"os"

	"github.com/lestrrat-3d/sketch"
)

// calculateInvolutePoint mirrors the spec's exact math. ok=false when
// intersectionRadius < baseRadius (sample inside the base circle — dropped).
func calculateInvolutePoint(baseRadius, intersectionRadius float64) (x, y float64, ok bool) {
	if intersectionRadius < baseRadius {
		return 0, 0, false
	}
	alpha := math.Acos(baseRadius / intersectionRadius)
	t := math.Tan(alpha)
	x = baseRadius * (math.Cos(t) + t*math.Sin(t))
	y = baseRadius * (math.Sin(t) - t*math.Cos(t))
	return x, y, true
}

func rot(x, y, a float64) (float64, float64) {
	return x*math.Cos(a) - y*math.Sin(a), x*math.Sin(a) + y*math.Cos(a)
}

type pt struct{ x, y float64 }

func dist(a, b pt) float64 { return math.Hypot(a.x-b.x, a.y-b.y) }

// checkTwistedProfile builds the helical twisted Gear Profile sketch (the tooth
// drawn at `helixAngle`) and returns whether it PASSES the primary
// full-constraint gate. helixAngle is in radians; helixAngle==0 reduces to the
// spur (untwisted) profile.
func checkTwistedProfile(module, toothNumber, pressureAng, helixAngle float64, involSteps int) bool {
	angle := helixAngle // the whole tooth is drawn pre-rotated by this (spur step 4)
	pitchR := module * toothNumber / 2
	baseR := pitchR * math.Cos(pressureAng)
	rootR := (module*toothNumber - 2.5*module) / 2
	tipR := (module*toothNumber + 2*module) / 2
	embedded := baseR < rootR
	fmt.Printf("\n=== M=%.2f N=%.0f PA=%.1f° helix=%.1f°  base=%.3f root=%.3f tip=%.3f embedded=%v ===\n",
		module, toothNumber, pressureAng*180/math.Pi, helixAngle*180/math.Pi, baseR, rootR, tipR, embedded)
	if embedded {
		fmt.Println("(embedded 4-curve variant not modelled — helical requires non-embedded; see README)")
		return true
	}

	// sample the involute flank, mirror across +X, rotate the pitch crossing to
	// +pi/(2N), then apply the requested `angle` (the helix twist).
	var mirrored []pt
	for i := 0; i < involSteps; i++ {
		r := baseR + (tipR-baseR)*float64(i)/float64(involSteps-1)
		x, y, ok := calculateInvolutePoint(baseR, r)
		if !ok {
			continue
		}
		mirrored = append(mirrored, pt{x, -y})
	}
	px, py, _ := calculateInvolutePoint(baseR, pitchR)
	rotateAngle := math.Pi/(2*toothNumber) - math.Atan2(-py, px)
	var left, right []pt
	for _, p := range mirrored {
		lx, ly := rot(p.x, p.y, rotateAngle)
		lx, ly = rot(lx, ly, angle)
		left = append(left, pt{lx, ly})
		right = append(right, pt{lx, -ly})
	}

	w := sketch.NewWorld()
	s, _ := w.CreateSketch(w.XY())

	origin := s.CreatePoint(0, 0)
	origin.MoveTo(0, 0)
	s.Fix(origin) // anchor coincidence (spec step 5)

	mkCircle := func(r float64) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(true)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	rootCircle := mkCircle(rootR)
	tipCircle := mkCircle(tipR)
	mkCircle(baseR)
	mkCircle(pitchR)

	leftPts := make([]*sketch.Point, len(left))
	rightPts := make([]*sketch.Point, len(right))
	for i := range left {
		leftPts[i] = s.CreatePoint(left[i].x, left[i].y)
		rightPts[i] = s.CreatePoint(right[i].x, right[i].y)
	}
	if _, err := s.CreateSpline(leftPts...); err != nil {
		fmt.Println("left spline:", err)
		return false
	}
	if _, err := s.CreateSpline(rightPts...); err != nil {
		fmt.Println("right spline:", err)
		return false
	}

	// spine: origin -> tooth-top (pre-rotated to `angle`). [SPUR-F-SPINE].
	ttx, tty := rot(tipR, 0, angle)
	toothTop := s.CreatePoint(ttx, tty)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	if angle == 0 {
		// angle==0 (spur) path: the spine's only extra constraint is horizontal.
		s.AddConstraint(sketch.NewHorizontal(spine))
	} else {
		// angle!=0 (helical/herringbone/bevel) path: a +X horizontal REFERENCE line
		// whose far end is PINNED on the tip circle (else its length floats and the
		// sketch is under-constrained — the [SPUR-F-SPINE] end-pin), then an angular
		// dimension from the reference to the spine fixes the twist.
		refEnd := s.CreatePoint(tipR, 0)
		refLine := s.CreateLine(origin, refEnd)
		refLine.SetConstruction(true)
		s.AddConstraint(sketch.NewHorizontal(refLine))
		s.AddConstraint(sketch.NewPointOnCircle(refEnd, tipCircle)) // the end-pin
		// CCW from the +X reference to the spine == helixAngle (NewAngle is in degrees).
		s.AddConstraint(sketch.NewAngle(refLine, spine, helixAngle*180/math.Pi))
	}

	// tooth-top arc (free center + diameter dim, faithful to Fusion's 3-point arc).
	topArc := s.CreateArc(s.CreatePoint(rot(0, tipR*0.1, angle)), rightPts[len(rightPts)-1], leftPts[len(leftPts)-1])
	s.AddConstraint(sketch.NewDiameter(topArc, 2*tipR)) // [SPUR-F-TOOTHTOP-ARC]

	// ribs: lock each flank pair to the spine. Exact [SPUR-F-RIBS] order.
	prevMid := origin
	prevMidPt := pt{0, 0}
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		s.AddConstraint(sketch.NewDistance(leftPts[i], rightPts[i], dist(left[i], right[i])))
		t := left[i].x*math.Cos(angle) + left[i].y*math.Sin(angle) // foot on the spine
		mx, my := t*math.Cos(angle), t*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		s.AddConstraint(sketch.NewDistance(prevMid, mid, dist(prevMidPt, pt{mx, my})))
		prevMid, prevMidPt = mid, pt{mx, my}
	}

	// flank-to-root lines ([SPUR-F-FLANK-ROOT], non-embedded).
	addFlankRoot := func(flankStart *sketch.Point, seed pt) *sketch.Point {
		n := math.Hypot(seed.x, seed.y)
		re := s.CreatePoint(rootR*seed.x/n, rootR*seed.y/n)
		line := s.CreateLine(re, flankStart)
		s.AddConstraint(sketch.NewPointOnCircle(re, rootCircle))
		s.AddConstraint(sketch.NewPointOnLine(origin, line))
		return re
	}
	leftFoot := addFlankRoot(leftPts[0], left[0])
	rightFoot := addFlankRoot(rightPts[0], right[0])

	// explicit root arc completing the 6-curve loop.
	rootArc := s.CreateArc(s.CreatePoint(rot(0, -rootR*0.1, angle)), rightFoot, leftFoot)
	s.AddConstraint(sketch.NewDiameter(rootArc, 2*rootR))

	res, err := s.Solve()
	if err != nil {
		fmt.Println("solve error:", err)
	}
	rep := s.Verify(sketch.WithProbe())
	fmt.Printf("Solve: converged=%v DOF=%d redundant=%d residual=%.1e | Verify: status=%s conditioning=%.2e\n",
		res.Converged, res.DOF, res.Redundant, res.Residual, rep.Status, rep.Conditioning)

	condGate := math.Max(1e-6, 4*math.Sqrt(1e-10))
	primary := rep.Status == sketch.FullyConstrained && rep.Conditioning >= condGate
	fmt.Printf("  PRIMARY GATE (full constraint) = %v\n", primary)
	fmt.Printf("  advisory: profilesValid=%v (true with the engine's #12 corner-join fix) probeAmbiguous=%v (expected — seeded)\n",
		rep.ProfilesValid, rep.Probe != nil && rep.Probe.Ambiguous())
	return primary
}

func main() {
	const pa = 20.0 * math.Pi / 180.0
	deg := func(d float64) float64 { return d * math.Pi / 180.0 }

	allPass := true
	check := func(m, n, helix float64) {
		if !checkTwistedProfile(m, n, pa, helix, 15) {
			allPass = false
		}
	}

	// The helical default helix angle (14.5°) across non-embedded sizes, to show
	// the angle!=0 twisted profile fully constrains parametrically.
	for _, c := range []struct{ m, n float64 }{{1, 12}, {1, 17}, {2, 20}, {3, 15}} {
		check(c.m, c.n, deg(14.5))
	}
	// A sweep of helix angles at N=17 to show the angle!=0 path is robust to the
	// twist magnitude; plus angle==0 as the spur-equivalent sanity baseline.
	for _, h := range []float64{0, 10, 25, 35} {
		check(1, 17, deg(h))
	}

	fmt.Println()
	if allPass {
		fmt.Println("ALL PASS — the helical twisted Gear Profile fully constrains across sizes and helix angles.")
		fmt.Println("Cleared to generate Fusion add-in code.")
	} else {
		fmt.Println("FAIL — scheme does not fully constrain; fix the scheme before generating Fusion code.")
		os.Exit(1)
	}
}
