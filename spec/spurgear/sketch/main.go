// Prototype of the spur gear "Gear Profile" sketch in the lestrrat-3d/sketch
// engine. It reproduces the SPUR-F constraint scheme from spec/spurgear/fusion.md
// and proves it FULLY CONSTRAINS (DOF==0, no redundant/conflicting constraints,
// well-conditioned) BEFORE any Fusion add-in code is generated — the sketch-first
// gate ([PB-SKETCH-FIRST]). It runs the check across several tooth counts and angles
// to show the parametric scheme holds as Module / Tooth Number / angle change.
//
//	go run .
package main

import (
	"context"
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

// checkGearProfile builds the spur Gear Profile sketch for the given parameters
// and returns whether it PASSES the primary full-constraint gate.
func checkGearProfile(ctx context.Context, module, toothNumber, pressureAng, angle float64, involSteps int, toothTopArcCentreFree bool) bool {
	pitchR := module * toothNumber / 2
	baseR := pitchR * math.Cos(pressureAng)
	rootR := (module*toothNumber - 2.5*module) / 2
	tipR := (module*toothNumber + 2*module) / 2
	embedded := baseR < rootR
	fmt.Printf("\n=== M=%.2f N=%.0f PA=%.1f° angle=%.1f°  pitch=%.3f base=%.3f root=%.3f tip=%.3f embedded=%v ===\n",
		module, toothNumber, pressureAng*180/math.Pi, angle*180/math.Pi, pitchR, baseR, rootR, tipR, embedded)
	if embedded {
		fmt.Println("(embedded 4-curve variant: no flank-to-root stubs, root boundary is the solid root circle)")
	}

	// sample the involute flank (base -> tip, equal radial steps), mirror across
	// +X, rotate so the pitch crossing lands at +pi/(2N), then apply `angle`.
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
	// The right flank is the mirror of the +X-symmetric LEFT flank, taken BEFORE
	// the requested `angle` is applied; both are then swung by `angle` together.
	// Mirroring after the swing would leave the tooth symmetric about +X instead
	// of about the `angle` direction (no observable difference at angle == 0).
	var left, right []pt
	for _, p := range mirrored {
		lx, ly := rot(p.x, p.y, rotateAngle)
		rx, ry := lx, -ly
		lx, ly = rot(lx, ly, angle)
		rx, ry = rot(rx, ry, angle)
		left = append(left, pt{lx, ly})
		right = append(right, pt{rx, ry})
	}

	w := sketch.NewWorld()
	s, _ := w.CreateSketch(w.XY())

	// local origin: the movable anchor. Fix == the anchor coincidence that
	// grounds the whole Gear Profile sketch (spec step 5).
	origin := s.CreatePoint(0, 0)
	origin.MoveTo(0, 0)
	s.Fix(origin)

	// four circles sharing the origin as center; driving diameter dims. Only the
	// root circle is solid in Fusion; tip/base/pitch are construction. (This
	// prototype models the tooth's root boundary with an explicit root arc, so
	// the root circle is construction here too — see the flank-to-root block.)
	mkCircle := func(r float64, construction bool) *sketch.Circle {
		c := s.CreateCircle(origin, r)
		c.SetConstruction(construction)
		s.AddConstraint(sketch.NewDiameter(c, 2*r))
		return c
	}
	// In Fusion the root circle is the one SOLID circle. The non-embedded model
	// replaces its role in the tooth loop with an explicit root arc between the two
	// feet, so it is construction there; the embedded tooth has no feet and is
	// bounded by the circle itself, so it stays solid.
	mkCircle(rootR, !embedded)
	tipCircle := mkCircle(tipR, true)
	mkCircle(baseR, true)
	mkCircle(pitchR, true)

	// flank fit points + splines
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

	// spine: origin -> tooth-top; tooth-top on tip circle; +X reference and angle pin.
	topX, topY := rot(tipR, 0, angle)
	toothTop := s.CreatePoint(topX, topY)
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tipCircle))
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	refEnd := s.CreatePoint(tipR, 0)
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, refEnd, tipR),
		sketch.NewVerticalDistance(origin, refEnd, 0),
	)
	reference := s.CreateLine(origin, refEnd)
	reference.SetConstruction(true)
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi)) // [SPUR-F-SPINE]

	// tooth-top arc: caps the flank ends, with no diameter dimension.
	//
	// Fusion's SketchArcs.addByCenterStartEnd does NOT share the SketchPoint handed to
	// it as the centre — it creates a fresh point at that location ([PB-SHARE-XOR-COINCIDENT]).
	// Modelling it as a shared origin proves a scheme the add-in does not build, so the
	// centre is a free point here and is then tied back to the local origin explicitly.
	// Without that coincidence the centre keeps only the arc's own equal-radius relation
	// to the two flank ends, which leaves it one degree of freedom along their
	// perpendicular bisector. Everything else is built about the local origin and dragged
	// onto the anchor, so the stranded centre stays behind by the drag distance and the
	// arc's radius comes out as anything at all. Measured in Fusion 2026-09-02 on a
	// default bevel pair: centre 22.9 mm behind the origin on the pinion for a 0.5743 mm
	// radius, 5.5 mm behind on the driving gear for 17.0204 mm, against the 22.5 mm tip
	// radius both should have had.
	arcCentre := s.CreatePoint(0, 0)
	s.CreateArc(arcCentre, rightPts[len(rightPts)-1], leftPts[len(leftPts)-1]) // [SPUR-F-TOOTHTOP-ARC]
	if !toothTopArcCentreFree {
		s.AddConstraint(sketch.NewCoincident(arcCentre, origin))
	}

	// ribs: lock each flank pair to the spine. Exact [SPUR-F-RIBS] order.
	// The rib takes the axis across the spine and the chain takes the axis along it.
	acrossIsVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	prevMid := origin
	prevMidPt := pt{0, 0}
	for i := range left {
		rib := s.CreateLine(leftPts[i], rightPts[i])
		rib.SetConstruction(true)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPts[i], rightPts[i], right[i].y-left[i].y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPts[i], rightPts[i], right[i].x-left[i].x))
		}
		t := left[i].x*math.Cos(angle) + left[i].y*math.Sin(angle) // foot on spine
		mx, my := t*math.Cos(angle), t*math.Sin(angle)
		mid := s.CreatePoint(mx, my)
		s.AddConstraint(sketch.NewPointOnLine(mid, spine)) // 4. midpoint onto spine first
		s.AddConstraint(sketch.NewMidpoint(mid, rib))      // 5. then midpoint of rib
		if i != len(left)-1 {
			s.AddConstraint(sketch.NewPerpendicular(spine, rib)) // 6. then rib ⊥ spine
		} // The tooth-top arc already constrains the final rib's perpendicular.
		// chain distance from previous midpoint (origin for the first rib)
		if acrossIsVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(prevMid, mid, mx-prevMidPt.x))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(prevMid, mid, my-prevMidPt.y))
		}
		prevMid, prevMidPt = mid, pt{mx, my}
	}

	// flank-to-root lines (non-embedded): radial line root circle -> flank start,
	// with exactly two axis dimensions from the local origin ([SPUR-F-FLANK-ROOT]).
	// The bench cannot reach Fusion's dimension-value semantics: these targets are
	// signed, but a Fusion dimension holds a magnitude plus a direction captured at
	// creation, and assigning a negative value flips the point to the other side
	// ([PB-DIM-VALUE-SEMANTICS], found in-Fusion 2026-08-24). A transcription that
	// writes ry < 0 into parameter.value passes here and mirrors the root end there.
	addFlankRoot := func(flankStart *sketch.Point, seed pt) *sketch.Point {
		n := math.Hypot(seed.x, seed.y)
		rx, ry := rootR*seed.x/n, rootR*seed.y/n
		re := s.CreatePoint(rx, ry)
		s.CreateLine(re, flankStart)
		s.AddConstraint(
			sketch.NewHorizontalDistance(origin, re, rx),
			sketch.NewVerticalDistance(origin, re, ry),
		)
		return re
	}
	// The embedded tooth draws neither stub: its flank already starts inside the root
	// circle, so there is nothing to bridge and Fusion derives the bottom boundary by
	// splitting the solid root circle where the flanks cross it. Nothing else in the
	// scheme replaces what the stubs' four axis dimensions were pinning, which is the
	// question this case exists to answer.
	if !embedded {
		leftFoot := addFlankRoot(leftPts[0], left[0])
		rightFoot := addFlankRoot(rightPts[0], right[0])

		// explicit root arc — the boundary Fusion derives by splitting the solid root
		// circle at the two feet. Completes the 6-curve tooth loop.
		rootArc := s.CreateArc(s.CreatePoint(0, -rootR*0.1), rightFoot, leftFoot)
		s.AddConstraint(sketch.NewDiameter(rootArc, 2*rootR))
	}

	// --- solve & verify ---
	res, err := s.Solve(ctx)
	if err != nil {
		fmt.Println("solve error:", err)
	}
	rep := s.Verify(ctx, sketch.WithProbe())
	fmt.Printf("Solve: converged=%v DOF=%d redundant=%d residual=%.1e | Verify: status=%s conditioning=%.2e profiles=%d\n",
		res.Converged, res.DOF, res.Redundant, res.Residual, rep.Status, rep.Conditioning, len(rep.Profiles))

	// PRIMARY gate: the "fully constrained" proof — the faithful analog of
	// Fusion's isFullyConstrained + not-over-constrained. Status==FullyConstrained
	// already implies solvable + DOF 0 + no redundant + no conflict; add the
	// scale-invariant conditioning check so the DOF-0 verdict isn't near-singular.
	condGate := math.Max(1e-6, 4*math.Sqrt(1e-10)) // 4e-5 at the default tolerance
	primary := rep.Status == sketch.FullyConstrained && rep.Conditioning >= condGate

	// ADVISORY signals (reported, not part of the full-constraint gate; see README):
	//   * ProfilesValid: TRUE — the tooth forms one clean, extrudable 6-curve loop.
	//     (This required a fix in the sketch engine: a line meeting an arc at a
	//     shared loop corner — the flank-to-root line meeting the root arc — was
	//     false-flagged as a degenerate arrangement. Fixed in lestrrat-3d/sketch
	//     main (PR #12). Against an older engine WITHOUT that fix this reads false,
	//     which is a tool bug, not a gear-scheme defect.)
	//   * Probe ambiguity: TRUE and expected — a draw-then-constrain tooth is seeded
	//     at its pose and constrained; the pure-constraint system still admits
	//     branch/mirror flips the seed resolves, exactly as Fusion relies on initial
	//     geometry placement. DOF==0 means each discrete solution is itself rigid.
	fmt.Printf("  PRIMARY GATE (full constraint) = %v\n", primary)
	fmt.Printf("  advisory: profilesValid=%v (true with the engine's #12 corner-join fix) probeAmbiguous=%v (expected — seeded)\n",
		rep.ProfilesValid, rep.Probe != nil && rep.Probe.Ambiguous())
	return primary
}

func main() {
	const pa = 20.0 * math.Pi / 180.0
	// The spur default (N=17) plus a small and a large non-embedded size, to show
	// the scheme fully constrains parametrically.
	// Healthy non-embedded sizes, comfortably clear of the base≈root transition
	// (~N=42 at PA=20°) where the flank-to-root stubs vanish and the system turns
	// ill-conditioned — see README for that caught-fragility finding.
	cases := []struct {
		module, teeth, angle float64
	}{
		{1, 12, 0}, {1, 17, 0}, {2, 20, 0}, {3, 15, 0},
		{2, 20, 30 * math.Pi / 180},
		{1, 17, 90 * math.Pi / 180},
		{3, 15, -60 * math.Pi / 180},
		// Embedded sizes (base < root, above ~N=42 at PA=20°). These are not an
		// exotic corner: the bevel gear borrows this drawer through VirtualSpurProxy
		// and its virtual tooth count is embedded for every ordinary bevel, so the
		// bevel tooth has only ever been drawn on this branch. N=43 at module 1 with
		// a 180 degree angle is the exact case a default 31/31 bevel pair draws.
		{1, 43, 0},
		{1, 43, 180 * math.Pi / 180},
		{1, 50, 0},
		{2, 60, 30 * math.Pi / 180},
	}
	ctx := context.Background()
	allPass := true
	for _, c := range cases {
		if !checkGearProfile(ctx, c.module, c.teeth, pa, c.angle, 15, false) {
			allPass = false
		}
	}
	// The tooth-top arc's centre must be tied to the local origin. Prove the bench
	// actually detects its absence, so this cannot regress back into a silent free
	// centre: the same case with the coincidence dropped has to FAIL the gate.
	fmt.Println("\n--- negative control: tooth-top arc centre left free (must FAIL) ---")
	if checkGearProfile(ctx, 1, 43, pa, 180*math.Pi/180, 15, true) {
		fmt.Println("NEGATIVE CONTROL DID NOT FAIL — the bench no longer detects a free tooth-top arc centre.")
		allPass = false
	} else {
		fmt.Println("negative control failed as required: a free arc centre does not fully constrain.")
	}

	fmt.Println()
	if allPass {
		fmt.Println("ALL PASS — the spur Gear Profile constraint scheme fully constrains across sizes.")
		fmt.Println("Cleared to generate Fusion add-in code.")
	} else {
		fmt.Println("FAIL — scheme does not fully constrain; fix the scheme before generating Fusion code.")
		os.Exit(1)
	}
}
