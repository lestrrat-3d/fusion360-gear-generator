// Package spurgear_test proves the spur gear's sketches and solids against the
// step list compiled from spec/spurgear/instructions.md and spec/spurgear/fusion.md.
//
// Units. The spec and the Fusion module work in Fusion's internal centimetres;
// this proof works in the sketch engine's base millimetres, because
// proof/involute is written in millimetres and the two harnesses share that
// base. Nothing the proof asserts depends on the choice: every claim here is a
// ratio, a count, or a length compared against another length in the same unit.
package spurgear_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
)

// Parameter keys. A case table carries float64s only, so the two booleans the
// dialog offers are carried as 0 and 1.
const (
	pModule      = "module"
	pToothNumber = "toothNumber"
	pPressure    = "pressureAngle"
	pSteps       = "involuteSteps"
	pAngle       = "angle"
	pThickness   = "thickness"
	pBore        = "boreDiameter"
	pChamfer     = "chamferTooth"
)

// dims reads the four gear circles out of a case's parameters.
func dims(p map[string]float64) involute.Dimensions {
	return involute.Derive(p[pModule], p[pToothNumber], p[pPressure])
}

// toolsCases sweeps the sizes the Tools sketch has to carry the anchor at. The
// sketch holds one projected point and no dimensioned geometry, so the regime
// that matters to it is only that the projection grounds whatever is built on
// it, at any anchor position.
var toolsCases = []proofkit.Case{
	{Name: "origin-anchor", Params: map[string]float64{"anchorX": 0, "anchorY": 0}},
	{Name: "offset-anchor", Params: map[string]float64{"anchorX": 37.5, "anchorY": -12.25}},
	{Name: "negative-anchor", Params: map[string]float64{"anchorX": -8, "anchorY": -8}},
}

// stepToolsSketch draws the Tools sketch: the projection of the user's anchor
// point and nothing else.
//
// The projection is modelled with CreateReferencePoint, the engine's
// externally-locked import, which is what sketch.project(...) produces in
// Fusion: a point whose position comes from outside this sketch and which the
// solver never moves. Everything the gear builds later is tied back to this one
// point ([SPUR-F-ANCHOR-CHAIN]).
func stepToolsSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the user's anchor point into the Tools sketch")
	anchor := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "user anchor point")
	anchor.SetName("toolsAnchor")

	if got := len(s.Entities()); got != 0 {
		t.Fatalf("Tools sketch drew %d curves; the spec says it draws no geometry of its own", got)
	}
	if got := len(s.Points()); got != 1 {
		t.Fatalf("Tools sketch holds %d authored points, want only the projected anchor", got)
	}
	if anchor.X() != p["anchorX"] || anchor.Y() != p["anchorY"] {
		t.Fatalf("projected anchor sits at (%v, %v), want (%v, %v)",
			anchor.X(), anchor.Y(), p["anchorX"], p["anchorY"])
	}
}

// profileCases is the regime the Gear Profile constraint scheme has to hold
// across, as the spec's Sketch Discipline section states it: several
// Module/ToothNumber sizes, the whole signed range of the angle argument
// including a quarter turn either way and the bevel virtual tooth's half turn,
// the low end of the involute-sample count that sets how many ribs the sketch
// carries, and both routes into the embedded shape — a high tooth count at the
// ordinary pressure angle, and a moderate tooth count at a large one.
//
// The last case sits on the embedded test's strict-< boundary, where the base
// circle and the root circle have the same radius.
var profileCases = []proofkit.Case{
	{Name: "standard", Params: profileParams(1, 17, 20, 15, 0)},
	{Name: "fine-small", Params: profileParams(0.5, 12, 20, 15, 0)},
	{Name: "coarse-large", Params: profileParams(4, 40, 20, 15, 0)},
	{Name: "three-samples", Params: profileParams(1, 17, 20, 3, 0)},
	{Name: "four-samples", Params: profileParams(1, 17, 20, 4, 0)},
	{Name: "right-hand-helix", Params: profileParams(1, 17, 20, 15, 30)},
	{Name: "left-hand-helix", Params: profileParams(1, 17, 20, 15, -30)},
	{Name: "quarter-turn", Params: profileParams(1, 17, 20, 15, 90)},
	{Name: "negative-quarter-turn", Params: profileParams(1, 17, 20, 15, -90)},
	{Name: "bevel-virtual-tooth", Params: profileParams(1, 17, 20, 15, 180)},
	{Name: "embedded-by-tooth-count", Params: profileParams(1, 60, 20, 15, 0)},
	{Name: "embedded-by-pressure-angle", Params: profileParams(1, 30, 25, 15, 0)},
	{Name: "embedded-and-rotated", Params: profileParams(1, 60, 20, 6, -45)},
	{Name: "base-equals-root", Params: profileParams(1, 42, equalRootPressureAngle(42), 15, 0)},
}

// profileParams builds one case's parameters from display-unit numbers: degrees
// for the two angles, everything else as the spec states it.
func profileParams(module, toothNumber, pressureDeg float64, steps int, angleDeg float64) map[string]float64 {
	return map[string]float64{
		pModule:      module,
		pToothNumber: toothNumber,
		pPressure:    pressureDeg * math.Pi / 180,
		pSteps:       float64(steps),
		pAngle:       angleDeg * math.Pi / 180,
	}
}

// equalRootPressureAngle returns the pressure angle, in degrees, at which the
// base circle lands exactly on the root circle for this tooth count. It is the
// inverse of the spec's embedded threshold toothNumber = 2.5 / (1 - cos(a)).
func equalRootPressureAngle(toothNumber float64) float64 {
	return math.Acos(1-2.5/toothNumber) * 180 / math.Pi
}

// stepGearProfileSketch draws the whole Gear Profile sketch — the four gear
// circles, the involute tooth with its ribs, spine and flank-to-root lines, and
// the anchoring that slides the drawing onto the user's anchor point.
//
// It is one step because it is one Fusion timeline entry.
func stepGearProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	d := dims(p)
	teeth := p[pToothNumber]
	steps := int(p[pSteps])
	angle := p[pAngle]

	proofkit.Step(t, "local origin on the projected anchor")
	// The Gear Profile sketch re-projects the Tools sketch's anchor
	// ([SPUR-F-ANCHOR-CHAIN]) and keeps its own movable local origin
	// ([SPUR-F-LOCAL-ORIGIN]): a fresh point at (0, 0), never the sketch's own
	// immutable origin. The coincidence between the two is the step-5 anchoring,
	// added here at the point in the build where the tooth generator's draw()
	// adds it — after the geometry, so the whole drawing is dragged onto the
	// anchor as a unit. The proof adds it first only because the engine, unlike
	// Fusion, does not solve incrementally: the order of AddConstraint calls
	// cannot change the system it builds.
	anchor := s.CreateReferencePoint(0, 0, "Tools sketch anchor projection")
	anchor.SetName("projectedAnchor")
	origin := s.CreatePoint(0, 0)
	origin.SetName("localOrigin")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))

	proofkit.Step(t, "the four gear circles, all sharing the local origin")
	root := s.CreateCircle(origin, d.Root)
	tip := s.CreateCircle(origin, d.Tip)
	base := s.CreateCircle(origin, d.Base)
	pitch := s.CreateCircle(origin, d.Pitch)
	root.SetName("rootCircle")
	tip.SetName("tipCircle")
	base.SetName("baseCircle")
	pitch.SetName("pitchCircle")
	// Only the root circle is solid; the other three are construction, which is
	// why the tip circle bounds no profile and the body extrude sees a disc
	// rather than an annulus.
	tip.SetConstruction(true)
	base.SetConstruction(true)
	pitch.SetConstruction(true)
	s.AddConstraint(
		sketch.NewDiameter(root, 2*d.Root),
		sketch.NewDiameter(tip, 2*d.Tip),
		sketch.NewDiameter(base, 2*d.Base),
		sketch.NewDiameter(pitch, 2*d.Pitch),
	)

	proofkit.Step(t, "sample the involute flanks and draw them as fitted splines")
	left, right := involute.Flanks(d.Base, d.Tip, d.Pitch, teeth, steps, angle)
	if len(left) != steps || len(right) != steps {
		t.Fatalf("flank sampling produced %d/%d points, want %d each",
			len(left), len(right), steps)
	}
	if got := math.Hypot(left[0].X, left[0].Y); math.Abs(got-d.Base) > 1e-9*d.Base {
		t.Fatalf("first flank sample sits at radius %v, want the base radius %v", got, d.Base)
	}
	if got := math.Hypot(left[len(left)-1].X, left[len(left)-1].Y); math.Abs(got-d.Tip) > 1e-9*d.Tip {
		t.Fatalf("last flank sample sits at radius %v, want the tip radius %v", got, d.Tip)
	}
	leftPoints := make([]*sketch.Point, len(left))
	rightPoints := make([]*sketch.Point, len(right))
	for i := range left {
		leftPoints[i] = s.CreatePoint(left[i].X, left[i].Y)
		rightPoints[i] = s.CreatePoint(right[i].X, right[i].Y)
		leftPoints[i].SetName(fmt.Sprintf("leftFit%d", i))
		rightPoints[i].SetName(fmt.Sprintf("rightFit%d", i))
	}
	leftFlank, err := s.CreateFitSpline(leftPoints...)
	if err != nil {
		t.Fatalf("left flank spline: %v", err)
	}
	rightFlank, err := s.CreateFitSpline(rightPoints...)
	if err != nil {
		t.Fatalf("right flank spline: %v", err)
	}
	leftFlank.SetName("leftFlank")
	rightFlank.SetName("rightFlank")

	proofkit.Step(t, "tooth-top point and the arc that caps the tooth [SPUR-F-TOOTHTOP-ARC]")
	last := len(leftPoints) - 1
	toothTop := s.CreatePoint(d.Tip*math.Cos(angle), d.Tip*math.Sin(angle))
	toothTop.SetName("toothTopPoint")
	s.AddConstraint(sketch.NewPointOnCircle(toothTop, tip))
	// addByCenterStartEnd shares the start and end points and COPIES the centre,
	// so the arc's centre is a fresh point that has to be tied back to the local
	// origin ([PB-SHARE-XOR-COINCIDENT], [SPUR-F-TOOTHTOP-ARC] step 3). The
	// engine's CreateArc copies its centre the same way, so the two model the
	// same defect: leave the coincidence out and the centre is free.
	arcCentre := s.CreatePoint(0, 0)
	arcCentre.SetName("toothTopArcCentre")
	toothTopArc := s.CreateArc(arcCentre, rightPoints[last], leftPoints[last])
	toothTopArc.SetName("toothTopArc")
	s.AddConstraint(sketch.NewCoincident(arcCentre, origin))
	// No diameter dimension on the arc: the coincident centre and the two shared
	// ends already determine it.

	proofkit.Step(t, "spine, +X reference line and the confirming angular dimension [SPUR-F-SPINE]")
	spine := s.CreateLine(origin, toothTop)
	spine.SetConstruction(true)
	spine.SetName("spine")
	referenceEnd := s.CreatePoint(d.Tip, 0)
	referenceEnd.SetName("plusXReferenceEnd")
	reference := s.CreateLine(origin, referenceEnd)
	reference.SetConstruction(true)
	reference.SetName("plusXReference")
	s.AddConstraint(
		sketch.NewHorizontalDistance(origin, referenceEnd, d.Tip),
		sketch.NewVerticalDistance(origin, referenceEnd, 0),
	)
	// The angular dimension runs from the reference to the spine, in that order,
	// and carries the sign of the requested rotation: it is what a mirrored
	// scheme gets wrong at -angle while still solving at +angle. The engine reads
	// this dimension in the sketch's default angle unit, which is degrees.
	s.AddConstraint(sketch.NewAngle(reference, spine, angle*180/math.Pi))

	proofkit.Step(t, "one rib per fit-point index, endpoints included [SPUR-F-RIBS]")
	// The rib takes the axis ACROSS the spine and the midpoint chain the one
	// ALONG it. At angle 0 that is vertical for the rib and horizontal for the
	// chain; past a half turn of the |cos| >= |sin| test the two swap, which is
	// what keeps a tooth at a quarter turn solvable.
	ribAcrossVertical := math.Abs(math.Cos(angle)) >= math.Abs(math.Sin(angle))
	previous := origin
	for i := range leftPoints {
		rib := s.CreateLine(leftPoints[i], rightPoints[i])
		rib.SetConstruction(true)
		rib.SetName(fmt.Sprintf("rib%d", i))
		// The engine's linear-distance target is signed; Fusion's is the
		// magnitude with the direction captured from the seed
		// ([PB-DIM-VALUE-SEMANTICS]), so the sign here is the seed side there.
		if ribAcrossVertical {
			s.AddConstraint(sketch.NewVerticalDistance(leftPoints[i], rightPoints[i], right[i].Y-left[i].Y))
		} else {
			s.AddConstraint(sketch.NewHorizontalDistance(leftPoints[i], rightPoints[i], right[i].X-left[i].X))
		}
		// The midpoint is seeded at the foot of the left fit point on the spine,
		// not at the rib's own 2-D midpoint and not at (fitX, 0).
		along := left[i].X*math.Cos(angle) + left[i].Y*math.Sin(angle)
		midX, midY := along*math.Cos(angle), along*math.Sin(angle)
		mid := s.CreatePoint(midX, midY)
		mid.SetName(fmt.Sprintf("ribMid%d", i))
		s.AddConstraint(sketch.NewPointOnLine(mid, spine))
		s.AddConstraint(sketch.NewMidpoint(mid, rib))
		if i != len(leftPoints)-1 {
			// The last rib carries no perpendicular: the tooth-top arc already
			// holds its two ends at equal radius either side of the spine, and
			// keeping both is what over-constrains the sketch.
			s.AddConstraint(sketch.NewPerpendicular(spine, rib))
		}
		if ribAcrossVertical {
			s.AddConstraint(sketch.NewHorizontalDistance(previous, mid, midX-previous.X()))
		} else {
			s.AddConstraint(sketch.NewVerticalDistance(previous, mid, midY-previous.Y()))
		}
		previous = mid
	}

	embedded := d.Embedded()
	if math.Hypot(left[0].X, left[0].Y) < d.Root != embedded {
		t.Fatalf("embedded flag %v disagrees with the drawn flank start", embedded)
	}
	if !embedded {
		proofkit.Step(t, "flank-to-root lines, two axis dimensions each [SPUR-F-FLANK-ROOT]")
		scale := d.Root / d.Base
		for _, side := range []struct {
			name  string
			start *sketch.Point
			at    involute.Pt
		}{
			{"left", leftPoints[0], left[0]},
			{"right", rightPoints[0], right[0]},
		} {
			endX, endY := side.at.X*scale, side.at.Y*scale
			rootEnd := s.CreatePoint(endX, endY)
			rootEnd.SetName(side.name + "RootEnd")
			stub := s.CreateLine(rootEnd, side.start)
			stub.SetName(side.name + "FlankToRoot")
			s.AddConstraint(
				sketch.NewHorizontalDistance(origin, rootEnd, endX),
				sketch.NewVerticalDistance(origin, rootEnd, endY),
			)
		}
	}

	proofkit.Step(t, "the two regions the sketch closes, and their curve counts")
	assertProfileContract(t, s, d, embedded)
}

// assertProfileContract checks the two closed regions the Gear Profile sketch
// owes the two extrude steps, and the curve counts each of those steps searches
// on.
//
// The counts are Fusion's. The engine keeps a crossed curve whole and reports
// the boundary as parameter ranges on it, where Fusion splits the curve into
// separate SketchArcs, so the root circle reaches the tooth loop here as
// fragments of one *sketch.Circle rather than as one SketchArc, and reaches the
// disc whole rather than as the two arcs its two contacts cut it into. Both
// counts are therefore derived from the engine's own boundary composition — the
// number of distinct entities on the loop, and the number of places the tooth
// boundary lands on the root circle — rather than read off a stand-in drawn to
// have them.
func assertProfileContract(t testing.TB, s *sketch.Sketch, d involute.Dimensions, embedded bool) {
	t.Helper()
	if _, err := s.Solve(context.Background()); err != nil {
		t.Fatalf("solve before reading profiles: %v", err)
	}
	profiles := s.Profiles()
	if d.Base == d.Root {
		// The embedded test is a strict <, so a base circle sitting exactly on
		// the root circle counts as NOT embedded and draws a zero-length
		// flank-to-root stub on each side. The stub's two ends coincide and the
		// flank starts exactly on the root circle, and the engine reports that
		// arrangement as degenerate: every detected region comes back
		// unextrudable, so the harness refuses the case however the constraints
		// come out. This is the ill-conditioned boundary the spec keeps the
		// strict comparison for, and it is the one point of the regime this
		// proof cannot gate.
		report := s.Verify(context.Background())
		proofkit.Unmodelled(t,
			"base radius equals root radius, so each flank-to-root stub has zero length; "+
				"the engine reports the arrangement as degenerate (%d of %d regions unextrudable) "+
				"and refuses the region set, though the constraint system itself reached DOF=%d "+
				"at conditioning %.2e",
			len(report.InvalidProfiles), len(report.Profiles), report.DOF, report.Conditioning)
		return
	}
	if len(profiles) != 2 {
		for _, p := range profiles {
			t.Logf("region: area=%.5f entities=%d valid=%v", p.Area, len(p.Entities), p.Valid)
		}
		t.Fatalf("sketch closes %d regions, want exactly two: the tooth section and the disc inside the root circle", len(profiles))
	}
	discArea := math.Pi * d.Root * d.Root
	var tooth, disc *sketch.Profile
	for _, p := range profiles {
		if math.Abs(p.Area-discArea) < 1e-6*discArea {
			disc = p
		} else {
			tooth = p
		}
	}
	if disc == nil || tooth == nil {
		t.Fatalf("no region has the root circle's area %.5f; areas are %.5f and %.5f",
			discArea, profiles[0].Area, profiles[1].Area)
	}
	if !tooth.Valid || !disc.Valid {
		t.Fatalf("regions are not both extrudable: tooth valid=%v disc valid=%v", tooth.Valid, disc.Valid)
	}
	if tooth.Area <= 0 {
		t.Fatalf("tooth section has area %v", tooth.Area)
	}

	// The disc's boundary is the root circle and nothing else: the tip circle is
	// construction geometry and bounds no profile, so this is a disc and not an
	// annulus.
	if len(disc.Entities) != 1 {
		t.Fatalf("disc boundary uses %d entities, want the root circle alone", len(disc.Entities))
	}
	if disc.Entities[0].Name() != "rootCircle" {
		t.Fatalf("disc boundary is bounded by %q, want the root circle", disc.Entities[0].Name())
	}

	splines, arcs, lines := 0, 0, 0
	rootCuts := map[float64]bool{}
	for _, e := range tooth.Entities {
		switch e.(type) {
		case *sketch.FitSpline:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			lines++
		case *sketch.Circle:
			// The root circle. Fusion's split turns the stretch of it this loop
			// walks into one SketchArc.
			arcs++
		default:
			t.Fatalf("tooth loop carries an unexpected entity %T", e)
		}
	}
	for _, edge := range tooth.Outer {
		if _, ok := edge.Entity.(*sketch.Circle); !ok {
			continue
		}
		// A circle's parameter wraps at its seam, so a stretch that crosses the
		// seam arrives as two fragments. The bounds that are not the seam are
		// the places the tooth actually meets the root circle.
		for _, bound := range []float64{edge.TStart, edge.TEnd} {
			if bound > 1e-9 && bound < 1-1e-9 {
				rootCuts[bound] = true
			}
		}
	}
	if len(rootCuts) != 2 {
		t.Fatalf("the tooth meets the root circle at %d places, want 2 — the two cuts that split it into the disc's two arcs", len(rootCuts))
	}

	wantLines := 2
	if embedded {
		wantLines = 0
	}
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Fatalf("tooth loop has %d NURBS, %d arcs and %d lines; the extrude step searches for 2, 2 and %d",
			splines, arcs, lines, wantLines)
	}
	if total := splines + arcs + lines; total != 6-2*boolToInt(embedded) {
		t.Fatalf("tooth loop has %d curves, want %d", total, 6-2*boolToInt(embedded))
	}
}

func boolToInt(v bool) int {
	if v {
		return 1
	}
	return 0
}

// boreCases sweeps the bore diameters the Bore Profile sketch is drawn at, at
// the two ends of the size range. Bore Diameter 0 is not a case here: the bore
// step returns before the sketch is created.
var boreCases = []proofkit.Case{
	{Name: "small-bore", Params: map[string]float64{pBore: 3, "anchorX": 0, "anchorY": 0}},
	{Name: "large-bore", Params: map[string]float64{pBore: 40, "anchorX": 0, "anchorY": 0}},
	{Name: "offset-anchor", Params: map[string]float64{pBore: 6, "anchorX": -14, "anchorY": 21}},
}

// stepBoreProfileSketch draws the Bore Profile sketch: the projection of the
// anchor, the bore circle centred on it, and the tooth generator's stray local
// origin grounded on the same projection.
func stepBoreProfileSketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project the anchor and draw the bore circle on it")
	projected := s.CreateReferencePoint(p["anchorX"], p["anchorY"], "Tools sketch anchor projection")
	projected.SetName("projectedAnchor")
	bore := s.CreateCircle(projected, p[pBore]/2)
	bore.SetName("boreCircle")
	s.AddConstraint(sketch.NewDiameter(bore, p[pBore]))

	proofkit.Step(t, "ground the tooth generator's stray local origin on the same projection")
	// The tooth generator's constructor always adds a local origin at (0, 0, 0),
	// so this sketch carries one unused point. It is grounded on the projected
	// anchor, not on the sketch's own origin point, so the sketch reaches full
	// constraint and the bore follows the anchor.
	localOrigin := s.CreatePoint(0, 0)
	localOrigin.SetName("localOrigin")
	s.AddConstraint(sketch.NewCoincident(localOrigin, projected))

	if bore.R() <= 0 {
		t.Fatalf("bore radius is %v; the step runs only for a positive Bore Diameter", bore.R())
	}
}
