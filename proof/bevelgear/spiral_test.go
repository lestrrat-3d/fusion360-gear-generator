package bevelgear_test

// The spiral tooth body, section 3a.
//
// This is the psi > 0 branch of the tooth-body hook. At psi = 0 the hook
// returns immediately with the straight tooth's two conical trims, and every
// step here is skipped; any value above 0 builds a curved tooth in place of
// them. Both sides of that gate are in the case table.
//
// ---------------------------------------------------------------------------
// ONE SUBSTITUTION GOVERNS THE WHOLE CHAIN, and it is the one the tooth loft
// already carries: the slabs are cut PERPENDICULAR TO THE SHAFT AXIS rather
// than to the cone element, because the tooth this proof builds stands its
// sections square to that axis. The two differ by the root cone angle, so every
// station here is the cone distance times its cosine.
//
// It costs nothing the steps read. The twist law keys on
// (R_mean - R_heelFace) / span and the crown on (R_heel - R_heelFace) / span,
// and a cone distance and an axial station differ by ONE constant factor, which
// both ratios cancel. What the substitution does give up is the tilt of each cut
// face, which no reading below depends on.
//
// The boolean rule from solids_test.go holds here too: nothing is split and
// nothing is joined. A slab is built as the piece the split would have left,
// and the slabs are laid apart along the axis.
// ---------------------------------------------------------------------------

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// straightBranch reports whether this case takes the psi = 0 path, where the
// spiral build does not run at all.
func straightBranch(c bevelCase) bool { return c.SpiralAngle <= 0 }

// slabLift is how far apart consecutive slabs are laid along the axis so no two
// of them touch. A contact the read-only intersection cannot classify is
// reported Suspect, and laying them apart is what the whole proof does instead.
const slabLift = 400.0

// ---------------------------------------------------------------------------
// S19 -- the cone-element sketch.
// ---------------------------------------------------------------------------

// stepConeElement draws the cone-element construction line Apex -> (Apex +
// R_heel * coneVec) on the axial plane. The Trace Plane is that axial plane
// rotated 90 degrees about this line, so this line is what decides where the
// trace plane lands.
//
// THE TANGENT PLANE IS ON THE ROOT CONE, not the pitch cone. The canonical
// crown-gear construction lays the trace in the plane tangent to the PITCH
// cone; this implementation uses the dedendum element Apex->C / Apex->D
// instead. The two tangent planes differ only by the small dedendum angle, so
// the arc's shape is essentially identical, but psi then ends up measured on
// the root cone rather than the pitch cone. That is a departure from the
// canonical reference and it is recorded here rather than left to be
// rediscovered.
func stepConeElement(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit.Unmodelled(t, "psi is 0, so this is a straight bevel: the tooth "+
			"body takes the apex loft and the two conical trims unchanged and "+
			"authors no cone-element sketch at all")
	}
	f := latticeOf(c, m)
	tr := traceFor(c, m)

	proofkit.Step(t, "%s: the cone element out to R_heel = %.4f", m.Name, tr.RHeel)
	coneDir := f.Ded.direction()
	apex := s.CreatePoint(0, 0)
	apex.SetName("Apex")
	s.Fix(apex)
	end := s.CreatePoint(coneDir.X*tr.RHeel, coneDir.Y*tr.RHeel)
	end.SetName(m.Name + " cone element end")
	s.Fix(end)
	line := s.CreateLine(apex, end)
	line.SetName(m.Name + " Cone Element")
	line.SetConstruction(true)

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: the cone element sketch did not solve: %v", m.Name, err)
	}

	// The x axis of the trace frame IS this element, and a point's x there is
	// its cone distance. Getting the element wrong -- building it along
	// Apex->Apex2 or along the shaft axis -- skews the whole frame.
	near(t, m.Name+" cone element length", line.Length(), tr.RHeel, 1e-9)
	gammaRoot := m.Gamma - math.Atan2(1.25*c.Module, c.R)
	near(t, m.Name+" cone element runs along the ROOT cone",
		math.Atan2(coneDir.Y, coneDir.X), gammaRoot, 1e-9)
	if math.Abs(gammaRoot-m.Gamma) < 1e-12 {
		t.Errorf("%s: the root cone and the pitch cone came out identical, so the "+
			"departure this step records cannot be seen", m.Name)
	}

	// The heel is the OUTER end. A negative span silently inverts the entire
	// spiral frame -- the cutter-arc direction, the slice direction and the
	// per-segment twist -- and the gear comes out completely wrong with no
	// error, so the frame checks it before building anything.
	if tr.Span <= 0 {
		t.Errorf("%s: span is %.6f; the heel must be the end FARTHER from the apex",
			m.Name, tr.Span)
	}
	near(t, m.Name+" span is the face measured along the element",
		tr.Span, tr.RHeel-tr.RToe, 1e-12)
	near(t, m.Name+" R_mean is midway along the face",
		tr.RMean, (tr.RToe+tr.RHeel)/2, 1e-12)
}

// ---------------------------------------------------------------------------
// S21 -- the 2-D tooth trace, the genuine cutter arc.
// ---------------------------------------------------------------------------

// stepToothTrace draws the cutter circle and the trace arc in the tangent
// plane's own frame: origin at the apex, x along the cone element so a point's
// x is its cone distance, y circumferential.
//
// The trace is EXACTLY an arc of a circle of the cutter's radius. The only
// questions are where its centre sits and which portion is kept, and both are
// answered in closed form here rather than by fitting a spline through sampled
// points: a fitted spline looks similar, is not the cutter circle, and cannot
// carry the radius and centre constraints.
//
// THE BENCH BUILDS THE ARC FROM ITS CENTRE, START AND END where Fusion builds a
// three-point arc and then constrains its centre and radius. Fusion's form is
// deliberately left with free degrees of freedom -- dimensioning the endpoints
// over-constrains the solve against the cone-element plane -- and is exempt
// from the full-constraint gate there. The bench gate does not exempt anything,
// so the arc is built from the two ends the closed form fixes and the MEAN
// POINT lying on it becomes an assertion instead of a constraint. That is the
// invariant that matters: the arc contains (R_mean, 0) and its centre is r_c
// away from it.
func stepToothTrace(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		// Even on the branch that does not author this sketch, the limit is
		// worth stating: as psi goes to 0 the centre goes due "north" of the
		// mean point and the arc through it becomes tangent to the element,
		// which is the straight tooth.
		tr := traceFor(c, m)
		near(t, m.Name+" at psi = 0 the cutter centre is due north of the mean point",
			tr.Centre.X, tr.RMean, 1e-9)
		near(t, m.Name+" at psi = 0 the centre stands off by the cutter radius",
			math.Abs(tr.Centre.Y), tr.CutterRadius, 1e-9)
		proofkit.Unmodelled(t, "psi is 0, so this is a straight bevel and no trace "+
			"sketch is authored")
	}
	tr := traceFor(c, m)

	proofkit.Step(t, "%s: cutter radius %.4f, hand %+.0f, centre (%.4f, %.4f)",
		m.Name, tr.CutterRadius, tr.HandSign, tr.Centre.X, tr.Centre.Y)

	centre := s.CreatePoint(tr.Centre.X, tr.Centre.Y)
	centre.SetName(m.Name + " cutter centre")
	s.Fix(centre)
	cutter := s.CreateCircle(centre, tr.CutterRadius)
	cutter.SetName(m.Name + " cutter circle")
	cutter.SetConstruction(true)
	s.AddConstraint(sketch.NewDiameter(cutter, 2*tr.CutterRadius))

	// The kept ends are taken a hair PAST the face, so the arc reaches cleanly
	// beyond the end trims.
	toe := s.CreatePoint(tr.Toe2D.X, tr.Toe2D.Y)
	toe.SetName(m.Name + " trace toe")
	s.Fix(toe)
	heel := s.CreatePoint(tr.Heel2D.X, tr.Heel2D.Y)
	heel.SetName(m.Name + " trace heel")
	s.Fix(heel)

	// The arc gets its OWN centre point, as Fusion's three-point arc does, and
	// its centre is then made coincident with the cutter circle's. That
	// coincident is two rows and the arc's own equal-radius row already holds
	// one of them -- a point equidistant from the two ends lies on their
	// perpendicular bisector, and the true centre does. So the proof states
	// only the row that is not implied, exactly as the tooth's own arcs do.
	arcCentre := s.CreatePoint(tr.Centre.X, tr.Centre.Y)
	arcCentre.SetName(m.Name + " trace arc centre")
	arc := s.CreateArc(arcCentre, toe, heel)
	arc.SetName(m.Name + " 2D Tooth Trace")
	// Which of the two axis alignments is the row that is not implied depends
	// on which way the bisector runs: an alignment parallel to it would meet it
	// everywhere instead of once.
	bisector := pv(-(tr.Heel2D.Y - tr.Toe2D.Y), tr.Heel2D.X-tr.Toe2D.X)
	if math.Abs(bisector.X) >= math.Abs(bisector.Y) {
		s.AddConstraint(sketch.NewVerticalPoints(centre, arcCentre))
	} else {
		s.AddConstraint(sketch.NewHorizontalPoints(centre, arcCentre))
	}

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: the trace sketch did not solve: %v", m.Name, err)
	}

	// The invariants a correct trace must satisfy.
	//
	// 1. APEX-CENTRED. The toe and heel loci are circles about the apex, so a
	// point's distance from the origin IS its cone distance. Centring them
	// anywhere else -- on the back-cone centre, on a sketch origin -- makes
	// every toe and heel placement wrong.
	lo := tr.RToe - spiralEndOvershoot*tr.Span
	hi := tr.RHeel + spiralEndOvershoot*tr.Span
	near(t, m.Name+" the toe end lies on the toe circle", solved(toe).norm(), lo, 1e-6)
	near(t, m.Name+" the heel end lies on the heel circle", solved(heel).norm(), hi, 1e-6)

	// 2. CUTTER RADIUS. The arc is one circle, of the cutter's radius, centred
	// where the cutter circle is -- so it is the GENUINE cutter arc and not a
	// look-alike spline through the same three points.
	near(t, m.Name+" the trace arc's radius is the cutter radius",
		arc.R(), tr.CutterRadius, 1e-6)
	near(t, m.Name+" the trace arc's centre gap to the cutter circle",
		solved(arc.Center).distance(solved(centre)), 0, 1e-9)
	near(t, m.Name+" the cutter circle's radius", cutter.R(), tr.CutterRadius, 1e-9)
	if c.CutterRadius == 0 {
		near(t, m.Name+" an unset Cutter Radius defaults to the mean cone distance",
			tr.CutterRadius, tr.RMean, 1e-12)
	}

	// 3. PASSES THROUGH THE MEAN POINT, and the centre is exactly r_c from it.
	mean := pv(tr.RMean, 0)
	near(t, m.Name+" the centre is one cutter radius from the mean point",
		solved(centre).distance(mean), tr.CutterRadius, 1e-9)

	// 4. THE SPIRAL ANGLE IS REALISED AT THE MEAN POINT. psi is the angle
	// between the trace's tangent there and the cone ELEMENT -- not the cone
	// axis and not the perpendicular to the element, which some looser sources
	// use and which are 90 degrees off.
	radial := mean.minus(solved(centre))
	tangent := pv(-radial.Y, radial.X).direction()
	got := math.Abs(foldAngle(math.Atan2(tangent.Y, tangent.X)))
	if got > math.Pi/2 {
		got = math.Pi - got
	}
	near(t, m.Name+" the spiral angle at the mean point", got, c.SpiralAngle, 1e-9)

	// 5. THE HAND SIGN IS ON THE COS TERM AND NEVER ON THE SIN TERM. Opposite
	// hands mirror the centre across the cone element, y = 0, which flips Cy.
	// Putting the sign on Cx mirrors about x = R_mean instead: a DIFFERENT
	// curve, which gives the two gears unequal twist where an equal-teeth pair
	// must come out as exact mirror images.
	near(t, m.Name+" the centre's along-element coordinate carries no hand sign",
		tr.Centre.X, tr.RMean-tr.CutterRadius*math.Sin(c.SpiralAngle), 1e-12)
	near(t, m.Name+" the centre's circumferential coordinate carries the hand sign",
		tr.Centre.Y, tr.HandSign*tr.CutterRadius*math.Cos(c.SpiralAngle), 1e-12)
	// The pinion is built with the OPPOSITE hand to the driving gear, because
	// the pair meshes.
	other := c.Driving
	if m.Name == "Driving" {
		other = c.Pinion
	}
	if traceFor(c, other).HandSign == tr.HandSign {
		t.Errorf("%s: both members of the pair came out with the same hand", m.Name)
	}
	if c.Pinion.Teeth == c.Driving.Teeth {
		mirror := traceFor(c, other)
		near(t, m.Name+" an equal-teeth pair's two traces are exact mirror images",
			tr.Centre.Y, -mirror.Centre.Y, 1e-12)
		near(t, m.Name+" and differ in nothing else",
			tr.Centre.X, mirror.Centre.X, 1e-12)
	}
}

// ---------------------------------------------------------------------------
// The slab chain: S22 slice, S23 scrap, S24 twist, S25 crown.
// ---------------------------------------------------------------------------

// slab is one cross-section piece of the tooth, between two stations.
type slab struct {
	Index     int
	Near, Far float64 // axial stations, apex-most first
	Twist     float64 // the rotation step S24 gives it
	Crown     float64 // the scale factor step S25 gives it
	Body      *decad.Body
}

// HeelFace is the slab's heel face station: the FARTHEST-along face, which is
// the exact section the loft samples. The twist, the crown and the loft order
// all key on this face and never on the slab's centroid -- centroid keying
// leaves the loft's mid-face section rotated by half a segment.
func (s slab) HeelFace() float64 { return s.Far }

// sliceStations is the cut scheme: a FIXED eight planes, not user-configurable,
// stepped from the parent transverse tooth plane toward the apex in span/6
// increments. The offsets are sign * (k+1) * span/6 for k = 0..7, with the sign
// chosen per gear so they move apex-ward.
func sliceStations(heel, span float64) []float64 {
	out := make([]float64, 0, sliceCount)
	for k := range sliceCount {
		out = append(out, heel-float64(k+1)*span/6)
	}
	return out
}

// slabsOf is the pieces the slice leaves, apex-most first. The apex-side scrap
// is the first of them.
func slabsOf(c bevelCase, m sideMember) []slab {
	f := latticeOf(c, m)
	heel, toe := f.Ded.X, f.Toe.X
	span := heel - toe
	cuts := sliceStations(heel, span)
	// Ascending, so the pieces come out apex-most first.
	sort.Float64s(cuts)

	// The apex end takes the same shrunken stand-in the tooth loft uses: a
	// section at the apex itself has no area, and a loft to a point is not a
	// section pair this evaluator takes.
	edges := append([]float64{apexShrink * heel}, cuts...)
	edges = append(edges, heel)
	out := make([]slab, 0, len(edges)-1)
	for i := 0; i+1 < len(edges); i++ {
		// Crown 1 is "not crowned": the crown step is S25 and every slab leaves
		// the slice at full size.
		out = append(out, slab{Index: i, Near: edges[i], Far: edges[i+1], Crown: 1})
	}
	return out
}

// twistedSlabs applies steps S24 and S25 to the working segments: the twist
// keyed on each slab's heel face and centred on R_mean, then the crown keyed on
// the monotonic heel-distance fraction.
func twistedSlabs(c bevelCase, m sideMember, working []slab) []slab {
	f := latticeOf(c, m)
	tr := traceFor(c, m)
	// Cone distance and axial station differ by one constant factor, which
	// every ratio below cancels, so the law is written on the axial stations
	// this proof builds on.
	heel, toe := f.Ded.X, f.Toe.X
	span := heel - toe
	mean := (heel + toe) / 2

	out := make([]slab, len(working))
	copy(out, working)
	for i := range out {
		out[i].Twist = -tr.HandSign * tr.Total * (mean - out[i].HeelFace()) / span
	}
	// The crown is recomputed from the POST-TWIST heel faces, and the outermost
	// one is the greatest of them. A twist about the shaft axis does not move a
	// station, so the order here is the order above; recomputing is what keeps
	// that true rather than assumed.
	order := make([]int, len(out))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		return out[order[a]].HeelFace() < out[order[b]].HeelFace()
	})
	outermost := order[len(order)-1]
	for i := range out {
		if i == outermost {
			out[i].Crown = 1
			continue
		}
		u := (heel - out[i].HeelFace()) / span
		out[i].Crown = 1 - crownPerRad*(math.Abs(tr.Total)/2)*u
	}
	return out
}

// buildSlab builds one slab as the piece the split would have left: a loft
// between the tooth's own section at each of its two stations, crowned about
// the root point of its heel face, laid apart from its neighbours.
func buildSlab(t *testing.T, doc *decad.Document, w *sketch.World,
	c bevelCase, m sideMember, s slab, heelStation float64) *decad.Body {
	t.Helper()
	section := toothSectionOf(c, m)
	loop := toothChords(section)

	// The crown is a UNIFORM scale about a point on the ROOT EDGE of the heel
	// face, and never about that face's centroid. A uniform scale keeps every
	// line through its base point invariant, so anchoring on the root keeps the
	// root edge on the seating cone and the tooth stays flush; anchoring on the
	// centroid lifts the root by (1-factor) times half the tooth height and the
	// Combine-Join then leaves a gap.
	rootBase := rootEdgeMidpoint(section).times(s.Far / heelStation)

	draw := func(station float64) (*sketch.Sketch, *sketch.Profile) {
		scale := station / heelStation
		local := liftedSketch(t, w, crownedStation(station, s, rootBase, heelStation))
		pts := make([]*sketch.Point, 0, len(loop))
		for _, v := range loop {
			q := crownPoint(v.times(scale), rootBase, s.Crown)
			q = rotatePlane(q, s.Twist)
			pts = append(pts, local.CreatePoint(q.X, q.Y))
		}
		lines := make([]*sketch.Line, 0, len(pts))
		for i := range pts {
			lines = append(lines, local.CreateLine(pts[i], pts[(i+1)%len(pts)]))
		}
		for _, l := range lines {
			local.Fix(l.Start)
			local.Fix(l.End)
		}
		return local, decadtest.SolveRegion(t, local)
	}

	s0, p0 := draw(s.Near)
	s1, p1 := draw(s.Far)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s: slab %d between %.4f and %.4f failed: %v",
			m.Name, s.Index, s.Near, s.Far, err)
	}
	// Laid apart along the axis so no two slabs touch.
	apart, err := r3.Translation(r3.NewVec(0, 0, float64(s.Index+1)*slabLift))
	if err != nil {
		t.Fatalf("%s: laying slab %d apart failed: %v", m.Name, s.Index, err)
	}
	moved, err := body.Placed(apart)
	if err != nil {
		t.Fatalf("%s: laying slab %d apart failed: %v", m.Name, s.Index, err)
	}
	return moved
}

// crownedStation is where a uniform scale about the root base point puts a
// station. The base point sits on the slab's heel face, so that face does not
// move and the toe face pulls toward it.
func crownedStation(station float64, s slab, _ planeVec, _ float64) float64 {
	return s.Far + (station-s.Far)*s.Crown
}

// crownPoint is the uniform scale itself, in the section plane.
func crownPoint(p, base planeVec, factor float64) planeVec {
	return base.plus(p.minus(base).times(factor))
}

func rotatePlane(p planeVec, a float64) planeVec {
	s, c := math.Sin(a), math.Cos(a)
	return pv(p.X*c-p.Y*s, p.X*s+p.Y*c)
}

// rootEdgeMidpoint is the midpoint of the heel face's two ROOT corners: of its
// vertices, the two with the smallest perpendicular distance to the shaft axis.
// The tip corners are the farthest from the axis.
func rootEdgeMidpoint(s toothPolar) planeVec {
	at := func(p polarPt) planeVec {
		return pv(p.Radius*math.Cos(p.Theta), p.Radius*math.Sin(p.Theta))
	}
	return at(s.Right[0]).plus(at(s.Left[0])).times(0.5)
}

// stepSliceTooth cuts the uncut apex-to-heel tooth into cross-section slabs.
//
// THE SLICE MUST ACTUALLY SPLIT THE TOOTH. If the body comes back in one piece
// the offset sign was wrong or the parent plane sits outside the tooth's span,
// and the whole cut is retried once with the opposite sign; still one piece is
// a raised error naming the gear, the piece count, the span and the sign tried.
// Returning an unsliced result is what makes the crown crash far from the
// cause, with `max() iterable argument is empty`, because the scrap drop then
// leaves the segment list empty.
func stepSliceTooth(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so the tooth body is not sliced at all")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	pieces := slabsOf(c, m)
	if len(pieces) < 2 {
		t.Fatalf("%s: the slice produced %d piece(s), expected more than 1 -- the cut "+
			"planes missed (span %.4f)", m.Name, len(pieces), f.Ded.X-f.Toe.X)
	}
	bodies := make([]*decad.Body, 0, len(pieces))
	for _, s := range pieces {
		bodies = append(bodies, buildSlab(t, doc, w, c, m, s, f.Ded.X))
	}
	return bodies
}

func assertSliceTooth(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	pieces := slabsOf(c, m)
	span := f.Ded.X - f.Toe.X

	if len(bodies) != len(pieces) {
		t.Fatalf("%s: built %d slab(s) for %d piece(s)", m.Name, len(bodies), len(pieces))
		return
	}
	// Eight cut planes leave nine pieces, and the count is FIXED: the scheme is
	// not user-configurable.
	if len(pieces) != sliceCount+1 {
		t.Errorf("%s: %d cut planes left %d piece(s), want %d",
			m.Name, sliceCount, len(pieces), sliceCount+1)
	}

	// The offsets step apex-ward from the parent plane in span/6 increments.
	cuts := sliceStations(f.Ded.X, span)
	for k, station := range cuts {
		near(t, m.Name+" cut plane offset",
			f.Ded.X-station, float64(k+1)*span/6, 1e-9)
		if station >= f.Ded.X {
			t.Errorf("%s: cut plane %d at %.4f did not move toward the apex from the "+
				"parent plane at %.4f", m.Name, k, station, f.Ded.X)
		}
	}

	// The pieces tile the tooth end to end, with no gap and no overlap.
	for i, s := range pieces {
		if s.Far <= s.Near {
			t.Errorf("%s: slab %d runs backwards, %.4f to %.4f", m.Name, i, s.Near, s.Far)
		}
		if i > 0 && math.Abs(s.Near-pieces[i-1].Far) > 1e-9 {
			t.Errorf("%s: slab %d starts at %.4f where slab %d ended at %.4f",
				m.Name, i, s.Near, i-1, pieces[i-1].Far)
		}
	}
	near(t, m.Name+" the slabs reach the heel", pieces[len(pieces)-1].Far, f.Ded.X, 1e-9)
	near(t, m.Name+" the slabs start at the apex stand-in",
		pieces[0].Near, apexShrink*f.Ded.X, 1e-12)

	// Each slab is a real body with the volume its own two sections imply.
	section := polygonArea(toothChords(toothSectionOf(c, m)))
	for i, s := range pieces {
		k0, k1 := s.Near/f.Ded.X, s.Far/f.Ded.X
		a0, a1 := section*k0*k0, section*k1*k1
		want := (s.Far - s.Near) * (a0 + a1 + math.Sqrt(a0*a1)) / 3
		v, err := bodies[i].Volume()
		if err != nil {
			t.Fatalf("%s: slab %d volume: %v", m.Name, i, err)
		}
		decadtest.Measures(t, m.Name+" slab volume", v,
			units.CubicMillimeters(want), oracleSlack)
	}
}

// ---------------------------------------------------------------------------
// S23 -- order the segments and drop the apex scrap.
// ---------------------------------------------------------------------------

// stepDropScrap sorts the pieces by the cone distance of their centroid and
// removes the first, the long apex-side scrap below the toe. The list is
// re-sliced BEFORE the scrap is deleted, so the working segments never hold a
// body that has been removed.
//
// AFTER THE DROP THE SEGMENTS MUST BE NON-EMPTY. An empty list means the slice
// failed, and the twist and the crown both assume at least one segment.
func stepDropScrap(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so there is no slab list to trim")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	pieces := slabsOf(c, m)
	sort.Slice(pieces, func(a, b int) bool {
		return (pieces[a].Near+pieces[a].Far)/2 < (pieces[b].Near+pieces[b].Far)/2
	})
	working := pieces[1:]
	if len(working) == 0 {
		t.Fatalf("%s: dropping the apex scrap left no segments; the slice failed",
			m.Name)
	}
	bodies := make([]*decad.Body, 0, len(working))
	for _, s := range working {
		bodies = append(bodies, buildSlab(t, doc, w, c, m, s, f.Ded.X))
	}
	return bodies
}

func assertDropScrap(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	pieces := slabsOf(c, m)
	span := f.Ded.X - f.Toe.X

	if len(bodies) != len(pieces)-1 {
		t.Fatalf("%s: %d segment(s) left of %d piece(s); exactly one scrap is dropped",
			m.Name, len(bodies), len(pieces))
		return
	}
	if len(bodies) == 0 {
		t.Fatalf("%s: the working segments are empty", m.Name)
	}

	// The scrap is the apex-most piece and it is the LONG one: it runs from the
	// apex to the toe-most cut, which is more than a span/6 slab.
	scrap := pieces[0]
	if scrap.Far-scrap.Near <= span/6 {
		t.Errorf("%s: the apex scrap is %.4f long, no longer than a working slab "+
			"at %.4f -- it should reach from the apex to below the toe",
			m.Name, scrap.Far-scrap.Near, span/6)
	}
	if scrap.Far > f.Toe.X {
		t.Errorf("%s: the apex scrap reaches %.4f, past the toe at %.4f",
			m.Name, scrap.Far, f.Toe.X)
	}

	// Every surviving segment sits outside the scrap.
	for i, s := range pieces[1:] {
		if s.Near < scrap.Far-1e-9 {
			t.Errorf("%s: segment %d starts at %.4f, inside the dropped scrap",
				m.Name, i, s.Near)
		}
	}
}

// ---------------------------------------------------------------------------
// S24 -- the twist.
// ---------------------------------------------------------------------------

// stepTwistSegments rotates each segment about the SHAFT AXIS so the tooth
// follows the trace, centred on R_mean so the mid-face section stays unrotated.
// That section then meshes exactly like the straight tooth, which is what lets
// the pinion's mesh phase stay at zero.
func stepTwistSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so no segment is twisted")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	working := slabsOf(c, m)[1:]
	twisted := twistedSlabs(c, m, working)
	bodies := make([]*decad.Body, 0, len(twisted))
	for i := range twisted {
		// The crown belongs to the next step; this one turns the slab only.
		s := twisted[i]
		s.Crown = 1
		bodies = append(bodies, buildSlab(t, doc, w, c, m, s, f.Ded.X))
	}
	return bodies
}

func assertTwistSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	tr := traceFor(c, m)
	working := twistedSlabs(c, m, slabsOf(c, m)[1:])
	heel, toe := f.Ded.X, f.Toe.X
	span := heel - toe
	mean := (heel + toe) / 2

	// THE TOTAL TWIST COMES FROM THE CONJUGATE CROWN-GEAR LAW, not from a
	// measured projection: a spiral bevel is generated by an imaginary flat
	// crown gear, and the work gear's shaft rotation relates to the developed
	// crown-plane azimuth by the roll ratio 1 / sin(gamma).
	phi := math.Atan2(tr.Heel2D.Y, tr.Heel2D.X) - math.Atan2(tr.Toe2D.Y, tr.Toe2D.X)
	near(t, m.Name+" the developed crown azimuth", tr.PhiCrown, phi, 1e-12)
	near(t, m.Name+" the toe-to-heel twist is the azimuth over sin(gamma)",
		tr.Total, math.Abs(phi)/math.Sin(m.Gamma), 1e-12)

	// gamma is the PITCH cone angle and never acos(coneVec . axisDir), which is
	// the root cone angle and yields a twist around 1.6 times too large.
	gammaRoot := m.Gamma - math.Atan2(1.25*c.Module, c.R)
	if wrong := math.Abs(phi) / math.Sin(gammaRoot); math.Abs(wrong-tr.Total) < 1e-9 {
		t.Errorf("%s: the pitch cone angle and the root cone angle give the same "+
			"twist here, so this case cannot tell the two apart", m.Name)
	}

	// The two members of a pair legitimately get DIFFERENT twists: same cutter,
	// same psi, but gamma differs so 1 / sin(gamma) does. That is why any method
	// that gets the roll ratio wrong still meshes an equal-teeth pair and fails
	// a ratio pair.
	other := c.Driving
	if m.Name == "Driving" {
		other = c.Pinion
	}
	otherTrace := traceFor(c, other)
	if c.Pinion.Teeth == c.Driving.Teeth {
		near(t, m.Name+" an equal pair's two twists match in magnitude",
			tr.Total, otherTrace.Total, 1e-9)
	} else if math.Abs(tr.Total-otherTrace.Total) < 1e-9 {
		t.Errorf("%s: an unequal pair came out with the same twist on both members, "+
			"%.6f, so the roll ratio is not being applied per gear", m.Name, tr.Total)
	}

	// THE MID-FACE SECTION STAYS UNROTATED. The law is keyed on the segment's
	// HEEL FACE, not its centroid: the loft samples the heel face, so that face
	// is what has to land at the right azimuth, and centroid keying leaves the
	// mid-face section rotated by half a segment.
	for i, s := range working {
		want := -tr.HandSign * tr.Total * (mean - s.HeelFace()) / span
		near(t, m.Name+" segment twist", s.Twist, want, 1e-12)
		if math.Abs(s.HeelFace()-mean) < 1e-9 && math.Abs(s.Twist) > 1e-9 {
			t.Errorf("%s: segment %d sits at the mean face and was still turned by "+
				"%.6f", m.Name, i, s.Twist)
		}
		centroidKey := -tr.HandSign * tr.Total * (mean - (s.Near+s.Far)/2) / span
		if math.Abs(centroidKey-s.Twist) < 1e-12 {
			t.Errorf("%s: segment %d keys the same on its centroid as on its heel "+
				"face, so this case cannot tell the two apart", m.Name, i)
		}
	}

	// The twist across the whole face is the total the law names.
	first, last := working[0], working[len(working)-1]
	near(t, m.Name+" the twist across the working face",
		math.Abs(last.Twist-first.Twist),
		tr.Total*(last.HeelFace()-first.HeelFace())/span, 1e-9)

	// A rigid rotation changes no volume.
	for i := range bodies {
		v, err := bodies[i].Volume()
		if err != nil {
			t.Fatalf("%s: twisted slab %d volume: %v", m.Name, i, err)
		}
		s := working[i]
		section := polygonArea(toothChords(toothSectionOf(c, m)))
		k0, k1 := s.Near/heel, s.Far/heel
		a0, a1 := section*k0*k0, section*k1*k1
		want := (s.Far - s.Near) * (a0 + a1 + math.Sqrt(a0*a1)) / 3
		decadtest.Measures(t, m.Name+" twisted slab volume", v,
			units.CubicMillimeters(want), oracleSlack)
	}
}

// ---------------------------------------------------------------------------
// S25 -- the lengthwise crown.
// ---------------------------------------------------------------------------

// stepCrownSegments scales every segment except the outermost down by a
// MONOTONIC factor, full at the heel and growing smoothly toward the toe.
func stepCrownSegments(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so no segment is crowned")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	crowned := twistedSlabs(c, m, slabsOf(c, m)[1:])
	bodies := make([]*decad.Body, 0, len(crowned))
	for _, s := range crowned {
		if s.Crown <= 0 {
			t.Fatalf("%s: segment %d came out with a crown factor of %.6f at u %.6f; "+
				"a non-positive factor is never scaled by",
				m.Name, s.Index, s.Crown, (f.Ded.X-s.HeelFace())/(f.Ded.X-f.Toe.X))
		}
		bodies = append(bodies, buildSlab(t, doc, w, c, m, s, f.Ded.X))
	}
	return bodies
}

func assertCrownSegments(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	tr := traceFor(c, m)
	crowned := twistedSlabs(c, m, slabsOf(c, m)[1:])
	heel, toe := f.Ded.X, f.Toe.X
	span := heel - toe

	// THE OUTERMOST SEGMENT IS HELD FULL. Its heel face is the loft's heel end
	// and has to stay full so the heel cone trims it flush with the gear base.
	outermost := 0
	for i := range crowned {
		if crowned[i].HeelFace() > crowned[outermost].HeelFace() {
			outermost = i
		}
	}
	near(t, m.Name+" the outermost segment is held full", crowned[outermost].Crown, 1, 0)

	// THE RELIEF IS KEYED ON THE MONOTONIC HEEL DISTANCE u, never on |ang|.
	// Keying on the twist magnitude is symmetric about the mid face -- maximal
	// at BOTH ends -- so with the heel slab held full the slab just inside it
	// becomes the most relieved one and dips below both its neighbours. That
	// was the observed bug: 0.932 next to the heel against 0.972 one further
	// in, a notch that reverses the heel-to-toe taper.
	for i, s := range crowned {
		if i == outermost {
			continue
		}
		u := (heel - s.HeelFace()) / span
		near(t, m.Name+" crown factor", s.Crown,
			1-crownPerRad*(math.Abs(tr.Total)/2)*u, 1e-12)
		if s.Crown <= 0 {
			t.Errorf("%s: segment %d has a non-positive crown factor %.6f",
				m.Name, i, s.Crown)
		}
		if s.Crown > 1 {
			t.Errorf("%s: segment %d was crowned UP, to %.6f", m.Name, i, s.Crown)
		}
	}

	// Monotonic from the held-full heel to the toe: sorted by heel-face
	// station, the factors never rise as the station falls.
	order := make([]int, len(crowned))
	for i := range order {
		order[i] = i
	}
	sort.Slice(order, func(a, b int) bool {
		return crowned[order[a]].HeelFace() > crowned[order[b]].HeelFace()
	})
	for k := 1; k < len(order); k++ {
		if crowned[order[k]].Crown > crowned[order[k-1]].Crown+1e-12 {
			t.Errorf("%s: the crown is not monotonic heel to toe: the segment at "+
				"%.4f reads %.6f against %.6f at %.4f just outside it -- that is the "+
				"notch that reverses the taper", m.Name,
				crowned[order[k]].HeelFace(), crowned[order[k]].Crown,
				crowned[order[k-1]].Crown, crowned[order[k-1]].HeelFace())
		}
	}

	// The maximum relief is now at the TOE and keeps the magnitude the old
	// per-end peak had.
	near(t, m.Name+" the peak relief is the per-end twist magnitude",
		1-crowned[order[len(order)-1]].Crown,
		crownPerRad*(math.Abs(tr.Total)/2)*(heel-crowned[order[len(order)-1]].HeelFace())/span,
		1e-12)
	near(t, m.Name+" the crown constant", crownPerRad, 0.5, 0)

	// THE ROOT EDGE STAYS PUT. A uniform scale keeps every line through its
	// base point invariant, so anchoring on the heel face's root-edge midpoint
	// keeps the root on the seating cone while the tip is relieved. Anchoring
	// on the face's CENTROID instead lifts the root by (1-factor) times half
	// the tooth height, the tooth floats off the gear base, and the join leaves
	// a gap -- clearly visible on a ratio pair.
	section := toothSectionOf(c, m)
	base := rootEdgeMidpoint(section)
	centroidAnchor := pv(0, 0)
	for _, v := range toothChords(section) {
		centroidAnchor = centroidAnchor.plus(v)
	}
	centroidAnchor = centroidAnchor.times(1 / float64(len(toothChords(section))))
	for i, s := range crowned {
		if i == outermost {
			continue
		}
		scale := s.Far / heel
		anchored := crownPoint(base.times(scale), base.times(scale), s.Crown)
		near(t, m.Name+" the root edge does not move under the crown",
			anchored.distance(base.times(scale)), 0, 1e-12)
		lifted := crownPoint(base.times(scale), centroidAnchor.times(scale), s.Crown)
		if lifted.distance(base.times(scale)) <= 1e-9 {
			t.Errorf("%s: segment %d cannot tell a root anchor from a centroid one, "+
				"so this case does not cover the gap the centroid anchor leaves",
				m.Name, i)
		}
	}
	if len(bodies) != len(crowned) {
		t.Errorf("%s: built %d crowned slab(s) for %d segment(s)",
			m.Name, len(bodies), len(crowned))
	}
}

// ---------------------------------------------------------------------------
// S26 -- the spiral loft.
// ---------------------------------------------------------------------------

// stepSpiralLoft lofts the curved tooth through the segments' faces, in the
// order the POST-TWIST heel-face cone distance gives.
//
// THE SEGMENTS ARE RE-SORTED HERE, after the twist and the crown, and the
// pre-twist slice order is not reused. The twist rotates each slab about the
// shaft axis, and for a high-twist unequal-ratio pair that rotation can change
// the slabs' along-cone order enough to reorder adjacent ones; lofting in the
// stale order then assembles the cross-sections out of sequence and the crowned
// tooth comes out distorted. For an equal or low-twist pair the two orders
// coincide, which is exactly why an equal-teeth pair meshes even with the stale
// order while a ratio pair distorts.
//
// decad's loft takes TWO sections, so the chain is built pairwise and the
// pieces are laid apart. THE COST IS THE SINGLE BODY: this does not show the
// evaluator running one loft through every section.
func stepSpiralLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so the straight tooth is kept and nothing "+
			"is lofted through slabs")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	segments := loftOrder(c, m)
	bodies := make([]*decad.Body, 0, len(segments))
	for _, s := range segments {
		bodies = append(bodies, buildSlab(t, doc, w, c, m, s, f.Ded.X))
	}
	return bodies
}

// loftOrder is the segments sorted by their POST-TWIST heel-face station, which
// is the order the loft consumes them in.
func loftOrder(c bevelCase, m sideMember) []slab {
	segments := twistedSlabs(c, m, slabsOf(c, m)[1:])
	sort.SliceStable(segments, func(a, b int) bool {
		return segments[a].HeelFace() < segments[b].HeelFace()
	})
	return segments
}

func assertSpiralLoft(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	segments := loftOrder(c, m)

	if len(bodies) != len(segments) {
		t.Fatalf("%s: lofted %d section(s) for %d segment(s)",
			m.Name, len(bodies), len(segments))
		return
	}

	// The order is strictly ascending in the heel-face station, and the first
	// section the loft takes is the TOE-most segment's apex-side face, which is
	// what pushes the loft past the toe cone so the toe trim bites.
	for i := 1; i < len(segments); i++ {
		if segments[i].HeelFace() <= segments[i-1].HeelFace() {
			t.Errorf("%s: the loft order is not ascending: %.4f follows %.4f",
				m.Name, segments[i].HeelFace(), segments[i-1].HeelFace())
		}
	}
	if segments[0].Near > f.Toe.X+1e-9 {
		t.Errorf("%s: the loft's first section sits at %.4f, inside the toe at %.4f, "+
			"so it would not push past the toe cone", m.Name, segments[0].Near, f.Toe.X)
	}
	near(t, m.Name+" the loft's last section reaches the heel",
		segments[len(segments)-1].Far, f.Ded.X, 1e-9)

	// A slab end face is found by the ALL-FACES-BY-CENTROID rule -- the greatest
	// along-element centroid is the heel face and the least the toe face --
	// searched across ALL of the slab's faces with NO surface-type filter. A
	// sliced slab in Fusion is bounded by a mix of the two planar cut faces and
	// ruled side faces, and a PlaneSurfaceType filter can pick the wrong face or
	// miss the cut face entirely, which makes this loft fail with
	// ASM_NOT_ALL_SECTIONS_MEET.
	//
	// THE BENCH CANNOT SHOW THAT FILTER PICKING THE WRONG FACE. The tooth's
	// flanks are chorded into line segments here, because decad's loft refuses a
	// free-form pair, so every face of a slab comes out planar and a
	// surface-type filter would select them all. What the bench does show is the
	// other half of the rule: the two cut faces are there, they are
	// distinguishable, and the one the loft samples is the heel face.
	section := polygonArea(toothChords(toothSectionOf(c, m)))
	for i, body := range bodies {
		s := segments[i]
		faces := body.Faces()
		if len(faces) < 3 {
			t.Errorf("%s: slab %d has %d face(s); a cross-section slab is bounded by "+
				"two cut faces and its sides", m.Name, i, len(faces))
			continue
		}
		k0, k1 := s.Near/f.Ded.X, s.Far/f.Ded.X
		wantToe := section * k0 * k0 * s.Crown * s.Crown
		// The crown is a UNIFORM scale, so it shrinks the heel face too; what it
		// leaves in place is the root-edge point it is anchored on, not the face.
		wantHeel := section * k1 * k1 * s.Crown * s.Crown
		var toeFaces, heelFaces int
		for _, face := range faces {
			area, err := face.Area()
			if err != nil {
				t.Fatalf("%s: slab %d face area: %v", m.Name, i, err)
			}
			got := area.Value.Base()
			if math.Abs(got-wantToe) <= math.Max(1e-6, wantToe*1e-6) {
				toeFaces++
			}
			if math.Abs(got-wantHeel) <= math.Max(1e-6, wantHeel*1e-6) {
				heelFaces++
			}
		}
		if toeFaces != 1 || heelFaces != 1 {
			t.Errorf("%s: slab %d carries %d toe face(s) and %d heel face(s) of the "+
				"sections it was built from; the rule that finds them has to pick "+
				"exactly one of each", m.Name, i, toeFaces, heelFaces)
		}
		if wantHeel <= wantToe {
			t.Errorf("%s: slab %d's heel face (%.6f mm^2) is not larger than its toe "+
				"face (%.6f mm^2), so the farthest-along face is not the heel one",
				m.Name, i, wantHeel, wantToe)
		}
	}
}

// ---------------------------------------------------------------------------
// S27 -- the spiral flush trim.
// ---------------------------------------------------------------------------

// stepSpiralTrim returns the curved tooth through the SAME toe-then-heel
// two-cone trim the straight tooth takes, so the curved tooth's ends sit flush
// on the gear base. Neither cut is performed, for the reason solids_test.go
// opens with; the operands are built and laid apart.
func stepSpiralTrim(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	if straightBranch(c) {
		proofkit3d.Unmodelled(t, "psi is 0, so the straight tooth's own trims run "+
			"instead and this step is not reached")
	}
	f := latticeOf(c, m)
	w := sketch.NewWorld()

	// The curved tooth, as the outermost two segments leave it: the heel end is
	// held full and is what the heel cone trims.
	segments := loftOrder(c, m)
	tooth := buildSlab(t, doc, w, c, m, segments[len(segments)-1], f.Ded.X)

	bands := bandsOf(c, m)
	toe := buildBand(t, doc, w, band{Name: "spiral toe cone",
		Near: bands[2].Near, Far: bands[2].Far, Lift: 7000})
	heel := buildBand(t, doc, w, band{Name: "spiral heel cone",
		Near: bands[0].Near, Far: bands[0].Far, Lift: 8000})
	return []*decad.Body{tooth, toe.Body, heel.Body}
}

func assertSpiralTrim(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	c := resolveCase(t, p)
	m := memberFor(c, p)
	f := latticeOf(c, m)
	bands := bandsOf(c, m)
	segments := loftOrder(c, m)

	if len(bodies) != 3 {
		t.Fatalf("%s: the spiral trim substitution built %d body(ies), want the "+
			"curved tooth and two cones", m.Name, len(bodies))
		return
	}

	// The two cones are the SAME two the straight tooth is trimmed with, so the
	// flush band they leave is the same band.
	apexOf := func(b band) (apex, slope float64) {
		slope = b.Slope()
		return b.Near.Station - b.Near.Radius/slope, slope
	}
	toeApex, toeSlope := apexOf(bands[2])
	heelApex, heelSlope := apexOf(bands[0])
	section := toothSectionOf(c, m)
	rootSlope := section.RootRadius / f.Ded.X
	crossing := func(coneSlope, coneApex, surfaceSlope float64) float64 {
		return -coneSlope * coneApex / (surfaceSlope - coneSlope)
	}
	near(t, m.Name+" the spiral toe cut lands at the flush band's toe end",
		crossing(toeSlope, toeApex, rootSlope), f.Toe.X, c.RootSink)
	near(t, m.Name+" the spiral heel cut lands at the flush band's heel end",
		crossing(heelSlope, heelApex, rootSlope), f.Ded.X, c.RootSink)

	// The heel segment reaches the heel cone at full height, which is why it is
	// the one the crown holds full.
	outermost := segments[len(segments)-1]
	near(t, m.Name+" the heel segment is uncrowned", outermost.Crown, 1, 0)
	near(t, m.Name+" the heel segment reaches the heel", outermost.Far, f.Ded.X, 1e-9)

	// THE MESH PHASE IS NOT APPLIED HERE. The toe and heel phasing is the
	// caller's mesh-rotate step, and the pinion's extra phase stays zero
	// because the mid-face section was left unrotated and already meshes.
	near(t, m.Name+" the spiral leaves the pinion's mesh phase alone",
		pinionMeshPhaseTeeth, 0, 0)
}
