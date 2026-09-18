// Package cycloidal_test proves the cycloidal drive's build, one function per
// step of spec/cycloidal/steps.md.
//
// Units. The generator works in Fusion-internal centimetres and converts the
// dialog's millimetres with misc.to_cm. This proof works in millimetres from
// end to end, because millimetre is the base unit of both engines: sketch
// solves in plane-local millimetres and decad reads every position as a
// millimetre coordinate. Nothing proven here depends on the choice — every
// relation the spec states is a ratio of lengths or a count — so the proof
// states the dialog's own numbers and never converts.
//
// What this file holds is the gear's arithmetic: the point function of
// epitrochoid-trace.md, the adaptive sampling that draws it, the swept
// envelope the pinless casing contours to, the undercut guard, and the
// parameter cases every step is proven against. The steps themselves are in
// sketches_test.go and solids_test.go.
package cycloidal_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The dialog input ids, used as the case tables' parameter keys so a case reads
// as the dialog the user fills in. `disc` is not a dialog input: it is the disc
// index d of spec/cycloidal/instructions.md §0, which selects the per-disc
// centre sign and clocking a case is proven at.
const (
	keyPinCount            = "pinCount"
	keyPinCircleDiameter   = "pinCircleDiameter"
	keyPinDiameter         = "pinDiameter"
	keyEccentricity        = "eccentricity"
	keyDiskClearance       = "diskClearance"
	keyDiscThickness       = "discThickness"
	keyDiscGap             = "discGap"
	keyCenterBearingDia    = "centerBearingDiameter"
	keyInputShaftDiameter  = "inputShaftDiameter"
	keyBearingClearance    = "bearingClearance"
	keyOutputPinCircleDia  = "outputPinCircleDiameter"
	keyOutputPinCount      = "outputPinCount"
	keyOutputPinDiameter   = "outputPinDiameter"
	keyWall                = "wall"
	keyBaseThickness       = "baseThickness"
	keyOutputPlateThicknes = "outputPlateThickness"
	keyChamferSize         = "chamferSize"
	keyDiscCount           = "discCount"
	keyDisc                = "disc"
)

// The sampling constants epitrochoid-trace.md pins. They are named here because
// every one of them is a number the spec fixes rather than a number this proof
// is free to choose: a different fine resolution, turn threshold, sweep count
// or bin count draws a different curve.
const (
	fineSteps    = 2000                // "Sampling" step 1, and the curvature scan
	turnLimit    = 5.0 * math.Pi / 180 // "Sampling" step 2, exactly 5.0 degrees
	sweepSteps   = 240                 // "Pinless ring casing", N-theta = N-t = 240
	contourBins  = 80                  // "Pinless ring casing", nbins = 80
	bisectRounds = 40                  // the undercut bound's 40 bisection rounds
)

// point is a plane-local millimetre coordinate pair, the same carve-out decad
// makes for positions.
type point struct{ X, Y float64 }

// dims is one case's resolved dimension set: what _resolveDimensions stashes
// before any geometry is drawn.
type dims struct {
	N, L, M         int
	R, E, C         float64
	Rr, RrEff, Rv   float64
	Rop             float64
	DPin, DHole     float64
	T, G            float64
	CBD, ISD, Clr   float64
	Wall, BaseT     float64
	PlateT, Chamfer float64
	D               int
	Disc            int
	Sign            float64 // s_d: +1 for disc 0, -1 for disc 1
	Phi             float64 // phi_d = d*pi
}

// derive resolves a case's dialog values exactly as the spec's resolve order
// does: the auto-versus-override branches first, then everything computed from
// them.
func derive(p map[string]float64) dims {
	var d dims
	d.N = int(math.Round(p[keyPinCount]))
	d.L = d.N - 1
	d.M = int(math.Round(p[keyOutputPinCount]))
	d.R = p[keyPinCircleDiameter] / 2
	d.E = p[keyEccentricity]
	d.C = p[keyDiskClearance]
	if pd := p[keyPinDiameter]; pd > 0 {
		d.Rr = pd / 2
	} else {
		d.Rr = 0.5 * (d.E + d.R*math.Sin(math.Pi/float64(d.N)))
	}
	d.RrEff = d.Rr + d.C
	d.Rv = d.R - d.RrEff - d.E
	d.Rop = p[keyOutputPinCircleDia] / 2
	if op := p[keyOutputPinDiameter]; op > 0 {
		d.DPin = op
	} else {
		d.DPin = d.Rop*math.Sin(math.Pi/float64(d.M)) - d.E
	}
	d.DHole = d.DPin + 2*d.E
	d.T = p[keyDiscThickness]
	d.G = p[keyDiscGap]
	d.CBD = p[keyCenterBearingDia]
	d.ISD = p[keyInputShaftDiameter]
	d.Clr = p[keyBearingClearance]
	d.Wall = p[keyWall]
	d.BaseT = p[keyBaseThickness]
	d.PlateT = p[keyOutputPlateThicknes]
	d.Chamfer = p[keyChamferSize]
	d.D = int(math.Round(p[keyDiscCount]))
	if d.D < 1 {
		d.D = 1
	}
	d.Disc = int(math.Round(p[keyDisc]))
	d.Sign = 1
	if d.Disc == 1 {
		d.Sign = -1
	}
	d.Phi = math.Pi * float64(d.Disc)
	return d
}

// centre is Od_d = O + s_d*E*Xhat, the eccentric disc centre every disc-owned
// feature is built on.
func (d dims) centre() point { return point{d.Sign * d.E, 0} }

// housingOuterRadius is the pinless outer wall: the contour peak at
// R - PinRadius + 2E cleared by Wall.
func (d dims) housingOuterRadius() float64 { return d.R - d.Rr + 2*d.E + d.Wall }

// housingInnerRadius is the base annulus's inner floor lip, Wall inside the
// contour valley at R - PinRadius.
func (d dims) housingInnerRadius() float64 { return d.R - d.Rr - d.Wall }

// plateRadius is half OutputPlateDiameter = 2*Rop + D_pin + 2*Wall.
func (d dims) plateRadius() float64 { return d.Rop + d.DPin/2 + d.Wall }

// boreRadius is the disc's centre bore, the cam outer enlarged by the running
// clearance.
func (d dims) boreRadius() float64 { return (d.CBD + d.Clr) / 2 }

// stackTop is (D-1)*(T+g) + T, the top of the disc stack above the target plane.
func (d dims) stackTop() float64 { return float64(d.D-1)*(d.T+d.G) + d.T }

// discBase is z_d = d*(T+g), where disc d's own plane sits.
func (d dims) discBase() float64 { return float64(d.Disc) * (d.T + d.G) }

// camSectionHeight is T + g for every section but the last, which is T.
func (d dims) camSectionHeight() float64 {
	if d.Disc < d.D-1 {
		return d.T + d.G
	}
	return d.T
}

// diskPoint is epitrochoid-trace.md's point function, reproduced exactly: the
// equidistant of a shortened epitrochoid, offset along the curve normal by
// Rr_eff = Rr + c, for a rotor centred at (cx, cy) and clocked by phi.
func diskPoint(d dims, t, cx, cy, phi float64) point {
	n := float64(d.N)
	num := math.Sin((1 - n) * t)
	den := d.R/(d.E*n) - math.Cos((1-n)*t)
	psi := math.Atan2(num, den) // uses R, E, N only — not Rr
	x0 := d.R*math.Cos(t) - d.RrEff*math.Cos(t+psi) - d.E*math.Cos(n*t)
	y0 := -d.R*math.Sin(t) + d.RrEff*math.Sin(t+psi) + d.E*math.Sin(n*t)
	return point{
		X: cx + x0*math.Cos(phi) - y0*math.Sin(phi),
		Y: cy + x0*math.Sin(phi) + y0*math.Cos(phi),
	}
}

// lobeSamples is one lobe, t in [0, 2*pi/L], sampled by bounded turn angle.
//
// The fine trace is exactly fineSteps uniform steps; a point is kept whenever
// the accumulated direction change since the last kept point reaches turnLimit,
// and the first and last are always kept, so the two ends land exactly on t = 0
// and t = 2*pi/L — the two valleys the root circle pins.
func lobeSamples(d dims, cx, cy, phi float64) []point {
	span := 2 * math.Pi / float64(d.L)
	fine := make([]point, fineSteps+1)
	for i := range fine {
		fine[i] = diskPoint(d, span*float64(i)/float64(fineSteps), cx, cy, phi)
	}
	kept := []point{fine[0]}
	acc := 0.0
	for i := 1; i < fineSteps; i++ {
		a := math.Atan2(fine[i].Y-fine[i-1].Y, fine[i].X-fine[i-1].X)
		b := math.Atan2(fine[i+1].Y-fine[i].Y, fine[i+1].X-fine[i].X)
		acc += math.Abs(math.Atan2(math.Sin(b-a), math.Cos(b-a)))
		if acc >= turnLimit {
			kept = append(kept, fine[i])
			acc = 0
		}
	}
	return append(kept, fine[fineSteps])
}

// fullLobeSamples is the whole rotor boundary: the one lobe of lobeSamples
// repeated L times, each copy turned by -2*pi/L about the disc centre, which is
// the boundary the L patterned sectors tile. The shared valley point between
// two lobes is emitted once, so the result is a simple closed polygon.
func fullLobeSamples(d dims, cx, cy, phi float64) []point {
	one := lobeSamples(d, 0, 0, 0)
	out := make([]point, 0, (len(one)-1)*d.L)
	for k := range d.L {
		a := phi - 2*math.Pi*float64(k)/float64(d.L)
		for _, q := range one[:len(one)-1] {
			out = append(out, point{
				X: cx + q.X*math.Cos(a) - q.Y*math.Sin(a),
				Y: cy + q.X*math.Sin(a) + q.Y*math.Cos(a),
			})
		}
	}
	return out
}

// contourPitch is the casing's inner wall over one pin pitch: the disc's swept
// envelope env(phi) offset outward by the clearance c, emitted at bin EDGES.
//
// The edges matter and the spec says why: a point set emitted at bin centres
// insets the two ends by half a bin, so adjacent patterned sectors do not touch
// and the Join leaves N loose bodies. Emitting at edges puts the first point
// exactly on -pi/N and the last exactly on +pi/N, which stepPatternJoinCasing
// measures.
func contourPitch(d dims) []point {
	half := math.Pi / float64(d.N)
	binMax := make([]float64, contourBins)
	hit := make([]bool, contourBins)
	for i := range sweepSteps {
		th := 2 * math.Pi * float64(i) / float64(sweepSteps)
		cx, cy := d.E*math.Cos(th), d.E*math.Sin(th)
		clock := -th / float64(d.L)
		for j := range sweepSteps {
			q := diskPoint(d, 2*math.Pi*float64(j)/float64(sweepSteps), cx, cy, clock)
			a := math.Atan2(q.Y, q.X)
			if a < -half || a > half {
				continue
			}
			b := int((a + half) / (2 * half) * float64(contourBins))
			b = min(max(b, 0), contourBins-1)
			if r := math.Hypot(q.X, q.Y); !hit[b] || r > binMax[b] {
				binMax[b], hit[b] = r, true
			}
		}
	}
	out := make([]point, contourBins+1)
	for i := range contourBins + 1 {
		ang := -half + 2*half*float64(i)/float64(contourBins)
		peak := 0.0 // both neighbours unhit leaves the edge at radius c
		if i > 0 && hit[i-1] {
			peak = math.Max(peak, binMax[i-1])
		}
		if i < contourBins && hit[i] {
			peak = math.Max(peak, binMax[i])
		}
		r := d.C + peak
		out[i] = point{r * math.Cos(ang), r * math.Sin(ang)}
	}
	return out
}

// fullContour is the casing's whole inner wall: contourPitch turned through the
// N pin pitches, the shared end point emitted once, so the N sectors' contours
// form one simple closed polygon.
func fullContour(d dims) []point {
	pitch := contourPitch(d)
	out := make([]point, 0, (len(pitch)-1)*d.N)
	for k := range d.N {
		a := 2 * math.Pi * float64(k) / float64(d.N)
		for _, q := range pitch[:len(pitch)-1] {
			out = append(out, point{
				X: q.X*math.Cos(a) - q.Y*math.Sin(a),
				Y: q.X*math.Sin(a) + q.Y*math.Cos(a),
			})
		}
	}
	return out
}

// rhoMinTowardO is the base trochoid's smallest radius of curvature at the
// points whose centre of curvature lies toward O — epitrochoid-trace.md's
// "No-undercut guard", scanned at exactly fineSteps points over [0, 2*pi).
//
// The drawn profile is the inward equidistant of that trochoid offset by
// Rr_eff, and an inward offset overruns itself once Rr_eff reaches this radius,
// so Rr_eff < rhoMinTowardO is the binding eccentricity limit — tighter than
// E < R/N.
func rhoMinTowardO(d dims) float64 {
	n := float64(d.N)
	best := math.Inf(1)
	for i := range fineSteps {
		t := 2 * math.Pi * float64(i) / float64(fineSteps)
		bx := d.R*math.Cos(t) - d.E*math.Cos(n*t)
		by := -d.R*math.Sin(t) + d.E*math.Sin(n*t)
		xp := -d.R*math.Sin(t) + d.E*n*math.Sin(n*t)
		yp := -d.R*math.Cos(t) + d.E*n*math.Cos(n*t)
		xpp := -d.R*math.Cos(t) + d.E*n*n*math.Cos(n*t)
		ypp := d.R*math.Sin(t) - d.E*n*n*math.Sin(n*t)
		k := xp*ypp - yp*xpp
		if math.Abs(k) < 1e-12 {
			continue
		}
		s := math.Hypot(xp, yp)
		rho := math.Pow(s, 3) / k
		cx := bx + rho*(-yp/s)
		cy := by + rho*(xp/s)
		if cx*cx+cy*cy < bx*bx+by*by {
			best = math.Min(best, math.Abs(rho))
		}
	}
	return best
}

// undercutLimit is E*, the largest eccentricity the guard still admits with
// every other input held, found by exactly bisectRounds bisections over
// (0, hi]. Both Rr_eff and rhoMinTowardO move with E' when Pin Diameter is 0,
// so each round re-resolves the whole dimension set.
//
// The spec brackets the search at the offending eccentricity itself, because it
// only runs the bisection once the guard has already failed and needs a number
// below that value to put in the message. A proof that ran the same bracket on
// a valid dialog would get the dialog's own eccentricity back and learn
// nothing, so the bracket is a parameter here and every caller states one wide
// enough to hold the real bound.
func undercutLimit(p map[string]float64, hi float64) float64 {
	lo := 0.0
	for range bisectRounds {
		mid := (lo + hi) / 2
		t := atEccentricity(p, mid)
		if t.RrEff < rhoMinTowardO(t) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

// atEccentricity resolves a case's dimensions at a different eccentricity,
// every other dialog value held, which is what the undercut guard's bound is
// searched over.
func atEccentricity(p map[string]float64, e float64) dims {
	q := make(map[string]float64, len(p))
	for k, v := range p {
		q[k] = v
	}
	q[keyEccentricity] = e
	return derive(q)
}

// polygonArea is the shoelace area of a closed polygon, positive for either
// winding.
func polygonArea(pts []point) float64 {
	sum := 0.0
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		sum += p.X*q.Y - q.X*p.Y
	}
	return math.Abs(sum) / 2
}

// polygonBounds is the axis-aligned extent of a point set, which is the
// bounding box of the prism extruded through it.
func polygonBounds(pts []point) (lo, hi point) {
	lo = point{math.Inf(1), math.Inf(1)}
	hi = point{math.Inf(-1), math.Inf(-1)}
	for _, p := range pts {
		lo.X, lo.Y = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y)
		hi.X, hi.Y = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y)
	}
	return lo, hi
}

// -- shared fixtures ---------------------------------------------------------

// groundedSketch draws the anchor chain every cycloidal sketch opens with: the
// user's Anchor arrives as reference geometry, since Fusion projects it in
// rather than authoring it, and a fresh local origin is constrained coincident
// to it. Everything else in the sketch is drawn relative to that origin.
func groundedSketch(t testing.TB, s *sketch.Sketch) *sketch.Point {
	t.Helper()
	anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
	anchor.SetName("projected Anchor")
	origin := s.CreatePoint(0, 0)
	origin.SetName("local origin O")
	s.AddConstraint(sketch.NewCoincident(origin, anchor))
	return origin
}

// eccentricCentre builds Od_d off the local origin: a horizontal construction
// line and a driving distance dimension carrying the Eccentricity.
//
// The dimension is a SIGNED horizontal distance where Fusion's
// addDistanceDimension is an unsigned magnitude whose side comes from the seed.
// That substitution is required, not cosmetic: an unsigned distance plus a
// horizontal leaves Od at +E or -E, two configurations the probe finds and
// RequireSound refuses. [PB-DIM-VALUE-SEMANTICS] is the rule that licenses it —
// the engine's target is signed, and the sign crosses over to Fusion as the
// seeded side, never as a negative parameter value.
func eccentricCentre(t testing.TB, s *sketch.Sketch, origin *sketch.Point, d dims) (*sketch.Point, *sketch.Line) {
	t.Helper()
	c := d.centre()
	centre := s.CreatePoint(c.X, c.Y)
	centre.SetName("disc centre Od")
	ecc := s.CreateLine(origin, centre)
	ecc.SetConstruction(true)
	s.AddConstraint(
		sketch.NewHorizontal(ecc),
		sketch.NewHorizontalDistance(origin, centre, d.Sign*d.E),
	)
	return centre, ecc
}

// circleOn draws a circle whose centre is a fresh point coincident to at, with
// a driving diameter dimension — the addByCenterRadius + addCoincident +
// addDiameterDimension shape every reference circle in this gear uses. Fusion
// creates a free centre point and constrains it ([PB-CIRCLE-CENTER]), so the
// proof does the same rather than sharing the point ([PB-SHARE-XOR-COINCIDENT]).
func circleOn(t testing.TB, s *sketch.Sketch, at *sketch.Point, r float64, name string, construction bool) *sketch.Circle {
	t.Helper()
	c := s.CreatePoint(at.X(), at.Y())
	circle := s.CreateCircle(c, r)
	circle.SetName(name)
	circle.SetConstruction(construction)
	s.AddConstraint(sketch.NewCoincident(c, at), sketch.NewDiameter(circle, 2*r))
	return circle
}

// polyline draws a chain of lines through pts and returns their shared points.
// A closed chain joins the last point back to the first.
func polyline(s *sketch.Sketch, pts []point, closed bool) []*sketch.Point {
	sp := make([]*sketch.Point, len(pts))
	for i, q := range pts {
		sp[i] = s.CreatePoint(q.X, q.Y)
	}
	last := len(sp) - 1
	for i := range last {
		s.CreateLine(sp[i], sp[i+1])
	}
	if closed {
		s.CreateLine(sp[last], sp[0])
	}
	return sp
}

// -- solid fixtures ----------------------------------------------------------

// prismFromPolygon extrudes a closed polygon from z0 by height. The polygon's
// points are grounded, which is what decadtest.SolveRegion needs to hand back
// exactly one valid region; nothing about the constraint scheme is being proven
// here — sketches_test.go owns that — so the shape is stated rather than solved.
func prismFromPolygon(t *testing.T, doc *decad.Document, pts []point, z0, height float64, label string) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	for _, p := range polyline(s, pts, true) {
		s.Fix(p)
	}
	profile := decadtest.SolveRegion(t, s)
	body := decadtest.NewPrism(t, doc, s, profile, units.Millimeters(height))
	return liftTo(t, body, z0, label)
}

// hole is one circular opening in an extruded profile.
type hole struct {
	CX, CY, R float64
}

// prismWithHoles extrudes a closed polygon carrying circular holes, in one
// feature. It is how a step whose Fusion feature is a repeated cut is proven:
// decad refuses a second boolean against the mesh the first one left, so the
// openings are stated as holes in the profile and the resulting solid — which
// is the same solid — is read exactly rather than to a facet chord.
func prismWithHoles(t *testing.T, doc *decad.Document, pts []point, holes []hole, z0, height float64, label string) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	for _, q := range polyline(s, pts, true) {
		s.Fix(q)
	}
	for _, h := range holes {
		c := s.CreatePoint(h.CX, h.CY)
		s.Fix(c)
		circle := s.CreateCircle(c, h.R)
		s.AddConstraint(sketch.NewDiameter(circle, 2*h.R))
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("%s: solve the holed profile: %v", label, err)
	}
	var found *sketch.Profile
	for _, prof := range s.Profiles() {
		if len(prof.Holes) != len(holes) || !prof.Valid {
			continue
		}
		if found != nil {
			t.Fatalf("%s: more than one region carries %d hole(s)", label, len(holes))
		}
		found = prof
	}
	if found == nil {
		t.Fatalf("%s: no valid region carries %d hole(s); the openings may cross the boundary",
			label, len(holes))
	}
	body, err := doc.Extrude(s, found, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	if err != nil {
		t.Fatalf("%s: extrude the holed profile: %v", label, err)
	}
	return liftTo(t, body, z0, label)
}

// cylinder extrudes a disc of radius r about (cx, cy) from z0 by height.
func cylinder(t *testing.T, doc *decad.Document, cx, cy, r, z0, height float64, label string) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	circle := s.CreateCircle(c, r)
	s.AddConstraint(sketch.NewDiameter(circle, 2*r))
	profile := decadtest.SolveRegion(t, s)
	body := decadtest.NewPrism(t, doc, s, profile, units.Millimeters(height))
	return liftTo(t, body, z0, label)
}

// annulus extrudes the ring between two circles. The outer circle sits on
// (ox, oy) and the inner on (ix, iy), so an eccentric cam section is the same
// call as a concentric housing base. The ring is selected by its single hole,
// which is the engine's reading of the spec's profileLoops.count == 2 rule.
func annulus(t *testing.T, doc *decad.Document, ox, oy, ro, ix, iy, ri, z0, height float64, label string) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	oc := s.CreatePoint(ox, oy)
	s.Fix(oc)
	outer := s.CreateCircle(oc, ro)
	ic := s.CreatePoint(ix, iy)
	s.Fix(ic)
	inner := s.CreateCircle(ic, ri)
	s.AddConstraint(sketch.NewDiameter(outer, 2*ro), sketch.NewDiameter(inner, 2*ri))
	profile := holedProfile(t, s)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	if err != nil {
		t.Fatalf("%s: extrude the annulus: %v", label, err)
	}
	return liftTo(t, body, z0, label)
}

// holedProfile solves the sketch and returns its one region with a hole. Two
// nested closed curves detect as two regions — the inner disc and the ring —
// and it is the ring that every annular extrude in this gear wants.
func holedProfile(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("solve the annular sketch: %v", err)
	}
	var found *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) != 1 || !p.Valid {
			continue
		}
		if found != nil {
			t.Fatalf("the sketch holds more than one valid holed region")
		}
		found = p
	}
	if found == nil {
		t.Fatalf("the sketch holds no valid holed region")
	}
	return found
}

// liftTo moves a body so its extrude base sits at z0, and names it, so a
// decadtest failure opens with the step's own name for the body rather than an
// index and a recipe step.
func liftTo(t *testing.T, body *decad.Body, z0 float64, label string) *decad.Body {
	t.Helper()
	if z0 == 0 {
		return body
	}
	tr, err := r3.Translation(r3.NewVec(0, 0, z0))
	if err != nil {
		t.Fatalf("%s: translation to z=%g: %v", label, z0, err)
	}
	moved, err := body.Placed(tr)
	if err != nil {
		t.Fatalf("%s: place at z=%g: %v", label, z0, err)
	}
	return moved
}

// measuresVolume reads a body's volume and compares it against the step's own
// formula under the step's own label, since decadtest otherwise names a body by
// index and recipe step, which does not say which feature is wrong.
func measuresVolume(t *testing.T, label string, body *decad.Body, want float64, opts ...decadtest.Option) {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("%s: read the volume: %v", label, err)
	}
	decadtest.Measures(t, label+" volume", v, units.CubicMillimeters(want), opts...)
}

// measuresBox reads a body's bounding box under the step's own label.
func measuresBox(t *testing.T, label string, body *decad.Body, lo, hi r3.Vec, opts ...decadtest.Option) {
	t.Helper()
	box, err := body.Bounds()
	if err != nil {
		t.Fatalf("%s: read the bounds: %v", label, err)
	}
	decadtest.MeasuresBox(t, label+" bounds", box, lo, hi, opts...)
}

// measuresPolygonBox reads a prism's bounding box against the polygon it was
// extruded from and the two z faces it spans.
func measuresPolygonBox(t *testing.T, label string, body *decad.Body, pts []point, z0, z1 float64) {
	t.Helper()
	lo, hi := polygonBounds(pts)
	measuresBox(t, label, body, r3.NewVec(lo.X, lo.Y, z0), r3.NewVec(hi.X, hi.Y, z1),
		decadtest.Within(units.Millimeters(boxSlack)))
}

// volumeOf is the reading itself, for the comparisons decadtest.Agree makes
// between two readings.
func volumeOf(t *testing.T, label string, body *decad.Body) decad.Measurement {
	t.Helper()
	v, err := body.Volume()
	if err != nil {
		t.Fatalf("%s: read the volume: %v", label, err)
	}
	return v
}

// requireOneLump fails when a body arrived as several disconnected pieces,
// which is how a join that did not close reads.
func requireOneLump(t *testing.T, label string, body *decad.Body) {
	t.Helper()
	if n := len(body.Lumps()); n != 1 {
		t.Fatalf("%s: the body has %d disconnected lump(s), want 1", label, n)
	}
}

// -- tolerances --------------------------------------------------------------

// exact is the slack for a reading whose expected value is a closed-form
// polygon or prism volume the evaluator computes the same way: float64
// rounding only.
func exact() decadtest.Option { return decadtest.WithinRel(units.Scalar(1e-9)) }

// faceted is the slack for a reading taken after a boolean against a
// cylindrical tool. decad tessellates the cylinder before the boolean, so the
// hole it actually removes is an inscribed prism slightly smaller than
// pi*r^2*h; measured at 4.3e-4 of the removed volume for a 7 mm hole through an
// 8 mm disc, so 2e-3 leaves an order of magnitude over the chord error and
// still refuses a wrong hole count or a wrong radius.
func faceted() decadtest.Option { return decadtest.WithinRel(units.Scalar(2e-3)) }

// -- case tables -------------------------------------------------------------

// baseCase is the dialog's own defaults, in the display units the dialog shows.
func baseCase() map[string]float64 {
	return map[string]float64{
		keyPinCount:            16,
		keyPinCircleDiameter:   90,
		keyPinDiameter:         0,
		keyEccentricity:        1.5,
		keyDiskClearance:       0.3,
		keyDiscThickness:       8,
		keyDiscGap:             0.5,
		keyCenterBearingDia:    30,
		keyInputShaftDiameter:  8,
		keyBearingClearance:    0.2,
		keyOutputPinCircleDia:  50,
		keyOutputPinCount:      6,
		keyOutputPinDiameter:   0,
		keyWall:                3,
		keyBaseThickness:       5,
		keyOutputPlateThicknes: 5,
		keyChamferSize:         0.5,
		keyDiscCount:           1,
		keyDisc:                0,
	}
}

// with returns the defaults overridden by changes, so a case reads as its own
// delta from the dialog.
func with(changes map[string]float64) map[string]float64 {
	p := baseCase()
	for k, v := range changes {
		p[k] = v
	}
	return p
}

// sketchCases is the table every sketch step is proven against. It reaches both
// sides of every branch the spec offers a sketch: the two discs (and so both
// signs of the eccentric offset), auto and override on each of the two pin
// sizes, the two count floors, an input-shaft bore and none, a zero clearance,
// and the top of the eccentricity range where the profile is closest to
// undercutting.
var sketchCases = []proofkit.Case{
	{Name: "defaults", Params: baseCase()},
	{Name: "second_disc_negative_eccentric", Params: with(map[string]float64{
		keyDiscCount: 2, keyDisc: 1,
	})},
	{Name: "pin_diameter_override", Params: with(map[string]float64{
		keyPinDiameter: 12,
	})},
	{Name: "output_pin_diameter_override", Params: with(map[string]float64{
		keyOutputPinDiameter: 8,
	})},
	{Name: "min_pin_count", Params: with(map[string]float64{
		keyPinCount: 4, keyOutputPinCount: 3, keyOutputPinCircleDia: 24,
		keyCenterBearingDia: 10, keyInputShaftDiameter: 6,
	})},
	{Name: "high_pin_count", Params: with(map[string]float64{
		keyPinCount: 24, keyOutputPinCount: 12,
	})},
	{Name: "high_eccentricity", Params: with(map[string]float64{
		keyEccentricity: 2.4,
	})},
	{Name: "no_input_bore_no_chamfer", Params: with(map[string]float64{
		keyInputShaftDiameter: 0, keyChamferSize: 0,
	})},
	{Name: "zero_clearances", Params: with(map[string]float64{
		keyDiskClearance: 0, keyBearingClearance: 0, keyDiscGap: 0,
	})},
}

// solidCases is the table every solid step is proven against. It is the sketch
// table's branch coverage carried into three dimensions, trimmed of the cases
// that differ from another only in a quantity no solid step reads.
var solidCases = []proofkit3d.Case{
	{Name: "defaults", Params: baseCase()},
	{Name: "second_disc_negative_eccentric", Params: with(map[string]float64{
		keyDiscCount: 2, keyDisc: 1,
	})},
	{Name: "both_diameters_overridden", Params: with(map[string]float64{
		keyPinDiameter: 12, keyOutputPinDiameter: 8,
	})},
	{Name: "min_pin_count", Params: with(map[string]float64{
		keyPinCount: 4, keyOutputPinCount: 3, keyOutputPinCircleDia: 24,
		keyCenterBearingDia: 10, keyInputShaftDiameter: 6,
	})},
	{Name: "high_eccentricity", Params: with(map[string]float64{
		keyEccentricity: 2.4,
	})},
	{Name: "no_input_bore_no_chamfer", Params: with(map[string]float64{
		keyInputShaftDiameter: 0, keyChamferSize: 0,
	})},
}

// twoDiscCases is the table for the steps that exist only when the dialog asks
// for two discs: the cam's section join. Both sides of the Disc Gap range are
// here, since the gap is what the lower section's extra height fills.
var twoDiscCases = []proofkit3d.Case{
	{Name: "two_discs_with_gap", Params: with(map[string]float64{
		keyDiscCount: 2, keyDisc: 0,
	})},
	{Name: "two_discs_no_gap", Params: with(map[string]float64{
		keyDiscCount: 2, keyDisc: 0, keyDiscGap: 0,
	})},
}
