package bevelgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/involute"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ---------------------------------------------------------------------------
// The solid model, and what it substitutes.
//
// Every gear is built in its own world frame: the Apex at the origin, the
// shaft axis along +Z, and the axial (Gear Profiles) plane at y = 0. A lattice
// point (along, radial) is then the world point (radial, 0, along), and the
// axial sketch is the world XZ plane whose own (u, v) is exactly (radial,
// along).
//
// Two substitutions run through all of these steps, and both are named again
// where they bite:
//
//   - THE TOOTH SECTION IS PERPENDICULAR TO THE SHAFT AXIS, not on the back
//     cone. Fusion draws the virtual spur tooth on the `{gear} Plane`, which
//     contains the tooth-centre reference line C->K' and is therefore tilted
//     from the axis-perpendicular plane by the pitch cone angle, and lofts to
//     it from the Apex point. decad's loft takes two profiles on distinct
//     planes and no point section, so the proof lofts between two
//     axis-perpendicular sections instead and applies the Tredgold mapping
//     that carries the back-cone tooth onto the real gear: a point's radial
//     offset from the root is preserved and its angular offset from the tooth
//     centre is divided by cos(gamma), which is what turns the virtual tooth's
//     360/virtualTeeth angular pitch into the real gear's 360/teeth. The cost
//     is that the proof does not exercise the tilt of the tooth plane itself
//     or the degenerate apex section; what it keeps is the tooth's size, its
//     curve inventory, the embedded flag, the taper toward the Apex, the
//     angular pitch the pattern depends on, and both conical trims.
//   - THE TOOTH ROOT IS SUNK a twentieth of the tooth's height below the gear
//     body's root cone. In Fusion the tooth seats exactly ON that cone and the
//     Combine-Join takes the face-to-face contact; decad refuses a boolean
//     whose operands graze within the chord tolerance without provably
//     crossing, so the tooth is sunk far enough to interpenetrate. What that
//     costs is the exactness of the seating: the proof shows the join leaves
//     one lump, not that a tooth resting exactly on the cone would.
//   - THE APEX SECTION IS A SHRUNKEN SECTION, not a point. The loft starts at
//     cone-distance fraction 0.02 rather than 0, since a degenerate section is
//     not representable. Everything the step pins — the taper ratio and the
//     trimmed extent — is measured well outside that stub, which the toe trim
//     removes anyway.
// ---------------------------------------------------------------------------

// pressureAngle is the spur drawer's own default, which the framework proxy
// serves to it: 20 degrees, and not a bevel dialog input.
var pressureAngle = 20 * math.Pi / 180

// involuteSteps is the proxy's InvoluteSteps: how many samples each flank
// carries.
const involuteSteps = 15

// toothRootSink is how far the tooth's root is sunk below the gear body's root
// cone, as a fraction of the tooth's height. See the substitution note above.
const toothRootSink = 0.05

// outlinePt is one point of a tooth section, in polar coordinates about the
// shaft axis: R is the distance from the axis and Th the angle in the section
// plane, both at the HEEL station (cone-distance fraction 1).
type outlinePt struct{ R, Th float64 }

// toothOutline is one gear's tooth cross-section at the heel.
type toothOutline struct {
	Right, Left  []outlinePt // flank samples, root end first, tip end last
	RootR, TipR  float64
	Embedded     bool
	VirtualTeeth int
}

// newToothOutline builds the virtual spur tooth this gear's section 3 draws,
// and maps it onto the gear.
//
// The virtual tooth number comes from the CLOSED FORM — floor(2 *
// (PitchDiameter/2)/cos(gamma) / Module) — never from measuring Apex2->K', and
// it is independent of Tooth Spacing, which moves only where the tooth is
// centred. The tooth is drawn already rotated by 180 degrees, through the
// drawer's own angle argument rather than by rotating the sketch afterwards.
func newToothOutline(d design, g gear) toothOutline {
	vt := virtualTeeth(d.Module, g.PitchDiameter, g.Gamma)
	dims := involute.Derive(d.Module, float64(vt), pressureAngle)
	left, right := involute.Flanks(dims.Base, dims.Tip, dims.Pitch,
		float64(vt), involuteSteps, math.Pi)

	rootAtHeel := g.PitchDiameter/2 - 1.25*d.Module*math.Cos(g.Gamma) -
		toothRootSink*(dims.Tip-dims.Root)
	cosGamma := math.Cos(g.Gamma)
	mapped := func(p involute.Pt) outlinePt {
		rv := math.Hypot(p.X, p.Y)
		phi := math.Atan2(p.Y, p.X) - math.Pi
		phi = math.Atan2(math.Sin(phi), math.Cos(phi))
		return outlinePt{R: rootAtHeel + (rv - dims.Root), Th: math.Pi + phi/cosGamma}
	}
	convert := func(src []involute.Pt) []outlinePt {
		out := make([]outlinePt, 0, len(src))
		for _, p := range src {
			out = append(out, mapped(p))
		}
		return out
	}

	o := toothOutline{
		Right:        convert(right),
		Left:         convert(left),
		RootR:        rootAtHeel,
		TipR:         rootAtHeel + (dims.Tip - dims.Root),
		Embedded:     dims.Embedded(),
		VirtualTeeth: vt,
	}
	// Put the flank with the smaller angle first, so the loop below runs
	// counter-clockwise.
	if o.Right[0].Th > o.Left[0].Th {
		o.Right, o.Left = o.Left, o.Right
	}
	if o.Embedded {
		// The flank starts inside the root circle, so tip, root and flanks meet
		// with no connecting lines: trim each flank at the root radius.
		o.Right = trimToRoot(o.Right, o.RootR)
		o.Left = trimToRoot(o.Left, o.RootR)
	}
	return o
}

// trimToRoot drops the flank samples that lie inside the root radius and pulls
// the first survivor onto it, which is where the embedded tooth's flank meets
// its root arc.
func trimToRoot(flank []outlinePt, rootR float64) []outlinePt {
	out := make([]outlinePt, 0, len(flank))
	for _, p := range flank {
		if p.R >= rootR {
			out = append(out, p)
		}
	}
	if len(out) == 0 {
		return flank
	}
	out[0].R = rootR
	return out
}

// section places the outline at a cone-distance fraction, twisted about the
// shaft axis and crowned about its own root.
//
// The whole gear is a cone from the Apex, so a section at fraction k is the
// heel section scaled by k — radius and axial position alike. crown scales
// only the tooth's HEIGHT, about the root edge, which is what anchoring the
// scale on the root rather than the face centroid means: the root stays on the
// seating cone and only the tip is relieved.
func (o toothOutline) section(k, twist, crown float64) ([]vec2, []vec2, float64, float64) {
	place := func(p outlinePt) vec2 {
		r := k * (o.RootR + crown*(p.R-o.RootR))
		th := p.Th + twist
		return v2(r*math.Cos(th), r*math.Sin(th))
	}
	right := make([]vec2, len(o.Right))
	for i, p := range o.Right {
		right[i] = place(p)
	}
	left := make([]vec2, len(o.Left))
	for i, p := range o.Left {
		left[i] = place(p)
	}
	return right, left, k * o.RootR, k * (o.RootR + crown*(o.TipR-o.RootR))
}

// drawSection draws one tooth section into a sketch on a plane perpendicular
// to the shaft axis, and returns its single profile.
//
// With chord false the loop is exactly the one section 3's profile search keys
// on: two NURBS flanks, two arcs (tip and root), and either no connecting lines
// when the tooth is embedded or two when it is not. That count is not a guess —
// for a given gear only ONE of the two counts is the real tooth, and an
// unrelated annular loop can carry the other.
//
// With chord true the same loop is emitted as a polyline. That is the
// substitution the solid steps need: decad refuses to extrude or loft a
// free-form boundary at this size, reporting that exact integration needs more
// than its fixed work budget, so every SOLID here is built from the chorded
// loop while the curve inventory is asserted on the curved one. The cost is
// that the solids carry a chording error, which every measurement below allows
// for explicitly rather than hiding.
func drawSection(t *testing.T, w *sketch.World, plane *sketch.Plane, o toothOutline,
	k, twist, crown float64, chord bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("tooth section sketch: %v", err)
	}
	right, left, rootR, _ := o.section(k, twist, crown)

	centre := s.CreatePoint(0, 0)
	s.Fix(centre)

	pt := func(p vec2) *sketch.Point {
		q := s.CreatePoint(p.X, p.Y)
		s.Fix(q)
		return q
	}
	radialAt := func(p vec2, r float64) vec2 { return p.unit().scale(r) }

	rightPts := make([]*sketch.Point, len(right))
	for i, p := range right {
		rightPts[i] = pt(p)
	}
	leftPts := make([]*sketch.Point, len(left))
	for i, p := range left {
		leftPts[i] = pt(p)
	}

	rightRoot, leftRoot := rightPts[0], leftPts[0]
	if !o.Embedded {
		// The flank starts outside the root circle, so a connecting line runs
		// radially inward from its first sample to the root arc.
		rightRoot = pt(radialAt(right[0], rootR))
		leftRoot = pt(radialAt(left[0], rootR))
		s.CreateLine(rightRoot, rightPts[0])
		s.CreateLine(leftPts[0], leftRoot)
	}

	rightTip := rightPts[len(rightPts)-1]
	leftTip := leftPts[len(leftPts)-1]
	if chord {
		polyline(s, rightPts)
		polyline(s, leftPts)
		arcPolyline(s, pt, rightTip, leftTip)
		arcPolyline(s, pt, rightRoot, leftRoot)
	} else {
		if _, err := s.CreateFitSpline(rightPts...); err != nil {
			t.Fatalf("right flank: %v", err)
		}
		if _, err := s.CreateFitSpline(leftPts...); err != nil {
			t.Fatalf("left flank: %v", err)
		}
		// The tip arc runs counter-clockwise from the right flank's tip to the
		// left flank's; the root arc is the one UNDER the tooth, between the
		// two root ends.
		s.CreateArc(centre, rightTip, leftTip)
		s.CreateArc(centre, rightRoot, leftRoot)
	}

	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("tooth section: solve: %v", err)
	}
	profs := s.Profiles()
	if len(profs) != 1 {
		t.Fatalf("tooth section holds %d regions, want exactly 1", len(profs))
	}
	return s, profs[0]
}

// arcSegments is how finely a chorded arc is cut. The tip and root arcs each
// span well under a tooth pitch, so eight segments hold the chord error to a
// few parts in ten thousand of the radius.
const arcSegments = 8

func polyline(s *sketch.Sketch, pts []*sketch.Point) {
	for i := 0; i+1 < len(pts); i++ {
		s.CreateLine(pts[i], pts[i+1])
	}
}

// arcPolyline chords an arc about the section's own centre, from start to end
// the short way round.
func arcPolyline(s *sketch.Sketch, pt func(vec2) *sketch.Point, start, end *sketch.Point) {
	a0 := math.Atan2(start.Y(), start.X())
	a1 := math.Atan2(end.Y(), end.X())
	r := math.Hypot(start.X(), start.Y())
	sweep := a1 - a0
	for sweep > math.Pi {
		sweep -= 2 * math.Pi
	}
	for sweep < -math.Pi {
		sweep += 2 * math.Pi
	}
	prev := start
	for i := 1; i <= arcSegments; i++ {
		next := end
		if i < arcSegments {
			a := a0 + sweep*float64(i)/float64(arcSegments)
			next = pt(v2(r*math.Cos(a), r*math.Sin(a)))
		}
		s.CreateLine(prev, next)
		prev = next
	}
}

// sectionPlane is the plane perpendicular to the shaft axis at the given
// cone-distance fraction of the root element Apex->C.
func sectionPlane(t *testing.T, w *sketch.World, f gearFrame, k float64) *sketch.Plane {
	t.Helper()
	p, err := w.CreateOffsetPlane(w.XY(), k*f.Ded.X)
	if err != nil {
		t.Fatalf("section plane at fraction %g: %v", k, err)
	}
	return p
}

// ---------------------------------------------------------------------------
// The gear body: revolve the section 2 hexagon.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Solids of revolution, as polygonal sweeps.
//
// SUBSTITUTION, and what it costs. Every body bevel revolves — the Gear Body
// frustum, the two cut cones, the bore tool — is a solid of revolution, and
// decad's revolve publishes a volume whose PROVEN bound is the volume itself:
// measured here on a plain cylinder, a cone and this gear's own frustum alike,
// the reading is Approximate with a bound of 2x the value, so a revolved body
// is Suspect at any tolerance and can never pass the harness gate. Its extrude,
// loft and union of polygonal profiles are Sound.
//
// So each solid of revolution is built as a chain of lofts between coaxial
// polygonal sections — the union of the bands its profile edges sweep, which is
// what the solid IS — and each band's expected volume is computed in closed
// form for the same polygon count, so the measurement stays exact against the
// model. Every reading is also tied back to the true solid of revolution within
// the chording error, so a wrong lattice still fails. The cost is that the
// proof does not exercise decad's revolve; it measures the same solid to a few
// parts in ten thousand.
// ---------------------------------------------------------------------------

// sweepFacets is the polygon count each revolved band is chorded to. Every
// reading below is written against the polygon rather than the circle — the
// band volumes come from sweptVolume and the Pappus tie-back carries
// chordFactor — so the count trades run time against fidelity to the true
// solid of revolution and against no tolerance at all. It must stay divisible
// by four, so a vertex lands on each axis and chordRadius is exactly 1.
const sweepFacets = 32

// chordFactor is the ratio of the inscribed regular polygon's area to the
// circle's, which is what the polygonal sweep's volume is short by.
func chordFactor(n int) float64 {
	return float64(n) * math.Sin(2*math.Pi/float64(n)) / (2 * math.Pi)
}

// station is one cross-section of a solid of revolution: an axial position and
// the inner and outer radii there. RIn 0 means a full disc.
type station struct{ Z, RIn, ROut float64 }

// ringProfile draws one station as a polygon, with a concentric hole when the
// station is annular.
func ringProfile(t *testing.T, w *sketch.World, z float64, st station, n int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	if err != nil {
		t.Fatalf("station plane at z=%g: %v", z, err)
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		t.Fatalf("station sketch at z=%g: %v", z, err)
	}
	ring := func(r float64) {
		pts := make([]*sketch.Point, n)
		for i := range n {
			a := 2 * math.Pi * float64(i) / float64(n)
			pts[i] = s.CreatePoint(r*math.Cos(a), r*math.Sin(a))
			s.Fix(pts[i])
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%n])
		}
	}
	ring(st.ROut)
	if st.RIn > 0 {
		ring(st.RIn)
	}
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatalf("station at z=%g: solve: %v", z, err)
	}
	profs := s.Profiles()
	for _, pr := range profs {
		if st.RIn > 0 && len(pr.Holes) == 1 {
			return s, pr
		}
		if st.RIn == 0 && len(pr.Holes) == 0 && len(pr.Entities) == n {
			return s, pr
		}
	}
	t.Fatalf("station at z=%g produced %d regions, none of the expected shape", z, len(profs))
	return nil, nil
}

// sweptBand lofts one pair of coaxial polygonal stations into the band the
// corresponding profile edge sweeps. It is ONE loft and no boolean: at the
// pinned decad revision a boolean refuses a loft operand outright, so nothing
// in this proof may join two bands.
func sweptBand(t *testing.T, doc *decad.Document, a, b station, n int) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0 := ringProfile(t, w, a.Z, a, n)
	s1, p1 := ringProfile(t, w, b.Z, b, n)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("band [%.4f, %.4f]: %v", a.Z, b.Z, err)
	}
	return body
}

// sweptVolume is the closed-form volume of the polygonal band sweptBand
// builds: its cross-sectional area varies quadratically with height, so the
// integral is the prismatoid form.
func sweptVolume(sts []station, n int) float64 {
	k := chordFactor(n) * math.Pi
	total := 0.0
	for i := 0; i+1 < len(sts); i++ {
		a, b := sts[i], sts[i+1]
		h := b.Z - a.Z
		outer := (a.ROut*a.ROut + a.ROut*b.ROut + b.ROut*b.ROut) / 3
		inner := (a.RIn*a.RIn + a.RIn*b.RIn + b.RIn*b.RIn) / 3
		total += k * h * (outer - inner)
	}
	return math.Abs(total)
}

// ---------------------------------------------------------------------------
// Laying bodies apart.
//
// SUBSTITUTION, and what it costs. At the decad revision this repo pins,
// 3ff4b3bb55cc, a boolean accepts only prism, cup and faceted payloads: an
// operand built by Loft is refused with "tessellation does not support payload
// decad.loftPayload". Every solid in this gear is conical — the frustum, both
// cut cones, the bore tool, the tooth — and a cone is a Loft or a Revolve at
// that revision, never a prism, because Extrude refuses a nonzero taper as
// ErrUnsupported. So NO boolean in this model is available: not the union that
// joins the frustum's bands, not the intersection and cut that trim the tooth
// to its flush band, not the bore's through-cut, not the Combine-Join.
//
// The substitute is uniform rather than per-step, so it is one thing to audit
// and one thing to undo when the pin moves: every step that would perform a
// boolean builds its operands, lays them apart, and asserts from their own
// measured geometry what the operation would have produced. A revolve is the
// union of the bands its profile edges sweep, and the union does not have to be
// performed for the bands' volumes, their cone half-angles and their signed sum
// against Pappus to be checked.
//
// What it costs is stated per step below, and it is the same shape each time:
// the proof shows that the operands are the right solids in the right places,
// and not that the evaluator's boolean joins, trims or pierces them.
//
// Bodies are laid apart ALONG THE SHAFT AXIS, so every reading the assertions
// take — a volume, a distance from that axis, a cone half-angle — is unchanged
// by the move, and only the axial extents shift, by an offset both sides
// compute from the same lattice.
// ---------------------------------------------------------------------------

// asideOffset is how far the i-th body of a step is moved along the shaft axis
// to lay it clear of the others. The stride is wider than any body's own reach,
// so the bodies are disjoint by their BOUNDING BOXES alone and decad never has
// to intersect two of them: at the pinned revision that read-only intersection
// stage is itself unsupported for a loft, and a pair it cannot separate cheaply
// comes back undecided. The cut cones are the long ones — each reaches
// 3 * (dedendum radius) * tan(gamma) back from its own apex — so the stride
// carries that term, which grows with the Shaft Angle.
func asideOffset(g gear, f gearFrame, i int) float64 {
	reach := (f.Base.X - toeStub(f)) + 4*f.Ded.Y*(1+math.Tan(g.Gamma))
	return float64(i) * 2 * reach
}

// layAside moves one body along the shaft axis by an asideOffset.
func layAside(t *testing.T, body *decad.Body, dz float64) *decad.Body {
	t.Helper()
	if dz == 0 {
		return body
	}
	tr, err := r3.Translation(r3.NewVec(0, 0, dz))
	if err != nil {
		t.Fatalf("lay aside by %.4f: %v", dz, err)
	}
	moved, err := body.Placed(tr)
	if err != nil {
		t.Fatalf("lay aside by %.4f: %v", dz, err)
	}
	return moved
}

// bandGeometry is a swept band read back off its own vertices: the two ring
// radii and the axial station each sits at, with the aside offset removed.
type bandGeometry struct {
	ZLo, ZHi     float64
	RLo, RHi     float64 // the greatest distance from the shaft axis at each end
	RLoIn, RHiIn float64 // and the least, which for a tooth section is its root
}

// Slope is the band's lateral cone in the axial plane: the change in radius per
// unit of axial distance, so atan(Slope) is its half-angle.
func (b bandGeometry) Slope() float64 { return (b.RHi - b.RLo) / (b.ZHi - b.ZLo) }

// measureBand reads a band's two rings from the solid rather than from the
// numbers it was built with. dz is the aside offset to subtract.
func measureBand(t *testing.T, body *decad.Body, dz float64) bandGeometry {
	t.Helper()
	verts := body.Vertices()
	if len(verts) == 0 {
		t.Fatal("band has no vertices to measure")
	}
	zLo, zHi := math.Inf(1), math.Inf(-1)
	for _, v := range verts {
		z := v.Position().Value.Z - dz
		zLo = math.Min(zLo, z)
		zHi = math.Max(zHi, z)
	}
	out := bandGeometry{ZLo: zLo, ZHi: zHi,
		RLoIn: math.Inf(1), RHiIn: math.Inf(1)}
	for _, v := range verts {
		pos := v.Position().Value
		z := pos.Z - dz
		r := math.Hypot(pos.X, pos.Y)
		if math.Abs(z-zLo) < math.Abs(z-zHi) {
			out.RLo = math.Max(out.RLo, r)
			out.RLoIn = math.Min(out.RLoIn, r)
		} else {
			out.RHi = math.Max(out.RHi, r)
			out.RHiIn = math.Min(out.RHiIn, r)
		}
	}
	return out
}

// requireBand checks a measured band against the station pair it was built
// from, at the aside offset it was moved by.
func requireBand(t *testing.T, body *decad.Body, dz float64, want [2]station, what string, args ...any) bandGeometry {
	t.Helper()
	got := measureBand(t, body, dz)
	label := sprintfArgs(what, args...)
	requireClose(t, got.ZLo, want[0].Z, 1e-6, "%s toe station", label)
	requireClose(t, got.ZHi, want[1].Z, 1e-6, "%s heel station", label)
	requireClose(t, got.RLo, want[0].ROut*chordRadius(sweepFacets), 1e-6, "%s toe radius", label)
	requireClose(t, got.RHi, want[1].ROut*chordRadius(sweepFacets), 1e-6, "%s heel radius", label)
	requireVolume(t, body, sweptVolume(want[:], sweepFacets), 1e-6, "%s volume", label)
	return got
}

// toeStubFraction leaves the very tip of the toe dish out of the swept model:
// at the toe corner M the cross-section is a single circle of zero area, which
// no loft can start from. The omitted sliver is 2% of the toe-edge span, and
// every volume reading below is taken against the hexagon TRUNCATED at the same
// station rather than against the untruncated one, so the truncation is stated
// rather than absorbed.
const toeStubFraction = 0.02

// radialAtZ interpolates the line through a and b, taken as (along, radial),
// at an axial position.
func radialAtZ(a, b vec2, z float64) float64 {
	if a.X == b.X {
		return a.Y
	}
	return a.Y + (b.Y-a.Y)*(z-a.X)/(b.X-a.X)
}

// truncatedHexagon is the section 2 frustum profile with its toe corner cut
// back to the stub station: the polygon the swept model actually realises.
func truncatedHexagon(f gearFrame) []vec2 {
	z := toeStub(f)
	return []vec2{
		f.Axis, f.Base, f.Heel, f.Ded,
		v2(z, radialAtZ(f.Toe, f.Ded, z)),
		v2(z, radialAtZ(f.Toe, f.ToeIn, z)),
		f.ToeIn,
	}
}

func toeStub(f gearFrame) float64 {
	return f.Toe.X + toeStubFraction*(f.ToeIn.X-f.Toe.X)
}

// frustumStations returns the three bands the section 2 frustum profile's edges
// sweep. The frustum is the first two MINUS the third: its outer surface
// changes generator at the dedendum corner C, and its front face is conical,
// not flat, so the toe end is a dish rather than a cap.
//
//	root — the root cone, from the toe stub out to C
//	heel — the heel cone, from C out to the heel end
//	plug — the cone the toe edge M->N sweeps, which hollows the dish
//
// The plug lies wholly inside the root band: it ends at the toe edge's inner
// end N, which is nearer the Apex than C, and its radius is below the root
// cone's everywhere above the stub, the two meeting exactly at the toe corner
// M. That containment is what makes root + heel - plug the frustum exactly,
// and the proof asserts it rather than assuming it.
func frustumStations(f gearFrame) (root, heel, plug [2]station) {
	zStub := toeStub(f)
	root = [2]station{
		{Z: zStub, ROut: radialAtZ(f.Toe, f.Ded, zStub)},
		{Z: f.Ded.X, ROut: f.Ded.Y},
	}
	heel = [2]station{
		{Z: f.Ded.X, ROut: f.Ded.Y},
		{Z: f.Base.X, ROut: f.Heel.Y},
	}
	plug = [2]station{
		{Z: zStub, ROut: radialAtZ(f.Toe, f.ToeIn, zStub)},
		{Z: f.ToeIn.X, ROut: f.ToeIn.Y},
	}
	return root, heel, plug
}

// frustumBands builds those three bands and lays them apart along the shaft
// axis. They are never joined; see the substitution note above.
func frustumBands(t *testing.T, doc *decad.Document, g gear, f gearFrame) []*decad.Body {
	t.Helper()
	root, heel, plug := frustumStations(f)
	out := make([]*decad.Body, 0, 3)
	for i, pair := range [][2]station{root, heel, plug} {
		out = append(out, layAside(t, sweptBand(t, doc, pair[0], pair[1], sweepFacets),
			asideOffset(g, f, i)))
	}
	return out
}

// stepRevolveGearBody revolves one gear's hexagon a full turn about its shaft
// edge, producing the Gear Body: the frustum whose conical faces are reused as
// the cutting tools for both end trims.
//
// The sketch holds exactly one closed loop, so the revolve takes its single
// profile without filtering, and the axis is the hexagon's FIRST edge — A->G
// resp. B->I, in the same sketch as the profile — never the section 2 Apex->A
// construction line, which lives in another sketch and which Fusion's revolve
// will not accept.
//
// SUBSTITUTION. decad's own revolve publishes a volume whose proven bound is the
// volume itself, so a revolved body is Suspect at any tolerance and cannot pass
// the harness gate; and at the pinned revision the union that would join the
// bands of a polygonal sweep refuses a loft operand. So the three bands are
// built and laid apart, never joined, and the frustum is asserted as their
// SIGNED SUM against Pappus on the section 2 hexagon. The cost is that the
// proof does not show the three bands closing into one watertight solid — each
// is separately watertight, and the shape they would form is checked by volume,
// by station and by cone half-angle instead.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->
func stepRevolveGearBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	return frustumBands(t, doc, g, f)
}

// assertRevolveGearBody checks the frustum against Pappus on the section 2
// hexagon: a full revolution sweeps 2*pi*rbar*A, with A the hexagon's own area
// and rbar its centroid's distance from the axis. Both come from the lattice, so
// the reading ties the solid back to section 2 rather than to a restatement of
// it. The hexagon used is the truncated one named above, and the tolerance is
// the polygonal sweep's own chording error at this facet count.
func assertRevolveGearBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 3 {
		t.Fatalf("%s gear body: got %d bands, want the root cone, the heel cone and the toe plug",
			g.Label, len(bodies))
	}
	root, heel, plug := frustumStations(f)

	// Each band is the band its own profile edge sweeps: right stations, right
	// ring radii, right volume, read back off the solid.
	rootGeom := requireBand(t, bodies[0], asideOffset(g, f, 0), root, "%s root cone band", g.Label)
	heelGeom := requireBand(t, bodies[1], asideOffset(g, f, 1), heel, "%s heel cone band", g.Label)
	plugGeom := requireBand(t, bodies[2], asideOffset(g, f, 2), plug, "%s toe dish plug", g.Label)

	// The signed sum IS the frustum. Nothing here performs the union; this is
	// the reading that stands in for it.
	sum := volumeOf(t, bodies[0]) + volumeOf(t, bodies[1]) - volumeOf(t, bodies[2])
	area, radial := polygonAreaCentroid(truncatedHexagon(f))
	want := 2 * math.Pi * radial * area * chordFactor(sweepFacets)
	if math.Abs(sum-want) > 1e-6*math.Max(1, want) {
		t.Fatalf("%s root + heel - plug is %.6f mm^3 against Pappus's %.6f mm^3",
			g.Label, sum, want)
	}

	// The cone half-angles, measured from the bands themselves. The heel band
	// and the toe plug lie on the SAME cone family — both are swept by lines
	// parallel to the back cone — which is what makes the face width constant
	// from toe to heel; the root band is a different family, and the angle
	// between them is the dedendum angle.
	requireClose(t, rootGeom.Slope(), (f.Ded.Y-f.Toe.Y)/(f.Ded.X-f.Toe.X), 1e-6,
		"%s root cone slope", g.Label)
	requireClose(t, heelGeom.Slope(), (f.Heel.Y-f.Ded.Y)/(f.Heel.X-f.Ded.X), 1e-6,
		"%s heel cone slope", g.Label)
	requireClose(t, plugGeom.Slope(), heelGeom.Slope(), 1e-6,
		"%s toe cone is parallel to the heel cone", g.Label)
	requireClose(t, heelGeom.Slope(), -1/math.Tan(g.Gamma), 1e-6,
		"%s back cone slope is -1/tan(gamma)", g.Label)
	if math.Abs(rootGeom.Slope()-heelGeom.Slope()) < 1e-9 {
		t.Fatalf("%s root and back cones came out parallel, so the dedendum angle vanished",
			g.Label)
	}

	// The plug is inside the root band, which is what makes the signed sum the
	// frustum rather than a coincidence of volumes.
	if plugGeom.ZLo < rootGeom.ZLo-1e-9 || plugGeom.ZHi > rootGeom.ZHi+1e-9 {
		t.Fatalf("%s toe plug spans [%.6f, %.6f] outside the root band's [%.6f, %.6f]",
			g.Label, plugGeom.ZLo, plugGeom.ZHi, rootGeom.ZLo, rootGeom.ZHi)
	}
	for _, z := range []float64{plugGeom.ZLo, plugGeom.ZHi} {
		inner := radialAtZ(f.Toe, f.ToeIn, z)
		outer := radialAtZ(f.Toe, f.Ded, z)
		if inner > outer+1e-9 {
			t.Fatalf("%s toe plug reaches %.6f mm at z=%.6f, outside the root cone's %.6f mm",
				g.Label, inner, z, outer)
		}
	}

	// And the frustum reaches where the lattice says: its widest point is the
	// dedendum corner and its heel end is the shaft edge's far end.
	requireClose(t, heelGeom.RLo, f.Ded.Y*chordRadius(sweepFacets), 1e-6,
		"%s widest radius is the dedendum corner", g.Label)
	requireClose(t, heelGeom.ZHi, f.Base.X, 1e-6, "%s heel station", g.Label)
	_ = d
}

// chordRadius is the fraction of the true radius an inscribed polygon's widest
// vertex reaches along an axis. With an even facet count a vertex lands on the
// axis, so it is 1.
func chordRadius(n int) float64 {
	if n%4 == 0 {
		return 1
	}
	return math.Cos(math.Pi / float64(n))
}

// polygonAreaCentroid returns a closed polygon's area and the distance of its
// centroid from the axis, taking each vertex as (along, radial).
func polygonAreaCentroid(poly []vec2) (area, radial float64) {
	var cx float64
	for i := range poly {
		a := poly[i]
		b := poly[(i+1)%len(poly)]
		cross := a.Y*b.X - b.Y*a.X
		area += cross
		cx += (a.Y + b.Y) * cross
	}
	area /= 2
	cx /= 6 * area
	return math.Abs(area), math.Abs(cx)
}

// ---------------------------------------------------------------------------
// The tooth profile, and the loft that turns it into a body.
// ---------------------------------------------------------------------------

// toothSectionThickness is the nominal extrusion the tooth-profile step uses to
// turn its section into something the solid harness can judge.
const toothSectionThickness = 0.5

// stepToothProfile draws one gear's `{gearLabel} Tooth` sketch and extrudes it
// a nominal thickness so the harness has a solid to gate.
//
// This is the ONE sketch bevel does not gate on isFullyConstrained: it is
// drawn by the borrowed spur generator, which labels its four circles with
// along-path text, and sketch text holds a degree of freedom, so a tooth
// sketch whose geometry is completely determined still reads False. The proof
// therefore proves it through the solid harness rather than the sketch gate,
// which is why the step is a solid step at all. What the step owns and what
// this asserts is bevel's side of the contract: the virtual tooth number, the
// tooth centre, the 180 degree draw angle, the embedded flag, and the curve
// inventory the profile search selects on. The spur tooth's own constraint
// scheme belongs to spec/spurgear and is proved there.
//
// <!-- proof-run: proofkit3d.RunSolid(toothSolidCases, stepToothProfile, assertToothProfile) -->
func stepToothProfile(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)

	w := sketch.NewWorld()
	_, curved := drawSection(t, w, sectionPlane(t, w, f, 1), o, 1, 0, 1, false)
	prof := curved

	// The inventory the tooth-profile selection keys on: two NURBS flanks, two
	// arcs, and 0 lines when the tooth is embedded, 2 when it is not. Accepting
	// "0 OR 2" here would let an unrelated annular loop through and the
	// apex-to-profile loft would die with LOFT_NO_TOOLBODY.
	wantLines := 2
	if o.Embedded {
		wantLines = 0
	}
	splines, arcs, lines := inventory(prof)
	if splines != 2 || arcs != 2 || lines != wantLines {
		t.Fatalf("%s tooth loop holds %d NURBS, %d arcs, %d lines; want 2, 2, %d (embedded=%v)",
			g.Label, splines, arcs, lines, wantLines, o.Embedded)
	}

	// The solid the harness gates is the chorded twin of that same loop.
	sc, pc := drawSection(t, w, sectionPlane(t, w, f, 1), o, 1, 0, 1, true)
	body, err := doc.Extrude(sc, pc,
		decad.Distance{D: units.Millimeters(toothSectionThickness), Dir: decad.Along})
	if err != nil {
		t.Fatalf("%s tooth section: extrude: %v", g.Label, err)
	}
	return []*decad.Body{body}
}

func assertToothProfile(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)

	// The virtual tooth number is the closed form, and Tooth Spacing does not
	// touch it: the spacing moves the tooth CENTRE and nothing else.
	requireClose(t, float64(o.VirtualTeeth),
		math.Floor(2*virtualPitchRadius(g.PitchDiameter, g.Gamma)/d.Module), tightTol,
		"%s virtual tooth number", g.Label)
	zero := d
	zero.ToothSpacing = 0
	if other := newToothOutline(zero, g); other.VirtualTeeth != o.VirtualTeeth {
		t.Fatalf("%s virtual tooth number moved with Tooth Spacing: %d vs %d",
			g.Label, o.VirtualTeeth, other.VirtualTeeth)
	}

	// The embedded flag is the spur drawer's own reading, and it is what the
	// line count above was taken from — not a second opinion.
	dims := involute.Derive(d.Module, float64(o.VirtualTeeth), pressureAngle)
	if o.Embedded != dims.Embedded() {
		t.Fatalf("%s embedded flag disagrees with the drawn tooth", g.Label)
	}

	// The tooth is drawn ALREADY rotated 180 degrees, through the drawer's
	// angle argument. Its centre therefore points away from the tooth-centre
	// reference line's outward direction, which is what the loft and the
	// pattern are built around.
	right, left, rootR, tipR := o.section(1, 0, 1)
	mid := right[len(right)-1].add(left[len(left)-1]).scale(0.5)
	requireClose(t, wrapAngle(math.Atan2(mid.Y, mid.X)-math.Pi), 0, 1e-6,
		"%s tooth centre angle away from the reference direction", g.Label)

	// The tooth stands on the gear body's root cone at the heel: its root
	// radius is the dedendum corner's radial distance, so the Combine-Join
	// leaves no gap.
	requireClose(t, rootR, f.Ded.Y-toothRootSink*(dims.Tip-dims.Root), tightTol,
		"%s tooth root radius at the heel, sunk into the root cone", g.Label)
	requireClose(t, tipR-rootR, dims.Tip-dims.Root, tightTol, "%s tooth height", g.Label)

	// The angular pitch the pattern will use: the virtual tooth's angular
	// thickness divided by cos(gamma) is the real gear's, which is what makes
	// 360/teeth copies close the circle.
	half := math.Abs(wrapAngle(math.Atan2(left[0].Y, left[0].X)-math.Atan2(right[0].Y, right[0].X))) / 2
	if half <= 0 || half >= math.Pi/g.Teeth {
		t.Fatalf("%s tooth half-thickness %.6f rad does not fit a %g-tooth pitch",
			g.Label, half, g.Teeth)
	}

	if len(bodies) != 1 {
		t.Fatalf("%s tooth section: got %d bodies, want 1", g.Label, len(bodies))
	}
}

// inventory tallies a profile's boundary by curve type, the same three counts
// find_profile_by_curve_counts matches on.
func inventory(prof *sketch.Profile) (splines, arcs, lines int) {
	for _, e := range prof.Entities {
		switch e.(type) {
		case *sketch.FitSpline, *sketch.Spline, *sketch.NURBS:
			splines++
		case *sketch.Arc:
			arcs++
		case *sketch.Line:
			lines++
		}
	}
	return splines, arcs, lines
}

// apexStubFraction is where the substituted apex section sits: the loft cannot
// start at a degenerate point, so it starts at 5% of the cone distance, well
// inside the toe trim that removes it and far enough out that its facets do
// not collapse below the evaluator's mesh bound. Every case's toe trim lands
// above 0.11 of the cone distance, since the Maximum Face Width caps the face
// at 0.95*sin(gamma)^2 of the pitch cone distance.
const apexStubFraction = 0.05

// heelOverrunFraction carries the loft past the heel cone so the heel trim has
// something to bite on.
const heelOverrunFraction = 1.12

// loftToothBody builds the uncut Apex-to-heel tooth.
func loftToothBody(t *testing.T, doc *decad.Document, d design, g gear, f gearFrame) *decad.Body {
	t.Helper()
	o := newToothOutline(d, g)
	w := sketch.NewWorld()
	s0, p0 := drawSection(t, w, sectionPlane(t, w, f, apexStubFraction), o, apexStubFraction, 0, 1, true)
	s1, p1 := drawSection(t, w, sectionPlane(t, w, f, heelOverrunFraction), o, heelOverrunFraction, 0, 1, true)
	body, err := doc.Loft(s0, p0, s1, p1)
	if err != nil {
		t.Fatalf("%s tooth loft: %v", g.Label, err)
	}
	return body
}

// stepLoftToothBody lofts the section 2 Apex sketch point to this gear's
// section 3 tooth profile, giving the uncut Tooth Body.
//
// The Apex end is a SKETCH point in Fusion, never a construction point: the
// Design component is never activated and construction geometry needs an
// active component. decad has no point section at all, so the proof lofts from
// the shrunken section named at the top of this file instead. What survives
// the substitution is the taper: the tooth's radial size at any station is
// proportional to its cone distance, which is what makes one lofted tooth
// serve the whole face width.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftToothBody, assertLoftToothBody) -->
func stepLoftToothBody(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	return []*decad.Body{loftToothBody(t, doc, d, g, f)}
}

func assertLoftToothBody(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)
	if len(bodies) != 1 {
		t.Fatalf("%s tooth body: got %d bodies, want 1", g.Label, len(bodies))
	}
	bounds, err := bodies[0].Bounds()
	if err != nil {
		t.Fatalf("%s tooth body bounds: %v", g.Label, err)
	}
	requireClose(t, bounds.Min.Z, apexStubFraction*f.Ded.X, 1e-6,
		"%s tooth body starts at the substituted apex station", g.Label)
	requireClose(t, bounds.Max.Z, heelOverrunFraction*f.Ded.X, 1e-6,
		"%s tooth body reaches past the heel cone", g.Label)

	// The taper: the tip radius at the heel-overrun station is the heel tip
	// radius scaled by the overrun, and the tooth never reaches further out
	// than that.
	_, _, _, tipHeel := o.section(heelOverrunFraction, 0, 1)
	reach := math.Max(math.Abs(bounds.Min.X), math.Abs(bounds.Max.X))
	requireClose(t, reach, tipHeel, 1e-3, "%s tooth body outer reach", g.Label)
}

// ---------------------------------------------------------------------------
// The two conical end trims.
// ---------------------------------------------------------------------------

// coneBandSlack is how far each cut cone is shifted outward, as a fraction of
// the distance between the two cone apexes. Both cut cones pass exactly through
// the tooth's own root edge — the toe cone through M and the heel cone through
// C — so an exact trim grazes the tooth along a whole curve. The sliver keeps
// the two surfaces apart, and every station below is written against it.
const coneBandSlack = 0.01

// cutConeStations returns this gear's two trim cones: the heel cone the tooth
// must lie INSIDE and the toe cone it must lie OUTSIDE.
//
// Both are cones of revolution generated by lines parallel to the back cone —
// the toe edge M->N and the heel edge C->H — so each meets the shaft axis at a
// single point, and the two are parallel. Each pair runs from its wide end at
// zLow up to a stub just short of its own apex, since a loft cannot start at a
// degenerate ring.
func cutConeStations(g gear, f gearFrame) (heel, toe [2]station, heelAxis, toeAxis float64) {
	tanGamma := math.Tan(g.Gamma)
	heelAxis = f.Ded.X + f.Ded.Y*tanGamma
	toeAxis = f.Toe.X + f.Toe.Y*tanGamma
	slack := coneBandSlack * (heelAxis - toeAxis)
	toeAxis -= slack
	heelAxis += slack

	maxR := 3 * f.Ded.Y
	zLow := toeAxis - maxR*tanGamma
	tip := 1e-4 * maxR
	heel = [2]station{
		{Z: zLow, ROut: (heelAxis - zLow) / tanGamma},
		{Z: heelAxis - tip*tanGamma, ROut: tip},
	}
	toe = [2]station{
		{Z: zLow - tip, ROut: (toeAxis - zLow + tip) / tanGamma},
		{Z: toeAxis - tip*tanGamma, ROut: tip},
	}
	return heel, toe, heelAxis, toeAxis
}

// bandStations returns the trimmed tooth's toe and heel axial extents.
//
// Each cut cone's radius FALLS as z grows, and the tooth's own surfaces are
// cones through the Apex whose radius RISES with z, so the two ends are reached
// on different surfaces: the toe end at the tooth's TIP, which leaves the toe
// cone first, and the heel end at the tooth's ROOT, which stays inside the heel
// cone longest. Each is the crossing of two cones written in z.
func bandStations(o toothOutline, g gear, f gearFrame) (zToe, zHeel float64) {
	tanGamma := math.Tan(g.Gamma)
	heelAxis := f.Ded.X + f.Ded.Y*tanGamma
	toeAxis := f.Toe.X + f.Toe.Y*tanGamma
	slack := coneBandSlack * (heelAxis - toeAxis)
	qRoot := o.RootR / f.Ded.X // the tooth's root surface, radius per unit of z
	qTip := o.TipR / f.Ded.X   // and its tip surface
	return (toeAxis - slack) / (1 + qTip*tanGamma),
		(heelAxis + slack) / (1 + qRoot*tanGamma)
}

// coneCrossing solves for the axial station at which a cone measured off a
// solid meets a surface of the tooth, itself a cone through the Apex of the
// given slope. Both are straight lines in the axial plane, so this is where
// they cross.
func coneCrossing(cone bandGeometry, toothSlope float64) float64 {
	slope := cone.Slope()
	return (cone.RLo - slope*cone.ZLo) / (toothSlope - slope)
}

// stepCutConicalEnds trims the uncut Tooth Body to a flush band with the toe
// cone and then the heel cone.
//
// Two distinct bodies are involved and they must not be conflated: the cutting
// TOOLS are cone faces of the Gear Body, the revolved-hexagon frustum, and the
// TARGET being split is the lofted Tooth Body. The lofted tooth has no cone
// faces of its own, so searching it for one finds nothing. Each cone face is
// found by the cut edge's world MIDPOINT rather than an endpoint, because an
// endpoint near the apex singularity cannot be evaluated at all. The toe cut
// runs first and must split; only the heel cut is lenient, and only through the
// typed NonIntersectError.
//
// SUBSTITUTION, and what it costs. At the pinned decad revision neither the
// intersection nor the cut can be performed at all: both operands are Lofts,
// and a boolean refuses a loft payload. So the tooth and the two cones are
// built and laid apart, and the trim is asserted from their own measured
// geometry — each cone's apex and half-angle read off the cone, each of the
// tooth's two surfaces read off the tooth, and the stations where they cross
// solved from those readings. What that costs is the split itself: the proof
// does not show the evaluator dividing the tooth, selecting the keeper, or
// leaving a watertight body. What it does show is that the cut lands exactly
// where the flush band requires, and that the two ends land on DIFFERENT
// surfaces of the tooth — the toe on its tip, the heel on its root — which is
// the observable signature of a conical cut face rather than a planar one.
//
// <!-- proof-run: proofkit3d.RunSolid(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->
func stepCutConicalEnds(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	heel, toe, _, _ := cutConeStations(g, f)
	return []*decad.Body{
		layAside(t, loftToothBody(t, doc, d, g, f), asideOffset(g, f, 0)),
		layAside(t, sweptBand(t, doc, heel[0], heel[1], sweepFacets), asideOffset(g, f, 1)),
		layAside(t, sweptBand(t, doc, toe[0], toe[1], sweepFacets), asideOffset(g, f, 2)),
	}
}

func assertCutConicalEnds(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)
	if len(bodies) != 3 {
		t.Fatalf("%s conical trim: got %d bodies, want the tooth and its two cut cones",
			g.Label, len(bodies))
	}
	tooth := measureBand(t, bodies[0], asideOffset(g, f, 0))
	heelCone := measureBand(t, bodies[1], asideOffset(g, f, 1))
	toeCone := measureBand(t, bodies[2], asideOffset(g, f, 2))
	_, _, heelAxis, toeAxis := cutConeStations(g, f)

	// The tooth's two surfaces are cones through the Apex: its root radius and
	// its tip radius both grow in proportion to the cone distance, which is
	// what one lofted tooth serving the whole face width means.
	rootSlope := (tooth.RHiIn - tooth.RLoIn) / (tooth.ZHi - tooth.ZLo)
	tipSlope := (tooth.RHi - tooth.RLo) / (tooth.ZHi - tooth.ZLo)
	requireClose(t, rootSlope, o.RootR/f.Ded.X, 1e-4, "%s tooth root cone slope", g.Label)
	requireClose(t, tipSlope, o.TipR/f.Ded.X, 1e-4, "%s tooth tip cone slope", g.Label)

	// Each cut cone is on the back-cone family and meets the shaft axis where
	// the lattice puts it.
	tanGamma := math.Tan(g.Gamma)
	for _, c := range []struct {
		name string
		geom bandGeometry
		axis float64
	}{{"heel", heelCone, heelAxis}, {"toe", toeCone, toeAxis}} {
		requireClose(t, c.geom.Slope(), -1/tanGamma, 1e-6,
			"%s %s cone slope is the back cone's", g.Label, c.name)
		apex := c.geom.ZLo - c.geom.RLo/c.geom.Slope()
		requireClose(t, apex, c.axis, 1e-5, "%s %s cone apex on the shaft axis", g.Label, c.name)
	}
	// The two are parallel, which is what makes the trimmed band's face width
	// constant from toe to heel.
	requireClose(t, toeCone.Slope(), heelCone.Slope(), 1e-9,
		"%s the two cut cones are parallel", g.Label)

	// Where each cone crosses each tooth surface. The toe end of the trimmed
	// tooth is reached on the TIP and the heel end on the ROOT, and the two
	// stations at each end differ — a planar cut would put them at one z.
	zToeTip := coneCrossing(toeCone, tipSlope)
	zToeRoot := coneCrossing(toeCone, rootSlope)
	zHeelTip := coneCrossing(heelCone, tipSlope)
	zHeelRoot := coneCrossing(heelCone, rootSlope)
	if !(zToeTip < zToeRoot) {
		t.Fatalf("%s toe cone meets the tooth's tip at z=%.6f and its root at z=%.6f: "+
			"the cut face is not conical", g.Label, zToeTip, zToeRoot)
	}
	if !(zHeelRoot > zHeelTip) {
		t.Fatalf("%s heel cone meets the tooth's root at z=%.6f and its tip at z=%.6f: "+
			"the cut face is not conical", g.Label, zHeelRoot, zHeelTip)
	}

	// And those stations are the band the flush trim leaves.
	wantToe, wantHeel := bandStations(o, g, f)
	requireClose(t, zToeTip, wantToe, 1e-3, "%s trimmed tooth toe station", g.Label)
	requireClose(t, zHeelRoot, wantHeel, 1e-3, "%s trimmed tooth heel station", g.Label)

	// The trim removes material at BOTH ends: the band is strictly inside the
	// uncut loft, which reaches from the substituted apex stub out past the
	// heel cone.
	if !(zToeTip > tooth.ZLo) {
		t.Fatalf("%s toe trim at z=%.6f removes nothing from a tooth starting at z=%.6f",
			g.Label, zToeTip, tooth.ZLo)
	}
	if !(zHeelRoot < tooth.ZHi) {
		t.Fatalf("%s heel trim at z=%.6f removes nothing from a tooth ending at z=%.6f",
			g.Label, zHeelRoot, tooth.ZHi)
	}
	if !(wantHeel-wantToe < tooth.ZHi-tooth.ZLo) {
		t.Fatalf("%s trimmed band spans %.6f mm of a %.6f mm loft: nothing was trimmed",
			g.Label, wantHeel-wantToe, tooth.ZHi-tooth.ZLo)
	}
}

// ---------------------------------------------------------------------------
// Pattern and Combine.
// ---------------------------------------------------------------------------

// stepCircularPattern circular-patterns the tooth around the shaft-axis edge,
// one copy per tooth.
//
// The pattern is pinned at quantity = this gear's tooth number, totalAngle =
// 360 deg and isSymmetric = false; all three are set explicitly rather than
// left to Fusion's defaults. The angular spacing stays 360/N for the whole face
// width even though the pitch diameter shrinks toward the Apex: the radial
// taper is already in the lofted tooth, so the pattern only rotates that one
// tapered tooth into N evenly spaced copies. A pattern returns the SEED plus
// the copies, so the seed is never re-added.
//
// SUBSTITUTION. decad has no pattern feature, and leaving N copies live in one
// document makes its verification check every PAIR of them for interference,
// which it cannot decide for teeth this close together. So the proof applies
// ONE pattern increment, consuming the seed rather than copying it, and
// measures where that increment put the tooth. The count and the closure are
// then arithmetic over the same increment, and stepToothProfile independently
// checks the tooth is thin enough to repeat N times without overlapping itself.
// The tooth patterned here is the UNCUT loft rather than the trimmed one,
// because at the pinned decad revision the trim cannot be performed at all
// (see stepCutConicalEnds); the increment is a rigid rotation either way.
//
// <!-- proof-run: proofkit3d.RunSolid(patternCases, stepCircularPattern, assertCircularPattern) -->
func stepCircularPattern(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	seed := loftToothBody(t, doc, d, g, f)

	before, err := seed.Centroid()
	if err != nil {
		t.Fatalf("%s pattern seed centroid: %v", g.Label, err)
	}
	patternSeedAzimuth = math.Atan2(before.Value.Y, before.Value.X)
	patternSeedRadius = math.Hypot(before.Value.X, before.Value.Y)
	patternSeedHeight = before.Value.Z
	patternSeedVolume = volumeOf(t, seed)

	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(2*math.Pi/g.Teeth))
	if err != nil {
		t.Fatalf("%s pattern rotation: %v", g.Label, err)
	}
	// Placed retires the seed, so exactly one body stays live and decad is not
	// asked to decide a pair it cannot.
	moved, err := seed.Placed(rot)
	if err != nil {
		t.Fatalf("%s pattern copy: %v", g.Label, err)
	}
	return []*decad.Body{moved}
}

// patternSeedAzimuth and the readings beside it carry the seed's measurements
// from the build to the assertion, because the seed is retired by the move and
// cannot be measured afterwards. The harness runs one case at a time in one
// goroutine.
var (
	patternSeedAzimuth float64
	patternSeedRadius  float64
	patternSeedHeight  float64
	patternSeedVolume  float64
)

func assertCircularPattern(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, _ := sideOf(d, p)
	if len(bodies) != 1 {
		t.Fatalf("%s patterned tooth: got %d bodies, want 1", g.Label, len(bodies))
	}
	n := g.Teeth
	pitch := 2 * math.Pi / n

	// One pattern increment moves the tooth exactly one tooth pitch about the
	// shaft axis and changes nothing else about it.
	after, err := bodies[0].Centroid()
	if err != nil {
		t.Fatalf("%s patterned tooth centroid: %v", g.Label, err)
	}
	az := math.Atan2(after.Value.Y, after.Value.X)
	requireClose(t, wrapAngle(az-patternSeedAzimuth-pitch), 0, 1e-6,
		"%s pattern increment is one tooth pitch", g.Label)
	requireClose(t, volumeOf(t, bodies[0]), patternSeedVolume, 1e-9,
		"%s pattern copy is the seed, moved", g.Label)
	// The copy keeps its distance from the axis and its station along it, which
	// is what makes the increment a rotation ABOUT that axis rather than any
	// other move: the pattern must not translate the tooth toward the Apex.
	requireClose(t, math.Hypot(after.Value.X, after.Value.Y), patternSeedRadius, 1e-6,
		"%s pattern copy stays the same distance from the shaft axis", g.Label)
	requireClose(t, after.Value.Z, patternSeedHeight, 1e-6,
		"%s pattern copy stays at the same station along the shaft axis", g.Label)

	// N increments close the circle exactly once, which is what
	// totalAngle = 360 deg with isSymmetric false means.
	requireClose(t, n*pitch, 2*math.Pi, tightTol,
		"%s pattern closes the circle exactly once", g.Label)

	// N copies at that pitch only fit if the tooth is thinner than the pitch.
	o := newToothOutline(d, g)
	right, left, _, _ := o.section(1, 0, 1)
	thickness := math.Abs(wrapAngle(
		math.Atan2(left[0].Y, left[0].X) - math.Atan2(right[0].Y, right[0].X)))
	if thickness >= pitch {
		t.Fatalf("%s tooth spans %.6f rad of a %.6f rad tooth pitch: %g copies would overlap",
			g.Label, thickness, pitch, n)
	}
}

// stepCombineJoin joins the patterned tooth pieces to the Gear Body in a single
// Combine-Join, the Gear Body as the target and the tooth bodies as the tools.
//
// pattern.bodies is a BRepBodies and combineFeatures.createInput rejects that
// type, so its items are copied into a fresh ObjectCollection first, and the
// seed is already among them.
//
// SUBSTITUTION, and what it costs. At the pinned decad revision the union
// cannot be performed: the frustum's bands and the tooth are all Lofts, and a
// boolean refuses a loft operand. So the operands are laid apart and the join's
// two consequences are asserted from their own measured geometry instead. A
// join leaves ONE lump when the tooth's root is at or below the body's root
// cone — seated, not floating — and the joined body reaches further out than
// the frustum when the tooth's tip stands proud of it. Both are readings on the
// two solids and both are what the lump count and the reach stand for. What the
// substitution cannot show is the evaluator stitching them into one boundary.
//
// The proof also sinks the tooth's root a twentieth of the tooth height below
// the root cone, which is what makes "seated" measurable as a strict
// inequality. In the generated module the tooth seats exactly on the cone.
//
// <!-- proof-run: proofkit3d.RunSolid(patternCases, stepCombineJoin, assertCombineJoin) -->
func stepCombineJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	root, heel, plug := frustumStations(f)
	return []*decad.Body{
		layAside(t, sweptBand(t, doc, root[0], root[1], sweepFacets), asideOffset(g, f, 0)),
		layAside(t, sweptBand(t, doc, heel[0], heel[1], sweepFacets), asideOffset(g, f, 1)),
		layAside(t, sweptBand(t, doc, plug[0], plug[1], sweepFacets), asideOffset(g, f, 2)),
		layAside(t, loftToothBody(t, doc, d, g, f), asideOffset(g, f, 3)),
	}
}

func assertCombineJoin(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	o := newToothOutline(d, g)
	if len(bodies) != 4 {
		t.Fatalf("%s combine-join: got %d bodies, want the three frustum bands and the tooth",
			g.Label, len(bodies))
	}
	rootBand := measureBand(t, bodies[0], asideOffset(g, f, 0))
	tooth := measureBand(t, bodies[3], asideOffset(g, f, 3))

	rootSlope := (tooth.RHiIn - tooth.RLoIn) / (tooth.ZHi - tooth.ZLo)
	tipSlope := (tooth.RHi - tooth.RLo) / (tooth.ZHi - tooth.ZLo)
	bodySlope := rootBand.Slope()

	// Across the whole band the join would cover, the tooth's root sits at or
	// below the gear body's root cone and its tip stands proud of it. The first
	// is why the join leaves one lump rather than two; the second is why the
	// joined body reaches further out than the frustum alone.
	zToe, zHeel := bandStations(o, g, f)
	for _, z := range []float64{zToe, (zToe + zHeel) / 2, zHeel} {
		bodyR := rootBand.RLo + bodySlope*(z-rootBand.ZLo)
		if toothRoot := rootSlope * z; toothRoot > bodyR {
			t.Fatalf("%s tooth root is %.6f mm at z=%.6f, above the body's root cone at %.6f mm: "+
				"the tooth floats and the join would leave two lumps",
				g.Label, toothRoot, z, bodyR)
		}
		if toothTip := tipSlope * z; toothTip <= bodyR {
			t.Fatalf("%s tooth tip is %.6f mm at z=%.6f, no further out than the body's %.6f mm: "+
				"the join would add nothing", g.Label, toothTip, z, bodyR)
		}
	}
	// The sink is exactly the one this proof asks for, and no more: a deeper
	// tooth would be buried rather than seated.
	dims := involute.Derive(d.Module, float64(o.VirtualTeeth), pressureAngle)
	requireClose(t, rootSlope*f.Ded.X, f.Ded.Y-toothRootSink*(dims.Tip-dims.Root), 1e-4,
		"%s tooth root is sunk exactly one twentieth of the tooth height", g.Label)
}

// ---------------------------------------------------------------------------
// The bore.
// ---------------------------------------------------------------------------

// stepBoreCut cuts the cylindrical through bore along the shaft axis.
//
// The cut is a symmetric through-extrude of the Bore sketch's circle,
// restricted to this Gear Body, with 2 * Cone Distance as the half-length per
// side so it clears any face width. The step is skipped entirely when Enable
// Bore is unchecked.
//
// SUBSTITUTION, and what it costs. The tool is a real Extrude — a prism, which
// is what setSymmetricExtent produces — but the target is the frustum, whose
// bands are Lofts, and at the pinned decad revision a boolean refuses a loft
// operand. So the tool and the bands are laid apart and the cut is asserted
// from their own measured geometry: the tool's diameter and its symmetric
// extent read off the prism, that it reaches past both ends of the frustum so
// the cut is a through cut, and the material it would remove computed from the
// frustum's own profile clipped to the bore radius. What it cannot show is the
// pierced body: that the result is one lump with a hole and no enclosed void.
//
// <!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepBoreCut, assertBoreCut) -->
func stepBoreCut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	bodies := frustumBands(t, doc, g, f)
	if !d.BoreEnable {
		return bodies
	}

	// The bore plane is rooted at the start of the shaft-axis edge, so the
	// sketch origin already lies on the axis and the circle is centred there:
	// its centre is FIXED and its diameter dimensioned, never coincident to the
	// sketch origin. The extent is symmetric, 2 * Cone Distance per side.
	w := sketch.NewWorld()
	s, prof := ringProfile(t, w, f.Axis.X, station{Z: f.Axis.X, ROut: g.Bore / 2}, sweepFacets)
	tool, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(2 * d.ConeDistance)})
	if err != nil {
		t.Fatalf("%s bore tool: %v", g.Label, err)
	}
	return append(bodies, layAside(t, tool, asideOffset(g, f, 3)))
}

func assertBoreCut(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if !d.BoreEnable {
		if len(bodies) != 3 {
			t.Fatalf("%s bore unchecked: got %d bodies, want the three frustum bands alone",
				g.Label, len(bodies))
		}
		if g.Bore != 0 {
			t.Fatalf("%s resolved a bore diameter of %.6f mm with Enable Bore unchecked",
				g.Label, g.Bore)
		}
		return
	}
	if len(bodies) != 4 {
		t.Fatalf("%s bore: got %d bodies, want the three frustum bands and the bore tool",
			g.Label, len(bodies))
	}
	tool := measureBand(t, bodies[3], asideOffset(g, f, 3))

	// The tool is a cylinder of the resolved bore diameter, centred on the
	// shaft axis, extruded symmetrically 2 * Cone Distance each way from the
	// start of the shaft-axis edge.
	requireClose(t, 2*tool.RLo, g.Bore*chordRadius(sweepFacets), 1e-6,
		"%s bore tool diameter", g.Label)
	requireClose(t, tool.Slope(), 0, 1e-9, "%s bore tool is a cylinder, not a cone", g.Label)
	requireClose(t, tool.ZLo, f.Axis.X-2*d.ConeDistance, 1e-6,
		"%s bore tool reaches 2 * Cone Distance below the shaft edge start", g.Label)
	requireClose(t, tool.ZHi, f.Axis.X+2*d.ConeDistance, 1e-6,
		"%s bore tool reaches 2 * Cone Distance above it", g.Label)

	// It is a THROUGH cut: the tool clears both ends of the frustum.
	if !(tool.ZLo < toeStub(f) && tool.ZHi > f.Base.X) {
		t.Fatalf("%s bore tool spans [%.6f, %.6f], inside the frustum's [%.6f, %.6f]",
			g.Label, tool.ZLo, tool.ZHi, toeStub(f), f.Base.X)
	}
	// And it pierces rather than grazes: the bore radius is below the smallest
	// radius the frustum reaches, at the toe.
	if g.Bore/2 >= f.Toe.Y {
		t.Fatalf("%s bore radius %.6f mm reaches the toe root radius %.6f mm",
			g.Label, g.Bore/2, f.Toe.Y)
	}

	// What the cut would remove: the frustum's own material inside the bore
	// radius, and it is a real bite out of the body rather than a sliver.
	solid := volumeOf(t, bodies[0]) + volumeOf(t, bodies[1]) - volumeOf(t, bodies[2])
	removed := revolvedVolumeInside(truncatedHexagon(f), g.Bore/2) * chordFactor(sweepFacets)
	if !(removed > 0 && removed < solid) {
		t.Fatalf("%s bore would remove %.6f mm^3 of a %.6f mm^3 body", g.Label, removed, solid)
	}
	// "0 means auto" resolves to this gear's Pitch Diameter / 4.
	if p[keyDrivingBore] == 0 && g.Label == "Driving" {
		requireClose(t, g.Bore, g.PitchDiameter/4, tightTol, "driving auto bore diameter")
	}
	if p[keyPinionBore] == 0 && g.Label == "Pinion" {
		requireClose(t, g.Bore, g.PitchDiameter/4, tightTol, "pinion auto bore diameter")
	}
}

// revolvedVolumeInside is the volume of the hexagon's solid of revolution
// clipped to radii at or below r, computed by integrating the axial thickness
// over the radius. It is the material a through bore of that radius removes.
func revolvedVolumeInside(poly []vec2, r float64) float64 {
	const steps = 4000
	total := 0.0
	for i := range steps {
		rad := r * (float64(i) + 0.5) / steps
		lo, hi, ok := axialSpanAt(poly, rad)
		if !ok {
			continue
		}
		total += 2 * math.Pi * rad * (hi - lo) * (r / steps)
	}
	return total
}

// axialSpanAt returns the axial extent of the hexagon at a given radius.
func axialSpanAt(poly []vec2, rad float64) (lo, hi float64, ok bool) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		if (a.Y-rad)*(b.Y-rad) > 0 {
			continue
		}
		if a.Y == b.Y {
			continue
		}
		f := (rad - a.Y) / (b.Y - a.Y)
		z := a.X + f*(b.X-a.X)
		lo = math.Min(lo, z)
		hi = math.Max(hi, z)
		ok = true
	}
	return lo, hi, ok
}

// ---------------------------------------------------------------------------
// The meshing rotation.
// ---------------------------------------------------------------------------

// stepMeshRotation rotates the driving body by half a tooth pitch about its own
// shaft axis, so a driving valley sits where the pinion tooth crosses the axial
// plane.
//
// The rotation runs in the Design component, before the body is moved out, and
// takes its axis and origin from the profile edge's WORLD endpoints. The
// pinion's own extra phase is _PINION_MESH_PHASE_TEETH tooth-fractions, which
// is 0: a zero angle is not a move at all — Fusion refuses the identity
// transform with "invalid transform" — so the helper returns early rather than
// making each call site guard it.
//
// The body rotated here is the frustum's three bands, laid apart, since at the
// pinned decad revision they cannot be joined into one; the rotation is applied
// to each, which is the same rigid move the whole body would take.
//
// <!-- proof-run: proofkit3d.RunSolid(meshCases, stepMeshRotation, assertMeshRotation) -->
func stepMeshRotation(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	root, heel, plug := frustumStations(f)

	angle := meshAngle(d, g, p)
	out := make([]*decad.Body, 0, 3)
	for i, pair := range [][2]station{root, heel, plug} {
		body := sweptBand(t, doc, pair[0], pair[1], sweepFacets)
		if angle != 0 {
			// A zero angle is a no-op the caller must not attempt: the identity
			// transform is refused, which is why the helper absorbs it.
			rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(angle))
			if err != nil {
				t.Fatalf("%s mesh rotation: %v", g.Label, err)
			}
			if body, err = body.Placed(rot); err != nil {
				t.Fatalf("%s mesh rotation: %v", g.Label, err)
			}
		}
		out = append(out, layAside(t, body, asideOffset(g, f, i)))
	}
	return out
}

// meshAngle is the rotation this gear receives: half a tooth pitch for the
// driving gear, and the pinion's own mesh phase (0 by default) for the pinion.
func meshAngle(d design, g gear, p map[string]float64) float64 {
	if g.Label == "Driving" {
		return math.Pi / g.Teeth
	}
	return pinionMeshPhase(d.Pinion.Teeth)
}

func assertMeshRotation(t *testing.T, doc *decad.Document, bodies []*decad.Body, p map[string]float64) {
	d := newDesign(t, p)
	g, f := sideOf(d, p)
	if len(bodies) != 3 {
		t.Fatalf("%s rotated body: got %d bands, want 3", g.Label, len(bodies))
	}
	// The rotation is about the body's own axis of revolution, so it changes
	// neither volume nor any distance from that axis — which is exactly why the
	// angle has to be checked as a number rather than read off the solid.
	root, heel, plug := frustumStations(f)
	for i, pair := range [][2]station{root, heel, plug} {
		requireBand(t, bodies[i], asideOffset(g, f, i), pair,
			"%s band %d after the meshing rotation", g.Label, i)
	}

	angle := meshAngle(d, g, p)
	if g.Label == "Driving" {
		requireClose(t, angle, math.Pi/g.Teeth, tightTol,
			"driving meshing rotation is half a tooth pitch")
		// Half a pitch is what puts a valley where the other gear's tooth is:
		// twice it is a whole pitch, which the pattern already repeats.
		requireClose(t, 2*angle, 2*math.Pi/g.Teeth, tightTol,
			"twice the meshing rotation is one whole tooth pitch")
	} else {
		requireClose(t, angle, 0, tightTol, "pinion mesh phase is zero by default")
	}
}

// ---------------------------------------------------------------------------
// Measurement helpers.
// ---------------------------------------------------------------------------

func volumeOf(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	m, err := b.Volume()
	if err != nil {
		t.Fatalf("volume: %v", err)
	}
	v, err := m.Value.In(units.CubicMillimeter)
	if err != nil {
		t.Fatalf("volume units: %v", err)
	}
	return v
}

func requireVolume(t *testing.T, b *decad.Body, want, rel float64, what string, args ...any) {
	t.Helper()
	got := volumeOf(t, b)
	if math.Abs(got-want) > rel*math.Max(1, math.Abs(want)) {
		t.Fatalf("%s: got %.6f mm^3, want %.6f mm^3 (relative tolerance %g)",
			sprintfArgs(what, args...), got, want, rel)
	}
}

// wrapAngle folds an angle into (-pi, pi].
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
