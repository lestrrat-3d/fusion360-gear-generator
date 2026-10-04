package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var cellCases = []proofkit3d.Case{
	{Name: "four-teeth", Params: dimensions()},
	{Name: "one-tooth", Params: changed(map[string]float64{"teeth": 1})},
	{Name: "slow-twist", Params: changed(map[string]float64{"teeth": 1, "lead": 400})},
	{Name: "fast-twist", Params: changed(map[string]float64{"teeth": 1, "lead": 20})},
	{Name: "negative-slant", Params: changed(map[string]float64{"teeth": 1, "slant": -25.8})},
	{Name: "straight-ridge", Params: changed(map[string]float64{"teeth": 1, "slant": 0, "bow": 0})},
	{Name: "gear-b-negative-phase", Params: changed(map[string]float64{"teeth": 1, "gear": 1, "phase": -1.31})},
	{Name: "positive-phase", Params: changed(map[string]float64{"teeth": 1, "phase": 1.31})},
}

func sectionSketch(t *testing.T, p map[string]float64, station float64, corners [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	return sectionSketchSense(t, p, station, corners, false)
}

func sectionSketchSense(t *testing.T, p map[string]float64, station float64, corners [][2]float64, reverse bool) (*sketch.Sketch, *sketch.Profile) {
	w := sketch.NewWorld()
	angle := p["cross"] * math.Pi / 360
	u := r3.Vec{Z: 1}
	origin := r3.Vec{Z: -(p["W"] - p["engage"]) / 2}
	if p["gear"] == 1 {
		angle = -angle
		u = r3.Vec{Z: -1}
		origin.Z = -origin.Z
	}
	dir := r3.Vec{X: math.Cos(angle), Y: math.Sin(angle)}
	v := dir.Cross(u)
	theta := station/(p["lead"]/(2*math.Pi)) + p["phi"]*math.Pi/180
	uTurn := u.Scale(math.Cos(theta)).Add(v.Scale(math.Sin(theta)))
	vTurn := u.Scale(-math.Sin(theta)).Add(v.Scale(math.Cos(theta)))
	if reverse {
		vTurn = vTurn.Scale(-1)
		flipped := make([][2]float64, len(corners))
		for i := range corners {
			c := corners[len(corners)-1-i]
			flipped[i] = [2]float64{c[0], -c[1]}
		}
		corners = flipped
	}
	f, err := r3.NewFrame(origin.Add(dir.Scale(station)), uTurn, vTurn)
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixedPolygon(s, corners)
	return s, decadtest.SolveRegion(t, s)
}
func loftChain(t *testing.T, doc *decad.Document, p map[string]float64, start, end float64, count int, outline func(float64) [][2]float64) *decad.Body {
	previous, profile := sectionSketch(t, p, start, outline(start))
	// Union refuses these solids' exactly shared coplanar caps. Build the same
	// two-section loft walls as sheets, add oppositely oriented end caps, and
	// stitch the closed boundary. No volume overlap or tolerance waiver is used.
	capSketch, capProfile := sectionSketchSense(t, p, start, outline(start), true)
	startCap, err := doc.Patch(t.Context(), capSketch, capProfile)
	require.NoError(t, err)
	sheets := []*decad.Body{startCap}
	for i := 1; i < count; i++ {
		station := start + (end-start)*float64(i)/float64(count-1)
		next, nextProfile := sectionSketch(t, p, station, outline(station))
		part, err := doc.Loft(t.Context(), previous, profile, next, nextProfile, decad.WithSurfaceResult())
		require.NoError(t, err)
		sheets = append(sheets, part)
		previous, profile = next, nextProfile
	}
	endCap, err := doc.Patch(t.Context(), previous, profile)
	require.NoError(t, err)
	sheets = append(sheets, endCap)
	result, err := decad.Stitch(t.Context(), sheets...)
	require.NoError(t, err)
	return result
}
func buildCell(t *testing.T, doc *decad.Document, p map[string]float64) *decad.Body {
	// Fusion's single smooth loft is replaced by two-section polygonal lofts.
	// The tooth fit points are chorded, and each ruled wall cell is triangulated.
	// This proves the sampled section construction and solid topology, not the
	// fitted spline or the smooth interpolation between sections (up to 0.125 mm).
	n := math.Max(8, math.Ceil(p["P"]/(p["lead"]/(2*math.Pi))/(2*math.Pi/180)))
	start := p["phase"] - p["N"]*p["P"]/2
	return loftChain(t, doc, p, start, start+p["teeth"]*p["P"], int(p["teeth"]*n)+1,
		func(station float64) [][2]float64 {
			corners := sectionCorners(p, station)
			if p["slant"] == 0 && p["bow"] == 0 {
				// All eleven tooth fit points are collinear in this regime.
				// Coalesce their ten chords to one identical straight edge:
				// repeated collinear facets make decad's validity undecided.
				// This changes no section geometry; the sketch proof still
				// checks every fit point and the original thirteen edges.
				return [][2]float64{corners[0], corners[1], corners[11], corners[12]}
			}
			return corners
		})
}
func stepCellLoft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	return []*decad.Body{buildCell(t, doc, p)}
}
func assertCellLoft(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 1)
	require.Len(t, b[0].Lumps(), 1)
	// The stand-in chords eleven fit points and triangulates the ruled walls.
	// A 4% volume slack covers the recorded 2.2% shortfall at straight ridges.
	want := (p["W"] - p["H"]/2 - p["bow"]*p["T"]*p["T"]/12) * p["T"] * p["P"] * p["teeth"]
	decadtest.MeasuresVolume(t, b[0], units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(0.04)))
}

func stepCellCopy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	seed := buildCell(t, doc, p)
	copy, err := seed.Duplicate(t.Context())
	require.NoError(t, err)
	// Decad cannot verify coincident copies: its pair-contact classifier rejects
	// shared face planes. Move the actual duplicate far away before verification.
	// This still checks duplication and volume identity, but cannot prove that
	// Fusion leaves the copied body's placement coincident before its move entry.
	separation, err := r3.Translation(r3.Vec{X: 1000, Y: 1000, Z: 1000})
	require.NoError(t, err)
	copy, err = copy.Placed(t.Context(), separation)
	require.NoError(t, err)
	return []*decad.Body{seed, copy}
}
func assertCellCopy(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	require.Len(t, b, 2)
	a, err := b[0].Volume()
	require.NoError(t, err)
	c, err := b[1].Volume()
	require.NoError(t, err)
	// A copy is exact; the extra slack covers floating point roundoff only.
	decadtest.Agree(t, "copied cell volume", a, c, decadtest.Within(units.CubicMillimeters(1e-7)))
}
func screwTransform(t *testing.T, p map[string]float64, k float64) r3.Transform {
	angle := p["cross"] * math.Pi / 360
	origin := r3.Vec{Z: -(p["W"] - p["engage"]) / 2}
	if p["gear"] == 1 {
		angle = -angle
		origin.Z = -origin.Z
	}
	dir := r3.Vec{X: math.Cos(angle), Y: math.Sin(angle)}
	rotation, err := r3.RotationAround(origin, dir, units.Radians(k*p["P"]/(p["lead"]/(2*math.Pi))))
	require.NoError(t, err)
	shift, err := r3.Translation(dir.Scale(k * p["P"]))
	require.NoError(t, err)
	transform, err := rotation.Then(shift)
	require.NoError(t, err)
	return transform
}

// This step stays serial because its assertion consumes the seed reading.
var cellMoveExpected decad.VecMeasurement

func stepCellMove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	body := buildCell(t, doc, p)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	transform := screwTransform(t, p, p["teeth"])
	cellMoveExpected = centroid
	cellMoveExpected.Value = transform.Apply(centroid.Value)
	moved, err := body.Placed(t.Context(), transform)
	require.NoError(t, err)
	return []*decad.Body{moved}
}
func assertCellMove(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	assertCellLoft(t, doc, b, p)
	centroid, err := b[0].Centroid()
	require.NoError(t, err)
	// Rigid motion preserves the seed centroid's bound. Count that bound as
	// formula slack; MeasuresVec adds the moved reading's bound independently.
	decadtest.MeasuresVec(t, "screw-moved cell centroid", centroid, cellMoveExpected.Value,
		decadtest.Within(cellMoveExpected.Bound))
}
func stepCellJoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	// Decad refuses boolean composition of stitched boundaries because their
	// mesh has no proven volume bound. Substitute the joined outer boundary:
	// extend the same sampled loft chain over both adjacent screw-moved blocks.
	// The seam is exactly the end/start section because tooth is pitch-periodic.
	// This proves the resulting volume and solid topology, but cannot
	// prove Fusion Combine consumes the copy or preserves the target identity.
	doubled := make(map[string]float64, len(p))
	for k, v := range p {
		doubled[k] = v
	}
	doubled["teeth"] *= 2
	return []*decad.Body{buildCell(t, doc, doubled)}
}
func assertCellJoin(t *testing.T, doc *decad.Document, b []*decad.Body, p map[string]float64) {
	copy := make(map[string]float64, len(p))
	for k, v := range p {
		copy[k] = v
	}
	copy["teeth"] *= 2
	assertCellLoft(t, doc, b, copy)
}
