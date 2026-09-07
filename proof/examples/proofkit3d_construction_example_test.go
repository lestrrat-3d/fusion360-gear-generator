package examples_test

import (
	"context"
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Each construction keeps body handles with their owning document.
type construction struct {
	doc    *decad.Document
	bodies []*decad.Body
}

func newXYSketch() (*sketch.Sketch, error) {
	w := sketch.NewWorld()
	return w.CreateSketch(w.XY())
}

func solvedProfile(ctx context.Context, s *sketch.Sketch, holes int) (*sketch.Profile, error) {
	result, err := s.Solve(ctx)
	if err != nil {
		return nil, fmt.Errorf("solve sketch: %w", err)
	}
	if !result.Converged {
		return nil, fmt.Errorf("solve did not converge: residual %g, DOF %d", result.Residual, result.DOF)
	}
	var selected *sketch.Profile
	for _, profile := range s.Profiles() {
		if !profile.Valid || len(profile.Holes) != holes {
			continue
		}
		if selected != nil {
			return nil, fmt.Errorf("multiple valid profiles with %d holes", holes)
		}
		selected = profile
	}
	if selected == nil {
		return nil, fmt.Errorf("no valid profile with %d holes", holes)
	}
	return selected, nil
}

func extrudeProfile(doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, height float64) (*decad.Body, error) {
	return doc.Extrude(s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
}

func rectangleIn(ctx context.Context, doc *decad.Document, x, width, height, depth float64) (*decad.Body, error) {
	s, err := newXYSketch()
	if err != nil {
		return nil, fmt.Errorf("create rectangle sketch: %w", err)
	}
	rect := s.CreateRectangle(x, 0, x+width, height)
	s.Fix(rect.A)
	p, err := solvedProfile(ctx, s, 0)
	if err != nil {
		return nil, err
	}
	return extrudeProfile(doc, s, p, depth)
}

func cylinderIn(ctx context.Context, doc *decad.Document, x, y, radius, height float64) (*decad.Body, error) {
	s, err := newXYSketch()
	if err != nil {
		return nil, fmt.Errorf("create circle sketch: %w", err)
	}
	center := s.CreatePoint(x, y)
	s.Fix(center)
	s.CreateCircle(center, radius)
	p, err := solvedProfile(ctx, s, 0)
	if err != nil {
		return nil, err
	}
	return extrudeProfile(doc, s, p, height)
}

func independentDocuments(ctx context.Context) ([]construction, error) {
	var builds []construction
	// The first two documents each hold one comparison body. The third
	// demonstrates spatial separation inside a single document.
	for _, positions := range [][]float64{{0}, {0}, {0, 100}} {
		build := construction{doc: decad.New()}
		for _, x := range positions {
			body, err := rectangleIn(ctx, build.doc, x, 20, 20, 10)
			if err != nil {
				return nil, fmt.Errorf("build comparison block: %w", err)
			}
			build.bodies = append(build.bodies, body)
		}
		builds = append(builds, build)
	}
	return builds, nil
}

func annularProfile(ctx context.Context) ([]construction, *sketch.Profile, *sketch.Profile, error) {
	s, err := newXYSketch()
	if err != nil {
		return nil, nil, nil, fmt.Errorf("create annular sketch: %w", err)
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	s.CreateCircle(center, 2)
	annulus, err := solvedProfile(ctx, s, 1)
	if err != nil {
		return nil, nil, nil, err
	}
	var inner *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Holes) == 0 {
			if inner != nil {
				return nil, nil, nil, fmt.Errorf("multiple valid inner disks")
			}
			inner = profile
		}
	}
	if inner == nil {
		return nil, nil, nil, fmt.Errorf("missing valid inner disk")
	}
	doc := decad.New()
	body, err := extrudeProfile(doc, s, annulus, 8)
	if err != nil {
		return nil, nil, nil, fmt.Errorf("extrude annulus: %w", err)
	}
	diskDoc := decad.New()
	disk, err := cylinderIn(ctx, diskDoc, 0, 0, 10, 8)
	if err != nil {
		return nil, nil, nil, fmt.Errorf("extrude comparison disk: %w", err)
	}
	return []construction{{doc, []*decad.Body{body}}, {diskDoc, []*decad.Body{disk}}}, annulus, inner, nil
}

func placedTool(ctx context.Context, doc *decad.Document,
	x, y, radius, height, z float64) (*decad.Body, error) {
	pin, err := cylinderIn(ctx, doc, x, y, radius, height)
	if err != nil {
		return nil, fmt.Errorf("extrude cutting tool: %w", err)
	}
	down, err := r3.Translation(r3.Vec{Z: z})
	if err != nil {
		return nil, fmt.Errorf("build tool translation: %w", err)
	}
	tool, err := pin.Placed(down)
	if err != nil {
		return nil, fmt.Errorf("place cutting tool: %w", err)
	}
	return tool, nil
}

func placedCut(ctx context.Context, doc *decad.Document, target *decad.Body,
	x, y, radius, height, z float64) (*decad.Body, error) {
	tool, err := placedTool(ctx, doc, x, y, radius, height, z)
	if err != nil {
		return nil, err
	}
	return decad.CutContext(ctx, target, tool)
}

func explicitCut(ctx context.Context) (construction, error) {
	doc := decad.New()
	plate, err := rectangleIn(ctx, doc, 0, 20, 20, 8)
	if err != nil {
		return construction{}, fmt.Errorf("build plate: %w", err)
	}
	body, err := placedCut(ctx, doc, plate, 14, 6, 2, 20, -6)
	if err != nil {
		return construction{}, fmt.Errorf("cut bore: %w", err)
	}
	return construction{doc, []*decad.Body{body}}, nil
}

func separateCapChamfers(ctx context.Context) ([]construction, error) {
	var builds []construction
	for _, end := range []bool{false, true} {
		doc := decad.New()
		body, err := cylinderIn(ctx, doc, 0, 0, 10, 8)
		if err != nil {
			return nil, fmt.Errorf("build chamfer cylinder: %w", err)
		}
		edges := decad.Edges(decad.CreatedBy(decad.CapStart(body)))
		if end {
			edges = decad.Edges(decad.CreatedBy(decad.CapEnd(body)))
		}
		chamfered, err := body.ChamferContext(ctx, edges, units.Millimeters(0.5))
		if err != nil {
			return nil, fmt.Errorf("chamfer cap (end=%t): %w", end, err)
		}
		builds = append(builds, construction{doc, []*decad.Body{chamfered}})
	}
	return builds, nil
}

func chamferVolume() float64 {
	const R, H, d = 10.0, 8.0, 0.5
	return math.Pi*R*R*(H-d) + math.Pi*d/3*(R*R+R*(R-d)+(R-d)*(R-d))
}

// Build both operands before asking for the expected refusal. A setup failure
// must not be mistaken for an evaluator-budget refusal.
func arrangementOperands(ctx context.Context) (construction, error) {
	s, err := newXYSketch()
	if err != nil {
		return construction{}, fmt.Errorf("create perforated plate sketch: %w", err)
	}
	rect := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(rect.A)
	for i := range 17 {
		center := s.CreatePoint(float64(10+15*(i%5)), float64(10+15*(i/5)))
		s.Fix(center)
		s.CreateCircle(center, 1)
	}
	profile, err := solvedProfile(ctx, s, 17)
	if err != nil {
		return construction{}, err
	}
	doc := decad.New()
	plate, err := extrudeProfile(doc, s, profile, 8)
	if err != nil {
		return construction{}, fmt.Errorf("extrude perforated plate: %w", err)
	}
	toolSketch, err := newXYSketch()
	if err != nil {
		return construction{}, fmt.Errorf("create arrangement tool sketch: %w", err)
	}
	center := toolSketch.CreatePoint(90, 90)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 0.5)
	toolProfile, err := solvedProfile(ctx, toolSketch, 0)
	if err != nil {
		return construction{}, err
	}
	// Preserve the shared XY sketch plane for analytic admission while the
	// tool extends past both target caps, from z=-1 to z=9.
	tool, err := doc.Extrude(toolSketch, toolProfile, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(9)},
		Two: decad.DistanceSide{D: units.Millimeters(1)},
	})
	if err != nil {
		return construction{}, fmt.Errorf("extrude arrangement tool: %w", err)
	}
	return construction{doc, []*decad.Body{plate, tool}}, nil
}

func volumeMM(m *decad.Measurement) (float64, float64, error) {
	if m == nil {
		return 0, 0, fmt.Errorf("missing volume measurement")
	}
	value, err := m.Value.In(units.CubicMillimeter)
	if err != nil {
		return 0, 0, fmt.Errorf("convert volume: %w", err)
	}
	bound, err := m.Bound.In(units.CubicMillimeter)
	if err != nil {
		return 0, 0, fmt.Errorf("convert volume bound: %w", err)
	}
	return value, bound, nil
}

func TestConstructionIndependentDocuments(t *testing.T) {
	builds, err := independentDocuments(t.Context())
	require.NoError(t, err)
	require.Len(t, builds, 3)
	for i, build := range builds {
		wantBodies := 1
		if i == 2 {
			wantBodies = 2
		}
		require.Len(t, build.bodies, wantBodies)
		proofkit3d.RequireSound(t, build.doc, build.bodies)
		for _, body := range build.bodies {
			value, bound, err := volumeMM(proofkit3d.BodyReport(t, build.doc, body).Volume)
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(value-4000), bound+1e-8)
		}
	}
}

func TestConstructionAnnularProfile(t *testing.T) {
	builds, annulus, inner, err := annularProfile(t.Context())
	require.NoError(t, err)
	for _, build := range builds {
		proofkit3d.RequireSolid(t, build.doc, build.bodies)
	}
	require.True(t, annulus.Valid)
	require.Len(t, annulus.Holes, 1)
	// Profile.Area is a closed-form value and carries no reported bound.
	require.LessOrEqual(t, math.Abs(annulus.Area-96*math.Pi), 1e-8)
	require.LessOrEqual(t, math.Abs(inner.Area-4*math.Pi), 1e-8)
	values, bounds := make([]float64, 2), make([]float64, 2)
	for i, want := range []float64{768 * math.Pi, 800 * math.Pi} {
		build := builds[i]
		require.Len(t, build.bodies, 1)
		values[i], bounds[i], err = volumeMM(proofkit3d.BodyReport(t, build.doc, build.bodies[0]).Volume)
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(values[i]-want), bounds[i]+1e-8)
	}
	require.LessOrEqual(t, math.Abs(values[1]-values[0]-32*math.Pi), bounds[0]+bounds[1]+1e-8)
}

func TestConstructionExplicitCut(t *testing.T) {
	build, err := explicitCut(t.Context())
	require.NoError(t, err)
	proofkit3d.RequireSolid(t, build.doc, build.bodies)
	value, bound, err := volumeMM(proofkit3d.BodyReport(t, build.doc, build.bodies[0]).Volume)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(value-(3200-32*math.Pi)), bound+1e-8)
}

func TestConstructionSeparateCapChamfers(t *testing.T) {
	builds, err := separateCapChamfers(t.Context())
	require.NoError(t, err)
	require.Len(t, builds, 2)
	for _, build := range builds {
		proofkit3d.RequireSolid(t, build.doc, build.bodies)
		value, bound, err := volumeMM(proofkit3d.BodyReport(t, build.doc, build.bodies[0]).Volume)
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(value-chamferVolume()), bound+1e-8)
		require.Less(t, value, 800*math.Pi)
	}
}

func TestConstructionArrangementLimit(t *testing.T) {
	build, err := arrangementOperands(t.Context())
	require.NoError(t, err)
	_, err = decad.CutContext(t.Context(), build.bodies[0], build.bodies[1])
	require.Error(t, err)
	require.ErrorContains(t, err, "arranger segments")
	require.ErrorContains(t, err, "cap of")
	t.Logf("expected arrangement refusal: %s", err)
}

// Examples use the engine verdict directly because they have no testing.T.
// Reports are checked before the independent closed-form volume comparisons.
func verifiedVolumes(ctx context.Context, builds []construction, wants [][]float64) ([][]float64, [][]float64, error) {
	values, bounds := make([][]float64, len(builds)), make([][]float64, len(builds))
	if len(builds) != len(wants) {
		return nil, nil, fmt.Errorf("unexpected document count: %d", len(builds))
	}
	for i, build := range builds {
		report, err := build.doc.Verify(ctx)
		if err != nil {
			return nil, nil, fmt.Errorf("verify document %d: %w", i, err)
		}
		if !report.Trustworthy() {
			return nil, nil, fmt.Errorf("document %d is not trustworthy: %s, %v", i, report.Status, report.Diagnostics)
		}
		if len(build.bodies) != len(wants[i]) || len(report.Bodies) != len(build.bodies) {
			return nil, nil, fmt.Errorf("unexpected body count in document %d", i)
		}
		for j, body := range build.bodies {
			var record *decad.BodyReport
			for _, candidate := range report.Bodies {
				if candidate.Body == body {
					record = candidate
					break
				}
			}
			if record == nil {
				return nil, nil, fmt.Errorf("document %d omitted body %d", i, j)
			}
			value, bound, err := volumeMM(record.Volume)
			if err != nil {
				return nil, nil, err
			}
			if !(math.Abs(value-wants[i][j]) <= bound+1e-8) {
				return nil, nil, fmt.Errorf("document %d body %d volume %g, want %g, bound %g", i, j, value, wants[i][j], bound)
			}
			values[i] = append(values[i], value)
			bounds[i] = append(bounds[i], bound)
		}
	}
	return values, bounds, nil
}

func Example_proofkit3d_independent_documents() {
	ctx := context.Background()
	builds, err := independentDocuments(ctx)
	if err != nil {
		fmt.Printf("failed to build independent documents: %s\n", err)
		return
	}
	if _, _, err := verifiedVolumes(ctx, builds, [][]float64{{4000}, {4000}, {4000, 4000}}); err != nil {
		fmt.Printf("failed to verify independent documents: %s\n", err)
		return
	}
	fmt.Println("sound: true")
	fmt.Println("volume matches: true")
	// Output:
	// sound: true
	// volume matches: true
}

func Example_proofkit3d_annular_profile() {
	ctx := context.Background()
	builds, annulus, inner, err := annularProfile(ctx)
	if err != nil {
		fmt.Printf("failed to build annular profile: %s\n", err)
		return
	}
	values, bounds, err := verifiedVolumes(ctx, builds, [][]float64{{768 * math.Pi}, {800 * math.Pi}})
	if err != nil {
		fmt.Printf("failed to verify annular profile: %s\n", err)
		return
	}
	if !(math.Abs(annulus.Area-96*math.Pi) <= 1e-8) || !(math.Abs(inner.Area-4*math.Pi) <= 1e-8) {
		fmt.Printf("failed to match profile areas: annulus %g, inner disk %g\n", annulus.Area, inner.Area)
		return
	}
	if !(math.Abs(values[1][0]-values[0][0]-32*math.Pi) <= bounds[0][0]+bounds[1][0]+1e-8) {
		fmt.Println("failed to match removed volume")
		return
	}
	fmt.Println("sound: true")
	fmt.Println("volume matches: true")
	// Output:
	// sound: true
	// volume matches: true
}

func Example_proofkit3d_explicit_cut() {
	ctx := context.Background()
	build, err := explicitCut(ctx)
	if err != nil {
		fmt.Printf("failed to build explicit cut: %s\n", err)
		return
	}
	if _, _, err := verifiedVolumes(ctx, []construction{build}, [][]float64{{3200 - 32*math.Pi}}); err != nil {
		fmt.Printf("failed to verify explicit cut: %s\n", err)
		return
	}
	fmt.Println("sound: true")
	fmt.Println("volume matches: true")
	// Output:
	// sound: true
	// volume matches: true
}

func Example_proofkit3d_separate_cap_chamfers() {
	ctx := context.Background()
	builds, err := separateCapChamfers(ctx)
	if err != nil {
		fmt.Printf("failed to build separate cap chamfers: %s\n", err)
		return
	}
	values, _, err := verifiedVolumes(ctx, builds, [][]float64{{chamferVolume()}, {chamferVolume()}})
	if err != nil {
		fmt.Printf("failed to verify separate cap chamfers: %s\n", err)
		return
	}
	for _, value := range values {
		if !(value[0] < 800*math.Pi) {
			fmt.Printf("failed to remove cap material: volume %g\n", value[0])
			return
		}
	}
	fmt.Println("sound: true")
	fmt.Println("volume matches: true")
	// Output:
	// sound: true
	// volume matches: true
}

func Example_proofkit3d_arrangement_limit() {
	ctx := context.Background()
	build, err := arrangementOperands(ctx)
	if err != nil {
		fmt.Printf("failed to build arrangement operands: %s\n", err)
		return
	}
	_, err = decad.CutContext(ctx, build.bodies[0], build.bodies[1])
	if err == nil || !strings.Contains(err.Error(), "arranger segments") || !strings.Contains(err.Error(), "cap of") {
		fmt.Printf("failed to get arrangement-budget refusal: %v\n", err)
		return
	}
	fmt.Println("budget refusal: true")
	// Output:
	// budget refusal: true
}
