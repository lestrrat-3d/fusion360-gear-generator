package examples_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// boreBody uses one profile with a hole. It proves the resulting solid when
// an explicit boolean cut exceeds the analytic evaluator's segment budget.
func boreBody(ctx context.Context, innerRadius float64) (*decad.Document, *decad.Body, *sketch.Profile, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, nil, nil, fmt.Errorf("create sketch: %w", err)
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	s.CreateCircle(center, innerRadius)
	result, err := s.Solve(ctx)
	if err != nil {
		return nil, nil, nil, fmt.Errorf("solve sketch: %w", err)
	}
	if !result.Converged {
		return nil, nil, nil, fmt.Errorf("sketch did not converge: DOF %d", result.DOF)
	}
	var annulus *sketch.Profile
	for _, profile := range s.Profiles() {
		if !profile.Valid || len(profile.Holes) != 1 {
			continue
		}
		if annulus != nil {
			return nil, nil, nil, fmt.Errorf("multiple annular profiles")
		}
		annulus = profile
	}
	if annulus == nil {
		return nil, nil, nil, fmt.Errorf("missing annular profile")
	}
	doc := decad.New()
	body, err := doc.Extrude(s, annulus, decad.Distance{
		D: units.Millimeters(8), Dir: decad.Along,
	})
	if err != nil {
		return nil, nil, nil, fmt.Errorf("extrude annulus: %w", err)
	}
	return doc, body, annulus, nil
}

func boreVolumeMatches(measurement decad.Measurement, innerRadius float64) (bool, error) {
	volume, err := measurement.Value.In(units.CubicMillimeter)
	if err != nil {
		return false, err
	}
	bound, err := measurement.Bound.In(units.CubicMillimeter)
	if err != nil {
		return false, err
	}
	want := math.Pi * (10*10 - innerRadius*innerRadius) * 8
	return math.Abs(volume-want) <= bound+1e-8, nil
}

func TestBoreProfile(t *testing.T) {
	doc, body, profile, err := boreBody(t.Context(), 2)
	require.NoError(t, err)
	proofkit3d.RequireSolid(t, doc, []*decad.Body{body})
	require.Len(t, profile.Entities, 1, "the outer boundary has one source circle")
	require.Len(t, profile.Outer, 1, "the outer loop has one unsplit boundary edge")
	require.Len(t, profile.Holes, 1)
	require.Len(t, profile.Holes[0], 1, "the bore loop has one unsplit boundary edge")
	require.InDelta(t, 96*math.Pi, profile.Area, 1e-8)
	measurement, err := body.Volume()
	require.NoError(t, err)
	matches, err := boreVolumeMatches(measurement, 2)
	require.NoError(t, err)
	require.True(t, matches, "the annular solid must exclude a radius-2 bore")

	// The same verified measurement must reject the solid with a larger bore.
	matches, err = boreVolumeMatches(measurement, 3)
	require.NoError(t, err)
	require.False(t, matches, "the assertion must detect the wrong bore radius")
}

func Example_proofkit3d_bore_profile() {
	ctx := context.Background()
	doc, body, profile, err := boreBody(ctx, 2)
	if err != nil {
		fmt.Printf("failed to build bore profile: %s\n", err)
		return
	}
	report, err := doc.Verify(ctx)
	if err != nil {
		fmt.Printf("failed to verify bore profile: %s\n", err)
		return
	}
	if !report.Passed() || len(profile.Entities) != 1 || len(profile.Outer) != 1 ||
		len(profile.Holes) != 1 || len(profile.Holes[0]) != 1 {
		fmt.Println("failed to verify bore geometry")
		return
	}
	measurement, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to read bore volume: %s\n", err)
		return
	}
	matches, err := boreVolumeMatches(measurement, 2)
	if err != nil || !matches {
		fmt.Printf("failed to match bore volume: %v\n", err)
		return
	}
	fmt.Println("sound: true")
	fmt.Println("bore volume matches: true")
	// Output:
	// sound: true
	// bore volume matches: true
}
