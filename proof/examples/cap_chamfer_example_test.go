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

const chamferRadius = 10.0
const chamferHeight = 8.0
const chamferSetback = 0.5

func makeCapChamfer(ctx context.Context, start bool) (*decad.Document, *decad.Body, int, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, nil, 0, fmt.Errorf("create cylinder sketch: %w", err)
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, chamferRadius)
	result, err := s.Solve(ctx)
	if err != nil {
		return nil, nil, 0, fmt.Errorf("solve cylinder sketch: %w", err)
	}
	if !result.Converged {
		return nil, nil, 0, fmt.Errorf("cylinder sketch has %d DOF", result.DOF)
	}
	var region *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Holes) == 0 {
			if region != nil {
				return nil, nil, 0, fmt.Errorf("multiple cylinder profiles")
			}
			region = profile
		}
	}
	if region == nil {
		return nil, nil, 0, fmt.Errorf("missing cylinder profile")
	}
	doc := decad.New()
	body, err := doc.Extrude(s, region, decad.Distance{
		D: units.Millimeters(chamferHeight), Dir: decad.Along,
	})
	if err != nil {
		return nil, nil, 0, fmt.Errorf("extrude cylinder: %w", err)
	}
	query := decad.Edges(decad.CreatedBy(decad.CapEnd(body)))
	if start {
		query = decad.Edges(decad.CreatedBy(decad.CapStart(body)))
	}
	edges, err := query.SelectEdges(body)
	if err != nil {
		return nil, nil, 0, fmt.Errorf("select cap edge: %w", err)
	}
	chamfered, err := body.Chamfer(ctx, query, units.Millimeters(chamferSetback))
	if err != nil {
		return nil, nil, 0, fmt.Errorf("chamfer cap: %w", err)
	}
	return doc, chamfered, len(edges), nil
}

func oneCapChamferVolume(setback float64) float64 {
	r, h := chamferRadius, chamferHeight
	return math.Pi*r*r*(h-setback) + math.Pi*setback/3*(r*r+r*(r-setback)+(r-setback)*(r-setback))
}

func chamferVolumeMatches(body *decad.Body, setback float64) (bool, error) {
	volume, err := body.Volume()
	if err != nil {
		return false, err
	}
	value, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		return false, err
	}
	bound, err := volume.Bound.In(units.CubicMillimeter)
	if err != nil {
		return false, err
	}
	return math.Abs(value-oneCapChamferVolume(setback)) <= bound+1e-8, nil
}

func TestSeparateCapChamfers(t *testing.T) {
	for _, start := range []bool{true, false} {
		doc, body, count, err := makeCapChamfer(t.Context(), start)
		require.NoError(t, err)
		require.Equal(t, 1, count, "one circular rim must be selected")
		proofkit3d.RequireSolid(t, doc, []*decad.Body{body})
		matches, err := chamferVolumeMatches(body, chamferSetback)
		require.NoError(t, err)
		require.True(t, matches, "the chamfer must remove the expected cap material")
		matches, err = chamferVolumeMatches(body, 1)
		require.NoError(t, err)
		require.False(t, matches, "the assertion must reject a different setback")
	}
}

func Example_proofkit3d_separate_cap_chamfers() {
	ctx := context.Background()
	for _, start := range []bool{true, false} {
		doc, body, count, err := makeCapChamfer(ctx, start)
		if err != nil {
			fmt.Printf("failed to chamfer cap: %s\n", err)
			return
		}
		report, err := doc.Verify(ctx)
		if err != nil || !report.Passed() || count != 1 {
			fmt.Printf("failed to verify cap chamfer: %v\n", err)
			return
		}
		matches, err := chamferVolumeMatches(body, chamferSetback)
		if err != nil || !matches {
			fmt.Printf("failed to match chamfer volume: %v\n", err)
			return
		}
	}
	fmt.Println("both cap chamfers sound: true")
	fmt.Println("both volumes match: true")
	// Output:
	// both cap chamfers sound: true
	// both volumes match: true
}
