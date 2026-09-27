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

type comparisonBody struct {
	doc  *decad.Document
	body *decad.Body
}

func makeComparisonBlock(ctx context.Context) (comparisonBody, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return comparisonBody{}, fmt.Errorf("create comparison sketch: %w", err)
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	result, err := s.Solve(ctx)
	if err != nil {
		return comparisonBody{}, fmt.Errorf("solve comparison sketch: %w", err)
	}
	if !result.Converged {
		return comparisonBody{}, fmt.Errorf("comparison sketch has %d DOF", result.DOF)
	}
	var region *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Holes) == 0 {
			if region != nil {
				return comparisonBody{}, fmt.Errorf("multiple comparison profiles")
			}
			region = profile
		}
	}
	if region == nil {
		return comparisonBody{}, fmt.Errorf("missing comparison profile")
	}
	doc := decad.New()
	body, err := doc.Extrude(s, region, decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	if err != nil {
		return comparisonBody{}, fmt.Errorf("extrude comparison block: %w", err)
	}
	return comparisonBody{doc: doc, body: body}, nil
}

func independentDocuments(a, b comparisonBody) bool {
	return a.doc != nil && b.doc != nil && a.doc != b.doc
}

func comparisonVolumeMatches(body *decad.Body, want float64) (bool, error) {
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
	return math.Abs(value-want) <= bound+1e-8, nil
}

func TestIndependentComparisonBodies(t *testing.T) {
	first, err := makeComparisonBlock(t.Context())
	require.NoError(t, err)
	second, err := makeComparisonBlock(t.Context())
	require.NoError(t, err)
	require.True(t, independentDocuments(first, second))
	require.False(t, independentDocuments(first, first), "the assertion must reject shared ownership")
	for _, comparison := range []comparisonBody{first, second} {
		proofkit3d.RequireSound(t, comparison.doc, []*decad.Body{comparison.body})
		matches, err := comparisonVolumeMatches(comparison.body, 4000)
		require.NoError(t, err)
		require.True(t, matches)
		matches, err = comparisonVolumeMatches(comparison.body, 5000)
		require.NoError(t, err)
		require.False(t, matches, "the volume assertion must reject a wrong block")
	}
}

func Example_proofkit3d_independent_comparison_bodies() {
	ctx := context.Background()
	first, err := makeComparisonBlock(ctx)
	if err != nil {
		fmt.Printf("failed to build first block: %s\n", err)
		return
	}
	second, err := makeComparisonBlock(ctx)
	if err != nil {
		fmt.Printf("failed to build second block: %s\n", err)
		return
	}
	if !independentDocuments(first, second) {
		fmt.Println("failed to separate comparison documents")
		return
	}
	for _, comparison := range []comparisonBody{first, second} {
		report, err := comparison.doc.Verify(ctx)
		if err != nil || !report.Passed() {
			fmt.Printf("failed to verify comparison body: %v\n", err)
			return
		}
		matches, err := comparisonVolumeMatches(comparison.body, 4000)
		if err != nil || !matches {
			fmt.Printf("failed to match comparison volume: %v\n", err)
			return
		}
	}
	fmt.Println("separate documents: true")
	fmt.Println("both volumes match: true")
	// Output:
	// separate documents: true
	// both volumes match: true
}
