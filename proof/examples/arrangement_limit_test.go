package examples_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The tool and plate sketches stay coplanar so decad takes its analytic path.
func arrangementOperands(t *testing.T, holes int) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	world := sketch.NewWorld()
	plateSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := plateSketch.CreateRectangle(0, 0, 100, 100)
	plateSketch.Fix(rect.A)
	for i := range holes {
		center := plateSketch.CreatePoint(float64(10+15*(i%5)), float64(10+15*(i/5)))
		plateSketch.Fix(center)
		plateSketch.CreateCircle(center, 1)
	}
	result, err := plateSketch.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, result.Converged)
	var plateProfile *sketch.Profile
	for _, profile := range plateSketch.Profiles() {
		if profile.Valid && len(profile.Holes) == holes {
			require.Nil(t, plateProfile)
			plateProfile = profile
		}
	}
	require.NotNil(t, plateProfile)
	require.Len(t, plateProfile.Holes, holes)

	toolSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := toolSketch.CreatePoint(90, 90)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 0.5)
	result, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, result.Converged)
	var toolProfile *sketch.Profile
	for _, profile := range toolSketch.Profiles() {
		if profile.Valid && len(profile.Holes) == 0 {
			require.Nil(t, toolProfile)
			toolProfile = profile
		}
	}
	require.NotNil(t, toolProfile)

	doc := decad.New()
	plate, err := doc.Extrude(plateSketch, plateProfile, decad.Distance{
		D: units.Millimeters(8), Dir: decad.Along,
	})
	require.NoError(t, err)
	tool, err := doc.Extrude(toolSketch, toolProfile, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(9)},
		Two: decad.DistanceSide{D: units.Millimeters(1)},
	})
	require.NoError(t, err)
	return doc, plate, tool
}

func TestArrangementLimit(t *testing.T) {
	_, plate, tool := arrangementOperands(t, 17)
	_, err := decad.Cut(t.Context(), plate, tool)
	require.Error(t, err)
	require.True(t, strings.Contains(err.Error(), "arranger segments"), err)
	require.True(t, strings.Contains(err.Error(), "cap of"), err)
}

func TestArrangementWithinLimit(t *testing.T) {
	doc, plate, tool := arrangementOperands(t, 14)
	cut, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	proofkit3d.RequireSolid(t, doc, []*decad.Body{cut})
	volume, err := cut.Volume()
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	want := (10000 - 14.25*math.Pi) * 8
	require.LessOrEqual(t, math.Abs(value-want), bound+1e-8)
}
