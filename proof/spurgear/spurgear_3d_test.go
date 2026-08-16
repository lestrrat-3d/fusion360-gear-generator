package spurgear_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit3d"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/spurgear"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	p3Module       = "module"
	p3Teeth        = "teeth"
	p3Pressure     = "pressureAngle"
	p3Steps        = "involuteSteps"
	p3RootChords   = "rootChords"
	p3Thickness    = "thickness"
	p3BoreDiameter = "boreDiameter"
)

func TestSpurGearSolid(t *testing.T) {
	proofkit3d.Run(t, []proofkit3d.Case{
		{Name: "m1_t12", Params: spur3Params(1, 12, 5, 0)},
		{Name: "m1_t45_embedded", Params: spur3Params(1, 45, 5, 0)},
	}, buildSpurSolid, assertSpurSolid)
}

func TestSpurGearBore(t *testing.T) {
	proofkit3d.RunSolid(t, []proofkit3d.Case{
		{Name: "m2_t20_bore6", Params: spur3Params(2, 20, 8, 6)},
	}, buildSpurSolid, assertSpurSolid)
}

func buildSpurSolid(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	outline, err := spurgear.NewOutline(
		p[p3Module], p[p3Teeth], p[p3Pressure], int(p[p3Steps]), int(p[p3RootChords]),
	)
	if errors.Is(err, spurgear.ErrEmbedded) {
		proofkit3d.Unmodelled(t, "%v", err)
	}
	require.NoError(t, err)

	s, profile, err := outline.Sketch()
	require.NoError(t, err)
	body, err := doc.Extrude(s, profile, decad.Distance{
		D:   units.Millimeters(p[p3Thickness]),
		Dir: decad.Along,
	})
	require.NoError(t, err)

	bore := p[p3BoreDiameter]
	if bore <= 0 {
		return []*decad.Body{body}
	}
	toolSketch, toolProfile := boreProfile(t, bore)
	tool, err := doc.Extrude(toolSketch, toolProfile, decad.Distance{
		D:   units.Millimeters(p[p3Thickness] + 2),
		Dir: decad.Along,
	})
	require.NoError(t, err)
	cut, err := decad.Cut(body, tool)
	require.NoError(t, err)
	return []*decad.Body{cut}
}

func assertSpurSolid(t *testing.T, _ *decad.Document, bodies []*decad.Body, p map[string]float64) {
	body := bodies[len(bodies)-1]
	volume, err := body.Volume()
	require.NoError(t, err)

	outline, err := spurgear.NewOutline(
		p[p3Module], p[p3Teeth], p[p3Pressure], int(p[p3Steps]), int(p[p3RootChords]),
	)
	require.NoError(t, err)
	want := outline.Area * p[p3Thickness]
	if p[p3BoreDiameter] > 0 {
		want -= math.Pi * math.Pow(p[p3BoreDiameter]/2, 2) * p[p3Thickness]
	}
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9,
		"volume %.12g does not enclose expected %.12g with bound %.12g",
		volume.Value.Base(), want, volume.Bound.Base())

	box, err := body.Bounds()
	require.NoError(t, err)
	tip := outline.Dimensions.Tip
	require.GreaterOrEqual(t, box.Min.X, -tip-box.Bound.Base())
	require.LessOrEqual(t, box.Max.X, tip+box.Bound.Base())
	require.GreaterOrEqual(t, box.Min.Y, -tip-box.Bound.Base())
	require.LessOrEqual(t, box.Max.Y, tip+box.Bound.Base())
	require.InDelta(t, 0, box.Min.Z, box.Bound.Base()+1e-9)
	require.InDelta(t, p[p3Thickness], box.Max.Z, box.Bound.Base()+1e-9)
}

func boreProfile(t *testing.T, diameter float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, diameter/2)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func spur3Params(module, teeth, thickness, bore float64) map[string]float64 {
	return map[string]float64{
		p3Module:       module,
		p3Teeth:        teeth,
		p3Pressure:     20,
		p3Steps:        15,
		p3RootChords:   8,
		p3Thickness:    thickness,
		p3BoreDiameter: bore,
	}
}
