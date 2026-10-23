package gallery_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/stretchr/testify/require"
)

// loadSpur reads the spur gear package at the revision port/go.mod pins.
func loadSpur(t *testing.T) *gallery.Model {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	return m
}

func TestLoadSpurInputs(t *testing.T) {
	m := loadSpur(t)
	require.Equal(t, "spurgear", m.Package)

	got := map[string]gallery.Input{}
	var names []string
	for _, in := range m.Inputs {
		got[in.Name] = in
		names = append(names, in.Name)
	}
	require.Equal(t, []string{
		"Module", "Teeth", "PressureAngle", "Thickness", "Samples",
		"BoreDiameter", "CapChamfer", "Placement",
	}, names)

	require.True(t, got["Module"].HasDefault)
	require.InDelta(t, 1, got["Module"].Default, 0)
	require.InDelta(t, 17, got["Teeth"].Default, 0)
	require.InDelta(t, 20*math.Pi/180, got["PressureAngle"].Default, 1e-15)
	require.InDelta(t, 10, got["Thickness"].Default, 0)
	require.InDelta(t, 15, got["Samples"].Default, 0)
	require.False(t, got["BoreDiameter"].HasDefault)
	require.False(t, got["Placement"].HasDefault)
	require.Equal(t, "int", got["Teeth"].Type)
	require.Equal(t, "*r3.Frame", got["Placement"].Type)
}

func TestLoadSpurTable(t *testing.T) {
	m := loadSpur(t)

	var names []string
	for _, p := range m.Table {
		names = append(names, p.Name)
	}
	require.Equal(t, []string{
		"module", "teeth", "pressure_angle", "thickness", "samples", "bore_diameter", "cap_chamfer",
		"tool_margin", "pitch_radius", "base_radius", "root_radius", "tip_radius",
		"involute_angle", "fillet_radius", "bore_tool_length",
	}, names)

	byName := map[string]gallery.Parameter{}
	for _, p := range m.Table {
		byName[p.Name] = p
	}
	require.Equal(t, gallery.Parameter{
		Name: "module", Kind: gallery.KindInput, Unit: "Millimeters", Field: "Module",
	}, byName["module"])
	require.Equal(t, gallery.Parameter{
		Name: "teeth", Kind: gallery.KindCount, Field: "Teeth",
	}, byName["teeth"])
	require.Equal(t, gallery.Parameter{
		Name: "pressure_angle", Kind: gallery.KindInput, Unit: "Radians", Field: "PressureAngle",
	}, byName["pressure_angle"])
	require.Equal(t, gallery.Parameter{
		Name: "base_radius", Kind: gallery.KindExpr, Unit: "Millimeter",
		Expr: "pitch_radius * cos(pressure_angle)",
	}, byName["base_radius"])
	require.Equal(t, "One", byName["involute_angle"].Unit)
}
