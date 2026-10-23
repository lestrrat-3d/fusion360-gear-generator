package fusionexpr_test

import (
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/fusionexpr"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/stretchr/testify/require"
)

// kindOfUnit maps the units constructor a gallery table entry names to a kind.
func kindOfUnit(unit string) fusionexpr.Kind {
	switch unit {
	case "Millimeters", "Millimeter":
		return fusionexpr.KindLength
	case "Radians", "Radian":
		return fusionexpr.KindAngle
	}
	return fusionexpr.KindNone
}

func TestTranslateSpurTable(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)

	kinds := map[string]fusionexpr.Kind{}
	for _, p := range m.Table {
		kinds[p.Name] = kindOfUnit(p.Unit)
	}

	want := map[string]fusionexpr.Result{
		"tool_margin": {Format: "1", Kind: fusionexpr.KindNone},
		"pitch_radius": {
			Format: "{} * {} / 2", Names: []string{"module", "teeth"}, Kind: fusionexpr.KindLength,
		},
		"base_radius": {
			Format: "{} * cos({})", Names: []string{"pitch_radius", "pressure_angle"},
			Kind: fusionexpr.KindLength,
		},
		"root_radius": {
			Format: "({} * {} - 2.5 * {}) / 2", Names: []string{"module", "teeth", "module"},
			Kind: fusionexpr.KindLength,
		},
		"tip_radius": {
			Format: "({} * {} + 2 * {}) / 2", Names: []string{"module", "teeth", "module"},
			Kind: fusionexpr.KindLength,
		},
		"involute_angle": {
			Format: "tan({}) - ({} / 1 deg) * pi / 180", Names: []string{"pressure_angle", "pressure_angle"},
			Kind: fusionexpr.KindNone,
		},
		"fillet_radius": {
			Format: "0.9 * {} * (pi / {} - 2 * {}) / 2", Names: []string{"root_radius", "teeth", "involute_angle"},
			Kind: fusionexpr.KindLength,
		},
		"bore_tool_length": {
			Format: "{} + 2 * {}", Names: []string{"thickness", "tool_margin"}, Kind: fusionexpr.KindLength,
		},
	}
	seen := 0
	for _, p := range m.Table {
		if p.Kind != gallery.KindExpr {
			continue
		}
		got, err := fusionexpr.Translate(p.Expr, kinds)
		require.NoError(t, err, p.Name)
		require.Equal(t, want[p.Name], got, p.Name)
		seen++
	}
	require.Len(t, want, seen)
}

func TestTranslateRefusesWhatItHasNoRuleFor(t *testing.T) {
	kinds := map[string]fusionexpr.Kind{"a": fusionexpr.KindLength, "b": fusionexpr.KindAngle}
	for _, tc := range []struct{ name, expr string }{
		{"unknown name", "c * 2"},
		{"unknown function", "hypot(a, a)"},
		{"power operator", "a ^ 2"},
		{"modulo", "a % 2"},
		{"length plus angle", "a + b"},
		{"deg of a length", "deg(a)"},
		{"two arguments", "cos(a, a)"},
	} {
		_, err := fusionexpr.Translate(tc.expr, kinds)
		require.Error(t, err, tc.name)
	}
}
