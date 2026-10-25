package gallery_test

import (
	"math"
	"os"
	"path/filepath"
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

func TestSpurDecadCalls(t *testing.T) {
	m := loadSpur(t)
	calls, err := m.DecadCalls("Build")
	require.NoError(t, err)
	require.Equal(t, []string{
		"Extrude",       // root disc
		"Extrude",       // tooth
		"PatternCopies", // teeth
		"Union",         // join
		"Fillet",        // ordinary root fillet
		"Fillet",        // embedded root fillet
		"Extrude",       // bore tool
		"Cut",           // bore
		"Chamfer",       // cap chamfer
	}, calls)
}

// loadSource parses src as a model package beside the Params type and DefaultParams function the reader requires.
func loadSource(t *testing.T, src string) *gallery.Model {
	t.Helper()
	dir := t.TempDir()
	text := "package m\n\nimport \"math\"\n\ntype Params struct{ A float64 }\n\n" +
		"func DefaultParams() Params { return Params{A: 1} }\n\n" + src
	require.NoError(t, os.WriteFile(filepath.Join(dir, "m.go"), []byte(text), 0o600))
	m, err := gallery.Load(dir)
	require.NoError(t, err)
	return m
}

func digestOf(t *testing.T, src string) string {
	t.Helper()
	d, err := loadSource(t, src).SourceDigest("f")
	require.NoError(t, err)
	return d
}

func TestSourceDigestIgnoresCommentsAndFormatting(t *testing.T) {
	base := digestOf(t, `func f(a, b float64) float64 {
	c := a + b
	return math.Max(c, 2)
}
`)
	commented := digestOf(t, `// f adds and clamps.
func f(a, b float64) float64 {
	// The sum.
	c := a + b /* inline */

	// Clamp it.
	return math.Max(c, 2)
}
`)
	require.Equal(t, base, commented, "comments must not change the digest")
	reformatted := digestOf(t, `func f(a,
	b float64) float64 {
	c := a +
		b; return math.Max(
		c,
		2,
	) }
`)
	require.Equal(t, base, reformatted, "line breaks, spacing and trailing commas must not change the digest")
	changed := digestOf(t, `func f(a, b float64) float64 {
	c := a - b
	return math.Max(c, 2)
}
`)
	require.NotEqual(t, base, changed, "a changed expression must change the digest")

	m := loadSpur(t)
	a, err := m.SourceDigest("Build", "cutBore")
	require.NoError(t, err)
	c, err := m.SourceDigest("cutBore", "Build")
	require.NoError(t, err)
	require.NotEqual(t, a, c)
	_, err = m.SourceDigest("noSuchFunction")
	require.Error(t, err)
}

// TestTableReadsTheParamsArgumentByItsName requires the table reader to find the Params argument by its declared
// name, to read a value computed in defineParameters as a snapshot, and to refuse any other selector.
func TestTableReadsTheParamsArgumentByItsName(t *testing.T) {
	m := loadSource(t, `
var other struct{ X float64 }

func defineParameters(t *param.Table, q Params) error {
	d := resolve(q)
	t.SetValue("a", units.Millimeters(q.A))
	t.SetValue("b", units.Millimeters(d.B))
	return nil
}
`)
	require.NoError(t, m.TableErr)
	require.Equal(t, []gallery.Parameter{
		{Name: "a", Kind: gallery.KindInput, Unit: "Millimeters", Field: "A"},
		{Name: "b", Kind: gallery.KindSnapshot, Unit: "Millimeters", Field: "d.B"},
	}, m.Table)

	m = loadSource(t, `
var other struct{ X float64 }

func defineParameters(t *param.Table, q Params) error {
	t.SetValue("x", units.Millimeters(other.X))
	return nil
}
`)
	require.ErrorContains(t, m.TableErr, "other.X")
}
