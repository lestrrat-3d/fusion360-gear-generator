package cycloidal_test

import (
	"encoding/json"
	"maps"
	"os"
	"path/filepath"
	"regexp"
	"slices"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/cycloidal"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

func generate(t *testing.T) (string, *gallery.Model, spur.Reviewed, string) {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "cycloidal")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	fragment, err := os.ReadFile(filepath.Join("fragments", "cycloidal.py.frag"))
	require.NoError(t, err)
	raw, err := os.ReadFile("reviewed.json")
	require.NoError(t, err)
	var reviewed spur.Reviewed
	require.NoError(t, json.Unmarshal(raw, &reviewed))
	out, err := cycloidal.Generate(m, src.Version, string(fragment), reviewed)
	require.NoError(t, err)
	return out, m, reviewed, string(fragment)
}

// TestCommittedAddInIsFresh requires lib/geargen/cycloidal.py to be exactly what the
// port generates now.
func TestCommittedAddInIsFresh(t *testing.T) {
	out, _, _, _ := generate(t)
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "cycloidal.py"))
	require.NoError(t, err)
	require.Equal(t, out, string(have), "run gearport gen cycloidal")
}

func TestBuildIdMatchesTheFile(t *testing.T) {
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "cycloidal.py"))
	require.NoError(t, err)
	require.Contains(t, string(have), "GEAR_BUILD_ID = '"+spur.FileID(string(have))+"'")
}

func TestDialogDefaultsComeFromTheGallery(t *testing.T) {
	out, _, _, _ := generate(t)
	require.Contains(t, out, "createByReal(16))")
	require.Contains(t, out, "createByReal(to_cm(90))")
	require.Contains(t, out, "discCountInput.listItems.add('1', True)")
	require.Contains(t, out, "discCountInput.listItems.add('2', False)")
}

// TestEveryTableEntryHasAFusionParameter requires a constant, a registration and a lookup entry for
// every name of the gallery's parameter table, so the translated build can read each one.
func TestEveryTableEntryHasAFusionParameter(t *testing.T) {
	out, m, _, _ := generate(t)
	require.NoError(t, m.TableErr)
	for _, e := range m.Table {
		c := cycloidal.ParamConst(e.Name)
		require.Contains(t, out, c+" = '"+cycloidal.ParamName(e.Name)+"'", e.Name)
		require.Contains(t, out, "'"+e.Name+"': "+c+",", e.Name)
		if e.Kind != gallery.KindInput && e.Kind != gallery.KindCount {
			require.Contains(t, out, "            "+c+", adsk.core.ValueInput.create", e.Name)
		}
	}
}

func TestGenerateRefusesAnUnmappedInput(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	m.Inputs = append(m.Inputs, gallery.Input{Name: "NewOption", Type: "float64"})
	_, err := cycloidal.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "NewOption")
}

// TestEveryCreatedObjectIsNamedByRole requires every sketch and construction plane the
// add-in creates to be given a name, so the Fusion browser shows roles and not Sketch1.
func TestEveryCreatedObjectIsNamedByRole(t *testing.T) {
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "cycloidal.py"))
	require.NoError(t, err)
	text := string(have)
	created := regexp.MustCompile(`(?m)^\s*(\w+)\s*(?::[^=]+)?=\s*[\w.]*(?:sketches|constructionPlanes)\.add\(`)
	matches := created.FindAllStringSubmatch(text, -1)
	require.NotEmpty(t, matches)
	for _, m := range matches {
		named := regexp.MustCompile(`(?m)^\s*(?:self\.)?` + regexp.QuoteMeta(m[1]) + `\.name = `)
		require.Truef(t, named.MatchString(text), "%s is created without a name", m[1])
	}
}

// TestGenerateRefusesChangedCreatingCalls requires the port to stop when the creating calls of a translated function
// differ from reviewed.json, as when the gallery swaps two calls of one operation and the role names would follow
// the positions.
func TestGenerateRefusesChangedCreatingCalls(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	calls := slices.Clone(reviewed.Creations["buildHousing"])
	require.NotEmpty(t, calls)
	calls[1], calls[4] = calls[4], calls[1]
	require.NotEqual(t, reviewed.Creations["buildHousing"], calls)
	reviewed.Creations = maps.Clone(reviewed.Creations)
	reviewed.Creations["buildHousing"] = calls
	_, err := cycloidal.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "buildHousing")
	require.ErrorContains(t, err, "gearport accept cycloidal")

	reviewed.Creations = nil
	_, err = cycloidal.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "records no creating calls")
}
