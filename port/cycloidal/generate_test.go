package cycloidal_test

import (
	"encoding/json"
	"os"
	"path/filepath"
	"regexp"
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

func TestPinsMatchTheGalleryTable(t *testing.T) {
	_, m, _, _ := generate(t)
	require.NoError(t, m.TableErr)
	names := map[string]bool{}
	for _, e := range m.Table {
		names[e.Name] = true
	}
	for name := range cycloidal.RegisteredParams {
		require.Truef(t, names[name], "gallery table lacks %s", name)
	}
}

func TestGenerateRefusesChangedGalleryCode(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	reviewed.Digest = "0000"
	_, err := cycloidal.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "gearport accept")
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
	created := regexp.MustCompile(`(?m)^\s*(\w+)\s*=\s*[\w.]*(?:sketches|constructionPlanes)\.add\(`)
	matches := created.FindAllStringSubmatch(text, -1)
	require.NotEmpty(t, matches)
	for _, m := range matches {
		named := regexp.MustCompile(`(?m)^\s*` + regexp.QuoteMeta(m[1]) + `\.name = `)
		require.Truef(t, named.MatchString(text), "%s is created without a name", m[1])
	}
}
