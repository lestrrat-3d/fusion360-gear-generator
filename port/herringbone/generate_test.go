package herringbone_test

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/herringbone"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

func generate(t *testing.T) (string, *gallery.Model, spur.Reviewed, string) {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "herringbonegear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	fragment, err := os.ReadFile(filepath.Join("fragments", "herringbonegear.py.frag"))
	require.NoError(t, err)
	raw, err := os.ReadFile("reviewed.json")
	require.NoError(t, err)
	var reviewed spur.Reviewed
	require.NoError(t, json.Unmarshal(raw, &reviewed))
	out, err := herringbone.Generate(m, src.Version, string(fragment), reviewed)
	require.NoError(t, err)
	return out, m, reviewed, string(fragment)
}

// TestCommittedAddInIsFresh requires lib/geargen/herringbonegear.py to be exactly what
// the port generates now.
func TestCommittedAddInIsFresh(t *testing.T) {
	out, _, _, _ := generate(t)
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "herringbonegear.py"))
	require.NoError(t, err)
	require.Equal(t, out, string(have), "run gearport gen herringbonegear")
}

func TestBuildIdMatchesTheFile(t *testing.T) {
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "herringbonegear.py"))
	require.NoError(t, err)
	require.Contains(t, string(have), "GEAR_BUILD_ID = '"+spur.FileID(string(have))+"'")
}

func TestGenerateRefusesChangedGalleryCode(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	reviewed.Digest = "0000"
	_, err := herringbone.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "gearport accept")
}
