package bevel_test

import (
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/bevel"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

func generate(t *testing.T) (string, *gallery.Model, spur.Reviewed, string) {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "bevelgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	fragment, err := os.ReadFile(filepath.Join("fragments", "bevelgear.py.frag"))
	require.NoError(t, err)
	raw, err := os.ReadFile("reviewed.json")
	require.NoError(t, err)
	var reviewed spur.Reviewed
	require.NoError(t, json.Unmarshal(raw, &reviewed))
	out, err := bevel.Generate(m, src.Version, string(fragment), reviewed)
	require.NoError(t, err)
	return out, m, reviewed, string(fragment)
}

// TestCommittedAddInIsFresh requires lib/geargen/bevelgear.py to be exactly what the
// port generates now.
func TestCommittedAddInIsFresh(t *testing.T) {
	out, _, _, _ := generate(t)
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "bevelgear.py"))
	require.NoError(t, err)
	require.Equal(t, out, string(have), "run gearport gen bevelgear")
}

func TestBuildIdMatchesTheFile(t *testing.T) {
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "bevelgear.py"))
	require.NoError(t, err)
	require.Contains(t, string(have), "GEAR_BUILD_ID = '"+spur.FileID(string(have))+"'")
}

func TestDialogDefaultsComeFromTheGallery(t *testing.T) {
	out, _, _, _ := generate(t)
	require.Contains(t, out, "createByString('90 deg')")
	require.Contains(t, out, "createByString('35 deg')")
	// Straight teeth are the default tooth type, and the dropdown comes before the spiral-only inputs.
	require.Contains(t, out, "toothInput.listItems.add(_TOOTH_STRAIGHT, True)")
	require.Contains(t, out, "toothInput.listItems.add(_TOOTH_SPIRAL, False)")
	require.Less(t, strings.Index(out, "INPUT_ID_TOOTH_TYPE, 'Tooth Type'"), strings.Index(out, "INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle'"))
	require.Contains(t, out, "SpiralAngle=inputs.itemById(INPUT_ID_SPIRAL_ANGLE).value if _is_spiral(inputs) else 0.0")
	require.Contains(t, out, "handInput.listItems.add(_HAND_RIGHT, True)")
	require.Contains(t, out, "addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)")
}

func TestGenerateRefusesAnUnmappedInput(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	m.Inputs = append(m.Inputs, gallery.Input{Name: "NewOption", Type: "float64"})
	_, err := bevel.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "NewOption")
}
