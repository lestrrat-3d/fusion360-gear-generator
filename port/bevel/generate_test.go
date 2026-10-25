package bevel_test

import (
	"encoding/json"
	"maps"
	"os"
	"path/filepath"
	"slices"
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

// TestApexScrapRemovalNamedByRole requires the removal of the apex-side slab, the last decad.Remove of
// sliceSpiralPointTooth as translated, to carry its own role name.
func TestApexScrapRemovalNamedByRole(t *testing.T) {
	out, _, _, _ := generate(t)
	require.Contains(t, out, `_fx.decad_remove(ranked[0].body, name="Spiral Apex Scrap Removal")`)
}

// TestGenerateRefusesChangedCreatingCalls requires the port to stop when the creating calls of a translated function
// differ from reviewed.json, as when the gallery swaps two calls of one operation and the role names would follow
// the positions.
func TestGenerateRefusesChangedCreatingCalls(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	calls := slices.Clone(reviewed.Creations["cutBore"])
	require.NotEmpty(t, calls)
	calls[0], calls[len(calls)-1] = calls[len(calls)-1], calls[0]
	require.NotEqual(t, reviewed.Creations["cutBore"], calls)
	reviewed.Creations = maps.Clone(reviewed.Creations)
	reviewed.Creations["cutBore"] = calls
	_, err := bevel.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "cutBore")
	require.ErrorContains(t, err, "gearport accept bevelgear")

	reviewed.Creations = nil
	_, err = bevel.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "records no creating calls")
}

// TestMissedSlicePlaneIsSkipped requires the spiral slicer to catch decad's ErrDegenerate from a split, as the
// gallery's errors.Is does, and to remove the sheet and move to the next plane.
func TestMissedSlicePlaneIsSkipped(t *testing.T) {
	out, _, _, _ := generate(t)
	require.Contains(t, out, "from . import decaderrors as _errors\n")
	require.Contains(t, out, "                try:\n"+
		"                    cut = _fx.decad_split(inner, sheet, name=\"Spiral Slice\")\n"+
		"                    err = None\n"+
		"                except _errors.Degenerate as _caught:\n"+
		"                    err = _caught\n")
	require.Contains(t, out, "            if isinstance(err, _errors.Degenerate):\n"+
		"                _fx.decad_remove(sheet, name=\"Missed Slice Sheet Removal\")\n"+
		"                continue\n")
}
