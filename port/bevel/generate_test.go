package bevel_test

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
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
	require.Contains(t, out, "handInput.listItems.add(_HAND_RIGHT, True)")
	require.Contains(t, out, "addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)")
}

// TestFixedToothParametersMatchTheGallery requires the pressure angle and sample count
// that lib/geargen/spurproxy.py fixes for the bevel tooth to equal the gallery defaults.
func TestFixedToothParametersMatchTheGallery(t *testing.T) {
	_, m, _, _ := generate(t)
	proxy, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "spurproxy.py"))
	require.NoError(t, err)
	angle := regexp.MustCompile(`pressureAngleRad=math\.radians\(([0-9.]+)\)`).FindSubmatch(proxy)
	steps := regexp.MustCompile(`involuteSteps=([0-9]+)`).FindSubmatch(proxy)
	require.NotNil(t, angle)
	require.NotNil(t, steps)
	degrees, err := strconv.ParseFloat(string(angle[1]), 64)
	require.NoError(t, err)
	for _, in := range m.Inputs {
		switch in.Name {
		case "PressureAngle":
			require.InDelta(t, in.Default, degrees*math.Pi/180, 1e-12)
		case "Samples":
			require.EqualValues(t, mustInt(t, string(steps[1])), in.Default)
		}
	}
}

func mustInt(t *testing.T, s string) int {
	n, err := strconv.Atoi(s)
	require.NoError(t, err)
	return n
}

func TestGenerateRefusesChangedGalleryCode(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	reviewed.Digest = "0000"
	_, err := bevel.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "gearport accept")
}

func TestGenerateRefusesAnUnmappedInput(t *testing.T) {
	_, m, reviewed, fragment := generate(t)
	m.Inputs = append(m.Inputs, gallery.Input{Name: "NewOption", Type: "float64"})
	_, err := bevel.Generate(m, "v0", fragment, reviewed)
	require.ErrorContains(t, err, "NewOption")
}
