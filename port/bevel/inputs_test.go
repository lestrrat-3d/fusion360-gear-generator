package bevel_test

import (
	"bytes"
	"encoding/json"
	"os/exec"
	"testing"

	"github.com/stretchr/testify/require"
)

// toothTypeDriver imports the generated add-in with the Fusion modules stubbed, reads the gallery Params from a
// fake dialog holding the defaults and a negative Cutter Radius, and runs the translated derive on them, for a
// Straight and a Spiral Tooth Type. It prints the spiral fields read and the error, if any.
const toothTypeDriver = `
import importlib, importlib.util, json, sys, types
from unittest import mock
root = sys.argv[1]
real_stdout = sys.stdout
sys.stdout = sys.stderr
adsk = types.ModuleType('adsk'); adsk.__path__ = []
for name in ('core', 'fusion'):
    stub = mock.MagicMock(name='adsk.' + name)
    sys.modules['adsk.' + name] = stub
    setattr(adsk, name, stub)
sys.modules['adsk'] = adsk
spec = importlib.util.spec_from_file_location('addin', root + '/__init__.py', submodule_search_locations=[root])
sys.modules['addin'] = importlib.util.module_from_spec(spec)
bevel = importlib.import_module('addin.lib.geargen.bevelgear')
selected = lambda name: types.SimpleNamespace(selectedItem=types.SimpleNamespace(name=name))
value = lambda v: types.SimpleNamespace(value=v)
out = []
for tooth in ('Straight', 'Spiral'):
    items = {
        bevel.INPUT_ID_MODULE: value(1), bevel.INPUT_ID_DRIVING_TEETH: value(31), bevel.INPUT_ID_PINION_TEETH: value(31),
        bevel.INPUT_ID_SHAFT_ANGLE: value(1.5707963267948966), bevel.INPUT_ID_FACE_WIDTH: value(0),
        bevel.INPUT_ID_BORE_ENABLE: value(False), bevel.INPUT_ID_DRIVING_BORE: value(0), bevel.INPUT_ID_PINION_BORE: value(0),
        bevel.INPUT_ID_TOOTH_TYPE: selected(tooth), bevel.INPUT_ID_SPIRAL_ANGLE: value(0.6), bevel.INPUT_ID_HAND: selected('Left'),
        bevel.INPUT_ID_CUTTER_RADIUS: value(-0.5), bevel.INPUT_ID_TOOTH_SPACING: value(0), bevel.INPUT_ID_TOE_EXTENSION: value(0),
        bevel.INPUT_ID_DRIVING_TOE_RADIUS: value(0), bevel.INPUT_ID_PINION_TOE_RADIUS: value(0),
        bevel.INPUT_ID_DRIVING_BASE_HEIGHT: value(0), bevel.INPUT_ID_PINION_BASE_HEIGHT: value(0),
    }
    inputs = types.SimpleNamespace(itemById=lambda i, items=items: items[i])
    p = bevel._params_of_inputs(inputs)
    error = ''
    try:
        bevel._derive(p)
    except RuntimeError as e:
        error = str(e)
    out.append({'angle': p.SpiralAngle, 'hand': p.SpiralHand, 'cutter': p.CutterRadius, 'error': error})
json.dump(out, real_stdout)
`

// TestStraightTeethIgnoreTheSpiralInputs requires a straight pair to build with the gallery defaults of the three
// spiral-only inputs, so a negative Cutter Radius typed while Spiral was selected does not fail it, while a spiral
// pair still reads and checks them.
func TestStraightTeethIgnoreTheSpiralInputs(t *testing.T) {
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", toothTypeDriver, "../..")
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	type result struct {
		Angle  float64 `json:"angle"`
		Hand   string  `json:"hand"`
		Cutter float64 `json:"cutter"`
		Error  string  `json:"error"`
	}
	var got []result
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Equal(t, []result{
		{Angle: 0, Hand: "right", Cutter: 0, Error: ""},
		{Angle: 0.6, Hand: "left", Cutter: -5, Error: "length inputs cannot be negative"},
	}, got)
}
