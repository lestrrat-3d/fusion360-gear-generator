package herringbone_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

// addinValidateDriver imports the generated add-ins with the Fusion modules stubbed, and runs validateGallery of
// the spur, helical and herringbone generators on user parameters given in the dialog's units: millimetres,
// radians and counts. It stores a length parameter in centimetres, as Fusion does, and computes the derived ones
// from the gallery's parameter table. It prints each message, or "" for accepted inputs.
const addinValidateDriver = `
import importlib, importlib.util, json, math, sys, types
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
gears = {
    'spur': importlib.import_module('addin.lib.geargen.spurgear').SpurGearGenerator,
    'helical': importlib.import_module('addin.lib.geargen.helicalgear').HelicalGearGenerator,
    'herringbone': importlib.import_module('addin.lib.geargen.herringbonegear').HerringboneGearGenerator,
}
data = json.load(sys.stdin)

def parameters(inputs):
    names = {'cos': math.cos, 'tan': math.tan, 'pi': math.pi, 'deg': math.degrees}
    for e in data['table']:
        names[e['Name']] = inputs[e['Name']] if e['Kind'] in ('input', 'count') else eval(e['Expr'], {'__builtins__': {}}, names)
    values = {}
    for m in data['mapping']:
        values[m['Name']] = names[m['Table']] / 10 if m['Unit'] == 'mm' else names[m['Table']]
    values['HelixAngle'] = inputs['helix_angle']
    return values

units = {m['Name']: m['Unit'] for m in data['mapping']}
units['HelixAngle'] = 'rad'

out = []
for c in data['cases']:
    values = parameters(c['inputs'])
    g = object.__new__(gears[c['gear']])
    g.getComponent = lambda: types.SimpleNamespace(name='Gear')
    g.parameterName = lambda constant: constant
    g.getParameter = lambda constant, values=values: types.SimpleNamespace(value=values[constant], unit=units.get(constant, ''))
    try:
        g.validateGallery()
        out.append('')
    except RuntimeError as error:
        out.append(str(error))
json.dump(out, real_stdout)
`

// TestAddInsValidateTheirDefaults runs the generated add-ins' validateGallery on the gallery defaults and on a few
// refused inputs. The herringbone generator inherits the helical check, as the gallery's herringbone Build calls
// helicalgear.ValidateHelixAngle. A refused length is reported in millimetres, the unit the dialog shows.
func TestAddInsValidateTheirDefaults(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	require.NoError(t, m.TableErr)

	deg := math.Pi / 180
	defaults := map[string]float64{
		"module": 1, "teeth": 17, "pressure_angle": 20 * deg, "thickness": 10, "samples": 15,
		"bore_diameter": 0, "cap_chamfer": 0, "helix_angle": 14.5 * deg,
	}
	with := func(changes map[string]float64) map[string]float64 {
		out := map[string]float64{}
		for k, v := range defaults {
			out[k] = v
		}
		for k, v := range changes {
			out[k] = v
		}
		return out
	}
	type testCase struct {
		Gear   string             `json:"gear"`
		Inputs map[string]float64 `json:"inputs"`
		want   string
	}
	cases := []testCase{
		{Gear: "spur", Inputs: defaults},
		{Gear: "helical", Inputs: defaults},
		{Gear: "herringbone", Inputs: defaults},
		{Gear: "helical", Inputs: with(map[string]float64{"module": 2, "teeth": 40, "thickness": 16, "helix_angle": 20 * deg})},
		{Gear: "herringbone", Inputs: with(map[string]float64{"module": 2, "teeth": 40, "thickness": 20, "helix_angle": 20 * deg})},
		{Gear: "spur", Inputs: with(map[string]float64{"bore_diameter": 20}),
			want: "spurgear: bore diameter 20 must be below root diameter 14.5"},
		{Gear: "helical", Inputs: with(map[string]float64{"helix_angle": 50 * deg}),
			want: "helix angle must be from 1 to 45 degrees"},
		{Gear: "herringbone", Inputs: with(map[string]float64{"helix_angle": 0}),
			want: "zero helix angle is a spur gear; use spurgear.Build"},
		{Gear: "herringbone", Inputs: with(map[string]float64{"teeth": 2}),
			want: "spurgear: teeth 2 must be at least 3"},
	}
	in, err := json.Marshal(map[string]any{"table": m.Table, "mapping": spur.Params, "cases": cases})
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", addinValidateDriver, "../..")
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got []string
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(cases))
	for i, c := range cases {
		require.Equal(t, c.want, got[i], "case %d (%s)", i, c.Gear)
	}
}
