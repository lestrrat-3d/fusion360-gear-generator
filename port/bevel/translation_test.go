package bevel_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	gbevel "github.com/lestrrat-3d/decad-gallery/models/bevelgear"
	"github.com/stretchr/testify/require"
)

// boreDriver imports the generated add-in with the Fusion modules stubbed and runs the translated
// ResolveBoreDiameter for every case. That chain runs the translated derive, the parameter table and the blank section.
const boreDriver = `
import contextlib, importlib, importlib.util, json, sys, types
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
out = []
for c in json.load(sys.stdin):
    try:
        out.append({'value': bevel.resolve_bore_diameter(bevel._Params(**c, Placement=None)), 'error': ''})
    except RuntimeError as error:
        out.append({'value': 0, 'error': str(error)})
json.dump(out, real_stdout)
`

type boreResult struct {
	Value float64 `json:"value"`
	Error string  `json:"error"`
}

// TestTranslatedBoreMatchesTheGallery requires the translated ResolveBoreDiameter to give the gallery's diameter, or
// its error message, for valid and invalid pairs.
func TestTranslatedBoreMatchesTheGallery(t *testing.T) {
	base := gbevel.DefaultParams()
	base.SpiralAngle = 0
	change := func(f func(p *gbevel.Params)) gbevel.Params {
		p := base
		f(&p)
		return p
	}
	cases := []gbevel.Params{
		base,
		change(func(p *gbevel.Params) { p.Teeth, p.MateTeeth, p.ShaftAngle = 16, 12, 70*math.Pi/180 }),
		change(func(p *gbevel.Params) { p.Teeth, p.MateTeeth = 4, 4 }),
		change(func(p *gbevel.Params) { p.Module, p.BoreDiameter = 2.5, 6 }),
		change(func(p *gbevel.Params) { p.FaceWidth = 3 }),
		change(func(p *gbevel.Params) { p.ToeRadius, p.ToeExtension = 8, 50 }),
		change(func(p *gbevel.Params) { p.Module = 0 }),
		change(func(p *gbevel.Params) { p.ShaftAngle = 0.1 }),
		change(func(p *gbevel.Params) { p.Teeth = 2 }),
		change(func(p *gbevel.Params) { p.FaceWidth = 1e6 }),
		change(func(p *gbevel.Params) { p.BoreDiameter = 1e6 }),
		change(func(p *gbevel.Params) { p.BaseHeight = 1e6 }),
		change(func(p *gbevel.Params) { p.ToeRadius = -1 }),
	}
	type input struct {
		Module, ShaftAngle, FaceWidth, PressureAngle, BoreDiameter, MateBoreDiameter, SpiralAngle, CutterRadius float64
		ToothSpacing, ToeExtension, ToeRadius, MateToeRadius, BaseHeight, MateBaseHeight                        float64
		Teeth, MateTeeth, Samples                                                                               int
		EnableBore                                                                                              bool
		SpiralHand                                                                                              string
	}
	inputs := make([]input, len(cases))
	for i, p := range cases {
		inputs[i] = input{
			Module: p.Module, ShaftAngle: p.ShaftAngle, FaceWidth: p.FaceWidth, PressureAngle: p.PressureAngle,
			BoreDiameter: p.BoreDiameter, MateBoreDiameter: p.MateBoreDiameter, SpiralAngle: p.SpiralAngle,
			CutterRadius: p.CutterRadius, ToothSpacing: p.ToothSpacing, ToeExtension: p.ToeExtension,
			ToeRadius: p.ToeRadius, MateToeRadius: p.MateToeRadius, BaseHeight: p.BaseHeight,
			MateBaseHeight: p.MateBaseHeight, Teeth: p.Teeth, MateTeeth: p.MateTeeth, Samples: p.Samples,
			EnableBore: p.EnableBore, SpiralHand: p.SpiralHand,
		}
	}
	// The Python keyword names are the gallery's field names.
	raw, err := json.Marshal(inputs)
	require.NoError(t, err)
	var generic []map[string]any
	require.NoError(t, json.Unmarshal(raw, &generic))
	renamed := make([]map[string]any, len(generic))
	for i, c := range generic {
		renamed[i] = map[string]any{}
		for k, v := range c {
			renamed[i][k] = v
		}
	}
	in, err := json.Marshal(renamed)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", boreDriver, "../..")
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	out, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got []boreResult
	require.NoError(t, json.Unmarshal(out, &got))
	require.Len(t, got, len(cases))
	for i, p := range cases {
		want, wantErr := gbevel.ResolveBoreDiameter(p)
		if wantErr != nil {
			require.Equal(t, wantErr.Error(), got[i].Error, "case %d", i)
			continue
		}
		require.Empty(t, got[i].Error, "case %d", i)
		require.InDelta(t, want, got[i].Value, 1e-9, "case %d", i)
	}
}
