package spur_test

import (
	"bytes"
	"encoding/json"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad-gallery/models/spurgear"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// validateDriver runs the translated validate on every case and prints its message, or "" for valid inputs. Its
// derive stands for the Fusion user parameters: it evaluates the gallery's own parameter table, in order.
const validateDriver = `
import json, math, sys, types

data = json.load(sys.stdin)

def derive(p):
    names = {'cos': math.cos, 'tan': math.tan, 'pi': math.pi, 'deg': math.degrees}
    for e in data['table']:
        if e['Kind'] in ('input', 'count'):
            names[e['Name']] = float(getattr(p, e['Field']))
        elif e['Kind'] == 'expr':
            names[e['Name']] = eval(e['Expr'], {'__builtins__': {}}, names)
    return types.SimpleNamespace(pitch=names['pitch_radius'], base=names['base_radius'], root=names['root_radius'],
                                 tip=names['tip_radius'], fillet=names['fillet_radius'])

_fx = types.SimpleNamespace(derive=derive)
out = []
for c in data['cases']:
    p = types.SimpleNamespace(Placement=None, **c)
    try:
        _validate(p)
        out.append('')
    except RuntimeError as error:
        out.append(str(error))
json.dump(out, sys.stdout)
`

// TestTranslatedValidateMatchesTheGallery requires the translated validate to accept the inputs the gallery accepts
// and to refuse the others with the gallery's own message. The gallery's checks run first in its exported
// BuildToothSection, which is how the test reaches them.
func TestTranslatedValidateMatchesTheGallery(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	require.NoError(t, m.TableErr)
	text, err := go2py.TranspileSpec(m, spur.FeatureSpec)
	require.NoError(t, err)

	base := spurgear.DefaultParams()
	change := func(f func(p *spurgear.Params)) spurgear.Params {
		p := base
		f(&p)
		return p
	}
	// Lengths are chosen so Go's shortest %g and Python's six-digit %g print them alike.
	cases := []spurgear.Params{
		base,
		change(func(p *spurgear.Params) { p.Module, p.Teeth, p.Thickness = 2, 40, 16 }),
		change(func(p *spurgear.Params) { p.BoreDiameter, p.CapChamfer = 5, 0.5 }),
		change(func(p *spurgear.Params) { p.Module = 0 }),
		change(func(p *spurgear.Params) { p.Thickness = -1 }),
		change(func(p *spurgear.Params) { p.Teeth = 2 }),
		change(func(p *spurgear.Params) { p.Samples = 2 }),
		change(func(p *spurgear.Params) { p.PressureAngle = 0 }),
		change(func(p *spurgear.Params) { p.PressureAngle = 2 }),
		change(func(p *spurgear.Params) { p.BoreDiameter = -1 }),
		change(func(p *spurgear.Params) { p.CapChamfer = -0.5 }),
		change(func(p *spurgear.Params) { p.BoreDiameter = 14.5 }),
		change(func(p *spurgear.Params) { p.BoreDiameter = 20 }),
	}
	type row struct {
		Module        float64 `json:"Module"`
		Teeth         int     `json:"Teeth"`
		PressureAngle float64 `json:"PressureAngle"`
		Thickness     float64 `json:"Thickness"`
		Samples       int     `json:"Samples"`
		BoreDiameter  float64 `json:"BoreDiameter"`
		CapChamfer    float64 `json:"CapChamfer"`
	}
	rows := make([]row, len(cases))
	for i, p := range cases {
		rows[i] = row{p.Module, p.Teeth, p.PressureAngle, p.Thickness, p.Samples, p.BoreDiameter, p.CapChamfer}
	}
	in, err := json.Marshal(map[string]any{"table": m.Table, "cases": rows})
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", "import collections\nimport math\n\n"+text+"\n"+validateDriver)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got []string
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(cases))

	for i, p := range cases {
		world := sketch.NewWorld()
		_, _, err := spurgear.BuildToothSection(t.Context(), world, world.XY(), p, 0)
		want := ""
		if err != nil && strings.HasPrefix(err.Error(), "spurgear:") {
			want = err.Error()
		}
		require.Equal(t, want, got[i], "case %d", i)
	}
	require.Empty(t, got[0], "the default gear is valid")
	require.NotEmpty(t, got[len(got)-1], "a bore wider than the root circle is refused")
}

// TestGeneratedSpurValidatesBeforeBuilding requires the add-in to run the translated checks right after it reads
// the dialog, before the first sketch.
func TestGeneratedSpurValidatesBeforeBuilding(t *testing.T) {
	out, _, _ := generate(t)
	require.Contains(t, out, "        self.processInputs(inputs)\n        self.validateGallery()\n")
	require.Contains(t, out, "        _validate(self.galleryParams(_MILLIMETRE))\n")
	require.Contains(t, out, "\ndef _validate(p):\n")
}
