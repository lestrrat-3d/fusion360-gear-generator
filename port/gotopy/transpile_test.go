package gotopy_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/decad-gallery/models/spurgear"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gotopy"
	"github.com/stretchr/testify/require"
)

// helperFuncs are the pure spur helpers the Fusion tooth drawer needs, in dependency order.
var helperFuncs = []string{"rotate", "involutePoint", "flankBeta", "sampleFlanks"}

func transpileSpur(t *testing.T) string {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	out, err := gotopy.Transpile(m, "_", []string{"point"}, helperFuncs)
	require.NoError(t, err)
	return out
}

func TestTranspileSpurHelpersText(t *testing.T) {
	out := transpileSpur(t)
	require.Contains(t, out, "_Point = collections.namedtuple('Point', ['x', 'y'])")
	require.Contains(t, out, "def _involute_point(base, r):")
	require.Contains(t, out, "t = math.tan(math.acos(ratio))")
	require.Contains(t, out, "def _sample_flanks(pitch, base, start, tip, teeth, samples, angle):")
	require.Contains(t, out, "for i in range(0, samples):")
}

type flankCase struct {
	Module        float64 `json:"module"`
	Teeth         int     `json:"teeth"`
	PressureAngle float64 `json:"pressure_angle"`
	Samples       int     `json:"samples"`
	Angle         float64 `json:"angle"`
}

// driver runs the transpiled helpers in Python for every case and prints the flanks.
const driver = `
import json, sys
cases = json.load(sys.stdin)
out = []
for c in cases:
    pitch = c['module'] * c['teeth'] / 2
    base = pitch * math.cos(c['pressure_angle'])
    tip = pitch + c['module']
    left, right = _sample_flanks(pitch, base, base, tip, float(c['teeth']), c['samples'], c['angle'])
    out.append({'left': [list(p) for p in left], 'right': [list(p) for p in right]})
json.dump(out, sys.stdout)
`

// TestTranspiledFlanksMatchGo runs the Python translation and the Go original on the
// same inputs and requires the sampled flank points to agree.
func TestTranspiledFlanksMatchGo(t *testing.T) {
	cases := []flankCase{
		{1, 17, 20 * math.Pi / 180, 15, 0},
		{2, 40, 20 * math.Pi / 180, 15, 0.3},
		{1.5, 12, 14.5 * math.Pi / 180, 9, 2.1},
		{1, 60, 25 * math.Pi / 180, 21, -1.2},
	}
	in, err := json.Marshal(cases)
	require.NoError(t, err)

	py := "import collections\nimport math\n\n" + transpileSpur(t) + "\n" + driver
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", py)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())

	var got []struct {
		Left  [][2]float64 `json:"left"`
		Right [][2]float64 `json:"right"`
	}
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(cases))

	for i, c := range cases {
		pitch := c.Module * float64(c.Teeth) / 2
		base := pitch * math.Cos(c.PressureAngle)
		tip := pitch + c.Module
		left, right, err := spurgear.InvoluteFlanks(pitch, base, tip, float64(c.Teeth), c.Samples, c.Angle)
		require.NoError(t, err)
		requireClose(t, left, got[i].Left, i, "left")
		requireClose(t, right, got[i].Right, i, "right")
	}
}

func requireClose(t *testing.T, want, got [][2]float64, caseIndex int, side string) {
	t.Helper()
	require.Len(t, got, len(want), "case %d %s", caseIndex, side)
	for k := range want {
		require.InDelta(t, want[k][0], got[k][0], 1e-9, "case %d %s point %d x", caseIndex, side, k)
		require.InDelta(t, want[k][1], got[k][1], 1e-9, "case %d %s point %d y", caseIndex, side, k)
	}
}

func TestTranspileRefusesWhatItCannotTranslate(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)

	// Build uses contexts, documents and method calls outside the subset.
	_, err = gotopy.Transpile(m, "_", []string{"point"}, []string{"Build"})
	require.Error(t, err)
}

func TestTranspileSketchRefusesWhatItDoesNotKnow(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)

	args := []gotopy.StepArg{
		{Name: "s", Type: "*sketch.Sketch"}, {Name: "p", Type: "Params"},
		{Name: "d", Type: "dimensions"}, {Name: "angle", Type: "float64"},
	}
	spec := gotopy.Spec{
		Prefix:   "_",
		Structs:  []string{"point", "dimensions", "toothPoints"},
		TypeOnly: []string{"Params"},
		Funcs: []string{
			"rotate", "involutePoint", "flankBeta", "sampleFlanks", "rootContact", "addToothPoints",
		},
	}

	// A stop call the function never makes lets the translation run into the solver and
	// profile checks, which the Python runtime does not perform, so the port refuses.
	spec.Steps = []gotopy.Steps{{Name: "buildToothSection", PyName: "x", Args: args, StopAt: "noSuchCall"}}
	_, err = gotopy.TranspileSpec(m, spec)
	require.ErrorContains(t, err, "unsupported error check")

	// A function that is not in the list cannot be called from a translated one.
	spec.Funcs = []string{"rotate", "involutePoint", "flankBeta", "sampleFlanks", "addToothPoints"}
	spec.Steps = nil
	_, err = gotopy.TranspileSpec(m, spec)
	require.ErrorContains(t, err, "rootContact")
}
