package herringbone_test

import (
	"bytes"
	"encoding/json"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/herringbone"
	"github.com/stretchr/testify/require"
)

// fakes record the calls of the translated chevron tooth. The helical loft and the spur's tooth
// input are replaced by recorders, since port/helical and port/spur test them.
const fakes = `
import json, sys, types
LOG = []

class Value:
    def __init__(self, name): self.expression = name
class Doc:
    def param(self, name): return Value(name)
class Body:
    def __init__(self, name): self.name = name

_helical = types.SimpleNamespace(loft_tooth=lambda in_: (LOG.append(['loft_tooth']), Body('half'))[1])
def shifted(in_, dist):
    LOG.append(['shifted', dist.expression]); return 'mid-frame'
_spur = types.SimpleNamespace(tooth_input_shifted=shifted)
def mirrored(source, mirror, name, body):
    LOG.append(['mirror', source.name, mirror[1], name, body]); return Body(body)
def union(base, *others, name):
    LOG.append(['union', base.name, [o.name for o in others], name]); return base
_fx = types.SimpleNamespace(decad_mirror_frame=lambda Frame: ('mirror', Frame),
                            decad_mirrored_copy=mirrored, decad_union=union)

result = _chevron(types.SimpleNamespace(Doc=Doc()))
json.dump({'log': LOG, 'result': result.name}, sys.stdout)
`

// TestTranslatedChevronTooth requires the gallery's order: the lofted half, the mid-height frame,
// the mirrored half, and the join of the two.
func TestTranslatedChevronTooth(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "herringbonegear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	text, err := go2py.TranspileSpec(m, herringbone.FeatureSpec)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", text+"\n"+fakes)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got struct {
		Log    [][]any
		Result string
	}
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Equal(t, [][]any{
		{"loft_tooth"},
		{"shifted", "helix_plane_offset"},
		{"mirror", "half", "mid-frame", "Tooth Half Mirror", "Tooth Body (Mirrored)"},
		{"union", "half", []any{"Tooth Body (Mirrored)"}, "Tooth Halves Join"},
	}, got.Log)
	require.Equal(t, "half", got.Result)
}
