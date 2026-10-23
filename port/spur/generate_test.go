package spur_test

import (
	"bytes"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

// generate runs the port on the pinned gallery spur with the stored fragment.
func generate(t *testing.T) (string, *gallery.Model, spur.Reviewed) {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)

	fragment, err := os.ReadFile(filepath.Join("fragments", "spurgear.py.frag"))
	require.NoError(t, err)
	raw, err := os.ReadFile("reviewed.json")
	require.NoError(t, err)
	var reviewed spur.Reviewed
	require.NoError(t, json.Unmarshal(raw, &reviewed))

	out, err := spur.Generate(m, src.Version, string(fragment), reviewed)
	require.NoError(t, err)
	return out, m, reviewed
}

// TestCommittedAddInIsFresh requires lib/geargen/spurgear.py to be exactly what the
// port generates now. A hand edit, a gallery bump or a fragment change that was
// not followed by `gearport gen spurgear` fails here.
func TestCommittedAddInIsFresh(t *testing.T) {
	out, _, _ := generate(t)
	have, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "spurgear.py"))
	require.NoError(t, err)
	require.Equal(t, out, string(have), "run gearport gen spurgear")
}

// surfaceCheck parses the generated module and reports the classes it defines and
// the PARAM_ and INPUT_ID_ names it uses without defining.
const surfaceCheck = `
import ast, json, sys
tree = ast.parse(sys.stdin.read())
defined, used, classes = set(), set(), set()
for node in ast.walk(tree):
    if isinstance(node, ast.ClassDef):
        classes.add(node.name)
    if isinstance(node, ast.Assign):
        for target in node.targets:
            if isinstance(target, ast.Name):
                defined.add(target.id)
    if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
        if node.id.startswith(('PARAM_', 'INPUT_ID_')):
            used.add(node.id)
json.dump({'classes': sorted(classes), 'defined': sorted(defined), 'used': sorted(used)}, sys.stdout)
`

func TestGeneratedModuleKeepsTheSurfaceOtherGearsUse(t *testing.T) {
	out, _, _ := generate(t)

	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", surfaceCheck)
	cmd.Stdin = bytes.NewReader([]byte(out))
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())

	var got struct {
		Classes, Defined, Used []string
	}
	require.NoError(t, json.Unmarshal(raw, &got))

	// helicalgear, herringbonegear, bevelgear and commands/spurgear import these.
	require.Subset(t, got.Classes, []string{
		"SpurGearCommandInputsConfigurator", "SpurGearGenerationContext",
		"SpurGearGenerator", "SpurGearInvoluteToothDesignGenerator",
	})
	require.Subset(t, got.Defined, []string{"PARAM_MODULE", "PARAM_TOOTH_NUMBER", "PARAM_THICKNESS"})
	require.Subset(t, got.Defined, got.Used, "a PARAM_ or INPUT_ID_ name is used but never defined")
}

func TestGenerateRefusesChangedGalleryCode(t *testing.T) {
	_, m, reviewed := generate(t)
	reviewed.Digest = "0000"
	fragment, err := os.ReadFile(filepath.Join("fragments", "spurgear.py.frag"))
	require.NoError(t, err)
	_, err = spur.Generate(m, "v0", string(fragment), reviewed)
	require.ErrorContains(t, err, "gearport accept")
}

func TestGenerateRefusesAnUnfilledFragment(t *testing.T) {
	_, m, reviewed := generate(t)
	_, err := spur.Generate(m, "v0", "x = 1\n", reviewed)
	require.ErrorContains(t, err, "markers")
}
