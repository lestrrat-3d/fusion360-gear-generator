package spur_test

import (
	"bytes"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

// definitions lists, for each class of a Python module, the names it defines (methods, class
// attributes and attributes assigned to self), and the module-level names.
const definitions = `
import ast, json, sys
tree = ast.parse(sys.stdin.read())
top, classes = set(), {}
for node in tree.body:
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
        top.add(node.name)
    elif isinstance(node, ast.Assign):
        for t in node.targets:
            if isinstance(t, ast.Name):
                top.add(t.id)
    if isinstance(node, ast.ClassDef):
        names = set()
        for sub in ast.walk(node):
            if isinstance(sub, ast.FunctionDef):
                names.add(sub.name)
            if isinstance(sub, ast.Assign):
                for t in sub.targets:
                    if isinstance(t, ast.Name):
                        names.add(t.id)
                    if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) and t.value.id == 'self':
                        names.add(t.attr)
        classes[node.name] = sorted(names)
json.dump({'top': sorted(top), 'classes': classes}, sys.stdout)
`

// runtimeClass maps the Go type a method belongs to onto the class of lib/geargen/decadfusion.py.
var runtimeClass = map[string]string{
	"decad.Document": "Runtime", "sketch.World": "Runtime", "units.Value": "Value", "decad.Body": "Body",
	"decad.EdgeQuery": "Selector", "decad.Edge": "Edge", "decad.Vertex": "Vertex", "decad.Position": "Position",
	"decad.Face": "Face", "decad.Cylinder": "Cylinder", "decad.Centroid": "Centroid",
	"sketch.Sketch": "Sketch", "sketch.Profile": "Profile", "r3.Frame": "Frame", "r3.Vec": "Vec",
}

// TestRuntimeDefinesEveryOperationTheTranslatorAccepts requires the Fusion runtime to define a
// function, method, field or constant for each entry of the translator's API table, so a call the
// translator emits can never reach a name that does not exist.
func TestRuntimeDefinesEveryOperationTheTranslatorAccepts(t *testing.T) {
	source, err := os.ReadFile(filepath.Join("..", "..", "lib", "geargen", "decadfusion.py"))
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", definitions)
	cmd.Stdin = bytes.NewReader(source)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var defs struct {
		Top     []string
		Classes map[string][]string
	}
	require.NoError(t, json.Unmarshal(raw, &defs))

	var missing []string
	for _, entry := range go2py.API() {
		if entry.Receiver == "" {
			if !contains(defs.Top, entry.Python) {
				missing = append(missing, entry.Go)
			}
			continue
		}
		class, ok := runtimeClass[entry.Receiver]
		if !ok {
			missing = append(missing, entry.Go+" (no runtime class for "+entry.Receiver+")")
			continue
		}
		if !contains(defs.Classes[class], entry.Python) {
			missing = append(missing, entry.Go+" (want "+class+"."+entry.Python+")")
		}
	}
	require.Empty(t, missing, "decadfusion.py lacks these operations of the API table")
}

func contains(list []string, s string) bool {
	for _, x := range list {
		if x == s {
			return true
		}
	}
	return false
}
