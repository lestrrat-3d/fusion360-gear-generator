package go2py_test

import (
	"bytes"
	"encoding/json"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

// translate translates the functions funcs of a model made of src, with the rest of spec.
func translate(t *testing.T, src string, spec go2py.Spec) (string, error) {
	t.Helper()
	if spec.Prefix == "" {
		spec.Prefix = "_"
	}
	return go2py.TranspileSpec(loadModel(t, src), spec)
}

// evalPython runs the translated code and prints the JSON value of each expression, which may call it.
func evalPython(t *testing.T, code string, exprs ...string) []any {
	t.Helper()
	program := "import collections, json, math\n\n" + code + "\n\nprint(json.dumps([" + strings.Join(exprs, ", ") + "]))\n"
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", program)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var out []any
	require.NoError(t, json.Unmarshal(raw, &out))
	return out
}

func TestShadowReadAgainIsRefused(t *testing.T) {
	for name, src := range map[string]string{
		"after the block": `
func shadow(c bool) float64 {
	r := 1.0
	if c {
		r := 2.0
		r = r * 2
	}
	return r
}
`,
		"in a later turn": `
func shadow(xs []float64) float64 {
	s := 0.0
	for _, x := range xs {
		s = s + x
		if x > 0 {
			s := x * 2
			s = s + 1
		}
	}
	return 0
}
`,
	} {
		t.Run(name, func(t *testing.T) {
			_, err := translate(t, src, go2py.Spec{Funcs: []string{"shadow"}})
			require.ErrorContains(t, err, "hides")
		})
	}
}

func TestShadowNeverReadAgainTranslates(t *testing.T) {
	out, err := translate(t, `
func count(xs []int) int {
	n := 0
	ok := len(xs) > 0
	if !ok {
		return 0
	}
	for _, x := range xs {
		if ok := x > 1; ok {
			n = n + x
		}
	}
	return n
}
`, go2py.Spec{Funcs: []string{"count"}, TypeOnly: []string{"Params"}})
	require.NoError(t, err)
	require.Equal(t, []any{5.0}, evalPython(t, out, "_count([1, 2, 3])"))
}

func TestFloatLoopVariableIsRefused(t *testing.T) {
	_, err := translate(t, `
func sum() float64 {
	s := 0.0
	for x := 0.0; x < 1; x += 0.25 {
		s = s + x
	}
	return s
}
`, go2py.Spec{Funcs: []string{"sum"}})
	require.ErrorContains(t, err, "floating-point")
}

func TestFloatCounterStepByOneTranslatesAndContinueRunsTheStep(t *testing.T) {
	out, err := translate(t, `
func sum(home float64) float64 {
	s := 0.0
	for m := home - 1; m <= home+1; m++ {
		if m == home {
			continue
		}
		s = s + m
	}
	return s
}
`, go2py.Spec{Funcs: []string{"sum"}})
	require.NoError(t, err)
	// m runs home-1, home, home+1; the middle turn continues, so the sum is (home-1) + (home+1).
	require.Equal(t, []any{8.0}, evalPython(t, out, "_sum(4.0)"))
}

func TestIntegerDivisionTruncatesTowardZero(t *testing.T) {
	out, err := translate(t, `
func steps(total, parts int) int {
	return total / parts
}
`, go2py.Spec{Funcs: []string{"steps"}})
	require.NoError(t, err)
	require.Equal(t, []any{22.0}, evalPython(t, out, "_steps(720, 32)"))
	require.Equal(t, []any{-2.0}, evalPython(t, out, "_steps(-5, 2)"), "Go truncates -2.5 toward zero; Python's // would give -3")
}

func TestLoopBoundTheBodyChangesIsRefused(t *testing.T) {
	cases := map[string]string{
		"a length the body grows": `
func grow(xs []int) []int {
	for i := 0; i < len(xs); i++ {
		if i < 3 {
			xs = append(xs, i)
		}
	}
	return xs
}
`,
		"the loop variable": `
func grow(xs []int) []int {
	for i := 0; i < 10; i++ {
		i = i + 1
	}
	return xs
}
`,
		"a call": `
func grow(xs []int) []int {
	for i := 0; i < limit(); i++ {
		xs = append(xs, i)
	}
	return xs
}
`,
	}
	for name, src := range cases {
		t.Run(name, func(t *testing.T) {
			src += "\nfunc limit() int { return 3 }\n"
			_, err := translate(t, src, go2py.Spec{Funcs: []string{"limit", "grow"}})
			require.ErrorContains(t, err, "counted loop")
		})
	}
}

func TestMapIncrementTranslates(t *testing.T) {
	out, err := translate(t, `
func count(keys []int) map[int]int {
	m := map[int]int{}
	for _, k := range keys {
		m[k]++
	}
	return m
}
`, go2py.Spec{Funcs: []string{"count"}})
	require.NoError(t, err)
	require.Equal(t, []any{map[string]any{"1": 2.0, "2": 1.0}}, evalPython(t, out, "_count([1, 1, 2])"))
}

func TestBreakInASwitchIsRefused(t *testing.T) {
	for name, src := range map[string]string{
		"switch": `
func walk(xs []int) int {
	n := 0
	for _, x := range xs {
		switch {
		case x > 10:
			break
		}
		n = n + 1
	}
	return n
}
`,
		"label": `
func walk(xs []int) int {
	n := 0
outer:
	for _, x := range xs {
		if x > 10 {
			break outer
		}
		n = n + 1
	}
	return n
}
`,
	} {
		t.Run(name, func(t *testing.T) {
			_, err := translate(t, src, go2py.Spec{Funcs: []string{"walk"}})
			require.Error(t, err)
		})
	}
}

func TestClosureIncrementDeclaresNonlocal(t *testing.T) {
	out, err := translate(t, `
func counted(xs []int) int {
	n := 0
	inc := func() int {
		n++
		return n
	}
	for range xs {
		inc()
	}
	return n
}
`, go2py.Spec{Funcs: []string{"counted"}})
	require.NoError(t, err)
	require.Equal(t, []any{3.0}, evalPython(t, out, "_counted([1, 2, 3])"))
}
