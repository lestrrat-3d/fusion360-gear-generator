package go2py_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

// sketchSpec turns on the translator's sketch mode, which string literals and fmt need.
func sketchSpec(funcs ...string) go2py.Spec {
	return go2py.Spec{Funcs: funcs, TypeOnly: []string{"Params"}}
}

func TestIntWithFloatConstantIsRefused(t *testing.T) {
	_, err := translate(t, `func half(n int) int { return n / 2.0 }`, go2py.Spec{Funcs: []string{"half"}})
	require.ErrorContains(t, err, "floating-point constant")
	_, err = translate(t, `func twice(n int) int { return 2.0 * n }`, go2py.Spec{Funcs: []string{"twice"}})
	require.ErrorContains(t, err, "floating-point constant")

	out, err := translate(t, `func half(x float64) float64 { return x / 2.0 }

func third() float64 { return 1 / 3.0 }`, go2py.Spec{Funcs: []string{"half", "third"}})
	require.NoError(t, err)
	require.Equal(t, []any{2.5, 1.0 / 3.0}, evalPython(t, out, "_half(5)", "_third()"))
}

func TestRemainderFollowsGo(t *testing.T) {
	out, err := translate(t, `
func rem(a, b int) int { return a % b }

func even(a int) bool { return a%2 == 0 }

func wrap(xs []int) int {
	n := 0
	for i := range xs {
		n = n + xs[(i+1)%len(xs)]
	}
	return n
}
`, go2py.Spec{Funcs: []string{"rem", "even", "wrap"}})
	require.NoError(t, err)
	require.Contains(t, out, "return (a % 2) == 0", "a remainder compared with zero agrees in both languages")
	require.Contains(t, out, "xs[(i + 1) % len(xs)]", "a remainder of counters agrees in both languages")
	require.Equal(t, []any{float64(-7 % 3), float64(7 % -3), true, false, 6.0},
		evalPython(t, out, "_rem(-7, 3)", "_rem(7, -3)", "_even(-4)", "_even(-3)", "_wrap([1, 2, 3])"))
}

func TestInfSignsFollowGo(t *testing.T) {
	_, err := translate(t, `func inf(s int) float64 { return math.Inf(s) }`, go2py.Spec{Funcs: []string{"inf"}})
	require.ErrorContains(t, err, "constant sign")
	_, err = translate(t, `func isInf(x float64, s int) bool { return math.IsInf(x, s) }`, go2py.Spec{Funcs: []string{"isInf"}})
	require.ErrorContains(t, err, "constant sign")

	out, err := translate(t, `
func up(x float64) bool { return math.IsInf(x, 1) }

func down(x float64) bool { return math.IsInf(x, -1) }

func either(x float64) bool { return math.IsInf(x, 0) }

func low() float64 { return math.Inf(-1) }
`, go2py.Spec{Funcs: []string{"up", "down", "either", "low"}})
	require.NoError(t, err)
	require.Equal(t, []any{false, true, true, true},
		evalPython(t, out, "_up(-math.inf)", "_down(-math.inf)", "_either(-math.inf)", "_low() == -math.inf"))
}

func TestAnonymousStructNamesAreUnambiguous(t *testing.T) {
	out, err := translate(t, `
func both() float64 {
	s := 0.0
	for _, r := range []struct{ ab, c float64 }{{1, 2}} {
		s = s + r.ab + 10*r.c
	}
	for _, r := range []struct{ a, bc float64 }{{3, 4}} {
		s = s + 100*r.a + 1000*r.bc
	}
	return s
}
`, go2py.Spec{Funcs: []string{"both"}})
	require.NoError(t, err)
	require.Equal(t, []any{4321.0}, evalPython(t, out, "_both()"))
}

func TestRawStringIsAPythonLiteral(t *testing.T) {
	out, err := translate(t, "func path() string { return `C:\\dir \"x\" 'y'` }", sketchSpec("path"))
	require.NoError(t, err)
	require.Equal(t, []any{`C:\dir "x" 'y'`}, evalPython(t, out, "_path()"))
}

func TestFormatVerbsFollowGo(t *testing.T) {
	for verb, src := range map[string]string{
		"%t":         `func f(b bool) string { return fmt.Sprintf("%t", b) }`,
		"%q":         `func f(s string) string { return fmt.Sprintf("%q", s) }`,
		"%T":         `func f(x float64) string { return fmt.Sprintf("%T", x) }`,
		"%v float":   `func f(x float64) string { return fmt.Sprintf("%v", x) }`,
		"%d float":   `func f(x float64) string { return fmt.Sprintf("%d", x) }`,
		"%s int":     `func f(n int) string { return fmt.Sprintf("%s", n) }`,
		"%5d":        `func f(n int) string { return fmt.Sprintf("%5d", n) }`,
		"Errorf %t":  `func f(b bool) (int, error) { return 0, fmt.Errorf("bad %t", b) }`,
		"Errorf %w":  `func f(err error) (int, error) { return 0, fmt.Errorf("bad %w", err) }`,
		"two verbs":  `func f(n int) string { return fmt.Sprintf("%d %d", n) }`,
		"no verb":    `func f(n int) string { return fmt.Sprintf("n", n) }`,
		"open verb":  `func f(n int) string { return fmt.Sprintf("%", n) }`,
		"%.2d":       `func f(n int) string { return fmt.Sprintf("%.2d", n) }`,
		"%v of 3.0":  `func f() string { return fmt.Sprintf("%v", 3.0) }`,
		"%x float":   `func f(x float64) string { return fmt.Sprintf("%x", x) }`,
		"%g string":  `func f(s string) string { return fmt.Sprintf("%g", s) }`,
		"%s float64": `func f(x float64) string { return fmt.Sprintf("%s", x) }`,
	} {
		t.Run(verb, func(t *testing.T) {
			_, err := translate(t, src, sketchSpec("f"))
			require.Error(t, err)
		})
	}

	out, err := translate(t, `func f(n int, s string, x float64) string {
	return fmt.Sprintf("%d %s %.2f %g %x %v %% %.3g", n, s, x, x, n, s, x)
}`, sketchSpec("f"))
	require.NoError(t, err)
	want := fmt.Sprintf("%d %s %.2f %g %x %v %% %.3g", 42, "ab", 2.5, 2.5, 42, "ab", 2.5)
	require.Equal(t, []any{want}, evalPython(t, out, `_f(42, "ab", 2.5)`))
}

func TestNilSliceComparisonIsRefused(t *testing.T) {
	_, err := translate(t, `func empty(xs []float64) bool { return xs == nil }`, go2py.Spec{Funcs: []string{"empty"}})
	require.ErrorContains(t, err, "nil")
	_, err = translate(t, `func set(m map[int]int) bool { return nil != m }`, go2py.Spec{Funcs: []string{"set"}})
	require.ErrorContains(t, err, "nil")
}

func TestArrayStartsWithItsZeroValues(t *testing.T) {
	out, err := translate(t, `
type pair struct{ v [2]float64 }

func arr() float64 {
	var a [3]float64
	a[2] = 1
	var p pair
	p.v[1] = 4
	return a[2] + a[0] + p.v[1]
}
`, go2py.Spec{Funcs: []string{"arr"}, Structs: []string{"pair"}, Mutable: []string{"pair"}})
	require.NoError(t, err)
	require.Equal(t, []any{5.0}, evalPython(t, out, "_arr()"))
}
