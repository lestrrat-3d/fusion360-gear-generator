package go2py_test

import (
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

const splitSource = `
func split(ctx context.Context, bodies []*decad.Body, sheet *decad.Body) (int, error) {
	n := 0
	for _, b := range bodies {
		var err error
		_, err = decad.Split(ctx, b, sheet)
		if !errors.Is(err, decad.ErrUnsupported) {
			n = n + 0
		}
		if errors.Is(err, decad.ErrDegenerate) {
			continue
		}
		if err != nil {
			return 0, err
		}
		n++
	}
	return n, nil
}
`

func splitSpec() go2py.Spec {
	return go2py.Spec{
		Funcs: []string{"split"},
		Roles: map[string]go2py.Role{"split:decad.Split#1": {Feature: "Split"}},
		Catch: map[string]string{"decad.ErrDegenerate": "Degenerate"},
	}
}

// splitRuntime stands in for the runtime: decad_split raises Degenerate for a body the sheet misses and a plain
// error for a body it cannot cut.
const splitRuntime = `
import types

class Degenerate(RuntimeError):
    pass

def decad_split(body, sheet, name):
    if body == 'missed':
        raise Degenerate('missed')
    if body == 'broken':
        raise RuntimeError('broken')
    return [body + '1', body + '2']

_fx = types.SimpleNamespace(decad_split=decad_split)

def run(bodies):
    try:
        return _split(bodies, 'sheet')
    except RuntimeError as error:
        return str(error)
`

// TestCaughtErrorSkipsAsTheGalleryDoes requires errors.Is of an error in Spec.Catch to become a caught exception:
// a missed body is skipped, as the gallery's loop continues, and any other failure still raises.
func TestCaughtErrorSkipsAsTheGalleryDoes(t *testing.T) {
	out, err := translate(t, splitSource, splitSpec())
	require.NoError(t, err)
	require.Contains(t, out, "except Degenerate as _caught:")
	require.Equal(t, []any{2.0, 1.0, "broken"},
		evalPython(t, splitRuntime+"\n"+out, "run(['a', 'b'])", "run(['a', 'missed'])", "run(['a', 'broken'])"))
}

func TestCatchRefusesWhatItCannotTranslate(t *testing.T) {
	_, err := translate(t, `
func split(ctx context.Context, b, sheet *decad.Body) (int, error) {
	_, err := decad.Split(ctx, b, sheet)
	if err != nil {
		return 0, err
	}
	if errors.Is(err, decad.ErrDegenerate) {
		return 1, nil
	}
	return 2, nil
}
`, splitSpec())
	require.ErrorContains(t, err, "!= nil")

	_, err = translate(t, `
func split(ctx context.Context, b, sheet *decad.Body) (int, error) {
	var err error
	if errors.Is(err, decad.ErrDegenerate) {
		return 1, nil
	}
	_, err = decad.Split(ctx, b, sheet)
	return 2, nil
}
`, splitSpec())
	require.ErrorContains(t, err, "follows no call")

	spec := splitSpec()
	spec.Catch["decad.ErrOther"] = "Other"
	_, err = translate(t, splitSource, spec)
	require.ErrorContains(t, err, `Catch["decad.ErrOther"]`)
}
