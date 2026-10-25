package go2py_test

import (
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

const pointSource = `
type pt struct{ x, y float64 }

func moved(p pt) pt {
	p.x = p.x + 1
	return p
}

func viaParameter() float64 {
	a := pt{1, 2}
	b := moved(a)
	return a.x + 10*b.x
}

func viaAssign() float64 {
	a := pt{1, 2}
	var b pt
	b = a
	b.x = 5
	return a.x
}

func viaIndex() float64 {
	ps := []pt{{1, 2}}
	q := ps[0]
	q.x = 9
	return ps[0].x
}

func viaRange() float64 {
	ps := []pt{{1, 2}}
	for _, q := range ps {
		q.x = 9
	}
	return ps[0].x
}

func viaAppend() float64 {
	a := pt{1, 2}
	ps := []pt{}
	ps = append(ps, a)
	a.x = 7
	return ps[0].x
}

func same() bool { return pt{1, 2} == pt{1, 2} }
`

// TestStructCopiesFollowGo requires every Go copy of a struct the code changes, through a parameter, an assignment,
// an index, a range value or an append, to be a copy in Python.
func TestStructCopiesFollowGo(t *testing.T) {
	funcs := []string{"moved", "viaParameter", "viaAssign", "viaIndex", "viaRange", "viaAppend", "same"}
	out, err := translate(t, pointSource, go2py.Spec{Funcs: funcs, Structs: []string{"pt"}, Mutable: []string{"pt"}})
	require.NoError(t, err)
	require.Equal(t, []any{21.0, 1.0, 1.0, 1.0, 1.0, true},
		evalPython(t, out, "_via_parameter()", "_via_assign()", "_via_index()", "_via_range()", "_via_append()", "_same()"))
}

func TestFieldWriteNeedsMutable(t *testing.T) {
	_, err := translate(t, pointSource, go2py.Spec{Funcs: []string{"moved"}, Structs: []string{"pt"}})
	require.ErrorContains(t, err, "namedtuple")

	bag := `
type bag struct{ xs []float64 }

func fill() bag {
	var b bag
	b.xs = []float64{1}
	return b
}

func copied() bag {
	b := fill()
	c := b
	c.xs = nil
	return b
}

func equal() bool { return fill() == fill() }
`
	_, err = translate(t, bag, go2py.Spec{Funcs: []string{"fill"}, Structs: []string{"bag"}})
	require.ErrorContains(t, err, "Spec.Filled")
	_, err = translate(t, bag, go2py.Spec{Funcs: []string{"fill"}, Structs: []string{"bag"}, Filled: []string{"bag"}})
	require.NoError(t, err)
	_, err = translate(t, bag, go2py.Spec{Funcs: []string{"fill", "copied"}, Structs: []string{"bag"}, Filled: []string{"bag"}})
	require.ErrorContains(t, err, "never copied")
	_, err = translate(t, bag, go2py.Spec{Funcs: []string{"fill", "equal"}, Structs: []string{"bag"}, Filled: []string{"bag"}})
	require.ErrorContains(t, err, "equality by field")
}
