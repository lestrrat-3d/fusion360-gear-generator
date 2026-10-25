package helical_test

import (
	"bytes"
	"encoding/json"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/helical"
	"github.com/stretchr/testify/require"
)

// fakes stand in for the Fusion runtime (_fx) and the translated spur (_spur). They record every
// call of the translated helical code, in order, with the role names it passes.
const fakes = `
import json, math, sys, types

LOG = []

class Vec:
    def __init__(self, x, y, z): self.X, self.Y, self.Z = x, y, z

class Value:
    def __init__(self, name, v): self.expression, self._v = name, v
    def base(self): return self._v

class Doc:
    def __init__(self, values): self.values = values
    def param(self, name): return Value(name, self.values[name])

class Plane:
    def __init__(self, name): self.name = name

class Body:
    def __init__(self, name): self.name = name

class Sketch:
    def __init__(self, name): self.name = name

class Profile: pass

class Cylinder:
    def __init__(self, r): self.Radius = Value('r', r)
class Other: pass
class Line: pass
class Curve: pass

class Vertex:
    def __init__(self, v): self._v = v
    def position(self): return types.SimpleNamespace(Value=self._v)

class Edge:
    def __init__(self, tag, a, b, line=True): self.tag, self._a, self._b, self._line = tag, a, b, line
    def curve(self): return Line() if self._line else Curve()
    def start(self): return Vertex(Vec(*self._a))
    def end(self): return Vertex(Vec(*self._b))
    def __hash__(self): return hash(self.tag)
    def __eq__(self, o): return self.tag == o.tag

class Face:
    def __init__(self, surface, edges): self._s, self._e = surface, edges
    def surface(self): return self._s
    def edges(self): return self._e

class Gear:
    def __init__(self, faces): self._f = faces
    def faces(self): return self._f

class Predicate:
    def __init__(self, point): self.point = point

class Query:
    def __init__(self, groups): self.groups, self.count = groups, None
    def or_(self, *p): self.groups.append(list(p)); return self
    def exactly(self, n): self.count = n; return self

fx = types.SimpleNamespace()
fx.decad_is_cylinder = lambda s: isinstance(s, Cylinder)
fx.decad_is_line3 = lambda c: isinstance(c, Line)
fx.decad_endpoint_at = lambda p: Predicate(p)
fx.decad_edges = lambda *p: Query([list(p)])
def fillet(body, sel, radius, name):
    LOG.append(['fillet', name, sel.count, len(sel.groups), radius.expression]); return body
fx.decad_fillet = fillet
def loft(doc, ls, lp, us, up, name, body):
    LOG.append(['loft', ls.name, us.name, name, body]); return Body(body)
fx.decad_loft = loft
fx.decad_mirror_frame = lambda Frame: ('mirror', Frame)
def mirrored(source, mirror, name, body):
    LOG.append(['mirror', name, body, mirror[1]]); return Body(body)
fx.decad_mirrored_copy = mirrored
def union(base, *others, name):
    LOG.append(['union', name, len(others)]); return base
fx.decad_union = union

spur = types.SimpleNamespace()
def section(world, plane, p, angle):
    LOG.append(['section', plane.name, round(angle, 6)]); return Sketch('s@' + plane.name), Profile()
spur.build_tooth_section = section
def plane_at(in_, dist):
    LOG.append(['plane_at', dist.expression]); return Plane('upper')
spur.tooth_input_plane_at = plane_at
def shifted(in_, dist):
    LOG.append(['shifted', dist.expression]); return 'mid-frame'
spur.tooth_input_shifted = shifted

_fx, _spur = fx, spur
_helical = types.SimpleNamespace(loft_tooth=loft_tooth)

values = {'helix_plane_offset': 5.0, 'helix_angle': 0.25, 'root_radius': 8.0, 'helical_fillet_radius': 0.4}
in_ = types.SimpleNamespace(Doc=Doc(values), World='world', Lower=Plane('lower'), Params='params', Frame='frame')
out = {}

LOG.clear(); loft_tooth(in_); out['loft'] = LOG[:]

def axial(tag, x, y): return Edge(tag, (x, y, 0), (x, y, 5))
a, b = axial('a', 8, 0), axial('b', 0, 8)
tilted, circle = Edge('t', (0, 0, 0), (5, 0, 5)), Edge('c', (0, 0, 0), (0, 0, 0), line=False)
gear = Gear([Face(Cylinder(8.0), [a, b, tilted, circle]), Face(Cylinder(8.0), [b]), Face(Cylinder(3.0), [axial('x', 3, 0)]), Face(Other(), [axial('y', 1, 1)])])
LOG.clear(); _fillet_root_edges(in_.Doc, gear, None); out['fillet'] = LOG[:]

LOG.clear(); _fillet_root_edges(in_.Doc, Gear([Face(Cylinder(3.0), [a])]), None); out['no root'] = LOG[:]

values['helical_fillet_radius'] = 0
LOG.clear(); _fillet_root_edges(in_.Doc, gear, None); out['no radius'] = LOG[:]

if 'chevron' in globals() or '_chevron' in globals():
    LOG.clear(); _chevron(in_); out['chevron'] = LOG[:]
json.dump(out, sys.stdout)
`

func runHelical(t *testing.T, extra string) map[string][][]any {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "helicalgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	text, err := go2py.TranspileSpec(m, helical.FeatureSpec)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", "import math\n\n"+text+"\n"+fakes+extra)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got map[string][][]any
	require.NoError(t, json.Unmarshal(raw, &got))
	return got
}

// TestTranslatedHelicalBuild runs the translated loft and root fillet against fakes and requires
// the gallery's sequence: the twisted plane, the two tooth sections, the loft, and a fillet that
// takes each axial root edge once.
func TestTranslatedHelicalBuild(t *testing.T) {
	got := runHelical(t, "")
	require.Equal(t, [][]any{
		{"plane_at", "helix_plane_offset"},
		{"section", "lower", 0.0},
		{"section", "upper", 0.25},
		{"loft", "s@lower", "s@upper", "Twisted Tooth Loft", "Tooth"},
	}, got["loft"])
	// Edges a and b are axial lines on the root cylinder, b appears on two faces, and the tilted line,
	// the circle and the edges of other faces are left out.
	require.Equal(t, [][]any{{"fillet", "Root Fillet", 2.0, 2.0, "helical_fillet_radius"}}, got["fillet"])
	require.Empty(t, got["no root"], "a gear with no root edge skips the fillet")
	require.Empty(t, got["no radius"], "a zero radius skips the fillet")
}
