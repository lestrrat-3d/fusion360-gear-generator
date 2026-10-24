package spur_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/stretchr/testify/require"
)

// fakeRuntime stands in for lib/geargen/decadfusion.py. It records every call of the translated
// build, in order, with the role names it passes. Its edges are the root corners of the teeth the
// tooth sketch drew, so the translated fillet code can find them the way it finds them in Fusion.
const fakeRuntime = `
import collections, json, math, sys, types

LOG = []

class Vec:
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z
    def add(self, o): return Vec(self.x + o.x, self.y + o.y, self.z + o.z)
    def sub(self, o): return Vec(self.x - o.x, self.y - o.y, self.z - o.z)
    def scale(self, k): return Vec(self.x * k, self.y * k, self.z * k)
    def len(self): return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

class Frame:
    def __init__(self, o, u, v): self._o, self._u, self._v = o, u, v
    def origin(self): return self._o
    def u(self): return self._u
    def v(self): return self._v
    def n(self): return Vec(0, 0, 1)

class Value:
    def __init__(self, name, v): self.expression, self._v = name, v
    def base(self): return self._v

class Body:
    def __init__(self, name): self.name = name
    def is_solid(self): return True

class Profile:
    Valid = True
    def __init__(self, n): self.Outer = list(range(n))

class Plane: pass

class Sketch:
    def __init__(self, name):
        self.name = name
        self.points, self.curves = [], 0
    def origin(self): return 'origin'
    def dimension_point(self, x, y):
        self.points.append((x, y)); return len(self.points) - 1
    def dimension_arc_end(self, x, y):
        self.points.append((x, y)); return len(self.points) - 1
    def create_line(self, a, b): self.curves += 1
    def create_arc(self, c, a, b): self.curves += 1
    def create_fit_spline(self, *p): self.curves += 1
    def create_circle(self, c, r):
        self.curves += 1; return 'circle'
    def add_constraint(self, c): pass
    def bind(self, c, table, name): LOG.append(['bind', name])
    def solve(self): LOG.append(['solve', self.name])
    def profiles(self): return [Profile(self.curves)]

TOOTH = {}

class Runtime:
    def __init__(self, values): self.values = values
    def param(self, name): return Value(name, self.values[name])
    def params(self): return None
    def xy(self): return Plane()
    def create_sketch(self, plane, name):
        LOG.append(['sketch', name]); s = Sketch(name); TOOTH.setdefault(name, s); TOOTH[name] = s; return s
    def create_plane_from_frame(self, frame, name):
        LOG.append(['plane', name]); return Plane()

class Predicate:
    def __init__(self, kind, arg=None): self.kind, self.arg = kind, arg

class Edge:
    def __init__(self, p): self._p = p
    def start(self): return types.SimpleNamespace(position=lambda: types.SimpleNamespace(Value=self._p))

class Selector:
    def __init__(self, groups, count=None, edges=None): self.groups, self.count, self.edges = groups, count, edges
    def or_(self, *p): return Selector(self.groups + [list(p)], self.count, self.edges)
    def exactly(self, n): return Selector(self.groups, n, self.edges)
    def select_edges(self, body): return EDGES()

def EDGES():
    sk = TOOTH['Tooth Profile']
    n = len(sk.points) // 2 - 1
    frame, teeth = RUN['frame'], RUN['teeth']
    out = [Edge(Vec(99, 99, 99))]
    for k in range(teeth):
        a = 2 * math.pi * k / teeth
        for (x, y) in sk.points[-2:]:
            c, s = math.cos(a), math.sin(a)
            out.append(Edge(frame.origin().add(frame.u().scale(x * c - y * s)).add(frame.v().scale(x * s + y * c))))
    return out

RUN = {}

def make_fx():
    fx = types.SimpleNamespace()
    fx.DECAD_ALONG = 1
    fx.decad_distance = lambda D, Dir: types.SimpleNamespace(D=D, Dir=Dir)
    fx.decad_circular_pattern = lambda Center, Axis, Count: types.SimpleNamespace(Center=Center, Axis=Axis, Count=Count)
    fx.r3_new_vec = lambda x, y, z: Vec(x, y, z)
    fx.r3_new_frame = lambda o, u, v: Frame(o, u, v)
    fx.sketch_new_world = lambda: RUNTIME
    fx.sketch_new_radius = lambda c, v: ('radius', v)
    fx.sketch_new_diameter = lambda c, v: ('diameter', v)
    fx.sketchcheck_verify_fully_constrained = lambda s: None
    fx.decad_parallel_to = lambda a: Predicate('parallel')
    fx.decad_endpoint_at = lambda p: Predicate('endpoint', p)
    fx.decad_concave = lambda: Predicate('concave')
    fx.decad_cap_start = lambda b: Predicate('cap_start')
    fx.decad_cap_end = lambda b: Predicate('cap_end')
    fx.decad_outer_loop_of = lambda s: Predicate('outer_loop', s.kind)
    fx.decad_edges = lambda *p: Selector([list(p)])
    def extrude(doc, sk, prof, extent, name, body):
        LOG.append(['extrude', name, body, extent.D.expression]); return Body(body)
    def pattern(tooth, pat, name):
        LOG.append(['pattern', name, pat.Count]); return [Body('copy') for _ in range(pat.Count - 1)]
    def union(base, *others, name):
        LOG.append(['union', name, len(others)]); return base
    def cut(base, tool, name):
        LOG.append(['cut', name]); return base
    def fillet(body, sel, radius, name):
        LOG.append(['fillet', name, sel.count, len(sel.groups), radius.expression]); return body
    def chamfer(body, sel, setback, name):
        LOG.append(['chamfer', name, len(sel.groups), setback.expression]); return body
    fx.decad_extrude, fx.decad_pattern_copies, fx.decad_union = extrude, pattern, union
    fx.decad_cut, fx.decad_fillet, fx.decad_chamfer = cut, fillet, chamfer
    return fx

_fx = make_fx()
RUNTIME = None
_Run = collections.namedtuple('Run', ['doc', 'world', 'p', 'd', 'frame', 'lower', 'extent', 'teeth'])

out = []
for c in json.load(sys.stdin):
    LOG.clear(); TOOTH.clear()
    m, teeth = c['module'], c['teeth']
    pitch = m * teeth / 2
    base = pitch * math.cos(c['pressure_angle'])
    d = _Dimensions(pitch, base, pitch - 1.25 * m, pitch + m, 0.4 * m if c['fillet'] else 0.0)
    values = {'thickness': 5.0, 'tool_margin': 0.1, 'bore_tool_length': 5.2, 'fillet_radius': d.fillet,
              'cap_chamfer': c['chamfer'], 'root_radius': d.root}
    RUNTIME = Runtime(values)
    frame = Frame(Vec(1, 2, 3), Vec(1, 0, 0), Vec(0, 1, 0))
    Params = collections.namedtuple('P', ['Teeth', 'Samples', 'BoreDiameter', 'CapChamfer', 'Placement'])
    p = Params(teeth, c['samples'], c['bore'], c['chamfer'], frame)
    run = _Run(doc=RUNTIME, world=RUNTIME, p=p, d=d, frame=frame, lower=Plane(),
               extent=types.SimpleNamespace(D=RUNTIME.param('thickness'), Dir=1), teeth=teeth)
    RUN.update(frame=frame, teeth=teeth)
    _assemble(run)
    out.append(LOG[:])
json.dump(out, sys.stdout)
`

type buildCase struct {
	Module        float64 `json:"module"`
	Teeth         int     `json:"teeth"`
	PressureAngle float64 `json:"pressure_angle"`
	Samples       int     `json:"samples"`
	Bore          float64 `json:"bore"`
	Chamfer       float64 `json:"chamfer"`
	Fillet        bool    `json:"fillet"`
}

// TestTranslatedBuildCreatesTheGalleryFeaturesInOrder runs the translated build against a fake
// runtime and requires the sketches, features, role names and counts to be the gallery's.
func TestTranslatedBuildCreatesTheGalleryFeaturesInOrder(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	text, err := go2py.TranspileSpec(m, spur.FeatureSpec)
	require.NoError(t, err)

	cases := []buildCase{
		{1, 17, 20 * math.Pi / 180, 15, 0, 0, true},
		{1, 12, 20 * math.Pi / 180, 9, 3, 0.2, true},
		{1, 17, 20 * math.Pi / 180, 15, 0, 0, false},
		{1, 50, 20 * math.Pi / 180, 15, 3, 0, true},
	}
	in, err := json.Marshal(cases)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", "import collections\nimport math\n\n"+text+"\n"+fakeRuntime)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got [][][]any
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(cases))

	ordinary := func(teeth int, fillet bool, bore, chamfer bool) [][]any {
		n := float64(teeth)
		log := [][]any{
			{"sketch", "Tooth Profile"}, {"solve", "Tooth Profile"},
			{"extrude", "Tooth Extrude", "Tooth", "thickness"},
			{"sketch", "Root Disc Profile"}, {"bind", "root_radius"}, {"solve", "Root Disc Profile"},
			{"extrude", "Root Disc Extrude", "Gear Body", "thickness"},
			{"pattern", "Tooth Pattern", n},
			{"union", "Teeth Join", n},
		}
		if fillet {
			log = append(log, []any{"fillet", "Root Fillet", 2 * n, 2 * n, "fillet_radius"})
		}
		if bore {
			log = append(log,
				[]any{"plane", "Bore Tool Plane"}, []any{"sketch", "Bore Profile"}, []any{"bind", "bore_diameter"},
				[]any{"solve", "Bore Profile"}, []any{"extrude", "Bore Tool Extrude", "Bore Tool", "bore_tool_length"},
				[]any{"cut", "Bore Cut"})
		}
		if chamfer {
			log = append(log, []any{"chamfer", "Cap Chamfer", 2.0, "cap_chamfer"})
		}
		return log
	}
	require.Equal(t, ordinary(17, true, false, false), got[0], "17 teeth, fillet only")
	require.Equal(t, ordinary(12, true, true, true), got[1], "12 teeth, bore and chamfer")
	require.Equal(t, ordinary(17, false, false, false), got[2], "17 teeth, no fillet")

	// An embedded gear fillets the concave axial edges instead: one group, two edges per tooth.
	want := ordinary(50, true, true, false)
	want[9] = []any{"fillet", "Root Fillet", 100.0, 1.0, "fillet_radius"}
	require.Equal(t, want, got[3], "50 teeth, embedded")
}
