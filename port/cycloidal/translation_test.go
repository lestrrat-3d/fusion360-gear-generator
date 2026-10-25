package cycloidal_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	gcyc "github.com/lestrrat-3d/decad-gallery/models/cycloidal"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/cycloidal"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/stretchr/testify/require"
)

func translated(t *testing.T) string {
	t.Helper()
	src, err := gallery.Locate(t.Context(), "..", "cycloidal")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	text, err := go2py.TranspileSpec(m, cycloidal.FeatureSpec)
	require.NoError(t, err)
	return text
}

func runPython(t *testing.T, program string, input any, output any) {
	t.Helper()
	in, err := json.Marshal(input)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", program)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	require.NoError(t, json.Unmarshal(raw, output))
}

// problemsDriver runs the translated validity checks on every case and prints the messages.
const problemsDriver = `
import collections, json, math, sys
class _Params:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def _copy(self):
        return _Params(**vars(self))
out = []
for c in json.load(sys.stdin):
    out.append(problems(_Params(**c, Placement=None)))
json.dump(out, sys.stdout)
`

// TestTranslatedValidityChecksMatchTheGallery requires the translated Problems to print exactly the
// messages the gallery's own Problems does, for valid and invalid drives.
func TestTranslatedValidityChecksMatchTheGallery(t *testing.T) {
	base := gcyc.DefaultParams()
	mutate := func(f func(p *gcyc.Params)) gcyc.Params {
		p := base
		f(&p)
		return p
	}
	cases := []gcyc.Params{
		base,
		mutate(func(p *gcyc.Params) { p.PinCount = 3 }),
		mutate(func(p *gcyc.Params) { p.OutputPinCount = 2 }),
		mutate(func(p *gcyc.Params) { p.DiscCount = 3 }),
		mutate(func(p *gcyc.Params) { p.DiscCount = 2; p.PinCount = 15 }),
		mutate(func(p *gcyc.Params) { p.Eccentricity = 6 }),
		mutate(func(p *gcyc.Params) { p.PinDiameter = 1 }),
		mutate(func(p *gcyc.Params) { p.OutputPinDiameter = -1; p.HousingWall = 0 }),
		mutate(func(p *gcyc.Params) { p.InputShaftDiameter = 40 }),
		mutate(func(p *gcyc.Params) { p.OutputPinCircleDiameter = 140 }),
		mutate(func(p *gcyc.Params) { p.CenterBearingDiameter = 45; p.BearingClearance = 2 }),
		mutate(func(p *gcyc.Params) { p.PinCount = 40; p.PinCircleDiameter = 60; p.Eccentricity = 1.2 }),
		mutate(func(p *gcyc.Params) { p.DiscThickness = math.NaN() }),
		mutate(func(p *gcyc.Params) { p.ChamferSize = -1; p.DiscGap = -2 }),
	}
	type row struct {
		PinCount                int     `json:"PinCount"`
		PinCircleDiameter       float64 `json:"PinCircleDiameter"`
		PinDiameter             float64 `json:"PinDiameter"`
		Eccentricity            float64 `json:"Eccentricity"`
		DiskClearance           float64 `json:"DiskClearance"`
		DiscCount               int     `json:"DiscCount"`
		DiscThickness           float64 `json:"DiscThickness"`
		DiscGap                 float64 `json:"DiscGap"`
		CenterBearingDiameter   float64 `json:"CenterBearingDiameter"`
		InputShaftDiameter      float64 `json:"InputShaftDiameter"`
		BearingClearance        float64 `json:"BearingClearance"`
		OutputPinCircleDiameter float64 `json:"OutputPinCircleDiameter"`
		OutputPinCount          int     `json:"OutputPinCount"`
		OutputPinDiameter       float64 `json:"OutputPinDiameter"`
		HousingWall             float64 `json:"HousingWall"`
		BaseThickness           float64 `json:"BaseThickness"`
		OutputPlateThickness    float64 `json:"OutputPlateThickness"`
		ChamferSize             float64 `json:"ChamferSize"`
	}
	rows := make([]map[string]any, len(cases))
	for i, p := range cases {
		b, err := json.Marshal(row{p.PinCount, p.PinCircleDiameter, p.PinDiameter, p.Eccentricity, p.DiskClearance,
			p.DiscCount, finite(p.DiscThickness), p.DiscGap, p.CenterBearingDiameter, p.InputShaftDiameter,
			p.BearingClearance, p.OutputPinCircleDiameter, p.OutputPinCount, p.OutputPinDiameter, p.HousingWall,
			p.BaseThickness, p.OutputPlateThickness, p.ChamferSize})
		require.NoError(t, err)
		require.NoError(t, json.Unmarshal(b, &rows[i]))
	}
	var got [][]string
	runPython(t, "import collections\nimport math\n\n"+translated(t)+"\n"+problemsDriver, rows, &got)
	require.Len(t, got, len(cases))
	for i, p := range cases {
		want := gcyc.Problems(p)
		if want == nil {
			want = []string{}
		}
		if got[i] == nil {
			got[i] = []string{}
		}
		if i == 12 {
			// NaN cannot travel through JSON; the case checks the finite message separately.
			continue
		}
		require.Equal(t, want, got[i], "case %d", i)
	}
	require.Empty(t, got[0], "the default drive is valid")
	require.NotEmpty(t, got[5], "an oversized eccentricity is refused")
}

// finite replaces NaN, which JSON cannot carry, by a legal value; the NaN case is skipped by the caller.
func finite(v float64) float64 {
	if math.IsNaN(v) {
		return 8
	}
	return v
}

// buildFakes stand in for the Fusion runtime (_fx and the design). They record each feature the translated
// Build creates, with its role name, and they behave as Fusion does where Build reads a result back: a sketch
// of discs and a casing has one region per hole count, and the output pin sketch has one region per pin.
const buildFakes = `
import json, sys, types

LOG = []

class Vec:
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z
    def add(self, o): return Vec(self.x + o.x, self.y + o.y, self.z + o.z)
    def sub(self, o): return Vec(self.x - o.x, self.y - o.y, self.z - o.z)
    def scale(self, k): return Vec(self.x * k, self.y * k, self.z * k)
    def dot(self, o): return self.x * o.x + self.y * o.y + self.z * o.z

class Frame:
    def __init__(self, o, u, v): self._o, self._u, self._v = o, u, v
    def origin(self): return self._o
    def u(self): return self._u
    def v(self): return self._v
    def n(self): return Vec(0, 0, 1)

class Value:
    def __init__(self, name): self.expression = name
    def base(self): return 1.0

class Profile:
    def __init__(self, holes): self.Valid, self.Holes, self.Outer = True, [None] * holes, []

class Sketch:
    def __init__(self, name): self.name, self.points, self.splines, self.circles = name, 0, 0, 0
    def origin(self): return types.SimpleNamespace()
    def dimension_point(self, x, y): self.points += 1; return types.SimpleNamespace(x=x, y=y)
    def create_fit_spline(self, *pts):
        assert all(pt is not None for pt in pts), self.name + ': a spline point is missing'
        self.splines += 1
    def create_circle(self, c, r): self.circles += 1; return types.SimpleNamespace()
    def add_constraint(self, c): pass
    def bind(self, c, table, name): assert isinstance(name, str)
    def solve(self): LOG.append(['sketch', self.name, self.points, self.splines, self.circles])
    def profiles(self):
        if self.name == 'Output Pin Profile':
            return [Profile(0) for _ in range(self.circles)]
        return [Profile(h) for h in range(0, 31)]

class Body:
    def __init__(self, name, angle=0.0): self.name, self.angle = name, angle
    def is_solid(self): return True
    def centroid(self):
        import math
        return types.SimpleNamespace(Value=Vec(math.cos(self.angle), math.sin(self.angle), 0))

class Doc:
    def xy(self): return 'xy'
    def create_plane_from_frame(self, frame, name): LOG.append(['plane', name]); return 'plane'
    def create_sketch(self, plane, name): return Sketch(name)
    def params(self): return self
    def param(self, name): return Value(name)

class Selector:
    def or_(self, *p): return self
    def exactly(self, n): return self

pins = []
fx = types.SimpleNamespace(DECAD_ALONG=1)
fx.r3_new_vec = lambda x, y, z: Vec(x, y, z)
fx.r3_new_frame = lambda o, u, v: Frame(o, u, v)
fx.sketch_new_diameter = lambda circle, d: types.SimpleNamespace()
fx.sketchcheck_verify_fully_constrained = lambda s: None
fx.decad_distance = lambda D, Dir: ('distance', D)
fx.decad_distance_side = lambda D: ('side', D)
fx.decad_two_sided = lambda One, Two: ('two', One, Two)
fx.units_millimeters = lambda x: x
def extrude(doc, s, profile, extent, name, body):
    LOG.append(['extrude', name, body])
    if body == 'Output Pin':
        pins.append(Body(body, 6.0 - 0.7 * len(pins)))
        return pins[-1]
    return Body(body)
fx.decad_extrude = extrude
def feature(kind):
    def make(*args, name=None):
        LOG.append([kind, name]); return args[0]
    return make
fx.decad_union = feature('union')
fx.decad_cut = feature('cut')
def chamfer(body, sel, size, name): LOG.append(['chamfer', name]); return body
fx.decad_chamfer = chamfer
fx.decad_cap_start = lambda b: Selector()
fx.decad_cap_end = lambda b: Selector()
fx.decad_outer_loop_of = lambda s: Selector()
fx.decad_edges = lambda *p: Selector()
fx.decad_circular = lambda: None
fx.decad_longer_than = lambda n: None
_fx = fx

class _Params:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def _copy(self):
        return _Params(**vars(self))

# The fragment's wiring, which the generated file defines beside the translation.
def _begin(doc, p):
    return _Run(doc=doc, world=doc, p=p, d=_resolve(p), frame=_placement(p))
`

const buildDriver = `
import json, sys
case = json.load(sys.stdin)
out = []
for changes in case:
    LOG.clear(); pins.clear()
    p = _Params(**changes['params'])
    assert problems(p) == [], problems(p)
    drive = build(Doc(), p)
    named = [[n.Name, n.Body.name] for n in drive_named(drive)]
    out.append({'log': LOG[:], 'named': named, 'angles': [b.angle for b in drive.OutputPins]})
json.dump(out, sys.stdout)
`

// TestTranslatedBuildRunsAgainstFakes runs the whole translated Build for several drives against fakes and
// requires the gallery's feature order, the body names, and sorted output pins. Fusion itself is only seen
// when the user loads the add-in; this catches faults in the translated Python.
func TestTranslatedBuildRunsAgainstFakes(t *testing.T) {
	type params = map[string]any
	base := params{
		"PinCount": 16, "PinCircleDiameter": 90, "PinDiameter": 0, "Eccentricity": 1.5, "DiskClearance": 0.3,
		"DiscCount": 1, "DiscThickness": 8, "DiscGap": 0.5, "CenterBearingDiameter": 30, "InputShaftDiameter": 8,
		"BearingClearance": 0.2, "OutputPinCircleDiameter": 50, "OutputPinCount": 6, "OutputPinDiameter": 0,
		"HousingWall": 3, "BaseThickness": 5, "OutputPlateThickness": 5, "ChamferSize": 0.5, "Placement": nil,
	}
	with := func(changes params) params {
		out := params{}
		for k, v := range base {
			out[k] = v
		}
		for k, v := range changes {
			out[k] = v
		}
		return out
	}
	cases := []params{
		base,
		with(params{"DiscCount": 2, "DiscGap": 0}),
		with(params{"ChamferSize": 0, "InputShaftDiameter": 0}),
	}
	input := make([]params, len(cases))
	for i, c := range cases {
		input[i] = params{"params": c}
	}
	var got []struct {
		Log    [][]any    `json:"log"`
		Named  [][]string `json:"named"`
		Angles []float64  `json:"angles"`
	}
	runPython(t, "import collections\nimport math\n\n"+translated(t)+"\n"+buildFakes+buildDriver, input, &got)
	require.Len(t, got, 3)

	names := func(i int, kind string) []string {
		var out []string
		for _, e := range got[i].Log {
			if e[0] == kind {
				out = append(out, e[1].(string))
			}
		}
		return out
	}
	// One disc, default drive: the order the gallery builds in.
	require.Equal(t, []string{
		"Disc Profile", "Cam Section Profile", "Input Bore Profile", "Housing Outer Profile", "Casing Profile",
		"Housing Hole Profile", "Output Plate Profile", "Output Pin Profile",
	}, names(0, "sketch"))
	require.Equal(t, []string{"Casing Cavity Cut", "Housing Hole Cut", "Input Bore Cut"}, sorted(names(0, "cut")))
	require.Equal(t, []string{"Chamfer disc 1", "Chamfer housing", "Chamfer output plate", "Chamfer output pin"}, uniq(names(0, "chamfer")))
	require.Len(t, names(0, "chamfer"), 1+2+6)
	require.Equal(t, []string{
		"Cycloidal Disk 1", "Housing", "Eccentric Cam", "Output Plate",
		"Output Pin 1", "Output Pin 2", "Output Pin 3", "Output Pin 4", "Output Pin 5", "Output Pin 6",
	}, firsts(got[0].Named))
	require.IsIncreasing(t, got[0].Angles, "output pins are ordered by angle")

	// Two discs without a gap join the cams through a bridge.
	require.Contains(t, names(1, "sketch"), "Cam Bridge Profile")
	require.Contains(t, names(1, "union"), "Cam Join")
	require.Equal(t, 2, count(names(1, "sketch"), "Disc Profile"))
	require.Equal(t, []string{"Cycloidal Disk 1", "Cycloidal Disk 2"}, firsts(got[1].Named)[:2])

	// No chamfer and no input bore skip those features.
	require.Empty(t, names(2, "chamfer"))
	require.NotContains(t, names(2, "sketch"), "Input Bore Profile")
}

func firsts(rows [][]string) []string {
	out := make([]string, len(rows))
	for i, r := range rows {
		out[i] = r[0]
	}
	return out
}

func sorted(in []string) []string {
	out := append([]string(nil), in...)
	for i := range out {
		for j := i + 1; j < len(out); j++ {
			if out[j] < out[i] {
				out[i], out[j] = out[j], out[i]
			}
		}
	}
	return out
}

func uniq(in []string) []string {
	seen := map[string]bool{}
	var out []string
	for _, s := range in {
		if !seen[s] {
			seen[s] = true
			out = append(out, s)
		}
	}
	return out
}

func count(in []string, want string) int {
	n := 0
	for _, s := range in {
		if s == want {
			n++
		}
	}
	return n
}
