package spur_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"testing"

	"github.com/lestrrat-3d/decad-gallery/models/spurgear"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gotopy"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// recorder stands in for the Fusion runtime and records the calls the translated tooth
// section makes, in order.
const recorder = `
import collections, json, math, sys

class Recorder:
    def __init__(self):
        self.points, self.lines, self.arcs, self.splines = [], [], [], []
    def origin(self):
        return 'origin'
    def dimension_point(self, x, y):
        self.points.append([x, y, False])
        return len(self.points) - 1
    def dimension_arc_end(self, x, y):
        self.points.append([x, y, True])
        return len(self.points) - 1
    def create_line(self, a, b):
        self.lines.append([a, b])
    def create_arc(self, c, a, b):
        self.arcs.append([c, a, b])
    def create_fit_spline(self, *pts):
        self.splines.append(list(pts))

Params = collections.namedtuple('Params', ['Teeth', 'Samples'])
out = []
for c in json.load(sys.stdin):
    pitch = c['module'] * c['teeth'] / 2
    d = _Dimensions(pitch, pitch * math.cos(c['pressure_angle']), pitch - 1.25 * c['module'], pitch + c['module'], 0.0)
    r = Recorder()
    _draw_tooth_section(r, Params(c['teeth'], c['samples']), d, c['angle'])
    out.append(r.__dict__)
json.dump(out, sys.stdout)
`

type toothCase struct {
	Module        float64 `json:"module"`
	Teeth         int     `json:"teeth"`
	PressureAngle float64 `json:"pressure_angle"`
	Samples       int     `json:"samples"`
	Angle         float64 `json:"angle"`
}

type recorded struct {
	Points  [][3]any `json:"points"`
	Lines   [][2]int `json:"lines"`
	Arcs    [][3]any `json:"arcs"`
	Splines [][]int  `json:"splines"`
}

// TestTranslatedToothSectionMatchesGallery runs the translated tooth section against a
// recording runtime and requires its sequence and its points to agree with the gallery's
// own verified tooth section.
func TestTranslatedToothSectionMatchesGallery(t *testing.T) {
	src, err := gallery.Locate(t.Context(), "..", "spurgear")
	require.NoError(t, err)
	m, err := gallery.Load(src.Dir)
	require.NoError(t, err)
	text, err := gotopy.TranspileSpec(m, spur.FeatureSpec)
	require.NoError(t, err)

	cases := []toothCase{
		{1, 17, 20 * math.Pi / 180, 15, 0},
		{2, 24, 25 * math.Pi / 180, 9, 0},
		{1, 40, 20 * math.Pi / 180, 15, 14.5 * math.Pi / 180},
		{1.5, 17, 20 * math.Pi / 180, 5, -14.5 * math.Pi / 180},
		{1, 17, 20 * math.Pi / 180, 15, math.Pi / 2},
	}
	in, err := json.Marshal(cases)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", "import collections\nimport math\n\n"+text+"\n"+recorder)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got []recorded
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(cases))

	for i, c := range cases {
		n := c.Samples
		g := got[i]
		// Gallery order: right flank points, left flank points, left root, right root.
		right, left, leftRoot, rightRoot := indexes(0, n), indexes(n, n), 2*n, 2*n+1
		require.Len(t, g.Points, 2*n+2, "case %d points", i)
		require.Equal(t, [][2]int{{rightRoot, right[0]}, {left[0], leftRoot}}, g.Lines, "case %d lines", i)
		require.Equal(t, [][]int{right, left}, g.Splines, "case %d splines", i)
		require.Len(t, g.Arcs, 2, "case %d arcs", i)
		require.Equal(t, []any{"origin", float64(right[n-1]), float64(left[n-1])}, g.Arcs[0][:], "case %d tip arc", i)
		require.Equal(t, []any{"origin", float64(rightRoot), float64(leftRoot)}, g.Arcs[1][:], "case %d root arc", i)
		for k, p := range g.Points {
			wantArcEnd := k == left[n-1] || k == leftRoot
			require.Equal(t, wantArcEnd, p[2], "case %d point %d arc end", i, k)
		}

		p := spurgear.DefaultParams()
		p.Module, p.Teeth, p.PressureAngle, p.Samples = c.Module, c.Teeth, c.PressureAngle, c.Samples
		world := sketch.NewWorld()
		_, profile, err := spurgear.BuildToothSection(t.Context(), world, world.XY(), p, c.Angle)
		require.NoError(t, err, "case %d", i)
		for k, pt := range g.Points {
			x, y := pt[0].(float64), pt[1].(float64)
			require.True(t, onProfile(profile, x, y), "case %d point %d (%g, %g) is not on the gallery profile", i, k, x, y)
		}
	}
}

// onProfile reports whether (x, y) lies on the profile's outer boundary. A spline's polyline
// only approximates the curve, so the tolerance is loose; a wrong point is millimetres away.
func onProfile(profile *sketch.Profile, x, y float64) bool {
	for _, edge := range profile.Outer {
		for k := 1; k < len(edge.Polyline); k++ {
			if segmentDistance(edge.Polyline[k-1], edge.Polyline[k], x, y) < 5e-3 {
				return true
			}
		}
	}
	return false
}

func segmentDistance(a, b [2]float64, x, y float64) float64 {
	dx, dy := b[0]-a[0], b[1]-a[1]
	t := 0.0
	if l2 := dx*dx + dy*dy; l2 > 0 {
		t = math.Max(0, math.Min(1, ((x-a[0])*dx+(y-a[1])*dy)/l2))
	}
	return math.Hypot(a[0]+t*dx-x, a[1]+t*dy-y)
}

func indexes(from, n int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = from + i
	}
	return out
}
