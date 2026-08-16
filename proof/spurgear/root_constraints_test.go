package spurgear_test

import (
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
)

func TestRootEndSignedConstraintRecipeDoesNotDrift(t *testing.T) {
	for _, tc := range []struct {
		name string
		file string
		want []string
		ban  []string
	}{
		{
			name: "generator",
			file: "lib/geargen/spurgear.py",
			want: []string{
				"addDistanceDimension(\n            origin, rootEndPoint,\n            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation",
				"addDistanceDimension(\n            origin, rootEndPoint,\n            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation",
				"horizontalDimension.parameter.value = dx",
				"verticalDimension.parameter.value = dy",
			},
			ban: []string{
				"addCoincident(line.startSketchPoint, self.rootCircle)",
				"addCoincident(origin, line)",
			},
		},
		{
			name: "sketch prototype",
			file: "spec/spurgear/sketch/main.go",
			want: []string{
				"sketch.NewHorizontalDistance(origin, re, rx)",
				"sketch.NewVerticalDistance(origin, re, ry)",
			},
			ban: []string{
				"sketch.NewPointOnCircle(re, rootCircle)",
				"sketch.NewPointOnLine(origin, line)",
			},
		},
		{
			name: "sketch README",
			file: "spec/spurgear/sketch/README.md",
			want: []string{
				"flank-to-root lines: root endpoint pinned by signed Δx and Δy from the local origin",
				"NewHorizontalDistance(origin, rootEnd, dx)",
				"NewVerticalDistance(origin, rootEnd, dy)",
			},
			ban: []string{
				"flank-to-root lines: root-end-on-circle + origin-on-line",
				"NewPointOnCircle` + `NewPointOnLine",
				"origin-on-line\" constraint",
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			data, err := os.ReadFile(repoPath(t, tc.file))
			if err != nil {
				t.Fatalf("read %s: %v", tc.file, err)
			}
			text := string(data)
			for _, want := range tc.want {
				if !strings.Contains(text, want) {
					t.Fatalf("%s no longer contains required root-end signed-dimension text %q", tc.file, want)
				}
			}
			for _, ban := range tc.ban {
				if strings.Contains(text, ban) {
					t.Fatalf("%s contains rejected root-end coincidence recipe %q", tc.file, ban)
				}
			}
		})
	}
}

func repoPath(t *testing.T, rel string) string {
	t.Helper()
	_, file, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("runtime.Caller failed")
	}
	return filepath.Join(filepath.Dir(file), "..", "..", rel)
}
