package proofkit

import (
	"os"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/sketch"
)

const skipCase = "skip"

func TestRunAllowsCompletedCase(t *testing.T) {
	var built int
	Run(t, []Case{
		{Name: "complete", Params: map[string]float64{}},
	}, countingBuild(&built))

	if built != 1 {
		t.Fatalf("completed case count: build=%d", built)
	}
}

func TestRunAllowsMixedCompletedAndSkippedCases(t *testing.T) {
	var built int
	Run(t, []Case{
		{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		{Name: "complete", Params: map[string]float64{}},
	}, countingBuild(&built))

	if built != 2 {
		t.Fatalf("mixed case count: build=%d", built)
	}
}

func TestRunRejectsAllSkippedCases(t *testing.T) {
	if os.Getenv("PROOFKIT_ALL_SKIPPED_HELPER") == "1" {
		Run(t, []Case{
			{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		}, countingBuild(new(int)))
		t.Fatal("Run returned after every case skipped")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestRunRejectsAllSkippedCases$")
	cmd.Env = append(os.Environ(), "PROOFKIT_ALL_SKIPPED_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("all-skipped Run passed; output:\n%s", output)
	}
	if !strings.Contains(string(output), "proofkit: no non-skipped proof cases completed") {
		t.Fatalf("all-skipped failure did not explain the invariant; output:\n%s", output)
	}
}

func countingBuild(count *int) Build {
	return func(t testing.TB, s *sketch.Sketch, params map[string]float64) {
		(*count)++
		if params[skipCase] == 1 {
			Unmodelled(t, "fixture case is intentionally unsupported")
		}
		anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
		anchor.SetName("fixture anchor")
	}
}
