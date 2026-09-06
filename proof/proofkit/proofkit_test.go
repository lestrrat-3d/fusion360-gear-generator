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

func TestRunRejectsEmptyBuild(t *testing.T) {
	if os.Getenv("PROOFKIT_EMPTY_BUILD_HELPER") == "1" {
		Run(t, []Case{{Name: "empty", Params: map[string]float64{}}},
			func(testing.TB, *sketch.Sketch, map[string]float64) {})
		t.Fatal("Run returned after an empty build")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestRunRejectsEmptyBuild$")
	cmd.Env = append(os.Environ(), "PROOFKIT_EMPTY_BUILD_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("empty build passed; output:\n%s", output)
	}
	if !strings.Contains(string(output), "proofkit: case \"empty\" created no authored geometry") {
		t.Fatalf("empty-build failure did not explain the invariant; output:\n%s", output)
	}
}

func TestRunWithExpectedFailures(t *testing.T) {
	RunWithExpectedFailures(t,
		[]Case{{Name: "positive", Params: map[string]float64{}}},
		func(_ testing.TB, s *sketch.Sketch, p map[string]float64) {
			point := s.CreatePoint(0, 0)
			if p["free"] == 1 {
				return
			}
			s.Fix(point)
		},
		[]ExpectedFailureCase{{
			Case:     Case{Name: "free_point", Params: map[string]float64{"free": 1}},
			Expected: ExpectedFailure{Status: sketch.Underconstrained, DOF: 2, Reason: sketch.ErrNotFullyConstrained},
		}},
	)
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
