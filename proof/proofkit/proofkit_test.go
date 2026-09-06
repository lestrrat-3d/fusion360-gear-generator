package proofkit

import (
	"os"
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

	output, err := runHelper(t, "^TestRunRejectsAllSkippedCases$", "PROOFKIT_ALL_SKIPPED_HELPER=1")
	if err == nil {
		t.Fatalf("all-skipped Run passed; output:\n%s", output)
	}
	if !strings.Contains(output, "proofkit: no non-skipped proof cases completed") {
		t.Fatalf("all-skipped failure did not explain the invariant; output:\n%s", output)
	}
}

func TestRunRejectsEmptyBuild(t *testing.T) {
	if os.Getenv("PROOFKIT_EMPTY_BUILD_HELPER") == "1" {
		Run(t, []Case{{Name: "empty", Params: map[string]float64{}}},
			func(testing.TB, *sketch.Sketch, map[string]float64) {})
		t.Fatal("Run returned after an empty build")
	}

	output, err := runHelper(t, "^TestRunRejectsEmptyBuild$", "PROOFKIT_EMPTY_BUILD_HELPER=1")
	if err == nil {
		t.Fatalf("empty build passed; output:\n%s", output)
	}
	if !strings.Contains(output, "proofkit: case \"empty\" created no authored geometry") {
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

func TestRunWithExpectedFailuresRejectsWrongExpectations(t *testing.T) {
	// Dispatch the child before any subprocess launch. Reversing this order
	// makes each child launch another copy without ever reaching the fixture.
	if name := os.Getenv("PROOFKIT_EXPECTED_FAILURE_HELPER"); name != "" {
		build := func(_ testing.TB, s *sketch.Sketch, p map[string]float64) {
			point := s.CreatePoint(0, 0)
			if p["free"] == 1 {
				return
			}
			s.Fix(point)
		}
		if name == "failed-positive" {
			build = func(_ testing.TB, _ *sketch.Sketch, _ map[string]float64) {}
		}
		if name == "empty-failures" {
			RunWithExpectedFailures(t, []Case{{Name: "positive", Params: map[string]float64{}}}, build, nil)
			return
		}
		expected := ExpectedFailure{Status: sketch.Underconstrained, DOF: 2, Reason: sketch.ErrNotFullyConstrained}
		if name == "wrong-dof" {
			expected.DOF = 1
		}
		if name == "wrong-reason" {
			expected.Reason = sketch.ErrUnsolvable
		}
		RunWithExpectedFailures(t,
			[]Case{{Name: "positive", Params: map[string]float64{}}},
			build,
			[]ExpectedFailureCase{{Case: Case{Name: "free", Params: map[string]float64{"free": 1}}, Expected: expected}},
		)
		return
	}

	for _, tc := range []struct{ name, want string }{
		{"wrong-dof", "DOF=1, got status="},
		{"wrong-reason", "expected exactly one reason matching"},
		{"empty-failures", "proofkit: no expected-failure cases"},
		{"failed-positive", "proofkit: case \"positive\" created no authored geometry"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			output, err := runHelper(t, "^TestRunWithExpectedFailuresRejectsWrongExpectations$",
				"PROOFKIT_EXPECTED_FAILURE_HELPER="+tc.name)
			if err == nil {
				t.Fatalf("invalid expected-failure fixture passed: %s\n%s", tc.name, output)
			}
			// A launch failure, timeout, or nesting rejection is not evidence that
			// RunWithExpectedFailures rejected the intended invalid expectation.
			if !strings.Contains(output, tc.want) {
				t.Fatalf("fixture %s did not report %q: %v\n%s", tc.name, tc.want, err, output)
			}
		})
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
