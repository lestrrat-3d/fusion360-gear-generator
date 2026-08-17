package proofkit3d

import (
	"os"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
)

const skipCase = "skip"

func TestRunWithGateAllowsCompletedCase(t *testing.T) {
	var built, gated, asserted int
	RunWithGate(t, []Case{
		{Name: "complete", Params: map[string]float64{}},
	}, countingBuild(&built), countingGate(&gated), countingAssert(&asserted))

	if built != 1 || gated != 1 || asserted != 1 {
		t.Fatalf("completed case counts: build=%d gate=%d assert=%d", built, gated, asserted)
	}
}

func TestRunWithGateAllowsMixedCompletedAndSkippedCases(t *testing.T) {
	var built, gated, asserted int
	RunWithGate(t, []Case{
		{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		{Name: "complete", Params: map[string]float64{}},
	}, countingBuild(&built), countingGate(&gated), countingAssert(&asserted))

	if built != 2 || gated != 1 || asserted != 1 {
		t.Fatalf("mixed case counts: build=%d gate=%d assert=%d", built, gated, asserted)
	}
}

func TestRunWithGateRejectsAllSkippedCases(t *testing.T) {
	if os.Getenv("PROOFKIT3D_ALL_SKIPPED_HELPER") == "1" {
		RunWithGate(t, []Case{
			{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		}, countingBuild(new(int)), countingGate(new(int)), countingAssert(new(int)))
		t.Fatal("RunWithGate returned after every case skipped")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestRunWithGateRejectsAllSkippedCases$")
	cmd.Env = append(os.Environ(), "PROOFKIT3D_ALL_SKIPPED_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("all-skipped RunWithGate passed; output:\n%s", output)
	}
	if !strings.Contains(string(output), "proofkit3d: no non-skipped proof cases completed") {
		t.Fatalf("all-skipped failure did not explain the invariant; output:\n%s", output)
	}
}

func TestRequireSoundRejectsNilBody(t *testing.T) {
	if os.Getenv("PROOFKIT3D_NIL_BODY_HELPER") == "1" {
		RequireSound(t, decad.New(), []*decad.Body{nil})
		t.Fatal("RequireSound returned after receiving a nil body")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestRequireSoundRejectsNilBody$")
	cmd.Env = append(os.Environ(), "PROOFKIT3D_NIL_BODY_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("RequireSound accepted a nil body; output:\n%s", output)
	}
	if !strings.Contains(string(output), "proofkit3d: build returned nil body at index 0") {
		t.Fatalf("nil-body failure did not explain the invariant; output:\n%s", output)
	}
}

func countingBuild(count *int) Build {
	return func(t *testing.T, _ *decad.Document, params map[string]float64) []*decad.Body {
		(*count)++
		if params[skipCase] == 1 {
			Unmodelled(t, "fixture case is intentionally unsupported")
		}
		return nil
	}
}

func countingGate(count *int) Gate {
	return func(t *testing.T, _ *decad.Document, _ []*decad.Body) {
		(*count)++
	}
}

func countingAssert(count *int) Assert {
	return func(t *testing.T, _ *decad.Document, _ []*decad.Body, _ map[string]float64) {
		(*count)++
	}
}
