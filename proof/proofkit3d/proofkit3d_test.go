package proofkit3d

import (
	"os"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
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

func TestBodyReportFromReturnsMatchingPointer(t *testing.T) {
	body1, body2 := new(decad.Body), new(decad.Body)
	report1 := &decad.BodyReport{Body: body1}
	report2 := &decad.BodyReport{Body: body2}
	report := &decad.Report{Bodies: []*decad.BodyReport{report1, report2}}

	if got := bodyReportFrom(t, report, body1); got != report1 {
		t.Fatalf("body 1 report pointer = %p, want %p", got, report1)
	}
	if got := bodyReportFrom(t, report, body2); got != report2 {
		t.Fatalf("body 2 report pointer = %p, want %p", got, report2)
	}
}

func TestBodyReportRejectsMissingBody(t *testing.T) {
	if os.Getenv("PROOFKIT3D_MISSING_BODY_HELPER") == "1" {
		BodyReport(t, decad.New(), new(decad.Body))
		t.Fatal("BodyReport returned after a missing body")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestBodyReportRejectsMissingBody$")
	cmd.Env = append(os.Environ(), "PROOFKIT3D_MISSING_BODY_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("BodyReport accepted a missing body; output:\n%s", output)
	}
	if !strings.Contains(string(output), "verification report omitted body") {
		t.Fatalf("missing-body failure did not explain the omission; output:\n%s", output)
	}
}

func TestRequireSolidAndBodyReportUseRealSolid(t *testing.T) {
	doc, bodies := rectangularSolids(t, 1)
	RequireSolid(t, doc, bodies)
	got := BodyReport(t, doc, bodies[0])
	if got.Body != bodies[0] || !got.Solid || got.Lumps != 1 || got.Voids != 0 {
		t.Fatalf("unexpected solid report: %+v", got)
	}
}

func TestRequireSolidLeavesReportsForSeparatedBodies(t *testing.T) {
	doc, bodies := rectangularSolids(t, 2)
	RequireSolid(t, doc, bodies)
	for _, body := range bodies {
		got := BodyReport(t, doc, body)
		if got.Body != body || !got.Solid {
			t.Fatalf("body report unavailable after gate: %+v", got)
		}
	}
}

func rectangularSolids(t *testing.T, count int) (*decad.Document, []*decad.Body) {
	t.Helper()
	world := sketch.NewWorld()
	doc := decad.New()
	bodies := make([]*decad.Body, 0, count)
	for i := range count {
		s, err := world.CreateSketch(world.XY())
		if err != nil {
			t.Fatalf("create sketch: %v", err)
		}
		offset := float64(i * 100)
		s.CreateRectangle(offset, 0, offset+20, 20)
		result, err := s.Solve(t.Context())
		if err != nil {
			t.Fatalf("solve sketch: %v", err)
		}
		if !result.Converged {
			t.Fatalf("solver did not converge: residual %.3e, DOF %d", result.Residual, result.DOF)
		}
		profiles := s.Profiles()
		if len(profiles) != 1 {
			t.Fatalf("rectangle produced %d profiles, want 1", len(profiles))
		}
		body, err := doc.Extrude(s, profiles[0], decad.Distance{
			D: units.Millimeters(10), Dir: decad.Along,
		})
		if err != nil {
			t.Fatalf("extrude rectangle: %v", err)
		}
		bodies = append(bodies, body)
	}
	return doc, bodies
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
