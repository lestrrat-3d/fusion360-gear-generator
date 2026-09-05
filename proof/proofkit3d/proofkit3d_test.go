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

func TestBodyReportFromFindsEachBodyByPointer(t *testing.T) {
	first, second := new(decad.Body), new(decad.Body)
	firstRecord := &decad.BodyReport{Body: first}
	secondRecord := &decad.BodyReport{Body: second}
	report := &decad.Report{Bodies: []*decad.BodyReport{firstRecord, secondRecord}}

	if got := bodyReportFrom(t, report, first); got != firstRecord {
		t.Fatalf("first body resolved to %p, want %p", got, firstRecord)
	}
	if got := bodyReportFrom(t, report, second); got != secondRecord {
		t.Fatalf("second body resolved to %p, want %p", got, secondRecord)
	}
}

func TestBodyReportRejectsBodyOutsideTheDocument(t *testing.T) {
	if os.Getenv("PROOFKIT3D_MISSING_BODY_HELPER") == "1" {
		BodyReport(t, decad.New(), new(decad.Body))
		t.Fatal("BodyReport returned for a body the document does not hold")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestBodyReportRejectsBodyOutsideTheDocument$")
	cmd.Env = append(os.Environ(), "PROOFKIT3D_MISSING_BODY_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("BodyReport accepted a body the document does not hold; output:\n%s", output)
	}
	if !strings.Contains(string(output), "verification report omitted body") {
		t.Fatalf("missing-body failure did not name the omission; output:\n%s", output)
	}
}

// The gate is asked for more than one body so that a lookup which only ever
// returned the report's first record would fail here.
func TestRequireSolidChecksEveryRequestedBody(t *testing.T) {
	doc, bodies := separatedBlocks(t, 2)
	RequireSolid(t, doc, bodies)

	for i, body := range bodies {
		got := BodyReport(t, doc, body)
		if got.Body != body {
			t.Fatalf("body %d resolved to the record for %p", i, got.Body)
		}
		if !got.Solid || !got.Watertight || !got.Manifold || got.SelfIntersecting {
			t.Fatalf("body %d is not a sound solid: %+v", i, got)
		}
		if got.Lumps != 1 || got.Voids != 0 {
			t.Fatalf("body %d topology: lumps=%d voids=%d", i, got.Lumps, got.Voids)
		}
	}
}

// A cache behind BodyReport would answer the second call from a report taken
// before the extrusion, and that report holds no record for the new body.
func TestBodyReportSeesABodyAddedSinceTheLastCall(t *testing.T) {
	doc, bodies := separatedBlocks(t, 1)
	if got := BodyReport(t, doc, bodies[0]); got.Body != bodies[0] {
		t.Fatalf("first body resolved to the record for %p", got.Body)
	}

	added := separatedBlocksIn(t, doc, 1, 1)
	got := BodyReport(t, doc, added[0])
	if got.Body != added[0] {
		t.Fatalf("the body added after the first call resolved to the record for %p", got.Body)
	}
	if !got.Solid {
		t.Fatalf("the body added after the first call is not a solid: %+v", got)
	}
}

func separatedBlocks(t *testing.T, count int) (*decad.Document, []*decad.Body) {
	t.Helper()
	doc := decad.New()
	return doc, separatedBlocksIn(t, doc, 0, count)
}

// separatedBlocksIn extrudes count 20x20x10 blocks into doc, each 100 mm along
// x from the last, so no pair interferes and the gate reads one lump per body.
// first offsets the row, which lets a second call add blocks clear of the ones
// already there.
func separatedBlocksIn(t *testing.T, doc *decad.Document, first, count int) []*decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	bodies := make([]*decad.Body, 0, count)
	for i := range count {
		s, err := world.CreateSketch(world.XY())
		if err != nil {
			t.Fatalf("create sketch: %v", err)
		}
		x := float64((first + i) * 100)
		s.CreateRectangle(x, 0, x+20, 20)
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
	return bodies
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
