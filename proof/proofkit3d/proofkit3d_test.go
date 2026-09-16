package proofkit3d

import (
	"os"
	"os/exec"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
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

// The lookup behind [BodyReport] is decad's own Report.ForBody, reached through
// decadtest.FindBodyReport. What is checked here is that a report of several
// bodies resolves each one to its own record, which is what [BodyReport]
// promises its callers.
func TestBodyReportFindsEachBodyByPointer(t *testing.T) {
	doc, bodies := separatedBlocks(t, 2)
	report := decadtest.Verify(t, doc)

	for i, body := range bodies {
		if got := decadtest.FindBodyReport(t, report, body); got.Body != body {
			t.Fatalf("body %d resolved to the record for %p", i, got.Body)
		}
	}
}

func TestBodyReportRejectsBodyOutsideTheDocument(t *testing.T) {
	if os.Getenv("PROOFKIT3D_MISSING_BODY_HELPER") == "1" {
		// A real body, built in a document of its own, so the lookup is asked
		// about a body that exists and is simply not this document's.
		_, foreign := separatedBlocks(t, 1)
		BodyReport(t, decad.New(), foreign[0])
		t.Fatal("BodyReport returned for a body the document does not hold")
	}

	cmd := exec.Command(os.Args[0], "-test.run=^TestBodyReportRejectsBodyOutsideTheDocument$")
	cmd.Env = append(os.Environ(), "PROOFKIT3D_MISSING_BODY_HELPER=1")
	output, err := cmd.CombinedOutput()
	if err == nil {
		t.Fatalf("BodyReport accepted a body the document does not hold; output:\n%s", output)
	}
	if !strings.Contains(string(output), "the report holds no record of this body") {
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
		if got.Validity.Outcome != decad.ValidityValid {
			t.Fatalf("body %d validity is %s, want valid: %+v", i, got.Validity.Outcome, got)
		}
		if got.Topology.Lumps != 1 || got.Topology.Voids != 0 {
			t.Fatalf("body %d topology: lumps=%d voids=%d", i, got.Topology.Lumps, got.Topology.Voids)
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
	if got.Validity.Outcome != decad.ValidityValid {
		t.Fatalf("the body added after the first call is %s, want valid: %+v", got.Validity.Outcome, got)
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
	bodies := make([]*decad.Body, 0, count)
	for i := range count {
		x := float64((first + i) * 100)
		bodies = append(bodies, decadtest.NewBlock(t, doc, x, 0, x+20, 20, units.Millimeters(10)))
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
