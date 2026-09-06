// Package proofkit3d is the shared harness for solid-geometry proofs.
package proofkit3d

import (
	"maps"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
)

// Case is one parameter set to prove. Name becomes the subtest name.
type Case struct {
	Name   string
	Params map[string]float64
}

// Build creates the bodies for one case in doc.
type Build func(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body

// Assert checks proof-specific measurements after the common solid gate passes.
type Assert func(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64)

// Gate applies the common verification gate before the proof-specific assertion.
type Gate func(t *testing.T, doc *decad.Document, bodies []*decad.Body)

// ParallelGroup is the subtest the parallel entry points nest their cases
// under, for the reason [proofkit.ParallelGroup] gives: Go releases a parallel
// subtest only after its parent's function has returned, so cases started
// directly by a runner would still be waiting when that runner reached its
// completion check. The group is a synchronous subtest whose t.Run does not
// return until every parallel child has finished.
//
// It adds one level to every case's name: `TestX/cases/name`. That is a
// deliberate consequence of opting in, which is why the serial entry points do
// not add it.
const ParallelGroup = proofkit.ParallelGroup

// Run proves every case with a fresh document, one case at a time.
func Run(t *testing.T, cases []Case, build Build, assert Assert) {
	RunWithGate(t, cases, build, RequireSound, assert)
}

// RunSolid is Run with the bounded-solid gate. It allows only the documented
// area or centroid tolerance diagnostic from a faceted boolean result.
func RunSolid(t *testing.T, cases []Case, build Build, assert Assert) {
	RunWithGate(t, cases, build, RequireSolid, assert)
}

// RunSolidParallel is [RunSolid] with the cases running concurrently.
//
// It proves the same thing in the same way: same build, same gate, same
// assertion, same completion invariant, same failure and skip behaviour. Only
// the scheduling differs, and every case's name gains the [ParallelGroup]
// level.
//
// Each case still gets its own [decad.Document], and is additionally handed its
// own copy of Params so a build that writes to the map it is given cannot reach
// a sibling. Within one case that copy is shared by build, gate and assert
// exactly as the serial path shares the original.
//
// Nothing else is per-case, and that is the whole reason opting in is decided
// one step at a time. proof/bevelgear's stepCircularPattern measures its seed
// solid, stores the readings in package-level vars, and retires the seed; its
// assertion reads them back. Two cases running at once overwrite each other's
// readings and the proof reports a wrong verdict rather than failing loudly, so
// that step keeps [RunSolid]. Audit a proof for state held between build and
// assert before moving it to this runner.
//
// -parallel bounds how many cases run at once. It defaults to GOMAXPROCS.
func RunSolidParallel(t *testing.T, cases []Case, build Build, assert Assert) {
	t.Helper()
	runCases(t, cases, build, RequireSolid, assert, true)
}

// RunWithGate proves every case with the supplied verification gate, one case
// at a time. At least one case must complete; a proof table where every case is
// unmodelled proves no solid.
func RunWithGate(t *testing.T, cases []Case, build Build, gate Gate, assert Assert) {
	t.Helper()
	runCases(t, cases, build, gate, assert, false)
}

// RunWithGateParallel is [RunWithGate] with the cases running concurrently, on
// the terms [RunSolidParallel] describes.
func RunWithGateParallel(t *testing.T, cases []Case, build Build, gate Gate, assert Assert) {
	t.Helper()
	runCases(t, cases, build, gate, assert, true)
}

// runCases owns the rules every entry point shares, so the parallel paths
// cannot drift from the serial ones. Only the scheduling is decided here.
func runCases(t *testing.T, cases []Case, build Build, gate Gate, assert Assert, parallel bool) {
	t.Helper()
	if len(cases) == 0 {
		t.Fatal("proofkit3d: no cases — a proof with nothing to prove passes vacuously")
	}
	if build == nil {
		t.Fatal("proofkit3d: nil build function")
	}
	if gate == nil {
		t.Fatal("proofkit3d: nil verification gate")
	}
	if assert == nil {
		t.Fatal("proofkit3d: nil assertion function")
	}
	// Atomic because the parallel path increments it from several cases at once.
	// The serial path reads the same value it always did.
	var completed atomic.Int64
	prove := func(t *testing.T, c Case) {
		doc := decad.New()
		bodies := build(t, doc, c.Params)
		gate(t, doc, bodies)
		assert(t, doc, bodies, c.Params)
		completed.Add(1)
	}

	if !parallel {
		for _, c := range cases {
			t.Run(c.Name, func(t *testing.T) { prove(t, c) })
		}
	} else {
		t.Run(ParallelGroup, func(t *testing.T) {
			for _, c := range cases {
				c := Case{Name: c.Name, Params: maps.Clone(c.Params)}
				t.Run(c.Name, func(t *testing.T) {
					t.Parallel()
					prove(t, c)
				})
			}
		})
	}

	if completed.Load() == 0 && !t.Failed() {
		t.Fatal("proofkit3d: no non-skipped proof cases completed — every proof must prove at least one solid")
	}
}

// Unmodelled skips a case that the current evaluator cannot represent.
func Unmodelled(t *testing.T, format string, args ...any) {
	t.Helper()
	t.Skipf("not modelled: "+format, args...)
}

// RequireSound gates a proof on decad's complete verification verdict.
func RequireSound(t *testing.T, doc *decad.Document, bodies []*decad.Body) {
	t.Helper()
	if len(bodies) == 0 {
		t.Fatal("proofkit3d: build returned no bodies")
	}
	for i, body := range bodies {
		if body == nil {
			t.Fatalf("proofkit3d: build returned nil body at index %d", i)
		}
	}
	report, err := doc.Verify(t.Context())
	if err != nil {
		t.Fatalf("verify failed: %v", err)
	}
	if report.Trustworthy() {
		return
	}
	for _, diagnostic := range report.Diagnostics {
		t.Logf("diagnostic: %+v", diagnostic)
	}
	t.Fatalf("verification was not sound: %s", report.Status)
}

// RequireSolid verifies topology and bounded readings for a solid. A boolean
// result can carry a bounded area or centroid reading too coarse for the
// default tolerance while still proving the solid, volume and topology. Those
// are the only diagnostics this gate accepts; a new diagnostic fails the proof.
func RequireSolid(t *testing.T, doc *decad.Document, bodies []*decad.Body) {
	t.Helper()
	if len(bodies) == 0 {
		t.Fatal("proofkit3d: build returned no bodies")
	}
	report, err := doc.Verify(t.Context())
	if err != nil {
		t.Fatalf("verify failed: %v", err)
	}
	for _, diagnostic := range report.Diagnostics {
		if diagnostic.Code != decad.DiagMeasurementBeyondTolerance ||
			(diagnostic.Reading != decad.ReadingArea && diagnostic.Reading != decad.ReadingCentroid) {
			t.Fatalf("unexpected verification diagnostic: %+v", diagnostic)
		}
	}
	// The report above already carries a record per live body, and nothing here
	// touches the document between producing it and reading it, so every body is
	// looked up in that one report rather than verified again per body.
	for _, body := range bodies {
		bodyReport := bodyReportFrom(t, report, body)
		if !bodyReport.Solid || !bodyReport.Watertight || !bodyReport.Manifold || bodyReport.SelfIntersecting {
			t.Fatalf("body is not a sound solid: %+v", bodyReport)
		}
		if bodyReport.Lumps != 1 || bodyReport.Voids != 0 {
			t.Fatalf("body topology is not one solid lump: lumps=%d voids=%d", bodyReport.Lumps, bodyReport.Voids)
		}
	}
}

// BodyReport returns the verification record for body, or fails the test when
// the body was not included in the document report. It verifies the document on
// every call, so a caller may build or consume bodies between two calls and
// still read the model as it stands.
func BodyReport(t *testing.T, doc *decad.Document, body *decad.Body) *decad.BodyReport {
	t.Helper()
	report, err := doc.Verify(t.Context())
	if err != nil {
		t.Fatalf("verify failed: %v", err)
	}
	return bodyReportFrom(t, report, body)
}

// bodyReportFrom finds body's record in a report already produced for its
// document. The caller owns the report's freshness: a report describes the
// document as it stood when Verify ran, so only a caller that has not touched
// the document since may read one.
func bodyReportFrom(t *testing.T, report *decad.Report, body *decad.Body) *decad.BodyReport {
	t.Helper()
	for _, bodyReport := range report.Bodies {
		if bodyReport.Body == body {
			return bodyReport
		}
	}
	t.Fatalf("verification report omitted body %p", body)
	return nil
}
