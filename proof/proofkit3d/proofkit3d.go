// Package proofkit3d is the shared harness for solid-geometry proofs.
package proofkit3d

import (
	"testing"

	"github.com/lestrrat-3d/decad"
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

// Run proves every case with a fresh document.
func Run(t *testing.T, cases []Case, build Build, assert Assert) {
	RunWithGate(t, cases, build, RequireSound, assert)
}

// RunSolid is Run with the bounded-solid gate. It allows only the documented
// area or centroid tolerance diagnostic from a faceted boolean result.
func RunSolid(t *testing.T, cases []Case, build Build, assert Assert) {
	RunWithGate(t, cases, build, RequireSolid, assert)
}

// RunWithGate proves every case with the supplied verification gate. At least
// one case must complete; a proof table where every case is unmodelled proves
// no solid.
func RunWithGate(t *testing.T, cases []Case, build Build, gate Gate, assert Assert) {
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
	completed := 0
	for _, c := range cases {
		c := c
		t.Run(c.Name, func(t *testing.T) {
			doc := decad.New()
			bodies := build(t, doc, c.Params)
			gate(t, doc, bodies)
			assert(t, doc, bodies, c.Params)
			completed++
		})
	}
	if completed == 0 && !t.Failed() {
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
	for _, body := range bodies {
		bodyReport := BodyReport(t, doc, body)
		if !bodyReport.Solid || !bodyReport.Watertight || !bodyReport.Manifold || bodyReport.SelfIntersecting {
			t.Fatalf("body is not a sound solid: %+v", bodyReport)
		}
		if bodyReport.Lumps != 1 || bodyReport.Voids != 0 {
			t.Fatalf("body topology is not one solid lump: lumps=%d voids=%d", bodyReport.Lumps, bodyReport.Voids)
		}
	}
}

// BodyReport returns the verification record for body, or fails the test when
// the body was not included in the document report.
func BodyReport(t *testing.T, doc *decad.Document, body *decad.Body) *decad.BodyReport {
	t.Helper()
	report, err := doc.Verify(t.Context())
	if err != nil {
		t.Fatalf("verify failed: %v", err)
	}
	for _, bodyReport := range report.Bodies {
		if bodyReport.Body == body {
			return bodyReport
		}
	}
	t.Fatalf("verification report omitted body %p", body)
	return nil
}
