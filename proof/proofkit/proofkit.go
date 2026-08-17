// Package proofkit is the shared harness for the gear proofs.
//
// A proof answers one question: does this modelling workflow hold together? It
// rebuilds a gear's sketch in the sketch engine and checks that the constraint
// scheme closes, before any Fusion code is written. A scheme that cannot reach
// DOF 0 here is a specification defect, found in seconds rather than in a Fusion
// session.
//
// The split is deliberate. proofkit runs and judges; it holds no gear geometry.
// A cycloidal proof has no involute in it, so shared tooth math belongs in its
// own package that the gears which need it import.
//
// A gear's proof is a table of parameter cases and a build function:
//
//	func TestSpurGearProfile(t *testing.T) {
//		proofkit.Run(t, cases, buildGearProfile)
//	}
//
// proofkit gives each case a fresh sketch, runs the build, and gates the result.
package proofkit

import (
	"context"
	"fmt"
	"sort"
	"strings"
	"testing"

	"github.com/lestrrat-3d/sketch"
)

// Case is one parameter set to prove the scheme against. Name becomes the
// subtest name, so it appears verbatim in any failure.
type Case struct {
	Name   string
	Params map[string]float64
}

// Build draws one case's geometry into s.
//
// It should call [Step] as it moves between steps, so a failure can say where it
// happened, and [Unmodelled] for a case the proof cannot represent — never
// return quietly, which reads as a pass.
type Build func(t testing.TB, s *sketch.Sketch, p map[string]float64)

// Run proves build against every case, one subtest each.
//
// Each case gets a fresh world and sketch, and is gated with [RequireSound]
// after build returns. A case that creates no authored points or entities fails
// before solving. A case that fails does not stop the others. At least one case
// must complete; a proof table where every case is unmodelled proves no sketch.
func Run(t *testing.T, cases []Case, build Build) {
	t.Helper()
	if len(cases) == 0 {
		t.Fatal("proofkit: no cases — a proof with nothing to prove passes vacuously")
	}
	completed := 0
	for _, c := range cases {
		c := c
		t.Run(c.Name, func(t *testing.T) {
			s := NewSketch(t)
			build(t, s, c.Params)
			if len(s.Points()) == 0 && len(s.Entities()) == 0 {
				t.Fatalf("proofkit: case %q created no authored geometry", c.Name)
			}
			RequireSound(t, s)
			completed++
		})
	}
	if completed == 0 && !t.Failed() {
		t.Fatal("proofkit: no non-skipped proof cases completed — every proof must prove at least one sketch")
	}
}

// NewSketch returns an empty sketch on the world XY plane.
func NewSketch(t testing.TB) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatalf("proofkit: create sketch: %v", err)
	}
	return s
}

// Step records which part of the build is running. The line only surfaces when
// the case fails or under -v, so it costs nothing to call on every step.
func Step(t testing.TB, format string, args ...any) {
	t.Helper()
	t.Logf("step: "+format, args...)
}

// Unmodelled skips the case, naming what the proof cannot represent.
//
// Use this rather than returning early. A case the proof cannot model is not a
// case that passed, and go test reports a skip as a skip.
func Unmodelled(t testing.TB, format string, args ...any) {
	t.Helper()
	t.Skipf("not modelled: "+format, args...)
}

// RequireSound solves the sketch and gates it on the engine's own verdict.
//
// The gate is [sketch.VerificationReport.Check], which asks for more than DOF 0:
// no conflicting or redundant constraints, no stale or broken reference
// geometry, valid profiles, a constraint system that is not near-singular, and —
// since the probe runs — no discrete ambiguity. Every failed condition is
// reported, so one run names all of them rather than the first.
//
// Nothing is waived here. A sketch can reach DOF 0 and still admit a mirrored or
// 180-degree-rotated solution that the seed happens to avoid, which is how a
// tooth ends up built on the wrong branch, so ambiguity fails like anything else.
// A gear whose scheme is ambiguous by design has to say so at its call site, and
// give the reason.
//
// A failure reports what the solver found rather than classifying the cause.
// Whether the geometry is wrong or the order of operations is wrong is a
// judgement for whoever reads it.
func RequireSound(t testing.TB, s *sketch.Sketch) {
	t.Helper()
	ctx := context.Background()

	res, err := s.Solve(ctx)
	if err != nil {
		t.Fatalf("solve failed: %v", err)
	}
	if !res.Converged {
		t.Fatalf("solver did not converge: residual %.3e, DOF %d, %d redundant",
			res.Residual, res.DOF, res.Redundant)
	}

	rep := s.Verify(ctx, sketch.WithProbe())
	failed := rep.Check()
	if failed == nil {
		return
	}
	for _, reason := range failed.Unwrap() {
		t.Error(reason)
	}
	t.Log(detail(s, rep))
}

// detail adds the specifics behind a failed condition. Check names what failed;
// this says which constraints and points were involved.
func detail(s *sketch.Sketch, rep *sketch.VerificationReport) string {
	var b strings.Builder
	fmt.Fprintf(&b, "DOF=%d conditioning=%.2e", rep.DOF, rep.Conditioning)

	if n := len(rep.Conflicts); n > 0 {
		fmt.Fprintf(&b, "\n  %d conflicting constraint(s):", n)
		for _, c := range rep.Conflicts {
			fmt.Fprintf(&b, "\n    %s", constraintName(s, c.Constraint))
			if len(c.With) > 0 {
				names := make([]string, 0, len(c.With))
				for _, w := range c.With {
					names = append(names, constraintName(s, w))
				}
				fmt.Fprintf(&b, " fights %s", strings.Join(names, ", "))
			}
		}
	}
	if n := len(rep.Redundant); n > 0 {
		fmt.Fprintf(&b, "\n  %d redundant constraint(s):", n)
		for _, c := range rep.Redundant {
			fmt.Fprintf(&b, "\n    %s", constraintName(s, c))
		}
	}
	if n := len(rep.FreePoints); n > 0 {
		fmt.Fprintf(&b, "\n  %d point(s) still free:", n)
		names := make([]string, 0, n)
		for _, p := range rep.FreePoints {
			names = append(names, pointName(p))
		}
		sort.Strings(names)
		fmt.Fprintf(&b, " %s", strings.Join(names, ", "))
	}
	if len(rep.InvalidProfiles) > 0 {
		fmt.Fprintf(&b, "\n  %d of %d detected region(s) are not extrudable",
			len(rep.InvalidProfiles), len(rep.Profiles))
	}
	if rep.Probe != nil && rep.Probe.Ambiguous() {
		fmt.Fprintf(&b, "\n  %d discrete configurations satisfy these constraints, so the "+
			"result depends on where the geometry was seeded", len(rep.Probe.Configurations))
	}
	if !rep.ParametersValid {
		fmt.Fprintf(&b, "\n  parameter expressions did not evaluate: %v", rep.ParameterErrors)
	}
	return b.String()
}

func constraintName(s *sketch.Sketch, c sketch.Constraint) string {
	if n := s.ConstraintName(c); n != "" {
		return n
	}
	return fmt.Sprintf("%T", c)
}

func pointName(p *sketch.Point) string {
	if n := p.Name(); n != "" {
		return n
	}
	return fmt.Sprintf("point#%d", p.ID())
}
