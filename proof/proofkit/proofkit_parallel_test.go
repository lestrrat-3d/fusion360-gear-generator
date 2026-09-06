package proofkit

import (
	"context"
	"errors"
	"flag"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/sketch"
)

// The serial fixtures next door count with a plain int, which is what the
// serial path allows. Everything here counts with an atomic, because these
// cases really do run at the same time.

func TestRunParallelAllowsCompletedCase(t *testing.T) {
	var built atomic.Int64
	RunParallel(t, []Case{
		{Name: "complete", Params: map[string]float64{}},
	}, parallelCountingBuild(&built))

	if built.Load() != 1 {
		t.Fatalf("completed case count: build=%d", built.Load())
	}
}

func TestRunParallelAllowsMixedCompletedAndSkippedCases(t *testing.T) {
	var built atomic.Int64
	RunParallel(t, []Case{
		{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		{Name: "complete", Params: map[string]float64{}},
	}, parallelCountingBuild(&built))

	if built.Load() != 2 {
		t.Fatalf("mixed case count: build=%d", built.Load())
	}
}

// A skipped case must not count as completed, exactly as it does not serially.
func TestRunParallelRejectsAllSkippedCases(t *testing.T) {
	if os.Getenv("PROOFKIT_PARALLEL_ALL_SKIPPED_HELPER") == "1" {
		RunParallel(t, []Case{
			{Name: "skipped_one", Params: map[string]float64{skipCase: 1}},
			{Name: "skipped_two", Params: map[string]float64{skipCase: 1}},
		}, parallelCountingBuild(new(atomic.Int64)))
		t.Fatal("RunParallel returned after every case skipped")
	}

	output, err := runHelper(t, "^TestRunParallelRejectsAllSkippedCases$",
		"PROOFKIT_PARALLEL_ALL_SKIPPED_HELPER=1")
	if err == nil {
		t.Fatalf("all-skipped RunParallel passed; output:\n%s", output)
	}
	if !strings.Contains(output, "proofkit: no non-skipped proof cases completed") {
		t.Fatalf("all-skipped failure did not explain the invariant; output:\n%s", output)
	}
}

// A build that authors nothing fails the same way it does serially, and names
// the case that did it rather than some sibling that happened to run beside it.
func TestRunParallelRejectsEmptyBuild(t *testing.T) {
	if os.Getenv("PROOFKIT_PARALLEL_EMPTY_BUILD_HELPER") == "1" {
		RunParallel(t, []Case{
			{Name: "authored", Params: map[string]float64{}},
			{Name: "empty", Params: map[string]float64{emptyCase: 1}},
		}, parallelCountingBuild(new(atomic.Int64)))
		t.Fatal("RunParallel returned after an empty build")
	}

	output, err := runHelper(t, "^TestRunParallelRejectsEmptyBuild$",
		"PROOFKIT_PARALLEL_EMPTY_BUILD_HELPER=1")
	if err == nil {
		t.Fatalf("empty build passed; output:\n%s", output)
	}
	if !strings.Contains(output, "proofkit: case \"empty\" created no authored geometry") {
		t.Fatalf("empty-build failure did not name the failing case; output:\n%s", output)
	}
}

// An under-constrained sketch must still fail the gate. This is the failed-gate
// counterpart of the failed-build case above: the build authors geometry, so it
// reaches RequireSound, and RequireSound is what refuses it.
func TestRunParallelRejectsUnsoundSketch(t *testing.T) {
	if os.Getenv("PROOFKIT_PARALLEL_UNSOUND_HELPER") == "1" {
		RunParallel(t, []Case{
			{Name: "sound", Params: map[string]float64{}},
			{Name: "free", Params: map[string]float64{freeCase: 1}},
		}, parallelCountingBuild(new(atomic.Int64)))
		t.Fatal("RunParallel returned after an unsound sketch")
	}

	output, err := runHelper(t, "^TestRunParallelRejectsUnsoundSketch$",
		"PROOFKIT_PARALLEL_UNSOUND_HELPER=1")
	if err == nil {
		t.Fatalf("unsound sketch passed; output:\n%s", output)
	}
	if !strings.Contains(output, "cases/free") {
		t.Fatalf("unsound failure did not name the failing case; output:\n%s", output)
	}
}

// The invariant the grouping subtest exists for: RunParallel must not return
// while a case is still running. A case that had not finished would leave the
// counter short, and would also make the completion check below read a value
// that was still moving.
func TestRunParallelReturnsOnlyAfterEveryCaseFinishes(t *testing.T) {
	const cases = 8
	var entered, finished atomic.Int64

	table := make([]Case, 0, cases)
	for i := range cases {
		table = append(table, Case{Name: "case_" + strconv.Itoa(i), Params: map[string]float64{}})
	}

	RunParallel(t, table, func(t testing.TB, s *sketch.Sketch, _ map[string]float64) {
		entered.Add(1)
		// Long enough that a runner returning before its children finished would
		// almost certainly read a short count below.
		time.Sleep(25 * time.Millisecond)
		anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
		anchor.SetName("fixture anchor")
		finished.Add(1)
	})

	if entered.Load() != cases {
		t.Fatalf("cases entered: got %d, want %d", entered.Load(), cases)
	}
	if finished.Load() != cases {
		t.Fatalf("RunParallel returned with %d of %d cases finished", finished.Load(), cases)
	}
}

// The cases genuinely overlap rather than merely being allowed to. Every case
// waits for all of them to arrive, so a serial runner would deadlock and a
// bounded one would time out; both are reported rather than hung.
//
// The check is skipped when -parallel is below the case count, because the
// runner is then correct to hold cases back.
func TestRunParallelActuallyOverlaps(t *testing.T) {
	const cases = 4
	if limit := parallelLimit(t); limit < cases {
		t.Skipf("-test.parallel is %d, below the %d cases this fixture needs", limit, cases)
	}

	arrived := make(chan struct{}, cases)
	release := make(chan struct{})
	var overlapped atomic.Bool

	table := make([]Case, 0, cases)
	for i := range cases {
		table = append(table, Case{Name: "case_" + strconv.Itoa(i), Params: map[string]float64{}})
	}

	RunParallel(t, table, func(t testing.TB, s *sketch.Sketch, _ map[string]float64) {
		anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
		anchor.SetName("fixture anchor")

		arrived <- struct{}{}
		if len(arrived) == cases {
			overlapped.Store(true)
			close(release)
		}
		select {
		case <-release:
		case <-time.After(30 * time.Second):
			t.Error("timed out waiting for the other cases to start; they did not overlap")
		}
	})

	if !overlapped.Load() {
		t.Fatal("no moment had every case running at once")
	}
}

// Two independent parent tests, and one parent running the table twice, are
// where a package-level counter or a shared table would show up.
func TestRunParallelIsIndependentPerParent(t *testing.T) {
	var first atomic.Int64
	RunParallel(t, twoCases(), parallelCountingBuild(&first))
	if first.Load() != 2 {
		t.Fatalf("first invocation build count: %d", first.Load())
	}

	var second atomic.Int64
	RunParallel(t, twoCases(), parallelCountingBuild(&second))
	if second.Load() != 2 {
		t.Fatalf("second invocation build count: %d", second.Load())
	}
}

func TestRunParallelInASecondParentTest(t *testing.T) {
	var built atomic.Int64
	RunParallel(t, twoCases(), parallelCountingBuild(&built))
	if built.Load() != 2 {
		t.Fatalf("build count: %d", built.Load())
	}
}

// The parallel path hands each case its own copy of Params, so a build that
// writes to the map it is given cannot be seen by a sibling or by a later
// invocation. The shared table below is deliberately reused.
func TestRunParallelCopiesCaseParams(t *testing.T) {
	shared := []Case{
		{Name: "writer_one", Params: map[string]float64{"seed": 1}},
		{Name: "writer_two", Params: map[string]float64{"seed": 2}},
	}

	RunParallel(t, shared, func(t testing.TB, s *sketch.Sketch, p map[string]float64) {
		p["written"] = p["seed"]
		anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
		anchor.SetName("fixture anchor")
	})

	for _, c := range shared {
		if _, written := c.Params["written"]; written {
			t.Fatalf("case %q had its caller-owned Params written through", c.Name)
		}
	}
}

func twoCases() []Case {
	return []Case{
		{Name: "one", Params: map[string]float64{}},
		{Name: "two", Params: map[string]float64{}},
	}
}

// parallelLimit reports the -test.parallel value this binary is running under.
func parallelLimit(t *testing.T) int {
	t.Helper()
	f := flag.Lookup("test.parallel")
	if f == nil {
		t.Fatal("no -test.parallel flag; the testing package changed shape")
	}
	limit, err := strconv.Atoi(f.Value.String())
	if err != nil {
		t.Fatalf("cannot read -test.parallel %q: %v", f.Value.String(), err)
	}
	return limit
}

// runHelper re-runs one test in a child process with env set, which is how a
// fixture observes a t.Fatal the parent process would treat as its own failure.
func runHelper(t *testing.T, run, env string) (string, error) {
	t.Helper()
	// Reject nesting before starting a process, even if a fixture accidentally
	// checks its own environment marker after calling runHelper.
	if os.Getenv(helperProcessEnv) != "" {
		return "", errNestedHelper
	}
	ctx, cancel := context.WithTimeout(t.Context(), 15*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, os.Args[0], "-test.run="+run, "-test.timeout=10s")
	cmd.Env = append(os.Environ(), env, helperProcessEnv+"=1")
	cmd.WaitDelay = time.Second
	output, err := cmd.CombinedOutput()
	if ctx.Err() != nil {
		t.Fatalf("proofkit helper exceeded its deadline: %v\n%s", ctx.Err(), output)
	}
	return string(output), err
}

const helperProcessEnv = "PROOFKIT_HELPER_PROCESS"

var errNestedHelper = errors.New("proofkit: helper subprocess cannot launch another helper")

func TestRunHelperRejectsNestedProcess(t *testing.T) {
	t.Setenv(helperProcessEnv, "1")
	// This target is harmless even if the nesting guard regresses.
	output, err := runHelper(t, "^TestRunAllowsCompletedCase$", "PROOFKIT_UNUSED_HELPER=1")
	if !errors.Is(err, errNestedHelper) || output != "" {
		t.Fatalf("nested helper was not rejected before execution: err=%v, output=%q", err, output)
	}
}

const (
	emptyCase = "empty"
	freeCase  = "free"
)

func parallelCountingBuild(count *atomic.Int64) Build {
	return func(t testing.TB, s *sketch.Sketch, params map[string]float64) {
		count.Add(1)
		if params[skipCase] == 1 {
			Unmodelled(t, "fixture case is intentionally unsupported")
		}
		if params[emptyCase] == 1 {
			return
		}
		anchor := s.CreateReferencePoint(0, 0, "anchorPoint")
		anchor.SetName("fixture anchor")
		if params[freeCase] == 1 {
			// An authored point, unlike a reference point, carries degrees of
			// freedom until something ties it down. Nothing does, which is what
			// RequireSound refuses.
			free := s.CreatePoint(10, 10)
			free.SetName("fixture free point")
		}
	}
}
