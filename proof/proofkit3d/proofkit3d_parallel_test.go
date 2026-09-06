package proofkit3d

import (
	"flag"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
)

// The serial fixtures next door count with a plain int, which is what the
// serial path allows. Everything here counts with an atomic, because these
// cases really do run at the same time.

func TestRunWithGateParallelAllowsCompletedCase(t *testing.T) {
	var built, gated, asserted atomic.Int64
	RunWithGateParallel(t, []Case{
		{Name: "complete", Params: map[string]float64{}},
	}, parallelCountingBuild(&built), parallelCountingGate(&gated),
		parallelCountingAssert(&asserted))

	if built.Load() != 1 || gated.Load() != 1 || asserted.Load() != 1 {
		t.Fatalf("completed case counts: build=%d gate=%d assert=%d",
			built.Load(), gated.Load(), asserted.Load())
	}
}

// A skipped case reaches neither the gate nor the assertion, exactly as it does
// not serially.
func TestRunWithGateParallelAllowsMixedCompletedAndSkippedCases(t *testing.T) {
	var built, gated, asserted atomic.Int64
	RunWithGateParallel(t, []Case{
		{Name: "skipped", Params: map[string]float64{skipCase: 1}},
		{Name: "complete", Params: map[string]float64{}},
	}, parallelCountingBuild(&built), parallelCountingGate(&gated),
		parallelCountingAssert(&asserted))

	if built.Load() != 2 || gated.Load() != 1 || asserted.Load() != 1 {
		t.Fatalf("mixed case counts: build=%d gate=%d assert=%d",
			built.Load(), gated.Load(), asserted.Load())
	}
}

func TestRunWithGateParallelRejectsAllSkippedCases(t *testing.T) {
	if os.Getenv("PROOFKIT3D_PARALLEL_ALL_SKIPPED_HELPER") == "1" {
		RunWithGateParallel(t, []Case{
			{Name: "skipped_one", Params: map[string]float64{skipCase: 1}},
			{Name: "skipped_two", Params: map[string]float64{skipCase: 1}},
		}, parallelCountingBuild(new(atomic.Int64)), parallelCountingGate(new(atomic.Int64)),
			parallelCountingAssert(new(atomic.Int64)))
		t.Fatal("RunWithGateParallel returned after every case skipped")
	}

	output, err := runHelper(t, "^TestRunWithGateParallelRejectsAllSkippedCases$",
		"PROOFKIT3D_PARALLEL_ALL_SKIPPED_HELPER=1")
	if err == nil {
		t.Fatalf("all-skipped RunWithGateParallel passed; output:\n%s", output)
	}
	if !strings.Contains(output, "proofkit3d: no non-skipped proof cases completed") {
		t.Fatalf("all-skipped failure did not explain the invariant; output:\n%s", output)
	}
}

// A failing gate names the case that failed, not a sibling that happened to run
// beside it. This is the failed-gate fixture.
func TestRunWithGateParallelReportsTheFailingGateCase(t *testing.T) {
	if os.Getenv("PROOFKIT3D_PARALLEL_GATE_HELPER") == "1" {
		RunWithGateParallel(t, []Case{
			{Name: "sound", Params: map[string]float64{}},
			{Name: "gated_out", Params: map[string]float64{failGate: 1}},
		}, parallelCountingBuild(new(atomic.Int64)), failingGate(),
			parallelCountingAssert(new(atomic.Int64)))
		t.Fatal("RunWithGateParallel returned after a failing gate")
	}

	output, err := runHelper(t, "^TestRunWithGateParallelReportsTheFailingGateCase$",
		"PROOFKIT3D_PARALLEL_GATE_HELPER=1")
	if err == nil {
		t.Fatalf("failing gate passed; output:\n%s", output)
	}
	if !strings.Contains(output, "cases/gated_out") {
		t.Fatalf("gate failure did not name the failing case; output:\n%s", output)
	}
	if strings.Contains(output, "cases/sound") {
		t.Fatalf("gate failure blamed a passing sibling; output:\n%s", output)
	}
}

// The failed-assert fixture. The gate passes, so only the assertion can refuse
// the case, and the refusal must land on that case.
func TestRunWithGateParallelReportsTheFailingAssertCase(t *testing.T) {
	if os.Getenv("PROOFKIT3D_PARALLEL_ASSERT_HELPER") == "1" {
		RunWithGateParallel(t, []Case{
			{Name: "asserted", Params: map[string]float64{}},
			{Name: "refused", Params: map[string]float64{failAssert: 1}},
		}, parallelCountingBuild(new(atomic.Int64)), parallelCountingGate(new(atomic.Int64)),
			failingAssert())
		t.Fatal("RunWithGateParallel returned after a failing assertion")
	}

	output, err := runHelper(t, "^TestRunWithGateParallelReportsTheFailingAssertCase$",
		"PROOFKIT3D_PARALLEL_ASSERT_HELPER=1")
	if err == nil {
		t.Fatalf("failing assertion passed; output:\n%s", output)
	}
	if !strings.Contains(output, "cases/refused") {
		t.Fatalf("assertion failure did not name the failing case; output:\n%s", output)
	}
}

// The failed-build fixture, and the proof that a build failure stops its own
// case before the gate and the assertion see it.
func TestRunWithGateParallelStopsAFailedBuildBeforeTheGate(t *testing.T) {
	if os.Getenv("PROOFKIT3D_PARALLEL_BUILD_HELPER") == "1" {
		var gated, asserted atomic.Int64
		RunWithGateParallel(t, []Case{
			{Name: "broken", Params: map[string]float64{failBuild: 1}},
		}, parallelCountingBuild(new(atomic.Int64)), parallelCountingGate(&gated),
			parallelCountingAssert(&asserted))
		t.Fatalf("RunWithGateParallel returned after a failed build: gate=%d assert=%d",
			gated.Load(), asserted.Load())
	}

	output, err := runHelper(t, "^TestRunWithGateParallelStopsAFailedBuildBeforeTheGate$",
		"PROOFKIT3D_PARALLEL_BUILD_HELPER=1")
	if err == nil {
		t.Fatalf("failed build passed; output:\n%s", output)
	}
	if !strings.Contains(output, "fixture build refused this case") {
		t.Fatalf("build failure did not surface; output:\n%s", output)
	}
}

// The invariant the grouping subtest exists for. A case still running when the
// runner returned would leave the counter short.
func TestRunWithGateParallelReturnsOnlyAfterEveryCaseFinishes(t *testing.T) {
	const cases = 8
	var finished atomic.Int64

	table := make([]Case, 0, cases)
	for i := range cases {
		table = append(table, Case{Name: "case_" + strconv.Itoa(i), Params: map[string]float64{}})
	}

	RunWithGateParallel(t, table,
		func(t *testing.T, _ *decad.Document, _ map[string]float64) []*decad.Body {
			time.Sleep(25 * time.Millisecond)
			return nil
		},
		func(*testing.T, *decad.Document, []*decad.Body) {},
		func(*testing.T, *decad.Document, []*decad.Body, map[string]float64) {
			finished.Add(1)
		})

	if finished.Load() != cases {
		t.Fatalf("RunWithGateParallel returned with %d of %d assertions finished",
			finished.Load(), cases)
	}
}

// The cases genuinely overlap. A serial runner would deadlock here and a
// bounded one would time out; both are reported rather than hung, and the
// fixture skips when -parallel is too low to allow the overlap.
func TestRunWithGateParallelActuallyOverlaps(t *testing.T) {
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

	RunWithGateParallel(t, table,
		func(t *testing.T, _ *decad.Document, _ map[string]float64) []*decad.Body {
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
			return nil
		},
		func(*testing.T, *decad.Document, []*decad.Body) {},
		func(*testing.T, *decad.Document, []*decad.Body, map[string]float64) {})

	if !overlapped.Load() {
		t.Fatal("no moment had every case running at once")
	}
}

// Two independent parents, and one parent running the table twice, are where a
// package-level counter or a shared table would show up.
func TestRunWithGateParallelIsIndependentPerParent(t *testing.T) {
	var first atomic.Int64
	RunSolidParallelFixture(t, &first)
	if first.Load() != 2 {
		t.Fatalf("first invocation build count: %d", first.Load())
	}

	var second atomic.Int64
	RunSolidParallelFixture(t, &second)
	if second.Load() != 2 {
		t.Fatalf("second invocation build count: %d", second.Load())
	}
}

func TestRunWithGateParallelInASecondParentTest(t *testing.T) {
	var built atomic.Int64
	RunSolidParallelFixture(t, &built)
	if built.Load() != 2 {
		t.Fatalf("build count: %d", built.Load())
	}
}

// RunSolidParallelFixture exercises the real RunSolidParallel entry point,
// gate included, on two blocks that are genuinely solid.
func RunSolidParallelFixture(t *testing.T, built *atomic.Int64) {
	t.Helper()
	RunSolidParallel(t, []Case{
		{Name: "one", Params: map[string]float64{}},
		{Name: "two", Params: map[string]float64{}},
	}, func(t *testing.T, doc *decad.Document, _ map[string]float64) []*decad.Body {
		built.Add(1)
		return separatedBlocksIn(t, doc, 0, 1)
	}, func(*testing.T, *decad.Document, []*decad.Body, map[string]float64) {})
}

// Each case gets its own copy of Params, so a build that writes to the map it
// is given cannot reach a sibling or a later invocation.
func TestRunWithGateParallelCopiesCaseParams(t *testing.T) {
	shared := []Case{
		{Name: "writer_one", Params: map[string]float64{"seed": 1}},
		{Name: "writer_two", Params: map[string]float64{"seed": 2}},
	}

	RunWithGateParallel(t, shared,
		func(t *testing.T, _ *decad.Document, p map[string]float64) []*decad.Body {
			p["written"] = p["seed"]
			return nil
		},
		func(*testing.T, *decad.Document, []*decad.Body) {},
		func(*testing.T, *decad.Document, []*decad.Body, map[string]float64) {})

	for _, c := range shared {
		if _, written := c.Params["written"]; written {
			t.Fatalf("case %q had its caller-owned Params written through", c.Name)
		}
	}
}

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

func runHelper(t *testing.T, run, env string) (string, error) {
	t.Helper()
	cmd := exec.Command(os.Args[0], "-test.run="+run)
	cmd.Env = append(os.Environ(), env)
	output, err := cmd.CombinedOutput()
	return string(output), err
}

const (
	failBuild  = "failBuild"
	failGate   = "failGate"
	failAssert = "failAssert"
)

func parallelCountingBuild(count *atomic.Int64) Build {
	return func(t *testing.T, _ *decad.Document, params map[string]float64) []*decad.Body {
		count.Add(1)
		if params[skipCase] == 1 {
			Unmodelled(t, "fixture case is intentionally unsupported")
		}
		if params[failBuild] == 1 {
			t.Fatal("fixture build refused this case")
		}
		return nil
	}
}

func parallelCountingGate(count *atomic.Int64) Gate {
	return func(_ *testing.T, _ *decad.Document, _ []*decad.Body) {
		count.Add(1)
	}
}

func parallelCountingAssert(count *atomic.Int64) Assert {
	return func(_ *testing.T, _ *decad.Document, _ []*decad.Body, _ map[string]float64) {
		count.Add(1)
	}
}

// failingGate refuses only the case whose params ask for it, so the fixture can
// prove the refusal lands on that case and not on its sibling.
func failingGate() Gate {
	var seen atomic.Int64
	return func(t *testing.T, _ *decad.Document, _ []*decad.Body) {
		seen.Add(1)
		if strings.HasSuffix(t.Name(), "gated_out") {
			t.Fatal("fixture gate refused this case")
		}
	}
}

func failingAssert() Assert {
	return func(t *testing.T, _ *decad.Document, _ []*decad.Body, params map[string]float64) {
		if params[failAssert] == 1 {
			t.Fatal("fixture assertion refused this case")
		}
	}
}
