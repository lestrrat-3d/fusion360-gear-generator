# Case parallelism — result

Result record for the case-parallelism task, following the execution protocol's completion rules,
and for follow-up F1 of the post-#108 pipeline timing audit.

Those three documents are not on this branch, and are named here by branch rather than linked so
the reference does not rot into a broken link when they merge:
`docs/proof-optimization/case-parallelism.md` and `execution.md` on `docs-proof-optimization-handoff`,
and `docs/proof-optimization/2026-09-06-pipeline-timing-audit.md` on `chore-pipeline-timing-audit`.
Turn these into ordinary links once they share a branch with this file.

Measured 2026-09-06 on linux/amd64 (WSL2), AMD Ryzen 9 7900X3D, 24 logical CPUs, Go toolchain
1.26.8 resolving `proof/go.mod`. The host is shared and was not idle, which is stated in full under
"Measurement conditions" below. Every figure is a median of separate `-count=1`
runs with its own range beside it.

## Verdict

| Item | Verdict |
|---|---|
| Opt-in parallel cases in both harnesses | Implemented and accepted. `proof/bevelgear` runs 2.06× faster at `-parallel 4`, 37.99 s against 78.20 s. |
| Memory | Not a constraint. Peak resident memory goes from 26.0 MB to 32.0 MB. The task's reason for deferring solid cases does not survive measurement. |
| Solid cases | Included. Sketch cases alone reach 1.19×, because `proof/bevelgear` spends about 16 s of its 75 in sketch proofs. |
| `stepCircularPattern` | Excluded by name. Its proof carries per-case readings in package-level vars, and running its cases together produced a wrong verdict. |
| Serial APIs | Unchanged in behaviour. No regression is established; see "What the small differences are". |
| Any gear enabled | No. This branch adds the capability only. See "How to enable a gear safely" for the route, which is not a full recompile. |
| CI benefit | Not measured and not claimed, because no gear is enabled on this branch. |

A PR is warranted on these numbers. It is not merged.

## What was changed

| Item | Value |
|---|---|
| Repository | `lestrrat-3d/fusion360-gear-generator` |
| Worktree | `.worktrees/feat-parallel-proof-cases` |
| Branch | `feat-parallel-proof-cases` |
| Base commit | `322eea3` ("record pipeline timing evidence (#108)"), which is `origin/main` |
| Engine pins | unchanged: sketch `34765bc10360`, decad `1080cbf4c0a7`, read from `proof/go.mod` |
| Harness files | `proof/proofkit/proofkit.go`, `proof/proofkit3d/proofkit3d.go` |
| New fixtures | `proof/proofkit/proofkit_parallel_test.go`, `proof/proofkit3d/proofkit3d_parallel_test.go` |
| Generator regressions | `.claude/skills/generate-gear/test_check_compile.py`, `test_scaffold_proof.py` |

No dependency pin moved and no gear's step list is changed on this branch. The new runners are
opt-in and nothing calls them yet outside their own fixtures, which is why the measurement below
uses a temporary, scripted flip that is reverted after every use.

## The audit, and what it found

A fresh document per case is not evidence of independence, so the audit went past that. Five
classes of sharing were checked across `proof/bevelgear`, `proof/cycloidal`, `proof/spurgear`,
`proof/helicalgear` and `proof/herringbonegear`.

| Class | Finding |
|---|---|
| Environment mutation | None. `os.Setenv`, `t.Setenv`, `os.Unsetenv` and `os.Chdir` appear nowhere under `proof/`. |
| Filesystem writes | Two, both outside the harness. `proof/render/render.go:315` writes a PNG behind the `-render.out` flag, and `proof/cmd/genexamples/main.go:88` creates its output directory. Neither package registers a harness case, and both stay serial. |
| Existing concurrency | None. There is no `go func`, no `sync` import and no `t.Parallel()` anywhere under `proof/`. |
| Shared `Params` maps | The same table is reused by several tests, so the maps are shared by reference. Every write found is at table-construction time into a map that was just made (`spurgear`'s `at`, `bevelgear`'s `params`, `perGear` and `perGearSketch`, `cycloidal`'s case helper), and `bevelgear`'s `requireRejected`/`requireAccepted` copy the base map before writing. No build or assert writes the map it is handed. The parallel path copies it anyway; see below. |
| Package-level state written at run time | **One real case, and it is the finding.** |

`.tmp/audit-package-state.py` is the scan and `.tmp/audit-package-state.txt` is its output. It
lists every package-level `var` in a gear proof package and every site that assigns one from inside
a function, in two passes: bare assignment, and mutation through an index, a field or `append`. The
second pass found nothing in any package.

The first pass reports three things, and two of them are not findings. `proof/cycloidal`'s
`housingColor`, `discColor`, `camColor` and `plateColor` are each reported at their own declaration
line, so they are initialisers rather than run-time writes. The blank identifier `_` is reported in
`proof/bevelgear` because assigning to it is not a write to anything. What is left is one real
case:

```
proof/bevelgear/solids_test.go:1188  patternSeedAzimuth
proof/bevelgear/solids_test.go:1189  patternSeedRadius
proof/bevelgear/solids_test.go:1190  patternSeedHeight
proof/bevelgear/solids_test.go:1191  patternSeedVolume
```

`stepCircularPattern` measures its seed solid into those four vars and then retires the seed with
`Placed`, and `assertCircularPattern` reads them back, because the seed cannot be measured after
the move. The declaration's own comment states the assumption: "The harness runs one case at a time
in one goroutine."

That is not a hypothetical. An earlier run of this experiment moved every solid step to the
parallel runner, and `TestCircularPattern` failed on both its cases:

```
solids_test.go:1212: Driving pattern copy is the seed, moved:
    got 7.55785957246, want 4.56931838444 (tolerance 4.5693183844356075e-09)
```

Two cases overwrote each other's seed readings. The failure is loud here only because the two gear
sides differ in volume; a table whose cases had similar readings would have produced a **wrong pass**
rather than a failure, which is the reason this step is excluded by name rather than left to be
caught by a future run.

`stepCircularPattern` therefore keeps `proofkit3d.RunSolid`. Nothing else in any gear proof carries
state between a build and its assertion.

## The design

The two harnesses gain opt-in parallel entry points and keep every existing one serial and
behaviour-compatible.

| Package | Serial, unchanged | New, opt-in |
|---|---|---|
| `proofkit` | `Run` | `RunParallel` |
| `proofkit3d` | `Run`, `RunSolid`, `RunWithGate` | `RunSolidParallel`, `RunWithGateParallel` |

Each package now routes every entry point through one unexported `runCases`, which owns the build,
gate, assertion and completion rules. Only the scheduling differs between the serial and parallel
paths, so the parallel path cannot drift from the serial one as the rules change.

### Scheduling

Cases run under a synchronous grouping subtest rather than directly under the proof's own test:

```
TestSliceToothSlabs                 the proof's test, still serial
  cases                             synchronous grouping subtest
    psi_35_left_pinion              t.Parallel()
    psi_35_right_pinion             t.Parallel()
    ...
```

This is forced by how Go schedules subtests, which was read from
`$(go env GOROOT)/src/testing/testing.go` rather than assumed. `T.Parallel` appends the child to
`t.parent.sub`, signals the parent, and then blocks on `<-t.parent.barrier`. The parent closes that
barrier only after **its own function has returned**, and then blocks on `<-sub.signal` for every
child before running cleanups. So cases started directly by `Run` would still be blocked at
`t.Parallel()` when `Run` reached its completion check, and `Run` would return before any case had
done anything. Nesting them under a grouping `t.Run` puts a return between the two: that inner
`t.Run` does not return to `runCases` until every parallel child has finished.

`-parallel` bounds how many cases run at once. Go's `waitParallel` releases the grouping subtest's
own slot before the children start and reacquires it afterwards, so the group does not consume one
of the N.

### Completion accounting

`completed` is an `atomic.Int64` in both harnesses, incremented only after the build, the gate and
the assertion have all returned for that case. A skipped or failed case never reaches it, exactly as
before. The serial path reads the same value it always did. The all-skipped invariant is checked
after the grouping subtest returns, which is after every case has finished, so it reads a settled
number rather than a moving one.

Failure propagation needed no change. Go's `common.Fail` walks the whole parent chain, so a failing
case marks the grouping subtest and the proof's own test failed, and the `!t.Failed()` guard behaves
identically in both paths.

### Params copies

The parallel paths hand each case `maps.Clone(c.Params)`. The audit found no callback that writes
the map it is given, so this changes nothing observable today. It is there because the case tables
are package-level and shared between tests, so a future build that did write one would otherwise
corrupt a sibling case silently. Within one case the copy is shared by build, gate and assert
exactly as the serial path shares the original, so a build that stashes a value in the map for its
own assertion still works.

### The subtest name change

Opting a step in adds one level to every case's name: `TestGearProfiles/cases/lattice_1` rather than
`TestGearProfiles/lattice_1`. This is deliberate and is the visible cost of the grouping subtest.
The serial entry points do not add it, so nothing that has not opted in moves.

Anything that filters on a case path has to account for it. The comparison in
"Coverage and verdicts" normalises `/cases/` back out, for this migration only.

The one `-run` filter written down in the repository is unaffected. `CLAUDE.md` documents
`proof/run.sh --package ./bevelgear -- -run '^TestGearProfiles$' -count=1`, and Go splits a `-run`
expression on `/` and applies each part to one subtest level, so a single-part expression selects
top-level tests and runs everything beneath them. A filter that names a case explicitly, such as
`-run 'TestGearProfiles/lattice_1'`, would need `cases/` inserted.

That reasoning was read out of Go's `-run` handling rather than tested here, since no gear is
enabled on this branch. It has since been confirmed against a real enabled gear on branch
`perf-bevel-parallel-cases`, where the documented filter ran all 16 cases under
`TestGearProfiles/cases/` and passed.

### The generator

`spec/<gear>/steps.md` carries one `proof-run` annotation per `[GO]` step, and
`.claude/skills/generate-gear/scaffold_proof.py` turns those into
`proof/<gear>/zz_registrations_test.go`. The set of legal run methods is not written down anywhere:
`check_compile.py`'s `derive_proof_run_arguments` reads the harness sources and derives it. Adding
the new methods to the harness was therefore enough for both the gate and the generator to accept
them:

```
proofkit.Run                       3
proofkit.RunParallel               3
proofkit3d.Run                     4
proofkit3d.RunSolid                4
proofkit3d.RunSolidParallel        4
proofkit3d.RunWithGate             5
proofkit3d.RunWithGateParallel     5
```

Two regression tests pin that table by hand and are the tripwire for exactly this change. Both were
updated: `test_check_compile.py`'s `test_derived_table_matches_the_harness_signatures`, which is
documented as the place a new run method "fails here and gets reviewed", and
`test_scaffold_proof.py`'s `test_unknown_run_method_names_the_declared_ones`, which asserts on the
list of declared methods a complaint prints.

No generated registration file is changed on this branch. Enabling a gear means changing its step
list, which is a compiled artifact, so the decision has to be written where a regeneration will
reproduce it. See "How to enable a gear safely".

## Coverage and verdicts

`proof/bevelgear` was run with every eligible step flipped to a parallel runner and its full
`-json` inventory compared against the same package's inventory on `origin/main`, by
`(package, test)` and final action. The comparison normalises the `/cases/` level back out, for
this migration only.

| Check | Result |
|---|---|
| Baseline entries | 199 |
| Candidate entries | 218 |
| Missing from candidate | 0 |
| Added beyond the grouping subtests | 0 |
| Verdicts changed | 0 |
| Skips, baseline against candidate | 10 against 10 |
| Cases now running under a parallel group | 173 |

The 19 extra entries are exactly the 19 grouping subtests, one per opted-in step. No test was lost,
no verdict moved, and no skip appeared. `.tmp/verdict-compare-both.txt` holds the comparison and
`.tmp/both-bevelgear.jsonl` the run it reads.

`check_compile.py bevelgear` passes with the flip applied, so the gate accepts the new run methods
in a real step list rather than only in the derived table.

## Measurement

`proof/bevelgear` only. It is the suite's critical path, so its wall time is the suite's wall time,
and it is the package worth deciding on. Three binaries, built once before any sample:

| Binary | Built from |
|---|---|
| `bevelgear-pristine.test` | `origin/main`, from the untouched worktree at `.worktrees/chore-pipeline-timing-audit` |
| `bevelgear-serial.test` | this branch, harness changed, no gear enabled |
| `bevelgear-both.test` | this branch, `bevelgear` flipped to the parallel runners except `stepCircularPattern` |

Three rounds, interleaved, `GOMAXPROCS=8`, `-test.count=1`, wall time from `/usr/bin/time -v`.

| Configuration | n | Median | Range | CPU | Peak RSS | Against this branch's serial |
|---|---|---|---|---|---|---|
| `pristine-main`, `-parallel 1` | 3 | 74.38 s | 73.29 – 74.67 s | 131% | 26.4 MB | 1.05× |
| `baseline-serial`, `-parallel 1` | 3 | 78.20 s | 77.88 – 78.62 s | 129% | 26.0 MB | 1.00× |
| `both-1` | 3 | 81.71 s | 78.97 – 82.17 s | 126% | 30.0 MB | 0.96× |
| `both-2` | 3 | 52.43 s | 52.41 – 56.61 s | 248% | 27.1 MB | 1.49× |
| `both-4` | 3 | 37.99 s | 37.82 – 38.37 s | 397% | 32.0 MB | **2.06×** |

The gain is real, repeatable and cheap in memory. At `-parallel 4` the package takes 37.99 s against
78.20 s, a 2.06× speedup, and the three samples span 0.55 s. Peak resident memory moves from 26.0 MB
to 32.0 MB, which is 6 MB on a machine with 30 GiB. The memory objection the task raised against
solid cases does not survive measurement: the whole test binary's working set is tens of megabytes,
not gigabytes, because decad's documents are discarded as each case ends.

Scaling is close to linear up to 4 and CPU tracks it: 129% serial, 248% at 2, 397% at 4. Nothing
suggests 4 is the ceiling, but 4 is where this was measured and no higher bound is claimed.

### Sketch cases alone are not enough

An earlier block measured the sketch-only variant, before the audit and the memory readings
justified extending to solids. Within that block, and comparing the same binary against itself so
no second binary is involved, `-parallel 1` took 79.38 s (range 78.97 – 79.43) and `-parallel 4`
took 66.78 s (range 66.25 – 68.06), a 1.19× gain. `proof/bevelgear` spends about 16 s of its 75 in
sketch proofs and the rest in solid ones, so sketch-only parallelism cannot reach much beyond that.
This is why the work did not stop where the task suggested it might.

That block's own serial baseline was measured before its `/usr/bin/time` files were overwritten by
the later block, so the 1.19× above is stated as the same binary at two bounds rather than against
a separate serial arm. That comparison is the stronger one anyway, since it involves no second
binary and so no build-layout difference.

### What the small differences are, and why they are not attributable

Three of the numbers above are small, and none of them can be read as a cost of this change. This
host produces run-to-run swings on a single package that are larger than any of them.

`pristine-main` measures 74.38 s against `baseline-serial`'s 78.20 s, a 5% difference between two
binaries whose serial path should behave identically. That was chased down rather than waved away,
and the chase is the interesting part.

**It is not run order.** An A/B of four rounds alternating which binary ran first put
`pristine-main` at a median of 73.45 s (73.23 – 74.29) and `baseline-serial` at 78.87 s
(78.02 – 81.69). Position moved each arm by about 1 – 3 s and never closed the gap.

**It is not the harness.** Narrowing to `TestGearProfiles` alone made the gap look far worse:
15.80 s against 23.93 s, a 51% difference on one test. The refactor adds one closure call and one
atomic increment per case, and that test has 17 cases, so the change would have to cost 480 ms per
case. CPU profiles of both binaries on that test settle it:

| Function | `pristine-main` flat | `baseline-serial` flat |
|---|---|---|
| `sketch.solveLinearInto` | 13.02 s | 17.08 s |
| `sketch.normalEquationsSparseInto` | 2.40 s | 2.47 s |
| `sketch.transposeInto` | 0.72 s | 0.73 s |
| `sketch.jacobianLocalInto` | 0.12 s | 0.12 s |
| `proofkit.runCases.func1` / `.func2` | — | 0 s |
| `proofkit.Run.func1` | 0 s | — |

The call paths are identical, the harness frames carry no time in either, and every function in the
solver loop matches except `solveLinearInto`, which is 31% slower in one binary. More solver
iterations would have raised all of them together. The same amount of work is being done, and one
leaf function this change cannot reach is executing it more slowly.

**It is the host, and it is not static.** Appending one unused, uncalled function to `proofkit.go`
and rebuilding moved `TestGearProfiles` from 23.9 s to 20.2 s. Timing all three binaries
interleaved then showed the same binary landing on different levels in different rounds:

| Round | `pristine-main` | `baseline-serial` | `baseline-layout` |
|---|---|---|---|
| 1 | 19.26 s | 20.11 s | 23.75 s |
| 2 | 19.23 s | 23.77 s | 20.10 s |
| 3 | 15.58 s | 23.70 s | 20.11 s |

`baseline-serial` measured 20.11 s and 23.77 s; `baseline-layout` measured 23.75 s and 20.10 s.
Neither binary changed between its own rounds. The results cluster at discrete levels rather than
scattering, which is what core placement looks like on a Ryzen 9 7900X3D: its two core complexes
differ in cache size and clock, and a dense numeric solve is exactly the workload that notices.
WSL2 reports L3 as one 96 MiB pool shared by all 24 CPUs, so the guest cannot see the boundary and
`taskset` inside it cannot pin across it.

So the honest statement is that this host delivers up to a 50% swing on a single-package run of this
test, independently of the code. The 5% whole-package difference and the 4.5% `both-1` figure both
sit inside that band and establish nothing. No serial-path regression is claimed, and none is ruled
out either; what is shown is that no mechanism for one appears in the profile.

The 2.06× figure is what survives. `baseline-serial` spans 77.88 – 78.62 s and `both-4` spans
37.82 – 38.37 s, two non-overlapping bands a factor of two apart, which is an order of magnitude
beyond the noise this host produces on whole-package runs.

## Measurement conditions

The host was busy and not mine alone, which is stated here rather than buried, because it bounds
what these numbers mean.

Load average sat between 12 and 19 throughout. Roughly 2.6 of the 24 cores were held by processes
belonging to neither this work nor the run: six `review-82-pass-12-start-server` processes from
`lestrrat-go/server-starter`, each about seven days old at roughly 36% CPU, and one `codex` process
about 23 hours old at roughly 46%. None was killed, and none is mine to kill. A second agent
working decad follow-up F2 on the same host held all of its own timing runs for the duration of
each window, and confirmed the foreign processes are not its either.

The controls that survive a busy host were applied. `GOMAXPROCS` was pinned to 8 for every sample,
so no configuration could take the machine. Samples never overlapped. Rounds interleave across
configurations, so drift in the background load lands on all five arms rather than on whichever ran
last. Every sample forces execution with `-test.count=1`. The three binaries were built once,
before any sample, so no build time is inside a measurement and the tree state cannot drift between
configurations.

Wall times are `/usr/bin/time -v`'s own elapsed figure, not the driver's, because on this WSL2 host
the two disagreed by several seconds and the figure that comes from the same tool as the CPU and
memory readings is the one worth trusting.

What this does not support is an absolute claim about how long the suite takes on an idle machine.
The comparison between arms holds, because they all paid the same background cost.

## Validation

| Check | Command | Result |
|---|---|---|
| Harness fixtures, both packages | `go test -count=1 ./proofkit/ ./proofkit3d/` | pass |
| Harness fixtures under the race detector | `go test -count=1 -race ./proofkit/ ./proofkit3d/` | pass, no race reported |
| Overlap fixtures under a bound below the case count | `go test -count=1 -parallel 1 -run ActuallyOverlaps` | both skip with a reason, neither hangs |
| Enabled gear paths under the race detector | `go test -count=1 -race -timeout 40m -run '^(TestGearProfiles\|TestSliceToothSlabs\|TestBoreSketch\|TestResolveInputBounds)$' ./bevelgear/` | pass in 53.75 s, zero `DATA RACE` reports |
| `check_compile.py` regressions | `python3 .claude/skills/generate-gear/test_check_compile.py` | 130 tests, pass |
| `scaffold_proof.py` regressions | `python3 .claude/skills/generate-gear/test_scaffold_proof.py` | 25 tests, pass |
| `check_step_calls.py` regressions | `python3 .claude/skills/generate-gear/test_check_step_calls.py` | 76 tests, pass |
| Compile gate on a flipped gear | `python3 .claude/skills/generate-gear/check_compile.py bevelgear` | OK, 34 steps, 20 proof functions |
| Vet, whole proof module | `go vet ./...` | clean |
| Whole pinned suite | `proof/run.sh` | pass, all 9 packages, engine revisions verified against `proof/go.mod` |

The repository has no `golangci-lint` configuration, so `go vet` is the linting bar here.

### The fixtures, and what each one pins

Both harnesses carry the same set, adapted to what each one gates.

- A complete case, and a mixed table of one skipped and one complete case, pin that a skip reaches
  neither the gate nor the assertion and never counts as completed.
- An all-skipped table pins the completion invariant. It runs in a child process, because the
  invariant is a `t.Fatal` the parent would otherwise treat as its own failure.
- A failed build, a failed gate and a failed assertion each pin that the refusal names the case
  that caused it. The gate and assertion fixtures also check that a passing sibling is **not**
  named, which is the failure mode a parallel runner would introduce.
- A completion fixture runs eight cases that each sleep, then checks after the runner returns that
  all eight finished. A runner that returned while children were still running would read a short
  count.
- An overlap fixture proves the cases really do run at once rather than merely being allowed to:
  every case waits for all of them to arrive. A serial runner would deadlock and a bounded runner
  would time out, so both are reported rather than hung, and the fixture skips when
  `-test.parallel` is below the case count.
- Two independent parent tests, and one parent that runs a table twice, cover repeated invocation.
- A params fixture writes through the map each case is handed and then checks the caller's table
  was not touched.

### The race check needs a raised timeout

Running the whole `proof/bevelgear` package under `-race` hits Go's default 10-minute test timeout
and panics. That is true on `origin/main` with no change of this branch involved; the package takes
about 75 s without the detector and the detector's slowdown carries it past the limit. The targeted
four-test race run above used `-timeout 40m` and finished in 53.75 s, helped by the parallel cases
themselves.

## What is not done

**No gear is enabled on this branch.** The two harnesses gain the capability and nothing calls it
outside its own fixtures, so `origin/main`'s proof runtime is unchanged by merging this.

Enabling a gear means changing `proofkit.Run` to `proofkit.RunParallel` in that gear's
`spec/<gear>/steps.md` annotations. The step list is a compiled artifact. `CLAUDE.md` and
`emit-gear/SKILL.md` both say it is never hand-edited, and the reason applies here: the next
`/compile-gear` run would rewrite the annotation back unless the spec itself asks for the parallel
runner, so a hand edit alone would drift on the first regeneration. Enabling is therefore a separate
change with its own review, and "How to enable a gear safely" gives the route.

### How to enable a gear safely

An earlier draft of this document said enabling "belongs to `/compile-gear`", which reads as an
instruction to run a full recompile. Do not do that. A full recompile deletes hand-written proof
files.

`stage.py` places every drafted proof file and prunes the rest: `plan_actions` computes
`prune` as every `.go` name present in `proof/<gear>/` and absent from the draft's sources, and
`compile-gear/SKILL.md` states the same behaviour at its step 4, "deleting any `.go` file there the
draft no longer produces". The clean-tree guard does not catch it. `classify_destination` refuses a
file that is neither `stage.py`'s own output nor clean against HEAD, so a committed, clean file
passes and is then pruned, leaving only a printed `pruned <path>` line behind.

Two files are exposed today, `proof/bevelgear/render_test.go` and `proof/cycloidal/render_test.go`,
both added by `88b1ea4` (#85). Nothing in `compile-gear/prompt.md` mentions rendering, so a re-draft
would not produce either one. The deletion is recoverable from git, and it is quiet enough to ride
along unnoticed in a large diff.

The safe route writes the decision into the spec and regenerates only the one generated file:

1. Ask for the parallel entry points in `spec/<gear>/instructions.md`, naming the steps that must
   stay serial. This is what makes a later regeneration reproduce the choice instead of reverting it.
2. Move the eligible `proof-run` annotations in `spec/<gear>/steps.md` so the step list matches its
   own source.
3. Regenerate `proof/<gear>/zz_registrations_test.go` with
   `python3 .claude/skills/generate-gear/scaffold_proof.py <gear>`, which is the only generated file
   involved and rewrites nothing else.

Branch `perf-bevel-parallel-cases` carries that work for `bevelgear`. The prune behaviour itself is
raised separately; this section only records why enabling must not go through a full recompile.

The measurements below use a scripted flip (`.tmp/flip_parallel.py`) that rewrites those
annotations, regenerates the registration file through `scaffold_proof.py`, and is reverted from
git afterwards. Every measurement in this document was taken from a pre-built binary, and the
working tree was reverted before and after each build, so no measurement left the flip in place.

**No CI benefit is claimed.** Since no gear is enabled, CI on this branch runs the same serial
proofs it runs today, and the CI figure cannot be measured from this PR. The protocol's rule is to
measure on CI before claiming a CI benefit, so this document claims none. The CI measurement
belongs to the enabling change.

**Only `proof/bevelgear` was measured.** It is the suite's critical path, so it is the package that
decides the suite's wall time, but the per-package gain elsewhere is unmeasured. `cycloidal` is the
package that would become the new constraint, and its own figure is not in this document.

**`stepCircularPattern` stays serial.** Fixing the proof so its cases are independent would mean
carrying the seed readings per case instead of in package vars, which is an edit to
`proof/bevelgear/solids_test.go`. That file is also compile-stage output, so the fix belongs with
the enabling change rather than here.

## Reproduction

From `.worktrees/feat-parallel-proof-cases`, with `sketch` and `decad` checked out beside the
repository at the revisions `proof/go.mod` pins.

```sh
# The audit that found the shared state. Both passes; the second found nothing.
python3 .tmp/audit-package-state.py    # output kept in .tmp/audit-package-state.txt

# Harness fixtures, including the failure, completion and overlap cases
GOWORK=off go -C proof test -mod=readonly -count=1 ./proofkit/ ./proofkit3d/
GOWORK=off go -C proof test -mod=readonly -count=1 -race ./proofkit/ ./proofkit3d/

# The overlap fixtures must skip rather than hang when the bound is below the case count
GOWORK=off go -C proof test -mod=readonly -count=1 -parallel 1 -v \
  -run 'ActuallyOverlaps' ./proofkit/ ./proofkit3d/

# Generator regressions, including the two that pin the derived run table
python3 .claude/skills/generate-gear/test_check_compile.py
python3 .claude/skills/generate-gear/test_scaffold_proof.py
python3 .claude/skills/generate-gear/test_check_step_calls.py

# The derived run table the gate and the generator share
python3 -c "import sys; sys.path.insert(0,'.claude/skills/generate-gear'); import check_compile; \
  print(check_compile.derive_proof_run_arguments())"

# Flip bevelgear, check the gate still accepts it, and revert
python3 .tmp/flip_parallel.py bevelgear both
python3 .claude/skills/generate-gear/check_compile.py bevelgear
python3 .tmp/flip_parallel.py bevelgear --revert

# Race check on the enabled paths. -timeout must be raised: the whole package
# under -race exceeds Go's default 10 minutes on origin/main, unrelated to this change.
python3 .tmp/flip_parallel.py bevelgear both
GOWORK=off go -C proof test -mod=readonly -count=1 -race -timeout 40m \
  -run '^(TestGearProfiles|TestSliceToothSlabs|TestBoreSketch|TestResolveInputBounds)$' ./bevelgear/
python3 .tmp/flip_parallel.py bevelgear --revert

# Build the three binaries the comparison uses, reverting the flip between them
GOWORK=off go -C ../chore-pipeline-timing-audit/proof test -mod=readonly -c \
  -o "$PWD/.tmp/bevelgear-pristine.test" ./bevelgear/          # origin/main, untouched harness
GOWORK=off go -C proof test -mod=readonly -c \
  -o "$PWD/.tmp/bevelgear-serial.test" ./bevelgear/            # this branch, no gear enabled
python3 .tmp/flip_parallel.py bevelgear both
GOWORK=off go -C proof test -mod=readonly -c \
  -o "$PWD/.tmp/bevelgear-both.test" ./bevelgear/              # this branch, bevelgear enabled
python3 .tmp/flip_parallel.py bevelgear --revert

# The comparison. Rounds interleave across configurations, samples never overlap,
# GOMAXPROCS is pinned, and every sample forces execution with -test.count=1.
python3 .tmp/measure_parallel.py 3 8

# The three follow-ups that chased the small pristine-against-branch difference
python3 .tmp/serial-ab.py 4      # order alternated, whole package
python3 .tmp/subset-ab.py 3      # narrowed to one test, order alternated
python3 .tmp/layout-ab.py 3      # three binaries including a no-op layout probe

# The profiles that showed the difference is inside sketch.solveLinearInto
cd proof
../.tmp/bevelgear-pristine.test -test.count=1 -test.run '^TestGearProfiles$' \
  -test.cpuprofile ../.tmp/gp-pristine.cpu
../.tmp/bevelgear-serial.test   -test.count=1 -test.run '^TestGearProfiles$' \
  -test.cpuprofile ../.tmp/gp-serial.cpu
cd ..
go tool pprof -top -cum .tmp/bevelgear-pristine.test .tmp/gp-pristine.cpu
go tool pprof -top -cum .tmp/bevelgear-serial.test   .tmp/gp-serial.cpu

# The whole pinned suite, which is what a handoff means
proof/run.sh
```
