# The gear proofs

`proof/` is a Go module of its own. It rebuilds each gear's sketches and solids in two headless
engines — [sketch](https://github.com/lestrrat-3d/sketch) for constrained 2D geometry and
[decad](https://github.com/lestrrat-3d/decad) for solids — and checks what comes out. A constraint
scheme that cannot reach DOF 0, or a boolean that leaves a body in two lumps, fails here in seconds
instead of in a Fusion session.

Two packages run every proof. `proofkit` runs the sketch steps and `proofkit3d` the solid steps.
Neither one holds gear geometry. Each creates the world, calls a step function, and judges what the
step built, so the same harness runs a cycloidal disc and an involute tooth. Tooth math that several
gears share lives in `involute/`, which the gear packages that need it import.

## What the module holds

| Path | What it is |
|---|---|
| `proofkit/` | The sketch harness: case tables, the runners, and the constraint gate. |
| `proofkit3d/` | The solid harness: the same runner shape over a `decad.Document`, plus the solid gates. |
| `spurgear/`, `helicalgear/`, `herringbonegear/`, `bevelgear/`, `cycloidal/` | One package per gear: a proof function per `[GO]` step of `spec/<gear>/steps.md`, and whatever geometry or picture tests that gear keeps beside them. |
| `involute/` | The involute tooth math the spur family shares, and that the bevel proof draws its virtual spur tooth from. |
| `render/`, `cmd/genexamples/` | The README's example images, meshed from the proved geometry. |
| `run.sh`, `run_test.sh` | The suite runner, and the runner's own tests. |

## The shape of one proof

A gear proof is three things: a table of parameter cases, one build function per step, and a
generated `Test` that hands the two to a runner. `proof/spurgear/zz_registrations_test.go` is that
third thing:

```go
func TestGearProfileSketch(t *testing.T) {
	proofkit.RunWithExpectedFailures(t, profileCases, stepGearProfileSketch, profileFailureCases)
}

func TestExtrudeTooth(t *testing.T) {
	proofkit3d.RunSolid(t, solidCases, stepExtrudeTooth, assertExtrudeTooth)
}
```

A case is a name and a `map[string]float64` of parameters. The name becomes the subtest name, so a
failure names the case verbatim:

```go
var profileCases = []proofkit.Case{
	{Name: "M1_N17_flat_default", Params: params(1, 17, 20, 0, 15)},
	{Name: "M3_N15_minus60", Params: params(3, 15, 20, -60, 15)},
}
```

The table decides what the proof covers. Proving one gear proves nothing about the next one, so a
table sweeps the regime the spec claims the scheme holds across — several module and tooth-number
pairs, both signs of every angle, the low end of any count.

A build function builds one case. `proofkit.Build` receives a fresh `*sketch.Sketch`;
`proofkit3d.Build` receives a fresh `*decad.Document` and returns the bodies it made. Two calls
belong in every build:

- `proofkit.Step(t, format, args...)` records which part of the build is running. The line is
  printed only when the case fails or under `-v`, so calling it on every step costs nothing.
- `proofkit.Unmodelled(t, format, args...)` skips a case, naming what the proof cannot represent.
  Returning early instead reports a pass for a case nobody proved. `proofkit3d.Unmodelled` does the
  same for solid cases.

Every runner fails a table with no cases, fails a sketch case that authored no points and no
entities, and fails the whole proof when no case completes — a table whose every case skips proves
nothing and would otherwise report success.

## What `proofkit` checks

`RequireSound` solves the sketch, verifies it with the ambiguity probe switched on
(`s.Verify(ctx, sketch.WithProbe())`), and reports every condition of
`sketch.VerificationReport.Check` the sketch fails. It runs after the build on every case of `Run`
and `RunParallel`. The engine's verdict asks for more than DOF 0:

- the solver converged and the system is solvable;
- the status is `FullyConstrained`;
- no conflicting and no redundant constraints;
- no broken, foreign or stale references, and no non-finite coordinate, dimension or constraint;
- every detected region is a valid profile, and every parameter expression evaluates;
- the constraint system is not near-singular, by `Check`'s own conditioning threshold;
- the probe found one configuration, not several.

Nothing is waived. A sketch can reach DOF 0 and still admit a mirrored or half-turned solution that
the seed happens to avoid, which is how a tooth gets built on the wrong branch, so ambiguity fails
like any other condition. A gear whose scheme is ambiguous by design has to say so at its own call
site and give the reason.

`Check` returns every failed condition rather than the first, and `proofkit` calls `t.Error` on each
one, then logs the specifics behind them: which constraints conflict and what they fight, which
constraints are redundant, which points are still free, how many detected regions are not
extrudable, and how many discrete configurations satisfy the constraints. A failure reports what the
solver found and does not classify the cause. Whether the geometry or the order of operations is
wrong is a judgement for whoever reads it.

### Proving that something fails

A negative control is a case that must fail, in one named way. `RunWithExpectedFailures` runs the
positive table first, then each declared failure in a fresh sketch:

```go
var profileFailureCases = []proofkit.ExpectedFailureCase{{
	Case:     proofkit.Case{Name: "M1_N43_flat_tooth_top_arc_centre_free", Params: failureParams()},
	Expected: proofkit.ExpectedFailure{Status: sketch.Underconstrained, DOF: 2, Reason: sketch.ErrNotFullyConstrained},
}}
```

The case still has to solve and converge, and its report has to be clean apart from the declared
failure: the status and DOF have to match, no other condition may fail, and `Check` has to return
exactly one reason, matched with `errors.Is` so the engine's diagnostic text stays useful. A case
that fails for a second reason as well fails the proof. The positive table stays mandatory, and a
call with no failure cases is an error.

## What `proofkit3d` checks

A solid step registers a build, a gate and an assertion. The build returns bodies, the gate applies
the verdict every solid proof shares, and the assertion then measures whatever that step claims —
volume, a bounding box, a face count, a centroid.

Two gates ship. `RequireSound` demands `decad`'s complete verdict (`report.Trustworthy()`).
`RequireSolid` accepts one documented diagnostic: a bounded area or centroid reading from a faceted
boolean can land beyond the default tolerance while the solid, its volume and its topology are still
proved. Any other diagnostic code, and any other reading behind that code, fails. `RequireSolid`
then reads each body's record out of that one report and requires it to be solid, watertight and
manifold, not self-intersecting, and one lump with no voids.

`Run` uses `RequireSound`, `RunSolid` uses `RequireSolid`, and `RunWithGate` takes a gate of your
own. Every gear step registered today uses `RunSolid` or `RunSolidParallel`.

`BodyReport` verifies the document and returns one body's record, for an assertion that needs a
reading the gate does not take. It verifies on every call, so a caller may build or consume bodies
between two calls and still read the model as it stands.

## Running cases in parallel

`RunParallel`, `RunSolidParallel` and `RunWithGateParallel` prove the same thing in the same way as
their serial counterparts — same build, same gate, same completion rule, same failure and skip
behaviour. Only the scheduling differs. When `bevelgear` was moved to them, the package took 47.66 s
at `-parallel 4` against 97.65 s serial and the whole suite 54.77 s against 100.63 s
([measurements](../docs/proof-optimization/bevel-case-parallelism-result.md)). `-parallel` bounds
how many cases run at once and defaults to `GOMAXPROCS`.

Two things follow from opting in.

Every case's name gains one level, `TestX/cases/name` rather than `TestX/name`, because a parallel
case cannot be a direct child of the proof's own test: Go releases a parallel subtest only after its
parent function returns, so cases started directly by the runner would still be waiting when the
runner reached its completion check. The `cases` group is an ordinary synchronous subtest, and its
`t.Run` does not return until every child has finished.

Per case, the runner gives out a fresh sketch or document and its own copy of `Params`. Nothing else
is per case, which is why opting in is decided one step at a time rather than package-wide.
`bevelgear`'s `stepCircularPattern` measures its seed solid, stores the readings in package-level
vars and retires the seed, and its assertion reads them back; two cases at once overwrite each
other's readings and the proof reports a wrong verdict instead of failing loudly. That step keeps
`RunSolid`. Audit a step for state carried between build and assert before moving it to a parallel
runner.

## Registrations are generated, not written

Nothing in a registration is a decision. It is a function of four things — the run method, the case
table, the step function, and whatever else that method takes — and all four are written in the step
list, one annotation per `[GO]` step:

```markdown
<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->
```

`.claude/skills/generate-gear/scaffold_proof.py` turns those annotations into
`proof/<gear>/zz_registrations_test.go`, in step-list order, deterministically. The package and
method it accepts, and the argument count each one takes, are derived by
`.claude/skills/generate-gear/check_compile.py` from the harness sources themselves rather than
copied into Python, so adding a runner here is enough to make it available to a step list, and
`proofkit.RunSolid` — a pairing neither package declares — is reported rather than emitted.

So do not hand-edit `zz_registrations_test.go`; change the annotation and rerun the scaffold. Do not
hand-edit `proof/<gear>/stage-manifest.json` either: it lists the `.go` files the pipeline generates
in that directory, and `stage.py` rewrites it on every successful placement. A `.go` file the
manifest does not name — `bevelgear/render_test.go` and `cycloidal/render_test.go` are the two
today — is left alone by the pipeline.

## Running the proofs

`proof/run.sh` with no arguments runs the whole suite, which is what CI runs and what a handoff
means. It verifies the engine checkouts before it runs anything: the revisions are pinned in
`proof/go.mod`, in the pseudo-version Go records for a module with no tags, and that is the only
place they are written. Point `SKETCH_DIR` and `DECAD_DIR` at checkouts of those commits — a
detached `git worktree` of each is the cheapest way — or the run refuses rather than quietly
proceeding against whatever sits beside the repository. `PROOF_VERIFY_REVISIONS=0` runs anyway, and
a green run with it set proves nothing about the pinned engine; `CLAUDE.md` covers when that is the
right thing to do.

For a focused local run, `--package ./<dir>` may repeat, and every token after `--` goes to
`go test`:

```sh
proof/run.sh --package ./bevelgear -- -run '^TestGearProfiles$' -count=1
```

A bad option fails before any setup output. A `-run` selector that matches no test fails too,
because `go test` otherwise exits 0 when it matches nothing, which hides a misspelled proof name.
Selection is a way to wait less while working on one gear, not a way to be told the suite passed.

## What the proofs cannot reach

A proof checks the modelling workflow, not Fusion. Sketch text is the standing example: this engine
has no sketch text, and in Fusion a text object carries its own position along the curve and is
never pinned, so a labelled sketch never reports `isFullyConstrained` even when its geometry is
completely determined (`[PB-TEXT-HOLDS-DOF]`). The proof cannot reproduce that, and the gear cannot
gate on `isFullyConstrained` in Fusion either. Where a proof substitutes geometry the engine accepts
for geometry the spec names — a chorded flank for a fitted spline, one Combine-Join for a chain of
them — the substitution is written next to the builder that makes it, with what it costs.

Loading a gear into Fusion is still the only check that sees the real thing. When Fusion gives a
verdict, `CLAUDE.md` says where the finding goes and requires one more question of it: could the
proof have caught this? If it could, the case goes in before anything else, so the next run fails in
seconds instead of in the GUI. If it could not, the reason is written into the proof file, beside
the thing it cannot reach.
