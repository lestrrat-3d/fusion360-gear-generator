# T7. Add small, executable proof examples

## Assignment

The implementer gives future prose drafters in `fusion360-gear-generator` generic examples of supported
proof construction. T0 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
These examples explain harness mechanics; they do not repair or replace any gear's generated proof.

## Allowed edits and required reads

Allowed new files are `proof/examples/proofkit3d_construction_example_test.go` and
`proof/examples/CONSTRUCTION.md`. The compile prompt and its rendering tests may be updated to require
that document. Existing harness functions, gear proofs, engine code, and `proof/go.mod` are read-only.
The implementer reads the Go pre-read document before writing Go.

Repository references are `proofkit3d.Run`, `RequireSound`, `RequireSolid`, `BodyReport`, and the
`separatedBlocksIn` helper in `proofkit3d_test.go`.
The dispatcher supplies pinned engine checkouts for these additional read-only references:

| Engine file | Named reference |
|---|---|
| sketch `profiles.go` | `Profile.Outer`, `Profile.Holes`, `Profile.Area`, `Profile.Valid` |
| decad `examples/decad_boolean_example_test.go` | `Example_decad_cut` |
| decad `capblend_test.go` | `capLoopEdges`, `circleProfile`, `TestCapBlendChamferCircularRim` |
| decad `prism_boolean.go` | `prismMaxArrangementSegments` |

## Shared construction rules

The test file uses package `examples_test` in the proof module's examples directory.
The existing block helper is a read-only construction reference; the new examples use public APIs.
Each positive regression test uses a fresh document and calls `RequireSound` or `RequireSolid` before its assertions.
The examples use the default verification tolerance. No case skips, custom permissive gate, or relaxed tolerance is allowed.
The unit for lengths is millimeters and the unit for volume is cubic millimeters.

Every measurement comparison checks `abs(measured - expected) <= reported_bound + 1e-8` in the stated units.
The small numerical allowance handles floating-point evaluation of the independent closed form; it does not
replace the harness's own bound check. A reported volume-bound failure remains a test failure.

## Exact examples

### Independent comparison bodies

`TestConstructionIndependentDocuments` creates one 20 by 20 by 10 block in each of two documents.
Each document passes its full gate and each block has volume 4000. The two bodies are never placed
in the same document. The test also creates two such blocks in one document at x = 0 and x = 100 and
checks each volume, proving the spatial-separation alternative without comparing pointers across documents.

### A profile with a bore

`TestConstructionAnnularProfile` creates concentric circles of radii 10 and 2, solves the sketch,
and selects the unique valid profile with exactly one hole. It does not select `Profiles()[0]` blindly.
The selected profile has area `96 * pi`; the inner disk has area `4 * pi`.
The profile is extruded by 8. The result must be one sound solid with volume `768 * pi`.
A separate solid disk of radius 10 and height 8, in another document, has volume `800 * pi`.
The removed volume is therefore `32 * pi`.

`CONSTRUCTION.md` states that this proves resulting geometry and material removal, not Fusion's
participant-body selection or a native Cut operation. It also distinguishes `Entities` from `Outer` fragments.

### An explicit bore cut

`TestConstructionExplicitCut` follows the pinned engine's public example using a 20 by 20 by 8 plate,
a radius-2 tool centered at (14, 6), and a tool height of 20 translated by z = -6.
The implementer uses `r3.Translation`, `Placed`, and `decad.CutContext` as that example does.
The independent expected volume is `3200 - 32 * pi`. The resulting document must pass `RequireSolid`.
The annular-profile example remains a documented alternative, not a fallback that makes this test pass.

### Separate cap chamfers

`TestConstructionSeparateCapChamfers` builds two independent radius-10, height-8 cylinders.
One receives a setback-0.5 chamfer on `decad.Edges(decad.CreatedBy(decad.CapStart(body)))`.
The other receives the same operation using `CapEnd`. Each result is gated in its own document.
With `R=10`, `H=8`, and `d=0.5`, each expected volume is:

```text
pi * R * R * (H - d) + pi * d / 3 * (R*R + R*(R-d) + (R-d)*(R-d))
```

Each volume is strictly below the original cylinder volume. The document explicitly says this does
not prove one body with both caps chamfered, and does not prove a chamfer applied after a gear's root fillets.

### A bounded evaluator refusal

`TestConstructionArrangementLimit` creates a 100 by 100 outer rectangle with 17 disjoint radius-1 holes.
Hole centers occupy the first 17 positions of a row-major 5-column grid starting at (10, 10) with spacing 15.
The selected profile has 17 holes and is extruded by 8. A radius-0.5 cutting tool centered at (90, 90)
extends from z = -1 to z = 9. Calling the same-plane analytic cut is expected to return the arrangement-budget error.
The test asserts a non-nil error containing `arranger segments` and `cap of`, not the complete diagnostic wording.
It does not call a success gate on a failed operation or increase the evaluator limit.

The documentation links the limit to its engine-owned constant and records the inspected revision.
It does not create a second authoritative constant in the gear repository.

## Executable usage examples

The same file includes five `Example_proofkit3d_` functions with suffixes `independent_documents`,
`annular_profile`, `explicit_cut`, `separate_cap_chamfers`, and `arrangement_limit`.
They share construction helpers with the regression tests so each recipe has one implementation.
Helpers accept `context.Context` and return constructed documents and bodies or an error; they do not accept `testing.T`.
Regression tests supply `t.Context()`. Example functions supply `context.Background()`.

Each positive Example verifies its documents with `doc.Verify`, requires a trustworthy report, and checks
the same independent volume formulas. It prints exactly `sound: true` and `volume matches: true` on separate lines.
The refusal Example prints exactly `budget refusal: true` after checking the expected error fragments.
Each Example ends with the corresponding exact `// Output:` block. Errors print a specific failure and return,
which fails the expected-output test. The accompanying Test functions retain the full harness gates and assertions.

## Ordered delivery and validation

1. The implementer adds and runs the positive examples first, with no prompt change.
2. The implementer adds the expected-refusal example and checks that it fails for the intended reason only.
3. `CONSTRUCTION.md` links each recipe to its executable Example and regression test and states its limits.
4. The compile prompt adds this generic document to its required input list.

```sh
bash proof/run.sh --package ./examples -- -vet=all -run '^(TestConstruction|Example_proofkit3d_)' -count=1 > .tmp/t7-examples.txt 2>&1
```

The [common validation](README.md#common-validation), including the complete proof suite, follows.
A positive fixture that fails its bound or topology checks, an unexpected budget result, or an unavailable
public API is an escalation with the smallest failing fixture. The implementer does not substitute another
geometry, change the expected values, or weaken an assertion. Completion requires all five named examples.
