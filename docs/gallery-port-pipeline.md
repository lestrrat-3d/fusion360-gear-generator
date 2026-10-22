# Gallery port pipeline

Status: design, agreed in conversation on 2026-10-23. Nothing here is built yet. Sections marked "Proposed" were
not decided by the user and need confirmation before they are treated as settled. The user accepted the fragment
store and regenerate-and-diff drift check.

## Goal

Replace the first half of the gear generator pipeline, which runs from `spec/<gear>/instructions.md` through
`/compile-gear` to `steps.md` and the Go proofs in `proof/<gear>/`. Models are built and proved in the
[decad-gallery](https://github.com/lestrrat-3d/decad-gallery) repository instead, as Go code on top of
[decad](https://github.com/lestrrat-3d/decad) and [sketch](https://github.com/lestrrat-3d/sketch). This
repository then only translates a gallery model into the Fusion add-in Python under `lib/geargen/`.

## Current pipeline and target pipeline

| Stage | Today | Target |
| --- | --- | --- |
| Construction contract | `spec/<gear>/instructions.md` and `fusion.md` | `models/<gear>/instructions.md` in the gallery, plus Fusion rules kept here |
| Step list | `spec/<gear>/steps.md`, written by `/compile-gear` | `models/<gear>/steps.md` in the gallery |
| Geometry proof | `proof/<gear>/`, run by `proof/run.sh` | the gallery's `go test ./models/<gear>`; this repository does not repeat it |
| Fusion code | `/emit-gear` writes `lib/geargen/<gear>.py` from the step list | a translator writes `lib/geargen/<gear>.py` from the gallery's Go source |
| Fusion check | the user loads the add-in by hand | unchanged |

## Decisions

The user made these decisions during the planning conversation.

1. The translator reads the gallery's generated Go code as its main input. The gallery and decad provide the geometry
   proof, so this repository does not prove geometry again.
2. The translator is a mix of a deterministic part and an LLM part. The Go-source reader is the deterministic part and
   is written in Go in this repository.
3. The ported add-in stays parametric, with Fusion user parameters and a feature timeline that update when the user
   edits a value.
4. The command dialog and the sketch-only mode of today's add-in carry over unchanged. They have no source in the
   gallery, so they live here as a fixed template.
5. The translator writes `lib/geargen/<gear>.py` directly, replacing the current generated file.
6. Fusion-specific knowledge stays in this repository. Where it can be expressed either as prose or as code, the user
   prefers code.
7. The ported gear must be close enough to today's gear in its end result. It does not have to match
   `spec/<gear>/` step for step.
8. The Fusion load check is a manual step, because only the user can run Fusion. Findings still go to `PLAYBOOK.md` or
   the gear's `fusion.md`, as `CLAUDE.md` requires.
9. The gallery revision is pinned in a `go.mod` pseudo-version, like the decad and sketch pins in `proof/go.mod`.
10. The first milestone is the spur gear alone. Helical and herringbone, which the gallery already has, wait until the
    spur gear loads in Fusion.
11. The existing `spec/`, `proof/`, `steps.md`, `/compile-gear` and `/emit-gear` are removed only after every gear is
    ported.

## What the translator reads and writes

Input is a pinned revision of the gallery. For the spur gear that is `models/spurgear/*.go`, which holds
`Params`, `DefaultParams`, `validate`, `derive`, the profile builders and `Build`.

Output is `lib/geargen/spurgear.py`. The generated file keeps the current add-in's public surface: the command inputs,
the generation context, and the registered parameter names.

### Deterministic part

The Go-source reader parses the gallery package and extracts:

- the `Params` fields and their defaults, which become the dialog inputs;
- the derived values and their expressions, which become Fusion user parameters;
- the ordered list of sketches, constraints and feature calls in `Build`.

It maps each Decad call to a Fusion call:

| Decad | Fusion |
| --- | --- |
| `sketch.CreateCircle`, `CreateLine`, `CreateArc`, `CreateFitSpline` | `sketchCurves` additions |
| `sketch.NewRadius`, `NewDiameter` and the other dimensions | `sketchDimensions` and `geometricConstraints` |
| `decad.Extrude` | `extrudeFeatures` |
| `decad.PatternCopies` with `Union` | circular pattern, then a join `combineFeatures` |
| `decad.Union`, `decad.Cut` | `combineFeatures` |
| `decad.Fillet`, `decad.Chamfer` | `filletFeatures`, `chamferFeatures` |

### LLM part

Some Decad constructs have no one-to-one Fusion form. Selectors such as `ParallelTo`, `EndpointAt` and
`OuterLoopOf(CapStart)` become BRep face and edge searches. Profile finding after a sketch, and the coincidence order
that avoids `VCS_SKETCH_OVER_CONSTRAINTS`, come from the rules in `PLAYBOOK.md` and `spec/spurgear/fusion.md`.

The LLM output for each such construct is stored as a reviewable fragment file beside the translator, so a rerun
with unchanged input is byte-identical and the pipeline stays repeatable.

1. The reader splits the gallery Go into constructs: a sketch, a constraint group, a feature call.
2. The mapping table turns each mapped construct into Python.
3. For an unmapped construct the translator looks up a fragment by key. The key is a hash of the construct's
   normalized Go text, the names of the variables the fragment reads and writes, the text of the Fusion rules and
   helpers its prompt cites, and the prompt version.
4. On a miss the translator writes a prompt file and stops. An LLM session produces the fragment, a human reviews it
   in the pull request, and a second run finishes.
5. Changed input invalidates only the fragments whose key changed, so the Fusion recheck covers those.

The fragment store is transitional. After Fusion accepts a fragment, and the construct appears in a second gear, the
fragment is promoted to a rule in the mapping table, which is code.

### Fusion knowledge as code

Proposed: the rules in `PLAYBOOK.md` and each gear's `fusion.md` move into code where they can. Helper functions in
`lib/geargen/` (for example `solids.py` and `utilities.py`) hold the Fusion workarounds, and the translator's mapping
table calls them. A rule that cannot be expressed as code stays as a `[PB-…]` or `[<GEAR>-F-…]` anchor, and the
translator or its tests cite the anchor.

## Parametric requirement and its dependency on decad

The gallery spur code today computes `derive(p)` and every dimension as plain floats, so its Go source carries no
parameter names. A parametric add-in needs Fusion parameter expressions such as a root radius written in terms of
module and tooth count.

The sketch library already has a parameter engine in `sketch/param`: a `param.Table` of named parameters with
expressions, forward references and cycle errors, and `Sketch.Bind` to tie a dimension to an expression. Decad's
feature calls (`Extrude`, `Fillet`, `Chamfer`, pattern count) take plain `units.Value` numbers and cannot carry an
expression.

Agreed plan:

1. Decad uses `sketch/param` directly now. Extracting `param` into its own module is a later step, once it is worth a
   third consumer.
2. Decad has no feature history: the Go builder function is the feature tree. So an edit means calling
   `table.Set(...)` and running the builder again. Fusion keeps its own timeline, so decad needs no replay engine.
3. Decad gains a `decad.WithParams(*param.Table)` document option, `doc.Param(name)` and `doc.Expr(expr)`, which
   return the existing `units.Value` after evaluating through the table, and `doc.ParamInt(name)` for counts. The
   existing call signatures do not change. A read-only `Document.Bindings()` ledger lists each use with its feature,
   field, expression, evaluated value and dependencies. The decad session records this design in
   `agent-docs/param-binding-design.md` in the decad repository and ships it with the implementation.
4. After decad is ready, the gallery spur rewrites `derive`, its dimensions and its feature values as
   `param.Table` entries.
5. The translator reads those names and expressions from the Go source. Parameter names and expressions must be
   string literals in the gallery source (`table.Set("root_radius", "...")` and `doc.Param("root_radius")`), not
   built by concatenation or loops. The `Bindings()` ledger from a real run checks the reader's output.

Sampled involute points depend on a sample count and loops, so they may remain computed values that read the table.
The spur port does not depend on settling that detail first.

## Drift detection and pinning

The gallery will keep changing in small ways. The gallery revision is pinned in a `go.mod` pseudo-version in this
repository. There is no record file of hashes. CI detects drift by regenerating:

1. CI checks out the pinned gallery revision, as the proofs already do from `proof/go.mod`.
2. CI runs the translator with the committed fragments.
3. CI fails if the output differs from the committed `lib/geargen/<gear>.py`, or if a fragment is missing.

This catches a pin bump with no regeneration, a hand edit of the generated file, a stale fragment, and a translator
change. It depends on the translator being deterministic once the fragments exist. A header comment in the generated
file records the gallery revision and translator version for human readers; the check does not read it.

Bumping the pin is its own change, and the Fusion check decides whether the bump is accepted.

## Spur gear milestone

Order of work:

1. Gallery changes the spur tooth from a loft between two identical sections to one `decad.Extrude` of the tooth
   section. The gallery already builds one tooth, copies it with `decad.PatternCopies`, and joins it with one
   `decad.Union` (gallery PRs #68 and #69). The extrude is the remaining difference from today's Fusion construction.
2. Decad accepts expression-carrying feature values, and the gallery spur adopts them.
3. This repository builds the Go-source reader, the mapping table and the drift check against the pinned gallery spur.
4. The translator writes `lib/geargen/spurgear.py`, and the user loads it in Fusion.
5. Fusion findings go into the supplement and the translator, and the spur port is rerun until it loads.

Steps 1 and 2 are owned by the gallery and decad sessions. Step 3 can start before they finish, because the reader
works on the gallery code as it stands.

Known gallery limits that carry into the port: an oblique placement frame joins but its root fillet refuses, a fillet
radius larger than the connector length refuses, and copies whose spline edges cross each other or an arc refuse.

## Open points for later verification

- Acceptance check. The user decided that decad provides the geometry proof and that the Fusion load is manual. No one
  has decided whether to also compare the Fusion body with the gallery body (for example by volume and bounds) to catch
  translation mistakes before the Fusion load.
- Where the Go translator lives in this repository. Proposed: a new module beside `proof/`, with its own `go.mod` that
  pins the gallery.
- How long the old pipeline coexists. Decision 11 removes it after every gear is ported. Until then both pipelines run,
  and the gears without a gallery model keep using `/compile-gear` and `/emit-gear`.
- Whether the gallery spur and today's `lib/geargen/spurgear.py` differ in behavior the user cares about, such as
  input names or parameter names. The user accepted "close enough", so differences are tolerated unless the add-in's
  dialog or saved designs depend on them.

## Verification of this design

Check each of these against the built pipeline:

- No file under `proof/` or `spec/` is read when a gear is ported from the gallery.
- `lib/geargen/spurgear.py` is written only by the translator. A hand edit makes the drift check fail.
- The ported add-in registers Fusion user parameters, and editing one updates the body.
- The gallery pin in `go.mod` equals the revision recorded for the port.
- A Fusion load of the ported spur gear succeeds for the default parameters and for the bore and chamfer cases.
