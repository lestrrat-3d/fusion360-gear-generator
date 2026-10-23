# Gallery port pipeline

Status: design agreed in conversation on 2026-10-23, with a spur gear prototype built (see "Spur gear prototype").
The prototype has not been loaded in Fusion. Sections marked "Proposed" were not decided by the user and need
confirmation before they are treated as settled. The user accepted the fragment store and the regenerate-and-diff
drift check.

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

## Spur gear prototype

The prototype lives in `port/`, a Go module whose `go.mod` pins the gallery revision.
`go build -C port -o ../.tmp/gearport ./cmd/gearport` builds the command. From the repository root:

| Command | Effect |
| --- | --- |
| `gearport gen spurgear` | Writes `lib/geargen/spurgear.py`. |
| `gearport check spurgear` | Exits non-zero when that file differs from a fresh generation. |
| `gearport accept spurgear` | Records the digest of the gallery code the fragment was reviewed against. |

Packages:

| Package | Role |
| --- | --- |
| `port/gallery` | Parses the pinned gallery spur source: `Params` fields and defaults, the parameter table, the decad operations `Build` performs, and a digest of named functions. |
| `port/fusionexpr` | Rewrites a gallery parameter expression into a Fusion expression. It refuses a function it has no rule for. |
| `port/gotopy` | Translates the pure numeric helpers (`rotate`, `involutePoint`, `flankBeta`, `sampleFlanks`) to Python. A test runs the Python and the Go on the same inputs and requires the flank points to agree to 1e-9. |
| `port/spur` | The mapping from gallery names to Fusion names and dialog inputs, the stored fragment `fragments/spurgear.py.frag`, and the generator that fills the fragment's marker lines. |

What comes from the gallery source: the dialog inputs and their defaults, the registered Fusion parameters and their
expressions, and the involute sampling code. What stays in the fragment: the dialog selections, the anchor and
sketch construction, the extrude, pattern, join, fillet, bore and chamfer calls, and the profile finding. The first
fragment is the existing add-in with the generated parts replaced by marker lines.

The generator stops, with a message naming the cause, in three cases:

- The gallery parameter table gains or loses an entry the mapping does not list.
- The decad operations `Build` performs differ from `ExpectedOps` in `port/spur/mapping.go`.
- The code of the functions in `ReviewedFuncs` changes. A person reviews the fragment, then runs `gearport accept`.

### Surface kept for the other gears

`helicalgear.py`, `herringbonegear.py` and `bevelgear.py` import classes and constants from `spurgear.py`, and the
helical gear overrides `filletHelixFactorExpression`. The generated file keeps the four classes
(`SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`, `SpurGearGenerator`,
`SpurGearInvoluteToothDesignGenerator`), the `PARAM_` and `INPUT_ID_` names they use, and the fillet helix hook.
`port/spur/generate_test.go` checks that the classes exist and that no `PARAM_` or `INPUT_ID_` name is used without
a definition. The helical, herringbone and bevel gates pass against the generated file.

### Differences from the previous add-in

- The pitch, base, root and tip diameters, the tooth space angle and arc, and the fillet clearance parameters are
  gone. The gallery table has radii only, and no file outside `spurgear.py` read the removed names except as keys
  that `spurproxy.py` serves.
- Two parameters are new: `InvoluteAngle`, and the fillet radius now follows the gallery formula
  `0.9 * root_radius * (pi / teeth - 2 * involute_angle) / 2`.
- The involute sampling code is the gallery's, not a hand-written copy.
- The embedded-flank tooth keeps the Fusion construction (flanks from the base circle, trimmed by the root circle).
  The gallery closes its embedded tooth with an arc just inside the root circle. Both give the same joined gear.

### Fusion findings

First load, Fusion 2705.1.25, 2026-10-23: `FilletRadius` was rejected with "Invalid expression". A probe of about
twenty expression variants (`diag_params.py`, one run) showed:

- Through the API, lowercase `pi` parses as a unit, not a constant. It evaluates to 0 in a unitless parameter and is
  rejected next to a length, which made `InvoluteAngle` silently 0 and `FilletRadius` invalid. Capital `PI` is the
  constant, so the translator writes `PI` (`port/fusionexpr`, with a test).
- Dividing an angle by `1 deg` gives a plain number, so the `deg()` rule holds.

Sketch-only mode in Fusion 2705.1.25: on the first run the Tools and Gear Profile sketches showed as not fully
constrained in the browser, and each showed fully constrained after the sketch was opened. A later run showed both
fully constrained without any action. The proof reads both as fully constrained, probes that build sketches the way
the generators do (in the root and in a new component) showed no stale icon at all, and the playbook records that
the icon and the property can disagree and that a labelled sketch reads unstably (`PB-SETTLE-DISPLAY`,
`PB-TEXT-HOLDS-DOF`). The first reading was not reproduced, and nothing in the port changed the sketch calls, so no
code changed.

Loaded in Fusion 2705.1.25 on 2026-10-23 after the `PI` change, each without error: the default gear, an embedded
gear (12 teeth), a bore, a cap chamfer, and the default helical gear (which reuses the spur generator and its
fillet helix hook). Not checked: sketch-only mode, the herringbone and bevel gears with the new file, and numeric
agreement of the resulting body with the gallery body.

### CI

The old gates no longer run on `spurgear`: the contract manifest lists the removed constants, and the API-call
check does not know `collections.namedtuple`. A new `port` job runs `go test ./...` in `port/` and
`gearport check spurgear`. `WorkflowGateWiringTest` skips a gear whose file starts with the generated header, and
the exact-values drift test skips the module comparison for the same reason.

## Helical gear port

`gearport gen helicalgear` writes `lib/geargen/helicalgear.py` from the pinned `models/helicalgear`. The gallery
helical gear is the spur gear plus `HelixAngle` and a loft between two twisted sections, and the add-in already
builds on the generated spur add-in. So the port generates only what the helical source defines: the `HelixAngle`
dialog input, its default and the two name constants. The rest is the stored fragment
`port/helical/fragments/helicalgear.py.frag`, which is the previous add-in with those lines replaced by markers.
The generated file equals the previous one except for the header.

The generator stops when the helical build performs operations other than `Loft`, `Extrude`, `Union` and `Fillet`,
when `Params` gains a field beyond `HelixAngle`, or when the code of the functions in `ReviewedFuncs`
(`port/helical/mapping.go`) changes. The root fillet radius, which the gallery scales by `cos(HelixAngle)`, stays
in the spur parameter through `filletHelixFactorExpression`.

Loaded in Fusion on 2026-10-23 from the regenerated file (build id `a37e99a9aedb`, confirmed in the Fusion log), the
default helical gear built without error. Not checked: a bore, a cap chamfer, an embedded gear, sketch-only mode, and
numeric agreement of the body with the gallery body.

### Build id

Every generated add-in carries `GEAR_BUILD_ID`, the first 12 hex digits of the SHA-256 of the file without its three
stamp lines, and logs `<gear> build <id>` when Fusion imports the module. `gearport id <gear>` prints the id the
file in the working tree carries. A match between the Fusion log and that output shows Fusion ran that file.

## Herringbone gear port

`gearport gen herringbonegear` writes `lib/geargen/herringbonegear.py`. The gallery herringbone `Params` is an alias
of the helical `Params`, so the gear has no input or parameter of its own, and the stored fragment
`port/herringbone/fragments/herringbonegear.py.frag` is the whole previous add-in. The generated file equals the
previous one except for the header and the build id lines.

The generator stops when `Params` stops aliasing `helicalgear.Params`, when the herringbone build performs operations
other than `Loft`, `Union` and `Union`, or when the code of `Build` or `BuildTooth` changes. The helical and spur
checks cover the pattern, join and finishing operations the gallery performs in those packages.

Loaded in Fusion on 2026-10-23 from the regenerated file (build id `484f3cd3b91a`), the default herringbone gear
built without error. Not checked: a bore, a cap chamfer, an embedded gear and sketch-only mode.

## Gallery pin 685d4e0

The pin moved from 872277b to 685d4e0. The gallery now builds spur, helical and herringbone through one
`spurgear.Variant` pipeline and reads dimensions from the document's parameter table. The geometry did not change,
so the fragments did not change. The reader follows methods of the model package and package functions passed as
values, once per method, so a build split into pipeline stages still lists its decad operations. The helical and
herringbone gears list only the operations in their own package, and the spur port checks the shared stages. The
`ReviewedFuncs` lists name the new functions. Only the header and build id of the three generated files changed.

## Bevel gear port

`gearport gen bevelgear` writes `lib/geargen/bevelgear.py` from the pinned gallery `models/bevelgear`. The add-in
builds the pair with Fusion calls that the stored fragment `port/bevel/fragments/bevelgear.py.frag` holds, and it
registers no Fusion user parameters, so the gallery's derived parameter table is not read. The port takes the dialog
inputs and their defaults from the gallery `Params` and `DefaultParams`: module, teeth, shaft angle, base heights,
bore, face width, tooth spacing, spiral angle and hand, cutter radius, toe extension and toe radii. The generated file
equals the previous one except for the header and build id lines.

The generator stops when a gallery `Params` field has no mapping, when the decad operations of `Build` differ from
`ExpectedOps` in `port/bevel/mapping.go`, or when the code of the functions in `ReviewedFuncs` changes. A test requires
the pressure angle (20 degrees) and the sample count (15) that `lib/geargen/spurproxy.py` fixes for the tooth to equal
the gallery defaults.

The pin is gallery 838801e (decad c664e84), the revision `decad-gallery-93` reported as final on 2026-10-24.
Review against 7679362: `Params`, `derive`, `defineParameters`, `prepareBuild`, `BuildPair` and `buildMember` did not
change. `buildSpiralMember` now always takes the serial route and decad's gear-shaped spiral route is deleted, so the
spiral part of the decad operation order in `ExpectedOps` is shorter. The add-in's Fusion construction does not follow
the gallery's spiral construction, so the fragment did not change.

Pin update to gallery 04ee58c (PR #84, 55 of 55 CI jobs green). The gallery now builds a spiral member for any module,
tooth counts and shaft angle (`validateSpiralBuild` refuses only samples other than 15 and a hand other than right or
left), so the gap that the previous pin left is closed. The 20-tooth member of a 40/20 pair builds but fails
`VerifyAll` at 0.1 mm, with the 14.5 degree pressure angle, 20 degree spiral angle and the lone cutter-radius-30 member
(the shallow-crossing mesh bound recorded in decad's `agent-docs/faceted-vertex-bounds-design.md`). `validateSpiralBuild`
is now in `ReviewedFuncs`, so a later tightening of that gate stops the port. The port drops no input.

The gallery builds every other option the add-in offers as of gallery PR #80 (7679362). Left-hand spiral, spiral angle 20
degrees and a 14.5 degree pressure angle build in the gallery with a weaker check than the others (a volume bound
below 15 percent instead of `VerifyAll` at 0.1 mm), per `decad-7e`'s report. The Fusion add-in does not depend on that
check. Not yet loaded in Fusion.

The reader now reads string and bool defaults, follows `Params` that alias another model's, and no longer fails on a
model whose `defineParameters` uses loops: the bevel table form (a loop over struct literals and a computed face width
expression) is not read, and `Model.TableErr` records why. A port that needs the table reports it.
