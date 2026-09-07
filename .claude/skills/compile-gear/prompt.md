Compile the specification for `{{gear}}` into a step list and a runnable proof. Work in the repo
worktree. Write the step list to `.tmp/{{gear}}.steps.md`, and the proof, as one or more Go files,
to `.tmp/{{gear}}-proof/`.

**Read, in full, only these:**

- `spec/{{gear}}/instructions.md`, which is required.
- `spec/{{gear}}/fusion.md`, if it exists.
- `spec/{{gear}}/contract.json`, if it exists.
- Every Markdown document either prose source references by name. In the bundle condition, each
  resolved repository-relative path is a manifest file entry.
- `.claude/skills/generate-gear/PLAYBOOK.md`.
- `docs/prose-pipeline-handoffs/formats.md` for step metadata.
- `proof/examples/CONSTRUCTION.md` for generic executable solid-construction recipes and their limits.
- `proof/examples/proofkit3d_construction_example_test.go` for the tested implementations of those recipes.
- `proof/proofkit/`, meaning every Go file beneath the sketch harness directory.
- `proof/proofkit3d/`, meaning every Go file beneath the solid harness directory.
- `proof/involute/involute.go` for the involute tooth math the spur family shares, so you import it
  rather than deriving it again.

**Bundle consumption:** When the operator supplies an input bundle, this rendered prompt is the
`@rendered-prompt` entry and is delivered exactly once. Resolve every logical source path above
through `manifest.json`, then read all of that file's chunks in manifest order. Resolve a directory
to sorted manifest file entries beneath that prefix, then read every chunk for each entry; the
manifest contains file entries, not directory entries. Do not reopen a packed original source path.
The documented API-status and signature queries below remain available. Save their complete outputs
unchanged under the same policy in every compared condition. Report an undocumented dynamic input or
a different query policy instead of guessing a bundle exclusion.

**Do not read** `lib/geargen/{{gear}}.py`, any other gear's implementation, or a previous
`steps.md` or proof for this gear. If the spec is unclear, record it as a spec gap in your report
and make your best attempt. Never resolve it by looking at existing output.

**A step is one entry in the Fusion timeline.** Drawing a whole sketch is one step, however much
geometry goes into it. So is each extrude, chamfer, pattern, combine, fillet. Write the step list
at that size, and keep the detail inside the step it belongs to.

**The step list opens with one sentence naming the proof files**, above the provenance heading and
before any other section, written as the committed paths `proof/{{gear}}/<file>.go` even though you
are writing the files themselves to `.tmp/{{gear}}-proof/`, since the step list ships next to the
placed proof, not next to your scratch copy. A gate reads only the text above the provenance
heading for those paths and requires each one to exist and be committed, so a sentence written
below that heading, or one naming bare file names, leaves the gate with nothing to check.
Include `proof/{{gear}}/zz_registrations_test.go`, the generated registration file, among the
paths.

**Immediately after the proof-file sentence, write `<!-- step-metadata: 2 -->` on its own line.**
It must precede `## Provenance`, because provenance stamping replaces that whole section. Write
the marker exactly once and before the first step.

**Write the `## Provenance` heading and leave its section empty.** The provenance table and the
complete `## Compilation contract` section are generated from the source files after you finish,
by `.claude/skills/generate-gear/gen_provenance.py`. Do not write the contract section yourself.
Never run
`git hash-object` and never type a hash: a hand-copied hash is a defect the gate can only report
as drift. Put the heading below the sentence naming the proof files and above the first step
heading, since a gate reads the text above it for those paths and the generator inserts the
contract after provenance and before that first step.

**Each step carries** a heading of the form `## <id> `[GO]` <title>` or with `[PROSE]`, the
instructions themselves, one version-2 `step-meta` JSON comment naming the spec files and line
ranges you compiled it from, and every Fusion API call it requires written inside a code span.
Write the comment before any rendered citation line, with an opening line exactly
`<!-- step-meta`, a closing line exactly `-->`, and the JSON produced by
`json.dumps(payload, sort_keys=True, ensure_ascii=False, indent=2)`.
Read `docs/prose-pipeline-handoffs/formats.md#version-2` for the exact fields and role rules;
`.claude/skills/generate-gear/step_metadata.py` owns accepted syntax.
Do not write a `**From:**` line. The orchestrator renders it mechanically after the
draft. A `[GO]` step also names the proof function that realises it and carries the `proof-run`
annotation described below.

**Cite by anchor every playbook rule a step relies on.** Write the anchor in the step, as
`[PB-SKETCH-FIRST]`, wherever the step's instructions only make sense because of a rule the
playbook states. You are the only stage that reads `PLAYBOOK.md`: `/emit-gear` is handed an
extract built from your citations alone, so a rule you leaned on without naming is a rule the
transcriber never sees. Cite a per-gear `fusion.md` anchor the same way; those resolve to nothing
in the extract, and it says so, but the citation still records where the instruction came from.
A gate refuses a step list that cites no playbook anchor at all.

**Carry into the step every value the emit stage cannot look up.** `/emit-gear` reads your step
list, the proof and a playbook extract, and nothing else — it is forbidden to open the prose spec.
So a step that says "verbatim from the input table", "as the spec gives", or "per the table above"
with no table following resolves to nothing, and the transcriber fills the gap with a plausible
invention. Measured: a step list that pointed at the dialog's input table instead of reproducing it
shipped nine of seventeen input ids wrong, because `boreEnable` reads more naturally as
`enableBore`. Reproduce the whole table. The same holds for any id, label, tooltip, constant name
and its value, unit string, tolerance, magic number or fixed string a step depends on. Citation
metadata records where a step came from; it is not a way for the reader to go and look.

**Name the exact entity a call is made against.** Where the spec pins the operand, write it; never
reduce it to "again", "the same as above", or "likewise". Two operands that describe the same
geometry are not interchangeable as constraint arguments — measured, a step that said "collinear
again" was transcribed against the shaft axis rather than the line the spec named, and Fusion
refused the sketch with `VCS_SKETCH_OVER_CONSTRAINTS`.

**Declare every call-shaped inline span using the version-2 format owner above.**
The following checklist is non-authoritative; the format owner wins over this summary.

- Give each `(span, name, receiver)` exactly one declaration, including calls in step titles.
- Use `required` for execution obligations and preserve any stated condition.
- Give required API calls qualified owners; owner-null required locals must belong to the existing
  Python/math, framework, or contract name sets used by `check_compile.py`.
- Use `inherited` only for an actual definition in the shared framework, never a contract-only method.
- Give `example`, `forbidden`, `inherited`, and `prose` entries reasons under the format owner's rules.
- Use `prose` only when the whole span fails Python expression parsing.
- Keep call-shaped spans inside their relevant steps and never write global ignore directives.
- Do not change an existing required call's role because an existing module omits it.
  Reclassification requires source review. A missing implementation call remains emission work.
  Never guess production roles for automatic migration.

Declarations do not prove that every prose requirement was captured. Proof and source review still
check that boundary. A `forbidden` role does not replace source guards.

**Before naming any `adsk.*` call**, run
`python3 .claude/skills/generate-gear/query_api_status.py --owner <qualified-class> --member
<member>`. Its shared status is the support decision used by both validation gates: `documented`
is allowed, `unverified` is visible advisory evidence, `refuted` and `not_found` block, and
`unavailable` is a setup failure. A `not_found` result says the database has no declaration for
that receiver; it does not prove runtime absence. For signature and argument detail, ask the
`fusion:query-api` skill `show <Class>.<member>`. It resolves inherited declarations and gives the
member's signature and documentation. Write the call with the arguments that signature asks for.
Use `members <Class>` to discover the documented alternative when status blocks. If the spec
names a blocked call or passes an argument of the wrong type, say so in your report and do not
quietly correct it.

**The proof is a Go test** in package `{{gear}}_test`, spread over as many files as the split
needs, with one function per step. Every step function, 2D or 3D alike, is declared as a
function, `func step<Title>(...)`, matching what the step list names — a step bound to a
variable is not read.

**Write no Go `Test` function.** Registrations are generated, not drafted. In each `[GO]`
step's body, on its own line, write one annotation:

`<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->`

naming a run method a harness package declares — `proofkit.Run` for a sketch step;
`proofkit3d.Run`, `proofkit3d.RunSolid` or `proofkit3d.RunWithGate` for a solid step — then, in
order: the case-table variable, the step function, and the remaining arguments the method's own
signature declares after the build, in declaration order (`proofkit.Run` takes none,
`proofkit3d.Run` and `RunSolid` take the assertion, `RunWithGate` takes the gate then the
assertion). Each argument is a plain name — the gate argument may be qualified, like
`proofkit3d.RequireSolid` — and the case table is a package-level named variable, not a call
built in place. Each run takes a table of parameter cases, one subtest each. The scaffolder
turns these annotations into `proof/{{gear}}/zz_registrations_test.go`; never write that file,
and never register a step yourself — a hand-written registration collides with the generated
one or fails the check by name.

**Use a parallel entry point only where the spec for this gear asks for one.** Each harness also
declares parallel counterparts — `proofkit.RunParallel` beside `proofkit.Run`, and
`proofkit3d.RunSolidParallel` and `proofkit3d.RunWithGateParallel` beside `RunSolid` and
`RunWithGate` — which run a table's cases at the same time under a `cases` grouping subtest
rather than one after another. A counterpart takes the same arguments in the same order as the
serial method it sits beside. The serial method is the default: write a parallel one for a step
only where the spec says that step's cases may run together, and the serial one everywhere else,
including for every step the spec does not mention. A parallel run is sound only when nothing
outside a case carries state from that case's build to its assertion, so where a step needs such
a hand-off — a reading the build takes that the assertion cannot take again — hold it in a
package-level variable only for a step the spec keeps serial, and write beside that variable, in
the proof file, that the step is serial because of it.

**A proof file carries no build constraint.** Any `//go:build` header line, and every `+build`
spelling, is refused by name and line; write the header without one. The refusal says what to
write; write that.

**Name proof files so Go compiles them.** Go decides which files are in a package from their
names alone: a name starting with `_` or `.` is invisible to it, and a name whose trailing
`_`-separated words are a GOOS, a GOARCH, or a GOOS and a GOARCH — `steps_windows_test.go`,
`steps_arm64_test.go`, `steps_windows_amd64_test.go` — is compiled only on that platform. A
proof in such a file registers nothing, `go test` reports no test files rather than failing, and
the gate names the file and asks for a rename. Ordinary trailing words are unaffected, so
`geometry_test.go`, `sketches_test.go` and `solids_test.go` are all fine.

**Write proof files as UTF-8 Go can read.** Go refuses to compile a source file whose bytes it
cannot read, and a refused file registers nothing, so the gate refuses it too, naming the file,
the line and the column. Four byte patterns do it: a UTF-16 byte order mark, which is what a
file saved as UTF-16 opens with; any other bytes that are not UTF-8; a NUL byte anywhere; and a
byte order mark anywhere other than the very first character. One leading `EF BB BF` is the
exception Go strips, so a file that opens with a single mark is read normally and every line
below it keeps its number. Write the file as UTF-8, with none of those bytes in it.

**Separate tokens and spell names the way Go's scanner reads them.** Between two tokens Go skips
space, tab, carriage return and newline and nothing else, and a name is a Unicode letter or `_`
followed by letters, decimal digits or `_`. A vertical tab between `func` and a name, a
non-breaking space in the indentation, a superscript digit inside a step title: each makes the
file illegal where it sits, `go test` reports it as an illegal character and the package builds
nothing, so the gate reads no declaration on that line and names the step or the run that went
missing instead. The name a step claims in the step list is held to the same rule and is read
whole: with such a character written against it the claim is not a Go name at all, so it credits
nothing and the step is reported as naming no proof function. Unicode letters themselves are
ordinary — `stepPrüfung` is a name Go compiles and the gate reads.

**The case table reaches every branch, from every direction the spec offers.** A branch a step
takes needs a case on each side of it, and a branch the spec says is reachable in more than one
way needs a case for each way, because the ways differ in what they get wrong. Cover the ends of
every range the spec states for a parameter, negative values included wherever the spec says the
value is signed — a sign the scheme drops is still solvable at the positive value, so a table of
positive cases proves nothing about the sign. Where the spec names the regime the design must
hold across, that regime is the table.

**Assert what the spec pins.** Where the spec fixes a fact a later step selects or matches on — a
curve count a profile search takes as its key, an edge or face count, an extent, a volume — the
proof asserts that fact on the geometry it actually built, in the step that produces it. Restating
the number in a comment is not proving it. Assert it against the real construction rather than
against a simplified stand-in drawn for the purpose, since the stand-in is the thing whose
agreement is in question.

**A boundary a harness refuses is not permission to drop the step.** Substitute geometry the
harness does accept — chord a curve it will not trim, draw a split the engine will not perform —
and assert what the substitute still pins, saying in the proof what was substituted and what the
substitution costs. Only where no substitute survives the gate is the step `[PROSE]`, and then say
so **in the proof file**, next to the nearest thing the proof does build, with the reason it
cannot be reached. A limit recorded only in the step list is a limit the next reader of the proof
will not find.

**A sketch step** builds through `proofkit.Run`, whose build function is
`func(t testing.TB, s *sketch.Sketch, p map[string]float64)`. Model what Fusion does: use
`CreateReferencePoint` for anything projected in rather than fixing coordinates, mark solid and
construction geometry as Fusion would, and let profile detection split curves at crossings rather
than drawing boundary arcs by hand. Call `proofkit.Step` as you move between parts of a step, and
`proofkit.Unmodelled` for a case the proof cannot represent, never a silent return.

**A solid step** builds through `proofkit3d`, whose build function is
`func(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body` and returns
the bodies the step leaves behind. Every `proofkit3d` run also takes an assertion,
`func(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64)`, which
runs after the gate and checks the measurements the step is supposed to produce; it is required,
and a nil one fails the run. Name it in the step's `proof-run` annotation.
`proofkit3d.Unmodelled` is the 3D counterpart of
`proofkit.Unmodelled`, for a case `decad` cannot represent.

**The proof must pass with nothing waived.** `proofkit` gates a sketch on
`sketch.VerificationReport.Check`, which asks for more than DOF 0: no conflicting or redundant
constraint, no stale or broken reference geometry, valid profiles, a system that is not
near-singular, and no discrete ambiguity. A scheme that reaches DOF 0 but still allows a mirrored
or 180-degree-rotated answer fails there, and the fix is a constraint that carries a direction,
not a comment.

`proofkit3d.Run` gates a solid on `decad`'s own verification verdict: the document report has to
come back trustworthy, and the build has to return bodies rather than nothing or a nil.
`proofkit3d.RunSolid` reads the same report but tolerates exactly one kind of diagnostic, an area
or centroid reading a faceted boolean left outside the default tolerance, and adds the topology a
solid has to have: every body reports as solid, watertight, manifold and free of
self-intersection, with a single lump and no voids. `proofkit3d.RunWithGate` takes the gate as an
argument; do not pass a weaker one to get a build through.

**The two artifacts must describe the same build.** Every `[GO]` step names its proof function,
and every proof function is named by a step.

**Report:** what you produced, and every place the spec was unclear, incomplete, contradictory,
or wrong about the Fusion API. Those are the defects to fix. Do not smooth them over.
