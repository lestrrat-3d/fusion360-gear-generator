Compile the specification for `{{gear}}` into a step list and a runnable proof. Work in the repo
worktree. Write the step list to `.tmp/{{gear}}.steps.md`, and the proof, as one or more Go files,
to `.tmp/{{gear}}-proof/`.

**Read, in full, only these:** `spec/{{gear}}/instructions.md`, `spec/{{gear}}/fusion.md` if it
exists, every document those reference by name, `.claude/skills/generate-gear/PLAYBOOK.md`,
`proof/proofkit/` for the sketch harness API, `proof/proofkit3d/` for the solid harness API, and
`proof/involute/` for the involute tooth math the spur family shares, so you import it rather
than deriving it again.

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

**Write the `## Provenance` heading and leave its section empty.** The provenance table is
generated from the spec files after you finish, by
`.claude/skills/generate-gear/gen_provenance.py`, and written into that section. Never run
`git hash-object` and never type a hash: a hand-copied hash is a defect the gate can only report
as drift. Put the heading below the sentence naming the proof files and above the first step
heading, since a gate reads the text above it for those paths and the generator writes below it.

**Each step carries** a heading of the form `## <id> `[GO]` <title>` or with `[PROSE]`, the
instructions themselves, a `**From:**` line naming the spec files and line ranges you compiled it
from, and every Fusion API call it requires written inside a code span. A `[GO]` step also names
the proof function that realises it.

**A call span in a step is a call the module must make.** A later gate reads every call written
in a code span and requires the generated module to make it. A name a step mentions without
requiring it therefore has to be marked: a method the module defines for the framework to call, a
call named only to forbid it, and one of several alternatives the spec lets the implementation
choose between are all mentions, not requirements. Mark each with the exemption directive on its
own line in the step that mentions it, `<!-- check-step-calls: ignore nameOne nameTwo -->`, and
say in the step's prose why the mention is not a requirement.

**Before naming any `adsk.*` call**, ask the `fusion:query-api` skill about it. Two questions
carry most of the work: `members <Class>` lists everything a class offers, inherited members
included, each with the class that declares it, which is how you find out whether the class you
are calling on really has the member; and `show <Class>.<member>` gives one member's signature
and documentation. Write the call with the arguments that signature asks for. If the spec names
a call the API does not have, or passes an argument of the wrong type, say so in your report and
do not quietly correct it.

**The proof is a Go test** in package `{{gear}}_test`, spread over as many files as the split
needs, with one function per step. Every step function, 2D or 3D alike, is declared as a
function, `func step<Title>(...)`, matching what the step list names — a step bound to a
variable is not read — and is registered by the Go `Test` of the same title, in a shape that is
fixed and has no variants:

```go
func TestGearProfileSketch(t *testing.T) {
        proofkit.Run(t, profileCases, stepGearProfileSketch)
}
```

Three lines, headed exactly as above, with the testing package named in full: `go test` also
runs `Test_Foo`, `Test1x` and a header written against an aliased import of `testing`, and the
gate refuses all three by name rather than reading them. The header and the closing `}` each
start at column 1, which is where `gofmt` writes them. Go compiles an indented one, and the gate
refuses it anyway, because these three lines are the shape rather than ordinary code; the refusal
names the header or quotes the shape, so it is never silent. The step function is not part of
that shape and is read wherever it starts on its line. The `Test` function holds nothing else.
Use `proofkit3d.Run`, `proofkit3d.RunSolid` or `proofkit3d.RunWithGate` in place of
`proofkit.Run` for a solid step, and put the assertion after the build where the run takes one. The build argument is always the
third, each argument is written out as a plain name, and the case table is a named variable
rather than a call built in place. Each run takes a table of parameter cases, one subtest each.
Pass exactly the arguments the run's own signature declares — `proofkit3d.RunWithGate` takes the
gate as well as the assertion, and `proofkit.Run` takes neither — since a count the method does
not declare is a call Go cannot compile. The gate reads the run methods and their argument
counts out of `proof/proofkit/` and `proof/proofkit3d/`, so those sources are what a run is
checked against.

The gate reads this shape by line rather than by parsing Go, so anything else is refused, not
interpreted. A build under another name, a `Test` header outside the shape shown above, a `Test`
whose title does not match the step it builds, a run reached through a loop, a condition or a
closure, a run whose arguments come from one forwarded call as in `proofkit3d.Run(runArgs(t))`,
a run passing a different number of arguments than its method declares, and a proof file whose
header carries a build constraint all fail the check by name and line. A call to a `Run…` method
no harness package declares fails the same way, and that one is not confined to a registration:
a misspelled run in a helper is `undefined` to Go wherever it sits, so it is named wherever it
is written. Go reads a constraint from `//go:build` when nothing sits
between the slashes and the directive, whitespace or the end of the line follows the directive,
and the line is above the package clause. A leading byte order mark hides nothing: Go strips one
and honours what is under it, and the gate reads it the same way. The same text further down,
inside a string, inside a `/* */` comment in the header, written `// go:build …` with a space, or
run straight on as `//go:build!ignore`, is content rather than a constraint, and all of it
passes. The legacy `+build` form is refused in every spelling, including the near-misses Go
builds, because a proof file needs no build constraint at all. The refusal says what to write;
write that.

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
and a nil one fails the run. `proofkit3d.Unmodelled` is the 3D counterpart of
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
