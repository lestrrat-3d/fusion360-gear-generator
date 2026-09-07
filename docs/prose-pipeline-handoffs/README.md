# Prose pipeline implementation handoffs

## Assignment

These documents specify changes to the prose-to-code pipeline in `fusion360-gear-generator`.
They refine [the backlog](../prose-pipeline-tasks.md). Implementation has not started.
An implementer receives this file and exactly one task document. Each numbered submission inside a task
is a separate checkpoint; the implementer completes that submission before starting the next one.

The inspected source baseline is `f38c26da97f27dd9ec962a65646d19c3cf106cf0`.
The dispatcher supplies an isolated worktree containing the prerequisite changes and these documents.
The implementer checks the named functions before editing. A changed signature is a reason to report
the difference and request an updated handoff, not to redesign the task.

## Dispatch order

| Assignment | Document | Required predecessors |
|---|---|---|
| T0 | [Preserve evidence](00-baseline.md) | None |
| T1 | [Contract handoff](01-contract.md) | T0 |
| T2 | [Citation metadata](02-citations.md) | T0 |
| T4A | [Initial gate order](04-gates.md#submission-a-initial-gate-order) | T0 |
| T5 | [API status](05-api-status.md) | T0 |
| T3 | [Required-call metadata](03-calls.md) | T1, T2, T5 |
| T4B | [Emission readiness](04-gates.md#submission-b-emission-readiness) | T3, T4A |
| T6 | [Typed emitter guidance](06-types.md) | T1, T5 |
| T7 | [Proof examples](07-proof-examples.md) | T0 |
| T8 | [Controlled measurements](08-measurement.md) | T0; serial trials also require T4B |
| T9A, T9B, T9C | [Context and model trials](09-context-models.md) | Named tooling prerequisites; accepted T8 baseline before trials |

T8 records trials after each completed change. Before T4B, a trial that cannot reach emission is recorded
as a failed compilation; the dispatcher does not bypass the existing stage rules.

## Common execution contract

The implementer reads applicable repository instructions and their linked pre-read documents first.
All paths below are repository-relative. Tool filenames without a directory refer to
`.claude/skills/generate-gear/`. `P` in validation commands refers to that directory.

```sh
P=.claude/skills/generate-gear
mkdir -p .tmp
```

The worktree must have writable caches and the Go toolchain required by `proof/go.mod`.
The dispatcher provides the available model identity and pinned engine checkout paths when a task needs them.
The implementer does not invent model names or bypass engine revision verification.

Each task names its allowed edits. Existing generated gear modules, gear steps, and gear proofs are not
hand-edited to satisfy a gate. Generic fixtures and compiler tooling are authored directly as specified.
Changes to source specifications or acceptance rules require a separate diagnosis; an implementer reports
the conflict instead of choosing which source to override. Existing skill rules remain in effect until
the task explicitly updates their owning procedure.

Tests use local fixtures. They do not import `adsk` or run a generated module outside Fusion.
Fixtures use the invented gear name `fixturegear` and invented helper names unless testing a shared API policy table.
Production manifest contents remain unchanged unless a task explicitly permits an edit.

## Common validation

Each task's focused command must exit 0. The broader suite below must also exit 0 before handoff.
An environmental failure is reported with its command and log; a test is not deleted or weakened to pass.

```sh
python3 -m unittest discover -s "$P" -p 'test_*.py' > .tmp/handoff-python-tests.txt 2>&1
git diff --check
```

For proof-source changes, the additional final command is:

```sh
bash proof/run.sh > .tmp/handoff-proof-tests.txt 2>&1
```

The dispatcher supplies `SKETCH_DIR` and `DECAD_DIR` when the sibling repositories are not at the pins.
These commands are summaries of existing repository verification, not replacement gate definitions.
The corresponding skills own complete validation of generated artifacts.

## Escalation

The implementer stops the dependent part of a task when a prerequisite is absent, a required file or
function changed incompatibly, a test requires a new acceptance rule, or an example fails its stated
mathematical assertion on pinned engines. Independent in-scope work continues where possible.

The escalation report contains the task and submission, exact command, saved output path, smallest local
fixture, expected result, actual result, and the specific decision needed. It does not contain a guessed fix.
Open-ended optimization, source interpretation, and tolerance changes are outside these handoffs.

## Completion record

Each implementer writes `.tmp/<task>-handoff.md` with these fields:

| Field | Required content |
|---|---|
| Task and submission | The assigned ID and completed checkpoint |
| Base | The starting commit and prerequisite references |
| Files | Every edited path and why it changed |
| Tests | Commands, exit codes, and saved logs |
| Compatibility | Legacy behavior tested and intentional migration requirements |
| Remaining work | Concrete failures or `None` |
| Measurements | Observed values or `Not measured` |

The implementation must satisfy every fixture row in its document. Passing tests does not establish
Fusion runtime behavior or a performance improvement. T8 owns performance comparisons.

## Copyable dispatch instruction

```text
Implement the assigned prose-to-code pipeline change in fusion360-gear-generator.
Read docs/prose-pipeline-handoffs/README.md and the assigned task document.
Use the supplied isolated worktree and complete only the assigned submission.
Follow its fixed interfaces, allowed edits, fixture matrix, and validation commands.
Do not resolve an escalation condition by changing the design or weakening a check.
Return the completion record specified by README.md.
```

The dispatcher attaches the task filename, submission ID, worktree path, prerequisite references,
and any task-specific inputs. No conversation history is required beyond those inputs and repository instructions.
