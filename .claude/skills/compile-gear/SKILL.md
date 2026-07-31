---
name: compile-gear
description: Compile a gear's natural-language spec `spec/<gear>/instructions.md` (plus `fusion.md`) into two checkable artifacts — an ordered step list `spec/<gear>/steps.md` and a runnable geometry proof `proof/<gear>/`. The proof is executed, so a constraint scheme that cannot fully constrain fails here rather than inside Fusion. Use before `/emit-gear`, or when a spec changes and its step list needs rebuilding. Args: optional `<gear>` name (default `spurgear`).
---

# Compile a gear spec into a step list and a proof

The prose spec is the source of truth. This stage turns it into two artifacts a machine can check,
so that a mistake in the spec surfaces in seconds instead of after a Fusion session.

The step list and the proof are produced **together**, and neither is authoritative over the
other. Compiling is a loop: draft both, run the proof, check that the two agree, and reconcile
until the proof is green and both describe the same build.

## Inputs

- `gear` (default `spurgear`) names `spec/<gear>/instructions.md` and the outputs
  `spec/<gear>/steps.md` and `proof/<gear>/<gear>_test.go`.

## What a step is

A step is one entry in the Fusion timeline. Drawing a whole sketch is a single step, however much
geometry and however many constraints go into it. Each extrude, chamfer, pattern, combine and
fillet is a step. The detail inside one step stays inside it rather than being split across
several.

A step is `[GO]` when the proof exercises it and `[PROSE]` when nothing can. Everything 3D is
`[PROSE]` until `decad` can carry these shapes.

## Procedure

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists. Read this file,
   `PLAYBOOK.md`, the gear's spec end to end, and `proof/proofkit/` for the harness API.

2. **Build the API index.** Run `python3 .claude/skills/generate-gear/build_fusion_index.py`. It
   clones the Fusion API reference on demand, so a fresh machine needs no setup, and it is cheap
   once the clone exists. Run it every time.

3. **Record provenance.** Take `git hash-object` of `spec/<gear>/instructions.md` and
   `spec/<gear>/fusion.md`. These go in the step list's provenance table, and step 6 checks them.

4. **Draft.** Spawn a subagent with the verbatim prompt in the appendix, substituting only
   `<gear>`. It writes `.tmp/<gear>.steps.md` and `.tmp/<gear>_test.go`. Do **not** add per-gear
   hints, gotcha reminders or "high-risk" lists to that prompt. A hand-tuned prompt varies run to
   run and hides gaps by spoon-feeding what the prose should have said, so a green run would no
   longer say anything about the spec.

5. **Run the proof.** Copy the drafted test into `proof/<gear>/` and run
   `go test ./proof/<gear>/`. The proof must pass with nothing waived.

6. **Check.** Run `python3 .claude/skills/generate-gear/check_compile.py <gear>` from the repo
   root. It gates citations, step-to-proof agreement, the reality of every named API call, and the
   provenance hashes. It also prints the spec lines no step claims, which is worth skimming but
   never gates.

7. **Diagnose and loop.** Classify any failure with the table below. A draft fault goes back to
   step 4 with the failure text appended, up to about three rounds. A prose fault stops the run.

8. **Place.** On success, move both drafts into the working tree. This writes files only; it does
   not commit, push, or touch Fusion's add-in directory.

9. **Report.** State what was produced, the coverage list, and every prose fault found.

## Telling a draft fault from a prose fault

| Symptom | Usually |
|---|---|
| The proof fails to build | Draft fault |
| A cited line range does not exist | Draft fault |
| A step and its proof function disagree | Draft fault |
| A named API call does not exist, and the spec did not name it | Draft fault |
| A provenance hash does not match | Draft fault, or someone edited the spec mid-run |
| **A named API call does not exist, and the spec named it** | **Prose fault** |
| **The proof cannot fully constrain after three rounds** | **Prose fault** |
| **The drafter reports the spec as contradictory** | **Prose fault** |

The two bold API rows are the same check with different blame. When the spec itself writes
`setByOffset(plane, 0)` and the signature wants a `ValueInput`, the drafter is right to reproduce
it and the spec is what needs fixing.

A prose fault ends the run with a report. Never edit `instructions.md` or `fusion.md` from inside
this skill: a compiler that rewrites its own source removes the thing being checked.

## Rules

- Never read `lib/geargen/<gear>.py`, another gear's implementation, or a previous step list or
  proof for this gear. Otherwise the pipeline cannot bring up a gear that has no implementation
  yet, which is the point of building it.
- Never hand-edit the drafted artifacts to get a check to pass. They are build output, and a wrong
  step list means the prose or this procedure is wrong.
- Never add gear-specific guidance to the drafting prompt.

## Standard drafting prompt (use verbatim — substitute only `<gear>`)

> Compile the specification for `<gear>` into a step list and a runnable proof. Work in the repo
> worktree. Write `.tmp/<gear>.steps.md` and `.tmp/<gear>_test.go`.
>
> **Read, in full, only these:** `spec/<gear>/instructions.md`, `spec/<gear>/fusion.md` if it
> exists, every document those reference by name, `.claude/skills/generate-gear/PLAYBOOK.md`, and
> `proof/proofkit/` for the harness API.
>
> **Do not read** `lib/geargen/<gear>.py`, any other gear's implementation, or a previous
> `steps.md` or proof for this gear. If the spec is unclear, record it as a spec gap in your report
> and make your best attempt. Never resolve it by looking at existing output.
>
> **A step is one entry in the Fusion timeline.** Drawing a whole sketch is one step, however much
> geometry goes into it. So is each extrude, chamfer, pattern, combine, fillet. Write the step list
> at that size, and keep the detail inside the step it belongs to.
>
> **The step list opens with a provenance table** of each spec file and its `git hash-object`
> value, in a two-column markdown table with both cells in backticks.
>
> **Each step carries** a heading of the form `## <id> `[GO]` <title>` or with `[PROSE]`, the
> instructions themselves, a `**From:**` line naming the spec files and line ranges you compiled it
> from, and every Fusion API call it requires written inside a code span. A `[GO]` step also names
> the proof function that realises it.
>
> **Before naming any `adsk.*` call**, grep the API index at
> `~/.cache/fusion360-gear-generator/fusion-api-index.jsonl` for `"name":"<TheName>"` and read its
> `class` and `sig`. Write the call with the arguments that signature asks for. If the spec names a
> call that does not exist, or passes an argument of the wrong type, say so in your report and do
> not quietly correct it.
>
> **The proof is a Go test** in package `<gear>_test`, with one function per step named
> `step<Title>`, matching what the step list names. It builds a table of parameter cases and runs
> them through `proofkit.Run`. Model what Fusion does: use `CreateReferencePoint` for anything
> projected in rather than fixing coordinates, mark solid and construction geometry as Fusion
> would, and let profile detection split curves at crossings rather than drawing boundary arcs by
> hand. Call `proofkit.Step` as you move between parts of a step, and `proofkit.Unmodelled` for a
> case the proof cannot represent, never a silent return.
>
> **The proof must pass with nothing waived.** `proofkit` gates on the engine's full verdict,
> including that the constraints admit only one solution. A scheme that reaches DOF 0 but still
> allows a mirrored or rotated answer fails, and the fix is a constraint that carries a direction,
> not a comment.
>
> **The two artifacts must describe the same build.** Every `[GO]` step names its proof function,
> and every proof function is named by a step.
>
> **Report:** what you produced, and every place the spec was unclear, incomplete, contradictory,
> or wrong about the Fusion API. Those are the defects to fix. Do not smooth them over.
