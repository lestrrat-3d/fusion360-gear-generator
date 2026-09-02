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
  `spec/<gear>/steps.md` and the proof directory `proof/<gear>/`. The proof is a directory, not a
  single file: a gear splits its sketch proof, its solid proof and the geometry they share across
  as many Go files as the split needs.

## What a step is

A step is one entry in the Fusion timeline. Drawing a whole sketch is a single step, however much
geometry and however many constraints go into it. Each extrude, chamfer, pattern, combine and
fillet is a step. The detail inside one step stays inside it rather than being split across
several.

A step is `[GO]` when one of the two harnesses can exercise it. `proofkit` covers the sketch and
constraint steps, in the sketch engine. `proofkit3d` covers the solid steps, in `decad`, which
builds bodies by extrude, revolve and loft, combines them with union, cut and intersect, blends
their edges with chamfer and fillet, hollows one with shell, and repeats one as placed copies,
which is how a pattern is expressed.

A step is `[PROSE]` when neither harness reaches it: reading the dialog, creating the component and
registering its parameters, visibility and cleanup, and any solid operation `decad` cannot
represent. `[PROSE]` says something about the harnesses, not about the dimension. A 3D step whose
shape `decad` can build is `[GO]` like any other.

A boundary the harness refuses is not by itself a `[PROSE]` verdict. Substitute geometry the
harness does accept — chord a curve the harness will not trim, draw a split the engine will not
perform — and assert what the substitute still pins, saying in the proof what was substituted and
what the substitution costs. Only a step no substitute reaches is `[PROSE]`, and then the proof
file records that, next to the nearest thing it does build, with the reason. That record is what
`CLAUDE.md` asks for, and it belongs in the proof rather than only in the step list, because the
proof is where the next reader is looking for the missing check.

## Procedure

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists. Run
   `python3 .claude/skills/generate-gear/preflight.py <gear> --stage compile --default-model
   <the session's default model>` and fix every
   `[FAIL]` before drafting; it verifies the engines, the go toolchain and the API database so a
   broken environment fails here instead of mid-run. Read this file end to end. Do **not** read
   `PLAYBOOK.md`, the spec, or the harness APIs up front: the drafting subagent reads them in
   full, and the orchestrator reads on demand — the drafted step list around each name step 5
   classifies, the spec lines a `fault:` line names in step 6, and a harness or playbook section
   only when a specific diagnosis calls for it.

2. **Stamp provenance.** The provenance table is generated, never typed. After each drafting round
   in step 3, run `python3 .claude/skills/generate-gear/gen_provenance.py <gear> --write
   .tmp/<gear>.steps.md` from the repo root. It computes the input set owned by
   `.claude/skills/generate-gear/check_compile.py` — existing `spec/<gear>/instructions.md`, optional
   `spec/<gear>/fusion.md`, `.claude/skills/generate-gear/PLAYBOOK.md`, and existing auxiliary Markdown
   documents referenced by those two spec files — and writes the table under the heading the draft
   left empty. The table travels with the file when it is copied, so nothing else needs stamping,
   and step 5 checks the result. A hash mismatch there means a source changed after the stamp: run
   the same command against the copy the check reads, then check again. The playbook belongs in the
   set because steps cite its rules by anchor, so a playbook fix leaves every step list stale until
   it is recompiled.

3. **Draft.** Spawn a subagent with the standard drafting prompt: run
   `python3 .claude/skills/generate-gear/render_prompt.py compile-gear <gear>` and pass its
   printed output to the subagent unchanged. It writes `.tmp/<gear>.steps.md` and the proof
   files under `.tmp/<gear>-proof/`. This drafter takes the `design` role, because the stage
   interprets prose, so it runs on the session's default model; resolve it with
   `python3 .claude/skills/generate-gear/pick_model.py --role design --default <the session's
   default model>` and see `.claude/skills/generate-gear/MODELS.md` for the rule. Do
   **not** add per-gear hints, gotcha reminders or "high-risk" lists to that prompt. A hand-tuned
   prompt varies run to run and hides gaps by spoon-feeding what the prose should have said, so a
   green run would no longer say anything about the spec.

4. **Scaffold, place, and run the gates.** First run
   `python3 .claude/skills/generate-gear/scaffold_proof.py <gear> --steps .tmp/<gear>.steps.md
   --out .tmp/<gear>-proof/zz_registrations_test.go`. It turns each `[GO]` step's `proof-run`
   annotation into the Go `Test` registrations the drafter never writes; a scaffolder finding is a
   draft fault, handled as step 6 handles one. The two tools have separate jobs: `scaffold_proof.py`
   generates that one file, and `stage.py` places every drafted file including it.

   Then run `python3 .claude/skills/generate-gear/stage.py <gear> compile` from the repo root. It
   places both drafted artifacts in one call: `.tmp/<gear>.steps.md` at `spec/<gear>/steps.md`,
   and every drafted proof file from `.tmp/<gear>-proof/` into `proof/<gear>/`, deleting any `.go`
   file there the draft no longer produces and indexing the result so step 5's
   tracked-or-committed check can see a first-time proof. The step list must be placed here too,
   because `check_compile.py` reads `spec/<gear>/steps.md`, never the draft in `.tmp/`. The
   command refuses both placements if it would refuse either; exit 2 means nothing moved. There
   is no `--run`; the gate runner below runs the proof.

   Then run `python3 .claude/skills/generate-gear/run_compile_gates.py <gear> >
   .tmp/<gear>.compile-gates.txt` from the repo root and read the file for the verdict. The stored
   copy is also what a retry round hands back to the drafter.
   It runs `bash proof/run.sh`, then `check_compile.py <gear>`, then
   `extract_playbook.py <gear> --min-anchors 1`, then — only when `lib/geargen/<gear>.py`
   exists — `check_step_calls.py`, in that order, and prints one verdict
   plus a first-pass fault classification. The proof wrapper enters the `proof/` module and
   configures the local engine replacements; the proof must pass with nothing waived. Exit 1 means
   a gate failed on content; exit 2 is a setup error, and a setup error never goes back to the
   drafter.

5. **Check.** The runner already ran every check. `check_compile.py` gates spec citations,
   step-to-proof agreement, the reality of every named API call, and the provenance hashes. It
   also prints the spec lines no step claims, and every call on the unverified watchlist the step
   list makes; both are worth reading and neither gates.

   `extract_playbook.py` builds the playbook slice `/emit-gear` will read and, with
   `--min-anchors 1`, refuses a step list that cites no `[PB-…]` anchor. That extract is the only
   playbook text the emit drafter sees, so a step list citing nothing leaves it with the core
   sections and nothing else — about 8% of the file. The failure is the drafter's: send the report
   back and let it cite the rules the steps lean on. The stage exits 1 for one other reason, an
   anchor cited that the playbook does not define, which is either an invented anchor or a
   playbook edited since the step list was drafted; the printed fault line says which of the two
   it read.

   `check_step_calls.py` runs **only if `lib/geargen/<gear>.py` already exists**, and the runner
   reports it as skipped when it does not. That gate runs in CI against the checked-in module, so a
   recompiled step list that disagrees with it breaks the build even though the other checks are
   green. On a failure, run `python3 .claude/skills/generate-gear/check_step_calls.py
   spec/<gear>/steps.md lib/geargen/<gear>.py --names`, which prints exactly the missing call
   names, one per line; classify each name and pass only the names back to the drafter: a name the
   step list mentions without requiring takes the exemption directive, and a call the module
   genuinely fails to make is work for `/emit-gear`, not for this stage. Never hand the drafter
   anything the module does or does not contain, and never let it read the module — the pipeline
   has to be able to compile a gear that has no implementation yet. Classify by reading the
   drafted step list around each printed name in `.tmp/<gear>.steps.md`; that is the only text
   the classification needs.

6. **Diagnose and loop.** Classify any failure with the table below. A draft fault goes back to
   the drafter, up to about three rounds in total. A prose fault stops the run. Read, at this
   point, only what the failure names: the spec file and line a `fault: prose` line prints, the
   step and proof text the failure quotes, and any playbook anchor the step cites.

   A draft fault does not change any input file, so the drafter that produced it still holds
   every input in context. Send it the stored gate report `.tmp/<gear>.compile-gates.txt`
   verbatim with `SendMessage` and let it revise `.tmp/<gear>.steps.md` and
   `.tmp/<gear>-proof/`; never paste failure text edited by hand, and do not tell the drafter to
   re-read inputs it has already read.

   Spawn a fresh drafting subagent (a full step 3) only when one of these holds: this is the
   first round; an input file (the spec, `fusion.md`, the playbook, or a harness package)
   changed since the drafter read it; the same check fails on the same step in two consecutive
   continued rounds; or the drafter is no longer reachable. A fresh spawn keeps the `design`
   role, and a continued round changes no model, because a resumed agent keeps the one it was
   spawned on. A fresh retry round re-renders the
   prompt with `python3 .claude/skills/generate-gear/render_prompt.py compile-gear <gear>
   --failure-file .tmp/<gear>.compile-gates.txt`, which appends the stored gate report verbatim;
   hand the printed output to the drafter unchanged. The first round's prompt is always the
   rendered standard prompt with no failure file.

7. **Place.** On success, run `python3 .claude/skills/generate-gear/stage.py <gear> compile`
   from the repo root. It repeats step 4's placement of the step list and the proof, so it
   should report every file unchanged; it exists as a step so a run whose gates were green
   ends with the working tree verified to match the draft. This writes files and the git index
   only; it does not commit, push, or touch Fusion's add-in directory.

8. **Report.** State what was produced, the coverage list, and every prose fault found.

## Telling a draft fault from a prose fault

| Symptom | Usually |
|---|---|
| The proof fails to build | Draft fault |
| A cited line range does not exist | Draft fault |
| A step and its proof function disagree | Draft fault |
| A step names a call the module is not required to make | Draft fault |
| A named API call does not exist, and the spec did not name it | Draft fault |
| A provenance hash does not match | A source changed after the table was stamped; re-run `gen_provenance.py` and check again |
| The step list cites no playbook anchor | Draft fault |
| A cited `[PB-…]` anchor is defined nowhere in the playbook | Draft fault, unless the playbook changed after the step list was drafted |
| The scaffolder refuses an annotation, or the registration check names a mismatch | Draft fault |
| **A named API call does not exist, and the spec named it** | **Prose fault** |
| **The proof cannot fully constrain after three rounds** | **Prose fault** |
| **The proof cannot build a sound solid after three rounds** | **Prose fault** |
| **The drafter reports the spec as contradictory** | **Prose fault** |

The two bold API rows are the same check with different blame. When the spec itself writes
`setByOffset(plane, 0)` and the signature wants a `ValueInput`, the drafter is right to reproduce
it and the spec is what needs fixing. For the "does not exist" half, `check_compile.py` prints the
blame itself as a `fault: prose`/`fault: draft` line under each unresolved call, naming the spec
file and line when one names the call, so the row needs no manual spec search.

A prose fault ends the run with a report. Never edit `instructions.md` or `fusion.md` from inside
this skill: a compiler that rewrites its own source removes the thing being checked.

## Rules

- Never read `lib/geargen/<gear>.py`, another gear's implementation, or a previous step list or
  proof for this gear. Otherwise the pipeline cannot bring up a gear that has no implementation
  yet, which is the point of building it.
- Never hand-edit the drafted artifacts to get a check to pass. They are build output, and a wrong
  step list means the prose or this procedure is wrong.
- Never add gear-specific guidance to the drafting prompt.
- Never write or hand-edit `proof/<gear>/zz_registrations_test.go`. It is generated; rerun
  `scaffold_proof.py` instead.

## Standard drafting prompt

The prompt text lives in `.claude/skills/compile-gear/prompt.md`; `{{gear}}` is its only
placeholder. Render it — never retype or paraphrase it — with:

    python3 .claude/skills/generate-gear/render_prompt.py compile-gear <gear>

Hand the printed output to the drafting subagent unchanged. On a retry round, `--failure-file`
appends the previous round's gate report verbatim; the framing text is fixed in the renderer, so
the retry prompt is as standard as the first. The renderer refuses to print anything for an
unknown skill name, a missing template, or a template carrying a placeholder it was not given, so
a garbled render can never reach the subagent.
