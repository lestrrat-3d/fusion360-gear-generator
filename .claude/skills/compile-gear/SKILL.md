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

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists. Read this file,
   `PLAYBOOK.md`, the gear's spec end to end, and both harness APIs, `proof/proofkit/` and
   `proof/proofkit3d/`.

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
   files under `.tmp/<gear>-proof/`. Do
   **not** add per-gear hints, gotcha reminders or "high-risk" lists to that prompt. A hand-tuned
   prompt varies run to run and hides gaps by spoon-feeding what the prose should have said, so a
   green run would no longer say anything about the spec.

4. **Run the proof.** Run `python3 .claude/skills/generate-gear/stage.py <gear> proof --run` from
   the repo root. It copies every drafted proof file from `.tmp/<gear>-proof/` into
   `proof/<gear>/`, deletes any `.go` file there the draft no longer produces, indexes the result
   so step 5's tracked-or-committed check can see a first-time proof, and then runs
   `bash proof/run.sh`. The wrapper enters the `proof/` module and configures the local engine
   replacements; the proof must pass with nothing waived. Exit 2 means the placement was refused
   and nothing moved; exit 1 means the proof itself is red.

5. **Check.** Run `python3 .claude/skills/generate-gear/check_compile.py <gear>` from the repo
   root. It gates citations, step-to-proof agreement, the reality of every named API call, and the
   provenance hashes. It also prints the spec lines no step claims, and every call on the
   unverified watchlist the step list makes; both are worth reading and neither gates.

   Then, **only if `lib/geargen/<gear>.py` already exists**, run
   `python3 .claude/skills/generate-gear/check_step_calls.py spec/<gear>/steps.md
   lib/geargen/<gear>.py`. That gate runs in CI against the checked-in module, so a recompiled step
   list that disagrees with it breaks the build even though both other checks are green. Run it
   with `--names`, which prints exactly the missing call names, one per line; classify each name
   and pass only the names back to the drafter: a name
   the step list mentions without requiring takes the exemption directive, and a call the module
   genuinely fails to make is work for `/emit-gear`, not for this stage. Never hand the drafter
   anything the module does or does not contain, and never let it read the module — the pipeline
   has to be able to compile a gear that has no implementation yet.

6. **Diagnose and loop.** Classify any failure with the table below. A draft fault goes back to
   step 3 with the failure text appended, up to about three rounds. A prose fault stops the run.

7. **Place.** On success, run `python3 .claude/skills/generate-gear/stage.py <gear> steps` and
   `python3 .claude/skills/generate-gear/stage.py <gear> proof` from the repo root. They put the
   drafted step list and every drafted proof file into the working tree and report what moved; the
   proof call repeats step 4's placement, so it should report every file unchanged. This writes
   files and the git index only; it does not commit, push, or touch Fusion's add-in directory.

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
| **A named API call does not exist, and the spec named it** | **Prose fault** |
| **The proof cannot fully constrain after three rounds** | **Prose fault** |
| **The proof cannot build a sound solid after three rounds** | **Prose fault** |
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

## Standard drafting prompt

The prompt text lives in `.claude/skills/compile-gear/prompt.md`; `{{gear}}` is its only
placeholder. Render it — never retype or paraphrase it — with:

    python3 .claude/skills/generate-gear/render_prompt.py compile-gear <gear>

Hand the printed output to the drafting subagent unchanged. The renderer refuses to print
anything for an unknown skill name, a missing template, or a template carrying a placeholder it
was not given, so a garbled render can never reach the subagent.
