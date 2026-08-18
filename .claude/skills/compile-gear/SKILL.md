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

## Procedure

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists. Read this file,
   `PLAYBOOK.md`, the gear's spec end to end, and both harness APIs, `proof/proofkit/` and
   `proof/proofkit3d/`.

2. **Record provenance.** Use the provenance input set owned by
   `.claude/skills/generate-gear/check_compile.py`: existing `spec/<gear>/instructions.md`, optional
   `spec/<gear>/fusion.md`, `.claude/skills/generate-gear/PLAYBOOK.md`, and existing auxiliary Markdown
   documents referenced by those two spec files. Take `git hash-object` of every member and put each
   row in the step list's provenance table. Step 5 checks them. The playbook belongs there because
   steps cite its rules by anchor, so a playbook fix leaves every step list stale until it is recompiled.

3. **Draft.** Spawn a subagent with the verbatim prompt in the appendix, substituting only
   `<gear>`. It writes `.tmp/<gear>.steps.md` and the proof files under `.tmp/<gear>-proof/`. Do
   **not** add per-gear hints, gotcha reminders or "high-risk" lists to that prompt. A hand-tuned
   prompt varies run to run and hides gaps by spoon-feeding what the prose should have said, so a
   green run would no longer say anything about the spec.

4. **Run the proof.** Copy every drafted proof file into `proof/<gear>/` and run
   `bash proof/run.sh`. The wrapper enters the `proof/` module and configures the local engine
   replacements; the proof must pass with nothing waived.

5. **Check.** Run `python3 .claude/skills/generate-gear/check_compile.py <gear>` from the repo
   root. It gates citations, step-to-proof agreement, the reality of every named API call, and the
   provenance hashes. It also prints the spec lines no step claims, and every call on the
   unverified watchlist the step list makes; both are worth reading and neither gates.

6. **Diagnose and loop.** Classify any failure with the table below. A draft fault goes back to
   step 3 with the failure text appended, up to about three rounds. A prose fault stops the run.

7. **Place.** On success, move the drafted step list and every drafted proof file into the working
   tree. This writes files only; it does not commit, push, or touch Fusion's add-in directory.

8. **Report.** State what was produced, the coverage list, and every prose fault found.

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

## Standard drafting prompt (use verbatim — substitute only `<gear>`)

> Compile the specification for `<gear>` into a step list and a runnable proof. Work in the repo
> worktree. Write the step list to `.tmp/<gear>.steps.md`, and the proof, as one or more Go files,
> to `.tmp/<gear>-proof/`.
>
> **Read, in full, only these:** `spec/<gear>/instructions.md`, `spec/<gear>/fusion.md` if it
> exists, every document those reference by name, `.claude/skills/generate-gear/PLAYBOOK.md`,
> `proof/proofkit/` for the sketch harness API, `proof/proofkit3d/` for the solid harness API, and
> `proof/involute/` for the involute tooth math the spur family shares, so you import it rather
> than deriving it again.
>
> **Do not read** `lib/geargen/<gear>.py`, any other gear's implementation, or a previous
> `steps.md` or proof for this gear. If the spec is unclear, record it as a spec gap in your report
> and make your best attempt. Never resolve it by looking at existing output.
>
> **A step is one entry in the Fusion timeline.** Drawing a whole sketch is one step, however much
> geometry goes into it. So is each extrude, chamfer, pattern, combine, fillet. Write the step list
> at that size, and keep the detail inside the step it belongs to.
>
> **The step list opens with a provenance table** of each file in the provenance input set owned by
> `.claude/skills/generate-gear/check_compile.py`, with its `git hash-object` value, in a two-column
> markdown table with both cells in backticks. The set covers the gear's `instructions.md`, its
> `fusion.md` if present, the playbook, and existing auxiliary Markdown documents referenced by the
> two spec files.
>
> **Each step carries** a heading of the form `## <id> `[GO]` <title>` or with `[PROSE]`, the
> instructions themselves, a `**From:**` line naming the spec files and line ranges you compiled it
> from, and every Fusion API call it requires written inside a code span. A `[GO]` step also names
> the proof function that realises it.
>
> **Before naming any `adsk.*` call**, ask the `fusion:query-api` skill about it. Two questions
> carry most of the work: `members <Class>` lists everything a class offers, inherited members
> included, each with the class that declares it, which is how you find out whether the class you
> are calling on really has the member; and `show <Class>.<member>` gives one member's signature
> and documentation. Write the call with the arguments that signature asks for. If the spec names
> a call the API does not have, or passes an argument of the wrong type, say so in your report and
> do not quietly correct it.
>
> **The proof is a Go test** in package `<gear>_test`, spread over as many files as the split
> needs, with one function per step. Every step function, 2D or 3D alike, is named `step<Title>`,
> matching what the step list names, and is passed as the build argument — the third — to a
> `proofkit.Run`, `proofkit3d.Run`, `proofkit3d.RunSolid` or `proofkit3d.RunWithGate` call inside a
> Go `Test` function. A build function under any other name, or one no `Test` reaches, is invisible
> to the gate, and the step naming it fails the check. Each run takes a table of parameter cases,
> one subtest each.
>
> **A sketch step** builds through `proofkit.Run`, whose build function is
> `func(t testing.TB, s *sketch.Sketch, p map[string]float64)`. Model what Fusion does: use
> `CreateReferencePoint` for anything projected in rather than fixing coordinates, mark solid and
> construction geometry as Fusion would, and let profile detection split curves at crossings rather
> than drawing boundary arcs by hand. Call `proofkit.Step` as you move between parts of a step, and
> `proofkit.Unmodelled` for a case the proof cannot represent, never a silent return.
>
> **A solid step** builds through `proofkit3d`, whose build function is
> `func(t *testing.T, doc *decad.Document, params map[string]float64) []*decad.Body` and returns
> the bodies the step leaves behind. Every `proofkit3d` run also takes an assertion,
> `func(t *testing.T, doc *decad.Document, bodies []*decad.Body, params map[string]float64)`, which
> runs after the gate and checks the measurements the step is supposed to produce; it is required,
> and a nil one fails the run. `proofkit3d.Unmodelled` is the 3D counterpart of
> `proofkit.Unmodelled`, for a case `decad` cannot represent.
>
> **The proof must pass with nothing waived.** `proofkit` gates a sketch on
> `sketch.VerificationReport.Check`, which asks for more than DOF 0: no conflicting or redundant
> constraint, no stale or broken reference geometry, valid profiles, a system that is not
> near-singular, and no discrete ambiguity. A scheme that reaches DOF 0 but still allows a mirrored
> or 180-degree-rotated answer fails there, and the fix is a constraint that carries a direction,
> not a comment.
>
> `proofkit3d.Run` gates a solid on `decad`'s own verification verdict: the document report has to
> come back trustworthy, and the build has to return bodies rather than nothing or a nil.
> `proofkit3d.RunSolid` reads the same report but tolerates exactly one kind of diagnostic, an area
> or centroid reading a faceted boolean left outside the default tolerance, and adds the topology a
> solid has to have: every body reports as solid, watertight, manifold and free of
> self-intersection, with a single lump and no voids. `proofkit3d.RunWithGate` takes the gate as an
> argument; do not pass a weaker one to get a build through.
>
> **The two artifacts must describe the same build.** Every `[GO]` step names its proof function,
> and every proof function is named by a step.
>
> **Report:** what you produced, and every place the spec was unclear, incomplete, contradictory,
> or wrong about the Fusion API. Those are the defects to fix. Do not smooth them over.
