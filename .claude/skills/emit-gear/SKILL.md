---
name: emit-gear
description: Write `lib/geargen/<gear>.py` from a compiled step list `spec/<gear>/steps.md` and its proof `proof/<gear>/`, without reading the prose spec or any existing implementation. The interpretation already happened during `/compile-gear`, so this stage is transcription against a fixed API and a small model should manage it. Use after `/compile-gear`. Args: optional `<gear>` name (default `spurgear`).
---

# Emit the add-in from a compiled step list

The step list and the proof are the only description of the gear this stage gets. Reading the
prose here would hide a thin step list behind a working add-in, and the step list is the artifact
the pipeline exists to make trustworthy.

## Inputs

- `gear` (default `spurgear`) names `spec/<gear>/steps.md` and `proof/<gear>/`, and the output
  `lib/geargen/<gear>.py`.

## Procedure

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists.

2. **Check the inputs are current.** Run
   `python3 .claude/skills/generate-gear/check_compile.py <gear>`. Emitting from a stale step list
   wastes the round. If it fails, run `/compile-gear <gear>` first.

3. **Draft.** Spawn a subagent with the verbatim prompt in the appendix, substituting only
   `<gear>`. It writes `.tmp/<gear>.generated.py`. Add no per-gear hints; anything the drafter
   needs belongs in the step list.

4. **Gate.** Run all seven checks below. Every one must exit 0.

5. **Diagnose and loop.** Classify any failure with the table below. An emit fault goes back to
   step 3 with the failure text appended, up to about three rounds. A compile fault stops the run.

6. **Place.** On success, move the draft to `lib/geargen/<gear>.py`. This writes a file only; it
   does not commit, push, or touch Fusion's add-in directory.

7. **Report.** State the gate results and every step that was thin or wrong.

## Gates

| Gate | Command | What it catches |
|---|---|---|
| Parse | `python3 -c "import ast; ast.parse(open('.tmp/<gear>.generated.py').read())"` | Syntax |
| Types | `pyright_check.py .tmp/<gear>.generated.py` | Undefined names, wrong `adsk` submodule |
| Input reads | `check_input_read.py .tmp/<gear>.generated.py` | A dialog input read with the wrong helper |
| Contract | `check_contract.py spec/<gear>/contract.json .tmp/<gear>.generated.py` | Missing classes, hooks, `ctx` fields, constants; a guarded constraint recipe reverted to its rejected alternative |
| Step calls | `check_step_calls.py spec/<gear>/steps.md .tmp/<gear>.generated.py` | A named call never made, an abandoned stub, a shared point passed as `.geometry` |
| Anchors | `python3 .claude/skills/generate-gear/check_anchors.py` | A proof or step-list anchor is missing or stale |
| API calls | `check_api_calls.py .tmp/<gear>.generated.py` | A method that exists nowhere in the Fusion API |

`pyright_check.py` REVIEW findings are advisory stub noise. Only BLOCKING gates.

## Triage the type complaints

After the seven gates pass, run
`python3 .claude/skills/generate-gear/check_novel_types.py .tmp/<gear>.generated.py`. It reports
every type complaint the candidate draws that no shipped gear in `lib/geargen/` draws, on the
grounds that a complaint working code never produces is worth a look.

This reports rather than gates, because an API the shipped gears never touch has no baseline and
correct code using it reads as new. Read each finding and decide. It is the only check that has
caught a method called on a class that does not define it, or a `BRepBodies` handed to a parameter
typed `ObjectCollection` — both of which passed all seven gates.

Pass `--gate` to make findings fail the run, once the baseline covers the API surface in question.

## Telling an emit fault from a compile fault

| Symptom | Usually |
|---|---|
| Syntax error, undefined name, wrong submodule | Emit fault |
| A named call never made, or a stub left behind | Emit fault |
| A method that exists nowhere | Emit fault |
| A wrong argument type where the step gave the signature | Emit fault |
| **A step names a call that does not exist** | **Compile fault** |
| **A step gives an argument type the signature rejects** | **Compile fault** |
| **A step is too thin to act on, and the drafter says so** | **Compile fault** |
| **Two steps contradict each other** | **Compile fault** |

A compile fault ends the run. Fix the prose or the compile procedure and recompile. Never patch the
step list by hand, and never work around it in the Python.

## Rules

- Never read the prose spec, an existing `lib/geargen/<gear>.py`, or a previous draft.
- Never hand-edit the step list or the proof.
- Never add gear-specific guidance to the drafting prompt.

## Standard drafting prompt (use verbatim — substitute only `<gear>`)

> Write the Fusion 360 add-in implementation for `<gear>` by following its compiled step list. Work
> in the repo worktree. Write `.tmp/<gear>.generated.py`. Do not try to run it; the `adsk` modules
> only exist inside Fusion, so parsing is as far as it goes.
>
> **Read, in this order:** `spec/<gear>/steps.md`, which is your instruction set and which you work
> through in order; `proof/<gear>/`, the checked geometry, which steps tagged `[GO]` tell you to
> transliterate literally rather than re-derive; `.claude/skills/generate-gear/PLAYBOOK.md` for the
> rules the steps cite by anchor; and the framework you build on and must not reimplement, which is
> `lib/geargen/base.py`, `misc.py`, `utilities.py`, `spurproxy.py` and `lib/fusion360utils/`.
>
> **Do not read** `lib/geargen/<gear>.py`, `spec/<gear>/instructions.md`, `spec/<gear>/fusion.md`,
> or any previous draft. The step list is deliberately the only description of the gear you get. If
> a step is unclear, record it as a defect in your report and make your best attempt.
>
> **Before writing any `adsk.*` call**, ask the `fusion:query-api` skill about it. Two questions
> carry most of the work: `members <Class>` lists everything a class offers, inherited members
> included, each with the class that declares it, which is how you find out whether the class you
> are calling on really has the member; and `show <Class>.<member>` gives one member's signature
> and documentation. Pass what the signature asks for. Where it says `ValueInput`, a bare number
> raises. Where it says `ObjectCollection`, a Python list raises. Where it says `Point3D`, a
> `SketchPoint` raises. If the step list names a call the API does not have, report it and do not
> quietly correct it.
>
> **Do every step.** A step you cannot finish is a defect to report, never a comment left in the
> file and never a silent omission.
>
> **Self-check before finishing**, fixing until all seven gates pass: parse, `pyright_check.py` with
> 0 BLOCKING, `check_input_read.py`, `check_contract.py`, `check_anchors.py`, `check_step_calls.py`
> and `check_api_calls.py`. Never silence a finding by deleting a comment, renaming a variable, or
> removing the call it objects to.
>
> **Report:** the gate results, the final line count, and every step that was unclear, incomplete,
> or that you could not carry out as written, named by its step ID.
