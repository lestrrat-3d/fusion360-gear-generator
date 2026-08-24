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

3. **Draft.** Spawn a subagent with the standard drafting prompt: run
   `python3 .claude/skills/generate-gear/render_prompt.py emit-gear <gear>` and pass its printed
   output to the subagent unchanged. It writes `.tmp/<gear>.generated.py`. Add no per-gear
   hints; anything the drafter needs belongs in the step list.

4. **Gate.** Run `python3 .claude/skills/generate-gear/run_gates.py <gear>`. It runs all seven
   checks below plus the advisory novel-type report, and prints one verdict. Exit 0 = every gate
   that ran passed; exit 1 = a gate failed; exit 2 = a setup error (missing input, missing stubs,
   unreachable API database) that no new draft can fix.

5. **Diagnose and loop.** The runner prints a first-pass fault classification; confirm the rows it
   marks NEEDS JUDGMENT against the table below. An emit fault goes back to step 3 with the runner
   output appended, up to about three rounds. A compile fault, or an exit 2, stops the run.

6. **Place.** On success, run `python3 .claude/skills/generate-gear/stage.py <gear> module` from
   the repo root. It puts `.tmp/<gear>.generated.py` at `lib/geargen/<gear>.py` and reports what
   moved. This writes a file only; it does not commit, push, or touch Fusion's add-in directory.

7. **Report.** State the gate results and every step that was thin or wrong.

## Gates

`run_gates.py` runs every row below, in a cost-ordered sequence, and continues past a failure so
one round reports every problem. The one exception is a parse failure, which skips the gates that
read the candidate as Python and runs only the anchor check. The commands are listed so a single
gate can be re-run by hand; the runner prints the exact command for each row it reports.

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

A gear with no `spec/<gear>/contract.json` gets the contract gate reported as SKIP, and its
contract is prose-checked by the reviewing agent instead (`generate-gear/SKILL.md`). Pass
`--require-contract` where the manifest must exist, as it must for `spurgear`.

## Triage the type complaints

`run_gates.py` runs `check_novel_types.py` last, as its advisory row. It reports every type
complaint the candidate draws that no shipped gear in `lib/geargen/` draws, on the grounds that a
complaint working code never produces is worth a look.

This reports rather than gates, because an API the shipped gears never touch has no baseline and
correct code using it reads as new. Read each finding and decide. It is the only check that has
caught a method called on a class that does not define it, or a `BRepBodies` handed to a parameter
typed `ObjectCollection` — both of which passed all seven gates.

Pass `run_gates.py --gate-novel-types` to make findings fail the run, once the baseline covers the
API surface in question.

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

## Standard drafting prompt

The prompt text lives in `.claude/skills/emit-gear/prompt.md`; `{{gear}}` is its only
placeholder. Render it — never retype or paraphrase it — with:

    python3 .claude/skills/generate-gear/render_prompt.py emit-gear <gear>

Hand the printed output to the drafting subagent unchanged. The renderer refuses to print
anything for an unknown skill name, a missing template, or a template carrying a placeholder it
was not given, so a garbled render can never reach the subagent.
