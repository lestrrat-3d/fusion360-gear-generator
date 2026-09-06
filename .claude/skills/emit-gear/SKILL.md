---
name: emit-gear
description: Write `lib/geargen/<gear>.py` from a compiled step list `spec/<gear>/steps.md` and its proof `proof/<gear>/`, without reading the prose spec or any existing implementation. The interpretation already happened during `/compile-gear`, so this stage is transcription against a fixed API and its drafter takes the mechanical model tier. Use after `/compile-gear`. Args: optional `<gear>` name (default `spurgear`).
---

# Emit the add-in from a compiled step list

The step list and the proof are the only description of the gear this stage gets. Reading the
prose here would hide a thin step list behind a working add-in, and the step list is the artifact
the pipeline exists to make trustworthy.

## Inputs

- `gear` (default `spurgear`) names `spec/<gear>/steps.md` and `proof/<gear>/`, and the output
  `lib/geargen/<gear>.py`.

## Optional pipeline timing

For an opt-in timing run, use `.claude/skills/generate-gear/pipeline_timing.py` and follow the
event boundaries in [the pipeline timing pilot](../generate-gear/pipeline-timing-pilot.md).
Record observed `preflight`, `input_reading`, `drafting`, `validation`,
`placement`, and `overall` events around the existing commands. Import the complete gate JSON
after validation so its runner policy remains attached to the timing data.
Use one `drafting` round per draft attempt, including retries. A validation event covers the
complete runner invocation and its report import; record advisory triage only after reviewing
the advisory findings.

## Procedure

1. **Setup.** Work in a worktree, never the root checkout. Ensure `.tmp/` exists. Run
   `python3 .claude/skills/generate-gear/preflight.py <gear> --stage emit --default-model <the
   session's default model>` and fix every `[FAIL]`
   before drafting; it verifies the engines, the go toolchain and the API database, and runs
   `check_compile.py <gear>` as its `steps-current` row, so a broken environment or a stale
   step list fails here instead of mid-run. If `steps-current` fails, run `/compile-gear <gear>`
   first.

2. **Draft.** First run `python3 .claude/skills/generate-gear/extract_playbook.py <gear>` from the
   repo root. It writes `.tmp/<gear>.playbook-extract.md`, the playbook rules the step list cites
   by anchor plus the fixed core sections, which is the only playbook text the drafter reads. A
   non-zero exit means the steps file and playbook disagree; run `check_anchors.py` and fix before
   drafting. Then spawn a subagent with the standard drafting prompt: run
   `python3 .claude/skills/generate-gear/render_prompt.py emit-gear <gear>` and pass its printed
   output to the subagent unchanged. It writes `.tmp/<gear>.generated.py`. Add no per-gear
   hints; anything the drafter needs belongs in the step list.
   This drafter takes the `mechanical` role: the stage is transcription against a fixed API,
   and step 4's gates, not the drafter, judge the output. Resolve its model with
   `python3 .claude/skills/generate-gear/pick_model.py --role mechanical --default <the
   session's default model>` and pass the printed name as the Agent tool's `model` option,
   skipping the option where the harness offers none. Never write a model name into this file;
   `.claude/skills/generate-gear/MODELS.md` holds the ladder and the reason the tier is
   relative. The compile-gear drafter takes the `design` role instead, because that stage
   interprets prose and only this one transcribes.

3. **Gate (authoritative owner).** After every draft submission, run the complete battery with
   `python3 .claude/skills/generate-gear/run_gates.py <gear> > .tmp/<gear>.gates.txt` from the repo
   root. Run this command without `--no-advisory`, `--only`, or `--fail-fast`; it runs all seven
   checks below plus the advisory novel-type report and prints one verdict. Read the entire stored
   report, including every gate row, advisory finding, and classification. This is the only
   validation pass for that submitted draft, and the stored copy is what a retry round hands back
   to the drafter. Exit 0 = every gate that ran passed; exit 1 = a gate failed; exit 2 = a setup
   error (missing input, missing stubs, unreachable API database) that no new draft can fix.

4. **Diagnose and loop.** The runner prints a first-pass fault classification; confirm the rows it
   marks NEEDS JUDGMENT against the table below. An emit fault goes back to the drafter, up to
   about three rounds in total. A compile fault, or an exit 2, stops the run.

   An emit fault does not change any input file, so the drafter that produced it still holds the
   step list, the proof and the framework in context. Send it the stored gate report
   `.tmp/<gear>.gates.txt` verbatim with `SendMessage` and let it revise
   `.tmp/<gear>.generated.py`; never paste failure text edited by hand, and do not tell the
   drafter to re-read inputs it has already read. Submit the revised artifact to the complete Gate
   battery again before any placement.

   Spawn a fresh drafting subagent (a full step 2) only when one of these holds: this is the
   first round; an input file (`steps.md`, the proof, the playbook, or a framework module)
   changed since the drafter read it; the same gate fails in two consecutive continued rounds;
   or the drafter is no longer reachable. Resolve a fresh spawn's model the same way step 2
   does, except where drafts have failed the gates with emit faults in two consecutive rounds:
   add `--escalated` to the `pick_model.py` call, which steps the drafter back up to the
   session's default model for the rounds that remain. A continued round changes no model,
   because a resumed agent keeps the one it was spawned on. A fresh retry round re-renders the
   prompt with
   `python3 .claude/skills/generate-gear/render_prompt.py emit-gear <gear> --failure-file
   .tmp/<gear>.gates.txt`, which appends the stored gate report verbatim; hand the printed
   output to the drafter unchanged. The first round's prompt is always the rendered standard
   prompt with no failure file.

5. **Place.** Place only the artifact that passed the most recent complete Gate battery. If the
   artifact or any relevant input changed after validation, return to step 3 and run the complete
   battery again; never substitute an older passing report. On success, run
   `python3 .claude/skills/generate-gear/stage.py <gear> module` from the repo root. It puts
   `.tmp/<gear>.generated.py` at `lib/geargen/<gear>.py` and reports what moved. This writes a
   file only; it does not commit, push, or touch Fusion's add-in directory.

6. **Report.** State the complete gate results, every advisory finding and its triage decision, and
   every step that was thin or wrong. The drafter's report supplies only the artifact facts and
   unresolved step IDs described in the standard drafting prompt.

## Gates

`run_gates.py` runs every row below, in a cost-ordered sequence, and continues past a failure so
one complete submission battery reports every problem. The one exception is a parse failure, which
skips the gates that read the candidate as Python and runs only the anchor check. The commands are
listed so a single gate can be re-run by hand; the runner prints the exact command for each row it
reports. A manual re-run diagnoses a row only; it never replaces the complete submission battery.

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
correct code using it reads as new. Read each finding and decide. Record a finding judged to be
stub noise by re-running the same command with `--accept N --why "<reason>"`, naming it by its
printed index, which writes the `accepted_type_noise.json` entry for you. It is the only check
that has caught a method called on a class that does not define it, or a `BRepBodies` handed to a
parameter typed `ObjectCollection` — both of which passed all seven gates.

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

Hand the printed output to the drafting subagent unchanged. On a retry round, `--failure-file`
appends the previous round's gate report verbatim; the framing text is fixed in the renderer, so
the retry prompt is as standard as the first. The renderer refuses to print anything for an
unknown skill name, a missing template, or a template carrying a placeholder it was not given, so
a garbled render can never reach the subagent.
