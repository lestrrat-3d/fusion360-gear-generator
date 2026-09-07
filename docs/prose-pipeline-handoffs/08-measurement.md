# T8. Compare complete serial compilations

## Assignment

The operator measures accepted prose-to-code output for `fusion360-gear-generator` after each change.
This handoff fixes the experiment and record format; the existing skills own drafting and retry procedures.
The [common execution contract](README.md#common-execution-contract) applies.

## Required dispatcher inputs

| Input | Meaning |
|---|---|
| `control_commit` | Full commit ID for the unchanged condition |
| `candidate_commit` | Full commit ID containing one completed change |
| `default_model` | Actual model identifier available in the execution host |
| `SKETCH_DIR`, `DECAD_DIR` | Checkouts matching `proof/go.mod` |
| `experiment_id` | A unique directory-safe identifier supplied by the dispatcher |

Missing inputs prevent trial launch. The operator does not pick model names or infer branch tips.
T0 evidence must exist. Serial emission-readiness trials require T4B; earlier conditions use their actual
skill rules and report a blocked handoff as a failed trial instead of bypassing validation.

## Allowed outputs

Trials run in fresh worktrees named `chore-prose-<experiment_id>-<condition>-<trial>`.
The model changes only generated artifacts through the selected skills. Source specifications and tooling
remain fixed within a condition. Raw evidence belongs in each worktree's `.tmp/t8/`.
The final comparison belongs in `docs/prose-pipeline-evidence/<experiment_id>/` in a report worktree.
This task does not deploy, push, or merge artifacts.

## Fixed schedule

The first gear is `spurgear`; the cross-gear check is `helicalgear`.
The operator runs one unscored warm-up per condition, then three scored trials per condition in this order:

| Position | Condition | Trial |
|---|---|---|
| 1 | Control | warm-up |
| 2 | Candidate | warm-up |
| 3 | Control | 1 |
| 4 | Candidate | 1 |
| 5 | Candidate | 2 |
| 6 | Control | 2 |
| 7 | Control | 3 |
| 8 | Candidate | 3 |

No model drafting or validation trial overlaps another. Each trial starts with a fresh drafter and worktree.
Continued repair rounds use the owning skill's rules. A condition has a private Go build cache and temporary
directory shared only by its own warm-up and scored trials. The operator does not erase user caches or claim
that unobservable provider prompt caches are cold. Warm-up failures remain recorded but are not scored.
The complete warm-up and scored schedule is repeated for helicalgear after the combined changes pass on spur.

## Per-trial procedure

1. The operator records the commit, source hashes, prompt/tool hashes, engine revisions, model role and actual
   resolution, environment, and cache locations before drafting. The immutable compile retry base is the trial HEAD.
2. The operator starts a compile timing run and follows the compile skill with the unchanged rendered prompt.
   The operator imports every gate JSON once under its actual draft round and records observed triage.
3. A source fault, setup error, or exhausted retry allowance ends that trial under the skill's stop rules.
   Its status is failed or setup_error, with the reason retained; emission is not invented from old accepted steps.
4. When T4B readiness is true, the operator records SHA-256 digests of the new steps and every produced proof file.
   These exact files become inputs to the emit timing run in the same trial worktree.
5. The operator follows the emit skill. A passed emitter candidate is staged through its normal placement command.
6. The operator runs the ordinary complete compile gates again after implementation synchronization.
   Acceptance requires both complete stage reports to pass for the unchanged final artifact digests.
7. Each final check without another draft gets its own validation event name, not an invented drafting round.
   The final compile report and digests are stored in the trial acceptance record in addition to round imports.
8. The operator closes the timing runs and verifies that wall intervals are not double-counted.

The final verification commands, used only at the matching skill checkpoint, are:

```sh
python3 "$P/run_gates.py" spurgear --json-out .tmp/t8/final-emit.json > .tmp/t8/final-emit.txt 2>&1
python3 "$P/run_compile_gates.py" spurgear --json-out .tmp/t8/final-compile.json > .tmp/t8/final-compile.txt 2>&1
```

For the second gear, the operator substitutes `helicalgear` in both commands.
The timing pilot owns the start/event/import/summarize command syntax and event boundaries.
This handoff does not redefine first-pass acceptance to include a later repaired pass.

## Acceptance record

Each trial writes one `trial.json` with these keys:

```json
{
  "schema": 1,
  "experiment_id": "fixture-study",
  "condition": "candidate",
  "trial": 1,
  "scored": true,
  "gear": "spurgear",
  "commit": "<full commit ID>",
  "status": "accepted",
  "stop_reason": null,
  "compile_run_id": "<recorded ID>",
  "emit_run_id": "<recorded ID>",
  "compile_output_digests": [],
  "emit_input_digests": [],
  "final_artifact_digests": [],
  "final_compile_report": "final-compile.json",
  "final_emit_report": "final-emit.json",
  "serial_wall_time_s": 0,
  "input_tokens": null,
  "output_tokens": null,
  "cost": null
}
```

Digest entries contain `path`, `sha256`, and `bytes`, sorted by path. The two handoff digest lists must match.
Status is `accepted`, `failed`, or `setup_error`. Unreached emit IDs and report paths are null.
The wall time is measured from the first compile start through final acceptance or the actual stop.
The zero and angle-bracket values in the example are placeholders; the operator supplies observations.
Unavailable token counts and cost remain null and are not estimated from byte counts.

## Comparison and decision

The report lists every scored trial, first-pass results, repair rounds, failure categories, draft time,
gate time, diagnosis time, serial wall time, accepted count, and observed usage.
Accepted latency is summarized only for accepted trials, beside the full acceptance count and failed-trial times.
No accepted trials means accepted latency is unavailable. Individual times and min/median/max are sufficient;
three trials do not support a reliable p95 claim.

## Escalation

Missing dispatcher inputs, unavailable requested models, source-hash changes during a trial, or mismatched
handoff digests stop the affected trial as setup_error. The operator preserves its evidence and reports
the mismatch to the dispatcher. The operator does not substitute old outputs, change models, or rerun
under the same trial ID to conceal the invalid attempt. Ordinary draft failures follow the skill's retry
and stop rules and remain failed observations in the comparison.

## Completion

The candidate is a provisional improvement only when it passes complete validation, preserves acceptance count,
and lowers median accepted serial latency in both gears without dropping assertions or gate coverage.
Otherwise the report says insufficient evidence or names the regression for review. The operator does not
choose another model or relax the source to improve the score.

Completion requires the eight scheduled records per gear, raw report hashes, matching handoff digests,
and a comparison that distinguishes observed improvements from unresolved failures.
