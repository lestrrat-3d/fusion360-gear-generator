# Pipeline timing pilot

This recipe is opt in. It measures local pipeline boundaries for `spurgear`, `bevelgear`, and
`cycloidal` with the same checked in inputs, model settings, and complete validation policy.
It does not launch an LLM generation run. Use a fresh run directory for each gear and stage.

## Stage variants

Use **direct emit** when `spec/<gear>/steps.md` and `proof/<gear>/` already contain the snapshot
being measured. Include the steps file and every proof file in `start --input`, run the ordinary
emit workflow, and import its complete `run_gates.py` JSON.

Use **compile plus emit** as two linked timing runs, one with `--stage compile` and one with
`--stage emit`, when `/compile-gear` produces the snapshot in this run. Both runs must have
complete full-policy evidence. Report their durations together only after both summaries pass;
this prevents a selective compile report from qualifying the emit run's first pass. The compile
retry path must scaffold and place every returned draft before its iteration gates, as required
by `compile-gear/SKILL.md`.

## One run

Run these commands from the repository root. Replace `<gear>`, `<stage>`, and input paths with the
same values for every comparison. The `start` command records `overall.start` automatically.
For a compile run, omit the steps and proof inputs until compilation has produced them.

```sh
TIMING=.tmp/pipeline-timing/<gear>-<stage>-<run-id>
python3 .claude/skills/generate-gear/pipeline_timing.py start \
  --gear <gear> --stage <stage> --run-dir "$TIMING" --root . \
  --input spec/<gear>/instructions.md \
  --input spec/<gear>/steps.md \
  --input proof/<gear>/<proof-file>.go \
  --input .claude/skills/generate-gear/PLAYBOOK.md \
  --model-role <design-or-mechanical> --model <model-name>

python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase preflight --action start
python3 .claude/skills/generate-gear/preflight.py <gear> --stage <compile-or-emit> --default-model <model-name>
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase preflight --action finish

# Add input_reading start and finish only when the drafting agent's input reads are observed.
# Omit both events when those reads are not observed; the summary then keeps that boundary unknown.

python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase drafting --action start --round 1
# Run the selected compile or emit drafting step here.
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase drafting --action finish --round 1

python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase validation --action start --round 1
python3 .claude/skills/generate-gear/run_gates.py <gear> --json-out "$TIMING/gates.json"
python3 .claude/skills/generate-gear/pipeline_timing.py import-gates \
  --run-dir "$TIMING" --round 1 --file "$TIMING/gates.json"
# Review all advisory findings before recording this complete state.
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase validation --action record --round 1 \
  --event-name advisory-triage --advisory-triage complete
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase validation --action finish --round 1

python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase placement --action start --round 1
# Run the workflow's ordinary placement command here.
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase placement --action finish --round 1
python3 .claude/skills/generate-gear/pipeline_timing.py event \
  --run-dir "$TIMING" --phase overall --action finish --round 1
python3 .claude/skills/generate-gear/pipeline_timing.py summarize \
  --run-dir "$TIMING" --format json
```

For the compile run, use `run_compile_gates.py <gear> --json-out "$TIMING/compile-gates.json"`
and import that report. Then create a fresh timing directory for the emit run after placement.
A selective, fail-fast, no-advisory, or incomplete proof report remains evidence for diagnostics
and cannot qualify as a full first pass. Contract skips qualify only when the runner reports an
actual not-applicable manifest reason.

Repeat the direct-emit run for each pilot gear. Keep retries as separate numbered rounds in one
run, and import every complete report once per round. Compare `drafting_time_s`,
`validation_time_s`, `gate_duration_s`, `total_wall_time_s`, `completed_rounds`, `first_pass`, and
the `advisory_triage` state from the JSON summary. Shared Pyright time is owned by the runner's
timing metadata and is counted once during import.

For a comparison, pin one starting commit and verify identical input digests, model role, model
name, and runner settings. Label one run `cold`, then make three `warm` repeats from the same
snapshot. Keep the input-reading and advisory-triage labels tied to observed work; a missing
observation stays unknown or incomplete in the summary.
