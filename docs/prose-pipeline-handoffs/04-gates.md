# T4. Deliver early feedback and an explicit emission handoff

## Assignment

The implementer improves compilation feedback in `fusion360-gear-generator` without treating unfinished
implementation work as accepted output. Submissions A and B are separate changes.
The [common execution contract](README.md#common-execution-contract) applies.

Allowed edits are `run_compile_gates.py`, `check_step_calls.py`, `pipeline_timing.py`, their tests,
`test_render_prompt.py`, `pipeline-timing-pilot.md`, and the compile and emit skills and prompts.
The implementer reads `build_plan`, `execute`, `execute_iteration`, `build_json`,
`finalize_default_metadata`, `overall`, and `compile_policy_for_run` before editing.

## Submission A: Initial gate order

T0 must be complete. Normal initial runs use `compile`, `playbook`, `step_calls`, `proof` order.
`--iteration-base` keeps its existing scope selection. Explicit `--only` runs retain their current
selection semantics; this change does not start unselected prerequisites.

1. The implementer adds an initial-run executor that runs the three inexpensive stages first.
2. A compile or playbook `fail` or `error` omits proof execution, even without `--fail-fast`.
3. A step-call content failure alone does not omit the proof in a normal run.
   `--fail-fast` retains its explicit stop behavior for any failed stage.
4. An omitted proof has `status=skip`, null duration, and the same deterministic omission-reason codes
   already used by iteration validation. The report marks `effective_proof_scope=omitted`.
5. After valid initial structural checks, the proof command is exactly `bash proof/run.sh`.
   A passing selected proof remains insufficient for complete final acceptance.
6. The implementer updates the owning skill, report metadata, and command-order tests together.

| Test name | Setup | Expected result |
|---|---|---|
| `test_initial_compile_failure_omits_proof` | Compile fails; other cheap stages run. | The proof subprocess is never called. |
| `test_initial_playbook_error_omits_proof` | Playbook exits 2. | Overall exit 2; proof omitted. |
| `test_initial_missing_calls_runs_full_proof` | Only step_calls fails. | Full proof runs; overall result remains failed. |
| `test_initial_valid_uses_full_proof` | All cheap stages pass. | The proof command has no package filter. |
| `test_only_proof_stays_selected` | The caller uses `--only proof`. | No unselected checker is launched. |
| `test_iteration_scope_unchanged` | Shared source changed since the retry base. | Existing full-scope expansion still occurs. |
| `test_omitted_proof_cannot_be_complete` | Proof is omitted. | `proof_is_complete` and first-pass eligibility are false. |

Submission A ends after its tests and the common validation pass. It does not add emission readiness.

## Submission B: Emission readiness

T3 and T4A must be complete. This change resolves the case where valid compiled requirements name new
calls that the old generated module does not yet make. It does not turn that compatibility failure into a pass.

The runner adds this JSON object:

```json
{
  "handoff": {
    "ready_for_emit": true,
    "implementation_sync_required": true,
    "missing_call_names": ["addWidget"],
    "reasons": []
  }
}
```

`ready_for_emit` is true only when all of the following are true:

1. The report is an ordinary full run without `--only`, `--fail-fast`, or `--iteration-base`.
2. Compile and playbook stages pass, and the complete proof passes for the current artifacts.
3. The step-call stage passes, is skipped solely because the implementation module does not exist,
   or fails exclusively on missing required reachable calls.
4. Every required report field is present and structurally valid.

For the missing-call case, `parse_error` must be null and both `stubs` and `shared_point` must be empty.
There must be at least one `missing` entry. A setup error, malformed report, or another finding makes readiness false.
`implementation_sync_required` is true for missing calls or an absent implementation module.
`missing_call_names` contains sorted unique names; it is empty when the module is absent.
`reasons` contains sorted failed-condition codes when readiness is false and is otherwise empty.
The codes are `non_full_run`, `compile_not_passed`, `playbook_not_passed`, `proof_incomplete`,
`step_calls_not_ready`, and `step_calls_report_invalid`.

### Mechanical integration

1. The runner invokes `check_step_calls.py` with its existing `--json` option.
2. `StageResult` gains a final optional `details` field defaulting to null. The runner decodes the step-call
   JSON into it, preserves the original stdout, and includes details in the stage JSON.
3. Invalid JSON or an invalid expected field shape produces a stage setup error. The parser must not infer
   readiness from human diagnostic wording. Existing text rendering prints the structured missing-call details.
4. A pure function `emission_handoff(args, results, metadata) -> dict` implements the readiness rules.
   The existing overall verdict, exit code, proof-completeness fields, and first-pass policy remain unchanged.
5. The compile and emit skills allow emission from a true readiness result. The compile skill reports that
   state as ready for emission, not successful final compilation. It does not require an impossible compatibility pass first.
6. After emission passes its ordinary gates and stages the module, the pipeline reruns the ordinary full
   compile gates. Final pipeline acceptance requires both ordinary complete reports to pass on the current artifacts.
7. The timing pilot records pre-emission readiness separately from the final compile acceptance report.
   A previously failed first pass stays failed; a later final acceptance does not rewrite its history.

### Fixture matrix

| Test name | Fixture | Expected result |
|---|---|---|
| `test_ready_full_pass` | Every ordinary stage passes. | Ready; no implementation sync required. |
| `test_ready_missing_calls_only` | Full proof passes; only a required call is missing. | Ready; sync required; overall exit stays 1. |
| `test_ready_no_module` | Full proof passes; step_calls has its existing no-module skip. | Ready; sync required. |
| `test_stub_blocks_readiness` | Missing calls plus a stub marker. | Not ready. |
| `test_shared_point_blocks_readiness` | Missing calls plus shared-point misuse. | Not ready. |
| `test_malformed_details_blocks_readiness` | Invalid JSON or missing expected fields. | Setup error; not ready. |
| `test_selected_proof_blocks_readiness` | All selected iteration checks pass. | Not ready. |
| `test_final_acceptance_requires_resync` | Emission passes but final compile still fails. | The pipeline remains unaccepted. |
| `test_first_pass_not_rewritten` | A later final report passes after a failed round. | The recorded first pass remains false. |

## Validation and completion

```sh
python3 -m unittest discover -s "$P" -p 'test_run_compile_gates.py' > .tmp/t4-runner-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_pipeline_timing.py' > .tmp/t4-timing-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_step_calls.py' > .tmp/t4-call-tests.txt 2>&1
```

The [common validation](README.md#common-validation) follows each submission.
A need to waive a proof, reinterpret a source requirement, or alter final acceptance is an escalation.
The completion record names A or B explicitly and does not claim a measured speedup.
