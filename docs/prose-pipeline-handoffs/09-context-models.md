# T9. Run bounded context and model experiments

## Assignment

The implementer prepares three independent experiments for the prose-to-code pipeline in
`fusion360-gear-generator`. Tooling implementation may start before T8 acceptance when the named prerequisite
interfaces below are present. Do not read or change a frozen experiment while implementing the tooling.
An accepted T8 baseline is required before any performance or model trial.
The [common execution contract](README.md#common-execution-contract) applies.
Submissions A, B, and C are separate changes and separate T8 conditions. Before comparison, rebase each trial
branch onto the same accepted baseline and verify identical inputs and runner policy.

| Submission | Implementation prerequisites |
|---|---|
| T9A | Current provenance and prompt interfaces, `formats.md`, T6 emitter interfaces, and T7 proof examples |
| T9B | Current complete runner JSON shapes and T4B emission-readiness reports |
| T9C | Current model resolver and preflight interfaces |

## Submission A: Deterministic input bundles

Allowed new files are `pack_pipeline_inputs.py` and `test_pack_pipeline_inputs.py`.
Allowed existing edits are `test_render_prompt.py`, both prompts and skills, and `pipeline-timing-pilot.md`.
No semantic summarizer or approximate relevance filter is part of this task.

The CLI is:

```text
pack_pipeline_inputs.py <gear> --stage compile|emit --out <new-directory> [--omit-registrations]
```

### Input discovery

Compile input discovery uses `provenance.provenance_inputs(gear)`, every `.go` file beneath
`proof/proofkit/` and `proof/proofkit3d/`, `proof/involute/involute.go`, `proof/examples/CONSTRUCTION.md`
and its executable example file when T7 is present, `docs/prose-pipeline-handoffs/formats.md`, and the
rendered standard compile prompt. T9A changes the compile prompt's `proof/involute/` declaration to the exact
`proof/involute/involute.go` path so the prompt and discovery list agree.
It does not include the gear's old steps, old proof, or generated implementation.

Compile discovery treats `spec/<gear>/fusion.md` and `spec/<gear>/contract.json` as optional. Every other named
compile input is required. `provenance_inputs` returns existing paths only, so the packer separately rejects a
missing instruction file, playbook, format owner, harness input, involute file, or T7 example file. Before packing,
it also checks every Markdown reference matched in the instruction file or optional Fusion sidecar. It uses
`provenance.DOCUMENT_REF` and the same source-relative then repository-relative candidate order as
`provenance.referenced_documents`. After those candidates, a bare `PLAYBOOK.md` resolves only to
`provenance.PLAYBOOK`, which is already a mandatory input. Any other reference with no candidate means exit 2.
Do not change `provenance_inputs` for this task.

The emit prompt's `Read, in this order` block owns the complete emit input list. Emit discovery uses an explicit
list pinned to that block: current steps, every file in the gear proof directory except `stage-manifest.json`,
the extracted playbook, the generated T6 interface sheet, the format owner, the rendered emit prompt, and the
named framework inputs. Expand only Python files beneath framework directories named by that block. Sort every
directory expansion by repository-relative path. Do not scrape arbitrary prose paths.

Tests pin both discovery lists to their owning prompt declarations. The bundle fails when any required input is
missing, unreadable, non-UTF-8, or not a regular file. It does not silently drop a required generated extract,
interface sheet, or empty directory expansion.

### Bundle format

Each source file is packed as UTF-8 text chunks of at most 12,000 bytes, preserving every source byte.
Chunks end at the last complete newline within that bound when possible. A longer line is split at a valid
UTF-8 code-point boundary. Files are ordered by repository-relative path; chunks use `000001.txt` numbering.
The rendered prompt is recorded under logical path `@rendered-prompt` and is not recursively bundled.

`manifest.json` has schema 1 and a `files` array. Each file entry contains `path`, `sha256`, `bytes`, and
`chunks`. Each chunk entry contains `path`, `start_byte`, `end_byte`, and `sha256`.
Byte ranges are half-open. Concatenating a file's chunks must reproduce its exact original bytes and digest.
There are no embedded headers in chunk contents. Empty files have an empty chunks list and the empty-byte digest.

With `--omit-registrations`, only emit mode may omit `proof/<gear>/zz_registrations_test.go`.
The tool first requires `scaffold_proof.py <gear> --check` to succeed. It records the omitted file's hash
and reason separately under `omitted`. It removes no other test functions, helper declarations, or assertions.
A failed scaffolder check prevents omission and exits 1. The flag is invalid in compile mode.

Success exits 0. Missing/unreadable inputs or an existing output directory exit 2.
The tool builds in a temporary sibling directory and removes only its own partial output on failure.

### Bundle consumption

The operator supplies the bundle directory and `manifest.json` to the drafter. The standard rendered prompt is
delivered exactly once from the `@rendered-prompt` entry. For every source path the prompt names, the drafter
resolves that logical path through the manifest and reads all listed chunks in manifest order. It does not reopen
the packed original source path. The prompt and owning skills state this protocol without adding another copy of
the input lists. For a prompt directory input, resolve the directory to sorted manifest file entries beneath its
declared prefix, then read every chunk of each file. The manifest contains file entries, not directory entries.

Documented conditional API-status and signature queries remain available. Save their complete outputs unchanged
under the same policy in every compared condition. An undocumented dynamic input or a query policy that differs
between conditions is an escalation, not a bundle exclusion.

| Test name | Fixture | Expected result |
|---|---|---|
| `test_round_trip_all_bytes` | Multiple Unicode files include a long line. | Concatenated chunks exactly match every input. |
| `test_chunk_bound` | A line exceeds 12,000 bytes. | Every chunk is valid UTF-8 and within the byte bound. |
| `test_deterministic_bundle` | Identical inputs are packed twice into new directories. | Identical manifests and chunk bytes. |
| `test_required_input_missing` | The extracted playbook is absent. | Exit 2; no usable output directory remains. |
| `test_required_input_manifest_agreement` | A mandatory prompt input is absent or becomes optional in discovery. | The consistency test detects the mismatch. |
| `test_auxiliary_reference_resolution` | Prose names bare `PLAYBOOK.md` and one missing auxiliary document. | The playbook resolves canonically; the missing auxiliary reference still exits 2. |
| `test_compile_excludes_old_outputs` | Old steps/proofs/modules exist. | None enters the compile bundle. |
| `test_checked_registration_omission` | Registration output matches the scaffolder. | Only that file is omitted and recorded. |
| `test_modified_registration_not_omitted` | Registration output differs. | Exit 1. |
| `test_prompt_input_manifest_agreement` | A prompt adds or removes a required input. | The input-list consistency test detects drift. |
| `test_bundle_consumption_instructions` | The rendered prompt and manifest are supplied. | Instructions deliver `@rendered-prompt` once and direct every logical source read through ordered chunks. |

```sh
python3 -m unittest discover -s "$P" -p 'test_pack_pipeline_inputs.py' > .tmp/t9a-bundle-tests.txt 2>&1
```

The full bundle and registration-omission variant receive separate T8 comparisons.
If a binary proof-directory file or an undocumented dynamic input is encountered, the implementer escalates
with its path instead of adding an exclusion rule. Completion requires exact byte reconstruction.

## Submission B: One complete retry representation

Allowed new files are `render_retry_feedback.py` and `test_render_retry_feedback.py`.
Allowed existing edits are `test_render_prompt.py`, the compile and emit skills, and `pipeline-timing-pilot.md`.
The current gate runners and their complete JSON output remain unchanged.

The CLI is `render_retry_feedback.py --report <gate.json> --out <feedback.txt>`.
The tool parses the runner's `--json-out` file and writes the entire object using
`json.dumps(report, sort_keys=True, ensure_ascii=False, indent=2) + '\n'`.
No field is filtered, summarized, relabeled, or duplicated in an additional human report.
`schema` must be the integer `1`; boolean `true` is not an integer schema. Exactly one recognized runner list is
required: compile reports contain `stages`, and emit reports contain `gates`. The recognized value must be a list,
and a report containing both recognized keys is rejected as ambiguous. Unknown additional keys are preserved.
Duplicate keys, invalid UTF-8, invalid JSON, a non-object root, or an unsupported schema cause exit 2.
Output replacement is atomic and an error leaves existing output untouched.

The owning skills save each complete runner's raw `--json-out` report unchanged, then render exactly one feedback
file from it. A fresh retry feeds that feedback file to the existing `render_prompt.py --failure-file` path. A
continued drafter receives the same feedback file unchanged. No console rendering or second diagnostic view is
sent to the drafter.

Timing imports the raw `--json-out` report exactly once for the round. It records that raw report and its hash.
The feedback file is not a gate import.

| Test name | Fixture | Expected result |
|---|---|---|
| `test_lossless_report_round_trip` | Both runner report shapes are supplied. | Parsing rendered feedback equals the original object. |
| `test_preserves_unknown_fields` | A report includes additional metadata. | The field and value survive. |
| `test_preserves_nested_diagnostics` | stdout/stderr contain fences, braces, and Unicode. | Every string remains identical after JSON decoding. |
| `test_rejects_duplicate_keys` | A report repeats a key. | Exit 2 with unchanged output. |
| `test_rejects_ambiguous_runner_shape` | A report contains both `stages` and `gates`. | Exit 2 with unchanged output. |
| `test_rejects_boolean_schema` | A report uses `true` as its schema. | Exit 2 with unchanged output. |
| `test_prompt_preserves_feedback` | The file passes through `--failure-file`. | The report text remains unchanged inside its framing. |
| `test_no_double_import` | A workflow records raw evidence and renders feedback. | Exactly one raw gate import exists for the round. |

```sh
python3 -m unittest discover -s "$P" -p 'test_render_retry_feedback.py' > .tmp/t9b-feedback-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_render_prompt.py' > .tmp/t9b-prompt-tests.txt 2>&1
```

Completion requires full JSON round-trip equality. A request to delete noisy diagnostics is an escalation
to a separately designed filtering task; this submission does not decide which evidence is expendable.

## Submission C: Explicit mechanical-model mapping

Allowed edits are `pick_model.py`, `test_pick_model.py`, `MODELS.md`, `preflight.py`, `test_preflight.py`,
and the compile and emit skills. `test_pipeline_timing.py` may gain recording fixtures.
No production model mapping or default-model change is included in this submission.

The resolver adds optional `--mapping <json-path>`, `load_mapping(path) -> dict`, and
`resolve(role, default, escalated=False, mapping=None)`. Append `mapping` after `escalated` to preserve positional
and keyword callers. The argument is the validated complete mapping object.
The mapping format is:

```json
{
  "schema": 1,
  "mechanical": {"fixture-large": "fixture-small"}
}
```

The parser permits only those keys. Model identifiers must be nonempty strings without whitespace.
Duplicate keys, unsupported schema, unreadable files, and wrong types cause exit 2 with empty stdout.
The CLI validates a supplied mapping before resolving any role.

The exact resolution order after validation is:

1. Design and orchestrator roles return the session default.
2. An escalated mechanical role returns the session default.
3. A mechanical mapping for that exact default returns its configured target.
4. An unmapped default uses the current `step_down` and off-ladder fallback behavior unchanged.

Stdout remains one model identifier followed by a newline. Stderr explains which branch resolved it.
Existing `resolve` callers without a mapping keep their current return values and reasons.
Preflight and skills use the same optional mapping path. Preflight adds `--mapping`, calls the shared
`load_mapping` once, stores the returned object in its Context, and passes it to both role resolutions.
Its constructor becomes `Context(root, gear, default_model=None, mapping=None)`. Append `mapping` after
`default_model` so existing Context construction remains valid.
The dispatcher confirms that both model identifiers are available in the execution host before T8 runs.
Timing records the actual launched model; an unavailable target stops the trial as setup_error.

| Test name | Fixture | Expected result |
|---|---|---|
| `test_no_mapping_legacy_results` | Existing resolver cases run. | Exact existing behavior is preserved. |
| `test_mapped_mechanical` | The example map and fixture-large default are supplied. | fixture-small is returned. |
| `test_design_ignores_mapping_target` | The role is design. | fixture-large is returned. |
| `test_escalation_precedes_mapping` | Mechanical escalation is requested. | fixture-large is returned. |
| `test_unmapped_falls_back` | The map lacks the default. | Existing ladder/fallback result. |
| `test_bad_mapping_no_stdout` | A malformed file is supplied for any role. | Exit 2; stdout is empty. |
| `test_preflight_forwards_mapping` | Preflight receives the mapping path. | The shared loader reads it once; both roles receive its result. |
| `test_actual_model_recorded` | A mapped role launches the target. | The timing model field records the target. |

```sh
python3 -m unittest discover -s "$P" -p 'test_pick_model.py' > .tmp/t9c-model-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_preflight.py' > .tmp/t9c-preflight-tests.txt 2>&1
```

The [common validation](README.md#common-validation) follows each submission.
Completion of model support does not choose a deployment default. T8 determines whether a supplied mapping
improves accepted-output performance; unknown usage data remains unknown.
