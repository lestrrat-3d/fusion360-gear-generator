# Prose-to-code pipeline improvement tasks

## Objective

Reduce the time required for `fusion360-gear-generator` to turn prose specifications into accepted
steps, geometry proofs, and Python implementations. Preserve source meaning, geometric assertions,
contract coverage, and complete final validation.

This file is an implementation backlog. All implementation tasks remain open. The existing compile,
emit, and timing skills own their procedures; this plan proposes changes to those owners where stated.
Creating this task list does not implement the proposed changes.

The [implementation handoffs](prose-pipeline-handoffs/README.md) provide the fixed designs, bounded file edits,
fixture matrices, commands, and escalation conditions for assigning this work to another model.
Their dispatch order and submission boundaries refine this backlog's broader delivery groups.

## Measured starting point

The September 7, 2026 JST study started at commit `f38c26da97f27dd9ec962a65646d19c3cf106cf0`.

| Independent trial | Drafting time | Gate command time | Outcome |
|---|---:|---:|---|
| Spur prose to steps and proof | 26m 23s across three submissions | 4m 29s total | Failed |
| Previously accepted spur steps to Python | 6m 42s for one submission | 2.33s | Failed |

Drafting includes input reading and local tools. Model-only time, token counts, and cost were unavailable.
The trials ran concurrently and did not form a successful serial compilation. Their times cannot be
added to establish accepted end-to-end latency. No proposed improvement has a measured speedup yet.

The original compiler evidence remains under
`.worktrees/chore-prose-compile-study/.tmp/prose-study/` in the project root.
The original emitter evidence remains under
`.worktrees/chore-prose-emit-study/.tmp/prose-study/` in the project root.
The compiler directory contains `report.md`, `compile-summary.json`, and the three gate JSONs under `compile/`.
The emitter directory contains `emit-summary.json`, `triage.md`, and the gate JSON under `emit/`.
These scratch locations are local evidence, not durable repository dependencies; T0 preserves the necessary record.

## Delivery order

Each row is a separate reviewable change. Dependencies describe implementation order, not permission requirements.

| Task | Priority | Deliverable | Dependencies |
|---|---|---|---|
| T0 | First | Durable baseline and experiment manifest | None |
| T1 | High | Complete contract handoff | T0 |
| T2 | High | Generated and validated citations | T0 |
| T3 | High | Explicit required-call metadata | T1, T2, T5 |
| T4 | High | Inexpensive initial checks before proofs | T0 |
| T5 | High | Shared API-status policy | T0 |
| T6 | Next | Typed emitter scaffolding | T1, T5 |
| T7 | Next | Tested proof construction examples | T0 |
| T8 | Required | Controlled serial compilation comparison | Measurements follow each change |
| T9 | Later | Context and model experiments | Accepted baseline from T8 |

T1, T2, T4, and T5 form the first implementation batch. T3 is a separate format change because it
affects existing step lists and multiple checkers. T8 records a comparison after each completed change;
it also evaluates the combined pipeline after T3, T6, and T7.
T4 has a second submission for emission readiness after T3. Its handoff keeps unfinished implementation
compatibility separate from final acceptance so emission can add required new calls without waiving a gate.

## T0. Preserve the baseline and define the experiment

The deliverable is a compact evidence record that survives cleanup of the study worktrees.
The timing procedure remains owned by `.claude/skills/generate-gear/pipeline-timing-pilot.md`.

- [ ] The evidence record preserves input hashes, the starting commit, engine revisions, model-role resolution,
  stage timings, complete failure categories, and references to retained raw reports.
- [ ] The record separates draft wall time, command time, diagnosis time, and total elapsed time.
- [ ] The record identifies the emitter's independent accepted inputs and marks both trials as failed.
- [ ] The experiment manifest records source hashes, prompt and tool revisions, model settings, concurrency,
  cache policy, retry policy, and the stopping condition before each comparison begins.
- [ ] The retained evidence excludes build caches and unrelated generated binaries.

T0 is complete when another developer can identify the measured inputs and reproduce the trial setup
without recovering details from the conversation. Unknown usage data remains explicitly unknown.

## T1. Make every output contract requirement visible

The measured emitter failed method and helper-location requirements absent from its allowed inputs.
The deliverable is a mechanically checked contract handoff from compilation to emission.

The primary files are `spec/<gear>/contract.json`, `.claude/skills/compile-gear/prompt.md`,
`.claude/skills/emit-gear/prompt.md`, and the shared tools `check_contract.py`, `check_compile.py`,
`provenance.py`, and `gen_provenance.py` under `.claude/skills/generate-gear/`.

- [ ] The contract review classifies every rule as an interface requirement, a behavior requirement,
  or an intentional restriction on implementation structure.
- [ ] The compiler receives the relevant contract manifest as an explicit input.
- [ ] A deterministic contract section carries every applicable rule into the emitter's allowed inputs.
- [ ] The compile checker rejects an incomplete or stale contract handoff before emission begins.
- [ ] Provenance includes the contract input, so a manifest change invalidates affected compiled output.
- [ ] Helper-location rules retain their current effect unless repository evidence supports a deliberate rule change.
- [ ] The updated standard prompts and skill procedures reference the contract owner without maintaining manual copies.

Tests belong in `test_check_contract.py`, `test_check_compile.py`, `test_gen_provenance.py`, and
`test_render_prompt.py`. Fixtures must cover an omitted required method, a changed manifest, a scoped
operation moved into a helper, and a gear without an optional contract. Unsupported manifest fields
must produce an explicit diagnostic rather than silently disappearing from the handoff.

T1 is complete when every gated contract requirement is present before drafting and stale handoffs fail locally.

## T2. Generate citation syntax and check source bounds

All 13 first-draft citations failed formatting. A later repair still cited beyond a source file's end.
The deliverable is a deterministic citation renderer backed by validated path and line-range data.

The primary files are `.claude/skills/compile-gear/prompt.md`, `check_compile.py`, `gen_provenance.py`,
and `scaffold_proof.py`. Shared tool names in this file refer to `.claude/skills/generate-gear/`.

- [ ] A versioned metadata format represents each citation's source path and start and end lines.
- [ ] The renderer owns the accepted Markdown spelling and emits a stable result for the same metadata.
- [ ] Validation rejects missing files, reversed ranges, invalid line numbers, and ranges past the file's end.
- [ ] The standard drafting prompt supplies the exact metadata schema and one generic valid example.
- [ ] Existing accepted step lists remain readable during migration, with an explicit compatibility test.
- [ ] Generation preserves provenance headings, step identities, proof annotations, and playbook references.

Tests belong in `test_check_compile.py`, `test_gen_provenance.py`, `test_scaffold_proof.py`, and
`test_render_prompt.py`, plus a renderer test module if a new tool is introduced.
Fixtures must cover single lines, multiple ranges, referenced sidecars, malformed metadata, deterministic
rendering, and existing accepted citation syntax.

T2 is complete when formatting cannot produce a repair round from otherwise valid citation metadata.

## T3. Represent required API calls explicitly

An ordinary parenthetical word became an API failure. Its exemption worked in one checker but not another.
The deliverable is a single call-declaration format consumed consistently by compilation and emission checks.

The primary files are `call_parser.py`, `check_compile.py`, `check_step_calls.py`, and both drafting prompts.

- [ ] Each required-call entry identifies its step, receiver type, member, and any condition on execution.
- [ ] The schema distinguishes required execution, explanatory examples, and inherited framework behavior.
- [ ] Both checkers consume the same parsed representation and exemption semantics.
- [ ] The migration path preserves checks on legacy step lists until their explicit declarations are complete.
- [ ] The new format still detects a required call missing from reachable implementation code.
- [ ] A missing implementation call remains emission work instead of being removed from the specification to pass a gate.

Tests belong in `test_check_compile.py`, `test_check_step_calls.py`, and the shared parser's test coverage.
Fixtures must include prose parentheses, inline examples, conditional calls, inherited calls, a genuine
missing call, and an attempted omission during legacy migration.

T3 is complete when ordinary prose cannot create a required call and required behavior cannot disappear
through a format conversion or exemption mismatch.

## T4. Run inexpensive initial checks before proof execution

The 0.32-second structural check followed a 252.74-second proof phase in the initial trial.
The retry path already performs inexpensive checks first. The deliverable extends that feedback policy
to initial drafting while retaining complete final validation.

The primary files are `run_compile_gates.py`, `pipeline_timing.py`, and
`.claude/skills/compile-gear/SKILL.md`.

- [ ] Initial validation runs structural and playbook checks before starting the geometry proof.
- [ ] A structural failure returns a complete diagnostic report with the proof explicitly marked as omitted.
- [ ] Setup errors retain their distinct status and do not become draft-repair requests.
- [ ] Required-call compatibility findings retain their existing compile-versus-emit classification.
- [ ] A successful final report still requires the ordinary complete proof suite for the current artifacts.
- [ ] Timing records distinguish omitted, selected, expanded, and complete proof execution.
- [ ] Standard workflow text and retry-report handling match the implemented runner policy.

Tests belong in `test_run_compile_gates.py`, `test_pipeline_timing.py`, and `test_render_prompt.py`.
Runner fixtures must verify command order, omission on structural failure, full proof execution on valid
input, scope expansion after shared changes, and refusal to report an iteration pass as final acceptance.

T4 is complete when malformed input receives feedback before proof execution without weakening acceptance.
T8 must measure total repair time too: earlier structural feedback may require a later geometry repair round.

## T5. Share API-status policy between drafting and checking

The prompt treated database absence as nonexistence while the checker classified some calls as unverified.
The deliverable is one status owner and one query result shape used throughout the pipeline.

The primary files are `fusion_api.py`, `check_compile.py`, `check_api_calls.py`, and both drafting prompts.

- [ ] The shared result distinguishes database-backed, unverified, refuted, and unavailable outcomes.
- [ ] The result includes receiver ownership, evidence source, and the applicable diagnostic policy.
- [ ] Drafter-facing lookups and both checkers consume that result without separate status tables.
- [ ] Database absence alone does not become a claim about actual Fusion runtime behavior.
- [ ] Existing unverified calls retain visible advisories and do not become silently approved calls.
- [ ] Fault classification distinguishes malformed draft receivers from confirmed source contradictions.
- [ ] Any unresolved Fusion behavior remains recorded for runtime verification instead of being guessed away.

Tests belong in `test_fusion_api.py`, `test_fusion_api_session.py`, `test_check_compile.py`,
`test_check_api_calls.py`, and `test_render_prompt.py`. Fixtures must cover the existing watchlist,
a known member, a refuted member, an incorrect receiver, an ambiguous lookup, and database failure.

T5 is complete when the same query has the same status in drafting guidance and validator output.

## T6. Provide typed emitter interfaces

All 28 measured emitter API failures concerned unknown receiver ownership.
The deliverable is typed interface scaffolding and a documented set of supported receiver expressions.

The primary files are `check_api_calls.py`, `.claude/skills/emit-gear/prompt.md`, and the relevant
framework declarations under `lib/`. Framework source remains the signature owner.

- [ ] The emitter receives method signatures with explicit parameter and return types where required.
- [ ] Scaffolding derives from framework declarations or is checked against them to prevent signature drift.
- [ ] The supported receiver expressions include documented handling of aliases and casts.
- [ ] Any resolver extension retains separate checks for invalid members and incorrect receiver classes.
- [ ] A cast does not count as proof that the runtime object has the declared type.
- [ ] New diagnostics identify the missing receiver information at its origin instead of obscuring it in a call chain.

Tests belong in `test_check_api_calls.py`, `test_pyright_check.py`, and `test_render_prompt.py`.
Fixtures must cover a typed command input, a valid alias, an unsupported expression, a misleading cast,
and a member invoked on the wrong receiver class.

T6 is complete when supported scaffolding avoids the observed ownership failures while incorrect calls still fail.

## T7. Supply tested proof construction examples

The trial rediscovered body-overlap problems and evaluator limits during repair.
The deliverable is a small set of generic, executable examples tied to the pinned harness behavior.

The primary areas are `proof/proofkit/`, `proof/proofkit3d/`, and `.claude/skills/compile-gear/prompt.md`.
Engine revisions remain owned by `proof/go.mod`.

- [ ] Examples show independent comparison bodies without unintended overlap or interference.
- [ ] Examples explain how profile boundary representations relate to geometric assertions.
- [ ] A bore example verifies material removal and documents a supported substitute when a Boolean operation is limited.
- [ ] Examples expose evaluator limits through tested behavior or an authoritative capability source.
- [ ] Chamfer examples retain meaningful volume and topology checks with their limitations stated explicitly.
- [ ] The compiler can read these examples without access to an existing implementation or previous proof for its gear.
- [ ] Documentation and tests preserve assertion strength instead of raising tolerances merely to make a trial pass.

Validation includes focused harness cases and the complete `proof/run.sh` suite against pinned engines.
T7 is complete when the examples execute successfully and document exactly what their substitutes do and do not prove.

## T8. Measure accepted serial compilation

The deliverable is a comparison report with raw timing records and accepted-output evidence.
The existing timing pilot and compile and emit skills own event boundaries and retry procedures.

- [ ] Each completed change receives at least three fresh spur trials against its immediate parent condition.
- [ ] Trial order and scheduling keep concurrency and cache conditions comparable between conditions.
- [ ] Trials freeze source inputs, model settings, and engine revisions; intentional input changes form a new condition.
- [ ] Each successful compilation feeds its own resulting artifacts directly into emission.
- [ ] Failed compilation prevents that trial from being reported as a successful serial build.
- [ ] Every trial records first-pass acceptance, draft rounds, failure categories, gate time, diagnosis time,
  total elapsed time, and actual token usage and cost when available.
- [ ] Reports preserve failed attempts and explain their stopping conditions instead of averaging only successes.
- [ ] Reports separate time to first feedback from time to accepted output and show individual runs alongside summaries.
- [ ] Final acceptance requires all applicable complete gates for the unchanged artifacts.
- [ ] The combined changes are repeated on a second gear before a cross-gear performance claim is made.

T8 is complete when the report supports a decision to retain, revise, or revert each change.
If a condition has no accepted outputs, its accepted latency is unavailable; it cannot support a speedup claim.
Three trials are an initial comparison, not a basis for precise tail-latency estimates.

## T9. Evaluate context and model changes separately

This task follows an accepted baseline so generation quality can be compared meaningfully.
The primary files are `render_prompt.py`, `extract_playbook.py`, `pipeline_timing.py`, `pick_model.py`,
and `.claude/skills/generate-gear/MODELS.md`.

- [ ] An input manifest enumerates bounded sections and hashes so large reads do not silently truncate requirements.
- [ ] A smaller proof bundle retains every construction requirement and declares any excluded test plumbing.
- [ ] Retry rendering keeps full machine evidence separately and emits one complete diagnostic view for the drafter.
- [ ] Diagnostic-rendering tests preserve every failure, location, advisory, and policy field needed for repair decisions.
- [ ] Role mapping supports the active provider and records the actual resolved model for each stage.
- [ ] Context reduction and model changes run as separate T8 conditions before they are combined.

Tests belong in `test_render_prompt.py`, `test_extract_playbook.py`, `test_run_gates.py`,
`test_pick_model.py`, and `test_pipeline_timing.py`.
T9 is complete only when accepted-output comparisons support the chosen defaults; fewer bytes alone are insufficient.

## Validation and handoff

Implementation work uses isolated worktrees and the repository's applicable pre-read documents.
Existing generated files remain build outputs; fixes belong in their source or generation procedure.
The implementation follows the skill-owned rules for fresh drafting and continued retries.

Focused Python regression tests run after the corresponding implementation settles. The broader checker
suite runs before each implementation handoff using the same command as repository CI.
Test runs use the repository's required Go toolchain and writable build caches:

```sh
python3 -m unittest discover -s .claude/skills/generate-gear -p 'test_*.py' > .tmp/pipeline-tests.txt 2>&1
```

Proof changes also require the complete pinned proof suite. Generated artifacts require the ordinary
complete stage gates; selected tests and timing-record completeness do not establish accepted output.

- [ ] Each completed task records its change reference, validation commands, results, and remaining limitations.
- [ ] Format or input changes include their migration and regeneration requirements.
- [ ] Measurement reports identify whether a benefit is demonstrated, suggested by one trace, or still unmeasured.
- [ ] The final handoff identifies any unresolved source or Fusion-runtime question and its next verification step.
