# Prose-to-code compilation performance study

I measured fresh model drafting in `fusion360-gear-generator`, starting from commit
`f38c26da97f27dd9ec962a65646d19c3cf106cf0`, on September 7, 2026 JST.
The measurements cover interpreting prose, producing steps and Go proofs, producing Python,
and receiving validation feedback. They are not timings of Go compilation alone.

## Measurements

| Independent pilot | First draft | Gate runner | First result |
|---|---:|---:|---|
| Prose specification to steps and Go proof | 1,007.47 s (16m 47s) | 253.16 s (4m 13s) | Failed |
| Existing accepted steps and proof to Python | 401.87 s (6m 42s) | 2.33 s | Failed |

The compiler's second submission took another 504.86 seconds (8m 25s) of drafting.
Its retry runner took 0.49 seconds and omitted proof execution because two compile findings remained:
an out-of-range citation and the incidental word parsed as an API call.
The third submission took 70.66 seconds (1m 11s) to correct those text findings.
Total compiler drafting across three submissions was 1,582.98 seconds (26m 23s), including
575.51 seconds (9m 36s) of repair. None of these drafting intervals is model-only time.

The final retry runner took 15.33 seconds. Structural compilation and playbook extraction passed,
but the selected spur proof and required-call compatibility check failed. Bore cutting exceeded
the evaluator's 4,096-segment limit, and a chamfer case exceeded the allowed volume uncertainty.
The remaining required-call findings concern `project2` and `createInput3` in the proposed steps;
updating their implementation belongs to emission. The faster final gate is a failed, focused
iteration over changed artifacts, not a measured speedup of an equivalent successful build.
Neither independent pilot produced accepted output, and no successful end-to-end latency was established.

The compiler produced 13 steps and two authored proof files. The emitter produced 599 Python lines.
The pilots ran concurrently in separate fresh agents with unchanged standard prompts.
The emitter used the repository's previously accepted steps, not the first pilot's new failed steps.
These rows therefore cannot be added to claim a successful end-to-end build time.

Drafting includes input reading and local tools. Initial reading occupied 71.95 seconds of the
compiler interval and 38.40 seconds of the emitter interval; those times must not be added again.
Isolated model processing time, input/output tokens, billing, and provider scheduling time are unknown.
The role selector returned the session default for both roles; timing metadata calls it `gpt-6`.

The first compiler validation phase took 403.26 seconds including diagnosis and report handling;
the runner itself took 253.16 seconds. The emitter validation phase took 68.28 seconds including
diagnosis; its runner took 2.33 seconds. These research intervals include manual orchestration and
are unsuitable as production throughput estimates. One sample per stage provides no variance estimate.

## Findings and proposed changes

### 1. Make every required output contract visible before drafting

The emitter failed three contract checks. Neither `registerDerivedParameters` nor `_drawFlankToRoot`
appears in its allowed input bundle, although `spec/spurgear/contract.json` requires those names.
The third check requires `addCoincident` inside `buildBore`; the draft places grounding in its
called `drawBore` helper. That remains a failed structural check, even though grounding was not omitted.

Add the contract manifest to compiler inputs, then generate or check a complete contract section
in the steps handed to the emitter. Explicitly distinguish required interfaces from private helper
placement. This addresses failed requirements discovered after 6m 42s of drafting.
The pilot does not quantify the eventual time saving.

Validate this with repository fixtures that remove a required method or move a guarded operation.
Require compilation to reject an incomplete handoff before emission begins.

### 2. Remove formatting work and deliver inexpensive feedback first

All 13 first-draft citations failed the checker's required syntax. The prompt requests a file and
line range without giving the exact accepted form. The standalone compile checker reproduced all
16 compile findings in 0.32 seconds. The ordinary runner nevertheless executed a 252.74-second
proof phase before that check. Citation failures therefore waited about four minutes for feedback.

Generate citation syntax from structured path/range values, as provenance and registrations already
are generated. Carry required calls as explicit data rather than inferring them from prose punctuation:
the incidental word `unitless(` was still an API failure after the first repair, even though the
separate step-call checker accepted its exemption. Copy input IDs and constant tables mechanically
where their source format allows it, leaving the model to interpret construction behavior.
Run inexpensive structural checks before proof execution on the initial draft.
The retry runner already does this. Preserve the complete final proof requirement.
Earlier feedback is supported by this trace; net accepted-build savings remain unmeasured because
running proof first also supplies geometry failures for the same repair round.

Validate exact citation round trips and early-failure ordering in the repository's checker tests.
Test that a final successful report still requires the complete proof suite.

### 3. Give drafting and validation one API-status policy

The drafting prompt treats a missing API database member as nonexistent. `fusion_api.py` instead
marks three such calls as unverified and nonblocking. The compiler queried those calls, reported
source defects, and drafted replacements. Some replacements then disagreed with the existing module.
Database absence establishes neither runtime invalidity nor runtime validity.

Expose a shared query result with verified, unverified, refuted, and unavailable states to both
the drafter and gates. Carry uncertainty into the handoff instead of forcing rediscovery.
Test the existing three watchlist cases, a known member, a rejected member, and database failure.
Keep any Fusion runtime question explicitly unresolved until tested in Fusion.

### 4. Provide typed scaffolding and reusable proof construction guidance

All 28 emitter API failures concerned unknown receiver ownership. The candidate's untyped `cmd`
caused one chain of failures, and its `typing.cast` expressions were not understood by the custom
AST resolver. Pyright reported no blocking errors. Supply typed framework method signatures and
supported receiver conventions before generation, or conservatively extend the resolver.
Test valid receiver inference and continued rejection of an actually wrong receiver class.

The compiler also drafted four failing geometry tests. One chamfer case took 214.97 seconds and
contained overlapping comparison bodies. Provide small, tested harness examples for independent
body comparisons, profile boundaries, and hole construction. These should explain shared mechanics
without giving the agent an existing gear implementation or weakening its geometric assertions.
The final draft's bore failure also shows why the harness bundle should expose evaluator limits
and tested substitute constructions before drafting, rather than discovering them after repairs.

### 5. Tune context and model choice after the requirements agree

The compiler's core inputs plus one referenced sidecar total 221,680 bytes, before extra API and
engine reads. The emitter's prescribed inputs total 177,336 bytes, plus a 4,365-byte helper read.
The compiler encountered truncated combined reads and repeated bounded reads.
Build a deterministic input manifest with bounded sections and hashes before removing any context.
Evaluate a construction-focused proof bundle separately from the full proof and regression tests.

The first emitter gate file also repeats its diagnostics as text and JSON, totaling 28,728 bytes.
Keep machine evidence separately and render one complete diagnostic view for retries.
The configured model ladder contains provider-specific names and did not separate roles in this run.
Make role mapping configurable, then compare accepted results before changing defaults.
Neither byte reduction nor a different model has a measured speedup in this study.

The relevant source locations are `spec/spurgear/contract.json:79`,
`.claude/skills/compile-gear/prompt.md:36`, `.claude/skills/compile-gear/prompt.md:72`,
`.claude/skills/generate-gear/fusion_api.py:96`,
`.claude/skills/generate-gear/run_compile_gates.py:71`, and
`.claude/skills/generate-gear/check_api_calls.py:692`.

## Evaluation plan

Apply contract visibility, citation generation, and shared API policy first, in separate changes.
Run their repository regression tests before repeating model trials. Compare at least three fresh
runs per condition on the same frozen spur inputs, and repeat on a second gear before generalizing.
Feed each successful compilation directly into emission to measure the real serial pipeline.

Record accepted-output latency, first-pass acceptance, drafting rounds, failure categories, gate
time, triage time, and actual usage when exposed. Count failed trials rather than discarding them.
Keep source inputs, model settings, engine revisions, concurrency, and cache state comparable.
Require complete final gates and unchanged geometric coverage. A faster failed draft is not an
improvement in accepted compilation throughput.

## Evidence and workspace

The preserved [compiler evidence](compiler/) contains timestamped [events](compiler/events/),
the first [complete gate JSON](compiler/gates-round-1.json), and the retry gate JSONs for
[round two](compiler/gates-round-2.json) and [round three](compiler/gates-round-3.json).
The [summary](compiler/summary.json) aggregates
the timing evidence, and the [context manifest](compiler/context-manifest.json) records compiler
input sizes and hashes. Earlier generated steps, proof files, and diagnostic scratch output remain
in the read-only source study worktree and are outside this durable evidence allowlist.

The preserved [emitter evidence](emitter/) contains timestamped [events](emitter/events/),
the [complete gate JSON](emitter/gates-round-1.json), the [summary](emitter/summary.json), and the
[triage report](emitter/triage.md). The failed generated Python candidate and its text gate rendering
remain in the read-only source study worktree and are outside this durable evidence allowlist.
The timing summary's `complete: true` means complete timing evidence, not accepted generated code.

The emitter stopped on the hidden compilation contract requirements. The
[emit skill](../../../.claude/skills/emit-gear/SKILL.md) explicitly says,
"A compile fault ends the run." No failed Python output was placed in the generated module directory.
The compiler trial stopped after three submissions with unresolved proof and harness failures.
The [compile skill](../../../.claude/skills/compile-gear/SKILL.md) limits draft retries to
"up to about three rounds in total" and treats failure to build a sound solid after three rounds
as a reason to stop and diagnose the source or procedure. This does not establish that the gear
specification is impossible; the observed final failures include an evaluator limit.
Compiler trial artifacts remain isolated in its worktree. No source specification or pipeline tool
was changed, and nothing was committed, pushed, or deployed. The user has no action item.
