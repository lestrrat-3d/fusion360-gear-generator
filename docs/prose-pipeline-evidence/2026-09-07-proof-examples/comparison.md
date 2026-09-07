# T8 executable proof example comparison

## Current result

The comparison stopped after the dispatcher confirmed shared source conflicts. T8 remains incomplete.
Neither observed warm-up produced a submitted draft, a gate report,
or accepted output. No scored trial has run. Accepted latency is unavailable, and no performance
improvement or regression can be inferred from these observations.

## Fixed conditions

| Input | Value |
|---|---|
| Control | `667f24f20375a7d12ab1da84af2bb393c97bef0d` |
| Candidate | `f43942d540db978e88c1c1a218ec8d5358296f2a` |
| Isolated change | T7 generic executable proof examples |
| Design model | `gpt-6-astra`, high reasoning |
| Mechanical model | `gpt-6-astra`, resolved by each condition's `pick_model.py`; no emitter launched |
| Sketch engine | `34765bc10360aefd3015133afb0f3bb1586abbba` |
| Decad engine | `f200324e2a14ac657cd190b0e90cad54acb2f4c9` |
| Go toolchain | `go1.26.8` |
| `GOMAXPROCS` | `8` |

Both revisions contain T1–T6 and T4B. This pair does not measure earlier changes independently.
Preflight verified the pinned engines, toolchain, and API database in both observed worktrees.

## Observations

| Position | Gear | Condition | Trial | Status | Serial wall seconds | Completed drafts | Gate reports |
|---|---|---|---|---|---:|---:|---:|
| 1 | spurgear | Control | warm-up | setup_error | 228.390388 | 0 | 0 |
| 2 | spurgear | Candidate | warm-up | failed | 318.401261 | 0 | 0 |

The control warm-up stopped because the operator changed one semicolon to a comma while transferring
the rendered prompt into the agent launch message. The operator immediately interrupted the drafter.
The record retains the failed attempt and its original trial ID. No retry reused that ID.

The candidate warm-up used byte-exact file delivery: the fresh agent read the complete saved rendered
prompt and verified its SHA-256 digest before drafting. The operator verified the saved prompt and
immutable source hashes again after delivery and at stop; no mismatch occurred.

The candidate drafter reported contradictory source instructions and stopped under
`compile-gear/SKILL.md`, “Telling a draft fault from a prose fault.” Its report identified labelled-sketch
constraint checks and the collection type passed to combine. The dispatcher confirmed both conflicts
and stopped the experiment before scored trials. The source files are identical across both conditions.
The drafter initially cited line 516 for labels; the verified label requirement is line 513.
The initial historical profile-count concern was withdrawn. The stricter prompt instruction resolves
the historical ambiguity guidance for this draft.

Exact passages, local API query results, model resolution, immutable input hashes, timing events,
and raw evidence hashes are retained beside each trial record. No generated trial artifact is published.
Two raw text files contain original trailing spaces. Their `.raw.json` wrappers preserve every source byte
as a UTF-8 line array; concatenate `lines` to recover the original file and verify `original_sha256`.

## Methodology limits

- The two observed trials were serial and used fresh agents and worktrees.
- The interrupted control agent stopped before candidate drafting began.
- Each condition has a private build cache and temporary directory preserved for its full schedule.
- The interrupted control warm-up did not establish equal cache conditioning between conditions.
- Shared installed toolchain and module caches were reused; provider prompt-cache state is unknown.
- Serial wall time includes operator handling between stage start and recorded stop.
- The control drafting interval includes prompt transmission and interrupted startup.
- Input-reading boundaries were not timed separately and remain unknown.
- Gate first-pass results are unavailable because no validation ran; diagnosis was not timed separately.
- No repair round, emission handoff, emission timing run, or final acceptance check occurred.
- Input tokens, output tokens, and cost remain null because the host did not expose them.
- The unscored warm-ups cannot support accepted-latency statistics or a p95 claim.

## Remaining schedule

The six scored spurgear positions remain unstarted: control 1, candidate 1, candidate 2,
control 2, control 3, candidate 3. The helicalgear schedule has not started.
T8 completion still requires the prescribed records and complete final acceptance evidence.
The dispatcher will handle source alignment separately, then supply new fixed revisions for a new experiment.
The unstarted positions are not represented by fabricated trial records.

## Proposed source alignment

The dispatcher confirmed these corrections for a separate source-alignment change; this report does not
modify source instructions, playbook rules, engine pins, or acceptance gates.

- Replace the no-exceptions constraint-check instruction in `spec/spurgear/instructions.md:320–322` with
  a requirement for fully constrained geometry. Defer labelled-sketch checks to `[PB-TEXT-HOLDS-DOF]`;
  retain `[PB-FULL-CONSTRAINT]` for sketches without text.
- Replace the collection-as-is instruction at `spec/spurgear/instructions.md:585` with a requirement to
  preserve all `pattern.bodies` members, including the seed, while using the existing `ObjectCollection`
  conversion in `[PB-PATTERN-BODIES]` before `CombineFeatures.createInput`.

The local API database requires `toolBodies: core.ObjectCollection`. No Fusion runtime test was performed.

## Report validation

`python3 -m unittest discover -s .claude/skills/generate-gear -p 'test_*.py'` passed:
843 tests in 28.924 seconds. The full log is `.tmp/handoff-python-tests.txt` in the report worktree.
Report validation used its own cache and temporary directory, separate from the trial condition caches.
The evidence validator also passed. It checked the two observed records, all raw hashes, byte-exact
wrapper recovery, distinct condition caches, and non-overlapping trial intervals.
