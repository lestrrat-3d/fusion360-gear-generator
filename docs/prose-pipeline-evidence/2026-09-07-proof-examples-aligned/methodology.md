# Aligned prose compilation study

## Scope

This study compares T7's generic executable proof examples against T6 with the same source alignment in both inputs.
The exact commits, changed paths, requested model, and fixed schedule are in `experiment.json`.
Both conditions already contain T1 through T6, including T4B emission readiness.
This pair cannot measure the benefit of those earlier changes.

[The first study](https://github.com/lestrrat-3d/fusion360-gear-generator/pull/128) stopped after two warm-ups.
It remains blocked and incomplete. This aligned study supersedes its attempted comparison after
[source alignment](https://github.com/lestrrat-3d/fusion360-gear-generator/pull/129).
The studies have distinct trial IDs and condition caches. Their timings are not pooled.

## Execution

Each trial uses a fresh worktree at its condition's exact commit and a fresh design drafter.
The saved standard rendered prompt is delivered by file, without transcription, and its SHA-256 is checked.
Fresh drafters have no inherited conversation and receive no earlier trial findings.
Continued repairs follow the owning compile or emit skill and receive the unchanged complete gate report.
The operator preserves each submission before validation and does not revise generated artifacts.

Trials run serially in the handoff's fixed order. Each condition has its own new Go build cache and temporary directory.
The installed Go toolchain and module caches are shared. Provider prompt-cache state is unknown.
Input-reading intervals are unavailable unless separately observed; drafting includes the drafter's own input reads.
Token counts and cost remain null because the host does not expose them.
Serial wall time includes operator orchestration, review, and waiting between stage boundaries.
It is not an isolated measure of model generation speed. Diagnosis time remains null where no complete interval
was independently observed; subtracting other phases does not supply a measured diagnosis interval.

An ordinary full compile report must explicitly report emission readiness before newly compiled artifacts reach emission.
The compile-output and emit-input digests must match. Final acceptance requires complete passing emit and compile reports
on unchanged final artifacts after normal module placement. A failed first pass remains failed after repair.
Focused iteration reports supply retry feedback and never replace final ordinary full reports.

## Measurement setup failure

The candidate warm-up has an operator timing setup error. Two validation intervals with the same phase and round
were nested. The timing tool pairs phase and round; descriptive event names do not create separate interval identities.
The operator interrupted the second drafting round and retained the first-round proof failure and partial second draft.
The original raw events and summary retain the duplicate-start and unmatched-finish findings.
No reconstructed interval total replaces those invalid benchmark measurements.

After that warm-up, an operator-only helper checks existing raw events before writing a start or finish.
It rejects duplicate starts and unmatched finishes, permits sequential named validation intervals in the same actual
draft round, and pairs the final overall finish with the original round-one overall start.
Synthetic checks exercised each behavior before scored trials began. Frozen condition tooling remains unchanged.
Each subsequent trial records the helper hash and method version in its pre-draft metadata.

The control warm-up stopped before submission or validation, while the candidate warm-up ran a complete proof attempt.
The two condition caches therefore do not have equivalent observed warm-up histories.
The fixed schedule retains both failures and does not restart either trial ID.

## Pause between scored trials

The parent paused the schedule after candidate trial 1 for the user's separate PR120 CI repair request.
The study resumed at candidate trial 2 with the same frozen commits and preserved private condition caches.
The parent reported that the repair used another worktree and the general Go cache, without using either T8 cache.
The installed module and toolchain caches remain shared, so their state may have changed during the pause.
The repair's generated artifacts and source changes were not incorporated into either condition.
No trial interval includes this gap between trials. `pause-and-resume.json` records the checkpoint and resume observation.

## Candidate trial two protocol deviation and pause

The candidate trial-two drafter ran one early four-query `go doc` command from the pinned sketch checkout
without explicitly applying the recorded environment. The observed command duration was 1.398618278 seconds.
Its inherited toolchain and cache settings were not captured, so cache isolation for that command is unverified.
The successful documentation came from the pinned source checkout; no source mismatch was established.
The parent retained this scheduled scored trial and its eventual correctness result, but excluded its wall time
from accepted-latency comparisons. The evidence does not claim full protocol compliance or reconstruct unknown settings.

The parent also paused candidate trial two after its second draft and already-running proof validation finished,
then resumed it for the user's request to continue T8 alongside separate published-stack rebases.
The recorded pause lasted 940.437558 seconds. Raw overall wall time retains this pause without subtraction.
The trial's pause and resume records preserve exact observations; neither event invents a completed trial.
The parallel rebase work excludes frozen T8 worktrees and private condition caches. Shared machine resources and
installed toolchain/module caches may still affect later timings, which limits comparisons during that work.
The parent also confirmed concurrent local checker suites and documentation preparation alongside candidate two
and control two. Shared-host CPU/load isolation is not established. Exact load and other task intervals were not
observed and are not reconstructed. `concurrent-work.json` preserves this limitation; concurrently affected timings
cannot establish an isolated T7 speedup even when private-cache settings are known.

## Coverage and interpretation

Mechanical gate acceptance and the handoff's no-dropped-assertions requirement are separate checks.
Proof scope, substituted geometry, and omitted checks must remain visible even when a gate passes.
The control trial-one drafter explicitly reported fillet and chamfer as `[PROSE]` and bore verification on unfilleted
geometry. These limitations cannot be treated as complete verification of the finished gear.
The comparison must retain those limitations and any changes in assertions across repair rounds.
The second control trial-one draft normalizes circle-seam boundary counts, permits 0.1 percent chorded-area difference,
adds `1e-5 * expected volume` to the volume report bound, and substitutes an assembled outline for boolean union.
Its root-contact adjustment and unfilleted bore checks further limit the geometry established by its passing proof.
The drafter's broad claim that chamfer supports only lateral edges is an unverified, overbroad scope rationale;
the candidate's generic examples demonstrate cap chamfers. That candidate-only knowledge was not sent to the
control drafter. Gate acceptance does not resolve whether a suitable substitute could cover the omitted gear checks.

Candidate trial two's third draft removes its attempted completed-gear chamfer cases and volume assertions.
It also replaces joint pattern-body verification with separate documents for each tooth, leaving the pairwise
partition unproved. It adds root-contact and rotated-centroid checks and strengthens its fillet-volume inequality.
Longer root chords substitute for short embedded facets, so the fillet proof does not establish fitted-spline behavior.
The repaired proof passes the full compile gates, but these changes do not establish preserved assertion coverage.

The candidate trial-two emitter reports the Tools projection/full-constraint conflict, and the parent applies
the owning compile-fault stop consistently with the control warm-up. That was the textual diagnosis at the stop;
actual Fusion behavior remains unconfirmed. Candidate trial three narrows this diagnosis below. Separate gate findings include eight ctx-field omissions despite
annotated assignments with values, two Python `set.add` receiver findings, and three Surface-to-Plane advisories.
Those reports do not establish that the checker is correct about missing initialization or invalid Python calls.
The trial preserves the failures without changing the frozen checker or accepting any advisory waiver.
The parent later independently confirmed the eight ctx-field reports as checker false positives: `_ctx_fields`
handles `Assign` but ignores initialized `AnnAssign`, while all eight fields are assigned `None`.
`posthoc-adjudications.json` records that later finding. The separate checker repair does not change frozen trial
inputs, original results or timings, and this diagnosis was not supplied to subsequent drafters.

Control trial two's third draft replaces curved pattern and combine geometry with endpoint polygons using two flank
samples. Direct outline construction and area checks substitute for the refused face-contact boolean operation.
Its root selector uses concave axial vertices at the root radius instead of cylindrical faces. It removes the actual
chamfer operation and material-removal checks, retaining cap selection and bore exclusion beside the bore proof.
The final bore cuts still fail the pinned engine's arranger-segment cap, so those downstream selection checks are
unreached in the nonzero-bore cases. The three-round stop establishes neither a sound finished solid nor preserved coverage.

Control trial three replaces direct profile-fragment counts with actual contact checks, chords the solid flanks,
and removes early root samples until each initial chord spans at least two fillet radii. Its bore substitute draws
a hole before extrusion and applies root blends afterward, retaining a relative `1e-5` removed-volume tolerance.
The final proof still reports undecided pattern pairs and no valid through-bore section. Chamfer remains `[PROSE]`.
Its withdrawn prefix conflict and inaccurate API argument names remain in source-clarification evidence: explicit
framework naming, omitted driving-dimension arguments, and the required chamfer `False` are executable without conflict.

Candidate trial three withdraws the Tools-only conflict: the reference-only sketch has no attached geometry
to constrain. Earlier trial dispositions and raw records retain their original wording. The remaining source
inconsistency is specific to attached Bore Profile geometry: `instructions.md:604` claims that coincidence
between the movable origin and the projection yields full constraint, while `PLAYBOOK.md:461–462` requires
an already-fixed local point. `fusion.md:26–31` supplies the movable-origin prescription. The parent confirmed
this textual inconsistency and the owning prose-fault stop; no Fusion runtime failure was demonstrated.
The submitted native-operation proof attempt was not validated because the source-fault stop preceded gates.
The prefix claim was withdrawn and the axis-context wording remained unclear rather than incompatible.

Three scored observations per condition cannot support a reliable p95 claim.
Accepted latency is available only for accepted trials, beside acceptance counts and failed-trial times.
No accepted output means accepted latency is unavailable. The helical schedule requires passing spur output;
unreached positions remain unrun and do not receive invented trial records.

## Evidence format

Each actual trial directory contains its record, original timing reports, gate evidence, and a SHA-256 manifest.
Generated trial artifacts stay in the trial worktree's `.tmp/t8/artifacts/`; evidence records their hashes.
Large or whitespace-sensitive raw files use a `.raw.json` wrapper to keep the documentation diff readable.
Concatenate its `chunks` or `lines` without separators and encode as UTF-8 to recover the exact original bytes.
Each wrapper includes the original byte count and SHA-256; the manifest additionally hashes the wrapper itself.
