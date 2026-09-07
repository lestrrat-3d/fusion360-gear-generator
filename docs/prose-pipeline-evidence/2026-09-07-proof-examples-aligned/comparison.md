# Controlled prose compilation comparison

This study records 8 spur trials and 0 helical trials, with 0 accepted outputs.
The evidence does not establish a provisional T7 improvement.

Accepted serial latency is unavailable because no output passed the complete end-to-end acceptance rule.
The helical prerequisite remains unmet. Its eight positions are unrun and have no trial records.

T8 remains incomplete under the handoff's requirement for eight records per gear.
Failed observations remain in the comparison; missing positions are not manufactured.

The exact historical input commits and schedule are in [experiment.json](experiment.json).
The only condition input difference is T7 generic executable examples.
Both inputs already include T1–T6 and identical source alignment.
The earlier blocked study is separate; its observations are not pooled.

## Actual trials

| Gear | Condition | Trial | Status | Compile drafts | Emit drafts | Compile first pass | Emit first pass | Wall seconds |
|---|---|---|---|---:|---:|---|---|---:|
| spurgear | control | warm-up | failed | 1 | 0 | Unavailable | Unavailable | 181.675953 |
| spurgear | candidate | warm-up | setup_error | 2 | 0 | Fail | Unavailable | 1304.495684 |
| spurgear | control | 1 | failed | 2 | 1 | Fail | Fail | 3561.260176 |
| spurgear | candidate | 1 | failed | 3 | 1 | Fail | Fail | 2521.958243 |
| spurgear | candidate | 2 | failed | 3 | 1 | Fail | Fail | 4615.938531 |
| spurgear | control | 2 | failed | 3 | 0 | Fail | Unavailable | 2064.325155 |
| spurgear | control | 3 | failed | 3 | 0 | Fail | Unavailable | 2215.986373 |
| spurgear | candidate | 3 | failed | 1 | 0 | Unavailable | Unavailable | 1117.660315 |

Warm-ups are unscored. Draft counts include started repair rounds; interrupted drafts remain visible in raw records.
First-pass failures are retained after later successful preparation or gates.

## Scored timings

| Gear | Condition | Trial | Draft seconds | Validation seconds | Gate seconds | Wall seconds |
|---|---|---|---:|---:|---:|---:|
| spurgear | control | 1 | 2735.105489 | 79.678691 | 79.090000 | 3561.260176 |
| spurgear | candidate | 1 | 1756.875498 | 304.129303 | 303.390000 | 2521.958243 |
| spurgear | candidate | 2 | 2635.807306 | 159.969087 | 159.050000 | 4615.938531 |
| spurgear | control | 2 | 1871.601045 | 48.465866 | 48.060000 | 2064.325155 |
| spurgear | control | 3 | 1872.838049 | 78.012132 | 77.580000 | 2215.986373 |
| spurgear | candidate | 3 | 926.021798 | Unavailable | 0.000000 | 1117.660315 |

Gate time is contained within validation time; these columns must not be added together.
Diagnosis intervals, input/output token counts and cost remain unavailable, represented by null in JSON.
Candidate trial two is excluded from accepted-latency comparisons because its early command cache isolation
was unverified. Its raw wall time includes the recorded 940.437558-second parent-directed pause.
Concurrent non-trial work prevents an isolated speedup claim from affected observations.

## Acceptance counts

| Gear | Condition | Scored observations | Accepted outputs | Timing-eligible accepted outputs | Accepted median seconds |
|---|---|---:|---:|---:|---:|
| spurgear | control | 3 | 0 | 0 | Unavailable |
| spurgear | candidate | 3 | 0 | 0 | Unavailable |
| helicalgear | control | 0 | 0 | 0 | Unavailable |
| helicalgear | candidate | 0 | 0 | 0 | Unavailable |

Individual observations and available min/median/max values are in [observations.json](observations.json).
Three scored trials per condition cannot support a reliable p95 estimate.

## Failures and limits

- [spurgear control warm-up](spurgear/control-warm-up/trial.json) records failed. Drafter reported contradictory
  Tools-sketch projection and full-constraint requirements; source concern pending dispatcher review. No artifacts
  submitted.
- [spurgear candidate warm-up](spurgear/candidate-warm-up/trial.json) records setup_error. Operator nested
  validation/diagnosis intervals with the same phase and round; raw timing is invalid. Parent stopped trial and
  interrupted round-2 drafter; retain round-1 proof failure.
- [spurgear control 1](spurgear/control-1/trial.json) records failed. Compile fault: generated steps duplicate 13 source
  anchors and contradict the SketchOnly method boundary; emission stopped after its first complete battery.
- [spurgear candidate 1](spurgear/candidate-1/trial.json) records failed. Compile fault: generated steps duplicate 13
  source anchors; emission stopped after its first complete battery.
- [spurgear candidate 2](spurgear/candidate-2/trial.json) records failed. Compile fault: Tools projection-only sketch
  and full-constraint requirement conflict textually; Fusion behavior unconfirmed. Parent stopped after complete first
  emitter battery.
- [spurgear control 2](spurgear/control-2/trial.json) records failed. Three compile drafts exhausted: bore cut exceeds
  the pinned evaluator arranger segment cap. No ordinary-full readiness or emission.
- [spurgear control 3](spurgear/control-3/trial.json) records failed. Three compile drafts exhausted: pattern partition
  remains undecided and the completed section has no valid through-bore region. No readiness or emission.
- [spurgear candidate 3](spurgear/candidate-3/trial.json) records failed. Reported prose fault: projected-reference
  free-DOF guidance conflicts with Bore Profile grounding by coincidence alone. Submitted artifacts preserved; no
  preparation or gates.

The control warm-up record preserves its initial pending disposition; its dispatcher-disposition.json records
the later textual support for the projection concern and the withdrawn equality-policy claim.
Candidate trial three narrows that earlier textual diagnosis: the reference-only Tools sketch has no attached
geometry to constrain, so the Tools-only claim does not establish a conflict. The earlier trial dispositions
and raw records remain unchanged. The specific remaining inconsistency concerns the Bore Profile: its movable
origin is said to become fully constrained through coincidence with a projection, while the playbook requires
an already-fixed local point. No Fusion runtime failure has been demonstrated.

Proof coverage must be assessed separately from passing gates. [methodology.md](methodology.md) records
removed chamfer assertions, omitted or substituted solid operations, tolerance changes and scope limits.
Each trial preserves repair diffs and coverage notes when its artifacts reached those stages.
The evidence does not establish the handoff's no-dropped-assertions condition.

## Evidence and reproducibility

Every actual trial preserves original reports and events under a raw-evidence-digests.json SHA-256 manifest.
Newly compiled steps and every produced proof file were hashed before emission; matching handoff lists are
present for each trial that reached emission. Unreached final acceptance reports remain null.
Generated trial artifacts remain local; artifact indexes and repair diffs preserve their evidence.
Lossless raw wrappers retain original bytes and hashes as described in [methodology.md](methodology.md).
No source, frozen tool, model, or gate policy was changed to rescue a trial.
