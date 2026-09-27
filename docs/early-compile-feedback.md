# Early compile feedback measurement

The initial compile gate now runs `compile`, `playbook`, and applicable `step_calls` before
`proof`. A blocking compile or playbook result omits proof and records the cause in the JSON
report. A passing initial draft runs the complete pinned proof before handoff. Step-call
mismatches still run proof because a reviewed mismatch can be an emit handoff.

## Real artifact check

The placed `spec/spurgear/steps.md`, `proof/spurgear/`, and final
`lib/geargen/spurgear.py` passed the complete compile gate and all eight emit gates. The
compile report has `proof_is_complete: true`; the emit report has no skipped gates or advisory
findings. This check reused the placed final Python; it did not run a new model draft.

## Same-source timing pilot

Both runners read the same ten input files with matching SHA-256 digests. Each used a separate
empty Go build cache for its cold run, followed by three warm repeats. The installed Go 1.26.8
toolchain and a writable cache were selected explicitly. Every run passed the complete proof.

| Compile gate duration | Previous runner | Early checks |
|---|---:|---:|
| Cold | 38.35 s | 39.08 s |
| Warm 1 | 0.81 s | 0.81 s |
| Warm 2 | 0.86 s | 0.79 s |
| Warm 3 | 0.79 s | 0.78 s |

One diagnostic trial changed a single provenance hash in the placed step list, then restored
the file. The previous runner spent 39.17 s before reporting the compile failure and passed
the proof. The new runner reported the same compile failure in 0.50 s and marked proof omitted,
with `proof_is_complete: false`. This intentionally stale step list was a test input, not a
model-produced draft.

These are gate times. The pilot did not measure model drafting or time to accepted output.
It therefore supports earlier feedback for this structural failure and no claimed change in
end-to-end generation time. Initial setup attempts failed because the default Go binary was
1.26.1 and its default cache was read-only; neither attempt entered the comparison.
