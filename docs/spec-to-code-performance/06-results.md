# Explicit call metadata trial

## Real spur handoff

The compiler produced a version-2 spur step list and proof from the prose spec. The complete
compile battery passed all four stages with a complete proof. A fresh emitter produced the final
Python, and all seven blocking emit gates passed. The user confirmed generation in Fusion with the
deployed file whose SHA-256 is `bdbec37283c822fe9a7b168879e37a28ecdc48735b6cca11d96c80172f992975`.
PR #174 passed all seven remote CI checks, including geometry proofs and the gates for the other
gears.

The step list declares 95 call-shaped spans: 93 required calls and two examples. Source review
classified `normalize` and `dotProduct` as optional method spellings while retaining the required
normalized axial direction and `0.01` tolerance. The source explicitly names `vectorTo`, so the
review retained it as required. The earlier checked-in spur Python lacked that call; the new
emitter supplied it.

The proof records its limits. It checks individual patterned tooth placements, a substituted
bore profile, and a root-disc chamfer. The local solid evaluator cannot prove simultaneous tooth
separation, the completed gear's chamfer, or bore-edge exclusion. Fusion generation is the runtime
check for the resulting add-in.

## Call extraction measurement

A legacy-equivalent file was derived from the same final spur step prose by removing only the
version marker and per-step metadata comments and adding the legacy ignore directive for the two
example names. Both files produced the same 57 required `(name, receiver)` shapes. In 100 warm,
in-process runs of `check_step_calls.named_call_shapes`, the median was 24.466 ms for the legacy
parser and 11.818 ms for version 2. The respective 95th-percentile times were 25.652 ms and
12.469 ms. This measures call extraction only, not compiler or emitter drafting time.

The final version-2 step file is 129,387 bytes. Its metadata comments account for 27,422 bytes;
the derived legacy-equivalent prose file is 101,981 bytes. The checked-in pre-trial step file was
36,365 bytes, but the fresh compiler also expanded the prose, so that file-size difference cannot
be attributed to metadata alone.

No paired same-source baseline and candidate drafting runs were timed with a pinned model.
Accepted-output time and repair-round savings therefore remain unmeasured. The trial supports a
faster call-extraction check and clearer call requirements; it does not establish an end-to-end
generation speedup.
