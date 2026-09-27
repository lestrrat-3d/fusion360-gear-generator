# Tested proof construction examples

The compiler prompt now points to the operation index in
[`proof/examples/OPERATIONS.md`](../../proof/examples/OPERATIONS.md). The index links
executable examples for the analytic cut limit, annular bore profiles, independent
comparison bodies, profile boundaries, and one-cap chamfers. The standard prompt
grew from 17,957 to 18,101 bytes, an increase of 144 bytes. The compiler reads
the index and the relevant example only when an operation needs them.

## End-to-end result

The fresh spur compile used the bore substitute described in the index. Its
placed steps and proof passed all four compile gates with the sketch engine at
`80849197f03e` and decad at `38decf13cb16`. Freshly emitted Python passed all
eight emit gates. The deployed add-in generated a spur gear in Fusion with the
default inputs except for a 4 mm bore. The Fusion-tested Python file has SHA-256
`99567f2d01d4d4572b321a37e1fb0dc00bcffce6c3c5175dff0eeb7a579d3406`.
After that run, the emitted file dropped one overly narrow parameter type annotation
to admit bevel's existing `VirtualSpurProxy` call. The final file has SHA-256
`94d840adbd93db7c034d2e7d29c88182ed4acad369ac1bfe2597225f1dbce202`;
the drawing statements are unchanged.

The bore example checks an annular profile's boundary counts, area, solid
verdict, and volume. Its negative check rejects an incorrect bore radius. The
other examples check separate body ownership, an incorrect comparison volume,
an incorrect chamfer setback, and both sides of the pinned arrangement limit.
The complete proof suite still runs during the final compile gate.

## Measurement limits

The final compile gate reported 0.61 seconds for the proof stage with a warm Go
cache. This is a gate runtime, not the time to accepted Python. The old and new
compiler drafts were not run as a timed pair with the same model and cache state.
The drafting and repair rounds were not timed consistently. Therefore these
runs establish successful generation and tested guidance, but no defensible
change in repair rounds or accepted-output time.

The September 2026 failed trial reported a 214.97-second chamfer test and a
later bore evaluator limit. It used different draft artifacts and cannot serve
as the performance baseline for this change. A future paired trial should time
fresh baseline and candidate drafts from the same spec and pinned engines,
record each repair round, and apply the same complete compile and emit gates.
