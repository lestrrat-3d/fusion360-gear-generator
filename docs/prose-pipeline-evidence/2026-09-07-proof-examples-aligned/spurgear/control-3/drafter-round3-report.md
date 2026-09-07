Round 3 addresses the proof failures without changing emission requirements. The actual-sketch assertion now accounts
for the engine coalescing the root disc to one Circle and splitting a wrapped tooth-root arc at the parameter seam;
it verifies the two real root contacts on that same sketch. Solid tooth extrusion uses documented chord substitution
for unrecordable trimmed free-form boundaries. The completed solid uses longer initial root chords so fillets can
consume the intended radius. Bore proof places the circular hole in the section before extrusion and applies the
disjoint root blends afterward, avoiding the Boolean segment cap while checking the exact removed volume.
Gofmt passed; no operator gates were run.

I revised the profile-count assertions, solid tooth substitution, root-fillet approximation, and bore construction in
`.worktrees/chore-prose-2026-09-07-proof-examples-aligned-control-3-spurgear`.

I submitted:

- `.tmp/spurgear.steps.md`
- `.tmp/spurgear-proof/sketches_test.go`
- `.tmp/spurgear-proof/solids_test.go`

Gofmt passed. The operator must regenerate metadata and rerun validation.
