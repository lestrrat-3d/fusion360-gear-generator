Round 3 drafts are ready. I corrected circle-seam accounting and replaced unresolved curved pattern/combine geometry
with documented polygon substitutes. The fillet now uses a built-in query checked against the measured root-edge set.
Chamfer is PROSE because the supported selectors cannot express the full required cap set; stepBoreCut checks that
set and bore exclusion. I ran gofmt only; operator gates remain yours.

I resubmitted these drafts in `.worktrees/chore-prose-2026-09-07-proof-examples-aligned-control-2-spurgear`:

- `.tmp/spurgear.steps.md`
- `.tmp/spurgear-proof/sketches_test.go`
- `.tmp/spurgear-proof/solids_test.go`

I corrected circle-seam accounting and documented polygon substitutes for unresolved curved contacts.
The chamfer remains `[PROSE]`; the bore proof checks its edge selection and bore exclusion.

Go formatting passed. The operator must rerun validation.
