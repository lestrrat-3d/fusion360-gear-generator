I stopped drafting in `.worktrees/chore-prose-2026-09-07-proof-examples-candidate-warm-up-spurgear`.
I wrote no step list or proof files.

The conflicting passages are:

- `spec/spurgear/instructions.md:320–322` requires: “Every sketch here is fully constrained … with no exceptions.”
  Line 516 also requires circle labels. `.claude/skills/generate-gear/PLAYBOOK.md:514–522` says a labelled sketch
  never reports fully constrained and instructs: “A gear that labels a sketch cannot gate that sketch on
  `isFullyConstrained`; log the result rather than raising.”
- `spec/spurgear/instructions.md:585` requires: “Feed the pattern's `bodies` collection to the combine as-is.”
  `.claude/skills/generate-gear/PLAYBOOK.md:660–664` requires: “Copy them into an `ObjectCollection` first,”
  because the combine input rejects `BRepBodies`. The local API lookup confirms that
  `CombineFeatures.createInput` requires `core.ObjectCollection`.

The historical proof guidance also differs from the drafting gate. `spec/spurgear/sketch/README.md:77–93`
treats ambiguity as advisory and “true and expected.” The saved prompt at lines 216–221 requires no discrete
ambiguity. The prompt clearly establishes the stricter drafting requirement.

I have not established a profile-count contradiction; my earlier update overstated that finding.

I preserved these API outputs:

- `.tmp/t8/drafter-api-status.jsonl`
- `.tmp/t8/drafter-api-details.txt`

The status queries found three unverified calls: `Sketch.project`, `SketchTexts.createInput2`,
and `FilletFeatureInput.addConstantRadiusEdgeSet`. The remaining 49 queries reported documented members.

The operator must resolve the source conflict disposition before restarting this draft.
