I stopped before writing artifacts in
`.worktrees/chore-prose-2026-09-07-proof-examples-aligned-control-warm-up-spurgear`.

The source instructions conflict:

- `spec/spurgear/instructions.md:320–323` requires every unlabelled sketch to satisfy `[PB-FULL-CONSTRAINT]`.
- `spec/spurgear/instructions.md:499–501` requires Tools to contain only the projected anchor.
- `.claude/skills/generate-gear/PLAYBOOK.md:455–469` says projected geometry retains free DOF and requires
  additional grounding.

Under the playbook’s stated behavior, the prescribed Tools sketch cannot satisfy its required constraint check.
This is a source contradiction; I have not tested which statement matches Fusion.

`spec/spurgear/fusion.md:204–208` explicitly defines equality as non-embedded with zero-length stubs.
That is a documented degenerate boundary, not an unspecified case or contradictory branch rule.

The prompt checksum matched. No artifacts or log files were written. The sole API lookup reported
`SketchArcs.addByCenterStartEnd` as documented. Operator action is required to resolve the source contradiction
before drafting resumes.
