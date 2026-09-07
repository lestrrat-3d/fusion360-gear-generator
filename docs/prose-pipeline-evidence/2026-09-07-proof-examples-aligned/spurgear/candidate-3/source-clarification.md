The prefix and axis reports do not establish incompatible executable requirements. One source inconsistency remains
about whether projected points ground attached geometry.

- The prefix wording is imprecise. `spec/spurgear/instructions.md:37` says parameters use the “`SpurGear<N>_` prefix.”
  `PLAYBOOK.md:83–86` gives the inherited implementation precisely:
  `f'{self.prefixBase()}_{component.id without dashes}'`. The specification requires `base.Generator`; it never defines
  an integer counter or requires overriding its prefix generation. The inherited helper therefore settles execution.
  Calling `<N>` explicitly illustrative was my interpretation, not wording the source supplies.
- The Tools-only conflict was overstated, but the projection-grounding inconsistency remains.
  `spec/spurgear/instructions.md:501` says Tools “draws no geometry of its own” and owns one projected reference.
  `spec/spurgear/sketch/README.md:135` reinforces “just an anchor projection — nothing to constrain.”
  `PLAYBOOK.md:455–460` says projected geometry “still carries free DOF” and specifically describes geometry hanging
  from projected points as under-constrained. That does not establish that a reference-only Tools sketch fails.
  However, `PLAYBOOK.md:461–462` requires coincidence to an “already-fixed point,” while
  `spec/spurgear/fusion.md:26–31` specifies a movable local origin grounded only by coincidence to the projection.
  The clearest conflicting claim is `spec/spurgear/instructions.md:604`: after
  `addCoincident(toothGen.anchorPoint, projectedAnchor)`, “the Bore Profile sketch is fully constrained.”
  Those descriptions disagree about the same attached geometry. This is a source inconsistency requiring clarification
  or Fusion evidence, not a demonstrated runtime failure.
- The construction-axis report is an unresolved context description. `spec/spurgear/instructions.md:578` explicitly
  requires `constructionAxes.add(axisInput)`. `PLAYBOOK.md:758–766` says this requires the owning component to be active.
  But `spec/spurgear/instructions.md:484` also says occurrence creation shifts the “active component context” to the new
  occurrence. `PLAYBOOK.md:788–795` prohibits explicitly calling `occurrence.activate()`, within its multi-component
  section, while comparing this behavior to single-gear generators. Avoiding that explicit call does not prove the new
  component is inactive after occurrence creation. The sources leave “active component context” versus “activated
  component” unclear; they do not establish that spur must execute the axis operation in a forbidden state.

I left all submitted artifacts and sources unchanged in
`.worktrees/chore-prose-2026-09-07-proof-examples-aligned-candidate-3-spurgear`. No user action is needed.
