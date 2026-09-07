# Spur source clarification

This evidence note clarifies two claims in the drafting report for fusion360-gear-generator.
It does not revise the generated drafts or record an operator-gate result.

## Parameter prefix

- `spec/spurgear/instructions.md` L37 describes registered parameters with the prefix `SpurGear<N>_`.
- `spec/spurgear/instructions.md` L15–16 and L29–33 explicitly select the shared `base.Generator` architecture.
- `.claude/skills/generate-gear/PLAYBOOK.md` L75–85 says this architecture inherits occurrence creation and its
  prefix algorithm without reimplementation. The prefix joins `prefixBase`, an underscore, and the component ID
  with dashes removed.
- `.claude/skills/generate-gear/PLAYBOOK.md` L90–91 appends an underscore and the parameter name, and identifies
  `SpurGear` as the spur override of `prefixBase`.

A literal template reading of L37 would place its unspecified `<N>` immediately after `SpurGear`.
The inherited algorithm puts an underscore after `SpurGear` and then the component identifier.
However, L37 neither defines `<N>` nor requires an ordinal counter or a replacement prefix algorithm.
The text can therefore be resolved as introductory shorthand for a unique instance prefix, with the explicit
inherited algorithm controlling actual names. The earlier report overstated this as a demonstrated behavioral
conflict. It is imprecise introductory notation, not evidence of two fully specified incompatible algorithms.

## Annular body

- `spec/spurgear/instructions.md` L345–351 says the call graph fixes method names and override boundaries.
- `spec/spurgear/instructions.md` L366 labels `buildBody(ctx)` with the comment “extrude the annular body” and
  explicitly points to step 9.
- `spec/spurgear/instructions.md` L572–574 defines that same step 9 as extruding the solid disc inside the root
  circle. L574 explicitly says it is not an annulus and requires the profile selected with `arcs=2`.
- `spec/spurgear/instructions.md` L369 places the optional bore after the main body construction.
  L600–604 specifies the separate optional bore sketch and subsequent cut.

Taken literally as geometry requirements, an annular extrusion would leave an inner hole immediately after
`buildBody`, whereas the disc extrusion leaves material through the center until a later optional bore cut.
Both passages name the same method and step, so they do not describe different construction stages.
The call-graph comment also supplies no inner boundary or hole size from which an annulus could be built.
This is a stale descriptive word in the call-graph summary. The explicit step-9 geometry resolves the intended
behavior as a disc, while the graph still controls the method boundary and operation order.
It is a textual inconsistency, but it does not require an unresolved choice between two complete build recipes.

## Evidence scope

The clarification used the cited prose sources only. No generated draft was changed, no implementation was
consulted, and no operator gate was run.
