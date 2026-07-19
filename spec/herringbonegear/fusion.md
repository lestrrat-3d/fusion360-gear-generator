# Herringbone Gear — Fusion realization notes

Herringbone's only Fusion-specific delta over helical is its `buildTooth`: loft one half, then mirror
and combine into a single tooth body before the pattern. Everything else is helical's
(`[HELI-F-…]`) / spur's (`[SPUR-F-…]`).

## Mirror + combine the lofted half

- **[HERR-F-MIRROR-COMBINE] `buildTooth(ctx)` — loft half, mirror across the mid-plane, combine, then
  chamfer.** With `helicalPlaneOffset()` overridden to `Thickness/2`, the inherited `buildSketches`
  has placed `ctx.helixPlane` at mid-body and `loftTooth` builds the bottom-half tooth into
  `ctx.toothBody` named `'Tooth Body'`. `buildTooth` then:
  ```python
  self.loftTooth(ctx)

  # Mirror the lofted half across the mid-body helix plane to form the other half.
  entities = adsk.core.ObjectCollection.create()
  entities.add(ctx.toothBody)
  mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
  mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
  mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'

  # Combine the mirrored half into the original so the pattern/combine steps in the
  # inherited spur pipeline operate on a single body.
  entities = adsk.core.ObjectCollection.create()
  entities.add(mirrorResult.bodies.item(0))
  combineInput = self.getComponent().features.combineFeatures.createInput(
      self.getComponent().bRepBodies.itemByName('Tooth Body'),
      entities,
  )
  self.getComponent().features.combineFeatures.add(combineInput)

  self.chamferTooth(ctx)
  ```
  Key points, exactly as the current code does them:
  - The mirror **target plane is `ctx.helixPlane`** (the mid-body plane), not a fresh plane. Like
    helical's helix plane, this plane is **left visible after generation** — see the visibility
    statement in `[HELI-F-TWIST-PLANE]`; a regen must not add cleanup for it.
  - The combine **leaves `combineInput.operation` at its API default (Join)** — no operation is
    assigned. Deliberate/faithful; do not "fix" it by setting the operation explicitly.
  - The mirrored body is renamed `'Tooth Body (Mirrored)'`; the combine's **target is looked up by
    name** — `self.getComponent().bRepBodies.itemByName('Tooth Body')` — and the mirrored half is the
    tool. After the combine there is a single `'Tooth Body'` spanning the full thickness, which the
    inherited `patternTeeth` then circular-patterns and joins into the gear body.
  - **`chamferTooth(ctx)` is still the last action** (spur's `buildTooth` boundary). Herringbone
    inherits helical's `chamferWantEdges()` of 4 — see `[HELI-F-CHAMFER-COUNT]` for the same flagged
    caveat (chamfer is fragile for the default non-embedded tooth; effectively exercised only at
    chamfer 0).
  - The gear **body** is still extruded across the **full** thickness by the inherited `buildBody`;
    only the tooth is built half-then-mirrored.
