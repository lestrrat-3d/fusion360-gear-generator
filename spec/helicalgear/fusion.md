# Helical Gear — Fusion realization notes

Helical reuses spur's Fusion mechanics for everything except the twisted-profile plane, the loft, and
the chamfer edge count. These are the only helical-specific anchors; the tooth itself is spur's
(`[SPUR-F-…]`). `instructions.md` cites these instead of restating them.

## Twisted top profile

- **[HELI-F-TWIST-PLANE] The twisted profile's plane and sketch (`buildSketches`, after
  `super().buildSketches`).** Create the plane on the gear's own component:
  ```python
  constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
  constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
  plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
  ctx.helixPlane = plane
  loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
  SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
      ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
  ctx.twistedGearProfileSketch = loftSketch
  ```
  The offset is taken from **`self.helicalPlaneOffset()`** (a hook — helical: full `Thickness`;
  herringbone: `Thickness/2`), so herringbone's mirror plane lands mid-body. The twist angle is read as
  a raw `.value` (radians) and passed to the spur tooth generator's `draw(anchorPoint, angle=…)`; the
  generator draws the whole tooth already rotated by it (`[SPUR-F-ROTATE-CONFIRM]` / `[SPUR-F-SPINE]`).
  `createSketchObject` returns a hidden sketch; the inherited spur pipeline handles visibility/cleanup.

## Loft the tooth

- **[HELI-F-LOFT] Loft between the two profiles (`buildTooth` → `loftTooth`).** Find the tooth loop in
  each sketch and loft bottom→top into a new body:
  ```python
  lofts = self.getComponent().features.loftFeatures
  bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
  topToothProfile    = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
  loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
  loftInput.loftSections.add(bottomToothProfile)
  loftInput.loftSections.add(topToothProfile)
  loftResult = lofts.add(loftInput)
  ctx.toothBody = loftResult.bodies.item(0)
  ctx.toothBody.name = 'Tooth Body'
  ```
  Add the bottom section **first**, then the top. Use the framework helper `find_profile_by_curve_counts`
  (do not re-implement the loop search — PLAYBOOK "Shared geargen helper library").
  **⚠️ Non-embedded only.** Both sections pass a fixed `nurbs=2, arcs=2, lines=2` (the non-embedded
  6-curve tooth). This implementation does **not** read `ctx.toothProfileIsEmbedded` and has **no
  embedded branch**: an embedded low-tooth-count helical gear (flank starts inside the root circle,
  `lines=0`) would fail to find the profile. This is faithful to the current code — a documented
  limitation, not a bug to fix in the spec.

## Chamfer edge count

- **[HELI-F-CHAMFER-COUNT] `chamferWantEdges()` returns `4`.** The inherited `chamferTooth`
  (`[SPUR-F]` step 8 / spur `chamferTooth`) selects the tooth's front face by
  `face.edges.count == chamferWantEdges()` plus the sketch-plane coplanarity test, and helical bumps
  the expected count from spur's 6 to **4** — it does **not** override `chamferTooth` itself and adds
  no "must contain two NURBS flanks" content filter (that phrasing in the spur spec is speculative and
  unimplemented).
  **⚠️ Asserted / unverified — reproduce verbatim, do not "fix".** The lofted tooth here is the
  non-embedded **6-curve** profile, so its front cap face would be expected to have **6** edges, not 4.
  A `chamferWantEdges()` of 4 would then make `chamferTooth` fail to find the front face, so helical
  chamfer is likely **fragile/broken for the default non-embedded tooth** and is effectively exercised
  only at the default **chamfer 0** (no chamfer). This value (`4`) is copied straight from
  `lib/geargen/helicalgear.py`; it is flagged here for a future, deliberate correction once the front
  face edge count is confirmed in Fusion — changing it now would be an un-verified behavior change.
