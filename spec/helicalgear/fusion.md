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
  The offset is a **numeric snapshot**: `getParameterAsValueInput` returns
  `ValueInput.createByReal(param.value)` — the `Thickness` value at generation time, not a live
  parameter reference.

  **Visibility — two deliberate facts; a regen must reproduce both, not "clean them up":**
  - **The Twisted Gear Profile sketch stays hidden its whole life.** `createSketchObject` returns a
    hidden sketch, and nothing ever shows it — not `buildSketches`, and not spur's `cleanup` (which
    touches only its own three sketches). The loft's profile-finding works fine on the hidden
    sketch. This is a declared delta from `[PB-HIDE-AFTER-USE]` (there is no "shown then hidden
    after use" phase — it is never shown at all).
  - **The helix `ConstructionPlane` (`ctx.helixPlane`) is left visible after generation.** It is
    never light-bulbed off: spur's `cleanup` hides only the entities spur itself created (Extrusion
    End Plane, normalized plane, `Gear Center` axis), so the offset plane stays lit. Faithful,
    deliberate behavior — a regen must NOT add cleanup for it.

  In **SketchOnly** mode the same holds: the helix plane is still created (and left visible) and the
  twisted sketch is drawn but stays hidden — only the bottom Gear Profile is shown, so the twisted
  profile is not inspectable in sketch-only mode. Faithful to code.

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

## Completed-gear chamfer

- **[HELI-F-CHAMFER-COUNT] The shared `chamferTeeth` no longer uses a tooth-cap edge count.** The
  earlier `4` predicate aborted the feature, and the later `6` predicate chamfered only the bottom
  cap because it required coplanarity with the base sketch plane. Those observations were confirmed
  in Fusion on 2026-08-30.
  The current shared pipeline runs after patterning, root fillets, and the optional bore. It scans
  every planar face of the completed `ctx.gearBody` that is parallel to the Gear Profile plane and
  adds every unique boundary edge once. Root-radius arcs are deliberately included. When a bore is
  present, its two circular cap edges are excluded by the positive bore radius. Helical and
  herringbone inherit this behavior without an override.
  The final completed-gear selection remains pending Fusion verification. The local proof covers
  the final patterned geometry and the bore-edge exclusion, but it does not reproduce Fusion BRep
  topology.
