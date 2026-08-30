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

## Chamfer edge count

- **[HELI-F-CHAMFER-COUNT] Helical does NOT override `chamferWantEdges()`; it inherits spur's `6`.**
  The inherited `chamferTooth` (spur `chamferTooth`, spur instructions step 8 — no `[SPUR-F-…]`
  anchor covers it) selects the tooth's front face by `face.edges.count == chamferWantEdges()` plus
  the sketch-plane coplanarity test. Helical's lofted tooth is built from the non-embedded
  **6-curve** profile, so its cap face carries **6** edges, exactly as spur's extruded tooth does.
  Helical overrides `chamferTooth` not at all, adds no "must contain two NURBS flanks" content
  filter, and now adds no count override either.
  **✅ Confirmed in Fusion, 2026-08-30.** This anchor previously carried `4` with an
  asserted/unverified flag. A default helical gear at **chamfer 0.2** raised, aborting the whole
  component through the execute handler's `deleteComponent()`:
  `chamferTooth: no face of the tooth body has 4 edges and is coplanar with the sketch plane
  (faces: 8)`. The `4` was a defect carried from the pre-pipeline module, not a Fusion behaviour.
  **The load-bearing number here is the cap's edge count, `6`, and that alone.** Fusion's own
  total of 8 faces is consistent with a 6-curve section lofted into six walls and two caps, which
  is what corroborated the count at the time; it is a Fusion reading, not a figure any proof
  reproduces, since a faceting harness splits each ruled wall and counts more. Assert the cap
  edge count, never a total face count.
  **What the embedded branch does is unchanged and still raises.** An embedded tooth's section is
  the **4-curve** loop, so its cap carries 4 edges and the inherited count of 6 finds nothing: an
  embedded helical gear at a non-zero chamfer raises exactly as an embedded **spur** gear does, and
  for the same reason. That is spur's documented behaviour, not a helical special case: users
  disable chamfer there. A regen must keep the raise a raise — it must NOT soften it into a skip.
  **What a design-time proof can establish here, and what it cannot.** No harness measures a Fusion
  cap face, so the provable fact is the **sketch loop's curve count**, 6 non-embedded and 4
  embedded; the cap's edge count follows from it only through the one-curve-one-edge
  correspondence, and that correspondence is a Fusion behaviour this anchor asserts rather than a
  proof result. Prove the loop count and say the cap count follows. This anchor names no proof
  case: which cases carry the count is the proof's own business, and a spec that names them makes
  the two artifacts one.
  **Herringbone inherits this and is unverified.** Herringbone subclasses helical and overrides no
  count of its own, so it moves from the old `4` to `6` with this change. Its tooth is a mirrored
  pair and its cap topology has never been measured or loaded, so treat `6` there as inherited
  rather than confirmed until a Fusion session says otherwise.
