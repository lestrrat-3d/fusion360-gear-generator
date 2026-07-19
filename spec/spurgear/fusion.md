# Spur Gear — Fusion realization notes

This is the **Fusion-specific** sidecar to `instructions.md`. `instructions.md` carries the design
and geometry *intent* (what to build and why); this file carries the *how* — the Fusion-API
realization that is **specific to the spur (involute) gear family** and not worth promoting to the
shared `PLAYBOOK.md`. The build steps in `instructions.md` cite these rules by anchor (`[SPUR-F…]`).

**Anchor convention** (mirrors the playbook's `[PB-…]`): rules carry stable IDs like
`**[SPUR-F-RIBS]**`. `instructions.md` cites the anchor instead of restating it; the text HERE is
authoritative. IDs are permanent — never renamed or reused once cited. Where a rule is just the
spur application of a cross-gear convention, it cites the `[PB-…]` anchor and adds only the
spur-specific delta (which points, which entities) — no duplication of the shared rule.

These rules are **binding**, exactly like the spec body: every ⚠️ here encodes a known failure
mode. The whole construction falls apart without them, and they're not obvious from the step list.

## Sketch-following-the-anchor design

- **[SPUR-F-ANCHOR-CHAIN] The gear tracks the user's anchor through a chain of sketch
  projections.** A sketch can't reference a `SketchPoint` or curve owned by another sketch, so when
  the Gear Profile or Bore Profile sketches need the user's anchor they call `sketch.project(...)`
  to pull it in locally. The **Tools-sketch projection is the canonical handle**; every later
  sketch projects *that* in again, forming a chain of projections all tied back to the user's
  original anchor entity — so the whole gear moves if the anchor moves later.

- **[SPUR-F-LOCAL-ORIGIN] Each sketch that must follow the anchor keeps its own movable local
  origin.** That is a fresh `SketchPoint` added at (0, 0, 0) — **not** `sketch.originPoint`, which
  is immutable and can't be coincident-constrained to anything brought in from elsewhere. The tooth
  generator draws all its geometry relative to this local origin (the field `self.anchorPoint`),
  then at the very end constrains it coincident with the projected anchor (step 5); Fusion then
  slides the whole sketch onto the user's anchor as a unit.

- **[SPUR-F-SHARED-ADJACENCY] Every adjacency in the tooth profile loop is a *shared*
  `SketchPoint`** — not two free points that happen to share coordinates. Ribs pass through the
  flank splines' `fitPoints[i]`; the tooth-top arc passes through the flanks' `endSketchPoint`s;
  flank-to-root lines end at the flanks' `startSketchPoint`s. Handing Fusion raw `Point3D`s at
  matching coordinates creates *fresh* sketch points, and then the tooth loop is not recognised as
  a closed profile when the extrude step searches for it. This is `[PB-SHARE-XOR-COINCIDENT]`
  applied to the profile loop: pass the existing `SketchPoint` object directly into the creation
  call (share it), never create from `.geometry` and then re-coincident. The spur points anchored
  this way are the four circle centers, the spine start, and the rib chain — all on the local
  origin; piling redundant coincidents onto that shared origin is what makes the solver fail
  (`VCS_SKETCH_SOLVING_FAILED`) or over-constrain.

## Rotation (shared with helical / herringbone / bevel virtual tooth)

- **[SPUR-F-ROTATE-CONFIRM] The requested rotation is drawn AND confirmed — two distinct,
  both-required actions.** When a non-zero `angle` is passed, the tooth geometry is drawn **already
  rotated by `angle`** in the Python point math (step 4 — every flank point, the tooth-top point,
  and the rib-midpoint seeds sit at their `angle`-rotated positions). Then, **as the very last
  action after the entire constraint network exists**, the spine-to-horizontal angular dimension's
  value is set to `angle` (step 7). These are NOT alternatives — do both: the pre-rotation puts the
  geometry on the correct solver branch, and the final dimension value-set *confirms and locks*
  that rotation rather than swinging the tooth into place from +X. (Drawing the tooth flat and
  relying solely on the dimension to swing it lets Fusion pick the wrong ~180°-off branch and ruins
  the helical loft — bottom profile at 0°, top ~180° away → the loft passes through the gear
  centre.) Concretely: `if angle != 0: spineAngularDimension.parameter.value = angle`. For
  `angle = 0` (spur) there is no angular dimension and nothing to set.

## Per-step constraint recipes (the over-constraint-sensitive ones)

These are the spur-specific constraint constructions whose **exact set and order** matter — a
different set or order throws `VCS_SKETCH_OVER_CONSTRAINTS` or `VCS_SKETCH_SOLVING_FAILED`. (They
build on the shared `[PB-FULL-CONSTRAINT]`, `[PB-SHARE-XOR-COINCIDENT]`, `[PB-NO-OVERCONSTRAIN]`,
`[PB-DRIVING-DIM]` rules; here is the spur application.)

- **[SPUR-F-TOOTHTOP-ARC] Tooth-top arc — minimal constraint set (step 6).** Add the *minimal* set
  below and **nothing more** — pinning the arc's centre, or separately coincident-constraining the
  arc endpoints, over-determines the sketch and it blows up later (when the last rib's perpendicular
  is added) with `VCS_SKETCH_OVER_CONSTRAINTS`.
  1. Materialize a **tooth-top point**: a `SketchPoint` at the tip, **rotated by `angle`** to match
     the rotated flanks — `(Tip Circle Radius · cos(angle), Tip Circle Radius · sin(angle))` —
     constrained **coincident to the tip circle** (the *point* lies on the tip circle). This is the
     only coincidence here. Do **not** constrain the arc's `centerSketchPoint` to anything.
  2. Create the arc with `sketchArcs.addByThreePoints(rightFlankEndPoint, toothTopPoint.geometry,
     leftFlankEndPoint)` — pass the two flank splines' **end `SketchPoint`s directly** as first and
     third arguments (so the arc shares those endpoints; no separate coincidences), and the
     tooth-top point's *geometry* as the middle through-point.
  3. Add a single **driving diameter dimension** on the arc equal to the tip circle diameter. The
     two shared flank ends plus this diameter fully determine the arc.

- **[SPUR-F-SPINE] Spine + horizontal reference + angular pin (step 7).** Draw the spine as a
  construction line `addByTwoPoints(localOrigin, toothTopPoint)` — pass **both** existing
  `SketchPoint`s directly (share them). Do **not** create it from `.geometry`, do **not** add a
  separate start-coincident to the origin (sharing already ties it; an extra coincident makes the
  solver fail), and do **not** constrain the spine's end onto the arc (the tooth-top point already
  lies on the tip circle). Because the tooth is drawn at its final `angle` (step 4), the spine
  starts out pointing in the `angle` direction. Pin its absolute rotation:
  - For `angle = 0` (spur), the spine's only additional constraint is **horizontal**.
  - Otherwise add a horizontal reference construction line from the origin **that always points
    +X** — give it a far endpoint at `(Tip Circle Radius, 0)` (a fixed positive x; do *not* derive
    it from the tooth-top point's x, which goes negative once `angle > 90°` and would flip the
    reference). **Then pin that far endpoint: `addCoincident(horizontal.endSketchPoint, tipCircle)`.**
    `addHorizontal` fixes only the line's *direction*, so without this pin the reference line's far
    end is a **free DOF** (its length floats) and the whole tooth sketch is under-constrained
    (`isFullyConstrained` False) even though the geometry is correct. (This loose end appears only
    on the `angle != 0` path — helical / herringbone / bevel virtual tooth.) Add an angular
    dimension measured **from the spine to that horizontal reference, in that argument order**
    (`addAngularDimension(spine, horizontal, …)`); place its text on the **bisector of the intended
    angle** (`(R·cos(angle/2), R·sin(angle/2))` for small R) so Fusion selects `angle`, not its
    supplement. Set its value to `angle` as the very last action (see `[SPUR-F-ROTATE-CONFIRM]`).

- **[SPUR-F-RIBS] Ribs — exact construction order (step 8).** A rib construction line runs between
  each pair of matching left/right flank points — **one per fit-point index `i` for all N indices,
  endpoints included** (the base-circle pair `i=0` and the tip pair `i=N-1` both get a rib, even
  though the tip ends are also joined by the tooth-top arc; the fit-points have no other constraint,
  so an omitted endpoint rib leaves the sketch under-constrained). Each needs a materialized
  **midpoint sketch point** on the spine. Build each rib in **this exact order** — a different order
  over-constrains the sketch (`VCS_SKETCH_OVER_CONSTRAINTS`):
  1. Add the rib with `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])` — pass the
     two fit-point `SketchPoint`s **directly** so the rib shares them; mark it construction.
  2. Dimension the rib's **aligned length** to its current measured value.
  3. Add a fresh `SketchPoint` for the midpoint, created **already on the spine**. The spine is the
     line at `angle` through the local origin, so seed the midpoint at the **foot of the left fit
     point on that line**: with `t = fitX·cos(angle) + fitY·sin(angle)`, the seed is
     `(t·cos(angle), t·sin(angle))`. (For `angle = 0` this reduces to `(fitX, 0)`.) Do **not** seed
     it at the rib's true 2-D midpoint, and do **not** seed it at `(fitX, 0)` for a rotated tooth.
  4. `addCoincident(midpoint, spine)` — pin the point onto the spine **first**.
  5. `addMidPoint(midpoint, rib)` — then make it the rib's midpoint.
  6. `addPerpendicular(spine, rib)` — then make the rib perpendicular to the spine.

  Then dimension the aligned distance from each rib's midpoint to the previous rib's midpoint — and
  **for the first rib, dimension the distance from the local origin to its midpoint** (start the
  chain with `previous = local origin`). Without that origin-to-first dim the whole rib chain has
  one residual DOF (it slides along the spine as a unit) and the sketch never fully constrains. Per
  rib this is exactly determined; any further constraint, wrong order, or off-spine midpoint seed
  over-constrains it.

- **[SPUR-F-FLANK-ROOT] Flank-to-root lines — exactly two constraints, and embedded-case detection
  (step 9).** If the flank's first point (on the base circle) lies **outside** the root circle, draw
  a short radial line from the root circle up to that start point on each side. Build each as
  `addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — pass the flank spline's **start
  `SketchPoint` directly** as the far endpoint (share it; no separate coincident). Then add
  **exactly these two** constraints, no others:
  - (a) the line's **root-end** point coincident to the **root circle**;
  - (b) the **local origin** coincident to the line itself (point-on-line, line treated as infinite)
    — this pins the line to a radial direction.

  Together they exactly constrain the line (root end: 2 DOF → (a) → 1 → (b) → 0). Omitting (b)
  leaves the root end free to slide on the circle (tooth comes out skewed); a third constraint (e.g.
  re-coinciding the shared flank endpoint) over-constrains it. This common case yields a tooth loop
  of **6 curves** (2 splines + 2 flank-to-root lines + 2 arcs). If instead the flank starts
  **inside** the root circle (low tooth-count / pressure-angle combinations drop the base circle
  below the root), no flank-to-root line is drawn and the loop has **4 curves** (2 splines + 2
  arcs) — the profile is "embedded."

  **The embedded test is strict `<`:** with `firstRadius` the distance from the local origin to the
  left flank's first fit point, `embedded = firstRadius < Root Circle Radius` (compare raw values,
  no tolerance). Exact equality therefore counts as **non**-embedded and draws a **zero-length**
  flank-to-root stub (root end and flank start coincide) — this is the ill-conditioned region the
  bench proof flags. Keep the strict comparison; do not "improve" it with `<=` or a tolerance.

  **Embedded-flag mechanism (the tooth generator has no `ctx`):** the tooth generator sets the
  boolean on its **parent generator** — `self.parent._lastToothEmbedded = True/False` — during
  `drawTooth`. `SpurGearGenerator.__init__` MUST pre-initialise `self._lastToothEmbedded = False`
  (alongside `self.toolsSketch = None` and `self.boreSketch = None`). Then `buildSketches` (which
  holds `ctx`) copies it across: `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Do not try
  to set `ctx.toothProfileIsEmbedded` from inside the tooth generator — it cannot reach `ctx`.

## Cleanup

- **[SPUR-F-CLEANUP] End-of-build cleanup recipe.** Hide construction geometry and sketches per
  `[PB-HIDE-AFTER-USE]` (`isLightBulbOn = False` for construction planes/axes; `isVisible = False`
  for sketches — never crossed). The spur-specific recipe: the cleanup turns off the light bulb on
  **every** construction plane/axis it created — the Extrusion End Plane, the `Gear Center` axis,
  and the normalized target plane if one was created in step 1 — and sets `isVisible = False` on the
  Tools, Gear Profile, and Bore Profile sketches, so only the finished gear body shows. **Split by
  entity kind and mode:** the construction-plane/axis hiding **always runs, in both modes**
  (including Generate-Sketches-Only, so no stray plane floats); the **sketch** hiding runs **only on
  the full-build path** (sketch-only mode leaves Tools/Gear Profile visible for inspection — see
  step 6). Guard each entity individually (only hide it if it was actually created — the `Gear
  Center` axis and Bore Profile sketch don't exist in sketch-only mode).

## Numeric snapshots

- **[SPUR-F-SNAPSHOT] Dimensions and feature inputs are numeric snapshots.** Per
  `[PB-NUMERIC-SNAPSHOT]`: every dimension and feature input (extrude offset, chamfer distance,
  fillet radius, pattern count, bore-circle diameter) is set with the *current numeric value* of its
  source parameter at generation time, not a live expression. Editing a `<prefix>_…` user parameter
  does **not** change an existing spur gear — re-run the dialog to regenerate. The parameters stay
  visible in the table for reference only. (This matches the original implementation.)
